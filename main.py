# main.py
# FINAL HIGH-PERFORMANCE VERSION - MODIFIED FOR CYTRON MDD10A DRIVER & DIRECT FAN CONTROL
# Implements a PID controller with a safety power cap for stable, high-precision control.

import network
import socket
import time
import machine
import onewire
import ds18x20
from machine import Pin, PWM
import uasyncio as asyncio
import gc
import json
import sys

# --- Global Debug Flag ---
DEBUG = False

# --- SAFETY & TUNING PARAMETERS ---
MAX_POWER = 75.0  # Safety Cap: Never allow TEC power above this percentage (e.g., 75%)

# PID GAINS - These require methodical tuning for your specific setup
p_gain = 30.0  # Proportional: The main "power" of the controller
i_gain = 2.0  # Integral: Eliminates the final steady-state error
d_gain = 10.0  # Derivative: The "brakes" to prevent overshoot

# --- WiFi Configuration ---
WIFI_SSID = "Aweg 6-5"
WIFI_PASSWORD = "4190263160"

# --- Hardware & State ---
if DEBUG:
    print("DEBUG MODE IS ON")
    print(f"Firmware: {sys.version}")

if DEBUG:
    print("DEBUG: Initializing hardware pins...")

led = Pin("LED", Pin.OUT)

# --- Pin definitions for Cytron MDD10A Driver ---
tec1_dir = Pin(14, Pin.OUT)
tec1_pwm = PWM(Pin(16))
tec1_pwm.freq(20000)  # Cytron supports up to 20kHz for silent operation

tec2_dir = Pin(17, Pin.OUT)
tec2_pwm = PWM(Pin(19))
tec2_pwm.freq(20000)  # Cytron supports up to 20kHz for silent operation

try:
    ds_pin = Pin(22)
    ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))
    roms = ds_sensor.scan()
    if not roms:
        print("CRITICAL: No DS18B20 sensors found!")
    else:
        print("Found DS devices:", roms)
except Exception as e:
    print(f"CRITICAL: Failed to initialize DS18B20 sensor: {e}")
    roms = []

# --- MODIFIED: Simplified Fan Pin ---
# One pin sends the PWM speed signal directly to the fan's control wire.
fan_speed_pwm_pin = PWM(Pin(20))
fan_speed_pwm_pin.freq(25000)
# --- END MODIFICATION ---

if DEBUG:
    print("DEBUG: Hardware initialization complete.")

system_state = {
    "power_on": False,
    "target_temp": 20.0,
    "current_temp": 0.0,
    "fan_speed_percent": 0,
    "tec_power_percent": 0,
}

integral_error = 0.0
last_error = 0.0
last_time = time.ticks_ms()


# --- Core Logic Function for Cytron MDD10A ---
def set_tec_power(power_percent, mode="cool"):
    """Sets the power and direction for the TECs using the Cytron driver."""
    if DEBUG:
        print(f"DEBUG: set_tec_power called with {power_percent:.1f}% in '{mode}' mode")

    if not system_state["power_on"]:
        power_percent = 0

    system_state["tec_power_percent"] = power_percent
    duty_cycle = int((power_percent / 100) * 65535)

    # Set direction based on mode
    if mode == "heat":
        tec1_dir.high()
        tec2_dir.high()
    else:  # mode == "cool"
        tec1_dir.low()
        tec2_dir.low()

    # Set PWM duty cycle for both TECs
    tec1_pwm.duty_u16(duty_cycle)
    tec2_pwm.duty_u16(duty_cycle)


# --- MODIFIED: Simplified Fan Control Function ---
def set_fan_pwm_speed(percent):
    """Sets the PWM speed signal for the fans."""
    percent = max(0, min(100, percent))
    system_state["fan_speed_percent"] = percent
    duty_cycle = int((percent / 100) * 65535)
    fan_speed_pwm_pin.duty_u16(duty_cycle)


# --- END MODIFICATION ---


# --- PID Controller Loop ---
async def temperature_controller():
    """Main control loop for reading temperature and adjusting TEC/Fan power."""
    global integral_error, last_error, last_time

    while True:
        try:
            current_time = time.ticks_ms()
            dt = time.ticks_diff(current_time, last_time) / 1000.0
            last_time = current_time
            if dt <= 0:
                await asyncio.sleep(1)
                continue

            # Read temperature
            if roms:
                ds_sensor.convert_temp()
                await asyncio.sleep_ms(750)
                system_state["current_temp"] = round(ds_sensor.read_temp(roms[0]), 2)

            if system_state["power_on"]:
                temp_error = system_state["target_temp"] - system_state["current_temp"]

                # Integral Windup Fix: Reset integral when error crosses zero
                if (temp_error > 0 and last_error < 0) or (
                    temp_error < 0 and last_error > 0
                ):
                    if DEBUG:
                        print(
                            f"DEBUG: Error crossed zero. Resetting integral. Old I_err: {integral_error:.2f}"
                        )
                    integral_error = 0.0

                # 1. Integral Term with anti-windup clamping
                integral_error += temp_error * dt
                integral_max = 200 / i_gain if i_gain != 0 else 0
                integral_error = max(-integral_max, min(integral_max, integral_error))

                # 2. Derivative Term
                derivative = (temp_error - last_error) / dt
                last_error = temp_error

                # 3. PID Output Calculation
                p_term = p_gain * temp_error
                i_term = i_gain * integral_error
                d_term = d_gain * derivative
                power = p_term + i_term + d_term

                if DEBUG:
                    print(
                        f"DEBUG: On|Tgt:{system_state['target_temp']}|Curr:{system_state['current_temp']}|Err:{temp_error:.2f}|P:{p_term:.1f}|I:{i_term:.1f}|D:{d_term:.1f}|Out:{power:.1f}"
                    )

                # 4. Apply power based on PID output
                if power > 0:  # Need to HEAT
                    set_fan_pwm_speed(0)  # Turn fans off when heating
                    capped_power = min(power, MAX_POWER)
                    set_tec_power(capped_power, mode="heat")
                else:  # Need to COOL (power is negative)
                    set_fan_pwm_speed(100)  # Set fan to 100% speed when cooling
                    capped_power = min(abs(power), MAX_POWER)
                    set_tec_power(capped_power, mode="cool")
            else:
                # System is off, ensure everything is shut down and reset PID state
                if integral_error != 0 or last_error != 0:
                    if DEBUG:
                        print("DEBUG: System off, resetting PID errors.")
                    integral_error = 0.0
                    last_error = 0.0
                set_tec_power(0)
                set_fan_pwm_speed(0)
        except Exception as e:
            print("ERROR in controller:")
            sys.print_exception(e)

        await asyncio.sleep(1)


async def blink_led():
    """Blinks the onboard LED to indicate the script is running."""
    while True:
        led.toggle()
        await asyncio.sleep(1)


# --- Interactive Terminal UI ---
async def terminal_ui():
    """Provides a simple command-line interface for controlling the system."""
    global integral_error, last_error
    print("\n\n===== PID Cooler Control (Direct Fan) =====")
    print("Type 'help' for commands.")
    reader = asyncio.StreamReader(sys.stdin)
    while True:
        sys.stdout.write("> ")
        res = await reader.readline()
        command = res.decode().strip().lower()
        if not command:
            continue

        parts = command.split()
        cmd = parts[0]

        if cmd == "help":
            print("  on | off | target <temp> | status | quit")
        elif cmd == "on":
            print("System ON.")
            system_state["power_on"] = True
        elif cmd == "off":
            print("System OFF.")
            system_state["power_on"] = False
        elif cmd == "status":
            p_str = "ON" if system_state["power_on"] else "OFF"
            print(
                f"\n--- Status ---\n  State:{p_str}|Curr:{system_state['current_temp']:.2f}C|Tgt:{system_state['target_temp']:.2f}C\n  TEC:{system_state['tec_power_percent']:.0f}%|Fan:{system_state['fan_speed_percent']:.0f}%\n  I_err:{integral_error:.2f}|Last_err:{last_error:.2f}\n--------------\n"
            )
        elif cmd == "target":
            try:
                system_state["target_temp"] = float(parts[1])
                # Reset PID errors to respond quickly to the new target
                integral_error = 0.0
                last_error = 0.0
                print(
                    f"Target set to {system_state['target_temp']:.2f}C. PID errors reset."
                )
            except (ValueError, IndexError):
                print("Usage: target <temperature>")
        elif cmd in ["quit", "exit"]:
            print("Exiting...")
            break
        else:
            print(f"Unknown command: '{command}'. Type 'help'.")

        await asyncio.sleep(0.1)


async def main():
    """Main function to initialize WiFi and start all tasks."""
    print("Setting up WiFi...")
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)

    max_wait = 15
    while max_wait > 0 and wlan.status() < 3:
        max_wait -= 1
        print("waiting for connection...")
        await asyncio.sleep(1)

    if wlan.status() == 3:
        print(f"Connected! IP = {wlan.ifconfig()[0]}")
    else:
        print("Could not connect to WiFi.")

    print("Starting background tasks...")
    asyncio.create_task(blink_led())
    asyncio.create_task(temperature_controller())

    # Start the user interface
    await terminal_ui()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    except Exception as e:
        print("\n\nFATAL ERROR")
        sys.print_exception(e)
    finally:
        print("Performing clean shutdown...")
        machine.reset()
