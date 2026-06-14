/*
  ESP32 TEC Bed-Cooler Controller — NETWORK CONTROL VERSION (no dial input)

  HARDWARE REVISION (current sensors removed):
  - 4x TEC via Cytron MD13S (PWM + DIR)              -> unchanged
  - ACS712 current sensors                            -> REMOVED
  - Coolant temperature probe (2-wire NTC thermistor) -> NEW, on the pump block.
       This is the regulated "water" temperature used by the PID loop.
  - 2x DS18B20 (OneWire)                              -> glued to the TEC heatsinks,
       for monitoring + heatsink over-temperature protection.
  - PWM D5 pump (4 lead: 12V red/black power, PWM blue, tach green)
       Pump runs ONLY when the water-level sensor reports the loop is full.
  - XKC-Y26 non-contact water-level sensor            -> NEW (submersion interlock)
  - ST7789 TFT display, can be fully blanked from the settings menu (dark room).
*/

#include <Arduino.h>
#include <math.h>
#include <Preferences.h>

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino_GFX_Library.h>

#ifndef BLACK
#define BLACK 0x0000
#endif
#ifndef WHITE
#define WHITE 0xFFFF
#endif

// ======================= USER HARDWARE CONFIG =======================

static const int NUM_TEC = 4;
static const int NUM_HEATSINK = 2; // DS18B20 sensors glued to the heatsinks

// Cytron MD13S control pins (PWM + DIR)
static const int TEC_PWM_PINS[NUM_TEC] = {25, 27, 32, 16};
static const int TEC_DIR_PINS[NUM_TEC] = {26, 14, 33, 17};

// DS18B20 OneWire bus (now carries the 2 heatsink sensors)
static const int ONE_WIRE_PIN = 13;

// Coolant temperature probe: 2-wire NTC thermistor on the pump block.
// Wiring of the divider:  3.3V --- NTC --- [ADC node] --- R_FIXED --- GND
static const int NTC_ADC_PIN = 34; // input-only ADC1 pin (was an ACS712 pin)

// PWM D5 pump
static const int PUMP_PWM_PIN = 19;  // -> pump BLUE wire (PWM control input)
static const int PUMP_TACH_PIN = 22; // <- pump GREEN wire (tach / speed output)

// XKC-Y26 water-level sensor (YELLOW signal wire)
static const int WATER_LEVEL_PIN = 21;
// XKC-Y26 OUT is HIGH when liquid is detected (black MODE wire left floating =
// normally-open / positive output). Set false if you short MODE->GND (inverted).
static const bool WATER_LEVEL_ACTIVE_HIGH = true;

// TFT SPI pins
static const int TFT_SCK = 18;
static const int TFT_MOSI = 23;
static const int TFT_DC = 4;

// TFT backlight control (drives the BLK/BL pin). Lets us fully blank the panel
// for a dark room. Set to GFX_NOT_DEFINED if BL is hard-wired to 3V3.
static const int TFT_BL = 15;

// If CS is tied to GND, keep GFX_NOT_DEFINED.
static const int TFT_CS = GFX_NOT_DEFINED;
// If you can wire RST to a GPIO, set it here. Otherwise keep GFX_NOT_DEFINED.
static const int TFT_RST = GFX_NOT_DEFINED;

// ST7789 170x320 + offsets
static const int TFT_NATIVE_W = 170;
static const int TFT_NATIVE_H = 320;
static const int TFT_COL_OFFSET_1 = 35;
static const int TFT_ROW_OFFSET_1 = 0;
static const int TFT_COL_OFFSET_2 = 35;
static const int TFT_ROW_OFFSET_2 = 0;

// Landscape orientation (should end up w=320 h=170)
static const int TFT_ROTATION = 1;

// IMPORTANT: lower SPI speed for stability
static const uint32_t TFT_SPI_SPEED = 8000000; // 8 MHz

// ======================= NTC CALIBRATION =======================
// Standard 10k NTC, beta 3950, with a 10k fixed resistor, supplied from 3.3V.
static const float NTC_SUPPLY_MV = 3300.0f;
static const float NTC_R_FIXED = 10000.0f; // fixed divider resistor (ohms)
static const float NTC_R0 = 10000.0f;      // NTC resistance at 25C
static const float NTC_T0_K = 298.15f;     // 25C in Kelvin
static const float NTC_BETA = 3950.0f;

// ======================= PUMP / TACH =======================
static const uint32_t PUMP_PWM_FREQ_HZ = 25000; // PC/D5 standard 25 kHz
static const uint8_t PUMP_PWM_RES_BITS = 8;
static const int PUMP_LEDC_CH = 4; // TECs use channels 0..3
static const float PUMP_TACH_PULSES_PER_REV = 2.0f;
static const float DEFAULT_PUMP_SPEED_PERCENT = 70.0f;

// ======================= CONTROL LIMITS =======================

static const float DEFAULT_MAX_POWER_PERCENT = 75.0f;

// Heatsink over-temperature protection (replaces the old overcurrent trip)
static const float HEATSINK_MAX_C = 70.0f;
static const float HEATSINK_CLEAR_C = 60.0f; // hysteresis for auto-clear

// PID defaults
static const float DEFAULT_KP = 30.0f;
static const float DEFAULT_KI = 2.0f;
static const float DEFAULT_KD = 10.0f;

// PWM (stable on ESP32): 20kHz @ 11-bit
static const uint32_t TEC_PWM_FREQ_HZ = 20000;
static const uint8_t TEC_PWM_RES_BITS = 11;

// Base direction level for COOL (can be inverted per channel)
static const bool COOL_DIR_LEVEL_HIGH = false;

// ======================= NETWORK SETTINGS =======================

static const char *AP_SSID = "TEC-CTRL-SETUP";
static const byte DNS_PORT = 53;
static const char *MDNS_HOST = "tec-ctrl"; // http://tec-ctrl.local/

// ======================= POLARITY DEFAULTS =======================
static const bool TEC_DIR_INVERT_DEFAULT[NUM_TEC] = {false, true, false, true};

// ======================= GLOBALS =======================

// Display
Arduino_DataBus *bus = new Arduino_ESP32SPI(
    TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, GFX_NOT_DEFINED);

Arduino_GFX *gfx = new Arduino_ST7789(
    bus,
    TFT_RST,
    TFT_ROTATION,
    true,
    TFT_NATIVE_W,
    TFT_NATIVE_H,
    TFT_COL_OFFSET_1, TFT_ROW_OFFSET_1,
    TFT_COL_OFFSET_2, TFT_ROW_OFFSET_2);

// DS18B20 (heatsinks)
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature ds18b20(&oneWire);
static DeviceAddress hsAddr[NUM_HEATSINK];
static int hsCount = 0;
static float heatsinkTempC[NUM_HEATSINK] = {NAN, NAN};

// Non-blocking heatsink temp conversion
static bool tempConvInFlight = false;
static uint32_t tempConvStartMs = 0;
static uint32_t lastTempKickMs = 0;
static const uint16_t tempConvDelayMs = 200; // 10-bit ~187ms

// Coolant (NTC) temperature
static float waterTempC = NAN; // regulated coolant temp from the pump-block NTC

// Water level
static bool waterLevelOK = false;
static bool waterLevelRawLast = false;
static uint32_t waterLevelChangeMs = 0;

// Pump
static float pumpDutyApplied = 0.0f;
static volatile uint32_t pumpTachCount = 0;
static uint32_t lastTachSampleMs = 0;
static float pumpRpm = 0.0f;

// Preferences
Preferences prefs;

// Web
WebServer server(80);
DNSServer dnsServer;
static bool apMode = false;

static String staSSID = "";
static String staPASS = "";
static String staIP = "";

// Reboot scheduling
static bool rebootPending = false;
static uint32_t rebootAtMs = 0;

// System state
static bool systemOn = false;
static bool profileMode = false;
static float targetConstC = 20.0f;
static float profileC[24];

static float maxPowerPercent = DEFAULT_MAX_POWER_PERCENT;

// Pump + display settings (persisted)
static bool pumpEnabled = true;
static float pumpSpeedPercent = DEFAULT_PUMP_SPEED_PERCENT;
static bool displayOn = true;

static float kp = DEFAULT_KP;
static float ki = DEFAULT_KI;
static float kd = DEFAULT_KD;

static float targetTempC = NAN;

static float integralErr = 0.0f;
static float lastErr = 0.0f;
static uint32_t lastPidMs = 0;
static uint32_t profileStartMs = 0;

// Direction invert per TEC (runtime)
static bool dirInvert[NUM_TEC] = {false, false, false, false};

static float tecDutyApplied[NUM_TEC] = {0};
static uint32_t lastPowerChangeMs = 0;

// Fault
static bool faultTripped = false;
static String faultMsg;
static uint8_t overTempCount = 0;

// Manual test
static bool manualActive = false;
static uint32_t manualEndMs = 0;
static int manualChan = 0;
static bool manualHeat = false;
static float manualDuty = 0.0f;

// Display caching
struct DispCache
{
  String waterLine;
  String targetLine;
  String levelLine;
  String hsLine;
  String ssidLine;
  String ipLine;
  bool fault = false;
  String faultLine;
};
static DispCache dispLast;
static bool dispStaticDrawn = false;
static bool dispDirty = true;
static bool dispBlankedDrawn = false; // true once we've painted the OFF (black) state

// Layout (landscape 320x170)
static int W = 0, H = 0;
static const int M = 8;
static int waterValX, waterValY, waterValW, waterValH;
static int targetValX, targetValY, targetValW, targetValH;
static int levelX, levelY, levelW, levelH;
static int hsX, hsY, hsW, hsH;
static int ssidX, ssidY, ssidW, ssidH;
static int ipX, ipY, ipW, ipH;
static int faultY, faultH;

// ======================= HELPERS =======================

static inline float clampf(float v, float lo, float hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
  return gfx->color565(r, g, b);
}

static String htmlEscape(const String &s)
{
  String o;
  o.reserve(s.length() + 16);
  for (size_t i = 0; i < s.length(); i++)
  {
    char c = s[i];
    if (c == '&')
      o += "&amp;";
    else if (c == '<')
      o += "&lt;";
    else if (c == '>')
      o += "&gt;";
    else if (c == '"')
      o += "&quot;";
    else
      o += c;
  }
  return o;
}

static void scheduleReboot(uint32_t msFromNow = 800)
{
  rebootPending = true;
  rebootAtMs = millis() + msFromNow;
}

static void resetPid()
{
  integralErr = 0.0f;
  lastErr = 0.0f;
  lastPidMs = millis();
}

static void allTecOff();
static void setPump(float duty01);

// ======================= POLARITY SNIPPET =======================

static String invertSnippet()
{
  String s = "static const bool TEC_DIR_INVERT_DEFAULT[4] = {";
  for (int i = 0; i < NUM_TEC; i++)
  {
    if (i)
      s += ", ";
    s += (dirInvert[i] ? "true" : "false");
  }
  s += "};";
  return s;
}

static void printInvertSnippetToSerial()
{
  Serial.println("Paste into code to make polarity permanent:");
  Serial.println(invertSnippet());
}

// ======================= SETTINGS (NVS) =======================

static void loadSettings()
{
  prefs.begin("tecctrl", false);

  systemOn = prefs.getBool("on", false);
  profileMode = prefs.getBool("mode", false);
  targetConstC = prefs.getFloat("tconst", 20.0f);
  maxPowerPercent = prefs.getFloat("maxpwr", DEFAULT_MAX_POWER_PERCENT);

  pumpEnabled = prefs.getBool("pumpen", true);
  pumpSpeedPercent = prefs.getFloat("pumpspd", DEFAULT_PUMP_SPEED_PERCENT);
  displayOn = prefs.getBool("dispon", true);

  for (int i = 0; i < NUM_TEC; i++)
    dirInvert[i] = TEC_DIR_INVERT_DEFAULT[i];
  for (int i = 0; i < NUM_TEC; i++)
  {
    String k = "inv" + String(i);
    if (prefs.isKey(k.c_str()))
      dirInvert[i] = prefs.getBool(k.c_str(), dirInvert[i]);
  }

  size_t n = prefs.getBytesLength("prof");
  if (n == sizeof(profileC))
  {
    prefs.getBytes("prof", profileC, sizeof(profileC));
  }
  else
  {
    for (int i = 0; i < 24; i++)
      profileC[i] = targetConstC;
    prefs.putBytes("prof", profileC, sizeof(profileC));
  }

  kp = prefs.getFloat("kp", DEFAULT_KP);
  ki = prefs.getFloat("ki", DEFAULT_KI);
  kd = prefs.getFloat("kd", DEFAULT_KD);

  prefs.end();

  systemOn = false; // safety boot OFF
}

static void saveSettings()
{
  prefs.begin("tecctrl", false);

  prefs.putBool("on", systemOn);
  prefs.putBool("mode", profileMode);
  prefs.putFloat("tconst", targetConstC);
  prefs.putFloat("maxpwr", maxPowerPercent);

  prefs.putBool("pumpen", pumpEnabled);
  prefs.putFloat("pumpspd", pumpSpeedPercent);
  prefs.putBool("dispon", displayOn);

  for (int i = 0; i < NUM_TEC; i++)
  {
    String k = "inv" + String(i);
    prefs.putBool(k.c_str(), dirInvert[i]);
  }

  prefs.putBytes("prof", profileC, sizeof(profileC));

  prefs.putFloat("kp", kp);
  prefs.putFloat("ki", ki);
  prefs.putFloat("kd", kd);

  prefs.end();
}

// WiFi creds
static bool loadWiFiCreds(String &ssid, String &pass)
{
  prefs.begin("net", true);
  ssid = prefs.getString("ssid", "");
  pass = prefs.getString("pass", "");
  prefs.end();
  return ssid.length() > 0;
}

static void saveWiFiCreds(const String &ssid, const String &pass)
{
  prefs.begin("net", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}

// ======================= DS18B20 (HEATSINKS) =======================

static void printDeviceAddress(const DeviceAddress &addr)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (addr[i] < 16)
      Serial.print("0");
    Serial.print(addr[i], HEX);
  }
}

static void setupDs18b20()
{
  pinMode(ONE_WIRE_PIN, INPUT_PULLUP);
  delay(5);

  Serial.printf("GPIO%d idle level (expect 1): %d\n", ONE_WIRE_PIN, digitalRead(ONE_WIRE_PIN));

  ds18b20.begin();
  int count = ds18b20.getDeviceCount();
  Serial.printf("DS18B20 device count: %d\n", count);

  hsCount = 0;
  for (int i = 0; i < NUM_HEATSINK && i < count; i++)
  {
    if (ds18b20.getAddress(hsAddr[i], i))
    {
      Serial.printf("DS18B20[%d] addr: ", i);
      printDeviceAddress(hsAddr[i]);
      Serial.println();
      ds18b20.setResolution(hsAddr[i], 10);
      hsCount++;
    }
  }

  ds18b20.setWaitForConversion(false);

  if (hsCount > 0)
  {
    ds18b20.requestTemperatures();
    tempConvStartMs = millis();
    tempConvInFlight = true;
    lastTempKickMs = millis();
  }
  else
  {
    Serial.println("No heatsink DS18B20 detected. Check wiring + pull-up DATA->3V3.");
  }
}

static void serviceHeatsinkTemps()
{
  if (hsCount <= 0)
    return;

  uint32_t now = millis();

  if (!tempConvInFlight)
  {
    if ((now - lastTempKickMs) >= 1000)
    {
      ds18b20.requestTemperatures();
      tempConvStartMs = now;
      tempConvInFlight = true;
    }
    return;
  }

  if ((now - tempConvStartMs) >= tempConvDelayMs)
  {
    lastTempKickMs = now;
    tempConvInFlight = false;

    for (int i = 0; i < hsCount; i++)
    {
      float t = ds18b20.getTempC(hsAddr[i]);
      float newTemp = (t == DEVICE_DISCONNECTED_C) ? NAN : t;
      if ((isnan(newTemp) != isnan(heatsinkTempC[i])) ||
          (!isnan(newTemp) && fabsf(newTemp - heatsinkTempC[i]) > 0.1f))
      {
        heatsinkTempC[i] = newTemp;
        dispDirty = true;
      }
    }
  }
}

static float maxHeatsinkC()
{
  float m = NAN;
  for (int i = 0; i < hsCount; i++)
  {
    if (!isnan(heatsinkTempC[i]))
    {
      if (isnan(m) || heatsinkTempC[i] > m)
        m = heatsinkTempC[i];
    }
  }
  return m;
}

// ======================= COOLANT NTC PROBE =======================

static void serviceCoolantTemp()
{
  const int samples = 16;
  uint32_t sum = 0;
  for (int k = 0; k < samples; k++)
    sum += (uint32_t)analogReadMilliVolts(NTC_ADC_PIN);
  float mv = (float)sum / (float)samples;

  float newTemp;
  // Out-of-range readings indicate an open or shorted probe.
  if (mv < 50.0f || mv > (NTC_SUPPLY_MV - 50.0f))
  {
    newTemp = NAN;
  }
  else
  {
    // 3.3V -- NTC -- [node] -- R_FIXED -- GND
    float rNtc = NTC_R_FIXED * ((NTC_SUPPLY_MV / mv) - 1.0f);
    float tK = 1.0f / ((1.0f / NTC_T0_K) + (1.0f / NTC_BETA) * logf(rNtc / NTC_R0));
    newTemp = tK - 273.15f;
  }

  if ((isnan(newTemp) != isnan(waterTempC)) ||
      (!isnan(newTemp) && fabsf(newTemp - waterTempC) > 0.05f))
  {
    waterTempC = newTemp;
    dispDirty = true;
  }
}

// ======================= WATER LEVEL =======================

static void serviceWaterLevel()
{
  bool raw = (digitalRead(WATER_LEVEL_PIN) == HIGH);
  if (!WATER_LEVEL_ACTIVE_HIGH)
    raw = !raw;

  uint32_t now = millis();
  if (raw != waterLevelRawLast)
  {
    waterLevelRawLast = raw;
    waterLevelChangeMs = now;
  }

  // 250 ms debounce
  if (raw != waterLevelOK && (now - waterLevelChangeMs) >= 250)
  {
    waterLevelOK = raw;
    dispDirty = true;
  }
}

// ======================= HEATSINK OVER-TEMP SAFETY =======================

static void serviceOverTemp()
{
  float hot = maxHeatsinkC();

  // Trip while running if a heatsink exceeds the limit.
  if (!faultTripped && (systemOn || manualActive) && !isnan(hot) && hot >= HEATSINK_MAX_C)
  {
    if (overTempCount < 255)
      overTempCount++;
    if (overTempCount >= 3)
    {
      faultTripped = true;
      faultMsg = "HEATSINK OVERTEMP (" + String(hot, 1) + "C)";
      systemOn = false;
      manualActive = false;
      allTecOff();
      saveSettings();
      dispDirty = true;
    }
  }
  else
  {
    overTempCount = 0;
  }
}

static void serviceFaultAutoClear()
{
  if (!faultTripped)
    return;
  if (systemOn || manualActive)
    return;

  // Clear once the heatsinks have cooled below the hysteresis threshold.
  float hot = maxHeatsinkC();
  if (isnan(hot) || hot <= HEATSINK_CLEAR_C)
  {
    faultTripped = false;
    faultMsg = "";
    overTempCount = 0;
    dispDirty = true;
  }
}

// ======================= TEC OUTPUT =======================

static void setTecOutput(int i, bool heatMode, float duty01)
{
  duty01 = clampf(duty01, 0.0f, 1.0f);

  bool coolLevel = COOL_DIR_LEVEL_HIGH;
  if (dirInvert[i])
    coolLevel = !coolLevel;

  bool dirLevel = heatMode ? !coolLevel : coolLevel;
  digitalWrite(TEC_DIR_PINS[i], dirLevel ? HIGH : LOW);

  const int maxDuty = (1 << TEC_PWM_RES_BITS) - 1;
  int dutyCounts = (int)lroundf(duty01 * (float)maxDuty);
  dutyCounts = constrain(dutyCounts, 0, maxDuty);

  ledcWrite(i, dutyCounts);

  float prev = tecDutyApplied[i];
  tecDutyApplied[i] = duty01;
  if (fabsf(prev - tecDutyApplied[i]) > 0.02f)
    dispDirty = true;
}

static void allTecOff()
{
  for (int i = 0; i < NUM_TEC; i++)
    setTecOutput(i, false, 0.0f);
}

// ======================= PUMP OUTPUT =======================

static void IRAM_ATTR pumpTachISR()
{
  pumpTachCount++;
}

static void setPump(float duty01)
{
  duty01 = clampf(duty01, 0.0f, 1.0f);
  const int maxDuty = (1 << PUMP_PWM_RES_BITS) - 1;
  int counts = (int)lroundf(duty01 * (float)maxDuty);
  counts = constrain(counts, 0, maxDuty);
  ledcWrite(PUMP_LEDC_CH, counts);

  if (fabsf(pumpDutyApplied - duty01) > 0.02f)
    dispDirty = true;
  pumpDutyApplied = duty01;
}

// The pump may only spin when the loop is full (sensor submerged); running it
// dry can destroy the D5. It runs whenever the system (or a manual test) is
// active AND there is water AND no fault.
static void servicePump()
{
  bool wantPump = pumpEnabled && waterLevelOK && !faultTripped && (systemOn || manualActive);
  setPump(wantPump ? (pumpSpeedPercent / 100.0f) : 0.0f);
}

static void servicePumpTach()
{
  uint32_t now = millis();
  uint32_t dt = now - lastTachSampleMs;
  if (dt < 1000)
    return;

  noInterrupts();
  uint32_t c = pumpTachCount;
  pumpTachCount = 0;
  interrupts();

  float rpm = (PUMP_TACH_PULSES_PER_REV > 0.0f)
                  ? ((float)c * (60000.0f / (float)dt) / PUMP_TACH_PULSES_PER_REV)
                  : 0.0f;
  lastTachSampleMs = now;

  if (fabsf(rpm - pumpRpm) > 30.0f)
    dispDirty = true;
  pumpRpm = rpm;
}

// ======================= TARGETING =======================

static float computeProfileTarget(float &posHoursOut)
{
  if (profileStartMs == 0)
    profileStartMs = millis();

  float elapsedHours = (millis() - profileStartMs) / 3600000.0f;
  float h = fmodf(elapsedHours, 24.0f);
  if (h < 0)
    h += 24.0f;

  int h0 = (int)floorf(h);
  float frac = h - (float)h0;
  int h1 = (h0 + 1) % 24;

  posHoursOut = h;
  float t0 = profileC[h0];
  float t1 = profileC[h1];
  return t0 * (1.0f - frac) + t1 * frac;
}

// ======================= PID CONTROL =======================

static void updateControl()
{
  // TECs must never run without coolant flow: require water level OK.
  if (faultTripped || !systemOn || !waterLevelOK || isnan(waterTempC) || manualActive)
  {
    if (!manualActive)
      allTecOff();
    resetPid();
    return;
  }

  float posHours = NAN;
  targetTempC = profileMode ? computeProfileTarget(posHours) : targetConstC;

  uint32_t now = millis();
  float dt = (now - lastPidMs) / 1000.0f;
  if (dt <= 0.0f)
    dt = 0.001f;
  lastPidMs = now;

  float err = targetTempC - waterTempC;

  if ((err > 0 && lastErr < 0) || (err < 0 && lastErr > 0))
    integralErr = 0.0f;

  integralErr += err * dt;
  float integralMax = (ki != 0.0f) ? (200.0f / ki) : 0.0f;
  integralErr = clampf(integralErr, -integralMax, integralMax);

  float deriv = (err - lastErr) / dt;
  lastErr = err;

  float output = kp * err + ki * integralErr + kd * deriv;

  bool heatMode = (output > 0.0f);
  float demandPercent = clampf(fabsf(output), 0.0f, maxPowerPercent);
  float baseDuty = demandPercent / 100.0f;

  for (int i = 0; i < NUM_TEC; i++)
    setTecOutput(i, heatMode, baseDuty);
}

// ======================= MANUAL TEST =======================

static void startManualTest(int chan, bool heat, float duty, uint32_t durationMs)
{
  chan = constrain(chan, 0, NUM_TEC - 1);
  duty = clampf(duty, 0.0f, 1.0f);

  manualActive = true;
  manualChan = chan;
  manualHeat = heat;
  manualDuty = duty;
  manualEndMs = millis() + durationMs;

  systemOn = false;
  resetPid();
  allTecOff();
  lastPowerChangeMs = millis();
  dispDirty = true;
}

static void serviceManualTest()
{
  if (!manualActive)
    return;

  // Stop the test on a fault or if the loop runs dry (TECs would overheat).
  if (faultTripped || !waterLevelOK)
  {
    manualActive = false;
    allTecOff();
    dispDirty = true;
    return;
  }

  if ((int32_t)(millis() - manualEndMs) >= 0)
  {
    manualActive = false;
    allTecOff();
    lastPowerChangeMs = millis();
    dispDirty = true;
    return;
  }

  for (int i = 0; i < NUM_TEC; i++)
  {
    if (i == manualChan)
      setTecOutput(i, manualHeat, manualDuty);
    else
      setTecOutput(i, false, 0.0f);
  }
}

// ======================= DISPLAY =======================

static void clearBox(int x, int y, int w, int h)
{
  gfx->fillRect(x, y, w, h, BLACK);
}

static String truncateToFit(const String &s, int maxChars)
{
  if ((int)s.length() <= maxChars)
    return s;
  if (maxChars <= 3)
    return s.substring(0, maxChars);
  return s.substring(0, maxChars - 3) + "...";
}

static void applyBacklight(bool on)
{
  if (TFT_BL == GFX_NOT_DEFINED)
    return;
  digitalWrite(TFT_BL, on ? HIGH : LOW);
}

static void computeLayout()
{
  W = gfx->width();
  H = gfx->height();

  waterValX = 120;
  waterValY = 18;
  waterValW = W - waterValX - M;
  waterValH = 36;

  targetValX = 120;
  targetValY = 64;
  targetValW = W - targetValX - M;
  targetValH = 36;

  levelX = M;
  levelY = 108;
  levelW = W - 2 * M;
  levelH = 14;

  hsX = M;
  hsY = 124;
  hsW = W - 2 * M;
  hsH = 12;

  ssidX = M;
  ssidY = H - 24;
  ssidW = W - 2 * M;
  ssidH = 12;

  ipX = M;
  ipY = H - 12;
  ipW = W - 2 * M;
  ipH = 12;

  faultH = 18;
  faultY = H - faultH;
}

static void drawDisplayStatic()
{
  dispStaticDrawn = true;
  computeLayout();

  gfx->fillScreen(BLACK);
  gfx->setTextWrap(false);
  gfx->setTextColor(WHITE);

  gfx->setTextSize(2);
  gfx->setCursor(M, 22);
  gfx->print("WATER");

  gfx->setCursor(M, 68);
  gfx->print("TARGET");

  gfx->drawLine(M, 58, W - M, 58, rgb565(40, 40, 40));
  gfx->drawLine(M, H - 28, W - M, H - 28, rgb565(40, 40, 40));

  dispLast = DispCache();
  dispDirty = true;
}

static void drawDisplayDynamicIfNeeded()
{
  // Display fully off (dark room): blank panel + backlight off, draw nothing.
  if (!displayOn)
  {
    if (!dispBlankedDrawn)
    {
      gfx->fillScreen(BLACK);
      applyBacklight(false);
      dispBlankedDrawn = true;
      dispStaticDrawn = false; // force full redraw when turned back on
    }
    return;
  }

  if (dispBlankedDrawn)
  {
    applyBacklight(true);
    dispBlankedDrawn = false;
    dispStaticDrawn = false;
  }

  if (!dispStaticDrawn)
    drawDisplayStatic();
  if (!dispDirty)
    return;

  String waterLine;
  if (isnan(waterTempC))
    waterLine = "PROBE?";
  else
    waterLine = String(waterTempC, 2) + "C";

  float tgt = targetConstC;
  if (profileMode)
  {
    float ph = NAN;
    tgt = (systemOn ? computeProfileTarget(ph) : profileC[0]);
  }
  String targetLine = String(tgt, 2) + "C";

  String levelLine = waterLevelOK ? "LEVEL: FULL" : "LEVEL: NEEDS WATER";

  String hsLine = "HS ";
  for (int i = 0; i < NUM_HEATSINK; i++)
  {
    if (i)
      hsLine += " ";
    hsLine += String(i + 1) + ":";
    hsLine += isnan(heatsinkTempC[i]) ? String("--") : String(heatsinkTempC[i], 0);
  }
  hsLine += "C PUMP:" + String((int)pumpRpm);

  String ssidLine;
  String ipLine;
  if (apMode)
  {
    ssidLine = String("SSID: ") + AP_SSID;
    ipLine = String("IP: 192.168.4.1");
  }
  else
  {
    ssidLine = "SSID: " + truncateToFit(staSSID, 26);
    ipLine = "IP: " + staIP + "  tec-ctrl.local";
  }

  bool f = faultTripped;
  String faultLine = f ? ("FAULT: " + faultMsg) : "";

  if (waterLine != dispLast.waterLine)
  {
    clearBox(waterValX, waterValY, waterValW, waterValH);
    gfx->setTextColor(WHITE);
    gfx->setTextSize(4);
    gfx->setCursor(waterValX, waterValY + 4);
    gfx->print(waterLine);
    dispLast.waterLine = waterLine;
  }

  if (targetLine != dispLast.targetLine)
  {
    clearBox(targetValX, targetValY, targetValW, targetValH);
    gfx->setTextColor(WHITE);
    gfx->setTextSize(4);
    gfx->setCursor(targetValX, targetValY + 4);
    gfx->print(targetLine);
    dispLast.targetLine = targetLine;
  }

  // Water level status (green = full, red = needs water)
  if (levelLine != dispLast.levelLine)
  {
    clearBox(levelX, levelY, levelW, levelH);
    gfx->setTextSize(1);
    gfx->setTextColor(waterLevelOK ? rgb565(0, 220, 0) : rgb565(255, 60, 60));
    gfx->setCursor(levelX, levelY + 2);
    gfx->print(levelLine);
    dispLast.levelLine = levelLine;
  }

  if (hsLine != dispLast.hsLine)
  {
    clearBox(hsX, hsY, hsW, hsH);
    gfx->setTextSize(1);
    gfx->setTextColor(rgb565(220, 220, 220));
    gfx->setCursor(hsX, hsY + 2);
    gfx->print(truncateToFit(hsLine, 52));
    dispLast.hsLine = hsLine;
  }

  if (ssidLine != dispLast.ssidLine)
  {
    clearBox(ssidX, ssidY, ssidW, ssidH);
    gfx->setTextSize(1);
    gfx->setTextColor(rgb565(220, 220, 220));
    gfx->setCursor(ssidX, ssidY + 2);
    gfx->print(ssidLine);
    dispLast.ssidLine = ssidLine;
  }

  if (ipLine != dispLast.ipLine)
  {
    clearBox(ipX, ipY, ipW, ipH);
    gfx->setTextSize(1);
    gfx->setTextColor(rgb565(220, 220, 220));
    gfx->setCursor(ipX, ipY + 2);
    gfx->print(ipLine);
    dispLast.ipLine = ipLine;
  }

  if (f != dispLast.fault || faultLine != dispLast.faultLine)
  {
    gfx->fillRect(0, faultY, W, faultH, BLACK);
    gfx->drawLine(M, H - 28, W - M, H - 28, rgb565(40, 40, 40));

    if (f)
    {
      gfx->fillRect(0, faultY, W, faultH, rgb565(160, 0, 0));
      gfx->setTextColor(WHITE);
      gfx->setTextSize(1);
      gfx->setCursor(6, faultY + 4);
      gfx->print(truncateToFit(faultLine, 42));
    }

    dispLast.fault = f;
    dispLast.faultLine = faultLine;
  }

  dispDirty = false;
}

// ======================= WEB UI =======================

static String pageHeader(const String &title)
{
  String s;
  s.reserve(900);
  s += "<!doctype html><html><head><meta charset='utf-8'>";
  s += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  s += "<title>" + htmlEscape(title) + "</title>";
  s += "<style>";
  s += "body{font-family:system-ui,Segoe UI,Arial;margin:16px;max-width:900px}";
  s += ".card{border:1px solid #ddd;border-radius:12px;padding:14px;margin:12px 0}";
  s += ".row{display:flex;gap:12px;flex-wrap:wrap}";
  s += "label{display:block;font-size:12px;color:#444;margin-bottom:6px}";
  s += "input,select,button{font-size:16px;padding:10px;border-radius:10px;border:1px solid #ccc}";
  s += "button{cursor:pointer}";
  s += ".btn{background:#111;color:#fff;border:0}";
  s += ".btn2{background:#0b5;color:#fff;border:0}";
  s += ".btn3{background:#b30;color:#fff;border:0}";
  s += ".small{font-size:12px;color:#666}";
  s += "pre{background:#f6f6f6;padding:10px;border-radius:10px;overflow:auto}";
  s += "</style></head><body>";
  return s;
}

static String pageFooter() { return "</body></html>"; }

static String wifiPage(bool showSaved)
{
  String ssid, pass;
  bool has = loadWiFiCreds(ssid, pass);

  String s = pageHeader("WiFi Setup");
  s += "<h2>WiFi Setup</h2>";
  s += "<div class='card'>";
  s += "<p>Enter WiFi credentials (2.4GHz). Device saves them and reboots.</p>";
  if (showSaved)
    s += "<p><b>Saved.</b> Rebooting...</p>";
  s += "<form method='POST' action='/wifisave'>";
  s += "<label>SSID</label><input name='ssid' value='" + htmlEscape(has ? ssid : "") + "' style='width:100%'>";
  s += "<label>Password</label><input name='pass' type='password' value='' style='width:100%'>";
  s += "<div style='margin-top:12px'><button class='btn2' type='submit'>Save & Reboot</button></div>";
  s += "</form>";
  s += "</div>";
  s += pageFooter();
  return s;
}

static String controlPage();

// ======================= API JSON =======================

static String jsonState()
{
  String s;
  s.reserve(700);

  float tgt = NAN;
  if (profileMode)
  {
    float ph = NAN;
    tgt = systemOn ? computeProfileTarget(ph) : profileC[0];
  }
  else
  {
    tgt = targetConstC;
  }

  s += "{";
  s += "\"on\":" + String(systemOn ? "true" : "false") + ",";
  s += "\"manual\":" + String(manualActive ? "true" : "false") + ",";
  s += "\"mode\":" + String(profileMode ? "true" : "false") + ",";
  s += "\"water\":" + (isnan(waterTempC) ? String("null") : String(waterTempC, 3)) + ",";
  s += "\"level\":" + String(waterLevelOK ? "true" : "false") + ",";
  s += "\"target\":" + (isnan(tgt) ? String("null") : String(tgt, 3)) + ",";
  s += "\"tconst\":" + String(targetConstC, 2) + ",";
  s += "\"maxp\":" + String(maxPowerPercent, 1) + ",";
  s += "\"pumpen\":" + String(pumpEnabled ? "true" : "false") + ",";
  s += "\"pumpspd\":" + String(pumpSpeedPercent, 0) + ",";
  s += "\"pumprpm\":" + String((int)pumpRpm) + ",";
  s += "\"dispon\":" + String(displayOn ? "true" : "false") + ",";
  s += "\"fault\":" + String(faultTripped ? "true" : "false") + ",";
  s += "\"faultMsg\":\"" + htmlEscape(faultMsg) + "\",";
  s += "\"ip\":\"" + htmlEscape(apMode ? WiFi.softAPIP().toString() : staIP) + "\",";
  s += "\"ssid\":\"" + htmlEscape(apMode ? String(AP_SSID) : staSSID) + "\",";
  s += "\"invert_snippet\":\"" + htmlEscape(invertSnippet()) + "\",";
  s += "\"hs\":[";
  for (int i = 0; i < NUM_HEATSINK; i++)
  {
    if (i)
      s += ",";
    s += isnan(heatsinkTempC[i]) ? String("null") : String(heatsinkTempC[i], 2);
  }
  s += "],\"duty\":[";
  for (int i = 0; i < NUM_TEC; i++)
  {
    if (i)
      s += ",";
    s += String(tecDutyApplied[i], 3);
  }
  s += "],\"inv\":[";
  for (int i = 0; i < NUM_TEC; i++)
  {
    if (i)
      s += ",";
    s += (dirInvert[i] ? "true" : "false");
  }
  s += "]";
  s += "}";
  return s;
}

// Minimal JSON helpers
static String bodyText()
{
  if (server.hasArg("plain"))
    return server.arg("plain");
  return "";
}
static bool jsonBool(const String &b, const char *key, bool &out)
{
  String k = String("\"") + key + "\":";
  int p = b.indexOf(k);
  if (p < 0)
    return false;
  p += k.length();
  while (p < (int)b.length() && isspace((unsigned char)b[p]))
    p++;
  if (b.startsWith("true", p))
  {
    out = true;
    return true;
  }
  if (b.startsWith("false", p))
  {
    out = false;
    return true;
  }
  return false;
}
static bool jsonFloat(const String &b, const char *key, float &out)
{
  String k = String("\"") + key + "\":";
  int p = b.indexOf(k);
  if (p < 0)
    return false;
  p += k.length();
  while (p < (int)b.length() && isspace((unsigned char)b[p]))
    p++;
  int e = p;
  while (e < (int)b.length() && (isDigit(b[e]) || b[e] == '-' || b[e] == '+' || b[e] == '.' || b[e] == 'e' || b[e] == 'E'))
    e++;
  if (e <= p)
    return false;
  out = b.substring(p, e).toFloat();
  return true;
}
static bool jsonInt(const String &b, const char *key, int &out)
{
  float f;
  if (!jsonFloat(b, key, f))
    return false;
  out = (int)lroundf(f);
  return true;
}
static bool jsonInvArray(const String &b, bool outArr[NUM_TEC])
{
  int p = b.indexOf("\"inv\"");
  if (p < 0)
    return false;
  p = b.indexOf("[", p);
  if (p < 0)
    return false;
  int q = b.indexOf("]", p);
  if (q < 0)
    return false;
  String inside = b.substring(p + 1, q);
  inside.replace(" ", "");
  inside.replace("\n", "");

  int idx = 0;
  int start = 0;
  while (idx < NUM_TEC)
  {
    int comma = inside.indexOf(",", start);
    String tok = (comma < 0) ? inside.substring(start) : inside.substring(start, comma);
    tok.trim();
    if (tok == "true")
      outArr[idx] = true;
    else if (tok == "false")
      outArr[idx] = false;
    else
      return false;
    idx++;
    if (comma < 0)
      break;
    start = comma + 1;
  }
  return (idx == NUM_TEC);
}

// ======================= WEB HANDLERS =======================

static void handleRoot()
{
  if (apMode)
  {
    server.sendHeader("Location", "/wifi", true);
    server.send(302, "text/plain", "");
    return;
  }
  server.send(200, "text/html", controlPage());
}

static void handleWiFiPage() { server.send(200, "text/html", wifiPage(false)); }

static void handleWiFiSave()
{
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  ssid.trim();

  if (ssid.length() == 0)
  {
    server.send(400, "text/html", pageHeader("WiFi Setup") + "<h2>WiFi Setup</h2><p>SSID required.</p><a href='/wifi'>Back</a>" + pageFooter());
    return;
  }

  saveWiFiCreds(ssid, pass);

  server.sendHeader("Connection", "close");
  server.send(200, "text/html", wifiPage(true));

  scheduleReboot(900);
}

static void handleApiState() { server.send(200, "application/json", jsonState()); }

static void handleApiControl()
{
  String b = bodyText();

  bool onv;
  if (jsonBool(b, "on", onv))
  {
    if (!faultTripped)
    {
      bool prev = systemOn;
      systemOn = onv;
      manualActive = false;
      if (systemOn && !prev)
        profileStartMs = millis();
      resetPid();
      lastPowerChangeMs = millis();
      if (!systemOn)
        allTecOff();
      dispDirty = true;
    }
  }

  bool modev;
  if (jsonBool(b, "mode", modev))
  {
    if (profileMode != modev)
    {
      profileMode = modev;
      profileStartMs = millis();
      resetPid();
      dispDirty = true;
    }
  }

  bool bv;
  if (jsonBool(b, "pumpen", bv))
    pumpEnabled = bv;
  if (jsonBool(b, "dispon", bv))
  {
    displayOn = bv;
    dispDirty = true;
  }

  float f;
  if (jsonFloat(b, "tconst", f))
  {
    targetConstC = clampf(f, -5.0f, 60.0f);
    dispDirty = true;
  }
  if (jsonFloat(b, "maxp", f))
  {
    maxPowerPercent = clampf(f, 0.0f, 100.0f);
  }
  if (jsonFloat(b, "pumpspd", f))
  {
    pumpSpeedPercent = clampf(f, 0.0f, 100.0f);
  }

  saveSettings();
  server.send(200, "application/json", jsonState());
}

static void handleApiInvert()
{
  String b = bodyText();
  bool inv[NUM_TEC];
  if (!jsonInvArray(b, inv))
  {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }
  bool changed = false;
  for (int i = 0; i < NUM_TEC; i++)
  {
    if (dirInvert[i] != inv[i])
    {
      dirInvert[i] = inv[i];
      changed = true;
    }
  }
  if (changed)
  {
    saveSettings();
    printInvertSnippetToSerial();
  }
  server.send(200, "application/json", jsonState());
}

static void handleApiProfile()
{
  String b = bodyText();
  int hour = 0;
  float temp = targetConstC;
  if (!jsonInt(b, "hour", hour) || !jsonFloat(b, "temp", temp))
  {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }
  if (hour < 0 || hour > 23)
  {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }
  profileC[hour] = clampf(temp, -5.0f, 60.0f);
  saveSettings();
  dispDirty = true;
  server.send(200, "application/json", jsonState());
}

static void handleApiFaultClear()
{
  if (!systemOn && !manualActive)
  {
    faultTripped = false;
    faultMsg = "";
    overTempCount = 0;
    dispDirty = true;
  }
  server.send(200, "application/json", jsonState());
}

static void handleApiTest()
{
  String b = bodyText();
  int chan = 0;
  bool heat = false;
  float duty = 0.2f;
  int dur = 10000;

  jsonInt(b, "chan", chan);
  jsonBool(b, "heat", heat);
  jsonFloat(b, "duty", duty);
  jsonInt(b, "dur", dur);

  // Refuse a manual TEC test with no coolant flow (dry loop).
  if (!faultTripped && waterLevelOK)
    startManualTest(chan, heat, duty, (uint32_t)constrain(dur, 1000, 600000));
  server.send(200, "application/json", jsonState());
}

static void handleApiTestStop()
{
  manualActive = false;
  allTecOff();
  lastPowerChangeMs = millis();
  dispDirty = true;
  server.send(200, "application/json", jsonState());
}

static void handleNotFound()
{
  if (apMode)
  {
    server.sendHeader("Location", "/wifi", true);
    server.send(302, "text/plain", "");
    return;
  }
  server.send(404, "text/plain", "Not found");
}

static void setupWebServer()
{
  server.on("/", HTTP_GET, handleRoot);

  server.on("/wifi", HTTP_GET, handleWiFiPage);
  server.on("/wifisave", HTTP_POST, handleWiFiSave);

  server.on("/api/state", HTTP_GET, handleApiState);
  server.on("/api/control", HTTP_POST, handleApiControl);
  server.on("/api/invert", HTTP_POST, handleApiInvert);
  server.on("/api/profile", HTTP_POST, handleApiProfile);
  server.on("/api/fault_clear", HTTP_POST, handleApiFaultClear);
  server.on("/api/test", HTTP_POST, handleApiTest);
  server.on("/api/test_stop", HTTP_POST, handleApiTestStop);

  server.on("/generate_204", HTTP_GET, handleWiFiPage);
  server.on("/hotspot-detect.html", HTTP_GET, handleWiFiPage);
  server.on("/connecttest.txt", HTTP_GET, handleWiFiPage);
  server.on("/fwlink", HTTP_GET, handleWiFiPage);

  server.onNotFound(handleNotFound);
  server.begin();
}

// ======================= WIFI =======================

static void startAPPortal()
{
  apMode = true;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);
  IPAddress ip = WiFi.softAPIP();

  Serial.printf("AP mode started. SSID=%s IP=%s\n", AP_SSID, ip.toString().c_str());

  dnsServer.start(DNS_PORT, "*", ip);
  setupWebServer();

  staIP = "";
  dispStaticDrawn = false;
  dispDirty = true;
}

static bool connectSTA(const String &ssid, const String &pass)
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(MDNS_HOST);

  WiFi.begin(ssid.c_str(), pass.c_str());
  Serial.printf("Connecting to WiFi SSID='%s'...\n", ssid.c_str());

  uint32_t start = millis();
  while (millis() - start < 15000)
  {
    if (WiFi.status() == WL_CONNECTED)
      return true;
    delay(200);
  }
  return false;
}

static void startSTAWeb(const String &ssid, const String &pass)
{
  apMode = false;
  staSSID = ssid;
  staPASS = pass;
  staIP = WiFi.localIP().toString();

  Serial.printf("Connected. IP=%s\n", staIP.c_str());

  if (MDNS.begin(MDNS_HOST))
  {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("mDNS: http://%s.local/\n", MDNS_HOST);
  }
  else
  {
    Serial.println("mDNS start failed");
  }

  setupWebServer();

  dispStaticDrawn = false;
  dispDirty = true;
}

// ======================= WEB PAGE =======================

static String controlPage()
{
  String s = pageHeader("TEC Controller");
  s += "<h2>TEC Bed Cooler</h2>";

  s += "<div class='card'><div class='row'>";

  s += "<div style='flex:1;min-width:280px'>";
  s += "<div><b>Status</b></div>";
  s += "<div class='small' id='net'></div>";
  s += "<pre id='status'>Loading...</pre>";
  s += "<div class='row'>";
  s += "<button class='btn' onclick=\"setPower(1)\">Power ON</button>";
  s += "<button class='btn3' onclick=\"setPower(0)\">Power OFF</button>";
  s += "<button onclick=\"clearFault()\">Clear Fault</button>";
  s += "</div>";
  s += "</div>";

  s += "<div style='flex:1;min-width:280px'>";
  s += "<div><b>Setpoints</b></div>";
  s += "<label>Mode</label><select id='mode'><option value='0'>Constant</option><option value='1'>Profile</option></select>";
  s += "<label>Const target (C)</label><input id='tconst' type='number' step='0.1'>";
  s += "<label>Max power (%)</label><input id='maxp' type='number' step='1'>";
  s += "<div style='margin-top:10px'><button class='btn2' onclick='saveControl()'>Apply</button></div>";
  s += "</div>";

  s += "</div></div>";

  // Pump + display settings
  s += "<div class='card'>";
  s += "<div><b>Pump &amp; Display</b></div>";
  s += "<div class='row' style='margin-top:10px'>";
  s += "<div><label>Pump enabled</label><select id='pumpen'><option value='1'>On</option><option value='0'>Off</option></select></div>";
  s += "<div><label>Pump speed (%)</label><input id='pumpspd' type='number' step='1' min='0' max='100'></div>";
  s += "<div><label>Display</label><select id='dispon'><option value='1'>On</option><option value='0'>Off (dark room)</option></select></div>";
  s += "</div>";
  s += "<div class='small' style='margin-top:8px'>Pump runs only when the water-level sensor reports FULL. If it reads NEEDS WATER, the pump and TECs are locked off to avoid running the pump dry.</div>";
  s += "<div style='margin-top:10px'><button class='btn2' onclick='savePumpDisp()'>Apply</button></div>";
  s += "</div>";

  s += "<div class='card'>";
  s += "<div><b>Polarity / Manual TEC Test</b></div>";
  s += "<div class='small'>Verify each TEC, then paste the snippet into the code to make polarity permanent. (Requires water level FULL.)</div>";
  s += "<div class='row' style='margin-top:10px'>";
  s += "<div><label>Channel</label><select id='mchan'><option value='0'>TEC1</option><option value='1'>TEC2</option><option value='2'>TEC3</option><option value='3'>TEC4</option></select></div>";
  s += "<div><label>Direction</label><select id='mdir'><option value='0'>COOL</option><option value='1'>HEAT</option></select></div>";
  s += "<div><label>Duty (%)</label><input id='mduty' type='number' value='20' step='1'></div>";
  s += "<div><label>Duration (s)</label><input id='mdur' type='number' value='10' step='1'></div>";
  s += "</div>";
  s += "<div class='row' style='margin-top:10px'>";
  s += "<button class='btn2' onclick='startTest()'>Start Test</button>";
  s += "<button class='btn3' onclick='stopTest()'>Stop Test</button>";
  s += "</div>";

  s += "<hr>";
  s += "<div><b>Invert direction per channel</b></div>";
  s += "<div class='row' style='margin-top:8px'>";
  for (int i = 0; i < NUM_TEC; i++)
  {
    s += "<label style='display:flex;align-items:center;gap:8px;margin-right:12px'>";
    s += "<input type='checkbox' id='inv" + String(i) + "'>";
    s += "Invert TEC" + String(i + 1) + "</label>";
  }
  s += "</div>";
  s += "<div style='margin-top:10px'><button onclick='saveInvert()'>Save Invert</button></div>";

  s += "<div style='margin-top:10px'><b>CPP snippet</b></div>";
  s += "<pre id='snip'>Loading...</pre>";

  s += "</div>";

  s += "<div class='card'>";
  s += "<div><b>Profile</b> <span class='small'>(hourly)</span></div>";
  s += "<div class='row' style='margin-top:10px'>";
  s += "<div><label>Hour (0-23)</label><input id='ph' type='number' min='0' max='23' value='0'></div>";
  s += "<div><label>Temp (C)</label><input id='pt' type='number' step='0.1' value='20.0'></div>";
  s += "<div style='align-self:flex-end'><button class='btn2' onclick='setProfilePoint()'>Set Point</button></div>";
  s += "</div></div>";

  s += "<div class='card'><b>WiFi</b><div class='row' style='margin-top:10px'>";
  s += "<a href='/wifi'>Change WiFi credentials</a>";
  s += "</div></div>";

  s += "<script>";
  s += "async function jget(u){const r=await fetch(u);return await r.json();}\n";
  s += "async function jpost(u,obj){const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(obj)});return await r.json();}\n";
  s += "function fmtState(st){\n";
  s += "let lines=[];\n";
  s += "lines.push('Power: '+(st.on?'ON':'OFF')+(st.manual?' (MANUAL TEST)':''));\n";
  s += "lines.push('Mode: '+(st.mode?'PROFILE':'CONST'));\n";
  s += "lines.push('Water level: '+(st.level?'FULL':'NEEDS WATER'));\n";
  s += "lines.push('Coolant: '+(isFinite(st.water)?(st.water.toFixed(2)+' C'):'PROBE ERR'));\n";
  s += "lines.push('Target: '+(isFinite(st.target)?st.target.toFixed(2)+' C':'----'));\n";
  s += "lines.push('Max power: '+st.maxp.toFixed(0)+' %');\n";
  s += "lines.push('Pump: '+(st.pumpen?'EN':'DIS')+' '+st.pumpspd.toFixed(0)+'%  '+st.pumprpm+' rpm');\n";
  s += "for(let i=0;i<st.hs.length;i++) lines.push('Heatsink'+(i+1)+': '+(st.hs[i]==null?'--':st.hs[i].toFixed(1)+' C'));\n";
  s += "for(let i=0;i<st.duty.length;i++) lines.push('TEC'+(i+1)+': duty '+(st.duty[i]*100).toFixed(0)+'%  inv:'+(st.inv[i]?'Y':'N'));\n";
  s += "if(st.fault) lines.push('\\nFAULT: '+st.faultMsg);\n";
  s += "return lines.join('\\n');\n";
  s += "}\n";
  s += "async function refresh(){\n";
  s += "const st=await jget('/api/state');\n";
  s += "document.getElementById('status').textContent=fmtState(st);\n";
  s += "document.getElementById('net').textContent='IP: '+st.ip+'  SSID: '+st.ssid+'  (try http://tec-ctrl.local/)';\n";
  s += "document.getElementById('mode').value=st.mode?1:0;\n";
  s += "document.getElementById('tconst').value=st.tconst.toFixed(1);\n";
  s += "document.getElementById('maxp').value=st.maxp.toFixed(0);\n";
  s += "document.getElementById('pumpen').value=st.pumpen?1:0;\n";
  s += "document.getElementById('pumpspd').value=st.pumpspd.toFixed(0);\n";
  s += "document.getElementById('dispon').value=st.dispon?1:0;\n";
  s += "for(let i=0;i<st.inv.length;i++) document.getElementById('inv'+i).checked=st.inv[i];\n";
  s += "document.getElementById('snip').textContent=st.invert_snippet;\n";
  s += "}\n";
  s += "async function setPower(v){await jpost('/api/control',{on:!!v});refresh();}\n";
  s += "async function clearFault(){await jpost('/api/fault_clear',{});refresh();}\n";
  s += "async function saveControl(){\n";
  s += "const obj={mode:document.getElementById('mode').value==='1',tconst:parseFloat(tconst.value),maxp:parseFloat(maxp.value)};\n";
  s += "await jpost('/api/control',obj);refresh();}\n";
  s += "async function savePumpDisp(){\n";
  s += "const obj={pumpen:document.getElementById('pumpen').value==='1',pumpspd:parseFloat(document.getElementById('pumpspd').value),dispon:document.getElementById('dispon').value==='1'};\n";
  s += "await jpost('/api/control',obj);refresh();}\n";
  s += "async function saveInvert(){\n";
  s += "let inv=[];for(let i=0;i<4;i++) inv.push(!!document.getElementById('inv'+i).checked);\n";
  s += "await jpost('/api/invert',{inv});refresh();}\n";
  s += "async function startTest(){\n";
  s += "const chan=parseInt(document.getElementById('mchan').value);\n";
  s += "const heat=document.getElementById('mdir').value==='1';\n";
  s += "const duty=parseFloat(document.getElementById('mduty').value)/100.0;\n";
  s += "const dur=parseFloat(document.getElementById('mdur').value)*1000;\n";
  s += "await jpost('/api/test',{chan,heat,duty,dur});refresh();}\n";
  s += "async function stopTest(){await jpost('/api/test_stop',{});refresh();}\n";
  s += "async function setProfilePoint(){\n";
  s += "const hour=parseInt(document.getElementById('ph').value);\n";
  s += "const temp=parseFloat(document.getElementById('pt').value);\n";
  s += "await jpost('/api/profile',{hour,temp});refresh();}\n";
  s += "setInterval(refresh,1000);refresh();\n";
  s += "</script>";

  s += pageFooter();
  return s;
}

// ======================= TFT INIT (robust) =======================

static void initTFT()
{
  if (TFT_BL != GFX_NOT_DEFINED)
  {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
  }

  gfx->begin(TFT_SPI_SPEED);

  gfx->fillScreen(BLACK);
  gfx->setTextWrap(false);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(10, 10);
  gfx->print("Display init OK");

  delay(150);
  applyBacklight(displayOn);
  dispStaticDrawn = false;
  dispDirty = true;

  Serial.printf("Display w=%d h=%d\n", gfx->width(), gfx->height());
}

// ======================= SETUP / LOOP =======================

void setup()
{
  Serial.begin(115200);
  delay(200);

  loadSettings();

  // DIR pins
  for (int i = 0; i < NUM_TEC; i++)
  {
    pinMode(TEC_DIR_PINS[i], OUTPUT);
    digitalWrite(TEC_DIR_PINS[i], COOL_DIR_LEVEL_HIGH ? HIGH : LOW);
  }

  // TEC PWM channels (LEDC 0..3)
  for (int i = 0; i < NUM_TEC; i++)
  {
    double actual = ledcSetup(i, TEC_PWM_FREQ_HZ, TEC_PWM_RES_BITS);
    if (actual <= 0.0)
      Serial.printf("LEDC setup failed ch=%d\n", i);
    ledcAttachPin(TEC_PWM_PINS[i], i);
    ledcWrite(i, 0);
  }

  // Pump PWM (LEDC ch 4) + tach input
  ledcSetup(PUMP_LEDC_CH, PUMP_PWM_FREQ_HZ, PUMP_PWM_RES_BITS);
  ledcAttachPin(PUMP_PWM_PIN, PUMP_LEDC_CH);
  ledcWrite(PUMP_LEDC_CH, 0);
  pinMode(PUMP_TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PUMP_TACH_PIN), pumpTachISR, FALLING);
  lastTachSampleMs = millis();

  // Water level sensor input (pull-down => "no water / off" if disconnected = safe)
  pinMode(WATER_LEVEL_PIN, INPUT_PULLDOWN);

  // NTC coolant probe ADC
  analogReadResolution(12);
  analogSetPinAttenuation(NTC_ADC_PIN, ADC_11db);

  // DS18B20 heatsink sensors
  setupDs18b20();

  // TFT
  initTFT();

  allTecOff();
  setPump(0.0f);
  resetPid();
  printInvertSnippetToSerial();

  // Prime water level state
  serviceWaterLevel();
  waterLevelOK = waterLevelRawLast;

  String ssid, pass;
  bool hasCreds = loadWiFiCreds(ssid, pass);

  if (hasCreds && connectSTA(ssid, pass))
  {
    startSTAWeb(ssid, pass);
  }
  else
  {
    startAPPortal();
  }

  lastPowerChangeMs = millis();
  dispDirty = true;
}

void loop()
{
  if (rebootPending && (int32_t)(millis() - rebootAtMs) >= 0)
  {
    Serial.println("Rebooting now...");
    delay(50);
    ESP.restart();
  }

  if (apMode)
    dnsServer.processNextRequest();
  server.handleClient();

  if (!apMode && WiFi.status() == WL_CONNECTED)
  {
    String newIp = WiFi.localIP().toString();
    if (newIp != staIP)
    {
      staIP = newIp;
      dispDirty = true;
    }
  }

  serviceWaterLevel();
  serviceCoolantTemp();
  serviceHeatsinkTemps();
  servicePumpTach();
  serviceOverTemp();

  static uint32_t lastControlMs = 0;
  if (millis() - lastControlMs >= 250)
  {
    lastControlMs = millis();
    updateControl();
  }

  serviceManualTest();
  servicePump();
  serviceFaultAutoClear();

  drawDisplayDynamicIfNeeded();
}
