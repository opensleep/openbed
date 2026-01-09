/*
  ESP32 TEC Controller (4 channels) for:
  - 4x Cytron MD13S (PWM + DIR)
  - 4x ACS712-20A current sensors (OUT -> 10k/10k divider -> ESP32 ADC1)
  - 1x DS18B20 water temp sensor (GPIO13 + 4.7k pull-up to 3V3)
  - 1x 1.9" IPS SPI TFT (ST7789 170x320 with offsets; LANDSCAPE UI)
  - 1x KY-040 rotary encoder + push button

  Key fixes included:
  - LEDC: 20kHz @ 11-bit (stable on ESP32)
  - DS18B20: non-blocking conversion (no UI stalls)
  - Encoder: glitch filter + guard window to stop “rotate triggers click”
  - Display: keeps the ORIGINAL working orientation/constructor/begin(),
             and uses partial redraw on STATUS screen to avoid flicker.

  Notes:
  - STATUS screen is horizontal/landscape and shows only Water + Target + State (no graph).
*/

#include <Arduino.h>
#include <math.h>
#include <Preferences.h>

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

// Cytron MD13S control pins (PWM + DIR)
static const int TEC_PWM_PINS[NUM_TEC] = {25, 27, 32, 16};
static const int TEC_DIR_PINS[NUM_TEC] = {26, 14, 33, 17};

// ACS712 analog pins (ADC1 recommended)
static const int ACS_ADC_PINS[NUM_TEC] = {34, 35, 36, 39};

// KY-040 encoder
static const int ENC_CLK_PIN = 19;
static const int ENC_DT_PIN = 21;
static const int ENC_SW_PIN = 22; // active LOW

// DS18B20
static const int ONE_WIRE_PIN = 13;

// TFT SPI pins
static const int TFT_SCK = 18;
static const int TFT_MOSI = 23;
static const int TFT_DC = 4;
static const int TFT_CS = GFX_NOT_DEFINED;  // CS tied to GND
static const int TFT_RST = GFX_NOT_DEFINED; // RES tied to EN/3V3 (or wire to GPIO and set it here)

// ST7789 170x320 + offsets (your working baseline)
static const int TFT_NATIVE_W = 170;
static const int TFT_NATIVE_H = 320;
static const int TFT_COL_OFFSET_1 = 35;
static const int TFT_ROW_OFFSET_1 = 0;
static const int TFT_COL_OFFSET_2 = 35;
static const int TFT_ROW_OFFSET_2 = 0;

// IMPORTANT: Keep original working orientation (landscape UI)
static const int TFT_ROTATION = 1;

// ======================= ELECTRICAL CALIBRATION =======================

// You confirmed you are using 10k/10k divider (ACS OUT -> 10k -> ADC node -> 10k -> GND)
// So sensitivity at ADC node is 50mV/A (100mV/A * 0.5)
static const float ACS_SENS_MV_PER_A_AT_ADC = 50.0f;
static const float CURRENT_NOISE_FLOOR_A = 0.20f;

// ======================= CONTROL LIMITS =======================

static const float DEFAULT_MAX_POWER_PERCENT = 75.0f;
static const float DEFAULT_CURRENT_LIMIT_A = 10.0f;
static const float HARD_OVERCURRENT_A = 12.5f;

// PID defaults
static const float DEFAULT_KP = 30.0f;
static const float DEFAULT_KI = 2.0f;
static const float DEFAULT_KD = 10.0f;

// PWM: stable combo on ESP32
static const uint32_t TEC_PWM_FREQ_HZ = 20000;
static const uint8_t TEC_PWM_RES_BITS = 11;

static const bool COOL_DIR_LEVEL_HIGH = false;

// ======================= UI SETTINGS =======================

static const uint32_t UI_INACTIVITY_TIMEOUT_MS = 15000;
static const uint32_t STATUS_REFRESH_MS = 300;

static const float TEMP_MIN_C = -5.0f;
static const float TEMP_MAX_C = 60.0f;

static const float TEMP_STEP_C = 0.1f;
static const float CURRENT_STEP_A = 0.1f;
static const float POWER_STEP_PERCENT = 1.0f;

// Encoder tuning
static const int ENC_STEPS_PER_DETENT = 2;

// Button filtering
static const uint32_t BTN_DEBOUNCE_MS = 80;
static const uint32_t BTN_GUARD_AFTER_MOVE_MS = 180;

// ======================= GLOBALS =======================

// ---- Display ---- (keep original working constructor + begin())
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

// ---- DS18B20 ----
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature ds18b20(&oneWire);
static DeviceAddress waterAddr;
static bool waterPresent = false;

// Non-blocking temp conversion
static bool tempConvInFlight = false;
static uint32_t tempConvStartMs = 0;
static uint32_t lastTempKickMs = 0;
// 10-bit conversion is ~187ms; use a safe margin
static const uint16_t tempConvDelayMs = 200;

// ---- Preferences ----
Preferences prefs;

// ---- Encoder ISR ----
static volatile int32_t encDelta = 0;
static volatile uint8_t encLastAB = 0;
static volatile uint32_t encLastUs = 0;

static const int8_t encTable[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0};
static int32_t encAccum = 0;

static uint32_t lastEncMoveMs = 0;

// ---- System state ----
static bool systemOn = false;
static bool profileMode = false;
static float targetConstC = 20.0f;
static float profileC[24];

static float maxPowerPercent = DEFAULT_MAX_POWER_PERCENT;
static float currentLimitA = DEFAULT_CURRENT_LIMIT_A;

// PID runtime
static float kp = DEFAULT_KP;
static float ki = DEFAULT_KI;
static float kd = DEFAULT_KD;

static float waterTempC = NAN;
static float targetTempC = NAN;

static float integralErr = 0.0f;
static float lastErr = 0.0f;
static uint32_t lastPidMs = 0;

static uint32_t profileStartMs = 0;

// Current sensing
static float acsOffsetMv[NUM_TEC] = {0};
static float tecCurrentAbsA[NUM_TEC] = {0};
static float tecDutyApplied[NUM_TEC] = {0}; // 0..1

// Fault
static bool faultTripped = false;
static String faultMsg;

// ---- UI state machine ----
enum UiState
{
    UI_STATUS = 0,
    UI_MENU,
    UI_MENU_EDIT,
    UI_PROFILE_SELECT,
    UI_PROFILE_EDIT
};
static UiState uiState = UI_STATUS;

static uint32_t lastInteractionMs = 0;
static uint32_t lastStatusRenderMs = 0;
static bool uiDirty = true;

// status partial redraw
static bool statusStaticDrawn = false;

// Menu
static int menuIndex = 0;
static const int MENU_COUNT = 7;

enum EditItem
{
    EDIT_NONE = 0,
    EDIT_TARGET_CONST,
    EDIT_CURRENT_LIMIT,
    EDIT_MAX_POWER
};
static EditItem editItem = EDIT_NONE;
static float editValue = 0.0f;

// Profile editor
static int profileSel = 0; // 0..24 (24=Back)
static int editingHour = 0;

// Button
static bool lastBtnReading = false;
static bool btnStable = false;
static uint32_t btnLastChangeMs = 0;
static uint32_t lastBtnEventMs = 0;

// ======================= HELPERS =======================

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static inline int wrapi(int v, int lo, int hi)
{
    const int span = (hi - lo + 1);
    while (v < lo)
        v += span;
    while (v > hi)
        v -= span;
    return v;
}

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return gfx->color565(r, g, b);
}

// ======================= ENCODER ISR =======================

void IRAM_ATTR onEncoderChange()
{
    uint32_t nowUs = (uint32_t)micros();
    if ((nowUs - encLastUs) < 150)
        return; // glitch filter
    encLastUs = nowUs;

    uint8_t a = (uint8_t)digitalRead(ENC_CLK_PIN);
    uint8_t b = (uint8_t)digitalRead(ENC_DT_PIN);
    uint8_t ab = (a << 1) | b;
    uint8_t idx = (encLastAB << 2) | ab;
    encDelta += encTable[idx];
    encLastAB = ab;
}

// ======================= PREFERENCES =======================

void loadSettings()
{
    prefs.begin("tecctrl", false);

    systemOn = prefs.getBool("on", false);
    profileMode = prefs.getBool("mode", false);
    targetConstC = prefs.getFloat("tconst", 20.0f);
    maxPowerPercent = prefs.getFloat("maxpwr", DEFAULT_MAX_POWER_PERCENT);
    currentLimitA = prefs.getFloat("ilim", DEFAULT_CURRENT_LIMIT_A);

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

    prefs.end();

    // Always boot OFF for safety
    systemOn = false;
}

void saveSettings()
{
    prefs.begin("tecctrl", false);
    prefs.putBool("on", systemOn);
    prefs.putBool("mode", profileMode);
    prefs.putFloat("tconst", targetConstC);
    prefs.putFloat("maxpwr", maxPowerPercent);
    prefs.putFloat("ilim", currentLimitA);
    prefs.putBytes("prof", profileC, sizeof(profileC));
    prefs.end();
}

// ======================= DS18B20 =======================

static void printDeviceAddress(const DeviceAddress &addr)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (addr[i] < 16)
            Serial.print("0");
        Serial.print(addr[i], HEX);
    }
}

void setupDs18b20()
{
    pinMode(ONE_WIRE_PIN, INPUT_PULLUP);
    delay(5);

    Serial.printf("GPIO13 idle level (expect 1): %d\n", digitalRead(ONE_WIRE_PIN));

    ds18b20.begin();
    int count = ds18b20.getDeviceCount();
    Serial.printf("DS18B20 device count: %d\n", count);

    if (count > 0 && ds18b20.getAddress(waterAddr, 0))
    {
        waterPresent = true;
        Serial.print("DS18B20[0] addr: ");
        printDeviceAddress(waterAddr);
        Serial.println();

        ds18b20.setResolution(waterAddr, 10);
        ds18b20.setWaitForConversion(false);

        ds18b20.requestTemperaturesByAddress(waterAddr);
        tempConvStartMs = millis();
        tempConvInFlight = true;
        lastTempKickMs = millis();
    }
    else
    {
        waterPresent = false;
        Serial.println("DS18B20 NOT detected. Check wiring + pull-up DATA->3V3.");
    }
}

void serviceTemperature()
{
    if (!waterPresent)
    {
        waterTempC = NAN;
        return;
    }

    uint32_t now = millis();

    if (!tempConvInFlight)
    {
        if ((now - lastTempKickMs) >= 1000)
        {
            ds18b20.requestTemperaturesByAddress(waterAddr);
            tempConvStartMs = now;
            tempConvInFlight = true;
        }
        return;
    }

    if ((now - tempConvStartMs) >= tempConvDelayMs)
    {
        float t = ds18b20.getTempC(waterAddr);
        lastTempKickMs = now;
        tempConvInFlight = false;

        waterTempC = (t == DEVICE_DISCONNECTED_C) ? NAN : t;
        uiDirty = true;
    }
}

// ======================= CURRENT SENSOR =======================

void calibrateAcsOffsets()
{
    // ensure TEC outputs off while calibrating
    for (int i = 0; i < NUM_TEC; i++)
    {
        ledcWrite(i, 0);
        digitalWrite(TEC_DIR_PINS[i], COOL_DIR_LEVEL_HIGH ? HIGH : LOW);
        tecCurrentAbsA[i] = 0.0f;
    }
    delay(300);

    for (int i = 0; i < NUM_TEC; i++)
    {
        uint32_t sum = 0;
        const int samples = 300;
        for (int k = 0; k < samples; k++)
        {
            sum += (uint32_t)analogReadMilliVolts(ACS_ADC_PINS[i]);
            delay(0);
        }
        acsOffsetMv[i] = (float)sum / (float)samples;
    }

    Serial.println("ACS offsets (mV @ ADC node):");
    for (int i = 0; i < NUM_TEC; i++)
    {
        Serial.printf("  TEC%d: %.1f mV\n", i + 1, acsOffsetMv[i]);
    }
}

void updateCurrents()
{
    for (int i = 0; i < NUM_TEC; i++)
    {
        const int samples = 16;
        uint32_t sum = 0;
        for (int k = 0; k < samples; k++)
            sum += (uint32_t)analogReadMilliVolts(ACS_ADC_PINS[i]);
        float mv = (float)sum / (float)samples;

        float ampsSigned = (mv - acsOffsetMv[i]) / ACS_SENS_MV_PER_A_AT_ADC;
        float ampsAbs = fabsf(ampsSigned);

        if (ampsAbs < CURRENT_NOISE_FLOOR_A)
            ampsAbs = 0.0f;

        // low-pass filter
        tecCurrentAbsA[i] = tecCurrentAbsA[i] * 0.8f + ampsAbs * 0.2f;

        // hard trip
        if (!faultTripped && systemOn && tecCurrentAbsA[i] > HARD_OVERCURRENT_A)
        {
            faultTripped = true;
            faultMsg = "OVERCURRENT TEC" + String(i + 1);
            systemOn = false;
            saveSettings();
            uiDirty = true;
        }
    }
}

// ======================= TEC OUTPUT =======================

void setTecOutput(int i, bool heatMode, float duty01)
{
    duty01 = clampf(duty01, 0.0f, 1.0f);

    bool dirLevel = heatMode ? !COOL_DIR_LEVEL_HIGH : COOL_DIR_LEVEL_HIGH;
    digitalWrite(TEC_DIR_PINS[i], dirLevel ? HIGH : LOW);

    const int maxDuty = (1 << TEC_PWM_RES_BITS) - 1;
    int dutyCounts = (int)lroundf(duty01 * (float)maxDuty);
    dutyCounts = constrain(dutyCounts, 0, maxDuty);

    ledcWrite(i, dutyCounts);
    tecDutyApplied[i] = duty01;
}

void allTecOff()
{
    for (int i = 0; i < NUM_TEC; i++)
        setTecOutput(i, false, 0.0f);
}

// ======================= TARGETING =======================

float computeProfileTarget(float &posHoursOut)
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

// ======================= PID =======================

void resetPid()
{
    integralErr = 0.0f;
    lastErr = 0.0f;
    lastPidMs = millis();
}

void updateControl()
{
    if (faultTripped || !systemOn || isnan(waterTempC))
    {
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
    {
        float duty = baseDuty;
        float amps = tecCurrentAbsA[i];
        if (amps > currentLimitA && amps > 0.2f)
            duty *= (currentLimitA / amps);
        setTecOutput(i, heatMode, duty);
    }
}

// ======================= UI DRAWING (LANDSCAPE STATUS) =======================

static void clearBox(int x, int y, int w, int h)
{
    gfx->fillRect(x, y, w, h, BLACK);
}

static void drawStatusStatic()
{
    statusStaticDrawn = true;

    const int W = gfx->width();
    const int H = gfx->height();

    gfx->fillScreen(BLACK);
    gfx->setTextColor(WHITE);

    // Header
    gfx->setTextSize(2);
    gfx->setCursor(8, 6);
    gfx->print("TEC Temperature");

    // Divider line
    gfx->drawLine(0, 28, W - 1, 28, rgb565(60, 60, 60));

    // Labels
    gfx->setTextSize(2);
    gfx->setCursor(8, 40);
    gfx->print("WATER");

    gfx->setCursor(8, 94);
    gfx->print("TARGET");

    // Footer help
    gfx->setTextSize(1);
    gfx->setTextColor(rgb565(200, 200, 200));
    gfx->setCursor(8, H - 10);
    gfx->print("Press knob: menu");
}

static void drawStatusDynamic()
{
    const int W = gfx->width();
    const int H = gfx->height();

    // Layout tuned for landscape (typically W=320, H=170)
    const int valueX = 120;
    const int valueW = W - valueX - 8;

    // WATER value
    clearBox(valueX, 34, valueW, 44);
    gfx->setTextColor(WHITE);
    gfx->setTextSize(4);
    gfx->setCursor(valueX, 38);
    if (!waterPresent)
    {
        gfx->setTextSize(2);
        gfx->print("SENSOR ERR");
    }
    else if (isnan(waterTempC))
    {
        gfx->print("____");
    }
    else
    {
        gfx->print(String(waterTempC, 2));
        gfx->setTextSize(2);
        gfx->print(" C");
    }

    // TARGET value
    float tgt = targetConstC;
    if (profileMode)
    {
        float ph = NAN;
        tgt = systemOn ? computeProfileTarget(ph) : profileC[0];
    }

    clearBox(valueX, 88, valueW, 44);
    gfx->setTextColor(WHITE);
    gfx->setTextSize(4);
    gfx->setCursor(valueX, 92);
    gfx->print(String(tgt, 2));
    gfx->setTextSize(2);
    gfx->print(" C");

    // Right-side status (small)
    clearBox(W - 120, 6, 112, 20);
    gfx->setTextSize(1);
    gfx->setTextColor(rgb565(200, 200, 200));
    gfx->setCursor(W - 118, 10);
    gfx->print(systemOn ? "ON" : "OFF");
    gfx->print("  ");
    gfx->print(profileMode ? "PROFILE" : "CONST");
    gfx->print("  BTN:");
    gfx->print((millis() - lastBtnEventMs) < 500 ? "OK" : "--");

    // Fault banner
    if (faultTripped)
    {
        gfx->fillRect(0, H - 18, W, 18, rgb565(160, 0, 0));
        gfx->setTextColor(WHITE);
        gfx->setTextSize(1);
        gfx->setCursor(6, H - 14);
        gfx->print("FAULT: ");
        gfx->print(faultMsg);
    }
}

// ----------------------- Menu / Edit Screens -----------------------

String menuLabel(int idx)
{
    switch (idx)
    {
    case 0:
        return String("Power: ") + (systemOn ? "ON" : "OFF");
    case 1:
        return String("Mode: ") + (profileMode ? "Profile" : "Constant");
    case 2:
        return String("Const target: ") + String(targetConstC, 1) + "C";
    case 3:
        return String("Edit profile");
    case 4:
        return String("Current limit: ") + String(currentLimitA, 1) + "A";
    case 5:
        return String("Max TEC power: ") + String(maxPowerPercent, 0) + "%";
    case 6:
        return String("Back");
    default:
        return String("");
    }
}

void drawMenuScreen()
{
    gfx->fillScreen(BLACK);
    gfx->setTextColor(WHITE);

    gfx->setTextSize(2);
    gfx->setCursor(8, 6);
    gfx->print("Menu");

    gfx->setTextSize(1);
    int y = 34;
    for (int i = 0; i < MENU_COUNT; i++)
    {
        if (i == menuIndex)
        {
            gfx->fillRect(6, y - 2, gfx->width() - 12, 14, rgb565(40, 80, 160));
            gfx->setTextColor(WHITE);
        }
        else
        {
            gfx->setTextColor(rgb565(200, 200, 200));
        }
        gfx->setCursor(10, y);
        gfx->print(menuLabel(i));
        y += 16;
    }

    gfx->setTextColor(rgb565(200, 200, 200));
    gfx->setCursor(8, gfx->height() - 10);
    gfx->print("Rotate=move  Press=select");
}

void drawMenuEditScreen()
{
    gfx->fillScreen(BLACK);
    gfx->setTextColor(WHITE);

    gfx->setTextSize(2);
    gfx->setCursor(8, 6);
    gfx->print("Adjust");

    gfx->setTextSize(2);
    gfx->setCursor(8, 44);

    switch (editItem)
    {
    case EDIT_TARGET_CONST:
        gfx->print("Const target:");
        gfx->setCursor(8, 78);
        gfx->setTextSize(3);
        gfx->print(String(editValue, 2));
        gfx->setTextSize(2);
        gfx->print(" C");
        break;
    case EDIT_CURRENT_LIMIT:
        gfx->print("Current limit:");
        gfx->setCursor(8, 78);
        gfx->setTextSize(3);
        gfx->print(String(editValue, 2));
        gfx->setTextSize(2);
        gfx->print(" A");
        break;
    case EDIT_MAX_POWER:
        gfx->print("Max power:");
        gfx->setCursor(8, 78);
        gfx->setTextSize(3);
        gfx->print(String(editValue, 0));
        gfx->setTextSize(2);
        gfx->print(" %");
        break;
    default:
        break;
    }

    gfx->setTextColor(rgb565(200, 200, 200));
    gfx->setTextSize(1);
    gfx->setCursor(8, gfx->height() - 24);
    gfx->print("Rotate=change");
    gfx->setCursor(8, gfx->height() - 10);
    gfx->print("Press=save");
}

void drawProfileSelectScreen()
{
    gfx->fillScreen(BLACK);
    gfx->setTextColor(WHITE);

    gfx->setTextSize(2);
    gfx->setCursor(8, 6);
    gfx->print("Edit Profile");

    gfx->setTextSize(1);
    gfx->setTextColor(rgb565(200, 200, 200));
    gfx->setCursor(8, 30);
    gfx->print("Rotate=hour  Press=edit/back");

    gfx->setTextColor(WHITE);
    gfx->setTextSize(3);
    gfx->setCursor(8, 60);

    if (profileSel <= 23)
    {
        gfx->print("H");
        gfx->print(profileSel);
        gfx->print(": ");
        gfx->print(String(profileC[profileSel], 1));
        gfx->print("C");
    }
    else
    {
        gfx->print("Back");
    }
}

void drawProfileEditScreen()
{
    gfx->fillScreen(BLACK);
    gfx->setTextColor(WHITE);

    gfx->setTextSize(2);
    gfx->setCursor(8, 6);
    gfx->print("Hour ");
    gfx->print(editingHour);

    gfx->setTextSize(2);
    gfx->setCursor(8, 44);
    gfx->print("Set temp:");

    gfx->setCursor(8, 78);
    gfx->setTextSize(3);
    gfx->print(String(editValue, 2));
    gfx->setTextSize(2);
    gfx->print(" C");

    gfx->setTextColor(rgb565(200, 200, 200));
    gfx->setTextSize(1);
    gfx->setCursor(8, gfx->height() - 24);
    gfx->print("Rotate=change");
    gfx->setCursor(8, gfx->height() - 10);
    gfx->print("Press=save");
}

// ======================= UI INPUT =======================

void onEncoderSteps(int steps)
{
    if (steps == 0)
        return;

    lastInteractionMs = millis();
    lastEncMoveMs = lastInteractionMs;
    uiDirty = true;

    switch (uiState)
    {
    case UI_STATUS:
        break;

    case UI_MENU:
        menuIndex = wrapi(menuIndex + steps, 0, MENU_COUNT - 1);
        break;

    case UI_MENU_EDIT:
        if (editItem == EDIT_TARGET_CONST)
        {
            editValue = clampf(editValue + (float)steps * TEMP_STEP_C, TEMP_MIN_C, TEMP_MAX_C);
        }
        else if (editItem == EDIT_CURRENT_LIMIT)
        {
            editValue = clampf(editValue + (float)steps * CURRENT_STEP_A, 0.0f, 20.0f);
        }
        else if (editItem == EDIT_MAX_POWER)
        {
            editValue = clampf(editValue + (float)steps * POWER_STEP_PERCENT, 0.0f, 100.0f);
        }
        break;

    case UI_PROFILE_SELECT:
        profileSel = wrapi(profileSel + steps, 0, 24);
        break;

    case UI_PROFILE_EDIT:
        editValue = clampf(editValue + (float)steps * TEMP_STEP_C, TEMP_MIN_C, TEMP_MAX_C);
        break;
    }
}

void onButtonPress()
{
    lastInteractionMs = millis();
    lastBtnEventMs = lastInteractionMs;
    uiDirty = true;

    switch (uiState)
    {
    case UI_STATUS:
        uiState = UI_MENU;
        menuIndex = 0;
        break;

    case UI_MENU:
        if (menuIndex == 0)
        {
            if (!faultTripped)
            {
                bool wasOn = systemOn;
                systemOn = !systemOn;

                if (!wasOn && systemOn)
                {
                    profileStartMs = millis();
                    resetPid();
                }

                if (!systemOn)
                {
                    allTecOff();
                    resetPid();
                }

                saveSettings();
            }
        }
        else if (menuIndex == 1)
        {
            profileMode = !profileMode;
            profileStartMs = millis();
            resetPid();
            saveSettings();
        }
        else if (menuIndex == 2)
        {
            editItem = EDIT_TARGET_CONST;
            editValue = targetConstC;
            uiState = UI_MENU_EDIT;
        }
        else if (menuIndex == 3)
        {
            uiState = UI_PROFILE_SELECT;
            profileSel = 0;
        }
        else if (menuIndex == 4)
        {
            editItem = EDIT_CURRENT_LIMIT;
            editValue = currentLimitA;
            uiState = UI_MENU_EDIT;
        }
        else if (menuIndex == 5)
        {
            editItem = EDIT_MAX_POWER;
            editValue = maxPowerPercent;
            uiState = UI_MENU_EDIT;
        }
        else if (menuIndex == 6)
        {
            uiState = UI_STATUS;
            statusStaticDrawn = false;
        }
        break;

    case UI_MENU_EDIT:
        if (editItem == EDIT_TARGET_CONST)
        {
            targetConstC = editValue;
            resetPid();
        }
        else if (editItem == EDIT_CURRENT_LIMIT)
        {
            currentLimitA = editValue;
        }
        else if (editItem == EDIT_MAX_POWER)
        {
            maxPowerPercent = editValue;
        }
        saveSettings();
        editItem = EDIT_NONE;
        uiState = UI_MENU;
        break;

    case UI_PROFILE_SELECT:
        if (profileSel == 24)
        {
            uiState = UI_MENU;
        }
        else
        {
            editingHour = profileSel;
            editValue = profileC[profileSel];
            uiState = UI_PROFILE_EDIT;
        }
        break;

    case UI_PROFILE_EDIT:
        profileC[editingHour] = editValue;
        saveSettings();
        uiState = UI_PROFILE_SELECT;
        break;
    }
}

void handleEncoder()
{
    int32_t d;
    noInterrupts();
    d = encDelta;
    encDelta = 0;
    interrupts();

    if (d == 0)
        return;

    encAccum += d;
    int steps = encAccum / ENC_STEPS_PER_DETENT;
    encAccum -= steps * ENC_STEPS_PER_DETENT;

    if (steps != 0)
        onEncoderSteps(steps);
}

void handleButton()
{
    bool reading = (digitalRead(ENC_SW_PIN) == LOW);
    uint32_t now = millis();

    // Guard: ignore button transitions shortly after rotation
    if ((now - lastEncMoveMs) < BTN_GUARD_AFTER_MOVE_MS)
    {
        lastBtnReading = reading;
        btnLastChangeMs = now;
        return;
    }

    if (reading != lastBtnReading)
    {
        btnLastChangeMs = now;
        lastBtnReading = reading;
    }

    if ((now - btnLastChangeMs) > BTN_DEBOUNCE_MS)
    {
        if (reading != btnStable)
        {
            btnStable = reading;
            if (btnStable)
                onButtonPress();
        }
    }
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

    // PWM channels
    for (int i = 0; i < NUM_TEC; i++)
    {
        double actual = ledcSetup(i, TEC_PWM_FREQ_HZ, TEC_PWM_RES_BITS);
        if (actual <= 0.0)
        {
            Serial.printf("LEDC setup failed ch=%d\n", i);
        }
        ledcAttachPin(TEC_PWM_PINS[i], i);
        ledcWrite(i, 0);
    }

    // ADC
    analogReadResolution(12);
    for (int i = 0; i < NUM_TEC; i++)
    {
        analogSetPinAttenuation(ACS_ADC_PINS[i], ADC_11db);
    }

    // Encoder pins
    pinMode(ENC_CLK_PIN, INPUT_PULLUP);
    pinMode(ENC_DT_PIN, INPUT_PULLUP);
    pinMode(ENC_SW_PIN, INPUT_PULLUP);

    encLastAB = ((uint8_t)digitalRead(ENC_CLK_PIN) << 1) | (uint8_t)digitalRead(ENC_DT_PIN);
    attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), onEncoderChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_DT_PIN), onEncoderChange, CHANGE);

    setupDs18b20();

    // Display (keep original working init)
    gfx->begin();
    gfx->fillScreen(BLACK);
    gfx->setTextColor(WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(8, 20);
    gfx->print("Booting...");

    Serial.printf("Display w=%d h=%d\n", gfx->width(), gfx->height());

    allTecOff();
    resetPid();
    calibrateAcsOffsets();

    lastInteractionMs = millis();
    uiDirty = true;
    statusStaticDrawn = false;
}

void loop()
{
    uint32_t now = millis();

    handleEncoder();
    handleButton();

    if (uiState != UI_STATUS && (now - lastInteractionMs) > UI_INACTIVITY_TIMEOUT_MS)
    {
        uiState = UI_STATUS;
        statusStaticDrawn = false;
        uiDirty = true;
    }

    static uint32_t lastCurrentMs = 0;
    static uint32_t lastControlMs = 0;

    serviceTemperature();

    if (now - lastCurrentMs >= 100)
    {
        lastCurrentMs = now;
        updateCurrents();
    }

    if (now - lastControlMs >= 250)
    {
        lastControlMs = now;
        updateControl();
    }

    // Render UI
    if (uiState == UI_STATUS)
    {
        if (!statusStaticDrawn)
        {
            drawStatusStatic();
            uiDirty = true;
        }
        if (uiDirty || (now - lastStatusRenderMs) >= STATUS_REFRESH_MS)
        {
            lastStatusRenderMs = now;
            drawStatusDynamic();
            uiDirty = false;
        }
    }
    else
    {
        if (uiDirty)
        {
            switch (uiState)
            {
            case UI_MENU:
                drawMenuScreen();
                break;
            case UI_MENU_EDIT:
                drawMenuEditScreen();
                break;
            case UI_PROFILE_SELECT:
                drawProfileSelectScreen();
                break;
            case UI_PROFILE_EDIT:
                drawProfileEditScreen();
                break;
            default:
                break;
            }
            uiDirty = false;
        }
    }
}
