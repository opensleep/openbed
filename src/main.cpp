/*
  ESP32 TEC Controller — NETWORK CONTROL VERSION (no dial input)

  Robust display init changes vs last version:
  - TFT SPI speed reduced to 8 MHz (much more stable)
  - Removed startWrite()/endWrite() transactions
  - Forces a clear + basic text immediately after begin()
  - Keeps clean stats-only screen (Water, Target, SSID, IP)
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

// Cytron MD13S control pins (PWM + DIR)
static const int TEC_PWM_PINS[NUM_TEC] = {25, 27, 32, 16};
static const int TEC_DIR_PINS[NUM_TEC] = {26, 14, 33, 17};

// ACS712 analog pins (ADC1)
static const int ACS_ADC_PINS[NUM_TEC] = {34, 35, 36, 39};

// DS18B20
static const int ONE_WIRE_PIN = 13;

// TFT SPI pins
static const int TFT_SCK = 18;
static const int TFT_MOSI = 23;
static const int TFT_DC = 4;

// If CS is tied to GND, keep GFX_NOT_DEFINED.
// If you can wire CS to a GPIO (recommended), set it here (ex: 5) and wire it.
static const int TFT_CS = GFX_NOT_DEFINED;

// If you can wire RST to a GPIO (recommended), set it here (ex: 15) and wire it.
// Otherwise keep GFX_NOT_DEFINED.
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

// ======================= ELECTRICAL CALIBRATION =======================

// ACS712 20A ~100mV/A at sensor output; 10k/10k divider => 50mV/A at ADC node
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
// Permanent defaults in code (edit after testing):
static const bool TEC_DIR_INVERT_DEFAULT[NUM_TEC] = {false, false, false, false};

// ======================= GLOBALS =======================

// Display (use default SPI host by omitting VSPI param; this is more compatible)
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

// DS18B20
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature ds18b20(&oneWire);
static DeviceAddress waterAddr;
static bool waterPresent = false;

// Non-blocking temp conversion
static bool tempConvInFlight = false;
static uint32_t tempConvStartMs = 0;
static uint32_t lastTempKickMs = 0;
static const uint16_t tempConvDelayMs = 200; // 10-bit ~187ms

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
static float currentLimitA = DEFAULT_CURRENT_LIMIT_A;

static float kp = DEFAULT_KP;
static float ki = DEFAULT_KI;
static float kd = DEFAULT_KD;

static float waterTempC = NAN;
static float targetTempC = NAN;

static float integralErr = 0.0f;
static float lastErr = 0.0f;
static uint32_t lastPidMs = 0;
static uint32_t profileStartMs = 0;

// Direction invert per TEC (runtime)
static bool dirInvert[NUM_TEC] = {false, false, false, false};

// Current sensing
static float acsOffsetMv[NUM_TEC] = {0};
static float tecCurrentAbsA[NUM_TEC] = {0};
static float tecDutyApplied[NUM_TEC] = {0};

// Overcurrent debounce + ignore window around on/off
static uint8_t overCount[NUM_TEC] = {0};
static uint32_t lastPowerChangeMs = 0;

// Fault
static bool faultTripped = false;
static String faultMsg;

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
  String ssidLine;
  String ipLine;
  bool fault = false;
  String faultLine;
};
static DispCache dispLast;
static bool dispStaticDrawn = false;
static bool dispDirty = true;

// Layout (landscape 320x170)
static int W = 0, H = 0;
static const int M = 8;
static int waterValX, waterValY, waterValW, waterValH;
static int targetValX, targetValY, targetValW, targetValH;
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
  currentLimitA = prefs.getFloat("ilim", DEFAULT_CURRENT_LIMIT_A);

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
  prefs.putFloat("ilim", currentLimitA);

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

static void setupDs18b20()
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

static void serviceTemperature()
{
  if (!waterPresent)
  {
    if (!isnan(waterTempC))
      dispDirty = true;
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

    float newTemp = (t == DEVICE_DISCONNECTED_C) ? NAN : t;
    if ((isnan(newTemp) != isnan(waterTempC)) || (!isnan(newTemp) && fabsf(newTemp - waterTempC) > 0.01f))
    {
      waterTempC = newTemp;
      dispDirty = true;
    }
  }
}

// ======================= CURRENT SENSOR =======================

static void calibrateAcsOffsets()
{
  allTecOff();
  delay(300);

  for (int i = 0; i < NUM_TEC; i++)
  {
    tecCurrentAbsA[i] = 0.0f;
    overCount[i] = 0;
  }

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

static void updateCurrents()
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

    float prev = tecCurrentAbsA[i];
    tecCurrentAbsA[i] = tecCurrentAbsA[i] * 0.8f + ampsAbs * 0.2f;
    if (fabsf(tecCurrentAbsA[i] - prev) > 0.05f)
      dispDirty = true;

    bool ignoreWindow = (millis() - lastPowerChangeMs) < 800;
    bool relevantLoad = (tecDutyApplied[i] > 0.06f);

    if (!faultTripped && !ignoreWindow && relevantLoad && (systemOn || manualActive))
    {
      if (tecCurrentAbsA[i] > HARD_OVERCURRENT_A)
      {
        if (overCount[i] < 255)
          overCount[i]++;
      }
      else
      {
        overCount[i] = 0;
      }

      if (overCount[i] >= 5)
      {
        faultTripped = true;
        faultMsg = "OVERCURRENT TEC" + String(i + 1) + " (" + String(tecCurrentAbsA[i], 1) + "A)";
        systemOn = false;
        manualActive = false;
        allTecOff();
        saveSettings();
        dispDirty = true;
      }
    }
    else
    {
      overCount[i] = 0;
    }
  }
}

static void serviceFaultAutoClear()
{
  static uint32_t zeroSince = 0;

  if (!faultTripped)
  {
    zeroSince = 0;
    return;
  }
  if (systemOn || manualActive)
  {
    zeroSince = 0;
    return;
  }

  bool allZero = true;
  for (int i = 0; i < NUM_TEC; i++)
  {
    if (tecCurrentAbsA[i] > 0.05f)
    {
      allZero = false;
      break;
    }
  }

  if (allZero)
  {
    if (zeroSince == 0)
      zeroSince = millis();
    if ((millis() - zeroSince) > 2000)
    {
      faultTripped = false;
      faultMsg = "";
      zeroSince = 0;
      dispDirty = true;
    }
  }
  else
  {
    zeroSince = 0;
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
  if (faultTripped || !systemOn || isnan(waterTempC) || manualActive)
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
  {
    float duty = baseDuty;
    float amps = tecCurrentAbsA[i];
    if (amps > currentLimitA && amps > 0.2f)
      duty *= (currentLimitA / amps);
    setTecOutput(i, heatMode, duty);
  }
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

  if (faultTripped)
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

// ======================= DISPLAY (clean stats) =======================

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

static void computeLayout()
{
  W = gfx->width();
  H = gfx->height();

  waterValX = 120;
  waterValY = 22;
  waterValW = W - waterValX - M;
  waterValH = 50;

  targetValX = 120;
  targetValY = 78;
  targetValW = W - targetValX - M;
  targetValH = 50;

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
  gfx->setCursor(M, 26);
  gfx->print("WATER");

  gfx->setCursor(M, 82);
  gfx->print("TARGET");

  gfx->drawLine(M, 68, W - M, 68, rgb565(40, 40, 40));
  gfx->drawLine(M, H - 28, W - M, H - 28, rgb565(40, 40, 40));

  dispLast = DispCache();
  dispDirty = true;
}

static void drawDisplayDynamicIfNeeded()
{
  if (!dispStaticDrawn)
    drawDisplayStatic();
  if (!dispDirty)
    return;

  String waterLine;
  if (!waterPresent)
    waterLine = "SENSOR";
  else if (isnan(waterTempC))
    waterLine = "____";
  else
    waterLine = String(waterTempC, 2) + "C";

  float tgt = targetConstC;
  if (profileMode)
  {
    float ph = NAN;
    tgt = (systemOn ? computeProfileTarget(ph) : profileC[0]);
  }
  String targetLine = String(tgt, 2) + "C";

  String ssidLine;
  String ipLine;

  if (apMode)
  {
    ssidLine = String("SSID: ") + AP_SSID;
    ipLine = String("IP: 192.168.4.1");
  }
  else
  {
    String shownSsid = truncateToFit(staSSID, 26);
    ssidLine = "SSID: " + shownSsid;
    ipLine = "IP: " + staIP + "  tec-ctrl.local";
  }

  bool f = faultTripped;
  String faultLine = f ? ("FAULT: " + faultMsg) : "";

  // WATER value
  if (waterLine != dispLast.waterLine)
  {
    clearBox(waterValX, waterValY, waterValW, waterValH);
    gfx->setTextColor(WHITE);
    gfx->setTextSize(4);
    gfx->setCursor(waterValX, waterValY + 6);
    gfx->print(waterLine);
    dispLast.waterLine = waterLine;
  }

  // TARGET value
  if (targetLine != dispLast.targetLine)
  {
    clearBox(targetValX, targetValY, targetValW, targetValH);
    gfx->setTextColor(WHITE);
    gfx->setTextSize(4);
    gfx->setCursor(targetValX, targetValY + 6);
    gfx->print(targetLine);
    dispLast.targetLine = targetLine;
  }

  // SSID line
  if (ssidLine != dispLast.ssidLine)
  {
    clearBox(ssidX, ssidY, ssidW, ssidH);
    gfx->setTextSize(1);
    gfx->setTextColor(rgb565(220, 220, 220));
    gfx->setCursor(ssidX, ssidY + 2);
    gfx->print(ssidLine);
    dispLast.ssidLine = ssidLine;
  }

  // IP line
  if (ipLine != dispLast.ipLine)
  {
    clearBox(ipX, ipY, ipW, ipH);
    gfx->setTextSize(1);
    gfx->setTextColor(rgb565(220, 220, 220));
    gfx->setCursor(ipX, ipY + 2);
    gfx->print(ipLine);
    dispLast.ipLine = ipLine;
  }

  // Fault banner
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
  s += "\"water_ok\":" + String(waterPresent ? "true" : "false") + ",";
  s += "\"water\":" + (isnan(waterTempC) ? String("null") : String(waterTempC, 3)) + ",";
  s += "\"target\":" + (isnan(tgt) ? String("null") : String(tgt, 3)) + ",";
  s += "\"tconst\":" + String(targetConstC, 2) + ",";
  s += "\"ilim\":" + String(currentLimitA, 2) + ",";
  s += "\"maxp\":" + String(maxPowerPercent, 1) + ",";
  s += "\"fault\":" + String(faultTripped ? "true" : "false") + ",";
  s += "\"faultMsg\":\"" + htmlEscape(faultMsg) + "\",";
  s += "\"ip\":\"" + htmlEscape(apMode ? WiFi.softAPIP().toString() : staIP) + "\",";
  s += "\"ssid\":\"" + htmlEscape(apMode ? String(AP_SSID) : staSSID) + "\",";
  s += "\"invert_snippet\":\"" + htmlEscape(invertSnippet()) + "\",";
  s += "\"curr\":[";
  for (int i = 0; i < NUM_TEC; i++)
  {
    if (i)
      s += ",";
    s += String(tecCurrentAbsA[i], 3);
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

  float f;
  if (jsonFloat(b, "tconst", f))
  {
    targetConstC = clampf(f, -5.0f, 60.0f);
    dispDirty = true;
  }
  if (jsonFloat(b, "ilim", f))
  {
    currentLimitA = clampf(f, 0.0f, 20.0f);
  }
  if (jsonFloat(b, "maxp", f))
  {
    maxPowerPercent = clampf(f, 0.0f, 100.0f);
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
    for (int i = 0; i < NUM_TEC; i++)
      overCount[i] = 0;
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

  if (!faultTripped)
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
  s += "<h2>TEC Controller</h2>";

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
  s += "<label>Current limit (A)</label><input id='ilim' type='number' step='0.1'>";
  s += "<label>Max power (%)</label><input id='maxp' type='number' step='1'>";
  s += "<div style='margin-top:10px'><button class='btn2' onclick='saveControl()'>Apply</button></div>";
  s += "</div>";

  s += "</div></div>";

  s += "<div class='card'>";
  s += "<div><b>Polarity / Manual TEC Test</b></div>";
  s += "<div class='small'>Use this to verify each TEC. Then copy/paste the snippet into the code to make it permanent.</div>";
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
  s += "lines.push('Water: '+(st.water_ok?(st.water.toFixed(2)+' C'):'SENSOR ERR'));\n";
  s += "lines.push('Target: '+(isFinite(st.target)?st.target.toFixed(2)+' C':'----'));\n";
  s += "lines.push('I limit: '+st.ilim.toFixed(1)+' A   Max power: '+st.maxp.toFixed(0)+' %');\n";
  s += "for(let i=0;i<st.curr.length;i++) lines.push('TEC'+(i+1)+': '+st.curr[i].toFixed(2)+' A  duty '+(st.duty[i]*100).toFixed(0)+'%  inv:'+(st.inv[i]?'Y':'N'));\n";
  s += "if(st.fault) lines.push('\\nFAULT: '+st.faultMsg);\n";
  s += "return lines.join('\\n');\n";
  s += "}\n";
  s += "async function refresh(){\n";
  s += "const st=await jget('/api/state');\n";
  s += "document.getElementById('status').textContent=fmtState(st);\n";
  s += "document.getElementById('net').textContent='IP: '+st.ip+'  SSID: '+st.ssid+'  (try http://tec-ctrl.local/)';\n";
  s += "document.getElementById('mode').value=st.mode?1:0;\n";
  s += "document.getElementById('tconst').value=st.tconst.toFixed(1);\n";
  s += "document.getElementById('ilim').value=st.ilim.toFixed(1);\n";
  s += "document.getElementById('maxp').value=st.maxp.toFixed(0);\n";
  s += "for(let i=0;i<st.inv.length;i++) document.getElementById('inv'+i).checked=st.inv[i];\n";
  s += "document.getElementById('snip').textContent=st.invert_snippet;\n";
  s += "}\n";
  s += "async function setPower(v){await jpost('/api/control',{on:!!v});refresh();}\n";
  s += "async function clearFault(){await jpost('/api/fault_clear',{});refresh();}\n";
  s += "async function saveControl(){\n";
  s += "const obj={mode:document.getElementById('mode').value==='1',tconst:parseFloat(tconst.value),ilim:parseFloat(ilim.value),maxp:parseFloat(maxp.value)};\n";
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
  gfx->begin(TFT_SPI_SPEED);

  // Force a deterministic clean screen immediately.
  gfx->fillScreen(BLACK);
  gfx->setTextWrap(false);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(10, 10);
  gfx->print("Display init OK");

  delay(150);
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

  // PWM channels
  for (int i = 0; i < NUM_TEC; i++)
  {
    double actual = ledcSetup(i, TEC_PWM_FREQ_HZ, TEC_PWM_RES_BITS);
    if (actual <= 0.0)
      Serial.printf("LEDC setup failed ch=%d\n", i);
    ledcAttachPin(TEC_PWM_PINS[i], i);
    ledcWrite(i, 0);
  }

  // ADC
  analogReadResolution(12);
  for (int i = 0; i < NUM_TEC; i++)
    analogSetPinAttenuation(ACS_ADC_PINS[i], ADC_11db);

  // DS18B20
  setupDs18b20();

  // TFT
  initTFT();

  allTecOff();
  resetPid();
  calibrateAcsOffsets();
  printInvertSnippetToSerial();

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

  serviceTemperature();

  static uint32_t lastCurrentMs = 0;
  if (millis() - lastCurrentMs >= 100)
  {
    lastCurrentMs = millis();
    updateCurrents();
  }

  static uint32_t lastControlMs = 0;
  if (millis() - lastControlMs >= 250)
  {
    lastControlMs = millis();
    updateControl();
  }

  serviceManualTest();
  serviceFaultAutoClear();

  drawDisplayDynamicIfNeeded();
}
