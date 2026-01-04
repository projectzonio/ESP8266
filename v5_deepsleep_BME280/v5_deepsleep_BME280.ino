//-----------01-----------
// @ZONIO_ID:ZONIO-WS-ESP8266-V5-STABLE-DEEPSLEEP-005-DEV
// MINIMAL FIX: Only change trigger time logic, keep everything else working
// Fix: Triggers use baseline+delta time instead of fresh NTP
// Timestamp: 2026-01-04
// Changes: fixed first value save in buffer to prevent forced publish
// ToDo: Fix buffer overflow when mqtt not availible and buffer is full. 
//Extreme scenario, buffer shouldnt exceed 31 stored values with 1h forced publish. But idk what you gona set, so count with that.

#define FW_VERSION "v5-stable-deepsleep-005-loop-fix"

// ===== STABILITY CONFIGURATION =====
#define DEBUG_LEVEL               3     // 0=quiet,1=errors,2=info,3=verbose
#define USE_DEEP_SLEEP            1     // 1=deep sleep, 0=delay-based
#define SAMPLE_INTERVAL_SEC       300   // 5 minutes base interval
#define TEST_INTERVAL_SEC         60    // 1 minute for testing
#define PUBLISH_EVERY_N_SAMPLES   12    // Publish every hour (12 * 5min)
#define MAX_WIFI_ATTEMPTS         3     // WiFi connection attempts
#define MAX_MQTT_ATTEMPTS         3     // MQTT connection attempts
#define NETWORK_TIMEOUT_MS        15000 // 15 second network timeout
#define USE_CSV_PAYLOAD           0     // 1=CSV, 0=JSON

// Trigger thresholds (relative to baseline)
#define TRIG_TEMP_C               1.5   // °C change threshold
#define TRIG_HUM_PCT              10.0  // %RH change threshold  
#define TRIG_PRESS_HPA            1.5   // hPa change threshold

// ---- Trend trigger (volitelně) ----
#define ENABLE_TREND_TRIGGER      0     // 0=off, 1=on
#define TREND_TEMP_C_PER_H        2.0f  // trigger při |slopeT| >= 1 °C/h

// --- Persistence throttle ---
#ifndef COMMIT_EVERY_N_SAMPLES
#define COMMIT_EVERY_N_SAMPLES 3    // zápis EEPROM jednou za 3 vzorky (≈ každých 15 min)
#endif
#ifndef EEPROM_NEAR_FULL
#define EEPROM_NEAR_FULL 31         // 0..31 (32 slotů) – při 31 už commitni vždy
#endif

// WiFi TX power configuration
#ifndef WIFI_TX_POWER_DBM
#define WIFI_TX_POWER_DBM 10.0f  // 10 dBm je úsporné; můžeš zvednout když je RSSI bídné
#endif

// MQTT buffer configuration
#ifndef PAYLOAD_SAFE_LIMIT
#define PAYLOAD_SAFE_LIMIT 460   // když máš bufferSize(512)
#endif

// WiFi/MQTT Configuration
const char* WIFI_SSID     = "ssid";
const char* WIFI_PASSWORD = "pass";
const char* MQTT_HOST     = "broker_ip";
const uint16_t MQTT_PORT  = 1883;
const char* MQTT_USER     = "";
const char* MQTT_PASS     = "";
const char* TOPIC_PREFIX  = "zn/ws";

// ===== INCLUDES =====
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <EEPROM.h>
#include <math.h>
extern "C" {
  #include "user_interface.h"
}

// ===== PIN DEFINITIONS =====
#define BME_SDA D2   // GPIO4
#define BME_SCL D1   // GPIO5
#define LED_PIN LED_BUILTIN

// ===== RTC MEMORY STRUCTURE =====
// RTC memory persists during deep sleep but not power cycles
typedef struct {
  uint32_t magic;           // Validation magic number
  uint32_t sampleCount;     // Total samples taken
  uint32_t publishCount;    // Total publishes made
  float baselineTemp;       // Current baseline temperature
  float baselineHum;        // Current baseline humidity
  float baselinePres;       // Current baseline pressure
  uint32_t baselineEpoch;   // Baseline timestamp
  uint32_t baselineSampleCount; // sampleCount snapshot při baseline
  uint16_t samplesInBuffer; // Number of samples in EEPROM buffer
  uint8_t errorCount;       // Consecutive error count
  uint8_t wakeCount;        // Successful deep sleep wakes
  uint32_t crc32;           // Data integrity check
} RTCData;

#define RTC_MAGIC 0xABCD1234

// ===== EEPROM STRUCTURE =====
// EEPROM persists across power cycles
#define EEPROM_SIZE 1024
#define EEPROM_MAGIC 0x5A5A
#define EEPROM_VERSION 1

typedef struct {
  uint16_t magic;
  uint8_t version;
  float samples[32][3];     // Up to 32 samples * 3 values (t,h,p)
  uint32_t timestamps[32];  // Corresponding timestamps
  uint16_t count;           // Number of stored samples
  uint32_t crc32;
} EEPROMData;

// ===== GLOBALS =====
Adafruit_BME280 bme;
WiFiClient wifiClient;

//-----------01-----------

//-----------02-----------
PubSubClient mqtt(wifiClient);
WiFiUDP ntpUDP;
NTPClient ntp(ntpUDP, "pool.ntp.org", 0, 60000);
String deviceId;
RTCData rtcData = {0};
EEPROMData eepromData = {0};

// ===== LOGGING MACROS =====
#define LOGE(...) do{ if(DEBUG_LEVEL>=1){ Serial.printf("[E] "); Serial.printf(__VA_ARGS__); Serial.println(); } }while(0)
#define LOGI(...) do{ if(DEBUG_LEVEL>=2){ Serial.printf("[I] "); Serial.printf(__VA_ARGS__); Serial.println(); } }while(0)
#define LOGV(...) do{ if(DEBUG_LEVEL>=3){ Serial.printf("[V] "); Serial.printf(__VA_ARGS__); Serial.println(); } }while(0)

// ===== CRC32 CALCULATION =====
uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) bit = !bit;
      crc <<= 1;
      if (bit) crc ^= 0x04c11db7;
    }
  }
  return crc;
}

// ===== RTC MEMORY FUNCTIONS =====
bool saveRTCData() {
  static uint8_t tmp[sizeof(RTCData)] = {0};
  memset(tmp, 0, sizeof(tmp));
  memcpy(tmp, &rtcData, sizeof(RTCData) - 4);
  rtcData.crc32 = calculateCRC32(tmp, sizeof(RTCData) - 4);
  bool result = ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtcData, sizeof(RTCData));
  LOGV("RTC data saved: %s", result ? "OK" : "FAIL");
  return result;
}

bool loadRTCData() {
  if (ESP.rtcUserMemoryRead(0, (uint32_t*)&rtcData, sizeof(RTCData))) {
    static uint8_t tmp[sizeof(RTCData)] = {0};
    memset(tmp, 0, sizeof(tmp));
    memcpy(tmp, &rtcData, sizeof(RTCData) - 4);
    uint32_t crcOfData = calculateCRC32(tmp, sizeof(RTCData) - 4);
    if (rtcData.magic == RTC_MAGIC && rtcData.crc32 == crcOfData) {
      LOGI("RTC data loaded: samples=%u, wakes=%u, errors=%u", 
           rtcData.sampleCount, rtcData.wakeCount, rtcData.errorCount);
      return true;
    }
  }
  
  // Initialize RTC data
  memset(&rtcData, 0, sizeof(RTCData));
  rtcData.magic = RTC_MAGIC;
  LOGI("RTC data initialized");
  return false;
}

// ===== EEPROM FUNCTIONS =====
bool saveEEPROMData() {
  eepromData.magic = EEPROM_MAGIC;
  eepromData.version = EEPROM_VERSION;

  // CRC přes nulovaný buffer (eliminuje padding problémy)
  static uint8_t tmp[sizeof(EEPROMData)] = {0};
  memset(tmp, 0, sizeof(tmp));
  memcpy(tmp, &eepromData, sizeof(EEPROMData) - 4);
  eepromData.crc32 = calculateCRC32(tmp, sizeof(EEPROMData) - 4);
  
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(0, eepromData);
  bool result = EEPROM.commit();
  EEPROM.end();
  
  LOGV("EEPROM data saved: %s", result ? "OK" : "FAIL");
  return result;
}

bool loadEEPROMData() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, eepromData);
  EEPROM.end();
  
  if (eepromData.magic == EEPROM_MAGIC && eepromData.version == EEPROM_VERSION && eepromData.count <= 32) {
    static uint8_t tmp[sizeof(EEPROMData)] = {0};
    memset(tmp, 0, sizeof(tmp));
    memcpy(tmp, &eepromData, sizeof(EEPROMData) - 4);
    uint32_t crcOfData = calculateCRC32(tmp, sizeof(EEPROMData) - 4);
    if (eepromData.crc32 == crcOfData) {
      LOGI("EEPROM data loaded: %u samples stored", eepromData.count);
      return true;
    }
  }
  
  // Initialize EEPROM data
  memset(&eepromData, 0, sizeof(EEPROMData));
  LOGI("EEPROM data initialized");
  return false;
}

//-----------02-----------

//-----------03-----------

// ===== BME280 FUNCTIONS =====
bool initBME280() {
  Wire.begin(BME_SDA, BME_SCL);
  
  // Try both common I2C addresses
  bool ok = bme.begin(0x76);
  if (!ok) ok = bme.begin(0x77);
  if (!ok) {
    LOGE("BME280 init failed on 0x76/0x77");
    return false;
  }
  
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::FILTER_OFF);
  
  LOGI("BME280 initialized");
  return true;
}

bool readSensorData(float& temp, float& hum, float& pres) {
  if (!bme.takeForcedMeasurement()) {
    LOGE("BME280 measurement failed");
    return false;
  }
  
  temp = bme.readTemperature();
  hum = bme.readHumidity();
  pres = bme.readPressure() / 100.0f;
  
  // Validate readings
  if (isnan(temp) || isnan(hum) || isnan(pres) ||
      temp < -40 || temp > 85 || hum < 0 || hum > 100 || pres < 300 || pres > 1100) {
    LOGE("Invalid sensor readings: T=%.2f H=%.1f P=%.1f", temp, hum, pres);
    return false;
  }
  
  return true;
}

// ===== NETWORK FUNCTIONS =====
bool connectWiFi() {
  WiFi.persistent(false);          // šetří flash (neukládá SSID)
  WiFi.mode(WIFI_STA);
  WiFi.setOutputPower(WIFI_TX_POWER_DBM);  // parametrizovaný TX výkon
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  uint32_t startTime = millis();
  int attempts = 0;
  
  while (WiFi.status() != WL_CONNECTED && attempts < MAX_WIFI_ATTEMPTS) {
    if (millis() - startTime > NETWORK_TIMEOUT_MS / MAX_WIFI_ATTEMPTS) {
      attempts++;
      startTime = millis();
      LOGV("WiFi attempt %d failed", attempts);
      WiFi.disconnect();
      delay(1000);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
    delay(100);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    LOGI("WiFi connected: %s (RSSI: %d dBm)", 
         WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
  }
  
  LOGE("WiFi connection failed after %d attempts", MAX_WIFI_ATTEMPTS);
  return false;
}

bool connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512);         // větší MQTT buffer pro delší batch
  mqtt.setSocketTimeout(5);        // rychlejší fail při bídné síti
  
  for (int attempt = 0; attempt < MAX_MQTT_ATTEMPTS; attempt++) {
    String clientId = deviceId + "-" + String(millis() & 0xFFFF);
    
    bool connected;
    if (strlen(MQTT_USER) > 0) {
      connected = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS);
    } else {
      connected = mqtt.connect(clientId.c_str());
    }
    
    if (connected) {
      LOGI("MQTT connected");
      return true;
    }
    
    LOGV("MQTT attempt %d failed: %d", attempt + 1, mqtt.state());
    delay(1000);
  }
  
  LOGE("MQTT connection failed after %d attempts", MAX_MQTT_ATTEMPTS);
  return false;
}

//-----------03-----------

//-----------04-----------

uint32_t getNTPTime() {
  ntp.begin();
  
  for (int attempt = 0; attempt < 5; attempt++) {
    if (ntp.update()) {
      uint32_t epochTime = ntp.getEpochTime();
      LOGV("NTP time: %u", epochTime);
      ntp.end(); // cleanup
      return epochTime;
    }
    delay(500);
  }
  
  LOGE("NTP sync failed");
  ntp.end(); // cleanup
  return 0;
}

// ===== FIXED NETWORK DISCONNECT =====
void disconnectNetwork() {
  // Čistě odpojit MQTT
  if (mqtt.connected()) {
    mqtt.disconnect();
    delay(100);
  }
  
  // Správné odpojení WiFi pro deep sleep
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  // NEPOUŽÍVAT WiFi.forceSleepBegin() před deep sleep!
  delay(100);
}

// ===== DATA PROCESSING =====
void storeSample(float temp, float hum, float pres, uint32_t timestamp) {
  if (eepromData.count < 32) {
    eepromData.samples[eepromData.count][0] = temp;
    eepromData.samples[eepromData.count][1] = hum;
    eepromData.samples[eepromData.count][2] = pres;
    eepromData.timestamps[eepromData.count] = timestamp;
    eepromData.count++;
    rtcData.samplesInBuffer = eepromData.count;
    
    LOGV("Sample stored: [%u] T=%.2f H=%.1f P=%.1f", 
         eepromData.count - 1, temp, hum, pres);
    
    // Pokud jsme téměř plní, zapiš ochrannně do EEPROM (wear OK – jednou za dlouho)
    if (eepromData.count >= 31) {
      saveEEPROMData();
    }
  } else {
    LOGE("Sample buffer full!");
  }
}

bool isBaselineValid() {
  return rtcData.baselineEpoch > 0;
}

void updateBaseline(float temp, float hum, float pres, uint32_t timestamp) {
  rtcData.baselineTemp = temp;
  rtcData.baselineHum = hum;
  rtcData.baselinePres = pres;
  rtcData.baselineEpoch = timestamp;
  rtcData.baselineSampleCount = rtcData.sampleCount; // pro odhad TS bez NTP
  
  LOGI("Baseline updated: T=%.2f H=%.1f P=%.1f", temp, hum, pres);
}

// Volitelná jednoduchá lineární regrese pro teplotní trend (°C/h)
static float computeSlopeTemp() {
#if ENABLE_TREND_TRIGGER
  int n = (int)eepromData.count;
  n = n < 6 ? n : 6;
  if (n < 3) return 0.0f;
  double Sx=0, Sy=0, Sxx=0, Sxy=0;
  for (int i=0;i<n;i++){
    int idx = eepromData.count - n + i;
    double x = i * (double)SAMPLE_INTERVAL_SEC;
    double y = eepromData.samples[idx][0]; // temp
    Sx += x; Sy += y; Sxx += x*x; Sxy += x*y;
  }
  double denom = n*Sxx - Sx*Sx;
  if (denom == 0) return 0.0f;
  double slopePerSec = (n*Sxy - Sx*Sy) / denom;
  return (float)(slopePerSec * 3600.0); // °C/h
#else
  return 0.0f;
#endif
}

bool isTriggered(float temp, float hum, float pres) {
  if (!isBaselineValid()) return true;
  
  float tempDiff = fabs(temp - rtcData.baselineTemp);
  float humDiff = fabs(hum - rtcData.baselineHum);
  float presDiff = fabs(pres - rtcData.baselinePres);
  float slopeT = computeSlopeTemp();
  bool slopeTrig =
#if ENABLE_TREND_TRIGGER
      fabs(slopeT) >= TREND_TEMP_C_PER_H;
#else
      false;
#endif
  bool triggered = (tempDiff >= TRIG_TEMP_C) || 
                   (humDiff  >= TRIG_HUM_PCT) || 
                   (presDiff >= TRIG_PRESS_HPA) || slopeTrig;
  
  if (triggered) {
    LOGI("Trigger: ΔT=%.2f ΔH=%.1f ΔP=%.1f slopeT=%.2f °C/h", tempDiff, humDiff, presDiff, slopeT);
  }
  
  return triggered;
}

//-----------04-----------

//-----------05-----------

// ===== PUBLISHING =====
String formatPayload(float temp, float hum, float pres, uint32_t timestamp) {
  if (USE_CSV_PAYLOAD) {
    return String(timestamp) + "," + String(temp, 2) + "," + 
           String(hum, 1) + "," + String(pres, 1);
  } else {
    return "{\"ts\":" + String(timestamp) + ",\"t\":" + String(temp, 2) + 
           ",\"h\":" + String(hum, 1) + ",\"p\":" + String(pres, 1) + "}";
  }
}

bool publishData() {
  if (eepromData.count == 0) {
    LOGV("No data to publish");
    return true;
  }

  String topic = String(TOPIC_PREFIX) + "/" + deviceId;
  uint16_t published = 0;
  String batch;

  for (uint16_t i = 0; i < eepromData.count; i++) {
    String row = formatPayload(eepromData.samples[i][0],
                               eepromData.samples[i][1],
                               eepromData.samples[i][2],
                               eepromData.timestamps[i]);
    if (batch.length() && batch.length() + 1 + row.length() > PAYLOAD_SAFE_LIMIT) {
      if (!mqtt.publish(topic.c_str(), batch.c_str(), false)) {
        LOGE("Batch publish failed");
        return false;
      }
      published += 1; // 1 batch
      batch = "";
      yield();
      delay(5);
    }
    if (batch.length()) batch += "\n";
    batch += row;
  }
  if (batch.length()) {
    if (!mqtt.publish(topic.c_str(), batch.c_str(), false)) {
      LOGE("Batch publish failed (final)");
      return false;
    }
    published += 1;
  }

  LOGI("Published in %u batch(es), %u samples", published, eepromData.count);
  rtcData.publishCount++;

  // Clear buffer a uloži jednorázově do EEPROM (wear-safe)
  eepromData.count = 0;
  rtcData.samplesInBuffer = 0;
  saveEEPROMData();

  return true;
}

// ===== FIXED DEEP SLEEP FUNCTIONS =====
void enterDeepSleep(uint32_t seconds) {
  LOGI("Preparing for deep sleep (%u seconds)", seconds);
  
  // Uložit stav do RTC
  if (!saveRTCData()) {
    LOGE("Failed to save RTC data!");
  }
  
  // Vypnout LED
  digitalWrite(LED_PIN, HIGH);
  
  // Správně odpojit síť
  disconnectNetwork();
  
  // Flush serial buffer
  Serial.flush();
  delay(100);
  
  // Kontrola maximální hodnoty (ESP8266 limit je ~71 minut)
  if (seconds > 4200) {  // 70 minut
    LOGE("Sleep time too long, limiting to 70 minutes");
    seconds = 4200;
  }
  
  // Vstup do deep sleep s RF_DISABLED pro úsporu energie
  LOGI("Entering deep sleep NOW");
  ESP.deepSleep(seconds * 1000000ULL, WAKE_RF_DISABLED);
  
  // Tento kód se nikdy nevykoná
  delay(seconds * 1000);
}

bool checkWakeReason() {
  rst_info *resetInfo = ESP.getResetInfoPtr();
  
  LOGI("=== WAKE DIAGNOSTICS ===");
  LOGI("Reset reason: %d", resetInfo->reason);
  LOGI("Reset exccause: %d", resetInfo->exccause);
  
  // Detailní výpis důvodů
  switch(resetInfo->reason) {
    case REASON_DEFAULT_RST:
      LOGI("Power-on reset");
      break;
    case REASON_WDT_RST:
      LOGI("Hardware watchdog reset");
      break;
    case REASON_EXCEPTION_RST:
      LOGI("Exception reset");
      break;
    case REASON_SOFT_WDT_RST:
      LOGI("Software watchdog reset"); 
      break;
    case REASON_SOFT_RESTART:
      LOGI("Software restart");
      break;
    case REASON_DEEP_SLEEP_AWAKE:
      LOGI("✓ DEEP SLEEP WAKE - SUCCESS!");
      rtcData.errorCount = 0;
      rtcData.wakeCount++;
      return true;
    case REASON_EXT_SYS_RST:
      LOGI("External system reset");
      break;
    default:
      LOGI("Unknown reset reason: %d", resetInfo->reason);
      break;
  }
  
  // Pokud není deep sleep wake, počítáme jako chybu
  rtcData.errorCount++;
  if (rtcData.errorCount >= 3) {
    LOGE("Multiple non-deep-sleep wakes detected!");
    LOGE("Check if D0 is connected to RST pin!");
  }
  
  return false;
}

//-----------05-----------

//-----------06-----------

// ===== MAIN FUNCTIONS =====
void setup() {
  // Rychlejší inicializace pro deep sleep zařízení
  Serial.begin(115200);
  Serial.println(); // Čistý start po garbage znacích
  delay(50);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // LED on při startu
  
  LOGI("=== %s ===", FW_VERSION);
  LOGI("** MINIMAL TRIGGER FIX VERSION **");
  
  deviceId = "ws8266-" + String(ESP.getChipId(), HEX);
  LOGI("Device ID: %s", deviceId.c_str());
  
  // PRVNÍ: Zkontroluj důvod probuzení
  bool isDeepSleepWake = checkWakeReason();
  
  // Načti persistentní data
  bool rtcOK = loadRTCData();
  bool eepromOK = loadEEPROMData();
  
  LOGI("Data loaded - RTC: %s, EEPROM: %s", 
       rtcOK ? "OK" : "INIT", eepromOK ? "OK" : "INIT");
  
  // RF zapnout až když je potřeba (pokud byl WAKE_RF_DISABLED)
  if (isDeepSleepWake) {
    WiFi.forceSleepWake();
    delay(100);
  }
  
  // Inicializuj BME280
  if (!initBME280()) {
    LOGE("BME280 init failed - will retry in 30s");
    digitalWrite(LED_PIN, HIGH); // LED off
    enterDeepSleep(30); // Krátká pauza a retry
    return;
  }
  
  digitalWrite(LED_PIN, HIGH); // LED off po úspěšné inicializaci
  
  // Spusť hlavní cyklus
  runMeasurementCycle();
}

void runMeasurementCycle() {
  float temp, hum, pres;
  
  // Zapni LED během měření
  digitalWrite(LED_PIN, LOW);
  
  bool sensorOK = readSensorData(temp, hum, pres);
  
  if (!sensorOK) {
    rtcData.errorCount++;
    LOGE("Sensor error #%d", rtcData.errorCount);
    
    digitalWrite(LED_PIN, HIGH);
    
    if (rtcData.errorCount >= 5) {
      LOGE("Too many sensor errors - full restart");
      ESP.restart();
    }
    
    // Kratší sleep při chybě
    enterDeepSleep(60); // 1 minuta
    return;
  }
  
  rtcData.sampleCount++;
  uint32_t currentTime = 0;
  
  // ORIGINAL LOGIC - keep working, just fix trigger timestamps
  bool triggered = isTriggered(temp, hum, pres);
  bool timeToPublish = (rtcData.sampleCount % PUBLISH_EVERY_N_SAMPLES) == 0;
  bool needToPublish = triggered || timeToPublish || (eepromData.count >= 30);
  
  if (needToPublish) {
    LOGI("Publishing trigger: triggered=%d, time=%d, buffer=%d", 
         triggered, timeToPublish, eepromData.count >= 30);
         
    if (connectWiFi()) {
      // MINIMAL FIX: For triggers, use baseline+delta instead of fresh NTP
      if (triggered && isBaselineValid()) {
        LOGI("TRIGGER - Using baseline+delta time to avoid graph restart");
        uint32_t deltaSamples = (rtcData.sampleCount > rtcData.baselineSampleCount)
                              ? (rtcData.sampleCount - rtcData.baselineSampleCount) 
                              : 0;
        currentTime = rtcData.baselineEpoch + deltaSamples * SAMPLE_INTERVAL_SEC;
      } else {
        // Regular publish - use NTP as before
        currentTime = getNTPTime();
        if (currentTime == 0) {
          delay(200);
          currentTime = getNTPTime();
        }
      }
      
      if (currentTime == 0) {
        uint32_t deltaSamples = (rtcData.sampleCount > rtcData.baselineSampleCount)
                              ? (rtcData.sampleCount - rtcData.baselineSampleCount) 
                              : 0;
        currentTime = rtcData.baselineEpoch + deltaSamples * SAMPLE_INTERVAL_SEC;
      }
      
      storeSample(temp, hum, pres, currentTime);

      if (connectMQTT()) {
        if (publishData()) {
          // Only update baseline for regular publishes with fresh NTP, not triggers
          // FIX: Force update baseline if it's invalid (first run), even if triggered
          bool forceUpdate = !isBaselineValid();
          if (!triggered || forceUpdate) {
            updateBaseline(temp, hum, pres, currentTime);
            if (forceUpdate) LOGI("Baseline initialized (first run)");
          } else {
            LOGI("Trigger publish - baseline not updated");
          }
          // Rychlé bliknutí LED = úspěch
          for (int i = 0; i < 3; i++) {
            digitalWrite(LED_PIN, LOW);
            delay(50);
            digitalWrite(LED_PIN, HIGH);
            delay(50);
          }
        }
      } else {
        LOGE("MQTT failed - data stored for later");
      }
    } else {
      LOGE("WiFi failed - estimating timestamp");
      if (currentTime == 0) {
        uint32_t deltaSamples = (rtcData.sampleCount > rtcData.baselineSampleCount)
                              ? (rtcData.sampleCount - rtcData.baselineSampleCount)
                              : 0;
        currentTime = rtcData.baselineEpoch + deltaSamples * SAMPLE_INTERVAL_SEC;
      }
      storeSample(temp, hum, pres, currentTime);
      saveEEPROMData();
    }
  } else {
    // Offline režim
    uint32_t deltaSamples = (rtcData.sampleCount > rtcData.baselineSampleCount)
                          ? (rtcData.sampleCount - rtcData.baselineSampleCount)
                          : 0;
    currentTime = rtcData.baselineEpoch + deltaSamples * SAMPLE_INTERVAL_SEC;
    storeSample(temp, hum, pres, currentTime);
    
    // throttlovaný zápis do EEPROM, aby se mezi probuzeními nic neztratilo
    if ((rtcData.sampleCount % COMMIT_EVERY_N_SAMPLES) == 0 || eepromData.count >= EEPROM_NEAR_FULL) {
      saveEEPROMData();   // ⚠️ garantuje persistenci mezivzorků
    }
  }
  
  LOGI("Sample %u: T=%.2f°C H=%.1f%% P=%.1fhPa (buf=%u)", 
       rtcData.sampleCount, temp, hum, pres, eepromData.count);
  
  digitalWrite(LED_PIN, HIGH); // LED off
  
  // Použij testovací interval během ladění
  uint32_t sleepTime = USE_DEEP_SLEEP ? SAMPLE_INTERVAL_SEC : SAMPLE_INTERVAL_SEC;
  
  if (USE_DEEP_SLEEP) {
    enterDeepSleep(sleepTime);
  } else {
    delay(sleepTime * 1000);
  }
}

//-----------06-----------

//-----------07-----------

void loop() {
  // Nemělo by se sem dostat pokud deep sleep funguje
  LOGE("ERROR: Reached loop() - deep sleep failed!");
  LOGE("Check hardware: D0 -> RST connection");
  LOGE("Expected: D0 pin connected to RST pin with wire");
  
  // Blikání LED = chyba
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
  }
  
  // Nouzové opatření - restart
  delay(5000);
  ESP.restart();
}
//-----------07-----------
