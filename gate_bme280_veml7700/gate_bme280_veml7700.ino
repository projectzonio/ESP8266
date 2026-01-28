// @ZONIO_ID:ZONIO-ESP8266-GATE
// Zonio ESP8266 D1 Mini VEML7700 Light Sensor with Config Gate Integration
// Version: 4.1.3-GATE
// Hardware: ESP8266 D1 Mini + VEML7700
// Config: Via Zonio Config Gate
// XOR obfuscation

//------------01-----------
// BLOCK 01: INCLUDES & COMPATIBILITY LAYER


#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <stddef.h>  // For offsetof macro

// ===== VEML7700 Compatibility Layer (Adafruit 2.1.6 macros + fallback) =====
#if defined(VEML7700_GAIN_1) && defined(VEML7700_IT_100MS)
  #define VEML_API_MACRO 1
#else
  #define VEML_API_ENUM  1
#endif

#if defined(VEML_API_MACRO)
  using VGain_t = uint8_t;
  using VIt_t   = uint8_t;
  #define VGAIN_1    VEML7700_GAIN_1
  #define VGAIN_2    VEML7700_GAIN_2
  #define VGAIN_1_4  VEML7700_GAIN_1_4
  #define VGAIN_1_8  VEML7700_GAIN_1_8
  #define VIT_25     VEML7700_IT_25MS
  #define VIT_50     VEML7700_IT_50MS
  #define VIT_100    VEML7700_IT_100MS
  #define VIT_200    VEML7700_IT_200MS
  #define VIT_400    VEML7700_IT_400MS
  #define VIT_800    VEML7700_IT_800MS
  static inline void VEML_SetGain(Adafruit_VEML7700& d, VGain_t g){ d.setGain(g); }
  static inline void VEML_SetIT  (Adafruit_VEML7700& d, VIt_t   it){ d.setIntegrationTime(it); }
#else
  using VGain_t = Adafruit_VEML7700::veml7700_gain_t;
  using VIt_t   = Adafruit_VEML7700::veml7700_integrationtime_t;
  #define VGAIN_1    Adafruit_VEML7700::VEML7700_GAIN_1
  #define VGAIN_2    Adafruit_VEML7700::VEML7700_GAIN_2
  #define VGAIN_1_4  Adafruit_VEML7700::VEML7700_GAIN_1_4
  #define VGAIN_1_8  Adafruit_VEML7700::VEML7700_GAIN_1_8
  #define VIT_25     Adafruit_VEML7700::VEML7700_IT_25MS
  #define VIT_50     Adafruit_VEML7700::VEML7700_IT_50MS
  #define VIT_100    Adafruit_VEML7700::VEML7700_IT_100MS
  #define VIT_200    Adafruit_VEML7700::VEML7700_IT_200MS
  #define VIT_400    Adafruit_VEML7700::VEML7700_IT_400MS
  #define VIT_800    Adafruit_VEML7700::VEML7700_IT_800MS
  static inline void VEML_SetGain(Adafruit_VEML7700& d, VGain_t g){ d.setGain(g); }
  static inline void VEML_SetIT  (Adafruit_VEML7700& d, VIt_t   it){ d.setIntegrationTime(it); }
#endif

//------------01-----------

//------------02-----------
// BLOCK 02: CONFIGURATION & CONSTANTS


// ===== FIRMWARE VERSION =====
#define FW_VERSION              "v4.1.2-GATE-XOR"
#define DEVICE_TYPE             "BME280_veml7700_combi_sensor"

// ===== XOR ENCRYPTION KEY =====
// Key is used for E2E payload obfuscation - Now configurable via Config Gate
// Default: "MojeTajneHeslo1234567890"

// ===== CONFIG GATE SETTINGS =====
#define ENABLE_VEML_AUTORANGE   1      // Enable VEML Autorange features
#define GATE_AP_SSID            "Zonio-Gate"
#define GATE_AP_PASS            "12345678"
#define CONFIG_VALID_FLAG       0xC6A7
#define EEPROM_SIZE             512

// ===== HARDWARE PINS =====
#define I2C_SDA                 D2        // GPIO4
#define I2C_SCL                 D1        // GPIO5
#define LED_PIN                 2         // Built-in LED (GPIO2)
#define PIN_FIND_GATE           12        // D6 = GPIO12

// ===== I2C ADDRESSES =====
#define BME280_ADDR_PRIMARY     0x76
#define BME280_ADDR_SECONDARY   0x77

// ===== TIMING CONSTANTS (ms) =====
#define WIFI_CONNECT_TIMEOUT    10000     // 10 seconds
#define WIFI_CONNECT_RETRIES    20        // 20 × 500ms = 10s
#define INT_FAST                500       // 500ms sensor read
#define INT_STATUS              60000     // 60s status report
#define RECONNECT_BASE          5000      // 5s base MQTT reconnect
#define RECONNECT_MAX           300000    // 5min max reconnect interval

// ===== MQTT PAYLOAD KEYS =====
// Status keys
#define KEY_STATUS "status"
#define KEY_IP "ip"
#define KEY_RSSI "rssi"
#define KEY_DEVICE "device"
#define KEY_FIRMWARE "firmware"

// Weather keys
#define KEY_TEMP "temp"
#define KEY_HUM "hum"
#define KEY_PRES "pres"
#define KEY_LUX "lux"

// System keys
#define KEY_DEVICE_IP "deviceIp"
#define KEY_FW_VER "firmwareVersion"
#define KEY_UPTIME "uptime"
#define KEY_RSSI_SYS "rssi"
#define KEY_RECONNECT "reconnectCount"
#define KEY_FREE_HEAP "freeHeap"
#define KEY_SAMPLE_INT "sampleInterval"
#define KEY_BME280 "bme280Status"
#define KEY_VEML7700 "veml7700Status"
#define KEY_CHIP_ID "chipId"
#define KEY_FLASH_SIZE "flashSize"

// ===== VEML7700 AUTORANGE =====
// Note: autorange is now configurable via Config Gate (stored in EEPROM)
// Default: disabled - enable via Config Gate to allow autorange
#define VEML_AR_RAW_LOW         120
#define VEML_AR_RAW_HIGH        62000
#define VEML_AR_DWELL_MS        3000
#define VEML_AR_SKIP_MS         150
#define VEML_AR_FAST_SAT        60000

//------------02-----------

//------------03-----------
// BLOCK 03: DATA STRUCTURES

// ===== DEVICE CONFIGURATION (EEPROM) =====
struct DeviceConfig {
  uint16_t validFlag;                    // 0xC6A7 if valid
  char wifi_ssid[64];
  char wifi_password[64];
  char mqtt_server[64];
  uint16_t mqtt_port;
  char mqtt_user[32];
  char mqtt_pass[32];
  char mqtt_base_topic[64];              // e.g., "zonio/weather/DP1"
  char xor_key[32];                      // Encryption key
  bool veml_autorange;                   // Enable/disable VEML autorange (default: false)
  uint32_t checksum;
};

// ===== SENSOR DATA =====
struct SensorData {
  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  float lux = 0.0;
  bool bme280Working = false;
  bool vemlWorking = false;
  unsigned long lastUpdate = 0;
};

// ===== VEML AUTORANGE STEP =====
struct VemlRangeStep {
  VGain_t gain;
  VIt_t   it;
  const char* name;
};

//------------03-----------

//------------04-----------
//BLOCK 04: GLOBAL VARIABLES

// ===== CONFIGURATION =====
DeviceConfig deviceConfig;
bool isGateMode = false;

// ===== SENSOR =====
SensorData currentSensorData;
Adafruit_VEML7700 veml;
Adafruit_BME280 bme;

// ===== NETWORK =====
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
ESP8266WebServer configServer(80);

// ===== TIMING =====
unsigned long lastSensorRead = 0;
unsigned long lastStatusReport = 0;
unsigned long lastMQTTReconnectAttempt = 0;
unsigned long lastWiFiReconnectAttempt = 0;
unsigned long uptimeSeconds = 0;
unsigned long lastUptimeUpdate = 0;

// ===== RECONNECT STATE =====
unsigned long mqttReconnectInterval = RECONNECT_BASE;
int reconnectCount = 0;
int wifiReconnectAttempts = 0;
unsigned long disconnectStartTime = 0; // For 1h timeout
bool wasEverConnected = false;         // To verify stability

// ===== VEML AUTORANGE STATE =====
#if ENABLE_VEML_AUTORANGE
static const VemlRangeStep VEML_STEPS[] = {
  { VGAIN_1,   VIT_400, "G1_IT400"  },
  { VGAIN_1_4, VIT_200, "G1/4_IT200"},
  { VGAIN_1_8, VIT_100, "G1/8_IT100"}
};
static const int VEML_STEPS_COUNT = 3;
static int           g_vemlStepIdx      = 1;
static unsigned long g_vemlLastChangeMs = 0;
static unsigned long g_vemlSkipUntilMs  = 0;
#endif

// ===== DEVICE INFO =====
char chipId[16] = "";

//------------04-----------


//------------05-----------
//BLOCK 05: FORWARD DECLARATIONS (Required for Arduino IDE)

uint32_t calculateChecksum(const DeviceConfig& cfg);
bool loadConfig();
bool saveConfig();
void handleSchema();
void handleConfig();
void setupGateMode();
void loopGateMode();
void publishOnlineStatus();
void publishWeatherData();
void publishSystemStatus();
void setupNormalMode();
void loopNormalMode();
bool initSensors();
void readSensors();

// XOR encrypt/decrypt payload IN-PLACE (symmetric operation)
// Works directly on buffer to avoid RAM fragmentation
void xorPayload(char* buffer, int len) {
  int keyLen = strlen(deviceConfig.xor_key);
  if (keyLen == 0) return; // No encryption if key is empty
  
  for (int i = 0; i < len; i++) {
    buffer[i] = buffer[i] ^ deviceConfig.xor_key[i % keyLen];
  }
}


// CONFIGURATION MANAGEMENT (EEPROM)

uint32_t calculateChecksum(const DeviceConfig& cfg) {
  uint32_t sum = 0;
  const uint8_t* data = (const uint8_t*)&cfg;
  size_t len = offsetof(DeviceConfig, checksum);
  for (size_t i = 0; i < len; i++) {
    sum += data[i];
    sum = (sum << 1) | (sum >> 31);
  }
  return sum;
}

bool loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, deviceConfig);
  EEPROM.end();
  
  if (deviceConfig.validFlag != CONFIG_VALID_FLAG) {
    Serial.println("[CFG] Invalid flag");
    return false;
  }
  
  uint32_t storedChecksum = deviceConfig.checksum;
  uint32_t calculatedChecksum = calculateChecksum(deviceConfig);
  
  if (storedChecksum != calculatedChecksum) {
    Serial.println("[CFG] Checksum mismatch");
    return false;
  }
  
  Serial.println("[CFG] Loaded OK");
  return true;
}

bool saveConfig() {
  deviceConfig.validFlag = CONFIG_VALID_FLAG;
  deviceConfig.checksum = calculateChecksum(deviceConfig);
  
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(0, deviceConfig);
  bool success = EEPROM.commit();
  EEPROM.end();
  
  Serial.print("[CFG] Save: ");
  Serial.println(success ? "OK" : "FAIL");
  return success;
}

//------------05-----------

//------------06-----------
// BLOCK 06: VEML7700 AUTORANGE IMPLEMENTATION


// Note: Autorange is now runtime configurable via deviceConfig.veml_autorange
// These functions work regardless of compile-time settings

static inline void VEML_ApplyStep(Adafruit_VEML7700& dev, int idx) {
  if (idx < 0) idx = 0;
  if (idx >= VEML_STEPS_COUNT) idx = VEML_STEPS_COUNT - 1;
  
  if (idx != g_vemlStepIdx) {
    g_vemlStepIdx = idx;
    VEML_SetGain(dev, VEML_STEPS[idx].gain);
    VEML_SetIT  (dev, VEML_STEPS[idx].it);
    g_vemlLastChangeMs = millis();
    g_vemlSkipUntilMs  = g_vemlLastChangeMs + VEML_AR_SKIP_MS;
    
    Serial.print("[VEML-AR] -> ");
    Serial.print(VEML_STEPS[idx].name);
    Serial.print(" (step ");
    Serial.print(idx);
    Serial.println(")");
  }
}

static inline void VEML_AutoRangeInit(Adafruit_VEML7700& dev) {
  Serial.print("[VEML-AR] Init, start: ");
  Serial.println(VEML_STEPS[g_vemlStepIdx].name);
  VEML_ApplyStep(dev, g_vemlStepIdx);
}

static inline void VEML_AutoRangeUpdate(Adafruit_VEML7700& dev, uint16_t raw) {
  const unsigned long now = millis();
  
  // Fast saturation escape
  if (raw >= VEML_AR_FAST_SAT && g_vemlStepIdx < (VEML_STEPS_COUNT - 1)) {
    Serial.print("[VEML-AR] Fast sat RAW=");
    Serial.print(raw);
    Serial.println(" -> reduce sensitivity");
    VEML_ApplyStep(dev, g_vemlStepIdx + 1);
    return;
  }
  
  // Respect dwell time
  if (now - g_vemlLastChangeMs < VEML_AR_DWELL_MS) return;
  
  // Hysteresis around low/high
  if (raw < VEML_AR_RAW_LOW && g_vemlStepIdx > 0) {
    Serial.print("[VEML-AR] Low RAW=");
    Serial.print(raw);
    Serial.println(" -> increase sensitivity");
    VEML_ApplyStep(dev, g_vemlStepIdx - 1);
  } else if (raw > VEML_AR_RAW_HIGH && g_vemlStepIdx < (VEML_STEPS_COUNT - 1)) {
    Serial.print("[VEML-AR] High RAW=");
    Serial.print(raw);
    Serial.println(" -> reduce sensitivity");
    VEML_ApplyStep(dev, g_vemlStepIdx + 1);
  }
}

bool readVEML(uint16_t& raw, float& lux) {
  if (millis() < g_vemlSkipUntilMs) {
    return false;
  }
  raw = veml.readALS();
  lux = veml.readLux();
  return true;
}

const char* VEML_GetCurrentStepName() {
  return VEML_STEPS[g_vemlStepIdx].name;
}

int VEML_GetCurrentStepIndex() {
  return g_vemlStepIdx;
}

//------------06-----------

//------------07-----------
// BLOCK 07: SENSOR MANAGEMENT

bool initSensors() {
  Serial.println("[SENS] Initializing sensors...");
  
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  delay(50);
  
  bool success = true;
  
  // ===== BME280 INITIALIZATION =====
  Serial.print("[SENS] BME280 @0x76... ");
  if (!bme.begin(BME280_ADDR_PRIMARY)) {
    Serial.println("FAIL");
    Serial.print("[SENS] BME280 @0x77... ");
    if (!bme.begin(BME280_ADDR_SECONDARY)) {
      Serial.println("FAIL");
      Serial.println("[SENS] BME280 not found!");
      currentSensorData.bme280Working = false;
      success = false;
    } else {
      Serial.println("OK");
      currentSensorData.bme280Working = true;
      Serial.println("[SENS] BME280 initialized @0x77");
    }
  } else {
    Serial.println("OK");
    currentSensorData.bme280Working = true;
    Serial.println("[SENS] BME280 initialized @0x76");
  }
  
  // BME280 configuration
  if (currentSensorData.bme280Working) {
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,    // Temperature
                    Adafruit_BME280::SAMPLING_X16,   // Pressure
                    Adafruit_BME280::SAMPLING_X16,   // Humidity
                    Adafruit_BME280::FILTER_X16,     // IIR filter
                    Adafruit_BME280::STANDBY_MS_500);
    delay(100);
    Serial.println("[SENS] BME280 configured for precise measurements");
  }
  
  // ===== VEML7700 INITIALIZATION =====
  Serial.print("[SENS] VEML7700... ");
  int retries = 0;
  bool vemlSuccess = false;
  
  while (!vemlSuccess && retries < 3) {
    retries++;
    if (!veml.begin()) {
      Serial.print("Attempt ");
      Serial.print(retries);
      Serial.print("/3... ");
      delay(200);
    } else {
      vemlSuccess = true;
      currentSensorData.vemlWorking = true;
      Serial.println("OK");
      
      // Configure VEML based on autorange setting from config
      if (deviceConfig.veml_autorange) {
        VEML_AutoRangeInit(veml);
        Serial.println("[VEML] Autorange ENABLED (from config)");
      } else {
        VEML_SetGain(veml, VGAIN_1_8);
        VEML_SetIT(veml, VIT_100);
        Serial.println("[VEML] Autorange DISABLED - Fixed G1/8 IT100ms");
      }
      delay(150);
    }
  }
  
  if (!vemlSuccess) {
    Serial.println("FAIL");
    Serial.println("[SENS] VEML7700 not available!");
    currentSensorData.vemlWorking = false;
    success = false;
  }
  
  // Summary
  Serial.println("\n=== SENSOR SUMMARY ===");
  Serial.print("BME280 (temp/hum/press): ");
  Serial.println(currentSensorData.bme280Working ? "OK" : "FAIL");
  Serial.print("VEML7700 (light): ");
  Serial.println(currentSensorData.vemlWorking ? "OK" : "FAIL");
  Serial.print("VEML Autorange: ");
  Serial.println(deviceConfig.veml_autorange ? "ENABLED" : "DISABLED");
  Serial.println("======================");
  
  return success;
}

void readSensors() {
  // ===== BME280 READING =====
  if (currentSensorData.bme280Working) {
    float temp = bme.readTemperature();
    float hum = bme.readHumidity();
    float press = bme.readPressure() / 100.0F; // Pa -> hPa
    
    // Validate BME280 data
    if (!isnan(temp) && !isnan(hum) && !isnan(press) &&
        temp > -40.0 && temp < 85.0 &&
        hum >= 0.0 && hum <= 100.0 &&
        press > 800.0 && press < 1200.0) {
      
      currentSensorData.temperature = temp;
      currentSensorData.humidity = hum;
      currentSensorData.pressure = press;
    }
  }
  
  // ===== VEML7700 READING =====
  if (currentSensorData.vemlWorking) {
    uint16_t raw;
    float lux;
    
    if (readVEML(raw, lux)) {
      bool luxOk = !isnan(lux) && lux >= 0.0f && lux <= 150000.0f;
      
      if (luxOk) {
        // Only update autorange if enabled in config
        if (deviceConfig.veml_autorange) {
          VEML_AutoRangeUpdate(veml, raw);
        }
        currentSensorData.lux = lux;
      }
    }
  }
  
  currentSensorData.lastUpdate = millis();
}

//------------07-----------

//------------08-----------
// BLOCK 08: GATE MODE (HTTP CONFIG SERVER)

void handleSchema() {
  JsonDocument doc;
  
  doc["device_type"] = DEVICE_TYPE;
  doc["fw_version"] = FW_VERSION;
  doc["chip_id"] = chipId;
  
  JsonObject config = doc["config"].to<JsonObject>();
  
  // WiFi settings
  JsonObject wifi = config["wifi"].to<JsonObject>();
  wifi["ssid"]["type"] = "text";
  wifi["ssid"]["label"] = "WiFi SSID";
  wifi["ssid"]["required"] = true;
  
  wifi["password"]["type"] = "password";
  wifi["password"]["label"] = "WiFi Password";
  wifi["password"]["required"] = true;
  
  // MQTT settings
  JsonObject mqtt = config["mqtt"].to<JsonObject>();
  mqtt["server"]["type"] = "text";
  mqtt["server"]["label"] = "MQTT Server";
  mqtt["server"]["default"] = "192.168.0.9";
  mqtt["server"]["required"] = true;
  
  mqtt["port"]["type"] = "number";
  mqtt["port"]["label"] = "MQTT Port";
  mqtt["port"]["default"] = 1883;
  
  mqtt["user"]["type"] = "text";
  mqtt["user"]["label"] = "MQTT User";
  
  mqtt["pass"]["type"] = "password";
  mqtt["pass"]["label"] = "MQTT Password";
  
  mqtt["base_topic"]["type"] = "text";
  mqtt["base_topic"]["label"] = "Base Topic";
  mqtt["base_topic"]["default"] = "zonio/weather/DP1";
  mqtt["base_topic"]["required"] = true;
  
  // VEML settings
  JsonObject veml = config["veml"].to<JsonObject>();
  veml["autorange"]["type"] = "boolean";
  veml["autorange"]["label"] = "VEML Autorange";
  veml["autorange"]["default"] = false;  // Default: disabled

  // Security settings
  JsonObject security = config["security"].to<JsonObject>();
  security["xor_key"]["type"] = "text";
  security["xor_key"]["label"] = "XOR Key";
  security["xor_key"]["default"] = "MojeTajneHeslo1234567890";
  
  String out;
  serializeJson(doc, out);
  configServer.send(200, "application/json", out);
  
  Serial.println("[GATE] Schema sent");
}

void handleConfig() {
  if (configServer.method() != HTTP_POST) {
    configServer.send(405, "text/plain", "Method Not Allowed");
    return;
  }
  
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, configServer.arg("plain"));
  
  if (error) {
    Serial.print("[GATE] JSON error: ");
    Serial.println(error.c_str());
    configServer.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }
  
  // Parse WiFi
  if (doc["wifi"].is<JsonObject>()) {
    strlcpy(deviceConfig.wifi_ssid, 
            doc["wifi"]["ssid"] | "", 
            sizeof(deviceConfig.wifi_ssid));
    strlcpy(deviceConfig.wifi_password, 
            doc["wifi"]["password"] | "", 
            sizeof(deviceConfig.wifi_password));
  }
  
  // Parse MQTT
  if (doc["mqtt"].is<JsonObject>()) {
    strlcpy(deviceConfig.mqtt_server, 
            doc["mqtt"]["server"] | "", 
            sizeof(deviceConfig.mqtt_server));
    deviceConfig.mqtt_port = doc["mqtt"]["port"] | 1883;
    strlcpy(deviceConfig.mqtt_user, 
            doc["mqtt"]["user"] | "", 
            sizeof(deviceConfig.mqtt_user));
    strlcpy(deviceConfig.mqtt_pass, 
            doc["mqtt"]["pass"] | "", 
            sizeof(deviceConfig.mqtt_pass));
    strlcpy(deviceConfig.mqtt_base_topic, 
            doc["mqtt"]["base_topic"] | "", 
            sizeof(deviceConfig.mqtt_base_topic));
  }
  
  // Parse VEML settings
  if (doc["veml"].is<JsonObject>()) {
    deviceConfig.veml_autorange = doc["veml"]["autorange"] | false;  // Default: disabled
  }

  // Parse Security settings
  if (doc["security"].is<JsonObject>()) {
    strlcpy(deviceConfig.xor_key, 
            doc["security"]["xor_key"] | "", 
            sizeof(deviceConfig.xor_key));
  }
  
  Serial.println("[GATE] Config received:");
  Serial.print("  WiFi: ");
  Serial.println(deviceConfig.wifi_ssid);
  Serial.print("  MQTT: ");
  Serial.print(deviceConfig.mqtt_server);
  Serial.print(":");
  Serial.println(deviceConfig.mqtt_port);
  Serial.print("  XOR Key: ");
  Serial.println(strlen(deviceConfig.xor_key) > 0 ? "SET" : "EMPTY");
  
  if (saveConfig()) {
    configServer.send(200, "application/json", "{\"status\":\"saved\"}");
    Serial.println("[GATE] Config saved, restarting...");
    delay(500);
    ESP.restart();
  } else {
    configServer.send(500, "application/json", "{\"status\":\"error\"}");
  }
}

void setupGateMode() {
  Serial.println("[GATE] === GATE MODE ===");
  Serial.print("[GATE] Connecting to: ");
  Serial.println(GATE_AP_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(GATE_AP_SSID, GATE_AP_PASS);
  
  // Wait for connection (non-blocking in loop)
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[GATE] Connected to Gate AP");
    Serial.print("[GATE] IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[GATE] Failed to connect to Gate AP");
    Serial.println("[GATE] Continuing anyway (will retry in loop)");
  }
  
  // Start config server
  configServer.on("/schema", HTTP_GET, handleSchema);
  configServer.on("/config", HTTP_POST, handleConfig);
  configServer.begin();
  
  Serial.println("[GATE] Config server started on port 80");
  digitalWrite(LED_PIN, HIGH); // LED off
}

void loopGateMode() {
  configServer.handleClient();
  
  // Slow blink to indicate Gate Mode
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    lastBlink = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  
  // Check WiFi connection
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck > 5000) {
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[GATE] Reconnecting to Gate AP...");
      WiFi.begin(GATE_AP_SSID, GATE_AP_PASS);
    }
  }
}

//------------09-----------
// BLOCK 09: NORMAL MODE (MQTT OPERATION)

void publishOnlineStatus() {
  if (!mqttClient.connected()) return;
  
  char msg[256];
  IPAddress ip = WiFi.localIP();
  
  snprintf(msg, sizeof(msg),
           "{\"%s\":\"online\",\"%s\":\"%d.%d.%d.%d\",\"%s\":%d,\"%s\":\"%s\",\"%s\":\"%s\"}",
           KEY_STATUS,
           KEY_IP, ip[0], ip[1], ip[2], ip[3],
           KEY_RSSI, WiFi.RSSI(),
           KEY_DEVICE, DEVICE_TYPE,
           KEY_FIRMWARE, FW_VERSION);
  
  String statusTopic = "zonio/status/" + String(chipId);
  mqttClient.publish(statusTopic.c_str(), msg, true);  // Retained
}

void publishWeatherData() {
  if (!mqttClient.connected()) return;
  
  char msg[256];
  int payloadLen = snprintf(msg, sizeof(msg), 
           "{\"%s\":%.1f,\"%s\":%.1f,\"%s\":%d,\"%s\":%.1f}", 
           KEY_TEMP, currentSensorData.temperature,
           KEY_HUM, currentSensorData.humidity,
           KEY_PRES, (int)currentSensorData.pressure,
           KEY_LUX, currentSensorData.lux);

  // XOR encrypt IN-PLACE
  xorPayload(msg, payloadLen);
  
  // Publish as BINARY data
  // Use (uint8_t*) cast and length parameter because XOR can produce 0x00 bytes
  bool published = mqttClient.publish(deviceConfig.mqtt_base_topic, (uint8_t*)msg, payloadLen);

  if (published) {
    Serial.println("[MQTT] Weather data published (XOR encrypted)");
  } else {
    Serial.println("[MQTT] Publish failed");
  }
}

void publishSystemStatus() {
  if (!mqttClient.connected()) return;
  
  char msg[512];
  IPAddress ip = WiFi.localIP();
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t flashSize = ESP.getFlashChipSize();
  
  snprintf(msg, sizeof(msg),
           "{\"%s\":\"%d.%d.%d.%d\",\"%s\":\"%s\",\"%s\":%lu,\"%s\":%d,\"%s\":%d,"
           "\"%s\":%lu,\"%s\":%lu,\"%s\":%s,\"%s\":%s,\"%s\":\"%s\",\"%s\":%lu",
           KEY_DEVICE_IP, ip[0], ip[1], ip[2], ip[3],
           KEY_FW_VER, FW_VERSION,
           KEY_UPTIME, uptimeSeconds,
           KEY_RSSI_SYS, WiFi.RSSI(),
           KEY_RECONNECT, reconnectCount,
           KEY_FREE_HEAP, freeHeap,
           KEY_SAMPLE_INT, (INT_FAST / 1000),
           KEY_BME280, (currentSensorData.bme280Working ? "true" : "false"),
           KEY_VEML7700, (currentSensorData.vemlWorking ? "true" : "false"),
           KEY_CHIP_ID, chipId,
           KEY_FLASH_SIZE, flashSize);
  
  // Add VEML autorange info
  int len = strlen(msg);
  snprintf(msg + len, sizeof(msg) - len,
           ",\"veml_autorange_enabled\":%s",
           deviceConfig.veml_autorange ? "true" : "false");
  
  if (deviceConfig.veml_autorange) {
    len = strlen(msg);
    snprintf(msg + len, sizeof(msg) - len,
             ",\"veml_step\":%d,\"veml_range\":\"%s\"",
             VEML_GetCurrentStepIndex(), VEML_GetCurrentStepName());
  }
  
  strcat(msg, "}");
  String systemTopic = "zonio/system/" + String(chipId);
  mqttClient.publish(systemTopic.c_str(), msg);
}

void setupNormalMode() {
  Serial.println("[NORM] === NORMAL MODE ===");
  Serial.print("[NORM] Connecting to WiFi: ");
  Serial.println(deviceConfig.wifi_ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(deviceConfig.wifi_ssid, deviceConfig.wifi_password);
  
  // Try to connect briefly (non-blocking style preferred, but we do a short wait here)
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(200);
    Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Fast blink
    retries++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[NORM] WiFi Connected!");
    Serial.print("[NORM] IP: ");
    Serial.println(WiFi.localIP());
    wasEverConnected = true;
    digitalWrite(LED_PIN, HIGH); // LED off
  } else {
    Serial.println("\n[NORM] Connection not established immediately. Entering Loop with Reconnect Logic...");
    // We do NOT switch to Gate Mode here anymore. 
    // The loopNormalMode will handle the "Scan Gate -> If not found -> Retry WiFi" logic.
  }
  
  mqttClient.setServer(deviceConfig.mqtt_server, deviceConfig.mqtt_port);
  mqttClient.setKeepAlive(60);
}

void loopNormalMode() {
  unsigned long now = millis();
  
  // ==========================================================
  // 1. WiFi CONNECTION & RECOVERY LOGIC
  // ==========================================================
  if (WiFi.status() != WL_CONNECTED) {
    // A) Handle Disconnect Timer
    if (disconnectStartTime == 0) {
      disconnectStartTime = now;
      Serial.println("[WiFi] Disconnected! Starting timer.");
    }
    
    // B) 1-Hour Emergency Restart
    // If we have been disconnected for >1 hour, restart to clear any potential stack/heap/hardware issues
    if (now - disconnectStartTime > 3600000) { 
       Serial.println("[System] Offline for >1h. EMERGENCY RESTART.");
       ESP.restart();
    }

    // C) Recovery Loop (Periodically try to fix connection)
    // We check every ~15s to give time for operations and avoid spamming
    if (now - lastWiFiReconnectAttempt > 15000) {  
      lastWiFiReconnectAttempt = now;
      wifiReconnectAttempts++;
      
      Serial.print("[WiFi] Recovery Loop (Attempt ");
      Serial.print(wifiReconnectAttempts);
      Serial.println(")");

      // Step 1: Scan for Zonio Gate (Config Trigger)
      // If the user wants to reconfigure the device, they will turn on the Gate.
      Serial.println("[WiFi] Scanning for Zonio Gate...");
      int n = WiFi.scanNetworks();
      bool gateFound = false;
      for (int i=0; i<n; i++) {
        if (WiFi.SSID(i) == GATE_AP_SSID) {
          gateFound = true;
          break;
        }
      }
      
      // Step 2: Decision
      if (gateFound) {
         Serial.println("[WiFi] Zonio Gate FOUND! Switching to Gate Mode.");
         isGateMode = true;
         setupGateMode();
         return; // Exit Normal Mode Loop
      } else {
         // Step 3: Standard Reconnect
         Serial.print("[WiFi] Gate not found. Retrying Configured WiFi: ");
         Serial.println(deviceConfig.wifi_ssid);
         WiFi.begin(deviceConfig.wifi_ssid, deviceConfig.wifi_password);
         // Note: WiFi.begin is non-blocking, but we need to give it time to associate.
         // We won't block here with a delay loop; we'll let the main loop cycle.
         // If it connects, the next loop pass will catch 'WiFi.status() == WL_CONNECTED'.
      }
    }
    
    // While disconnected, we skip the rest of the loop (MQTT, Sensors requiring net, etc.)
    // Blink LED slowly to indicate "Offline / Searching"
    static unsigned long lastOfflineBlink = 0;
    if (now - lastOfflineBlink > 500) {
       lastOfflineBlink = now;
       digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
    }
    return; // CRITICAL: Exit here - WiFi is not connected, skip MQTT logic
  } 
  
  // ==========================================================
  // 2. WiFi CONNECTED - RESET DISCONNECT STATE
  // ==========================================================
  // Reset disconnect info if we just reconnected
  if (disconnectStartTime != 0) {
     Serial.println("[WiFi] Reconnected!");
     disconnectStartTime = 0;
     wifiReconnectAttempts = 0;
     mqttReconnectInterval = RECONNECT_BASE; // Fast retry for MQTT
     wasEverConnected = true;
     digitalWrite(LED_PIN, HIGH); // Ensure LED is off (active low logic usually, but here HIGH=OFF based on setup)
  }
  
  // ==========================================================
  // 3. MQTT CONNECTION MANAGEMENT (WiFi is connected)
  // ==========================================================
  if (!mqttClient.connected()) {
    if (now - lastMQTTReconnectAttempt > mqttReconnectInterval) {
      lastMQTTReconnectAttempt = now;
      reconnectCount++;
      
      Serial.print("[MQTT] Connecting (attempt ");
      Serial.print(reconnectCount);
      Serial.print(")...");
      
      String clientId = "ESP8266-" + String(ESP.getChipId(), HEX);
      
      if (mqttClient.connect(clientId.c_str(), 
                             deviceConfig.mqtt_user, 
                             deviceConfig.mqtt_pass)) {
        Serial.println(" OK");
        mqttReconnectInterval = RECONNECT_BASE;  // Reset interval
        publishOnlineStatus();
        publishSystemStatus();
      } else {
        Serial.print(" FAIL rc=");
        Serial.println(mqttClient.state());
        
        // Exponential backoff
        mqttReconnectInterval *= 2;
        if (mqttReconnectInterval > RECONNECT_MAX) {
          mqttReconnectInterval = RECONNECT_MAX;
        }
        Serial.print("[MQTT] Next attempt in ");
        Serial.print(mqttReconnectInterval / 1000);
        Serial.println("s");
      }
    }
  } else {
    mqttClient.loop();
  }
  
  // ==========================================================
  // 4. SENSOR READING & PUBLISHING (WiFi connected)
  // ==========================================================
  // Sensor reading
  if (now - lastSensorRead >= INT_FAST) {
    lastSensorRead = now;
    readSensors();
    if (mqttClient.connected()) {
      publishWeatherData();
    }
  }
  
  // Status reporting
  if (now - lastStatusReport >= INT_STATUS) {
    lastStatusReport = now;
    if (mqttClient.connected()) {
      publishSystemStatus();
    }
  }
  
  // ==========================================================
  // 5. STATUS LED (WiFi connected)
  // ==========================================================
  // Fast heartbeat blink
  static unsigned long lastBlink = 0;
  if (now - lastBlink > 100) {
    lastBlink = now;
    digitalWrite(LED_PIN, LOW);
    delay(5);
    digitalWrite(LED_PIN, HIGH);
  }
}

//------------09-----------

//------------10-----------
// BLOCK 10: SETUP & MAIN LOOP

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n\n=================================================");
  Serial.println("    ZONIO VEML7700 SENSOR + CONFIG GATE");
  Serial.println("=================================================");
  Serial.print("FW Version: ");
  Serial.println(FW_VERSION);
  
  // Setup pins
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // LED ON during boot
  pinMode(PIN_FIND_GATE, INPUT_PULLUP);
  
  // Get chip ID
  snprintf(chipId, sizeof(chipId), "%08X", ESP.getChipId());
  Serial.print("Chip ID: ");
  Serial.println(chipId);
  
  // Initialize sensors (always needed)
  initSensors();
  
  // ===== BOOT DECISION LOGIC =====
  Serial.println("\n--- BOOT DECISION ---");
  
  // 1. Check Pin D6
  bool pinTriggered = (digitalRead(PIN_FIND_GATE) == LOW);
  Serial.print("Pin D6: ");
  Serial.println(pinTriggered ? "LOW (TRIGGER)" : "HIGH");
  
  if (pinTriggered) {
    Serial.println("DECISION: Pin triggered -> GATE MODE");
    isGateMode = true;
    setupGateMode();
    return;
  }
  
  // 2. Check Config validity
  bool configValid = loadConfig();
  Serial.print("Config: ");
  Serial.println(configValid ? "VALID" : "INVALID");
  
  if (!configValid) {
    Serial.println("DECISION: No config -> GATE MODE");
    isGateMode = true;
    setupGateMode();
    return;
  }
  
  // 3. Try Normal Mode (will fallback to Gate if WiFi fails)
  Serial.println("DECISION: Config valid -> TRY NORMAL MODE");
  isGateMode = false;
  setupNormalMode();
}

void loop() {
  // Update uptime
  unsigned long now = millis();
  if (now - lastUptimeUpdate >= 1000) {
    lastUptimeUpdate = now;
    uptimeSeconds++;
  }
  
  // Run mode-specific loop
  if (isGateMode) {
    loopGateMode();
  } else {
    loopNormalMode();
  }
  
  // Watchdog
  ESP.wdtFeed();
  yield();
}

//------------10-----------
// END OF FIRMWARE
//------------10-----------
