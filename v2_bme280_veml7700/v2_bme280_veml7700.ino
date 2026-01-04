// @ZONIO_ID:ZONIO-ESP8266-121
// Zonio ESP8266 D1 Mini Meteostanice - No Display Version
// Verze: 2.3.0 - ESP8266 D1 Mini adaptation + GPT improvements + VEML Autorange
//------------01----------- 
//Configuration & Constants

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_BME280.h>

// ===== VEML7700 Compatibility Layer (Adafruit 2.1.6 macros + fallback na enum API) =====
// Detekce: když existuje makro VEML7700_GAIN_1, používáme "macro API" (v2.1.6).
#if defined(VEML7700_GAIN_1) && defined(VEML7700_IT_100MS)
  #define VEML_API_MACRO 1
#else
  #define VEML_API_ENUM  1
#endif

#if defined(VEML_API_MACRO)
// Makrové API: setGain(uint8_t), setIntegrationTime(uint8_t)
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
// Enumové API (novější knihovny) – bezpečný fallback
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

// ===== BEZPEČNOSTNÍ KONFIGURACE =====
// KRITICKÉ: Přesuň do secrets.h a přidej do .gitignore!
#ifdef USE_SECRETS_FILE
  #include "secrets.h"
#else
  // Fallback pro development (SMAŽ před produkcí!)
  #warning "Používáš hardcoded credentials! Vytvoř secrets.h"
  const char* WIFI_SSID = "ssid";
  const char* WIFI_PASSWORD = "wifi_pass";
  const char* MQTT_SERVER = "broker_ip";
  const char* MQTT_USERNAME = "mqtt_usr";
  const char* MQTT_PASSWORD = "mqtt_pass";
#endif

const uint16_t MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID_BASE = "ESP8266_D1_DP";

// ===== MQTT TÉMATA KONFIGURACE =====
// Hlavní témata
const char* TOPIC_STATUS = "zonio/status/DP";
const char* TOPIC_WEATHER = "zonio/weather/DP"; 
const char* TOPIC_SYSTEM = "zonio/system/DP";

// Podtémata pro jednotlivé senzory (připraveno pro budoucnost)
const char* TOPIC_WEATHER_TEMP = "zonio/weather/DP/temperature";
const char* TOPIC_WEATHER_HUM = "zonio/weather/DP/humidity";
const char* TOPIC_WEATHER_PRES = "zonio/weather/DP/pressure";
const char* TOPIC_WEATHER_LUX = "zonio/weather/DP/light";

const char* FIRMWARE_VERSION = "v2.3.0-ESP8266-GPT-AR";

// ===== VZORKOVACÍ INTERVALY =====
// Časování v ms - optimalizované pro ESP8266
const unsigned long INT_FAST = 500;       // 5s - rychlé vzorkování
const unsigned long INT_NORMAL = 700;    // 15s - normální
const unsigned long INT_SLOW = 1000;      // 60s - pomalé

// System intervals (ms)
const unsigned long STATUS_INT = 60000;           // Status report
const unsigned long RECONNECT_BASE = 5000;        // Base reconnect
const unsigned long RECONNECT_MAX = 300000;       // Max reconnect
const unsigned long MAX_DISCONNECT_TIME = 3600000; // 1 hodina - poté restart
const int MAX_FAILED_RECONNECTS = 50;             // Max neúspěšných pokusů

// ===== HARDWARE KONFIGURACE ESP8266 D1 MINI =====
// ESP8266 D1 Mini má pouze jeden I2C bus
#define I2C_SDA D2        // GPIO4 - SDA pin
#define I2C_SCL D1        // GPIO5 - SCL pin
#define LED_BUILTIN 2     // Built-in LED
#define HISTORY_SIZE 10   // Velikost historie pro stability detection

// ===== SENZOR ADRESY =====
#define BME280_ADDR_PRIMARY 0x76
#define BME280_ADDR_SECONDARY 0x77
#define VEML7700_ADDR 0x10

// ===== MQTT PAYLOAD KLÍČE =====
// Status payload keys
#define KEY_STATUS "status"
#define KEY_IP "ip"
#define KEY_RSSI "rssi"
#define KEY_UPTIME "uptime"
#define KEY_DEVICE "device"
#define KEY_FIRMWARE "firmware"

// Weather payload keys - podle požadavku, jednoduché názvy
#define KEY_TEMP "temp"         // teplota - 1 desetinné místo
#define KEY_HUM "hum"           // vlhkost - 1 desetinné místo
#define KEY_PRES "pres"         // tlak - celé číslo
#define KEY_LUX "lux"           // světlo - 1 desetinné místo
// Příklad payload: {"temp":23.5,"hum":43.2,"pres":989,"lux":21568.7}

// System payload keys
#define KEY_DEVICE_IP "deviceIp"
#define KEY_FW_VER "firmwareVersion"
#define KEY_UPTIME_SYS "uptime"
#define KEY_RSSI_SYS "rssi"
#define KEY_RECONNECT "reconnectCount"
#define KEY_FREE_HEAP "freeHeap"
#define KEY_SAMPLE_INT "sampleInterval"
#define KEY_BME280 "bme280Status"
#define KEY_VEML7700 "veml7700Status"
#define KEY_CHIP_ID "chipId"
#define KEY_FLASH_SIZE "flashSize"

// ===== STABILITY DETECTION =====
const float TEMP_THRESH = 0.3;    // °C/min pro stabilitu
const float HUM_THRESH = 1.5;     // %/min pro stabilitu
const float PRES_THRESH = 0.5;    // hPa/min pro stabilitu
const float LUX_THRESH = 50.0;    // lux/min pro stabilitu
const int STABILITY_SAMPLES = 6;  // Počet vzorků pro stability detection

// ===== VEML7700 AUTORANGE KONFIGURACE =====
const bool CONF_VEML_AUTORANGE = false;   // true = Autorange (Outdoor), false = Raw (Indoor)

// ===== TIMEOUTS & DELAYS =====
const unsigned long SENSOR_READ_DELAY = 100;     // Delay mezi čteními senzorů
const unsigned long I2C_INIT_DELAY = 50;         // I2C inicializace delay
const unsigned long WIFI_TIMEOUT = 20000;        // WiFi connection timeout
const unsigned long MQTT_TIMEOUT = 10000;        // MQTT connection timeout

// ===== OPERATIONAL MODES =====
enum OperationalMode {
  MODE_FAST,      // Rychlé vzorkování (5s)
  MODE_NORMAL,    // Normální vzorkování (15s)  
  MODE_SLOW       // Pomalé vzorkování (60s)
};

// ===== DEVICE INFORMATION =====
const char* DEVICE_TYPE = "ESP8266_D1_MINI";
const char* DEVICE_LOCATION = "DataPoint1";

// ===== BUFFER SIZES (pro char buffery místo String) =====
#define JSON_BUFFER_SIZE 256      // MQTT JSON payloads
#define STATUS_BUFFER_SIZE 512    // Status zprávy (větší)
#define TEMP_BUFFER_SIZE 64       // Dočasné stringy

//Configuration & Constants END
//------------01----------- 


//------------02----------- 
//VEML7700 AutoRange Implementation

// ===== FORWARD DECLARATIONS =====
// Předem deklarujeme objekty, které budou definovány v Block 2
extern Adafruit_VEML7700 veml;

// #if ENABLE_VEML_AUTORANGE <- Removed, now always compiled, controlled by runtime flag
// ===== TUNABLES (lze dát i do konfigurace) =====
#ifndef VEML_AR_RAW_LOW
  #define VEML_AR_RAW_LOW        120    // pod tímto zvyšujeme citlivost (pokud to jde)
#endif
#ifndef VEML_AR_RAW_HIGH
  #define VEML_AR_RAW_HIGH       62000  // nad tímto snižujeme citlivost
#endif
#ifndef VEML_AR_DWELL_MS
  #define VEML_AR_DWELL_MS       6000   // minimální čas setrvání v kroku
#endif
#ifndef VEML_AR_SKIP_MS
  #define VEML_AR_SKIP_MS        150    // po přepnutí ignoruj čtení
#endif
#ifndef VEML_AR_FAST_SAT
  #define VEML_AR_FAST_SAT       60000  // "okamžitá" redukce citlivosti bez dwell
#endif

// ===== AUTORANGE STRUKTURY =====
struct VemlRangeStep {
  VGain_t gain;
  VIt_t   it;
  const char* name;
};

// 3 praktické kroky: šero / den / slunce
static const VemlRangeStep VEML_STEPS[] = {
  { VGAIN_1,   VIT_400, "G1_IT400"  },  // šero (max citlivost z trojice)
  { VGAIN_1_4, VIT_200, "G1/4_IT200"},  // den
  { VGAIN_1_8, VIT_100, "G1/8_IT100"}   // slunce (nejnižší citlivost)
};
static const int VEML_STEPS_COUNT = 3;

// ===== GLOBÁLNÍ STAV AUTORANGE =====
static int           g_vemlStepIdx      = 1;   // start uprostřed (den)
static unsigned long g_vemlLastChangeMs = 0;
static unsigned long g_vemlSkipUntilMs  = 0;

// ===== AUTORANGE HELPER FUNKCE =====
static inline void VEML_ApplyStep(Adafruit_VEML7700& dev, int idx){
  if (idx < 0) idx = 0;
  if (idx >= VEML_STEPS_COUNT) idx = VEML_STEPS_COUNT - 1;
  
  if (idx != g_vemlStepIdx) {
    g_vemlStepIdx = idx;
    VEML_SetGain(dev, VEML_STEPS[idx].gain);
    VEML_SetIT  (dev, VEML_STEPS[idx].it);
    g_vemlLastChangeMs = millis();
    g_vemlSkipUntilMs  = g_vemlLastChangeMs + VEML_AR_SKIP_MS;
    
    Serial.print("[VEML-AR] Auto-range -> ");
    Serial.print(VEML_STEPS[idx].name);
    Serial.print(" (step ");
    Serial.print(idx);
    Serial.println(")");
  }
}

static inline void VEML_AutoRangeInit(Adafruit_VEML7700& dev){
  Serial.print("[VEML-AR] Inicializace autorange, start krok: ");
  Serial.println(VEML_STEPS[g_vemlStepIdx].name);
  VEML_ApplyStep(dev, g_vemlStepIdx);
}

// Zavolej to na každý validní RAW sample (po čtení)
static inline void VEML_AutoRangeUpdate(Adafruit_VEML7700& dev, uint16_t raw){
  const unsigned long now = millis();

  // Okamžitý ústup při saturaci
  if (raw >= VEML_AR_FAST_SAT && g_vemlStepIdx < (VEML_STEPS_COUNT - 1)){
    Serial.print("[VEML-AR] Fast saturace RAW=");
    Serial.print(raw);
    Serial.println(" -> snižuji citlivost");
    VEML_ApplyStep(dev, g_vemlStepIdx + 1);
    return;
  }

  // Respektuj dwell
  if (now - g_vemlLastChangeMs < VEML_AR_DWELL_MS) return;

  // Hysteréze kolem low/high
  if (raw < VEML_AR_RAW_LOW && g_vemlStepIdx > 0){
    Serial.print("[VEML-AR] Nízký RAW=");
    Serial.print(raw);
    Serial.println(" -> zvyšuji citlivost");
    VEML_ApplyStep(dev, g_vemlStepIdx - 1); // zvýšit citlivost
  } else if (raw > VEML_AR_RAW_HIGH && g_vemlStepIdx < (VEML_STEPS_COUNT - 1)){
    Serial.print("[VEML-AR] Vysoký RAW=");
    Serial.print(raw);
    Serial.println(" -> snižuji citlivost");
    VEML_ApplyStep(dev, g_vemlStepIdx + 1); // snížit citlivost
  }
}

// #else ... #endif removed. Logic merged below.

// ===== HELPER PRO ČTENÍ SENZORU =====
bool readVEML(uint16_t& raw, float& lux){
  if (CONF_VEML_AUTORANGE) {
    if (millis() < g_vemlSkipUntilMs) {
      Serial.println("[VEML-AR] Přeskakuji čtení po změně rozsahu");
      return false;   // po přepnutí ignoruj
    }
  }
  
  raw = veml.readALS();
  lux = veml.readLux();
  return true;
}

// ===== AUTORANGE STATUS PRO TELEMETRII =====
const char* VEML_GetCurrentStepName() {
  if (!CONF_VEML_AUTORANGE) return "FIXED";
  return VEML_STEPS[g_vemlStepIdx].name;
}

int VEML_GetCurrentStepIndex() {
  if (!CONF_VEML_AUTORANGE) return -1;
  return g_vemlStepIdx;
}

// VEML7700 AutoRange Implementation END
//------------02----------- 

//------------03----------- 
//Data Structures & Global Variables

// ===== OBJEKTY =====
Adafruit_VEML7700 veml;
Adafruit_BME280 bme;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ===== SENZOROVÁ DATA =====
struct SensorData {
  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  float lux = 0.0;
  bool bme280Working = false;
  bool vemlWorking = false;
  unsigned long lastUpdate = 0;
};

// ===== HISTORIE PRO STABILITY DETECTION =====
struct SensorHistory {
  float values[HISTORY_SIZE];
  int index;
  bool filled;
  unsigned long timestamps[HISTORY_SIZE];

  void init() {
    index = 0;
    filled = false;
    for (int i = 0; i < HISTORY_SIZE; i++) {
      values[i] = 0.0;
      timestamps[i] = 0;
    }
  }

  void add(float value, unsigned long timestamp) {
    values[index] = value;
    timestamps[index] = timestamp;
    index = (index + 1) % HISTORY_SIZE;
    if (index == 0) filled = true;
  }

  float getChangeRate(int sampleCount) {
    if (index == 0 && !filled) return 0.0;
    
    int count = filled ? HISTORY_SIZE : index;
    count = min(count, sampleCount);
    
    if (count < 2) return 0.0;
    
    int startIdx = (index - count + HISTORY_SIZE) % HISTORY_SIZE;
    int endIdx = (index - 1 + HISTORY_SIZE) % HISTORY_SIZE;
    
    float valueChange = fabs(values[endIdx] - values[startIdx]);
    unsigned long timeChange = timestamps[endIdx] - timestamps[startIdx];
    
    if (timeChange == 0) return 0.0;
    
    return valueChange * 60000.0 / timeChange; // změna za minutu
  }

  bool isStable(float maxChangePerMinute, int sampleCount) {
    float rate = getChangeRate(sampleCount);
    return rate <= maxChangePerMinute;
  }
};

// ===== GLOBÁLNÍ PROMĚNNÉ =====
// Senzorová data
SensorData currentSensorData;
SensorData lastSensorData;

// Historie pro stability detection (včetně pressure!)
SensorHistory tempHistory;
SensorHistory humidityHistory;
SensorHistory pressureHistory;  // Přidáno pressure do stability detection
SensorHistory luxHistory;

// Operační režim
OperationalMode currentMode = MODE_NORMAL;
OperationalMode lastMode = MODE_NORMAL;
unsigned long currentSensorInterval = INT_NORMAL;
bool stabilityDetected = false;
unsigned long stabilityStartTime = 0;

// Síťové připojení
bool wifiConnected = false;
bool mqttConnected = false;
unsigned long lastReconnectAttempt = 0;
unsigned long reconnectInterval = RECONNECT_BASE;
int reconnectCount = 0;
// Robust reconnect globals
unsigned long disconnectStartTime = 0;
int failedReconnectCount = 0;
bool wasEverConnected = false;

// Časování
unsigned long lastSensorRead = 0;
unsigned long lastStatusReport = 0;
unsigned long uptimeSeconds = 0;
unsigned long lastUptimeUpdate = 0;
unsigned long bootTime = 0;

// ESP8266 specifické informace - CHAR buffery místo String!
char chipId[16] = "";           // Dříve String chipId
uint32_t flashSize = 0;
uint32_t freeHeap = 0;

// Status LED
bool ledState = false;
unsigned long lastLedBlink = 0;
const unsigned long LED_BLINK_INTERVAL = 1000; // Blikání každou sekundu

// ===== CHAR BUFFERY PRO MQTT (místo String concatenation) =====
char g_jsonBuffer[JSON_BUFFER_SIZE];
char g_statusBuffer[STATUS_BUFFER_SIZE]; 
char g_tempBuffer[TEMP_BUFFER_SIZE];

//Data Structures & Global Variables END
//------------03-----------

//------------04-----------
//Sensor Management

// ===== FUNCTION FORWARD DECLARATIONS =====
void setOperationalMode(OperationalMode newMode, const char* reason);
const char* getOperationalModeString(OperationalMode mode);

// ===== I2C INICIALIZACE =====
bool initI2C() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // 100kHz pro stabilitu ESP8266
  delay(I2C_INIT_DELAY);
  
  Serial.println("I2C inicializován - SDA: D2, SCL: D1");
  return true;
}

// ===== SENSOR INICIALIZACE =====
bool initSensors() {
  bool success = true;
  
  Serial.println("Inicializace senzorů...");
  
  // BME280 inicializace (zkus obě adresy)
  Serial.print("BME280 test @0x76... ");
  if (!bme.begin(BME280_ADDR_PRIMARY)) {
    Serial.println("FAIL");
    Serial.print("BME280 test @0x77... ");
    if (!bme.begin(BME280_ADDR_SECONDARY)) {
      Serial.println("FAIL");
      Serial.println("BME280 nenalezen na žádné adrese!");
      currentSensorData.bme280Working = false;
      success = false;
    } else {
      Serial.println("OK");
      currentSensorData.bme280Working = true;
      Serial.println("BME280 inicializován @0x77");
    }
  } else {
    Serial.println("OK");
    currentSensorData.bme280Working = true;
    Serial.println("BME280 inicializován @0x76");
  }
  
  // BME280 konfigurace (pokud funguje)
  if (currentSensorData.bme280Working) {
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,    // Teplota
                    Adafruit_BME280::SAMPLING_X16,   // Tlak  
                    Adafruit_BME280::SAMPLING_X16,   // Vlhkost
                    Adafruit_BME280::FILTER_X16,     // IIR filtr
                    Adafruit_BME280::STANDBY_MS_500);
    delay(100);
    Serial.println("BME280 konfigurován pro přesná měření");
  }
  
  // VEML7700 inicializace (s retry logikou)
  Serial.print("VEML7700 inicializace... ");
  int vemlRetries = 0;
  bool vemlSuccess = false;
  
  while (!vemlSuccess && vemlRetries < 3) {
    vemlRetries++;
    if (!veml.begin()) {
      Serial.print("pokus ");
      Serial.print(vemlRetries);
      Serial.print("/3... ");
      delay(200);
    } else {
      vemlSuccess = true;
      currentSensorData.vemlWorking = true;
      Serial.println("OK");
      
      // VEML7700 konfigurace - autorange vs fixní
      // VEML7700 konfigurace - autorange vs fixní
      if (CONF_VEML_AUTORANGE) {
        // Nastav autorange
        VEML_AutoRangeInit(veml);
      } else {
        // Původní fixní nastavení
        VEML_SetGain(veml, VGAIN_1_8);
        VEML_SetIT  (veml, VIT_100);
        Serial.println("[VEML] Init s fixním nastavením G1/8 IT100ms");
      }
      delay(150); // settle time po konfiguraci
    }
  }
  
  if (!vemlSuccess) {
    Serial.println("FAIL");
    Serial.println("VEML7700 nedostupný!");
    currentSensorData.vemlWorking = false;
    success = false;
  }
  
  // Shrnutí inicializace
  Serial.println("\n=== SHRNUTÍ SENZORŮ ===");
  Serial.print("BME280 (teplota/vlhkost/tlak): ");
  Serial.println(currentSensorData.bme280Working ? "OK" : "CHYBA");
  Serial.print("VEML7700 (světlo): ");
  Serial.println(currentSensorData.vemlWorking ? "OK" : "CHYBA");
  if (CONF_VEML_AUTORANGE) {
    Serial.println("VEML7700 autorange: AKTIVNÍ");
  } else {
    Serial.println("VEML7700 autorange: NEAKTIVNÍ");
  }
  Serial.println("========================");
  
  return success;
}

// ===== SENSOR ČTENÍ =====
void readSensors() {
  unsigned long readStart = millis();
  
  // LED signalizace čtení
  digitalWrite(LED_BUILTIN, LOW); // Zapni LED (ESP8266 má inverted logic)
  
  Serial.println("\n=== ČTENÍ SENZORŮ ===");
  
  // BME280 čtení (pokud dostupný)
  if (currentSensorData.bme280Working) {
    float temp = bme.readTemperature();
    float hum = bme.readHumidity();
    float press = bme.readPressure() / 100.0F; // Pa -> hPa
    
    delay(SENSOR_READ_DELAY);
    
    // Validace dat BME280
    if (!isnan(temp) && !isnan(hum) && !isnan(press) && 
        temp > -40.0 && temp < 85.0 && 
        hum >= 0.0 && hum <= 100.0 && 
        press > 800.0 && press < 1200.0) {
      
      currentSensorData.temperature = temp;
      currentSensorData.humidity = hum;
      currentSensorData.pressure = press;
      
      Serial.print("BME280 - Teplota: ");
      Serial.print(currentSensorData.temperature, 1);
      Serial.print("°C, Vlhkost: ");
      Serial.print(currentSensorData.humidity, 0);
      Serial.print("%, Tlak: ");
      Serial.print(currentSensorData.pressure, 0);
      Serial.println(" hPa");
    } else {
      Serial.println("BME280 - Neplatné hodnoty!");
      // Ponechej poslední platné hodnoty
    }
  } else {
    Serial.println("BME280 - Nedostupný");
  }
  
  // VEML7700 čtení s autorange podporou
  if (currentSensorData.vemlWorking) {
    uint16_t raw;
    float lux;
    
    // Použij unifikovaný reader (autorange-aware)
    if (readVEML(raw, lux)) {
      // Validace dat VEML7700
      bool luxOk = !isnan(lux) && lux >= 0.0f && lux <= 150000.0f;
      
      if (luxOk) {
        if (CONF_VEML_AUTORANGE) {
          // Autorange update na základě RAW hodnoty
          VEML_AutoRangeUpdate(veml, raw);
        }
        
        // Ulož validní data
        currentSensorData.lux = lux;
        
        Serial.print("VEML7700 - Světlo: ");
        Serial.print(currentSensorData.lux, 0);
        Serial.print(" lux, Raw: ");
        Serial.print(raw);
        if (CONF_VEML_AUTORANGE) {
          Serial.print(", Step: ");
          Serial.print(VEML_GetCurrentStepName());
          Serial.print(" (");
          Serial.print(VEML_GetCurrentStepIndex());
          Serial.print(")");
        }
        Serial.println();
      } else {
        Serial.println("VEML7700 - Neplatné hodnoty!");
        // Ponechej poslední platné hodnoty
      }
    } else {
      // readVEML vrátil false (skip po autorange změně)
      Serial.println("VEML7700 - Přeskočeno (autorange settle)");
    }
  } else {
    Serial.println("VEML7700 - Nedostupný");
  }
  
  // Update timestamp
  currentSensorData.lastUpdate = millis();
  
  // LED signalizace konec
  digitalWrite(LED_BUILTIN, HIGH); // Vypni LED
  
  unsigned long readDuration = millis() - readStart;
  Serial.print("Čtení dokončeno za ");
  Serial.print(readDuration);
  Serial.println(" ms");
  Serial.println("====================");
}

// ===== SENSOR HISTORY UPDATE =====
void updateSensorHistories() {
  unsigned long now = millis();
  tempHistory.add(currentSensorData.temperature, now);
  humidityHistory.add(currentSensorData.humidity, now);
  pressureHistory.add(currentSensorData.pressure, now);  // Přidáno pressure!
  luxHistory.add(currentSensorData.lux, now);
}

// ===== STABILITY DETECTION (ROZŠÍŘENÉ - včetně pressure!) =====
void checkStability() {
  // Kontrola stability všech senzorů včetně pressure
  bool isTempStable = tempHistory.isStable(TEMP_THRESH, STABILITY_SAMPLES);
  bool isHumStable = humidityHistory.isStable(HUM_THRESH, STABILITY_SAMPLES);
  bool isPresStable = pressureHistory.isStable(PRES_THRESH, STABILITY_SAMPLES);  // NOVÉ!
  bool isLuxStable = luxHistory.isStable(LUX_THRESH, STABILITY_SAMPLES);
  
  // Všechny senzory musí být stabilní
  bool isCurrentlyStable = isTempStable && isHumStable && isPresStable && isLuxStable;
  
  // Detekce změny stability
  if (isCurrentlyStable && !stabilityDetected) {
    stabilityDetected = true;
    stabilityStartTime = millis();
    Serial.println("STABILITA DETEKOVÁNA (včetně tlaku)");
    
    // Přepni na pomalejší režim
    if (currentMode == MODE_FAST) {
      setOperationalMode(MODE_NORMAL, "Stabilita detekována");
    }
  } else if (!isCurrentlyStable && stabilityDetected) {
    stabilityDetected = false;
    Serial.println("ZTRÁTA STABILITY");
    
    // Přepni na rychlejší režim
    setOperationalMode(MODE_FAST, "Ztráta stability");
  }
}

// ===== OPERATIONAL MODE MANAGEMENT =====
void setOperationalMode(OperationalMode newMode, const char* reason) {
  if (currentMode != newMode) {
    lastMode = currentMode;
    currentMode = newMode;
    
    Serial.print("REŽIM: ");
    Serial.print(reason);
    Serial.print(" -> ");
    
    switch (newMode) {
      case MODE_FAST:
        currentSensorInterval = INT_FAST;
        Serial.println("FAST (5s interval)");
        break;
        
      case MODE_NORMAL:
        currentSensorInterval = INT_NORMAL;
        Serial.println("NORMAL (15s interval)");
        break;
        
      case MODE_SLOW:
        currentSensorInterval = INT_SLOW;
        Serial.println("SLOW (60s interval)");
        break;
    }
  }
}

// ===== MODE INFO =====
const char* getOperationalModeString(OperationalMode mode) {
  switch (mode) {
    case MODE_FAST: return "FAST";
    case MODE_NORMAL: return "NORMAL"; 
    case MODE_SLOW: return "SLOW";
    default: return "UNKNOWN";
  }
}

//Sensor Management END
//------------04-----------

//------------05-----------
//MQTT Communication

// ===== FORWARD DECLARATIONS =====
void emergencyRestart(const char* reason);
void publishOnlineStatus(); // Forward decl due to usage in connectMQTT

// ===== WIFI PŘIPOJENÍ =====
void connectWiFi() {
  Serial.println("WiFi připojování...");
  digitalWrite(LED_BUILTIN, LOW); // LED on
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  unsigned long startAttempt = millis();
  int attempts = 0;
  
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < WIFI_TIMEOUT) {
    delay(500);
    Serial.print(".");
    attempts++;
    
    // Blink LED během připojování
    digitalWrite(LED_BUILTIN, attempts % 2);
    
    if (attempts > 40) { // 20 sekund timeout
      break;
    }
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\nWiFi připojeno!");
    Serial.print("IP adresa: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    
    digitalWrite(LED_BUILTIN, HIGH); // LED off
  } else {
    wifiConnected = false;
    Serial.println("\nWiFi připojení selhalo!");
    digitalWrite(LED_BUILTIN, HIGH); // LED off
  }
}

// ===== MQTT PŘIPOJENÍ =====
void connectMQTT() {
  if (!wifiConnected || (millis() - lastReconnectAttempt < reconnectInterval)) {
    return;
  }
  
  digitalWrite(LED_BUILTIN, LOW); // LED on
  lastReconnectAttempt = millis();
  Serial.println("MQTT připojování...");
  reconnectCount++;
  
  // Jedinečné client ID - používáme char buffer místo String
  snprintf(g_tempBuffer, sizeof(g_tempBuffer), "%s-%08X", MQTT_CLIENT_ID_BASE, ESP.getChipId());
  
  // Will message pro status topic - char buffer místo String
  snprintf(g_jsonBuffer, sizeof(g_jsonBuffer), "{\"%s\":\"offline\"}", KEY_STATUS);
  
  if (mqttClient.connect(g_tempBuffer, MQTT_USERNAME, MQTT_PASSWORD, 
                         TOPIC_STATUS, 1, true, g_jsonBuffer)) {
    mqttConnected = true;
    Serial.println("MQTT připojeno!");
    reconnectCount = 0;
    reconnectInterval = RECONNECT_BASE;
    
    // Robust reconnect reset (bude potvrzeno v main loopu, ale pro jistotu zde)
    failedReconnectCount = 0;
    wasEverConnected = true; 
    
    // Publikuj online status
    publishOnlineStatus();
    
    digitalWrite(LED_BUILTIN, HIGH); // LED off
  } else {
    mqttConnected = false;
    failedReconnectCount++; // Počítání neúspěšných pokusů
    Serial.print("MQTT připojení selhalo, rc=");
    Serial.print(mqttClient.state());
    Serial.print(", fail info: ");
    Serial.print(failedReconnectCount);
    Serial.println("x");
    
    // Exponential backoff
    if (reconnectInterval < RECONNECT_MAX) {
      reconnectInterval *= 2;
      if (reconnectInterval > RECONNECT_MAX) {
        reconnectInterval = RECONNECT_MAX;
      }
    }
    
    digitalWrite(LED_BUILTIN, HIGH); // LED off
  }
}

// ===== MQTT PUBLIKACE - ONLINE STATUS (CHAR BUFFERY!) =====
void publishOnlineStatus() {
  if (!mqttConnected) return;
  
  IPAddress ip = WiFi.localIP();
  
  // Char buffer místo String concatenation!
  snprintf(g_statusBuffer, sizeof(g_statusBuffer),
           "{\"%s\":\"online\",\"%s\":\"%d.%d.%d.%d\",\"%s\":%d,\"%s\":%lu,\"%s\":\"%s\",\"%s\":\"%s\"}",
           KEY_STATUS,
           KEY_IP, ip[0], ip[1], ip[2], ip[3],
           KEY_RSSI, WiFi.RSSI(),
           KEY_UPTIME, uptimeSeconds,
           KEY_DEVICE, DEVICE_TYPE,
           KEY_FIRMWARE, FIRMWARE_VERSION);
  
  mqttClient.publish(TOPIC_STATUS, g_statusBuffer, true);
  Serial.println("Online status publikován");
}

// ===== MQTT PUBLIKACE - WEATHER DATA (CHAR BUFFERY!) =====
void publishWeatherData() {
  if (!mqttConnected) return;
  
  digitalWrite(LED_BUILTIN, LOW); // LED on během publikace
  
  // Char buffer místo String concatenation - efektivnější!
  snprintf(g_jsonBuffer, sizeof(g_jsonBuffer),
           "{\"%s\":%.1f,\"%s\":%.1f,\"%s\":%d,\"%s\":%.1f}",
           KEY_TEMP, currentSensorData.temperature,
           KEY_HUM, currentSensorData.humidity,
           KEY_PRES, (int)currentSensorData.pressure,
           KEY_LUX, currentSensorData.lux);
  
  bool published = mqttClient.publish(TOPIC_WEATHER, g_jsonBuffer);
  
  if (published) {
    Serial.println("Weather data publikována:");
    Serial.print("Topic: ");
    Serial.println(TOPIC_WEATHER);
    Serial.print("Payload: ");
    Serial.println(g_jsonBuffer);
  } else {
    Serial.println("Chyba při publikaci weather dat!");
  }
  
  digitalWrite(LED_BUILTIN, HIGH); // LED off
}

// ===== MQTT PUBLIKACE - SYSTEM STATUS (CHAR BUFFERY + AUTORANGE!) =====
void publishSystemStatus() {
  if (!mqttConnected) return;
  
  IPAddress ip = WiFi.localIP();
  freeHeap = ESP.getFreeHeap();
  
  // Velký buffer pro system status - více dat
  snprintf(g_statusBuffer, sizeof(g_statusBuffer),
           "{\"%s\":\"%d.%d.%d.%d\",\"%s\":\"%s\",\"%s\":%lu,\"%s\":%d,\"%s\":%d,"
           "\"%s\":%lu,\"%s\":%lu,\"%s\":%s,\"%s\":%s,\"%s\":\"%s\",\"%s\":%lu",
           KEY_DEVICE_IP, ip[0], ip[1], ip[2], ip[3],
           KEY_FW_VER, FIRMWARE_VERSION,
           KEY_UPTIME_SYS, uptimeSeconds,
           KEY_RSSI_SYS, WiFi.RSSI(),
           KEY_RECONNECT, reconnectCount,
           KEY_FREE_HEAP, (unsigned long)freeHeap,
           KEY_SAMPLE_INT, (currentSensorInterval / 1000),
           KEY_BME280, (currentSensorData.bme280Working ? "true" : "false"),
           KEY_VEML7700, (currentSensorData.vemlWorking ? "true" : "false"),
           KEY_CHIP_ID, chipId,
           KEY_FLASH_SIZE, (unsigned long)flashSize);

  // Přidej VEML autorange info
#if ENABLE_VEML_AUTORANGE
  int len = strlen(g_statusBuffer);
  snprintf(g_statusBuffer + len, sizeof(g_statusBuffer) - len,
           ",\"veml_autorange\":true,\"veml_step\":%d,\"veml_range\":\"%s\"",
           VEML_GetCurrentStepIndex(), VEML_GetCurrentStepName());
#else
  int len = strlen(g_statusBuffer);
  snprintf(g_statusBuffer + len, sizeof(g_statusBuffer) - len,
           ",\"veml_autorange\":false,\"veml_range\":\"FIXED\"");
#endif

  // Zavři JSON
  strcat(g_statusBuffer, "}");
  
  mqttClient.publish(TOPIC_SYSTEM, g_statusBuffer);
  Serial.println("System status publikován");
  
#if ENABLE_VEML_AUTORANGE
  Serial.print("VEML autorange stav: step ");
  Serial.print(VEML_GetCurrentStepIndex());
  Serial.print(" (");
  Serial.print(VEML_GetCurrentStepName());
  Serial.println(")");
#endif
}

// ===== CONNECTION MANAGEMENT =====
void checkConnections() {
  static unsigned long lastWifiCheck = 0;
  unsigned long now = millis();
  
  // WiFi kontrola každých 5 sekund
  if (now - lastWifiCheck > 5000) {
    lastWifiCheck = now;
    
    if (WiFi.status() != WL_CONNECTED) {
      if (wifiConnected) {
        Serial.println("WiFi ztraceno!");
        wifiConnected = false;
        mqttConnected = false;
      }
      
      // Pokus o znovupřipojení
      connectWiFi();
    } else {
      if (!wifiConnected) {
        wifiConnected = true;
        Serial.println("WiFi obnoveno");
        reconnectInterval = RECONNECT_BASE; // Reset MQTT interval
      }
    }
  }
  
  // MQTT kontrola
  if (wifiConnected) {
    if (mqttConnected && !mqttClient.connected()) {
      Serial.println("MQTT ztraceno!");
      mqttConnected = false;
    }
    
    if (!mqttConnected) {
      connectMQTT();
    } else {
      mqttClient.loop(); // Zpracování MQTT zpráv
    }
  }
}

// ===== ROBUST RECONNECT LOGIC (Backported from v2) =====
void checkEmergencyRestart() {
  unsigned long now = millis();
  
  // Pokud jsme už někdy byli připojeni a teď nejsme
  if (wasEverConnected && (!wifiConnected || !mqttConnected)) {
    if (disconnectStartTime == 0) {
      disconnectStartTime = now;
      Serial.println("!! Disconnect Timer Started !!");
    }
    
    // Kontrola celkového času odpojení
    if (now - disconnectStartTime > MAX_DISCONNECT_TIME) {
      emergencyRestart("Dlouhodobé odpojení (>1h)");
    }
    
    // Kontrola počtu neúspěšných pokusů
    if (failedReconnectCount > MAX_FAILED_RECONNECTS) {
      emergencyRestart("Příliš mnoho neúspěšných MQTT reconnect pokusů");
    }
  } else if (wifiConnected && mqttConnected) {
    // Spojení je OK - reset čítačů
    if (disconnectStartTime != 0) {
      Serial.println("Spojení stabilní - nulování disconnect timeru");
      disconnectStartTime = 0;
    }
    if (failedReconnectCount > 0) {
      failedReconnectCount = 0;
    }
    if (!wasEverConnected) {
       wasEverConnected = true;
    }
  }
}

// ===== STATUS LED BLINK =====
void updateStatusLed() {
  unsigned long now = millis();
  
  if (now - lastLedBlink > LED_BLINK_INTERVAL) {
    lastLedBlink = now;
    
    if (wifiConnected && mqttConnected) {
      // Vše OK - krátké bliknutí
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
      digitalWrite(LED_BUILTIN, HIGH);
    } else if (wifiConnected) {
      // WiFi OK, MQTT problém - dvojité bliknutí
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      // WiFi problém - dlouhé bliknutí
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
}

//MQTT Communication END
//------------05-----------

//------------06-----------
//Setup Function

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println();
  Serial.println("================================");
  Serial.println("=== Zonio ESP8266 D1 Mini ===");
  Serial.println("=== Meteostanice v2.3.0    ===");
  Serial.println("=== AutoRange Enhanced     ===");
  Serial.println("================================");
  
  // Boot time zaznamenání
  bootTime = millis();
  
  // LED inicializace
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // ESP8266 má inverted logic - HIGH = OFF
  
  // ESP8266 informace - CHAR BUFFER místo String!
  snprintf(chipId, sizeof(chipId), "%08X", ESP.getChipId());
  flashSize = ESP.getFlashChipSize();
  
  Serial.println("\n=== HARDWARE INFO ===");
  Serial.print("Chip ID: ");
  Serial.println(chipId);
  Serial.print("Flash velikost: ");
  Serial.print(flashSize / 1024);
  Serial.println(" KB");
  Serial.print("Free heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  Serial.print("CPU frekvence: ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println(" MHz");
  Serial.println("======================");
  
  // Bezpečnostní varování
#ifndef USE_SECRETS_FILE
  Serial.println("\n⚠️ BEZPEČNOSTNÍ VAROVÁNÍ ⚠️");
  Serial.println("Používáš hardcoded credentials!");
  Serial.println("Pro produkci vytvoř secrets.h");
  Serial.println("a definuj USE_SECRETS_FILE");
  Serial.println("================================");
#endif
  
  // Feature info s autorange
  Serial.println("\n=== AKTIVNÍ VYLEPŠENÍ ===");
#if ENABLE_VEML_AUTORANGE
  Serial.println("✓ VEML7700 autorange (3-kroky)");
  Serial.print("  - Kroky: ");
  for (int i = 0; i < VEML_STEPS_COUNT; i++) {
    Serial.print(VEML_STEPS[i].name);
    if (i < VEML_STEPS_COUNT - 1) Serial.print(", ");
  }
  Serial.println();
  Serial.print("  - RAW prahy: ");
  Serial.print(VEML_AR_RAW_LOW);
  Serial.print(" - ");
  Serial.println(VEML_AR_RAW_HIGH);
  Serial.print("  - Dwell time: ");
  Serial.print(VEML_AR_DWELL_MS);
  Serial.println(" ms");
#else
  Serial.println("✗ VEML7700 fixní rozsah");
#endif
  Serial.println("✓ Char buffery (no String)");
  Serial.println("✓ Pressure stability detection");
  Serial.println("✓ Enhanced error handling");
  Serial.println("✗ Delta publishing (deaktivováno)");
  Serial.println("============================");
  
  // Compatibility layer info
#ifdef VEML_API_MACRO
  Serial.println("\n=== VEML7700 API ===");
  Serial.println("Detekováno: Macro API (v2.1.6)");
  Serial.println("Kompatibilita: setGain(uint8_t)");
#else
  Serial.println("\n=== VEML7700 API ===");
  Serial.println("Detekováno: Enum API (novější)");
  Serial.println("Kompatibilita: setGain(enum)");
#endif
  Serial.println("==================");
  
  // I2C inicializace
  Serial.println("\n=== I2C INICIALIZACE ===");
  if (!initI2C()) {
    Serial.println("CHYBA: I2C inicializace selhala!");
    while (true) {
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
    }
  }
  
  // Senzory inicializace
  Serial.println("\n=== SENZORY INICIALIZACE ===");
  bool sensorsOk = initSensors();
  
  if (!sensorsOk) {
    Serial.println("VAROVÁNÍ: Některé senzory nejsou dostupné!");
    Serial.println("Systém bude pokračovat s dostupnými senzory.");
    
    // Rychlé blikání při problémech se senzory
    for (int i = 0; i < 6; i++) {
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
    }
  } else {
    Serial.println("Všechny senzory úspěšně inicializovány!");
  }
  
  // Historie inicializace - včetně pressure!
  Serial.println("\n=== HISTORIE INICIALIZACE ===");
  tempHistory.init();
  humidityHistory.init();
  pressureHistory.init();  // NOVÉ!
  luxHistory.init();
  Serial.println("Sensor historie inicializovány (včetně tlaku)");
  
  // Operační režim nastavení
  currentMode = MODE_NORMAL;
  currentSensorInterval = INT_NORMAL;
  Serial.print("Výchozí režim: ");
  Serial.print(getOperationalModeString(currentMode));
  Serial.print(" (");
  Serial.print(currentSensorInterval / 1000);
  Serial.println("s interval)");
  
  // WiFi připojení
  Serial.println("\n=== WIFI PŘIPOJENÍ ===");
  connectWiFi();
  
  // MQTT konfigurace
  Serial.println("\n=== MQTT KONFIGURACE ===");
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(30);
  
  Serial.print("MQTT server: ");
  Serial.print(MQTT_SERVER);
  Serial.print(":");
  Serial.println(MQTT_PORT);
  
  // MQTT témata info
  Serial.println("\n=== MQTT TÉMATA ===");
  Serial.print("Status: ");
  Serial.println(TOPIC_STATUS);
  Serial.print("Weather: ");
  Serial.println(TOPIC_WEATHER);
  Serial.print("System: ");
  Serial.println(TOPIC_SYSTEM);
  
  // Weather payload formát info
  Serial.println("\n=== WEATHER PAYLOAD FORMÁT ===");
  Serial.println("Příklad: {\"temp\":23.5,\"hum\":43.2,\"pres\":989,\"lux\":21568.7}");
  Serial.print("Klíče: ");
  Serial.print(KEY_TEMP);
  Serial.print(", ");
  Serial.print(KEY_HUM);
  Serial.print(", ");
  Serial.print(KEY_PRES);
  Serial.print(", ");
  Serial.println(KEY_LUX);
  
  // Buffer info
  Serial.println("\n=== MEMORY OPTIMALIZACE ===");
  Serial.print("JSON buffer: ");
  Serial.print(JSON_BUFFER_SIZE);
  Serial.println(" bytes");
  Serial.print("Status buffer: ");
  Serial.print(STATUS_BUFFER_SIZE);
  Serial.println(" bytes");
  Serial.print("Temp buffer: ");
  Serial.print(TEMP_BUFFER_SIZE);
  Serial.println(" bytes");
  Serial.println("Char buffery místo String objektů!");
  
  // Autorange status
#if ENABLE_VEML_AUTORANGE
  Serial.println("\n=== AUTORANGE STATUS ===");
  Serial.print("Startovní krok: ");
  Serial.print(VEML_GetCurrentStepIndex());
  Serial.print(" (");
  Serial.print(VEML_GetCurrentStepName());
  Serial.println(")");
  Serial.println("Autorange aktivní a připraven!");
#endif
  
  // Úvodní sensor čtení
  Serial.println("\n=== ÚVODNÍ ČTENÍ SENZORŮ ===");
  readSensors();
  updateSensorHistories();
  
  // Startup dokončen
  Serial.println("\n================================");
  Serial.println("=== INICIALIZACE DOKONČENA ===");
  Serial.println("=== SYSTÉM SPUŠTĚN (v2.3.0) ===");
  Serial.println("================================");
  
  // LED signalizace úspěšného startu
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
  }
  
  // Nastavení časovačů
  lastSensorRead = millis();
  lastStatusReport = millis();
  lastUptimeUpdate = millis();
  
  Serial.println("\nSystém je připraven k provozu...\n");
  
  // Výpis vylepšení po startu
  Serial.println("🚀 AKTIVNÍ GPT + AUTORANGE VYLEPŠENÍ:");
  Serial.println("  • Bezpečnější credential management");
  Serial.println("  • Memory-efficient char buffery");
  Serial.println("  • Pressure stability detection");
  Serial.println("  • Enhanced debugging & monitoring");
#if ENABLE_VEML_AUTORANGE
  Serial.println("  • VEML7700 3-step autorange (AKTIVNÍ)");
  Serial.println("  • Autorange telemetrie v MQTT");
#else
  Serial.println("  • VEML7700 autorange připraven (neaktivní)");
#endif
  Serial.println();
}

//Setup Function END
//------------06-----------

//------------07-----------
//Main Loop & Debug Functions

// ===== FUNCTION FORWARD DECLARATIONS =====
void printDetailedStatus();
void checkMemoryHealth();
void emergencyRestart(const char* reason);

void loop() {
  unsigned long now = millis();
  
  // === CONNECTION MANAGEMENT ===
  checkConnections();
  
  // === UPTIME UPDATE ===
  if (now - lastUptimeUpdate >= 1000) {
    lastUptimeUpdate = now;
    uptimeSeconds++;
  }
  
  // === STATUS LED UPDATE ===
  updateStatusLed();
  
  // === HLAVNÍ SENSOR LOOP ===
  if (now - lastSensorRead >= currentSensorInterval) {
    lastSensorRead = now;
    
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║      SENSOR CYCLE START              ║");
    Serial.println("╚═══════════════════════════════════════╝");
    
    // Aktuální stav systému
    Serial.print("Režim: ");
    Serial.print(getOperationalModeString(currentMode));
    Serial.print(" | Interval: ");
    Serial.print(currentSensorInterval / 1000);
    Serial.print("s | Uptime: ");
    Serial.print(uptimeSeconds / 3600);
    Serial.print("h ");
    Serial.print((uptimeSeconds % 3600) / 60);
    Serial.print("m ");
    Serial.print(uptimeSeconds % 60);
    Serial.println("s");
    
    // Čtení senzorů
    readSensors();
    
    // Update historie
    updateSensorHistories();
    
    // Kontrola stability (včetně pressure!)
    checkStability();
    
    // MQTT publikace weather dat - PŮVODNÍ PŘÍSTUP (bez delta checking)
    if (mqttConnected) {
      publishWeatherData();
      Serial.println("Weather data publikována (standardní režim)");
    } else {
      Serial.println("MQTT nedostupné - weather data nepublikována");
    }
    
    // Stability info s rozšířenými detaily
    Serial.print("Stabilita: ");
    if (stabilityDetected) {
      unsigned long stabDuration = (now - stabilityStartTime) / 1000;
      Serial.print("DETEKOVÁNA (");
      Serial.print(stabDuration);
      Serial.println(" sekund)");
    } else {
      Serial.println("NEDETEKOVÁNA");
    }
    
    // Změnové rychlosti pro debugging
    Serial.println("Stability rates:");
    Serial.print("  Temp: ");
    Serial.print(tempHistory.getChangeRate(STABILITY_SAMPLES), 3);
    Serial.print("°C/min (limit ");
    Serial.print(TEMP_THRESH);
    Serial.println(")");
    Serial.print("  Hum: ");
    Serial.print(humidityHistory.getChangeRate(STABILITY_SAMPLES), 2);
    Serial.print("%/min (limit ");
    Serial.print(HUM_THRESH);
    Serial.println(")");
    Serial.print("  Pres: ");
    Serial.print(pressureHistory.getChangeRate(STABILITY_SAMPLES), 2);
    Serial.print("hPa/min (limit ");
    Serial.print(PRES_THRESH);
    Serial.println(")");
    Serial.print("  Lux: ");
    Serial.print(luxHistory.getChangeRate(STABILITY_SAMPLES), 1);
    Serial.print("lux/min (limit ");
    Serial.print(LUX_THRESH);
    Serial.println(")");
    
#if ENABLE_VEML_AUTORANGE
    // VEML autorange info - pouze pokud aktivní
    Serial.print("VEML autorange: step ");
    Serial.print(VEML_GetCurrentStepIndex());
    Serial.print(" (");
    Serial.print(VEML_GetCurrentStepName());
    Serial.println(")");
#else
    // VEML fixní režim info
    Serial.println("VEML: fixní režim G1/8 IT100ms");
#endif
    
    Serial.println("╔═══════════════════════════════════════╗");
    Serial.println("║       SENSOR CYCLE END              ║");
    Serial.println("╚═══════════════════════════════════════╝\n");
  }
  
  // === STATUS REPORTING ===
  if (now - lastStatusReport >= STATUS_INT) {
    lastStatusReport = now;
    
    if (mqttConnected) {
      Serial.println("Publikuji system status...");
      publishSystemStatus();
    } else {
      Serial.println("MQTT nedostupné - system status nepublikován");
    }
  }
  
  // === PERIODICKÝ DEBUG VÝPIS ===
  static unsigned long lastDebugPrint = 0;
  if (now - lastDebugPrint >= 60000) { // Každou minutu
    lastDebugPrint = now;
    printDetailedStatus();
  }
  
  // === MEMORY HEALTH CHECK ===
  static unsigned long lastMemCheck = 0;
  if (now - lastMemCheck >= 30000) { // Každých 30s
    lastMemCheck = now;
    checkMemoryHealth();
  }
  
  // === WATCHDOG & STABILITY ===
  
  // Robust Reconnect Check
  checkEmergencyRestart();
  
  // Krátká pauza pro stabilitu systému
  delay(50);
  
  // ESP8266 watchdog reset
  ESP.wdtFeed();
}

// ===== DETAILNÍ STATUS VÝPIS (ROZŠÍŘENÝ S AUTORANGE!) =====
void printDetailedStatus() {
  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.println("║            SYSTEM STATUS                      ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
  
  // Základní info
  Serial.println("┌─ ZÁKLADNÍ INFORMACE ─────────────────────────────────────────────");
  Serial.print("│ Device: ");
  Serial.print(DEVICE_TYPE);
  Serial.print(" (");
  Serial.print(chipId);
  Serial.println(")");
  Serial.print("│ Firmware: ");
  Serial.println(FIRMWARE_VERSION);
  Serial.print("│ Uptime: ");
  Serial.print(uptimeSeconds / 86400);
  Serial.print("d ");
  Serial.print((uptimeSeconds % 86400) / 3600);
  Serial.print("h ");
  Serial.print((uptimeSeconds % 3600) / 60);
  Serial.print("m ");
  Serial.print(uptimeSeconds % 60);
  Serial.println("s");
  Serial.println("└──────────────────────────────────────────────────────────────────");
  
  // Síťové připojení
  Serial.println("┌─ SÍŤOVÉ PŘIPOJENÍ ───────────────────────────────────────────────");
  Serial.print("│ WiFi: ");
  Serial.print(wifiConnected ? "PŘIPOJENO" : "ODPOJENO");
  if (wifiConnected) {
    Serial.print(" (");
    Serial.print(WiFi.localIP());
    Serial.print(", ");
    Serial.print(WiFi.RSSI());
    Serial.print(" dBm)");
  }
  Serial.println();
  Serial.print("│ MQTT: ");
  Serial.print(mqttConnected ? "PŘIPOJENO" : "ODPOJENO");
  Serial.print(" (");
  Serial.print(reconnectCount);
  Serial.println(" reconnects)");
  Serial.println("└──────────────────────────────────────────────────────────────────");
  
  // Senzory stav s autorange info
  Serial.println("┌─ SENZORY STAV ───────────────────────────────────────────────────");
  Serial.print("│ BME280: ");
  Serial.print(currentSensorData.bme280Working ? "OK" : "CHYBA");
  if (currentSensorData.bme280Working) {
    Serial.print(" | T:");
    Serial.print(currentSensorData.temperature, 1);
    Serial.print("°C H:");
    Serial.print(currentSensorData.humidity, 0);
    Serial.print("% P:");
    Serial.print(currentSensorData.pressure, 0);
    Serial.print("hPa");
  }
  Serial.println();
  Serial.print("│ VEML7700: ");
  Serial.print(currentSensorData.vemlWorking ? "OK" : "CHYBA");
  if (currentSensorData.vemlWorking) {
    Serial.print(" | L:");
    Serial.print(currentSensorData.lux, 0);
    Serial.print(" lux");
#if ENABLE_VEML_AUTORANGE
    Serial.print(" [AR:");
    Serial.print(VEML_GetCurrentStepName());
    Serial.print("]");
#else
    Serial.print(" [FIXED]");
#endif
  }
  Serial.println();
  
#if ENABLE_VEML_AUTORANGE
  // Autorange detaily
  Serial.print("│ AutoRange: AKTIVNÍ, krok ");
  Serial.print(VEML_GetCurrentStepIndex());
  Serial.print("/");
  Serial.print(VEML_STEPS_COUNT - 1);
  Serial.print(" (");
  Serial.print(VEML_GetCurrentStepName());
  Serial.println(")");
  Serial.print("│ AR prahy: RAW ");
  Serial.print(VEML_AR_RAW_LOW);
  Serial.print("-");
  Serial.print(VEML_AR_RAW_HIGH);
  Serial.print(", dwell ");
  Serial.print(VEML_AR_DWELL_MS);
  Serial.println("ms");
#else
  Serial.println("│ AutoRange: NEAKTIVNÍ (fixní režim)");
#endif
  Serial.println("└──────────────────────────────────────────────────────────────────");
  
  // Operační režim
  Serial.println("┌─ OPERAČNÍ REŽIM ─────────────────────────────────────────────────");
  Serial.print("│ Aktuální: ");
  Serial.print(getOperationalModeString(currentMode));
  Serial.print(" (");
  Serial.print(currentSensorInterval / 1000);
  Serial.println("s)");
  Serial.print("│ Stabilita: ");
  Serial.println(stabilityDetected ? "DETEKOVÁNA" : "NEDETEKOVÁNA");
  if (stabilityDetected) {
    unsigned long stabMin = (millis() - stabilityStartTime) / 60000;
    Serial.print("│ Trvání stability: ");
    Serial.print(stabMin);
    Serial.println(" minut");
  }
  Serial.println("└──────────────────────────────────────────────────────────────────");
  
  // Změnové rychlosti pro stability detection (včetně pressure!)
  Serial.println("┌─ ZMĚNOVÉ RYCHLOSTI ──────────────────────────────────────────────");
  Serial.print("│ Teplota: ");
  Serial.print(tempHistory.getChangeRate(STABILITY_SAMPLES), 3);
  Serial.print(" °C/min (limit ");
  Serial.print(TEMP_THRESH);
  Serial.println(")");
  Serial.print("│ Vlhkost: ");
  Serial.print(humidityHistory.getChangeRate(STABILITY_SAMPLES), 2);
  Serial.print(" %/min (limit ");
  Serial.print(HUM_THRESH);
  Serial.println(")");
  Serial.print("│ Tlak: ");
  Serial.print(pressureHistory.getChangeRate(STABILITY_SAMPLES), 2);
  Serial.print(" hPa/min (limit ");
  Serial.print(PRES_THRESH);
  Serial.println(")");
  Serial.print("│ Světlo: ");
  Serial.print(luxHistory.getChangeRate(STABILITY_SAMPLES), 1);
  Serial.print(" lux/min (limit ");
  Serial.print(LUX_THRESH);
  Serial.println(")");
  Serial.println("└──────────────────────────────────────────────────────────────────");
  
  // Systémové prostředky
  Serial.println("┌─ SYSTÉMOVÉ PROSTŘEDKY ───────────────────────────────────────────");
  Serial.print("│ Free heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  Serial.print("│ Flash size: ");
  Serial.print(flashSize / 1024);
  Serial.println(" KB");
  Serial.print("│ CPU freq: ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println(" MHz");
  Serial.println("└──────────────────────────────────────────────────────────────────");
  
  // MQTT témata reminder
  Serial.println("┌─ MQTT KONFIGURACE ───────────────────────────────────────────────");
  Serial.print("│ Weather topic: ");
  Serial.println(TOPIC_WEATHER);
  Serial.print("│ Status topic: ");
  Serial.println(TOPIC_STATUS);
  Serial.print("│ System topic: ");
  Serial.println(TOPIC_SYSTEM);
  Serial.println("└──────────────────────────────────────────────────────────────────");
  
  Serial.println("╔═══════════════════════════════════════════════╗");
  Serial.println("║          STATUS END                           ║");
  Serial.println("╚═══════════════════════════════════════════════╝\n");
}

// ===== EMERGENCY RESTART FUNKCE =====
void emergencyRestart(const char* reason) {
  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.println("║        EMERGENCY RESTART                     ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
  Serial.print("Důvod: ");
  Serial.println(reason);
  
  // Publikuj offline status pokud možno
  if (mqttConnected) {
    snprintf(g_jsonBuffer, sizeof(g_jsonBuffer), "{\"%s\":\"restarting\"}", KEY_STATUS);
    mqttClient.publish(TOPIC_STATUS, g_jsonBuffer, true);
    delay(1000);
  }
  
  // LED signalizace restartu
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
  
  Serial.println("Restartování za 3 sekundy...");
  delay(3000);
  ESP.restart();
}

// ===== MEMORY CHECK FUNKCE (VYLEPŠENÍ!) =====
void checkMemoryHealth() {
  static uint32_t lastFreeHeap = 0;
  static int lowMemoryCount = 0;
  static unsigned long lastMemoryReport = 0;
  
  uint32_t currentFreeHeap = ESP.getFreeHeap();
  
  // Kontrola příliš nízké paměti
  if (currentFreeHeap < 5000) { // Méně než 5KB
    lowMemoryCount++;
    Serial.print("VAROVÁNÍ: Nízká paměť! Free heap: ");
    Serial.print(currentFreeHeap);
    Serial.println(" bytes");
    
#if ENABLE_VEML_AUTORANGE
    Serial.print("AutoRange stav: ");
    Serial.print(VEML_GetCurrentStepName());
    Serial.print(" (step ");
    Serial.print(VEML_GetCurrentStepIndex());
    Serial.println(")");
#endif
    
    if (lowMemoryCount > 5) {
      emergencyRestart("Kriticky nízká paměť");
    }
  } else {
    lowMemoryCount = 0; // Reset počítadla
  }
  
  // Periodický memory report
  unsigned long now = millis();
  if (now - lastMemoryReport > 300000) { // Každých 5 minut
    lastMemoryReport = now;
    Serial.print("Memory health: ");
    Serial.print(currentFreeHeap);
    Serial.print(" bytes free");
    
    if (lastFreeHeap > 0) {
      int32_t diff = (int32_t)currentFreeHeap - (int32_t)lastFreeHeap;
      Serial.print(" (");
      Serial.print(diff > 0 ? "+" : "");
      Serial.print(diff);
      Serial.print(" from last check)");
    }
    
#if ENABLE_VEML_AUTORANGE
    Serial.print(" | VEML-AR: ");
    Serial.print(VEML_GetCurrentStepName());
#endif
    Serial.println();
  }
  
  lastFreeHeap = currentFreeHeap;
}

//Main Loop & Debug Functions END
//------------07-----------