//-----------01-----------
// @ZONIO_ID:ZONIO-OC-120
// Zonio Wemos D1 Mini + BME280 Meteostanice
// Verze: 1.6.1 - Upraveno pro Wemos D1 mini bez displeje
// Založeno na ESP32-C3 firmware v1.6.0

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

// ===== KONFIGURACE =====
// WiFi nastavení
const char* WIFI_SSID     = "ssid";    // Název WiFi sítě
const char* WIFI_PASSWORD = "pass";       // Heslo WiFi

// MQTT nastavení
const char*   MQTT_SERVER        = "mqtt_server_ip";  // Adresa MQTT brokeru
const uint16_t MQTT_PORT         = 1883;           // Port MQTT brokeru
const char*   MQTT_USERNAME      = "usr";        // MQTT uživatelské jméno
const char*   MQTT_PASSWORD      = "pass";      // MQTT heslo
const char*   MQTT_CLIENT_ID_BASE = "WemosD1_Meteo_sensor1"; // Základ MQTT Client ID

// MQTT témata
const char* TOPIC_STATUS  = "zonio/device/status/sensor1";   // LWT + stav online/offline
const char* TOPIC_WEATHER = "zonio/weather/sensor1";         // Naměřené hodnoty počasí (JSON)
const char* TOPIC_SYSTEM  = "zonio/system/status/sensor1";   // Systémový status zařízení (JSON)

// Další nastavení
const char* FIRMWARE_VERSION = "v1.6.1-WemosD1-BME280";

// Časové intervaly (ms)
const unsigned long STATUS_INTERVAL   = 60000;   // Interval odesílání systémového statusu (60 s)
const unsigned long RECONNECT_INTERVAL_BASE = 5000;    // Základní interval pokusů o MQTT připojení (5 s)
const unsigned long RECONNECT_INTERVAL_MAX  = 300000;  // Maximální interval pokusů o MQTT připojení (5 min)
const unsigned long MAX_DISCONNECT_TIME = 3600000; // 1 hodina - poté restart
const int MAX_FAILED_RECONNECTS = 50; // Max neúspěšných pokusů

// Adaptivní intervaly vzorkování (ms)
const unsigned long INTERVAL_ULTRASHORT = 2000; // 2 sekundy - ultra krátký interval (počáteční)
const unsigned long INTERVAL_SHORT = 6000;      // 6 sekund - krátký interval (po 10 minutách stability)
const unsigned long INTERVAL_MEDIUM = 15000;    // 15 sekund - střední interval (následujících 15 minut)
const unsigned long INTERVAL_LONG = 30000;      // 30 sekund - dlouhý interval (po 25 minutách stability)

// Počet opakovaných odeslání dat při dlouhém intervalu
const int REPEAT_COUNT = 3;  // Při dlouhém vzorkování odeslat 3x po sobě

// Pinová konfigurace Wemos D1 mini + BME280
// I2C pro BME280: SDA = D2 (GPIO4), SCL = D1 (GPIO5)
#define BME_SDA    D2  // GPIO4
#define BME_SCL    D1  // GPIO5
#define LED_BUILTIN D4 // GPIO2 - vestavěná LED na Wemos D1 mini

// ===== OBJEKTY =====
Adafruit_BME280 bme;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ===== GLOBÁLNÍ PROMĚNNÉ =====
// Struktura pro ukládání naměřených hodnot a stavů senzorů
struct SensorData {
  float temperature = 0;
  float humidity    = 0;
  float pressure    = 0;
  bool bme280Working = false;
};

// Struktura pro ukládání historie měření
#define HISTORY_SIZE 10  // Historie pro detekci stability
struct SensorHistory {
  float values[HISTORY_SIZE];
  int index;
  bool filled;
  unsigned long timestamps[HISTORY_SIZE];

  void init() {
    index = 0;
    filled = false;
    for (int i = 0; i < HISTORY_SIZE; i++) {
      values[i] = 0;
      timestamps[i] = 0;
    }
  }

  void add(float value, unsigned long timestamp) {
    values[index] = value;
    timestamps[index] = timestamp;
    index = (index + 1) % HISTORY_SIZE;
    if (index == 0) filled = true;
  }

  // Získat průměrnou hodnotu změny za minutu za posledních n záznamů
  float getChangeRate(int sampleCount) {
    if (index == 0 && !filled) return 0; // Prázdná historie
    
    int count = filled ? HISTORY_SIZE : index;
    count = min(count, sampleCount);
    
    if (count < 2) return 0; // Potřebujeme alespoň 2 záznamy pro výpočet změny
    
    int startIdx = (index - count + HISTORY_SIZE) % HISTORY_SIZE;
    int endIdx = (index - 1 + HISTORY_SIZE) % HISTORY_SIZE;
    
    float valueChange = fabs(values[endIdx] - values[startIdx]);
    unsigned long timeChange = timestamps[endIdx] - timestamps[startIdx];
    
    // Ochrana proti dělení nulou
    if (timeChange == 0) return 0;
    
    // Převod na změnu za minutu
    return valueChange * 60000 / timeChange;
  }

//-----------01-----------

//-----------02-----------

  // Zkontrolovat stabilitu hodnot (malé změny za minutu)
  bool isStable(float maxChangePerMinute, int sampleCount) {
    float rate = getChangeRate(sampleCount);
    return rate <= maxChangePerMinute;
  }
};

SensorHistory tempHistory;
SensorHistory humidityHistory;
SensorHistory pressureHistory;

// Globální deklarace proměnných
SensorData sensorData;
SensorData lastSensorData;

enum AdaptiveMode { ULTRASHORT_INTERVAL, SHORT_INTERVAL, MEDIUM_INTERVAL, LONG_INTERVAL };

AdaptiveMode currentMode = SHORT_INTERVAL;
unsigned long currentSensorInterval = INTERVAL_SHORT;
unsigned long modeStartTime = 0;
unsigned long stabilityStartTime = 0;
bool isStabilityDetected = false;

bool wifiConnected = false;
bool mqttConnected = false;
unsigned long lastReconnectAttempt = 0;
unsigned long reconnectInterval   = RECONNECT_INTERVAL_BASE;
int reconnectCount = 0;
unsigned long lastSensorRead   = 0;
unsigned long lastStatusReport = 0;
unsigned long uptimeSeconds    = 0;
unsigned long lastUptimeUpdate = 0;
unsigned long lastPublish = 0;
unsigned long disconnectStartTime = 0;
int failedReconnectCount = 0;
bool wasEverConnected = false;

// ===== FUNKCE =====

// Funkce pro aktualizaci historií
void updateHistories() {
  unsigned long now = millis();
  tempHistory.add(sensorData.temperature, now);
  humidityHistory.add(sensorData.humidity, now);
  pressureHistory.add(sensorData.pressure, now);
}

// Kontrola podmínek pro změnu režimu vzorkování
void checkSamplingMode() {
  unsigned long now = millis();
  
  // Výpočet doby od začátku aktuálního režimu
  unsigned long modeElapsedTime = (now - modeStartTime) / 60000; // v minutách
  
  // Kontrola stability měření (nízká rychlost změny)
  bool isTemperatureStable = tempHistory.isStable(0.5, 10); // max 0.5°C za minutu
  bool isHumidityStable = humidityHistory.isStable(2.0, 10); // max 2% za minutu
  
  bool isCurrentlyStable = isTemperatureStable && isHumidityStable;
  
  // Logování stability
  if (isCurrentlyStable && !isStabilityDetected) {
    isStabilityDetected = true;
    stabilityStartTime = now;
    Serial.println("Detekována stabilita hodnot!");
  } else if (!isCurrentlyStable && isStabilityDetected) {
    isStabilityDetected = false;
    Serial.println("Ztráta stability hodnot!");
    
    // Při ztrátě stability se vždy vrátíme na ultra krátký interval
    if (currentMode != ULTRASHORT_INTERVAL) {
      currentMode = ULTRASHORT_INTERVAL;
      currentSensorInterval = INTERVAL_ULTRASHORT;
      modeStartTime = now;
      Serial.println("Ztráta stability - návrat na ULTRA KRÁTKÝ interval (2s)");
    }
    
    // Dřívější return - nepokračujeme dál v kontrole přechodu na delší intervaly
    return;
  }
  
  // Kontrola trvání stability
  unsigned long stabilityDuration = 0;
  if (isStabilityDetected) {
    stabilityDuration = (now - stabilityStartTime) / 60000; // v minutách
    
    // Přechod na delší intervaly jen když je dostatečně dlouhá stabilita
    if (stabilityDuration >= 5) {  // Minimálně 5 minut stability
      // Logika pro přechod mezi režimy
      switch (currentMode) {
        case ULTRASHORT_INTERVAL:
          // Po 10 minutách přejít na krátký interval
          if (modeElapsedTime >= 10) {
            currentMode = SHORT_INTERVAL;
            currentSensorInterval = INTERVAL_SHORT;
            modeStartTime = now;
            Serial.println("Přechod na KRÁTKÝ interval (6s)");
          }
          break;
        
        case SHORT_INTERVAL:
          // Po 10 minutách přejít na střední interval
          if (modeElapsedTime >= 10) {
            currentMode = MEDIUM_INTERVAL;
            currentSensorInterval = INTERVAL_MEDIUM;
            modeStartTime = now;
            Serial.println("Přechod na STŘEDNÍ interval (15s)");
          }
          break;
          
        case MEDIUM_INTERVAL:
          // Po 15 minutách přejít na dlouhý interval
          if (modeElapsedTime >= 15) {
            currentMode = LONG_INTERVAL;
            currentSensorInterval = INTERVAL_LONG;
            modeStartTime = now;
            Serial.println("Přechod na DLOUHÝ interval (30s)");
          }
          break;
      }
    }
  } else {
    // Pokud není stabilita, vždy použít ultra krátký interval
    if (currentMode != ULTRASHORT_INTERVAL) {
      currentMode = ULTRASHORT_INTERVAL;
      currentSensorInterval = INTERVAL_ULTRASHORT;
      modeStartTime = now;
      Serial.println("Není detekována stabilita - nastavení na ULTRA KRÁTKÝ interval (2s)");
    }
  }
}

//-----------02-----------

//-----------03-----------

// Inicializace BME280 senzoru
bool initSensors() {
  bool success = true;
  
  // Inicializace I2C na správných pinech pro Wemos D1 mini
  Wire.begin(BME_SDA, BME_SCL);
  Serial.println("I2C inicializováno na pinech SDA=D2(GPIO4), SCL=D1(GPIO5)");
  
  // Inicializace BME280 (teplota, vlhkost, tlak)
  if (!bme.begin(0x76)) {
    Serial.println("BME280 senzor nenalezen na primární adrese 0x76!");
    // Zkusíme alternativní adresu
    if (!bme.begin(0x77)) {
      Serial.println("BME280 senzor nenalezen ani na alternativní adrese 0x77!");
      sensorData.bme280Working = false;
      success = false;
    } else {
      sensorData.bme280Working = true;
      Serial.println("BME280 inicializován na adrese 0x77");
    }
  } else {
    sensorData.bme280Working = true;
    Serial.println("BME280 inicializován na adrese 0x76");
  }
  
  if (sensorData.bme280Working) {
    // Nastavení oversamplingu a filtrace BME280
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,   // oversampling teplota
                    Adafruit_BME280::SAMPLING_X16,  // oversampling tlak
                    Adafruit_BME280::SAMPLING_X16,  // oversampling vlhkost
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_500);
    
    // První čtení ze senzoru pro inicializaci
    delay(50);
    float temp = bme.readTemperature();
    if (isnan(temp)) {
      Serial.println("BME280 první čtení neúspěšné, zkouším jiné nastavení");
      // Zkusíme jiné nastavení
      bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                      Adafruit_BME280::SAMPLING_X1,   
                      Adafruit_BME280::SAMPLING_X1,  
                      Adafruit_BME280::SAMPLING_X1,  
                      Adafruit_BME280::FILTER_OFF,
                      Adafruit_BME280::STANDBY_MS_1000);
    }
  }
  
  return success;
}

// Připojení k WiFi síti s omezeným čekáním
void connectWiFi() {
  Serial.println("Připojování k WiFi...");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // LED svítí při připojování (LOW = svítí na Wemos D1)
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // Čekání na připojení (timeout 15 s)
  unsigned long startAttempt = millis();
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(500);
    Serial.print(".");
    
    // Blikání LED při připojování
    digitalWrite(LED_BUILTIN, counter % 2);
    counter++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\nWiFi připojeno");
    Serial.print("IP adresa: ");
    Serial.println(WiFi.localIP());
    
    // LED zhasne po úspěšném připojení
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    wifiConnected = false;
    Serial.println("\nPřipojení k WiFi selhalo!");
    
    // LED zůstane svítit při chybě
    digitalWrite(LED_BUILTIN, LOW);
  }
}

// Připojení k MQTT brokeru (s exponenciálním odstupňováním pokusů)
void connectMQTT() {
  if (!wifiConnected || (millis() - lastReconnectAttempt < reconnectInterval)) {
    return;
  }
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // Indikace aktivity
  lastReconnectAttempt = millis();
  Serial.println("Pokus o připojení k MQTT brokeru...");
  reconnectCount++;
  
  // Vytvoření client ID (unikátní kombinace jména a náhodného čísla)
  String clientId = MQTT_CLIENT_ID_BASE;
  clientId += "-";
  clientId += String(random(0xffff), HEX);
  
  // Nastavení Last Will zprávy (stav offline pro LWT)
  const char* willMsg = "{\"status\":\"offline\"}";
  
  // Pokus o připojení
  if (mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD, 
                         TOPIC_STATUS, 1, true, willMsg)) {
    mqttConnected = true;
    Serial.println("Připojeno k MQTT brokeru");
    // Resetování intervalu a čítače pokusů
    reconnectCount = 0;
    reconnectInterval = RECONNECT_INTERVAL_BASE;
    
    // Publikování online statusu (retained zpráva)
    IPAddress ip = WiFi.localIP();
    char statusMsg[200];
    snprintf(statusMsg, sizeof(statusMsg),
             "{\"status\":\"online\",\"ip\":\"%d.%d.%d.%d\","
             "\"rssi\":%d,\"uptime\":%lu,\"deviceType\":\"%s\","
             "\"firmware\":\"%s\"}",
             ip[0], ip[1], ip[2], ip[3],
             WiFi.RSSI(),
             millis() / 1000,
             MQTT_CLIENT_ID_BASE,
             FIRMWARE_VERSION);
    mqttClient.publish(TOPIC_STATUS, statusMsg, true);
  } else {
    mqttConnected = false;
    failedReconnectCount++; // Počítání neúspěšných pokusů
    Serial.print("MQTT připojení selhalo, rc=");
    Serial.print(mqttClient.state());
    Serial.print(", pokus #");
    Serial.println(failedReconnectCount);
  
    // Exponenciální prodloužení intervalu pro další pokus
    if (reconnectInterval < RECONNECT_INTERVAL_MAX) {
      reconnectInterval *= 2;
      if (reconnectInterval > RECONNECT_INTERVAL_MAX) {
        reconnectInterval = RECONNECT_INTERVAL_MAX;
      }
    }
  }
  
  digitalWrite(LED_BUILTIN, HIGH);  // Konec indikace
}

//-----------03-----------

//-----------04-----------

// Publikování systémových informací (stav zařízení) na MQTT
void publishSystemStatus() {
  if (!mqttConnected) return;
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // Indikace aktivity
  
  // Sestavení JSON zprávy se systémovými údaji
  IPAddress ip = WiFi.localIP();
  char statusPayload[300];
  snprintf(statusPayload, sizeof(statusPayload),
           "{\"deviceIp\":\"%d.%d.%d.%d\",\"firmwareVersion\":\"%s\","
           "\"uptime\":%lu,\"rssi\":%d,\"reconnectCount\":%d,"
           "\"freeHeap\":%u,\"sampleMode\":\"%s\",\"sampleInterval\":%lu,"
           "\"bme280\":%s}",
           ip[0], ip[1], ip[2], ip[3],
           FIRMWARE_VERSION,
           uptimeSeconds,
           WiFi.RSSI(),
           reconnectCount,
           ESP.getFreeHeap(),
           currentMode == ULTRASHORT_INTERVAL ? "ULTRASHORT" :
           currentMode == SHORT_INTERVAL ? "SHORT" : 
           currentMode == MEDIUM_INTERVAL ? "MEDIUM" : "LONG",
           currentSensorInterval,
           sensorData.bme280Working ? "true" : "false");
  
  mqttClient.publish(TOPIC_SYSTEM, statusPayload, false);
  digitalWrite(LED_BUILTIN, HIGH);  // Konec indikace
}

// Kontrola stavu WiFi/MQTT a opětovné připojení při výpadku
void checkConnections() {
  static unsigned long lastWifiAttempt = 0;
  
  // Kontrola WiFi připojení
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      Serial.println("WiFi odpojeno!");
      wifiConnected = false;
      mqttConnected = false;
    }
    
    // Pokus o znovupřipojení k WiFi každých 10 s
    if (millis() - lastWifiAttempt > 5000) {
      lastWifiAttempt = millis();
      connectWiFi();
    }
  } else {
    if (!wifiConnected) {
      wifiConnected = true;
      Serial.println("WiFi znovu připojeno");
      reconnectInterval = RECONNECT_INTERVAL_BASE;
    }
  }
  
  // Kontrola MQTT připojení (pokud WiFi je OK)
  if (wifiConnected) {
    if (mqttConnected && !mqttClient.connected()) {
      Serial.println("MQTT odpojeno!");
      mqttConnected = false;
      reconnectInterval = RECONNECT_INTERVAL_BASE; // RESET intervalu!
      Serial.println("MQTT interval resetován");
    }
    
    if (!mqttConnected) {
      connectMQTT();
    } else {
      mqttClient.loop(); // KRITICKÉ: Zpracování MQTT zpráv
    }
  }
}

// Čtení hodnot z BME280 senzoru
void readSensors() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // Indikace aktivity
  
  // BME280 (teplota, vlhkost, tlak)
  if (sensorData.bme280Working) {
    // Čtení hodnot s kontrolou platnosti
    float temp = bme.readTemperature();
    float hum = bme.readHumidity();
    float press = bme.readPressure() / 100.0F; // Pa -> hPa
    
    // Kontrola platnosti hodnot
    if (isnan(temp) || isnan(hum) || isnan(press) || 
        temp < -40 || temp > 85 || hum < 0 || hum > 100 || press < 800 || press > 1200) {
      Serial.println("BME280 vrací neplatné hodnoty!");
      
      // Pokus o znovučtení s menším počtem nastavení
      delay(50);
      temp = bme.readTemperature();
      hum = bme.readHumidity();
      press = bme.readPressure() / 100.0F;
      
      if (isnan(temp) || isnan(hum) || isnan(press) || 
          temp < -40 || temp > 85 || hum < 0 || hum > 100 || press < 800 || press > 1200) {
        Serial.println("BME280 stále vrací neplatné hodnoty, pokus o reinicializaci!");
        
        // Pokus o restart senzoru
        if (!bme.begin(0x76) && !bme.begin(0x77)) {
          sensorData.bme280Working = false;
          Serial.println("Reinicializace BME280 selhala!");
        } else {
          Serial.println("BME280 reinicializován");
          bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                        Adafruit_BME280::SAMPLING_X1,
                        Adafruit_BME280::SAMPLING_X1,
                        Adafruit_BME280::SAMPLING_X1,
                        Adafruit_BME280::FILTER_OFF,
                        Adafruit_BME280::STANDBY_MS_1000);
          delay(100);
          
          // Ještě jeden pokus o čtení
          temp = bme.readTemperature();
          hum = bme.readHumidity();
          press = bme.readPressure() / 100.0F;
          
          if (isnan(temp) || isnan(hum) || isnan(press) || 
              temp < -40 || temp > 85 || hum < 0 || hum > 100 || press < 800 || press > 1200) {
            sensorData.bme280Working = false;
            Serial.println("BME280 stále vrací neplatné hodnoty, deaktivace!");
          } else {
            sensorData.temperature = temp;
            sensorData.humidity = hum;
            sensorData.pressure = press;
            Serial.println("BME280 opraven po reinicializaci");
          }
        }
      } else {
        // Druhý pokus byl úspěšný
        sensorData.temperature = temp;
        sensorData.humidity = hum;
        sensorData.pressure = press;
        Serial.println("BME280 čtení opraveno druhým pokusem");
      }
    } else {
      // První čtení bylo úspěšné
      sensorData.temperature = temp;
      sensorData.humidity = hum;
      sensorData.pressure = press;
      
      Serial.print("Teplota: ");
      Serial.print(sensorData.temperature, 1);
      Serial.print(" °C, Vlhkost: ");
      Serial.print(sensorData.humidity, 0);
      Serial.print(" %, Tlak: ");
      Serial.print(sensorData.pressure, 0);
      Serial.println(" hPa");
    }
  }
  
  digitalWrite(LED_BUILTIN, HIGH);  // Konec indikace aktivity
}

//-----------04-----------

//-----------05-----------

// Publikování naměřených dat na MQTT
void publishSensorData(int repeatCount = 1) {
  if (!mqttConnected) return;
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // Indikace aktivity
  
  char payload[150];
  for (int i = 0; i < repeatCount; i++) {
    // Přidáváme číslo opakování pro rozlišení na dashboardu
    snprintf(payload, sizeof(payload),
             "{\"temp\":%.1f,\"hum\":%.0f,\"pres\":%.0f,\"repeat\":%d}",
             sensorData.temperature,
             sensorData.humidity,
             sensorData.pressure,
             i);
    
    mqttClient.publish(TOPIC_WEATHER, payload);
    Serial.print("Data odeslána (opakování ");
    Serial.print(i+1);
    Serial.print("/");
    Serial.print(repeatCount);
    Serial.println(")");
    
    if (i < repeatCount - 1) {
      delay(500);  // Krátká pauza mezi opakovanými odesláními
    }
  }
  
  digitalWrite(LED_BUILTIN, HIGH);  // Konec indikace
}

// Emergency restart při dlouhodobém odpojení
void checkEmergencyRestart() {
  unsigned long now = millis();
  
  // Pokud jsme už někdy byli připojeni a teď nejsme
  if (wasEverConnected && (!wifiConnected || !mqttConnected)) {
    if (disconnectStartTime == 0) {
      disconnectStartTime = now;
      Serial.println("Začátek odpojení zaznamenán");
    }
    
    // Kontrola celkového času odpojení
    if (now - disconnectStartTime > MAX_DISCONNECT_TIME) {
      Serial.println("EMERGENCY RESTART: Dlouhodobé odpojení (>1h)");
      ESP.restart();
    }
    
    // Kontrola počtu neúspěšných pokusů
    if (failedReconnectCount > MAX_FAILED_RECONNECTS) {
      Serial.println("EMERGENCY RESTART: Příliš mnoho neúspěšných pokusů");
      ESP.restart();
    }
  } else if (wifiConnected && mqttConnected) {
    // Spojení je OK - reset čítačů
    disconnectStartTime = 0;
    failedReconnectCount = 0;
    wasEverConnected = true;
  }
}

// ===== SETUP =====
void setup() {
  // Inicializace sériové komunikace
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== Zonio Wemos D1 Mini + BME280 Meteostanice v2.0.0 ===");
  Serial.println("Inicializace...");
  
  // Nastavení LED pinu
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // LED zhasnutá na začátku (HIGH = zhasnutá na Wemos D1)
  
  // Inicializace senzorů
  bool sensorsOk = initSensors();
  
  if (!sensorsOk || !sensorData.bme280Working) {
    Serial.println("BME280 senzor nebyl detekován!");
    // Blikání LED při chybě senzoru
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
    }
  } else {
    Serial.println("BME280 senzor úspěšně inicializován");
  }
  
  // Připojení k WiFi
  connectWiFi();
  
  // Nastavení MQTT klienta
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setKeepAlive(60);

  // Inicializace historie pro sledování trendů
  tempHistory.init();
  humidityHistory.init();
  pressureHistory.init();
  
  // Nastavení počátečního režimu vzorkování
  currentMode = ULTRASHORT_INTERVAL;
  currentSensorInterval = INTERVAL_ULTRASHORT;
  modeStartTime = millis();
  
  Serial.println("Inicializace dokončena.");
  Serial.print("Počáteční interval vzorkování: ");
  Serial.print(currentSensorInterval / 1000);
  Serial.println(" sekund");
  Serial.println("Pinout: BME280 SDA=D2(GPIO4), SCL=D1(GPIO5)");
}

//-----------05-----------

//-----------06-----------

// ===== LOOP =====
void loop() {
  // ESP8266 watchdog feed
  ESP.wdtFeed();
  // Debug výpis intervalu vzorkování
  Serial.print("Aktuální interval vzorkování: ");
  Serial.print(currentSensorInterval / 1000);
  Serial.println(" s");
  
  // Kontrola a obnovení spojení (WiFi/MQTT)
  checkConnections();
  
  // Zpracování MQTT zpráv (pokud připojeno)
  if (mqttConnected) {
    mqttClient.loop();
  }
  
  // Aktualizace uptime každou sekundu
  unsigned long now = millis();
  if (now - lastUptimeUpdate >= 1000) {
    lastUptimeUpdate = now;
    uptimeSeconds++;
  }
  
  // Čtení senzorů a odeslání dat (podle aktuálního intervalu)
  if (now - lastSensorRead >= currentSensorInterval) {
    lastSensorRead = now;
    Serial.println("Čtení senzorů...");
    
    // Přečtení hodnot ze senzorů
    readSensors();
    
    // Aktualizace historických dat (pouze pokud BME280 funguje)
    if (sensorData.bme280Working) {
      updateHistories();
    }
    
    // Uložení aktuálních hodnot pro příští porovnání
    lastSensorData = sensorData;
    
    // Odeslání naměřených dat na MQTT (ve formátu JSON)
    if (mqttConnected && sensorData.bme280Working) {
      // Při dlouhém intervalu posíláme opakovaně pro spolehlivost
      if (currentMode == LONG_INTERVAL) {
        publishSensorData(REPEAT_COUNT);
      } else {
        publishSensorData(1);
      }
      lastPublish = now;
    }
    
    // Kontrola podmínek pro změnu režimu vzorkování (pouze pokud BME280 funguje)
    if (sensorData.bme280Working) {
      checkSamplingMode();
    }
  }
  
  // Pravidelné odeslání systémového statusu (každých STATUS_INTERVAL)
  if (now - lastStatusReport >= STATUS_INTERVAL) {
    lastStatusReport = now;
    if (mqttConnected) {
      Serial.println("Odesílání systémového statusu...");
      publishSystemStatus();
    }
  }

  // Emergency restart kontrola
  checkEmergencyRestart();  

  // Krátká pauza pro stabilitu systému
  delay(10);
}
//-----------06-----------
