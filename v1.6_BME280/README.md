# HP2 Sensor v2 - Weather Station

Firmware for **Wemos D1 Mini** with **BME280** sensor, serving as an intelligent weather station for the Zonio system.

## ⚡ Features
- **Sensors**: Temperature, Humidity, Pressure (BME280).
- **Adaptive Sampling**: Measurement interval (2s - 30s) automatically adjusts based on stability of readings.
- **Stability Detection**: Algorithm monitors value changes over time.
- **MQTT Communication**: Sends data and device status in JSON format.
- **Watchdog & Recovery**: Automatic restart upon long-term connection loss or sensor error.

## 🛠️ Hardware
- **MCU**: Wemos D1 Mini (ESP8266)
- **Sensor**: BME280 (I2C)

### Wiring (I2C)
| BME280 | Wemos D1 Mini |
|--------|---------------|
| VCC    | 3.3V          |
| GND    | G             |
| SCL    | D1 (GPIO5)    |
| SDA    | D2 (GPIO4)    |

## ⚙️ Configuration
Before flashing, modify the **CONFIGURATION** section in `v2_refactored.ino`:

```cpp
// WiFi settings
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// MQTT settings
const char* MQTT_SERVER   = "BROKER_IP_ADDRESS";
const char* MQTT_USERNAME = "mqtt_user";
const char* MQTT_PASSWORD = "mqtt_password";
```

## 📡 MQTT Topics
| Topic | Description |
|-------|-------------|
| `zonio/weather/HP2` | Measurement data (temperature, humidity, pressure) |
| `zonio/device/status/HP2` | LWT + online/offline status |
| `zonio/system/status/HP2` | System info (IP, uptime, RAM, interval) |

## 📦 Prerequisites (Arduino IDE)
Install the following libraries:
- `ESP8266WiFi`
- `PubSubClient`
- `Adafruit BME280 Library`
- `Adafruit Unified Sensor`

## 🚀 Installation
1. Open `v2_refactored.ino` in Arduino IDE.
2. Set board to **LOLIN(WEMOS) D1 R2 & mini**.
3. Update credentials.
4. Upload firmware.
