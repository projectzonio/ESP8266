# Deep Technical Documentation: bme280_veml7700.ino

> **Monolith Analysis**: Decomposed into 14 blocks.



## Analysis: Prelude
# Block 1/8: Prelude & Metadata

## 1. Responsibilities
This block serves as the **File Header and Metadata Definition**. Its primary responsibilities are:
*   **System Identification**: Uniquely identifies the device logic within the broader fleet management system using a specific ID tag.
*   **Hardware Definition**: Explicitly targets the **ESP8266 D1 Mini** platform (noting a deviation from the generic "ESP32" system context provided in the prompt).
*   **Versioning**: Tracks the firmware version (2.3.0) and highlights specific feature sets included (VEML Autorange, No Display logic).

## 2. Key Elements
Since this block consists entirely of comments, there are no executable functions or variables. However, it defines critical **Metadata Keys**:

| Key / Tag | Value | Description |
| :--- | :--- | :--- |
| `@ZONIO_ID` | `ZONIO-ESP8266-121` | Unique identifier likely used by OTA (Over-The-Air) servers or deployment scripts to match firmware to specific hardware nodes. |
| `Hardware` | `ESP8266 D1 Mini` | Defines the specific microcontroller architecture and pinout expectations. |
| `Application` | `Meteostanice` | Defines the logical role: Weather Station. |
| `Version` | `2.3.0` | Current release iteration. |
| `Features` | `VEML Autorange` | Notes specific algorithmic logic present in later blocks (auto-adjusting integration time for light sensors). |

## 3. State & Logic
*   **Flow**: This block contains **no runtime flow**. It is static documentation.
*   **Logic**:
    *   The presence of `VEML Autorange` in the version notes implies that in upcoming blocks (specifically the `loop` or sensor reading functions), there will be logic to read the VEML7700 sensor, check for saturation (too bright) or under-exposure (too dark), and dynamically adjust gain/integration time.
    *   The `No Display Version` note implies code related to LCD/OLED management (I2C display initialization) will be absent or commented out in subsequent blocks to save memory and GPIO pins.

## 4. Integration
Although this block is not code, it exposes logical constraints to the rest of the system:
*   **External Tooling**: CI/CD pipelines or OTA managers parse the `@ZONIO_ID` to determine where to deploy this binary.
*   **Hardware Abstraction**: The definition `ESP8266` sets the compilation target, which dictates which lower-level libraries (e.g., `ESP8266WiFi.h` vs `WiFi.h`) must be included in the subsequent library import block.

---


## Analysis: Block 01
# Block 01: Configuration, Constants & Hardware Abstraction

## 1. Responsibilities
This block acts as the **foundation layer** of the monolithic firmware. It does not execute runtime logic (like loops or connections) but defines the environment in which the system operates. Its specific responsibilities are:

*   **Dependency Injection**: Importing necessary network (`ESP8266WiFi`, `PubSubClient`) and sensor (`Wire`, `Adafruit_*`) libraries.
*   **Hardware Abstraction (HAL)**: Normalizing the API differences between versions of the `Adafruit_VEML7700` library to ensure compilation success regardless of the library version installed.
*   **Security & Credentials**: Managing WiFi and MQTT credentials via conditional compilation (Secrets file vs. Hardcoded fallback).
*   **System Configuration**: Defining hardware pinouts (I2C), memory buffer sizes, and communication protocols (MQTT topics and JSON keys).
*   **Timing & Thresholds**: Establishing sampling intervals, connection timeouts, and thresholds for "Stability Detection" (logic used to determine if sensor readings have settled).

## 2. Key Functions & Variables

### VEML7700 Compatibility Layer
This section handles API changes in the Adafruit library using C++ preprocessor logic.

| Macro / Alias | Description |
| :--- | :--- |
| `VEML_API_MACRO` vs `VEML_API_ENUM` | Flags determined by checking if `VEML7700_GAIN_1` exists. Selects which API syntax to use. |
| `VGain_t` / `VIt_t` | Type aliases for Gain and Integration Time types (either `uint8_t` or library `enum`). |
| `VEML_SetGain(d, g)` | Inline wrapper function to call `d.setGain(g)` using the correct types. |
| `VEML_SetIT(d, it)` | Inline wrapper function to call `d.setIntegrationTime(it)`. |

### System Configuration Variables
| Variable / Constant | Value / Type | Purpose |
| :--- | :--- | :--- |
| `WIFI_SSID` / `PASS` | `char*` | Network credentials (conditionally compiled). |
| `TOPIC_*` | `const char*` | MQTT Topics for Status, Weather, and System reports. |
| `INT_FAST` | `500 ms` | Sampling interval for "Fast" mode (high activity). |
| `INT_NORMAL` | `700 ms` | Sampling interval for "Normal" mode (standard operation). |
| `I2C_SDA` / `SCL` | `D2` / `D1` | **ESP8266** specific I2C pin mapping. |
| `OperationalMode` | `enum` | Defines system states: `MODE_FAST`, `MODE_NORMAL`, `MODE_SLOW`. |
| `*_THRESH` | `float` | Delta values (e.g., 0.3°C) used to detect if the environment is stable. |
| `JSON_BUFFER_SIZE` | `256` | Static size for char arrays to prevent Heap Fragmentation. |

## 3. State & Logic Analysis

Although this is a configuration block, it contains significant **Preprocessor Logic** and **Type Logic**:

1.  **Library Version Detection**:
    *   The code checks `#if defined(VEML7700_GAIN_1)`.
    *   **Case A (Old Lib)**: Macros exist. It defines `VEML_API_MACRO`. Types are `uint8_t`.
    *   **Case B (New Lib)**: Macros missing. It defines `VEML_API_ENUM`. Types are `Adafruit_VEML7700::veml7700_gain_t`.
    *   *Result*: The rest of the code can simply call `VEML_SetGain(...)` without worrying about the underlying library version.

2.  **Security Hierarchy**:
    *   Checks `#ifdef USE_SECRETS_FILE`.
    *   If defined, it delegates to an external header (best practice).
    *   If undefined, it triggers a compiler `#warning` and loads hardcoded credentials (development fallback).

3.  **Platform Identification**:
    *   **Crucial Note**: The context provided mentioned "ESP32", but this code block specifically includes `<ESP8266WiFi.h>` and defines pins `D1`/`D2`. This confirms the target hardware for this specific code file is an **ESP8266** (e.g., Wemos D1 Mini).

## 4. Integration with Other Blocks

This block exposes global constants used by **all** subsequent blocks:

*   **To Global Variables (Block 02)**: The `OperationalMode` enum is required to define the state variable.
*   **To Setup (Block 03)**: `I2C_SDA` and `I2C_SCL` are used to initialize `Wire.begin()`.
*   **To Network Manager (Block 04/05)**: `WIFI_SSID`, `MQTT_SERVER`, and `TOPIC_*` constants are required for connection logic.
*   **To Sensor Loop (Block 06/07)**: `VEML_SetGain` wrapper and `INT_*` (timing constants) are essential for the main regulation loop.
*   **To JSON/Reporting**: `KEY_*` constants (e.g., `KEY_TEMP`) ensure consistent JSON key naming across the entire application lifecycle.

---


## Analysis: Block 02
## Block 02: VEML7700 AutoRange Implementation

This block implements a dynamic logic layer over the standard Adafruit VEML7700 driver. It solves the common problem where a light sensor saturates in direct sunlight or provides too little data in darkness by automatically adjusting Gain and Integration Time.

### 1. Responsibilities
*   **Dynamic Sensitivity Management**: Automatically switches between three pre-defined sensitivity profiles (steps) based on raw light readings.
*   **Hysteresis & Debouncing**: Implements "dwell time" to prevent rapid oscillating between ranges and "skip time" to ignore unstable readings immediately after a hardware setting change.
*   **Saturation Protection**: Includes a "Fast Saturation" check to immediately lower sensitivity if light levels spike, ignoring the standard dwell timer.
*   **Abstraction**: Provides a safe wrapper (`readVEML`) that manages the complexity of the sensor state, hiding it from the main application loop.
*   **Telemetry Support**: Exposes current range state (Name/Index) for logging.

### 2. Key Functions & Variables

| Name | Type | Scope | Description |
| :--- | :--- | :--- | :--- |
| `VEML_STEPS[]` | `const struct` | Local | Lookup table defining 3 profiles: **0=Dark** (High Sens), **1=Day** (Med), **2=Sun** (Low Sens). |
| `g_vemlStepIdx` | `int` | Global | Tracks the current active profile index (Default: 1). |
| `g_vemlLastChangeMs` | `ulong` | Global | Timestamp of the last gain/IT change for dwell timing. |
| `g_vemlSkipUntilMs` | `ulong` | Global | Timestamp indicating when the sensor data becomes valid again after a change. |
| `VEML_ApplyStep(...)` | `void` | Internal | Applies Gain/IT to hardware via I2C and sets the "skip" timer. |
| `VEML_AutoRangeUpdate(...)` | `void` | Internal | Evaluates raw sensor data against thresholds (`LOW`, `HIGH`, `SAT`) and triggers step changes. |
| `readVEML(...)` | `bool` | **Public** | The main access point. Checks if reading is allowed (skip timer), reads HW, and returns success/fail. |
| `VEML_AR_DWELL_MS` | `Define` | Config | Time (6000ms) the system must remain in a state before auto-ranging (except in saturation). |

### 3. State & Logic Flow

The logic functions as a state machine driven by the `raw` sensor value:

1.  **Read Request (`readVEML`)**:
    *   The main loop calls this.
    *   **Check**: Is `millis() < g_vemlSkipUntilMs`?
        *   **Yes**: Return `false`. (Sensor is integrating after a setting change; data is invalid).
        *   **No**: Read `veml.readALS()` and `veml.readLux()`. Return `true`.

2.  **Update Logic (`VEML_AutoRangeUpdate`)**:
    *   Called immediately after a successful read.
    *   **Fast Saturation Check**: If `raw > 60000` (near 16-bit limit) AND we are not at minimum sensitivity, switch to the next lower sensitivity step **immediately** (bypassing dwell timer).
    *   **Dwell Check**: If time since last change < 6000ms, do nothing (stabilize).
    *   **Threshold Check**:
        *   If `raw < 120` (Dark) AND step > 0: **Increase Sensitivity** (Step - 1).
        *   If `raw > 62000` (Bright) AND step < Max: **Decrease Sensitivity** (Step + 1).

3.  **Step Configuration (`VEML_STEPS`)**:
    *   **Step 0 (Darkness)**: Gain x1, Integration 400ms (Max sensitivity).
    *   **Step 1 (Daylight)**: Gain x1/4, Integration 200ms.
    *   **Step 2 (Direct Sun)**: Gain x1/8, Integration 100ms (Min sensitivity to prevent saturation).

### 4. Integration with Other Blocks

*   **Block 01 (Config)**: Relies on `ENABLE_VEML_AUTORANGE`. If this is disabled, the block compiles a fallback version of `readVEML` that acts as a simple pass-through without logic.
*   **Block 03 (Setup)**: Will call `VEML_AutoRangeInit` to set the initial hardware state (Index 1).
*   **Block 04-08 (Loop/Sensors)**: Instead of calling `veml.readALS()` directly, other blocks must call `readVEML()`. They must also call `VEML_AutoRangeUpdate()` passing the raw value back to this block to drive the logic.
*   **External HW**: Relies on the global object `veml` (Adafruit_VEML7700) declared `extern` here but instantiated elsewhere (typically Block 03).

---


## Analysis: Block 03
Here is the detailed analysis of **Block 03**.

### 1. Block Title
**Block 03: Global Objects, Data Structures & State Variables**

### 2. Responsibilities
This block serves as the **memory allocation and state definition layer** of the firmware. It is responsible for:
*   **Driver Instantiation:** Creating global instances for sensor libraries (Adafruit BME280/VEML7700) and network clients (WiFi/MQTT).
*   **Data Structure Definition:** Defining the `SensorData` structure for snapshotting readings and `SensorHistory` for time-series analysis.
*   **Stability Logic Implementation:** Embedding the mathematical logic for calculating rates of change (derivative) and determining signal stability directly within the `SensorHistory` struct.
*   **State Management:** holding global flags for network status, operation modes, and timing.
*   **Memory Management:** Pre-allocating fixed-size `char` buffers (`g_jsonBuffer`) to prevent heap fragmentation (Stack vs. Heap optimization) typically caused by Arduino `String` concatenation.

### 3. Key Functions & Variables

#### Objects & Structures
| Name | Type | Description |
| :--- | :--- | :--- |
| `veml`, `bme` | Driver Objects | Instances of Adafruit sensor libraries. |
| `mqttClient` | `PubSubClient` | MQTT handler wrapping the `wifiClient`. |
| `SensorData` | `struct` | Plain Old Data (POD) structure storing `temp`, `hum`, `press`, `lux`, validity flags, and timestamp. |
| `SensorHistory`| `struct` | **Crucial Logic Component.** A circular buffer implementation that stores historical values and timestamps to calculate trends. |

#### `SensorHistory` Methods (Embedded Logic)
| Method | Description |
| :--- | :--- |
| `init()` | Zeroes out the history buffer. |
| `add(value, time)` | Inserts new reading into the circular buffer and advances the index. |
| `getChangeRate(count)`| Calculates the absolute change per minute (derivative) based on the oldest and newest sample within the `count` window. |
| `isStable(max, count)`| Boolean check returning true if the `getChangeRate` is below the defined `maxChangePerMinute` threshold. |

#### Critical Global Variables
| Variable | Type | Purpose |
| :--- | :--- | :--- |
| `currentSensorData` | `SensorData` | Holds the most recent raw readings from the hardware. |
| `tempHistory`, `pressureHistory`, etc. | `SensorHistory` | Separate history instances for each physical metric to track stability independently. |
| `currentMode` | `OperationalMode` | Stores the active state (NORMAL, TURBO, TURBO_STABLE) determined by the logic engine. |
| `g_jsonBuffer` | `char[]` | Pre-allocated memory for building the MQTT payload. Size defined in Block 02 constants. |
| `chipId` | `char[16]` | Buffer for the unique MCU identifier (replaces `String` for memory safety). |
| `disconnectStartTime` | `unsigned long` | Used for the "Robust Reconnect" logic to track how long the system has been offline. |

### 4. State & Logic Analysis

**1. Circular Buffer Implementation (`SensorHistory`)**
This is not merely storage; it is the core of the automation logic.
*   **Mechanism:** Uses the modulus operator `index = (index + 1) % HISTORY_SIZE` to overwrite old data indefinitely without shifting memory.
*   **Calculus:** The `getChangeRate` function computes:
    $$ \text{Rate} = \frac{|V_{new} - V_{old}|}{T_{new} - T_{old}} \times 60000 $$
    This normalizes the change to "units per minute", allowing the system to compare rapid sampling (Turbo mode) vs. slow sampling (Normal mode) against the same stability thresholds.

**2. Memory Optimization Strategy**
The code explicitly avoids dynamic allocation for strings:
*   Instead of `String json = "{" + var + "}";`, it declares `char g_jsonBuffer[JSON_BUFFER_SIZE];`.
*   This suggests the subsequent code will use `snprintf` or `strcpy`, significantly increasing long-term stability by preventing heap fragmentation, a common issue on ESP8266/ESP32 running for months.

**3. State Persistence**
*   **Network:** `wasEverConnected` allows the logic to distinguish between "booting up" (allow time to connect) and "connection lost" (retry aggressively or reboot).
*   **Hardware:** `bme280Working` and `vemlWorking` flags in `SensorData` allow the main loop to continue running partially if one sensor fails (Partial Failure Support).

### 5. Integration

This block acts as the **central repository** for the entire firmware.

*   **To Block 04 (Setup):** Will initialize the `veml` and `bme` objects and call `tempHistory.init()`.
*   **To Block 05 (Loop/Read):** The read functions will populate `currentSensorData` and call `tempHistory.add()`.
*   **To Block 08 (Logic):** The logic engine will query `tempHistory.isStable()` and read `currentMode` to decide whether to switch transmission intervals.
*   **To Network Blocks:** The MQTT publishing functions will write into `g_jsonBuffer` and read `chipId` to form topics.

**Note on Hardware Platform:**
The code comment `// ESP8266 specifické informace` conflicts with the prompt's context of **ESP32**. However, the variable types (`uint32_t`, `char[]`) are cross-compatible. The logic relies on `chipId` being populated correctly in the Setup block (likely via `ESP.getEfuseMac()` for ESP32 or `ESP.getChipId()` for ESP8266).

---


## Analysis: Block 04
# Block 04: Sensor Management & Operational Logic

This block contains the core logic for hardware interaction, data acquisition, data validation, and the system's adaptive behavior (switching modes based on environmental stability). It bridges the low-level I2C drivers with high-level application state.

## 1. Responsibilities
*   **Hardware Initialization**: Initializing the I2C bus and configuring specific sensors (BME280, VEML7700) with retry logic and address scanning.
*   **Data Acquisition**: Reading raw data from sensors, converting units (Pa to hPa), and applying sanity checks (validating ranges).
*   **Autorange Management**: Handling dynamic gain/integration time adjustments for the VEML7700 light sensor.
*   **History & Stability Tracking**: Pushing new readings into circular buffers and analyzing them to detect if the environment is stable or changing.
*   **Adaptive Sampling**: Automatically switching between FAST, NORMAL, and SLOW operational modes based on the stability analysis.

## 2. Key Functions & Variables

| Element | Type | Description |
| :--- | :--- | :--- |
| `initI2C()` | Function | Starts the Wire library on defined pins (SDA, SCL) at 100kHz. |
| `initSensors()` | Function | Orchestrates BME280 (address auto-detect 0x76/0x77) and VEML7700 startup. Sets `bme280Working` and `vemlWorking` flags. |
| `readSensors()` | Function | The main data gathering routine. Reads hardware, validates data against hardcoded safety limits, and updates `currentSensorData`. Toggles LED for visual feedback. |
| `updateSensorHistories()` | Function | Pushes valid readings into global `HistoryBuffer` objects (defined in Block 02). |
| `checkStability()` | Function | Analyzes variance in history buffers. If all metrics (Temp, Hum, Press, Lux) are within thresholds, triggers stable mode; otherwise triggers fast mode. |
| `setOperationalMode()` | Function | State machine transition handler. Updates `currentSensorInterval` based on the new mode (FAST/NORMAL/SLOW). |
| `currentSensorData` | Struct | **Global** (from Block 02). Holds the latest valid readings and sensor status flags. |
| `bme`, `veml` | Objects | **Global** driver instances for the sensors. |

## 3. State & Logic Flow

### Initialization Phase (`initSensors`)
1.  **BME280**: Attempts connection at `0x76`. If it fails, tries `0x77`. If successful, sets sampling parameters (Oversampling x2/x16/x16, IIR Filter x16).
2.  **VEML7700**: Attempts connection up to 3 times.
    *   If `ENABLE_VEML_AUTORANGE` is active, it calls `VEML_AutoRangeInit`.
    *   Otherwise, it applies fixed settings (Gain 1/8, Integration 100ms).

### Data Reading Cycle (`readSensors`)
1.  **Visual**: Turns on `LED_BUILTIN`.
2.  **BME280**: Reads Temp/Hum/Pressure.
    *   **Validation**: Checks for `NaN` and realistic bounds (e.g., Temp: -40 to 85°C, Pressure: 800-1200 hPa).
    *   Only updates `currentSensorData` if valid.
3.  **VEML7700**: Uses `readVEML()` (an abstraction for reading raw/lux).
    *   If Auto-range is enabled, `VEML_AutoRangeUpdate` is called with the raw value to adjust gain for the *next* cycle.
4.  **Timing**: Calculates and logs the read duration.
5.  **Visual**: Turns off `LED_BUILTIN`.

### Adaptive Stability Logic (`checkStability`)
The system attempts to save power or data bandwidth by slowing down when nothing is happening.
1.  **Check**: Calls `.isStable()` on all four history buffers (`temp`, `humidity`, `pressure`, `lux`).
2.  **Logic**: `isCurrentlyStable = isTempStable && isHumStable && isPresStable && isLuxStable`.
3.  **Transitions**:
    *   **Unstable -> Stable**: Switches mode to `MODE_NORMAL` (slower sampling).
    *   **Stable -> Unstable**: Switches mode to `MODE_FAST` (rapid sampling to capture the event).

## 4. Integration

*   **Exposed to Main Loop**:
    *   `initI2C()` and `initSensors()` are called during `setup()`.
    *   `readSensors()`, `updateSensorHistories()`, and `checkStability()` are called cyclically in the `loop()`.
    *   `currentSensorInterval` is updated by this block but used by the Main Loop to determine sleep/delay duration.
*   **Dependencies**:
    *   Requires **Block 02** (Data Structures) for `SensorData` struct and `HistoryBuffer` classes.
    *   Requires **Block 03** (VEML Utils) for `readVEML`, `VEML_AutoRangeInit`, etc.
    *   Requires Global Config macros (e.g., `INT_FAST`, `TEMP_THRESH`) defined in **Block 01**.
*   **Hardware Abstraction**:
    *   This block isolates the main application logic from the specific I2C addresses and driver calls. If sensors change, only this block needs significant rewriting.

---


## Analysis: Block 05
Here is the detailed technical analysis of **Block 05** (Network & Telemetry).

### 1. Block Title
**Block 05: Network Connectivity & Telemetry (WiFi/MQTT)**

### 2. Responsibilities
This block handles the entire networking stack of the firmware. Its primary responsibilities are:
*   **WiFi Management**: Establishing and maintaining the connection to the Access Point.
*   **MQTT Management**: Handling authentication, connection persistence, Last Will and Testament (LWT), and topic subscriptions.
*   **Data Serialization**: converting raw sensor data (from Block 04) and system metrics into JSON format using efficient memory buffers.
*   **Telemetry Publishing**: Sending Weather data (`TOPIC_WEATHER`) and System status (`TOPIC_SYSTEM`) to the broker.
*   **Self-Healing**: Implementing exponential backoff, connection timeouts, and a "software watchdog" (`checkEmergencyRestart`) to reboot the device if network connectivity remains lost for an extended period.
*   **User Feedback**: Driving the Status LED to indicate connection states visually.

### 3. Key Functions & Variables

| Function / Variable | Type | Description |
| :--- | :--- | :--- |
| **`connectWiFi()`** | Function | Blocking (with timeout) attempt to connect to WiFi. Blinks LED while negotiating. |
| **`connectMQTT()`** | Function | Connects to MQTT broker using a generated Client ID and LWT. Implements exponential backoff on failure. |
| **`publishWeatherData()`** | Function | Serializes `currentSensorData` into JSON and publishes to `TOPIC_WEATHER`. |
| **`publishSystemStatus()`** | Function | Serializes system metrics (IP, Heap, RSSI, VEML range) into JSON and publishes to `TOPIC_SYSTEM`. |
| **`checkConnections()`** | Function | The main polling routine called by the loop. Checks WiFi status every 5s and manages MQTT reconnection. |
| **`checkEmergencyRestart()`** | Function | Logic to force a system reboot if disconnected for > 1 hour or after too many failed MQTT retries. |
| **`updateStatusLed()`** | Function | Non-blocking LED blink patterns based on connectivity state (OK, No MQTT, No WiFi). |
| `g_jsonBuffer` | `char[]` | Global buffer used for `snprintf` to build Weather JSON payloads (prevents heap fragmentation). |
| `g_statusBuffer` | `char[]` | Larger global buffer used specifically for the System Status JSON payload. |
| `reconnectInterval` | `long` | Dynamic interval for MQTT retries (starts at base, doubles on failure). |
| `failedReconnectCount` | `int` | Counter for consecutive MQTT failures, used to trigger emergency restarts. |

### 4. State & Logic Analysis

#### A. Connection Strategy
The system treats WiFi and MQTT as sequential dependencies:
1.  **WiFi**: Connection is attempted. If successful (`WL_CONNECTED`), `wifiConnected` flag is set.
2.  **MQTT**: Connection is attempted only if WiFi is present.
    *   **Client ID**: Generated as `[BASE]-[CHIP_ID_HEX]` to ensure uniqueness on the network.
    *   **LWT**: Sets a "status" topic to `offline` so the broker knows if the device drops unexpectedly.
    *   **Backoff**: If MQTT fails, `reconnectInterval` doubles (Exponential Backoff) to prevent network congestion, capped at `RECONNECT_MAX`.

#### B. JSON Serialization (Memory Optimization)
Unlike typical Arduino sketches that use the `String` class (which causes heap fragmentation over time), this block uses `snprintf` with pre-allocated `char` arrays (`g_jsonBuffer`, `g_statusBuffer`).
*   **Weather JSON**: Contains Temp, Hum, Pressure, Lux.
*   **System JSON**: Contains IP, Uptime, Heap, RSSI, Reconnect Counts, and conditionally VEML Autorange steps (dynamic string construction based on `#if ENABLE_VEML_AUTORANGE`).

#### C. Robustness & Watchdog (`checkEmergencyRestart`)
This is a critical reliability feature for "headless" operation:
*   **Timer**: Tracks `disconnectStartTime`.
*   **Logic**:
    *   If `(now - disconnectStartTime > MAX_DISCONNECT_TIME)` (e.g., 1 hour), call `emergencyRestart()`.
    *   If `failedReconnectCount > MAX_FAILED_RECONNECTS`, call `emergencyRestart()`.
*   **Reset**: If both WiFi and MQTT reconnect successfully, the timers and counters are reset.

#### D. Visual Status (LED)
The `updateStatusLed` function provides diagnostic codes without blocking code execution:
*   **Short Blink**: Fully Online (WiFi + MQTT).
*   **Double Blink**: WiFi Connected, MQTT Disconnected.
*   **Long Blink**: No WiFi.

### 5. Integration

*   **Inputs**:
    *   **From Block 01/02**: Configuration constants (`WIFI_SSID`, `MQTT_BROKER`, timeouts).
    *   **From Block 04**: `currentSensorData` (struct containing the actual float values to publish).
    *   **From Block 08 (VEML)**: `VEML_GetCurrentStepIndex()` for debugging autorange status.
*   **Outputs**:
    *   **To MQTT Broker**: JSON payloads via `mqttClient.publish`.
    *   **To Block 07 (Main Loop)**: Updates global flags `wifiConnected` and `mqttConnected` which control whether the main loop attempts to read sensors or sleep.
    *   **To Block 08 (Helpers)**: Calls `emergencyRestart()` (forward declared here, defined later) if recovery fails.

---


## Analysis: Block 06
Here is the detailed analysis of **Block 06 (System Initialization)**.

### 1. Block Title
**Block 06: System Initialization (Setup)**

### 2. Responsibilities
This block serves as the entry point for the firmware execution (`setup()`). Its primary role is to transition the hardware from a powered-off state to a fully operational state before handing control to the main loop.
*   **Hardware Bootstrapping**: Initializes Serial communication, GPIOs (LED), and records boot time.
*   **Diagnostic Reporting**: Prints detailed hardware specs (Chip ID, Flash, Heap), active compilation features, and memory buffer configurations to the Serial Monitor.
*   **Sub-system Initialization**: Starts the I2C bus, initializes physical sensors (BME280, VEML7700), and clears data history buffers.
*   **Connectivity Setup**: Establishes the WiFi connection and configures the MQTT client (server, timeouts, keep-alive).
*   **System Priming**: Performs an initial sensor read to ensure valid data exists immediately upon entering the main loop.

### 3. Key Functions & Variables

| Function/Variable | Type | Description |
| :--- | :--- | :--- |
| `setup()` | **Function** | Standard Arduino entry point; executes once at startup. |
| `initI2C()` | Function Call | Attempts to start the I2C bus. **Critical**: Halts system if it fails. |
| `initSensors()` | Function Call | Configures BME280/VEML7700. Non-critical failure (system warns but continues). |
| `connectWiFi()` | Function Call | Connects to the AP defined in `secrets.h`. |
| `bootTime` | `unsigned long` | Stores `millis()` at startup for uptime calculations. |
| `chipId` | `char[]` | Buffer holding the unique ESP8266 identifier (hex formatted). |
| `tempHistory`, `pressureHistory`, etc. | Objects | Custom ring-buffer objects initialized here via `.init()`. |
| `mqttClient` | Object | Configured with server IP, port, keep-alive (60s), and socket timeout (30s). |
| `lastSensorRead`, `lastStatusReport` | `unsigned long` | Timers reset to current `millis()` at the end of setup to synchronize loops. |
| `ENABLE_VEML_AUTORANGE` | Macro | Conditional logic to display active Lux sensor gain strategy. |

### 4. State & Logic Flow
The execution flow within `setup()` is strictly sequential:

1.  **Boot & UI**: Serial starts (115200 baud). The "Zonio ESP8266" banner is printed. The LED is set to OFF (High, due to inverted logic).
2.  **Hardware Introspection**:
    *   Retrieves Chip ID and Flash size.
    *   **Optimization**: Uses `snprintf` into `char` arrays rather than `String` concatenation to prevent heap fragmentation.
3.  **Feature Validation**:
    *   Checks `#ifndef USE_SECRETS_FILE` to warn about security risks.
    *   Reports if **VEML Autorange** is active (listing steps and thresholds) or if Fixed mode is used.
    *   Detects VEML library API version (Macro vs. Enum) for compatibility.
4.  **Hardware Initialization (Critical Path)**:
    *   Calls `initI2C()`. **Logic Stop**: If this fails, the code enters an infinite `while(true)` loop, blinking the LED slowly. The system cannot proceed without I2C.
    *   Calls `initSensors()`. If this fails, the LED blinks fast 6 times, but execution **continues**.
5.  **Data Structure Init**:
    *   Calls `.init()` on all history trackers (Temperature, Humidity, Pressure, Lux).
    *   Sets defaults: `currentMode = MODE_NORMAL`.
6.  **Network Setup**:
    *   Connects to WiFi (blocking call until connected or timeout).
    *   Configures MQTT parameters (topics, payload format info printed for debugging).
7.  **System Priming**:
    *   Calls `readSensors()` and `updateSensorHistories()` immediately. This prevents the first iteration of the `loop()` from having zero/null values.
    *   Signals success (3 fast LED blinks).
    *   Resets all software timers (`last...` variables) to `millis()`.

### 5. Integration Relationships
*   **Dependencies**:
    *   **Block 01/02**: Relies entirely on global configuration macros (`MQTT_SERVER`, `ENABLE_VEML_AUTORANGE`) and global variables (`tempHistory`).
    *   **Block 03/04**: Calls `initSensors()` and `connectWiFi()` defined in previous blocks.
*   **Exposes**:
    *   **Initialized State**: When this block finishes, the global `mqttClient` is configured (but not yet connected), sensors are ready, and `currentMode` is set.
    *   **Buffers**: The `history` arrays are cleared and ready to accept data from the main loop.
    *   **Flow Control**: The `loop()` (Block 08) relies on the timers (`lastSensorRead`, etc.) reset at the very end of this block to know when to trigger the first scheduled task.

---


## Analysis: Block 07
Here is the detailed technical analysis of **Block 07** (Main Loop & System Health).

### 1. Block Title
**Block 07: Main Executive Loop, System Diagnostics & Health Monitoring**

### 2. Responsibilities
This block acts as the **Central Orchestrator** of the firmware. It ties together all previous modules (Network, Sensors, History, Stability) into a cohesive runtime environment. Its specific responsibilities include:
*   **Task Scheduling**: implementing a non-blocking, time-sliced scheduler using `millis()` to handle tasks with different frequencies (sensors, status reports, debug prints).
*   **System Lifecycle**: managing uptime counting, connection recovery (`checkConnections`), and emergency restarts.
*   **Data Aggregation & Telemetry**: triggering sensor reads, updating history buffers, and dispatching data via MQTT.
*   **Human-Readable Diagnostics**: rendering complex system state (stability rates, VEML autorange steps, memory usage) into formatted ASCII tables for the Serial Monitor.
*   **Self-Protection**: monitoring Heap memory health and feeding the hardware Watchdog Timer (WDT) to prevent lockups.

### 3. Key Functions & Variables

| Function / Variable | Type | Description |
| :--- | :--- | :--- |
| `loop()` | **Core** | The infinite loop provided by the Arduino framework. Acts as the main scheduler. |
| `printDetailedStatus()` | Function | Generates a full-page ASCII dashboard to Serial, showing WiFi/MQTT state, sensor values, stability calculation rates, and hardware info. |
| `checkMemoryHealth()` | Function | Monitors `ESP.getFreeHeap()`. Triggers alerts if memory is low (<5KB) and forces a restart if the condition persists to prevent undefined behavior. |
| `emergencyRestart()` | Function | Performs a graceful reboot: publishes "restarting" status to MQTT, flashes the LED, and calls `ESP.restart()`. |
| `uptimeSeconds` | Global `ulong` | Incremented every second to track system run duration. |
| `lastSensorRead` | Static `ulong` | Timestamp of the last sensor acquisition cycle. |
| `lastStatusReport` | Static `ulong` | Timestamp of the last MQTT system status JSON publication. |
| `currentSensorInterval` | Global `long` | Dynamic interval (defined in Block 03) used to throttle the sensor loop based on the operational mode. |

### 4. State & Logic Flow

The `loop()` function operates on a "Check and Act" logic without blocking delays (except for a minimal `delay(50)` for electrical stability).

#### A. The Main Schedule
The loop evaluates `millis()` against several timers:
1.  **Connection Manager**: Calls `checkConnections()` (from Block 02) every iteration to ensure WiFi/MQTT are alive.
2.  **Uptime Ticker**: Every 1000ms, increments `uptimeSeconds`.
3.  **Sensor Cycle** (Variable Interval):
    *   **Trigger**: `now - lastSensorRead >= currentSensorInterval`.
    *   **Action**:
        1.  Prints "SENSOR CYCLE START".
        2.  Calls `readSensors()` (Block 05) to fetch BME280/VEML7700 data.
        3.  Calls `updateSensorHistories()` (Block 04) to push new data into ring buffers.
        4.  Calls `checkStability()` (Block 03) to calculate slopes and determine Dynamic Mode.
        5.  **Telemetry**: If MQTT is connected, publishes weather data.
        6.  **Debug**: Prints calculated change rates vs. thresholds (e.g., "Temp: 0.1°C/min (limit 0.5)").
        7.  **VEML Info**: Prints Autorange step index if enabled.
4.  **Status Reporting**:
    *   **Trigger**: `STATUS_INT` (usually 60s or 5m).
    *   **Action**: Publishes system health JSON (IP, uptime, RSSI) to `TOPIC_STATUS`.
5.  **Memory Watchdog**:
    *   **Trigger**: Every 30 seconds.
    *   **Action**: Checks Free Heap. If `< 5000` bytes for 5 consecutive checks, triggers `emergencyRestart()`.

#### B. Diagnostic Logic (`printDetailedStatus`)
This function formats global state into an ASCII report. It handles conditional compilation (`#if ENABLE_VEML_AUTORANGE`) to display either:
*   **Fixed Mode**: "G1/8 IT100ms".
*   **Autorange Mode**: Current Step Index, Gain/Integration times, and raw thresholds.

#### C. Safety Mechanisms
*   **Soft WDT**: `ESP.wdtFeed()` is called at the end of every loop iteration to prevent the hardware watchdog from resetting the ESP32 due to CPU starvation.
*   **Robust Reconnect**: `checkEmergencyRestart()` is called to handle cases where WiFi fails to reconnect for a prolonged period (logic defined in Block 02).

### 5. Integration

As the final block, this code integrates **all** previous blocks. It does not export variables for other blocks to use; rather, it consumes the Global State established by them.

**Dependencies Consumed:**
*   **From Block 01 (Config)**: `DEVICE_TYPE`, `FIRMWARE_VERSION`, `STATUS_INT`, MQTT Topics.
*   **From Block 02 (Network)**: `wifiConnected`, `mqttConnected`, `checkConnections()`, `publishWeatherData()`, `publishSystemStatus()`.
*   **From Block 03 (Logic)**: `currentMode`, `stabilityDetected`, `getOperationalModeString()`.
*   **From Block 04 (History)**: `tempHistory`, `luxHistory`, etc., and their `.getChangeRate()` methods.
*   **From Block 05 (Sensors)**: `currentSensorData`, `readSensors()`, `VEML_GetCurrentStepIndex()`.

**System State Management**:
This block maintains the **temporal integrity** of the firmware. While previous blocks define *how* to read a sensor or *how* to calculate stability, this block defines *when* those actions happen, ensuring the ESP32 remains responsive to network callbacks while processing sensor data.

---
