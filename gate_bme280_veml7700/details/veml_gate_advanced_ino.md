# Deep Technical Documentation: veml_gate_advanced.ino

> **Monolith Analysis**: Decomposed into 20 blocks.



## Analysis: Block 01
# Block 01: Includes & Compatibility Layer

## 1. Overview
This block serves as the foundation for the entire firmware. Its primary role is **environment preparation** and **hardware abstraction**. It imports the necessary drivers for the ESP8266 architecture, networking protocols (WiFi, MQTT), and specific I2C sensors (VEML7700, BME280).

Crucially, it implements a **Compatibility Layer** for the `Adafruit_VEML7700` library. This library underwent API changes between versions (shifting from Enums to Macros/`uint8_t`), and this block uses preprocessor directives to ensure the code compiles successfully regardless of the library version installed in the IDE.

**Note on Hardware**: While the system context indicates ESP32 usage, the specific includes in this block (`ESP8266WiFi.h`) confirm this specific node is running on an **ESP8266 D1 Mini**.

## 2. Responsibilities
*   **Library Imports**: Loading core dependencies for WiFi, WebServer, MQTT, I2C, EEPROM, and JSON handling.
*   **Sensor Driver Loading**: Importing drivers for VEML7700 (Light) and BME280 (Temp/Hum/Pressure).
*   **API Normalization**: Abstracting the VEML7700 gain and integration time settings into unified types (`VGain_t`, `VIt_t`) and wrapper functions to prevent compilation errors caused by library updates.

## 3. Key Macros & Types

| Name | Type | Description |
| :--- | :--- | :--- |
| `VGain_t` | Type definition | Abstracts the data type for Sensor Gain. Resolves to `uint8_t` (New API) or `veml7700_gain_t` (Old API). |
| `VIt_t` | Type definition | Abstracts the data type for Integration Time. Resolves to `uint8_t` (New API) or `veml7700_integrationtime_t` (Old API). |
| `VGAIN_x` | Macro | Normalized constants for gain (e.g., `VGAIN_1`, `VGAIN_1_4`) mapped to the underlying library specific constants. |
| `VIT_x` | Macro | Normalized constants for integration time (e.g., `VIT_25`, `VIT_100`) mapped to underlying library constants. |

## 4. Key Functions (Inline Wrappers)

| Function | Signature | Purpose |
| :--- | :--- | :--- |
| `VEML_SetGain` | `void VEML_SetGain(Adafruit_VEML7700& d, VGain_t g)` | A wrapper that calls `d.setGain(g)`. It creates a uniform function call signature regardless of the underlying data types required by the library. |
| `VEML_SetIT` | `void VEML_SetIT(Adafruit_VEML7700& d, VIt_t it)` | A wrapper that calls `d.setIntegrationTime(it)`. Ensures integration time settings are applied correctly across different library versions. |

## 5. State & Logic Flow
This block does not contain runtime execution flow (loops or setup). Instead, it contains **Compile-Time Logic**:

1.  **Check for Macros**: The preprocessor checks if `VEML7700_GAIN_1` and `VEML7700_IT_100MS` are defined.
2.  **Branch 1 (New API)**: If defined, the library is using `#define` macros. The code defines `VEML_API_MACRO`. Types are aliased to `uint8_t`.
3.  **Branch 2 (Old API)**: If not defined, the library is using Enums. The code defines `VEML_API_ENUM`. Types are aliased to the specific library Enums.
4.  **Unification**: Regardless of the branch taken, the code exposes the standard `VGAIN_...` macros and `VEML_Set...` functions to the rest of the firmware.

## 6. Integration with Other Blocks
This block provides the necessary **Type Definitions** and **Global Constants** required by subsequent blocks:
*   **Block 02 (Globals)**: Will likely instantiate the `Adafruit_VEML7700` and `Adafruit_BME280` objects using headers included here.
*   **Sensor Logic Blocks**: Will use `VGain_t` and `VEML_SetGain` to adjust lux sensitivity without needing to know which library version is installed.
*   **Networking Blocks**: Will rely on `ESP8266WiFi.h` and `PubSubClient.h` included here.

---


## Analysis: Block 02
# Block 02: Configuration & Constants

## 1. Responsibilities
This block acts as the **Static Configuration Layer** for the entire firmware. It provides a centralized location for system-wide definitions, decoupling the application logic from specific hardware pins, protocol keys, and tuning parameters.

Its specific responsibilities include:
*   **Hardware Abstraction**: Mapping physical GPIO pins to functional names (I2C, LED, Inputs).
*   **Identity Management**: Defining firmware versioning (`FW_VERSION`) and device type strings used for OTA and registration.
*   **Protocol Contract**: Establishing the specific string keys used for MQTT JSON payloads, ensuring consistency between the device and the backend dashboard.
*   **Operational Tuning**: Setting critical timeouts, loop intervals, and reconnection backoff strategies.
*   **Sensor Calibration**: Defining thresholds for the VEML7700 auto-ranging algorithm.

## 2. Key Constants & Definitions

| Category | Constant Name | Value | Description |
| :--- | :--- | :--- | :--- |
| **Identity** | `FW_VERSION` | "v3.1.0..." | Used for OTA checks and MQTT status messages. |
| | `DEVICE_TYPE` | "veml7700..." | Identifies device capabilities to the server. |
| **Hardware** | `I2C_SDA` / `SCL` | D2 (GPIO4) / D1 (GPIO5) | Pin definitions for the I2C bus (BME280/VEML7700). |
| | `PIN_FIND_GATE` | 12 (GPIO12) | Input pin, likely a button/jumper to trigger "Config Gate" mode. |
| **Config** | `GATE_AP_SSID` | "Zonio-Gate" | SSID broadcasted when the device enters AP configuration mode. |
| | `EEPROM_SIZE` | 512 | Memory allocation size for storing WiFi/MQTT credentials. |
| **Timing** | `INT_FAST` | 500 ms | The primary sensor polling rate. |
| | `INT_STATUS` | 60,000 ms | The interval for sending "heartbeat" status updates to MQTT. |
| | `RECONNECT_MAX` | 300,000 ms | Caps the exponential backoff for network reconnection at 5 minutes. |
| **VEML** | `VEML_AR_RAW_LOW` | 120 | Lower threshold of raw counts; triggers gain increase. |
| | `VEML_AR_RAW_HIGH` | 62000 | Upper threshold of raw counts; triggers gain reduction to prevent saturation. |

## 3. State & Logic
While this block contains no executable code (functions), it defines the **bounds and behavior** of the logic in subsequent blocks:

1.  **Network Strategy**: The definitions `WIFI_CONNECT_TIMEOUT` and `WIFI_CONNECT_RETRIES` dictate that the connection logic (likely in Block 03 or 04) will attempt to connect for exactly 10 seconds before failing over (likely to AP mode or Reboot).
2.  **Data Structure**: The MQTT Keys (e.g., `KEY_TEMP`, `KEY_LUX`) imply that the main loop will construct a JSON object. The separation of `KEY_STATUS` vs `KEY_TEMP` suggests the firmware sends two types of messages: Telemetry (Environment) and State (System health).
3.  **Autorange Logic**: The VEML constants (`RAW_LOW`, `RAW_HIGH`, `DWELL_MS`) configure a state machine for the light sensor. The logic will likely look like:
    *   *If raw > 62000*: Decrease sensitivity immediately.
    *   *If raw < 120*: Increase sensitivity.
    *   *Dwell*: Wait `VEML_AR_DWELL_MS` (3s) before switching again to prevent "flickering" between gain settings.

## 4. Integration with Other Blocks
This block is the dependency root for the rest of the firmware.
*   **Block 01 (Libraries)**: Uses these constants to initialize objects (e.g., passing `I2C_SDA` to `Wire.begin()`).
*   **Global Variables (Block 03)**: Will likely use `EEPROM_SIZE` to initialize storage buffers.
*   **Setup/Loop (Block 09/10)**: Will rely on `INT_FAST` and `INT_STATUS` to manage non-blocking delay timers (likely `millis()` subtraction logic).
*   **MQTT Handler**: Will use the `KEY_` macros to serialize data. Changing a key here updates the entire protocol instantly.

---


## Analysis: Block 03
# Block 03: Data Structures

## 1. Responsibilities
This block defines the fundamental **data blueprints** used throughout the monolithic firmware. It does not contain executable code but establishes the memory layout for:
*   **Non-Volatile Storage (EEPROM):** How configuration, credentials, and settings are serialized and saved to flash memory.
*   **Runtime State:** A unified structure to hold live sensor readings and hardware health status.
*   **Sensor Calibration:** A structure to define gain and integration time tuples for the Lux sensor's auto-ranging algorithm.

## 2. Key Structures & Members

### `struct DeviceConfig`
This structure defines the memory map for the simulated EEPROM.

| Member Variable | Type | Description |
| :--- | :--- | :--- |
| `validFlag` | `uint16_t` | **Magic Number** (`0xC6A7`). Used to detect if the EEPROM contains initialized data or garbage (factory fresh). |
| `wifi_ssid` | `char[64]` | Storage for Wi-Fi Network Name. |
| `wifi_password` | `char[64]` | Storage for Wi-Fi Password. |
| `mqtt_server` | `char[64]` | MQTT Broker URL/IP. |
| `mqtt_port` | `uint16_t` | MQTT Broker Port (standard 1883). |
| `mqtt_user` | `char[32]` | MQTT Username. |
| `mqtt_pass` | `char[32]` | MQTT Password. |
| `mqtt_base_topic` | `char[64]` | Root topic path (e.g., `zonio/weather/DP1`) used to build telemetry/command subtopics. |
| `veml_autorange` | `bool` | Feature flag: Enables dynamic gain adjustment for the Lux sensor. |
| `checksum` | `uint32_t` | Used to verify data integrity upon loading from EEPROM. |

### `struct SensorData`
The centralized container for environmental metrics.

| Member Variable | Type | Description |
| :--- | :--- | :--- |
| `temperature` | `float` | Current ambient temperature (°C). |
| `humidity` | `float` | Current relative humidity (%). |
| `pressure` | `float` | Current atmospheric pressure (hPa). |
| `lux` | `float` | Current light intensity (Lux). |
| `bme280Working` | `bool` | Health flag: `true` if BME280 initializes and returns valid data. |
| `vemlWorking` | `bool` | Health flag: `true` if VEML7700 initializes and returns valid data. |
| `lastUpdate` | `ulong` | Timestamp (`millis()`) of the last successful sensor read cycle. |

### `struct VemlRangeStep`
Used for the auto-ranging logic lookup table.

| Member Variable | Type | Description |
| :--- | :--- | :--- |
| `gain` | `VGain_t` | Enum from VEML library defining amplifier gain (e.g., 1/8, 1/4, 1, 2). |
| `it` | `VIt_t` | Enum from VEML library defining Integration Time (e.g., 25ms, 100ms). |
| `name` | `const char*` | Human-readable string for debugging (e.g., "G=1/8 IT=25ms"). |

## 3. State & Logic Analysis

While this block is declarative, it enforces specific logic patterns for the rest of the firmware:

1.  **Boot Validation Logic:**
    *   The inclusion of `validFlag` (0xC6A7) dictates that the startup routine (likely Block 04 or 05) must read the first 2 bytes of EEPROM.
    *   If the bytes do not match `0xC6A7`, the system assumes a "Factory Reset" state and likely loads default values or enters a configuration mode (Access Point).

2.  **Fault Tolerance:**
    *   `SensorData` includes `bme280Working` and `vemlWorking`. This implies the main loop will check these flags before attempting to read sensors or publish MQTT data. If a sensor fails, the system continues running (partial failure mode) rather than crashing.

3.  **Optimization:**
    *   `VemlRangeStep` suggests the auto-ranging algorithm is deterministic. Instead of calculating gain on the fly, the system will likely iterate through a pre-defined array of these structs to find the optimal setting for current light conditions.

## 4. Integration with Other Blocks

*   **Block 02 (Libraries):** Dependent on Block 02 for `VGain_t` and `VIt_t` type definitions (likely from `Adafruit_VEML7700.h`).
*   **Globals (Upcoming Blocks):** These structures will be instantiated as global variables (e.g., `DeviceConfig config;`, `SensorData currentData;`) in a subsequent block to hold actual values.
*   **Web Server / WiFi:** The `wifi_ssid` and `password` buffers in `DeviceConfig` link directly to the connection logic.
*   **MQTT Logic:** The `mqtt_base_topic` is critical for string concatenation logic when publishing sensor data.

---


## Analysis: Block 04
# Block 04: Global Variable Instantiation & State Management

## 1. Responsibilities
This block instantiates the global objects and state variables required for the system's operation. It serves as the **runtime memory allocation** layer, connecting the data structures defined in Block 03 with the drivers and logic that will follow.

Its specific responsibilities include:
*   **Hardware Abstraction**: Instantiating driver objects for I2C sensors (VEML7700, BME280).
*   **Network Client Setup**: creating instances for WiFi, MQTT, and the Web Server.
*   **Scheduler Initialization**: Defining `unsigned long` timestamp trackers for non-blocking multitasking.
*   **Stability State**: Initializing counters and flags for the connection watchdog and exponential backoff algorithms.
*   **Autorange Logic**: Defining the specific gain/integration steps for the VEML7700 sensor algorithm.

## 2. Key Variables & Objects

| Category | Variable / Object | Type | Description |
| :--- | :--- | :--- | :--- |
| **Config** | `deviceConfig` | `DeviceConfig` | Holds runtime configuration loaded from EEPROM/Preferences (WiFi creds, MQTT topics). |
| | `isGateMode` | `bool` | Flag determining if device acts as an RS485 gate or a standalone sensor node. |
| **Sensors** | `currentSensorData` | `SensorData` | Central struct holding latest readings (Lux, Temp, Hum, Pressure) for transmission. |
| | `veml` | `Adafruit_VEML7700` | Driver instance for the Lux sensor. |
| | `bme` | `Adafruit_BME280` | Driver instance for Environmental sensor. |
| **Network** | `mqttClient` | `PubSubClient` | Handles MQTT protocol (Publish/Subscribe). Wraps `wifiClient`. |
| | `configServer` | `ESP8266WebServer` | HTTP server on Port 80 for configuration mode (AP mode). |
| **Timing** | `lastSensorRead` | `unsigned long` | Timestamp for the sensor polling interval. |
| | `uptimeSeconds` | `unsigned long` | Tracks total device uptime (independent of `millis()` rollover). |
| **Stability** | `mqttReconnectInterval` | `unsigned long` | **Dynamic** interval for MQTT reconnection (implements exponential backoff). |
| | `disconnectStartTime` | `unsigned long` | Timestamp when connection was lost; used to trigger a hardware reset after 1 hour of downtime. |
| **Device** | `chipId` | `char[16]` | Stores the unique ESP32 MAC/Chip ID used for MQTT Client ID and Topic paths. |

## 3. State & Logic Analysis

### A. Non-Blocking Timing State
The block initializes a suite of `unsigned long` variables initialized to `0`. These are essential for the `loop()` (in future blocks) to perform multitasking without using `delay()`.
*   **Logic**: `current_time - last_event > interval`
*   **Variables**: `lastSensorRead`, `lastStatusReport`, `lastMQTTReconnectAttempt`, `lastWiFiReconnectAttempt`.

### B. Connection Watchdog State
Variables here imply a robust error recovery strategy:
*   `mqttReconnectInterval`: Initialized to `RECONNECT_BASE` (from Block 02). This variable will likely double on every failure in later logic (1s -> 2s -> 4s...).
*   `wasEverConnected`: A boolean latch. If the device reboots and *never* connects, it might behave differently (e.g., go to AP mode immediately) compared to a device that was working and lost signal.

### C. VEML7700 Autorange State Machine
This section (`#if ENABLE_VEML_AUTORANGE`) defines the logic for handling extreme light conditions (darkness vs. direct sun) by adjusting Gain and Integration Time (IT).
*   **`VEML_STEPS` Array**: A Lookup Table defining 3 distinct sensitivity levels:
    1.  **Low Light**: Gain 1, IT 400ms (Max sensitivity).
    2.  **Medium**: Gain 1/4, IT 200ms.
    3.  **Bright**: Gain 1/8, IT 100ms (Min sensitivity).
*   **State Variables**:
    *   `g_vemlStepIdx`: Tracks current index in the steps array (starts at index 1: Medium).
    *   `g_vemlSkipUntilMs`: A "debounce" timer to prevent oscillating between ranges too quickly.

## 4. Integration

This block acts as the **Data Link** between definitions and execution:

*   **From Previous Blocks**:
    *   Uses struct definitions (`DeviceConfig`, `SensorData`, `VemlRangeStep`) from **Block 03**.
    *   Uses configuration macros (`RECONNECT_BASE`, `ENABLE_VEML_AUTORANGE`) from **Block 02**.
*   **To Future Blocks**:
    *   **Block 05/06 (Setup)**: Will initialize `veml.begin()`, `bme.begin()`, and load data into `deviceConfig` via EEPROM.
    *   **Block 07+ (Loop)**: The variables defined here (`currentSensorData`, `uptimeSeconds`) will be constantly mutated by the main logic loop.
    *   **Networking**: The `mqttClient` and `configServer` objects defined here are global, allowing callback functions in later blocks to access network resources directly.

---


## Analysis: Block 05
Here is the detailed technical analysis for **Block 05** of the monolithic firmware.

## Block 05: Configuration Management (EEPROM)

This block provides the non-volatile storage layer for the system. It handles the serialization, validation, and persistence of the global configuration structure (`deviceConfig`) using the ESP32's emulated EEPROM (Flash). It also includes the necessary C++ forward declarations to ensure compilation stability for the monolithic file structure.

### 1. Responsibilities
*   **Data Persistence**: Saving system settings (WiFi credentials, MQTT endpoints, sensor intervals, thresholds) to non-volatile memory so they survive power cycles.
*   **Data Integrity Verification**: Ensuring loaded data is not garbage (e.g., from a fresh chip or corrupted flash sector) using "Magic Flags" and Checksums.
*   **Compilation Linking**: Providing forward declarations for functions defined in later blocks (e.g., `setupGateMode`, `loopNormalMode`), allowing the compiler to resolve calls made in earlier blocks (like `setup()`).
*   **Flash Management**: Handling the commit process to the ESP32's flash storage partition.

### 2. Key Functions & Variables

| Element | Type | Description |
| :--- | :--- | :--- |
| **`calculateChecksum`** | Function | Calculates a 32-bit integrity hash of the `deviceConfig` structure. Uses `offsetof` to exclude the checksum field itself from the calculation. |
| **`loadConfig`** | Function | Reads raw bytes from EEPROM into RAM (`deviceConfig`). Validates the `validFlag` and compares the computed checksum against the stored checksum. Returns `true` only if data is valid. |
| **`saveConfig`** | Function | Updates the `validFlag` and checksum in the struct, writes the struct to the EEPROM buffer, and calls `EEPROM.commit()` to write to physical flash. |
| `deviceConfig` | Global Var | **External Dependency** (defined in Block 02/03). The actual data structure being manipulated. |
| `CONFIG_VALID_FLAG` | Constant | **External Dependency**. A magic number (e.g., `0xA5A5A5A5`) used to quickly check if the EEPROM has ever been initialized. |
| `EEPROM_SIZE` | Constant | **External Dependency**. Defines the size of the flash partition reserved for settings. |

### 3. State & Logic Flow

The logic within this block focuses on the **Integrity-First** approach to loading settings:

#### Checksum Algorithm
The `calculateChecksum` function implements a basic hashing algorithm:
1.  Iterates through the `deviceConfig` struct byte-by-byte.
2.  Stops exactly before the `checksum` field (using `offsetof`).
3.  Performs an accumulation with a **Circular Left Shift** (`(sum << 1) | (sum >> 31)`). This ensures that the position of bytes matters (preventing `AB` and `BA` resulting in the same sum).

#### Load Sequence (`loadConfig`)
1.  **Initialize**: `EEPROM.begin(EEPROM_SIZE)`.
2.  **Fetch**: `EEPROM.get(0, deviceConfig)` reads the data into the global variable.
3.  **Sanity Check**: Checks if `deviceConfig.validFlag` equals `CONFIG_VALID_FLAG`. If not, the memory is considered uninitialized or corrupt.
4.  **Integrity Check**:
    *   Reads the `checksum` stored in the struct.
    *   Recalculates the checksum based on the data fields.
    *   Compares them. If they mismatch, the configuration is rejected.
5.  **Result**: Returns `false` if any check fails, triggering the system to likely enter a default or "Gate Mode" (configuration mode) in the main setup.

#### Save Sequence (`saveConfig`)
1.  **Prepare**: Sets the `validFlag` and calculates a fresh `checksum` based on current values.
2.  **Write**: `EEPROM.put(0, deviceConfig)` writes to the RAM buffer.
3.  **Commit**: `EEPROM.commit()` performs the actual flash write operation. This is critical on ESP32; without it, changes are lost on reboot.

### 4. Integration

This block is the bridge between runtime logic and persistent storage.

*   **Upstream Integration (Inputs)**:
    *   Requires the `DeviceConfig` struct definition (Block 02).
    *   Requires constants `EEPROM_SIZE` and `CONFIG_VALID_FLAG` (Block 01/02).
*   **Downstream Integration (Outputs)**:
    *   **Modified Global State**: Populates `deviceConfig` with trusted data.
    *   **Control Flow Signals**: `loadConfig()` returns a boolean that determines the boot path in **Block 03 (Setup)**:
        *   If `true`: System proceeds to **Normal Mode** (Block 08).
        *   If `false`: System enters **Gate Mode** (Access Point) (Block 06).
*   **Web API Usage**: The `handleConfig` function (Block 06) calls `saveConfig` after receiving JSON data from the web dashboard to persist changes.

---


## Analysis: Block 06
Here is the detailed analysis of **Block 06: VEML7700 Autorange Implementation**.

### 1. Block Title
**Block 06: VEML7700 Autorange Implementation**

### 2. Responsibilities
This block encapsulates the dynamic sensitivity logic for the light sensor. Its primary goal is to ensure valid readings across a massive dynamic range (from dim indoor light to direct sunlight) without manual intervention.

*   **Dynamic Sensitivity Management**: Automatically switches between pre-defined Gain and Integration Time combinations (`VEML_STEPS`) based on raw sensor readings.
*   **Sensor Protection & Stabilization**: Enforces "blind" periods (`g_vemlSkipUntilMs`) immediately after changing settings to prevent reading garbage data while the sensor stabilizes.
*   **Hysteresis & Dwell Control**: Prevents rapid oscillation between ranges (flickering) using dwell timers and overlapping low/high thresholds.
*   **Data Access Abstraction**: Provides a wrapper (`readVEML`) that creates a safe interface for the main loop to request data only when the sensor is ready.

### 3. Key Functions & Variables

| Element | Type | Description |
| :--- | :--- | :--- |
| **`VEML_ApplyStep`** | Function | Applies a specific configuration step (Index 0-N) to the hardware. Updates the global index, sets the dwell timer, and calculates the stabilization delay. |
| **`VEML_AutoRangeUpdate`** | Function | The core decision engine. Takes the current `raw` reading and decides if the sensitivity needs to be increased (step - 1) or decreased (step + 1). |
| **`readVEML`** | Function | **Crucial Interface**. Attempts to read the sensor. Returns `false` if the system is currently in a stabilization period (data invalid). Returns `true` and populates references if safe. |
| **`g_vemlStepIdx`** | Global (State) | The current index pointer into the `VEML_STEPS` array (defined in Block 05). |
| **`g_vemlSkipUntilMs`** | Global (Timer) | Timestamp. Readings are forbidden until `millis()` exceeds this value. Set after every configuration change. |
| **`VEML_AR_FAST_SAT`** | Constant | Threshold for immediate saturation. If raw value hits this, the system decreases sensitivity immediately, ignoring dwell time. |
| **`VEML_AR_DWELL_MS`** | Constant | Minimum time the system must stay in a specific range before switching again (prevents oscillation). |

### 4. State & Logic Analysis

The logic flows sequentially through `VEML_AutoRangeUpdate` and `readVEML`.

**A. The Autorange Decision Logic (`VEML_AutoRangeUpdate`)**
1.  **Fast Saturation Check**:
    *   If `raw` reading is $\ge$ `VEML_AR_FAST_SAT` (near 65535):
    *   **Action**: Immediately move to the next step (lower sensitivity).
    *   **Reason**: The sensor is blinded; we need valid data immediately. This bypasses the dwell timer.
2.  **Dwell Time Check**:
    *   If `millis()` < (`g_vemlLastChangeMs` + `VEML_AR_DWELL_MS`):
    *   **Action**: Return immediately. Do not change settings.
    *   **Reason**: Prevents the system from "bouncing" between two steps rapidly if the light level is right on the boundary.
3.  **Low Threshold Check**:
    *   If `raw` < `VEML_AR_RAW_LOW` (reading is too small for precision) AND current step $> 0$:
    *   **Action**: Move to `step - 1` (Higher sensitivity: Higher Gain or Longer Integration).
4.  **High Threshold Check**:
    *   If `raw` > `VEML_AR_RAW_HIGH` (reading is nearing max) AND current step is not maxed out:
    *   **Action**: Move to `step + 1` (Lower sensitivity: Lower Gain or Shorter Integration).

**B. The Safe Read Logic (`readVEML`)**
*   Unlike standard `veml.readALS()`, this function returns a `bool`.
*   It checks `g_vemlSkipUntilMs`. If the time hasn't passed, it returns `false`, effectively telling the main loop: *"I adjusted the lens recently, the image is still blurry, come back later."*

### 5. Integration

This block acts as the **Driver Layer** between the raw hardware definitions (Block 05) and the Main Loop (Block 08/09).

*   **Input**:
    *   Consumes `VEML_STEPS` struct array (defined in Block 05).
    *   Consumes `deviceConfig.veml_autorange` (from Block 02/03 Configuration).
*   **Output**:
    *   Exposes `readVEML(uint16_t&, float&)` to the **Main Loop**. This is the only way the rest of the code should access the sensor.
    *   Exposes `VEML_GetCurrentStepName()` for **Logging/MQTT/WebUI** (Block 10) to report which gain/time is currently active.
*   **Dependencies**:
    *   Requires the global `Adafruit_VEML7700 veml` object (from Block 05).
    *   Relies on `millis()` for non-blocking timing.

---


## Analysis: Block 07
# Block 07: Sensor Management

This block handles the physical interaction with the I2C sensors (BME280 and VEML7700). It encapsulates initialization, configuration sequences, raw data retrieval, data validation (sanity checks), and updating the global sensor state structure.

## 1. Responsibilities
*   **I2C Bus Initialization**: Sets up the ESP32 I2C interface at 400kHz.
*   **Sensor Discovery**: detecting BME280 at multiple addresses (0x76/0x77) and retrying VEML7700 connection.
*   **Configuration**: Applying filter settings, oversampling (BME280), and gain/integration time (VEML7700) based on global configuration.
*   **Data Acquisition**: Periodically reading temperature, humidity, pressure, and lux.
*   **Sanity Checking**: filtering out `NaN` or physically impossible values (e.g., pressure > 1200 hPa) before updating global state.

## 2. Key Functions & Variables

| Entity | Type | Description |
| :--- | :--- | :--- |
| `initSensors()` | Function | Initializes I2C, attempts to find BME280 (0x76/0x77) and VEML7700. Configures sensor parameters. Returns `true` if initialization logic completes (even if individual sensors fail). |
| `readSensors()` | Function | Reads available sensors, performs range validation, executes auto-ranging logic (if enabled), and updates the global data struct. |
| `bme` | Object | Instance of `Adafruit_BME280`. |
| `veml` | Object | Instance of `Adafruit_VEML7700`. |
| `currentSensorData` | Global Struct | **Modified**. Flags (`bme280Working`, `vemlWorking`) and values (`temperature`, `lux`, etc.) are written here. |
| `deviceConfig` | Global Struct | **Read-only**. Used to determine if `veml_autorange` should be enabled during init and runtime. |
| `VEML_AutoRange*` | External Funcs | Calls helper functions (likely defined in Block 05/06) to handle the specific complexity of lux gain switching. |

## 3. State & Logic Analysis

### Initialization Flow (`initSensors`)
1.  **I2C Setup**: Starts `Wire` on `I2C_SDA` and `I2C_SCL` with a 400kHz clock.
2.  **BME280 Hunt**:
    *   Attempts address `0x76`. If fail, attempts `0x77`.
    *   If successful, sets `currentSensorData.bme280Working = true` and configures heavy oversampling (x16 for Press/Hum) and IIR filtering to stabilize readings.
3.  **VEML7700 Setup**:
    *   Enters a retry loop (max 3 attempts) to connect to the lux sensor.
    *   If successful, checks `deviceConfig.veml_autorange`:
        *   **If True**: Calls `VEML_AutoRangeInit`.
        *   **If False**: Locks sensor to Fixed Gain (1/8) and Integration Time (100ms).
4.  **Reporting**: Prints a summary to Serial indicating which sensors are online and returns status.

### Runtime Flow (`readSensors`)
1.  **BME280**:
    *   Checks `bme280Working` flag.
    *   Reads T/H/P. Pressure is converted from Pa to hPa.
    *   **Sanity Check**: Values are only written to `currentSensorData` if they are numbers (`!isnan`) and within defined bounds (e.g., Temp -40 to 85°C). This protects the regulation logic from sensor glitches.
2.  **VEML7700**:
    *   Checks `vemlWorking` flag.
    *   Calls external helper `readVEML` to get `raw` and `lux`.
    *   **Auto-Range**: If `deviceConfig.veml_autorange` is true, calls `VEML_AutoRangeUpdate` to dynamically adjust gain based on the raw value.
    *   Updates `currentSensorData.lux` if values are valid.
3.  **Timestamp**: Updates `currentSensorData.lastUpdate` with `millis()`.

## 4. Integration Points

This block acts as the **Producer** for the system's environmental data.

*   **Inputs**:
    *   Relies on **Block 02/03** (Configuration) to provide `deviceConfig.veml_autorange`.
    *   Relies on **Block 05/06** (VEML Helpers) for low-level lux calculation and gain switching (`VEML_AutoRangeUpdate`, `readVEML`).
*   **Outputs**:
    *   Populates **`currentSensorData`**. This structure is the "source of truth" consumed by:
        *   **Display Logic** (Block 08).
        *   **Regulation/Control Loops** (Block 09 - Motor/Relay control based on Temp/Lux).
        *   **Network Reporting** (MQTT/API sending data to cloud).

---


## Analysis: Block 08
Here is the detailed technical analysis of **Block 08 (Gate Mode - HTTP Config Server)**.

### 1. Block Title
**Block 08: Gate Mode (HTTP Config Server)**

### 2. Responsibilities
This block implements the **Provisioning and Configuration** state of the device. Unlike many IoT devices that create their own Hotspot (SoftAP) for configuration, this system operates in a "Centralized Provisioning" architecture where the device acts as a Station (STA) connecting to a specific "Gate" Access Point.

Its specific responsibilities include:
*   **Centralized Connection**: Connecting to a pre-defined Gateway AP (`GATE_AP_SSID`) to receive configuration.
*   **Schema Definition**: Serving a JSON schema that defines the UI/Form fields required by the external configuration application (App or Web Dashboard).
*   **Configuration Ingestion**: Parsing incoming JSON configuration payloads (WiFi credentials, MQTT details, Sensor settings).
*   **Persistence & Reboot**: Validating data, saving it to Non-Volatile Storage (NVS), and triggering a system restart to apply changes.
*   **Visual Feedback**: Driving the LED with a specific pattern to indicate "Gate Mode" is active.

### 3. Key Functions & Variables

| Name | Type | Description |
| :--- | :--- | :--- |
| `setupGateMode()` | Function | Initializer for this mode. Sets WiFi to STA, attempts connection to `GATE_AP_SSID`, and starts the HTTP server. |
| `loopGateMode()` | Function | The main loop execution for this mode. Handles HTTP clients, manages LED blinking (1s interval), and performs WiFi reconnection logic. |
| `handleSchema()` | Function | **HTTP GET /schema**. Generates a JSON document describing the required configuration fields (Types, Labels, Defaults) for the frontend. |
| `handleConfig()` | Function | **HTTP POST /config**. Deserializes JSON, populates the `deviceConfig` global struct, calls `saveConfig()`, and restarts the ESP32. |
| `configServer` | Object | (Global/External) Instance of `WebServer` used to handle HTTP routes. |
| `deviceConfig` | Struct | (Global/External) The main configuration structure where settings are stored. |
| `GATE_AP_SSID` | Constant | The hardcoded SSID of the provisioning gateway the device must connect to. |

### 4. State & Logic Flow

The logic within this block is state-dependent, operating entirely isolated from the standard sensing loops.

**A. Initialization (`setupGateMode`)**
1.  **Network Setup**: Forces the WiFi interface into Station Mode (`WIFI_STA`).
2.  **Connection**: Attempts to connect to the factory/site-specific Gateway AP (`GATE_AP_SSID`).
3.  **Blocking Wait**: It waits up to 20 seconds (40 attempts * 500ms) for a connection, flashing the LED rapidly.
4.  **Server Start**: Regardless of connection success (it retries later), it defines HTTP routes and starts the web server.

**B. The Handshake Protocol (Schema & Config)**
This block implements a dynamic configuration protocol:
1.  **Schema Request**: The external config tool requests `/schema`.
2.  **Schema Generation**: The ESP32 builds a dynamic JSON containing:
    *   **Metadata**: Firmware version, Chip ID.
    *   **Form Definitions**: Field definitions for WiFi (SSID/Pass), MQTT (Server/Port/User/Pass/Topic), and Sensor (VEML Autorange).
    *   *Note*: This allows the frontend to be "dumb"—it renders whatever form the firmware dictates.
3.  **Config Submission**: The tool POSTs JSON to `/config`.
4.  **Parsing**: The firmware validates JSON and copies strings safely (`strlcpy`) into the `deviceConfig` struct.
5.  **Termination**: If `saveConfig()` returns true, the device sends HTTP 200 and performs `ESP.restart()`.

**C. Runtime Loop (`loopGateMode`)**
*   **Heartbeat**: Blinks the LED every 1000ms (1Hz) to differentiate this mode from the "Run Mode" (which likely blinks differently).
*   **Watchdog**: Every 5 seconds, it checks `WiFi.status()`. If disconnected from the Gate AP, it attempts to reconnect automatically.

### 5. Integration Points

This block interacts with the monolithic system as follows:

*   **Global Config (`deviceConfig`)**: This is the primary output. Block 08 populates this struct, which Block 01 (Setup) and Block 04 (MQTT) rely on during normal operation.
*   **NVS Storage (`saveConfig`)**: Relies on Block 02 (Preferences) to physically write the updated `deviceConfig` to flash memory.
*   **Sensor Logic Link**: It explicitly configures `veml.autorange`. This setting directly impacts the logic in **Block 06/07** (Sensor Reading/Normalization).
*   **Hardware Abstraction**: Directly controls `LED_PIN` for status indication.
*   **Dependencies**: Heavy reliance on `ArduinoJson` for serialization/deserialization.

---


## Analysis: Block 09
Here is the detailed technical analysis for **Block 09: Normal Mode (MQTT Operation)**.

### 1. Block Title
**BLOCK 09: NORMAL MODE (MQTT OPERATION)**

### 2. Responsibilities
This block acts as the primary operational state ("Main Loop") of the firmware. It handles the device's lifecycle when configured and deployed. Its specific responsibilities include:
*   **WiFi Connection Management**: Establishing initial connection and handling reconnections with exponential backoff or recovery strategies.
*   **Failover/Recovery Logic**: Periodically scanning for the "Zonio Gate" (Configuration AP) if the primary WiFi is unreachable, allowing remote reconfiguration without physical access.
*   **MQTT Lifecycle**: connecting to the broker, maintaining keep-alive, and managing subscription/publishing flow.
*   **Telemetry Publishing**: Formatting and transmitting sensor data (Weather) and device health statistics (System Status) as JSON payloads.
*   **Safety Watchdog**: Implementing a "1-hour offline" emergency restart to clear potential stack/heap corruption if connectivity is lost for extended periods.

### 3. Key Functions & Variables

| Entity | Type | Description |
| :--- | :--- | :--- |
| **`setupNormalMode()`** | Function | Initializes WiFi in Station (STA) mode, sets MQTT server details, and attempts a brief initial connection sequence before handing off to the loop. |
| **`loopNormalMode()`** | Function | The core repetitive logic. Handles non-blocking timers for WiFi recovery, MQTT retries, sensor reading, and status reporting. |
| **`publishOnlineStatus()`** | Function | Publishes a retained MQTT message with IP, RSSI, and Firmware version to `zonio/status/{chipId}` upon connection. |
| **`publishWeatherData()`** | Function | Aggregates sensor struct data (`currentSensorData`) into JSON and publishes to the user-configured topic. |
| **`publishSystemStatus()`** | Function | Constructs a comprehensive JSON diagnostic report (Heap, Uptime, Flash size, VEML autorange state) sent to `zonio/system/{chipId}`. |
| **`wifiReconnectAttempts`** | Variable | Counter tracking how many times the WiFi recovery loop has run during an outage. |
| **`disconnectStartTime`** | Variable | Timestamp used to track how long the device has been offline (triggers 1h emergency restart). |
| **`mqttReconnectInterval`** | Variable | Dynamic interval for MQTT connection retries, implementing exponential backoff (e.g., 2s, 4s, 8s...). |
| **`GATE_AP_SSID`** | Constant | The hardcoded SSID of the configuration gateway the device scans for when the main WiFi fails. |

### 4. State & Logic Analysis

The logic within `loopNormalMode` is divided into two distinct mutually exclusive states based on `WiFi.status()`:

#### A. Disconnected State (Recovery Logic)
If WiFi is down, the system enters a recovery subroutine rather than blocking:
1.  **Timer Start**: Records `disconnectStartTime` if not already set.
2.  **Emergency Restart**: If `now - disconnectStartTime > 1 hour`, the ESP calls `ESP.restart()`. This prevents "zombie" devices (connected to power but logically hung) from remaining offline indefinitely.
3.  **Gate Scanning (Every 15s)**:
    *   The device performs a `WiFi.scanNetworks()`.
    *   It looks specifically for `GATE_AP_SSID`.
    *   **Logic**: If the Gate AP is found, the user is likely requesting reconfiguration. The device sets `isGateMode = true`, calls `setupGateMode()`, and exits the Normal loop immediately.
    *   **Fallback**: If the Gate is *not* found, it calls `WiFi.begin()` to retry the stored credentials.
4.  **Feedback**: The LED blinks slowly (500ms) to indicate "Searching/Offline".

#### B. Connected State (Operational Logic)
If `WiFi.status() == WL_CONNECTED`:
1.  **Cleanup**: Resets disconnect timers and counters.
2.  **MQTT Management**:
    *   Checks `mqttClient.connected()`.
    *   **If Disconnected**: Uses **Exponential Backoff** (doubling wait time on every fail up to `RECONNECT_MAX`) to attempt reconnection. On success, publishes immediate "Online" and "System" status messages.
    *   **If Connected**: Calls `mqttClient.loop()` to handle incoming packets/pings.
3.  **Sensor & Status Intervals**:
    *   Checks `millis()` against `INT_FAST`. If elapsed, calls `readSensors()` (from Block 07) and `publishWeatherData()`.
    *   Checks `millis()` against `INT_STATUS`. If elapsed, calls `publishSystemStatus()`.
4.  **Heartbeat**: A very fast (5ms) LED blip every 100ms indicates the CPU is running and WiFi is connected.

### 5. Integration

This block serves as the **Integration Hub** for the entire firmware:

*   **Upstream Dependencies**:
    *   **Block 02 (Config)**: Consumes `deviceConfig` for WiFi credentials (`wifi_ssid`, `wifi_pass`) and MQTT settings.
    *   **Block 07 (Sensors)**: Calls `readSensors()` and consumes `currentSensorData` struct for payloads.
    *   **Block 03 (Global Objects)**: Uses `mqttClient`, `WiFi`, `chipId`.
*   **Downstream / Transitions**:
    *   **Block 10 (Gate Mode)**: If the "Gate Scan" logic finds the specific SSID, it transitions control to `setupGateMode()`.
*   **Hardware Control**:
    *   Directly manipulates `LED_PIN` for complex status indication patterns (Fast blink, Slow blink, Heartbeat blip).
    *   Monitors `WiFi.RSSI()` for signal strength reporting.

**Technical Note on JSON Construction**:
The block uses `snprintf` extensively with C-style strings (`char msg[512]`) rather than Arduino `String` concatenation for the main payloads. This is a best practice on microcontrollers to prevent heap fragmentation, which is critical for long-term stability in automation environments.

---


## Analysis: Block 10
Here is the detailed technical analysis of **Block 10: Setup & Main Loop**.

### 1. Block Title
**Block 10: Setup & Main Loop**

### 2. Responsibilities
This block serves as the **Firmware Entry Point and Executive Supervisor**. Its primary responsibilities are:
*   **Hardware Initialization**: Configuring GPIOs, Serial communication, and basic sensor interfaces immediately upon boot.
*   **Boot Logic Orchestration**: Determining whether the device should boot into **Gate Mode** (Configuration AP) or **Normal Mode** (WiFi Client/Automation) based on physical inputs (Pin D6) and file system integrity.
*   **Main Execution Cycle**: Providing the infinite loop that tracks system uptime, delegates logic to the active mode's sub-loops, and maintains system stability via the Watchdog Timer.

### 3. Key Functions & Variables

| Name | Type | Description |
| :--- | :--- | :--- |
| `setup()` | Function | Standard Arduino entry point. Handles one-time hardware init and decides the operational mode. |
| `loop()` | Function | Standard Arduino main loop. Handles uptime counting, WDT feeding, and task delegation. |
| `isGateMode` | Global Bool | **Critical Flag**. True = Config Mode (AP), False = Normal Mode (Station). Set in `setup()`. |
| `PIN_FIND_GATE` | Constant | The GPIO pin (D6) checked at boot. If LOW, forces Gate Mode. |
| `chipId` | Global Char[] | Buffer storing the unique Hex ID of the ESP unit. |
| `uptimeSeconds` | Global U-Long | Counters for system runtime tracking. |
| `initSensors()` | External Func | Calls the sensor initialization logic (likely from Block 3 or 4). |
| `loadConfig()` | External Func | Attempts to load JSON config from flash (likely from Block 2). |
| `setupGateMode()` | External Func | Initializes WiFi AP and Web Server (likely from Block 8/9). |
| `setupNormalMode()` | External Func | Initializes WiFi Station and MQTT/Automation logic (likely from Block 5/6). |

### 4. State & Logic Analysis

#### A. Boot Sequence (`setup()`)
The boot process follows a strict "Fail-Safe" decision tree to ensure the device is never unreachable.

1.  **Hardware Prep**:
    *   Serial starts at 115,200 baud.
    *   LED is turned ON (LOW signal) to indicate power/boot process.
    *   `PIN_FIND_GATE` is set to `INPUT_PULLUP`.
    *   **Sensors are initialized** regardless of mode. This ensures hardware health checks occur before network logic.

2.  **Mode Decision Tree**:
    *   **Check 1: Physical Override**: The code reads `PIN_FIND_GATE`.
        *   If `LOW` (Grounded): The user is physically requesting configuration. **Action**: Set `isGateMode = true`, run `setupGateMode()`.
    *   **Check 2: Configuration Integrity**: The code attempts `loadConfig()`.
        *   If `false` (File missing or JSON invalid): The device cannot operate normally. **Action**: Set `isGateMode = true`, run `setupGateMode()`.
    *   **Check 3: Normal Operation**:
        *   If Pin is HIGH and Config is Valid. **Action**: Set `isGateMode = false`, run `setupNormalMode()`.

#### B. Runtime Loop (`loop()`)
The main loop acts as a non-blocking wrapper.

1.  **Timekeeping**: It compares `millis()` to `lastUptimeUpdate`. Every 1000ms, it increments `uptimeSeconds`. This provides a drift-free uptime counter (independent of WiFi connection status).
2.  **Mode Dispatch**:
    *   If `isGateMode` is true, it executes `loopGateMode()`.
    *   Otherwise, it executes `loopNormalMode()`.
3.  **System Health**:
    *   `ESP.wdtFeed()`: Resets the hardware watchdog timer to prevent resets during long operations.
    *   `yield()`: Passes control briefly to the underlying OS (essential for WiFi stack maintenance on ESP chips).

### 5. Integration
As the final block, this section integrates the entire monolithic structure:
*   **Upstream Dependencies**: It consumes services defined in almost all previous blocks:
    *   **Block 2 (Config)**: Used for `loadConfig`.
    *   **Block 3/4 (Sensors)**: Used for `initSensors`.
    *   **Block 8/9 (Gate Mode)**: Used for AP and Web Server logic.
    *   **Block 5/6 (Normal Mode)**: Used for MQTT and Regulation logic.
*   **Global State Control**: It is the **sole authority** on the variable `isGateMode`. No other block sets this initial state; they only react to it.
*   **Hardware Abstraction**: It ties the physical user interface (Reset/Config button on Pin D6) directly to the software logic flow.

---
