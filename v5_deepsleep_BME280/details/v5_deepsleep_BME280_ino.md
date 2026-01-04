# Deep Technical Documentation: v3_refactored.ino

> **Monolith Analysis**: Decomposed into 14 blocks.



## Analysis: Block 01
Here is the detailed technical analysis for **Block 01** of `v3_refactored.ino`.

### 1. Block Title
**Block 01: Header, Configuration, and Persistence Structures**

### 2. Responsibilities
This block serves as the **Definition and Configuration Layer** of the firmware. It does not contain executable logic (functions) but establishes the system architecture. Its specific responsibilities are:
*   **Version Control**: Defines firmware identity, versioning, and change logs.
*   **System Configuration**: Sets critical parameters for timing, network timeouts, debug levels, and sensor thresholds.
*   **Hardware Abstraction**: Imports necessary libraries (`ESP8266WiFi`, `BME280`, `Wire`) and maps physical pins.
*   **Memory Architecture Definition**: Defines the data structures for **RTC Memory** (Deep Sleep persistence) and **EEPROM** (Power-cycle persistence/Data Buffering).
*   **Global Instantiation**: Creates global object instances (`bme`, `wifiClient`) used by subsequent blocks.

**Note on Hardware Context**: While the user context mentions **ESP32**, the source code explicitly imports `<ESP8266WiFi.h>` and uses `<user_interface.h>`. This code is written for the **ESP8266** architecture.

### 3. Key Functions/Variables

| Name | Type | Description |
| :--- | :--- | :--- |
| `RTCData` | `struct` | **Critical**: Defines the state machine memory layout used during Deep Sleep. Includes counters (`sampleCount`), baseline sensor values for change detection, and buffer tracking. |
| `EEPROMData` | `struct` | Defines the non-volatile storage layout. Acts as a ring buffer or batch storage to hold up to 32 samples before WiFi upload. |
| `SAMPLE_INTERVAL_SEC` | `Define` (300) | Sets the heartbeat of the system to 5 minutes. |
| `PUBLISH_EVERY_N_SAMPLES`| `Define` (12) | Defines the batch size. 12 samples * 5 mins = 1 hour uploads. |
| `TRIG_TEMP_C` | `Define` (1.5) | Delta threshold. If Temp changes > 1.5°C from baseline, the system wakes/publishes immediately. |
| `COMMIT_EVERY_N_SAMPLES` | `Define` (3) | Writes to EEPROM flash every 3rd sample to reduce flash wear (Write amplification mitigation). |
| `WIFI_...` / `MQTT_...` | `const char*` | Hardcoded network credentials and endpoint configuration. |
| `bme` | `Adafruit_BME280` | Global instance of the sensor driver. |

### 4. State/Logic Flow
Although this block contains no functions, it dictates the **Data Flow Strategy** for the entire firmware:

1.  **The Persistence Strategy**:
    *   The code utilizes a hybrid memory approach.
    *   **RTC Memory (`RTCData`)**: This struct is designed to stay alive while the CPU is in Deep Sleep. It tracks the `baselineTemp` (last uploaded value) to compare against the *next* reading to decide if a wake-up is needed.
    *   **EEPROM (`EEPROMData`)**: This struct is used for bulk storage. The definition `float samples[32][3]` implies a batching mechanism where data is collected offline and uploaded in chunks.

2.  **The Trigger Logic Strategy**:
    *   The defines (`TRIG_TEMP_C`, `TRIG_HUM_PCT`) suggest a **"Reporting on Exception"** logic. The device attempts to sleep for 5 minutes. If the new reading deviates from the `baseline` stored in `RTCData`, it forces an upload; otherwise, it buffers the data and returns to sleep.

3.  **The "Fix" Logic (from Header)**:
    *   The comment `// Fix: Triggers use baseline+delta time instead of fresh NTP` implies that in later blocks, the system will calculate timestamps based on the `baselineEpoch` stored in `RTCData` rather than connecting to WiFi to get NTP time every single time it wakes up, saving significant battery.

### 5. Integration
This block exposes the following to all other blocks (2 through 7):

*   **Global Configuration**: All `#define` macros (Timing, Thresholds, Pins).
*   **Memory Schemas**: The `RTCData` and `EEPROMData` typedefs are required by the Setup block (to read memory) and the Main Loop (to write memory).
*   **Hardware Objects**: The `bme` and `wifiClient` objects are initialized here but configured/connected in later blocks.
*   **Credentials**: Defines `MQTT_HOST`, `WIFI_SSID`, etc., which will be used in the Network/Connection blocks.

**Potential Issues for Refactoring**:
*   Credentials are hardcoded.
*   The code targets ESP8266 libraries while the context is ESP32; this will cause compile errors if compiled for an ESP32 board without porting `ESP8266WiFi.h` to `WiFi.h`.

---


## Analysis: Block 02
# Block 02: Core Globals, Logging, and Persistence Layer

This block establishes the foundational infrastructure for the firmware. It instantiates critical global objects for communication, defines debugging macros, and implements a robust data persistence layer (RTC and EEPROM) with integrity checking.

## 1. Responsibilities
*   **Global Object Instantiation**: Creates instances for MQTT client, NTP time synchronization, and local data containers (`rtcData`, `eepromData`).
*   **Logging System**: Defines macros (`LOGE`, `LOGI`, `LOGV`) to handle serial output based on a pre-defined `DEBUG_LEVEL`.
*   **Data Integrity**: Implements a standard CRC32 algorithm to verify data before saving to or loading from memory.
*   **RTC Memory Management**: Handles saving/loading system state to RTC memory (preserved during Deep Sleep).
*   **EEPROM Management**: Handles saving/loading configuration and backup logs to flash memory (preserved during power loss).

## 2. Key Functions & Variables

### Global Objects
| Variable | Type | Description |
| :--- | :--- | :--- |
| `mqtt` | `PubSubClient` | Handles MQTT communication (initialized with `wifiClient` from Block 01). |
| `ntp` | `NTPClient` | Handles time synchronization with `pool.ntp.org`. |
| `rtcData` | `RTCData` | Instance of the structure holding state preserved across Deep Sleep. |
| `eepromData` | `EEPROMData` | Instance of the structure holding offline logs/config in Flash. |

### Helper Functions
| Function | Description |
| :--- | :--- |
| `calculateCRC32` | Computes a checksum for a byte array using the polynomial `0x04c11db7`. Used to detect data corruption. |
| `saveRTCData` | Serializes `rtcData`, calculates CRC, and writes to RTC memory. |
| `loadRTCData` | Reads from RTC memory, validates Magic Number and CRC. Resets data if invalid. |
| `saveEEPROMData` | Serializes `eepromData`, calculates CRC, and commits to Flash memory (EEPROM). |
| `loadEEPROMData` | Reads from EEPROM, validates Magic, Version, and CRC. Resets data if invalid. |

## 3. State & Logic Analysis

### Logging Logic
The macros use a `do...while(0)` construct to ensure safe execution within `if/else` blocks. They rely on `DEBUG_LEVEL` (presumably defined in Block 01 or a header):
*   **Level 1**: Errors (`LOGE`)
*   **Level 2**: Info (`LOGI`)
*   **Level 3**: Verbose (`LOGV`)

### Persistence Strategy (The "Temp Buffer" Logic)
Both `saveRTCData` and `saveEEPROMData` use a specific technique to avoid checksum errors caused by compiler structure padding:
1.  **Zeroing**: A temporary buffer `tmp` is created and filled with zeros (`memset`).
2.  **Copying**: The actual data struct is copied into `tmp` via `memcpy`.
3.  **CRC Calculation**: The CRC is calculated based on `tmp`.
4.  **Why?**: If the compiler adds padding bytes between struct members for alignment, those bytes usually contain random garbage memory. Calculating a CRC on the raw struct would result in random checksums. Zeroing the buffer first ensures deterministic behavior.

### Validation Logic
When loading data (`loadRTCData` or `loadEEPROMData`), three checks must pass:
1.  **Magic Number**: Confirms the data was written by this specific firmware (`RTC_MAGIC` / `EEPROM_MAGIC`).
2.  **Version/Sanity**: Checks version (EEPROM) or logic bounds (e.g., `eepromData.count <= 32`).
3.  **CRC32**: Recalculates the checksum of the loaded data and compares it against the stored checksum.

## 4. Integration Variables
This block exposes several critical components to the subsequent blocks:

*   **`rtcData`**: Will be used in **Block 03 (Setup)** to determine if the device is waking from deep sleep or a fresh boot, and in **Block 05/06 (Loop)** to increment counters (`wakeCount`, `sampleCount`).
*   **`eepromData`**: Will be used in **Block 06** to buffer sensor readings if WiFi is unavailable.
*   **`mqtt` & `ntp`**: Will be configured and connected in **Block 03** and **Block 04**.
*   **Logging Macros**: Will be used throughout the entire firmware for debugging.

---


## Analysis: Block 03
# Block 03 Analysis: Sensor Abstraction & Network Connectivity

## 1. Block Title
**Block 03: Hardware Drivers (BME280) and Network Layer (WiFi/MQTT)**

## 2. Responsibilities
This block implements the **Hardware Abstraction Layer (HAL)** for environmental sensing and the **Communication Infrastructure**. It acts as the bridge between the physical world (I2C sensors) and the digital upstream (Cloud/MQTT Broker).

*   **Environmental Sensing**: Initializes and drives the BME280 sensor (Temperature, Humidity, Pressure) with specific power-saving configurations.
*   **Data Validation**: Implements "sanity checks" on sensor data to filter out hardware glitches or broken wires before processing.
*   **WiFi Management**: Manages the connection to the Access Point, including transmission power settings and retry logic.
*   **MQTT Protocol Handler**: Manages the connection to the MQTT broker, including authentication, client ID generation, and buffer configuration for payload transmission.

## 3. Key Functions & Variables

| Function / Variable | Type | Description |
| :--- | :--- | :--- |
| **`initBME280()`** | `bool` | Initializes I2C on `BME_SDA`/`BME_SCL`. Scans addresses `0x76` and `0x77`. Sets sensor to **Forced Mode** (sleep until requested). |
| **`readSensorData(...)`** | `bool` | Triggers a measurement, reads raw values, converts Pressure to hPa. Validates ranges (e.g., Temp -40 to 85°C). Updates variables via reference. |
| **`connectWiFi()`** | `bool` | Sets STA mode, disables flash persistence (preserves EEPROM life), sets TX power. Implements a blocking retry loop with timeouts. |
| **`connectMQTT()`** | `bool` | Configures the `PubSubClient`. Sets a 512-byte buffer (critical for large JSON batches). Handles Auth and Dynamic Client IDs. |
| `bme` | Global Obj | Instance of `Adafruit_BME280` (assumed declared in Block 01/02). |
| `mqtt` | Global Obj | Instance of `PubSubClient` (assumed declared in Block 01/02). |
| `WIFI_TX_POWER_DBM` | Constant | Control over radio transmission power (likely for power saving or range tuning). |
| `MAX_WIFI_ATTEMPTS` | Constant | Safety limit to prevent infinite loops during network outages. |

## 4. State & Logic Flow

### Sensor Logic (BME280)
1.  **Initialization**: Attempts to find the sensor on the I2C bus. If found, it configures the sensor to **MODE_FORCED**.
    *   *Why Forced Mode?* The ESP32 triggers a read explicitly. Between reads, the sensor sleeps. This prevents self-heating of the sensor (which would skew temperature readings) and saves power.
2.  **Reading**:
    *   Wake up -> `takeForcedMeasurement()` -> Read ADC.
    *   **Validation**: The code checks `isnan()` and specific physical bounds (e.g., Humidity must be 0-100%). If values are out of bounds, it logs an error and returns `false`, preventing bad data from entering the regulation logic in later blocks.

### Network Logic
1.  **WiFi Connection**:
    *   Clears previous credentials logic (`persistent(false)`).
    *   Enters a `while` loop monitoring connection status.
    *   **Timeout Logic**: If connection takes too long, it disconnects and explicitly calls `begin()` again. This handles cases where the ESP32 radio stack gets stuck.
2.  **MQTT Connection**:
    *   Configures a large buffer (`512` bytes). This suggests the system sends **batched data** or long JSON strings, not just single values.
    *   Generates a unique `clientId` using `deviceId + millis()` to avoid broker collisions if the device restarts rapidly.
    *   Supports both Anonymous and Authenticated (User/Pass) modes based on configuration length.

## 5. Integration

This block provides the essential **inputs** and **transport** for the rest of the firmware:

*   **Exposed to Block 04 (Main Loop/Logic)**:
    *   `readSensorData` is called within the main cycle to populate the system's current state variables (`currentTemp`, `currentHum`, etc.).
    *   `connectWiFi` and `connectMQTT` are called during `setup()` and likely within a "Watchdog" or "Reconnection" routine in the main loop if connectivity drops.
*   **Dependencies**:
    *   Requires definitions from Block 01/02: `BME_SDA`, `BME_SCL`, `WIFI_SSID`, `MQTT_HOST`, `deviceId`.
    *   Requires Global Objects: `Wire`, `WiFi`, `bme`, `mqtt`.

**Note on Refactoring**: This block is highly modular. The sensor logic is decoupled from the network logic, making it easy to swap the BME280 for a different sensor (e.g., DHT22 or SHT30) without breaking the networking code.

---


## Analysis: Block 04
# Block 04: Time Sync, Network Teardown, and Adaptive Sampling Logic

This block implements the critical decision-making logic for the **Adaptive Reporting** strategy. It handles time synchronization, proper network shutdown sequences for power management, local data buffering, and the algorithmic determination of whether a sensor reading deviates enough from the baseline to warrant an immediate alert (trigger).

## 1. Responsibilities
*   **Time Synchronization**: Fetches accurate Epoch time via NTP with retry logic.
*   **Power Management (Network)**: executes a clean disconnection sequence for MQTT and WiFi to ensure low current consumption during Deep Sleep.
*   **Local Buffering**: Stores sensor readings into a structured buffer (`eepromData`) intended for batch transmission.
*   **Adaptive Triggering**: Compares current readings against a "Baseline" (stored in RTC memory) to calculate deltas.
*   **Trend Analysis**: Optionally computes the rate of change (slope) for temperature to detect rapid environmental shifts.

## 2. Key Functions & Variables

| Function / Variable | Type | Description |
| :--- | :--- | :--- |
| **`getNTPTime()`** | `uint32_t` | Initiates NTP client, retries 5 times (500ms delay), returns Epoch or 0 on failure. |
| **`disconnectNetwork()`** | `void` | Cleanly closes MQTT session, disconnects WiFi, and sets mode to `WIFI_OFF`. Critical for maximizing Deep Sleep battery life. |
| **`storeSample(...)`** | `void` | Pushes T/H/P + Timestamp into the `eepromData` array. Syncs count to `rtcData`. Auto-saves to EEPROM if buffer is near full (31/32). |
| **`updateBaseline(...)`** | `void` | Overwrites the reference values in `rtcData` (Temp, Hum, Pres, Time). Used to reset the delta check. |
| **`computeSlopeTemp()`** | `float` | Performs a linear regression (Least Squares) on the last $N$ samples to determine temperature trend in °C/h. |
| **`isTriggered(...)`** | `bool` | The core decision logic. Returns `true` if current readings exceed defined thresholds relative to the baseline or if the slope is steep. |
| `ntp` | Object | Global NTP client instance (external dependency). |
| `eepromData` | Struct | Global buffer for batch storage (likely stored in RTC or mirrored to EEPROM). |
| `rtcData` | Struct | Global state preserved across Deep Sleep (holds baseline values). |

## 3. State & Logic Analysis

### Time Synchronization
The `getNTPTime()` function is blocking but short-lived (max ~2.5 seconds). It is essential for timestamping samples before they are stored in the buffer. If NTP fails, it returns 0, implying the system might need to rely on relative time estimation or RTC extrapolation in other blocks.

### Network Teardown
`disconnectNetwork()` explicitly addresses ESP32 quirks regarding Deep Sleep:
1.  **MQTT Disconnect**: Gracefully closes the socket to prevent broker-side "Keep Alive" timeouts or Last Will firing unexpectedly later.
2.  **WiFi Disconnect**: `WiFi.disconnect(true)` cleans WiFi credentials from RAM (depending on SDK version) and shuts down the radio.
3.  **Mode OFF**: `WiFi.mode(WIFI_OFF)` is explicitly called. The comment notes *not* to use `forceSleepBegin()`, suggesting a specific ESP32 Arduino Core version requirement where `WIFI_OFF` is the preferred method before `esp_deep_sleep_start`.

### Data Buffering (`storeSample`)
*   **Capacity Check**: Checks `eepromData.count < 32`.
*   **Storage**: Saves float data (Temp, Hum, Pres) and Timestamp.
*   **RTC Sync**: Updates `rtcData.samplesInBuffer`. This suggests `rtcData` resides in `RTC_SLOW_MEM` (survives sleep), while `eepromData` might be in standard RAM that requires flushing to actual EEPROM flash memory before power loss.
*   **Failsafe**: If count hits 31, it forces `saveEEPROMData()`. This prevents buffer overflow if the device fails to upload data for an extended period.

### Adaptive Triggering Logic (`isTriggered`)
This is the "Smart" part of the firmware. Instead of sending every sample, it evaluates importance:
1.  **Validity Check**: If no baseline exists (`!isBaselineValid`), it triggers immediately (to establish a baseline).
2.  **Delta Calculation**: Calculates absolute difference (`fabs`) between `current` and `baseline` for T, H, and P.
3.  **Trend Calculation**: Calls `computeSlopeTemp()`.
    *   Uses a macro `ENABLE_TREND_TRIGGER`.
    *   Selects up to the last 6 samples.
    *   Calculates linear regression slope.
    *   Scales result to **°C per Hour**.
4.  **Decision**: The function returns `true` if **ANY** condition is met:
    *   $\Delta T \ge$ `TRIG_TEMP_C`
    *   $\Delta H \ge$ `TRIG_HUM_PCT`
    *   $\Delta P \ge$ `TRIG_PRESS_HPA`
    *   $|Slope| \ge$ `TREND_TEMP_C_PER_H`

## 4. Integration with Other Blocks

*   **Preceded by**: Sensor Reading Block (Block 03). The inputs `temp`, `hum`, `pres` passed to `storeSample` and `isTriggered` come from there.
*   **Succeeded by**: Data Upload Block (Block 05). If `isTriggered` returns `true`, Block 05 will likely initiate the WiFi/MQTT connection sequence. If `false`, the system will likely go straight to sleep (Block 06/07).
*   **Global State Dependency**: Heavily relies on `rtcData` (NoInit RAM/RTC memory) to remember the "State of the World" (Baseline) between Deep Sleep cycles.
*   **EEPROM Dependency**: Relies on `saveEEPROMData()` (likely defined in Block 02) to persist the buffer when full.

---


## Analysis: Block 05
Here is the detailed technical analysis of **Block 05**.

### 1. Block Title
**Block 05: Data Publishing, Deep Sleep, and Wake Diagnostics**

### 2. Responsibilities
This block manages the end-of-cycle operations for the firmware. Its primary responsibilities are:
*   **Data Formatting**: Converting raw sensor values and timestamps into transmission-ready strings (JSON or CSV).
*   **MQTT Batch Publishing**: Aggregating multiple buffered samples into payloads that fit within network limits (`PAYLOAD_SAFE_LIMIT`), publishing them to the broker, and handling retries/failures.
*   **Buffer Management**: Clearing the EEPROM/RAM sample buffer upon successful publication to prevent data duplication.
*   **Power Management (Deep Sleep)**: Preparing the hardware for low-power mode, including saving state context (RTC) and triggering the specific hardware sleep command.
*   **Wake-up Diagnostics**: determining *why* the device restarted (e.g., Timer Wake vs. Watchdog Reset) and updating health metrics accordingly.

### 3. Key Functions & Variables

| Function / Variable | Type | Description |
| :--- | :--- | :--- |
| `formatPayload` | `String` | Helper function. Formats a single data row based on `USE_CSV_PAYLOAD` flag (returns CSV or JSON). |
| `publishData` | `bool` | **Core Function**. Iterates through `eepromData.samples`, creates batches, publishes to MQTT, and clears the buffer. |
| `enterDeepSleep` | `void` | Prepares system for sleep: saves RTC, disables LED/Network, and calls hardware sleep API (sets timer). |
| `checkWakeReason` | `bool` | Inspects hardware reset info. Returns `true` if wake was caused by Deep Sleep timer, `false` otherwise (e.g., power loss/crash). |
| `eepromData` | `struct` | Global struct (from Block 04). Contains `count`, `samples[][]`, and `timestamps[]`. |
| `rtcData` | `struct` | Global struct (from Block 04). Stores volatile state across sleep (`wakeCount`, `errorCount`). |
| `PAYLOAD_SAFE_LIMIT` | `const` | (External) Maximum byte size for a single MQTT packet to prevent buffer overflows. |
| `WAKE_RF_DISABLED` | `const` | Flag used during sleep to ensure WiFi radio remains off immediately upon wake-up until explicitly enabled. |

### 4. State & Logic Analysis

**A. Publishing Logic (`publishData`)**
1.  **Empty Check**: If `eepromData.count == 0`, it returns `true` immediately (nothing to do).
2.  **Batching Loop**:
    *   Iterates through stored samples `0` to `count`.
    *   Formats the current sample.
    *   Checks if adding the current row to `batch` exceeds `PAYLOAD_SAFE_LIMIT`.
    *   **If Limit Exceeded**: Publishes current `batch`, clears `batch`, waits (5ms), and starts a new batch with the current row.
    *   **Delimiter**: Adds a newline `\n` between rows within the batch.
3.  **Final Flush**: If data remains in `batch` after the loop, it performs a final MQTT publish.
4.  **Cleanup**:
    *   Resets `eepromData.count` to 0.
    *   Resets `rtcData.samplesInBuffer` to 0.
    *   Calls `saveEEPROMData()` to persist the "empty" state (preventing re-sending old data if the device crashes before the next sample).

**B. Architecture Mismatch Note (ESP8266 vs ESP32)**
While the user context states **ESP32**, the code in this block uses **ESP8266 specific APIs**:
*   `ESP.deepSleep(..., WAKE_RF_DISABLED)` is ESP8266 syntax (ESP32 uses `esp_deep_sleep_start`).
*   `rst_info *resetInfo = ESP.getResetInfoPtr()` and `REASON_DEEP_SLEEP_AWAKE` are ESP8266 SDK structures.
*   **Logic Implication**: If this code is compiled on an ESP32, it will fail. If running on ESP8266, the logic limits sleep to ~71 minutes (`seconds > 4200`) to avoid the 32-bit hardware timer overflow inherent to the ESP8266.

**C. Wake Diagnostics (`checkWakeReason`)**
1.  Retrieves the hardware reset reason.
2.  **Success Path**: If `REASON_DEEP_SLEEP_AWAKE` is detected, it increments `rtcData.wakeCount` and resets error counters.
3.  **Failure Path**: Any other reason (Watchdog, Power-on, Exception) increments `rtcData.errorCount`.
4.  **Hardware Warning**: If `errorCount >= 3`, it logs a warning about the physical connection (D0 to RST), which is required for ESP8266 deep sleep wake-up.

### 5. Integration Points

*   **Block 04 (Storage)**: This block heavily relies on `eepromData` (to read samples for publishing) and `rtcData` (to save state before sleep).
*   **Block 02 (Network)**: Relies on the `mqtt` object to perform the actual network transmission and `disconnectNetwork()` to cleanly close connections.
*   **Block 01 (Config)**: Uses configuration defines like `TOPIC_PREFIX`, `USE_CSV_PAYLOAD`, and `LED_PIN`.
*   **Hardware Abstraction**: Directly interacts with the MCU reset controller and deep sleep timer.

**Transition**: After this block executes (specifically `enterDeepSleep`), the standard code execution flow **stops**. The device halts until the hardware timer triggers a reset, restarting execution at Block 01 (Setup).

---


## Analysis: Block 06
Here is the detailed analysis of **Block 06**.

### 1. Block Title
**Block 06: Main Initialization & Measurement Cycle Logic**

### 2. Responsibilities
This block acts as the **Orchestrator** of the firmware. It contains the standard Arduino `setup()` and the custom `runMeasurementCycle()` function which replaces the traditional `loop()`. Its primary responsibilities are:
*   **System Initialization**: Setting up Serial, LEDs, and identifying the specific device ID.
*   **State Restoration**: loading context from RTC memory (volatile but survives deep sleep) and EEPROM (flash storage).
*   **Sensor Acquisition**: Orchestrating the BME280 reading and handling sensor hardware failures.
*   **Decision Engine**: Determining whether to stay offline (store data locally) or go online (WiFi/MQTT) based on timers (`PUBLISH_EVERY_N_SAMPLES`) or sensor triggers (`isTriggered`).
*   **Time Management**: Implementing the "Minimal Trigger Fix" to calculate accurate timestamps using relative deltas when waking on triggers, ensuring graph continuity without needing constant NTP synchronization.
*   **Power Management**: Executing the transition to Deep Sleep.

### 3. Key Functions & Variables

| Function / Variable | Type | Description |
| :--- | :--- | :--- |
| `setup()` | `void` | Standard entry point. Handles hardware init, data loading (`loadRTCData`), and calls the measurement cycle. |
| `runMeasurementCycle()` | `void` | The core application logic. Measures, decides logic path (Online vs Offline), saves data, and triggers sleep. |
| `isDeepSleepWake` | `bool` | Flag derived from `checkWakeReason()` to determine if WiFi radio needs explicit waking. |
| `rtcData.sampleCount` | `uint32_t` | Global counter of measurement cycles, used for timing calculations. |
| `triggered` | `bool` | Result of `isTriggered()`; true if sensor values exceed defined thresholds (from Block 05). |
| `timeToPublish` | `bool` | True if the current sample count modulo `PUBLISH_EVERY_N_SAMPLES` is zero. |
| `needToPublish` | `bool` | Aggregated flag (`triggered` OR `timeToPublish` OR buffer full) initiating WiFi connection. |
| `currentTime` | `uint32_t` | The calculated Unix timestamp for the current sample. Logic varies based on wake reason (NTP vs. Calculated Delta). |
| `updateBaseline(...)` | `void` | Updates the reference time (`baselineEpoch`) in RTC, crucial for the delta-time calculation logic. |

### 4. State & Logic Flow

The flow within this block is linear and designed to execute once per wake-up cycle before returning to sleep.

#### A. Initialization (`setup`)
1.  **Hardware**: Serial init, LED ON.
2.  **Identity**: Generates `deviceId` (e.g., "ws8266-1234AB").
3.  **State Recovery**: Calls `loadRTCData()` and `loadEEPROMData()`. If RTC is lost (power cycle), flags are reset.
4.  **Sensor Init**: Initializes BME280.
    *   *Failure Mode*: If sensor fails, logs error, blinks LED, and enters a **short deep sleep (30s)** to retry, skipping the rest of the logic.

#### B. Measurement Loop (`runMeasurementCycle`)
1.  **Acquisition**: Reads Temperature, Humidity, Pressure.
    *   *Sensor Error Check*: If reading is invalid, increments `rtcData.errorCount`. If errors > 5, performs a full system restart (`ESP.restart()`). Otherwise, sleeps for 60s.
2.  **Logic Decision**:
    *   Checks if values violate thresholds (`isTriggered`).
    *   Checks if it is time to report based on sample count.
    *   Checks if EEPROM buffer is nearly full.
3.  **Path 1: Online (Publishing)**
    *   Connects to WiFi.
    *   **Time Calculation (The Fix)**:
        *   **Trigger Event**: Uses `baselineEpoch + (delta_samples * interval)` to calculate time. This avoids fetching a fresh NTP time which might cause "time jumps" or gaps in visualization graphs due to slight clock drifts.
        *   **Periodic Event**: Fetches fresh `getNTPTime()`.
    *   Stores sample in internal buffer.
    *   Connects to MQTT and calls `publishData()`.
    *   **Post-Publish Logic**:
        *   If successful (Periodic): Updates `baselineEpoch` with fresh NTP time.
        *   If successful (Trigger): Does **not** update baseline (preserves the time-grid alignment).
        *   Visual Feedback: 3 rapid LED blinks.
4.  **Path 2: Offline (Buffering)**
    *   Calculates timestamp using the Delta method (Relative to last known valid online time).
    *   Stores sample in buffer.
    *   **Persistence**: Writes to EEPROM if `COMMIT_EVERY_N_SAMPLES` is reached (prevents data loss if power fails before next upload).
5.  **Shutdown**:
    *   Turns LED OFF.
    *   Calls `enterDeepSleep(SAMPLE_INTERVAL_SEC)`.

### 5. Integration Notes

This block serves as the **Logic Layer** binding the Monolith together.

*   **Dependencies**:
    *   **Block 02 (Config)**: Needs `LED_PIN`, `SAMPLE_INTERVAL_SEC`, `FW_VERSION`.
    *   **Block 03 (Storage)**: Relies heavily on `rtcData` struct, `storeSample`, `loadRTCData`, `saveEEPROMData`.
    *   **Block 04 (Network)**: Calls `connectWiFi`, `connectMQTT`, `getNTPTime`, `publishData`.
    *   **Block 05 (Sensors)**: Calls `initBME280`, `readSensorData`, `isTriggered`.
*   **Exposed Behavior**:
    *   It does not expose variables directly but defines the runtime behavior of the device.
    *   It serves as the definitive source of truth for `rtcData.sampleCount` incrementation.

*   **Specific Code Observation**:
    *   The code uses `ESP.getChipId()` and `WiFi.forceSleepWake()`. These are strictly **ESP8266** API calls. While the context prompt mentions "ESP32", this specific code block is written for the ESP8266 architecture. If compiled on ESP32, these lines would cause compilation errors without an abstraction layer or `#ifdef` guards.

---


## Analysis: Block 07
Here is the detailed technical analysis for **Block 07**.

### 1. Block Title
**Block 07: Main Loop (Error Handling & Fail-Safe)**

### 2. Responsibilities
This block implements the standard Arduino `loop()` function. However, in this specific **Deep Sleep architecture**, the standard loop represents a **critical failure state**.

Its primary responsibilities are:
1.  **Error Trapping**: Catch the program flow if the `setup()` function finishes without successfully entering Deep Sleep.
2.  **Diagnostic Logging**: Output specific error messages indicating that the Deep Sleep mechanism failed (likely a hardware configuration issue).
3.  **Visual Alarm**: Rapidly blink the system LED to visually signal a hardware/logic fatal error to the user.
4.  **System Recovery**: Force a software restart (`ESP.restart()`) to attempt to reset the system state and try again.

### 3. Key Functions & Variables

| Element | Type | Description |
| :--- | :--- | :--- |
| `loop()` | **Core Function** | The infinite loop entry point. In this firmware, reaching this function is considered a bug or hardware failure. |
| `LOGE(...)` | **Macro** | Logging mechanism (defined in previous blocks) used here to print "ERROR" level messages to the Serial console. |
| `LED_PIN` | **Constant** | GPIO pin number for the onboard status LED. |
| `digitalWrite()` | **Function** | Used to toggle the `LED_PIN` to create a blinking pattern. |
| `ESP.restart()` | **System Function** | Triggers a software reset of the ESP32, re-running the bootloader and `setup()`. |

### 4. State & Logic Analysis
This block operates on a simple linear fail-safe logic:

1.  **Entry Condition**: The code enters `loop()` only if the logic in **Block 06** (or previous blocks) failed to call `esp_deep_sleep_start()` or if the sleep function failed to halt execution.
2.  **Logging**: It immediately logs `ERROR: Reached loop() - deep sleep failed!`.
    *   *Hardware Hint*: It specifically suggests checking the "D0 -> RST connection".
    *   *Note*: While common on ESP8266, ESP32s usually handle deep sleep internally via the RTC controller without external wiring, unless this refers to a specific external hardware watchdog or legacy code comments.
3.  **Visual Feedback (The Panic Blink)**:
    *   A `for` loop executes 10 times.
    *   It toggles `LED_PIN` LOW and HIGH with 100ms delays (rapid strobe).
    *   This distinguishes this specific error state from normal operation (usually slower blinks) or WiFi connection (usually solid or rhythmic).
4.  **Watchdog/Recovery**:
    *   `delay(5000)`: Pauses for 5 seconds to allow the user to read the serial logs or see the blinking LED.
    *   `ESP.restart()`: Forces a reboot to prevent the device from hanging in an undefined state.

### 5. Integration with System
*   **One-Shot Architecture**: This block confirms the firmware is designed as a "One-Shot" system. The intended lifecycle is: **Boot -> Setup (Run Tasks) -> Deep Sleep -> Wake (Reset)**.
*   **Hardware Dependencies**: It relies on `LED_PIN` (defined in Block 01/02) for visual feedback.
*   **Debugging**: This block is the primary indicator for developers that the Sleep subsystem or Hardware Wakeup circuitry is malfunctioning.

---
