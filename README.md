# Project Z.O.N.I.O. (ESP8266 Ecosystem)

> **Zone Online Network Input/Output**  
> *Edge Intelligence for the IoT Era.*

## 1. Vision & Philosophy

**Project Z.O.N.I.O.** represents a shift from the traditional "Dumb Pipe" IoT architecture (streaming raw data to the cloud) to a **"Fat Edge"** philosophy. 

Our devices don't just measure; they **Analyze**, **Decide**, and only then **Transmit**.

*   **Edge Intelligence**: We calculate trends (slope analysis), detect stability, and manage data buffers locally.
*   **Deep Sleep First**: The architecture is built around a "One-Shot" lifecycle (Wake $\rightarrow$ Work $\rightarrow$ Sleep) to maximize battery life, operating for months on standard cells.
*   **Modular Monolith**: A complex linear logic flow decomposed into strict, interchangeable blocks.

---

## 2. The Modular Block Architecture

This firmware is designed to be **LLM-Friendly** and **Human-Readable**. The codebase is structured as a "Logical Monolith" where specific functions are isolated into numbered **Blocks**. 

This allows for rapid refactoring. You can swap out the **Sensor Block (03)** for a different hardware driver without breaking the **Network Block (04)** or **Logic Block (05)**.

### The Structure
The firmware is composed of the following strict logical blocks:

| Block | Name | Responsibility |
| :--- | :--- | :--- |
| **01** | **Configuration & Globals** | Library imports, Pin Definitions, and Secrets (stripped in repo). |
| **02** | **Memory & Integrity** | CRC32-protected `RTCData` (Sleep) and `EEPROMData` (Flash) structures. |
| **03** | **Hardware Abstraction** | Drivers for BME280, VEML7700, or generic inputs. Handles I2C multiplexing. |
| **04** | **Network Logic** | WiFi/MQTT connection managers with exponential backoff and "Birth" messages. |
| **05** | **Watchdogs & Safety** | Error handling, stability detection, and self-healing algorithms. |
| **06** | **Initialization** | The `setup()` sequence. Orchestrates hardware boot and initial checks. |
| **07** | **Scheduler Loop** | The main execution loop. Manages the "One-Shot" logic flow and Sleep entry. |

> **Tip for Developers**: When asking an LLM to modify this code, reference the Block Number (e.g., *"Refactor Block 03 to support a DHT22 sensor"*).

---

## 3. Branches & Roadmap

v1.6 single i2c sensor with dynamic sampling
v5 Deep sleep integration

---

## 4. Security & Limitations

This project is classified as **Low Application Type**. Security is balanced against extreme power efficiency.

### ⚠️ Security Constraints
1.  **Hardcoded Credentials**: WiFi SSID, Password, and MQTT Credentials are defined in code (Block 01). 
    *   *Note*: All releases on this GitHub have these credentials **stripped**. You must populate them in `config.h` (or Block 01) before compiling.
2.  **No Secure Boot**: This firmware is designed for the user's private, trusted local network. It does **not** implement Secure Boot or Flash Encryption features found in enterprise ESP32 deployments.
3.  **Transport**: Default MQTT communication is unencrypted (Port 1883) to save 2s of handshake battery life. Use on trusted VLANs only.

### ✅ Data Integrity
Despite the "Low" security classification, **Data Integrity** is "High".
*   **CRC32 Checksums**: Every save to RTC memory or Flash is strictly validated.
*   **Self-Healing**: Devices detect memory corruption (brownouts) and factory-reset their internal state automatically to prevent bad data ingestion.

---

## 5. Getting Started

1.  Pick FW in branch.
2.  Clone the repository.
3.  Open the project in **Arduino IDE**.
4.  Navigate to **Block 01** and populate your:
    *   `WIFI_SSID`
    *   `WIFI_PASSWORD`
    *   `MQTT_SERVER`
5.  Compile and Upload.

---

* 2025-2026 Project Z.O.N.I.O. - Open Source Intelligence for the Edge.*
