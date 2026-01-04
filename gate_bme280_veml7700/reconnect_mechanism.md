Zonio Firmware Standard: Robust Reconnect Logic
Overview
This document defines the standard logic for handling network connectivity in Zonio ESP8266/ESP32 firmwares. The goal is to ensure the device:

Never gets stuck in a failed state.
Always prioritizes configuration access (Config Gate) when the network is down.
Self-heals via restart if connectivity is impossible for a prolonged period.
The Logic Flow
The logic replaces the traditional "connect in setup, then loop" approach with a state-machine-like loop that constantly evaluates the environment.

1. The Priority Hierarchy
When disconnected, the device performs the following checks in order:

Check Hardware/Stack Health: Has it been offline for >1 hour? -> RESTART.
Check User Intent: Is the "Zonio-Gate" AP visible? -> SWITCH TO GATE MODE.
Attempt Recovery: Try connecting to the configured WiFi -> RETRY.
2. Detailed Behavior
Startup: The device attempts a quick connection (approx 4-5s). If it fails, it does not immediately switch to Gate Mode or block. It falls through to the main loop to begin the recovery cycle.
Recovery Loop (Offline State):
Executed every 15 seconds (to prevent spamming and allow time for scanning).
Step A: Passive Scan. The device scans for WiFi networks.
Step B: Gate Detection. If Zonio-Gate (or configured Gate SSID) is found in the scan results, the device immediately aborts the reconnect attempt and starts the Config Gate (AP Mode + Web Server).
Step C: WiFi Retry. If the Gate is not found, WiFi.begin() is called to attempt connection to the saved credentials.
Safety Net: A timer (disconnectStartTime) tracks how long the device has been offline. If this exceeds 3600000ms (1 hour), ESP.restart() is triggered. This clears memory leaks, stack corruption, or hardware radio glitches.
Implementation Guide
A. Global Variables
Add these to the global scope:

// ===== RECONNECT STATE =====
unsigned long mqttReconnectInterval = 5000;  // RECONNECT_BASE
int reconnectCount = 0;
int wifiReconnectAttempts = 0;
unsigned long disconnectStartTime = 0;       // For 1h timeout
unsigned long lastWiFiReconnectAttempt = 0;
bool wasEverConnected = false;               // To verify stability
B. Setup Phase
Modify 

setupNormalMode()
 (or 

setup()
) to be non-blocking. Do not use while(WiFi.status() != WL_CONNECTED) infinite loops.

void setupNormalMode() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  
  // Optional: Brief wait (e.g., 4 seconds) to catch fast connections
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(200); 
    retries++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    // Standard success logic
  } else {
    // IMPORTANT: Just log and exit setup. 
    // The loop() will handle the rest. DO NOT switch to GateMode here.
    Serial.println("Entering Recovery Loop...");
  }
}
C. The Standard Loop
Replace the connection logic in your 

loop()
 with this standard block:

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
    if (now - disconnectStartTime > 3600000) { 
       Serial.println("[System] Offline for >1h. EMERGENCY RESTART.");
       ESP.restart();
    }
    // C) Recovery Loop (Every 15s)
    if (now - lastWiFiReconnectAttempt > 15000) {  
      lastWiFiReconnectAttempt = now;
      wifiReconnectAttempts++;
      
      // Step 1: Scan for Zonio Gate
      Serial.println("[WiFi] Scanning for Zonio Gate...");
      int n = WiFi.scanNetworks();
      bool gateFound = false;
      for (int i=0; i<n; i++) {
        if (WiFi.SSID(i) == "Zonio-Gate") { // Or GATE_AP_SSID
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
         Serial.println("[WiFi] Retrying Configured WiFi...");
         WiFi.begin(deviceConfig.wifi_ssid, deviceConfig.wifi_password);
      }
    }
    
    // Fast blink to indicate searching
    static unsigned long lastBlink = 0;
    if (now - lastBlink > 500) {
       lastBlink = now;
       digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
    }
    return; // Stop here, don't try MQTT or Sensors while offline
  } 
  
  // ==========================================================
  // 2. CONNECTED STATE MAINTENANCE
  // ==========================================================
  if (WiFi.status() == WL_CONNECTED) {
    if (disconnectStartTime != 0) {
       Serial.println("[WiFi] Reconnected!");
       disconnectStartTime = 0;
       wifiReconnectAttempts = 0;
       mqttReconnectInterval = 5000; // Reset MQTT backoff
       digitalWrite(LED_PIN, HIGH);  // Stable status (LED Off)
    }
  }
  // ... Continue with MQTT and Application Logic ...
}
Benefits of this Approach
Field Serviceability: To reconfigure a device mounted on a roof/pole, you don't need to climb up and press a button. Just create a hotspot named "Zonio-Gate", and the device will find it within 15 seconds and switch to config mode.
Robustness: Temporary WiFi outages (router reboots, ISP issues) are handled gracefully without freezing.
Long-term Stability: The 1h restart ensures that if the ESP8266 radio stack gets stuck (a known issue), it will recover automatically.