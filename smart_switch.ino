/*
 * smart_switch — ESP32-C3 mains relay switch
 * Raw Arduino/C++, espMqttClient, MQTT QoS 1, HA auto-discovery
 *
 * Mirrors the conventions used in the SHTC3 temp/humidity sensor and the
 * door sensor projects: espMqttClient, retained discovery configs, LWT
 * availability, diagnostic entities. Unlike those, this device is
 * mains-powered, so there is no deep sleep — it stays connected
 * continuously.
 *
 * As of v2.0.0: WiFi, MQTT, and device identity are no longer compiled
 * into secrets.h — they're configured at runtime via a built-in
 * WiFiManager web portal and persisted in NVS, same system as
 * door_sensor/TH_2_v4. This device already has a spare button and LED
 * (unlike door_sensor), so it follows TH_2_v4's convention: the portal is
 * reached via a physical button-hold gesture, not an MQTT switch.
 *   - Short press (as before): toggles the relay.
 *   - Held BUTTON_SETUP_HOLD_MS (10s): opens the setup portal immediately
 *     (doesn't wait for release) — does NOT also toggle the relay.
 *   - Held FACTORY_RESET_HOLD_MS (5s) again while the portal is open:
 *     wipes all saved settings + the radio's own saved WiFi credentials
 *     and restarts unconfigured. A quick press instead cancels the portal.
 *   - A never-configured device goes straight to the portal on boot.
 * No button-hold OTA gesture is needed here (unlike TH_2_v4, which is
 * normally asleep) — ArduinoOTA already runs continuously in loop() on
 * this always-on device.
 *
 * Diagnostics: WiFi signal, reset reason, boot count, connect-fail count
 * (resets on next successful connect) and total-fail count (lifetime,
 * NVS-persisted), and firmware version — all published/discovered
 * automatically, same set as the rest of the fleet minus anything
 * battery-specific.
 *
 * Hardware (from the ESPHome sketch this replaces):
 *   GPIO1  - relay control (active high)
 *   GPIO5  - physical button, active low, internal pullup
 *   GPIO10 - single WS2812 status LED
 *
 * Libraries needed (Library Manager / PlatformIO):
 *   espMqttClient   (bertmelis/espMqttClient)
 *   Adafruit NeoPixel
 *   WiFiManager     (tzapu/WiFiManager)
 */

#include <WiFi.h>
#include <WiFiManager.h>
#include <espMqttClient.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoOTA.h>
#include <esp_system.h>
#include <Preferences.h>
#include <vector>
// OTA password, setup-portal AP password/timeout, device identity/firmware
// version, and button-hold thresholds. WiFi/MQTT credentials are NOT here
// any more as of v2.0.0 -- they're runtime settings, see Settings below.
#include "secrets.h"

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------
static const uint8_t RELAY_PIN  = 1;
static const uint8_t BUTTON_PIN = 5;
static const uint8_t LED_PIN    = 10;
static const uint8_t LED_COUNT  = 1;

// Default LED brightness as a percentage (0-100), used until a value is
// loaded from NVS or set via MQTT/HA.
static const uint8_t DEFAULT_LED_BRIGHTNESS_PCT = 50;

// ---------------------------------------------------------------------------
// Runtime settings (WiFi/MQTT/identity, via the setup portal)
// ---------------------------------------------------------------------------
// Nothing here is compiled in -- read from NVS at boot (defaults to
// unconfigured) and only ever written by runMaintenanceMode() after a
// portal save, or wiped by a button-hold factory reset. Declared before
// buildTopics() below since that reads settings.deviceId.
struct Settings {
  String wifiSsid;
  String wifiPassword;
  String mqttHost;
  uint16_t mqttPort = 1883;
  String mqttUser;
  String mqttPassword;
  String deviceId;   // used in MQTT topics/unique_ids -- keep stable once this device exists in HA
  String deviceName; // friendly name shown in Home Assistant
  bool configured = false;
};
Settings settings;
Preferences settingsPrefs;

String getShortChipId() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[7];
  snprintf(buf, sizeof(buf), "%06X", (uint32_t)(mac & 0xFFFFFFULL));
  return String(buf);
}

void loadSettings() {
  String chipId = getShortChipId();
  settingsPrefs.begin("settings", true); // read-only
  settings.configured   = settingsPrefs.getBool("configured", false);
  settings.wifiSsid     = settingsPrefs.getString("wifiSsid", "");
  settings.wifiPassword = settingsPrefs.getString("wifiPass", "");
  settings.mqttHost     = settingsPrefs.getString("mqttHost", "");
  settings.mqttPort     = settingsPrefs.getUShort("mqttPort", 1883);
  settings.mqttUser     = settingsPrefs.getString("mqttUser", "");
  settings.mqttPassword = settingsPrefs.getString("mqttPass", "");
  settings.deviceId     = settingsPrefs.getString("deviceId", "switch_" + chipId);
  settings.deviceName   = settingsPrefs.getString("deviceName", "Smart Switch " + chipId);
  settingsPrefs.end();
}

void saveSettings() {
  settingsPrefs.begin("settings", false);
  settingsPrefs.putBool("configured", settings.configured);
  settingsPrefs.putString("wifiSsid", settings.wifiSsid);
  settingsPrefs.putString("wifiPass", settings.wifiPassword);
  settingsPrefs.putString("mqttHost", settings.mqttHost);
  settingsPrefs.putUShort("mqttPort", settings.mqttPort);
  settingsPrefs.putString("mqttUser", settings.mqttUser);
  settingsPrefs.putString("mqttPass", settings.mqttPassword);
  settingsPrefs.putString("deviceId", settings.deviceId);
  settingsPrefs.putString("deviceName", settings.deviceName);
  settingsPrefs.end();
}

// Boot count / lifetime fail count: NVS-persisted, not RTC memory (unlike
// door_sensor's equivalents) -- this device doesn't deep-sleep, so a "boot"
// is a rare, meaningful event (actual power cycle, crash, or OTA restart),
// and there's no flash-wear concern from writing on every one of those.
uint32_t bootCount = 0;
uint32_t totalFailCount = 0; // lifetime MQTT disconnects, never resets

void loadCounters() {
  settingsPrefs.begin("settings", true);
  bootCount = settingsPrefs.getUInt("bootCount", 0);
  totalFailCount = settingsPrefs.getUInt("totalFail", 0);
  settingsPrefs.end();
}

void incrementBootCount() {
  bootCount++;
  settingsPrefs.begin("settings", false);
  settingsPrefs.putUInt("bootCount", bootCount);
  settingsPrefs.end();
}

void incrementTotalFailCount() {
  totalFailCount++;
  settingsPrefs.begin("settings", false);
  settingsPrefs.putUInt("totalFail", totalFailCount);
  settingsPrefs.end();
}

// ---------------------------------------------------------------------------
// MQTT topics -- built at runtime from settings.deviceId, not compiled in
// ---------------------------------------------------------------------------
String baseTopic, relayStateTopic, relayCommandTopic, availabilityTopic,
       buttonStateTopic, wifiSignalTopic, resetReasonTopic,
       connectFailCountTopic, totalFailCountTopic, bootCountTopic,
       firmwareVersionTopic, ledBrightnessStateTopic, ledBrightnessCommandTopic,
       otaRestartCommandTopic;

String discoverySwitchTopic, discoveryButtonTopic, discoveryWifiSignalTopic,
       discoveryResetReasonTopic, discoveryConnectFailCountTopic,
       discoveryTotalFailCountTopic, discoveryBootCountTopic,
       discoveryFirmwareVersionTopic, discoveryLedBrightnessTopic,
       discoveryOtaRestartTopic;

void buildTopics() {
  baseTopic = String("switch/") + settings.deviceId;
  relayStateTopic   = baseTopic + "/relay/state";
  relayCommandTopic = baseTopic + "/relay/set";
  availabilityTopic = baseTopic + "/availability";
  buttonStateTopic  = baseTopic + "/button/state";
  wifiSignalTopic   = baseTopic + "/wifi_signal/state";
  resetReasonTopic  = baseTopic + "/reset_reason/state";
  connectFailCountTopic = baseTopic + "/connect_fail_count/state";
  totalFailCountTopic   = baseTopic + "/total_fail_count/state";
  bootCountTopic        = baseTopic + "/boot_count/state";
  firmwareVersionTopic  = baseTopic + "/firmware_version/state";
  ledBrightnessStateTopic   = baseTopic + "/led_brightness/state";
  ledBrightnessCommandTopic = baseTopic + "/led_brightness/set";
  otaRestartCommandTopic    = baseTopic + "/ota_restart/set";

  discoverySwitchTopic       = String("homeassistant/switch/") + settings.deviceId + "/relay/config";
  discoveryButtonTopic       = String("homeassistant/binary_sensor/") + settings.deviceId + "/button/config";
  discoveryWifiSignalTopic   = String("homeassistant/sensor/") + settings.deviceId + "/wifi_signal/config";
  discoveryResetReasonTopic  = String("homeassistant/sensor/") + settings.deviceId + "/reset_reason/config";
  discoveryConnectFailCountTopic = String("homeassistant/sensor/") + settings.deviceId + "/connect_fail_count/config";
  discoveryTotalFailCountTopic   = String("homeassistant/sensor/") + settings.deviceId + "/total_fail_count/config";
  discoveryBootCountTopic        = String("homeassistant/sensor/") + settings.deviceId + "/boot_count/config";
  discoveryFirmwareVersionTopic  = String("homeassistant/sensor/") + settings.deviceId + "/firmware_version/config";
  discoveryLedBrightnessTopic = String("homeassistant/number/") + settings.deviceId + "/led_brightness/config";
  discoveryOtaRestartTopic    = String("homeassistant/button/") + settings.deviceId + "/ota_restart/config";
}

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
espMqttClient mqttClient;
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
Preferences prefs;

bool relayState = false;
uint8_t ledBrightnessPct = DEFAULT_LED_BRIGHTNESS_PCT; // 0-100, persisted in NVS

// button debounce + hold tracking
bool lastButtonReading = HIGH;
bool buttonStable = HIGH;
unsigned long lastButtonChangeMs = 0;
static const unsigned long DEBOUNCE_MS = 20;
unsigned long pressStartMs = 0;  // 0 == not currently tracking a press
bool setupTriggered = false;     // true once the current press has already opened the portal

// mqtt reconnect / diagnostics
unsigned long lastMqttAttemptMs = 0;
unsigned long mqttBackoffMs = 1000;
static const unsigned long MQTT_BACKOFF_MAX_MS = 30000;
uint32_t connectFailCount = 0; // resets to 0 on next successful connect
bool everConnected = false;

unsigned long lastWifiSignalPublishMs = 0;
static const unsigned long WIFI_SIGNAL_INTERVAL_MS = 120000; // 2 min, matches your wifi_signal sensor

bool bootAnimationDone = false;

// wifi reconnect state (non-blocking)
bool wifiConnectInProgress = false;
unsigned long wifiConnectStartMs = 0;
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000;

// setup-portal LED pulse (used during a hold and while the portal's open)
static const unsigned long SETUP_LED_BLINK_PERIOD_MS = 1000;
static const unsigned long SETUP_LED_PULSE_MS = 150;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void connectWiFi();
void pollWiFi();
bool checkedPublish(const String &topic, uint8_t qos, bool retain, const String &payload);
void connectMqtt();
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason);
void onMqttMessage(const espMqttClientTypes::MessageProperties& properties,
                    const char* topic, const uint8_t* payload, size_t len,
                    size_t index, size_t total);
void publishDiscovery();
void publishRelayState();
void setRelay(bool on, bool publish);
void handleButton();
void updateLedForRelayState();
void runBootAnimation();
void publishDiagnostics(bool force);
String resetReasonString();
void setLedBrightness(uint8_t pct, bool save, bool publish);
void publishLedBrightness();
void setLedColor(uint8_t r, uint8_t g, uint8_t b);
void runMaintenanceMode(bool viaButton);

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Write the level before enabling as OUTPUT so the pin doesn't have an
  // undefined/floating moment between reset and this line taking effect.
  digitalWrite(RELAY_PIN, LOW); // default off, matches RESTORE_DEFAULT_OFF
  pinMode(RELAY_PIN, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  prefs.begin("smart_switch", false);
  ledBrightnessPct = prefs.getUChar("led_bright", DEFAULT_LED_BRIGHTNESS_PCT);
  if (ledBrightnessPct > 100) ledBrightnessPct = DEFAULT_LED_BRIGHTNESS_PCT;

  led.begin();
  led.setBrightness(map(ledBrightnessPct, 0, 100, 0, 255));
  led.clear();
  led.show();

  loadSettings();
  loadCounters();
  buildTopics();

  // A never-configured device goes straight to the portal -- no button
  // hold needed, same as door_sensor/TH_2_v4.
  if (!settings.configured) {
    runMaintenanceMode(false);
    // Only reached if the portal timed out / failed -- nothing saved yet,
    // so just keep retrying instead of falling through to a normal cycle
    // with empty WiFi/MQTT settings.
    Serial.println("[setup] Still unconfigured after portal timeout -- retrying.");
    delay(2000);
    ESP.restart();
  }

  incrementBootCount();

  connectWiFi();

  // One-time bounded wait at boot only — gives ArduinoOTA's mDNS responder
  // and the first MQTT attempt a real chance at a live link. This never
  // recurs after setup(), so it never blocks the button during normal
  // operation the way the old loop()-level blocking wait did.
  {
    unsigned long waitStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - waitStart < 5000) {
      delay(100);
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnectInProgress = false;
    Serial.printf("[wifi] connected, ip=%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[wifi] not yet connected at boot, will keep retrying in loop()");
  }

  mqttClient.setServer(settings.mqttHost.c_str(), settings.mqttPort);
  mqttClient.setCredentials(settings.mqttUser.c_str(), settings.mqttPassword.c_str());
  mqttClient.setClientId(settings.deviceId.c_str());
  mqttClient.setWill(availabilityTopic.c_str(), 1, true, "offline");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  connectMqtt();

  ArduinoOTA.setHostname(settings.deviceId.c_str());
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------
void loop() {
  pollWiFi();

  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastMqttAttemptMs >= mqttBackoffMs) {
      connectMqtt();
    }
  }

  mqttClient.loop();
  ArduinoOTA.handle();

  handleButton();

  if (!bootAnimationDone && mqttClient.connected()) {
    runBootAnimation();
    bootAnimationDone = true;
  }

  unsigned long now = millis();
  if (now - lastWifiSignalPublishMs >= WIFI_SIGNAL_INTERVAL_MS) {
    lastWifiSignalPublishMs = now;
    publishDiagnostics(false);
  }
}

// ---------------------------------------------------------------------------
// MQTT publish helper — logs a failure instead of silently dropping it.
// mqttClient.publish() returns 0 on failure (e.g. espMqttClient's internal
// low-memory guard, or not connected); every call site was previously
// ignoring that, so a failed publish was invisible.
// ---------------------------------------------------------------------------
bool checkedPublish(const String &topic, uint8_t qos, bool retain, const String &payload) {
  uint16_t packetId = mqttClient.publish(topic.c_str(), qos, retain, payload.c_str());
  if (packetId == 0) {
    Serial.print("[mqtt] publish FAILED topic=");
    Serial.print(topic);
    Serial.print(" free_heap=");
    Serial.println(ESP.getFreeHeap());
  }
  return packetId != 0;
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED || wifiConnectInProgress) return;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // matches power_save_mode: NONE
  WiFi.begin(settings.wifiSsid.c_str(), settings.wifiPassword.c_str());
  wifiConnectInProgress = true;
  wifiConnectStartMs = millis();
  Serial.println("[wifi] connecting...");
}

// Non-blocking — call every loop() iteration. Kicks off a connect attempt
// if needed, and lets a stuck attempt time out and retry without ever
// stalling the rest of loop() (button, MQTT, OTA) the way a blocking
// while-wait would.
void pollWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiConnectInProgress) {
      wifiConnectInProgress = false;
      Serial.printf("[wifi] connected, ip=%s\n", WiFi.localIP().toString().c_str());
    }
    return;
  }

  if (!wifiConnectInProgress) {
    connectWiFi();
  } else if (millis() - wifiConnectStartMs > WIFI_CONNECT_TIMEOUT_MS) {
    Serial.println("[wifi] connect attempt timed out, will retry");
    wifiConnectInProgress = false; // next call to pollWiFi() starts a fresh attempt
  }
}

// ---------------------------------------------------------------------------
// MQTT
// ---------------------------------------------------------------------------
void connectMqtt() {
  lastMqttAttemptMs = millis();
  if (WiFi.status() != WL_CONNECTED) return;
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("MQTT connected");
  mqttBackoffMs = 1000; // reset backoff on success
  everConnected = true;
  connectFailCount = 0; // this run's fail streak ends on a successful connect

  checkedPublish(availabilityTopic, 1, true, "online");

  mqttClient.subscribe(relayCommandTopic.c_str(), 1);
  mqttClient.subscribe(ledBrightnessCommandTopic.c_str(), 1);
  mqttClient.subscribe(otaRestartCommandTopic.c_str(), 1);

  publishDiscovery();
  publishRelayState();
  publishLedBrightness();
  updateLedForRelayState();
  publishDiagnostics(true);
}

void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason) {
  Serial.printf("MQTT disconnected, reason: %u\n", static_cast<uint8_t>(reason));
  if (everConnected) {
    connectFailCount++;
    incrementTotalFailCount();
  }
  mqttBackoffMs = min(mqttBackoffMs * 2, MQTT_BACKOFF_MAX_MS);
}

void onMqttMessage(const espMqttClientTypes::MessageProperties& properties,
                    const char* topic, const uint8_t* payload, size_t len,
                    size_t index, size_t total) {
  String topicStr(topic);
  String payloadStr;
  payloadStr.reserve(len);
  for (size_t i = 0; i < len; i++) payloadStr += (char)payload[i];

  if (topicStr == relayCommandTopic) {
    bool wantOn = payloadStr.equalsIgnoreCase("ON");
    setRelay(wantOn, true);
  } else if (topicStr == ledBrightnessCommandTopic) {
    int pct = payloadStr.toInt();
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    setLedBrightness((uint8_t)pct, true, true);
  } else if (topicStr == otaRestartCommandTopic) {
    Serial.println("OTA restart requested via MQTT, rebooting...");
    checkedPublish(availabilityTopic, 1, true, "offline");
    delay(200); // give the publish a moment to go out before we drop the link
    ESP.restart();
  }
}

// ---------------------------------------------------------------------------
// Home Assistant discovery
// ---------------------------------------------------------------------------
void publishDiscovery() {
  String deviceJson = String("{") +
      "\"identifiers\":[\"" + settings.deviceId + "\"]," +
      "\"name\":\"" + settings.deviceName + "\"," +
      "\"manufacturer\":\"" + DEVICE_MANUFACTURER + "\"," +
      "\"model\":\"" + DEVICE_MODEL + "\"," +
      "\"hw_version\":\"" + DEVICE_HW_VERSION + "\"," +
      "\"sw_version\":\"" + FIRMWARE_VERSION + "\"" +
      "}";

  // Switch (relay)
  {
    String payload = String("{") +
        "\"name\":\"Relay\"," +
        "\"unique_id\":\"" + settings.deviceId + "_relay\"," +
        "\"state_topic\":\"" + relayStateTopic + "\"," +
        "\"command_topic\":\"" + relayCommandTopic + "\"," +
        "\"payload_on\":\"ON\"," +
        "\"payload_off\":\"OFF\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoverySwitchTopic, 1, true, payload);
  }

  // Button (binary_sensor, momentary)
  {
    String payload = String("{") +
        "\"name\":\"Button\"," +
        "\"unique_id\":\"" + settings.deviceId + "_button\"," +
        "\"state_topic\":\"" + buttonStateTopic + "\"," +
        "\"payload_on\":\"ON\"," +
        "\"payload_off\":\"OFF\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryButtonTopic, 1, true, payload);
  }

  // WiFi signal (diagnostic)
  {
    String payload = String("{") +
        "\"name\":\"WiFi Signal\"," +
        "\"unique_id\":\"" + settings.deviceId + "_wifi_signal\"," +
        "\"state_topic\":\"" + wifiSignalTopic + "\"," +
        "\"unit_of_measurement\":\"dBm\"," +
        "\"device_class\":\"signal_strength\"," +
        "\"state_class\":\"measurement\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryWifiSignalTopic, 1, true, payload);
  }

  // Reset reason (diagnostic)
  {
    String payload = String("{") +
        "\"name\":\"Reset Reason\"," +
        "\"unique_id\":\"" + settings.deviceId + "_reset_reason\"," +
        "\"state_topic\":\"" + resetReasonTopic + "\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryResetReasonTopic, 1, true, payload);
  }

  // Boot count (diagnostic)
  {
    String payload = String("{") +
        "\"name\":\"Boot Count\"," +
        "\"unique_id\":\"" + settings.deviceId + "_boot_count\"," +
        "\"state_topic\":\"" + bootCountTopic + "\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"state_class\":\"total_increasing\"," +
        "\"icon\":\"mdi:counter\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryBootCountTopic, 1, true, payload);
  }

  // Connect fail count (diagnostic) -- resets to 0 on next successful
  // connect, so "measurement" not "total_increasing" (which HA would read
  // as a meter that only ever counts up, same as door_sensor's equivalent).
  {
    String payload = String("{") +
        "\"name\":\"Connect Fail Count\"," +
        "\"unique_id\":\"" + settings.deviceId + "_connect_fail_count\"," +
        "\"state_topic\":\"" + connectFailCountTopic + "\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"state_class\":\"measurement\"," +
        "\"icon\":\"mdi:wifi-alert\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryConnectFailCountTopic, 1, true, payload);
  }

  // Total fail count (diagnostic) -- lifetime, never resets
  {
    String payload = String("{") +
        "\"name\":\"Total Fail Count\"," +
        "\"unique_id\":\"" + settings.deviceId + "_total_fail_count\"," +
        "\"state_topic\":\"" + totalFailCountTopic + "\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"state_class\":\"total_increasing\"," +
        "\"icon\":\"mdi:counter\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryTotalFailCountTopic, 1, true, payload);
  }

  // Firmware version (diagnostic)
  {
    String payload = String("{") +
        "\"name\":\"Firmware Version\"," +
        "\"unique_id\":\"" + settings.deviceId + "_firmware_version\"," +
        "\"state_topic\":\"" + firmwareVersionTopic + "\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryFirmwareVersionTopic, 1, true, payload);
  }

  // LED brightness (number entity, global brightness control)
  {
    String payload = String("{") +
        "\"name\":\"LED Brightness\"," +
        "\"unique_id\":\"" + settings.deviceId + "_led_brightness\"," +
        "\"state_topic\":\"" + ledBrightnessStateTopic + "\"," +
        "\"command_topic\":\"" + ledBrightnessCommandTopic + "\"," +
        "\"min\":0," +
        "\"max\":100," +
        "\"step\":1," +
        "\"unit_of_measurement\":\"%\"," +
        "\"icon\":\"mdi:brightness-percent\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryLedBrightnessTopic, 1, true, payload);
  }

  // OTA restart button (diagnostic) — reboots cleanly before an OTA push
  {
    String payload = String("{") +
        "\"name\":\"OTA Restart\"," +
        "\"unique_id\":\"" + settings.deviceId + "_ota_restart\"," +
        "\"command_topic\":\"" + otaRestartCommandTopic + "\"," +
        "\"payload_press\":\"PRESS\"," +
        "\"device_class\":\"restart\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryOtaRestartTopic, 1, true, payload);
  }
}

// ---------------------------------------------------------------------------
// Relay
// ---------------------------------------------------------------------------
void setRelay(bool on, bool publish) {
  relayState = on;
  digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
  updateLedForRelayState();
  if (publish) publishRelayState();
}

void publishRelayState() {
  if (!mqttClient.connected()) return;
  checkedPublish(relayStateTopic, 1, true, relayState ? "ON" : "OFF");
}

// ---------------------------------------------------------------------------
// Button (GPIO5, active low, debounced)
//
// Short press (release before BUTTON_SETUP_HOLD_MS): toggles the relay,
// same as before v2.0.0. Held past BUTTON_SETUP_HOLD_MS: commits
// immediately (without waiting for release, so a long hold feels
// immediate) to opening the setup portal instead -- the relay is NOT
// toggled for that press. See runMaintenanceMode() for the in-portal
// factory-reset gesture.
// ---------------------------------------------------------------------------
void handleButton() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastButtonChangeMs = millis();
  }

  if (millis() - lastButtonChangeMs > DEBOUNCE_MS && reading != buttonStable) {
    buttonStable = reading;

    // active low: pressed == LOW
    bool pressed = (buttonStable == LOW);

    if (mqttClient.connected()) {
      checkedPublish(buttonStateTopic, 1, false, pressed ? "ON" : "OFF");
    }

    if (pressed) {
      pressStartMs = millis();
      setupTriggered = false;
    } else {
      if (!setupTriggered) {
        setRelay(!relayState, true);
      }
      pressStartMs = 0;
    }
  }

  // Long-hold detection, independent of the debounce-driven edges above --
  // has to fire the instant the threshold is crossed, not wait for release.
  if (pressStartMs != 0 && !setupTriggered && buttonStable == LOW &&
      millis() - pressStartMs >= BUTTON_SETUP_HOLD_MS) {
    setupTriggered = true;
    Serial.println("[button] held past setup threshold -- entering Setup Mode.");
    runMaintenanceMode(true);
    // runMaintenanceMode() only returns on portal timeout/cancel; resync
    // debounce state in case the button is still held, so it doesn't
    // immediately retrigger or register a spurious release-toggle.
    lastButtonReading = digitalRead(BUTTON_PIN);
    buttonStable = lastButtonReading;
    pressStartMs = 0;
    setupTriggered = false;
  }

  lastButtonReading = reading;
}

// ---------------------------------------------------------------------------
// LED status
// ---------------------------------------------------------------------------
void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

void updateLedForRelayState() {
  if (relayState) {
    setLedColor(255, 0, 0); // red, relay on
  } else if (WiFi.status() == WL_CONNECTED) {
    setLedColor(0, 255, 0); // green, relay off, wifi connected
  } else {
    setLedColor(0, 0, 0); // off, no wifi
  }
}

void setLedBrightness(uint8_t pct, bool save, bool publish) {
  ledBrightnessPct = pct;
  led.setBrightness(map(ledBrightnessPct, 0, 100, 0, 255));
  led.show(); // redraw current color at the new brightness

  if (save) {
    prefs.putUChar("led_bright", ledBrightnessPct);
  }
  if (publish) {
    publishLedBrightness();
  }
}

void publishLedBrightness() {
  if (!mqttClient.connected()) return;
  checkedPublish(ledBrightnessStateTopic, 1, true, String(ledBrightnessPct));
}

void runBootAnimation() {
  for (int i = 0; i < 3; i++) {
    setLedColor(0, 0, 255); // blue
    delay(700);
    setLedColor(0, 0, 0);
    delay(700);
  }
  updateLedForRelayState();
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
String resetReasonString() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_POWERON:   return "Power on";
    case ESP_RST_EXT:       return "External pin";
    case ESP_RST_SW:        return "Software reset";
    case ESP_RST_PANIC:     return "Panic/exception";
    case ESP_RST_INT_WDT:   return "Interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "Task watchdog";
    case ESP_RST_WDT:       return "Other watchdog";
    case ESP_RST_BROWNOUT:  return "Brownout";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "Unknown";
  }
}

void publishDiagnostics(bool force) {
  if (!mqttClient.connected()) return;

  if (WiFi.status() == WL_CONNECTED) {
    long rssi = WiFi.RSSI();
    checkedPublish(wifiSignalTopic, 1, true, String(rssi));
  }

  static bool resetReasonSent = false;
  if (force || !resetReasonSent) {
    checkedPublish(resetReasonTopic, 1, true, resetReasonString());
    resetReasonSent = true;
  }

  checkedPublish(bootCountTopic, 1, true, String(bootCount));
  checkedPublish(connectFailCountTopic, 1, true, String(connectFailCount));
  checkedPublish(totalFailCountTopic, 1, true, String(totalFailCount));
  checkedPublish(firmwareVersionTopic, 1, true, String(FIRMWARE_VERSION));
}

// ---------------------------------------------------------------------------
// Setup portal
//
// Entered when the button is held past BUTTON_SETUP_HOLD_MS, or when this
// device has never been configured yet (settings.configured == false).
// Broadcasts "<DEVICE_MANUFACTURER> <DEVICE_MODEL> XXXX" (last 4 hex chars
// of the chip MAC), and serves a page (WiFiManager) with a WiFi picker plus
// custom fields for MQTT and device identity -- one form, one Save. Holding
// the button again for FACTORY_RESET_HOLD_MS while this page is open wipes
// the device back to a fully unconfigured state instead. If the portal
// succeeds, settings are saved and the device restarts. If it times out or
// is cancelled, this returns and the caller resumes with whatever settings
// already existed (unchanged).
//
// Note: a successful save always ends in ESP.restart(), which re-runs
// setup() from scratch -- the relay briefly goes to its default-off state
// during that reboot, same as any other restart of this device.
// ---------------------------------------------------------------------------
void runMaintenanceMode(bool viaButton) {
  Serial.println(viaButton
    ? "Button held >=10s -- entering Setup Mode."
    : "No saved WiFi config yet -- entering first-time setup.");

  // Clean disconnect before handing the radio to WiFiManager, in case this
  // was reached mid-operation (viaButton) on an already-connected device.
  if (mqttClient.connected()) {
    checkedPublish(availabilityTopic, 1, true, "offline");
    mqttClient.disconnect();
    delay(200);
  }
  WiFi.disconnect();

  String wifiStatusStr = settings.configured
    ? (settings.wifiSsid.length() ? ("last connected: " + settings.wifiSsid) : String("no WiFi saved yet"))
    : String("not yet configured");
  String mqttStatusStr = (settings.configured && settings.mqttHost.length())
    ? (settings.mqttHost + ":" + String(settings.mqttPort))
    : String("not yet configured");
  String statusHtml = String("<div style='background:#f4f4f4;border-radius:6px;padding:10px;margin:10px 0;font-size:0.9em;'>")
    + "<strong>Device status</strong><br>"
    + "Relay: " + (relayState ? "ON" : "OFF") + "<br>"
    + "WiFi: " + wifiStatusStr + "<br>"
    + "MQTT broker: " + mqttStatusStr + "<br>"
    + "Boot count: " + String(bootCount) + " &middot; connect fails: " + String(connectFailCount)
    + " this run / " + String(totalFailCount) + " total"
    + "</div>";

  char mqttPortStr[6];
  snprintf(mqttPortStr, sizeof(mqttPortStr), "%u", settings.mqttPort);

  WiFiManagerParameter p_mqtt_heading(
    "<p style='margin-bottom:0;'><strong>MQTT &amp; device settings</strong><br>"
    "(same form as the WiFi network above -- fill in both, then Save once)</p>");
  WiFiManagerParameter p_mqtt_host("mqtt_host", "MQTT broker host or IP", settings.mqttHost.c_str(), 64, "required");
  WiFiManagerParameter p_mqtt_port("mqtt_port", "MQTT broker port", mqttPortStr, 6);
  WiFiManagerParameter p_mqtt_user("mqtt_user", "MQTT username", settings.mqttUser.c_str(), 32);
  WiFiManagerParameter p_mqtt_pass("mqtt_pass", "MQTT password", settings.mqttPassword.c_str(), 32, "type='password'");
  WiFiManagerParameter p_device_name("device_name", "Device name (shown in Home Assistant)", settings.deviceName.c_str(), 40);
  WiFiManagerParameter p_device_id("device_id", "Device ID (MQTT topics, no spaces)", settings.deviceId.c_str(), 32);

  WiFiManager wm;
  // Shown at the top of every portal page -- see door_sensor/TH_2_v4 for
  // why this uses setCustomHeadElement() rather than setCustomBodyHeader().
  String versionHeader = "<style>body::before{content:'" + String(DEVICE_MANUFACTURER) + " " + String(DEVICE_MODEL)
                          + "\\A Firmware v" + String(FIRMWARE_VERSION)
                          + "';white-space:pre-line;display:block;text-align:center;color:#888;margin:4px 0;}"
                          + "form[action='/wifi'] button{font-size:0;}"
                          + "form[action='/wifi'] button::after{content:'Configure';font-size:1rem;}"
                          + "h1,h3{display:none;}</style>";
  wm.setCustomHeadElement(versionHeader.c_str());
  wm.addParameter(&p_mqtt_heading);
  wm.addParameter(&p_mqtt_host);
  wm.addParameter(&p_mqtt_port);
  wm.addParameter(&p_mqtt_user);
  wm.addParameter(&p_mqtt_pass);
  wm.addParameter(&p_device_name);
  wm.addParameter(&p_device_id);
  wm.setConfigPortalTimeout(PORTAL_TIMEOUT_SEC);
  // See door_sensor/TH_2_v4 for why "param" is deliberately left out of the
  // menu -- keeps the MQTT/device fields on the same "Configure WiFi" page
  // as the network picker instead of a separate, easy-to-miss page.
  std::vector<const char*> menu = {"custom", "wifi", "sep", "restart", "exit"};
  wm.setMenu(menu);
  wm.setCustomMenuHTML(statusHtml.c_str());
  wm.setCaptivePortalEnable(true);

  // WPA2 requires an 8-63 character password -- fall back to an open setup
  // network rather than failing silently on a bad secrets.h value.
  const char* apPassword = AP_PASSWORD;
  size_t apPasswordLen = strlen(apPassword);
  if (apPasswordLen > 0 && apPasswordLen < 8) {
    Serial.printf("[setup] AP_PASSWORD is %u characters -- WPA2 needs at least 8, "
                  "falling back to an OPEN setup network instead of failing silently.\n",
                  (unsigned)apPasswordLen);
    apPassword = nullptr;
  }

  String apName = String(DEVICE_MANUFACTURER) + " " + String(DEVICE_MODEL) + " " + getShortChipId().substring(2);
  wm.setConfigPortalBlocking(false);
  wm.startConfigPortal(apName.c_str(), apPassword);

  // Holding the button again for FACTORY_RESET_HOLD_MS while the portal is
  // open wipes settings + the radio's own saved WiFi credentials and
  // restarts unconfigured. A quick press-and-release cancels the portal
  // instead. sawIdleSinceEntry guards against the tail of the very hold
  // that opened this portal (viaButton path) being misread as a cancel.
  unsigned long resetHoldStart = 0;
  bool sawIdleSinceEntry = false;
  bool ledOn = false;
  while (wm.getConfigPortalActive() && WiFi.status() != WL_CONNECTED) {
    wm.process();

    bool pressed = (digitalRead(BUTTON_PIN) == LOW);
    if (!pressed && resetHoldStart == 0) {
      sawIdleSinceEntry = true;
    }

    if (pressed) {
      if (resetHoldStart == 0) resetHoldStart = millis();
      if (millis() - resetHoldStart >= FACTORY_RESET_HOLD_MS) {
        Serial.println("Button held during portal -- factory reset requested.");
        settingsPrefs.begin("settings", false);
        settingsPrefs.clear();
        settingsPrefs.end();
        WiFi.disconnect(true, true); // also erase the radio's own persisted WiFi credentials
        setLedColor(0, 0, 0);
        Serial.println("Restarting into unconfigured state...");
        Serial.flush();
        delay(200);
        ESP.restart();
      }
    } else if (resetHoldStart != 0) {
      resetHoldStart = 0;
      if (sawIdleSinceEntry) {
        Serial.println("Button pressed -- canceling setup portal early.");
        wm.stopConfigPortal();
        break;
      }
    }

    bool shouldBeOn = (millis() % SETUP_LED_BLINK_PERIOD_MS) < SETUP_LED_PULSE_MS;
    if (shouldBeOn != ledOn) {
      ledOn = shouldBeOn;
      setLedColor(0, 0, ledOn ? 255 : 0); // pulsing blue while the portal is open
    }
    delay(10);
  }
  bool connected = (WiFi.status() == WL_CONNECTED);

  if (!connected) {
    Serial.println("Setup portal timed out / no connection -- resuming with existing settings.");
    setLedColor(0, 0, 0);
    updateLedForRelayState();
    return;
  }

  settings.mqttHost     = p_mqtt_host.getValue();
  int parsedPort        = atoi(p_mqtt_port.getValue());
  settings.mqttPort     = (parsedPort > 0 && parsedPort <= 65535) ? (uint16_t)parsedPort : 1883;
  settings.mqttUser     = p_mqtt_user.getValue();
  settings.mqttPassword = p_mqtt_pass.getValue();
  settings.deviceName   = p_device_name.getValue();
  settings.deviceId     = p_device_id.getValue();
  settings.deviceId.replace(" ", "_"); // MQTT topics can't contain spaces
  settings.wifiSsid     = WiFi.SSID();
  settings.wifiPassword = WiFi.psk();

  // Backstop -- mqtt_host is HTML `required`, so a normal browser won't
  // submit with it blank. If it's somehow empty anyway, don't mark this
  // configured: the next boot goes straight back to the portal on its own.
  if (settings.mqttHost.length() == 0) {
    saveSettings(); // still keep the WiFi/device fields that were filled in
    Serial.println("Setup portal closed with an empty MQTT broker host -- not marking as configured.");
    setLedColor(0, 0, 0);
    updateLedForRelayState();
    return;
  }

  settings.configured = true;
  saveSettings();
  Serial.printf("Setup saved: device_id=%s mqtt=%s:%u\n",
                settings.deviceId.c_str(), settings.mqttHost.c_str(), settings.mqttPort);

  setLedColor(0, 0, 0);
  Serial.println("Restarting into normal operation...");
  Serial.flush();
  delay(200);
  ESP.restart();
}
