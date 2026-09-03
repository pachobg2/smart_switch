/*
 * smart_switch — ESP32-C3 mains relay switch
 * Raw Arduino/C++, espMqttClient, MQTT QoS 1, HA auto-discovery
 *
 * Mirrors the conventions used in the SHTC3 temp/humidity sensor and the
 * door sensor projects: espMqttClient, retained discovery configs, LWT
 * availability, diagnostic entities (reset reason, consecutive MQTT
 * connect-fail count). Unlike those, this device is mains-powered, so
 * there is no deep sleep — it stays connected continuously.
 *
 * Hardware (from the ESPHome sketch this replaces):
 *   GPIO1  - relay control (active high)
 *   GPIO5  - physical button, active low, internal pullup
 *   GPIO10 - single WS2812 status LED
 *
 * Libraries needed (Library Manager / PlatformIO):
 *   espMqttClient   (bertmelis/espMqttClient)
 *   Adafruit NeoPixel
 */

#include <WiFi.h>
#include <espMqttClient.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoOTA.h>
#include <esp_system.h>
#include <Preferences.h>
#include "secrets.h" // WIFI_SSID, WIFI_PASSWORD, MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASSWORD, OTA_PASSWORD — not checked into version control

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
static const char* DEVICE_NAME       = "smart_switch";
static const char* DEVICE_FRIENDLY   = "Smart switch";
static const char* FW_VERSION        = "1.1";

// Default LED brightness as a percentage (0-100), used until a value is
// loaded from NVS or set via MQTT/HA.
static const uint8_t DEFAULT_LED_BRIGHTNESS_PCT = 50;

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------
static const uint8_t RELAY_PIN  = 1;
static const uint8_t BUTTON_PIN = 5;
static const uint8_t LED_PIN    = 10;
static const uint8_t LED_COUNT  = 1;

// ---------------------------------------------------------------------------
// MQTT topics
// ---------------------------------------------------------------------------
String baseTopic         = String("switch/") + DEVICE_NAME;
String relayStateTopic   = baseTopic + "/relay/state";
String relayCommandTopic = baseTopic + "/relay/set";
String availabilityTopic = baseTopic + "/availability";
String buttonStateTopic  = baseTopic + "/button/state";
String wifiSignalTopic   = baseTopic + "/wifi_signal/state";
String resetReasonTopic  = baseTopic + "/reset_reason/state";
String failCountTopic    = baseTopic + "/mqtt_fail_count/state";
String ledBrightnessStateTopic   = baseTopic + "/led_brightness/state";
String ledBrightnessCommandTopic = baseTopic + "/led_brightness/set";
String otaRestartCommandTopic    = baseTopic + "/ota_restart/set";

String discoverySwitchTopic       = String("homeassistant/switch/") + DEVICE_NAME + "/relay/config";
String discoveryButtonTopic       = String("homeassistant/binary_sensor/") + DEVICE_NAME + "/button/config";
String discoveryWifiSignalTopic   = String("homeassistant/sensor/") + DEVICE_NAME + "/wifi_signal/config";
String discoveryResetReasonTopic  = String("homeassistant/sensor/") + DEVICE_NAME + "/reset_reason/config";
String discoveryFailCountTopic    = String("homeassistant/sensor/") + DEVICE_NAME + "/mqtt_fail_count/config";
String discoveryLedBrightnessTopic = String("homeassistant/number/") + DEVICE_NAME + "/led_brightness/config";
String discoveryOtaRestartTopic    = String("homeassistant/button/") + DEVICE_NAME + "/ota_restart/config";

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
espMqttClient mqttClient;
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
Preferences prefs;

bool relayState = false;
uint8_t ledBrightnessPct = DEFAULT_LED_BRIGHTNESS_PCT; // 0-100, persisted in NVS

// button debounce
bool lastButtonReading = HIGH;
bool buttonStable = HIGH;
unsigned long lastButtonChangeMs = 0;
static const unsigned long DEBOUNCE_MS = 20;

// mqtt reconnect / diagnostics
unsigned long lastMqttAttemptMs = 0;
unsigned long mqttBackoffMs = 1000;
static const unsigned long MQTT_BACKOFF_MAX_MS = 30000;
uint32_t mqttFailCount = 0;
bool everConnected = false;

unsigned long lastWifiSignalPublishMs = 0;
static const unsigned long WIFI_SIGNAL_INTERVAL_MS = 120000; // 2 min, matches your wifi_signal sensor

bool bootAnimationDone = false;

// wifi reconnect state (non-blocking)
bool wifiConnectInProgress = false;
unsigned long wifiConnectStartMs = 0;
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000;

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

  prefs.begin(DEVICE_NAME, false);
  ledBrightnessPct = prefs.getUChar("led_bright", DEFAULT_LED_BRIGHTNESS_PCT);
  if (ledBrightnessPct > 100) ledBrightnessPct = DEFAULT_LED_BRIGHTNESS_PCT;

  led.begin();
  led.setBrightness(map(ledBrightnessPct, 0, 100, 0, 255));
  led.clear();
  led.show();

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

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);
  mqttClient.setClientId(DEVICE_NAME);
  mqttClient.setWill(availabilityTopic.c_str(), 1, true, "offline");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  connectMqtt();

  ArduinoOTA.setHostname(DEVICE_NAME);
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
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
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
    mqttFailCount++;
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
      "\"identifiers\":[\"" + DEVICE_NAME + "\"]," +
      "\"name\":\"" + DEVICE_FRIENDLY + "\"," +
      "\"manufacturer\":\"P@cho\"," +
      "\"model\":\"ESP32-C3 Power Switch\"," +
      "\"sw_version\":\"" + FW_VERSION + "\"" +
      "}";

  // Switch (relay)
  {
    String payload = String("{") +
        "\"name\":\"Relay\"," +
        "\"unique_id\":\"" + DEVICE_NAME + "_relay\"," +
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
        "\"unique_id\":\"" + DEVICE_NAME + "_button\"," +
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
        "\"unique_id\":\"" + DEVICE_NAME + "_wifi_signal\"," +
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
        "\"unique_id\":\"" + DEVICE_NAME + "_reset_reason\"," +
        "\"state_topic\":\"" + resetReasonTopic + "\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryResetReasonTopic, 1, true, payload);
  }

  // LED brightness (number entity, global brightness control)
  {
    String payload = String("{") +
        "\"name\":\"LED Brightness\"," +
        "\"unique_id\":\"" + DEVICE_NAME + "_led_brightness\"," +
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
        "\"unique_id\":\"" + DEVICE_NAME + "_ota_restart\"," +
        "\"command_topic\":\"" + otaRestartCommandTopic + "\"," +
        "\"payload_press\":\"PRESS\"," +
        "\"device_class\":\"restart\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryOtaRestartTopic, 1, true, payload);
  }

  // MQTT consecutive fail count (diagnostic)
  {
    String payload = String("{") +
        "\"name\":\"MQTT Fail Count\"," +
        "\"unique_id\":\"" + DEVICE_NAME + "_mqtt_fail_count\"," +
        "\"state_topic\":\"" + failCountTopic + "\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"state_class\":\"total_increasing\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryFailCountTopic, 1, true, payload);
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
      setRelay(!relayState, true);
    }
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

  checkedPublish(failCountTopic, 1, true, String(mqttFailCount));
}
