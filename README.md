# smart_switch — ESP32-C3 mains relay switch

Raw Arduino C++ replacement for the ESPHome sketch: `espMqttClient` with
QoS 1 + PUBACK, non-blocking WiFi/MQTT reconnect with exponential backoff,
LWT availability, retained HA MQTT discovery, WiFi-signal / MQTT-fail-count
diagnostics, NVS-persisted status-LED brightness, `ArduinoOTA`, manufacturer
`P@cho`. Mains-powered, so there's no deep sleep — it stays connected
continuously, unlike the battery-powered sensor projects.

## Files

- `smart_switch.ino` — the sketch.
- `secrets.h.example` — copy to `secrets.h` and fill in: WiFi SSID/password,
  MQTT host/port/user/password, OTA password. Keep `secrets.h` out of git
  (already covered by `.gitignore`).

## Hardware

ESP32-C3, replacing an ESPHome-based relay switch:

- **GPIO1** — relay control, active-high
- **GPIO5** — physical button, active-low, internal pull-up — toggles the
  relay locally and publishes a momentary button state
- **GPIO10** — single WS2812 status LED (red = relay on, green = relay off
  + WiFi connected, off = no WiFi)

## Before building

1. **Arduino IDE board package**: "esp32 by Espressif Systems" (Boards
   Manager).
2. **Libraries** (Library Manager):
   - `espMqttClient` by bertmelis
   - `Adafruit NeoPixel`
3. Select board **"ESP32C3 Dev Module"**.
4. Copy `secrets.h.example` to `secrets.h` and fill in your real WiFi/MQTT/
   OTA values.
5. Plain DHCP — no static IP in firmware. For a stable address, set a DHCP
   reservation for this device's MAC on your router.

## First flash vs. later updates

First flash needs a USB cable. After that, `ArduinoOTA` exposes the board
as a network port in `Tools > Port` — subsequent updates can go out over
WiFi, protected by `OTA_PASSWORD` from `secrets.h`.

## MQTT / Home Assistant

Base topic: `switch/smart_switch/...`

| Purpose | Topic | Payload |
|---|---|---|
| Relay command | `switch/smart_switch/relay/set` | `ON` / `OFF` |
| Relay state (retained) | `switch/smart_switch/relay/state` | `ON` / `OFF` |
| Button (momentary) | `switch/smart_switch/button/state` | `ON` / `OFF` |
| Availability / LWT | `switch/smart_switch/availability` | `online` / `offline` |
| LED brightness | `switch/smart_switch/led_brightness/state`, `.../set` | 0-100 |
| WiFi signal | `switch/smart_switch/wifi_signal/state` | dBm |
| Reset reason | `switch/smart_switch/reset_reason/state` | string |
| MQTT fail count | `switch/smart_switch/mqtt_fail_count/state` | integer |
| OTA restart | `switch/smart_switch/ota_restart/set` | any payload |

On every MQTT connect the firmware publishes retained HA discovery configs
for the relay switch, the button (binary sensor), the LED-brightness number
entity, an OTA-restart button, and three diagnostic sensors (WiFi signal,
reset reason, MQTT fail count) — all bundled under one device in Home
Assistant automatically, no `configuration.yaml` edits needed.

LED brightness persists in NVS (Preferences namespace `smart_switch`), so
it survives a reboot.

## Status LED

| Color | Meaning |
|---|---|
| 3 blue blinks | Boot animation, once MQTT first connects |
| Red | Relay on |
| Green | Relay off, WiFi connected |
| Off | No WiFi |

## Diagnostics

WiFi signal and MQTT fail count republish every 2 minutes. Reset reason is
sent once per boot (or on every MQTT reconnect via `publishDiagnostics(true)`
in `onMqttConnect`). MQTT reconnects use exponential backoff (1s doubling to
a 30s cap); WiFi has a single bounded 5s wait at boot only, then non-blocking
retry via `pollWiFi()` every `loop()` iteration.

## Config file

Credentials live in `secrets.h` (gitignored) — copy `secrets.h.example` to
`secrets.h` and fill in `WIFI_SSID`, `WIFI_PASSWORD`, `MQTT_HOST`,
`MQTT_PORT`, `MQTT_USER`, `MQTT_PASSWORD`, `OTA_PASSWORD`. Device identity
(`DEVICE_NAME`, `DEVICE_FRIENDLY`, `FW_VERSION`) and hardware pins are
constants directly in `smart_switch.ino`, not in `secrets.h` — edit those in
the sketch itself if you're flashing a second unit under a different name.

## Version History

`FW_VERSION` is a constant directly in `smart_switch.ino` (not `secrets.h`),
so it's git-tracked.

| Version | Date | Changes |
|---|---|---|
| v1.1 | 2026-09-03 | Initial release. |
