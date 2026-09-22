# smart_switch — ESP32-C3 mains relay switch

Raw Arduino C++ replacement for the ESPHome sketch: `espMqttClient` with
QoS 1 + PUBACK, non-blocking WiFi/MQTT reconnect with exponential backoff,
LWT availability, retained HA MQTT discovery, diagnostic entities,
NVS-persisted status-LED brightness, `ArduinoOTA`, manufacturer `P@cho`.
Mains-powered, so there's no deep sleep — it stays connected continuously,
unlike the battery-powered sensor projects.

As of v2.0.0, WiFi, MQTT, and device identity are configured at runtime via
a built-in `WiFiManager` web setup portal and persisted in NVS — same
system as `door_sensor`/`TH_2_v4` — instead of being compiled into
`secrets.h`. Since this device already has a spare button and LED, the
portal is reached with a physical button-hold gesture (see "Setup Mode"
below), the same convention `TH_2_v4` uses, rather than an MQTT switch.

## Files

- `smart_switch.ino` — the sketch.
- `secrets.h.example` — copy to `secrets.h` and fill in: OTA password,
  setup-portal AP password/timeout, device identity/firmware version,
  button-hold thresholds. WiFi/MQTT credentials are *not* here any more —
  see "Setup Mode". Keep `secrets.h` out of git (already covered by
  `.gitignore`).

## Hardware

ESP32-C3, replacing an ESPHome-based relay switch:

- **GPIO1** — relay control, active-high
- **GPIO5** — physical button, active-low, internal pull-up — short press
  toggles the relay; held ≥10s opens the setup portal; held ≥5s again
  while the portal is open triggers a factory reset (see "Setup Mode")
- **GPIO10** — single WS2812 status LED (red = relay on, green = relay off
  + WiFi connected, off = no WiFi, pulsing blue = setup hold/portal active)

## Before building

1. **Arduino IDE board package**: "esp32 by Espressif Systems" (Boards
   Manager).
2. **Libraries** (Library Manager):
   - `espMqttClient` by bertmelis
   - `Adafruit NeoPixel`
   - `WiFiManager` by tzapu
3. Select board **"ESP32C3 Dev Module"**.
4. Copy `secrets.h.example` to `secrets.h` and fill in your OTA password
   and (optionally) the setup-portal AP password. WiFi/MQTT are set up
   after flashing, through the portal — see "Setup Mode" below.
5. Plain DHCP — no static IP support in this project.

## First flash vs. later updates

First flash needs a USB cable. A never-configured device boots straight
into the setup portal (see below). After that, `ArduinoOTA` exposes the
board as a network port in `Tools > Port` — subsequent updates can go out
over WiFi, protected by `OTA_PASSWORD` from `secrets.h`.

## Setup Mode

On first boot (or after a factory reset), the device broadcasts its own
temporary WiFi network — `<manufacturer> <model> XXXX` (last 4 hex chars
of the chip MAC), password from `AP_PASSWORD` in `secrets.h` — and serves
a setup page: the normal WiFiManager network picker, plus MQTT broker
host/port/user/password and device name/ID fields on the same page, saved
together in one submission. The portal times out after `PORTAL_TIMEOUT_SEC`
(default 10 min) and resumes with whatever settings already existed.

To reopen the portal on an already-configured device (e.g. to move it to a
new WiFi network or MQTT broker), **hold the button for 10 seconds**
(`BUTTON_SETUP_HOLD_MS`) — it fires the instant the hold crosses that
threshold, without needing to release first, and does *not* also toggle
the relay. The LED pulses blue for the duration of the hold and while the
portal is open.

While the portal is open, **holding the button again for 5 seconds**
(`FACTORY_RESET_HOLD_MS`) wipes all saved settings — WiFi, MQTT, device
identity — and the radio's own saved WiFi credentials, then restarts fully
unconfigured. A quick press instead cancels the portal early.

A successful portal save always ends in a restart, so the relay will
briefly return to its default-off state during that reboot — same as any
other restart of this device.

**Note for existing units upgrading to v2.0.0**: this is a breaking
change. The device will boot straight into the setup portal on first run
of the new firmware, since saved WiFi credentials move from the old
compiled-in `secrets.h` values to the portal's own NVS-backed store. Use
the same device ID (`smart_switch`) during setup to keep the existing
Home Assistant entities instead of creating new ones.

## MQTT / Home Assistant

Base topic: `switch/<device_id>/...` (device ID is set during Setup Mode,
defaults to `switch_<chip-id>` if never configured).

| Purpose | Topic | Payload |
|---|---|---|
| Relay command | `.../relay/set` | `ON` / `OFF` |
| Relay state (retained) | `.../relay/state` | `ON` / `OFF` |
| Button (momentary) | `.../button/state` | `ON` / `OFF` |
| Availability / LWT | `.../availability` | `online` / `offline` |
| LED brightness | `.../led_brightness/state`, `.../set` | 0-100 |
| WiFi signal | `.../wifi_signal/state` | dBm |
| Reset reason | `.../reset_reason/state` | string |
| Boot count | `.../boot_count/state` | integer |
| Connect fail count (resets on success) | `.../connect_fail_count/state` | integer |
| Total fail count (lifetime) | `.../total_fail_count/state` | integer |
| Firmware version | `.../firmware_version/state` | string |
| OTA request/restart | `.../ota_restart/set` | any payload |

On every MQTT connect the firmware publishes retained HA discovery configs
for the relay switch, the button (binary sensor), the LED-brightness number
entity, an OTA-restart button, and diagnostic sensors (WiFi signal, reset
reason, boot count, connect/total fail counts, firmware version) — all
bundled under one device in Home Assistant automatically, no
`configuration.yaml` edits needed.

LED brightness persists in NVS (Preferences namespace `smart_switch`), so
it survives a reboot.

## Status LED

| Color | Meaning |
|---|---|
| 3 blue blinks | Boot animation, once MQTT first connects |
| Red | Relay on |
| Green | Relay off, WiFi connected |
| Off | No WiFi |
| Pulsing blue | Setup-mode button hold in progress, or portal open |

## Diagnostics

WiFi signal, boot count, and connect/total fail counts republish every 2
minutes. Reset reason is sent once per boot (or on every MQTT reconnect via
`publishDiagnostics(true)` in `onMqttConnect`). Connect fail count resets
to 0 on the next successful MQTT connect; total fail count is
NVS-persisted and never resets — both increment together on every MQTT
disconnect. Boot count is also NVS-persisted, incremented once per actual
device boot (not on every WiFi/MQTT reconnect). MQTT reconnects use
exponential backoff (1s doubling to a 30s cap); WiFi has a single bounded
5s wait at boot only, then non-blocking retry via `pollWiFi()` every
`loop()` iteration.

## Config file

`secrets.h` (gitignored) holds the OTA password, setup-portal AP
password/timeout, device identity constants (`DEVICE_MANUFACTURER`,
`DEVICE_MODEL`, `DEVICE_HW_VERSION`, `FIRMWARE_VERSION`), and button-hold
thresholds — copy `secrets.h.example` to `secrets.h` and fill in real
values. WiFi credentials, MQTT broker settings, and this device's own
name/ID are **not** here — they're runtime settings, configured through
the setup portal (see "Setup Mode") and persisted in NVS.

## Version History

| Version | Date | Changes |
|---|---|---|
| v1.1 | 2026-09-03 | Initial release. |
| v2.0.0 | 2026-09-22 | Ported the `door_sensor`/`TH_2_v4` runtime WiFiManager setup portal: WiFi/MQTT credentials and device identity moved out of `secrets.h` into NVS-persisted settings, configured through a web portal reached via a 10s button hold (5s in-portal hold for factory reset), same button-gesture convention as `TH_2_v4`. Split `mqtt_fail_count` into `connect_fail_count` (resets on next successful connect) and `total_fail_count` (lifetime, NVS-persisted); added `boot_count` and `firmware_version` diagnostic sensors. Breaking change — existing units need re-provisioning through the portal on first boot after this update. |
