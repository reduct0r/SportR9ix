# SKAT SmartPort → MAVLink Telemetry Bridge

**Russian documentation:** [README.md](README.md)

Firmware for **ESP32**: a bridge between **ArduPilot FrSky Passthrough** (SmartPort on the ground FrSky module) and **MAVLink** for **Mission Planner**. Designed for in-flight monitoring over 900 MHz R9 RF without a direct USB connection to the flight controller.

---

## Table of Contents

1. [Purpose](#purpose)
2. [Compatible Hardware](#compatible-hardware)
3. [Capabilities and Limitations](#capabilities-and-limitations)
4. [Architecture](#architecture)
5. [Wiring](#wiring)
6. [ArduPilot Configuration](#ardupilot-configuration)
7. [Installation and Build](#installation-and-build)
8. [Flashing the ESP32](#flashing-the-esp32)
9. [Mission Planner Connection](#mission-planner-connection)
10. [`config.h` Reference](#configh-reference)
11. [Passthrough Decoding and MAVLink Output](#passthrough-decoding-and-mavlink-output)
12. [Firmware Structure](#firmware-structure)
13. [Customization](#customization)
14. [Test Tools](#test-tools)
15. [Troubleshooting](#troubleshooting)

---

## Purpose

On the aircraft, ArduPilot sends telemetry to the FrSky receiver using **Passthrough** (`SERIALx_PROTOCOL = 10`). The handset (ER9X / OpenTX / EdgeTX) polls the **R9M** module SmartPort bus; aircraft replies travel over RF and appear on the module rear **S.Port** connector.

The ESP32 **passively listens** to that bus (or actively polls the module on the bench), **decodes** `0x5000`–`0x50FF` packets, **builds MAVLink 2**, and streams to Mission Planner over **USB** or **WiFi UDP**.

This is a **telemetry display bridge**, not a full GCS link to the autopilot.

---

## Compatible Hardware

### Flight Controller

| Component | Requirements |
|-----------|--------------|
| Autopilot | **ArduPilot** (Plane/Copter/Rover) with FrSky Passthrough support |
| Receiver port | `SERIALx_PROTOCOL = 10`, `SERIALx_BAUD = 57` (57600) |
| Aircraft wiring | UART inverter between FC and receiver (standard FrSky setup) |

### Receiver and Transmitter (RF)

| Component | Status |
|-----------|--------|
| FrSky **R9 Slim+** / **R9 Mini** (aircraft) | Verified in SKAT project |
| FrSky **R9M** / **R9M Lite** (ground module, rear S.Port) | Verified |
| Other SmartPort passthrough receivers | Expected compatible with `FRSKY_DNLINK_ID = 27` (0x1B) |

### Handset (SmartPort Bus Master)

| Firmware | `config.h` mode |
|----------|-----------------|
| **ER9X** (Turnigy 9X, etc.) | `SPORT_ACTIVE_POLL 0` — listen-only, handset polls the bus |
| **OpenTX / EdgeTX** | `SPORT_ACTIVE_POLL 0` or `1` (bench without handset) |
| No handset (bench) | `SPORT_ACTIVE_POLL 1` — ESP polls R9M |

### Microcontroller

| Parameter | Value |
|-----------|-------|
| Chip | **ESP32** (tested: ESP32-D0WD-V3, 30-pin DevKit) |
| Framework | Arduino (PlatformIO `espressif32`) |
| Flash / RAM | ~744 KB flash, ~46 KB RAM (typical build) |
| USB-UART | CH9102 / CP2102 / similar, **57600** for MAVLink |

**Do not use GPIO12** (D12) — must be LOW at reset.

### Ground Control Station

| Software | Connection |
|----------|------------|
| **Mission Planner** | USB COM @ 57600 or UDP @ 14550 |
| Other GCS | Not tested; MAVLink UDP may work |

---

## Capabilities and Limitations

### Supported

- Attitude (roll, pitch, yaw)
- GPS (coordinates, fix, HDOP, AMSL altitude when fix available)
- Home distance, bearing, relative altitude
- Vertical and horizontal speed (passthrough quantization applies)
- Battery: voltage, current, mAh, remaining %
- Flight mode, armed state, failsafe flags (display)
- FC STATUSTEXT (0x5000 chunks)
- Mission waypoint index (MISSION_CURRENT)
- Simultaneous output: **USB + WiFi**

### Not Supported

- Mission upload, parameter changes, arm/disarm through this link
- Full USB-MAVLink rate and precision (passthrough quantizes e.g. pitch 0.2°, altitude 0.1 m)
- Camera/gimbal control, MAVLink commands to FC
- Connecting MP directly to the **flight controller COM port** — MP must connect to **ESP**

---

## Architecture

```
ArduPilot (SERIALx_PROTOCOL=10)
    → [inverter] → FrSky receiver (aircraft)
        → RF 900 MHz → R9M (handset module)
            → S.Port+ ── GPIO16 (RX2) ESP32
            → GND ──── GND ESP32
                → [SportParser] → [PassthroughDecoder] → [MavlinkSender]
                    → USB Serial 57600  ─┐
                    → WiFi UDP 14550   ─┴→ Mission Planner
```

**Two UART inversion domains (do not confuse):**

| Segment | Inversion |
|---------|-----------|
| FC → receiver (aircraft) | Hardware inverter |
| R9M → ESP32 (ground) | Software (`SPORT_UART_INVERT`) |

---

## Wiring

### R9M → ESP32

| R9M (rear connector) | ESP32 DevKit |
|----------------------|--------------|
| **GND** | **GND** |
| **S.Port+** | **GPIO16** (RX2, `SPORT_PIN`) |
| S.Port− | not connected |
| +5V | not connected (power ESP from USB) |

SmartPort: **57600 baud**, single-wire, **inverted UART**.

---

## ArduPilot Configuration

On the serial port wired to the FrSky receiver:

```text
SERIALx_PROTOCOL = 10    # FrSky Passthrough
SERIALx_BAUD = 57        # 57600
```

Ensure downlink ID matches polling (default in firmware: physical ID **27** → wire **0x1B**).

Handset telemetry should show aircraft passthrough data when RF link is up.

---

## Installation and Build

### Requirements

- [PlatformIO](https://platformio.org/) (CLI or VS Code / Cursor)
- Python 3 + `pyserial` (for `tools/` scripts)
- USB-UART driver for ESP32

### Clone and Build

```bash
git clone https://github.com/YOUR_USER/SKAT-SPort-MAVLink-Telemetry.git
cd SKAT-SPort-MAVLink-Telemetry
pio run
```

Set upload port in `platformio.ini` (uncomment `upload_port`) or via CLI:

```bash
pio run -t upload --upload-port COM8
```

---

## Flashing the ESP32

```bash
pio run -t upload
```

After boot, Mission Planner Messages should show:

- `SKAT telem bridge ready`
- `SKAT WiFi: SKAT-TELEM UDP 14550` (if `WIFI_ENABLED 1`)

If no aircraft passthrough (>15 s, bus active):

- `SKAT: no FC passthrough - aircraft+RF?`

---

## Mission Planner Connection

### USB (recommended for debugging)

1. Connect ESP32 via USB.
2. Mission Planner → **Connect** → ESP COM port, **57600**.
3. Do **not** connect USB to the flight controller at the same time.

### WiFi UDP

1. ESP AP: **`SKAT-TELEM`** / password **`skat12345`**, IP **`192.168.4.1`**.
2. Close all Mission Planner windows (free port **14550**).
3. PC → WiFi **`SKAT-TELEM`**, network type **Private**.
4. Allow `MissionPlanner.exe` through firewall (UDP 14550, private network).
5. MP → **UDP** → port **14550** → Remote host **empty** (Listen) or **`192.168.4.1`** → **Connect**.
6. Do not use USB COM and UDP to the same bridge simultaneously.

**Port in use:** `taskkill /IM MissionPlanner.exe /F`, restart MP.

**Socket closed:** Disconnect → close MP → reconnect; verify WiFi and `ping 192.168.4.1`.

---

## `config.h` Reference

All settings are in [`include/config.h`](include/config.h).

### SmartPort (input)

| Macro | Default | Description |
|-------|---------|-------------|
| `SPORT_PIN` | `16` | SmartPort RX GPIO (RX2) |
| `SPORT_BAUD` | `57600` | SmartPort baud rate |
| `SPORT_UART_INVERT` | `true` | Software UART inversion (R9M → ESP) |
| `SPORT_ACTIVE_POLL` | `0` | `0` = listen-only (ER9X master), `1` = ESP polls R9M |
| `SPORT_OPENTX_ROTATE_POLL` | `1` | Rotate 28 wire IDs (OpenTX style) in active poll |
| `SPORT_POLL_ID_COUNT` | `28` | Poll slot count |
| `SPORT_POLL_INTERVAL_MS` | `12` | Poll period (active poll) |
| `SPORT_POLL_RX_WINDOW_MS` | `8` | RX window after poll |
| `SPORT_DEFAULT_SENSOR_ID` | `0x1B` | Sensor ID without rotation (ID 27) |
| `SPORT_SKIP_CRC` | `0` | `1` = accept frames without CRC (debug) |
| `SPORT_AUTO_POLL_FALLBACK` | `0` | Auto switch to poll on silence (conflicts with handset) |
| `SPORT_PERIODIC_SCAN` | `0` | Periodic ID scan (conflicts with handset) |
| `SPORT_SCAN_*` | see file | Scan parameters |
| `DEBUG_USB` | `0` | `1` = debug text on USB (breaks MP on same port) |

### MAVLink (output)

| Macro | Default | Description |
|-------|---------|-------------|
| `MAVLINK_BAUD` | `57600` | USB serial baud rate |
| `MAVLINK_SYSTEM_ID` | `1` | MAVLink system ID |
| `MAVLINK_COMPONENT_ID` | `1` | MAVLink component ID |
| `MAVLINK_DIAG` | `0` | `1` = STATUSTEXT bus diagnostics (ids, hex, sync) |
| `SPORT_RAW_DIAG_INTERVAL_MS` | `10000` | Hex dump interval (only if `MAVLINK_DIAG=1`) |

### WiFi

| Macro | Default | Description |
|-------|---------|-------------|
| `WIFI_ENABLED` | `1` | Access point + UDP MAVLink |
| `WIFI_AP_SSID` | `SKAT-TELEM` | Network name |
| `WIFI_AP_PASSWORD` | `skat12345` | WPA2 password |
| `WIFI_AP_IP_OCTETS` | `192,168,4,1` | ESP AP IP address |
| `WIFI_AP_BCAST_OCTETS` | `192,168,4,255` | Subnet broadcast for UDP |
| `WIFI_UDP_PORT` | `14550` | MAVLink UDP port (Mission Planner) |
| `WIFI_UDP_BROADCAST` | `1` | Broadcast fallback when client unknown |

### Battery

| Macro | Default | Description |
|-------|---------|-------------|
| `BATT_CAPACITY_MAH` | `0` | Fallback pack mAh; `0` = wait for 0x5007 from FC |

---

## Passthrough Decoding and MAVLink Output

### ArduPilot App IDs (input)

| App ID | Content |
|--------|---------|
| `0x5000` | STATUSTEXT chunks |
| `0x5001` | Mode, armed, failsafe, throttle, IMU temp |
| `0x5002` | GPS fix, sats, HDOP, AMSL alt |
| `0x5003` / `0x5008` | Battery 1 / 2 |
| `0x5004` | Home distance, bearing, relative altitude |
| `0x5005` | Vario, yaw, ground/airspeed |
| `0x5006` | Roll, pitch, rangefinder |
| `0x5007` | Parameters (battery capacity, etc.) |
| `0x5009` / `0x500D` | Waypoint |
| `0x500A`–`0x500C` | RPM, terrain, wind (decoded; not all mapped to MP) |
| `0x0800` | GPS lat/lon |

### MAVLink messages (output)

| MAVLink | Source / MP usage |
|---------|-------------------|
| `HEARTBEAT` | Vehicle type, armed bit |
| `ATTITUDE` | Roll, pitch, yaw |
| `GLOBAL_POSITION_INT` | GPS, relative alt, vz |
| `GPS_RAW_INT` | When GPS valid |
| `VFR_HUD` | Airspeed, groundspeed, heading, throttle, alt, climb |
| `SYS_STATUS` | Voltage, current, remaining % |
| `BATTERY_STATUS` | Battery details for MP |
| `MISSION_CURRENT` | Waypoint number |
| `STATUSTEXT` | FC + bridge status messages |
| `PARAM_VALUE` | Stub `SKAT_TELEM` (MP parameter request response) |

---

## Firmware Structure

```text
include/config.h          — all configuration
src/main.cpp              — main loop: SmartPort → decode → MAVLink
src/SportParser.cpp       — frame RX (sequential + sliding window)
src/PassthroughDecoder.cpp — App ID decode → TelemetryState
src/MavlinkSender.cpp     — MAVLink 2, USB + WiFi UDP
src/SportDiag.cpp         — App ID diagnostics (MAVLINK_DIAG)
tools/                    — Python verification scripts
platformio.ini            — ESP32 build
```

**Listen mode** (`SPORT_ACTIVE_POLL 0`): sliding-window CRC extracts frames from a stream containing handset poll bytes `0x7E`.

**Active poll** (`SPORT_ACTIVE_POLL 1`): classic sequential parser; ESP sends polls to R9M.

---

## Customization

1. Fork the repo and edit [`include/config.h`](include/config.h) for your wiring and handset mode.
2. Different GPIO: change `SPORT_PIN` (avoid GPIO12).
3. Different WiFi: `WIFI_AP_SSID`, `WIFI_AP_PASSWORD`, `WIFI_UDP_PORT` if needed.
4. New passthrough sensor: add a case in `PassthroughDecoder::handlePacket`, a field in `TelemetryState`, and output in `MavlinkSender::update`.
5. Bus debug: set `MAVLINK_DIAG 1`, rebuild; read STATUSTEXT in MP or run `tools/monitor_full_com8.py`.
6. Build: `pio run`; flash: `pio run -t upload`.

Decoding follows [ArduPilot AP_Frsky_SPort_Passthrough](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp).

---

## Test Tools

Scripts in [`tools/`](tools/); dependency: `pip install pyserial`.

| Script | Purpose | Example |
|--------|---------|---------|
| [`verify_mp_telemetry.py`](tools/verify_mp_telemetry.py) | Verify MP fields (ATTITUDE, VFR_HUD, BATTERY) | `python tools/verify_mp_telemetry.py COM8 40` |
| [`live_mavlink_watch.py`](tools/live_mavlink_watch.py) | Live alt/climb/roll monitor + stats | `python tools/live_mavlink_watch.py COM8 60` |
| [`monitor_full_com8.py`](tools/monitor_full_com8.py) | Full capture + App ID histogram (`MAVLINK_DIAG=1`) | `python tools/monitor_full_com8.py COM8 55` |
| [`analyze_com8_mavlink.py`](tools/analyze_com8_mavlink.py) | MAVLink message counters | `python tools/analyze_com8_mavlink.py COM8` |
| [`sport_listen_com8.py`](tools/sport_listen_com8.py) | Raw stream listen | `python tools/sport_listen_com8.py COM8` |
| [`sniff_com7.py`](tools/sniff_com7.py) | Compare direct FC MAVLink over USB | `python tools/sniff_com7.py COM7` |
| [`test_mp_params.py`](tools/test_mp_params.py) | PARAM_REQUEST response check | `python tools/test_mp_params.py COM8` |
| [`monitor_esp.py`](tools/monitor_esp.py) | USB monitor when `DEBUG_USB=1` | `python tools/monitor_esp.py COM8` |

**Typical post-flash check:**

```bash
python tools/verify_mp_telemetry.py COM8 45
```

Expect `VERDICT: PASS` with aircraft powered and RF linked. `ATTITUDE` and `VFR_HUD` counts should increase; `pt=0` in diag means no FC passthrough.

---

## Troubleshooting

| Symptom | Cause | Action |
|---------|-------|--------|
| MP shows only HEARTBEAT | No passthrough on bus | Power aircraft + RF; check `SERIALx_PROTOCOL=10` |
| Only `F101`/`F104`, no `0x500x` | R9M module telemetry only | No aircraft link or FC not sending passthrough |
| Alt/climb = 0 on bench | Passthrough quantization | Normal; USB to FC is finer |
| Pitch jitter on WiFi | Duplicate UDP (fixed) | Update firmware; single UDP target per packet |
| MP: port 14550 in use | Stale MP process | Close all MP windows / `taskkill` |
| No `SKAT-TELEM` WiFi | ESP not flashed / no power | Reflash, check USB power |

---

## License

Provided as-is for educational and hobby use. ArduPilot and FrSky are trademarks of their respective owners.

---

**SKAT** — SmartPort MAVLink telemetry bridge for long-range FrSky R9 systems.
