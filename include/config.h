#pragma once

// FrSky SmartPort on the ground side (R9M module -> ESP32)
// Inversion is done in software by ESP32 UART (no extra hardware on the ground).
// Aircraft side: keep your existing inverter between Pixhawk and receiver.
//
// Pin on a typical ESP32 DevKit (30-pin):
//   S.Port+ -> RX2  (this is GPIO16, already used below)
//   GND     -> GND
//
// Avoid GPIO12 (pin D12) — it must stay LOW during reset.
#define SPORT_PIN 16
#define SPORT_BAUD 57600
#define SPORT_UART_INVERT true
// USB debug text corrupts Mission Planner on the same serial port — keep off.
#define DEBUG_USB 0

// MAVLink output to Mission Planner
#define MAVLINK_BAUD 57600
#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID 1

// WiFi UDP telemetry (Mission Planner: UDP, port 14550)
#define WIFI_ENABLED 1
#define WIFI_AP_SSID "SKAT-TELEM"
#define WIFI_AP_PASSWORD "skat12345"
#define WIFI_AP_IP_OCTETS 192, 168, 4, 1
#define WIFI_AP_BCAST_OCTETS 192, 168, 4, 255
#define WIFI_UDP_PORT 14550
// 1 = always UDP-broadcast MAVLink to :14550 (MP listen mode). Required for WiFi.
#define WIFI_UDP_BROADCAST 1

// 0 = listen-only (ER9X handset polls the bus; ESP taps rear S.Port).
// 1 = ESP polls R9M (bench / no handset master).
#define SPORT_ACTIVE_POLL 0
// OpenTX rotates 28 wire IDs every ~12 ms (same table as sportPollSerialPorts).
#define SPORT_OPENTX_ROTATE_POLL 1
#define SPORT_POLL_ID_COUNT 28
#define SPORT_AUTO_POLL_FALLBACK 0
#define SPORT_POLL_INTERVAL_MS 12
#define SPORT_POLL_RX_WINDOW_MS 8

// 0 = OpenTX CRC (sum bytes 1..N == 0xFF). 1 = accept all 8/9-byte frames.
#define SPORT_SKIP_CRC 0

// 0 = clean MAVLink for Mission Planner. 1 = STATUSTEXT debug (hex/sync/ids).
#define MAVLINK_DIAG 0

// Periodic raw-byte hex dump (only when MAVLINK_DIAG=1).
#define SPORT_RAW_DIAG_INTERVAL_MS 10000

// Default ArduPilot downlink physical ID 27 -> sensor 0x1B
#define SPORT_DEFAULT_SENSOR_ID 0x1B

// Periodic poll scan conflicts with handset master — keep off on ground side.
#define SPORT_PERIODIC_SCAN 0
#define SPORT_SCAN_INTERVAL_MS 20000
#define SPORT_SCAN_DURATION_MS 3000
#define SPORT_SCAN_STEP_MS 15
#define SPORT_SCAN_BOOT_DELAY_MS 3000

// Fallback pack capacity (mAh) if 0x5007 from FC not seen yet. 0 = wait for FC only.
#define BATT_CAPACITY_MAH 0
