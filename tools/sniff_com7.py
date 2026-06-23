#!/usr/bin/env python3
"""Quick MAVLink sniffer for COM7."""
import serial
import time
import sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM7"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 57600

MSG_NAMES = {
    0: "HEARTBEAT", 1: "SYS_STATUS", 24: "GPS_RAW_INT", 30: "ATTITUDE",
    33: "GLOBAL_POSITION_INT", 74: "VFR_HUD", 147: "BATTERY_STATUS",
    65: "RC_CHANNELS", 253: "STATUSTEXT",
}

def main():
    ser = serial.Serial(PORT, BAUD, timeout=3)
    time.sleep(0.5)
    data = ser.read(4096)
    ser.close()
    print(f"Port {PORT} @ {BAUD}: {len(data)} bytes")
    if not data:
        return
    print("First 64 bytes:", data[:64].hex())
    counts = {}
    i = 0
    while i < len(data):
        if data[i] == 0xFE and i + 6 < len(data):
            plen = data[i + 1]
            msgid = data[i + 5]
            counts[msgid] = counts.get(msgid, 0) + 1
            i += plen + 8
        elif data[i] == 0xFD and i + 10 < len(data):
            plen = data[i + 1] | (data[i + 2] << 8)
            msgid = data[i + 7] | (data[i + 8] << 8) | (data[i + 9] << 16)
            counts[msgid & 0xFF] = counts.get(msgid & 0xFF, 0) + 1
            i += plen + 12
        else:
            i += 1
    print("Message counts:")
    for mid, cnt in sorted(counts.items()):
        name = MSG_NAMES.get(mid, f"ID_{mid}")
        print(f"  {name} ({mid}): {cnt}")

if __name__ == "__main__":
    main()
