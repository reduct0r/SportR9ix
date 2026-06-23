#!/usr/bin/env python3
"""Listen to ESP32 MAVLink STATUSTEXT diagnostics on COM8 (SmartPort sniffer via bridge)."""
import struct
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM8"
BAUD = 57600
DURATION = int(sys.argv[2]) if len(sys.argv) > 2 else 45


def parse_v2(buf):
    msgs = []
    i = 0
    while i < len(buf):
        if buf[i] != 0xFD:
            i += 1
            continue
        if i + 10 > len(buf):
            break
        plen = buf[i + 1]
        total = 12 + plen
        if i + total > len(buf):
            break
        msg_id = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        payload = buf[i + 10 : i + 10 + plen]
        msgs.append((msg_id, payload))
        i += total
    return msgs


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    time.sleep(2)
    print(f"Listening {PORT} @ {BAUD} for {DURATION}s (disconnect Mission Planner first)", flush=True)
    data = bytearray()
    t0 = time.time()
    while time.time() - t0 < DURATION:
        chunk = ser.read(4096)
        if chunk:
            data.extend(chunk)
    ser.close()

    texts = []
    for mid, pl in parse_v2(bytes(data)):
        if mid == 253 and len(pl) > 1:
            txt = pl[1:51].split(b"\0", 1)[0].decode("ascii", errors="ignore")
            texts.append(txt)

    print(f"\nSTATUSTEXT messages: {len(texts)}", flush=True)
    for t in texts:
        print(f"  {t}")

    scans = [t for t in texts if t.startswith("scan ") or "scan start" in t]
    ids = [t for t in texts if t.startswith("ids")]
    status = [t for t in texts if t.startswith("L ") or t.startswith("P ")]

    print("\n--- Summary ---", flush=True)
    if status:
        print(f"Last status: {status[-1]}")
    if ids:
        print("App ID histogram lines:")
        for t in ids[-3:]:
            print(f"  {t}")
    if scans:
        print("Poll scan lines:")
        for t in scans:
            print(f"  {t}")
    else:
        print("No poll scan yet (wait >=3s after boot)")

    has500x = any("500" in t for t in texts)
    print(f"Passthrough 0x500x seen: {'YES' if has500x else 'NO'}", flush=True)


if __name__ == "__main__":
    main()
