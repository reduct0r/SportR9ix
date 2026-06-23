#!/usr/bin/env python3
"""Monitor ESP32 debug text on USB serial."""
import re
import serial
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM8"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 57600
DURATION = int(sys.argv[3]) if len(sys.argv) > 3 else 30

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    time.sleep(2)
    print(f"Monitoring {PORT} @ {BAUD} for {DURATION}s...", flush=True)

    raw = bytearray()
    start = time.time()
    while time.time() - start < DURATION:
        chunk = ser.read(1024)
        if chunk:
            raw.extend(chunk)

    ser.close()

    text = raw.decode("ascii", errors="ignore")
    printable = "".join(c if c.isprintable() or c in "\r\n" else " " for c in text)
    lines = [ln.strip() for ln in printable.splitlines() if ln.strip()]

    print("\n--- Text lines ---")
    for ln in lines[-30:]:
        print(ln)

    status = re.findall(r"sport=\d+ passthrough=\d+ last=\d+ms", printable)
    print("\n--- Summary ---")
    print(f"Total bytes: {len(raw)}")
    print(f"Status lines: {len(status)}")
    if status:
        print(f"Last status: {status[-1]}")
    print(f"MAVLink v1 starts (0xFE): {raw.count(0xFE)}")
    print(f"MAVLink v2 starts (0xFD): {raw.count(0xFD)}")

    if len(raw) < 50:
        print("WARNING: almost no data from ESP32")

if __name__ == "__main__":
    main()
