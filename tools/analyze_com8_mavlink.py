#!/usr/bin/env python3
"""Analyze MAVLink stream from ESP32 bridge on COM8."""
import struct
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM8"
BAUD = 57600
DURATION = 15


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
    time.sleep(1)
    print(f"Reading MAVLink from {PORT} for {DURATION}s...", flush=True)
    data = bytearray()
    t0 = time.time()
    while time.time() - t0 < DURATION:
        chunk = ser.read(2048)
        if chunk:
            data.extend(chunk)
    ser.close()

    msgs = parse_v2(bytes(data))
    counts = {}
    last_att = None
    last_vfr = None
    last_stat = []
    for mid, pl in msgs:
        counts[mid] = counts.get(mid, 0) + 1
        if mid == 30 and len(pl) >= 16:
            roll, pitch, yaw = struct.unpack_from("<fff", pl, 4)
            last_att = (roll, pitch, yaw)
        elif mid == 74 and len(pl) >= 8:
            gs, climb = struct.unpack_from("<ff", pl, 0)
            last_vfr = (gs, climb)
        elif mid == 253 and len(pl) > 1:
            txt = pl[1:51].split(b"\0", 1)[0].decode("ascii", errors="ignore")
            last_stat.append(txt)

    names = {0: "HEARTBEAT", 30: "ATTITUDE", 33: "GLOBAL_POSITION", 74: "VFR_HUD", 253: "STATUSTEXT", 22: "PARAM_VALUE"}
    print(f"Total bytes: {len(data)}, messages: {len(msgs)}")
    print("Counts:")
    for mid, c in sorted(counts.items()):
        print(f"  {names.get(mid, f'ID_{mid}')}: {c}")
    if last_att:
        r, p, y = last_att
        print(f"Last ATTITUDE: roll={r:.3f} rad pitch={p:.3f} rad yaw={y:.3f} rad")
    else:
        print("Last ATTITUDE: none")
    if last_vfr:
        print(f"Last VFR_HUD: gs={last_vfr[0]:.2f} m/s climb={last_vfr[1]:.2f}")
    if last_stat:
        print("STATUSTEXT samples:")
        for t in last_stat[-5:]:
            print(f"  {t}")


if __name__ == "__main__":
    main()
