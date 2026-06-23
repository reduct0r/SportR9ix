#!/usr/bin/env python3
"""Live MAVLink client: stream alt/climb/gspeed/attitude with variance stats."""
import struct
import sys
import time
from collections import deque

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM8"
BAUD = 57600
DUR = int(sys.argv[2]) if len(sys.argv) > 2 else 60


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
        payload = bytes(buf[i + 10 : i + 10 + plen])
        msgs.append((msg_id, payload))
        i += total
    return msgs


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    time.sleep(1.5)
    print(f"=== Live MAVLink watch {PORT} {DUR}s ===\n", flush=True)

    buf = bytearray()
    t0 = time.time()
    last_print = t0

    alts = deque(maxlen=2000)
    climbs = deque(maxlen=2000)
    gspeeds = deque(maxlen=2000)
    rolls = deque(maxlen=2000)
    vzs = deque(maxlen=2000)
    diag_lines = []

    counts = {}

    while time.time() - t0 < DUR:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)

        while len(buf) >= 12:
            if buf[0] != 0xFD:
                del buf[0]
                continue
            plen = buf[1]
            total = 12 + plen
            if len(buf) < total:
                break
            frame = bytes(buf[:total])
            del buf[:total]
            msg_id = frame[7] | (frame[8] << 8) | (frame[9] << 16)
            payload = frame[10 : 10 + plen]
            counts[msg_id] = counts.get(msg_id, 0) + 1

            if msg_id == 74 and len(payload) >= 20:
                aspd, gspd, hdg, thr, alt, climb = struct.unpack_from("<ffhHff", payload, 0)
                alts.append(alt)
                climbs.append(climb)
                gspeeds.append(gspd)
            elif msg_id == 33 and len(payload) >= 28:
                rel = struct.unpack_from("<i", payload, 16)[0] / 1000.0
                vz = struct.unpack_from("<h", payload, 24)[0] / 100.0
                vzs.append(vz)
            elif msg_id == 30 and len(payload) >= 16:
                r, p, y = struct.unpack_from("<fff", payload, 4)
                rolls.append(r)
            elif msg_id == 253 and len(payload) > 1:
                txt = payload[1:51].split(b"\0", 1)[0].decode("ascii", errors="ignore")
                if txt.startswith(("L ", "P ", "ids")):
                    diag_lines.append(txt)

        now = time.time()
        if now - last_print >= 5.0:
            last_print = now
            elapsed = now - t0
            def stats(name, arr, fmt=".3f"):
                if not arr:
                    return f"{name}: (none)"
                vals = list(arr)
                mn, mx = min(vals), max(vals)
                uniq = len(set(round(v, 2) for v in vals))
                last = vals[-1]
                return (
                    f"{name}: last={last:{fmt}} min={mn:{fmt}} max={mx:{fmt}} "
                    f"span={mx-mn:{fmt}} uniq(0.01)={uniq}"
                )

            print(f"[{elapsed:5.1f}s]", flush=True)
            print(" ", stats("VFR alt", alts, ".2f"), flush=True)
            print(" ", stats("VFR climb", climbs), flush=True)
            print(" ", stats("VFR gspeed", gspeeds), flush=True)
            print(" ", stats("GPOS vz", vzs), flush=True)
            print(" ", stats("ATT roll", rolls), flush=True)
            if diag_lines:
                print(f"  diag: {diag_lines[-1]}", flush=True)
            print(flush=True)

    ser.close()

    print("=== Summary ===")
    for mid, c in sorted(counts.items()):
        names = {0: "HEARTBEAT", 30: "ATTITUDE", 33: "GPOS", 74: "VFR_HUD", 253: "STATUSTEXT"}
        print(f"  {names.get(mid, f'MSG_{mid}'):14s} {c}")

    import re

    app_ids = {}
    for line in diag_lines:
        if not line.startswith("ids"):
            continue
        for m in re.finditer(r"([0-9A-Fa-f]{4}):(\d+)", line):
            app_ids[m.group(1).upper()] = app_ids.get(m.group(1).upper(), 0) + int(m.group(2))

    if app_ids:
        print("\nPassthrough AppIDs (from STATUSTEXT ids lines):")
        for aid, cnt in sorted(app_ids.items(), key=lambda x: -x[1]):
            note = ""
            if aid == "5004":
                note = " <- baro rel alt"
            elif aid == "5005":
                note = " <- climb/gspeed/yaw"
            elif aid == "5006":
                note = " <- attitude"
            print(f"  0x{aid}: {cnt}{note}")
    else:
        print("\nNo ids diag (set MAVLINK_DIAG=1 and reflash)")


if __name__ == "__main__":
    main()
