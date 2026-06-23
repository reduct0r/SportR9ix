#!/usr/bin/env python3
"""Verify SKAT bridge MAVLink fields used by Mission Planner."""
import struct
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM8"
BAUD = 57600
DUR = int(sys.argv[2]) if len(sys.argv) > 2 else 40

NAMES = {
    0: "HEARTBEAT",
    1: "SYS_STATUS",
    24: "GPS_RAW",
    30: "ATTITUDE",
    33: "GLOBAL_POSITION",
    42: "MISSION_CURRENT",
    74: "VFR_HUD",
    147: "BATTERY_STATUS",
    253: "STATUSTEXT",
}


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
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    time.sleep(2)
    print(f"=== MP telemetry verify {PORT} {DUR}s ===\n")
    data = bytearray()
    t0 = time.time()
    while time.time() - t0 < DUR:
        chunk = ser.read(4096)
        if chunk:
            data.extend(chunk)
    ser.close()

    msgs = parse_v2(bytes(data))
    counts = {}
    last = {}

    climbs = []
    alts = []
    fc_texts = []

    for mid, pl in msgs:
        counts[mid] = counts.get(mid, 0) + 1
        if mid == 74 and len(pl) >= 20:
            aspd, gspd, hdg, thr, alt, climb = struct.unpack_from("<ffhHff", pl, 0)
            last["vfr"] = (aspd, gspd, hdg, thr, alt, climb)
            climbs.append(climb)
            alts.append(alt)
        elif mid == 33 and len(pl) >= 28:
            rel = struct.unpack_from("<i", pl, 16)[0] / 1000.0
            vz = struct.unpack_from("<h", pl, 24)[0] / 100.0
            last["gpos"] = (rel, vz)
        elif mid == 30 and len(pl) >= 16:
            r, p, y = struct.unpack_from("<fff", pl, 4)
            last["att"] = (r, p, y)
        elif mid == 147 and len(pl) >= 36:
            v = struct.unpack_from("<H", pl, 10)[0] / 1000.0
            rem = struct.unpack_from("<b", pl, 35)[0]
            last["batt"] = (v, rem)
        elif mid == 1 and len(pl) >= 19:
            rem = struct.unpack_from("<b", pl, 18)[0]
            v = struct.unpack_from("<H", pl, 14)[0] / 1000.0
            last["sys"] = (v, rem)
        elif mid == 42 and len(pl) >= 2:
            last["wp"] = struct.unpack_from("<H", pl, 0)[0]
        elif mid == 253 and len(pl) > 1:
            txt = pl[1:51].split(b"\0", 1)[0].decode("ascii", errors="ignore")
            if txt and not txt.startswith("SKAT"):
                fc_texts.append(txt)

    print("Message counts:")
    for mid, c in sorted(counts.items()):
        print(f"  {NAMES.get(mid, f'MSG_{mid}'):18s} {c}")

    print("\n--- Last values (MP fields) ---")
    if "att" in last:
        r, p, y = last["att"]
        print(f"ATTITUDE roll={r:.3f} pitch={p:.3f} yaw={y:.3f}")
    if "gpos" in last:
        rel, vz = last["gpos"]
        print(f"GLOBAL_POSITION relative_alt(alt)={rel:.2f}m  vz={vz:.2f}m/s")
    if "vfr" in last:
        aspd, gspd, hdg, thr, alt, climb = last["vfr"]
        print(f"VFR_HUD airspeed={aspd:.2f} gspeed={gspd:.2f} alt={alt:.2f} climb(climbrate)={climb:.3f}")
    if "batt" in last:
        print(f"BATTERY_STATUS voltage={last['batt'][0]:.2f}V remaining={last['batt'][1]}%")
    if "sys" in last:
        print(f"SYS_STATUS voltage={last['sys'][0]:.2f}V remaining={last['sys'][1]}%")
    if "wp" in last:
        print(f"MISSION_CURRENT seq={last['wp']}")

    if climbs:
        cmin, cmax = min(climbs), max(climbs)
        uniq = len(set(round(c, 2) for c in climbs))
        print(f"\nClimb samples: n={len(climbs)} min={cmin:.3f} max={cmax:.3f} unique(0.01)={uniq}")
        if uniq <= 1 and abs(cmin) < 0.05:
            print("  NOTE: climb flat — passthrough quantizes to 0.1m/s; USB FC shows finer baro noise")
    if alts:
        amin, amax = min(alts), max(alts)
        print(f"Alt samples: min={amin:.2f} max={amax:.2f} span={amax-amin:.2f}m")

    if fc_texts:
        print(f"\nFC STATUSTEXT ({len(fc_texts)}): {fc_texts[-1][:60]}")

    ok = counts.get(30, 0) > 10 and counts.get(74, 0) > 5
    print(f"\nVERDICT: {'PASS' if ok else 'FAIL'} (need ATTITUDE+VFR_HUD stream)")


if __name__ == "__main__":
    main()
