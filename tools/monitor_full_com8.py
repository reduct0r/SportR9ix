#!/usr/bin/env python3
"""Full COM8 capture: MAVLink stats + STATUSTEXT timeline + passthrough detection."""
import re
import struct
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM8"
BAUD = 57600
DURATION = int(sys.argv[2]) if len(sys.argv) > 2 else 55


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
    print(f"=== Monitor {PORT} @ {BAUD} for {DURATION}s ===\n", flush=True)
    data = bytearray()
    t0 = time.time()
    while time.time() - t0 < DURATION:
        chunk = ser.read(4096)
        if chunk:
            data.extend(chunk)
    ser.close()

    msgs = parse_v2(bytes(data))
    texts = []
    counts = {}
    last_att = None
    last_pos = None

    for mid, pl in msgs:
        counts[mid] = counts.get(mid, 0) + 1
        if mid == 253 and len(pl) > 1:
            txt = pl[1:51].split(b"\0", 1)[0].decode("ascii", errors="ignore")
            texts.append(txt)
        elif mid == 30 and len(pl) >= 16:
            last_att = struct.unpack_from("<fff", pl, 4)
        elif mid == 33 and len(pl) >= 28:
            lat, lon, alt = struct.unpack_from("<iii", pl, 4)
            last_pos = (lat / 1e7, lon / 1e7, alt / 1000.0)

    names = {
        0: "HEARTBEAT",
        30: "ATTITUDE",
        33: "GLOBAL_POSITION",
        74: "VFR_HUD",
        124: "SYS_STATUS",
        24: "GPS_RAW",
        253: "STATUSTEXT",
        22: "PARAM_VALUE",
    }

    print(f"Bytes: {len(data)}  MAVLink v2 msgs: {len(msgs)}")
    print("\nMessage counts:")
    for mid, c in sorted(counts.items()):
        print(f"  {names.get(mid, f'MSG_{mid}'):20s} {c}")

    status = [t for t in texts if t.startswith("P ") or t.startswith("L ")]
    ids_lines = [t for t in texts if t.startswith("ids")]
    poll_lines = [t for t in texts if t.startswith("poll ")]
    sync = [t for t in texts if t.startswith("sync ")]
    hex_lines = [t for t in texts if t.startswith("hex")]
    skat = [t for t in texts if t.startswith("SKAT")]

    print("\n--- STATUSTEXT timeline (unique types, last of each) ---")
    seen = {}
    for t in texts:
        key = t.split()[0] if t else ""
        seen[key] = t
    for t in texts:
        pass
    for label in ["SKAT", "P", "L", "ids", "poll", "hex"]:
        matches = [t for t in texts if t.startswith(label + " ") or t.startswith(label + ":")]
        if label == "SKAT":
            matches = [t for t in texts if t.startswith("SKAT")]
        if matches:
            print(f"  [{label}] {matches[-1]}")

    print("\n--- All status lines (P/L) ---")
    for t in status[-8:]:
        print(f"  {t}")

    print("\n--- ID histogram lines ---")
    for t in ids_lines[-5:]:
        print(f"  {t}")

    if poll_lines:
        print("\n--- Poll diag ---")
        for t in poll_lines[-3:]:
            print(f"  {t}")
    else:
        print("\n--- Poll diag: none (old firmware or not yet 10s) ---")

    if sync:
        print("\n--- Sliding CRC scan (desync / invert) ---")
        for t in sync[-5:]:
            print(f"  {t}")

    if hex_lines:
        print("\n--- Raw hex samples ---")
        for t in hex_lines[-3:]:
            print(f"  {t}")
    else:
        print("\n--- Raw hex: none (needs new firmware) ---")

    # Parse app IDs from ids lines
    app_ids = {}
    for line in ids_lines:
        for m in re.finditer(r"([0-9A-Fa-f]{4}):(\d+)", line):
            aid = m.group(1).upper()
            app_ids[aid] = app_ids.get(aid, 0) + int(m.group(2))

    print("\n--- Aggregated SmartPort App IDs ---")
    for aid, cnt in sorted(app_ids.items(), key=lambda x: -x[1])[:15]:
        tag = ""
        if aid in ("5001", "5005", "5006", "5002", "5003", "5004", "5008", "0800"):
            tag = " <-- ArduPilot passthrough"
        elif aid.startswith("F1") or aid.startswith("F0"):
            tag = " <-- R9M module"
        print(f"  0x{aid}: {cnt}{tag}")

    has500x = any(aid.startswith("500") for aid in app_ids)
    has5006 = "5006" in app_ids
    pt_vals = []
    for t in status:
        m = re.search(r"pt=(\d+)", t)
        if m:
            pt_vals.append(int(m.group(1)))
    max_pt = max(pt_vals) if pt_vals else 0

    print("\n=== VERDICT ===")
    if last_att:
        r, p, y = last_att
        print(f"ATTITUDE to MP: YES (roll={r:.3f} pitch={p:.3f})")
    else:
        print("ATTITUDE to MP: NO")
    if last_pos and (last_pos[0] != 0 or last_pos[1] != 0):
        print(f"GPS to MP: lat={last_pos[0]:.5f} lon={last_pos[1]:.5f}")
    else:
        print("GPS to MP: no fix / zero")
    print(f"Passthrough 0x500x on bus: {'YES' if has500x else 'NO'}")
    print(f"0x5006 (attitude): {'YES' if has5006 else 'NO'}")
    print(f"Max pt counter: {max_pt}")
    if status:
        print(f"Last bus status: {status[-1]}")
        if "fl=" in status[-1]:
            print("Firmware: NEW (CRC fail counter present)")
        else:
            print("Firmware: likely OLD (no fl= field — flash latest build)")


if __name__ == "__main__":
    main()
