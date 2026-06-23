#!/usr/bin/env python3
"""Simulate Mission Planner param request on COM8 and log MAVLink exchange."""
import struct
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM8"
BAUD = 57600


def crc_accumulate(data, crc):
    tmp = data ^ (crc & 0xFF)
    tmp ^= (tmp << 4) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def mavlink_crc(data, crc_extra):
    crc = 0xFFFF
    for b in data:
        crc = crc_accumulate(b, crc)
    crc = crc_accumulate(crc_extra, crc)
    return crc


def build_v2(msg_id, payload, sysid=255, compid=190, seq=0):
    header = bytes([
        0xFD, len(payload), 0, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF,
        msg_id & 0xFF, (msg_id >> 8) & 0xFF, (msg_id >> 16) & 0xFF,
    ])
    crc_extra = {
        21: 159,  # PARAM_REQUEST_LIST
        22: 220,  # PARAM_VALUE
        0: 50,
        253: 83,
    }.get(msg_id, 0)
    crc = mavlink_crc(header[1:] + payload, crc_extra)
    return header + payload + struct.pack("<H", crc)


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
    print(f"Open {PORT}, reading heartbeat...", flush=True)

    data = bytearray()
    for _ in range(20):
        chunk = ser.read(512)
        if chunk:
            data.extend(chunk)
        time.sleep(0.1)

    msgs = parse_v2(bytes(data))
    print(f"Before request: {len(msgs)} MAVLink msgs: {[m[0] for m in msgs[:10]]}")

    req = build_v2(21, bytes([1, 1]), seq=1)  # PARAM_REQUEST_LIST to sys1 comp1
    print("Sending PARAM_REQUEST_LIST...", flush=True)
    ser.write(req)

    data = bytearray()
    t0 = time.time()
    while time.time() - t0 < 5:
        chunk = ser.read(512)
        if chunk:
            data.extend(chunk)
        time.sleep(0.05)

    ser.close()
    msgs = parse_v2(bytes(data))
    names = {0: "HEARTBEAT", 22: "PARAM_VALUE", 30: "ATTITUDE", 253: "STATUSTEXT"}
    print(f"After request: {len(msgs)} msgs in 5s")
    for mid, payload in msgs:
        name = names.get(mid, f"ID_{mid}")
        extra = ""
        if mid == 22 and len(payload) >= 8:
            pid = payload[8:24].split(b"\0", 1)[0].decode("ascii", errors="ignore")
            extra = f" param={pid!r} count={struct.unpack_from('<H', payload, 4)[0]}"
        if mid == 253 and len(payload) > 1:
            extra = " text=" + payload[1:51].split(b"\0", 1)[0].decode("ascii", errors="ignore")
        print(f"  {name}{extra}")

    got_param = any(m[0] == 22 for m in msgs)
    print("RESULT:", "OK param response" if got_param else "FAIL no PARAM_VALUE")


if __name__ == "__main__":
    main()
