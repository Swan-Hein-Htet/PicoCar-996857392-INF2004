#!/usr/bin/env python3
"""Decode P5-IS20 PicoCar telemetry (docs/MQTT.md), version 1, 21 bytes.

Usage:
    decode_telemetry.py <hex>                 decode one record
    decode_telemetry.py --selftest <hex>      check the host-test reference
    decode_telemetry.py --mqtt <broker-ip>    live (needs paho-mqtt)
    decode_telemetry.py --cmd <broker-ip> <START|STOP|RESET|END> <seq>

The layout is written out independently of the C code on purpose: the
self-test proves the firmware packer and this decoder agree field by field.
"""

import sys

VERSION = 2
RECORD_BYTES = 24       # v2 (2026-11-19); v1 records are 21 bytes
RECORD_BYTES_V1 = 21

# (name, bit offset, width, signed)
FIELDS = [
    ("version", 0, 4, False),
    ("seq", 4, 8, False),
    ("mission_state", 12, 4, False),
    ("motion_state", 16, 3, False),
    ("speed_left_mms", 19, 10, True),
    ("speed_right_mms", 29, 10, True),
    ("enc_left", 39, 16, False),
    ("enc_right", 55, 16, False),
    ("line_state", 71, 2, False),
    ("line_error", 73, 8, True),
    ("last_barcode", 81, 3, False),
    ("reject_reason", 84, 3, False),
    ("hump_max_mm", 87, 7, False),
    ("obstacle_width_mm", 94, 10, False),
    ("obstacle_closest_mm", 104, 9, False),
    ("obstacle_bearing_x10deg", 113, 5, False),
    ("bypass_plan", 118, 2, False),
    ("distance_mm", 120, 15, False),
    ("motion_class", 135, 3, False),
    ("turn_rate_dps", 138, 10, True),
    ("faults", 148, 16, False),
    ("scan_state", 164, 4, False),
    # v2: tilt and hump meters
    ("pitch_x10", 168, 10, True),       # 0.1 deg, + = nose up
    ("roll_deg", 178, 8, True),
    ("hump_now_mm", 186, 6, False),     # height above flat right now
]
V2_FIELDS = ("pitch_x10", "roll_deg", "hump_now_mm")

MISSION = ["INIT", "CALIBRATE", "READY", "FOLLOW", "BARCODE", "MANOEUVRE",
           "HUMP", "OBSTACLE", "SEARCH", "FINISHED", "FAULT"]
# last_barcode: the last barcode's COMMAND from the A-Z map (2026-11-17);
# before the map it was the letter A-D, which mapped to the same commands.
BARCODE = ["none", "left", "right", "straight", "u-turn", "unmapped"]
FAULTS = ["ESTOP_REQ", "MOTOR_STALL", "ENCODER", "IMU", "IMPACT", "NO_ECHO",
          "BYPASS_FAIL", "WIFI_LOSS", "LINE_NOT_FOUND", "SELF_TEST",
          "REMOTE_STOP", "TASK_OVERRUN", "LIFTED"]
COMMANDS = {"START": 0, "STOP": 1, "RESET": 2, "END": 3}


def get_bits(data, offset, width):
    value = 0
    for bit in range(width):
        pos = offset + bit
        if (data[pos // 8] >> (pos % 8)) & 1:
            value |= 1 << bit
    return value


def decode(data):
    """Version 2 (24 bytes), or version 1 (21 bytes, older firmware and
    logs) with the v2 fields read as 0."""
    version = get_bits(data, 0, 4) if data else 0
    if (version, len(data)) not in ((2, RECORD_BYTES), (1, RECORD_BYTES_V1)):
        raise ValueError("unknown telemetry version %d / %d bytes"
                         % (version, len(data)))
    out = {}
    for name, offset, width, signed in FIELDS:
        if version == 1 and name in V2_FIELDS:
            out[name] = 0
            continue
        value = get_bits(data, offset, width)
        if signed and value >= (1 << (width - 1)):
            value -= 1 << width
        out[name] = value
    return out


def pretty(rec):
    names = [FAULTS[b] for b in range(len(FAULTS)) if rec["faults"] >> b & 1]
    state = rec["mission_state"]
    return ("seq %3d  S%-2d %-9s  v=(%4d,%4d) mm/s  dist %5d mm  line %d err %4d"
            "  barcode %-5s  hump %3d mm  obst w%d c%d  faults %s" % (
                rec["seq"], state, MISSION[state] if state < len(MISSION)
                else "?", rec["speed_left_mms"], rec["speed_right_mms"],
                rec["distance_mm"], rec["line_state"], rec["line_error"],
                BARCODE[rec["last_barcode"]] if rec["last_barcode"] < 6
                else "?", rec["hump_max_mm"], rec["obstacle_width_mm"],
                rec["obstacle_closest_mm"], ",".join(names) or "-"))


def command_byte(name, seq):
    return bytes([(COMMANDS[name] & 0x3) | ((seq & 0x3F) << 2)])


def selftest(hexstr):
    rec = decode(bytes.fromhex(hexstr))
    expect = {
        "version": 2, "seq": 0x5A, "mission_state": 7, "motion_state": 3,
        "speed_left_mms": -250, "speed_right_mms": 249, "enc_left": 0x1234,
        "enc_right": 0xFEDC, "line_state": 1, "line_error": -64,
        "last_barcode": 4, "reject_reason": 3, "hump_max_mm": 42,
        "obstacle_width_mm": 101, "obstacle_closest_mm": 187,
        "obstacle_bearing_x10deg": 9, "bypass_plan": 1, "distance_mm": 12345,
        "motion_class": 3, "turn_rate_dps": -321, "faults": 0x00A0,
        "scan_state": 7, "pitch_x10": -123, "roll_deg": -7,
        "hump_now_mm": 23,
    }
    bad = [k for k, v in expect.items() if rec[k] != v]
    for key in bad:
        print("[PY] FAIL %s: got %r want %r" % (key, rec[key], expect[key]))
    try:
        decode(bytes([0x03]) + bytes(RECORD_BYTES - 1))
        bad.append("version check")
        print("[PY] FAIL unknown version accepted")
    except ValueError:
        pass
    print("[PY] decoder self-test %s (%d fields)" %
          ("PASS" if not bad else "FAIL", len(expect)))
    print("[PY] " + pretty(rec))
    return 0 if not bad else 1


def live(broker):
    import paho.mqtt.client as mqtt  # optional dependency

    def on_message(_client, _userdata, msg):
        if msg.topic.endswith("telemetry"):
            try:
                print(pretty(decode(msg.payload)))
            except ValueError as err:
                print("rejected:", err)
        else:
            print(msg.topic, msg.payload)

    client = mqtt.Client()
    client.on_message = on_message
    client.connect(broker, 1883, 60)
    client.subscribe([("p5is20/car/telemetry", 0), ("p5is20/car/status", 1)])
    client.loop_forever()


def send(broker, name, seq):
    import paho.mqtt.publish as publish  # optional dependency
    publish.single("p5is20/car/cmd", command_byte(name, int(seq)), qos=1,
                   hostname=broker)


def main(argv):
    if len(argv) >= 3 and argv[1] == "--selftest":
        return selftest(argv[2])
    if len(argv) >= 3 and argv[1] == "--mqtt":
        live(argv[2])
        return 0
    if len(argv) >= 5 and argv[1] == "--cmd":
        send(argv[2], argv[3].upper(), argv[4])
        return 0
    if len(argv) == 2:
        print(pretty(decode(bytes.fromhex(argv[1]))))
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
