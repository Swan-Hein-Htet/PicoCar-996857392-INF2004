#!/usr/bin/env python3
"""PicoCar command console: control the car over MQTT and watch everything.

    python3 tools/mqtt/car_console.py [--host 127.0.0.1] [--port 1883]
                                      [--log logs/car_x.log]
                                      [--mode control|verbose]

Normally started by tools/mqtt/start_broker.sh.  Type a command and press
Enter (docs/MQTT.md, "Console"):

    cal      calibrate (same as GP20 on the line; after a run it resets
             the car first and then starts a fresh calibration)
    go       start driving (same as GP20 once calibrated)
    stop     stop at once (critical fault REMOTE_STOP; "reset" clears it)
    reset    clear a fault / back to start-up
    end      backup end-of-course marker
    status   full decoded state of the car
    faults   active faults with explanations
    raw <START|STOP|RESET|END>   send a command byte with no state checks
    map      show the barcode map the car has: which letters A-Z turn
             left, right, go straight or U-turn (the rest unmapped)
    map A=L B=R C=S D=U    set it before the run (parked only); letters
             not listed are unmapped.  "map clear" unmaps all,
             "map default" is A=L B=R C=S D=U
    drive    manual driving: w a s d (hold to keep going), space stops,
             q leaves.  Only while parked (before calibrating, calibrated,
             or finished); w refuses an obstacle within 15 cm
    scan     barcode test build: creep over the barcode, next scan creeps
             back (same as GP20; keep the path clear, no sonar)
    control  show only the important notices (the default)
    verbose  show every line from the car as well
    mode     say which of the two is on
    help, quit

Everything the car publishes is saved in both modes:
    <log>                    console lines, commands, state/fault changes
    <log minus .log>.csv     every telemetry record (5 per second)

Standard library only (no paho-mqtt); MQTT 3.1.1, QoS 0 and 1.
"""

import argparse
import os
import random
import re
import socket
import struct
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import decode_telemetry as tdec  # noqa: E402

try:
    import readline  # noqa: F401  (line editing and history for input())
except ImportError:
    readline = None

TOPIC_TELEM = "p5is20/car/telemetry"
TOPIC_CMD = "p5is20/car/cmd"
TOPIC_STATUS = "p5is20/car/status"
TOPIC_LOG = "p5is20/car/log"
TOPIC_DRIVE = "p5is20/car/drive"
TOPIC_MAP = "p5is20/car/map"
MAP_WORDS = {"L": "turn left at the next junction", "R": "turn right at the next junction",
             "S": "straight on at the next junction", "U": "U-turn"}
DEFAULT_MAP = "LRSU" + "-" * 22

S_INIT, S_CAL, S_READY, S_FINISHED, S_FAULT = 0, 1, 2, 9, 10
S_FOLLOW, S_MANOEUVRE, S_HUMP, S_OBSTACLE, S_SEARCH = 3, 5, 6, 7, 8
MOTION = ["M0 idle", "M1 follow", "M2 forward", "M3 backward", "M4 turn L",
          "M5 turn R", "M6 stopping", "M7 off"]
LINE = ["calibrating", "tracking", "junction", "LOST"]
PLAN = ["continue", "bypass left", "bypass right", "reverse"]
CLASS = ["stationary", "driving", "accelerating", "turning", "climbing",
         "descending", "impact"]
REJECT = ["-", "bar count", "framing", "width ratio", "unknown pattern",
          "overrun"]
FAULT_HELP = {
    "ESTOP_REQ": "GP21 emergency-stop button pressed",
    "MOTOR_STALL": "high motor duty but the wheel is not turning (500 ms)",
    "ENCODER": "a wheel counter froze, ran backwards or is far too slow",
    "IMU": "accelerometer not answering on I2C (warning only)",
    "IMPACT": "acceleration above 3 g -- collision",
    "NO_ECHO": "ultrasonic sensor returned nothing for 5 pings",
    "BYPASS_FAIL": "could not get round an obstacle",
    "WIFI_LOSS": "MQTT link was down (warning only)",
    "LINE_NOT_FOUND": "line search timed out",
    "SELF_TEST": "start-up self-test or IR calibration failed",
    "REMOTE_STOP": "'stop' sent from this console",
    "TASK_OVERRUN": "a control task stopped running (watchdog-level)",
    "LIFTED": "the car was picked up: motors stopped, end of the demo",
}
CRITICAL = 0x1FFF & ~(0x0080 | 0x0008)

# Control mode: the car's log lines worth seeing, as plain notices.  State
# changes, faults and humps come from telemetry instead (it repeats, the
# log does not), so they are not listed here.
NOTICES = (
    ("T6: calibration sweep", "Calibrating: sweeping over the line"),
    ("T5: obstacle at", "Obstacle detected: {rest}"),
    ("T6: pending command", "Barcode command expired: no junction reached"),
    ("T6: scan pass done", "Scan pass done"),
    ("T6: scan pass", "Scanning: {rest}"),
    ("T6: PLAID armed,", "PLAID ARMED: {rest}"),
    ("T6: PLAID off", "Plaid off {rest}"),
    ("T6: stunt", "Stunt {rest}"),
    ("T6: barcode map refused", "Barcode map REFUSED: {rest}"),
)
# Log lines shown as they are in control mode: they say what went wrong.
PROBLEMS = (
    "T1: control task stalled", "T2: ADC not answering",
    "T2: neither line sensor", "T2: ch", "T2: BARCODE TEST build",
    "T2: that pass", "T2: trace saved",
    "T3: IMPACT", "T4: encoder fault", "T4: motor stall",
    "T5: bypass leg blocked", "T5: no echo", "T5: no way round",
    "T6: RESET refused", "T6: manual drive only", "T6: manual drive blocked",
    "T6: manual drive stopped",
)


def now_stamp():
    return time.strftime("%H:%M:%S")


# ---------------------------------------------------------------- MQTT ---

def enc_str(text):
    raw = text.encode()
    return struct.pack("!H", len(raw)) + raw


def enc_len(length):
    out = bytearray()
    while True:
        byte = length % 128
        length //= 128
        out.append(byte | (0x80 if length else 0))
        if not length:
            return bytes(out)


class Mqtt:
    """Minimal MQTT 3.1.1 client: one socket, one reader thread."""

    def __init__(self, host, port, on_publish, on_link):
        self.host, self.port = host, port
        self.on_publish, self.on_link = on_publish, on_link
        self.sock = None
        self.lock = threading.Lock()
        self.pid = random.randint(1, 60000)
        self.up = False

    def _send(self, data):
        with self.lock:
            if self.sock is None:
                raise OSError("not connected")
            self.sock.sendall(data)

    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise OSError("broker closed the connection")
            buf += chunk
        return buf

    def _read_packet(self):
        b0 = self._recv_exact(1)[0]
        mult, length = 1, 0
        while True:
            byte = self._recv_exact(1)[0]
            length += (byte & 0x7F) * mult
            mult *= 128
            if not byte & 0x80:
                break
        return b0, self._recv_exact(length) if length else b""

    def connect(self):
        sock = socket.create_connection((self.host, self.port), timeout=5)
        sock.settimeout(None)
        self.sock = sock
        cid = "p5is20-console-%d" % os.getpid()
        body = enc_str("MQTT") + bytes([4, 0x02]) + struct.pack("!H", 30) \
            + enc_str(cid)
        self._send(bytes([0x10]) + enc_len(len(body)) + body)
        b0, body = self._read_packet()
        if b0 >> 4 != 2 or len(body) < 2 or body[1] != 0:
            raise OSError("broker refused the connection")
        subs = [(TOPIC_LOG, 0), (TOPIC_STATUS, 1), (TOPIC_TELEM, 0)]
        payload = b"".join(enc_str(t) + bytes([q]) for t, q in subs)
        body = struct.pack("!H", self._next_pid()) + payload
        self._send(bytes([0x82]) + enc_len(len(body)) + body)
        self.up = True

    def _next_pid(self):
        self.pid = self.pid % 65535 + 1
        return self.pid

    def publish(self, topic, payload, qos=1):
        body = enc_str(topic)
        if qos:
            body += struct.pack("!H", self._next_pid())
        body += payload
        self._send(bytes([0x30 | (qos << 1)]) + enc_len(len(body)) + body)

    def run(self):
        """Reader loop with automatic reconnect; runs in a thread."""
        while True:
            try:
                if not self.up:
                    self.connect()
                    self.on_link(True, None)
                b0, body = self._read_packet()
                kind = b0 >> 4
                if kind == 3:                       # PUBLISH
                    qos = (b0 >> 1) & 3
                    tlen = struct.unpack("!H", body[:2])[0]
                    topic = body[2:2 + tlen].decode(errors="replace")
                    pos = 2 + tlen
                    if qos:
                        pid = body[pos:pos + 2]
                        pos += 2
                        self._send(bytes([0x40, 2]) + pid)   # PUBACK
                    self.on_publish(topic, body[pos:])
            except OSError as err:
                was_up = self.up
                self.up = False
                with self.lock:
                    if self.sock is not None:
                        try:
                            self.sock.close()
                        except OSError:
                            pass
                        self.sock = None
                if was_up:
                    self.on_link(False, str(err))
                time.sleep(2)

    def keepalive(self):
        while True:
            time.sleep(15)
            if self.up:
                try:
                    self._send(bytes([0xC0, 0]))      # PINGREQ
                except OSError:
                    pass


# ------------------------------------------------------------- console ---

class Console:
    def __init__(self, args):
        self.args = args
        self.rec = None             # latest decoded telemetry
        self.rec_time = 0.0
        self.prev = None
        self.hump_base = None       # hump max at the last calibration
        self.car_status = "unknown"
        self.cmd_seq = random.randint(0, 63)
        self.acks = {}              # seq -> threading.Event
        self.out_lock = threading.Lock()
        self.telem_gap_warned = False
        self.n_telem = 0
        self.n_logs = 0
        self.n_barcodes = 0
        self.last_letter = None
        self.pass_open = False
        self.pass_result = False
        self.pass_peak = 0
        self.pass_dark = 0
        self.pass_trace = []
        self.verbose = args.mode == "verbose"
        self.car_map = None         # 26 chars L R S U -, as the car reported
        self.map_asked = 0.0
        # The map the user set, kept on disk: the car keeps it only in RAM,
        # so a reboot (a brown-out mid-turn, 2026-11-19) silently put the
        # default back and the next barcode read 'unmapped'.  Restored
        # automatically once the car is parked again.
        self.want_map = self.load_map()
        self.map_restore = False
        base = args.log[:-4] if args.log.endswith(".log") else args.log
        os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
        self.logf = open(args.log, "a", buffering=1)
        self.csvf = open(base + ".csv", "a", buffering=1)
        if self.csvf.tell() == 0:
            self.csvf.write("pc_time," + ",".join(f[0] for f in tdec.FIELDS)
                            + "\n")
        self.mqtt = Mqtt(args.host, args.port, self.on_publish, self.on_link)

    # ---- output: keep the half-typed command line intact ----
    def out(self, text, save=True, show=True):
        with self.out_lock:
            if show:
                line_buf = readline.get_line_buffer() if readline else ""
                sys.stdout.write("\r\033[K" + text + "\n")
                if self.prompting:
                    sys.stdout.write("car> " + line_buf)
                sys.stdout.flush()
            if save:
                self.logf.write(text + "\n")

    prompting = False

    def note(self, text, important=True):
        """A console notice.  Saved always, shown in verbose mode, and in
        control mode only if important."""
        self.out("[%s] >> %s" % (now_stamp(), text),
                 show=important or self.verbose)

    def notice(self, text):
        """The control-mode wording of a log line.  The raw line is saved
        and shown in verbose mode, so this is neither."""
        if not self.verbose:
            self.out("[%s] >> %s" % (now_stamp(), text), save=False)

    def control_view(self, text):
        """Pick out what control mode shows from one car log line."""
        body = text.split("] ", 1)[1] if text.startswith("[") and "] " in text \
            else text
        for start, wording in NOTICES:
            if body.startswith(start):
                self.notice(wording.format(rest=body[len(start):].strip()))
                return
        routine = body.startswith("T2: ch") and "never saw" not in body
        if body.startswith(PROBLEMS) and not routine:
            self.notice(body.split(": ", 1)[1])     # "T2: ch0 white ..." is
                                                    # normal calibration output

    # ---- incoming ----
    def on_link(self, b_up, why):
        if b_up:
            self.note("connected to broker %s:%d -- waiting for the car"
                      % (self.args.host, self.args.port))
        else:
            self.note("lost the broker (%s); retrying every 2 s" % why)

    def on_publish(self, topic, payload):
        if topic == TOPIC_LOG:
            text = payload.decode(errors="replace").rstrip()
            self.n_logs += 1
            self.out("car %s" % text, show=self.verbose)
            self.control_view(text)
            self.check_ack(text)
            self.check_map(text)
            self.check_barcode(text)
        elif topic == TOPIC_STATUS:
            status = payload.decode(errors="replace")
            if status != self.car_status:
                self.car_status = status
                self.note("car is %s" % status.upper())
        elif topic == TOPIC_TELEM:
            try:
                rec = tdec.decode(payload)
            except ValueError as err:
                self.note("bad telemetry record: %s" % err)
                return
            self.n_telem += 1
            if self.car_status != "online":
                self.car_status = "online"
            self.csvf.write("%.3f," % time.time() + ",".join(
                str(rec[f[0]]) for f in tdec.FIELDS) + "\n")
            self.rec, self.rec_time = rec, time.time()
            if self.telem_gap_warned:
                self.note("telemetry is back")
                self.telem_gap_warned = False
            self.changes(rec)

    MAP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "barcode_map.txt")

    def load_map(self):
        try:
            with open(self.MAP_FILE) as handle:
                m = handle.read().strip().upper()
            return m if len(m) == 26 and all(c in "LRSU-" for c in m) else None
        except OSError:
            return None

    def save_map(self, m):
        try:
            with open(self.MAP_FILE, "w") as handle:
                handle.write(m + "\n")
        except OSError:
            pass

    def check_map(self, text):
        if "T6: barcode map refused" in text and self.want_map and \
                self.car_map != self.want_map:
            self.map_restore = True         # try again when parked
        match = re.search(r"T6: barcode map ([LRSU-]{26}) \((\d+) mapped(.*)\)", text)
        if not match:
            return
        new = match.group(1)
        if new != self.car_map:
            self.car_map = new
            self.note("barcode map on the car: %s" % self.map_text(new))
        if self.want_map and new != self.want_map and "default" in match.group(3):
            self.map_restore = True
            self.note("the car restarted with the default barcode map -- "
                      "sending yours again (%s) once it is parked"
                      % self.map_text(self.want_map))

    def restore_map(self, rec):
        """Send the user's map back after a reboot, once parked."""
        if self.map_restore and rec["mission_state"] in (S_CAL, S_READY,
                                                         S_FINISHED):
            self.map_restore = False
            if self.want_map and self.want_map != self.car_map:
                self.send_map(self.want_map, remember=False)

    @staticmethod
    def map_text(m):
        parts = ["%s=%s" % (chr(65 + i), c) for i, c in enumerate(m) if c != "-"]
        return " ".join(parts) or "nothing mapped"

    def check_barcode(self, text):
        # Firmware (task_line.c): "T2: barcode 'X'" or
        # "T2: barcode rejected, reason N (why), M elements ..."
        # STATUS lines repeat the last letter: "| barcode 'X' |" ('-' = none)
        idx = text.find("| barcode '")
        if idx >= 0 and len(text) > idx + 11:
            letter = text[idx + 11]
            if letter != "-" and letter != self.last_letter:
                self.last_letter = letter
                self.note("BARCODE '%s'  (from STATUS: its log line was "
                          "lost)" % letter)
            return
        self.check_pass(text)
        idx = text.find("T2: barcode '")
        if idx >= 0 and len(text) > idx + 13:
            letter = text[idx + 13]
            self.last_letter = letter
            self.n_barcodes += 1
            # The firmware appends its verdict: "-> left" ... "-> unmapped".
            verdict = re.search(r"-> (left|right|straight|u-turn|unmapped)", text)
            meaning = {"left": MAP_WORDS["L"], "right": MAP_WORDS["R"],
                       "straight": MAP_WORDS["S"], "u-turn": MAP_WORDS["U"],
                       "unmapped": "unmapped, no command"}.get(
                verdict.group(1) if verdict else "", "no navigation meaning")
            self.note("BARCODE '%s'  (%s)  [#%d]" % (letter, meaning,
                                                   self.n_barcodes))
        elif "T2: barcode rejected" in text:
            why = text.split("reason", 1)[1].strip() if "reason" in text \
                else text
            self.note("barcode NOT read: reason %s" % why)

    def check_pass(self, text):
        """A scan pass that sees too little makes no result line at all:
        say so once its trace is in."""
        if "T6: scan pass" in text and "done" not in text:
            self.pass_open, self.pass_result = True, False
            self.pass_peak, self.pass_dark = 0, 0
            self.pass_trace = []
        elif "T2: barcode" in text:
            self.pass_result = True
        elif "T2: trace saved" in text:
            self.d0_balance()
            if self.pass_open and not self.pass_result:
                self.note("No barcode this pass: too few bars seen. Darkest "
                          "reading %d (a bar reads about 80), dark for %d of "
                          "the trace samples. Is the sensor over the bars?"
                          % (self.pass_peak, self.pass_dark))
            self.pass_open = False
        elif self.pass_open:
            data = re.search(r"T2: trace \d+:(.*)", text)
            if data:
                values = [int(v) for v in data.group(1).split()]
                self.pass_trace += values
                self.pass_peak = max([self.pass_peak] + values)
                self.pass_dark += sum(v >= 20 for v in values)

    def d0_balance(self):
        """Digital (D0) barcode sensor: a thin bar and a thin space are the
        same width on paper, so their run lengths in the trace should match.
        If bars come out much shorter, the board's threshold only calls black
        in the darkest middle of a bar, and thin bars go missing."""
        trace = self.pass_trace
        if len(trace) < 100 or any(0 < v < 90 for v in trace):
            return                          # not a D0 trace, or too short
        runs, start = [], 0
        for idx in range(1, len(trace)):
            if (trace[idx] >= 50) != (trace[idx - 1] >= 50):
                runs.append((trace[idx - 1] >= 50, idx - start))
                start = idx
        runs = runs[1:]                     # the first run is cut off
        bars = sorted(n for black, n in runs if black)
        gaps = sorted(n for black, n in runs if not black)
        if len(bars) < 6 or len(gaps) < 6:
            return
        # thin ones are the lower part of each list
        thin_bar = bars[len(bars) // 4]
        thin_gap = gaps[len(gaps) // 4]
        ratio = thin_bar / max(1, thin_gap)
        if ratio < 0.67:
            advice = ("thin bars read too short: adjust the D0 potentiometer "
                      "so the sensor calls black SOONER (its LED should light "
                      "over thin bars)")
        elif ratio > 1.5:
            advice = ("thin bars read too long: adjust the D0 potentiometer "
                      "so the sensor calls black LATER")
        else:
            advice = "balanced, leave the potentiometer"
        self.note("D0 balance: thin bars %d, thin spaces %d (should be about "
                  "equal) -- %s" % (thin_bar, thin_gap, advice))

    def check_ack(self, text):
        # Firmware: "CMD <NAME> seq <n> received in ..." / "... duplicate"
        idx = text.find("CMD ")
        if idx < 0:
            return
        parts = text[idx:].split()
        if len(parts) >= 4 and parts[2] == "seq":
            try:
                event = self.acks.get(int(parts[3]))
            except ValueError:
                return
            if event:
                event.set()

    def zero_hump(self):
        """Calibration: the hump height reads 0 from here.  The car clears
        its own maximum too (D62); until its telemetry shows that (or with
        older firmware that never does) the value it had is hidden."""
        self.hump_base = self.rec["hump_max_mm"] if self.rec else 0
        if hasattr(self, "hump_last"):
            self.hump_last = None
            self.hump_max = None

    def hump_now(self, rec=None):
        """The run's highest hump, 0 straight after calibrating."""
        rec = rec or self.rec
        if not rec:
            return 0
        value = rec["hump_max_mm"]
        if self.hump_base is not None:
            if value == 0 or value != self.hump_base:
                self.hump_base = None       # cleared, or a new hump
            else:
                return 0
        return value

    def changes(self, rec):
        """Report what changed between two telemetry records."""
        self.restore_map(rec)
        prev, self.prev = self.prev, rec
        if (prev is not None and rec["mission_state"] in (S_INIT, S_CAL)
                and prev["mission_state"] not in (S_INIT, S_CAL)):
            self.zero_hump()
        if prev is None:
            self.note("telemetry: " + self.summary(rec), important=False)
            self.note("car is in %s" % self.state_name(rec))
            self.hint(rec, None)
            if rec["faults"]:
                self.note("active faults: " + self.fault_names(rec["faults"]))
            return
        if rec["mission_state"] != prev["mission_state"]:
            self.note("state %s -> %s" % (self.state_name(prev),
                                          self.state_name(rec)),
                      important=False)
            self.hint(rec, prev["mission_state"])
        gained = rec["faults"] & ~prev["faults"]
        lost = prev["faults"] & ~rec["faults"]
        if gained:
            self.note("FAULT  + %s" % self.fault_names(gained, True))
        if lost:
            self.note("fault cleared: %s" % self.fault_names(lost))
        if rec["line_state"] != prev["line_state"] and rec["line_state"] == 3:
            self.note("line LOST at %d mm" % rec["distance_mm"],
                      important=False)
        if rec["bypass_plan"] != prev["bypass_plan"] and rec["bypass_plan"]:
            self.note("obstacle %d mm wide, closest %d mm -> %s" % (
                rec["obstacle_width_mm"], rec["obstacle_closest_mm"],
                PLAN[rec["bypass_plan"]]))
        if (rec["hump_max_mm"] > prev["hump_max_mm"]
                and self.hump_now(rec) > 0):
            self.note("HUMP measured: %d mm high" % rec["hump_max_mm"])
        if (rec["seq"] - prev["seq"]) % 256 > 3:
            self.note("%d telemetry records missed (weak Wi-Fi?)"
                      % ((rec["seq"] - prev["seq"]) % 256 - 1),
                      important=False)

    @staticmethod
    def state_name(rec):
        state = rec["mission_state"]
        name = tdec.MISSION[state] if state < len(tdec.MISSION) else "?"
        return "S%d %s" % (state, name)

    def hint(self, rec, was):
        """Plain notice for entering a state; was = previous state."""
        state = rec["mission_state"]
        if state == S_FOLLOW:
            back = {S_READY: "Driving", S_FINISHED: "Driving",
                    S_MANOEUVRE: "Junction done, following the line",
                    S_HUMP: "Off the hump",
                    S_OBSTACLE: "Obstacle passed, back on the line",
                    S_SEARCH: "Line found again"}.get(was)
            if back:
                self.note(back)
        elif state == S_MANOEUVRE:
            self.note("Turning at the junction")
        elif state == S_HUMP:
            self.note("On a hump, slowing down")
        elif state == S_OBSTACLE:
            self.note("Stopped for an obstacle, going round it")
        elif state == S_SEARCH:
            self.note("Line lost at %d mm, searching" % rec["distance_mm"])
        elif state == S_CAL:
            self.note("place the car on the line, then type: cal")
        elif state == S_READY:
            self.note("calibrated -- type: go")
        elif state == S_FINISHED:
            self.note("course finished at %d mm -- 'go' runs again, 'cal' "
                      "recalibrates" % rec["distance_mm"])
        elif state == S_FAULT:
            self.note("SAFE STATE: %s -- fix it, then type: reset"
                      % self.fault_names(rec["faults"], True))

    @staticmethod
    def fault_names(mask, explain=False):
        out = []
        for bit, name in enumerate(tdec.FAULTS):
            if mask >> bit & 1:
                kind = "critical" if (1 << bit) & CRITICAL else "warning"
                out.append("%s [%s: %s]" % (name, kind, FAULT_HELP[name])
                           if explain else name)
        return ", ".join(out) or "none"

    @staticmethod
    def summary(rec):
        return "%s | %s | line %s err %+d | speed %d/%d mm/s | %d mm | " \
            "faults 0x%04x" % (
                Console.state_name(rec), MOTION[rec["motion_state"]],
                LINE[rec["line_state"]], rec["line_error"],
                rec["speed_left_mms"], rec["speed_right_mms"],
                rec["distance_mm"], rec["faults"])

    def ask_map(self):
        """Ask the car for its map until it has reported one."""
        if self.car_map is None and self.rec_time and \
                time.time() - self.rec_time < 3 and \
                time.time() - self.map_asked > 3:
            self.map_asked = time.time()
            try:
                self.mqtt.publish(TOPIC_MAP, b"?", qos=1)
            except OSError:
                pass

    def send_map(self, m, remember=True):
        try:
            self.mqtt.publish(TOPIC_MAP, m.encode(), qos=1)
        except OSError as err:
            self.note("could not send the map: %s" % err)
            return False
        if remember:
            self.want_map = m
            self.save_map(m)
        self.note("sent barcode map %s" % self.map_text(m), important=False)
        return True

    @staticmethod
    def parse_map(words):
        """'A=L B=R ...', 'clear', 'default' or a 26-char string -> map."""
        if len(words) == 1 and words[0].lower() == "clear":
            return "-" * 26
        if len(words) == 1 and words[0].lower() == "default":
            return DEFAULT_MAP
        if len(words) == 1 and len(words[0]) == 26:
            m = words[0].upper()
            return m if all(c in "LRSU-" for c in m) else None
        m = ["-"] * 26
        for word in words:
            match = re.fullmatch(r"([A-Za-z])=([LRSUlrsu-])", word)
            if not match:
                return None
            m[ord(match.group(1).upper()) - 65] = match.group(2).upper()
        return "".join(m)

    def do_map(self, words):
        if not words:
            if self.car_map is None:
                self.note("the car has not reported its map yet (is it online?)")
            else:
                self.out("  barcode map: %s" % self.map_text(self.car_map))
            return
        m = self.parse_map(words)
        if m is None:
            self.note("map: use letters like A=L B=R C=S D=U (L left, R right, "
                      "S straight, U U-turn), or 'clear' / 'default'")
            return
        state = self.state()
        if state is not None and state not in (S_INIT, S_CAL, S_READY, S_FINISHED):
            self.note("the map is set before the run; the car is in %s"
                      % self.state_name(self.rec))
        self.send_map(m)

    def watchdog(self):
        while True:
            time.sleep(1)
            self.ask_map()
            if self.rec_time and not self.telem_gap_warned and \
                    time.time() - self.rec_time > 3:
                self.telem_gap_warned = True
                self.note("no telemetry for 3 s -- car off, out of Wi-Fi "
                          "range or rebooting (last: %s)"
                          % self.state_name(self.rec))

    # ---- outgoing ----
    def send(self, name):
        self.cmd_seq = (self.cmd_seq + 1) % 64
        seq = self.cmd_seq
        event = threading.Event()
        self.acks[seq] = event
        try:
            self.mqtt.publish(TOPIC_CMD, tdec.command_byte(name, seq), qos=1)
        except OSError as err:
            self.note("could not send %s: %s" % (name, err))
            return False
        self.note("sent %s (seq %d)" % (name, seq), important=False)
        if event.wait(2.0):
            del self.acks[seq]
            return True
        self.note("no reply to %s in 2 s -- is the car online? (status: %s)"
                  % (name, self.car_status))
        return False

    def wait_state(self, states, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if self.rec and self.rec["mission_state"] in states:
                return True
            time.sleep(0.05)
        return False

    def state(self):
        if self.rec is None or time.time() - self.rec_time > 3:
            return None
        return self.rec["mission_state"]

    def do_cal(self):
        self.zero_hump()
        state = self.state()
        if state is None:
            self.note("no telemetry from the car yet; sending START "
                      "anyway (same as GP20)")
            self.send("START")
        elif state == S_CAL:
            self.send("START")
        elif state == S_INIT:
            self.note("car is starting up; calibrating as soon as it is "
                      "ready")
            if self.wait_state({S_CAL}, 5):
                self.send("START")
            elif self.state() == S_READY:
                self.note("it was already calibrated; resetting first")
                self.do_cal()
        elif state in (S_READY, S_FINISHED):
            self.note("resetting the car to calibrate again")
            if self.send("RESET") and self.wait_state({S_CAL}, 5):
                self.send("START")
            else:
                self.note("car did not return to calibration")
        elif state == S_FAULT:
            self.note("car is in the SAFE STATE; type 'reset' first")
        else:
            self.note("car is driving (%s); type 'stop' first"
                      % self.state_name(self.rec))

    def do_go(self):
        state = self.state()
        if state is None:
            self.note("no telemetry from the car yet; sending START "
                      "anyway (same as GP20)")
            self.send("START")
        elif state in (S_READY, S_FINISHED):
            self.send("START")
        elif state in (S_INIT, S_CAL):
            self.note("not calibrated yet -- type 'cal' first")
        elif state == S_FAULT:
            self.note("car is in the SAFE STATE (%s); type 'reset' first"
                      % self.fault_names(self.rec["faults"]))
        else:
            self.note("already driving (%s)" % self.state_name(self.rec))

    def do_status(self):
        if self.rec is None:
            self.note("no telemetry received yet (car status: %s)"
                      % self.car_status)
            return
        rec = self.rec
        age = time.time() - self.rec_time
        lines = [
            "car %s, last record %.1f s ago, %d records, %d log lines"
            % (self.car_status, age, self.n_telem, self.n_logs),
            "state     %s, motion %s, %s" % (
                self.state_name(rec), MOTION[rec["motion_state"]],
                CLASS[min(rec["motion_class"], 6)]),
            "line      %s, error %+d (+ = line to the left)" % (
                LINE[rec["line_state"]], rec["line_error"]),
            "speed     left %d, right %d mm/s, turn %d deg/s" % (
                rec["speed_left_mms"], rec["speed_right_mms"],
                rec["turn_rate_dps"]),
            "distance  %d mm, encoders %d / %d" % (
                rec["distance_mm"], rec["enc_left"], rec["enc_right"]),
            "barcode   last %s, last reject %s" % (
                tdec.BARCODE[min(rec["last_barcode"], 5)],
                REJECT[min(rec["reject_reason"], 5)]),
            "letter    last read %s, %d read this session" % (
                repr(self.last_letter) if self.last_letter else "none",
                self.n_barcodes),
            "hump      max %d mm" % self.hump_now(rec),
            "obstacle  width %d mm, closest %d mm, bearing %d deg, plan %s,"
            " scan O%d" % (
                rec["obstacle_width_mm"], rec["obstacle_closest_mm"],
                rec["obstacle_bearing_x10deg"] * 10 - 90,
                PLAN[rec["bypass_plan"]], rec["scan_state"]),
            "faults    0x%04x %s" % (rec["faults"],
                                     self.fault_names(rec["faults"], True)),
        ]
        for line in lines:
            self.out("  " + line)

    def do_drive(self):
        """Single-key driving: each w/a/s/d/space press goes to the car at
        once (QoS 0).  Holding a key repeats it, which keeps the car moving;
        the car stops ~0.6 s after the last one."""
        try:
            import termios
            import tty
        except ImportError:
            self.note("drive needs a Unix terminal")
            return
        if not sys.stdin.isatty():
            self.note("drive needs an interactive terminal")
            return
        state = self.state()
        if state not in (None, S_CAL, S_READY, S_FINISHED):
            self.note("the car is in %s: manual driving works only while "
                      "parked (S1, S2, S9)" % self.state_name(self.rec))
        self.note("DRIVE: hold w (forward), s (back), a (left), d (right); "
                  "space = stop; q = leave")
        fd = sys.stdin.fileno()
        saved = termios.tcgetattr(fd)
        last = None
        try:
            tty.setcbreak(fd)
            while True:
                key = sys.stdin.read(1).lower()
                if key in ("q", "\x1b", ""):
                    break
                if key in "wasd ":
                    try:
                        self.mqtt.publish(TOPIC_DRIVE, key.encode(), qos=0)
                    except OSError as err:
                        self.note("could not send: %s" % err)
                        break
                    if key != last:
                        self.logf.write("[%s] drive %r\n" % (now_stamp(), key))
                        last = key
        except KeyboardInterrupt:
            pass
        finally:
            try:
                self.mqtt.publish(TOPIC_DRIVE, b" ", qos=0)
            except OSError:
                pass
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        self.note("left DRIVE (car told to stop)")

    def handle(self, line):
        words = line.strip().split()
        if not words:
            return True
        cmd = words[0].lower()
        self.logf.write("[%s] car> %s\n" % (now_stamp(), line.strip()))
        if cmd in ("cal", "calibrate"):
            self.do_cal()
        elif cmd in ("go", "start", "run"):
            self.do_go()
        elif cmd in ("stop", "s", "x", "estop"):
            self.send("STOP")
        elif cmd == "reset":
            self.send("RESET")
        elif cmd == "end":
            self.send("END")
        elif cmd == "scan":
            self.send("START")
        elif cmd == "drive":
            self.do_drive()
        elif cmd == "map":
            self.do_map(words[1:])
        elif cmd in ("status", "st", "?"):
            self.do_status()
        elif cmd in ("faults", "fault", "f"):
            mask = self.rec["faults"] if self.rec else 0
            self.out("  faults 0x%04x: %s" % (mask,
                                               self.fault_names(mask, True)))
        elif cmd == "raw" and len(words) == 2 and \
                words[1].upper() in tdec.COMMANDS:
            self.send(words[1].upper())
        elif cmd in ("control", "verbose"):
            self.verbose = cmd == "verbose"
            self.note("%s mode (the log file keeps everything either way)"
                      % cmd)
        elif cmd == "mode":
            self.note("%s mode" % ("verbose" if self.verbose else "control"))
        elif cmd in ("quit", "exit", "q"):
            return False
        else:
            self.out(__doc__.split("Enter (docs/MQTT.md, \"Console\"):")[1]
                     .split("Everything")[0].rstrip(), save=False)
        return True

    def run(self):
        for target in (self.mqtt.run, self.mqtt.keepalive, self.watchdog):
            threading.Thread(target=target, daemon=True).start()
        self.note("saving the log to %s" % self.args.log)
        self.note("commands: cal, go, stop, reset, map, status, faults, "
                  "control, verbose, help, quit")
        self.note("%s mode" % ("verbose" if self.verbose else "control"))
        try:
            while True:
                self.prompting = True
                line = input("car> ")
                self.prompting = False
                if not self.handle(line):
                    break
        except (EOFError, KeyboardInterrupt):
            pass
        self.prompting = False
        self.note("console closed")


def main():
    root = os.path.abspath(os.path.join(HERE, "..", ".."))
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--log", default=os.path.join(
        root, "logs", time.strftime("car_%Y%m%d_%H%M%S.log")))
    ap.add_argument("--mode", choices=("control", "verbose"),
                    default="control")
    Console(ap.parse_args()).run()


if __name__ == "__main__":
    main()
