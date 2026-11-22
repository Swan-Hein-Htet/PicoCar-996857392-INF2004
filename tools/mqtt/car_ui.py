#!/usr/bin/env python3
"""PicoCar console in the browser: the same commands, notices and logs as
car_console.py, as a local web page.

    python3 tools/mqtt/car_ui.py [--host 127.0.0.1] [--port 1883]
                                 [--ui-port 8080] [--log logs/car_x.log]

Normally started by tools/mqtt/start_ui.sh, which also starts the MQTT
broker and opens the page.  The page is served on 127.0.0.1 only.

Everything car_console.py saves is saved here too (the .log and the
telemetry .csv): this reuses its Console class and only changes where
the output goes.  Standard library only.
"""

import argparse
import collections
import json
import os
import queue
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import car_console as cc  # noqa: E402

PAGE = os.path.join(HERE, "car_ui.html")
MANUAL = os.path.join(HERE, "manual.html")
METERS = os.path.join(HERE, "meters.js")
THEME = os.path.join(HERE, "..", "ui", "theme.css")
DRIVE_KEYS = "wasd .!jrbtpfo"   # drive, stop, plaid off/on, stunts


class Hub:
    """Fan-out of console events to every open page (Server-Sent Events).
    Keeps the last lines so a reloaded page shows the history."""

    def __init__(self):
        self.lock = threading.Lock()
        self.clients = []
        self.backlog = collections.deque(maxlen=1500)
        self.state = None

    def push(self, event, keep=True):
        data = json.dumps(event)
        with self.lock:
            if keep:
                self.backlog.append(data)
            else:
                self.state = data
            for q in self.clients:
                q.put(data)

    def join(self):
        q = queue.Queue()
        with self.lock:
            for data in self.backlog:
                q.put(data)
            if self.state:
                q.put(self.state)
            self.clients.append(q)
        return q

    def leave(self, q):
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)


class UiConsole(cc.Console):
    """car_console.Console with its output sent to the page.  The page
    filters control / verbose itself, so every line carries its kind:
      note    console notice (imp = shown in control mode)
      notice  control-mode wording of a car log line
      car     raw car log line (verbose mode)
      out     reply to a command (always shown)"""

    def __init__(self, args, hub):
        self.hub = hub
        self.file_lock = threading.Lock()
        self.last_drive = None
        self.plaid = False          # as the car last confirmed it
        self.stunt = None           # name of the stunt running, or None
        self.sonar = None           # mm ahead from the STATUS line
        self.sonar_ts = 0.0
        self.hump_last = None       # last hump height, mm, from the car's log
        self.hump_max = None
        super().__init__(args)

    def on_publish(self, topic, payload):
        super().on_publish(topic, payload)
        if topic != cc.TOPIC_LOG:
            return
        text = payload.decode(errors="replace")
        if "T6: PLAID armed" in text:
            self.plaid = True
        elif "T6: PLAID off" in text:
            self.plaid = False
            self.stunt = None
        elif "T6: stunt " in text:
            what = text.split("T6: stunt ", 1)[1].strip()
            ends = (" done", " aborted", " stopped", "refused", "need")
            if what.endswith(ends) or " stopped," in what or what.startswith(
                    ("refused", "s need")):
                self.stunt = None
            else:
                self.stunt = what
        elif ("T3: tilt and hump zeroed" in text or
              "T3: hump height zeroed" in text):
            self.hump_last = None
            self.hump_max = None
        elif "T3: hump peak " in text:
            match = cc.re.search(r"T3: hump peak (\d+) mm \(run max (\d+)\)", text)
            if match:
                self.hump_last = int(match.group(1))
                self.hump_max = int(match.group(2))
        elif "| sonar " in text:
            match = cc.re.search(r"\| sonar (-?\d+) mm", text)
            if match:
                self.sonar = int(match.group(1))
                self.sonar_ts = time.time()

    def save(self, text):
        with self.file_lock:
            self.logf.write(text + "\n")

    def out(self, text, save=True, show=True):
        if save:
            self.save(text)
        self.hub.push({"k": "car" if text.startswith("car ") else "out",
                       "t": text})

    # car_console.py's notices name typed commands; here they are buttons.
    WORDING = (
        ("then type: cal", "then press Calibrate"),
        ("calibrated -- type: go", "calibrated -- press Go"),
        ("then type: reset", "then press Reset"),
        ("type 'reset' first", "press Reset first"),
        ("type 'cal' first", "press Calibrate first"),
        ("type 'stop' first", "press Stop first"),
        ("'go' runs again, 'cal' recalibrates",
         "Go runs again, Calibrate recalibrates"),
    )

    def note(self, text, important=True):
        for old, new in self.WORDING:
            text = text.replace(old, new)
        line = "[%s] >> %s" % (cc.now_stamp(), text)
        self.save(line)
        self.hub.push({"k": "note", "imp": bool(important), "t": line})

    def notice(self, text):
        self.hub.push({"k": "notice",
                       "t": "[%s] >> %s" % (cc.now_stamp(), text)})

    def command(self, line):
        cmd = line.strip().lower()
        if cmd in ("quit", "exit", "q"):
            self.note("close the terminal window (or Ctrl-C) to stop the "
                      "console")
        elif cmd == "drive":
            self.note("use the drive pad on the page: turn on keyboard "
                      "driving, then hold W A S D")
        elif cmd in ("control", "verbose", "mode"):
            self.note("use the Control / Verbose switch at the top right")
        else:
            self.out("car> " + line.strip(), save=False)
            self.handle(line)

    def drive(self, key):
        try:
            self.mqtt.publish(cc.TOPIC_DRIVE, key.encode(), qos=0)
        except OSError as err:
            self.note("could not send drive key: %s" % err)
            return False
        if key != self.last_drive:
            self.save("[%s] drive %r" % (cc.now_stamp(), key))
            self.last_drive = key
        return True

    def snapshot(self):
        """Live state for the page, five times a second."""
        while True:
            age = time.time() - self.rec_time if self.rec_time else None
            self.hub.push({
                "k": "state", "broker": self.mqtt.up,
                "car": self.car_status, "age": age, "rec": self.rec,
                "letter": self.last_letter, "reads": self.n_barcodes,
                "plaid": self.plaid, "stunt": self.stunt,
                "map": self.car_map,
                "humpLast": self.hump_last, "humpMax": self.hump_max,
                "humpNow": self.hump_now(),
                "sonar": self.sonar if time.time() - self.sonar_ts < 4
                else None,
                "faultHelp": {name: [cc.FAULT_HELP[name],
                                     bool((1 << bit) & cc.CRITICAL)]
                              for bit, name in enumerate(cc.tdec.FAULTS)},
                "log": self.args.log,
            }, keep=False)
            time.sleep(0.2)

    def start(self):
        for target in (self.mqtt.run, self.mqtt.keepalive, self.watchdog,
                       self.snapshot):
            threading.Thread(target=target, daemon=True).start()
        self.note("saving the log to %s" % self.args.log)


def make_handler(console, hub, ui_port):
    allowed_hosts = {"127.0.0.1:%d" % ui_port, "localhost:%d" % ui_port}

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass                                    # keep the terminal quiet

        def _reply(self, code, body=b"", ctype="text/plain"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _local(self):
            # Only this machine's own page: a page from another site cannot
            # set the custom header without a CORS preflight, which is never
            # answered, and a foreign Host header (DNS rebinding) is refused.
            return self.headers.get("Host") in allowed_hosts

        def do_GET(self):
            if not self._local():
                return self._reply(403, b"forbidden")
            if self.path in ("/manual", "/manual.html"):
                with open(MANUAL, "rb") as handle:
                    return self._reply(200, handle.read(),
                                       "text/html; charset=utf-8")
            if self.path in ("/", "/index.html"):
                with open(PAGE, "rb") as handle:
                    return self._reply(200, handle.read(),
                                       "text/html; charset=utf-8")
            if self.path == "/meters.js":
                with open(METERS, "rb") as handle:
                    return self._reply(200, handle.read(),
                                       "text/javascript; charset=utf-8")
            if self.path == "/theme.css":
                with open(THEME, "rb") as handle:
                    return self._reply(200, handle.read(),
                                       "text/css; charset=utf-8")
            if self.path == "/events":
                return self._events()
            return self._reply(404, b"not found")

        def _events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            q = hub.join()
            try:
                while True:
                    try:
                        data = q.get(timeout=10)
                        self.wfile.write(b"data: " + data.encode() + b"\n\n")
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                hub.leave(q)
                self.close_connection = True

        def do_POST(self):
            if not self._local() or self.headers.get("X-Car-UI") != "1":
                return self._reply(403, b"forbidden")
            try:
                length = int(self.headers.get("Content-Length", "0"))
                body = json.loads(self.rfile.read(length) or b"{}")
            except ValueError:
                return self._reply(400, b"bad json")
            if self.path == "/cmd":
                line = str(body.get("line", ""))[:80]
                # Commands wait up to a few seconds for the car's reply:
                # run them off the request so the page stays responsive.
                threading.Thread(target=console.command, args=(line,),
                                 daemon=True).start()
                return self._reply(202, b"ok")
            if self.path == "/map":
                m = str(body.get("map", "")).upper()
                if len(m) != 26 or any(c not in "LRSU-" for c in m):
                    return self._reply(400, b"map: 26 characters L R S U -")
                ok = console.send_map(m)
                return self._reply(202 if ok else 503)
            if self.path == "/drive":
                key = str(body.get("key", ""))[:1].lower()
                if key not in DRIVE_KEYS or not key:
                    return self._reply(400, b"unknown drive key")
                ok = console.drive(key)
                return self._reply(204 if ok else 503)
            return self._reply(404, b"not found")

    return Handler


def main():
    root = os.path.abspath(os.path.join(HERE, "..", ".."))
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1", help="MQTT broker")
    ap.add_argument("--port", type=int, default=1883, help="MQTT port")
    ap.add_argument("--ui-port", type=int, default=8080,
                    help="web page port (127.0.0.1 only)")
    ap.add_argument("--log", default=os.path.join(
        root, "logs", time.strftime("car_%Y%m%d_%H%M%S.log")))
    args = ap.parse_args()
    args.mode = "control"                   # the page switches its own view

    hub = Hub()
    console = UiConsole(args, hub)
    console.start()
    server = ThreadingHTTPServer(("127.0.0.1", args.ui_port),
                                 make_handler(console, hub, args.ui_port))
    server.daemon_threads = True
    print("PicoCar console: http://127.0.0.1:%d   (Ctrl-C to stop)"
          % args.ui_port, flush=True)
    print("saving to %s" % args.log, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    console.save("[%s] >> console closed" % cc.now_stamp())


if __name__ == "__main__":
    main()
