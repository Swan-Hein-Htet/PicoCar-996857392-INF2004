#!/usr/bin/env python3
"""PicoCar telemetry database: a local, DynamoDB-style store fed live from
the car over MQTT, with a browser UI to browse runs and query the tables.

    python3 tools/telemetry/telemetry_db.py [--broker 127.0.0.1] [--port 1883]
                                            [--ui-port 8081] [--db FILE]
                                            [--import DIR] [--no-collect]

Normally started by tools/telemetry/start_telemetry.sh.

Storage is SQLite (standard library), shaped like DynamoDB so the tables
move to AWS unchanged: every table has a partition key and an optional sort
key, items are schemaless JSON, and the API offers GetItem, PutItem,
UpdateItem, DeleteItem, Query (key condition + filter + Limit +
ExclusiveStartKey, Limit counted before the filter, as in DynamoDB) and
Scan, plus one global secondary index.

Tables
    PicoCarRuns       PK run_id                one item per run segment
    PicoCarTelemetry  PK run_id, SK ts (ms)    every telemetry record (v2 24 bytes, v1 21), decoded
    PicoCarEvents     PK run_id, SK ts_seq     state changes, faults,
                      GSI by_type: PK type     barcodes, commands, link ...
    PicoCarLogs       PK run_id, SK ts_seq     every car log line

A run segment is either a "course" (from Go until finished, fault, reset
or the car going offline) or "standby" (the car online in between).
"""

import argparse
import csv
import glob
import json
import os
import queue
import re
import signal
import socket
import sqlite3
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, "..", "mqtt"))
sys.path.insert(0, os.path.join(HERE, ".."))
import car_console as cc          # noqa: E402  (MQTT client, names)
import decode_telemetry as tdec   # noqa: E402

PAGE = os.path.join(HERE, "telemetry_ui.html")
THEME = os.path.join(HERE, "..", "ui", "theme.css")
TOPIC_CMD = cc.TOPIC_CMD
TOPIC_DRIVE = cc.TOPIC_DRIVE

# ------------------------------------------------------------- tables ---

TABLES = {
    "PicoCarRuns": {
        "pk": "run_id", "sk": None,
        "about": "One item per run segment: a course attempt (Go to finish, "
                 "fault, reset or offline) or standby time in between. "
                 "Summary statistics, updated live.",
    },
    "PicoCarTelemetry": {
        "pk": "run_id", "sk": "ts",
        "about": "Every telemetry record (5 per second), decoded, with the "
                 "raw bytes and derived values. ts = milliseconds since 1970.",
    },
    "PicoCarEvents": {
        "pk": "run_id", "sk": "ts_seq",
        "gsi": {"by_type": "type"},
        "about": "Everything that happened: state changes, faults set and "
                 "cleared, barcodes and rejects, commands and replies, line "
                 "lost, obstacles, humps, link and packet loss.",
    },
    "PicoCarLogs": {
        "pk": "run_id", "sk": "ts_seq",
        "about": "Every log line the car published (and console lines from "
                 "imported logs).",
    },
}
ATTR_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)*$")
FILTER_OPS = {"=": "=", "<>": "<>", "<": "<", "<=": "<=", ">": ">",
              ">=": ">="}


def now_ms():
    return int(time.time() * 1000)


def ts_seq(ts, seq):
    """Sort key for events and logs: time, then arrival order."""
    return "%013d#%04d" % (ts, seq % 10000)


def iso(ts):
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(ts / 1000.0)) \
        + ".%03d" % (ts % 1000)


class Store:
    """DynamoDB-shaped tables on SQLite.  One writer connection (the
    collector thread); readers open their own (WAL mode)."""

    def __init__(self, path):
        self.path = path
        self.local = threading.local()
        con = self.con()
        con.execute("PRAGMA journal_mode=WAL")
        for name, spec in TABLES.items():
            con.execute('CREATE TABLE IF NOT EXISTS "%s" (pk TEXT NOT NULL, '
                        'sk NOT NULL, item TEXT NOT NULL, '
                        'PRIMARY KEY (pk, sk)) WITHOUT ROWID' % name)
            for index, attr in spec.get("gsi", {}).items():
                con.execute('CREATE INDEX IF NOT EXISTS "%s_%s" ON "%s" '
                            '(json_extract(item, \'$.%s\'), sk)'
                            % (name, index, name, attr))
        con.execute("CREATE TABLE IF NOT EXISTS meta (k TEXT PRIMARY KEY, "
                    "v TEXT)")
        con.commit()

    def con(self):
        con = getattr(self.local, "con", None)
        if con is None:
            con = sqlite3.connect(self.path, timeout=10,
                                  check_same_thread=False)
            con.execute("PRAGMA synchronous=NORMAL")
            self.local.con = con
        return con

    @staticmethod
    def keys(table, item):
        spec = TABLES[table]
        pk = item[spec["pk"]]
        sk = item[spec["sk"]] if spec["sk"] else ""
        return pk, sk

    # ---- writes ----
    def put(self, table, item, commit=False):
        pk, sk = self.keys(table, item)
        con = self.con()
        con.execute('INSERT OR REPLACE INTO "%s" (pk, sk, item) VALUES '
                    '(?, ?, ?)' % table, (pk, sk, json.dumps(item)))
        if commit:
            con.commit()

    def update(self, table, key, attrs):
        item = self.get(table, key)
        if item is None:
            return None
        item.update(attrs)
        self.put(table, item, commit=True)
        return item

    def delete_partition(self, pk):
        con = self.con()
        counts = {}
        for name in TABLES:
            cur = con.execute('DELETE FROM "%s" WHERE pk = ?' % name, (pk,))
            counts[name] = cur.rowcount
        con.commit()
        return counts

    # ---- reads ----
    def get(self, table, key):
        spec = TABLES[table]
        sk = key.get(spec["sk"], "") if spec["sk"] else ""
        row = self.con().execute('SELECT item FROM "%s" WHERE pk = ? AND '
                                 'sk = ?' % table,
                                 (key[spec["pk"]], sk)).fetchone()
        return json.loads(row[0]) if row else None

    @staticmethod
    def _filter_sql(filters, args):
        sql = []
        for flt in filters or []:
            attr, op = flt.get("attr", ""), flt.get("op", "=")
            if not ATTR_RE.match(attr):
                raise ValueError("bad attribute name %r" % attr)
            path = "json_extract(item, '$.%s')" % attr
            value = flt.get("value")
            if op in FILTER_OPS:
                sql.append("%s %s ?" % (path, FILTER_OPS[op]))
                args.append(value)
            elif op == "between":
                sql.append("%s BETWEEN ? AND ?" % path)
                args += list(value)[:2]
            elif op == "begins_with":
                sql.append("substr(%s, 1, ?) = ?" % path)
                args += [len(str(value)), str(value)]
            elif op == "contains":
                sql.append("instr(CAST(%s AS TEXT), ?) > 0" % path)
                args.append(str(value))
            elif op == "exists":
                sql.append("%s IS NOT NULL" % path)
            elif op == "not_exists":
                sql.append("%s IS NULL" % path)
            else:
                raise ValueError("unknown filter operator %r" % op)
        return sql

    @staticmethod
    def _coerce(table, value):
        """Telemetry sort keys are numbers; the rest are strings."""
        if TABLES[table]["sk"] == "ts":
            try:
                return int(value)
            except (TypeError, ValueError):
                raise ValueError("ts is a number (milliseconds)")
        return value

    def query(self, table, pk, sk=None, filters=None, limit=100,
              forward=True, start=None, index=None):
        """DynamoDB Query.  sk = {"op": "="|"<"|...|"between"|
        "begins_with", "value": v or [a, b]}.  Limit counts items read
        before the filter, like DynamoDB; LastEvaluatedKey continues."""
        spec = TABLES[table]
        limit = max(1, min(int(limit or 100), 5000))
        args, where = [], []
        if index:
            attr = spec.get("gsi", {}).get(index)
            if not attr:
                raise ValueError("%s has no index %r" % (table, index))
            where.append("json_extract(item, '$.%s') = ?" % attr)
        else:
            where.append("pk = ?")
        args.append(pk)
        if sk and sk.get("op"):
            op, value = sk["op"], sk.get("value")
            if op in FILTER_OPS and op != "<>":
                where.append("sk %s ?" % op)
                args.append(self._coerce(table, value))
            elif op == "between":
                where.append("sk BETWEEN ? AND ?")
                args += [self._coerce(table, v) for v in list(value)[:2]]
            elif op == "begins_with":
                where.append("substr(sk, 1, ?) = ?")
                args += [len(str(value)), str(value)]
            else:
                raise ValueError("bad sort key condition %r" % op)
        if start:
            # Continue after the last item of the previous page.
            if index:
                where.append("(sk, pk) %s (?, ?)" % (">" if forward else "<"))
                args += [self._coerce(table, start["sk"]), start["pk"]]
            else:
                where.append("sk %s ?" % (">" if forward else "<"))
                args.append(self._coerce(table, start["sk"]))
        order = "ASC" if forward else "DESC"
        sql = 'SELECT pk, sk, item FROM "%s" WHERE %s ORDER BY sk %s%s ' \
              'LIMIT ?' % (table, " AND ".join(where), order,
                           ", pk %s" % order if index else "")
        rows = self.con().execute(sql, args + [limit]).fetchall()
        return self._page(rows, filters, limit)

    def scan(self, table, filters=None, limit=100, start=None):
        limit = max(1, min(int(limit or 100), 5000))
        args, where = [], ["1"]
        if start:
            where.append("(pk, sk) > (?, ?)")
            args += [start["pk"], self._coerce(table, start["sk"])
                     if TABLES[table]["sk"] else ""]
        rows = self.con().execute(
            'SELECT pk, sk, item FROM "%s" WHERE %s ORDER BY pk, sk LIMIT ?'
            % (table, " AND ".join(where)), args + [limit]).fetchall()
        return self._page(rows, filters, limit)

    def _page(self, rows, filters, limit):
        items = [json.loads(r[2]) for r in rows]
        if filters:
            items = self._apply_filters(items, filters)
        last = None
        if len(rows) == limit:
            last = {"pk": rows[-1][0], "sk": rows[-1][1]}
        return {"Items": items, "Count": len(items),
                "ScannedCount": len(rows), "LastEvaluatedKey": last}

    def _apply_filters(self, items, filters):
        # Same semantics as _filter_sql, on already-read items (DynamoDB
        # applies FilterExpression after reading Limit items).
        con = sqlite3.connect(":memory:")
        con.execute("CREATE TABLE t (n INTEGER, item TEXT)")
        con.executemany("INSERT INTO t VALUES (?, ?)",
                        [(n, json.dumps(i)) for n, i in enumerate(items)])
        args = []
        sql = self._filter_sql(filters, args)
        keep = {r[0] for r in con.execute(
            "SELECT n FROM t WHERE " + " AND ".join(sql), args)}
        return [i for n, i in enumerate(items) if n in keep]

    def describe(self):
        out = []
        con = self.con()
        for name, spec in TABLES.items():
            count, size = con.execute('SELECT count(*), coalesce(sum('
                                      'length(item)), 0) FROM "%s"'
                                      % name).fetchone()
            out.append({
                "TableName": name, "ItemCount": count, "TableSizeBytes": size,
                "KeySchema": [{"AttributeName": spec["pk"],
                               "KeyType": "HASH"}] +
                             ([{"AttributeName": spec["sk"],
                                "KeyType": "RANGE"}] if spec["sk"] else []),
                "GlobalSecondaryIndexes": [
                    {"IndexName": i, "KeySchema": [
                        {"AttributeName": a, "KeyType": "HASH"},
                        {"AttributeName": spec["sk"], "KeyType": "RANGE"}]}
                    for i, a in spec.get("gsi", {}).items()],
                "About": spec["about"],
            })
        return out

    def meta_get(self, key):
        row = self.con().execute("SELECT v FROM meta WHERE k = ?",
                                 (key,)).fetchone()
        return row[0] if row else None

    def meta_set(self, key, value):
        self.con().execute("INSERT OR REPLACE INTO meta VALUES (?, ?)",
                           (key, value))


# ---------------------------------------------------------- collector ---

FAULTS = tdec.FAULTS
MISSION = tdec.MISSION
CRITICAL = cc.CRITICAL
COURSE_STATES = {3, 4, 5, 6, 7, 8}
LOG_EVENTS = (
    # (pattern in the car's log line, event type)
    ("T2: barcode '", "barcode"),
    ("T2: barcode rejected", "barcode_reject"),
    ("T5: obstacle at", "obstacle"),
    ("T2: mounting", "calibration"),
    ("T6: calibration sweep", "calibration"),
    ("T6: scan pass", "scan"),
    ("microT-Kernel", "boot"),
    ("lost bytes", "log_gap"),
    ("T6: manual drive", "manual_drive"),
    ("T6: barcode map", "barcode_map"),
)


def fault_names(mask):
    return [n for b, n in enumerate(FAULTS) if mask >> b & 1]


class Run:
    """One run segment and its running statistics."""

    def __init__(self, run_id, kind, ts, rec=None):
        self.item = {
            "run_id": run_id, "kind": kind, "started": ts,
            "started_iso": iso(ts), "ended": None, "ended_iso": None,
            "duration_s": 0.0, "outcome": "open", "label": "", "notes": "",
            "records": 0, "records_lost": 0, "log_lines": 0, "events": 0,
            "distance_mm": 0, "speed_max_mms": 0, "speed_mean_mms": 0.0,
            "line_err_abs_max": 0, "line_err_rms": 0.0,
            "line_err_abs_mean": 0.0, "turn_rate_abs_max_dps": 0,
            "line_lost": 0, "searches": 0, "barcodes": [],
            "barcode_rejects": 0, "faults_seen": [], "obstacles": 0,
            "hump_max_mm": 0, "commands": 0, "state_time_s": {},
            "states_visited": [], "first_state": None, "last_state": None,
            "telemetry_gaps": 0, "source": "live",
        }
        self._sum_speed = 0.0
        self._sum_err2 = 0.0
        self._sum_err_abs = 0.0
        self._driving = 0
        self._last_dist = rec["distance_mm"] if rec else None
        self._state_since = ts
        self._state = rec["mission_state"] if rec else None
        self.dirty = True

    @property
    def run_id(self):
        return self.item["run_id"]

    def add_record(self, rec, ts):
        it = self.item
        it["records"] += 1
        state = rec["mission_state"]
        name = "S%d %s" % (state, MISSION[state] if state < len(MISSION)
                           else "?")
        if it["first_state"] is None:
            it["first_state"] = name
        it["last_state"] = name
        if name not in it["states_visited"]:
            it["states_visited"].append(name)
        if self._state is None:
            self._state, self._state_since = state, ts
        if state != self._state:
            self.close_state(ts)
            self._state, self._state_since = state, ts
        if self._last_dist is not None:
            it["distance_mm"] += (rec["distance_mm"] - self._last_dist) \
                % 32768
        self._last_dist = rec["distance_mm"]
        speed = (abs(rec["speed_left_mms"]) + abs(rec["speed_right_mms"])) / 2
        it["speed_max_mms"] = max(it["speed_max_mms"],
                                  abs(rec["speed_left_mms"]),
                                  abs(rec["speed_right_mms"]))
        it["turn_rate_abs_max_dps"] = max(it["turn_rate_abs_max_dps"],
                                          abs(rec["turn_rate_dps"]))
        it["hump_max_mm"] = max(it["hump_max_mm"], rec["hump_max_mm"])
        if state in COURSE_STATES:
            err = rec["line_error"]
            self._driving += 1
            self._sum_speed += speed
            self._sum_err2 += err * err
            self._sum_err_abs += abs(err)
            it["line_err_abs_max"] = max(it["line_err_abs_max"], abs(err))
            it["speed_mean_mms"] = round(self._sum_speed / self._driving, 1)
            it["line_err_rms"] = round((self._sum_err2 / self._driving)
                                       ** 0.5, 1)
            it["line_err_abs_mean"] = round(self._sum_err_abs
                                            / self._driving, 1)
        for name in fault_names(rec["faults"]):
            if name not in it["faults_seen"]:
                it["faults_seen"].append(name)
        self.touch(ts)

    def close_state(self, ts):
        if self._state is None:
            return
        key = "S%d %s" % (self._state, MISSION[self._state]
                          if self._state < len(MISSION) else "?")
        times = self.item["state_time_s"]
        times[key] = round(times.get(key, 0.0)
                           + (ts - self._state_since) / 1000.0, 2)
        self._state_since = ts

    def touch(self, ts):
        it = self.item
        it["duration_s"] = round((ts - it["started"]) / 1000.0, 2)
        self.dirty = True

    def end(self, ts, outcome):
        self.close_state(ts)
        self.item["ended"] = ts
        self.item["ended_iso"] = iso(ts)
        self.item["outcome"] = outcome
        self.touch(ts)


class Collector:
    """Turns MQTT messages into items.  Everything runs on one thread (the
    writer), fed through a queue, so the run logic needs no locks."""

    GAP_MS = 3000           # telemetry silence that counts as a gap
    OFFLINE_MS = 10000      # silence that ends the run segment

    def __init__(self, store, source="live"):
        self.store = store
        self.source = source
        self.q = queue.Queue()
        self.run = None
        self.prev = None
        self.prev_ts = None
        self.seq = 0
        self.last_drive = None
        self.car_status = "unknown"
        self.stats = {"telemetry": 0, "logs": 0, "events": 0,
                      "started": now_ms(), "last_telemetry": None,
                      "last_message": None}
        self.pending = 0
        self.last_commit = time.time()
        self.lock = threading.Lock()        # guards live snapshot only
        self.live = {"rec": None, "ts": None}
        self.stopped = threading.Event()
        self.lost_acc = 0
        self.created = []
        self.lost_event_ts = 0

    # ---- run segments ----
    def new_run(self, kind, ts, rec=None):
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(ts / 1000.0))
        run_id = "%s-%s" % ("course" if kind == "course" else "standby",
                            stamp)
        if self.store.get("PicoCarRuns", {"run_id": run_id}):
            run_id += "-%03d" % (ts % 1000)
        self.run = Run(run_id, kind, ts, rec)
        self.run.item["source"] = self.source
        self.created.append(run_id)
        self.flush_run(force=True)
        return self.run

    def end_run(self, ts, outcome):
        if self.run is None:
            return
        self.run.end(ts, outcome)
        self.event(ts, "run_end", outcome=outcome,
                   kind=self.run.item["kind"])
        self.flush_run(force=True)
        self.run = None

    def ensure_run(self, ts, rec=None):
        if self.run is None:
            self.new_run("standby", ts, rec)
        return self.run

    def flush_run(self, force=False):
        if self.run and (self.run.dirty or force):
            self.store.put("PicoCarRuns", self.run.item)
            self.run.dirty = False

    # ---- items ----
    def event(self, ts, etype, **attrs):
        run = self.ensure_run(ts)
        self.seq += 1
        item = {"run_id": run.run_id, "ts_seq": ts_seq(ts, self.seq),
                "ts": ts, "time": iso(ts), "type": etype}
        item.update(attrs)
        self.store.put("PicoCarEvents", item)
        run.item["events"] += 1
        self.stats["events"] += 1
        self.pending += 1

    def log(self, ts, text, source="car"):
        run = self.ensure_run(ts)
        self.seq += 1
        self.store.put("PicoCarLogs", {
            "run_id": run.run_id, "ts_seq": ts_seq(ts, self.seq), "ts": ts,
            "time": iso(ts), "source": source, "text": text})
        run.item["log_lines"] += 1
        run.touch(ts)
        self.stats["logs"] += 1
        self.pending += 1

    # ---- inputs ----
    def on_telemetry(self, rec, raw, ts):
        prev, prev_ts = self.prev, self.prev_ts
        state = rec["mission_state"]
        pstate = prev["mission_state"] if prev else None

        # Segments: a course starts at Go (S2/S9 -> driving), or when
        # telemetry first shows the car already driving.
        if self.run is not None and prev_ts is not None and \
                ts - prev_ts > self.OFFLINE_MS:
            self.end_run(prev_ts, "offline")
        if state in COURSE_STATES and (self.run is None or
                                       self.run.item["kind"] != "course"):
            if self.run is not None:
                self.end_run(ts, "course started")
            self.new_run("course", ts, rec)
            self.event(ts, "run_start", kind="course",
                       from_state=self.state_name(pstate))
        elif self.run is not None and self.run.item["kind"] == "course" \
                and state not in COURSE_STATES:
            outcome = {9: "finished", 10: "fault"}.get(state, "reset")
            if state == 10:
                # Name the critical faults: warnings such as IMU are often
                # on all the time and did not stop the car.
                crit = fault_names(rec["faults"] & CRITICAL)
                outcome = "fault: " + ", ".join(
                    crit or fault_names(rec["faults"]) or ["unknown"])
            # keep the record that shows the outcome in the course
            self.run.add_record(rec, ts)
            self.store_record(rec, raw, ts, prev, prev_ts)
            self.diff_events(rec, prev, ts)
            self.end_run(ts, outcome)
            self.new_run("standby", ts, rec)
            self.prev, self.prev_ts = rec, ts
            return
        run = self.ensure_run(ts, rec)
        run.add_record(rec, ts)
        self.store_record(rec, raw, ts, prev, prev_ts)
        self.diff_events(rec, prev, ts)
        self.prev, self.prev_ts = rec, ts

    def store_record(self, rec, raw, ts, prev, prev_ts):
        lost = 0
        if prev is not None and prev_ts is not None and \
                ts - prev_ts < self.OFFLINE_MS:
            lost = max(0, (rec["seq"] - prev["seq"]) % 256 - 1)
        item = {"run_id": self.run.run_id, "ts": ts, "time": iso(ts)}
        item.update(rec)
        state = rec["mission_state"]
        item.update({
            "mission_name": MISSION[state] if state < len(MISSION) else "?",
            "motion_name": cc.MOTION[rec["motion_state"]],
            "line_name": cc.LINE[rec["line_state"]],
            "fault_names": fault_names(rec["faults"]),
            "faults_critical": bool(rec["faults"] & CRITICAL),
            "speed_mean_mms": (rec["speed_left_mms"]
                               + rec["speed_right_mms"]) / 2.0,
            "speed_diff_mms": rec["speed_right_mms"] - rec["speed_left_mms"],
            "dt_ms": ts - prev_ts if prev_ts else None,
            "lost_before": lost,
            "raw_hex": raw.hex() if raw else None,
        })
        self.store.put("PicoCarTelemetry", item)
        if lost:
            self.run.item["records_lost"] += lost
            # One event per 5 s at most, with the total since the last.
            self.lost_acc += lost
            if self.lost_acc >= 3 and ts - self.lost_event_ts >= 5000:
                self.event(ts, "packet_loss", records=self.lost_acc,
                           since_ms=ts - self.lost_event_ts
                           if self.lost_event_ts else None)
                self.lost_acc, self.lost_event_ts = 0, ts
        if prev_ts and self.GAP_MS < ts - prev_ts < self.OFFLINE_MS:
            self.run.item["telemetry_gaps"] += 1
            self.event(ts, "telemetry_gap", seconds=round((ts - prev_ts)
                                                          / 1000.0, 1))
        self.stats["telemetry"] += 1
        self.stats["last_telemetry"] = ts
        self.pending += 1
        with self.lock:
            self.live = {"rec": item, "ts": ts}

    @staticmethod
    def state_name(state):
        if state is None:
            return None
        return "S%d %s" % (state, MISSION[state] if state < len(MISSION)
                           else "?")

    def diff_events(self, rec, prev, ts):
        run = self.run.item
        if prev is None:
            self.event(ts, "first_record", state=self.state_name(
                rec["mission_state"]), faults=fault_names(rec["faults"]))
            return
        if rec["mission_state"] != prev["mission_state"]:
            self.event(ts, "state", from_state=self.state_name(
                prev["mission_state"]), to_state=self.state_name(
                rec["mission_state"]), distance_mm=rec["distance_mm"])
            if rec["mission_state"] == 8:
                run["searches"] += 1
        gained = rec["faults"] & ~prev["faults"]
        cleared = prev["faults"] & ~rec["faults"]
        for name in fault_names(gained):
            bit = FAULTS.index(name)
            self.event(ts, "fault_set", fault=name,
                       critical=bool((1 << bit) & CRITICAL),
                       meaning=cc.FAULT_HELP[name],
                       state=self.state_name(rec["mission_state"]))
        for name in fault_names(cleared):
            self.event(ts, "fault_cleared", fault=name)
        if rec["line_state"] == 3 and prev["line_state"] != 3:
            run["line_lost"] += 1
            self.event(ts, "line_lost", distance_mm=rec["distance_mm"],
                       line_error=prev["line_error"])
        if rec["line_state"] != 3 and prev["line_state"] == 3:
            self.event(ts, "line_found", distance_mm=rec["distance_mm"])
        if rec["bypass_plan"] != prev["bypass_plan"] and rec["bypass_plan"]:
            run["obstacles"] += 1
            self.event(ts, "obstacle_plan",
                       plan=cc.PLAN[rec["bypass_plan"]],
                       width_mm=rec["obstacle_width_mm"],
                       closest_mm=rec["obstacle_closest_mm"],
                       bearing_deg=rec["obstacle_bearing_x10deg"] * 10 - 90)
        if rec["hump_max_mm"] > prev["hump_max_mm"]:
            self.event(ts, "hump", peak_mm=rec["hump_max_mm"])
        if rec["reject_reason"] != prev["reject_reason"] and \
                rec["reject_reason"]:
            run["barcode_rejects"] += 0     # counted from the log line
        if rec["motion_class"] == 6 and prev["motion_class"] != 6:
            self.event(ts, "impact", state=self.state_name(
                rec["mission_state"]))

    def on_log(self, text, ts, source="car"):
        self.log(ts, text, source)
        if source != "car":
            return
        run = self.run.item
        for pattern, etype in LOG_EVENTS:
            if pattern in text:
                attrs = {"text": text}
                if etype == "barcode":
                    idx = text.find("T2: barcode '")
                    letter = text[idx + 13:idx + 14]
                    attrs["letter"] = letter
                    # The car's verdict from its A-Z map ("-> left" ...);
                    # older logs have none: the old fixed A-D meaning.
                    verdict = re.search(
                        r"-> (left|right|straight|u-turn|unmapped)", text)
                    attrs["command"] = verdict.group(1) if verdict else {
                        "A": "left", "B": "right", "C": "straight",
                        "D": "u-turn"}.get(letter, "unmapped")
                    run["barcodes"].append(letter)
                elif etype == "barcode_map":
                    match = re.search(r"map ([LRSU-]{26})", text)
                    if match:
                        attrs["map"] = match.group(1)
                        attrs["mapped"] = " ".join(
                            "%s=%s" % (chr(65 + i), c)
                            for i, c in enumerate(match.group(1)) if c != "-")
                    attrs["refused"] = "refused" in text
                elif etype == "barcode_reject":
                    run["barcode_rejects"] += 1
                    match = re.search(r"reason (\d+) \(([^)]*)\)", text)
                    if match:
                        attrs["reason"] = int(match.group(1))
                        attrs["why"] = match.group(2)
                    match = re.search(r"(\d+) elements", text)
                    if match:
                        attrs["elements"] = int(match.group(1))
                self.event(ts, etype, **attrs)
                return
        match = re.search(r"CMD (\w+) seq (\d+) (received|duplicate)", text)
        if match:
            self.event(ts, "command_ack", command=match.group(1),
                       cmd_seq=int(match.group(2)), result=match.group(3),
                       text=text)
            return
        body = text.split("] ", 1)[1] if text.startswith("[") and \
            "] " in text else text
        if body.startswith(cc.PROBLEMS) and not (
                body.startswith("T2: ch") and "never saw" not in body):
            self.event(ts, "problem", text=text)

    def on_status(self, status, ts):
        if status == self.car_status:
            return
        self.car_status = status
        self.ensure_run(ts)
        self.event(ts, "link", status=status)
        if status == "offline" and self.run is not None:
            self.end_run(ts, "offline")

    def on_command(self, payload, ts, source="mqtt"):
        if not payload:
            return
        byte = payload[0]
        name = {v: k for k, v in tdec.COMMANDS.items()}.get(byte & 3, "?")
        self.ensure_run(ts)
        self.run.item["commands"] += 1
        self.event(ts, "command", command=name, cmd_seq=byte >> 2,
                   via=source)

    def on_drive(self, key, ts):
        if key == self.last_drive:
            return
        self.last_drive = key
        self.event(ts, "drive_key", key={"w": "forward", "s": "back",
                                         "a": "left", "d": "right",
                                         " ": "stop"}.get(key, key))

    def on_shutdown(self, ts):
        if self.run is not None:
            self.end_run(ts, "collector stopped")
        self.store.con().commit()
        self.stopped.set()

    def tick(self, ts):
        """Once a second: end a run whose car went silent, save stats."""
        if self.run is not None and self.prev_ts is not None and \
                ts - self.prev_ts > self.OFFLINE_MS and self.source == "live":
            self.end_run(self.prev_ts, "offline")
            self.prev, self.prev_ts = None, None
        self.flush_run()

    # ---- thread ----
    def submit(self, kind, *args):
        self.q.put((kind, args, now_ms()))

    def loop(self):
        while True:
            try:
                kind, args, ts = self.q.get(timeout=0.5)
                self.stats["last_message"] = ts
                try:
                    getattr(self, "on_" + kind)(*args, ts)
                except Exception as err:          # keep collecting
                    print("collector: %s failed: %r" % (kind, err),
                          file=sys.stderr, flush=True)
            except queue.Empty:
                pass
            if time.time() - self.last_commit >= 0.5:
                self.tick(now_ms())
                self.store.con().commit()
                self.pending = 0
                self.last_commit = time.time()


# ---------------------------------------------------------------- MQTT ---

class CarLink(cc.Mqtt):
    """car_console's client, also subscribed to the command and drive
    topics so commands from any console are recorded."""

    def connect(self):
        sock = socket.create_connection((self.host, self.port), timeout=5)
        sock.settimeout(None)
        self.sock = sock
        body = cc.enc_str("MQTT") + bytes([4, 0x02]) + struct.pack("!H", 30) \
            + cc.enc_str("p5is20-telemetry-db-%d" % os.getpid())
        self._send(bytes([0x10]) + cc.enc_len(len(body)) + body)
        b0, body = self._read_packet()
        if b0 >> 4 != 2 or len(body) < 2 or body[1] != 0:
            raise OSError("broker refused the connection")
        subs = [(cc.TOPIC_LOG, 0), (cc.TOPIC_STATUS, 1),
                (cc.TOPIC_TELEM, 0), (TOPIC_CMD, 1), (TOPIC_DRIVE, 0)]
        payload = b"".join(cc.enc_str(t) + bytes([q]) for t, q in subs)
        body = struct.pack("!H", self._next_pid()) + payload
        self._send(bytes([0x82]) + cc.enc_len(len(body)) + body)
        self.up = True


def start_live(collector, host, port, status):
    def on_publish(topic, payload):
        if topic == cc.TOPIC_TELEM:
            try:
                rec = tdec.decode(payload)
            except ValueError:
                return
            collector.submit("telemetry", rec, bytes(payload))
        elif topic == cc.TOPIC_LOG:
            collector.submit("log", payload.decode(errors="replace").rstrip())
        elif topic == cc.TOPIC_STATUS:
            collector.submit("status", payload.decode(errors="replace"))
        elif topic == TOPIC_CMD:
            collector.submit("command", bytes(payload))
        elif topic == TOPIC_DRIVE:
            collector.submit("drive", payload.decode(errors="replace")[:1])

    def on_link(up, why):
        status["broker"] = up
        status["broker_error"] = why
        print("broker %s%s" % ("connected" if up else "lost",
                               "" if up else " (%s)" % why), flush=True)

    link = CarLink(host, port, on_publish, on_link)
    for target in (link.run, link.keepalive):
        threading.Thread(target=target, daemon=True).start()
    return link


# -------------------------------------------------------------- import ---

STAMP_RE = re.compile(r"^\[(\d\d):(\d\d):(\d\d)\] (.*)$")


def import_logs(store, folder):
    """Replay the console's saved logs (car_<date>_<time>.csv + .log)
    through the collector, each file once."""
    files = sorted(glob.glob(os.path.join(folder, "car_*.csv")))
    done = 0
    for path in files:
        stat = os.stat(path)
        key = "imported:%s" % os.path.basename(path)
        if store.meta_get(key) == str(int(stat.st_mtime)):
            continue
        # A file that grew since last time: replace its earlier runs.
        runs_key = "runs:%s" % os.path.basename(path)
        for run_id in json.loads(store.meta_get(runs_key) or "[]"):
            store.delete_partition(run_id)
        n, created = import_one(store, path)
        store.meta_set(key, str(int(stat.st_mtime)))
        store.meta_set(runs_key, json.dumps(created))
        store.con().commit()
        done += 1
        if n < 0:
            print("skipped %s: recorded live already"
                  % os.path.basename(path), flush=True)
        elif n:
            print("imported %s: %d records" % (os.path.basename(path), n),
                  flush=True)
    return done


def import_one(store, csv_path):
    base = csv_path[:-4]
    match = re.search(r"car_(\d{8})_(\d{6})", base)
    day = time.strptime(match.group(1), "%Y%m%d") if match else None
    items = []
    with open(csv_path, newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                ts = int(float(row["pc_time"]) * 1000)
                rec = {f[0]: int(row[f[0]]) if row.get(f[0], "") != "" else 0
                       for f in tdec.FIELDS}
            except (ValueError, KeyError, TypeError):
                continue
            items.append((ts, 0, "telemetry", rec))
    log_path = base + ".log"
    if os.path.exists(log_path) and day is not None:
        start = items[0][0] if items else None
        last = start
        order = 0
        with open(log_path, errors="replace") as handle:
            for line in handle:
                line = line.rstrip("\n")
                stamp = STAMP_RE.match(line)
                if stamp:
                    ts = int(time.mktime((day.tm_year, day.tm_mon,
                                          day.tm_mday, int(stamp.group(1)),
                                          int(stamp.group(2)),
                                          int(stamp.group(3)), 0, 0, -1))
                             * 1000)
                    if last and ts < last - 12 * 3600 * 1000:
                        ts += 24 * 3600 * 1000      # ran past midnight
                    last = ts
                    body = stamp.group(4)
                    order += 1
                    if body.startswith("car> "):
                        items.append((ts, order, "console_cmd", body[5:]))
                    elif body.startswith(">> car is "):
                        items.append((ts, order, "status",
                                      body[10:].strip().lower()))
                    elif body.startswith("drive "):
                        items.append((ts, order, "drive", body[7:8]))
                    else:
                        items.append((ts, order, "console", body))
                elif line.startswith("car ") and last:
                    order += 1
                    items.append((last, order, "log", line[4:]))
    items.sort(key=lambda x: (x[0], x[1]))
    if items and live_overlap(store, items[0][0], items[-1][0]):
        return -1, []
    col = Collector(store, source="import")
    for ts, _, kind, data in items:
        if kind == "telemetry":
            col.on_telemetry(data, None, ts)
        elif kind == "log":
            col.on_log(data, ts)
        elif kind == "console":
            col.on_log(data, ts, source="console")
        elif kind == "console_cmd":
            col.ensure_run(ts)
            col.run.item["commands"] += 1
            col.event(ts, "console_command", line=data)
        elif kind == "status":
            col.on_status(data, ts)
        elif kind == "drive":
            col.on_drive(data, ts)
    if col.run is not None:
        col.end_run(col.prev_ts or now_ms(), "log ended")
    store.con().commit()
    return sum(1 for i in items if i[2] == "telemetry"), col.created


def live_overlap(store, first, last):
    """True if the collector recorded this stretch live: importing the
    console's copy of it would duplicate the runs."""
    row = store.con().execute(
        'SELECT count(*) FROM "PicoCarRuns" WHERE '
        "json_extract(item, '$.source') = 'live' AND "
        "json_extract(item, '$.started') <= ? AND "
        "coalesce(json_extract(item, '$.ended'), 9e15) >= ?",
        (last, first)).fetchone()
    return row[0] > 0


# ------------------------------------------------------------- web API ---

def series_for(store, run_id, max_points=2400):
    """Telemetry of one run as column arrays for the charts, thinned to
    about max_points (every k-th, keeping each window's line-error
    extreme so spikes survive)."""
    rows = store.con().execute(
        'SELECT item FROM "PicoCarTelemetry" WHERE pk = ? ORDER BY sk',
        (run_id,)).fetchall()
    recs = [json.loads(r[0]) for r in rows]
    step = max(1, -(-len(recs) // max_points))
    keep = []
    for start in range(0, len(recs), step):
        window = recs[start:start + step]
        keep.append(max(window, key=lambda r: abs(r["line_error"])))
    cols = ["ts", "speed_left_mms", "speed_right_mms", "line_error",
            "turn_rate_dps", "distance_mm", "mission_state", "line_state",
            "faults", "motion_state", "hump_max_mm", "obstacle_closest_mm"]
    return {"n": len(recs), "step": step,
            "cols": {c: [r.get(c) for r in keep] for c in cols}}


def make_handler(store, collector, status, ui_port, db_path):
    hosts = {"127.0.0.1:%d" % ui_port, "localhost:%d" % ui_port}

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype="application/json", extra=None):
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _local(self):
            return self.headers.get("Host") in hosts

        def do_GET(self):
            if not self._local():
                return self._send(403, {"error": "forbidden"})
            url = urlparse(self.path)
            qs = {k: v[0] for k, v in parse_qs(url.query).items()}
            try:
                if url.path in ("/", "/index.html"):
                    with open(PAGE, "rb") as handle:
                        return self._send(200, handle.read(),
                                          "text/html; charset=utf-8")
                if url.path == "/theme.css":
                    with open(THEME, "rb") as handle:
                        return self._send(200, handle.read(),
                                          "text/css; charset=utf-8")
                if url.path == "/api/tables":
                    return self._send(200, {"Tables": store.describe(),
                                            "db": db_path,
                                            "db_bytes": os.path.getsize(
                                                db_path)})
                if url.path == "/api/runs":
                    return self._send(200, self.runs(qs))
                if url.path == "/api/run":
                    return self._send(200, self.run(qs["id"]))
                if url.path == "/api/live":
                    return self._send(200, self.live(qs))
                if url.path == "/api/export":
                    return self.export(qs)
            except (KeyError, ValueError) as err:
                return self._send(400, {"error": str(err)})
            return self._send(404, {"error": "not found"})

        def runs(self, qs):
            rows = store.con().execute(
                'SELECT item FROM "PicoCarRuns"').fetchall()
            runs = [json.loads(r[0]) for r in rows]
            runs.sort(key=lambda r: r["started"], reverse=True)
            return {"Items": runs[:int(qs.get("limit", 2000))],
                    "Count": len(runs)}

        def run(self, run_id):
            item = store.get("PicoCarRuns", {"run_id": run_id})
            if item is None:
                raise KeyError("no run %s" % run_id)
            events = store.query("PicoCarEvents", run_id, limit=5000)
            logs = store.query("PicoCarLogs", run_id, limit=5000)
            return {"run": item, "series": series_for(store, run_id),
                    "events": events["Items"], "logs": logs["Items"],
                    "logs_more": logs["LastEvaluatedKey"] is not None}

        def live(self, qs):
            since = int(qs.get("since", "0"))
            with collector.lock:
                live = dict(collector.live)
            run = collector.run.item if collector.run else None
            out = {"broker": status.get("broker", False),
                   "collecting": status.get("collecting", False),
                   "car_status": collector.car_status,
                   "now": now_ms(), "stats": collector.stats,
                   "run": run, "rec": live.get("rec")}
            if run:
                window = max(since, now_ms() - 60000)
                rows = store.con().execute(
                    'SELECT item FROM "PicoCarTelemetry" WHERE pk = ? AND '
                    'sk > ? ORDER BY sk', (run["run_id"], window)).fetchall()
                out["recent"] = [json.loads(r[0]) for r in rows]
                ev = store.query("PicoCarEvents", run["run_id"],
                                 limit=30, forward=False)
                out["events"] = ev["Items"]
            return out

        def export(self, qs):
            run_id, table = qs["run"], qs.get("table", "PicoCarTelemetry")
            if table not in TABLES:
                raise ValueError("unknown table")
            rows = store.con().execute(
                'SELECT item FROM "%s" WHERE pk = ? ORDER BY sk' % table,
                (run_id,)).fetchall()
            items = [json.loads(r[0]) for r in rows]
            name = "%s_%s" % (run_id, table)
            if qs.get("fmt") == "json":
                body = json.dumps({"run": store.get(
                    "PicoCarRuns", {"run_id": run_id}), table: items},
                    indent=1)
                return self._send(200, body, "application/json", {
                    "Content-Disposition":
                        'attachment; filename="%s.json"' % name})
            cols = []
            for it in items:
                for k in it:
                    if k not in cols:
                        cols.append(k)
            lines = [",".join(cols)]
            for it in items:
                cells = []
                for c in cols:
                    v = it.get(c, "")
                    v = json.dumps(v) if isinstance(v, (list, dict)) else \
                        ("" if v is None else str(v))
                    if any(ch in v for ch in ',"\n'):
                        v = '"' + v.replace('"', '""') + '"'
                    cells.append(v)
                lines.append(",".join(cells))
            return self._send(200, "\n".join(lines) + "\n", "text/csv", {
                "Content-Disposition":
                    'attachment; filename="%s.csv"' % name})

        def do_POST(self):
            if not self._local() or self.headers.get("X-Telemetry-UI") != "1":
                return self._send(403, {"error": "forbidden"})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                body = json.loads(self.rfile.read(length) or b"{}")
                table = body.get("TableName", "")
                if self.path != "/api/delete-run" and table not in TABLES:
                    raise ValueError("unknown table %r" % table)
                if self.path == "/api/query":
                    return self._send(200, store.query(
                        table, body["pk"], body.get("sk"),
                        body.get("filters"), body.get("Limit", 100),
                        body.get("ScanIndexForward", True),
                        body.get("ExclusiveStartKey"),
                        body.get("IndexName") or None))
                if self.path == "/api/scan":
                    return self._send(200, store.scan(
                        table, body.get("filters"), body.get("Limit", 100),
                        body.get("ExclusiveStartKey")))
                if self.path == "/api/get":
                    item = store.get(table, body["Key"])
                    return self._send(200, {"Item": item})
                if self.path == "/api/update":
                    if table != "PicoCarRuns":
                        raise ValueError("only run labels and notes can "
                                         "be edited")
                    attrs = {k: str(v)[:2000] for k, v in
                             body.get("Attributes", {}).items()
                             if k in ("label", "notes")}
                    item = store.update(table, body["Key"], attrs)
                    if collector.run and item and \
                            collector.run.run_id == item["run_id"]:
                        collector.run.item.update(attrs)
                    return self._send(200, {"Attributes": item})
                if self.path == "/api/delete-run":
                    run_id = body["run_id"]
                    if collector.run and collector.run.run_id == run_id:
                        raise ValueError("this run is still being "
                                         "recorded")
                    return self._send(200, {"Deleted":
                                            store.delete_partition(run_id)})
            except (KeyError, ValueError, TypeError) as err:
                return self._send(400, {"error": str(err)})
            except sqlite3.Error as err:
                return self._send(500, {"error": "database: %s" % err})
            return self._send(404, {"error": "not found"})

    return Handler


def close_interrupted(store):
    """Runs still marked open were cut off (crash, power loss): close
    them at their last telemetry record."""
    con = store.con()
    rows = con.execute('SELECT item FROM "PicoCarRuns" WHERE '
                       "json_extract(item, '$.outcome') = 'open'").fetchall()
    for (raw,) in rows:
        item = json.loads(raw)
        last = con.execute('SELECT max(sk) FROM "PicoCarTelemetry" WHERE '
                           'pk = ?', (item["run_id"],)).fetchone()[0]
        end = last or item["started"]
        item.update({"outcome": "interrupted", "ended": end,
                     "ended_iso": iso(end),
                     "duration_s": round((end - item["started"]) / 1000.0,
                                         2)})
        store.put("PicoCarRuns", item)
    con.commit()
    if rows:
        print("closed %d run(s) left open last time" % len(rows),
              flush=True)


def stop_on_term(*_):
    raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--broker", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--ui-port", type=int, default=8081)
    ap.add_argument("--db", default=os.path.join(ROOT, "telemetry",
                                                 "picocar.db"))
    ap.add_argument("--import", dest="import_dir", default=None,
                    help="replay saved console logs (car_*.csv/.log) first")
    ap.add_argument("--no-collect", action="store_true",
                    help="browse only, do not connect to the broker")
    args = ap.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.db)), exist_ok=True)
    store = Store(args.db)
    if args.import_dir:
        print("importing saved logs from %s ..." % args.import_dir,
              flush=True)
        n = import_logs(store, args.import_dir)
        print("%d new log file(s) imported" % n, flush=True)

    close_interrupted(store)
    collector = Collector(store)
    status = {"broker": False, "collecting": not args.no_collect}
    threading.Thread(target=collector.loop, daemon=True).start()
    if not args.no_collect:
        start_live(collector, args.broker, args.port, status)

    server = ThreadingHTTPServer(("127.0.0.1", args.ui_port),
                                 make_handler(store, collector, status,
                                              args.ui_port, args.db))
    server.daemon_threads = True
    signal.signal(signal.SIGTERM, stop_on_term)   # the launcher's kill
    print("PicoCar telemetry: http://127.0.0.1:%d   database %s   "
          "(Ctrl-C to stop)" % (args.ui_port, args.db), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    collector.submit("shutdown")
    collector.stopped.wait(3)


if __name__ == "__main__":
    main()
