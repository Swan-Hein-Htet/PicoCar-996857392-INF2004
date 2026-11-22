#!/usr/bin/env python3
"""A small MQTT 3.1.1 broker for the PicoCar, standard library only.

Used by start_broker.sh when Mosquitto is not installed.  Enough of MQTT
for the car and ordinary clients: CONNECT (with last will), PUBLISH QoS 0
and 1, retained messages, SUBSCRIBE / UNSUBSCRIBE with + and # wildcards,
PINGREQ, DISCONNECT.  It also prints the car's console log and status
lines and appends them to a file, so no separate subscriber is needed.

    mini_broker.py [--port 1883] [--log FILE]
"""
import argparse
import asyncio
import sys

LOG_TOPIC = "p5is20/car/log"
STATUS_TOPIC = "p5is20/car/status"


def topic_matches(filt: str, topic: str) -> bool:
    f = filt.split("/")
    t = topic.split("/")
    for i, part in enumerate(f):
        if part == "#":
            return True
        if i >= len(t):
            return False
        if part != "+" and part != t[i]:
            return False
    return len(f) == len(t)


def enc_len(n: int) -> bytes:
    out = bytearray()
    while True:
        b = n % 128
        n //= 128
        out.append(b | (0x80 if n else 0))
        if not n:
            return bytes(out)


def enc_str(s: bytes) -> bytes:
    return len(s).to_bytes(2, "big") + s


class Broker:
    def __init__(self, logfile):
        self.clients = {}       # writer -> {"subs": {filter: qos}, "id": str}
        self.retained = {}      # topic -> (payload, qos)
        self.logfile = logfile
        self.next_id = 1

    def show(self, topic: str, payload: bytes) -> None:
        if topic == LOG_TOPIC:
            line = payload.decode("utf-8", "replace")
        elif topic == STATUS_TOPIC:
            line = "[status] " + payload.decode("utf-8", "replace")
        else:
            return
        print(line, flush=True)
        if self.logfile:
            self.logfile.write(line + "\n")
            self.logfile.flush()

    def publish(self, topic: str, payload: bytes, qos: int, retain: bool) -> None:
        self.show(topic, payload)
        if retain:
            if payload:
                self.retained[topic] = (payload, qos)
            else:
                self.retained.pop(topic, None)
        for w, c in list(self.clients.items()):
            granted = max((q for f, q in c["subs"].items() if topic_matches(f, topic)),
                          default=None)
            if granted is not None:
                self.send_publish(w, topic, payload, min(qos, granted), False)

    def send_publish(self, w, topic, payload, qos, retain):
        header = 0x30 | (qos << 1) | (1 if retain else 0)
        body = enc_str(topic.encode())
        if qos:
            body += self.next_id.to_bytes(2, "big")
            self.next_id = self.next_id % 65535 + 1
        body += payload
        try:
            w.write(bytes([header]) + enc_len(len(body)) + body)
        except Exception:
            pass

    async def handle(self, r: asyncio.StreamReader, w: asyncio.StreamWriter):
        peer = w.get_extra_info("peername")
        will = None
        clean = False
        name = "?"
        try:
            while True:
                b0 = (await r.readexactly(1))[0]
                mult, length = 1, 0
                while True:
                    d = (await r.readexactly(1))[0]
                    length += (d & 0x7F) * mult
                    mult *= 128
                    if not d & 0x80:
                        break
                body = await r.readexactly(length) if length else b""
                kind = b0 >> 4
                if kind == 1:                                   # CONNECT
                    i = 2 + int.from_bytes(body[0:2], "big")    # protocol name
                    flags = body[i + 1]
                    i += 4                                      # level, flags, keepalive
                    n = int.from_bytes(body[i:i + 2], "big")
                    name = body[i + 2:i + 2 + n].decode(errors="replace")
                    i += 2 + n
                    if flags & 0x04:
                        n = int.from_bytes(body[i:i + 2], "big")
                        wt = body[i + 2:i + 2 + n].decode()
                        i += 2 + n
                        n = int.from_bytes(body[i:i + 2], "big")
                        wm = body[i + 2:i + 2 + n]
                        will = (wt, wm, (flags >> 3) & 3, bool(flags & 0x20))
                    self.clients[w] = {"subs": {}, "id": name}
                    w.write(b"\x20\x02\x00\x00")               # CONNACK accepted
                    print(f"[broker] {name} connected from {peer[0]}", flush=True)
                elif kind == 3:                                 # PUBLISH
                    qos = (b0 >> 1) & 3
                    n = int.from_bytes(body[0:2], "big")
                    topic = body[2:2 + n].decode(errors="replace")
                    i = 2 + n
                    if qos:
                        pid = body[i:i + 2]
                        i += 2
                        w.write(b"\x40\x02" + pid)              # PUBACK
                    self.publish(topic, body[i:], min(qos, 1), bool(b0 & 1))
                elif kind == 8:                                 # SUBSCRIBE
                    pid = body[0:2]
                    i, codes = 2, bytearray()
                    while i < len(body):
                        n = int.from_bytes(body[i:i + 2], "big")
                        filt = body[i + 2:i + 2 + n].decode()
                        q = min(body[i + 2 + n], 1)
                        i += 3 + n
                        self.clients[w]["subs"][filt] = q
                        codes.append(q)
                        for t, (p, rq) in self.retained.items():
                            if topic_matches(filt, t):
                                self.send_publish(w, t, p, min(rq, q), True)
                    w.write(b"\x90" + enc_len(2 + len(codes)) + pid + bytes(codes))
                elif kind == 10:                                # UNSUBSCRIBE
                    pid = body[0:2]
                    i = 2
                    while i < len(body):
                        n = int.from_bytes(body[i:i + 2], "big")
                        self.clients[w]["subs"].pop(body[i + 2:i + 2 + n].decode(), None)
                        i += 2 + n
                    w.write(b"\xb0\x02" + pid)
                elif kind == 12:                                # PINGREQ
                    w.write(b"\xd0\x00")
                elif kind == 14:                                # DISCONNECT
                    clean = True
                    break
                await w.drain()
        except (asyncio.IncompleteReadError, ConnectionError, IndexError, UnicodeDecodeError):
            pass
        finally:
            self.clients.pop(w, None)
            print(f"[broker] {name} disconnected{'' if clean else ' (lost)'}", flush=True)
            if will and not clean:
                self.publish(will[0], will[1], min(will[2], 1), will[3])
            w.close()


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--log", default=None, help="append the car's log here")
    args = ap.parse_args()
    logfile = open(args.log, "a", encoding="utf-8") if args.log else None
    broker = Broker(logfile)
    server = await asyncio.start_server(broker.handle, "0.0.0.0", args.port)
    print(f"[broker] MQTT broker on port {args.port}", flush=True)
    async with server:
        await server.serve_forever()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(asyncio.run(main()))
    except KeyboardInterrupt:
        sys.exit(0)
