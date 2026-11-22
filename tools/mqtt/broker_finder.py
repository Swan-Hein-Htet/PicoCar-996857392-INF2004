#!/usr/bin/env python3
"""Answer the PicoCar's broker search on the local network.

The car broadcasts "P5IS20 broker?" to UDP port 18830 until it gets
"P5IS20 broker" back; the reply's source address is where it connects
for MQTT.  Needed because a phone hotspot gives the PC a new address
each time.  Standard library only.
"""
import socket
import sys
import time

PORT = 18830
ASK = b"P5IS20 broker?"
REPLY = b"P5IS20 broker"


def main() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", PORT))
    print(f"[finder] answering broker searches on UDP {PORT}", flush=True)
    last = {}
    while True:
        data, (host, port) = sock.recvfrom(64)
        if data != ASK:
            continue
        sock.sendto(REPLY, (host, port))
        now = time.time()
        if now - last.get(host, 0) > 10:
            print(f"[finder] car at {host} asked; told it to use this PC",
                  flush=True)
        last[host] = now


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
