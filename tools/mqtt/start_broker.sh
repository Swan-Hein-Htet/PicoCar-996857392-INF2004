#!/bin/bash
# Start everything the car needs on this PC, then open the car console.
#   tools/mqtt/start_broker.sh
# Runs the MQTT broker (Mosquitto if installed, else mini_broker.py;
# BROKER=python forces mini_broker.py) and the broker finder, then
# car_console.py: type cal / go / stop / reset / status to control the
# car; every console line, state change and fault is shown and saved to
# logs/car_<date>_<time>.log (telemetry in .csv).  "quit" or Ctrl-D
# stops it all.  Shows only important notices; type "verbose" for every
# line, or start with MODE=verbose.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
BREW=$(brew --prefix 2>/dev/null || echo /usr/local)
MOSQ="$BREW/sbin/mosquitto"

mkdir -p "$ROOT/logs"
LOG="$ROOT/logs/car_$(date +%Y%m%d_%H%M%S).log"
PIDS=()
cleanup() { kill "${PIDS[@]}" 2>/dev/null; wait 2>/dev/null; echo; echo "saved $LOG"; }
trap cleanup EXIT INT TERM

echo "PC addresses (the car finds this PC by itself):"
ifconfig | awk '/inet / && $2 != "127.0.0.1" {print "   " $2}'

python3 -u "$HERE/broker_finder.py" >> "${LOG%.log}_finder.txt" 2>&1 &
PIDS+=($!)

if [ -x "$MOSQ" ] && [ "${BROKER:-}" != "python" ]; then
    "$MOSQ" -c "$HERE/mosquitto.conf" > "${LOG%.log}_broker.txt" 2>&1 &
    PIDS+=($!)
else
    # No Mosquitto: the built-in Python broker.
    python3 "$HERE/mini_broker.py" > "${LOG%.log}_broker.txt" 2>&1 &
    PIDS+=($!)
fi
sleep 1
# The console: type cal / go / stop / reset / status; shows and saves
# every log line, state change and fault (docs/MQTT.md "Console").
python3 "$HERE/car_console.py" --log "$LOG" --mode "${MODE:-control}"
