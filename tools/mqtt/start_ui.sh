#!/bin/bash
# Start everything the car needs on this PC and open the console in the
# browser:
#   tools/mqtt/start_ui.sh
# Runs the MQTT broker (Mosquitto if installed, else mini_broker.py;
# BROKER=python forces mini_broker.py), the broker finder, and car_ui.py,
# then opens http://127.0.0.1:8080 (UI_PORT=xxxx to change it).  Every
# console line, state change and fault is saved to
# logs/car_<date>_<time>.log, telemetry to the matching .csv, as with
# start_broker.sh.  Ctrl-C (or closing this window) stops it all.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
BREW=$(brew --prefix 2>/dev/null || echo /usr/local)
MOSQ="$BREW/sbin/mosquitto"
UI_PORT=${UI_PORT:-8080}
URL="http://127.0.0.1:$UI_PORT"

mkdir -p "$ROOT/logs"
LOG="$ROOT/logs/car_$(date +%Y%m%d_%H%M%S).log"
PIDS=()
cleanup() { kill "${PIDS[@]}" 2>/dev/null; wait 2>/dev/null; echo; echo "saved $LOG"; }
trap cleanup EXIT INT TERM

echo "PC addresses (the car finds this PC by itself):"
ifconfig 2>/dev/null | awk '/inet / && $2 != "127.0.0.1" {print "   " $2}'

python3 -u "$HERE/broker_finder.py" >> "${LOG%.log}_finder.txt" 2>&1 &
PIDS+=($!)

if [ -x "$MOSQ" ] && [ "${BROKER:-}" != "python" ]; then
    "$MOSQ" -c "$HERE/mosquitto.conf" > "${LOG%.log}_broker.txt" 2>&1 &
else
    python3 "$HERE/mini_broker.py" > "${LOG%.log}_broker.txt" 2>&1 &
fi
PIDS+=($!)
sleep 1

python3 -u "$HERE/car_ui.py" --log "$LOG" --ui-port "$UI_PORT" &
UI_PID=$!
PIDS+=($UI_PID)
sleep 1
if [ -n "${NO_OPEN:-}" ]; then echo "console at $URL"
elif command -v open >/dev/null; then open "$URL"
elif command -v xdg-open >/dev/null; then xdg-open "$URL" >/dev/null 2>&1
else echo "open $URL in your browser"; fi
wait "$UI_PID"
