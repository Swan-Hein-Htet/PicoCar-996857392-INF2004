#!/bin/bash
# PicoCar telemetry database: record everything the car publishes and
# browse it in the browser.
#   tools/telemetry/start_telemetry.sh
# - uses the MQTT broker already running on this PC (start_ui.sh or
#   start_broker.sh), or starts one if none is (Mosquitto if installed,
#   else mini_broker.py)
# - imports any saved console logs in logs/ not imported yet
# - collects live into telemetry/picocar.db and opens http://127.0.0.1:8081
# Settings: UI_PORT=8081  MQTT_PORT=1883  DB=telemetry/picocar.db
#           NO_IMPORT=1 (skip logs/)  NO_OPEN=1 (do not open the browser)
# Ctrl-C (or closing this window) stops it; the database stays.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
MQTT=$ROOT/tools/mqtt
UI_PORT=${UI_PORT:-8081}
MQTT_PORT=${MQTT_PORT:-1883}
DB=${DB:-$ROOT/telemetry/picocar.db}
URL="http://127.0.0.1:$UI_PORT"
PIDS=()
cleanup() { kill "${PIDS[@]}" 2>/dev/null; wait 2>/dev/null; echo; echo "database kept at $DB"; }
trap cleanup EXIT INT TERM

mkdir -p "$ROOT/logs" "$(dirname "$DB")"
if nc -z 127.0.0.1 "$MQTT_PORT" 2>/dev/null; then
    echo "using the MQTT broker already running on port $MQTT_PORT"
else
    BREW=$(brew --prefix 2>/dev/null || echo /usr/local)
    BLOG="$ROOT/logs/telemetry_broker_$(date +%Y%m%d_%H%M%S).txt"
    if [ -x "$BREW/sbin/mosquitto" ] && [ "$MQTT_PORT" = 1883 ]; then
        "$BREW/sbin/mosquitto" -c "$MQTT/mosquitto.conf" > "$BLOG" 2>&1 &
    else
        python3 "$MQTT/mini_broker.py" --port "$MQTT_PORT" > "$BLOG" 2>&1 &
    fi
    PIDS+=($!)
    python3 -u "$MQTT/broker_finder.py" >> "${BLOG%.txt}_finder.txt" 2>&1 &
    PIDS+=($!)
    echo "started an MQTT broker on port $MQTT_PORT (the car finds this PC by itself)"
    sleep 1
fi

ARGS=(--port "$MQTT_PORT" --ui-port "$UI_PORT" --db "$DB")
[ -z "${NO_IMPORT:-}" ] && ARGS+=(--import "$ROOT/logs")
python3 -u "$HERE/telemetry_db.py" "${ARGS[@]}" &
SERVER=$!
PIDS+=($SERVER)
for _ in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do
    nc -z 127.0.0.1 "$UI_PORT" 2>/dev/null && break
    sleep 0.5
done
if [ -n "${NO_OPEN:-}" ]; then echo "telemetry at $URL"
elif command -v open >/dev/null; then open "$URL"
elif command -v xdg-open >/dev/null; then xdg-open "$URL" >/dev/null 2>&1
else echo "open $URL in your browser"; fi
wait "$SERVER"
