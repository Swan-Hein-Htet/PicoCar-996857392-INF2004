#!/bin/sh
# Build and run the host tests, then cross-check the Python decoder.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT="$HERE/../.."
OUT="${TMPDIR:-/tmp}/picocar_host_test"
cc -std=c11 -Wall -Wextra -O1 -I"$HERE/stub" -DAPP_TEST=0 \
   -o "$OUT" "$HERE/test_host.c" \
   "$ROOT/app_program/subsystem3/barcode.c" \
   "$ROOT/app_program/subsystem1/telemetry.c" \
   "$ROOT/app_program/app_math.c" -lm
"$OUT" | tee "$OUT.log"
REF=$(grep "REF" "$OUT.log" | awk '{print $3}')
python3 "$ROOT/tools/decode_telemetry.py" --selftest "$REF"
