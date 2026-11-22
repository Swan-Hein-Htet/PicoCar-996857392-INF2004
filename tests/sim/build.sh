#!/bin/sh
# Build the PicoCar simulator: the real application sources, unchanged,
# against the simulated kernel and RP2040 peripherals.
#   ./build.sh                normal build (TM_MQTT=1, watchdog on) -> build/sim
#   SAN=1 ./build.sh          AddressSanitizer + UBSan          -> build/sim_san
#   APP_TEST=1 ./build.sh     with the on-target tests          -> build/sim_test
#   BARCODE_TEST=1 ./build.sh barcode bench build (bc_bench scenario) -> build/sim_bc
set -e
cd "$(dirname "$0")"
mkdir -p build
NAME=sim
FLAGS="-std=c11 -D_DEFAULT_SOURCE -O1 -g -Istub -I."
if [ -n "$SAN" ]; then
    FLAGS="$FLAGS -fsanitize=address,undefined -fno-omit-frame-pointer"
    NAME=sim_san
fi
[ "${APP_TEST:-0}" = 1 ] && NAME=sim_test
[ "${BARCODE_TEST:-0}" = 1 ] && NAME=sim_bc
DEFS="-DAPP_TEST=${APP_TEST:-0} -DAPP_BARCODE_TEST=${BARCODE_TEST:-0} -DTM_MQTT=1 -DAPP_WATCHDOG=1"
SRC=$(cd ../.. && find app_program -name '*.c' | sort | sed 's|^|../../|')
# shellcheck disable=SC2086
cc $FLAGS $DEFS -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -o "build/$NAME" \
   $SRC sim_kernel.c sim_hw.c sim_world.c sim_main.c -lpthread -lm
echo "built tests/sim/build/$NAME"
