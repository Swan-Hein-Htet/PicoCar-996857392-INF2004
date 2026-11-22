#!/bin/bash
# Variant build for what-if runs: firmware files with sed substitutions.
#   ./var_build.sh <name> <file under app_program> '<sed>' [<file> '<sed>' ...]
cd "$(dirname "$0")"
NAME=$1; shift
mkdir -p "build/var_$NAME"
SKIP=""; VAR=""
while [ $# -ge 2 ]; do
    FILE=$1; SCRIPT=$2; shift 2
    DIR=$(dirname "$FILE"); BASE=$(basename "$FILE")
    sed -e "$SCRIPT" -e "s|#include \"\.\./|#include \"../../../../app_program/|g" \
        -e "s|#include \"\([a-z_0-9]*\.h\)\"|#include \"../../../../app_program/$DIR/\1\"|" \
        "../../app_program/$FILE" > "build/var_$NAME/$BASE"
    SKIP="$SKIP -e $BASE"; VAR="$VAR build/var_$NAME/$BASE"
done
SRC=$(cd ../.. && find app_program -name "*.c" | grep -v $SKIP | sort | sed "s|^|../../|")
cc -std=c11 -D_DEFAULT_SOURCE -O1 -g -Istub -I. -DAPP_TEST=0 -DTM_MQTT=1 -DAPP_WATCHDOG=1 -w \
   -o "build/sim_$NAME" $SRC $VAR sim_kernel.c sim_hw.c sim_world.c sim_main.c -lpthread -lm
