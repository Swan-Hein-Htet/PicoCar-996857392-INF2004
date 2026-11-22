#!/bin/bash
# Measure Change Requirement 1 sizes: packed (firmware) vs natural types.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT="$HERE/../.."
OBJ="${TMPDIR:-/tmp}/cr1_natural.o"
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -O2 -c -o "$OBJ" "$HERE/natural_types.c"
echo "== natural-type equivalents (bytes) =="
arm-none-eabi-nm -S --size-sort "$OBJ" | while read -r _a size _t name; do
    printf "%-24s %d\n" "$name" "$((16#$size))"
done
ELF=$(ls -t "$ROOT"/build_make/*.elf | head -1)
echo "== packed firmware objects in $(basename "$ELF") (bytes) =="
arm-none-eabi-nm -S "$ELF" | grep -E " (g_line_word|g_motion_word|g_odom_word|g_terrain_word|g_hump_byte|g_echo_word|g_mission_word|g_faults|g_enc_left|g_enc_right|g_vehicle_state)$" \
  | while read -r _a size _t name; do printf "%-24s %d\n" "$name" "$((16#$size))"; done
echo "== image =="
arm-none-eabi-size "$ELF"
