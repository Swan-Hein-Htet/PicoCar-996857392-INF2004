# Change Requirement 1 — minimum-bit data representation

Every shared, queued or transmitted field uses only the bits its range
needs, stored with explicit shifts and masks (`app_program/app_bits.h`).
C bit-fields are not used for shared data: their bit order is
implementation-defined (Barr-C 5.2.b) and signed bit-fields are banned
(Barr-C 5.3.a). Signed values are two's-complement inside unsigned fields,
converted arithmetically.

## Packed access rules (NFR20 as amended at Tribunal 1)

- **R1** One writer per storage unit; it writes the whole unit with one
  aligned store; readers use one aligned load (`_Static_assert` on sizes).
- **R2** A multi-writer unit (`g_faults`) changes only inside
  `fault_set/fault_clear`, which mask interrupts for the read-modify-write.
  All writers run on core 1, so masking that core is sufficient.
- **R3** Nothing an ISR writes shares a unit with task data (encoder
  counters, echo word and ISR timestamps are each on their own).

## Data representation table

| Field | Range | Bits | Storage | Writer → readers |
|---|---|---|---|---|
| Mission state S0–S10 | 11 | 4 | `g_mission_word[3:0]` | T6 → all |
| Pending barcode command | none/L/R/S | 3 | `g_mission_word[6:4]` | T6 |
| Pending distance budget | 0–511 mm | 9 | `g_mission_word[15:7]` | T6 |
| Motion state M0–M7 | 8 | 3 | `g_motion_word[22:20]` | T4 |
| Wheel speed L, R | ±350 mm/s | 10+10 s | `g_motion_word[19:0]` | T4 → T3, T7 |
| Armed, heartbeat, last cmd | 1, 0–15, 0–7 | 1+4+3 | `g_motion_word[31:23]` | T4 → T1 |
| Distance (wrapping) | mod 65536 mm | 16 | `g_odom_word[15:0]` | T4 → T2, T3, T6 |
| Total distance + saturated | 0–32767 mm | 15+1 | `g_odom_word[31:16]` | T4 → T7 |
| Encoder counters L, R | free-running | 16 each | `g_enc_left/right` | own ISR → T4 |
| Line error | ±1.0 ×127 | 8 s | `g_line_word[7:0]` | T2 → T4 |
| Line state L0–L3 | 4 | 2 | `g_line_word[9:8]` | T2 |
| Junction, end flags | bool | 1+1 | `g_line_word[11:10]` | T2 |
| Line sequence | 0–15 | 4 | `g_line_word[15:12]` | T2 → T1 (liveness) |
| Last barcode | none/A/B/C/D/other | 3 | `g_line_word[18:16]` | T2 → T7 |
| Reject reason | 6 values | 3 | `g_line_word[21:19]` | T2 → T7 |
| Black-run length | 0–255 mm | 8 | `g_line_word[29:22]` | T2 → T5 |
| Black now, capturing | bool | 1+1 | `g_line_word[31:30]` | T2 |
| Barcode element widths | 1–255 ms | 8 × 29 | `uint8_t[29]` T2-private | — |
| IR calibration white/black | 0–4095 | 12+12 per ch | `uint32_t` T2-private | — |
| Pitch | ±45.0° in 0.1° | 10 s | `g_terrain_word[9:0]` | T3 → T7 |
| Turn rate | ±500 °/s | 10 s | `g_terrain_word[19:10]` | T3 → T7 |
| Motion class | 7 values | 3 | `g_terrain_word[22:20]` | T3 |
| IMU state I0–I7, hump flag | 8, bool | 3+1 | `g_terrain_word[26:23]` | T3 |
| Max hump peak | 0–127 mm | 7 | `g_hump_byte[6:0]` | T3 → T6, T7 |
| Echo width + valid | 0–32767 µs | 15+1 | `g_echo_word` | ISR_ECHO → T5 |
| Obstacle bearing | 0–18 (×10°) | 5 | `profile0[4:0]` | T5 → T7 (mutex) |
| Obstacle width | 0–1023 mm | 10 | `profile0[14:5]` | T5 |
| Closest point | 0–511 mm | 9 | `profile0[23:15]` | T5 |
| Plan decision | 4 | 2 | `profile0[25:24]` | T5 |
| Scan state O0–O11 | 12 | 4 | `profile0[29:26]` | T5 |
| Clearance L, R | 0–1023 mm | 10+10 | `profile1[19:0]` | T5 |
| Retry used | 0/1 | 1 | `profile1[20]` | T5 |
| Fault mask | 12 defined | 16 | `g_faults` | helpers (R2) |
| `motion_req_t` | cmd 7 / arg 0–4095 / src | 3+12+1 | `uint16_t` message | T6, T5 → T4 |
| `nav_cmd_t` | cmd / char 0–43 / reason | 3+6+3 | `uint16_t` message | T2 → T6 |
| MQTT telemetry | `docs/MQTT.md` | 168 | 21-byte array | T7 → broker |
| MQTT command | 4 cmds + 6-bit seq | 2+6 | 1 byte | broker → T7 |

## Measured sizes (`tools/cr1/measure.sh`, arm-none-eabi-gcc 14.2, -O2)

"Before" is the Week 6 data model written with natural C types and
compiled for the same target (`tools/cr1/natural_types.c`). There was no
`cr1-baseline` firmware build to compare against, so this is the honest
measured equivalent; re-run against that tag if the team creates one.

| Item | Natural types | Packed | Saving |
|---|---|---|---|
| Motion request (queue message) | 12 B | 2 B | 83 % |
| Navigation command (queue message) | 12 B | 2 B | 83 % |
| Telemetry record (MQTT payload) | 254 B JSON text | 21 B | 92 % |
| Command (MQTT payload) | 5 B text | 1 B | 80 % |
| Barcode capture buffer | 58 B | 29 B | 50 % |
| All shared state (line, motion, odometry, encoders, terrain, hump, echo, mission, faults, profile) | 156 B | 35 B | 78 % |

Whole image (`CONSOLE=uart WIFI=none`): text 74 044 B, data 20 B,
bss 16 744 B (mostly kernel task stacks and objects).

Still needed on hardware for the Week 13 evidence: the boot-time `[CR1]`
size print, and before/after GPIO-toggle WCET traces of T2, T4 and T7.
