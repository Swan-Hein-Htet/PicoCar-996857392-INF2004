# MQTT interface (FR20–FR23)

| Topic | Direction | QoS | Payload |
|---|---|---|---|
| `p5is20/car/telemetry` | car → broker | 0 | 21-byte binary record, 5 Hz |
| `p5is20/car/cmd` | broker → car | 1 | exactly 1 byte |
| `p5is20/car/status` | car → broker | 1, retained | `online`; last will `offline` |
| `p5is20/car/log` | car → broker | 0 | one console line per message, `[  sss.mmm] text` (time since boot) |
| `p5is20/car/drive` | broker → car | 0 | one character: `w` `a` `s` `d` drive (hold: repeats keep it going), space stops; `!` arms plaid (250 mm/s), `.` disarms it; with plaid armed `j` J-turn, `r` reverse slalom, `b` bootleg, `t` trail turn, `p` pendulum turn, `f` drift, `o` donut, and any key aborts a stunt. Parked states only (S1 before calibrating, S2, S9); `w` refused within 150 mm of an obstacle (300 mm in plaid), stunts within 500 mm |
| `p5is20/car/map` | broker → car | 1 | barcode letter → command, set before the run: 26 characters for A-Z, each `L` (turn left at the next junction), `R` (turn right), `S` (straight on), `U` (U-turn now) or `-` (unmapped: read and reported, no command). Applied only while parked; refused during a run. `?` makes the car log its map. Default `LRSU` then 22 `-` |

Broker address, SSID and client ID: `config/wifi_secrets.h` (copy the
example). Build: `make NET=lwip MQTT=1 -j8` with `PICO_SDK_PATH` set.

## Watching a run from the PC

1. Phone hotspot on, 2.4 GHz ("Maximise compatibility" on an iPhone). Join
   it with the PC.
2. `tools/mqtt/start_broker.sh`: starts Mosquitto (port 1883), the
   broker finder (UDP 18830) and the car console (see "Console" below),
   which shows and saves everything to `logs/car_<date>_<time>.log`.
   Allow incoming connections if macOS asks.
3. Power the car. It joins the hotspot, broadcasts `P5IS20 broker?` on UDP
   18830, connects to whichever PC answered, and publishes its log.

`MQTT_BROKER_IP "auto"` enables that search; a fixed address skips it.

The log is copied from `tm_printf` by `tm_log_tap()` (a weak hook in
`lib/libtm/libtm_printf.c`, overridden in `net_bridge.c`) into an 8 KB ring,
about 100 lines; `mqtt_utk.c` publishes up to 8 lines every 50 ms. If the
ring fills (no broker for a long time) newer text is dropped and a
`[log] lost bytes: N` line marks the gap. The USB / UART console still
prints everything.

## Console: controlling the car from the PC

`tools/mqtt/start_broker.sh` ends in `tools/mqtt/car_console.py` (standard
library only). Type at the `car>` prompt:

| Command | Does | Sends |
|---|---|---|
| `cal` | calibrate. In S1: same as GP20. In S2/S9: RESET, wait for S1, then START (fresh calibration). Refused while driving or in S10 | START (or RESET + START) |
| `go` | start driving. Only in S2/S9; in S0/S1 it says "cal first" | START |
| `stop` | stop now (`FAULT_REMOTE_STOP`, critical) | STOP |
| `reset` | clear a fault (S10), or back to S0 to recalibrate (S2/S9) | RESET |
| `end` | backup END marker | END |
| `status` | every telemetry field, decoded, with fault explanations | - |
| `faults` | active faults explained | - |
| `raw NAME` | send a command byte with no state check | NAME |
| `drive` | manual driving with w/a/s/d, space stops, q leaves (parked only) | `p5is20/car/drive` |
| `control` | show only important notices (default) | - |
| `verbose` | show every line from the car as well | - |
| `mode` | say which of the two is on | - |

The console checks the car's state from telemetry before sending, and
waits up to 2 s for the car's `CMD ... received` log line; it reports
"no reply" otherwise. GP20 and GP21 keep working alongside it.

The log file is the same in both modes. Start in verbose with
`MODE=verbose tools/mqtt/start_broker.sh`. A scan pass with too few bars for a result says so, with its darkest
reading. Control mode shows calibrating,
calibrated, driving, barcode letters and rejects, obstacles, humps,
junctions, line lost, faults with their cause, and car online/offline.

### Console in the browser

`tools/mqtt/start_ui.sh` starts the same broker and broker finder, then
`tools/mqtt/car_ui.py`, and opens http://127.0.0.1:8080 (`UI_PORT=xxxx`
to change it). The page has the same commands as buttons (Calibrate, Go,
Stop, Reset, End course, Scan barcode, Full status) plus a command box
for the rest (`faults`, `raw NAME`, `help`). It also shows the
mission state as a course line with the current station marked, live line
error and wheel speeds, the last barcode letter, active faults with
their cause, and the console log with a Control / Verbose switch. The
drive pad sends w/a/s/d while held (on screen, or the keyboard once
"Keyboard driving" is on); release or space stops, and Esc sends STOP
from anywhere on the page.

It reuses `car_console.py`'s Console class, so the checks, replies and
saved files (`logs/car_<date>_<time>.log` and `.csv`) are identical. The
page is served on 127.0.0.1 only and refuses requests from other sites.
Ctrl-C in the terminal stops everything. Standard library only, and no
internet is needed (system fonts).

### Barcode commands (A-Z map)

The car reads every Code 39 character. Which letters carry a command is
decided before each run:
- **On the console page:** in the **Barcode commands** panel, pick a
  command (turn left, turn right, straight, U-turn or unmapped), click
  letters to paint them, then press **Send to car**.
  - The summary shows which letters do what.
  - Amber underlines mark letters that differ from the car's map.
  - The status reads "The car has this map" once the car confirms.
- **In the terminal console:** `map` shows the map, `map K=L Z=U` sets it
  (unlisted letters unmapped), and `map clear` / `map default` reset it.

The car applies a map only while parked (S0, S1 before calibrating, S2,
S9). It logs the active map at start-up, on request and on every change:
`T6: barcode map ----------L--------------U (2 mapped, set)`. A read logs
the car's verdict, for example `T2: barcode 'K' (widths by distance) ->
left` or `-> unmapped`. The map lives in RAM: after a power cycle it is
back to the default A=L B=R C=S D=U, and the console shows that as soon
as the car reports it. Telemetry `last_barcode` now carries the command
(1 left, 2 right, 3 straight, 4 U-turn, 5 unmapped).

### Manual control page

http://127.0.0.1:8080/manual (linked from the console page) drives the car by
hand:
- **Drive:** W A S D, held. Space stops; Esc is the emergency STOP, as on
  the console.
- **Plaid** drives at 250 mm/s. It is engaged behind a warning and an
  "area is clear" checkbox, and the page shows it as engaged only once the
  car confirms in its log. The car turns plaid off itself when it leaves
  the parked states.
- **Stunts:** five show manoeuvres, keys J R B T P, live only in plaid. Each
  card's path diagram is drawn from the same geometry the firmware runs.
  Any key aborts a stunt.

| Stunt | Key | Choreography (all at plaid speed) | Net turn |
|---|---|---|---|
| J-turn | J | reverse 350 mm, spin 180 in place, forward 350 mm | 180 |
| Reverse slalom | R | five reverse arcs, radius 250 mm: +30, -60, +60, -60, +30 | 0 |
| Bootleg turn | B | forward 400 mm, swing 180 round the stopped left wheel, forward 400 mm | 180 |
| Trail turn | T | forward 400 mm, right-hand 90 on radius 200 mm braking 90 % to 70 % then 85 %, forward 300 mm | 90 right |
| Pendulum turn | P | forward 300 mm, feint 25 right (radius 300), swing 115 left (radius 180), forward 300 mm | 90 left |
| Drift | F | forward 400 mm, then **raw power**: kick (left 90 %, right -30 %) 0.3 s, hold (80 %, 35 %) 0.7 s, counter-steer (45 %, 75 %) 0.25 s, then forward 300 mm | about 120 right on a grippy floor; slides further on a smooth one |
| Donut | O | forward 200 mm, raw power round the stopped right wheel (left 85 %) 4.5 s, forward 200 mm | about two turns |

Stunts are timed (wheel speeds from geometry, plus half of each speed
change at 400 mm/s²). Drift and donut steps drive motor power directly,
with no speed loop or acceleration limit, so the tyres can break
traction; the wheels reach about 390 mm/s, above plaid. On a slippery floor the real turn differs; tests/sim
lands each within 16 degrees.

### Telemetry database

`tools/telemetry/start_telemetry.sh` records everything the car publishes
into a local DynamoDB-style database and opens a page to browse runs and
query the tables (http://127.0.0.1:8081). It runs alongside either
console. See docs/TELEMETRY.md.

What is saved, and shown in verbose mode (`logs/car_<date>_<time>.log`, telemetry in
`.csv` alongside; broker and finder output in `_broker.txt`/`_finder.txt`):

- every car console line (`car [  sss.mmm] ...`);
- from the car: `STATE S2 READY -> S3 FOLLOW`, `CMD START seq 5 received
  in S2 READY -> start driving`, `FAULT SET ENCODER (0x0004) -- critical,
  motors off` / `FAULT CLEAR ...`, and a `STATUS` line (state, motion,
  line state and error, wheel speeds, encoders, distance, sonar range,
  obstacle, hump, last barcode character, fault mask, T2 overruns) once a second while running,
  every 5 s while parked;
- from the console (`>>` lines): state changes with what to type next,
  faults gained/cleared with their meaning, barcodes read or rejected
  (with reason), line lost, obstacle plan, hump peak, missed telemetry
  records, telemetry silent for 3 s, car online/offline.

Log lines are up to 256 bytes; the car's log ring is 8 KB.

## Telemetry record, version 2 (24 bytes)

Bit stream, least-significant bit first (bit *n* is bit `n % 8` of byte
`n / 8`). Signed fields are two's complement. Decoder:
`tools/decode_telemetry.py`. It also reads version 1 records (21 bytes,
bits 0-167 only; the version 2 fields then read 0) and rejects anything
else.

| Bits | Field | Width | Notes |
|---|---|---|---|
| 0–3 | version | 4 | = 2 |
| 4–11 | sequence | 8 | wraps; gaps = lost records |
| 12–15 | mission state | 4 | S0 INIT … S10 FAULT |
| 16–18 | motion state | 3 | M0–M7 |
| 19–28 | left speed | 10 s | mm/s |
| 29–38 | right speed | 10 s | mm/s |
| 39–54 | left encoder | 16 | wrapping; unwrap on the dashboard |
| 55–70 | right encoder | 16 | wrapping |
| 71–72 | line state | 2 | L0 cal, L1 track, L2 junction, L3 lost |
| 73–80 | line error | 8 s | ±127 = ±1.0, + = line to the left |
| 81–83 | last barcode | 3 | command: 0 none, 1 left, 2 right, 3 straight, 4 U-turn, 5 unmapped |
| 84–86 | reject reason | 3 | 1 count, 2 frame, 3 ratio, 4 pattern, 5 overrun |
| 87–93 | max hump peak | 7 | mm |
| 94–103 | obstacle width | 10 | mm |
| 104–112 | obstacle closest | 9 | mm |
| 113–117 | obstacle bearing | 5 | ×10° (9 = ahead) |
| 118–119 | bypass plan | 2 | continue, left, right, reverse |
| 120–134 | distance | 15 | mm, saturating |
| 135–137 | motion class | 3 | stationary, driving, accelerating, turning, climbing, descending, impact |
| 138–147 | turn rate | 10 s | °/s (kinematic) |
| 148–163 | fault mask | 16 | `docs/FAULTS.md` |
| 164–167 | scan state | 4 | O0–O11 |
| 168–177 | pitch | 10 s | 0.1°, relative to the calibrated level, + = nose up (version 2) |
| 178–185 | roll | 8 s | °, + = left side up (version 2) |
| 186–191 | hump height now | 6 | mm above flat while on a hump, else 0 (version 2) |

## Command byte

`[1:0]` command: 0 START, 1 STOP, 2 RESET, 3 END. `[7:2]` sequence 0–63:
a repeat of the previous sequence number is ignored (QoS 1 duplicates).
Any payload that is not exactly one byte is rejected.

- **STOP** sets `FAULT_REMOTE_STOP` (critical): motors off from T7 at once.
  NFR6 as reworded: 100 ms counted from T7's receipt. The GP21 button is
  the guaranteed path.
- **RESET** is accepted only when no condition-based critical fault
  remains (`FAULT_RESETTABLE_MASK`).
- **END** is a backup end-of-course marker; the car does not need the
  network to finish (NFR19).

- **RESET** in S2 READY or S9 FINISHED returns to S0 and recalibrates
  (the console's `cal` after a run); in other running states it is ignored.

Example: `python3 tools/decode_telemetry.py --cmd 192.168.1.10 START 1`,
or use the console above.
