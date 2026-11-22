# Pre-hardware test report, 2026-10-31

Everything here ran on a PC. **Nothing has run on the car.** The simulator
(`tests/sim`, see its README) runs the unchanged `app_program/` sources
against a model of micro T-Kernel, the RP2040 registers and the car. Its
motor, sensor and course constants are estimates, so results about
*logic and timing* are strong evidence; results about *tuning* are not.

## 1. What was run

| Layer | How | Result |
|---|---|---|
| Firmware builds | `make CONSOLE=uart WIFI=none`, `make TEST=1`, `make NET=lwip MQTT=1`, `make` | all build, 0 warnings in `app_program/` |
| Strict warnings | every app file, `-Wall -Wextra -Wshadow -Wconversion -Wsign-conversion` | 0 (two `TMO`→`RELTIM` casts fixed) |
| Host unit tests | `tests/host/run.sh`, H-01…H-10 | 10/10; Python decoder 22/22 fields |
| On-target tests | `make TEST=1` checks run inside the simulator | 30/30 (two test bugs fixed, below) |
| Scenarios | 53 scenarios × 5 seeds (`./build/sim all 5`) | see §3 |
| Sanitizers | ASan + UBSan build, key scenarios | no memory or UB errors |

New host tests: H-07 every bit-helper width and value; H-08 10 000 random
telemetry snapshots field by field; H-09 200 000 random width arrays never
decode to a bad symbol; H-10 reversed symbols under ±15 % jitter never
decode to a wrong character.

## 2. Defects found and fixed

| # | Defect (how it showed) | Fix |
|---|---|---|
| 1 | T6 bounced S3↔S8 without end when T2 left "lost" via a junction: queue flooded, telemetry starved | `set_lstate()` in T2 clears LINE_LOST on every exit from L3 |
| 2 | Same bounce for up to 10 ms at every line loss (100+ `ERROR` lines) | T6 checks the flag, not the 10 ms line word |
| 3 | Every obstacle planned REVERSE: own echoes counted as walls 16 mm away | own hits excluded from clearance |
| 4 | Backward moves spun the car in place (skew correction sign) | correction applied along each wheel's direction |
| 5 | A START pressed mid-run restarted the car at the END marker | latched START/RESET cleared on entering S1/S2/S9/S10 |
| 6 | END detected once per power-up | re-armed when the black area ends |
| 7 | False hump at every start (take-off read as 7°) | hump confirmed over 25 mm of climb; height now 38/40 mm and 19/20 mm |
| 8 | Dead encoder never detected while steering | encoder watch reset only by counter movement |
| 9 | Reversed motor or encoder: car spun for ever in calibration | wrong-direction and no-progress checks → ENCODER fault |
| 10 | Mounting detection flipped with 3 mm / 3° placement error | decided from the sweep (both sensors dark at once) |
| 11 | Barcode bar under the right line sensor read as a junction | junction needs 12 mm (bars ≤ 9 mm, line 18 mm) |
| 12 | Three boxes abreast: bypass drove into a side box | scan-limit side not clear; sonar watches forward bypass legs |
| 13 | Obstacle offset left still bypassed on the left | both clear → shorter way round |
| 14 | Leaving S7, T4 could drop T6's FOLLOW (race) | state published before the request |
| 15 | Queues held 3 and 1 instead of 4 and 2 | kernel size accounting |
| 16 | Line end at a gap: car left at −16° (30 mm full-lock escape) | `ESCAPE_MM` 16 |
| 17 | Test S3-U08 corrupted an element that was already wide; S1-U04 raced the MQTT task | tests fixed |

## 3. Scenario results (5 seeds each, final firmware)

All 5/5: boot, calibrate, straight, curves (R300–R500), tight curve (R200),
crossing straight through, hump 40 / 20 / none, obstacle centred / offset /
wall / boxed-in / corridor / beside, line gap, line ends → LINE_NOT_FOUND,
line step 20 mm, E-STOP (motors off in the same interrupt), MQTT STOP
(off in 1 ms; requirement 100 ms), MQTT duplicates and malformed payloads,
WiFi loss, IMU absent / dies / flaky, impact, stall, dead encoder, dead
sonar, stuck ADC, inverted IR polarity, straddle mounting, motor mismatch,
reversed motor, reversed encoder, stray START, E-STOP bounce, E-STOP during
bypass, CPU load ×5, 300 s endurance (64 m, odometry wrap and saturation),
full course with bars across the line, slow servo, barcode across the line.

Known limitations, kept as expected-failure scenarios: line resuming 60 mm
to the side (BETWEEN mounting drives 250 mm blind before searching), walls
parallel to the path (the sonar cannot see them at grazing angles),
mirrored servo.

**Not passing: barcodes beside the line.** With bars assumed 16–40 mm right
of the line centre: A 2/5, B 1/5, C 3/5, D 3/5, other 2/5, expiry 2/5,
C-far 3/5, so the full course (beside-line bars) 0/5 and the geometry-error
run 2/5. The same courses pass 5/5 with the bars across the line.

Why: the barcode sensor (20 mm right of centre) is only 6.5 mm from the
right line sensor (13.5 mm). With the line between the line sensors the car
cannot sense offsets under ±4.5 mm, and it weaves about ±8° (heading RMS
5°). The barcode sensor, 70 mm ahead of the axle, therefore drifts on and
off the inner ends of the bars and misses a narrow bar. Bars starting at
22 mm decode 0 %. Retuning the steering helps (with gains 100/150 *and* an
8 mm escape window the weave fell to 1.4° RMS) but the 8 mm window lost the
line on R350 bends, so it was not kept.

## 4. Before the car runs

1. **Measure where the barcode bars are** relative to the line. If they are
   beside the line, move the barcode sensor further out (≥ 30 mm) or make
   the bars cross the line; then rerun `SIM_BAR=in,out ./build/sim all 5
   barcode_`.
2. Tune steering on the car (new starting values 100/150 came from the
   model) and the escape window (8–16 mm).
3. Time the servo: the firmware allows exactly 600°/s with no margin.
4. Measure the provisional constants in `docs/DECISIONS.md` §2.
