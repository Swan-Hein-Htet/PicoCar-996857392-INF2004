# Subsystem 5 — test log

**Subsystem:** Ultrasonic scanning, obstacle profiling and bypass (T5)  ·  **Student ID:** 2500803  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.
*Method:* **auto** = `TEST_CHECK` in `subsystem5_test.c` (`make TEST=1`);
**host** = `tests/host/run.sh` on a PC; **manual** = observed on the car.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | ISR_ECHO completes pings via the dispatcher | A5 | auto | subsystem5_test.c | `[TEST] S5-U01 PASS` |
| S5-U02 | a valid echo word never holds a no-return width | Version C | auto | subsystem5_test.c | `[TEST] S5-U02 PASS` |
| S5-U03 | mtx_state copy-out hold < 100 µs | NFR17 | auto | subsystem5_test.c | `[TEST] S5-U03 PASS` |
| S5-U04 | profile carries a valid scan state | IF-04 | auto | subsystem5_test.c | `[TEST] S5-U04 PASS` |
| S5-M01 | ECHO divider 3.0–3.6 V; servo V+ under motor load | A5 | manual | meter | in range |
| S5-M02 | servo travel 30/90/150 deg and direction | A9 | manual | protractor | set SERVO_MIRROR / pulse range |
| S5-M03 | width of 50/100/150 mm blocks | NFR22 | manual | 10x each | ± 10 % |
| S5-M04 | bypass: no contact, ≥ 50 mm, < 5 s | NFR14, NFR23 | manual | course | as stated |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| | | | | | | |
| 2026-10-31 | S5-SIM | (uncommitted) | Claude (simulator) | see report | `testing/SIMULATION_REPORT.md` | host simulator, 53 scenarios x 5 seeds; on-target checks 30/30 in the model; no hardware |
