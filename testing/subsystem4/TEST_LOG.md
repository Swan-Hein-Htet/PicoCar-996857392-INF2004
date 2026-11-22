# Subsystem 4 — test log

**Subsystem:** IMU terrain (T3), safety monitor (T1), fault mask  ·  **Student ID:** 2500136  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.
*Method:* **auto** = `TEST_CHECK` in `subsystem4_test.c` (`make TEST=1`);
**host** = `tests/host/run.sh` on a PC; **manual** = observed on the car.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | WiFi loss is not critical | NFR19 | auto | subsystem4_test.c | `[TEST] S4-U01 PASS` |
| S4-U02 | fault_clear removes only its bit | R2 | auto | subsystem4_test.c | `[TEST] S4-U02 PASS` |
| S4-U03 | 20 000 fault RMWs with ISRs running lose no bit | R2, R5 | auto | subsystem4_test.c | `[TEST] S4-U03 PASS` |
| S4-U04 | signed fields round-trip and saturate | CR1 | auto | subsystem4_test.c | `[TEST] S4-U04 PASS` |
| S4-U05 | app_math accuracy | D9 | auto | subsystem4_test.c | `[TEST] S4-U05 PASS` |
| S4-U06 | T3 publishes a valid IMU state | IF-11 | auto | subsystem4_test.c | `[TEST] S4-U06 PASS` |
| S4-H01 | app_math within stated bounds; signed packing | D9, CR1 | host | tests/host/run.sh | PASS |
| S4-M01 | hump peak vs measured, 10 hand pushes + 150/250 mm/s | NFR13, A6 | manual | known hump | ± 5 mm |
| S4-M02 | IMU board marking photographed | A4 | manual | photo | LSM303DLHC / GY-511 |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-31 | S4-H01 | (uncommitted) | Claude (host) | PASS | `testing/host/2026-10-31_host.txt` | host build |
| 2026-10-31 | S4-SIM | (uncommitted) | Claude (simulator) | see report | `testing/SIMULATION_REPORT.md` | host simulator, 53 scenarios x 5 seeds; on-target checks 30/30 in the model; no hardware |
