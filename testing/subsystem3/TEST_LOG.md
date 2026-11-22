# Subsystem 3 — test log

**Subsystem:** IR line following and Code 39 barcode (T2)  ·  **Student ID:** 2500226  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.
*Method:* **auto** = `TEST_CHECK` in `subsystem3_test.c` (`make TEST=1`);
**host** = `tests/host/run.sh` on a PC; **manual** = observed on the car.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | all 43 characters decode | FR5 | auto | subsystem3_test.c | `[TEST] S3-U01 PASS` |
| S3-U02 | decode when read backwards | FR5 | auto | subsystem3_test.c | `[TEST] S3-U02 PASS` |
| S3-U03 | 100–350 mm/s widths decode | NFR8 | auto | subsystem3_test.c | `[TEST] S3-U03 PASS` |
| S3-U04 | speed change mid-symbol | NFR8 | auto | subsystem3_test.c | `[TEST] S3-U04 PASS` |
| S3-U05 | truncated symbol rejected | FR6 | auto | subsystem3_test.c | `[TEST] S3-U05 PASS` |
| S3-U06 | marks after symbol rejected | FR6 | auto | subsystem3_test.c | `[TEST] S3-U06 PASS` |
| S3-U07 | saturated element rejected | FR6 | auto | subsystem3_test.c | `[TEST] S3-U07 PASS` |
| S3-U08 | corrupted start character rejected | FR6 | auto | subsystem3_test.c | `[TEST] S3-U08 PASS` |
| S3-U09 | line word republished | NFR2 | auto | subsystem3_test.c | `[TEST] S3-U09 PASS` |
| S3-U10 | no 1 ms overruns over 1 s | NFR1 | auto | subsystem3_test.c | `[TEST] S3-U10 PASS` |
| S3-H01 | decoder: 43 chars, both directions, 100–350 mm/s | FR5, NFR8 | host | tests/host/run.sh | PASS |
| S3-H02 | 20 000 trials with ±15 % element jitter: no wrong character | NFR9 | host | tests/host/run.sh | 0 wrong |
| S3-M01 | narrow element width of printed symbol (A3) | A3 | manual | callipers | 3 mm ± 0.3 |
| S3-M02 | 20 passes per symbol A–D | NFR9 | manual | course | ≥ 95 %, 0 false |
| S3-M03 | mounting detected at calibration | D12 | manual | console `T2: mounting` | matches the real sensor spacing |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-31 | S3-H01 | (uncommitted) | Claude (host) | PASS | `testing/host/2026-10-31_host.txt` | host build |
| 2026-10-31 | S3-H02 | (uncommitted) | Claude (host) | PASS | `testing/host/2026-10-31_host.txt` | 20000/20000 decoded, 0 wrong |
| 2026-10-31 | S3-SIM | (uncommitted) | Claude (simulator) | see report | `testing/SIMULATION_REPORT.md` | host simulator, 53 scenarios x 5 seeds; on-target checks 30/30 in the model; no hardware |
