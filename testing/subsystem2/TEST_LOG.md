# Subsystem 2 — test log

**Subsystem:** Motion control (T4), encoders, GPIO dispatcher  ·  **Student ID:** 2503058  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.
*Method:* **auto** = `TEST_CHECK` in `subsystem2_test.c` (`make TEST=1`);
**host** = `tests/host/run.sh` on a PC; **manual** = observed on the car.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | motion_req_t packs command/argument/source | IF-02, CR1 | auto | subsystem2_test.c | `[TEST] S2-U01 PASS` |
| S2-U02 | argument saturates, never spills | CR1 | auto | subsystem2_test.c | `[TEST] S2-U02 PASS` |
| S2-U03 | mbf_motion carries 2-byte requests | IF-02 | auto | subsystem2_test.c | `[TEST] S2-U03 PASS` |
| S2-U04 | T4 runs every 20 ms | NFR3 | auto | subsystem2_test.c | `[TEST] S2-U04 PASS` |
| S2-U05 | motors disarmed until a request | FR24 | auto | subsystem2_test.c | `[TEST] S2-U05 PASS` |
| S2-M01 | edges per revolution (A2) | A2 | manual | roll 1 m, 5x each way | record value, update subsystem2.h |
| S2-M02 | 2 m straight distance error | NFR11 | manual | tape | < 5 % |
| S2-M03 | 90 deg turn | NFR12 | manual | protractor, 10 turns | within ±5° |
| S2-M04 | E-STOP during M3/M4, PWM sampled 10x | NFR6, R9 | manual | GP21 mid-turn | both motors off < 100 ms |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| | | | | | | |
| 2026-10-31 | S2-SIM | (uncommitted) | Claude (simulator) | see report | `testing/SIMULATION_REPORT.md` | host simulator, 53 scenarios x 5 seeds; on-target checks 30/30 in the model; no hardware |
