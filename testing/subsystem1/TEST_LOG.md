# Subsystem 1 — test log

**Subsystem:** Mission state machine (T6), telemetry and MQTT (T7)  ·  **Student ID:** 2501984  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.
*Method:* **auto** = `TEST_CHECK` in `subsystem1_test.c` (`make TEST=1`);
**host** = `tests/host/run.sh` on a PC; **manual** = observed on the car.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | record starts with version and sequence | FR21, R8 | auto | subsystem1_test.c | `[TEST] S1-U01 PASS` |
| S1-U02 | telemetry fields survive pack/unpack | FR21, CR1 | auto | subsystem1_test.c | `[TEST] S1-U02 PASS` |
| S1-U03 | T7 queues 21-byte records | IF-08 | auto | subsystem1_test.c | `[TEST] S1-U03 PASS` |
| S1-U04 | bridge hands over one record | IF-08 | auto | subsystem1_test.c | `[TEST] S1-U04 PASS` |
| S1-U05 | mission word holds a valid state | IF-12 | auto | subsystem1_test.c | `[TEST] S1-U05 PASS` |
| S1-H01 | Python decoder matches C packer field by field; unknown version rejected | FR23, R8 | host | tests/host/run.sh | `[PY] decoder self-test PASS` |
| S1-M01 | MQTT START/STOP/RESET/END and duplicate sequence | FR22, NFR6 | manual | `decode_telemetry.py --cmd` | transitions per docs/MQTT.md; duplicate ignored |
| S1-M02 | complete run with WiFi off | NFR19 | manual | offline image | course completed |
| S1-M03 | command at barcode executes at junction; expiry after 500 mm | FR7 | manual | course | per tribunal test plan |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-31 | S1-H01 | (uncommitted) | Claude (host) | PASS | `testing/host/2026-10-31_host.txt` | host build, no hardware |
| 2026-10-31 | S1-SIM | (uncommitted) | Claude (simulator) | see report | `testing/SIMULATION_REPORT.md` | host simulator, 53 scenarios x 5 seeds; on-target checks 30/30 in the model; no hardware |
