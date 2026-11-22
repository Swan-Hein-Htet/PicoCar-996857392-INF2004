# Consolidated fault table (tutor review, bullet 9)

One row per bit of `g_faults` (`subsystem4.h`). A **critical** fault forces
both motors off immediately from the detector's context, sets flg_sys
ESTOP, and T1 re-asserts both every 20 ms. The mission enters S10 and stays
there until RESET (GP20 button in S10, or MQTT RESET).

| Bit | Name | Detector | Condition | Critical | Cleared by |
|---|---|---|---|---|---|
| 0 | ESTOP_REQ | ISR_BTN | GP21 pressed | yes | RESET |
| 1 | MOTOR_STALL | T4 | duty > 60 %, speed < 20 mm/s, 500 ms | yes | RESET |
| 2 | ENCODER | T4 | a driven wheel's counter frozen for 25 driven periods while the other wheel made >= 75 edges; or moving > 40 mm/s against its set-point at > 60 % duty for 500 ms; or a turn/move with no progress for 1 s (motor or encoder wired backwards) | yes | RESET |
| 3 | IMU | T3 | init/readback failed, or 3 I2C errors in a row | **no** | T3 re-init + 2 s bias OK |
| 4 | IMPACT | T3 | **off** (D75: IMPACT_STOP 0; was \|a\| > 3 g for two samples). The car stops for lift-off (LIFTED) instead | yes | RESET |
| 5 | NO_ECHO | T5 | no echo edge for 5 pings | yes | T5, first echo edge |
| 6 | BYPASS_FAIL | T5 / T6 | second blocked attempt, or S7 > 30 s | yes | RESET |
| 7 | WIFI_LOSS | T7 | MQTT not connected (MQTT builds) | **no** | T7, link up |
| 8 | LINE_NOT_FOUND | T6 | S8 search pattern or 8 s exhausted | yes | RESET |
| 9 | SELF_TEST | T2 | calibration span < 300 counts, or ADC timeout | yes | RESET (recalibrates) |
| 10 | REMOTE_STOP | T7 | MQTT STOP (source marker vs. bit 0) | yes | RESET |
| 11 | TASK_OVERRUN | T1 | T4 heartbeat or T2 sequence frozen 100 ms | yes | RESET |
| 12 | LIFTED | T3 | car picked up (end of the demo): Z above flat gravity by 300 mg for 30 ms (50 ms average), or rolled 25 deg / pitched 35 deg for 200 ms | yes | RESET |
| 12–15 | reserved | | | | |

RESET clears bits 0, 1, 2, 4, 6, 8, 9, 10, 11; it is refused while a
condition-based critical fault (bit 5) is still present.
