# Team and subsystems — P5-IS20

Single place that says **who owns what**. Pins follow `docs/WIRING.md`,
which is fixed: change the wiring document first, then this table.

Rule: **only the owner edits a subsystem's folders.** Shared files
(`app_program/app.h`, `app_bits.h`, `app_math.*`, `app_main.c`, this file,
`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`) change only by agreement.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | Owner | Code | Tests |
|---|---|---|---|---|---|
| 1 | Mission state machine (T6), WiFi/MQTT command + telemetry (T7) | 2501984 | Swan Hein Htet | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion control (T4), motors, encoders, GPIO IRQ dispatcher, `motion_force_off()` | 2503058 | Charmaine Wong Shi Qi | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | IR line following + Code 39 barcode (T2) | 2500226 | Yun Cong | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU terrain (T3), safety monitor (T1), fault mask | 2500136 | Toh You Yang | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Ultrasonic scan, obstacle profile, bypass, reacquire (T5) | 2500803 | Danish Hazwan | `app_program/subsystem5/` | `testing/subsystem5/` |

## 2. Pins (from docs/WIRING.md — do not change)

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 / GP1 | — | UART0 console only with `CRASH_REPORT=1` / `CONSOLE=uart` |
| GP2 | S5 (T5) | HC-SR04 TRIG (Grove 2) |
| GP3 | S5 (ISR_ECHO) | HC-SR04 ECHO via 10 kΩ / 15 kΩ divider |
| GP4 / GP5 | S4 (T3) | I2C0 SDA / SCL, GY-511 LSM303DLHC @0x19 (Grove 3) |
| GP6 | — | unused |
| GP7 | S4 (T1) | carrier status LED (Grove 7 GP7 contact left open) |
| GP8 / GP9 | S2 (T4; T1 force-off) | M1A / M1B left motor, PWM slice 4, 20 kHz |
| GP10 / GP11 | S2 (T4; T1 force-off) | M2A / M2B right motor, PWM slice 5, 20 kHz |
| GP12 | S5 (T5) | SG90 servo, servo port 1, PWM slice 6, 50 Hz |
| GP13 | S2 (ISR_ENC_L) | left encoder phase B (GPIO header; servo port 2 unusable) |
| GP14 / GP15 | — | unused |
| GP16 | S2 (ISR_ENC_L) | left encoder phase A (Grove 4) |
| GP17 | S2 (ISR_ENC_R) | right encoder phase A (Grove 4) |
| GP18 | — | onboard RGB LED, not driven |
| GP19 | S2 (ISR_ENC_R) | right encoder phase B (GPIO header) |
| GP20 | S1 (ISR_BTN) | START button (also RESET in S10) |
| GP21 | S1 (ISR_BTN) | E-STOP button |
| GP22 | — | buzzer, not driven |
| GP26 | S3 (T2) | ADC0 right line sensor A0 (Grove 5) |
| GP27 | S3 (T2) | ADC1 left line sensor A0 (Grove 6) |
| GP28 | S3 (T2) | ADC2 barcode sensor A0 (Grove 7, VBAT jumper open) |

## 3. Peripherals

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| I2C0 | S4 | T3 ImuTerrain (own register driver, kernel IIC driver stays off) | GP4, GP5 |
| ADC | S3 | T2 LineBarcode (own register driver, 1 kHz) | GP26–GP28 |
| PWM slices 4, 5 | S2 | T4 MotionCtrl (+ documented force-off by T1/fault helpers) | GP8–GP11 |
| PWM slice 6 | S5 | T5 ObstacleScan | GP12 |
| IO_IRQ_BANK0 | S2 | dispatcher, enabled by T4 on core 1 | GP3, 16, 17, 20, 21 |
| Watchdog | S4 | T1 SafetyMonitor (not in `TEST=1` builds) | — |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (core 0) via `net_bridge.c` | GP23–25, GP29 |

## 4. Task priorities (1 highest; USB console 3 and CYW43 4 are on core 0)

All application tasks are pinned to **core 1** (`APP_CORE`) so no kernel
service task can pre-empt a control loop. Kernel tick: **1 ms**.

| Task | Subsystem | Priority | Release | Core |
|---|---|---|---|---|
| T1 SafetyMonitor | 4 | 2 | 20 ms cyclic waker | 1 |
| T2 LineBarcode | 3 | 6 | 1 ms cyclic waker | 1 |
| T3 ImuTerrain | 4 | 7 | 10 ms cyclic waker | 1 |
| T4 MotionCtrl | 2 | 8 | 20 ms cyclic waker | 1 |
| T5 ObstacleScan | 5 | 10 | 70 ms ping, woken by ISR_ECHO | 1 |
| T6 VehicleCtrl | 1 | 12 | event flag + 50 ms timeout | 1 |
| T7 Telemetry | 1 | 20 | 200 ms (absolute) | 1 |

T1 sits above the template's per-subsystem bands by team agreement
(tribunal risk R6: it must be above every kernel service task).

## 5. Interfaces between subsystems

| ID | From → To | Mechanism | Object | Data / meaning |
|---|---|---|---|---|
| IF-01 | T2 → T6 | message buffer (owned S1) | `mbf_nav` | `nav_cmd_t` (uint16) + `FLG_NAV` |
| IF-02 | T6/T5 → T4 | message buffer (owned S2) | `mbf_motion` | `motion_req_t` (uint16, source bit) |
| IF-03 | all → all | event flag (owned S1) | `flg_sys` | bits in `app.h`, one clearer each |
| IF-04 | T5 → T7 | data under mutex (owned S1) | `g_vehicle_state`, `mtx_state` | obstacle profile, 2 words |
| IF-05 | T2 → T4,T5,T6,T7 | single-writer word | `g_line_word` | `subsystem3.h` |
| IF-06 | T4 → T1,T3,T6,T7 | single-writer words | `g_motion_word`, `g_odom_word` | `subsystem2.h` |
| IF-07 | ISRs → T4, T5 | single-writer halfwords | `g_enc_left/right`, `g_echo_word` | free-running counters / echo |
| IF-08 | T7 → service task | message buffer (owned S1) | `mbf_telem` | 21-byte record (`docs/MQTT.md`) |
| IF-09 | service task → T7 | message buffer (owned S1) | `mbf_cmd` | 1-byte command |
| IF-10 | all → T1, T6, T7 | helper-guarded halfword (owned S4) | `g_faults` | `fault_set/clear/get` |
| IF-11 | T3 → T6, T7 | single-writer words | `g_terrain_word`, `g_hump_byte` | `subsystem4.h` |
| IF-12 | T6 → T4, T5, T7 | single-writer halfword | `g_mission_word` | state, pending command, budget |

## 6. Kernel object budget

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 2 | 1 | 1 | 2 | 1 |
| Event flag | 16 | 1 | | | | |
| Mutex | 12 | 1 | | | | |
| Message buffer | 12 | 3 | 1 | | | |
| Cyclic handler | 12 | | 1 | 1 | 2 | |
| Interrupt handler | — | | 1 (IRQ 13) | | | |
