# PicoCar host simulator

Runs the **real, unchanged** `app_program/` sources on a PC: every task,
driver and ISR, against a model of the kernel, the RP2040 registers and the
car on a course. Each scenario is judged against ground truth from the model
and against the firmware's own shared words.

```
./build.sh                    # -> build/sim
./build/sim list              # scenarios
./build/sim all 5             # every scenario x 5 seeds, one process each
./build/sim run obstacle 3 v  # one run with the console log
SAN=1 ./build.sh              # AddressSanitizer + UBSan -> build/sim_san
APP_TEST=1 ./build.sh && ./build/sim_test run boot 1 v | grep '\[TEST\]'
                              # the on-target make TEST=1 checks, in the model
```

Environment knobs: `SIM_BAR=in,out` (barcode bar extent, mm right of the
line centre; default 16,40), `SIM_SPOT=mm` (IR spot diameter),
`SIM_Y0=mm` (start offset), `SIM_TRACE=n` (pose/words every n ms).
`var_build.sh` builds what-if variants (a firmware file with a sed edit),
`dbg_build.sh` a T2 that logs every barcode capture.

## What is modelled

| Part | Model |
|---|---|
| Kernel (`sim_kernel.c`) | Tasks as threads with one baton: strict priority preemption on one core. Event flags (TA_WMUL), message buffers with the kernel's exact size accounting, mutex with inheritance, cyclic handlers, sleep/wake-up counts, timeouts at tick boundaries, `DI`/`EI`, interrupt 13. Every kernel call and TIMER read costs 2-25 us of virtual time (seeded), so interrupts and higher-priority tasks land in the middle of a task's work. |
| Registers (`sim_hw.c`) | TIMER, RESETS, SIO CPUID (core 1), ADC (start/ready/result), I2C0 DW_apb + LSM303DLHC register file (auto-increment bit, CTRL_REG1/4 scaling, BDU latch, NACK/abort), PWM CC/TOP/DIV, IO_BANK0 INTR/INTE/INTS (write-1-to-clear), WDT LOAD/CTRL, libbsp GPIO/PWM. |
| Car (`sim_world.c`) | Duty from PWM -> first-order wheel speed with deadband and climbing load -> quadrature edges through the GPIO interrupt -> differential-drive pose. IR sensors integrate a 4 mm spot over the course map. Humps as sin^2 height profiles with pitch; accelerometer = gravity + forward and centripetal acceleration + mount tilt + noise. HC-SR04: 9-ray 15 deg beam, no echo beyond 50 deg incidence, 38 ms pulse on no return; servo slews at 600 deg/s. Collisions are detected. Buttons are active-low edges. |
| Network | The CYW43 task is modelled by calls to `net_bridge_*` every 50 ms: records decoded and checked, commands injected. |

Not modelled: real motor and sensor behaviour (the constants are estimates),
cache/flash timing, the USB/CYW43 tasks on core 0, wheel slip.
**A pass here is evidence about logic and timing, not about tuning.**

## Checks on every run

Blocking call with interrupts masked or from an ISR, `ERROR` console lines,
T2 1 ms overruns and release latency, watchdog enabled and fed (gap
< 100 ms), telemetry at 5 Hz with no gaps or malformed records and a
mission state actually held, no unexpected faults, no requests rejected by
T4 except during handovers.
