/** @file drv_servo.h
 *
 * @brief SG90 scan servo on GP12 (servo port 1, PWM slice 6 A), 50 Hz.
 *        Owned by T5.  90 degrees = straight ahead, 0 = right, 180 = left.
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <stdint.h>

/* Pulse range from docs/WIRING.md section 6: a STARTING range.  Check
   the real travel against a protractor (A9) and widen toward 500-2500 us
   if 30 and 150 degrees are not reached. */
#define SERVO_MIN_US        1000U
#define SERVO_MAX_US        2000U
#define SERVO_CENTRE_DEG    90U

/* 1 if a larger pulse turns the sensor to the RIGHT on the real mount.
   Verify at the bench: a wrong value mirrors every bypass decision. */
#define SERVO_MIRROR        0

/* The kit's servo is an FS90R CONTINUOUS-rotation servo (owner,
   2026-11-02): its pulse sets a speed, not an angle.  With SERVO_SCAN 0 it
   is held at SERVO_STOP_US (front: face the sonar straight ahead by hand,
   power off), and turned only by TIMED moves within +-SERVO_LIMIT_DEG:
   slowly (SERVO_STOP_US +- SERVO_CONT_DELTA_US) for degrees /
   SERVO_CONT_DPS seconds.  There is no position feedback, so the angle is
   approximate; every look is symmetric (right, left twice as far, right)
   so timing errors cancel.  Tune SERVO_CONT_DPS on the car: time a slow
   full turn.  Build with SERVO_SCAN=1 for a 180-degree positional servo
   (SG90) and the full scan. */
#define SERVO_STOP_US       1500U   /* FS90R neutral: no rotation        */
#define SERVO_LIMIT_DEG     15U     /* owner: 15 left, 15 right at most  */
#define SERVO_CONT_DELTA_US 100U    /* slow: 1400 / 1600 us              */
#define SERVO_CONT_DPS      120U    /* deg/s at that speed: ESTIMATE     */

/* Radar sweep (continuous servo only): +-SERVO_LIMIT_DEG, left-right-left,
   at SERVO_STOP_US +- SERVO_SWEEP_DELTA_US.  Each swing is TIMED by a 5 ms
   cyclic handler, so every swing is exactly as long as the others and a
   stop always lands on the front.  If the servo turns faster one way than
   the other the sweep's centre creeps sideways: add SERVO_SWEEP_TRIM_US
   (+ = favour left) until it stays centred. */
#define SERVO_SWEEP_DELTA_US 70U    /* slow and smooth                   */
#define SERVO_SWEEP_DPS     70U     /* deg/s at that speed: ESTIMATE     */
#ifndef SERVO_SWEEP_TRIM_US
#define SERVO_SWEEP_TRIM_US 0
#endif
#ifndef SERVO_SCAN
#define SERVO_SCAN          0
#endif

/* Degrees the sonar points LEFT of straight ahead when the servo is at
   its electrical centre (horn fitted off by that much).  Every requested
   angle is shifted to compensate; requests beyond the servo's travel are
   clamped (with more than ~30 degrees the right-hand side of the scan is
   lost).  Build with: make SERVO_TRIM=<deg>.  Negative = points right. */
#ifndef SERVO_TRIM_DEG
#define SERVO_TRIM_DEG      0
#endif

void     servo_init(void);

/* Move and return the settle time (ms) the move needs: 50 ms per 30
   degrees of travel (A9 working figure), at least 20 ms. */
uint32_t servo_move(uint32_t degrees);

/* Radar sweep on/off (continuous servo; no-op with SERVO_SCAN 1).  Off
   returns once the servo has stopped at the front (< 0.5 s). */
void     servo_sweep(bool b_on);

#endif /* DRV_SERVO_H */

/*** end of file ***/
