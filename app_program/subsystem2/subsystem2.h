/** @file subsystem2.h
 *
 * @brief Subsystem 2 -- Motion control: motors, encoders, wheel PID,
 *        odometry, the motion request API and the shared GPIO interrupt
 *        dispatcher.  Buddy 2, Charmaine Wong Shi Qi, 2503058.
 *
 * Public interface of subsystem 2.  Other subsystems include it as
 * "../subsystem2/subsystem2.h".
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

/* ---------------------------------------------------------------------
 *  Robot geometry and encoder scale.  PROVISIONAL (assumption A2):
 *  estimated from the kit photos and the motor sheet (PH2.0-6P hall
 *  encoder motor, 65 mm wheel).  Replace with the measured values from
 *  the 1 m roll test before PID tuning; every derived figure follows.
 * --------------------------------------------------------------------- */
#define MOTION_WHEEL_DIA_MM     65.0f   /* photo: standard 65 mm wheel   */
#define MOTION_TRACK_MM         130.0f  /* wheel centre to wheel centre  */
#define MOTION_EDGES_PER_REV    1056.0f /* phase A both edges: 2x11x48   */
#define MOTION_ENC_SIGN_L       (-1)    /* MEASURED 2026-11-02 (wheel test):
                                           left forward -708, backward
                                           +1254 counts */
#define MOTION_ENC_SIGN_R       1       /* right encoder counts up going
                                           forward (owner 2026-11-02:
                                           with -1 here AND -1 for the
                                           motor the right wheel ran
                                           backwards, unnoticed) */
#define MOTION_MOTOR_SIGN_L     1       /* M1A drives the left wheel
                                           forward (confirmed in the
                                           same log)                   */
#define MOTION_MOTOR_SIGN_R     1       /* M2A drives the right wheel
                                           forward (owner, 2026-11-02) */

/* Speed limits (mm/s) used by the mission states. */
/* 100 mm/s throughout at the owner's request, 2026-11-10, for barcode
   reading (was 200/160/120/100/120; before that 250/200/150/120/150). */
#ifndef SPEED_FOLLOW
#define SPEED_FOLLOW            100U
#endif
#define SPEED_BARCODE           100U
#define SPEED_HUMP              100U
#define SPEED_CREEP             100U
#define SPEED_TURN              70U     /* in-place turns (was 100: brown-out mid-turn, D77) */
#define SPEED_MAX               350U
/* Manual "plaid" mode and stunts (console manual page, owner 2026-11-16):
   about half the motors' full-duty speed, so the speed loop keeps room to
   correct and the car stays straight. */
#ifndef SPEED_PLAID
#define SPEED_PLAID             250U
#endif
#ifndef WHEEL_ACCEL_MMS2
#define WHEEL_ACCEL_MMS2        400U    /* per-wheel set-point slew, MREQ_WHEELS:
                                           1200 and 800 overshot plaid by
                                           20-40 % (speed-loop integral
                                           wound up while ramping, sim) */
#endif

/* ---------------------------------------------------------------------
 *  motion_req_t: one uint16_t (Change Requirement 1)
 *    [2:0]   command (MREQ_x)
 *    [14:3]  argument 0-4095: mm, degrees or mm/s
 *    [15]    source: 0 = T6 VehicleCtrl, 1 = T5 ObstacleScan
 *  Copied whole by mbf_motion, so it is never shared in place.
 *  T4 rejects any request whose source is not the active owner
 *  (T5 while the mission is in S7, T6 otherwise).
 * --------------------------------------------------------------------- */
typedef uint16_t motion_req_t;

#define MREQ_STOP               0U  /* ramp down and hold                 */
#define MREQ_FORWARD            1U  /* arg = distance, mm                 */
#define MREQ_BACKWARD           2U  /* arg = distance, mm                 */
#define MREQ_TURN_LEFT          3U  /* arg = angle, deg, in place         */
#define MREQ_TURN_RIGHT         4U  /* arg = angle, deg, in place         */
#define MREQ_SET_SPEED          5U  /* arg = cruise speed, mm/s           */
#define MREQ_FOLLOW             6U  /* arg = speed, mm/s; steer on line   */
#define MREQ_WHEELS             7U  /* follow g_wheel_word (manual only)  */

#define MREQ_CMD_SHIFT          0U
#define MREQ_CMD_BITS           3U
#define MREQ_ARG_SHIFT          3U
#define MREQ_ARG_BITS           12U
#define MREQ_SRC_SHIFT          15U
#define MREQ_SRC_T6             0U
#define MREQ_SRC_T5             1U

static inline motion_req_t
motion_req_make (uint32_t cmd, uint32_t arg, uint32_t src)
{
    uint32_t word = 0U;

    word = bits_put(word, MREQ_CMD_SHIFT, MREQ_CMD_BITS, cmd);
    word = bits_put(word, MREQ_ARG_SHIFT, MREQ_ARG_BITS,
                    bits_sat_u(arg, MREQ_ARG_BITS));
    word = bits_put(word, MREQ_SRC_SHIFT, 1U, src);
    return ((motion_req_t)word);
}

/* Non-blocking request API (FR10).  Returns E_OK or a kernel error
   (E_TMOUT if the 4-deep queue is full). */
IMPORT ER motion_request(uint32_t cmd, uint32_t arg, uint32_t src);

/* Drain every queued request (T6 calls this at the S3->S7 handover; the
   source bit is the real guarantee, this is defence in depth). */
IMPORT void motion_flush(void);

/* Force both motors off (brake) from any context, including ISRs.  The
   one documented exception to T4's ownership of the PWM (used by T1 and
   the fault helpers).  Disarms the outputs until T4 re-arms them. */
IMPORT void motion_force_off(void);

/* ---------------------------------------------------------------------
 *  Shared single-writer words (rule R1)
 *
 *  g_motion_word (uint32, writer T4)
 *    [9:0]   left wheel speed,  mm/s, signed 10 bits
 *    [19:10] right wheel speed, mm/s, signed 10 bits
 *    [22:20] motion state M0-M7
 *    [23]    outputs armed
 *    [27:24] heartbeat, increments every T4 period (T1 liveness check)
 *    [30:28] last accepted request command
 *
 *  g_odom_word (uint32, writer T4)
 *    [15:0]  distance travelled, mm, WRAPPING (consumers subtract)
 *    [30:16] total distance, mm, saturating at 32767 (telemetry)
 *    [31]    total distance saturated
 *
 *  g_enc_left / g_enc_right (uint16 each, writer: its own encoder ISR)
 *    free-running edge counters; T4 subtracts, never clears (rule R3).
 * --------------------------------------------------------------------- */
#define MOTION_SPEED_L_SHIFT    0U
#define MOTION_SPEED_R_SHIFT    10U
#define MOTION_SPEED_BITS       10U
#define MOTION_STATE_SHIFT      20U
#define MOTION_STATE_BITS       3U
#define MOTION_ARMED_SHIFT      23U
#define MOTION_BEAT_SHIFT       24U
#define MOTION_BEAT_BITS        4U
#define MOTION_LASTCMD_SHIFT    28U

#define ODOM_WRAP_SHIFT         0U
#define ODOM_WRAP_BITS          16U
#define ODOM_TOTAL_SHIFT        16U
#define ODOM_TOTAL_BITS         15U
#define ODOM_SAT_SHIFT          31U

#define MSTATE_DISARMED         0U  /* M0 */
#define MSTATE_IDLE             1U  /* M1 */
#define MSTATE_DRIVE            2U  /* M2 */
#define MSTATE_TURN             3U  /* M3 */
#define MSTATE_MOVE             4U  /* M4 */
#define MSTATE_RAMP_DOWN        5U  /* M5 */
#define MSTATE_HOLD             6U  /* M6 */
#define MSTATE_STALL            7U  /* M7 */

IMPORT volatile uint32_t g_motion_word;
IMPORT volatile uint32_t g_odom_word;
/* ---------------------------------------------------------------------
 *  g_wheel_word (uint32, single writer T6, rule R1): per-wheel targets
 *  for MREQ_WHEELS, mm/s, two's complement.  T4 follows it until the next
 *  request; equal targets get the straight-line correction of a move.
 *    [11:0]  left    [23:12] right
 * --------------------------------------------------------------------- */
#define WHEEL_L_SHIFT           0U
#define WHEEL_R_SHIFT           12U
#define WHEEL_BITS              12U
#define WHEEL_RAW_BIT           24U     /* set: L/R are motor power in
                                           permille, no speed loop, no
                                           slew (drift and donut stunts) */
IMPORT volatile uint32_t g_wheel_word;
IMPORT volatile uint16_t g_enc_left;
IMPORT volatile uint16_t g_enc_right;

/* Wrapping distance (mm) now; consumers keep the previous value and use
   odom_delta_mm() so the result is right across the wrap. */
static inline uint16_t
odom_now_mm (void)
{
    return ((uint16_t)bits_get(g_odom_word, ODOM_WRAP_SHIFT, ODOM_WRAP_BITS));
}

static inline uint32_t
odom_delta_mm (uint16_t now, uint16_t before)
{
    return ((uint32_t)(uint16_t)(now - before));
}

/* ---------------------------------------------------------------------
 *  Shared GPIO interrupt dispatcher.  pico-sdk style "one callback per
 *  core": every GPIO interrupt source in the system is registered here
 *  and the single IO_IRQ_BANK0 handler calls each in turn.
 * --------------------------------------------------------------------- */
#define GPIO_EDGE_FALL          0x4U
#define GPIO_EDGE_RISE          0x8U

typedef void (* gpio_handler_t)(uint32_t pin, uint32_t events,
                                uint32_t now_us);

IMPORT ER   gpio_irq_register(uint32_t pin, uint32_t events,
                              gpio_handler_t handler);
IMPORT ER   gpio_irq_start(void);
IMPORT UW   g_gpio_irq_count;   /* dispatcher entries, for the tests */

/* Private to subsystem 2 */
#ifndef S2_T4_PERIOD_MS
#define S2_T4_PERIOD_MS         10U     /* = T2's line-word rate: 20 ms
                                           used every other sample and
                                           the car slithered (2026-11-03) */
#endif
IMPORT ID   mbf_motion;
IMPORT ID   tid_motion;
IMPORT void motion_task(INT stacd, void * p_exinf);

#endif /* SUBSYSTEM2_H */

/*** end of file ***/
