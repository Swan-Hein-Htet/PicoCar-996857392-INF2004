/** @file task_motion.c
 *
 * @brief T4 MotionCtrl: per-wheel PID speed control, odometry, line
 *        steering and the M0-M7 motion state machine.  Released every
 *        10 ms by a cyclic handler (NFR3).
 *
 * Speed is the edge count over the last MOTION_SPEED_WINDOW periods
 * (100 ms; re-derive once A2 is measured -- a fine hall encoder allows a
 * shorter window).  Steering in M2 DRIVE turns the line error into a
 * difference between the two wheel set-points; a feed-forward term makes
 * the duty react to it at once while the PID removes the residual.
 *
 * PID integrators, filter states and set-points are task-private
 * arithmetic intermediates and keep native float width: a justified
 * deviation from Change Requirement 1 recorded in
 * docs/DATA_REPRESENTATION.md (never stored, shared or transmitted).
 */

#include "subsystem2.h"
#include "../app_math.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"

#define PI_F                    3.14159265f
#define DT_S                    ((float)S2_T4_PERIOD_MS / 1000.0f)
#define PERIODS(ms)             ((uint32_t)(ms) / S2_T4_PERIOD_MS)
#define MM_PER_EDGE             ((PI_F * MOTION_WHEEL_DIA_MM) /           \
                                 MOTION_EDGES_PER_REV)
#define MOTION_SPEED_WINDOW     PERIODS(100U)   /* speed over 100 ms */

/* PID and steering gains: starting values, tune per the PID report. */
#define PID_FF                  (1.0f / 450.0f) /* duty per mm/s         */
#define PID_KP                  0.0020f
#define PID_KI                  0.0060f
#define PID_I_LIMIT             0.30f           /* duty from the I term  */
#define PID_DEADBAND            0.10f           /* static friction       */
/* Steering, at the 10 ms line rate.  The BETWEEN-mounted pair gives an
   almost on/off error, so the gains set how hard each brush is answered:
   100 / 150-per-20-ms threw one wheel to 0 and the other to 2x on every
   brush (real car 2026-11-03: +-127 error, 40/480 mm/s, slithering).
   Now a small P term answers a brief brush gently, and a leaky integral
   builds the authority a curve needs from sustained contact.  tests/sim,
   6 seeds, straight: wheel speed swing 140 -> 33 mm/s, heading rms
   3.7 -> 1.0 deg; curves and the R200 tight curve unchanged or better. */
#ifndef STEER_KP
#define STEER_KP                40.0f           /* mm/s per unit error   */
#endif
#ifndef STEER_KD
#define STEER_KD                1.0f            /* mm/s per unit/s of
                                                   error change          */
#endif
#ifndef STEER_KI
#define STEER_KI                700.0f          /* mm/s per unit*s.  Was
                                                   200: on sharp bends the
                                                   car turned 11-15 deg/s
                                                   and left the line (log
                                                   2026-11-16); 700 holds
                                                   R60 in tests/sim.  Only
                                                   real contact builds it
                                                   (b_contact below).    */
#endif
#ifndef STEER_LEAK_S
#define STEER_LEAK_S            0.3f            /* integral decay while
                                                   the error is zero     */
#endif
#define STEER_REF_MMS           200.0f          /* KI and leak tuned here */
#define STEER_I_MAX             1.0f            /* unit*s                */
#define STEER_FULL_ERR          0.95f           /* escaped: full lock   */
#ifndef CURVE_SLOW
#define CURVE_SLOW              2.0f            /* speed cut per unit of
                                                   built-up steering     */
#endif
#ifndef STEER_LOCK
#define STEER_LOCK              1.8f            /* steering up to this x the
                                                   forward speed: past 1 the
                                                   inner wheel runs backwards
                                                   (hairpins, D68; was 1) */
#endif
#ifndef TURN_OUTER_MIN
#define TURN_OUTER_MIN          110.0f          /* mm/s: outer wheel at
                                                   least this once the inner
                                                   one reverses            */
#endif
#ifndef ESCAPE_FWD
#define ESCAPE_FWD              0.15f           /* line escaped to one side:
                                                   keep this share of the
                                                   forward speed ...       */
#endif
#ifndef ESCAPE_SPIN_MMS
#define ESCAPE_SPIN_MMS         110.0f          /* ... and spin towards it
                                                   at least this fast      */
#endif
#ifndef ESCAPE_HOLD_S
#define ESCAPE_HOLD_S           0.12f           /* full-scale this long: a
                                                   real escape, not a brush
                                                   (straight/curves/line_step
                                                   slowed otherwise)       */
#endif
#ifndef ESCAPE_SIDE_S
#define ESCAPE_SIDE_S           0.3f            /* ... and only in a bend:
                                                   the line on the same
                                                   side this long.  A wobble
                                                   from side to side (after
                                                   a bypass, every ~0.14 s)
                                                   reached full scale too,
                                                   and spinning made it a
                                                   zig-zag (sim full_course) */
#endif
#ifndef CURVE_MIN
#define CURVE_MIN               0.4f            /* never below this share
                                                   of cruise             */
#endif
#define STRAIGHT_K              4.0f            /* mm/s per edge of skew */
#define LINE_ERR_SCALE          127.0f

#define ACCEL_STEP_MMS          (600.0f * DT_S)  /* set-point ramp     */
#ifndef TURN_ACCEL_MMS2
#define TURN_ACCEL_MMS2         250.0f  /* in-place turns build up gently and
                                           from standstill: one wheel
                                           reversing at full rate browned
                                           the supply out mid-turn (real car
                                           2026-11-19, D77)               */
#endif
#define TURN_STEP_MMS           (TURN_ACCEL_MMS2 * DT_S)
#define STOP_STEP_MMS           (1250.0f * DT_S) /* for STOP          */
#define WHEEL_STEP_MMS          ((float)WHEEL_ACCEL_MMS2 * DT_S)
#define RAMP_MOVE_MM            40.0f   /* decelerate over the last 40 mm */
#define MIN_GOAL_SPEED          60.0f
#define STOPPED_MMS             15.0f
#define HOLD_PERIODS            PERIODS(100U)   /* M6: brake 100 ms  */

#define STALL_DUTY              0.6f
#define STALL_SPEED             20.0f
#define STALL_PERIODS           PERIODS(500U)
#define ENC_FAULT_DUTY          0.3f
#define ENC_FAULT_EDGES         3
#define ENC_FAULT_MOVED         75      /* other wheel's edges meanwhile */
#define WRONG_DIR_MMS           40.0f   /* moving against the set-point  */
#define GOAL_STUCK_PERIODS      PERIODS(1000U)  /* no progress to goal */
#define SCALE_DUTY              0.45f   /* above this much power ...     */
#define SCALE_RATIO             0.33f   /* ... under a third of the speed
                                           that power gives (PID_FF)    */

EXPORT volatile uint32_t g_motion_word = 0U;
EXPORT volatile uint32_t g_odom_word = 0U;
EXPORT volatile uint32_t g_wheel_word = 0U;

typedef struct
{
    uint32_t state;
    uint32_t last_cmd;
    uint32_t beat;
    uint32_t hold;
    uint32_t stall;
    uint32_t still_l;       /* periods driven with a frozen counter */
    uint32_t still_r;
    int32_t  other_l;       /* edges the other wheel made meanwhile */
    int32_t  other_r;
    uint32_t wrong_dir;
    uint32_t too_slow;      /* periods: full power, little speed    */
    uint32_t goal_stuck;    /* periods without progress (M3/M4)    */
    float    goal_best;     /* best progress so far, edges         */
    uint32_t hist_idx;
    uint32_t rejected;
    int32_t  hist_l[MOTION_SPEED_WINDOW];
    int32_t  hist_r[MOTION_SPEED_WINDOW];
    int32_t  dir_l;         /* goal direction per wheel, +1 / -1   */
    int32_t  dir_r;
    int32_t  prog_l;        /* signed edges since the goal started */
    int32_t  prog_r;
    float    goal_edges;
    float    goal_speed;
    float    cruise;        /* commanded cruise speed, mm/s        */
    float    ramp;          /* ramped speed actually used, mm/s    */
    float    speed_l;
    float    speed_r;
    float    integ_l;
    float    integ_r;
    float    line_prev;
    float    escape_s;      /* time the line error has stayed full scale */
    float    side_s;        /* time the line has stayed on one side      */
    float    line_integ;
    float    odom_frac;
    uint32_t odom_wrap;
    uint32_t odom_total;
    uint16_t enc_prev_l;
    uint16_t enc_prev_r;
    bool     b_goal;        /* a goal is running: set MOTION_DONE  */
    bool     b_stopping;    /* M5 entered by STOP, not by a goal   */
    bool     b_wheels;      /* M2 following g_wheel_word, not line */
    float    wheel_l;       /* slewed per-wheel set-points, mm/s   */
    float    wheel_r;
    uint32_t wheel_prev;    /* last g_wheel_word: new = new segment */
    bool     b_slewing;     /* wheel set-points still ramping      */
    bool     b_raw;         /* this step drives power directly     */
    float    raw_l;         /* its motor power, -1 .. 1            */
    float    raw_r;
    float    integ_per_l;   /* speed-loop integral per mm/s, learnt */
    float    integ_per_r;
} motion_ctx_t;

static motion_ctx_t g_mc;

/* ------------------------------------------------------------------ */

static float
clampf (float value, float limit)
{
    float out = value;

    if (out > limit)
    {
        out = limit;
    }
    else if (out < -limit)
    {
        out = -limit;
    }
    else
    {
        /* In range. */
    }
    return (out);
}

static float
pid_step (float set_point, float measured, float * p_integ)
{
    float const err = set_point - measured;
    float       duty;

    *p_integ = clampf(*p_integ + (err * DT_S), PID_I_LIMIT / PID_KI);
    duty = (PID_FF * set_point) + (PID_KP * err) + (PID_KI * (*p_integ));

    if (set_point > 1.0f)
    {
        duty += PID_DEADBAND;
    }
    else if (set_point < -1.0f)
    {
        duty -= PID_DEADBAND;
    }
    else
    {
        duty = 0.0f;        /* zero set-point: brake, no hunting */
        *p_integ = 0.0f;
    }
    return (clampf(duty, 1.0f));
}

static void
reset_control (void)
{
    g_mc.integ_l = 0.0f;
    g_mc.integ_r = 0.0f;
    g_mc.ramp = 0.0f;
    g_mc.stall = 0U;
    g_mc.still_l = 0U;
    g_mc.still_r = 0U;
    g_mc.other_l = 0;
    g_mc.other_r = 0;
    g_mc.wrong_dir = 0U;
    g_mc.too_slow = 0U;
}

static void
goal_done (void)
{
    g_mc.state = MSTATE_HOLD;
    g_mc.hold = 0U;
}

static void
start_goal (uint32_t state, float edges, int32_t dir_l, int32_t dir_r,
            float speed)
{
    g_mc.state = state;
    g_mc.goal_edges = edges;
    g_mc.goal_speed = speed;
    g_mc.dir_l = dir_l;
    g_mc.dir_r = dir_r;
    g_mc.prog_l = 0;
    g_mc.prog_r = 0;
    g_mc.goal_stuck = 0U;
    g_mc.goal_best = 0.0f;
    g_mc.ramp = 0.0f;
    g_mc.b_goal = true;
    g_mc.b_stopping = false;
    g_mc.b_wheels = false;
}

/* ------------------------------------------------------------------ */

static void
apply_request (motion_req_t req)
{
    uint32_t const cmd = bits_get(req, MREQ_CMD_SHIFT, MREQ_CMD_BITS);
    uint32_t const arg = bits_get(req, MREQ_ARG_SHIFT, MREQ_ARG_BITS);
    float const    turn_edges = ((PI_F * MOTION_TRACK_MM * (float)arg) /
                                 360.0f) / MM_PER_EDGE;
    float const    move_edges = (float)arg / MM_PER_EDGE;
    float const    goal_speed = (g_mc.cruise > 1.0f) ?
                                g_mc.cruise : (float)SPEED_FOLLOW;
    /* Turns: SPEED_TURN, or slower if the cruise speed was set lower
       (T6's calibration sweep, T5's creep). */
    float const    turn_speed = (goal_speed < (float)SPEED_TURN) ?
                                goal_speed : (float)SPEED_TURN;

    if ((MSTATE_DISARMED == g_mc.state) || (MSTATE_STALL == g_mc.state))
    {
        if (fault_is_critical())
        {
            return;         /* stay safe until RESET clears the fault */
        }
        motor_arm(true);
        reset_control();
        g_mc.state = MSTATE_IDLE;
    }

    g_mc.last_cmd = cmd;

    switch (cmd)
    {
        case MREQ_STOP:
            if (g_mc.b_wheels)
            {
                g_mc.ramp = (app_fabsf(g_mc.wheel_l) > app_fabsf(g_mc.wheel_r))
                            ? app_fabsf(g_mc.wheel_l) : app_fabsf(g_mc.wheel_r);
                g_mc.b_wheels = false;
            }
            g_mc.b_goal = true;
            if ((MSTATE_IDLE == g_mc.state) || (MSTATE_HOLD == g_mc.state))
            {
                goal_done();
            }
            else
            {
                g_mc.state = MSTATE_RAMP_DOWN;
                g_mc.b_stopping = true;
            }
        break;

        case MREQ_FORWARD:
            start_goal(MSTATE_MOVE, move_edges, 1, 1, goal_speed);
        break;

        case MREQ_BACKWARD:
            start_goal(MSTATE_MOVE, move_edges, -1, -1, goal_speed);
        break;

        case MREQ_TURN_LEFT:
            g_mc.ramp = 0.0f;               /* from standstill (D77) */
            start_goal(MSTATE_TURN, turn_edges, -1, 1, turn_speed);
        break;

        case MREQ_TURN_RIGHT:
            g_mc.ramp = 0.0f;               /* from standstill (D77) */
            start_goal(MSTATE_TURN, turn_edges, 1, -1, turn_speed);
        break;

        case MREQ_SET_SPEED:
            g_mc.cruise = (float)bits_sat_u(arg, 9U);
            if (g_mc.cruise > (float)SPEED_MAX)
            {
                g_mc.cruise = (float)SPEED_MAX;
            }
        break;

        case MREQ_WHEELS:
            /* Manual plaid driving and stunts.  Start from the speeds
               the wheels have now, so a change of mode does not jerk. */
            if (!g_mc.b_wheels)
            {
                g_mc.wheel_l = g_mc.speed_l;
                g_mc.wheel_r = g_mc.speed_r;
                g_mc.wheel_prev = ~g_wheel_word;
            }
            g_mc.b_wheels = true;
            g_mc.state = MSTATE_DRIVE;
            g_mc.b_goal = false;
            g_mc.b_stopping = false;
        break;

        case MREQ_FOLLOW:
            g_mc.b_wheels = false;
            g_mc.cruise = (arg > SPEED_MAX) ? (float)SPEED_MAX : (float)arg;
            if (MSTATE_DRIVE != g_mc.state)
            {
                g_mc.line_integ = 0.0f;     /* no steering left over */
                g_mc.line_prev = 0.0f;
            }
            g_mc.state = MSTATE_DRIVE;
            g_mc.b_goal = false;
            g_mc.b_stopping = false;
        break;

        default:
            g_mc.rejected++;
        break;
    }
}

/* Drain mbf_motion, rejecting requests from the inactive owner. */
static void
receive_requests (void)
{
    motion_req_t req = 0U;
    uint32_t     owner = MREQ_SRC_T6;

    if (MISSION_OBSTACLE == mission_state())
    {
        owner = MREQ_SRC_T5;
    }

    while (tk_rcv_mbf(mbf_motion, &req, TMO_POL) > 0)
    {
        if (bits_get(req, MREQ_SRC_SHIFT, 1U) != owner)
        {
            g_mc.rejected++;
            tm_printf((UB *)"T4: rejected request 0x%04x (owner %d)\n",
                      (INT)req, (INT)owner);
            continue;
        }
        apply_request(req);
    }
}

/* ------------------------------------------------------------------ */

static void
measure (int32_t * p_dl, int32_t * p_dr)
{
    uint16_t const enc_l = g_enc_left;      /* one halfword load each */
    uint16_t const enc_r = g_enc_right;
    int32_t const  dl = (int32_t)(int16_t)(uint16_t)(enc_l - g_mc.enc_prev_l);
    int32_t const  dr = (int32_t)(int16_t)(uint16_t)(enc_r - g_mc.enc_prev_r);
    int32_t        sum_l = 0;
    int32_t        sum_r = 0;
    uint32_t       idx;
    float          step_mm;

    g_mc.enc_prev_l = enc_l;
    g_mc.enc_prev_r = enc_r;
    g_mc.prog_l += dl;
    g_mc.prog_r += dr;

    g_mc.hist_l[g_mc.hist_idx] = dl;
    g_mc.hist_r[g_mc.hist_idx] = dr;
    g_mc.hist_idx = (g_mc.hist_idx + 1U) % MOTION_SPEED_WINDOW;
    for (idx = 0U; idx < MOTION_SPEED_WINDOW; idx++)
    {
        sum_l += g_mc.hist_l[idx];
        sum_r += g_mc.hist_r[idx];
    }
    g_mc.speed_l = ((float)sum_l * MM_PER_EDGE) /
                   ((float)MOTION_SPEED_WINDOW * DT_S);
    g_mc.speed_r = ((float)sum_r * MM_PER_EDGE) /
                   ((float)MOTION_SPEED_WINDOW * DT_S);

    /* Odometry: path length of the axle centre (an in-place turn adds
       nothing), whole mm carried on. */
    step_mm = 0.5f * (float)(((dl + dr) < 0) ? -(dl + dr) : (dl + dr)) *
              MM_PER_EDGE;
    g_mc.odom_frac += step_mm;
    while (g_mc.odom_frac >= 1.0f)
    {
        g_mc.odom_frac -= 1.0f;
        g_mc.odom_wrap++;
        g_mc.odom_total++;
    }

    *p_dl = dl;
    *p_dr = dr;
}

/* MREQ_WHEELS: per-wheel targets from g_wheel_word, each slewed at
   WHEEL_ACCEL_MMS2.  Equal targets (a straight) also get the encoder
   skew correction of a move, so plaid speed stays straight; a new word
   starts a new segment and a new skew count. */
static void
wheel_points (float * p_sp_l, float * p_sp_r)
{
    uint32_t const word = g_wheel_word;
    float          tl = (float)bits_to_signed(
        bits_get(word, WHEEL_L_SHIFT, WHEEL_BITS), WHEEL_BITS);
    float          tr = (float)bits_to_signed(
        bits_get(word, WHEEL_R_SHIFT, WHEEL_BITS), WHEEL_BITS);
    float          sp_l;
    float          sp_r;

    if (0U != bits_get(word, WHEEL_RAW_BIT, 1U))
    {
        /* Raw power (drift, donut): no speed loop and no slew, so the
           tyres can break traction.  The slewed set-points follow the
           measured speeds, so the next closed-loop step starts from what
           the wheels really do.  Set-points for the stall checks are the
           speeds that power would give on the ground. */
        g_mc.raw_l = clampf(tl / 1000.0f, 1.0f);
        g_mc.raw_r = clampf(tr / 1000.0f, 1.0f);
        g_mc.b_raw = true;
        g_mc.b_slewing = false;
        g_mc.wheel_prev = word;
        g_mc.wheel_l = g_mc.speed_l;
        g_mc.wheel_r = g_mc.speed_r;
        g_mc.integ_l = 0.0f;
        g_mc.integ_r = 0.0f;
        *p_sp_l = g_mc.raw_l / PID_FF;
        *p_sp_r = g_mc.raw_r / PID_FF;
        g_mc.ramp = (app_fabsf(g_mc.speed_l) > app_fabsf(g_mc.speed_r)) ?
                    app_fabsf(g_mc.speed_l) : app_fabsf(g_mc.speed_r);
        return;
    }
    g_mc.b_raw = false;
    if (word != g_mc.wheel_prev)
    {
        /* New step: the speed-loop integral mostly corrects the motor
           gain, so scale it to the new target (a reversal flips it);
           carried over unscaled it overshot the next step by 10 %. */
        /* After a raw step there is no speed target to scale from: use
           the learnt integral per mm/s (ol = or_ = 0 below). */
        bool const  b_was_raw =
            (0U != bits_get(g_mc.wheel_prev, WHEEL_RAW_BIT, 1U));
        float const ol = b_was_raw ? 0.0f : (float)bits_to_signed(
            bits_get(g_mc.wheel_prev, WHEEL_L_SHIFT, WHEEL_BITS), WHEEL_BITS);
        float const or_ = b_was_raw ? 0.0f : (float)bits_to_signed(
            bits_get(g_mc.wheel_prev, WHEEL_R_SHIFT, WHEEL_BITS), WHEEL_BITS);

        g_mc.integ_l = (app_fabsf(ol) > 1.0f) ?
                       g_mc.integ_l * clampf(tl / ol, 2.0f) :
                       g_mc.integ_per_l * tl;
        g_mc.integ_r = (app_fabsf(or_) > 1.0f) ?
                       g_mc.integ_r * clampf(tr / or_, 2.0f) :
                       g_mc.integ_per_r * tr;
        g_mc.wheel_prev = word;
        g_mc.prog_l = 0;
        g_mc.prog_r = 0;
    }
    tl = clampf(tl, (float)SPEED_MAX);
    tr = clampf(tr, (float)SPEED_MAX);
    g_mc.wheel_l += clampf(tl - g_mc.wheel_l, WHEEL_STEP_MMS);
    g_mc.wheel_r += clampf(tr - g_mc.wheel_r, WHEEL_STEP_MMS);
    g_mc.b_slewing = (app_fabsf(tl - g_mc.wheel_l) > 0.5f) ||
                     (app_fabsf(tr - g_mc.wheel_r) > 0.5f);
    /* Learn the integral per mm/s while settled, for a wheel that next
       restarts from standstill (whose integral pid_step cleared). */
    if (!g_mc.b_slewing)
    {
        if (app_fabsf(g_mc.wheel_l) > 50.0f)
        {
            g_mc.integ_per_l = g_mc.integ_l / g_mc.wheel_l;
        }
        if (app_fabsf(g_mc.wheel_r) > 50.0f)
        {
            g_mc.integ_per_r = g_mc.integ_r / g_mc.wheel_r;
        }
    }
    sp_l = g_mc.wheel_l;
    sp_r = g_mc.wheel_r;
    if ((tl == tr) && (0.0f != tl))
    {
        float const dir = (tl > 0.0f) ? 1.0f : -1.0f;
        float const skew = dir * (float)(g_mc.prog_l - g_mc.prog_r);

        sp_l -= dir * STRAIGHT_K * skew;
        sp_r += dir * STRAIGHT_K * skew;
    }
    g_mc.ramp = (app_fabsf(sp_l) > app_fabsf(sp_r)) ?
                app_fabsf(sp_l) : app_fabsf(sp_r);
    *p_sp_l = sp_l;
    *p_sp_r = sp_r;
}

/* Set-points for this period from the motion state. */
static void
set_points (float * p_sp_l, float * p_sp_r)
{
    float sp_l = 0.0f;
    float sp_r = 0.0f;

    switch (g_mc.state)
    {
        case MSTATE_DRIVE:
        if (g_mc.b_wheels)
        {
            wheel_points(&sp_l, &sp_r);
        }
        else
        {
            uint32_t const line = g_line_word;
            uint32_t const lstate = bits_get(line, LINE_STATE_SHIFT,
                                             LINE_STATE_BITS);
            float          err = (float)bits_to_signed(
                bits_get(line, LINE_ERR_SHIFT, LINE_ERR_BITS),
                LINE_ERR_BITS) / LINE_ERR_SCALE;
            float          steer;

            if (LSTATE_TRACK != lstate)
            {
                err = 0.0f;     /* junction or lost: hold the heading */
            }
            /* Leaky integral: a brief brush adds little, a sustained
               contact (a curve) builds up steering.  It decays while the
               line is centred and restarts when the side changes. */
            float const pace = g_mc.ramp / STEER_REF_MMS;

            if (((err > 0.0f) && (g_mc.line_integ < 0.0f)) ||
                ((err < 0.0f) && (g_mc.line_integ > 0.0f)))
            {
                g_mc.line_integ = 0.0f;
            }
            /* Build-up and decay by distance, not time: tuned at
               STEER_REF_MMS, the same per millimetre at any speed (by
               time it built twice as much over a line gap at 100 mm/s
               and the car left at 15 deg, 2026-11-10). */
            /* Only a measured contact builds up: a sensor black now, or
               the full-scale error of a line that escaped.  The gentle
               probe after a white stretch (task_line.c) is a guess; built
               up, it turned the car off a straight line (sim, line_step).
               Nor do barcode bars in S4: one line sensor reaching a bar
               before the other is no bend, and built up it yawed the car
               mid-symbol (sim barcode_A/B/D 2/6 -> 1/6; gated, 3/6).  Not
               the capture bit: in a sharp bend the barcode sensor crosses
               the line and starts a capture too. */
            bool const b_contact =
                ((0U != bits_get(line, LINE_BLACK_SHIFT, 1U)) ||
                 (app_fabsf(err) >= STEER_FULL_ERR)) &&
                (MISSION_BARCODE != mission_state());

            if ((0.0f == err) || (!b_contact))
            {
                g_mc.line_integ -= g_mc.line_integ *
                                   ((DT_S * pace) / STEER_LEAK_S);
            }
            if (b_contact)
            {
                g_mc.line_integ = clampf(g_mc.line_integ +
                                         (err * DT_S * pace), STEER_I_MAX);
            }
            /* The gains were tuned at STEER_REF_MMS: scaled with speed, an
               error asks for the same turn, not the same wheel difference.
               Unscaled at 100 mm/s a brush stopped one wheel (2026-11-10). */
            steer = pace * ((STEER_KP * err) + (STEER_KI * g_mc.line_integ) +
                    (STEER_KD * (err - g_mc.line_prev) / DT_S));
            g_mc.line_prev = err;

            {
                /* Slow down while a curve holds the steering built up: the
                   same wheel difference then turns a tighter radius and
                   the sensors get more time per millimetre of bend.  On a
                   straight the build-up leaks away and cruise returns. */
                float target = g_mc.cruise *
                               (1.0f - (CURVE_SLOW *
                                        app_fabsf(g_mc.line_integ)));

                if (target < (g_mc.cruise * CURVE_MIN))
                {
                    target = g_mc.cruise * CURVE_MIN;
                }
                g_mc.ramp += clampf(target - g_mc.ramp, ACCEL_STEP_MMS);
            }
            /* Hairpins (owner 2026-11-19: tight 90s worked, 180s did not):
               with steering capped at the forward speed the inner wheel
               could only stop, pivoting the car on it, and the curve
               slowdown left the outer wheel at ~80 mm/s.  Now the inner
               wheel may reverse, turning the car about its middle, and
               the wheels keep speed while it does. */
            steer = clampf(steer, g_mc.ramp * STEER_LOCK);
            {
                float fwd = g_mc.ramp;
                float const outer = fwd + app_fabsf(steer);

                if ((app_fabsf(steer) > fwd) && (outer > 1.0f) &&
                    (outer < TURN_OUTER_MIN))
                {
                    float const k = TURN_OUTER_MIN / outer;

                    fwd *= k;
                    steer *= k;
                }
                /* Escaped (full-scale error): in a hairpin the IR row,
                   70 mm ahead of the axle, runs off the inside of the
                   bend while the car still rolls forward.  Nearly stop
                   and spin towards the side the line went (R50 hairpin,
                   tests/sim). */
                /* Not over a barcode: its bars can read full scale on
                   one sensor (sim barcode_A, map_running seed 1). */
                g_mc.escape_s = ((LSTATE_TRACK == lstate) &&
                                 (app_fabsf(err) >= STEER_FULL_ERR) &&
                                 (MISSION_BARCODE != mission_state()) &&
                                 (0U == bits_get(line, LINE_CAPTURE_SHIFT,
                                                 1U))) ?
                                (g_mc.escape_s + DT_S) : 0.0f;
                if (((err > 0.0f) && (g_mc.side_s >= 0.0f)) ||
                    ((err < 0.0f) && (g_mc.side_s <= 0.0f)))
                {
                    /* signed: + while left, - while right */
                    g_mc.side_s += (err > 0.0f) ? DT_S : -DT_S;
                }
                else if (0.0f != err)
                {
                    g_mc.side_s = (err > 0.0f) ? DT_S : -DT_S;
                }
                else
                {
                    /* Centred: keep the count. */
                }
                if ((g_mc.escape_s >= ESCAPE_HOLD_S) &&
                    (app_fabsf(g_mc.side_s) >= ESCAPE_SIDE_S))
                {
                    float const spin = (app_fabsf(steer) > ESCAPE_SPIN_MMS) ?
                                       app_fabsf(steer) : ESCAPE_SPIN_MMS;

                    fwd *= ESCAPE_FWD;
                    steer = (err > 0.0f) ? spin : -spin;
                }
                sp_l = fwd - steer;
                sp_r = fwd + steer;
            }
        }
        break;

        case MSTATE_TURN:
        case MSTATE_MOVE:
        case MSTATE_RAMP_DOWN:
        {
            float const progress = 0.5f *
                (float)((g_mc.prog_l * g_mc.dir_l) +
                        (g_mc.prog_r * g_mc.dir_r));
            float const remaining_mm = (g_mc.goal_edges - progress) *
                                       MM_PER_EDGE;
            float const skew = (float)((g_mc.prog_l * g_mc.dir_l) -
                                       (g_mc.prog_r * g_mc.dir_r));
            float       speed;

            if (g_mc.b_stopping)
            {
                g_mc.ramp -= STOP_STEP_MMS;
                if (g_mc.ramp <= 0.0f)
                {
                    g_mc.ramp = 0.0f;
                    if ((app_fabsf(g_mc.speed_l) < STOPPED_MMS) &&
                        (app_fabsf(g_mc.speed_r) < STOPPED_MMS))
                    {
                        goal_done();
                    }
                }
                sp_l = (g_mc.speed_l >= 0.0f) ? g_mc.ramp : -g_mc.ramp;
                sp_r = (g_mc.speed_r >= 0.0f) ? g_mc.ramp : -g_mc.ramp;
                break;
            }

            if (remaining_mm <= 0.0f)
            {
                goal_done();
                break;
            }

            g_mc.ramp += clampf(g_mc.goal_speed - g_mc.ramp,
                                (MSTATE_TURN == g_mc.state) ? TURN_STEP_MMS :
                                ACCEL_STEP_MMS);
            speed = g_mc.ramp;
            if (remaining_mm < RAMP_MOVE_MM)
            {
                g_mc.state = MSTATE_RAMP_DOWN;
                speed = g_mc.goal_speed * (remaining_mm / RAMP_MOVE_MM);
                if (speed < MIN_GOAL_SPEED)
                {
                    /* Never above the set speed (the slow barcode scan). */
                    speed = (g_mc.goal_speed < MIN_GOAL_SPEED) ?
                            g_mc.goal_speed : MIN_GOAL_SPEED;
                }
            }

            /* skew is in goal-progress terms (+ = left ahead), so the
               correction slows the leading wheel along ITS direction.
               Subtracting it from the signed set-point instead made the
               leading wheel faster whenever dir = -1: a backward move
               span the car on the spot within ~0.3 s. */
            sp_l = (float)g_mc.dir_l * (speed - (STRAIGHT_K * skew));
            sp_r = (float)g_mc.dir_r * (speed + (STRAIGHT_K * skew));
        }
        break;

        default:
            /* M0, M1, M6, M7: zero set-point. */
        break;
    }

    *p_sp_l = sp_l;
    *p_sp_r = sp_r;
}

/* One wheel's encoder watch.  The counter must move while the wheel is
   driven; only a counter that moves resets it, so steering that briefly
   drops the duty (line-following weave) cannot hide a dead encoder. */
static void
enc_watch (uint32_t * p_still, int32_t * p_other, int32_t d_self,
           int32_t d_other, float duty)
{
    if (0 != d_self)
    {
        *p_still = 0U;
        *p_other = 0;
        return;
    }
    if (app_fabsf(duty) > ENC_FAULT_DUTY)
    {
        (*p_still)++;
    }
    *p_other += (d_other < 0) ? -d_other : d_other;
}

/* Driven one way, measured moving the other way: a motor or an encoder
   wired backwards.  Without this the PID saturates and the car spins in
   place for ever (the stall test sees plenty of speed). */
static bool
wrong_way (float set_point, float duty, float speed)
{
    return ((app_fabsf(duty) > STALL_DUTY) &&
            (((set_point > 0.0f) && (speed < -WRONG_DIR_MMS)) ||
             ((set_point < 0.0f) && (speed > WRONG_DIR_MMS))));
}

/* A turn or move must make progress.  With a motor or an encoder wired
   backwards the wheels fight the PID and the car shuffles on the spot,
   never reaching the goal and never looking stalled. */
static bool
goal_stuck (float duty_l, float duty_r)
{
    float const progress = 0.5f *
        (float)((g_mc.prog_l * g_mc.dir_l) + (g_mc.prog_r * g_mc.dir_r));

    if (((MSTATE_TURN != g_mc.state) && (MSTATE_MOVE != g_mc.state) &&
         (MSTATE_RAMP_DOWN != g_mc.state)) || g_mc.b_stopping)
    {
        g_mc.goal_stuck = 0U;
        return (false);
    }
    if (progress > (g_mc.goal_best + (float)ENC_FAULT_EDGES))
    {
        g_mc.goal_best = progress;
        g_mc.goal_stuck = 0U;
    }
    else if ((app_fabsf(duty_l) > ENC_FAULT_DUTY) ||
             (app_fabsf(duty_r) > ENC_FAULT_DUTY))
    {
        g_mc.goal_stuck++;
    }
    else
    {
        /* Not driven hard: no evidence either way. */
    }
    return (g_mc.goal_stuck >= GOAL_STUCK_PERIODS);
}

static bool
too_slow (float duty, float speed)
{
    float const mag = app_fabsf(duty);
    float const expected = (mag - PID_DEADBAND) / PID_FF;

    return ((mag > SCALE_DUTY) && (app_fabsf(speed) >= STALL_SPEED) &&
            (app_fabsf(speed) < (SCALE_RATIO * expected)));
}

static void
check_stall (float sp_l, float sp_r, float duty_l, float duty_r,
             int32_t dl, int32_t dr)
{
    int32_t const moved = ENC_FAULT_MOVED;
    bool const    b_stalled =
        ((app_fabsf(duty_l) > STALL_DUTY) &&
         (app_fabsf(g_mc.speed_l) < STALL_SPEED)) ||
        ((app_fabsf(duty_r) > STALL_DUTY) &&
         (app_fabsf(g_mc.speed_r) < STALL_SPEED));
    bool const    b_wrong = wrong_way(sp_l, duty_l, g_mc.speed_l) ||
                            wrong_way(sp_r, duty_r, g_mc.speed_r);
    /* Measured speed far below what the power should give (PID_FF,
       PID_DEADBAND): the encoder counts far fewer edges per wheel turn
       than MOTION_EDGES_PER_REV.  Every turn and distance then overshoots
       and the car spins round and round (bench 2026-11-02); the PID
       cannot see it, since the low count looks like a slow wheel.  A
       jammed wheel reads ~0 and is reported as a stall instead. */
    bool const    b_slow = too_slow(duty_l, g_mc.speed_l) ||
                           too_slow(duty_r, g_mc.speed_r);

    g_mc.stall = b_stalled ? (g_mc.stall + 1U) : 0U;
    g_mc.wrong_dir = b_wrong ? (g_mc.wrong_dir + 1U) : 0U;
    g_mc.too_slow = b_slow ? (g_mc.too_slow + 1U) : 0U;
    enc_watch(&g_mc.still_l, &g_mc.other_l, dl, dr, duty_l);
    enc_watch(&g_mc.still_r, &g_mc.other_r, dr, dl, duty_r);

    bool const    b_stuck = goal_stuck(duty_l, duty_r);

    if (((g_mc.still_l >= STALL_PERIODS) && (g_mc.other_l >= moved)) ||
        ((g_mc.still_r >= STALL_PERIODS) && (g_mc.other_r >= moved)) ||
        (g_mc.wrong_dir >= STALL_PERIODS) || b_stuck ||
        (g_mc.too_slow >= STALL_PERIODS))
    {
        fault_set(FAULT_ENCODER);
        g_mc.state = MSTATE_STALL;
        tm_printf((UB *)"T4: encoder fault (%s)\n",
                  (g_mc.too_slow >= STALL_PERIODS) ?
                  "too few counts: check MOTION_EDGES_PER_REV" :
                  (b_stuck ? "no progress to goal" :
                  ((g_mc.wrong_dir >= STALL_PERIODS) ? "wrong direction"
                                                      : "counter frozen")));
        /* Which wheel, and what it was doing: set-point, measured speed
           (mm/s), duty (%), periods its counter stood still. */
        tm_printf((UB *)"T4:   left  sp %d speed %d duty %d still %d\n",
                  (INT)sp_l, (INT)g_mc.speed_l, (INT)(duty_l * 100.0f),
                  (INT)g_mc.still_l);
        tm_printf((UB *)"T4:   right sp %d speed %d duty %d still %d\n",
                  (INT)sp_r, (INT)g_mc.speed_r, (INT)(duty_r * 100.0f),
                  (INT)g_mc.still_r);
    }
    else if (g_mc.stall >= STALL_PERIODS)
    {
        fault_set(FAULT_MOTOR_STALL);
        g_mc.state = MSTATE_STALL;
        tm_printf((UB *)"T4: motor stall: left speed %d duty %d, right"
                  " speed %d duty %d\n", (INT)g_mc.speed_l,
                  (INT)(duty_l * 100.0f), (INT)g_mc.speed_r,
                  (INT)(duty_r * 100.0f));
    }
    else
    {
        /* Healthy. */
    }
}

static void
publish (void)
{
    uint32_t word = 0U;
    uint32_t odom = 0U;

    g_mc.beat++;
    word = bits_put(word, MOTION_SPEED_L_SHIFT, MOTION_SPEED_BITS,
                    bits_from_signed((int32_t)g_mc.speed_l,
                                     MOTION_SPEED_BITS));
    word = bits_put(word, MOTION_SPEED_R_SHIFT, MOTION_SPEED_BITS,
                    bits_from_signed((int32_t)g_mc.speed_r,
                                     MOTION_SPEED_BITS));
    word = bits_put(word, MOTION_STATE_SHIFT, MOTION_STATE_BITS, g_mc.state);
    word = bits_put(word, MOTION_ARMED_SHIFT, 1U,
                    motor_is_armed() ? 1U : 0U);
    word = bits_put(word, MOTION_BEAT_SHIFT, MOTION_BEAT_BITS, g_mc.beat);
    word = bits_put(word, MOTION_LASTCMD_SHIFT, MREQ_CMD_BITS,
                    g_mc.last_cmd);
    g_motion_word = word;           /* one aligned store (rule R1) */

    odom = bits_put(odom, ODOM_WRAP_SHIFT, ODOM_WRAP_BITS, g_mc.odom_wrap);
    odom = bits_put(odom, ODOM_TOTAL_SHIFT, ODOM_TOTAL_BITS,
                    bits_sat_u(g_mc.odom_total, ODOM_TOTAL_BITS));
    odom = bits_put(odom, ODOM_SAT_SHIFT, 1U,
                    (g_mc.odom_total > BITS_MASK(ODOM_TOTAL_BITS)) ? 1U : 0U);
    g_odom_word = odom;
}

static void
motion_cycle (void)
{
    float   sp_l = 0.0f;
    float   sp_r = 0.0f;
    float   duty_l = 0.0f;
    float   duty_r = 0.0f;
    int32_t dl = 0;
    int32_t dr = 0;

    measure(&dl, &dr);
    receive_requests();

    /* Forced off by T1 / a fault: stop fighting the override. */
    if ((!motor_is_armed()) && (MSTATE_DISARMED != g_mc.state) &&
        (MSTATE_STALL != g_mc.state))
    {
        g_mc.state = MSTATE_DISARMED;
        g_mc.b_goal = false;
        reset_control();
    }

    set_points(&sp_l, &sp_r);

    if (MSTATE_HOLD == g_mc.state)
    {
        g_mc.hold++;
        if (g_mc.hold >= HOLD_PERIODS)
        {
            g_mc.state = MSTATE_IDLE;
            if (g_mc.b_goal)
            {
                g_mc.b_goal = false;
                (void)tk_set_flg(flg_sys, FLG_MOTION_DONE);
            }
        }
    }

    if ((MSTATE_DRIVE <= g_mc.state) && (MSTATE_RAMP_DOWN >= g_mc.state))
    {
        float const keep_l = g_mc.integ_l;
        float const keep_r = g_mc.integ_r;

        duty_l = pid_step(sp_l, g_mc.speed_l, &g_mc.integ_l);
        duty_r = pid_step(sp_r, g_mc.speed_r, &g_mc.integ_r);
        if (g_mc.b_wheels && g_mc.b_slewing)
        {
            /* Plaid: no integral while the set-point still ramps (the
               averaged speed lags it, the integral wound up and the car
               overshot 250 mm/s by 10-20 %, sim). */
            g_mc.integ_l = keep_l;
            g_mc.integ_r = keep_r;
        }
        if (g_mc.b_wheels && g_mc.b_raw && (MSTATE_DRIVE == g_mc.state))
        {
            duty_l = g_mc.raw_l;            /* stunt: power, not speed */
            duty_r = g_mc.raw_r;
            g_mc.integ_l = 0.0f;
            g_mc.integ_r = 0.0f;
        }
        check_stall(sp_l, sp_r, duty_l, duty_r, dl, dr);
    }
    else
    {
        g_mc.integ_l = 0.0f;
        g_mc.integ_r = 0.0f;
    }

    if (MSTATE_STALL == g_mc.state)
    {
        duty_l = 0.0f;
        duty_r = 0.0f;
    }

    motor_write(duty_l, duty_r);    /* re-checks faults with IRQs off */
    publish();
}

EXPORT void motion_task (INT stacd, void * p_exinf)
{
    ER er;

    (void)stacd;
    (void)p_exinf;

    /* T4 runs on APP_CORE, so the GPIO interrupt is routed here too. */
    er = gpio_irq_start();
    CHECK(er, "gpio_irq_start");

    g_mc.state = MSTATE_DISARMED;
    g_mc.enc_prev_l = g_enc_left;
    g_mc.enc_prev_r = g_enc_right;
    g_mc.cruise = (float)SPEED_FOLLOW;
    publish();

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);     /* released by the 10 ms waker */
        (void)tk_can_wup(TSK_SELF);
        motion_cycle();
    }
}

/*** end of file ***/
