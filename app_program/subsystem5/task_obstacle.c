/** @file task_obstacle.c
 *
 * @brief T5 ObstacleScan: forward watch every 70 ms (O0), then, once the
 *        mission has handed motion over (S7): halt, coarse scan, fine
 *        scan, profile, plan, three-leg bypass and line reacquisition
 *        (O1-O11, tribunal Figure 7).
 *
 * The bypass is a straight sequence of blocking steps because T5 is its
 * own task: every step re-checks flg_sys ESTOP and gives up at once.
 * During S7 T5 is the only sender to mbf_motion and the only clearer of
 * MOTION_DONE.
 *
 * Geometry (lateral y positive to the LEFT of the line, x forward, all
 * relative to the sensor at servo angle theta, 90 = ahead):
 *     x = r sin(theta), y = -r cos(theta).
 * Each obstacle edge is pulled in by d * tan(7.5 deg) for the HC-SR04's
 * 15 deg beam (A9).  Depth is taken equal to width (A10).
 */

#include "subsystem5.h"
#include "../app_math.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"

#define TRIGGER_HITS            2U      /* consecutive pings under 200 mm */
#define PROFILE_MM              400U    /* coarse return worth a fine scan */
#define HIT_BAND_MM             120U    /* within this of the closest     */
#define CLEAR_LOOK_MM           1000U
#define CLEAR_NONE              1023U
#define COARSE_COUNT            5U
#define FINE_COUNT              7U
#define FINE_STEP_DEG           10U
#define MAX_SAMPLES             (COARSE_COUNT + FINE_COUNT)
#define NO_ECHO_LIMIT           5U
#define NO_ECHO_BOOT_LIMIT      25U     /* until the first echo, ~2 s: the
                                           HC-SR04 misses its first pings
                                           after power-up, and the fault it
                                           raised stuck the car in S10 at
                                           boot (bench 2026-11-07)       */
#define MARGIN_MM               50U
#define SIDE_MARGIN_MM          50U     /* passing clearance (30 tried
                                           2026-11-19: the real car then
                                           overshot, back to 50, D70)     */
#define FLOOR_EXTRA_MM          50U     /* more when the sonar sees floor
                                           (25 tried, back to 50)         */
#define SIDE_TIE_MM             20U
#define BYPASS_ANGLE_DEG        60U
#define SIN60                   0.8660254f
#define TAN30                   0.5773503f
#define TAN_HALF_BEAM           0.1316525f  /* tan(7.5 deg) */
#define DEG_TO_RAD              0.01745329f
#define REVERSE_MM              120U
#define REACQ_EXTRA_MM          400U
#define REJOIN_MIN_DEG          35U     /* turn back at least this far ...  */
#define REJOIN_MAX_DEG          85U     /* ... and at most this, line or not */
#define REJOIN_CENTRE_ERR       25      /* of +-127: sensors over the line  */
#define REJOIN_TRIM_MM          5       /* closer than this: no correction */
#define REJOIN_TURN_MMS         50U     /* slow: the IR row crosses the line */
#ifndef LINE_ACCEPT_MM
#define LINE_ACCEPT_MM          12U     /* 18 mm line at 60 deg = 21 mm;
                                           a 9 mm wide bar = 10 mm       */
#endif
#define CROSS_MM                40U     /* one line sensor black, then the
                                           other within this: the line was
                                           crossed (D71)                  */
#define SWEEP_DEG               45U
#define MOTION_TIMEOUT_MS       20000   /* one leg at 50 mm/s creep */
#define POLL_MS                 20
#define COOLDOWN_PINGS          15U     /* ~1 s of no re-trigger      */
#define FLOOR_SPREAD_MM         40U     /* coarse ranges this alike: floor */
#define FLOOR_MARGIN_MM         40U     /* trigger this much nearer      */
#define TRIGGER_MIN_MM          80U
#define EMERGENCY_MM            100U    /* stop at once, cooldown or not */
#define BODY_STEP_DEG           4U      /* body scan: turn per ping       */
#define BODY_MAX_STEPS          14U     /* 56 degrees each way at most    */
#define BODY_MISS_EDGE          2U      /* misses in a row: past the edge */
#define BODY_SAMPLES            ((2U * BODY_MAX_STEPS) + 1U)
#define MAX_BYPASS_WIDTH_MM     400U    /* wider: no way round (a wall)   */
#define TURN_ROOM_MM            180U    /* back off to this before turning */
#define APPROACH_MIN_MOVE_MM    25U     /* travel between comparisons:
                                           over twice STEADY_BAND_MM, or an
                                           obstacle approached slowly looks
                                           steady (8 mm took one for the
                                           floor at 50 mm/s, 2026-11-10)   */
#define STEADY_BAND_MM          12      /* range unchanged within this   */
#define STEADY_PINGS            3U      /* ... while moving: the floor   */
#define GUARD_MM                120U    /* bypass leg: stop if closer */
#define GUARD_PINGS             115U    /* ~8 s at one ping per 70 ms */

typedef struct
{
    uint16_t range;     /* mm, US_NO_RETURN_MM if none */
    uint8_t  angle;     /* degrees */
} scan_sample_t;

typedef struct
{
    scan_sample_t samples[MAX_SAMPLES];
    uint32_t      count;
    uint32_t      scan_state;
    uint32_t      no_echo;
    bool          b_echo_seen;
    uint32_t      hits;
    uint32_t      cooldown;
    uint32_t      retry;
    int32_t       y_left;       /* corrected obstacle edges, mm */
    int32_t       y_right;
    uint32_t      width;
    uint32_t      closest;
    uint32_t      bearing;
    uint32_t      clear_left;
    uint32_t      clear_right;
    uint32_t      plan;
    uint32_t      floor_mm;     /* floor echo range, 0 = none seen   */
    uint32_t      steady;       /* pings with an unchanging range    */
    uint32_t      near_hist;    /* last 3 pings: near and approaching */
    uint16_t      prev_range;
    uint16_t      prev_odom;
    bool          b_approach;   /* last approach verdict             */
    bool          b_in_cal;     /* in S0/S1: profile cleared once     */
    uint16_t      line_odom;    /* odometer when the line was seen   */
} obstacle_ctx_t;

static obstacle_ctx_t g_oc;

/* ------------------------------------------------------------------ */

static bool
estop (void)
{
    T_RFLG rflg;

    return ((E_OK == tk_ref_flg(flg_sys, &rflg)) &&
            (0U != (rflg.flgptn & FLG_ESTOP)));
}

static void
publish_profile (void)
{
    uint32_t p0 = 0U;
    uint32_t p1 = 0U;

    p0 = bits_put(p0, PROF_BEARING_SHIFT, PROF_BEARING_BITS, g_oc.bearing);
    p0 = bits_put(p0, PROF_WIDTH_SHIFT, PROF_WIDTH_BITS,
                  bits_sat_u(g_oc.width, PROF_WIDTH_BITS));
    p0 = bits_put(p0, PROF_CLOSEST_SHIFT, PROF_CLOSEST_BITS,
                  bits_sat_u(g_oc.closest, PROF_CLOSEST_BITS));
    p0 = bits_put(p0, PROF_PLAN_SHIFT, PROF_PLAN_BITS, g_oc.plan);
    p0 = bits_put(p0, PROF_SCAN_SHIFT, PROF_SCAN_BITS, g_oc.scan_state);
    p1 = bits_put(p1, PROF_LEFT_SHIFT, PROF_CLEAR_BITS,
                  bits_sat_u(g_oc.clear_left, PROF_CLEAR_BITS));
    p1 = bits_put(p1, PROF_RIGHT_SHIFT, PROF_CLEAR_BITS,
                  bits_sat_u(g_oc.clear_right, PROF_CLEAR_BITS));
    p1 = bits_put(p1, PROF_RETRY_SHIFT, 1U, g_oc.retry);

    /* Two words: too large for one atomic store, so under mtx_state.
       Held for two stores only (NFR17). */
    (void)tk_loc_mtx(mtx_state, TMO_FEVR);
    g_vehicle_state.profile0 = p0;
    g_vehicle_state.profile1 = p1;
    (void)tk_unl_mtx(mtx_state);
}

static void
set_scan_state (uint32_t state)
{
    g_oc.scan_state = state;
    publish_profile();
}

/* One ping with no-echo fault bookkeeping. */
static uint16_t
ping (void)
{
    bool           b_edge = false;
    uint16_t const range = ultrasonic_ping(&b_edge);

    if (b_edge)
    {
        g_oc.no_echo = 0U;
        g_oc.b_echo_seen = true;
        if (0U != ((uint32_t)fault_get() & FAULT_NO_ECHO))
        {
            fault_clear(FAULT_NO_ECHO);
        }
    }
    else
    {
        g_oc.no_echo++;
        if ((g_oc.b_echo_seen ? NO_ECHO_LIMIT : NO_ECHO_BOOT_LIMIT) ==
            g_oc.no_echo)
        {
            fault_set(FAULT_NO_ECHO);
            tm_printf((UB *)"T5: no echo from HC-SR04\n");
        }
    }
    return (range);
}

#if SERVO_SCAN
static uint16_t
measure_at (uint32_t angle)
{
    (void)tk_dly_tsk((RELTIM)servo_move(angle));
    return (ping());
}
#endif /* SERVO_SCAN */

/* Post a non-goal request (SET_SPEED): no MOTION_DONE follows. */
static bool
post (uint32_t cmd, uint32_t arg)
{
    return (E_OK == motion_request(cmd, arg, MREQ_SRC_T5));
}

/* Post a goal request as T5 and wait for its MOTION_DONE (or abort). */
static bool
move (uint32_t cmd, uint32_t arg)
{
    UINT ptn = 0U;
    ER   er;

    (void)tk_clr_flg(flg_sys, ~FLG_MOTION_DONE);
    if (E_OK != motion_request(cmd, arg, MREQ_SRC_T5))
    {
        return (false);
    }
    er = tk_wai_flg(flg_sys, FLG_MOTION_DONE | FLG_ESTOP, TWF_ORW, &ptn,
                    MOTION_TIMEOUT_MS);
    (void)tk_clr_flg(flg_sys, ~FLG_MOTION_DONE);
    return ((E_OK == er) && (0U == (ptn & FLG_ESTOP)));
}

/* A forward bypass leg with the sonar (centred) watching ahead.  The
   scan samples only a few angles, so an object between two of them can
   sit on the planned path; stopping short turns that into a blocked
   bypass (reverse, retry, BYPASS_FAIL) instead of a collision. */
static bool
guarded_forward (uint32_t mm)
{
    UINT     ptn = 0U;
    uint32_t idx;

    (void)tk_clr_flg(flg_sys, ~FLG_MOTION_DONE);
    if (E_OK != motion_request(MREQ_FORWARD, mm, MREQ_SRC_T5))
    {
        return (false);
    }
    for (idx = 0U; idx < GUARD_PINGS; idx++)
    {
        uint16_t const range = ping();      /* paced at 70 ms */

        if (E_OK == tk_wai_flg(flg_sys, FLG_MOTION_DONE | FLG_ESTOP,
                               TWF_ORW, &ptn, TMO_POL))
        {
            (void)tk_clr_flg(flg_sys, ~FLG_MOTION_DONE);
            return (0U == (ptn & FLG_ESTOP));
        }
        if (range < GUARD_MM)
        {
            tm_printf((UB *)"T5: bypass leg blocked at %d mm\n", (INT)range);
            (void)move(MREQ_STOP, 0U);
            return (false);
        }
    }
    return (false);
}

/* ------------------------------------------------------------------ */

#if SERVO_SCAN
static void
add_sample (uint32_t angle, uint16_t range)
{
    /* Once the floor's range is known, an echo from that far is the
       floor: no return, so the obstacle's sides are not read as walls. */
    if ((0U != g_oc.floor_mm) &&
        ((uint32_t)range + (FLOOR_MARGIN_MM / 2U) >= g_oc.floor_mm))
    {
        range = US_NO_RETURN_MM;
    }
    if (g_oc.count < MAX_SAMPLES)
    {
        g_oc.samples[g_oc.count].angle = (uint8_t)angle;
        g_oc.samples[g_oc.count].range = range;
        g_oc.count++;
    }
}
#endif /* SERVO_SCAN */

#if SERVO_SCAN
/* O2: five coarse angles.  Returns the angle of the nearest return. */
static uint32_t
coarse_scan (uint16_t * p_min)
{
    static uint8_t const angles[COARSE_COUNT] = {30U, 60U, 90U, 120U, 150U};
    uint32_t idx;
    uint32_t best = SERVO_CENTRE_DEG;
    uint16_t min_range = US_NO_RETURN_MM;

    set_scan_state(SCAN_COARSE);
    g_oc.count = 0U;
    for (idx = 0U; (idx < COARSE_COUNT) && !estop(); idx++)
    {
        uint16_t range;

        add_sample(angles[idx], measure_at(angles[idx]));
        range = g_oc.samples[g_oc.count - 1U].range;    /* floor removed */
        if (range < min_range)
        {
            min_range = range;
            best = angles[idx];
        }
    }
    *p_min = min_range;
    return (best);
}

/* The coarse scan saw the same short range in every direction from 30
   to 150 degrees.  An obstacle or a wall gives short ranges ahead and
   longer ones (or none) to the sides; only the floor, reached by the
   lower edge of the wide beam of a low-mounted sensor, is equally near
   all round (bench 2026-11-02: "obstacles" 160-190 mm away, ~600 mm wide,
   symmetric, at every point of the course).  Remember its range: from
   then on only something clearly nearer triggers a stop. */
static bool
floor_echo (void)
{
    uint32_t idx;
    uint32_t lo = US_NO_RETURN_MM;
    uint32_t hi = 0U;

    if (COARSE_COUNT != g_oc.count)
    {
        return (false);
    }
    for (idx = 0U; idx < COARSE_COUNT; idx++)
    {
        uint32_t const r = g_oc.samples[idx].range;

        if (r >= PROFILE_MM)
        {
            return (false);
        }
        lo = (r < lo) ? r : lo;
        hi = (r > hi) ? r : hi;
    }
    if ((hi - lo) > FLOOR_SPREAD_MM)
    {
        return (false);
    }
    g_oc.floor_mm = lo;
    tm_printf((UB *)"T5: that was the floor (%d-%d mm all round), not an"
              " obstacle\n", (INT)lo, (INT)hi);
    return (true);
}

#endif /* SERVO_SCAN */

/* Forward range that counts as an obstacle: S5_TRIGGER_MM, or nearer
   than the floor echo once one has been seen. */
static uint32_t
trigger_mm (void)
{
    uint32_t limit = S5_TRIGGER_MM;

    if ((g_oc.floor_mm > (FLOOR_MARGIN_MM + TRIGGER_MIN_MM)) &&
        ((g_oc.floor_mm - FLOOR_MARGIN_MM) < limit))
    {
        limit = g_oc.floor_mm - FLOOR_MARGIN_MM;
    }
    return ((limit < TRIGGER_MIN_MM) ? TRIGGER_MIN_MM : limit);
}

#if SERVO_SCAN
/* O3: seven angles 10 degrees apart around the coarse minimum, starting
   from the end nearer the servo (it is at 150 after the coarse pass). */
static void
fine_scan (uint32_t centre)
{
    int32_t first = (int32_t)centre + 30;
    int32_t step = -(int32_t)FINE_STEP_DEG;
    uint32_t idx;

    set_scan_state(SCAN_FINE);
    for (idx = 0U; (idx < FINE_COUNT) && !estop(); idx++)
    {
        int32_t const angle = first + (step * (int32_t)idx);

        if ((angle >= 0) && (angle <= 180) &&
            (0 != ((angle - (int32_t)centre) % 30)))
        {
            add_sample((uint32_t)angle, measure_at((uint32_t)angle));
        }
    }
}

static float
lateral (scan_sample_t const * p_s)
{
    return (-(float)p_s->range * app_cosf((float)p_s->angle * DEG_TO_RAD));
}

/* O4: reduce the samples to edges, width, closest point, clearances. */
static void
profile (void)
{
    uint32_t idx;
    uint32_t closest = US_NO_RETURN_MM;
    uint32_t a_min = 180U;
    uint32_t a_max = 0U;
    uint32_t s_min = 0U;
    uint32_t s_max = 0U;
    float    pull;
    float    y_l;
    float    y_r;

    set_scan_state(SCAN_PROFILE);

    for (idx = 0U; idx < g_oc.count; idx++)
    {
        if (g_oc.samples[idx].range < closest)
        {
            closest = g_oc.samples[idx].range;
            g_oc.bearing = (uint32_t)g_oc.samples[idx].angle / 10U;
        }
    }

    /* Hits: returns within HIT_BAND_MM of the closest point. */
    for (idx = 0U; idx < g_oc.count; idx++)
    {
        uint32_t const r = g_oc.samples[idx].range;
        uint32_t const a = g_oc.samples[idx].angle;

        if ((r <= (closest + HIT_BAND_MM)) && (r < PROFILE_MM))
        {
            if (a <= a_min)
            {
                a_min = a;
                s_min = idx;
            }
            if (a >= a_max)
            {
                a_max = a;
                s_max = idx;
            }
        }
    }

    pull = (float)closest * TAN_HALF_BEAM;
    y_l = lateral(&g_oc.samples[s_max]) - pull;     /* larger angle: left */
    y_r = lateral(&g_oc.samples[s_min]) + pull;
    if (y_l < y_r)
    {
        float const mid = 0.5f * (y_l + y_r);       /* narrower than beam */

        y_l = mid + 10.0f;
        y_r = mid - 10.0f;
    }
    if (a_max >= 150U)
    {
        y_l += (float)MARGIN_MM;    /* edge at the scan limit: unknown */
    }
    if (a_min <= 30U)
    {
        y_r -= (float)MARGIN_MM;
    }

    g_oc.y_left = (int32_t)y_l;
    g_oc.y_right = (int32_t)y_r;
    g_oc.width = (uint32_t)(y_l - y_r);
    g_oc.closest = closest;

    /* Clearance: free lateral space beyond each edge, up to 1 m.  Returns
       from the obstacle itself (the hits above) are not other objects:
       with a 15 degree beam they land just outside the pulled-in edges
       and would otherwise read as a wall a few mm away on both sides. */
    g_oc.clear_left = CLEAR_NONE;
    g_oc.clear_right = CLEAR_NONE;
    for (idx = 0U; idx < g_oc.count; idx++)
    {
        uint32_t const r = g_oc.samples[idx].range;
        float const    y = lateral(&g_oc.samples[idx]);

        if ((r >= CLEAR_LOOK_MM) ||
            ((r <= (closest + HIT_BAND_MM)) && (r < PROFILE_MM)))
        {
            continue;
        }
        if ((y > y_l) && ((uint32_t)(y - y_l) < g_oc.clear_left))
        {
            g_oc.clear_left = (uint32_t)(y - y_l);
        }
        if ((y < y_r) && ((uint32_t)(y_r - y) < g_oc.clear_right))
        {
            g_oc.clear_right = (uint32_t)(y_r - y);
        }
    }

    /* An obstacle still being hit at the 30 / 150 degree scan limit may
       go on beyond it: that side's clearance is unknown, not free.  The
       reverse-and-rescan then looks again from further back, where the
       same angles cover more of the width. */
    if (a_max >= 150U)
    {
        g_oc.clear_left = 0U;
    }
    if (a_min <= 30U)
    {
        g_oc.clear_right = 0U;
    }

    tm_printf((UB *)"T5: profile width %d closest %d edges L%d R%d"
              " clear L%d R%d\n", (INT)g_oc.width, (INT)g_oc.closest,
              (INT)g_oc.y_left, (INT)g_oc.y_right,
              (INT)g_oc.clear_left, (INT)g_oc.clear_right);
}

#endif /* SERVO_SCAN */

/* O5: continue, left, right, or reverse-and-reattempt. */
static uint32_t
plan (void)
{
    int32_t const half_path = (int32_t)((ROBOT_WIDTH_MM / 2U) + MARGIN_MM);
    uint32_t const need = ROBOT_WIDTH_MM + (2U * MARGIN_MM);
    bool const b_left = (g_oc.clear_left >= need);
    bool const b_right = (g_oc.clear_right >= need);
    uint32_t decision = PLAN_REVERSE;

    set_scan_state(SCAN_PLAN);

    if ((g_oc.y_right > half_path) || (g_oc.y_left < -half_path))
    {
        decision = PLAN_CONTINUE;           /* outside the swept path */
    }
    else if (g_oc.width > MAX_BYPASS_WIDTH_MM)
    {
        /* A wall, or boxes abreast: no way round.  Going round anyway
           clipped the outer box, whose corner the sonar cannot see at that
           angle (tests/sim obstacle_boxed).  Reverse, rescan, BYPASS_FAIL
           if still blocked. */
        decision = PLAN_REVERSE;
    }
    else if (b_left && b_right)
    {
        /* Both free: the shorter way round, i.e. past the edge nearer the
           line (an obstacle sitting left of the line is passed on the
           right).  Clearance only breaks a tie. */
        int32_t const via_left = g_oc.y_left;
        int32_t const via_right = -g_oc.y_right;

        if ((via_right + (int32_t)SIDE_TIE_MM) < via_left)
        {
            decision = PLAN_RIGHT;
        }
        else if ((via_left + (int32_t)SIDE_TIE_MM) < via_right)
        {
            decision = PLAN_LEFT;
        }
        else
        {
            decision = ((g_oc.clear_right > (g_oc.clear_left + SIDE_TIE_MM)) ?
                        PLAN_RIGHT : PLAN_LEFT);
        }
    }
    else if (b_left)
    {
        decision = PLAN_LEFT;
    }
    else if (b_right)
    {
        decision = PLAN_RIGHT;
    }
    else
    {
        decision = PLAN_REVERSE;
    }

    g_oc.plan = decision;
    publish_profile();
    return (decision);
}

/* Is a line channel on a run long enough to be the line, not a bar? */
static bool
line_seen (void)
{
    uint32_t const line = g_line_word;

    return ((0U != bits_get(line, LINE_BLACK_SHIFT, 1U)) &&
            (bits_get(line, LINE_RUN_SHIFT, LINE_RUN_BITS) >= LINE_ACCEPT_MM));
}

/* Drive 'mm' forward at the current speed, stopping early on the line. */
static bool
creep_until_line (uint32_t mm, bool * p_b_found)
{
    UINT ptn = 0U;
    ER   er;

    bool     b_left = false;        /* each line sensor seen black,  */
    bool     b_right = false;       /* and where (odometer)          */
    uint16_t at_left = 0U;
    uint16_t at_right = 0U;

    *p_b_found = false;
    (void)tk_clr_flg(flg_sys, ~FLG_MOTION_DONE);
    if (E_OK != motion_request(MREQ_FORWARD, mm, MREQ_SRC_T5))
    {
        return (false);
    }

    for (;;)
    {
        er = tk_wai_flg(flg_sys, FLG_MOTION_DONE | FLG_ESTOP, TWF_ORW, &ptn,
                        POLL_MS);
        if ((E_OK == er) && (0U != (ptn & FLG_ESTOP)))
        {
            return (false);
        }
        if (E_OK == er)
        {
            (void)tk_clr_flg(flg_sys, ~FLG_MOTION_DONE);
            return (true);                  /* leg finished, no line */
        }
        if (line_seen())
        {
            *p_b_found = true;
            g_oc.line_odom = odom_now_mm();
            return (move(MREQ_STOP, 0U));
        }
        /* A crossing: the line under one sensor, then under the other.
           Across an 18 mm line at 60 deg the real car's wide sensor spots
           saw each too briefly for LINE_ACCEPT_MM and it drove on past
           the line (2026-11-19 17:07, D71).  The line lies between the
           two contacts. */
        {
            uint32_t const line = g_line_word;
            uint16_t const now = odom_now_mm();

            if (0U != bits_get(line, LINE_BLACK_SHIFT, 1U))
            {
                int32_t const err = bits_to_signed(
                    bits_get(line, LINE_ERR_SHIFT, LINE_ERR_BITS),
                    LINE_ERR_BITS);

                if (err > 0)
                {
                    b_left = true;          /* + = line to the left */
                    at_left = now;
                }
                else if (err < 0)
                {
                    b_right = true;
                    at_right = now;
                }
                else
                {
                    /* Both: a cross bar, or the line square on. */
                }
            }
            if (b_left && b_right)
            {
                /* Wrap-safe: which came later, and how far apart. */
                bool const     b_left_later =
                    (odom_delta_mm(at_left, at_right) < 0x8000U);
                uint32_t const gap = b_left_later ?
                    odom_delta_mm(at_left, at_right) :
                    odom_delta_mm(at_right, at_left);

                if (gap <= CROSS_MM)
                {
                    *p_b_found = true;
                    g_oc.line_odom = b_left_later ?
                        (uint16_t)(at_right + (gap / 2U)) :
                        (uint16_t)(at_left + (gap / 2U));
                    tm_printf((UB *)"T5: line crossed (sensors %d mm"
                              " apart)\n", (INT)gap);
                    return (move(MREQ_STOP, 0U));
                }
                /* Too far apart: two separate marks; keep the later. */
                if (b_left_later)
                {
                    b_right = false;
                }
                else
                {
                    b_left = false;
                }
            }
        }
    }
}

#if !SERVO_SCAN
static void  body_mark (void);
static float body_heading (void);
#endif

/* Sensors over the line: black with a small error, or the error changed
   sign since the last black sample (crossed the centre between polls).
   *p_prev holds the last black error, 0 = none yet. */
static bool
line_centred (int32_t * p_prev)
{
    uint32_t const line = g_line_word;
    int32_t const  err = bits_to_signed(bits_get(line, LINE_ERR_SHIFT,
                                                 LINE_ERR_BITS),
                                        LINE_ERR_BITS);
    bool           b_hit = false;

    if (0U != bits_get(line, LINE_BLACK_SHIFT, 1U))
    {
        b_hit = ((err <= REJOIN_CENTRE_ERR) && (err >= -REJOIN_CENTRE_ERR)) ||
                ((0 != *p_prev) && ((err > 0) != (*p_prev > 0)));
        *p_prev = (0 != err) ? err : *p_prev;
    }
    return (b_hit);
}

/* Turn back along the line with the axle on it: the IR row sweeps an arc
   round the axle and crosses the line exactly when the car is aligned,
   so stop there instead of after a fixed 60 degrees (the real car ended
   up across the line after a bypass, 2026-11-19).  Heading from the
   encoders. */
static bool
turn_until_line (uint32_t cmd, uint32_t back)
{
    UINT    ptn = 0U;
    ER      er;
    int32_t prev = 0;

    if (!post(MREQ_SET_SPEED, REJOIN_TURN_MMS))
    {
        return (false);
    }
    (void)tk_clr_flg(flg_sys, ~FLG_MOTION_DONE);
#if !SERVO_SCAN
    body_mark();
#endif
    if (E_OK != motion_request(cmd, REJOIN_MAX_DEG, MREQ_SRC_T5))
    {
        return (false);
    }
    for (;;)
    {
        er = tk_wai_flg(flg_sys, FLG_MOTION_DONE | FLG_ESTOP, TWF_ORW, &ptn,
                        POLL_MS);
        if ((E_OK == er) && (0U != (ptn & FLG_ESTOP)))
        {
            return (false);
        }
        if (E_OK == er)
        {
            (void)tk_clr_flg(flg_sys, ~FLG_MOTION_DONE);
            tm_printf((UB *)"T5: rejoin turn: line not centred within %d deg,"
                      " back to %d deg\n", (INT)REJOIN_MAX_DEG,
                      (INT)BYPASS_ANGLE_DEG);
            return (move(back, REJOIN_MAX_DEG - BYPASS_ANGLE_DEG));
        }
#if !SERVO_SCAN
        if (line_centred(&prev) &&
            (app_fabsf(body_heading()) >= (float)REJOIN_MIN_DEG))
        {
            tm_printf((UB *)"T5: rejoin turn stopped on the line after %d"
                      " deg\n", (INT)app_fabsf(body_heading()));
            return (move(MREQ_STOP, 0U));
        }
#endif
    }
}

/* O6-O9 for one side; true once the line is reacquired and aligned. */
static bool
bypass (uint32_t side)
{
    uint32_t const away = (PLAN_LEFT == side) ? MREQ_TURN_LEFT
                                              : MREQ_TURN_RIGHT;
    uint32_t const back = (PLAN_LEFT == side) ? MREQ_TURN_RIGHT
                                              : MREQ_TURN_LEFT;
    int32_t        edge = (PLAN_LEFT == side) ? g_oc.y_left : -g_oc.y_right;
    float          shift;
    float          along;
    bool           b_found = false;

    if (edge < 0)
    {
        edge = 0;
    }
    shift = (float)edge + (float)(ROBOT_WIDTH_MM / 2U) +
            (float)SIDE_MARGIN_MM;
    if (0U != g_oc.floor_mm)
    {
        /* Seen from close up through a floor-limited sonar the edges are
           less certain: pass wider. */
        shift += (float)FLOOR_EXTRA_MM;
    }
    along = ((float)g_oc.closest + (float)ROBOT_US_AHEAD_MM +
             (float)g_oc.width + (float)ROBOT_LENGTH_MM + (float)MARGIN_MM) -
            (shift * TAN30);    /* leg 1 already gained this */
    if (along < 100.0f)
    {
        along = 100.0f;
    }

    /* Room to turn: a low sonar sees the obstacle late and the car can
       stop ~80 mm short; turning on the spot there swings a front corner
       into it. */
    if (g_oc.closest < TURN_ROOM_MM)
    {
        set_scan_state(SCAN_REVERSE);
        if (!move(MREQ_BACKWARD, TURN_ROOM_MM - g_oc.closest))
        {
            return (false);
        }
        g_oc.closest = TURN_ROOM_MM;
    }

    set_scan_state(SCAN_OUT);
    (void)servo_move(SERVO_CENTRE_DEG);
    if (!post(MREQ_SET_SPEED, SPEED_FOLLOW) ||
        !move(away, BYPASS_ANGLE_DEG) ||
        !guarded_forward((uint32_t)(shift / SIN60)))
    {
        return (false);
    }

    set_scan_state(SCAN_ALONG);
    if (!move(back, BYPASS_ANGLE_DEG) ||
        !guarded_forward((uint32_t)along))
    {
        return (false);
    }

    set_scan_state(SCAN_BACK);
    if (!move(back, BYPASS_ANGLE_DEG) ||
        !post(MREQ_SET_SPEED, SPEED_CREEP) ||
        !creep_until_line((uint32_t)(shift / SIN60), &b_found))
    {
        return (false);
    }

    set_scan_state(SCAN_REACQUIRE);
    if (!b_found)
    {
        if (!creep_until_line(REACQ_EXTRA_MM, &b_found))
        {
            return (false);
        }
    }
    if (!b_found)
    {
        /* Fallback sweep: +-45 degrees in place. */
        if (!move(away, SWEEP_DEG))
        {
            return (false);
        }
        b_found = line_seen();
        if (!b_found)
        {
            if (!move(back, 2U * SWEEP_DEG))
            {
                return (false);
            }
            b_found = line_seen();
        }
        if (!b_found)
        {
            return (false);
        }
        return (post(MREQ_SET_SPEED, SPEED_FOLLOW));
    }

    /* Put the axle where the IR row saw the line, allowing for how far
       the car rolled before it stopped (not modelled: on the real car it
       carried on past the line), then turn back along it. */
    {
        int32_t const rolled = (int32_t)odom_delta_mm(odom_now_mm(),
                                                      g_oc.line_odom);
        int32_t const fwd = (int32_t)LINE_SENSOR_AHEAD_MM - rolled;

        tm_printf((UB *)"T5: line seen, rolled %d mm past: %s %d mm to put"
                  " the axle on it\n", (INT)rolled,
                  (UB *)((fwd >= 0) ? "forward" : "back"),
                  (INT)((fwd >= 0) ? fwd : -fwd));
        if (fwd > REJOIN_TRIM_MM)
        {
            if (!move(MREQ_FORWARD, (uint32_t)fwd))
            {
                return (false);
            }
        }
        else if (fwd < -REJOIN_TRIM_MM)
        {
            if (!move(MREQ_BACKWARD, (uint32_t)(-fwd)))
            {
                return (false);
            }
        }
        else
        {
            /* Close enough. */
        }
    }
    return (turn_until_line(away, back) &&
            post(MREQ_SET_SPEED, SPEED_FOLLOW));
}

/* The whole S7 episode.  Returns when T6 can take motion back. */
#if !SERVO_SCAN
/* Fixed sonar (SERVO_SCAN 0): the obstacle is measured by turning the
   car (body scan below).  False if nothing is there any more. */
/* A look to one side: the floor echo or no echo both mean open. */
static uint32_t
open_range (uint16_t range)
{
    if ((range >= US_NO_RETURN_MM) ||
        ((0U != g_oc.floor_mm) &&
         (((uint32_t)range + (FLOOR_MARGIN_MM / 2U)) >= g_oc.floor_mm)))
    {
        return (CLEAR_LOOK_MM);
    }
    return ((uint32_t)range);
}

/* Body scan (owner 2026-11-17: the servo only turns 15 degrees each way,
   too little to see the edges of an obstacle).  The car turns in place
   in BODY_STEP_DEG steps, pinging at each, until the echo has left the
   obstacle for BODY_MISS_EDGE steps or BODY_MAX_STEPS are done, comes back
   to the centre, then does the other side.  The sonar sits
   ROBOT_US_AHEAD_MM ahead of the axle, so a return at heading t and range
   r is at y = (ahead + r) sin t across the line (+ = left). */
typedef struct
{
    int16_t  deg;       /* + = left of the start heading */
    uint16_t range;
} body_sample_t;

/* Heading change since body_mark(), deg (+ = left), from the encoders:
   small in-place turns overshoot a little each, and the scan once came
   back 10 degrees off and drove the bypass into the box (tests/sim). */
static uint16_t g_body_enc_l;
static uint16_t g_body_enc_r;

static void
body_mark (void)
{
    g_body_enc_l = g_enc_left;
    g_body_enc_r = g_enc_right;
}

static float
body_heading (void)
{
    int32_t const dl = (int32_t)(int16_t)(uint16_t)(g_enc_left - g_body_enc_l);
    int32_t const dr = (int32_t)(int16_t)(uint16_t)(g_enc_right - g_body_enc_r);
    float const   mm = (APP_PI_F * MOTION_WHEEL_DIA_MM) / MOTION_EDGES_PER_REV;

    return (((float)(dr - dl) * mm / MOTION_TRACK_MM) * (180.0f / APP_PI_F));
}

/* Turn back to the heading of body_mark() (within a degree). */
static void
body_recentre (void)
{
    float const h = body_heading();
    uint32_t const deg = (uint32_t)(app_fabsf(h) + 0.5f);

    if (deg >= 1U)
    {
        (void)move((h > 0.0f) ? MREQ_TURN_RIGHT : MREQ_TURN_LEFT, deg);
    }
}

static bool
body_hit (uint16_t range, uint32_t closest)
{
    uint32_t const r = open_range(range);

    return ((r < PROFILE_MM) && (r <= (closest + HIT_BAND_MM)));
}

static uint32_t
body_side (uint32_t turn, int32_t sign, body_sample_t * p_s, uint32_t n,
           uint32_t closest)
{
    uint32_t step;
    uint32_t turned = 0U;
    uint32_t misses = 0U;

    for (step = 1U; step <= BODY_MAX_STEPS; step++)
    {
        uint16_t r;

        if (estop() || !move(turn, BODY_STEP_DEG))
        {
            break;
        }
        turned += BODY_STEP_DEG;
        r = ping();
        {
            float const h = body_heading();     /* the heading actually reached */

            p_s[n].deg = (int16_t)((h >= 0.0f) ? (h + 0.5f) : (h - 0.5f));
        }
        p_s[n].range = r;
        n++;
        tm_printf((UB *)"T5: scan %d deg %d mm\n", (INT)p_s[n - 1U].deg,
                  (INT)r);
        misses = body_hit(r, closest) ? 0U : (misses + 1U);
        if (misses >= BODY_MISS_EDGE)
        {
            break;
        }
    }
    (void)sign;
    if (turned > 0U)
    {
        body_recentre();                    /* back to the line's heading */
    }
    return (n);
}

/* One side's edge from its samples s[first .. last-1], in scan order
   (outwards).  Open if the scan stopped while still on the obstacle. */
static float
body_edge (body_sample_t const * p_s, uint32_t first, uint32_t last,
           uint32_t closest, bool * p_b_open)
{
    float const ahead = (float)ROBOT_US_AHEAD_MM;
    uint32_t    hit = 0U;           /* sample 0 (straight ahead) is a hit */
    uint32_t    idx;
    int32_t     deg;
    int32_t     sign = 1;

    for (idx = first; idx < last; idx++)
    {
        if (body_hit(p_s[idx].range, closest))
        {
            hit = idx;
        }
    }
    if (first < last)
    {
        sign = (p_s[first].deg < 0) ? -1 : 1;
    }
    if ((hit + 1U < last) && (hit + 1U >= first))
    {
        /* Midway between the last hit and the first miss. */
        deg = (p_s[hit].deg + p_s[hit + 1U].deg) / 2;
        *p_b_open = false;
    }
    else if (0U == hit)
    {
        deg = sign * (int32_t)BODY_STEP_DEG;    /* no samples that side */
        *p_b_open = (first >= last);
    }
    else
    {
        deg = p_s[hit].deg + (sign * (int32_t)BODY_STEP_DEG);
        *p_b_open = true;                   /* still hitting at the limit */
    }
    /* The beam still sees the edge half a beam after passing it. */
    return (((ahead + (float)open_range(p_s[hit].range)) *
             app_sinf((float)deg * DEG_TO_RAD)) -
            ((float)sign * (float)closest * TAN_HALF_BEAM));
}

static bool
fixed_profile (void)
{
    body_sample_t s[BODY_SAMPLES];
    uint16_t      range = ping();
    uint32_t      n = 0U;
    uint32_t      idx;
    uint32_t      closest;
    uint32_t      n_right;
    int32_t       deg_c = 0;
    bool          b_open_l = false;
    bool          b_open_r = false;
    float         y_l;
    float         y_r;
    float const   ahead = (float)ROBOT_US_AHEAD_MM;

    if ((range >= PROFILE_MM) ||
        ((0U != g_oc.floor_mm) &&
         (((uint32_t)range + (FLOOR_MARGIN_MM / 2U)) >= g_oc.floor_mm)))
    {
        g_oc.closest = range;
        return (false);
    }
    /* Room for the front corners while turning. */
    if (range < TURN_ROOM_MM)
    {
        (void)move(MREQ_BACKWARD, TURN_ROOM_MM - range);
        range = ping();
    }

    set_scan_state(SCAN_COARSE);
    body_mark();
    s[n].deg = 0;
    s[n].range = range;
    n++;
    closest = open_range(range);
    n = body_side(MREQ_TURN_RIGHT, -1, s, n, closest);
    n_right = n;                        /* s[1 .. n_right-1]: right side */
    n = body_side(MREQ_TURN_LEFT, 1, s, n, closest);
    body_recentre();                        /* any error left: once more */
    tm_printf((UB *)"T5: body scan back to %d deg of the start heading\n",
              (INT)body_heading());

    set_scan_state(SCAN_PROFILE);
    for (idx = 0U; idx < n; idx++)
    {
        uint32_t const r = open_range(s[idx].range);

        if (r < closest)
        {
            closest = r;
            deg_c = s[idx].deg;
        }
    }
    /* Each edge midway between the last hit and the first miss, from the
       headings the encoders say the car reached (the nominal 4 degree
       steps were really 6-7, which made the first attempt read a 100 mm
       box as 29 mm and clip it), pulled in by half the 15 degree beam. */
    y_l = body_edge(s, n_right, n, closest, &b_open_l);
    y_r = body_edge(s, 1U, n_right, closest, &b_open_r);
    if (y_l < y_r)
    {
        float const mid = 0.5f * (y_l + y_r);

        y_l = mid + 10.0f;
        y_r = mid - 10.0f;
    }

    g_oc.clear_left = CLEAR_NONE;
    g_oc.clear_right = CLEAR_NONE;
    for (idx = 0U; idx < n; idx++)
    {
        uint32_t const r = open_range(s[idx].range);
        float const    y = (ahead + (float)r) *
                           app_sinf((float)s[idx].deg * DEG_TO_RAD);

        if ((r >= CLEAR_LOOK_MM) || body_hit(s[idx].range, closest))
        {
            continue;
        }
        if ((y > y_l) && ((uint32_t)(y - y_l) < g_oc.clear_left))
        {
            g_oc.clear_left = (uint32_t)(y - y_l);
        }
        if ((y < y_r) && ((uint32_t)(y_r - y) < g_oc.clear_right))
        {
            g_oc.clear_right = (uint32_t)(y_r - y);
        }
    }
    /* Still on the obstacle at the last step: that edge is further out. */
    if (b_open_l)
    {
        y_l += (float)MARGIN_MM;
        g_oc.clear_left = 0U;
    }
    if (b_open_r)
    {
        y_r -= (float)MARGIN_MM;
        g_oc.clear_right = 0U;
    }

    g_oc.count = 1U;
    g_oc.samples[0].range = (uint16_t)closest;
    g_oc.samples[0].angle = (uint8_t)(90 + deg_c);
    g_oc.closest = closest;
    g_oc.bearing = (uint32_t)(90 + deg_c) / 10U;
    g_oc.y_left = (int32_t)y_l;
    g_oc.y_right = (int32_t)y_r;
    g_oc.width = (uint32_t)(y_l - y_r);
    tm_printf((UB *)"T5: body scan %d pings: width %d mm, closest %d,"
              " edges L%d R%d, clear L%d R%d\n", (INT)n, (INT)g_oc.width,
              (INT)closest, (INT)g_oc.y_left, (INT)g_oc.y_right,
              (INT)g_oc.clear_left, (INT)g_oc.clear_right);
    return (true);
}
#endif /* !SERVO_SCAN */

static void
handle_obstacle (void)
{
    uint16_t min_range = US_NO_RETURN_MM;
    uint32_t centre;
    bool     b_done = false;

    g_oc.retry = 0U;

    while (!b_done)
    {
        set_scan_state(SCAN_HALT);
        if (!move(MREQ_STOP, 0U))
        {
            break;
        }
        servo_sweep(false);     /* brake first, then face front */

#if !SERVO_SCAN
        (void)centre;
        (void)min_range;
        if (!fixed_profile())
        {
            g_oc.plan = PLAN_CONTINUE;      /* gone, or only the floor */
            publish_profile();
            b_done = true;
            break;
        }
#else
        centre = coarse_scan(&min_range);
        if (estop())
        {
            break;
        }

        if (floor_echo())
        {
            g_oc.plan = PLAN_CONTINUE;      /* the floor, not an obstacle */
            g_oc.closest = min_range;
            publish_profile();
            b_done = true;
            break;
        }

        if (min_range >= PROFILE_MM)
        {
            g_oc.plan = PLAN_CONTINUE;      /* spurious: nothing near */
            g_oc.closest = min_range;
            publish_profile();
            b_done = true;
            break;
        }

        fine_scan(centre);
        profile();
#endif

        switch (plan())
        {
            case PLAN_CONTINUE:
                b_done = true;
            break;

            case PLAN_LEFT:
            case PLAN_RIGHT:
                if (bypass(g_oc.plan))
                {
                    b_done = true;
                    break;
                }
                if (estop())
                {
                    break;
                }
                /* Line not found: treat as blocked. */
                set_scan_state(SCAN_BLOCKED);
            /* fall through */

            default:
                set_scan_state(SCAN_BLOCKED);
                /* Backing off to look again only helps if the sonar can
                   see further than the obstacle; the floor echo stops it
                   at ~floor_mm, so with a floor known it cannot. */
                if ((0U == g_oc.retry) && (0U == g_oc.floor_mm) &&
                    (0 != SERVO_SCAN))
                {
                    g_oc.retry = 1U;
                    set_scan_state(SCAN_REVERSE);
                    if (!move(MREQ_BACKWARD, REVERSE_MM))
                    {
                        b_done = true;
                    }
                }
                else
                {
                    tm_printf((UB *)"T5: no way round\n");
                    fault_set(FAULT_BYPASS_FAIL);
                    b_done = true;
                }
            break;
        }

        if (estop())
        {
            break;
        }
    }

    (void)servo_move(SERVO_CENTRE_DEG);
    set_scan_state(SCAN_WATCH);
    if (!estop())
    {
        (void)tk_set_flg(flg_sys, FLG_LINE_REACQ);
    }
}

static bool
watching (uint32_t state)
{
    return ((MISSION_FOLLOW == state) || (MISSION_BARCODE == state) ||
            (MISSION_MANOEUVRE == state) || (MISSION_HUMP == state));
}

EXPORT void obstacle_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    servo_init();
    set_scan_state(SCAN_WATCH);

    for (;;)
    {
        uint16_t const range = ping();      /* paced at 70 ms */
        uint32_t const state = mission_state();
        bool           b_approach;
        bool const     b_cal = (MISSION_INIT == state) ||
                               (MISSION_CALIBRATE == state);

        /* Calibration: the last obstacle's measurements read zero. */
        if (b_cal && !g_oc.b_in_cal)
        {
            g_oc.width = 0U;
            g_oc.closest = 0U;
            g_oc.bearing = 0U;
            g_oc.plan = 0U;
            g_oc.clear_left = 0U;
            g_oc.clear_right = 0U;
            g_oc.retry = 0U;
            g_oc.y_left = 0;
            g_oc.y_right = 0;
            publish_profile();
            tm_printf((UB *)"T5: obstacle width zeroed for calibration\n");
        }
        g_oc.b_in_cal = b_cal;

        if (MISSION_OBSTACLE == state)
        {
            handle_obstacle();
            g_oc.hits = 0U;
            g_oc.near_hist = 0U;
            g_oc.b_approach = false;
            g_oc.cooldown = COOLDOWN_PINGS;
            continue;
        }

        /* Radar sweep only while following the line; anywhere else the
           sonar faces front (servo_sweep(false) waits for that). */
        servo_sweep(watching(state));

        /* Floor or obstacle?  An obstacle comes nearer as the car drives
           at it; the floor echo of a low sonar stays the same distance
           ahead however far the car goes. */
        {
            uint16_t const odom = odom_now_mm();
            uint32_t const moved = odom_delta_mm(odom, g_oc.prev_odom);

            /* The verdict holds between comparisons, which may be several
               pings apart at low speed. */
            if ((moved >= APPROACH_MIN_MOVE_MM) && (range < PROFILE_MM) &&
                (g_oc.prev_range < PROFILE_MM))
            {
                int32_t const diff = (int32_t)g_oc.prev_range - (int32_t)range;

                g_oc.b_approach = (diff >= (int32_t)(moved / 2U));
                if ((diff <= STEADY_BAND_MM) && (diff >= -STEADY_BAND_MM))
                {
                    if ((++g_oc.steady >= STEADY_PINGS) && (0U == g_oc.floor_mm))
                    {
                        g_oc.floor_mm = range;
                        tm_printf((UB *)"T5: floor echo at %d mm, not an"
                                  " obstacle\n", (INT)range);
                    }
                }
                else
                {
                    g_oc.steady = 0U;
                }
            }
            if (moved >= APPROACH_MIN_MOVE_MM)
            {
                if ((range >= PROFILE_MM) || (g_oc.prev_range >= PROFILE_MM))
                {
                    g_oc.b_approach = false;    /* nothing in range */
                }
                g_oc.prev_odom = odom;
                g_oc.prev_range = range;
            }
            b_approach = g_oc.b_approach;
        }

        if (watching(state) && (range < EMERGENCY_MM))
        {
            /* Very close: one ping is enough, cooldown or not.  With a
               low sonar the floor hides anything beyond ~175 mm, so this
               is all the warning there is (bench 2026-11-02). */
            g_oc.hits = TRIGGER_HITS;
            g_oc.cooldown = 0U;
            tm_printf((UB *)"T5: obstacle at %d mm\n", (INT)range);
            (void)tk_set_flg(flg_sys, FLG_OBSTACLE);
        }
        else if (g_oc.cooldown > 0U)
        {
            g_oc.cooldown--;    /* an obstacle just judged clear */
        }
        else if (watching(state))
        {
            /* Two of the last three pings near and approaching: while the
               sonar sweeps, an obstacle off to one side is in the beam
               only part of the time, so "two in a row" would miss it. */
            uint32_t const near = ((range < trigger_mm()) && b_approach) ?
                                  1U : 0U;

            g_oc.near_hist = ((g_oc.near_hist << 1) | near) & 0x7U;
            g_oc.hits = (g_oc.near_hist & 1U) + ((g_oc.near_hist >> 1) & 1U) +
                        ((g_oc.near_hist >> 2) & 1U);
            if ((0U != near) && (g_oc.hits >= TRIGGER_HITS))
            {
                tm_printf((UB *)"T5: obstacle at %d mm\n", (INT)range);
                (void)tk_set_flg(flg_sys, FLG_OBSTACLE);
                g_oc.near_hist = 0U;
            }
        }
        else
        {
            g_oc.near_hist = 0U;
            g_oc.hits = 0U;
        }
    }
}

/*** end of file ***/
