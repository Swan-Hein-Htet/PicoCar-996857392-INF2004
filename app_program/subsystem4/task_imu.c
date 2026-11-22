/** @file task_imu.c
 *
 * @brief T3 ImuTerrain: pitch, hump detection and peak height, motion
 *        classification, impact detection.  Released every 10 ms by a
 *        cyclic handler (NFR4).
 *
 * Kit IMU is an LSM303DLHC (no gyro), so this is the tribunal's
 * conditional redesign of I2 FLAT:
 *   - pitch from the gravity vector, after removing the vehicle's own
 *     forward acceleration (derived from T4's wheel speeds; 0.1 g of
 *     acceleration would otherwise read as ~6 deg of pitch);
 *   - first-order low-pass, updated only when |a| is within 1 +- 0.1 g;
 *   - turn rate is kinematic: (v_right - v_left) / track.
 * Hump height h = sum(sin(pitch_rel) * delta_d), assumption A6.
 */

#include "subsystem4.h"
#include "../app_math.h"
#include "drv_lsm303.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"

#define BIAS_SAMPLES            200U    /* 2 s stationary (I1)           */
#define BIAS_MOTION_MG          40      /* |a| off its running mean: moved */
#define BIAS_G_MIN_MG           700     /* |a| outside this: not a sane    */
#define BIAS_G_MAX_MG           1300    /* reading of gravity at all       */
#define BIAS_REPORT_CYCLES      500U    /* waiting: say why every 5 s      */
#define GATE_BAND_MG            100     /* pitch only within g_ref +- this */
#define LEVEL_SAMPLES           50U     /* calibration: 0.5 s still re-zeroes */
#define LEVEL_STILL_MMS         5.0f    /* wheels stopped: not sweeping      */
#ifndef IMPACT_STOP
#define IMPACT_STOP             0       /* collision (IMPACT) detection off:
                                           owner 2026-11-19 (D75), the car
                                           stops for wheel lift-off only;
                                           obstacle avoidance unchanged  */
#endif
#define IMPACT_MG               3000
#define IMPACT_SAMPLES          2U
#define I2C_FAIL_LIMIT          3U
#define INIT_RETRY_CYCLES       100U    /* retry the IMU every 1 s       */
#define DIAG_EVERY_TRIES        10U     /* say why every 10 s            */
/* Picked up (end of the demo): an upward push above gravity, or a tilt
   no hump gives (humps pitch the nose, they do not roll the car). */
#define LIFT_Z_ALPHA            0.2f    /* Z average, about 50 ms         */
#define LIFT_Z_MG               300.0f  /* above flat-ground gravity      */
#define LIFT_Z_SAMPLES          5U      /* 50 ms                          */
#define LIFT_ROLL_DEG           25.0f
#define LIFT_PITCH_DEG          35.0f   /* the 40 mm hump pitches < 20    */
#define LIFT_TOTAL_DEG          28.0f   /* tilt from flat, any direction:
                                           mounting-proof (D66); a 1 cm
                                           bump tilts the car ~21 at most */
#define HUMP_MAX_TILT_DEG       25.0f   /* tilted more: a hand, not a hump */
#define LIFT_TILT_SAMPLES       20U     /* 200 ms                         */
/* Which way X points: learnt from the wheels while the car speeds up or
   slows down (the GY-511 was found mounted upside down, 2026-11-19). */
#define XLEARN_ACCEL_MMS2       150.0f  /* use samples accelerating this hard */
#define XLEARN_SAMPLES          12U     /* a set-off gives about 15 */
#define XLEARN_CORR             0.6f    /* strong agreement only: a wrong early
                                           guess would hide humps; the first
                                           hump decides otherwise           */
#define XLEARN_Y_CORR           0.8f    /* Y forward: Y this strong ...    */
#define XLEARN_X_MAX            0.3f    /* ... and X this weak             */
#define XLEARN_TURN_DPS         15.0f   /* only while going straight        */
#define XLEARN_GIVE_UP          600U    /* samples: say X is not along the car */
#define DIAG_MAX_FOUND          8U

#define PITCH_ALPHA             0.15f
#define ACCEL_ALPHA             0.2f
#define HUMP_FLAT_DEG           2.0f
#define RISE_SEG_MM             10.0f   /* hump detector: height gained per */
#define RISE_SEGS               6U      /* 10 mm, over the last 60 mm       */
#define HUMP_RISE_MM            4.0f    /* gained over 60 mm: a hump (D66)  */
#define HUMP_DROP_MM            2.0f    /* below the peak by this: going down */
#define HUMP_END_FRAC           0.3f    /* down to this share of the peak: off */
#define CUM_FORGET              0.03f   /* per 10 mm: low point kept ~300 mm */
#define BASE_ALPHA              0.005f  /* flat-ground pitch baseline, ~2 s */
#define BASE_LIMIT_DEG          4.0f    /* a baseline past this is a slope */
#define HUMP_SAMPLES            3U
#define HUMP_MAX_SPAN_MM        600.0f  /* give up waiting for a descent */
#define PITCH_LIMIT_DECI        450     /* +-45.0 deg                    */
#define RATE_LIMIT_DPS          500
#define TURNING_DPS             30.0f
#define ACCEL_CLASS_MMS2        150.0f
#define MOVING_MMS              20.0f

#define RAD_TO_DEG              57.29578f
#define DEG_TO_RAD              0.01745329f
#define MG_PER_MMS2             0.10197f    /* 1000 / 9806.65            */
#define DT_S                    0.01f

EXPORT volatile uint32_t g_terrain_word = 0U;
EXPORT volatile uint8_t  g_hump_byte = 0U;
EXPORT volatile uint32_t g_tilt_word = 0U;

typedef struct
{
    uint32_t state;         /* I0-I7                                  */
    uint32_t count;         /* consecutive-sample / calibration count */
    uint32_t i2c_fails;
    uint32_t impact_count;
    uint32_t retry;
    uint32_t init_tries;    /* failed starts, for the diagnosis  */
    float    sum_x;
    float    sum_y;
    float    sum_z;
    float    sum_mag;       /* bias: running |a| sum              */
    float    g_ref;         /* |gravity| as this part reads it    */
    uint32_t bias_wait;     /* cycles waiting to calibrate        */
    float    pitch0;        /* flat-ground reference, deg             */
    float    z0;            /* flat-ground gravity on Z, mg           */
    float    g0x;           /* flat-ground gravity vector, mg: the    */
    float    g0y;           /* "up" for the lift test, whichever way  */
    float    g0z;           /* the board is mounted                   */
    float    x_sign;        /* +1 X forward, -1 X backward (learnt)   */
    float    xc_sum;        /* X-versus-wheel acceleration evidence   */
    float    xc_aa;
    float    xc_dd;
    uint32_t xc_n;
    bool     b_x_known;
    bool     b_y_fwd;       /* board Y runs along the car (turned 90 deg):
                               readings are rotated so "X" is forward  */
    float    yc_sum;        /* Y-versus-wheel acceleration evidence   */
    float    yc_dd;
    float    side;          /* filtered tilt about the other axis, deg */
    float    base;          /* pitch on flat ground while driving, deg: */
    float    base_side;     /* humps are measured from it (D65)        */
    float    seg_x;         /* hump detector: height gained in the   */
    float    seg_y;         /* current 10 mm, each axis, mm          */
    float    seg_mm;
    float    win_x[RISE_SEGS];  /* ... and in the last RISE_SEGS     */
    float    win_y[RISE_SEGS];
    uint32_t win_i;
    float    rise;          /* largest gain over the window now, mm  */
    float    cum;           /* height summed along the way (forward  */
    float    cum_min;       /* axis) and its recent lowest, mm: the  */
                            /* climb is counted from that low point  */
    float    hump_x;        /* first hump, axis unknown: tilt seen on */
    float    hump_y;        /* each axis while confirming             */
    int32_t  hump_sign_y;
    int32_t  hump_sign;     /* first slope: + up, - down, while X unknown */
    float    roll0;         /* flat-ground roll, deg                  */
    float    z_avg;         /* Z, about 50 ms average, mg             */
    uint32_t lift_up;       /* samples pushed upwards                 */
    uint32_t lift_tilt;     /* samples tilted past a hump             */
    bool     b_lifted;
    float    pitch;         /* filtered relative pitch, deg           */
    float    accel_mms2;    /* filtered forward acceleration          */
    float    speed_prev;
    float    height;        /* current hump relative height, mm       */
    float    peak;          /* this hump's peak, mm                   */
    float    span;          /* distance since the hump started, mm    */
    float    confirm_mm;    /* distance climbing so far in I2 FLAT    */
    float    confirm_h;     /* height gained over that distance       */
    uint16_t odom_prev;
    uint8_t  peak_max;      /* run maximum, mm                        */
    float    roll_rel;      /* roll from flat, deg (+-180)            */
    float    tilt_total;    /* angle from the flat gravity vector, deg */
    uint32_t mission_prev;  /* to reset peak_max when a run starts    */
    uint32_t lv_n;          /* calibration re-zero: still samples     */
    float    lv_x;
    float    lv_y;
    float    lv_z;
    float    lv_mag;
    bool     b_lv_said;     /* re-zeroed this calibration             */
    bool     b_in_cal;      /* in S0/S1, or S2 READY straight after   */
} imu_ctx_t;

static imu_ctx_t g_imu;

static float
speed_of (uint32_t word, uint32_t shift)
{
    return ((float)bits_to_signed(bits_get(word, shift, MOTION_SPEED_BITS),
                                  MOTION_SPEED_BITS));
}

static uint32_t
classify (float rate_dps, float speed)
{
    uint32_t mclass = MCLASS_STATIONARY;

    if (IMU_IMPACT == g_imu.state)
    {
        mclass = MCLASS_IMPACT;
    }
    else if ((IMU_ASCEND == g_imu.state) || (IMU_CREST == g_imu.state))
    {
        mclass = MCLASS_CLIMBING;
    }
    else if (IMU_DESCEND == g_imu.state)
    {
        mclass = MCLASS_DESCENDING;
    }
    else if (app_fabsf(rate_dps) > TURNING_DPS)
    {
        mclass = MCLASS_TURNING;
    }
    else if (app_fabsf(g_imu.accel_mms2) > ACCEL_CLASS_MMS2)
    {
        mclass = MCLASS_ACCELERATING;
    }
    else if (app_fabsf(speed) > MOVING_MMS)
    {
        mclass = MCLASS_DRIVING;
    }
    else
    {
        mclass = MCLASS_STATIONARY;
    }

    return (mclass);
}

/* Board turned 90 deg (Y along the car): rotate the reading so the rest
   of T3 sees X forward.  A rotation, not a swap, so roll keeps its hand. */
static void
orient (int32_t * p_ax, int32_t * p_ay)
{
    if (g_imu.b_y_fwd)
    {
        int32_t const x = *p_ax;

        *p_ax = *p_ay;
        *p_ay = -x;
    }
}

/* Level references from the flat-ground gravity vector (already
   oriented). */
static void
level_refs (void)
{
    float const mx = g_imu.g0x;
    float const my = g_imu.g0y;
    float const mz = g_imu.g0z;

    g_imu.pitch0 = app_atan2f(mx, app_sqrtf((my * my) + (mz * mz))) *
                   RAD_TO_DEG;
    g_imu.roll0  = app_atan2f(my, mz) * RAD_TO_DEG;
}

/* Tilt about the board axis that is not "forward" now, relative to flat:
   on a board turned 90 deg this is where a hump shows (as roll). */
static float
side_tilt (int32_t ax, int32_t ay, int32_t az)
{
    float const now = app_atan2f((float)ay,
                                 app_sqrtf(((float)ax * (float)ax) +
                                           ((float)az * (float)az)));
    float const flat = app_atan2f(g_imu.g0y,
                                  app_sqrtf((g_imu.g0x * g_imu.g0x) +
                                            (g_imu.g0z * g_imu.g0z)));

    return ((now - flat) * RAD_TO_DEG);
}

/* Y runs along the car: from now on the rotated reading is used, so the
   hump is measured from what was the roll.  sign: +1 if +Y is forward. */
static void
use_y_forward (float sign, UB const * p_why)
{
    float const gx = g_imu.g0x;

    g_imu.b_y_fwd = true;
    g_imu.g0x = g_imu.g0y;              /* same rotation as orient() */
    g_imu.g0y = -gx;
    level_refs();
    g_imu.x_sign = sign;
    g_imu.pitch = sign * g_imu.side;
    g_imu.base = sign * g_imu.base_side;
    g_imu.side = 0.0f;
    g_imu.base_side = 0.0f;
    tm_printf((UB *)"T3: IMU board turned 90 deg: Y axis points %s along the"
              " car (%s) - hump height now taken from the board's roll\n",
              (sign > 0.0f) ? "forward" : "backward", p_why);
}

/* Which way the board's X axis points along the car.  Speeding up or
   slowing down (from the wheels) shows on X as +accel if X points
   forward: correlate the two over the first hard samples on flat ground.
   Until then x_sign = +1, as mounted per docs/WIRING.md. */
static void
xlearn_step (int32_t ax, int32_t ay, float rate_dps)
{
    float const a = g_imu.accel_mms2 * MG_PER_MMS2;     /* expected, mg */
    float const d = (float)ax - g_imu.g0x;              /* seen on X    */
    float const dy = (float)ay - g_imu.g0y;             /* seen on Y    */

    /* Straight only: turning on the spot (the calibration sweep) shows on
       X as turning forces, and its wheel speeds made the "forward
       acceleration" jump; with those samples the two never agreed. */
    if (g_imu.b_x_known || (IMU_FLAT != g_imu.state) ||
        (app_fabsf(rate_dps) > XLEARN_TURN_DPS) ||
        (app_fabsf(g_imu.accel_mms2) < XLEARN_ACCEL_MMS2))
    {
        return;
    }
    g_imu.xc_sum += a * d;
    g_imu.xc_aa += a * a;
    g_imu.xc_dd += d * d;
    g_imu.yc_sum += a * dy;
    g_imu.yc_dd += dy * dy;
    g_imu.xc_n++;
    if (g_imu.xc_n >= XLEARN_SAMPLES)
    {
        float const corr = g_imu.xc_sum /
                           app_sqrtf((g_imu.xc_aa * g_imu.xc_dd) + 1.0f);
        float const corr_y = g_imu.yc_sum /
                             app_sqrtf((g_imu.xc_aa * g_imu.yc_dd) + 1.0f);

        /* The board may be turned 90 deg: then Y follows the wheels.
           Only on a clear win: Y 60 % against X 56 % once turned a
           correctly mounted board sideways and every later hump was
           measured on the wrong axis (sim full_course, 2026-11-19).  An
           unclear result waits for the first hump instead. */
        if ((app_fabsf(corr_y) >= XLEARN_Y_CORR) &&
            (app_fabsf(corr) < XLEARN_X_MAX))
        {
            g_imu.b_x_known = true;
            tm_printf((UB *)"T3: IMU Y follows the car's acceleration"
                      " (agreement %d %%, X %d %%)\n",
                      (INT)(app_fabsf(corr_y) * 100.0f),
                      (INT)(app_fabsf(corr) * 100.0f));
            use_y_forward((corr_y > 0.0f) ? 1.0f : -1.0f,
                          (UB const *)"from the wheels");
        }
        else if (app_fabsf(corr) >= XLEARN_CORR)
        {
            g_imu.b_x_known = true;
            g_imu.x_sign = (corr > 0.0f) ? 1.0f : -1.0f;
            tm_printf((UB *)"T3: IMU X axis points %s (learnt from %d samples,"
                      " agreement %d %%)%s\n",
                      (corr > 0.0f) ? "forward" : "BACKWARD", (INT)g_imu.xc_n,
                      (INT)(app_fabsf(corr) * 100.0f),
                      (corr > 0.0f) ? "" : ": pitch flipped");
        }
        else if (g_imu.xc_n >= XLEARN_GIVE_UP)
        {
            g_imu.b_x_known = true;         /* keep +1, say why */
            tm_printf((UB *)"T3: IMU X axis does not follow the car's"
                      " acceleration (agreement %d %%, Y %d %%): mount the"
                      " board with X or Y along the car\n",
                      (INT)(app_fabsf(corr) * 100.0f),
                      (INT)(app_fabsf(corr_y) * 100.0f));
        }
    }
}

/* Picked up: raise FAULT_LIFTED (critical: T6 stops the motors and holds
   until RESET).  Re-armed once RESET has cleared it. */
static void
lift_step (int32_t ax, int32_t ay, int32_t az, float pitch_rel)
{
    float       roll = (app_atan2f((float)ay, (float)az) * RAD_TO_DEG) -
                       g_imu.roll0;
    /* Acceleration along the calibrated gravity: picking the car up pushes
       it "up" whichever way the board is mounted (Z only missed this with
       the board upside down). */
    float const along = (((float)ax * g_imu.g0x) + ((float)ay * g_imu.g0y) +
                         ((float)az * g_imu.g0z)) / g_imu.g_ref;
    bool const  b_hump = (IMU_ASCEND <= g_imu.state) &&
                         (IMU_DESCEND >= g_imu.state);
    float const mag = app_sqrtf(((float)ax * (float)ax) +
                                ((float)ay * (float)ay) +
                                ((float)az * (float)az));
    float       cosine = (mag > 1.0f) ? (along / mag) : 1.0f;
    bool        b_up;
    bool        b_tilt;

    /* Tilt from flat in any direction: does not depend on which board
       axis was taken as forward (a tilt in the hand once taught it the
       wrong axis, and its roll then read as pitch under the 35 deg
       limit: tests/sim lifted_tilt). */
    cosine = (cosine > 1.0f) ? 1.0f : ((cosine < -1.0f) ? -1.0f : cosine);
    g_imu.tilt_total = app_atan2f(app_sqrtf(1.0f - (cosine * cosine)),
                                  cosine) * RAD_TO_DEG;

    /* Roll relative to the resting roll, wrapped into +-180: upside down
       the resting roll is near 180, and a wobble across it read as a
       350 deg roll and a false LIFTED (2026-11-19). */
    while (roll > 180.0f)
    {
        roll -= 360.0f;
    }
    while (roll <= -180.0f)
    {
        roll += 360.0f;
    }
    g_imu.roll_rel = roll;

    if (g_imu.b_lifted)
    {
        if (0U == ((uint32_t)fault_get() & FAULT_LIFTED))
        {
            g_imu.b_lifted = false;
            g_imu.lift_up = 0U;
            g_imu.lift_tilt = 0U;
            g_imu.z_avg = g_imu.g_ref;
        }
        return;
    }
    g_imu.z_avg += LIFT_Z_ALPHA * (along - g_imu.z_avg);
    /* Not on a hump: its ramps really push the car up for a moment. */
    b_up = ((g_imu.z_avg - g_imu.g_ref) > LIFT_Z_MG) && !b_hump;
    b_tilt = (app_fabsf(roll) > LIFT_ROLL_DEG) ||
             (app_fabsf(pitch_rel) > LIFT_PITCH_DEG) ||
             (g_imu.tilt_total > LIFT_TOTAL_DEG);
    g_imu.lift_up = b_up ? (g_imu.lift_up + 1U) : 0U;
    g_imu.lift_tilt = b_tilt ? (g_imu.lift_tilt + 1U) : 0U;
    if ((g_imu.lift_up >= LIFT_Z_SAMPLES) ||
        (g_imu.lift_tilt >= LIFT_TILT_SAMPLES))
    {
        g_imu.b_lifted = true;
        fault_set(FAULT_LIFTED);
        tm_printf((UB *)"T3: LIFTED (%s, up %d mg, roll %d deg) - motors"
                  " stopped, end of the demo\n",
                  (UB *)((g_imu.lift_up >= LIFT_Z_SAMPLES) ? "raised" :
                         "tilted"),
                  (INT)(g_imu.z_avg - g_imu.g_ref), (INT)roll);
    }
}

/* Nothing answered: say what the bus looks like, so the cause can be told
   apart (no power or a short: lines low; wrong part or address: other
   addresses answer; nothing on the bus: no answers at all). */
static void
imu_diagnose (void)
{
    uint8_t        found[DIAG_MAX_FOUND];
    uint32_t const lines = lsm303_lines();
    uint32_t const n = lsm303_scan(found, DIAG_MAX_FOUND);
    uint32_t       idx;

    tm_printf((UB *)"T3: no accelerometer at 0x19/0x18 (LSM303DLHC/AGR),"
              " 0x1D/0x1E (LSM303D) or 0x68/0x69 (MPU-6050)\n");
    tm_printf((UB *)"T3: I2C idle SDA(GP%d) %s, SCL(GP%d) %s%s\n",
              (INT)PIN_IMU_SDA, (0U != (lines & 1U)) ? "high" : "LOW",
              (INT)PIN_IMU_SCL, (0U != (lines & 2U)) ? "high" : "LOW",
              (3U == lines) ? "" : " - check VIN/GND, and SDA/SCL not swapped");
    if (0U == n)
    {
        tm_printf((UB *)"T3: no device answers on I2C0: check wiring to"
                  " Grove 3 and the board's power\n");
    }
    for (idx = 0U; idx < n; idx++)
    {
        tm_printf((UB *)"T3: device answering at 0x%02x\n", (INT)found[idx]);
    }
}

/* Tell T6 about a hump only while it follows the line: a "hump" felt
   while parked or in a hand (the car tilted 20 deg with its wheels
   turning, real car 2026-11-19) left FLG_HUMP_START set, and the next
   START went straight into S6 and missed the real hump. */
static void
hump_flag (UINT flag)
{
    uint32_t const m = mission_state();

    if ((MISSION_FOLLOW == m) || (MISSION_BARCODE == m) ||
        (MISSION_HUMP == m))
    {
        (void)tk_set_flg(flg_sys, flag);
    }
}

static void
hump_report (void)
{
    float    peak = g_imu.peak + 0.5f;
    uint32_t peak_mm = 0U;
    uint32_t byte;

    if (peak > 0.0f)
    {
        peak_mm = bits_sat_u((uint32_t)peak, HUMP_PEAK_BITS);
    }
    if (peak_mm > (uint32_t)g_imu.peak_max)
    {
        g_imu.peak_max = (uint8_t)peak_mm;
    }

    byte = bits_put(0U, 0U, HUMP_PEAK_BITS, g_imu.peak_max);
    byte = bits_put(byte, HUMP_SEEN_SHIFT, 1U, 1U);
    g_hump_byte = (uint8_t)byte;

    tm_printf((UB *)"T3: hump peak %d mm (run max %d)\n",
              (INT)peak_mm, (INT)g_imu.peak_max);
    hump_flag(FLG_HUMP_END);
}

/* Hump state machine I2-I6 on the filtered relative pitch. */
/* Hump detector window empty (after a hump, calibration, a lift). */
static void
hump_window_clear (void)
{
    uint32_t k;

    for (k = 0U; k < RISE_SEGS; k++)
    {
        g_imu.win_x[k] = 0.0f;
        g_imu.win_y[k] = 0.0f;
    }
    g_imu.seg_x = 0.0f;
    g_imu.seg_y = 0.0f;
    g_imu.seg_mm = 0.0f;
    g_imu.rise = 0.0f;
    g_imu.cum = 0.0f;
    g_imu.cum_min = 0.0f;
}

static void
hump_step (float delta_mm)
{
    /* From the driving baseline: the real car read -1.5 deg parked and
       about -2.5 driving, and a 1 cm hump (+3.7) never reached 3 deg. */
    float const rel = g_imu.pitch - g_imu.base;

    /* Lifted (or being lifted): tilting the car in a hand is no hump.  A
       hump measured that way stuck at 27 mm as the run maximum (real car
       2026-11-19).  Drop any hump in progress, measure nothing. */
    if (g_imu.b_lifted || (0U != ((uint32_t)fault_get() & FAULT_LIFTED)))
    {
        if (IMU_FLAT != g_imu.state)
        {
            g_imu.state = IMU_FLAT;
            g_imu.height = 0.0f;
            g_imu.peak = 0.0f;
        }
        g_imu.count = 0U;
        g_imu.confirm_mm = 0.0f;
        g_imu.confirm_h = 0.0f;
        g_imu.hump_sign = 0;
        g_imu.hump_sign_y = 0;
        g_imu.hump_x = 0.0f;
        g_imu.hump_y = 0.0f;
        hump_window_clear();
        return;
    }
    /* A new run (Go from S2 or S9): the run maximum starts again. */
    {
        uint32_t const m = mission_state();

        if ((MISSION_FOLLOW == m) && ((MISSION_READY == g_imu.mission_prev) ||
                                      (MISSION_FINISHED == g_imu.mission_prev)))
        {
            g_imu.peak_max = 0U;
            g_hump_byte = 0U;
            if (IMU_FLAT != g_imu.state)
            {
                g_imu.state = IMU_FLAT;     /* one felt while parked */
                g_imu.height = 0.0f;
                g_imu.peak = 0.0f;
            }
        }
        g_imu.mission_prev = m;
    }

    if ((IMU_ASCEND <= g_imu.state) && (IMU_DESCEND >= g_imu.state))
    {
        g_imu.height += app_sinf(rel * DEG_TO_RAD) * delta_mm;
        g_imu.span   += delta_mm;
        if (g_imu.height > g_imu.peak)
        {
            g_imu.peak = g_imu.height;
        }
    }

    switch (g_imu.state)
    {
        case IMU_FLAT:
            /* Detect by height, not by angle (D66): add up sin(pitch) x
               distance over the last 60 mm.  Vibration (the real car's
               pitch swings +-2-4 deg while driving) cancels out in the
               sum; a real climb adds up.  A 1 cm hump gains about 10 mm,
               flat ground well under 2.  The axis not known yet: both
               board axes are summed, the larger gain wins (the board may
               be turned 90 deg) and its sign tells which way the axis
               points (humps start uphill). */
        {
            float const side = g_imu.side - g_imu.base_side;
            bool const  b_known0 = g_imu.b_x_known;
            float       rx = 0.0f;
            float       ry = 0.0f;
            uint32_t    k;

            g_imu.seg_x += app_sinf(rel * DEG_TO_RAD) * delta_mm;
            g_imu.seg_y += app_sinf(side * DEG_TO_RAD) * delta_mm;
            g_imu.seg_mm += delta_mm;
            if (g_imu.seg_mm < RISE_SEG_MM)
            {
                break;
            }
            g_imu.win_x[g_imu.win_i] = g_imu.seg_x;
            g_imu.win_y[g_imu.win_i] = g_imu.seg_y;
            g_imu.win_i = (g_imu.win_i + 1U) % RISE_SEGS;
            /* The low point the climb started from, forgotten slowly
               (a gentle start slides out of the 60 mm window first). */
            g_imu.cum += g_imu.seg_x;   /* used once the axis is known */
            if (g_imu.cum < g_imu.cum_min)
            {
                g_imu.cum_min = g_imu.cum;
            }
            g_imu.cum_min += CUM_FORGET * (g_imu.cum - g_imu.cum_min);
            g_imu.seg_x = 0.0f;
            g_imu.seg_y = 0.0f;
            g_imu.seg_mm = 0.0f;
            for (k = 0U; k < RISE_SEGS; k++)
            {
                rx += g_imu.win_x[k];
                ry += g_imu.win_y[k];
            }
            if (g_imu.b_x_known)
            {
                g_imu.rise = rx;
            }
            else
            {
                g_imu.rise = (app_fabsf(rx) >= app_fabsf(ry)) ?
                             app_fabsf(rx) : app_fabsf(ry);
            }
            if ((g_imu.rise < HUMP_RISE_MM) ||
                (g_imu.tilt_total > HUMP_MAX_TILT_DEG))
            {
                break;      /* flat, or tilted like a car in a hand */
            }
            if (!g_imu.b_x_known && (app_fabsf(ry) > app_fabsf(rx)))
            {
                g_imu.b_x_known = true;
                use_y_forward((ry >= 0.0f) ? 1.0f : -1.0f,
                              (UB const *)"from the first hump: humps"
                              " start uphill");
            }
            else if (!g_imu.b_x_known)
            {
                float const flip = (rx >= 0.0f) ? 1.0f : -1.0f;

                g_imu.x_sign *= flip;
                g_imu.pitch *= flip;
                g_imu.base *= flip;
                g_imu.b_x_known = true;
                tm_printf((UB *)"T3: IMU X axis points %s (from the first"
                          " hump: humps start uphill)%s\n",
                          (g_imu.x_sign > 0.0f) ? "forward" : "BACKWARD",
                          (flip < 0.0f) ? ": pitch flipped" : "");
            }
            else
            {
                /* Axis known. */
            }
            g_imu.state  = IMU_ASCEND;
            g_imu.count  = 0U;
            /* Climbed so far: from the low point, if that saw more. */
            g_imu.height = (b_known0 &&
                            ((g_imu.cum - g_imu.cum_min) > g_imu.rise)) ?
                           (g_imu.cum - g_imu.cum_min) : g_imu.rise;
            g_imu.peak   = g_imu.height;
            g_imu.span   = RISE_SEG_MM * (float)RISE_SEGS;
            hump_window_clear();
            hump_flag(FLG_HUMP_START);
        }
        break;

        case IMU_ASCEND:
        case IMU_CREST:
            /* Going down once the height has dropped clearly below the
               peak (a small hump hardly tilts the car on the way down). */
            if ((g_imu.peak - g_imu.height) >=
                ((HUMP_DROP_MM > (0.3f * g_imu.peak)) ? HUMP_DROP_MM :
                 (0.3f * g_imu.peak)))
            {
                g_imu.state = IMU_DESCEND;
                g_imu.count = 0U;
            }
            else
            {
                g_imu.state = (rel < HUMP_FLAT_DEG) ? IMU_CREST : IMU_ASCEND;
            }
        break;

        case IMU_DESCEND:
            g_imu.count = (app_fabsf(rel) < HUMP_FLAT_DEG) ?
                          (g_imu.count + 1U) : 0U;
            if ((g_imu.count >= HUMP_SAMPLES) ||
                (g_imu.height <= (HUMP_END_FRAC * g_imu.peak)))
            {
                g_imu.state = IMU_REPORT;
            }
        break;

        case IMU_REPORT:
            hump_report();
            hump_window_clear();
            g_imu.state = IMU_FLAT;
            g_imu.count = 0U;
        break;

        default:
            /* I0, I1, I7 are handled by the caller. */
        break;
    }

    /* A ramp with no descent within the span is reported as it stands. */
    if (((IMU_ASCEND == g_imu.state) || (IMU_CREST == g_imu.state)) &&
        (g_imu.span > HUMP_MAX_SPAN_MM))
    {
        g_imu.state = IMU_REPORT;
    }
}

static void
publish (float rate_dps, float speed)
{
    int32_t  pitch_deci = (int32_t)(g_imu.pitch * 10.0f);
    int32_t  rate = (int32_t)rate_dps;
    uint32_t word = 0U;
    bool     b_hump = (IMU_ASCEND <= g_imu.state) &&
                      (IMU_REPORT >= g_imu.state);

    if (pitch_deci > PITCH_LIMIT_DECI)
    {
        pitch_deci = PITCH_LIMIT_DECI;
    }
    else if (pitch_deci < -PITCH_LIMIT_DECI)
    {
        pitch_deci = -PITCH_LIMIT_DECI;
    }
    else
    {
        /* In range. */
    }
    if (rate > RATE_LIMIT_DPS)
    {
        rate = RATE_LIMIT_DPS;
    }
    else if (rate < -RATE_LIMIT_DPS)
    {
        rate = -RATE_LIMIT_DPS;
    }
    else
    {
        /* In range. */
    }

    word = bits_put(word, TERRAIN_PITCH_SHIFT, TERRAIN_PITCH_BITS,
                    bits_from_signed(pitch_deci, TERRAIN_PITCH_BITS));
    word = bits_put(word, TERRAIN_RATE_SHIFT, TERRAIN_RATE_BITS,
                    bits_from_signed(rate, TERRAIN_RATE_BITS));
    word = bits_put(word, TERRAIN_CLASS_SHIFT, TERRAIN_CLASS_BITS,
                    classify(rate_dps, speed));
    word = bits_put(word, TERRAIN_IMU_SHIFT, TERRAIN_IMU_BITS, g_imu.state);
    word = bits_put(word, TERRAIN_HUMP_SHIFT, 1U, b_hump ? 1U : 0U);
    g_terrain_word = word;      /* one aligned store (rule R1) */

    {
        int32_t roll = (int32_t)g_imu.roll_rel;
        float   h = b_hump ? g_imu.height : 0.0f;
        uint32_t tilt;

        roll = (roll > 127) ? 127 : ((roll < -127) ? -127 : roll);
        h = (h < 0.0f) ? 0.0f : ((h > 63.0f) ? 63.0f : h);
        tilt = bits_put(0U, TILT_ROLL_SHIFT, TILT_ROLL_BITS,
                        bits_from_signed(roll, TILT_ROLL_BITS));
        tilt = bits_put(tilt, TILT_HNOW_SHIFT, TILT_HNOW_BITS,
                        (uint32_t)(h + 0.5f));
        g_tilt_word = tilt;
    }
}

/* I1: 2 s still.  "Still" is a steady |a| (within BIAS_MOTION_MG of its
   own running mean), not |a| = 1000 mg: real parts read gravity several
   per cent off (the car's LSM303D never calibrated against a 60 mg band
   round 1000, 2026-11-17, so hump height was never measured).  The mean
   |a| becomes g_ref, the gravity the pitch gate is centred on. */
static void
bias_step (int32_t ax, int32_t ay, int32_t az, float mag)
{
    bool const b_sane = (mag > (float)BIAS_G_MIN_MG) &&
                        (mag < (float)BIAS_G_MAX_MG);
    bool const b_steady = (0U == g_imu.count) ||
        (app_fabsf(mag - (g_imu.sum_mag / (float)g_imu.count)) <=
         (float)BIAS_MOTION_MG);

    g_imu.bias_wait++;
    if ((g_imu.bias_wait % BIAS_REPORT_CYCLES) == 0U)
    {
        tm_printf((UB *)"T3: still waiting to calibrate the IMU: |a| %d mg"
                  " (x %d y %d z %d)%s\n", (INT)mag, (INT)ax, (INT)ay,
                  (INT)az, b_sane ? " - keep the car still and flat" :
                  " - not a reading of 1 g: check the IMU's mounting and"
                  " scale");
    }
    if (!b_sane || !b_steady)
    {
        g_imu.count = 0U;       /* moved: start the 2 s again */
        g_imu.sum_x = 0.0f;
        g_imu.sum_y = 0.0f;
        g_imu.sum_z = 0.0f;
        g_imu.sum_mag = 0.0f;
        if (!b_sane)
        {
            return;
        }
    }

    g_imu.sum_mag += mag;
    g_imu.sum_x += (float)ax;
    g_imu.sum_y += (float)ay;
    g_imu.sum_z += (float)az;
    g_imu.count++;

    if (g_imu.count >= BIAS_SAMPLES)
    {
        float const n  = (float)BIAS_SAMPLES;
        float const mx = g_imu.sum_x / n;
        float const my = g_imu.sum_y / n;
        float const mz = g_imu.sum_z / n;

        /* Gravity is NOT subtracted: it is the pitch reference. */
        g_imu.pitch0 = app_atan2f(mx, app_sqrtf((my * my) + (mz * mz))) *
                       RAD_TO_DEG;
        g_imu.pitch  = 0.0f;
        g_imu.z0     = mz;
        g_imu.g_ref  = g_imu.sum_mag / n;
        g_imu.g0x    = mx;
        g_imu.g0y    = my;
        g_imu.g0z    = mz;
        g_imu.x_sign = 1.0f;
        g_imu.xc_sum = 0.0f;
        g_imu.xc_aa  = 0.0f;
        g_imu.xc_dd  = 0.0f;
        g_imu.xc_n   = 0U;
        g_imu.b_x_known = false;
        g_imu.yc_sum = 0.0f;
        g_imu.yc_dd = 0.0f;
        g_imu.side = 0.0f;
        g_imu.base = 0.0f;
        g_imu.base_side = 0.0f;
        g_imu.hump_sign = 0;
        g_imu.bias_wait = 0U;
        g_imu.z_avg  = g_imu.g_ref;
        g_imu.roll0  = app_atan2f(my, mz) * RAD_TO_DEG;
        g_imu.state  = IMU_FLAT;
        g_imu.count  = 0U;
        fault_clear(FAULT_IMU);
        tm_printf((UB *)"T3: IMU calibrated, flat pitch %d.%d deg, gravity"
                  " reads %d mg\n", (INT)g_imu.pitch0,
                  (INT)(app_fabsf(g_imu.pitch0 * 10.0f)) % 10,
                  (INT)g_imu.g_ref);
    }
}

/* Calibration (S0 INIT, S1 CALIBRATE): tilt and hump read zero.  The hump
   values clear once as calibration starts; then, the first time the car
   is still for 0.5 s (wheels stopped, steady |a|), where it stands
   becomes level, so pitch and roll read 0 from there.  Only once: the
   car is driven by hand while parked here, and a stop on a hump must not
   become "level".  Hump detection keeps running (manual humps).  The
   console sends START as soon as S1 is reached, leaving about 0.7 s
   before the sweep; if that was too short, S2 READY finishes the job
   before Go.  The learnt axes are kept: they depend on the mounting, not
   on the floor. */
static void
level_step (int32_t ax, int32_t ay, int32_t az, float mag, float speed,
            float rate_dps)
{
    uint32_t const m = mission_state();
    bool const     b_cal = (MISSION_INIT == m) || (MISSION_CALIBRATE == m);
    bool const     b_parked = b_cal || (MISSION_READY == m);
    bool           b_still;

    if (b_cal && !g_imu.b_in_cal)
    {
        /* Calibration starts: the run maximum and any hump clear. */
        g_imu.state = IMU_FLAT;
        g_imu.height = 0.0f;
        g_imu.peak = 0.0f;
        g_imu.span = 0.0f;
        g_imu.count = 0U;
        g_imu.confirm_mm = 0.0f;
        g_imu.confirm_h = 0.0f;
        g_imu.hump_sign = 0;
        g_imu.hump_sign_y = 0;
        g_imu.hump_x = 0.0f;
        g_imu.hump_y = 0.0f;
        hump_window_clear();
        g_imu.peak_max = 0U;
        g_hump_byte = 0U;
        g_imu.b_lv_said = false;
        tm_printf((UB *)"T3: hump height zeroed for calibration\n");
        g_imu.lv_n = 0U;
    }
    g_imu.b_in_cal = b_parked && (b_cal || g_imu.b_in_cal);
    if (!g_imu.b_in_cal || g_imu.b_lv_said)
    {
        return;
    }

    b_still = (app_fabsf(speed) < LEVEL_STILL_MMS) &&
              (app_fabsf(rate_dps) < LEVEL_STILL_MMS) &&
              !g_imu.b_lifted &&
              (0U == ((uint32_t)fault_get() & FAULT_LIFTED)) &&
              (mag > (float)BIAS_G_MIN_MG) && (mag < (float)BIAS_G_MAX_MG) &&
              ((0U == g_imu.lv_n) ||
               (app_fabsf(mag - (g_imu.lv_mag / (float)g_imu.lv_n)) <=
                (float)BIAS_MOTION_MG));
    if (!b_still)
    {
        g_imu.lv_n = 0U;
        g_imu.lv_x = 0.0f;
        g_imu.lv_y = 0.0f;
        g_imu.lv_z = 0.0f;
        g_imu.lv_mag = 0.0f;
        return;
    }
    g_imu.lv_x += (float)ax;
    g_imu.lv_y += (float)ay;
    g_imu.lv_z += (float)az;
    g_imu.lv_mag += mag;
    g_imu.lv_n++;
    if (g_imu.lv_n >= LEVEL_SAMPLES)
    {
        float const n  = (float)g_imu.lv_n;
        float const mx = g_imu.lv_x / n;
        float const my = g_imu.lv_y / n;
        float const mz = g_imu.lv_z / n;
        float const old_pitch = g_imu.pitch;
        float const old_roll = g_imu.roll_rel;

        g_imu.g_ref  = g_imu.lv_mag / n;
        g_imu.g0x    = mx;
        g_imu.g0y    = my;
        g_imu.g0z    = mz;
        level_refs();
        g_imu.side   = 0.0f;
        g_imu.base   = 0.0f;
        g_imu.base_side = 0.0f;
        g_imu.z0     = mz;
        g_imu.z_avg  = g_imu.g_ref;
        g_imu.pitch  = 0.0f;
        g_imu.roll_rel = 0.0f;
        g_imu.lv_n = 0U;
        g_imu.lv_x = 0.0f;
        g_imu.lv_y = 0.0f;
        g_imu.lv_z = 0.0f;
        g_imu.lv_mag = 0.0f;
        if (!g_imu.b_lv_said)
        {
            g_imu.b_lv_said = true;
            tm_printf((UB *)"T3: tilt and hump zeroed for calibration (was"
                      " pitch %d deg, roll %d deg)\n", (INT)old_pitch,
                      (INT)old_roll);
        }
    }
}

/* Wheel kinematics: speed, kinematic turn rate (no gyro on the kit IMU)
   and filtered forward acceleration.  Runs every cycle, IMU or not. */
static void
kinematics (float * p_rate, float * p_speed)
{
    uint32_t const motion = g_motion_word;
    float const    v_l = speed_of(motion, MOTION_SPEED_L_SHIFT);
    float const    v_r = speed_of(motion, MOTION_SPEED_R_SHIFT);
    float const    speed = 0.5f * (v_l + v_r);

    g_imu.accel_mms2 += ACCEL_ALPHA *
        ((((speed - g_imu.speed_prev) / DT_S)) - g_imu.accel_mms2);
    g_imu.speed_prev = speed;
    *p_rate = ((v_r - v_l) / MOTION_TRACK_MM) * RAD_TO_DEG;
    *p_speed = speed;
}

static void
imu_cycle (void)
{
    uint16_t const odom = odom_now_mm();
    float const    delta_mm = (float)odom_delta_mm(odom, g_imu.odom_prev);
    float          rate = 0.0f;
    float          speed = 0.0f;
    int32_t        ax = 0;
    int32_t        ay = 0;
    int32_t        az = 0;
    float          mag;

    g_imu.odom_prev = odom;
    kinematics(&rate, &speed);

    if (!lsm303_read_mg(&ax, &ay, &az))
    {
        g_imu.i2c_fails++;
        if (g_imu.i2c_fails >= I2C_FAIL_LIMIT)
        {
            fault_set(FAULT_IMU);
            g_imu.state = IMU_INIT;
            g_imu.retry = 0U;
        }
        publish(rate, speed);
        return;
    }
    g_imu.i2c_fails = 0U;
    orient(&ax, &ay);

    mag = app_sqrtf(((float)ax * (float)ax) + ((float)ay * (float)ay) +
                    ((float)az * (float)az));

    /* I7 IMPACT: two consecutive samples above 3 g.  Not while the IMU
       is still calibrating (I1): its first readings after power-up can
       be garbage, and one (10369 mg, 20 ms after a reboot) raised a
       false IMPACT that stopped the car (real car 2026-11-19, D74). */
    g_imu.impact_count = ((0 != IMPACT_STOP) && (IMU_BIAS != g_imu.state) &&
                          (mag > (float)IMPACT_MG)) ?
                         (g_imu.impact_count + 1U) : 0U;
    if ((g_imu.impact_count >= IMPACT_SAMPLES) &&
        (IMU_IMPACT != g_imu.state))
    {
        g_imu.state = IMU_IMPACT;
        fault_set(FAULT_IMPACT);
        tm_printf((UB *)"T3: IMPACT %d mg\n", (INT)mag);
    }

    if (IMU_BIAS == g_imu.state)
    {
        bias_step(ax, ay, az, mag);
    }
    else if (IMU_IMPACT == g_imu.state)
    {
        /* Leave I7 only once RESET has cleared the impact fault. */
        if (0U == ((uint32_t)fault_get() & FAULT_IMPACT))
        {
            g_imu.state = IMU_FLAT;
        }
    }
    else
    {
        float const ax_corr = (float)ax -
                              (g_imu.x_sign * g_imu.accel_mms2 * MG_PER_MMS2);
        float const lateral = app_sqrtf(((float)ay * (float)ay) +
                                        ((float)az * (float)az));
        float const tilt = app_atan2f(ax_corr, lateral);
        /* Nose up is + whichever way X points (x_sign, learnt below). */
        float const rel = g_imu.x_sign *
                          ((tilt * RAD_TO_DEG) - g_imu.pitch0);

        xlearn_step(ax, ay, rate);

        if (app_fabsf(mag - g_imu.g_ref) < (float)GATE_BAND_MG)
        {
            g_imu.pitch += PITCH_ALPHA * (rel - g_imu.pitch);
            g_imu.side += PITCH_ALPHA *
                          (side_tilt(ax, ay, az) - g_imu.side);
            /* Flat ground, going straight or still, no hump starting:
               follow the slow offset (floor, placement, driving). */
            if ((IMU_FLAT == g_imu.state) &&
                (g_imu.rise < (0.5f * HUMP_RISE_MM)) &&
                (app_fabsf(rate) < XLEARN_TURN_DPS) && !g_imu.b_lifted)
            {
                g_imu.base += BASE_ALPHA * (g_imu.pitch - g_imu.base);
                g_imu.base_side += BASE_ALPHA * (g_imu.side - g_imu.base_side);
                g_imu.base = (g_imu.base > BASE_LIMIT_DEG) ? BASE_LIMIT_DEG :
                    ((g_imu.base < -BASE_LIMIT_DEG) ? -BASE_LIMIT_DEG :
                     g_imu.base);
                g_imu.base_side = (g_imu.base_side > BASE_LIMIT_DEG) ?
                    BASE_LIMIT_DEG : ((g_imu.base_side < -BASE_LIMIT_DEG) ?
                                      -BASE_LIMIT_DEG : g_imu.base_side);
            }
        }
        lift_step(ax, ay, az, rel);
        hump_step(delta_mm);
        level_step(ax, ay, az, mag, speed, rate);
    }

    publish(rate, speed);
}

EXPORT void imu_task (INT stacd, void * p_exinf)
{
    float rate = 0.0f;
    float speed = 0.0f;

    (void)stacd;
    (void)p_exinf;

    g_imu.state = IMU_INIT;
    g_imu.odom_prev = odom_now_mm();

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);     /* released by the 10 ms waker */
        (void)tk_can_wup(TSK_SELF);

        if (IMU_INIT == g_imu.state)
        {
            if (0U == g_imu.retry)
            {
                if (lsm303_init())
                {
                    g_imu.state = IMU_BIAS;
                    g_imu.b_y_fwd = false;  /* raw axes until learnt */
                    g_imu.count = 0U;
                    g_imu.sum_x = 0.0f;
                    g_imu.sum_y = 0.0f;
                    g_imu.sum_z = 0.0f;
                    g_imu.sum_mag = 0.0f;
                    g_imu.bias_wait = 0U;
                    g_imu.i2c_fails = 0U;
                    tm_printf((UB *)"T3: %s at 0x%02x ready, hold still 2 s\n",
                              (UB *)lsm303_part(), (INT)lsm303_addr());
                }
                else
                {
                    fault_set(FAULT_IMU);
                    if (0U == (g_imu.init_tries % DIAG_EVERY_TRIES))
                    {
                        imu_diagnose();
                    }
                    g_imu.init_tries++;
                }
            }
            g_imu.retry = (g_imu.retry + 1U) % INIT_RETRY_CYCLES;
            kinematics(&rate, &speed);
            g_imu.odom_prev = odom_now_mm();
            publish(rate, speed);       /* turn rate still reported */
            continue;
        }

        imu_cycle();
    }
}

/*** end of file ***/
