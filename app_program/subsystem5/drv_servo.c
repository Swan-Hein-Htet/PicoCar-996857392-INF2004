/** @file drv_servo.c
 *
 * @brief 50 Hz servo PWM: clk_sys 125 MHz / 125 = 1 MHz, TOP 19999.
 */

#include "../app.h"
#include "drv_servo.h"

#define SERVO_SLICE             6U
#define PWM_DIV(slice)          (PWM_BASE + PWM_CHx_DIV + ((slice) * 0x14U))
#define PWM_DIV_1MHZ            (125U << 4U)    /* integer part, [11:4] */
#define SERVO_TOP               19999U
#define SERVO_SPAN_DEG          180U
#define SETTLE_MS_PER_30DEG     50U
#define SETTLE_MIN_MS           20U

static uint32_t g_servo_deg = SERVO_CENTRE_DEG;

#if !SERVO_SCAN
#define SWEEP_TICK_MS           5
#define SWEEP_HALF_MS           ((((int32_t)SERVO_LIMIT_DEG * 1000) /          \
                                  (int32_t)SERVO_SWEEP_DPS /                   \
                                  SWEEP_TICK_MS) * SWEEP_TICK_MS)
#define SWEEP_STOP_WAIT_MS      1000

/* Written by servo_sweep() (T5) and the sweep handler; the handler runs
   in interrupt context on the same core, so word stores are atomic. */
static volatile int32_t  g_sweep_pos = 0;       /* ms of swing from front */
static volatile int32_t  g_sweep_dir = 1;       /* +1 left, -1 right      */
static volatile bool     g_b_sweep_req = false;
static volatile bool     g_b_sweeping = false;

static void
sweep_pulse (int32_t dir)
{
    int32_t us = (int32_t)SERVO_STOP_US + (int32_t)SERVO_SWEEP_TRIM_US;

#if SERVO_MIRROR
    dir = -dir;
#endif
    us += dir * (int32_t)SERVO_SWEEP_DELTA_US;
    (void)pwm_set_cc(PIN_SERVO, (UW)us);
}

/* Every 5 ms: advance the timed position, turn round at the ends, and on
   a stop request stop exactly where the swing passes the front. */
static void
sweep_tick (void * p_exinf)
{
    (void)p_exinf;

    if (!g_b_sweeping)
    {
        return;
    }
    g_sweep_pos += g_sweep_dir * SWEEP_TICK_MS;
    if ((!g_b_sweep_req) && (0 == g_sweep_pos))
    {
        (void)pwm_set_cc(PIN_SERVO, SERVO_STOP_US);
        g_b_sweeping = false;
        return;
    }
    if (g_sweep_pos >= SWEEP_HALF_MS)
    {
        g_sweep_dir = -1;
    }
    else if (g_sweep_pos <= -SWEEP_HALF_MS)
    {
        g_sweep_dir = 1;
    }
    else
    {
        /* Mid-swing. */
    }
    sweep_pulse(g_sweep_dir);
}
#endif /* !SERVO_SCAN */

#if SERVO_SCAN
static uint32_t
deg_to_us (uint32_t degrees)
{
    int32_t trimmed = (int32_t)degrees - (int32_t)SERVO_TRIM_DEG;

    if (trimmed < 0)
    {
        trimmed = 0;
    }
    else if (trimmed > (int32_t)SERVO_SPAN_DEG)
    {
        trimmed = (int32_t)SERVO_SPAN_DEG;
    }
    else
    {
        /* Within travel. */
    }
    degrees = (uint32_t)trimmed;
#if SERVO_MIRROR
    degrees = SERVO_SPAN_DEG - degrees;
#endif
    return (SERVO_MIN_US +
            ((degrees * (SERVO_MAX_US - SERVO_MIN_US)) / SERVO_SPAN_DEG));
}
#endif /* SERVO_SCAN */

void
servo_init (void)
{
    out_w(PWM_DIV(SERVO_SLICE), PWM_DIV_1MHZ);
    (void)pwm_set_wrap(PIN_SERVO, SERVO_TOP);
#if SERVO_SCAN
    (void)pwm_set_cc(PIN_SERVO, deg_to_us(SERVO_CENTRE_DEG));
#else
    (void)pwm_set_cc(PIN_SERVO, SERVO_STOP_US);     /* held, facing front */
#endif
    (void)pwm_set_pin(PIN_SERVO);
    (void)pwm_set_enabled(PIN_SERVO, TRUE);
    g_servo_deg = SERVO_CENTRE_DEG;
#if !SERVO_SCAN
    {
        T_CCYC ccyc = {0};

        ccyc.cycatr = TA_HLNG | TA_STA;
        ccyc.cychdr = (FP)sweep_tick;
        ccyc.cyctim = (RELTIM)SWEEP_TICK_MS;
        ccyc.cycphs = (RELTIM)SWEEP_TICK_MS;
        CHECK(tk_cre_cyc(&ccyc), "servo sweep timer");
    }
#endif
}

void
servo_sweep (bool b_on)
{
#if !SERVO_SCAN
    if (b_on)
    {
        if (!g_b_sweeping)
        {
            g_sweep_pos = 0;            /* starts from the front */
            g_sweep_dir = 1;
            g_b_sweep_req = true;
            sweep_pulse(1);
            g_b_sweeping = true;
        }
        g_b_sweep_req = true;
    }
    else
    {
        int32_t waited = 0;

        g_b_sweep_req = false;
        while (g_b_sweeping && (waited < SWEEP_STOP_WAIT_MS))
        {
            (void)tk_dly_tsk((RELTIM)SWEEP_TICK_MS);
            waited += SWEEP_TICK_MS;
        }
        g_b_sweeping = false;
        (void)pwm_set_cc(PIN_SERVO, SERVO_STOP_US);
        g_servo_deg = SERVO_CENTRE_DEG;
    }
#else
    (void)b_on;
#endif
}

uint32_t
servo_move (uint32_t degrees)
{
    uint32_t const target = (degrees > SERVO_SPAN_DEG) ? SERVO_SPAN_DEG
                                                       : degrees;
    uint32_t const travel = (target > g_servo_deg) ? (target - g_servo_deg)
                                                   : (g_servo_deg - target);
    uint32_t       settle = (travel * SETTLE_MS_PER_30DEG) / 30U;

#if SERVO_SCAN
    (void)pwm_set_cc(PIN_SERVO, deg_to_us(target));
    g_servo_deg = target;
#else
    {
        /* Continuous servo: a timed turn, kept within +-SERVO_LIMIT_DEG
           of the front, then stop.  Larger pulse = left, like a
           positional servo (SERVO_MIRROR reverses it). */
        uint32_t const lo = SERVO_CENTRE_DEG - SERVO_LIMIT_DEG;
        uint32_t const hi = SERVO_CENTRE_DEG + SERVO_LIMIT_DEG;
        uint32_t const goal = (target < lo) ? lo : ((target > hi) ? hi : target);
        bool           b_left = (goal > g_servo_deg);
        uint32_t const turn = b_left ? (goal - g_servo_deg)
                                     : (g_servo_deg - goal);

#if SERVO_MIRROR
        b_left = !b_left;
#endif
        if (0U != turn)
        {
            (void)pwm_set_cc(PIN_SERVO, b_left ?
                             (SERVO_STOP_US + SERVO_CONT_DELTA_US) :
                             (SERVO_STOP_US - SERVO_CONT_DELTA_US));
            (void)tk_dly_tsk((RELTIM)((turn * 1000U) / SERVO_CONT_DPS));
            (void)pwm_set_cc(PIN_SERVO, SERVO_STOP_US);
        }
        g_servo_deg = goal;
        settle = 0U;        /* already there: the move was timed */
        (void)travel;
    }
#endif

    return ((settle < SETTLE_MIN_MS) ? SETTLE_MIN_MS : settle);
}

/*** end of file ***/
