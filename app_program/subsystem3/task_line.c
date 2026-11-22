/** @file task_line.c
 *
 * @brief T2 LineBarcode: 1 kHz sampling of the three IR channels (NFR1),
 *        line error every 10 ms (NFR2), junction / end-marker / lost
 *        detection (L0-L3) and Code 39 capture (B0-B4).
 *
 * Release: a 1 ms cyclic handler wakes this task (drift-free, absolute
 * time); any queued wake-up above one is counted as an overrun.
 *
 * Sensor mounting is DETECTED during the calibration sweep, which drags
 * the line right across the pair: STRADDLE when at some instant BOTH
 * sensors were over the line together (spacing no wider than the line),
 * BETWEEN when they never were (the kit photos suggest ~27 mm spacing for
 * an 18 mm line).  This does not depend on how well the car was centred
 * when START was pressed; the centred readings only decide whether a
 * grey-edge presence cue exists.  e = nL - nR in both cases; only
 * presence and loss detection differ.  The barcode
 * sensor sits on the right-hand arm over the 20 mm-offset barcode track.
 */

#include <string.h>

#include "subsystem3.h"
#include "drv_ir_adc.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem4/subsystem4.h"

#define PUBLISH_PERIODS         10U     /* line word every 10 ms        */
#define PERMILLE                1000
#define BLACK_ON                600     /* hysteresis on normalised n   */
#define BLACK_OFF               400
#define JUNCTION_ON             800     /* both channels this dark      */
#ifndef JUNCTION_ON_FREE
#define JUNCTION_ON_FREE        700     /* ... with a command waiting    */
#endif
#ifndef JUNCTION_FREE_MM
#define JUNCTION_FREE_MM        3U      /* ... for this far (D69)        */
#endif
#define PRESENT_ON              300     /* nL + nR: line under the pair */
#define PRESENT_OFF             150
#define SIDE_MIN                100     /* |e| to remember a side       */
#define CENTRE_SAMPLES          100U    /* first 100 ms of the sweep    */
#define BOTH_DARK_MIN           300     /* both this dark at once: STRADDLE */
#define GREY_MIN                90      /* centred nL+nR usable as cue,
                                           with each at least half of it */
#define CAL_DEAD_RAW            60      /* raw ADC this near 0 or full: no signal */
#define LOST_BETWEEN_MM         250U    /* no cue: both white this far  */
#ifndef PROBE_AFTER_MM
#define PROBE_AFTER_MM          120U    /* BETWEEN, both white this far:
                                           steer gently, see below       */
#endif
#ifndef PROBE_ERR
#define PROBE_ERR               300     /* that gentle steer, permille    */
#endif
#define ESCAPE_MM               16U     /* black this recently: escaped.
                                           30 mm of full lock drove the
                                           car off at -16 deg where the
                                           line ends at a gap; 8 mm lost
                                           the line on R350 bends.  16 mm
                                           does neither (tests/sim).     */
#define ESCAPE_PEAK             800     /* a contact that got this dark
                                           had the line pass the sensor
                                           centre: it went OUT.  A lighter
                                           brush came back into the gap
                                           (2026-11-03: full lock after
                                           every brush made the car
                                           slither from side to side)    */
#define BC_D0_DEBOUNCE_MS       2U      /* D0 level held this long: the
                                           comparator chatters on an edge */
#define BC_D0_AVG_SHIFT         13      /* ~8 s average of the D0 level:
                                           most of the time is white     */
#define BC_D0_FULL              (PERMILLE * 16)
#define BC_D0_WHITE_HIGH        (700 * 16)  /* mostly 1: white reads 1    */
#define BC_D0_WHITE_LOW         (300 * 16)  /* mostly 0: white reads 0    */
#define CAL_MIN_SPAN            300U    /* ADC counts                   */
#define ADC_FAIL_LIMIT          10U     /* consecutive failed samples   */
#define SWEEP_EVERY             10U     /* keep 1 sample in 10 ...      */
#define SWEEP_KEEP              150U    /* ... up to 1.5 s of the sweep */

#define ADC_FULL_SCALE          4095U
#define CAL_BITS                12U
#define CAL_BLACK_SHIFT         12U
#ifndef APP_BARCODE_TEST
#define APP_BARCODE_TEST        0
#endif
#define QUIET_MM                10U     /* barcode quiet zone           */
#ifndef BC_EDGE_DELTA
#define BC_EDGE_DELTA           150     /* swing that makes an edge, n    */
#endif
#define BC_EDGE_MIN             250     /* a bar is at least this dark    */
#define BC_LEVEL_STEP           25U     /* swing recorded this finely, n  */
#define BC_LEVELS               41U     /* 0..1000 in BC_LEVEL_STEP       */
#define BC_FILTER_SAMPLES       4U      /* moving average, 1 ms samples   */
#define BC_DIST_MIN_EDGES       100U    /* symbol moved at least this far
                                           (~19 mm): widths by distance  */
#define BC_DIST_MIN_ELEMENT     2U      /* every element >= 2 edges       */
#if APP_BARCODE_TEST
#define BC_UNIT_MS              2U      /* a 2-4 s swipe of a 141 mm symbol:
                                           narrow ~20-40 ms, wide ~120   */
#define CAPTURE_TIMEOUT_MS      3000U   /* a pause mid-symbol by hand    */
#define QUIET_MS                1500U   /* at hand speed one wide space
                                           outlasts 200 ms: it armed inside
                                           symbols (bench 2026-11-07)    */
#define BC_RANGE_RELAX_MS       64U     /* auto-level drift: 1 count per */
#define BC_BLACK_GUESS          2000    /* before any black is seen: with
                                           no guess the first bar after boot
                                           triggered early, 30 edges for 16 */
#define BC_STATUS_MS            2000U
#define BC_TRACE_EDGES          5U      /* one sample per ~1 mm of travel */
#define BC_TRACE_MAX            500U    /* samples: a whole scan pass     */
#define BC_TRACE_MIN            50U     /* shorter: not worth a trace     */
#define BC_TRACE_IDLE_MS        500U    /* stopped this long: print it    */
#define BC_TRACE_PER_LINE       25U
#define BC_TRACE_LINE_MS        100U    /* print slowly: T2 runs at 1 ms  */
#define BC_TRACE_SCALE          41      /* raw 0-4095 to 0-99             */
#else
#define BC_UNIT_MS              1U
#define CAPTURE_TIMEOUT_MS      300U
#define QUIET_MS                200U    /* ... or this long white: arms
                                           with the car still            */
#endif
#define CAPTURE_REPORT_MIN      9U      /* fewer elements: stray mark   */
#define CAPTURE_ANNOUNCE        2U
#define ELEMENT_SAT             255U
#define ERR_FULL                127

typedef struct
{
    uint32_t ms;                    /* 1 ms steps taken                */
    uint32_t d;                     /* encoder edges travelled         */
} bc_pos_t;

typedef enum
{
    BC_QUIET = 0,       /* B0 */
    BC_CAPTURE,         /* B1 */
    BC_IDLE             /* not armed (no quiet zone yet) */
} bc_state_t;

typedef struct
{
    uint32_t cal[IR_CHANNELS];      /* [11:0] white, [23:12] black */
    uint16_t raw[IR_CHANNELS];
    uint16_t cal_min[IR_CHANNELS];
    uint16_t cal_max[IR_CHANNELS];
    uint32_t cal_sum[IR_CHANNELS];
    uint32_t cal_n;
    uint32_t adc_fails;             /* consecutive failed conversions */
    uint32_t d0_level;              /* barcode D0, debounced          */
    uint32_t d0_count;              /* samples the raw level differed */
    int32_t  d0_avg;                /* average level, x16 permille    */
    bool     b_d0_started;
    bool     b_d0_black_high;       /* D0 reads 1 over black          */
    uint32_t beyond_n[IR_CHANNELS]; /* samples past the black level   */
    uint16_t beyond[IR_CHANNELS];   /* least extreme of those samples */
    int32_t  acc_err;               /* sum of e over the 10 ms window */
    int32_t  last_sign;
    int32_t  peak[2];               /* darkest n of the current contact,
                                       [0] left, [1] right              */
    bool     b_escaped;             /* last contact passed its centre */
    uint32_t tick;
    uint32_t lstate;
    uint32_t seq;
    uint32_t last_bc;
    uint32_t reject;
    uint16_t white_start;           /* odom where both went white     */
    uint16_t junction_start;
    bool     b_no_end;              /* this dark area began with a command
                                       waiting: a junction, never END   */
    bool     b_junc_tried;          /* this dark area tried as the end
                                       of a barcode (D72)             */
    uint16_t black_start;
    uint16_t quiet_start;
    uint16_t last_black;            /* odom when a sensor was black   */
    uint32_t centre_raw[2];         /* sums over CENTRE_SAMPLES       */
    uint16_t sweep_l[SWEEP_KEEP];   /* every SWEEP_EVERY-th sample:   */
    uint16_t sweep_r[SWEEP_KEEP];   /* both sensors at the same time  */
    uint32_t sweep_n;
    int32_t  present_on;            /* nL+nR thresholds, 0 = no cue   */
    int32_t  present_off;
    uint32_t lost_mm;
    uint32_t black_run;
    bool     b_black;
    bool     b_between;             /* mounting detected at calibration */
    bool     b_present;             /* line under the sensor pair   */
    bool     b_junction;
    bool     b_end;
    bool     b_calibrated;
    bool     b_cal_active;
    bool     b_bar;                 /* barcode channel black now      */
    /* Barcode capture */
    bc_state_t bc;
    uint32_t count;
    int32_t  bc_hist[BC_FILTER_SAMPLES];
    uint32_t bc_hist_i;
    bc_pos_t bc_now;
    bc_pos_t bc_up[BC_LEVELS];      /* the rise from bc_valley         */
    bc_pos_t bc_dn[BC_LEVELS];      /* the fall from bc_peak           */
    uint32_t bc_up_top;
    uint32_t bc_dn_top;
    int32_t  bc_valley;
    int32_t  bc_peak;
    bc_pos_t bc_e;                  /* the edge a flip just settled    */
    bc_pos_t bc_edge;               /* last edge placed                */
    uint32_t bc_flip_d;             /* distance at the last flip       */
    bool     b_edge;                /* bc_edge is valid                */
    uint16_t bc_enc_l;              /* encoder counts at the last step */
    uint16_t bc_enc_r;
    uint32_t idle_ms;
    bool     b_tried;               /* trailing search done this white */
    uint32_t bc_start_ms;           /* when this capture began        */
    uint8_t  elements_d[BARCODE_MAX_ELEMENTS];   /* window, by distance */
    uint8_t  try_t[BARCODE_MAX_ELEMENTS];        /* trailing search     */
    uint8_t  try_d[BARCODE_MAX_ELEMENTS];
    uint32_t quiet_ms;              /* white time while not capturing */
#if APP_BARCODE_TEST
    int32_t  bc_lo;                 /* auto-levelled white / black    */
    int32_t  bc_hi;
    uint32_t bc_relax;
    uint32_t bc_status;
    uint8_t  trace[BC_TRACE_MAX];   /* raw reading per mm of travel   */
    uint32_t trace_n;
    uint32_t trace_d;               /* distance of the last sample    */
    uint32_t trace_idle;
    uint32_t trace_out;             /* next sample to print           */
    uint32_t trace_wait;
    bool     b_trace_out;
#endif
    uint8_t  elements[BARCODE_MAX_ELEMENTS];
} line_ctx_t;

EXPORT volatile uint32_t g_line_word = 0U;
EXPORT volatile uint32_t g_cross_word = 0U;
EXPORT volatile uint8_t  g_barcode_last = 0U;
EXPORT UW                g_line_overruns = 0U;

static line_ctx_t g_lc;

/* ------------------------------------------------------------------ */

static uint32_t
cal_pack (uint32_t white, uint32_t black)
{
    return (bits_put(bits_put(0U, 0U, CAL_BITS, white),
                     CAL_BLACK_SHIFT, CAL_BITS, black));
}

/* Normalise a raw reading to 0 (white) .. 1000 (black), either polarity. */
static int32_t
normalise_raw (uint32_t channel, int32_t raw)
{
    int32_t const white = (int32_t)bits_get(g_lc.cal[channel], 0U, CAL_BITS);
    int32_t const black = (int32_t)bits_get(g_lc.cal[channel],
                                            CAL_BLACK_SHIFT, CAL_BITS);
    int32_t const span = black - white;
    int32_t       n;

    if (0 == span)
    {
        return (0);
    }
    n = ((raw - white) * PERMILLE) / span;
    if (n < 0)
    {
        n = 0;
    }
    else if (n > PERMILLE)
    {
        n = PERMILLE;
    }
    else
    {
        /* In range. */
    }
    return (n);
}

static int32_t
normalise (uint32_t channel)
{
    return (normalise_raw(channel, (int32_t)g_lc.raw[channel]));
}

/* Every line-state change goes through here.  Leaving L3 by ANY route
   (back to the line, straight onto a junction bar, recalibration) clears
   FLG_LINE_LOST; otherwise T6 would keep waking on a stale bit and
   bounce between S3 and S8. */
static void
set_lstate (uint32_t state)
{
    if ((LSTATE_LOST == g_lc.lstate) && (LSTATE_LOST != state))
    {
        (void)tk_clr_flg(flg_sys, ~FLG_LINE_LOST);
        if ((LSTATE_TRACK == state) || (LSTATE_JUNCTION == state))
        {
            (void)tk_set_flg(flg_sys, FLG_LINE_FOUND);
        }
    }
    g_lc.lstate = state;
}

/* ------------------------------------------------------------------ */

static void
calibrate_sample (void)
{
    uint32_t ch;

    uint16_t const l = g_lc.raw[IR_CH_LEFT];
    uint16_t const r = g_lc.raw[IR_CH_RIGHT];

    if (g_lc.cal_n < CENTRE_SAMPLES)
    {
        g_lc.centre_raw[IR_CH_LEFT] += l;
        g_lc.centre_raw[IR_CH_RIGHT] += r;
    }

    /* Pairs taken at the same instant, judged once each sensor's own
       white/black is known: the two need not read black the same way
       round (bench 2026-11-02: left falls over black, right rises). */
    if ((0U == (g_lc.cal_n % SWEEP_EVERY)) && (g_lc.sweep_n < SWEEP_KEEP))
    {
        g_lc.sweep_l[g_lc.sweep_n] = l;
        g_lc.sweep_r[g_lc.sweep_n] = r;
        g_lc.sweep_n++;
    }

    for (ch = 0U; ch < IR_CHANNELS; ch++)
    {
        if (g_lc.raw[ch] < g_lc.cal_min[ch])
        {
            g_lc.cal_min[ch] = g_lc.raw[ch];
        }
        if (g_lc.raw[ch] > g_lc.cal_max[ch])
        {
            g_lc.cal_max[ch] = g_lc.raw[ch];
        }
        g_lc.cal_sum[ch] += g_lc.raw[ch];
    }
    g_lc.cal_n++;
}

/* Keep calibrating while driving: a reading further toward black than
   the calibrated black, for 3 samples running (not one spike), moves the
   black level out to the least extreme of them.  The sweep may have shown
   a sensor only part of the line, or none of it (borrowed span). */
static void
cal_track (void)
{
    uint32_t ch;

    for (ch = 0U; ch < IR_CHANNELS; ch++)
    {
        uint32_t const white = bits_get(g_lc.cal[ch], 0U, CAL_BITS);
        uint32_t const black = bits_get(g_lc.cal[ch], CAL_BLACK_SHIFT, CAL_BITS);
        uint32_t const raw = g_lc.raw[ch];
        bool const     b_high = (black > white);
        bool const     b_past = b_high ? (raw > black) : (raw < black);

        if (!b_past)
        {
            g_lc.beyond_n[ch] = 0U;
            continue;
        }
        if ((0U == g_lc.beyond_n[ch]) ||
            (b_high ? (raw < g_lc.beyond[ch]) : (raw > g_lc.beyond[ch])))
        {
            g_lc.beyond[ch] = (uint16_t)raw;
        }
        if (++g_lc.beyond_n[ch] >= 3U)
        {
            g_lc.cal[ch] = cal_pack(white, g_lc.beyond[ch]);
            g_lc.beyond_n[ch] = 0U;
        }
    }
}

/* Did this channel see both the floor and the line during the sweep? */
static bool
cal_saw_both (uint32_t ch)
{
    return ((uint32_t)(g_lc.cal_max[ch] - g_lc.cal_min[ch]) >= CAL_MIN_SPAN);
}

/* The extreme nearer the mean is white (the floor is mostly white). */
static void
cal_own (uint32_t ch)
{
    uint32_t const lo = g_lc.cal_min[ch];
    uint32_t const hi = g_lc.cal_max[ch];
    uint32_t const mean = g_lc.cal_sum[ch] / g_lc.cal_n;

    g_lc.cal[ch] = ((mean - lo) < (hi - mean)) ? cal_pack(lo, hi)
                                               : cal_pack(hi, lo);
}

/* A channel that never saw black (the sweep is right-then-left, so only
   the left sensor crosses the line): its own white level, the mean of
   its all-white readings, plus the black-white difference of a channel
   that did see the line.  Same sensor type on every channel. */
static void
cal_borrow (uint32_t ch, uint32_t from)
{
    int32_t const white = (int32_t)(g_lc.cal_sum[ch] / g_lc.cal_n);
    int32_t       span =
        (int32_t)bits_get(g_lc.cal[from], CAL_BLACK_SHIFT, CAL_BITS) -
        (int32_t)bits_get(g_lc.cal[from], 0U, CAL_BITS);
    int32_t       black;

    /* Borrow the SIZE of the swing only.  The direction follows this
       channel's own floor reading: near the bottom of the range, black
       reads higher; near the top, lower.  Borrowing the direction too made
       the right sensor blind (bench 2026-11-02: left reads ~3000 on the
       floor and dips over black, right reads ~170 and rises). */
    span = (span < 0) ? -span : span;
    black = (white < (int32_t)(ADC_FULL_SCALE / 2U)) ? (white + span)
                                                    : (white - span);

    if (black < 0)
    {
        black = 0;
    }
    else if (black > (int32_t)ADC_FULL_SCALE)
    {
        black = (int32_t)ADC_FULL_SCALE;
    }
    else
    {
        /* In range. */
    }
    g_lc.cal[ch] = cal_pack((uint32_t)white, (uint32_t)black);
    tm_printf((UB *)"T2: ch%d never saw the line (span %d): white %d,"
              " span borrowed from ch%d\n", (INT)ch,
              (INT)(g_lc.cal_max[ch] - g_lc.cal_min[ch]), (INT)white,
              (INT)from);
}

/* Finish the sweep: per-channel white/black and, from which extreme is
   nearer the mean (the floor is mostly white), the sensor polarity. */
static bool
calibrate_finish (void)
{
    uint32_t ch;
    bool     b_ok = (g_lc.cal_n > 0U);
    bool     b_left = false;
    bool     b_right = false;

    if (b_ok)
    {
        b_left = cal_saw_both(IR_CH_LEFT);
        b_right = cal_saw_both(IR_CH_RIGHT);
        if (!b_left && !b_right)
        {
            uint32_t const lo = (g_lc.cal_min[IR_CH_LEFT] < g_lc.cal_min[IR_CH_RIGHT]) ?
                                g_lc.cal_min[IR_CH_LEFT] : g_lc.cal_min[IR_CH_RIGHT];
            uint32_t const hi = (g_lc.cal_max[IR_CH_LEFT] > g_lc.cal_max[IR_CH_RIGHT]) ?
                                g_lc.cal_max[IR_CH_LEFT] : g_lc.cal_max[IR_CH_RIGHT];

            tm_printf((UB *)"T2: neither line sensor saw the line (spans"
                      " %d, %d)\n",
                      (INT)(g_lc.cal_max[IR_CH_LEFT] - g_lc.cal_min[IR_CH_LEFT]),
                      (INT)(g_lc.cal_max[IR_CH_RIGHT] - g_lc.cal_min[IR_CH_RIGHT]));
            /* Say which of the usual causes it is (raw ADC, 0-4095). */
            tm_printf((UB *)"T2: raw left %d..%d, right %d..%d: %s\n",
                      (INT)g_lc.cal_min[IR_CH_LEFT], (INT)g_lc.cal_max[IR_CH_LEFT],
                      (INT)g_lc.cal_min[IR_CH_RIGHT], (INT)g_lc.cal_max[IR_CH_RIGHT],
                      (hi < CAL_DEAD_RAW) ?
                      "both read near 0 - sensor power or the A0 wires (GP26/GP27)" :
                      (lo > (ADC_FULL_SCALE - CAL_DEAD_RAW)) ?
                      "both read full scale - nothing reflecting: car lifted or sensors too high" :
                      "sensors read the floor but never the line - start with the black line"
                      " between the two sensors, then Calibrate");
            b_ok = false;
        }
    }

    if (b_ok)
    {
        uint32_t const donor = b_left ? IR_CH_LEFT : IR_CH_RIGHT;

        if (b_left)
        {
            cal_own(IR_CH_LEFT);
        }
        if (b_right)
        {
            cal_own(IR_CH_RIGHT);
        }
        if (!b_left)
        {
            cal_borrow(IR_CH_LEFT, IR_CH_RIGHT);
        }
        if (!b_right)
        {
            cal_borrow(IR_CH_RIGHT, IR_CH_LEFT);
        }
        if (cal_saw_both(IR_CH_BARCODE))
        {
            cal_own(IR_CH_BARCODE);
        }
        else
        {
            cal_borrow(IR_CH_BARCODE, donor);
        }
        for (ch = 0U; ch < IR_CHANNELS; ch++)
        {
            tm_printf((UB *)"T2: ch%d white %d black %d\n", (INT)ch,
                      (INT)bits_get(g_lc.cal[ch], 0U, CAL_BITS),
                      (INT)bits_get(g_lc.cal[ch], CAL_BLACK_SHIFT, CAL_BITS));
        }
    }

    if (b_ok && (g_lc.cal_n >= CENTRE_SAMPLES))
    {
        int32_t const n_l = normalise_raw(IR_CH_LEFT,
            (int32_t)(g_lc.centre_raw[IR_CH_LEFT] / CENTRE_SAMPLES));
        int32_t const n_r = normalise_raw(IR_CH_RIGHT,
            (int32_t)(g_lc.centre_raw[IR_CH_RIGHT] / CENTRE_SAMPLES));
        int32_t const sum = n_l + n_r;
        int32_t       both = 0;
        uint32_t      k;

        /* How dark were both sensors at once, at most? */
        for (k = 0U; k < g_lc.sweep_n; k++)
        {
            int32_t const a = normalise_raw(IR_CH_LEFT, (int32_t)g_lc.sweep_l[k]);
            int32_t const b = normalise_raw(IR_CH_RIGHT, (int32_t)g_lc.sweep_r[k]);
            int32_t const m = (a < b) ? a : b;

            if (m > both)
            {
                both = m;
            }
        }

        g_lc.b_between = (both < BOTH_DARK_MIN);
        if (!g_lc.b_between)
        {
            g_lc.present_on = PRESENT_ON;
            g_lc.present_off = PRESENT_OFF;
            g_lc.lost_mm = LINE_LOST_MM;
        }
        else if ((sum >= GREY_MIN) && (n_l >= (GREY_MIN / 2)) &&
                 (n_r >= (GREY_MIN / 2)))
        {
            /* Both sensors see the line's edges when centred.  One grey
               channel alone only means the car was placed off-centre;
               using it as a cue made the car hunt around that offset. */
            g_lc.present_on = (sum * 2) / 3;    /* grey edge cue */
            g_lc.present_off = sum / 3;
            g_lc.lost_mm = LINE_LOST_MM;
        }
        else
        {
            g_lc.present_on = 0;                /* no cue: distance only */
            g_lc.present_off = 0;
            g_lc.lost_mm = LOST_BETWEEN_MM;
        }
        tm_printf((UB *)"T2: mounting %s (both dark %d, centred nL %d"
                  " nR %d, cue %d)\n",
                  g_lc.b_between ? "BETWEEN" : "STRADDLE", (INT)both,
                  (INT)n_l, (INT)n_r, (INT)g_lc.present_on);
    }

    return (b_ok);
}

static void
calibrate_reset (void)
{
    uint32_t ch;

    for (ch = 0U; ch < IR_CHANNELS; ch++)
    {
        g_lc.cal_min[ch] = (uint16_t)ADC_FULL_SCALE;
        g_lc.cal_max[ch] = 0U;
        g_lc.cal_sum[ch] = 0U;
    }
    g_lc.cal_n = 0U;
    g_lc.centre_raw[IR_CH_LEFT] = 0U;
    g_lc.centre_raw[IR_CH_RIGHT] = 0U;
    g_lc.sweep_n = 0U;
}

/* Every 10 ms: follow the CAL_REQ / CAL_STOP handshake with T6. */
static void
calibrate_poll (void)
{
    T_RFLG rflg;

    if (E_OK != tk_ref_flg(flg_sys, &rflg))
    {
        return;
    }

    if ((0U != (rflg.flgptn & FLG_CAL_REQ)) && (!g_lc.b_cal_active))
    {
        calibrate_reset();
        g_lc.b_cal_active = true;
        set_lstate(LSTATE_CALIBRATE);
    }

    if (g_lc.b_cal_active && (0U != (rflg.flgptn & FLG_CAL_STOP)))
    {
        g_lc.b_cal_active = false;
        (void)tk_clr_flg(flg_sys, ~(FLG_CAL_REQ | FLG_CAL_STOP));
        if (calibrate_finish())
        {
            g_lc.b_calibrated = true;
            set_lstate(LSTATE_TRACK);
            (void)tk_set_flg(flg_sys, FLG_CAL_DONE);
        }
        else
        {
            fault_set(FAULT_SELF_TEST);
        }
    }
}

/* ------------------------------------------------------------------ */

static void
post_nav (uint32_t cmd, uint32_t index, uint32_t reason)
{
    uint32_t  word = 0U;
    nav_cmd_t msg;

    word = bits_put(word, NAV_CMD_SHIFT, NAV_CMD_BITS, cmd);
    word = bits_put(word, NAV_CHAR_SHIFT, NAV_CHAR_BITS, index);
    word = bits_put(word, NAV_REJ_SHIFT, NAV_REJ_BITS, reason);
    msg = (nav_cmd_t)word;

    if (E_OK == tk_snd_mbf(mbf_nav, &msg, sizeof(msg), TMO_POL))
    {
        (void)tk_set_flg(flg_sys, FLG_NAV);
    }
}

static char const * const k_reject_names[] =
{
    "ok", "wrong number of bars/spaces", "start/stop is not '*'",
    "a bar or space far too narrow/wide", "not a Code 39 pattern",
    "no barcode found in the marks"
};

/* The widths, for working out why a symbol was rejected (or how evenly
   it was passed): by distance in encoder edges (~0.19 mm) and by time. */
static void
barcode_print_widths (void)
{
    uint32_t idx;

    /* The window: the last 29 elements when there were more. */
    tm_printf((UB *)"T2: widths by distance (encoder edges, ~0.19 mm):");
    for (idx = 0U; (idx < g_lc.count) && (idx < BARCODE_MAX_ELEMENTS); idx++)
    {
        tm_printf((UB *)" %d", (INT)g_lc.elements_d[idx]);
    }
    tm_printf((UB *)"\nT2: widths by time (%d ms units):", (INT)BC_UNIT_MS);
    for (idx = 0U; (idx < g_lc.count) && (idx < BARCODE_MAX_ELEMENTS); idx++)
    {
        tm_printf((UB *)" %d", (INT)g_lc.elements[idx]);
    }
    tm_printf((UB *)"\n");
}

/* Distance widths are used whenever the car moved over the symbol: they
   do not depend on speed at all (a hand roll, a speed change while
   steering).  A card swiped under a standing car falls back to time. */
static bool
barcode_by_distance (uint8_t const * p_d, uint32_t count)
{
    uint32_t idx;
    uint32_t sum = 0U;

    for (idx = 0U; idx < count; idx++)
    {
        if (p_d[idx] < BC_DIST_MIN_ELEMENT)
        {
            return (false);
        }
        sum += p_d[idx];
    }
    return (sum >= BC_DIST_MIN_EDGES);
}

/* One window of 29 widths: returns BC_REJ_NONE and the character index,
   and how the widths were read. */
static uint8_t
barcode_read (uint8_t const * p_t, uint8_t const * p_d, uint8_t * p_index,
              char const ** pp_how)
{
    bool const b_dist = barcode_by_distance(p_d, BARCODE_MAX_ELEMENTS);
    uint8_t    reason = barcode_decode(b_dist ? p_d : p_t,
                                       BARCODE_MAX_ELEMENTS, p_index);

    *pp_how = b_dist ? "distance" : "time";
    if (b_dist && (BC_REJ_NONE != reason) &&
        (BC_REJ_NONE == barcode_decode(p_t, BARCODE_MAX_ELEMENTS, p_index)))
    {
        reason = BC_REJ_NONE;               /* time read it after all */
        *pp_how = "time";
    }
    /* An analog sensor riding high sees every bar thinner and every
       space wider by the same amount: undo that, then decode as usual. */
    if (b_dist && (BC_REJ_NONE != reason) &&
        (BC_REJ_NONE == barcode_decode_balanced(p_d, BARCODE_MAX_ELEMENTS,
                                                p_index)))
    {
        reason = BC_REJ_NONE;
        *pp_how = "distance, bar width corrected";
    }
    /* Blur shifts bars against spaces past the ratio limits; distance
       widths are still in the right order. */
    if (b_dist && (BC_REJ_NONE != reason) &&
        (BC_REJ_NONE == barcode_decode_loose(p_d, BARCODE_MAX_ELEMENTS,
                                             p_index)))
    {
        reason = BC_REJ_NONE;
        *pp_how = "distance, 3 widest per character";
    }
#if APP_BARCODE_TEST
    /* A hand swipe: the speed changes too much for the ratio rules. */
    if ((!b_dist) && (BC_REJ_NONE != reason) &&
        (BC_REJ_NONE == barcode_decode_loose(p_t, BARCODE_MAX_ELEMENTS,
                                             p_index)))
    {
        reason = BC_REJ_NONE;
        *pp_how = "time, 3 widest per character";
    }
#endif
    return (reason);
}

static void
barcode_stop (void)
{
    g_lc.bc = BC_IDLE;
    g_lc.quiet_start = odom_now_mm();
    g_lc.quiet_ms = 0U;
}

/* A symbol was found in the window. */
static void
barcode_accept (uint8_t index, char const * p_how)
{
    static char const * const k_cmd_word[] =
    {
        "unmapped", "left", "right", "straight", "u-turn"
    };
    char const chr = g_barcode_chars[index];
    uint32_t   cmd = NAV_OTHER;

    /* The letter's command from the map set before the run; digits and
       symbols, and unmapped letters, carry none.  last_bc (telemetry) is
       the command: 1 left .. 4 U-turn, 5 none. */
    if ((chr >= 'A') && (chr <= 'Z'))
    {
        uint32_t const m = g_bc_map[(uint32_t)(chr - 'A')];

        if ((m >= NAV_LEFT) && (m <= NAV_UTURN))
        {
            cmd = m;
        }
    }
    g_lc.last_bc = (NAV_OTHER == cmd) ? BC_LAST_OTHER : (uint8_t)cmd;
    g_barcode_last = (uint8_t)chr;
    g_lc.reject = BC_REJ_NONE;
    post_nav(cmd, index, BC_REJ_NONE);
    tm_printf((UB *)"T2: barcode '%c' (widths by %s) -> %s\n", (INT)chr,
              p_how, k_cmd_word[(NAV_OTHER == cmd) ? 0U : cmd]);
#if APP_BARCODE_TEST
    barcode_print_widths();
#endif
    barcode_stop();
}

/* Elements were captured but no symbol was found in them. */
static void
barcode_finish (void)
{
    if (g_lc.count >= CAPTURE_REPORT_MIN)
    {
        uint8_t      index = 0U;
        char const * p_how = "time";
        uint8_t      reason = (g_lc.count < BARCODE_MAX_ELEMENTS) ?
                              BC_REJ_COUNT :
                              barcode_read(g_lc.elements, g_lc.elements_d,
                                           &index, &p_how);

        if ((BC_REJ_NONE == reason) || (g_lc.count > BARCODE_MAX_ELEMENTS))
        {
            reason = BC_REJ_OVERRUN;
        }
        g_lc.reject = reason;
        post_nav(NAV_REJECT, 0U, reason);
        tm_printf((UB *)"T2: barcode rejected, reason %d (%s), %d elements"
                  " (29 expected), widths by %s\n", (INT)reason,
                  k_reject_names[(reason <= BC_REJ_OVERRUN) ? reason : 0U],
                  (INT)g_lc.count, p_how);
        barcode_print_widths();
#if APP_BARCODE_TEST
        tm_printf((UB *)"T2: that pass took %d s; slide a barcode through in"
                  " 2-4 s, or use scan\n",
                  (INT)((g_lc.bc_now.ms - g_lc.bc_start_ms) / 1000U));
#endif
    }
#if APP_BARCODE_TEST
    else if (g_lc.count >= 3U)
    {
        tm_printf((UB *)"T2: stray mark, %d elements (29 make a symbol)\n",
                  (INT)g_lc.count);
    }
    else
    {
        /* Not worth a line. */
    }
#endif
    barcode_stop();
}

static void
barcode_abort (void)
{
    g_lc.bc = BC_IDLE;
    g_lc.count = 0U;
    g_lc.quiet_start = odom_now_mm();
    g_lc.quiet_ms = 0U;
}

/* Encoder edges moved since the last 1 ms step: the larger of the two
   wheels (one wheel may not turn when the car is pushed by hand). */
static uint32_t
barcode_edges (void)
{
    uint16_t const enc_l = g_enc_left;      /* one halfword load each */
    uint16_t const enc_r = g_enc_right;
    int32_t        dl = (int32_t)(int16_t)(uint16_t)(enc_l - g_lc.bc_enc_l);
    int32_t        dr = (int32_t)(int16_t)(uint16_t)(enc_r - g_lc.bc_enc_r);

    g_lc.bc_enc_l = enc_l;
    g_lc.bc_enc_r = enc_r;
    dl = (dl < 0) ? -dl : dl;
    dr = (dr < 0) ? -dr : dr;
    return ((uint32_t)((dl > dr) ? dl : dr));
}

/* The bar signal is followed from turning point to turning point instead
   of against a fixed level: seen through a blurred spot a thin white gap
   is only a dip (100% black down to 70%), which never crossed BLACK_OFF,
   so its two bars merged (bench 2026-11-07).  A swing of BC_EDGE_DELTA back
   from the last extreme is an edge.  Each swing records where it passed
   every BC_LEVEL_STEP, so once both ends are known the edge is placed where
   the signal crossed half way: full bars and shallow dips alike, and noise
   on the flat parts cannot move it. */
static void
bc_run_record (bc_pos_t * p_run, uint32_t * p_top, int32_t swing)
{
    uint32_t level = (swing <= 0) ? 0U : ((uint32_t)swing / BC_LEVEL_STEP);

    if (level >= BC_LEVELS)
    {
        level = BC_LEVELS - 1U;
    }
    while (*p_top < level)
    {
        (*p_top)++;
        p_run[*p_top] = g_lc.bc_now;
    }
}

/* Where the swing between valley and peak crossed half way. */
static void
bc_half_way (bc_pos_t const * p_run, uint32_t top)
{
    uint32_t level = ((uint32_t)(g_lc.bc_peak - g_lc.bc_valley) / 2U) /
                     BC_LEVEL_STEP;

    if (level > top)
    {
        level = top;
    }
    g_lc.bc_e = p_run[level];
}

static bool
barcode_track (int32_t n)
{
    bool b_flip = false;

    if (g_lc.b_bar)
    {
        if (n > g_lc.bc_peak)
        {
            g_lc.bc_peak = n;               /* a new peak: fall starts over */
            g_lc.bc_dn_top = 0U;
            g_lc.bc_dn[0] = g_lc.bc_now;
        }
        b_flip = (n <= (g_lc.bc_peak - BC_EDGE_DELTA));
    }
    else
    {
        if (n < g_lc.bc_valley)
        {
            g_lc.bc_valley = n;             /* a new valley: rise starts over */
            g_lc.bc_up_top = 0U;
            g_lc.bc_up[0] = g_lc.bc_now;
        }
        b_flip = ((n >= (g_lc.bc_valley + BC_EDGE_DELTA)) &&
                  (n >= BC_EDGE_MIN));
    }
    bc_run_record(g_lc.bc_up, &g_lc.bc_up_top, n - g_lc.bc_valley);
    bc_run_record(g_lc.bc_dn, &g_lc.bc_dn_top, g_lc.bc_peak - n);

    if (b_flip)
    {
        if (g_lc.b_bar)
        {
            bc_half_way(g_lc.bc_up, g_lc.bc_up_top);    /* the rise, closed */
            g_lc.bc_valley = n;
            g_lc.bc_up_top = 0U;
            g_lc.bc_up[0] = g_lc.bc_now;
        }
        else
        {
            bc_half_way(g_lc.bc_dn, g_lc.bc_dn_top);    /* the fall, closed */
            g_lc.bc_peak = n;
            g_lc.bc_dn_top = 0U;
            g_lc.bc_dn[0] = g_lc.bc_now;
        }
        g_lc.b_bar = !g_lc.b_bar;
    }
    return (b_flip);
}

/* The last edge has no flip to close it: use the swing so far. */
static void
barcode_flush_edge (void)
{
    if (g_lc.b_bar)
    {
        bc_half_way(g_lc.bc_up, g_lc.bc_up_top);
    }
    else
    {
        bc_half_way(g_lc.bc_dn, g_lc.bc_dn_top);
    }
}

/* Append one element to a window of the last 29, oldest dropped. */
static void
barcode_widths (uint8_t * p_t, uint8_t * p_d, uint32_t ms, uint32_t d)
{
    uint32_t at = g_lc.count;

    if (at >= BARCODE_MAX_ELEMENTS)
    {
        at = BARCODE_MAX_ELEMENTS - 1U;
        (void)memmove(p_t, &p_t[1], at);
        (void)memmove(p_d, &p_d[1], at);
    }
    p_t[at] = (uint8_t)((ms > ELEMENT_SAT) ? ELEMENT_SAT :
                        ((0U == ms) ? 1U : ms));
    p_d[at] = (uint8_t)((d > ELEMENT_SAT) ? ELEMENT_SAT : d);
}

/* The edge in bc_e closes the element that began at the previous edge. */
static void
barcode_mark (void)
{
    bc_pos_t const edge = g_lc.bc_e;

    if (g_lc.b_edge)
    {
        uint32_t ms = (edge.ms - g_lc.bc_edge.ms) / BC_UNIT_MS;
        uint32_t d = edge.d - g_lc.bc_edge.d;

        barcode_widths(g_lc.elements, g_lc.elements_d, ms, d);
        g_lc.count++;
    }
    g_lc.bc_edge = edge;
    g_lc.b_edge = true;
}

/* A symbol has 29 elements and starts with a bar: with an odd total the
   last 29 are a candidate, whatever came before them. */
static bool
barcode_search (void)
{
    uint8_t      index = 0U;
    char const * p_how = "";

    if ((g_lc.count < BARCODE_MAX_ELEMENTS) || (0U == (g_lc.count % 2U)) ||
        (BC_REJ_NONE != barcode_read(g_lc.elements, g_lc.elements_d, &index,
                                     &p_how)))
    {
        return (false);
    }
    barcode_accept(index, p_how);
    return (true);
}

/* White after what may be the last bar, which no flip has closed yet: try
   the window with that bar in place, without committing to its edge. */
static bool
barcode_search_trailing (void)
{
    uint8_t      index = 0U;
    char const * p_how = "";
    uint32_t     keep = g_lc.count;
    bc_pos_t     edge;

    if ((g_lc.count < (BARCODE_MAX_ELEMENTS - 1U)) ||
        (0U != (g_lc.count % 2U)))
    {
        return (false);
    }
    barcode_flush_edge();
    edge = g_lc.bc_e;
    (void)memcpy(g_lc.try_t, g_lc.elements, BARCODE_MAX_ELEMENTS);
    (void)memcpy(g_lc.try_d, g_lc.elements_d, BARCODE_MAX_ELEMENTS);
    g_lc.count = (keep >= BARCODE_MAX_ELEMENTS) ? BARCODE_MAX_ELEMENTS
                                                : keep;
    barcode_widths(g_lc.try_t, g_lc.try_d,
                   (edge.ms - g_lc.bc_edge.ms) / BC_UNIT_MS,
                   edge.d - g_lc.bc_edge.d);
    g_lc.count = keep;
    if (BC_REJ_NONE != barcode_read(g_lc.try_t, g_lc.try_d, &index, &p_how))
    {
        return (false);
    }
    (void)memcpy(g_lc.elements, g_lc.try_t, BARCODE_MAX_ELEMENTS);
    (void)memcpy(g_lc.elements_d, g_lc.try_d, BARCODE_MAX_ELEMENTS);
    g_lc.count = keep + 1U;
    barcode_accept(index, p_how);
    return (true);
}


/* A barcode command is waiting for its junction (T6's g_mission_word). */
static bool
cmd_waiting (void)
{
    return (NAV_NONE != bits_get((uint32_t)g_mission_word, MISSION_PEND_SHIFT,
                                 MISSION_PEND_BITS));
}
/* One 1 ms barcode step. */
static void
barcode_step (int32_t n_bar, uint16_t odom)
{
    uint32_t const edges = barcode_edges();
    int32_t        sum = 0;
    uint32_t       idx;
    bool           b_flip;

    g_lc.bc_now.ms++;
    g_lc.bc_now.d += edges;
    g_lc.bc_hist[g_lc.bc_hist_i] = n_bar;
    g_lc.bc_hist_i = (g_lc.bc_hist_i + 1U) % BC_FILTER_SAMPLES;
    for (idx = 0U; idx < BC_FILTER_SAMPLES; idx++)
    {
        sum += g_lc.bc_hist[idx];
    }
    b_flip = barcode_track(sum / (int32_t)BC_FILTER_SAMPLES);

    switch (g_lc.bc)
    {
        case BC_IDLE:
        case BC_QUIET:
            if (g_lc.b_bar)
            {
                /* The bench has no quiet zone to trust (a car lifted off
                   the paper reads black): any bar may start a capture, the
                   window finds the symbol. */
                /* Not while a command waits: that bar is its junction. */
                if (b_flip && !cmd_waiting() &&
                    ((BC_QUIET == g_lc.bc) || (0 != APP_BARCODE_TEST)))
                {
                    g_lc.bc = BC_CAPTURE;   /* first bar after quiet zone */
                    g_lc.count = 0U;
                    g_lc.b_edge = false;
                    g_lc.b_tried = false;
                    g_lc.bc_start_ms = g_lc.bc_now.ms;
                    g_lc.idle_ms = 0U;
                    g_lc.bc_flip_d = g_lc.bc_now.d;
                }
                g_lc.quiet_start = odom;
                g_lc.quiet_ms = 0U;
            }
            else if ((odom_delta_mm(odom, g_lc.quiet_start) >= QUIET_MM) ||
                     (++g_lc.quiet_ms >= QUIET_MS))
            {
                g_lc.bc = BC_QUIET;         /* armed */
            }
            else
            {
                /* Still measuring the quiet zone. */
            }
        break;

        case BC_CAPTURE:
            if (b_flip)
            {
                barcode_mark();
                g_lc.idle_ms = 0U;
                g_lc.b_tried = false;
                g_lc.quiet_start = odom;
                g_lc.bc_flip_d = g_lc.bc_now.d;

                if (CAPTURE_ANNOUNCE == g_lc.count)
                {
                    (void)tk_set_flg(flg_sys, FLG_BARCODE_START);
                }
                (void)barcode_search();
            }
            else
            {
                g_lc.idle_ms++;
                if ((!g_lc.b_bar) && (!g_lc.b_tried) &&
                    ((odom_delta_mm(odom, g_lc.quiet_start) >= QUIET_MM) ||
                     (g_lc.idle_ms >= QUIET_MS)))
                {
                    g_lc.b_tried = true;
                    if (barcode_search_trailing())
                    {
                        break;
                    }
                }
                if ((g_lc.idle_ms >= CAPTURE_TIMEOUT_MS) ||
                    ((g_lc.bc_now.d - g_lc.bc_flip_d) >= ELEMENT_SAT))
                {
                    barcode_finish();       /* no symbol in these marks */
                }
            }
        break;

        default:
            barcode_abort();
        break;
    }
}

/* ------------------------------------------------------------------ */

/* Per-sensor contact: remember how dark it got.  When the sensor goes
   white again, a contact that reached ESCAPE_PEAK means the line crossed
   under the sensor centre and is now outside the pair; a lighter one
   means the steering turned it back into the gap (centred, error 0). */
static void
contact_track (uint32_t side, int32_t n, bool b_dark)
{
    if (b_dark)
    {
        if (n > g_lc.peak[side])
        {
            g_lc.peak[side] = n;
        }
    }
    else if ((n < BLACK_OFF) && (0 != g_lc.peak[side]))
    {
        g_lc.b_escaped = (g_lc.peak[side] >= ESCAPE_PEAK);
        g_lc.peak[side] = 0;
    }
    else
    {
        /* White, no contact in progress. */
    }
}

/* One 1 ms line step: line state, error accumulation, black run.
   On a straight line each sensor is about half black, so presence is
   judged on the SUM of both channels, not on either one alone. */
static void
line_step (int32_t n_left, int32_t n_right, uint16_t odom)
{
    bool const b_left = (n_left > BLACK_ON);
    bool const b_right = (n_right > BLACK_ON);
    /* Hysteresis: start a junction / END area at JUNCTION_ON, keep it
       while both stay above BLACK_ON.  One noisy sample under 800 used to
       restart the 60 mm END count, so a weak sensor (0.9 over black,
       bench 2026-11-02) drove straight over the END marker. */
    /* Both channels over 700 for 3 mm is a junction (D69, D72): the real
       car missed every junction at 800 for 12 mm (a wide sensor spot
       keeps an 18 mm bar above 800 for only part of it; at a slant the
       two sensors cross it at different moments).  A bar beside the line
       during a capture is told apart below by reading the symbol. */
    int32_t const on = JUNCTION_ON_FREE;
    uint32_t const need_mm = JUNCTION_FREE_MM;
    bool const b_both = g_lc.b_junction ?
                        ((n_left > BLACK_ON) && (n_right > BLACK_ON)) :
                        ((n_left > on) && (n_right > on));
    int32_t    err = n_left - n_right;   /* -1000 .. +1000 */

    if (b_left || b_right)
    {
        g_lc.last_black = odom;
    }
    contact_track(0, n_left, b_left);
    contact_track(1, n_right, b_right);

    if (0 == g_lc.present_on)
    {
        g_lc.b_present = b_left || b_right;     /* BETWEEN, no grey cue */
    }
    else if ((n_left + n_right) > g_lc.present_on)
    {
        g_lc.b_present = true;
    }
    else if ((n_left + n_right) < g_lc.present_off)
    {
        g_lc.b_present = false;
    }
    else
    {
        /* Hysteresis band. */
    }

    /* Black-run length for T5's line-versus-bar test. */
    if (b_left || b_right)
    {
        if (!g_lc.b_black)
        {
            g_lc.black_start = odom;
        }
        g_lc.black_run = odom_delta_mm(odom, g_lc.black_start);
    }
    g_lc.b_black = b_left || b_right;

    /* Junction (L2) and end marker. */
    if (b_both)
    {
        if (!g_lc.b_junction)
        {
            g_lc.b_junction = true;
            g_lc.junction_start = odom;
            g_lc.b_no_end = cmd_waiting();      /* D73 */
        }
        bool b_take = (odom_delta_mm(odom, g_lc.junction_start) >= need_mm);

        /* During a barcode capture (D72): a junction close after the
           barcode was swept into the capture (real car 2026-11-19 17:54:
           decoded 150 mm late, junction missed); a bar beside the line
           can also darken both channels.  Read the symbol first: if it
           decodes, its command is for this junction (g_cross_word tells
           T6); if not, this was a bar, not a junction: keep capturing. */
        if (b_take && (LSTATE_JUNCTION != g_lc.lstate) &&
            (BC_CAPTURE == g_lc.bc))
        {
            if (g_lc.b_junc_tried)
            {
                b_take = false;
            }
            else
            {
                g_lc.b_junc_tried = true;
                if (barcode_search() || barcode_search_trailing())
                {
                    g_cross_word = bits_put((uint32_t)g_lc.junction_start,
                                            CROSS_VALID_SHIFT, 1U, 1U);
                    tm_printf((UB *)"T2: junction right after the barcode\n");
                }
                else
                {
                    b_take = false;
                }
            }
        }
        if (b_take)
        {
            if (LSTATE_JUNCTION != g_lc.lstate)
            {
                set_lstate(LSTATE_JUNCTION);
                (void)tk_set_flg(flg_sys, FLG_JUNCTION);
                barcode_abort();    /* the junction bar crosses it too */
                tm_printf((UB *)"T2: junction (both sensors dark, L %d R %d)"
                          "%s\n", (INT)n_left, (INT)n_right,
                          (UB *)(cmd_waiting() ? ", command waiting" : ""));
            }
            /* Not with a command waiting when this area began: the real
               crossroads read dark for over 60 mm and ended the run
               (2026-11-19 18:20, D73).  The command is for it. */
            if ((!g_lc.b_end) && (!g_lc.b_no_end) &&
                (odom_delta_mm(odom, g_lc.junction_start) >= LINE_END_MM))
            {
                g_lc.b_end = true;
                (void)tk_set_flg(flg_sys, FLG_END);
            }
        }
        /* Steering held only for a junction (or the wait to decide):
           not for a bar beside the line in a capture (D72). */
        if (b_take || (BC_CAPTURE != g_lc.bc) ||
            (LSTATE_JUNCTION == g_lc.lstate))
        {
            err = 0;
        }
    }
    else
    {
        g_lc.b_junction = false;
        g_lc.b_junc_tried = false;
        g_lc.b_end = false;         /* re-armed for the next black area */
        if (LSTATE_JUNCTION == g_lc.lstate)
        {
            set_lstate(LSTATE_TRACK);
        }
    }

    /* Lost (L3): no line under the pair for LINE_LOST_MM.  Until then
       steer hard toward the side the line was last seen. */
    if (g_lc.b_present)
    {
        g_lc.white_start = odom;
        if ((err > SIDE_MIN) || (err < -SIDE_MIN))
        {
            g_lc.last_sign = (err > 0) ? 1 : -1;
        }
        if (LSTATE_LOST == g_lc.lstate)
        {
            set_lstate(LSTATE_TRACK);
        }
    }
    else
    {
        /* BETWEEN with no grey cue: both white normally means centred;
           only a black reading just before means the line escaped. */
        if ((0 != g_lc.present_on) ||
            (g_lc.b_escaped &&
             (odom_delta_mm(odom, g_lc.last_black) < ESCAPE_MM)))
        {
            err = g_lc.last_sign * PERMILLE;
        }
        /* With the line between the sensors, both white is also what a
           centred car sees, so it cannot tell centred from lost.  Probe:
           after PROBE_AFTER_MM of white, steer gently toward the side the
           line was last seen.  Centred, the line soon comes under a sensor
           and following resumes; really gone, the probe turns back toward
           it.  Only if that fails too is the line lost and searched for:
           before, a car driving straight and centred was declared lost
           every 250 mm and swerved into a search (bench 2026-11-10: 74
           searches, most over in 0.2 s). */
        else if ((0 == g_lc.present_on) &&
                 (odom_delta_mm(odom, g_lc.white_start) >= PROBE_AFTER_MM))
        {
            err = g_lc.last_sign * PROBE_ERR;
        }
        else
        {
            /* Centred: hold the heading. */
        }
        if ((LSTATE_TRACK == g_lc.lstate) &&
            (odom_delta_mm(odom, g_lc.white_start) >= g_lc.lost_mm))
        {
            set_lstate(LSTATE_LOST);
            (void)tk_set_flg(flg_sys, FLG_LINE_LOST);
            barcode_abort();
        }
    }

    g_lc.acc_err += err;
}

static void
publish (void)
{
    int32_t const err = (g_lc.acc_err * ERR_FULL) /
                        (PERMILLE * (int32_t)PUBLISH_PERIODS);
    uint32_t      word = 0U;

    g_lc.acc_err = 0;
    g_lc.seq++;

    word = bits_put(word, LINE_ERR_SHIFT, LINE_ERR_BITS,
                    bits_from_signed(err, LINE_ERR_BITS));
    word = bits_put(word, LINE_STATE_SHIFT, LINE_STATE_BITS, g_lc.lstate);
    word = bits_put(word, LINE_JUNCTION_SHIFT, 1U, g_lc.b_junction ? 1U : 0U);
    word = bits_put(word, LINE_END_SHIFT, 1U, g_lc.b_end ? 1U : 0U);
    word = bits_put(word, LINE_SEQ_SHIFT, LINE_SEQ_BITS, g_lc.seq);
    word = bits_put(word, LINE_LASTBC_SHIFT, LINE_LASTBC_BITS, g_lc.last_bc);
    word = bits_put(word, LINE_REJECT_SHIFT, LINE_REJECT_BITS, g_lc.reject);
    word = bits_put(word, LINE_RUN_SHIFT, LINE_RUN_BITS,
                    bits_sat_u(g_lc.black_run, LINE_RUN_BITS));
    word = bits_put(word, LINE_BLACK_SHIFT, 1U, g_lc.b_black ? 1U : 0U);
    word = bits_put(word, LINE_CAPTURE_SHIFT, 1U,
                    (BC_CAPTURE == g_lc.bc) ?
                    1U : 0U);
    g_line_word = word;             /* one aligned store (rule R1) */
}

#if APP_BARCODE_TEST
/* Raw trace (bench): the barcode reading every ~1 mm while the car moves,
   printed in short lines once it stops, to see in the log what the sensor
   really saw: thin gaps as dips, or nothing. */
static void
barcode_trace (int32_t raw)
{
    if (g_lc.b_trace_out)
    {
        if (++g_lc.trace_wait >= BC_TRACE_LINE_MS)
        {
            uint32_t idx;

            g_lc.trace_wait = 0U;
            tm_printf((UB *)"T2: trace %d:", (INT)g_lc.trace_out);
            for (idx = 0U; (idx < BC_TRACE_PER_LINE) &&
                 (g_lc.trace_out < g_lc.trace_n); idx++)
            {
                tm_printf((UB *)" %d", (INT)g_lc.trace[g_lc.trace_out]);
                g_lc.trace_out++;
            }
            tm_printf((UB *)"\n");
            if (g_lc.trace_out >= g_lc.trace_n)
            {
                tm_printf((UB *)"T2: trace saved in the log, %d mm\n",
                          (INT)g_lc.trace_n);
                g_lc.b_trace_out = false;
                g_lc.trace_n = 0U;
                g_lc.trace_idle = 0U;
            }
        }
        return;
    }

    if ((g_lc.bc_now.d - g_lc.trace_d) >= BC_TRACE_EDGES)
    {
        g_lc.trace_d = g_lc.bc_now.d;
        g_lc.trace_idle = 0U;
        if (g_lc.trace_n < BC_TRACE_MAX)
        {
            g_lc.trace[g_lc.trace_n] = (uint8_t)(raw / BC_TRACE_SCALE);
            g_lc.trace_n++;
        }
    }
    else if ((g_lc.trace_n > 0U) &&
             (++g_lc.trace_idle >= BC_TRACE_IDLE_MS))
    {
        if (g_lc.trace_n >= BC_TRACE_MIN)
        {
            tm_printf((UB *)"T2: trace start, %d samples, raw/41, 1 mm"
                      " apart\n", (INT)g_lc.trace_n);
            g_lc.trace_out = 0U;
            g_lc.trace_wait = 0U;
            g_lc.b_trace_out = true;
        }
        else
        {
            g_lc.trace_n = 0U;
            g_lc.trace_idle = 0U;
        }
    }
    else
    {
        /* Still, or moving between samples. */
    }
}

/* Barcode bench (make BARCODE_TEST=1): the barcode channel levels itself
   from the darkest and lightest readings seen, both drifting slowly back
   toward the current reading, and is decoded all the time. */
static void
barcode_bench (uint16_t odom)
{
    int32_t const raw = (int32_t)g_lc.raw[IR_CH_BARCODE];
    int32_t       span;
    int32_t       n = 0;

    if (raw < g_lc.bc_lo)
    {
        g_lc.bc_lo = raw;
    }
    if (raw > g_lc.bc_hi)
    {
        g_lc.bc_hi = raw;
    }
    if (++g_lc.bc_relax >= BC_RANGE_RELAX_MS)
    {
        g_lc.bc_relax = 0U;
        if (g_lc.bc_lo < raw)
        {
            g_lc.bc_lo++;
        }
        if (g_lc.bc_hi > raw)
        {
            g_lc.bc_hi--;
        }
    }
    span = g_lc.bc_hi - g_lc.bc_lo;
    if (span >= (int32_t)CAL_MIN_SPAN)
    {
        n = ((raw - g_lc.bc_lo) * PERMILLE) / span;     /* dark = high */
    }
    barcode_step(n, odom);
    barcode_trace(raw);

    if (++g_lc.bc_status >= BC_STATUS_MS)
    {
        g_lc.bc_status = 0U;
        tm_printf((UB *)"BCTEST D0 %d (black reads %d)  -> %s%s\n",
                  (INT)g_lc.d0_level, g_lc.b_d0_black_high ? 1 : 0,
                  (span < (int32_t)CAL_MIN_SPAN) ?
                  "pass the sensor over black and white once" :
                  (g_lc.b_bar ? "BLACK" : "white"),
                  (BC_QUIET == g_lc.bc) ? ", ready for a barcode" :
                  ((BC_IDLE == g_lc.bc) ? ", waiting for white" :
                   ", reading"));
    }
}
#endif

/* The barcode sensor's D0 as a reading the rest of T2 already handles:
   black is full scale, white zero.  Which level is black is learnt from
   the long-run average (the sensor is mostly over white, and over white
   at power-up), so either board polarity works. */
static uint16_t
barcode_digital (void)
{
    uint32_t const level = ir_barcode_level();

    if (!g_lc.b_d0_started)
    {
        g_lc.b_d0_started = true;
        g_lc.d0_level = level;
        g_lc.d0_avg = (int32_t)level * BC_D0_FULL;
        g_lc.b_d0_black_high = (0U == level);
    }

    if (level != g_lc.d0_level)
    {
        if (++g_lc.d0_count >= BC_D0_DEBOUNCE_MS)
        {
            g_lc.d0_level = level;
            g_lc.d0_count = 0U;
        }
    }
    else
    {
        g_lc.d0_count = 0U;
    }

    g_lc.d0_avg += (((int32_t)g_lc.d0_level * BC_D0_FULL) - g_lc.d0_avg) /
                   (1 << BC_D0_AVG_SHIFT);
    if (g_lc.b_d0_black_high && (g_lc.d0_avg > BC_D0_WHITE_HIGH))
    {
        g_lc.b_d0_black_high = false;
        tm_printf((UB *)"T2: barcode D0 reads black as 0\n");
    }
    else if ((!g_lc.b_d0_black_high) && (g_lc.d0_avg < BC_D0_WHITE_LOW))
    {
        g_lc.b_d0_black_high = true;
        tm_printf((UB *)"T2: barcode D0 reads black as 1\n");
    }
    else
    {
        /* Polarity unchanged. */
    }

    return ((g_lc.d0_level == (g_lc.b_d0_black_high ? 1U : 0U)) ?
            (uint16_t)ADC_FULL_SCALE : 0U);
}

static void
line_sample (void)
{
    uint16_t const odom = odom_now_mm();
    uint32_t       ch;

    for (ch = 0U; ch < IR_CHANNELS; ch++)
    {
        if (IR_CH_BARCODE == ch)
        {
            g_lc.raw[ch] = barcode_digital();
        }
        else if (ir_adc_read(ch, &g_lc.raw[ch]))
        {
            g_lc.adc_fails = 0U;
        }
        else if (++g_lc.adc_fails >= ADC_FAIL_LIMIT)
        {
            /* A dead ADC, not one late conversion: the previous value
               stands in for a single miss.  Reported once per fault. */
            if (0U == ((uint32_t)fault_get() & FAULT_SELF_TEST))
            {
                tm_printf((UB *)"T2: ADC not answering (channel %d)\n",
                          (INT)ch);
            }
            fault_set(FAULT_SELF_TEST);
            g_lc.adc_fails = 0U;
        }
        else
        {
            /* Keep the last reading of this channel. */
        }
    }

#if APP_BARCODE_TEST
    barcode_bench(odom);
#endif
    if (g_lc.b_cal_active)
    {
        calibrate_sample();
    }
    else if (g_lc.b_calibrated)
    {
        cal_track();
        line_step(normalise(IR_CH_LEFT), normalise(IR_CH_RIGHT), odom);
        if (LSTATE_TRACK == g_lc.lstate)
        {
            barcode_step(normalise(IR_CH_BARCODE), odom);
        }
    }
    else
    {
        /* Uncalibrated (L0): publish a zero error only. */
    }

    g_lc.tick++;
    if (g_lc.tick >= PUBLISH_PERIODS)
    {
        g_lc.tick = 0U;
        calibrate_poll();
        publish();
    }
}

EXPORT void line_task (INT stacd, void * p_exinf)
{
    INT queued;

    (void)stacd;
    (void)p_exinf;

    ir_adc_init();
    g_lc.lstate = LSTATE_CALIBRATE;
    g_lc.bc = BC_IDLE;
    g_lc.last_sign = 1;
    g_lc.present_on = PRESENT_ON;
    g_lc.present_off = PRESENT_OFF;
    g_lc.lost_mm = LINE_LOST_MM;
    g_lc.quiet_start = odom_now_mm();
    calibrate_reset();
#if APP_BARCODE_TEST
    g_lc.bc_lo = (int32_t)ADC_FULL_SCALE;
    g_lc.bc_hi = BC_BLACK_GUESS;
    tm_printf((UB *)"T2: BARCODE TEST build -- type scan, or press"
              " GP20, to creep over a barcode\n");
#endif

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);     /* released by the 1 ms waker */
        queued = tk_can_wup(TSK_SELF);
        if (queued > 0)
        {
            g_line_overruns += (UW)queued;
        }
        line_sample();
    }
}

/*** end of file ***/
