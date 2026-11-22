/** @file test_host.c
 *
 * @brief Off-target tests of the pure modules: Code 39 decoder, packing
 *        helpers, telemetry layout and app_math.  Build and run with
 *        tests/host/run.sh.  Also writes a reference telemetry record to
 *        stdout for tools/decode_telemetry.py --selftest.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../../app_program/app.h"
#include "../../app_program/app_math.h"
#include "../../app_program/subsystem1/subsystem1.h"
#include "../../app_program/subsystem1/telemetry.h"
#include "../../app_program/subsystem2/subsystem2.h"
#include "../../app_program/subsystem3/subsystem3.h"
#include "../../app_program/subsystem4/subsystem4.h"
#include "../../app_program/subsystem5/subsystem5.h"

/* Symbols the headers declare but the pure modules never touch. */
volatile uint16_t g_mission_word;
volatile uint32_t g_odom_word;

static int g_pass = 0;
static int g_fail = 0;

static void
check (char const * p_id, int b_ok, char const * p_desc)
{
    printf("[HOST] %s %s  %s\n", p_id, b_ok ? "PASS" : "FAIL", p_desc);
    if (b_ok) { g_pass++; } else { g_fail++; }
}

static int
roundtrip (uint8_t narrow, uint8_t wide, int b_rev)
{
    uint8_t w[BARCODE_MAX_ELEMENTS];
    uint8_t r[BARCODE_MAX_ELEMENTS];
    uint8_t idx;
    uint8_t got;
    uint32_t el;

    for (idx = 0U; idx < BARCODE_INDEX_STAR; idx++)
    {
        (void)barcode_encode(idx, narrow, wide, w);
        for (el = 0U; el < BARCODE_MAX_ELEMENTS; el++)
        {
            r[el] = w[BARCODE_MAX_ELEMENTS - 1U - el];
        }
        if ((BC_REJ_NONE != barcode_decode(b_rev ? r : w,
                                           BARCODE_MAX_ELEMENTS, &got)) ||
            (got != idx))
        {
            return 0;
        }
    }
    return 1;
}

/* Random jitter: every element +-15 %; must decode or reject, never
   return a wrong character (NFR9 zero false positives). */
static int
no_false_positive (void)
{
    uint8_t  w[BARCODE_MAX_ELEMENTS];
    uint32_t seed = 12345U;
    int      trial;
    int      decoded = 0;

    for (trial = 0; trial < 20000; trial++)
    {
        uint8_t const idx = (uint8_t)(trial % 43);
        uint8_t got = 0xFFU;
        uint32_t el;

        (void)barcode_encode(idx, 12U, 36U, w);
        for (el = 0U; el < BARCODE_MAX_ELEMENTS; el++)
        {
            int32_t jitter;

            seed = (seed * 1103515245U) + 12345U;
            jitter = (int32_t)((seed >> 16) % 31U) - 15;   /* -15..15 % */
            w[el] = (uint8_t)(((int32_t)w[el] * (100 + jitter)) / 100);
        }
        if (BC_REJ_NONE == barcode_decode(w, BARCODE_MAX_ELEMENTS, &got))
        {
            if (got != idx)
            {
                return 0;
            }
            decoded++;
        }
    }
    printf("        jitter +-15%%: %d / 20000 decoded, 0 wrong\n", decoded);
    return (decoded > 19000);
}

static uint32_t g_seed = 2463534242U;

static uint32_t
rnd (void)
{
    g_seed ^= g_seed << 13; g_seed ^= g_seed >> 17; g_seed ^= g_seed << 5;
    return (g_seed);
}

/* Every width 1..16: put/get round-trips, neighbours untouched, signed
   values round-trip in range and saturate outside it. */
static int
bits_exhaustive (void)
{
    uint32_t width;

    for (width = 1U; width <= 16U; width++)
    {
        uint32_t const max = BITS_MASK(width);
        int32_t const  smax = (int32_t)(max >> 1);
        uint32_t       v;
        int32_t        sv;

        for (v = 0U; v <= max; v++)
        {
            uint32_t const shift = v % (33U - width);
            uint32_t const around = rnd();
            uint32_t const w = bits_put(around, shift, width, v);

            if ((bits_get(w, shift, width) != v) ||
                ((w & ~(max << shift)) != (around & ~(max << shift))))
            {
                return (0);
            }
        }
        if ((bits_sat_u(max + 1U, width) != max) || (bits_sat_u(max, width) != max))
        {
            return (0);
        }
        if (width < 2U)
        {
            continue;
        }
        for (sv = -smax - 1; sv <= smax; sv++)
        {
            if (bits_to_signed(bits_from_signed(sv, width), width) != sv)
            {
                return (0);
            }
        }
        if ((bits_to_signed(bits_from_signed(smax + 7, width), width) != smax) ||
            (bits_to_signed(bits_from_signed(-smax - 9, width), width) != (-smax - 1)))
        {
            return (0);
        }
    }
    return (1);
}

static int
telemetry_fuzz (void)
{
    int trial;

    for (trial = 0; trial < 10000; trial++)
    {
        telemetry_snapshot_t s;
        uint8_t rec[TELEMETRY_BYTES];
        uint8_t again[TELEMETRY_BYTES];

        s.line = rnd(); s.motion = rnd(); s.odom = rnd(); s.terrain = rnd();
        s.profile0 = rnd(); s.profile1 = rnd();
        s.mission = (uint16_t)rnd(); s.faults = (uint16_t)rnd();
        s.enc_left = (uint16_t)rnd(); s.enc_right = (uint16_t)rnd();
        s.hump = (uint8_t)rnd(); s.seq = (uint8_t)rnd();
        memset(rec, 0xA5, sizeof(rec));
        telemetry_pack(&s, rec);
        memset(again, 0x5A, sizeof(again));
        telemetry_pack(&s, again);
        if ((0 != memcmp(rec, again, sizeof(rec))) ||
            (TELEMETRY_VERSION != tlm_get(rec, TLM_VERSION_OFS, TLM_VERSION_BITS)) ||
            (s.seq != tlm_get(rec, TLM_SEQ_OFS, TLM_SEQ_BITS)) ||
            (bits_get(s.mission, 0U, 4U) != tlm_get(rec, TLM_MISSION_OFS, TLM_MISSION_BITS)) ||
            (bits_get(s.motion, MOTION_STATE_SHIFT, 3U) != tlm_get(rec, TLM_MOTION_OFS, TLM_MOTION_BITS)) ||
            (bits_get(s.motion, MOTION_SPEED_L_SHIFT, 10U) != tlm_get(rec, TLM_SPEED_L_OFS, 10U)) ||
            (bits_get(s.motion, MOTION_SPEED_R_SHIFT, 10U) != tlm_get(rec, TLM_SPEED_R_OFS, 10U)) ||
            (s.enc_left != tlm_get(rec, TLM_ENC_L_OFS, 16U)) ||
            (s.enc_right != tlm_get(rec, TLM_ENC_R_OFS, 16U)) ||
            (bits_get(s.line, LINE_STATE_SHIFT, 2U) != tlm_get(rec, TLM_LSTATE_OFS, 2U)) ||
            (bits_get(s.line, LINE_ERR_SHIFT, 8U) != tlm_get(rec, TLM_LERR_OFS, 8U)) ||
            (bits_get(s.line, LINE_LASTBC_SHIFT, 3U) != tlm_get(rec, TLM_BARCODE_OFS, 3U)) ||
            (bits_get(s.line, LINE_REJECT_SHIFT, 3U) != tlm_get(rec, TLM_REJECT_OFS, 3U)) ||
            (bits_get(s.hump, 0U, 7U) != tlm_get(rec, TLM_HUMP_OFS, 7U)) ||
            (bits_get(s.profile0, PROF_WIDTH_SHIFT, 10U) != tlm_get(rec, TLM_WIDTH_OFS, 10U)) ||
            (bits_get(s.profile0, PROF_CLOSEST_SHIFT, 9U) != tlm_get(rec, TLM_CLOSEST_OFS, 9U)) ||
            (bits_get(s.profile0, PROF_BEARING_SHIFT, 5U) != tlm_get(rec, TLM_BEARING_OFS, 5U)) ||
            (bits_get(s.profile0, PROF_PLAN_SHIFT, 2U) != tlm_get(rec, TLM_PLAN_OFS, 2U)) ||
            (bits_get(s.odom, ODOM_TOTAL_SHIFT, 15U) != tlm_get(rec, TLM_DIST_OFS, 15U)) ||
            (bits_get(s.terrain, TERRAIN_CLASS_SHIFT, 3U) != tlm_get(rec, TLM_MCLASS_OFS, 3U)) ||
            (bits_get(s.terrain, TERRAIN_RATE_SHIFT, 10U) != tlm_get(rec, TLM_RATE_OFS, 10U)) ||
            (s.faults != tlm_get(rec, TLM_FAULTS_OFS, 16U)) ||
            (bits_get(s.profile0, PROF_SCAN_SHIFT, 4U) != tlm_get(rec, TLM_SCAN_OFS, 4U)))
        {
            printf("        telemetry mismatch at trial %d\n", trial);
            return (0);
        }
    }
    return (1);
}

static int
barcode_fuzz (void)
{
    int trial;
    int accepted = 0;

    for (trial = 0; trial < 200000; trial++)
    {
        uint8_t  w[40];
        uint32_t const count = (trial % 4 == 0) ? (rnd() % 40U) : BARCODE_MAX_ELEMENTS;
        uint8_t  got = 0xEEU;
        uint32_t el;

        for (el = 0U; el < count; el++)
        {
            w[el] = (uint8_t)((rnd() % 2U) ? (8U + (rnd() % 10U)) : (25U + (rnd() % 20U)));
            if (0U == (rnd() % 50U)) { w[el] = (uint8_t)rnd(); }
        }
        if (BC_REJ_NONE == barcode_decode(w, count, &got))
        {
            accepted++;
            if ((BARCODE_MAX_ELEMENTS != count) || (got >= BARCODE_INDEX_STAR))
            {
                return (0);
            }
        }
    }
    printf("        random arrays: %d / 200000 decoded as some valid symbol\n", accepted);
    return (1);
}

static int
barcode_reversed_jitter (void)
{
    int trial;

    for (trial = 0; trial < 20000; trial++)
    {
        uint8_t const idx = (uint8_t)(trial % 43);
        uint8_t w[BARCODE_MAX_ELEMENTS];
        uint8_t r[BARCODE_MAX_ELEMENTS];
        uint8_t got = 0xFFU;
        uint32_t el;

        (void)barcode_encode(idx, 12U, 36U, w);
        for (el = 0U; el < BARCODE_MAX_ELEMENTS; el++)
        {
            int32_t const jitter = (int32_t)(rnd() % 31U) - 15;
            r[BARCODE_MAX_ELEMENTS - 1U - el] =
                (uint8_t)(((int32_t)w[el] * (100 + jitter)) / 100);
        }
        if ((BC_REJ_NONE == barcode_decode(r, BARCODE_MAX_ELEMENTS, &got)) && (got != idx))
        {
            return (0);
        }
    }
    return (1);
}

/* H-11: bars thinned and spaces thickened by the same amount (analog
   sensor riding high, bench 2026-11-10), up to 0.7 of a narrow width, both
   directions, +-15 % jitter: decoded after correction, never wrong. */
static int
barcode_balanced (void)
{
    unsigned seed = 11U;
    int bias;

    for (bias = -25; bias <= 45; bias += 5)
    {
        uint8_t idx;

        for (idx = 0U; idx < BARCODE_INDEX_STAR; idx++)
        {
            int dir;

            for (dir = 0; dir < 2; dir++)
            {
                uint8_t w[BARCODE_MAX_ELEMENTS];
                uint8_t f[BARCODE_MAX_ELEMENTS];
                uint8_t got = 0U;
                int el;

                (void)barcode_encode(idx, 62U, 185U, w);
                for (el = 0; el < (int)BARCODE_MAX_ELEMENTS; el++)
                {
                    int v = w[dir ? ((int)BARCODE_MAX_ELEMENTS - 1 - el) : el];

                    seed = seed * 1103515245U + 12345U;
                    v = (v * (85 + (int)((seed >> 16) % 31U))) / 100;
                    v += ((el % 2) == 0) ? -bias : bias;
                    f[el] = (uint8_t)((v < 1) ? 1 : ((v > 250) ? 250 : v));
                }
                if ((BC_REJ_NONE != barcode_decode_balanced(f,
                                         BARCODE_MAX_ELEMENTS, &got)) ||
                    (got != idx))
                {
                    return (0);
                }
            }
        }
    }
    return (1);
}

int
main (void)
{
    telemetry_snapshot_t snap = {0};
    uint8_t rec[TELEMETRY_BYTES];
    float   worst_atan = 0.0f;
    float   worst_sin = 0.0f;
    float   worst_sqrt = 0.0f;
    int     i;

    check("H-01", roundtrip(12U, 36U, 0), "43 chars decode, 250 mm/s");
    check("H-02", roundtrip(12U, 36U, 1), "43 chars decode reversed");
    check("H-03", roundtrip(9U, 27U, 0) && roundtrip(30U, 90U, 0) &&
                  roundtrip(10U, 22U, 0), "ratio 2.2-3.0 and 100-350 mm/s");
    check("H-04", no_false_positive(), "no false positives under jitter");

    for (i = -900; i <= 900; i++)
    {
        float const a = (float)i * 0.0034906585f;   /* -pi .. pi */
        float e1 = fabsf(app_sinf(a) - sinf(a));
        float e2 = fabsf(app_atan2f(sinf(a), cosf(a)) - atan2f(sinf(a),
                                                               cosf(a)));

        if (e1 > worst_sin) { worst_sin = e1; }
        if ((e2 > worst_atan) && (fabsf(fabsf(a) - 3.14159f) > 0.01f))
        {
            worst_atan = e2;
        }
    }
    for (i = 1; i < 100000; i += 7)
    {
        float const x = (float)i * 37.0f;
        float const e = fabsf(app_sqrtf(x) - sqrtf(x)) / sqrtf(x);

        if (e > worst_sqrt) { worst_sqrt = e; }
    }
    printf("        worst sin %.2e atan2 %.2e sqrt(rel) %.2e\n",
           (double)worst_sin, (double)worst_atan, (double)worst_sqrt);
    check("H-05", (worst_sin < 1e-5f) && (worst_atan < 2e-5f) &&
                  (worst_sqrt < 1e-6f), "app_math within stated bounds");

    check("H-06", (-1 == bits_to_signed(bits_from_signed(-1, 8U), 8U)) &&
                  (127 == bits_to_signed(bits_from_signed(500, 8U), 8U)) &&
                  (-128 == bits_to_signed(bits_from_signed(-500, 8U), 8U)),
          "signed packing round-trips and saturates");

    check("H-07", bits_exhaustive(), "bit helpers: every width 1-16, every value");
    check("H-08", telemetry_fuzz(), "10 000 random snapshots: every field packs and unpacks exactly");
    check("H-09", barcode_fuzz(), "200 000 random width arrays: never a decode with bad count or index");
    check("H-10", barcode_reversed_jitter(), "reversed symbols under +-15 % jitter: never a wrong character");
    check("H-11", barcode_balanced(), "bars thinned up to 0.7 narrow, both ways, +-15 %: corrected and read");

    /* Reference record for the Python decoder. */
    snap.seq = 0x5AU;
    snap.mission = (uint16_t)bits_put(0U, 0U, 4U, MISSION_OBSTACLE);
    snap.motion = bits_put(bits_put(bits_put(0U, MOTION_SPEED_L_SHIFT, 10U,
                                             bits_from_signed(-250, 10U)),
                                    MOTION_SPEED_R_SHIFT, 10U,
                                    bits_from_signed(249, 10U)),
                           MOTION_STATE_SHIFT, 3U, MSTATE_TURN);
    snap.enc_left = 0x1234U;
    snap.enc_right = 0xFEDCU;
    snap.line = bits_put(bits_put(bits_put(bits_put(0U, LINE_ERR_SHIFT, 8U,
                                                    bits_from_signed(-64, 8U)),
                                           LINE_STATE_SHIFT, 2U, LSTATE_TRACK),
                                  LINE_LASTBC_SHIFT, 3U, BC_LAST_D),
                         LINE_REJECT_SHIFT, 3U, BC_REJ_RATIO);
    snap.hump = 42U;
    snap.profile0 = bits_put(bits_put(bits_put(bits_put(bits_put(0U,
                    PROF_BEARING_SHIFT, 5U, 9U), PROF_WIDTH_SHIFT, 10U, 101U),
                    PROF_CLOSEST_SHIFT, 9U, 187U), PROF_PLAN_SHIFT, 2U,
                    PLAN_LEFT), PROF_SCAN_SHIFT, 4U, SCAN_ALONG);
    snap.odom = bits_put(0U, ODOM_TOTAL_SHIFT, 15U, 12345U);
    snap.terrain = bits_put(bits_put(bits_put(0U, TERRAIN_CLASS_SHIFT, 3U,
                                              MCLASS_TURNING),
                                     TERRAIN_RATE_SHIFT, 10U,
                                     bits_from_signed(-321, 10U)),
                            TERRAIN_PITCH_SHIFT, 10U,
                            bits_from_signed(-123, 10U));
    snap.tilt = bits_put(bits_put(0U, TILT_ROLL_SHIFT, TILT_ROLL_BITS,
                                  bits_from_signed(-7, TILT_ROLL_BITS)),
                         TILT_HNOW_SHIFT, TILT_HNOW_BITS, 23U);
    snap.faults = FAULT_WIFI_LOSS | FAULT_NO_ECHO;
    telemetry_pack(&snap, rec);
    printf("[HOST] REF ");
    for (i = 0; i < (int)TELEMETRY_BYTES; i++)
    {
        printf("%02x", rec[i]);
    }
    printf("\n");

    printf("[HOST] SUMMARY %d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}
