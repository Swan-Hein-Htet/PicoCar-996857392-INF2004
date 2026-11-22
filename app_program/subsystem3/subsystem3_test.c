/** @file subsystem3_test.c
 *
 * @brief On-target tests for subsystem 3.  Built only with: make TEST=1
 *        Record results in testing/subsystem3/TEST_LOG.md.
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

#define NARROW_MS       12U     /* 3 mm at 250 mm/s */
#define WIDE_MS         36U
#define SETTLE_MS       1000

/* Decode every character at one speed, forwards and reversed. */
static bool
roundtrip_all (uint8_t narrow, uint8_t wide, bool b_reverse)
{
    uint8_t  widths[BARCODE_MAX_ELEMENTS];
    uint8_t  flipped[BARCODE_MAX_ELEMENTS];
    uint8_t  idx;
    uint8_t  got = 0U;
    uint32_t el;

    for (idx = 0U; idx < BARCODE_INDEX_STAR; idx++)
    {
        (void)barcode_encode(idx, narrow, wide, widths);
        for (el = 0U; el < BARCODE_MAX_ELEMENTS; el++)
        {
            flipped[el] = widths[BARCODE_MAX_ELEMENTS - 1U - el];
        }
        if ((BC_REJ_NONE != barcode_decode(b_reverse ? flipped : widths,
                                           BARCODE_MAX_ELEMENTS, &got)) ||
            (got != idx))
        {
            return (false);
        }
    }
    return (true);
}

EXPORT void subsystem3_test (void)
{
    uint8_t  widths[BARCODE_MAX_ELEMENTS + 1U];
    uint8_t  got = 0U;
    uint32_t el;
    UW       seq_before;
    UW       ovr_before;

    TEST_INIT();

    TEST_CHECK("S3-U01", roundtrip_all(NARROW_MS, WIDE_MS, false),
               "all 43 Code 39 characters decode at 250 mm/s");
    TEST_CHECK("S3-U02", roundtrip_all(NARROW_MS, WIDE_MS, true),
               "all characters decode when read backwards");
    TEST_CHECK("S3-U03", roundtrip_all(9U, 30U, false) &&
               roundtrip_all(30U, 75U, false),
               "speed independent: 100-350 mm/s, ratios 2.5-3.3 (NFR8)");

    /* Speed change between characters: second half 40% slower. */
    (void)barcode_encode(10U, NARROW_MS, WIDE_MS, widths);
    for (el = 10U; el < BARCODE_MAX_ELEMENTS; el++)
    {
        widths[el] = (uint8_t)(((uint32_t)widths[el] * 14U) / 10U);
    }
    TEST_CHECK("S3-U04",
               (BC_REJ_NONE == barcode_decode(widths, BARCODE_MAX_ELEMENTS,
                                              &got)) && (10U == got),
               "decodes 'A' through a mid-symbol speed change");

    (void)barcode_encode(10U, NARROW_MS, WIDE_MS, widths);
    TEST_CHECK("S3-U05",
               BC_REJ_COUNT == barcode_decode(widths, 28U, &got),
               "truncated symbol rejected, never guessed (FR6)");
    widths[BARCODE_MAX_ELEMENTS] = NARROW_MS;
    TEST_CHECK("S3-U06",
               BC_REJ_OVERRUN == barcode_decode(widths, 30U, &got),
               "marks after the symbol rejected (FR6)");
    widths[4] = 255U;
    TEST_CHECK("S3-U07",
               BC_REJ_NONE != barcode_decode(widths, BARCODE_MAX_ELEMENTS,
                                             &got),
               "saturated / out-of-ratio element rejected");
    (void)barcode_encode(10U, NARROW_MS, WIDE_MS, widths);
    widths[0] = WIDE_MS;            /* start '*' now has 4 wide elements */
    TEST_CHECK("S3-U08",
               BC_REJ_NONE != barcode_decode(widths, BARCODE_MAX_ELEMENTS,
                                             &got),
               "corrupted start character rejected");

    seq_before = bits_get(g_line_word, LINE_SEQ_SHIFT, LINE_SEQ_BITS);
    ovr_before = g_line_overruns;
    tk_dly_tsk(SETTLE_MS);
    TEST_CHECK("S3-U09",
               seq_before != bits_get(g_line_word, LINE_SEQ_SHIFT,
                                      LINE_SEQ_BITS),
               "T2 republishes the line word (NFR2)");
    TEST_CHECK("S3-U10", g_line_overruns == ovr_before,
               "no 1 ms release overruns over 1 s (NFR1)");

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */

/*** end of file ***/
