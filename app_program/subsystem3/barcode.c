/** @file barcode.c
 *
 * @brief Code 39 decoder (see barcode.h).  No kernel calls, no globals
 *        written: safe to call from T2 and from the host tests.
 */

#include "barcode.h"

#define RATIO_SCALE         100U    /* ratios are in percent             */
#define RATIO_MIN           50U     /* below 0.5x the reference: reject  */
#define RATIO_WIDE          200U    /* 2.0x: narrow/wide boundary        */
#define RATIO_MAX           450U    /* above 4.5x: reject                */
#define RATIO_GAP_MAX       350U    /* inter-character gap upper bound   */
#define WIDE_PER_CHAR       3U
#define WIDTH_SATURATED     255U
#define CHARS_PER_SYMBOL    3U

char const g_barcode_chars[BARCODE_CHAR_COUNT] =
{
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
    'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
    'U', 'V', 'W', 'X', 'Y', 'Z', '-', '.', ' ', '$',
    '/', '+', '%', '*'
};

/* Nine elements, bar first, most significant bit first; 1 = wide. */
static uint16_t const g_patterns[BARCODE_CHAR_COUNT] =
{
    0x034U, 0x121U, 0x061U, 0x160U, 0x031U, 0x130U, 0x070U, 0x025U, /* 0-7 */
    0x124U, 0x064U, 0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U, /* 8-F */
    0x00DU, 0x10CU, 0x04CU, 0x01CU, 0x103U, 0x043U, 0x142U, 0x013U, /* G-N */
    0x112U, 0x052U, 0x007U, 0x106U, 0x046U, 0x016U, 0x181U, 0x0C1U, /* O-V */
    0x1C0U, 0x091U, 0x190U, 0x0D0U, 0x085U, 0x184U, 0x0C4U, 0x0A8U, /* W-$ */
    0x0A2U, 0x08AU, 0x02AU, 0x094U                                  /* /-* */
};

/*!
 * @brief Classify the nine elements of one character.
 * @return BC_REJ_NONE and the 9-bit pattern in *p_pattern, or a reason.
 */
static uint8_t
decode_char (uint8_t const * p_widths, uint16_t * p_pattern,
             uint32_t * p_ref)
{
    uint8_t  sorted[BARCODE_CHAR_ELEMENTS];
    uint32_t idx;
    uint32_t pos;
    uint32_t ref;
    uint32_t wide_count = 0U;
    uint32_t pattern = 0U;

    for (idx = 0U; idx < BARCODE_CHAR_ELEMENTS; idx++)
    {
        if (WIDTH_SATURATED <= p_widths[idx])
        {
            return (BC_REJ_RATIO);
        }
        sorted[idx] = p_widths[idx];
    }

    /* Insertion sort: nine elements, bounded and branch-light. */
    for (idx = 1U; idx < BARCODE_CHAR_ELEMENTS; idx++)
    {
        uint8_t const key = sorted[idx];

        pos = idx;
        while ((pos > 0U) && (sorted[pos - 1U] > key))
        {
            sorted[pos] = sorted[pos - 1U];
            pos--;
        }
        sorted[pos] = key;
    }

    /* Reference narrow width: median of the six narrowest elements. */
    ref = ((uint32_t)sorted[2] + (uint32_t)sorted[3] + 1U) / 2U;
    if (0U == ref)
    {
        return (BC_REJ_RATIO);
    }

    for (idx = 0U; idx < BARCODE_CHAR_ELEMENTS; idx++)
    {
        uint32_t const ratio = ((uint32_t)p_widths[idx] * RATIO_SCALE) / ref;

        if ((ratio < RATIO_MIN) || (ratio > RATIO_MAX))
        {
            return (BC_REJ_RATIO);
        }

        pattern <<= 1U;
        if (ratio >= RATIO_WIDE)
        {
            pattern |= 1U;
            wide_count++;
        }
    }

    if (WIDE_PER_CHAR != wide_count)
    {
        return (BC_REJ_PATTERN);
    }

    *p_pattern = (uint16_t)pattern;
    *p_ref = ref;
    return (BC_REJ_NONE);
}

static uint8_t
lookup (uint16_t pattern, uint8_t * p_index)
{
    uint8_t idx;

    for (idx = 0U; idx < BARCODE_CHAR_COUNT; idx++)
    {
        if (g_patterns[idx] == pattern)
        {
            *p_index = idx;
            return (BC_REJ_NONE);
        }
    }
    return (BC_REJ_PATTERN);
}

/*!
 * @brief Decode a 29-element array in the given order.
 */
static uint8_t
decode_ordered (uint8_t const * p_widths, uint8_t * p_index)
{
    uint8_t  chars[CHARS_PER_SYMBOL];
    uint32_t chr;
    uint8_t  reason;

    for (chr = 0U; chr < CHARS_PER_SYMBOL; chr++)
    {
        uint8_t const * const p_char =
            &p_widths[chr * (BARCODE_CHAR_ELEMENTS + 1U)];
        uint16_t pattern = 0U;
        uint32_t ref = 0U;

        reason = decode_char(p_char, &pattern, &ref);
        if (BC_REJ_NONE != reason)
        {
            return (reason);
        }

        /* The gap after this character must be narrow-ish. */
        if (chr < (CHARS_PER_SYMBOL - 1U))
        {
            uint32_t const gap = p_char[BARCODE_CHAR_ELEMENTS];

            if (((gap * RATIO_SCALE) / ref) > RATIO_GAP_MAX)
            {
                return (BC_REJ_RATIO);
            }
        }

        reason = lookup(pattern, &chars[chr]);
        if (BC_REJ_NONE != reason)
        {
            return (reason);
        }
    }

    if ((BARCODE_INDEX_STAR != chars[0]) ||
        (BARCODE_INDEX_STAR != chars[2]) ||
        (BARCODE_INDEX_STAR == chars[1]))
    {
        return (BC_REJ_FRAME);
    }

    *p_index = chars[1];
    return (BC_REJ_NONE);
}

uint8_t
barcode_decode (uint8_t const * p_widths, uint32_t count, uint8_t * p_index)
{
    uint8_t  reversed[BARCODE_MAX_ELEMENTS];
    uint32_t idx;
    uint8_t  reason;

    if (BARCODE_MAX_ELEMENTS != count)
    {
        return ((count > BARCODE_MAX_ELEMENTS) ? BC_REJ_OVERRUN
                                               : BC_REJ_COUNT);
    }

    reason = decode_ordered(p_widths, p_index);
    if (BC_REJ_NONE == reason)
    {
        return (reason);
    }

    /* Read backwards (car driving the other way past the symbol). */
    for (idx = 0U; idx < BARCODE_MAX_ELEMENTS; idx++)
    {
        reversed[idx] = p_widths[BARCODE_MAX_ELEMENTS - 1U - idx];
    }

    if (BC_REJ_NONE == decode_ordered(reversed, p_index))
    {
        return (BC_REJ_NONE);
    }

    return (reason);
}

static uint8_t
decode_loose_ordered (uint8_t const * p_widths, uint8_t * p_index)
{
    uint8_t  chars[CHARS_PER_SYMBOL];
    uint32_t chr;

    for (chr = 0U; chr < CHARS_PER_SYMBOL; chr++)
    {
        uint8_t const * const p_char =
            &p_widths[chr * (BARCODE_CHAR_ELEMENTS + 1U)];
        uint32_t pattern = 0U;
        uint32_t wide;

        for (wide = 0U; wide < WIDE_PER_CHAR; wide++)
        {
            uint32_t best = 0U;
            uint32_t top = 0U;
            uint32_t idx;

            for (idx = 0U; idx < BARCODE_CHAR_ELEMENTS; idx++)
            {
                uint32_t const mask =
                    1UL << (BARCODE_CHAR_ELEMENTS - 1U - idx);

                if ((0U == (pattern & mask)) && (p_char[idx] >= top))
                {
                    top = p_char[idx];
                    best = mask;
                }
            }
            pattern |= best;
        }
        if (BC_REJ_NONE != lookup((uint16_t)pattern, &chars[chr]))
        {
            return (BC_REJ_PATTERN);
        }
    }

    if ((BARCODE_INDEX_STAR != chars[0]) ||
        (BARCODE_INDEX_STAR != chars[2]) ||
        (BARCODE_INDEX_STAR == chars[1]))
    {
        return (BC_REJ_FRAME);
    }

    *p_index = chars[1];
    return (BC_REJ_NONE);
}

/* Median of every other element starting at first (bars: 0, spaces: 1). */
static uint32_t
median_of (uint8_t const * p_widths, uint32_t first)
{
    uint8_t  sorted[BARCODE_MAX_ELEMENTS];
    uint32_t n = 0U;
    uint32_t idx;
    uint32_t pos;

    for (idx = first; idx < BARCODE_MAX_ELEMENTS; idx += 2U)
    {
        pos = n;
        while ((pos > 0U) && (sorted[pos - 1U] > p_widths[idx]))
        {
            sorted[pos] = sorted[pos - 1U];
            pos--;
        }
        sorted[pos] = p_widths[idx];
        n++;
    }
    return (sorted[n / 2U]);
}

uint8_t
barcode_decode_balanced (uint8_t const * p_widths, uint32_t count,
                         uint8_t * p_index)
{
    uint8_t  fixed[BARCODE_MAX_ELEMENTS];
    int32_t  shift;
    uint32_t idx;

    if (BARCODE_MAX_ELEMENTS != count)
    {
        return (BC_REJ_COUNT);
    }

    /* Most bars and most spaces are narrow, so the two medians are one
       narrow width seen as a bar and as a space: half their difference is
       how far every bar was thinned (or thickened) and every space the
       opposite.  Shift it back, then decode as usual. */
    shift = ((int32_t)median_of(p_widths, 1U) -
             (int32_t)median_of(p_widths, 0U)) / 2;
    for (idx = 0U; idx < BARCODE_MAX_ELEMENTS; idx++)
    {
        int32_t v = (int32_t)p_widths[idx] +
                    ((0U == (idx % 2U)) ? shift : -shift);

        if (v < 1)
        {
            v = 1;
        }
        fixed[idx] = (uint8_t)((v > (int32_t)(WIDTH_SATURATED - 1U)) ?
                               (WIDTH_SATURATED - 1U) : (uint32_t)v);
    }
    return (barcode_decode(fixed, BARCODE_MAX_ELEMENTS, p_index));
}

uint8_t
barcode_decode_loose (uint8_t const * p_widths, uint32_t count,
                      uint8_t * p_index)
{
    uint8_t  reversed[BARCODE_MAX_ELEMENTS];
    uint32_t idx;

    if (BARCODE_MAX_ELEMENTS != count)
    {
        return (BC_REJ_COUNT);
    }
    if (BC_REJ_NONE == decode_loose_ordered(p_widths, p_index))
    {
        return (BC_REJ_NONE);
    }
    for (idx = 0U; idx < BARCODE_MAX_ELEMENTS; idx++)
    {
        reversed[idx] = p_widths[BARCODE_MAX_ELEMENTS - 1U - idx];
    }
    return (decode_loose_ordered(reversed, p_index));
}

uint32_t
barcode_encode (uint8_t index, uint8_t narrow, uint8_t wide,
                uint8_t * p_widths)
{
    uint8_t  chars[CHARS_PER_SYMBOL];
    uint32_t chr;
    uint32_t bit;
    uint32_t out = 0U;

    if (index >= BARCODE_CHAR_COUNT)
    {
        return (0U);
    }

    chars[0] = BARCODE_INDEX_STAR;
    chars[1] = index;
    chars[2] = BARCODE_INDEX_STAR;

    for (chr = 0U; chr < CHARS_PER_SYMBOL; chr++)
    {
        uint32_t const pattern = g_patterns[chars[chr]];

        for (bit = 0U; bit < BARCODE_CHAR_ELEMENTS; bit++)
        {
            uint32_t const mask = 1UL << (BARCODE_CHAR_ELEMENTS - 1U - bit);

            p_widths[out] = (0U != (pattern & mask)) ? wide : narrow;
            out++;
        }

        if (chr < (CHARS_PER_SYMBOL - 1U))
        {
            p_widths[out] = narrow;     /* inter-character gap */
            out++;
        }
    }

    return (out);
}

/*** end of file ***/
