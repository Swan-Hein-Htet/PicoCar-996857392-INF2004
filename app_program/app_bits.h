/** @file app_bits.h
 *
 * @brief Minimum-bit packing helpers (Change Requirement 1).
 *
 * Every shared or transmitted application field is stored in an unsigned
 * word through explicit masks and shifts.  C bit-fields are not used for
 * shared data because their bit order is implementation-defined (Barr-C
 * 5.2.b) and Barr-C 5.3.a forbids signed bit-fields.  Signed quantities are
 * stored in two's-complement form inside an unsigned field and converted
 * with arithmetic only, so no bitwise operator ever touches signed data
 * (Barr-C 5.3.b).
 *
 * Packed access rules agreed at Tribunal 1 (NFR20 as amended):
 *   R1  One writer per packed storage unit; it writes the whole unit with
 *       one aligned store, readers use one aligned load.
 *   R2  A genuinely multi-writer unit is changed only inside a helper that
 *       performs the read-modify-write in a critical section (faults.c).
 *   R3  Nothing an ISR writes shares a storage unit with task-written data.
 *
 * This header is pure C99: it is shared by the firmware and the host tests.
 */

#ifndef APP_BITS_H
#define APP_BITS_H

#include <stdint.h>
#include <stdbool.h>

/* Mask of the low 'width' bits; width is 1..31. */
#define BITS_MASK(width)    ((uint32_t)((UINT32_C(1) << (width)) - 1U))

/*!
 * @brief Read an unsigned field from a packed word.
 */
static inline uint32_t
bits_get (uint32_t word, uint32_t shift, uint32_t width)
{
    return ((word >> shift) & BITS_MASK(width));
}

/*!
 * @brief Return 'word' with the field replaced by 'value' (truncated).
 */
static inline uint32_t
bits_put (uint32_t word, uint32_t shift, uint32_t width, uint32_t value)
{
    uint32_t const mask = BITS_MASK(width) << shift;

    return ((word & ~mask) | ((value << shift) & mask));
}

/*!
 * @brief Saturate an unsigned value to the largest value of 'width' bits.
 */
static inline uint32_t
bits_sat_u (uint32_t value, uint32_t width)
{
    return ((value > BITS_MASK(width)) ? BITS_MASK(width) : value);
}

/*!
 * @brief Encode a signed value as a 'width'-bit two's-complement field,
 *        saturating at the field limits.  Arithmetic only.
 */
static inline uint32_t
bits_from_signed (int32_t value, uint32_t width)
{
    int32_t const max_val = (int32_t)(BITS_MASK(width - 1U));
    int32_t const min_val = -max_val - 1;
    int32_t       clamped = value;

    if (clamped > max_val)
    {
        clamped = max_val;
    }
    else if (clamped < min_val)
    {
        clamped = min_val;
    }
    else
    {
        /* In range: nothing to do. */
    }

    if (clamped < 0)
    {
        clamped += (int32_t)(BITS_MASK(width)) + 1;
    }

    return ((uint32_t)clamped);
}

/*!
 * @brief Decode a 'width'-bit two's-complement field.  Arithmetic only.
 */
static inline int32_t
bits_to_signed (uint32_t raw, uint32_t width)
{
    uint32_t const field = raw & BITS_MASK(width);
    int32_t        value = (int32_t)field;

    if (field > BITS_MASK(width - 1U))
    {
        value -= (int32_t)(BITS_MASK(width)) + 1;
    }

    return (value);
}

#endif /* APP_BITS_H */

/*** end of file ***/
