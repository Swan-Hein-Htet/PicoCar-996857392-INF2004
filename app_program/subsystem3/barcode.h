/** @file barcode.h
 *
 * @brief Code 39 decoder: a pure function over an array of element
 *        widths, so it is unit-tested off-hardware (tests/host).
 *
 * A course symbol is "*X*": three characters of nine elements (five bars,
 * four spaces, three of them wide) separated by two narrow gaps, i.e. 29
 * elements starting and ending with a bar.  Widths are the number of 1 ms
 * samples each element lasted, saturating at 255 (Change Requirement 1:
 * 29 bytes instead of 58).  Classification is on width RATIOS within each
 * character, so the decode is independent of speed (NFR8) and tolerates a
 * speed change between characters.  Both reading directions are accepted.
 */

#ifndef BARCODE_H
#define BARCODE_H

#include <stdint.h>

#define BARCODE_MAX_ELEMENTS    29U
#define BARCODE_CHAR_ELEMENTS   9U
#define BARCODE_CHAR_COUNT      44U
#define BARCODE_INDEX_STAR      43U

/* Reject reasons (3 bits).  Zero means a valid decode. */
#define BC_REJ_NONE             0U
#define BC_REJ_COUNT            1U  /* not exactly 29 elements           */
#define BC_REJ_FRAME            2U  /* start/stop character not '*'      */
#define BC_REJ_RATIO            3U  /* an element outside 0.5x..4.5x     */
#define BC_REJ_PATTERN          4U  /* not three wide, or unknown char   */
#define BC_REJ_OVERRUN          5U  /* marks after the symbol, timeout   */

/* Character for each index; index 43 is the '*' delimiter. */
extern char const g_barcode_chars[BARCODE_CHAR_COUNT];

/*!
 * @brief Decode one delimited symbol.
 * @param[in]  p_widths  element widths, first element is a bar
 * @param[in]  count     number of elements captured
 * @param[out] p_index   index of the middle character on success
 * @return BC_REJ_NONE on success, otherwise the reject reason.
 */
uint8_t barcode_decode(uint8_t const * p_widths, uint32_t count,
                       uint8_t * p_index);

/*!
 * @brief Decode with no ratio limits: in each character the three widest
 *        elements are the wide ones.  For blurred distance widths and for a
 *        hand swipe, whose speed changes within a symbol.  The '*' framing
 *        is still required.
 * @return BC_REJ_NONE on success, otherwise a reason.
 */
uint8_t barcode_decode_loose(uint8_t const * p_widths, uint32_t count,
                             uint8_t * p_index);

/*!
 * @brief barcode_decode() after undoing a uniform thinning of every bar and
 *        thickening of every space (or the reverse).  An analog sensor that
 *        rides higher or blurs more sees bars narrower: bench 2026-11-10,
 *        every bar ~38 counts short and every space ~38 long.
 */
uint8_t barcode_decode_balanced(uint8_t const * p_widths, uint32_t count,
                                uint8_t * p_index);

/*!
 * @brief Build the element widths of "*X*" (test helper).
 * @return number of elements written (29), or 0 for a bad index.
 */
uint32_t barcode_encode(uint8_t index, uint8_t narrow, uint8_t wide,
                        uint8_t * p_widths);

#endif /* BARCODE_H */

/*** end of file ***/
