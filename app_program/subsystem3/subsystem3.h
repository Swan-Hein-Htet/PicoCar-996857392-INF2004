/** @file subsystem3.h
 *
 * @brief Subsystem 3 -- IR line following and Code 39 barcode decoding.
 *        Buddy 3, Yun Cong, 2500226.
 *
 * Public interface of subsystem 3.  Other subsystems include it as
 * "../subsystem3/subsystem3.h".
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "barcode.h"

/* ---------------------------------------------------------------------
 *  g_line_word (uint32, single writer T2, rule R1).  Rewritten every
 *  10 ms (NFR2) and on every line-state change.
 *    [7:0]   line error, signed 8 bits, +-127 = +-1.0 (positive: line is
 *            to the LEFT of centre, steer left)
 *    [9:8]   line state L0-L3
 *    [10]    junction now (both line channels black)
 *    [11]    end marker seen (black for > LINE_END_MM)
 *    [15:12] sequence, wraps (staleness check by T4)
 *    [18:16] last barcode (BC_LAST_x)
 *    [21:19] last reject reason (BC_REJ_x)
 *    [29:22] length of the current/last black run on a line channel,
 *            mm, saturating 255 (T5 uses it to reject barcode bars)
 *    [30]    a line channel is black now
 *    [31]    barcode capture in progress (B1)
 * --------------------------------------------------------------------- */
#define LINE_ERR_SHIFT          0U
#define LINE_ERR_BITS           8U
#define LINE_STATE_SHIFT        8U
#define LINE_STATE_BITS         2U
#define LINE_JUNCTION_SHIFT     10U
#define LINE_END_SHIFT          11U
#define LINE_SEQ_SHIFT          12U
#define LINE_SEQ_BITS           4U
#define LINE_LASTBC_SHIFT       16U
#define LINE_LASTBC_BITS        3U
#define LINE_REJECT_SHIFT       19U
#define LINE_REJECT_BITS        3U
#define LINE_RUN_SHIFT          22U
#define LINE_RUN_BITS           8U
#define LINE_BLACK_SHIFT        30U
#define LINE_CAPTURE_SHIFT      31U

#define LSTATE_CALIBRATE        0U  /* L0 */
#define LSTATE_TRACK            1U  /* L1 */
#define LSTATE_JUNCTION         2U  /* L2 */
#define LSTATE_LOST             3U  /* L3 */

/* Last decoded barcode in telemetry: none, A, B, C, D, other (3 bits). */
#define BC_LAST_NONE            0U
/* Since the A-Z map (2026-11-17) these are the last barcode's COMMAND,
   = NAV_x: 1 left, 2 right, 3 straight, 4 U-turn, 5 none (unmapped). */
#define BC_LAST_A               1U
#define BC_LAST_B               2U
#define BC_LAST_C               3U
#define BC_LAST_D               4U
#define BC_LAST_OTHER           5U

IMPORT volatile uint32_t g_line_word;
/* g_cross_word (single writer T2, rule R1): the last fresh crossing of a
   cross bar by the IR row (both line sensors turned dark within
   CROSS_FRESH_MM of each other, D72).  T6 uses it when a barcode command
   arrives just after the junction it was meant for.
     [15:0]  odometer (odom_now_mm) when the IR row was on the bar
     [16]    valid */
#define CROSS_ODOM_SHIFT        0U
#define CROSS_ODOM_BITS         16U
#define CROSS_VALID_SHIFT       16U
IMPORT volatile uint32_t g_cross_word;

/* Line geometry and thresholds (mm of travel, from g_odom_word). */
#define LINE_LOST_MM            40U     /* both white this far -> L3   */
#define LINE_JUNCTION_MM        12U     /* both black this far -> L2:
                                           longer than the widest barcode
                                           bar (9 mm), shorter than the
                                           18 mm junction line            */
#define LINE_END_MM             60U     /* both black this far -> END  */
#define LINE_SENSOR_AHEAD_MM    70U     /* IR row ahead of the axle    */

/* ---------------------------------------------------------------------
 *  nav_cmd_t: one uint16_t through mbf_nav (T2 -> T6)
 *    [2:0]   command (NAV_x in app.h)
 *    [8:3]   decoded character index into barcode_chars (0-43)
 *    [11:9]  reject reason (BC_REJ_x)
 * --------------------------------------------------------------------- */
typedef uint16_t nav_cmd_t;

#define NAV_CMD_SHIFT           0U
#define NAV_CMD_BITS            3U
#define NAV_CHAR_SHIFT          3U
#define NAV_CHAR_BITS           6U
#define NAV_REJ_SHIFT           9U
#define NAV_REJ_BITS            3U

/* Private to subsystem 3 */
#define S3_T2_PERIOD_MS         1U
IMPORT ID   tid_line;
IMPORT volatile uint8_t g_barcode_last;  /* last decoded Code 39 character,
                                            any of the 43; 0 = none yet
                                            (T2 writes, one byte, rule R1) */
IMPORT UW   g_line_overruns;    /* queued wake-ups above one (NFR1) */
IMPORT void line_task(INT stacd, void * p_exinf);

#endif /* SUBSYSTEM3_H */

/*** end of file ***/
