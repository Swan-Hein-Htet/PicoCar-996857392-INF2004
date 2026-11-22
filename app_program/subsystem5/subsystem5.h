/** @file subsystem5.h
 *
 * @brief Subsystem 5 -- Adaptive ultrasonic scanning, obstacle profiling,
 *        bypass and line reacquisition.  Buddy 5, Danish Hazwan, 2500803.
 *
 * Public interface of subsystem 5.  Other subsystems include it as
 * "../subsystem5/subsystem5.h".
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

/* ---------------------------------------------------------------------
 *  Robot body, PROVISIONAL (estimated from the kit photos on the 10 mm
 *  cutting-mat grid; measure B and L in the A5 bench session).
 * --------------------------------------------------------------------- */
#define ROBOT_WIDTH_MM          150U    /* B: outside of both tyres       */
#define ROBOT_LENGTH_MM         180U    /* L: IR bracket to rear          */
#define ROBOT_US_AHEAD_MM       60U     /* HC-SR04 face ahead of the axle */

/* ---------------------------------------------------------------------
 *  g_echo_word (uint16, single writer ISR_ECHO, rules R1 and R3)
 *    [14:0]  echo pulse width, us, clamped at 32767
 *    [15]    valid: cleared when the pulse reached the ranging timeout,
 *            so a no-return is never read as a short range
 * --------------------------------------------------------------------- */
#define ECHO_WIDTH_BITS         15U
#define ECHO_VALID_SHIFT        15U

IMPORT volatile uint16_t g_echo_word;

/* ---------------------------------------------------------------------
 *  Obstacle profile, two words in g_vehicle_state under mtx_state
 *  (writer T5, too large for one atomic store).
 *  profile0:
 *    [4:0]   bearing of the closest point, 10-degree index 0-18
 *    [14:5]  corrected width, mm, 0-1023
 *    [23:15] closest point, mm, 0-511
 *    [25:24] plan decision (PLAN_x)
 *    [29:26] scan state O0-O11
 *  profile1:
 *    [9:0]   left clearance,  mm, 0-1023
 *    [19:10] right clearance, mm, 0-1023
 *    [20]    retry used (O11 taken once)
 * --------------------------------------------------------------------- */
#define PROF_BEARING_SHIFT      0U
#define PROF_BEARING_BITS       5U
#define PROF_WIDTH_SHIFT        5U
#define PROF_WIDTH_BITS         10U
#define PROF_CLOSEST_SHIFT      15U
#define PROF_CLOSEST_BITS       9U
#define PROF_PLAN_SHIFT         24U
#define PROF_PLAN_BITS          2U
#define PROF_SCAN_SHIFT         26U
#define PROF_SCAN_BITS          4U
#define PROF_LEFT_SHIFT         0U
#define PROF_RIGHT_SHIFT        10U
#define PROF_CLEAR_BITS         10U
#define PROF_RETRY_SHIFT        20U

#define PLAN_CONTINUE           0U
#define PLAN_LEFT               1U
#define PLAN_RIGHT              2U
#define PLAN_REVERSE            3U

#define SCAN_WATCH              0U  /* O0  */
#define SCAN_HALT               1U  /* O1  */
#define SCAN_COARSE             2U  /* O2  */
#define SCAN_FINE               3U  /* O3  */
#define SCAN_PROFILE            4U  /* O4  */
#define SCAN_PLAN               5U  /* O5  */
#define SCAN_OUT                6U  /* O6  */
#define SCAN_ALONG              7U  /* O7  */
#define SCAN_BACK               8U  /* O8  */
#define SCAN_REACQUIRE          9U  /* O9  */
#define SCAN_BLOCKED            10U /* O10 */
#define SCAN_REVERSE            11U /* O11 */

/* Private to subsystem 5 */
#define S5_PING_PERIOD_MS       70U     /* HC-SR04: cycle > 60 ms        */
#define S5_TRIGGER_MM           200U
IMPORT ID   tid_obstacle;
IMPORT UW   g_echo_count;       /* echo ISR completions, for the tests */
IMPORT void obstacle_task(INT stacd, void * p_exinf);

#endif /* SUBSYSTEM5_H */

/*** end of file ***/
