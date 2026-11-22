/** @file subsystem4.h
 *
 * @brief Subsystem 4 -- IMU terrain monitoring, safety monitor and the
 *        shared fault mask.  Buddy 4, Toh You Yang, 2500136.
 *
 * Public interface of subsystem 4.  Other subsystems include it as
 * "../subsystem4/subsystem4.h".
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

/* ---------------------------------------------------------------------
 *  Fault mask (one bit per fault, 16-bit, docs/FAULTS.md).  Many writers
 *  including ISRs, so it is changed ONLY through fault_set()/fault_clear()
 *  (packed access rule R2).  Readers use fault_get() (one halfword load).
 * --------------------------------------------------------------------- */
#define FAULT_ESTOP_REQ     0x0001U /* GP21 button                         */
#define FAULT_MOTOR_STALL   0x0002U /* T4: high duty, no speed, 500 ms     */
#define FAULT_ENCODER       0x0004U /* T4: one wheel's counter frozen      */
#define FAULT_IMU           0x0008U /* T3: init/calibration/3 I2C errors   */
#define FAULT_IMPACT        0x0010U /* T3: |a| > 3 g, two samples          */
#define FAULT_NO_ECHO       0x0020U /* T5: no echo edge for 5 pings        */
#define FAULT_BYPASS_FAIL   0x0040U /* T5: second blocked attempt          */
#define FAULT_WIFI_LOSS     0x0080U /* T7: MQTT link down                  */
#define FAULT_LINE_NOT_FOUND 0x0100U /* T6: line search timed out          */
#define FAULT_SELF_TEST     0x0200U /* T2/T6: POST or calibration failed   */
#define FAULT_REMOTE_STOP   0x0400U /* T7: MQTT STOP (source marker)       */
#define FAULT_TASK_OVERRUN  0x0800U /* T1: control task heartbeat stalled  */
#define FAULT_LIFTED        0x1000U /* T3: car picked up (end of the demo)  */

/* Losing the network, or the IMU, must not stop the car (NFR19, and hump
   reporting is the only consumer of the IMU). */
#define FAULT_CRITICAL_MASK (0x1FFFU & ~(FAULT_WIFI_LOSS | FAULT_IMU))

/* Cleared by an accepted RESET.  The others clear only when their
   detector sees the condition gone. */
#define FAULT_RESETTABLE_MASK (FAULT_ESTOP_REQ | FAULT_MOTOR_STALL |       \
                               FAULT_ENCODER | FAULT_IMPACT |              \
                               FAULT_BYPASS_FAIL | FAULT_LINE_NOT_FOUND |  \
                               FAULT_REMOTE_STOP | FAULT_TASK_OVERRUN |    \
                               FAULT_SELF_TEST | FAULT_LIFTED)

IMPORT volatile uint16_t g_faults;

IMPORT void     fault_set(uint16_t mask);
IMPORT void     fault_clear(uint16_t mask);
IMPORT uint16_t fault_get(void);
IMPORT bool     fault_is_critical(void);

/* ---------------------------------------------------------------------
 *  g_terrain_word (uint32, single writer T3, rule R1)
 *    [9:0]   pitch, 0.1 deg, signed 10 bits (+-45.0 deg)
 *    [19:10] turn rate, deg/s, signed 10 bits (+-500)
 *    [22:20] motion class (MCLASS_x)
 *    [25:23] IMU state (I0-I7)
 *    [26]    hump in progress
 *  g_hump_byte (uint8, single writer T3)
 *    [6:0]   maximum hump peak this run, whole mm (0-127)
 *    [7]     at least one hump completed
 * --------------------------------------------------------------------- */
#define TERRAIN_PITCH_SHIFT     0U
#define TERRAIN_PITCH_BITS      10U
#define TERRAIN_RATE_SHIFT      10U
#define TERRAIN_RATE_BITS       10U
#define TERRAIN_CLASS_SHIFT     20U
#define TERRAIN_CLASS_BITS      3U
#define TERRAIN_IMU_SHIFT       23U
#define TERRAIN_IMU_BITS        3U
#define TERRAIN_HUMP_SHIFT      26U

#define HUMP_PEAK_BITS          7U
#define HUMP_SEEN_SHIFT         7U

#define MCLASS_STATIONARY       0U
#define MCLASS_DRIVING          1U
#define MCLASS_ACCELERATING     2U
#define MCLASS_TURNING          3U
#define MCLASS_CLIMBING         4U
#define MCLASS_DESCENDING       5U
#define MCLASS_IMPACT           6U

#define IMU_INIT                0U  /* I0 */
#define IMU_BIAS                1U  /* I1 */
#define IMU_FLAT                2U  /* I2 */
#define IMU_ASCEND              3U  /* I3 */
#define IMU_CREST               4U  /* I4 */
#define IMU_DESCEND             5U  /* I5 */
#define IMU_REPORT              6U  /* I6 */
#define IMU_IMPACT              7U  /* I7 */

IMPORT volatile uint32_t g_terrain_word;
IMPORT volatile uint8_t  g_hump_byte;

/* ---------------------------------------------------------------------
 *  g_tilt_word (uint32, single writer T3, rule R1), for the tilt and hump
 *  meters (telemetry v2):
 *    [7:0]   roll, whole deg, signed 8 bits (+ = left side up)
 *    [13:8]  height above flat ground now, whole mm (0 when not on a hump)
 * --------------------------------------------------------------------- */
#define TILT_ROLL_SHIFT         0U
#define TILT_ROLL_BITS          8U
#define TILT_HNOW_SHIFT         8U
#define TILT_HNOW_BITS          6U
IMPORT volatile uint32_t g_tilt_word;

/* Priorities and periods */
#define S4_T1_PERIOD_MS         20U
#define S4_T3_PERIOD_MS         10U

/* Private to subsystem 4 */
IMPORT ID   tid_safety;
IMPORT ID   tid_imu;
IMPORT void safety_task(INT stacd, void * p_exinf);
IMPORT void imu_task(INT stacd, void * p_exinf);
IMPORT void safety_led_init(void);

#endif /* SUBSYSTEM4_H */

/*** end of file ***/
