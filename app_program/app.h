/** @file app.h
 *
 * @brief Definitions shared by the whole team.  Changed only by team
 *        agreement (TEAM.md).
 *
 * Anything that belongs to ONE subsystem goes in that subsystem's own
 * header (app_program/subsystemN/subsystemN.h).
 */

#ifndef APP_H
#define APP_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#define PROHIBIT_DEF_SIZE_T     /* size_t comes from <stddef.h> */
#include <tk/tkernel.h>     /* micro T-Kernel API (tk_xxx_yyy) */
#include <tm/tmonitor.h>    /* T-Monitor API: tm_printf() console output */
#include <bsp/libbsp.h>     /* Board helpers: gpio_set_pin/val, pwm_... */
#include "app_bits.h"

/* ---------------------------------------------------------------------
 *  Pin map.  FIXED by docs/WIRING.md -- do not change without changing
 *  the wiring document first.  Every pin is also claimed in TEAM.md.
 * --------------------------------------------------------------------- */
#define PIN_US_TRIG         2U   /* HC-SR04 TRIG (Grove 2)                 */
#define PIN_US_ECHO         3U   /* HC-SR04 ECHO via 10k/15k divider       */
#define PIN_IMU_SDA         4U   /* I2C0 SDA, GY-511 (Grove 3)             */
#define PIN_IMU_SCL         5U   /* I2C0 SCL, GY-511 (Grove 3)             */
#define PIN_STATUS_LED      7U   /* carrier status LED (Grove 7 GP7 open)  */
#define PIN_M1A             8U   /* left motor  M1A, PWM slice 4 A         */
#define PIN_M1B             9U   /* left motor  M1B, PWM slice 4 B         */
#define PIN_M2A             10U  /* right motor M2A, PWM slice 5 A         */
#define PIN_M2B             11U  /* right motor M2B, PWM slice 5 B         */
#define PIN_SERVO           12U  /* SG90 scan servo, servo port 1, slice 6 */
#define PIN_ENC_L_B         13U  /* left encoder phase B (GPIO header)     */
#define PIN_ENC_L_A         16U  /* left encoder phase A (Grove 4)         */
#define PIN_ENC_R_A         17U  /* right encoder phase A (Grove 4)        */
#define PIN_ENC_R_B         19U  /* right encoder phase B (GPIO header)    */
#define PIN_BTN_START       20U  /* onboard button, active low             */
#define PIN_BTN_ESTOP       21U  /* onboard button, active low             */
#define PIN_IR_RIGHT        26U  /* ADC0, right line sensor A0 (Grove 5)   */
#define PIN_IR_LEFT         27U  /* ADC1, left line sensor A0 (Grove 6)    */
#define PIN_IR_BARCODE      28U  /* barcode sensor D0, digital (Grove 7)   */

/* ---------------------------------------------------------------------
 *  Cores.  Both kernel service tasks (USB console pri 3, CYW43 pri 4) are
 *  pinned to core 0.  Every application task is pinned to core 1 so that
 *  no service task can pre-empt a control loop (tribunal risk R6).  On a
 *  single-core build (SMP=0) the affinity is ignored.
 * --------------------------------------------------------------------- */
#if TK_SUPPORT_SMP
#define APP_CORE            TP_PRC2
#else
#define APP_CORE            0U
#endif

/* ---------------------------------------------------------------------
 *  Task priorities (1 is highest; the initial task is 1, the USB console
 *  task 3 and the WiFi task 4).  Rate-monotonic order with one safety
 *  exception: T1 sits strictly above every kernel service task.
 * --------------------------------------------------------------------- */
#define PRI_T1_SAFETY       2
#define PRI_T2_LINE         6
#define PRI_T3_IMU          7
#define PRI_T4_MOTION       8
#define PRI_T5_OBSTACLE     10
#define PRI_T6_VEHICLE      12
#define PRI_T7_TELEMETRY    20

#define TASK_STACK_SIZE     2048    /* bytes; tm_printf needs ~1 KB */

/* ---------------------------------------------------------------------
 *  flg_sys: the one system event flag (created by subsystem 1).
 *  Every bit has exactly ONE clearer, listed on the right.
 * --------------------------------------------------------------------- */
#define FLG_ESTOP           0x00000001U /* set: faults.c   clear: T6 (RESET) */
#define FLG_LINE_LOST       0x00000002U /* set: T2         clear: T2         */
#define FLG_OBSTACLE        0x00000004U /* set: T5         clear: T6         */
#define FLG_HUMP_START      0x00000008U /* set: T3         clear: T6         */
#define FLG_NET_UP          0x00000010U /* set: T7         clear: T7         */
#define FLG_START           0x00000020U /* set: ISR_BTN,T7 clear: T6         */
#define FLG_MOTION_DONE     0x00000040U /* set: T4  clear: requester (T6/T5) */
#define FLG_RESET_REQ       0x00000080U /* set: T7         clear: T6         */
#define FLG_NAV             0x00000100U /* set: T2         clear: T6         */
#define FLG_JUNCTION        0x00000200U /* set: T2         clear: T6         */
#define FLG_END             0x00000400U /* set: T2, T7     clear: T6         */
#define FLG_LINE_REACQ      0x00000800U /* set: T5         clear: T6         */
#define FLG_CAL_DONE        0x00001000U /* set: T2         clear: T6         */
#define FLG_HUMP_END        0x00002000U /* set: T3         clear: T6         */
#define FLG_BARCODE_START   0x00004000U /* set: T2         clear: T6         */
#define FLG_LINE_FOUND      0x00008000U /* set: T2         clear: T6         */
#define FLG_CAL_REQ         0x00010000U /* set: T6         clear: T2         */
#define FLG_CAL_STOP        0x00020000U /* set: T6         clear: T2         */

/* ---------------------------------------------------------------------
 *  Mission states (S0-S10, 4 bits).  Owned by T6, published in
 *  g_mission_word (subsystem1.h).
 * --------------------------------------------------------------------- */
#define MISSION_INIT        0U  /* S0  power-on self-test                */
#define MISSION_CALIBRATE   1U  /* S1  IR calibration sweep              */
#define MISSION_READY       2U  /* S2  armed, waiting for START          */
#define MISSION_FOLLOW      3U  /* S3  line following                    */
#define MISSION_BARCODE     4U  /* S4  barcode capture, speed clamp      */
#define MISSION_MANOEUVRE   5U  /* S5  executing a navigation command    */
#define MISSION_HUMP        6U  /* S6  on a hump, speed clamp            */
#define MISSION_OBSTACLE    7U  /* S7  T5 owns motion (bypass)           */
#define MISSION_SEARCH      8U  /* S8  line search                       */
#define MISSION_FINISHED    9U  /* S9  end of course                     */
#define MISSION_FAULT       10U /* S10 safe state                        */

/* Navigation commands decoded from barcodes (3 bits). */
#define NAV_NONE            0U
#define NAV_LEFT            1U  /* turn left at the next junction  */
#define NAV_RIGHT           2U  /* turn right at the next junction */
#define NAV_STRAIGHT        3U  /* straight on at the next junction */
#define NAV_UTURN           4U  /* U-turn now                      */
#define NAV_REJECT          5U  /* unreadable symbol, nothing executed */
#define NAV_OTHER           6U  /* valid Code 39 character, no command */

/* Barcode letter -> command, set before a run over MQTT (p5is20/car/map,
   docs/MQTT.md).  One NAV_x per letter A-Z, NAV_NONE = unmapped.  Single
   writer T6 (only while parked), read by T2 when a symbol decodes.
   Default A left, B right, C straight, D U-turn, the rest unmapped. */
#define BC_MAP_LETTERS      26U
IMPORT volatile uint8_t g_bc_map[BC_MAP_LETTERS];

/* Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
   0 for "any core".  Defined in app_main.c. */
IMPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name);

/* Create a cyclic handler that wakes 'tid' every 'period_ms'.  The handler
   does nothing but tk_wup_tsk(), giving the task a drift-free, absolute-
   time release (tribunal decision R1).  Defined in app_main.c. */
IMPORT ID app_start_waker(ID *p_tid, UW period_ms);

/* Microsecond free-running timer (RP2040 TIMER, low word). */
IMPORT uint32_t app_micros(void);

/* ---------------------------------------------------------------------
 *  Every subsystem provides these.  usermain() calls ALL the _init()
 *  functions first and only then ALL the _start() functions, so every
 *  kernel object exists before any task can use it.
 * --------------------------------------------------------------------- */
IMPORT ER subsystem1_init(void);
IMPORT ER subsystem2_init(void);
IMPORT ER subsystem3_init(void);
IMPORT ER subsystem4_init(void);
IMPORT ER subsystem5_init(void);

IMPORT ER subsystem1_start(void);
IMPORT ER subsystem2_start(void);
IMPORT ER subsystem3_start(void);
IMPORT ER subsystem4_start(void);
IMPORT ER subsystem5_start(void);

#if APP_TEST
IMPORT void subsystem1_test(void);
IMPORT void subsystem2_test(void);
IMPORT void subsystem3_test(void);
IMPORT void subsystem4_test(void);
IMPORT void subsystem5_test(void);
#endif

/* Print a readable message if a kernel call failed (negative error code) */
#define CHECK(er, what)                                                     \
    do {                                                                    \
        INT check_er_ = (INT)(er);  /* evaluate once */                     \
        if (check_er_ < E_OK)                                               \
        {                                                                   \
            tm_printf((UB *)"ERROR %s: %d\n", (what), check_er_);           \
        }                                                                   \
    } while (0)

#endif /* APP_H */

/*** end of file ***/
