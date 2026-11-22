/** @file task_safety.c
 *
 * @brief T1 SafetyMonitor, every 20 ms at priority 2 (above every kernel
 *        service task, tribunal R6).
 *
 *   - Any critical fault: force the motors off and assert flg_sys ESTOP
 *     (NFR5: within 50 ms; the detector has usually done it already).
 *   - Liveness: T4's heartbeat and T2's line-word sequence must keep
 *     moving; a stalled control task is a critical fault.
 *   - Hardware watchdog: fed only from here, so a starved T1 resets the
 *     chip, and reset leaves the motor pins low (brake).
 *   - Status LED on GP7: the vehicle state is visible with no network.
 */

#include "subsystem4.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"

#define LIVENESS_PERIODS        5U      /* 100 ms without a heartbeat     */
#define STARTUP_GRACE_PERIODS   50U     /* 1 s for every task to start    */

#define WDT_BASE                0x40058000U
#define WDT_CTRL                (WDT_BASE + 0x00U)
#define WDT_LOAD                (WDT_BASE + 0x04U)
#define WDT_TICK                (WDT_BASE + 0x2CU)
#define WDT_CTRL_ENABLE         0x40000000U
#define WDT_CTRL_PAUSE_DBG      0x07000000U
#define WDT_TICK_ENABLE_12MHZ   (0x200U | 12U)
#define PSM_WDSEL               0x40010008U
#define PSM_WDSEL_ALL_BUT_OSC   0x0001FFFCU
#define WDT_LOAD_500MS          1000000U    /* RP2040-E1: counts by 2 */
#define WDT_REASON              (WDT_BASE + 0x08U)
#define WDT_REASON_TIMER        0x1U
#define WDT_REASON_FORCE        0x2U
#define CHIP_RESET              0x40064008U /* VREG_AND_CHIP_RESET      */
#define CHIP_HAD_POR            0x00000100U /* power-on or brown-out    */
#define CHIP_HAD_RUN            0x00010000U /* RUN pin                  */
#define CHIP_HAD_PSM            0x00100000U /* debugger                 */

#ifndef APP_WATCHDOG
#define APP_WATCHDOG            (!APP_TEST)
#endif

typedef struct
{
    uint32_t beat_prev;
    uint32_t seq_prev;
    uint32_t beat_stale;
    uint32_t seq_stale;
    uint32_t periods;
} safety_ctx_t;

static safety_ctx_t g_safety;

static void
watchdog_start (void)
{
#if APP_WATCHDOG
    out_w(WDT_TICK, WDT_TICK_ENABLE_12MHZ);
    out_w(PSM_WDSEL, PSM_WDSEL_ALL_BUT_OSC);
    out_w(WDT_LOAD, WDT_LOAD_500MS);
    out_w(WDT_CTRL, WDT_CTRL_ENABLE | WDT_CTRL_PAUSE_DBG);
#endif
}

static void
watchdog_feed (void)
{
#if APP_WATCHDOG
    out_w(WDT_LOAD, WDT_LOAD_500MS);
#endif
}

EXPORT void safety_led_init (void)
{
    (void)gpio_set_pin(PIN_STATUS_LED, GPIO_MODE_OUT);
    (void)gpio_set_val(PIN_STATUS_LED, 0U);
}

/* LED pattern from the mission state: fault 5 Hz, waiting 1 Hz,
   running solid, finished off. */
static void
led_update (uint32_t periods)
{
    uint32_t const state = mission_state();
    uint32_t       level;

    if (fault_is_critical() || (MISSION_FAULT == state))
    {
        level = ((periods / 5U) % 2U);
    }
    else if (MISSION_FINISHED == state)
    {
        level = 0U;
    }
    else if (MISSION_READY >= state)
    {
        level = ((periods / 25U) % 2U);
    }
    else
    {
        level = 1U;
    }
    (void)gpio_set_val(PIN_STATUS_LED, level);
}

static void
liveness_check (void)
{
    uint32_t const beat = bits_get(g_motion_word, MOTION_BEAT_SHIFT,
                                   MOTION_BEAT_BITS);
    uint32_t const seq = bits_get(g_line_word, LINE_SEQ_SHIFT,
                                  LINE_SEQ_BITS);

    g_safety.beat_stale = (beat == g_safety.beat_prev) ?
                          (g_safety.beat_stale + 1U) : 0U;
    g_safety.seq_stale = (seq == g_safety.seq_prev) ?
                         (g_safety.seq_stale + 1U) : 0U;
    g_safety.beat_prev = beat;
    g_safety.seq_prev = seq;

    if ((g_safety.periods > STARTUP_GRACE_PERIODS) &&
        ((g_safety.beat_stale >= LIVENESS_PERIODS) ||
         (g_safety.seq_stale >= LIVENESS_PERIODS)))
    {
        if (0U == ((uint32_t)fault_get() & FAULT_TASK_OVERRUN))
        {
            tm_printf((UB *)"T1: control task stalled (T4 %d, T2 %d)\n",
                      (INT)g_safety.beat_stale, (INT)g_safety.seq_stale);
        }
        fault_set(FAULT_TASK_OVERRUN);
    }
}

/* Why the chip last started (D74): a reboot mid-run showed up only as a
   false IMPACT (real car 2026-11-19 18:41, during a junction turn).
   Read before the watchdog is armed. */
static void
reset_reason (void)
{
    uint32_t const wdt = in_w(WDT_REASON);
    uint32_t const chip = in_w(CHIP_RESET);
    char const *   why = "unknown";

    if (0U != (wdt & WDT_REASON_TIMER))
    {
        why = "WATCHDOG (a task stalled for 500 ms)";
    }
    else if (0U != (wdt & WDT_REASON_FORCE))
    {
        why = "forced by software";
    }
    else if (0U != (chip & CHIP_HAD_POR))
    {
        why = "power-on or BROWN-OUT (supply dipped: battery, motors)";
    }
    else if (0U != (chip & CHIP_HAD_RUN))
    {
        why = "RUN pin / reset button";
    }
    else if (0U != (chip & CHIP_HAD_PSM))
    {
        why = "debugger";
    }
    else
    {
        /* Nothing recorded. */
    }
    tm_printf((UB *)"T1: started after %s (watchdog 0x%x, chip 0x%x)\n",
              (UB *)why, (INT)wdt, (INT)chip);
}

EXPORT void safety_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    reset_reason();
    watchdog_start();

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);     /* released by the 20 ms waker */
        (void)tk_can_wup(TSK_SELF);
        g_safety.periods++;

        if (fault_is_critical())
        {
            motion_force_off();
            (void)tk_set_flg(flg_sys, FLG_ESTOP);
        }

        liveness_check();
        led_update(g_safety.periods);
        watchdog_feed();
    }
}

/*** end of file ***/
