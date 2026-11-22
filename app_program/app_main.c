/** @file app_main.c
 *
 * @brief START HERE.  usermain() is the program's entry point.
 *
 * reset -> kernel initialises itself -> initial task -> usermain()
 *
 * usermain() runs in two phases: every subsystem creates its kernel objects
 * (subsystemN_init), then every subsystem starts its tasks
 * (subsystemN_start).  It then sleeps forever; returning would shut the
 * kernel down.
 *
 * Subsystems (one folder per buddy, see TEAM.md):
 *   subsystem1  Buddy 1  T6 VehicleCtrl (mission), T7 Telemetry, MQTT bridge
 *   subsystem2  Buddy 2  T4 MotionCtrl, motors, encoders, GPIO dispatcher
 *   subsystem3  Buddy 3  T2 LineBarcode, IR ADC, Code 39 decoder
 *   subsystem4  Buddy 4  T1 SafetyMonitor, T3 ImuTerrain, fault mask
 *   subsystem5  Buddy 5  T5 ObstacleScan, HC-SR04, servo
 */

#include "app.h"
#include "subsystem1/subsystem1.h"
#include "subsystem2/subsystem2.h"
#include "subsystem3/subsystem3.h"
#include "subsystem4/subsystem4.h"
#include "subsystem5/subsystem5.h"

#define TIMER_RAW_LOW   (0x40054028U)   /* RP2040 TIMER.TIMERAWL */
#define WDT_TICK        (0x4005802CU)   /* watchdog tick: drives TIMER  */
#define WDT_TICK_1US    (0x200U | 12U)  /* enable, 12 MHz clk_ref / 12  */

/*!
 * @brief Cyclic handler body shared by every periodic task: wake the task
 *        whose ID 'exinf' points at.  Interrupt context, so nothing else.
 */
static void app_waker (void * p_exinf)
{
    (void)tk_wup_tsk(*(ID *)p_exinf);
}

EXPORT uint32_t app_micros (void)
{
    return ((uint32_t)in_w(TIMER_RAW_LOW));
}

EXPORT ID app_start_task (FP entry, PRI pri, UW prc, const char * name)
{
    T_CTSK ctsk = {0};
    ID     tid;
    ER     er;

    ctsk.task    = entry;
    ctsk.itskpri = pri;
    ctsk.stksz   = TASK_STACK_SIZE;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
    if (0U != prc)
    {
        ctsk.tskatr |= TA_ASSPRC;
        ctsk.assprc  = prc;
    }
#else
    (void)prc;
#endif

    tid = tk_cre_tsk(&ctsk);
    if (tid < E_OK)
    {
        CHECK(tid, name);
        return (tid);
    }

    er = tk_sta_tsk(tid, 0);
    CHECK(er, name);
    return (tid);
}

EXPORT ID app_start_waker (ID * p_tid, UW period_ms)
{
    T_CCYC ccyc = {0};

    ccyc.cycatr = TA_HLNG | TA_STA;
    ccyc.cychdr = (FP)app_waker;
    ccyc.exinf  = (void *)p_tid;
    ccyc.cyctim = (RELTIM)period_ms;
    ccyc.cycphs = (RELTIM)period_ms;

    return (tk_cre_cyc(&ccyc));
}

/*!
 * @brief Change Requirement 1 evidence: print the size of every shared
 *        word and every message type at boot (compare with tag
 *        cr1-baseline, docs/DATA_REPRESENTATION.md).
 */
static void app_report_sizes (void)
{
    tm_printf((UB *)"[CR1] motion_req=%d nav_cmd=%d telemetry=%d cmd=%d\n",
              (INT)sizeof(motion_req_t), (INT)sizeof(nav_cmd_t),
              (INT)TELEMETRY_BYTES, (INT)COMMAND_BYTES);
    tm_printf((UB *)"[CR1] line=%d motion=%d odom=%d terrain=%d hump=%d"
              " echo=%d mission=%d faults=%d enc=%d+%d profile=%d"
              " barcode_buf=%d\n",
              (INT)sizeof(g_line_word), (INT)sizeof(g_motion_word),
              (INT)sizeof(g_odom_word), (INT)sizeof(g_terrain_word),
              (INT)sizeof(g_hump_byte), (INT)sizeof(g_echo_word),
              (INT)sizeof(g_mission_word), (INT)sizeof(g_faults),
              (INT)sizeof(g_enc_left), (INT)sizeof(g_enc_right),
              (INT)sizeof(g_vehicle_state), (INT)BARCODE_MAX_ELEMENTS);
}

#if APP_SERVO_TEST
#include "subsystem2/drv_motor.h"
#include "subsystem5/drv_servo.h"

/*!
 * @brief Servo bench check (make SERVO_TEST=1).  Every 2 s: 1000 us
 *        (right end), 1500 us (centre), 2000 us (left end), no pulses.
 *        The GP7 LED blinks 1, 2, 3 or 4 times for the current step.
 *        A normal 180-degree SG90 moves to each position and HOLDS it,
 *        and goes limp with no pulses.  A 360-degree "continuous" servo
 *        instead SPINS one way, nearly stops, spins the other way, stops.
 */
static void servo_bench (void)
{
    static uint32_t const angle[3] = { 0U, 90U, 180U };
    uint32_t step = 0U;

    motor_init();                       /* wheels held off */
    safety_led_init();
    servo_init();
    tm_printf((UB *)"SERVO TEST: right, centre, left, off; 2 s each\n");

    for (;;)
    {
        uint32_t blink;

        if (step < 3U)
        {
            (void)servo_move(angle[step]);
            tm_printf((UB *)"SERVO TEST: %d us\n", (INT)(1000U + (500U * step)));
        }
        else
        {
            (void)pwm_set_cc(PIN_SERVO, 0U);    /* line held low */
            tm_printf((UB *)"SERVO TEST: no pulses\n");
        }
        for (blink = 0U; blink <= step; blink++)
        {
            (void)gpio_set_val(PIN_STATUS_LED, 1U);
            (void)tk_dly_tsk(150);
            (void)gpio_set_val(PIN_STATUS_LED, 0U);
            (void)tk_dly_tsk(150);
        }
        (void)tk_dly_tsk((RELTIM)(2000U - (300U * (step + 1U))));
        step = (step + 1U) % 4U;
    }
}
#endif

#if APP_WHEEL_TEST
#include "subsystem2/drv_motor.h"
#include "subsystem2/drv_encoder.h"

/*!
 * @brief Motor and encoder bench check (make WHEEL_TEST=1), wheels OFF
 *        the ground.  Repeats: left forward, left backward, right
 *        forward, right backward, 2 s each at 35 % duty with 1 s stops,
 *        logging how far each encoder counted.  "Forward" is what the
 *        firmware believes is forward after MOTION_MOTOR_SIGN_x; a wheel
 *        that turns the other way, or an encoder that counts down while
 *        its wheel turns forward, shows up here.  GP7 blinks the step.
 */
/* 30 s with the motors off: mark a wheel, turn it exactly ONE full turn
   forward by hand, and read the change in its count = edges per wheel
   turn (MOTION_EDGES_PER_REV, firmware assumes 1056). */
static void hand_count (void)
{
    uint16_t const l0 = g_enc_left;
    uint16_t const r0 = g_enc_right;
    int32_t        last_l = 0;
    int32_t        last_r = 0;
    uint32_t       n;

    motor_write(0.0f, 0.0f);
    motor_arm(false);
    tm_printf((UB *)"WHEEL TEST: HAND COUNT for 30 s: turn ONE wheel exactly"
              " one full turn forward by hand (firmware assumes %d)\n",
              (INT)MOTION_EDGES_PER_REV);
    for (n = 0U; n < 60U; n++)
    {
        int32_t const l = (int32_t)(int16_t)(uint16_t)(g_enc_left - l0);
        int32_t const r = (int32_t)(int16_t)(uint16_t)(g_enc_right - r0);

        if ((l != last_l) || (r != last_r))
        {
            tm_printf((UB *)"WHEEL TEST: hand count left %d right %d\n",
                      (INT)l, (INT)r);
            last_l = l;
            last_r = r;
        }
        (void)gpio_set_val(PIN_STATUS_LED, n & 1U);
        (void)tk_dly_tsk(500);
    }
    tm_printf((UB *)"WHEEL TEST: hand count final left %d right %d\n",
              (INT)last_l, (INT)last_r);
}

static void wheel_bench (INT stacd, void * p_exinf)
{
    static char const * const name[4] = { "LEFT forward", "LEFT backward",
                                          "RIGHT forward", "RIGHT backward" };
    static float const duty_l[4] = { 0.35f, -0.35f, 0.0f, 0.0f };
    static float const duty_r[4] = { 0.0f, 0.0f, 0.35f, -0.35f };
    uint32_t step = 0U;

    (void)stacd;
    (void)p_exinf;
    /* Runs as a task on APP_CORE, exactly like T4, so the GPIO interrupt
       is taken on the control core and the network core is left alone
       (run from the initial task, on the network core, the car joined
       the hotspot but never reached the broker). */
    (void)gpio_irq_start();
    (void)tk_dly_tsk(8000);             /* time for WiFi/MQTT to connect */
    tm_printf((UB *)"WHEEL TEST: wheels off the ground. Expect + counts"
              " forward, - counts backward, 0 on the idle wheel.\n");

    for (;;)
    {
        uint16_t const l0 = g_enc_left;
        uint16_t const r0 = g_enc_right;
        uint32_t       blink;
        int32_t        dl;
        int32_t        dr;

        for (blink = 0U; blink <= step; blink++)
        {
            (void)gpio_set_val(PIN_STATUS_LED, 1U);
            (void)tk_dly_tsk(120);
            (void)gpio_set_val(PIN_STATUS_LED, 0U);
            (void)tk_dly_tsk(120);
        }
        motor_arm(true);
        motor_write(duty_l[step], duty_r[step]);
        (void)tk_dly_tsk(2000);
        motor_write(0.0f, 0.0f);
        (void)tk_dly_tsk(300);          /* count the coast too */
        dl = (int32_t)(int16_t)(uint16_t)(g_enc_left - l0);
        dr = (int32_t)(int16_t)(uint16_t)(g_enc_right - r0);
        tm_printf((UB *)"WHEEL TEST: %s: left encoder %d, right encoder %d"
                  "\n", name[step], (INT)dl, (INT)dr);
        (void)tk_dly_tsk(700);
        step = (step + 1U) % 4U;
        if (0U == step)
        {
            hand_count();
            tm_printf((UB *)"WHEEL TEST: ---- repeat ----\n");
        }
    }
}
#endif

EXPORT INT usermain (void)
{
    T_RVER ver;

    /* The RP2040 TIMER (app_micros: ADC and I2C timeouts, echo widths,
       log time stamps) counts the watchdog tick, which only T1's
       watchdog_start() used to set up: images without it (TEST=1, the
       bench tests) ran the timer ~40x slow (bench 2026-11-02). */
    out_w(WDT_TICK, WDT_TICK_1US);

#if APP_WHEEL_TEST
    motor_init();
    safety_led_init();
    (void)encoder_init();
    (void)app_start_task((FP)wheel_bench, PRI_T4_MOTION, APP_CORE,
                         "wheel_test");
    (void)tk_slp_tsk(TMO_FEVR);
#endif

#if APP_SERVO_TEST
    servo_bench();
#endif

    (void)tk_ref_ver(&ver);
    tm_printf((UB *)"\n=== P5-IS20 PicoCar on micro T-Kernel"
              " (spec 0x%04x) ===\n", ver.spver);
#if TK_SUPPORT_SMP
    tm_printf((UB *)"dual-core build: services core 0, control core 1\n");
#else
    tm_printf((UB *)"single-core build\n");
#endif
    app_report_sizes();

    /* Phase 1: every subsystem creates its kernel objects.  Subsystem 4
       first: the fault mask must exist before any detector can run. */
    CHECK(subsystem4_init(), "subsystem4_init");
    CHECK(subsystem1_init(), "subsystem1_init");
    CHECK(subsystem2_init(), "subsystem2_init");
    CHECK(subsystem3_init(), "subsystem3_init");
    CHECK(subsystem5_init(), "subsystem5_init");

    /* Phase 2: start the tasks.  Safety (T1) and motion (T4, which owns
       the GPIO dispatcher and forces the motors off at start) first. */
    CHECK(subsystem4_start(), "subsystem4_start");
    CHECK(subsystem2_start(), "subsystem2_start");
    CHECK(subsystem3_start(), "subsystem3_start");
    CHECK(subsystem5_start(), "subsystem5_start");
    CHECK(subsystem1_start(), "subsystem1_start");

    tm_printf((UB *)"all subsystems started\n\n");

#if APP_TEST
    tm_printf((UB *)"[TEST] === test run start ===\n");
    subsystem1_test();
    subsystem2_test();
    subsystem3_test();
    subsystem4_test();
    subsystem5_test();
    tm_printf((UB *)"[TEST] === test run end ===\n");
#endif

    (void)tk_slp_tsk(TMO_FEVR);
    return (0);
}

/*** end of file ***/
