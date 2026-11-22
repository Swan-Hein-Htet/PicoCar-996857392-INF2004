/** @file subsystem1.c
 *
 * @brief Subsystem 1: creates flg_sys, mbf_nav, mtx_state and the two
 *        network queues, registers ISR_BTN, and starts T6 and T7.
 */

#include "subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"

#define NAV_QUEUE_DEPTH         4U
#define TELEM_QUEUE_DEPTH       2U
#define CMD_QUEUE_DEPTH         8U
#define CMD_MAX_BYTES           4U      /* longer is rejected by T7 */
#define MBF_HEADER_BYTES        4U
#define MBF_ROUND(sz)           (((sz) + 3U) & ~3U)
#define DEBOUNCE_US             50000U
#define PAD_IE_PUE_SCHMITT      0x4AU

EXPORT ID              flg_sys = 0;
EXPORT ID              mbf_nav = 0;
EXPORT ID              mtx_state = 0;
EXPORT ID              mbf_telem = 0;
EXPORT ID              mbf_cmd = 0;
EXPORT ID              tid_vehicle = 0;
EXPORT ID              tid_telemetry = 0;
EXPORT vehicle_state_t g_vehicle_state = {0U, 0U};

static uint32_t g_btn_last_us = 0U;     /* ISR-private */

_Static_assert(2U == sizeof(g_mission_word), "mission word is a halfword");

/* ISR_BTN: both onboard buttons, falling edge, 50 ms debounce.
   E-STOP acts at once from here (motors off inside fault_set()). */
static void
button_isr (uint32_t pin, uint32_t events, uint32_t now_us)
{
    (void)events;

    if ((now_us - g_btn_last_us) < DEBOUNCE_US)
    {
        return;
    }
    g_btn_last_us = now_us;

    if (PIN_BTN_ESTOP == pin)
    {
        fault_set(FAULT_ESTOP_REQ);
    }
    else
    {
        (void)tk_set_flg(flg_sys, FLG_START);   /* also RESET in S10 */
    }
}

static ID
create_mbf (UINT depth, UINT msg_size)
{
    T_CMBF cmbf = {0};

    cmbf.mbfatr = TA_TFIFO;
    /* The kernel stores each message as a 4-byte header plus the body
       rounded up to 4 bytes (messagebuf.c ROUNDSZ); without the rounding
       a "4-deep" 2-byte queue held 3 and the 2-deep telemetry queue 1. */
    cmbf.bufsz  = (SZ)(depth * (MBF_ROUND(msg_size) + MBF_HEADER_BYTES));
    cmbf.maxmsz = (INT)msg_size;
    return (tk_cre_mbf(&cmbf));
}

EXPORT ER subsystem1_init (void)
{
    T_CFLG cflg = {0};
    T_CMTX cmtx = {0};
    ER     er;

    cflg.flgatr  = TA_TFIFO | TA_WMUL;  /* T6, T5 and T2 all wait on it */
    cflg.iflgptn = 0U;
    flg_sys = tk_cre_flg(&cflg);
    if (flg_sys < E_OK)
    {
        return (flg_sys);
    }

    cmtx.mtxatr = TA_INHERIT;           /* priority inheritance */
    mtx_state = tk_cre_mtx(&cmtx);
    if (mtx_state < E_OK)
    {
        return (mtx_state);
    }

    mbf_nav = create_mbf(NAV_QUEUE_DEPTH, sizeof(nav_cmd_t));
    if (mbf_nav < E_OK)
    {
        return (mbf_nav);
    }
    mbf_telem = create_mbf(TELEM_QUEUE_DEPTH, TELEMETRY_BYTES);
    if (mbf_telem < E_OK)
    {
        return (mbf_telem);
    }
    mbf_cmd = create_mbf(CMD_QUEUE_DEPTH, CMD_MAX_BYTES);
    if (mbf_cmd < E_OK)
    {
        return (mbf_cmd);
    }

    (void)gpio_set_pin(PIN_BTN_START, GPIO_MODE_IN);
    (void)gpio_set_pin(PIN_BTN_ESTOP, GPIO_MODE_IN);
    out_w(GPIO(PIN_BTN_START), PAD_IE_PUE_SCHMITT);
    out_w(GPIO(PIN_BTN_ESTOP), PAD_IE_PUE_SCHMITT);

    er = gpio_irq_register(PIN_BTN_START, GPIO_EDGE_FALL, button_isr);
    if (E_OK == er)
    {
        er = gpio_irq_register(PIN_BTN_ESTOP, GPIO_EDGE_FALL, button_isr);
    }
    return (er);
}

EXPORT ER subsystem1_start (void)
{
    tid_vehicle = app_start_task(vehicle_task, PRI_T6_VEHICLE, APP_CORE,
                                 "T6_vehicle");
    if (tid_vehicle < E_OK)
    {
        return (tid_vehicle);
    }

    tid_telemetry = app_start_task(telemetry_task, PRI_T7_TELEMETRY,
                                   APP_CORE, "T7_telemetry");
    return ((tid_telemetry < E_OK) ? tid_telemetry : E_OK);
}

/*** end of file ***/
