/** @file subsystem2.c
 *
 * @brief Subsystem 2: creates mbf_motion and starts T4 MotionCtrl with
 *        its 20 ms release.  Also the motion request API (FR10).
 */

#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"

#define MOTION_QUEUE_DEPTH      4U

EXPORT ID mbf_motion = 0;
EXPORT ID tid_motion = 0;

_Static_assert(2U == sizeof(motion_req_t), "motion_req_t is two bytes");

EXPORT ER subsystem2_init (void)
{
    T_CMBF cmbf = {0};
    ER     er;

    /* Motor pins low (brake) before anything else can run. */
    motor_init();

    /* Every GPIO interrupt source registers in phase 1; T4 enables the
       dispatcher on APP_CORE once all of them are known. */
    er = encoder_init();
    if (E_OK != er)
    {
        return (er);
    }

    cmbf.mbfatr = TA_TFIFO;
    /* 4-byte kernel header + body rounded up to 4 (messagebuf.c). */
    cmbf.bufsz  = MOTION_QUEUE_DEPTH *
                  (((sizeof(motion_req_t) + 3U) & ~3U) + 4U);
    cmbf.maxmsz = sizeof(motion_req_t);
    mbf_motion = tk_cre_mbf(&cmbf);
    return ((mbf_motion < E_OK) ? mbf_motion : E_OK);
}

EXPORT ER subsystem2_start (void)
{
    ID id;

    tid_motion = app_start_task(motion_task, PRI_T4_MOTION, APP_CORE,
                                "T4_motion");
    if (tid_motion < E_OK)
    {
        return (tid_motion);
    }
    id = app_start_waker(&tid_motion, S2_T4_PERIOD_MS);
    return ((id < E_OK) ? id : E_OK);
}

EXPORT ER motion_request (uint32_t cmd, uint32_t arg, uint32_t src)
{
    motion_req_t const req = motion_req_make(cmd, arg, src);

    return (tk_snd_mbf(mbf_motion, (void *)&req, sizeof(req), TMO_POL));
}

EXPORT void motion_flush (void)
{
    motion_req_t req = 0U;

    while (tk_rcv_mbf(mbf_motion, &req, TMO_POL) > 0)
    {
        tm_printf((UB *)"T6: flushed stale motion request 0x%04x\n",
                  (INT)req);
    }
}

/*** end of file ***/
