/** @file subsystem2_test.c
 *
 * @brief On-target tests for subsystem 2.  Built only with: make TEST=1
 *        Motor and encoder behaviour are manual tests (TEST_LOG.md).
 */

#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

#define WAIT_MS     200

EXPORT void subsystem2_test (void)
{
    motion_req_t const req = motion_req_make(MREQ_TURN_RIGHT, 4095U,
                                             MREQ_SRC_T5);
    motion_req_t const sat = motion_req_make(MREQ_FORWARD, 9000U,
                                             MREQ_SRC_T6);
    T_RMBF rmbf;
    UW     beat_before;

    TEST_INIT();

    TEST_CHECK("S2-U01",
               (MREQ_TURN_RIGHT == bits_get(req, MREQ_CMD_SHIFT,
                                            MREQ_CMD_BITS)) &&
               (4095U == bits_get(req, MREQ_ARG_SHIFT, MREQ_ARG_BITS)) &&
               (MREQ_SRC_T5 == bits_get(req, MREQ_SRC_SHIFT, 1U)),
               "motion_req_t packs command, argument and source");
    TEST_CHECK("S2-U02",
               (4095U == bits_get(sat, MREQ_ARG_SHIFT, MREQ_ARG_BITS)) &&
               (MREQ_FORWARD == bits_get(sat, MREQ_CMD_SHIFT,
                                         MREQ_CMD_BITS)),
               "out-of-range argument saturates, never spills");
    TEST_CHECK("S2-U03",
               (E_OK == tk_ref_mbf(mbf_motion, &rmbf)) &&
               (2 == rmbf.maxmsz),
               "mbf_motion carries 2-byte requests");

    beat_before = bits_get(g_motion_word, MOTION_BEAT_SHIFT,
                           MOTION_BEAT_BITS);
    tk_dly_tsk(WAIT_MS);
    TEST_CHECK("S2-U04",
               beat_before != bits_get(g_motion_word, MOTION_BEAT_SHIFT,
                                       MOTION_BEAT_BITS),
               "T4 runs every 20 ms (heartbeat moves)");
    TEST_CHECK("S2-U05",
               MSTATE_DISARMED == bits_get(g_motion_word, MOTION_STATE_SHIFT,
                                           MOTION_STATE_BITS),
               "motors stay disarmed until a request arrives");

    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */

/*** end of file ***/
