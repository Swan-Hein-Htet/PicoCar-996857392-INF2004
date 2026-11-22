/** @file subsystem5_test.c
 *
 * @brief On-target tests for subsystem 5.  Built only with: make TEST=1
 */

#include "subsystem5.h"
#include "../subsystem1/subsystem1.h"
#include "../test_report.h"

#if APP_TEST

#define PINGS_MS        500
#define HOLD_LIMIT_US   100U    /* NFR17 */

EXPORT void subsystem5_test (void)
{
    UW       echoes_before = g_echo_count;
    uint32_t start;
    uint32_t held;
    uint32_t copy0;

    TEST_INIT();

    tk_dly_tsk(PINGS_MS);
    TEST_CHECK("S5-U01", g_echo_count > echoes_before,
               "ISR_ECHO completes pings via the dispatcher (A5)");
    TEST_CHECK("S5-U02",
               (0U == bits_get(g_echo_word, ECHO_VALID_SHIFT, 1U)) ||
               (bits_get(g_echo_word, 0U, ECHO_WIDTH_BITS) < 23500U),
               "a valid echo word never carries a no-return width");

    start = app_micros();
    (void)tk_loc_mtx(mtx_state, TMO_FEVR);
    copy0 = g_vehicle_state.profile0;
    (void)tk_unl_mtx(mtx_state);
    held = app_micros() - start;
    TEST_CHECK("S5-U03", held < HOLD_LIMIT_US,
               "mtx_state copy-out hold under 100 us (NFR17)");
    TEST_CHECK("S5-U04",
               bits_get(copy0, PROF_SCAN_SHIFT, PROF_SCAN_BITS) <=
               SCAN_REVERSE, "profile carries a valid scan state");

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */

/*** end of file ***/
