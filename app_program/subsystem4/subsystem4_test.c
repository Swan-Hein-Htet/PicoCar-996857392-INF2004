/** @file subsystem4_test.c
 *
 * @brief On-target tests for subsystem 4.  Built only with: make TEST=1
 */

#include "subsystem4.h"
#include "../app_math.h"
#include "../subsystem1/subsystem1.h"
#include "../test_report.h"

#if APP_TEST

#define STRESS_ROUNDS   20000U

EXPORT void subsystem4_test (void)
{
    T_RFLG   rflg;
    uint32_t idx;
    bool     b_ok = true;
    float    worst = 0.0f;

    TEST_INIT();

    /* Non-critical fault: recorded, but never raises ESTOP. */
    fault_set(FAULT_WIFI_LOSS);
    TEST_CHECK("S4-U01",
               (0U != ((uint32_t)fault_get() & FAULT_WIFI_LOSS)) &&
               !fault_is_critical() &&
               (E_OK == tk_ref_flg(flg_sys, &rflg)) &&
               (0U == (rflg.flgptn & FLG_ESTOP)),
               "WiFi loss is not critical (NFR19)");
    fault_clear(FAULT_WIFI_LOSS);
    TEST_CHECK("S4-U02", 0U == ((uint32_t)fault_get() & FAULT_WIFI_LOSS),
               "fault_clear removes exactly its bit");

    /* Packed-data stress (rule R2): set/clear one bit while the encoder,
       echo and button ISRs run; no other bit may change. */
    for (idx = 0U; idx < STRESS_ROUNDS; idx++)
    {
        uint16_t const before = fault_get();

        fault_set(FAULT_IMU);
        fault_clear(FAULT_IMU);
        if ((fault_get() & ~FAULT_IMU) != (before & ~FAULT_IMU))
        {
            b_ok = false;
        }
    }
    TEST_CHECK("S4-U03", b_ok, "20000 fault RMWs lose no other bit");

    TEST_CHECK("S4-U04",
               (-300 == bits_to_signed(bits_from_signed(-300, 10U), 10U)) &&
               (511 == bits_to_signed(bits_from_signed(900, 10U), 10U)) &&
               (-512 == bits_to_signed(bits_from_signed(-900, 10U), 10U)),
               "signed fields round-trip and saturate");

    for (idx = 0U; idx <= 90U; idx += 5U)
    {
        float const rad = (float)idx * 0.01745329f;
        float const err = app_fabsf(app_atan2f(app_sinf(rad),
                                               app_cosf(rad)) - rad);

        if (err > worst)
        {
            worst = err;
        }
    }
    TEST_CHECK("S4-U05", (worst < 0.0002f) &&
               (app_fabsf(app_sqrtf(1.0e6f) - 1000.0f) < 0.01f),
               "app_math accuracy (atan2/sin/cos/sqrt)");

    TEST_CHECK("S4-U06",
               bits_get(g_terrain_word, TERRAIN_IMU_SHIFT,
                        TERRAIN_IMU_BITS) <= IMU_IMPACT,
               "T3 publishes a valid IMU state");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */

/*** end of file ***/
