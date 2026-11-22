/** @file subsystem1_test.c
 *
 * @brief On-target tests for subsystem 1.  Built only with: make TEST=1
 */

#include "subsystem1.h"
#include "telemetry.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem4/subsystem4.h"
#include "../test_report.h"

#ifndef TM_MQTT
#define TM_MQTT 0
#endif

#if APP_TEST

EXPORT void subsystem1_test (void)
{
    telemetry_snapshot_t snap = {0};
    uint8_t              rec[TELEMETRY_BYTES];
    uint8_t              msg[TELEMETRY_BYTES];
    T_RMBF               rmbf;
    int                  taken;

    TEST_INIT();

    snap.motion = bits_put(0U, MOTION_SPEED_L_SHIFT, MOTION_SPEED_BITS,
                           bits_from_signed(-123, MOTION_SPEED_BITS));
    snap.enc_left = 0xBEEFU;
    snap.faults = FAULT_WIFI_LOSS | FAULT_IMU;
    snap.mission = (uint16_t)bits_put(0U, MISSION_STATE_SHIFT,
                                      MISSION_STATE_BITS, MISSION_HUMP);
    snap.seq = 0xA5U;
    telemetry_pack(&snap, rec);

    TEST_CHECK("S1-U01",
               (TELEMETRY_VERSION == tlm_get(rec, TLM_VERSION_OFS,
                                             TLM_VERSION_BITS)) &&
               (0xA5U == tlm_get(rec, TLM_SEQ_OFS, TLM_SEQ_BITS)),
               "record starts with version and sequence");
    TEST_CHECK("S1-U02",
               (-123 == bits_to_signed(tlm_get(rec, TLM_SPEED_L_OFS,
                                               TLM_SPEED_BITS),
                                       TLM_SPEED_BITS)) &&
               (0xBEEFU == tlm_get(rec, TLM_ENC_L_OFS, TLM_ENC_BITS)) &&
               (MISSION_HUMP == tlm_get(rec, TLM_MISSION_OFS,
                                        TLM_MISSION_BITS)) &&
               ((FAULT_WIFI_LOSS | FAULT_IMU) ==
                tlm_get(rec, TLM_FAULTS_OFS, TLM_FAULTS_BITS)),
               "fields survive pack/unpack field by field");

    tk_dly_tsk(500);
    TEST_CHECK("S1-U03",
               (E_OK == tk_ref_mbf(mbf_telem, &rmbf)) &&
               ((uint32_t)rmbf.maxmsz == TELEMETRY_BYTES) &&
               (rmbf.wtsk >= 0),
               "T7 queues 21-byte records for the network task");
    /* In an MQTT build the network task drains mbf_telem every 50 ms as
       well, so finding it empty is not a failure there; anything other
       than a whole record or nothing is. */
    taken = net_bridge_take_telemetry(msg, (int)sizeof(msg));
    TEST_CHECK("S1-U04",
               (taken == (int)TELEMETRY_BYTES) || (TM_MQTT && (0 == taken)),
               "bridge hands over exactly one 21-byte record");

    TEST_CHECK("S1-U05", mission_state() <= MISSION_FAULT,
               "mission word holds a valid state");

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */

/*** end of file ***/
