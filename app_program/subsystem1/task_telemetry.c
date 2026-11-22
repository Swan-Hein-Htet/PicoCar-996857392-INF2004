/** @file task_telemetry.c
 *
 * @brief T7 Telemetry (lowest priority, 200 ms): snapshot every shared
 *        word, pack the 21-byte record, hand it to the network service
 *        task, and act on validated one-byte MQTT commands.
 *
 * Template rule (TEAM.md section 3): no application task may call
 * cyw43, lwIP or MQTT.  The kernel's CYW43 service task owns the whole
 * stack; the MQTT code there (mqtt_utk.c) takes records from mbf_telem
 * and puts commands into mbf_cmd, never blocking.  T7 therefore never
 * waits on the network, and nothing waits on T7 (NFR19).
 */

#include "subsystem1.h"
#include "telemetry.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

#define CMD_CODE_BITS           2U
#define CMD_SEQ_SHIFT           2U
#define CMD_SEQ_BITS            6U
#define CONSOLE_EVERY           5U      /* status line once a second   */
#define CONSOLE_IDLE_EVERY      25U     /* every 5 s while parked      */
#define ECHO_US_PER_10MM        58U     /* round trip, 343 m/s         */
#define FAULT_BITS              13U

typedef struct
{
    uint32_t period;
    uint32_t last_seq;
    uint16_t faults_prev;
    bool     b_have_seq;
    bool     b_was_up;
    uint8_t  seq;
} telemetry_ctx_t;

static telemetry_ctx_t g_tc;

/* docs/FAULTS.md order, bit 0 first. */
static char const * const k_fault_names[FAULT_BITS] =
{
    "ESTOP_BUTTON", "MOTOR_STALL", "ENCODER", "IMU", "IMPACT", "NO_ECHO",
    "BYPASS_FAIL", "WIFI_LOSS", "LINE_NOT_FOUND", "SELF_TEST",
    "REMOTE_STOP", "TASK_OVERRUN", "LIFTED"
};

static char const * const k_cmd_names[4] = { "START", "STOP", "RESET", "END" };

static char const * const k_line_names[4] = { "cal", "track", "junction",
                                              "LOST" };

/* Report every fault bit that appeared or went away since last period,
   by name, with whether it stops the car.  Faults are set from ISRs and
   high-priority tasks that must not print; this is where they are seen. */
static void
fault_log (void)
{
    uint16_t const now = fault_get();
    uint32_t const diff = (uint32_t)(now ^ g_tc.faults_prev);
    uint32_t       bit;

    if (0U == diff)
    {
        return;
    }
    for (bit = 0U; bit < FAULT_BITS; bit++)
    {
        uint32_t const mask = 1UL << bit;

        if (0U == (diff & mask))
        {
            continue;
        }
        if (0U != ((uint32_t)now & mask))
        {
            tm_printf((UB *)"FAULT SET   %s (0x%04x)%s\n",
                      k_fault_names[bit], (INT)mask,
                      (0U != (mask & (uint32_t)FAULT_CRITICAL_MASK)) ?
                      " -- critical, motors off" : " -- warning only");
        }
        else
        {
            tm_printf((UB *)"FAULT CLEAR %s (0x%04x)\n",
                      k_fault_names[bit], (INT)mask);
        }
    }
    tm_printf((UB *)"FAULT mask now 0x%04x\n", (INT)now);
    g_tc.faults_prev = now;
}

static void
snapshot (telemetry_snapshot_t * p_snap)
{
    /* Each single-writer word is one aligned load (rule R1). */
    p_snap->line = g_line_word;
    p_snap->motion = g_motion_word;
    p_snap->odom = g_odom_word;
    p_snap->terrain = g_terrain_word;
    p_snap->mission = g_mission_word;
    p_snap->faults = fault_get();
    p_snap->enc_left = g_enc_left;
    p_snap->enc_right = g_enc_right;
    p_snap->hump = g_hump_byte;
    p_snap->tilt = g_tilt_word;
    p_snap->seq = g_tc.seq;

    /* The profile: copy under the lock, release before packing. */
    (void)tk_loc_mtx(mtx_state, TMO_FEVR);
    p_snap->profile0 = g_vehicle_state.profile0;
    p_snap->profile1 = g_vehicle_state.profile1;
    (void)tk_unl_mtx(mtx_state);
}

/* One readable line: once a second while the car is doing something,
   every 5 s while it waits (S0-S2, S9, S10). */
static void
status_line (telemetry_snapshot_t const * p_snap)
{
    uint32_t const state = bits_get(p_snap->mission, MISSION_STATE_SHIFT,
                                    MISSION_STATE_BITS);
    bool const     b_idle = (state <= MISSION_READY) ||
                            (MISSION_FINISHED == state) ||
                            (MISSION_FAULT == state);
    uint16_t const echo = g_echo_word;
    uint8_t const  bc = g_barcode_last;
    INT            sonar_mm = -1;   /* -1: no echo */

    if (0U != (g_tc.period % (b_idle ? CONSOLE_IDLE_EVERY : CONSOLE_EVERY)))
    {
        return;
    }
    if (0U != ((uint32_t)echo >> ECHO_VALID_SHIFT))
    {
        sonar_mm = (INT)((bits_get(echo, 0U, ECHO_WIDTH_BITS) * 10U) /
                         ECHO_US_PER_10MM);
    }
    tm_printf((UB *)"STATUS %s M%d | line %s err %d | speed L %d R %d mm/s"
              " | enc %u/%u | dist %d mm | sonar %d mm | obst %d mm"
              " | hump %d | barcode '%c' | faults 0x%04x | ovr %d\n",
              mission_name(state),
              (INT)bits_get(p_snap->motion, MOTION_STATE_SHIFT,
                            MOTION_STATE_BITS),
              k_line_names[bits_get(p_snap->line, LINE_STATE_SHIFT,
                                    LINE_STATE_BITS)],
              (INT)bits_to_signed(bits_get(p_snap->line, LINE_ERR_SHIFT,
                                           LINE_ERR_BITS), LINE_ERR_BITS),
              (INT)bits_to_signed(bits_get(p_snap->motion,
                                           MOTION_SPEED_L_SHIFT,
                                           MOTION_SPEED_BITS),
                                  MOTION_SPEED_BITS),
              (INT)bits_to_signed(bits_get(p_snap->motion,
                                           MOTION_SPEED_R_SHIFT,
                                           MOTION_SPEED_BITS),
                                  MOTION_SPEED_BITS),
              (UINT)p_snap->enc_left, (UINT)p_snap->enc_right,
              (INT)bits_get(p_snap->odom, ODOM_TOTAL_SHIFT, ODOM_TOTAL_BITS),
              sonar_mm,
              (INT)bits_get(p_snap->profile0, PROF_CLOSEST_SHIFT,
                            PROF_CLOSEST_BITS),
              (INT)bits_get(p_snap->hump, 0U, HUMP_PEAK_BITS),
              (0U == bc) ? (INT)'-' : (INT)bc,
              (INT)p_snap->faults, (INT)g_line_overruns);
}

static void
publish (void)
{
    telemetry_snapshot_t snap;
    uint8_t              record[TELEMETRY_BYTES];
    uint8_t              stale[TELEMETRY_BYTES];

    snapshot(&snap);
    telemetry_pack(&snap, record);
    g_tc.seq++;

    /* Newest replaces oldest: drop an unsent record rather than wait. */
    if (E_OK != tk_snd_mbf(mbf_telem, record, TELEMETRY_BYTES, TMO_POL))
    {
        (void)tk_rcv_mbf(mbf_telem, stale, TMO_POL);
        (void)tk_snd_mbf(mbf_telem, record, TELEMETRY_BYTES, TMO_POL);
    }

    status_line(&snap);
}

static void
link_update (void)
{
#if TM_MQTT
    bool const b_up = (0U != g_mqtt_connected);

    if (b_up && !g_tc.b_was_up)
    {
        (void)tk_set_flg(flg_sys, FLG_NET_UP);
        fault_clear(FAULT_WIFI_LOSS);
    }
    else if (!b_up)
    {
        (void)tk_clr_flg(flg_sys, ~FLG_NET_UP);
        fault_set(FAULT_WIFI_LOSS);     /* non-critical: never stops us */
    }
    else
    {
        /* Still up. */
    }
    g_tc.b_was_up = b_up;
#endif
}

/* Validate and act on one command message (FR22, FR23). */
static void
command (uint8_t const * p_msg, INT len)
{
    uint32_t code;
    uint32_t seq;

    uint32_t state;
    char const * p_effect = "";

    if (COMMAND_BYTES != (uint32_t)len)
    {
        tm_printf((UB *)"CMD rejected: %d bytes (must be exactly 1)\n", len);
        return;
    }

    code = bits_get(p_msg[0], 0U, CMD_CODE_BITS);
    seq = bits_get(p_msg[0], CMD_SEQ_SHIFT, CMD_SEQ_BITS);

    /* QoS 1 may deliver twice: a repeated sequence number is ignored. */
    if (g_tc.b_have_seq && (seq == g_tc.last_seq))
    {
        tm_printf((UB *)"CMD %s seq %d: duplicate, ignored\n",
                  k_cmd_names[code], (INT)seq);
        return;
    }
    g_tc.b_have_seq = true;
    g_tc.last_seq = seq;
    state = mission_state();

    switch (code)
    {
        case CMD_START:
            /* Exactly what GP20 does in the same state. */
            if (MISSION_CALIBRATE == state)
            {
                p_effect = " -> calibration sweep (if not already running)";
            }
            else if ((MISSION_READY == state) ||
                     (MISSION_FINISHED == state))
            {
                p_effect = " -> start driving";
            }
            else if (MISSION_FAULT == state)
            {
                p_effect = " -> try to reset the fault";
            }
            else
            {
                p_effect = " -> ignored, not waiting for START now";
            }
            (void)tk_set_flg(flg_sys, FLG_START);
        break;

        case CMD_STOP:
            /* NFR6 (as reworded): 100 ms counted from receipt here. */
            fault_set(FAULT_REMOTE_STOP);
        break;

        case CMD_RESET:
            p_effect = (MISSION_FAULT == state) ? " -> try to reset" :
                       ((MISSION_READY == state) ||
                        (MISSION_FINISHED == state)) ?
                       " -> back to start-up (recalibrate)" :
                       " -> ignored, nothing to reset";
            (void)tk_set_flg(flg_sys, FLG_RESET_REQ);
        break;

        default:    /* CMD_END: backup end-of-course marker */
            (void)tk_set_flg(flg_sys, FLG_END);
        break;
    }
    tm_printf((UB *)"CMD %s seq %d received in %s%s\n", k_cmd_names[code],
              (INT)seq, mission_name(state), p_effect);
}

EXPORT void telemetry_task (INT stacd, void * p_exinf)
{
    uint8_t msg[COMMAND_BYTES + 3U];
    SYSTIM  now;
    UW      next;
    INT     len;

    (void)stacd;
    (void)p_exinf;

    (void)tk_get_tim(&now);
    next = now.lo + S1_T7_PERIOD_MS;

    for (;;)
    {
        g_tc.period++;
        publish();
        link_update();
        fault_log();

        /* Wait for commands until the next 200 ms release. */
        for (;;)
        {
            (void)tk_get_tim(&now);
            if ((INT)(next - now.lo) <= 0)
            {
                break;
            }
            len = tk_rcv_mbf(mbf_cmd, msg, (TMO)(next - now.lo));
            if (len > 0)
            {
                command(msg, len);
            }
        }
        next += S1_T7_PERIOD_MS;
    }
}

/*** end of file ***/
