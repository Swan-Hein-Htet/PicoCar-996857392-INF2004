/** @file subsystem1.h
 *
 * @brief Subsystem 1 -- Vehicle control (mission state machine, T6) and
 *        WiFi/MQTT command and telemetry (T7).  Buddy 1, Swan Hein Htet,
 *        2501984.
 *
 * Public interface of subsystem 1.  Other subsystems include it as
 * "../subsystem1/subsystem1.h".
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

/* The system event flag (bits in app.h) and the navigation queue. */
IMPORT ID   flg_sys;
IMPORT ID   mbf_nav;        /* nav_cmd_t from T2 (subsystem3.h)         */

/* ---------------------------------------------------------------------
 *  g_vehicle_state: the only data still guarded by mtx_state.  It holds
 *  the obstacle profile (two words, layout in subsystem5.h), which is
 *  too large for one atomic store (tribunal Version C).  Writer: T5.
 *  Readers copy it under the lock and release before using it (NFR17).
 * --------------------------------------------------------------------- */
typedef struct
{
    uint32_t profile0;
    uint32_t profile1;
} vehicle_state_t;

IMPORT ID              mtx_state;
IMPORT vehicle_state_t g_vehicle_state;

/* ---------------------------------------------------------------------
 *  g_mission_word (uint16, single writer T6, rule R1)
 *    [3:0]   mission state S0-S10 (app.h MISSION_x)
 *    [6:4]   pending barcode command (NAV_x), consumed at a junction
 *    [15:7]  distance the pending command has waited, mm (0-511, saturates;
 *            commands no longer expire, D73)
 * --------------------------------------------------------------------- */
#define MISSION_STATE_SHIFT     0U
#define MISSION_STATE_BITS      4U
#define MISSION_PEND_SHIFT      4U
#define MISSION_PEND_BITS       3U
#define MISSION_BUDGET_SHIFT    7U
#define MISSION_BUDGET_BITS     9U

IMPORT volatile uint16_t g_mission_word;

IMPORT volatile uint8_t g_drive_key;    /* manual drive key, net_bridge.c */
IMPORT volatile uint8_t g_drive_seq;    /* +1 per key press               */
IMPORT void net_bridge_drive(unsigned char key);
IMPORT volatile uint8_t g_map_req[BC_MAP_LETTERS];  /* net_bridge.c       */
IMPORT volatile uint8_t g_map_req_len;              /* 0 = invalid length */
IMPORT volatile uint8_t g_map_seq;                  /* +1 per message     */
IMPORT void net_bridge_map(unsigned char const * p_data, int len);
IMPORT char const * mission_name(uint32_t state);   /* "S3 FOLLOW" */

static inline uint32_t
mission_state (void)
{
    return (bits_get(g_mission_word, MISSION_STATE_SHIFT,
                     MISSION_STATE_BITS));
}

/* ---------------------------------------------------------------------
 *  MQTT wire formats (docs/MQTT.md).  Topics:
 *    p5is20/car/telemetry   QoS 0, 21-byte binary record at 5 Hz
 *    p5is20/car/cmd         QoS 1, exactly one byte
 *    p5is20/car/status      retained "online"/"offline" (last will)
 *  Command byte: [1:0] command, [7:2] sequence (duplicates ignored).
 * --------------------------------------------------------------------- */
#define TELEMETRY_VERSION       2U  /* 2: + pitch, roll, hump height now */
#define TELEMETRY_BYTES         24U
#define COMMAND_BYTES           1U

#define CMD_START               0U
#define CMD_STOP                1U
#define CMD_RESET               2U
#define CMD_END                 3U

/* Network bridge, called ONLY from the kernel's CYW43/lwIP service task
   (lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c).  Never blocks. */
IMPORT int  net_bridge_take_telemetry(unsigned char * p_buf, int size);
IMPORT void net_bridge_command(unsigned char const * p_data, int len);
IMPORT void net_bridge_connected(int b_connected);
/* Next console line for p5is20/car/log (MQTT builds only): length, or 0
   if no complete line is waiting.  The line has no trailing newline. */
IMPORT int  net_bridge_take_log(unsigned char * p_buf, int size);

/* Private to subsystem 1 */
#define S1_T7_PERIOD_MS         200U
/* S1_PENDING_BUDGET_MM removed (D73): a command waits for the first junction. */

IMPORT ID   mbf_telem;
IMPORT ID   mbf_cmd;
IMPORT ID   tid_vehicle;
IMPORT ID   tid_telemetry;
IMPORT volatile uint8_t g_mqtt_connected;   /* writer: service task */
IMPORT void vehicle_task(INT stacd, void * p_exinf);
IMPORT void telemetry_task(INT stacd, void * p_exinf);

#endif /* SUBSYSTEM1_H */

/*** end of file ***/
