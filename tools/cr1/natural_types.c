/* Change Requirement 1 "before" reference: the Week 6 data model written
 * with natural C types, compiled for the same target so the sizes are
 * measured, not estimated (tools/cr1/measure.sh).  Not part of the image. */
#include <stdint.h>
#include <stdbool.h>

typedef enum { CMD_FWD, CMD_BACK, CMD_LEFT, CMD_RIGHT, CMD_SPEED, CMD_STOP,
               CMD_FOLLOW } cmd_e;
typedef struct { cmd_e cmd; float arg; int source; } motion_req_nat_t;
typedef struct { int cmd; char character; int reject_reason; } nav_cmd_nat_t;
typedef struct { float bearing_deg; float width_mm; float closest_mm;
                 float left_clear_mm; float right_clear_mm; int decision;
                 int scan_state; int retries; } profile_nat_t;
typedef struct { bool estop, stall, encoder, imu, impact, no_echo, bypass,
                 wifi, line_lost, self_test, remote_stop, overrun; } faults_nat_t;
typedef struct { float line_error; int line_state; bool junction; bool end;
                 int seq; int last_barcode; int reject; float black_run_mm;
                 bool black; bool capturing; } line_nat_t;
typedef struct { float speed_l; float speed_r; int state; bool armed;
                 int beat; int last_cmd; } motion_nat_t;
typedef struct { float pitch_deg; float turn_dps; int motion_class;
                 int imu_state; bool hump; float hump_max_mm; } terrain_nat_t;
typedef struct { int state; int pending; float budget_mm; } mission_nat_t;
typedef struct { uint32_t width_us; bool valid; } echo_nat_t;

motion_req_nat_t n_motion_req;
nav_cmd_nat_t    n_nav_cmd;
profile_nat_t    n_profile;
faults_nat_t     n_faults;
line_nat_t       n_line;
motion_nat_t     n_motion;
terrain_nat_t    n_terrain;
mission_nat_t    n_mission;
echo_nat_t       n_echo;
float            n_distance_mm;
int32_t          n_enc_left;
int32_t          n_enc_right;
uint16_t         n_barcode_elements[29];   /* ms durations as uint16 */

/* Week 6 telemetry was a text record.  A representative line with the
   FR21 fields, as it would be published: */
char const n_telemetry_text[] =
    "{\"v\":1,\"seq\":90,\"state\":\"OBSTACLE\",\"motion\":\"TURN\","
    "\"vl\":-250,\"vr\":249,\"encl\":4660,\"encr\":65244,\"line\":\"TRACK\","
    "\"err\":-0.50,\"bc\":\"D\",\"rej\":3,\"hump\":42,\"ow\":101,\"oc\":187,"
    "\"ob\":90,\"plan\":\"LEFT\",\"dist\":12345,\"class\":\"TURNING\","
    "\"rate\":-321,\"faults\":160,\"scan\":7}";
char const n_command_text[] = "RESET";
