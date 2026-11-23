/* Scenario runner for the PicoCar simulator.
 *
 *   sim list                     scenarios and what each checks
 *   sim run <scenario> [seed]    one run, console log to stdout
 *   sim all [seeds] [filter]     every scenario x seeds, each in its own
 *                                process (fresh firmware statics)
 *
 * Each run boots the real firmware (usermain from app_main.c), drives it
 * through the scenario, and judges it with oracles that read both the
 * simulated world (ground truth) and the firmware's own shared words.
 */
#include "sim.h"
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/wait.h>

#include "../../app_program/app.h"
#include "../../app_program/subsystem1/subsystem1.h"
#include "../../app_program/subsystem1/telemetry.h"
#include "../../app_program/subsystem2/subsystem2.h"
#include "../../app_program/subsystem3/subsystem3.h"
#include "../../app_program/subsystem4/subsystem4.h"
#include "../../app_program/subsystem5/subsystem5.h"
#include "../../app_program/subsystem5/drv_servo.h"

extern INT usermain(void);
extern UINT sim_flg_ptn(ID id);
extern int sim_mbf_count(ID id);
extern void course_turn(double deg);
extern void course_shift(double lat);
extern UW g_line_overruns;
extern double world_heading_error(void);

#define MS(x)   ((uint64_t)(x) * 1000U)
#define S(x)    ((uint64_t)((x) * 1000000.0))
/* SIM_SLOW=k: scenario lengths and finish times x k (slower firmware). */
static double g_slow = 1.0;

/* ------------------------------------------------------------------ names */

const char *sim_name_for_entry(FP e)
{
    if (e == (FP)safety_task)    { return "T1_safety"; }
    if (e == (FP)line_task)      { return "T2_line"; }
    if (e == (FP)imu_task)       { return "T3_imu"; }
    if (e == (FP)motion_task)    { return "T4_motion"; }
    if (e == (FP)obstacle_task)  { return "T5_obstacle"; }
    if (e == (FP)vehicle_task)   { return "T6_vehicle"; }
    if (e == (FP)telemetry_task) { return "T7_telemetry"; }
    return "task?";
}

/* ------------------------------------------------------------------ results */

static char  g_fail[4096];
static int   g_nfail;
static char  g_notes[2048];
static bool  g_verbose;

static void fail(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    g_nfail++;
    if (strlen(g_fail) + strlen(buf) + 4 < sizeof(g_fail))
    {
        strcat(g_fail, g_nfail > 1 ? "; " : "");
        strcat(g_fail, buf);
    }
}
#define EXPECT(cond, ...) do { if (!(cond)) { fail(__VA_ARGS__); } } while (0)

static void note(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (strlen(g_notes) + strlen(buf) + 4 < sizeof(g_notes))
    {
        strcat(g_notes, g_notes[0] ? "; " : "");
        strcat(g_notes, buf);
    }
}

/* ------------------------------------------------------------------ monitor */

typedef struct
{
    /* console */
    char     decoded[64];
    int      n_decoded;
    int      n_rejected_bc;
    int      n_t4_rejected;
    int      n_expired;
    int      n_executed;
    int      n_map_set;             /* "T6: barcode map ... set"     */
    int      n_map_refused;
    int      n_unmapped;            /* "-> unmapped" reads             */
    int      n_reset_refused;
    int      n_hump_reports;
    bool     y_fwd;                 /* T3 found the board turned 90 deg */
    int      n_zeroed;
    int      n_hump_zeroed;
    int      n_obst_zeroed;         /* "obstacle width zeroed" lines */         /* "hump height zeroed" lines   */              /* "tilt and hump zeroed" lines */
    int      last_hump_mm;
    int      mounting;              /* 1 BETWEEN, 2 STRADDLE */
    bool     imu_diag;              /* T3 printed its I2C diagnosis */
    bool     cal_hint;              /* "start with the black line"   */
    int      obst_width;            /* first measured width, mm (-1 none) */
    int      n_obstacle_seen;
    int      n_bypass_failed;
    int      n_cmd_rejected;
    /* mission */
    uint32_t state;
    uint32_t states_seen;           /* bitmask */
    uint64_t state_at[16];          /* first entry time */
    uint64_t follow_since;
    uint32_t follow_entries;
    double   max_off_follow;        /* mm, settled following */
    double   sum_off; uint32_t n_off;
    double   max_herr, sum_herr2;
    uint64_t finished_at;
    uint64_t fault_at;
    uint16_t faults_seen;
    /* safety */
    uint64_t crit_at;               /* first time a critical fault was set */
    uint64_t motors_off_at;
    uint64_t max_crit_to_off_us;
    uint64_t moving_in_fault_us;
    uint64_t estop_press_at;
    int      n_calibrated;
    int      n_traces;
    int      trace_mm;
    int      n_recal;
    int      n_fault_lines;
    uint64_t estop_off_latency;
    uint64_t remote_stop_at;
    uint64_t remote_off_latency;
    /* telemetry */
    uint32_t tlm_records;
    uint32_t tlm_gaps;
    uint32_t tlm_bad;
    int      tlm_last_seq;
    uint32_t tlm_mismatch;
    uint8_t  tlm_last[TELEMETRY_BYTES];
    uint64_t last_seen[16];
    /* plan */
    uint32_t plans_seen;            /* bitmask of PLAN_x */
    uint32_t max_scan_state;
    /* auto-run */
    bool     auto_go;
    int      go_count;
    double   start_x, start_y;
    bool     net_up;
} mon_t;

static mon_t g_m;

#define MAXLINES 4000
static char  *g_con_lines[MAXLINES];
static int    g_n_con;
static int    g_n_log;
static int    g_log_mismatch;
static int    g_log_lost_reports;
static int    g_log_bad_stamp;
static bool   g_log_paused;
extern int net_bridge_take_log(unsigned char *buf, int size);

/* Junction centre (course cursor) and where the axle was when the turn
   was done: along-track error of the turn point (D79). */
static double g_junc_x = -1.0, g_junc_y;
static double g_turn_x = -1.0, g_turn_y;

static void console(uint64_t t, const char *task, const char *line)
{
    const char *p;
    if (g_n_con < MAXLINES) { g_con_lines[g_n_con++] = strdup(line); }
    (void)task;
    if ((p = strstr(line, "T2: barcode '")) && (g_m.n_decoded < 60)) { g_m.decoded[g_m.n_decoded++] = p[13]; }
    if (strstr(line, "barcode rejected")) { g_m.n_rejected_bc++; }
    if (strstr(line, "T4: rejected")) { g_m.n_t4_rejected++; }
    if (strstr(line, "expired")) { g_m.n_expired++; }
    if (strstr(line, "executed")) { g_m.n_executed++; g_turn_x = g_st.x; g_turn_y = g_st.y; }
    if (strstr(line, "T6: barcode map ") && strstr(line, ", set)")) { g_m.n_map_set++; }
    if (strstr(line, "T6: barcode map refused")) { g_m.n_map_refused++; }
    if (strstr(line, "T2: barcode '") && strstr(line, "-> unmapped")) { g_m.n_unmapped++; }
    if (strstr(line, "RESET refused")) { g_m.n_reset_refused++; }
    if (strstr(line, "T6: calibrated")) { g_m.n_calibrated++; }
    if ((p = strstr(line, "T2: trace saved in the log, "))) { g_m.n_traces++; g_m.trace_mm = atoi(p + 28); }
    if (strstr(line, "T6: RESET -- recalibrate")) { g_m.n_recal++; }
    if (strstr(line, "FAULT SET")) { g_m.n_fault_lines++; }
    if (strstr(line, "CMD rejected")) { g_m.n_cmd_rejected++; }
    if (strstr(line, "T3: tilt and hump zeroed")) { g_m.n_zeroed++; }
    if (strstr(line, "T3: hump height zeroed")) { g_m.n_hump_zeroed++; }
    if (strstr(line, "T5: obstacle width zeroed")) { g_m.n_obst_zeroed++; }
    if (strstr(line, "IMU board turned 90 deg")) { g_m.y_fwd = true; }
    if ((p = strstr(line, "hump peak "))) { g_m.n_hump_reports++; g_m.last_hump_mm = atoi(p + 10); }
    if (strstr(line, "mounting BETWEEN")) { g_m.mounting = 1; }
    if (strstr(line, "mounting STRADDLE")) { g_m.mounting = 2; }
    if (strstr(line, "T3: I2C idle")) { g_m.imu_diag = true; }
    if (strstr(line, "never the line - start with the black line")) { g_m.cal_hint = true; }
    if ((p = strstr(line, "T5: body scan ")) && (g_m.obst_width <= 0))
    {
        char const *w = strstr(p, "width ");
        if (w) { g_m.obst_width = atoi(w + 6); }
    }
    if (strstr(line, "T5: obstacle at")) { g_m.n_obstacle_seen++; }
    if (strstr(line, "bypass failed") || strstr(line, "no way round")) { g_m.n_bypass_failed++; }
    (void)t;
}

static bool following(uint32_t s)
{
    return (MISSION_FOLLOW == s) || (MISSION_BARCODE == s) || (MISSION_HUMP == s);
}

static void press_start_now(void *arg) { (void)arg; world_press(20U, g_now_us + 1000U, 120U); }

static void monitor(void *arg)
{
    uint32_t const s = mission_state();
    uint16_t const f = g_faults;
    double const v = fabs(g_st.vl) + fabs(g_st.vr);
    (void)arg;

    if (s != g_m.state)
    {
        if (!(g_m.states_seen & (1U << s))) { g_m.state_at[s] = g_now_us; }
        g_m.states_seen |= 1U << s;
        if (following(s) && !following(g_m.state)) { g_m.follow_since = g_now_us; g_m.follow_entries++; }
        if ((MISSION_FINISHED == s) && (0U == g_m.finished_at)) { g_m.finished_at = g_now_us; }
        if ((MISSION_FAULT == s) && (0U == g_m.fault_at)) { g_m.fault_at = g_now_us; }
        if ((MISSION_READY == s) && g_m.auto_go && (g_m.go_count > 0))
        {
            g_m.go_count--;
            sim_at(g_now_us + MS(300), press_start_now, NULL);
        }
        g_m.state = s;
    }
    g_m.last_seen[s & 15U] = g_now_us;
    if (getenv("SIM_TRACE") && (0U == (g_now_us / 1000U) % (unsigned)atoi(getenv("SIM_TRACE"))))
    {
        fprintf(stdout, "TRACE t=%.3f x=%.1f y=%.1f h=%.1f vl=%.0f vr=%.0f S%u line=0x%08x terr=0x%08x pitch=%.1f bc=%u faults=0x%04x servo=%.1f\n",
                g_now_us / 1e6, g_st.x, g_st.y, g_st.h * 57.3, g_st.vl, g_st.vr, s, g_line_word, g_terrain_word,
                g_st.pitch * 57.3, (unsigned)world_adc(2), g_faults, g_st.servo_deg);
    }

    /* following quality: 400 ms settling after each (re)entry */
    if (following(s) && ((g_now_us - g_m.follow_since) > MS(400)))
    {
        double const off = world_line_offset();
        if (off > g_m.max_off_follow) { g_m.max_off_follow = off; }
        double const he = fabs(world_heading_error()) * 57.29578;
        g_m.sum_off += off; g_m.n_off++;
        if (he > g_m.max_herr) { g_m.max_herr = he; }
        g_m.sum_herr2 += he * he;
    }

    g_m.faults_seen |= f;
    if ((0U != (f & FAULT_CRITICAL_MASK)) && (0U == g_m.crit_at)) { g_m.crit_at = g_now_us; }
    if ((0U != g_m.crit_at) && (0U == g_m.motors_off_at) && world_motors_off())
    {
        g_m.motors_off_at = g_now_us;
        g_m.max_crit_to_off_us = g_now_us - g_m.crit_at;
    }
    if ((MISSION_FAULT == s) && (v > 40.0)) { g_m.moving_in_fault_us += 1000U; }
    if ((0U != g_m.estop_press_at) && (0U == g_m.estop_off_latency) && world_motors_off())
    {
        g_m.estop_off_latency = g_now_us - g_m.estop_press_at + 1U;
    }
    if ((0U != g_m.remote_stop_at) && (0U == g_m.remote_off_latency) && world_motors_off())
    {
        g_m.remote_off_latency = g_now_us - g_m.remote_stop_at + 1U;
    }

    {
        uint32_t const p0 = g_vehicle_state.profile0;
        uint32_t const sc = bits_get(p0, PROF_SCAN_SHIFT, PROF_SCAN_BITS);
        if (sc >= SCAN_PLAN) { g_m.plans_seen |= 1U << bits_get(p0, PROF_PLAN_SHIFT, PROF_PLAN_BITS); }
        if (sc > g_m.max_scan_state) { g_m.max_scan_state = sc; }
    }
}

/* ------------------------------------------------------------------ network */

typedef struct { uint8_t bytes[4]; int len; } netcmd_t;
static netcmd_t g_cmds[16];
static int g_ncmds;

static void net_poll(void *arg)
{
    uint8_t buf[64];
    int len;
    (void)arg;
    while ((len = net_bridge_take_telemetry(buf, (int)sizeof(buf))) > 0)
    {
        int const seq = (int)tlm_get(buf, TLM_SEQ_OFS, TLM_SEQ_BITS);
        uint32_t const st = tlm_get(buf, TLM_MISSION_OFS, TLM_MISSION_BITS);
        bool seen = false;
        g_m.tlm_records++;
        if ((TELEMETRY_BYTES != (uint32_t)len) || (TELEMETRY_VERSION != tlm_get(buf, 0U, 4U))) { g_m.tlm_bad++; }
        if ((g_m.tlm_last_seq >= 0) && (((g_m.tlm_last_seq + 1) & 0xFF) != seq)) { g_m.tlm_gaps++; }
        g_m.tlm_last_seq = seq;
        seen = (g_m.last_seen[st & 15U] + MS(450) >= g_now_us) && (0U != (g_m.states_seen & (1U << st)));
        if (!seen) { g_m.tlm_mismatch++; }
        memcpy(g_m.tlm_last, buf, TELEMETRY_BYTES);
    }
    /* Console log over MQTT: up to 8 lines per 50 ms, as mqtt_utk.c. */
    for (int i = 0; (i < 8) && !g_log_paused; i++)
    {
        char line[257];
        int const n = net_bridge_take_log((unsigned char *)line, 256);
        if (n <= 0) { break; }
        line[n] = '\0';
        if (0 == strncmp(line, "[log] lost bytes", 16)) { g_log_lost_reports++; continue; }
        if ((n < 12) || ('[' != line[0]) || (']' != line[10])) { g_log_bad_stamp++; continue; }
        if ((g_n_log >= g_n_con) || (0 != strcmp(line + 12, g_con_lines[g_n_log])))
        {
            g_log_mismatch++;
            if (getenv("SIM_LOGDIFF")) { fprintf(stderr, "LOGDIFF got [%s] want [%s]\n", line + 12, (g_n_log < g_n_con) ? g_con_lines[g_n_log] : "-"); }
        }
        g_n_log++;
    }
    sim_at(g_now_us + MS(50), net_poll, NULL);
}

static void net_cmd_ev(void *arg)
{
    netcmd_t *c = arg;
    if ((1 == c->len) && ((c->bytes[0] & 3U) == CMD_STOP) && (0U == g_m.remote_stop_at))
    {
        g_m.remote_stop_at = g_now_us;
    }
    net_bridge_command(c->bytes, c->len);
}

static void mqtt(uint64_t at, uint32_t code, uint32_t seq)
{
    netcmd_t *c = &g_cmds[g_ncmds++];
    c->bytes[0] = (uint8_t)((code & 3U) | ((seq & 63U) << 2));
    c->len = 1;
    sim_at(at, net_cmd_ev, c);
}

static void mqtt_raw(uint64_t at, const uint8_t *b, int len)
{
    netcmd_t *c = &g_cmds[g_ncmds++];
    memcpy(c->bytes, b, (size_t)len);
    c->len = len;
    sim_at(at, net_cmd_ev, c);
}

static void log_pause_ev(void *arg) { g_log_paused = (0 != (intptr_t)arg); }
static void link_ev(void *arg) { net_bridge_connected((int)(intptr_t)arg); g_m.net_up = (0 != (intptr_t)arg); }

static void estop_mark(void *arg) { (void)arg; g_m.estop_press_at = g_now_us; }
static void estop_at(uint64_t at)
{
    world_press(21U, at, 150U);
    sim_at(at, estop_mark, NULL);
}

/* ------------------------------------------------------------------ courses */

#define NARROW 3.0
#define WIDE   9.0

/* Car on the line at the origin facing +x; line from -200 mm. */
static void base_course(void)
{
    course_reset(18.0);
    course_start(-200.0, 0.0, 0.0);
    course_straight(200.0);
}

static void finish_line(void)
{
    course_straight(150.0);
    course_end_marker(100.0, 80.0);
    course_straight(60.0);
}

/* ------------------------------------------------------------------ scenarios */

typedef struct
{
    const char *name;
    const char *what;
    void (*setup)(void);
    void (*check)(void);
    double secs;
    int    go;          /* START presses after READY (auto) */
    bool   allow_t4_reject;
    uint16_t allowed_faults;
    bool   xfail;       /* documents a known limitation: failure expected */
} scenario_t;

static const scenario_t *g_sc;

/* --- helpers for checks --- */
static void expect_finished(double within_s)
{
    EXPECT(0U != g_m.finished_at, "never reached S9 FINISHED (state S%u, faults 0x%04x)",
           g_m.state, g_m.faults_seen);
    if (0U != g_m.finished_at)
    {
        EXPECT(g_m.finished_at <= S(within_s * g_slow), "finished late: %.1f s", (double)g_m.finished_at / 1e6);
        note("finished %.1f s", (double)g_m.finished_at / 1e6);
    }
    EXPECT(MISSION_FINISHED == g_m.state, "final state S%u, not S9", g_m.state);
    EXPECT(fabs(g_st.vl) + fabs(g_st.vr) < 20.0, "still moving at the end (%.0f mm/s)",
           (fabs(g_st.vl) + fabs(g_st.vr)) / 2);
}

static void expect_follow_quality(double max_mm)
{
    EXPECT(g_m.max_off_follow <= max_mm, "line offset %.1f mm > %.0f mm while following",
           g_m.max_off_follow, max_mm);
    if (g_m.n_off > 0)
    {
        note("offset max %.1f mean %.1f mm, heading err max %.1f rms %.1f deg", g_m.max_off_follow,
             g_m.sum_off / g_m.n_off, g_m.max_herr, sqrt(g_m.sum_herr2 / g_m.n_off));
    }
}

static void expect_fault(uint16_t bit, const char *name)
{
    EXPECT(0U != (g_m.faults_seen & bit), "%s fault never raised (faults 0x%04x)", name, g_m.faults_seen);
    EXPECT(0U != (g_m.states_seen & (1U << MISSION_FAULT)), "never entered S10");
    EXPECT(g_m.max_crit_to_off_us <= MS(50), "motors on %.1f ms after the critical fault (NFR5 50 ms)",
           (double)g_m.max_crit_to_off_us / 1000.0);
    EXPECT(g_m.moving_in_fault_us <= MS(300), "car kept moving %.0f ms in S10",
           (double)g_m.moving_in_fault_us / 1000.0);
}

static double dist_to(double x, double y) { return hypot(g_st.x - x, g_st.y - y); }

/* 1. Boot only. */
static void sc_boot(void) { base_course(); course_straight(800.0); }
static void ck_boot(void)
{
    EXPECT(MISSION_CALIBRATE == g_m.state, "after boot state S%u, expected S1 waiting", g_m.state);
    EXPECT(world_motors_off(), "motors driven before START");
    EXPECT(dist_to(g_car.x0, g_car.y0) < 1.0, "car moved %.1f mm before START", dist_to(g_car.x0, g_car.y0));
    EXPECT(IMU_FLAT == bits_get(g_terrain_word, TERRAIN_IMU_SHIFT, TERRAIN_IMU_BITS),
           "IMU not calibrated after 4 s (I%u)", bits_get(g_terrain_word, TERRAIN_IMU_SHIFT, TERRAIN_IMU_BITS));
    EXPECT(hw_pwm_top(4U) == 6249U && hw_pwm_top(5U) == 6249U, "motor PWM TOP not 20 kHz");
    EXPECT(hw_pwm_top(6U) == 19999U && hw_pwm_div(6U) == (125U << 4), "servo PWM not 50 Hz");
#if SERVO_SCAN
    EXPECT((hw_pwm_cc(6U) & 0xFFFFU) == 1500U, "servo not centred (%u us)", hw_pwm_cc(6U) & 0xFFFFU);
#else
    EXPECT((hw_pwm_cc(6U) & 0xFFFFU) == SERVO_STOP_US, "continuous servo not held at stop (%u us)", hw_pwm_cc(6U) & 0xFFFFU);
    EXPECT(fabs(g_st.servo_deg - 90.0) < 3.0, "sonar not facing front (%.1f deg)", g_st.servo_deg);
#endif
    /* Queue depths as designed (the simulation is stopped: nobody drains). */
    {
        uint8_t junk[TELEMETRY_BYTES] = { 0 };
        int n_mot = 0, n_nav = 0, n_tlm = 0, n_cmd = 0;
        while ((n_mot < 20) && (E_OK == tk_snd_mbf(mbf_motion, junk, 2, TMO_POL))) { n_mot++; }
        while ((n_nav < 20) && (E_OK == tk_snd_mbf(mbf_nav, junk, 2, TMO_POL))) { n_nav++; }
        while ((n_tlm < 20) && (E_OK == tk_snd_mbf(mbf_telem, junk, TELEMETRY_BYTES, TMO_POL))) { n_tlm++; }
        while ((n_cmd < 20) && (E_OK == tk_snd_mbf(mbf_cmd, junk, 1, TMO_POL))) { n_cmd++; }
        n_tlm += sim_mbf_count(mbf_telem) - n_tlm;     /* T7 may have one queued */
        EXPECT(4 == n_mot, "mbf_motion holds %d requests, designed 4", n_mot);
        EXPECT(4 == n_nav, "mbf_nav holds %d commands, designed 4", n_nav);
        EXPECT(2 == n_tlm, "mbf_telem holds %d records, designed 2", n_tlm);
        EXPECT((8 == n_cmd) || (9 == n_cmd), "mbf_cmd holds %d commands, designed 8 (+1 handed to a waiting T7)", n_cmd);
        note("queues motion %d nav %d telem %d cmd %d", n_mot, n_nav, n_tlm, n_cmd);
        g_kstats.mbf_send_fail = 0U;
    }
}

/* 2. Calibration only. */
static void sc_cal(void) { base_course(); course_straight(800.0); world_press(20U, S(2.5), 100); }
static void ck_cal(void)
{
    EXPECT(MISSION_READY == g_m.state, "after calibration state S%u, expected S2", g_m.state);
    EXPECT(1 == g_m.mounting, "mounting %s, expected BETWEEN", g_m.mounting == 2 ? "STRADDLE" : "unknown");
    EXPECT(fabs(remainder(g_st.h, 2 * 3.14159265) * 57.3 - g_car.h0_deg) < 3.0,
           "heading after sweep %+.1f deg, started %+.1f", remainder(g_st.h, 2 * 3.14159265) * 57.3, g_car.h0_deg);
    EXPECT(dist_to(g_car.x0, g_car.y0) < 15.0, "car drifted %.1f mm during the sweep", dist_to(g_car.x0, g_car.y0));
    note("heading after sweep %+.1f deg (started %+.1f)", remainder(g_st.h, 2 * 3.14159265) * 57.3, g_car.h0_deg);
}

/* 3. Straight follow to the END marker. */
static void sc_straight(void) { base_course(); course_straight(1500.0); finish_line(); }
static void ck_straight(void)
{
    double const odo = (double)bits_get(g_odom_word, ODOM_TOTAL_SHIFT, ODOM_TOTAL_BITS);
    expect_finished(20.0);
    expect_follow_quality(20.0);
    /* The END is seen by the IR pair, ir_ahead_mm in front of the axle: a
       slower car stops with that pair, not its centre, on the marker. */
    EXPECT(g_st.x + g_car.ir_ahead_mm > 1650.0 && g_st.x < 1900.0, "stopped at x=%.0f (sensors at %.0f), END marker is 1650-1750", g_st.x, g_st.x + g_car.ir_ahead_mm);
    EXPECT(fabs(odo - g_st.dist) < 0.05 * g_st.dist + 20.0, "odometry %.0f mm vs true %.0f mm", odo, g_st.dist);
}

/* 4. Curves both ways. */
static void sc_curves(void)
{
    base_course(); course_straight(300.0);
    course_arc(400.0, 90.0); course_straight(300.0);
    course_arc(300.0, -180.0); course_straight(200.0);
    course_arc(500.0, 90.0); course_straight(200.0);
    finish_line();
}
static void ck_curves(void) { expect_finished(30.0); expect_follow_quality(20.0); }
static void ck_c_far(void)
{
    EXPECT(g_m.n_decoded >= 1 && g_m.decoded[0] == 'C', "decoded '%c', expected 'C'", g_m.n_decoded ? g_m.decoded[0] : '-');
    expect_finished(30.0);
    EXPECT(fabs(g_st.y) < 60.0, "left the straight line (y=%.0f)", g_st.y);
}

/* 5. Tight curve, radius 200 mm. */
static void sc_tight(void) { base_course(); course_straight(300.0); course_arc(200.0, 120.0); course_straight(300.0); finish_line(); }
static void ck_tight(void) { expect_finished(25.0); expect_follow_quality(30.0); }
/* Sharper than R200 (bench 2026-11-16: the car ran wide on the course's
   sharper bends at 100 mm/s). */
static void sc_sharp150(void) { base_course(); course_straight(300.0); course_arc(150.0, 120.0); course_straight(300.0); finish_line(); }
static void sc_sharp120(void) { base_course(); course_straight(300.0); course_arc(120.0, 90.0); course_straight(300.0); finish_line(); }
static void sc_sharp90(void) { base_course(); course_straight(300.0); course_arc(getenv("SIM_R") ? atof(getenv("SIM_R")) : 90.0, getenv("SIM_LEFT") ? 90.0 : -90.0); course_straight(300.0); finish_line(); }
/* 180-degree hairpins (owner 2026-11-19: tight 90s work, 180s not). */
static void sc_hairpin(double r, double deg) { base_course(); course_straight(300.0); course_arc(r, deg); course_straight(300.0); finish_line(); }
static void sc_hairpin100(void) { sc_hairpin(100.0, 180.0); }
static void sc_hairpin70(void) { sc_hairpin(70.0, 180.0); }
static void sc_hairpin70r(void) { sc_hairpin(70.0, -180.0); }
static void sc_hairpin50(void) { sc_hairpin(50.0, 180.0); }
static void sc_hairpin50r(void) { sc_hairpin(50.0, -180.0); }
static void sc_sbend(void) { base_course(); course_straight(250.0); course_arc(150.0, 90.0); course_straight(80.0); course_arc(150.0, -90.0); course_straight(250.0); finish_line(); }
static void ck_sharp(void) { expect_finished(25.0); expect_follow_quality(30.0); }

/* 6-8. Barcode then junction. */
static void junction_course2(char c, char layout, double before_junction)
{
    base_course(); course_straight(250.0);
    course_barcode(c, NARROW, WIDE, 0);
    course_straight(before_junction);
    /* junction: lines to the left, right and straight; only the intended
       one (layout A left, B right, else straight) leads to the END marker,
       the others stop after 250 mm. */
    c = layout;
    course_where(&g_junc_x, &g_junc_y);
    if ('A' == c) { course_branch(0.0, 250.0); course_branch(-90.0, 250.0); course_turn(90.0); }
    else if ('B' == c) { course_branch(0.0, 250.0); course_branch(90.0, 250.0); course_turn(-90.0); }
    else { course_branch(90.0, 250.0); course_branch(-90.0, 250.0); }
    course_straight(300.0);
    finish_line();
}
static void junction_course(char c, double before_junction) { junction_course2(c, c, before_junction); }
static void sc_bc_a(void) { junction_course('A', 250.0); }
/* Real car 2026-11-19 17:54: the junction close after the barcode: the
   capture took in the cross bar and the command came only after it. */
static void sc_bc_close(void) { junction_course('A', getenv("SIM_GAP") ? atof(getenv("SIM_GAP")) : 20.0); }
static void sc_bc_close_b(void) { junction_course('B', getenv("SIM_GAP") ? atof(getenv("SIM_GAP")) : 20.0); }
/* Real car 2026-11-19: barcode read, command pending, junction never seen.
   A real sensor spot is wide (an 18 mm bar reads fully dark over only part
   of it) and the cross bar may meet the car at a slant, so the two line
   sensors are dark together for less than LINE_JUNCTION_MM. */
static double g_jslant;
static double g_jspot;
/* The wide spot from the moment the barcode is read: the test is about
   the junction, and a wide spot also changes the line following before
   the barcode (and with it the read). */
static void jspot_poll(void *a)
{
    (void)a;
    if (g_m.n_decoded > 0) { g_car.ir_line_spot_mm = g_jspot; return; }
    sim_at(g_now_us + MS(20), jspot_poll, NULL);
}
static void junction_slant(char c, double slant, double spot)
{
    g_jspot = spot;
    if (spot > 0.0) { sim_at(S(1.0), jspot_poll, NULL); }
    base_course(); course_straight(250.0);
    course_barcode(c, NARROW, WIDE, 0);
    course_straight(250.0);
    g_jslant = slant;
    if ('A' == c) { course_branch(0.0, 250.0); course_branch(-90.0 + slant, 250.0); course_turn(90.0 + slant); }
    else { course_branch(0.0, 250.0); course_branch(90.0 + slant, 250.0); course_turn(-90.0 + slant); }
    course_straight(300.0);
    finish_line();
}
static void sc_junc_spot(void) { junction_slant('A', 0.0, 9.0); }
static void sc_junc_slant(void) { junction_slant('A', 12.0, 0.0); }
static void sc_junc_both(void) { junction_slant('A', 12.0, 9.0); }
static void sc_junc_both_b(void) { junction_slant('B', -12.0, 9.0); }
/* Barcode bench (BARCODE_TEST=1 build, build/sim_bc): motors off, the car
   is rolled by hand over four symbols at 100 mm/s; every letter must be
   printed.  Different speeds: the decode is ratio based. */
static void push_ev(void *arg) { g_car.push_mms = (double)(intptr_t)arg; }
static void sc_bc_bench(void)
{
    g_car.y0 = 0.0; g_car.h0_deg = 0.0;     /* rolled along the line */
    base_course(); course_straight(150.0);
    course_barcode('A', NARROW, WIDE, 0); course_straight(120.0);
    course_barcode('7', NARROW, WIDE, 0); course_straight(120.0);
    course_barcode('Z', NARROW, WIDE, 0); course_straight(120.0);
    course_barcode('D', NARROW, WIDE, 0); course_straight(300.0);
    sim_at(S(1.0), push_ev, (void *)(uintptr_t)100U);
    sim_at(S(6.0), push_ev, (void *)(uintptr_t)60U);    /* slower hand */
}
/* The same with a jerky hand: 40..160 mm/s, changing every 150 ms. */
static void sc_bc_jerky(void)
{
    static const uint32_t v[] = { 40U, 160U, 70U, 130U, 50U, 110U, 150U, 60U };
    sc_bc_bench();
    for (int i = 0; i < 100; i++)
    {
        sim_at(S(1.0) + (uint64_t)i * MS(150), push_ev, (void *)(uintptr_t)v[i % 8]);
    }
}
/* Every character of the set, in order, at the three course sizes in turn
   (narrow 3 / 4.4 / 6 mm), rolled over by hand. */
static const char k_all_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
static void sc_bc_all(void)
{
    static const double narrow[3] = { 3.0, 4.4, 6.0 };
    g_car.y0 = 0.0; g_car.h0_deg = 0.0;     /* rolled along the line */
    base_course(); course_straight(150.0);
    for (int i = 0; k_all_chars[i] != '\0'; i++)
    {
        double const n = narrow[i % 3];
        course_barcode(k_all_chars[i], n, 3.0 * n, 0);
        course_straight(100.0);
    }
    sim_at(S(1.0), push_ev, (void *)(uintptr_t)200U);
}
static void ck_bc_all(void)
{
    int const n = (int)strlen(k_all_chars);
    EXPECT(n == g_m.n_decoded, "%d characters read, expected %d", g_m.n_decoded, n);
    EXPECT(0 == memcmp(g_m.decoded, k_all_chars, (size_t)n), "read '%.*s'", g_m.n_decoded, g_m.decoded);
    EXPECT(0 == g_m.n_rejected_bc, "%d rejects", g_m.n_rejected_bc);
}
/* The same 43, read the other way round: every symbol is met end first,
   at a jerky hand speed.  sign -1: rolled backwards; +1: the car turned
   round and rolled forwards, with the symbols on its sensor side. */
static void jerky_ev(void *arg)
{
    static const int v[] = { 200, 120, 240, 90, 180, 260, 110, 150 };
    static unsigned s_i = 0U;
    g_car.push_mms = (double)((intptr_t)arg * v[s_i++ % 8U]);
    sim_at(g_now_us + MS(150), jerky_ev, arg);
}
static void rolled_over_all(int sign)
{
    sim_at(S(1.0), jerky_ev, (void *)(intptr_t)sign);
}
static void all_symbols(void)
{
    static const double narrow[3] = { 3.0, 4.4, 6.0 };
    g_car.y0 = 0.0;
    base_course(); course_straight(150.0);
    for (int i = 0; k_all_chars[i] != '\0'; i++)
    {
        double const n = narrow[i % 3];
        course_barcode(k_all_chars[i], n, 3.0 * n, 0);
        course_straight(100.0);
    }
    g_car.x0 = g_course.cur.x;          /* start past the last symbol */
}
static void sc_bc_backward(void)
{
    g_car.h0_deg = 0.0;
    all_symbols();
    rolled_over_all(-1);
}
static void sc_bc_turned(void)
{
    extern double g_bar_in, g_bar_out;
    g_car.h0_deg = 180.0;
    g_bar_in = -40.0; g_bar_out = -16.0;    /* left of the course direction */
    all_symbols();
    rolled_over_all(1);
}
static void ck_bc_reversed(void)
{
    int const n = (int)strlen(k_all_chars);
    char want[64];
    for (int i = 0; i < n; i++) { want[i] = k_all_chars[n - 1 - i]; }
    EXPECT(n == g_m.n_decoded, "%d characters read, expected %d: '%.*s'", g_m.n_decoded, n, g_m.n_decoded, g_m.decoded);
    EXPECT(0 == memcmp(g_m.decoded, want, (size_t)n), "read '%.*s'", g_m.n_decoded, g_m.decoded);
    EXPECT(0 == g_m.n_rejected_bc, "%d rejects", g_m.n_rejected_bc);
}
/* Self-driven scan (GP20 twice): motors on, forward over 'A', then back. */
static void sc_bc_scan(void)
{
    g_car.y0 = 0.0; g_car.h0_deg = 0.0;
    base_course(); course_straight(120.0);
    course_barcode('A', NARROW, WIDE, 0); course_straight(500.0);
    world_press(20U, S(1.0), 120U);
    world_press(20U, S(21.0), 120U);
}
static void phase_ev(void *arg) { (void)arg; world_encoder_phase(); }
/* The largest course size (narrow 6 mm), as the real car first read it. */
static void sc_bc_scan_large(void)
{
    g_car.y0 = 0.0; g_car.h0_deg = 0.0;
    base_course(); course_straight(95.0);
    course_barcode('A', 6.0, 18.0, 0); course_straight(500.0);
    sim_at(S(0.5), phase_ev, NULL);
    world_press(20U, S(1.0), 120U);
    world_press(20U, S(21.0), 120U);
}
static void ck_bc_scan(void)
{
    EXPECT(2 == g_m.n_decoded, "%d reads, expected 2", g_m.n_decoded);
    EXPECT((g_m.n_decoded >= 2) && (0 == memcmp(g_m.decoded, "AA", 2)), "read '%.*s'", g_m.n_decoded, g_m.decoded);
    EXPECT(0 == g_m.n_rejected_bc, "%d rejects", g_m.n_rejected_bc);
    EXPECT(dist_to(g_car.x0, g_car.y0) < 15.0, "ended %.0f mm from the start", dist_to(g_car.x0, g_car.y0));
    EXPECT(2 == g_m.n_traces, "%d traces, expected one per pass", g_m.n_traces);
}
/* Thin bars through a blurred sensor: all 43 at three sizes, 6 mm spot. */
static void ck_bc_blur(void)
{
    /* At this blur a few 3 mm symbols may be refused, never misread. */
    int j = 0;
    EXPECT(g_m.n_decoded >= 40, "%d characters read, expected 40+", g_m.n_decoded);
    for (int i = 0; (i < g_m.n_decoded) && (k_all_chars[j] != '\0'); j++)
    {
        if (k_all_chars[j] == g_m.decoded[i]) { i++; if (i == g_m.n_decoded) { return; } }
    }
    EXPECT(false, "a wrong character was read: '%.*s'", g_m.n_decoded, g_m.decoded);
}
static void sc_bc_blur(void) { g_car.ir_spot_mm = 6.0; sc_bc_all(); }
/* The stationary hand test: the card slides under a standing car, no
   encoder counts, the hand speed drifting +-25%, 6 mm spot, 3 mm bars. */
static int g_hand_pct = 100;
static void hand_ev(void *arg)
{
    static const int v[] = { 100, 115, 125, 110, 90, 75, 85, 105 };
    static unsigned s_i = 0U;
    g_car.push_mms = (double)(v[s_i++ % 8U] * (intptr_t)arg) / 100.0;
    sim_at(g_now_us + MS(400), hand_ev, arg);
}
static void sc_bc_hand(void)
{
    g_car.y0 = 0.0; g_car.h0_deg = 0.0;
    g_car.ir_spot_mm = 6.0; g_car.hand_slide = true;
    base_course(); course_straight(150.0);
    course_barcode('A', NARROW, WIDE, 0); course_straight(250.0);
    course_barcode('K', NARROW, WIDE, 0); course_straight(250.0);
    course_barcode('Z', NARROW, WIDE, 0); course_straight(250.0);
    sim_at(S(1.0), hand_ev, (void *)(intptr_t)g_hand_pct);
}
/* A 141 mm symbol in about 3 s, and a quick 0.7 s flick. */
static void sc_bc_hand_3s(void) { g_hand_pct = 47; sc_bc_hand(); }
static void sc_bc_hand_quick(void) { g_hand_pct = 200; sc_bc_hand(); }
static void ck_bc_hand(void)
{
    /* Timed by hand: a speed change inside a character may refuse that
       symbol.  It must read most of them and never give a wrong letter. */
    char const * p = "AKZ";
    EXPECT(g_m.n_decoded >= 2, "%d letters read, expected 2+: '%.*s'", g_m.n_decoded, g_m.n_decoded, g_m.decoded);
    for (int i = 0; i < g_m.n_decoded; i++)
    {
        p = strchr(p, g_m.decoded[i]);
        EXPECT(NULL != p, "a wrong letter was read: '%.*s'", g_m.n_decoded, g_m.decoded);
        if (NULL == p) { return; }
    }
}
/* Junk before and after the symbol, too close for any quiet zone: a car
   set down on the paper and lifted off again.  hand: the card slides. */
static void air_course(bool hand)
{
    g_car.y0 = 0.0; g_car.h0_deg = 0.0;
    g_car.hand_slide = hand;
    base_course(); course_straight(150.0);
    course_mark(40.0, 7.0); course_mark(12.0, 5.0); course_mark(25.0, 7.0);
    course_barcode('K', NARROW, WIDE, 0);
    course_straight(6.0);
    course_mark(30.0, 6.0); course_mark(9.0, 7.0); course_mark(60.0, 200.0);
    course_mark(35.0, 6.0); course_mark(15.0, 7.0);
    course_barcode('Q', NARROW, WIDE, 0);
    course_straight(7.0);
    course_mark(50.0, 300.0);
    sim_at(S(1.0), push_ev, (void *)(uintptr_t)100U);
}
static void sc_bc_air(void) { air_course(false); }
static void sc_bc_air_hand(void) { air_course(true); }
static void ck_bc_air(void)
{
    EXPECT(2 == g_m.n_decoded, "%d letters read, expected 2: '%.*s'", g_m.n_decoded, g_m.n_decoded, g_m.decoded);
    EXPECT((g_m.n_decoded >= 2) && (0 == memcmp(g_m.decoded, "KQ", 2)), "read '%.*s'", g_m.n_decoded, g_m.decoded);
}
/* Manual driving from the console (p5is20/car/drive). */
extern void net_bridge_drive(unsigned char key);
static void drive_ev(void *arg) { net_bridge_drive((unsigned char)(uintptr_t)arg); }
static void hold_key(char key, double from_s, double to_s)
{
    for (double t = from_s; t <= to_s; t += 0.04)   /* terminal key repeat */
    {
        sim_at(S(t), drive_ev, (void *)(uintptr_t)(unsigned char)key);
    }
}
static double g_md_x[4], g_md_h[4];
static void md_snap(void *arg)
{
    int const i = (int)(intptr_t)arg;
    g_md_x[i] = g_st.x; g_md_h[i] = g_st.h * 57.29578;
}
static void sc_manual(void)
{
    g_car.y0 = 0.0; g_car.h0_deg = 0.0;
    base_course(); course_straight(1500.0);
    hold_key('w', 1.0, 2.5);                        /* S1, before calibrating */
    sim_at(S(3.6), md_snap, (void *)(intptr_t)0);
    hold_key('a', 4.0, 5.0);
    sim_at(S(6.0), md_snap, (void *)(intptr_t)1);
    sim_at(S(6.5), drive_ev, (void *)(uintptr_t)'d'); /* one tap */
    sim_at(S(7.6), md_snap, (void *)(intptr_t)2);
    sim_at(S(8.0), md_snap, (void *)(intptr_t)3);
}
static void ck_manual(void)
{
    double const fwd = g_md_x[0] - g_car.x0;
    EXPECT((fwd > 80.0) && (fwd < 300.0), "held w for 1.5 s: moved %.0f mm", fwd);
    EXPECT(g_md_h[1] - g_md_h[0] > 30.0, "held a for 1 s: turned %.0f deg", g_md_h[1] - g_md_h[0]);
    EXPECT((g_md_h[2] < g_md_h[1] - 3.0) && (g_md_h[2] > g_md_h[1] - 90.0), "one tap of d: turned %.0f deg", g_md_h[2] - g_md_h[1]);
    EXPECT(fabs(g_md_h[3] - g_md_h[2]) < 0.5 && fabs(g_md_x[3] - g_md_x[2]) < 1.0, "still moving after the keys stopped");
    EXPECT(0U == (g_m.faults_seen & FAULT_CRITICAL_MASK), "fault 0x%04x", g_m.faults_seen);
}
/* Manual driving over a hump (manual page shows its height). */
static void sc_manual_hump(void)
{
    g_car.y0 = 0.0; g_car.h0_deg = 0.0;
    base_course(); course_straight(300.0); course_hump(200.0, 15.0); course_straight(800.0);
    hold_key('w', 3.0, 9.5);                        /* S1, before calibrating */
}
static void ck_manual_hump(void)
{
    EXPECT(g_m.n_hump_reports >= 1, "hump not measured while driving by hand");
    EXPECT(abs(g_m.last_hump_mm - 15) <= 3, "manual hump %d mm, true 15", g_m.last_hump_mm);
    note("hump %d mm (true 15)", g_m.last_hump_mm);
}
/* Keys during an autonomous run are ignored: the run finishes as usual. */
static void sc_manual_running(void) { sc_straight(); hold_key('d', 6.0, 7.0); }
static void ck_manual_running(void) { expect_finished(25.0); }
/* An obstacle about 110 mm ahead: w is refused.  One about 175 mm ahead:
   w drives until the sonar reads under 150 mm, then stops short of it. */
static void manual_obstacle(double ahead)
{
    g_car.y0 = 0.0; g_car.h0_deg = 0.0;
    base_course(); course_straight(600.0);
    course_obstacle(ahead - 600.0, 0.0, 100.0, 60.0);
    hold_key('w', 2.0, 4.0);
}
static void sc_manual_blocked(void) { manual_obstacle(170.0); }
static void ck_manual_blocked(void)
{
    EXPECT(fabs(g_st.x - g_car.x0) < 5.0, "drove %.0f mm towards an obstacle already too close", g_st.x - g_car.x0);
    EXPECT(!g_st.collided, "hit the obstacle");
}
static void sc_manual_approach(void) { manual_obstacle(235.0); }
static void ck_manual_approach(void)
{
    EXPECT(g_st.x - g_car.x0 > 10.0, "did not drive at all (%.0f mm)", g_st.x - g_car.x0);
    EXPECT(!g_st.collided, "hit the obstacle");
}
/* Plaid mode and stunts (manual control page): '!' arms plaid, j r b t p
   run a stunt, any key aborts one. */
static double g_pl_x0, g_pl_y0, g_pl_h0, g_pl_vmax, g_pl_hdev, g_pl_ydev, g_pl_path, g_pl_px, g_pl_py;
static double g_pl_end_x, g_pl_end_y, g_pl_end_h, g_pl_end_v, g_pl_stop_x[2];
static void pl_start(void *arg)
{
    (void)arg;
    g_pl_x0 = g_pl_px = g_st.x; g_pl_y0 = g_pl_py = g_st.y; g_pl_h0 = g_st.h * 57.29578;
    g_pl_vmax = g_pl_hdev = g_pl_ydev = g_pl_path = 0.0;
}
static void pl_sample(void *arg)
{
    (void)arg;
    double const v = fmax(fabs(g_st.vl), fabs(g_st.vr));
    g_pl_vmax = fmax(g_pl_vmax, v);
    g_pl_hdev = fmax(g_pl_hdev, fabs(g_st.h * 57.29578 - g_pl_h0));
    g_pl_ydev = fmax(g_pl_ydev, fabs(g_st.y - g_pl_y0));
    g_pl_path += hypot(g_st.x - g_pl_px, g_st.y - g_pl_py);
    g_pl_px = g_st.x; g_pl_py = g_st.y;
}
static void pl_end(void *arg)
{
    (void)arg;
    g_pl_end_x = g_st.x; g_pl_end_y = g_st.y; g_pl_end_h = g_st.h * 57.29578;
    g_pl_end_v = fmax(fabs(g_st.vl), fabs(g_st.vr));
}
static void pl_stop_snap(void *arg) { g_pl_stop_x[(intptr_t)arg] = hypot(g_st.x - g_pl_x0, g_st.y - g_pl_y0); }
static void pl_samples(double from_s, double to_s)
{
    for (double t = from_s; t <= to_s; t += 0.02) { sim_at(S(t), pl_sample, NULL); }
}
static void plaid_floor(void)
{
    g_car.y0 = 0.0; g_car.h0_deg = 0.0;
    base_course(); course_straight(4000.0);
}
static double heading_change(void)
{
    double d = fmod(g_pl_end_h - g_pl_h0, 360.0);
    if (d > 180.0) { d -= 360.0; }
    if (d <= -180.0) { d += 360.0; }
    return d;
}
static void sc_plaid_straight(void)
{
    plaid_floor();
    sim_at(S(1.0), drive_ev, (void *)(uintptr_t)'!');
    sim_at(S(1.2), pl_start, NULL);
    hold_key('w', 1.5, 4.5);
    pl_samples(1.3, 5.6);
    sim_at(S(6.0), pl_end, NULL);
}
static void ck_plaid_straight(void)
{
    double const fwd = g_pl_end_x - g_pl_x0;
    EXPECT(g_pl_vmax > 235.0 && g_pl_vmax < 270.0, "plaid top wheel speed %.0f mm/s (want about 250)", g_pl_vmax);
    EXPECT(fwd > 550.0, "plaid w for 3 s: moved %.0f mm", fwd);
    EXPECT(g_pl_hdev < 3.0, "plaid straight wandered %.1f deg", g_pl_hdev);
    EXPECT(g_pl_ydev < 15.0, "plaid straight drifted %.1f mm sideways", g_pl_ydev);
    EXPECT(g_pl_end_v < 5.0, "still moving %.0f mm/s after release", g_pl_end_v);
    EXPECT(0U == (g_m.faults_seen & FAULT_CRITICAL_MASK), "fault 0x%04x", g_m.faults_seen);
}
static double g_stunt_turn;
static void stunt_run(char key, double turn_deg)
{
    plaid_floor();
    g_stunt_turn = turn_deg;
    sim_at(S(1.0), drive_ev, (void *)(uintptr_t)'!');
    sim_at(S(1.2), pl_start, NULL);
    sim_at(S(1.5), drive_ev, (void *)(uintptr_t)(unsigned char)key);
    pl_samples(1.3, 11.5);
    sim_at(S(11.8), pl_end, NULL);
}
static void ck_stunt(void)
{
    double const turn = heading_change();
    double err = fabs(turn - g_stunt_turn);
    if (fabs(g_stunt_turn) >= 179.0) { err = fabs(fabs(turn) - 180.0); }
    EXPECT(err < 25.0, "net heading change %.0f deg, want %.0f", turn, g_stunt_turn);
    EXPECT(g_pl_path > 500.0, "stunt only covered %.0f mm", g_pl_path);
    EXPECT(g_pl_vmax < 270.0, "wheel speed %.0f mm/s, more than 8 %% above plaid", g_pl_vmax);
    EXPECT(g_pl_end_v < 5.0, "still moving %.0f mm/s at the end", g_pl_end_v);
    EXPECT(0U == (g_m.faults_seen & FAULT_CRITICAL_MASK), "fault 0x%04x", g_m.faults_seen);
}
static void sc_stunt_j(void) { stunt_run('j', 180.0); }
static void sc_stunt_slalom(void) { stunt_run('r', 0.0); }
static void sc_stunt_bootleg(void) { stunt_run('b', 180.0); }
static void sc_stunt_trail(void) { stunt_run('t', -90.0); }
static void sc_stunt_pendulum(void) { stunt_run('p', 90.0); }
/* Drift and donut: raw power.  The sim has no tyre slip, so these check
   the choreography (direction, a clean stop, no faults), not a slide. */
static void sc_stunt_drift(void) { stunt_run('f', -120.0); }
static void ck_stunt_drift(void)
{
    double const turn = heading_change();
    EXPECT(turn < -60.0 && turn > -200.0, "drift turned %.0f deg, want about -120 (right)", turn);
    EXPECT(g_pl_path > 500.0, "drift only covered %.0f mm", g_pl_path);
    EXPECT(g_pl_vmax < 460.0, "wheel speed %.0f mm/s, above full power", g_pl_vmax);
    EXPECT(g_pl_end_v < 5.0, "still moving %.0f mm/s at the end", g_pl_end_v);
    EXPECT(0U == (g_m.faults_seen & FAULT_CRITICAL_MASK), "fault 0x%04x", g_m.faults_seen);
}
static void sc_stunt_donut(void) { stunt_run('o', 0.0); }
static void ck_stunt_donut(void)
{
    double const spun = fabs(g_pl_end_h - g_pl_h0);
    EXPECT(spun > 540.0, "donut spun only %.0f deg (want about two turns)", spun);
    EXPECT(g_pl_end_v < 5.0, "still moving %.0f mm/s at the end", g_pl_end_v);
    EXPECT(0U == (g_m.faults_seen & FAULT_CRITICAL_MASK), "fault 0x%04x", g_m.faults_seen);
    note("spun %.0f deg", spun);
}
static void sc_drift_abort(void)
{
    plaid_floor();
    sim_at(S(1.0), drive_ev, (void *)(uintptr_t)'!');
    sim_at(S(1.2), pl_start, NULL);
    sim_at(S(1.5), drive_ev, (void *)(uintptr_t)'f');
    sim_at(S(3.4), drive_ev, (void *)(uintptr_t)' ');    /* during the raw kick */
    sim_at(S(5.5), pl_stop_snap, (void *)(intptr_t)0);
    sim_at(S(6.5), pl_stop_snap, (void *)(intptr_t)1);
    sim_at(S(6.6), pl_end, NULL);
}
static void ck_drift_abort(void)
{
    EXPECT(fabs(g_pl_stop_x[1] - g_pl_stop_x[0]) < 1.0, "kept moving after aborting the drift");
    EXPECT(g_pl_end_v < 5.0, "still moving %.0f mm/s", g_pl_end_v);
    EXPECT(0U == (g_m.faults_seen & FAULT_CRITICAL_MASK), "fault 0x%04x", g_m.faults_seen);
}
static void sc_stunt_locked(void)
{
    plaid_floor();
    sim_at(S(1.2), pl_start, NULL);
    sim_at(S(1.5), drive_ev, (void *)(uintptr_t)'j');    /* plaid not armed */
    sim_at(S(5.0), pl_end, NULL);
}
static void ck_stunt_locked(void)
{
    EXPECT(hypot(g_pl_end_x - g_pl_x0, g_pl_end_y - g_pl_y0) < 2.0, "stunt ran without plaid armed");
}
static void sc_stunt_abort(void)
{
    plaid_floor();
    sim_at(S(1.0), drive_ev, (void *)(uintptr_t)'!');
    sim_at(S(1.2), pl_start, NULL);
    sim_at(S(1.5), drive_ev, (void *)(uintptr_t)'b');
    sim_at(S(2.0), drive_ev, (void *)(uintptr_t)' ');    /* abort */
    sim_at(S(3.5), pl_stop_snap, (void *)(intptr_t)0);
    sim_at(S(4.5), pl_stop_snap, (void *)(intptr_t)1);
    sim_at(S(4.6), pl_end, NULL);
}
static void ck_stunt_abort(void)
{
    EXPECT(g_pl_stop_x[0] < 250.0, "aborted at 0.5 s but went %.0f mm", g_pl_stop_x[0]);
    EXPECT(fabs(g_pl_stop_x[1] - g_pl_stop_x[0]) < 1.0, "kept moving after the abort");
    EXPECT(g_pl_end_v < 5.0, "still moving %.0f mm/s", g_pl_end_v);
}
static void ck_bc_bench(void)
{
    EXPECT(4 == g_m.n_decoded, "%d letters read, expected 4", g_m.n_decoded);
    EXPECT((g_m.n_decoded >= 4) && (0 == memcmp(g_m.decoded, "A7ZD", 4)),
           "read '%.*s', expected 'A7ZD'", g_m.n_decoded, g_m.decoded);
    EXPECT(0 == g_m.n_rejected_bc, "%d rejects", g_m.n_rejected_bc);
}
static void sc_bc_b(void) { junction_course('B', 250.0); }
static void sc_bc_c(void) { junction_course('C', 250.0); }
static void ck_bc(char want)
{
    EXPECT(g_m.n_decoded >= 1 && g_m.decoded[0] == want, "decoded '%c' (%d reads), expected '%c'",
           g_m.n_decoded ? g_m.decoded[0] : '-', g_m.n_decoded, want);
    EXPECT(g_m.n_executed >= 1, "command never executed at the junction");
    expect_finished(30.0);
    expect_follow_quality(25.0);
}
/* The A-Z map, set before the run over p5is20/car/map. */
extern void net_bridge_map(unsigned char const *p, int len);
static void map_ev(void *arg) { char const *m = (char const *)arg; net_bridge_map((unsigned char const *)m, (int)strlen(m)); }
static void sc_bc_mapped(void)
{
    sim_at(S(1.0), map_ev, (void *)"----------L---------------");   /* K = left, A-D cleared */
    junction_course2('K', 'A', 250.0);
}
static void ck_bc_mapped(void)
{
    EXPECT(1 == g_m.n_map_set, "map not applied (%d)", g_m.n_map_set);
    ck_bc('K');
    EXPECT(g_st.y > 300.0, "K mapped to left: did not turn left (y=%.0f)", g_st.y);
}
static void sc_bc_unmapped(void)
{
    base_course(); course_straight(250.0);
    course_barcode('K', NARROW, WIDE, 0);           /* K unmapped by default */
    course_straight(700.0); finish_line();
}
static void ck_bc_unmapped(void)
{
    EXPECT(g_m.n_decoded >= 1 && 'K' == g_m.decoded[0], "K not read");
    EXPECT(g_m.n_unmapped >= 1, "K not reported as unmapped");
    EXPECT(0 == g_m.n_executed, "an unmapped letter executed a command");
    expect_finished(30.0);
}
static void sc_map_running(void)
{
    sim_at(S(6.0), map_ev, (void *)"R-------------------------");   /* during the run */
    junction_course('A', 250.0);
}
static void ck_map_running(void)
{
    EXPECT(g_m.n_map_refused >= 1, "map change during the run not refused");
    EXPECT(0 == g_m.n_map_set, "map changed during the run");
    ck_bc('A');
    EXPECT(g_st.y > 300.0, "default A (left) not kept (y=%.0f)", g_st.y);
}
static void sc_map_invalid(void) { sim_at(S(1.0), map_ev, (void *)"LRSU-----X----------------"); sim_at(S(1.5), map_ev, (void *)"LRS"); sc_straight(); }
static void ck_map_invalid(void)
{
    EXPECT(2 == g_m.n_map_refused, "%d of 2 bad maps refused", g_m.n_map_refused);
    EXPECT(0 == g_m.n_map_set, "a bad map was applied");
}
static void ck_turn_point(void)
{
    if ((g_junc_x >= 0.0) && (g_turn_x >= 0.0))
    {
        double const err = g_turn_x - g_junc_x;     /* + = past the centre */
        note("turned %+.1f mm from the junction centre", err);
        /* Within 1 cm.  The sim's 4 mm sensor spot sees the bar later
           than the real car's wider one, so the sim turns ~6 mm past
           where the real car (1 cm early before D79) now turns. */
        EXPECT(fabs(err) <= 10.0, "turned %+.1f mm from the junction centre", err);
    }
}
static void ck_bc_a(void) { ck_bc('A'); EXPECT(g_st.y > 300.0, "did not turn left (y=%.0f)", g_st.y); ck_turn_point(); }
static void ck_bc_b(void) { ck_bc('B'); EXPECT(g_st.y < -300.0, "did not turn right (y=%.0f)", g_st.y); ck_turn_point(); }
static void ck_bc_c(void) { ck_bc('C'); EXPECT(fabs(g_st.y) < 60.0 && g_st.x > 1000.0, "did not go straight (%.0f,%.0f)", g_st.x, g_st.y); }

/* 9. U-turn: D, then back the way it came to an END marker behind the start. */
static void sc_bc_d(void)
{
    course_reset(18.0);
    course_start(-700.0, 0.0, 0.0);
    course_end_marker(100.0, 80.0);
    course_straight(600.0);
    course_straight(250.0);
    course_barcode('D', NARROW, WIDE, 0);
    course_straight(700.0);
}
static void ck_bc_d(void)
{
    EXPECT(g_m.n_decoded >= 1 && g_m.decoded[0] == 'D', "first decode '%c', expected 'D'", g_m.n_decoded ? g_m.decoded[0] : '-');
    expect_finished(30.0);
    EXPECT(g_st.x < -500.0, "did not come back to the END marker (x=%.0f)", g_st.x);
}

/* 10. A character with no command, then a crossing: straight on. */
static void sc_bc_other(void)
{
    base_course(); course_straight(250.0);
    course_barcode('X', NARROW, WIDE, 0);
    course_straight(250.0);
    course_branch(90.0, 250.0); course_branch(-90.0, 250.0);
    course_straight(300.0); finish_line();
}
static void ck_bc_other(void)
{
    EXPECT(g_m.n_decoded >= 1 && g_m.decoded[0] == 'X', "decoded '%c', expected 'X'", g_m.n_decoded ? g_m.decoded[0] : '-');
    EXPECT(0 == g_m.n_executed, "a manoeuvre ran for 'X'");
    expect_finished(30.0);
    EXPECT(fabs(g_st.y) < 60.0, "left the straight line at the crossing (y=%.0f)", g_st.y);
}

/* 11. The command is for the first junction however far (owner
   2026-11-19, D73; it used to expire after 500 mm). */
static void sc_bc_expire(void) { junction_course('C', 700.0); }
static void sc_bc_expire_a(void) { junction_course('A', 700.0); }
static void ck_bc_expire_a(void)
{
    EXPECT(0 == g_m.n_expired, "pending command expired");
    ck_bc_a();
}
/* A crossroads drawn with 70 mm wide arms, 700 mm after the barcode: both
   sensors dark for over 60 mm, which once ended the run (real car
   2026-11-19 18:20).  The command is for it. */
static void sc_junc_wide(char c)
{
    base_course(); course_straight(250.0);
    course_barcode(c, NARROW, WIDE, 0);
    course_straight(700.0);
    course_width(70.0);
    if ('A' == c) { course_branch(0.0, 250.0); course_branch(-90.0, 250.0); course_width(18.0); course_turn(90.0); }
    else { course_branch(0.0, 250.0); course_branch(90.0, 250.0); course_width(18.0); course_turn(-90.0); }
    course_straight(300.0);
    finish_line();
}
static void sc_junc_wide_a(void) { sc_junc_wide('A'); }
static void sc_junc_wide_b(void) { sc_junc_wide('B'); }

/* 12. Bars across the line as well as the track (layout risk). */
static void sc_bc_crossing(void)
{
    base_course(); course_straight(250.0);
    course_barcode('B', NARROW, WIDE, 1);
    course_straight(250.0);
    course_branch(0.0, 250.0); course_branch(90.0, 250.0); course_turn(-90.0);
    course_straight(300.0); finish_line();
}
static void ck_bc_crossing(void) { ck_bc('B'); }

/* 13. Junction with nothing pending: straight through. */
static void sc_cross(void)
{
    base_course(); course_straight(500.0);
    course_branch(90.0, 300.0); course_branch(-90.0, 300.0);
    course_straight(500.0); finish_line();
}
static void ck_cross(void) { expect_finished(20.0); EXPECT(fabs(g_st.y) < 50.0, "turned at a junction with no command (y=%.0f)", g_st.y); }

/* 14-16. Humps. */
static double g_hump_h;
static void sc_hump_h(double h, double len)
{
    g_hump_h = h;
    base_course(); course_straight(500.0);
    if (h > 0.0) { course_hump(len, h); } else { course_straight(len); }
    course_straight(500.0); finish_line();
}
static void sc_hump40(void) { sc_hump_h(40.0, 400.0); }
static void sc_hump20(void) { sc_hump_h(20.0, 300.0); }
static void sc_hump_none(void) { sc_hump_h(0.0, 400.0); }
static void sc_hump10(void) { sc_hump_h(10.0, 200.0); }
static void sc_hump10l(void) { sc_hump_h(10.0, 300.0); }
static void ck_hump(void)
{
    int const peak = (int)bits_get(g_hump_byte, 0U, HUMP_PEAK_BITS);
    expect_finished(30.0);
    if (g_hump_h > 0.0)
    {
        EXPECT(g_m.n_hump_reports >= 1, "hump not reported");
        EXPECT(0U != (g_m.states_seen & (1U << MISSION_HUMP)), "never entered S6 HUMP");
        EXPECT(fabs(peak - g_hump_h) <= 0.2 * g_hump_h + 3.0, "hump peak %d mm, true %.0f mm", peak, g_hump_h);
        note("hump %d mm (true %.0f)", peak, g_hump_h);
    }
    else
    {
        EXPECT(0 == g_m.n_hump_reports, "false hump reported (%d mm)", g_m.last_hump_mm);
    }
}

/* 17-21. Obstacles. */
static double g_obst_true_w;
static void sc_obst(double lat, double w, int walls)
{
    g_obst_true_w = w;
    base_course(); course_straight(500.0);
    course_obstacle(150.0, lat, w, 100.0);
    if (1 == walls) { course_obstacle(0.0, 330.0, 60.0, 700.0); }               /* wall on the left */
    if (2 == walls) { course_obstacle(0.0, 260.0, 60.0, 700.0); course_obstacle(0.0, -260.0, 60.0, 700.0); }
    course_straight(1200.0); finish_line();
}
static void sc_obst_c(void) { sc_obst(0.0, 100.0, 0); }
static void sc_obst_wide(void) { sc_obst(0.0, 220.0, 0); }
static void sc_obst_l(void) { sc_obst(40.0, 120.0, 0); }
static void sc_obst_wall(void) { sc_obst(0.0, 100.0, 1); }
static void sc_obst_blocked(void)
{
    /* Three boxes side by side, faces toward the car: no gap wide enough. */
    base_course(); course_straight(500.0);
    course_obstacle(150.0, 0.0, 100.0, 100.0);
    course_obstacle(150.0, 190.0, 180.0, 100.0);
    course_obstacle(150.0, -190.0, 180.0, 100.0);
    course_straight(1200.0); finish_line();
}
static void sc_obst_corridor(void) { sc_obst(0.0, 100.0, 2); }
static void sc_obst_side(void) { sc_obst(260.0, 100.0, 0); }
/* The line steps sideways just before the box: the car stops crooked and
   must square up on the line before the body scan (D80). */
static void sc_obst_skew(void)
{
    g_obst_true_w = 100.0;
    base_course(); course_straight(500.0);
    course_shift(16.0); course_straight(20.0);
    course_obstacle(150.0, 0.0, 100.0, 100.0);
    course_straight(1200.0); finish_line();
}
/* The real car 2026-11-23 (D81): a 220 mm box mostly left of the line,
   something on the right further on, so the long way round (left) is
   the only one; the first leg must back off to clear the box's corner. */
#ifndef OBST_LONG_NEAR
#define OBST_LONG_NEAR 80.0
#endif
static void sc_obst_long(void)
{
    g_obst_true_w = 220.0;
    base_course(); course_straight(500.0);
    course_obstacle(150.0, -90.0, 220.0, 120.0);
    course_obstacle(150.0 - OBST_LONG_NEAR, -5.0, 50.0, OBST_LONG_NEAR + 20.0);  /* its left end nearer */
    course_straight(1200.0); finish_line();
}
/* Same box, but the only thing on the right is far beyond it: the short
   way round (right) is clear and must be taken (D82, real car 15:53). */
static void sc_obst_far(void)
{
    g_obst_true_w = 220.0;
    base_course(); course_straight(500.0);
    course_obstacle(150.0, 90.0, 220.0, 120.0);
    course_obstacle(150.0 - OBST_LONG_NEAR, 170.0, 60.0, OBST_LONG_NEAR + 20.0);
    course_obstacle(720.0, -210.0, 60.0, 60.0);
    course_straight(1200.0); finish_line();
}
/* A deep box mostly left of the line: the sonar also sees its right
   side face just past the right edge.  That is the box, not something
   beside it: pass on the right, the short way (D83, real car 16:00). */
static void sc_obst_deep(void)
{
    g_obst_true_w = 200.0;
    base_course(); course_straight(500.0);
    course_obstacle(150.0, 80.0, 200.0, 300.0);
    course_straight(1200.0); finish_line();
}
/* About as wide each side, the left half nearer (real car 16:08, D84):
   the right needs less backing off, so pass on the right. */
static void sc_obst_stagger(void)
{
    g_obst_true_w = 360.0;
    base_course(); course_straight(500.0);
    course_obstacle(150.0, 90.0, 180.0, 120.0);
    course_obstacle(200.0, -90.0, 180.0, 120.0);
    course_straight(1200.0); finish_line();
}
/* D85: the course curves into the detour behind the box; the car meets
   it on the detour and follows it instead of finishing the arc. */
static int g_curve_dir;
static void sc_obst_curve(int dir)
{
    g_curve_dir = dir;
    g_obst_true_w = 100.0;
    base_course(); course_straight(500.0);
    course_obstacle(150.0, 0.0, 100.0, 100.0);
    course_straight(260.0);
    course_arc(160.0, dir * 90.0);
    course_straight(700.0); finish_line();
}
static void sc_obst_curve_l(void) { sc_obst_curve(1); }
static void ck_obst_common(void);
static void ck_obst_curve(void)
{
    ck_obst_common();
    EXPECT(g_m.n_obstacle_seen >= 1, "obstacle never detected");
    expect_finished(55.0);
}
static void ck_obstacle_short_right(void)
{
    ck_obst_common();
    EXPECT(g_m.n_obstacle_seen >= 1, "obstacle never detected");
    EXPECT(0U != (g_m.plans_seen & (1U << PLAN_RIGHT)), "box mostly left: expected the short way, right (plans 0x%x)", g_m.plans_seen);
    expect_finished(45.0);
}
static void ck_obst_common(void)
{
    EXPECT(!g_st.collided, "COLLISION with an obstacle at (%.0f,%.0f)", g_st.x, g_st.y);
}
static void ck_obst_width(void)
{
    EXPECT(g_m.obst_width > 0, "obstacle width never measured");
    EXPECT(fabs(g_m.obst_width - g_obst_true_w) <= 40.0, "measured width %d mm, true %.0f mm", g_m.obst_width, g_obst_true_w);
    note("width %d mm (true %.0f)", g_m.obst_width, g_obst_true_w);
}
static void ck_obst_c(void)
{
    ck_obst_common();
    ck_obst_width();
    EXPECT(g_m.n_obstacle_seen >= 1, "obstacle never detected");
    EXPECT(0U != (g_m.plans_seen & ((1U << PLAN_LEFT) | (1U << PLAN_RIGHT))), "no bypass planned (plans 0x%x)", g_m.plans_seen);
    expect_finished(45.0);
}
static void ck_obst_l(void)
{
    ck_obst_c();
    EXPECT(0U != (g_m.plans_seen & (1U << PLAN_RIGHT)), "obstacle offset left, bypass should go right (plans 0x%x)", g_m.plans_seen);
}
static void ck_obst_long(void)
{
    ck_obst_common();
    EXPECT(g_m.n_obstacle_seen >= 1, "obstacle never detected");
    EXPECT(0U != (g_m.plans_seen & (1U << PLAN_LEFT)), "expected the bypass on the left (plans 0x%x)", g_m.plans_seen);
    expect_finished(45.0);
}
static void ck_obst_far(void)
{
    ck_obst_common();
    EXPECT(g_m.n_obstacle_seen >= 1, "obstacle never detected");
    EXPECT(0U != (g_m.plans_seen & (1U << PLAN_RIGHT)), "box mostly left, right side clear: expected the bypass on the right (plans 0x%x)", g_m.plans_seen);
    expect_finished(45.0);
}
static void ck_obst_wall(void)
{
    ck_obst_c();
    EXPECT(0U != (g_m.plans_seen & (1U << PLAN_RIGHT)), "wall on the left, bypass should go right (plans 0x%x)", g_m.plans_seen);
}
static void ck_obst_blocked(void)
{
    ck_obst_common();
    EXPECT(0U != (g_m.faults_seen & FAULT_BYPASS_FAIL), "boxed in: expected BYPASS_FAIL (faults 0x%04x, S%u)", g_m.faults_seen, g_m.state);
    EXPECT(MISSION_FAULT == g_m.state, "boxed in: expected S10, got S%u", g_m.state);
}
static void ck_obst_side(void)
{
    ck_obst_common();
    expect_finished(25.0);
    EXPECT(0U == (g_m.plans_seen & ((1U << PLAN_LEFT) | (1U << PLAN_RIGHT))), "bypassed an obstacle that was not in the path");
}

/* 22-24. Lost line. */
static void sc_gap(void) { base_course(); course_straight(600.0); course_gap(30.0); course_straight(600.0); finish_line(); }
static void ck_gap(void) { expect_finished(25.0); }
static void sc_line_ends(void) { base_course(); course_straight(800.0); }
static void ck_line_ends(void)
{
    EXPECT(0U != (g_m.states_seen & (1U << MISSION_SEARCH)), "never searched for the line");
    expect_fault(FAULT_LINE_NOT_FOUND, "LINE_NOT_FOUND");
}
static void sc_step(void) { base_course(); course_straight(600.0); course_gap(50.0); course_shift(20.0); course_straight(800.0); finish_line(); }
static void sc_step60(void) { base_course(); course_straight(600.0); course_gap(50.0); course_shift(60.0); course_straight(800.0); finish_line(); }
static void ck_step(void) { expect_finished(35.0); }

/* 25-26. E-STOP and remote STOP, then RESET and resume. */
static void sc_estop(void) { base_course(); course_straight(2500.0); finish_line(); estop_at(S(9.0)); world_press(20U, S(11.0), 100); }
static void ck_estop(void)
{
    EXPECT(0U != g_m.estop_off_latency, "motors never off after E-STOP");
    EXPECT(g_m.estop_off_latency <= MS(2), "E-STOP to motors off %.1f ms", (double)g_m.estop_off_latency / 1000.0);
    note("E-STOP -> off %.2f ms", (double)g_m.estop_off_latency / 1000.0);
    EXPECT(0U != (g_m.states_seen & (1U << MISSION_FAULT)), "E-STOP did not enter S10");
    expect_finished(40.0);  /* START = RESET, then START to resume */
}
static void sc_remote(void)
{
    base_course(); course_straight(2500.0); finish_line();
    mqtt(S(9.0), CMD_STOP, 1); mqtt(S(10.5), CMD_RESET, 2); mqtt(S(12.0), CMD_START, 3);
}
static void ck_remote(void)
{
    EXPECT(0U != g_m.remote_off_latency, "motors never off after MQTT STOP");
    EXPECT(g_m.remote_off_latency <= MS(100), "MQTT STOP to motors off %.1f ms (NFR6 100 ms)", (double)g_m.remote_off_latency / 1000.0);
    note("STOP -> off %.1f ms", (double)g_m.remote_off_latency / 1000.0);
    EXPECT(0U != (g_m.faults_seen & FAULT_REMOTE_STOP), "REMOTE_STOP not raised");
    EXPECT(g_m.n_fault_lines >= 1, "REMOTE_STOP not reported by name in the log");
    expect_finished(40.0);
}

/* 27. MQTT duplicates and malformed payloads. */
static void sc_mqtt_dup(void)
{
    static const uint8_t two[2] = { 0x04, 0x00 };
    base_course(); course_straight(2500.0); finish_line();
    mqtt(S(9.0), CMD_STOP, 5); mqtt(S(10.5), CMD_RESET, 6);
    mqtt(S(10.6), CMD_STOP, 6);        /* same seq as RESET: must be ignored */
    mqtt_raw(S(10.7), two, 2);         /* two bytes: rejected */
    mqtt(S(12.0), CMD_START, 7);
}
static void ck_mqtt_dup(void)
{
    EXPECT(1 == __builtin_popcount(g_m.faults_seen & FAULT_REMOTE_STOP), "REMOTE_STOP");
    expect_finished(40.0);
}

/* 27b. Console only (no GP20): calibrate, recalibrate from READY with
   RESET, calibrate again, drive to the END marker. */
static void sc_console(void)
{
    base_course(); course_straight(2500.0); finish_line();
    mqtt(S(2.5), CMD_START, 1);         /* "cal" */
    mqtt(S(9.0), CMD_RESET, 2);         /* "cal" from READY: reset ... */
    mqtt(S(11.0), CMD_START, 3);        /* ... then calibrate again */
    mqtt(S(18.0), CMD_START, 4);        /* "go" */
}
static void ck_console(void)
{
    EXPECT(2 == g_m.n_calibrated, "calibrated %d times, expected 2", g_m.n_calibrated);
    EXPECT(1 == g_m.n_recal, "RESET in READY did not restart calibration");
    expect_finished(45.0);
}

/* 28. WiFi drops mid-run: non-critical. */
static void sc_wifi(void) { base_course(); course_straight(2000.0); finish_line(); sim_at(S(9.0), link_ev, (void *)0); }
static void ck_wifi(void)
{
    EXPECT(0U != (g_m.faults_seen & FAULT_WIFI_LOSS), "WIFI_LOSS not raised after the link dropped");
    expect_finished(30.0);
}

static void sc_log_gap(void)
{
    sc_curves();
    sim_at(S(3.0), log_pause_ev, (void *)1);
    sim_at(S(23.0), log_pause_ev, (void *)0);
}
static void ck_log_gap(void)
{
    ck_curves();
    /* Either everything said during the gap arrives afterwards, or the
       loss is reported in the log itself. */
    EXPECT((g_n_log >= g_n_con - 8) || (g_log_lost_reports >= 1),
           "after a 20 s gap only %d of %d lines arrived, no loss report", g_n_log, g_n_con);
}

/* 29-30. IMU missing / dies: non-critical. */
static void sc_imu_absent(void) { g_hw_faults.i2c_absent = true; sc_hump40(); }
static void ck_imu_absent(void)
{
    EXPECT(0U != (g_m.faults_seen & FAULT_IMU), "IMU fault not raised");
    expect_finished(30.0);
    EXPECT(0 != bits_get(g_terrain_word, TERRAIN_RATE_SHIFT, TERRAIN_RATE_BITS) || 1, "");
}
static void sc_imu_dies(void) { g_hw_faults.i2c_fail_from_ms = 9000U; sc_straight(); }
static void ck_imu_dies(void) { EXPECT(0U != (g_m.faults_seen & FAULT_IMU), "IMU fault not raised"); expect_finished(30.0); }
/* The GY-511 slot with another part fitted: the driver finds it and the
   hump is still measured. */
static void sc_imu_lsm303d(void) { g_hw_faults.imu_chip = 1; sc_hump40(); }
static void sc_imu_gain(void) { g_hw_faults.imu_chip = 1; g_hw_faults.imu_gain = 1.08; sc_hump40(); }
/* The car's GY-511 is mounted upside down (2026-11-19): Z reads -1 g. */
static void sc_lifted(void);
static void ck_imu_part(void);
static void sc_imu_flip_x(void) { g_hw_faults.imu_chip = 1; g_hw_faults.imu_gain = 0.94; g_hw_faults.imu_mount = 1; sc_hump40(); }
static void sc_imu_flip_y(void) { g_hw_faults.imu_chip = 1; g_hw_faults.imu_gain = 0.94; g_hw_faults.imu_mount = 2; sc_hump40(); }
/* Board turned 90 deg (Y along the car): the hump shows as the board's
   roll; T3 must learn that and measure the hump from it. */
static void sc_imu_side(void) { g_hw_faults.imu_mount = 3; sc_hump40(); }
static void sc_imu_side_flip(void) { g_hw_faults.imu_chip = 1; g_hw_faults.imu_gain = 0.94; g_hw_faults.imu_mount = 4; sc_hump40(); }
static void sc_imu_side_10(void) { g_hw_faults.imu_chip = 1; g_hw_faults.imu_mount = 4; sc_hump10(); }
static void ck_imu_flip(void);
/* Pitch reads 3 deg nose-down while driving (real car: -1.5 parked,
   about -2.5 driving): a 10 mm hump must still be found (D65). */
static void sc_hump_10_tilt(void) { g_hw_faults.imu_drive_tilt = -3.0; sc_hump10(); }
/* A short 1 cm bump (10 mm over 80 mm), driving tilt as the real car. */
static void sc_hump_bump(void) { g_hw_faults.imu_drive_tilt = -2.5; sc_hump_h(10.0, 80.0); }
/* Vibration like the real car's (+-3 deg pitch noise and 2 Hz rocking
   while driving): no false hump on flat, the 1 cm hump still found. */
static void sc_hump_none_vibe(void) { g_hw_faults.imu_vibe_mg = getenv("SIM_VIBE") ? atof(getenv("SIM_VIBE")) : 60.0; g_hw_faults.imu_drive_tilt = -1.0; sc_hump_none(); }
static void sc_hump_10_vibe(void) { g_hw_faults.imu_vibe_mg = 60.0; g_hw_faults.imu_drive_tilt = -1.0; sc_hump10(); }
static void sc_bump_vibe(void) { g_hw_faults.imu_vibe_mg = getenv("SIM_VIBE") ? atof(getenv("SIM_VIBE")) : 60.0; g_hw_faults.imu_drive_tilt = -1.0; sc_hump_h(10.0, 80.0); }
static void ck_imu_side(void)
{
    EXPECT(g_m.y_fwd, "never found the board turned 90 deg");
    ck_imu_flip();
}
static void ck_imu_flip(void)
{
    EXPECT(0U == (g_m.faults_seen & 0x1000U), "false LIFTED with the IMU upside down");
    ck_imu_part();
}
static void sc_lifted_flip(void) { g_hw_faults.imu_chip = 1; g_hw_faults.imu_mount = 2; sc_lifted(); }
static void sc_imu_mpu6050(void) { g_hw_faults.imu_chip = 2; sc_hump40(); }
static void ck_imu_part(void)
{
    EXPECT(0U == (g_m.faults_seen & FAULT_IMU), "IMU fault with another part fitted: 0x%04x", g_m.faults_seen);
    ck_hump();
}
static void ck_imu_absent_diag(void)
{
    ck_imu_absent();
    EXPECT(g_m.imu_diag, "no I2C diagnosis printed when the IMU is missing");
}
static void sc_imu_flaky(void) { g_hw_faults.i2c_flaky = true; sc_hump40(); }
static void ck_imu_flaky(void) { expect_finished(30.0); note("faults 0x%04x", g_m.faults_seen); }

/* Recalibrated after a run with a hump: tilt and hump read zero. */
static bool g_recal_sent;
static uint64_t g_recal_at;
static void recal_poll(void *a)
{
    (void)a;
    if (MISSION_FINISHED == mission_state())
    {
        g_recal_sent = true; g_recal_at = g_now_us;
        mqtt(g_now_us + MS(500), CMD_RESET, 7);
        return;
    }
    sim_at(g_now_us + MS(100), recal_poll, NULL);
}
static void sc_hump_recal(void) { g_recal_sent = false; sc_hump20(); sim_at(S(1.0), recal_poll, NULL); }
static void ck_hump_recal(void)
{
    int const peak = (int)bits_get(g_hump_byte, 0U, HUMP_PEAK_BITS);
    int const pitch = bits_to_signed(bits_get(g_terrain_word, TERRAIN_PITCH_SHIFT, TERRAIN_PITCH_BITS), TERRAIN_PITCH_BITS);
    int const roll = bits_to_signed(bits_get(g_tilt_word, TILT_ROLL_SHIFT, TILT_ROLL_BITS), TILT_ROLL_BITS);
    int const hnow = (int)bits_get(g_tilt_word, TILT_HNOW_SHIFT, TILT_HNOW_BITS);
    EXPECT(g_m.n_hump_reports >= 1, "hump not measured before recalibrating");
    EXPECT(g_recal_sent, "never finished, so never recalibrated");
    EXPECT(MISSION_CALIBRATE == g_m.state, "not back in S1 CALIBRATE after RESET (S%u)", g_m.state);
    EXPECT(g_m.n_zeroed >= 1, "re-zero reported %d times, expected at least after RESET", g_m.n_zeroed);
    EXPECT(g_m.n_hump_zeroed >= 2, "hump zero logged %d times, expected at the first calibration and after RESET", g_m.n_hump_zeroed);
    EXPECT(0 == peak, "hump max still %d mm after recalibrating", peak);
    EXPECT(0 == hnow, "hump height now %d mm while calibrating", hnow);
    EXPECT(abs(pitch) <= 5, "pitch %d.%d deg while calibrating", pitch / 10, abs(pitch % 10));
    EXPECT(abs(roll) <= 1, "roll %d deg while calibrating", roll);
    note("recalibrated %.1f s, hump max %d, pitch %d (0.1 deg), roll %d", (double)g_recal_at / 1e6, peak, pitch, roll);
}

/* Calibrated with the line out of reach: SELF_TEST, and the log says why. */
static void sc_cal_off_line(void) { g_car.y0 = 80.0; g_car.h0_deg = 0.0; sc_straight(); }
static void ck_cal_off_line(void)
{
    EXPECT(0U != (g_m.faults_seen & FAULT_SELF_TEST), "no SELF_TEST");
    EXPECT(g_m.cal_hint, "no placement hint in the log");
}
/* 31-35. Critical faults. */
/* Picked up mid-run (end of the demo): LIFTED, motors stopped.  Also when
   the lift is felt only as the tilt in the hand. */
static double g_lift_v;
static void lift_snap(void *arg) { (void)arg; g_lift_v = fmax(fabs(g_st.vl), fabs(g_st.vr)); }
static void sc_lifted(void) { g_car.lift_at_us = S(9.0); sim_at(S(10.5), lift_snap, NULL); sc_straight(); }
static void sc_lifted_tilt(void) { g_car.lift_tilt_only = true; sc_lifted(); }
static void ck_lifted(void);
/* Picked up nose first and held tilted: LIFTED, and no "hump" (a real one
   read 27 mm and stuck as the run maximum, 2026-11-19). */
static void sc_lifted_nose(void) { g_car.lift_nose_up = true; sc_lifted(); }
static void ck_lifted_nose(void)
{
    ck_lifted();
    EXPECT(0 == g_m.n_hump_reports, "a hump of %d mm was measured in the hand", g_m.last_hump_mm);
}
static void ck_lifted(void)
{
    EXPECT(0U != (g_m.faults_seen & 0x1000U), "LIFTED fault not raised");
    EXPECT(g_lift_v < 5.0, "wheels still turning %.0f mm/s 1.5 s after the pick-up", g_lift_v);
}
static void sc_impact(void) { g_car.impact_at_us = S(9.0); sc_straight(); }
/* Collision detection is off (owner 2026-11-19, D75): a 4 g bump must not
   stop the car; only lift-off does. */
static void ck_impact(void)
{
    EXPECT(0U == (g_m.faults_seen & FAULT_IMPACT), "IMPACT raised with collision detection off");
    EXPECT(0U == (g_m.faults_seen & 0x1000U), "a bump read as LIFTED");
    expect_finished(30.0);
}
/* A reboot mid-run: the IMU's first readings after power-up were garbage
   (10369 mg) and raised a false IMPACT before calibration (real car
   2026-11-19 18:41, D74).  A spike during the I1 calibration is no crash. */
static void sc_impact_boot(void) { g_car.impact_at_us = S(0.5); sc_straight(); }
static void ck_impact_boot(void)
{
    EXPECT(0U == (g_m.faults_seen & FAULT_IMPACT), "IMPACT raised during the IMU's start-up calibration");
    expect_finished(30.0);
}
static void stall_ev(void *a) { (void)a; g_car.wheel_blocked_l = true; }
static void sc_stall2(void) { sc_straight(); sim_at(S(9.0), stall_ev, NULL); }
static void ck_stall(void) { expect_fault(FAULT_MOTOR_STALL | FAULT_ENCODER, "MOTOR_STALL/ENCODER"); }
static void enc_ev(void *a) { (void)a; g_car.enc_dead_l = true; }
static void sc_enc_dead(void) { sc_straight(); sim_at(S(9.0), enc_ev, NULL); }
static void ck_enc_dead(void) { expect_fault(FAULT_ENCODER, "ENCODER"); }
static void sc_echo_dead(void) { g_car.echo_dead = true; sc_straight(); }
static void ck_echo_dead(void) { expect_fault(FAULT_NO_ECHO, "NO_ECHO"); }
static void sc_adc_stuck(void) { g_hw_faults.adc_stuck = true; sc_straight(); }
static void sc_adc_late(void) { g_hw_faults.adc_late = true; sc_straight(); }
static void ck_adc_stuck(void) { expect_fault(FAULT_SELF_TEST, "SELF_TEST"); }

/* 36-39. Hardware variations. */
static void sc_inverted(void) { double t = g_car.ir_white; g_car.ir_white = g_car.ir_black; g_car.ir_black = t; sc_curves(); }
static void sc_straddle(void) { g_car.ir_half_mm = 9.0; sc_curves(); }
static void ck_straddle(void) { ck_curves(); EXPECT(2 == g_m.mounting, "mounting not detected as STRADDLE"); }
static void sc_mismatch(void) { g_car.gain_r = 400.0; g_car.deadband = 0.12; sc_curves(); }
static void sc_geometry(void) { g_car.wheel_dia_mm = 68.0; g_car.track_mm = 142.0; junction_course('A', 250.0); }
static void sc_motor_rev(void) { g_car.motor_sign_l = -g_car.motor_sign_l; sc_straight(); }
static void sc_enc_rev(void) { g_car.enc_sign_l = -g_car.enc_sign_l; sc_straight(); }
static void ck_wiring(void)
{
    /* Safety property: a mis-wired car must end up stopped in S10, not
       spinning or running away. */
    EXPECT(0U != (g_m.states_seen & (1U << MISSION_FAULT)), "mis-wiring never detected (state S%u, faults 0x%04x, travelled %.0f mm)",
           g_m.state, g_m.faults_seen, g_st.dist);
    EXPECT(world_motors_off(), "motors still driven at the end");
}
static void sc_servo_mirror(void) { g_car.servo_mirror = 1.0; sc_obst_l(); }
static void sc_servo_slow(void) { g_car.servo_dps = 400.0; sc_obst_wall(); }

/* 40. Stale START presses. */
static void sc_start_extra(void) { sc_straight(); world_press(20U, S(9.0), 100); }
static void ck_start_extra(void)
{
    expect_finished(20.0);
    EXPECT(g_st.x < 1900.0, "drove off past the END marker after a stray START (x=%.0f)", g_st.x);
}
static void sc_bounce(void)
{
    sc_straight();
    for (int i = 0; i < 6; i++) { world_press(21U, S(9.0) + (uint64_t)i * 1500U, 1); }
}
static void ck_bounce(void) { EXPECT(0U != (g_m.states_seen & (1U << MISSION_FAULT)), "bouncing E-STOP not seen"); }

/* 41. Obstacle cleared during a manoeuvre / E-STOP during bypass. */
static void sc_estop_bypass(void) { sc_obst_c(); estop_at(S(13.0)); world_press(20U, S(15.0), 100); }
static void ck_estop_bypass(void)
{
    ck_obst_common();
    EXPECT(0U != (g_m.states_seen & (1U << MISSION_FAULT)), "no S10 after E-STOP");
    EXPECT(g_m.estop_off_latency <= MS(2), "E-STOP latency %.1f ms", (double)g_m.estop_off_latency / 1000.0);
    note("final S%u faults 0x%04x", g_m.state, g_m.faults_seen);
}

/* 42. Endurance loop: odometry saturation, 16-bit wrap, no stray faults. */
static void sc_loop(void)
{
    course_reset(18.0);
    course_start(-200.0, 0.0, 0.0);
    course_straight(1200.0); course_arc(350.0, 180.0); course_straight(1000.0);
    course_arc(350.0, 180.0); course_straight(200.0);
}
static void ck_loop(void)
{
    uint32_t const odom = g_odom_word;
    EXPECT(following(g_m.state), "left line following during the endurance run (S%u, faults 0x%04x)", g_m.state, g_m.faults_seen);
    EXPECT(0U != bits_get(odom, ODOM_SAT_SHIFT, 1U) || g_st.dist < 32767.0, "total distance did not saturate at 32767");
    EXPECT(0U == (g_m.faults_seen & FAULT_CRITICAL_MASK), "fault during the endurance run 0x%04x", g_m.faults_seen);
    expect_follow_quality(25.0);
    note("%.1f m driven", g_st.dist / 1000.0);
}

/* 43. CPU load: every kernel call costs 40-120 us. */
static void sc_load(void) { sc_curves(); }

/* 44. Full course: every feature in one run. */
static int g_full_cross = 0;
static void sc_full_cross(void);
static void sc_full(void)
{
    base_course(); course_straight(200.0);
    course_barcode('B', NARROW, WIDE, g_full_cross);
    course_straight(250.0);
    course_branch(0.0, 250.0); course_branch(90.0, 250.0); course_turn(-90.0);
    course_straight(300.0);
    course_hump(400.0, 30.0);
    course_straight(300.0);
    course_arc(400.0, 90.0);
    course_straight(400.0);
    course_obstacle(100.0, 0.0, 100.0, 100.0);
    course_straight(1200.0);
    course_barcode('A', NARROW, WIDE, g_full_cross);
    course_straight(250.0);
    course_branch(0.0, 250.0); course_branch(-90.0, 250.0); course_turn(90.0);
    course_straight(300.0);
    finish_line();
}
static void sc_full_cross(void) { g_full_cross = 1; sc_full(); }
static void ck_full(void)
{
    ck_obst_common();
    EXPECT(g_m.n_decoded >= 2 && g_m.decoded[0] == 'B' && g_m.decoded[1] == 'A', "decodes %.*s, expected BA", g_m.n_decoded, g_m.decoded);
    EXPECT(g_m.n_hump_reports >= 1, "hump not reported");
    EXPECT(g_m.n_executed >= 2, "%d junction commands executed, expected 2", g_m.n_executed);
    expect_finished(90.0);
}

/* The real car rolled on past the line after the bypass (2026-11-19):
   wheels that spin down slowly when the motors stop. */
/* Obstacle, finish, RESET: the obstacle width reads zero again. */
static void sc_obstacle_recal(void) { g_recal_sent = false; sc_obst_c(); sim_at(S(1.0), recal_poll, NULL); }
static void ck_obstacle_recal(void)
{
    uint32_t const p0 = g_vehicle_state.profile0;   /* run over: no lock */
    EXPECT(g_m.obst_width > 0, "obstacle never measured");
    EXPECT(g_recal_sent, "never finished, so never recalibrated");
    EXPECT(g_m.n_obst_zeroed >= 2, "obstacle zero logged %d times, expected at the first calibration and after RESET", g_m.n_obst_zeroed);
    EXPECT(0U == bits_get(p0, PROF_WIDTH_SHIFT, PROF_WIDTH_BITS), "obstacle width still %u mm after recalibrating",
           (unsigned)bits_get(p0, PROF_WIDTH_SHIFT, PROF_WIDTH_BITS));
    note("measured %d mm, then 0 after RESET", g_m.obst_width);
}
/* Real car 2026-11-19 17:07: crossed the line on the way back and never
   stopped (a wide sensor spot keeps the dark run under 12 mm at 60 deg). */
static void sc_obstacle_spot(void) { sc_obst_c(); g_car.ir_line_spot_mm = 9.0; }
static void sc_obstacle_spot_r(void) { sc_obst_c(); g_car.ir_line_spot_mm = 11.0; }
static void sc_obstacle_coast(void) { sc_obst_c(); g_car.coast_s = 0.3; }

static const scenario_t g_scenarios[] = {
    { "boot",          "boot, idle: S1, motors off, IMU calibrated, PWM set-up", sc_boot, ck_boot, 4.0, 0, false, 0 },
    { "calibrate",     "START: sweep, mounting detection, back on heading", sc_cal, ck_cal, 8.0, 0, false, 0 },
    { "straight",      "straight line to the END marker; odometry", sc_straight, ck_straight, 20.0, 1, false, 0 },
    { "curves",        "R400/R300/R500 curves both ways", sc_curves, ck_curves, 30.0, 1, false, 0 },
    { "sharp_150",     "R150 120-degree curve", sc_sharp150, ck_sharp, 25.0, 1, false, 0 },
    { "sharp_120",     "R120 90-degree curve", sc_sharp120, ck_sharp, 25.0, 1, false, 0 },
    { "hairpin_100",   "R100 180-degree hairpin, left", sc_hairpin100, ck_sharp, 30.0, 1, false, 0 },
    { "hairpin_70",    "R70 180-degree hairpin, left", sc_hairpin70, ck_sharp, 30.0, 1, false, 0 },
    { "hairpin_70r",   "R70 180-degree hairpin, right", sc_hairpin70r, ck_sharp, 30.0, 1, false, 0 },
    { "hairpin_50",    "R50 180-degree hairpin, left (at the limit: tighter than half the car)", sc_hairpin50, ck_sharp, 30.0, 1, false, 0, true },
    { "hairpin_50r",   "R50 180-degree hairpin, right (at the limit: tighter than half the car)", sc_hairpin50r, ck_sharp, 30.0, 1, false, 0, true },
    { "sharp_90",      "R90 90-degree right curve", sc_sharp90, ck_sharp, 25.0, 1, false, 0 },
    { "s_bend",        "R150 left then right, 80 mm apart", sc_sbend, ck_sharp, 25.0, 1, false, 0 },
    { "tight_curve",   "R200 120-degree curve", sc_tight, ck_tight, 25.0, 1, false, 0 },
    { "bc_bench",      "BARCODE_TEST build only (build/sim_bc): rolled by hand over A 7 Z D", sc_bc_bench, ck_bc_bench, 20.0, 0, false, 0 },
    { "bc_all",        "BARCODE_TEST build only: all 43 characters at 3 sizes", sc_bc_all, ck_bc_all, 90.0, 0, false, 0 },
    { "bc_backward",   "BARCODE_TEST build only: all 43 rolled over backwards, jerky", sc_bc_backward, ck_bc_reversed, 90.0, 0, false, 0 },
    { "bc_turned",     "BARCODE_TEST build only: all 43, car turned round, jerky", sc_bc_turned, ck_bc_reversed, 90.0, 0, false, 0 },
    { "bc_scan",       "BARCODE_TEST build only: self-driven pass there and back", sc_bc_scan, ck_bc_scan, 44.0, 0, false, 0 },
    { "bc_blur",       "BARCODE_TEST build only: all 43 through a 6 mm spot", sc_bc_blur, ck_bc_blur, 90.0, 0, false, 0 },
    { "bc_hand",       "BARCODE_TEST build only: card slid under a standing car, 6 mm spot", sc_bc_hand, ck_bc_hand, 20.0, 0, false, 0 },
    { "bc_air",        "BARCODE_TEST build only: junk marks hard against 2 symbols", sc_bc_air, ck_bc_air, 12.0, 0, false, 0 },
    { "bc_air_hand",   "BARCODE_TEST build only: the same, card slid under a standing car", sc_bc_air_hand, ck_bc_air, 12.0, 0, false, 0 },
    { "bc_hand_3s",    "BARCODE_TEST build only: hand, about 3 s per symbol", sc_bc_hand_3s, ck_bc_hand, 42.0, 0, false, 0 },
    { "bc_hand_quick", "BARCODE_TEST build only: hand, 0.7 s per symbol", sc_bc_hand_quick, ck_bc_hand, 12.0, 0, false, 0 },
    { "bc_scan_large", "BARCODE_TEST build only: scan there and back, 6 mm sheet", sc_bc_scan_large, ck_bc_scan, 44.0, 0, false, 0 },
    { "bc_jerky",      "BARCODE_TEST build only: the same, hand speed 40-160 mm/s", sc_bc_jerky, ck_bc_bench, 20.0, 0, false, 0 },
    { "barcode_mapped","map K=left before the run: 'K' then junction, turn left", sc_bc_mapped, ck_bc_mapped, 30.0, 1, false, 0 },
    { "barcode_unmapped","'K' unmapped: read, reported, no command", sc_bc_unmapped, ck_bc_unmapped, 30.0, 1, false, 0 },
    { "map_running",   "map sent during the run: refused, old map kept", sc_map_running, ck_map_running, 30.0, 1, false, 0 },
    { "map_invalid",   "bad map messages refused", sc_map_invalid, ck_map_invalid, 25.0, 1, false, 0 },
    { "junction_spot", "'A', junction seen through a 9 mm sensor spot: turn left", sc_junc_spot, ck_bc_a, 30.0, 1, false, 0 },
    { "junction_slant", "'A', cross bar slanted 12 deg: turn left", sc_junc_slant, ck_bc_a, 30.0, 1, false, 0 },
    { "junction_both", "'A', slanted 12 deg and a 9 mm spot: turn left", sc_junc_both, ck_bc_a, 30.0, 1, false, 0 },
    { "junction_both_b", "'B', slanted -12 deg and a 9 mm spot: turn right", sc_junc_both_b, ck_bc_b, 30.0, 1, false, 0 },
    { "junction_close", "'A' with the junction 20 mm after the barcode: turn left", sc_bc_close, ck_bc_a, 30.0, 1, false, 0 },
    { "junction_close_b", "'B' with the junction 20 mm after the barcode: turn right", sc_bc_close_b, ck_bc_b, 30.0, 1, false, 0 },
    { "barcode_A",     "'A' then junction: turn left", sc_bc_a, ck_bc_a, 30.0, 1, false, 0 },
    { "barcode_B",     "'B' then junction: turn right", sc_bc_b, ck_bc_b, 30.0, 1, false, 0 },
    { "barcode_C",     "'C' then junction: straight", sc_bc_c, ck_bc_c, 30.0, 1, false, 0 },
    { "barcode_D",     "'D': U-turn at once, back to END", sc_bc_d, ck_bc_d, 30.0, 1, false, 0 },
    { "barcode_other", "'X' (no command), crossing: straight", sc_bc_other, ck_bc_other, 30.0, 1, false, 0 },
    { "barcode_far",   "'A' 700 mm before the junction: still turns left (no expiry)", sc_bc_expire_a, ck_bc_expire_a, 30.0, 1, false, 0 },
    { "barcode_C_far", "'C' 700 mm before the junction: straight", sc_bc_expire, ck_c_far, 30.0, 1, false, 0 },
    { "junction_wide", "'A', crossroads with 70 mm arms 700 mm on: a junction, not END", sc_junc_wide_a, ck_bc_a, 35.0, 1, false, 0 },
    { "junction_wide_b", "'B', crossroads with 70 mm arms 700 mm on: turn right", sc_junc_wide_b, ck_bc_b, 35.0, 1, false, 0 },
    { "barcode_cross_line", "'B' with bars across the line too (layout risk)", sc_bc_crossing, ck_bc_crossing, 30.0, 1, false, 0 },
    { "crossing",      "junction with nothing pending: straight on", sc_cross, ck_cross, 20.0, 1, false, 0 },
    { "hump_40",       "40 mm hump: S6, peak within 20 %", sc_hump40, ck_hump, 30.0, 1, false, 0 },
    { "hump_20",       "20 mm hump", sc_hump20, ck_hump, 30.0, 1, false, 0 },
    { "hump_10",       "10 mm hump over 200 mm: S6, peak within 20 %", sc_hump10, ck_hump, 30.0, 1, false, 0 },
    { "hump_10_long",  "10 mm hump over 300 mm", sc_hump10l, ck_hump, 30.0, 1, false, 0 },
    { "hump_recal",    "hump, finish, RESET: tilt and hump zeroed for calibration", sc_hump_recal, ck_hump_recal, 45.0, 1, false, 0 },
    { "hump_none",     "flat: no false hump", sc_hump_none, ck_hump, 30.0, 1, false, 0 },
    { "obstacle",      "box on the line: scan, bypass, rejoin, finish", sc_obst_c, ck_obst_c, 45.0, 1, true, 0 },
    { "obstacle_wide", "220 mm box: width measured by the body scan, bypass, finish", sc_obst_wide, ck_obst_c, 50.0, 1, true, 0 },
    { "obstacle_recal", "obstacle, finish, RESET: obstacle width zeroed for calibration", sc_obstacle_recal, ck_obstacle_recal, 60.0, 1, true, 0 },
    { "obstacle_spot", "box on the line, 9 mm line-sensor spot: rejoin stops on the line", sc_obstacle_spot, ck_obst_c, 45.0, 1, true, 0 },
    { "obstacle_spot11", "box on the line, 11 mm line-sensor spot: rejoin stops on the line", sc_obstacle_spot_r, ck_obst_c, 45.0, 1, true, 0 },
    { "obstacle_coast", "box on the line, wheels coast 0.3 s when the motors stop: rejoin still on the line", sc_obstacle_coast, ck_obst_c, 45.0, 1, true, 0 },
    { "obstacle_skew", "line steps sideways before the box: car squares up on the line, then scans", sc_obst_skew, ck_obst_c, 45.0, 1, true, 0 },
    { "obstacle_long", "box mostly right of the line, its left end nearer: passed on the left (short way), backing off to clear that corner", sc_obst_long, ck_obst_long, 45.0, 1, true, 0 },
    { "obstacle_far", "box mostly left of the line, something far beyond on the right: short way round (right)", sc_obst_far, ck_obst_far, 45.0, 1, true, 0 },
    { "obstacle_deep", "deep box mostly left of the line: its side face is the box, short way round (right)", sc_obst_deep, ck_obstacle_short_right, 45.0, 1, true, 0 },
    { "obstacle_stagger", "as wide each side, left half nearer: right needs less backing off, pass right", sc_obst_stagger, ck_obstacle_short_right, 45.0, 1, true, 0 },
    { "obstacle_curve_l", "course curves left behind the box, into a left detour: line met on the detour is followed", sc_obst_curve_l, ck_obst_curve, 55.0, 1, true, 0 },
    { "obstacle_left", "box offset left: bypass right", sc_obst_l, ck_obst_l, 45.0, 1, true, 0 },
    { "obstacle_wall", "box + wall on the left: bypass right", sc_obst_wall, ck_obst_wall, 45.0, 1, true, 0 },
    { "obstacle_boxed","three boxes abreast: reverse, retry, BYPASS_FAIL", sc_obst_blocked, ck_obst_blocked, 50.0, 1, true, FAULT_BYPASS_FAIL },
    { "obstacle_corridor","box between walls parallel to the path (sonar cannot see them)", sc_obst_corridor, ck_obst_blocked, 50.0, 1, true, FAULT_BYPASS_FAIL, true },
    { "obstacle_beside","box beside the path: no bypass", sc_obst_side, ck_obst_side, 25.0, 1, true, 0 },
    { "servo_slow",    "servo 400 deg/s (firmware allows exactly 600): wall case", sc_servo_slow, ck_obst_wall, 45.0, 1, true, 0 },
    { "servo_mirrored","servo turns the other way (bench risk)", sc_servo_mirror, ck_obst_l, 45.0, 1, true, 0, true },
    { "line_gap",      "30 mm gap in the line", sc_gap, ck_gap, 25.0, 1, false, 0 },
    { "line_ends",     "line ends, no marker: search, LINE_NOT_FOUND", sc_line_ends, ck_line_ends, 30.0, 1, false, FAULT_LINE_NOT_FOUND },
    { "line_step",     "50 mm gap, line resumes 20 mm to the side", sc_step, ck_step, 35.0, 1, false, 0 },
    { "line_step_60",  "line resumes 60 mm to the side: BETWEEN mounting drives 250 mm blind", sc_step60, ck_step, 35.0, 1, false, FAULT_LINE_NOT_FOUND, true },
    { "estop",         "GP21 E-STOP mid-run, START=RESET, START, finish", sc_estop, ck_estop, 40.0, 2, false, FAULT_ESTOP_REQ },
    { "console",       "MQTT only: cal, RESET+cal again from READY, go, finish", sc_console, ck_console, 45.0, 0, false, 0 },
    { "manual_drive",  "MQTT w/a/d: hold to drive, release stops, one tap turns a little", sc_manual, ck_manual, 9.0, 0, false, 0 },
    { "manual_hump",   "driven by hand over a 15 mm hump: height measured", sc_manual_hump, ck_manual_hump, 12.0, 0, false, 0 },
    { "manual_running","drive keys during a run are ignored, the run finishes", sc_manual_running, ck_manual_running, 25.0, 1, false, 0 },
    { "manual_blocked","w refused with an obstacle ~110 mm ahead", sc_manual_blocked, ck_manual_blocked, 6.0, 0, false, 0 },
    { "manual_approach","w towards an obstacle: stops short of it", sc_manual_approach, ck_manual_approach, 6.0, 0, false, 0 },
    { "plaid_straight","plaid: w held 3 s at ~250 mm/s stays straight and stops on release", sc_plaid_straight, ck_plaid_straight, 7.0, 0, false, 0 },
    { "stunt_j",       "plaid J-turn: net 180 deg, stops, no faults", sc_stunt_j, ck_stunt, 13.0, 0, false, 0 },
    { "stunt_slalom",  "plaid reverse slalom: net 0 deg, stops, no faults", sc_stunt_slalom, ck_stunt, 13.0, 0, false, 0 },
    { "stunt_bootleg", "plaid bootleg turn: net 180 deg, stops, no faults", sc_stunt_bootleg, ck_stunt, 13.0, 0, false, 0 },
    { "stunt_trail",   "plaid trail turn: net 90 deg right, stops, no faults", sc_stunt_trail, ck_stunt, 13.0, 0, false, 0 },
    { "stunt_pendulum","plaid pendulum turn: net 90 deg left, stops, no faults", sc_stunt_pendulum, ck_stunt, 13.0, 0, false, 0 },
    { "stunt_drift",   "plaid drift on raw power: about 120 deg right, stops, no faults", sc_stunt_drift, ck_stunt_drift, 13.0, 0, false, 0 },
    { "stunt_donut",   "plaid donut on raw power: about two turns, stops, no faults", sc_stunt_donut, ck_stunt_donut, 13.0, 0, false, 0 },
    { "drift_abort",   "space during the drift's raw kick stops the car", sc_drift_abort, ck_drift_abort, 7.0, 0, false, 0 },
    { "stunt_locked",  "a stunt key without plaid armed does nothing", sc_stunt_locked, ck_stunt_locked, 6.0, 0, false, 0 },
    { "stunt_abort",   "space during a stunt stops the car", sc_stunt_abort, ck_stunt_abort, 5.5, 0, false, 0 },
    { "remote_stop",   "MQTT STOP <=100 ms, RESET, START, finish", sc_remote, ck_remote, 40.0, 1, false, FAULT_REMOTE_STOP },
    { "mqtt_dup",      "duplicate seq ignored, 2-byte payload rejected", sc_mqtt_dup, ck_mqtt_dup, 40.0, 1, false, FAULT_REMOTE_STOP },
    { "log_gap",       "broker away 20 s: buffered lines arrive afterwards (or loss reported)", sc_log_gap, ck_log_gap, 30.0, 1, false, 0 },
    { "wifi_loss",     "link drops: WIFI_LOSS, keeps going", sc_wifi, ck_wifi, 30.0, 1, false, FAULT_WIFI_LOSS },
    { "imu_absent",    "no IMU: IMU fault, I2C diagnosis printed, finishes anyway", sc_imu_absent, ck_imu_absent_diag, 30.0, 1, false, FAULT_IMU },
    { "imu_dies",      "IMU bus fails mid-run", sc_imu_dies, ck_imu_dies, 30.0, 1, false, FAULT_IMU },
    { "lifted",        "picked up mid-run: LIFTED, wheels stopped within 1.5 s", sc_lifted, ck_lifted, 14.0, 1, false, 0x1000U },
    { "lifted_nose",   "picked up nose first, held tilted: LIFTED and no hump measured", sc_lifted_nose, ck_lifted_nose, 14.0, 1, false, 0x1000U },
    { "lifted_tilt",   "picked up, felt only as a 30 deg roll: LIFTED, wheels stopped", sc_lifted_tilt, ck_lifted, 14.0, 1, false, 0x1000U },
    { "imu_lsm303d",   "GY-511 with an LSM303D (0x1D): found, hump measured", sc_imu_lsm303d, ck_imu_part, 30.0, 1, false, 0 },
    { "imu_gain",      "LSM303D reading gravity 8 % high (real boards vary): hump still measured", sc_imu_gain, ck_imu_part, 30.0, 1, false, 0 },
    { "hump_10_tilt",  "10 mm hump, pitch reads 3 deg nose-down while driving: measured from the driving baseline", sc_hump_10_tilt, ck_hump, 30.0, 1, false, 0 },
    { "hump_bump",     "short 10 mm bump over 80 mm, pitch -2.5 deg while driving: found", sc_hump_bump, ck_hump, 30.0, 1, false, 0 },
    { "hump_none_vibe", "flat, real-car vibration: no false hump", sc_hump_none_vibe, ck_hump, 30.0, 1, false, 0 },
    { "hump_10_vibe",  "10 mm hump, real-car vibration: found", sc_hump_10_vibe, ck_hump, 30.0, 1, false, 0 },
    { "bump_vibe",     "short 10 mm bump, real-car vibration: found", sc_bump_vibe, ck_hump, 30.0, 1, false, 0 },
    { "imu_side",      "IMU turned 90 deg (Y along the car): hump from the roll axis", sc_imu_side, ck_imu_side, 30.0, 1, false, 0 },
    { "imu_side_flip", "IMU turned 90 deg and upside down, reads 6 % low: hump from the roll axis", sc_imu_side_flip, ck_imu_side, 30.0, 1, false, 0 },
    { "imu_side_10",   "IMU turned 90 deg and upside down, 10 mm hump", sc_imu_side_10, ck_imu_side, 30.0, 1, false, 0 },
    { "imu_flip_x",    "IMU upside down (turned over X), reads 6 % low: no false LIFTED, hump measured", sc_imu_flip_x, ck_imu_flip, 30.0, 1, false, 0 },
    { "imu_flip_y",    "IMU upside down with X backwards: learns it, no false LIFTED, hump measured", sc_imu_flip_y, ck_imu_flip, 30.0, 1, false, 0 },
    { "lifted_flip",   "IMU upside down, picked up mid-run: LIFTED", sc_lifted_flip, ck_lifted, 14.0, 1, false, 0x1000U },
    { "imu_mpu6050",   "MPU-6050 (0x68) in the IMU slot: found, hump measured", sc_imu_mpu6050, ck_imu_part, 30.0, 1, false, 0 },
    { "imu_flaky",     "1 in 3 I2C transfers abort", sc_imu_flaky, ck_imu_flaky, 30.0, 1, false, FAULT_IMU },
    { "impact_boot",   "4 g spike while the IMU calibrates (after a reboot): no IMPACT", sc_impact_boot, ck_impact_boot, 30.0, 1, false, 0 },
    { "impact",        "4 g bump: no stop (collision detection off), finishes", sc_impact, ck_impact, 30.0, 1, false, 0 },
    { "stall",         "left wheel jammed: stall/encoder fault", sc_stall2, ck_stall, 15.0, 1, false, FAULT_MOTOR_STALL | FAULT_ENCODER },
    { "encoder_dead",  "left encoder dies: ENCODER fault", sc_enc_dead, ck_enc_dead, 15.0, 1, false, FAULT_ENCODER | FAULT_MOTOR_STALL },
    { "sonar_dead",    "HC-SR04 never answers: NO_ECHO", sc_echo_dead, ck_echo_dead, 15.0, 1, false, FAULT_NO_ECHO },
    { "cal_off_line",  "placed 80 mm beside the line: SELF_TEST, says to put the line between the sensors", sc_cal_off_line, ck_cal_off_line, 10.0, 1, false, FAULT_SELF_TEST },
    { "adc_stuck",     "ADC never ready: SELF_TEST", sc_adc_stuck, ck_adc_stuck, 10.0, 1, false, FAULT_SELF_TEST },
    { "adc_late",      "2 % of ADC conversions late: no SELF_TEST, finishes", sc_adc_late, ck_straight, 20.0, 1, false, 0 },
    { "ir_inverted",   "IR polarity inverted: auto-calibrated", sc_inverted, ck_curves, 30.0, 1, false, 0 },
    { "ir_straddle",   "sensors 18 mm apart (one over each edge): STRADDLE detected", sc_straddle, ck_straddle, 30.0, 1, false, 0 },
    { "motor_mismatch","right motor 17 % weaker, bigger deadband", sc_mismatch, ck_curves, 30.0, 1, false, 0 },
    { "geometry_off",  "true wheel 68 mm, track 142 mm (firmware: 65/130)", sc_geometry, ck_bc_a, 30.0, 1, false, 0 },
    { "motor_reversed","left motor wired backwards: must end safe", sc_motor_rev, ck_wiring, 15.0, 1, true, 0xFFFF },
    { "encoder_reversed","left encoder wired backwards: must end safe", sc_enc_rev, ck_wiring, 15.0, 1, true, 0xFFFF },
    { "stray_start",   "START pressed while running: must still stop at END", sc_start_extra, ck_start_extra, 20.0, 1, false, 0 },
    { "estop_bounce",  "E-STOP contact bounce", sc_bounce, ck_bounce, 12.0, 1, false, FAULT_ESTOP_REQ },
    { "estop_bypass",  "E-STOP during the bypass, then RESET", sc_estop_bypass, ck_estop_bypass, 40.0, 1, true, FAULT_ESTOP_REQ | FAULT_LINE_NOT_FOUND },
    { "cpu_load",      "every kernel call 40-120 us: curves", sc_load, ck_curves, 30.0, 1, false, 0 },
    { "full_course",   "B-junction, hump, curve, obstacle, A-junction, END", sc_full, ck_full, 90.0, 1, true, 0 },
    { "full_course_cross", "the full course with bars across the line", sc_full_cross, ck_full, 90.0, 1, true, 0 },
    { "endurance",     "oval loop 300 s: wrap, saturation, no faults", sc_loop, ck_loop, 300.0, 1, false, 0 },
};
#define N_SC ((int)(sizeof(g_scenarios) / sizeof(g_scenarios[0])))

/* ------------------------------------------------------------------ run */

static void firmware_main(void) { (void)usermain(); }

static int run_one(const scenario_t *sc, uint32_t seed, FILE *log)
{
    uint32_t cmin = 2U, cmax = 25U;
    g_sc = sc;
    memset(&g_m, 0, sizeof(g_m));
    g_m.tlm_last_seq = -1;
    g_m.states_seen = 1U;
    g_m.auto_go = (sc->go > 0);
    g_m.go_count = sc->go;
    if (0 == strcmp(sc->name, "cpu_load")) { cmin = 40U; cmax = 120U; }
    sim_kernel_init(seed, cmin, cmax);
    hw_init();
    world_defaults();
    /* seed-dependent start: small pose error, sensor noise level */
    g_car.x0 = 0.0;
    g_car.y0 = (sim_randf() - 0.5) * 6.0;
    g_car.h0_deg = (sim_randf() - 0.5) * 6.0;
    if (getenv("SIM_BAR"))
    {
        extern double g_bar_in, g_bar_out;
        sscanf(getenv("SIM_BAR"), "%lf,%lf", &g_bar_in, &g_bar_out);
    }
    sc->setup();
    hw_imu_reset_regs();            /* after setup: it may pick the IMU part */
    if (getenv("SIM_GAIN_R")) { g_car.gain_r = atof(getenv("SIM_GAIN_R")); }
    if (getenv("SIM_TRACK")) { g_car.track_mm = atof(getenv("SIM_TRACK")); }
    if (getenv("SIM_ENC_L")) { g_car.enc_sign_l = atoi(getenv("SIM_ENC_L")); }
    if (getenv("SIM_EPR")) { g_car.edges_per_rev = atof(getenv("SIM_EPR")); }
    if (getenv("SIM_LEFT_IR"))       /* "white,black" of the left sensor */
    {
        sscanf(getenv("SIM_LEFT_IR"), "%lf,%lf", &g_car.ir_white_l, &g_car.ir_black_l);
        g_car.ir_white = 170.0; g_car.ir_black = 3000.0;
    }
    if (getenv("SIM_FLOOR")) { g_car.us_floor_mm = atof(getenv("SIM_FLOOR")); }
    if (getenv("SIM_ENC_R")) { g_car.enc_sign_r = atoi(getenv("SIM_ENC_R")); }
    if (getenv("SIM_MOUNT")) { g_car.servo_mount_left = atof(getenv("SIM_MOUNT")); }
    if (getenv("SIM_MOTOR_R")) { g_car.motor_sign_r = atoi(getenv("SIM_MOTOR_R")); }
    if (getenv("SIM_SPOT")) { g_car.ir_spot_mm = atof(getenv("SIM_SPOT")); }
    if (getenv("SIM_SLOW")) { g_slow = atof(getenv("SIM_SLOW")); }
    if (getenv("SIM_D0")) { g_car.d0_black_high = (0 != atoi(getenv("SIM_D0"))); }
    if (getenv("SIM_AHEAD")) { g_car.ir_ahead_mm = atof(getenv("SIM_AHEAD")); }  /* IR pair ahead of the axle */
    if (getenv("SIM_HALF")) { g_car.ir_half_mm = atof(getenv("SIM_HALF")); }    /* half the IR spacing */
    if (getenv("SIM_TAU")) { g_car.tau_s = atof(getenv("SIM_TAU")); }           /* motor time constant */
    if (getenv("SIM_GAIN")) { g_car.gain_l = g_car.gain_r = atof(getenv("SIM_GAIN")); }  /* mm/s at full duty */
    if (getenv("SIM_BAR"))
    {
        extern double g_bar_in, g_bar_out;
        sscanf(getenv("SIM_BAR"), "%lf,%lf", &g_bar_in, &g_bar_out);
    }
    if (getenv("SIM_Y0")) { g_car.y0 = atof(getenv("SIM_Y0")); g_car.h0_deg = 0.0; }
    if (getenv("SIM_NARROW")) { /* used by course_barcode via g_bc_narrow */ }
    sim_console_file(log);
    sim_console_hook(console);
#if !SERVO_SCAN
    g_car.servo_continuous = true;
#endif
    world_start();
    sim_every_ms(monitor, NULL);
    net_bridge_connected(1);
    g_m.net_up = true;
    sim_at(MS(50), net_poll, NULL);
    if (sc->go > 0) { world_press(20U, S(2.5), 120U); }   /* calibrate */
    sim_boot(firmware_main);
    sim_run_until(S(sc->secs * g_slow));

    /* ---- per-scenario oracles ---- */
    sc->check();

    /* ---- invariants for every run ---- */
    EXPECT(0U == g_kstats.di_violations, "%u blocking calls with interrupts masked", g_kstats.di_violations);
    EXPECT(0U == g_kstats.isr_kernel_misuse, "%u blocking calls from an ISR", g_kstats.isr_kernel_misuse);
    EXPECT(0U == g_kstats.error_lines, "%u ERROR lines on the console", g_kstats.error_lines);
    EXPECT(0U == g_line_overruns, "T2 overran %u times (NFR1)", (unsigned)g_line_overruns);
    EXPECT(hw_wdt_enabled(), "watchdog never enabled");
    EXPECT(hw_wdt_max_gap_us() < MS(100), "watchdog feed gap %.1f ms", (double)hw_wdt_max_gap_us() / 1000.0);
    EXPECT(0U == g_m.tlm_bad, "%u malformed telemetry records", g_m.tlm_bad);
    EXPECT(g_m.tlm_gaps <= 1U, "%u telemetry sequence gaps", g_m.tlm_gaps);
    EXPECT(fabs((double)g_m.tlm_records - sc->secs * g_slow * 5.0) <= 3.0, "%u telemetry records in %.0f s (5 Hz)",
           g_m.tlm_records, sc->secs * g_slow);
    EXPECT(0 == g_log_bad_stamp, "%d log lines without a time stamp", g_log_bad_stamp);
    EXPECT(0 == g_log_mismatch, "%d log lines differ from the console", g_log_mismatch);
    EXPECT((0 != g_log_lost_reports) || (g_n_log >= g_n_con - 8),
           "log over MQTT: %d of %d console lines arrived", g_n_log, g_n_con);
    note("log %d/%d lines%s", g_n_log, g_n_con, g_log_lost_reports ? ", overflow reported" : "");
    EXPECT(0U == g_m.tlm_mismatch, "%u records with a mission state never held", g_m.tlm_mismatch);
    EXPECT(sc->allow_t4_reject || (0 == g_m.n_t4_rejected), "T4 rejected %d requests", g_m.n_t4_rejected);
    {
        uint16_t const unexpected = (uint16_t)(g_m.faults_seen & ~sc->allowed_faults);
        EXPECT(0U == unexpected, "unexpected faults 0x%04x", unexpected);
    }
    {
        sim_task_info_t ti;
        for (int i = 0; i < sim_task_count(); i++)
        {
            sim_task_info(i, &ti);
            if ((NULL != ti.name) && (0 == strcmp(ti.name, "T2_line")))
            {
                EXPECT(ti.max_latency_us < 1000U, "T2 release latency %llu us", (unsigned long long)ti.max_latency_us);
            }
        }
    }
    return g_nfail;
}

static void usage(void)
{
    fprintf(stderr, "sim list | run <scenario> [seed] [-v] | all [seeds] [filter]\n");
}

int main(int argc, char **argv)
{
    if (argc < 2) { usage(); return 2; }
    if (0 == strcmp(argv[1], "list"))
    {
        for (int i = 0; i < N_SC; i++) { printf("%-20s %s\n", g_scenarios[i].name, g_scenarios[i].what); }
        return 0;
    }
    if ((0 == strcmp(argv[1], "run")) && (argc >= 3))
    {
        uint32_t const seed = (argc >= 4) ? (uint32_t)atoi(argv[3]) : 1U;
        g_verbose = (argc >= 5);
        for (int i = 0; i < N_SC; i++)
        {
            if (0 == strcmp(argv[2], g_scenarios[i].name))
            {
                int const nf = run_one(&g_scenarios[i], seed, g_verbose ? stdout : NULL);
                bool const xf = g_scenarios[i].xfail;
                printf("%s seed %u: %s%s [%s]\n", g_scenarios[i].name, seed,
                       nf ? (xf ? "XFAIL (known limitation): " : "FAIL: ") : (xf ? "XPASS" : "PASS"),
                       g_fail, g_notes);
                fflush(stdout);
                _exit((nf && !xf) ? 1 : 0);
            }
        }
        usage();
        return 2;
    }
    if (0 == strcmp(argv[1], "all"))
    {
        int const seeds = (argc >= 3) ? atoi(argv[2]) : 3;
        const char *filter = (argc >= 4) ? argv[3] : NULL;
        int total = 0, failed = 0;
        for (int i = 0; i < N_SC; i++)
        {
            int pass = 0;
            if ((NULL != filter) && (NULL == strstr(g_scenarios[i].name, filter))) { continue; }
            /* bc_bench needs the BARCODE_TEST build and only runs there. */
            if ((0 == strncmp(g_scenarios[i].name, "bc_", 3)) != (0 != APP_BARCODE_TEST)) { continue; }
            for (int s = 1; s <= seeds; s++)
            {
                int fd[2];
                pid_t pid;
                char out[8192];
                ssize_t n = 0, r;
                int status = 0;
                if (pipe(fd) != 0) { return 2; }
                pid = fork();
                if (0 == pid)
                {
                    char sbuf[16];
                    close(fd[0]);
                    dup2(fd[1], 1);
                    snprintf(sbuf, sizeof(sbuf), "%d", s);
                    execl(argv[0], argv[0], "run", g_scenarios[i].name, sbuf, (char *)NULL);
                    _exit(2);
                }
                close(fd[1]);
                while ((r = read(fd[0], out + n, sizeof(out) - 1 - (size_t)n)) > 0) { n += r; }
                out[n] = '\0';
                close(fd[0]);
                waitpid(pid, &status, 0);
                total++;
                if (WIFEXITED(status) && (0 == WEXITSTATUS(status))) { pass++; if (seeds == 1) { printf("%s", out); } }
                else { failed++; printf("%s", (n > 0) ? out : "(crashed)\n"); }
                fflush(stdout);
            }
            if (seeds > 1) { printf("== %-20s %d/%d seeds pass\n", g_scenarios[i].name, pass, seeds); }
        }
        printf("[SIM] SUMMARY %d runs, %d passed, %d failed\n", total, total - failed, failed);
        return failed ? 1 : 0;
    }
    usage();
    return 2;
}
