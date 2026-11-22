/** @file task_vehicle.c
 *
 * @brief T6 VehicleCtrl: the mission state machine S0-S10 (Figure 3,
 *        as amended at Tribunal 1).  Event driven: waits on flg_sys for
 *        the bits that matter in the current state, acts briefly (posts a
 *        motion request, never blocks inside a transition), and returns
 *        to waiting.  A 50 ms timeout drives the distance and time checks.
 *
 * Tribunal decisions implemented here:
 *   - Barcode A/B/C go PENDING and are consumed at the first junction
 *     after the barcode, however far (owner 2026-11-19, D73: the 500 mm
 *     budget expired every command before the real junction); D (U-turn)
 *     executes at once.  A newer barcode replaces a waiting command.
 *   - END is all-black for > 60 mm (T2).  At a junction the car first
 *     drives the IR-to-axle distance; if END arrives meanwhile it wins
 *     over the turn.  MQTT END is a backup only.
 *   - S3 -> S7 handover: T6 drains its own queued requests, then the
 *     mission word changes to S7 so T4 rejects T6-sourced requests; T5
 *     posts only after it sees S7, so its requests are never flushed.
 *   - An obstacle seen mid-manoeuvre stays latched in flg_sys and is
 *     acted on after MOTION_DONE.
 */

#include "subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

#ifndef APP_BARCODE_TEST
#define APP_BARCODE_TEST        0
#endif
#define DRIVE_NONE              0xFFU
#define DRIVE_HOLD_MS           600U    /* key repeat starts ~0.5 s after
                                           the press: stop after this    */
#define DRIVE_MOVE_MM           3000U   /* "keep going" while held      */
#define DRIVE_TURN_DEG          720U
#define DRIVE_BLOCK_MM          150U    /* sonar: no driving forward in */
#define DRIVE_NOTE_MS           2000U
#define DRIVE_WHEELS            0xFEU   /* plaid w a s d (MREQ_WHEELS)    */
#define DRIVE_STUNT             0xFDU   /* a stunt is running             */
#define PLAID_TURN_PCT          70U     /* plaid a / d: wheel speed share */
#define PLAID_BLOCK_MM          300U    /* plaid: room to stop from 250   */
#define STUNT_CLEAR_MM          500U    /* sonar clear this far to start  */
#define ECHO_US_PER_10MM        58U
#define SCAN_STEP               9U      /* S1 step: a scan pass is running */
#define SCAN_MM                 350U    /* longest course symbol + margin  */
#define SCAN_SPEED_MMS          50U     /* owner, 2026-11-10, with the D0
                                           sensor (15 stick-slipped)     */
#define TICK_MS                 50
#define INIT_SETTLE_MS          500U
#define BARCODE_TIMEOUT_MS      1500U
#define LATE_JUNCTION_MM        200U    /* a command this soon after a
                                           cross bar is for that bar (D72) */
#define OBSTACLE_TIMEOUT_MS     60000U  /* x2 for 50 mm/s creep        */
#define SEARCH_TIMEOUT_MS       20000U  /* x2.5 for 50 mm/s turns      */
#define CAL_SWEEP_DEG           35U
#define CAL_SPEED_MMS           60U     /* slow sweep: less coasting     */
#define CAL_SETTLE_MS           200U    /* T2 reads the centred values  */
#define CAL_RETURN_MIN_DEG      4U      /* below this the correcting turn
                                           would coast as far as the error */
#define CAL_RETURN_MAX_DEG      90U     /* larger: something is wrong    */
#define RAD_TO_DEG_F            57.29578f
#define PI_F                    3.14159265f
#define TURN_DEG                90U
#define UTURN_DEG               180U
#define STRAIGHT_PAST_MM        30U
#define SEARCH_DEG              45U
#define SEARCH_STEP_MM          60U
#define SEARCH_ROUNDS           2U

/* Bits T6 never consumes on wake-up (other owners clear them, or, for
   ESTOP, only an accepted RESET clears it). */
#define FLG_NOT_MINE            (FLG_ESTOP | FLG_LINE_LOST | FLG_NET_UP |  \
                                 FLG_CAL_REQ | FLG_CAL_STOP)

#define FOLLOW_EVENTS           (FLG_ESTOP | FLG_NAV | FLG_BARCODE_START | \
                                 FLG_JUNCTION | FLG_END | FLG_LINE_LOST |  \
                                 FLG_HUMP_START | FLG_HUMP_END |           \
                                 FLG_OBSTACLE | FLG_LINE_FOUND)

typedef enum
{
    MAN_JUNCTION = 0,   /* centre on the junction, then turn/straight */
    MAN_UTURN           /* turn 180 in place now                      */
} manoeuvre_t;

typedef struct
{
    uint32_t    state;
    uint32_t    pending;
    uint32_t    budget;
    uint32_t    executing;      /* NAV command being executed (S5)  */
    uint32_t    step;           /* sub-step inside S1, S5, S8       */
    uint32_t    entry_ms;
    uint32_t    search_side;    /* MREQ_TURN_LEFT / MREQ_TURN_RIGHT */
    manoeuvre_t man;
    uint16_t    odom_prev;
    bool        b_calibrated;
    uint16_t    cal_enc_l;      /* encoder counts when the sweep began */
    uint16_t    cal_enc_r;
    uint8_t     drive_seq;      /* last manual key press seen       */
    uint32_t    drive_cmd;      /* manual move running, or DRIVE_NONE */
    uint32_t    drive_until;    /* stop then unless the key repeats */
    uint32_t    drive_note_ms;  /* last "not now" message           */
    uint8_t     drive_key;      /* key of the running manual move   */
    bool        b_plaid;        /* plaid armed ('!'), parked only   */
    struct stunt_s const * stunt;   /* running stunt, or NULL       */
    uint32_t    stunt_i;        /* next step                        */
    uint32_t    seg_until;      /* this step ends then              */
    int32_t     seg_l;          /* this step's wheel targets, mm/s  */
    int32_t     seg_r;
    uint8_t     map_seq;        /* last barcode-map message seen    */
    uint16_t    bc_odom;        /* odometer when S4 last began      */
    bool        b_bc_odom;
    uint16_t    cross_used;     /* g_cross_word bar already turned at */
    uint32_t    late_past;      /* junction already passed by, mm   */
} vehicle_ctx_t;

/* Barcode letter -> command (app.h).  Written here only. */
EXPORT volatile uint8_t g_bc_map[BC_MAP_LETTERS] =
{
    NAV_LEFT, NAV_RIGHT, NAV_STRAIGHT, NAV_UTURN     /* A B C D; rest 0 */
};

EXPORT volatile uint16_t g_mission_word = 0U;

static vehicle_ctx_t g_vc;

/* ------------------------------------------------------------------ */

static uint32_t
now_ms (void)
{
    SYSTIM tim;

    (void)tk_get_tim(&tim);
    return ((uint32_t)tim.lo);
}

static void
publish (void)
{
    uint32_t word = 0U;

    word = bits_put(word, MISSION_STATE_SHIFT, MISSION_STATE_BITS,
                    g_vc.state);
    word = bits_put(word, MISSION_PEND_SHIFT, MISSION_PEND_BITS,
                    g_vc.pending);
    word = bits_put(word, MISSION_BUDGET_SHIFT, MISSION_BUDGET_BITS,
                    bits_sat_u(g_vc.budget, MISSION_BUDGET_BITS));
    g_mission_word = (uint16_t)word;    /* one aligned store (rule R1) */
}

static char const * const k_mission_names[MISSION_FAULT + 1U] =
{
    "S0 INIT", "S1 CALIBRATE", "S2 READY", "S3 FOLLOW", "S4 BARCODE",
    "S5 MANOEUVRE", "S6 HUMP", "S7 OBSTACLE", "S8 SEARCH", "S9 FINISHED",
    "S10 FAULT"
};

EXPORT char const * mission_name (uint32_t state)
{
    return ((state <= MISSION_FAULT) ? k_mission_names[state] : "S? ---");
}

static void
enter (uint32_t state)
{
    if (state != g_vc.state)
    {
        tm_printf((UB *)"STATE %s -> %s\n", mission_name(g_vc.state),
                  mission_name(state));
    }
    g_vc.state = state;
    g_vc.step = 0U;
    g_vc.entry_ms = now_ms();
    publish();

    /* START and RESET are only meaningful while waiting for them.  One
       pressed earlier (mid-run, or MQTT) stays latched in flg_sys and would
       otherwise restart the car the moment it stops at the END marker or
       reset a fault nobody has looked at. */
    if ((MISSION_CALIBRATE == state) || (MISSION_READY == state) ||
        (MISSION_FINISHED == state) || (MISSION_FAULT == state))
    {
        (void)tk_clr_flg(flg_sys, ~(FLG_START | FLG_RESET_REQ));
    }
}

static uint32_t
in_state_ms (void)
{
    return (now_ms() - g_vc.entry_ms);
}

static void
consume (UINT bits)
{
    (void)tk_clr_flg(flg_sys, ~(bits & ~FLG_NOT_MINE));
}

/* Post a request as T6.  A goal's completion arrives as MOTION_DONE. */
static void
request (uint32_t cmd, uint32_t arg)
{
    if ((MREQ_STOP == cmd) || (MREQ_FORWARD == cmd) ||
        (MREQ_BACKWARD == cmd) || (MREQ_TURN_LEFT == cmd) ||
        (MREQ_TURN_RIGHT == cmd))
    {
        (void)tk_clr_flg(flg_sys, ~FLG_MOTION_DONE);
    }
    CHECK(motion_request(cmd, arg, MREQ_SRC_T6), "motion_request");
}

static bool
following (uint32_t state)
{
    return ((MISSION_FOLLOW == state) || (MISSION_BARCODE == state) ||
            (MISSION_HUMP == state));
}

static void
follow (uint32_t state)
{
    uint32_t speed = SPEED_FOLLOW;

    if (MISSION_BARCODE == state)
    {
        speed = SPEED_BARCODE;
    }
    else if (MISSION_HUMP == state)
    {
        speed = SPEED_HUMP;
    }
    else
    {
        /* Nominal line-following speed. */
    }
    /* Publish the state BEFORE posting: leaving S7, T4 (higher priority)
       may run between the two and must already see T6 as the owner, or it
       drops this FOLLOW and the car sits still in S3. */
    enter(state);
    request(MREQ_FOLLOW, speed);
}

/* ------------------------------------------------------------------ */

static void
start_manoeuvre (manoeuvre_t man, uint32_t nav)
{
    g_vc.man = man;
    g_vc.executing = nav;
    enter(MISSION_MANOEUVRE);

    if (MAN_UTURN == man)
    {
        request(MREQ_TURN_LEFT, UTURN_DEG);
    }
    else if (0U != g_vc.late_past)
    {
        /* The bar is behind the IR row already (D72): put the axle on it,
           forward or back by what is left. */
        uint32_t const past = g_vc.late_past;

        g_vc.late_past = 0U;
        if (past < LINE_SENSOR_AHEAD_MM)
        {
            request(MREQ_FORWARD, LINE_SENSOR_AHEAD_MM - past);
        }
        else if (past > LINE_SENSOR_AHEAD_MM)
        {
            request(MREQ_BACKWARD, past - LINE_SENSOR_AHEAD_MM);
        }
        else
        {
            request(MREQ_FORWARD, 1U);      /* on it: a token move */
        }
    }
    else
    {
        /* IR row is ahead of the axle: drive the axle onto the junction
           first.  END (all-black > 60 mm) is decided during this move. */
        request(MREQ_FORWARD, LINE_SENSOR_AHEAD_MM);
    }
}

/* A command that arrives just after its junction (D72): the cross bar
   came close after the barcode and was swept into the capture, so the
   symbol was decoded only once the IR row had passed the bar (real car
   2026-11-19 17:54: 'F' decoded 150 mm late, junction missed, command
   expired).  T2 remembers the last fresh cross-bar crossing; if it is
   since this barcode began and at most LATE_JUNCTION_MM back, turn there. */
static bool
late_junction (uint32_t nav)
{
    uint32_t const w = g_cross_word;
    uint16_t const at = (uint16_t)bits_get(w, CROSS_ODOM_SHIFT,
                                           CROSS_ODOM_BITS);
    uint16_t const now = odom_now_mm();
    uint32_t const past = odom_delta_mm(now, at);

    if ((!following(g_vc.state)) ||
        (0U == bits_get(w, CROSS_VALID_SHIFT, 1U)) ||
        (at == g_vc.cross_used) || (!g_vc.b_bc_odom) ||
        (past > LATE_JUNCTION_MM) ||
        (odom_delta_mm(at, g_vc.bc_odom) >= 0x8000U))   /* before S4 */
    {
        return (false);
    }
    g_vc.cross_used = at;
    g_vc.late_past = past;
    tm_printf((UB *)"T6: command %d is for the junction %d mm back\n",
              (INT)nav, (INT)past);
    start_manoeuvre(MAN_JUNCTION, nav);
    return (true);
}

static void
manoeuvre_done (void)
{
    if ((MAN_JUNCTION == g_vc.man) && (0U == g_vc.step))
    {
        g_vc.step = 1U;
        if (NAV_LEFT == g_vc.executing)
        {
            request(MREQ_TURN_LEFT, TURN_DEG);
            return;
        }
        if (NAV_RIGHT == g_vc.executing)
        {
            request(MREQ_TURN_RIGHT, TURN_DEG);
            return;
        }
        request(MREQ_FORWARD, STRAIGHT_PAST_MM);    /* NAV_STRAIGHT */
        return;
    }

    tm_printf((UB *)"T6: command %d executed\n", (INT)g_vc.executing);
    g_vc.executing = NAV_NONE;
    follow(MISSION_FOLLOW);
}

static void
handle_nav (void)
{
    nav_cmd_t msg = 0U;

    while (tk_rcv_mbf(mbf_nav, &msg, TMO_POL) > 0)
    {
        uint32_t const cmd = bits_get(msg, NAV_CMD_SHIFT, NAV_CMD_BITS);

        if ((NAV_LEFT == cmd) || (NAV_RIGHT == cmd) || (NAV_STRAIGHT == cmd))
        {
            if (late_junction(cmd))
            {
                g_vc.pending = NAV_NONE;
                continue;
            }
            g_vc.pending = cmd;
            g_vc.budget = 0U;               /* distance waited (D73) */
            g_vc.odom_prev = odom_now_mm();
            tm_printf((UB *)"T6: command %d pending for the junction\n",
                      (INT)cmd);
        }
        else if ((NAV_UTURN == cmd) && following(g_vc.state))
        {
            g_vc.pending = NAV_NONE;
            start_manoeuvre(MAN_UTURN, NAV_UTURN);
        }
        else
        {
            /* REJECT / OTHER: logged by T2, nothing to execute. */
        }
    }
    publish();
}

/* ------------------------------------------------------------------ */

static void
enter_fault (void)
{
    motion_force_off();
    g_vc.pending = NAV_NONE;
    g_vc.executing = NAV_NONE;
    enter(MISSION_FAULT);
    tm_printf((UB *)"T6: SAFE STATE, faults 0x%04x\n", (INT)fault_get());
}

static void
try_reset (void)
{
    fault_clear(FAULT_RESETTABLE_MASK);
    if (fault_is_critical())
    {
        tm_printf((UB *)"T6: RESET refused, faults 0x%04x still active\n",
                  (INT)fault_get());
        return;
    }
    (void)tk_clr_flg(flg_sys, ~FLG_ESTOP);
    enter(MISSION_INIT);
}

static void
start_search (void)
{
    uint32_t const err = bits_get(g_line_word, LINE_ERR_SHIFT, LINE_ERR_BITS);
    T_RFLG         rflg;

    enter(MISSION_SEARCH);
    (void)tk_clr_flg(flg_sys, ~FLG_LINE_FOUND);
    /* The flag, not g_line_word, says whether the line is still lost: T2
       sets and clears it the moment L3 changes, but publishes the word
       only every 10 ms.  Checking the word made T6 bounce S3 <-> S8 for
       up to 10 ms, flooding mbf_motion with FOLLOW requests. */
    if ((E_OK == tk_ref_flg(flg_sys, &rflg)) &&
        (0U == (rflg.flgptn & FLG_LINE_LOST)))
    {
        follow(MISSION_FOLLOW);         /* already back */
        return;
    }
    g_vc.search_side = (bits_to_signed(err, LINE_ERR_BITS) >= 0) ?
                       MREQ_TURN_LEFT : MREQ_TURN_RIGHT;
    request(g_vc.search_side, SEARCH_DEG);
}

/* Search pattern after the first turn toward the last-seen side:
   other side 90, back 45, creep 60 mm, last side 45, ... */
static void
search_done (void)
{
    uint32_t const other = (MREQ_TURN_LEFT == g_vc.search_side) ?
                           MREQ_TURN_RIGHT : MREQ_TURN_LEFT;
    uint32_t const phase = g_vc.step % 4U;

    if (g_vc.step >= (4U * SEARCH_ROUNDS))
    {
        fault_set(FAULT_LINE_NOT_FOUND);
        return;
    }
    g_vc.step++;

    switch (phase)
    {
        case 0U:
            request(other, 2U * SEARCH_DEG);
        break;

        case 1U:
            request(g_vc.search_side, SEARCH_DEG);
        break;

        case 2U:
            request(MREQ_FORWARD, SEARCH_STEP_MM);
        break;

        default:
            request(g_vc.search_side, SEARCH_DEG);
        break;
    }
}

/* After the sweep: turn back by the net rotation the encoders measured
   since it began.  Each turn of the sweep ends with the car coasting on a
   little in its own direction, so the three do not cancel exactly (bench
   2026-11-01: the car did not end up facing the way it started).  The
   encoders keep counting while it coasts, so this undoes the overshoot. */
static bool
cal_return (void)
{
    float const   mm_per_edge = (PI_F * MOTION_WHEEL_DIA_MM) /
                                MOTION_EDGES_PER_REV;
    int32_t const dl = (int32_t)(int16_t)(uint16_t)(g_enc_left - g_vc.cal_enc_l);
    int32_t const dr = (int32_t)(int16_t)(uint16_t)(g_enc_right - g_vc.cal_enc_r);
    float const   deg = ((float)(dr - dl) * mm_per_edge / MOTION_TRACK_MM) *
                        RAD_TO_DEG_F;   /* + = ended up turned left */
    uint32_t const mag = (uint32_t)((deg < 0.0f) ? -deg : deg);

    tm_printf((UB *)"T6: sweep ended %d deg off the start heading\n",
              (INT)deg);
    if ((mag < CAL_RETURN_MIN_DEG) || (mag > CAL_RETURN_MAX_DEG))
    {
        return (false);
    }
    request((deg > 0.0f) ? MREQ_TURN_RIGHT : MREQ_TURN_LEFT, mag);
    return (true);
}

/* ------------------------------------------------------------------ */

static UINT
wait_mask (void)
{
    UINT mask = FLG_ESTOP;

    switch (g_vc.state)
    {
        case MISSION_CALIBRATE:
            mask |= FLG_START | FLG_MOTION_DONE | FLG_CAL_DONE;
        break;

        case MISSION_READY:
        case MISSION_FINISHED:
            mask |= FLG_START | FLG_RESET_REQ;
        break;

        case MISSION_FOLLOW:
        case MISSION_BARCODE:
        case MISSION_HUMP:
            mask = FOLLOW_EVENTS;
        break;

        case MISSION_MANOEUVRE:
            mask |= FLG_MOTION_DONE | FLG_END;
        break;

        case MISSION_OBSTACLE:
            mask |= FLG_LINE_REACQ;
        break;

        case MISSION_SEARCH:
            mask |= FLG_MOTION_DONE | FLG_LINE_FOUND | FLG_END;
        break;

        case MISSION_FAULT:
            mask = FLG_START | FLG_RESET_REQ;   /* ESTOP stays set here */
        break;

        default:    /* MISSION_INIT: time driven */
        break;
    }
    return (mask);
}

static void
handle_follow (UINT ptn)
{
    if (0U != (ptn & FLG_END))
    {
        request(MREQ_STOP, 0U);
        enter(MISSION_FINISHED);
        return;
    }
    if (0U != (ptn & FLG_NAV))
    {
        handle_nav();
        if (!following(g_vc.state))
        {
            return;                     /* a U-turn started */
        }
        if (MISSION_BARCODE == g_vc.state)
        {
            follow(MISSION_FOLLOW);
        }
    }
    if (0U != (ptn & FLG_OBSTACLE))
    {
        /* Hand motion to T5: drain T6's own queued requests FIRST, then
           publish S7 (T4 then rejects any T6-sourced request).  Flushing
           after the switch could discard T5's first request. */
        motion_flush();
        enter(MISSION_OBSTACLE);
        return;
    }
    if (0U != (ptn & FLG_LINE_LOST))
    {
        start_search();
        return;
    }
    if ((0U != (ptn & FLG_JUNCTION)) && (NAV_NONE != g_vc.pending))
    {
        uint32_t const nav = g_vc.pending;

        g_vc.pending = NAV_NONE;
        start_manoeuvre(MAN_JUNCTION, nav);
        return;
    }
    if ((0U != (ptn & FLG_HUMP_START)) && (MISSION_HUMP != g_vc.state))
    {
        follow(MISSION_HUMP);
    }
    else if ((0U != (ptn & FLG_HUMP_END)) && (MISSION_HUMP == g_vc.state))
    {
        follow(MISSION_FOLLOW);
    }
    else if ((0U != (ptn & FLG_BARCODE_START)) &&
             (MISSION_FOLLOW == g_vc.state))
    {
        g_vc.bc_odom = odom_now_mm();       /* D72 */
        g_vc.b_bc_odom = true;
        follow(MISSION_BARCODE);
    }
    else
    {
        /* Nothing else changes the state. */
    }
}

static void
handle_events (UINT ptn)
{
    if ((0U != (ptn & FLG_ESTOP)) && (MISSION_FAULT != g_vc.state))
    {
        enter_fault();
        return;
    }

    switch (g_vc.state)
    {
        case MISSION_CALIBRATE:
#if APP_BARCODE_TEST
            /* Self-driven scan pass (bars are measured by wheel distance):
               START creeps forward, the next START creeps back, so repeat
               passes read the symbol from both ends.  Not sonar guarded. */
            if ((0U != (ptn & FLG_START)) && (SCAN_STEP != g_vc.step))
            {
                static bool s_b_back = false;

                tm_printf((UB *)"T6: scan pass %s, %d mm at %d mm/s\n",
                          s_b_back ? "backward" : "forward", (INT)SCAN_MM,
                          (INT)SCAN_SPEED_MMS);
                g_vc.step = SCAN_STEP;
                request(MREQ_SET_SPEED, SCAN_SPEED_MMS);
                request(s_b_back ? MREQ_BACKWARD : MREQ_FORWARD, SCAN_MM);
                s_b_back = !s_b_back;
            }
            else if ((0U != (ptn & FLG_MOTION_DONE)) &&
                     (SCAN_STEP == g_vc.step))
            {
                g_vc.step = 0U;
                tm_printf((UB *)"T6: scan pass done\n");
            }
            else
            {
                /* Waiting. */
            }
            break;
#endif
            if ((0U == g_vc.step) && (0U != (ptn & FLG_START)))
            {
                /* Step 1: hold still while T2 reads the centred values
                   (mounting detection); periodic() starts the sweep. */
                tm_printf((UB *)"T6: calibration sweep\n");
                g_vc.step = 1U;
                g_vc.entry_ms = now_ms();
                (void)tk_set_flg(flg_sys, FLG_CAL_REQ);
            }
            else if ((0U != (ptn & FLG_MOTION_DONE)) && (g_vc.step >= 2U) &&
                     (g_vc.step <= 4U))
            {
                /* Sweep (owner's choice, 2026-11-02): front right, then
                   left back to the start, then the measured correction. */
                g_vc.step++;
                if (3U == g_vc.step)
                {
                    request(MREQ_TURN_LEFT, CAL_SWEEP_DEG);
                }
                else if ((4U == g_vc.step) && cal_return())
                {
                    /* Turning back onto the starting heading. */
                }
                else
                {
                    (void)tk_set_flg(flg_sys, FLG_CAL_STOP);
                }
            }
            else if (0U != (ptn & FLG_CAL_DONE))
            {
                g_vc.b_calibrated = true;
                request(MREQ_SET_SPEED, SPEED_FOLLOW);  /* sweep speed off */
                enter(MISSION_READY);
                tm_printf((UB *)"T6: calibrated -- press START to run\n");
            }
            else
            {
                /* Waiting. */
            }
        break;

        case MISSION_READY:
        case MISSION_FINISHED:
            if (0U != (ptn & FLG_START))
            {
                tm_printf((UB *)"T6: START -- driving\n");
                g_vc.pending = NAV_NONE;
                /* Nothing left over from before the run: a hump (D65),
                   an end marker or junction (a START once went straight
                   to S9, real car 2026-11-19). */
                (void)tk_clr_flg(flg_sys, ~(FLG_HUMP_START | FLG_HUMP_END |
                                            FLG_END | FLG_JUNCTION));
                follow(MISSION_FOLLOW);
            }
            else if (0U != (ptn & FLG_RESET_REQ))
            {
                /* MQTT RESET while parked: start again from S0 and
                   calibrate afresh (the console's "cal" after a run). */
                tm_printf((UB *)"T6: RESET -- recalibrate\n");
                g_vc.b_calibrated = false;
                enter(MISSION_INIT);
            }
            else
            {
                /* Waiting. */
            }
        break;

        case MISSION_FOLLOW:
        case MISSION_BARCODE:
        case MISSION_HUMP:
            handle_follow(ptn);
        break;

        case MISSION_MANOEUVRE:
            if ((0U != (ptn & FLG_END)) && (MAN_JUNCTION == g_vc.man) &&
                (0U == g_vc.step))
            {
                request(MREQ_STOP, 0U);     /* end marker beats the turn */
                enter(MISSION_FINISHED);
            }
            else if (0U != (ptn & FLG_MOTION_DONE))
            {
                manoeuvre_done();
            }
            else
            {
                /* Waiting for the manoeuvre. */
            }
        break;

        case MISSION_OBSTACLE:
            if (0U != (ptn & FLG_LINE_REACQ))
            {
                g_vc.odom_prev = odom_now_mm();     /* budget resumes */
                follow(MISSION_FOLLOW);
            }
        break;

        case MISSION_SEARCH:
            if (0U != (ptn & FLG_END))
            {
                request(MREQ_STOP, 0U);
                enter(MISSION_FINISHED);
            }
            else if (0U != (ptn & FLG_LINE_FOUND))
            {
                g_vc.odom_prev = odom_now_mm();
                follow(MISSION_FOLLOW);     /* FOLLOW replaces the turn */
            }
            else if (0U != (ptn & FLG_MOTION_DONE))
            {
                search_done();
            }
            else
            {
                /* Searching. */
            }
        break;

        case MISSION_FAULT:
            if (0U != (ptn & (FLG_RESET_REQ | FLG_START)))
            {
                try_reset();
            }
        break;

        default:
        break;
    }

    if ((MISSION_FINISHED == g_vc.state) && (0U != (ptn & FLG_END)))
    {
        tm_printf((UB *)"T6: course complete, max hump %d mm, %d mm\n",
                  (INT)bits_get(g_hump_byte, 0U, HUMP_PEAK_BITS),
                  (INT)bits_get(g_odom_word, ODOM_TOTAL_SHIFT,
                                ODOM_TOTAL_BITS));
    }
}

/* Sonar range ahead in mm, or 0 if there was no echo. */
static uint32_t
sonar_mm (void)
{
    uint16_t const echo = g_echo_word;

    if (0U == ((uint32_t)echo >> ECHO_VALID_SHIFT))
    {
        return (0U);
    }
    return ((bits_get(echo, 0U, ECHO_WIDTH_BITS) * 10U) / ECHO_US_PER_10MM);
}

/* ---- manual driving: plain, plaid, stunts ------------------------- */

/* Pack per-wheel targets for T4 (subsystem2.h, g_wheel_word). */
static void
wheels (int32_t left, int32_t right)
{
    g_wheel_word = (((uint32_t)left & 0xFFFU) << WHEEL_L_SHIFT) |
                   (((uint32_t)right & 0xFFFU) << WHEEL_R_SHIFT);
}

static void
drive_stop (void)
{
    request(MREQ_STOP, 0U);
    g_vc.drive_cmd = DRIVE_NONE;
    g_vc.stunt = NULL;
}

/* Stunts (owner, 2026-11-16): show manoeuvres at plaid speed, no course
   function.  Each is a short choreography of straights, in-place pivots
   and arcs, given by geometry and turned into wheel speeds and times at
   SPEED_PLAID.  Heading changes are signed, + = anticlockwise (left). */
typedef enum { ST_LINE, ST_PIVOT, ST_ARC, ST_RAW } stunt_kind_t;
typedef struct
{
    uint8_t kind;
    int16_t a;      /* LINE: mm (- = backwards)  PIVOT: deg  ARC: radius mm */
    int16_t b;      /* ARC: heading change, deg                            */
    int8_t  dir;    /* ARC: +1 forwards, -1 backwards                      */
    uint8_t pct;    /* share of SPEED_PLAID for the fastest wheel;         */
                    /* RAW: a, b = left, right power, permille; pct = time
                       in 10 ms (no speed loop: the tyres may slide)      */
} stunt_step_t;

/* J-turn: reverse fast, whip the nose round 180, drive off forwards. */
static const stunt_step_t ST_J[] = {
    { ST_LINE, -350, 0, 0, 100 }, { ST_PIVOT, 180, 0, 0, 80 },
    { ST_LINE, 350, 0, 0, 100 } };
/* Controlled reverse slalom: backwards through three gates. */
static const stunt_step_t ST_SLALOM[] = {
    { ST_ARC, 250, 30, -1, 90 }, { ST_ARC, 250, -60, -1, 90 },
    { ST_ARC, 250, 60, -1, 90 }, { ST_ARC, 250, -60, -1, 90 },
    { ST_ARC, 250, 30, -1, 90 } };
/* Bootleg turn: charge forwards, swing 180 round the locked left wheel
   (the "handbrake"), charge back the way it came. */
static const stunt_step_t ST_BOOTLEG[] = {
    { ST_LINE, 400, 0, 0, 100 }, { ST_ARC, 65, 180, 1, 100 },
    { ST_LINE, 400, 0, 0, 100 } };
/* Trail turn: brake while turning in (speed falls through the first two
   thirds of a right-hand 90), power out of the last third. */
static const stunt_step_t ST_TRAIL[] = {
    { ST_LINE, 400, 0, 0, 100 }, { ST_ARC, 200, -30, 1, 90 },
    { ST_ARC, 200, -30, 1, 70 }, { ST_ARC, 200, -30, 1, 85 },
    { ST_LINE, 300, 0, 0, 100 } };
/* Pendulum (Scandinavian flick): feint right, then swing hard left
   through 115 for a net 90 left. */
static const stunt_step_t ST_PENDULUM[] = {
    { ST_LINE, 300, 0, 0, 100 }, { ST_ARC, 300, -25, 1, 100 },
    { ST_ARC, 180, 115, 1, 100 }, { ST_LINE, 300, 0, 0, 100 } };

/* Drift: plaid entry, then raw power: a kick (outside wheel hard forward,
   inside back) breaks traction while the car still carries its speed, a
   hold keeps it rotating as it slides, a counter-steer catches it, and a
   plaid straight drives out.  About 120 deg to the right on a floor with
   grip (tests/sim has no tyre slip); a smooth floor slides more. */
static const stunt_step_t ST_DRIFT[] = {
    { ST_LINE, 400, 0, 0, 100 }, { ST_RAW, 900, -300, 0, 30 },
    { ST_RAW, 800, 350, 0, 70 }, { ST_RAW, 450, 750, 0, 25 },
    { ST_LINE, 300, 0, 0, 100 } };
/* Donut: round the stopped right wheel on raw power, about two turns. */
static const stunt_step_t ST_DONUT[] = {
    { ST_LINE, 200, 0, 0, 100 }, { ST_RAW, 850, 0, 0, 225 },
    { ST_RAW, 850, 0, 0, 225 }, { ST_LINE, 200, 0, 0, 100 } };

typedef struct stunt_s
{
    char                 key;
    char const *         name;
    stunt_step_t const * steps;
    uint32_t             n;
} stunt_t;
#define STUNT(k, nm, t) { (k), (nm), (t), (uint32_t)(sizeof(t) / sizeof((t)[0])) }
static const stunt_t STUNTS[] = {
    STUNT('j', "J-turn", ST_J), STUNT('r', "reverse slalom", ST_SLALOM),
    STUNT('b', "bootleg turn", ST_BOOTLEG), STUNT('t', "trail turn", ST_TRAIL),
    STUNT('p', "pendulum turn", ST_PENDULUM), STUNT('f', "drift", ST_DRIFT),
    STUNT('o', "donut", ST_DONUT) };

/* One step as wheel speeds; returns its time in ms. */
static uint32_t
stunt_wheels (stunt_step_t const * st, int32_t * p_l, int32_t * p_r)
{
    int32_t const v = (int32_t)((SPEED_PLAID * (uint32_t)st->pct) / 100U);
    int32_t const half = (int32_t)(MOTION_TRACK_MM / 2.0f);
    uint32_t      ms = 0U;

    if (ST_RAW == st->kind)
    {
        *p_l = st->a;               /* power, permille: see stunt_next */
        *p_r = st->b;
        ms = (uint32_t)st->pct * 10U;
    }
    else if (ST_LINE == st->kind)
    {
        int32_t const d = st->a;

        *p_l = (d >= 0) ? v : -v;
        *p_r = *p_l;
        ms = ((uint32_t)((d >= 0) ? d : -d) * 1000U) / (uint32_t)v;
    }
    else if (ST_PIVOT == st->kind)
    {
        int32_t const  th = st->a;
        uint32_t const arc = ((uint32_t)((th >= 0) ? th : -th) *
                              (uint32_t)half * 355U) / (180U * 113U);

        *p_l = (th >= 0) ? -v : v;
        *p_r = -*p_l;
        ms = (arc * 1000U) / (uint32_t)v;
    }
    else
    {
        /* Arc of radius R about its centre: the outer wheel at v, the
           centre at vc = v R / (R + W/2); omega = vc / R. */
        int32_t const  r = st->a;
        int32_t const  th = st->b;
        int32_t const  vc = (v * r) / (r + half);
        int32_t const  dw = (vc * half) / r;
        int32_t const  sgn = (th >= 0) ? 1 : -1;
        uint32_t const path = ((uint32_t)r * (uint32_t)((th >= 0) ? th : -th) *
                               355U) / (180U * 113U);

        *p_l = ((int32_t)st->dir * vc) - (sgn * dw);
        *p_r = ((int32_t)st->dir * vc) + (sgn * dw);
        ms = (path * 1000U) / (uint32_t)vc;
    }
    return (ms);
}

/* Next step of the running stunt, or finish it. */
static void
stunt_next (uint32_t now)
{
    int32_t  l;
    int32_t  r;
    int32_t  dl;
    int32_t  dr;
    uint32_t ms;

    if (g_vc.stunt_i >= g_vc.stunt->n)
    {
        tm_printf((UB *)"T6: stunt %s done\n", (UB *)g_vc.stunt->name);
        drive_stop();
        return;
    }
    ms = stunt_wheels(&g_vc.stunt->steps[g_vc.stunt_i], &l, &r);
    if (ST_RAW == g_vc.stunt->steps[g_vc.stunt_i].kind)
    {
        /* One store: T4 must never see the values without the flag. */
        g_wheel_word = (((uint32_t)l & 0xFFFU) << WHEEL_L_SHIFT) |
                       (((uint32_t)r & 0xFFFU) << WHEEL_R_SHIFT) |
                       (1UL << WHEEL_RAW_BIT);      /* power, not speed */
        /* Roughly the speeds that power gives (450 mm/s at full power,
           PID_FF): for the sonar check and the next step's ramp. */
        l = (l * 450) / 1000;
        r = (r * 450) / 1000;
    }
    else
    {
        /* The wheels slew at WHEEL_ACCEL_MMS2: give back half the ramp. */
        dl = (l > g_vc.seg_l) ? (l - g_vc.seg_l) : (g_vc.seg_l - l);
        dr = (r > g_vc.seg_r) ? (r - g_vc.seg_r) : (g_vc.seg_r - r);
        ms += ((uint32_t)((dl > dr) ? dl : dr) * 1000U) /
              (2U * WHEEL_ACCEL_MMS2);
        wheels(l, r);
    }
    g_vc.seg_l = l;
    g_vc.seg_r = r;
    g_vc.seg_until = now + ms;
    g_vc.stunt_i++;
}

static stunt_t const *
stunt_find (uint8_t key)
{
    uint32_t idx;

    for (idx = 0U; idx < (sizeof(STUNTS) / sizeof(STUNTS[0])); idx++)
    {
        if ((uint8_t)STUNTS[idx].key == key)
        {
            return (&STUNTS[idx]);
        }
    }
    return (NULL);
}

static bool
parked (void)
{
    return ((MISSION_READY == g_vc.state) ||
            (MISSION_FINISHED == g_vc.state) ||
            ((MISSION_CALIBRATE == g_vc.state) && (0U == g_vc.step)));
}

static void
drive_note (uint32_t now, char const * text, uint32_t value)
{
    if ((now - g_vc.drive_note_ms) >= DRIVE_NOTE_MS)
    {
        g_vc.drive_note_ms = now;
        tm_printf((UB *)text, (INT)value);
    }
}

/* Plaid w a s d: per-wheel targets, held like the plain keys. */
static void
plaid_key (uint8_t key, uint32_t now)
{
    int32_t const v = (int32_t)SPEED_PLAID;
    int32_t const t = (int32_t)((SPEED_PLAID * PLAID_TURN_PCT) / 100U);

    switch (key)
    {
        case 'w': wheels(v, v); break;
        case 's': wheels(-v, -v); break;
        case 'a': wheels(-t, t); break;
        default:  wheels(t, -t); break;
    }
    if (DRIVE_WHEELS != g_vc.drive_cmd)
    {
        request(MREQ_WHEELS, 0U);
        g_vc.drive_cmd = DRIVE_WHEELS;
    }
    if (key != g_vc.drive_key)
    {
        tm_printf((UB *)"T6: plaid drive %c\n", (INT)key);
    }
    g_vc.drive_key = key;
    g_vc.drive_until = now + DRIVE_HOLD_MS;
}

/* Manual driving from the console (w a s d, space stops).  Parked only:
   before calibrating, calibrated, or finished.  Hold to drive: a key
   starts one long move and its repeats only extend the deadline (a new
   move would restart acceleration from zero); releasing it stops the car
   DRIVE_HOLD_MS later.
   '!' arms plaid mode (SPEED_PLAID, w a s d through MREQ_WHEELS), '.'
   disarms it.  With plaid armed, j r b t p f o run a stunt; any key aborts
   it.  Plaid disarms itself when the car leaves the parked states. */
static void
manual_drive (void)
{
    uint8_t const  seq = g_drive_seq;
    uint32_t       now;
    uint32_t       range;
    bool           b_parked;
    bool           b_blocked;
    bool           b_blocked_fast;

    if (g_vc.b_plaid && !parked())
    {
        g_vc.b_plaid = false;
        tm_printf((UB *)"T6: PLAID off (%s)\n", (UB *)mission_name(g_vc.state));
    }
    if ((seq == g_vc.drive_seq) && (DRIVE_NONE == g_vc.drive_cmd))
    {
        return;                         /* no key, nothing moving */
    }
    now = now_ms();
    range = sonar_mm();
    b_parked = parked();
    b_blocked = (0U != range) && (range < DRIVE_BLOCK_MM);
    b_blocked_fast = (0U != range) && (range < PLAID_BLOCK_MM);

    if (seq != g_vc.drive_seq)
    {
        uint8_t const         key = g_drive_key;
        stunt_t const * const stunt = stunt_find(key);
        uint32_t              cmd = DRIVE_NONE;
        uint32_t              arg = DRIVE_MOVE_MM;

        g_vc.drive_seq = seq;
        switch (key)
        {
            case 'w': cmd = MREQ_FORWARD; break;
            case 's': cmd = MREQ_BACKWARD; break;
            case 'a': cmd = MREQ_TURN_LEFT; arg = DRIVE_TURN_DEG; break;
            case 'd': cmd = MREQ_TURN_RIGHT; arg = DRIVE_TURN_DEG; break;
            default: break;             /* space or anything else: stop */
        }

        if (DRIVE_STUNT == g_vc.drive_cmd)
        {
            tm_printf((UB *)"T6: stunt %s aborted\n", (UB *)g_vc.stunt->name);
            drive_stop();               /* any key aborts a stunt */
        }
        else if (!b_parked)
        {
            if ((now - g_vc.drive_note_ms) >= DRIVE_NOTE_MS)
            {
                g_vc.drive_note_ms = now;
                tm_printf((UB *)"T6: manual drive only while parked"
                          " (S1 before calibrating, S2, S9), now %s\n",
                          mission_name(g_vc.state));
            }
        }
        else if ('!' == key)
        {
            if (!g_vc.b_plaid)
            {
                g_vc.b_plaid = true;
                tm_printf((UB *)"T6: PLAID armed, %d mm/s. Keep 1.5 m clear"
                          " around the car\n", (INT)SPEED_PLAID);
            }
        }
        else if ('.' == key)
        {
            if (DRIVE_NONE != g_vc.drive_cmd)
            {
                drive_stop();
            }
            if (g_vc.b_plaid)
            {
                g_vc.b_plaid = false;
                tm_printf((UB *)"T6: PLAID off\n");
            }
        }
        else if (NULL != stunt)
        {
            if (!g_vc.b_plaid)
            {
                tm_printf((UB *)"T6: stunts need PLAID armed first\n");
            }
            else if ((0U != range) && (range < STUNT_CLEAR_MM))
            {
                tm_printf((UB *)"T6: stunt refused, obstacle %d mm ahead\n",
                          (INT)range);
            }
            else
            {
                if (DRIVE_NONE != g_vc.drive_cmd)
                {
                    request(MREQ_STOP, 0U);
                }
                tm_printf((UB *)"T6: stunt %s\n", (UB *)stunt->name);
                g_vc.stunt = stunt;
                g_vc.stunt_i = 0U;
                g_vc.seg_l = 0;
                g_vc.seg_r = 0;
                stunt_next(now);
                request(MREQ_WHEELS, 0U);
                g_vc.drive_cmd = DRIVE_STUNT;
            }
        }
        else if (DRIVE_NONE == cmd)
        {
            if (DRIVE_NONE != g_vc.drive_cmd)
            {
                drive_stop();
                tm_printf((UB *)"T6: manual drive stop\n");
            }
        }
        else if (('w' == key) && (g_vc.b_plaid ? b_blocked_fast : b_blocked))
        {
            if (DRIVE_NONE != g_vc.drive_cmd)
            {
                drive_stop();
            }
            drive_note(now, "T6: manual drive blocked, obstacle %d mm ahead\n",
                       range);
        }
        else if (g_vc.b_plaid)
        {
            plaid_key(key, now);
        }
        else
        {
            if (cmd != g_vc.drive_cmd)
            {
                request(MREQ_SET_SPEED, SPEED_FOLLOW);
                request(cmd, arg);
                g_vc.drive_cmd = cmd;
                tm_printf((UB *)"T6: manual drive %c\n", (INT)key);
            }
            g_vc.drive_key = key;
            g_vc.drive_until = now + DRIVE_HOLD_MS;
        }
    }

    if (DRIVE_STUNT == g_vc.drive_cmd)
    {
        if (!b_parked)
        {
            drive_stop();               /* a run began or a fault */
        }
        else if (((g_vc.seg_l + g_vc.seg_r) > 0) && b_blocked_fast)
        {
            tm_printf((UB *)"T6: stunt %s stopped, obstacle %d mm ahead\n",
                      (UB *)g_vc.stunt->name, (INT)range);
            drive_stop();
        }
        else if ((int32_t)(now - g_vc.seg_until) >= 0)
        {
            stunt_next(now);
        }
        else
        {
            /* Step in progress. */
        }
    }
    else if (DRIVE_NONE != g_vc.drive_cmd)
    {
        bool const b_fwd = ('w' == g_vc.drive_key);

        if ((!b_parked) || ((int32_t)(now - g_vc.drive_until) >= 0))
        {
            drive_stop();               /* key released (or a run began) */
        }
        else if (b_fwd && ((DRIVE_WHEELS == g_vc.drive_cmd) ?
                           b_blocked_fast : b_blocked))
        {
            drive_stop();
            tm_printf((UB *)"T6: manual drive stopped, obstacle %d mm"
                      " ahead\n", (INT)range);
        }
        else
        {
            /* Still held. */
        }
    }
}

/* ---- barcode map (p5is20/car/map) ------------------------------- */

static char const k_map_char[] = "-LRSU";   /* by NAV_x, 0 = unmapped */

static void
map_print (char const * p_why)
{
    char     text[BC_MAP_LETTERS + 1U];
    uint32_t idx;
    uint32_t n = 0U;

    for (idx = 0U; idx < BC_MAP_LETTERS; idx++)
    {
        uint8_t const m = g_bc_map[idx];

        text[idx] = (m <= NAV_UTURN) ? k_map_char[m] : '-';
        if ((m >= NAV_LEFT) && (m <= NAV_UTURN))
        {
            n++;
        }
    }
    text[BC_MAP_LETTERS] = '\0';
    tm_printf((UB *)"T6: barcode map %s (%d mapped%s)\n", (UB *)text,
              (INT)n, (UB *)p_why);
}

/* A message on the map topic: 26 characters, one per letter A-Z, each
   L(eft) R(ight) S(traight) U(-turn) or - (unmapped); or "?" to report
   the map.  Applied only while parked: the map is set before the run. */
static void
handle_map (void)
{
    uint8_t const seq = g_map_seq;
    uint8_t       tmp[BC_MAP_LETTERS];
    uint32_t      len;
    uint32_t      idx;

    if (seq == g_vc.map_seq)
    {
        return;
    }
    g_vc.map_seq = seq;
    len = g_map_req_len;
    if ((1U == len) && ('?' == g_map_req[0]))
    {
        map_print("");
        return;
    }
    if (BC_MAP_LETTERS != len)
    {
        tm_printf((UB *)"T6: barcode map refused: need 26 characters for"
                  " A-Z (L R S U or -), got %d\n", (INT)len);
        return;
    }
    if (!parked())
    {
        tm_printf((UB *)"T6: barcode map refused during %s: set it before"
                  " the run\n", mission_name(g_vc.state));
        map_print(", unchanged");
        return;
    }
    for (idx = 0U; idx < BC_MAP_LETTERS; idx++)
    {
        uint8_t const c = (uint8_t)(g_map_req[idx] & 0xDFU);   /* upper */

        switch (c)
        {
            case 'L': tmp[idx] = NAV_LEFT; break;
            case 'R': tmp[idx] = NAV_RIGHT; break;
            case 'S': tmp[idx] = NAV_STRAIGHT; break;
            case 'U': tmp[idx] = NAV_UTURN; break;
            default:
                if (('-' == g_map_req[idx]) || ('.' == g_map_req[idx]) ||
                    ('N' == c))
                {
                    tmp[idx] = NAV_NONE;
                    break;
                }
                tm_printf((UB *)"T6: barcode map refused: letter %c has"
                          " '%c' (use L R S U or -)\n", (INT)('A' + idx),
                          (INT)g_map_req[idx]);
                return;
        }
    }
    for (idx = 0U; idx < BC_MAP_LETTERS; idx++)
    {
        g_bc_map[idx] = tmp[idx];
    }
    if (NAV_NONE != g_vc.pending)
    {
        g_vc.pending = NAV_NONE;    /* a command from the old map */
        g_vc.budget = 0U;
    }
    map_print(", set");
}

/* Every wake-up: timeouts and the pending command's distance budget. */
static void
periodic (void)
{
    uint16_t const odom = odom_now_mm();
    uint32_t const moved = odom_delta_mm(odom, g_vc.odom_prev);

    if (fault_is_critical() && (MISSION_FAULT != g_vc.state))
    {
        enter_fault();
        return;
    }

    manual_drive();
    handle_map();

    /* Budget counts only while following the line (suspended in S5,
       S7 and S8 -- tribunal Version C). */
    if (following(g_vc.state))
    {
        g_vc.odom_prev = odom;
        if (NAV_NONE != g_vc.pending)
        {
            /* No expiry (D73): the command is for the first junction.
               The budget field now counts the distance waited, for the
               telemetry (saturates). */
            g_vc.budget = (g_vc.budget + moved > 511U) ? 511U :
                          (g_vc.budget + moved);
            publish();
        }
    }

    switch (g_vc.state)
    {
        case MISSION_INIT:
            if (in_state_ms() >= INIT_SETTLE_MS)
            {
                if (g_vc.b_calibrated)
                {
                    enter(MISSION_READY);
                    tm_printf((UB *)"T6: ready -- press START to run\n");
                }
                else
                {
                    enter(MISSION_CALIBRATE);
                    tm_printf((UB *)"T6: place on the line, press START"
                              " to calibrate\n");
                }
            }
        break;

        case MISSION_CALIBRATE:
            if ((1U == g_vc.step) && (in_state_ms() >= CAL_SETTLE_MS))
            {
                g_vc.step = 2U;
                g_vc.cal_enc_l = g_enc_left;
                g_vc.cal_enc_r = g_enc_right;
                request(MREQ_SET_SPEED, CAL_SPEED_MMS);
                request(MREQ_TURN_RIGHT, CAL_SWEEP_DEG);
            }
        break;

        case MISSION_BARCODE:
            if (in_state_ms() >= BARCODE_TIMEOUT_MS)
            {
                follow(MISSION_FOLLOW);
            }
        break;

        case MISSION_OBSTACLE:
            if (in_state_ms() >= OBSTACLE_TIMEOUT_MS)
            {
                fault_set(FAULT_BYPASS_FAIL);
            }
        break;

        case MISSION_SEARCH:
            if (in_state_ms() >= SEARCH_TIMEOUT_MS)
            {
                fault_set(FAULT_LINE_NOT_FOUND);
            }
        break;

        default:
        break;
    }
}

EXPORT void vehicle_task (INT stacd, void * p_exinf)
{
    UINT ptn;
    UINT mask;
    ER   er;

    (void)stacd;
    (void)p_exinf;

    g_vc.state = MISSION_INIT;
    g_vc.drive_cmd = DRIVE_NONE;
    g_vc.drive_seq = g_drive_seq;
    g_vc.b_plaid = false;
    g_vc.stunt = NULL;
    g_vc.map_seq = g_map_seq;
    map_print(" by default");
    g_vc.odom_prev = odom_now_mm();
    enter(MISSION_INIT);

    for (;;)
    {
        mask = wait_mask();
        ptn = 0U;
        er = tk_wai_flg(flg_sys, mask, TWF_ORW, &ptn, TICK_MS);
        if (E_OK == er)
        {
            ptn &= mask;
            consume(ptn);
            handle_events(ptn);
        }
        periodic();
    }
}

/*** end of file ***/
