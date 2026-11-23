/* The car and the course.
 *
 * Units: mm, s, rad.  World frame x east, y north, heading CCW from x.
 * Body frame (firmware convention): x forward, y left, z up.
 *
 * Every millisecond: motor duty from the PWM registers -> first-order
 * wheel speed -> encoder edges (quadrature, through the GPIO interrupt) ->
 * pose -> hump height and pitch -> accelerometer registers.  ADC reads are
 * answered from the course map at the moment of conversion; an HC-SR04
 * trigger schedules the echo edges from a ray cast at the actual servo
 * angle.  Collisions with obstacles are detected and stop the car.
 */
#include "sim.h"
#include <math.h>
#include <string.h>

#define PI          3.14159265358979
#define G_MMS2      9806.65
#define DT          0.001
#define PIN_ENC_L_A 16U
#define PIN_ENC_L_B 13U
#define PIN_ENC_R_A 17U
#define PIN_ENC_R_B 19U
#define PIN_ECHO    3U
#define PIN_START   20U
#define PIN_ESTOP   21U

car_params_t g_car;
double g_bar_in = 16.0, g_bar_out = 40.0;
course_t     g_course;
car_state_t  g_st;

static double g_edge_acc[2];
static int    g_enc_a[2];
static double g_vfwd_prev;

void world_defaults(void)
{
    memset(&g_car, 0, sizeof(g_car));
    g_car.wheel_dia_mm = 65.0;  g_car.track_mm = 130.0;
    g_car.edges_per_rev = 1056.0;
    g_car.gain_l = 480.0; g_car.gain_r = 480.0;
    g_car.deadband = 0.08; g_car.tau_s = 0.06;
    g_car.enc_sign_l = -1; g_car.enc_sign_r = 1;    /* as the real car (2026-11-02) */
    g_car.motor_sign_l = 1; g_car.motor_sign_r = 1;
    g_car.ir_ahead_mm = 70.0; g_car.ir_half_mm = 13.5;
    g_car.bc_lat_mm = 20.0; g_car.ir_spot_mm = 4.0;
    g_car.ir_white = 350.0; g_car.ir_black = 3300.0; g_car.ir_noise = 25.0;
    g_car.us_ahead_mm = 60.0;
    g_car.servo_dps = 600.0;          /* SG90 datasheet, 4.8 V */
    g_car.imu_pitch0_deg = 1.5; g_car.imu_noise_mg = 12.0;
    g_car.width_mm = 150.0; g_car.length_mm = 180.0;
    memset(&g_course, 0, sizeof(g_course));
    g_course.line_w = 18.0;
}

/* ---------------------------------------------------------------- course */

static void bbox(prim_t *p)
{
    double r = p->half_w + 1.0;
    p->minx = fmin(p->a.x, p->b.x) - r; p->maxx = fmax(p->a.x, p->b.x) + r;
    p->miny = fmin(p->a.y, p->b.y) - r; p->maxy = fmax(p->a.y, p->b.y) + r;
}

static void add_seg(vec2 a, vec2 b, double half_w, int path)
{
    prim_t *p;
    if (g_course.n_prims >= MAX_PRIMS) { return; }
    p = &g_course.prims[g_course.n_prims++];
    p->kind = PRIM_SEG; p->a = a; p->b = b; p->half_w = half_w; p->path = path;
    bbox(p);
}

/* Oriented rectangle: centre line a->b (its length), half extent across. */
static void add_rect(vec2 a, vec2 b, double half_w)
{
    prim_t *p;
    if (g_course.n_prims >= MAX_PRIMS) { return; }
    p = &g_course.prims[g_course.n_prims++];
    p->kind = PRIM_RECT; p->a = a; p->b = b; p->half_w = half_w; p->path = 0;
    bbox(p);
    p->minx -= half_w; p->maxx += half_w; p->miny -= half_w; p->maxy += half_w;
}

static vec2 fwd(vec2 p, double h, double d) { return (vec2){ p.x + d * cos(h), p.y + d * sin(h) }; }
static vec2 left(vec2 p, double h, double d) { return (vec2){ p.x - d * sin(h), p.y + d * cos(h) }; }

void course_reset(double line_w) { memset(&g_course, 0, sizeof(g_course)); g_course.line_w = line_w; }

void course_start(double x, double y, double h_deg)
{
    g_course.cur = (vec2){ x, y };
    g_course.cur_h = h_deg * PI / 180.0;
}

void course_straight(double len)
{
    vec2 const b = fwd(g_course.cur, g_course.cur_h, len);
    add_seg(g_course.cur, b, g_course.line_w / 2.0, 1);
    g_course.cur = b;
    g_course.s += len;
}

void course_arc(double radius, double deg)
{
    /* Turn direction is the sign of deg; a negative radius also means a
       right turn (it used to draw the arc backwards). */
    if (radius < 0.0) { radius = -radius; deg = -fabs(deg); }
    int const n = (int)ceil(fabs(deg) / 4.0);
    double const step = (deg / n) * PI / 180.0;
    for (int i = 0; i < n; i++)
    {
        double const chord = 2.0 * radius * sin(fabs(step) / 2.0);
        g_course.cur_h += step / 2.0;
        course_straight(chord);
        g_course.cur_h += step / 2.0;
    }
}

void course_turn(double deg) { g_course.cur_h += deg * PI / 180.0; }

void course_width(double w) { g_course.line_w = w; }
void course_where(double *p_x, double *p_y) { *p_x = g_course.cur.x; *p_y = g_course.cur.y; }
void course_branch(double rel_deg, double len)
{
    double const h = g_course.cur_h + rel_deg * PI / 180.0;
    add_seg(g_course.cur, fwd(g_course.cur, h, len), g_course.line_w / 2.0, 0);
}

void course_gap(double len)
{
    g_course.cur = fwd(g_course.cur, g_course.cur_h, len);
    g_course.s += len;
}

void course_shift(double lat)
{
    g_course.cur = left(g_course.cur, g_course.cur_h, lat);
}

/* A black stripe on the barcode track and the white after it: the air under
   a car being set down on the paper, or any other mark that is no symbol. */
void course_mark(double black_mm, double white_mm)
{
    double const lat_c = -0.5 * (g_bar_in + g_bar_out);
    double const lat_h = 0.5 * (g_bar_out - g_bar_in);
    vec2 const a = left(g_course.cur, g_course.cur_h, lat_c);
    vec2 const p = fwd(g_course.cur, g_course.cur_h, black_mm + white_mm);

    add_rect(a, fwd(a, g_course.cur_h, black_mm), lat_h);
    add_seg(g_course.cur, p, g_course.line_w / 2.0, 1);
    g_course.s += black_mm + white_mm;
    g_course.cur = p;
}

void course_barcode(char c, double narrow, double wide, int crossing)
{
    /* Code 39, '*c*', narrow inter-character gap, bars beside the line on
       the right (barcode track 20 mm right of the line centre) or across
       the line and the track. */
    extern uint32_t barcode_encode(uint8_t, uint8_t, uint8_t, uint8_t *);
    extern char const g_barcode_chars[];
    uint8_t w[29];
    int idx = 0;
    vec2 p = g_course.cur;
    double const h = g_course.cur_h;
    /* Bars beside the line span g_bar_in..g_bar_out mm right of the line
       centre (the course drawing is not available: a parameter). */
    double const lat_c = crossing ? -10.0 : -0.5 * (g_bar_in + g_bar_out);
    double const lat_h = crossing ? 32.0 : 0.5 * (g_bar_out - g_bar_in);

    while ((idx < 43) && (g_barcode_chars[idx] != c)) { idx++; }
    (void)barcode_encode((uint8_t)idx, 10U, 30U, w);
    for (int el = 0; el < 29; el++)
    {
        double const len = (w[el] == 10U) ? narrow : wide;
        if (0 == (el % 2))      /* even elements are bars */
        {
            vec2 const a = left(p, h, lat_c);
            add_rect(a, fwd(a, h, len), lat_h);
        }
        p = fwd(p, h, len);
    }
    /* the line runs on through the symbol */
    add_seg(g_course.cur, p, g_course.line_w / 2.0, 1);
    g_course.s += hypot(p.x - g_course.cur.x, p.y - g_course.cur.y);
    g_course.cur = p;
}

void course_end_marker(double len, double width)
{
    vec2 const b = fwd(g_course.cur, g_course.cur_h, len);
    add_rect(g_course.cur, b, width / 2.0);
    add_seg(g_course.cur, b, g_course.line_w / 2.0, 1);
    g_course.cur = b;
    g_course.s += len;
}

void course_hump(double length, double height)
{
    hump_t *hp;
    if (g_course.n_humps >= MAX_HUMPS) { return; }
    hp = &g_course.humps[g_course.n_humps++];
    hp->c = g_course.cur; hp->heading = g_course.cur_h;
    hp->length = length; hp->height = height; hp->half_width = 400.0;
    course_straight(length);
}

void course_obstacle(double ahead, double lat, double w, double d)
{
    obstacle_t *o;
    if (g_course.n_obst >= MAX_OBST) { return; }
    o = &g_course.obst[g_course.n_obst++];
    o->c = left(fwd(g_course.cur, g_course.cur_h, ahead + d / 2.0), g_course.cur_h, lat);
    o->heading = g_course.cur_h; o->half_l = d / 2.0; o->half_w = w / 2.0;
}

/* ---------------------------------------------------------------- queries */

static double seg_dist(vec2 p, vec2 a, vec2 b)
{
    double const vx = b.x - a.x, vy = b.y - a.y;
    double const l2 = vx * vx + vy * vy;
    double t = (l2 > 0.0) ? (((p.x - a.x) * vx + (p.y - a.y) * vy) / l2) : 0.0;
    t = (t < 0.0) ? 0.0 : ((t > 1.0) ? 1.0 : t);
    return hypot(p.x - (a.x + t * vx), p.y - (a.y + t * vy));
}

static bool in_rect(vec2 p, const prim_t *r)
{
    double const vx = r->b.x - r->a.x, vy = r->b.y - r->a.y;
    double const len = hypot(vx, vy);
    double const ux = vx / len, uy = vy / len;
    double const dx = p.x - r->a.x, dy = p.y - r->a.y;
    double const along = dx * ux + dy * uy;
    double const across = -dx * uy + dy * ux;
    return (along >= 0.0) && (along <= len) && (fabs(across) <= r->half_w);
}

static bool black_at(vec2 p)
{
    for (int i = 0; i < g_course.n_prims; i++)
    {
        const prim_t *q = &g_course.prims[i];
        if ((p.x < q->minx) || (p.x > q->maxx) || (p.y < q->miny) || (p.y > q->maxy)) { continue; }
        if (PRIM_SEG == q->kind) { if (seg_dist(p, q->a, q->b) <= q->half_w) { return true; } }
        else if (in_rect(p, q)) { return true; }
    }
    return false;
}

static vec2 body(double fx, double ly)
{
    return left(fwd((vec2){ g_st.x, g_st.y }, g_st.h, fx), g_st.h, ly);
}

static double coverage_r(vec2 c, double spot);
static double coverage(vec2 c) { return coverage_r(c, g_car.ir_spot_mm); }
static double coverage_r(vec2 c, double spot)
{
    /* A large spot: smooth, weighted along the travel direction (bars are
       uniform across it), as a lens blurs.  The 13-point ring below would
       ripple at this size. */
    if ((spot == g_car.ir_spot_mm) && (spot > 5.0))
    {
        double sum = 0.0, weight = 0.0;
        double const r = spot / 2.0;
        for (int i = -12; i <= 12; i++)
        {
            double const u = (double)i / 12.0;
            double const w = sqrt(1.0 - u * u) + 0.01;
            vec2 const p = { c.x + u * r * cos(g_st.h), c.y + u * r * sin(g_st.h) };
            weight += w;
            if (black_at(p)) { sum += w; }
        }
        return sum / weight;
    }
    static const double ring[12][2] = {
        {1,0},{0.5,0.866},{-0.5,0.866},{-1,0},{-0.5,-0.866},{0.5,-0.866},
        {0.5,0},{0.25,0.433},{-0.25,0.433},{-0.5,0},{-0.25,-0.433},{0.25,-0.433} };
    double const r = spot / 2.0;
    int n = black_at(c) ? 1 : 0;
    for (int i = 0; i < 12; i++)
    {
        if (black_at((vec2){ c.x + ring[i][0] * r, c.y + ring[i][1] * r })) { n++; }
    }
    return (double)n / 13.0;
}

uint16_t world_adc(uint32_t ch)
{
    double lat;
    double f;
    double v;
    /* Wiring since 2026-11-02: ADC0 GP26 = right sensor (Grove 5),
       ADC1 GP27 = left sensor (Grove 6). */
    if (1U == ch) { lat = g_car.ir_half_mm; }
    else if (0U == ch) { lat = -g_car.ir_half_mm; }
    else if (2U == ch) { lat = -g_car.bc_lat_mm; }
    else { return 0U; }
    f = ((ch < 2U) && (g_car.ir_line_spot_mm > 0.0)) ?
        coverage_r(body(g_car.ir_ahead_mm, lat), g_car.ir_line_spot_mm) :
        coverage(body(g_car.ir_ahead_mm, lat));
    if ((1U == ch) && (g_car.ir_white_l > 0.0))
    {
        v = g_car.ir_white_l + f * (g_car.ir_black_l - g_car.ir_white_l) + g_car.ir_noise * sim_gauss();
    }
    else
    {
        v = g_car.ir_white + f * (g_car.ir_black - g_car.ir_white) + g_car.ir_noise * sim_gauss();
    }
    if (v < 0.0) { v = 0.0; }
    if (v > 4095.0) { v = 4095.0; }
    return (uint16_t)v;
}

double world_line_offset(void)
{
    vec2 const p = body(g_car.ir_ahead_mm, 0.0);
    double best = 1e9;
    for (int i = 0; i < g_course.n_prims; i++)
    {
        const prim_t *q = &g_course.prims[i];
        if ((PRIM_SEG == q->kind) && (1 == q->path)) { best = fmin(best, seg_dist(p, q->a, q->b)); }
    }
    return best;
}

/* Heading error against the nearest path segment at the IR row, rad. */
double world_heading_error(void)
{
    vec2 const p = body(g_car.ir_ahead_mm, 0.0);
    double best = 1e9, dir = 0.0;
    for (int i = 0; i < g_course.n_prims; i++)
    {
        const prim_t *q = &g_course.prims[i];
        if ((PRIM_SEG == q->kind) && (1 == q->path))
        {
            double const d = seg_dist(p, q->a, q->b);
            if (d < best) { best = d; dir = atan2(q->b.y - q->a.y, q->b.x - q->a.x); }
        }
    }
    return remainder(g_st.h - dir, 2.0 * PI);
}

double world_line_offset_any(void)
{
    vec2 const p = body(g_car.ir_ahead_mm, 0.0);
    double best = 1e9;
    for (int i = 0; i < g_course.n_prims; i++)
    {
        const prim_t *q = &g_course.prims[i];
        if (PRIM_SEG == q->kind) { best = fmin(best, seg_dist(p, q->a, q->b)); }
    }
    return best;
}

/* ---------------------------------------------------------------- terrain */

static double height_at(vec2 p)
{
    double z = 0.0;
    for (int i = 0; i < g_course.n_humps; i++)
    {
        const hump_t *hp = &g_course.humps[i];
        double const dx = p.x - hp->c.x, dy = p.y - hp->c.y;
        double const u = dx * cos(hp->heading) + dy * sin(hp->heading);
        double const v = -dx * sin(hp->heading) + dy * cos(hp->heading);
        if ((u > 0.0) && (u < hp->length) && (fabs(v) < hp->half_width))
        {
            double const s = sin(PI * u / hp->length);
            z += hp->height * s * s;
        }
    }
    return z;
}

/* ---------------------------------------------------------------- sonar */

static double ray_box(vec2 o, double ang, const obstacle_t *b, double *p_inc)
{
    /* Ray against an oriented box, in the box frame (slab method). */
    double const c = cos(b->heading), s = sin(b->heading);
    double const ox = (o.x - b->c.x) * c + (o.y - b->c.y) * s;
    double const oy = -(o.x - b->c.x) * s + (o.y - b->c.y) * c;
    double const dx = cos(ang - b->heading), dy = sin(ang - b->heading);
    double t0 = -1e18, t1 = 1e18;
    int axis = 0;
    double const lo[2] = { -b->half_l, -b->half_w }, hi[2] = { b->half_l, b->half_w };
    double const oo[2] = { ox, oy }, dd[2] = { dx, dy };
    for (int k = 0; k < 2; k++)
    {
        if (fabs(dd[k]) < 1e-12) { if ((oo[k] < lo[k]) || (oo[k] > hi[k])) { return -1.0; } continue; }
        double ta = (lo[k] - oo[k]) / dd[k], tb = (hi[k] - oo[k]) / dd[k];
        if (ta > tb) { double t = ta; ta = tb; tb = t; }
        if (ta > t0) { t0 = ta; axis = k; }
        if (tb < t1) { t1 = tb; }
    }
    if ((t0 > t1) || (t1 < 0.0)) { return -1.0; }
    *p_inc = fabs((0 == axis) ? dx : dy);       /* cos of incidence */
    return (t0 > 0.0) ? t0 : 0.0;
}

static double sonar_range(void)
{
    vec2 const o = body(g_car.us_ahead_mm, 0.0);
    double const beam = g_st.h + (g_st.servo_deg + g_car.servo_mount_left - 90.0) * PI / 180.0;
    double best = -1.0;
    for (int k = -4; k <= 4; k++)
    {
        double const a = beam + (double)k * (7.5 / 4.0) * PI / 180.0;
        for (int i = 0; i < g_course.n_obst; i++)
        {
            double inc = 1.0;
            double const t = ray_box(o, a, &g_course.obst[i], &inc);
            if ((t >= 0.0) && (inc > 0.5) && ((best < 0.0) || (t < best))) { best = t; }
        }
    }
    return best;
}

static void echo_rise(void *arg) { (void)arg; hw_gpio_input(PIN_ECHO, 1); }
static void echo_fall(void *arg) { (void)arg; hw_gpio_input(PIN_ECHO, 0); }

void world_trigger(void)
{
    double r;
    uint64_t rise;
    if (g_car.echo_dead) { return; }
    r = sonar_range();
    if ((g_car.us_floor_mm > 0.0) && ((r < 0.0) || (r > g_car.us_floor_mm)))
    {
        r = g_car.us_floor_mm;
    }
    rise = g_now_us + 460U;
    sim_at(rise, echo_rise, NULL);
    if ((r < 0.0) || (r > 4000.0))
    {
        sim_at(rise + 38000U, echo_fall, NULL);     /* no return: ~38 ms */
        return;
    }
    if ((g_car.us_floor_mm > 0.0) && ((r < 0.0) || (r > g_car.us_floor_mm)))
    {
        r = g_car.us_floor_mm + 8.0 * sim_gauss();      /* the floor ring */
    }
    if (r < 20.0) { r = 20.0; }
    r += 2.0 * sim_gauss();
    sim_at(rise + (uint64_t)(2.0 * r / 0.343), echo_fall, NULL);
}

/* ---------------------------------------------------------------- buttons */

typedef struct { uint32_t pin; int level; } press_t;
static press_t g_press[32];
static int g_npress;

static void press_ev(void *arg) { press_t *p = arg; hw_gpio_input(p->pin, p->level); }

void world_press(uint32_t pin, uint64_t at, uint32_t hold_ms)
{
    if (g_npress > 30) { return; }
    g_press[g_npress] = (press_t){ pin, 0 };
    sim_at(at, press_ev, &g_press[g_npress++]);
    g_press[g_npress] = (press_t){ pin, 1 };
    sim_at(at + (uint64_t)hold_ms * 1000U, press_ev, &g_press[g_npress++]);
}

/* ---------------------------------------------------------------- motors */

double world_duty(int side)
{
    uint32_t const cc = hw_pwm_cc(side ? 5U : 4U);
    double const top = (double)hw_pwm_top(side ? 5U : 4U);
    double const a = (double)(cc & 0xFFFFU), b = (double)(cc >> 16);
    if (top <= 0.0) { return 0.0; }
    return (a - b) / top;
}

bool world_motors_off(void)
{
    return (0U == hw_pwm_cc(4U)) && (0U == hw_pwm_cc(5U));
}

/* Half an edge between the wheels, as on the real car: they tick in
   different 1 ms steps at low speed. */
void world_encoder_phase(void) { g_edge_acc[1] += 0.5; }

static void encoder_edges(int side, double ds)
{
    double const per_mm = g_car.edges_per_rev / (PI * g_car.wheel_dia_mm);
    int const sign = side ? g_car.enc_sign_r : g_car.enc_sign_l;
    uint32_t const pa = side ? PIN_ENC_R_A : PIN_ENC_L_A;
    uint32_t const pb = side ? PIN_ENC_R_B : PIN_ENC_L_B;
    if (!side && g_car.enc_dead_l) { return; }
    g_edge_acc[side] += ds * per_mm;
    while (fabs(g_edge_acc[side]) >= 1.0)
    {
        int const dir = (g_edge_acc[side] > 0.0) ? 1 : -1;
        int const a_new = !g_enc_a[side];
        /* firmware: A == B after an A edge counts +1 (times wiring sign) */
        int const b = ((dir * sign) > 0) ? a_new : !a_new;
        g_edge_acc[side] -= dir;
        hw_gpio_input(pb, b);
        g_enc_a[side] = a_new;
        hw_gpio_input(pa, a_new);
    }
}

static void imu_update(double omega)
{
    double const p0 = g_car.imu_pitch0_deg * PI / 180.0;
    double const th = g_st.pitch + p0;
    double fx = G_MMS2 * sin(th) + g_st.acc_fwd * cos(p0);
    double fz = G_MMS2 * cos(th) - g_st.acc_fwd * sin(p0);
    double fy = ((g_st.vl + g_st.vr) / 2.0) * omega;
    double const n = g_car.imu_noise_mg;
    double mx = fx / G_MMS2 * 1000.0 + n * sim_gauss();
    double my = fy / G_MMS2 * 1000.0 + n * sim_gauss();
    double mz = fz / G_MMS2 * 1000.0 + n * sim_gauss();
    if ((0U != g_car.impact_at_us) && (g_now_us >= g_car.impact_at_us) &&
        (g_now_us < g_car.impact_at_us + 40000U))
    {
        mx -= 4200.0;
    }
    if ((0U != g_car.lift_at_us) && (g_now_us >= g_car.lift_at_us))
    {
        /* Picked up: a push up (unless the scenario only tilts it), then
           held rolled 30 deg in the hand. */
        uint64_t const t = g_now_us - g_car.lift_at_us;
        if ((t < 150000U) && !g_car.lift_tilt_only) { mz += 450.0; }
        else if ((t >= 150000U) && g_car.lift_nose_up)
        {
            mx = 423.0 + n * sim_gauss();               /* nose up 25 deg */
            mz = 906.0 + n * sim_gauss();
        }
        else if (t >= 150000U)
        {
            my = 500.0 + n * sim_gauss();
            mz = 866.0 + n * sim_gauss();
        }
    }
    hw_imu_set_mg(mx, my, mz);
}

static bool collides(void)
{
    double const front = g_car.ir_ahead_mm + 15.0;
    double const rear = g_car.length_mm - front;
    double const hw = g_car.width_mm / 2.0;
    vec2 const corners[4] = { body(front, hw), body(front, -hw), body(-rear, hw), body(-rear, -hw) };
    for (int i = 0; i < g_course.n_obst; i++)
    {
        const obstacle_t *o = &g_course.obst[i];
        double const c = cos(o->heading), s = sin(o->heading);
        /* car corners in the box, and box corners in the car (SAT-lite) */
        for (int k = 0; k < 4; k++)
        {
            double const dx = corners[k].x - o->c.x, dy = corners[k].y - o->c.y;
            if ((fabs(dx * c + dy * s) <= o->half_l) && (fabs(-dx * s + dy * c) <= o->half_w)) { return true; }
        }
        for (int k = 0; k < 4; k++)
        {
            double const bl = (k & 1) ? o->half_l : -o->half_l, bw = (k & 2) ? o->half_w : -o->half_w;
            double const bx = o->c.x + bl * c - bw * s, by = o->c.y + bl * s + bw * c;
            double const dx = bx - g_st.x, dy = by - g_st.y;
            double const al = dx * cos(g_st.h) + dy * sin(g_st.h);
            double const ac = -dx * sin(g_st.h) + dy * cos(g_st.h);
            if ((al <= front) && (al >= -rear) && (fabs(ac) <= hw)) { return true; }
        }
    }
    return false;
}

static void step(void *arg)
{
    double duty[2], v_ss[2];
    double omega, vf, slope;
    (void)arg;

    duty[0] = world_duty(0) * g_car.motor_sign_l;
    duty[1] = world_duty(1) * g_car.motor_sign_r;
    for (int k = 0; k < 2; k++)
    {
        double const mag = fabs(duty[k]);
        double const gain = k ? g_car.gain_r : g_car.gain_l;
        v_ss[k] = (mag <= g_car.deadband) ? 0.0 : ((duty[k] > 0 ? 1.0 : -1.0) * (mag - g_car.deadband) * gain);
        /* climbing load slows the wheel */
        v_ss[k] -= 0.03 * G_MMS2 * sin(g_st.pitch) * ((v_ss[k] != 0.0) ? 1.0 : 0.0);
    }
    {
        double const off = (g_car.coast_s > 0.0) ? g_car.coast_s : 0.03;
        double const tl = (0.0 == duty[0]) ? off : g_car.tau_s;
        double const tr = (0.0 == duty[1]) ? off : g_car.tau_s;
        g_st.vl += (v_ss[0] - g_st.vl) * (DT / tl);
        g_st.vr += (v_ss[1] - g_st.vr) * (DT / tr);
    }
    if (g_car.push_mms != 0.0) { g_st.vl = g_car.push_mms; g_st.vr = g_car.push_mms; }
    if (g_car.wheel_blocked_l) { g_st.vl = 0.0; }
    if (g_st.collided) { g_st.vl = 0.0; g_st.vr = 0.0; }

    if (!g_car.hand_slide)
    {
        encoder_edges(0, g_st.vl * DT);
        encoder_edges(1, g_st.vr * DT);
    }

    vf = 0.5 * (g_st.vl + g_st.vr);
    omega = (g_st.vr - g_st.vl) / g_car.track_mm;
    if ((0U != g_car.lift_at_us) && (g_now_us >= g_car.lift_at_us))
    {
        vf = 0.0; omega = 0.0;          /* off the floor: wheels spin free */
    }
    g_st.h += omega * DT;
    g_st.x += vf * cos(g_st.pitch) * cos(g_st.h) * DT;
    g_st.y += vf * cos(g_st.pitch) * sin(g_st.h) * DT;
    g_st.dist += fabs(vf) * DT;

    {
        vec2 const p = { g_st.x, g_st.y };
        double const z0 = height_at(fwd(p, g_st.h, -1.0));
        double const z1 = height_at(fwd(p, g_st.h, 1.0));
        slope = (z1 - z0) / 2.0;
        g_st.pitch = atan(slope);
        g_st.z = height_at(p);
        if (g_st.z > g_st.max_hump_true) { g_st.max_hump_true = g_st.z; }
    }
    g_st.acc_fwd += 0.3 * (((vf - g_vfwd_prev) / DT) - g_st.acc_fwd);
    g_vfwd_prev = vf;

    /* servo: pulse width (1 us ticks) -> commanded angle, slew limited */
    {
        double const pulse = (double)(hw_pwm_cc(6U) & 0xFFFFU);
        double cmd = (pulse - 1000.0) * 180.0 / 1000.0;
        double const max_step = g_car.servo_dps * DT;
        if (pulse < 500.0) { cmd = g_st.servo_deg; }        /* not driven */
        if (g_car.servo_continuous)
        {
            /* FS90R: 1500 us stops; +-100 us turns at ~120 deg/s */
            double rate = (pulse < 500.0) ? 0.0 : (pulse - 1500.0) * 1.2;
            if (g_car.servo_mirror > 0.5) { rate = -rate; }
            g_st.servo_deg += rate * DT;
            cmd = g_st.servo_deg;
        }
        if (g_car.servo_mirror > 0.5) { cmd = 180.0 - cmd; }
        cmd = fmin(180.0, fmax(0.0, cmd));
        g_st.servo_deg += fmax(-max_step, fmin(max_step, cmd - g_st.servo_deg));
    }

    imu_update(omega);

#ifndef PIN_IR_BARCODE
#define PIN_IR_BARCODE 28U      /* app.h: barcode sensor D0 */
#endif
    /* Barcode sensor D0: the board's comparator, half covered = black,
       with a little hysteresis as on the LM393 boards. */
    {
        static int s_black = 0;
        double const f = coverage(body(g_car.ir_ahead_mm, -g_car.bc_lat_mm));
        if (f > 0.55) { s_black = 1; } else if (f < 0.45) { s_black = 0; }
        hw_gpio_input(PIN_IR_BARCODE, g_car.d0_black_high ? s_black : !s_black);
    }

    if (!g_st.collided && (g_course.n_obst > 0) && collides())
    {
        g_st.collided = true;
        g_car.impact_at_us = g_now_us;
    }
}

void world_start(void)
{
    memset(&g_st, 0, sizeof(g_st));
    g_st.x = g_car.x0; g_st.y = g_car.y0; g_st.h = g_car.h0_deg * PI / 180.0;
    g_st.servo_deg = 90.0;
    g_edge_acc[0] = g_edge_acc[1] = 0.0;
    g_enc_a[0] = g_enc_a[1] = 0;
    hw_gpio_input(PIN_START, 1);        /* buttons idle high (pull-up) */
    hw_gpio_input(PIN_ESTOP, 1);
    imu_update(0.0);
    sim_every_ms(step, NULL);
}
