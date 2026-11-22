/* PicoCar host simulator: shared declarations.
 *
 *   sim_kernel.c  virtual-time micro T-Kernel model (tasks, flags, message
 *                 buffers, mutex, cyclic handlers, interrupts, console)
 *   sim_hw.c      RP2040 registers the drivers touch: TIMER, RESETS, ADC,
 *                 I2C0 + LSM303DLHC, PWM, IO_BANK0 interrupts, WDT, GPIO
 *   sim_world.c   the car and the course: motors, wheels, encoders, IR
 *                 sensors, barcodes, humps, IMU, HC-SR04 + servo, buttons
 *   sim_main.c    scenarios, oracles, and the multi-seed runner
 */
#ifndef SIM_H
#define SIM_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <tk/tkernel.h>

/* ---------------- kernel ---------------- */
extern uint64_t g_now_us;

typedef void (*sim_event_fn)(void *arg);

void     sim_kernel_init(uint32_t seed, uint32_t cost_min_us,
                         uint32_t cost_max_us);
void     sim_boot(void (*initial)(void));      /* runs usermain at pri 1 */
void     sim_run_until(uint64_t t_us);
void     sim_at(uint64_t t_us, sim_event_fn fn, void *arg);
void     sim_every_ms(sim_event_fn fn, void *arg);    /* each tick, first */
void     sim_raise_irq(UINT intno);
bool     sim_in_isr(void);
uint32_t sim_rand(void);
double   sim_randf(void);                       /* [0, 1) */
double   sim_gauss(void);
const char *sim_task_name(void);

typedef struct
{
    uint32_t switches;
    uint32_t preemptions;
    uint32_t di_violations;     /* blocking call while interrupts masked */
    uint32_t isr_kernel_misuse; /* blocking call from an ISR             */
    uint32_t mbf_send_fail;     /* tk_snd_mbf returned an error          */
    uint32_t error_lines;       /* console lines starting "ERROR"        */
    uint32_t max_wup_backlog;   /* largest tk_can_wup() result           */
    uint32_t mtx_max_hold_us;
} sim_kstats_t;
extern sim_kstats_t g_kstats;

typedef struct
{
    const char *name;
    int         pri;
    uint32_t    releases;       /* times the task left a wait state     */
    uint64_t    max_latency_us; /* cyclic release -> task running       */
} sim_task_info_t;
int  sim_task_count(void);
void sim_task_info(int idx, sim_task_info_t *p);

/* Console capture: every tm_printf line, with time and task. */
typedef void (*sim_console_fn)(uint64_t t_us, const char *task,
                               const char *line);
void sim_console_hook(sim_console_fn fn);
void sim_console_file(FILE *fp);

/* ---------------- hardware ---------------- */
#define SIM_IRQ_IO_BANK0    13U

void     hw_init(void);
void     hw_gpio_input(uint32_t pin, int level);   /* edges raise IRQ 13 */
int      hw_gpio_out(uint32_t pin);
uint32_t hw_pwm_cc(uint32_t slice);
uint32_t hw_pwm_top(uint32_t slice);
uint32_t hw_pwm_div(uint32_t slice);
void     hw_imu_set_mg(double x, double y, double z);
void     hw_imu_reset_regs(void);
uint32_t hw_wdt_feeds(void);
uint64_t hw_wdt_max_gap_us(void);
bool     hw_wdt_enabled(void);

typedef struct
{
    bool     adc_stuck;         /* ADC never sets READY                 */
    bool     adc_late;          /* 2 % of conversions take 40 us        */
    bool     i2c_absent;        /* no ACK from 0x19                      */
    bool     i2c_flaky;         /* abort ~1 transfer in 3                */
    int      imu_chip;          /* 0 LSM303DLHC 0x19, 1 LSM303D 0x1D, 2 MPU-6050 0x68 */
    double   imu_gain;          /* 0 = 1.0: scale error of the part (real boards: +-10 %) */
    int      imu_mount;         /* 0 as WIRING.md, 1 upside down (X fwd), 2 upside down (X back) */
    double   imu_drive_tilt;    /* deg added to the pitch while the wheels turn (vibration, body squat) */
    double   imu_vibe_mg;       /* driving vibration: white noise this size, plus body rocking */
    uint32_t i2c_fail_from_ms;  /* 0 = never: bus dies at this time     */
} hw_faults_t;
extern hw_faults_t g_hw_faults;

/* World callbacks used by sim_hw.c */
uint16_t world_adc(uint32_t channel);
void     world_trigger(void);                 /* HC-SR04 TRIG fell     */

/* ---------------- world ---------------- */
#define MAX_PRIMS       1024
#define MAX_HUMPS       4
#define MAX_OBST        4

typedef struct { double x, y; } vec2;

typedef struct
{
    /* car (true values; the firmware has its own provisional ones) */
    double wheel_dia_mm, track_mm, edges_per_rev;
    double gain_l, gain_r;            /* mm/s per unit duty above deadband */
    double deadband;
    double tau_s;
    double coast_s;             /* spin-down time constant, motors off (0 = 0.03 s) */
    int    enc_sign_l, enc_sign_r;    /* wiring: +1 forward counts up      */
    int    motor_sign_l, motor_sign_r;
    double ir_ahead_mm, ir_half_mm, bc_lat_mm, ir_spot_mm;
    double ir_line_spot_mm;     /* line sensors' spot, 0 = ir_spot_mm (real ones are wider) */
    double push_mms;    /* != 0: car rolled by hand at this speed (< 0 backwards), motors ignored */
    bool   hand_slide;
    bool   d0_black_high; /* barcode D0 polarity (SIM_D0=1: 1 over black) */  /* the barcode moves, not the car: encoders stay still */
    double ir_white, ir_black, ir_noise;
    double ir_white_l, ir_black_l;    /* left sensor, if set (0 = same) */
    double us_ahead_mm;
    double us_floor_mm;               /* low sonar: floor echo all round */
    double servo_mirror;              /* 1: real servo turns the other way */
    double servo_dps;
    double servo_mount_left;
    bool   servo_continuous;          /* FS90R: pulse sets speed */          /* sonar points this far left at centre */
    double imu_pitch0_deg, imu_noise_mg;
    double width_mm, length_mm;       /* footprint for collision checks    */
    /* start pose */
    double x0, y0, h0_deg;
    /* faults */
    bool   enc_dead_l;
    bool   wheel_blocked_l;
    bool   echo_dead;                 /* sensor never answers              */
    bool   echo_clone_high;           /* holds ECHO high 38 ms on no return */
    uint64_t impact_at_us;            /* scripted 4 g bump                 */
    uint64_t lift_at_us;              /* scripted pick-up: push up, roll   */
    bool     lift_tilt_only;          /* pick-up felt only as a slow tilt  */
    bool     lift_nose_up;            /* held nose up 25 deg after the lift */
} car_params_t;

typedef enum { PRIM_SEG, PRIM_RECT } prim_kind_t;
typedef struct
{
    prim_kind_t kind;
    vec2 a, b;           /* SEG: centre line a->b; RECT: oriented box    */
    double half_w;       /* SEG: half width; RECT: half extent across    */
    double minx, miny, maxx, maxy;
    int path;            /* SEG belonging to the intended path: 1        */
} prim_t;

typedef struct
{
    vec2 c; double heading;  /* axis along which the hump rises          */
    double length, height, half_width;
} hump_t;

typedef struct { vec2 c; double heading, half_l, half_w; } obstacle_t;

typedef struct
{
    prim_t      prims[MAX_PRIMS];
    int         n_prims;
    hump_t      humps[MAX_HUMPS];
    int         n_humps;
    obstacle_t  obst[MAX_OBST];
    int         n_obst;
    /* builder cursor */
    vec2        cur;
    double      cur_h;
    double      line_w;
    double      s;           /* path length so far                     */
} course_t;

typedef struct
{
    double x, y, h;          /* axle centre, heading rad (CCW, x east) */
    double vl, vr;           /* wheel surface speeds, mm/s             */
    double pitch;            /* rad, nose up positive                  */
    double z;                /* height of the axle point, mm           */
    double acc_fwd;          /* mm/s^2                                 */
    double servo_deg;        /* actual servo angle                     */
    double dist;             /* true path length driven, mm            */
    bool   collided;
    double max_hump_true;    /* highest hump crossed, mm               */
} car_state_t;

extern car_params_t g_car;
extern course_t     g_course;
extern car_state_t  g_st;

void   world_defaults(void);
void   world_start(void);    /* registers the 1 ms step, puts the car down */
void   course_reset(double line_w);
void   world_encoder_phase(void);
void   course_mark(double black_mm, double white_mm);
void   course_start(double x, double y, double h_deg);
void   course_straight(double len);
void   course_arc(double radius, double deg);      /* + = left            */
void   course_branch(double rel_deg, double len);  /* line off the cursor */
void   course_width(double w);                    /* line width from here, mm */
void   course_barcode(char c, double narrow, double wide, int crossing);
void   course_end_marker(double len, double width);
void   course_gap(double len);                     /* break in the line   */
void   course_hump(double length, double height);  /* at the cursor       */
void   course_obstacle(double ahead, double lat, double w, double d);
double world_line_offset(void);       /* IR row to nearest path line, mm */
double world_line_offset_any(void);   /* to nearest line of any kind     */
void   world_press(uint32_t pin, uint64_t at_us, uint32_t hold_ms);
bool   world_motors_off(void);
double world_duty(int side);

#endif
