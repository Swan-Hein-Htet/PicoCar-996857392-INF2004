/* RP2040 peripherals as seen by the real drivers (register level):
 * TIMER, RESETS, SIO CPUID, ADC, I2C0 with an LSM303DLHC behind it, PWM,
 * IO_BANK0 edge interrupts, PADS, WDT, and the libbsp GPIO/PWM helpers.
 * Unknown addresses are plain storage.
 */
#include "sim.h"
#include <string.h>
#include <math.h>
#include <bsp/libbsp.h>

extern void sim_charge_us(uint32_t us);

hw_faults_t g_hw_faults;

/* ---------------------------------------------------------- storage */
#define MAP_SIZE 4096
static UW g_key[MAP_SIZE];
static UW g_val[MAP_SIZE];
static bool g_used[MAP_SIZE];

static UW *slot(UW addr)
{
    uint32_t h = (addr * 2654435761U) % MAP_SIZE;
    while (g_used[h] && (g_key[h] != addr)) { h = (h + 1U) % MAP_SIZE; }
    if (!g_used[h]) { g_used[h] = true; g_key[h] = addr; g_val[h] = 0U; }
    return &g_val[h];
}

/* ---------------------------------------------------------- addresses */
#define TIMER_RAW_LOW   0x40054028U
#define SIO_CPUID       (SIO_BASE + 0x000U)
#define ADC_CS          0x4004C000U
#define ADC_RESULT      0x4004C004U
#define I2C0            0x40044000U
#define IC_DATA_CMD     (I2C0 + 0x10U)
#define IC_RAW_INTR     (I2C0 + 0x34U)
#define IC_CLR_INTR     (I2C0 + 0x40U)
#define IC_CLR_TX_ABRT  (I2C0 + 0x54U)
#define IC_CLR_STOP_DET (I2C0 + 0x60U)
#define IC_ENABLE       (I2C0 + 0x6CU)
#define IC_TAR          (I2C0 + 0x04U)
#define IC_RXFLR        (I2C0 + 0x78U)
#define IO_INTR0        (IO_BANK0_BASE + 0x0F0U)
#define IO_P0_INTE0     (IO_BANK0_BASE + 0x100U)
#define IO_P0_INTS0     (IO_BANK0_BASE + 0x120U)
#define IO_P1_INTE0     (IO_BANK0_BASE + 0x130U)
#define IO_P1_INTS0     (IO_BANK0_BASE + 0x150U)
#define WDT_CTRL        0x40058000U
#define WDT_LOAD        0x40058004U
#define PWM_SLICE(s)    (PWM_BASE + ((s) * 0x14U))

/* ---------------------------------------------------------- ADC */
static uint64_t g_adc_ready_at;
static UW       g_adc_result;

/* ---------------------------------------------------------- I2C + LSM */
static uint8_t  g_lsm[0x80];
static uint8_t  g_rx[32];
static int      g_rx_n;
static int      g_rx_head;
static bool     g_new_txn = true;
static uint8_t  g_ptr;
static bool     g_autoinc;
static UW       g_raw_intr;
static uint32_t g_xfer;
static double   g_imu_mg[3] = { 0.0, 0.0, 1000.0 };

#define LSM_CTRL1 0x20U
#define LSM_CTRL4 0x23U
#define LSM_OUT_X 0x28U

void hw_imu_set_mg(double x, double y, double z)
{
    double const k = (g_hw_faults.imu_gain > 0.0) ? g_hw_faults.imu_gain : 1.0;
    if (0.0 != g_hw_faults.imu_drive_tilt)
    {   /* the real car read about 1 deg more nose-down driving than parked:
           grows with speed, full at 100 mm/s */
        double const v = fmin(1.0, (fabs(g_st.vl) + fabs(g_st.vr)) / 200.0);
        double const t = g_hw_faults.imu_drive_tilt * v * M_PI / 180.0;
        double const x0 = x;
        x = x0 * cos(t) - z * sin(t);
        z = x0 * sin(t) + z * cos(t);
    }
    if ((0.0 != g_hw_faults.imu_vibe_mg) && (fabs(g_st.vl) + fabs(g_st.vr) > 20.0))
    {   /* the real car's pitch swung +-2-4 deg driving (2026-11-19) */
        double const n = g_hw_faults.imu_vibe_mg;
        double const rock = 0.6 * n * sin(2.0 * M_PI * 2.0 * (double)g_now_us / 1e6);
        x += (sim_randf() - 0.5) * 2.0 * n + rock;
        y += (sim_randf() - 0.5) * 2.0 * n;
        z += (sim_randf() - 0.5) * n;
    }
    if (1 == g_hw_faults.imu_mount) { y = -y; z = -z; }        /* turned over about X */
    if (2 == g_hw_faults.imu_mount) { x = -x; z = -z; }        /* turned over about Y */
    if (3 == g_hw_faults.imu_mount) { double t = x; x = y; y = -t; }      /* turned 90 deg: X across the car */
    if (4 == g_hw_faults.imu_mount) { double t = x; x = y; y = t; z = -z; } /* turned 90 deg and over (the real car?) */
    g_imu_mg[0] = x * k; g_imu_mg[1] = y * k; g_imu_mg[2] = z * k;
}

/* Other parts the GY-511 slot has been seen with (g_hw_faults.imu_chip). */
static uint8_t imu_addr(void)
{
    return (uint8_t)((1 == g_hw_faults.imu_chip) ? 0x1DU : (2 == g_hw_faults.imu_chip) ? 0x68U : 0x19U);
}
static void put16(int reg, double counts, bool big)
{
    long c = lround(counts);
    if (c > 32767) { c = 32767; }
    if (c < -32768) { c = -32768; }
    uint16_t const w = (uint16_t)(int16_t)c;
    g_lsm[reg] = (uint8_t)(big ? (w >> 8) : (w & 0xFFU));
    g_lsm[reg + 1] = (uint8_t)(big ? (w & 0xFFU) : (w >> 8));
}
static void lsmd_latch(void)
{
    static const double mg_lsb[5] = { 0.061, 0.122, 0.183, 0.244, 0.732 };
    int const afs = (g_lsm[0x21] >> 3) & 7;
    bool const on = (0U != (g_lsm[0x20] & 0xF0U));
    for (int ax = 0; ax < 3; ax++) { put16(0x28 + 2 * ax, on ? g_imu_mg[ax] / mg_lsb[afs > 4 ? 4 : afs] : 0.0, false); }
}
static void mpu_latch(void)
{
    static const double lsb_g[4] = { 16384.0, 8192.0, 4096.0, 2048.0 };
    bool const awake = (0U == (g_lsm[0x6B] & 0x40U));
    double const k = lsb_g[(g_lsm[0x1C] >> 3) & 3] / 1000.0;
    for (int ax = 0; ax < 3; ax++) { put16(0x3B + 2 * ax, awake ? g_imu_mg[ax] * k : 0.0, true); }
}
void hw_imu_reset_regs(void)
{
    memset(g_lsm, 0, sizeof(g_lsm));
    if (1 == g_hw_faults.imu_chip) { g_lsm[0x0F] = 0x49U; }
    if (2 == g_hw_faults.imu_chip) { g_lsm[0x75] = 0x68U; g_lsm[0x6B] = 0x40U; }   /* asleep at reset */
}

/* Latch the output registers as the part does (BDU: one sample/burst). */
static void lsm_latch(void)
{
    uint8_t const fs = (uint8_t)((g_lsm[LSM_CTRL4] >> 4) & 3U);
    bool const    hr = (0U != (g_lsm[LSM_CTRL4] & 0x08U));
    static const double mg_per_lsb_hr[4] = { 1.0, 2.0, 4.0, 12.0 };
    double const  mg_lsb = mg_per_lsb_hr[fs] * (hr ? 1.0 : 4.0);
    double const  range = (double)(2 << fs) * 1000.0;
    bool const    on = (0U != (g_lsm[LSM_CTRL1] & 0xF0U)) &&
                       (0U != (g_lsm[LSM_CTRL1] & 0x07U));

    for (int ax = 0; ax < 3; ax++)
    {
        double mg = on ? g_imu_mg[ax] : 0.0;
        long counts;
        int16_t raw;
        if (mg > range) { mg = range; }
        if (mg < -range) { mg = -range; }
        counts = lround(mg / mg_lsb);
        if (hr) { raw = (int16_t)(counts * 16); }      /* 12-bit, left-justified */
        else    { raw = (int16_t)(counts * 64); }      /* 10-bit */
        g_lsm[LSM_OUT_X + (2 * ax)] = (uint8_t)((uint16_t)raw & 0xFFU);
        g_lsm[LSM_OUT_X + (2 * ax) + 1] = (uint8_t)((uint16_t)raw >> 8);
    }
}

static bool i2c_dead(void)
{
    if (g_hw_faults.i2c_absent) { return true; }
    if ((0U != g_hw_faults.i2c_fail_from_ms) &&
        (g_now_us >= (uint64_t)g_hw_faults.i2c_fail_from_ms * 1000U)) { return true; }
    return false;
}

static void i2c_cmd(UW v)
{
    bool const stop = (0U != (v & 0x200U));
    bool const read = (0U != (v & 0x100U));

    if (imu_addr() != (*slot(IC_TAR) & 0x3FFU) || (0U == (*slot(IC_ENABLE) & 1U)) ||
        i2c_dead() || (g_hw_faults.i2c_flaky && g_new_txn && ((sim_rand() % 3U) == 0U)))
    {
        g_raw_intr |= 0x040U;                   /* TX_ABRT */
        g_raw_intr |= 0x200U;                   /* STOP_DET */
        g_rx_n = 0; g_rx_head = 0;
        g_new_txn = true;
        return;
    }
    if (g_new_txn) { g_xfer++; }
    if (read)
    {
        if (g_rx_n < 32) { g_rx[(g_rx_head + g_rx_n) % 32] = g_lsm[g_ptr & 0x7FU]; g_rx_n++; }
        if (g_autoinc || (2 == g_hw_faults.imu_chip)) { g_ptr++; }   /* the MPU always auto-increments */
    }
    else if (g_new_txn)
    {
        g_ptr = (uint8_t)(v & 0x7FU);
        g_autoinc = (2 != g_hw_faults.imu_chip) && (0U != (v & 0x80U));
        if (2 == g_hw_faults.imu_chip) { if (0x3BU == g_ptr) { mpu_latch(); } }
        else if (1 == g_hw_faults.imu_chip) { if (LSM_OUT_X == g_ptr) { lsmd_latch(); } }
        else if (LSM_OUT_X == g_ptr) { lsm_latch(); }
        g_new_txn = false;
    }
    else
    {
        g_lsm[g_ptr & 0x7FU] = (uint8_t)v;
        if (g_autoinc) { g_ptr++; }
    }
    if (stop) { g_raw_intr |= 0x200U; g_new_txn = true; }
}

/* ---------------------------------------------------------- GPIO */
static int  g_in[32];
static int  g_out[32];
static int  g_mode[32];
static bool g_trig_high;

#define PIN_TRIG 2U

void hw_gpio_input(uint32_t pin, int level)
{
    UW *intr;
    level = level ? 1 : 0;
    if (g_in[pin] == level) { return; }
    g_in[pin] = level;
    intr = slot(IO_INTR0 + ((pin / 8U) * 4U));
    *intr |= (level ? 0x8U : 0x4U) << ((pin % 8U) * 4U);
    sim_raise_irq(SIM_IRQ_IO_BANK0);
}

int hw_gpio_out(uint32_t pin) { return g_out[pin]; }

ER gpio_set_pin(UINT no, UINT mode) { if (no >= 30U) { return E_PAR; } g_mode[no] = (int)mode; return E_OK; }

ER gpio_set_val(UINT no, UINT val)
{
    if (no >= 30U) { return E_PAR; }
    g_out[no] = val ? 1 : 0;
    if (PIN_TRIG == no)
    {
        if (val) { g_trig_high = true; }
        else if (g_trig_high) { g_trig_high = false; world_trigger(); }
    }
    return E_OK;
}

UINT gpio_get_val(UINT no)
{
    if (no >= 30U) { return 0U; }
    return (UINT)((GPIO_MODE_OUT == g_mode[no]) ? g_out[no] : g_in[no]);
}

/* ---------------------------------------------------------- PWM (libbsp) */
#define PWM_CHNO(pin) (((pin) >> 1) & 7U)
ER pwm_set_pin(UINT no) { out_w(GPIO_CTRL(no), 4U); return E_OK; }
ER pwm_set_wrap(UINT no, UW wrap) { out_w(PWM_SLICE(PWM_CHNO(no)) + PWM_CHx_TOP, wrap); return E_OK; }
ER pwm_set_cc(UINT no, UW cc)
{
    UW const addr = PWM_SLICE(PWM_CHNO(no)) + PWM_CHx_CC;
    UW const reg = in_w(addr);
    out_w(addr, (no & 1U) ? ((reg & 0x0000FFFFU) | (cc << 16)) : ((reg & 0xFFFF0000U) | (cc & 0xFFFFU)));
    return E_OK;
}
ER pwm_set_enabled(UINT no, BOOL en)
{
    UW const addr = PWM_SLICE(PWM_CHNO(no)) + PWM_CHx_CSR;
    out_w(addr, en ? (in_w(addr) | 1U) : (in_w(addr) & ~1U));
    return E_OK;
}
uint32_t hw_pwm_cc(uint32_t s)  { return *slot(PWM_SLICE(s) + PWM_CHx_CC); }
uint32_t hw_pwm_top(uint32_t s) { return *slot(PWM_SLICE(s) + PWM_CHx_TOP); }
uint32_t hw_pwm_div(uint32_t s) { return *slot(PWM_SLICE(s) + PWM_CHx_DIV); }

/* ---------------------------------------------------------- WDT */
static bool     g_wdt_on;
static uint64_t g_wdt_last;
static uint64_t g_wdt_gap;
static uint32_t g_wdt_feeds;

uint32_t hw_wdt_feeds(void) { return g_wdt_feeds; }
uint64_t hw_wdt_max_gap_us(void)
{
    uint64_t gap = g_now_us - g_wdt_last;
    return (g_wdt_on && (gap > g_wdt_gap)) ? gap : g_wdt_gap;
}
bool hw_wdt_enabled(void) { return g_wdt_on; }

/* ---------------------------------------------------------- bus */

void out_w(UW addr, UW data)
{
    if (ADC_CS == addr)
    {
        *slot(addr) = data & ~0x100U;
        if (0U != (data & 0x4U))
        {
            g_adc_result = world_adc((data >> 12) & 7U) & 0xFFFU;
            g_adc_ready_at = g_hw_faults.adc_stuck ? UINT64_MAX : (g_now_us + 2U);
            if (g_hw_faults.adc_late && (0U == (sim_rand() % 50U))) { g_adc_ready_at = g_now_us + 40U; }
        }
        return;
    }
    if (IC_DATA_CMD == addr) { i2c_cmd(data); return; }
    if ((addr >= IO_INTR0) && (addr < IO_INTR0 + 16U)) { *slot(addr) &= ~data; return; }
    if (WDT_CTRL == addr)
    {
        *slot(addr) = data;
        if ((0U != (data & 0x40000000U)) && !g_wdt_on) { g_wdt_on = true; g_wdt_last = g_now_us; }
        return;
    }
    if (WDT_LOAD == addr)
    {
        *slot(addr) = data;
        if (g_wdt_on)
        {
            uint64_t const gap = g_now_us - g_wdt_last;
            if (gap > g_wdt_gap) { g_wdt_gap = gap; }
        }
        g_wdt_last = g_now_us;
        g_wdt_feeds++;
        return;
    }
    *slot(addr) = data;
}

UW in_w(UW addr)
{
    if (TIMER_RAW_LOW == addr)
    {
        UW const t = (UW)g_now_us;
        sim_charge_us(1U);          /* a busy-wait burns real time */
        return t;
    }
    if (RESETS_RESET_DONE == addr) { return 0xFFFFFFFFU; }
    if (SIO_CPUID == addr) { return 1U; }      /* every app task: core 1 */
    if (ADC_CS == addr) { return *slot(addr) | ((g_now_us >= g_adc_ready_at) ? 0x100U : 0U); }
    if (ADC_RESULT == addr) { return g_adc_result; }
    if (IC_RAW_INTR == addr) { return g_raw_intr; }
    if (IC_CLR_INTR == addr) { g_raw_intr = 0U; return 0U; }
    if (IC_CLR_TX_ABRT == addr) { g_raw_intr &= ~0x040U; return 0U; }
    if (IC_CLR_STOP_DET == addr) { g_raw_intr &= ~0x200U; return 0U; }
    if (IC_RXFLR == addr) { return (UW)g_rx_n; }
    if (IC_DATA_CMD == addr)
    {
        UW v = 0U;
        if (g_rx_n > 0) { v = g_rx[g_rx_head]; g_rx_head = (g_rx_head + 1) % 32; g_rx_n--; }
        return v;
    }
    if ((addr >= IO_P0_INTS0) && (addr < IO_P0_INTS0 + 16U))
    {
        return *slot(IO_INTR0 + (addr - IO_P0_INTS0)) & *slot(IO_P0_INTE0 + (addr - IO_P0_INTS0));
    }
    if ((addr >= IO_P1_INTS0) && (addr < IO_P1_INTS0 + 16U))
    {
        return *slot(IO_INTR0 + (addr - IO_P1_INTS0)) & *slot(IO_P1_INTE0 + (addr - IO_P1_INTS0));
    }
    return *slot(addr);
}

void set_w(UW addr, UW data) { out_w(addr, in_w(addr) | data); }
void clr_w(UW addr, UW data) { out_w(addr, in_w(addr) & ~data); }

void hw_init(void)
{
    memset(g_lsm, 0, sizeof(g_lsm));
    g_lsm[LSM_CTRL1] = 0x07U;      /* power-on default: power-down, XYZ */
    g_lsm[0x0F] = 0x33U;
    g_adc_ready_at = UINT64_MAX;
}
