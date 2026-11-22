/** @file drv_lsm303.c
 *
 * @brief Register-level I2C0 master and accelerometer driver: probes the
 *        LSM303DLHC/AGR (0x19/0x18), LSM303D (0x1D/0x1E) and MPU-6050/6500
 *        (0x68/0x69), and can report bus levels and scan the bus.
 *        Every wait is bounded by the 1 MHz hardware timer.
 */

#include "../app.h"
#include "drv_lsm303.h"

/* RP2040 I2C0 (DW_apb_i2c) */
#define I2C0_BASE               0x40044000U
#define IC_CON                  (I2C0_BASE + 0x00U)
#define IC_TAR                  (I2C0_BASE + 0x04U)
#define IC_DATA_CMD             (I2C0_BASE + 0x10U)
#define IC_FS_SCL_HCNT          (I2C0_BASE + 0x1CU)
#define IC_FS_SCL_LCNT          (I2C0_BASE + 0x20U)
#define IC_RAW_INTR_STAT        (I2C0_BASE + 0x34U)
#define IC_CLR_INTR             (I2C0_BASE + 0x40U)
#define IC_CLR_TX_ABRT          (I2C0_BASE + 0x54U)
#define IC_CLR_STOP_DET         (I2C0_BASE + 0x60U)
#define IC_ENABLE               (I2C0_BASE + 0x6CU)
#define IC_TXFLR                (I2C0_BASE + 0x74U)
#define IC_RXFLR                (I2C0_BASE + 0x78U)
#define IC_SDA_HOLD             (I2C0_BASE + 0x7CU)
#define IC_FS_SPKLEN            (I2C0_BASE + 0xA0U)

#define IC_CON_FAST_MASTER      0x0165U /* master, 400k, restart, no slave,
                                           TX_EMPTY_CTRL */
#define IC_CMD_READ             0x100U
#define IC_CMD_STOP             0x200U
#define IC_CMD_RESTART          0x400U
#define IC_RAW_TX_ABRT          0x040U
#define IC_RAW_STOP_DET         0x200U
#define IC_TX_FIFO_DEPTH        16U

/* 400 kHz from clk_sys = 125 MHz (pico-sdk i2c_set_baudrate arithmetic) */
#define I2C_HCNT                126U
#define I2C_LCNT                187U
#define I2C_SPKLEN              11U
#define I2C_SDA_HOLD            38U

#define RESET_I2C0              0x00000008U
#define PAD_PUE                 0x08U
#define PAD_IE                  0x40U
#define PAD_SCHMITT             0x02U
#define FUNC_I2C                3U

#define I2C_TIMEOUT_US          1000U

/* Accelerometers the GY-511 slot has been seen with.  The probe tries them
   in this order; each has its own set-up and scaling to milli-g. */
#define LSM_CTRL_REG1_A         0x20U   /* DLHC / AGR / LIS3DH, and LSM303D CTRL1 */
#define LSM_CTRL_REG4_A         0x23U
#define LSM_OUT_X_L_A           0x28U
#define LSM_AUTO_INC            0x80U
#define LSM_REG1_100HZ_XYZ      0x57U
#define LSM_REG4_BDU_8G_HR      0xA8U
#define LSM_LSB_PER_COUNT       16      /* 12-bit left-justified          */
#define LSM_MG_PER_COUNT        4       /* +-8 g, high resolution         */
#define LSM_AXES                3U
#define LSMD_WHO_AM_I           0x0FU   /* LSM303D: reads 0x49            */
#define LSMD_ID                 0x49U
#define LSMD_CTRL1_100HZ_BDU    0x5FU   /* 100 Hz, BDU, X Y Z on          */
#define LSMD_CTRL2              0x21U
#define LSMD_CTRL2_8G           0x18U   /* AFS = +-8 g: 0.244 mg/LSB      */
#define LSMD_UG_PER_COUNT       244
#define MPU_WHO_AM_I            0x75U   /* 0x68 6050, 0x70 6500, 0x71 9250 */
#define MPU_PWR_MGMT_1          0x6BU
#define MPU_CLK_PLL             0x01U   /* awake, gyro X PLL clock         */
#define MPU_CONFIG              0x1AU
#define MPU_DLPF_44HZ           0x03U
#define MPU_ACCEL_CONFIG        0x1CU
#define MPU_ACCEL_8G            0x10U   /* +-8 g: 4096 LSB/g               */
#define MPU_ACCEL_XOUT_H        0x3BU
#define MPU_LSB_PER_G           4096
#define SCAN_FIRST              0x08U
#define SCAN_LAST               0x77U

typedef enum { PART_NONE, PART_DLHC, PART_LSMD, PART_MPU } part_t;
static part_t  g_part = PART_NONE;
static uint8_t g_addr = 0U;
static char const * const PART_NAME[] = {
    "none", "LSM303DLHC/AGR", "LSM303D", "MPU-6050/6500" };

static bool
wait_set (uint32_t reg, uint32_t mask)
{
    uint32_t const start = app_micros();

    while (0U == (in_w(reg) & mask))
    {
        if ((app_micros() - start) > I2C_TIMEOUT_US)
        {
            return (false);
        }
    }
    return (true);
}

/* Finish a transfer: wait for STOP, report (and clear) any abort. */
static bool
i2c_finish (void)
{
    bool const b_stopped = wait_set(IC_RAW_INTR_STAT, IC_RAW_STOP_DET);
    bool const b_abort =
        (0U != (in_w(IC_RAW_INTR_STAT) & IC_RAW_TX_ABRT));

    (void)in_w(IC_CLR_STOP_DET);
    (void)in_w(IC_CLR_TX_ABRT);
    return (b_stopped && !b_abort);
}

static bool
i2c_write_reg (uint8_t reg, uint8_t value)
{
    (void)in_w(IC_CLR_INTR);
    out_w(IC_DATA_CMD, (uint32_t)reg);
    out_w(IC_DATA_CMD, (uint32_t)value | IC_CMD_STOP);
    return (i2c_finish());
}

static bool
i2c_read_regs (uint8_t reg, uint8_t * p_buf, uint32_t count)
{
    uint32_t idx;

    (void)in_w(IC_CLR_INTR);
    out_w(IC_DATA_CMD, (uint32_t)reg);

    for (idx = 0U; idx < count; idx++)
    {
        uint32_t cmd = IC_CMD_READ;

        if (0U == idx)
        {
            cmd |= IC_CMD_RESTART;
        }
        if ((count - 1U) == idx)
        {
            cmd |= IC_CMD_STOP;
        }
        out_w(IC_DATA_CMD, cmd);
    }

    for (idx = 0U; idx < count; idx++)
    {
        if (!wait_set(IC_RXFLR, 0xFFU))
        {
            (void)i2c_finish();
            return (false);
        }
        p_buf[idx] = (uint8_t)in_w(IC_DATA_CMD);
    }

    return (i2c_finish());
}

static void
i2c_bus_init (void)
{
    uint32_t const start = app_micros();
    uint32_t       pin;

    clr_w(RESETS_RESET, RESET_I2C0);
    while (0U == (in_w(RESETS_RESET_DONE) & RESET_I2C0))
    {
        if ((app_micros() - start) > I2C_TIMEOUT_US)
        {
            break;
        }
    }

    out_w(IC_ENABLE, 0U);
    out_w(IC_CON, IC_CON_FAST_MASTER);
    out_w(IC_FS_SCL_HCNT, I2C_HCNT);
    out_w(IC_FS_SCL_LCNT, I2C_LCNT);
    out_w(IC_FS_SPKLEN, I2C_SPKLEN);
    out_w(IC_SDA_HOLD, I2C_SDA_HOLD);
    out_w(IC_ENABLE, 1U);

    for (pin = PIN_IMU_SDA; pin <= PIN_IMU_SCL; pin++)
    {
        out_w(GPIO(pin), PAD_IE | PAD_PUE | PAD_SCHMITT);
        out_w(GPIO_CTRL(pin), FUNC_I2C);
    }
}

/* IC_TAR can only change while the block is disabled. */
static void
set_target (uint8_t addr)
{
    out_w(IC_ENABLE, 0U);
    out_w(IC_TAR, (uint32_t)addr);
    out_w(IC_ENABLE, 1U);
}

static bool
read_reg (uint8_t reg, uint8_t * p_val)
{
    return (i2c_read_regs(reg, p_val, 1U));
}

/* LSM303DLHC, LSM303AGR and LIS3DH share this register map. */
static bool
try_dlhc (uint8_t addr)
{
    uint8_t back = 0U;

    set_target(addr);
    return (i2c_write_reg(LSM_CTRL_REG1_A, LSM_REG1_100HZ_XYZ) &&
            i2c_write_reg(LSM_CTRL_REG4_A, LSM_REG4_BDU_8G_HR) &&
            read_reg(LSM_CTRL_REG1_A, &back) &&
            (LSM_REG1_100HZ_XYZ == back));
}

static bool
try_lsmd (uint8_t addr)
{
    uint8_t id = 0U;
    uint8_t back = 0U;

    set_target(addr);
    return (read_reg(LSMD_WHO_AM_I, &id) && (LSMD_ID == id) &&
            i2c_write_reg(LSM_CTRL_REG1_A, LSMD_CTRL1_100HZ_BDU) &&
            i2c_write_reg(LSMD_CTRL2, LSMD_CTRL2_8G) &&
            read_reg(LSMD_CTRL2, &back) && (LSMD_CTRL2_8G == back));
}

static bool
try_mpu (uint8_t addr)
{
    uint8_t id = 0U;
    uint8_t back = 0U;

    set_target(addr);
    return (read_reg(MPU_WHO_AM_I, &id) &&
            ((0x68U == id) || (0x70U == id) || (0x71U == id) ||
             (0x73U == id)) &&
            i2c_write_reg(MPU_PWR_MGMT_1, MPU_CLK_PLL) &&
            i2c_write_reg(MPU_CONFIG, MPU_DLPF_44HZ) &&
            i2c_write_reg(MPU_ACCEL_CONFIG, MPU_ACCEL_8G) &&
            read_reg(MPU_ACCEL_CONFIG, &back) && (MPU_ACCEL_8G == back));
}

bool
lsm303_init (void)
{
    static const uint8_t dlhc[] = { 0x19U, 0x18U };
    static const uint8_t lsmd[] = { 0x1DU, 0x1EU };
    static const uint8_t mpu[] = { 0x68U, 0x69U };
    uint32_t             idx;

    i2c_bus_init();
    g_part = PART_NONE;
    for (idx = 0U; idx < 2U; idx++)
    {
        if (try_dlhc(dlhc[idx]))
        {
            g_part = PART_DLHC;
            g_addr = dlhc[idx];
            return (true);
        }
    }
    for (idx = 0U; idx < 2U; idx++)
    {
        if (try_lsmd(lsmd[idx]))
        {
            g_part = PART_LSMD;
            g_addr = lsmd[idx];
            return (true);
        }
    }
    for (idx = 0U; idx < 2U; idx++)
    {
        if (try_mpu(mpu[idx]))
        {
            g_part = PART_MPU;
            g_addr = mpu[idx];
            return (true);
        }
    }
    return (false);
}

char const *
lsm303_part (void)
{
    return (PART_NAME[g_part]);
}

uint8_t
lsm303_addr (void)
{
    return (g_addr);
}

/* Idle bus levels: bit 0 SDA, bit 1 SCL (both should read 1). */
uint32_t
lsm303_lines (void)
{
    return (((0U != gpio_get_val(PIN_IMU_SDA)) ? 1U : 0U) |
            ((0U != gpio_get_val(PIN_IMU_SCL)) ? 2U : 0U));
}

/* Every 7-bit address that acknowledges a one-byte read. */
uint32_t
lsm303_scan (uint8_t * p_found, uint32_t max)
{
    uint32_t n = 0U;
    uint32_t addr;

    for (addr = SCAN_FIRST; addr <= SCAN_LAST; addr++)
    {
        set_target((uint8_t)addr);
        (void)in_w(IC_CLR_INTR);
        out_w(IC_DATA_CMD, IC_CMD_READ | IC_CMD_STOP);
        if (i2c_finish() && (n < max))
        {
            p_found[n] = (uint8_t)addr;
            n++;
        }
        while (0U != in_w(IC_RXFLR))
        {
            (void)in_w(IC_DATA_CMD);
        }
    }
    g_part = PART_NONE;
    return (n);
}

bool
lsm303_read_mg (int32_t * p_x, int32_t * p_y, int32_t * p_z)
{
    uint8_t  raw[2U * LSM_AXES];
    int32_t  axis[LSM_AXES];
    uint32_t idx;

    uint8_t const first = (PART_MPU == g_part) ? MPU_ACCEL_XOUT_H :
                          (LSM_OUT_X_L_A | LSM_AUTO_INC);

    if ((PART_NONE == g_part) ||
        !i2c_read_regs(first, raw, sizeof(raw)))
    {
        return (false);
    }

    for (idx = 0U; idx < LSM_AXES; idx++)
    {
        uint32_t const lo = raw[2U * idx];
        uint32_t const hi = raw[(2U * idx) + 1U];
        /* MPU registers are big-endian, the LSM303s little-endian. */
        uint32_t const word = (PART_MPU == g_part) ? ((lo << 8U) | hi) :
                                                     ((hi << 8U) | lo);
        int32_t const counts = (int32_t)(int16_t)(uint16_t)word;

        if (PART_MPU == g_part)
        {
            axis[idx] = (counts * 1000) / MPU_LSB_PER_G;
        }
        else if (PART_LSMD == g_part)
        {
            axis[idx] = (counts * LSMD_UG_PER_COUNT) / 1000;
        }
        else
        {
            axis[idx] = (counts / LSM_LSB_PER_COUNT) * LSM_MG_PER_COUNT;
        }
    }

    *p_x = axis[0];
    *p_y = axis[1];
    *p_z = axis[2];
    return (true);
}

/*** end of file ***/
