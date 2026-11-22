/** @file drv_lsm303.h
 *
 * @brief GY-511 / LSM303DLHC accelerometer on I2C0 (GP4 SDA, GP5 SCL,
 *        Grove 3).  Owned by T3 ImuTerrain only (TEAM.md section 3).
 *
 * The LSM303DLHC has NO gyroscope (tribunal A4): pitch comes from the
 * gravity vector and turn rate from the wheel-speed differential.
 * Mounting per docs/WIRING.md: X forward, Y left, Z up, address 0x19.
 */

#ifndef DRV_LSM303_H
#define DRV_LSM303_H

#include <stdint.h>
#include <stdbool.h>

/* Configure I2C0 at 400 kHz and find the accelerometer: LSM303DLHC/AGR
   (or LIS3DH) at 0x19/0x18, LSM303D at 0x1D/0x1E, MPU-6050/6500 at
   0x68/0x69.  Each is set to about 100 Hz and +-8 g and verified by
   reading a written register back. */
bool lsm303_init(void);

/* Burst-read X, Y, Z in milli-g.  False on a bus error or timeout. */
bool lsm303_read_mg(int32_t * p_x, int32_t * p_y, int32_t * p_z);

/* After a successful init: the part found and its address. */
char const * lsm303_part(void);
uint8_t lsm303_addr(void);

/* Diagnostics when nothing answers: idle bus levels (bit 0 SDA, bit 1 SCL,
   both should be 1) and every address acknowledging a read (up to max). */
uint32_t lsm303_lines(void);
uint32_t lsm303_scan(uint8_t * p_found, uint32_t max);

#endif /* DRV_LSM303_H */

/*** end of file ***/
