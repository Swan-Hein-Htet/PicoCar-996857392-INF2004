/** @file app_math.h
 *
 * @brief Small single-precision maths for the FPU-less Cortex-M0+.
 *
 * The template links no libm, and newlib's libm would also pull in errno
 * and reentrancy support.  These cover the few functions the firmware
 * needs, over the ranges it uses, at well below the sensor noise:
 *   app_sqrtf   relative error < 1e-6 (Newton, range-reduced)
 *   app_atan2f  absolute error < 2e-5 rad (minimax polynomial)
 *   app_sinf/app_cosf  absolute error < 1e-5 (odd polynomial, |x| <= pi)
 * Pure C, no globals: also used by the host tests.
 */

#ifndef APP_MATH_H
#define APP_MATH_H

#define APP_PI_F        3.14159265f
#define APP_HALF_PI_F   1.57079633f

float app_fabsf(float x);
float app_sqrtf(float x);
float app_atan2f(float y, float x);
float app_sinf(float x);
float app_cosf(float x);

#endif /* APP_MATH_H */

/*** end of file ***/
