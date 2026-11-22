/** @file app_math.c
 *
 * @brief See app_math.h.
 */

#include <stdint.h>
#include "app_math.h"

#define SQRT_ITERATIONS     6U
#define TWO_PI_F            6.28318531f

float
app_fabsf (float x)
{
    return ((x < 0.0f) ? -x : x);
}

float
app_sqrtf (float x)
{
    float    scale = 1.0f;
    float    value = x;
    float    guess;
    uint32_t iter;

    if (value <= 0.0f)
    {
        return (0.0f);
    }

    /* Bring the argument into [0.25, 4): sqrt(4^k v) = 2^k sqrt(v). */
    while (value >= 4.0f)
    {
        value *= 0.25f;
        scale *= 2.0f;
    }
    while (value < 0.25f)
    {
        value *= 4.0f;
        scale *= 0.5f;
    }

    guess = 0.5f * (value + 1.0f);
    for (iter = 0U; iter < SQRT_ITERATIONS; iter++)
    {
        guess = 0.5f * (guess + (value / guess));
    }
    return (guess * scale);
}

/* atan(z) for |z| <= 1. */
static float
atan_unit (float z)
{
    float const z2 = z * z;

    return (z * (0.9998660f +
                 (z2 * (-0.3302995f +
                        (z2 * (0.1801410f +
                               (z2 * (-0.0851330f + (z2 * 0.0208351f)))))))));
}

float
app_atan2f (float y, float x)
{
    float const ax = app_fabsf(x);
    float const ay = app_fabsf(y);
    float       angle;

    if ((0.0f == ax) && (0.0f == ay))
    {
        return (0.0f);
    }

    if (ax >= ay)
    {
        angle = atan_unit(ay / ax);
    }
    else
    {
        angle = APP_HALF_PI_F - atan_unit(ax / ay);
    }

    if (x < 0.0f)
    {
        angle = APP_PI_F - angle;
    }
    return ((y < 0.0f) ? -angle : angle);
}

float
app_sinf (float x)
{
    float value = x;
    float x2;

    /* Reduce to [-pi, pi], then to [-pi/2, pi/2] by symmetry. */
    while (value > APP_PI_F)
    {
        value -= TWO_PI_F;
    }
    while (value < -APP_PI_F)
    {
        value += TWO_PI_F;
    }
    if (value > APP_HALF_PI_F)
    {
        value = APP_PI_F - value;
    }
    else if (value < -APP_HALF_PI_F)
    {
        value = -APP_PI_F - value;
    }
    else
    {
        /* Already in range. */
    }

    x2 = value * value;
    return (value * (1.0f +
                     (x2 * (-1.6666667e-1f +
                            (x2 * (8.3333333e-3f +
                                   (x2 * (-1.9841270e-4f +
                                          (x2 * 2.7557319e-6f)))))))));
}

float
app_cosf (float x)
{
    return (app_sinf(x + APP_HALF_PI_F));
}

/*** end of file ***/
