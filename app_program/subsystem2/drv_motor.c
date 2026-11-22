/** @file drv_motor.c
 *
 * @brief Sign-magnitude PWM on the Robo Pico H-bridge, plus the one
 *        documented exception to single ownership: motion_force_off().
 */

#include "subsystem2.h"
#include "drv_motor.h"
#include "../subsystem4/subsystem4.h"   /* fault_is_critical() */

#define PWM_SLICE_LEFT          4U
#define PWM_SLICE_RIGHT         5U
#define PWM_CC(slice)           (PWM_BASE + PWM_CHx_CC + ((slice) * 0x14U))
#define PWM_TOP_20KHZ           6249U   /* 125 MHz / 6250 = 20 kHz */
#define PWM_CC_B_SHIFT          16U
#define RESET_PWM               0x00004000U
#define RESET_TIMEOUT_US        10000U

static volatile bool g_b_armed = false;

/* Compare-register value for one wheel: channel A forward, B backward. */
static uint32_t
duty_to_cc (float duty, int32_t sign)
{
    float    signed_duty = duty * (float)sign;
    float    magnitude = (signed_duty < 0.0f) ? -signed_duty : signed_duty;
    uint32_t level;

    if (magnitude > 1.0f)
    {
        magnitude = 1.0f;
    }
    level = (uint32_t)(magnitude * (float)PWM_TOP_20KHZ);

    return ((signed_duty >= 0.0f) ? level : (level << PWM_CC_B_SHIFT));
}

void
motor_init (void)
{
    uint32_t const start = app_micros();
    uint32_t       pin;

    clr_w(RESETS_RESET, RESET_PWM);
    while (0U == (in_w(RESETS_RESET_DONE) & RESET_PWM))
    {
        if ((app_micros() - start) > RESET_TIMEOUT_US)
        {
            break;
        }
    }

    for (pin = PIN_M1A; pin <= PIN_M2B; pin++)
    {
        (void)pwm_set_wrap(pin, PWM_TOP_20KHZ);
        (void)pwm_set_cc(pin, 0U);
        (void)pwm_set_pin(pin);
        (void)pwm_set_enabled(pin, TRUE);
    }
    g_b_armed = false;
}

void
motor_arm (bool b_enable)
{
    UINT intsts;

    DI(intsts);
    g_b_armed = b_enable && !fault_is_critical();
    EI(intsts);
}

bool
motor_is_armed (void)
{
    return (g_b_armed);
}

void
motor_write (float duty_left, float duty_right)
{
    uint32_t cc_left = duty_to_cc(duty_left, MOTION_MOTOR_SIGN_L);
    uint32_t cc_right = duty_to_cc(duty_right, MOTION_MOTOR_SIGN_R);
    UINT     intsts;

    DI(intsts);
    if ((!g_b_armed) || fault_is_critical())
    {
        cc_left = 0U;
        cc_right = 0U;
    }
    out_w(PWM_CC(PWM_SLICE_LEFT), cc_left);
    out_w(PWM_CC(PWM_SLICE_RIGHT), cc_right);
    EI(intsts);
}

EXPORT void motion_force_off (void)
{
    UINT intsts;

    DI(intsts);
    g_b_armed = false;
    out_w(PWM_CC(PWM_SLICE_LEFT), 0U);     /* both low = brake */
    out_w(PWM_CC(PWM_SLICE_RIGHT), 0U);
    EI(intsts);
}

/*** end of file ***/
