/** @file drv_encoder.c
 *
 * @brief Quadrature hall encoders.  Phase A (GP16 left, GP17 right, both
 *        Grove 4) interrupts on both edges; phase B (GP13 left, GP19
 *        right) gives the direction.
 *
 * Each ISR only increments or decrements its own free-running uint16_t
 * (rules R1 and R3).  T4 subtracts the previous value and never clears,
 * so there is no clear race.  Width rule: at 350 mm/s and ~0.19 mm/edge
 * the counter moves ~37 edges per 20 ms period, far from a 16-bit wrap.
 */

#include "subsystem2.h"
#include "drv_encoder.h"

#define PAD_ENC_INPUT           0xCAU

EXPORT volatile uint16_t g_enc_left = 0U;
EXPORT volatile uint16_t g_enc_right = 0U;

_Static_assert(2U == sizeof(g_enc_left), "encoder counter is a halfword");

/* Step +1 when A and B agree after an A edge, else -1, then the wiring
   sign.  Returns the new counter value (wrapping). */
static uint16_t
enc_step (uint16_t count, uint32_t pin_a, uint32_t pin_b, int32_t sign)
{
    bool const    b_same = (gpio_get_val(pin_a) == gpio_get_val(pin_b));
    int32_t const step = (b_same ? 1 : -1) * sign;

    return ((uint16_t)((int32_t)count + step));
}

static void
enc_left_isr (uint32_t pin, uint32_t events, uint32_t now_us)
{
    (void)pin;
    (void)events;
    (void)now_us;
    g_enc_left = enc_step(g_enc_left, PIN_ENC_L_A, PIN_ENC_L_B,
                          MOTION_ENC_SIGN_L);
}

static void
enc_right_isr (uint32_t pin, uint32_t events, uint32_t now_us)
{
    (void)pin;
    (void)events;
    (void)now_us;
    g_enc_right = enc_step(g_enc_right, PIN_ENC_R_A, PIN_ENC_R_B,
                           MOTION_ENC_SIGN_R);
}

ER
encoder_init (void)
{
    ER er;

    (void)gpio_set_pin(PIN_ENC_L_A, GPIO_MODE_IN);
    (void)gpio_set_pin(PIN_ENC_L_B, GPIO_MODE_IN);
    (void)gpio_set_pin(PIN_ENC_R_A, GPIO_MODE_IN);
    (void)gpio_set_pin(PIN_ENC_R_B, GPIO_MODE_IN);

    /* Pull-ups instead of the RP2040 reset default (pull-down): an open-
       collector hall output can only pull low, so with a pull-down it
       never reads high and the counter freezes.  Harmless for push-pull
       outputs.  OD | IE | PUE | SCHMITT. */
    out_w(GPIO(PIN_ENC_L_A), PAD_ENC_INPUT);
    out_w(GPIO(PIN_ENC_L_B), PAD_ENC_INPUT);
    out_w(GPIO(PIN_ENC_R_A), PAD_ENC_INPUT);
    out_w(GPIO(PIN_ENC_R_B), PAD_ENC_INPUT);

    er = gpio_irq_register(PIN_ENC_L_A, GPIO_EDGE_RISE | GPIO_EDGE_FALL,
                           enc_left_isr);
    if (E_OK == er)
    {
        er = gpio_irq_register(PIN_ENC_R_A, GPIO_EDGE_RISE | GPIO_EDGE_FALL,
                               enc_right_isr);
    }
    return (er);
}

/*** end of file ***/
