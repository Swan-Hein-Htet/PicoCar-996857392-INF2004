/** @file drv_ultrasonic.c
 *
 * @brief HC-SR04 trigger and ISR_ECHO.
 *
 * ISR_ECHO (called by the shared GPIO dispatcher) timestamps the rising
 * edge in an ISR-private variable and, on the falling edge, writes the
 * whole g_echo_word with ONE halfword store before waking T5 (rules R1,
 * R3).  A pulse at or beyond the ranging timeout is stored with the valid
 * bit clear, so a no-return can never be read as a short range
 * (tribunal Version C).
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "../subsystem2/subsystem2.h"

#define TRIG_PULSE_US           12U     /* datasheet: >= 10 us           */
#define ECHO_TIMEOUT_US         23500U  /* ~4 m: beyond this, no return  */
#define ECHO_MIN_US             116U    /* 2 cm: closer is not reliable  */
#define ECHO_WAIT_MS            50      /* sleep for the falling edge    */
#define MM_PER_US_X2000         343U    /* 343 m/s, out and back: /2000  */

EXPORT volatile uint16_t g_echo_word = 0U;
EXPORT UW                g_echo_count = 0U;

static uint32_t g_rise_us = 0U;         /* ISR-private */
static volatile uint32_t g_rise_count = 0U; /* ISR-private writer */
static uint32_t g_last_trigger_ms = 0U;

_Static_assert(2U == sizeof(g_echo_word), "echo word is a halfword");

static void
echo_isr (uint32_t pin, uint32_t events, uint32_t now_us)
{
    uint32_t width;
    uint32_t word;

    (void)events;

    if (0U != gpio_get_val(pin))
    {
        g_rise_us = now_us;
        g_rise_count++;
        return;
    }

    width = now_us - g_rise_us;
    word = bits_put(0U, 0U, ECHO_WIDTH_BITS,
                    bits_sat_u(width, ECHO_WIDTH_BITS));
    word = bits_put(word, ECHO_VALID_SHIFT, 1U,
                    ((width >= ECHO_MIN_US) && (width < ECHO_TIMEOUT_US)) ?
                    1U : 0U);
    g_echo_word = (uint16_t)word;
    g_echo_count++;
    (void)tk_wup_tsk(tid_obstacle);
}

ER
ultrasonic_init (void)
{
    (void)gpio_set_val(PIN_US_TRIG, 0U);
    (void)gpio_set_pin(PIN_US_TRIG, GPIO_MODE_OUT);
    (void)gpio_set_pin(PIN_US_ECHO, GPIO_MODE_IN);

    return (gpio_irq_register(PIN_US_ECHO, GPIO_EDGE_RISE | GPIO_EDGE_FALL,
                              echo_isr));
}

static uint32_t
now_ms (void)
{
    SYSTIM tim;

    (void)tk_get_tim(&tim);
    return ((uint32_t)tim.lo);
}

uint16_t
ultrasonic_ping (bool * p_b_edge)
{
    uint32_t const since = now_ms() - g_last_trigger_ms;
    uint32_t       start;
    uint32_t       word;
    UW const       count_before = g_echo_count;
    uint32_t const rise_before = g_rise_count;

    if (since < S5_PING_PERIOD_MS)
    {
        (void)tk_dly_tsk((RELTIM)(S5_PING_PERIOD_MS - since));
    }

    (void)tk_can_wup(TSK_SELF);         /* drop stale wake-ups */
    g_last_trigger_ms = now_ms();

    start = app_micros();
    (void)gpio_set_val(PIN_US_TRIG, 1U);
    while ((app_micros() - start) < TRIG_PULSE_US)
    {
        /* 12 us trigger pulse. */
    }
    (void)gpio_set_val(PIN_US_TRIG, 0U);

    (void)tk_slp_tsk(ECHO_WAIT_MS);
    /* Any edge proves the sensor is alive, even if a clone holds ECHO
       high for longer than the wait when nothing is in range. */
    *p_b_edge = (count_before != g_echo_count) ||
                (rise_before != g_rise_count);
    if (count_before == g_echo_count)
    {
        return (US_NO_RETURN_MM);
    }

    word = g_echo_word;                  /* one halfword load */
    if (0U == bits_get(word, ECHO_VALID_SHIFT, 1U))
    {
        return (US_NO_RETURN_MM);
    }
    return ((uint16_t)((bits_get(word, 0U, ECHO_WIDTH_BITS) *
                        MM_PER_US_X2000) / 2000U));
}

/*** end of file ***/
