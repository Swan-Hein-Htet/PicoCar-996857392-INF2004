/** @file drv_ir_adc.c
 *
 * @brief Register-level RP2040 ADC access for the 1 ms loop (the kernel
 *        ADC device driver's open/read path is too heavy at 1 kHz).
 */

#include "../app.h"
#include "drv_ir_adc.h"

#define ADC_BASE                0x4004C000U
#define ADC_CS                  (ADC_BASE + 0x00U)
#define ADC_RESULT              (ADC_BASE + 0x04U)
#define ADC_CS_EN               0x001U
#define ADC_CS_START_ONCE       0x004U
#define ADC_CS_READY            0x100U
#define ADC_CS_AINSEL_SHIFT     12U
#define ADC_RESULT_MASK         0x0FFFU
#define ADC_TIMEOUT_US          20U
#define RESET_ADC               0x00000001U
#define RESET_TIMEOUT_US        1000U
#define FUNC_NULL               31U
#define PAD_OUTPUT_DISABLE      0x80U
#define PAD_IE_PUE_SCHMITT      0x4AU   /* digital input, pull-up      */

void
ir_adc_init (void)
{
    uint32_t const start = app_micros();
    uint32_t       pin;

    clr_w(RESETS_RESET, RESET_ADC);
    while (0U == (in_w(RESETS_RESET_DONE) & RESET_ADC))
    {
        if ((app_micros() - start) > RESET_TIMEOUT_US)
        {
            break;
        }
    }

    /* Analogue pins GP26-GP27: no digital function, no pulls, input
       buffer off.  (Not "from PIN_IR_LEFT": left is GP27 since the
       Grove 5/6 swap, and GP26 would be skipped.) */
    for (pin = PIN_IR_RIGHT; pin <= PIN_IR_LEFT; pin++)
    {
        out_w(GPIO_CTRL(pin), FUNC_NULL);
        out_w(GPIO(pin), PAD_OUTPUT_DISABLE);
    }

    /* Barcode sensor D0 on GP28: a digital input (the board's comparator
       output is open-collector on many boards: pull-up on). */
    (void)gpio_set_pin(PIN_IR_BARCODE, GPIO_MODE_IN);
    out_w(GPIO(PIN_IR_BARCODE), PAD_IE_PUE_SCHMITT);

    out_w(ADC_CS, ADC_CS_EN);
}

bool
ir_adc_read (uint32_t channel, uint16_t * p_value)
{
    uint32_t const start = app_micros();

    out_w(ADC_CS, ADC_CS_EN | ADC_CS_START_ONCE |
                  (channel << ADC_CS_AINSEL_SHIFT));

    while (0U == (in_w(ADC_CS) & ADC_CS_READY))
    {
        /* Re-read READY once the time is up: an interrupt between the
           read above and this check can outlast the timeout while the
           2 us conversion finished long ago (bench 2026-11-02: false
           SELF_TEST faults with the car standing still). */
        if (((app_micros() - start) > ADC_TIMEOUT_US) &&
            (0U == (in_w(ADC_CS) & ADC_CS_READY)))
        {
            return (false);
        }
    }

    *p_value = (uint16_t)(in_w(ADC_RESULT) & ADC_RESULT_MASK);
    return (true);
}

uint32_t
ir_barcode_level (void)
{
    return ((0U != gpio_get_val(PIN_IR_BARCODE)) ? 1U : 0U);
}

/*** end of file ***/
