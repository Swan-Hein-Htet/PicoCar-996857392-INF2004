/** @file drv_ir_adc.h
 *
 * @brief IR sensors (ST188 boards).  Line pair analogue (A0 output): ADC0
 *        GP26 right line (Grove 5), ADC1 GP27 left line (Grove 6).
 *        Barcode sensor DIGITAL since 2026-11-10: the board's D0
 *        comparator output on GP28 (VBAT-sense jumper open), threshold set
 *        by the board's potentiometer.  Owned by T2 LineBarcode only.
 *
 * The line sensors' cables were swapped between Grove 5 and Grove 6 on
 * 2026-11-02 (docs/WIRING.md section 2); only these two numbers change.
 */

#ifndef DRV_IR_ADC_H
#define DRV_IR_ADC_H

#include <stdint.h>
#include <stdbool.h>

#define IR_CH_LEFT      1U      /* ADC1, GP27, Grove 6 */
#define IR_CH_RIGHT     0U      /* ADC0, GP26, Grove 5 */
#define IR_CH_BARCODE   2U
#define IR_CHANNELS     3U

void ir_adc_init(void);

/* One-shot conversion, 0-4095.  False if the ADC did not finish in
   time (bounded busy-wait, ~2 us normally). */
bool ir_adc_read(uint32_t channel, uint16_t * p_value);

/* The barcode sensor's D0 level, 0 or 1 (which one is black is learnt
   by T2: the board may drive either way). */
uint32_t ir_barcode_level(void);

#endif /* DRV_IR_ADC_H */

/*** end of file ***/
