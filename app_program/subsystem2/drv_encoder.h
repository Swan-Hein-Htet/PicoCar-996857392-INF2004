/** @file drv_encoder.h
 *
 * @brief Wheel encoder inputs and their ISRs (registered with the shared
 *        GPIO dispatcher).  Counters are g_enc_left / g_enc_right.
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

/* Configure the four encoder pins and register both phase-A handlers. */
ER encoder_init(void);

#endif /* DRV_ENCODER_H */

/*** end of file ***/
