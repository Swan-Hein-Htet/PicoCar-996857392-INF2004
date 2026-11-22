/** @file drv_ultrasonic.h
 *
 * @brief HC-SR04: TRIG on GP2, ECHO on GP3 through the 10 k / 15 k
 *        divider (docs/WIRING.md section 4).  Owned by T5.
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>

#define US_NO_RETURN_MM     0xFFFFU

/* Configure the pins and register the echo handler with the dispatcher. */
ER ultrasonic_init(void);

/* One ping: trigger, sleep until ISR_ECHO wakes the task (or timeout).
   Keeps at least 70 ms between triggers.  Returns the range in mm, or
   US_NO_RETURN_MM for no return.  *p_b_edge is false when no echo pulse
   arrived at all (sensor fault, FAULT_NO_ECHO after five in a row). */
uint16_t ultrasonic_ping(bool * p_b_edge);

#endif /* DRV_ULTRASONIC_H */

/*** end of file ***/
