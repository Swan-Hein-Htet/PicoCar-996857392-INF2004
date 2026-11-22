/* Simulator stand-in for <bsp/libbsp.h> (rp2040 GPIO and PWM helpers),
   with the same register effects as lib/libbsp (sim_hw.c). */
#ifndef SIM_LIBBSP_H
#define SIM_LIBBSP_H
#include <tk/tkernel.h>
#define GPIO_MODE_OUT 1
#define GPIO_MODE_IN  0
ER   gpio_set_pin(UINT no, UINT mode);
ER   gpio_set_val(UINT no, UINT val);
UINT gpio_get_val(UINT no);
ER   pwm_set_pin(UINT no);
ER   pwm_set_wrap(UINT no, UW wrap);
ER   pwm_set_cc(UINT no, UW cc);
ER   pwm_set_enabled(UINT no, BOOL enable);
#endif
