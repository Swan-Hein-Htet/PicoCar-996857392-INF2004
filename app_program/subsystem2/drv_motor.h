/** @file drv_motor.h
 *
 * @brief Robo Pico motor driver outputs: M1 on GP8/GP9 (PWM slice 4),
 *        M2 on GP10/GP11 (PWM slice 5), 20 kHz.  Written only by T4,
 *        except motion_force_off() (T1 and the fault helpers).
 *
 * Truth table (Robo Pico datasheet table 3): A=PWM/B=0 forward,
 * A=0/B=PWM backward, both low = BRAKE, both high = coast.
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

void motor_init(void);

/* Arm or disarm.  Arming is refused while a critical fault is set. */
void motor_arm(bool b_enable);
bool motor_is_armed(void);

/* Duty in [-1, 1] per wheel, sign = direction.  The critical-fault check
   and the register write happen together with interrupts masked, so a
   force-off can never be overwritten by a duty computed before it
   (tribunal R9). */
void motor_write(float duty_left, float duty_right);

#endif /* DRV_MOTOR_H */

/*** end of file ***/
