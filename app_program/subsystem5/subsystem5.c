/** @file subsystem5.c
 *
 * @brief Subsystem 5: starts T5 ObstacleScan.  T5 paces itself: one ping
 *        every 70 ms, sleeping until ISR_ECHO wakes it.
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"

EXPORT ID tid_obstacle = 0;

EXPORT ER subsystem5_init (void)
{
    /* Register ISR_ECHO now: T4 enables the dispatcher in phase 2. */
    return (ultrasonic_init());
}

EXPORT ER subsystem5_start (void)
{
    tid_obstacle = app_start_task(obstacle_task, PRI_T5_OBSTACLE, APP_CORE,
                                  "T5_obstacle");
    return ((tid_obstacle < E_OK) ? tid_obstacle : E_OK);
}

/*** end of file ***/
