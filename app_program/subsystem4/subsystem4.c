/** @file subsystem4.c
 *
 * @brief Subsystem 4: creates T1 SafetyMonitor and T3 ImuTerrain and
 *        their drift-free 20 ms / 10 ms releases.
 */

#include "subsystem4.h"

EXPORT ID tid_safety = 0;
EXPORT ID tid_imu = 0;

EXPORT ER subsystem4_init (void)
{
    /* The fault mask is statically initialised; the status LED is set
       low before any task can drive it. */
    safety_led_init();
    return (E_OK);
}

EXPORT ER subsystem4_start (void)
{
    ID id;

    tid_safety = app_start_task(safety_task, PRI_T1_SAFETY, APP_CORE,
                                "T1_safety");
    if (tid_safety < E_OK)
    {
        return (tid_safety);
    }
    id = app_start_waker(&tid_safety, S4_T1_PERIOD_MS);
    if (id < E_OK)
    {
        return (id);
    }

    tid_imu = app_start_task(imu_task, PRI_T3_IMU, APP_CORE, "T3_imu");
    if (tid_imu < E_OK)
    {
        return (tid_imu);
    }
    id = app_start_waker(&tid_imu, S4_T3_PERIOD_MS);
    return ((id < E_OK) ? id : E_OK);
}

/*** end of file ***/
