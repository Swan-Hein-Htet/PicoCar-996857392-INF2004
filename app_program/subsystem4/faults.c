/** @file faults.c
 *
 * @brief The shared 16-bit fault mask and its only access helpers.
 *
 * g_faults is written by several tasks and by ISR_BTN.  The Cortex-M0+
 * has no atomic read-modify-write, so every change is made with
 * interrupts masked (packed access rule R2): a few cycles, well inside
 * NFR16.  Readers do one aligned halfword load, which is atomic.
 *
 * SMP note: DI masks the CALLING core only.  Every fault writer (all
 * application tasks and the GPIO dispatcher) runs on APP_CORE, so masking
 * that core is sufficient.  The network bridge on core 0 never writes
 * faults (it forwards commands through mbf_cmd instead).
 *
 * A critical fault forces the motors off immediately, from the detector's
 * own context, and sets flg_sys ESTOP; T1 re-asserts both every 20 ms.
 */

#include "subsystem4.h"
#include "../subsystem1/subsystem1.h"   /* flg_sys */
#include "../subsystem2/subsystem2.h"   /* motion_force_off() */

EXPORT volatile uint16_t g_faults = 0U;

_Static_assert(2U == sizeof(g_faults), "fault mask is one halfword");

EXPORT void fault_set (uint16_t mask)
{
    UINT     intsts;
    uint32_t updated;

    DI(intsts);
    updated  = (uint32_t)g_faults | (uint32_t)mask;
    g_faults = (uint16_t)updated;
    EI(intsts);

    if (0U != ((uint32_t)mask & (uint32_t)FAULT_CRITICAL_MASK))
    {
        motion_force_off();
        (void)tk_set_flg(flg_sys, FLG_ESTOP);
    }
}

EXPORT void fault_clear (uint16_t mask)
{
    UINT     intsts;
    uint32_t updated;

    DI(intsts);
    updated  = (uint32_t)g_faults & ~(uint32_t)mask;
    g_faults = (uint16_t)updated;
    EI(intsts);
}

EXPORT uint16_t fault_get (void)
{
    return (g_faults);
}

EXPORT bool fault_is_critical (void)
{
    return (0U != ((uint32_t)g_faults & (uint32_t)FAULT_CRITICAL_MASK));
}

/*** end of file ***/
