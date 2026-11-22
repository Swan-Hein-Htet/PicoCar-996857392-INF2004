/* Simulator stand-in for <tk/smp.h>: one core is modelled, so a spin lock
   is interrupt masking (zero-initialised lock = unlocked, as on target). */
#ifndef SIM_SMP_H
#define SIM_SMP_H
#include <tk/tkernel.h>
typedef volatile UW T_SPLOCK;
static inline ER ISpinLock(T_SPLOCK *lock, UINT *intsts) { (void)lock; DI(*intsts); return E_OK; }
static inline ER ISpinUnlock(T_SPLOCK *lock, UINT intsts) { (void)lock; EI(intsts); return E_OK; }
#endif
