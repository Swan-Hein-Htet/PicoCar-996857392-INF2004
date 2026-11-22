/* Simulator stand-in for <tm/tmonitor.h>: console output is captured
   with its virtual time stamp by sim_kernel.c. */
#ifndef SIM_TMONITOR_H
#define SIM_TMONITOR_H
#include <tk/tkernel.h>
int tm_printf(const UB *format, ...);
#endif
