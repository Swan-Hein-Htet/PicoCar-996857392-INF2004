/* Host-test stand-in for <tk/tkernel.h>: types and no-op kernel calls,
   enough to compile the pure modules (barcode, telemetry, app_math). */
#ifndef HOST_TKERNEL_H
#define HOST_TKERNEL_H
#include <stdint.h>
typedef int32_t  INT;  typedef uint32_t UINT; typedef int32_t ID;
typedef int32_t  ER;   typedef int32_t PRI;   typedef uint32_t UW;
typedef uint8_t  UB;   typedef int32_t TMO;   typedef int32_t SZ;
typedef void (*FP)(void);
typedef int BOOL;
#define IMPORT extern
#define EXPORT
#define LOCAL static
#define E_OK 0
#define TRUE 1
#define FALSE 0
#define TK_SUPPORT_SMP 0
#define TP_PRC1 1U
#define TP_PRC2 2U
#endif
