/* Simulator stand-in for <tk/tkernel.h>.  The application code is compiled
   unchanged against this header; every call lands in sim_kernel.c, a
   virtual-time, priority-preemptive model of micro T-Kernel 3.0 that
   follows the real kernel's rules for the calls the application uses
   (including the message-buffer size accounting of messagebuf.c). */
#ifndef SIM_TKERNEL_H
#define SIM_TKERNEL_H

#include <stdint.h>
#include <stddef.h>

typedef int8_t   B;   typedef uint8_t  UB;
typedef int16_t  H;   typedef uint16_t UH;
typedef int32_t  W;   typedef uint32_t UW;
typedef int32_t  INT; typedef uint32_t UINT;
typedef int32_t  ID;  typedef int32_t  ER;  typedef int32_t PRI;
typedef int32_t  TMO; typedef uint32_t RELTIM; typedef int32_t SZ;
typedef uint32_t ATR; typedef int BOOL; typedef void * VP;
typedef void (*FP)();   /* as in the real typedef.h */

#define IMPORT extern
#define EXPORT
#define LOCAL static
#define CONST const
#define TRUE  1
#define FALSE 0

#define E_OK      0
#define E_SYS     (-5)
#define E_NOSPT   (-9)
#define E_PAR     (-17)
#define E_ID      (-18)
#define E_CTX     (-25)
#define E_NOMEM   (-33)
#define E_LIMIT   (-34)
#define E_OBJ     (-41)
#define E_NOEXS   (-42)
#define E_QOVR    (-43)
#define E_RLWAI   (-49)
#define E_TMOUT   (-50)

#define TMO_POL   0
#define TMO_FEVR  (-1)
#define TSK_SELF  0

#define TA_HLNG   0x00000001U
#define TA_RNG3   0x00000300U
#define TA_ASSPRC 0x00200000U
#define TA_TFIFO  0x00000000U
#define TA_TPRI   0x00000001U
#define TA_WSGL   0x00000000U
#define TA_WMUL   0x00000008U
#define TA_INHERIT 0x00000002U
#define TA_STA    0x00000002U
#define TA_USERBUF 0x00000020U

#define TWF_ANDW  0x00U
#define TWF_ORW   0x01U
#define TWF_CLR   0x10U
#define TWF_BITCLR 0x20U

#define TK_SUPPORT_SMP 1
#define TP_PRC1   0x01U
#define TP_PRC2   0x02U

typedef struct { W hi; UW lo; } SYSTIM;

typedef struct { void *exinf; ATR tskatr; FP task; PRI itskpri; SZ stksz;
                 UINT assprc; } T_CTSK;
typedef struct { void *exinf; ATR flgatr; UINT iflgptn; } T_CFLG;
typedef struct { void *exinf; ID wtsk; UINT flgptn; } T_RFLG;
typedef struct { void *exinf; ATR mbfatr; SZ bufsz; INT maxmsz;
                 void *bufptr; } T_CMBF;
typedef struct { void *exinf; ID wtsk; ID stsk; INT msgsz; SZ frbufsz;
                 INT maxmsz; } T_RMBF;
typedef struct { void *exinf; ATR mtxatr; PRI ceilpri; } T_CMTX;
typedef struct { void *exinf; ATR cycatr; FP cychdr; RELTIM cyctim;
                 RELTIM cycphs; } T_CCYC;
typedef struct { ATR intatr; FP inthdr; } T_DINT;
typedef struct { UH maker; UH prid; UH spver; UH prver; UH prno[4]; } T_RVER;

ID  tk_cre_tsk(const T_CTSK *pk);
ER  tk_sta_tsk(ID tskid, INT stacd);
ER  tk_slp_tsk(TMO tmout);
ER  tk_wup_tsk(ID tskid);
INT tk_can_wup(ID tskid);
ER  tk_dly_tsk(RELTIM dlytim);
ID  tk_cre_flg(const T_CFLG *pk);
ER  tk_set_flg(ID flgid, UINT setptn);
ER  tk_clr_flg(ID flgid, UINT clrptn);
ER  tk_wai_flg(ID flgid, UINT waiptn, UINT wfmode, UINT *p_flgptn,
               TMO tmout);
ER  tk_ref_flg(ID flgid, T_RFLG *pk);
ID  tk_cre_mbf(const T_CMBF *pk);
ER  tk_snd_mbf(ID mbfid, const void *msg, INT msgsz, TMO tmout);
INT tk_rcv_mbf(ID mbfid, void *msg, TMO tmout);
ER  tk_ref_mbf(ID mbfid, T_RMBF *pk);
ID  tk_cre_mtx(const T_CMTX *pk);
ER  tk_loc_mtx(ID mtxid, TMO tmout);
ER  tk_unl_mtx(ID mtxid);
ID  tk_cre_cyc(const T_CCYC *pk);
ER  tk_get_tim(SYSTIM *pk);
ER  tk_def_int(UINT intno, const T_DINT *pk);
ER  tk_ref_ver(T_RVER *pk);

/* Interrupt masking and NVIC (sim_kernel.c) */
UINT sim_di(void);
void sim_ei(UINT intsts);
void EnableInt(UINT intno, INT level);
#define DI(intsts)  ((intsts) = sim_di())
#define EI(intsts)  (sim_ei(intsts))

/* Memory-mapped I/O (sim_hw.c models the RP2040 peripherals used) */
void out_w(UW addr, UW data);
UW   in_w(UW addr);
void set_w(UW addr, UW data);
void clr_w(UW addr, UW data);

/* RP2040 register map (values from include/sys/sysdepend/cpu/rp2040) */
#define RESETS_BASE         0x4000C000U
#define RESETS_RESET        (RESETS_BASE + 0x0U)
#define RESETS_RESET_DONE   (RESETS_BASE + 0x8U)
#define IO_BANK0_BASE       0x40014000U
#define GPIO_NUM            30
#define GPIO_CTRL(n)        (IO_BANK0_BASE + 0x04U + ((n) * 8U))
#define PADS_BANK0_BASE     0x4001c000U
#define GPIO(n)             (PADS_BANK0_BASE + 0x4U + ((n) * 4U))
#define SIO_BASE            0xD0000000U
#define PWM_BASE            0x40050000U
#define PWM_CHx_CSR         0x00U
#define PWM_CHx_DIV         0x04U
#define PWM_CHx_CTR         0x08U
#define PWM_CHx_CC          0x0CU
#define PWM_CHx_TOP         0x10U

#endif
