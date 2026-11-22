/* Virtual-time model of micro T-Kernel 3.0 for the host simulator.
 *
 * One host thread per task, but only one ever runs: a baton (g_cur) is
 * passed under one mutex, so the firmware sees a single core with strict
 * priority preemption (core 1, where every application task is pinned).
 *
 * Time: g_now_us advances only (a) when every task is waiting, to the next
 * event, and (b) by a random "CPU cost" charged on every kernel call and
 * every TIMER read.  Charging can cross a tick or an event, which then
 * fires at once as an interrupt and may preempt the running task -- so
 * tasks are interrupted in the middle of their work, as on the target,
 * with the interleaving varied by the seed.  Nothing fires while DI is
 * held; it is delivered at EI.
 *
 * Kernel rules copied from kernel/tkernel: TA_WMUL FIFO flag waits,
 * message-buffer free-space accounting (HEADERSZ + ROUNDSZ per message,
 * free check HEADERSZ + msgsz), preempted tasks resume at the head of
 * their ready queue, timeouts release at the first tick >= the deadline.
 */
#include "sim.h"
#include <pthread.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#define MAX_TSK     16
#define MAX_FLG     8
#define MAX_MBF     8
#define MAX_MTX     4
#define MAX_CYC     8
#define MAX_EVT     1024        /* plaid tests sample every 20 ms */
#define MAX_TICKFN  4
#define MBF_HDR     4U
#define ROUND4(x)   (((uint32_t)(x) + 3U) & ~3U)
#define WUP_MAX     65535

uint64_t     g_now_us = 0U;
sim_kstats_t g_kstats;

typedef enum { TS_FREE = 0, TS_DORMANT, TS_READY, TS_RUN, TS_WAIT } tstate_t;
typedef enum { W_NONE = 0, W_SLP, W_DLY, W_FLG, W_RMBF, W_MTX } wkind_t;

typedef struct
{
    tstate_t  st;
    int       base_pri;
    int       pri;
    void    (*entry)(INT, void *);
    void     *exinf;
    char      name[24];
    pthread_t th;
    pthread_cond_t cv;
    int64_t   rq_seq;
    int       wupcnt;
    wkind_t   wk;
    uint64_t  tmo_at;           /* 0: no timeout */
    uint64_t  wseq;
    ER        wres;
    ID        wobj;
    UINT      waiptn;
    UINT      wfmode;
    UINT     *p_flgptn;
    void     *rbuf;
    INT       rlen;
    uint64_t  released_at;      /* for release latency */
    bool      b_cyc_release;
    uint32_t  releases;
    uint64_t  max_latency;
} tcb_t;

typedef struct { bool used; ATR atr; UINT ptn; } flg_t;
typedef struct { uint32_t size; uint8_t data[64]; } msg_t;
typedef struct
{
    bool used; INT maxmsz; int32_t bufsz; int32_t frbufsz;
    msg_t q[64]; int head; int count;
} mbf_t;
typedef struct { bool used; ID owner; uint64_t locked_at; } mtx_t;
typedef struct { bool used; FP hdr; void *exinf; uint64_t next; uint32_t period_us; } cyc_t;
typedef struct { bool used; uint64_t t; uint64_t seq; sim_event_fn fn; void *arg; } evt_t;

static tcb_t  g_tcb[MAX_TSK + 1];
static flg_t  g_flg[MAX_FLG + 1];
static mbf_t  g_mbf[MAX_MBF + 1];
static mtx_t  g_mtx[MAX_MTX + 1];
static cyc_t  g_cyc[MAX_CYC + 1];
static evt_t  g_evt[MAX_EVT];
static sim_event_fn g_tickfn[MAX_TICKFN];
static void  *g_tickarg[MAX_TICKFN];
static int    g_ntickfn;
static uint64_t g_evt_seq;

static pthread_mutex_t g_big = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_main_cv = PTHREAD_COND_INITIALIZER;
static int      g_cur = 0;          /* running task, 0 = scheduler/main */
static int      g_isr = 0;
static int      g_di = 0;
static bool     g_irq_pending[32];
static FP       g_inthdr[32];
static bool     g_int_enabled[32];
static int64_t  g_head_seq = 0;
static int64_t  g_tail_seq = 0;
static uint64_t g_wseq = 0;
static uint64_t g_next_tick = 1000U;
static uint32_t g_cost_min = 1U;
static uint32_t g_cost_max = 20U;
static uint64_t g_rng = 88172645463325252ULL;
static bool     g_in_events = false;
static sim_console_fn g_con_hook = NULL;
static FILE   *g_con_fp = NULL;
static void  (*g_initial)(void) = NULL;

/* ------------------------------------------------------------ rng */

uint32_t sim_rand(void)
{
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 7; g_rng ^= g_rng << 17;
    return (uint32_t)(g_rng >> 11);
}
double sim_randf(void) { return (double)(sim_rand() & 0xFFFFFFU) / 16777216.0; }
double sim_gauss(void)
{
    double u = sim_randf() + 1e-12, v = sim_randf();
    return sqrt(-2.0 * log(u)) * cos(6.283185307 * v);
}

static void fatal(const char *msg, int id)
{
    fprintf(stderr, "[SIM] FATAL %s (%d) at %llu us in %s\n", msg, id,
            (unsigned long long)g_now_us, sim_task_name());
    fflush(NULL);
    _exit(3);
}

const char *sim_task_name(void)
{
    if (g_isr) { return "ISR"; }
    return (g_cur > 0) ? g_tcb[g_cur].name : "core0";
}
bool sim_in_isr(void) { return g_isr > 0; }

/* ------------------------------------------------------------ scheduling */

static int pick_ready(void)
{
    int best = 0;
    for (int i = 1; i <= MAX_TSK; i++)
    {
        tcb_t *t = &g_tcb[i];
        if (TS_READY != t->st) { continue; }
        if ((0 == best) || (t->pri < g_tcb[best].pri) ||
            ((t->pri == g_tcb[best].pri) && (t->rq_seq < g_tcb[best].rq_seq)))
        {
            best = i;
        }
    }
    return best;
}

static void make_ready(int id)
{
    tcb_t *t = &g_tcb[id];
    t->st = TS_READY;
    t->rq_seq = ++g_tail_seq;
    t->released_at = g_now_us;
    t->releases++;
}

/* Hand the CPU back to the scheduler; return when chosen again. */
static void yield_to_main(void)
{
    int me = g_cur;
    g_cur = 0;
    pthread_cond_signal(&g_main_cv);
    while (g_cur != me) { pthread_cond_wait(&g_tcb[me].cv, &g_big); }
    g_tcb[me].st = TS_RUN;
}

/* Preemption point: called after anything that may ready a task. */
static void resched(void)
{
    int best;
    if ((g_cur <= 0) || g_isr || (g_di > 0)) { return; }
    best = pick_ready();
    if ((0 != best) && (g_tcb[best].pri < g_tcb[g_cur].pri))
    {
        g_kstats.preemptions++;
        g_tcb[g_cur].st = TS_READY;
        g_tcb[g_cur].rq_seq = --g_head_seq;   /* head of its queue */
        yield_to_main();
    }
}

static void release(int id, ER res)
{
    tcb_t *t = &g_tcb[id];
    t->wres = res;
    t->wk = W_NONE;
    t->tmo_at = 0U;
    make_ready(id);
}

static uint64_t tick_ceil(uint64_t t)
{
    return ((t + 999U) / 1000U) * 1000U;
}

/* Block the running task.  Returns the release code. */
static ER block(wkind_t wk, TMO tmo)
{
    tcb_t *t = &g_tcb[g_cur];
    if (g_cur <= 0) { fatal("blocking call outside a task", 0); }
    if (g_isr) { g_kstats.isr_kernel_misuse++; return E_CTX; }
    if (g_di > 0) { g_kstats.di_violations++; g_di = 0; }
    t->st = TS_WAIT;
    t->wk = wk;
    t->wseq = ++g_wseq;
    t->tmo_at = (tmo < 0) ? 0U : tick_ceil(g_now_us + (uint64_t)tmo * 1000U);
    if ((tmo >= 0) && (t->tmo_at <= g_now_us)) { t->tmo_at = g_now_us + 1U; }
    t->b_cyc_release = false;
    yield_to_main();
    return t->wres;
}

/* ------------------------------------------------------------ events */

static void run_isr(sim_event_fn fn, void *arg)
{
    g_isr++;
    fn(arg);
    g_isr--;
}

static void cyc_fire(cyc_t *c)
{
    g_isr++;
    ((void (*)(void *))c->hdr)(c->exinf);
    g_isr--;
}

static void tick(void)
{
    /* World first (sensors and edges for this millisecond), then the
       kernel's timer: timeouts, then cyclic handlers. */
    for (int i = 0; i < g_ntickfn; i++) { run_isr(g_tickfn[i], g_tickarg[i]); }

    for (int i = 1; i <= MAX_TSK; i++)
    {
        tcb_t *t = &g_tcb[i];
        if ((TS_WAIT == t->st) && (0U != t->tmo_at) && (t->tmo_at <= g_now_us))
        {
            if (W_MTX == t->wk) { /* not used with a timeout */ }
            release(i, (W_DLY == t->wk) ? E_OK : E_TMOUT);
        }
    }
    for (int i = 1; i <= MAX_CYC; i++)
    {
        cyc_t *c = &g_cyc[i];
        while (c->used && (c->next <= g_now_us))
        {
            cyc_fire(c);
            c->next += c->period_us;
        }
    }
}

/* Deliver everything due at or before g_now_us (interrupt context). */
static void process_due(void)
{
    if (g_in_events || g_isr || (g_di > 0)) { return; }
    g_in_events = true;
    for (;;)
    {
        int best = -1;
        for (int i = 0; i < MAX_EVT; i++)
        {
            if (g_evt[i].used && (g_evt[i].t <= g_now_us) &&
                ((best < 0) || (g_evt[i].t < g_evt[best].t) ||
                 ((g_evt[i].t == g_evt[best].t) && (g_evt[i].seq < g_evt[best].seq))))
            {
                best = i;
            }
        }
        if ((best >= 0) && (g_evt[best].t <= g_next_tick))
        {
            evt_t e = g_evt[best];
            uint64_t saved = g_now_us;
            g_evt[best].used = false;
            g_now_us = (e.t > g_now_us) ? e.t : g_now_us;
            run_isr(e.fn, e.arg);
            g_now_us = (saved > g_now_us) ? saved : g_now_us;
            continue;
        }
        if (g_next_tick <= g_now_us)
        {
            tick();
            g_next_tick += 1000U;
            continue;
        }
        break;
    }
    g_in_events = false;
}

void sim_at(uint64_t t_us, sim_event_fn fn, void *arg)
{
    for (int i = 0; i < MAX_EVT; i++)
    {
        if (!g_evt[i].used)
        {
            g_evt[i] = (evt_t){ true, t_us, ++g_evt_seq, fn, arg };
            return;
        }
    }
    fatal("event table full", 0);
}

void sim_every_ms(sim_event_fn fn, void *arg)
{
    if (g_ntickfn >= MAX_TICKFN) { fatal("tick table full", 0); }
    g_tickfn[g_ntickfn] = fn;
    g_tickarg[g_ntickfn] = arg;
    g_ntickfn++;
}

/* CPU cost of a kernel call: time passes, interrupts may land. */
static void charge(uint32_t us)
{
    if ((g_cur <= 0) || g_isr) { return; }
    g_now_us += us;
    if (g_di > 0) { return; }
    process_due();
    resched();
}
static void kcost(void)
{
    charge(g_cost_min + (sim_rand() % (g_cost_max - g_cost_min + 1U)));
}
void sim_charge_us(uint32_t us) { charge(us); }

/* ------------------------------------------------------------ interrupts */

UINT sim_di(void) { g_di++; return 0U; }
void sim_ei(UINT intsts)
{
    (void)intsts;
    if (g_di > 0) { g_di--; }
    if (0 == g_di)
    {
        for (UINT i = 0; i < 32U; i++)
        {
            if (g_irq_pending[i]) { g_irq_pending[i] = false; sim_raise_irq(i); }
        }
        if (!g_isr) { process_due(); resched(); }
    }
}

void EnableInt(UINT intno, INT level) { (void)level; if (intno < 32U) { g_int_enabled[intno] = true; } }

ER tk_def_int(UINT intno, const T_DINT *pk)
{
    if (intno >= 32U) { return E_PAR; }
    g_inthdr[intno] = pk->inthdr;
    return E_OK;
}

void sim_raise_irq(UINT intno)
{
    if ((intno >= 32U) || !g_int_enabled[intno] || (NULL == g_inthdr[intno])) { return; }
    if (g_di > 0) { g_irq_pending[intno] = true; return; }
    g_isr++;
    ((void (*)(UINT))g_inthdr[intno])(intno);
    g_isr--;
}

/* ------------------------------------------------------------ tasks */

static void *task_thread(void *arg)
{
    int me = (int)(intptr_t)arg;
    pthread_mutex_lock(&g_big);
    while (g_cur != me) { pthread_cond_wait(&g_tcb[me].cv, &g_big); }
    g_tcb[me].st = TS_RUN;
    g_tcb[me].entry(0, g_tcb[me].exinf);
    /* Returning from a task body = tk_ext_tsk */
    g_tcb[me].st = TS_DORMANT;
    g_cur = 0;
    pthread_cond_signal(&g_main_cv);
    pthread_mutex_unlock(&g_big);
    return NULL;
}

static ID new_task(void (*entry)(INT, void *), int pri, const char *name)
{
    for (int i = 1; i <= MAX_TSK; i++)
    {
        if (TS_FREE == g_tcb[i].st)
        {
            tcb_t *t = &g_tcb[i];
            memset(t, 0, sizeof(*t));
            t->st = TS_DORMANT;
            t->base_pri = t->pri = pri;
            t->entry = entry;
            snprintf(t->name, sizeof(t->name), "%s", name);
            pthread_cond_init(&t->cv, NULL);
            return i;
        }
    }
    return E_LIMIT;
}

extern const char *sim_name_for_entry(FP entry);
ID tk_cre_tsk(const T_CTSK *pk)
{
    kcost();
    if ((pk->itskpri < 1) || (pk->itskpri > 140)) { return E_PAR; }
    if (pk->stksz < 512) { return E_PAR; }
    if ((0U != (pk->tskatr & TA_ASSPRC)) && (TP_PRC2 != pk->assprc))
    {
        fatal("application task not pinned to core 1", (int)pk->assprc);
    }
    return new_task((void (*)(INT, void *))pk->task, pk->itskpri,
                    sim_name_for_entry(pk->task));
}

ER tk_sta_tsk(ID id, INT stacd)
{
    pthread_attr_t attr;
    (void)stacd;
    kcost();
    if ((id < 1) || (id > MAX_TSK) || (TS_DORMANT != g_tcb[id].st)) { return E_OBJ; }
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 1U << 20);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    pthread_create(&g_tcb[id].th, &attr, task_thread, (void *)(intptr_t)id);
    make_ready(id);
    resched();
    return E_OK;
}

static int self_or(ID id) { return (TSK_SELF == id) ? g_cur : id; }

ER tk_slp_tsk(TMO tmo)
{
    kcost();
    tcb_t *t = &g_tcb[g_cur];
    if (t->wupcnt > 0) { t->wupcnt--; return E_OK; }
    if (TMO_POL == tmo) { return E_TMOUT; }
    return block(W_SLP, tmo);
}

ER tk_wup_tsk(ID id)
{
    int i;
    kcost();
    i = self_or(id);
    if ((i < 1) || (i > MAX_TSK) || (TS_FREE == g_tcb[i].st)) { return E_NOEXS; }
    if ((i == g_cur) && !g_isr) { return E_OBJ; }
    if (TS_DORMANT == g_tcb[i].st) { return E_OBJ; }
    if ((TS_WAIT == g_tcb[i].st) && (W_SLP == g_tcb[i].wk))
    {
        release(i, E_OK);
        g_tcb[i].b_cyc_release = true;
        resched();
        return E_OK;
    }
    if (g_tcb[i].wupcnt >= WUP_MAX) { return E_QOVR; }
    g_tcb[i].wupcnt++;
    return E_OK;
}

INT tk_can_wup(ID id)
{
    int i = self_or(id);
    INT n;
    kcost();
    n = g_tcb[i].wupcnt;
    g_tcb[i].wupcnt = 0;
    if ((uint32_t)n > g_kstats.max_wup_backlog) { g_kstats.max_wup_backlog = (uint32_t)n; }
    return n;
}

ER tk_dly_tsk(RELTIM ms)
{
    kcost();
    if (0U == ms) { return E_OK; }
    return block(W_DLY, (TMO)ms);
}

/* ------------------------------------------------------------ flags */

ID tk_cre_flg(const T_CFLG *pk)
{
    kcost();
    for (int i = 1; i <= MAX_FLG; i++)
    {
        if (!g_flg[i].used) { g_flg[i] = (flg_t){ true, pk->flgatr, pk->iflgptn }; return i; }
    }
    return E_LIMIT;
}

static bool flg_ok(UINT ptn, UINT wai, UINT mode)
{
    return (0U != (mode & TWF_ORW)) ? (0U != (ptn & wai)) : ((ptn & wai) == wai);
}

ER tk_set_flg(ID id, UINT set)
{
    kcost();
    if ((id < 1) || (id > MAX_FLG) || !g_flg[id].used) { return E_NOEXS; }
    g_flg[id].ptn |= set;
    for (;;)
    {
        int best = 0;
        for (int i = 1; i <= MAX_TSK; i++)
        {
            tcb_t *t = &g_tcb[i];
            if ((TS_WAIT == t->st) && (W_FLG == t->wk) && (t->wobj == id) &&
                flg_ok(g_flg[id].ptn, t->waiptn, t->wfmode) &&
                ((0 == best) || (t->wseq < g_tcb[best].wseq)))
            {
                best = i;
            }
        }
        if (0 == best) { break; }
        *g_tcb[best].p_flgptn = g_flg[id].ptn;
        if (0U != (g_tcb[best].wfmode & TWF_CLR)) { g_flg[id].ptn = 0U; }
        release(best, E_OK);
    }
    resched();
    return E_OK;
}

ER tk_clr_flg(ID id, UINT clr)
{
    kcost();
    if ((id < 1) || (id > MAX_FLG) || !g_flg[id].used) { return E_NOEXS; }
    g_flg[id].ptn &= clr;
    return E_OK;
}

ER tk_wai_flg(ID id, UINT wai, UINT mode, UINT *p_ptn, TMO tmo)
{
    kcost();
    if ((id < 1) || (id > MAX_FLG) || !g_flg[id].used) { return E_NOEXS; }
    if (0U == wai) { return E_PAR; }
    if (flg_ok(g_flg[id].ptn, wai, mode))
    {
        *p_ptn = g_flg[id].ptn;
        if (0U != (mode & TWF_CLR)) { g_flg[id].ptn = 0U; }
        return E_OK;
    }
    if (TMO_POL == tmo) { return E_TMOUT; }
    tcb_t *t = &g_tcb[g_cur];
    t->wobj = id; t->waiptn = wai; t->wfmode = mode; t->p_flgptn = p_ptn;
    return block(W_FLG, tmo);
}

ER tk_ref_flg(ID id, T_RFLG *pk)
{
    kcost();
    if ((id < 1) || (id > MAX_FLG) || !g_flg[id].used) { return E_NOEXS; }
    pk->flgptn = g_flg[id].ptn;
    pk->wtsk = 0;
    return E_OK;
}

UINT sim_flg_ptn(ID id) { return g_flg[id].ptn; }

/* ------------------------------------------------------------ mbf */

ID tk_cre_mbf(const T_CMBF *pk)
{
    kcost();
    if ((pk->bufsz < 0) || (pk->maxmsz <= 0) || (pk->maxmsz > 64)) { return E_PAR; }
    for (int i = 1; i <= MAX_MBF; i++)
    {
        if (!g_mbf[i].used)
        {
            memset(&g_mbf[i], 0, sizeof(g_mbf[i]));
            g_mbf[i].used = true;
            g_mbf[i].maxmsz = pk->maxmsz;
            g_mbf[i].bufsz = (int32_t)ROUND4(pk->bufsz);
            g_mbf[i].frbufsz = g_mbf[i].bufsz;
            return i;
        }
    }
    return E_LIMIT;
}

ER tk_snd_mbf(ID id, const void *msg, INT sz, TMO tmo)
{
    mbf_t *m;
    kcost();
    if ((id < 1) || (id > MAX_MBF) || !g_mbf[id].used) { g_kstats.mbf_send_fail++; return E_NOEXS; }
    m = &g_mbf[id];
    if ((sz <= 0) || (sz > m->maxmsz)) { g_kstats.mbf_send_fail++; return E_PAR; }

    /* A receiver is waiting: hand over directly (buffer must be empty). */
    {
        int best = 0;
        for (int i = 1; i <= MAX_TSK; i++)
        {
            tcb_t *t = &g_tcb[i];
            if ((TS_WAIT == t->st) && (W_RMBF == t->wk) && (t->wobj == id) &&
                ((0 == best) || (t->wseq < g_tcb[best].wseq))) { best = i; }
        }
        if (0 != best)
        {
            memcpy(g_tcb[best].rbuf, msg, (size_t)sz);
            g_tcb[best].rlen = sz;
            release(best, E_OK);
            resched();
            return E_OK;
        }
    }
    if ((int32_t)(MBF_HDR + (uint32_t)sz) <= m->frbufsz)
    {
        msg_t *slot = &m->q[(m->head + m->count) % 64];
        slot->size = (uint32_t)sz;
        memcpy(slot->data, msg, (size_t)sz);
        m->count++;
        m->frbufsz -= (int32_t)(MBF_HDR + ROUND4(sz));
        return E_OK;
    }
    (void)tmo;      /* the application only sends with TMO_POL */
    g_kstats.mbf_send_fail++;
    return E_TMOUT;
}

INT tk_rcv_mbf(ID id, void *msg, TMO tmo)
{
    mbf_t *m;
    kcost();
    if ((id < 1) || (id > MAX_MBF) || !g_mbf[id].used) { return E_NOEXS; }
    m = &g_mbf[id];
    if (m->count > 0)
    {
        msg_t *slot = &m->q[m->head];
        INT sz = (INT)slot->size;
        memcpy(msg, slot->data, slot->size);
        m->head = (m->head + 1) % 64;
        m->count--;
        m->frbufsz += (int32_t)(MBF_HDR + ROUND4(sz));
        return sz;
    }
    if (TMO_POL == tmo) { return E_TMOUT; }
    if (g_cur <= 0) { fatal("blocking receive from core 0", id); }
    tcb_t *t = &g_tcb[g_cur];
    t->wobj = id; t->rbuf = msg; t->rlen = 0;
    {
        ER er = block(W_RMBF, tmo);
        return (E_OK == er) ? t->rlen : er;
    }
}

ER tk_ref_mbf(ID id, T_RMBF *pk)
{
    kcost();
    if ((id < 1) || (id > MAX_MBF) || !g_mbf[id].used) { return E_NOEXS; }
    memset(pk, 0, sizeof(*pk));
    pk->msgsz = (g_mbf[id].count > 0) ? (INT)g_mbf[id].q[g_mbf[id].head].size : 0;
    pk->frbufsz = g_mbf[id].frbufsz;
    pk->maxmsz = g_mbf[id].maxmsz;
    return E_OK;
}

int sim_mbf_count(ID id) { return g_mbf[id].count; }

/* ------------------------------------------------------------ mutex */

ID tk_cre_mtx(const T_CMTX *pk)
{
    (void)pk;
    kcost();
    for (int i = 1; i <= MAX_MTX; i++)
    {
        if (!g_mtx[i].used) { g_mtx[i] = (mtx_t){ true, 0, 0 }; return i; }
    }
    return E_LIMIT;
}

ER tk_loc_mtx(ID id, TMO tmo)
{
    kcost();
    if ((id < 1) || (id > MAX_MTX) || !g_mtx[id].used) { return E_NOEXS; }
    if (g_cur <= 0) { fatal("mutex from outside a task", id); }
    if (0 == g_mtx[id].owner)
    {
        g_mtx[id].owner = g_cur;
        g_mtx[id].locked_at = g_now_us;
        return E_OK;
    }
    if (g_mtx[id].owner == g_cur) { return -28; /* E_ILUSE */ }
    if (TMO_POL == tmo) { return E_TMOUT; }
    /* Priority inheritance (TA_INHERIT). */
    if (g_tcb[g_cur].pri < g_tcb[g_mtx[id].owner].pri)
    {
        g_tcb[g_mtx[id].owner].pri = g_tcb[g_cur].pri;
    }
    g_tcb[g_cur].wobj = id;
    return block(W_MTX, tmo);
}

ER tk_unl_mtx(ID id)
{
    int best = 0;
    uint64_t held;
    kcost();
    if ((id < 1) || (id > MAX_MTX) || (g_mtx[id].owner != g_cur)) { return -28; }
    held = g_now_us - g_mtx[id].locked_at;
    if (held > g_kstats.mtx_max_hold_us) { g_kstats.mtx_max_hold_us = (uint32_t)held; }
    g_tcb[g_cur].pri = g_tcb[g_cur].base_pri;
    for (int i = 1; i <= MAX_TSK; i++)
    {
        tcb_t *t = &g_tcb[i];
        if ((TS_WAIT == t->st) && (W_MTX == t->wk) && (t->wobj == id) &&
            ((0 == best) || (t->pri < g_tcb[best].pri) ||
             ((t->pri == g_tcb[best].pri) && (t->wseq < g_tcb[best].wseq)))) { best = i; }
    }
    g_mtx[id].owner = best;
    if (0 != best)
    {
        g_mtx[id].locked_at = g_now_us;
        release(best, E_OK);
    }
    resched();
    return E_OK;
}

/* ------------------------------------------------------------ time */

ID tk_cre_cyc(const T_CCYC *pk)
{
    kcost();
    if (0U == pk->cyctim) { return E_PAR; }
    for (int i = 1; i <= MAX_CYC; i++)
    {
        if (!g_cyc[i].used)
        {
            g_cyc[i] = (cyc_t){ true, pk->cychdr, pk->exinf,
                                tick_ceil(g_now_us + (uint64_t)pk->cycphs * 1000U),
                                pk->cyctim * 1000U };
            if (g_cyc[i].next <= g_now_us) { g_cyc[i].next += 1000U; }
            return i;
        }
    }
    return E_LIMIT;
}

ER tk_get_tim(SYSTIM *pk)
{
    kcost();
    pk->hi = 0;
    pk->lo = (UW)(g_now_us / 1000U);
    return E_OK;
}

ER tk_ref_ver(T_RVER *pk)
{
    memset(pk, 0, sizeof(*pk));
    pk->spver = 0x6300;
    return E_OK;
}

/* ------------------------------------------------------------ console */

void sim_console_hook(sim_console_fn fn) { g_con_hook = fn; }
void sim_console_file(FILE *fp) { g_con_fp = fp; }

static char g_line[512];
static size_t g_line_len = 0;

/* Same hook as lib/libtm/libtm_printf.c: weak default, the application
   overrides it in MQTT builds (net_bridge.c). */
__attribute__((weak)) void tm_log_tap(const UB *str, INT len) { (void)str; (void)len; }

int tm_printf(const UB *fmt, ...)
{
    char buf[512];
    va_list ap;
    int n;
    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), (const char *)fmt, ap);
    va_end(ap);

    for (char *p = buf; *p; p++)
    {
        if ('\n' == *p)
        {
            g_line[g_line_len] = '\0';
            if (0 == strncmp(g_line, "ERROR", 5)) { g_kstats.error_lines++; }
            if (NULL != g_con_fp)
            {
                fprintf(g_con_fp, "%9.3f %-13s %s\n", (double)g_now_us / 1000.0,
                        sim_task_name(), g_line);
            }
            if (NULL != g_con_hook) { g_con_hook(g_now_us, sim_task_name(), g_line); }
            g_line_len = 0;
        }
        else if (g_line_len < sizeof(g_line) - 1U)
        {
            g_line[g_line_len++] = *p;
        }
    }
    /* After the console copy: releasing the tap's lock can deliver the
       network poll, which must not see a line the console has not. */
    if (n > 0) { tm_log_tap((const UB *)buf, (n < (int)sizeof(buf)) ? n : (int)sizeof(buf) - 1); }
    return n;
}

/* ------------------------------------------------------------ boot, run */

void sim_kernel_init(uint32_t seed, uint32_t cmin, uint32_t cmax)
{
    g_rng ^= ((uint64_t)seed * 0x9E3779B97F4A7C15ULL) | 1U;
    for (int i = 0; i < 10; i++) { (void)sim_rand(); }
    g_cost_min = cmin;
    g_cost_max = (cmax < cmin) ? cmin : cmax;
}

static void initial_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;
    g_initial();
    for (;;) { (void)tk_slp_tsk(TMO_FEVR); }
}

void sim_boot(void (*initial)(void))
{
    ID id;
    pthread_mutex_lock(&g_big);
    g_initial = initial;
    id = new_task(initial_task, 1, "init");
    g_tcb[id].st = TS_DORMANT;
    {
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, 1U << 20);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        pthread_create(&g_tcb[id].th, &attr, task_thread, (void *)(intptr_t)id);
    }
    make_ready(id);
    pthread_mutex_unlock(&g_big);
}

void sim_run_until(uint64_t end)
{
    pthread_mutex_lock(&g_big);
    while (g_now_us < end)
    {
        int t = pick_ready();
        if (0 != t)
        {
            tcb_t *tc = &g_tcb[t];
            g_kstats.switches++;
            if (tc->b_cyc_release)
            {
                uint64_t lat = g_now_us - tc->released_at;
                if (lat > tc->max_latency) { tc->max_latency = lat; }
                tc->b_cyc_release = false;
            }
            g_cur = t;
            pthread_cond_signal(&tc->cv);
            while (0 != g_cur) { pthread_cond_wait(&g_main_cv, &g_big); }
            continue;
        }
        /* Idle: jump to the next event. */
        {
            uint64_t next = g_next_tick;
            for (int i = 0; i < MAX_EVT; i++)
            {
                if (g_evt[i].used && (g_evt[i].t < next)) { next = g_evt[i].t; }
            }
            if (next > g_now_us) { g_now_us = next; }
            process_due();
        }
    }
    pthread_mutex_unlock(&g_big);
}

int sim_task_count(void) { return MAX_TSK; }
void sim_task_info(int idx, sim_task_info_t *p)
{
    tcb_t *t = &g_tcb[idx + 1];
    p->name = (TS_FREE == t->st) ? NULL : t->name;
    p->pri = t->base_pri;
    p->releases = t->releases;
    p->max_latency_us = t->max_latency;
}
