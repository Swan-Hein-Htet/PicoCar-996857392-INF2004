/** @file net_bridge.c
 *
 * @brief The only functions the kernel's CYW43/lwIP service task calls
 *        into the application (from mqtt_utk.c, on core 0).  Each one
 *        is a non-blocking kernel-object operation (TMO_POL) or a single
 *        byte store, so the network can never stall a control task and a
 *        control task can never stall the network.
 */

#include "subsystem1.h"

EXPORT volatile uint8_t g_mqtt_connected = 0U;  /* writer: service task */

EXPORT int net_bridge_take_telemetry (unsigned char * p_buf, int size)
{
    INT len;

    if ((size < (int)TELEMETRY_BYTES) || (mbf_telem <= 0))
    {
        return (0);
    }
    len = tk_rcv_mbf(mbf_telem, p_buf, TMO_POL);
    return ((len > 0) ? (int)len : 0);
}

EXPORT void net_bridge_command (unsigned char const * p_data, int len)
{
    /* Anything that is not exactly one byte is rejected here; T7 checks
       again.  A full queue drops the command (the sender sees no ack in
       telemetry and may resend with a new sequence number). */
    if ((COMMAND_BYTES == (uint32_t)len) && (mbf_cmd > 0))
    {
        (void)tk_snd_mbf(mbf_cmd, (void *)p_data, (INT)len, TMO_POL);
    }
}

/* Manual driving (p5is20/car/drive, docs/MQTT.md): the key, then a
   sequence number T6 watches for a new press.  Single-byte stores. */
EXPORT volatile uint8_t g_drive_key = 0U;
EXPORT volatile uint8_t g_drive_seq = 0U;

/* Barcode map (p5is20/car/map): the raw bytes and a sequence number;
   T6 checks and applies them (only while parked). */
EXPORT volatile uint8_t g_map_req[BC_MAP_LETTERS];
EXPORT volatile uint8_t g_map_req_len = 0U;
EXPORT volatile uint8_t g_map_seq = 0U;

EXPORT void net_bridge_map (unsigned char const * p_data, int len)
{
    int idx;

    if ((len < 1) || (len > (int)BC_MAP_LETTERS))
    {
        len = 0;                    /* T6 reports it as invalid */
    }
    for (idx = 0; idx < len; idx++)
    {
        g_map_req[idx] = (uint8_t)p_data[idx];
    }
    g_map_req_len = (uint8_t)len;
    g_map_seq = (uint8_t)(g_map_seq + 1U);
}

EXPORT void net_bridge_drive (unsigned char key)
{
    g_drive_key = (uint8_t)key;
    g_drive_seq = (uint8_t)(g_drive_seq + 1U);
}

EXPORT void net_bridge_connected (int b_connected)
{
    g_mqtt_connected = (0 != b_connected) ? 1U : 0U;
}

/* ---------------------------------------------------------------------
 *  Console log over MQTT (p5is20/car/log, docs/MQTT.md).
 *
 *  tm_log_tap() overrides the weak hook in libtm_printf.c, so every
 *  tm_printf line from any task or ISR is also copied here, stamped with
 *  the time since boot.  The service task takes whole lines with
 *  net_bridge_take_log() and publishes them.  Writers on core 1 and the
 *  reader on core 0 share the ring under a spin lock with interrupts
 *  masked: a few microseconds, never a wait on the other side.  When the
 *  ring is full new text is dropped and counted, never the control loop.
 * --------------------------------------------------------------------- */
#if TM_MQTT

#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

#define LOG_RING_BYTES          8192U   /* ~100 lines: boot banner + slack */
#define LOG_RING_MASK           (LOG_RING_BYTES - 1U)
#define LOG_STAMP_BYTES         12U     /* "[  123.456] " */

static uint8_t           g_log[LOG_RING_BYTES];
static volatile uint32_t g_log_head = 0U;   /* next write */
static volatile uint32_t g_log_tail = 0U;   /* next read  */
static uint32_t          g_log_lost = 0U;   /* bytes dropped, ring full */
static bool              g_b_line_start = true;
#if TK_SUPPORT_SMP
static T_SPLOCK          g_log_lock;        /* zero = unlocked */
#endif

_Static_assert(0U == (LOG_RING_BYTES & LOG_RING_MASK),
               "log ring size is a power of two");

static UINT
log_lock (void)
{
    UINT intsts = 0U;

#if TK_SUPPORT_SMP
    (void)ISpinLock(&g_log_lock, &intsts);
#else
    DI(intsts);
#endif
    return (intsts);
}

static void
log_unlock (UINT intsts)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(&g_log_lock, intsts);
#else
    EI(intsts);
#endif
}

static uint32_t
log_free (void)
{
    return (LOG_RING_BYTES - (g_log_head - g_log_tail));
}

static void
log_put (uint8_t byte)
{
    g_log[g_log_head & LOG_RING_MASK] = byte;
    g_log_head++;
}

/* "[sssss.mmm] " from the free-running microsecond timer. */
static void
log_stamp (void)
{
    uint32_t const ms = app_micros() / 1000U;
    uint32_t       secs = ms / 1000U;
    uint32_t       frac = ms % 1000U;
    uint8_t        digits[LOG_STAMP_BYTES];
    uint32_t       idx = LOG_STAMP_BYTES;

    digits[--idx] = (uint8_t)' ';
    digits[--idx] = (uint8_t)']';
    digits[--idx] = (uint8_t)('0' + (frac % 10U));
    digits[--idx] = (uint8_t)('0' + ((frac / 10U) % 10U));
    digits[--idx] = (uint8_t)('0' + (frac / 100U));
    digits[--idx] = (uint8_t)'.';
    do
    {
        digits[--idx] = (uint8_t)('0' + (secs % 10U));
        secs /= 10U;
    } while ((secs > 0U) && (idx > 1U));
    while (idx > 1U)
    {
        digits[--idx] = (uint8_t)' ';
    }
    digits[0] = (uint8_t)'[';

    for (idx = 0U; idx < LOG_STAMP_BYTES; idx++)
    {
        log_put(digits[idx]);
    }
}

EXPORT void tm_log_tap (const UB * p_str, INT len)
{
    UINT intsts = log_lock();
    INT  idx;

    for (idx = 0; idx < len; idx++)
    {
        uint8_t const c = (uint8_t)p_str[idx];

        if ('\r' == c)
        {
            continue;
        }
        if (log_free() < (LOG_STAMP_BYTES + 2U))
        {
            g_log_lost++;               /* keep room for a stamp */
            continue;
        }
        if (g_b_line_start)
        {
            log_stamp();
            g_b_line_start = false;
        }
        log_put(c);
        if ('\n' == c)
        {
            g_b_line_start = true;
        }
    }
    log_unlock(intsts);
}

/* Copy the next complete line (without its '\n') into p_buf.  A line
   longer than the buffer comes out in pieces.  Returns its length, 0 if
   no complete line is waiting.  A loss notice is reported first. */
EXPORT int net_bridge_take_log (unsigned char * p_buf, int size)
{
    UINT     intsts;
    uint32_t avail;
    uint32_t idx;
    uint32_t len = 0U;
    uint32_t lost;
    bool     b_found = false;

    if (size < 32)
    {
        return (0);
    }

    intsts = log_lock();
    lost = g_log_lost;
    g_log_lost = 0U;
    avail = g_log_head - g_log_tail;
    for (idx = 0U; idx < avail; idx++)
    {
        if ('\n' == g_log[(g_log_tail + idx) & LOG_RING_MASK])
        {
            b_found = true;
            break;
        }
        if (idx >= ((uint32_t)size - 1U))
        {
            b_found = true;             /* over-long line: send a piece */
            break;
        }
    }
    if (b_found && (0U == lost))
    {
        for (len = 0U; len < idx; len++)
        {
            p_buf[len] = g_log[(g_log_tail + len) & LOG_RING_MASK];
        }
        g_log_tail += idx;
        if ((g_log_tail != g_log_head) &&
            ('\n' == g_log[g_log_tail & LOG_RING_MASK]))
        {
            g_log_tail++;
        }
    }
    log_unlock(intsts);

    if (0U != lost)
    {
        /* Report the gap where it happened, before the next line. */
        static char const k_msg[] = "[log] lost bytes: ";
        uint8_t digits[10];
        uint32_t n = 0U;

        for (len = 0U; '\0' != k_msg[len]; len++)
        {
            p_buf[len] = (unsigned char)k_msg[len];
        }
        do
        {
            digits[n++] = (uint8_t)('0' + (lost % 10U));
            lost /= 10U;
        } while ((lost > 0U) && (n < sizeof(digits)));
        while (n > 0U)
        {
            p_buf[len++] = digits[--n];
        }
    }
    return ((int)len);
}

#endif /* TM_MQTT */

/*** end of file ***/
