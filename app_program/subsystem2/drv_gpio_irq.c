/** @file drv_gpio_irq.c
 *
 * @brief The one GPIO interrupt dispatcher (IO_IRQ_BANK0).
 *
 * The RP2040 has a single bank-0 GPIO interrupt per core, so registering
 * a second handler would silently replace the first (tutor review,
 * bullet 3).  Every source -- both encoders, the echo pin and both
 * buttons -- registers here, and the handler calls each in turn.  Each
 * handler is a few microseconds; the whole dispatch stays inside the
 * 20 us of NFR16.
 *
 * gpio_irq_start() must run on APP_CORE: it enables the per-core GPIO
 * interrupt for the calling core and that core's NVIC, so the encoder,
 * echo and button handlers run on the same core as every fault writer.
 */

#include "subsystem2.h"

#define IRQ_IO_BANK0            13U
#define IRQ_LEVEL               2
#define IO_INTR0                (IO_BANK0_BASE + 0x0F0U)
#define IO_PROC0_INTE0          (IO_BANK0_BASE + 0x100U)
#define IO_PROC0_INTS0          (IO_BANK0_BASE + 0x120U)
#define IO_PROC1_INTE0          (IO_BANK0_BASE + 0x130U)
#define IO_PROC1_INTS0          (IO_BANK0_BASE + 0x150U)
#define SIO_CPUID               (SIO_BASE + 0x000U)
#define PINS_PER_REG            8U
#define BITS_PER_PIN            4U
#define GPIO_REG_COUNT          4U
#define EVENT_MASK              0xFU
#define MAX_HANDLERS            8U

typedef struct
{
    gpio_handler_t handler;
    uint8_t        pin;
    uint8_t        events;
} gpio_slot_t;

static gpio_slot_t g_slots[MAX_HANDLERS];
static uint32_t    g_slot_count = 0U;
static uint32_t    g_ints_base = IO_PROC0_INTS0;
EXPORT UW          g_gpio_irq_count = 0U;

static void
gpio_dispatch (UINT intno)
{
    uint32_t const now = app_micros();
    uint32_t       idx;

    (void)intno;
    g_gpio_irq_count++;

    for (idx = 0U; idx < g_slot_count; idx++)
    {
        uint32_t const pin = g_slots[idx].pin;
        uint32_t const reg = (pin / PINS_PER_REG) * 4U;
        uint32_t const shift = (pin % PINS_PER_REG) * BITS_PER_PIN;
        uint32_t const events =
            (in_w(g_ints_base + reg) >> shift) & (uint32_t)g_slots[idx].events;

        if (0U != events)
        {
            /* Edge events are write-1-to-clear in INTR. */
            out_w(IO_INTR0 + reg, events << shift);
            g_slots[idx].handler(pin, events, now);
        }
    }
}

EXPORT ER gpio_irq_register (uint32_t pin, uint32_t events,
                             gpio_handler_t handler)
{
    if ((g_slot_count >= MAX_HANDLERS) || (pin >= GPIO_NUM) ||
        (NULL == handler))
    {
        return (E_LIMIT);
    }

    g_slots[g_slot_count].handler = handler;
    g_slots[g_slot_count].pin = (uint8_t)pin;
    g_slots[g_slot_count].events = (uint8_t)(events & EVENT_MASK);
    g_slot_count++;
    return (E_OK);
}

EXPORT ER gpio_irq_start (void)
{
    T_DINT   dint = {0};
    uint32_t inte_base = IO_PROC0_INTE0;
    uint32_t idx;
    ER       er;

    if (0U != in_w(SIO_CPUID))
    {
        inte_base = IO_PROC1_INTE0;
        g_ints_base = IO_PROC1_INTS0;
    }

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)gpio_dispatch;
    er = tk_def_int(IRQ_IO_BANK0, &dint);
    if (E_OK != er)
    {
        return (er);
    }

    for (idx = 0U; idx < g_slot_count; idx++)
    {
        uint32_t const pin = g_slots[idx].pin;
        uint32_t const reg = (pin / PINS_PER_REG) * 4U;
        uint32_t const shift = (pin % PINS_PER_REG) * BITS_PER_PIN;

        out_w(IO_INTR0 + reg, EVENT_MASK << shift);    /* stale edges */
        set_w(inte_base + reg, (uint32_t)g_slots[idx].events << shift);
    }

    EnableInt(IRQ_IO_BANK0, IRQ_LEVEL);
    return (E_OK);
}

/*** end of file ***/
