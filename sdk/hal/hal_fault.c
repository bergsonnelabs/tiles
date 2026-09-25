/**
 * hal_fault.c — Cortex-M fault handlers with register dump
 *
 * Captures the exception stack frame and:
 *   1. Calls an optional user callback
 *   2. Dumps registers over USB CDC (polled, works with interrupts disabled)
 *   3. If a DFU bootloader is configured (APP_OFFSET or ROM_DFU):
 *      blinks one SOS cycle, then reboots into DFU — prevents a hard
 *      fault from permanently bricking a board without BOOT0/SWD access.
 *   4. Otherwise: blinks SOS on the LED forever.
 *
 * The USB dump uses direct PMA writes and endpoint polling — no ISR needed.
 * If USB CDC was never initialized, the dump is silently skipped.
 */

#include "hal_fault.h"
#include "ll_common.h"
#include "ll_rcc.h"
#include "ll_gpio.h"

#if defined(STM32L422xx)
#include "ll_usb.h"
#endif

#if defined(APP_OFFSET) || defined(ROM_DFU)
#include "hal_dfu.h"
#endif

/* Reboot into DFU after a fault only where a DFU path exists (USB Cores:
 * hal_dfu.h defines DFU_MAGIC_ADDR). "bootloader": "rom" on a Core.ST.L0 /
 * Core.ST.W5 sets ROM_DFU as well, and used to fail to compile here. */
#if (defined(APP_OFFSET) || defined(ROM_DFU)) && defined(DFU_MAGIC_ADDR)
  #define HAL_FAULT_DFU_REBOOT 1
#endif

/* ---- User callback ---- */

static hal_fault_callback_t _fault_cb;

void hal_fault_set_callback(hal_fault_callback_t cb)
{
    _fault_cb = cb;
}

/* ---- Polled USB CDC TX (works with interrupts disabled) ---- */

#if defined(STM32L422xx)

#define PMA_EP1_TX  0xA8
#define EP1_MAX     64

/**
 * Send a buffer over USB CDC EP1 by polling the endpoint status.
 * Does not depend on the USB interrupt — safe to call from fault context.
 * Silently does nothing if USB was never initialized.
 */
static void fault_usb_write(const uint8_t *data, uint16_t len)
{
    /* Check if USB is powered and configured (DADDR FADDR != 0 means enumerated) */
    uint16_t daddr = *(volatile uint16_t *)(0x40006800UL + 0x4C);
    if (!(daddr & (1U << 7)))
        return;  /* USB function not enabled — skip */

    uint16_t sent = 0;
    while (sent < len) {
        /* Wait for EP1 TX to be NAK (ready for new data) */
        uint32_t timeout = 500000;
        while (timeout--) {
            uint16_t ep1r = ll_usb_ep_read(1);
            uint16_t stat = (ep1r >> 4) & 0x3;
            if (stat == 0x2)  /* NAK = ready */
                break;
        }
        if (timeout == 0)
            return;  /* Timed out — host not reading, give up */

        uint16_t chunk = len - sent;
        if (chunk > EP1_MAX) chunk = EP1_MAX;

        ll_usb_pma_write(PMA_EP1_TX, data + sent, chunk);
        ll_usb_bdt_set_tx_count(1, chunk);
        ll_usb_ep_set_stat_tx(1, USB_EP_STAT_VALID);

        sent += chunk;

        /* Wait for TX complete */
        timeout = 500000;
        while (timeout--) {
            uint16_t ep1r = ll_usb_ep_read(1);
            if (ep1r & USB_EP_CTR_TX) {
                ll_usb_ep_clr_ctr_tx(1);
                break;
            }
        }
    }
}

static void fault_usb_puts(const char *s)
{
    uint16_t len = 0;
    while (s[len]) len++;
    fault_usb_write((const uint8_t *)s, len);
}

/* Hex conversion without printf */
static const char hex[] = "0123456789ABCDEF";

static void fault_usb_hex32(uint32_t val)
{
    char buf[11];
    buf[0] = '0';
    buf[1] = 'x';
    for (int i = 7; i >= 0; i--) {
        buf[2 + (7 - i)] = hex[(val >> (i * 4)) & 0xF];
    }
    buf[10] = '\0';
    fault_usb_puts(buf);
}

static void fault_usb_reg(const char *name, uint32_t val)
{
    fault_usb_puts("  ");
    fault_usb_puts(name);
    fault_usb_puts(" = ");
    fault_usb_hex32(val);
    fault_usb_puts("\r\n");
}

#endif /* STM32L422xx */

/* ---- LED SOS (raw GPIO, no coregen dependency) ---- */

/* MCU-specific LED pin.  Must match the tile JSON for each family. */
#if defined(STM32WBA55xx) || defined(STM32H523xx)
  /* Core.ST.W5: PB12 (active-high)    Core.ST.H5: PB12 (active-high) */
  #define FAULT_LED_PORT   GPIOB
  #define FAULT_LED_PIN    12
#else
  /* Core.ST.L4 / Core.ST.L0: PA8 (active-high) */
  #define FAULT_LED_PORT   GPIOA
  #define FAULT_LED_PIN    8
#endif

static void fault_delay(volatile uint32_t count)
{
    while (count--)
        ;
}

static void fault_led_init(void)
{
    ll_rcc_gpio_clk_enable(FAULT_LED_PORT);
    /* Set LED pin as output */
    MOD_BITS(FAULT_LED_PORT->MODER,
             3UL << (FAULT_LED_PIN * 2),
             1UL << (FAULT_LED_PIN * 2));
}

static void fault_led_on(void)  { FAULT_LED_PORT->BSRR = (1UL << FAULT_LED_PIN); }
static void fault_led_off(void) { FAULT_LED_PORT->BSRR = (1UL << (FAULT_LED_PIN + 16)); }

static void fault_blink(int n, uint32_t on_ticks, uint32_t off_ticks)
{
    for (int i = 0; i < n; i++) {
        fault_led_on();
        fault_delay(on_ticks);
        fault_led_off();
        fault_delay(off_ticks);
    }
}

#if !defined(HAL_FAULT_DFU_REBOOT)
static void fault_sos(void) __attribute__((noreturn));
static void fault_sos(void)
{
    fault_led_init();
    while (1) {
        fault_blink(3, 80000, 80000);    /* S: ... */
        fault_blink(3, 300000, 300000);  /* O: --- */
        fault_blink(3, 80000, 80000);    /* S: ... */
        fault_delay(1500000);
    }
}
#endif

/* ---- Fault type names ---- */

/* Only the USB CDC dump below names the fault, and that is L422-only — on every
 * other Core this is dead code the compiler rightly complains about. */
#if defined(STM32L422xx)
static const char *fault_name(hal_fault_type_t type)
{
    switch (type) {
    case HAL_FAULT_HARD:      return "HardFault";
    case HAL_FAULT_MEMMANAGE: return "MemManage";
    case HAL_FAULT_BUS:       return "BusFault";
    case HAL_FAULT_USAGE:     return "UsageFault";
    default:                  return "Unknown";
    }
}
#endif /* STM32L422xx */

/* ---- Common fault handler ---- */

static void fault_handler(hal_fault_type_t type, uint32_t *stack)
{
    hal_fault_frame_t frame = {
        .r0  = stack[0],
        .r1  = stack[1],
        .r2  = stack[2],
        .r3  = stack[3],
        .r12 = stack[4],
        .lr  = stack[5],
        .pc  = stack[6],
        .psr = stack[7],
    };

    /* User callback (if registered) */
    if (_fault_cb)
        _fault_cb(type, &frame);

    /* Dump over USB CDC (polled) */
#if defined(STM32L422xx)
    fault_usb_puts("\r\n*** ");
    fault_usb_puts(fault_name(type));
    fault_usb_puts(" ***\r\n");
    fault_usb_reg("PC ", frame.pc);
    fault_usb_reg("LR ", frame.lr);
    fault_usb_reg("R0 ", frame.r0);
    fault_usb_reg("R1 ", frame.r1);
    fault_usb_reg("R2 ", frame.r2);
    fault_usb_reg("R3 ", frame.r3);
    fault_usb_reg("R12", frame.r12);
    fault_usb_reg("PSR", frame.psr);

    /* Dump CFSR for more detail */
    uint32_t cfsr = REG32(0xE000ED28UL);
    fault_usb_reg("CFSR", cfsr);
    if (cfsr & (1UL << 15)) {
        /* BFARVALID — Bus Fault Address Register is valid */
        fault_usb_reg("BFAR", REG32(0xE000ED38UL));
    }
    if (cfsr & (1UL << 7)) {
        /* MMARVALID — MemManage Fault Address Register is valid */
        fault_usb_reg("MMFAR", REG32(0xE000ED34UL));
    }
    fault_usb_puts("\r\nSOS...\r\n");
#endif

#if defined(HAL_FAULT_DFU_REBOOT)
    /* DFU recovery: blink SOS once so the fault is visible, then reboot
     * into the bootloader.  This prevents a hard fault from permanently
     * bricking a board that has no BOOT0 or SWD access. */
    fault_led_init();
    fault_blink(3, 80000, 80000);    /* S */
    fault_blink(3, 300000, 300000);  /* O */
    fault_blink(3, 80000, 80000);    /* S */
    fault_delay(500000);
    hal_dfu_reboot();  /* does not return */
#else
    /* No bootloader — blink SOS forever */
    fault_sos();
#endif
}

/* ---- Exception entry — extract the correct stack pointer ---- */

/* Per-type trampolines that call fault_handler with the right type.
 * These are normal (non-naked) C functions — the naked handler below
 * passes the stack pointer in r0 and branches here. */
static void fault_hard(uint32_t *stack)      { fault_handler(HAL_FAULT_HARD, stack); }
/* Cortex-M0+ has only HardFault — no MemManage/Bus/Usage exception exists to
 * trampoline into, so these three have no entry point there (see the L011 case
 * below) and the compiler flags them as dead. */
#if !defined(STM32L011xx)
static void fault_memmanage(uint32_t *stack)  { fault_handler(HAL_FAULT_MEMMANAGE, stack); }
static void fault_bus(uint32_t *stack)        { fault_handler(HAL_FAULT_BUS, stack); }
static void fault_usage(uint32_t *stack)      { fault_handler(HAL_FAULT_USAGE, stack); }
#endif

/* Naked entry points — read the correct SP and branch to the trampoline.
 * The trampoline address is loaded from a literal pool via ldr. */

#if defined(STM32L011xx)
/* Cortex-M0+: no PSP in typical usage, no Thumb-2 conditional execution.
 * Always use MSP. Only HardFault exists (no MemManage/Bus/Usage). */
void HardFault_Handler(void) __attribute__((naked));
void HardFault_Handler(void)
{
    __asm volatile (
        "mrs r0, msp      \n"
        "ldr r1, =%0      \n"
        "bx r1            \n"
        :
        : "X" (fault_hard)
    );
}

#else
/* Cortex-M3/M4/M33: check EXC_RETURN bit 2 to select MSP vs PSP. */
#define FAULT_ENTRY(name, trampoline)                                   \
    void name(void) __attribute__((naked));                             \
    void name(void)                                                     \
    {                                                                   \
        __asm volatile (                                                \
            "tst lr, #4         \n"                                    \
            "ite eq             \n"                                     \
            "mrseq r0, msp      \n"                                    \
            "mrsne r0, psp      \n"                                    \
            "ldr r1, =%0        \n"                                    \
            "bx r1              \n"                                     \
            :                                                          \
            : "X" (trampoline)                                         \
        );                                                             \
    }

FAULT_ENTRY(HardFault_Handler,   fault_hard)
FAULT_ENTRY(MemManage_Handler,   fault_memmanage)
FAULT_ENTRY(BusFault_Handler,    fault_bus)
FAULT_ENTRY(UsageFault_Handler,  fault_usage)
#endif
