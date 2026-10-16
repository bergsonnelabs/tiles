/**
 * core_reset.c — core_reset() and the boot-time reset reason (core_reset.h)
 *
 * Compiled into every build, like core_watchdog.c. The BLE link drop lives in
 * core_ble.c (BLE builds only) behind a weak hook; the USB flush in
 * hal_usb_cdc.c (Core.ST.L4 / H5).
 */

#include "core_reset.h"
#include "ll_common.h"
#include "ll_rcc.h"
#include "ll_iwdg.h"

#if defined(STM32L422xx) || defined(STM32H523xx)
#include "hal_usb_cdc.h"
#endif
#if defined(STM32L422xx)
#include "ll_flash.h"
#endif

/* ============================================================
 * Reset reason
 * ============================================================ */

static uint32_t            s_flags;
static core_reset_reason_t s_reason = CORE_RESET_OTHER;
static uint8_t             s_captured;

static core_reset_reason_t classify(uint32_t f)
{
    if (f & (CORE_RESET_FLAG_IWDG | CORE_RESET_FLAG_WWDG)) return CORE_RESET_WATCHDOG;
    if (f & (CORE_RESET_FLAG_SFT | CORE_RESET_FLAG_OBL))   return CORE_RESET_SOFTWARE;
    if (f & (CORE_RESET_FLAG_LPWR | CORE_RESET_FLAG_FW))   return CORE_RESET_OTHER;
    if (f & CORE_RESET_FLAG_POR)                           return CORE_RESET_POWER_ON;
    if (f & CORE_RESET_FLAG_PIN)                           return CORE_RESET_PIN;
    return CORE_RESET_OTHER;
}

void core_reset_capture(void)
{
    if (s_captured) return;
    s_flags    = ll_rcc_reset_flags();
    s_reason   = classify(s_flags);
    s_captured = 1;
    ll_rcc_clear_reset_flags();
}

core_reset_reason_t core_reset_reason(void)
{
    core_reset_capture();
    return s_reason;
}

uint32_t core_reset_flags(void)
{
    core_reset_capture();
    return s_flags;
}

/* ============================================================
 * core_reset()
 * ============================================================ */

/* Drops a connected BLE link and waits for it to go (core_ble.c). Absent
 * from builds without BLE. */
extern void _core_reset_ble(void) __attribute__((weak));

#if defined(STM32L422xx)
/* RM0394 §2.6 and §3.7.5. The L4's empty-flash check (boot the ROM bootloader
 * when 0x08000000 reads erased) is decided only when the option bytes load:
 * at power-on or OBL_LAUNCH. A plain reset keeps the old verdict. So a Core
 * flashed blank by ROM DFU (whose ":leave" jumps to the image) or by SWD, and
 * not power-cycled since, still has FLASH_SR.PEMPTY = 1, and a plain reset
 * would land in the ROM bootloader.
 *
 * This program runs from flash, so page 0 is programmed and PEMPTY = 1 is
 * stale. RM0394: PEMPTY "can also be set and cleared by software" (writing 1
 * toggles it). Clear it, then the plain reset starts this program. If it
 * didn't take, fall back to OBL_LAUNCH, which the serial-update flasher uses
 * for every reset (docs/serial-update-protocol.md §5).
 *
 * Why not always OBL_LAUNCH? It reloads the option bytes (a slower reset that
 * sets OBLRSTF, not SFTRSTF) and needs the flash and option-byte unlock
 * sequences. With PEMPTY right, a plain SYSRESETREQ does exactly what's
 * wanted. The flasher needs OBL_LAUNCH because it changes page 0 itself; an
 * app reset doesn't. */
#define L4_FLASH_SR_PEMPTY   (1UL << 17)
#define L4_FLASH_OPTKEYR     REG32(FLASH_BASE + 0x0CUL)
#define L4_FLASH_CR_OBL      (1UL << 27)
#define L4_FLASH_CR_OPTLOCK  (1UL << 30)

static void l4_fix_empty_check(void)
{
    if (!(FLASH_SR & L4_FLASH_SR_PEMPTY)) return;
    if (REG32(0x08000000UL) == 0xFFFFFFFFUL) return;   /* really empty: leave it */
    FLASH_SR = L4_FLASH_SR_PEMPTY;                     /* toggle to 0 (other bits: write 0 = no effect) */
    if (!(FLASH_SR & L4_FLASH_SR_PEMPTY)) return;

    ll_flash_wait_bsy();
    ll_flash_unlock();
    if (FLASH_CR & L4_FLASH_CR_OPTLOCK) {
        L4_FLASH_OPTKEYR = 0x08192A3BUL;               /* RM0394 §3.7.4 */
        L4_FLASH_OPTKEYR = 0x4C5D6E7FUL;
    }
    if (!(FLASH_CR & L4_FLASH_CR_OPTLOCK)) FLASH_CR |= L4_FLASH_CR_OBL;   /* resets here */
}
#endif

void core_reset(void)
{
    ll_iwdg_refresh();    /* harmless if the watchdog isn't running */

    /* 1. BLE: a clean drop, so the central sees it now rather than at its
     *    supervision timeout. Bounded; skipped from interrupt context. */
    if (_core_reset_ble) _core_reset_ble();

#if defined(STM32L422xx) || defined(STM32H523xx)
    /* 2. USB CDC: let queued output reach the terminal. Bounded. */
    hal_usb_cdc_flush_for_reset();
#endif

    __asm volatile ("cpsid i" ::: "memory");

#if defined(STM32L422xx)
    l4_fix_empty_check();
#endif

    /* 3. System reset (ARMv7-M / ARMv8-M AIRCR; PRIGROUP kept, as CMSIS
     *    NVIC_SystemReset does). Sets SFTRSTF. */
    __asm volatile ("dsb" ::: "memory");
    REG32(0xE000ED0CUL) = (0x05FAUL << 16) | (REG32(0xE000ED0CUL) & (7UL << 8)) | (1UL << 2);
    __asm volatile ("dsb" ::: "memory");
    for (;;)
        ;
}
