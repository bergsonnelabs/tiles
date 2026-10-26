/**
 * port_core_reset_hw.h — STM32 port: the hardware behind core_reset.c
 *
 * Included by core_reset.c only; not for direct use. The RCC reset flags
 * (read, clear, and their mapping onto core_reset_reason_t), and the steps of
 * core_reset() that touch the chip: the IWDG refresh, the USB CDC flush
 * (Core.ST.L4 / H5) and, on the L4, the empty-flash check corrected just
 * before the reset. The reset itself (AIRCR) is the Cortex-M architecture's
 * and stays in core_reset.c.
 */

#ifndef PORT_CORE_RESET_HW_H
#define PORT_CORE_RESET_HW_H

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

/* The flags read at boot, as core_reset_reason_t: the priority core_reset.h
 * lists, first match wins. */
static core_reset_reason_t port_core_reset_classify(uint32_t f)
{
    if (f & (CORE_ST_RESET_FLAG_IWDG | CORE_ST_RESET_FLAG_WWDG)) return CORE_RESET_WATCHDOG;
    if (f & (CORE_ST_RESET_FLAG_SFT | CORE_ST_RESET_FLAG_OBL))   return CORE_RESET_SOFTWARE;
    if (f & (CORE_ST_RESET_FLAG_LPWR | CORE_ST_RESET_FLAG_FW))   return CORE_RESET_OTHER;
    if (f & CORE_ST_RESET_FLAG_POR)                              return CORE_RESET_POWER_ON;
    if (f & CORE_ST_RESET_FLAG_PIN)                              return CORE_RESET_PIN;
    return CORE_RESET_OTHER;
}

/* The RCC reset flags (CORE_ST_RESET_FLAG_*, bits 31..24). */
static inline uint32_t port_core_reset_read_flags(void)
{
    return ll_rcc_reset_flags();
}

/* Clear them (RMVF), so the next boot reads only its own reset. */
static inline void port_core_reset_clear_flags(void)
{
    ll_rcc_clear_reset_flags();
}

/* ============================================================
 * core_reset()
 * ============================================================ */

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

static void port_core_reset_l4_fix_empty_check(void)
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

/* Refresh the IWDG; harmless if the watchdog isn't running. */
static inline void port_core_reset_feed_watchdog(void)
{
    ll_iwdg_refresh();
}

/* Let queued serial output reach the terminal before the reset. Bounded. */
static inline void port_core_reset_flush(void)
{
#if defined(STM32L422xx) || defined(STM32H523xx)
    /* USB CDC: let queued output reach the terminal. Bounded. */
    hal_usb_cdc_flush_for_reset();
#endif
}

/* Last step before the system reset, with interrupts already off: on the
 * L4, correct a stale empty-flash verdict (which may itself reset, through
 * OBL_LAUNCH). Nothing on the other Cores. */
static inline void port_core_reset_prepare(void)
{
#if defined(STM32L422xx)
    port_core_reset_l4_fix_empty_check();
#endif
}

#endif /* PORT_CORE_RESET_HW_H */
