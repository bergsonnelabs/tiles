/**
 * hal_dfu.h — Reboot into DFU bootloader
 *
 * Two DFU modes are supported:
 *
 * 1. Custom bootloader (BOOTLOADER=1). PARKED: gated behind
 *    CUSTOM_BOOTLOADER_ACK=1; flashing it onto a ROM-DFU board corrupts the
 *    resident app. Use the ROM bootloader.
 *    App at 0x08002000, custom DFU 1.1 bootloader at 0x08000000.
 *    hal_dfu_reboot() writes magic → reset → bootloader enters DFU.
 *
 * 2. ROM bootloader (ROM_DFU=1):
 *    App at 0x08000000 (no custom bootloader needed).
 *    hal_dfu_reboot() writes magic → reset → core_init() detects magic
 *    → hal_dfu_jump_to_rom() remaps system memory and jumps to the
 *    factory-programmed ST ROM bootloader (DfuSe protocol, 0483:DF11).
 *
 * The magic lives in the last 16 bytes of SRAM, which is reserved
 * by the linker scripts (LENGTH = <total> - 16). The startup code
 * doesn't touch this area, and SRAM survives system reset.
 *
 * Typical trigger: 1200-baud touch on USB CDC (detected in
 * hal_usb_cdc.c) calls hal_dfu_reboot().
 */

#ifndef HAL_DFU_H
#define HAL_DFU_H

#include "ll_common.h"

/* Magic value and its fixed RAM address (top of SRAM, outside linker regions).
 * The linker scripts reserve the last 16 bytes of SRAM for this. */
#define DFU_MAGIC           0xDEADBEEFUL

#if defined(STM32L422xx)
  /* L422: 40KB SRAM, top = 0x2000A000. The linker reserves the last 16 bytes;
   * word 0 is the DFU magic, words 1-3 hold the brick-recovery strike state. */
  #define DFU_MAGIC_ADDR      (*(volatile uint32_t *)0x20009FF0UL)
  #define DFU_STRIKE_ADDR     (*(volatile uint32_t *)0x20009FF4UL)
  #define DFU_STRIKE_TAG_ADDR (*(volatile uint32_t *)0x20009FF8UL)
  #define DFU_CAUSE_ADDR      (*(volatile uint32_t *)0x20009FFCUL)
  #define DFU_ROM_ADDR      0x1FFF0000UL
#elif defined(STM32H523xx)
  /* H523: 272KB SRAM, top = 0x20044000 */
  #define DFU_MAGIC_ADDR      (*(volatile uint32_t *)0x20043FF0UL)
  #define DFU_STRIKE_ADDR     (*(volatile uint32_t *)0x20043FF4UL)
  #define DFU_STRIKE_TAG_ADDR (*(volatile uint32_t *)0x20043FF8UL)
  #define DFU_CAUSE_ADDR      (*(volatile uint32_t *)0x20043FFCUL)
  #define DFU_ROM_ADDR      0x0BF97000UL  /* AN2606: H523/H533 bootloader entry */
#endif

/* Validity tag for the recovery words: distinguishes "carried across a warm
 * reset" from "random SRAM after a cold power-on" (which reads as 0 strikes). */
#define DFU_STRIKE_TAG      0xB0BB1E00UL

#ifdef DFU_STRIKE_TAG_ADDR
/** Consecutive watchdog-reset strikes carried across warm reset (0 if cold). */
static inline uint32_t hal_recovery_strikes(void)
{
    return (DFU_STRIKE_TAG_ADDR == DFU_STRIKE_TAG) ? DFU_STRIKE_ADDR : 0UL;
}
/** True if the recovery words survived a warm reset (tag valid). */
static inline int hal_recovery_valid(void)
{
    return DFU_STRIKE_TAG_ADDR == DFU_STRIKE_TAG;
}
/** Set the strike count and stamp the validity tag. */
static inline void hal_recovery_set_strikes(uint32_t n)
{
    DFU_STRIKE_ADDR = n;
    DFU_STRIKE_TAG_ADDR = DFU_STRIKE_TAG;
}
/** Stash whether this boot was a watchdog reset (so the app's
 *  core_watchdog_caused_reset() still works after we clear the HW flag). */
static inline void hal_recovery_stash_cause(int was_watchdog)
{
    DFU_CAUSE_ADDR = was_watchdog ? 1UL : 0UL;
    DFU_STRIKE_TAG_ADDR = DFU_STRIKE_TAG;
}
/** Read back the stashed reset cause (1 = watchdog). */
static inline uint32_t hal_recovery_stashed_cause(void)
{
    return DFU_CAUSE_ADDR;
}
#endif /* DFU_STRIKE_TAG_ADDR */

/* SCB AIRCR: Application Interrupt and Reset Control Register */
#define SCB_AIRCR           REG32(0xE000ED0CUL)
#define SCB_AIRCR_SYSRESETREQ  (1UL << 2)
#define SCB_AIRCR_VECTKEY      (0x05FAUL << 16)

/* The DFU entry points exist only where hal_dfu.h defined the reserved-SRAM
 * addresses above (USB-capable Cores). On a Core without a DFU path — e.g.
 * Core.ST.W5 / STM32WBA55, which has no USB — these would reference undefined
 * DFU_MAGIC_ADDR / DFU_ROM_ADDR, and merely INCLUDING this header would fail to
 * compile. Self-guard them the same way the recovery helpers above do, so
 * core_watchdog.h (which includes this header unconditionally) stays usable on
 * every Core. */
#ifdef DFU_MAGIC_ADDR

/**
 * Reboot into DFU mode. Does not return.
 *
 * Writes the DFU magic to a reserved SRAM address, then triggers
 * a system reset. SRAM survives system reset, so the bootloader
 * (custom or ROM) will see the magic on the next boot.
 */
static inline void hal_dfu_reboot(void) __attribute__((noreturn));
static inline void hal_dfu_reboot(void)
{
    DFU_MAGIC_ADDR = DFU_MAGIC;

    __asm volatile ("dsb" ::: "memory");

    /* Request system reset */
    SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;

    __asm volatile ("dsb" ::: "memory");
    while (1)
        ;
}

/**
 * Jump to the STM32 ROM (system memory) bootloader. Does not return.
 *
 * Called from core_init() when ROM_DFU is enabled and the DFU magic
 * was found. Runs before ANY peripheral or clock init, so the MCU is
 * in near-reset state. The ROM bootloader configures its own clocks
 * (HSI48 + CRS for USB) and enumerates as 0483:DF11 with DfuSe.
 *
 * L4 (Cortex-M4): Remaps system memory to 0x00000000 via SYSCFG_MEMRMP,
 *   then jumps. The remap ensures the ROM bootloader reads its own
 *   vectors from address 0 and doesn't bounce back to flash.
 *
 * H5 (Cortex-M33): No MEMRMP equivalent in SBS. Instead, sets VTOR
 *   directly to the ROM bootloader base — the M33 always respects VTOR
 *   for vector fetches, so no memory remap is needed.
 */
static inline void hal_dfu_jump_to_rom(void) __attribute__((noreturn));
static inline void hal_dfu_jump_to_rom(void)
{
#if defined(STM32L422xx)
    /* Enable SYSCFG clock: RCC_APB2ENR bit 0 (SYSCFGEN)
     * RCC_BASE = 0x40021000, APB2ENR offset = 0x60 */
    SET_BITS(REG32(0x40021060UL), (1UL << 0));
    (void)REG32(0x40021060UL);  /* read-back fence */

    /* Remap system memory to 0x00000000:
     * SYSCFG_MEMRMP (0x40010000) bits [2:0] = 001 (System Flash) */
    MOD_BITS(REG32(0x40010000UL), 0x07UL, 0x01UL);

#elif defined(STM32H523xx)
    /* H5 Cortex-M33: Set VTOR to ROM bootloader base.
     * SBS has no MEMRMP — VTOR handles vector relocation directly. */
    REG32(0xE000ED08UL) = DFU_ROM_ADDR;
#endif

    __asm volatile ("dsb" ::: "memory");
    __asm volatile ("isb" ::: "memory");

    /* Read ROM bootloader's initial SP and reset vector */
    volatile uint32_t *rom = (volatile uint32_t *)DFU_ROM_ADDR;
    uint32_t sp = rom[0];
    uint32_t rv = rom[1];

    __asm volatile (
        "msr msp, %0 \n"
        "bx  %1      \n"
        :
        : "r" (sp), "r" (rv)
        : "memory"
    );

    __builtin_unreachable();
}

#endif /* DFU_MAGIC_ADDR */

#endif /* HAL_DFU_H */
