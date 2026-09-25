/**
 * ll_pwr.h — Low-level power control
 *
 * Sleep, Stop, and Standby mode entry. Voltage regulator
 * configuration and backup domain access.
 *
 * Power modes (roughly, across all families):
 *   Sleep:   CPU stopped, peripherals running. Wake by any interrupt.
 *   Stop:    Most clocks stopped, SRAM retained. Wake by EXTI.
 *   Standby: Everything off except RTC/IWDG. Wake by WKUP pin or RTC.
 */

#ifndef LL_PWR_H
#define LL_PWR_H

#include "ll_common.h"

/* ---- PWR base address ---- */

#if defined(STM32L011xx)
  #define PWR_BASE          0x40007000UL
#elif defined(STM32L422xx)
  #define PWR_BASE          0x40007000UL
#elif defined(STM32WBA55xx)
  #define PWR_BASE          0x46020800UL
#elif defined(STM32H523xx)
  #define PWR_BASE          0x44020800UL
#endif

/* ---- Cortex-M SCB registers (for sleep modes) ---- */

#define SCB_SCR             REG32(0xE000ED10UL)
#define SCB_SCR_SLEEPDEEP   (1UL << 2)
#define SCB_SCR_SLEEPONEXIT (1UL << 1)

/* ============================================================
 * PWR clock enable
 * ============================================================ */

static inline void ll_rcc_pwr_clk_enable(void)
{
#if defined(STM32L011xx)
    SET_BITS(REG32(RCC_BASE + 0x38UL), (1UL << 28));  /* APB1ENR: PWREN */
#elif defined(STM32L422xx)
    SET_BITS(REG32(RCC_BASE + 0x58UL), (1UL << 28));  /* APB1ENR1: PWREN */
#elif defined(STM32WBA55xx)
    /* WBA: PWR IS clock-gated — needs AHB4ENR.PWREN before any PWR access.
     * (The previous "always accessible" comment was wrong and left core_init's
     * PWR/regulator setup silently dropped, killing the radio.) */
    SET_BITS(REG32(RCC_BASE + 0x94UL), (1UL << 2));   /* AHB4ENR: PWREN */
    (void)REG32(RCC_BASE + 0x94UL);                   /* propagate enable */
#elif defined(STM32H523xx)
    /* H5: PWR is always accessible via SRD domain */
#endif
    (void)REG32(RCC_BASE);
}

/* ============================================================
 * Backup domain access
 * ============================================================ */

/**
 * Enable access to the backup domain (RTC, backup registers).
 * Must be called before writing to RTC or backup registers.
 */
static inline void ll_pwr_enable_backup_access(void)
{
#if defined(STM32L011xx)
    SET_BITS(REG32(PWR_BASE + 0x00UL), (1UL << 8));   /* CR: DBP */
    while (!(REG32(PWR_BASE + 0x00UL) & (1UL << 8))) ;
#elif defined(STM32L422xx)
    SET_BITS(REG32(PWR_BASE + 0x00UL), (1UL << 8));   /* CR1: DBP */
    while (!(REG32(PWR_BASE + 0x00UL) & (1UL << 8))) ;
#elif defined(STM32WBA55xx)
    /* WBA: enable the PWR bus clock (AHB4ENR.PWREN) BEFORE touching PWR, then
     * set DBP in the dedicated PWR_DBPR register (offset 0x28, bit 0).
     * RM0493 §11.10.9. The generic CR1-bit8 path is wrong for WBA and silently
     * no-ops without the clock — breaking every caller, including the radio
     * (linklayer_plat.c), which is why advertising died. Restores the correct
     * implementation that commit 7092941 removed from ll_rcc.h. */
    SET_BITS(REG32(RCC_BASE + 0x94UL), (1UL << 2));   /* AHB4ENR: PWREN */
    (void)REG32(RCC_BASE + 0x94UL);                   /* propagate enable */
    SET_BITS(REG32(PWR_BASE + 0x28UL), (1UL << 0));   /* PWR_DBPR: DBP */
#elif defined(STM32H523xx)
    SET_BITS(REG32(PWR_BASE + 0x24UL), (1UL << 0));   /* DBPCR: DBP (offset 0x024, bit 0) */
    while (!(REG32(PWR_BASE + 0x24UL) & (1UL << 0))) ;
#endif
}

/* ============================================================
 * Sleep mode
 * ============================================================ */

/**
 * Enter Sleep mode.
 * CPU clock stopped, all peripherals continue running.
 * Wakes on any enabled interrupt.
 *
 * Use WFI (wait for interrupt) or WFE (wait for event).
 */
static inline void ll_pwr_sleep_wfi(void)
{
    CLR_BITS(SCB_SCR, SCB_SCR_SLEEPDEEP);
    __asm volatile ("wfi");
}

static inline void ll_pwr_sleep_wfe(void)
{
    CLR_BITS(SCB_SCR, SCB_SCR_SLEEPDEEP);
    __asm volatile ("wfe");
}

/* ============================================================
 * Stop mode
 * ============================================================ */

/**
 * Enter Stop mode (lowest power with SRAM retention).
 *
 * All oscillators stopped except LSI/LSE. Wakes via EXTI
 * (GPIO, RTC alarm, etc.). After waking, clock is MSI/HSI
 * — you must reinitialize the PLL if needed.
 *
 * On L4: Stop 0, Stop 1, or Stop 2 available.
 * On WBA/H5: Stop 0 or Stop 1.
 */
static inline void ll_pwr_stop(void)
{
#if defined(STM32L011xx)
    /* L0: Set LPSDSR for low-power in Stop, clear PDDS for Stop (not Standby) */
    CLR_BITS(REG32(PWR_BASE + 0x00UL), (1UL << 1));   /* CR: PDDS=0 → Stop */
    SET_BITS(REG32(PWR_BASE + 0x00UL), (1UL << 0));   /* CR: LPSDSR → low-power regulator */
    /* RM0377 Table 39: Stop is entered only with WUF = 0 in PWR_CSR. An RTC
     * wakeup sets WUF, so without this every Stop after the first one was
     * silently skipped. CWUF clears it within 2 system clocks. */
    SET_BITS(REG32(PWR_BASE + 0x00UL), (1UL << 2));   /* CR: CWUF */

#elif defined(STM32L422xx)
    /* L4: Enter Stop 1 mode (good balance of power vs wake time)
       CR1 LPMS[2:0] = 001 → Stop 1 */
    MOD_BITS(REG32(PWR_BASE + 0x00UL), 0x7UL, 0x1UL);

#elif defined(STM32WBA55xx)
    /* WBA: CR1 LPMS[2:0] = 001 → Stop 1. Stop 1 exits on HSI16 in voltage
     * range 2: FLASH and SRAM wait states must be >= 1 before entry and
     * HDIV5 comes back set (RM0493 §11.7.7) — core_power.h handles both. */
    MOD_BITS(REG32(PWR_BASE + 0x00UL), 0x7UL, 0x1UL);

#elif defined(STM32H523xx)
    /* H5: PMCR LPMS = 00 → Stop 0 (Stop 1 doesn't wake from RTC on H523) */
    MOD_BITS(REG32(PWR_BASE + 0x00UL), 0x7UL, 0x0UL);
#endif

    /* Set SLEEPDEEP bit and execute WFI */
    SET_BITS(SCB_SCR, SCB_SCR_SLEEPDEEP);
    __asm volatile ("dsb" ::: "memory");
    __asm volatile ("isb");
    __asm volatile ("wfi");

    /* After waking: clear SLEEPDEEP */
    CLR_BITS(SCB_SCR, SCB_SCR_SLEEPDEEP);
}

/* ============================================================
 * Wakeup flags
 * ============================================================ */

/**
 * Clear the PWR wake-up flags (wake-up pins and, on the L0, the combined
 * WUF). Stop (L0) and Standby (all) are skipped while one is set.
 */
static inline void ll_pwr_clear_wakeup_flags(void)
{
#if defined(STM32L011xx)
    SET_BITS(REG32(PWR_BASE + 0x00UL), (1UL << 2));   /* CR: CWUF */
#elif defined(STM32L422xx)
    /* PWR_SCR is at 0x18 (RM0394 §5.4.7): CWUF1..5 = bits 0-4. 0x14 is the
     * read-only PWR_SR2, which is what this used to write. */
    REG32(PWR_BASE + 0x18UL) = 0x1FUL;
#elif defined(STM32H523xx)
    REG32(PWR_BASE + 0x40UL) = 0x1F;                  /* WUSCR: clear all WUF */
#elif defined(STM32WBA55xx)
    /* PWR_WUSCR is at 0x48 (RM0493 §11.10.15): CWUF1..8 = bits 0-7. 0x14 is
     * PWR_WUCR1 — writing 0x1F there ENABLED wake-up pins 1-5. */
    REG32(PWR_BASE + 0x48UL) = 0xFFUL;
#endif
}

/* ============================================================
 * Standby mode
 * ============================================================ */

/**
 * Enter Standby mode (lowest power, SRAM lost).
 *
 * Only RTC, IWDG, and wakeup pins remain powered.
 * On wake, the MCU resets (starts from Reset_Handler).
 *
 * WARNING: All SRAM contents are lost. Save state to backup
 * registers or flash before entering standby.
 */
static inline void ll_pwr_standby(void)
{
#if defined(STM32L011xx)
    SET_BITS(REG32(PWR_BASE + 0x00UL), (1UL << 1));   /* CR: PDDS=1 → Standby */

#elif defined(STM32L422xx)
    /* CR1 LPMS[2:0] = 011 → Standby */
    MOD_BITS(REG32(PWR_BASE + 0x00UL), 0x7UL, 0x3UL);

#elif defined(STM32WBA55xx)
    /* CR1 LPMS[2:0] = 10x → Standby (RM0493 §11.10.1). 011 is reserved. */
    MOD_BITS(REG32(PWR_BASE + 0x00UL), 0x7UL, 0x4UL);

#elif defined(STM32H523xx)
    MOD_BITS(REG32(PWR_BASE + 0x00UL), 0x7UL, 0x3UL);
#endif

    /* Clear wakeup flags (Standby is not entered while one is set) */
    ll_pwr_clear_wakeup_flags();

    SET_BITS(SCB_SCR, SCB_SCR_SLEEPDEEP);
    __asm volatile ("dsb" ::: "memory");
    __asm volatile ("wfi");

    /* Should never reach here — MCU resets on standby wake */
    while (1)
        ;
}

/* ============================================================
 * Low-Power Run mode (STM32L0 only)
 * ============================================================ */

/**
 * Enter Low-Power Run mode on STM32L0.
 * Call AFTER switching SYSCLK to MSI ≤ 1.048MHz.
 * Lowers Vcore to the low-power regulator; active current ~8µA.
 */
static inline void ll_pwr_lp_run_enable(void)
{
#if defined(STM32L011xx)
    SET_BITS(REG32(PWR_BASE + 0x00UL), (1UL << 14));  /* CR: LPRUN */
#endif
}

/**
 * Check if we woke from standby. The flag is not cleared by a system reset,
 * so it stays set across later resets until ll_pwr_clear_standby_flag().
 */
static inline int ll_pwr_woke_from_standby(void)
{
#if defined(STM32L011xx)
    return (REG32(PWR_BASE + 0x04UL) & (1UL << 1)) != 0;  /* CSR: SBF */
#elif defined(STM32L422xx)
    return (REG32(PWR_BASE + 0x10UL) & (1UL << 8)) != 0;  /* SR1: SBF */
#elif defined(STM32H523xx)
    return (REG32(PWR_BASE + 0x04UL) & (1UL << 6)) != 0;  /* PMSR: SBF (bit 6) */
#elif defined(STM32WBA55xx)
    /* PWR_SR at 0x38, SBF = bit 2 (RM0493 §11.10.12). 0x10 is PWR_SVMCR. */
    return (REG32(PWR_BASE + 0x38UL) & (1UL << 2)) != 0;
#endif
}

/** Clear the standby flag */
static inline void ll_pwr_clear_standby_flag(void)
{
#if defined(STM32L011xx)
    SET_BITS(REG32(PWR_BASE + 0x00UL), (1UL << 3));   /* CR: CSBF */
#elif defined(STM32L422xx)
    /* PWR_SCR (0x18, write-only): CSBF = bit 8 (RM0394 §5.4.7). */
    REG32(PWR_BASE + 0x18UL) = (1UL << 8);
#elif defined(STM32H523xx)
    SET_BITS(REG32(PWR_BASE + 0x00UL), (1UL << 7));   /* PMCR: CSSF (clears SBF+STOPF) */
#elif defined(STM32WBA55xx)
    /* PWR_SR (0x38): writing CSSF (bit 0) clears SBF and STOPF (RM0493 §11.10.12). */
    REG32(PWR_BASE + 0x38UL) = (1UL << 0);
#endif
}

#endif /* LL_PWR_H */
