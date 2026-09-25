/**
 * ll_iwdg.h — Low-level Independent Watchdog
 *
 * The IWDG is clocked from LSI (LL_LSI_HZ: 32 kHz nominal, 37 kHz on
 * the L0) and runs independently of the main clock. Once started, it
 * cannot be stopped — only the MCU reset will disable it. The register
 * layout is identical across all STM32 families.
 *
 * It keeps counting in Stop and Standby: on the L4 and WBA with the option
 * bytes as shipped (IWDG_STOP / IWDG_STDBY = 1), and always on the L0, whose
 * freeze-in-low-power capability was removed (RM0377 revision history).
 * core_power.h sleeps in watchdog-safe chunks because of this.
 *
 * If the watchdog is not refreshed before the timeout, it resets
 * the MCU. Use this for fault recovery in deployed systems.
 *
 * Timeout calculation:
 *   timeout_ms = (reload + 1) * prescaler * 1000 / LL_LSI_HZ
 *
 * Prescaler values: 4, 8, 16, 32, 64, 128, 256
 * Reload range: 0-4095 (12-bit)
 *
 * Common configurations (32 kHz LSI; the helpers below scale for the L0):
 *   1s:   prescaler=32, reload=999   → 1ms × 1000 = 1000ms
 *   2s:   prescaler=32, reload=1999
 *   5s:   prescaler=64, reload=2499
 *   10s:  prescaler=256, reload=1249
 *   28s:  prescaler=256, reload=4095  (maximum; ~28.3 s on the L0)
 */

#ifndef LL_IWDG_H
#define LL_IWDG_H

#include "ll_common.h"

/* ---- IWDG base address (same across all families) ---- */

#define IWDG_BASE           0x40003000UL

/* ---- IWDG registers ---- */

#define IWDG_KR             REG32(IWDG_BASE + 0x00UL)  /* Key register */
#define IWDG_PR             REG32(IWDG_BASE + 0x04UL)  /* Prescaler register */
#define IWDG_RLR            REG32(IWDG_BASE + 0x08UL)  /* Reload register */
#define IWDG_SR             REG32(IWDG_BASE + 0x0CUL)  /* Status register */

/* ---- Key values ---- */

#define LL_IWDG_KEY_UNLOCK  0x5555UL   /* Unlock PR and RLR for writing */
#define LL_IWDG_KEY_RELOAD  0xAAAAUL   /* Refresh the watchdog (feed the dog) */
#define LL_IWDG_KEY_START   0xCCCCUL   /* Start the watchdog (irreversible!) */

/* ---- Prescaler values (tick times at 32 kHz; ~14 % shorter on the L0) ---- */

#define LL_IWDG_PSC_4       0x0UL      /* LSI / 4   = ~8kHz    → 0.125ms/tick */
#define LL_IWDG_PSC_8       0x1UL      /* LSI / 8   = ~4kHz    → 0.25ms/tick  */
#define LL_IWDG_PSC_16      0x2UL      /* LSI / 16  = ~2kHz    → 0.5ms/tick   */
#define LL_IWDG_PSC_32      0x3UL      /* LSI / 32  = ~1kHz    → 1ms/tick     */
#define LL_IWDG_PSC_64      0x4UL      /* LSI / 64  = ~500Hz   → 2ms/tick     */
#define LL_IWDG_PSC_128     0x5UL      /* LSI / 128 = ~250Hz   → 4ms/tick     */
#define LL_IWDG_PSC_256     0x6UL      /* LSI / 256 = ~125Hz   → 8ms/tick     */

/* ---- SR bit definitions ---- */

#define LL_IWDG_SR_PVU      (1UL << 0)    /* Prescaler value update */
#define LL_IWDG_SR_RVU      (1UL << 1)    /* Reload value update */

/* ============================================================
 * Configuration and control
 * ============================================================ */

/**
 * Initialize and start the independent watchdog.
 *   prescaler: LL_IWDG_PSC_* value
 *   reload:    reload value (0-4095)
 *
 * WARNING: Once started, the IWDG cannot be stopped. The only
 * way to disable it is a full MCU reset. Make sure your main
 * loop calls ll_iwdg_refresh() regularly.
 *
 * The LSI oscillator is automatically started when IWDG is enabled.
 */
static inline void ll_iwdg_init(uint32_t prescaler, uint32_t reload)
{
    /* Start the watchdog (enables LSI automatically) */
    IWDG_KR = LL_IWDG_KEY_START;

    /* Unlock registers */
    IWDG_KR = LL_IWDG_KEY_UNLOCK;

    /* Set prescaler */
    IWDG_PR = prescaler;

    /* Set reload value */
    IWDG_RLR = reload & 0xFFFUL;

    /* Wait for registers to be updated */
    while (IWDG_SR & (LL_IWDG_SR_PVU | LL_IWDG_SR_RVU))
        ;

    /* Initial refresh */
    IWDG_KR = LL_IWDG_KEY_RELOAD;
}

/**
 * Refresh (feed) the watchdog. Call this periodically in your
 * main loop to prevent a watchdog reset.
 */
static inline void ll_iwdg_refresh(void)
{
    IWDG_KR = LL_IWDG_KEY_RELOAD;
}

/* ============================================================
 * Convenience: common timeout values
 * ============================================================ */

/* Reload values scale with the per-family LSI nominal (ll_common.h), so these
 * are the same registers as before at 32 kHz (999 / 1999 / 2499 / 1249) and
 * correct on the 37 kHz L0 instead of ~14 % short. */

/** Start IWDG with ~1 second timeout */
static inline void ll_iwdg_init_1s(void)
{
    ll_iwdg_init(LL_IWDG_PSC_32, (1UL * LL_LSI_HZ) / 32UL - 1UL);
}

/** Start IWDG with ~2 second timeout */
static inline void ll_iwdg_init_2s(void)
{
    ll_iwdg_init(LL_IWDG_PSC_32, (2UL * LL_LSI_HZ) / 32UL - 1UL);
}

/** Start IWDG with ~5 second timeout */
static inline void ll_iwdg_init_5s(void)
{
    ll_iwdg_init(LL_IWDG_PSC_64, (5UL * LL_LSI_HZ) / 64UL - 1UL);
}

/** Start IWDG with ~10 second timeout */
static inline void ll_iwdg_init_10s(void)
{
    ll_iwdg_init(LL_IWDG_PSC_256, (10UL * LL_LSI_HZ) / 256UL - 1UL);
}

/* ============================================================
 * Reset detection
 * ============================================================ */

/**
 * Check if the last reset was caused by the IWDG.
 * Call early in main() to detect watchdog resets.
 */
static inline int ll_iwdg_caused_reset(void)
{
#if defined(STM32L011xx)
    return (REG32(RCC_BASE + 0x50UL) & (1UL << 29)) != 0;  /* CSR: IWDGRSTF */
#elif defined(STM32L422xx)
    return (REG32(RCC_BASE + 0x94UL) & (1UL << 29)) != 0;  /* CSR: IWDGRSTF */
#elif defined(STM32WBA55xx)
    /* RM0493: RCC_CSR is at offset 0x0F4 (was 0xE4 here, which reads 0 — so
     * this returned false after a real watchdog reset. Caught on hardware.) */
    return (REG32(RCC_BASE + 0xF4UL) & (1UL << 29)) != 0;  /* CSR: IWDGRSTF */
#elif defined(STM32H523xx)
    return (REG32(RCC_BASE + 0xF4UL) & (1UL << 29)) != 0;
#endif
}

/**
 * Clear the reset flags (so the next check gives a fresh result).
 * Clears all reset flags, not just IWDG.
 */
static inline void ll_rcc_clear_reset_flags(void)
{
#if defined(STM32L011xx)
    SET_BITS(REG32(RCC_BASE + 0x50UL), (1UL << 23));  /* CSR: RMVF */
#elif defined(STM32L422xx)
    SET_BITS(REG32(RCC_BASE + 0x94UL), (1UL << 23));  /* CSR: RMVF */
#elif defined(STM32WBA55xx)
    SET_BITS(REG32(RCC_BASE + 0xF4UL), (1UL << 23));  /* CSR: RMVF */
#elif defined(STM32H523xx)
    SET_BITS(REG32(RCC_BASE + 0xF4UL), (1UL << 23));
#endif
}

#endif /* LL_IWDG_H */
