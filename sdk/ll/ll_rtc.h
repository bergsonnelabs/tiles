/**
 * ll_rtc.h — Low-level RTC (Real-Time Clock) operations
 *
 * Time/date set and read, wakeup timer, alarm configuration, backup
 * registers. Requires backup domain access (ll_pwr_enable_backup_access).
 *
 * Two RTC generations are in play, and the register maps differ:
 *   L0 (STM32L011, RM0377 §22.7): the older RTC. RTC_ISR at 0x0C carries
 *       INIT/INITF plus the rc_w0 event flags, ALRAWF gates alarm writes,
 *       RTC_ALRMAR is at 0x1C and the backup registers (5) sit inside the
 *       RTC at 0x50.
 *   L4 (STM32L422, RM0394 §35/§36), WBA55 (RM0493 §36/§37), H5: the newer
 *       RTC plus a separate TAMP block. RTC_ICSR at 0x0C is init/status only,
 *       event flags live in RTC_SR (0x50) and clear through RTC_SCR (0x5C),
 *       there is no ALRAWF, RTC_ALRMAR is at 0x40 and the 32 backup registers
 *       are TAMP_BKPxR at TAMP + 0x100. The L422 is NOT the L43x layout: RM0394
 *       §37 (the older RTC) applies to L43x..L46x only.
 *
 * RTC is clocked from LSE (32.768kHz crystal) or LSI (LL_LSI_HZ, ll_common.h).
 * The async/sync prescalers divide the clock to get a 1Hz tick:
 *   LSE: PREDIV_A=127, PREDIV_S=255 → 32768/128/256 = 1Hz
 *   LSI: PREDIV_A=127, PREDIV_S=LL_LSI_HZ/128-1 (249 at 32 kHz, 288 on the
 *        37 kHz L0) → ~1Hz
 */

#ifndef LL_RTC_H
#define LL_RTC_H

#include "ll_common.h"
#include "ll_pwr.h"
#include "ll_rcc.h"

/* ---- RTC base address ---- */

#if defined(STM32L011xx)
  #define RTC_BASE          0x40002800UL
#elif defined(STM32L422xx)
  #define RTC_BASE          0x40002800UL
#elif defined(STM32WBA55xx)
  #define RTC_BASE          0x46007800UL
#elif defined(STM32H523xx)
  #define RTC_BASE          0x44007800UL
#endif

/* ---- RTC register offsets ---- */

#define RTC_TR              REG32(RTC_BASE + 0x00UL)  /* Time register */
#define RTC_DR              REG32(RTC_BASE + 0x04UL)  /* Date register */
#if defined(STM32L011xx)
  #define RTC_SSR           REG32(RTC_BASE + 0x28UL)  /* Sub second (RM0377 §22.7.10) */
#else
  #define RTC_SSR           REG32(RTC_BASE + 0x08UL)  /* Sub second (RM0394 §35.6.3, RM0493 §36.6.3) */
#endif
#define RTC_ICSR            REG32(RTC_BASE + 0x0CUL)  /* Init/status (L4+) */
#define RTC_PRER            REG32(RTC_BASE + 0x10UL)  /* Prescaler register */
#define RTC_WUTR            REG32(RTC_BASE + 0x14UL)  /* Wakeup timer */
#if defined(STM32L011xx)
  #define RTC_CR            REG32(RTC_BASE + 0x08UL)  /* Control (RM0377 §22.7.3; the older RTC) */
#else
  #define RTC_CR            REG32(RTC_BASE + 0x18UL)  /* Control (RM0394 §35.6.x, RM0493 §36.6.x) */
#endif
#define RTC_WPR             REG32(RTC_BASE + 0x24UL)  /* Write protection */

#if defined(STM32L011xx)
  /* L0: ISR at 0x0C holds INIT/INITF and the event flags (rc_w0). */
  #define RTC_ISR           REG32(RTC_BASE + 0x0CUL)
#else
  /* Newer RTC: event flags in SR (read-only), cleared by writing 1 to SCR.
   * RM0394 §35.6.18/§35.6.20, RM0493 §36.6.20/§36.6.23. */
  #define RTC_SR            REG32(RTC_BASE + 0x50UL)
  #define RTC_SCR           REG32(RTC_BASE + 0x5CUL)
#endif

/* ---- CR bit definitions ---- */

#define LL_RTC_CR_WUTE          (1UL << 10)   /* Wakeup timer enable */
#define LL_RTC_CR_ALRAE         (1UL << 8)    /* Alarm A enable */
#define LL_RTC_CR_WUTIE         (1UL << 14)   /* Wakeup timer interrupt enable */
#define LL_RTC_CR_ALRAIE        (1UL << 12)   /* Alarm A interrupt enable */
#define LL_RTC_CR_FMT           (1UL << 6)    /* Hour format: 1=12h, 0=24h */

/* Wakeup clock selection (WUCKSEL[2:0] in CR) */
#define LL_RTC_WUCKSEL_DIV16    0x0UL   /* RTC/16 */
#define LL_RTC_WUCKSEL_DIV8     0x1UL
#define LL_RTC_WUCKSEL_DIV4     0x2UL
#define LL_RTC_WUCKSEL_DIV2     0x3UL
#define LL_RTC_WUCKSEL_1HZ      0x4UL   /* 1Hz from calendaring clock */

/* ---- ISR/ICSR bit definitions ---- */

#define LL_RTC_ISR_INITF        (1UL << 6)    /* Init mode flag */
#define LL_RTC_ISR_INIT         (1UL << 7)    /* Init mode enable */
#define LL_RTC_ISR_RSF          (1UL << 5)    /* Registers sync flag */
#define LL_RTC_ISR_WUTF         (1UL << 10)   /* Wakeup timer flag (L0 ISR only) */
#define LL_RTC_ISR_WUTWF        (1UL << 2)    /* Wakeup timer write flag */
#define LL_RTC_ISR_ALRAF        (1UL << 8)    /* Alarm A flag (L0 ISR only) */
#define LL_RTC_ISR_ALRAWF       (1UL << 0)    /* Alarm A write flag (L0 ISR only) */

/* ---- SR / SCR bit definitions (newer RTC) ---- */

#define LL_RTC_SR_ALRAF         (1UL << 0)    /* SR: Alarm A flag; SCR: CALRAF */
#define LL_RTC_SR_WUTF          (1UL << 2)    /* SR: wakeup flag;  SCR: CWUTF */

/* ---- RTC wakeup-timer interrupt routing ----
 * NVIC line for the wakeup timer and, on L0/L4, the EXTI line it rides on
 * (RM0377 EXTI line 20 + vector 2 "RTC"; RM0394 EXTI line 20 + vector 3
 * "RTC_WKUP"). The WBA55 RTC is not on EXTI at all (RM0493 Table 136 lists
 * lines 0-18) — it wakes Stop directly and interrupts on vector 2. */
#if defined(STM32L011xx)
  #define LL_RTC_WKUP_IRQn      2UL
  #define LL_RTC_WKUP_EXTI      20UL
#elif defined(STM32L422xx)
  #define LL_RTC_WKUP_IRQn      3UL
  #define LL_RTC_WKUP_EXTI      20UL
#elif defined(STM32WBA55xx)
  #define LL_RTC_WKUP_IRQn      2UL
#elif defined(STM32H523xx)
  #define LL_RTC_WKUP_IRQn      2UL
#endif

/* ============================================================
 * BCD helpers
 * ============================================================ */

static inline uint8_t ll_bcd_to_bin(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static inline uint8_t ll_bin_to_bcd(uint8_t bin)
{
    return ((bin / 10) << 4) | (bin % 10);
}

/* ============================================================
 * Write protection
 * ============================================================ */

/** Unlock RTC write protection */
static inline void ll_rtc_unlock(void)
{
    RTC_WPR = 0xCAUL;
    RTC_WPR = 0x53UL;
}

/** Re-enable RTC write protection */
static inline void ll_rtc_lock(void)
{
    RTC_WPR = 0xFFUL;
}

/* ============================================================
 * LSE / LSI clock enable
 * ============================================================ */

/**
 * Enable LSE (32.768kHz external crystal).
 * Requires backup domain access to be enabled first.
 * A no-op on Core.ST.L0: it has no crystal, and its OSC32 pins are board nets.
 */
static inline void ll_rcc_lse_enable(void)
{
#if defined(STM32L011xx)
    /* Deliberately empty. Core.ST.L0.1 has no 32 kHz crystal, and LSEON would
     * hand PC14/PC15 (OSC32_IN/OUT) to the oscillator, taking them from the
     * board. (LSEON is RCC_CSR bit 8, RM0377 §7.3.20 — this used to set bit 0,
     * which is LSION.) */
#elif defined(STM32L422xx)
    SET_BITS(REG32(RCC_BASE + 0x90UL), (1UL << 0));   /* BDCR: LSEON */
#elif defined(STM32WBA55xx)
    SET_BITS(REG32(RCC_BASE + 0xF0UL), (1UL << 0));   /* BDCR1: LSEON (RM0493 §12.8.39) */
#elif defined(STM32H523xx)
    SET_BITS(REG32(RCC_BASE + 0xF0UL), (1UL << 0));   /* BDCR: LSEON */
#endif
}

static inline int ll_rcc_lse_ready(void)
{
#if defined(STM32L011xx)
    return (REG32(RCC_BASE + 0x50UL) & (1UL << 9)) != 0;   /* CSR: LSERDY */
#elif defined(STM32L422xx)
    return (REG32(RCC_BASE + 0x90UL) & (1UL << 1)) != 0;
#elif defined(STM32WBA55xx)
    return (REG32(RCC_BASE + 0xF0UL) & (1UL << 1)) != 0;   /* BDCR1: LSERDY */
#elif defined(STM32H523xx)
    return (REG32(RCC_BASE + 0xF0UL) & (1UL << 1)) != 0;
#else
    return 0;
#endif
}

/**
 * Enable LSI (internal RC, LL_LSI_HZ nominal). On the WBA55 this is LSI1.
 */
static inline void ll_rcc_lsi_enable(void)
{
#if defined(STM32L011xx)
    SET_BITS(REG32(RCC_BASE + 0x50UL), (1UL << 0));   /* CSR: LSION (RM0377 §7.3.20) */
#elif defined(STM32L422xx)
    SET_BITS(REG32(RCC_BASE + 0x94UL), (1UL << 0));   /* CSR: LSION */
#elif defined(STM32WBA55xx)
    SET_BITS(REG32(RCC_BASE + 0xF0UL), (1UL << 26));  /* BDCR1: LSI1ON (RM0493 §12.8.39) */
#elif defined(STM32H523xx)
    SET_BITS(REG32(RCC_BASE + 0xF0UL), (1UL << 26));  /* BDCR: LSION */
#endif
}

static inline int ll_rcc_lsi_ready(void)
{
#if defined(STM32L011xx)
    return (REG32(RCC_BASE + 0x50UL) & (1UL << 1)) != 0;   /* CSR: LSIRDY */
#elif defined(STM32L422xx)
    return (REG32(RCC_BASE + 0x94UL) & (1UL << 1)) != 0;
#elif defined(STM32WBA55xx)
    return (REG32(RCC_BASE + 0xF0UL) & (1UL << 27)) != 0;  /* BDCR1: LSI1RDY */
#elif defined(STM32H523xx)
    return (REG32(RCC_BASE + 0xF0UL) & (1UL << 27)) != 0;  /* BDCR: LSIRDY */
#else
    return 0;
#endif
}

/**
 * Select RTC clock source. Call before enabling RTC.
 *   0 = no clock, 1 = LSE, 2 = LSI, 3 = HSE/32
 */
static inline void ll_rcc_rtc_set_source(uint32_t source)
{
#if defined(STM32L011xx)
    MOD_BITS(REG32(RCC_BASE + 0x50UL), 0x3UL << 16, source << 16);
#elif defined(STM32L422xx)
    MOD_BITS(REG32(RCC_BASE + 0x90UL), 0x3UL << 8, source << 8);  /* BDCR RTCSEL */
#elif defined(STM32WBA55xx)
    MOD_BITS(REG32(RCC_BASE + 0xF0UL), 0x3UL << 8, source << 8);  /* BDCR1 RTCSEL */
#elif defined(STM32H523xx)
    MOD_BITS(REG32(RCC_BASE + 0xF0UL), 0x3UL << 8, source << 8);
#endif
}

/** Current RTC clock source (RTCSEL): 0 = none, 1 = LSE, 2 = LSI, 3 = HSE/32. */
static inline uint32_t ll_rcc_rtc_get_source(void)
{
#if defined(STM32L011xx)
    return (REG32(RCC_BASE + 0x50UL) >> 16) & 0x3UL;   /* CSR RTCSEL */
#elif defined(STM32L422xx)
    return (REG32(RCC_BASE + 0x90UL) >> 8) & 0x3UL;    /* BDCR RTCSEL */
#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    return (REG32(RCC_BASE + 0xF0UL) >> 8) & 0x3UL;    /* BDCR(1) RTCSEL */
#else
    return 0;
#endif
}

/** Enable RTC peripheral clock */
static inline void ll_rcc_rtc_enable(void)
{
#if defined(STM32L011xx)
    SET_BITS(REG32(RCC_BASE + 0x50UL), (1UL << 18));  /* CSR: RTCEN */
#elif defined(STM32L422xx)
    SET_BITS(REG32(RCC_BASE + 0x90UL), (1UL << 15));  /* BDCR: RTCEN */
#elif defined(STM32WBA55xx)
    /* No RTCEN on the WBA: a non-zero RTCSEL both selects and enables the
     * RTC/TAMP kernel clock (RM0493 §12.8.39). Bit 15 is reserved. */
#elif defined(STM32H523xx)
    SET_BITS(REG32(RCC_BASE + 0xF0UL), (1UL << 15));
#endif
}

/** Returns 1 if the RTC kernel clock is enabled. */
static inline int ll_rcc_rtc_enabled(void)
{
#if defined(STM32L011xx)
    return (REG32(RCC_BASE + 0x50UL) & (1UL << 18)) != 0;  /* CSR: RTCEN */
#elif defined(STM32L422xx)
    return (REG32(RCC_BASE + 0x90UL) & (1UL << 15)) != 0;  /* BDCR: RTCEN */
#elif defined(STM32WBA55xx)
    return ll_rcc_rtc_get_source() != 0;
#elif defined(STM32H523xx)
    return (REG32(RCC_BASE + 0xF0UL) & (1UL << 15)) != 0;
#else
    return 0;
#endif
}

/**
 * Enable the RTC (and TAMP) register-interface clock. Without it the RTC
 * reads as 0 and ignores writes, so every INITF/WUTWF poll hangs.
 *   L0: none — the RTC has no separate bus-clock enable.
 *   L4: RCC_APB1ENR1.RTCAPBEN (bit 10, set at reset; RM0394 §6.4.18).
 *   WBA: RCC_APB7ENR.RTCAPBEN (bit 21, clear at reset; RM0493 §12.8.27) —
 *        only the BLE stack set it before, so RTC/TAMP were dead without BLE.
 *   H5: RCC_APB3ENR.RTCAPBEN (bit 21).
 */
static inline void ll_rtc_bus_clk_enable(void)
{
#if defined(STM32L422xx)
    SET_BITS(REG32(RCC_BASE + 0x58UL), (1UL << 10));  /* APB1ENR1: RTCAPBEN */
    (void)REG32(RCC_BASE + 0x58UL);
#elif defined(STM32WBA55xx)
    SET_BITS(REG32(RCC_BASE + 0xA8UL), (1UL << 21));  /* APB7ENR: RTCAPBEN */
    (void)REG32(RCC_BASE + 0xA8UL);
#elif defined(STM32H523xx)
    SET_BITS(REG32(RCC_BASE + 0xA8UL), (1UL << 21));  /* APB3ENR: RTCAPBEN */
    (void)REG32(RCC_BASE + 0xA8UL);
#endif
}

#if defined(STM32L011xx)
/** L0: switch the LSE off if earlier firmware left it on. It lives in the RTC
 *  domain (only a POR or RTCRST clears it), and the old code here enabled it
 *  whenever it meant to enable the LSI. Needs DBP. */
static inline void ll_rtc_l0_lse_off(void)
{
    if (REG32(RCC_BASE + 0x50UL) & (1UL << 8)) {
        CLR_BITS(REG32(RCC_BASE + 0x50UL), (1UL << 8));   /* CSR: LSEON = 0 */
    }
}
#endif

/* ============================================================
 * RTC initialization
 * ============================================================ */

/** Enter RTC init mode (freezes calendar, allows register writes) */
static inline void ll_rtc_enter_init(void)
{
#if defined(STM32L011xx)
    SET_BITS(RTC_ISR, LL_RTC_ISR_INIT);
    while (!(RTC_ISR & LL_RTC_ISR_INITF))
        ;
#else
    SET_BITS(RTC_ICSR, LL_RTC_ISR_INIT);
    while (!(RTC_ICSR & LL_RTC_ISR_INITF))
        ;
#endif
}

/** Exit RTC init mode (resumes calendar) */
static inline void ll_rtc_exit_init(void)
{
#if defined(STM32L011xx)
    CLR_BITS(RTC_ISR, LL_RTC_ISR_INIT);
#else
    CLR_BITS(RTC_ICSR, LL_RTC_ISR_INIT);
#endif
}

/**
 * Wait for the calendar shadow registers to synchronize.
 * Call after ll_rtc_exit_init() before reading time/date.
 * Takes up to 2 RTCCLK cycles (~62 us with 32 kHz clock).
 */
static inline void ll_rtc_wait_sync(void)
{
#if defined(STM32L011xx)
    CLR_BITS(RTC_ISR, LL_RTC_ISR_RSF);
    while (!(RTC_ISR & LL_RTC_ISR_RSF))
        ;
#else
    CLR_BITS(RTC_ICSR, LL_RTC_ISR_RSF);
    while (!(RTC_ICSR & LL_RTC_ISR_RSF))
        ;
#endif
}

/**
 * Re-synchronize the calendar shadow registers, e.g. after waking from Stop
 * (RM0377 §22.4.8, RM0394 §35.3.10: "After waking up from low-power mode, RSF
 * must be cleared by software" and then set again before TR/DR are read —
 * the shadows don't update while the APB clock is off). Reads DR first to
 * release the lock a lone TR read leaves on it. RSF is write-protected, hence
 * the unlock. Bounded wait: returns even if the RTC clock is dead.
 */
static inline void ll_rtc_resync(void)
{
    (void)RTC_DR;
    ll_rtc_unlock();
#if defined(STM32L011xx)
    RTC_ISR = (~(LL_RTC_ISR_RSF | LL_RTC_ISR_INIT) & 0x0000FFFFUL)
            | (RTC_ISR & LL_RTC_ISR_INIT);
    for (uint32_t t = 100000UL; t && !(RTC_ISR & LL_RTC_ISR_RSF); t--)
        ;
#else
    CLR_BITS(RTC_ICSR, LL_RTC_ISR_RSF);
    for (uint32_t t = 100000UL; t && !(RTC_ICSR & LL_RTC_ISR_RSF); t--)
        ;
#endif
    ll_rtc_lock();
}

/** Synchronous prescaler (PREDIV_S) that gives a 1 Hz ck_spre from `use_lse`. */
static inline uint32_t ll_rtc_prediv_s(int use_lse)
{
    return use_lse ? 255UL : (LL_LSI_HZ / 128UL) - 1UL;
}

/**
 * Initialize the RTC with LSE or LSI.
 *   use_lse: 1 = use LSE (32.768kHz crystal), 0 = use LSI (LL_LSI_HZ)
 *   Core.ST.L0 has no LSE: it always uses LSI.
 *
 * Re-enters init mode, which restarts the prescaler; prefer ll_rtc_ensure()
 * when all you need is a running RTC.
 *
 * Prerequisites:
 *   - PWR clock enabled
 *   - Backup domain access enabled
 */
static inline void ll_rtc_init(int use_lse)
{
    /* Ensure backup domain access is enabled (PWR clock + DBP) */
    ll_rcc_pwr_clk_enable();
    ll_pwr_enable_backup_access();

#if defined(STM32L011xx)
    use_lse = 0;             /* no crystal on Core.ST.L0.1 — see ll_rcc_lse_enable() */
    ll_rtc_l0_lse_off();
#endif

    uint32_t desired_src = use_lse ? 1 : 2;

    if (use_lse) {
        ll_rcc_lse_enable();
        while (!ll_rcc_lse_ready())
            ;
    } else {
        ll_rcc_lsi_enable();
        while (!ll_rcc_lsi_ready())
            ;
    }

    /* RTCSEL is write-once per backup domain reset. If it's already set
     * to a different source (e.g. ROM bootloader set HSE/32), we must
     * reset the backup domain before we can select our clock. */
    {
#if defined(STM32L011xx)
        uint32_t bdcr = REG32(RCC_BASE + 0x50UL);
        uint32_t cur_sel = (bdcr >> 16) & 0x3;
#elif defined(STM32L422xx)
        uint32_t bdcr = REG32(RCC_BASE + 0x90UL);
        uint32_t cur_sel = (bdcr >> 8) & 0x3;
#elif defined(STM32WBA55xx)
        uint32_t bdcr = REG32(RCC_BASE + 0xF0UL);   /* BDCR1 */
        uint32_t cur_sel = (bdcr >> 8) & 0x3;
#elif defined(STM32H523xx)
        uint32_t bdcr = REG32(RCC_BASE + 0xF0UL);
        uint32_t cur_sel = (bdcr >> 8) & 0x3;
#endif
        if (cur_sel != 0 && cur_sel != desired_src) {
            /* Wrong source locked in — backup domain reset required */
#if defined(STM32L011xx)
            SET_BITS(REG32(RCC_BASE + 0x50UL), (1UL << 19));   /* CSR: RTCRST */
            for (volatile int i = 0; i < 100; i++) ;
            CLR_BITS(REG32(RCC_BASE + 0x50UL), (1UL << 19));
#elif defined(STM32L422xx)
            SET_BITS(REG32(RCC_BASE + 0x90UL), (1UL << 16));
            for (volatile int i = 0; i < 100; i++) ;
            CLR_BITS(REG32(RCC_BASE + 0x90UL), (1UL << 16));
#elif defined(STM32WBA55xx)
            SET_BITS(REG32(RCC_BASE + 0xF0UL), (1UL << 16));   /* BDCR1: BDRST */
            for (volatile int i = 0; i < 100; i++) ;
            CLR_BITS(REG32(RCC_BASE + 0xF0UL), (1UL << 16));
#elif defined(STM32H523xx)
            SET_BITS(REG32(RCC_BASE + 0xF0UL), (1UL << 16));
            for (volatile int i = 0; i < 100; i++) ;
            CLR_BITS(REG32(RCC_BASE + 0xF0UL), (1UL << 16));
#endif
        }
    }

    ll_rcc_rtc_set_source(desired_src);
    ll_rcc_rtc_enable();

#if defined(STM32H523xx)
    /* H5: enable the RTC APB interface clock (not enabled by default in run mode) */
    SET_BITS(REG32(RCC_BASE + 0xA8UL), (1UL << 21));  /* APB3ENR: RTCAPBEN */
    (void)REG32(RCC_BASE + 0xA8UL);  /* read-back fence */
#else
    ll_rtc_bus_clk_enable();
#endif

    ll_rtc_unlock();
    ll_rtc_enter_init();

    /* Set prescalers for 1Hz */
#if defined(STM32H523xx)
    RTC_PRER = (127UL << 16) | ll_rtc_prediv_s(use_lse);
#else
    /* RM0377 / RM0394 / RM0493: PRER takes two separate writes, synchronous
     * factor first. */
    RTC_PRER = ll_rtc_prediv_s(use_lse);
    RTC_PRER = (127UL << 16) | ll_rtc_prediv_s(use_lse);
#endif

    /* 24-hour format */
    CLR_BITS(RTC_CR, LL_RTC_CR_FMT);

    ll_rtc_exit_init();
#if !defined(STM32H523xx)
    /* "After an initialization, the software must wait until RSF is set
     * before reading" TR/DR (RM0377 §22.4.8, RM0394 §35.3.10); without it a
     * read right after a reset returns the shadow registers' reset value. */
    ll_rtc_wait_sync();
#endif
    ll_rtc_lock();
}

/**
 * Make sure the RTC kernel clock is selected, enabled and running, without
 * touching the calendar or prescalers. Selects LSI if nothing is selected yet,
 * and restarts the LSI oscillator when it is the source (LSION is cleared by
 * every system reset on the L0/L4 while RTCSEL survives). Needs PWR + DBP.
 */
static inline void ll_rtc_clock_ensure(void)
{
#if defined(STM32L011xx)
    ll_rtc_l0_lse_off();
#endif
    uint32_t src = ll_rcc_rtc_get_source();
    if (src == 0) {
        ll_rcc_rtc_set_source(2);
        src = 2;
    }
    if (src == 2) {
        ll_rcc_lsi_enable();
        while (!ll_rcc_lsi_ready())
            ;
    }
    ll_rcc_rtc_enable();
    ll_rtc_bus_clk_enable();
}

/**
 * Make sure the RTC is running with a 1 Hz calendar, without disturbing one
 * that already is. ll_rtc_init() restarts the prescaler on every call (and on
 * a clock-source mismatch resets the whole backup domain), so power code that
 * only needs "the RTC ticks" calls this instead. Keeps an LSE the application
 * selected (never on the L0); anything else ends up on LSI.
 */
static inline void ll_rtc_ensure(void)
{
    ll_rcc_pwr_clk_enable();
    ll_pwr_enable_backup_access();
#if defined(STM32L011xx)
    ll_rtc_l0_lse_off();
#endif

    uint32_t src = ll_rcc_rtc_get_source();
    int use_lse = -1;                       /* -1: needs a full init */
#if !defined(STM32L011xx)
    if (src == 1 && ll_rcc_lse_ready()) use_lse = 1;
#endif
    if (src == 2) {
        ll_rcc_lsi_enable();
        while (!ll_rcc_lsi_ready())
            ;
        use_lse = 0;
    }
    if (use_lse >= 0 && ll_rcc_rtc_enabled()) {
        ll_rtc_bus_clk_enable();
        if (RTC_PRER == ((127UL << 16) | ll_rtc_prediv_s(use_lse)))
            return;                         /* already running at 1 Hz */
    }
    ll_rtc_init(use_lse > 0 ? 1 : 0);
}

/** RTCCLK frequency in Hz for the selected source (LSE 32768, else LL_LSI_HZ). */
static inline uint32_t ll_rtc_clk_hz(void)
{
    return (ll_rcc_rtc_get_source() == 1) ? 32768UL : LL_LSI_HZ;
}

/* ============================================================
 * Time / Date
 * ============================================================ */

/** Set the time (24h format) */
static inline void ll_rtc_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    ll_rtc_unlock();
    ll_rtc_enter_init();

    RTC_TR = ((uint32_t)ll_bin_to_bcd(hours) << 16)
           | ((uint32_t)ll_bin_to_bcd(minutes) << 8)
           | ((uint32_t)ll_bin_to_bcd(seconds));

    ll_rtc_exit_init();
    ll_rtc_wait_sync();
    ll_rtc_lock();
}

/** Set the date */
static inline void ll_rtc_set_date(uint8_t year, uint8_t month,
                                   uint8_t day, uint8_t weekday)
{
    ll_rtc_unlock();
    ll_rtc_enter_init();

    RTC_DR = ((uint32_t)ll_bin_to_bcd(year) << 16)
           | ((uint32_t)weekday << 13)
           | ((uint32_t)ll_bin_to_bcd(month) << 8)
           | ((uint32_t)ll_bin_to_bcd(day));

    ll_rtc_exit_init();
    ll_rtc_wait_sync();
    ll_rtc_lock();
}

/** Read the current time */
static inline void ll_rtc_get_time(uint8_t *hours, uint8_t *minutes, uint8_t *seconds)
{
    uint32_t tr = RTC_TR;
#if !defined(STM32H523xx)
    /* Reading TR locks the DR shadow until DR is read (RM0377 §22.4.8,
     * RM0394 §35.3.10); read it so a later get_date isn't stale. */
    (void)RTC_DR;
#endif
    *hours   = ll_bcd_to_bin((tr >> 16) & 0x3F);
    *minutes = ll_bcd_to_bin((tr >> 8) & 0x7F);
    *seconds = ll_bcd_to_bin(tr & 0x7F);
}

/** Read the current date */
static inline void ll_rtc_get_date(uint8_t *year, uint8_t *month,
                                   uint8_t *day, uint8_t *weekday)
{
    uint32_t dr = RTC_DR;
    *year    = ll_bcd_to_bin((dr >> 16) & 0xFF);
    *month   = ll_bcd_to_bin((dr >> 8) & 0x1F);
    *day     = ll_bcd_to_bin(dr & 0x3F);
    *weekday = (dr >> 13) & 0x07;
}

/**
 * Time of day in ms (0..86399999), with the sub-second from RTC_SSR (BCD
 * mode, 24 h format — what ll_rtc_init() sets up). After Stop, call
 * ll_rtc_resync() first: the shadow registers are stale until RSF sets again.
 * Reading SSR locks TR/DR until DR is read, so the three are one instant
 * (RM0377 §22.4.8, RM0394 §35.3.10, RM0493 §36.3.12).
 */
static inline uint32_t ll_rtc_ms_of_day(void)
{
    uint32_t ssr = RTC_SSR & 0xFFFFUL;
    uint32_t tr  = RTC_TR;
    (void)RTC_DR;
    uint32_t ps  = RTC_PRER & 0x7FFFUL;
    if (ssr > ps) ssr = ps;              /* only after a shift operation */
    uint32_t s = (uint32_t)ll_bcd_to_bin((uint8_t)((tr >> 16) & 0x3FUL)) * 3600UL
               + (uint32_t)ll_bcd_to_bin((uint8_t)((tr >> 8) & 0x7FUL)) * 60UL
               + (uint32_t)ll_bcd_to_bin((uint8_t)(tr & 0x7FUL));
    return s * 1000UL + ((ps - ssr) * 1000UL) / (ps + 1UL);
}

/* ============================================================
 * Wakeup timer
 * ============================================================ */

/** Returns 1 if the wakeup-timer flag (WUTF) is set. */
static inline int ll_rtc_wakeup_flag(void)
{
#if defined(STM32L011xx)
    return (RTC_ISR & LL_RTC_ISR_WUTF) != 0;
#else
    return (RTC_SR & LL_RTC_SR_WUTF) != 0;
#endif
}

/** Clear the wakeup timer flag */
static inline void ll_rtc_wakeup_clear_flag(void)
{
#if defined(STM32L011xx)
    /* ISR flags are rc_w0: write 0 to WUTF, 1 to the other flags (no effect),
     * and keep INIT. A read-modify-write would also clear any flag that got
     * set between the read and the write. */
    RTC_ISR = (~(LL_RTC_ISR_WUTF | LL_RTC_ISR_INIT) & 0x0000FFFFUL)
            | (RTC_ISR & LL_RTC_ISR_INIT);
#else
    /* SCR at offset 0x5C, CWUTF at bit 2 (RM0394 §35.6.20, RM0493 §36.6.23,
     * RM0481). The L4/WBA used to write 0x34, which is RTC_TSDR — so the flag
     * was never cleared and the next Stop/Standby had no wake edge. */
    RTC_SCR = LL_RTC_SR_WUTF;
#endif
}

/**
 * Configure the RTC wakeup timer from a raw clock selection and reload.
 *   wucksel: LL_RTC_WUCKSEL_* (DIV16 ticks RTCCLK/16; 1HZ ticks ck_spre)
 *   wut:     reload, 0..0xFFFF — WUTF sets every (wut + 1) ck_wut ticks
 * Enables the wakeup interrupt (WUTIE) and clears a stale WUTF first.
 */
static inline void ll_rtc_wakeup_config_raw(uint32_t wucksel, uint32_t wut)
{
    ll_rtc_unlock();

    /* Disable wakeup timer */
    CLR_BITS(RTC_CR, LL_RTC_CR_WUTE);

    /* Wait for wakeup timer write flag */
#if defined(STM32L011xx)
    while (!(RTC_ISR & LL_RTC_ISR_WUTWF))
        ;
#else
    while (!(RTC_ICSR & LL_RTC_ISR_WUTWF))
        ;
#endif

#if !defined(STM32H523xx)
    /* A WUTF left over from an earlier run must be clear before the next
     * Stop/Standby entry (RM0377 Table 39/40, RM0394, RM0493 Table 90/92). */
    ll_rtc_wakeup_clear_flag();
#endif

    /* Set wakeup clock and reload value. On the newer RTC the upper half of
     * WUTR is WUTOCLR (auto-clear); keep it 0 so WUTF is cleared by software. */
    MOD_BITS(RTC_CR, 0x7UL, wucksel & 0x7UL);
    RTC_WUTR = wut & 0xFFFFUL;

#if defined(STM32H523xx)
    /* H5: WUTE must be set during init mode for reliable timer start.
     * Setting WUTE outside init mode doesn't propagate to the RTC domain
     * (WUTWF stays 1, counter never starts). Enter init briefly to set WUTE. */
    ll_rtc_enter_init();
    SET_BITS(RTC_CR, LL_RTC_CR_WUTE | LL_RTC_CR_WUTIE);
    ll_rtc_exit_init();
#else
    /* Enable wakeup timer and its interrupt. Never via init mode: leaving it
     * reloads RTC_SSR with PREDIV_S (RM0493 §36.3.11), which throws away the
     * current second's fraction. The WBA used to take the H5 path above, and
     * core_stop_for(10), arming 2.5 s chunks half a second into a calendar
     * second, lost 0.5 s per chunk: 8 s on the calendar for a 10 s sleep.
     * Bench, 2026-09-25: WUTE set here starts the counter on the WBA55. */
    SET_BITS(RTC_CR, LL_RTC_CR_WUTE | LL_RTC_CR_WUTIE);
#endif

    ll_rtc_lock();
}

/**
 * Configure the RTC wakeup timer.
 *   seconds: wakeup interval in seconds, 1-65536 with the 1Hz clock
 *            (0 is treated as 1; larger values are clamped to 65536).
 *
 * Uses WUCKSEL = 1Hz (CK_SPRE) for simple second-based intervals.
 * Wakes Stop through EXTI line 20 on L0/L4, directly (vector 2, no EXTI line)
 * on WBA, and EXTI line 17 on H5.
 */
static inline void ll_rtc_wakeup_config(uint32_t seconds)
{
    if (seconds == 0) seconds = 1;
    if (seconds > 65536UL) seconds = 65536UL;
    ll_rtc_wakeup_config_raw(LL_RTC_WUCKSEL_1HZ, seconds - 1UL);
}

/** Disable the wakeup timer */
static inline void ll_rtc_wakeup_disable(void)
{
    ll_rtc_unlock();
    CLR_BITS(RTC_CR, LL_RTC_CR_WUTE | LL_RTC_CR_WUTIE);
    ll_rtc_lock();
}

/* ============================================================
 * Alarm A
 *
 * Alarm A matches on a configurable subset of calendar fields
 * (hours, minutes, seconds). Set MSKx bits to ignore a field.
 * ============================================================ */

/* Alarm A register offset differs by RTC generation:
 *   L0 (older RTC): 0x1C (RM0377 §22.7.7)
 *   L4/WBA/H5 (newer RTC): 0x40 (RM0394 §35.6.14, RM0493 §36.6.16,
 *   RM0481 §46.6.16). On the L422 0x1C is reserved and on the WBA it is
 *   RTC_PRIVCFGR — which is why ALRMAR "read back 0" on the L4. */
#if defined(STM32L011xx)
  #define RTC_ALRMAR        REG32(RTC_BASE + 0x1CUL)
  #define RTC_ALRMASSR      REG32(RTC_BASE + 0x44UL)
#else
  #define RTC_ALRMAR        REG32(RTC_BASE + 0x40UL)
  #define RTC_ALRMASSR      REG32(RTC_BASE + 0x44UL)
#endif

/* ALRMAR field masks */
#define LL_RTC_ALRM_MSK4   (1UL << 31)  /* Mask day/date (ignore) */
#define LL_RTC_ALRM_MSK3   (1UL << 23)  /* Mask hours (ignore) */
#define LL_RTC_ALRM_MSK2   (1UL << 15)  /* Mask minutes (ignore) */
#define LL_RTC_ALRM_MSK1   (1UL << 7)   /* Mask seconds (ignore) */

/** Clear the Alarm A flag. */
static inline void ll_rtc_alarm_a_clear_flag(void)
{
#if defined(STM32L011xx)
    /* ALRAF is rc_w0 in ISR; same no-RMW pattern as the wakeup flag. */
    RTC_ISR = (~(LL_RTC_ISR_ALRAF | LL_RTC_ISR_INIT) & 0x0000FFFFUL)
            | (RTC_ISR & LL_RTC_ISR_INIT);
#else
    /* SCR at 0x5C, CALRAF at bit 0. (The L4 wrote ICSR bit 8 — reserved on
     * the L422 — and the WBA wrote 0x34, RTC_TSDR.) */
    RTC_SCR = LL_RTC_SR_ALRAF;
#endif
}

/** Check if Alarm A flag is set. */
static inline int ll_rtc_alarm_a_flag(void)
{
#if defined(STM32L011xx)
    return (RTC_ISR & LL_RTC_ISR_ALRAF) != 0;   /* ALRAF in ISR */
#else
    /* SR at offset 0x50, ALRAF at bit 0 (RM0394 §35.6.18, RM0493 §36.6.20,
     * RM0481 §46.6.22). The WBA read 0x30 (RTC_TSTR). */
    return (RTC_SR & LL_RTC_SR_ALRAF) != 0;
#endif
}

/**
 * Set Alarm A to trigger at a specific time (24h format).
 * Set hours/minutes/seconds to the desired alarm time.
 * Fields with 0xFF are masked (ignored for matching).
 *
 * Example: alarm at 08:30:00 every day:
 *   ll_rtc_alarm_a_set(8, 30, 0);
 *
 * Example: alarm every hour at XX:30:00:
 *   ll_rtc_alarm_a_set(0xFF, 30, 0);
 */
static inline void ll_rtc_alarm_a_set(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    ll_rtc_unlock();

    /* Disable Alarm A */
    CLR_BITS(RTC_CR, LL_RTC_CR_ALRAE);

    /* Wait for alarm write allowed */
#if defined(STM32L011xx)
    while (!(RTC_ISR & LL_RTC_ISR_ALRAWF))
        ;
#elif defined(STM32H523xx)
    /* H5: no ALRAWF — write is immediately effective after ALRAE=0 */
    for (volatile int i = 0; i < 10; i++) ;
#else
    /* L4/WBA: no ALRAWF on this RTC either (ICSR bits 1:0 are reserved; they
     * reset to 1, so the old poll only hung once the RTC bus clock was off).
     * ALRMAR is writable once ALRAE = 0 (RM0394 §35.6.14, RM0493 §36.6.16);
     * the write loop below absorbs the ~2 RTCCLK the CR change takes. */
#endif

    /* Build alarm register value */
    uint32_t alrm = LL_RTC_ALRM_MSK4;  /* Always mask day/date */

    if (hours == 0xFF)   alrm |= LL_RTC_ALRM_MSK3;
    else                 alrm |= ((uint32_t)ll_bin_to_bcd(hours) << 16);

    if (minutes == 0xFF) alrm |= LL_RTC_ALRM_MSK2;
    else                 alrm |= ((uint32_t)ll_bin_to_bcd(minutes) << 8);

    if (seconds == 0xFF) alrm |= LL_RTC_ALRM_MSK1;
    else                 alrm |= (uint32_t)ll_bin_to_bcd(seconds);

#if defined(STM32L422xx) || defined(STM32WBA55xx)
    for (uint32_t t = 100000UL; t; t--) {
        RTC_ALRMAR = alrm;
        if (RTC_ALRMAR == alrm) break;
    }
#else
    RTC_ALRMAR = alrm;
#endif

#if !defined(STM32H523xx)
    /* Drop a flag left by an earlier match so alarm_fired() reports this one. */
    ll_rtc_alarm_a_clear_flag();
#endif

    /* Enable Alarm A + interrupt */
    SET_BITS(RTC_CR, LL_RTC_CR_ALRAE | LL_RTC_CR_ALRAIE);

    ll_rtc_lock();
}

/** Disable Alarm A. */
static inline void ll_rtc_alarm_a_disable(void)
{
    ll_rtc_unlock();
    CLR_BITS(RTC_CR, LL_RTC_CR_ALRAE | LL_RTC_CR_ALRAIE);
    ll_rtc_lock();
}

/* ============================================================
 * Backup registers
 *
 * Needs the RTC/TAMP bus clock (ll_rtc_bus_clk_enable; on the L0 the RTC
 * clock itself). Writing also needs PWR clock + backup domain access:
 *   ll_rcc_pwr_clk_enable();
 *   ll_pwr_enable_backup_access();
 * core_backup.h does all of that.
 * ============================================================ */

/* Backup register base varies by family:
 *   L0:        RTC_BKP0R..4R inside the RTC at RTC_BASE + 0x50 (RM0377 §22.7.20)
 *   L4 (L422): TAMP_BKP0R..31R at TAMP_BASE + 0x100 (RM0394 §36.6.8). RTC+0x50
 *              is RTC_SR on this part, which is why BKP writes "read back 0".
 *   WBA/H5:    TAMP_BKP0R..31R at TAMP_BASE + 0x100 (RM0493 §37.6.18) */
#if defined(STM32L422xx)
  #define TAMP_BASE           0x40003400UL    /* RM0394 memory map */
  #define RTC_BKP_BASE        (TAMP_BASE + 0x100UL)
#elif defined(STM32WBA55xx)
  #define TAMP_BASE           0x46007C00UL
  #define RTC_BKP_BASE        (TAMP_BASE + 0x100UL)
#elif defined(STM32H523xx)
  #define TAMP_BASE           0x44007C00UL
  #define RTC_BKP_BASE        (TAMP_BASE + 0x100UL)
#else
  #define RTC_BKP_BASE        (RTC_BASE + 0x50UL)
#endif

/** Read backup register n (0–31; 0–4 on the L0). Needs the RTC/TAMP bus clock. */
static inline uint32_t ll_rtc_bkp_read(uint32_t n)
{
    return REG32(RTC_BKP_BASE + n * 4);
}

/** Write backup register n (0–31; 0–4 on the L0). Requires backup domain access. */
static inline void ll_rtc_bkp_write(uint32_t n, uint32_t val)
{
    REG32(RTC_BKP_BASE + n * 4) = val;
}

#endif /* LL_RTC_H */
