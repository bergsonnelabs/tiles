/**
 * port_core_watchdog.h — STM32 port: the bodies of core_watchdog.h
 *
 * Included at the end of core_watchdog.h, after its prototypes; not for
 * direct use. The IWDG over ll_iwdg (prescaler and reload from ll_lsi_hz()),
 * the RCC reset-flag clear, the ROM-DFU strike-counter auto-clear (hal_dfu's
 * reserved-SRAM words, uptime from ll_systick) and the DBGMCU freeze
 * register of each MCU.
 */

#ifndef PORT_CORE_WATCHDOG_H
#define PORT_CORE_WATCHDOG_H

#include "ll_iwdg.h"
#include "ll_systick.h" /* _systick_ticks: uptime for the strike-counter auto-clear */
#include "hal_dfu.h"  /* brick-recovery strike counter (core_watchdog_feed) */
#include "core_reset.h"  /* core_reset_reason(), core_reset_capture() */

static inline void core_watchdog_start(uint32_t timeout_ms)
{
    /* LSI = ll_lsi_hz(): 32 kHz nominal; on the L0 measured once at the
       first call (it is untrimmed, 26-56 kHz), so a 5 s timeout is 5 s rather
       than anything from 3.3 to 7.1 s. Pick the smallest prescaler whose
       12-bit reload (1..4096 ticks) fits.
       tick_ms = psc * 1000 / LSI  →  reload = timeout_ms * LSI / (psc * 1000).
       In Hz, not kHz: a measured 36.99 kHz as "36" would be 2.7 % short. */
    static const uint32_t psc_vals[] = { 4, 8, 16, 32, 64, 128, 256 };
    static const uint32_t psc_regs[] = {
        LL_IWDG_PSC_4, LL_IWDG_PSC_8, LL_IWDG_PSC_16, LL_IWDG_PSC_32,
        LL_IWDG_PSC_64, LL_IWDG_PSC_128, LL_IWDG_PSC_256
    };
    const uint32_t lsi_hz = ll_lsi_hz();

    /* The longest the hardware can do is 4096 x 256 ticks (~40 s at the L0's
     * slowest LSI), and 60 s x 70 kHz still fits in 32 bits. */
    if (timeout_ms > 60000UL) timeout_ms = 60000UL;

    for (int i = 0; i < 7; i++) {
        /* (The earlier form divided by an extra 1000, making every timeout
         *  ~1000x too short.) */
        uint32_t reload = (timeout_ms * lsi_hz) / (psc_vals[i] * 1000UL);
        if (reload == 0) reload = 1;
        if (reload <= 4096) {
            ll_iwdg_init(psc_regs[i], reload - 1);
            _core_watchdog_timeout_ms = (reload * psc_vals[i] * 1000UL) / lsi_hz;
            if (_core_watchdog_timeout_ms == 0) _core_watchdog_timeout_ms = 1;
            return;
        }
    }
    /* Fallback: max timeout (4096 x 256 LSI ticks: ~32.8 s at 32 kHz,
     * ~28.3 s at the L0's nominal 37 kHz) */
    ll_iwdg_init(LL_IWDG_PSC_256, 4095);
    _core_watchdog_timeout_ms = (4096UL * 256UL * 1000UL) / lsi_hz;
}

static inline int core_watchdog_running(void)
{
    return _core_watchdog_timeout_ms != 0;
}

static inline uint32_t core_watchdog_sleep_chunk_ms(void)
{
    uint32_t t = _core_watchdog_timeout_ms;
    if (t == 0) return 0;
    return (t >= 2UL) ? t / 2UL : 1UL;
}

static inline void core_watchdog_feed(void)
{
    ll_iwdg_refresh();
#if defined(THREAD_ENABLED)
    /* Thread builds: a loop that feeds keeps Thread serviced even if it never
     * delays (at most one sequencer pass every 5 ms; core_thread.c). */
    core_thread_service_tick();
#endif
#if defined(DFU_STRIKE_TAG_ADDR) && defined(ROM_DFU)
    /* (ROM_DFU only: that is where core_init() keeps the count, and where the
     * linker reserves these SRAM words; elsewhere they are the stack top.)
     * Brick recovery (core_recovery.h): core_init() counts consecutive
     * watchdog resets and parks the Core in the ROM bootloader at the limit.
     * Nothing used to zero the count on a healthy run, so three unrelated
     * watchdog resets without a power cycle parked a working board. An app
     * that is still feeding after max(10 s, 2 x timeout) of uptime is healthy:
     * zero it here, once. While the count is 0 (every boot not preceded by a
     * watchdog reset, and every feed after the clear) this costs one SRAM load
     * and a compare. Uptime includes time in core_stop_for(). */
    if (DFU_STRIKE_ADDR != 0UL && hal_recovery_valid()) {
        uint32_t healthy_ms = 2UL * _core_watchdog_timeout_ms;
        if (healthy_ms < 10000UL) healthy_ms = 10000UL;
        if (_systick_ticks >= healthy_ms) {
            hal_recovery_set_strikes(0);
        }
    }
#endif
}

static inline int core_watchdog_caused_reset(void)
{
    return core_reset_reason() == CORE_RESET_WATCHDOG;
}

static inline void core_watchdog_clear_flags(void)
{
    core_reset_capture();          /* keep this boot's cause before clearing */
    ll_rcc_clear_reset_flags();
}

static inline void core_watchdog_debug_freeze(void)
{
    /* DBG_IWDG_STOP is bit 12 everywhere; only the register moves. Note the two
     * Cortex-M33 parts do NOT put DBGMCU at 0xE0042000 — that is the CoreSight
     * CTI block on both (confirmed on a WBA55 by walking its ROM table). */
#if defined(STM32L011xx)
    /* RM0377 27.9.4: DBG_APB1_FZ is mapped at 0x40015808. */
    SET_BITS(REG32(0x40015808UL), (1UL << 12));
#elif defined(STM32L422xx)
    /* RM0394: DBGMCU_APB1FZR1 at 0xE0042008. */
    SET_BITS(REG32(0xE0042008UL), (1UL << 12));
#elif defined(STM32WBA55xx)
    /* RM0493 43.12.7: DBGMCU @ 0xE0044000, DBGMCU_APB1LFZR at offset 0x08.
     * The CPU reaches DBGMCU at that same address (§43.12.1, Table 425), except
     * while a debugger is connected to AP0, when a CPU access bus-faults.
     * Verified on hardware: firmware writes bit 12 and reads it back, and a
     * 6 s halt under the CoreProbe with a 2 s IWDG no longer resets the part. */
    SET_BITS(REG32(0xE0044008UL), (1UL << 12));
#elif defined(STM32H523xx)
    /* RM0481 59.12.4: DBGMCU is at 0xE00E4000 for the DEBUGGER and 0x44024000
     * for SOFTWARE; DBGMCU_APB1LFZR is at offset 0x08. Firmware must use the
     * software base. (This previously wrote 0xE004203C — the CTI block — so the
     * freeze never took effect on the H5.) */
    SET_BITS(REG32(0x44024008UL), (1UL << 12));
#endif
}

#endif /* PORT_CORE_WATCHDOG_H */
