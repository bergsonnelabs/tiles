/**
 * port_core_backup.h — STM32 port: the bodies of core_backup.h
 *
 * Included at the end of core_backup.h, after its prototypes; not for direct
 * use. CORE_BACKUP_COUNT per MCU, and the RTC / TAMP backup registers over
 * ll_rtc, with the clock and backup-domain access each MCU needs (ll_pwr,
 * ll_rcc).
 */

#ifndef PORT_CORE_BACKUP_H
#define PORT_CORE_BACKUP_H

#include "ll_rtc.h"
#include "ll_pwr.h"
#include "ll_rcc.h"

/* Number of backup registers per core */
#if defined(STM32L011xx)
  #define CORE_BACKUP_COUNT  5
#else
  #define CORE_BACKUP_COUNT  32
#endif

static inline void _core_backup_ensure_clk(void)
{
    ll_rcc_pwr_clk_enable();
    ll_pwr_enable_backup_access();

#if defined(STM32L011xx) || defined(STM32L422xx)
    ll_rtc_clock_ensure();               /* includes the bus clock on the L4 */
#elif defined(STM32WBA55xx)
    ll_rtc_bus_clk_enable();             /* APB7ENR: RTCAPBEN */
#elif defined(STM32H523xx)
    SET_BITS(REG32(RCC_BASE + 0xA8UL), (1UL << 21));  /* APB3ENR: RTCAPBEN */
#endif
}

static inline uint32_t core_backup_read(uint8_t index)
{
    if (index >= CORE_BACKUP_COUNT) return 0;
    _core_backup_ensure_clk();
    return ll_rtc_bkp_read(index);
}

static inline void core_backup_write(uint8_t index, uint32_t value)
{
    if (index >= CORE_BACKUP_COUNT) return;
    _core_backup_ensure_clk();
    ll_rtc_bkp_write(index, value);
}

#endif /* PORT_CORE_BACKUP_H */
