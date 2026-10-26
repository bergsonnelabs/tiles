/**
 * port_core_recovery.h — STM32 port: the bodies of core_recovery.h
 *
 * Included at the end of core_recovery.h, after its prototypes; not for
 * direct use. The strike count lives in hal_dfu's reserved-SRAM words; the
 * reset cause comes from core_st_reset_flags() (core_reset.h).
 */

#ifndef PORT_CORE_RECOVERY_H
#define PORT_CORE_RECOVERY_H

#include "hal_dfu.h"
#include "core_reset.h"

#ifdef DFU_STRIKE_TAG_ADDR

static inline uint32_t core_st_rom_dfu_note_boot(void)
{
    uint32_t strikes = hal_recovery_strikes();
    /* IWDGRSTF as core_reset_capture() read it at the top of core_init(),
     * which has already cleared the flags (RMVF, else the flag latches and
     * miscounts). */
    int wd = (core_st_reset_flags() & CORE_ST_RESET_FLAG_IWDG) != 0;
    hal_recovery_stash_cause(wd);
    strikes = wd ? (strikes + 1u) : 0u;  /* any non-watchdog reset = fresh start */
    hal_recovery_set_strikes(strikes);
    return strikes;
}

static inline int core_st_rom_dfu_over_limit(uint32_t strikes)
{
    return strikes >= CORE_RECOVERY_STRIKE_LIMIT;
}

static inline void core_st_rom_dfu_clear_strikes(void)
{
    hal_recovery_set_strikes(0);
}

#endif /* DFU_STRIKE_TAG_ADDR */

#endif /* PORT_CORE_RECOVERY_H */
