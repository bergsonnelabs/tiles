/**
 * core_backup.h — Backup register access
 *
 * 32-bit registers that persist through resets and Standby mode.
 * Useful for crash counters, boot flags, and state preservation.
 *
 * Count: Core.ST.L0 has 5 registers (0–4, inside the RTC), all others have
 * 32 (0–31, in the TAMP block). Retained as long as VBAT or VDD is present.
 *
 * Usage:
 *   core_backup_write(0, 0xDEADBEEF);   // store a value
 *   uint32_t val = core_backup_read(0);  // read it back
 *
 * No init required — the RTC clock domain is enabled automatically.
 *
 * @studio category backup label=Core.Backup icon=◫
 *
 * @studio coverage
 *   id:    backup
 *   name:  Backup — persistent registers
 *   blurb: 32-bit registers retained through reset and Standby (and across
 *          power loss when VBAT is wired). Tier 2 read/write helpers take
 *          an index and resolve the underlying RTC/TAMP block per Core
 *          family — Core.ST.L0 exposes 5 registers, others expose 32. Useful
 *          for boot counters, sticky flags, and small scraps of state.
 */

#ifndef CORE_BACKUP_H
#define CORE_BACKUP_H

#include "core_types.h"   /* CORE_INLINE, uint8_t, uint32_t */

/* CORE_BACKUP_COUNT: the number of backup registers on this Core (5 on
 * Core.ST.L0, 32 on the others). The port defines it (port_core_backup.h). */

/**
 * Ensure the backup registers are reachable.
 *   L0: RTC_BKPxR live inside the RTC (RM0377 §22.7.20), so the RTC clock
 *       must be selected, enabled and running (LSI; never the LSE here).
 *   L4 (L422): TAMP_BKPxR (RM0394 §36.6.8) on the RTC APB clock; the RTC
 *       kernel clock is brought up too, as before, in case nothing else has.
 *   WBA: TAMP_BKPxR (RM0493 §37.6.18) need RCC_APB7ENR.RTCAPBEN, which is
 *       off at reset — without it TAMP read 0 and ignored writes unless the
 *       BLE stack happened to have set it.
 *   H5: TAMP on RCC_APB3ENR.RTCAPBEN.
 * All of them need PWR + DBP for writes.
 */
CORE_INLINE void _core_backup_ensure_clk(void);

/**
 * Read a backup register.
 *
 * @studio expose category=backup name=read returns=int
 * @studio twin full
 * @param index [0..31] Register index (Core.ST.L0 caps at 4; others at 31).
 */
CORE_INLINE uint32_t core_backup_read(uint8_t index);

/**
 * Write a backup register. Backup domain write access is enabled automatically.
 *
 * @studio expose category=backup name=write
 * @studio twin full
 * @param index [0..31] Register index (Core.ST.L0 caps at 4; others at 31).
 * @param value 32-bit value to store.
 */
CORE_INLINE void core_backup_write(uint8_t index, uint32_t value);

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=L title="Twin backup state wipes on Reset"
//   read/write round-trip within a single run, but the per-slot store
//   is cleared on every project reload — DSL programs that rely on
//   backup state surviving a soft reset (e.g., boot counters across
//   the user clicking Reset in the IDE) can't be exercised end-to-end.
//
// @studio unsupported tier=1 value=L title="No tamper / anti-tamper integration"
//   The TAMP block on WBA/H5 hosts the backup registers but also drives
//   tamper detection (active edge, anti-tamper erase, time-stamping on
//   tamper). None of that is wrapped — backup is read/write only.

/* The port's bodies of the functions above. */
#include "port_core_backup.h"

#endif /* CORE_BACKUP_H */
