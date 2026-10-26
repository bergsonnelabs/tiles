/**
 * core_rtc.h — Real-time clock
 *
 * @studio category rtc label=Core.RTC icon=◴
 *
 * @studio coverage
 *   id:    rtc
 *   name:  RTC — real-time clock
 *   blurb: Wall-clock time + date, periodic wakeup timer, and Alarm A
 *          (calendar-based with per-field wildcards). Tier 2 covers
 *          init / set_time / set_date / wakeup(_stop) / set_alarm /
 *          clear_alarm / alarm_fired. Defaults to LSI (internal RC,
 *          ~32 kHz; ~37 kHz on Core.ST.L0) — for LSE precision callers
 *          drop into ll_rtc_init(1) from C (not on Core.ST.L0, which has
 *          no crystal).
 */

#ifndef CORE_RTC_H
#define CORE_RTC_H

#include "core_types.h"   /* CORE_INLINE, uint8_t, uint32_t */

/* ============================================================
 * Init
 * ============================================================ */

/**
 * Initialize the RTC using LSI (internal RC, ~32 kHz; ~37 kHz on Core.ST.L0,
 * scaled so a second is still ~1 s). Call once from `on start` before setting
 * time, date, or alarms. For LSE, call ll_rtc_init(1) directly in
 * hand-written C (Core.ST.L0 has no LSE and stays on LSI).
 *
 * @studio expose category=rtc name=init
 * @studio twin full
 */
CORE_INLINE void core_rtc_init(void);

/* ============================================================
 * Time
 * ============================================================ */

/**
 * Set the time (24h format).
 *
 * @studio expose category=rtc name=set_time
 * @studio twin full
 * @param h [0..23] Hours.
 * @param m [0..59] Minutes.
 * @param s [0..59] Seconds.
 */
CORE_INLINE void core_rtc_set_time(uint8_t h, uint8_t m, uint8_t s);

/** Read the current time. */
CORE_INLINE void core_rtc_get_time(uint8_t *h, uint8_t *m, uint8_t *s);

/* ============================================================
 * Date
 * ============================================================ */

/**
 * Set the date.
 *
 * @studio expose category=rtc name=set_date
 * @studio twin full
 * @param y [0..99] 2-digit year (2025 = 25).
 * @param mo [1..12] Month.
 * @param d [1..31] Day of month.
 * @param wd [1..7] Weekday (1 = Mon, 7 = Sun).
 */
CORE_INLINE void core_rtc_set_date(uint8_t y, uint8_t mo, uint8_t d,
                                    uint8_t wd);

/** Read the current date. */
CORE_INLINE void core_rtc_get_date(uint8_t *y, uint8_t *mo, uint8_t *d,
                                    uint8_t *wd);

/* ============================================================
 * Periodic wakeup timer
 * ============================================================ */

/**
 * Configure a periodic wakeup timer.
 *
 * @studio expose category=rtc name=wakeup
 * @studio twin full
 * @param seconds [1..65535] Wakeup interval in seconds.
 */
CORE_INLINE void core_rtc_wakeup(uint32_t seconds);

/**
 * Disable the periodic wakeup timer.
 *
 * @studio expose category=rtc name=wakeup_stop
 * @studio twin full
 */
CORE_INLINE void core_rtc_wakeup_stop(void);

/* Backward compat */
#define core_rtc_alarm       core_rtc_wakeup
#define core_rtc_alarm_stop  core_rtc_wakeup_stop

/* ============================================================
 * Alarm A — calendar-based alarm
 *
 * Triggers when the RTC time matches the configured fields.
 * Pass 0xFF for any field to ignore it (wildcard).
 *
 * Usage:
 *   core_rtc_set_alarm(8, 30, 0);      // every day at 08:30:00
 *   core_rtc_set_alarm(0xFF, 0, 0);    // every hour at XX:00:00
 *   core_rtc_set_alarm(0xFF, 0xFF, 0); // every minute at XX:XX:00
 * ============================================================ */

/**
 * Set Alarm A to trigger at a specific time.
 * Pass 0xFF for hours, minutes, or seconds to ignore that field.
 * The alarm fires when all non-masked fields match the RTC time.
 *
 * @studio expose category=rtc name=set_alarm
 * @studio twin full
 * @param hours   [0..255] 0-23, or 255 (0xFF) to match any hour.
 * @param minutes [0..255] 0-59, or 255 (0xFF) to match any minute.
 * @param seconds [0..255] 0-59, or 255 (0xFF) to match any second.
 */
CORE_INLINE void core_rtc_set_alarm(uint8_t hours, uint8_t minutes,
                                     uint8_t seconds);

/**
 * Clear the alarm (disable Alarm A).
 *
 * @studio expose category=rtc name=clear_alarm
 * @studio twin full
 */
CORE_INLINE void core_rtc_clear_alarm(void);

/**
 * Check if the alarm has fired (poll mode). Clears the flag on read.
 *
 * @studio expose category=rtc name=alarm_fired returns=bool
 * @studio twin full
 */
CORE_INLINE int core_rtc_alarm_fired(void);

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=M title="No DSL get_time / get_date"
//   The C surface has core_rtc_get_time / get_date taking out-pointers;
//   they're not exposed because the host-call ABI doesn't map cleanly
//   to multi-out values. The newer multi-scalar-out ABI (see DSL
//   Capability Coverage close, 2026-05-03) could land them.
//
// @studio unsupported tier=1 value=M title="LSE / external crystal not exposed"
//   core_rtc_init always selects LSI (~32 kHz RC, ±5% typical drift on the
//   L4/WBA). The L0's LSI is 26-56 kHz part to part; it is measured once
//   against SYSCLK (ll_lsi_hz(), sdk/hal/hal_lsi.c) and the RTC prescaler
//   is set from that, so the second is as good as SYSCLK at that moment
//   (HSI16 at high / max, MSI at low / medium) and drifts with temperature.
//   Cores with an LSE crystal need ll_rtc_init(1) — no Tier 2 wrapper
//   that takes a clock-source argument.
//
// @studio unsupported tier=1 value=L title="No subsecond / millisecond accuracy"
//   The wrapper exposes whole-second resolution. STM32 RTCs have a
//   subsecond register (1/PREDIV_S) that's reachable from ll_rtc but
//   not from this header — wall-clock timestamps round to the second.

/* The port's bodies of the functions above. */
#include "port_core_rtc.h"

#endif /* CORE_RTC_H */
