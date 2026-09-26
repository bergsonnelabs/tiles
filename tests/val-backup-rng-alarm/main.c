/**
 * val-backup-rng-alarm — Validation: backup registers, RNG, RTC alarm
 *
 * Core.ST.H5, clock=default (64 MHz)
 *
 * Exercises: core_backup_read/write, CORE_BACKUP_COUNT,
 *            core_rng_init/read/fill/error/deinit,
 *            core_rtc_init, core_rtc_set_alarm, core_rtc_alarm_fired,
 *            core_rtc_clear_alarm, core_rtc_set_time, core_rtc_get_time
 */

#include "core.h"
#include "core_backup.h"
#include "core_rng.h"
#include "core_rtc.h"

static volatile int alarm_checked;

int main(void)
{
    core_init();

    /* ---- Backup registers ---- */
    core_backup_write(0, 0x12345678);
    uint32_t v = core_backup_read(0);
    (void)v;

    core_backup_write(CORE_BACKUP_COUNT - 1, 0xABCD);
    v = core_backup_read(CORE_BACKUP_COUNT - 1);
    (void)v;

    /* Out-of-range should return 0 */
    v = core_backup_read(CORE_BACKUP_COUNT);
    (void)v;

    /* ---- Hardware RNG ---- */
    core_rng_init();

    uint32_t r1 = core_rng_read();
    (void)r1;

    uint32_t buf[4];
    uint32_t filled = core_rng_fill(buf, 4);
    (void)filled;

    int err = core_rng_error();
    (void)err;

    core_rng_deinit();

    /* ---- RTC Alarm A ---- */
    core_rtc_init();
    core_rtc_set_time(12, 0, 0);

    /* Set alarm for 12:00:03 */
    core_rtc_set_alarm(12, 0, 3);

    /* Poll for alarm */
    alarm_checked = core_rtc_alarm_fired();

    /* Clear alarm */
    core_rtc_clear_alarm();

    /* Wildcard alarm: every minute at XX:XX:00 */
    core_rtc_set_alarm(0xFF, 0xFF, 0);
    alarm_checked = core_rtc_alarm_fired();
    core_rtc_clear_alarm();

    /* Wakeup timer (backward compat) */
    core_rtc_alarm(5);
    core_rtc_alarm_stop();

    while (1) {
        core_delay_ms(1000);
    }
}
