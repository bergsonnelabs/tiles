/**
 * hw-sleep-cycle — bench test for the 2026-09-25 sleep / RTC / backup-register /
 * watchdog-during-sleep fixes on Core.ST.L0.1, Core.ST.L4.1 / L4.2 and
 * Core.ST.W5. See README.md for flashing and reading the result.
 *
 * config.json arms the 5 s watchdog in core_init(), the same as a Studio
 * project. Tests, one bit each in the pass/fail masks:
 *   T1 backup    write + read back two backup registers
 *   T2 alarm     RTC_ALRMAR reads back what was written, and Alarm A fires
 *                ~3 s after it is set
 *   T3 stop #1   core_stop_for(10) with the watchdog running: returns without
 *                a watchdog reset, 9-12 s by the RTC
 *   T4 stop #2   the same again (stale wake flags used to break the 2nd one)
 *   T5 guard     core_standby_for(10) refuses (HAL_ERROR) with a 5 s watchdog
 *   T6 standby   (L4 only) core_standby_for(2) resets into a boot where
 *                core_woke_from_standby() == 1
 *   T7 clear     (L4 only) core_clear_standby_flag() makes it read 0
 *   T8 watchdog  core_init() armed the watchdog (5 s → 2.5 s sleep chunks)
 *
 * Progress survives resets in backup registers 0-4 (the L0 has only five), so
 * a watchdog reset in the middle of a sleep is recorded as that test failing.
 */

#include "core.h"
#include "core_power.h"
#include "core_rtc.h"
#include "core_backup.h"
#include "core_watchdog.h"
#if defined(STM32L422xx)
#include "core_usb.h"
#endif

/* ---- Backup-register layout (indices 0-4 exist on every Core) ---- */
#define BKP_STATE    0u   /* MAGIC << 24 | phase */
#define BKP_RESULT   1u   /* fail mask << 16 | pass mask */
#define BKP_TIMING   2u   /* watchdog resets << 16 | stop #2 s << 8 | stop #1 s */
#define BKP_VERDICT  3u   /* VERDICT_PASS, or VERDICT_FAIL | first failing test */
#define BKP_ALARM    4u   /* T2: ms from set_alarm to the flag (0 = never) */

#define MAGIC        0x5Cu
enum { PH_START = 0, PH_GUARD, PH_STOP1, PH_STOP2, PH_STBY, PH_REPORT, PH_DONE };

#define T_BKP     (1u << 0)
#define T_ALARM   (1u << 1)
#define T_STOP1   (1u << 2)
#define T_STOP2   (1u << 3)
#define T_GUARD   (1u << 4)
#define T_STBY    (1u << 5)
#define T_CLEAR   (1u << 6)
#define T_WDOG    (1u << 7)

#if defined(STM32L422xx)
#define EXPECTED  (T_BKP | T_ALARM | T_STOP1 | T_STOP2 | T_GUARD | T_STBY | T_CLEAR | T_WDOG)
#else
#define EXPECTED  (T_BKP | T_ALARM | T_STOP1 | T_STOP2 | T_GUARD | T_WDOG)
#endif

#define VERDICT_PASS  0x600D600Du
#define VERDICT_FAIL  0xBAD00000u   /* | index (1-8) of the first failing test */

/* SWD-readable copy of the result: its address is
 *   arm-none-eabi-nm build/hw-sleep-cycle.elf | grep g_sleep_cycle */
typedef struct {
    uint32_t magic;          /* 0x51EEC7C1 once the report is up */
    uint32_t verdict;        /* same encoding as BKP_VERDICT */
    uint32_t pass, fail, expected;
    uint32_t phase, wd_resets;
    uint32_t stop_s[2];      /* RTC seconds each core_stop_for(10) took */
    uint32_t stop_ms[2];     /* core_millis() across each (should be ~10000) */
    uint32_t alarm_ms;       /* T2 latency */
    uint32_t alrmar;         /* T2 RTC_ALRMAR read-back */
} sleep_cycle_result_t;

__attribute__((used)) volatile sleep_cycle_result_t g_sleep_cycle;

static uint32_t s_pass, s_fail, s_timing;

static void mark(uint32_t t, int ok)
{
    if (ok) s_pass |= t; else s_fail |= t;
}

static void save(uint32_t phase)
{
    core_backup_write(BKP_RESULT, (s_fail << 16) | s_pass);
    core_backup_write(BKP_TIMING, s_timing);
    core_backup_write(BKP_STATE, (MAGIC << 24) | phase);
}

static uint32_t rtc_now_s(void)
{
    uint8_t h, m, s;
    core_rtc_get_time(&h, &m, &s);
    return (uint32_t)h * 3600u + (uint32_t)m * 60u + s;
}

/* Feed-while-waiting delay (the watchdog is 5 s). */
static void wait_ms(uint32_t ms)
{
    while (ms) {
        uint32_t d = ms > 100u ? 100u : ms;
        core_watchdog_feed();
        core_delay_ms(d);
        ms -= d;
    }
}

/* T3/T4: one core_stop_for(10) with the watchdog armed. */
static void stop_test(int n)
{
    uint32_t t0 = rtc_now_s(), m0 = core_millis();
    LED_ON();                                 /* on while asleep */
    core_stop_for(10);
    LED_OFF();
    uint32_t dt = (rtc_now_s() + 86400u - t0) % 86400u;
    g_sleep_cycle.stop_s[n]  = dt;
    g_sleep_cycle.stop_ms[n] = core_millis() - m0;
    s_timing |= (dt & 0xFFu) << (8 * n);
    mark(n ? T_STOP2 : T_STOP1, dt >= 9u && dt <= 12u);
}

/* Returns 1 once it has been through Stop (USB is gone on the L4 after that). */
static int run_from_start(void)
{
    s_pass = s_fail = s_timing = 0;

    /* T8: core_init() armed the 5 s watchdog, so sleeps chunk at ~2.5 s. */
    uint32_t chunk = core_watchdog_sleep_chunk_ms();
    mark(T_WDOG, core_watchdog_running() && chunk >= 2000u && chunk <= 3000u);

    /* T1: backup registers. The top register (4 on the L0, 31 elsewhere) and
     * BKP_VERDICT, which is overwritten with the verdict at the end. */
    core_backup_write(CORE_BACKUP_COUNT - 1, 0xA5C35A3Cu);
    core_backup_write(BKP_VERDICT, 0x0BADF00Du);
    int bkp_ok = core_backup_read(CORE_BACKUP_COUNT - 1) == 0xA5C35A3Cu
              && core_backup_read(BKP_VERDICT) == 0x0BADF00Du;
    mark(T_BKP, bkp_ok);

    /* T2: Alarm A at xx:xx:03, set at 12:00:00. */
    core_rtc_set_time(12, 0, 0);
    core_rtc_set_alarm(0xFF, 0xFF, 3);
    g_sleep_cycle.alrmar = RTC_ALRMAR;
    uint32_t a0 = core_millis(), alarm_ms = 0;
    while ((uint32_t)(core_millis() - a0) < 8000u) {
        if (core_rtc_alarm_fired()) { alarm_ms = core_millis() - a0; break; }
        wait_ms(20);
    }
    core_rtc_clear_alarm();
    g_sleep_cycle.alarm_ms = alarm_ms;
    core_backup_write(BKP_ALARM, alarm_ms);
    mark(T_ALARM, g_sleep_cycle.alrmar == 0x80808003u
                  && alarm_ms >= 1500u && alarm_ms <= 4500u);

    if (!bkp_ok) return 0;        /* can't carry state across resets: report now */

    /* T5: a 10 s Standby can't be fed through a 5 s watchdog → HAL_ERROR.
     * If it slept anyway, the next boot finds PH_GUARD and fails T5. */
    save(PH_GUARD);
    mark(T_GUARD, core_standby_for(10) == HAL_ERROR);

    /* T3, T4 */
    save(PH_STOP1);
    stop_test(0);
    save(PH_STOP2);
    stop_test(1);

#if defined(STM32L422xx)
    /* T6: 2 s Standby (inside the 2.5 s window). Resets on wake. */
    save(PH_STBY);
    core_watchdog_feed();
    (void)core_standby_for(2);
    mark(T_STBY, 0);              /* only reached if it refused */
#endif
    return 1;
}

static uint32_t first_failure(uint32_t missing)
{
    for (uint32_t i = 0; i < 8; i++)
        if (missing & (1u << i)) return i + 1u;
    return 0;
}

#if defined(STM32L422xx)
static void print_report(uint32_t verdict)
{
    static const char *const names[8] = {
        "T1 backup regs", "T2 RTC alarm", "T3 stop_for #1", "T4 stop_for #2",
        "T5 standby guard", "T6 standby wake", "T7 standby clear", "T8 watchdog on"
    };
    core_usb_printf("\r\n[hw-sleep-cycle] %s  (pass=0x%02lx fail=0x%02lx expected=0x%02lx)\r\n",
                    verdict == VERDICT_PASS ? "PASS" : "FAIL",
                    (unsigned long)s_pass, (unsigned long)s_fail, (unsigned long)EXPECTED);
    for (uint32_t i = 0; i < 8; i++) {
        uint32_t b = 1u << i;
        core_usb_printf("  %-18s %s\r\n", names[i],
                        (s_pass & b) ? "PASS" : (s_fail & b) ? "FAIL" : "not run");
    }
    core_usb_printf("  stop #1 %lu s, stop #2 %lu s (RTC); alarm after %lu ms; "
                    "watchdog resets mid-test: %lu\r\n",
                    (unsigned long)(s_timing & 0xFFu), (unsigned long)((s_timing >> 8) & 0xFFu),
                    (unsigned long)core_backup_read(BKP_ALARM),
                    (unsigned long)(s_timing >> 16));
}
#endif

int main(void)
{
    core_init();                  /* arms the 5 s watchdog (config.json) */
    core_led_init();
    core_rtc_init();              /* RTC on LSI; same source → backup regs kept */

    int wd = core_watchdog_caused_reset();
    core_watchdog_clear_flags();  /* L0/W5 keep RCC reset flags until cleared */

    uint32_t state = core_backup_read(BKP_STATE);
    uint32_t phase = ((state >> 24) == MAGIC) ? (state & 0xFFu) : PH_START;
    s_pass = core_backup_read(BKP_RESULT) & 0xFFFFu;
    s_fail = core_backup_read(BKP_RESULT) >> 16;
    s_timing = core_backup_read(BKP_TIMING);
    int slept_this_boot = 0;

    switch (phase) {
    case PH_GUARD:                /* core_standby_for(10) slept instead of refusing */
        mark(T_GUARD, 0);
        if (wd) s_timing += 1u << 16;
        save(PH_STOP1);
        stop_test(0);
        save(PH_STOP2);
        stop_test(1);
        slept_this_boot = 1;
        break;
    case PH_STOP1:
    case PH_STOP2:
        if (!wd) {                /* reset/reflash mid-test: start over */
            slept_this_boot = run_from_start();
            break;
        }
        s_timing += 1u << 16;     /* the watchdog fired inside core_stop_for */
        mark(phase == PH_STOP1 ? T_STOP1 : T_STOP2, 0);
        if (phase == PH_STOP1) {
            save(PH_STOP2);
            stop_test(1);
            slept_this_boot = 1;
        }
        break;
    case PH_STBY:
        /* SBF is also set if the watchdog reset us out of Standby: not a pass. */
        mark(T_STBY, core_woke_from_standby() && !wd);
        core_clear_standby_flag();
        mark(T_CLEAR, !core_woke_from_standby());
        if (wd) s_timing += 1u << 16;
        break;
    case PH_REPORT:
        break;                    /* fresh boot after the run: show the result */
    default:                      /* PH_START, PH_DONE, or garbage: run it all */
        slept_this_boot = run_from_start();
        break;
    }

    uint32_t missing = (EXPECTED & ~s_pass) | s_fail;
    uint32_t verdict = missing ? (VERDICT_FAIL | first_failure(missing)) : VERDICT_PASS;
    core_backup_write(BKP_VERDICT, verdict);

#if defined(STM32L422xx)
    /* USB doesn't survive Stop: report from a fresh boot. */
    if (phase != PH_REPORT && slept_this_boot) {
        save(PH_REPORT);
        SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
        while (1) ;
    }
#else
    (void)slept_this_boot;
#endif
    save(PH_DONE);                /* the next reset runs the test again */

    g_sleep_cycle.verdict  = verdict;
    g_sleep_cycle.pass     = s_pass;
    g_sleep_cycle.fail     = s_fail;
    g_sleep_cycle.expected = EXPECTED;
    g_sleep_cycle.phase    = phase;
    g_sleep_cycle.wd_resets = s_timing >> 16;
    g_sleep_cycle.magic    = 0x51EEC7C1u;

    /* PASS: slow 1 s on / 1 s off. FAIL: N quick blinks (N = first failing
     * test), then a 1.5 s pause. */
    uint32_t last_print = 0;
    while (1) {
#if defined(STM32L422xx)
        if ((uint32_t)(core_millis() - last_print) >= 3000u) {
            last_print = core_millis();
            print_report(verdict);
        }
#else
        (void)last_print;
#endif
        if (verdict == VERDICT_PASS) {
            LED_ON();  wait_ms(1000);
            LED_OFF(); wait_ms(1000);
        } else {
            for (uint32_t i = 0; i < (verdict & 0xFFu); i++) {
                LED_ON();  wait_ms(150);
                LED_OFF(); wait_ms(250);
            }
            wait_ms(1500);
        }
    }
}
