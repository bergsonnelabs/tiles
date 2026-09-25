/**
 * hw-pin-wake — bench test for pin-change wake from Stop
 * (core_stop_until_on_change, sdk/core/core_power.h) on Core.ST.L4.1.
 * See README.md for the bench procedure and how to read the result.
 *
 * Hardware (definitions/Core-ST-L4-1-b*.json): pad 4 is PB6, wired through a
 * 2.2 kOhm on-tile resistor to chip pin PA9. Driving PA9 high/low pulls pad 4
 * high/low. Pad 1 is GND, pad 10 is V+. The onboard LED is PA8, active-high.
 * PA9 is a raw chip pin, not a tile pad, so it's driven directly through the
 * LL GPIO layer (ll_gpio_*) rather than core_pad_*.
 *
 * Tests, one bit each in the pass/fail masks:
 *   T1  loopback   drive PA9 high/low, read pad 4 back high/low (run mode)
 *   T2  fast-fail   core_stop_until_on_change() on a pad with no GPIO (pad 1,
 *                    GND) returns right away instead of sleeping
 *   T3  wake        core_stop_until_on_change_timeout(4, EDGE_FALLING, 60 s)
 *                    actually wakes the Core from Stop when a human shorts pad 4
 *                    to pad 1 (unattended, it times out and T3 reports FAIL)
 *   T4  at-level    pad 4 already low (PA9 low) when a falling-edge wait starts:
 *                    both the old call and the _timeout call return at once
 *                    (1) instead of sleeping forever (added 2026-09-25)
 *   T5  timeout     pad 4 held high, core_stop_until_on_change_timeout(..., 3000)
 *                    returns 0 after 2.5-4 s of Stop, no watchdog reset
 *
 * T3 needs a human: nothing that runs in Stop can drive PA9, so a real
 * wake-from-Stop edge has to come from outside the chip. USB CDC doesn't
 * reliably survive Stop on the L4 (a known issue — see hw-sleep-cycle), so,
 * like that test, progress and results are kept in backup registers and the
 * Core does a software reset after each Stop attempt to get a clean
 * re-enumeration before printing again. An attempt counter (also in a backup
 * register) bounds the retry loop so a human who is too fast (edge caught
 * inside the first watchdog chunk, so the >2.5 s pass bar isn't met) doesn't
 * make the test spin forever.
 */

#include "core.h"
#include "core_power.h"
#include "core_backup.h"
#include "core_watchdog.h"   /* also pulls in hal_dfu.h -> SCB_AIRCR */
#include "core_rtc.h"
#include "core_usb.h"        /* Core.ST.L4 only -- config.json pins this test to it */

/* ---- Backup-register layout ---- */
#define BKP_STATE    0u   /* MAGIC << 24 | phase */
#define BKP_RESULT   1u   /* fail mask << 16 | pass mask */
#define BKP_TIMING   2u   /* last T3 attempt's measured sleep, ms */
#define BKP_VERDICT  3u   /* VERDICT_PASS, or VERDICT_FAIL | first failing test */
#define BKP_ATTEMPT  4u   /* T3 attempt counter (1-based) */
#define BKP_WD       5u   /* 1 if a watchdog reset was ever seen during a T3 wait */

#define BKP_T5       6u   /* T5: measured ms << 8 | return value + 1 */

#define MAGIC  0xC9u
enum { PH_START = 0, PH_RETRY, PH_ARMED, PH_REPORT, PH_T5 };

#define T_T1  (1u << 0)
#define T_T2  (1u << 1)
#define T_T3  (1u << 2)
#define T_T4  (1u << 3)
#define T_T5  (1u << 4)
#define EXPECTED  (T_T1 | T_T2 | T_T3 | T_T4 | T_T5)
#define N_TESTS   5u

#define VERDICT_PASS  0x900DBEEFu
#define VERDICT_FAIL  0xBAD00000u   /* | index (1-5) of the first failing test */

#define T3_TIMEOUT_MS  60000u       /* unattended runs end here instead of hanging */

#define T3_MAX_ATTEMPTS  5u
/* PAD_GND (coregen, core_pads.h) is pad 1 -- no GPIO, proves the "can't take
 * an edge interrupt" path. LOOP_PAD is pad 4 (PB6), wired to PA9 through the
 * on-tile 2.2k. (Not PAD_4 -- coregen doesn't generate a name for it, and
 * PAD_4_PORT/PAD_4_PIN already mean something else.) */
#define LOOP_PAD  4u

/* SWD-readable copy of the result:
 *   arm-none-eabi-nm build/hw-pin-wake.elf | grep g_pin_wake */
typedef struct {
    uint32_t magic;      /* 0x51EEC7C2 once the report is up */
    uint32_t verdict;
    uint32_t pass, fail, expected;
    uint32_t phase, attempt, wd_seen;
    uint32_t slept_ms;    /* T3: measured Stop duration of the winning attempt */
    uint32_t chunk_ms;    /* watchdog sleep-chunk size the pass bar is measured against */
} pin_wake_result_t;

__attribute__((used)) volatile pin_wake_result_t g_pin_wake;

/* Feed-while-waiting delay (the watchdog is 5 s). */
static void wait_ms(uint32_t ms)
{
    while (ms) {
        uint32_t d = ms > 50u ? 50u : ms;
        core_watchdog_feed();
        core_delay_ms(d);
        ms -= d;
    }
}

static void save(uint32_t phase, uint32_t pass, uint32_t fail,
                  uint32_t attempt, uint32_t wd_seen, uint32_t slept_ms)
{
    core_backup_write(BKP_RESULT, (fail << 16) | pass);
    core_backup_write(BKP_ATTEMPT, attempt);
    core_backup_write(BKP_WD, wd_seen);
    core_backup_write(BKP_TIMING, slept_ms);
    core_backup_write(BKP_STATE, (MAGIC << 24) | phase);
}

static void do_reset(void)
{
    SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
    while (1) { }
}

/* T1: PA9 -[2.2k]- pad 4 (PB6) loopback, run mode only. */
static int run_t1(void)
{
    ll_rcc_gpio_clk_enable(GPIOA);
    ll_gpio_config_output(GPIOA, 9);
    core_pad_input(LOOP_PAD, PULL_NONE);

    ll_gpio_set(GPIOA, 1UL << 9);           /* PA9 high */
    core_delay_ms(5);
    int hi = core_pad_read(LOOP_PAD);

    ll_gpio_clear(GPIOA, 1UL << 9);         /* PA9 low */
    core_delay_ms(5);
    int lo = core_pad_read(LOOP_PAD);

    int ok = (hi == 1) && (lo == 0);
    core_usb_printf("[hw-pin-wake] T1 loopback: PA9 hi -> pad4=%d (want 1), "
                    "PA9 lo -> pad4=%d (want 0)  %s\r\n",
                    hi, lo, ok ? "PASS" : "FAIL");
    return ok;
}

/*
 * T2: core_stop_until_on_change() promises to "return right away if the pad
 * can't take an edge interrupt" (core_power.h). Pad 1 is GND -- no GPIO --
 * so core_pad_on_change() fails immediately (hal_pad_lookup() returns a NULL
 * port for a pad with no PAD_n_PORT define) and the function returns without
 * ever arming an EXTI or touching Stop. That's automatable and bounded, so
 * that's what this checks: elapsed time under 100 ms.
 *
 * The case this test once couldn't cover, an edge that already happened
 * before the call (the pad already sits at its wake level), is T4: since
 * 2026-09-25 core_stop_until_on_change() reads the pad after arming and
 * returns at once, and core_stop_until_on_change_timeout() bounds every wait.
 */
static int run_t2(void)
{
    uint32_t t0 = core_millis();
    core_stop_until_on_change(PAD_GND, EDGE_FALLING);
    uint32_t dt = core_millis() - t0;

    int ok = dt < 100u;
    core_usb_printf("[hw-pin-wake] T2 no-GPIO pad returns promptly: pad 1 (GND) "
                    "-> core_stop_until_on_change returned in %lu ms (want <100)  %s\r\n",
                    (unsigned long)dt, ok ? "PASS" : "FAIL");
    return ok;
}

/* T4: PA9 low pulls pad 4 low (2.2k beats the ~40k internal pull-up the
 * falling-edge arm selects). A falling-edge wait must see the pad is already
 * at its wake level and return without entering Stop.
 *
 * Needs T1's loopback: without it pad 4 isn't low, and the untimed call
 * would wait in Stop for an edge nobody makes (a bench run on 2026-09-25 did
 * exactly that). So it is skipped when T1 failed, and the bounded _timeout
 * call goes first; the untimed call runs only once that returned at once. */
static int run_t4(int loop_ok)
{
    if (!loop_ok) {
        core_usb_printf("[hw-pin-wake] T4 skipped: needs T1's PA9 -> pad 4 loopback  FAIL\r\n");
        return 0;
    }
    ll_rcc_gpio_clk_enable(GPIOA);
    ll_gpio_config_output(GPIOA, 9);
    ll_gpio_clear(GPIOA, 1UL << 9);
    core_delay_ms(5);

    uint32_t t0 = core_millis();
    int r = core_stop_until_on_change_timeout(LOOP_PAD, EDGE_FALLING, 3000u);
    uint32_t dt_new = core_millis() - t0;
    uint32_t dt_old = 0xFFFFFFFFu;
    if (r == 1 && dt_new < 100u) {
        t0 = core_millis();
        core_stop_until_on_change(LOOP_PAD, EDGE_FALLING);
        dt_old = core_millis() - t0;
    }
    core_pad_on_change_stop(LOOP_PAD);

    int ok = dt_old < 100u && dt_new < 100u && r == 1;
    core_usb_printf("[hw-pin-wake] T4 pad already low: _timeout call %lu ms returned %d, "
                    "old call %lu ms (want <100 ms, 1, <100 ms)  %s\r\n",
                    (unsigned long)dt_new, r, (unsigned long)dt_old, ok ? "PASS" : "FAIL");
    return ok;
}

/* T5: pad 4 held high by PA9, a 3 s falling-edge wait with nothing touching
 * it must time out (return 0) after ~3 s of Stop. USB doesn't survive Stop:
 * the result goes to a backup register and the Core resets to report it. */
static void run_t5(uint32_t s_pass, uint32_t s_fail)
{
    ll_rcc_gpio_clk_enable(GPIOA);
    ll_gpio_config_output(GPIOA, 9);
    ll_gpio_set(GPIOA, 1UL << 9);
    core_delay_ms(5);
    core_usb_printf("[hw-pin-wake] T5: 3 s timed wait in Stop (don't touch pad 4)\r\n");
    core_delay_ms(50);

    save(PH_T5, s_pass, s_fail, 0u, 0u, 0u);
    uint32_t t0 = core_millis();
    int r = core_stop_until_on_change_timeout(LOOP_PAD, EDGE_FALLING, 3000u);
    uint32_t dt = core_millis() - t0;
    core_pad_on_change_stop(LOOP_PAD);
    core_backup_write(BKP_T5, (dt << 8) | (uint32_t)(r + 1));
    if (r == 0 && dt >= 2500u && dt <= 4000u) s_pass |= T_T5; else s_fail |= T_T5;
    save(PH_RETRY, s_pass, s_fail, 0u, 0u, 0u);
    do_reset();                   /* fresh boot -> T3 */
}

/*
 * T3: one attempt at a real wake-from-Stop. Drives PA9 high (pull-up on pad
 * 4 through the 2.2k, ~1.5 mA if shorted to GND -- safe), prompts, blinks the
 * LED for ~30 s so a human has time to get the tweezers on pad 4 and pad 1,
 * then calls core_stop_until_on_change_timeout(4, EDGE_FALLING, 60 s). With
 * nobody at the bench it times out and T3 is scored FAIL ("no touch"), so an
 * unattended run finishes instead of sleeping forever.
 *
 * PASS bar: slept longer than one watchdog chunk (2.5 s at the 5 s timeout
 * here) with no watchdog reset. A touch inside the first chunk still means
 * the wake worked, just too fast to prove the chunked-sleep path survived a
 * full feed-and-resleep cycle, so it's scored as "too fast" and retried
 * (attempt counter bounds this at T3_MAX_ATTEMPTS so a fast human -- or a
 * human who never touches it at all and eventually power-cycles the board --
 * can't spin this forever).
 *
 * USB CDC doesn't reliably survive Stop on the L4, so results are saved to
 * backup registers and the Core does its own software reset after every
 * attempt; the next boot picks the story back up from BKP_STATE.
 */
static void t3_attempt(uint32_t s_pass, uint32_t s_fail, uint32_t attempt, uint32_t wd_seen)
{
    attempt++;

    if (attempt > T3_MAX_ATTEMPTS) {
        s_fail |= T_T3;
        core_usb_printf("[hw-pin-wake] T3: giving up after %lu attempts -- the edge "
                        "keeps arriving before a full watchdog chunk. Wait a beat "
                        "longer after \"sleeping now\" before touching the pad.\r\n",
                        (unsigned long)T3_MAX_ATTEMPTS);
        save(PH_REPORT, s_pass, s_fail, attempt - 1u, wd_seen, core_backup_read(BKP_TIMING));
        return;   /* USB is still fine this boot (never slept again) -- no reset needed */
    }

    /* PA9 high: pull-up on pad 4. GPIO state is retained through Stop, so this
     * holds for the whole wait, including while the Core is actually asleep. */
    ll_rcc_gpio_clk_enable(GPIOA);
    ll_gpio_config_output(GPIOA, 9);
    ll_gpio_set(GPIOA, 1UL << 9);

    core_usb_printf("\r\n[hw-pin-wake] T3 attempt %lu/%lu: touch pad 4 to GND (pad 1) "
                    "within 30 s\r\n", (unsigned long)attempt, (unsigned long)T3_MAX_ATTEMPTS);

    for (uint32_t elapsed = 0; elapsed < 30000u; elapsed += 250u) {
        LED_ON();  wait_ms(100);
        LED_OFF(); wait_ms(150);
    }

    core_usb_printf("[hw-pin-wake] T3: sleeping now (Stop mode) -- touch pad 4 to "
                    "GND to wake it\r\n");
    core_delay_ms(50);      /* let the CDC TX drain before Stop kills the clock */

    save(PH_ARMED, s_pass, s_fail, attempt, wd_seen, core_backup_read(BKP_TIMING));

    uint32_t chunk = core_watchdog_sleep_chunk_ms();
    uint32_t t0 = core_millis();
    int r = core_stop_until_on_change_timeout(LOOP_PAD, EDGE_FALLING, T3_TIMEOUT_MS);
    uint32_t slept_ms = core_millis() - t0;

    if (core_watchdog_caused_reset()) wd_seen = 1;   /* belt-and-suspenders; see PH_ARMED at boot */

    if (r == 0) {
        s_fail |= T_T3;               /* nobody touched it: timed out */
        save(PH_REPORT, s_pass, s_fail, attempt, wd_seen, slept_ms | 0x80000000u);
    } else if (wd_seen) {
        s_fail |= T_T3;
        save(PH_REPORT, s_pass, s_fail, attempt, wd_seen, slept_ms);
    } else if (slept_ms > chunk) {
        s_pass |= T_T3;
        save(PH_REPORT, s_pass, s_fail, attempt, wd_seen, slept_ms);
    } else {
        /* Woke, but inside the first chunk -- too fast to prove the chunked
         * sleep path. Retry rather than fail outright. */
        save(PH_RETRY, s_pass, s_fail, attempt, wd_seen, slept_ms);
    }

    do_reset();   /* fresh boot -> clean USB re-enumeration */
}

static uint32_t first_failure(uint32_t missing)
{
    for (uint32_t i = 0; i < N_TESTS; i++)
        if (missing & (1u << i)) return i + 1u;
    return 0;
}

static void print_report(uint32_t verdict, uint32_t s_pass, uint32_t s_fail,
                          uint32_t slept_ms, uint32_t chunk, uint32_t wd_seen,
                          uint32_t attempt)
{
    static const char *const names[N_TESTS] = {
        "T1 pad loopback", "T2 no-GPIO fast-fail", "T3 wake from Stop",
        "T4 already at level", "T5 timed wait"
    };
    core_usb_printf("\r\n[hw-pin-wake] %s  (pass=0x%02lx fail=0x%02lx expected=0x%02lx)\r\n",
                    verdict == VERDICT_PASS ? "PASS" : "FAIL",
                    (unsigned long)s_pass, (unsigned long)s_fail, (unsigned long)EXPECTED);
    for (uint32_t i = 0; i < N_TESTS; i++) {
        uint32_t b = 1u << i;
        core_usb_printf("  %-22s %s\r\n", names[i],
                        (s_pass & b) ? "PASS" : (s_fail & b) ? "FAIL" : "not run");
    }
    if (slept_ms & 0x80000000u)
        core_usb_printf("  T3: no touch -- timed out after %lu ms (needs a human, see README)\r\n",
                        (unsigned long)(slept_ms & 0x7FFFFFFFu));
    else
        core_usb_printf("  T3: slept %lu ms before wake (need >%lu ms, one watchdog chunk), "
                        "watchdog reset seen: %s, attempts used: %lu\r\n",
                        (unsigned long)slept_ms, (unsigned long)chunk,
                        wd_seen ? "YES" : "no", (unsigned long)attempt);
    uint32_t t5 = core_backup_read(BKP_T5);
    core_usb_printf("  T5: returned %ld after %lu ms (want 0 after 2500-4000 ms)\r\n",
                    (long)(int32_t)((t5 & 0xFFu) - 1u), (unsigned long)(t5 >> 8));
}

int main(void)
{
    core_init();                 /* arms the 5 s watchdog (config.json) */
    core_led_init();
    core_rtc_init();             /* RTC on LSI; core_stop_until_on_change also brings it up */

    int wd = core_watchdog_caused_reset();
    core_watchdog_clear_flags();

    uint32_t state = core_backup_read(BKP_STATE);
    uint32_t phase = ((state >> 24) == MAGIC) ? (state & 0xFFu) : PH_START;
    uint32_t s_pass = core_backup_read(BKP_RESULT) & 0xFFFFu;
    uint32_t s_fail = core_backup_read(BKP_RESULT) >> 16;
    uint32_t attempt = core_backup_read(BKP_ATTEMPT);
    uint32_t wd_seen = core_backup_read(BKP_WD);
    uint32_t slept_ms = core_backup_read(BKP_TIMING);
    uint32_t chunk = core_watchdog_sleep_chunk_ms();

    wait_ms(1500);                /* CDC eats early prints right after enumeration */

    core_usb_printf("\r\n[hw-pin-wake] boot: cause=%s phase=%lu attempt=%lu\r\n",
                    wd ? "WATCHDOG" : "power/reset", (unsigned long)phase, (unsigned long)attempt);

    if (phase == PH_ARMED) {
        /* Landed back in main() with the last saved phase still "about to
         * sleep" -- we never reached the code that saves PH_REPORT/PH_RETRY
         * and resets on purpose, so something reset the Core while it was
         * waiting for the touch: a real watchdog reset (wd == 1, a genuine
         * SDK/chunking failure) or a manual power-cycle/reset (wd == 0, the
         * test was abandoned mid-wait). Either way, stop here and report
         * rather than silently retrying into a reset loop. */
        s_fail |= T_T3;
        if (wd) wd_seen = 1;
        core_usb_printf("[hw-pin-wake] T3: reset while waiting for the touch (%s) -- "
                        "treating as FAIL\r\n", wd ? "watchdog reset" : "manual reset/power-cycle");
        phase = PH_REPORT;
        save(phase, s_pass, s_fail, attempt, wd_seen, slept_ms);
    }

    switch (phase) {
    case PH_START:
        /* Magic didn't match BKP_STATE, so the other backup registers may
         * hold whatever an unrelated earlier firmware left there -- start
         * clean rather than trusting them. */
        s_pass = s_fail = attempt = wd_seen = slept_ms = 0u;
        core_backup_write(BKP_T5, 0u);
        int loop_ok = run_t1();
        if (loop_ok) s_pass |= T_T1; else s_fail |= T_T1;
        if (run_t2()) s_pass |= T_T2; else s_fail |= T_T2;
        if (run_t4(loop_ok)) s_pass |= T_T4; else s_fail |= T_T4;
        save(PH_START, s_pass, s_fail, 0u, wd_seen, slept_ms);
        run_t5(s_pass, s_fail);                        /* resets into PH_RETRY -> T3 */
        break;
    case PH_T5:
        /* Reset inside T5's timed Stop: a watchdog reset (or a manual one). */
        s_fail |= T_T5;
        save(PH_RETRY, s_pass, s_fail, 0u, wd ? 1u : 0u, 0u);
        t3_attempt(s_pass, s_fail, 0u, wd ? 1u : 0u);
        break;
    case PH_RETRY:
        t3_attempt(s_pass, s_fail, attempt, wd_seen);  /* does not return (resets) unless giving up */
        break;
    case PH_REPORT:
    default:
        break;                    /* fresh boot after the run (or an abandoned wait): report */
    }

    /* Re-read: t3_attempt() and the PH_ARMED handler above may have updated
     * these via save() without this boot's locals seeing it (give-up path). */
    s_pass = core_backup_read(BKP_RESULT) & 0xFFFFu;
    s_fail = core_backup_read(BKP_RESULT) >> 16;
    attempt = core_backup_read(BKP_ATTEMPT);
    wd_seen = core_backup_read(BKP_WD);
    slept_ms = core_backup_read(BKP_TIMING);

    uint32_t missing = (EXPECTED & ~s_pass) | s_fail;
    uint32_t verdict = missing ? (VERDICT_FAIL | first_failure(missing)) : VERDICT_PASS;
    core_backup_write(BKP_VERDICT, verdict);

    g_pin_wake.verdict  = verdict;
    g_pin_wake.pass     = s_pass;
    g_pin_wake.fail     = s_fail;
    g_pin_wake.expected = EXPECTED;
    g_pin_wake.phase    = PH_REPORT;
    g_pin_wake.attempt  = attempt;
    g_pin_wake.wd_seen  = wd_seen;
    g_pin_wake.slept_ms = slept_ms;
    g_pin_wake.chunk_ms = chunk;
    g_pin_wake.magic    = 0x51EEC7C2u;

    /* PASS: slow 1 s on / 1 s off. FAIL: N quick blinks (N = first failing
     * test), then a 1.5 s pause. Report over CDC every 3 s. */
    uint32_t last_print = 0;
    while (1) {
        if ((uint32_t)(core_millis() - last_print) >= 3000u) {
            last_print = core_millis();
            print_report(verdict, s_pass, s_fail, slept_ms, chunk, wd_seen, attempt);
        }
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
