/**
 * timer_sim.c — the timer server (sdk/wpan/stm32_timer.c) and its LPTIM2 +
 * SysTick interface (sdk/wpan/stm32_timer_if.c) on the host
 *
 * Builds both sources unchanged against a simulated LPTIM2 (16-bit one-shot,
 * PCLK1 / 32) and SysTick (1 kHz), and runs the timers the SDK relies on:
 *
 *   one-shots of 10 ms, 65 ms, 66 ms, 250 ms, 5 s and 60 s fire once, on time
 *   a periodic 1 s timer keeps firing every second
 *   interleaved timers fire in deadline order, including a short timer
 *   started while a long one runs (it becomes the new head mid-context)
 *   a restarted 250 ms timer (Thread's settings save) fires once, after the
 *   last restart; a stopped timer never fires
 *
 * Every scenario runs with the SysTick counter at 0 and just short of its
 * 32-bit wrap, at three phases against the SysTick edge, and with SysTick
 * and LPTIM2 interrupts that land on the same instant taken in both orders.
 * "On time" is the interface's resolution, SysTick's millisecond: a timer of
 * N ms fires within (N - 1, N + 1] ms of its start, or N + 2 when LPTIM2's
 * interrupt beats a simultaneous SysTick tick (it then re-arms 1 ms).
 *
 * The whole run repeats per clock level (SIM_MHZ = 16, 32, 64, 100), since
 * the longest LPTIM2 shot is 0xFFFF / (PCLK1 / 32 kHz): 131 ms down to 21 ms.
 *
 *   make            # all four clock levels
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Single-threaded simulation: no interrupt can preempt a call. */
#define UTIL_TIMER_INIT_CRITICAL_SECTION()
#define UTIL_TIMER_ENTER_CRITICAL_SECTION()
#define UTIL_TIMER_EXIT_CRITICAL_SECTION()
#define TIMER_IF_ENTER_CRITICAL()
#define TIMER_IF_EXIT_CRITICAL()

uint32_t sim_lptim2[8];
uint32_t sim_scratch;
int sim_nvic_enabled;
volatile uint32_t _systick_ticks;

#ifndef TIMER_IF_SRC
#define TIMER_IF_SRC "stm32_timer_if.c"
#endif
#include "stm32_timer.c"
#include TIMER_IF_SRC

/* ---- Simulated hardware ------------------------------------------------- */

#define TPM   ((uint64_t)(SIM_MHZ * 1000000UL / 32UL / 1000UL))   /* LPTIM ticks per ms */
#define R_ISR  0
#define R_ICR  1
#define R_DIER 2
#define R_CR   4
#define R_ARR  6
#define ARRM   (1u << 1)

static uint64_t s_t;             /* now, in LPTIM ticks */
static uint32_t s_sys_base;      /* SysTick count at s_t == 0 */
static int      s_lp_running;
static uint64_t s_lp_match;
static int      s_lptim_first;   /* on a tie, LPTIM2's interrupt runs before SysTick's */
static uint64_t s_shots;

/* Pick up what the driver wrote: ICR clears, CR stops or starts a shot. */
static void sync(void)
{
    uint32_t *r = sim_lptim2;
    if (r[R_ICR] & ARRM) {
        r[R_ISR] &= ~ARRM;
        r[R_ICR] = 0;
    }
    if (!(r[R_CR] & 1u)) {
        s_lp_running = 0;
    } else if (r[R_CR] & 2u) {               /* SNGSTRT: count 0..ARR from now */
        s_lp_running = 1;
        s_lp_match = s_t + (r[R_ARR] & 0xFFFFu) + 1u;
        r[R_CR] &= ~2u;
        s_shots++;
    }
}

static void systick(void)
{
    _systick_ticks = s_sys_base + (uint32_t)(s_t / TPM);
}

static void lptim_match(void)
{
    s_lp_running = 0;
    sim_lptim2[R_ISR] |= ARRM;
    if ((sim_lptim2[R_DIER] & ARRM) && sim_nvic_enabled) {
        LPTIM2_IRQHandler();
        sync();
    }
}

static void advance_to(uint64_t target)
{
    for (;;) {
        uint64_t next_sys = (s_t / TPM + 1u) * TPM;
        uint64_t next_lp = s_lp_running ? s_lp_match : UINT64_MAX;
        uint64_t next = next_sys < next_lp ? next_sys : next_lp;
        if (next > target) {
            s_t = target;
            return;
        }
        s_t = next;
        if (next == next_sys && next == next_lp) {
            if (s_lptim_first) { lptim_match(); systick(); }
            else               { systick(); lptim_match(); }
        } else if (next == next_sys) {
            systick();
        } else {
            lptim_match();
        }
    }
}

static void advance_ms(double ms) { advance_to(s_t + (uint64_t)(ms * (double)TPM)); }

/* ---- Timers under test -------------------------------------------------- */

#define MAX_FIRES 128

typedef struct {
    UTIL_TIMER_Object_t obj;
    const char *name;
    uint32_t ms;
    uint64_t start;
    int fires;
    uint64_t at[MAX_FIRES];
} tt_t;

static int s_order[16], s_order_n;
static tt_t *s_all[16];
static int s_all_n;

static void cb(void *arg)
{
    tt_t *x = arg;
    if (x->fires < MAX_FIRES)
        x->at[x->fires] = s_t;
    x->fires++;
    for (int i = 0; i < s_all_n; i++)
        if (s_all[i] == x && s_order_n < 16)
            s_order[s_order_n++] = i;
}

static void tt_start(tt_t *x, const char *name, uint32_t ms, UTIL_TIMER_Mode_t mode)
{
    memset(x, 0, sizeof *x);
    x->name = name;
    x->ms = ms;
    UTIL_TIMER_Create(&x->obj, ms, mode, cb, x);
    x->start = s_t;
    UTIL_TIMER_Start(&x->obj);
    sync();
    s_all[s_all_n++] = x;
}

static double since_ms(const tt_t *x, int i) { return (double)(x->at[i] - x->start) / (double)TPM; }

/* ---- Checks ------------------------------------------------------------- */

#ifndef SIM_PRINT_MAX
#define SIM_PRINT_MAX 25
#endif
static int s_fail, s_checks;
static double s_max_late, s_max_early;
static char s_ctx[96];

#define CHECK(cond, ...) do { s_checks++; if (!(cond)) { s_fail++; \
    if (s_fail <= SIM_PRINT_MAX) { printf("  FAIL [%s] ", s_ctx); printf(__VA_ARGS__); printf("\n"); } } } while (0)

/* Fired exactly once, within (N - 1, N + slack] ms of its start. */
static void check_once(const tt_t *x, double from_start_ms)
{
    CHECK(x->fires == 1, "%s (%u ms): fired %d times", x->name, x->ms, x->fires);
    if (x->fires < 1)
        return;
    double d = since_ms(x, 0) - from_start_ms;
    double slack = s_lptim_first ? 2.0 : 1.0;
    CHECK(d > -1.0 && d <= slack + 1e-9, "%s (%u ms): fired at %+.3f ms", x->name, x->ms, d);
    if (d > s_max_late) s_max_late = d;
    if (d < s_max_early) s_max_early = d;
}

static void reset(uint32_t sys_base, uint64_t phase, int lptim_first)
{
    memset(sim_lptim2, 0, sizeof sim_lptim2);
    s_t = 0;
    s_sys_base = sys_base;
    s_lp_running = 0;
    s_lptim_first = lptim_first;
    systick();
    UTIL_TIMER_Init();
    sync();
    advance_to(phase);
    s_order_n = 0;
    s_all_n = 0;
}

static void check_idle(void)
{
    CHECK(UTIL_TIMER_GetTimerList() == NULL, "timer list not empty at the end");
    CHECK(!s_lp_running, "LPTIM2 still running with no timer");
}

/* ---- Scenarios ---------------------------------------------------------- */

static void one_shot(uint32_t ms)
{
    tt_t a;
    tt_start(&a, "one-shot", ms, UTIL_TIMER_ONESHOT);
    advance_ms(ms * 3.0 + 50.0);
    check_once(&a, ms);
    check_idle();
}

static void periodic(void)
{
    tt_t p;
    tt_start(&p, "periodic", 1000, UTIL_TIMER_PERIODIC);
    advance_ms(20500);
    CHECK(p.fires == 20, "periodic 1 s: %d fires in 20.5 s", p.fires);
    for (int i = 0; i < p.fires && i < MAX_FIRES; i++) {
        double iv = (double)(p.at[i] - (i ? p.at[i - 1] : p.start)) / (double)TPM;
        CHECK(iv > 999.0 && iv <= 1000.0 + (s_lptim_first ? 2.0 : 1.0) + 1e-9,
              "periodic 1 s: interval %d = %.3f ms", i, iv);
    }
    UTIL_TIMER_Stop(&p.obj);
    sync();
    check_idle();
}

static void interleaved(void)
{
    tt_t a, b, c, d, e, f;
    uint64_t t0 = s_t;
    tt_start(&c, "C 5 s", 5000, UTIL_TIMER_ONESHOT);
    tt_start(&a, "A 250 ms", 250, UTIL_TIMER_ONESHOT);
    advance_ms(50.3);
    tt_start(&b, "B 100 ms", 100, UTIL_TIMER_ONESHOT);        /* new head mid-context */
    advance_to(t0 + 1234 * TPM + TPM / 3);
    tt_start(&d, "D 300 ms", 300, UTIL_TIMER_ONESHOT);        /* new head under C */
    tt_start(&e, "E 500 ms", 500, UTIL_TIMER_ONESHOT);
    tt_start(&f, "F 500 ms", 500, UTIL_TIMER_ONESHOT);        /* same deadline as E */
    advance_ms(6000);
    check_once(&b, 100);
    check_once(&a, 250);
    check_once(&d, 300);
    check_once(&c, 5000);
    /* E and F share a deadline. UTIL_TIMER runs one expiry per interrupt and
     * re-arms the other for its minimum timeout, so the second of the pair is
     * up to 1 ms later; it queues the later-started timer first on a tie. */
    const tt_t *tied[2] = { &e, &f };
    for (int i = 0; i < 2; i++) {
        CHECK(tied[i]->fires == 1, "%s: fired %d times", tied[i]->name, tied[i]->fires);
        if (tied[i]->fires == 1) {
            double dt = since_ms(tied[i], 0) - 500.0;
            CHECK(dt > -1.0 && dt <= 3.0, "%s (tied): fired at %+.3f ms", tied[i]->name, dt);
        }
    }
    /* s_all: C A B D E F = 0 1 2 3 4 5; expected B A D {E F} C */
    int ok = s_order_n == 6 && s_order[0] == 2 && s_order[1] == 1 && s_order[2] == 3 &&
             s_order[3] + s_order[4] == 9 && s_order[5] == 0;
    CHECK(ok, "order: %d fires, got %d %d %d %d %d %d", s_order_n,
          s_order[0], s_order[1], s_order[2], s_order[3], s_order[4], s_order[5]);
    check_idle();
}

static void restarted(void)
{
    /* Thread's settings save: Stop + Start on every change, 250 ms debounce. */
    tt_t s;
    memset(&s, 0, sizeof s);
    s.name = "save 250 ms";
    s.ms = 250;
    UTIL_TIMER_Create(&s.obj, 250, UTIL_TIMER_ONESHOT, cb, &s);
    s_all[s_all_n++] = &s;
    for (int i = 0; i < 10; i++) {
        UTIL_TIMER_Stop(&s.obj);
        s.start = s_t;
        UTIL_TIMER_Start(&s.obj);
        sync();
        advance_ms(100.7);
    }
    advance_ms(1000);
    check_once(&s, 250);
    check_idle();
}

static void stopped(void)
{
    tt_t a;
    tt_start(&a, "stopped 5 s", 5000, UTIL_TIMER_ONESHOT);
    advance_ms(2000);
    UTIL_TIMER_Stop(&a.obj);
    sync();
    advance_ms(8000);
    CHECK(a.fires == 0, "stopped 5 s timer fired %d times", a.fires);
    check_idle();
}

int main(void)
{
    static const uint32_t lens[] = { 10, 65, 66, 250, 5000, 60000 };
    static const uint32_t bases[] = { 0u, 0xFFFFFFFFu - 2500u };
    const uint64_t phases[] = { 0u, TPM / 2u, TPM - 1u };
    int runs = 0;

    for (unsigned bi = 0; bi < 2; bi++)
        for (unsigned pi = 0; pi < 3; pi++)
            for (int lf = 0; lf < 2; lf++) {
                snprintf(s_ctx, sizeof s_ctx, "%d MHz, systick %08x, phase %llu, %s first",
                         (int)SIM_MHZ, bases[bi], (unsigned long long)phases[pi], lf ? "LPTIM" : "SysTick");
                for (unsigned li = 0; li < sizeof lens / sizeof lens[0]; li++) {
                    reset(bases[bi], phases[pi], lf);
                    one_shot(lens[li]);
                }
                reset(bases[bi], phases[pi], lf); periodic();
                reset(bases[bi], phases[pi], lf); interleaved();
                reset(bases[bi], phases[pi], lf); restarted();
                reset(bases[bi], phases[pi], lf); stopped();
                runs++;
            }

    printf("timer_sim %3d MHz (max shot %.1f ms): %d runs, %d checks, %d failed; "
           "one-shot error %+.3f .. %+.3f ms\n",
           (int)SIM_MHZ, 65535.0 / (double)TPM, runs, s_checks, s_fail, s_max_early, s_max_late);
    return s_fail ? 1 : 0;
}
