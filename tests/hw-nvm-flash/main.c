/**
 * hw-nvm-flash — bench test for core_nvm's flash emulation on Core.ST.L4 /
 * L4.2, Core.ST.W5 and Core.ST.H5 (its flash high-cycle data area). See
 * README.md; host_test.py drives the reflash test.
 *
 * config.json arms the 5 s watchdog in core_init(), as in a Studio project.
 * Tests, one bit each in the pass / fail masks:
 *   T1 basic     erase_all, never-written bytes read 0xFF, bounds checks,
 *                a 64-byte pattern reads back
 *   T2 compact   ~3 pages' worth of 8-byte writes to one place (several
 *                compactions, seen as slow writes); last value and the T1
 *                pattern read back
 *   T3 reset     a boot counter in NVM survives a software reset
 *   T4 reflash   (host_test.py) a token written before the Core is flashed
 *                with another image and back is still there
 *   T5 radio     (W5, BLE build) writes and compactions while advertising
 *                leave the radio interrupts flowing
 *   T6 powerloss (not BLE) a window-watchdog reset lands at a swept delay
 *                inside a 600-byte write, appends and compactions (erase
 *                included) alike: the region always reads all-old or all-new
 *                and nothing else changes
 *   T7 bonds     (W5, BLE build) a record added to the BLE bond store reaches
 *                page 127 through the flash manager, and a discard erases it
 *
 * Progress survives the resets in backup registers 0-23. The L4 and H5
 * report over USB CDC every 3 s; every Core fills g_nvm_result (read it over
 * SWD) and blinks the verdict on the status LED.
 */

#include "core.h"
#include "core_nvm.h"
#include "core_backup.h"
#include "core_watchdog.h"
#include "core_uid.h"
#include "core_rng.h"
#if defined(STM32L422xx) || defined(STM32H523xx)
#include "core_usb.h"
#define NVM_CDC 1                 /* report over USB CDC */
#endif
#if defined(STM32H523xx)
#include "ll_usb_drd.h"           /* the 15 s no-USB safety net */
#endif
#if defined(BLE_ENABLED) && BLE_ENABLED
#include "core_ble.h"
#include "nvm.h"                  /* the BLE bond store (T7) */
#define HAS_BLE 1
#else
#define HAS_BLE 0
#endif

/* ---- NVM map used by the test ---- */
#define OFF_PATTERN   0u      /* T1: 64 bytes */
#define LEN_PATTERN   64u
#define OFF_T2        256u    /* T2: 8-byte value hammered */
#define OFF_BIG       384u    /* T6: 600 bytes, pattern(seed) */
#define LEN_BIG       600u
#define OFF_TOKEN     992u    /* T4: token, ~token, big-region seed */
#define OFF_ARMED     1004u   /* 'T4AR' while waiting for a reflash */
#define OFF_COUNTER   1016u   /* T3: boot counter */
#define ARMED_MAGIC   0x52413454u

/* ---- Backup registers ---- */
#define BKP_STATE     0u      /* MAGIC << 24 | phase << 16 | iteration */
#define BKP_RESULT    1u      /* fail << 16 | pass */
#define BKP_BOOT      2u      /* T3: counter value written before the reset */
#define BKP_PL        3u      /* T6: old << 16 | new */
#define BKP_PL2       4u      /* T6: garbage << 16 | ECC recoveries */
#define BKP_T2        5u      /* T2: compactions << 16 | slowest write ms */
#define BKP_VERDICT   6u
#define BKP_SEED      7u      /* seed of what the big region holds */
#define BKP_SEED_NEW  8u      /* seed of the write in flight */
#define BKP_TOKEN     9u
#define BKP_PL_NEW    10u     /* T6: bit i = iteration i read back new */
#define BKP_CAL       11u     /* T6: WWDG armed for 30 ms fired after (this >> 16) * 10 us;
                                 fastest << 8 | slowest 600-byte write, ms */
#define BKP_PL_LOG    12u     /* T6: 12..23, two iterations each (low half first):
                                 write time in 10 us, or 0xFFFF if the reset landed
                                 inside the write */

#define MAGIC         0x4Eu
enum { PH_START = 0, PH_T3, PH_PL, PH_ARMED, PH_DONE, PH_CAL };

#define T_BASIC   (1u << 0)
#define T_COMPACT (1u << 1)
#define T_RESET   (1u << 2)
#define T_REFLASH (1u << 3)
#define T_RADIO   (1u << 4)
#define T_PLOSS   (1u << 5)
#define T_BOND    (1u << 6)

#if HAS_BLE
#define EXPECTED  (T_BASIC | T_COMPACT | T_RESET | T_RADIO | T_BOND)
#else
#define EXPECTED  (T_BASIC | T_COMPACT | T_RESET | T_PLOSS)
#endif

#define VERDICT_PASS  0x600D600Du
#define VERDICT_FAIL  0xBAD00000u   /* | index (1-6) of the first failing test */

/* T6: WWDG delays. The first PL_APPENDS iterations cut a 600-byte append
 * (~10 ms on the L4 including its read-back), the rest a compaction (a ~22 ms
 * erase, then ~20 ms of copying on the L4). Before each one the log is set up
 * (unarmed writes) so the armed write is the kind wanted. */
#if defined(STM32H523xx)
/* H5 (bench 2026-09-26, "medium", 64 MHz): a 600-byte append is 304 16-bit
 * programs plus its read-back, ~12 ms; a compaction erases a sector, checks
 * it blank, then copies up to eight 136-byte records, up to ~35 ms. The last
 * delays of each kind land after the write has finished ("new"). */
static const uint16_t PL_DELAY_US[] = {
    /* appends */      200, 1000, 2500, 4000, 5500, 7000, 8500, 10000, 11500, 14000,
    /* compactions */  300, 1200, 2500, 4000, 6000, 9000, 12000, 15000,
                       18000, 21000, 25000, 29000, 34000, 40000,
};
#else
static const uint16_t PL_DELAY_US[] = {
    /* appends */      300, 900, 1600, 2400, 3200, 4000, 5000, 6000, 7500, 9500,
    /* compactions */  1000, 8000, 16000, 21000, 22500, 24000, 26000, 29000,
                       32000, 35000, 38000, 41000, 43500, 47000,
};
#endif
#define PL_N        (sizeof(PL_DELAY_US) / sizeof(PL_DELAY_US[0]))
#define PL_APPENDS  10u

extern uint32_t _core_nvm_log_free(void);   /* core_nvm.c, not API */

/* SWD-readable copy of the result:
 *   arm-none-eabi-nm build/hw-nvm-flash.elf | grep g_nvm_result */
typedef struct {
    uint32_t magic;          /* 0x4E564D52 ("RMVN") once the report is up */
    uint32_t verdict;
    uint32_t pass, fail, expected;
    uint32_t phase;
    uint32_t t2_compactions, t2_max_ms;
    uint32_t pl_old, pl_new, pl_garbage, pl_ecc;
    uint32_t token;          /* T4 */
    uint32_t t5_irq_before, t5_irq_during, t5_irq_after, t5_max_gap_ms, t5_writes, t5_max_write_ms;
    uint32_t last_rc;        /* last failing core_nvm return code */
    uint32_t t5_burst_ms;    /* how long T5's writes took */
    uint32_t t7_store_ms, t7_clear_ms;   /* bond store: record in flash / page erased */
} nvm_result_t;

__attribute__((used)) volatile nvm_result_t g_nvm_result;

/* Host → Core over SWD (no USB on the W5): write 'R' here for a fresh run. */
__attribute__((used)) volatile uint32_t g_nvm_cmd;

static uint32_t s_pass, s_fail;
static uint8_t  s_big[LEN_BIG];

static void mark(uint32_t t, int ok)
{
    if (ok) s_pass |= t; else s_fail |= t;
}

static void save(uint32_t phase, uint32_t iter)
{
    core_backup_write(BKP_RESULT, (s_fail << 16) | s_pass);
    core_backup_write(BKP_STATE, (MAGIC << 24) | (phase << 16) | iter);
}

static void soft_reset(void)
{
    SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
    for (;;) ;
}

static uint8_t pat(uint32_t seed, uint32_t i)
{
    return (uint8_t)(seed * 131u + i * 7u + (i >> 3) * 13u + 0x5Au);
}

static void fill(uint8_t *b, uint32_t n, uint32_t seed)
{
    for (uint32_t i = 0; i < n; i++) b[i] = pat(seed, i);
}

static int matches(uint32_t off, uint32_t n, uint32_t seed)
{
    uint8_t b[64];
    for (uint32_t i = 0; i < n; i += sizeof(b)) {
        uint32_t k = (n - i) < sizeof(b) ? (n - i) : sizeof(b);
        if (core_nvm_read(off + i, b, k) != 0) return 0;
        for (uint32_t j = 0; j < k; j++) if (b[j] != pat(seed, i + j)) return 0;
    }
    return 1;
}

static uint32_t read_u32(uint32_t off)
{
    uint32_t v = 0;
    if (core_nvm_read(off, &v, 4) != 0) return 0xDEADDEADu;
    return v;
}

static int write_u32(uint32_t off, uint32_t v)
{
    int rc = core_nvm_write(off, &v, 4);
    if (rc != 0) g_nvm_result.last_rc = (uint32_t)rc;
    return rc;
}

static void pump(void)
{
    core_watchdog_feed();
#if HAS_BLE
    core_ble_process();
#endif
}

/* Cycle counter: core_millis() loses the ticks a flash erase stalls through
 * (SysTick can't run while the CPU waits on flash), DWT_CYCCNT doesn't. */
#define DEMCR       REG32(0xE000EDFCUL)
#define DWT_CTRL    REG32(0xE0001000UL)
#define DWT_CYCCNT  REG32(0xE0001004UL)
static void dwt_start(void)
{
    DEMCR |= 1UL << 24;                          /* TRCENA */
    DWT_CTRL |= 1UL;                             /* CYCCNTENA */
}
static uint32_t us_since(uint32_t c0)
{
    return (DWT_CYCCNT - c0) / (SYSCLK_HZ / 1000000UL);
}

/* FLASH_ECCR's address field (0 after reset) shows an ECC error was hit and
 * recovered (core_nvm's NMI handler clears the flag, not the address). On the
 * H5, core_nvm counts the data-area words it read with an uncorrectable
 * error (a torn write leaves some). */
static int ecc_seen(void)
{
#if defined(STM32H523xx)
    return _core_nvm_h5_ecc_errors() != 0;
#else
#if defined(STM32L422xx)
    uint32_t a = 0x08000000u + (REG32(0x40022018UL) & 0x7FFFFu);
#else
    uint32_t a = 0x08000000u + (REG32(0x40022030UL) & 0xFFFFFu);
#endif
    return a >= CORE_NVM_FLASH_ADDR && a < CORE_NVM_FLASH_ADDR + CORE_NVM_FLASH_SIZE;
#endif
}

/* ---- T6: arm the window watchdog to reset us `us` from now ----
 * The WWDG runs off PCLK1, so it keeps counting while the CPU is stalled on a
 * flash erase, and a system reset turns it off again (unlike the IWDG). Its
 * reset is not a watchdog strike for the L4's brick recovery. */
#define WWDG_CR   REG32(0x40002C00UL)
#define WWDG_CFR  REG32(0x40002C04UL)
#if defined(STM32L422xx)
#define WWDG_EN_REG   REG32(RCC_BASE + 0x58UL)   /* RCC_APB1ENR1, RM0394 §6.4.18 */
#define WDGTB_SHIFT   7u                         /* WDGTB[1:0], RM0394 §34.5.2 */
#define WDGTB_MAX     3u
#else
#define WWDG_EN_REG   REG32(RCC_BASE + 0x9CUL)   /* RCC_APB1ENR1, RM0493 */
#define WDGTB_SHIFT   11u                        /* WDGTB[2:0] */
#define WDGTB_MAX     7u
#endif

static void wwdg_arm_us(uint32_t us)
{
    SET_BITS(WWDG_EN_REG, 1UL << 11);            /* WWDGEN */
    (void)WWDG_EN_REG;
    uint32_t tb = 0, ticks = 64;
    for (; tb <= WDGTB_MAX; tb++) {
        uint64_t tick_ns = (4096ull << tb) * 1000000000ull / PCLK1_HZ;
        ticks = (uint32_t)(((uint64_t)us * 1000u + tick_ns / 2u) / tick_ns);
        if (ticks <= 63u) break;
    }
    if (tb > WDGTB_MAX) { tb = WDGTB_MAX; ticks = 63u; }
    if (ticks == 0u) ticks = 1u;
    WWDG_CFR = (tb << WDGTB_SHIFT) | 0x7Fu;      /* no window */
    WWDG_CR  = 0x80u | (0x3Fu + ticks);          /* WDGA; reset when T6 clears */
}

/* ============================================================
 * Phases
 * ============================================================ */

static void t1_t2(void)
{
    /* T1 */
    int ok = core_nvm_erase_all() == 0 && core_nvm_size() == 1024u;
    uint8_t b[64];
    for (uint32_t o = 0; ok && o < 1024u; o += sizeof(b)) {
        ok = core_nvm_read(o, b, sizeof(b)) == 0;
        for (uint32_t j = 0; ok && j < sizeof(b); j++) ok = b[j] == 0xFFu;
    }
    ok = ok && core_nvm_read_byte(1023) == 0xFF;
    ok = ok && core_nvm_write(1020, b, 8) == CORE_NVM_ERR_RANGE;
    ok = ok && core_nvm_write(0xFFFFFFFFu, b, 1) == CORE_NVM_ERR_RANGE;
    ok = ok && core_nvm_read(0, b, 1025) == CORE_NVM_ERR_RANGE;
    ok = ok && core_nvm_read_byte(1024) == -1;
    fill(b, LEN_PATTERN, 1u);
    ok = ok && core_nvm_write(OFF_PATTERN, b, LEN_PATTERN) == 0 && matches(OFF_PATTERN, LEN_PATTERN, 1u);
    ok = ok && core_nvm_write_byte(OFF_T2 - 1u, 0x42) == 1 && core_nvm_read_byte(OFF_T2 - 1u) == 0x42;
    mark(T_BASIC, ok);

    /* T2: enough 8-byte records for ~3 pages of log */
    uint32_t n = 3u * CORE_NVM_PAGE_SIZE / 16u, slow = 0, max_ms = 0;
    uint32_t slow_ms = (CORE_NVM_PAGE_SIZE == 2048u) ? 10u : 3u;
    ok = 1;
    for (uint32_t k = 1; ok && k <= n; k++) {
        uint32_t v[2] = { k, ~k };
        uint32_t c0 = DWT_CYCCNT;
        int rc = core_nvm_write(OFF_T2, v, 8);
        uint32_t dt = us_since(c0) / 1000u;
        if (rc != 0) { g_nvm_result.last_rc = (uint32_t)rc; ok = 0; }
        if (dt > max_ms) max_ms = dt;
        if (dt >= slow_ms) slow++;
        pump();
    }
    uint32_t v[2] = { 0, 0 };
    ok = ok && core_nvm_read(OFF_T2, v, 8) == 0 && v[0] == n && v[1] == ~n;
    ok = ok && matches(OFF_PATTERN, LEN_PATTERN, 1u) && core_nvm_read_byte(OFF_T2 - 1u) == 0x42;
    mark(T_COMPACT, ok && slow >= 2u);
    core_backup_write(BKP_T2, (slow << 16) | (max_ms & 0xFFFFu));
}

static void pl_log(uint32_t i, uint32_t v)
{
    uint8_t r = (uint8_t)(BKP_PL_LOG + i / 2u);
    uint32_t sh = (i & 1u) * 16u, cur = core_backup_read(r);
    core_backup_write(r, (cur & ~(0xFFFFu << sh)) | ((v & 0xFFFFu) << sh));
}

static __attribute__((unused)) uint32_t pl_log_get(uint32_t i)   /* L4 report */
{
    return (core_backup_read((uint8_t)(BKP_PL_LOG + i / 2u)) >> ((i & 1u) * 16u)) & 0xFFFFu;
}

/* An unarmed 600-byte write of the next seed (it becomes the old value). */
static void pl_step(void)
{
    uint32_t sd = core_backup_read(BKP_SEED) + 1u;
    fill(s_big, LEN_BIG, sd);
    if (core_nvm_write(OFF_BIG, s_big, LEN_BIG) == 0) core_backup_write(BKP_SEED, sd);
    core_watchdog_feed();
}

static void pl_iteration(uint32_t i)
{
    /* Aim: an append needs room for the 608-byte record; a compaction needs
     * the record not to fit. At most a few unarmed writes either way. */
    uint32_t need = (8u + LEN_BIG + CORE_NVM_PROG_UNIT - 1u) & ~(CORE_NVM_PROG_UNIT - 1u);
    for (uint32_t k = 0; k < 8u; k++) {
        uint32_t f = _core_nvm_log_free();
        if (i < PL_APPENDS ? f >= need : f < need) break;
        pl_step();
    }
    uint32_t seed_new = core_backup_read(BKP_SEED) + 1u;
    core_backup_write(BKP_SEED_NEW, seed_new);
    save(PH_PL, i);
    fill(s_big, LEN_BIG, seed_new);
    pl_log(i, 0xFFFFu);
    core_watchdog_feed();
    LED_ON();
    wwdg_arm_us(PL_DELAY_US[i]);
    uint32_t c0 = DWT_CYCCNT;
    (void)core_nvm_write(OFF_BIG, s_big, LEN_BIG);
    uint32_t w = us_since(c0) / 10u;
    pl_log(i, w > 0xFFFEu ? 0xFFFEu : w);
    for (;;)                                     /* until the WWDG resets us */
        if (us_since(c0) > 500000u) soft_reset();
}

static void arm_t4(void)
{
    uint32_t seed = core_backup_read(BKP_SEED);
    uint32_t token = core_uid_hash() ^ (DWT_CYCCNT * 2654435761u) ^ read_u32(OFF_COUNTER);
#if !HAS_BLE
    core_rng_init();              /* the BLE stack owns the RNG in BLE builds */
    token ^= core_rng_read();
#endif
    uint32_t t[3] = { token, ~token, seed };
    int ok = core_nvm_write(OFF_TOKEN, t, sizeof(t)) == 0 && write_u32(OFF_ARMED, ARMED_MAGIC) == 0;
    if (ok) core_backup_write(BKP_TOKEN, token);
    g_nvm_result.token = token;
}

/* Boot with the armed marker set: this image was flashed back after another. */
static void check_t4(void)
{
    uint32_t t[3] = { 0, 0, 0 };
    int ok = core_nvm_read(OFF_TOKEN, t, sizeof(t)) == 0 && t[1] == ~t[0];
    ok = ok && matches(OFF_PATTERN, LEN_PATTERN, 1u) && matches(OFF_BIG, LEN_BIG, t[2]);
    ok = ok && read_u32(OFF_COUNTER) != 0xFFFFFFFFu;
    mark(T_REFLASH, ok);
    g_nvm_result.token = t[0];
    core_backup_write(BKP_TOKEN, t[0]);
    (void)write_u32(OFF_ARMED, 0u);
}

#if HAS_BLE
extern void (*radio_callback)(void);
static void (*orig_radio)(void);
static volatile uint32_t radio_irqs, radio_last_ms, radio_max_gap;
static void count_radio(void)
{
    uint32_t now = core_millis();
    if (radio_irqs && now - radio_last_ms > radio_max_gap) radio_max_gap = now - radio_last_ms;
    radio_last_ms = now;
    radio_irqs++;
    orig_radio();
}

static void run_for_ms(uint32_t ms)
{
    for (uint32_t t0 = core_millis(); core_millis() - t0 < ms; ) pump();
}

/* T5: advertising (150 ms interval) keeps its radio interrupts through writes
 * and compactions, which go through the flash manager's time windows. */
static void t5_radio(void)
{
    orig_radio = radio_callback;
    radio_callback = count_radio;
    run_for_ms(2000);
    uint32_t before = radio_irqs;
    radio_max_gap = 0;

    uint32_t n = 2u * CORE_NVM_PAGE_SIZE / 16u + 50u, max_ms = 0;
    int ok = 1;
    uint32_t t_start = core_millis();
    for (uint32_t k = 1; ok && k <= n; k++) {
        uint32_t v[2] = { 0xB1E00000u | k, ~k };
        uint32_t t0 = core_millis();
        int rc = core_nvm_write(OFF_T2, v, 8);
        if (core_millis() - t0 > max_ms) max_ms = core_millis() - t0;
        if (rc != 0) { g_nvm_result.last_rc = (uint32_t)rc; ok = 0; }
        pump();
    }
    uint32_t during = radio_irqs - before, dur = core_millis() - t_start;
    uint32_t gap = radio_max_gap;
    uint32_t mid = radio_irqs;
    run_for_ms(2000);
    uint32_t after = radio_irqs - mid;

    g_nvm_result.t5_irq_before = before;
    g_nvm_result.t5_irq_during = during;
    g_nvm_result.t5_irq_after  = after;
    g_nvm_result.t5_max_gap_ms = gap;
    g_nvm_result.t5_writes     = n;
    g_nvm_result.t5_max_write_ms = max_ms;
    g_nvm_result.t5_burst_ms   = dur;
    /* ~150 ms advertising: a gap under 3 intervals, and at least half the
     * baseline rate during the burst. */
    mark(T_RADIO, ok && before > 0u && after > 0u && gap < 450u &&
                  during * 2000u >= before * dur / 2u);
}

/* T7: the BLE bond store (sdk/ble/nvm_stub.c, page 127) flushes through the
 * flash manager asynchronously. Add a record the way the stack does, run the
 * BLE tasks until it is in flash, then discard the store and wait for the page
 * to read erased. (This clears any real bonds on the board.) */
static void icache_flush(void)
{
    if (!(REG32(0x40030400UL) & 1UL)) return;    /* ICACHE_CR.EN */
    REG32(0x4003040CUL) = 1UL << 1;              /* FCR: CBSYENDF */
    REG32(0x40030400UL) |= 1UL << 1;             /* CACHEINV */
    while (!(REG32(0x40030404UL) & (1UL << 1)))  /* SR: BSYENDF */
        ;
}

static int bond_page_has(const uint8_t *rec, uint16_t n)
{
    icache_flush();                              /* flash reads go through it */
    const volatile uint8_t *pg = (const volatile uint8_t *)0x080FE000UL;
    for (uint32_t pos = 0; pos + 4u + n <= 8192u; ) {
        if (pg[pos] != 0xBEu) return 0;          /* NVM_RECORD_MAGIC */
        uint16_t size = (uint16_t)(pg[pos + 2] | (pg[pos + 3] << 8));
        if (pg[pos + 1] == 0x5Au && size == n) {
            uint32_t k = 0;
            while (k < n && pg[pos + 4u + k] == rec[k]) k++;
            if (k == n) return 1;
        }
        pos += (4u + size + 15u) & ~15u;
    }
    return 0;
}

static int bond_page_erased(void)
{
    icache_flush();
    const volatile uint32_t *pg = (const volatile uint32_t *)0x080FE000UL;
    for (uint32_t i = 0; i < 8192u / 4u; i++) if (pg[i] != 0xFFFFFFFFu) return 0;
    return 1;
}

static void t7_bond(void)
{
    uint8_t rec[20];
    for (uint32_t i = 0; i < sizeof(rec); i++) rec[i] = (uint8_t)(0xA0u + i * 3u);
    int ok = NVM_Add(0x5A, rec, sizeof(rec), 0, 0) == 0;
    uint32_t t0 = core_millis();
    while (ok && !bond_page_has(rec, sizeof(rec)) && core_millis() - t0 < 3000u) pump();
    ok = ok && bond_page_has(rec, sizeof(rec));
    g_nvm_result.t7_store_ms = core_millis() - t0;
    NVM_Discard(0);
    t0 = core_millis();
    while (ok && !bond_page_erased() && core_millis() - t0 < 3000u) pump();
    ok = ok && bond_page_erased();
    g_nvm_result.t7_clear_ms = core_millis() - t0;
    mark(T_BOND, ok);
}
#endif

static uint32_t first_failure(uint32_t missing)
{
    for (uint32_t i = 0; i < 7; i++)
        if (missing & (1u << i)) return i + 1u;
    return 0;
}

#if defined(NVM_CDC)
static void print_report(uint32_t verdict, uint32_t phase)
{
    static const char *const names[6] = {
        "T1 basic", "T2 compaction", "T3 reset", "T4 reflash", "T5 radio", "T6 power loss"
    };
    uint32_t t2 = core_backup_read(BKP_T2), pl = core_backup_read(BKP_PL), pl2 = core_backup_read(BKP_PL2);
    core_usb_printf("\r\n[hw-nvm-flash] %s  (pass=0x%02lx fail=0x%02lx expected=0x%02lx)\r\n",
                    verdict == VERDICT_PASS ? "PASS" : "FAIL",
                    (unsigned long)s_pass, (unsigned long)s_fail, (unsigned long)EXPECTED);
    for (uint32_t i = 0; i < 6; i++) {
        uint32_t b = 1u << i;
        core_usb_printf("  %-16s %s\r\n", names[i],
                        (s_pass & b) ? "PASS" : (s_fail & b) ? "FAIL" : "not run");
    }
    core_usb_printf("  T2: %lu compactions, slowest write %lu ms\r\n",
                    (unsigned long)(t2 >> 16), (unsigned long)(t2 & 0xFFFFu));
    uint32_t cal = core_backup_read(BKP_CAL);
    core_usb_printf("  T6: 600-byte write %lu-%lu ms; WWDG set for 30000 us fired at %lu us\r\n",
                    (unsigned long)((cal >> 8) & 0xFFu), (unsigned long)(cal & 0xFFu),
                    (unsigned long)((cal >> 16) * 10u));
    core_usb_printf("  T6: %lu resets: %lu old, %lu new, %lu garbage; ECC errors recovered: %lu\r\n",
                    (unsigned long)((pl >> 16) + (pl & 0xFFFFu) + (pl2 >> 16)),
                    (unsigned long)(pl >> 16), (unsigned long)(pl & 0xFFFFu),
                    (unsigned long)(pl2 >> 16), (unsigned long)(pl2 & 0xFFFFu));
    uint32_t newm = core_backup_read(BKP_PL_NEW), done = (pl >> 16) + (pl & 0xFFFFu) + (pl2 >> 16);
    for (uint32_t i = 0; i < PL_N && i < done; i++) {
        uint32_t lg = pl_log_get(i);
        char took[24];
        if (lg == 0xFFFFu) snprintf(took, sizeof(took), "inside the write");
        else               snprintf(took, sizeof(took), "write took %lu us", (unsigned long)lg * 10u);
        core_usb_printf("    #%-2lu %-10s reset at %5u us: %-18s -> %s\r\n", (unsigned long)i,
                        i < PL_APPENDS ? "append" : "compaction", PL_DELAY_US[i], took,
                        (newm >> i) & 1u ? "new" : "old");
    }
    core_usb_printf("  boot counter %lu; token 0x%08lx (%s)\r\n",
                    (unsigned long)read_u32(OFF_COUNTER), (unsigned long)g_nvm_result.token,
                    phase == PH_ARMED ? "armed: reflash another image, then this one"
                                      : "checked after reflash");
    if (g_nvm_result.last_rc) core_usb_printf("  last core_nvm error %ld\r\n", (long)(int32_t)g_nvm_result.last_rc);
#if defined(STM32H523xx)
    const core_nvm_h5_prov_t *pv = _core_nvm_h5_prov();
    core_usb_printf("  H5 data area: FLASH_EDATA2R_CUR 0x%08lx; first-boot record %s state %ld, "
                    "%lu option change(s) in %lu boot(s) since power-on\r\n",
                    (unsigned long)REG32(FLASH_BASE + 0x1F0UL), pv ? "present," : "none",
                    pv ? (long)pv->state : 0L, pv ? (unsigned long)pv->writes : 0UL,
                    pv ? (unsigned long)pv->boots : 0UL);
#endif
}
#endif

int main(void)
{
    core_init();                  /* arms the 5 s watchdog (config.json) */
    core_led_init();
    dwt_start();
#if HAS_BLE
    core_ble_init();
    core_ble_advertise("nvm-test");
#endif

    uint32_t state = core_backup_read(BKP_STATE);
    uint32_t phase = ((state >> 24) == MAGIC) ? ((state >> 16) & 0xFFu) : PH_START;
    uint32_t iter  = state & 0xFFFFu;
    s_pass = core_backup_read(BKP_RESULT) & 0xFFFFu;
    s_fail = core_backup_read(BKP_RESULT) >> 16;

    /* A reflash can wipe nothing in NVM but could, in principle, lose the
     * backup registers: the armed marker in NVM decides. */
    int armed = read_u32(OFF_ARMED) == ARMED_MAGIC;
    int ecc = ecc_seen();

    if (armed && phase != PH_PL) {
        check_t4();
        phase = PH_DONE;
    } else switch (phase) {
    case PH_T3: {
        uint32_t c = read_u32(OFF_COUNTER);
        mark(T_RESET, c == core_backup_read(BKP_BOOT) && matches(OFF_PATTERN, LEN_PATTERN, 1u) &&
                      matches(OFF_BIG, LEN_BIG, core_backup_read(BKP_SEED)));
        (void)write_u32(OFF_COUNTER, c + 1u);
#if HAS_BLE
        t5_radio();
        t7_bond();
        phase = PH_ARMED;
        arm_t4();
#else
        /* Reference timings: three 600-byte writes (appends, and at least
         * one compaction), then the WWDG calibrated against DWT. */
        uint32_t lo = 0xFFFFFFFFu, hi = 0;
        for (uint32_t k = 0; k < 3u; k++) {
            uint32_t sd = core_backup_read(BKP_SEED) + 1u;
            fill(s_big, LEN_BIG, sd);
            uint32_t c0 = DWT_CYCCNT;
            if (core_nvm_write(OFF_BIG, s_big, LEN_BIG) == 0) core_backup_write(BKP_SEED, sd);
            uint32_t w = us_since(c0) / 1000u;
            if (w < lo) lo = w;
            if (w > hi) hi = w;
            core_watchdog_feed();
        }
        uint32_t wt = ((lo > 255u ? 255u : lo) << 8) | (hi > 255u ? 255u : hi);
        save(PH_CAL, 0);
        core_backup_write(BKP_CAL, 0xFFFF0000u | wt);
        wwdg_arm_us(30000u);
        for (uint32_t c0 = DWT_CYCCNT;;) core_backup_write(BKP_CAL, ((us_since(c0) / 10u) << 16) | wt);
#endif
        break;
    }
    case PH_CAL:
        core_backup_write(BKP_PL, 0);
        core_backup_write(BKP_PL2, 0);
        core_backup_write(BKP_PL_NEW, 0);
        pl_iteration(0);          /* resets */
        break;
    case PH_PL: {
        /* The reset of iteration `iter` just happened. */
        uint32_t so = core_backup_read(BKP_SEED), sn = core_backup_read(BKP_SEED_NEW);
        uint32_t pl = core_backup_read(BKP_PL), pl2 = core_backup_read(BKP_PL2);
        int others = matches(OFF_PATTERN, LEN_PATTERN, 1u) && read_u32(OFF_COUNTER) == core_backup_read(BKP_BOOT) + 1u;
        if (others && matches(OFF_BIG, LEN_BIG, so))      pl += 1u << 16;
        else if (others && matches(OFF_BIG, LEN_BIG, sn)) {
            pl += 1u;
            core_backup_write(BKP_SEED, sn);
            core_backup_write(BKP_PL_NEW, core_backup_read(BKP_PL_NEW) | (1u << iter));
        }
        else                                              pl2 += 1u << 16;
        if (ecc) pl2 += 1u;
        core_backup_write(BKP_PL, pl);
        core_backup_write(BKP_PL2, pl2);
        if (iter + 1u < PL_N) pl_iteration(iter + 1u);   /* resets */

        /* Done: everything consistent, and a write still works. */
        fill(s_big, LEN_BIG, core_backup_read(BKP_SEED) + 100u);
        int ok = (pl2 >> 16) == 0u && (pl >> 16) + (pl & 0xFFFFu) == PL_N &&
                 core_nvm_write(OFF_BIG, s_big, LEN_BIG) == 0 &&
                 matches(OFF_BIG, LEN_BIG, core_backup_read(BKP_SEED) + 100u);
        core_backup_write(BKP_SEED, core_backup_read(BKP_SEED) + 100u);
        mark(T_PLOSS, ok);
        phase = PH_ARMED;
        arm_t4();
        break;
    }
    case PH_ARMED:
    case PH_DONE:
        break;                    /* show the result again */
    default: {                    /* PH_START or garbage: run it all */
        s_pass = s_fail = 0;
        core_backup_write(BKP_PL, 0);
        core_backup_write(BKP_PL2, 0);
        core_backup_write(BKP_PL_NEW, 0);
        t1_t2();
        core_backup_write(BKP_SEED, 1u);
        fill(s_big, LEN_BIG, 1u);
        if (core_nvm_write(OFF_BIG, s_big, LEN_BIG) != 0) mark(T_RESET, 0);
        uint32_t c = read_u32(OFF_COUNTER);
        if (c == 0xFFFFFFFFu) c = 0;
        (void)write_u32(OFF_COUNTER, c + 1u);
        core_backup_write(BKP_BOOT, c + 1u);
        save(PH_T3, 0);
        soft_reset();
    }
    }

    uint32_t missing = (EXPECTED & ~s_pass & ~T_REFLASH) | s_fail;
    uint32_t verdict = missing ? (VERDICT_FAIL | first_failure(missing)) : VERDICT_PASS;
    core_backup_write(BKP_VERDICT, verdict);
    save(phase, 0);

    uint32_t t2 = core_backup_read(BKP_T2), pl = core_backup_read(BKP_PL), pl2 = core_backup_read(BKP_PL2);
    g_nvm_result.verdict  = verdict;
    g_nvm_result.pass     = s_pass;
    g_nvm_result.fail     = s_fail;
    g_nvm_result.expected = EXPECTED;
    g_nvm_result.phase    = phase;
    g_nvm_result.t2_compactions = t2 >> 16;
    g_nvm_result.t2_max_ms      = t2 & 0xFFFFu;
    g_nvm_result.pl_old = pl >> 16;
    g_nvm_result.pl_new = pl & 0xFFFFu;
    g_nvm_result.pl_garbage = pl2 >> 16;
    g_nvm_result.pl_ecc = pl2 & 0xFFFFu;
    if (!g_nvm_result.token) g_nvm_result.token = core_backup_read(BKP_TOKEN);
    g_nvm_result.magic = 0x4E564D52u;

    /* PASS: slow 1 s on / 1 s off. FAIL: N quick blinks (N = first failing
     * test), then a 1.5 s pause. 'R' over USB (L4) starts a fresh run. */
    uint32_t last_print = 0, blink_t0 = core_millis();
    while (1) {
        pump();
        uint32_t cmd = g_nvm_cmd;
#if defined(NVM_CDC)
        uint8_t ch;
        if (core_usb_try_read(&ch)) cmd = ch;
#endif
#if defined(STM32H523xx)
        /* Bench safety net (as in hw-h5-bringup): no USB address 15 s after
         * boot means USB never came up; reset (BOOT0 high: into the ROM). */
        if ((USB_DADDR & USB_DADDR_ADD_MASK) == 0 && core_millis() > 15000u) soft_reset();
#endif
        if (cmd == 'R') {
            (void)write_u32(OFF_ARMED, 0u);
            save(PH_START, 0);
            soft_reset();
        }
#if defined(NVM_CDC)
        if ((uint32_t)(core_millis() - last_print) >= 3000u) {
            last_print = core_millis();
            print_report(verdict, phase);
        }
#else
        (void)last_print;
#endif
        uint32_t t = core_millis() - blink_t0;
        if (verdict == VERDICT_PASS) {
            if (t < 1000u) LED_ON(); else LED_OFF();
            if (t >= 2000u) blink_t0 = core_millis();
        } else {
            uint32_t n = verdict & 0xFu, period = n * 400u + 1500u;
            if (t < n * 400u && (t % 400u) < 150u) LED_ON(); else LED_OFF();
            if (t >= period) blink_t0 = core_millis();
        }
    }
}
