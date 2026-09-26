/**
 * hw-nvm-eeprom — bench test for core_nvm on the Core.ST.L0.1's data EEPROM
 * (STM32L011, 512 B at 0x08080000), written for the 2026-09 fixes: the byte
 * helpers (tiles#284: read_byte used to return -1 for every byte) and the
 * FLASH_SR checks (tiles#291).
 *
 *   make CLOCK=low   (or max), then
 *   probe-rs download --chip STM32L011E4 --binary-format elf build/hw-nvm-eeprom.elf
 *   probe-rs reset --chip STM32L011E4; wait ~3 s; read `g_nvm` (nm) over SWD.
 *   PASS: magic 0xEE9D0C0D, verdict 0x600D600D, pass 0x3F.
 *
 * Only the scratch half of the EEPROM is touched (offsets 0x100-0x1FF): the
 * low bytes hold application settings (firmware/sense-adc-6 keeps its I2C
 * address at 0-3 and its acquisition settings at 4-8). core_nvm_erase_all()
 * is deliberately not called, because it would clear them.
 *
 *   T1 range     out-of-range reads / writes (incl. a wrapping offset) return
 *                CORE_NVM_ERR_RANGE; the byte helpers return -1
 *   T2 erased    CORE_NVM_ERASED is 0x00, a never-written area reads it, and
 *                a word written with zeros reads it back
 *   T3 buffer    a 32-byte pattern (different every run) reads back through
 *                core_nvm_read and straight from memory; PECR is relocked and
 *                the voltage range is back where it was
 *   T4 bytes     write_byte / read_byte round-trip 0xA5, 0x00 and 0x5A
 *   T5 sr        a write the hardware refuses (SIZERR, injected by leaving
 *                PECR.ERASE set) returns CORE_NVM_ERR_FLASH even though the
 *                byte already holds the value (only FLASH_SR can tell), and
 *                the error flags are cleared afterwards
 *   T6 reset     a boot counter in EEPROM survives a software reset
 *
 * Boot A bumps the counter, notes the expected value in backup register 4
 * and resets; boot B checks it, runs T1-T5 and reports. Resetting the Core
 * after the report runs it again.
 */

#include "core.h"
#include "core_nvm.h"
#include "core_backup.h"
#include "core_watchdog.h"

#if !defined(STM32L011xx)
#error "hw-nvm-eeprom is for Core.ST.L0.1 (true data EEPROM); hw-nvm-flash covers L4 / W5"
#endif

#ifndef REG32
#define REG32(a)       (*(volatile uint32_t *)(a))
#endif
/* RM0377 §3.7: FLASH 0x40022000; PWR 0x40007000 */
#define FLASH_PECR     REG32(0x40022004UL)
#define FLASH_PEKEYR   REG32(0x4002200CUL)
#define FLASH_SR       REG32(0x40022018UL)
#define PWR_CR         REG32(0x40007000UL)
#define PECR_PELOCK    (1UL << 0)
#define PECR_DATA      (1UL << 4)
#define PECR_ERASE     (1UL << 9)
#define SR_SIZERR      (1UL << 10)
#define SR_ERRS        ((1UL << 8) | (1UL << 9) | (1UL << 10) | (1UL << 16) | (1UL << 17))
#define SCB_AIRCR      REG32(0xE000ED0CUL)

/* ---- EEPROM scratch layout (offsets) ---- */
#define OFF_BLANK      0x100u   /* 0x100-0x17F: never written by this test */
#define LEN_BLANK      0x80u
#define OFF_STATE      0x180u   /* 'E' 'T' phase count */
#define OFF_PATTERN    0x1A0u   /* 32 bytes */
#define OFF_BYTE       0x1C0u
#define OFF_ZERO       0x1C4u   /* one word */
#define OFF_SR         0x1C8u

#define BKP_EXPECT     4u       /* boot counter expected after the reset */

#define T_RANGE   (1u << 0)
#define T_ERASED  (1u << 1)
#define T_BUFFER  (1u << 2)
#define T_BYTES   (1u << 3)
#define T_SR      (1u << 4)
#define T_RESET   (1u << 5)
#define EXPECTED  0x3Fu

#define VERDICT_PASS  0x600D600Du
#define VERDICT_FAIL  0xBAD00000u   /* | index (1-6) of the first failing test */

/* SWD-readable result: arm-none-eabi-nm build/hw-nvm-eeprom.elf | grep g_nvm */
typedef struct {
    uint32_t magic;           /* 0xEE9D0C0D once final */
    uint32_t verdict;
    uint32_t pass, fail;
    uint32_t boot_expect;     /* T6: counter noted before the reset */
    uint32_t boot_read;       /* T6: counter read after it */
    int32_t  rc_sizerr;       /* T5: core_nvm_write with PECR.ERASE set (-2) */
    uint32_t sr_after;        /* T5: FLASH_SR error bits after that write (0) */
    int32_t  rc_after;        /* T5: the same write once ERASE is cleared (0) */
    uint32_t vos_before, vos_after;   /* T3: PWR_CR.VOS around a write */
    uint32_t pecr_after;      /* T3: FLASH_PECR after a write (PELOCK set) */
    uint32_t write_ms;        /* T3: 32 bytes, ~3.2-6.4 ms each */
    int32_t  rb_a5, rb_00, rb_5a;     /* T4: read_byte results */
} nvm_eeprom_result_t;

__attribute__((used)) volatile nvm_eeprom_result_t g_nvm;

static uint32_t s_pass, s_fail;

static void mark(uint32_t t, int ok)
{
    if (ok) s_pass |= t; else s_fail |= t;
}

static void wait_ms(uint32_t ms)
{
    while (ms) {
        uint32_t d = ms > 100u ? 100u : ms;
        core_watchdog_feed();
        core_delay_ms(d);
        ms -= d;
    }
}

static void pecr_unlock(void)
{
    if (FLASH_PECR & PECR_PELOCK) {
        FLASH_PEKEYR = 0x89ABCDEFUL;          /* RM0377 §3.3.4 PEKEY1 / PEKEY2 */
        FLASH_PEKEYR = 0x02030405UL;
    }
}

static void t1_range(void)
{
    uint8_t b[2] = { 1, 2 };
    int ok = 1;
    ok &= core_nvm_read(CORE_NVM_SIZE, b, 1) == CORE_NVM_ERR_RANGE;
    ok &= core_nvm_read(CORE_NVM_SIZE - 1u, b, 2) == CORE_NVM_ERR_RANGE;
    ok &= core_nvm_read(0xFFFFFFFFu, b, 2) == CORE_NVM_ERR_RANGE;      /* wraps */
    ok &= core_nvm_write(CORE_NVM_SIZE, b, 1) == CORE_NVM_ERR_RANGE;
    ok &= core_nvm_write(0xFFFFFFFFu, b, 2) == CORE_NVM_ERR_RANGE;
    ok &= core_nvm_read_byte(CORE_NVM_SIZE) == -1;
    ok &= core_nvm_write_byte(CORE_NVM_SIZE, 7) == -1;
    ok &= core_nvm_size() == 512u;
    mark(T_RANGE, ok);
}

static void t2_erased(void)
{
    int ok = (CORE_NVM_ERASED == 0x00u);
    uint8_t b[LEN_BLANK];
    ok &= core_nvm_read(OFF_BLANK, b, LEN_BLANK) == CORE_NVM_OK;
    for (uint32_t i = 0; i < LEN_BLANK; i++) ok &= b[i] == CORE_NVM_ERASED;

    const uint8_t ones[4] = { 0x11, 0x22, 0x33, 0x44 };
    const uint8_t zero[4] = { 0, 0, 0, 0 };
    ok &= core_nvm_write(OFF_ZERO, ones, 4) == CORE_NVM_OK;
    ok &= REG32(CORE_NVM_BASE + OFF_ZERO) == 0x44332211u;
    ok &= core_nvm_write(OFF_ZERO, zero, 4) == CORE_NVM_OK;
    for (uint32_t i = 0; i < 4; i++) ok &= core_nvm_read_byte(OFF_ZERO + i) == (int)CORE_NVM_ERASED;
    mark(T_ERASED, ok);
}

static void t3_buffer(uint8_t seed)
{
    uint8_t w[32], r[32];
    for (uint32_t i = 0; i < 32; i++) w[i] = (uint8_t)(seed * 37u + i * 11u + 1u);

    g_nvm.vos_before = (PWR_CR >> 11) & 3u;
    uint32_t t0 = core_millis();
    int ok = core_nvm_write(OFF_PATTERN, w, 32) == CORE_NVM_OK;
    g_nvm.write_ms = core_millis() - t0;
    g_nvm.vos_after = (PWR_CR >> 11) & 3u;
    g_nvm.pecr_after = FLASH_PECR;

    ok &= core_nvm_read(OFF_PATTERN, r, 32) == CORE_NVM_OK;
    const volatile uint8_t *mem = (const volatile uint8_t *)(CORE_NVM_BASE + OFF_PATTERN);
    for (uint32_t i = 0; i < 32; i++) ok &= r[i] == w[i] && mem[i] == w[i];
    ok &= g_nvm.vos_after == g_nvm.vos_before;
    ok &= (g_nvm.pecr_after & PECR_PELOCK) != 0u;
    mark(T_BUFFER, ok);
}

static void t4_bytes(void)
{
    int ok = core_nvm_write_byte(OFF_BYTE, 0xA5) == 1;
    g_nvm.rb_a5 = core_nvm_read_byte(OFF_BYTE);
    ok &= core_nvm_write_byte(OFF_BYTE, 0x00) == 1;
    g_nvm.rb_00 = core_nvm_read_byte(OFF_BYTE);
    ok &= core_nvm_write_byte(OFF_BYTE, 0x5A) == 1;
    g_nvm.rb_5a = core_nvm_read_byte(OFF_BYTE);
    ok &= g_nvm.rb_a5 == 0xA5 && g_nvm.rb_00 == 0 && g_nvm.rb_5a == 0x5A;
    mark(T_BYTES, ok);
}

/* PECR.ERASE turns a store into an erase request, and a data-EEPROM erase is
 * word-only (RM0377 §3.3.4 "Erase data EEPROM": SIZERR otherwise). The byte
 * already holds the value written, so the read-back can't see the failure;
 * only the FLASH_SR check in core_nvm_write can. */
static void t5_sr(void)
{
    const uint8_t v = 0x3C;
    int ok = core_nvm_write(OFF_SR, &v, 1) == CORE_NVM_OK;

    pecr_unlock();
    FLASH_PECR |= PECR_ERASE | PECR_DATA;
    g_nvm.rc_sizerr = core_nvm_write(OFF_SR, &v, 1);
    g_nvm.sr_after = FLASH_SR & SR_ERRS;

    pecr_unlock();
    FLASH_PECR &= ~(PECR_ERASE | PECR_DATA);
    FLASH_PECR |= PECR_PELOCK;

    g_nvm.rc_after = core_nvm_write(OFF_SR, &v, 1);
    ok &= g_nvm.rc_sizerr == CORE_NVM_ERR_FLASH;
    ok &= g_nvm.sr_after == 0u;
    ok &= g_nvm.rc_after == CORE_NVM_OK;
    ok &= core_nvm_read_byte(OFF_SR) == v;
    mark(T_SR, ok);
}

int main(void)
{
    core_init();                  /* arms the 5 s watchdog (config.json) */
    core_led_init();
    wait_ms(200);

    uint8_t st[4];
    core_nvm_read(OFF_STATE, st, 4);
    int valid = st[0] == 'E' && st[1] == 'T';
    uint8_t count = valid ? st[3] : 0u;

    if (!valid || st[2] != 1u) {
        /* Boot A: bump the counter, note what boot B should read, reset. */
        uint8_t next = (uint8_t)(count + 1u) ? (uint8_t)(count + 1u) : 1u;
        uint8_t n[4] = { 'E', 'T', 1u, next };
        core_nvm_write(OFF_STATE, n, 4);
        core_backup_write(BKP_EXPECT, n[3]);
        wait_ms(50);
        SCB_AIRCR = 0x05FA0004UL;                 /* SYSRESETREQ */
        while (1) ;
    }

    /* Boot B */
    g_nvm.boot_expect = core_backup_read(BKP_EXPECT);
    g_nvm.boot_read = count;
    {
        uint8_t done[4] = { 'E', 'T', 2u, count };
        int ok = core_nvm_write(OFF_STATE, done, 4) == CORE_NVM_OK;
        mark(T_RESET, ok && count == (uint8_t)g_nvm.boot_expect && count != 0u);
    }

    t1_range();
    t2_erased();
    t3_buffer(count);
    t4_bytes();
    t5_sr();

    uint32_t verdict = VERDICT_PASS;
    if ((s_pass & EXPECTED) != EXPECTED || s_fail) {
        uint32_t f = s_fail | (EXPECTED & ~s_pass), n = 1;
        while (!(f & 1u)) { f >>= 1; n++; }
        verdict = VERDICT_FAIL | n;
    }
    g_nvm.pass = s_pass;
    g_nvm.fail = s_fail;
    g_nvm.verdict = verdict;
    g_nvm.magic = 0xEE9D0C0Du;

    /* PASS: slow 1 s on / 1 s off. FAIL: N quick blinks, a 1.5 s pause. */
    while (1) {
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
