/**
 * hw-spi-loopback — bench test for core_spi (portal task #285) on
 * Core.ST.L4.1 / L4.2 and Core.ST.W5, with one jumper from MOSI to MISO.
 * See README.md for the wiring, flashing and reading the result.
 *
 * Tests, one bit each in the pass/fail masks:
 *   T0 jumper   SPI parked, MOSI driven as a GPIO: MISO follows it (both
 *               levels, against the opposite pull). If not, the report says
 *               "loopback jumper missing" and T1-T6 are skipped (not failed).
 *   T1 bytes    single bytes 0x00 0xFF 0xA5 0x5A and 0..255, plus a 256-byte
 *               buffer, at /256 and the fastest legal SCK (polled and DMA)
 *   T2 buffers  1 2 3 63 64 65 255 256 1024 4096 bytes, polled and DMA,
 *               rx == tx; in place, NULL rx and NULL tx (fill 0xFF) too
 *   T3 modes    modes 0-3 and LSB-first: data comes back and SCK idles at CPOL
 *   T4 speed    4096-byte throughput at the fastest legal SCK (LB_SCK_MAX_HZ),
 *               polled and DMA
 *   T5 timeout  a transfer that cannot complete returns HAL_TIMEOUT within its
 *               stall budget (polled, blocking DMA, async DMA + wait; on the
 *               L4 also with the SPI clock gated), and the next one works
 *   T6 pal      core_tiles_pal() spi_transfer: command + 3-byte address + data
 *               read under ONE chip-select assertion (EXTI counts 2 CS edges)
 *   T7 2 tiles  (config-2tiles.json, `make TWO_TILES=1`) two SPI1 tiles with
 *               their own cs_pad: every bridge call asserts only its tile's
 *               pad (EXTI counts falling edges on both), an unknown cs selects
 *               nothing and returns -1, and both pads are high after init
 *
 * The result goes to USB CDC every 3 s on the L4 (send 'R' to run again) and
 * to g_spi_lb / g_spi_lb_log in RAM on every Core (read them over SWD, or
 * run read_swd.py; write 'R' to g_spi_lb_cmd to run again).
 */

#include "core.h"
#include "core_spi.h"
#include "core_pad.h"
#include "hal_exti.h"
#include "core_tiles.h"
#include "core_watchdog.h"
#if defined(STM32L422xx)
#include "core_usb.h"
#endif
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ---- Per-Core wiring (the Makefile passes LB_* for the L4.2 and W5) ---- */
#ifndef LB_MOSI
#define LB_MOSI  2      /* Core.ST.L4.1: PA7  SPI1.MOSI */
#endif
#ifndef LB_MISO
#define LB_MISO  8      /* Core.ST.L4.1: PB4  SPI1.MISO */
#endif
#ifndef LB_CS
#define LB_CS    9      /* Core.ST.L4.1: PA4  (GPIO chip select) */
#endif
#ifndef LB_SCK
#define LB_SCK   3      /* Core.ST.L4.1: PA1  SPI1.CLK */
#endif
/* LB_CS2 (Makefile, TWO_TILES=1): the second tile's chip select, pad 4 (PB6).
 * Tile 0 (instance 0) then selects on LB_CS, tile 1 (instance 1) on LB_CS2. */

#if defined(STM32WBA55xx)
#define SPI_H     (&core_spi3)
#define SPI_AF    6u    /* PB8/PB9/PA0 → SPI3 (Core-ST-W5-b.json) */
#else
#define SPI_H     (&core_spi1)
#define SPI_AF    5u    /* PA7/PB4/PA1 → SPI1 */
#endif

#define T_JUMPER  (1u << 0)
#define T_BYTES   (1u << 1)
#define T_BUFS    (1u << 2)
#define T_MODES   (1u << 3)
#define T_SPEED   (1u << 4)
#define T_TIMEOUT (1u << 5)
#define T_PAL     (1u << 6)
#define T_TWO     (1u << 7)
#ifdef LB_CS2
#define EXPECTED  0xFFu
#define N_TESTS   8
#else
#define EXPECTED  0x7Fu
#define N_TESTS   7
#endif

#define VERDICT_PASS  0x600D600Du
#define VERDICT_SKIP  0x5C1B0000u   /* jumper missing: nothing else was run */
#define VERDICT_FAIL  0xBAD00000u   /* | index (0-7) of the first failing test */

/* SWD-readable result:  arm-none-eabi-nm build/hw-spi-loopback.elf | grep g_spi_lb */
typedef struct {
    uint32_t magic;          /* 0x5B1100B5 once the run is done */
    uint32_t verdict;
    uint32_t pass, fail, expected;
    uint32_t runs;
    uint32_t sck_fast_hz, sck_slow_hz;
    uint32_t poll_Bps, dma_Bps;           /* T4 */
    uint32_t t5_poll_us, t5_dma_us, t5_async_us, t5_gated_us, t5_bound_us;
    uint32_t cs_edges;                    /* T6: CS edges in one PAL transaction */
    uint32_t first_err;                   /* test << 24 | case << 12 | size */
    uint32_t t7_edges_a, t7_edges_b;      /* T7: falling edges on LB_CS / LB_CS2 */
    uint32_t t7_wrong;                    /* T7: operations that moved the wrong pad */
} spi_lb_result_t;

__attribute__((used)) volatile spi_lb_result_t g_spi_lb;
__attribute__((used)) char g_spi_lb_log[1600];
__attribute__((used)) volatile uint32_t g_spi_lb_cmd;   /* write 0x52 ('R') over SWD to run again */
static uint32_t s_log_len;

static void lb_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void lb_log(const char *fmt, ...)
{
    if (s_log_len >= sizeof g_spi_lb_log - 1) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(g_spi_lb_log + s_log_len, sizeof g_spi_lb_log - s_log_len, fmt, ap);
    va_end(ap);
    if (n > 0) s_log_len += (uint32_t)n;
    if (s_log_len > sizeof g_spi_lb_log - 1) s_log_len = sizeof g_spi_lb_log - 1;
}

static uint32_t s_pass, s_fail;
static void mark(uint32_t t, int ok) { if (ok) s_pass |= t; else s_fail |= t; }

static void err_once(uint32_t t, uint32_t c, uint32_t n)
{
    if (!g_spi_lb.first_err) g_spi_lb.first_err = (t << 24) | ((c & 0xFFFu) << 12) | (n & 0xFFFu);
}

/* ---- Buffers ---- */
#define BUF_MAX 4096u
static uint8_t s_tx[BUF_MAX], s_rx[BUF_MAX];

static void fill_pattern(uint8_t *b, uint32_t n, uint32_t seed)
{
    uint32_t x = seed * 2654435761u + 12345u;
    for (uint32_t i = 0; i < n; i++) {
        x = x * 1664525u + 1013904223u;
        b[i] = (uint8_t)(x >> 24);
    }
}

static uint32_t cycles_to_us(uint32_t c) { return (uint32_t)(((uint64_t)c * 1000000u) / SYSCLK_HZ); }

/* Fastest SCK the bench supply allows (datasheet master-receiver limits):
 * the L4 runs at 3.3 V from USB, 40 MHz (STM32L422 DS Table 80); the W5 runs
 * at 1.8 V from a CoreProbe, 33 MHz below 2.7 V (DS14127 Table 94). coregen's
 * SPI_MAX_SCK_MHZ holds the same figures. The SPI kernel clock is SYSCLK. */
#if defined(STM32WBA55xx)
#define LB_SCK_MAX_HZ  33000000u
#else
#define LB_SCK_MAX_HZ  40000000u
#endif

/* Smallest prescaler (LL_SPI_PRESCALER_2 + k, SCK = SYSCLK / 2^(k+1)) whose
 * SCK is within LB_SCK_MAX_HZ: /2 at L4 80 MHz and W5 up to 64 MHz, /4 at W5
 * 100 MHz. */
static uint32_t fast_presc(void)
{
    uint32_t k = 0;
    while (k < 7u && (SYSCLK_HZ >> (k + 1u)) > LB_SCK_MAX_HZ) k++;
    return LL_SPI_PRESCALER_2 + k;
}

static hal_spi_config_t cfg_of(uint32_t presc, uint32_t mode, uint32_t lsb)
{
    hal_spi_config_t c = { .prescaler = presc, .cpol = (uint8_t)(mode >> 1),
                           .cpha = (uint8_t)(mode & 1u), .lsb_first = (uint8_t)lsb };
    return c;
}

/* ---- T0: jumper check with the SPI parked ---- */
static int jumper_present(void)
{
    hal_pad_gpio_t mo = hal_pad_lookup(LB_MOSI), mi = hal_pad_lookup(LB_MISO);
    if (!mo.port || !mi.port) return 0;
    ll_spi_disable(SPI_H->instance);

    ll_gpio_config_output(mo.port, mo.pin);
    int ok = 1;
    for (int k = 0; k < 4 && ok; k++) {
        /* High against a pull-down, then low against a pull-up: a floating
         * MISO (no jumper) follows its pull, not MOSI. */
        ll_gpio_config_input(mi.port, mi.pin, LL_GPIO_PULL_DOWN);
        ll_gpio_set(mo.port, 1UL << mo.pin);
        core_delay_us(50);
        ok &= ll_gpio_read(mi.port, 1UL << mi.pin) != 0;
        ll_gpio_config_input(mi.port, mi.pin, LL_GPIO_PULL_UP);
        ll_gpio_clear(mo.port, 1UL << mo.pin);
        core_delay_us(50);
        ok &= !(ll_gpio_read(mi.port, 1UL << mi.pin) != 0);
    }

    /* Back to SPI: AF, push-pull, very high speed, no pull (as coregen does). */
    ll_gpio_config_af(mo.port, mo.pin, SPI_AF, LL_GPIO_OTYPE_PP, LL_GPIO_SPEED_VHIGH, LL_GPIO_PULL_NONE);
    ll_gpio_config_af(mi.port, mi.pin, SPI_AF, LL_GPIO_OTYPE_PP, LL_GPIO_SPEED_VHIGH, LL_GPIO_PULL_NONE);
    (void)core_spi_configure(SPI_H, &SPI_H->cfg);
    return ok;
}

/* One buffer check: polled or DMA, rx == tx. */
static int buf_ok(uint32_t n, int dma, uint32_t seed)
{
    fill_pattern(s_tx, n, seed);
    memset(s_rx, 0x3C, n);
    hal_status_t s = dma ? core_spi_exchange_dma(SPI_H, s_tx, s_rx, n)
                         : core_spi_exchange(SPI_H, s_tx, s_rx, n);
    return s == HAL_OK && memcmp(s_tx, s_rx, n) == 0;
}

/* ---- T1 ---- */
static void t1_bytes(void)
{
    const uint32_t presc[2] = { LL_SPI_PRESCALER_256, fast_presc() };
    static const uint8_t pats[4] = { 0x00, 0xFF, 0xA5, 0x5A };
    int ok = 1;
    for (uint32_t p = 0; p < 2; p++) {
        hal_spi_config_t c = cfg_of(presc[p], 0, 0);
        core_spi_configure(SPI_H, &c);
        if (p == 0) g_spi_lb.sck_slow_hz = core_spi_sck_hz(SPI_H);
        else        g_spi_lb.sck_fast_hz = core_spi_sck_hz(SPI_H);
        for (uint32_t i = 0; i < 4; i++)
            if (core_spi_transfer(SPI_H, pats[i]) != pats[i]) { ok = 0; err_once(1, p * 16 + i, pats[i]); }
        for (uint32_t v = 0; v < 256; v++)
            if (core_spi_transfer(SPI_H, (uint8_t)v) != v) { ok = 0; err_once(1, p * 16 + 4, v); break; }
        for (int dma = 0; dma < 2; dma++) {
            for (uint32_t v = 0; v < 256; v++) s_tx[v] = (uint8_t)v;
            memset(s_rx, 0, 256);
            hal_status_t s = dma ? core_spi_exchange_dma(SPI_H, s_tx, s_rx, 256)
                                 : core_spi_exchange(SPI_H, s_tx, s_rx, 256);
            if (s != HAL_OK || memcmp(s_tx, s_rx, 256)) { ok = 0; err_once(1, p * 16 + 5 + dma, (uint32_t)-s); }
        }
        core_watchdog_feed();
    }
    mark(T_BYTES, ok);
    lb_log("  T1 bytes   %s  single 00 FF A5 5A + 0..255, 256-byte buffer; SCK %lu Hz and %lu Hz\r\n",
         ok ? "PASS" : "FAIL", (unsigned long)g_spi_lb.sck_slow_hz, (unsigned long)g_spi_lb.sck_fast_hz);
}

/* ---- T2 ---- */
static void t2_buffers(void)
{
    static const uint32_t sizes[] = { 1, 2, 3, 63, 64, 65, 255, 256, 1024, 4096 };
    hal_spi_config_t c = cfg_of(LL_SPI_PRESCALER_4, 0, 0);
    core_spi_configure(SPI_H, &c);
    int ok = 1;
    for (uint32_t i = 0; i < sizeof sizes / sizeof sizes[0]; i++) {
        for (int dma = 0; dma < 2; dma++) {
            if (!buf_ok(sizes[i], dma, i * 2u + (uint32_t)dma)) { ok = 0; err_once(2, i * 2u + (uint32_t)dma, sizes[i]); }
        }
        core_watchdog_feed();
    }
    /* In place (tx == rx), discard (rx NULL), fill (tx NULL → 0xFF), both paths. */
    for (int dma = 0; dma < 2; dma++) {
        fill_pattern(s_tx, 300, 77);
        memcpy(s_rx, s_tx, 300);
        hal_status_t s = dma ? core_spi_exchange_dma(SPI_H, s_rx, s_rx, 300) : core_spi_exchange(SPI_H, s_rx, s_rx, 300);
        if (s != HAL_OK || memcmp(s_tx, s_rx, 300)) { ok = 0; err_once(2, 40 + (uint32_t)dma, 300); }
        s = dma ? core_spi_exchange_dma(SPI_H, s_tx, NULL, 300) : core_spi_exchange(SPI_H, s_tx, NULL, 300);
        if (s != HAL_OK) { ok = 0; err_once(2, 42 + (uint32_t)dma, 300); }
        memset(s_rx, 0, 300);
        s = dma ? core_spi_exchange_dma(SPI_H, NULL, s_rx, 300) : core_spi_read(SPI_H, s_rx, 300);
        for (uint32_t k = 0; k < 300; k++) if (s_rx[k] != 0xFF) { s = HAL_ERROR; break; }
        if (s != HAL_OK) { ok = 0; err_once(2, 44 + (uint32_t)dma, 300); }
    }
    mark(T_BUFS, ok);
    lb_log("  T2 buffers %s  1 2 3 63 64 65 255 256 1024 4096 B, polled + DMA, in place, NULL rx, NULL tx; SCK %lu Hz\r\n",
         ok ? "PASS" : "FAIL", (unsigned long)core_spi_sck_hz(SPI_H));
}

/* ---- T3 ---- */
static int sck_level(void)
{
    hal_pad_gpio_t g = hal_pad_lookup(LB_SCK);
    return g.port ? (ll_gpio_read(g.port, 1UL << g.pin) != 0) : -1;
}

static void t3_modes(void)
{
    int ok = 1;
    char idle[6] = "....";
    for (uint32_t m = 0; m < 5; m++) {           /* 0-3, then mode 0 LSB first */
        uint32_t mode = m < 4 ? m : 0, lsb = (m == 4);
        hal_spi_config_t c = cfg_of(LL_SPI_PRESCALER_8, mode, lsb);
        core_spi_configure(SPI_H, &c);
        if (!buf_ok(256, 0, 100 + m) || !buf_ok(256, 1, 200 + m)) { ok = 0; err_once(3, m, 256); }
        core_delay_us(5);
        int lvl = sck_level();
        if (m < 4) {
            idle[m] = (char)('0' + lvl);
            if (lvl != (int)(mode >> 1)) { ok = 0; err_once(3, 10 + m, (uint32_t)lvl); }
        }
    }
    hal_spi_config_t c = cfg_of(LL_SPI_PRESCALER_8, 0, 0);
    core_spi_configure(SPI_H, &c);
    mark(T_MODES, ok);
    lb_log("  T3 modes   %s  modes 0-3 + LSB first, polled + DMA; SCK idle level per mode: %s (want 0011)\r\n",
         ok ? "PASS" : "FAIL", idle);
}

/* ---- T4 ---- */
static uint32_t measure_Bps(int dma, int *ok)
{
    const uint32_t reps = 8;
    fill_pattern(s_tx, BUF_MAX, 9);
    uint32_t t0 = CORE_DWT_CYCCNT;
    for (uint32_t r = 0; r < reps; r++) {
        hal_status_t s = dma ? core_spi_exchange_dma(SPI_H, s_tx, s_rx, BUF_MAX)
                             : core_spi_exchange(SPI_H, s_tx, s_rx, BUF_MAX);
        if (s != HAL_OK) *ok = 0;
    }
    uint32_t dt = CORE_DWT_CYCCNT - t0;
    if (memcmp(s_tx, s_rx, BUF_MAX)) *ok = 0;
    core_watchdog_feed();
    return (uint32_t)(((uint64_t)reps * BUF_MAX * SYSCLK_HZ) / (dt ? dt : 1));
}

static void t4_speed(void)
{
    hal_spi_config_t c = cfg_of(fast_presc(), 0, 0);
    core_spi_configure(SPI_H, &c);
    int ok = 1;
    uint32_t sck = core_spi_sck_hz(SPI_H);
    g_spi_lb.poll_Bps = measure_Bps(0, &ok);
    g_spi_lb.dma_Bps  = measure_Bps(1, &ok);
    if (!ok) err_once(4, 0, 0);
    mark(T_SPEED, ok);
    lb_log("  T4 speed   %s  SCK %lu Hz (wire max %lu B/s): polled %lu B/s, DMA %lu B/s (4096 B x 8, rx == tx)\r\n",
         ok ? "PASS" : "FAIL", (unsigned long)sck, (unsigned long)(sck / 8u),
         (unsigned long)g_spi_lb.poll_Bps, (unsigned long)g_spi_lb.dma_Bps);
    c = cfg_of(LL_SPI_PRESCALER_8, 0, 0);
    core_spi_configure(SPI_H, &c);
}

/* ---- T5: force a transfer that cannot complete ----
 * The SPI is switched to an unselected slave (MSTR / MASTER cleared, software
 * SS still high), so nothing drives SCK and no frame ever arrives: the
 * defined, safe stand-in for a stuck bus. The HAL must return HAL_TIMEOUT
 * within its stall budget and reset the peripheral, so the next transfer
 * works with no help from the test. On the L4 the same is also done with the
 * SPI's bus clock gated in RCC (its registers then read 0). */
static void force_unselected_slave(void)
{
    SPI_TypeDef *s = SPI_H->instance;
#if defined(STM32L422xx)
    CLR_BITS(s->CR1, LL_SPI_CR1_SPE);
    CLR_BITS(s->CR1, LL_SPI_CR1_MSTR);
    SET_BITS(s->CR1, LL_SPI_CR1_SPE);
#else
    CLR_BITS(s->CR1, LL_SPI_CR1_SPE);
    CLR_BITS(s->CFG2, LL_SPI_CFG2_MASTER);
#endif
}

static volatile uint32_t s_async_done;
static void on_async(void *ctx) { (void)ctx; s_async_done++; }

static void t5_timeout(void)
{
    hal_spi_config_t c = cfg_of(LL_SPI_PRESCALER_8, 0, 0);
    core_spi_configure(SPI_H, &c);
    uint32_t bound_us = cycles_to_us(ll_spi_stall_cycles(SPI_H->instance));
    g_spi_lb.t5_bound_us = bound_us;
    uint32_t limit_us = 2u * bound_us + 1000u;   /* two stalls (transfer + drain) + slack */
    int ok = 1;
    fill_pattern(s_tx, 64, 5);

    /* a) polled */
    force_unselected_slave();
    uint32_t t0 = CORE_DWT_CYCCNT;
    hal_status_t s = core_spi_exchange(SPI_H, s_tx, s_rx, 64);
    g_spi_lb.t5_poll_us = cycles_to_us(CORE_DWT_CYCCNT - t0);
    if (s != HAL_TIMEOUT || g_spi_lb.t5_poll_us > limit_us) { ok = 0; err_once(5, 1, (uint32_t)-s); }
    if (!buf_ok(64, 0, 51)) { ok = 0; err_once(5, 2, 0); }       /* recovered by the HAL */

    /* b) blocking DMA */
    force_unselected_slave();
    t0 = CORE_DWT_CYCCNT;
    s = core_spi_exchange_dma(SPI_H, s_tx, s_rx, 64);
    g_spi_lb.t5_dma_us = cycles_to_us(CORE_DWT_CYCCNT - t0);
    if (s != HAL_TIMEOUT || g_spi_lb.t5_dma_us > limit_us) { ok = 0; err_once(5, 3, (uint32_t)-s); }
    if (!buf_ok(64, 1, 52)) { ok = 0; err_once(5, 4, 0); }

    /* c) async DMA, then the bounded wait */
    force_unselected_slave();
    s_async_done = 0;
    t0 = CORE_DWT_CYCCNT;
    s = core_spi_xfer_dma(SPI_H, s_tx, s_rx, 64, on_async, NULL);
    hal_status_t w = core_spi_dma_wait(SPI_H);
    g_spi_lb.t5_async_us = cycles_to_us(CORE_DWT_CYCCNT - t0);
    if (s != HAL_OK || w != HAL_TIMEOUT || core_spi_busy(SPI_H) || s_async_done != 1
        || g_spi_lb.t5_async_us > limit_us) { ok = 0; err_once(5, 5, (uint32_t)-w); }
    /* ... and an async transfer that does complete */
    s_async_done = 0;
    fill_pattern(s_tx, 64, 6);
    s = core_spi_xfer_dma(SPI_H, s_tx, s_rx, 64, on_async, NULL);
    w = core_spi_dma_wait(SPI_H);
    if (s != HAL_OK || w != HAL_OK || s_async_done != 1 || memcmp(s_tx, s_rx, 64)) { ok = 0; err_once(5, 6, (uint32_t)-w); }

#if defined(STM32L422xx)
    /* d) L4: SPI1 bus clock gated (RCC_APB2ENR.SPI1EN = 0, RM0394 §6.4.20) */
    CLR_BITS(REG32(RCC_BASE + 0x60UL), LL_APB2_SPI1);
    (void)REG32(RCC_BASE + 0x60UL);
    t0 = CORE_DWT_CYCCNT;
    s = core_spi_exchange(SPI_H, s_tx, s_rx, 64);
    g_spi_lb.t5_gated_us = cycles_to_us(CORE_DWT_CYCCNT - t0);
    ll_rcc_apb2_clk_enable(LL_APB2_SPI1);
    if (s != HAL_TIMEOUT || g_spi_lb.t5_gated_us > limit_us) { ok = 0; err_once(5, 7, (uint32_t)-s); }
    (void)core_spi_configure(SPI_H, &c);   /* its registers were lost with the clock */
    if (!buf_ok(64, 0, 53)) { ok = 0; err_once(5, 8, 0); }
#endif
    core_watchdog_feed();
    mark(T_TIMEOUT, ok);
    lb_log("  T5 timeout %s  forced stall (unselected slave): polled %lu us, DMA %lu us, async+wait %lu us",
         ok ? "PASS" : "FAIL", (unsigned long)g_spi_lb.t5_poll_us,
         (unsigned long)g_spi_lb.t5_dma_us, (unsigned long)g_spi_lb.t5_async_us);
#if defined(STM32L422xx)
    lb_log(", clock gated %lu us", (unsigned long)g_spi_lb.t5_gated_us);
#endif
    lb_log(" (stall budget %lu us); next transfer OK\r\n", (unsigned long)bound_us);
}

/* ---- T6: tiles_pal spi_transfer, CS held ---- */
static volatile uint32_t s_cs_edges;
static void on_cs(void *ctx) { (void)ctx; s_cs_edges++; }

/* Select tile 0's chip select by hand. With one CS pad that is the bus's own
 * (core_spi_select); with a per-tile map (TWO_TILES) core_spi_select() drives
 * no pad, so select map entry 0 (pad LB_CS). */
static void lb_select(int on)
{
#ifdef LB_CS2
    if (on) (void)hal_spi_select_id(SPI_H, 0);
    else    hal_spi_deselect_id(SPI_H, 0);
#else
    if (on) core_spi_select(SPI_H);
    else    core_spi_deselect(SPI_H);
#endif
}

static void t6_pal(void)
{
    int ok = 1;
    tiles_pal_t *pal = core_tiles_pal(SPI_H);
    hal_pad_gpio_t cs = hal_pad_lookup(LB_CS);
    if (!pal || !pal->spi_transfer || !cs.port || (!SPI_H->cs_port && !SPI_H->cs_map_len)) { mark(T_PAL, 0); err_once(6, 0, 0); lb_log("  T6 pal     FAIL  no PAL / CS pad\r\n"); return; }
    /* hal_exti_enable, not core_pad_on_change: the core wrapper turns the pad
     * into an input, and this one must stay the driven chip select. The EXTI
     * sees an output pin's own edges through its input stage. */
    if (hal_exti_enable(LB_CS, HAL_EXTI_BOTH, on_cs, NULL) != HAL_OK) { ok = 0; err_once(6, 9, 0); }

    /* a) Store.O.128-style READ: 0x03 + 24-bit address, then 64 data bytes.
     *    In loopback the data phase returns what MOSI carried: the 0xFF fill. */
    static const uint8_t cmd[4] = { 0x03, 0x12, 0x34, 0x56 };
    uint8_t data[64];
    memset(data, 0, sizeof data);
    s_cs_edges = 0;
    int rc = pal->spi_transfer(pal->handle, 0, cmd, 4, data, sizeof data);
    core_delay_us(20);
    uint32_t e1 = s_cs_edges;
    int high_after = (ll_gpio_read(cs.port, 1UL << cs.pin) != 0);
    for (uint32_t i = 0; i < sizeof data; i++) if (data[i] != 0xFF) { ok = 0; err_once(6, 1, i); break; }
    if (rc != 0 || e1 != 2 || !high_after) { ok = 0; err_once(6, 2, e1); }
    g_spi_lb.cs_edges = e1;

    /* b) the same command stream captured full duplex under one select:
     *    every byte on MOSI (command, address, fill) comes back as sent. */
    uint8_t echo[4 + 16];
    s_cs_edges = 0;
    lb_select(1);
    hal_status_t s = core_spi_exchange(SPI_H, cmd, echo, 4);
    if (s == HAL_OK) s = core_spi_exchange(SPI_H, NULL, echo + 4, 16);
    lb_select(0);
    core_delay_us(20);
    if (s != HAL_OK || memcmp(echo, cmd, 4) || s_cs_edges != 2) { ok = 0; err_once(6, 3, s_cs_edges); }
    for (uint32_t i = 4; i < sizeof echo; i++) if (echo[i] != 0xFF) { ok = 0; err_once(6, 4, i); break; }

    /* c) register read / write adapters: one CS assertion each */
    s_cs_edges = 0;
    rc = pal->spi_read(pal->handle, 0, 0x0F, data, 6);
    rc |= pal->spi_write(pal->handle, 0, 0x20, cmd, 4);
    core_delay_us(20);
    if (rc != 0 || s_cs_edges != 4) { ok = 0; err_once(6, 5, s_cs_edges); }

    hal_exti_disable(LB_CS);
    mark(T_PAL, ok);
    lb_log("  T6 pal     %s  spi_transfer(03 12 34 56 + 64 B read): rc %d, %lu CS edges (want 2), data 0xFF; "
         "echo of cmd+addr under one select OK=%d\r\n",
         ok ? "PASS" : "FAIL", rc, (unsigned long)e1, s == HAL_OK);
}

/* ---- T7: two tiles on one bus, one chip select each ---- */
#ifdef LB_CS2
static int pad_high(uint8_t pad)
{
    hal_pad_gpio_t g = hal_pad_lookup(pad);
    return g.port && ll_gpio_read(g.port, 1UL << g.pin) != 0;
}

/* Right after core_init(), before any transfer: both CS pads driven high
 * (outputs, level high). -1 = not measured. */
static int s_cs_high_at_init = -1;

static int pad_is_output(uint8_t pad)
{
    hal_pad_gpio_t g = hal_pad_lookup(pad);
    return g.port && ((g.port->MODER >> (g.pin * 2u)) & 3u) == 1u;
}

static volatile uint32_t s_fall_a, s_fall_b;
static void on_fall_a(void *ctx) { (void)ctx; s_fall_a++; }
static void on_fall_b(void *ctx) { (void)ctx; s_fall_b++; }

#define T7_ROUNDS 5u
#define T7_OPS    4u    /* spi_transfer, spi_read, spi_write, full duplex under select_id */

static void t7_two_tiles(void)
{
    int ok = 1, data_ok = 1;
    tiles_pal_t *pal = core_tiles_pal(SPI_H);
    if (!pal || !pal->spi_transfer || SPI_H->cs_map_len != 2) {
        mark(T_TWO, 0); err_once(7, 0, SPI_H->cs_map_len);
        lb_log("  T7 2 tiles FAIL  no 2-entry CS map on the bus (have %u)\r\n", SPI_H->cs_map_len);
        return;
    }
    /* Falling edge = one assertion. hal_exti_enable keeps the pads outputs. */
    if (hal_exti_enable(LB_CS, HAL_EXTI_FALLING, on_fall_a, NULL) != HAL_OK ||
        hal_exti_enable(LB_CS2, HAL_EXTI_FALLING, on_fall_b, NULL) != HAL_OK) { ok = 0; err_once(7, 9, 0); }

    static const uint8_t cmd[4] = { 0x03, 0x12, 0x34, 0x56 };
    uint8_t data[16], tx[32], rx[32];
    uint32_t tot_a = 0, tot_b = 0, wrong = 0;
    for (uint32_t r = 0; r < T7_ROUNDS; r++) {
        for (uint8_t cs = 0; cs < 2; cs++) {
            for (uint32_t op = 0; op < T7_OPS; op++) {
                uint32_t a0 = s_fall_a, b0 = s_fall_b;
                int rc;
                memset(data, 0, sizeof data);
                if (op == 0) {          /* command + address, then read (loopback: the 0xFF fill) */
                    rc = pal->spi_transfer(pal->handle, cs, cmd, 4, data, sizeof data);
                    for (uint32_t i = 0; i < sizeof data; i++) if (data[i] != 0xFF) { data_ok = 0; err_once(7, 0x11, i); break; }
                } else if (op == 1) {   /* register read */
                    rc = pal->spi_read(pal->handle, cs, 0x0F, data, 6);
                    for (uint32_t i = 0; i < 6; i++) if (data[i] != 0xFF) { data_ok = 0; err_once(7, 0x12, i); break; }
                } else if (op == 2) {   /* register write */
                    rc = pal->spi_write(pal->handle, cs, 0x20, cmd, 4);
                } else {                /* full duplex under this tile's CS: rx == tx */
                    fill_pattern(tx, sizeof tx, r * 16u + cs * 4u + op);
                    memset(rx, 0, sizeof rx);
                    rc = hal_spi_select_id(SPI_H, cs);
                    if (rc == 0) {
                        rc = core_spi_exchange(SPI_H, tx, rx, sizeof tx) == HAL_OK ? 0 : -1;
                        hal_spi_deselect_id(SPI_H, cs);
                    }
                    if (memcmp(tx, rx, sizeof tx)) { data_ok = 0; err_once(7, 0x13, r); }
                }
                core_delay_us(10);
                uint32_t da = s_fall_a - a0, db = s_fall_b - b0;
                tot_a += da; tot_b += db;
                if (rc != 0) { ok = 0; err_once(7, 0x20 + op, r); }
                if (da != (cs == 0 ? 1u : 0u) || db != (cs == 1 ? 1u : 0u)) {
                    ok = 0; wrong++; err_once(7, 0x30 + op * 2u + cs, (da << 4) | db);
                }
                if (!pad_high(LB_CS) || !pad_high(LB_CS2)) { ok = 0; err_once(7, 0x40 + op, cs); }
            }
        }
    }

    /* A cs that is in no map entry: -1, and neither pad moves. */
    uint32_t a0 = s_fall_a, b0 = s_fall_b;
    int u1 = pal->spi_transfer(pal->handle, 7, cmd, 4, data, 4);
    int u2 = pal->spi_read(pal->handle, 2, 0x0F, data, 1);
    int u3 = pal->spi_write(pal->handle, 0xFF, 0x20, cmd, 1);
    core_delay_us(10);
    uint32_t unk_edges = (s_fall_a - a0) + (s_fall_b - b0);
    if (u1 != -1 || u2 != -1 || u3 != -1 || unk_edges) { ok = 0; err_once(7, 0x50, unk_edges); }

    hal_exti_disable(LB_CS);
    hal_exti_disable(LB_CS2);
    if (s_cs_high_at_init != 1) { ok = 0; err_once(7, 0x60, 0); }

    const uint32_t want = T7_ROUNDS * T7_OPS;
    if (tot_a != want || tot_b != want) ok = 0;
    ok = ok && data_ok;
    g_spi_lb.t7_edges_a = tot_a;
    g_spi_lb.t7_edges_b = tot_b;
    g_spi_lb.t7_wrong = wrong;
    mark(T_TWO, ok);
    lb_log("  T7 2 tiles %s  pad %d (cs 0): %lu falling edges, pad %d (cs 1): %lu (want %lu each); "
           "ops that moved the wrong pad: %lu; unknown cs 7/2/255 -> rc %d/%d/%d, %lu edges; "
           "both CS high (outputs) after init: %s; loopback data %s\r\n",
           ok ? "PASS" : "FAIL", LB_CS, (unsigned long)tot_a, LB_CS2, (unsigned long)tot_b,
           (unsigned long)want, (unsigned long)wrong, u1, u2, u3, (unsigned long)unk_edges,
           s_cs_high_at_init == 1 ? "yes" : "NO", data_ok ? "OK" : "BAD");
}
#endif /* LB_CS2 */

static uint32_t run_all(void)
{
    s_pass = s_fail = 0;
    s_log_len = 0;
    g_spi_lb_log[0] = 0;
    g_spi_lb.magic = 0;
    g_spi_lb.first_err = 0;
    g_spi_lb.runs++;

    int jumper = jumper_present();
    mark(T_JUMPER, jumper);
    uint32_t verdict;
    if (!jumper) {
        lb_log("[hw-spi-loopback] SKIP: loopback jumper missing (pad %d MOSI -> pad %d MISO); T1-T%d not run\r\n",
             LB_MOSI, LB_MISO, N_TESTS - 1);
        verdict = VERDICT_SKIP;
    } else {
        lb_log("  T0 jumper  PASS  MISO (pad %d) follows MOSI (pad %d)\r\n", LB_MISO, LB_MOSI);
        t1_bytes();
        t2_buffers();
        t3_modes();
        t4_speed();
        t5_timeout();
        t6_pal();
#ifdef LB_CS2
        t7_two_tiles();
#endif
        uint32_t missing = (EXPECTED & ~s_pass) | s_fail;
        uint32_t first = 0;
        while (first < N_TESTS && !(missing & (1u << first))) first++;
        verdict = missing ? (VERDICT_FAIL | first) : VERDICT_PASS;
    }

    g_spi_lb.verdict  = verdict;
    g_spi_lb.pass     = s_pass;
    g_spi_lb.fail     = s_fail;
    g_spi_lb.expected = EXPECTED;
    g_spi_lb.magic    = 0x5B1100B5u;
    return verdict;
}

#if defined(STM32L422xx)
static void print_report(uint32_t verdict)
{
    const char *v = verdict == VERDICT_PASS ? "PASS" : verdict == VERDICT_SKIP ? "SKIP" : "FAIL";
    core_usb_printf("\r\n[hw-spi-loopback] %s  (pass=0x%02lx fail=0x%02lx expected=0x%02lx run %lu, first_err 0x%08lx)\r\n",
                    v, (unsigned long)g_spi_lb.pass, (unsigned long)g_spi_lb.fail,
                    (unsigned long)EXPECTED, (unsigned long)g_spi_lb.runs,
                    (unsigned long)g_spi_lb.first_err);
    /* The log can be longer than one CDC packet: send it in chunks. */
    for (uint32_t off = 0; off < s_log_len; off += 48) {
        uint32_t n = s_log_len - off > 48 ? 48 : s_log_len - off;
        core_usb_write((const uint8_t *)g_spi_lb_log + off, (uint16_t)n);
    }
}
#endif

static void wait_ms(uint32_t ms)
{
    while (ms) {
        uint32_t d = ms > 100u ? 100u : ms;
        core_watchdog_feed();
        core_delay_ms(d);
        ms -= d;
    }
}

int main(void)
{
    core_init();                  /* SPI bus + CS pad from config.json; 5 s watchdog */
#ifdef LB_CS2
    /* Before any transfer: both tiles' chip selects are driven high. */
    s_cs_high_at_init = pad_is_output(LB_CS) && pad_is_output(LB_CS2) &&
                        pad_high(LB_CS) && pad_high(LB_CS2);
#endif
    core_led_init();
    core_cycle_init();
#if defined(STM32L422xx)
    wait_ms(1500);                /* let the host open the CDC port */
#endif
    uint32_t verdict = run_all();

    uint32_t last_print = 0;
    while (1) {
#if defined(STM32L422xx)
        if ((uint32_t)(core_millis() - last_print) >= 3000u) {
            last_print = core_millis();
            print_report(verdict);
        }
        uint8_t ch;
        if (core_usb_try_read(&ch) && (ch == 'R' || ch == 'r')) {
            verdict = run_all();
            last_print = core_millis() - 3000u;
        }
#else
        (void)last_print;
#endif
        if (g_spi_lb_cmd == 'R') {
            g_spi_lb_cmd = 0;
            verdict = run_all();
        }
        /* PASS: 1 s on / 1 s off. SKIP: two short blinks, 2 s pause.
         * FAIL: N+1 quick blinks (N = first failing test), 1.5 s pause. */
        if (verdict == VERDICT_PASS) {
            LED_ON();  wait_ms(1000);
            LED_OFF(); wait_ms(1000);
        } else if (verdict == VERDICT_SKIP) {
            for (int i = 0; i < 2; i++) { LED_ON(); wait_ms(100); LED_OFF(); wait_ms(200); }
            wait_ms(2000);
        } else {
            for (uint32_t i = 0; i <= (verdict & 0xFFu); i++) {
                LED_ON();  wait_ms(150);
                LED_OFF(); wait_ms(250);
            }
            wait_ms(1500);
        }
    }
}
