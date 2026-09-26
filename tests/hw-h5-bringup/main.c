/**
 * hw-h5-bringup — Core.ST.H5 (STM32H523) foundation check over USB CDC.
 *
 * The H5 test board has no LED, so everything is reported on the CDC port.
 * The report prints when a terminal opens the port, and again on "R":
 *
 *   ENTRY    RCC / USB / flash state found at main() entry, before core_init():
 *            what a reset, the ST ROM's DFU "leave" or the serial-update
 *            flasher left running (a ROM "leave" keeps HSIDIV /1 and VOS
 *            raised; after a BOOT0-high boot it also leaves GTZC securing
 *            peripherals: started_by_rom=1)
 *   RCC      RCC_CR / CFGR1 / CFGR2 / PLL1CFGR / PLL1DIVR, FLASH_ACR, VOS,
 *            ICACHE_CR after core_init()
 *   CLK      SYSCLK measured at each coregen clock level (low 16, medium 64,
 *            high 128, max 248 MHz): DWT cycles across 1000 USB SOFs, the
 *            host's 1 kHz frame clock (+-500 ppm, USB 2.0 §7.1.12), with the
 *            flash wait states and WRHIGHFREQ each level set
 *   TIM1     update ticks at 1 kHz over 1000 SOFs (expect 1000)
 *   ADC      VREFINT raw, VREFINT_CAL, VDDA mV, and a GPDMA1 circular run:
 *            half / full callbacks counted over 1 s (they keep coming), and
 *            none after hal_adc_stop_dma
 *   PERIPH   I2C3 clock (APB3ENR bit 7) + a register read-back; SPI (below);
 *            EXTI port select read-back
 *   SPI      SPI1 on its pads (pad 9 PA5 SCK, pad 8 PA7 MOSI, pad 3 PB4 MISO)
 *            and SPI3 (SCK on PB3, a ball no pad uses; MISO on pad 3): each
 *            first with SCK left analog, which must time out (the SPI v2
 *            master clocks only with SCK in AF mode, ll_spi.h), then with the
 *            pins in AF: polled and DMA exchanges. With the pad 8 -> pad 3
 *            strap SPI1 reads back what it sent, and SPI3 reads pad 8 driven
 *            high then low as a GPIO
 *   USB      the serial number (chip UID) the host should report
 *   IDS      DEV_ID, REV_ID, flash size
 *   NVM      ("N") core_nvm on the flash high-cycle data area: the first-boot
 *            record (_core_nvm_h5_boot: option registers before / after the
 *            one change, writes, boots), the option registers now, and
 *            "NW <hex off> <hex val>" / "NR <hex off>" / "NE" to write, read
 *            and erase the store
 *
 * Other commands (one per line): "L" re-measures the clock levels, "T" / "A"
 * / "P" / "S" / "I" run one check, "M <hex addr> [n]" / "W <hex addr> <hex val>"
 * peek and poke, "E <text>" echoes, "Q" drops off the bus to test the 15 s
 * no-USB safety net, "X" resets (with BOOT0 high: into the ST ROM
 * bootloader), "V" says hello.
 */

#include "core.h"
#include "core_usb.h"
#include "core_watchdog.h"
#include "core_uid.h"
#include "ll_rcc.h"
#include "ll_exti.h"
#include "ll_usb_drd.h"
#include "ll_systick.h"
#include "hal_timer.h"
#include "hal_adc.h"
#include "hal_spi.h"
#include "hal_dfu.h"
#include "hal_fault.h"
#include "core_nvm.h"
#include "ll_flash_edata.h"
#include <string.h>
#include <stdlib.h>

#define DEMCR     REG32(0xE000EDFCUL)   /* TRCENA = bit 24 */
#define DWT_CTRL  REG32(0xE0001000UL)   /* CYCCNTENA = bit 0 */
#define DWT_CYC   REG32(0xE0001004UL)
#define RCCR(o)   REG32(RCC_BASE + (o))

static uint32_t g_entry[13];

/* Fault record, kept across the reset a fault ends in (hal_fault reboots into
 * the ROM bootloader on a ROM-DFU build; SRAM is not erased on reset with this
 * board's option bytes). Printed by the next boot, then cleared. */
/* TAMP backup registers (TAMP_BKP0R.. at TAMP + 0x100, RM0481 §47): kept
 * across resets and untouched by the ROM bootloader. Needs RTCAPBEN (APB3ENR
 * bit 21) and backup-domain write access (PWR_DBPCR.DBP). */
#define FAULT_REC  ((volatile uint32_t *)0x44007D00UL)
#define FAULT_MAGIC 0xFA017EC0UL
static void fault_cb(hal_fault_type_t type, const hal_fault_frame_t *f)
{
    FAULT_REC[1] = (uint32_t)type;
    FAULT_REC[2] = f->pc;
    FAULT_REC[3] = f->lr;
    FAULT_REC[4] = REG32(0xE000ED28UL);   /* CFSR */
    FAULT_REC[5] = REG32(0xE000ED38UL);   /* BFAR */
    FAULT_REC[6] = REG32(0xE000ED2CUL);   /* HFSR */
    FAULT_REC[0] = FAULT_MAGIC;
}

static void capture_entry(void)
{
    g_entry[0]  = RCCR(0x00);                 /* RCC_CR */
    g_entry[1]  = RCCR(0x1C);                 /* CFGR1 */
    g_entry[2]  = RCCR(0x20);                 /* CFGR2 */
    g_entry[3]  = RCCR(0x28);                 /* PLL1CFGR */
    g_entry[4]  = RCCR(0xA4);                 /* APB2ENR (USBEN = bit 24) */
    g_entry[5]  = RCCR(0xF4);                 /* RSR */
    g_entry[6]  = (g_entry[4] & (1UL << 24)) ? USB_CNTR : 0xFFFFFFFFUL;
    g_entry[7]  = (g_entry[4] & (1UL << 24)) ? USB_BCDR : 0xFFFFFFFFUL;
    g_entry[8]  = REG32(0x40022000UL);        /* FLASH_ACR */
    g_entry[9]  = REG32(0x44020810UL);        /* PWR_VOSCR */
    g_entry[10] = REG32(0x40030400UL);        /* ICACHE_CR */
    g_entry[11] = hal_dfu_started_by_rom();
    __asm volatile ("mrs %0, ipsr" : "=r" (g_entry[12]));   /* 0 = thread mode */
}

static void cyc_init(void)
{
    DEMCR |= (1UL << 24);
    DWT_CYC = 0;
    DWT_CTRL |= 1UL;
}

static void feed(void) { core_watchdog_feed(); }

/* Wait for the next SOF (FNR.FN changes), bounded. Returns 0 on timeout. */
static int next_sof(void)
{
    uint32_t fn = USB_FNR & 0x7FFu;
    for (uint32_t i = 0; i < 50000000UL; i++)
        if ((USB_FNR & 0x7FFu) != fn) return 1;
    return 0;
}

/* CPU cycles across n SOFs (n ms of the host's frame clock). 0 = no SOFs. */
static uint32_t cycles_per_sofs(uint32_t n)
{
    feed();
    if (!next_sof()) return 0;
    uint32_t c0 = DWT_CYC;
    uint32_t fn0 = USB_FNR & 0x7FFu;
    uint32_t seen = 0;
    while (seen < n) {
        if (!next_sof()) return 0;
        seen = ((USB_FNR & 0x7FFu) - fn0) & 0x7FFu;
        if ((seen & 0xFFu) == 0) feed();
    }
    return (DWT_CYC - c0) / seen * n;         /* normalise if a frame was skipped */
}

typedef struct { const char *name; uint32_t sw, hsidiv, m, n, p, hz, vos; } level_t;
static const level_t k_levels[] = {
    { "low",    LL_RCC_H5_SYSCLK_HSI, LL_RCC_HSIDIV_4, 0, 0, 0,  16000000UL, 0 },
    { "medium", LL_RCC_H5_SYSCLK_HSI, LL_RCC_HSIDIV_1, 0, 0, 0,  64000000UL, 1 },
    { "high",   LL_RCC_H5_SYSCLK_PLL, LL_RCC_HSIDIV_1, 4, 16, 2, 128000000UL, 2 },
    { "max",    LL_RCC_H5_SYSCLK_PLL, LL_RCC_HSIDIV_1, 4, 31, 2, 248000000UL, 3 },
};

static void apply_level(const level_t *lv)
{
    uint32_t irq = ll_irq_save();
    int ok = ll_rcc_h5_clock_config(lv->sw, lv->hsidiv, lv->m, lv->n, lv->p, lv->hz, lv->vos);
    ll_systick_init(lv->hz);
    ll_irq_restore(irq);
    if (!ok) core_usb_printf("  (%s: clock config reported a failure)\n", lv->name);
}

static void measure_levels(void)
{
    for (unsigned i = 0; i < sizeof k_levels / sizeof k_levels[0]; i++) {
        const level_t *lv = &k_levels[i];
        apply_level(lv);
        core_delay_ms(20);
        uint32_t cyc = cycles_per_sofs(1000);          /* 1 s of frames */
        uint32_t hz  = cyc;                            /* cycles per 1000 ms = Hz */
        int32_t  ppm = cyc ? (int32_t)(((int64_t)hz - (int64_t)lv->hz) * 1000000LL / (int64_t)lv->hz) : 0;
        core_usb_printf("CLK %-6s expect=%lu measured=%lu err=%ldppm ACR=0x%03lx VOS=%lu "
                        "ACTVOS=%lu CFGR1=0x%08lx CR=0x%08lx\n",
                        lv->name, (unsigned long)lv->hz, (unsigned long)hz, (long)ppm,
                        (unsigned long)(REG32(0x40022000UL) & 0x1FFUL),
                        (unsigned long)((REG32(0x44020810UL) >> 4) & 3UL),
                        (unsigned long)((REG32(0x44020814UL) >> 14) & 3UL),
                        (unsigned long)RCCR(0x1C), (unsigned long)RCCR(0x00));
        feed();
    }
    /* Back to the build's level (max) with its SysTick. */
    apply_level(&k_levels[3]);
}

static volatile uint32_t g_tim_ticks;
static void tim_cb(void *ctx) { (void)ctx; g_tim_ticks++; }

static void check_tim1(void)
{
    static hal_timer_t t;
    hal_timer_tick_init(&t, TIM1, SYSCLK_HZ, 1000, tim_cb, 0);
    hal_timer_tick_start(&t);
    next_sof();
    g_tim_ticks = 0;
    uint32_t fn0 = USB_FNR & 0x7FFu, seen = 0;
    while (seen < 1000) {
        if (!next_sof()) break;
        seen = ((USB_FNR & 0x7FFu) - fn0) & 0x7FFu;
        if ((seen & 0xFFu) == 0) feed();
    }
    uint32_t ticks = g_tim_ticks;
    hal_timer_tick_stop(&t);
    core_usb_printf("TIM1 ticks=%lu over %lu SOFs (expect %lu) PSC=%lu ARR=%lu\n",
                    (unsigned long)ticks, (unsigned long)seen, (unsigned long)seen,
                    (unsigned long)TIM1->PSC, (unsigned long)TIM1->ARR);
}

static volatile uint32_t g_dma_cb;
static void adc_cb(void *ctx) { (void)ctx; g_dma_cb++; }

static void check_adc(void)
{
    static hal_adc_t adc;
    static uint16_t buf[32];
    memset(&adc, 0, sizeof adc);
    hal_status_t st = hal_adc_init(&adc, ADC1, SYSCLK_HZ, HAL_ADC_RES_12BIT);
    uint32_t vdda = hal_adc_read_vdda_mv(&adc);
    int ic = ll_icache_disable();
    uint16_t cal = *(volatile uint16_t *)0x08FFF810UL;  /* VREFINT_CAL: 16-bit read, RO area */
    if (ic) ll_icache_enable();
    hal_adc_add_channel(&adc, 17, HAL_ADC_SAMP_SLOW);    /* VREFINT = VINP[17] */
    uint16_t raw = hal_adc_read(&adc, 17);
    core_usb_printf("ADC init=%d VREFINT raw=%u cal=%u VDDA=%lumV (calibrated=%d) CCR=0x%08lx\n",
                    (int)st, raw, cal, (unsigned long)vdda, (int)adc.vdda_calibrated,
                    (unsigned long)REG32(0x42028308UL));
    /* Circular DMA: half + full callbacks keep coming (2 per pass over the
     * 32-sample ring); counted at 250 ms steps for 1 s, then after stop. */
    g_dma_cb = 0;
    memset(buf, 0, sizeof buf);
    st = hal_adc_start_dma(&adc, buf, 32, adc_cb, 0);
    uint32_t t0 = core_millis(), at[4];
    for (int k = 0; k < 4; k++) {
        while ((uint32_t)(core_millis() - t0) < 250u * (uint32_t)(k + 1)) feed();
        at[k] = g_dma_cb;
    }
    hal_adc_stop_dma(&adc);
    uint32_t stopped = g_dma_cb;
    core_delay_ms(50);
    uint16_t lo = 0xFFFF, hi = 0;
    for (int i = 0; i < 32; i++) { if (buf[i] < lo) lo = buf[i]; if (buf[i] > hi) hi = buf[i]; }
    core_usb_printf("ADC DMA start=%d callbacks at 250/500/750/1000 ms: %lu %lu %lu %lu "
                    "(%lu/s), after stop +50 ms: %lu; buf min/max %u/%u (VREFINT)\n",
                    (int)st, (unsigned long)at[0], (unsigned long)at[1], (unsigned long)at[2],
                    (unsigned long)at[3], (unsigned long)at[3], (unsigned long)(g_dma_cb - stopped),
                    lo, hi);
}

/* ---- SPI ---- */
typedef struct { uint32_t moder, otyper, ospeedr, pupdr, afr0, afr1, odr; } gpio_save_t;
static void gpio_save(GPIO_TypeDef *p, gpio_save_t *g)
{
    g->moder = p->MODER; g->otyper = p->OTYPER; g->ospeedr = p->OSPEEDR;
    g->pupdr = p->PUPDR; g->afr0 = p->AFR[0]; g->afr1 = p->AFR[1]; g->odr = p->ODR;
}
static void gpio_restore(GPIO_TypeDef *p, const gpio_save_t *g)
{
    p->ODR = g->odr; p->AFR[0] = g->afr0; p->AFR[1] = g->afr1; p->OTYPER = g->otyper;
    p->OSPEEDR = g->ospeedr; p->PUPDR = g->pupdr; p->MODER = g->moder;
}
static void spi_af(GPIO_TypeDef *p, uint32_t pin, uint32_t af)
{
    ll_gpio_config_af(p, pin, af, LL_GPIO_OTYPE_PP, LL_GPIO_SPEED_VHIGH, LL_GPIO_PULL_NONE);
}

/* One polled or DMA exchange: status, time, and the bytes that came back. */
static hal_status_t spi_run(hal_spi_t *h, int dma, const uint8_t *tx, uint8_t *rx,
                            uint32_t n, uint32_t *us)
{
    memset(rx, 0x3C, n);
    uint32_t t0 = DWT_CYC;
    hal_status_t st = dma ? hal_spi_exchange_dma(h, tx, rx, n) : hal_spi_exchange(h, tx, rx, n);
    *us = (DWT_CYC - t0) / (SYSCLK_HZ / 1000000UL);
    return st;
}

static void check_spi(void)
{
    static hal_spi_t spi;
    static uint8_t tx[256], rx[256];
    for (uint32_t i = 0; i < sizeof tx; i++) tx[i] = (uint8_t)(i * 37u + 11u);
    hal_spi_config_t cfg = { .prescaler = LL_SPI_PRESCALER_8 };
    gpio_save_t ga, gb;
    ll_rcc_gpio_clk_enable(GPIOA);
    ll_rcc_gpio_clk_enable(GPIOB);
    gpio_save(GPIOA, &ga);
    gpio_save(GPIOB, &gb);
    uint32_t us;

    /* SPI1, SCK (PA5) still analog: must time out (HAL_TIMEOUT = -3). */
    ll_gpio_config_analog(GPIOA, 5);
    hal_spi_init(&spi, SPI1, &cfg);
    hal_status_t st = spi_run(&spi, 0, tx, rx, 4, &us);
    core_usb_printf("SPI1 SCK analog: exchange=%d in %lu us (want -3, timeout)\n", (int)st,
                    (unsigned long)us);

    /* SPI1 on its pads, AF5: pad 9 PA5 SCK, pad 8 PA7 MOSI, pad 3 PB4 MISO. */
    spi_af(GPIOA, 5, 5);
    spi_af(GPIOA, 7, 5);
    spi_af(GPIOB, 4, 5);
    static const uint32_t sizes[] = { 1, 4, 17, 256 };
    for (int dma = 0; dma < 2; dma++) {
        for (unsigned k = 0; k < sizeof sizes / sizeof sizes[0]; k++) {
            uint32_t n = sizes[k];
            st = spi_run(&spi, dma, tx, rx, n, &us);
            core_usb_printf("SPI1 %s n=%lu exchange=%d in %lu us rx==tx %s (rx[0]=0x%02x, strap pad 8->3)\n",
                            dma ? "DMA   " : "polled", (unsigned long)n, (int)st, (unsigned long)us,
                            memcmp(tx, rx, n) ? "NO" : "yes", rx[0]);
        }
    }
    core_usb_printf("SPI1 sck=%lu Hz SR=0x%08lx CFG1=0x%08lx CFG2=0x%08lx\n",
                    (unsigned long)hal_spi_sck_hz(&spi), (unsigned long)SPI1->SR,
                    (unsigned long)SPI1->CFG1, (unsigned long)SPI1->CFG2);
    hal_spi_deinit(&spi);

    /* SPI3: no pad carries SPI3_SCK on this Core. PB3 (WLCSP39 ball A3,
     * JTDO) does (AF6, DS14540 Table 14) and no pad uses it; MISO is pad 3
     * (PB4, AF6). Pad 8 (PA7) is a GPIO output here, strapped to pad 3. */
    ll_gpio_config_analog(GPIOB, 3);
    spi_af(GPIOB, 4, 6);
    hal_spi_init(&spi, SPI3, &cfg);
    st = spi_run(&spi, 0, tx, rx, 4, &us);
    core_usb_printf("SPI3 SCK analog: exchange=%d in %lu us (want -3, timeout)\n", (int)st,
                    (unsigned long)us);
    spi_af(GPIOB, 3, 6);
    ll_gpio_config_output(GPIOA, 7);
    for (int lvl = 1; lvl >= 0; lvl--) {
        if (lvl) ll_gpio_set(GPIOA, 1UL << 7); else ll_gpio_clear(GPIOA, 1UL << 7);
        for (int dma = 0; dma < 2; dma++) {
            st = spi_run(&spi, dma, tx, rx, 17, &us);
            int all = 1;
            for (int i = 0; i < 17; i++) if (rx[i] != (lvl ? 0xFF : 0x00)) all = 0;
            core_usb_printf("SPI3 %s pad 8 %s: exchange=%d in %lu us, rx all 0x%02x %s\n",
                            dma ? "DMA   " : "polled", lvl ? "high" : "low ", (int)st,
                            (unsigned long)us, lvl ? 0xFF : 0x00, all ? "yes" : "NO");
        }
    }
    hal_spi_deinit(&spi);

    gpio_restore(GPIOA, &ga);
    gpio_restore(GPIOB, &gb);
    core_usb_printf("SPI APB2ENR.SPI1EN=%lu APB1LENR.SPI2EN/SPI3EN=%lu/%lu APB3ENR.SPI5EN=%lu "
                    "CCIPR3=0x%08lx CCIPR5=0x%08lx\n",
                    (unsigned long)((RCCR(0xA4) >> 12) & 1UL), (unsigned long)((RCCR(0x9C) >> 14) & 1UL),
                    (unsigned long)((RCCR(0x9C) >> 15) & 1UL), (unsigned long)((RCCR(0xA8) >> 5) & 1UL),
                    (unsigned long)RCCR(0xE0), (unsigned long)RCCR(0xE8));
}

static void check_periph(void)
{
    /* I2C3: APB3ENR bit 7, then a register that only reads back when the
     * peripheral is clocked (I2C3 TIMINGR at 0x44002800 + 0x10). */
    ll_rcc_apb3_clk_enable(LL_APB3_I2C3);
    REG32(0x44002810UL) = 0x10A0A6FBUL;
    core_usb_printf("I2C3 APB3ENR=0x%08lx (bit7=%lu) TIMINGR readback=0x%08lx (want 0x10a0a6fb)\n",
                    (unsigned long)RCCR(0xA8), (unsigned long)((RCCR(0xA8) >> 7) & 1UL),
                    (unsigned long)REG32(0x44002810UL));
    REG32(0x44002810UL) = 0;

    check_spi();

    /* EXTI: line 5 from port B (index 1), read EXTI_EXTICR2 (0x064) byte 1. */
    uint32_t before = REG32(EXTI_BASE + 0x64UL);
    ll_exti_set_source(5, 1);
    uint32_t after = REG32(EXTI_BASE + 0x64UL);
    ll_exti_set_source(5, 0);
    core_usb_printf("EXTI EXTICR2 before=0x%08lx after set(5, B)=0x%08lx (want byte1 = 0x01)\n",
                    (unsigned long)before, (unsigned long)after);
}

/* ---- NVM (core_nvm on the data area) ---- */
static void report_nvm(void)
{
    const core_nvm_h5_prov_t *p = _core_nvm_h5_prov();
    if (!p) {
        core_usb_printf("NVM PROV no record\n");
    } else {
        core_usb_printf("NVM PROV state=%ld writes=%lu boots=%lu EDATA2R before=0x%08lx after=0x%08lx "
                        "EDATA1R=0x%08lx OPTSR=0x%08lx NSSR=0x%08lx OPSR=0x%08lx mismatch=0x%03lx\n",
                        (long)p->state, (unsigned long)p->writes, (unsigned long)p->boots,
                        (unsigned long)p->edata2_before, (unsigned long)p->edata2_after,
                        (unsigned long)p->edata1, (unsigned long)p->optsr, (unsigned long)p->nssr,
                        (unsigned long)p->opsr, (unsigned long)p->mismatch);
    }
    if (p && p->writes) {
        static const char *const nm[CORE_NVM_H5_OB_N] = {
            "OPTSR", "NSEPOCHR", "SECEPOCHR", "OPTSR2", "NSBOOTR", "SECBOOTR", "OTPBLR",
            "SECWM1R", "WRP1R", "EDATA1R", "HDP1R", "SECWM2R", "WRP2R", "EDATA2R", "HDP2R" };
        static const uint16_t off[CORE_NVM_H5_OB_N] = {
            0x050, 0x060, 0x068, 0x070, 0x080, 0x088, 0x090, 0x0E0, 0x0E8, 0x0F0, 0x0F8,
            0x1E0, 0x1E8, 0x1F0, 0x1F8 };
        for (uint32_t i = 0; i < CORE_NVM_H5_OB_N; i++)
            core_usb_printf("NVM PROV %-9s _CUR before 0x%08lx after 0x%08lx now 0x%08lx%s\n", nm[i],
                            (unsigned long)p->ob_before[i], (unsigned long)p->ob_after[i],
                            (unsigned long)REG32(FLASH_BASE + off[i]),
                            p->ob_before[i] != p->ob_after[i] ? "  <- changed" : "");
    }
    core_usb_printf("NVM OB EDATA2R CUR=0x%08lx PRG=0x%08lx EDATA1R CUR=0x%08lx PRG=0x%08lx "
                    "OPTSR CUR=0x%08lx PRG=0x%08lx OPTSR2 CUR=0x%08lx OPTCR=0x%08lx NSSR=0x%08lx\n",
                    (unsigned long)FLASH_EDATA2R_CUR, (unsigned long)FLASH_EDATA2R_PRG,
                    (unsigned long)FLASH_EDATA1R_CUR, (unsigned long)REG32(FLASH_BASE + 0x0F4UL),
                    (unsigned long)REG32(FLASH_BASE + 0x050UL), (unsigned long)REG32(FLASH_BASE + 0x054UL),
                    (unsigned long)REG32(FLASH_BASE + 0x070UL), (unsigned long)FLASH_OPTCR,
                    (unsigned long)FLASH_SR);
    uint32_t v = 0;
    uint32_t t0 = DWT_CYC;
    int rc = core_nvm_read(0, &v, 4);
    uint32_t us = (DWT_CYC - t0) / (SYSCLK_HZ / 1000000UL);
    extern uint32_t _core_nvm_log_free(void);
    core_usb_printf("NVM read(0,4) rc=%d val=0x%08lx in %lu us; log free %lu B; uncorrectable reads %lu; "
                    "ICACHE on=%lu\n", rc, (unsigned long)v, (unsigned long)us,
                    (unsigned long)_core_nvm_log_free(), (unsigned long)_core_nvm_h5_ecc_errors(),
                    (unsigned long)(REG32(0x40030400UL) & 1UL));
}

static uint32_t hexarg(const char **p)
{
    const char *s = *p;
    while (*s == ' ') s++;
    uint32_t v = 0;
    for (;; s++) {
        char c = *s;
        if (c >= '0' && c <= '9') v = (v << 4) | (uint32_t)(c - '0');
        else if (c >= 'a' && c <= 'f') v = (v << 4) | (uint32_t)(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') v = (v << 4) | (uint32_t)(c - 'A' + 10);
        else break;
    }
    *p = s;
    return v;
}

static void peek(const char *arg)
{
    FAULT_REC[8] = 2;
    uint32_t a = hexarg(&arg);
    uint32_t n = hexarg(&arg);
    if (!n) n = 1;
    FAULT_REC[8] = 3;
    for (uint32_t i = 0; i < n && i < 16; i++) {
        uint32_t v = REG32(a + 4 * i);
        FAULT_REC[8] = 4;
        core_usb_printf("M 0x%08lx = 0x%08lx\n", (unsigned long)(a + 4 * i), (unsigned long)v);
        FAULT_REC[8] = 5;
    }
}

static void poke(const char *arg)
{
    uint32_t a = hexarg(&arg);
    uint32_t v = hexarg(&arg);
    REG32(a) = v;
    core_usb_printf("W 0x%08lx <- 0x%08lx, reads 0x%08lx\n", (unsigned long)a,
                    (unsigned long)v, (unsigned long)REG32(a));
}

static void report_short(void)
{
    core_usb_printf("\nH5 BRINGUP hw-h5-bringup, SYSCLK_HZ=%lu\n", (unsigned long)SYSCLK_HZ);
    core_usb_printf("ENTRY CR=0x%08lx CFGR1=0x%08lx CFGR2=0x%08lx PLL1CFGR=0x%08lx APB2ENR=0x%08lx "
                    "RSR=0x%08lx\n", (unsigned long)g_entry[0], (unsigned long)g_entry[1],
                    (unsigned long)g_entry[2], (unsigned long)g_entry[3], (unsigned long)g_entry[4],
                    (unsigned long)g_entry[5]);
    core_usb_printf("ENTRY USB_CNTR=0x%08lx USB_BCDR=0x%08lx ACR=0x%08lx VOSCR=0x%08lx "
                    "ICACHE_CR=0x%08lx started_by_rom=%lu IPSR=%lu\n", (unsigned long)g_entry[6],
                    (unsigned long)g_entry[7], (unsigned long)g_entry[8], (unsigned long)g_entry[9],
                    (unsigned long)g_entry[10], (unsigned long)g_entry[11],
                    (unsigned long)g_entry[12]);
    core_usb_printf("RCC CR=0x%08lx CFGR1=0x%08lx CFGR2=0x%08lx PLL1CFGR=0x%08lx PLL1DIVR=0x%08lx\n",
                    (unsigned long)RCCR(0x00), (unsigned long)RCCR(0x1C), (unsigned long)RCCR(0x20),
                    (unsigned long)RCCR(0x28), (unsigned long)RCCR(0x34));
    core_usb_printf("FLASH ACR=0x%08lx (LATENCY=%lu WRHIGHFREQ=%lu PRFTEN=%lu) VOSCR=0x%08lx "
                    "VOSSR=0x%08lx ICACHE CR=0x%08lx SR=0x%08lx (on=%lu)\n",
                    (unsigned long)REG32(0x40022000UL), (unsigned long)(REG32(0x40022000UL) & 0xFUL),
                    (unsigned long)((REG32(0x40022000UL) >> 4) & 3UL),
                    (unsigned long)((REG32(0x40022000UL) >> 8) & 1UL),
                    (unsigned long)REG32(0x44020810UL), (unsigned long)REG32(0x44020814UL),
                    (unsigned long)REG32(0x40030400UL), (unsigned long)REG32(0x40030404UL),
                    (unsigned long)(REG32(0x40030400UL) & 1UL));
}

static void report_ids(void)
{
    char uid[25];
    core_uid_hex(uid, sizeof uid);
    core_usb_printf("USB serial=%s CRS_CR=0x%08lx CRS_ISR=0x%08lx HSI48RDY=%lu\n", uid,
                    (unsigned long)REG32(0x40006000UL), (unsigned long)REG32(0x40006008UL),
                    (unsigned long)((RCCR(0x00) >> 13) & 1UL));
    /* The flash-size word is in the read-only area: a read through the
     * ICACHE bus-faults (0x08FFF80C, bench 2026-09-26). The ROM's vector
     * table is readable only in some boots (hidden after a ROM "leave"),
     * so it is not read here. */
    int ic = ll_icache_disable();
    uint16_t fkb = *(volatile uint16_t *)0x08FFF80CUL;
    if (ic) ll_icache_enable();
    core_usb_printf("DEV_ID=0x%03lx REV_ID=0x%04lx FLASH=%u KB\n",
                    (unsigned long)(REG32(0x44024000UL) & 0xFFFUL),
                    (unsigned long)(REG32(0x44024000UL) >> 16), fkb);
}

static void report(void)
{
    report_short();
    report_ids();
    measure_levels();
    check_tim1();
    check_adc();
    check_periph();
    core_usb_printf("REPORT END\n");
}

int main(void)
{
    capture_entry();
    SET_BITS(RCCR(0xA8), 1UL << 21);                   /* RTCAPBEN: TAMP registers */
    (void)RCCR(0xA8);
    SET_BITS(REG32(0x44020824UL), 1UL);                /* PWR_DBPCR.DBP */
    for (int t = 100000; !(REG32(0x44020824UL) & 1UL) && t; t--) { }
    uint32_t frec[9];
    frec[7] = FAULT_REC[7];
    frec[8] = FAULT_REC[8];
    for (int i = 0; i < 7; i++) frec[i] = FAULT_REC[i];
    FAULT_REC[0] = 0;
    core_init();                 /* max clock, USB CDC, 5 s watchdog */
    hal_fault_set_callback(fault_cb);
    core_usb_printf("EARLY hw-h5-bringup at t=%lu ms, before enumeration\n",
                    (unsigned long)core_millis());
    core_usb_printf("LAST CMD 0x%08lx checkpoint %lu\n", (unsigned long)frec[7],
                    (unsigned long)frec[8]);
    if (frec[0] == FAULT_MAGIC)
        core_usb_printf("LAST FAULT type=%lu PC=0x%08lx LR=0x%08lx CFSR=0x%08lx BFAR=0x%08lx "
                        "HFSR=0x%08lx\n", (unsigned long)frec[1], (unsigned long)frec[2],
                        (unsigned long)frec[3], (unsigned long)frec[4], (unsigned long)frec[5],
                        (unsigned long)frec[6]);
    cyc_init();

    char line[32];
    uint32_t len = 0;
    int was = 0;
    for (;;) {
        feed();
        /* Bench safety net: a board strapped BOOT0-high has no other way
         * back. No USB address from the host 15 s after boot means USB never
         * came up: reset (into the ROM bootloader with BOOT0 high). */
        if ((USB_DADDR & 0x7FUL) == 0 && core_millis() > 15000u) {
            SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
            for (;;) { }
        }
        int now = core_usb_connected();
        if (now && !was) {
            core_delay_ms(200);  /* let the open-time settle pass */
            report_short();
        }
        was = now;
        uint8_t b;
        while (core_usb_try_read(&b)) {
            if (b == '\r') continue;
            if (b != '\n') {
                if (len < sizeof line - 1) line[len++] = (char)b;
                continue;
            }
            line[len] = '\0';
            FAULT_REC[7] = (uint32_t)line[0] | ((uint32_t)line[1] << 8) |
                           ((uint32_t)line[2] << 16) | ((uint32_t)line[3] << 24);
            FAULT_REC[8] = 1;
            len = 0;
            switch (line[0]) {
            case 'R': report(); break;
            case 'L': measure_levels(); break;
            case 'T': check_tim1(); break;
            case 'A': check_adc(); break;
            case 'P': check_periph(); break;
            case 'S': check_spi(); break;
            case 'I': report_ids(); break;
            case 'N': {
                const char *a = line + 2;
                if (line[1] == 'W') {
                    uint32_t off = hexarg(&a), val = hexarg(&a);
                    uint32_t t0 = DWT_CYC;
                    int rc = core_nvm_write(off, &val, 4);
                    core_usb_printf("NVM write(0x%lx, 0x%08lx) rc=%d in %lu us\n", (unsigned long)off,
                                    (unsigned long)val, rc,
                                    (unsigned long)((DWT_CYC - t0) / (SYSCLK_HZ / 1000000UL)));
                } else if (line[1] == 'R') {
                    uint32_t off = hexarg(&a), val = 0;
                    int rc = core_nvm_read(off, &val, 4);
                    core_usb_printf("NVM read(0x%lx) rc=%d val=0x%08lx\n", (unsigned long)off, rc,
                                    (unsigned long)val);
                } else if (line[1] == 'E') {
                    core_usb_printf("NVM erase_all rc=%d\n", core_nvm_erase_all());
                } else {
                    report_nvm();
                }
                break;
            }
            case 'M': peek(line + 1); break;
            case 'W': poke(line + 1); break;
            case 'V': core_usb_printf("HELLO hw-h5-bringup\n"); break;
            case 'Q':           /* test the safety net: drop off the bus, lose the address */
                USB_BCDR &= ~USB_BCDR_DPPU_DPD;
                USB_DADDR = 0;
                break;
            case 'E': core_usb_printf("ECHO len=%u [%s]\n", (unsigned)strlen(line), line); break;
            case 'X':
                SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
                for (;;) { }
            default: break;
            }
        }
    }
}
