/**
 * port_core_nvm_hw.h — STM32 port: the hardware behind core_nvm.c (1 of 3)
 *
 * Included by core_nvm.c only, right after core_nvm.h; not for direct use.
 * core_nvm.c keeps the API, the range checks and the flash emulation's store
 * (page headers, records, CRC-32, replay, compaction); the port has what
 * touches the chip, under the names core_nvm.c calls. Three files, each
 * included where its code always sat in core_nvm.c (GCC lays the functions
 * out in the order they are defined, so one include at the top would move
 * them around in the image):
 *
 *   port_core_nvm_hw.h       (this file, at the top) the includes; Core.ST.L0's
 *                            port_core_nvm_eeprom_begin() / _wait() / _end()
 *                            (the data EEPROM's PECR unlock, the step out of
 *                            voltage range 3, the FLASH_SR checks); for the
 *                            flash emulation (L4, W5, H5) the H5's
 *                            NVM_LAYOUT_OK(), the double-ECC NMI handler (L4,
 *                            W5), port_core_nvm_hw_begin() / _hw_end() around
 *                            every call, port_core_nvm_flash_read() /
 *                            _flash_is_blank() (16-bit data-area reads on the
 *                            H5) and port_core_nvm_clear_ecc()
 *   port_core_nvm_hw_prog.h  (after the CRC-32) port_core_nvm_erase() /
 *                            _program(), with the cache upkeep and ST's flash
 *                            manager on a W5 with BLE or Thread
 *   port_core_nvm_h5_boot.h  (at the end) Core.ST.H5's first-boot switch of
 *                            the data area, _core_nvm_h5_boot()
 */

#ifndef PORT_CORE_NVM_HW_H
#define PORT_CORE_NVM_HW_H

#include "core_nvm.h"

#if CORE_NVM_FLASH_EMU
#include "ll_flash.h"
#if CORE_NVM_EDATA
#include "ll_flash_edata.h"
#endif
#include "ll_iwdg.h"
#if defined(STM32WBA55xx) && ((defined(BLE_ENABLED) && BLE_ENABLED) || \
                              (defined(THREAD_ENABLED) && THREAD_ENABLED))
#include "wpan_flash.h"
#define NVM_USE_FLASH_MANAGER 1
#else
#define NVM_USE_FLASH_MANAGER 0
#endif
#endif

#if CORE_NVM_EEPROM

/* ============================================================
 * Core.ST.L0 — data EEPROM
 * ============================================================ */

#define L0_FLASH_PECR        REG32(0x40022004UL)   /* FLASH_BASE + 0x04 */
#define L0_FLASH_PEKEYR      REG32(0x4002200CUL)   /* FLASH_BASE + 0x0C */
#define L0_FLASH_SR          REG32(0x40022018UL)   /* FLASH_BASE + 0x18 */
#define L0_PECR_PELOCK       (1UL << 0)
#define L0_PEKEY1            0x89ABCDEFUL          /* same across the L0 family */
#define L0_PEKEY2            0x02030405UL
#define L0_SR_BSY            (1UL << 0)
/* RM0377 §3.7.7: WRPERR 8, PGAERR 9, SIZERR 10, NOTZEROERR 16, FWWERR 17
 * (FWWERR: the write was aborted for a fetch and did not happen). */
#define L0_SR_ERR            ((1UL << 8) | (1UL << 9) | (1UL << 10) | (1UL << 16) | (1UL << 17))

#define L0_PWR_CR            REG32(0x40007000UL)
#define L0_PWR_CSR           REG32(0x40007004UL)
#define L0_RCC_APB1ENR       REG32(0x40021038UL)

/* Voltage range 3 (the "low" clock level) can't program the EEPROM
 * (RM0377 §6.1.4): step up to range 2 for the write and back after.
 * Range 3 runs at <= 4.2 MHz, which range 2 allows at 0 WS. */
static uint32_t port_core_nvm_eeprom_begin(void)
{
    uint32_t vos_was = (L0_PWR_CR >> 11) & 0x3UL;               /* PWR_CR.VOS */
    if (vos_was == 3u) {
        SET_BITS(L0_RCC_APB1ENR, (1UL << 28));                  /* PWREN */
        while (L0_PWR_CSR & (1UL << 4)) ;                       /* VOSF */
        MOD_BITS(L0_PWR_CR, 0x3UL << 11, 2UL << 11);
        while (L0_PWR_CSR & (1UL << 4)) ;
    }
    if (L0_FLASH_PECR & L0_PECR_PELOCK) {
        L0_FLASH_PEKEYR = L0_PEKEY1;
        L0_FLASH_PEKEYR = L0_PEKEY2;
    }
    L0_FLASH_SR = L0_SR_ERR;                                    /* rc_w1 */
    return vos_was;
}

static void port_core_nvm_eeprom_end(uint32_t vos_was)
{
    SET_BITS(L0_FLASH_PECR, L0_PECR_PELOCK);
    if (vos_was == 3u) {
        while (L0_PWR_CSR & (1UL << 4)) ;
        MOD_BITS(L0_PWR_CR, 0x3UL << 11, 3UL << 11);
        while (L0_PWR_CSR & (1UL << 4)) ;
    }
}

/* Wait for the write (tPROG ~3.2 ms) and report FLASH_SR errors; the audit
 * found this path never looked at them. */
static int port_core_nvm_eeprom_wait(void)
{
    while (L0_FLASH_SR & L0_SR_BSY)
        ;
    uint32_t sr = L0_FLASH_SR;
    if (sr & L0_SR_ERR) {
        L0_FLASH_SR = sr & L0_SR_ERR;
        return -1;
    }
    return 0;
}

#elif CORE_NVM_FLASH_EMU

/* ============================================================
 * Core.ST.L4 / Core.ST.W5 / Core.ST.H5 — flash emulation
 * ============================================================ */

extern const uint8_t __core_nvm_start[];     /* the linker script's (sdk/device/) */
#if CORE_NVM_EDATA && !defined(NVM_LAYOUT_OK)
/* H5: the images end below the two sectors' code address, the data area is
 * switched on for them (EDATA2_EN, EDATA2_STRT >= 1: the two last sectors),
 * and the banks are not swapped (with SWAP_BANK, bank 2's data would move to
 * 0x0900_0000, RM0481 §7.3.10). Checked before the area is touched: reading
 * it while it is off is a bus error (§7.3.10). */
static int port_core_nvm_h5_layout_ok(void)
{
    uint32_t e = FLASH_EDATA2R_CUR;
    return (uint32_t)(uintptr_t)__core_nvm_start == CORE_NVM_CODE_ADDR
        && !(FLASH_OPTSR_CUR & FLASH_OPTSR_SWAP_BANK)
        && (e & FLASH_EDATA_EN) && (e & FLASH_EDATA_STRT) >= 1u;
}
#define NVM_LAYOUT_OK()  port_core_nvm_h5_layout_ok()
#endif

/* Flash reads go through this, so tests/host-nvm-sim can run core_nvm.c on a
 * host against a simulated flash (with power cuts at every step). */
#ifndef NVM_MEM
#define NVM_MEM(addr)    ((uintptr_t)(addr))
#endif

#if !CORE_NVM_EDATA
/* ---- ECC (FLASH_ECCR) ---- */
#if defined(STM32L422xx)
#define NVM_FLASH_ECCR   REG32(FLASH_BASE + 0x18UL)    /* RM0394 §3.7.7 */
#define ECCR_ADDR_MASK   0x0007FFFFUL                  /* ADDR_ECC[18:0] */
#define ECCR_SYSF        (1UL << 20)
#else
#define NVM_FLASH_ECCR   REG32(FLASH_BASE + 0x30UL)    /* RM0493 §7.9.10 */
#define ECCR_ADDR_MASK   0x000FFFFFUL                  /* ADDR_ECC[19:0] */
#define ECCR_SYSF        (1UL << 22)
#endif
#define ECCR_ECCIE       (1UL << 24)
#define ECCR_ECCC        (1UL << 30)
#define ECCR_ECCD        (1UL << 31)

static volatile uint8_t s_ecc_hit;

#if defined(STM32WBA55xx)
/* The BLE update's reserved page (key + install marker log, page 124 below
 * core_nvm; docs/ble-update-protocol.md §7.1) can hold a quad-word a power
 * cut tore just the same. sdk/ble/ble_update.c clears this, reads, checks. */
#define OTA_PAGE_ADDR    0x080F8000UL
#define OTA_PAGE_SIZE    8192UL
volatile uint8_t _core_flash_ecc_ota;
#endif
#if defined(STM32WBA55xx) && defined(THREAD_ENABLED) && THREAD_ENABLED
/* The OpenThread settings pages (sdk/thread/thread_settings.c, linker script
 * symbols): a torn snapshot is caught the same way. thread_settings.c clears
 * this, reads, checks. */
extern const uint8_t __thread_settings_start[];
extern const uint8_t __thread_settings_end[];
volatile uint8_t _core_flash_ecc_thread;
#endif

/* A double ECC error raises an NMI (it is not a bus fault: the load completes
 * with bad data). Ours are the ones inside the NVM pages, which a torn
 * program can leave behind: note it for the read in progress and return. */
void NMI_Handler(void)
{
    uint32_t eccr = NVM_FLASH_ECCR;
    if ((eccr & ECCR_ECCD) && !(eccr & ECCR_SYSF)) {
        uint32_t a = FLASH_START + (eccr & ECCR_ADDR_MASK);
        if (a >= CORE_NVM_FLASH_ADDR && a < CORE_NVM_FLASH_ADDR + CORE_NVM_FLASH_SIZE) {
            NVM_FLASH_ECCR = (eccr & ECCR_ECCIE) | ECCR_ECCD;  /* rc_w1; keep ECCC */
            s_ecc_hit = 1;
            return;
        }
#if defined(STM32WBA55xx)
        if (a >= OTA_PAGE_ADDR && a < OTA_PAGE_ADDR + OTA_PAGE_SIZE) {
            NVM_FLASH_ECCR = (eccr & ECCR_ECCIE) | ECCR_ECCD;
            _core_flash_ecc_ota = 1;
            return;
        }
#endif
#if defined(STM32WBA55xx) && defined(THREAD_ENABLED) && THREAD_ENABLED
        if (a >= (uint32_t)(uintptr_t)__thread_settings_start &&
            a < (uint32_t)(uintptr_t)__thread_settings_end) {
            NVM_FLASH_ECCR = (eccr & ECCR_ECCIE) | ECCR_ECCD;
            _core_flash_ecc_thread = 1;
            return;
        }
#endif
    }
    for (;;)            /* anything else: what Default_Handler did */
        ;
}
#endif /* !CORE_NVM_EDATA */

#if defined(__arm__)
static inline void port_core_nvm_barrier(void)
{
    __asm volatile ("dsb 0xF\n\tisb 0xF" ::: "memory");
}

static inline uint32_t port_core_nvm_irq_save(void)
{
    uint32_t p;
    __asm volatile ("mrs %0, primask\n\tcpsid i" : "=r"(p) :: "memory");
    return p;
}

static inline void port_core_nvm_irq_restore(uint32_t p)
{
    __asm volatile ("msr primask, %0" :: "r"(p) : "memory");
}
#else   /* a host build: tests/host-nvm-sim */
static inline void port_core_nvm_barrier(void) { }
static inline uint32_t port_core_nvm_irq_save(void) { return 0; }
static inline void port_core_nvm_irq_restore(uint32_t p) { (void)p; }
#endif

#if CORE_NVM_EDATA
/* H5 data area: every access is 16 bits (an 8-bit read is a bus error,
 * RM0481 §7.3.4) through ll_flash_edata_*, between port_core_nvm_hw_begin()
 * and port_core_nvm_hw_end() (ICACHE off, the double-ECC NMI masked; see
 * ll_flash.h). */
static ll_flash_edata_ctx_t s_ed;
static uint32_t s_ecc_bad;          /* words read with an uncorrectable error */
static void port_core_nvm_hw_begin(void) { ll_flash_edata_begin(&s_ed); }
static void port_core_nvm_hw_end(void)   { ll_flash_edata_end(&s_ed); }

uint32_t _core_nvm_h5_ecc_errors(void) { return s_ecc_bad; }

/* Copy from the data area. A virgin word reads 0xFFFF, like erased flash.
 * Returns -1 if any word read with an uncorrectable ECC error, 0 otherwise. */
static int port_core_nvm_flash_read(uint32_t addr, void *dst, uint32_t len)
{
    uint8_t *d = (uint8_t *)dst;
    int bad = 0;
    while (len) {
        uint32_t a = addr & ~1UL;
        uint16_t h;
        if (ll_flash_edata_read_half(a, &h) < 0) { bad = 1; s_ecc_bad++; }
        for (uint32_t k = addr - a; k < 2u && len; k++, len--, addr++)
            *d++ = (uint8_t)(h >> (8u * k));
    }
    return bad ? -1 : 0;
}

/* 1 if every 16-bit word is virgin: erased and never written since. A word
 * written with 0xFFFF reads the same but is not blank. */
static int port_core_nvm_flash_is_blank(uint32_t addr, uint32_t len)
{
    for (uint32_t i = 0; i < len; i += 2u) {
        uint16_t h;
        if (ll_flash_edata_read_half(addr + i, &h) != 1) return 0;
    }
    return 1;
}
#else
static inline __attribute__((always_inline)) void port_core_nvm_hw_begin(void) { }
static inline __attribute__((always_inline)) void port_core_nvm_hw_end(void) { }

/* Copy from flash. Returns -1 if any of it read with an uncorrectable ECC
 * error (the NMI above fired), 0 otherwise. */
static int port_core_nvm_flash_read(uint32_t addr, void *dst, uint32_t len)
{
    s_ecc_hit = 0;
    const volatile uint8_t *s = (const volatile uint8_t *)NVM_MEM(addr);
    uint8_t *d = (uint8_t *)dst;
    for (uint32_t i = 0; i < len; i++) d[i] = s[i];
    port_core_nvm_barrier();
    return s_ecc_hit ? -1 : 0;
}

static int port_core_nvm_flash_is_blank(uint32_t addr, uint32_t len)
{
    s_ecc_hit = 0;
    const volatile uint32_t *p = (const volatile uint32_t *)NVM_MEM(addr);
    uint32_t acc = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < len / 4u; i++) acc &= p[i];
    port_core_nvm_barrier();
    return acc == 0xFFFFFFFFUL && !s_ecc_hit;
}
#endif

/* Before the first scan of the pages. */
static inline __attribute__((always_inline)) void port_core_nvm_clear_ecc(void)
{
#if !CORE_NVM_EDATA     /* H5: ll_flash_edata_begin() clears its flags */
    /* A latched single-bit correction (ECCC) stops ECCD from being reported
     * (RM0493 §7.9.10), so start from clear flags. */
    NVM_FLASH_ECCR = (NVM_FLASH_ECCR & ECCR_ECCIE) | ECCR_ECCC | ECCR_ECCD;
#endif
}

#endif /* CORE_NVM_EEPROM / CORE_NVM_FLASH_EMU */

#endif /* PORT_CORE_NVM_HW_H */
