/**
 * core_nvm.c — core_nvm on every Core (API and behaviour: core_nvm.h).
 *
 * Compiled into every build, like core_watchdog.c; --gc-sections drops it
 * when a project never calls core_nvm. The one exception is the NMI handler on
 * the L4 and W5, which the vector table always pulls in (see below).
 *
 *   Core.ST.L0  data EEPROM, written byte by byte (the header-only code that
 *               was here before, plus FLASH_SR checks and a read-back).
 *   Core.ST.L4  flash emulation (this file), page erase/program via ll_flash.
 *   Core.ST.W5  flash emulation (this file); with BLE running, every erase and
 *               program goes through ST's flash manager (sdk/ble/ble_flash.c).
 *   Core.ST.H5  flash emulation (this file) in the high-cycle data area
 *               (EDATA), written 16 bits at a time; plus the first-boot switch
 *               that turns the area on (_core_nvm_h5_boot, at the end).
 *
 * ---- Flash emulation: layout ----
 *
 * Two pages, A and B, at CORE_NVM_FLASH_ADDR. A unit (UNIT) is the program
 * granule: 8 B on the L4, 16 B on the W5. Everything is written in whole
 * units, in address order, and never rewritten until the page is erased.
 * On the H5 a unit is 8 B too, but the data area programs 16-bit words, so a
 * unit is four programs; a power cut can tear it anywhere, which is the same
 * torn-record case as below.
 *
 *   unit 0      page header: 'NVM1' magic, generation (u32)
 *   unit 1      commit: ~generation, CRC-32 of (magic, generation)
 *   unit 2...   records, back to back, each padded to a whole unit:
 *                 u16 offset, u16 length, u32 CRC-32 of (offset, length, data)
 *                 data[length]
 *
 * The valid page (header and commit both correct) with the highest
 * generation is active. Its records, replayed in order over an image of
 * 0xFF, give the stored bytes. The first record that is not valid ends the log.
 *
 * ---- Writes and compaction ----
 *
 * A write appends one record, then reads it back. When the record does not fit
 * (or the tail of the page is unusable, below), the write compacts instead:
 * erase the other page, write its header, copy the current contents into it as
 * 128-byte records (all-0xFF chunks skipped) with the new data merged in,
 * verify, and write the commit unit last. The old page is left as it is until
 * the next compaction erases it.
 *
 * ---- Power loss ----
 *
 *   - During an append: the torn record fails its CRC (or reads with an ECC
 *     error), so the log ends before it and the old bytes stand. The tail is
 *     marked unusable and the next write compacts.
 *   - During a compaction: until the commit unit is complete, the new page
 *     has no valid commit and the old page stays active (old value). Once it
 *     is, the new page has the higher generation (new value). The erase at the
 *     start only ever touches the page that is not active.
 *   - A torn program can leave a unit whose ECC can't be corrected: reading it
 *     raises an NMI (RM0394 §3.3.2, RM0493 §7.3.2). The NMI handler here
 *     recognises a double-ECC error inside the NVM pages, clears it and flags
 *     the read, so the scan treats that unit as invalid instead of hanging in
 *     Default_Handler at every boot. Any other NMI still goes to the same
 *     endless loop it did before.
 *
 * ST's EEPROM emulation (AN4894) uses the same two-page idea with per-page
 * state words; this keeps it to a generation number and a commit unit, and
 * makes a multi-byte write one CRC-checked record so it is atomic.
 */

#include "core_nvm.h"

#if CORE_NVM_FLASH_EMU
#include "ll_flash.h"
#if CORE_NVM_EDATA
#include "ll_flash_edata.h"
#endif
#include "ll_iwdg.h"
#if defined(STM32WBA55xx) && defined(BLE_ENABLED) && BLE_ENABLED
#include "ble_flash.h"
#define NVM_USE_FLASH_MANAGER 1
#else
#define NVM_USE_FLASH_MANAGER 0
#endif
#endif

/* ============================================================
 * Core.ST.L0 — data EEPROM
 * ============================================================ */

#if CORE_NVM_EEPROM

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
static uint32_t l0_begin(void)
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

static void l0_end(uint32_t vos_was)
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
static int l0_wait(void)
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

int core_nvm_read(uint32_t offset, void *buf, uint32_t len)
{
    /* Written this way, not `offset + len > SIZE`, because that sum wraps:
     * a large offset with a small len passes the naive check and then
     * reads or writes outside the region. */
    if (len > (uint32_t)CORE_NVM_SIZE ||
        offset > (uint32_t)CORE_NVM_SIZE - len)
        return CORE_NVM_ERR_RANGE;

    memcpy(buf, (const void *)(CORE_NVM_BASE + offset), len);
    return CORE_NVM_OK;
}

int core_nvm_write(uint32_t offset, const void *data, uint32_t len)
{
    if (len > (uint32_t)CORE_NVM_SIZE ||
        offset > (uint32_t)CORE_NVM_SIZE - len)
        return CORE_NVM_ERR_RANGE;

    /* True EEPROM: the hardware erases before it writes. */
    uint32_t vos_was = l0_begin();
    const uint8_t *src = (const uint8_t *)data;
    volatile uint8_t *dst = (volatile uint8_t *)(CORE_NVM_BASE + offset);
    int rc = CORE_NVM_OK;
    for (uint32_t i = 0; i < len; i++) {
        dst[i] = src[i];
        if (l0_wait() != 0 || dst[i] != src[i]) {
            rc = CORE_NVM_ERR_FLASH;
            break;
        }
    }
    l0_end(vos_was);
    return rc;
}

int core_nvm_erase_all(void)
{
    /* Word writes of 0 (an EEPROM erase: RM0377 §3.3.4). Words already 0 are
     * skipped, so a cleared EEPROM costs nothing. */
    uint32_t vos_was = l0_begin();
    volatile uint32_t *w = (volatile uint32_t *)CORE_NVM_BASE;
    int rc = CORE_NVM_OK;
    for (uint32_t i = 0; i < (uint32_t)CORE_NVM_SIZE / 4u; i++) {
        if (w[i] == 0u) continue;
        w[i] = 0u;
        if (l0_wait() != 0 || w[i] != 0u) {
            rc = CORE_NVM_ERR_FLASH;
            break;
        }
    }
    l0_end(vos_was);
    return rc;
}

/* ============================================================
 * Core.ST.L4 / Core.ST.W5 — flash emulation
 * ============================================================ */

#elif CORE_NVM_FLASH_EMU

#define UNIT       CORE_NVM_PROG_UNIT
#define PAGE       CORE_NVM_PAGE_SIZE
#define HDR_BYTES  (2u * UNIT)              /* header unit + commit unit */
#define REC_HDR    8u                       /* offset, length, CRC */
#define CHUNK      128u                     /* compaction record size */
#define MAGIC      0x314D564EUL             /* "NVM1" */
#define ROUND_U(n) (((n) + (UNIT - 1u)) & ~(UNIT - 1u))

/* ---- The linker script must reserve the pages (sdk/device/ scripts) ---- */
extern const uint8_t __core_nvm_start[];
#if CORE_NVM_EDATA && !defined(NVM_LAYOUT_OK)
/* H5: the images end below the two sectors' code address, the data area is
 * switched on for them (EDATA2_EN, EDATA2_STRT >= 1: the two last sectors),
 * and the banks are not swapped (with SWAP_BANK, bank 2's data would move to
 * 0x0900_0000, RM0481 §7.3.10). Checked before the area is touched: reading
 * it while it is off is a bus error (§7.3.10). */
static int nvm_layout_ok(void)
{
    uint32_t e = FLASH_EDATA2R_CUR;
    return (uint32_t)(uintptr_t)__core_nvm_start == CORE_NVM_CODE_ADDR
        && !(FLASH_OPTSR_CUR & FLASH_OPTSR_SWAP_BANK)
        && (e & FLASH_EDATA_EN) && (e & FLASH_EDATA_STRT) >= 1u;
}
#define NVM_LAYOUT_OK()  nvm_layout_ok()
#endif
#ifndef NVM_LAYOUT_OK
#define NVM_LAYOUT_OK()  ((uint32_t)(uintptr_t)__core_nvm_start == CORE_NVM_FLASH_ADDR)
#endif

/* Flash reads go through this, so tests/host-nvm-sim can run this file on a
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
    }
    for (;;)            /* anything else: what Default_Handler did */
        ;
}
#endif /* !CORE_NVM_EDATA */

#if defined(__arm__)
static inline void barrier(void)
{
    __asm volatile ("dsb 0xF\n\tisb 0xF" ::: "memory");
}

static inline uint32_t irq_save(void)
{
    uint32_t p;
    __asm volatile ("mrs %0, primask\n\tcpsid i" : "=r"(p) :: "memory");
    return p;
}

static inline void irq_restore(uint32_t p)
{
    __asm volatile ("msr primask, %0" :: "r"(p) : "memory");
}
#else   /* a host build: tests/host-nvm-sim */
static inline void barrier(void) { }
static inline uint32_t irq_save(void) { return 0; }
static inline void irq_restore(uint32_t p) { (void)p; }
#endif

#if CORE_NVM_EDATA
/* H5 data area: every access is 16 bits (an 8-bit read is a bus error,
 * RM0481 §7.3.4) through ll_flash_edata_*, between hw_begin() and hw_end()
 * (ICACHE off, the double-ECC NMI masked; see ll_flash.h). */
static ll_flash_edata_ctx_t s_ed;
static uint32_t s_ecc_bad;          /* words read with an uncorrectable error */
static void hw_begin(void) { ll_flash_edata_begin(&s_ed); }
static void hw_end(void)   { ll_flash_edata_end(&s_ed); }

uint32_t _core_nvm_h5_ecc_errors(void) { return s_ecc_bad; }

/* Copy from the data area. A virgin word reads 0xFFFF, like erased flash.
 * Returns -1 if any word read with an uncorrectable ECC error, 0 otherwise. */
static int flash_read(uint32_t addr, void *dst, uint32_t len)
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
static int flash_is_blank(uint32_t addr, uint32_t len)
{
    for (uint32_t i = 0; i < len; i += 2u) {
        uint16_t h;
        if (ll_flash_edata_read_half(addr + i, &h) != 1) return 0;
    }
    return 1;
}
#else
static inline __attribute__((always_inline)) void hw_begin(void) { }
static inline __attribute__((always_inline)) void hw_end(void) { }

/* Copy from flash. Returns -1 if any of it read with an uncorrectable ECC
 * error (the NMI above fired), 0 otherwise. */
static int flash_read(uint32_t addr, void *dst, uint32_t len)
{
    s_ecc_hit = 0;
    const volatile uint8_t *s = (const volatile uint8_t *)NVM_MEM(addr);
    uint8_t *d = (uint8_t *)dst;
    for (uint32_t i = 0; i < len; i++) d[i] = s[i];
    barrier();
    return s_ecc_hit ? -1 : 0;
}

static int flash_is_blank(uint32_t addr, uint32_t len)
{
    s_ecc_hit = 0;
    const volatile uint32_t *p = (const volatile uint32_t *)NVM_MEM(addr);
    uint32_t acc = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < len / 4u; i++) acc &= p[i];
    barrier();
    return acc == 0xFFFFFFFFUL && !s_ecc_hit;
}
#endif

/* ---- CRC-32 (IEEE, as zlib), nibble table ---- */
static uint32_t crc32_upd(uint32_t c, const uint8_t *p, uint32_t n)
{
    static const uint32_t t[16] = {
        0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
        0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
        0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
        0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL,
    };
    while (n--) {
        c ^= *p++;
        c = (c >> 4) ^ t[c & 0xFu];
        c = (c >> 4) ^ t[c & 0xFu];
    }
    return c;
}

/* ============================================================
 * Hardware: erase one page, program whole units
 * ============================================================ */

static void caches_after_erase(void)
{
#if defined(STM32L422xx)
    /* RM0394 §3.3.7 "Programming and caches": an erase is not reflected in the
     * data cache. Flush it (only allowed while it is disabled). The pages
     * hold no code, so the instruction cache never has them. */
    uint32_t acr = FLASH_ACR;
    if (acr & (1UL << 10)) {                        /* DCEN */
        FLASH_ACR = acr & ~(1UL << 10);
        FLASH_ACR = (acr & ~(1UL << 10)) | (1UL << 12);   /* DCRST */
        FLASH_ACR = acr & ~(1UL << 10);
        FLASH_ACR = acr;
    }
#endif
}

#if defined(STM32WBA55xx)
/* The ICACHE (enabled by BLE builds) sits on C-AHB and also caches data reads
 * of flash: invalidate it so a read-back sees the array (RM0493 §8.4.8). */
#define ICACHE_CR   REG32(0x40030400UL)
#define ICACHE_SR   REG32(0x40030404UL)
#define ICACHE_FCR  REG32(0x4003040CUL)
static void icache_invalidate(void)
{
    if (!(ICACHE_CR & 1UL)) return;                 /* EN */
    ICACHE_FCR = (1UL << 1);                        /* CBSYENDF */
    ICACHE_CR |= (1UL << 1);                        /* CACHEINV */
    while (!(ICACHE_SR & (1UL << 1)))               /* BSYENDF */
        ;
}
#else
static inline void icache_invalidate(void) { }
#endif

static int hw_erase(uint32_t page)
{
    /* The L4 stalls ~22 ms here with every interrupt held off: feed the
     * watchdog first (a no-op if it isn't running). */
    ll_iwdg_refresh();
    int rc;
#if NVM_USE_FLASH_MANAGER
    if (ble_flash_ready()) {
        rc = ble_flash_erase_page(page);
    } else
#endif
    {
        ll_flash_unlock();
        rc = ll_flash_erase_page(page);
        ll_flash_lock();
    }
    caches_after_erase();
    icache_invalidate();
    return rc;
}

/* Program len bytes (a whole number of units) at a unit-aligned address. */
static int hw_program(uint32_t addr, const uint8_t *src, uint32_t len)
{
    int rc = 0;
#if CORE_NVM_EDATA
    /* 16-bit words, each its own program (RM0481 §7.3.5). No interrupt
     * masking: nothing is left half-written between two words (there is no
     * write buffer for the data area), and a reset between them is a torn
     * record like a power cut. Code keeps running from bank 1 meanwhile. */
    ll_flash_unlock();
    for (uint32_t i = 0; i < len && rc == 0; i += 2u)
        rc = ll_flash_edata_program_half(addr + i, (uint16_t)(src[i] | ((uint16_t)src[i + 1u] << 8)));
    ll_flash_lock();
    return rc;
#else
#if NVM_USE_FLASH_MANAGER
    if (ble_flash_ready()) {
        rc = ble_flash_program(addr, (const uint32_t *)(const void *)src, len / 4u);
        icache_invalidate();
        return rc;
    }
#endif
    ll_flash_unlock();
    for (uint32_t i = 0; i < len && rc == 0; i += UNIT) {
        uint32_t w[UNIT / 4u];
        memcpy(w, src + i, UNIT);
        /* One unit is one program sequence: keep an interrupt from landing
         * between its words (an ISR could reset into the serial-update
         * flasher, for instance). ~82 us (L4) / ~118 us (W5). */
        uint32_t p = irq_save();
#if defined(STM32L422xx)
        rc = ll_flash_program_dword(addr + i, w[0], w[1]);
#else
        rc = ll_flash_program_qword(addr + i, w);
#endif
        irq_restore(p);
    }
    ll_flash_lock();
    icache_invalidate();
    return rc;
#endif
}

/* ============================================================
 * State
 * ============================================================ */

static struct {
    uint8_t  mounted;
    int8_t   active;        /* 0 / 1, or -1: no valid page (all bytes 0xFF) */
    uint8_t  dirty;         /* tail of the active page unusable: compact next */
    uint32_t gen;           /* highest valid generation seen */
    uint32_t end;           /* first free byte in the active page */
} s;

static volatile uint8_t s_busy;

/* Staging buffer for records: whole units, so the flash-manager path can hand
 * it over in one call. */
#define STAGE 256u
static uint32_t s_stage[STAGE / 4u];

static inline uint32_t page_addr(int p)
{
    return CORE_NVM_FLASH_ADDR + (uint32_t)p * PAGE;
}

/* Returns 1 and the generation if page p has a valid header and commit. */
static int read_header(int p, uint32_t *gen)
{
    uint32_t h[2], c[2];
    if (flash_read(page_addr(p), h, 8) != 0) return 0;
    if (flash_read(page_addr(p) + UNIT, c, 8) != 0) return 0;
    if (h[0] != MAGIC || h[1] == 0u || h[1] == 0xFFFFFFFFUL) return 0;
    if (c[0] != ~h[1] || c[1] != ~crc32_upd(0xFFFFFFFFUL, (const uint8_t *)h, 8)) return 0;
    *gen = h[1];
    return 1;
}

/* Record header at addr: 1 = valid record (off/len out), 0 = erased (end of
 * a clean log), -1 = invalid (torn or garbage). Checks the CRC. */
static int read_record(uint32_t addr, uint32_t limit, uint32_t *off, uint32_t *len)
{
    uint32_t h[2];
    if (flash_read(addr, h, 8) != 0) return -1;
    if (h[0] == 0xFFFFFFFFUL && h[1] == 0xFFFFFFFFUL)
        return flash_is_blank(addr, UNIT) ? 0 : -1;    /* a torn unit can be FF up front */
    uint32_t o = h[0] & 0xFFFFu, n = h[0] >> 16;
    if (n == 0u || n > (uint32_t)CORE_NVM_SIZE || o > (uint32_t)CORE_NVM_SIZE - n) return -1;
    if (ROUND_U(REC_HDR + n) > limit - addr) return -1;

    uint32_t c = crc32_upd(0xFFFFFFFFUL, (const uint8_t *)h, 4);
    uint8_t buf[64];
    for (uint32_t i = 0; i < n; i += sizeof(buf)) {
        uint32_t k = (n - i) < sizeof(buf) ? (n - i) : sizeof(buf);
        if (flash_read(addr + REC_HDR + i, buf, k) != 0) return -1;
        c = crc32_upd(c, buf, k);
    }
    if (~c != h[1]) return -1;
    *off = o;
    *len = n;
    return 1;
}

/* Walk the active page's log: sets s.end (first free byte) and s.dirty. */
static void scan(void)
{
    uint32_t base = page_addr(s.active), limit = base + PAGE;
    uint32_t pos = base + HDR_BYTES;
    s.dirty = 0;
    while (pos < limit) {
        uint32_t o, n;
        int r = read_record(pos, limit, &o, &n);
        if (r == 0) break;
        if (r < 0) { s.dirty = 1; break; }
        pos += ROUND_U(REC_HDR + n);
    }
    s.end = pos - base;
}

static int mount(void)
{
    if (s.mounted) return CORE_NVM_OK;
    if (!NVM_LAYOUT_OK())
        return CORE_NVM_ERR_LAYOUT;

#if !CORE_NVM_EDATA     /* H5: ll_flash_edata_begin() clears its flags */
    /* A latched single-bit correction (ECCC) stops ECCD from being reported
     * (RM0493 §7.9.10), so start from clear flags. */
    NVM_FLASH_ECCR = (NVM_FLASH_ECCR & ECCR_ECCIE) | ECCR_ECCC | ECCR_ECCD;
#endif

    uint32_t g0 = 0, g1 = 0;
    int v0 = read_header(0, &g0), v1 = read_header(1, &g1);
    if (v0 && v1)  s.active = (g1 > g0) ? 1 : 0;
    else if (v0)   s.active = 0;
    else if (v1)   s.active = 1;
    else           s.active = -1;
    s.gen = (g0 > g1) ? g0 : g1;
    if (s.active >= 0) scan();
    s.mounted = 1;
    return CORE_NVM_OK;
}

/* The stored bytes [off, off + len) from the active page (0xFF if none). */
static int replay(uint32_t off, uint8_t *dst, uint32_t len)
{
    memset(dst, 0xFF, len);
    if (s.active < 0) return CORE_NVM_OK;
    uint32_t base = page_addr(s.active);
    uint32_t pos = base + HDR_BYTES, end = base + s.end;
    while (pos < end) {
        uint32_t h;
        if (flash_read(pos, &h, 4) != 0) return CORE_NVM_ERR_FLASH;
        uint32_t o = h & 0xFFFFu, n = h >> 16;
        uint32_t lo = (o > off) ? o : off;
        uint32_t hi = (o + n < off + len) ? o + n : off + len;
        if (lo < hi &&
            flash_read(pos + REC_HDR + (lo - o), dst + (lo - off), hi - lo) != 0)
            return CORE_NVM_ERR_FLASH;
        pos += ROUND_U(REC_HDR + n);
    }
    return CORE_NVM_OK;
}

/* Program one record at addr: header, data, 0xFF padding to a whole unit. */
static int program_record(uint32_t addr, uint32_t off, const uint8_t *data, uint32_t len)
{
    uint32_t h[2];
    h[0] = (off & 0xFFFFu) | (len << 16);
    h[1] = ~crc32_upd(crc32_upd(0xFFFFFFFFUL, (const uint8_t *)h, 4), data, len);

    uint8_t *st = (uint8_t *)s_stage;
    uint32_t total = ROUND_U(REC_HDR + len), done = 0;
    while (done < total) {
        uint32_t n = (total - done) < STAGE ? (total - done) : STAGE;
        for (uint32_t i = 0; i < n; i++) {
            uint32_t k = done + i;
            st[i] = (k < REC_HDR) ? ((const uint8_t *)h)[k]
                  : (k < REC_HDR + len) ? data[k - REC_HDR] : 0xFFu;
        }
        if (hw_program(addr + done, st, n) != 0) return CORE_NVM_ERR_FLASH;
        done += n;
    }

    /* Read back: a program that reported no error can still be short. */
    uint32_t o, n;
    if (read_record(addr, addr + total, &o, &n) != 1 || o != off || n != len)
        return CORE_NVM_ERR_FLASH;
    uint8_t buf[64];
    for (uint32_t i = 0; i < len; i += sizeof(buf)) {
        uint32_t k = (len - i) < sizeof(buf) ? (len - i) : sizeof(buf);
        if (flash_read(addr + REC_HDR + i, buf, k) != 0 || memcmp(buf, data + i, k) != 0)
            return CORE_NVM_ERR_FLASH;
    }
    return CORE_NVM_OK;
}

/* Copy the current contents (with [off, off+len) replaced by data, or
 * nothing at all if wipe) into the other page and make it active. */
static int compact(uint32_t off, const uint8_t *data, uint32_t len, int wipe)
{
    int tgt = (s.active < 0) ? 0 : 1 - s.active;
    uint32_t base = page_addr(tgt);
    uint32_t gen = s.gen + 1u;
    if (gen == 0xFFFFFFFFUL) gen = 1u;          /* 4e9 compactions: never */

    /* Always erase, even a page that reads blank: an erase cut short by a
     * reset can leave cells that read 1 without being fully erased. */
    if (hw_erase(CORE_NVM_FIRST_PAGE + (uint32_t)tgt) != 0 || !flash_is_blank(base, PAGE))
        return CORE_NVM_ERR_FLASH;

    uint32_t u[UNIT / 4u];
    memset(u, 0xFF, sizeof(u));
    u[0] = MAGIC;
    u[1] = gen;
    if (hw_program(base, (const uint8_t *)u, UNIT) != 0) return CORE_NVM_ERR_FLASH;
    uint32_t hcrc = ~crc32_upd(0xFFFFFFFFUL, (const uint8_t *)u, 8);

    uint32_t pos = HDR_BYTES;
    for (uint32_t c = 0; !wipe && c < (uint32_t)CORE_NVM_SIZE; c += CHUNK) {
        uint8_t buf[CHUNK];
        if (replay(c, buf, CHUNK) != 0) return CORE_NVM_ERR_FLASH;
        uint32_t lo = (off > c) ? off : c;
        uint32_t hi = (off + len < c + CHUNK) ? off + len : c + CHUNK;
        if (lo < hi) memcpy(buf + (lo - c), data + (lo - off), hi - lo);

        int blank = 1;
        for (uint32_t i = 0; i < CHUNK; i++) if (buf[i] != 0xFFu) { blank = 0; break; }
        if (blank) continue;

        if (program_record(base + pos, c, buf, CHUNK) != 0) return CORE_NVM_ERR_FLASH;
        pos += ROUND_U(REC_HDR + CHUNK);
    }

    /* Commit: from here the new page wins. */
    memset(u, 0xFF, sizeof(u));
    u[0] = ~gen;
    u[1] = hcrc;
    if (hw_program(base + UNIT, (const uint8_t *)u, UNIT) != 0) return CORE_NVM_ERR_FLASH;
    uint32_t g;
    if (!read_header(tgt, &g) || g != gen) return CORE_NVM_ERR_FLASH;

    s.active = (int8_t)tgt;
    s.gen = gen;
    s.end = pos;
    s.dirty = 0;
    return CORE_NVM_OK;
}

/* 1 if [off, off+len) already holds data. */
static int unchanged(uint32_t off, const uint8_t *data, uint32_t len)
{
    uint8_t buf[64];
    for (uint32_t i = 0; i < len; i += sizeof(buf)) {
        uint32_t k = (len - i) < sizeof(buf) ? (len - i) : sizeof(buf);
        if (replay(off + i, buf, k) != 0 || memcmp(buf, data + i, k) != 0) return 0;
    }
    return 1;
}

static int do_write(uint32_t off, const uint8_t *data, uint32_t len)
{
    if (unchanged(off, data, len)) return CORE_NVM_OK;
    if (s.active < 0) return compact(off, data, len, 0);

    uint32_t need = ROUND_U(REC_HDR + len);
    uint32_t at = page_addr(s.active) + s.end;
    if (!s.dirty && s.end + need <= PAGE && flash_is_blank(at, need)) {
        if (program_record(at, off, data, len) == 0) {
            s.end += need;
            return CORE_NVM_OK;
        }
    }
    /* Full, or the tail is unusable (a torn or failed record): compact. */
    s.dirty = 1;
    return compact(off, data, len, 0);
}

/* ============================================================
 * API
 * ============================================================ */

/* Always inlined: on the L4 / W5 this is exactly the `s_busy = 0` it was. */
static inline __attribute__((always_inline)) void leave(void)
{
    hw_end();
    s_busy = 0;
}

static int enter(void)
{
    if (s_busy) return CORE_NVM_ERR_BUSY;
    s_busy = 1;
    hw_begin();
    int rc = mount();
    if (rc != CORE_NVM_OK) leave();
    return rc;
}

int core_nvm_read(uint32_t offset, void *buf, uint32_t len)
{
    /* Written this way, not `offset + len > SIZE`, because that sum wraps. */
    if (len > (uint32_t)CORE_NVM_SIZE ||
        offset > (uint32_t)CORE_NVM_SIZE - len)
        return CORE_NVM_ERR_RANGE;
    int rc = enter();
    if (rc != CORE_NVM_OK) return rc;
    rc = replay(offset, (uint8_t *)buf, len);
    leave();
    return rc;
}

int core_nvm_write(uint32_t offset, const void *data, uint32_t len)
{
    if (len > (uint32_t)CORE_NVM_SIZE ||
        offset > (uint32_t)CORE_NVM_SIZE - len)
        return CORE_NVM_ERR_RANGE;
    if (len == 0u) return CORE_NVM_OK;
    int rc = enter();
    if (rc != CORE_NVM_OK) return rc;
    rc = do_write(offset, (const uint8_t *)data, len);
    leave();
    return rc;
}

/* Not API: bytes left in the active page before the next compaction (0 when
 * the next write compacts anyway). tests/hw-nvm-flash uses it to aim its
 * power cuts at appends and at compactions. */
uint32_t _core_nvm_log_free(void)
{
    if (enter() != CORE_NVM_OK) return 0;
    uint32_t f = (s.active < 0 || s.dirty) ? 0u : PAGE - s.end;
    leave();
    return f;
}

int core_nvm_erase_all(void)
{
    int rc = enter();
    if (rc != CORE_NVM_OK) return rc;
    if (s.active >= 0 && (s.dirty || s.end > HDR_BYTES)) {
        /* Records may all be 0xFF already; only erase if something isn't. */
        int empty = 1;
        uint8_t buf[CHUNK];
        for (uint32_t c = 0; empty && c < (uint32_t)CORE_NVM_SIZE; c += CHUNK) {
            if (replay(c, buf, CHUNK) != 0) { empty = 0; break; }
            for (uint32_t i = 0; i < CHUNK; i++) if (buf[i] != 0xFFu) { empty = 0; break; }
        }
        if (!empty || s.dirty) rc = compact(0, 0, 0, 1);
    }
    leave();
    return rc;
}

#if CORE_NVM_EDATA && defined(__arm__)
/* ============================================================
 * Core.ST.H5 — switching the data area on at first boot
 * (core_nvm.h has the rules; RM0481 §7.4.3 has the sequence)
 * ============================================================ */

#include "hal_dfu.h"        /* hal_dfu_started_by_rom, SCB_AIRCR */

#define PROV_MAGIC  0xEDA7A2B0UL

/* Kept across the reset below: startup neither copies nor zeroes .noinit
 * (sdk/device/stm32h523he.ld), and a system reset keeps SRAM on this board,
 * as hal_dfu.h's recovery words also rely on. */
static core_nvm_h5_prov_t s_prov __attribute__((section(".noinit")));

/* The option registers with a _CUR / _PRG pair (RM0481 §7.11.19-53), the
 * order of core_nvm_h5_prov_t.ob_before / ob_after. How the check before a
 * change treats each _PRG:
 *   PAIR_EQ     must equal its _CUR, or the change is refused;
 *   PAIR_EPOCH  an epoch counter: OPTSTRT changes it only to a value above
 *               _CUR (§7.4.7, Table 52), so a _PRG at or below _CUR (the
 *               factory state reads _PRG 0, _CUR 1) changes nothing; above
 *               it would, and is refused;
 *   PAIR_SEC    secure-only, RAZ/WI to this non-secure code (§7.11.27-28,
 *               .33-34, .46-47): nothing here can read or change it. */
enum { PAIR_EQ, PAIR_EPOCH, PAIR_SEC };
static const uint16_t k_ob_pairs[CORE_NVM_H5_OB_N][3] = {
    { 0x050, 0x054, PAIR_EQ },  { 0x060, 0x064, PAIR_EPOCH }, { 0x068, 0x06C, PAIR_EPOCH },
    { 0x070, 0x074, PAIR_EQ },  { 0x080, 0x084, PAIR_EQ },    { 0x088, 0x08C, PAIR_SEC },
    { 0x090, 0x094, PAIR_EQ },  { 0x0E0, 0x0E4, PAIR_SEC },   { 0x0E8, 0x0EC, PAIR_EQ },
    { 0x0F0, 0x0F4, PAIR_EQ },  { 0x0F8, 0x0FC, PAIR_EQ },    { 0x1E0, 0x1E4, PAIR_SEC },
    { 0x1E8, 0x1EC, PAIR_EQ },  { 0x1F0, 0x1F4, PAIR_EQ },    { 0x1F8, 0x1FC, PAIR_EQ },
};
/* OPTSR, NSEPOCHR, SECEPOCHR, OPTSR2, NSBOOTR, SECBOOTR, OTPBLR, SECWM1R,
 * WRP1R, EDATA1R, HDP1R, SECWM2R, WRP2R, EDATA2R, HDP2R */

static void ob_snapshot(uint32_t *v)
{
    for (uint32_t i = 0; i < CORE_NVM_H5_OB_N; i++) v[i] = REG32(FLASH_BASE + k_ob_pairs[i][0]);
}

static int edata2_on(uint32_t e)
{
    return (e & FLASH_EDATA_EN) && (e & FLASH_EDATA_STRT) >= 1u;
}

const core_nvm_h5_prov_t *_core_nvm_h5_prov(void)
{
    return s_prov.magic == PROV_MAGIC ? &s_prov : 0;
}

void _core_nvm_h5_boot(void)
{
    if (s_prov.magic != PROV_MAGIC) {           /* power-on: a new record */
        memset(&s_prov, 0, sizeof s_prov);
        s_prov.magic = PROV_MAGIC;
    }
    s_prov.boots++;
    s_prov.opsr   = FLASH_OPSR;
    s_prov.optsr  = FLASH_OPTSR_CUR;
    s_prov.edata1 = FLASH_EDATA1R_CUR;
    uint32_t cur  = FLASH_EDATA2R_CUR;
    if (edata2_on(cur)) {                       /* the usual case: nothing to do */
        if (!s_prov.writes) s_prov.edata2_before = s_prov.edata2_after = cur;
        s_prov.state = CORE_NVM_H5_ENABLED;
        return;
    }
    if (s_prov.writes) {                        /* one change per power-on, whatever */
        s_prov.state = CORE_NVM_H5_RETRY;       /* happened to the last one: no loop */
        return;
    }
    if (hal_dfu_started_by_rom()) { s_prov.state = CORE_NVM_H5_ROM; return; }
    if (s_prov.optsr & FLASH_OPTSR_SWAP_BANK) { s_prov.state = CORE_NVM_H5_SWAP; return; }
    for (uint32_t i = 0; i < CORE_NVM_H5_OB_N; i++) {
        uint32_t c = REG32(FLASH_BASE + k_ob_pairs[i][0]), p = REG32(FLASH_BASE + k_ob_pairs[i][1]);
        int bad = (k_ob_pairs[i][2] == PAIR_EQ) ? (p != c)
                : (k_ob_pairs[i][2] == PAIR_EPOCH) ? (p > c) : 0;
        if (bad) {
            s_prov.mismatch = k_ob_pairs[i][1];
            s_prov.state = CORE_NVM_H5_PRG;
            return;
        }
    }

    /* RM0481 §7.4.3 "Option bytes modification sequence". */
    ll_iwdg_refresh();                          /* a watchdog from the last run survives resets */
    ll_flash_wait_bsy();                        /* 1, 2: BSY, WBNE, DBNE clear */
    FLASH_CCR = FLASH_SR_ERR_MASK | FLASH_SR_EOP | FLASH_SR_OPTCHANGEERR;    /* 3 */
    if (FLASH_OPTCR & FLASH_OPTCR_OPTLOCK) {    /* 4 (never twice: that locks it) */
        FLASH_OPTKEYR = FLASH_OPTKEY1;
        FLASH_OPTKEYR = FLASH_OPTKEY2;
    }
    s_prov.edata2_before = cur;
    ob_snapshot(s_prov.ob_before);
    s_prov.writes++;
    FLASH_EDATA2R_PRG = (FLASH_EDATA2R_PRG & ~(FLASH_EDATA_EN | FLASH_EDATA_STRT))
                      | FLASH_EDATA_EN | 1u;    /* 5: the two last sectors */
    SET_BITS(FLASH_OPTCR, FLASH_OPTCR_OPTSTRT); /* 6 */
    uint32_t guard = 200000000UL;               /* 7, 8: ~2 s at 248 MHz; OPTSTRT */
    while (((FLASH_SR & FLASH_SR_BSY) || (FLASH_OPTCR & FLASH_OPTCR_OPTSTRT)) && --guard) { }
    s_prov.nssr = FLASH_SR;
    SET_BITS(FLASH_OPTCR, FLASH_OPTCR_OPTLOCK);
    s_prov.edata2_after = FLASH_EDATA2R_CUR;
    ob_snapshot(s_prov.ob_after);
    if (!guard || (s_prov.nssr & (FLASH_SR_ERR_MASK | FLASH_SR_OPTCHANGEERR))
        || !edata2_on(s_prov.edata2_after)) {
        s_prov.state = CORE_NVM_H5_FAILED;      /* no reset: this boot runs without NVM */
        return;
    }
    s_prov.state = CORE_NVM_H5_CHANGED;
    /* 9: reset, if this record survives it (FLASH_OPTSR2.SRAM13_RST = 1: SRAM1
     * kept on a system reset, §7.11.23), so the next boot can't loop back
     * here. Otherwise carry on: _CUR already has the new value (§7.4.3). */
    if (REG32(FLASH_BASE + 0x070UL) & (1UL << 2)) {
        __asm volatile ("dsb 0xF" ::: "memory");
        SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
        for (;;) { }
    }
}
#endif /* CORE_NVM_EDATA && __arm__ */

/* ============================================================
 * A Core without NVM
 * ============================================================ */

#else

/* CORE_NVM_SIZE is 0: only a zero-length read at offset 0 is in range, and
 * it copies nothing (as the header-only version did). */
int core_nvm_read(uint32_t offset, void *buf, uint32_t len)
{
    (void)buf;
    if (len > (uint32_t)CORE_NVM_SIZE ||
        offset > (uint32_t)CORE_NVM_SIZE - len)
        return CORE_NVM_ERR_RANGE;
    return CORE_NVM_OK;
}

int core_nvm_write(uint32_t offset, const void *data, uint32_t len)
{
    (void)offset; (void)data; (void)len;
    return CORE_NVM_ERR_RANGE;
}

int core_nvm_erase_all(void)
{
    return CORE_NVM_ERR_RANGE;
}

#endif
