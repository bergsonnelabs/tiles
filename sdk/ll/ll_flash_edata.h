/**
 * ll_flash_edata.h — H5 flash high-cycle data area (EDATA) and the option
 * registers that switch it on. Kept out of ll_flash.h, whose text the
 * serial-update flasher blobs hash (tools/gen_serial_update_blob.py).
 */

#ifndef LL_FLASH_EDATA_H
#define LL_FLASH_EDATA_H

#include "ll_flash.h"

#if defined(STM32H523xx)

/* ============================================================
 * H5: flash high-cycle data (EDATA) and the option-byte registers it needs
 *
 * RM0481 §7.3.10: the last sectors of a bank can be switched, by option byte
 * (FLASH_EDATAxR, §7.11.37-38 / §7.11.50-51), to 6 KB sectors of 16-bit
 * words with 6 ECC bits each, 100k cycles, read at 0x0900_0000-0x0901_7FFF
 * (bank 2's sectors 24..31 from 0x0900_C000, 6 KB each: RM0481 Figure 27).
 *   - Access by 16 or 32 bits only: an 8-bit read is a bus error (§7.3.4),
 *     an 8-bit write is ignored (§7.3.5). Each 16-bit write is one program.
 *   - A word never written since its erase ("virgin") reads 0xFFFF with a
 *     double ECC error; FLASH_ECCDR then holds 0xFFFF, which is how a virgin
 *     word is told from a real error (§7.3.4 step 4, §7.3.8, §7.11.43).
 *   - A double ECC error raises an NMI unless SBS_ECCNMIR.ECCNMI_MASK_EN is
 *     set, which masks it for data-area reads (§7.9.10, §14.5.18).
 *   - Erase: the normal sector erase of the matching 8 KB sector (§7.3.10).
 *   - The ICACHE must not cache the area (§7.3.2: "MPU must be used to
 *     disable local cacheability"); these helpers keep it off meanwhile.
 * ============================================================ */

#define FLASH_OPSR          REG32(FLASH_BASE + 0x018UL)   /* §7.11.7 */
#define FLASH_OPTKEYR       REG32(FLASH_BASE + 0x00CUL)   /* §7.11.4 */
#define FLASH_OPTCR         REG32(FLASH_BASE + 0x01CUL)   /* §7.11.8 */
#define FLASH_OPTCR_OPTLOCK (1UL << 0)
#define FLASH_OPTCR_OPTSTRT (1UL << 1)
#define FLASH_OPTKEY1       0x08192A3BUL
#define FLASH_OPTKEY2       0x4C5D6E7FUL
#define FLASH_SR_OPTCHANGEERR (1UL << 23)                 /* NSSR, cleared by NSCCR bit 23 */
#define FLASH_EDATA1R_CUR   REG32(FLASH_BASE + 0x0F0UL)   /* §7.11.37 */
#define FLASH_EDATA2R_CUR   REG32(FLASH_BASE + 0x1F0UL)   /* §7.11.50 */
#define FLASH_EDATA2R_PRG   REG32(FLASH_BASE + 0x1F4UL)   /* §7.11.51 */
#define FLASH_EDATA_EN      (1UL << 15)
#define FLASH_EDATA_STRT    0x7UL       /* STRT = n: the n + 1 last sectors of the bank */
#define FLASH_ECCCORR       REG32(FLASH_BASE + 0x100UL)   /* §7.11.41 */
#define FLASH_ECCDETR       REG32(FLASH_BASE + 0x104UL)   /* §7.11.42 */
#define FLASH_ECCDR         REG32(FLASH_BASE + 0x108UL)   /* §7.11.43 */
#define FLASH_ECCCORR_ECCC   (1UL << 30)
#define FLASH_ECCCORR_ECCCIE (1UL << 25)
#define FLASH_ECCDETR_ECCD   (1UL << 31)
#define LL_SBS_ECCNMIR      REG32(0x4400054CUL)          /* SBS 0x4400_0400 + 0x14C, §14.5.18 */
#define LL_SBS_ECCNMI_MASK  (1UL << 0)

typedef struct { uint8_t icache, nmi_masked; } ll_flash_edata_ctx_t;

/** Before EDATA accesses: ICACHE off (§7.3.2), the double-ECC NMI masked for
 *  data-area reads (SBS clock on, RCC_APB3ENR.SBSEN bit 1, §11.8.32), stale
 *  ECC flags cleared (§7.9.10: "mandatory to clear ... before a new read"). */
static inline void ll_flash_edata_begin(ll_flash_edata_ctx_t *c)
{
    c->icache = (uint8_t)ll_icache_disable();
    SET_BITS(REG32(RCC_BASE + 0xA8UL), 1UL << 1);            /* SBSEN */
    (void)REG32(RCC_BASE + 0xA8UL);
    c->nmi_masked = (uint8_t)(LL_SBS_ECCNMIR & LL_SBS_ECCNMI_MASK);
    LL_SBS_ECCNMIR = LL_SBS_ECCNMI_MASK;
    FLASH_ECCDETR = FLASH_ECCDETR_ECCD;
    FLASH_ECCCORR = (FLASH_ECCCORR & FLASH_ECCCORR_ECCCIE) | FLASH_ECCCORR_ECCC;
}

/** After EDATA accesses: flags cleared, NMI mask and ICACHE as they were. */
static inline void ll_flash_edata_end(const ll_flash_edata_ctx_t *c)
{
    FLASH_ECCDETR = FLASH_ECCDETR_ECCD;
    FLASH_ECCCORR = (FLASH_ECCCORR & FLASH_ECCCORR_ECCCIE) | FLASH_ECCCORR_ECCC;
    if (!c->nmi_masked) LL_SBS_ECCNMIR = 0;
    if (c->icache) ll_icache_enable();
}

/**
 * Read one 16-bit EDATA word (inside ll_flash_edata_begin/end).
 * Returns 0: data (clean, or a single error corrected); 1: virgin (never
 * written since the erase; *v = 0xFFFF); -1: uncorrectable (double ECC error
 * on written data; *v = 0xFFFF).
 */
static inline int ll_flash_edata_read_half(uint32_t addr, uint16_t *v)
{
    uint32_t pm;
    __asm volatile ("mrs %0, primask\n\tcpsid i" : "=r"(pm) :: "memory");
    uint16_t d = *(volatile const uint16_t *)addr;
    __asm volatile ("dsb 0xF" ::: "memory");
    uint32_t det = FLASH_ECCDETR;
    int r = 0;
    if (det & FLASH_ECCDETR_ECCD) {
        r = ((FLASH_ECCDR & 0xFFFFUL) == 0xFFFFUL) ? 1 : -1;
        FLASH_ECCDETR = FLASH_ECCDETR_ECCD;                  /* rc_w1 */
        d = 0xFFFFu;
    } else if (FLASH_ECCCORR & FLASH_ECCCORR_ECCC) {
        FLASH_ECCCORR = (FLASH_ECCCORR & FLASH_ECCCORR_ECCCIE) | FLASH_ECCCORR_ECCC;
    }
    __asm volatile ("msr primask, %0" :: "r"(pm) : "memory");
    *v = d;
    return r;
}

/**
 * Program one 16-bit EDATA word (§7.3.5 single-write sequence; inside
 * ll_flash_edata_begin/end, NSCR unlocked). The word must be virgin: writing
 * over a written word "may lead to invalid data and inconsistent ECC code".
 * Returns 0, or -1 on a FLASH_NSSR error.
 */
static inline int ll_flash_edata_program_half(uint32_t addr, uint16_t v)
{
    ll_flash_wait_bsy();                        /* BSY, WBNE, DBNE clear */
    ll_flash_clear_errors();
    SET_BITS(FLASH_CR, FLASH_CR_PG);
    *(volatile uint16_t *)addr = v;
    __asm volatile ("dsb 0xF" ::: "memory");
    ll_flash_wait_bsy();
    CLR_BITS(FLASH_CR, FLASH_CR_PG);
    return (FLASH_SR & FLASH_SR_ERR_MASK) ? -1 : 0;
}
#endif /* STM32H523xx */

#endif /* LL_FLASH_EDATA_H */
