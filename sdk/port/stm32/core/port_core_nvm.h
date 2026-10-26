/**
 * port_core_nvm.h — STM32 port: the bodies of core_nvm.h
 *
 * Included at the end of core_nvm.h, after its prototypes; not for direct
 * use. The size and the byte-level pair over core_nvm_read / _write (whose
 * store is core_nvm.c, its hardware port_core_nvm_hw.h), and on Core.ST.H5
 * the declarations of the first-boot switch that turns the flash data area
 * on (coregen's core_init() calls it; tests read its record).
 */

#ifndef PORT_CORE_NVM_H
#define PORT_CORE_NVM_H

#include <stdint.h>

static inline uint32_t core_nvm_size(void)
{
    return CORE_NVM_SIZE;
}

static inline int core_nvm_read_byte(uint32_t offset)
{
    uint8_t b = 0;
    if (core_nvm_read(offset, &b, 1) != 0) return -1;
    return (int)b;
}

static inline int core_nvm_write_byte(uint32_t offset, uint8_t value)
{
    if (core_nvm_write(offset, &value, 1) != 0) return -1;
    return 1;
}

#if defined(STM32H523xx)
/* ---- Core.ST.H5: switching the data area on (not API) ----
 *
 * _core_nvm_h5_boot() runs from core_init() on every boot, after the clocks
 * and before the watchdog. If FLASH_EDATA2R_CUR already has the area on
 * (EDATA2_EN, and EDATA2_STRT >= 1: at least the two last sectors) it only
 * notes that. Otherwise it makes one option-byte change, RM0481 §7.4.3:
 *   - only FLASH_EDATA2R_PRG is written, read-modify-write of EDATA2_EN and
 *     EDATA2_STRT (= 1); nothing else. OPTSTRT programs every _PRG register,
 *     so it first checks that each other readable _PRG equals its _CUR (they
 *     are loaded equal at option-byte load; the epoch counters and the
 *     secure-only registers are the exceptions, see port_core_nvm_hw.h)
 *     and refuses otherwise; it records every _CUR before and after;
 *   - not when SWAP_BANK is set, nor when the ROM bootloader started the
 *     image (BOOT0 high: the reset below would land back in the ROM);
 *   - then it resets the chip (§7.4.3 step 9, "always recommended").
 * A reset or power loss during the change keeps the old option value
 * (§7.4.3 note), so the next boot simply tries again. At most one change is
 * made per power-on, so nothing can loop. The record below lives in SRAM that
 * startup leaves alone (.noinit), so the boot after the change can report it;
 * a power cycle clears it. On a Core whose option bytes erase SRAM1 on reset
 * (FLASH_OPTSR2.SRAM13_RST = 0) the record would not survive, so there the
 * change is made without the reset. */
#define CORE_NVM_H5_OB_N  15u
typedef struct {
    uint32_t magic;          /* valid record */
    int32_t  state;          /* CORE_NVM_H5_*: what this boot found or did */
    uint32_t writes;         /* option-byte changes made since the record began */
    uint32_t boots;          /* boots since the record began */
    uint32_t edata2_before;  /* FLASH_EDATA2R_CUR before the change */
    uint32_t edata2_after;   /* FLASH_EDATA2R_CUR after it */
    uint32_t edata1;         /* FLASH_EDATA1R_CUR (never written) */
    uint32_t optsr;          /* FLASH_OPTSR_CUR (never written) */
    uint32_t nssr;           /* FLASH_NSSR at the end of the change */
    uint32_t opsr;           /* FLASH_OPSR at boot: an operation a reset cut short */
    uint32_t mismatch;       /* _PRG offset that differed from its _CUR (refused) */
    uint32_t ob_before[CORE_NVM_H5_OB_N];   /* every option _CUR register before */
    uint32_t ob_after[CORE_NVM_H5_OB_N];    /* ... and after the change (port_core_nvm_hw.h) */
} core_nvm_h5_prov_t;

#define CORE_NVM_H5_ENABLED   1    /* on already: nothing written this boot */
#define CORE_NVM_H5_CHANGED   2    /* switched on this boot (the reset follows) */
#define CORE_NVM_H5_ROM      (-1)  /* started by the ROM bootloader: not tried */
#define CORE_NVM_H5_SWAP     (-2)  /* SWAP_BANK set: not tried */
#define CORE_NVM_H5_PRG      (-3)  /* another _PRG register differed from _CUR: refused */
#define CORE_NVM_H5_FAILED   (-4)  /* the change reported an error or did not take */
#define CORE_NVM_H5_RETRY    (-5)  /* changed once since power-on, still off: not again */

void _core_nvm_h5_boot(void);
/* The record, or NULL after a power cycle before this boot's call. */
const core_nvm_h5_prov_t *_core_nvm_h5_prov(void);
/* Data-area words core_nvm has read with an uncorrectable ECC error since
 * boot (a torn write leaves some; each one ends the log there). */
uint32_t _core_nvm_h5_ecc_errors(void);
#endif

#endif /* PORT_CORE_NVM_H */
