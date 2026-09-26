/**
 * core_nvm.h — Non-volatile memory (EEPROM / flash emulation)
 *
 * One offset-based byte store with the same API on every Core, so a project
 * that keeps its settings or calibration here ports between Cores unchanged:
 *   - Core.ST.L0 (STM32L011): true data EEPROM, 512 B at 0x08080000.
 *   - Core.ST.L4 / L4.2 (STM32L422): 1024 B, emulated in the last two 2 KB
 *     flash pages (0x0801F000-0x0801FFFF).
 *   - Core.ST.W5 (STM32WBA55): 1024 B, emulated in flash pages 125-126
 *     (0x080FA000-0x080FDFFF). Page 127 above it is the BLE bond store.
 *   - Core.ST.H5 (STM32H523): 1024 B, emulated in the flash's high-cycle
 *     data area (EDATA, 100k cycles, RM0481 §7.3.10): bank 2 sectors 30-31,
 *     the top 16 KB of flash (0x0807C000), read as two 6 KB data sectors at
 *     0x09015000-0x09017FFF. The area is an option-byte setting: the first
 *     time a Core.ST.H5 boots an SDK image, core_init() switches it on (one
 *     option-byte change, FLASH_EDATA2R, then one reset); every later boot
 *     just reads it. See _core_nvm_h5_boot() below.
 *
 * Usage:
 *   core_nvm_write(0, &data, sizeof(data));  // write at offset 0
 *   core_nvm_read(0, &data, sizeof(data));   // read back
 *
 * The data survives resets, power cycles and reflashing with `make
 * flash-serial`, `make flash-dfu`, Studio, probe-rs or CubeProgrammer: every
 * linker script ends the application below the NVM pages, and those tools
 * erase only the pages an image covers. A full-chip (mass) erase clears it,
 * as does any tool told to erase the whole chip (on the H5 a mass or bank
 * erase also erases the data sectors, RM0481 §7.3.6; the EDATA option byte
 * stays set).
 *
 * Never-written bytes read as CORE_NVM_ERASED: 0xFF on the flash-emulated
 * Cores (like erased flash), 0x00 on the L0 (its EEPROM's erased state).
 *
 * Flash emulation (L4, W5, H5): each core_nvm_write() appends one record
 * (offset, length, data, CRC-32) to a log in the active page. When the page is full,
 * the current contents are copied to the other page, which then becomes
 * active ("compaction": one page erase). A write is atomic: after a reset or
 * power loss at any instant, the bytes it covered read either all old or all
 * new, never a mix. Writing the bytes that are already stored costs nothing.
 *
 * Timing (flash emulation, measured on a Core.ST.L4 at 16 MHz):
 *   - A write programs its record and reads it back: a few hundred us for a
 *     few bytes, ~10 ms for 600 bytes (L4: 82 us per 8 B; W5: 118 us per 16 B).
 *   - A compaction erases one page, then copies the contents over: 35-45 ms
 *     on the L4. The L4 has a single flash bank, so for the ~22 ms erase (up
 *     to ~25 ms) the CPU stalls and every interrupt waits until it ends; while
 *     copying, interrupts run between 8-byte units. The W5 erase is ~1.5 ms
 *     (2.4 ms max). The watchdog is fed just before the erase.
 *   - W5 with BLE running: erases and writes go through ST's flash manager,
 *     which runs each in a gap between radio events, so a connection or
 *     advertising survives. The call blocks until that has happened, running
 *     the BLE tasks meanwhile; call it from the main loop or a BLE callback,
 *     never from an interrupt handler.
 *
 * H5 (tests/hw-nvm-flash, 2026-09-26): the data sectors are programmed 16
 * bits at a time (DS14540 Table 49: 31 us typ, 100 us max each), so a 4-byte
 * write takes ~0.3 ms and a 600-byte one ~12.6 ms; a compaction erases one
 * sector (2 ms typ, 10 ms max) and copies the contents, ~34 ms at 64 MHz.
 * Code keeps running meanwhile (it is in the other bank, RM0481 §7.3.7). Each
 * read of the area is a 16-bit flash access plus an ECC check, so reads and
 * writes slow down as the log fills: with ~300 records over the same bytes, a
 * small write took 3 ms at 64 MHz. The ICACHE is off during every core_nvm
 * call (the area must not be cached), which slows other code while it runs.
 *
 * Wear: each page takes 10,000 erases (L4 DS12470 §6.3.10, W5 DS14127
 * table 70), or 100,000 on the H5's data sectors (RM0481 §7.3.10), and the
 * two pages take turns. A write costs ceil((8 + len) /
 * unit) * unit bytes of log (unit = 8 B on the L4, 16 B on the W5); one erase
 * buys a page's worth of log minus the compacted contents (136-144 B per 128 B
 * of the store in use). With a small store: one-byte writes cost ~8.7 erases
 * per KB written on the L4 and ~2 on the W5, 64-byte writes ~0.6 and ~0.16.
 * So an L4 with a few settings takes ~2.4 million one-byte writes before its
 * pages reach 10,000 erases each.
 *
 * Not re-entrant: a call made while another is in progress (from an
 * interrupt) returns CORE_NVM_ERR_BUSY.
 *
 * @studio category nvm label=Core.NVM icon=▤
 *
 * @studio coverage
 *   id:    nvm
 *   name:  NVM — non-volatile memory
 *   blurb: Byte-level read / write with one API on every Core. Core.ST.L0 hits
 *          true EEPROM (512 B, no erase needed); Core.ST.L4, Core.ST.W5 and
 *          Core.ST.H5 emulate 1 KB in two flash pages (power-loss safe,
 *          wear-levelled, survives reflashing; on the W5 it shares the radio
 *          through ST's flash manager; on the H5 it lives in the 100k-cycle
 *          data area, switched on at first boot). Tier 2 exposes
 *          `nvm.size` and the byte-level read / write.
 */

#ifndef CORE_NVM_H
#define CORE_NVM_H

#include <stdint.h>
#include <string.h>
#include "ll_common.h"

/* ============================================================
 * Return codes (core_nvm_read / _write / _erase_all)
 * ============================================================ */

#define CORE_NVM_OK            0    /**< Success. */
#define CORE_NVM_ERR_RANGE    (-1)  /**< Offset / length outside the region, or this Core has no NVM. */
#define CORE_NVM_ERR_FLASH    (-2)  /**< Erase or program failed (FLASH_SR error, or the read-back did not match). */
#define CORE_NVM_ERR_BUSY     (-3)  /**< Called while another NVM call was in progress (e.g. from an interrupt). */
#define CORE_NVM_ERR_LAYOUT   (-4)  /**< The linker script does not reserve the NVM pages (a custom .ld). */

/* ============================================================
 * Platform-specific NVM configuration
 * ============================================================ */

#if defined(STM32L011xx)
  /* True EEPROM: 512 bytes, byte-writable, no erase needed */
  #define CORE_NVM_BASE        0x08080000UL
  #define CORE_NVM_SIZE        512
  #define CORE_NVM_EEPROM      1
  #define CORE_NVM_FLASH_EMU   0
  #define CORE_NVM_ERASED      0x00u   /**< RM0377 §3.3.4: an erased EEPROM byte reads 0. */

#elif defined(STM32L422xx)
  /* Flash emulation in the last two 2 KB pages (62, 63) of the 128 KB bank.
   * The linker scripts end the application at 0x0801F000 to match. */
  #define CORE_NVM_SIZE        1024
  #define CORE_NVM_EEPROM      0
  #define CORE_NVM_FLASH_EMU   1
  #define CORE_NVM_ERASED      0xFFu
  #define CORE_NVM_FLASH_ADDR  0x0801F000UL
  #define CORE_NVM_PAGE_SIZE   2048u
  #define CORE_NVM_FIRST_PAGE  62u
  #define CORE_NVM_PROG_UNIT   8u      /* RM0394 §3.3.7: 64-bit double-words */

#elif defined(STM32WBA55xx)
  /* Flash emulation in pages 125-126 (8 KB each). Page 127, the last one, is
   * the BLE bond store (sdk/ble/nvm_stub.c), which was there first. The
   * linker scripts end the application at 0x080FA000 to leave all three. */
  #define CORE_NVM_SIZE        1024
  #define CORE_NVM_EEPROM      0
  #define CORE_NVM_FLASH_EMU   1
  #define CORE_NVM_ERASED      0xFFu
  #define CORE_NVM_FLASH_ADDR  0x080FA000UL
  #define CORE_NVM_PAGE_SIZE   8192u
  #define CORE_NVM_FIRST_PAGE  125u
  #define CORE_NVM_PROG_UNIT   16u     /* RM0493 §7.3.7: 128-bit quad-words */

#elif defined(STM32H523xx)
  /* Flash emulation in the high-cycle data area (EDATA, RM0481 §7.3.10):
   * bank 2 sectors 30 and 31, 6 KB each as data, at 0x09015000 (bank 2's
   * sector 24 is data address 0x0900C000, RM0481 Figure 27). The linker
   * script ends every image at 0x0807C000, the same sectors' code address.
   * The array is written 16 bits at a time; records still align to 8 B. */
  #define CORE_NVM_SIZE        1024
  #define CORE_NVM_EEPROM      0
  #define CORE_NVM_FLASH_EMU   1
  #define CORE_NVM_EDATA       1
  #define CORE_NVM_ERASED      0xFFu
  #define CORE_NVM_FLASH_ADDR  0x09015000UL
  #define CORE_NVM_PAGE_SIZE   6144u
  #define CORE_NVM_FIRST_PAGE  62u     /* flat 8 KB sector: bank 2, sector 30 */
  #define CORE_NVM_PROG_UNIT   8u      /* record alignment (the hardware writes 16 bits) */
  #define CORE_NVM_CODE_ADDR   0x0807C000UL   /* the two sectors' code address */
#endif

#ifndef CORE_NVM_EDATA
  #define CORE_NVM_EDATA       0
#endif

#if CORE_NVM_FLASH_EMU
  #define CORE_NVM_FLASH_SIZE  (2u * CORE_NVM_PAGE_SIZE)
#endif

/* ============================================================
 * API (sdk/core/core_nvm.c)
 * ============================================================ */

/**
 * Read bytes from NVM. Bytes never written read as CORE_NVM_ERASED (0xFF on
 * Core.ST.L4 / W5 / H5, 0 on Core.ST.L0).
 *
 * @param offset Byte offset within the NVM region (0 to CORE_NVM_SIZE-1).
 * @param buf    Destination buffer.
 * @param len    Number of bytes to read.
 * @return 0 on success, CORE_NVM_ERR_RANGE (-1) if the range falls outside
 *         the NVM region, CORE_NVM_ERR_FLASH if the store can't be read.
 */
int core_nvm_read(uint32_t offset, void *buf, uint32_t len);

/**
 * Write bytes to NVM. On Core.ST.L4 / W5 / H5 (1 KB, flash-emulated) a write
 * is atomic: after a reset or power loss the range reads all old or all new,
 * and the data survives reflashing. Most writes take well under a millisecond;
 * about one small write in a hundred (L4; one in 500 on the W5, one in 350 on
 * the H5) compacts the store, which erases a flash page and stalls the
 * Core.ST.L4 for ~22 ms. Core.ST.L0 writes its EEPROM byte by byte (~3.2 ms
 * each).
 *
 * @param offset Byte offset within the NVM region (0 to CORE_NVM_SIZE-1).
 * @param data   Source buffer.
 * @param len    Number of bytes to write.
 * @return 0 on success, or a negative CORE_NVM_ERR_* code: RANGE (-1) if the
 *         range falls outside the region or the Core has no NVM, FLASH (-2)
 *         if the hardware reported an error, BUSY (-3), LAYOUT (-4).
 */
int core_nvm_write(uint32_t offset, const void *data, uint32_t len);

/**
 * Erase the whole NVM region: every byte reads CORE_NVM_ERASED afterwards.
 * Core.ST.L4 / W5 / H5: one page erase, or nothing if the region is already
 * empty. Core.ST.L0: clears the EEPROM word by word (up to ~0.4 s; words
 * already clear are skipped).
 *
 * @return 0 on success, or a negative CORE_NVM_ERR_* code.
 */
int core_nvm_erase_all(void);

/**
 * Returns the total NVM size in bytes for this Core.
 *
 * @studio expose category=nvm name=size returns=int
 * @studio twin full
 */
static inline uint32_t core_nvm_size(void)
{
    return CORE_NVM_SIZE;
}

/* ---- Tier 2 — byte-level read / write -------------------------------- */

/* These wrap the buffer-based core_nvm_read / write at byte
 * granularity so DSL programs can persist a flag, a counter, or
 * a small struct field without needing the array-IN / array-OUT
 * host-call ABI. Bulk transfers stay Tier 1 with the buffer forms
 * above; once the array ABI is wired the Tier 2 surface gets
 * `read_buf` / `write_buf` on top of these. */

/**
 * Read a single byte from NVM at `offset`. Returns the byte (0..255)
 * on success or -1 on any error (offset out of range, NVM disabled).
 * A byte never written reads 255 (0xFF) on Core.ST.L4 / W5 / H5 and 0 on
 * Core.ST.L0. The signed return lets DSL programs branch on `< 0`
 * without an out-pointer.
 *
 * @studio expose category=nvm name=read_byte returns=int
 * @studio twin full
 * @param offset [0..1023] Byte offset into the NVM region (Core.ST.L0: 0..511).
 */
static inline int core_nvm_read_byte(uint32_t offset)
{
    uint8_t b = 0;
    if (core_nvm_read(offset, &b, 1) != 0) return -1;
    return (int)b;
}

/**
 * Write a single byte to NVM at `offset`. Returns 1 on success or -1
 * on any error (offset out of range, flash error, no NVM on this Core).
 * Each call is one flash record on Core.ST.L4 / W5 / H5: fine for a setting
 * or a boot counter, wasteful in a tight loop (see core_nvm.h on wear).
 *
 * @studio expose category=nvm name=write_byte returns=int
 * @studio twin full
 * @param offset [0..1023] Byte offset into the NVM region (Core.ST.L0: 0..511).
 * @param value  [0..255] Byte to store.
 */
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
 *     secure-only registers are the exceptions, see core_nvm.c) and refuses
 *     otherwise; it records every _CUR before and after;
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
    uint32_t ob_after[CORE_NVM_H5_OB_N];    /* ... and after the change (core_nvm.c) */
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

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=M title="No bulk read / write Tier 2 yet"
//   Tier 2 covers the byte-level pair (read_byte, write_byte) — enough
//   for boot counters, single flags, small struct fields. Multi-byte
//   read_buf / write_buf still need the array-IN / array-OUT host-call
//   ABI prototyped on the tile-driver side. Tracked with the DSL
//   Capability Coverage close.
//
// @studio unsupported tier=2 value=L title="Twin NVM state wipes on Reset"
//   read_byte / write_byte round-trip within a single run, but the
//   per-slot store is cleared on every project reload — DSL programs
//   that rely on NVM state surviving a soft reset (boot counters,
//   crash flags) can't be exercised end-to-end in the IDE. Same issue
//   the Backup register gap calls out; closing both needs persistent
//   per-slot state across worker resets.
//
// @studio unsupported tier=1 value=L title="No wear-tracking API"
//   core_nvm_erase_all clears the region, but nothing reports the page
//   erase count or the remaining write budget. Long-running data loggers
//   budget their write rate from the figures in core_nvm.h.

#endif /* CORE_NVM_H */
