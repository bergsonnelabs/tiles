/**
 * @file   tile_store_o_128.h
 * @brief  Store.O.128 tile driver — Renesas/Adesto AT25QL128A 128 Mbit QSPI NOR flash.
 *
 * 16 MB serial NOR flash on SPI / Quad-SPI. Standard SPI-NOR command set:
 * JEDEC ID, status, write-enable, page program (256 B pages), 4 KB sector /
 * 64 KB block / chip erase, read, deep power-down.
 *
 * Platform-agnostic: uses the framework's raw SPI transaction primitive
 * (`tiles_pal_t.spi_transfer`) so command + 24-bit address + data fit one CS
 * assertion. A platform that doesn't implement spi_transfer yet leaves it NULL;
 * every call here guards against that (init reports an error; later calls
 * return 0 / do nothing) rather than faulting.
 *
 * Supply is 1.7-2.0 V only (datasheet §9, VCC). VIH max is VCC + 0.4 V, so
 * the SPI lines must be driven at the tile's 1.8 V rail, not 3.3 V.
 *
 * Sidebands: the AT25QL128A ships with QE = 1 (datasheet §7.7, factory
 * default), so WP (pad 6) and HOLD (pad 9) act as IO2/IO3 and their
 * WP/HOLD functions are inactive in single-lane SPI. Neither pin has an
 * internal pull-up (datasheet Table 1); the tile fits a 47 kΩ pull-up on WP
 * only, HOLD is not pulled.
 *
 * Timing (datasheet Table 26, max): tPP 5 ms, tSE 0.4 s, tBE2 2.5 s,
 * tCE 300 s. READ (0x03) is limited to 50 MHz SCK; FAST_READ (0x0B) to 104 MHz.
 *
 * Quick start:
 * @code
 *   #include "core_tiles.h"
 *
 *   tile_t flash;
 *   uint8_t buf[256];
 *   tile_store_o_128_init(core_tiles_pal(&core_spi1), 0, &flash, NULL);
 *   if (tile_is_ready(&flash)) {
 *       tile_store_o_128_read(&flash, 0x000000, buf, sizeof buf);
 *   }
 * @endcode
 *
 * Datasheet: Renesas AT25QL128A.
 *
 * @studio tile label=Store.O.128 icon=memory
 *
 * @studio unsupported severity=niche category="Quad / QPI I/O modes"
 *   The AT25QL128A supports Dual/Quad SPI and QPI (≤4 data lines). This driver
 *   drives standard single-lane SPI only; quad modes need a quad-capable SPI
 *   transfer in the platform layer (driver-deferred, not chip-gated).
 *
 * @studio unsupported severity=advanced category="Block protection / status register write"
 *   No Write Status Register (0x01/0x31) support: BP/TB/SEC/CMP block
 *   protection, SRP lock modes and the QE bit can't be changed. Factory
 *   default is nothing protected, so program/erase work out of the box
 *   (driver-deferred).
 *
 * @studio unsupported severity=niche category="32 KB block erase"
 *   The chip also erases 32 KB blocks (0x52); the driver exposes only 4 KB
 *   sector, 64 KB block and chip erase (driver-deferred).
 *
 * @studio unsupported severity=niche category="Suspend / resume, reset, OTP, SFDP"
 *   Erase/program suspend-resume (0x75/0x7A), software reset (0x66/0x99),
 *   the 4 kbit one-time-programmable security register and the SFDP table
 *   are not exposed (driver-deferred).
 */

#ifndef INC_TILE_STORE_O_128_H_
#define INC_TILE_STORE_O_128_H_

#include "tiles.h"
#include <stdint.h>

TILES_CHECK_VERSION(1, 0);

/* ---- Driver version ---- */
#define TILE_STORE_O_128_VERSION_MAJOR  1
#define TILE_STORE_O_128_VERSION_MINOR  1
#define TILE_STORE_O_128_VERSION_PATCH  0

/* ---- Device geometry ---- */
#define AT25QL128A_CAPACITY     0x1000000u  /**< 16 MB (128 Mbit)            */
#define AT25QL128A_PAGE_SIZE    256u        /**< program page size, bytes    */
#define AT25QL128A_SECTOR_SIZE  4096u       /**< 4 KB erase sector           */
#define AT25QL128A_BLOCK_SIZE   65536u      /**< 64 KB erase block           */

/* ---- JEDEC identity (RDID 0x9F) ---- */
#define AT25QL128A_MFR_ID       0x1F  /**< Adesto / Renesas manufacturer    */
#define AT25QL128A_CAPACITY_ID  0x18  /**< 2^24 bytes density code           */

/* ---- SPI-NOR command opcodes ---- */
#define AT25QL128A_CMD_WREN     0x06  /**< Write Enable                      */
#define AT25QL128A_CMD_WRDI     0x04  /**< Write Disable                     */
#define AT25QL128A_CMD_RDID     0x9F  /**< Read JEDEC ID (3 bytes)           */
#define AT25QL128A_CMD_RDSR1    0x05  /**< Read Status Register 1            */
#define AT25QL128A_CMD_RDSR2    0x35  /**< Read Status Register 2            */
#define AT25QL128A_CMD_WRSR     0x01  /**< Write Status Register             */
#define AT25QL128A_CMD_READ     0x03  /**< Read Data (cmd + 24-bit addr)     */
#define AT25QL128A_CMD_FAST_READ 0x0B /**< Fast Read (+1 dummy byte)         */
#define AT25QL128A_CMD_PP       0x02  /**< Page Program                      */
#define AT25QL128A_CMD_SE       0x20  /**< Sector Erase (4 KB)               */
#define AT25QL128A_CMD_BE       0xD8  /**< Block Erase (64 KB)               */
#define AT25QL128A_CMD_CE       0xC7  /**< Chip Erase                        */
#define AT25QL128A_CMD_DP       0xB9  /**< Deep Power-Down                   */
#define AT25QL128A_CMD_RDP      0xAB  /**< Release from Deep Power-Down      */

/* ---- Status Register 1 bits ---- */
#define AT25QL128A_SR1_WIP      0x01  /**< Write In Progress (busy)          */
#define AT25QL128A_SR1_WEL      0x02  /**< Write Enable Latch                */

/** Optional init config (pass NULL for defaults). */
typedef struct {
    uint8_t reserved;
} store_o_128_cfg_t;

/** Probe instance `instance`: reads the JEDEC ID, returns 1 if it matches. */
uint8_t tile_store_o_128_find(tiles_pal_t* hal, uint8_t instance);

/** Initialize: verify JEDEC identity, wake from deep power-down. */
void tile_store_o_128_init(tiles_pal_t* hal, uint8_t instance, tile_t* tile,
                           const store_o_128_cfg_t* cfg);

/* ---- identity / status ---- */

/**
 * @brief  Read the 24-bit JEDEC ID (manufacturer << 16 | type << 8 | capacity).
 * @studio expose category=tile name=read_jedec_id section=runtime
 * @return Packed JEDEC ID (e.g. 0x1F4218 for the AT25QL128A).
 */
uint32_t tile_store_o_128_read_jedec_id(tile_t* tile);

/**
 * @brief  Read Status Register 1.
 * @studio expose category=tile name=read_status section=runtime
 * @return SR1 byte (AT25QL128A_SR1_* bits).
 */
uint8_t tile_store_o_128_read_status(tile_t* tile);

/**
 * @brief  Whether a program/erase is in progress (WIP bit).
 * @studio expose category=tile name=is_busy returns=bool section=runtime
 * @return 1 if busy, 0 if idle.
 */
uint8_t tile_store_o_128_is_busy(tile_t* tile);

/**
 * @brief  Total flash capacity in bytes.
 * @studio expose category=tile name=get_capacity section=runtime
 * @return 16777216 (16 MB).
 */
uint32_t tile_store_o_128_get_capacity(tile_t* tile);

/* ---- read ---- */

/**
 * @brief  Read `len` bytes starting at `addr` (READ 0x03).
 * @studio expose category=tile name=read section=runtime
 * @param  addr  Byte address (0 .. capacity-1).
 * @param  buf   Output buffer.
 * @param  len   Number of bytes.
 */
void tile_store_o_128_read(tile_t* tile, uint32_t addr, uint8_t* buf, uint16_t len);

/**
 * @brief  Fast-read `len` bytes from `addr` (FAST_READ 0x0B, 1 dummy byte).
 *
 * Same result as tile_store_o_128_read(); use it when the SPI clock is above
 * the 50 MHz READ (0x03) limit (FAST_READ runs to 104 MHz).
 *
 * @studio expose category=tile name=fast_read section=runtime
 * @param  addr  Byte address (0 .. capacity-1).
 * @param  buf   Output buffer.
 * @param  len   Number of bytes.
 */
void tile_store_o_128_fast_read(tile_t* tile, uint32_t addr, uint8_t* buf, uint16_t len);

/* ---- write / erase ---- */

/**
 * @brief  Issue Write-Enable (sets WEL). Required before any program/erase.
 * @studio expose category=tile name=write_enable section=config
 */
void tile_store_o_128_write_enable(tile_t* tile);

/**
 * @brief  Program up to one page (≤256 B) within a single page boundary.
 *
 * Issues Write-Enable, programs, and waits for completion (up to 10 ms;
 * tPP max 5 ms). The caller must keep the write inside one 256-byte page:
 * (addr & 0xFF) + len ≤ 256. Bytes past the page end wrap to the start of
 * the same page (chip behavior), so use tile_store_o_128_write() for
 * unaligned spans. Programming only clears bits: erase first.
 *
 * @studio expose category=tile name=page_program section=runtime
 * @param  addr  Start address.
 * @param  buf   Data to program.
 * @param  len   Byte count (1..256).
 */
void tile_store_o_128_page_program(tile_t* tile, uint32_t addr, const uint8_t* buf, uint16_t len);

/**
 * @brief  Program an arbitrary-length span, splitting across page boundaries.
 *
 * Programming only clears bits (1 → 0); erase the covering sector(s) first.
 *
 * @studio expose category=tile name=write section=runtime
 * @param  addr  Start address (0 .. capacity-1).
 * @param  buf   Data to program.
 * @param  len   Byte count.
 */
void tile_store_o_128_write(tile_t* tile, uint32_t addr, const uint8_t* buf, uint32_t len);

/**
 * @brief  Erase the 4 KB sector containing `addr` (and wait, up to 0.5 s).
 * @studio expose category=tile name=erase_sector section=runtime
 * @param  addr  Any address inside the sector (low 12 bits ignored).
 */
void tile_store_o_128_erase_sector(tile_t* tile, uint32_t addr);

/**
 * @brief  Erase the 64 KB block containing `addr` (and wait, up to 3 s).
 * @studio expose category=tile name=erase_block section=runtime
 * @param  addr  Any address inside the block (low 16 bits ignored).
 */
void tile_store_o_128_erase_block(tile_t* tile, uint32_t addr);

/**
 * @brief  Erase the entire chip (and wait; typ 60 s, max 300 s).
 * @studio expose category=tile name=erase_chip section=runtime
 */
void tile_store_o_128_erase_chip(tile_t* tile);

/**
 * @brief  Block until any in-progress program/erase finishes, or timeout.
 * @studio expose category=tile name=wait_ready returns=bool section=runtime
 * @param  timeout_ms  Max wait.
 * @return 1 if ready, 0 if timed out.
 */
uint8_t tile_store_o_128_wait_ready(tile_t* tile, uint32_t timeout_ms);

/* ---- power ---- */

/**
 * @brief  Enter Deep Power-Down (typ 2 µA; every command except release is ignored).
 * @studio expose category=tile name=deep_power_down section=lifecycle
 */
void tile_store_o_128_deep_power_down(tile_t* tile);

/**
 * @brief  Release from Deep Power-Down.
 * @studio expose category=tile name=release section=lifecycle
 */
void tile_store_o_128_release(tile_t* tile);

#endif /* INC_TILE_STORE_O_128_H_ */
