/**
 * @file   tile_store_o_128.c
 * @brief  Store.O.128 (AT25QL128A 128 Mbit QSPI NOR flash) — platform-agnostic driver.
 */

#include "tile_store_o_128.h"
#include <stddef.h>

/* ---- Instance → chip-select mapping ----
 * For an SPI tile, tile->id carries the chip-select identifier (not an I2C
 * address). Instance 0 = the tile's default CS. */
static const uint8_t cs_table[] = { 0 };
#define NUM_INSTANCES  (sizeof(cs_table) / sizeof(cs_table[0]))

static uint8_t resolve_cs(uint8_t instance)
{
    return (instance < NUM_INSTANCES) ? cs_table[instance] : 0xFF;
}

/* ---- Low-level SPI helpers (all go through spi_transfer) ---- */

/* True only when the platform implements the raw SPI transaction. */
static uint8_t has_spi(tile_t* tile)
{
    return (tile->hal && tile->hal->spi_transfer) ? 1 : 0;
}

/* Send a bare command byte (WREN, CE, DP, …). */
static void flash_cmd(tile_t* tile, uint8_t cmd)
{
    tile->hal->spi_transfer(tile->hal->handle, tile->id, &cmd, 1, NULL, 0);
}

/* Send a command, then read `len` bytes (RDID, RDSR, …). */
static void flash_read_reg(tile_t* tile, uint8_t cmd, uint8_t* buf, uint16_t len)
{
    tile->hal->spi_transfer(tile->hal->handle, tile->id, &cmd, 1, buf, len);
}

/* Pack a command + 24-bit address into a tx header (MSB-first). */
static uint8_t addr_hdr(uint8_t* hdr, uint8_t cmd, uint32_t addr)
{
    hdr[0] = cmd;
    hdr[1] = (uint8_t)(addr >> 16);
    hdr[2] = (uint8_t)(addr >> 8);
    hdr[3] = (uint8_t)(addr);
    return 4;
}

/* ---- Public API ---- */

uint8_t tile_store_o_128_find(tiles_pal_t* hal, uint8_t instance)
{
    uint8_t cs = resolve_cs(instance);
    if (cs == 0xFF || !hal->spi_transfer) return 0;

    uint8_t cmd = AT25QL128A_CMD_RDID;
    uint8_t id[3] = { 0, 0, 0 };
    if (hal->spi_transfer(hal->handle, cs, &cmd, 1, id, 3) != 0) return 0;
    return (id[0] == AT25QL128A_MFR_ID && id[2] == AT25QL128A_CAPACITY_ID) ? 1 : 0;
}

void tile_store_o_128_init(tiles_pal_t* hal, uint8_t instance, tile_t* tile,
                           const store_o_128_cfg_t* cfg)
{
    (void)cfg;
    tile->hal      = NULL;
    tile->id       = 0;
    tile->state    = TILE_STATE_NONE;
    tile->flags    = 0;
    tile->callback = NULL;
    tile->cb_ctx   = NULL;

    uint8_t cs = resolve_cs(instance);
    if (cs == 0xFF) {
        TILE_ON_ERROR(tile, "init: invalid instance");
        tile->state = TILE_STATE_ERROR;
        return;
    }
    tile->hal = hal;
    tile->id  = cs;

    if (!has_spi(tile)) {
        TILE_ON_ERROR(tile, "init: platform has no spi_transfer");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    /* Make sure the part is awake, then verify identity. */
    flash_cmd(tile, AT25QL128A_CMD_RDP);
    hal->delay_ms(1);

    uint8_t id[3] = { 0, 0, 0 };
    flash_read_reg(tile, AT25QL128A_CMD_RDID, id, 3);
    if (id[0] != AT25QL128A_MFR_ID || id[2] != AT25QL128A_CAPACITY_ID) {
        TILE_ON_ERROR(tile, "init: unexpected JEDEC ID");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    tile->state = TILE_STATE_READY;
}

uint32_t tile_store_o_128_read_jedec_id(tile_t* tile)
{
    if (!has_spi(tile)) return 0;
    uint8_t id[3] = { 0, 0, 0 };
    flash_read_reg(tile, AT25QL128A_CMD_RDID, id, 3);
    return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
}

uint8_t tile_store_o_128_read_status(tile_t* tile)
{
    uint8_t sr = 0;
    if (!has_spi(tile)) return 0;
    flash_read_reg(tile, AT25QL128A_CMD_RDSR1, &sr, 1);
    return sr;
}

uint8_t tile_store_o_128_is_busy(tile_t* tile)
{
    return (tile_store_o_128_read_status(tile) & AT25QL128A_SR1_WIP) ? 1 : 0;
}

uint32_t tile_store_o_128_get_capacity(tile_t* tile)
{
    (void)tile;
    return AT25QL128A_CAPACITY;
}

uint8_t tile_store_o_128_wait_ready(tile_t* tile, uint32_t timeout_ms)
{
    if (!has_spi(tile)) return 0;
    while (timeout_ms > 0) {
        if (!tile_store_o_128_is_busy(tile)) return 1;
        tile->hal->delay_ms(1);
        timeout_ms--;
    }
    return tile_store_o_128_is_busy(tile) ? 0 : 1;
}

void tile_store_o_128_read(tile_t* tile, uint32_t addr, uint8_t* buf, uint16_t len)
{
    if (tile->state != TILE_STATE_READY || !buf || len == 0) return;
    uint8_t hdr[4];
    uint8_t n = addr_hdr(hdr, AT25QL128A_CMD_READ, addr);
    tile->hal->spi_transfer(tile->hal->handle, tile->id, hdr, n, buf, len);
}

void tile_store_o_128_fast_read(tile_t* tile, uint32_t addr, uint8_t* buf, uint16_t len)
{
    if (tile->state != TILE_STATE_READY || !buf || len == 0) return;
    /* FAST_READ: command + 24-bit address + 1 dummy byte, then the data burst. */
    uint8_t hdr[5];
    addr_hdr(hdr, AT25QL128A_CMD_FAST_READ, addr);
    hdr[4] = 0x00;  /* dummy */
    tile->hal->spi_transfer(tile->hal->handle, tile->id, hdr, 5, buf, len);
}

void tile_store_o_128_write_enable(tile_t* tile)
{
    if (!has_spi(tile)) return;
    flash_cmd(tile, AT25QL128A_CMD_WREN);
}

void tile_store_o_128_page_program(tile_t* tile, uint32_t addr,
                                   const uint8_t* buf, uint16_t len)
{
    if (tile->state != TILE_STATE_READY || !buf || len == 0) return;
    if (len > AT25QL128A_PAGE_SIZE) len = AT25QL128A_PAGE_SIZE;

    /* tx = [PP, a2, a1, a0, <payload...>] in one CS assertion. */
    uint8_t tx[4 + AT25QL128A_PAGE_SIZE];
    uint8_t n = addr_hdr(tx, AT25QL128A_CMD_PP, addr);
    for (uint16_t i = 0; i < len; i++) tx[n + i] = buf[i];

    tile_store_o_128_write_enable(tile);
    tile->hal->spi_transfer(tile->hal->handle, tile->id, tx, (uint16_t)(n + len), NULL, 0);
    if (!tile_store_o_128_wait_ready(tile, 10))  /* tPP max 5 ms (Table 26) */
        TILE_ON_ERROR(tile, "page_program: timeout");
}

void tile_store_o_128_write(tile_t* tile, uint32_t addr, const uint8_t* buf, uint32_t len)
{
    if (tile->state != TILE_STATE_READY || !buf || len == 0) return;

    uint32_t off = 0;
    while (off < len) {
        /* Clamp each program to the current 256-byte page boundary. */
        uint32_t page_left = AT25QL128A_PAGE_SIZE - ((addr + off) & (AT25QL128A_PAGE_SIZE - 1));
        uint32_t chunk = len - off;
        if (chunk > page_left) chunk = page_left;
        tile_store_o_128_page_program(tile, addr + off, buf + off, (uint16_t)chunk);
        off += chunk;
    }
}

/* Issue WREN + an address-bearing erase command, then wait. */
static void flash_erase(tile_t* tile, uint8_t cmd, uint32_t addr, uint32_t timeout_ms)
{
    if (tile->state != TILE_STATE_READY) return;
    uint8_t hdr[4];
    uint8_t n = addr_hdr(hdr, cmd, addr);
    tile_store_o_128_write_enable(tile);
    tile->hal->spi_transfer(tile->hal->handle, tile->id, hdr, n, NULL, 0);
    if (!tile_store_o_128_wait_ready(tile, timeout_ms))
        TILE_ON_ERROR(tile, "erase: timeout");
}

void tile_store_o_128_erase_sector(tile_t* tile, uint32_t addr)
{
    flash_erase(tile, AT25QL128A_CMD_SE, addr, 500);  /* tSE max 0.4 s (Table 26) */
}

void tile_store_o_128_erase_block(tile_t* tile, uint32_t addr)
{
    flash_erase(tile, AT25QL128A_CMD_BE, addr, 3000);  /* tBE2 max 2.5 s (Table 26) */
}

void tile_store_o_128_erase_chip(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) return;
    tile_store_o_128_write_enable(tile);
    flash_cmd(tile, AT25QL128A_CMD_CE);
    /* tCE typ 60 s, max 300 s (Table 26); allow a margin over the max. */
    if (!tile_store_o_128_wait_ready(tile, 320000))
        TILE_ON_ERROR(tile, "erase_chip: timeout");
}

void tile_store_o_128_deep_power_down(tile_t* tile)
{
    if (!has_spi(tile)) return;
    flash_cmd(tile, AT25QL128A_CMD_DP);
    tile->state = TILE_STATE_SLEEPING;
}

void tile_store_o_128_release(tile_t* tile)
{
    if (!has_spi(tile)) return;
    flash_cmd(tile, AT25QL128A_CMD_RDP);
    tile->hal->delay_ms(1);
    if (tile->state == TILE_STATE_SLEEPING) tile->state = TILE_STATE_READY;
}
