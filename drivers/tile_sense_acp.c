/**
 * @file  tile_sense_acp.c
 * @brief Sense.ACP tile driver implementation — TMD3725 ALS/Color/Proximity.
 */

#include "tile_sense_acp.h"

/* ---- Instance -> I2C address ----
 *
 * The TMD3725 has a single fixed address, so only instance 0 is valid. The
 * find()/init() signature keeps the instance parameter for framework
 * symmetry with multi-address tiles. */

static uint8_t resolve_id(uint8_t instance)
{
    return (instance == 0) ? TMD3725_I2C_ADDR : 0;
}

/* ---- Per-instance state ----
 *
 * Only the ENABLE byte is cached, so wake() can restore whichever engines
 * (AEN/PEN) were selected at init after sleep() zeroes the register. */

typedef struct {
    uint8_t enable;  /* Cached ENABLE for wake(). */
} acp_state_t;

static acp_state_t acp_state;

static acp_state_t *state_for(tile_t *tile)
{
    (void)tile;
    return &acp_state;
}

/* ---- Portable memzero ---- */

static void memzero(void *p, uint8_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) *b++ = 0;
}

/* ---- Bus helpers ---- */

static void acp_write_reg(tile_t *tile, uint8_t reg, uint8_t val)
{
    tile->hal->i2c_write(tile->hal->handle, tile->id, reg, &val, 1);
}

static uint8_t acp_read_reg(tile_t *tile, uint8_t reg)
{
    uint8_t val = 0;
    tile->hal->i2c_read(tile->hal->handle, tile->id, reg, &val, 1);
    return val;
}

/* Read a little-endian 16-bit channel (low byte at `reg`, high at reg+1). */
static uint16_t acp_read_u16(tile_t *tile, uint8_t reg)
{
    uint8_t buf[2] = {0, 0};
    tile->hal->i2c_read(tile->hal->handle, tile->id, reg, buf, 2);
    return (uint16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
}

/* ---- Internal helpers ---- */

/* Convert a requested LED drive current (mA) to a PLDRIVE field value.
 * i_LED = 6 x (PLDRIVE + 1) mA, so PLDRIVE = (mA / 6) - 1, clamped 0..31. */
static uint8_t drive_ma_to_pldrive(uint8_t ma)
{
    uint8_t steps;
    if (ma < 6) ma = 6;
    steps = (uint8_t)(ma / 6);
    if (steps < 1) steps = 1;
    if (steps > 32) steps = 32;
    return (uint8_t)(steps - 1);
}

/* ---- Lifecycle ---- */

uint8_t tile_sense_acp_find(tiles_pal_t *hal, uint8_t instance)
{
    uint8_t addr = resolve_id(instance);
    if (!addr) return 0;

    if (hal->i2c_is_ready(hal->handle, addr) != 0)
        return 0;

    uint8_t id = 0;
    hal->i2c_read(hal->handle, addr, TMD3725_REG_ID, &id, 1);
    return (id == TMD3725_ID_VALUE) ? 1 : 0;
}

void tile_sense_acp_init(tiles_pal_t *hal, uint8_t instance,
                         tile_t *tile, const sense_acp_cfg_t *cfg)
{
    memzero(tile, sizeof(tile_t));
    tile->hal = hal;
    tile->id  = resolve_id(instance);

    if (!tile->id) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_acp: invalid instance");
        return;
    }

    if (hal->i2c_is_ready(hal->handle, tile->id) != 0) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_acp: device not responding");
        return;
    }

    uint8_t id = acp_read_reg(tile, TMD3725_REG_ID);
    if (id != TMD3725_ID_VALUE) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_acp: ID mismatch");
        return;
    }

    acp_state_t *s = state_for(tile);
    memzero(s, sizeof(acp_state_t));

    /* Resolve config with defaults (matches the header's documented set). */
    uint8_t als_gain  = cfg ? cfg->als_gain  : SENSE_ACP_ALS_GAIN_16X;
    uint8_t atime     = cfg ? cfg->atime     : 0x2F;   /* ~135 ms */
    uint8_t prox_gain = cfg ? cfg->prox_gain : SENSE_ACP_PROX_GAIN_4X;
    uint8_t drive_ma  = cfg ? cfg->prox_drive_ma : 12;
    uint8_t en_als    = cfg ? cfg->enable_als  : 1;
    uint8_t en_prox   = cfg ? cfg->enable_prox : 1;

    /* Preset the operating-mode registers before enabling any engine
     * (datasheet: "Before activating AEN or PEN, preset each applicable
     * operating mode register"). Keep the sensor off while configuring. */
    acp_write_reg(tile, TMD3725_REG_ENABLE, 0x00);
    acp_write_reg(tile, TMD3725_REG_ATIME, atime);
    acp_write_reg(tile, TMD3725_REG_CFG1, (uint8_t)(als_gain & 0x03));
    acp_write_reg(tile, TMD3725_REG_PCFG1,
                  (uint8_t)(((prox_gain & 0x03) << 6) |
                            drive_ma_to_pldrive(drive_ma)));

    /* Build ENABLE. PON gates the oscillator/ADCs; AEN/PEN gate the ALS
     * and proximity engines. PON and AEN are asserted in the same write so
     * the chip runs auto-zero before the first ALS reading. */
    uint8_t enable = TMD3725_EN_PON;
    if (en_als)  enable |= TMD3725_EN_AEN;
    if (en_prox) enable |= TMD3725_EN_PEN;
    s->enable = enable;

    acp_write_reg(tile, TMD3725_REG_ENABLE, enable);
    hal->delay_ms(5);

    tile->state = TILE_STATE_READY;
}

void tile_sense_acp_sleep(tile_t *tile)
{
    acp_write_reg(tile, TMD3725_REG_ENABLE, 0x00);
    tile->state = TILE_STATE_SLEEPING;
}

void tile_sense_acp_wake(tile_t *tile)
{
    acp_state_t *s = state_for(tile);
    acp_write_reg(tile, TMD3725_REG_ENABLE, s->enable);
    tile->hal->delay_ms(5);
    tile->state = TILE_STATE_READY;
}

/* ---- Configuration ---- */

void tile_sense_acp_set_als_gain(tile_t *tile, sense_acp_als_gain_t gain)
{
    /* CFG1 holds AGAIN[1:0] and IR_TO_GREEN[3]; preserve the rest. */
    uint8_t cfg1 = acp_read_reg(tile, TMD3725_REG_CFG1);
    cfg1 = (uint8_t)((cfg1 & (uint8_t)~0x03) | ((uint8_t)gain & 0x03));
    acp_write_reg(tile, TMD3725_REG_CFG1, cfg1);
}

void tile_sense_acp_set_integration_time(tile_t *tile, uint8_t atime)
{
    acp_write_reg(tile, TMD3725_REG_ATIME, atime);
}

void tile_sense_acp_set_prox_gain(tile_t *tile, sense_acp_prox_gain_t gain)
{
    /* PCFG1 holds PGAIN[7:6] and PLDRIVE[4:0]; preserve PLDRIVE. */
    uint8_t pcfg1 = acp_read_reg(tile, TMD3725_REG_PCFG1);
    pcfg1 = (uint8_t)((pcfg1 & (uint8_t)~0xC0) | (((uint8_t)gain & 0x03) << 6));
    acp_write_reg(tile, TMD3725_REG_PCFG1, pcfg1);
}

void tile_sense_acp_set_prox_drive_ma(tile_t *tile, uint8_t ma)
{
    /* PCFG1 holds PGAIN[7:6] and PLDRIVE[4:0]; preserve PGAIN. */
    uint8_t pcfg1 = acp_read_reg(tile, TMD3725_REG_PCFG1);
    pcfg1 = (uint8_t)((pcfg1 & (uint8_t)~0x1F) | drive_ma_to_pldrive(ma));
    acp_write_reg(tile, TMD3725_REG_PCFG1, pcfg1);
}

/* ---- Ambient light / colour data ---- */

uint16_t tile_sense_acp_get_clear(tile_t *tile)
{
    return acp_read_u16(tile, TMD3725_REG_CDATAL);
}

uint16_t tile_sense_acp_get_red(tile_t *tile)
{
    return acp_read_u16(tile, TMD3725_REG_RDATAL);
}

uint16_t tile_sense_acp_get_green(tile_t *tile)
{
    return acp_read_u16(tile, TMD3725_REG_GDATAL);
}

uint16_t tile_sense_acp_get_blue(tile_t *tile)
{
    return acp_read_u16(tile, TMD3725_REG_BDATAL);
}

/* ---- Proximity data ---- */

uint8_t tile_sense_acp_get_proximity(tile_t *tile)
{
    return acp_read_reg(tile, TMD3725_REG_PDATA);
}

/* ---- Status ---- */

uint8_t tile_sense_acp_get_status(tile_t *tile)
{
    return acp_read_reg(tile, TMD3725_REG_STATUS);
}
