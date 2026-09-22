/**
 * @file   tile_sense_adc_6.c
 * @brief  Sense.ADC.6 -- platform-agnostic driver implementation.
 *
 * All bus access goes through tile->hal function pointers.
 *
 * The tile presents a small register file. Readings are published as a
 * set, and the tile latches that set at the start of a read, so a single
 * burst from REG_STATUS always returns six channels from one instant.
 * That is why update() reads all fourteen bytes in one go and read_mv()
 * then costs nothing.
 */

#include "tile_sense_adc_6.h"
#include <stddef.h>

/* ================================================================
 * Register map
 * ================================================================ */

#define REG_WHO_AM_I    0x00
#define REG_VERSION     0x01
#define REG_STATUS      0x02   /* status, seq, then six uint16 LE */
#define REG_SUPPLY      0x10
#define REG_ADDR_CUR    0x12
#define REG_ADDR_SET    0x13
#define REG_COMMIT      0x14
#define REG_RATE        0x15   /* rate, oversamp, window, apply */
#define REG_APPLY       0x18

#define WHO_AM_I_VALUE  0x6D
#define COMMIT_REQUEST  0xA5
#define APPLY_REQUEST   0x5A
#define RESULT_OK       0x01

/** status + seq + 6 channels, the one coherent block. */
#define DATA_LEN        (2 + 2 * SENSE_ADC_6_CHANNELS)

/* ================================================================
 * Instance -> I2C address table
 * ================================================================ */

static const uint8_t id_table[] = { 0x28, 0x29, 0x2A, 0x2B };

#define NUM_INSTANCES  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    return (instance < NUM_INSTANCES) ? id_table[instance] : 0;
}

/* ================================================================
 * Per-instance driver state
 * ================================================================ */

typedef struct {
    uint16_t mv[SENSE_ADC_6_CHANNELS];
    uint16_t supply_mv;
    uint8_t  status;
    uint8_t  seq;
    uint8_t  rate_hz;
    uint8_t  oversamp;
    uint8_t  window;
    uint8_t  fw_version;   /* major<<4 | minor, as the tile reports it */
} adc6_state_t;

static adc6_state_t state[NUM_INSTANCES];

static adc6_state_t *state_for(tile_t *tile)
{
    for (uint8_t i = 0; i < NUM_INSTANCES; i++)
        if (id_table[i] == tile->id) return &state[i];
    return &state[0];
}

/* CH0..CH5 in pad order. Matches the tile's own channel ordering. */
static const uint8_t channel_pads[SENSE_ADC_6_CHANNELS] = { 2, 3, 6, 7, 8, 9 };

/* ================================================================
 * Private helpers
 * ================================================================ */

static void memzero(void *p, uint16_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) *b++ = 0;
}

static uint8_t read_regs(tile_t *tile, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (!tile->hal) return 0;
    return tile->hal->i2c_read(tile->hal->handle, tile->id, reg, buf, len) == 0;
}

static uint8_t write_regs(tile_t *tile, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    if (!tile->hal) return 0;
    return tile->hal->i2c_write(tile->hal->handle, tile->id, reg, (uint8_t *)buf, len) == 0;
}

static uint8_t read_u8(tile_t *tile, uint8_t reg, uint8_t *out)
{
    return read_regs(tile, reg, out, 1);
}

/** Refresh the cached settings from the tile. */
static void refresh_settings(tile_t *tile, adc6_state_t *s)
{
    uint8_t b[3];
    if (read_regs(tile, REG_RATE, b, 3)) {
        s->rate_hz  = b[0];
        s->oversamp = b[1];
        s->window   = b[2];
    }
}

/* ================================================================
 * Lifecycle
 * ================================================================ */

uint8_t tile_sense_adc_6_find(tiles_pal_t *hal, uint8_t instance)
{
    uint8_t addr = resolve_id(instance);
    if (!addr || !hal) return 0;
    if (hal->i2c_is_ready(hal->handle, addr) != 0) return 0;

    /* Something answered. Confirm it is ours before claiming the
     * address: several tiles share this address range. */
    uint8_t who = 0;
    if (hal->i2c_read(hal->handle, addr, REG_WHO_AM_I, &who, 1) != 0) return 0;
    return who == WHO_AM_I_VALUE;
}

void tile_sense_adc_6_init(tiles_pal_t *hal, uint8_t instance,
                           tile_t *tile, const sense_adc_6_cfg_t *cfg)
{
    memzero(tile, sizeof(tile_t));
    tile->hal = hal;
    tile->id  = (cfg && cfg->address) ? cfg->address : resolve_id(instance);

    if (!tile->id) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_adc_6: invalid instance");
        return;
    }

    adc6_state_t *s = state_for(tile);
    memzero(s, sizeof(adc6_state_t));

    if (hal->i2c_is_ready(hal->handle, tile->id) != 0) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_adc_6: tile not found");
        return;
    }

    uint8_t id[2];
    if (!read_regs(tile, REG_WHO_AM_I, id, 2) || id[0] != WHO_AM_I_VALUE) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_adc_6: wrong device at this address");
        return;
    }
    /* Keep the firmware version in driver state. tile_t.flags is reserved
     * for ISR event flags (tiles.h), whatever other drivers do with it. */
    s->fw_version = id[1];

    refresh_settings(tile, s);

    /* Apply only what the caller asked for, so a tile that was
     * configured and saved comes up the way it was left. */
    if (cfg && (cfg->rate_hz || cfg->oversamp || cfg->window)) {
        uint8_t rate = cfg->rate_hz  ? cfg->rate_hz  : s->rate_hz;
        uint8_t ovs  = cfg->oversamp ? cfg->oversamp : s->oversamp;
        uint8_t win  = cfg->window   ? cfg->window   : s->window;
        if (!tile_sense_adc_6_set_rate(tile, rate, ovs, win)) {
            tile->state = TILE_STATE_ERROR;
            TILE_ON_ERROR(tile, "sense_adc_6: settings rejected");
            return;
        }
    }

    tile->state = TILE_STATE_READY;
}

/* ================================================================
 * Runtime
 * ================================================================ */

uint8_t tile_sense_adc_6_update(tile_t *tile)
{
    adc6_state_t *s = state_for(tile);
    uint8_t d[DATA_LEN];

    if (!read_regs(tile, REG_STATUS, d, DATA_LEN)) {
        TILE_ON_ERROR(tile, "sense_adc_6: read failed");
        return 0;
    }

    s->status = d[0];
    s->seq    = d[1];
    for (uint8_t i = 0; i < SENSE_ADC_6_CHANNELS; i++)
        s->mv[i] = (uint16_t)(d[2 + 2 * i] | ((uint16_t)d[3 + 2 * i] << 8));

    /* Cheap and useful: the supply doubles as full scale, and it moves
     * only with the rail. */
    uint8_t v[2];
    if (read_regs(tile, REG_SUPPLY, v, 2))
        s->supply_mv = (uint16_t)(v[0] | ((uint16_t)v[1] << 8));

    return 1;
}

uint16_t tile_sense_adc_6_read_mv(tile_t *tile, uint8_t ch)
{
    if (ch >= SENSE_ADC_6_CHANNELS) return 0;
    return state_for(tile)->mv[ch];
}

void tile_sense_adc_6_read_all(tile_t *tile, uint16_t *out)
{
    if (!out) return;
    adc6_state_t *s = state_for(tile);
    for (uint8_t i = 0; i < SENSE_ADC_6_CHANNELS; i++) out[i] = s->mv[i];
}

uint8_t tile_sense_adc_6_sequence(tile_t *tile)
{
    return state_for(tile)->seq;
}

uint8_t tile_sense_adc_6_is_valid(tile_t *tile)
{
    return (state_for(tile)->status & SENSE_ADC_6_ST_VALID) ? 1 : 0;
}

uint8_t tile_sense_adc_6_status(tile_t *tile)
{
    return state_for(tile)->status;
}

uint16_t tile_sense_adc_6_supply_mv(tile_t *tile)
{
    return state_for(tile)->supply_mv;
}

/* ================================================================
 * Configuration
 * ================================================================ */

uint8_t tile_sense_adc_6_set_rate(tile_t *tile, uint8_t rate_hz,
                                  uint8_t oversamp, uint8_t window)
{
    adc6_state_t *s = state_for(tile);

    /* Check here as well as on the tile, so an obviously bad call costs
     * no bus traffic and cannot disturb a running acquisition. */
    if (rate_hz  < 1 || rate_hz  > SENSE_ADC_6_RATE_MAX)     return 0;
    if (oversamp < 1 || oversamp > SENSE_ADC_6_OVERSAMP_MAX) return 0;
    if (window   < 1 || window   > SENSE_ADC_6_WINDOW_MAX)   return 0;

    uint8_t w[4] = { rate_hz, oversamp, window, APPLY_REQUEST };
    if (!write_regs(tile, REG_RATE, w, 4)) {
        TILE_ON_ERROR(tile, "sense_adc_6: settings write failed");
        return 0;
    }

    /* The tile applies within one output period. Wait the slowest one
     * (1 s at 1 Hz) only if it has to: poll instead of sleeping long. */
    uint8_t result = 0;
    for (uint8_t tries = 0; tries < 50; tries++) {
        tile->hal->delay_ms(4);
        if (read_u8(tile, REG_APPLY, &result) && result != 0) break;
    }

    if (result != RESULT_OK) {
        /* Rejected: the tile restores its own settings, so pick them up
         * rather than leaving the cache showing what we asked for. */
        refresh_settings(tile, s);
        TILE_ON_ERROR(tile, "sense_adc_6: settings rejected");
        return 0;
    }

    s->rate_hz  = rate_hz;
    s->oversamp = oversamp;
    s->window   = window;
    return 1;
}

uint8_t tile_sense_adc_6_get_rate(tile_t *tile)
{
    return state_for(tile)->rate_hz;
}

uint8_t tile_sense_adc_6_get_oversamp(tile_t *tile)
{
    return state_for(tile)->oversamp;
}

uint8_t tile_sense_adc_6_get_window(tile_t *tile)
{
    return state_for(tile)->window;
}

uint8_t tile_sense_adc_6_save(tile_t *tile)
{
    uint8_t cur = 0;

    /* The tile stores address and settings together, so send back the
     * address it already has. Saving must not move a tile by accident. */
    if (!read_u8(tile, REG_ADDR_CUR, &cur)) return 0;

    uint8_t w[2] = { cur, COMMIT_REQUEST };
    if (!write_regs(tile, REG_ADDR_SET, w, 2)) {
        TILE_ON_ERROR(tile, "sense_adc_6: save failed");
        return 0;
    }

    /* Storing holds the bus for up to ~30 ms, and the tile stretches the
     * clock through it, so the first read may simply be slow. */
    uint8_t result = 0;
    for (uint8_t tries = 0; tries < 20; tries++) {
        tile->hal->delay_ms(5);
        if (read_u8(tile, REG_COMMIT, &result) && result != 0) break;
    }

    if (result != RESULT_OK) {
        TILE_ON_ERROR(tile, "sense_adc_6: save rejected");
        return 0;
    }
    return 1;
}

/* ================================================================
 * Advanced
 * ================================================================ */

uint8_t tile_sense_adc_6_set_address(tile_t *tile, uint8_t addr)
{
    if (addr < SENSE_ADC_6_ADDR_MIN || addr > SENSE_ADC_6_ADDR_MAX) return 0;

    uint8_t w[2] = { addr, COMMIT_REQUEST };
    if (!write_regs(tile, REG_ADDR_SET, w, 2)) {
        TILE_ON_ERROR(tile, "sense_adc_6: address write failed");
        return 0;
    }

    uint8_t result = 0;
    for (uint8_t tries = 0; tries < 20; tries++) {
        tile->hal->delay_ms(5);
        if (read_u8(tile, REG_COMMIT, &result) && result != 0) break;
    }

    if (result != RESULT_OK) {
        TILE_ON_ERROR(tile, "sense_adc_6: address rejected");
        return 0;
    }

    /* Deliberately leave tile->id alone: the tile keeps answering on the
     * old address until it is power cycled. Re-init against the new one
     * after that. */
    return 1;
}

uint8_t tile_sense_adc_6_channel_pad(uint8_t ch)
{
    return (ch < SENSE_ADC_6_CHANNELS) ? channel_pads[ch] : 0;
}
