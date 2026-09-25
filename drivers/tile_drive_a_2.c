/**
 * @file   tile_drive_a_2.c
 * @brief  Drive.A.2 (DAC63202W + 2x TPA2028D1) — complete driver implementation.
 *
 * Platform-agnostic. All bus access via tile->hal function pointers.
 * Supports both I2C and SPI for DAC access; amplifier control is
 * I2C-only (TPA2028D1 has no SPI interface).
 *
 * DAC63202W uses 16-bit big-endian registers with 7-bit addresses.
 * TPA2028D1 uses 8-bit registers with 8-bit addresses.
 *
 * Reference: TI SLASF73A (DAC63202W), TI SLOS660C (TPA2028D1)
 */

#include "tile_drive_a_2.h"
#include <stddef.h>

/* ================================================================
 * Instance → I2C address table
 * ================================================================ */

static const uint8_t id_table[] = {
    DAC63202W_I2C_ADDR_DEFAULT,  /* 0: 0x49 (A0 → VDD, default) */
    DAC63202W_I2C_ADDR_GND,      /* 1: 0x48 (A0 → GND) */
    DAC63202W_I2C_ADDR_SDA,      /* 2: 0x4A (A0 → SDA) */
    DAC63202W_I2C_ADDR_SCL,      /* 3: 0x4B (A0 → SCL) */
};

#define NUM_INSTANCES  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    return (instance < NUM_INSTANCES) ? id_table[instance] : 0;
}

/* ================================================================
 * Per-instance driver state
 * ================================================================ */

typedef struct {
    uint8_t  amp_addr;        /* TPA2028D1 address (0x58), or 0 if unavailable */
    uint8_t  gain[2];         /* Cached VOUT-GAIN-X per DAC channel */
    uint16_t vref_mv;         /* Effective full-scale voltage in mV */
    int8_t   muted_gain_db;   /* Gain saved at mute() so unmute() can restore */
    uint8_t  is_muted;        /* 1 if mute() has been called and not yet undone */
} drive_a_2_state_t;

static drive_a_2_state_t drv_state[NUM_INSTANCES];

static drive_a_2_state_t *state_for(tile_t *tile)
{
    if (tile->hal->buses & TILES_BUS_SPI) {
        uint8_t idx = tile->id < NUM_INSTANCES ? tile->id : 0;
        return &drv_state[idx];
    }
    for (uint8_t i = 0; i < NUM_INSTANCES; i++)
        if (id_table[i] == tile->id) return &drv_state[i];
    return &drv_state[0];
}

/* ================================================================
 * Private helpers
 * ================================================================ */

static void memzero(void *p, uint8_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) *b++ = 0;
}

/* --- DAC register access (bus-aware) --- */

static void dac_write(tile_t *tile, uint8_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)(value & 0xFF);
    if (tile->hal->buses & TILES_BUS_SPI)
        tile->hal->spi_write(tile->hal->handle, tile->id, reg, buf, 2);
    else
        tile->hal->i2c_write(tile->hal->handle, tile->id, reg, buf, 2);
}

static uint16_t dac_read(tile_t *tile, uint8_t reg)
{
    uint8_t buf[2] = {0, 0};
    if (tile->hal->buses & TILES_BUS_SPI)
        tile->hal->spi_read(tile->hal->handle, tile->id, reg, buf, 2);
    else
        tile->hal->i2c_read(tile->hal->handle, tile->id, reg, buf, 2);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

/* --- Amplifier register access (I2C only) --- */

static void amp_write(tile_t *tile, uint8_t reg, uint8_t value)
{
    drive_a_2_state_t *s = state_for(tile);
    if (!s->amp_addr) return;
    tile->hal->i2c_write(tile->hal->handle, s->amp_addr, reg, &value, 1);
}

static uint8_t amp_read(tile_t *tile, uint8_t reg)
{
    drive_a_2_state_t *s = state_for(tile);
    if (!s->amp_addr) return 0;
    uint8_t val = 0;
    tile->hal->i2c_read(tile->hal->handle, s->amp_addr, reg, &val, 1);
    return val;
}

/* --- Channel → register address helpers --- */

static uint8_t dac_data_reg(uint8_t ch)
{
    return (ch == 0) ? DAC63202W_REG_DAC_0_DATA : DAC63202W_REG_DAC_1_DATA;
}

static uint8_t dac_vout_cmp_reg(uint8_t ch)
{
    return (ch == 0) ? DAC63202W_REG_DAC_0_VOUT_CMP_CFG
                     : DAC63202W_REG_DAC_1_VOUT_CMP_CFG;
}

static uint8_t dac_func_cfg_reg(uint8_t ch)
{
    return (ch == 0) ? DAC63202W_REG_DAC_0_FUNC_CFG
                     : DAC63202W_REG_DAC_1_FUNC_CFG;
}

static uint8_t dac_margin_high_reg(uint8_t ch)
{
    return (ch == 0) ? DAC63202W_REG_DAC_0_MARGIN_HIGH
                     : DAC63202W_REG_DAC_1_MARGIN_HIGH;
}

static uint8_t dac_margin_low_reg(uint8_t ch)
{
    return (ch == 0) ? DAC63202W_REG_DAC_0_MARGIN_LOW
                     : DAC63202W_REG_DAC_1_MARGIN_LOW;
}

/* --- Reference voltage resolution --- */

static uint16_t resolve_vref(uint8_t gain_sel)
{
    switch (gain_sel) {
    case DRIVE_A_2_GAIN_1X_EXT:   return 3300;  /* assume 3.3V external */
    case DRIVE_A_2_GAIN_1X_VDD:   return 3300;  /* assume 3.3V VDD */
    case DRIVE_A_2_GAIN_1P5X_INT: return 1815;  /* 1210 * 1.5 */
    case DRIVE_A_2_GAIN_2X_INT:   return 2420;  /* 1210 * 2 */
    case DRIVE_A_2_GAIN_3X_INT:   return 3630;  /* 1210 * 3 */
    case DRIVE_A_2_GAIN_4X_INT:   return 4840;  /* 1210 * 4 */
    default:                      return 3300;
    }
}

/* ================================================================
 * Public API — Lifecycle
 * ================================================================ */

uint8_t tile_drive_a_2_find(tiles_pal_t *hal, uint8_t instance)
{
    if (hal->buses & TILES_BUS_SPI) {
        /* SPI: probe via GENERAL-STATUS read — check DEVICE-ID */
        uint8_t buf[2] = {0, 0};
        hal->spi_read(hal->handle, instance,
                      DAC63202W_REG_GENERAL_STATUS, buf, 2);
        uint16_t status = ((uint16_t)buf[0] << 8) | buf[1];
        return ((status >> 2) & 0x3F) == DAC63202W_DEVICE_ID;
    }

    uint8_t addr = resolve_id(instance);
    if (!addr) return 0;
    return (hal->i2c_is_ready(hal->handle, addr) == 0) ? 1 : 0;
}

void tile_drive_a_2_init(tiles_pal_t *hal, uint8_t instance,
                         tile_t *tile, const drive_a_2_cfg_t *cfg)
{
    memzero(tile, sizeof(tile_t));
    tile->hal = hal;

    if (hal->buses & TILES_BUS_SPI) {
        tile->id = instance;
    } else {
        tile->id = resolve_id(instance);
        if (!tile->id) {
            tile->state = TILE_STATE_ERROR;
            TILE_ON_ERROR(tile, "drive_a_2: invalid instance");
            return;
        }
    }

    /* Probe DAC */
    if (hal->buses & TILES_BUS_SPI) {
        /* SPI: verified by DEVICE-ID read below */
    } else {
        if (hal->i2c_is_ready(hal->handle, tile->id) != 0) {
            tile->state = TILE_STATE_ERROR;
            TILE_ON_ERROR(tile, "drive_a_2: DAC not found on bus");
            return;
        }
    }

    /* Read and verify DEVICE-ID */
    uint16_t status = dac_read(tile, DAC63202W_REG_GENERAL_STATUS);
    uint8_t dev_id = (status >> 2) & 0x3F;
    if (dev_id != DAC63202W_DEVICE_ID) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "drive_a_2: unexpected DAC device ID");
        return;
    }

    /* Initialize per-instance state */
    drive_a_2_state_t *s = state_for(tile);
    memzero(s, sizeof(drive_a_2_state_t));

    /* Apply gain config (default: 1x external VREF — tile has VREF tied to VDD) */
    uint8_t gain_sel = cfg ? cfg->gain : DRIVE_A_2_GAIN_1X_EXT;
    s->gain[0] = gain_sel;
    s->gain[1] = gain_sel;
    s->vref_mv = resolve_vref(gain_sel);

    /* Power up both VOUT channels, power down IOUT.
     * No software reset — preserve NVM defaults for analog config. */
    uint16_t common_cfg = 0x0201;  /* VOUT-PDN-0/1 = 00, IOUT-PDN-0/1 = 1 */
    if (gain_sel >= DRIVE_A_2_GAIN_1P5X_INT)
        common_cfg |= (1 << 12);  /* EN-INT-REF */
    dac_write(tile, DAC63202W_REG_COMMON_CONFIG, common_cfg);

    /* Set both DAC outputs to mid-scale and allow settling.
     * The amp input is referenced to VDD/2, so mid-scale = zero
     * differential input — this prevents amp saturation on wake. */
    dac_write(tile, dac_data_reg(0), 2048 << 4);
    dac_write(tile, dac_data_reg(1), 2048 << 4);
    hal->delay_ms(50);

    /* Set gain on both channels (if non-default) */
    if (gain_sel != DRIVE_A_2_GAIN_1X_EXT) {
        for (uint8_t ch = 0; ch < 2; ch++) {
            uint16_t vout_cfg = dac_read(tile, dac_vout_cmp_reg(ch));
            vout_cfg &= ~(0x07 << 10);
            vout_cfg |= ((uint16_t)gain_sel << 10);
            dac_write(tile, dac_vout_cmp_reg(ch), vout_cfg);
        }
    }

    /* Probe and configure amplifiers (I2C only).
     * The TPA2028D1 AGC can cause saturation if it ramps gain while
     * there is any DC offset at the input.  Sequence:
     *   1. Put amp in software shutdown
     *   2. Configure for fixed gain, AGC compression off
     *   3. Wake amp — DAC is already settled at mid-scale */
    if (!(hal->buses & TILES_BUS_SPI)) {
        if (hal->i2c_is_ready(hal->handle, TPA2028D1_I2C_ADDR) == 0) {
            s->amp_addr = TPA2028D1_I2C_ADDR;

            /* Shutdown amp while configuring */
            amp_write(tile, TPA2028D1_REG_FUNC_CTRL, 0xE2);  /* EN=1, SWS=1, NG_EN=0 */
            hal->delay_ms(5);

            /* Set fixed gain (default 6 dB), AGC compression off */
            int8_t amp_gain = (cfg && cfg->amp_gain_db) ? cfg->amp_gain_db : 6;
            amp_write(tile, TPA2028D1_REG_AGC_GAIN,
                      (uint8_t)(amp_gain & 0x3F));
            amp_write(tile, TPA2028D1_REG_AGC_CTRL2, 0xC0);  /* max 30 dB, comp 1:1 */
            amp_write(tile, TPA2028D1_REG_AGC_CTRL1, 0x80);  /* limiter off */

            /* Wake amp — DAC is settled, no transient */
            amp_write(tile, TPA2028D1_REG_FUNC_CTRL, 0xC2);  /* EN=1, SWS=0, NG_EN=0 */
            hal->delay_ms(10);
        }
    }

    tile->state = TILE_STATE_READY;
}

void tile_drive_a_2_sleep(tile_t *tile)
{
    /* Power down both VOUT channels to Hi-Z */
    uint16_t common_cfg = dac_read(tile, DAC63202W_REG_COMMON_CONFIG);
    common_cfg |= (0x03 << 10);  /* VOUT-PDN-0 = 11 (Hi-Z) */
    common_cfg |= (0x03 << 1);   /* VOUT-PDN-1 = 11 (Hi-Z) */
    dac_write(tile, DAC63202W_REG_COMMON_CONFIG, common_cfg);

    /* Put amps into software shutdown */
    uint8_t reg1 = amp_read(tile, TPA2028D1_REG_FUNC_CTRL);
    reg1 |= (1 << 5);   /* SWS = 1 */
    amp_write(tile, TPA2028D1_REG_FUNC_CTRL, reg1);

    tile->state = TILE_STATE_SLEEPING;
}

void tile_drive_a_2_wake(tile_t *tile)
{
    drive_a_2_state_t *s = state_for(tile);

    /* Power up both VOUT channels, set EN-INT-REF if needed */
    uint16_t common_cfg = 0x0201;
    if (s->gain[0] >= DRIVE_A_2_GAIN_1P5X_INT ||
        s->gain[1] >= DRIVE_A_2_GAIN_1P5X_INT)
        common_cfg |= (1 << 12);
    dac_write(tile, DAC63202W_REG_COMMON_CONFIG, common_cfg);

    /* Settle DAC at mid-scale before waking amp */
    dac_write(tile, dac_data_reg(0), 2048 << 4);
    dac_write(tile, dac_data_reg(1), 2048 << 4);
    tile->hal->delay_ms(50);

    /* Restore gain on both channels */
    for (uint8_t ch = 0; ch < 2; ch++) {
        uint16_t vout_cfg = dac_read(tile, dac_vout_cmp_reg(ch));
        vout_cfg &= ~(0x07 << 10);
        vout_cfg |= ((uint16_t)s->gain[ch] << 10);
        dac_write(tile, dac_vout_cmp_reg(ch), vout_cfg);
    }

    /* Wake amp after DAC is settled */
    amp_write(tile, TPA2028D1_REG_FUNC_CTRL, 0xC2);  /* EN=1, SWS=0, NG_EN=0 */
    tile->hal->delay_ms(10);

    tile->state = TILE_STATE_READY;
}

void tile_drive_a_2_reset(tile_t *tile)
{
    dac_write(tile, DAC63202W_REG_COMMON_TRIGGER, 0x0A00);
    tile->state = TILE_STATE_NONE;
}

/* ================================================================
 * Public API — DAC output
 * ================================================================ */

void tile_drive_a_2_set(tile_t *tile, uint8_t channel, uint16_t value)
{
    if (channel > 1) return;
    if (value > DAC63202W_DAC_MAX) value = DAC63202W_DAC_MAX;
    dac_write(tile, dac_data_reg(channel), (uint16_t)(value << 4));
}

void tile_drive_a_2_set_mv(tile_t *tile, uint8_t channel, uint16_t mv)
{
    if (channel > 1) return;
    drive_a_2_state_t *s = state_for(tile);
    if (s->vref_mv == 0) return;
    uint32_t code = ((uint32_t)mv * 4096) / s->vref_mv;
    if (code > DAC63202W_DAC_MAX) code = DAC63202W_DAC_MAX;
    tile_drive_a_2_set(tile, channel, (uint16_t)code);
}

uint16_t tile_drive_a_2_get(tile_t *tile, uint8_t channel)
{
    if (channel > 1) return 0;
    uint16_t raw = dac_read(tile, dac_data_reg(channel));
    return (raw >> 4) & 0x0FFF;
}

/* ================================================================
 * Public API — DAC configuration
 * ================================================================ */

void tile_drive_a_2_set_gain(tile_t *tile, uint8_t channel,
                             drive_a_2_gain_t gain)
{
    if (channel > 1) return;
    drive_a_2_state_t *s = state_for(tile);

    /* Update VOUT-GAIN-X in DAC-X-VOUT-CMP-CONFIG */
    uint16_t vout_cfg = dac_read(tile, dac_vout_cmp_reg(channel));
    vout_cfg &= ~(0x07 << 10);
    vout_cfg |= ((uint16_t)gain << 10);
    dac_write(tile, dac_vout_cmp_reg(channel), vout_cfg);

    /* Update EN-INT-REF in COMMON-CONFIG if needed */
    uint16_t common_cfg = dac_read(tile, DAC63202W_REG_COMMON_CONFIG);
    if (gain >= DRIVE_A_2_GAIN_1P5X_INT)
        common_cfg |= (1 << 12);
    else if (s->gain[1 - channel] < DRIVE_A_2_GAIN_1P5X_INT)
        common_cfg &= ~(1 << 12);  /* safe to disable if neither channel needs it */
    dac_write(tile, DAC63202W_REG_COMMON_CONFIG, common_cfg);

    /* Cache */
    s->gain[channel] = (uint8_t)gain;
    s->vref_mv = resolve_vref((uint8_t)gain);
}

/* ================================================================
 * Public API — Waveform generation
 * ================================================================ */

void tile_drive_a_2_set_waveform(tile_t *tile, uint8_t channel,
                                 drive_a_2_wave_t wave)
{
    if (channel > 1) return;
    uint16_t func_cfg = dac_read(tile, dac_func_cfg_reg(channel));
    func_cfg &= ~(0x07 << 8);                 /* clear FUNC-CONFIG-X */
    func_cfg |= ((uint16_t)wave << 8);         /* set waveform type */
    dac_write(tile, dac_func_cfg_reg(channel), func_cfg);
}

/* COMMON-DAC-TRIG (SLASF73A §6.6.11): START-FUNC-0 is bit 0 and
 * START-FUNC-1 bit 12, both R/W and level-sensitive (0 = stop). The
 * other bits are write-only self-clearing triggers. Writing one
 * channel's START bit alone would clear the other's and stop its
 * generator, so both START bits are read back and preserved. */
#define DAC_TRIG_START_MASK  0x1001u

static uint16_t start_func_bit(uint8_t ch)
{
    return (ch == 0) ? 0x0001u : 0x1000u;
}

void tile_drive_a_2_start_waveform(tile_t *tile, uint8_t channel)
{
    if (channel > 1) return;
    uint16_t trig = dac_read(tile, DAC63202W_REG_COMMON_DAC_TRIG) & DAC_TRIG_START_MASK;
    trig |= start_func_bit(channel);
    dac_write(tile, DAC63202W_REG_COMMON_DAC_TRIG, trig);
}

void tile_drive_a_2_stop_waveform(tile_t *tile, uint8_t channel)
{
    if (channel > 1) return;
    uint16_t trig = dac_read(tile, DAC63202W_REG_COMMON_DAC_TRIG) & DAC_TRIG_START_MASK;
    trig &= (uint16_t)~start_func_bit(channel);
    dac_write(tile, DAC63202W_REG_COMMON_DAC_TRIG, trig);

    uint16_t func_cfg = dac_read(tile, dac_func_cfg_reg(channel));
    func_cfg &= ~(0x07 << 8);
    func_cfg |= ((uint16_t)DRIVE_A_2_WAVE_OFF << 8);
    dac_write(tile, dac_func_cfg_reg(channel), func_cfg);
}

void tile_drive_a_2_set_slew_rate(tile_t *tile, uint8_t channel,
                                  drive_a_2_slew_t slew)
{
    if (channel > 1) return;
    uint16_t func_cfg = dac_read(tile, dac_func_cfg_reg(channel));
    func_cfg &= ~0x000F;                       /* clear SLEW-RATE-X[3:0] */
    func_cfg |= ((uint16_t)slew & 0x000F);
    dac_write(tile, dac_func_cfg_reg(channel), func_cfg);
}

void tile_drive_a_2_set_code_step(tile_t *tile, uint8_t channel,
                                  drive_a_2_step_t step)
{
    if (channel > 1) return;
    uint16_t func_cfg = dac_read(tile, dac_func_cfg_reg(channel));
    func_cfg &= ~(0x07 << 4);                  /* clear CODE-STEP-X[6:4] */
    func_cfg |= (((uint16_t)step & 0x07) << 4);
    dac_write(tile, dac_func_cfg_reg(channel), func_cfg);
}

void tile_drive_a_2_set_margins(tile_t *tile, uint8_t channel,
                                uint16_t low, uint16_t high)
{
    if (channel > 1) return;
    if (low  > DAC63202W_DAC_MAX) low  = DAC63202W_DAC_MAX;
    if (high > DAC63202W_DAC_MAX) high = DAC63202W_DAC_MAX;
    /* Datasheet requires margin_high > margin_low — silently swap so
     * caller doesn't accidentally lock the function generator. */
    if (low > high) {
        uint16_t t = low; low = high; high = t;
    }
    /* Margin registers use the same left-shifted-by-4 alignment as
     * DAC-X-DATA (12-bit value occupies bits[15:4]). */
    dac_write(tile, dac_margin_low_reg(channel),  (uint16_t)(low  << 4));
    dac_write(tile, dac_margin_high_reg(channel), (uint16_t)(high << 4));
}

void tile_drive_a_2_set_phase(tile_t *tile, uint8_t channel,
                              drive_a_2_phase_t phase)
{
    if (channel > 1) return;
    uint16_t func_cfg = dac_read(tile, dac_func_cfg_reg(channel));
    func_cfg &= ~(0x03 << 11);                 /* clear PHASE-SEL-X[12:11] */
    func_cfg |= (((uint16_t)phase & 0x03) << 11);
    dac_write(tile, dac_func_cfg_reg(channel), func_cfg);
}

void tile_drive_a_2_set_waveform_params(tile_t *tile, uint8_t channel,
                                        drive_a_2_wave_t wave,
                                        drive_a_2_step_t step,
                                        drive_a_2_slew_t slew)
{
    if (channel > 1) return;
    /* Full-scale margins for max amplitude swing.
     * Use the existing setters so the bit-mask logic stays in one place. */
    tile_drive_a_2_set_margins(tile, channel, 0, DAC63202W_DAC_MAX);

    /* Single read-modify-write so we don't bounce slew/step/wave/phase
     * separately across three round-trips. */
    uint16_t func_cfg = dac_read(tile, dac_func_cfg_reg(channel));
    func_cfg &= ~((0x07 << 8) | (0x07 << 4) | 0x000F | (1 << 7));
    func_cfg |= ((uint16_t)wave & 0x07) << 8;   /* FUNC-CONFIG-X */
    func_cfg |= ((uint16_t)step & 0x07) << 4;   /* CODE-STEP-X */
    func_cfg |= ((uint16_t)slew & 0x0F);        /* SLEW-RATE-X */
    /* LOG-SLEW-EN-X (bit 7) cleared — linear slew. */
    dac_write(tile, dac_func_cfg_reg(channel), func_cfg);
}

/* ================================================================
 * Public API — Amplifier control
 * ================================================================ */

void tile_drive_a_2_amp_set_gain(tile_t *tile, int8_t gain_db)
{
    if (gain_db < -28) gain_db = -28;
    if (gain_db > 30) gain_db = 30;
    amp_write(tile, TPA2028D1_REG_AGC_GAIN, (uint8_t)(gain_db & 0x3F));
}

int8_t tile_drive_a_2_amp_get_gain(tile_t *tile)
{
    uint8_t raw = amp_read(tile, TPA2028D1_REG_AGC_GAIN) & 0x3F;
    /* Sign-extend 6-bit two's complement to int8_t */
    if (raw & 0x20) raw |= 0xC0;
    return (int8_t)raw;
}

void tile_drive_a_2_amp_enable(tile_t *tile)
{
    uint8_t reg1 = amp_read(tile, TPA2028D1_REG_FUNC_CTRL);
    reg1 |= (1 << 6);   /* EN = 1 */
    reg1 &= ~(1 << 5);  /* SWS = 0 */
    amp_write(tile, TPA2028D1_REG_FUNC_CTRL, reg1);
}

void tile_drive_a_2_amp_disable(tile_t *tile)
{
    uint8_t reg1 = amp_read(tile, TPA2028D1_REG_FUNC_CTRL);
    reg1 |= (1 << 5);   /* SWS = 1 (software shutdown) */
    amp_write(tile, TPA2028D1_REG_FUNC_CTRL, reg1);
}

void tile_drive_a_2_amp_set_agc(tile_t *tile, const drive_a_2_agc_cfg_t *cfg)
{
    if (!cfg) return;

    int8_t fixed_gain = cfg->fixed_gain_db;
    if (fixed_gain < -28) fixed_gain = -28;
    if (fixed_gain > 30)  fixed_gain = 30;

    amp_write(tile, TPA2028D1_REG_AGC_ATTACK,
              cfg->attack & 0x3F);

    amp_write(tile, TPA2028D1_REG_AGC_RELEASE,
              cfg->release & 0x3F);

    amp_write(tile, TPA2028D1_REG_AGC_HOLD,
              cfg->hold & 0x3F);

    amp_write(tile, TPA2028D1_REG_AGC_GAIN,
              (uint8_t)(fixed_gain & 0x3F));

    /* Register 0x07 first: [7:4] max gain (value - 18), [1:0] compression
     * ratio. The limiter-disable bit in 0x06 is only honoured while the
     * compression ratio is 1:1 (SLOS660C Table 11), so the ratio must be
     * in place before 0x06 is written. */
    uint8_t max_gain = cfg->max_gain_db;
    if (max_gain < 18) max_gain = 18;
    if (max_gain > 30) max_gain = 30;
    uint8_t ctrl2 = ((max_gain - 18) << 4) | (cfg->compression & 0x03);
    amp_write(tile, TPA2028D1_REG_AGC_CTRL2, ctrl2);

    /* Register 0x06: [7] limiter disable, [6:5] noise gate, [4:0] limiter level */
    uint8_t ctrl1 = (cfg->limiter_level & 0x1F)
                  | ((cfg->noise_gate & 0x03) << 5);
    if (cfg->compression == DRIVE_A_2_COMP_1_1)
        ctrl1 |= (1 << 7);  /* disable limiter when compression is off */
    amp_write(tile, TPA2028D1_REG_AGC_CTRL1, ctrl1);
}

uint8_t tile_drive_a_2_amp_read_status(tile_t *tile)
{
    return amp_read(tile, TPA2028D1_REG_FUNC_CTRL);
}

/* ================================================================
 * Public API — Status
 * ================================================================ */

uint16_t tile_drive_a_2_read_status(tile_t *tile)
{
    return dac_read(tile, DAC63202W_REG_GENERAL_STATUS);
}

/* ================================================================
 * Public API — NVM (shadow flash for power-on defaults)
 * ================================================================ */

/* Datasheet section 5.8 / 6.3.3: an NVM write blocks the bus for the
 * specified cycle time. 50 ms is comfortably above the typical
 * spec; an NVM-RELOAD is faster (handful of ms) but uses the same
 * delay for simplicity. */
#define DRIVE_A_2_NVM_WRITE_DELAY_MS  50

void tile_drive_a_2_nvm_save(tile_t *tile)
{
    /* COMMON-TRIGGER bit 1 = NVM-PROG (auto-resetting) */
    dac_write(tile, DAC63202W_REG_COMMON_TRIGGER, 0x0002);
    tile->hal->delay_ms(DRIVE_A_2_NVM_WRITE_DELAY_MS);
}

void tile_drive_a_2_nvm_reload(tile_t *tile)
{
    /* COMMON-TRIGGER bit 0 = NVM-RELOAD (auto-resetting) */
    dac_write(tile, DAC63202W_REG_COMMON_TRIGGER, 0x0001);
    tile->hal->delay_ms(DRIVE_A_2_NVM_WRITE_DELAY_MS);

    /* The reload may have changed the active gain; re-derive the
     * cached vref from whatever is now in DAC-0-VOUT-CMP-CONFIG so
     * set_mv() keeps producing sensible output codes. */
    drive_a_2_state_t *s = state_for(tile);
    uint16_t vout0 = dac_read(tile, dac_vout_cmp_reg(0));
    uint16_t vout1 = dac_read(tile, dac_vout_cmp_reg(1));
    s->gain[0] = (vout0 >> 10) & 0x07;
    s->gain[1] = (vout1 >> 10) & 0x07;
    s->vref_mv = resolve_vref(s->gain[0]);
}

/* ================================================================
 * Public API — Raw register access (escape hatches)
 * ================================================================ */

uint16_t tile_drive_a_2_read_reg(tile_t *tile, uint8_t reg)
{
    return dac_read(tile, reg);
}

void tile_drive_a_2_write_reg(tile_t *tile, uint8_t reg, uint16_t value)
{
    dac_write(tile, reg, value);
}

/* ================================================================
 * Tier-2 — runtime helpers
 *
 * All blocking calls use tile->hal->delay_ms. Sine synthesis uses a
 * 64-entry quarter-wave LUT + symmetry to stay integer-only on
 * Cortex-M0+ (no libm, no float).
 * ================================================================ */

/* Update rate for the software sine-LUT path (chirp). 8 kHz gives
 * a Nyquist of 4 kHz which covers most "indicator beep" tones; the
 * DAC's own function generator handles steady tones. */
#define DRIVE_A_2_SW_SAMPLE_RATE_HZ   8000
#define DRIVE_A_2_SW_SAMPLE_PERIOD_MS 1   /* used for chunked delay accounting */

/* Quarter-wave Q11 sine LUT — sin(π/2 × i/64) × 2047 for i=0..63.
 * Combined with the quadrant-symmetry switch in sine_unipolar()
 * this synthesises a full sine over phase 0..65535 → 0..2π. */
static const int16_t DRIVE_A_2_QSINE[64] = {
        0,    50,   100,   151,   201,   251,   300,   350,
      399,   449,   497,   546,   594,   642,   690,   737,
      783,   830,   875,   920,   965,  1009,  1052,  1095,
     1137,  1179,  1219,  1259,  1299,  1337,  1375,  1411,
     1447,  1483,  1517,  1550,  1582,  1614,  1644,  1674,
     1702,  1729,  1756,  1781,  1805,  1828,  1850,  1871,
     1891,  1910,  1927,  1944,  1959,  1973,  1986,  1997,
     2008,  2017,  2025,  2032,  2037,  2041,  2045,  2046,
};

/* Return a 12-bit unipolar DAC code (0..4095) for the given Q16
 * phase. Centre = 2048; full ± swing covers 1..4095. */
static uint16_t sine_unipolar(uint16_t phase)
{
    uint8_t q   = (uint8_t)((phase >> 14) & 0x03);
    uint8_t idx = (uint8_t)((phase >> 8) & 0x3F);
    int16_t v;
    switch (q) {
        case 0: v =  DRIVE_A_2_QSINE[idx];      break;
        case 1: v =  DRIVE_A_2_QSINE[63 - idx]; break;
        case 2: v = -DRIVE_A_2_QSINE[idx];      break;
        default:v = -DRIVE_A_2_QSINE[63 - idx]; break;
    }
    /* Map -2046..+2046 to ~1..4094 around mid-scale 2048. */
    int32_t code = (int32_t)2048 + v;
    if (code < 0) code = 0;
    if (code > DAC63202W_DAC_MAX) code = DAC63202W_DAC_MAX;
    return (uint16_t)code;
}

/* Iterate the body once for a single channel, or twice for BOTH. */
static void for_each_channel(drive_a_2_channel_t ch,
                             void (*body)(tile_t *, uint8_t, void *),
                             tile_t *tile, void *arg)
{
    if (ch == DRIVE_A_2_CH_BOTH) {
        body(tile, 0, arg);
        body(tile, 1, arg);
    } else if (ch == DRIVE_A_2_CH_LEFT || ch == DRIVE_A_2_CH_RIGHT) {
        body(tile, (uint8_t)ch, arg);
    }
}

/* --- play_silence ----------------------------------------------- */

static void silence_one(tile_t *tile, uint8_t ch, void *arg)
{
    (void)arg;
    /* Stop any running waveform so set() takes effect immediately. */
    tile_drive_a_2_stop_waveform(tile, ch);
    tile_drive_a_2_set(tile, ch, 2048);  /* mid-scale = 0 V differential */
}

void tile_drive_a_2_play_silence(tile_t *tile, drive_a_2_channel_t channel,
                                 uint16_t ms)
{
    for_each_channel(channel, silence_one, tile, NULL);
    if (ms) tile->hal->delay_ms(ms);
}

/* --- play_tone --------------------------------------------------- */

/* Pick the slew code whose sine pitch is nearest `freq_hz`.
 *
 * The DAC63202W sine generator plays 24 fixed points per cycle, one
 * per slew step, so f_sine = 1 / (24 × time_step) (SLASF73A Eq 8):
 * the code step and margins do not affect it. SINE_HZ_X10[k] is that
 * pitch for SLEW-RATE code k+1 (Table 6-30 step times), in 0.1 Hz.
 * "Nearest" is taken on a log scale: pick the higher pitch when
 * freq² > f_hi × f_lo (geometric midpoint). */
static const uint32_t SINE_HZ_X10[15] = {
    104167, 52083, 34722, 23148, 15409, 10293, 6862,
      4573,  3048,  1742,   995,   569,   325,  163,  81,
};

static void pick_wave_params(uint16_t freq_hz,
                             drive_a_2_slew_t *out_slew,
                             drive_a_2_step_t *out_step)
{
    uint32_t f_x10 = (uint32_t)freq_hz * 10u;
    uint8_t k = 14;                       /* slowest: 5128 µs, 8.1 Hz */
    for (uint8_t i = 0; i < 15; i++) {
        if (f_x10 >= SINE_HZ_X10[i]) {
            k = i;
            /* Between pitch i (below f) and i-1 (above f): take i-1
             * when f is past their geometric midpoint. The products
             * reach ~5.4e9, hence 64-bit. */
            if (i > 0 &&
                (uint64_t)f_x10 * f_x10 >
                (uint64_t)SINE_HZ_X10[i] * SINE_HZ_X10[i - 1])
                k = (uint8_t)(i - 1);
            break;
        }
        k = i;                            /* f below every pitch so far */
    }
    *out_slew = (drive_a_2_slew_t)(k + 1);
    *out_step = DRIVE_A_2_STEP_1_LSB;     /* unused by the sine generator */
}

typedef struct {
    drive_a_2_wave_t wave;
    drive_a_2_step_t step;
    drive_a_2_slew_t slew;
} tone_args_t;

static void tone_start_one(tile_t *tile, uint8_t ch, void *arg)
{
    tone_args_t *t = (tone_args_t *)arg;
    tile_drive_a_2_set_waveform_params(tile, ch, t->wave, t->step, t->slew);
    tile_drive_a_2_start_waveform(tile, ch);
}

static void tone_stop_one(tile_t *tile, uint8_t ch, void *arg)
{
    (void)arg;
    tile_drive_a_2_stop_waveform(tile, ch);
    /* Park at mid-scale so we don't leave the amp pinned at a rail. */
    tile_drive_a_2_set(tile, ch, 2048);
}

void tile_drive_a_2_play_tone(tile_t *tile, drive_a_2_channel_t channel,
                              uint16_t freq_hz, uint16_t ms)
{
    if (freq_hz == 0 || ms == 0) return;

    drive_a_2_state_t *s = state_for(tile);
    uint8_t was_muted = s->is_muted;
    if (was_muted) tile_drive_a_2_unmute(tile, channel);

    tone_args_t args;
    args.wave = DRIVE_A_2_WAVE_SINE;
    pick_wave_params(freq_hz, &args.slew, &args.step);

    for_each_channel(channel, tone_start_one, tile, &args);
    tile->hal->delay_ms(ms);
    for_each_channel(channel, tone_stop_one, tile, NULL);

    if (was_muted) tile_drive_a_2_mute(tile, channel);
}

/* --- play_chirp -------------------------------------------------- */

void tile_drive_a_2_play_chirp(tile_t *tile, drive_a_2_channel_t channel,
                               uint16_t start_hz, uint16_t end_hz,
                               uint16_t ms)
{
    if (ms == 0) return;
    if (start_hz == 0 && end_hz == 0) return;

    drive_a_2_state_t *s = state_for(tile);
    uint8_t was_muted = s->is_muted;
    if (was_muted) tile_drive_a_2_unmute(tile, channel);

    /* Stop any chip-side waveform so direct DAC writes are visible. */
    if (channel == DRIVE_A_2_CH_BOTH) {
        tile_drive_a_2_stop_waveform(tile, 0);
        tile_drive_a_2_stop_waveform(tile, 1);
    } else {
        tile_drive_a_2_stop_waveform(tile, (uint8_t)channel);
    }

    /* Sample rate is fixed at DRIVE_A_2_SW_SAMPLE_RATE_HZ. We update
     * the DAC each ~125 µs; on a 4 MHz I²C bus a 4-byte write is
     * <40 µs so this fits with margin. The hal->delay_ms granularity
     * limits us to 1 ms slots — we batch SAMPLES_PER_MS writes per
     * tick. */
    const uint32_t SAMPLES_PER_MS = DRIVE_A_2_SW_SAMPLE_RATE_HZ / 1000u;
    uint32_t total_samples = (uint32_t)ms * SAMPLES_PER_MS;
    if (total_samples == 0) return;

    /* Phase is Q16; phase step = freq × 65536 / fs. Linear sweep:
     *   freq(t) = start + (end - start) × t / total. */
    int32_t f0 = (int32_t)start_hz;
    int32_t f1 = (int32_t)end_hz;
    uint32_t phase = 0;

    for (uint32_t i = 0; i < total_samples; i++) {
        int32_t f_now = f0 + ((f1 - f0) * (int32_t)i) / (int32_t)total_samples;
        if (f_now < 0) f_now = 0;
        uint32_t step = ((uint32_t)f_now * 65536u) / DRIVE_A_2_SW_SAMPLE_RATE_HZ;
        uint16_t code = sine_unipolar((uint16_t)phase);
        if (channel == DRIVE_A_2_CH_BOTH) {
            tile_drive_a_2_set(tile, 0, code);
            tile_drive_a_2_set(tile, 1, code);
        } else {
            tile_drive_a_2_set(tile, (uint8_t)channel, code);
        }
        phase = (phase + step) & 0xFFFF;

        /* Pace ~1 ms of samples per delay_ms(1) — the bus + delay
         * together approximate the 8 kHz update cadence. */
        if ((i + 1) % SAMPLES_PER_MS == 0) {
            tile->hal->delay_ms(DRIVE_A_2_SW_SAMPLE_PERIOD_MS);
        }
    }

    /* Park at mid-scale. */
    if (channel == DRIVE_A_2_CH_BOTH) {
        tile_drive_a_2_set(tile, 0, 2048);
        tile_drive_a_2_set(tile, 1, 2048);
    } else {
        tile_drive_a_2_set(tile, (uint8_t)channel, 2048);
    }

    if (was_muted) tile_drive_a_2_mute(tile, channel);
}

/* --- set_volume_pct --------------------------------------------- */

void tile_drive_a_2_set_volume_pct(tile_t *tile, drive_a_2_channel_t channel,
                                   uint8_t pct)
{
    (void)channel;  /* shared amp address — no per-channel split possible */
    if (pct > 100) pct = 100;
    /* Linear-in-dB across the full -28..+30 dB programmable range
     * (58 dB span). Documented in the header. */
    int32_t gain_db = -28 + ((int32_t)pct * 58) / 100;
    tile_drive_a_2_amp_set_gain(tile, (int8_t)gain_db);

    /* Keep the muted_gain shadow in sync if the user adjusts volume
     * while muted — unmute() should restore the most-recently-set
     * volume, not the volume at the time of mute(). */
    drive_a_2_state_t *s = state_for(tile);
    if (s->is_muted) s->muted_gain_db = (int8_t)gain_db;
}

/* --- mute / unmute ---------------------------------------------- */

void tile_drive_a_2_mute(tile_t *tile, drive_a_2_channel_t channel)
{
    (void)channel;  /* shared amp address — both physical amps mute */
    drive_a_2_state_t *s = state_for(tile);
    if (!s->is_muted) {
        s->muted_gain_db = tile_drive_a_2_amp_get_gain(tile);
        s->is_muted = 1;
    }
    tile_drive_a_2_amp_disable(tile);
}

void tile_drive_a_2_unmute(tile_t *tile, drive_a_2_channel_t channel)
{
    (void)channel;
    drive_a_2_state_t *s = state_for(tile);
    if (s->is_muted) {
        tile_drive_a_2_amp_set_gain(tile, s->muted_gain_db);
        s->is_muted = 0;
    }
    tile_drive_a_2_amp_enable(tile);
}
