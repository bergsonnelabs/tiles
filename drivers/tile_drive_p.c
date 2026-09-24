/**
 * @file   tile_drive_p.c
 * @brief  Piezoelectric haptic driver implementation (BOS1921).
 */

#include "tile_drive_p.h"
#include <stddef.h>

/* -------------------------------------------------------------- */
/* Instance mapping                                                */
/* -------------------------------------------------------------- */

static const uint8_t id_table[] = {
    BOS1921_I2C_ADDR_DEFAULT,   /* instance 0 — power-on address (0x44) */
    BOS1921_I2C_ADDR_SECOND,    /* instance 1 — first of a pair, reassigned (0x45) */
    BOS1921_I2C_ADDR_THIRD,     /* instance 2 — second of a pair, reassigned (0x46) */
};

#define ID_TABLE_LEN  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    if (instance >= ID_TABLE_LEN) return 0x00;
    return id_table[instance];
}

/* -------------------------------------------------------------- */
/* Per-instance shadow state                                       */
/* -------------------------------------------------------------- */

/*
 * The BOS1921 has a single read-back register (selected via
 * COMM.RDADDR) and read-modify-write through that path costs
 * two I2C transactions. Several CONFIG bits also need to persist
 * across set_mode() calls (which writes the whole CONFIG register).
 * Track those bits in driver-side shadow state.
 *
 * - cfg_persistent_bits: GAINS, GAIND, RET — OR'd into every
 *   CONFIG write performed by set_mode() / sleep().
 * - comm_persistent_bits: TOUT — OR'd into the COMM write done by
 *   bos_set_return_reg() so the bit survives return-register changes.
 * - parcap: full PARCAP value (init computes the lower 8 bits;
 *   set_upi() flips bit 9). Tracked so we can rewrite it without
 *   reading back through the RDADDR path.
 */
typedef struct {
    uint16_t cfg_persistent_bits;
    uint16_t comm_persistent_bits;
    uint16_t parcap;
} drive_p_state_t;

static drive_p_state_t drv_state[ID_TABLE_LEN];

/* Shadow state is keyed by the tile INSTANCE (pointer), NOT its I2C address.
 * Two Drive.P on separate buses can legitimately share address 0x44 (the v1
 * Ring's split-bus layout does exactly this), so an address key would alias
 * their shadow state onto one slot. Each tile claims a free slot on its first
 * lookup (during init) and keeps it. */
static tile_t *state_owner[ID_TABLE_LEN];

static drive_p_state_t *state_for(tile_t *tile)
{
    for (uint8_t i = 0; i < ID_TABLE_LEN; i++)
        if (state_owner[i] == tile) return &drv_state[i];
    for (uint8_t i = 0; i < ID_TABLE_LEN; i++)
        if (state_owner[i] == NULL) { state_owner[i] = tile; return &drv_state[i]; }
    return &drv_state[0];   /* more Drive.P than slots — bump ID_TABLE_LEN */
}

/* -------------------------------------------------------------- */
/* Private helpers                                                 */
/* -------------------------------------------------------------- */

static void bos_write(tile_t* tile, uint8_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)(value & 0xFF);
    tile->hal->i2c_write(tile->hal->handle, tile->id, reg, buf, 2);
}

static uint16_t bos_read(tile_t* tile)
{
    uint8_t buf[2] = {0, 0};
    tile->hal->i2c_read(tile->hal->handle, tile->id, 0x00, buf, 2);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static void bos_set_return_reg(tile_t* tile, uint8_t reg)
{
    drive_p_state_t *st = state_for(tile);
    bos_write(tile, BOS1921_REG_COMM,
              (uint16_t)reg | st->comm_persistent_bits);
}

/* Apply persistent CONFIG bits (GAINS/GAIND/RET) on top of the
 * mode-specific value. Default GAINS=1 is the chip default. */
static uint16_t cfg_with_persistent(tile_t* tile, uint16_t base)
{
    drive_p_state_t *st = state_for(tile);
    /* Mask out the persistent-bit slots first so callers can pass
     * a base that matches the existing per-mode constants without
     * worrying about colliding with the shadow values. */
    uint16_t mask = (uint16_t)(BOS_CONFIG_GAINS_BIT |
                               BOS_CONFIG_GAIND_BIT |
                               BOS_CONFIG_RET_BIT);
    return (uint16_t)((base & ~mask) | st->cfg_persistent_bits);
}

/* -------------------------------------------------------------- */
/* Public API                                                      */
/* -------------------------------------------------------------- */

uint8_t tile_drive_p_find(tiles_pal_t* hal, uint8_t instance)
{
    uint8_t id = resolve_id(instance);
    if (id == 0x00) return 0;
    return (hal->i2c_is_ready(hal->handle, id) == 0) ? 1 : 0;
}

void tile_drive_p_init_at(tiles_pal_t* hal, uint8_t addr, tile_t* tile,
                          const drive_p_cfg_t *cfg)
{
    (void)cfg;  /* reserved */
    tile->hal      = NULL;
    tile->id       = 0;
    tile->state    = TILE_STATE_NONE;
    tile->flags    = 0;
    tile->callback = NULL;
    tile->cb_ctx   = NULL;

    if (addr == 0x00) {
        TILE_ON_ERROR(tile, "init: invalid address");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    tile->hal = hal;
    tile->id  = addr;

    /* Verify device is on bus */
    if (hal->i2c_is_ready(hal->handle, addr) != 0) {
        TILE_ON_ERROR(tile, "init: device not found on bus");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    /* Wake the chip by writing to REFERENCE register */
    bos_write(tile, BOS1921_REG_REFERENCE, 0x0000);

    /* Software reset. Safe even for a reassigned instance (0x45/0x46): a
     * soft reset clears registers but does NOT revert the operating I2C
     * address — that only changes via a GPIO-gated SUP_RISE write, or a
     * power cycle. After reset the return register defaults to CHIP_ID. */
    bos_write(tile, BOS1921_REG_CONFIG, 0x0040);
    hal->delay_ms(1);

    /* Verify chip ID */
    uint16_t chip_id = bos_read(tile);
    if ((chip_id & 0x0FFF) != BOS1921_CHIP_ID_DEFAULT) {
        TILE_ON_ERROR(tile, "init: unexpected chip ID");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    /* Initialize shadow state to chip defaults for the bits we
     * track. GAINS=1 (fine) is the BOS1921 reset default; GAIND=0
     * (±95 V), RET=0 (retain), TOUT=0 (no auto-sleep), UPI=0. */
    drive_p_state_t *st = state_for(tile);
    st->cfg_persistent_bits  = BOS_CONFIG_GAINS_BIT;
    st->comm_persistent_bits = 0;
    st->parcap               = 0x043A;

    /* Configure for 260nF piezo, L1=10µH, Rsense=0.2Ω, VDD=3.7V LiPo.
     * The tuned supply-rise timing is 0x09E2; the I2C_ADDR nibble
     * (bits [15:12]) is derived from this chip's address so a re-addressed
     * chip keeps its address instead of being staged back to 0x44. For an
     * address of 0x44 this evaluates to the original 0x49E2. */
    bos_write(tile, BOS1921_REG_PARCAP,   st->parcap);
    bos_write(tile, BOS1921_REG_SUP_RISE,
              (uint16_t)(((uint16_t)(addr & 0x0Fu) << BOS_SUP_RISE_I2C_ADDR_POS) | 0x09E2u));

    tile->state = TILE_STATE_READY;
}

/* Instance-indexed init: resolve the instance to its mapped address
 * (0→0x44, 1→0x45, 2→0x46) and defer to the address-explicit path. */
void tile_drive_p_init(tiles_pal_t* hal, uint8_t instance, tile_t* tile,
                       const drive_p_cfg_t *cfg)
{
    tile_drive_p_init_at(hal, resolve_id(instance), tile, cfg);
}

uint8_t tile_drive_p_reassign_address(tiles_pal_t* hal, uint8_t cur_addr,
                                      uint8_t new_addr)
{
    /* Only the low nibble is programmable: address is 0b100_XXXX. */
    if ((new_addr & 0x70u) != 0x40u) return 0;

    uint8_t buf[2];

    /* 1. Wake any chip still at cur_addr (a dummy REFERENCE write) so the
     *    readdress takes effect reliably. */
    buf[0] = 0x00; buf[1] = 0x00;
    hal->i2c_write(hal->handle, cur_addr, BOS1921_REG_REFERENCE, buf, 2);

    /* 2. COMM.GPIODIR = 1 (GPIO becomes the per-chip write gate). Also point
     *    RDADDR at CHIP_ID so the verify read below works without another
     *    COMM write. Written to cur_addr; every chip there sees it, but only
     *    the one whose GPIO the caller holds low will latch step 3. */
    uint16_t comm_gate = (uint16_t)BOS1921_REG_CHIP_ID | BOS_COMM_GPIODIR_BIT;
    buf[0] = (uint8_t)(comm_gate >> 8);
    buf[1] = (uint8_t)(comm_gate & 0xFF);
    hal->i2c_write(hal->handle, cur_addr, BOS1921_REG_COMM, buf, 2);

    /* 3. Write the new address into SUP_RISE.I2C_ADDR[3:0] (bits [15:12]),
     *    preserving the default supply-rise timing. Takes effect immediately
     *    on the GPIO-low chip. GPIODIR is left set (input); init's reset, or
     *    a later access, restores it — leaving it avoids the BOS GPIO fighting
     *    the Core pad during a multi-chip sequence. */
    uint16_t sup = (uint16_t)(((uint16_t)(new_addr & 0x0Fu) << BOS_SUP_RISE_I2C_ADDR_POS)
                              | BOS_SUP_RISE_TIMING_DEFAULT);
    buf[0] = (uint8_t)(sup >> 8);
    buf[1] = (uint8_t)(sup & 0xFF);
    hal->i2c_write(hal->handle, cur_addr, BOS1921_REG_SUP_RISE, buf, 2);

    /* 4. Settle (datasheet wants >= 1 µs; delay_ms(1) is the coarsest PAL
     *    primitive and is amply long). */
    hal->delay_ms(1);

    /* 5. Confirm the chip now answers at new_addr with the expected CHIP_ID
     *    (RDADDR already points there from step 2). */
    uint8_t rd[2] = {0, 0};
    if (hal->i2c_read(hal->handle, new_addr, 0x00, rd, 2) != 0) return 0;
    uint16_t id = ((uint16_t)rd[0] << 8) | rd[1];
    return ((id & 0x0FFFu) == BOS1921_CHIP_ID_DEFAULT) ? 1 : 0;
}

void tile_drive_p_reset(tile_t* tile)
{
    /* RST bit self-clears; chip returns to its power-on defaults
     * (CONFIG=0x1000, COMM=0x001E, PARCAP=0x003A). Mirror that in
     * the driver shadow so subsequent set_mode()/sleep() writes
     * are coherent until the caller re-runs init or the setters. */
    bos_write(tile, BOS1921_REG_CONFIG, 0x0040);
    drive_p_state_t *st = state_for(tile);
    st->cfg_persistent_bits  = BOS_CONFIG_GAINS_BIT;
    st->comm_persistent_bits = 0;
    st->parcap               = 0x003A;
    tile->state = TILE_STATE_NONE;
}

void tile_drive_p_set_mode(tile_t* tile, drive_p_mode_t mode)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_mode: not ready");
        return;
    }

    drive_p_state_t *st = state_for(tile);

    switch (mode) {
    default:
    case DRIVE_P_MODE_IDLE:
        bos_write(tile, BOS1921_REG_CONFIG, cfg_with_persistent(tile, 0x0000));
        bos_set_return_reg(tile, BOS1921_REG_IC_STATUS);
        break;

    case DRIVE_P_MODE_SENSE_FINE:
        /* SENSE_FINE explicitly selects 7.6 mV LSB — set GAINS=1 in
         * the shadow so subsequent set_sense_gain() calls have a
         * coherent starting point. */
        st->cfg_persistent_bits |= BOS_CONFIG_GAINS_BIT;
        bos_write(tile, BOS1921_REG_CONFIG, cfg_with_persistent(tile, 0x0010));
        bos_write(tile, BOS1921_REG_REFERENCE, 0x0000);
        tile->hal->delay_ms(5);
        bos_write(tile, BOS1921_REG_CONFIG, cfg_with_persistent(tile, 0x0000));
        bos_write(tile, BOS1921_REG_CONFIG, cfg_with_persistent(tile, 0x3010));
        bos_set_return_reg(tile, BOS1921_REG_SENSE_VAL);
        break;

    case DRIVE_P_MODE_SENSE_COARSE:
        /* SENSE_COARSE explicitly selects 54.5 mV LSB. */
        st->cfg_persistent_bits &= (uint16_t)~BOS_CONFIG_GAINS_BIT;
        bos_write(tile, BOS1921_REG_CONFIG, cfg_with_persistent(tile, 0x0010));
        bos_write(tile, BOS1921_REG_REFERENCE, 0x0000);
        tile->hal->delay_ms(5);
        bos_write(tile, BOS1921_REG_CONFIG, cfg_with_persistent(tile, 0x0000));
        bos_write(tile, BOS1921_REG_CONFIG, cfg_with_persistent(tile, 0x2010));
        bos_set_return_reg(tile, BOS1921_REG_SENSE_VAL);
        break;

    case DRIVE_P_MODE_PLAY_DIRECT:
        bos_write(tile, BOS1921_REG_CONFIG, cfg_with_persistent(tile, 0x0010));
        bos_set_return_reg(tile, BOS1921_REG_IC_STATUS);
        break;

    case DRIVE_P_MODE_PLAY_FIFO:
        bos_write(tile, BOS1921_REG_CONFIG, cfg_with_persistent(tile, 0x0217));
        bos_set_return_reg(tile, BOS1921_REG_IC_STATUS);
        break;

    case DRIVE_P_MODE_PLAY_RAM_SYNTH:
        bos_write(tile, BOS1921_REG_CONFIG, cfg_with_persistent(tile, 0x0610));
        bos_set_return_reg(tile, BOS1921_REG_IC_STATUS);
        break;
    }
}

uint16_t tile_drive_p_read(tile_t* tile)
{
    return bos_read(tile);
}

int16_t tile_drive_p_read_sense(tile_t* tile)
{
    bos_set_return_reg(tile, BOS1921_REG_SENSE_VAL);
    uint16_t raw = bos_read(tile);
    raw &= 0x0FFF;
    if (raw & 0x0800) {
        raw |= 0xF000;
    }
    return (int16_t)raw;
}

uint16_t tile_drive_p_read_status(tile_t* tile)
{
    bos_set_return_reg(tile, BOS1921_REG_IC_STATUS);
    return bos_read(tile);
}

void tile_drive_p_write_fifo(tile_t* tile, int16_t sample)
{
    bos_write(tile, BOS1921_REG_REFERENCE, (uint16_t)sample);
}

void tile_drive_p_write_reg(tile_t* tile, uint8_t reg, uint16_t value)
{
    bos_write(tile, reg, value);
}

void tile_drive_p_wfs_write(tile_t* tile, const uint16_t* words, uint16_t count)
{
    uint8_t buf[16];
    if (count > 8) count = 8;
    for (uint16_t i = 0; i < count; i++) {
        buf[i * 2]     = (uint8_t)(words[i] >> 8);
        buf[i * 2 + 1] = (uint8_t)(words[i] & 0xFF);
    }
    tile->hal->i2c_write(tile->hal->handle, tile->id,
                         BOS1921_REG_REFERENCE, buf, count * 2);
}

void tile_drive_p_sleep(tile_t* tile)
{
    /* DS=1 selects SLEEP. RET (bit 8) is folded in via the
     * persistent shadow so set_sleep_retention() takes effect. */
    bos_write(tile, BOS1921_REG_CONFIG, cfg_with_persistent(tile, 0x0008));
    tile->state = TILE_STATE_SLEEPING;
}

uint8_t tile_drive_p_check_and_recover(tile_t* tile, drive_p_mode_t restore_mode)
{
    bos_set_return_reg(tile, BOS1921_REG_IC_STATUS);
    uint16_t status = bos_read(tile);

    uint8_t needs_recovery = 0;

    if ((status & BOS_STATUS_STATE_MASK) == BOS_STATUS_STATE_ERROR) {
        needs_recovery = 1;
    }
    if (status & BOS_STATUS_FAULT_MASK) {
        needs_recovery = 1;
    }

    if (needs_recovery) {
        tile_drive_p_reset(tile);
        tile->hal->delay_ms(1);
        tile->state = TILE_STATE_READY;  /* restore before set_mode */
        tile_drive_p_set_mode(tile, restore_mode);
    }

    return needs_recovery;
}

/* -------------------------------------------------------------- */
/* CONFIG bit setters                                              */
/* -------------------------------------------------------------- */

void tile_drive_p_set_output_range(tile_t* tile, drive_p_output_range_t range)
{
    drive_p_state_t *st = state_for(tile);
    if (range == DRIVE_P_OUTPUT_LOW_V) {
        st->cfg_persistent_bits |= BOS_CONFIG_GAIND_BIT;
    } else {
        st->cfg_persistent_bits &= (uint16_t)~BOS_CONFIG_GAIND_BIT;
    }
    /* Apply immediately if the device is already configured (IDLE-style
     * write). Caller is expected to be in IDLE; if a play mode is active
     * the existing CONFIG.OE bit will be cleared by this write — that's
     * intentional and matches the datasheet's "set OE=0 before changing
     * gain" guidance (§7.5). */
    if (tile->state == TILE_STATE_READY) {
        bos_write(tile, BOS1921_REG_CONFIG, cfg_with_persistent(tile, 0x0000));
    }
}

void tile_drive_p_set_sense_gain(tile_t* tile, drive_p_sense_gain_t gain)
{
    drive_p_state_t *st = state_for(tile);
    if (gain == DRIVE_P_SENSE_FINE_GAIN) {
        st->cfg_persistent_bits |= BOS_CONFIG_GAINS_BIT;
    } else {
        st->cfg_persistent_bits &= (uint16_t)~BOS_CONFIG_GAINS_BIT;
    }
    if (tile->state == TILE_STATE_READY) {
        bos_write(tile, BOS1921_REG_CONFIG, cfg_with_persistent(tile, 0x0000));
    }
}

void tile_drive_p_set_sleep_retention(tile_t* tile, uint8_t retain)
{
    drive_p_state_t *st = state_for(tile);
    /* RET=0 retains, RET=1 clears (datasheet §6.10.6). */
    if (retain) {
        st->cfg_persistent_bits &= (uint16_t)~BOS_CONFIG_RET_BIT;
    } else {
        st->cfg_persistent_bits |= BOS_CONFIG_RET_BIT;
    }
    /* Bit takes effect on the next CONFIG write that includes DS=1.
     * No need to apply immediately. */
}

/* -------------------------------------------------------------- */
/* COMM bit setters                                                */
/* -------------------------------------------------------------- */

void tile_drive_p_set_auto_sleep(tile_t* tile, uint8_t enabled)
{
    drive_p_state_t *st = state_for(tile);
    if (enabled) {
        st->comm_persistent_bits |= BOS_COMM_TOUT_BIT;
    } else {
        st->comm_persistent_bits &= (uint16_t)~BOS_COMM_TOUT_BIT;
    }
    /* Re-issue the COMM write so the new TOUT bit lands. Use the
     * current return-register selection (CHIP_ID=0x1E is the reset
     * default; we keep the existing RDADDR by reading what we last
     * configured). The driver doesn't track RDADDR explicitly, but
     * write-via-set_return_reg with IC_STATUS is the pre-play
     * default — safe choice for live updates. */
    if (tile->state == TILE_STATE_READY ||
        tile->state == TILE_STATE_SLEEPING) {
        bos_set_return_reg(tile, BOS1921_REG_IC_STATUS);
    }
}

/* -------------------------------------------------------------- */
/* PARCAP bit setters                                              */
/* -------------------------------------------------------------- */

void tile_drive_p_set_upi(tile_t* tile, uint8_t enabled)
{
    drive_p_state_t *st = state_for(tile);
    if (enabled) {
        st->parcap |= BOS_PARCAP_UPI_BIT;
    } else {
        st->parcap &= (uint16_t)~BOS_PARCAP_UPI_BIT;
    }
    bos_write(tile, BOS1921_REG_PARCAP, st->parcap);
}

/* ============================================================== */
/* Runtime — tier-2 idiomatic helpers                              */
/* ============================================================== */

#define DRIVE_P_SAMPLE_RATE_HZ  8000  /* FIFO mode default per chip */

/** Quarter-wave Q12 sine LUT — sin(π/2 × i/64) × 2047 for i=0..63.
 *  Used by sine_q12() to synthesise samples without pulling in libm. */
static const int16_t DRIVE_P_QSINE[64] = {
        0,    50,   100,   151,   201,   251,   300,   350,
      399,   449,   497,   546,   594,   642,   690,   737,
      783,   830,   875,   920,   965,  1009,  1052,  1095,
     1137,  1179,  1219,  1259,  1299,  1337,  1375,  1411,
     1447,  1483,  1517,  1550,  1582,  1614,  1644,  1674,
     1702,  1729,  1756,  1781,  1805,  1828,  1850,  1871,
     1891,  1910,  1927,  1944,  1959,  1973,  1986,  1997,
     2008,  2017,  2025,  2032,  2037,  2041,  2045,  2046,
};

/** Q12 sine via 64-entry quarter LUT + symmetry. Phase is 16-bit
 *  (0..65535 = 0..2π). Output range −2046..+2046. */
static int16_t sine_q12(uint16_t phase)
{
    uint8_t  q   = (uint8_t)((phase >> 14) & 0x03);
    uint8_t  idx = (uint8_t)((phase >> 8) & 0x3F);
    int16_t  v;
    switch (q) {
        case 0: v = DRIVE_P_QSINE[idx];        break;
        case 1: v = DRIVE_P_QSINE[63 - idx];   break;
        case 2: v = (int16_t)-DRIVE_P_QSINE[idx];      break;
        default: v = (int16_t)-DRIVE_P_QSINE[63 - idx]; break;
    }
    return v;
}

/** Scale a Q12 sample (-2046..+2046) by intensity_pct (0..100) onto
 *  the rated output: 100 % peaks at BOS1921_REFERENCE_MAX (1743 = ±95 V,
 *  or ±13.28 V in the low range; §6.10.1). A 12-bit full-scale 2047
 *  would ask for ±111.6 V, beyond the device's rated output. */
static int16_t scale_intensity(int16_t sample, uint8_t intensity_pct)
{
    if (intensity_pct > 100) intensity_pct = 100;
    return (int16_t)(((int32_t)sample * intensity_pct * BOS1921_REFERENCE_MAX)
                     / (100 * 2047));
}

/** Clamp a caller sample to ±BOS1921_REFERENCE_MAX (±95 V rated output).
 *  Caller buffers from `play_samples` may overshoot. */
static int16_t clamp_ref(int16_t s)
{
    if (s >  BOS1921_REFERENCE_MAX) return  BOS1921_REFERENCE_MAX;
    if (s < -BOS1921_REFERENCE_MAX) return -BOS1921_REFERENCE_MAX;
    return s;
}

void tile_drive_p_play_click(tile_t* tile, uint8_t intensity_pct)
{
    /* Half-sine over 16 samples ≈ 2 ms at 8 ksps — sharp tactile
     * click, well above the piezo's mechanical resonance. */
    tile_drive_p_set_mode(tile, DRIVE_P_MODE_PLAY_FIFO);
    for (uint8_t i = 0; i < 16; i++) {
        int16_t s = sine_q12((uint16_t)(i * 2048));  /* 0..π over 16 samples */
        s = scale_intensity(s, intensity_pct);
        tile_drive_p_write_fifo(tile, s);
    }
}

void tile_drive_p_play_sine(tile_t* tile, uint16_t freq_hz,
                            uint8_t intensity_pct, uint16_t ms)
{
    if (freq_hz == 0 || ms == 0) return;

    tile_drive_p_set_mode(tile, DRIVE_P_MODE_PLAY_FIFO);

    /* Phase delta per sample = freq × 65536 / sample_rate, Q16. */
    uint32_t phase = 0;
    uint32_t step  = ((uint32_t)freq_hz * 65536u) / DRIVE_P_SAMPLE_RATE_HZ;
    /* Total samples = ms × 8 (samples per ms at 8 ksps). */
    uint32_t total = (uint32_t)ms * (DRIVE_P_SAMPLE_RATE_HZ / 1000);

    /* 1024-deep FIFO — refill every 64 samples (8 ms) so the chip
     * never starves. The chip drains at the sample rate; we wait
     * 8 ms between refill chunks to let the FIFO drain by the same
     * amount we're about to push. */
    const uint32_t CHUNK = 64;
    while (total > 0) {
        uint32_t n = (total < CHUNK) ? total : CHUNK;
        for (uint32_t i = 0; i < n; i++) {
            int16_t s = sine_q12((uint16_t)(phase >> 0));
            s = scale_intensity(s, intensity_pct);
            tile_drive_p_write_fifo(tile, s);
            phase = (phase + step) & 0xFFFF;
        }
        total -= n;
        if (total > 0) tile->hal->delay_ms(8);
    }
}

void tile_drive_p_play_buzz(tile_t* tile, uint8_t intensity_pct, uint16_t ms)
{
    /* 150 Hz — typical small-form-factor piezo resonance band. */
    tile_drive_p_play_sine(tile, 150, intensity_pct, ms);
}

void tile_drive_p_play_pulse_train(tile_t* tile, uint8_t intensity_pct,
                                   uint8_t count, uint16_t gap_ms)
{
    for (uint8_t i = 0; i < count; i++) {
        tile_drive_p_play_click(tile, intensity_pct);
        if (i + 1 < count && gap_ms > 0) {
            tile->hal->delay_ms(gap_ms);
        }
    }
}

uint8_t tile_drive_p_is_touched(tile_t* tile, uint16_t threshold_mv)
{
    /* Switch into fine sense mode and read one sample. The chip's
     * sense channel returns a signed 12-bit value at 7.6 mV/LSB
     * (fine mode); convert to mV and compare against the absolute
     * threshold. */
    tile_drive_p_set_mode(tile, DRIVE_P_MODE_SENSE_FINE);
    int16_t raw = tile_drive_p_read_sense(tile);
    int32_t mv  = ((int32_t)raw * 76) / 10;  /* 7.6 mV/LSB → integer scaling */
    if (mv < 0) mv = -mv;
    return (mv >= (int32_t)threshold_mv) ? 1 : 0;
}

uint8_t tile_drive_p_play_on_touch(tile_t* tile, uint8_t intensity_pct,
                                   uint16_t threshold_mv,
                                   uint32_t timeout_ms)
{
    /* Poll sense at 1 ms cadence. is_touched() leaves the chip in
     * sense mode; play_click() switches to FIFO mode. */
    for (uint32_t elapsed = 0; elapsed < timeout_ms; elapsed++) {
        if (tile_drive_p_is_touched(tile, threshold_mv)) {
            tile_drive_p_play_click(tile, intensity_pct);
            return 1;
        }
        tile->hal->delay_ms(1);
    }
    return 0;
}

void tile_drive_p_play_samples(tile_t* tile, const int16_t* samples,
                               uint16_t count)
{
    if (!samples || count == 0) return;

    tile_drive_p_set_mode(tile, DRIVE_P_MODE_PLAY_FIFO);

    /* Refill in chunks to avoid overrunning the 1024-deep FIFO. */
    const uint16_t CHUNK = 64;
    uint16_t i = 0;
    while (i < count) {
        uint16_t n = ((count - i) < CHUNK) ? (uint16_t)(count - i) : CHUNK;
        for (uint16_t j = 0; j < n; j++) {
            tile_drive_p_write_fifo(tile, clamp_ref(samples[i + j]));
        }
        i += n;
        if (i < count) tile->hal->delay_ms(8);
    }
}

void tile_drive_p_read_sense_samples(tile_t* tile, int16_t* buf,
                                     uint16_t count)
{
    if (!buf || count == 0) return;

    tile_drive_p_set_mode(tile, DRIVE_P_MODE_SENSE_FINE);
    for (uint16_t i = 0; i < count; i++) {
        buf[i] = tile_drive_p_read_sense(tile);
    }
}
