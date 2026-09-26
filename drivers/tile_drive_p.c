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
 * - cfg_mode: the mode part of the last CONFIG write (PLAY_MODE / OE /
 *   SENSE / DS / PLAY_SRATE), so the safe reset can clear OE without
 *   changing the play mode, and knows whether REFERENCE is a sample.
 */
typedef struct {
    uint16_t cfg_persistent_bits;
    uint16_t comm_persistent_bits;
    uint16_t parcap;
    uint16_t cfg_mode;
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

/* Write CONFIG = mode bits + persistent GAINS/GAIND/RET, and remember the
 * mode bits. */
static void bos_config(tile_t* tile, uint16_t base)
{
    state_for(tile)->cfg_mode = base;
    bos_write(tile, BOS1921_REG_CONFIG, cfg_with_persistent(tile, base));
}

/* SUP_RISE as init writes it: tuned supply-rise timing 0x09E2 plus this
 * chip's I2C_ADDR nibble (bits [15:12]). The nibble only latches with
 * COMM.GPIODIR = 1 and GPIO low (§6.10.8), and it equals the current
 * address anyway, so rewriting it never moves the chip. */
static uint16_t sup_rise_value(tile_t* tile)
{
    return (uint16_t)(((uint16_t)(tile->id & 0x0Fu) << BOS_SUP_RISE_I2C_ADDR_POS)
                      | 0x09E2u);
}

/* Rewrite every register the driver configures from the shadow: CONFIG
 * (IDLE: OE=0, DS=0, plus GAINS/GAIND/RET), PARCAP (tuned value + CCM +
 * UPI), SUP_RISE, COMM (TOUT). Then read CHIP_ID back and leave RDADDR on
 * IC_STATUS. Used after a no-retention SLEEP (wake) and after a software
 * reset (check_and_recover), both of which return registers to defaults.
 * Returns 1 if CHIP_ID matched. */
static uint8_t bos_apply_config(tile_t* tile)
{
    drive_p_state_t *st = state_for(tile);
    bos_config(tile, 0x0000);
    bos_write(tile, BOS1921_REG_PARCAP,   st->parcap);
    bos_write(tile, BOS1921_REG_SUP_RISE, sup_rise_value(tile));
    bos_set_return_reg(tile, BOS1921_REG_CHIP_ID);
    uint16_t chip_id = bos_read(tile);
    bos_set_return_reg(tile, BOS1921_REG_IC_STATUS);
    return ((chip_id & 0x0FFF) == BOS1921_CHIP_ID_DEFAULT) ? 1 : 0;
}

/* Longest wait for the chip to leave RUN before a reset: with OE=0 a FIFO
 * waveform keeps playing until the FIFO is empty (§6.6), and a full
 * 1024-sample FIFO is 128 ms at 8 ksps. */
#define DRIVE_P_STOP_MS  150u

/* Software reset in the datasheet's order (§6.2.8): output to 0 and OE=0,
 * wait for IC_STATUS.STATE to leave RUN (IDLE, or ERROR on a faulted chip,
 * which does not go IDLE), then RST. REFERENCE must be 0 at RST in Direct /
 * FIFO mode (§6.10.6 RST); in the RAM modes REFERENCE takes WFS commands, so
 * it is not written there. RST self-clears; registers return to defaults,
 * except the I2C address (§6.10.8). Bounded by DRIVE_P_STOP_MS. */
static void bos_safe_reset(tile_t* tile)
{
    drive_p_state_t *st = state_for(tile);
    uint16_t play_mode = (uint16_t)(st->cfg_mode & 0x0600u);   /* PLAY_MODE[1:0] */

    if (play_mode == 0x0000u || play_mode == 0x0200u) {        /* Direct / FIFO */
        bos_write(tile, BOS1921_REG_REFERENCE, 0x0000);
    }
    bos_config(tile, (uint16_t)(st->cfg_mode & ~(BOS_CONFIG_OE_BIT | BOS_CONFIG_DS_BIT)));

    bos_set_return_reg(tile, BOS1921_REG_IC_STATUS);
    for (uint16_t ms = 0; ms < DRIVE_P_STOP_MS; ms++) {
        uint16_t state = (uint16_t)(bos_read(tile) & BOS_STATUS_STATE_MASK);
        if (state == BOS_STATUS_STATE_IDLE || state == BOS_STATUS_STATE_ERROR) break;
        tile->hal->delay_ms(1);
    }

    bos_write(tile, BOS1921_REG_CONFIG, BOS_CONFIG_RST_BIT);
    tile->hal->delay_ms(1);
    st->cfg_mode = 0x0000;   /* chip default CONFIG 0x1000: Direct, OE=0 */
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
    st->cfg_mode             = 0x0000;

    /* Configure for 260nF piezo, L1=10µH, Rsense=0.2Ω, VDD=3.7V LiPo.
     * The tuned supply-rise timing is 0x09E2; the I2C_ADDR nibble
     * (bits [15:12]) is derived from this chip's address so a re-addressed
     * chip keeps its address instead of being staged back to 0x44. For an
     * address of 0x44 this evaluates to the original 0x49E2. */
    bos_write(tile, BOS1921_REG_PARCAP,   st->parcap);
    bos_write(tile, BOS1921_REG_SUP_RISE, sup_rise_value(tile));

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
    /* Safe order (§6.2.8): output 0, OE=0, wait for IDLE, then RST. RST
     * self-clears; chip returns to its power-on defaults (CONFIG=0x1000,
     * COMM=0x001E, PARCAP=0x003A). Mirror that in the driver shadow so
     * subsequent set_mode()/sleep() writes are coherent until the caller
     * re-runs init or the setters. */
    if (tile->state == TILE_STATE_SLEEPING) {
        /* The first write would only wake the chip (§6.2.6). */
        bos_write(tile, BOS1921_REG_REFERENCE, 0x0000);
        tile->hal->delay_ms(1);
    }
    bos_safe_reset(tile);
    drive_p_state_t *st = state_for(tile);
    st->cfg_persistent_bits  = BOS_CONFIG_GAINS_BIT;
    st->comm_persistent_bits = 0;
    st->parcap               = 0x003A;
    tile->state = TILE_STATE_NONE;
}

void tile_drive_p_set_mode(tile_t* tile, drive_p_mode_t mode)
{
    /* A sleeping chip discards the first write it sees (§6.2.6), so wake it
     * properly first instead of refusing. */
    if (tile->state == TILE_STATE_SLEEPING) {
        (void)tile_drive_p_wake(tile);
    }
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_mode: not ready");
        return;
    }

    drive_p_state_t *st = state_for(tile);

    switch (mode) {
    default:
    case DRIVE_P_MODE_IDLE:
        bos_config(tile, 0x0000);
        bos_set_return_reg(tile, BOS1921_REG_IC_STATUS);
        break;

    case DRIVE_P_MODE_SENSE_FINE:
        /* SENSE_FINE explicitly selects 7.6 mV LSB — set GAINS=1 in
         * the shadow so subsequent set_sense_gain() calls have a
         * coherent starting point. */
        st->cfg_persistent_bits |= BOS_CONFIG_GAINS_BIT;
        bos_config(tile, 0x0010);
        bos_write(tile, BOS1921_REG_REFERENCE, 0x0000);
        tile->hal->delay_ms(5);
        bos_config(tile, 0x0000);
        bos_config(tile, 0x3010);
        bos_set_return_reg(tile, BOS1921_REG_SENSE_VAL);
        break;

    case DRIVE_P_MODE_SENSE_COARSE:
        /* SENSE_COARSE explicitly selects 54.5 mV LSB. */
        st->cfg_persistent_bits &= (uint16_t)~BOS_CONFIG_GAINS_BIT;
        bos_config(tile, 0x0010);
        bos_write(tile, BOS1921_REG_REFERENCE, 0x0000);
        tile->hal->delay_ms(5);
        bos_config(tile, 0x0000);
        bos_config(tile, 0x2010);
        bos_set_return_reg(tile, BOS1921_REG_SENSE_VAL);
        break;

    case DRIVE_P_MODE_PLAY_DIRECT:
        bos_config(tile, 0x0010);
        bos_set_return_reg(tile, BOS1921_REG_IC_STATUS);
        break;

    case DRIVE_P_MODE_PLAY_FIFO:
        bos_config(tile, 0x0217);
        bos_set_return_reg(tile, BOS1921_REG_IC_STATUS);
        break;

    case DRIVE_P_MODE_PLAY_RAM_SYNTH:
        bos_config(tile, 0x0610);
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
    /* DS=1 selects SLEEP while OE=0 (§6.2.6, §6.10.6). RET (bit 8) is
     * folded in via the persistent shadow so set_sleep_retention() takes
     * effect. */
    bos_config(tile, BOS_CONFIG_DS_BIT);
    tile->state = TILE_STATE_SLEEPING;
}

uint8_t tile_drive_p_wake(tile_t* tile)
{
    /* Only a tile that init() brought up (now READY, or SLEEPING after
     * sleep()) can be woken; NONE / ERROR need init() or recovery. */
    if (tile->hal == NULL ||
        (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_SLEEPING)) {
        TILE_ON_ERROR(tile, "wake: tile not initialized");
        return 0;
    }

    /* 1. Dummy write: any bus traffic wakes the chip and its data is
     *    ignored (§6.2.6; start-up sequence §7.4.1 step 3). On an awake chip
     *    in FIFO mode this queues one 0 V sample, which is harmless. */
    bos_write(tile, BOS1921_REG_REFERENCE, 0x0000);

    /* 2. SLEEP -> IDLE takes 50 µs (§7.4.1 step 4); 1 ms is the PAL's
     *    coarsest delay and amply long. */
    tile->hal->delay_ms(1);

    /* 3. Program the registers (§7.4.1 step 5). All of them, from the
     *    shadow: with RET=1 the chip dropped them in SLEEP, with RET=0 the
     *    rewrite is a no-op. CONFIG first so DS=0 (stay in IDLE, not SLEEP,
     *    while OE=0) and OE=0 (output off). Then confirm CHIP_ID. */
    if (!bos_apply_config(tile)) {
        TILE_ON_ERROR(tile, "wake: unexpected chip ID");
        tile->state = TILE_STATE_ERROR;
        return 0;
    }
    tile->state = TILE_STATE_READY;
    return 1;
}

uint8_t tile_drive_p_check_and_recover(tile_t* tile, drive_p_mode_t restore_mode)
{
    /* A sleeping chip would discard the COMM write below (§6.2.6). */
    if (tile->state == TILE_STATE_SLEEPING) {
        (void)tile_drive_p_wake(tile);
    }
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
        /* Reset in the §6.2.8 order, then put back everything init and the
         * setters configured (tuned PARCAP + CCM + UPI, GAINS/GAIND/RET,
         * SUP_RISE, COMM.TOUT): the shadow is kept, not reset to chip
         * defaults. Before v3.5 recovery left PARCAP at 0x003A — UPI off. */
        bos_safe_reset(tile);
        if (!bos_apply_config(tile)) {
            TILE_ON_ERROR(tile, "recover: unexpected chip ID");
            tile->state = TILE_STATE_ERROR;
            return needs_recovery;
        }
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
        bos_config(tile, 0x0000);
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
        bos_config(tile, 0x0000);
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
    /* Re-issue the COMM write so the new TOUT bit lands. The driver
     * doesn't track RDADDR explicitly; IC_STATUS is the pre-play default
     * and a safe choice for live updates. Not while SLEEPING: the write
     * would only wake the chip and be discarded (§6.2.6) — wake() applies
     * the shadow instead. */
    if (tile->state == TILE_STATE_READY) {
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
    /* While SLEEPING the write would only wake the chip and be discarded
     * (§6.2.6); wake() rewrites PARCAP from the shadow. */
    if (tile->state != TILE_STATE_SLEEPING) {
        bos_write(tile, BOS1921_REG_PARCAP, st->parcap);
    }
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

/* -------------------------------------------------------------- */
/* FIFO streaming                                                  */
/* -------------------------------------------------------------- */

/*
 * FIFO mode plays REFERENCE samples at 8 ksps from a 1024-deep FIFO
 * (§6.6). The host keeps it fed per the datasheet's typical sequence
 * (§6.6.2): read FIFO_STATE (RDADDR = 0x11), then write as many samples as
 * FIFO_SPACE allows. Writes are bursts: a write to register 0x00 sends every
 * following 2-byte word to REFERENCE, whatever COMM.STR says (§6.3.1.1,
 * Table 43 STR), and the FIFO accepts packed words (§6.6).
 *
 * Burst size: 32 samples = 1 register byte + 64 data bytes, well under the
 * STM32 I2C NBYTES limit of 255 per transfer. At 400 kHz a burst takes
 * ~1.5 ms and carries 4 ms of audio.
 *
 * Blocking: the call returns once the last sample is queued. While the FIFO
 * is full it waits in 1 ms steps (one FIFO_STATE read each). PLAY_SRATE 0x7
 * (8 ksps) is the slowest rate, so a FIFO that frees no space for 20 ms is
 * not draining at all (chip asleep, output off, error) and the stream stops
 * instead of spinning.
 */
#define DRIVE_P_BURST          32u  /* samples per REFERENCE burst write */
#define DRIVE_P_STALL_MS       20u  /* give up after this long with no drain */
#define DRIVE_P_ZERO_LEAD       2u  /* 0 V samples ahead of tier-2 waveforms */

typedef int16_t (*stream_fn_t)(const void *ctx, uint32_t i);

/* Free FIFO locations from a FIFO_STATE word (§6.10.14): FIFO_SPACE = 0
 * means 1024 free if EMPTY, else full. */
static uint16_t fifo_space(uint16_t fifo_state)
{
    uint16_t space = (uint16_t)(fifo_state & BOS_FIFO_STATE_SPACE_MASK);
    if (space == 0 && (fifo_state & BOS_FIFO_STATE_EMPTY_BIT)) {
        space = BOS1921_FIFO_DEPTH;
    }
    return space;
}

/* Stream samples fn(ctx, 0..total-1) into the FIFO. Expects FIFO mode with
 * OE set (set_mode(PLAY_FIFO)). Leaves RDADDR on IC_STATUS, as set_mode()
 * does. Returns 1 if every sample was queued. Does not change tile->state:
 * a chip fault stays visible through read_status() / check_and_recover(). */
static uint8_t stream_fifo(tile_t* tile, uint32_t total,
                           stream_fn_t fn, const void *ctx)
{
    uint8_t  buf[DRIVE_P_BURST * 2u];
    uint32_t sent       = 0;
    uint16_t last_space = 0;
    uint16_t stall_ms   = 0;
    uint8_t  ok         = 1;

    bos_set_return_reg(tile, BOS1921_REG_FIFO_STATE);

    while (sent < total) {
        uint16_t fs = bos_read(tile);
        if (fs & BOS_FIFO_STATE_ERROR_BIT) {
            TILE_ON_ERROR(tile, "stream: chip in ERROR");
            ok = 0;
            break;
        }

        uint16_t space = fifo_space(fs);
        uint32_t left  = total - sent;
        uint32_t want  = (left < DRIVE_P_BURST) ? left : DRIVE_P_BURST;

        if (space < want) {
            /* Wait for the chip to drain a burst's worth (4 ms at most). */
            if (space > last_space) {
                stall_ms = 0;
            } else if (++stall_ms > DRIVE_P_STALL_MS) {
                TILE_ON_ERROR(tile, "stream: FIFO not draining");
                ok = 0;
                break;
            }
            last_space = space;
            tile->hal->delay_ms(1);
            continue;
        }

        /* Fill the free space, one burst per I2C transaction. Space only
         * grows while we write, so the reading taken above stays safe. */
        uint32_t n = (space < left) ? space : left;
        while (n > 0) {
            uint16_t chunk = (uint16_t)((n < DRIVE_P_BURST) ? n : DRIVE_P_BURST);
            for (uint16_t k = 0; k < chunk; k++) {
                uint16_t v = (uint16_t)fn(ctx, sent + k);
                buf[2u * k]      = (uint8_t)(v >> 8);
                buf[2u * k + 1u] = (uint8_t)(v & 0xFF);
            }
            tile->hal->i2c_write(tile->hal->handle, tile->id,
                                 BOS1921_REG_REFERENCE, buf,
                                 (uint16_t)(chunk * 2u));
            sent += chunk;
            n    -= chunk;
        }
        stall_ms   = 0;
        last_space = 0;
    }

    bos_set_return_reg(tile, BOS1921_REG_IC_STATUS);
    return ok;
}

/* Click: DRIVE_P_ZERO_LEAD x 0 V (discharges residual piezo charge,
 * §6.2.18), then a half-sine over i = 0..16 (phase 0..π, 16 steps = 2 ms).
 * The last sample (i = 16) is exactly 0 V: the FIFO holds its last entry on
 * the output (§6.6), and before v3.5 the click stopped at i = 15, leaving
 * ~20 % of the peak parked on the piezo. */
#define DRIVE_P_CLICK_STEPS    16u
#define DRIVE_P_CLICK_SAMPLES  (DRIVE_P_ZERO_LEAD + DRIVE_P_CLICK_STEPS + 1u)

static int16_t click_sample(const void *ctx, uint32_t i)
{
    uint8_t pct = *(const uint8_t *)ctx;
    if (i < DRIVE_P_ZERO_LEAD) return 0;
    i -= DRIVE_P_ZERO_LEAD;
    if (i >= DRIVE_P_CLICK_STEPS) return 0;          /* phase π: 0 V */
    return scale_intensity(sine_q12((uint16_t)(i * 2048u)), pct);
}

/* Sine: DRIVE_P_ZERO_LEAD x 0 V, then `body` samples starting at phase 0
 * whose last DRIVE_P_SINE_FADE samples ramp linearly to exactly 0 V. */
#define DRIVE_P_SINE_FADE      16u  /* 2 ms release at 8 ksps */

typedef struct {
    uint32_t step;   /* Q16 phase increment per sample */
    uint32_t body;   /* samples of tone, fade included */
    uint8_t  pct;
} sine_src_t;

static int16_t sine_sample(const void *ctx, uint32_t i)
{
    const sine_src_t *src = (const sine_src_t *)ctx;
    if (i < DRIVE_P_ZERO_LEAD) return 0;
    i -= DRIVE_P_ZERO_LEAD;
    if (i >= src->body) return 0;
    /* Phase is i x step mod 2^16; unsigned wrap keeps the low 16 bits exact. */
    int32_t s = scale_intensity(sine_q12((uint16_t)(i * src->step)), src->pct);
    uint32_t left = src->body - 1u - i;              /* 0 on the last sample */
    if (left < DRIVE_P_SINE_FADE) {
        s = (s * (int32_t)left) / (int32_t)DRIVE_P_SINE_FADE;
    }
    return (int16_t)s;
}

static int16_t buffer_sample(const void *ctx, uint32_t i)
{
    return clamp_ref(((const int16_t *)ctx)[i]);
}

/* Stop sequence step 6 (§6.5.1 / §6.6.2): OE=0 once the waveform is done.
 * Written right after the last (0 V) sample is queued — no waiting: with
 * OE=0 the chip keeps playing until the FIFO is empty (§6.6), so it stops
 * after the drain, at 0 V, and drops to IDLE (530 µA) instead of holding
 * 0 V in RUN. FIFO mode and 8 ksps stay selected. The next set_mode()
 * turns OE back on (< 300 µs start-up, §6.2.7). */
static void fifo_output_off_after_drain(tile_t* tile)
{
    bos_config(tile, (uint16_t)(0x0217u & ~BOS_CONFIG_OE_BIT));
}

/* One click, output left on (pulse_train chains these). */
static uint8_t click_once(tile_t* tile, uint8_t intensity_pct)
{
    tile_drive_p_set_mode(tile, DRIVE_P_MODE_PLAY_FIFO);
    if (tile->state != TILE_STATE_READY) return 0;
    return stream_fifo(tile, DRIVE_P_CLICK_SAMPLES, click_sample, &intensity_pct);
}

void tile_drive_p_play_click(tile_t* tile, uint8_t intensity_pct)
{
    if (click_once(tile, intensity_pct)) fifo_output_off_after_drain(tile);
}

void tile_drive_p_play_sine(tile_t* tile, uint16_t freq_hz,
                            uint8_t intensity_pct, uint16_t ms)
{
    if (freq_hz == 0 || ms == 0) return;

    tile_drive_p_set_mode(tile, DRIVE_P_MODE_PLAY_FIFO);
    if (tile->state != TILE_STATE_READY) return;

    sine_src_t src;
    /* Phase delta per sample = freq x 65536 / sample_rate, Q16. */
    src.step = ((uint32_t)freq_hz * 65536u) / DRIVE_P_SAMPLE_RATE_HZ;
    /* Samples = ms x 8 at 8 ksps. */
    src.body = (uint32_t)ms * (DRIVE_P_SAMPLE_RATE_HZ / 1000u);
    src.pct  = intensity_pct;

    if (stream_fifo(tile, DRIVE_P_ZERO_LEAD + src.body, sine_sample, &src)) {
        fifo_output_off_after_drain(tile);
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
    /* Output stays on between clicks (no OE toggling mid-drain) and goes
     * off once, after the last click is queued. */
    uint8_t ok = 0;
    for (uint8_t i = 0; i < count; i++) {
        ok = click_once(tile, intensity_pct);
        if (!ok) break;
        if (i + 1 < count && gap_ms > 0) {
            tile->hal->delay_ms(gap_ms);
        }
    }
    if (ok) fifo_output_off_after_drain(tile);
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
    if (tile->state != TILE_STATE_READY) return;

    /* Played as given (clamped to the rated range): no 0 V lead-in or tail,
     * and the output stays on (OE=1), so back-to-back calls can stream one
     * long waveform in pieces without a stop/start between them. */
    (void)stream_fifo(tile, count, buffer_sample, samples);
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
