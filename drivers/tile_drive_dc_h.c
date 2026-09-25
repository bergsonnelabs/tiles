/**
 * @file   tile_drive_dc_h.c
 * @brief  H-bridge DC motor driver implementation (DRV8214).
 */

#include "tile_drive_dc_h.h"
#include <stddef.h>

/* -------------------------------------------------------------- */
/* Instance mapping                                                */
/* -------------------------------------------------------------- */

static const uint8_t id_table[] = {
    0x34,  /* instance 0 — A1=Hi-Z, A0=Hi-Z (default) */
    0x30,  /* instance 1 — A1=0,    A0=0    */
    0x31,  /* instance 2 — A1=0,    A0=Hi-Z */
    0x32,  /* instance 3 — A1=0,    A0=1    */
    0x33,  /* instance 4 — A1=Hi-Z, A0=0    */
    0x35,  /* instance 5 — A1=Hi-Z, A0=1    */
    0x36,  /* instance 6 — A1=1,    A0=0    */
    0x37,  /* instance 7 — A1=1,    A0=Hi-Z */
    0x38,  /* instance 8 — A1=1,    A0=1    */
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
 * The DRV8214's RC tuning registers (RC_CTRL3/4) and W_SCALE field
 * encode physically meaningful units, but the chip can't tell us
 * back the per-revolution ripple count we used to compute them.
 * The tier-2 helper set_speed_rpm() needs that value to convert
 * RPM → WSET_VSET cleanly, so we cache it from the init config.
 */
typedef struct {
    uint8_t ripples_per_rev;  /**< Mirrors drive_dc_h_cfg_t.ripples_per_rev. */
} drive_dc_h_state_t;

static drive_dc_h_state_t drv_state[ID_TABLE_LEN];

static drive_dc_h_state_t *state_for(tile_t *tile)
{
    for (uint8_t i = 0; i < ID_TABLE_LEN; i++) {
        if (id_table[i] == tile->id) return &drv_state[i];
    }
    return &drv_state[0];
}

/* -------------------------------------------------------------- */
/* Private helpers                                                 */
/* -------------------------------------------------------------- */

static void drv_write(tile_t* tile, uint8_t reg, uint8_t value)
{
    tile->hal->i2c_write(tile->hal->handle, tile->id, reg, &value, 1);
}

static uint8_t drv_read(tile_t* tile, uint8_t reg)
{
    uint8_t val = 0;
    tile->hal->i2c_read(tile->hal->handle, tile->id, reg, &val, 1);
    return val;
}

/* Read-modify-write: clear `mask` bits, OR in `bits`. */
static void drv_rmw(tile_t* tile, uint8_t reg, uint8_t mask, uint8_t bits)
{
    uint8_t v = drv_read(tile, reg);
    v = (uint8_t)((v & ~mask) | (bits & mask));
    drv_write(tile, reg, v);
}

/* Fields marked * in the register map (datasheet Table 8-29) are "writable
 * only when EN_OUT=0": CONFIG0 VSNS_SEL / VM_GAIN_SEL / DUTY_CTRL, CONFIG3
 * IMODE..TSD_MODE, CONFIG4 PMODE / I2C_BC, REG_CTRL0 REG_CTRL / PWM_FREQ.
 * With the outputs on, a write to them is silently dropped. Take EN_OUT down
 * around such a write and put it back: the bridge coasts for the few I²C
 * bytes. CLR_CNT / CLR_FLT (CONFIG0 bits 2 / 1) are write-1 actions, so they
 * are never written back from a read. */
static void drv_locked_rmw(tile_t* tile, uint8_t reg, uint8_t mask, uint8_t bits)
{
    uint8_t c0 = (uint8_t)(drv_read(tile, DRV8214_REG_CONFIG0) & ~0x06u);
    uint8_t was_on = c0 & 0x80u;
    if (was_on) drv_write(tile, DRV8214_REG_CONFIG0, (uint8_t)(c0 & ~0x80u));
    drv_rmw(tile, reg, mask, bits);
    if (was_on) {
        c0 = (uint8_t)(drv_read(tile, DRV8214_REG_CONFIG0) & ~0x06u);
        drv_write(tile, DRV8214_REG_CONFIG0, (uint8_t)(c0 | 0x80u));
    }
}

/* W_SCALE encoding in REG_CTRL0 [1:0] (datasheet Table 8-24 / 8-49):
 *   00 → 16, 01 → 32, 10 → 64, 11 → 128  (rad/s per LSB).
 * Ripple speed (rad/s) = SPEED × W_SCALE (Eq. 8); WSET_VSET is in the
 * same units in speed mode. Ripple speed in rad/s = ripple Hz × 2π
 * (Eq. 11), so shaft RPM = SPEED × W_SCALE × 60 / (2π × ripples/rev). */
static const uint16_t w_scale_lookup[4] = { 16, 32, 64, 128 };

/* 2π × 1000, for the integer rad/s ↔ RPM conversions. */
#define TWO_PI_X1000  6283u

/* CONFIG4 base: STALL_REP=1, CBC_REP=1, PMODE=1 (PWM), I2C_BC=1 */
#define CONFIG4_BASE  0x3C

/* CONFIG4 bit masks */
#define CONFIG4_PMODE_MASK   0x08
#define CONFIG4_I2C_BC_MASK  0x04
#define CONFIG4_INX_MASK     0x03   /* I2C_EN_IN1 + I2C_PH_IN2 */

/* CS_GAIN_SEL → maximum current in mA */
static const uint16_t cs_max_ma[] = {
    4000,  /* 000b */
    2000,  /* 001b */
    1000,  /* 010b */
    500,   /* 011b */
    250,   /* 100b */
    125,   /* 101b */
    250,   /* 110b (same as 100b, bit 1 don't care) */
    125,   /* 111b (same as 101b, bit 1 don't care) */
};

/* True when the bridge is currently driven by I²C registers (I2C_BC=1).
 * Used as the gate for forward/reverse/brake/coast — those are no-ops
 * in pad-control modes. */
static uint8_t bridge_is_i2c(tile_t* tile)
{
    return (drv_read(tile, DRV8214_REG_CONFIG4) & CONFIG4_I2C_BC_MASK) ? 1 : 0;
}

/* -------------------------------------------------------------- */
/* Public API                                                      */
/* -------------------------------------------------------------- */

uint8_t tile_drive_dc_h_find(tiles_pal_t* hal, uint8_t instance)
{
    uint8_t id = resolve_id(instance);
    if (id == 0x00) return 0;
    return (hal->i2c_is_ready(hal->handle, id) == 0) ? 1 : 0;
}

/* Program the ripple-counter calibration (INV_R, KMC) from motor
 * parameters. Shared by init and set_motor_params(). No-op when
 * motor_mohm == 0 — the chip keeps its power-on defaults. */
static void drv_apply_motor_tuning(tile_t* tile, uint16_t motor_mohm,
                                   uint8_t ripples_per_rev,
                                   uint16_t kv_uv_per_rpm)
{
    if (motor_mohm == 0) return;

    /* INV_R = INV_R_SCALE / R_motor (ohms)
     *       = INV_R_SCALE * 1000 / motor_mohm
     * Try scales from largest to smallest for best precision. */
    static const uint32_t inv_r_scales[]    = { 8192, 1024, 64, 2 };
    static const uint8_t  inv_r_scale_bits[] = { 3, 2, 1, 0 };
    uint8_t inv_r = 0;
    uint8_t inv_r_sb = 0;

    for (uint8_t i = 0; i < 4; i++) {
        uint32_t v = inv_r_scales[i] * 1000 / motor_mohm;
        if (v >= 1 && v <= 255) {
            inv_r = (uint8_t)v;
            inv_r_sb = inv_r_scale_bits[i];
            break;
        }
    }

    /* KMC = (Kv / N_R) * KMC_SCALE
     * Kv in V/(rad/s) = kv_uv_per_rpm / 104720 (approx)
     * Pre-computed multipliers: KMC_SCALE * 60 / (2*pi*1e6)
     *   scale 3 (196608): ×1878/1000
     *   scale 2 (98304):  ×939/1000
     *   scale 1 (12288):  ×117/1000
     *   scale 0 (6144):   ×59/1000                              */
    uint8_t kmc = 0;
    uint8_t kmc_sb = 0;

    if (kv_uv_per_rpm > 0) {
        static const uint16_t kmc_mults[]      = { 1878, 939, 117, 59 };
        static const uint8_t  kmc_scale_bits[] = { 3, 2, 1, 0 };

        for (uint8_t i = 0; i < 4; i++) {
            uint32_t num = (uint32_t)kv_uv_per_rpm * kmc_mults[i];
            uint32_t den = (uint32_t)1000 * ripples_per_rev;
            uint32_t v   = (num + den / 2) / den;  /* round */
            if (v >= 1 && v <= 255) {
                kmc = (uint8_t)v;
                kmc_sb = kmc_scale_bits[i];
                break;
            }
        }
    }

    /* RC_CTRL2: scaling factors
     * [7:6] INV_R_SCALE
     * [5:4] KMC_SCALE
     * [3:2] RC_THR_SCALE = 11 (×64, default)
     * [1:0] RC_THR[9:8]  = 11 (default)                        */
    drv_write(tile, DRV8214_REG_RC_CTRL2,
              (inv_r_sb << 6) | (kmc_sb << 4) | 0x0F);

    /* RC_CTRL3: INV_R */
    if (inv_r > 0) {
        drv_write(tile, DRV8214_REG_RC_CTRL3, inv_r);
    }

    /* RC_CTRL4: KMC */
    if (kmc > 0) {
        drv_write(tile, DRV8214_REG_RC_CTRL4, kmc);
    }
}

void tile_drive_dc_h_init(tiles_pal_t* hal, uint8_t instance, tile_t* tile,
                          const drive_dc_h_cfg_t *cfg)
{
    tile->hal      = NULL;
    tile->id       = 0;
    tile->state    = TILE_STATE_NONE;
    tile->flags    = 0;
    tile->callback = NULL;
    tile->cb_ctx   = NULL;

    uint8_t id = resolve_id(instance);
    if (id == 0x00) {
        TILE_ON_ERROR(tile, "init: invalid instance");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    tile->hal = hal;
    tile->id  = id;

    /* Verify device is on bus */
    if (hal->i2c_is_ready(hal->handle, id) != 0) {
        TILE_ON_ERROR(tile, "init: device not found on bus");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    /* Parse config, apply defaults */
    uint8_t  mode            = DRIVE_DC_H_MODE_VOLTAGE;
    uint8_t  vm_gain         = 1;    /* 0-3.92V range */
    uint8_t  cs_gain         = 0;    /* 4A max */
    uint8_t  target          = 0xFF; /* full scale */
    uint16_t motor_mohm      = 0;
    uint8_t  ripples_per_rev = 12;
    uint16_t kv_uv_per_rpm   = 0;

    if (cfg != NULL) {
        mode    = cfg->mode;
        vm_gain = cfg->vm_gain ? 1 : 0;
        cs_gain = cfg->cs_gain;
        if (cs_gain > 5) cs_gain = 5;
        target  = cfg->target;
        motor_mohm      = cfg->motor_mohm;
        ripples_per_rev = cfg->ripples_per_rev;
        if (ripples_per_rev == 0) ripples_per_rev = 12;
        kv_uv_per_rpm   = cfg->kv_uv_per_rpm;
    }

    /* Cache ripples_per_rev for set_speed_rpm() — the chip doesn't
     * carry this anywhere we can read back. */
    state_for(tile)->ripples_per_rev = ripples_per_rev;

    /* Outputs off first: the starred fields below are only writable while
     * EN_OUT = 0 (datasheet Table 8-29), and the chip keeps EN_OUT across an
     * MCU reset if it stays powered. EN_OUT goes back on at the end. */
    drv_write(tile, DRV8214_REG_CONFIG0, 0x00);

    /* ---- CONFIG4: bridge control mode ----
     * [7:6] RC_REP     = 00  (no ripple count on nFAULT)
     * [5]   STALL_REP  = 1   (report stall on nFAULT)
     * [4]   CBC_REP    = 1   (report current regulation on nFAULT)
     * [3]   PMODE      = mode-dependent
     * [2]   I2C_BC     = mode-dependent
     * [1]   I2C_EN_IN1 = 0   (start in coast)
     * [0]   I2C_PH_IN2 = 0                                    */
    {
        uint8_t cfg4;
        if (mode == DRIVE_DC_H_MODE_PAD_PHEN) {
            /* PMODE=0 (PH/EN), I2C_BC=0 (external pad control) */
            cfg4 = 0x30;
        } else if (mode == DRIVE_DC_H_MODE_PAD_IN1IN2) {
            /* PMODE=1 (PWM, supports independent half-bridge), I2C_BC=0 */
            cfg4 = 0x38;
        } else {
            cfg4 = CONFIG4_BASE;
        }
        drv_write(tile, DRV8214_REG_CONFIG4, cfg4);
    }

    /* ---- CONFIG0: faults, voltage range (EN_OUT still 0) ----
     * [7]   EN_OUT       = 0   (set last, below, once the locked fields
     *                           are written)
     * [6]   EN_OVP       = 1   (overvoltage protection on)
     * [5]   EN_STALL     = mode-dependent (off for pad control)
     * [4]   VSNS_SEL     = 0   (analog output filter)
     * [3]   VM_GAIN_SEL  = cfg (voltage range selection)
     * [2]   CLR_CNT      = 0
     * [1]   CLR_FLT      = 1   (clear any power-on faults)
     * [0]   DUTY_CTRL    = 0   (internal duty control)             */
    drv_write(tile, DRV8214_REG_CONFIG0,
              0x62 | (vm_gain << 3));

    /* ---- CONFIG3: stall/current regulation settings ----
     * [7:6] IMODE     = mode-dependent
     *                   01 for I2C modes (current reg during tINRUSH)
     *                   00 for PAD_* (no current reg — user controls PWM)
     * [5]   SMODE     = 1   (stall = indication only, outputs stay on)
     * [4]   INT_VREF  = 1   (use internal 500 mV stall reference)
     * [3]   TBLANK    = 0   (1.8 us blanking time)
     * [2]   TDEG      = 0   (2 us deglitch time)
     * [1]   OCP_MODE  = 1   (auto-retry on overcurrent)
     * [0]   TSD_MODE  = 1   (auto-retry on thermal shutdown)        */
    {
        uint8_t imode = (mode == DRIVE_DC_H_MODE_PAD_PHEN ||
                         mode == DRIVE_DC_H_MODE_PAD_IN1IN2) ? 0x00 : 0x40;
        drv_write(tile, DRV8214_REG_CONFIG3, 0x33 | imode);
    }

    /* ---- REG_CTRL0: regulation mode, soft-start, PWM freq ----
     * [7:6] RSVD      = 00
     * [5]   EN_SS     = 1   (soft-start/stop enabled)
     * [4:3] REG_CTRL  = mode-dependent
     * [2]   PWM_FREQ  = 1   (25 kHz)
     * [1:0] W_SCALE   = 11  (128)                                  */
    uint8_t reg_ctrl_bits;
    if (mode == DRIVE_DC_H_MODE_SPEED) {
        reg_ctrl_bits = 0x02;  /* 10b = speed regulation */
    } else if (mode == DRIVE_DC_H_MODE_PAD_PHEN ||
               mode == DRIVE_DC_H_MODE_PAD_IN1IN2) {
        reg_ctrl_bits = 0x00;  /* 00b = no regulation (external PWM) */
    } else {
        reg_ctrl_bits = 0x03;  /* 11b = voltage regulation (also for RIPPLE_COUNT) */
    }
    drv_write(tile, DRV8214_REG_CTRL0,
              0x20 | (reg_ctrl_bits << 3) | 0x07);

    /* ---- REG_CTRL1: target voltage or speed ---- */
    drv_write(tile, DRV8214_REG_CTRL1, target);

    /* ---- CONFIG1/CONFIG2: inrush time (TINRUSH) ----
     * Set to ~100 ms. Each LSB = 102.4 us.
     * 100 ms / 102.4 us ≈ 977 = 0x03D1                             */
    drv_write(tile, DRV8214_REG_CONFIG1, 0xD1);  /* TINRUSH[7:0]  */
    drv_write(tile, DRV8214_REG_CONFIG2, 0x03);  /* TINRUSH[15:8] */

    /* ---- RC_CTRL0: current gain, ripple counting ----
     * [7]   EN_RC        = mode-dependent (enable for speed reg)
     * [6]   DIS_EC       = 0   (error correction enabled)
     * [5]   RC_HIZ       = 0   (bridge stays on at threshold)
     * [4:3] FLT_GAIN_SEL = 01  (gain = 4)
     * [2:0] CS_GAIN_SEL  = cfg                                     */
    uint8_t en_rc = (mode == DRIVE_DC_H_MODE_SPEED ||
                     mode == DRIVE_DC_H_MODE_RIPPLE_COUNT) ? 0x80 : 0x00;
    drv_write(tile, DRV8214_REG_RC_CTRL0,
              en_rc | 0x08 | (cs_gain & 0x07));

    /* ---- REG_CTRL2: output filter ----
     * [7:6] OUT_FLT  = 11  (1000 Hz cutoff — 20x below 25 kHz PWM)
     * [5:0] EXT_DUTY = 0   (not used in I2C bridge mode)           */
    drv_write(tile, DRV8214_REG_CTRL2, 0xC0);

    /* ---- Ripple counting tuning (motor-specific parameters) ----
     * Skipped if motor_mohm == 0. Can also be (re)programmed at
     * runtime via tile_drive_dc_h_set_motor_params().               */
    drv_apply_motor_tuning(tile, motor_mohm, ripples_per_rev,
                           kv_uv_per_rpm);

    /* Every locked field is written: enable the output stage. */
    drv_write(tile, DRV8214_REG_CONFIG0,
              (uint8_t)((drv_read(tile, DRV8214_REG_CONFIG0) & ~0x06u) | 0x80u));

    tile->state = TILE_STATE_READY;
}

/* ---- Motor control ---- */

void tile_drive_dc_h_forward(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "forward: not ready");
        return;
    }
    if (!bridge_is_i2c(tile)) {
        TILE_ON_ERROR(tile, "forward: bridge in pad-control mode");
        return;
    }
    /* PWM mode: IN1=1, IN2=0 → Forward (OUT1=H, OUT2=L) */
    drv_write(tile, DRV8214_REG_CONFIG4, CONFIG4_BASE | 0x02);
}

void tile_drive_dc_h_reverse(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "reverse: not ready");
        return;
    }
    if (!bridge_is_i2c(tile)) {
        TILE_ON_ERROR(tile, "reverse: bridge in pad-control mode");
        return;
    }
    /* PWM mode: IN1=0, IN2=1 → Reverse (OUT1=L, OUT2=H) */
    drv_write(tile, DRV8214_REG_CONFIG4, CONFIG4_BASE | 0x01);
}

void tile_drive_dc_h_brake(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "brake: not ready");
        return;
    }
    if (!bridge_is_i2c(tile)) {
        TILE_ON_ERROR(tile, "brake: bridge in pad-control mode");
        return;
    }
    /* PWM mode: IN1=1, IN2=1 → Brake (low-side slow decay) */
    drv_write(tile, DRV8214_REG_CONFIG4, CONFIG4_BASE | 0x03);
}

void tile_drive_dc_h_coast(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "coast: not ready");
        return;
    }
    if (!bridge_is_i2c(tile)) {
        TILE_ON_ERROR(tile, "coast: bridge in pad-control mode");
        return;
    }
    /* PWM mode: IN1=0, IN2=0 → Coast (Hi-Z) */
    drv_write(tile, DRV8214_REG_CONFIG4, CONFIG4_BASE);
}

/* ---- Bridge control source ---- */

void tile_drive_dc_h_set_control_mode(tile_t* tile,
                                      drive_dc_h_control_mode_t mode)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_control_mode: not ready");
        return;
    }

    /* Read CONFIG4, then update PMODE/I2C_BC/INx, preserving RC_REP/STALL_REP/CBC_REP. */
    uint8_t cfg4 = drv_read(tile, DRV8214_REG_CONFIG4);
    /* Clear PMODE, I2C_BC, INx bits — we set them per-mode below. */
    cfg4 &= (uint8_t)~(CONFIG4_PMODE_MASK | CONFIG4_I2C_BC_MASK | CONFIG4_INX_MASK);

    switch (mode) {
        case DRIVE_DC_H_CTRL_PAD_PHEN:
            /* PMODE=0 (PH/EN), I2C_BC=0 */
            break;
        case DRIVE_DC_H_CTRL_PAD_IN1IN2:
            /* PMODE=1 (PWM), I2C_BC=0 */
            cfg4 |= CONFIG4_PMODE_MASK;
            break;
        case DRIVE_DC_H_CTRL_I2C:
        default:
            /* PMODE=1 (PWM), I2C_BC=1, start in coast */
            cfg4 |= CONFIG4_PMODE_MASK | CONFIG4_I2C_BC_MASK;
            break;
    }
    /* PMODE / I2C_BC are locked while EN_OUT = 1 (Table 8-29). */
    drv_locked_rmw(tile, DRV8214_REG_CONFIG4, 0xFF, cfg4);
}

/* ---- Regulation ---- */

void tile_drive_dc_h_set_target(tile_t* tile, uint8_t value)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_target: not ready");
        return;
    }
    drv_write(tile, DRV8214_REG_CTRL1, value);
}

void tile_drive_dc_h_set_regulation_mode(tile_t* tile,
                                         drive_dc_h_reg_mode_t mode)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_regulation_mode: not ready");
        return;
    }

    /* REG_CTRL[1:0] occupies CTRL0 bits [4:3]. */
    uint8_t bits = (uint8_t)((mode & 0x03) << 3);
    drv_locked_rmw(tile, DRV8214_REG_CTRL0, 0x18, bits);  /* REG_CTRL: locked */

    /* Speed regulation requires EN_RC=1; force it (and leave it on
     * when switching away — clearing it would invalidate get_speed
     * unexpectedly). */
    if (mode == DRIVE_DC_H_REG_SPEED) {
        drv_rmw(tile, DRV8214_REG_RC_CTRL0, 0x80, 0x80);
    }
}

/* ---- Current regulation ---- */

void tile_drive_dc_h_set_current_regulation_mode(tile_t* tile,
                                                 drive_dc_h_imode_t mode)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_current_regulation_mode: not ready");
        return;
    }
    /* IMODE[1:0] occupies CONFIG3 bits [7:6]. */
    drv_locked_rmw(tile, DRV8214_REG_CONFIG3, 0xC0,
            (uint8_t)((mode & 0x03) << 6));
}

void tile_drive_dc_h_set_current_sense_gain(tile_t* tile, uint8_t code)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_current_sense_gain: not ready");
        return;
    }
    if (code > 5) code = 5;
    drv_rmw(tile, DRV8214_REG_RC_CTRL0, 0x07, code & 0x07);
}

/* ---- Stall detection ---- */

void tile_drive_dc_h_set_stall_enabled(tile_t* tile, uint8_t enabled)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_stall_enabled: not ready");
        return;
    }
    /* EN_STALL is CONFIG0 bit 5. */
    drv_rmw(tile, DRV8214_REG_CONFIG0, 0x20,
            enabled ? 0x20 : 0x00);
}

void tile_drive_dc_h_set_inrush_time_ms(tile_t* tile, uint16_t ms)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_inrush_time_ms: not ready");
        return;
    }
    /* Each LSB = 102.4 us → ticks = ms * 1000 / 102 ≈ ms * 9.766.
     * Use ms * 10000 / 1024 to stay integer. Clamp to 0xFFFF. */
    uint32_t ticks = ((uint32_t)ms * 10000u) / 1024u;
    if (ticks > 0xFFFF) ticks = 0xFFFF;
    drv_write(tile, DRV8214_REG_CONFIG1, (uint8_t)(ticks & 0xFF));
    drv_write(tile, DRV8214_REG_CONFIG2, (uint8_t)((ticks >> 8) & 0xFF));
}

void tile_drive_dc_h_set_stall_recovery(tile_t* tile,
                                        drive_dc_h_stall_recovery_t mode)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_stall_recovery: not ready");
        return;
    }
    /* SMODE is CONFIG3 bit 5. */
    drv_locked_rmw(tile, DRV8214_REG_CONFIG3, 0x20,
            (mode == DRIVE_DC_H_STALL_REPORT) ? 0x20 : 0x00);
}

/* ---- Ripple-counter tuning ---- */

void tile_drive_dc_h_set_ripple_threshold(tile_t* tile, uint16_t count)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_ripple_threshold: not ready");
        return;
    }

    /* Threshold = RC_THR (10-bit) × RC_THR_SCALE (2, 8, 16, or 64).
     * Pick the smallest scale that lets RC_THR fit in 10 bits. */
    static const uint16_t scales[] = { 2, 8, 16, 64 };
    static const uint8_t  scale_bits[] = { 0, 1, 2, 3 };
    uint16_t rc_thr = 0;
    uint8_t  scale_b = 3;

    for (uint8_t i = 0; i < 4; i++) {
        uint32_t v = (uint32_t)count / scales[i];
        if (v <= 0x3FF) {
            rc_thr  = (uint16_t)v;
            scale_b = scale_bits[i];
            break;
        }
    }
    /* If count exceeds 0x3FF × 64 = 65472, clamp at max. */
    if (count > 65472u) {
        rc_thr  = 0x3FF;
        scale_b = 3;
    }

    /* RC_CTRL1 = RC_THR[7:0]. */
    drv_write(tile, DRV8214_REG_RC_CTRL1, (uint8_t)(rc_thr & 0xFF));

    /* RC_CTRL2 [3:2]=RC_THR_SCALE, [1:0]=RC_THR[9:8]. Preserve INV_R_SCALE
     * and KMC_SCALE (bits [7:4]). */
    uint8_t lo = (uint8_t)(((scale_b & 0x03) << 2) | ((rc_thr >> 8) & 0x03));
    drv_rmw(tile, DRV8214_REG_RC_CTRL2, 0x0F, lo);
}

void tile_drive_dc_h_set_ripple_filter_gain(tile_t* tile, uint8_t code)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_ripple_filter_gain: not ready");
        return;
    }
    /* FLT_GAIN_SEL[1:0] is RC_CTRL0 bits [4:3]. */
    drv_rmw(tile, DRV8214_REG_RC_CTRL0, 0x18,
            (uint8_t)((code & 0x03) << 3));
}

/* ---- Monitoring ---- */

uint8_t tile_drive_dc_h_get_fault(tile_t* tile)
{
    return drv_read(tile, DRV8214_REG_FAULT);
}

void tile_drive_dc_h_clear_fault(tile_t* tile)
{
    uint8_t cfg0 = drv_read(tile, DRV8214_REG_CONFIG0);
    drv_write(tile, DRV8214_REG_CONFIG0, cfg0 | 0x02);  /* CLR_FLT */
}

uint8_t tile_drive_dc_h_is_stalled(tile_t* tile)
{
    return (drv_read(tile, DRV8214_REG_FAULT) & DRV8214_FAULT_STALL) ? 1 : 0;
}

uint16_t tile_drive_dc_h_get_voltage_mv(tile_t* tile)
{
    uint8_t raw = drv_read(tile, DRV8214_REG_REG_STATUS1);
    if (raw == 0) return 0;

    /* VMTR scales with VM_GAIN_SEL (CONFIG0 bit 3):
     *   0 → 0-15.7 V range (0xFF = 15700 mV)
     *   1 → 0-3.92 V range (0xFF = 3920 mV)  */
    uint8_t cfg0 = drv_read(tile, DRV8214_REG_CONFIG0);
    uint16_t fs = (cfg0 & 0x08) ? 3920 : 15700;

    return (uint16_t)((uint32_t)raw * fs / 255);
}

uint16_t tile_drive_dc_h_get_current_ma(tile_t* tile)
{
    uint8_t imtr = drv_read(tile, DRV8214_REG_REG_STATUS2);
    if (imtr == 0) return 0;

    /* Read CS_GAIN_SEL to determine current range */
    uint8_t rc_ctrl0 = drv_read(tile, DRV8214_REG_RC_CTRL0);
    uint8_t cs_sel = rc_ctrl0 & 0x07;

    /* IMTR: 0x00 = 0 A, 0xC0 = max current for gain setting
     * mA = imtr * max_mA / 192 */
    uint16_t max_ma = cs_max_ma[cs_sel];
    return (uint16_t)((uint32_t)imtr * max_ma / 192);
}

uint8_t tile_drive_dc_h_get_speed(tile_t* tile)
{
    return drv_read(tile, DRV8214_REG_RC_STATUS1);
}

uint32_t tile_drive_dc_h_get_speed_rpm(tile_t* tile)
{
    uint8_t raw   = drv_read(tile, DRV8214_REG_RC_STATUS1);
    uint8_t ctrl0 = drv_read(tile, DRV8214_REG_CTRL0);

    drive_dc_h_state_t *st = state_for(tile);
    uint8_t rpr = st->ripples_per_rev ? st->ripples_per_rev : 12;

    /* Inverse of the set_speed_rpm() conversion, using the live
     * W_SCALE from CTRL0 so the pair always agrees:
     * RPM = SPEED × W_SCALE [rad/s] × 60 / (2π × rpr).
     * Max numerator 255 × 128 × 60000 ≈ 1.96e9 fits uint32. */
    return ((uint32_t)raw * (uint32_t)w_scale_lookup[ctrl0 & 0x03] * 60000u)
           / (TWO_PI_X1000 * (uint32_t)rpr);
}

uint16_t tile_drive_dc_h_get_ripple_count(tile_t* tile)
{
    uint8_t lo = drv_read(tile, DRV8214_REG_RC_STATUS2);
    uint8_t hi = drv_read(tile, DRV8214_REG_RC_STATUS3);
    return ((uint16_t)hi << 8) | lo;
}

void tile_drive_dc_h_clear_ripple_count(tile_t* tile)
{
    uint8_t cfg0 = drv_read(tile, DRV8214_REG_CONFIG0);
    drv_write(tile, DRV8214_REG_CONFIG0, cfg0 | 0x04);  /* CLR_CNT */
}

/* ---- Power management ---- */

void tile_drive_dc_h_sleep(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) return;

    uint8_t cfg0 = drv_read(tile, DRV8214_REG_CONFIG0);
    drv_write(tile, DRV8214_REG_CONFIG0, cfg0 & ~0x86);  /* EN_OUT = 0; no CLR_* */
    tile->state = TILE_STATE_SLEEPING;
}

void tile_drive_dc_h_wake(tile_t* tile)
{
    if (tile->state != TILE_STATE_SLEEPING) return;

    uint8_t cfg0 = drv_read(tile, DRV8214_REG_CONFIG0);
    drv_write(tile, DRV8214_REG_CONFIG0, (cfg0 & ~0x06) | 0x80);  /* EN_OUT = 1 */
    tile->hal->delay_ms(1);  /* tWAKE < 410 us */
    tile->state = TILE_STATE_READY;
}

/* ============================================================== */
/* Runtime — tier-2 idiomatic helpers                              */
/* ============================================================== */

/* Drive in `direction` by writing the appropriate IN1/IN2 bridge
 * bits. Caller must already be in I²C bridge-control mode and the
 * tile must be READY. Returns 0 on success, 1 if the bridge is in
 * pad-control mode (matching the existing forward()/reverse() guard). */
static uint8_t drv_drive(tile_t *tile, drive_dc_h_direction_t direction)
{
    if (!bridge_is_i2c(tile)) return 1;
    uint8_t bits = (direction == DRIVE_DC_H_DIR_REVERSE) ? 0x01 : 0x02;
    drv_write(tile, DRV8214_REG_CONFIG4, CONFIG4_BASE | bits);
    return 0;
}

void tile_drive_dc_h_set_speed_rpm(tile_t *tile, uint32_t rpm,
                                   drive_dc_h_direction_t direction)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_speed_rpm: not ready");
        return;
    }

    /* Switch into speed-regulation mode. set_regulation_mode also
     * forces EN_RC=1 so the ripple-speed estimator is live. */
    tile_drive_dc_h_set_regulation_mode(tile, DRIVE_DC_H_REG_SPEED);

    drive_dc_h_state_t *st = state_for(tile);
    uint8_t rpr = st->ripples_per_rev ? st->ripples_per_rev : 12;

    /* Pick the finest W_SCALE (16/32/64/128 rad/s) whose 8-bit WSET
     * range still reaches the requested speed — a finer scale means
     * finer RPM granularity at the low end (LSB = 60 × W_SCALE /
     * (2π × rpr) RPM, e.g. ~12.7 RPM at W_SCALE=16 / rpr=12). The
     * driver owns CTRL0[1:0]; get_speed_rpm() reads it back so
     * conversions stay consistent.
     *
     * Ripple speed [rad/s] = rpm × rpr × 2π / 60. Saturate rpm × rpr
     * first so the ×6283 below fits uint32 (400000 × 6283 ≈ 2.5e9);
     * 400000 rpm·ripples is ~41900 rad/s, above the 32640 rad/s top
     * of the coarsest scale, so the clamp never changes the result. */
    if (rpm > 400000u) rpm = 400000u;
    uint32_t rpm_rpr = rpm * (uint32_t)rpr;
    if (rpm_rpr > 400000u) rpm_rpr = 400000u;

    uint8_t  scale_bits = 3;
    uint32_t wset = 0xFFu;
    for (uint8_t i = 0; i < 4; i++) {
        uint32_t den = 60000u * (uint32_t)w_scale_lookup[i];
        uint32_t v   = (rpm_rpr * TWO_PI_X1000 + den / 2) / den;  /* round */
        if (v <= 0xFFu) {
            scale_bits = i;
            wset = v;
            break;
        }
    }
    if (rpm > 0 && wset == 0) wset = 1;  /* don't round a spin request to stop */

    drv_rmw(tile, DRV8214_REG_CTRL0, 0x03, scale_bits);
    drv_write(tile, DRV8214_REG_CTRL1, (uint8_t)wset);

    /* Engage the bridge in the requested direction. The chip's PI loop
     * drives the motor toward the target speed autonomously. */
    if (drv_drive(tile, direction) != 0) {
        TILE_ON_ERROR(tile, "set_speed_rpm: bridge in pad-control mode");
    }
}

void tile_drive_dc_h_set_motor_params(tile_t* tile, uint16_t motor_mohm,
                                      uint16_t ripples_per_rev,
                                      uint16_t kv_uv_per_rpm)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_motor_params: not ready");
        return;
    }

    uint8_t rpr = (ripples_per_rev == 0)    ? 12
                : (ripples_per_rev > 255u)  ? 255u
                : (uint8_t)ripples_per_rev;

    /* Cache for the set_speed_rpm()/get_speed_rpm() conversions. */
    state_for(tile)->ripples_per_rev = rpr;

    drv_apply_motor_tuning(tile, motor_mohm, rpr, kv_uv_per_rpm);
}

void tile_drive_dc_h_move_distance(tile_t *tile, uint16_t ripples,
                                   drive_dc_h_direction_t direction)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "move_distance: not ready");
        return;
    }
    if (!bridge_is_i2c(tile)) {
        TILE_ON_ERROR(tile, "move_distance: bridge in pad-control mode");
        return;
    }
    if (ripples == 0) {
        return;
    }

    /* Program the ripple-count threshold so the chip raises CNT_DONE
     * once the running count reaches `ripples`. set_ripple_threshold
     * picks the smallest RC_THR_SCALE that fits and clamps at 65472. */
    tile_drive_dc_h_set_ripple_threshold(tile, ripples);

    /* Make sure ripple counting is enabled; init only flips EN_RC for
     * SPEED / RIPPLE_COUNT modes. Closed-loop step needs it on. */
    drv_rmw(tile, DRV8214_REG_RC_CTRL0, 0x80, 0x80);

    /* Zero the counter and clear any latent CNT_DONE. */
    tile_drive_dc_h_clear_ripple_count(tile);
    tile_drive_dc_h_clear_fault(tile);

    /* Engage the bridge in the requested direction. */
    (void)drv_drive(tile, direction);

    /* Poll the FAULT register for CNT_DONE or STALL. The chip's stall
     * detector is the closest thing to a built-in timeout — if the
     * motor jams, STALL fires and we abort. Polling cadence 5 ms
     * keeps response tight without flooding the I²C bus. */
    while (1) {
        uint8_t fault = drv_read(tile, DRV8214_REG_FAULT);
        if (fault & DRV8214_FAULT_CNT_DONE) {
            break;
        }
        if (fault & DRV8214_FAULT_STALL) {
            break;
        }
        tile->hal->delay_ms(5);
    }

    /* Brake-on-completion: actively hold the new position rather than
     * coasting past it. Caller can call coast() afterwards if they
     * want freewheel instead. */
    drv_write(tile, DRV8214_REG_CONFIG4, CONFIG4_BASE | 0x03);
}

uint8_t tile_drive_dc_h_is_running(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY) return 0;

    uint8_t cfg4 = drv_read(tile, DRV8214_REG_CONFIG4);

    /* Pad-control modes don't have I²C-visible drive state. */
    if (!(cfg4 & CONFIG4_I2C_BC_MASK)) return 0;

    /* IN1/IN2 patterns in PWM mode:
     *   00 = coast   → not running
     *   01 = reverse → running
     *   10 = forward → running
     *   11 = brake   → not running                                 */
    uint8_t inx = cfg4 & CONFIG4_INX_MASK;
    return (inx == 0x01 || inx == 0x02) ? 1 : 0;
}

uint8_t tile_drive_dc_h_wait_for_stop(tile_t *tile, uint32_t timeout_ms)
{
    if (tile->state != TILE_STATE_READY) return 0;

    /* Sample the ripple counter at 10 ms intervals. The rotor is
     * declared "stopped" when two consecutive 10 ms-spaced samples
     * (≥30 ms total observation) report the same count — well below
     * mechanical settling time even for fast unloaded motors. */
    uint16_t prev = tile_drive_dc_h_get_ripple_count(tile);
    uint8_t  stable_runs = 0;

    for (uint32_t elapsed = 0; elapsed < timeout_ms; elapsed += 10) {
        tile->hal->delay_ms(10);
        uint16_t cur = tile_drive_dc_h_get_ripple_count(tile);
        if (cur == prev) {
            stable_runs++;
            if (stable_runs >= 2) return 1;
        } else {
            stable_runs = 0;
            prev = cur;
        }
    }

    return 0;
}
