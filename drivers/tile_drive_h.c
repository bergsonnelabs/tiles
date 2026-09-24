/**
 * @file   tile_drive_h.c
 * @brief  LRA haptic driver implementation (DRV2605).
 */

#include "tile_drive_h.h"
#include <stddef.h>

/* -------------------------------------------------------------- */
/* Instance mapping                                                */
/* -------------------------------------------------------------- */

static const uint8_t id_table[] = {
    DRV2605L_I2C_ADDR_DEFAULT,   /* instance 0 — fixed address (0x5A) */
};

#define ID_TABLE_LEN  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    if (instance >= ID_TABLE_LEN) return 0x00;
    return id_table[instance];
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

/** @brief  Integer square root (floor), for the Eq. 3/5 loss factors. */
static uint32_t isqrt32(uint32_t x)
{
    uint32_t res = 0;
    uint32_t bit = 1uL << 30;
    while (bit > x) bit >>= 2;
    while (bit != 0) {
        if (x >= res + bit) {
            x -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

/**
 * @brief  Poll the GO bit until it self-clears or timeout.
 * @return 1 if GO cleared (process complete), 0 if timeout.
 */
static uint8_t drv_wait_go(tile_t* tile, uint16_t timeout_ms)
{
    while (timeout_ms > 0) {
        if ((drv_read(tile, DRV2605L_REG_GO) & 0x01) == 0) return 1;
        tile->hal->delay_ms(10);
        if (timeout_ms <= 10) break;
        timeout_ms -= 10;
    }
    return 0;
}

/* -------------------------------------------------------------- */
/* Public API                                                      */
/* -------------------------------------------------------------- */

uint8_t tile_drive_h_find(tiles_pal_t* hal, uint8_t instance)
{
    uint8_t id = resolve_id(instance);
    if (id == 0x00) return 0;
    return (hal->i2c_is_ready(hal->handle, id) == 0) ? 1 : 0;
}

void tile_drive_h_init(tiles_pal_t* hal, uint8_t instance, tile_t* tile,
                       const drive_h_cfg_t *cfg)
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

    /* Verify device ID (STATUS bits 7:5). Compare only the ID field:
     * DIAG_RESULT / OVER_TEMP / OC_DETECT are sticky until read, so a
     * leftover flag from a previous session must not fail init. */
    uint8_t status = drv_read(tile, DRV2605L_REG_STATUS);
    if ((status & DRV2605L_STATUS_DEVICE_ID) !=
        (DRV2605L_STATUS_DEFAULT & DRV2605L_STATUS_DEVICE_ID)) {
        TILE_ON_ERROR(tile, "init: unexpected device id");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    /* Exit standby */
    drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_INTERNAL_TRIG);
    hal->delay_ms(400);

    /* Parse config. Defaults target the typical externally attached
     * coin LRA (2.0 Vrms-class, ~235 Hz): closed-loop smart-loop. At
     * 238 Hz and SAMPLE_TIME 300 µs (CONTROL2 reset), 0x56 is ~2.2 Vrms
     * (SLOS825E Eq 3) and 0x8C a 3.07 V peak clamp (Eq 7). The tile
     * has no onboard actuator. */
    uint8_t closed_loop = 1;
    uint8_t library = 6;
    uint8_t rated_v = 0x56;   /* ~2.2 Vrms closed loop (bench-verified) */
    uint8_t od_clamp = 0x8C;  /* 3.07 V peak */
    if (cfg != NULL) {
        if (cfg->library >= 1 && cfg->library <= 6) {
            library = cfg->library;
        }
        closed_loop = cfg->closed_loop ? 1 : 0;
        if (cfg->rated_voltage != 0) rated_v  = cfg->rated_voltage;
        if (cfg->od_clamp != 0)      od_clamp = cfg->od_clamp;
    }

    /* LRA drive levels */
    drv_write(tile, DRV2605L_REG_RATED_VOLTAGE, rated_v);
    drv_write(tile, DRV2605L_REG_OD_CLAMP, od_clamp);

    /* Reset auto-cal results to defaults so diagnostics and calibration
     * start from a known state. The DRV2605 retains registers across
     * MCU resets if it stays powered — stale cal values from a prior
     * session would skew back-EMF thresholds. */
    drv_write(tile, DRV2605L_REG_A_CAL_COMP, 0x0D);
    drv_write(tile, DRV2605L_REG_A_CAL_BEMF, 0x6D);

    /* Configure FEEDBACK_CTRL: ERM_LRA bit [7], brake factor, loop gain, BEMF gain */
    uint8_t fb_ctrl = 0xB6;  /* LRA mode, default brake/loop/BEMF */
    if (library <= 5) {
        fb_ctrl = 0x36;      /* ERM mode (N_ERM_LRA = 0) */
    }
    drv_write(tile, DRV2605L_REG_FEEDBACK_CTRL, fb_ctrl);
    hal->delay_ms(100);

    /* Set DRIVE_TIME for the typical ~235 Hz coin LRA.
     * Optimal = 0.5 × LRA period = 2.13 ms → DRIVE_TIME = 16 (0x10).
     * CONTROL1: preserve STARTUP_BOOST [7], set DRIVE_TIME [4:0].
     * Retune for other actuators with tile_drive_h_set_resonance_hz(). */
    uint8_t ctrl1 = drv_read(tile, DRV2605L_REG_CONTROL1);
    ctrl1 = (ctrl1 & 0xE0) | 0x10;
    drv_write(tile, DRV2605L_REG_CONTROL1, ctrl1);

    /* Configure CONTROL3: LRA_OPEN_LOOP [0], ERM_OPEN_LOOP [5] */
    uint8_t ctrl3 = drv_read(tile, DRV2605L_REG_CONTROL3);
    if (library == 6) {
        /* LRA mode */
        if (closed_loop) {
            ctrl3 &= ~0x01;  /* LRA_OPEN_LOOP = 0 (closed loop) */
        } else {
            ctrl3 |= 0x01;   /* LRA_OPEN_LOOP = 1 (open loop) */
        }
    } else {
        /* ERM mode */
        if (closed_loop) {
            ctrl3 &= ~0x20;  /* ERM_OPEN_LOOP = 0 (closed loop) */
        } else {
            ctrl3 |= 0x20;   /* ERM_OPEN_LOOP = 1 (open loop) */
        }
    }
    drv_write(tile, DRV2605L_REG_CONTROL3, ctrl3);

    /* Select waveform library */
    drv_write(tile, DRV2605L_REG_LIBRARY_SEL, library);

    tile->state = TILE_STATE_READY;
}

void tile_drive_h_play(tile_t* tile, uint8_t index, uint8_t repeats)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "play: not ready");
        return;
    }

    /* Load effect into sequence slot 0, terminate in slot 1 */
    drv_write(tile, DRV2605L_REG_WAVE_SEQ_0, index);
    drv_write(tile, DRV2605L_REG_WAVE_SEQ_1, 0);

    /* Trigger */
    drv_write(tile, DRV2605L_REG_GO, 0x01);

    /* Repeat with gap */
    for (uint8_t i = 1; i < repeats; i++) {
        tile->hal->delay_ms(200);
        drv_write(tile, DRV2605L_REG_GO, 0x01);
    }
}

void tile_drive_h_play_sequence(tile_t* tile, const uint8_t *effects,
                                uint8_t count)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "play_sequence: not ready");
        return;
    }
    if (count == 0) return;
    if (count > DRV2605L_SEQ_MAX) count = DRV2605L_SEQ_MAX;

    /* Load effects into sequencer slots */
    for (uint8_t i = 0; i < count; i++) {
        drv_write(tile, DRV2605L_REG_WAVE_SEQ_0 + i, effects[i]);
    }

    /* Terminate sequence (if fewer than 8 effects) */
    if (count < DRV2605L_SEQ_MAX) {
        drv_write(tile, DRV2605L_REG_WAVE_SEQ_0 + count, 0x00);
    }

    /* Trigger playback */
    drv_write(tile, DRV2605L_REG_GO, 0x01);
}

void tile_drive_h_load_sequence(tile_t* tile, const uint8_t *effects,
                                uint8_t count)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "load_sequence: not ready");
        return;
    }
    if (count == 0) return;
    if (count > DRV2605L_SEQ_MAX) count = DRV2605L_SEQ_MAX;

    for (uint8_t i = 0; i < count; i++) {
        drv_write(tile, DRV2605L_REG_WAVE_SEQ_0 + i, effects[i]);
    }

    if (count < DRV2605L_SEQ_MAX) {
        drv_write(tile, DRV2605L_REG_WAVE_SEQ_0 + count, 0x00);
    }
}

void tile_drive_h_set_trigger(tile_t* tile, uint8_t mode)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_trigger: not ready");
        return;
    }

    uint8_t reg_mode;
    switch (mode) {
    case DRIVE_H_TRIG_EDGE:
        reg_mode = DRV2605L_MODE_EXT_EDGE;
        break;
    case DRIVE_H_TRIG_LEVEL:
        reg_mode = DRV2605L_MODE_EXT_LEVEL;
        break;
    default:
        reg_mode = DRV2605L_MODE_INTERNAL_TRIG;
        break;
    }
    drv_write(tile, DRV2605L_REG_MODE, reg_mode);
}

uint8_t tile_drive_h_is_playing(tile_t* tile)
{
    return (drv_read(tile, DRV2605L_REG_GO) & 0x01);
}

void tile_drive_h_stop(tile_t* tile)
{
    drv_write(tile, DRV2605L_REG_GO, 0x00);
}

void tile_drive_h_rtp_start(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "rtp_start: not ready");
        return;
    }
    drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_RTP);
    drv_write(tile, DRV2605L_REG_RTP, 0x00);
}

void tile_drive_h_rtp_write(tile_t* tile, uint8_t amplitude)
{
    drv_write(tile, DRV2605L_REG_RTP, amplitude);
}

void tile_drive_h_rtp_stop(tile_t* tile)
{
    drv_write(tile, DRV2605L_REG_RTP, 0x00);
    drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_INTERNAL_TRIG);
}

uint8_t tile_drive_h_get_status(tile_t* tile)
{
    return drv_read(tile, DRV2605L_REG_STATUS);
}

uint8_t tile_drive_h_diagnose(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "diagnose: not ready");
        return 0;
    }

    /* Diagnostics uses back-EMF sensing (closed-loop path) regardless
     * of normal operating mode. Temporarily ensure closed-loop so the
     * back-EMF thresholds match RATED_VOLTAGE / OD_CLAMP. */
    uint8_t ctrl3_saved = drv_read(tile, DRV2605L_REG_CONTROL3);
    drv_write(tile, DRV2605L_REG_CONTROL3,
              ctrl3_saved & ~0x21);  /* clear LRA_OPEN_LOOP & ERM_OPEN_LOOP */

    /* Reset auto-cal results to defaults so the diagnostic engine
     * evaluates back-EMF from a known baseline. Stale values from
     * a prior calibration can tighten thresholds and cause false
     * failures with small actuators. */
    uint8_t comp_saved = drv_read(tile, DRV2605L_REG_A_CAL_COMP);
    uint8_t bemf_saved = drv_read(tile, DRV2605L_REG_A_CAL_BEMF);
    drv_write(tile, DRV2605L_REG_A_CAL_COMP, 0x0D);
    drv_write(tile, DRV2605L_REG_A_CAL_BEMF, 0x6D);

    /* Enter diagnostics mode */
    drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_DIAGNOSTICS);

    /* Trigger diagnostic routine */
    drv_write(tile, DRV2605L_REG_GO, 0x01);

    /* Wait for completion (up to 2 seconds) */
    if (!drv_wait_go(tile, 2000)) {
        drv_write(tile, DRV2605L_REG_CONTROL3, ctrl3_saved);
        drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_INTERNAL_TRIG);
        return 0;
    }

    /* Read diagnostic result: DIAG_RESULT bit [3] of STATUS.
     * 0 = passed, 1 = failed. */
    uint8_t status = drv_read(tile, DRV2605L_REG_STATUS);
    uint8_t passed = (status & DRV2605L_STATUS_DIAG_RESULT) ? 0 : 1;

    /* Restore cal registers, open/closed-loop setting, and mode */
    drv_write(tile, DRV2605L_REG_A_CAL_COMP, comp_saved);
    drv_write(tile, DRV2605L_REG_A_CAL_BEMF, bemf_saved);
    drv_write(tile, DRV2605L_REG_CONTROL3, ctrl3_saved);
    drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_INTERNAL_TRIG);

    return passed;
}

uint8_t tile_drive_h_calibrate(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "calibrate: not ready");
        return 0;
    }

    /* Calibration always uses closed-loop back-EMF feedback.
     * Save and temporarily override open/closed-loop setting. */
    uint8_t ctrl3_saved = drv_read(tile, DRV2605L_REG_CONTROL3);
    drv_write(tile, DRV2605L_REG_CONTROL3,
              ctrl3_saved & ~0x21);  /* clear LRA_OPEN_LOOP & ERM_OPEN_LOOP */

    /* Set TI-recommended calibration parameters.
     * FEEDBACK_CTRL: preserve ERM_LRA bit, set FB_BRAKE_FACTOR=2,
     * LOOP_GAIN=1, BEMF_GAIN=2. */
    uint8_t fb_saved = drv_read(tile, DRV2605L_REG_FEEDBACK_CTRL);
    uint8_t fb_cal = (fb_saved & 0x80) | (2 << 4) | (1 << 2) | 2;
    drv_write(tile, DRV2605L_REG_FEEDBACK_CTRL, fb_cal);

    /* AUTO_CAL_TIME = 3 → 1000-1200 ms for reliable convergence */
    uint8_t ctrl4 = drv_read(tile, DRV2605L_REG_CONTROL4);
    ctrl4 = (ctrl4 & 0x0F) | (3 << 4);
    drv_write(tile, DRV2605L_REG_CONTROL4, ctrl4);

    /* Enter auto-calibration mode */
    drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_CALIBRATION);

    /* Trigger calibration */
    drv_write(tile, DRV2605L_REG_GO, 0x01);

    /* Wait for completion (up to 2 seconds) */
    if (!drv_wait_go(tile, 2000)) {
        drv_write(tile, DRV2605L_REG_FEEDBACK_CTRL, fb_saved);
        drv_write(tile, DRV2605L_REG_CONTROL3, ctrl3_saved);
        drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_INTERNAL_TRIG);
        return 0;
    }

    /* Check DIAG_RESULT: 0 = converged (pass), 1 = failed */
    uint8_t status = drv_read(tile, DRV2605L_REG_STATUS);
    uint8_t passed = (status & DRV2605L_STATUS_DIAG_RESULT) ? 0 : 1;

    /* Restore feedback control and open/closed-loop setting.
     * Note: A_CAL_COMP and A_CAL_BEMF are now populated by the
     * cal engine and persist until reset. */
    drv_write(tile, DRV2605L_REG_FEEDBACK_CTRL, fb_saved);
    drv_write(tile, DRV2605L_REG_CONTROL3, ctrl3_saved);
    drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_INTERNAL_TRIG);

    return passed;
}

uint16_t tile_drive_h_get_vbat_mv(tile_t* tile)
{
    uint8_t raw = drv_read(tile, DRV2605L_REG_VBAT);
    if (raw == 0) return 0;
    /* VBAT (V) = raw * 5.6 / 255
     * In mV:    raw * 5600 / 255
     * Max raw=255 → 255*5600 = 1,428,000 — fits uint32 */
    return (uint16_t)((uint32_t)raw * 5600 / 255);
}

uint16_t tile_drive_h_get_resonance_hz(tile_t* tile)
{
    uint8_t raw = drv_read(tile, DRV2605L_REG_LRA_PERIOD);
    if (raw == 0) return 0;
    /* Period (us) = raw * 98.46
     * Freq (Hz)   = 1e6 / (raw * 98.46)
     *             = 10156 / raw  (integer approximation) */
    return (uint16_t)(10156u / raw);
}

void tile_drive_h_standby(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) return;
    drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_STANDBY);
    tile->state = TILE_STATE_SLEEPING;
}

void tile_drive_h_wake(tile_t* tile)
{
    if (tile->state != TILE_STATE_SLEEPING) return;
    drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_INTERNAL_TRIG);
    tile->hal->delay_ms(5);
    tile->state = TILE_STATE_READY;
}

/* -------------------------------------------------------------- */
/* Sequencer slot wait                                             */
/* -------------------------------------------------------------- */

void tile_drive_h_set_sequence_wait(tile_t* tile, uint8_t slot,
                                    uint8_t delay_steps)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_sequence_wait: not ready");
        return;
    }
    if (slot >= DRV2605L_SEQ_MAX) return;

    /* Slot byte: bit 7 = WAIT flag, bits 6:0 = wait time × 10 ms.
     * Clip to 0x7F to keep the WAIT bit asserted. */
    uint8_t value = 0x80 | (delay_steps & 0x7F);
    drv_write(tile, DRV2605L_REG_WAVE_SEQ_0 + slot, value);
}

/* -------------------------------------------------------------- */
/* Library + actuator-tuning runtime setters                       */
/* -------------------------------------------------------------- */

void tile_drive_h_set_library(tile_t* tile, uint8_t library)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_library: not ready");
        return;
    }
    if (library > DRIVE_H_LIB_LRA) return;

    /* Update LIBRARY_SEL (low 3 bits, preserve HI_Z and reserved). */
    uint8_t lib_reg = drv_read(tile, DRV2605L_REG_LIBRARY_SEL);
    lib_reg = (lib_reg & ~0x07) | (library & 0x07);
    drv_write(tile, DRV2605L_REG_LIBRARY_SEL, lib_reg);

    /* Update N_ERM_LRA bit in FEEDBACK_CTRL: 1 = LRA, 0 = ERM.
     * Library 6 = LRA; libraries 1..5 = ERM (library 0 leaves it
     * at whatever the user previously selected). */
    if (library != DRIVE_H_LIB_EMPTY) {
        uint8_t fb = drv_read(tile, DRV2605L_REG_FEEDBACK_CTRL);
        if (library == DRIVE_H_LIB_LRA) {
            fb |= 0x80;
        } else {
            fb &= ~0x80;
        }
        drv_write(tile, DRV2605L_REG_FEEDBACK_CTRL, fb);
    }
}

void tile_drive_h_set_actuator_params(tile_t* tile,
                                      uint8_t rated_voltage,
                                      uint8_t od_clamp,
                                      uint8_t fb_brake,
                                      uint8_t loop_gain)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_actuator_params: not ready");
        return;
    }

    if (rated_voltage != 0) {
        drv_write(tile, DRV2605L_REG_RATED_VOLTAGE, rated_voltage);
    }
    if (od_clamp != 0) {
        drv_write(tile, DRV2605L_REG_OD_CLAMP, od_clamp);
    }

    /* FEEDBACK_CTRL: bit 7 N_ERM_LRA (preserve), bits 6:4 FB_BRAKE_FACTOR,
     * bits 3:2 LOOP_GAIN, bits 1:0 BEMF_GAIN (preserve). */
    if (fb_brake != 0xFF || loop_gain != 0xFF) {
        uint8_t fb = drv_read(tile, DRV2605L_REG_FEEDBACK_CTRL);
        if (fb_brake != 0xFF) {
            fb = (fb & ~0x70) | ((fb_brake & 0x07) << 4);
        }
        if (loop_gain != 0xFF) {
            fb = (fb & ~0x0C) | ((loop_gain & 0x03) << 2);
        }
        drv_write(tile, DRV2605L_REG_FEEDBACK_CTRL, fb);
    }
}

void tile_drive_h_set_loop_mode(tile_t* tile, uint8_t closed)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_loop_mode: not ready");
        return;
    }

    /* Set/clear both ERM_OPEN_LOOP [5] and LRA_OPEN_LOOP [0]; only
     * the bit for the active actuator type takes effect, and keeping
     * them in lockstep matches diagnose()/calibrate(). */
    uint8_t ctrl3 = drv_read(tile, DRV2605L_REG_CONTROL3);
    if (closed) {
        ctrl3 &= ~0x21;
    } else {
        ctrl3 |= 0x21;
    }
    drv_write(tile, DRV2605L_REG_CONTROL3, ctrl3);
}

void tile_drive_h_set_actuator_voltage(tile_t* tile, uint16_t rated_mv,
                                       uint16_t overdrive_mv)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_actuator_voltage: not ready");
        return;
    }

    if (rated_mv < 300)     rated_mv = 300;
    if (rated_mv > 3600)    rated_mv = 3600;
    if (overdrive_mv < rated_mv) overdrive_mv = rated_mv;
    if (overdrive_mv > 5000)     overdrive_mv = 5000;

    uint8_t is_lra = (drv_read(tile, DRV2605L_REG_FEEDBACK_CTRL) & 0x80) != 0;
    uint32_t rated_reg;
    uint32_t od_reg;

    if (!is_lra) {
        /* ERM — Eq 2: V_avg = 21.33 mV × RATED_VOLTAGE;
         *       Eq 4: V_avg = 21.96 mV × OD_CLAMP. */
        rated_reg = ((uint32_t)rated_mv * 100u) / 2133u;
        od_reg    = ((uint32_t)overdrive_mv * 100u) / 2196u;
    } else {
        /* LRA — Eq 3/5 depend on the drive frequency and back-EMF
         * sample window. Derive f from the programmed DRIVE_TIME
         * (period = 2 × (0.5 ms + N × 0.1 ms)) and t_SAMPLE from
         * CONTROL2, so set_resonance_hz() feeds into the maths. */
        uint32_t n = drv_read(tile, DRV2605L_REG_CONTROL1) & 0x1F;
        uint32_t f_hz = 1000000u / (2u * (500u + 100u * n));
        static const uint16_t sample_us[4] = { 150, 200, 250, 300 };
        uint32_t t_s = sample_us[(drv_read(tile, DRV2605L_REG_CONTROL2) >> 4) & 0x03];

        /* Eq 3: V_rms = 20.71 mV × RATED / √(1 − (4·t_s + 300 µs)·f).
         * Loss factors are carried in per-mille (‰) integer form. */
        uint32_t loss_pm = ((4u * t_s + 300u) * f_hz) / 1000u;
        if (loss_pm > 900u) loss_pm = 900u;
        uint32_t sqrt_pm = isqrt32((1000u - loss_pm) * 1000u);
        rated_reg = ((uint32_t)rated_mv * sqrt_pm) / 20710u;

        /* Eq 5: V_rms = 21.32 mV × OD_CLAMP × √(1 − f × 800 µs). */
        uint32_t loss2_pm = (f_hz * 800u) / 1000u;
        if (loss2_pm > 900u) loss2_pm = 900u;
        uint32_t sqrt2_pm = isqrt32((1000u - loss2_pm) * 1000u);
        od_reg = ((uint32_t)overdrive_mv * 100000u) / (2132u * sqrt2_pm);
    }

    if (rated_reg < 1)   rated_reg = 1;
    if (rated_reg > 255) rated_reg = 255;
    if (od_reg < 1)      od_reg = 1;
    if (od_reg > 255)    od_reg = 255;

    drv_write(tile, DRV2605L_REG_RATED_VOLTAGE, (uint8_t)rated_reg);
    drv_write(tile, DRV2605L_REG_OD_CLAMP, (uint8_t)od_reg);
}

void tile_drive_h_set_resonance_hz(tile_t* tile, uint16_t hz)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_resonance_hz: not ready");
        return;
    }

    if (hz < 125) hz = 125;
    if (hz > 300) hz = 300;

    /* DRIVE_TIME = (half-period − 0.5 ms) / 0.1 ms, clamped to [4:0]. */
    uint32_t half_period_us = 500000u / hz;
    uint32_t n = (half_period_us - 500u) / 100u;
    if (n > 31u) n = 31u;

    uint8_t ctrl1 = drv_read(tile, DRV2605L_REG_CONTROL1);
    drv_write(tile, DRV2605L_REG_CONTROL1, (uint8_t)((ctrl1 & 0xE0) | n));
}

void tile_drive_h_set_resonance_params(tile_t* tile,
                                       uint8_t sample_time,
                                       uint8_t blanking_time,
                                       uint8_t idiss_time)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_resonance_params: not ready");
        return;
    }

    if (sample_time == 0xFF && blanking_time == 0xFF && idiss_time == 0xFF) {
        return;
    }

    /* CONTROL2: bit 7 BIDIR_INPUT (preserve), bit 6 BRAKE_STABILIZER
     * (preserve), bits 5:4 SAMPLE_TIME, bits 3:2 BLANKING_TIME,
     * bits 1:0 IDISS_TIME. */
    uint8_t ctrl2 = drv_read(tile, DRV2605L_REG_CONTROL2);
    if (sample_time != 0xFF) {
        ctrl2 = (ctrl2 & ~0x30) | ((sample_time & 0x03) << 4);
    }
    if (blanking_time != 0xFF) {
        ctrl2 = (ctrl2 & ~0x0C) | ((blanking_time & 0x03) << 2);
    }
    if (idiss_time != 0xFF) {
        ctrl2 = (ctrl2 & ~0x03) | (idiss_time & 0x03);
    }
    drv_write(tile, DRV2605L_REG_CONTROL2, ctrl2);
}

/* -------------------------------------------------------------- */
/* Library waveform timing offsets (open-loop only)                */
/* -------------------------------------------------------------- */

void tile_drive_h_set_waveform_timing(tile_t* tile,
                                      int8_t overdrive,
                                      int8_t sustain_pos,
                                      int8_t sustain_neg,
                                      int8_t brake)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_waveform_timing: not ready");
        return;
    }
    drv_write(tile, DRV2605L_REG_ODT, (uint8_t)overdrive);
    drv_write(tile, DRV2605L_REG_SPT, (uint8_t)sustain_pos);
    drv_write(tile, DRV2605L_REG_SNT, (uint8_t)sustain_neg);
    drv_write(tile, DRV2605L_REG_BRT, (uint8_t)brake);
}

/* -------------------------------------------------------------- */
/* RTP format                                                      */
/* -------------------------------------------------------------- */

void tile_drive_h_set_rtp_format(tile_t* tile, uint8_t unsigned_, uint8_t bidir)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_rtp_format: not ready");
        return;
    }

    /* CONTROL3 bit 3 = DATA_FORMAT_RTP (1 = unsigned, 0 = signed). */
    uint8_t ctrl3 = drv_read(tile, DRV2605L_REG_CONTROL3);
    if (unsigned_) ctrl3 |=  0x08;
    else           ctrl3 &= ~0x08;
    drv_write(tile, DRV2605L_REG_CONTROL3, ctrl3);

    /* CONTROL2 bit 7 = BIDIR_INPUT (1 = bidirectional, 0 = unidirectional). */
    uint8_t ctrl2 = drv_read(tile, DRV2605L_REG_CONTROL2);
    if (bidir) ctrl2 |=  0x80;
    else       ctrl2 &= ~0x80;
    drv_write(tile, DRV2605L_REG_CONTROL2, ctrl2);
}

/* -------------------------------------------------------------- */
/* PWM / analog input modes                                        */
/* -------------------------------------------------------------- */

void tile_drive_h_pwm_input_start(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "pwm_input_start: not ready");
        return;
    }

    /* Clear N_PWM_ANALOG (bit 1) — 0 = PWM input.
     * Also clear AC_COUPLE in CONTROL1 (bit 5) — disables analog bias. */
    uint8_t ctrl3 = drv_read(tile, DRV2605L_REG_CONTROL3);
    ctrl3 &= ~0x02;
    drv_write(tile, DRV2605L_REG_CONTROL3, ctrl3);

    uint8_t ctrl1 = drv_read(tile, DRV2605L_REG_CONTROL1);
    ctrl1 &= ~0x20;
    drv_write(tile, DRV2605L_REG_CONTROL1, ctrl1);

    drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_PWM_ANALOG);
}

void tile_drive_h_analog_input_start(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "analog_input_start: not ready");
        return;
    }

    /* Set N_PWM_ANALOG (bit 1) — 1 = analog input. */
    uint8_t ctrl3 = drv_read(tile, DRV2605L_REG_CONTROL3);
    ctrl3 |= 0x02;
    drv_write(tile, DRV2605L_REG_CONTROL3, ctrl3);

    /* AC_COUPLE stays cleared for DC-coupled analog (the user can set
     * audio mode separately if AC coupling is needed). */
    uint8_t ctrl1 = drv_read(tile, DRV2605L_REG_CONTROL1);
    ctrl1 &= ~0x20;
    drv_write(tile, DRV2605L_REG_CONTROL1, ctrl1);

    drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_PWM_ANALOG);
}

void tile_drive_h_pwm_input_stop(tile_t* tile)
{
    /* Return to internal trigger. AC_COUPLE left wherever audio_start
     * put it; pwm_input_start clears it on next entry. */
    drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_INTERNAL_TRIG);
}

/* -------------------------------------------------------------- */
/* Audio-to-vibe                                                   */
/* -------------------------------------------------------------- */

void tile_drive_h_audio_start(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "audio_start: not ready");
        return;
    }

    /* Audio-to-vibe needs N_PWM_ANALOG=1 (analog) AND AC_COUPLE=1. */
    uint8_t ctrl3 = drv_read(tile, DRV2605L_REG_CONTROL3);
    ctrl3 |= 0x02;
    drv_write(tile, DRV2605L_REG_CONTROL3, ctrl3);

    uint8_t ctrl1 = drv_read(tile, DRV2605L_REG_CONTROL1);
    ctrl1 |= 0x20;
    drv_write(tile, DRV2605L_REG_CONTROL1, ctrl1);

    drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_AUDIO);
}

void tile_drive_h_set_audio_params(tile_t* tile,
                                   uint8_t peak_time,
                                   uint8_t filter,
                                   uint8_t min_input,
                                   uint8_t max_input,
                                   uint8_t min_drive,
                                   uint8_t max_drive)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "set_audio_params: not ready");
        return;
    }

    if (peak_time != 0xFF || filter != 0xFF) {
        uint8_t atv = drv_read(tile, DRV2605L_REG_ATV_CTRL);
        if (peak_time != 0xFF) {
            atv = (atv & ~0x0C) | ((peak_time & 0x03) << 2);
        }
        if (filter != 0xFF) {
            atv = (atv & ~0x03) | (filter & 0x03);
        }
        drv_write(tile, DRV2605L_REG_ATV_CTRL, atv);
    }
    if (min_input != 0xFF) {
        drv_write(tile, DRV2605L_REG_ATV_MIN_INPUT, min_input);
    }
    if (max_input != 0xFF) {
        drv_write(tile, DRV2605L_REG_ATV_MAX_INPUT, max_input);
    }
    if (min_drive != 0xFF) {
        drv_write(tile, DRV2605L_REG_ATV_MIN_DRIVE, min_drive);
    }
    if (max_drive != 0xFF) {
        drv_write(tile, DRV2605L_REG_ATV_MAX_DRIVE, max_drive);
    }
}

void tile_drive_h_audio_stop(tile_t* tile)
{
    /* Clear AC_COUPLE so subsequent PWM/analog modes start clean. */
    uint8_t ctrl1 = drv_read(tile, DRV2605L_REG_CONTROL1);
    ctrl1 &= ~0x20;
    drv_write(tile, DRV2605L_REG_CONTROL1, ctrl1);

    drv_write(tile, DRV2605L_REG_MODE, DRV2605L_MODE_INTERNAL_TRIG);
}

/* -------------------------------------------------------------- */
/* OTP programming (DESTRUCTIVE — no @studio expose)              */
/* -------------------------------------------------------------- */

uint8_t tile_drive_h_get_otp_status(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_SLEEPING) {
        return 0;
    }
    /* CONTROL4 bit 2 = OTP_STATUS (read-only). */
    return (drv_read(tile, DRV2605L_REG_CONTROL4) & 0x04) ? 1 : 0;
}

uint8_t tile_drive_h_program_otp(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "program_otp: not ready");
        return 0;
    }

    /* Refuse if already programmed — a second burn does nothing useful
     * and would confuse the caller. */
    uint8_t ctrl4 = drv_read(tile, DRV2605L_REG_CONTROL4);
    if (ctrl4 & 0x04) {
        TILE_ON_ERROR(tile, "program_otp: OTP already programmed");
        return 0;
    }

    /* Set OTP_PROGRAM bit (CONTROL4 bit 0). The chip latches values
     * 0x16..0x1A into OTP cells. Datasheet 7.5.7: VDD must be 4.0–4.4 V
     * during this call — we cannot enforce that from firmware. */
    drv_write(tile, DRV2605L_REG_CONTROL4, ctrl4 | 0x01);

    /* Programming completes within a few ms; wait generously. */
    tile->hal->delay_ms(50);

    /* Verify by reading OTP_STATUS. */
    ctrl4 = drv_read(tile, DRV2605L_REG_CONTROL4);
    return (ctrl4 & 0x04) ? 1 : 0;
}

/* ============================================================== */
/* Runtime — tier-2 idiomatic helpers                              */
/* ============================================================== */

void tile_drive_h_play_click(tile_t* tile)
{
    /* ROM library effect 1: "Strong Click — 100%". */
    tile_drive_h_play(tile, 1, 1);
}

void tile_drive_h_play_double_tap(tile_t* tile)
{
    /* ROM library effect 10: "Double Click — 100%". The chip plays
     * the two clicks back-to-back from a single sequencer slot;
     * second slot terminates the sequence. */
    static const uint8_t seq[2] = { 10, 0 };
    tile_drive_h_play_sequence(tile, seq, 2);
}

void tile_drive_h_play_alert(tile_t* tile)
{
    /* Strong Buzz (14) → Pulsing Sharp 1 (56) → Strong Buzz (14). */
    static const uint8_t seq[4] = { 14, 56, 14, 0 };
    tile_drive_h_play_sequence(tile, seq, 4);
}

void tile_drive_h_play_buzz(tile_t* tile, uint16_t ms)
{
    if (tile->state != TILE_STATE_READY) {
        TILE_ON_ERROR(tile, "play_buzz: not ready");
        return;
    }

    tile_drive_h_rtp_start(tile);
    tile_drive_h_rtp_write(tile, 0x7F);  /* full-scale amplitude */
    tile->hal->delay_ms(ms);
    tile_drive_h_rtp_write(tile, 0x00);
    tile_drive_h_rtp_stop(tile);
}

uint8_t tile_drive_h_is_calibrated(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) return 0;
    /* DIAG_RESULT is set by the cal engine: 0 = converged, 1 = failed. */
    uint8_t status = drv_read(tile, DRV2605L_REG_STATUS);
    return (status & DRV2605L_STATUS_DIAG_RESULT) ? 0 : 1;
}
