/**
 * val-drive-dc-h — Compile-only validation for the Drive.DC.H tile driver.
 *
 * Exercises every public API function in tile_drive_dc_h.h to verify
 * compilation. Does not require hardware — all results are cast to void.
 *
 * Core.ST.L4, clock=max, I2C1 at 400 kHz
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_drive_dc_h.h"

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_t motor;

    /* ---- Lifecycle ---- */
    uint8_t found = tile_drive_dc_h_find(hal, 0);
    (void)found;

    /* Init with defaults */
    tile_drive_dc_h_init(hal, 0, &motor, NULL);

    /* Init with explicit voltage-mode config */
    drive_dc_h_cfg_t cfg_v = {
        .mode    = DRIVE_DC_H_MODE_VOLTAGE,
        .vm_gain = 1,
        .cs_gain = 0,
        .target  = 180,
    };
    tile_drive_dc_h_init(hal, 0, &motor, &cfg_v);

    /* Init with speed-mode config (full motor params) */
    drive_dc_h_cfg_t cfg_s = {
        .mode            = DRIVE_DC_H_MODE_SPEED,
        .vm_gain         = 0,
        .cs_gain         = 2,
        .target          = 128,
        .motor_mohm      = 5000,
        .ripples_per_rev = 12,
        .kv_uv_per_rpm   = 417,
    };
    tile_drive_dc_h_init(hal, 0, &motor, &cfg_s);

    /* Init with ripple-count mode (position tracking) */
    drive_dc_h_cfg_t cfg_r = {
        .mode            = DRIVE_DC_H_MODE_RIPPLE_COUNT,
        .vm_gain         = 1,
        .cs_gain         = 3,
        .target          = 180,
        .motor_mohm      = 10000,
        .ripples_per_rev = 6,
    };
    tile_drive_dc_h_init(hal, 0, &motor, &cfg_r);

    /* Init in pad-control modes (PMODE/I2C_BC paths) */
    drive_dc_h_cfg_t cfg_phen = {
        .mode    = DRIVE_DC_H_MODE_PAD_PHEN,
        .vm_gain = 1,
        .cs_gain = 0,
    };
    tile_drive_dc_h_init(hal, 0, &motor, &cfg_phen);

    drive_dc_h_cfg_t cfg_in1in2 = {
        .mode    = DRIVE_DC_H_MODE_PAD_IN1IN2,
        .vm_gain = 1,
        .cs_gain = 0,
    };
    tile_drive_dc_h_init(hal, 0, &motor, &cfg_in1in2);

    /* Restore I²C-controlled mode for the rest of the test */
    tile_drive_dc_h_init(hal, 0, &motor, NULL);

    /* ---- Motor control ---- */
    tile_drive_dc_h_forward(&motor);
    tile_drive_dc_h_reverse(&motor);
    tile_drive_dc_h_brake(&motor);
    tile_drive_dc_h_coast(&motor);

    /* ---- Bridge control source ---- */
    tile_drive_dc_h_set_control_mode(&motor, DRIVE_DC_H_CTRL_PAD_PHEN);
    tile_drive_dc_h_set_control_mode(&motor, DRIVE_DC_H_CTRL_PAD_IN1IN2);
    tile_drive_dc_h_set_control_mode(&motor, DRIVE_DC_H_CTRL_I2C);

    /* ---- Regulation ---- */
    tile_drive_dc_h_set_target(&motor, 200);
    tile_drive_dc_h_set_target(&motor, 0);

    tile_drive_dc_h_set_regulation_mode(&motor, DRIVE_DC_H_REG_OPEN_LOOP);
    tile_drive_dc_h_set_regulation_mode(&motor, DRIVE_DC_H_REG_CYCLE_BY_CYCLE);
    tile_drive_dc_h_set_regulation_mode(&motor, DRIVE_DC_H_REG_VOLTAGE);
    tile_drive_dc_h_set_regulation_mode(&motor, DRIVE_DC_H_REG_SPEED);

    /* ---- Current regulation ---- */
    tile_drive_dc_h_set_current_regulation_mode(&motor, DRIVE_DC_H_IMODE_DISABLED);
    tile_drive_dc_h_set_current_regulation_mode(&motor, DRIVE_DC_H_IMODE_INRUSH);
    tile_drive_dc_h_set_current_regulation_mode(&motor, DRIVE_DC_H_IMODE_ALWAYS);

    tile_drive_dc_h_set_current_sense_gain(&motor, 0);   /* 4 A */
    tile_drive_dc_h_set_current_sense_gain(&motor, 5);   /* 0.125 A */
    tile_drive_dc_h_set_current_sense_gain(&motor, 99);  /* clamped */

    /* ---- Stall detection tuning ---- */
    tile_drive_dc_h_set_stall_enabled(&motor, 1);
    tile_drive_dc_h_set_stall_enabled(&motor, 0);
    tile_drive_dc_h_set_inrush_time_ms(&motor, 100);
    tile_drive_dc_h_set_inrush_time_ms(&motor, 1500);
    tile_drive_dc_h_set_inrush_time_ms(&motor, 8000);  /* clamps */
    tile_drive_dc_h_set_stall_recovery(&motor, DRIVE_DC_H_STALL_LATCH);
    tile_drive_dc_h_set_stall_recovery(&motor, DRIVE_DC_H_STALL_REPORT);

    /* ---- Ripple counter tuning ---- */
    tile_drive_dc_h_set_ripple_threshold(&motor, 50);     /* small */
    tile_drive_dc_h_set_ripple_threshold(&motor, 5000);   /* mid */
    tile_drive_dc_h_set_ripple_threshold(&motor, 60000);  /* near-max */
    tile_drive_dc_h_set_ripple_threshold(&motor, 65535);  /* clamps */

    tile_drive_dc_h_set_ripple_filter_gain(&motor, 0);   /* x2 */
    tile_drive_dc_h_set_ripple_filter_gain(&motor, 3);   /* x16 */

    /* ---- Monitoring ---- */
    uint8_t fault = tile_drive_dc_h_get_fault(&motor);
    (void)fault;

    tile_drive_dc_h_clear_fault(&motor);

    uint8_t stalled = tile_drive_dc_h_is_stalled(&motor);
    (void)stalled;

    uint16_t mv = tile_drive_dc_h_get_voltage_mv(&motor);
    (void)mv;

    uint16_t ma = tile_drive_dc_h_get_current_ma(&motor);
    (void)ma;

    uint8_t speed = tile_drive_dc_h_get_speed(&motor);
    (void)speed;

    uint16_t ripples = tile_drive_dc_h_get_ripple_count(&motor);
    (void)ripples;

    tile_drive_dc_h_clear_ripple_count(&motor);

    /* ---- Power management ---- */
    tile_drive_dc_h_sleep(&motor);
    tile_drive_dc_h_wake(&motor);

    /* ---- State checks ---- */
    uint8_t ready = tile_is_ready(&motor);
    (void)ready;

    tile_state_t state = tile_state(&motor);
    (void)state;

    /* ---- Fault mask compile check ---- */
    (void)DRV8214_FAULT_FAULT;
    (void)DRV8214_FAULT_STALL;
    (void)DRV8214_FAULT_OCP;
    (void)DRV8214_FAULT_OVP;
    (void)DRV8214_FAULT_TSD;
    (void)DRV8214_FAULT_NPOR;
    (void)DRV8214_FAULT_CNT_DONE;

    /* ---- v4.1 tier-2 runtime helpers ---- */
    tile_drive_dc_h_set_speed_rpm(&motor, 200, DRIVE_DC_H_DIR_FORWARD);
    tile_drive_dc_h_move_distance(&motor, 50, DRIVE_DC_H_DIR_REVERSE);
    uint8_t running = tile_drive_dc_h_is_running(&motor);
    (void)running;
    uint8_t stopped = tile_drive_dc_h_wait_for_stop(&motor, 500);
    (void)stopped;

    /* ---- v4.2 motor profile + RPM readback ---- */
    tile_drive_dc_h_set_motor_params(&motor, 6000, 12, 187);
    uint32_t rpm = tile_drive_dc_h_get_speed_rpm(&motor);
    (void)rpm;

    while (1) {
        core_delay_ms(1000);
    }
}
