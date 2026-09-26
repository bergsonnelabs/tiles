/**
 * val-drive-h — Compile-only validation for the Drive.H tile driver.
 *
 * Exercises every public API function in tile_drive_h.h to verify
 * compilation. Does not require hardware — all results are cast to void.
 *
 * Core.ST.L4, clock=max, I2C1 at 400 kHz
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_drive_h.h"

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_t haptic;

    /* ---- Lifecycle ---- */
    uint8_t found = tile_drive_h_find(hal, 0);
    (void)found;

    /* Init with defaults */
    tile_drive_h_init(hal, 0, &haptic, NULL);

    /* Init with explicit config */
    drive_h_cfg_t cfg = {
        .library     = 6,
        .closed_loop = 0,
    };
    tile_drive_h_init(hal, 0, &haptic, &cfg);

    /* ERM config variant */
    drive_h_cfg_t cfg_erm = {
        .library     = 2,
        .closed_loop = 1,
    };
    tile_drive_h_init(hal, 0, &haptic, &cfg_erm);

    /* ---- Single effect playback ---- */
    tile_drive_h_play(&haptic, 1, 1);
    tile_drive_h_play(&haptic, 47, 3);

    /* ---- Sequence playback ---- */
    const uint8_t seq[] = { 1, 12, 47 };
    tile_drive_h_play_sequence(&haptic, seq, 3);

    const uint8_t seq_full[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    tile_drive_h_play_sequence(&haptic, seq_full, 8);

    /* ---- Load sequence (no trigger) ---- */
    const uint8_t preload[] = { 10, 20 };
    tile_drive_h_load_sequence(&haptic, preload, 2);

    /* ---- Trigger modes ---- */
    tile_drive_h_set_trigger(&haptic, DRIVE_H_TRIG_EDGE);
    tile_drive_h_set_trigger(&haptic, DRIVE_H_TRIG_LEVEL);
    tile_drive_h_set_trigger(&haptic, DRIVE_H_TRIG_INTERNAL);

    /* ---- Playback status ---- */
    uint8_t playing = tile_drive_h_is_playing(&haptic);
    (void)playing;

    /* ---- Stop ---- */
    tile_drive_h_stop(&haptic);

    /* ---- RTP mode ---- */
    tile_drive_h_rtp_start(&haptic);
    tile_drive_h_rtp_write(&haptic, 0);
    tile_drive_h_rtp_write(&haptic, 64);
    tile_drive_h_rtp_write(&haptic, 127);
    tile_drive_h_rtp_stop(&haptic);

    /* ---- Status readback ---- */
    uint8_t status = tile_drive_h_get_status(&haptic);
    (void)status;

    /* ---- Diagnostics ---- */
    uint8_t diag = tile_drive_h_diagnose(&haptic);
    (void)diag;

    /* ---- Calibration ---- */
    uint8_t cal = tile_drive_h_calibrate(&haptic);
    (void)cal;

    /* ---- Battery voltage ---- */
    uint16_t vbat = tile_drive_h_get_vbat_mv(&haptic);
    (void)vbat;

    /* ---- Resonance frequency ---- */
    uint16_t res_hz = tile_drive_h_get_resonance_hz(&haptic);
    (void)res_hz;

    /* ---- Sequencer slot wait ---- */
    tile_drive_h_set_sequence_wait(&haptic, 1, 20);   /* 200 ms wait */
    tile_drive_h_set_sequence_wait(&haptic, 9, 20);   /* out-of-range slot, ignored */
    tile_drive_h_set_sequence_wait(&haptic, 2, 0xFF); /* clipped to 0x7F */

    /* ---- Library + actuator tuning runtime setters ---- */
    tile_drive_h_set_library(&haptic, DRIVE_H_LIB_LRA);
    tile_drive_h_set_library(&haptic, DRIVE_H_LIB_ERM_C);
    tile_drive_h_set_library(&haptic, DRIVE_H_LIB_EMPTY);

    tile_drive_h_set_actuator_params(&haptic, 0x1A, 0x25, 2, 1);
    tile_drive_h_set_actuator_params(&haptic, 0,    0,    0xFF, 0xFF); /* no-op */

    tile_drive_h_set_resonance_params(&haptic, 3, 1, 1);
    tile_drive_h_set_resonance_params(&haptic, 0xFF, 0xFF, 0xFF); /* no-op */

    tile_drive_h_set_loop_mode(&haptic, 0);
    tile_drive_h_set_loop_mode(&haptic, 1);

    tile_drive_h_set_brake_factor(&haptic, DRIVE_H_BRAKE_6X);
    tile_drive_h_set_brake_factor(&haptic, DRIVE_H_BRAKE_OFF);
    tile_drive_h_set_loop_gain(&haptic, DRIVE_H_LOOP_GAIN_HIGH);
    tile_drive_h_set_loop_gain(&haptic, DRIVE_H_LOOP_GAIN_MEDIUM);

    tile_drive_h_set_audio_envelope(&haptic, DRIVE_H_ATV_PEAK_20MS, DRIVE_H_ATV_LPF_125HZ);
    tile_drive_h_set_audio_levels(&haptic, 0x19, 0xFF, 0x19, 0xFF); /* reset values */

    tile_drive_h_set_actuator_voltage(&haptic, 1800, 2500);
    tile_drive_h_set_actuator_voltage(&haptic, 0, 9999);    /* clamped */

    tile_drive_h_set_resonance_hz(&haptic, 241);
    tile_drive_h_set_resonance_hz(&haptic, 50);             /* clamped */

    /* ---- Library waveform timing offsets ---- */
    tile_drive_h_set_waveform_timing(&haptic, 0, 0, 0, 0);
    tile_drive_h_set_waveform_timing(&haptic, 4, -2, 1, 8);

    /* ---- RTP format ---- */
    tile_drive_h_set_rtp_format(&haptic, 1, 1);  /* unsigned, bidirectional */
    tile_drive_h_set_rtp_format(&haptic, 0, 0);  /* signed, unidirectional */

    /* ---- PWM / analog input modes ---- */
    tile_drive_h_pwm_input_start(&haptic);
    tile_drive_h_pwm_input_stop(&haptic);

    tile_drive_h_analog_input_start(&haptic);
    tile_drive_h_pwm_input_stop(&haptic);

    /* ---- Audio-to-vibe ---- */
    tile_drive_h_audio_start(&haptic);
    tile_drive_h_set_audio_params(&haptic, 1, 1, 0x19, 0xFF, 0x19, 0xFF);
    tile_drive_h_set_audio_params(&haptic, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
    tile_drive_h_audio_stop(&haptic);

    /* ---- OTP status (read-only is safe; do NOT call program_otp) ---- */
    uint8_t otp_done = tile_drive_h_get_otp_status(&haptic);
    (void)otp_done;

    /* Reference (not invoked) — destructive, C-only. */
    (void)&tile_drive_h_program_otp;

    /* ---- Standby / wake ---- */
    tile_drive_h_standby(&haptic);
    tile_drive_h_wake(&haptic);

    /* ---- v4.1 tier-2 runtime helpers ---- */
    tile_drive_h_play_click(&haptic);
    tile_drive_h_play_double_tap(&haptic);
    tile_drive_h_play_alert(&haptic);
    tile_drive_h_play_buzz(&haptic, 200);
    uint8_t calibrated = tile_drive_h_is_calibrated(&haptic);
    (void)calibrated;

    /* ---- State checks ---- */
    uint8_t ready = tile_is_ready(&haptic);
    (void)ready;

    tile_state_t state = tile_state(&haptic);
    (void)state;

    while (1) {
        core_delay_ms(1000);
    }
}
