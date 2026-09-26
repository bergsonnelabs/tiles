/**
 * val-drive-a-2 — Compile-only validation for the Drive.A.2 tile driver.
 *
 * Exercises every public API function in tile_drive_a_2.h to verify
 * compilation. Does not require hardware — all results are cast to void.
 *
 * Core.ST.L4.1, clock=max, I2C1 at 400 kHz
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_drive_a_2.h"

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_t dac;

    /* ---- Lifecycle ---- */
    uint8_t found = tile_drive_a_2_find(hal, 0);
    (void)found;

    /* Init with defaults */
    tile_drive_a_2_init(hal, 0, &dac, NULL);

    /* Init with explicit config */
    drive_a_2_cfg_t cfg = {
        .gain        = DRIVE_A_2_GAIN_1X_VDD,
        .amp_gain_db = 6,
        .vdd_mv      = 5000,
    };
    tile_drive_a_2_init(hal, 0, &dac, &cfg);

    tile_drive_a_2_sleep(&dac);
    tile_drive_a_2_wake(&dac);
    tile_drive_a_2_reset(&dac);

    /* Re-init after reset */
    tile_drive_a_2_init(hal, 0, &dac, NULL);

    /* ---- DAC output ---- */
    tile_drive_a_2_set(&dac, 0, 2048);
    tile_drive_a_2_set(&dac, 1, 4095);

    tile_drive_a_2_set_mv(&dac, 0, 1650);
    tile_drive_a_2_set_mv(&dac, 1, 3000);

    uint16_t code = tile_drive_a_2_get(&dac, 0);
    (void)code;

    /* ---- DAC configuration ---- */
    tile_drive_a_2_set_gain(&dac, 0, DRIVE_A_2_GAIN_1X_EXT);
    tile_drive_a_2_set_gain(&dac, 0, DRIVE_A_2_GAIN_1X_VDD);
    tile_drive_a_2_set_gain(&dac, 0, DRIVE_A_2_GAIN_1P5X_INT);
    tile_drive_a_2_set_gain(&dac, 0, DRIVE_A_2_GAIN_2X_INT);
    tile_drive_a_2_set_gain(&dac, 1, DRIVE_A_2_GAIN_3X_INT);
    tile_drive_a_2_set_gain(&dac, 1, DRIVE_A_2_GAIN_4X_INT);

    /* v3.3: V+ drives set_mv's full scale in the 1x gains */
    tile_drive_a_2_set_supply_mv(&dac, 5000);
    tile_drive_a_2_set_mv(&dac, 0, 2500);
    tile_drive_a_2_set_supply_mv(&dac, DRIVE_A_2_VDD_DEFAULT_MV);

    /* ---- Waveform generation ---- */
    tile_drive_a_2_set_waveform(&dac, 0, DRIVE_A_2_WAVE_TRIANGLE);
    tile_drive_a_2_set_waveform(&dac, 0, DRIVE_A_2_WAVE_SAWTOOTH);
    tile_drive_a_2_set_waveform(&dac, 0, DRIVE_A_2_WAVE_INV_SAW);
    tile_drive_a_2_set_waveform(&dac, 1, DRIVE_A_2_WAVE_SINE);
    tile_drive_a_2_start_waveform(&dac, 0);
    tile_drive_a_2_start_waveform(&dac, 1);
    tile_drive_a_2_stop_waveform(&dac, 0);
    tile_drive_a_2_stop_waveform(&dac, 1);

    /* ---- Slew / margin / phase / waveform synthesis ---- */
    tile_drive_a_2_set_slew_rate(&dac, 0, DRIVE_A_2_SLEW_NONE);
    tile_drive_a_2_set_slew_rate(&dac, 0, DRIVE_A_2_SLEW_4_US);
    tile_drive_a_2_set_slew_rate(&dac, 1, DRIVE_A_2_SLEW_5128_US);

    tile_drive_a_2_set_code_step(&dac, 0, DRIVE_A_2_STEP_1_LSB);
    tile_drive_a_2_set_code_step(&dac, 1, DRIVE_A_2_STEP_32_LSB);

    tile_drive_a_2_set_margins(&dac, 0, 1024, 3072);
    tile_drive_a_2_set_margins(&dac, 1, 0, 4095);
    /* Caller passing low > high should be silently swapped */
    tile_drive_a_2_set_margins(&dac, 0, 3000, 1000);

    tile_drive_a_2_set_phase(&dac, 0, DRIVE_A_2_PHASE_0);
    tile_drive_a_2_set_phase(&dac, 1, DRIVE_A_2_PHASE_90);
    tile_drive_a_2_set_phase(&dac, 1, DRIVE_A_2_PHASE_120);
    tile_drive_a_2_set_phase(&dac, 1, DRIVE_A_2_PHASE_240);

    tile_drive_a_2_set_waveform_params(&dac, 0,
                                       DRIVE_A_2_WAVE_SINE,
                                       DRIVE_A_2_STEP_1_LSB,
                                       DRIVE_A_2_SLEW_4_US);
    tile_drive_a_2_set_waveform_params(&dac, 1,
                                       DRIVE_A_2_WAVE_TRIANGLE,
                                       DRIVE_A_2_STEP_8_LSB,
                                       DRIVE_A_2_SLEW_27_US);

    /* ---- Amplifier control ---- */
    tile_drive_a_2_amp_set_gain(&dac, 12);
    tile_drive_a_2_amp_set_gain(&dac, -10);
    tile_drive_a_2_amp_set_gain(&dac, 0);

    int8_t amp_gain = tile_drive_a_2_amp_get_gain(&dac);
    (void)amp_gain;

    tile_drive_a_2_amp_enable(&dac);
    tile_drive_a_2_amp_disable(&dac);

    drive_a_2_agc_cfg_t agc = {
        .compression   = DRIVE_A_2_COMP_4_1,
        .fixed_gain_db = 6,
        .max_gain_db   = 30,
        .limiter_level = 26,
        .attack        = 5,
        .release       = 11,
        .hold          = 0,
        .noise_gate    = 1,
        .limiter_disable = 0,
    };
    tile_drive_a_2_amp_set_agc(&dac, &agc);

    /* v3.3: output limiter (on by default after init) */
    tile_drive_a_2_amp_set_limiter(&dac, 1, DRIVE_A_2_LIMITER_DEFAULT);
    tile_drive_a_2_amp_set_limiter(&dac, 1, DRIVE_A_2_LIMITER_MAX);
    tile_drive_a_2_amp_set_limiter(&dac, 0, 0);  /* unsafe; honoured at 1:1 only */

    uint8_t amp_status = tile_drive_a_2_amp_read_status(&dac);
    (void)amp_status;

    /* ---- Status ---- */
    uint16_t dac_status = tile_drive_a_2_read_status(&dac);
    (void)dac_status;

    /* ---- NVM (shadow flash) ---- */
    /* nvm_save is destructive — don't actually call it on hardware,
     * but verify the symbol resolves via a taken pointer. */
    void (*save_fp)(tile_t *) = tile_drive_a_2_nvm_save;
    (void)save_fp;
    tile_drive_a_2_nvm_reload(&dac);

    /* ---- Raw register escape hatches ---- */
    uint16_t raw = tile_drive_a_2_read_reg(&dac, 0x22);  /* GENERAL-STATUS */
    (void)raw;
    tile_drive_a_2_write_reg(&dac, 0x1F, 0x0201);        /* COMMON-CONFIG default */

    /* ---- v3.1 tier-2 runtime helpers ---- */
    tile_drive_a_2_play_tone(&dac, DRIVE_A_2_CH_LEFT, 1000, 100);
    tile_drive_a_2_play_silence(&dac, DRIVE_A_2_CH_BOTH, 50);
    tile_drive_a_2_play_chirp(&dac, DRIVE_A_2_CH_RIGHT, 200, 2000, 250);
    tile_drive_a_2_set_volume_pct(&dac, DRIVE_A_2_CH_BOTH, 75);
    tile_drive_a_2_mute(&dac, DRIVE_A_2_CH_LEFT);
    tile_drive_a_2_unmute(&dac, DRIVE_A_2_CH_LEFT);

    /* ---- State checks ---- */
    uint8_t ready = tile_is_ready(&dac);
    (void)ready;

    tile_state_t state = tile_state(&dac);
    (void)state;

    while (1) {
        core_delay_ms(1000);
    }
}
