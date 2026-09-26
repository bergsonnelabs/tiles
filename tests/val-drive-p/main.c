/**
 * val-drive-p — Compile-only validation for the Drive.P tile driver.
 *
 * Exercises every public API function in tile_drive_p.h to verify
 * compilation. Does not require hardware — all results are cast to void.
 *
 * Core.ST.L4.1, clock=max, I2C1 at 400 kHz
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_drive_p.h"

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_t piezo;

    /* ---- Lifecycle ---- */
    uint8_t found = tile_drive_p_find(hal, 0);
    (void)found;

    tile_drive_p_init(hal, 0, &piezo, NULL);

    /* ---- Mode selection ---- */
    tile_drive_p_set_mode(&piezo, DRIVE_P_MODE_IDLE);
    tile_drive_p_set_mode(&piezo, DRIVE_P_MODE_SENSE_FINE);
    tile_drive_p_set_mode(&piezo, DRIVE_P_MODE_SENSE_COARSE);
    tile_drive_p_set_mode(&piezo, DRIVE_P_MODE_PLAY_DIRECT);
    tile_drive_p_set_mode(&piezo, DRIVE_P_MODE_PLAY_FIFO);
    tile_drive_p_set_mode(&piezo, DRIVE_P_MODE_PLAY_RAM_SYNTH);
    tile_drive_p_set_mode(&piezo, DRIVE_P_MODE_IDLE);

    /* ---- Reads ---- */
    uint16_t raw    = tile_drive_p_read(&piezo);
    int16_t  sense  = tile_drive_p_read_sense(&piezo);
    uint16_t status = tile_drive_p_read_status(&piezo);
    (void)raw; (void)sense; (void)status;

    /* ---- Waveform writes ---- */
    tile_drive_p_write_fifo(&piezo, 0x7FFF);
    tile_drive_p_write_fifo(&piezo, -32768);

    uint16_t wfs[4] = { 0x0014, 0x0000, 0x7FFF, 0x0000 };
    tile_drive_p_wfs_write(&piezo, wfs, 4);

    /* ---- Raw register access (escape hatch) ---- */
    tile_drive_p_write_reg(&piezo, BOS1921_REG_REFERENCE, 0x0000);

    /* ---- v1.1.0 / v3.0 chip-coverage setters ---- */
    tile_drive_p_set_output_range(&piezo, DRIVE_P_OUTPUT_HIGH_V);
    tile_drive_p_set_output_range(&piezo, DRIVE_P_OUTPUT_LOW_V);

    tile_drive_p_set_sense_gain(&piezo, DRIVE_P_SENSE_FINE_GAIN);
    tile_drive_p_set_sense_gain(&piezo, DRIVE_P_SENSE_COARSE_GAIN);

    tile_drive_p_set_sleep_retention(&piezo, 1);
    tile_drive_p_set_sleep_retention(&piezo, 0);

    tile_drive_p_set_auto_sleep(&piezo, 1);
    tile_drive_p_set_auto_sleep(&piezo, 0);

    tile_drive_p_set_upi(&piezo, 1);
    tile_drive_p_set_upi(&piezo, 0);

    /* ---- v3.1 tier-2 runtime helpers ---- */
    tile_drive_p_play_click(&piezo, 80);
    tile_drive_p_play_sine(&piezo, 250, 60, 100);
    tile_drive_p_play_buzz(&piezo, 70, 50);
    tile_drive_p_play_pulse_train(&piezo, 80, 3, 100);

    uint8_t touched = tile_drive_p_is_touched(&piezo, 200);
    (void)touched;

    uint8_t fired = tile_drive_p_play_on_touch(&piezo, 80, 200, /*timeout_ms=*/5);
    (void)fired;

    int16_t custom_samples[16] = {
        0, 256, 512, 768, 1024, 1280, 1536, 1792,
        2047, 1792, 1536, 1280, 1024, 768, 512, 256
    };
    tile_drive_p_play_samples(&piezo, custom_samples, 16);

    int16_t sense_buf[8];
    tile_drive_p_read_sense_samples(&piezo, sense_buf, 8);
    (void)sense_buf[0];

    /* ---- Recovery / sleep / reset ---- */
    uint8_t recovered = tile_drive_p_check_and_recover(&piezo,
                                                      DRIVE_P_MODE_IDLE);
    (void)recovered;

    tile_drive_p_sleep(&piezo);
    uint8_t woke = tile_drive_p_wake(&piezo);      /* v3.5: explicit wake */
    (void)woke;
    tile_drive_p_sleep(&piezo);
    tile_drive_p_play_click(&piezo, 50);            /* v3.5: helpers auto-wake */
    tile_drive_p_reset(&piezo);

    /* ---- State checks ---- */
    uint8_t ready = tile_is_ready(&piezo);
    (void)ready;
    tile_state_t state = tile_state(&piezo);
    (void)state;

    while (1) {
        core_delay_ms(1000);
    }
}
