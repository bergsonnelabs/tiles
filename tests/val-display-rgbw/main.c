/**
 * val-display-rgbw — Compile-only validation for the Display.RGBW
 * (LP5811) tile driver.
 *
 * Exercises every public API function in tile_display_rgbw.h to verify
 * compilation. Does not require hardware — all results are cast to void.
 *
 * Core.ST.L4, clock=max, I2C1 at 400 kHz
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_display_rgbw.h"

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_t led;

    /* ---- Lifecycle ---- */
    uint8_t found = tile_display_rgbw_find(hal, 0);
    (void)found;

    tile_display_rgbw_init(hal, 0, &led, NULL);

    /* ---- Basic colour + brightness ---- */
    tile_display_rgbw_set(&led, 255, 0, 0, 0);     /* red */
    tile_display_rgbw_set(&led, 0, 255, 0, 0);     /* green */
    tile_display_rgbw_set(&led, 0, 0, 255, 0);     /* blue */
    tile_display_rgbw_set(&led, 0, 0, 0, 128);     /* dim white */
    tile_display_rgbw_off(&led);

    /* ---- Per-channel current ---- */
    tile_display_rgbw_set_current(&led, 0x80, 0x80, 0x80, 0x80);
    tile_display_rgbw_set_current(&led, 0xFF, 0x40, 0x40, 0x40);

    /* ---- Max-current selector (MC bit) ---- */
    tile_display_rgbw_set_max_current(&led, DISP_RGBW_MAX_CURRENT_25_5_MA);
    tile_display_rgbw_set_max_current(&led, DISP_RGBW_MAX_CURRENT_51_MA);

    /* ---- Fault detection ---- */
    disp_rgbw_faults_t faults;
    tile_display_rgbw_read_faults(&led, &faults);
    (void)faults.open_mask;
    (void)faults.short_mask;
    (void)faults.thermal_shutdown;
    (void)faults.config_error;

    tile_display_rgbw_clear_faults(&led);

    tile_display_rgbw_set_short_threshold(&led, DISP_RGBW_LSD_TH_0_35);
    tile_display_rgbw_set_short_threshold(&led, DISP_RGBW_LSD_TH_0_45);
    tile_display_rgbw_set_short_threshold(&led, DISP_RGBW_LSD_TH_0_55);
    tile_display_rgbw_set_short_threshold(&led, DISP_RGBW_LSD_TH_0_65);

    tile_display_rgbw_set_short_shutdown(&led, 1);
    tile_display_rgbw_set_short_shutdown(&led, 0);

    tile_display_rgbw_set_open_shutdown(&led, 0);
    tile_display_rgbw_set_open_shutdown(&led, 1);

    /* ---- Power management ---- */
    tile_display_rgbw_sleep(&led);
    tile_display_rgbw_wake(&led);

    /* Skip reset() in val test — would put driver back in TILE_STATE_NONE
     * for the rest of the run. Keep referenced for compile check. */
    (void)tile_display_rgbw_reset;

    /* ---- v2.1 tier-2 runtime helpers ---- */
    tile_display_rgbw_set_color(&led, 255, 0, 0);
    tile_display_rgbw_pulse(&led, 0, 255, 0, 200);
    tile_display_rgbw_breathe(&led, 0, 0, 255, 1000);
    tile_display_rgbw_flash(&led, 255, 255, 0, 3);
    uint8_t faulted = tile_display_rgbw_is_faulted(&led);
    (void)faulted;

    /* ---- State checks ---- */
    uint8_t ready = tile_is_ready(&led);
    (void)ready;
    tile_state_t state = tile_state(&led);
    (void)state;

    while (1) {
        core_delay_ms(1000);
    }
}
