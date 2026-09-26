/**
 * val-sense-cap -- Compile-only validation for the Sense.CAP tile driver (v1.0).
 *
 * Exercises every public API function in tile_sense_cap.h to verify
 * compilation. Does not require hardware -- all results are cast to void.
 *
 * Core.ST.L4, clock=max, I2C1 at 400 kHz, tile pad 3 (MCLR) on Core
 * pad 3 as an open-drain output, idle released.
 */

#include "core.h"
#include "core_pad.h"
#include "core_tiles.h"
#include "tile_sense_cap.h"

#define MCLR_PAD  3

static void on_pad_event(tile_t *tile, uint8_t events, void *ctx)
{
    (void)tile; (void)events; (void)ctx;
}

static void on_touch_ev(tile_t *t, const sense_cap_touch_t *ev, void *ctx)
{
    (void)t; (void)ev; (void)ctx;
}
static void on_tap_ev(tile_t *t, uint8_t taps, uint16_t x, uint16_t y, void *ctx)
{
    (void)t; (void)taps; (void)x; (void)y; (void)ctx;
}
static void on_hold_ev(tile_t *t, uint16_t x, uint16_t y, void *ctx)
{
    (void)t; (void)x; (void)y; (void)ctx;
}
static void on_swipe_ev(tile_t *t, uint8_t dir, int16_t vx, int16_t vy, void *ctx)
{
    (void)t; (void)dir; (void)vx; (void)vy; (void)ctx;
}
static void on_drag_ev(tile_t *t, int16_t dx, int16_t dy, uint16_t x, uint16_t y,
                       void *ctx)
{
    (void)t; (void)dx; (void)dy; (void)x; (void)y; (void)ctx;
}
static void on_pinch_ev(tile_t *t, int16_t delta, void *ctx)
{
    (void)t; (void)delta; (void)ctx;
}

static uint32_t val_millis(void *ctx)
{
    (void)ctx;
    return core_millis();
}

/* Open-drain pad: 0 pulls MCLR low, 1 releases it to the tile pull-up. */
static void val_mclr(void *ctx, uint8_t level)
{
    (void)ctx;
    core_pad_write(MCLR_PAD, level ? 1 : 0);
}

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_t pad;
    sense_cap_result_t res;
    uint8_t ok;

    /* ---- Lifecycle ---- */

    uint8_t found = tile_sense_cap_find(hal, 0);
    (void)found;

    tile_sense_cap_init(hal, 0, &pad, NULL);

    sense_cap_cfg_t cfg = {
        .on_event      = on_pad_event,
        .event_ctx     = NULL,
        .millis        = val_millis,
        .millis_ctx    = NULL,
        .mclr          = val_mclr,
        .mclr_ctx      = NULL,
        .skip_baseline = 0,
        .layout        = SENSE_CAP_LAYOUT_GRID_2X3,
        .reset_on_init = 1,
    };
    tile_sense_cap_init(hal, 0, &pad, &cfg);
    /* init() returns once the chip answers; the baseline + layout job
     * finishes here, one register transaction per call. */
    while ((res = tile_sense_cap_surface_poll(&pad)) == SENSE_CAP_BUSY) { }

    /* ---- Surface ---- */

    sense_cap_surface_t surf = SENSE_CAP_SURFACE_DEFAULTS;
    surf.geometry.flip_x = 1;
    surf.tuning.touch_set_mult = 4;
    surf.tuning.touch_clear_mult = 2;
    surf.alp.enable = 0;
    res = tile_sense_cap_surface_begin(&pad, &surf);
    while ((res = tile_sense_cap_surface_poll(&pad)) == SENSE_CAP_BUSY) { }
    (void)res;

    res = tile_sense_cap_layout_preset(SENSE_CAP_LAYOUT_SLIDER_1X4, &surf);
    res = tile_sense_cap_configure_surface(&pad, &surf);
    res = tile_sense_cap_set_layout(&pad, SENSE_CAP_LAYOUT_BUTTONS_2X3);
    res = tile_sense_cap_set_layout(&pad, SENSE_CAP_LAYOUT_SLIDER_1X3);
    res = tile_sense_cap_set_layout(&pad, SENSE_CAP_LAYOUT_GRID_2X3);
    ok = tile_sense_cap_is_surface_ready(&pad);

    res = tile_sense_cap_ati_start(&pad, SENSE_CAP_ATI_BOTH);
    res = tile_sense_cap_ati_poll(&pad);
    res = tile_sense_cap_hw_reset(&pad);
    res = tile_sense_cap_reset(&pad);
    (void)res;

    uint16_t v = tile_sense_cap_get_version_major(&pad);
    v = tile_sense_cap_get_version_minor(&pad);
    v = tile_sense_cap_get_settings_version(&pad);
    (void)v;

    /* ---- Events ---- */

    res = tile_sense_cap_process(&pad);
    tile_sense_cap_on_event(&pad, on_pad_event, NULL);
    tile_sense_cap_on_touch(&pad, on_touch_ev, NULL);
    tile_sense_cap_on_tap(&pad, on_tap_ev, NULL);
    tile_sense_cap_on_long_press(&pad, on_hold_ev, NULL);
    tile_sense_cap_on_swipe(&pad, on_swipe_ev, NULL);
    tile_sense_cap_on_drag(&pad, on_drag_ev, NULL);
    tile_sense_cap_on_pinch(&pad, on_pinch_ev, NULL);

    sense_cap_touch_t tev;
    ok = tile_sense_cap_next_touch_event(&pad, &tev);

    uint16_t evbits = tile_sense_cap_get_touch_events(&pad);
    (void)evbits;
    ok = tile_sense_cap_was_tapped(&pad);

    int8_t zone = tile_sense_cap_get_zone(&pad);
    zone = tile_sense_cap_zone_at(&pad, 256, 128);
    (void)zone;
    int16_t pct = tile_sense_cap_get_x_pct(&pad);
    pct = tile_sense_cap_get_y_pct(&pad);
    (void)pct;
    ok = tile_sense_cap_wait_for_touch(&pad, 10);

    /* ---- Trackpad data ---- */

    uint16_t w = tile_sense_cap_get_info_flags(&pad);
    w = tile_sense_cap_get_gestures(&pad);
    (void)w;
    uint8_t fingers = tile_sense_cap_get_num_fingers(&pad);
    (void)fingers;
    uint16_t x = tile_sense_cap_get_finger_x(&pad, 0);
    uint16_t y = tile_sense_cap_get_finger_y(&pad, 0);
    uint16_t st = tile_sense_cap_get_finger_strength(&pad, 0);
    uint16_t ar = tile_sense_cap_get_finger_area(&pad, 1);
    int16_t rx = tile_sense_cap_get_relative_x(&pad);
    int16_t ry = tile_sense_cap_get_relative_y(&pad);
    (void)x; (void)y; (void)st; (void)ar; (void)rx; (void)ry;

    /* ---- Channels ---- */

    uint32_t touch = tile_sense_cap_get_touch_status(&pad);
    (void)touch;
    ok = tile_sense_cap_is_touched(&pad, 0);
    uint8_t n = tile_sense_cap_get_num_channels(&pad);
    (void)n;
    uint16_t cnt = tile_sense_cap_get_channel_count(&pad, 0);
    int16_t dlt = tile_sense_cap_get_channel_delta(&pad, 5);
    (void)cnt; (void)dlt;

    uint16_t counts[6];
    int16_t deltas[6];
    uint8_t got = tile_sense_cap_read_channel_counts(&pad, counts, 6);
    got = tile_sense_cap_read_channel_deltas(&pad, deltas, 6);
    got = tile_sense_cap_read_channels(&pad, counts, deltas, 6);
    (void)got;

    /* ---- ALP / status ---- */

    ok = tile_sense_cap_is_alp_active(&pad);
    uint16_t ac = tile_sense_cap_get_alp_count(&pad);
    uint16_t al = tile_sense_cap_get_alp_lta(&pad);
    (void)ac; (void)al;
    uint8_t mode = tile_sense_cap_get_mode(&pad);
    (void)mode;
    ok = tile_sense_cap_has_ati_error(&pad);

    /* ---- Configuration ---- */

    ok = tile_sense_cap_set_sensitivity(&pad, SENSE_CAP_SENS_HIGH);
    ok = tile_sense_cap_set_touch_multipliers(&pad, 8, 5);
    ok = tile_sense_cap_set_power_profile(&pad, SENSE_CAP_POWER_BALANCED);
    ok = tile_sense_cap_set_power_mode(&pad, SENSE_CAP_POWER_FORCE_ACTIVE);
    ok = tile_sense_cap_set_power_mode(&pad, SENSE_CAP_POWER_AUTO);
    ok = tile_sense_cap_set_active_rate(&pad, 10);
    ok = tile_sense_cap_set_idle_timeout(&pad, 20);
    ok = tile_sense_cap_set_report_rate(&pad, SENSE_CAP_MODE_LP2, 200);
    ok = tile_sense_cap_set_mode_timeout(&pad, SENSE_CAP_MODE_LP1, 30);
    ok = tile_sense_cap_set_alp_wake(&pad, 1);
    ok = tile_sense_cap_set_alp_threshold(&pad, 20);
    ok = tile_sense_cap_set_flip_x(&pad, 0);
    ok = tile_sense_cap_set_flip_y(&pad, 0);
    ok = tile_sense_cap_set_switch_xy(&pad, 0);
    ok = tile_sense_cap_set_resolution(&pad, 0, 0);
    ok = tile_sense_cap_set_tap_time(&pad, 300);
    ok = tile_sense_cap_set_hold_time(&pad, 300);
    ok = tile_sense_cap_set_swipe_time(&pad, 500);
    ok = tile_sense_cap_set_swipe_distance(&pad, 0);
    ok = tile_sense_cap_set_swipe_angle(&pad, 45);
    ok = tile_sense_cap_set_gesture_source(&pad, SENSE_CAP_GESTURES_CHIP);
    ok = tile_sense_cap_enable_gestures(&pad, SENSE_CAP_GESTURE_SINGLE_TAP |
                                              SENSE_CAP_GESTURE_SWIPE_X_POS);
    ok = tile_sense_cap_set_max_touches(&pad, 1);
    ok = tile_sense_cap_set_reference_update(&pad, 10);
    ok = tile_sense_cap_set_i2c_timeout(&pad, 50);
    ok = tile_sense_cap_set_watchdog(&pad, 1);
    ok = tile_sense_cap_reseed(&pad);
    (void)ok;

    /* ---- Escape hatch ---- */

    uint16_t r = tile_sense_cap_read_reg(&pad, IQS7211A_REG_X_RESOLUTION);
    res = tile_sense_cap_write_reg(&pad, IQS7211A_REG_X_RESOLUTION, r);
    (void)res;

    while (1) {
        (void)tile_sense_cap_process(&pad);
        core_delay_ms(10);
    }
}
