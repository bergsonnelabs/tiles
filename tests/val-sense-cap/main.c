/**
 * val-sense-cap -- Compile-only validation for the Sense.CAP tile driver.
 *
 * Exercises every public API function in tile_sense_cap.h to verify
 * compilation. Does not require hardware -- all results are cast to void.
 *
 * Core.ST.L4.1, clock=max, I2C1 at 400 kHz, RDY on pad 3
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_sense_cap.h"

static void on_pad_event(tile_t *tile, uint16_t info, void *ctx)
{
    (void)tile; (void)info; (void)ctx;
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

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_t pad;

    /* ---- Lifecycle ---- */

    uint8_t found = tile_sense_cap_find(hal, 0);
    (void)found;

    /* Init with defaults */
    tile_sense_cap_init(hal, 0, &pad, NULL);

    /* Init with config */
    sense_cap_cfg_t cfg = {
        .rdy_pin        = 3,
        .on_event       = on_pad_event,
        .event_ctx      = NULL,
        .gestures       = SENSE_CAP_GESTURE_ALL,
        .active_rate_ms = 10,
        .max_touches    = 2,
        .event_mode     = 0,
        .millis         = val_millis,
    };
    tile_sense_cap_init(hal, 0, &pad, &cfg);

    /* ---- Surface configuration ---- */

    uint8_t ok = tile_sense_cap_setup_2x3(&pad);
    (void)ok;

    sense_cap_surface_t surf = sense_cap_surface_2x3;
    surf.flip_x = 1;
    surf.ati_base = 0x023F;
    surf.touch_set_mult = 10;
    surf.touch_clear_mult = 6;
    ok = tile_sense_cap_configure_surface(&pad, &surf);
    (void)ok;

    /* ---- Touch events and zones ---- */

    tile_sense_cap_on_touch(&pad, on_touch_ev, NULL);
    tile_sense_cap_on_tap(&pad, on_tap_ev, NULL);
    tile_sense_cap_on_long_press(&pad, on_hold_ev, NULL);
    tile_sense_cap_on_swipe(&pad, on_swipe_ev, NULL);
    tile_sense_cap_on_drag(&pad, on_drag_ev, NULL);
    tile_sense_cap_on_pinch(&pad, on_pinch_ev, NULL);

    sense_cap_touch_t tev;
    uint8_t has_ev = tile_sense_cap_next_touch_event(&pad, &tev);
    (void)has_ev;

    uint16_t evbits = tile_sense_cap_get_touch_events(&pad);
    (void)evbits;
    uint8_t tapped = tile_sense_cap_was_tapped(&pad);
    (void)tapped;

    int8_t zone = tile_sense_cap_get_zone(&pad);
    (void)zone;
    zone = tile_sense_cap_zone_at(&pad, 256, 128);
    (void)zone;
    uint8_t zt = tile_sense_cap_is_zone_touched(&pad, 0);
    (void)zt;

    int32_t xp = 0, yp = 0;
    tile_sense_cap_get_position_pct(&pad, &xp, &yp);
    (void)xp; (void)yp;

    uint8_t got = tile_sense_cap_wait_for_touch(&pad, 10);
    (void)got;

    tile_sense_cap_set_sensitivity(&pad, 3);

    /* ---- Power management ---- */

    tile_sense_cap_sleep(&pad);
    tile_sense_cap_wake(&pad);
    tile_sense_cap_reset(&pad);
    tile_sense_cap_ack_reset(&pad);

    /* ---- Event processing ---- */

    tile_sense_cap_process(&pad);
    tile_sense_cap_on_event(&pad, on_pad_event, NULL);

    /* ---- Identification ---- */

    uint16_t v = tile_sense_cap_get_version_major(&pad);
    v = tile_sense_cap_get_version_minor(&pad);
    v = tile_sense_cap_get_settings_version(&pad);
    (void)v;

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
    (void)x; (void)y; (void)st; (void)ar;

    /* ---- Channels ---- */

    uint32_t touch = tile_sense_cap_get_touch_status(&pad);
    (void)touch;

    uint8_t t = tile_sense_cap_is_touched(&pad, 0);
    (void)t;

    uint8_t n = tile_sense_cap_get_num_channels(&pad);
    (void)n;

    uint16_t cnt = tile_sense_cap_get_channel_count(&pad, 0);
    uint16_t dlt = tile_sense_cap_get_channel_delta(&pad, 5);
    (void)cnt; (void)dlt;

    uint16_t counts[6], deltas[6];
    uint8_t rc = tile_sense_cap_read_channels(&pad, counts, deltas, 6);
    (void)rc; (void)counts; (void)deltas;

    /* ---- ALP ---- */

    uint8_t alp = tile_sense_cap_is_alp_active(&pad);
    (void)alp;
    uint16_t ac = tile_sense_cap_get_alp_count(&pad);
    uint16_t al = tile_sense_cap_get_alp_lta(&pad);
    (void)ac; (void)al;

    /* ---- Status ---- */

    uint8_t mode = tile_sense_cap_get_mode(&pad);
    (void)mode;
    uint8_t err = tile_sense_cap_has_ati_error(&pad);
    (void)err;

    /* ---- Configuration ---- */

    tile_sense_cap_enable_gestures(&pad, SENSE_CAP_GESTURE_SINGLE_TAP |
                                         SENSE_CAP_GESTURE_SWIPE_X_POS);
    tile_sense_cap_set_tap_timing(&pad, 150, 300);
    tile_sense_cap_set_swipe_timing(&pad, 150, 128, 96);
    tile_sense_cap_set_report_rate(&pad, SENSE_CAP_MODE_ACTIVE, 10);
    uint8_t mt = tile_sense_cap_set_mode_timeout(&pad, SENSE_CAP_MODE_IDLE, 30);
    (void)mt;
    uint8_t pl = tile_sense_cap_set_poll_latency(&pad, 10);
    (void)pl;
    tile_sense_cap_set_max_touches(&pad, 1);
    tile_sense_cap_set_resolution(&pad, 512, 256);
    tile_sense_cap_set_touch_multipliers(&pad, 20, 10);
    tile_sense_cap_set_alp_threshold(&pad, 8);
    tile_sense_cap_set_event_mode(&pad, 0, 0);
    tile_sense_cap_set_watchdog(&pad, 1);

    /* ---- ATI ---- */

    uint8_t ati = tile_sense_cap_re_ati(&pad);
    (void)ati;
    tile_sense_cap_reseed(&pad);

    /* ---- Escape hatch ---- */

    uint16_t r = tile_sense_cap_read_reg(&pad, IQS7211A_REG_INFO_FLAGS);
    tile_sense_cap_write_reg(&pad, IQS7211A_REG_X_RESOLUTION, r);

    while (1) {
        tile_sense_cap_process(&pad);
        core_delay_ms(20);
    }
}
