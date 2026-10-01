/**
 * hw-sense-cap -- touch-event bench for the Sense.CAP tile (driver v1.0).
 *
 * Board: Core.ST.L4 + Sense.CAP rev a (IQS7211A) + an electrode surface.
 * I2C1 on Core pads 4/5. Polled (rev a has no RDY); Core pad 3 drives the
 * chip's MCLR (tile pad 3) open-drain, idle released.
 *
 * Streams machine-parseable lines for visualizer_web.py:
 *   S,<layout>,<n_rx>,<n_tx>,<x_res>,<y_res>,<switch_xy>          surface, ~0.5 Hz
 *   D,<t_ms>,<nf>,<x>,<y>,<strength>,<zone>,<touchbits>,<d0..dN-1>,<c0..cN-1>  ~25 Hz
 *   E,<name>,<a>,<b>,<c>                                           events
 * Serial commands:
 *   r        re-ATI (re-baseline in the current mechanical configuration)
 *   s1..s5   sensitivity, least (1) to most (5) sensitive, relative to the layout
 *   h1..h5   stickiness: set/clear multipliers {16,10} {12,6} {8,4} {8,2} {6,1}
 *            (a wider set/clear gap keeps a latched finger through a signal dip)
 *   l0..l4   layout: none, grid 2x3, buttons 2x3, slider 1x3, slider 1x4
 *   x        hardware reset through MCLR
 *
 * Run `python3 visualizer_web.py` and open http://localhost:8765 (surface
 * view) or /slider (scroll view).
 */

#include "core.h"
#include "core_led.h"
#include "core_pad.h"
#include "core_usb.h"
#include "core_watchdog.h"
#include "core_timing.h"
#include "core_tiles.h"
#include "tile_sense_cap.h"

#define MCLR_PAD  3

static tile_t pad;
static uint8_t layout = SENSE_CAP_LAYOUT_GRID_2X3;
static uint8_t ati_running;

static uint32_t clk(void *ctx) { (void)ctx; return core_millis(); }

static void mclr(void *ctx, uint8_t level)
{
    (void)ctx;
    core_pad_write(MCLR_PAD, level ? 1 : 0);   /* open-drain: 0 pulls low */
}

static void on_touch_ev(tile_t *t, const sense_cap_touch_t *ev, void *ctx)
{
    (void)t; (void)ctx;
    if (ev->phase == SENSE_CAP_TOUCH_DOWN)
        core_usb_printf("E,DOWN,%u,%u,%u\r\n", ev->finger, ev->x, ev->y);
    else if (ev->phase == SENSE_CAP_TOUCH_UP)
        core_usb_printf("E,UP,%u,%u,%u\r\n", ev->finger, ev->x, ev->y);
}

static void on_tap_ev(tile_t *t, uint8_t taps, uint16_t x, uint16_t y, void *ctx)
{
    (void)t; (void)ctx;
    core_usb_printf("E,%s,%d,%u,%u\r\n", taps == 2 ? "DTAP" : "TAP",
                    tile_sense_cap_zone_at(&pad, x, y), x, y);
}

static void on_hold_ev(tile_t *t, uint16_t x, uint16_t y, void *ctx)
{
    (void)t; (void)ctx;
    core_usb_printf("E,HOLD,0,%u,%u\r\n", x, y);
}

static void on_swipe_ev(tile_t *t, uint8_t dir, int16_t vx, int16_t vy, void *ctx)
{
    (void)t; (void)ctx;
    static const char *names[] = { "LEFT", "RIGHT", "UP", "DOWN" };
    core_usb_printf("E,SWIPE_%s,%d,%d,0\r\n", names[dir & 3], vx, vy);
}

static void on_pinch_ev(tile_t *t, int16_t delta, void *ctx)
{
    (void)t; (void)ctx;
    core_usb_printf("E,PINCH,%d,0,0\r\n", delta);
}

static void on_event(tile_t *t, uint8_t events, void *ctx)
{
    (void)t; (void)ctx;
    if (events & SENSE_CAP_EV_RESET)
        core_usb_printf("E,CHIP_RESET,0,0,0\r\n");
}

static void print_surface(void)
{
    sense_cap_surface_t s;
    if (tile_sense_cap_layout_preset((sense_cap_layout_t)layout, &s) != SENSE_CAP_OK) return;
    uint8_t nx = s.geometry.switch_xy ? s.geometry.n_tx : s.geometry.n_rx;
    uint8_t ny = s.geometry.switch_xy ? s.geometry.n_rx : s.geometry.n_tx;
    core_usb_printf("S,%u,%u,%u,%u,%u,%u\r\n", layout, s.geometry.n_rx, s.geometry.n_tx,
                    nx > 1 ? (nx - 1) * 256 : 0, ny > 1 ? (ny - 1) * 256 : 0,
                    s.geometry.switch_xy);
}

/* Finish the surface job, feeding the watchdog; report the verdict. */
static void finish_surface(const char *what)
{
    sense_cap_result_t r;
    uint32_t t0 = core_millis();
    while ((r = tile_sense_cap_surface_poll(&pad)) == SENSE_CAP_BUSY) {
        core_watchdog_feed();
        core_delay_ms(2);
    }
    core_usb_printf("E,SURFACE_%s,%d,%lu,0\r\n", what, (int)r,
                    (unsigned long)(core_millis() - t0));
    print_surface();
}

static uint8_t next_digit(void)
{
    uint32_t t0 = core_millis();
    while (!core_usb_available()) {
        if (core_millis() - t0 > 500) return 0xFF;
        core_delay_ms(1);
    }
    return (uint8_t)(core_usb_getc() - '0');
}

int main(void)
{
    core_init();
    core_led_heartbeat(1000, 2);
    core_usb_init();
    core_delay_ms(1500);   /* CDC eats early prints */

    core_usb_printf("\r\n=== hw-sense-cap: touch-event bench (driver v%u.%u.%u) ===\r\n",
                    TILE_SENSE_CAP_VERSION_MAJOR, TILE_SENSE_CAP_VERSION_MINOR,
                    TILE_SENSE_CAP_VERSION_PATCH);

    sense_cap_cfg_t cfg = {
        .on_event = on_event,
        .millis   = clk,
        .mclr     = mclr,
        .layout   = SENSE_CAP_LAYOUT_GRID_2X3,   /* the bench-tuned 2x3 recipe */
        .reset_on_init = 1,                      /* known chip state at every boot */
    };
    tile_sense_cap_init(core_tiles_pal(&core_i2c1), 0, &pad, &cfg);

    if (!tile_is_ready(&pad)) {
        core_usb_printf("IQS7211A: NOT FOUND on I2C1 (0x56)\r\nFAIL\r\n");
        while (1) { core_watchdog_feed(); core_delay_ms(500); }
    }
    core_usb_printf("IQS7211A fw %u.%u\r\n", tile_sense_cap_get_version_major(&pad),
                    tile_sense_cap_get_version_minor(&pad));
    finish_surface("GRID_2X3");   /* init() started the baseline + layout job */
    core_usb_printf("%s\r\n", tile_sense_cap_is_surface_ready(&pad) ? "PASS" : "FAIL (ATI)");

    tile_sense_cap_on_touch(&pad, on_touch_ev, NULL);
    tile_sense_cap_on_tap(&pad, on_tap_ev, NULL);
    tile_sense_cap_on_long_press(&pad, on_hold_ev, NULL);
    tile_sense_cap_on_swipe(&pad, on_swipe_ev, NULL);
    tile_sense_cap_on_pinch(&pad, on_pinch_ev, NULL);

    uint32_t last_frame = 0, last_surface = 0;
    while (1) {
        core_watchdog_feed();
        (void)tile_sense_cap_process(&pad);

        if (ati_running) {
            sense_cap_result_t r = tile_sense_cap_ati_poll(&pad);
            if (r != SENSE_CAP_BUSY) {
                ati_running = 0;
                core_usb_printf("E,REATI_%s,%d,0,0\r\n", r == SENSE_CAP_OK ? "OK" : "FAIL", (int)r);
            }
        }

        while (core_usb_available()) {
            uint8_t ch = core_usb_getc();
            if (ch == 'r') {
                if (tile_sense_cap_ati_start(&pad, SENSE_CAP_ATI_TRACKPAD) == SENSE_CAP_OK) {
                    ati_running = 1;
                    core_usb_printf("E,REATI_START,0,0,0\r\n");
                }
            } else if (ch == 's') {
                uint8_t lvl = next_digit();
                if (lvl >= 1 && lvl <= 5) {
                    uint8_t ok = tile_sense_cap_set_sensitivity(&pad, (sense_cap_sensitivity_t)(lvl - 1));
                    core_usb_printf("E,SENS,%u,%u,0\r\n", lvl, ok);
                }
            } else if (ch == 'h') {
                static const uint8_t hyst[5][2] = {
                    { 16, 10 }, { 12, 6 }, { 8, 4 }, { 8, 2 }, { 6, 1 },
                };
                uint8_t lvl = next_digit();
                if (lvl >= 1 && lvl <= 5) {
                    uint8_t ok = tile_sense_cap_set_touch_multipliers(&pad, hyst[lvl - 1][0],
                                                                      hyst[lvl - 1][1]);
                    core_usb_printf("E,HYST,%u,%u,%u\r\n", lvl, hyst[lvl - 1][0],
                                    ok ? hyst[lvl - 1][1] : 0xFFu);
                }
            } else if (ch == 'l') {
                uint8_t l = next_digit();
                if (l <= SENSE_CAP_LAYOUT_SLIDER_1X4 &&
                    tile_sense_cap_set_layout(&pad, (sense_cap_layout_t)l) == SENSE_CAP_OK) {
                    layout = l;
                    finish_surface("LAYOUT");
                }
            } else if (ch == 'x') {
                sense_cap_result_t r = tile_sense_cap_hw_reset(&pad);
                core_usb_printf("E,MCLR,%d,0,0\r\n", (int)r);
                finish_surface("REAPPLY");
            }
        }

        uint32_t now = core_millis();
        if (now - last_surface >= 2000) {
            last_surface = now;
            print_surface();
        }
        if (now - last_frame >= 40 && !ati_running) {
            last_frame = now;
            uint8_t n = tile_sense_cap_get_num_channels(&pad);
            uint16_t counts[SENSE_CAP_MAX_CHANNELS] = { 0 };
            int16_t deltas[SENSE_CAP_MAX_CHANNELS] = { 0 };
            if (n > SENSE_CAP_MAX_CHANNELS) n = SENSE_CAP_MAX_CHANNELS;
            if (n) {
                (void)tile_sense_cap_read_channel_deltas(&pad, deltas, n);
                (void)tile_sense_cap_read_channel_counts(&pad, counts, n);
            }
            core_usb_printf("D,%lu,%u,%u,%u,%u,%d,%02X",
                            (unsigned long)now, tile_sense_cap_get_num_fingers(&pad),
                            tile_sense_cap_get_finger_x(&pad, 0),
                            tile_sense_cap_get_finger_y(&pad, 0),
                            tile_sense_cap_get_finger_strength(&pad, 0),
                            tile_sense_cap_get_zone(&pad),
                            (unsigned)(tile_sense_cap_get_touch_status(&pad) & 0x3F));
            for (uint8_t c = 0; c < n; c++) core_usb_printf(",%d", deltas[c]);
            for (uint8_t c = 0; c < n; c++) core_usb_printf(",%u", counts[c]);
            core_usb_printf("\r\n");
        }
        core_delay_ms(2);
    }
}
