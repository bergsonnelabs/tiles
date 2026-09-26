/**
 * hw-sense-adc-6 — hardware test for the Sense.ADC.6 driver.
 *
 * A Core.ST.L4 on I2C1 (pads 4/5) exercises every public entry point of
 * tile_sense_adc_6 against a real tile, and reports over USB-CDC.
 *
 * Open the serial port to run the sequence; it repeats if you reopen it.
 *
 * Deliberately not tested here: set_address() and save(), which both write
 * the tile's non-volatile memory. save() is slow enough to hold the bus,
 * and an address change only takes effect at the tile's next power-up, so
 * both belong in a bench procedure rather than an automated run.
 */

#include "core.h"
#include "core_i2c.h"
#include "core_usb.h"
#include "tile_sense_adc_6.h"

#define P core_usb_printf

static tile_t adc;
static unsigned pass_n, fail_n;

static void check(int ok, const char *what)
{
    if (ok) { pass_n++; P("  [PASS] %s\r\n", what); }
    else    { fail_n++; P("  [FAIL] %s\r\n", what); }
}

static void show_channels(void)
{
    P("  mV:");
    for (uint8_t ch = 0; ch < SENSE_ADC_6_CHANNELS; ch++)
        P(" ch%u(pad%u)=%4u", ch, tile_sense_adc_6_channel_pad(ch),
          tile_sense_adc_6_read_mv(&adc, ch));
    P("\r\n");
}

/** Publish rate in hundredths of a hertz, measured through the driver. */
static uint32_t measure_centi_hz(uint32_t ms)
{
    uint32_t t0 = core_millis(), published = 0, elapsed;
    uint8_t prev = 0;
    int have = 0;

    while ((elapsed = core_millis() - t0) < ms) {
        if (tile_sense_adc_6_update(&adc)) {
            uint8_t s = tile_sense_adc_6_sequence(&adc);
            if (have) published += (uint8_t)(s - prev);
            prev = s;
            have = 1;
        }
        core_delay_ms(3);
    }
    return elapsed ? (uint32_t)((uint64_t)published * 100000u / elapsed) : 0;
}

static void report_rate(const char *label, uint32_t want_hz)
{
    uint32_t c = measure_centi_hz(3000);
    P("  %s: %lu.%02lu Hz\r\n", label, c / 100, c % 100);
    check(c >= want_hz * 98 && c <= want_hz * 102, "rate within 2% of request");
}

static void run_sequence(void)
{
    pass_n = fail_n = 0;
    P("\r\n=== hw-sense-adc-6: driver against a real tile ===\r\n");

    /* ---- find / init ---- */
    P("\r\n--- lifecycle ---\r\n");
    check(tile_sense_adc_6_find(core_tiles_pal(&core_i2c1), 0), "find() locates the tile");

    tile_sense_adc_6_init(core_tiles_pal(&core_i2c1), 0, &adc, NULL);
    check(tile_is_ready(&adc), "init() leaves the tile ready");
    if (!tile_is_ready(&adc)) { P("  stopping\r\n"); return; }

    /* find() must reject an instance that isn't there, rather than
     * claiming whatever answers on another address. */
    check(!tile_sense_adc_6_find(core_tiles_pal(&core_i2c1), 3), "find() rejects an absent instance");

    /* ---- runtime ---- */
    P("\r\n--- runtime ---\r\n");
    check(tile_sense_adc_6_update(&adc), "update() reads a set");
    show_channels();
    check(tile_sense_adc_6_is_valid(&adc), "readings report valid");

    uint16_t supply = tile_sense_adc_6_supply_mv(&adc);
    P("  supply %u mV, status 0x%02X\r\n", supply, tile_sense_adc_6_status(&adc));
    check(supply > 1700 && supply < 3700, "supply_mv plausible");
    check(tile_sense_adc_6_read_mv(&adc, SENSE_ADC_6_CHANNELS) == 0,
          "read_mv() rejects an out-of-range channel");

    /* Every channel must read inside the rail; a stuck bus would show as
     * identical values everywhere. */
    uint16_t all[SENSE_ADC_6_CHANNELS];
    tile_sense_adc_6_read_all(&adc, all);
    int in_range = 1, all_same = 1;
    for (uint8_t i = 0; i < SENSE_ADC_6_CHANNELS; i++) {
        if (all[i] > supply) in_range = 0;
        if (all[i] != all[0]) all_same = 0;
    }
    check(in_range, "every channel reads at or below the supply");
    check(!all_same, "channels differ (not a stuck bus)");
    check(tile_sense_adc_6_channel_pad(0) == 2 && tile_sense_adc_6_channel_pad(5) == 9,
          "channel-to-pad mapping is CH0=pad2 .. CH5=pad9");

    /* ---- sequence advances ---- */
    uint8_t seq0 = tile_sense_adc_6_sequence(&adc);
    core_delay_ms(200);
    tile_sense_adc_6_update(&adc);
    check((uint8_t)(tile_sense_adc_6_sequence(&adc) - seq0) > 0, "sequence advances over time");

    /* ---- settings ---- */
    P("\r\n--- settings ---\r\n");
    P("  as found: %u Hz, oversamp %u, window %u\r\n",
      tile_sense_adc_6_get_rate(&adc), tile_sense_adc_6_get_oversamp(&adc),
      tile_sense_adc_6_get_window(&adc));
    report_rate("default", tile_sense_adc_6_get_rate(&adc));

    check(tile_sense_adc_6_set_rate(&adc, 120, 8, 2), "set_rate(120, 8, 2) accepted");
    check(tile_sense_adc_6_get_rate(&adc) == 120 && tile_sense_adc_6_get_window(&adc) == 2,
          "getters report the new settings");
    report_rate("120 Hz, 60 Hz rejection", 120);

    /* Out of range on the tile (64 sweeps): the driver must report the
     * rejection AND the tile must keep running as it was. */
    check(!tile_sense_adc_6_set_rate(&adc, 100, 32, 2), "set_rate(100, 32, 2) rejected");
    check(tile_sense_adc_6_get_rate(&adc) == 120, "rejected settings leave 120 Hz running");
    check(tile_sense_adc_6_update(&adc), "tile still responds after a rejection");

    /* Rejected locally, without touching the bus at all. */
    check(!tile_sense_adc_6_set_rate(&adc, 0, 8, 1), "set_rate rejects rate 0");
    check(!tile_sense_adc_6_set_rate(&adc, 100, 8, 3), "set_rate rejects window 3");

    check(tile_sense_adc_6_set_rate(&adc, 100, 8, 1), "set_rate(100, 8, 1) restores default");

    P("\r\n=== %u passed, %u failed ===\r\n", pass_n, fail_n);
    P("live readings follow; close the port to stop\r\n\r\n");
}

int main(void)
{
    core_init();

    for (;;) {
        while (!core_usb_connected()) core_delay_ms(20);
        core_delay_ms(300);
        run_sequence();

        while (core_usb_connected()) {
            if (tile_sense_adc_6_update(&adc)) show_channels();
            else P("  update failed\r\n");
            core_delay_ms(1000);
        }
    }
}
