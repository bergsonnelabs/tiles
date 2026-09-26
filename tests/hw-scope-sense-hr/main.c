/**
 * hw-scope-sense-hr — stream a Sense.HR (MAX86174A) PPG to the Scope over USB.
 *
 * Board: Core.ST.L4 + Sense.HR on I2C1, Core pads 4/5. The tile needs its
 * own 1.8 V on pad 10 and VLED on pad 9 (validators/sense-hr/README.md).
 *
 * Uses the tile_sense_hr driver from drivers/ (1.0.0, tiles#276).
 *
 * Slot 0 = green, polled, 128 fps. auto_gain() runs at boot and whenever you
 * send 'g' — rest a finger on the tile first, then send 'g'.
 *
 * Every slot-0 PPG1 sample in the FIFO is streamed (raw FIFO read, not
 * read_ppg(), which keeps only the newest frame and drops the rest), at the
 * chip's declared frame rate so the time axis is the chip's own clock.
 *
 * Channels: ppg (ADC counts, the waveform), ovf (running count of
 * overflow-tagged samples; if it climbs, the gain is wrong - send 'g').
 *
 *   make && make flash-dfu
 *   python3 tools/scope_plot.py
 */

#include "core.h"
#include "core_usb.h"
#include "core_tiles.h"
#include "core_scope.h"
#include "tile_sense_hr.h"

#define LED_MAX_UA 10000u   /* driver's safe cap for the 0402 emitters */

static tile_t hr;

static int32_t  ppg;
static uint32_t ovf;
static uint32_t fifo[64];

static void calibrate(void)
{
    core_usb_printf("auto_gain: keep still (~3 s)...\r\n");
    uint8_t ok = tile_sense_hr_auto_gain(&hr, 0, LED_MAX_UA);
    core_usb_printf("auto_gain %s: LED %lu uA, STATUS2 0x%02x\r\n",
                    ok ? "ok" : "FAILED",
                    (unsigned long)tile_sense_hr_get_led_current(&hr, 0),
                    tile_sense_hr_get_status2(&hr));
    tile_sense_hr_start(&hr);
}

int main(void)
{
    core_init();
    core_delay_ms(1500);   /* let the host open CDC before the first prints */

    core_scope_watch(ppg);
    core_scope_watch(ovf);

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    uint8_t found = tile_sense_hr_find(hal, 0);
    tile_sense_hr_init(hal, 0, &hr, NULL);
    core_usb_printf("Sense.HR find=%u state=%u\r\n", found, hr.state);
    if (hr.state == TILE_STATE_READY) {
        core_scope_declare_rate_hz(tile_sense_hr_get_frame_rate(&hr));
        calibrate();
    }

    uint32_t last_report = 0;
    while (1) {
        if (core_usb_available() && core_usb_getc() == 'g' && hr.state == TILE_STATE_READY) calibrate();

        if (hr.state != TILE_STATE_READY) {
            if (core_millis() - last_report > 2000) {
                last_report = core_millis();
                core_usb_printf("Sense.HR not ready (find=%u) - check 1.8 V on pad 10, pad 3 strap\r\n",
                                tile_sense_hr_find(hal, 0));
            }
            core_scope_pump();
            continue;
        }

        uint16_t n = tile_sense_hr_read_fifo_raw(&hr, fifo, 64);
        for (uint16_t i = 0; i < n; i++) {
            uint8_t tag = (uint8_t)((fifo[i] >> MAX86174A_FIFO_TAG_SHIFT) & 0x0F);
            if (tag == MAX86174A_FIFO_TAG_EXP_OVF || tag == MAX86174A_FIFO_TAG_ALC_OVF) {
                ovf++;
                continue;
            }
            if (tag != 0) continue;                        /* slot 0, PPG1 only */
            int32_t v = (int32_t)(fifo[i] & 0xFFFFFu);
            if (v & 0x80000) v -= 0x100000;                /* 20-bit two's complement */
            ppg = v;
            core_scope_sample();
        }
        core_scope_pump();
    }
}
