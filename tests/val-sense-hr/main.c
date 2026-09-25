/**
 * val-sense-hr -- Compile-only validation for the Sense.HR tile driver.
 *
 * Exercises every public API function in tile_sense_hr.h to verify
 * compilation. Does not require hardware -- all results are cast to void.
 *
 * Core.ST.L4.1, clock=max, I2C1 at 400 kHz, INTB on pad 3
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_sense_hr.h"

static void on_hr_event(tile_t *tile, uint8_t events, void *ctx)
{
    (void)tile; (void)events; (void)ctx;
}

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_t hr;

    /* ---- Lifecycle ---- */

    uint8_t found = tile_sense_hr_find(hal, 0);
    (void)found;
    found = tile_sense_hr_find(hal, 1);
    (void)found;

    /* Init with defaults */
    tile_sense_hr_init(hal, 0, &hr, NULL);

    /* Init with config + interrupt pin */
    sense_hr_cfg_t cfg = {
        .fps         = 256,
        .led         = SENSE_HR_LED_RED,
        .led_ua      = 5000,
        .tint        = SENSE_HR_TINT_58US,
        .aver        = SENSE_HR_AVER_8,
        .adc_range   = SENSE_HR_ADC_16UA,
        .pdsel       = SENSE_HR_PD_1_2,
        .ppg2_enable = 1,
        .int_pin     = 3,
        .on_event    = on_hr_event,
        .event_ctx   = NULL,
    };
    tile_sense_hr_init(hal, 0, &hr, &cfg);

    /* Alternate address, then back to default */
    tile_sense_hr_init(hal, 1, &hr, NULL);
    tile_sense_hr_init(hal, 0, &hr, NULL);

    tile_sense_hr_sleep(&hr);
    tile_sense_hr_wake(&hr);
    tile_sense_hr_reset(&hr);
    tile_sense_hr_init(hal, 0, &hr, NULL);

    /* ---- Events ---- */

    tile_sense_hr_on_event(&hr, on_hr_event, NULL);
    tile_sense_hr_set_interrupts(&hr, MAX86174A_STATUS1_FRAME_RDY |
                                      MAX86174A_STATUS1_A_FULL);
    tile_sense_hr_process(&hr);

    /* ---- Configuration ---- */

    uint16_t fps = tile_sense_hr_set_frame_rate(&hr, 128);
    (void)fps;
    fps = tile_sense_hr_get_frame_rate(&hr);
    (void)fps;

    tile_sense_hr_set_slot(&hr, 0, SENSE_HR_LED_GREEN, 5000);
    tile_sense_hr_set_slot(&hr, 1, SENSE_HR_LED_RED, 5000);
    tile_sense_hr_set_slot(&hr, 2, SENSE_HR_LED_IR, 5000);
    tile_sense_hr_set_slot(&hr, 3, SENSE_HR_LED_GREEN2, 1000);
    tile_sense_hr_set_ambient_slot(&hr, 4);
    tile_sense_hr_enable_slot(&hr, 3, 0);
    tile_sense_hr_enable_slot(&hr, 4, 0);

    tile_sense_hr_set_led_current(&hr, 0, 12500);
    uint32_t ua = tile_sense_hr_get_led_current(&hr, 0);
    (void)ua;
    tile_sense_hr_set_integration(&hr, 0, SENSE_HR_TINT_117US);
    tile_sense_hr_set_averaging(&hr, 0, SENSE_HR_AVER_4);
    tile_sense_hr_set_adc_range(&hr, 0, SENSE_HR_ADC_32UA);
    tile_sense_hr_set_dac_offset(&hr, 0, 8);
    tile_sense_hr_set_photodiodes(&hr, 0, SENSE_HR_PD_BOTH_PPG1);
    tile_sense_hr_set_ppg2_enabled(&hr, 0);

    uint8_t tuned = tile_sense_hr_auto_gain(&hr, 0, 10000);
    (void)tuned;

    /* ---- Runtime ---- */

    tile_sense_hr_start(&hr);

    uint8_t st = tile_sense_hr_get_status(&hr);
    (void)st;
    st = tile_sense_hr_get_status2(&hr);
    (void)st;

    uint8_t rdy = tile_sense_hr_frame_ready(&hr);
    (void)rdy;

    uint16_t count = tile_sense_hr_get_fifo_count(&hr);
    (void)count;
    uint8_t lost = tile_sense_hr_get_fifo_overflow(&hr);
    (void)lost;
    tile_sense_hr_set_fifo_watermark(&hr, 128);

    int32_t frame[SENSE_HR_FRAME_LEN];
    uint8_t got = tile_sense_hr_read_frame(&hr, frame);
    (void)got;

    int32_t ppg = tile_sense_hr_read_ppg(&hr, 0);
    (void)ppg;
    ppg = tile_sense_hr_read_ppg2(&hr, 0);
    (void)ppg;
    int32_t na = tile_sense_hr_read_photocurrent_na(&hr, 0);
    (void)na;
    uint16_t ovf = tile_sense_hr_get_overflow_count(&hr);
    (void)ovf;

    uint32_t raw[16];
    uint16_t n = tile_sense_hr_read_fifo_raw(&hr, raw, 16);
    (void)n;

    tile_sense_hr_flush_fifo(&hr);
    tile_sense_hr_stop(&hr);

    /* ---- Proximity / threshold ---- */

    tile_sense_hr_set_threshold(&hr, 0, 4, 200);
    tile_sense_hr_set_slot(&hr, 5, SENSE_HR_LED_IR, 500);
    tile_sense_hr_set_proximity_mode(&hr, 1, 4);
    tile_sense_hr_set_proximity_mode(&hr, 0, 0);

    /* ---- Advanced ---- */

    uint8_t reg = tile_sense_hr_read_reg(&hr, MAX86174A_REG_PART_ID);
    (void)reg;
    tile_sense_hr_write_reg(&hr, MAX86174A_REG_FIFO_CFG1, 0x7F);

    /* ---- State checks ---- */

    uint8_t ready = tile_is_ready(&hr);
    (void)ready;
    tile_state_t state = tile_state(&hr);
    (void)state;

    while (1) {
        tile_sense_hr_process(&hr);
        core_delay_ms(10);
    }
}
