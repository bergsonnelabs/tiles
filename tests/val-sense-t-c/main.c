/**
 * val-sense-t-c — Compile-only validation for the Sense.T.C driver.
 *
 * Exercises every public API function in tile_sense_t_c.h to verify
 * compilation. Does not require hardware — all results are cast to void.
 *
 * Core.ST.L4.1, clock=max, I2C1 at 400 kHz.
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_sense_t_c.h"

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_t touch;

    /* ---- Lifecycle ---- */
    uint8_t found = tile_sense_t_c_find(hal, 0);
    (void)found;
    tile_sense_t_c_init(hal, 0, &touch, NULL);
    tile_sense_t_c_process(&touch);
    tile_sense_t_c_on_event(&touch, NULL, NULL);
    tile_sense_t_c_sleep(&touch);
    tile_sense_t_c_wake(&touch);

    /* ---- Status / data reads ---- */
    (void)tile_sense_t_c_get_status(&touch);
    (void)tile_sense_t_c_get_gestures(&touch);
    (void)tile_sense_t_c_get_counts(&touch, SENSE_T_C_CH0);
    (void)tile_sense_t_c_get_lta(&touch, SENSE_T_C_CH0);
    (void)tile_sense_t_c_get_slider(&touch);

    /* ---- Configuration ---- */
    tile_sense_t_c_set_thresholds(&touch, SENSE_T_C_CH_SURFACE, 50, 100);
    tile_sense_t_c_set_touch_threshold(&touch, 40);
    tile_sense_t_c_set_prox_threshold(&touch, 20);
    tile_sense_t_c_set_power_mode(&touch, SENSE_T_C_POWER_LOW);
    tile_sense_t_c_enable_events(&touch, IQS323_STATUS_TOUCH_EVENT |
                                          IQS323_STATUS_PROX_EVENT);
    tile_sense_t_c_ati(&touch);

    /* ---- New: ATI fine-tuning ---- */
    tile_sense_t_c_set_ati_setup(&touch, SENSE_T_C_CH_SURFACE, 0x1A18);
    tile_sense_t_c_set_ati_setup(&touch, SENSE_T_C_CH_EXTERNAL, 0x1A18);
    tile_sense_t_c_set_ati_setup(&touch, SENSE_T_C_CH2, 0x1A18);

    /* ---- New: Filter / conversion frequency tuning ---- */
    tile_sense_t_c_set_counts_filter(&touch, 0x0808);
    tile_sense_t_c_set_conversion_freq(&touch, SENSE_T_C_CH_SURFACE, 0x007F);

    /* ---- New: Reference channel ---- */
    /* Set CH1 (surface) as reference; CH0 (external pad) follows it. */
    tile_sense_t_c_set_channel_mode(&touch, SENSE_T_C_CH_SURFACE,
                                     SENSE_T_C_CHANNEL_REFERENCE, 0);
    tile_sense_t_c_set_channel_mode(&touch, SENSE_T_C_CH_EXTERNAL,
                                     SENSE_T_C_CHANNEL_FOLLOWER,
                                     SENSE_T_C_CH_SURFACE);
    tile_sense_t_c_set_channel_mode(&touch, SENSE_T_C_CH2,
                                     SENSE_T_C_CHANNEL_INDEPENDENT, 0);
    /* Re-run ATI after reference-mode changes per datasheet §7.3. */
    tile_sense_t_c_ati(&touch);

    /* ---- New: Communication mode ---- */
    tile_sense_t_c_set_comm_mode(&touch, SENSE_T_C_COMM_STREAM);
    tile_sense_t_c_set_comm_mode(&touch, SENSE_T_C_COMM_EVENT);

    /* ---- Reset + raw register escape hatches ---- */
    tile_sense_t_c_reset(&touch);
    (void)tile_sense_t_c_read_reg(&touch, IQS323_REG_PRODUCT_NUM);
    tile_sense_t_c_write_reg(&touch, IQS323_REG_GESTURE_ENABLE, 0x003F);

    /* ---- v1.2 tier-2 runtime helpers ---- */
    uint8_t any_touch = tile_sense_t_c_is_touched_any(&touch);
    (void)any_touch;
    uint8_t touched = tile_sense_t_c_wait_for_touch(&touch, 5);
    (void)touched;
    uint16_t gesture = tile_sense_t_c_wait_for_gesture(&touch, 5);
    (void)gesture;
    uint8_t pct = 0;
    uint8_t slider_ok = tile_sense_t_c_read_slider_pct(&touch, &pct);
    (void)slider_ok; (void)pct;

    /* ---- v1.3 capability additions ---- */
    tile_sense_t_c_reseed(&touch);
    tile_sense_t_c_set_report_rate(&touch, SENSE_T_C_POWER_NORMAL, 16);
    tile_sense_t_c_set_report_rate(&touch, SENSE_T_C_POWER_ULTRA_LOW, 160);
    tile_sense_t_c_set_power_timeout(&touch, 2000);
    uint16_t comp = tile_sense_t_c_get_compensation(&touch, 1);
    (void)comp;
    tile_sense_t_c_set_compensation(&touch, 1, 512, 31);

    while (1) {
        core_delay_ms(1000);
    }
}
