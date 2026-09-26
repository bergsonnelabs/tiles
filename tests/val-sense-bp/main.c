/**
 * val-sense-bp — Compile-only validation for the Sense.BP tile driver.
 *
 * Exercises every public API function in tile_sense_bp.h to verify
 * compilation. Does not require hardware — all results are cast to void.
 *
 * Core.ST.L4, clock=max, I2C3 at 400 kHz
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_sense_bp.h"

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c3);
    tile_t baro;

    /* ---- Lifecycle ---- */

    uint8_t found = tile_sense_bp_find(hal, 0);
    (void)found;

    /* Init with defaults */
    tile_sense_bp_init(hal, 0, &baro, NULL);

    /* Init with config */
    sense_bp_cfg_t cfg = {
        .odr    = SENSE_BP_ODR_50HZ,
        .avg    = SENSE_BP_AVG_32,
        .fs     = SENSE_BP_FS_1260HPA,
        .lpf    = 1,
        .lpf_bw = SENSE_BP_LPF_ODR_9,
        .bdu    = 1,
    };
    tile_sense_bp_init(hal, 0, &baro, &cfg);

    /* Init with alternate address */
    tile_sense_bp_init(hal, 1, &baro, NULL);

    /* Re-init on default instance */
    tile_sense_bp_init(hal, 0, &baro, NULL);

    tile_sense_bp_sleep(&baro);
    tile_sense_bp_wake(&baro);
    tile_sense_bp_reset(&baro);

    /* Re-init after reset */
    tile_sense_bp_init(hal, 0, &baro, NULL);

    /* ---- Configuration ---- */

    tile_sense_bp_set_odr(&baro, SENSE_BP_ODR_10HZ);
    tile_sense_bp_set_odr(&baro, SENSE_BP_ODR_200HZ);
    tile_sense_bp_set_avg(&baro, SENSE_BP_AVG_128);
    tile_sense_bp_set_avg(&baro, SENSE_BP_AVG_512);
    tile_sense_bp_set_fullscale(&baro, SENSE_BP_FS_4060HPA);
    tile_sense_bp_set_fullscale(&baro, SENSE_BP_FS_1260HPA);
    tile_sense_bp_set_lpf(&baro, 1, SENSE_BP_LPF_ODR_4);
    tile_sense_bp_set_lpf(&baro, 0, SENSE_BP_LPF_ODR_9);

    /* ---- Pressure data ---- */

    int32_t raw_p = tile_sense_bp_get_pressure_raw(&baro);
    (void)raw_p;

    int32_t mhpa = tile_sense_bp_get_pressure_mhpa(&baro);
    (void)mhpa;

    /* ---- Temperature data ---- */

    int16_t raw_t = tile_sense_bp_get_temp_raw(&baro);
    (void)raw_t;

    int32_t cdeg = tile_sense_bp_get_temp_cdeg(&baro);
    (void)cdeg;

    /* ---- One-shot mode ---- */

    tile_sense_bp_set_odr(&baro, SENSE_BP_ODR_POWERDOWN);
    tile_sense_bp_oneshot(&baro);

    /* Restore continuous mode */
    tile_sense_bp_set_odr(&baro, SENSE_BP_ODR_25HZ);

    /* ---- Status ---- */

    uint8_t status = tile_sense_bp_get_status(&baro);
    (void)status;

    uint8_t p_rdy = tile_sense_bp_pressure_ready(&baro);
    (void)p_rdy;

    uint8_t t_rdy = tile_sense_bp_temp_ready(&baro);
    (void)t_rdy;

    /* ---- FIFO ---- */

    tile_sense_bp_set_fifo_mode(&baro, SENSE_BP_FIFO_CONTINUOUS);
    tile_sense_bp_set_fifo_watermark(&baro, 32);

    uint8_t fifo_level = tile_sense_bp_get_fifo_level(&baro);
    (void)fifo_level;

    uint8_t fifo_status = tile_sense_bp_get_fifo_status(&baro);
    (void)fifo_status;

    int32_t fifo_sample = tile_sense_bp_read_fifo_raw(&baro);
    (void)fifo_sample;

    int32_t fifo_buf[16];
    uint8_t read_count = tile_sense_bp_read_fifo_batch(&baro, fifo_buf, 16);
    (void)read_count;

    /* Reset FIFO to bypass */
    tile_sense_bp_set_fifo_mode(&baro, SENSE_BP_FIFO_BYPASS);

    /* Test other FIFO modes */
    tile_sense_bp_set_fifo_mode(&baro, SENSE_BP_FIFO_FIFO);
    tile_sense_bp_set_fifo_mode(&baro, SENSE_BP_FIFO_BYP2FIFO);
    tile_sense_bp_set_fifo_mode(&baro, SENSE_BP_FIFO_BYP2CONT);
    tile_sense_bp_set_fifo_mode(&baro, SENSE_BP_FIFO_CONT2FIFO);
    tile_sense_bp_set_fifo_mode(&baro, SENSE_BP_FIFO_BYPASS);

    /* ---- Interrupt / threshold ---- */

    tile_sense_bp_set_threshold_hpa(&baro, 5);
    tile_sense_bp_set_interrupt_cfg(&baro,
        ILPS22QS_INTCFG_PHE | ILPS22QS_INTCFG_PLE | ILPS22QS_INTCFG_LIR);

    uint8_t int_src = tile_sense_bp_get_int_source(&baro);
    (void)int_src;

    uint8_t booted = tile_sense_bp_is_boot_complete(&baro);
    (void)booted;

    /* Clear interrupt config */
    tile_sense_bp_set_interrupt_cfg(&baro, 0);

    /* ---- Reference / offset calibration ---- */

    tile_sense_bp_set_autozero(&baro);
    tile_sense_bp_reset_autozero(&baro);

    tile_sense_bp_set_autorefp(&baro);
    tile_sense_bp_reset_autorefp(&baro);

    tile_sense_bp_set_pressure_offset(&baro, 100);
    tile_sense_bp_set_pressure_offset(&baro, 0);

    int16_t ref_p = tile_sense_bp_get_ref_pressure(&baro);
    (void)ref_p;

    /* ---- v1.2 tier-2 runtime helpers ---- */
    int32_t alt_mm = tile_sense_bp_read_altitude_mm(&baro, 101325);
    (void)alt_mm;
    uint8_t pressure_changed = tile_sense_bp_wait_for_pressure_change(&baro, 1, 5);
    (void)pressure_changed;

    /* ---- State checks ---- */

    uint8_t ready = tile_is_ready(&baro);
    (void)ready;

    tile_state_t state = tile_state(&baro);
    (void)state;

    while (1) {
        core_delay_ms(1000);
    }
}
