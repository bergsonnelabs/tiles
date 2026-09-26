/**
 * val-sense-tof -- Compile-only validation for the Sense.TOF tile driver.
 *
 * Exercises every public API function in tile_sense_tof.h to verify
 * compilation. Does not require hardware -- all results are cast to void.
 *
 * Core.ST.L4, clock=max, I2C3 at 400 kHz
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_sense_tof.h"

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c3);
    tile_t tof;

    /* ---- Lifecycle ---- */

    uint8_t found = tile_sense_tof_find(hal, 0);
    (void)found;

    /* Init with defaults */
    tile_sense_tof_init(hal, 0, &tof, NULL);

    /* Init with config */
    sense_tof_cfg_t cfg = {
        .mode       = SENSE_TOF_RANGE_2500MM,
        .period_ms  = 30,
        .kilo_iters = 900,
        .threshold  = 6,
    };
    tile_sense_tof_init(hal, 0, &tof, &cfg);

    /* ---- Power management ---- */

    tile_sense_tof_sleep(&tof);
    tile_sense_tof_wake(&tof);
    tile_sense_tof_reset(&tof);

    /* Re-init after reset */
    tile_sense_tof_init(hal, 0, &tof, NULL);

    /* ---- Measurement control ---- */

    tile_sense_tof_start(&tof);
    tile_sense_tof_stop(&tof);

    /* Single-shot measurement */
    sense_tof_result_t result;
    uint8_t ok = tile_sense_tof_measure_single(&tof, &result, 500);
    (void)ok;

    /* Single-shot with NULL result */
    ok = tile_sense_tof_measure_single(&tof, NULL, 500);
    (void)ok;

    /* ---- Result reading ---- */

    uint16_t dist = tile_sense_tof_get_distance_mm(&tof);
    (void)dist;

    sense_tof_result_t res;
    tile_sense_tof_get_result(&tof, &res);
    (void)res.distance_mm;
    (void)res.status;
    (void)res.reliability;
    (void)res.temperature;
    (void)res.result_number;

    uint8_t ready = tile_sense_tof_result_ready(&tof);
    (void)ready;

    /* ---- Calibration ---- */

    uint8_t cal_ok = tile_sense_tof_factory_calibrate(&tof, 5000);
    (void)cal_ok;

    uint8_t cal_data[TMF8806_CALIB_DATA_LEN];
    tile_sense_tof_get_calibration(&tof, cal_data);
    tile_sense_tof_set_calibration(&tof, cal_data);

    /* ---- Info ---- */

    sense_tof_version_t ver;
    tile_sense_tof_get_app_version(&tof, &ver);
    (void)ver.major;
    (void)ver.minor;
    (void)ver.patch;

    uint8_t serial[4];
    uint8_t serial_ok = tile_sense_tof_get_serial_number(&tof, serial);
    (void)serial_ok;

    /* ---- Runtime configuration ---- */

    tile_sense_tof_set_distance_mode(&tof, SENSE_TOF_SHORT_RANGE);
    tile_sense_tof_set_distance_mode(&tof, SENSE_TOF_RANGE_5000MM);
    tile_sense_tof_set_distance_mode(&tof, SENSE_TOF_RANGE_2500MM);
    tile_sense_tof_set_period(&tof, 0x00);
    tile_sense_tof_set_period(&tof, 0xFE);
    tile_sense_tof_set_period(&tof, 0x1E);

    /* ---- Algorithm state save/restore ---- */

    uint8_t alg_state[TMF8806_STATE_DATA_LEN];
    tile_sense_tof_save_state(&tof, alg_state);
    tile_sense_tof_restore_state(&tof, alg_state);

    /* ---- Threshold-based interrupts (v1.1) ---- */

    uint8_t thr_ok = tile_sense_tof_set_threshold_interrupt(&tof,
                                                             /*persistence=*/3,
                                                             /*low_mm=*/100,
                                                             /*high_mm=*/500);
    (void)thr_ok;

    uint8_t   pers = 0;
    uint16_t  low_mm = 0, high_mm = 0;
    thr_ok = tile_sense_tof_get_threshold_interrupt(&tof,
                                                    &pers, &low_mm, &high_mm);
    (void)thr_ok; (void)pers; (void)low_mm; (void)high_mm;

    /* Disable threshold filtering — every-measurement INT */
    (void)tile_sense_tof_set_threshold_interrupt(&tof, 0, 0, 0);

    /* ---- Oscillator drift correction (v1.1) ---- */

    /* Reads SYS_CLOCK_0..3 as a 32-bit tick count for host-side
     * drift compensation per HostDriverCommunication §10. */
    uint32_t sys_ticks = tile_sense_tof_get_sys_clock_ticks(&tof);
    (void)sys_ticks;

    /* ---- Raw histogram readout (v1.1) ---- */

    uint8_t hist_buf[128];
    /* 0x10 = short-range histogram per HostDriverCommunication §8.11
     * example. Other types per the TMF8806 datasheet. */
    uint8_t hist_ok = tile_sense_tof_read_histogram(&tof, /*hist_type=*/0x10,
                                                     hist_buf,
                                                     /*timeout_ms=*/500);
    (void)hist_ok;

    /* ---- tier-2 presence helpers ---- */
    uint8_t within = tile_sense_tof_is_object_within(&tof, 500);
    (void)within;
    uint8_t saw = tile_sense_tof_wait_for_object(&tof, 500, 5);
    (void)saw;
    uint16_t mm_out = 0;
    uint8_t conf = 0;
    uint8_t got = tile_sense_tof_read_distance_with_confidence(&tof, &mm_out, &conf);
    (void)got; (void)mm_out; (void)conf;

    /* ---- DSL flat-output wrappers (Bucket D) ---- */
    int32_t res_flat[5] = {0};
    tile_sense_tof_get_result_flat(&tof, res_flat);
    (void)res_flat[0];

    int32_t mm_f = 0, status_f = 0, rel_f = 0, temp_f = 0, seq_f = 0;
    uint8_t ok_f = tile_sense_tof_measure_single_flat(&tof, &mm_f, &status_f,
                                                       &rel_f, &temp_f, &seq_f, 5);
    (void)ok_f; (void)mm_f; (void)status_f; (void)rel_f; (void)temp_f; (void)seq_f;

    int32_t v_major = 0, v_minor = 0, v_patch = 0;
    tile_sense_tof_get_app_version_flat(&tof, &v_major, &v_minor, &v_patch);
    (void)v_major; (void)v_minor; (void)v_patch;

    int32_t serial_flat[4] = {0};
    tile_sense_tof_get_serial_number_flat(&tof, serial_flat);
    (void)serial_flat[0];

    int32_t hist_flat[128] = {0};
    tile_sense_tof_read_histogram_flat(&tof, /*hist_type=*/0x10,
                                       /*timeout_ms=*/5, hist_flat);
    (void)hist_flat[0];

    /* ---- v1.3 runtime setters + signal-quality ---- */
    tile_sense_tof_set_kilo_iters(&tof, 1800);
    tile_sense_tof_set_threshold(&tof, 12);

    sense_tof_signal_t sig = {0};
    tile_sense_tof_get_signal_quality(&tof, &sig);
    (void)sig.reference_hits; (void)sig.object_hits; (void)sig.crosstalk;

    int32_t ref_hits = 0, obj_hits = 0, xtalk = 0;
    tile_sense_tof_get_signal_quality_flat(&tof, &ref_hits, &obj_hits, &xtalk);
    (void)ref_hits; (void)obj_hits; (void)xtalk;

    /* ---- State checks ---- */

    uint8_t is_ready = tile_is_ready(&tof);
    (void)is_ready;

    tile_state_t state = tile_state(&tof);
    (void)state;

    while (1) {
        core_delay_ms(1000);
    }
}
