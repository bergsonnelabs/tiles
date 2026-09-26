/**
 * val-sense-i-6p6 — Compile-only validation for the Sense.I.6P6 driver.
 *
 * Exercises every public API function in tile_sense_i_6p6.h to verify
 * compilation. Does not require hardware — all results are cast to void.
 *
 * Core.ST.L4, clock=max, I2C1 at 400 kHz.
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_sense_i_6p6.h"

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_t imu;

    /* ---- Lifecycle ---- */
    uint8_t found = tile_sense_i_6p6_find(hal, 0);
    (void)found;
    tile_sense_i_6p6_init(hal, 0, &imu, NULL);
    tile_sense_i_6p6_process(&imu);
    tile_sense_i_6p6_on_event(&imu, NULL, NULL);
    tile_sense_i_6p6_sleep(&imu);
    tile_sense_i_6p6_wake(&imu);

    /* ---- Range / ODR / power / filtering ---- */
    tile_sense_i_6p6_set_accel_range(&imu, SENSE_I_6P6_ACCEL_2G);
    tile_sense_i_6p6_set_gyro_range(&imu, SENSE_I_6P6_GYRO_250DPS);
    tile_sense_i_6p6_set_accel_odr(&imu, SENSE_I_6P6_ODR_100HZ);
    tile_sense_i_6p6_set_gyro_odr(&imu, SENSE_I_6P6_ODR_100HZ);
    tile_sense_i_6p6_set_power_mode(&imu, SENSE_I_6P6_MODE_LN, SENSE_I_6P6_MODE_LN);
    tile_sense_i_6p6_set_filter_bw(&imu, SENSE_I_6P6_FILT_BW_ODR_4,
                                          SENSE_I_6P6_FILT_BW_ODR_4);
    tile_sense_i_6p6_set_filter_order(&imu, SENSE_I_6P6_FILT_ORDER_3RD,
                                              SENSE_I_6P6_FILT_ORDER_3RD);
    tile_sense_i_6p6_set_temp_filter(&imu, SENSE_I_6P6_TEMP_FILT_4000HZ);
    tile_sense_i_6p6_set_temp_enabled(&imu, 1);

    /* ---- Reads ---- */
    uint8_t ready = tile_sense_i_6p6_data_ready(&imu);
    (void)ready;
    int16_t accel[3], gyro[3], six[6], all[7];
    tile_sense_i_6p6_get_raw_accels(&imu, accel);
    tile_sense_i_6p6_get_raw_gyros(&imu, gyro);
    tile_sense_i_6p6_get_raw_6dof(&imu, six);
    tile_sense_i_6p6_get_raw_all(&imu, all);
    int16_t temp = tile_sense_i_6p6_get_temperature(&imu);
    (void)temp;

    /* ---- FIFO ---- */
    tile_sense_i_6p6_fifo_config(&imu, SENSE_I_6P6_FIFO_STREAM,
                                  /*accel=*/1, /*gyro=*/1,
                                  /*temp=*/1, /*hires=*/0);
    tile_sense_i_6p6_fifo_set_watermark(&imu, 16);
    tile_sense_i_6p6_fifo_flush(&imu);
    sense_i_6p6_fifo_packet_t pkt;
    uint8_t got = tile_sense_i_6p6_fifo_read_packet(&imu, &pkt);
    (void)got;
    sense_i_6p6_fifo_packet_t pkts[4];
    uint16_t n = tile_sense_i_6p6_fifo_read_packets(&imu, pkts, 4);
    (void)n;
    uint16_t cnt = tile_sense_i_6p6_fifo_count(&imu);
    uint16_t lost = tile_sense_i_6p6_fifo_lost_count(&imu);
    (void)cnt; (void)lost;

    /* ---- INT1 routing ---- */
    tile_sense_i_6p6_int1_config(&imu, 0x07);
    tile_sense_i_6p6_int1_data_ready(&imu, 1);
    tile_sense_i_6p6_int1_fifo_ths(&imu, 1);
    tile_sense_i_6p6_int1_wom(&imu, 1);

    /* ---- INT2 routing (new in v1.1.0) ---- */
    tile_sense_i_6p6_int2_config(&imu, 0x07);
    tile_sense_i_6p6_int2_data_ready(&imu, 1);
    tile_sense_i_6p6_int2_fifo_ths(&imu, 1);
    tile_sense_i_6p6_int2_wom(&imu, 1);

    /* ---- INT pulse-width (new) ---- */
    tile_sense_i_6p6_set_int_pulse_duration(&imu, SENSE_I_6P6_INT_PULSE_8US);
    tile_sense_i_6p6_set_int_pulse_duration(&imu, SENSE_I_6P6_INT_PULSE_100US);

    /* ---- Status / motion / APEX ---- */
    (void)tile_sense_i_6p6_get_int_status(&imu);
    (void)tile_sense_i_6p6_get_int_status2(&imu);
    (void)tile_sense_i_6p6_get_int_status3(&imu);
    tile_sense_i_6p6_wom_config(&imu, 200, 200, 200, SENSE_I_6P6_WOM_PREVIOUS);
    tile_sense_i_6p6_wom_enable(&imu);
    tile_sense_i_6p6_wom_disable(&imu);
    tile_sense_i_6p6_smd_config(&imu, SENSE_I_6P6_SMD_LONG);
    tile_sense_i_6p6_pedometer_enable(&imu, SENSE_I_6P6_DMP_ODR_50HZ);
    tile_sense_i_6p6_pedometer_disable(&imu);
    (void)tile_sense_i_6p6_get_step_count(&imu);
    (void)tile_sense_i_6p6_get_step_cadence(&imu);
    tile_sense_i_6p6_tilt_enable(&imu, 2);
    tile_sense_i_6p6_tilt_disable(&imu);
    tile_sense_i_6p6_tap_enable(&imu);
    tile_sense_i_6p6_tap_disable(&imu);
    sense_i_6p6_tap_result_t tap;
    tile_sense_i_6p6_get_tap_result(&imu, &tap);

    /* ---- Offsets / self-test (already shipped in v1.0) ---- */
    tile_sense_i_6p6_set_gyro_offset(&imu, 0, 0, 0);
    tile_sense_i_6p6_set_accel_offset(&imu, 0, 0, 0);
    uint8_t accel_pass = 0, gyro_pass = 0;
    (void)tile_sense_i_6p6_self_test(&imu, &accel_pass, &gyro_pass);

    /* ---- Subsystem reset (new) ---- */
    tile_sense_i_6p6_subsystem_reset(&imu, SENSE_I_6P6_RESET_APEX);
    tile_sense_i_6p6_subsystem_reset(&imu, SENSE_I_6P6_RESET_TEMP);

    /* ---- Full reset + raw-reg escape hatches ---- */
    tile_sense_i_6p6_reset(&imu);
    (void)tile_sense_i_6p6_read_reg(&imu, 0, ICM42686P_REG_WHO_AM_I);
    tile_sense_i_6p6_write_reg(&imu, 0, ICM42686P_REG_INT_CONFIG, 0x00);

    /* ---- v1.2 tier-2 motion helpers ---- */
    uint8_t up   = tile_sense_i_6p6_is_face_up(&imu);
    uint8_t down = tile_sense_i_6p6_is_face_down(&imu);
    uint8_t mov  = tile_sense_i_6p6_is_moving(&imu, 200);
    (void)up; (void)down; (void)mov;
    int16_t tilt = 0;
    uint8_t tilt_ok = tile_sense_i_6p6_read_tilt_centi_degrees(&imu, 0, &tilt);
    (void)tilt_ok; (void)tilt;
    uint8_t tap_seen = tile_sense_i_6p6_wait_for_tap(&imu, 5);
    uint8_t mot_seen = tile_sense_i_6p6_wait_for_motion(&imu, 5);
    (void)tap_seen; (void)mot_seen;

    /* ---- DSL flat-output wrappers (Bucket D) ---- */
    int32_t pkt_flat[8] = {0};
    tile_sense_i_6p6_fifo_read_packet_flat(&imu, pkt_flat);
    (void)pkt_flat[0];

    int32_t tap_count = 0, tap_axis = 0, tap_dir = 0, tap_timing = 0;
    tile_sense_i_6p6_get_tap_result_flat(&imu, &tap_count, &tap_axis,
                                         &tap_dir, &tap_timing);
    (void)tap_count; (void)tap_axis; (void)tap_dir; (void)tap_timing;

    int32_t pkts_buf[16] = {0};  /* 2 packets × 8 ints */
    uint16_t n_pkts = tile_sense_i_6p6_fifo_read_packets_flat(&imu, pkts_buf, 16);
    (void)n_pkts; (void)pkts_buf[0];

    while (1) {
        core_delay_ms(1000);
    }
}
