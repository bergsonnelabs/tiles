/**
 * val-sense-i-9 — Compile-only validation for the Sense.I.9 driver.
 *
 * Exercises every public API function in tile_sense_i_9.h to verify
 * compilation. Does not require hardware — all results are cast to void.
 *
 * Core.ST.L4, clock=max, I2C1 at 400 kHz.
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_sense_i_9.h"

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_t imu;

    /* ---- Lifecycle ---- */
    uint8_t found = tile_sense_i_9_find(hal, 0);
    (void)found;
    tile_sense_i_9_init(hal, 0, &imu, NULL);

    /* ---- Range / ODR / mag mode ---- */
    tile_sense_i_9_set_accel_range(&imu, SENSE_I_9_ACCEL_2G);
    tile_sense_i_9_set_gyro_range(&imu, SENSE_I_9_GYRO_250DPS);
    tile_sense_i_9_set_accel_odr(&imu, 10);
    tile_sense_i_9_set_gyro_odr(&imu, 10);
    tile_sense_i_9_set_mag_mode(&imu, SENSE_I_9_MAG_CONTINUOUS_50HZ);

    /* ---- Reads ---- */
    (void)tile_sense_i_9_data_ready(&imu);
    int16_t accel[3], gyro[3], mag[3], six[6];
    tile_sense_i_9_get_raw_accels(&imu, accel);
    tile_sense_i_9_get_raw_gyros(&imu, gyro);
    tile_sense_i_9_get_raw_6dof(&imu, six);
    tile_sense_i_9_get_raw_mags(&imu, mag);
    (void)tile_sense_i_9_mag_overflowed(&imu);
    (void)tile_sense_i_9_get_temperature(&imu);

    /* ---- INT routing ---- */
    tile_sense_i_9_int_config(&imu, SENSE_I_9_INT_ACTIVE_LOW |
                                     SENSE_I_9_INT_OPEN_DRAIN |
                                     SENSE_I_9_INT_LATCHED);
    tile_sense_i_9_int_data_ready(&imu, 1);
    tile_sense_i_9_int_data_ready(&imu, 0);
    tile_sense_i_9_int_wom(&imu, 1);
    tile_sense_i_9_int_wom(&imu, 0);
    tile_sense_i_9_int_fifo_overflow(&imu, 1);
    tile_sense_i_9_int_fifo_overflow(&imu, 0);
    tile_sense_i_9_int_fifo_watermark(&imu, 1);
    tile_sense_i_9_int_fifo_watermark(&imu, 0);

    (void)tile_sense_i_9_get_int_status(&imu);
    (void)tile_sense_i_9_get_int_status_fifo_overflow(&imu);
    (void)tile_sense_i_9_get_int_status_fifo_watermark(&imu);

    /* ---- Wake on Motion ---- */
    tile_sense_i_9_wom_config(&imu, 200, SENSE_I_9_WOM_VS_PREVIOUS);
    tile_sense_i_9_wom_enable(&imu);
    tile_sense_i_9_wom_disable(&imu);

    /* ---- FIFO ---- */
    tile_sense_i_9_fifo_config(&imu, SENSE_I_9_FIFO_STREAM,
                                /*accel=*/1, /*gyro=*/1, /*temp=*/0);
    (void)tile_sense_i_9_fifo_count(&imu);
    sense_i_9_fifo_packet_t pkt;
    (void)tile_sense_i_9_fifo_read_packet(&imu, &pkt);
    tile_sense_i_9_fifo_flush(&imu);
    tile_sense_i_9_fifo_config(&imu, SENSE_I_9_FIFO_SNAPSHOT, 0, 0, 0);

    /* ---- Self-test (IMU + mag) ---- */
    uint8_t apass = 0, gpass = 0;
    (void)tile_sense_i_9_self_test(&imu, &apass, &gpass);
    (void)tile_sense_i_9_mag_self_test(&imu);

    /* ---- Sleep / wake / reset ---- */
    tile_sense_i_9_sleep(&imu);
    tile_sense_i_9_wake(&imu);
    tile_sense_i_9_reset(&imu);

    /* ---- v3.1 tier-2 motion helpers ---- */
    uint8_t up   = tile_sense_i_9_is_face_up(&imu);
    uint8_t down = tile_sense_i_9_is_face_down(&imu);
    uint8_t mov  = tile_sense_i_9_is_moving(&imu, 200);
    (void)up; (void)down; (void)mov;
    int16_t tilt = 0;
    tile_sense_i_9_read_tilt_centi_degrees(&imu, 0, &tilt);
    (void)tilt;
    uint16_t heading = 0;
    tile_sense_i_9_read_heading_centi_degrees(&imu, &heading);
    (void)heading;
    uint8_t mot_seen = tile_sense_i_9_wait_for_motion(&imu, 5);
    (void)mot_seen;

    /* ---- DSL flat-output wrapper (Bucket D) ---- */
    int32_t pkt_flat[6] = {0};
    tile_sense_i_9_fifo_read_packet_flat(&imu, pkt_flat);
    (void)pkt_flat[0];

    /* ---- DMP3 firmware load (Phase 1) ---- */
    uint8_t dmp_ok = tile_sense_i_9_dmp_load(&imu);
    uint8_t dmp_loaded = tile_sense_i_9_dmp_is_loaded(&imu);
    (void)dmp_ok; (void)dmp_loaded;

    /* ---- DMP3 9-axis quaternion (Phase 2) ---- */
    (void)tile_sense_i_9_dmp_start_quat9(&imu, /*period_ms=*/40);  /* ~25 Hz */
    (void)tile_sense_i_9_dmp_data_ready(&imu);
    int32_t q[4] = {0};
    uint16_t q_acc = 0;
    (void)tile_sense_i_9_dmp_read_quat9(&imu, q, &q_acc);
    (void)q[0]; (void)q_acc;
    tile_sense_i_9_dmp_stop(&imu);

    while (1) {
        core_delay_ms(1000);
    }
}
