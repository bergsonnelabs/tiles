/**
 * val-sense-m-3g -- Compile-only validation for the Sense.M.3G tile driver (v1.1).
 *
 * Exercises every public API function in tile_sense_m_3g.h to verify
 * compilation. Does not require hardware -- all results are cast to void.
 *
 * Core.ST.L4, clock=max, I2C1 at 400 kHz, polled (INT not wired).
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_sense_m_3g.h"

static void on_mag_sample(tile_t *tile, void *ctx)
{
    (void)tile; (void)ctx;
}

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_t mag;

    /* ---- Lifecycle ---- */

    uint8_t found = tile_sense_m_3g_find(hal, 0);
    (void)found;

    /* Defaults: measuring in normal mode at 25 Hz when init returns. */
    tile_sense_m_3g_init(hal, 0, &mag, NULL);

    /* Getters refresh on their own in normal mode. */
    int32_t x = tile_sense_m_3g_get_x_nt(&mag);
    int32_t y = tile_sense_m_3g_get_y_nt(&mag);
    int32_t z = tile_sense_m_3g_get_z_nt(&mag);
    int32_t t = tile_sense_m_3g_get_temperature_mc(&mag);
    uint32_t mag_nt = tile_sense_m_3g_get_magnitude_nt(&mag);
    (void)x; (void)y; (void)z; (void)t; (void)mag_nt;

    /* Low-power start, forced-mode sample. */
    sense_m_3g_cfg_t cfg = {
        .odr             = SENSE_M_3G_ODR_50HZ,
        .averaging       = SENSE_M_3G_AVG_4,
        .axes            = BMM350_EN_XYZ,
        .int_pin         = 0,
        .on_data         = on_mag_sample,
        .data_ctx        = NULL,
        .start_suspended = 1,
    };
    tile_sense_m_3g_init(hal, 0, &mag, &cfg);

    uint8_t ok = tile_sense_m_3g_trigger_measurement(&mag);
    ok = tile_sense_m_3g_read(&mag);
    (void)ok;

    tile_sense_m_3g_set_mode(&mag, SENSE_M_3G_MODE_NORMAL);
    tile_sense_m_3g_sleep(&mag);
    tile_sense_m_3g_wake(&mag);
    ok = tile_sense_m_3g_magnetic_reset(&mag);
    (void)ok;
    tile_sense_m_3g_reset(&mag);

    /* ---- Data ---- */

    tile_sense_m_3g_on_data(&mag, on_mag_sample, NULL);
    tile_sense_m_3g_process(&mag);
    ok = tile_sense_m_3g_data_ready(&mag);
    (void)ok;
    uint32_t st = tile_sense_m_3g_get_sensortime(&mag);
    (void)st;

    /* ---- Configuration ---- */

    tile_sense_m_3g_set_odr_averaging(&mag, SENSE_M_3G_ODR_25HZ, SENSE_M_3G_AVG_4);
    tile_sense_m_3g_set_axes(&mag, BMM350_EN_X | BMM350_EN_Y | BMM350_EN_Z);
    tile_sense_m_3g_configure_interrupt(&mag, 0, 0, 1, 1);
    tile_sense_m_3g_set_pad_drive(&mag, 7);
    tile_sense_m_3g_set_i2c_watchdog(&mag, 1, 1);
    tile_sense_m_3g_set_sensortime_always_on(&mag, 0);

    /* ---- Diagnostics / advanced ---- */

    uint8_t err = tile_sense_m_3g_get_error(&mag);
    uint8_t pmu = tile_sense_m_3g_get_pmu_status(&mag);
    uint16_t otp = tile_sense_m_3g_get_otp_word(&mag, 0);
    int32_t raw = tile_sense_m_3g_get_raw(&mag, 0);
    uint8_t id = tile_sense_m_3g_read_reg(&mag, BMM350_REG_CHIP_ID);
    tile_sense_m_3g_write_reg(&mag, BMM350_REG_PAD_CTRL, 7);
    (void)err; (void)pmu; (void)otp; (void)raw; (void)id;

    while (1) {
        (void)tile_sense_m_3g_get_magnitude_nt(&mag);
        core_delay_ms(40);
    }
}
