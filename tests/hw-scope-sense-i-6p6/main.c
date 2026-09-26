/**
 * hw-scope-sense-i-6p6 — stream a Sense.I.6P6 to the Scope over USB.
 *
 * Board: Core.ST.L4 + Sense.I.6P6 (ICM-42686P) on I2C1, Core pads 4/5.
 * Driver defaults: ±8 g, ±1000 dps, 100 Hz, polled.
 *
 * Channels (physical units, converted here so any Scope host plots them
 * as-is): accel_g[0..2], gyro_dps[0..2], temp_c.
 *
 *   make && make flash-dfu
 *   python3 tools/scope_plot.py
 */

#include "core.h"
#include "core_tiles.h"
#include "core_scope.h"
#include "tile_sense_i_6p6.h"

static tile_t imu;

static float accel_g[3];
static float gyro_dps[3];
static float temp_c;

int main(void)
{
    core_init();

    core_scope_watch_array(accel_g, 3);
    core_scope_watch_array(gyro_dps, 3);
    core_scope_watch(temp_c);

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_sense_i_6p6_init(hal, 0, &imu, NULL);

    int16_t raw[7];   /* Temp, AX, AY, AZ, GX, GY, GZ */
    while (1) {
        if (imu.state == TILE_STATE_READY && tile_sense_i_6p6_data_ready(&imu)) {
            tile_sense_i_6p6_get_raw_all(&imu, raw);
            temp_c = (float)raw[0] / 132.48f + 25.0f;
            for (int i = 0; i < 3; i++) {
                accel_g[i]  = (float)raw[1 + i] / 4096.0f;   /* ±8 g */
                gyro_dps[i] = (float)raw[4 + i] / 32.8f;     /* ±1000 dps */
            }
            core_scope_update();
        } else {
            core_scope_pump();
        }
    }
}
