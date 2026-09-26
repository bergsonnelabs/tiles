/**
 * val-sense-cam-p — Compile-only validation for the Sense.CAM.P driver.
 *
 * Exercises every public API through a dual-bus PAL (core_tiles_pal2):
 * I2C1 for configuration, SPI1 (mode 3) for image readout. Wiring matches
 * the bench validator (validators/sense-c-pag): tile 6/7/8/9 (MISO/CLK/CS/
 * MOSI) to Core pads 8/3/9/2. Does not require hardware.
 *
 * Core.ST.L4, clock=max.
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_sense_cam_p.h"

static uint8_t frame[160 * 120];

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal2(&core_i2c1, &core_spi1);
    tile_t cam;

    uint8_t found = tile_sense_cam_p_find(hal, 0);
    (void)found;

    sense_cam_p_cfg_t cfg = { .spi_cs = 0, .resolution = SENSE_CAM_P_RES_160x120 };
    tile_sense_cam_p_init(hal, 0, &cam, &cfg);

    (void)tile_sense_cam_p_part_id(&cam);
    (void)tile_sense_cam_p_spi_id(&cam);
    (void)tile_sense_cam_p_width(&cam);
    (void)tile_sense_cam_p_height(&cam);
    (void)tile_sense_cam_p_frame_bytes(&cam);
    (void)tile_sense_cam_p_capture(&cam, frame, sizeof(frame));
    tile_sense_cam_p_reset(&cam);

    for (;;) {
    }
}
