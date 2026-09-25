/**
 * @file   tile_sense_cam_p.c
 * @brief  Global-shutter camera driver implementation (PixArt PAG7920J3).
 */

#include "tile_sense_cam_p.h"
#include <stddef.h>

/* -------------------------------------------------------------- */
/* Instance mapping                                                */
/* -------------------------------------------------------------- */

static const uint8_t id_table[] = {
    PAG7920_I2C_ADDR_FLOAT,   /* instance 0 - GPIO2 floating (tile default) */
    PAG7920_I2C_ADDR_HIGH,    /* instance 1 - GPIO2 to VDDIO */
    PAG7920_I2C_ADDR_GND,     /* instance 2 - GPIO2 to GND */
};

#define ID_TABLE_LEN  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    if (instance >= ID_TABLE_LEN) return 0x00;
    return id_table[instance];
}

/* -------------------------------------------------------------- */
/* Driver state                                                    */
/*                                                                 */
/* tile_t carries one identifier and this part needs two: an I2C   */
/* address for configuration and an SPI chip-select for image data. */
/* tile->id holds the I2C address, so the SPI CS and the chosen     */
/* resolution live here. That makes the driver single-instance,     */
/* which is not a practical limit: the address strap is fixed at    */
/* power-on and one camera per bus is the realistic topology.       */
/* -------------------------------------------------------------- */

static uint8_t           s_spi_cs;
static sense_cam_p_res_t s_res;

/* -------------------------------------------------------------- */
/* Vendor initialisation tables                                    */
/*                                                                 */
/* Transcribed from the PAG7920J3 datasheet V0.8 by parsing the     */
/* numbered "Write register 0xAA with value 0xBB" lines; the        */
/* datasheet's own step numbering came out contiguous 1..N with no  */
/* gaps and no conflicting duplicates, which is what verifies the   */
/* transcription.                                                   */
/*                                                                 */
/* APPLY IN ORDER, VERBATIM. Registers repeat at different points,  */
/* 0xEF switches banks partway through, and the sensor is started   */
/* mid-table. This list cannot be sorted, deduped or reordered -    */
/* see the header for what happens if you try.                      */
/* -------------------------------------------------------------- */

typedef struct {
    uint8_t reg;
    uint8_t val;
} pag_reg_t;

/* Section 8.1.3.4 - Default, SPI out. Applied first, always. */
static const pag_reg_t init_base[] = {
    { 0xEF, 0x00 },   /*   1 */
    { 0x1D, 0x81 },   /*   2 */
    { 0x64, 0x02 },   /*   3 */
    { 0x45, 0x44 },   /*   4 */
    { 0x12, 0x20 },   /*   5 */
    { 0x0C, 0xC8 },   /*   6 */
    { 0x11, 0xC8 },   /*   7 */
    { 0x42, 0xC8 },   /*   8 */
    { 0x43, 0xC8 },   /*   9 */
    { 0x44, 0xC8 },   /*  10 */
    { 0xAF, 0x31 },   /*  11 */
    { 0x69, 0x14 },   /*  12 */
    { 0x23, 0x87 },   /*  13 */
    { 0x26, 0x1A },   /*  14 */
    { 0x27, 0x60 },   /*  15 */
    { 0x29, 0x00 },   /*  16 */
    { 0x2A, 0x30 },   /*  17 */
    { 0x2B, 0x02 },   /*  18 */
    { 0x36, 0x00 },   /*  19 */
    { 0x4C, 0x80 },   /*  20 */
    { 0x4D, 0x1A },   /*  21 */
    { 0x4E, 0x06 },   /*  22 */
    { 0x4F, 0x00 },   /*  23 */
    { 0xEF, 0x02 },   /*  24 */
    { 0x02, 0x64 },   /*  25 */
    { 0x03, 0x08 },   /*  26 */
    { 0x29, 0x64 },   /*  27 */
    { 0x2A, 0x00 },   /*  28 */
    { 0x2B, 0x64 },   /*  29 */
    { 0x2C, 0x00 },   /*  30 */
    { 0x31, 0x64 },   /*  31 */
    { 0x32, 0x00 },   /*  32 */
    { 0x46, 0x00 },   /*  33 */
    { 0x47, 0x46 },   /*  34 */
    { 0x48, 0x00 },   /*  35 */
    { 0x49, 0x01 },   /*  36 */
    { 0x4A, 0x00 },   /*  37 */
    { 0x4B, 0x00 },   /*  38 */
    { 0x5B, 0x01 },   /*  39 */
    { 0xEF, 0x04 },   /*  40 */
    { 0x2C, 0xE3 },   /*  41 */
    { 0x2D, 0x16 },   /*  42 */
    { 0x2E, 0x00 },   /*  43 */
    { 0x30, 0x31 },   /*  44 */
    { 0x31, 0x80 },   /*  45 */
    { 0x40, 0x1D },   /*  46 */
    { 0x41, 0x00 },   /*  47 */
    { 0x42, 0xA0 },   /*  48 */
    { 0x43, 0x03 },   /*  49 */
    { 0x44, 0xF0 },   /*  50 */
    { 0x45, 0x00 },   /*  51 */
    { 0x46, 0x00 },   /*  52 */
    { 0x47, 0x00 },   /*  53 */
    { 0x48, 0x70 },   /*  54 */
    { 0x49, 0xF3 },   /*  55 */
    { 0x4A, 0x05 },   /*  56 */
    { 0x4B, 0x00 },   /*  57 */
    { 0x50, 0x22 },   /*  58 */
    { 0x51, 0x1D },   /*  59 */
    { 0x52, 0x00 },   /*  60 */
    { 0x53, 0xC0 },   /*  61 */
    { 0x54, 0xD4 },   /*  62 */
    { 0x55, 0x01 },   /*  63 */
    { 0x56, 0x00 },   /*  64 */
    { 0xEF, 0x01 },   /*  65 */
    { 0xC4, 0x02 },   /*  66 */
    { 0xC6, 0x40 },   /*  67 */
    { 0xC7, 0x01 },   /*  68 */
    { 0xC8, 0xF0 },   /*  69 */
    { 0xC9, 0x00 },   /*  70 */
    { 0xEF, 0x02 },   /*  71 */
    { 0x11, 0x03 },   /*  72 */
    { 0x19, 0xFC },   /*  73 */
    { 0x1A, 0x00 },   /*  74 */
    { 0x21, 0x40 },   /*  75 */
    { 0x22, 0x01 },   /*  76 */
    { 0x23, 0xF0 },   /*  77 */
    { 0x24, 0x00 },   /*  78 */
    { 0x27, 0x0C },   /*  79 */
    { 0x28, 0x00 },   /*  80 */
    { 0x2D, 0xF0 },   /*  81 */
    { 0x2E, 0x00 },   /*  82 */
    { 0x56, 0x40 },   /*  83 */
    { 0x57, 0x01 },   /*  84 */
    { 0x58, 0xFE },   /*  85 */
    { 0x59, 0x00 },   /*  86 */
    { 0xEF, 0x01 },   /*  87 */
    { 0x33, 0x23 },   /*  88 */
    { 0x3B, 0x18 },   /*  89 */
    { 0x40, 0x50 },   /*  90 */
    { 0xD9, 0x19 },   /*  91 */
    { 0xDB, 0x32 },   /*  92 */
    { 0xDD, 0x32 },   /*  93 */
    { 0xEF, 0x02 },   /*  94 */
    { 0xA5, 0x00 },   /*  95 */
    { 0xA6, 0x50 },   /*  96 */
    { 0xEF, 0x00 },   /*  97 */
    { 0x09, 0x10 },   /*  98 */
    { 0x18, 0x01 },   /*  99 */
    { 0x2F, 0x44 },   /* 100 */
    { 0x37, 0x04 },   /* 101 */
    { 0x38, 0x06 },   /* 102 */
    { 0x3F, 0x01 },   /* 103 */
    { 0x55, 0x01 },   /* 104 */
    { 0x66, 0x01 },   /* 105 */
    { 0xEF, 0x01 },   /* 106 */
    { 0x03, 0x00 },   /* 107 */
    { 0x04, 0xAB },   /* 108 */
    { 0x07, 0x02 },   /* 109 */
    { 0x0A, 0x00 },   /* 110 */
    { 0x0B, 0x4C },   /* 111 */
    { 0x0F, 0x15 },   /* 112 */
    { 0x11, 0x14 },   /* 113 */
    { 0x13, 0x16 },   /* 114 */
    { 0x16, 0x00 },   /* 115 */
    { 0x17, 0xA9 },   /* 116 */
    { 0x36, 0x00 },   /* 117 */
    { 0x37, 0x02 },   /* 118 */
    { 0x4D, 0x02 },   /* 119 */
    { 0x56, 0xB8 },   /* 120 */
    { 0x57, 0x01 },   /* 121 */
    { 0x58, 0xB8 },   /* 122 */
    { 0x59, 0x01 },   /* 123 */
    { 0x62, 0x00 },   /* 124 */
    { 0x63, 0x06 },   /* 125 */
    { 0x69, 0x0F },   /* 126 */
    { 0x6A, 0x0F },   /* 127 */
    { 0x6B, 0x37 },   /* 128 */
    { 0x6C, 0x37 },   /* 129 */
    { 0x76, 0x06 },   /* 130 */
    { 0x77, 0x09 },   /* 131 */
    { 0x78, 0x02 },   /* 132 */
    { 0x79, 0x03 },   /* 133 */
    { 0x84, 0x23 },   /* 134 */
    { 0x86, 0x1E },   /* 135 */
    { 0x87, 0x23 },   /* 136 */
    { 0x89, 0x1E },   /* 137 */
    { 0x8A, 0x3C },   /* 138 */
    { 0x8C, 0x32 },   /* 139 */
    { 0x8D, 0x3C },   /* 140 */
    { 0x8F, 0x32 },   /* 141 */
    { 0x9D, 0x14 },   /* 142 */
    { 0xA0, 0x00 },   /* 143 */
    { 0xD1, 0xB9 },   /* 144 */
    { 0xD2, 0x00 },   /* 145 */
    { 0xEF, 0x02 },   /* 146 */
    { 0x92, 0x11 },   /* 147 */
    { 0x93, 0x01 },   /* 148 */
    { 0xC3, 0xB9 },   /* 149 */
    { 0xC4, 0x00 },   /* 150 */
    { 0xC5, 0xB9 },   /* 151 */
    { 0xC6, 0x00 },   /* 152 */
    { 0xD1, 0x45 },   /* 153 */
    { 0xEF, 0x00 },   /* 154 */
    { 0xEB, 0x80 },   /* 155 */
    { 0x30, 0x01 },   /* 156 */
};
#define INIT_BASE_LEN  (sizeof(init_base) / sizeof(init_base[0]))

/* Section 8.1.3.5 - 320x240 QVGA, SPI out. Applied after init_base. */
static const pag_reg_t init_qvga[] = {
    { 0xEF, 0x00 },   /*   1 */
    { 0xE5, 0x07 },   /*   2 */
    { 0xE5, 0x03 },   /*   3 */
    { 0xE5, 0x01 },   /*   4 */
    { 0xE5, 0x00 },   /*   5 */
    { 0x1D, 0x81 },   /*   6 */
    { 0x64, 0x02 },   /*   7 */
    { 0x0C, 0xC8 },   /*   8 */
    { 0x11, 0xC8 },   /*   9 */
    { 0x42, 0x08 },   /*  10 */
    { 0x43, 0x08 },   /*  11 */
    { 0x44, 0x08 },   /*  12 */
    { 0x0E, 0xC0 },   /*  13 */
    { 0x07, 0x04 },   /*  14 */
    { 0x32, 0x32 },   /*  15 */
    { 0xEF, 0x02 },   /*  16 */
    { 0xC1, 0x4D },   /*  17 */
    { 0xC2, 0x05 },   /*  18 */
    { 0xC7, 0x01 },   /*  19 */
    { 0xEF, 0x00 },   /*  20 */
    { 0x30, 0x01 },   /*  21 */
    { 0xEF, 0x00 },   /*  22 */
    { 0xB0, 0x01 },   /*  23 */
};
#define INIT_QVGA_LEN  (sizeof(init_qvga) / sizeof(init_qvga[0]))

/* Section 8.1.3.6 - 160x120 analog 2x skip, SPI out. After init_base. */
static const pag_reg_t init_qqvga[] = {
    { 0xEF, 0x00 },   /*   1 */
    { 0x09, 0x00 },   /*   2 */
    { 0x15, 0x0C },   /*   3 */
    { 0x16, 0x6D },   /*   4 */
    { 0x1C, 0x00 },   /*   5 */
    { 0x3E, 0x20 },   /*   6 */
    { 0xEF, 0x01 },   /*   7 */
    { 0x4B, 0x11 },   /*   8 */
    { 0x70, 0x10 },   /*   9 */
    { 0xC2, 0x02 },   /*  10 */
    { 0xC3, 0x00 },   /*  11 */
    { 0xC8, 0x78 },   /*  12 */
    { 0xC9, 0x00 },   /*  13 */
    { 0xCB, 0x04 },   /*  14 */
    { 0xCC, 0x00 },   /*  15 */
    { 0xCE, 0x00 },   /*  16 */
    { 0xEF, 0x02 },   /*  17 */
    { 0x19, 0x81 },   /*  18 */
    { 0x1A, 0x00 },   /*  19 */
    { 0x23, 0x78 },   /*  20 */
    { 0x24, 0x00 },   /*  21 */
    { 0x27, 0x09 },   /*  22 */
    { 0x28, 0x00 },   /*  23 */
    { 0x2D, 0x78 },   /*  24 */
    { 0x2E, 0x00 },   /*  25 */
    { 0xD9, 0x01 },   /*  26 */
    { 0xEF, 0x04 },   /*  27 */
    { 0x3A, 0x28 },   /*  28 */
    { 0x3B, 0x1E },   /*  29 */
    { 0xEF, 0x00 },   /*  30 */
    { 0xEB, 0x80 },   /*  31 */
    { 0x30, 0x01 },   /*  32 */
    { 0xE5, 0x07 },   /*  33 */
    { 0xE5, 0x03 },   /*  34 */
    { 0xE5, 0x01 },   /*  35 */
    { 0xE5, 0x00 },   /*  36 */
    { 0x1D, 0x81 },   /*  37 */
    { 0x64, 0x02 },   /*  38 */
    { 0x0C, 0xC8 },   /*  39 */
    { 0x11, 0xC8 },   /*  40 */
    { 0x42, 0x08 },   /*  41 */
    { 0x43, 0x08 },   /*  42 */
    { 0x44, 0x08 },   /*  43 */
    { 0x0E, 0xC0 },   /*  44 */
    { 0x07, 0x04 },   /*  45 */
    { 0x32, 0x32 },   /*  46 */
    { 0xEF, 0x02 },   /*  47 */
    { 0xC1, 0x4D },   /*  48 */
    { 0xC2, 0x05 },   /*  49 */
    { 0xC7, 0x01 },   /*  50 */
    { 0xEF, 0x00 },   /*  51 */
    { 0x7A, 0x10 },   /*  52 */
    { 0x30, 0x01 },   /*  53 */
    { 0xEF, 0x00 },   /*  54 */
    { 0xB0, 0x01 },   /*  55 */
};
#define INIT_QQVGA_LEN  (sizeof(init_qqvga) / sizeof(init_qqvga[0]))

/* -------------------------------------------------------------- */
/* Private helpers                                                 */
/* -------------------------------------------------------------- */

/* The vendor tables start the sensor 24 writes before they end, so the last
 * writes land on a running sensor and a few do not ACK first time: the bench
 * measured 3-22 retried writes per init. Retry each one, as the validator
 * does (8 tries), rather than failing init on the first NACK. The HAL only
 * sleeps in milliseconds; the validator's 200 us gap is a lower bound. */
#define PAG_WRITE_TRIES  8

static int pag_write(tile_t* tile, uint8_t reg, uint8_t value)
{
    int rc = -1;
    for (uint8_t t = 0; t < PAG_WRITE_TRIES; t++) {
        rc = tile->hal->i2c_write(tile->hal->handle, tile->id, reg, &value, 1);
        if (rc == 0) return 0;
        tile->hal->delay_ms(1);
    }
    return rc;
}

/** One register over I2C. Returns 0 and sets *val, or non-zero on a bus error. */
static int pag_read(tile_t* tile, uint8_t reg, uint8_t* val)
{
    *val = 0;
    return tile->hal->i2c_read(tile->hal->handle, tile->id, reg, val, 1);
}

/** @brief  Apply one vendor table in order. Returns writes that did not ACK. */
static uint16_t apply_table(tile_t* tile, const pag_reg_t* tbl, uint32_t len)
{
    uint16_t failed = 0;
    for (uint32_t i = 0; i < len; i++)
        if (pag_write(tile, tbl[i].reg, tbl[i].val) != 0) failed++;
    return failed;
}

/**
 * @brief  Read one SPI register.
 *
 * Byte 0 is [RD | addr6:0] - the address is only 7 bits over SPI because
 * bit 7 is the direction flag, so the SPI and I2C register maps are not
 * addressed identically.
 */
static int spi_read_reg(tile_t* tile, uint8_t reg, uint8_t* v)
{
    *v = 0;
    return tile->hal->spi_read(tile->hal->handle, s_spi_cs,
                               (uint8_t)(reg | PAG7920_SPI_RD_BIT), v, 1);
}

static int spi_write_reg(tile_t* tile, uint8_t reg, uint8_t val)
{
    return tile->hal->spi_write(tile->hal->handle, s_spi_cs,
                                (uint8_t)(reg & 0x7F), &val, 1);
}

/* Poll a CPU_INT0_Status bit until it sets, within `timeout_ms` of real
 * time. The PAL has no clock, so time is counted in the 1 ms sleeps taken
 * every POLL_BATCH polls: a lower bound on the time spent, never an
 * overcount. FB_Ovf seen along the way is reported through *ovf.
 * Returns 1 when the bit set, 0 on timeout or a bus error. */
#define POLL_BATCH  256
static uint8_t wait_status(tile_t* tile, uint8_t bit, uint32_t timeout_ms,
                           uint8_t* ovf)
{
    uint32_t waited = 0;
    for (uint32_t n = 1; ; n++) {
        uint8_t st;
        if (spi_read_reg(tile, PAG7920_SPI_INT_STATUS, &st) != 0) return 0;
        if (ovf && (st & PAG7920_INT_FB_OVF)) *ovf = 1;
        if (st & bit) return 1;
        if (n % POLL_BATCH == 0) {
            if (waited >= timeout_ms) return 0;
            tile->hal->delay_ms(1);
            waited++;
        }
    }
}

/* -------------------------------------------------------------- */
/* Public API                                                      */
/* -------------------------------------------------------------- */

uint8_t tile_sense_cam_p_find(tiles_pal_t* hal, uint8_t instance)
{
    uint8_t id = resolve_id(instance);
    if (id == 0x00) return 0;
    return (hal->i2c_is_ready(hal->handle, id) == 0) ? 1 : 0;
}

uint16_t tile_sense_cam_p_part_id(tile_t* tile)
{
    if (tile->hal == NULL) return 0;
    uint8_t lo, hi;
    if (pag_write(tile, PAG7920_REG_BANK, PAG7920_BANK_0) != 0 ||
        pag_read(tile, PAG7920_REG_PARTID_L, &lo) != 0 ||
        pag_read(tile, PAG7920_REG_PARTID_H, &hi) != 0)
        return 0;   /* a partial read is not an ID */
    return (uint16_t)((hi << 8) | lo);
}

uint16_t tile_sense_cam_p_spi_id(tile_t* tile)
{
    if (tile->hal == NULL || tile->hal->spi_read == NULL) return 0;
    uint8_t lo, hi;
    if (spi_read_reg(tile, PAG7920_SPI_CHECKID_L, &lo) != 0 ||
        spi_read_reg(tile, PAG7920_SPI_CHECKID_H, &hi) != 0)
        return 0;
    return (uint16_t)((hi << 8) | lo);
}

uint16_t tile_sense_cam_p_width(tile_t* tile)
{
    (void)tile;
    return (s_res == SENSE_CAM_P_RES_320x240) ? 320 : 160;
}

uint16_t tile_sense_cam_p_height(tile_t* tile)
{
    (void)tile;
    return (s_res == SENSE_CAM_P_RES_320x240) ? 240 : 120;
}

uint32_t tile_sense_cam_p_frame_bytes(tile_t* tile)
{
    return (uint32_t)tile_sense_cam_p_width(tile) *
           (uint32_t)tile_sense_cam_p_height(tile);
}

void tile_sense_cam_p_reset(tile_t* tile)
{
    if (tile->hal == NULL) return;
    pag_write(tile, PAG7920_REG_BANK, PAG7920_BANK_0);
    pag_write(tile, PAG7920_REG_SOFT_RESET, 0xFF);
    tile->hal->delay_ms(60);   /* >30 ms, matching power-on setup time */
    tile->state = TILE_STATE_NONE;
}

void tile_sense_cam_p_init(tiles_pal_t* hal, uint8_t instance, tile_t* tile,
                           const sense_cam_p_cfg_t* cfg)
{
    tile->hal      = NULL;
    tile->id       = 0;
    tile->state    = TILE_STATE_NONE;
    tile->flags    = 0;
    tile->callback = NULL;
    tile->cb_ctx   = NULL;

    s_spi_cs = (cfg != NULL) ? cfg->spi_cs : 0;
    s_res    = (cfg != NULL) ? cfg->resolution : SENSE_CAM_P_RES_160x120;

    uint8_t id = resolve_id(instance);
    if (id == 0x00) {
        TILE_ON_ERROR(tile, "init: invalid instance");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    tile->hal = hal;
    tile->id  = id;

    if (hal->spi_read == NULL || hal->spi_write == NULL) {
        TILE_ON_ERROR(tile, "init: PAL has no SPI; image output needs it");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    if (hal->i2c_is_ready(hal->handle, id) != 0) {
        TILE_ON_ERROR(tile, "init: device not found on bus");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    if (tile_sense_cam_p_part_id(tile) != PAG7920_PART_ID) {
        TILE_ON_ERROR(tile, "init: unexpected part id");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    /* Start from a known state rather than trusting whatever the last
     * session left behind - the tile's bulk capacitance means a power cycle
     * may not have reset the sensor at all. */
    tile_sense_cam_p_reset(tile);

    /* VERBATIM, IN ORDER. See the header. */
    uint16_t bad = apply_table(tile, init_base, INIT_BASE_LEN);
    bad += (s_res == SENSE_CAM_P_RES_320x240)
             ? apply_table(tile, init_qvga,  INIT_QVGA_LEN)
             : apply_table(tile, init_qqvga, INIT_QQVGA_LEN);

    if (bad != 0) {
        TILE_ON_ERROR(tile, "init: register writes did not ACK");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    hal->delay_ms(50);

    /* The SPI register interface only answers while the sensor is running,
     * which the tables have already arranged. Verify it now, before any
     * later mode change could stop the sensor and take SPI down with it. */
    if (tile_sense_cam_p_spi_id(tile) != PAG7920_SPI_CHECK_ID) {
        TILE_ON_ERROR(tile, "init: SPI link did not come up");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    tile->state = TILE_STATE_READY;
}

uint8_t tile_sense_cam_p_capture(tile_t* tile, uint8_t* dst, uint32_t len)
{
    if (tile->hal == NULL || tile->state != TILE_STATE_READY) return 0;
    if (dst == NULL) return 0;

    uint32_t need = tile_sense_cam_p_frame_bytes(tile);
    if (len < need) {
        TILE_ON_ERROR(tile, "capture: destination too small for one frame");
        return 0;
    }

    const uint32_t cycles = need / PAG7920_BUFFER_BYTES;

    /* Frame-lock. Discarding buffers is not enough - it leaves an arbitrary
     * phase and the image comes back rotated by whole 4800-byte buffers.
     * Clear all three flags first so a stale assertion is not mistaken for a
     * fresh frame boundary. A frame is 33 ms at the configured 30 fps; the
     * validator allows 1000 ms, and so does this. */
    if (spi_write_reg(tile, PAG7920_SPI_INT_STATUS, 0xF8) != 0 ||
        !wait_status(tile, PAG7920_INT_FRAME_START, 1000, NULL)) {
        TILE_ON_ERROR(tile, "capture: no Frame_Start");
        return 0;
    }
    spi_write_reg(tile, PAG7920_SPI_INT_STATUS, 0xFD);

    uint8_t overflowed = 0;
    for (uint32_t c = 0; c < cycles; c++) {
        if (!wait_status(tile, PAG7920_INT_FB_RDY, 500, &overflowed)) {
            TILE_ON_ERROR(tile, "capture: timed out waiting for buffer");
            return 0;
        }

        /* Datasheet asks for 1.5 us after Img_Rd_En; delay_ms(0) is not
         * granular enough, and the SPI transaction setup already exceeds it. */
        int rc = spi_write_reg(tile, PAG7920_SPI_IMG_RD_EN, 0x01);
        rc |= tile->hal->spi_read(tile->hal->handle, s_spi_cs,
                                  (uint8_t)(PAG7920_SPI_IMG_DATA | PAG7920_SPI_RD_BIT),
                                  dst + (c * PAG7920_BUFFER_BYTES),
                                  PAG7920_BUFFER_BYTES);
        rc |= spi_write_reg(tile, PAG7920_SPI_IMG_RD_EN, 0x00);
        rc |= spi_write_reg(tile, PAG7920_SPI_INT_STATUS, 0xFE);
        if (rc != 0) {
            TILE_ON_ERROR(tile, "capture: SPI transfer failed");
            return 0;
        }
    }

    /* An overflow during the last buffer shows up only after it; look once
     * more before calling the frame good. */
    uint8_t st;
    if (spi_read_reg(tile, PAG7920_SPI_INT_STATUS, &st) == 0 && (st & PAG7920_INT_FB_OVF))
        overflowed = 1;

    if (overflowed) {
        TILE_ON_ERROR(tile, "capture: FB_Ovf - frame torn, readout too slow");
        return 0;
    }
    return 1;
}
