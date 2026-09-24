/**
 * @file   tile_sense_i_6p6.c
 * @brief  Sense.I.6P6 (ICM-42686-P) — complete driver implementation.
 *
 * Platform-agnostic. All bus access via tile->hal function pointers.
 * Reference: TDK InvenSense DS-000639 Rev 1.0.
 */

#include "tile_sense_i_6p6.h"
#include <stddef.h>

/* ================================================================
 * Instance → I2C address table
 * ================================================================ */

static const uint8_t id_table[] = {
    ICM42686P_I2C_ADDR_DEFAULT,   /* 0: 0x69 (AD0 high — default on tile) */
    ICM42686P_I2C_ADDR_ALT,       /* 1: 0x68 (AD0 tied to GND) */
};

#define NUM_INSTANCES  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    return (instance < NUM_INSTANCES) ? id_table[instance] : 0;
}

/* ================================================================
 * Per-instance driver state
 * ================================================================ */

typedef struct {
    sense_i_6p6_event_cb_t on_event;
    void *event_ctx;
    volatile uint8_t int1_flag;
    uint8_t int1_pin;
    uint8_t int2_pin;
} icm_state_t;

static icm_state_t icm_state[NUM_INSTANCES];

static icm_state_t *state_for(tile_t *tile)
{
    if (tile->hal->buses & TILES_BUS_SPI) {
        /* SPI: id is CS index, use directly (clamped to array) */
        uint8_t idx = tile->id < NUM_INSTANCES ? tile->id : 0;
        return &icm_state[idx];
    }
    for (uint8_t i = 0; i < NUM_INSTANCES; i++)
        if (id_table[i] == tile->id) return &icm_state[i];
    return &icm_state[0];
}

/* INT1 pin EXTI callbacks */
static void _int1_isr_0(void *ctx) { icm_state[0].int1_flag = 1; (void)ctx; }
static void _int1_isr_1(void *ctx) { icm_state[1].int1_flag = 1; (void)ctx; }

/* ================================================================
 * Private helpers — bus-agnostic register access
 *
 * Dispatches to SPI or I2C based on hal->buses. For SPI, tile->id
 * holds the CS index. For I2C, it holds the 7-bit device address.
 * The SPI HAL callbacks handle the read/write bit (0x80) internally.
 * ================================================================ */

static void icm_write(tile_t *tile, uint8_t reg, uint8_t val)
{
    if (tile->hal->buses & TILES_BUS_SPI)
        tile->hal->spi_write(tile->hal->handle, tile->id, reg, &val, 1);
    else
        tile->hal->i2c_write(tile->hal->handle, tile->id, reg, &val, 1);
}

static uint8_t icm_read(tile_t *tile, uint8_t reg)
{
    uint8_t val = 0;
    if (tile->hal->buses & TILES_BUS_SPI)
        tile->hal->spi_read(tile->hal->handle, tile->id, reg, &val, 1);
    else
        tile->hal->i2c_read(tile->hal->handle, tile->id, reg, &val, 1);
    return val;
}

static void icm_read_buf(tile_t *tile, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (tile->hal->buses & TILES_BUS_SPI)
        tile->hal->spi_read(tile->hal->handle, tile->id, reg, buf, len);
    else
        tile->hal->i2c_read(tile->hal->handle, tile->id, reg, buf, len);
}

static void icm_set_bank(tile_t *tile, uint8_t bank)
{
    icm_write(tile, ICM42686P_REG_BANK_SEL, bank);
}

static void icm_modify(tile_t *tile, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t r = icm_read(tile, reg);
    r = (r & ~mask) | (val & mask);
    icm_write(tile, reg, r);
}

static int16_t swap16(uint8_t h, uint8_t l)
{
    return (int16_t)((uint16_t)h << 8 | l);
}

static void memzero(void *p, uint8_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) *b++ = 0;
}

/* ================================================================
 * Lifecycle
 * ================================================================ */

/** @brief Check if a Sense.I.6P6 is present on the bus. */
uint8_t tile_sense_i_6p6_find(tiles_pal_t *hal, uint8_t instance)
{
    if (hal->buses & TILES_BUS_SPI) {
        /* SPI: no address probe — do a WHO_AM_I read via CS index */
        uint8_t who = 0;
        hal->spi_read(hal->handle, instance, ICM42686P_REG_WHO_AM_I, &who, 1);
        return (who == 0x44 || who == 0x47);
    }
    uint8_t addr = resolve_id(instance);
    if (!addr) return 0;
    return hal->i2c_is_ready(hal->handle, addr) == 0;
}

/** @brief Initialize the ICM-42686P. */
void tile_sense_i_6p6_init(tiles_pal_t *hal, uint8_t instance,
                           tile_t *tile, const sense_i_6p6_cfg_t *cfg)
{
    memzero(tile, sizeof(tile_t));
    tile->hal = hal;

    if (hal->buses & TILES_BUS_SPI) {
        /* SPI: id = CS index (instance maps directly) */
        tile->id = instance;
    } else {
        /* I2C: id = 7-bit address from instance table */
        tile->id = resolve_id(instance);
        if (!tile->id) {
            tile->state = TILE_STATE_ERROR;
            TILE_ON_ERROR(tile, "sense_i_6p6: invalid instance");
            return;
        }
    }

    /* Initialize per-instance state */
    icm_state_t *s = state_for(tile);
    memzero(s, sizeof(icm_state_t));
    if (cfg) {
        s->on_event = cfg->on_event;
        s->event_ctx = cfg->event_ctx;
        s->int1_pin = cfg->int1_pin;
        s->int2_pin = cfg->int2_pin;
    }

    /* Probe bus */
    if (hal->buses & TILES_BUS_SPI) {
        /* SPI: no address probe — verified by WHO_AM_I below */
    } else {
        if (hal->i2c_is_ready(hal->handle, tile->id) != 0) {
            tile->state = TILE_STATE_ERROR;
            TILE_ON_ERROR(tile, "sense_i_6p6: device not found");
            return;
        }
    }

    /* Soft reset */
    icm_write(tile, ICM42686P_REG_DEVICE_CONFIG, 0x01);
    hal->delay_ms(10);  /* Datasheet: 1ms min; extra margin for SPI mode switch */

    /* Read WHO_AM_I — store in flags field for debug.
     * Expected 0x44 (ICM-42686-P) or 0x47 (ICM-42688-P).
     * Accept any non-0x00/0xFF value (device is responding). */
    uint8_t who = icm_read(tile, ICM42686P_REG_WHO_AM_I);
    tile->flags = who;
    if (who == 0x00 || who == 0xFF) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_i_6p6: WHO_AM_I read failed");
        return;
    }

    /* Ensure bank 0 */
    icm_set_bank(tile, ICM42686P_BANK_0);

    /* SPI output drive strength (datasheet section 12.3):
     * SPI_SLEW_RATE must be set to 5 for SPI operation (default is 1 = I2C). */
    if (hal->buses & TILES_BUS_SPI) {
        icm_modify(tile, ICM42686P_REG_DRIVE_CONFIG, 0x07, 0x05);
    }

    /* INT_CONFIG1: clear INT_ASYNC_RESET (bit 4) for proper INT operation */
    icm_modify(tile, ICM42686P_REG_INT_CONFIG1, 0x10, 0x00);

    /* INTF_CONFIG0.FIFO_COUNT_REC (bit 6) = 1: FIFO_COUNT and FIFO_WM are in
     * records (packets), not bytes (DS-000639 §14.34, §14.46). The FIFO API
     * (fifo_count, fifo_set_watermark, fifo_read_packets) is documented and
     * used in records; the reset value 0 would make them bytes. */
    icm_modify(tile, ICM42686P_REG_INTF_CONFIG0, 0x40, 0x40);

    /* Apply config or defaults */
    uint8_t accel_range = (cfg && cfg->accel_range) ? cfg->accel_range : SENSE_I_6P6_ACCEL_8G;
    uint8_t gyro_range  = (cfg && cfg->gyro_range)  ? cfg->gyro_range  : SENSE_I_6P6_GYRO_1000DPS;
    uint8_t odr         = (cfg && cfg->odr)         ? cfg->odr         : SENSE_I_6P6_ODR_100HZ;

    icm_write(tile, ICM42686P_REG_ACCEL_CONFIG0, (accel_range << 5) | odr);
    icm_write(tile, ICM42686P_REG_GYRO_CONFIG0, (gyro_range << 5) | odr);

    /* Enable accel + gyro in low-noise mode */
    icm_write(tile, ICM42686P_REG_PWR_MGMT0,
              ICM42686P_PWR_ACCEL_LN | ICM42686P_PWR_GYRO_LN);
    hal->delay_ms(1);  /* >200 µs after mode change */

    /* Set up INT1 pin interrupt if configured */
    if (s->int1_pin && hal->gpio_irq_enable) {
        /* Configure INT1: push-pull, active high, pulsed */
        icm_write(tile, ICM42686P_REG_INT_CONFIG, 0x03);
        void (*isr)(void *) = (instance == 0) ? _int1_isr_0 : _int1_isr_1;
        hal->gpio_irq_enable(hal->handle, s->int1_pin,
                             TILES_GPIO_EDGE_RISING, isr, NULL);
    }

    tile->state = TILE_STATE_READY;
}

/* ================================================================
 * Event processing
 * ================================================================ */

/** @brief Process pending interrupt events. */
void tile_sense_i_6p6_process(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY) return;

    icm_state_t *s = state_for(tile);

    /* In interrupt mode, skip if no INT1 fired */
    if (s->int1_pin && !s->int1_flag)
        return;
    s->int1_flag = 0;

    /* Read and clear INT_STATUS */
    uint8_t status = icm_read(tile, ICM42686P_REG_INT_STATUS);
    if (status && s->on_event) {
        s->on_event(tile, status, s->event_ctx);
    }
}

/** @brief Register or change the event callback. */
void tile_sense_i_6p6_on_event(tile_t *tile, sense_i_6p6_event_cb_t cb, void *ctx)
{
    icm_state_t *s = state_for(tile);
    s->on_event = cb;
    s->event_ctx = ctx;
}

/** @brief Enter sleep mode (accel + gyro off). */
void tile_sense_i_6p6_sleep(tile_t *tile)
{
    icm_write(tile, ICM42686P_REG_PWR_MGMT0,
              ICM42686P_PWR_ACCEL_OFF | ICM42686P_PWR_GYRO_OFF);
    tile->state = TILE_STATE_SLEEPING;
}

/** @brief Wake from sleep, restore low-noise mode. */
void tile_sense_i_6p6_wake(tile_t *tile)
{
    icm_write(tile, ICM42686P_REG_PWR_MGMT0,
              ICM42686P_PWR_ACCEL_LN | ICM42686P_PWR_GYRO_LN);
    tile->hal->delay_ms(1);
    tile->state = TILE_STATE_READY;
}

/** @brief Software reset. Must call init() again after. */
void tile_sense_i_6p6_reset(tile_t *tile)
{
    icm_write(tile, ICM42686P_REG_DEVICE_CONFIG, 0x01);
    tile->hal->delay_ms(2);
    tile->state = TILE_STATE_NONE;
}

/* ================================================================
 * Configuration
 * ================================================================ */

/** @brief Set accelerometer full-scale range. */
void tile_sense_i_6p6_set_accel_range(tile_t *tile, sense_i_6p6_accel_range_t range)
{
    icm_modify(tile, ICM42686P_REG_ACCEL_CONFIG0, 0xE0, (uint8_t)range << 5);
}

/** @brief Set gyroscope full-scale range. */
void tile_sense_i_6p6_set_gyro_range(tile_t *tile, sense_i_6p6_gyro_range_t range)
{
    icm_modify(tile, ICM42686P_REG_GYRO_CONFIG0, 0xE0, (uint8_t)range << 5);
}

/** @brief Set accelerometer output data rate. */
void tile_sense_i_6p6_set_accel_odr(tile_t *tile, sense_i_6p6_odr_t odr)
{
    icm_modify(tile, ICM42686P_REG_ACCEL_CONFIG0, 0x0F, (uint8_t)odr);
}

/** @brief Set gyroscope output data rate. */
void tile_sense_i_6p6_set_gyro_odr(tile_t *tile, sense_i_6p6_odr_t odr)
{
    icm_modify(tile, ICM42686P_REG_GYRO_CONFIG0, 0x0F, (uint8_t)odr);
}

/** @brief Set power mode independently for accel and gyro. */
void tile_sense_i_6p6_set_power_mode(tile_t *tile,
                                      sense_i_6p6_power_mode_t accel,
                                      sense_i_6p6_power_mode_t gyro)
{
    uint8_t pwr = ((uint8_t)gyro << 2) | (uint8_t)accel;
    icm_modify(tile, ICM42686P_REG_PWR_MGMT0, 0x0F, pwr);
    tile->hal->delay_ms(1);  /* >200 µs */
}

/** @brief Set UI filter bandwidth for both accel and gyro. */
void tile_sense_i_6p6_set_filter_bw(tile_t *tile,
                                     sense_i_6p6_filter_bw_t accel_bw,
                                     sense_i_6p6_filter_bw_t gyro_bw)
{
    icm_write(tile, ICM42686P_REG_GYRO_ACCEL_CFG0,
              ((uint8_t)accel_bw << 4) | (uint8_t)gyro_bw);
}

/** @brief Set UI filter order for accel and gyro. */
void tile_sense_i_6p6_set_filter_order(tile_t *tile,
                                        sense_i_6p6_filter_order_t accel_order,
                                        sense_i_6p6_filter_order_t gyro_order)
{
    icm_modify(tile, ICM42686P_REG_GYRO_CONFIG1, 0x0C, (uint8_t)gyro_order << 2);
    icm_modify(tile, ICM42686P_REG_ACCEL_CONFIG1, 0x18, (uint8_t)accel_order << 3);
}

/** @brief Set temperature sensor filter bandwidth. */
void tile_sense_i_6p6_set_temp_filter(tile_t *tile, sense_i_6p6_temp_filter_t bw)
{
    icm_modify(tile, ICM42686P_REG_GYRO_CONFIG1, 0xE0, (uint8_t)bw << 5);
}

/** @brief Enable or disable the temperature sensor. */
void tile_sense_i_6p6_set_temp_enabled(tile_t *tile, uint8_t enabled)
{
    icm_modify(tile, ICM42686P_REG_PWR_MGMT0, 0x20, enabled ? 0x00 : 0x20);
}

/* ================================================================
 * Data reads
 * ================================================================ */

/** @brief Check if new sensor data is available. */
uint8_t tile_sense_i_6p6_data_ready(tile_t *tile)
{
    return (icm_read(tile, ICM42686P_REG_INT_STATUS) & ICM42686P_INT_DATA_RDY) != 0;
}

/** @brief Read raw accelerometer [X, Y, Z]. */
void tile_sense_i_6p6_get_raw_accels(tile_t *tile, int16_t *buffer)
{
    uint8_t raw[6];
    icm_read_buf(tile, ICM42686P_REG_ACCEL_X_H, raw, 6);
    buffer[0] = swap16(raw[0], raw[1]);
    buffer[1] = swap16(raw[2], raw[3]);
    buffer[2] = swap16(raw[4], raw[5]);
}

/** @brief Read raw gyroscope [X, Y, Z]. */
void tile_sense_i_6p6_get_raw_gyros(tile_t *tile, int16_t *buffer)
{
    uint8_t raw[6];
    icm_read_buf(tile, ICM42686P_REG_GYRO_X_H, raw, 6);
    buffer[0] = swap16(raw[0], raw[1]);
    buffer[1] = swap16(raw[2], raw[3]);
    buffer[2] = swap16(raw[4], raw[5]);
}

/** @brief Burst read accel + gyro [AX, AY, AZ, GX, GY, GZ]. */
void tile_sense_i_6p6_get_raw_6dof(tile_t *tile, int16_t *buffer)
{
    uint8_t raw[12];
    icm_read_buf(tile, ICM42686P_REG_ACCEL_X_H, raw, 12);
    buffer[0] = swap16(raw[0],  raw[1]);
    buffer[1] = swap16(raw[2],  raw[3]);
    buffer[2] = swap16(raw[4],  raw[5]);
    buffer[3] = swap16(raw[6],  raw[7]);
    buffer[4] = swap16(raw[8],  raw[9]);
    buffer[5] = swap16(raw[10], raw[11]);
}

/** @brief Read temperature raw value. */
int16_t tile_sense_i_6p6_get_temperature(tile_t *tile)
{
    uint8_t raw[2];
    icm_read_buf(tile, ICM42686P_REG_TEMP_H, raw, 2);
    return swap16(raw[0], raw[1]);
}

/** @brief Read all 7 channels [Temp, AX, AY, AZ, GX, GY, GZ]. */
void tile_sense_i_6p6_get_raw_all(tile_t *tile, int16_t *buffer)
{
    /* Read temperature separately, then burst accel+gyro from 0x1F.
     * A single 14-byte burst starting at TEMP_H (0x1D) is unreliable
     * over SPI on the ICM-42686P — subsequent reads return 0xFF.
     * Splitting into temp + 6dof avoids this. */
    uint8_t tmp[2];
    icm_read_buf(tile, ICM42686P_REG_TEMP_H, tmp, 2);
    buffer[0] = swap16(tmp[0], tmp[1]);

    uint8_t raw[12];
    icm_read_buf(tile, ICM42686P_REG_ACCEL_X_H, raw, 12);
    for (int i = 0; i < 6; i++)
        buffer[i + 1] = swap16(raw[i * 2], raw[i * 2 + 1]);
}

/* ================================================================
 * FIFO
 * ================================================================ */

/** @brief Configure the FIFO. */
void tile_sense_i_6p6_fifo_config(tile_t *tile, sense_i_6p6_fifo_mode_t mode,
                                   uint8_t accel, uint8_t gyro,
                                   uint8_t temp, uint8_t hires)
{
    /* FIFO_CONFIG: mode in bits [7:6] */
    icm_write(tile, ICM42686P_REG_FIFO_CONFIG, (uint8_t)mode << 6);

    /* FIFO_CONFIG1: data selection */
    uint8_t cfg1 = 0;
    if (hires) cfg1 |= (1 << 4);
    if (temp)  cfg1 |= (1 << 2);
    if (gyro)  cfg1 |= (1 << 1);
    if (accel) cfg1 |= (1 << 0);
    icm_write(tile, ICM42686P_REG_FIFO_CONFIG1, cfg1);
}

/** @brief Set the FIFO watermark threshold in records. */
void tile_sense_i_6p6_fifo_set_watermark(tile_t *tile, uint16_t records)
{
    if (records == 0) records = 1;  /* 0 is invalid per datasheet */
    icm_write(tile, ICM42686P_REG_FIFO_CONFIG2, (uint8_t)(records & 0xFF));
    icm_write(tile, ICM42686P_REG_FIFO_CONFIG3, (uint8_t)((records >> 8) & 0x0F));
}

/** @brief Flush the FIFO (discard all data). */
void tile_sense_i_6p6_fifo_flush(tile_t *tile)
{
    icm_write(tile, ICM42686P_REG_SIGNAL_PATH_RESET, ICM42686P_FIFO_FLUSH);
}

/** @brief Read the FIFO record count. */
uint16_t tile_sense_i_6p6_fifo_count(tile_t *tile)
{
    uint8_t raw[2];
    /* Must read COUNTL first to latch both bytes */
    raw[1] = icm_read(tile, ICM42686P_REG_FIFO_COUNTL);
    raw[0] = icm_read(tile, ICM42686P_REG_FIFO_COUNTH);
    return ((uint16_t)raw[0] << 8) | raw[1];
}

/** @brief Read one standard FIFO packet (16 bytes). */
uint8_t tile_sense_i_6p6_fifo_read_packet(tile_t *tile, sense_i_6p6_fifo_packet_t *pkt)
{
    uint8_t raw[16];
    icm_read_buf(tile, ICM42686P_REG_FIFO_DATA, raw, 16);

    pkt->header = raw[0];

    if (pkt->header & SENSE_I_6P6_FIFO_HEADER_EMPTY)
        return 0;

    /* Standard packet: header + accel(6) + gyro(6) + temp(1) + timestamp(2) */
    pkt->accel[0] = swap16(raw[1],  raw[2]);
    pkt->accel[1] = swap16(raw[3],  raw[4]);
    pkt->accel[2] = swap16(raw[5],  raw[6]);
    pkt->gyro[0]  = swap16(raw[7],  raw[8]);
    pkt->gyro[1]  = swap16(raw[9],  raw[10]);
    pkt->gyro[2]  = swap16(raw[11], raw[12]);
    pkt->temp     = (int8_t)raw[13];
    pkt->timestamp = ((uint16_t)raw[14] << 8) | raw[15];

    return 1;
}

/** @brief Flat-output variant — fills int32_t out[8]. */
void tile_sense_i_6p6_fifo_read_packet_flat(tile_t *tile, int32_t *out)
{
    sense_i_6p6_fifo_packet_t pkt = {0};
    if (!tile_sense_i_6p6_fifo_read_packet(tile, &pkt)) return;
    if (!out) return;
    out[0] = (int32_t)pkt.accel[0];
    out[1] = (int32_t)pkt.accel[1];
    out[2] = (int32_t)pkt.accel[2];
    out[3] = (int32_t)pkt.gyro[0];
    out[4] = (int32_t)pkt.gyro[1];
    out[5] = (int32_t)pkt.gyro[2];
    out[6] = (int32_t)pkt.temp;
    out[7] = (int32_t)pkt.timestamp;
}

/** @brief Read multiple FIFO packets. */
uint16_t tile_sense_i_6p6_fifo_read_packets(tile_t *tile,
                                              sense_i_6p6_fifo_packet_t *packets,
                                              uint16_t max_count)
{
    uint16_t count = tile_sense_i_6p6_fifo_count(tile);
    if (count > max_count) count = max_count;

    uint16_t read = 0;
    for (uint16_t i = 0; i < count; i++) {
        if (!tile_sense_i_6p6_fifo_read_packet(tile, &packets[i]))
            break;
        read++;
    }
    return read;
}

/** @brief Flat-output variant — drains up to (cap_ints / 8) packets
 *  into a flat int32 buffer (8 ints per packet, same layout as
 *  fifo_read_packet_flat). Returns the number of packets read. */
uint16_t tile_sense_i_6p6_fifo_read_packets_flat(tile_t *tile,
                                                  int32_t *out,
                                                  uint16_t cap_ints)
{
    if (!out || cap_ints < 8) return 0;
    uint16_t max_packets = cap_ints / 8;
    uint16_t available = tile_sense_i_6p6_fifo_count(tile);
    if (max_packets > available) max_packets = available;

    uint16_t read = 0;
    for (uint16_t i = 0; i < max_packets; i++) {
        sense_i_6p6_fifo_packet_t pkt;
        if (!tile_sense_i_6p6_fifo_read_packet(tile, &pkt)) break;
        const uint16_t off = i * 8;
        out[off + 0] = (int32_t)pkt.accel[0];
        out[off + 1] = (int32_t)pkt.accel[1];
        out[off + 2] = (int32_t)pkt.accel[2];
        out[off + 3] = (int32_t)pkt.gyro[0];
        out[off + 4] = (int32_t)pkt.gyro[1];
        out[off + 5] = (int32_t)pkt.gyro[2];
        out[off + 6] = (int32_t)pkt.temp;
        out[off + 7] = (int32_t)pkt.timestamp;
        read++;
    }
    return read;
}

/** @brief Get the number of lost FIFO packets since last check. */
uint16_t tile_sense_i_6p6_fifo_lost_count(tile_t *tile)
{
    uint8_t lo = icm_read(tile, ICM42686P_REG_FIFO_LOST_PKT0);
    uint8_t hi = icm_read(tile, ICM42686P_REG_FIFO_LOST_PKT1);
    return ((uint16_t)hi << 8) | lo;
}

/* ================================================================
 * Interrupts
 * ================================================================ */

/** @brief Configure INT1 pin behavior. */
void tile_sense_i_6p6_int1_config(tile_t *tile, uint8_t config)
{
    /* INT_CONFIG bits [2:0] = INT1_POLARITY, INT1_DRIVE, INT1_MODE */
    icm_modify(tile, ICM42686P_REG_INT_CONFIG, 0x07, config);
}

/** @brief Route data-ready interrupt to INT1. */
void tile_sense_i_6p6_int1_data_ready(tile_t *tile, uint8_t enabled)
{
    icm_modify(tile, ICM42686P_REG_INT_SOURCE0, 0x08, enabled ? 0x08 : 0x00);
}

/** @brief Route FIFO watermark interrupt to INT1. */
void tile_sense_i_6p6_int1_fifo_ths(tile_t *tile, uint8_t enabled)
{
    icm_modify(tile, ICM42686P_REG_INT_SOURCE0, 0x04, enabled ? 0x04 : 0x00);
}

/** @brief Route WOM (wake-on-motion) interrupt to INT1. */
void tile_sense_i_6p6_int1_wom(tile_t *tile, uint8_t enabled)
{
    uint8_t val = enabled ? 0x07 : 0x00;  /* WOM_X, WOM_Y, WOM_Z */
    icm_modify(tile, ICM42686P_REG_INT_SOURCE1, 0x07, val);
}

/** @brief Configure INT2 pin behavior.
 *
 * INT_CONFIG bits [5:3] hold INT2's polarity / drive / mode (mirroring
 * INT1's [2:0]). Caller passes the same flag layout as `int1_config`;
 * we shift up by 3 internally. */
void tile_sense_i_6p6_int2_config(tile_t *tile, uint8_t config)
{
    icm_modify(tile, ICM42686P_REG_INT_CONFIG, 0x38,
               (uint8_t)((config & 0x07) << 3));
}

/** @brief Route data-ready interrupt to INT2.
 *
 * INT_SOURCE3 bit 3 = UI_DRDY_INT2_EN. */
void tile_sense_i_6p6_int2_data_ready(tile_t *tile, uint8_t enabled)
{
    icm_modify(tile, ICM42686P_REG_INT_SOURCE3, 0x08, enabled ? 0x08 : 0x00);
}

/** @brief Route FIFO watermark interrupt to INT2.
 *
 * INT_SOURCE3 bit 2 = FIFO_THS_INT2_EN. */
void tile_sense_i_6p6_int2_fifo_ths(tile_t *tile, uint8_t enabled)
{
    icm_modify(tile, ICM42686P_REG_INT_SOURCE3, 0x04, enabled ? 0x04 : 0x00);
}

/** @brief Route WOM (wake-on-motion) interrupt to INT2.
 *
 * INT_SOURCE4 bits [2:0] = WOM_X/Y/Z_INT2_EN. */
void tile_sense_i_6p6_int2_wom(tile_t *tile, uint8_t enabled)
{
    uint8_t val = enabled ? 0x07 : 0x00;
    icm_modify(tile, ICM42686P_REG_INT_SOURCE4, 0x07, val);
}

/** @brief Set INT pin pulse-width (chip-wide; INT_CONFIG1 bit 6). */
void tile_sense_i_6p6_set_int_pulse_duration(tile_t *tile,
                                             sense_i_6p6_int_pulse_t pulse)
{
    icm_modify(tile, ICM42686P_REG_INT_CONFIG1, 0x40,
               (pulse == SENSE_I_6P6_INT_PULSE_8US) ? 0x40 : 0x00);
}

/** @brief Reset a single chip subsystem (APEX or TEMP). */
void tile_sense_i_6p6_subsystem_reset(tile_t *tile,
                                      sense_i_6p6_subsystem_t which)
{
    /* SIGNAL_PATH_RESET (0x4B) bits, DS-000639 §14.33:
     *   bit 6 DMP_INIT_EN      — (re)start the DMP
     *   bit 5 DMP_MEM_RESET_EN — clear DMP memory
     *   bit 0                  — reserved (there is NO temperature-path
     *                            reset bit on the ICM-42686-P)
     * APEX: follow the §8.3/8.4 sequence — memory reset, wait 1 ms, then
     * DMP init (same as dmp_init_if_needed()).
     * TEMP: no hardware reset exists; deliberately a no-op (the previous
     * implementation wrote a reserved bit). */
    switch (which) {
        case SENSE_I_6P6_RESET_APEX:
            icm_write(tile, ICM42686P_REG_SIGNAL_PATH_RESET, ICM42686P_DMP_MEM_RESET);
            tile->hal->delay_ms(1);
            icm_write(tile, ICM42686P_REG_SIGNAL_PATH_RESET, ICM42686P_DMP_INIT_EN);
            tile->hal->delay_ms(50);  /* DMP boot, as in dmp_init_if_needed() */
            break;
        case SENSE_I_6P6_RESET_TEMP:
        default:
            return;
    }
}

/** @brief Read and clear INT_STATUS register. */
uint8_t tile_sense_i_6p6_get_int_status(tile_t *tile)
{
    return icm_read(tile, ICM42686P_REG_INT_STATUS);
}

/** @brief Read and clear INT_STATUS2 (WOM/SMD flags). */
uint8_t tile_sense_i_6p6_get_int_status2(tile_t *tile)
{
    return icm_read(tile, ICM42686P_REG_INT_STATUS2);
}

/** @brief Read and clear INT_STATUS3 (APEX: step/tilt/tap flags). */
uint8_t tile_sense_i_6p6_get_int_status3(tile_t *tile)
{
    return icm_read(tile, ICM42686P_REG_INT_STATUS3);
}

/* ================================================================
 * Wake on Motion (WOM)
 * ================================================================ */

/** @brief Configure Wake-on-Motion thresholds. */
void tile_sense_i_6p6_wom_config(tile_t *tile,
                                  uint16_t x_mg, uint16_t y_mg, uint16_t z_mg,
                                  sense_i_6p6_wom_mode_t mode)
{
    /* Threshold resolution: 1g/256 ≈ 3.9 mg. Register = mg * 256 / 1000,
     * clamped to the 8-bit register (255 ≈ 996 mg; 1000 mg would
     * otherwise wrap to 0 = "any motion"). */
    uint32_t x_thr = ((uint32_t)x_mg * 256 + 500) / 1000;
    uint32_t y_thr = ((uint32_t)y_mg * 256 + 500) / 1000;
    uint32_t z_thr = ((uint32_t)z_mg * 256 + 500) / 1000;
    if (x_thr > 0xFF) x_thr = 0xFF;
    if (y_thr > 0xFF) y_thr = 0xFF;
    if (z_thr > 0xFF) z_thr = 0xFF;

    /* Thresholds are in Bank 4 */
    icm_set_bank(tile, ICM42686P_BANK_4);
    icm_write(tile, ICM42686P_B4_WOM_X_THR, (uint8_t)x_thr);
    icm_write(tile, ICM42686P_B4_WOM_Y_THR, (uint8_t)y_thr);
    icm_write(tile, ICM42686P_B4_WOM_Z_THR, (uint8_t)z_thr);
    icm_set_bank(tile, ICM42686P_BANK_0);

    /* SMD_CONFIG: set WOM_MODE, keep SMD bits */
    icm_modify(tile, ICM42686P_REG_SMD_CONFIG, 0x04, (uint8_t)mode << 2);
}

/** @brief Enable WOM. */
void tile_sense_i_6p6_wom_enable(tile_t *tile)
{
    /* DS-000639 §8.6: WOM_INT_MODE = 0 (OR of axes), SMD_MODE = 01 turns
     * WOM on. WOM_MODE (bit 2) is left as wom_config() set it, and an SMD
     * mode already chosen (10/11, which also runs WOM) is kept. */
    uint8_t smd = icm_read(tile, ICM42686P_REG_SMD_CONFIG);
    smd &= (uint8_t)~0x08;
    if ((smd & 0x03) == 0) smd |= 0x01;
    icm_write(tile, ICM42686P_REG_SMD_CONFIG, smd);
    tile->hal->delay_ms(50);  /* Datasheet: wait 50ms after enabling */
}

/** @brief Disable WOM. */
void tile_sense_i_6p6_wom_disable(tile_t *tile)
{
    /* WOM is switched on by SMD_CONFIG.SMD_MODE != 00 (wom_enable writes
     * 01, DS-000639 §8.6); SMD_MODE = 00 turns it off (§14.44). This also
     * stops SMD, which is built on WOM. Thresholds are left as configured
     * so wom_enable() can re-arm without another wom_config().
     * (The previous implementation zeroed the thresholds and left
     * SMD_MODE set: a zero threshold makes WOM fire on any motion.) */
    icm_modify(tile, ICM42686P_REG_SMD_CONFIG, 0x03, 0x00);
}

/* ================================================================
 * Significant Motion Detection (SMD)
 * ================================================================ */

/** @brief Configure and enable SMD. */
void tile_sense_i_6p6_smd_config(tile_t *tile, sense_i_6p6_smd_mode_t mode)
{
    icm_modify(tile, ICM42686P_REG_SMD_CONFIG, 0x03, (uint8_t)mode);
}

/* ================================================================
 * APEX: DMP initialization helper
 * ================================================================ */

static void dmp_init_if_needed(tile_t *tile)
{
    /* Check if DMP is already idle (already initialized) */
    uint8_t data3 = icm_read(tile, ICM42686P_REG_APEX_DATA3);
    if (data3 & 0x04) return;  /* DMP_IDLE=1 means already initialized */

    /* DMP memory reset */
    icm_write(tile, ICM42686P_REG_SIGNAL_PATH_RESET, ICM42686P_DMP_MEM_RESET);
    tile->hal->delay_ms(1);

    /* DMP init */
    icm_write(tile, ICM42686P_REG_SIGNAL_PATH_RESET, ICM42686P_DMP_INIT_EN);
    tile->hal->delay_ms(50);  /* DMP boot time */
}

/* ================================================================
 * APEX: Pedometer
 * ================================================================ */

/** @brief Enable the pedometer (step counter + step detector). */
void tile_sense_i_6p6_pedometer_enable(tile_t *tile, sense_i_6p6_dmp_odr_t dmp_odr)
{
    dmp_init_if_needed(tile);

    /* Set DMP ODR and enable pedometer */
    uint8_t cfg0 = icm_read(tile, ICM42686P_REG_APEX_CONFIG0);
    cfg0 &= ~0x23;  /* Clear PED_ENABLE, DMP_ODR */
    cfg0 |= (1 << 5) | ((uint8_t)dmp_odr & 0x03);  /* PED_ENABLE + DMP_ODR */
    icm_write(tile, ICM42686P_REG_APEX_CONFIG0, cfg0);
}

/** @brief Disable the pedometer. */
void tile_sense_i_6p6_pedometer_disable(tile_t *tile)
{
    icm_modify(tile, ICM42686P_REG_APEX_CONFIG0, 0x20, 0x00);  /* Clear PED_ENABLE */
}

/** @brief Read the step count. */
uint16_t tile_sense_i_6p6_get_step_count(tile_t *tile)
{
    /* 42686P byte order: DATA0 = low, DATA1 = high (opposite from 42688P!) */
    uint8_t lo = icm_read(tile, ICM42686P_REG_APEX_DATA0);
    uint8_t hi = icm_read(tile, ICM42686P_REG_APEX_DATA1);
    return ((uint16_t)hi << 8) | lo;
}

/** @brief Read the step cadence. */
uint8_t tile_sense_i_6p6_get_step_cadence(tile_t *tile)
{
    return icm_read(tile, ICM42686P_REG_APEX_DATA2);
}

/** @brief Read the activity classification. */
sense_i_6p6_activity_t tile_sense_i_6p6_get_activity(tile_t *tile)
{
    return (sense_i_6p6_activity_t)(icm_read(tile, ICM42686P_REG_APEX_DATA3) & 0x03);
}

/* ================================================================
 * APEX: Tilt Detection
 * ================================================================ */

/** @brief Enable tilt detection. */
void tile_sense_i_6p6_tilt_enable(tile_t *tile, uint8_t wait_seconds)
{
    dmp_init_if_needed(tile);

    /* TILT_WAIT_TIME_SEL in Bank 4, APEX_CONFIG4 [7:6] */
    uint8_t sel;
    if (wait_seconds <= 0)      sel = 0x00;
    else if (wait_seconds <= 2) sel = 0x01;
    else if (wait_seconds <= 4) sel = 0x02;
    else                        sel = 0x03;

    icm_set_bank(tile, ICM42686P_BANK_4);
    icm_modify(tile, ICM42686P_B4_APEX_CONFIG4, 0xC0, sel << 6);
    icm_set_bank(tile, ICM42686P_BANK_0);

    /* Enable tilt in APEX_CONFIG0 */
    icm_modify(tile, ICM42686P_REG_APEX_CONFIG0, 0x10, 0x10);
}

/** @brief Disable tilt detection. */
void tile_sense_i_6p6_tilt_disable(tile_t *tile)
{
    icm_modify(tile, ICM42686P_REG_APEX_CONFIG0, 0x10, 0x00);
}

/* ================================================================
 * APEX: Tap Detection
 * ================================================================ */

/** @brief Enable tap detection (single + double tap). */
void tile_sense_i_6p6_tap_enable(tile_t *tile)
{
    dmp_init_if_needed(tile);
    icm_modify(tile, ICM42686P_REG_APEX_CONFIG0, 0x40, 0x40);
}

/** @brief Disable tap detection. */
void tile_sense_i_6p6_tap_disable(tile_t *tile)
{
    icm_modify(tile, ICM42686P_REG_APEX_CONFIG0, 0x40, 0x00);
}

/** @brief Read the last tap detection result. */
void tile_sense_i_6p6_get_tap_result(tile_t *tile, sense_i_6p6_tap_result_t *result)
{
    uint8_t d4 = icm_read(tile, ICM42686P_REG_APEX_DATA4);
    uint8_t d5 = icm_read(tile, ICM42686P_REG_APEX_DATA5);

    result->count     = (d4 >> 3) & 0x03;
    result->axis      = (d4 >> 1) & 0x03;
    result->direction = d4 & 0x01;
    result->timing    = d5 & 0x3F;
}

void tile_sense_i_6p6_get_tap_result_flat(tile_t *tile,
                                          int32_t *count,
                                          int32_t *axis,
                                          int32_t *direction,
                                          int32_t *timing)
{
    sense_i_6p6_tap_result_t r = {0};
    tile_sense_i_6p6_get_tap_result(tile, &r);
    if (count)     *count     = (int32_t)r.count;
    if (axis)      *axis      = (int32_t)r.axis;
    if (direction) *direction = (int32_t)r.direction;
    if (timing)    *timing    = (int32_t)r.timing;
}

/* ================================================================
 * Advanced: User offsets
 * ================================================================ */

/** @brief Set user-programmable gyro offset. */
void tile_sense_i_6p6_set_gyro_offset(tile_t *tile,
                                       int16_t x_dps16, int16_t y_dps16, int16_t z_dps16)
{
    int16_t x = x_dps16, y = y_dps16, z = z_dps16;
    /* 12-bit signed values packed across OFFSET_USER0-4 (Bank 4).
     * Layout: U0=[GX 7:0], U1=[GY 11:8 | GX 11:8], U2=[GY 7:0],
     *         U3=[GZ 7:0], U4=[AX 11:8 | GZ 11:8] */
    icm_set_bank(tile, ICM42686P_BANK_4);
    icm_write(tile, ICM42686P_B4_OFFSET_USER0, (uint8_t)(x & 0xFF));
    uint8_t u1 = ((uint8_t)((y >> 8) & 0x0F) << 4) | ((uint8_t)((x >> 8) & 0x0F));
    icm_write(tile, ICM42686P_B4_OFFSET_USER1, u1);
    icm_write(tile, ICM42686P_B4_OFFSET_USER2, (uint8_t)(y & 0xFF));
    icm_write(tile, ICM42686P_B4_OFFSET_USER3, (uint8_t)(z & 0xFF));
    icm_modify(tile, ICM42686P_B4_OFFSET_USER4, 0x0F, (uint8_t)((z >> 8) & 0x0F));
    icm_set_bank(tile, ICM42686P_BANK_0);
}

/** @brief Set user-programmable accel offset. */
void tile_sense_i_6p6_set_accel_offset(tile_t *tile,
                                        int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    int16_t x = x_mg, y = y_mg, z = z_mg;
    /* 12-bit signed values packed across OFFSET_USER4-8 (Bank 4).
     * Layout: U4=[AX 11:8 | GZ 11:8], U5=[AX 7:0], U6=[AY 7:0],
     *         U7=[AZ 11:8 | AY 11:8], U8=[AZ 7:0] */
    icm_set_bank(tile, ICM42686P_BANK_4);
    icm_modify(tile, ICM42686P_B4_OFFSET_USER4, 0xF0, (uint8_t)((x >> 8) & 0x0F) << 4);
    icm_write(tile, ICM42686P_B4_OFFSET_USER5, (uint8_t)(x & 0xFF));
    icm_write(tile, ICM42686P_B4_OFFSET_USER6, (uint8_t)(y & 0xFF));
    uint8_t u7 = ((uint8_t)((z >> 8) & 0x0F) << 4) | ((uint8_t)((y >> 8) & 0x0F));
    icm_write(tile, ICM42686P_B4_OFFSET_USER7, u7);
    icm_write(tile, ICM42686P_B4_OFFSET_USER8, (uint8_t)(z & 0xFF));
    icm_set_bank(tile, ICM42686P_BANK_0);
}

/* ================================================================
 * Advanced: Self-test
 * ================================================================ */

/** @brief Perform self-test for both accel and gyro. */
uint8_t tile_sense_i_6p6_self_test(tile_t *tile, uint8_t *accel_pass, uint8_t *gyro_pass)
{
    /* Save current state */
    uint8_t pwr_save = icm_read(tile, ICM42686P_REG_PWR_MGMT0);

    /* Configure for self-test: ±4g accel, ±250 dps gyro, LN mode */
    icm_write(tile, ICM42686P_REG_ACCEL_CONFIG0,
              (SENSE_I_6P6_ACCEL_4G << 5) | SENSE_I_6P6_ODR_1KHZ);
    icm_write(tile, ICM42686P_REG_GYRO_CONFIG0,
              (SENSE_I_6P6_GYRO_250DPS << 5) | SENSE_I_6P6_ODR_1KHZ);
    icm_write(tile, ICM42686P_REG_PWR_MGMT0,
              ICM42686P_PWR_ACCEL_LN | ICM42686P_PWR_GYRO_LN);
    tile->hal->delay_ms(100);

    /* Read baseline (average of 200 samples at 1 kHz) */
    int32_t ax0 = 0, ay0 = 0, az0 = 0;
    int32_t gx0 = 0, gy0 = 0, gz0 = 0;
    for (int i = 0; i < 200; i++) {
        while (!tile_sense_i_6p6_data_ready(tile)) ;
        int16_t a[3], g[3];
        tile_sense_i_6p6_get_raw_accels(tile, a);
        tile_sense_i_6p6_get_raw_gyros(tile, g);
        ax0 += a[0]; ay0 += a[1]; az0 += a[2];
        gx0 += g[0]; gy0 += g[1]; gz0 += g[2];
    }
    ax0 /= 200; ay0 /= 200; az0 /= 200;
    gx0 /= 200; gy0 /= 200; gz0 /= 200;

    /* Enable self-test */
    icm_write(tile, ICM42686P_REG_SELF_TEST_CONFIG,
              (1 << 6) | 0x3F);  /* ACCEL_ST_POWER + all axes */
    tile->hal->delay_ms(100);

    /* Read self-test values */
    int32_t ax1 = 0, ay1 = 0, az1 = 0;
    int32_t gx1 = 0, gy1 = 0, gz1 = 0;
    for (int i = 0; i < 200; i++) {
        while (!tile_sense_i_6p6_data_ready(tile)) ;
        int16_t a[3], g[3];
        tile_sense_i_6p6_get_raw_accels(tile, a);
        tile_sense_i_6p6_get_raw_gyros(tile, g);
        ax1 += a[0]; ay1 += a[1]; az1 += a[2];
        gx1 += g[0]; gy1 += g[1]; gz1 += g[2];
    }
    ax1 /= 200; ay1 /= 200; az1 /= 200;
    gx1 /= 200; gy1 /= 200; gz1 /= 200;

    /* Disable self-test */
    icm_write(tile, ICM42686P_REG_SELF_TEST_CONFIG, 0x00);

    /* Calculate self-test response */
    int32_t a_resp[3] = { ax1 - ax0, ay1 - ay0, az1 - az0 };
    int32_t g_resp[3] = { gx1 - gx0, gy1 - gy0, gz1 - gz0 };

    /* Check against minimum thresholds (from datasheet section 9.1):
     * Accel: min 225 mg @ ±4g → 225 * 8192 / 1000 ≈ 1843 LSB
     * Gyro:  min 60 dps @ ±250 dps → 60 * 131 = 7860 LSB */
    *accel_pass = 1;
    for (int i = 0; i < 3; i++) {
        if (a_resp[i] < 0) a_resp[i] = -a_resp[i];
        if (a_resp[i] < 1843) *accel_pass = 0;
    }

    *gyro_pass = 1;
    for (int i = 0; i < 3; i++) {
        if (g_resp[i] < 0) g_resp[i] = -g_resp[i];
        if (g_resp[i] < 7860) *gyro_pass = 0;
    }

    /* Restore previous state */
    icm_write(tile, ICM42686P_REG_PWR_MGMT0, pwr_save);

    return (*accel_pass && *gyro_pass) ? 1 : 0;
}

/* ================================================================
 * Advanced: Generic register access
 * ================================================================ */

/** @brief Read a register from any bank. */
uint8_t tile_sense_i_6p6_read_reg(tile_t *tile, uint8_t bank, uint8_t reg)
{
    if (bank != ICM42686P_BANK_0) icm_set_bank(tile, bank);
    uint8_t val = icm_read(tile, reg);
    if (bank != ICM42686P_BANK_0) icm_set_bank(tile, ICM42686P_BANK_0);
    return val;
}

/** @brief Write a register in any bank. */
void tile_sense_i_6p6_write_reg(tile_t *tile, uint8_t bank, uint8_t reg, uint8_t val)
{
    if (bank != ICM42686P_BANK_0) icm_set_bank(tile, bank);
    icm_write(tile, reg, val);
    if (bank != ICM42686P_BANK_0) icm_set_bank(tile, ICM42686P_BANK_0);
}

/* ================================================================
 * Tier-2 idiomatic helpers
 * ================================================================ */

/* Map current ACCEL_CONFIG0[7:5] to LSB-per-g sensitivity. */
static int32_t accel_lsb_per_g(tile_t *tile)
{
    uint8_t cfg = icm_read(tile, ICM42686P_REG_ACCEL_CONFIG0);
    switch ((cfg >> 5) & 0x07) {
        case SENSE_I_6P6_ACCEL_32G: return 1024;
        case SENSE_I_6P6_ACCEL_16G: return 2048;
        case SENSE_I_6P6_ACCEL_8G:  return 4096;
        case SENSE_I_6P6_ACCEL_4G:  return 8192;
        case SENSE_I_6P6_ACCEL_2G:  return 16384;
        default:                    return 4096;  /* fall back to ±8g */
    }
}

/** @brief Detect face-up orientation (Z ≈ +1g). */
uint8_t tile_sense_i_6p6_is_face_up(tile_t *tile)
{
    int16_t a[3];
    tile_sense_i_6p6_get_raw_accels(tile, a);
    int32_t lsb = accel_lsb_per_g(tile);
    int32_t one_g = lsb;
    int32_t tol_z = (lsb * 200) / 1000;   /* ±200 mg */
    int32_t tol_xy = (lsb * 300) / 1000;  /* ±300 mg */
    if (a[2] < (one_g - tol_z) || a[2] > (one_g + tol_z)) return 0;
    if (a[0] < -tol_xy || a[0] > tol_xy) return 0;
    if (a[1] < -tol_xy || a[1] > tol_xy) return 0;
    return 1;
}

/** @brief Detect face-down orientation (Z ≈ −1g). */
uint8_t tile_sense_i_6p6_is_face_down(tile_t *tile)
{
    int16_t a[3];
    tile_sense_i_6p6_get_raw_accels(tile, a);
    int32_t lsb = accel_lsb_per_g(tile);
    int32_t one_g = lsb;
    int32_t tol_z = (lsb * 200) / 1000;
    int32_t tol_xy = (lsb * 300) / 1000;
    if (a[2] < (-one_g - tol_z) || a[2] > (-one_g + tol_z)) return 0;
    if (a[0] < -tol_xy || a[0] > tol_xy) return 0;
    if (a[1] < -tol_xy || a[1] > tol_xy) return 0;
    return 1;
}

/** @brief Detect motion: |‖a‖ − 1g| > threshold (squared comparison). */
uint8_t tile_sense_i_6p6_is_moving(tile_t *tile, uint16_t threshold_mg)
{
    int16_t a[3];
    tile_sense_i_6p6_get_raw_accels(tile, a);
    int32_t lsb = accel_lsb_per_g(tile);
    /* Squared magnitude of measured vector, in LSB^2. */
    int32_t mag2 = (int32_t)a[0]*a[0] + (int32_t)a[1]*a[1] + (int32_t)a[2]*a[2];
    /* Squared 1-g reference, also LSB^2. */
    int32_t one_g_lsb = lsb;
    /* Squared bands: (1g + thr)^2 and (1g - thr)^2 for symmetric trigger.
     * thr_lsb = lsb * threshold_mg / 1000 */
    int32_t thr_lsb = ((int32_t)lsb * (int32_t)threshold_mg) / 1000;
    int32_t hi = one_g_lsb + thr_lsb;
    int32_t lo = one_g_lsb - thr_lsb;
    if (lo < 0) lo = 0;
    int32_t hi_sq = hi * hi;
    int32_t lo_sq = lo * lo;
    return (mag2 > hi_sq || mag2 < lo_sq) ? 1 : 0;
}

/* atan(k/16) in centi-degrees, k=0..16. Used by tilt helper. */
static const uint16_t atan_lut_cdeg[17] = {
    0, 358, 712, 1062, 1404, 1735, 2056, 2363,
    2657, 2936, 3200, 3451, 3687, 3909, 4119, 4315, 4500
};

/* Integer atan2 → centi-degrees, range [-18000, +18000]. */
static int32_t atan2_cdeg(int32_t y, int32_t x)
{
    int32_t ay = y < 0 ? -y : y;
    int32_t ax = x < 0 ? -x : x;
    int32_t result;

    if (ax == 0 && ay == 0) return 0;

    if (ay <= ax) {
        /* atan(|y|/|x|), result in [0,4500] */
        /* idx = |y| * 16 / |x|, fractional remainder for linear interp */
        int32_t scaled = ay * 16;
        int32_t idx = scaled / ax;
        int32_t rem = scaled - idx * ax;
        if (idx >= 16) result = atan_lut_cdeg[16];
        else result = atan_lut_cdeg[idx]
                      + ((atan_lut_cdeg[idx+1] - atan_lut_cdeg[idx]) * rem) / ax;
    } else {
        /* atan(|x|/|y|), then 90 - that */
        int32_t scaled = ax * 16;
        int32_t idx = scaled / ay;
        int32_t rem = scaled - idx * ay;
        int32_t inner;
        if (idx >= 16) inner = atan_lut_cdeg[16];
        else inner = atan_lut_cdeg[idx]
                     + ((atan_lut_cdeg[idx+1] - atan_lut_cdeg[idx]) * rem) / ay;
        result = 9000 - inner;
    }

    /* Quadrant fix-up. */
    if (x < 0) result = 18000 - result;
    if (y < 0) result = -result;
    return result;
}

/** @brief Tilt of one axis vs gravity, in 0.01°. */
uint8_t tile_sense_i_6p6_read_tilt_centi_degrees(tile_t *tile,
                                                  uint8_t axis,
                                                  int16_t *out_centi_deg)
{
    if (!out_centi_deg || axis > 2) return 0;
    int16_t a[3];
    tile_sense_i_6p6_get_raw_accels(tile, a);
    int32_t target = a[axis];
    int32_t o0 = a[(axis + 1) % 3];
    int32_t o1 = a[(axis + 2) % 3];
    /* perpendicular-magnitude squared, then take sqrt via Newton's method. */
    int32_t perp_sq = o0 * o0 + o1 * o1;
    if (perp_sq == 0 && target == 0) {
        *out_centi_deg = 0;
        return 0;
    }
    /* Integer sqrt (Newton). perp_sq fits in int32 for int16 inputs. */
    int32_t r = perp_sq;
    if (r > 0) {
        int32_t guess = r >> 8;
        if (guess == 0) guess = 1;
        for (int i = 0; i < 16; i++) {
            int32_t next = (guess + r / guess) >> 1;
            if (next == guess) break;
            guess = next;
        }
        r = guess;
    }
    int32_t cdeg = atan2_cdeg(target, r);
    if (cdeg > 18000) cdeg = 18000;
    if (cdeg < -18000) cdeg = -18000;
    *out_centi_deg = (int16_t)cdeg;
    return 1;
}

/** @brief Flat-output variant of read_tilt_centi_degrees (int32 out). */
uint8_t tile_sense_i_6p6_read_tilt_centi_degrees_flat(tile_t *tile,
                                                       uint8_t axis,
                                                       int32_t *out_centi_deg)
{
    int16_t v = 0;
    uint8_t ok = tile_sense_i_6p6_read_tilt_centi_degrees(tile, axis, &v);
    if (out_centi_deg) *out_centi_deg = (int32_t)v;
    return ok;
}

/** @brief Block until a tap interrupt fires or timeout. Polls at ~1 ms. */
uint8_t tile_sense_i_6p6_wait_for_tap(tile_t *tile, uint32_t timeout_ms)
{
    /* Single-shot when timeout_ms == 0. */
    if (icm_read(tile, ICM42686P_REG_INT_STATUS3) & ICM42686P_INT_STATUS3_TAP_DET)
        return 1;
    while (timeout_ms > 0) {
        tile->hal->delay_ms(1);
        if (icm_read(tile, ICM42686P_REG_INT_STATUS3) & ICM42686P_INT_STATUS3_TAP_DET)
            return 1;
        timeout_ms--;
    }
    return 0;
}

/** @brief Block until a wake-on-motion interrupt fires or timeout. Polls at ~1 ms. */
uint8_t tile_sense_i_6p6_wait_for_motion(tile_t *tile, uint32_t timeout_ms)
{
    if (icm_read(tile, ICM42686P_REG_INT_STATUS2) & ICM42686P_INT_STATUS2_WOM_ANY)
        return 1;
    while (timeout_ms > 0) {
        tile->hal->delay_ms(1);
        if (icm_read(tile, ICM42686P_REG_INT_STATUS2) & ICM42686P_INT_STATUS2_WOM_ANY)
            return 1;
        timeout_ms--;
    }
    return 0;
}
