/**
 * @file   tile_sense_i_6p6.h
 * @brief  Complete driver for the Sense.I.6P6 tile (ICM-42686-P).
 *         Supports both I2C and SPI bus access via tiles_pal_t.
 * @version 1.2.0
 *
 * 6-axis IMU with extended measurement range:
 *   - Accelerometer:  16-bit, ±2/4/8/16/32 G, up to 32 kHz ODR
 *   - Gyroscope:      16-bit, ±31.25 to ±4000 DPS, up to 32 kHz ODR
 *   - Temperature:    on-chip sensor
 *   - FIFO:           2 KB with multiple packet formats
 *   - APEX:           Pedometer, tilt, wake-on-motion, tap detection
 *
 * Platform-agnostic: uses tiles_pal_t for all bus access.
 *
 * Quick start — I2C (polling):
 * @code
 *   tile_t imu;
 *   tile_sense_i_6p6_init(core_tiles_pal(&core_i2c3), 0, &imu, NULL);
 *   int16_t accel[3], gyro[3];
 *   tile_sense_i_6p6_get_raw_accels(&imu, accel);
 *   tile_sense_i_6p6_get_raw_gyros(&imu, gyro);
 * @endcode
 *
 * Quick start — SPI:
 * @code
 *   tile_t imu;
 *   tile_sense_i_6p6_init(core_tiles_pal(&core_spi1), 0, &imu, NULL);
 *   int16_t accel[3];
 *   tile_sense_i_6p6_get_raw_accels(&imu, accel);
 * @endcode
 *
 * Quick start — interrupt-driven with callback:
 * @code
 *   void on_data(tile_t *t, uint8_t events, void *ctx) {
 *       int16_t buf[7];
 *       tile_sense_i_6p6_get_raw_all(t, buf);
 *   }
 *   sense_i_6p6_cfg_t cfg = {
 *       .on_event = on_data,
 *       .int1_pin = 9,  // Core pad connected to INT1
 *   };
 *   tile_sense_i_6p6_init(core_tiles_pal(&core_i2c3), 0, &imu, &cfg);
 *   tile_sense_i_6p6_int1_data_ready(&imu, 1);
 *   while (1) { tile_sense_i_6p6_process(&imu); }
 * @endcode
 *
 * Datasheet: TDK InvenSense DS-000639, Rev 1.0
 *
 * @studio tile label=Sense.I.6P6 icon=triad
 * @studio event name=data_ready mask=ICM42686P_INT_DATA_RDY
 * @studio event name=fifo_watermark mask=ICM42686P_INT_FIFO_THS
 * @studio event name=tilt mask=ICM42686P_INT_STATUS3_TILT_DET
 * @studio event name=tap mask=ICM42686P_INT_STATUS3_TAP_DET payload=count:int,axis:int,direction:int read=tile_sense_i_6p6_get_tap_result read_type=sense_i_6p6_tap_result_t
 * @studio event name=wake_on_motion mask=ICM42686P_INT_STATUS2_WOM_ANY
 *
 * Unsupported capabilities (gating noted per item — a HARDWARE gap means the
 * chip can do it but the tile doesn't wire the pin out, NOT a driver omission):
 *
 * @studio unsupported severity=advanced category="External timing pins (FSYNC / CLKIN)"
 *   Hardware-gated (not a driver gap). The ICM-42686-P can take a 31–50 kHz
 *   FSYNC input (TIMESTAMP_FSYNC_EN) for sample time-stamping and a ~32 kHz
 *   external CLKIN (RTC) for precise ODR. Neither pin is routed to a tile pad
 *   on Sense-I-6P6-a, so the capability simply isn't wired out — closing
 *   requires a tile hardware revision, not driver work.
 *
 * @studio unsupported severity=niche category="FIFO 20-bit hi-res mode"
 *   Design-gated. FIFO_HIRES_EN switches FIFO packets to 20-bit accel
 *   + 20-bit gyro (Packet 4 with 3-byte extensions) at higher
 *   sensitivity scale factors. Closing properly requires extending
 *   sense_i_6p6_fifo_packet_t to represent 20-bit fields and reworking
 *   the read_packet parser; toggling the bit alone breaks the existing
 *   16-bit parsing path. Deferred to a future driver pass.
 *
 * @studio unsupported severity=advanced category="I3C support"
 *   Ecosystem-gated. ICM-42686-P supports I3C SDR (up to 12.5 MHz
 *   with in-band interrupts and dynamic addressing) and the tile
 *   straps I3C on pads 3/4/5. The driver framework currently uses
 *   tiles_pal I²C calls only; closing requires a new bus abstraction
 *   in Studio. Defer to a multi-bus driver framework pass.
 *
 * @studio unsupported severity=advanced category="APEX raise-to-wake / raise-to-sleep"
 *   Driver-deferred (true driver gap — hardware supports it). The APEX engine
 *   detects raise-to-wake / raise-to-sleep gestures (RWR), routable to INT via
 *   INT_SOURCE6/7. The driver wraps pedometer / tilt / tap / WOM / SMD but not
 *   RWR; no hardware or framework blocker — closing is a driver pass.
 *
 * @studio unsupported severity=advanced category="UI notch / anti-alias filter tuning"
 *   Driver-deferred (true driver gap). set_filter_bw / set_filter_order expose
 *   the UI filter bandwidth and order, but not the notch-filter centre
 *   frequency / Q (Bank-1 GYRO_CFG_STATIC registers) for rejecting a specific
 *   vibration band.
 *
 * @studio unsupported severity=niche category="Standalone timestamp (TMSTVAL) read"
 *   Driver-deferred (true driver gap). FIFO packets carry a 16-bit timestamp,
 *   but the chip's free-running timestamp counter (Bank-1 TMSTVAL0-2) isn't
 *   exposed as a direct read.
 */

#ifndef INC_TILE_SENSE_I_6P6_H_
#define INC_TILE_SENSE_I_6P6_H_

#include "tiles.h"
#include <stdint.h>

/* ================================================================
 * Driver version
 * ================================================================ */

#define TILE_SENSE_I_6P6_VERSION_MAJOR  1
#define TILE_SENSE_I_6P6_VERSION_MINOR  2
#define TILE_SENSE_I_6P6_VERSION_PATCH  0

TILES_CHECK_VERSION(1, 0);

/* ================================================================
 * Instance mapping
 * ================================================================ */

/**
 * | Instance | ID   | Hardware config                    |
 * |----------|------|------------------------------------|
 * | 0        | 0x69 | Pad 2 (AD0) pulled high — default  |
 * | 1        | 0x68 | Pad 2 (AD0) tied to GND            |
 */
#define ICM42686P_I2C_ADDR_DEFAULT  0x69
#define ICM42686P_I2C_ADDR_ALT      0x68

/* ================================================================
 * Register map — Bank 0 (default)
 * ================================================================ */

/* Device management */
#define ICM42686P_REG_DEVICE_CONFIG     0x11
#define ICM42686P_REG_DRIVE_CONFIG      0x13
#define ICM42686P_REG_SIGNAL_PATH_RESET 0x4B
#define ICM42686P_REG_INTF_CONFIG0      0x4C
#define ICM42686P_REG_INTF_CONFIG1      0x4D
#define ICM42686P_REG_PWR_MGMT0        0x4E
#define ICM42686P_REG_WHO_AM_I         0x75
#define ICM42686P_REG_BANK_SEL         0x76

/* Sensor configuration */
#define ICM42686P_REG_GYRO_CONFIG0      0x4F
#define ICM42686P_REG_ACCEL_CONFIG0     0x50
#define ICM42686P_REG_GYRO_CONFIG1      0x51
#define ICM42686P_REG_GYRO_ACCEL_CFG0   0x52
#define ICM42686P_REG_ACCEL_CONFIG1     0x53
#define ICM42686P_REG_TMST_CONFIG       0x54

/* Data registers (big-endian, burst-readable from TEMP_H) */
#define ICM42686P_REG_TEMP_H            0x1D
#define ICM42686P_REG_TEMP_L            0x1E
#define ICM42686P_REG_ACCEL_X_H         0x1F
#define ICM42686P_REG_GYRO_X_H          0x25

/* Interrupt */
#define ICM42686P_REG_INT_CONFIG        0x14
#define ICM42686P_REG_INT_CONFIG0       0x63
#define ICM42686P_REG_INT_CONFIG1       0x64
#define ICM42686P_REG_INT_SOURCE0       0x65
#define ICM42686P_REG_INT_SOURCE1       0x66
#define ICM42686P_REG_INT_SOURCE3       0x68
#define ICM42686P_REG_INT_SOURCE4       0x69
#define ICM42686P_REG_INT_STATUS        0x2D
#define ICM42686P_REG_INT_STATUS2       0x37
#define ICM42686P_REG_INT_STATUS3       0x38

/* FIFO */
#define ICM42686P_REG_FIFO_CONFIG       0x16
#define ICM42686P_REG_FIFO_CONFIG1      0x5F
#define ICM42686P_REG_FIFO_CONFIG2      0x60
#define ICM42686P_REG_FIFO_CONFIG3      0x61
#define ICM42686P_REG_FIFO_COUNTH       0x2E
#define ICM42686P_REG_FIFO_COUNTL       0x2F
#define ICM42686P_REG_FIFO_DATA         0x30
#define ICM42686P_REG_FIFO_LOST_PKT0   0x6C
#define ICM42686P_REG_FIFO_LOST_PKT1   0x6D

/* APEX (DMP) */
#define ICM42686P_REG_APEX_CONFIG0      0x56
#define ICM42686P_REG_SMD_CONFIG        0x57
#define ICM42686P_REG_APEX_DATA0        0x31  /* STEP_CNT low byte (NOTE: reversed from 42688P) */
#define ICM42686P_REG_APEX_DATA1        0x32  /* STEP_CNT high byte */
#define ICM42686P_REG_APEX_DATA2        0x33  /* STEP_CADENCE (u6.2) */
#define ICM42686P_REG_APEX_DATA3        0x34  /* ACTIVITY_CLASS, DMP_IDLE */
#define ICM42686P_REG_APEX_DATA4        0x35  /* TAP_NUM, TAP_AXIS, TAP_DIR */
#define ICM42686P_REG_APEX_DATA5        0x36  /* DOUBLE_TAP_TIMING */

/* Self-test */
#define ICM42686P_REG_SELF_TEST_CONFIG  0x70

/* FSYNC */
#define ICM42686P_REG_FSYNC_CONFIG      0x62

/* ================================================================
 * Register map — Bank 1
 * ================================================================ */

#define ICM42686P_B1_SENSOR_CONFIG0     0x03
#define ICM42686P_B1_GYRO_CFG_STATIC2  0x0B
#define ICM42686P_B1_GYRO_CFG_STATIC3  0x0C
#define ICM42686P_B1_GYRO_CFG_STATIC4  0x0D
#define ICM42686P_B1_GYRO_CFG_STATIC5  0x0E
#define ICM42686P_B1_GYRO_NF_COSWZ_X   0x0F
#define ICM42686P_B1_GYRO_NF_COSWZ_Y   0x10
#define ICM42686P_B1_GYRO_NF_COSWZ_Z   0x11
#define ICM42686P_B1_GYRO_CFG_STATIC9  0x12
#define ICM42686P_B1_GYRO_CFG_STATIC10 0x13
#define ICM42686P_B1_XG_ST_DATA         0x5F
#define ICM42686P_B1_YG_ST_DATA         0x60
#define ICM42686P_B1_ZG_ST_DATA         0x61
#define ICM42686P_B1_TMSTVAL0           0x62
#define ICM42686P_B1_TMSTVAL1           0x63
#define ICM42686P_B1_TMSTVAL2           0x64
#define ICM42686P_B1_INTF_CONFIG4       0x7A
#define ICM42686P_B1_INTF_CONFIG5       0x7B

/* ================================================================
 * Register map — Bank 2
 * ================================================================ */

#define ICM42686P_B2_ACCEL_CFG_STATIC2 0x03
#define ICM42686P_B2_ACCEL_CFG_STATIC3 0x04
#define ICM42686P_B2_ACCEL_CFG_STATIC4 0x05
#define ICM42686P_B2_XA_ST_DATA         0x3B
#define ICM42686P_B2_YA_ST_DATA         0x3C
#define ICM42686P_B2_ZA_ST_DATA         0x3D

/* ================================================================
 * Register map — Bank 4
 * ================================================================ */

#define ICM42686P_B4_APEX_CONFIG1       0x40
#define ICM42686P_B4_APEX_CONFIG2       0x41
#define ICM42686P_B4_APEX_CONFIG3       0x42
#define ICM42686P_B4_APEX_CONFIG4       0x43
#define ICM42686P_B4_APEX_CONFIG7       0x46
#define ICM42686P_B4_APEX_CONFIG8       0x47
#define ICM42686P_B4_APEX_CONFIG9       0x48
#define ICM42686P_B4_WOM_X_THR          0x4A
#define ICM42686P_B4_WOM_Y_THR          0x4B
#define ICM42686P_B4_WOM_Z_THR          0x4C
#define ICM42686P_B4_INT_SOURCE6        0x4D
#define ICM42686P_B4_INT_SOURCE7        0x4E
#define ICM42686P_B4_INT_SOURCE8        0x4F
#define ICM42686P_B4_INT_SOURCE9        0x50
#define ICM42686P_B4_INT_SOURCE10       0x51
#define ICM42686P_B4_OFFSET_USER0       0x77
#define ICM42686P_B4_OFFSET_USER1       0x78
#define ICM42686P_B4_OFFSET_USER2       0x79
#define ICM42686P_B4_OFFSET_USER3       0x7A
#define ICM42686P_B4_OFFSET_USER4       0x7B
#define ICM42686P_B4_OFFSET_USER5       0x7C
#define ICM42686P_B4_OFFSET_USER6       0x7D
#define ICM42686P_B4_OFFSET_USER7       0x7E
#define ICM42686P_B4_OFFSET_USER8       0x7F

/* ================================================================
 * Bank selection
 * ================================================================ */

#define ICM42686P_BANK_0    0x00
#define ICM42686P_BANK_1    0x01
#define ICM42686P_BANK_2    0x02
#define ICM42686P_BANK_4    0x04

/* ================================================================
 * Chip identity
 * ================================================================ */

#define ICM42686P_WHOAMI_DEFAULT    0x44

/* ================================================================
 * PWR_MGMT0 bit definitions
 * ================================================================ */

#define ICM42686P_PWR_TEMP_DIS      (1 << 5)
#define ICM42686P_PWR_IDLE          (1 << 4)
#define ICM42686P_PWR_GYRO_OFF      (0 << 2)
#define ICM42686P_PWR_GYRO_STANDBY  (1 << 2)
#define ICM42686P_PWR_GYRO_LN       (3 << 2)
#define ICM42686P_PWR_ACCEL_OFF     (0 << 0)
#define ICM42686P_PWR_ACCEL_LP      (2 << 0)
#define ICM42686P_PWR_ACCEL_LN      (3 << 0)

/* ================================================================
 * INT_STATUS bit definitions
 * ================================================================ */

#define ICM42686P_INT_DATA_RDY      (1 << 3)
#define ICM42686P_INT_FIFO_THS      (1 << 2)
#define ICM42686P_INT_FIFO_FULL     (1 << 1)
#define ICM42686P_INT_RESET_DONE    (1 << 4)

/* INT_STATUS2 (0x37) — WOM / SMD flags */
#define ICM42686P_INT_STATUS2_WOM_X     (1 << 0)
#define ICM42686P_INT_STATUS2_WOM_Y     (1 << 1)
#define ICM42686P_INT_STATUS2_WOM_Z     (1 << 2)
#define ICM42686P_INT_STATUS2_SMD       (1 << 3)
/** Any-axis WOM: OR of the three per-axis bits. */
#define ICM42686P_INT_STATUS2_WOM_ANY \
    (ICM42686P_INT_STATUS2_WOM_X | ICM42686P_INT_STATUS2_WOM_Y | ICM42686P_INT_STATUS2_WOM_Z)

/* INT_STATUS3 (0x38) — APEX (DMP) flags */
#define ICM42686P_INT_STATUS3_TAP_DET     (1 << 5)
#define ICM42686P_INT_STATUS3_WAKE        (1 << 4)
#define ICM42686P_INT_STATUS3_TILT_DET    (1 << 3)
#define ICM42686P_INT_STATUS3_STEP_CNT_OVF (1 << 2)
#define ICM42686P_INT_STATUS3_STEP_DET    (1 << 1)

/* ================================================================
 * SIGNAL_PATH_RESET bits
 * ================================================================ */

#define ICM42686P_DMP_INIT_EN       (1 << 6)
#define ICM42686P_DMP_MEM_RESET     (1 << 5)
#define ICM42686P_FIFO_FLUSH        (1 << 1)

/* ================================================================
 * Configuration enums
 * ================================================================ */

/**
 * @brief  Accelerometer full-scale range.
 *
 * | Enum value   | FSR    | Sensitivity (LSB/g) |
 * |--------------|--------|---------------------|
 * | ACCEL_32G    | ±32 g  |  1,024              |
 * | ACCEL_16G    | ±16 g  |  2,048              |
 * | ACCEL_8G     | ±8 g   |  4,096              |
 * | ACCEL_4G     | ±4 g   |  8,192              |
 * | ACCEL_2G     | ±2 g   | 16,384              |
 */
typedef enum {
    SENSE_I_6P6_ACCEL_32G  = 0x00,  /**< +/- 32g */
    SENSE_I_6P6_ACCEL_16G  = 0x01,  /**< +/- 16g */
    SENSE_I_6P6_ACCEL_8G   = 0x02,  /**< +/- 8g */
    SENSE_I_6P6_ACCEL_4G   = 0x03,  /**< +/- 4g */
    SENSE_I_6P6_ACCEL_2G   = 0x04,  /**< +/- 2g */
} sense_i_6p6_accel_range_t;

/**
 * @brief  Gyroscope full-scale range.
 *
 * | Enum value    | FSR          | Sensitivity (LSB/dps) |
 * |---------------|--------------|----------------------|
 * | GYRO_4000DPS  | ±4000 °/s    |     8.2              |
 * | GYRO_2000DPS  | ±2000 °/s    |    16.4              |
 * | GYRO_1000DPS  | ±1000 °/s    |    32.8              |
 * | GYRO_500DPS   | ±500 °/s     |    65.5              |
 * | GYRO_250DPS   | ±250 °/s     |   131.0              |
 * | GYRO_125DPS   | ±125 °/s     |   262.0              |
 * | GYRO_62_5DPS  | ±62.5 °/s    |   524.3              |
 * | GYRO_31_25DPS | ±31.25 °/s   |  1048.6              |
 */
typedef enum {
    SENSE_I_6P6_GYRO_4000DPS   = 0x00,  /**< +/- 4000 deg/s */
    SENSE_I_6P6_GYRO_2000DPS   = 0x01,  /**< +/- 2000 deg/s */
    SENSE_I_6P6_GYRO_1000DPS   = 0x02,  /**< +/- 1000 deg/s */
    SENSE_I_6P6_GYRO_500DPS    = 0x03,  /**< +/- 500 deg/s */
    SENSE_I_6P6_GYRO_250DPS    = 0x04,  /**< +/- 250 deg/s */
    SENSE_I_6P6_GYRO_125DPS    = 0x05,  /**< +/- 125 deg/s */
    SENSE_I_6P6_GYRO_62_5DPS   = 0x06,  /**< +/- 62.5 deg/s */
    SENSE_I_6P6_GYRO_31_25DPS  = 0x07,  /**< +/- 31.25 deg/s */
} sense_i_6p6_gyro_range_t;

/**
 * @brief  Output data rate selection.
 *
 * Same encoding for both accel and gyro (GYRO_CONFIG0[3:0], ACCEL_CONFIG0[3:0]).
 * Rates above 200 Hz require Low-Noise mode. Rates below 12.5 Hz are accel LP only.
 */
typedef enum {
    SENSE_I_6P6_ODR_32KHZ    = 0x01,    /**< 32 kHz @studio value=32000 */
    SENSE_I_6P6_ODR_16KHZ    = 0x02,    /**< 16 kHz @studio value=16000 */
    SENSE_I_6P6_ODR_8KHZ     = 0x03,    /**< 8 kHz @studio value=8000 */
    SENSE_I_6P6_ODR_4KHZ     = 0x04,    /**< 4 kHz @studio value=4000 */
    SENSE_I_6P6_ODR_2KHZ     = 0x05,    /**< 2 kHz @studio value=2000 */
    SENSE_I_6P6_ODR_1KHZ     = 0x06,    /**< 1 kHz @studio value=1000 */
    SENSE_I_6P6_ODR_200HZ    = 0x07,    /**< 200 Hz @studio value=200 */
    SENSE_I_6P6_ODR_100HZ    = 0x08,    /**< 100 Hz @studio value=100 */
    SENSE_I_6P6_ODR_50HZ     = 0x09,    /**< 50 Hz @studio value=50 */
    SENSE_I_6P6_ODR_25HZ     = 0x0A,    /**< 25 Hz @studio value=25 */
    SENSE_I_6P6_ODR_12_5HZ   = 0x0B,    /**< 12.5 Hz @studio value=12.5 */
    SENSE_I_6P6_ODR_6_25HZ   = 0x0C,    /**< 6.25 Hz (accel LP only) @studio value=6.25 */
    SENSE_I_6P6_ODR_3_125HZ  = 0x0D,    /**< 3.125 Hz (accel LP only) @studio value=3.125 */
    SENSE_I_6P6_ODR_1_5625HZ = 0x0E,    /**< 1.5625 Hz (accel LP only) @studio value=1.5625 */
    SENSE_I_6P6_ODR_500HZ    = 0x0F,    /**< 500 Hz @studio value=500 */
} sense_i_6p6_odr_t;

/**
 * @brief  Power mode for accel/gyro independently.
 */
typedef enum {
    SENSE_I_6P6_MODE_OFF     = 0x00,   /**< Powered off */
    SENSE_I_6P6_MODE_STANDBY = 0x01,   /**< Gyro only: drive on, no output */
    SENSE_I_6P6_MODE_LP      = 0x02,   /**< Accel only: duty-cycled */
    SENSE_I_6P6_MODE_LN      = 0x03,   /**< Low-noise (full performance) */
} sense_i_6p6_power_mode_t;

/**
 * @brief  UI filter bandwidth selection.
 *
 * Used for both accel and gyro in GYRO_ACCEL_CONFIG0 (0x52).
 * In Low-Noise mode: sets the digital low-pass filter cutoff. Per the datasheet
 * (GYRO_ACCEL_CONFIG0, 0x52) the cutoff is ODR/2 for setting 0 and
 * max(400 Hz, ODR)/N for settings 1–7 — NOT ODR/N: below 400 Hz the filter stops
 * tracking the data rate. `@studio value=` is the divisor N.
 * In Low-Power mode: sets averaging factor.
 */
typedef enum {
    SENSE_I_6P6_FILT_BW_ODR_2     = 0x00,  /**< ODR/2 (Nyquist) @studio value=2 */
    SENSE_I_6P6_FILT_BW_ODR_4     = 0x01,  /**< max(400 Hz, ODR)/4 (default) @studio value=4 */
    SENSE_I_6P6_FILT_BW_ODR_5     = 0x02,  /**< max(400 Hz, ODR)/5 @studio value=5 */
    SENSE_I_6P6_FILT_BW_ODR_8     = 0x03,  /**< max(400 Hz, ODR)/8 @studio value=8 */
    SENSE_I_6P6_FILT_BW_ODR_10    = 0x04,  /**< max(400 Hz, ODR)/10 @studio value=10 */
    SENSE_I_6P6_FILT_BW_ODR_16    = 0x05,  /**< max(400 Hz, ODR)/16 @studio value=16 */
    SENSE_I_6P6_FILT_BW_ODR_20    = 0x06,  /**< max(400 Hz, ODR)/20 @studio value=20 */
    SENSE_I_6P6_FILT_BW_ODR_40    = 0x07,  /**< max(400 Hz, ODR)/40 @studio value=40 */
    SENSE_I_6P6_FILT_LP_1X_AVG    = 0x01,  /**< LP mode: 1× averaging (default) */
    SENSE_I_6P6_FILT_LP_16X_AVG   = 0x06,  /**< LP mode: 16× averaging */
} sense_i_6p6_filter_bw_t;

/**
 * @brief  UI filter order.
 */
typedef enum {
    SENSE_I_6P6_FILT_ORDER_1ST = 0x00,  /**< 1st order */
    SENSE_I_6P6_FILT_ORDER_2ND = 0x01,  /**< 2nd order */
    SENSE_I_6P6_FILT_ORDER_3RD = 0x02,  /**< 3rd order */
} sense_i_6p6_filter_order_t;

/**
 * @brief  Temperature filter bandwidth.
 */
typedef enum {
    SENSE_I_6P6_TEMP_FILT_4000HZ = 0x00,  /**< 4000 Hz cutoff @studio value=4000 */
    SENSE_I_6P6_TEMP_FILT_170HZ  = 0x01,  /**< 170 Hz cutoff @studio value=170 */
    SENSE_I_6P6_TEMP_FILT_82HZ   = 0x02,  /**< 82 Hz cutoff @studio value=82 */
    SENSE_I_6P6_TEMP_FILT_40HZ   = 0x03,  /**< 40 Hz cutoff @studio value=40 */
    SENSE_I_6P6_TEMP_FILT_20HZ   = 0x04,  /**< 20 Hz cutoff @studio value=20 */
    SENSE_I_6P6_TEMP_FILT_10HZ   = 0x05,  /**< 10 Hz cutoff @studio value=10 */
    SENSE_I_6P6_TEMP_FILT_5HZ    = 0x06,  /**< 5 Hz cutoff @studio value=5 */
} sense_i_6p6_temp_filter_t;

/**
 * @brief  FIFO mode.
 */
typedef enum {
    SENSE_I_6P6_FIFO_BYPASS      = 0x00,  /**< FIFO disabled */
    SENSE_I_6P6_FIFO_STREAM      = 0x01,  /**< Stream mode (oldest data replaced) */
    SENSE_I_6P6_FIFO_STOP_ON_FULL = 0x02,  /**< Stop on full */
} sense_i_6p6_fifo_mode_t;

/**
 * @brief  APEX DMP output data rate.
 */
typedef enum {
    SENSE_I_6P6_DMP_ODR_25HZ  = 0x00,  /**< 25 Hz */
    SENSE_I_6P6_DMP_ODR_50HZ  = 0x02,  /**< 50 Hz */
} sense_i_6p6_dmp_odr_t;

/**
 * @brief  WOM (Wake-on-Motion) compare mode.
 */
typedef enum {
    SENSE_I_6P6_WOM_INITIAL  = 0x00,   /**< Compare against first sample */
    SENSE_I_6P6_WOM_PREVIOUS = 0x01,   /**< Compare against previous sample */
} sense_i_6p6_wom_mode_t;

/**
 * @brief  SMD (Significant Motion Detection) mode.
 */
typedef enum {
    SENSE_I_6P6_SMD_DISABLED = 0x00,   /**< Disabled */
    SENSE_I_6P6_SMD_SHORT    = 0x02,   /**< 1 second WOM window */
    SENSE_I_6P6_SMD_LONG     = 0x03,   /**< 3 second WOM window */
} sense_i_6p6_smd_mode_t;

/**
 * @brief  Interrupt pin configuration.
 */
typedef enum {
    SENSE_I_6P6_INT_ACTIVE_LOW   = 0x00,  /**< Active low */
    SENSE_I_6P6_INT_ACTIVE_HIGH  = 0x01,  /**< Active high */
    SENSE_I_6P6_INT_OPEN_DRAIN   = 0x00,  /**< Open drain */
    SENSE_I_6P6_INT_PUSH_PULL    = 0x02,  /**< Push-pull */
    SENSE_I_6P6_INT_PULSED       = 0x00,  /**< Pulsed */
    SENSE_I_6P6_INT_LATCHED      = 0x04,  /**< Latched */
} sense_i_6p6_int_config_t;

/**
 * @brief  FIFO packet header bits.
 */
#define SENSE_I_6P6_FIFO_HEADER_EMPTY     (1 << 7)
#define SENSE_I_6P6_FIFO_HEADER_ACCEL     (1 << 6)
#define SENSE_I_6P6_FIFO_HEADER_GYRO      (1 << 5)
#define SENSE_I_6P6_FIFO_HEADER_20BIT     (1 << 4)
#define SENSE_I_6P6_FIFO_HEADER_TMST_ODR  (2 << 2)
#define SENSE_I_6P6_FIFO_HEADER_TMST_FSYNC (3 << 2)

/**
 * @brief  FIFO packet structure (16 bytes, accel + gyro + temp + timestamp).
 */
typedef struct {
    uint8_t  header;     /**< Packet header byte (use SENSE_I_6P6_FIFO_HEADER_* masks). */
    int16_t  accel[3];   /**< X, Y, Z in raw ADC counts */
    int16_t  gyro[3];    /**< X, Y, Z in raw ADC counts */
    int8_t   temp;       /**< 8-bit temperature: degC = temp/2.07 + 25 */
    uint16_t timestamp;  /**< 16-bit timestamp */
} sense_i_6p6_fifo_packet_t;

/**
 * @brief  APEX activity classification.
 */
typedef enum {
    SENSE_I_6P6_ACTIVITY_UNKNOWN = 0x00,  /**< Unknown */
    SENSE_I_6P6_ACTIVITY_WALK    = 0x01,  /**< Walking */
    SENSE_I_6P6_ACTIVITY_RUN     = 0x02,  /**< Running */
} sense_i_6p6_activity_t;

/**
 * @brief  Tap detection result.
 */
typedef struct {
    uint8_t count;       /**< 0=none, 1=single, 2=double */
    uint8_t axis;        /**< 0=X, 1=Y, 2=Z */
    uint8_t direction;   /**< 0=positive, 1=negative */
    uint8_t timing;      /**< Double-tap timing: seconds = timing * 16 / ODR */
} sense_i_6p6_tap_result_t;

/* ================================================================
 * Event callback
 * ================================================================ */

/**
 * Event callback fired by tile_sense_i_6p6_process() when an interrupt
 * event is detected. The events parameter is a bitmask of INT_STATUS
 * register bits (DATA_RDY, FIFO_THS, FIFO_FULL, etc.).
 *
 * Always runs in main-loop context (never ISR). I2C is safe to call.
 */
typedef void (*sense_i_6p6_event_cb_t)(tile_t *tile, uint8_t events, void *ctx);

/* ================================================================
 * Configuration
 * ================================================================ */

/**
 * Optional init config. Pass NULL for defaults (polling, no interrupts).
 */
typedef struct {
    /* Interrupt pins — set to enable interrupt-driven mode via gpio_irq.
     * INT1 (tile pad 9): primary interrupt output.
     * INT2 (tile pad 8): secondary interrupt output.
     * Set to 0 for polled mode (no interrupt wiring needed). */
    uint8_t int1_pin;             /**< Core pad number for INT1. 0 = polled. */
    uint8_t int2_pin;             /**< Core pad number for INT2. 0 = unused. */

    /* Event callback — fired by process() when interrupt events are detected */
    sense_i_6p6_event_cb_t on_event;  /**< Callback. NULL = no callback. */
    void *event_ctx;              /**< User context passed to callback. */

    /* Initial sensor config (0 = use defaults: ±8g, ±1000dps, 100Hz) */
    uint8_t accel_range;          /**< sense_i_6p6_accel_range_t. 0 = default. */
    uint8_t gyro_range;           /**< sense_i_6p6_gyro_range_t. 0 = default. */
    uint8_t odr;                  /**< sense_i_6p6_odr_t. 0 = default (100Hz). */
} sense_i_6p6_cfg_t;

/* ================================================================
 * Public API — Lifecycle
 * ================================================================ */

/** @brief  Check if a Sense.I.6P6 is present on the bus (address probe only). */
uint8_t tile_sense_i_6p6_find(tiles_pal_t *hal, uint8_t instance);

/**
 * @brief  Initialize the ICM-42686P.
 *
 * Performs soft reset, verifies WHO_AM_I, configures sensor ranges
 * and ODR. Pass cfg=NULL for defaults (±8g, ±1000dps, 100Hz, polled).
 *
 * @note   Blocks for ~2 ms (reset). Call once at startup.
 */
void tile_sense_i_6p6_init(tiles_pal_t *hal, uint8_t instance,
                           tile_t *tile, const sense_i_6p6_cfg_t *cfg);

/* ---- Event processing ---- */

/**
 * @brief  Process pending interrupt events. Call from your main loop.
 *
 * In interrupt mode: returns immediately if no INT1 interrupt fired;
 * reads INT_STATUS and fires callback only when events are pending.
 * In polled mode: reads INT_STATUS every call, fires callback if set.
 *
 * @studio expose category=tile name=process section=events
 */
void tile_sense_i_6p6_process(tile_t *tile);

/**
 * @brief  Register or change the event callback.
 *
 * @studio expose category=tile name=on_event section=events
 * @param  cb   Callback to fire on interrupt events. NULL to clear.
 * @param  ctx  Opaque user pointer passed to the callback.
 */
void tile_sense_i_6p6_on_event(tile_t *tile, sense_i_6p6_event_cb_t cb, void *ctx);

/**
 * @brief  Enter sleep mode (accel + gyro off). ~7.5 µA.
 *
 * @studio expose category=tile name=sleep section=lifecycle
 */
void tile_sense_i_6p6_sleep(tile_t *tile);

/**
 * @brief  Wake from sleep, restore low-noise mode. Range/ODR preserved.
 *
 * @studio expose category=tile name=wake section=lifecycle
 */
void tile_sense_i_6p6_wake(tile_t *tile);

/**
 * @brief  Software reset. Blocks ~2 ms. Must call init() again after.
 *
 * @studio expose category=tile name=reset section=lifecycle
 */
void tile_sense_i_6p6_reset(tile_t *tile);

/* ================================================================
 * Public API — Configuration
 * ================================================================ */

/**
 * @brief  Set accelerometer full-scale range.
 *
 * @studio expose category=tile name=set_accel_range section=runtime
 * @studio control range label="Accel range" tier=basic default=SENSE_I_6P6_ACCEL_8G when="set_power_mode.accel != SENSE_I_6P6_MODE_OFF"
 */
void tile_sense_i_6p6_set_accel_range(tile_t *tile, sense_i_6p6_accel_range_t range);

/**
 * @brief  Set gyroscope full-scale range.
 *
 * @studio expose category=tile name=set_gyro_range section=runtime
 * @studio control range label="Gyro range" tier=basic default=SENSE_I_6P6_GYRO_1000DPS when="set_power_mode.gyro != SENSE_I_6P6_MODE_OFF"
 */
void tile_sense_i_6p6_set_gyro_range(tile_t *tile, sense_i_6p6_gyro_range_t range);

/**
 * @brief  Set accelerometer output data rate.
 *
 * @studio expose category=tile name=set_accel_odr section=runtime
 * @studio control odr label="Accel data rate" tier=basic default=SENSE_I_6P6_ODR_100HZ role=sample_rate when="set_power_mode.accel != SENSE_I_6P6_MODE_OFF"
 * @studio require expr="value(odr) >= 12.5" when="set_power_mode.accel == SENSE_I_6P6_MODE_LN" message="Below 12.5 Hz the accelerometer only runs in low-power mode. Switch the accel power mode to low-power first."
 * @studio require expr="value(odr) <= 500" when="set_power_mode.accel == SENSE_I_6P6_MODE_LP" message="In low-power mode the accelerometer tops out at 500 Hz. Switch the accel power mode to low-noise for faster rates."
 */
void tile_sense_i_6p6_set_accel_odr(tile_t *tile, sense_i_6p6_odr_t odr);

/**
 * @brief  Set gyroscope output data rate.
 *
 * @studio expose category=tile name=set_gyro_odr section=runtime
 * @studio control odr label="Gyro data rate" tier=basic default=SENSE_I_6P6_ODR_100HZ role=sample_rate allow=SENSE_I_6P6_ODR_32KHZ,SENSE_I_6P6_ODR_16KHZ,SENSE_I_6P6_ODR_8KHZ,SENSE_I_6P6_ODR_4KHZ,SENSE_I_6P6_ODR_2KHZ,SENSE_I_6P6_ODR_1KHZ,SENSE_I_6P6_ODR_500HZ,SENSE_I_6P6_ODR_200HZ,SENSE_I_6P6_ODR_100HZ,SENSE_I_6P6_ODR_50HZ,SENSE_I_6P6_ODR_25HZ,SENSE_I_6P6_ODR_12_5HZ when="set_power_mode.gyro != SENSE_I_6P6_MODE_OFF"
 */
void tile_sense_i_6p6_set_gyro_odr(tile_t *tile, sense_i_6p6_odr_t odr);

/**
 * @brief  Set power mode independently for accel and gyro.
 *
 * @studio expose category=tile name=set_power_mode section=runtime
 * @studio control accel label="Accel power mode" tier=advanced default=SENSE_I_6P6_MODE_LN allow=SENSE_I_6P6_MODE_OFF,SENSE_I_6P6_MODE_LP,SENSE_I_6P6_MODE_LN
 * @studio control gyro label="Gyro power mode" tier=advanced default=SENSE_I_6P6_MODE_LN allow=SENSE_I_6P6_MODE_OFF,SENSE_I_6P6_MODE_STANDBY,SENSE_I_6P6_MODE_LN
 * @note   Wait 200 µs after changing mode before accessing other registers.
 */
void tile_sense_i_6p6_set_power_mode(tile_t *tile,
                                      sense_i_6p6_power_mode_t accel,
                                      sense_i_6p6_power_mode_t gyro);

/**
 * @brief  Set UI filter bandwidth for both accel and gyro.
 *
 * @studio expose category=tile name=set_filter_bw section=config
 * @studio control accel_bw label="Accel filter bandwidth" tier=advanced default=SENSE_I_6P6_FILT_BW_ODR_4 allow=SENSE_I_6P6_FILT_BW_ODR_2,SENSE_I_6P6_FILT_BW_ODR_4,SENSE_I_6P6_FILT_BW_ODR_5,SENSE_I_6P6_FILT_BW_ODR_8,SENSE_I_6P6_FILT_BW_ODR_10,SENSE_I_6P6_FILT_BW_ODR_16,SENSE_I_6P6_FILT_BW_ODR_20,SENSE_I_6P6_FILT_BW_ODR_40 show="accel_bw == SENSE_I_6P6_FILT_BW_ODR_2 ? value(set_accel_odr.odr) / 2 : max(400, value(set_accel_odr.odr)) / value(accel_bw)" unit=Hz when="set_power_mode.accel != SENSE_I_6P6_MODE_OFF"
 * @studio require expr="accel_bw == SENSE_I_6P6_FILT_BW_ODR_4 || accel_bw == SENSE_I_6P6_FILT_BW_ODR_20" when="set_power_mode.accel == SENSE_I_6P6_MODE_LP" message="In low-power mode the accel filter averages instead of low-passing: only the /4 setting (1x averaging) and the /20 setting (16x averaging) are valid."
 * @studio control gyro_bw label="Gyro filter bandwidth" tier=advanced default=SENSE_I_6P6_FILT_BW_ODR_4 allow=SENSE_I_6P6_FILT_BW_ODR_2,SENSE_I_6P6_FILT_BW_ODR_4,SENSE_I_6P6_FILT_BW_ODR_5,SENSE_I_6P6_FILT_BW_ODR_8,SENSE_I_6P6_FILT_BW_ODR_10,SENSE_I_6P6_FILT_BW_ODR_16,SENSE_I_6P6_FILT_BW_ODR_20,SENSE_I_6P6_FILT_BW_ODR_40 show="gyro_bw == SENSE_I_6P6_FILT_BW_ODR_2 ? value(set_gyro_odr.odr) / 2 : max(400, value(set_gyro_odr.odr)) / value(gyro_bw)" unit=Hz when="set_power_mode.gyro != SENSE_I_6P6_MODE_OFF"
 */
void tile_sense_i_6p6_set_filter_bw(tile_t *tile,
                                     sense_i_6p6_filter_bw_t accel_bw,
                                     sense_i_6p6_filter_bw_t gyro_bw);

/**
 * @brief  Set UI filter order for accel and gyro (1st, 2nd, or 3rd).
 *
 * @studio expose category=tile name=set_filter_order section=config
 * @studio control accel_order label="Accel filter order" tier=advanced default=SENSE_I_6P6_FILT_ORDER_2ND when="set_power_mode.accel != SENSE_I_6P6_MODE_OFF"
 * @studio control gyro_order label="Gyro filter order" tier=advanced default=SENSE_I_6P6_FILT_ORDER_2ND when="set_power_mode.gyro != SENSE_I_6P6_MODE_OFF"
 */
void tile_sense_i_6p6_set_filter_order(tile_t *tile,
                                        sense_i_6p6_filter_order_t accel_order,
                                        sense_i_6p6_filter_order_t gyro_order);

/**
 * @brief  Set temperature sensor filter bandwidth.
 *
 * @studio expose category=tile name=set_temp_filter section=config
 * @studio control bw label="Temperature filter" tier=advanced default=SENSE_I_6P6_TEMP_FILT_4000HZ when="set_temp_enabled.enabled == 1"
 */
void tile_sense_i_6p6_set_temp_filter(tile_t *tile, sense_i_6p6_temp_filter_t bw);

/**
 * @brief  Enable or disable the temperature sensor.
 *
 * @studio expose category=tile name=set_temp_enabled section=config
 * @studio control enabled label="Temperature sensor" tier=advanced type=bool default=1
 * @param  enabled [0..1] 1 = on, 0 = off.
 */
void tile_sense_i_6p6_set_temp_enabled(tile_t *tile, uint8_t enabled);

/* ================================================================
 * Public API — Data reads (blocking, polled)
 * ================================================================ */

/**
 * @brief  Check if new sensor data is available.
 *
 * @studio expose category=tile name=data_ready returns=bool section=runtime
 */
uint8_t tile_sense_i_6p6_data_ready(tile_t *tile);

/**
 * @brief  Read raw accelerometer [X, Y, Z]. Convert: g = raw / sensitivity.
 *
 * @studio expose category=tile name=get_raw_accels returns=int[3] section=runtime
 * @studio out_buffer buffer type=int16_t length=3
 */
void tile_sense_i_6p6_get_raw_accels(tile_t *tile, int16_t *buffer);

/**
 * @brief  Read raw gyroscope [X, Y, Z]. Convert: dps = raw / sensitivity.
 *
 * @studio expose category=tile name=get_raw_gyros returns=int[3] section=runtime
 * @studio out_buffer buffer type=int16_t length=3
 */
void tile_sense_i_6p6_get_raw_gyros(tile_t *tile, int16_t *buffer);

/**
 * @brief  Burst read accel + gyro [AX, AY, AZ, GX, GY, GZ]. One transaction.
 *
 * @studio expose category=tile name=get_raw_6dof returns=int[6] section=runtime
 * @studio out_buffer buffer type=int16_t length=6
 */
void tile_sense_i_6p6_get_raw_6dof(tile_t *tile, int16_t *buffer);

/**
 * @brief  Read temperature. Convert: degC = raw / 132.48 + 25.0
 *
 * @studio expose category=tile name=get_temperature returns=int section=runtime
 */
int16_t tile_sense_i_6p6_get_temperature(tile_t *tile);

/**
 * @brief  Read all 7 channels [Temp, AX, AY, AZ, GX, GY, GZ] in one burst.
 *
 * @studio expose category=tile name=get_raw_all returns=int[7] section=runtime
 * @studio out_buffer buffer type=int16_t length=7
 */
void tile_sense_i_6p6_get_raw_all(tile_t *tile, int16_t *buffer);

/* ================================================================
 * Public API — Tier-2 idiomatic helpers
 * ================================================================ */

/**
 * @brief  Detect whether the device is approximately face-up (Z ≈ +1g).
 *
 * Reads the accelerometer once and returns 1 when Z is within ±200 mg of
 * +1 g and X/Y are within ±300 mg of 0. Uses the current ACCEL_CONFIG0
 * full-scale range to compute thresholds at runtime, so the helper stays
 * correct after `set_accel_range`. Integer math only.
 *
 * @studio expose category=tile name=is_face_up returns=bool section=runtime
 * @return 1 if face-up, 0 otherwise.
 */
uint8_t tile_sense_i_6p6_is_face_up(tile_t *tile);

/**
 * @brief  Detect whether the device is approximately face-down (Z ≈ −1g).
 *
 * Mirror of `is_face_up` with an inverted Z target. Same ±200 mg /
 * ±300 mg tolerance bands; same range-aware thresholding.
 *
 * @studio expose category=tile name=is_face_down returns=bool section=runtime
 * @return 1 if face-down, 0 otherwise.
 */
uint8_t tile_sense_i_6p6_is_face_down(tile_t *tile);

/**
 * @brief  Detect motion: |‖a‖ − 1g| > threshold.
 *
 * Reads the accelerometer, computes the squared magnitude in raw counts,
 * and compares against the squared 1-g + threshold band. Squared-form
 * keeps the math integer (no sqrt). Threshold is symmetric around 1 g
 * so the helper triggers on both lift and drop. Range-aware.
 *
 * @studio expose category=tile name=is_moving returns=bool section=runtime
 * @studio control threshold_mg label="Motion threshold" tier=basic scope=usage
 * @param  threshold_mg [1..32000] Deviation threshold in milli-g (typical: 50–200).
 * @return 1 if moving, 0 otherwise.
 */
uint8_t tile_sense_i_6p6_is_moving(tile_t *tile, uint16_t threshold_mg);

/**
 * @brief  Tilt angle of one axis vs gravity, in 0.01°.
 *
 * Computes `atan2(axis, sqrt(other_a^2 + other_b^2))` for the requested
 * axis using a small integer atan2 approximation. Output range:
 * −18000..+18000 (i.e. −180.00°..+180.00°). Returns 0 and writes
 * 0 to `*out_centi_deg` if the device is in free-fall or the
 * accelerometer is off. Range-aware.
 *
 * @studio expose category=tile name=read_tilt_centi_degrees returns=bool section=runtime
 * @param  axis           0 = X, 1 = Y, 2 = Z.
 * @param  out_centi_deg  Output tilt in 0.01° (signed, −18000..+18000).
 * @return 1 on success, 0 on bad axis or zero vector.
 */
uint8_t tile_sense_i_6p6_read_tilt_centi_degrees(tile_t *tile,
                                                  uint8_t axis,
                                                  int16_t *out_centi_deg);

/**
 * @brief  Block until a tap interrupt fires or the timeout expires.
 *
 * Polls INT_STATUS3 at ~1 ms cadence (delay_ms(1)). Tap detection must be
 * enabled first via `tile_sense_i_6p6_tap_enable`. The status read is
 * destructive (clear-on-read), so call `get_tap_result` immediately after
 * a positive return to retrieve count/axis/direction.
 *
 * @studio expose category=tile name=wait_for_tap returns=bool section=tap
 * @studio control timeout_ms label="Tap wait timeout" tier=advanced scope=usage
 * @param  timeout_ms [1..60000] ms Max time to wait, in ms. Use 0 for a single-shot poll.
 * @return 1 if a tap was detected, 0 on timeout.
 */
uint8_t tile_sense_i_6p6_wait_for_tap(tile_t *tile, uint32_t timeout_ms);

/**
 * @brief  Block until a wake-on-motion interrupt fires or the timeout expires.
 *
 * Polls INT_STATUS2 at ~1 ms cadence. WOM must be configured (thresholds
 * + enable) first. Returns on any-axis WOM. The status read clears the
 * latch.
 *
 * @studio expose category=tile name=wait_for_motion returns=bool section=wom
 * @studio control timeout_ms label="Motion wait timeout" tier=advanced scope=usage
 * @param  timeout_ms [1..60000] ms Max time to wait, in ms. Use 0 for a single-shot poll.
 * @return 1 if motion was detected, 0 on timeout.
 */
uint8_t tile_sense_i_6p6_wait_for_motion(tile_t *tile, uint32_t timeout_ms);

/* ================================================================
 * Public API — FIFO
 * ================================================================ */

/**
 * @brief  Configure the FIFO.
 *
 * @studio expose category=tile name=fifo_config section=fifo
 * @param  mode       FIFO operating mode (bypass/stream/stop-on-full)
 * @param  accel      Include accel data in FIFO packets
 * @param  gyro       Include gyro data in FIFO packets
 * @param  temp       Include temperature in FIFO packets
 * @param  hires      Enable 20-bit high-resolution mode (forces ±32g/±4000dps)
 */
void tile_sense_i_6p6_fifo_config(tile_t *tile, sense_i_6p6_fifo_mode_t mode,
                                   uint8_t accel, uint8_t gyro,
                                   uint8_t temp, uint8_t hires);

/**
 * @brief  Set the FIFO watermark threshold in records (1–4095).
 *
 * @studio expose category=tile name=fifo_set_watermark section=fifo
 * @studio control records label="FIFO watermark" tier=advanced scope=usage
 * @param  records [1..4095] Records in the FIFO that raise the watermark interrupt.
 */
void tile_sense_i_6p6_fifo_set_watermark(tile_t *tile, uint16_t records);

/**
 * @brief  Flush the FIFO (discard all data).
 *
 * @studio expose category=tile name=flush_fifo section=fifo
 */
void tile_sense_i_6p6_fifo_flush(tile_t *tile);

/**
 * @brief  Read one standard FIFO packet (16 bytes: accel + gyro + temp + timestamp).
 * @return 1 if valid packet read, 0 if FIFO empty or error.
 */
uint8_t tile_sense_i_6p6_fifo_read_packet(tile_t *tile, sense_i_6p6_fifo_packet_t *pkt);

/**
 * @brief  DSL-friendly flat-output variant of fifo_read_packet().
 *
 * Drops the struct in favor of a positional int[8] array — the DSL
 * doesn't have a struct ABI yet, so the per-field outputs come back
 * indexed. Layout:
 *   out[0..2] = accel x,y,z   (raw int16 widened to int32)
 *   out[3..5] = gyro x,y,z    (raw int16 widened to int32)
 *   out[6]    = temp          (raw int8 widened to int32)
 *   out[7]    = timestamp     (raw uint16)
 *
 * Drops the success bool — call fifo_count() first to know whether
 * there's data. On an empty FIFO, the slots are left untouched.
 *
 * @studio expose category=tile name=fifo_read_packet returns=int[8] section=fifo
 * @studio out_buffer out type=int32_t length=8
 *
 * @param  tile  Initialised tile handle.
 * @param  out   Output buffer (8 int32_t slots).
 */
void tile_sense_i_6p6_fifo_read_packet_flat(tile_t *tile, int32_t *out);

/**
 * @brief  Read multiple FIFO packets.
 * @param  packets    Output array
 * @param  max_count  Maximum packets to read
 * @return Number of packets actually read
 */
uint16_t tile_sense_i_6p6_fifo_read_packets(tile_t *tile,
                                              sense_i_6p6_fifo_packet_t *packets,
                                              uint16_t max_count);

/**
 * @brief  DSL-friendly flat-output variant of fifo_read_packets().
 *
 * Drains up to N packets from the FIFO into the caller's int32 buffer,
 * 8 ints per packet (same layout as fifo_read_packet_flat). Returns
 * the number of *packets* read (not ints), so callers can iterate
 * with `for i in 0..n { let ax = buf[i*8 + 0]; ... }`.
 *
 * The caller's array length is the int-cap (N packets × 8 ints), not
 * the packet count — `cap_ints / 8` packets are attempted. Mirrors
 * the cap-mode array-OUT convention used by Sense.BP read_fifo_batch.
 *
 * @studio expose category=tile name=fifo_read_packets returns=int section=fifo
 * @studio out_buffer out type=int32_t cap_param=cap_ints
 *
 * @param  tile      Initialised tile handle.
 * @param  out       Output buffer; must be sized N × 8 ints.
 * @param  cap_ints  Buffer capacity in int slots (= packets × 8).
 * @return Number of packets actually drained from the FIFO.
 */
uint16_t tile_sense_i_6p6_fifo_read_packets_flat(tile_t *tile,
                                                  int32_t *out,
                                                  uint16_t cap_ints);

/**
 * @brief  Read the FIFO record count.
 *
 * @studio expose category=tile name=fifo_count returns=int section=fifo
 */
uint16_t tile_sense_i_6p6_fifo_count(tile_t *tile);

/**
 * @brief  Get the number of lost FIFO packets since last check.
 *
 * @studio expose category=tile name=fifo_lost_count returns=int section=fifo
 */
uint16_t tile_sense_i_6p6_fifo_lost_count(tile_t *tile);

/* ================================================================
 * Public API — Interrupts
 * ================================================================ */

/**
 * @brief  Configure INT1 pin behavior.
 *
 * @studio expose category=tile name=int1_config section=interrupts
 * @param  config  OR'd flags: ACTIVE_HIGH|PUSH_PULL|LATCHED etc.
 */
void tile_sense_i_6p6_int1_config(tile_t *tile, uint8_t config);

/**
 * @brief  Route data-ready interrupt to INT1.
 *
 * @studio expose category=tile name=int1_data_ready section=interrupts
 * @param  enabled  1 to enable, 0 to disable
 */
void tile_sense_i_6p6_int1_data_ready(tile_t *tile, uint8_t enabled);

/**
 * @brief  Route FIFO watermark interrupt to INT1.
 *
 * @studio expose category=tile name=int1_fifo_ths section=interrupts
 * @param  enabled  1 to enable, 0 to disable
 */
void tile_sense_i_6p6_int1_fifo_ths(tile_t *tile, uint8_t enabled);

/**
 * @brief  Route WOM (wake-on-motion) interrupt to INT1.
 *
 * @studio expose category=tile name=int1_wom section=interrupts
 * @param  enabled  1 to enable, 0 to disable
 */
void tile_sense_i_6p6_int1_wom(tile_t *tile, uint8_t enabled);

/**
 * @brief  Configure INT2 pin behavior.
 *
 * Mirrors int1_config for the chip's second interrupt pin (tile pad 8).
 * INT_CONFIG bits [5:3] hold INT2's polarity / drive / mode bits.
 *
 * @studio expose category=tile name=int2_config section=interrupts
 * @param  config  OR'd flags: ACTIVE_HIGH|PUSH_PULL|LATCHED etc. (same
 *                 layout as int1_config; the driver shifts the bits
 *                 into the INT2 positions internally).
 */
void tile_sense_i_6p6_int2_config(tile_t *tile, uint8_t config);

/**
 * @brief  Route data-ready interrupt to INT2.
 *
 * @studio expose category=tile name=int2_data_ready section=interrupts
 * @param  enabled  1 to enable, 0 to disable
 */
void tile_sense_i_6p6_int2_data_ready(tile_t *tile, uint8_t enabled);

/**
 * @brief  Route FIFO watermark interrupt to INT2.
 *
 * @studio expose category=tile name=int2_fifo_ths section=interrupts
 * @param  enabled  1 to enable, 0 to disable
 */
void tile_sense_i_6p6_int2_fifo_ths(tile_t *tile, uint8_t enabled);

/**
 * @brief  Route WOM (wake-on-motion) interrupt to INT2.
 *
 * @studio expose category=tile name=int2_wom section=interrupts
 * @param  enabled  1 to enable, 0 to disable
 */
void tile_sense_i_6p6_int2_wom(tile_t *tile, uint8_t enabled);

/** Pulse-width selector for INT pin pulse mode. */
typedef enum {
    SENSE_I_6P6_INT_PULSE_100US = 0,  /**< 100 µs pulse (default) */
    SENSE_I_6P6_INT_PULSE_8US   = 1,  /**< 8 µs pulse — for fast hosts */
} sense_i_6p6_int_pulse_t;

/**
 * @brief  Set the INT pin pulse-width when configured for pulse mode.
 *
 * @studio expose category=tile name=set_int_pulse_duration section=interrupts
 *
 * Applies to both INT1 and INT2 (it's a chip-wide setting in
 * INT_CONFIG1 bit 6). Only affects pulse-mode interrupts; in latched
 * mode the line stays asserted until the status register is read.
 *
 * @param  tile   Initialised tile handle
 * @param  pulse  Pulse-width selector
 */
void tile_sense_i_6p6_set_int_pulse_duration(tile_t *tile,
                                             sense_i_6p6_int_pulse_t pulse);

/** Subsystem selector for scoped resets. */
typedef enum {
    SENSE_I_6P6_RESET_APEX = 0,  /**< APEX (DMP) memory + state */
    SENSE_I_6P6_RESET_TEMP = 1,  /**< Temperature signal path */
} sense_i_6p6_subsystem_t;

/**
 * @brief  Reset a single chip subsystem without touching the others.
 *
 * @studio expose category=tile name=subsystem_reset section=advanced
 *
 * Issues the relevant SIGNAL_PATH_RESET bit. Distinct from the
 * driver's full `reset()` (which re-initialises the whole chip) and
 * from `fifo_flush()` (which targets just the FIFO, also a
 * subsystem reset but already exposed).
 *
 *   - APEX: clears DMP memory and re-runs the DMP init sequence.
 *           Use after reconfiguring pedometer / tilt / tap features.
 *   - TEMP: resets the temperature signal path. Useful if temp
 *           readings appear stuck after a power glitch.
 *
 * @param  tile   Initialised tile handle
 * @param  which  Subsystem to reset
 */
void tile_sense_i_6p6_subsystem_reset(tile_t *tile,
                                      sense_i_6p6_subsystem_t which);

/**
 * @brief  Read and clear INT_STATUS register.
 *
 * @studio expose category=tile name=get_int_status returns=int section=interrupts
 */
uint8_t tile_sense_i_6p6_get_int_status(tile_t *tile);

/**
 * @brief  Read and clear INT_STATUS2 (WOM/SMD flags).
 *
 * @studio expose category=tile name=get_int_status2 returns=int section=interrupts
 */
uint8_t tile_sense_i_6p6_get_int_status2(tile_t *tile);

/**
 * @brief  Read and clear INT_STATUS3 (APEX: step/tilt/tap flags).
 *
 * @studio expose category=tile name=get_int_status3 returns=int section=interrupts
 */
uint8_t tile_sense_i_6p6_get_int_status3(tile_t *tile);

/* ================================================================
 * Public API — Wake on Motion (WOM)
 * ================================================================ */

/**
 * @brief  Configure Wake-on-Motion thresholds.
 *
 * @studio expose category=tile name=wom_config section=wom
 * @studio control x_mg label="Wake threshold X" tier=advanced scope=usage
 * @studio control y_mg label="Wake threshold Y" tier=advanced scope=usage
 * @studio control z_mg label="Wake threshold Z" tier=advanced scope=usage
 * @studio control mode label="Wake compare" tier=advanced default=SENSE_I_6P6_WOM_INITIAL scope=usage
 * @param  x_mg [0..1000] X-axis threshold in mg (resolution ~3.9 mg)
 * @param  y_mg [0..1000] Y-axis threshold in mg
 * @param  z_mg [0..1000] Z-axis threshold in mg
 * @param  mode  Compare against initial or previous sample
 *
 * @note  Requires accel to be running (LN or LP mode).
 */
void tile_sense_i_6p6_wom_config(tile_t *tile,
                                  uint16_t x_mg, uint16_t y_mg, uint16_t z_mg,
                                  sense_i_6p6_wom_mode_t mode);

/**
 * @brief  Enable WOM. Must configure thresholds first.
 *
 * @studio expose category=tile name=wom_enable section=wom
 */
void tile_sense_i_6p6_wom_enable(tile_t *tile);

/**
 * @brief  Disable WOM.
 *
 * @studio expose category=tile name=wom_disable section=wom
 */
void tile_sense_i_6p6_wom_disable(tile_t *tile);

/* ================================================================
 * Public API — Significant Motion Detection (SMD)
 * ================================================================ */

/**
 * @brief  Configure and enable SMD. WOM thresholds must be set first.
 *
 * @studio expose category=tile name=smd_config section=smd
 * @studio control mode label="Significant motion" tier=advanced default=SENSE_I_6P6_SMD_DISABLED scope=usage
 */
void tile_sense_i_6p6_smd_config(tile_t *tile, sense_i_6p6_smd_mode_t mode);

/* ================================================================
 * Public API — APEX: Pedometer
 * ================================================================ */

/**
 * @brief  Enable the pedometer (step counter + step detector).
 *
 * Requires accel at ≥25 Hz. Initializes the DMP if not already running.
 *
 * @studio expose category=tile name=pedometer_enable section=pedometer
 * @studio control dmp_odr label="Pedometer rate" tier=advanced scope=usage
 * @param  dmp_odr  DMP processing rate (25 or 50 Hz)
 */
void tile_sense_i_6p6_pedometer_enable(tile_t *tile, sense_i_6p6_dmp_odr_t dmp_odr);

/**
 * @brief  Disable the pedometer.
 *
 * @studio expose category=tile name=pedometer_disable section=pedometer
 */
void tile_sense_i_6p6_pedometer_disable(tile_t *tile);

/**
 * @brief  Read the step count.
 *
 * @studio expose category=tile name=get_step_count returns=int section=pedometer
 * @return 16-bit step count (resets on power cycle or DMP reset)
 */
uint16_t tile_sense_i_6p6_get_step_count(tile_t *tile);

/**
 * @brief  Read the step cadence.
 *
 * @studio expose category=tile name=get_step_cadence returns=int section=pedometer
 * @return Steps per second in u6.2 fixed-point (divide by 4.0 for float)
 */
uint8_t tile_sense_i_6p6_get_step_cadence(tile_t *tile);

/**
 * @brief  Read the activity classification.
 *
 * @studio expose category=tile name=get_activity returns=int section=pedometer
 * @return 0=unknown, 1=walk, 2=run
 */
sense_i_6p6_activity_t tile_sense_i_6p6_get_activity(tile_t *tile);

/* ================================================================
 * Public API — APEX: Tilt Detection
 * ================================================================ */

/**
 * @brief  Enable tilt detection.
 *
 * Triggers when device tilts >35° for the configured wait time.
 * Requires accel at ≥25 Hz. Initializes DMP if needed.
 *
 * @studio expose category=tile name=enable_tilt section=tilt
 * @studio control wait_seconds label="Tilt delay" tier=advanced scope=usage
 * @param wait_seconds [0..6] s Time the tilt must be sustained (0, 2, 4, or 6).
 */
void tile_sense_i_6p6_tilt_enable(tile_t *tile, uint8_t wait_seconds);

/**
 * @brief  Disable tilt detection.
 *
 * @studio expose category=tile name=disable_tilt section=tilt
 */
void tile_sense_i_6p6_tilt_disable(tile_t *tile);

/* ================================================================
 * Public API — APEX: Tap Detection
 * ================================================================ */

/**
 * @brief  Enable tap detection (single + double tap).
 *
 * Requires accel in LN mode at 200 Hz, 500 Hz, or 1 kHz.
 * Initializes DMP if needed.
 *
 * @studio expose category=tile name=enable_tap section=tap
 */
void tile_sense_i_6p6_tap_enable(tile_t *tile);

/**
 * @brief  Disable tap detection.
 *
 * @studio expose category=tile name=disable_tap section=tap
 */
void tile_sense_i_6p6_tap_disable(tile_t *tile);

/**
 * @brief  Read the last tap detection result.
 * @param  result  Output tap information
 */
void tile_sense_i_6p6_get_tap_result(tile_t *tile, sense_i_6p6_tap_result_t *result);

/**
 * @brief  DSL-friendly flat-output variant of get_tap_result().
 *
 * Same data, four positional out-scalars instead of a struct. Useful
 * for polling-style flows; event-driven flows already get these values
 * via the `Sense.I.6P6.tap` event payload.
 *
 * @studio expose category=tile name=get_tap_result section=tap
 * @studio out_scalar count type=int32_t
 * @studio out_scalar axis type=int32_t
 * @studio out_scalar direction type=int32_t
 * @studio out_scalar timing type=int32_t
 *
 * @param  tile       Initialised tile handle.
 * @param  count      Output: 0 = none, 1 = single tap, 2 = double.
 * @param  axis       Output: 0 = X, 1 = Y, 2 = Z.
 * @param  direction  Output: 0 = positive, 1 = negative.
 * @param  timing     Output: double-tap timing (chip-specific units).
 */
void tile_sense_i_6p6_get_tap_result_flat(tile_t *tile,
                                          int32_t *count,
                                          int32_t *axis,
                                          int32_t *direction,
                                          int32_t *timing);

/* ================================================================
 * Public API — Advanced
 * ================================================================ */

/**
 * @brief  Set user-programmable gyro offset.
 *
 * @studio expose category=tile name=set_gyro_offset section=advanced
 * @param  x_dps  X offset in dps × 16 (12-bit signed, ±128 dps max, 0.0625 dps resolution)
 * @param  y_dps  Y offset
 * @param  z_dps  Z offset
 */
void tile_sense_i_6p6_set_gyro_offset(tile_t *tile,
                                       int16_t x_dps16, int16_t y_dps16, int16_t z_dps16);

/**
 * @brief  Set user-programmable accel offset.
 *
 * @studio expose category=tile name=set_accel_offset section=advanced
 * @param  x_mg  X offset in mg (12-bit signed, ±2000 mg max, 1 mg resolution)
 * @param  y_mg  Y offset
 * @param  z_mg  Z offset
 */
void tile_sense_i_6p6_set_accel_offset(tile_t *tile,
                                        int16_t x_mg, int16_t y_mg, int16_t z_mg);

/**
 * @brief  Perform self-test.
 *
 * Runs the built-in self-test for both accel and gyro.
 *
 * @studio expose category=tile name=self_test returns=bool section=advanced
 * @studio out_scalar accel_pass type=uint8_t
 * @studio out_scalar gyro_pass type=uint8_t
 *
 * @param  accel_pass  Output: 1 if accel self-test passed, 0 if failed (per-axis OR'd)
 * @param  gyro_pass   Output: 1 if gyro self-test passed, 0 if failed
 * @return 1 if both passed, 0 if either failed
 */
uint8_t tile_sense_i_6p6_self_test(tile_t *tile, uint8_t *accel_pass, uint8_t *gyro_pass);

/**
 * @brief  Read a register from any bank.
 *
 * Automatically switches to the specified bank and back to bank 0.
 * For advanced/debug use.
 *
 * @studio expose category=tile name=read_reg returns=int section=advanced
 * @param  bank  Bank index (0, 1, 2, or 4).
 * @param  reg   Register address within the bank.
 */
uint8_t tile_sense_i_6p6_read_reg(tile_t *tile, uint8_t bank, uint8_t reg);

/**
 * @brief  Write a register in any bank.
 *
 * Automatically switches to the specified bank and back to bank 0.
 * For advanced/debug use.
 *
 * @studio expose category=tile name=write_reg section=advanced
 * @param  bank  Bank index (0, 1, 2, or 4).
 * @param  reg   Register address within the bank.
 * @param  val   Byte to write.
 */
void tile_sense_i_6p6_write_reg(tile_t *tile, uint8_t bank, uint8_t reg, uint8_t val);

#endif /* INC_TILE_SENSE_I_6P6_H_ */
