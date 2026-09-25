/**
 * @file   tile_sense_i_9.h
 * @brief  9-DOF IMU driver for the Sense.I.9 tile (rev c).
 * @version 3.1.1
 *
 * Embeds the TDK InvenSense ICM-20948: 6-DOF IMU (accel + gyro)
 * with co-packaged AK09916 3-DOF magnetometer (accessed via the
 * ICM's I2C bypass).
 *
 * Sensor specifications:
 *   - Accelerometer:  16-bit, ±2/4/8/16 G, up to 4.5 kHz ODR
 *   - Gyroscope:      16-bit, ±250/500/1000/2000 DPS, up to 9 kHz ODR
 *   - Magnetometer:   16-bit, ±4900 µT, up to 100 Hz ODR (0.15 µT/LSB)
 *   - Temperature:    on-chip sensor
 *   - FIFO:           512-byte buffer, accel + gyro + temp packets
 *   - Wake-on-Motion: 4 mg/LSB threshold, INT routable
 *
 * Datasheet: https://www.bergsonne.io/tiles/sense/i9
 *
 * @studio tile label=Sense.I.9 icon=⊙
 * @studio event name=data_ready mask=ICM20948_INT_RAW_DATA_RDY
 * @studio event name=fifo_watermark mask=ICM20948_INT_FIFO_WM
 * @studio event name=fifo_overflow mask=ICM20948_INT_FIFO_OVF
 * @studio event name=wake_on_motion mask=ICM20948_INT_WOM
 *
 * Quick start:
 * @code
 *   #include "core_tiles.h"
 *
 *   tile_t imu;
 *   tile_sense_i_9_init(core_tiles_pal(&core_i2c1), 0, &imu, NULL);
 *   if (tile_is_ready(&imu)) {
 *       int16_t accel[3], gyro[3], mag[3];
 *       tile_sense_i_9_get_raw_accels(&imu, accel);
 *       tile_sense_i_9_get_raw_gyros(&imu, gyro);
 *       tile_sense_i_9_get_raw_mags(&imu, mag);
 *   }
 * @endcode
 *
 * Two tiles on one bus:
 * @code
 *   tile_t imu_a, imu_b;
 *   tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
 *   tile_sense_i_9_init(hal, 0, &imu_a, NULL);  // pad 2 floating (0x69)
 *   tile_sense_i_9_init(hal, 1, &imu_b, NULL);  // pad 2 grounded (0x68)
 * @endcode
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=common category="DMP3 — extended outputs" section=advanced
 *   Phase 1 (firmware load + verify) and Phase 2 (9-axis quaternion /
 *   ROTATION_VECTOR) are now exposed via dmp_load / dmp_start_quat9 /
 *   dmp_read_quat9. Still unimplemented: 6-axis quaternion (no-mag),
 *   geomagnetic rotation vector, pedometer + step detector, BAC activity
 *   classification, tap / double-tap, significant-motion detection,
 *   pickup, tilt, fsync timestamping, and the multi-sensor accumulator
 *   that lets several DMP outputs share one stream. Phase 4 plans
 *   dmp_read_orientation (yaw/pitch/roll) on top of Quat9.
 *
 * @studio unsupported severity=niche category="AK09916 FUSE ROM sensitivity adjustment"
 *   Not applicable to this chip. Earlier AKM parts (e.g. AK8963) shipped
 *   per-axis ASA codes in FUSE ROM that required `raw × (ASA + 128) / 256`
 *   correction. The AK09916 in the ICM-20948 has no FUSE ROM and no ASA
 *   registers (datasheet rev 015007392-E-02 §11) — sensitivity is
 *   factory-trimmed to a fixed 0.15 µT/LSB across all axes. Driver
 *   correctly returns raw counts at that scale; no per-axis correction
 *   is needed or possible. Annotation kept to document the verification.
 *
 * @studio unsupported severity=advanced category="Sensor hub for external aux sensors" section=advanced
 *   Driver uses INT_PIN_CFG.BYPASS_EN by default so the AK09916 is on
 *   the host bus directly (simpler + faster than routing through the
 *   ICM's internal I2C master). dmp_start_quat9() now drives the master
 *   internally — claiming slots 0 + 1 — but slots 2 + 3 remain unused
 *   and there is no public API for attaching user aux sensors to them.
 *   Closing this gap means promoting the I2C-master configuration into
 *   a public surface and reserving DMP's slot use behind it.
 *
 * @studio unsupported severity=advanced category="FSYNC external-clock / timestamping"
 *   Hardware-gated. Chip can take a 31–50 kHz FSYNC input and stamp
 *   samples against external timing. The Sense.I.9-c tile does not
 *   expose the FSYNC pin on any pad (verified in tile JSON: pads 6, 7,
 *   and 8 carry no function). Closing this gap requires a tile
 *   hardware revision.
 *
 * @studio unsupported severity=advanced category="Alternate bus modes (SPI / I3C)"
 *   Ecosystem-gated. Tile JSON straps support I²C (default) or SPI 4-wire
 *   on the same pads (AD0 = MISO, EN = CS), and the ICM-20948 is also
 *   I3C electrically compliant. The driver framework currently uses
 *   tiles_pal I²C calls only; closing requires extending the bus
 *   abstraction (see Sense.I.6P6 v1.0+ for the SPI pattern). Defer to
 *   a multi-bus driver framework pass.
 */

#ifndef INC_TILE_SENSE_I_9_H_
#define INC_TILE_SENSE_I_9_H_

#include "tiles.h"
#include <stdint.h>

/* -------------------------------------------------------------- */
/* Driver version                                                  */
/* -------------------------------------------------------------- */

#define TILE_SENSE_I_9_VERSION_MAJOR  3
#define TILE_SENSE_I_9_VERSION_MINOR  1
#define TILE_SENSE_I_9_VERSION_PATCH  1

TILES_CHECK_VERSION(1, 0);  /* requires tiles.h >= 1.0 */

/* -------------------------------------------------------------- */
/* Instance mapping                                                */
/* -------------------------------------------------------------- */

/**
 * @brief  Instance-to-address mapping for Sense.I.9.
 *
 * | Instance | ID   | Bus  | Hardware config                    |
 * |----------|------|------|------------------------------------|
 * | 0        | 0x69 | I2C  | Pad 2 (AD0) floating — default    |
 * | 1        | 0x68 | I2C  | Pad 2 (AD0) tied to GND           |
 *
 * @note  Pad 3 (I2C.EN) has a weak pull-up enabling I2C by default.
 *        Grounding pad 3 switches to SPI mode (not yet supported).
 */
#define ICM20948_I2C_ADDR_DEFAULT   0x69
#define ICM20948_I2C_ADDR_ALT       0x68

/** @brief  AK09916 magnetometer address (fixed, accessed via I2C bypass). */
#define AK09916_I2C_ADDR            0x0C

/* -------------------------------------------------------------- */
/* ICM-20948 register map                                          */
/* -------------------------------------------------------------- */

/* Bank selection */
#define ICM20948_BANK_0             0x00
#define ICM20948_BANK_1             0x01
#define ICM20948_BANK_2             0x02
#define ICM20948_BANK_3             0x03
#define ICM20948_REG_BANK_SEL       0x7F

/* Bank 0 registers */
#define ICM20948_REG_WHOAMI         0x00
#define ICM20948_USER_CTRL          0x03
#define ICM20948_REG_PWR_MGMT_1     0x06
#define ICM20948_REG_PWR_MGMT_2     0x07
#define ICM20948_REG_INT_PIN_CFG    0x0F
#define ICM20948_REG_INT_ENABLE     0x10  /**< WoM / DMP / I2C master */
#define ICM20948_REG_INT_ENABLE_1   0x11  /**< RAW_DATA_0_RDY_EN */
#define ICM20948_REG_INT_ENABLE_2   0x12  /**< FIFO_OVERFLOW_EN[4:0] */
#define ICM20948_REG_INT_ENABLE_3   0x13  /**< FIFO_WM_EN[4:0] */
#define ICM20948_REG_INT_STATUS     0x19
#define ICM20948_REG_INT_STATUS_1   0x1A
#define ICM20948_REG_INT_STATUS_2   0x1B  /**< FIFO_OVERFLOW_INT[4:0] */
#define ICM20948_REG_INT_STATUS_3   0x1C  /**< FIFO_WM_INT[4:0] */
#define ICM20948_REG_ACCEL_X_H      0x2D
#define ICM20948_REG_GYRO_X_H       0x33
#define ICM20948_REG_TEMP_H         0x39
#define ICM20948_REG_FIFO_EN_1      0x66
#define ICM20948_REG_FIFO_EN_2      0x67
#define ICM20948_REG_FIFO_RST       0x68
#define ICM20948_REG_FIFO_MODE      0x69
#define ICM20948_REG_FIFO_COUNTH    0x70
#define ICM20948_REG_FIFO_COUNTL    0x71
#define ICM20948_REG_FIFO_R_W       0x72

/* DMP RAM access (bank 0). The DMP firmware (~14 KB) lives in an
 * indirectly-addressed RAM split into 256-byte banks: write the 8-bit
 * bank number to MEM_BANK_SEL (the firmware spans banks 0..0x38, well
 * past 16 — do NOT mask the bank to 4 bits), the low-byte address to
 * MEM_START_ADDR (auto-increments on burst access), then read or write
 * through MEM_R_W. */
#define ICM20948_REG_MEM_START_ADDR 0x7C
#define ICM20948_REG_MEM_R_W        0x7D
#define ICM20948_REG_MEM_BANK_SEL   0x7E

/* USER_CTRL (0x03) bit fields used by DMP. */
#define ICM20948_USER_CTRL_DMP_EN     (1U << 7)
#define ICM20948_USER_CTRL_FIFO_EN    (1U << 6)
#define ICM20948_USER_CTRL_I2C_MST_EN (1U << 5)
#define ICM20948_USER_CTRL_DMP_RST    (1U << 3)
#define ICM20948_USER_CTRL_FIFO_RST   (1U << 2)

/* Bank 1 registers (self-test reference values) */
#define ICM20948_B1_SELF_TEST_X_GYRO  0x02
#define ICM20948_B1_SELF_TEST_Y_GYRO  0x03
#define ICM20948_B1_SELF_TEST_Z_GYRO  0x04
#define ICM20948_B1_SELF_TEST_X_ACCEL 0x0E
#define ICM20948_B1_SELF_TEST_Y_ACCEL 0x0F
#define ICM20948_B1_SELF_TEST_Z_ACCEL 0x10

/* Bank 2 registers */
#define ICM20948_REG_GYRO_SMPLRT      0x00
#define ICM20948_REG_GYRO_CONFIG      0x01
#define ICM20948_REG_GYRO_CONFIG_2    0x02  /**< Gyro self-test enables [5:3] + LP averaging */
#define ICM20948_REG_ACCEL_SMPLRT_H   0x10
#define ICM20948_REG_ACCEL_SMPLRT_L   0x11
#define ICM20948_REG_ACCEL_INTEL_CTRL 0x12  /**< WoM enable + mode */
#define ICM20948_REG_ACCEL_WOM_THR    0x13  /**< WoM threshold (4 mg/LSB) */
#define ICM20948_REG_ACCEL_CONFIG     0x14
#define ICM20948_REG_ACCEL_CONFIG_2   0x15  /**< Self-test enables + averaging */
#define ICM20948_REG_PRGM_START_ADDRH 0x50  /**< DMP firmware entry-point hi */
#define ICM20948_REG_PRGM_START_ADDRL 0x51  /**< DMP firmware entry-point lo */

/* Bank 3 registers (I2C master configuration). The DMP path drives the
 * AK09916 through the chip's internal I2C master — see comments around
 * tile_sense_i_9_dmp_start_quat9() for the rationale. */
#define ICM20948_REG_I2C_MST_ODR_CFG  0x00  /**< Mag read rate when gyro+accel are LP */
#define ICM20948_REG_I2C_MST_CTRL     0x01  /**< Bus speed + NSR */
#define ICM20948_REG_I2C_MST_DLY_CTRL 0x02
#define ICM20948_REG_I2C_PERIPH0_ADDR 0x03  /**< Slot 0 → AK09916 bulk read */
#define ICM20948_REG_I2C_PERIPH0_REG  0x04
#define ICM20948_REG_I2C_PERIPH0_CTRL 0x05
#define ICM20948_REG_I2C_PERIPH0_DO   0x06
#define ICM20948_REG_I2C_PERIPH1_ADDR 0x07  /**< Slot 1 → AK09916 single-meas trigger */
#define ICM20948_REG_I2C_PERIPH1_REG  0x08
#define ICM20948_REG_I2C_PERIPH1_CTRL 0x09
#define ICM20948_REG_I2C_PERIPH1_DO   0x0A

/* Bank 0: more registers used by DMP setup. */
#define ICM20948_REG_LP_CONFIG        0x05  /**< Per-block low-power cycling */
#define ICM20948_REG_SINGLE_FIFO_PRIO 0x26  /**< InvenSense-internal magic */
#define ICM20948_REG_HW_FIX_DISABLE   0x75  /**< InvenSense-internal magic */
#define ICM20948_REG_DATA_RDY_STATUS  0x74

/* INT_PIN_CFG bits (bank 0, 0x0F) */
#define ICM20948_INT_PIN_BYPASS_EN    (1U << 1)

/* AK09916 register address used by the DMP-side bulk read. The DMP
 * expects ten bytes starting at 0x03 (RSV2) — see initializeDMP()
 * in the SparkFun port for the empirical reverse-engineering note. */
#define AK09916_REG_RSV2              0x03

/* USER_CTRL bits (bank 0, 0x03) */
#define ICM20948_UC_FIFO_EN           (1 << 6)
#define ICM20948_UC_I2C_MST_EN        (1 << 5)
#define ICM20948_UC_I2C_IF_DIS        (1 << 4)
#define ICM20948_UC_DMP_RST           (1 << 3)
#define ICM20948_UC_SRAM_RST          (1 << 2)
#define ICM20948_UC_I2C_MST_RST       (1 << 1)

/* INT_ENABLE bits */
#define ICM20948_INTE_REG_WOF_EN      (1 << 7)
#define ICM20948_INTE_WOM_INT_EN      (1 << 3)
#define ICM20948_INTE_PLL_RDY_EN      (1 << 2)
#define ICM20948_INTE_DMP_INT1_EN     (1 << 1)
#define ICM20948_INTE_I2C_MST_INT_EN  (1 << 0)

/* INT_STATUS event masks (mapped to single byte for studio events) */
#define ICM20948_INT_RAW_DATA_RDY     (1 << 0)  /**< INT_STATUS_1 bit 0 */
#define ICM20948_INT_FIFO_OVF         (1 << 1)  /**< INT_STATUS_2 any bit (collapsed) */
#define ICM20948_INT_FIFO_WM          (1 << 2)  /**< INT_STATUS_3 any bit (collapsed) */
#define ICM20948_INT_WOM              (1 << 3)  /**< INT_STATUS bit 3 */

/* Chip ID */
#define ICM20948_WHOAMI_DEFAULT     0xEA

/* -------------------------------------------------------------- */
/* AK09916 magnetometer register map                               */
/* -------------------------------------------------------------- */

#define AK09916_REG_WIA2            0x01
#define AK09916_REG_ST1             0x10
#define AK09916_REG_HXL             0x11
#define AK09916_REG_ST2             0x18
#define AK09916_REG_CNTL2           0x31
#define AK09916_REG_CNTL3           0x32

#define AK09916_WHOAMI_DEFAULT      0x09

/* AK09916 ST2 bits */
#define AK09916_ST2_HOFL            (1 << 3)  /**< Magnetic overflow flag */

/* AK09916 self-test mode (CNTL2 = 0x10) */
#define AK09916_MODE_SELF_TEST      0x10

/* -------------------------------------------------------------- */
/* Configuration enums                                             */
/* -------------------------------------------------------------- */

/**
 * @brief  Accelerometer full-scale range.
 *
 * Sensitivity (LSB/g) for each range:
 *   SENSE_I_9_ACCEL_2G  → 16384,  SENSE_I_9_ACCEL_4G  → 8192,
 *   SENSE_I_9_ACCEL_8G  → 4096,   SENSE_I_9_ACCEL_16G → 2048
 */
typedef enum {
    SENSE_I_9_ACCEL_2G   = 0x00,  /**< +/- 2g */
    SENSE_I_9_ACCEL_4G   = 0x02,  /**< bits [2:1] of ACCEL_CONFIG */
    SENSE_I_9_ACCEL_8G   = 0x04,  /**< +/- 8g */
    SENSE_I_9_ACCEL_16G  = 0x06,  /**< +/- 16g */
} sense_i_9_accel_range_t;

/**
 * @brief  Gyroscope full-scale range.
 *
 * Sensitivity (LSB/°/s) for each range:
 *   SENSE_I_9_GYRO_250DPS  → 131.0,  SENSE_I_9_GYRO_500DPS  → 65.5,
 *   SENSE_I_9_GYRO_1000DPS → 32.8,   SENSE_I_9_GYRO_2000DPS → 16.4
 */
typedef enum {
    SENSE_I_9_GYRO_250DPS  = 0x00,  /**< +/- 250 deg/s */
    SENSE_I_9_GYRO_500DPS  = 0x02,  /**< bits [2:1] of GYRO_CONFIG */
    SENSE_I_9_GYRO_1000DPS = 0x04,  /**< +/- 1000 deg/s */
    SENSE_I_9_GYRO_2000DPS = 0x06,  /**< +/- 2000 deg/s */
} sense_i_9_gyro_range_t;

/**
 * @brief  Magnetometer operating mode.
 *
 * Single-measurement mode requires re-triggering for each read.
 * Continuous modes free-run at the specified rate.
 */
typedef enum {
    SENSE_I_9_MAG_POWER_DOWN       = 0x00,  /**< Power-down */
    SENSE_I_9_MAG_SINGLE           = 0x01,  /**< Single measurement */
    SENSE_I_9_MAG_CONTINUOUS_10HZ  = 0x02,  /**< Continuous at 10 Hz */
    SENSE_I_9_MAG_CONTINUOUS_20HZ  = 0x04,  /**< Continuous at 20 Hz */
    SENSE_I_9_MAG_CONTINUOUS_50HZ  = 0x06,  /**< Continuous at 50 Hz */
    SENSE_I_9_MAG_CONTINUOUS_100HZ = 0x08,  /**< Continuous at 100 Hz */
} sense_i_9_mag_mode_t;

/**
 * @brief  INT pin configuration flags. OR together for `int_config`.
 *
 * Active-low + open-drain + latched is typical for shared INT lines.
 */
typedef enum {
    SENSE_I_9_INT_ACTIVE_HIGH  = 0x00,  /**< INT pin active high (default) */
    SENSE_I_9_INT_ACTIVE_LOW   = 0x80,  /**< INT pin active low */
    SENSE_I_9_INT_PUSH_PULL    = 0x00,  /**< Push-pull driver */
    SENSE_I_9_INT_OPEN_DRAIN   = 0x40,  /**< Open-drain driver */
    SENSE_I_9_INT_PULSED       = 0x00,  /**< 50 µs pulse (default) */
    SENSE_I_9_INT_LATCHED      = 0x20,  /**< Held until status read */
    SENSE_I_9_INT_ANYRD_CLEAR  = 0x10,  /**< Clear status on any read */
} sense_i_9_int_flags_t;

/**
 * @brief  Wake-on-Motion compare mode (ACCEL_INTEL_MODE_INT).
 */
typedef enum {
    SENSE_I_9_WOM_VS_INITIAL  = 0x00,  /**< Compare against first sample */
    SENSE_I_9_WOM_VS_PREVIOUS = 0x01,  /**< Compare against previous sample */
} sense_i_9_wom_mode_t;

/**
 * @brief  FIFO operating mode.
 */
typedef enum {
    SENSE_I_9_FIFO_STREAM   = 0x00,  /**< Overwrite oldest data when full */
    SENSE_I_9_FIFO_SNAPSHOT = 0x1F,  /**< Stop accepting writes when full */
} sense_i_9_fifo_mode_t;

/**
 * @brief  Standard FIFO packet (12 bytes: accel + gyro).
 *
 * Populated by tile_sense_i_9_fifo_read_packet() when the FIFO was
 * configured for accel + gyro (the default packet layout).
 */
typedef struct {
    int16_t accel[3];   /**< X, Y, Z accel in raw ADC counts */
    int16_t gyro[3];    /**< X, Y, Z gyro in raw ADC counts */
} sense_i_9_fifo_packet_t;

/* -------------------------------------------------------------- */
/* Public API                                                      */
/* -------------------------------------------------------------- */

/**
 * @brief  Check whether a Sense.I.9 is present on the I2C bus.
 *
 * Performs an address-level probe only (no register reads).
 *
 * @param  hal       Platform HAL handle
 * @param  instance  Instance index (0 = default, see mapping table)
 * @return 1 if device ACKs, 0 otherwise
 */
uint8_t tile_sense_i_9_find(tiles_pal_t* hal, uint8_t instance);

/**
 * Optional init config. Pass NULL for defaults.
 * Reserved for future use (e.g., initial range/ODR settings).
 */
typedef struct {
    uint8_t reserved;   /**< Placeholder — no options yet. */
} sense_i_9_cfg_t;

/**
 * @brief  Initialize the ICM-20948 and AK09916.
 *
 * Performs a soft reset, verifies WHO_AM_I, wakes the device,
 * enables all accel + gyro axes, enables I2C bypass for direct
 * magnetometer access, and starts the magnetometer in continuous
 * 100 Hz mode. Pass cfg=NULL for defaults.
 *
 * @param  hal       Platform HAL handle
 * @param  instance  Instance index (0 = default, see mapping table)
 * @param  tile      Pointer to tile handle (populated by this function)
 * @param  cfg       Optional config, or NULL for defaults
 *
 * @note   Blocks for ~50 ms during reset. Call once at startup.
 */
void tile_sense_i_9_init(tiles_pal_t* hal, uint8_t instance, tile_t* tile,
                         const sense_i_9_cfg_t *cfg);

/**
 * @brief  Check if new IMU data is available.
 * @studio expose category=tile name=data_ready returns=bool section=runtime
 *
 * Reads the ICM-20948 interrupt status register.
 *
 * @param  tile  Pointer to tile handle
 * @return 1 if new data is available, 0 otherwise
 */
uint8_t tile_sense_i_9_data_ready(tile_t* tile);

/**
 * @brief  Set the accelerometer full-scale range.
 * @studio expose category=tile name=set_accel_range section=runtime
 *
 * @param  tile   Pointer to tile handle
 * @param  range  One of the sense_i_9_accel_range_t values
 */
void tile_sense_i_9_set_accel_range(tile_t* tile, sense_i_9_accel_range_t range);

/**
 * @brief  Set the gyroscope full-scale range.
 * @studio expose category=tile name=set_gyro_range section=runtime
 *
 * @param  tile   Pointer to tile handle
 * @param  range  One of the sense_i_9_gyro_range_t values
 */
void tile_sense_i_9_set_gyro_range(tile_t* tile, sense_i_9_gyro_range_t range);

/**
 * @brief  Set the magnetometer operating mode.
 * @studio expose category=tile name=set_mag_mode section=runtime
 *
 * @param  tile  Pointer to tile handle
 * @param  mode  One of the sense_i_9_mag_mode_t values
 *
 * @note   Switching modes resets the AK09916 measurement cycle.
 */
void tile_sense_i_9_set_mag_mode(tile_t* tile, sense_i_9_mag_mode_t mode);

/**
 * @brief  Set the accelerometer output data rate.
 * @studio expose category=tile name=set_accel_odr section=runtime
 *
 * ODR = 1125 / (1 + divider) Hz.  Examples:
 *   divider = 0   → 1125 Hz
 *   divider = 4   →  225 Hz
 *   divider = 10  → ~102 Hz
 *   divider = 44  →   25 Hz
 *
 * @param  tile     Pointer to tile handle
 * @param  divider  [0..4095] 12-bit ACCEL_SMPLRT_DIV (DS-000189 §10.11)
 */
void tile_sense_i_9_set_accel_odr(tile_t* tile, uint16_t divider);

/**
 * @brief  Set the gyroscope output data rate.
 * @studio expose category=tile name=set_gyro_odr section=runtime
 *
 * ODR = 1125 / (1 + divider) Hz (DS-000189 Table 16; divider only applies
 * while the DLPF is on, which init and set_gyro_range leave enabled).
 *
 * @param  tile     Pointer to tile handle
 * @param  divider  [0..255] 8-bit GYRO_SMPLRT_DIV
 */
void tile_sense_i_9_set_gyro_odr(tile_t* tile, uint8_t divider);

/**
 * @brief  Read raw accelerometer data (3-axis).
 * @studio expose category=tile name=get_raw_accels returns=int[3] section=runtime
 * @studio out_buffer buffer type=int16_t length=3
 *
 * Returns signed 16-bit ADC counts. Convert to milli-g using the
 * sensitivity for the configured range (e.g. ±2 G → 1 LSB ≈ 0.061 mg).
 *
 * @param  tile    Pointer to tile handle
 * @param  buffer  Output array, minimum 3 × int16_t [X, Y, Z]
 */
void tile_sense_i_9_get_raw_accels(tile_t* tile, int16_t* buffer);

/**
 * @brief  Read raw gyroscope data (3-axis).
 * @studio expose category=tile name=get_raw_gyros returns=int[3] section=runtime
 * @studio out_buffer buffer type=int16_t length=3
 *
 * Returns signed 16-bit ADC counts. Convert to °/s using the
 * sensitivity for the configured range (e.g. ±250 DPS → 131 LSB/°/s).
 *
 * @param  tile    Pointer to tile handle
 * @param  buffer  Output array, minimum 3 × int16_t [X, Y, Z]
 */
void tile_sense_i_9_get_raw_gyros(tile_t* tile, int16_t* buffer);

/**
 * @brief  Read raw accelerometer + gyroscope data in a single burst.
 * @studio expose category=tile name=get_raw_6dof returns=int[6] section=runtime
 * @studio out_buffer buffer type=int16_t length=6
 *
 * More efficient than calling get_raw_accels + get_raw_gyros separately
 * (one I2C transaction instead of two). Data is time-coherent.
 *
 * @param  tile    Pointer to tile handle
 * @param  buffer  Output array, minimum 6 × int16_t [AX, AY, AZ, GX, GY, GZ]
 */
void tile_sense_i_9_get_raw_6dof(tile_t* tile, int16_t* buffer);

/**
 * @brief  Read raw magnetometer data (3-axis).
 *
 * Returns signed 16-bit ADC counts from the AK09916.
 * Sensitivity: 0.15 µT/LSB on all axes (factory-trimmed; the AK09916
 * has no per-axis ASA / FUSE ROM correction — see datasheet §11).
 *
 * @studio expose category=tile name=get_raw_mags returns=int[3] section=runtime
 * @studio out_buffer buffer type=int16_t length=3
 * @param  tile    Pointer to tile handle
 * @param  buffer  Output array, minimum 3 × int16_t [X, Y, Z]
 *
 * @note   Automatically reads the ST2 register to release the
 *         magnetometer data lock, enabling the next measurement.
 */
void tile_sense_i_9_get_raw_mags(tile_t* tile, int16_t* buffer);

/**
 * @brief  Check whether the most recent magnetometer reading overflowed.
 * @studio expose category=tile name=mag_overflowed returns=bool section=runtime
 *
 * The AK09916 raises HOFL in ST2 when the sum |HX|+|HY|+|HZ| exceeds
 * 4912 µT (the chip's measurement range). Readings during overflow
 * appear valid in the data registers but are not — for compass-grade
 * work, discard samples where this returns 1.
 *
 * Reading this releases the AK09916 data-lock the same way
 * tile_sense_i_9_get_raw_mags() does, so it can be used standalone
 * after a non-locking peek.
 *
 * @param  tile  Pointer to tile handle
 * @return 1 if HOFL was set after the most recent measurement, 0 otherwise
 */
uint8_t tile_sense_i_9_mag_overflowed(tile_t* tile);

/**
 * @brief  Read the on-chip temperature sensor.
 * @studio expose category=tile name=get_temperature returns=int section=runtime
 *
 * Convert raw value to °C:  temp_degC = (raw / 333.87) + 21.0
 *
 * @param  tile  Pointer to tile handle
 * @return Raw signed 16-bit temperature value
 */
int16_t tile_sense_i_9_get_temperature(tile_t* tile);

/**
 * @brief  Enter low-power sleep mode.
 * @studio expose category=tile name=sleep section=lifecycle
 *
 * Stops all sensor sampling. Current draw drops to ~8 µA.
 * Call tile_sense_i_9_wake() to resume.
 *
 * @param  tile  Pointer to tile handle
 */
void tile_sense_i_9_sleep(tile_t* tile);

/**
 * @brief  Wake from sleep mode and resume sampling.
 * @studio expose category=tile name=wake section=lifecycle
 *
 * Restores auto clock selection. Previously configured ranges
 * and ODRs are preserved across sleep/wake cycles.
 *
 * @param  tile  Pointer to tile handle
 */
void tile_sense_i_9_wake(tile_t* tile);

/**
 * @brief  Perform a software reset.
 *
 * Resets all registers to defaults. Blocks for ~50 ms.
 * You must call tile_sense_i_9_init() again after reset.
 *
 * @studio expose category=tile name=reset section=lifecycle
 * @param  tile  Pointer to tile handle
 */
void tile_sense_i_9_reset(tile_t* tile);

/* ================================================================
 * Interrupt source configuration (INT_PIN_CFG + INT_ENABLE_x)
 * ================================================================ */

/**
 * @brief  Configure the INT pin electrical behaviour.
 *
 * @studio expose category=tile name=int_config section=interrupts
 *
 * Sets polarity (active low/high), drive (push-pull/open-drain),
 * mode (pulsed/latched), and the auto-clear-on-any-read bit.
 * OR together flags from sense_i_9_int_flags_t.
 *
 * @note  INT_PIN_CFG bit 1 (BYPASS_EN) is preserved by this call —
 *        the driver needs bypass mode to talk to the AK09916.
 *
 * @param  tile   Initialized tile handle
 * @param  flags  OR of SENSE_I_9_INT_* flags
 */
void tile_sense_i_9_int_config(tile_t* tile, uint8_t flags);

/**
 * @brief  Route the data-ready interrupt to the INT pin.
 *
 * @studio expose category=tile name=int_data_ready section=interrupts
 * @param  tile     Initialized tile handle
 * @param  enabled  1 to enable, 0 to disable
 */
void tile_sense_i_9_int_data_ready(tile_t* tile, uint8_t enabled);

/**
 * @brief  Route the wake-on-motion interrupt to the INT pin.
 *
 * @studio expose category=tile name=int_wom section=interrupts
 * @param  tile     Initialized tile handle
 * @param  enabled  1 to enable, 0 to disable
 */
void tile_sense_i_9_int_wom(tile_t* tile, uint8_t enabled);

/**
 * @brief  Route the FIFO overflow interrupt to the INT pin.
 *
 * @studio expose category=tile name=int_fifo_overflow section=interrupts
 * @param  tile     Initialized tile handle
 * @param  enabled  1 to enable for any sensor, 0 to disable all
 */
void tile_sense_i_9_int_fifo_overflow(tile_t* tile, uint8_t enabled);

/**
 * @brief  Route the FIFO watermark interrupt to the INT pin.
 *
 * @studio expose category=tile name=int_fifo_watermark section=interrupts
 * @param  tile     Initialized tile handle
 * @param  enabled  1 to enable for any sensor, 0 to disable all
 */
void tile_sense_i_9_int_fifo_watermark(tile_t* tile, uint8_t enabled);

/**
 * @brief  Read INT_STATUS (WoM / DMP / I2C-master / PLL-ready) and clear it.
 *
 * @studio expose category=tile name=get_int_status returns=int section=interrupts
 * @return 8-bit raw register value (use ICM20948_INT_WOM etc. masks)
 */
uint8_t tile_sense_i_9_get_int_status(tile_t* tile);

/**
 * @brief  Read INT_STATUS_2 (FIFO overflow per sensor) and clear it.
 *
 * @studio expose category=tile name=get_int_status_fifo_ovf returns=int section=interrupts
 * @return 5-bit FIFO_OVERFLOW_INT[4:0]; non-zero means overflow occurred
 */
uint8_t tile_sense_i_9_get_int_status_fifo_overflow(tile_t* tile);

/**
 * @brief  Read INT_STATUS_3 (FIFO watermark per sensor) and clear it.
 *
 * @studio expose category=tile name=get_int_status_fifo_wm returns=int section=interrupts
 * @return 5-bit FIFO_WM_INT[4:0]; non-zero means watermark crossed
 */
uint8_t tile_sense_i_9_get_int_status_fifo_watermark(tile_t* tile);

/* ================================================================
 * Wake-on-Motion
 * ================================================================ */

/**
 * @brief  Configure Wake-on-Motion threshold and compare mode.
 *
 * @studio expose category=tile name=wom_config section=wom
 *
 * The chip-wide threshold is one 8-bit value at 4 mg/LSB (range
 * 0–1020 mg). Compared independently against |X|, |Y|, |Z|; any
 * axis crossing the threshold raises WOM_INT.
 *
 * @note  Requires accel running. Mag and gyro can be off.
 *
 * @param  tile     Initialized tile handle
 * @param  thr_mg   Threshold in mg (clamped to 1020 mg / 0xFF LSB)
 * @param  mode     Compare against initial sample or previous sample
 */
void tile_sense_i_9_wom_config(tile_t* tile, uint16_t thr_mg,
                               sense_i_9_wom_mode_t mode);

/**
 * @brief  Enable Wake-on-Motion logic.
 *
 * @studio expose category=tile name=wom_enable section=wom
 *
 * Call after wom_config(). Routes to the INT pin only if int_wom()
 * was also enabled.
 *
 * @param  tile  Initialized tile handle
 */
void tile_sense_i_9_wom_enable(tile_t* tile);

/**
 * @brief  Disable Wake-on-Motion logic.
 *
 * @studio expose category=tile name=wom_disable section=wom
 * @param  tile  Initialized tile handle
 */
void tile_sense_i_9_wom_disable(tile_t* tile);

/* ================================================================
 * FIFO
 * ================================================================ */

/**
 * @brief  Configure which sensor streams write into the FIFO.
 *
 * @studio expose category=tile name=fifo_config section=fifo
 *
 * Enables FIFO operation in USER_CTRL and selects the data sources.
 * Always sets FIFO_MODE = stream when enabling. Disables FIFO entirely
 * if accel, gyro and temp are all 0.
 *
 * @param  tile   Initialized tile handle
 * @param  mode   FIFO operating mode (stream or snapshot)
 * @param  accel  1 to write all 3 accel axes
 * @param  gyro   1 to write all 3 gyro axes
 * @param  temp   1 to write the temperature sample
 */
void tile_sense_i_9_fifo_config(tile_t* tile, sense_i_9_fifo_mode_t mode,
                                uint8_t accel, uint8_t gyro, uint8_t temp);

/**
 * @brief  Reset the FIFO contents (clears all queued samples).
 *
 * @studio expose category=tile name=fifo_flush section=fifo
 * @param  tile  Initialized tile handle
 */
void tile_sense_i_9_fifo_flush(tile_t* tile);

/**
 * @brief  Read the current FIFO byte count.
 *
 * @studio expose category=tile name=fifo_count returns=int section=fifo
 *
 * Note this is bytes, not packets. A standard accel+gyro packet is
 * 12 bytes. Read FIFO_COUNTL first to latch both bytes (handled
 * internally).
 *
 * @param  tile  Initialized tile handle
 * @return Bytes available in the FIFO (0–512)
 */
uint16_t tile_sense_i_9_fifo_count(tile_t* tile);

/**
 * @brief  Read one accel + gyro packet from the FIFO.
 *
 * @param  tile  Initialized tile handle
 * @param  pkt   Output packet (populated only if return is 1)
 * @return 1 if a 12-byte packet was read, 0 if FIFO has fewer bytes
 */
uint8_t tile_sense_i_9_fifo_read_packet(tile_t* tile,
                                        sense_i_9_fifo_packet_t* pkt);

/**
 * @brief  DSL-friendly flat-output variant of fifo_read_packet().
 *
 * Drops the struct in favor of a positional int[6] array — the DSL
 * doesn't have a struct ABI yet, so the per-field outputs come back
 * indexed. Layout:
 *   out[0..2] = accel x,y,z   (raw int16 widened to int32)
 *   out[3..5] = gyro x,y,z    (raw int16 widened to int32)
 *
 * Drops the success bool — call fifo_count() first to know whether
 * there's data. On an empty FIFO, the slots are left untouched.
 *
 * @studio expose category=tile name=fifo_read_packet returns=int[6] section=fifo
 * @studio out_buffer out type=int32_t length=6
 *
 * @param  tile  Initialized tile handle.
 * @param  out   Output buffer (6 int32_t slots).
 */
void tile_sense_i_9_fifo_read_packet_flat(tile_t* tile, int32_t* out);

/* ================================================================
 * Self-test
 * ================================================================ */

/**
 * @brief  Run the built-in mechanical self-test for accel and gyro.
 *
 * @studio expose category=tile name=self_test returns=bool section=advanced
 * @studio out_scalar accel_pass type=uint8_t
 * @studio out_scalar gyro_pass type=uint8_t
 *
 * Drives the accel and gyro through their factory-stored
 * self-excitation routine and compares the response to the
 * stored reference values in SELF_TEST_*_GYRO/ACCEL (Bank 1).
 * Per TDK app-note AN-000150 the pass criterion is that each
 * axis's self-test response is within 50%–150% of the factory
 * reference.
 *
 * Blocks for ~250 ms. The chip is left in a fresh-init state on
 * exit; ranges/ODRs you set before this call may need to be
 * reapplied.
 *
 * @param  tile        Initialized tile handle
 * @param  accel_pass  Output: bit 0/1/2 = X/Y/Z accel pass (1 = pass)
 * @param  gyro_pass   Output: bit 0/1/2 = X/Y/Z gyro pass (1 = pass)
 * @return 1 if every accel axis and every gyro axis passed, 0 otherwise
 */
uint8_t tile_sense_i_9_self_test(tile_t* tile,
                                 uint8_t* accel_pass, uint8_t* gyro_pass);

/**
 * @brief  Run the AK09916 self-test (mag).
 *
 * @studio expose category=tile name=mag_self_test returns=bool section=advanced
 *
 * Triggers the AK09916's internal magnetic-source self-excitation.
 * The pass criterion is the per-axis range table from the AK09916
 * datasheet rev 015007392-E-02 §9.4.4.2:
 *   −200 ≤ HX ≤ 200,  −200 ≤ HY ≤ 200,  −1000 ≤ HZ ≤ −200
 *
 * Blocks for ~10 ms. Leaves the AK09916 in power-down — call
 * tile_sense_i_9_set_mag_mode() afterwards to resume measurements.
 *
 * @param  tile  Initialized tile handle
 * @return 1 if all three axes were within spec, 0 otherwise
 */
uint8_t tile_sense_i_9_mag_self_test(tile_t* tile);

/* ============================================================== */
/* Runtime — tier-2 idiomatic helpers                              */
/*                                                                  */
/* These compose the tier-1 surface above into "do the thing the   */
/* user wants to do" calls. They handle range/scale conversion and */
/* coordinate-frame interpretation so callers don't need to read   */
/* the ICM-20948 datasheet to detect a flip or read a heading.     */
/*                                                                  */
/* All conversions are integer-only so they run cheaply on the     */
/* Cores without pulling in soft-float code.                       */
/* ============================================================== */

/**
 * @brief  Check whether the tile is lying flat, face up.
 *
 * @studio expose category=tile name=is_face_up returns=bool section=runtime
 *
 * Reads the accelerometer and returns 1 when the Z-axis is close
 * to +1 g (i.e. gravity is pulling down through the back of the
 * tile). The acceptance band is roughly ±30° of true face-up; the
 * X/Y axes must each read below ~0.5 g for the test to pass, so
 * tilted-but-mostly-up orientations also count as face-up.
 *
 * @note  Assumes the accel range is the default ±2 g configured by
 *        @ref tile_sense_i_9_init. The thresholds are raw counts fixed
 *        for ±2 g (1 g = 16384 LSB), so after set_accel_range to ±4 g
 *        or wider this never returns 1.
 *
 * @param  tile  Initialized tile handle
 * @return 1 if face-up, 0 otherwise
 */
uint8_t tile_sense_i_9_is_face_up(tile_t* tile);

/**
 * @brief  Check whether the tile is lying flat, face down.
 *
 * @studio expose category=tile name=is_face_down returns=bool section=runtime
 *
 * Mirror of @ref tile_sense_i_9_is_face_up — Z-axis close to −1 g.
 *
 * @param  tile  Initialized tile handle
 * @return 1 if face-down, 0 otherwise
 */
uint8_t tile_sense_i_9_is_face_down(tile_t* tile);

/**
 * @brief  Check whether the tile is currently moving.
 *
 * @studio expose category=tile name=is_moving returns=bool section=runtime
 *
 * Reads the accelerometer and reports whether |‖a‖ − 1 g| exceeds
 * `threshold_mg`. At rest in any orientation the magnitude is ≈ 1 g,
 * so this captures linear acceleration regardless of which face is
 * up. Use ~50–100 mg for "is being handled" and ~300–500 mg for
 * "is being shaken".
 *
 * Implementation uses an integer approximation of the magnitude
 * comparison: it checks the squared deviation against the squared
 * threshold to avoid a square root.
 *
 * @note  Assumes ±2 g range (init default): 1 g is taken as 16384 LSB.
 *        After set_accel_range the result is wrong (at ±4 g a resting
 *        tile reads as 0.5 g, i.e. "moving" for thresholds < 500 mg).
 *
 * @param  tile          Initialized tile handle
 * @param  threshold_mg  Deviation from 1 g, in milli-g
 * @return 1 if moving, 0 if at rest
 */
uint8_t tile_sense_i_9_is_moving(tile_t* tile, uint16_t threshold_mg);

/**
 * @brief  Read the tilt of one accel axis vs gravity, in centi-degrees.
 *
 * Returns the angle (in 0.01° units) between the requested axis and
 * the measured +1 g reaction vector (the "up" direction when at rest).
 * Range is 0..18000 centi-degrees (0.00°..180.00°); the value is never
 * negative. Computed from `atan2(|other_components|, axis)` using an
 * integer approximation; worst-case error about 0.15° in the
 * noise-free case.
 *
 * @note  Semantics differ from Sense.I.6P6's read_tilt_centi_degrees,
 *        which returns the axis's elevation above horizontal (−90..+90°).
 *
 * Axis selector:
 *   0 = X axis (pitch around Y, with the chip lying on its back)
 *   1 = Y axis (roll around X)
 *   2 = Z axis (deviation from horizontal — 0 = face-up, 180 = face-down)
 *
 * @param  tile           Initialized tile handle
 * @param  axis           0/1/2 selecting X, Y, or Z
 * @param  out_centi_deg  Output tilt in 0.01° units (0..18000)
 */
void tile_sense_i_9_read_tilt_centi_degrees(tile_t* tile, uint8_t axis,
                                            int16_t* out_centi_deg);

/**
 * @brief  Angle between one accel axis and "up", in 0.01° (int32 out).
 *
 * Flat-output variant of read_tilt_centi_degrees(): same computation; the angle is written through an int32_t so Studio's
 * out-scalar locals (32-bit) receive it without a width mismatch.
 *
 * @studio expose category=tile name=read_tilt_centi_degrees section=runtime
 * @studio out_scalar out_centi_deg type=int32_t
 * @param  tile           Initialized tile handle.
 * @param  axis           [0..2] 0 = X, 1 = Y, 2 = Z.
 * @param  out_centi_deg  Output: angle between the axis and "up", 0.01° (0..18000).
 */
void tile_sense_i_9_read_tilt_centi_degrees_flat(tile_t* tile, uint8_t axis,
                                                 int32_t* out_centi_deg);

/**
 * @brief  Read a compass heading from the magnetometer, in centi-degrees.
 *
 * Returns the bearing of the +X axis relative to magnetic north in
 * 0.01° units, range 0..35999 (0.00°..359.99°). 0° means +X is
 * pointing along magnetic north; angle increases clockwise viewed
 * from above. Computed as `atan2(−mag_y, mag_x)` from the AK09916
 * raw counts (no per-axis trim is needed — the AK09916 has no FUSE
 * ROM and is factory-trimmed at 0.15 µT/LSB).
 *
 * @warning  This is a tilt-uncompensated heading. It is only
 *           accurate when the tile is held roughly level (within a
 *           few degrees of horizontal). When tilted, the heading
 *           drifts because the horizontal projection of the
 *           magnetic field changes with orientation. A
 *           tilt-compensated heading needs both accel data (to
 *           recover the gravity-aligned frame) and a 3D rotation —
 *           that is left to a later tier-3 helper or the Studio
 *           DSL.
 *
 * @warning  Local magnetic distortions (steel, motors, speakers)
 *           skew the result. For best accuracy, calibrate the
 *           sensor in its final enclosure before relying on the
 *           heading.
 *
 * @warning  The AK09916 axes are not the accel/gyro axes (DS-000189
 *           Figure 13); the driver applies no remap yet. Bench-unverified.
 *
 * @param  tile           Initialized tile handle
 * @param  out_centi_deg  Output heading in 0.01° units (0..35999)
 */
void tile_sense_i_9_read_heading_centi_degrees(tile_t* tile,
                                               uint16_t* out_centi_deg);

/**
 * @brief  Compass heading of +X from magnetic north, in 0.01° (int32 out).
 *
 * Flat-output variant of read_heading_centi_degrees(): same computation; the heading is written through an int32_t so Studio's
 * out-scalar locals (32-bit) receive it without a width mismatch.
 *
 * @studio expose category=tile name=read_heading_centi_degrees section=runtime
 * @studio out_scalar out_centi_deg type=int32_t
 * @param  tile           Initialized tile handle.
 * @param  out_centi_deg  Output: heading of +X from magnetic north, 0.01° (0..35999).
 */
void tile_sense_i_9_read_heading_centi_degrees_flat(tile_t* tile,
                                                    int32_t* out_centi_deg);

/**
 * @brief  Block until a Wake-on-Motion event fires, or timeout.
 *
 * @studio expose category=tile name=wait_for_motion returns=bool section=runtime
 *
 * Polls @ref tile_sense_i_9_get_int_status every ~5 ms and returns
 * 1 as soon as the WoM bit is observed. Returns 0 if `timeout_ms`
 * elapses without an event. The poll cadence is chosen as a
 * compromise: low enough to keep wake latency under ~5 ms but slow
 * enough that the I2C bus isn't saturated by status reads.
 *
 * @note  Wake-on-Motion must be configured (@ref
 *        tile_sense_i_9_wom_config) and enabled (@ref
 *        tile_sense_i_9_wom_enable) before this call. INT-pin
 *        routing (@ref tile_sense_i_9_int_wom) is optional — this
 *        function reads the status register directly.
 *
 * @note  This call blocks. For non-blocking use, drive @ref
 *        tile_sense_i_9_get_int_status from your own loop.
 *
 * @param  tile        Initialized tile handle
 * @param  timeout_ms  Maximum time to wait, in milliseconds
 * @return 1 if motion was detected, 0 on timeout
 */
uint8_t tile_sense_i_9_wait_for_motion(tile_t* tile, uint32_t timeout_ms);

/* ================================================================
 * DMP3 — on-chip Digital Motion Processor (Phase 1: firmware load)
 *
 * The ICM-20948 has a coprocessor (DMP3) that can run sensor-fusion
 * firmware on-chip. Without firmware loaded, the DMP is dormant and
 * the chip behaves as a plain raw-sample sensor. With firmware loaded
 * and configured, the DMP outputs derived quantities (quaternions,
 * linear accel, calibrated mag) into a packet stream the host reads
 * from FIFO.
 *
 * Firmware blob: 14301 bytes, vendored from TDK eMD via SparkFun's
 * ICM-20948 Arduino library. See tile_sense_i_9_dmp3.h.
 *
 * This phase ships the firmware-load + verify path only. Reading
 * DMP outputs (quaternions etc.) lands in subsequent phases.
 * ================================================================ */

/**
 * @brief  Load the DMP3 firmware into the chip's DMP RAM and verify.
 *
 * Performs the full eMD load sequence: chunked write of the 14301-byte
 * blob to DMP RAM (handling the 0x100-byte bank-page crossings), then
 * a chunked read-back compare. Idempotent — repeated calls re-verify
 * but don't reload.
 *
 * Blocks for ~250 ms on a 400 kHz I²C bus (chunk size = 16 bytes,
 * one bank-select write per crossing).
 *
 * The DMP is left in the loaded-but-disabled state. Enabling output
 * (DMP_EN in USER_CTRL + feature configuration) is a separate step,
 * shipped in a follow-up phase.
 *
 * @param  tile  Initialized tile handle.
 * @return 1 on successful load + verify, 0 on bus error or verify mismatch.
 */
uint8_t tile_sense_i_9_dmp_load(tile_t* tile);

/**
 * @brief  Check whether DMP firmware has been loaded since the last reset.
 *
 * Tracks driver-side state set by dmp_load(). A chip reset clears
 * the DMP RAM but doesn't auto-clear this flag — call after reset()
 * to know whether you need to reload.
 *
 * @param  tile  Initialized tile handle.
 * @return 1 if dmp_load() has succeeded since the driver last cleared its state.
 */
uint8_t tile_sense_i_9_dmp_is_loaded(tile_t* tile);

/* ================================================================
 * DMP3 — Phase 2: 9-axis quaternion (Android ROTATION_VECTOR)
 *
 * The DMP fuses accel + gyro + mag into a unit quaternion (q0,q1,q2,q3)
 * representing orientation in an absolute (north-aligned) frame. Only
 * q1, q2, q3 are emitted — q0 = sqrt(1 − q1² − q2² − q3²) and is
 * recovered host-side. Components are Q30 fixed-point.
 *
 * Mag-access mode rule (important):
 *
 *   - At init, the driver puts the AK09916 directly on the host I2C
 *     bus via INT_PIN_CFG.BYPASS_EN. tile_sense_i_9_get_raw_mags(),
 *     tile_sense_i_9_set_mag_mode(), tile_sense_i_9_mag_overflowed(),
 *     and tile_sense_i_9_mag_self_test() all rely on this and "just
 *     work".
 *
 *   - tile_sense_i_9_dmp_start_quat9() switches the chip into I2C-master
 *     mode (BYPASS_EN cleared, USER_CTRL.I2C_MST_EN set) so the DMP can
 *     pull mag samples internally. While DMP is running, the host has
 *     no direct path to the AK09916 — those four mag calls return
 *     stale / zero data and should not be used.
 *
 *   - tile_sense_i_9_dmp_stop() reverts to bypass mode and re-enables
 *     direct mag access. After dmp_stop() it is safe to call mag funcs
 *     again, though set_mag_mode() must be re-issued (the DMP path
 *     leaves the AK09916 in single-measurement mode).
 *
 * Phase 4 will add yaw/pitch/roll. For now, host-side conversion to
 * Euler angles or float is the caller's job.
 * ================================================================ */

/**
 * @brief  Configure the DMP for 9-axis quaternion output and start it.
 *
 * Wires the chip up to compute a fused (accel+gyro+mag) orientation
 * quaternion at the requested period and stream it to the FIFO.
 * Equivalent to InvenSense's "ROTATION_VECTOR" Android sensor.
 *
 * Requires dmp_load() to have succeeded since the last reset.
 *
 * Side effects:
 *   - Switches AK09916 access from bypass to I2C-master mode (see the
 *     mag-access-mode rule comment block above this function).
 *   - Reconfigures accel + gyro to 56.25 Hz ODR (DMP-recommended) and
 *     ±4 g / ±2000 dps full-scale (also DMP-recommended).
 *   - Resets and enables the FIFO; old packets are discarded.
 *
 * @studio expose category=tile name=dmp_start_quat9 returns=bool section=advanced
 * @param  tile              Initialized tile handle (DMP firmware loaded).
 * @param  output_period_ms  DMP output period, in ms (clamped to 10–1000 ms).
 *                           Internally maps to a divider against the DMP's
 *                           ~225 Hz base ODR.
 * @return 1 on success, 0 if firmware isn't loaded or a bus error occurred.
 */
uint8_t tile_sense_i_9_dmp_start_quat9(tile_t* tile, uint16_t output_period_ms);

/**
 * @brief  Stop the DMP and revert to direct-bus mag access.
 *
 * Clears DMP_EN, disables the I2C master, drops the data-output feature
 * masks, and re-enables BYPASS_EN. After this call the four mag helpers
 * (get_raw_mags / set_mag_mode / mag_overflowed / mag_self_test) work
 * again — but you'll need to re-issue set_mag_mode() to put the AK09916
 * back into a continuous mode (DMP leaves it in single-measurement).
 *
 * Idempotent — safe to call when the DMP is already stopped.
 *
 * @studio expose category=tile name=dmp_stop section=advanced
 * @param  tile  Initialized tile handle.
 */
void tile_sense_i_9_dmp_stop(tile_t* tile);

/**
 * @brief  Non-blocking check for a queued DMP quat9 packet.
 *
 * Reads the FIFO byte-count register and returns 1 when at least one
 * full Quat9 packet (header + payload + accuracy = 16 bytes) is
 * available. Cheap (a single 2-byte register read).
 *
 * @studio expose category=tile name=dmp_data_ready returns=bool section=advanced
 * @param  tile  Initialized tile handle.
 * @return 1 if a packet can be read, 0 otherwise.
 */
uint8_t tile_sense_i_9_dmp_data_ready(tile_t* tile);

/**
 * @brief  Read one 9-axis quaternion packet from the DMP FIFO.
 *
 * Pulls a header + 14-byte Quat9 payload from the FIFO and decodes:
 *   out_q[0] = q0 (scalar / w) — recovered as sqrt(1 − q1² − q2² − q3²)
 *   out_q[1] = q1 (vector / x)
 *   out_q[2] = q2 (vector / y)
 *   out_q[3] = q3 (vector / z)
 * Components are Q30 fixed-point — divide by 2^30 (1073741824) to get
 * a unit-quaternion float. The accuracy field is the DMP's heading
 * accuracy estimate (chip-internal scale; treat smaller = better).
 *
 * On a header mismatch (FIFO content isn't a Quat9 packet) the FIFO
 * is reset and the function returns 0 — this can happen briefly
 * after dmp_start_quat9() while the pipeline drains its first sample.
 *
 * @studio expose category=tile name=dmp_read_quat9 returns=int[4] section=advanced
 * @studio out_buffer out_q type=int32_t length=4
 * @studio out_scalar out_accuracy type=uint16_t
 *
 * @param  tile          Initialized tile handle.
 * @param  out_q         Output array, 4 × int32_t [q0, q1, q2, q3] in Q30.
 * @param  out_accuracy  Output: DMP heading-accuracy estimate.
 * @return 1 if a packet was decoded, 0 if FIFO didn't have one ready.
 */
uint8_t tile_sense_i_9_dmp_read_quat9(tile_t* tile, int32_t out_q[4],
                                      uint16_t* out_accuracy);

#endif /* INC_TILE_SENSE_I_9_H_ */
