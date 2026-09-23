/**
 * @file  tile_sense_tof.h
 * @brief TMF8806 time-of-flight distance sensor.
 *
 * Platform-agnostic driver for the AMS/Sciosense TMF8806 direct time-of-flight
 * (dToF) sensor with integrated VCSEL emitter and SPAD detector array.
 *
 * Key specifications:
 *  - Distance measurement up to 5000 mm (short-range, 2.5 m, and 5 m modes)
 *  - 16-bit distance output in millimeters
 *  - Configurable measurement period (30 ms to 2000 ms, or single-shot)
 *  - Configurable iteration count for accuracy vs. speed trade-off
 *  - 6-bit reliability indicator (0 = no object, 63 = highest confidence)
 *  - On-chip factory calibration with host-side storage and reload
 *  - Algorithm state save/restore for ultra-low-power resume
 *  - Embedded 8-bit temperature sensor
 *
 * Quick start:
 * @code
 *   #include "core.h"
 *   #include "core_tiles.h"
 *   #include "tile_sense_tof.h"
 *
 *   tile_t tof;
 *   sense_tof_cfg_t cfg = { .mode = SENSE_TOF_RANGE_2500MM };
 *   tile_sense_tof_init(core_tiles_pal(&core_i2c1), 0, &tof, &cfg);
 *
 *   tile_sense_tof_start(&tof);
 *   // ... poll or wait for interrupt ...
 *   uint16_t dist = tile_sense_tof_get_distance_mm(&tof);
 *
 *   // Single-shot convenience:
 *   sense_tof_result_t res;
 *   tile_sense_tof_measure_single(&tof, &res, 500);
 * @endcode
 *
 * Datasheet: https://ams.com/tmf8806
 *
 * @studio tile label=Sense.TOF icon=◊
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=common category="10 m extended-range mode" section=advanced
 *   Deferred to a dedicated session. TMF8806 App0 firmware supports
 *   up to 5 m range out of ROM; 10 m mode requires downloading a
 *   binary RAM patch from AMS via the bootloader's W_RAM +
 *   RAMREMAP_RESET protocol — non-trivial firmware-loading flow not
 *   in scope for this driver-coverage pass.
 *
 * @studio unsupported severity=niche category="GPIO0 / GPIO1 runtime control"
 *   GPIO0 is routed to tile pad 3 but is consumed as a hardware
 *   strap: a 100 k on-board pull-up to V+ holds GPIO0 high at
 *   startup, selecting the 1.8-3.3 V digital-I/O level required by
 *   this tile's 2.7-3.6 V rail (TMF8806 datasheet §6.7). It is NOT
 *   an I2C-address strap (the address is fixed at 0x41, changed only
 *   via command 0x49). Because the strap fixes GPIO0 high, runtime
 *   GPIO0 output modes (object-detect / VCSEL-sync, command 0x0F)
 *   are not exposed — driving the pad would fight the pull-up and
 *   risk changing the I/O level. GPIO1 is not routed to a pad.
 *
 * @note All bus I/O is routed through tiles_pal_t function pointers.
 *       This driver contains no platform-specific code.
 */

#ifndef TILE_SENSE_TOF_H
#define TILE_SENSE_TOF_H

#include "tiles.h"

/* ---- Driver version ---- */

#define TILE_SENSE_TOF_VERSION_MAJOR  1
#define TILE_SENSE_TOF_VERSION_MINOR  5
#define TILE_SENSE_TOF_VERSION_PATCH  0

TILES_CHECK_VERSION(1, 0);

/* ---- Instance mapping ----
 *
 *  Instance  |  I2C address  |  Notes
 * -----------|---------------|----------------------------
 *     0      |     0x41      |  Fixed address (only one)
 */

/* ---- I2C addresses ---- */

#define TMF8806_I2C_ADDR            0x41  /**< Fixed 7-bit I2C address */

/* ---- Register map ---- */

#define TMF8806_REG_APPID           0x00  /**< Application ID (0x80=BL, 0xC0=App0) */
#define TMF8806_REG_APPREV_MAJOR    0x01  /**< Application revision major */
#define TMF8806_REG_APPREQID        0x02  /**< Application request ID (write 0xC0) */
#define TMF8806_REG_CMD_DATA9       0x06  /**< Command payload byte 9 (first byte) */
#define TMF8806_REG_CMD_DATA8       0x07  /**< Command payload byte 8 */
#define TMF8806_REG_CMD_DATA7       0x08  /**< Command payload byte 7 */
#define TMF8806_REG_CMD_DATA6       0x09  /**< Command payload byte 6 */
#define TMF8806_REG_CMD_DATA5       0x0A  /**< Command payload byte 5 */
#define TMF8806_REG_CMD_DATA4       0x0B  /**< Command payload byte 4 */
#define TMF8806_REG_CMD_DATA3       0x0C  /**< Command payload byte 3 */
#define TMF8806_REG_CMD_DATA2       0x0D  /**< Command payload byte 2 */
#define TMF8806_REG_CMD_DATA1       0x0E  /**< Command payload byte 1 */
#define TMF8806_REG_CMD_DATA0       0x0F  /**< Command payload byte 0 */
#define TMF8806_REG_COMMAND         0x10  /**< Command trigger register */
#define TMF8806_REG_PREVIOUS        0x11  /**< Last executed command */
#define TMF8806_REG_APPREV_MINOR    0x12  /**< Application revision minor */
#define TMF8806_REG_APPREV_PATCH    0x13  /**< Application revision patch */
#define TMF8806_REG_STATE           0x1C  /**< Application state (2 = error) */
#define TMF8806_REG_STATUS          0x1D  /**< Result status byte */
#define TMF8806_REG_REG_CONTENTS    0x1E  /**< Register contents type indicator */
#define TMF8806_REG_TID             0x1F  /**< Transaction ID (per-result) */
#define TMF8806_REG_RESULT_NUMBER   0x20  /**< Result counter */
#define TMF8806_REG_RESULT_INFO     0x21  /**< Measurement status + reliability */
#define TMF8806_REG_DISTANCE_LSB    0x22  /**< Distance peak 0 (low byte) */
#define TMF8806_REG_DISTANCE_MSB    0x23  /**< Distance peak 0 (high byte) */
#define TMF8806_REG_SYS_CLOCK_0     0x24  /**< System clock byte 0 (LSB) */
#define TMF8806_REG_SYS_CLOCK_3     0x27  /**< System clock byte 3 (MSB) */
#define TMF8806_REG_STATE_DATA      0x28  /**< Algorithm state data (11 bytes) */
#define TMF8806_REG_TEMPERATURE     0x33  /**< Die temperature (signed 8-bit C) */
#define TMF8806_REG_REF_HITS_0      0x34  /**< Reference hits byte 0 (LSB) */
#define TMF8806_REG_OBJ_HITS_0      0x38  /**< Object hits byte 0 (LSB) */
#define TMF8806_REG_XTALK_LSB       0x3C  /**< Cross-talk count (low byte) */
#define TMF8806_REG_XTALK_MSB       0x3D  /**< Cross-talk count (high byte) */
#define TMF8806_REG_FACTORY_CALIB   0x20  /**< Factory calibration data (14 bytes) */
#define TMF8806_REG_STATE_DATA_WR   0x2E  /**< Algorithm state write (11 bytes) */
#define TMF8806_REG_ENABLE          0xE0  /**< Power control + CPU status */
#define TMF8806_REG_INT_STATUS      0xE1  /**< Interrupt status (write-1-clear) */
#define TMF8806_REG_INT_ENAB        0xE2  /**< Interrupt enable mask */
#define TMF8806_REG_ID              0xE3  /**< Device ID register */
#define TMF8806_REG_REVID           0xE4  /**< Silicon revision ID */

/* ---- Device ID ---- */

#define TMF8806_DEVICE_ID           0x09  /**< Expected ID value (bits 5:0 only) */
#define TMF8806_ID_MASK             0x3F  /**< Mask for valid ID bits */

/* ---- ENABLE register values ---- */

#define TMF8806_ENABLE_PON          0x01  /**< Power-on bit */
#define TMF8806_ENABLE_CPU_READY    0x41  /**< CPU ready + PON */
#define TMF8806_ENABLE_CPU_RESET    0x80  /**< CPU reset bit */

/* ---- APPID values ---- */

#define TMF8806_APPID_BOOTLOADER    0x80  /**< Bootloader is running */
#define TMF8806_APPID_APP0          0xC0  /**< Measurement application running */

/* ---- Command codes ---- */

#define TMF8806_CMD_MEASURE         0x02  /**< Start measurement */
#define TMF8806_CMD_FACTORY_CAL     0x0A  /**< Run factory calibration */
#define TMF8806_CMD_STOP            0xFF  /**< Stop measurement */
#define TMF8806_CMD_SERIAL          0x47  /**< Read serial number */

/** kIters for factory calibration: 40960 -> ~40.96M iterations (datasheet §6.9). */
#define TMF8806_FACTORY_CAL_KITERS  0xA000

/* ---- INT_STATUS bit masks ---- */

#define TMF8806_INT_RESULT          0x01  /**< Result interrupt flag */
#define TMF8806_INT_HISTOGRAM       0x02  /**< Histogram interrupt flag */

/* ---- REGISTER_CONTENTS values ---- */

#define TMF8806_CONTENTS_RESULT     0x55  /**< Result data available */
#define TMF8806_CONTENTS_CALIB      0x0A  /**< Calibration data available */
#define TMF8806_CONTENTS_SERIAL     0x47  /**< Serial number available */

/* ---- Calibration data ---- */

#define TMF8806_CALIB_DATA_LEN      14    /**< Factory calibration data length */
#define TMF8806_STATE_DATA_LEN      11    /**< Algorithm state data length */

/* ---- Boot / poll timeouts ---- */

#define TMF8806_BOOT_TIMEOUT_MS     500   /**< Maximum boot sequence wait */
#define TMF8806_CMD_TIMEOUT_MS      1000  /**< Maximum command completion wait */
#define TMF8806_POLL_INTERVAL_MS    2     /**< Polling interval during boot */

/* ---- Enumerations ---- */

/**
 * @brief  Distance mode selection.
 *
 * Controls the maximum detection range and power consumption.
 * Configured via algorithm selection bits in the measurement command.
 */
typedef enum {
    SENSE_TOF_SHORT_RANGE  = 0,  /**< Short range, ~200 mm max, lowest power */
    SENSE_TOF_RANGE_2500MM = 1,  /**< Medium range up to 2500 mm (default) */
    SENSE_TOF_RANGE_5000MM = 2,  /**< Long range up to 5000 mm */
} sense_tof_distance_mode_t;

/**
 * @brief  Measurement repetition period (cmd_data2, `repetitionPeriodMs`).
 *
 * 1-253 is the period in milliseconds; 0xFE and 0xFF are the chip's codes
 * for 1 s and 2 s (datasheet cmd 0x02). 0x00 (single shot) is what
 * measure_single() uses internally and is not a rate, so it is not listed.
 * Any other 1-253 value is accepted too. If one ranging takes longer than
 * the period (many iterations), the chip ranges back-to-back and the period
 * is ignored.
 */
typedef enum {
    SENSE_TOF_PERIOD_30MS  = 0x1E,  /**< Every 30 ms, ~33 per second (default) @studio value=33.3 */
    SENSE_TOF_PERIOD_50MS  = 0x32,  /**< Every 50 ms, 20 per second @studio value=20 */
    SENSE_TOF_PERIOD_100MS = 0x64,  /**< Every 100 ms, 10 per second @studio value=10 */
    SENSE_TOF_PERIOD_250MS = 0xFA,  /**< Every 250 ms, 4 per second @studio value=4 */
    SENSE_TOF_PERIOD_1S    = 0xFE,  /**< Every second @studio value=1 */
    SENSE_TOF_PERIOD_2S    = 0xFF,  /**< Every 2 seconds @studio value=0.5 */
} sense_tof_period_t;

/* ---- Configuration struct ---- */

/**
 * @brief  Optional init-time configuration for Sense.TOF.
 *
 * Pass NULL to tile_sense_tof_init() for defaults:
 * mode = 2.5 m, period = 30 ms continuous, kilo_iters = 900, threshold = 6.
 *
 * The configuration takes effect when tile_sense_tof_start() is called.
 * Values can be changed between start/stop cycles.
 */
typedef struct {
    uint8_t  mode;        /**< sense_tof_distance_mode_t (default: RANGE_2500MM). */
    uint8_t  period_ms;   /**< Repetition period code (default: 0x1E = 30 ms). 0x00 = single shot, 0xFE = 1 s, 0xFF = 2 s. */
    uint16_t kilo_iters;  /**< Iterations in thousands (default: 900). Higher values improve SNR at the cost of power. */
    uint8_t  threshold;   /**< Detection threshold, 0-63 (default: 6). Lower values increase sensitivity. */
} sense_tof_cfg_t;

/* ---- Result struct ---- */

/**
 * @brief  Measurement result from a single ToF capture.
 *
 * Populated by tile_sense_tof_get_result() or tile_sense_tof_measure_single().
 */
typedef struct {
    uint16_t distance_mm;   /**< Peak distance in millimeters. 0 if no target. */
    uint8_t  status;        /**< Result status. 0x00-0x0F = valid, 0x10+ = error. */
    uint8_t  reliability;   /**< Confidence indicator, 0-63. 0 = no object, 63 = highest confidence. */
    int8_t   temperature;   /**< Die temperature in degrees Celsius. */
    uint8_t  result_number; /**< Monotonically incrementing result counter. */
} sense_tof_result_t;

/* ---- Version info struct ---- */

/**
 * @brief  Application firmware version reported by the TMF8806.
 */
typedef struct {
    uint8_t major;  /**< Major version number. */
    uint8_t minor;  /**< Minor version number. */
    uint8_t patch;  /**< Patch version number. */
} sense_tof_version_t;

/* ---- Signal-quality struct ---- */

/**
 * @brief  Per-measurement signal-quality diagnostics from the TMF8806.
 *
 * Populated from the result block (registers 0x34-0x3D). Higher hit
 * counts mean a stronger return; crosstalk is the internal optical
 * leakage the factory calibration measures and subtracts.
 */
typedef struct {
    uint32_t reference_hits;  /**< Reference-channel photon hits (regs 0x34-0x37). */
    uint32_t object_hits;     /**< Object-channel photon hits (regs 0x38-0x3B). */
    uint16_t crosstalk;       /**< Cross-talk count (regs 0x3C-0x3D). */
} sense_tof_signal_t;

/* ---- Lifecycle ---- */

/**
 * @brief  Check if a Sense.TOF tile is present on the bus.
 *
 * Probes the I2C address and reads the ID register. The TMF8806 has a
 * single fixed address (0x41), so only instance 0 is valid.
 *
 * @param  hal       Platform HAL handle.
 * @param  instance  Must be 0 (single-address device).
 * @return 1 if device ACKs and ID matches (0x09), 0 otherwise.
 */
uint8_t tile_sense_tof_find(tiles_pal_t *hal, uint8_t instance);

/**
 * @brief  Initialise a Sense.TOF tile.
 *
 * Performs the full TMF8806 boot sequence:
 *  1. Waits for bootloader to enter sleep (ENABLE == 0x00)
 *  2. Wakes the bootloader (PON = 1)
 *  3. Waits for CPU ready (ENABLE == 0x41)
 *  4. Requests App0 measurement application
 *  5. Waits for App0 to start (APPID == 0xC0)
 *  6. Enables result interrupt
 *
 * Does NOT start measurements — call tile_sense_tof_start() after init.
 * Does NOT software-reset to preserve any existing calibration data.
 *
 * @param  hal       Platform HAL handle.
 * @param  instance  Must be 0 (single-address device).
 * @param  tile      Tile handle to initialise.
 * @param  cfg       Configuration (NULL for defaults: 2.5 m, 30 ms, 900k iters).
 */
void tile_sense_tof_init(tiles_pal_t *hal, uint8_t instance,
                         tile_t *tile, const sense_tof_cfg_t *cfg);

/**
 * @brief  Enter standby mode (ENABLE = 0x00).
 * @studio expose category=tile name=sleep section=lifecycle
 *
 * Stops any active measurement and powers down the sensor. Use
 * tile_sense_tof_wake() to resume without full re-initialisation.
 *
 * @param  tile  Initialised tile handle.
 */
void tile_sense_tof_sleep(tile_t *tile);

/**
 * @brief  Resume from standby mode.
 * @studio expose category=tile name=wake section=lifecycle
 *
 * Re-executes the bootloader wake and App0 request sequence. Does not
 * restart measurements — call tile_sense_tof_start() after waking.
 *
 * @param  tile  Sleeping tile handle.
 */
void tile_sense_tof_wake(tile_t *tile);

/**
 * @brief  Reset the device via the CPU reset bit in ENABLE.
 * @studio expose category=tile name=reset section=lifecycle
 *
 * Performs a full CPU reset and re-runs the boot sequence. All runtime
 * state including calibration is lost. Call init() again after reset.
 *
 * @param  tile  Tile handle.
 */
void tile_sense_tof_reset(tile_t *tile);

/* ---- Measurement control ---- */

/**
 * @brief  Start continuous or single-shot measurement.
 * @studio expose category=tile name=start section=runtime
 *
 * Writes the factory calibration data (if loaded), configures the
 * measurement command payload from the current cfg, and issues the
 * measurement command. Results are signaled via the result interrupt.
 *
 * If period_ms == 0x00 in the config, a single measurement is taken.
 * Otherwise measurements repeat at the configured period.
 *
 * @param  tile  Initialised tile handle.
 */
void tile_sense_tof_start(tile_t *tile);

/**
 * @brief  Stop an active measurement.
 * @studio expose category=tile name=stop section=runtime
 *
 * Sends the stop command and waits for the sensor to acknowledge.
 * No-op if no measurement is active.
 *
 * @param  tile  Initialised tile handle.
 */
void tile_sense_tof_stop(tile_t *tile);

/**
 * @brief  Perform a blocking single-shot measurement.
 *
 * Temporarily overrides the repetition period to single-shot mode,
 * starts a measurement, polls for the result interrupt, reads the
 * result, and restores the original period setting.
 *
 * @param  tile        Initialised tile handle.
 * @param  result      Output: measurement result (may be NULL to discard).
 * @param  timeout_ms  Maximum wait time in milliseconds.
 * @return 1 if a valid result was obtained, 0 on timeout or error.
 */
uint8_t tile_sense_tof_measure_single(tile_t *tile, sense_tof_result_t *result,
                                      uint32_t timeout_ms);

/* ---- Result reading ---- */

/**
 * @brief  Read the distance from the most recent result.
 * @studio expose category=tile name=get_distance_mm returns=int section=runtime
 *
 * Returns the peak distance in millimeters. Does not check whether
 * new data is available — call tile_sense_tof_result_ready() first
 * or use tile_sense_tof_get_result() for full status.
 *
 * When nothing is in range the sensor reports 0; this SATURATES that to
 * tile_sense_tof_max_range_mm()
 * rather than returning 0, so `if (distance < threshold)` behaves correctly
 * with no special case — "out of range" reads as "very far away", which is
 * what it physically means. Use tile_sense_tof_get_result() when you need to
 * tell "no target" from "target at max range"; that reports the raw value.
 *
 * @param  tile  Initialised tile handle.
 * @return Distance in millimeters, saturating at the configured max range.
 */
uint16_t tile_sense_tof_get_distance_mm(tile_t *tile);

/**
 * @brief  Maximum measurable distance for the configured range mode.
 * @studio expose category=tile name=max_range_mm returns=int section=runtime
 *
 * 200 mm (short range), 2500 mm, or 5000 mm. This is the value
 * tile_sense_tof_get_distance_mm() saturates to when no object is detected,
 * so `distance >= max_range_mm()` is the explicit "nothing in range" test.
 *
 * @param  tile  Initialised tile handle.
 * @return Maximum range in millimeters.
 */
uint16_t tile_sense_tof_max_range_mm(tile_t *tile);

/**
 * @brief  Read the full measurement result.
 *
 * Reads distance, status, reliability, temperature, and result number
 * from the sensor in a single bus transaction. Clears the result
 * interrupt flag after reading.
 *
 * @param  tile    Initialised tile handle.
 * @param  result  Output struct to populate.
 */
void tile_sense_tof_get_result(tile_t *tile, sense_tof_result_t *result);

/**
 * @brief  DSL-friendly flat-output variant of get_result().
 *
 * Drops the struct in favor of a positional int[5] array — the DSL
 * doesn't have a struct ABI yet, so the per-field outputs come back
 * indexed. Layout: out[0]=distance_mm, out[1]=status,
 * out[2]=reliability, out[3]=temperature_c, out[4]=result_number.
 * The temperature is widened from int8_t to int32_t so negative
 * values sign-extend correctly.
 *
 * @studio expose category=tile name=get_result returns=int[5] section=runtime
 * @studio out_buffer out type=int32_t length=5
 *
 * @param  tile  Initialised tile handle.
 * @param  out   Output buffer (5 int32_t slots).
 */
void tile_sense_tof_get_result_flat(tile_t *tile, int32_t *out);

/**
 * @brief  DSL-friendly flat-output variant of measure_single().
 *
 * Combines a single-shot measurement with positional int outputs.
 * Same layout convention as get_result_flat — the per-field values
 * land in five out-scalar slots. Returns the chip's success flag.
 *
 * @studio expose category=tile name=measure_single returns=bool section=runtime
 * @studio out_scalar mm type=int32_t
 * @studio out_scalar status type=int32_t
 * @studio out_scalar reliability type=int32_t
 * @studio out_scalar temp_c type=int32_t
 * @studio out_scalar seq type=int32_t
 *
 * @param  tile        Initialised tile handle.
 * @param  mm          Output: distance in millimetres.
 * @param  status      Output: result status code.
 * @param  reliability Output: 0–63 reliability score.
 * @param  temp_c      Output: die temperature in degrees Celsius.
 * @param  seq         Output: monotonic result counter.
 * @param  timeout_ms  Maximum wait for the measurement to complete.
 * @return 1 on a valid result, 0 on timeout / bus error.
 */
uint8_t tile_sense_tof_measure_single_flat(tile_t *tile,
                                           int32_t *mm,
                                           int32_t *status,
                                           int32_t *reliability,
                                           int32_t *temp_c,
                                           int32_t *seq,
                                           uint32_t timeout_ms);

/**
 * @brief  Check if a new measurement result is available.
 * @studio expose category=tile name=result_ready returns=bool section=runtime
 *
 * Reads the INT_STATUS register and checks the result interrupt bit.
 * Does not clear the interrupt — that is done by get_result() or
 * get_distance_mm().
 *
 * @param  tile  Initialised tile handle.
 * @return 1 if a new result is pending, 0 otherwise.
 */
uint8_t tile_sense_tof_result_ready(tile_t *tile);

/* ---- Calibration ---- */

/**
 * @brief  Run the factory calibration procedure.
 * @studio expose category=tile name=factory_calibrate returns=bool section=advanced
 *
 * Performs a calibration measurement using the current mode settings.
 * The sensor must be positioned with a known target or open field per
 * the TMF8806 calibration guidelines. Results are stored internally
 * and can be retrieved with tile_sense_tof_get_calibration().
 *
 * This is a blocking call that waits for the calibration to complete.
 *
 * @param  tile        Initialised tile handle.
 * @param  timeout_ms  Maximum wait time in milliseconds.
 * @return 1 if calibration completed successfully, 0 on timeout or error.
 */
uint8_t tile_sense_tof_factory_calibrate(tile_t *tile, uint32_t timeout_ms);

/**
 * @brief  Load factory calibration data into the driver.
 *
 * Stores a 14-byte calibration dataset that will be written to the
 * sensor before each measurement start. Call this after init() to
 * restore calibration from non-volatile storage.
 *
 * @studio expose category=tile name=set_calibration section=advanced
 * @studio in_buffer data type=uint8_t length=14
 *
 * @param  tile  Initialised tile handle.
 * @param  data  Pointer to 14-byte calibration data array.
 */
void tile_sense_tof_set_calibration(tile_t *tile, const uint8_t *data);

/**
 * @brief  Read the current factory calibration data from the sensor.
 *
 * Reads 14 bytes of calibration data from the sensor's calibration
 * registers. Typically called after tile_sense_tof_factory_calibrate()
 * to save the data for later reloading via set_calibration().
 *
 * @studio expose category=tile name=get_calibration returns=int[14] section=advanced
 * @studio out_buffer data type=uint8_t length=14
 *
 * @param  tile  Initialised tile handle.
 * @param  data  Output buffer for 14 bytes of calibration data.
 */
void tile_sense_tof_get_calibration(tile_t *tile, uint8_t *data);

/* ---- Info ---- */

/**
 * @brief  Read the application firmware version.
 *
 * Returns the major, minor, and patch version of the App0 measurement
 * application running on the TMF8806.
 *
 * @param  tile     Initialised tile handle.
 * @param  version  Output struct to populate.
 */
void tile_sense_tof_get_app_version(tile_t *tile, sense_tof_version_t *version);

/**
 * @brief  DSL-friendly flat-output variant of get_app_version().
 *
 * Drops the struct in favor of three positional out-scalars.
 *
 * @studio expose category=tile name=get_app_version section=advanced
 * @studio out_scalar major type=int32_t
 * @studio out_scalar minor type=int32_t
 * @studio out_scalar patch type=int32_t
 *
 * @param  tile   Initialised tile handle.
 * @param  major  Output: major version number.
 * @param  minor  Output: minor version number.
 * @param  patch  Output: patch version number.
 */
void tile_sense_tof_get_app_version_flat(tile_t *tile,
                                         int32_t *major,
                                         int32_t *minor,
                                         int32_t *patch);

/**
 * @brief  Read the device serial number.
 *
 * Issues the serial number command (0x47) and reads back the response.
 * The serial number is a 4-byte unique device identifier.
 *
 * @param  tile    Initialised tile handle.
 * @param  serial  Output buffer for 4 bytes of serial data.
 * @return 1 if serial number was read successfully, 0 on error.
 */
uint8_t tile_sense_tof_get_serial_number(tile_t *tile, uint8_t *serial);

/**
 * @brief  DSL-friendly flat-output variant of get_serial_number().
 *
 * Drops the success bool — on bus error, the buffer comes back as
 * all-zeros, which is invalid as a real serial number so callers can
 * detect failure by checking for a zero serial. Bytes widen to int32
 * for DSL int compatibility.
 *
 * @studio expose category=tile name=get_serial_number returns=int[4] section=advanced
 * @studio out_buffer out type=int32_t length=4
 *
 * @param  tile  Initialised tile handle.
 * @param  out   Output buffer (4 int32_t slots).
 */
void tile_sense_tof_get_serial_number_flat(tile_t *tile, int32_t *out);

/* ---- Runtime configuration ---- */

/**
 * @brief  Change the distance mode on the fly.
 * @studio expose category=tile name=set_distance_mode section=runtime
 * @studio control mode label="Distance range" tier=basic default=SENSE_TOF_RANGE_2500MM
 *
 * Stops any active measurement, updates the cached mode, and restarts.
 * If no measurement was running, only updates the config for the next start().
 *
 * @param  tile  Initialised tile handle.
 * @param  mode  New distance mode.
 */
void tile_sense_tof_set_distance_mode(tile_t *tile, sense_tof_distance_mode_t mode);

/**
 * @brief  Change the measurement repetition period on the fly.
 * @studio expose category=tile name=set_period section=runtime
 * @studio control period label="Measurement rate" tier=basic default=SENSE_TOF_PERIOD_30MS role=sample_rate
 *
 * Slower rates save most of the power: the laser only fires for the
 * ranging time (~24 ms at the default 900k iterations), and the chip
 * idles at ~140 uA for the rest of each period.
 *
 * Stops any active measurement, updates the cached period, and restarts.
 * If no measurement was running, only updates the config for the next start().
 *
 * @param  tile    Initialised tile handle.
 * @param  period  New repetition period (sense_tof_period_t, or 1-253 ms).
 */
void tile_sense_tof_set_period(tile_t *tile, sense_tof_period_t period);

/**
 * @brief  Change the per-measurement iteration count on the fly.
 * @studio expose category=tile name=set_kilo_iters section=runtime
 * @studio control kilo_iters label="Iterations (thousands)" tier=advanced default=900
 *
 * Iterations (in thousands) trade power for SNR/range: more iterations
 * give a stronger return and longer reach at higher current draw.
 * Typical range 10-4000 (10k-4M); the ranging default is 900. Stops any
 * active measurement, updates the cached value, and restarts.
 *
 * @param  tile         Initialised tile handle.
 * @param  kilo_iters   [10..4000] Iterations in thousands (e.g. 900 = 900k).
 */
void tile_sense_tof_set_kilo_iters(tile_t *tile, uint16_t kilo_iters);

/**
 * @brief  Change the detection threshold on the fly.
 * @studio expose category=tile name=set_threshold section=runtime
 * @studio control threshold label="Detection threshold" tier=advanced default=6
 *
 * Sets cmd_data3[5:0] — the minimum confidence for a reported target.
 * Higher values reject weak/spurious returns; 0 reports everything.
 * Stops any active measurement, updates the cached value, and restarts.
 *
 * @param  tile       Initialised tile handle.
 * @param  threshold  [0..63] Detection threshold, 0-63.
 */
void tile_sense_tof_set_threshold(tile_t *tile, uint8_t threshold);

/* ---- Signal-quality diagnostics ---- */

/**
 * @brief  Read per-measurement signal-quality diagnostics.
 *
 * Reads the reference/object hit counts and crosstalk from the most
 * recent result block. Useful for assessing return strength and
 * reflectance beyond the reliability byte. Call after a result is ready.
 *
 * @param  tile  Initialised tile handle.
 * @param  sig   Caller-allocated struct, populated on return.
 */
void tile_sense_tof_get_signal_quality(tile_t *tile, sense_tof_signal_t *sig);

/**
 * @brief  Read signal-quality diagnostics into flat out-params (Studio).
 * @studio expose category=tile name=get_signal_quality returns=int section=runtime
 *
 * @param  tile            Initialised tile handle.
 * @param  reference_hits  Reference-channel hit count (or NULL).
 * @param  object_hits     Object-channel hit count (or NULL).
 * @param  crosstalk       Cross-talk count (or NULL).
 */
void tile_sense_tof_get_signal_quality_flat(tile_t *tile,
                                            int32_t *reference_hits,
                                            int32_t *object_hits,
                                            int32_t *crosstalk);

/* ---- Algorithm state (ultra-low-power) ---- */

/**
 * @brief  Save the algorithm state from the sensor.
 *
 * Reads 11 bytes of algorithm state data from the sensor (registers
 * 0x28-0x32). This state should be saved before entering sleep in
 * ultra-low-power mode, and restored after wake via restore_state().
 *
 * Preserving algorithm state across power cycles avoids the ~8 ms
 * re-initialisation penalty and maintains measurement accuracy.
 *
 * @studio expose category=tile name=save_state returns=int[11] section=advanced
 * @studio out_buffer data type=uint8_t length=11
 *
 * @param  tile  Initialised tile handle (measurement should be stopped).
 * @param  data  Output buffer for 11 bytes of state data.
 */
void tile_sense_tof_save_state(tile_t *tile, uint8_t *data);

/**
 * @brief  Restore algorithm state to the sensor.
 *
 * Writes 11 bytes of previously saved algorithm state to the sensor
 * (registers 0x2E-0x38). Call this after wake() and before start()
 * to resume from the saved algorithm state.
 *
 * The calibration data bitmask in the measurement command (cmd_data7)
 * is automatically updated to include algState when state data has
 * been restored.
 *
 * @studio expose category=tile name=restore_state section=advanced
 * @studio in_buffer data type=uint8_t length=11
 *
 * @param  tile  Initialised tile handle (after wake, before start).
 * @param  data  Pointer to 11 bytes of previously saved state data.
 */
void tile_sense_tof_restore_state(tile_t *tile, const uint8_t *data);

/* ---- Threshold-based interrupts (App0 cmd 0x08 / 0x09) ---- */

/**
 * @brief  Configure threshold-based interrupt suppression.
 *
 * Without this configured, the chip fires INT on every measurement
 * completion. With it: INT only fires when an object is detected in
 * the [low_mm, high_mm] range for `persistence` consecutive
 * measurements. Lets a sleeping host stay asleep until something
 * gets close.
 *
 * @studio expose category=tile name=set_threshold_interrupt section=config
 *
 * Per HostDriverCommunication §8.12 (cmd 0x08 = WR_ADD_CONFIG):
 *   - persistence = 0  → interrupt every measurement (default)
 *   - persistence = N  → require N consecutive in-range hits
 *   - low_mm > high_mm → no interrupts (no valid range)
 *
 * @param  tile          Initialised tile handle.
 * @param  persistence   0–255; 0 = disabled (every-measurement INT).
 * @param  low_mm        Lower bound (inclusive), millimetres.
 * @param  high_mm       Upper bound (inclusive), millimetres.
 * @return 1 on success, 0 on bus / command-execution timeout.
 */
uint8_t tile_sense_tof_set_threshold_interrupt(tile_t *tile,
                                               uint8_t persistence,
                                               uint16_t low_mm,
                                               uint16_t high_mm);

/**
 * @brief  Read back the current threshold-interrupt configuration.
 *
 * Per HostDriverCommunication §8.12.2 (cmd 0x09 = RD_ADD_CONFIG).
 *
 * @studio expose category=tile name=get_threshold_interrupt returns=bool section=advanced
 * @studio out_scalar persistence type=uint8_t
 * @studio out_scalar low_mm type=uint16_t
 * @studio out_scalar high_mm type=uint16_t
 *
 * @param  tile          Initialised tile handle.
 * @param  persistence   Output (may be NULL).
 * @param  low_mm        Output (may be NULL).
 * @param  high_mm       Output (may be NULL).
 * @return 1 on success, 0 on timeout.
 */
uint8_t tile_sense_tof_get_threshold_interrupt(tile_t *tile,
                                               uint8_t *persistence,
                                               uint16_t *low_mm,
                                               uint16_t *high_mm);

/* ---- System-clock readout (host-side oscillator drift correction) ---- */

/**
 * @brief  Read the chip's 32-bit system-clock tick counter.
 *
 * The TMF8806's internal oscillator can drift ±5 % over temperature,
 * which biases distance readings if the host's measurement period
 * doesn't compensate. Reading this register set after each
 * measurement lets the host compute the actual elapsed chip-time
 * vs. its own elapsed wall-time and apply a software correction
 * (HostDriverCommunication §10).
 *
 * Reads SYS_CLOCK_0..3 (0x24–0x27) as a single 4-byte burst.
 *
 * @studio expose category=tile name=get_sys_clock_ticks returns=int section=runtime
 *
 * @param  tile  Initialised tile handle.
 * @return 32-bit system-clock tick count (0 if not in App0 / no
 *         measurement yet).
 */
uint32_t tile_sense_tof_get_sys_clock_ticks(tile_t *tile);

/* ---- Raw histogram readout ---- */

/**
 * @brief  Capture and read a raw histogram from the SPAD/TDC array.
 *
 * Useful for custom multi-target detection, reflectance estimation,
 * or peak-curvature analysis beyond what App0's distance API
 * provides. Per HostDriverCommunication §8.11.
 *
 * Procedure (handled internally): stop any running measurement,
 * configure histogram-readout mode (cmd 0x30), start cyclic
 * measurement, wait for INT_STATUS bit 1, issue 0x80 to start the
 * histogram block read, wait for TID to advance, then read 128
 * bytes from the histogram register.
 *
 * After this returns, the chip stays in histogram mode. Call
 * `tile_sense_tof_stop()` and re-issue the original measurement
 * config to return to normal distance reads.
 *
 * @param  tile         Initialised tile handle.
 * @param  hist_type    Histogram type byte (see HostDriverCommunication
 *                      §8.11 / TMF8806 datasheet for available types;
 *                      0x10 = short-range example).
 * @param  buf128       Caller-supplied 128-byte buffer.
 * @param  timeout_ms   Maximum wait for the histogram-ready interrupt.
 * @return 1 on success (buf128 populated), 0 on timeout / bus error.
 */
uint8_t tile_sense_tof_read_histogram(tile_t *tile, uint8_t hist_type,
                                      uint8_t *buf128, uint32_t timeout_ms);

/**
 * @brief  DSL-friendly flat-output variant of read_histogram().
 *
 * Drops the success bool — on timeout / bus error the buffer comes
 * back zero-filled. Bytes widen to int32 for DSL int compatibility.
 * Caller passes hist_type and timeout_ms as scalar args; the buffer
 * is the function's output (collapsed into the int[128] return).
 *
 * @studio expose category=tile name=read_histogram returns=int[128] section=advanced
 * @studio out_buffer out type=int32_t length=128
 *
 * @param  tile        Initialised tile handle.
 * @param  hist_type   Histogram-type byte (see read_histogram() docs).
 * @param  timeout_ms  Maximum wait for the histogram-ready interrupt.
 * @param  out         Output buffer (128 int32_t slots).
 */
void tile_sense_tof_read_histogram_flat(tile_t *tile,
                                        uint8_t hist_type,
                                        uint32_t timeout_ms,
                                        int32_t *out);

/* ============================================================== */
/* Runtime — tier-2 idiomatic helpers                              */
/*                                                                  */
/* These compose the tier-1 surface above into "do the thing the   */
/* user wants to do" calls. They take care of single-shot          */
/* measurement and reliability gating internally so callers don't  */
/* need to read the TMF8806 datasheet to do presence detection.   */
/* ============================================================== */

/**
 * @brief  Reliability cutoff used by the presence helpers.
 *
 * Hits below this confidence threshold (out of 0–63) are treated
 * as "no object" by `is_object_within` and `wait_for_object`.
 * 32 is roughly the midpoint and rejects most noise/sun-glare
 * artefacts while still firing on a clean target.
 */
#define SENSE_TOF_PRESENCE_RELIABILITY_MIN  32

/**
 * @brief  Reliability cutoff used by `wait_for_object` polling, in ms.
 *
 * Polls every 10 ms by default. At a 30 ms measurement period the
 * loop sees a fresh result roughly every other poll; tighter polls
 * waste bus traffic, looser polls add latency.
 */
#define SENSE_TOF_WAIT_POLL_INTERVAL_MS     10

/**
 * @brief  Single-shot measurement; true if an object is within `mm`.
 *
 * @studio expose category=tile name=is_object_within returns=bool section=runtime
 *
 * Performs one blocking single-shot measurement (up to 200 ms) and
 * returns 1 iff the reported distance is non-zero, less than or
 * equal to `mm`, and the reliability is at least
 * @ref SENSE_TOF_PRESENCE_RELIABILITY_MIN. A zero distance or
 * low-confidence hit is treated as "no object".
 *
 * Restores the prior measurement period on the way out, so this
 * mixes safely with continuous-mode use.
 *
 * @param  tile  Initialised tile handle.
 * @param  mm    Distance threshold in millimetres (inclusive).
 * @return 1 if an object is within range with adequate confidence,
 *         0 otherwise (no target, low reliability, or bus timeout).
 */
uint8_t tile_sense_tof_is_object_within(tile_t *tile, uint16_t mm);

/**
 * @brief  Block until an object is detected within `mm`, or timeout.
 *
 * @studio expose category=tile name=wait_for_object returns=bool section=runtime
 *
 * Polls single-shot measurements until one matches the
 * `is_object_within` predicate or `timeout_ms` elapses. Polls every
 * @ref SENSE_TOF_WAIT_POLL_INTERVAL_MS milliseconds; each
 * single-shot measurement adds ~30 ms of its own.
 *
 * @note v1 implementation polls — keeps the helper self-contained
 *       and avoids the need to wire the chip's INT pin into the
 *       tile pad map. A future revision could swap to the chip's
 *       threshold-INT (see @ref tile_sense_tof_set_threshold_interrupt)
 *       to let a sleeping host stay asleep until proximity wakes it.
 *
 * @param  tile        Initialised tile handle.
 * @param  mm          Distance threshold in millimetres (inclusive).
 * @param  timeout_ms  Maximum wait time in milliseconds.
 * @return 1 if an object entered range before timeout, 0 otherwise.
 */
uint8_t tile_sense_tof_wait_for_object(tile_t *tile, uint16_t mm,
                                       uint32_t timeout_ms);

/**
 * @brief  Read distance and confidence in one call.
 *
 * Performs a single-shot measurement and writes the distance (mm)
 * and the confidence remapped from the chip's 0–63 reliability
 * scale to a 0–100 percent value. Confidence is computed as
 * `(reliability * 100) / 63` — integer math, no floats.
 *
 * @studio expose category=tile name=read_distance_with_confidence returns=bool section=runtime
 * @studio out_scalar mm type=uint16_t
 * @studio out_scalar confidence_pct type=uint8_t
 *
 * @param  tile           Initialised tile handle.
 * @param  mm             Output: distance in millimetres (NULL allowed).
 * @param  confidence_pct Output: 0–100 confidence (NULL allowed).
 * @return 1 on a successful measurement, 0 on timeout / bus error.
 */
uint8_t tile_sense_tof_read_distance_with_confidence(tile_t *tile,
                                                     uint16_t *mm,
                                                     uint8_t *confidence_pct);

#endif /* TILE_SENSE_TOF_H */
