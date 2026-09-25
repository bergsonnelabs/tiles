/**
 * @file   tile_sense_m_3g.h
 * @brief  Triaxial geomagnetic magnetometer driver for Sense.M.3G (BMM350).
 * @version 1.0.0
 *
 * Bosch Sensortec BMM350: 3-axis magnetometer, +/-2000 uT range, ODR from
 * 1.5625 Hz to 400 Hz, on-chip magnetic reset, data-ready interrupt, and a
 * 24-bit sensor-time counter.
 *
 * Output is integer nanotesla (nT) and milli-degrees Celsius, fully
 * temperature- and trim-compensated. 1000 nT = 1 uT; Earth's field is
 * roughly 25000-65000 nT depending on where you are.
 *
 * ## Where the compensation math comes from
 *
 * The BMM350 datasheet (BST-BMM350-DS001-27, rev 1.27) deliberately does
 * NOT publish the compensation formulas or the OTP trim-word layout — its
 * chapter 7 is a single link to Bosch's SensorAPI, and every accuracy
 * figure in Table 3 is qualified "after API compensation". The data
 * registers hold *uncompensated* counts.
 *
 * The OTP word map and the compensation chain implemented here are
 * therefore ported from Bosch Sensortec's BMM350_SensorAPI:
 *
 *   https://github.com/boschsensortec/BMM350_SensorAPI
 *   Copyright (c) 2025 Bosch Sensortec GmbH, BSD-3-Clause
 *
 * Ported, not copied verbatim: the reference implementation is
 * floating-point, and this driver is integer-only per house style, so the
 * chain runs in Q16.16 fixed point on int64 intermediates. Every ported
 * constant is marked `[API]` in the implementation, and everything marked
 * `[DS]` is traceable to the datasheet instead. If you are changing the
 * compensation, check it against the upstream API, not the datasheet —
 * the datasheet cannot tell you whether it is right.
 *
 * ## Power modes
 *
 * Suspend (the boot state, lowest power, settings retained), normal
 * (free-running at the configured ODR), and forced (one conversion on
 * request, then back to suspend). Forced mode is reachable only FROM
 * suspend — a normal-to-forced transition is ignored by the device, so
 * set_mode() routes through suspend for you.
 *
 * Polled example (Cores SDK):
 * @code
 *   tile_t mag;
 *   tile_sense_m_3g_init(core_tiles_pal(&core_i2c1), 0, &mag, NULL);
 *   tile_sense_m_3g_set_mode(&mag, SENSE_M_3G_MODE_NORMAL);
 *   while (1) {
 *       if (tile_sense_m_3g_data_ready(&mag)) {
 *           tile_sense_m_3g_read(&mag);
 *           printf("%ld %ld %ld nT\n",
 *                  tile_sense_m_3g_get_x_nt(&mag),
 *                  tile_sense_m_3g_get_y_nt(&mag),
 *                  tile_sense_m_3g_get_z_nt(&mag));
 *       }
 *       core_delay_ms(10);
 *   }
 * @endcode
 *
 * Interrupt-driven example (INT wired to a Core pad):
 * @code
 *   sense_m_3g_cfg_t cfg = {
 *       .odr = SENSE_M_3G_ODR_50HZ,
 *       .averaging = SENSE_M_3G_AVG_4,
 *       .int_pin = 9,                       // Core pad carrying INT
 *       .on_data = on_mag_sample,
 *   };
 *   tile_sense_m_3g_init(core_tiles_pal(&core_i2c1), 0, &mag, &cfg);
 *   tile_sense_m_3g_set_mode(&mag, SENSE_M_3G_MODE_NORMAL);
 *   while (1) { tile_sense_m_3g_process(&mag); }
 * @endcode
 *
 * ## The dummy-byte rule
 *
 * Every BMM350 I2C read returns TWO dummy bytes before the real data
 * (datasheet §9.2.3), so an n-byte read must fetch n+2 and discard the
 * front. Verified on hardware. Miss it and every register is silently
 * shifted by two — plausible-looking wrong values rather than an obvious
 * failure. All bus access in this driver goes through helpers that handle
 * it; if you add a raw read, do the same.
 *
 * Datasheet: Bosch Sensortec BMM350, BST-BMM350-DS001-27
 *
 * @studio tile label=Sense.M.3G icon=🧭
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=advanced category="Self-test" section=advanced
 *   The BMM350 can inject a ~130 uT internal field to verify the X and Y
 *   channels (datasheet §5.1.6). The datasheet describes the concept and
 *   the pass criterion but not the register sequence, which lives in the
 *   SensorAPI's self_test_config / TMR_SELFTEST_USER handling. Exposing
 *   it means porting that sequence and validating it against a known
 *   field. Driver-deferred, and the highest-value thing to add next.
 *
 * @studio unsupported severity=advanced category="OTP programming" section=advanced
 *   OTP write commands (DIR_PRGM, DIR_PRGM_1B, EXT_PRGM) are exposed by
 *   the chip but are a factory-provisioning facility — programming OTP is
 *   irreversible and would destroy the part's trim data. Read access is
 *   used internally for compensation and available through get_otp_word.
 *
 * @studio unsupported severity=niche category="I3C interface and in-band interrupts" section=advanced
 *   Hardware-gated: the BMM350 supports I3C with in-band interrupts
 *   (IBI), but Sense.M.3G is wired for I2C and the Core's I3C support is
 *   not routed to this tile. INT_CTRL_IBI and the I3C error register are
 *   consequently not exposed.
 *
 * @studio unsupported severity=niche category="Cross-axis coefficient override" section=config
 *   The cross-axis compensation coefficients are read from OTP and
 *   applied automatically. There is no API to override them with values
 *   from a system-level calibration; hard-iron and soft-iron correction
 *   for a specific enclosure belongs above this driver anyway.
 */

#ifndef INC_TILE_SENSE_M_3G_H_
#define INC_TILE_SENSE_M_3G_H_

#include "tiles.h"
#include <stdint.h>

/* ================================================================
 * Driver version
 * ================================================================ */

#define TILE_SENSE_M_3G_VERSION_MAJOR  1
#define TILE_SENSE_M_3G_VERSION_MINOR  0
#define TILE_SENSE_M_3G_VERSION_PATCH  0

TILES_CHECK_VERSION(1, 0);

/* ================================================================
 * Instance mapping
 * ================================================================ */

/**
 * | Instance | Address | ADSEL |
 * |----------|---------|-------|
 * | 0        | 0x14    | low   |
 * | 1        | 0x15    | high  |
 *
 * Sense.M.3G straps ADSEL low, so instance 0 is the tile's address —
 * confirmed on hardware (0x15 does not ACK).
 */
#define BMM350_I2C_ADDR_LOW    0x14
#define BMM350_I2C_ADDR_HIGH   0x15

/** CHIP_ID reset value (datasheet Table 8). */
#define BMM350_CHIP_ID         0x33

/** Dummy bytes prepended to every I2C read (datasheet §9.2.3). */
#define BMM350_DUMMY_BYTES     2

/* ================================================================
 * Register map — 8-bit addresses, 8-bit data
 * ================================================================ */

#define BMM350_REG_CHIP_ID           0x00
#define BMM350_REG_ERR_REG           0x02
#define BMM350_REG_PAD_CTRL          0x03
#define BMM350_REG_PMU_CMD_AGGR_SET  0x04  /**< [5:4] avg, [3:0] odr; reset 0x14 (100 Hz, 2x) */
#define BMM350_REG_PMU_CMD_AXIS_EN   0x05
#define BMM350_REG_PMU_CMD           0x06
#define BMM350_REG_PMU_CMD_STATUS_0  0x07
#define BMM350_REG_PMU_CMD_STATUS_1  0x08
#define BMM350_REG_I3C_ERR           0x09
#define BMM350_REG_I2C_WDT_SET       0x0A
#define BMM350_REG_INT_CTRL          0x2E
#define BMM350_REG_INT_CTRL_IBI      0x2F
#define BMM350_REG_INT_STATUS        0x30
#define BMM350_REG_MAG_X_XLSB        0x31  /**< 12-byte block: X, Y, Z, temp */
#define BMM350_REG_SENSORTIME_XLSB   0x3D
#define BMM350_REG_OTP_CMD           0x50
#define BMM350_REG_OTP_DATA_MSB      0x52
#define BMM350_REG_OTP_DATA_LSB      0x53
#define BMM350_REG_OTP_STATUS        0x55
#define BMM350_REG_TMR_SELFTEST_USER 0x60
#define BMM350_REG_CTRL_USER         0x61
#define BMM350_REG_CMD               0x7E

/** Length of the magnetic + temperature burst at 0x31 (4 x 3 bytes). */
#define BMM350_MAG_TEMP_DATA_LEN     12

/* ---- PMU commands (register 0x06) ---- */

#define BMM350_PMU_CMD_SUS      0x00  /**< Suspend mode */
#define BMM350_PMU_CMD_NM       0x01  /**< Normal mode */
#define BMM350_PMU_CMD_UPD_OAE  0x02  /**< Apply a new ODR / averaging setting */
#define BMM350_PMU_CMD_FM       0x03  /**< Forced mode */
#define BMM350_PMU_CMD_FM_FAST  0x04  /**< Forced mode, fast (ODR >= 25 Hz) */
#define BMM350_PMU_CMD_FGR      0x05  /**< Flip-gain reset (magnetic reset) */
#define BMM350_PMU_CMD_FGR_FAST 0x06
#define BMM350_PMU_CMD_BR       0x07  /**< Bit reset (magnetic reset) */
#define BMM350_PMU_CMD_BR_FAST  0x08

/** Soft-reset command written to CMD (0x7E). */
#define BMM350_CMD_SOFTRESET    0xB6

/* ---- Axis enable (register 0x05) ---- */

#define BMM350_EN_X             (1U << 0)
#define BMM350_EN_Y             (1U << 1)
#define BMM350_EN_Z             (1U << 2)
#define BMM350_EN_XYZ           0x07

/* ---- INT_CTRL (0x2E) bits ---- */

#define BMM350_INT_MODE_LATCHED (1U << 0)  /**< 0 = pulsed, 1 = latched */
#define BMM350_INT_POL_HIGH     (1U << 1)  /**< 0 = active low, 1 = active high */
#define BMM350_INT_OD_PUSHPULL  (1U << 2)  /**< 0 = open drain, 1 = push-pull */
#define BMM350_INT_OUTPUT_EN    (1U << 3)  /**< Drive the INT pin at all */
#define BMM350_INT_DRDY_EN      (1U << 7)  /**< Map data-ready into INT_STATUS */

/** Data-ready flag in INT_STATUS (0x30). */
#define BMM350_INT_STATUS_DRDY  (1U << 2)

/* ---- ERR_REG (0x02) ---- */

#define BMM350_ERR_PMU_CMD      (1U << 0)  /**< Last PMU command was rejected */

/* ================================================================
 * Enums
 * ================================================================ */

/**
 * Output data rate in normal mode (PMU_CMD_AGGR_SET bits [3:0], datasheet
 * §8.4). A LOWER code is a FASTER rate. The highest rates cap averaging:
 * see sense_m_3g_avg_t.
 */
typedef enum {
    SENSE_M_3G_ODR_400HZ    = 0x2,  /**< 400 Hz (no averaging only) @studio value=400 */
    SENSE_M_3G_ODR_200HZ    = 0x3,  /**< 200 Hz (averaging up to 2x) @studio value=200 */
    SENSE_M_3G_ODR_100HZ    = 0x4,  /**< 100 Hz (averaging up to 4x) @studio value=100 */
    SENSE_M_3G_ODR_50HZ     = 0x5,  /**< 50 Hz @studio value=50 */
    SENSE_M_3G_ODR_25HZ     = 0x6,  /**< 25 Hz (driver default) @studio value=25 */
    SENSE_M_3G_ODR_12_5HZ   = 0x7,  /**< 12.5 Hz @studio value=12.5 */
    SENSE_M_3G_ODR_6_25HZ   = 0x8,  /**< 6.25 Hz @studio value=6.25 */
    SENSE_M_3G_ODR_3_125HZ  = 0x9,  /**< 3.125 Hz @studio value=3.125 */
    SENSE_M_3G_ODR_1_5625HZ = 0xA,  /**< 1.5625 Hz @studio value=1.5625 */
} sense_m_3g_odr_t;

/**
 * Averaging / noise performance (PMU_CMD_AGGR_SET bits [5:4]).
 *
 * More averaging means less noise and more current, and it caps the
 * achievable ODR: 400 Hz needs AVG_NONE, 200 Hz allows up to AVG_2, and
 * 100 Hz up to AVG_4 (datasheet Table 5). set_odr_averaging() clamps an
 * illegal pair rather than letting the device reject it. Supply current
 * scales roughly with averaging x ODR: 12 uA at 1.5625 Hz / 1 sample up
 * to 455 uA at 400 Hz / 1 sample (datasheet Table 5, approximate).
 */
typedef enum {
    SENSE_M_3G_AVG_NONE = 0x0,  /**< 1 sample (low power) */
    SENSE_M_3G_AVG_2    = 0x1,  /**< 2 samples (regular power) */
    SENSE_M_3G_AVG_4    = 0x2,  /**< 4 samples (low noise, driver default) */
    SENSE_M_3G_AVG_8    = 0x3,  /**< 8 samples (ultra low noise, 50 Hz or slower) */
} sense_m_3g_avg_t;

/**
 * Power mode: the PMU_CMD (0x06) command that enters it. Values equal the
 * BMM350_PMU_CMD_* codes; written as literals so tools can read them.
 */
typedef enum {
    SENSE_M_3G_MODE_SUSPEND     = 0x00,  /**< Suspend: no conversions, ~1.8 uA, settings kept */
    SENSE_M_3G_MODE_NORMAL      = 0x01,  /**< Normal: free-running at the configured ODR */
    SENSE_M_3G_MODE_FORCED      = 0x03,  /**< One conversion (full CRST recharge), then suspend */
    SENSE_M_3G_MODE_FORCED_FAST = 0x04,  /**< One conversion (fast CRST recharge; ODR 25 Hz or faster), then suspend */
} sense_m_3g_mode_t;

/* ================================================================
 * Event callback
 * ================================================================ */

/**
 * Fired by tile_sense_m_3g_process() once a new sample has been read and
 * compensated. Always main-loop context, never ISR — I2C is safe here.
 *
 * @param tile  The tile instance
 * @param ctx   User context from config or on_data registration
 */
typedef void (*sense_m_3g_data_cb_t)(tile_t *tile, void *ctx);

/* ================================================================
 * Configuration
 * ================================================================ */

/**
 * Optional init config. Pass NULL for defaults: 25 Hz, 4x averaging, all
 * three axes, polled operation, INT pin untouched.
 */
typedef struct {
    sense_m_3g_odr_t odr;         /**< Output data rate. 0 = default 25 Hz. */
    sense_m_3g_avg_t averaging;   /**< Averaging. 0 = AVG_NONE (this is a real value, not "unset"). */
    uint8_t axes;                 /**< Axis mask (BMM350_EN_*). 0 = all three. */

    /* INT pin — set to route data-ready to a Core pad. The driver
     * configures the device for push-pull active-low pulsed output and
     * registers a falling-edge ISR. 0 = polled mode. */
    uint8_t int_pin;              /**< Core pad carrying INT. 0 = polled. */

    sense_m_3g_data_cb_t on_data; /**< Called by process() on a new sample. */
    void *data_ctx;               /**< User context for the callback. */
} sense_m_3g_cfg_t;

/* ================================================================
 * Public API
 * ================================================================ */

/**
 * @brief  Probe the bus for a BMM350.
 * @param  hal       Tiles HAL handle (I2C bus)
 * @param  instance  0 for address 0x14 (Sense.M.3G), 1 for 0x15
 * @return 1 if a device ACKs and reports CHIP_ID 0x33, 0 otherwise
 */
uint8_t tile_sense_m_3g_find(tiles_pal_t *hal, uint8_t instance);

/**
 * @brief  Initialize the magnetometer.
 *
 * Soft-resets the device, verifies CHIP_ID, downloads the 32 OTP trim
 * words and derives the compensation coefficients from them, powers the
 * OTP back down, then runs a full magnetic reset so the transducer starts
 * from a known magnetic state. Leaves the device in suspend — call
 * set_mode() to start measuring.
 *
 * Takes roughly 100 ms, most of it the OTP download and the magnetic
 * reset's bit-reset and flip-gain-reset settling.
 *
 * @param  hal       Tiles HAL handle (I2C bus)
 * @param  instance  0 for 0x14, 1 for 0x15
 * @param  tile      Tile handle to initialize
 * @param  cfg       Optional config. NULL for defaults.
 */
void tile_sense_m_3g_init(tiles_pal_t *hal, uint8_t instance,
                          tile_t *tile, const sense_m_3g_cfg_t *cfg);

/* ---- Data ---- */

/**
 * @brief  Read one sample and apply the full compensation chain.
 * @studio expose category=tile name=read returns=bool section=runtime
 *
 * Burst-reads magnetic and temperature data in a single transaction —
 * mandatory, because the device freezes the data registers for the
 * duration of a burst and single reads would tear across an update
 * (datasheet §5.2).
 *
 * @return 1 on success, 0 if the bus read failed
 */
uint8_t tile_sense_m_3g_read(tile_t *tile);

/**
 * @brief  Poll for a new sample, then read and dispatch it.
 * @studio expose category=tile name=process section=runtime
 *
 * Call from your main loop. Checks data-ready (or the INT flag when an
 * INT pin is configured), and on a new sample calls read() and fires the
 * callback. Cheap when nothing is ready.
 *
 */
void tile_sense_m_3g_process(tile_t *tile);

/**
 * @brief  Register or change the new-sample callback.
 * @studio expose category=tile name=on_data section=runtime
 * @param  cb    Callback (NULL to disable)
 * @param  ctx   User context passed to the callback
 */
void tile_sense_m_3g_on_data(tile_t *tile, sense_m_3g_data_cb_t cb, void *ctx);

/**
 * @brief  Compensated X field from the last read, in nanotesla.
 * @studio expose category=tile name=get_x_nt returns=int section=runtime
 * @return X in nT (1000 nT = 1 uT)
 */
int32_t tile_sense_m_3g_get_x_nt(tile_t *tile);

/**
 * @brief  Compensated Y field from the last read, in nanotesla.
 * @studio expose category=tile name=get_y_nt returns=int section=runtime
 * @return Y in nT
 */
int32_t tile_sense_m_3g_get_y_nt(tile_t *tile);

/**
 * @brief  Compensated Z field from the last read, in nanotesla.
 * @studio expose category=tile name=get_z_nt returns=int section=runtime
 * @return Z in nT
 */
int32_t tile_sense_m_3g_get_z_nt(tile_t *tile);

/**
 * @brief  Die temperature from the last read, in milli-degrees Celsius.
 *
 * This is the compensation chain's own temperature, not an ambient
 * reading — the die sits above ambient by whatever the board is doing.
 *
 * @studio expose category=tile name=get_temperature_mc returns=int section=runtime
 * @return Temperature in m°C (25000 = 25.000 °C)
 */
int32_t tile_sense_m_3g_get_temperature_mc(tile_t *tile);

/**
 * @brief  Field magnitude of the last sample, in nanotesla.
 *
 * sqrt(x² + y² + z²), computed with an integer square root. Useful as a
 * sanity check: a quiet indoor spot should land in the 25000-65000 nT
 * band, and a magnet nearby will swamp it.
 *
 * @studio expose category=tile name=get_magnitude_nt returns=int section=runtime
 * @return Magnitude in nT
 */
uint32_t tile_sense_m_3g_get_magnitude_nt(tile_t *tile);

/**
 * @brief  Check whether the device has a new sample waiting.
 *
 * Reads INT_STATUS, which clears the flag. Init configures the interrupt
 * as PULSED, and in pulsed mode the flag also clears itself ~1.25 ms after
 * the sample (datasheet §5.5), so a loop polling less often than that
 * misses most samples. For polled use, call
 * configure_interrupt(tile, 0, 0, 1, 1) to latch it until read.
 *
 * @studio expose category=tile name=data_ready returns=bool section=runtime
 * @return 1 if INT_STATUS reports data ready
 */
uint8_t tile_sense_m_3g_data_ready(tile_t *tile);

/**
 * @brief  Read the 24-bit sensor-time counter.
 *
 * Ticks at 40 µs resolution and marks when the last sample was generated,
 * not when you read it. Only advances in normal mode unless sensor-time
 * always-on is enabled. Wraps at 24 bits without saturating.
 *
 * @studio expose category=tile name=get_sensortime returns=int section=runtime
 * @return Raw 24-bit counter value
 */
uint32_t tile_sense_m_3g_get_sensortime(tile_t *tile);

/* ---- Lifecycle ---- */

/**
 * @brief  Set the power mode.
 *
 * Forced mode is reachable only from suspend, so a request to enter it
 * from normal mode is routed through suspend first. Each transition
 * blocks for the settling time the device needs, which for suspend to
 * forced depends on the averaging setting.
 *
 * Init leaves the device in suspend, so nothing is measured until this
 * (or wake(), or trigger_measurement()) is called.
 *
 * @studio expose category=tile name=set_mode section=lifecycle
 * @param  mode  Target mode (sense_m_3g_mode_t)
 */
void tile_sense_m_3g_set_mode(tile_t *tile, sense_m_3g_mode_t mode);

/**
 * @brief  Trigger a single forced-mode conversion and wait for it.
 *
 * Uses the fast forced mode when the configured ODR is 25 Hz or above and
 * the slow one otherwise, per datasheet §5.1.4. The device returns to
 * suspend on its own once the conversion completes. Does not read the
 * sample — call read() after.
 *
 * @studio expose category=tile name=trigger_measurement returns=bool section=lifecycle
 * @return 1 if a conversion completed, 0 on timeout
 */
uint8_t tile_sense_m_3g_trigger_measurement(tile_t *tile);

/**
 * @brief  Put the device in suspend mode (lowest power, settings kept).
 * @studio expose category=tile name=sleep section=lifecycle
 */
void tile_sense_m_3g_sleep(tile_t *tile);

/**
 * @brief  Return the device to normal (free-running) mode.
 * @studio expose category=tile name=wake section=lifecycle
 */
void tile_sense_m_3g_wake(tile_t *tile);

/**
 * @brief  Soft-reset the device and re-run the full init sequence.
 *
 * Re-downloads the OTP trim data and repeats the magnetic reset, so the
 * compensation coefficients survive. Leaves the device in suspend.
 *
 * @studio expose category=tile name=reset section=lifecycle
 */
void tile_sense_m_3g_reset(tile_t *tile);

/**
 * @brief  Run a magnetic reset (bit reset, then flip-gain reset).
 *
 * Restores the transducer's magnetic state after exposure to a strong
 * field. The device does this automatically every ODR tick while
 * measuring and once at boot, so it is only needed after the part has sat
 * in suspend through a field event — the sensor cannot detect one while
 * suspended (datasheet §5.1.5). Worth calling after more than a couple of
 * seconds in suspend, which also recharges the CRST capacitor.
 *
 * Must be run from suspend; a device in normal mode is parked, reset, and
 * restored.
 *
 * @studio expose category=tile name=magnetic_reset returns=bool section=lifecycle
 * @return 1 if both reset phases were acknowledged, 0 otherwise
 */
uint8_t tile_sense_m_3g_magnetic_reset(tile_t *tile);

/* ---- Configuration ---- */

/**
 * @brief  Set output data rate and averaging together.
 *
 * The pair is constrained: 400 Hz forces no averaging, 200 Hz caps at 2x,
 * 100 Hz at 4x (datasheet Table 5). An illegal combination is clamped to
 * the highest averaging the requested rate allows rather than rejected.
 * Both live in one register and need an update command, which this issues.
 *
 * @studio expose category=tile name=set_odr_averaging section=config
 * @studio control odr label="Output data rate" tier=basic default=SENSE_M_3G_ODR_25HZ role=sample_rate
 * @studio control averaging label="Averaging" tier=advanced default=SENSE_M_3G_AVG_4
 * @param  odr        Output data rate (sense_m_3g_odr_t)
 * @param  averaging  Averaging (sense_m_3g_avg_t)
 */
void tile_sense_m_3g_set_odr_averaging(tile_t *tile, sense_m_3g_odr_t odr,
                                       sense_m_3g_avg_t averaging);

/**
 * @brief  Enable or disable individual measurement axes.
 *
 * A disabled axis stops converting, saving current, and reads back 0.
 *
 * @studio expose category=tile name=set_axes section=config
 * @param  mask  [0..7] OR of BMM350_EN_X (1) / _Y (2) / _Z (4)
 */
void tile_sense_m_3g_set_axes(tile_t *tile, uint8_t mask);

/**
 * @brief  Configure the data-ready interrupt on the INT pin.
 *
 * Data-ready is always mapped into INT_STATUS. `latched` also decides
 * how long the INT_STATUS flag lives: latched holds it until read,
 * pulsed clears it after ~1.25 ms (datasheet §5.5). Note the datasheet
 * marks INT_CTRL.int_output_en reserved on A-Si silicon.
 *
 * @studio expose category=tile name=configure_interrupt section=config
 * @param  enable      1 to drive the INT pin, 0 to leave it idle
 * @param  active_high 1 for active-high, 0 for active-low
 * @param  push_pull   1 for push-pull, 0 for open-drain (needs a pull-up)
 * @param  latched     1 to latch until INT_STATUS is read, 0 for a pulse
 */
void tile_sense_m_3g_configure_interrupt(tile_t *tile, uint8_t enable,
                                         uint8_t active_high, uint8_t push_pull,
                                         uint8_t latched);

/**
 * @brief  Set the INT/SDA/SCL pad drive strength (0-7, weakest to strongest).
 * @studio expose category=tile name=set_pad_drive section=config
 * @param  drive  [0..7] Drive strength (reset value 7)
 */
void tile_sense_m_3g_set_pad_drive(tile_t *tile, uint8_t drive);

/**
 * @brief  Enable or disable the device's I2C watchdog.
 *
 * Recovers the bus if the host stalls mid-transaction. Independent of the
 * Core's own watchdog.
 *
 * @studio expose category=tile name=set_i2c_watchdog section=config
 * @param  enable   1 to enable, 0 to disable
 * @param  long_wdt 1 for the ~40 ms timeout, 0 for ~1.28 ms
 */
void tile_sense_m_3g_set_i2c_watchdog(tile_t *tile, uint8_t enable, uint8_t long_wdt);

/**
 * @brief  Keep the sensor-time counter running outside normal mode.
 * @studio expose category=tile name=set_sensortime_always_on section=config
 * @param  enable  1 to keep counting in suspend and forced mode
 */
void tile_sense_m_3g_set_sensortime_always_on(tile_t *tile, uint8_t enable);

/* ---- Diagnostics / advanced ---- */

/**
 * @brief  Read the device's error register.
 * @studio expose category=tile name=get_error returns=int section=advanced
 * @return ERR_REG contents; bit 0 set means the last PMU command was rejected
 */
uint8_t tile_sense_m_3g_get_error(tile_t *tile);

/**
 * @brief  Read the PMU command status register.
 * @studio expose category=tile name=get_pmu_status returns=int section=advanced
 * @return PMU_CMD_STATUS_0 (datasheet §8.7): bit 0 busy, bit 1 ODR
 *         overwritten, bit 2 averaging overwritten, bit 3 normal mode,
 *         bit 4 illegal command. Bits [7:5] are reserved in the datasheet;
 *         Bosch's SensorAPI reads the last command's value there.
 */
uint8_t tile_sense_m_3g_get_pmu_status(tile_t *tile);

/**
 * @brief  Read one of the 32 OTP trim words cached at init.
 *
 * The compensation coefficients are derived from these. Exposed for
 * diagnostics — comparing them against a known-good part is the fastest
 * way to tell a compensation bug from a broken OTP download.
 *
 * @studio expose category=tile name=get_otp_word returns=int section=advanced
 * @param  word  [0..31] Word index
 * @return The cached 16-bit OTP word, or 0 if the index is out of range
 */
uint16_t tile_sense_m_3g_get_otp_word(tile_t *tile, uint8_t word);

/**
 * @brief  Read the uncompensated 21-bit signed counts for one axis.
 *
 * Straight from the data registers with no compensation applied — the
 * input to the compensation chain. For debugging only; these are not
 * field values and are not in spec.
 *
 * @studio expose category=tile name=get_raw returns=int section=advanced
 * @param  axis  [0..3] 0 = X, 1 = Y, 2 = Z, 3 = temperature
 * @return Signed raw counts from the last read()
 */
int32_t tile_sense_m_3g_get_raw(tile_t *tile, uint8_t axis);

/**
 * @brief  Read a raw 8-bit register, dummy bytes handled.
 * @studio expose category=tile name=read_reg returns=int section=advanced
 * @param  reg   [0..127] Register address
 * @return Register value, or 0 if the tile is not ready
 */
uint8_t tile_sense_m_3g_read_reg(tile_t *tile, uint8_t reg);

/**
 * @brief  Write a raw 8-bit register.
 * @studio expose category=tile name=write_reg section=advanced
 * @param  reg    [0..127] Register address
 * @param  value  [0..255] Value to write
 */
void tile_sense_m_3g_write_reg(tile_t *tile, uint8_t reg, uint8_t value);

#endif /* INC_TILE_SENSE_M_3G_H_ */
