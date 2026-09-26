/**
 * @file  tile_sense_bp.h
 * @brief ILPS22QS barometric pressure & temperature sensor.
 *
 * Platform-agnostic driver for the ST ILPS22QS dual full-scale absolute
 * pressure sensor (260–1260 hPa or 260–4060 hPa) with embedded temperature
 * sensor, 128-sample FIFO, low-pass filter, and Qvar electrostatic sensing.
 *
 * Key specifications:
 *  - 24-bit pressure output, 0.5 hPa absolute accuracy (mode 1)
 *  - 16-bit temperature output, 100 LSB/°C
 *  - Output data rates from 1 Hz to 200 Hz
 *  - Configurable averaging (4 to 512 samples)
 *  - 128-slot pressure FIFO with watermark interrupt
 *  - One-shot mode for ultra-low-power operation
 *  - Pressure threshold interrupts with autozero/autorefp
 *  - One-point calibration via hardware offset registers
 *
 * Quick start:
 * @code
 *   #include "core.h"
 *   #include "core_tiles.h"
 *   #include "tile_sense_bp.h"
 *
 *   tile_t baro;
 *   sense_bp_cfg_t cfg = { .odr = SENSE_BP_ODR_25HZ, .lpf = 1, .bdu = 1 };
 *   tile_sense_bp_init(core_tiles_pal(&core_i2c3), 0, &baro, &cfg);
 *
 *   int32_t pressure_mhpa = tile_sense_bp_get_pressure_mhpa(&baro);
 *   int32_t temp_cdeg     = tile_sense_bp_get_temp_cdeg(&baro);
 * @endcode
 *
 * Datasheet: https://www.st.com/resource/en/datasheet/ilps22qs.pdf
 *
 * @studio tile label=Sense.BP icon=◇
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=advanced category="Analog Hub / Qvar charge-variation sensing" section=config
 *   Hardware-gated on Sense.BP rev a. The ILPS22QS reads charge variation
 *   on dedicated input pins (chip pins 5 = AH2/QVAR2 and 7 = AH1/QVAR1)
 *   that need an external electrode wired in. The current tile rev
 *   leaves tile pads 6/7/8 unconnected (per Sense-BP-a.json) — those
 *   chip pins are tied to GND on the PCB per the datasheet's "if not
 *   used" recommendation. Even with AH_QVAR_EN=1, no signal can reach
 *   the chip. Closing this gap requires a tile hardware revision that
 *   routes one or both AH pins to a tile pad with electrode access.
 *
 * @studio unsupported severity=advanced category="Alternate bus modes (SPI / I3C)"
 *   Ecosystem-gated. ILPS22QS supports 3-wire and 4-wire SPI (CS = pad
 *   3 strap) and I3C SDR. The driver framework currently uses tiles_pal
 *   I²C calls exclusively; adding SPI requires plumbing hal->spi_*
 *   through the driver, and I3C requires a new bus abstraction in
 *   Studio that doesn't exist yet. Defer to a future multi-bus
 *   driver framework pass.
 *
 * @note All bus I/O is routed through tiles_pal_t function pointers.
 *       This driver contains no platform-specific code.
 */

#ifndef TILE_SENSE_BP_H
#define TILE_SENSE_BP_H

#include "tiles.h"

/* ---- Driver version ---- */

#define TILE_SENSE_BP_VERSION_MAJOR  1
#define TILE_SENSE_BP_VERSION_MINOR  3
#define TILE_SENSE_BP_VERSION_PATCH  0

TILES_CHECK_VERSION(1, 0);

/* ---- Instance mapping ----
 *
 *  Instance  |  I2C address  |  AD0 pin state
 * -----------|---------------|------------------
 *     0      |     0x5D      |  Float / high (default)
 *     1      |     0x5C      |  Connected to GND
 */

/* ---- I2C addresses ---- */

#define ILPS22QS_I2C_ADDR_DEFAULT   0x5D  /**< AD0 float/high (default) */
#define ILPS22QS_I2C_ADDR_ALT       0x5C  /**< AD0 to GND */

/* ---- Register map ---- */

#define ILPS22QS_REG_INTERRUPT_CFG  0x0B
#define ILPS22QS_REG_THS_P_L       0x0C
#define ILPS22QS_REG_THS_P_H       0x0D
#define ILPS22QS_REG_IF_CTRL       0x0E
#define ILPS22QS_REG_WHO_AM_I      0x0F
#define ILPS22QS_REG_CTRL_REG1     0x10
#define ILPS22QS_REG_CTRL_REG2     0x11
#define ILPS22QS_REG_CTRL_REG3     0x12
#define ILPS22QS_REG_FIFO_CTRL     0x14
#define ILPS22QS_REG_FIFO_WTM      0x15
#define ILPS22QS_REG_REF_P_L       0x16
#define ILPS22QS_REG_REF_P_H       0x17
#define ILPS22QS_REG_I3C_IF_CTRL   0x19
#define ILPS22QS_REG_RPDS_L        0x1A
#define ILPS22QS_REG_RPDS_H        0x1B
#define ILPS22QS_REG_INT_SOURCE    0x24
#define ILPS22QS_REG_FIFO_STATUS1  0x25
#define ILPS22QS_REG_FIFO_STATUS2  0x26
#define ILPS22QS_REG_STATUS        0x27
#define ILPS22QS_REG_PRESS_OUT_XL  0x28
#define ILPS22QS_REG_PRESS_OUT_L   0x29
#define ILPS22QS_REG_PRESS_OUT_H   0x2A
#define ILPS22QS_REG_TEMP_OUT_L    0x2B
#define ILPS22QS_REG_TEMP_OUT_H    0x2C
#define ILPS22QS_REG_FIFO_DATA_OUT_PRESS_XL  0x78
#define ILPS22QS_REG_FIFO_DATA_OUT_PRESS_L   0x79
#define ILPS22QS_REG_FIFO_DATA_OUT_PRESS_H   0x7A

/* ---- Device ID ---- */

#define ILPS22QS_WHO_AM_I_VALUE    0xB4

/* ---- CTRL_REG2 bit masks ---- */

#define ILPS22QS_CTRL2_BOOT        (1 << 7)
#define ILPS22QS_CTRL2_FS_MODE     (1 << 6)
#define ILPS22QS_CTRL2_LFPF_CFG    (1 << 5)
#define ILPS22QS_CTRL2_EN_LPFP     (1 << 4)
#define ILPS22QS_CTRL2_BDU         (1 << 3)
#define ILPS22QS_CTRL2_SWRESET     (1 << 1)
#define ILPS22QS_CTRL2_ONESHOT     (1 << 0)

/* ---- STATUS bit masks ---- */

#define ILPS22QS_STATUS_T_OR       (1 << 5)
#define ILPS22QS_STATUS_P_OR       (1 << 4)
#define ILPS22QS_STATUS_T_DA       (1 << 1)
#define ILPS22QS_STATUS_P_DA       (1 << 0)

/* ---- FIFO_STATUS2 bit masks ---- */

#define ILPS22QS_FIFO_WTM_IA      (1 << 7)
#define ILPS22QS_FIFO_OVR_IA      (1 << 6)
#define ILPS22QS_FIFO_FULL_IA     (1 << 5)

/* ---- INT_SOURCE bit masks ---- */

#define ILPS22QS_INT_SRC_BOOT_ON  (1 << 7)
#define ILPS22QS_INT_SRC_IA       (1 << 2)
#define ILPS22QS_INT_SRC_PL       (1 << 1)
#define ILPS22QS_INT_SRC_PH       (1 << 0)

/* ---- INTERRUPT_CFG bit masks ---- */

#define ILPS22QS_INTCFG_AUTOREFP  (1 << 7)
#define ILPS22QS_INTCFG_RESET_ARP (1 << 6)
#define ILPS22QS_INTCFG_AUTOZERO  (1 << 5)
#define ILPS22QS_INTCFG_RESET_AZ  (1 << 4)
#define ILPS22QS_INTCFG_LIR       (1 << 2)
#define ILPS22QS_INTCFG_PLE       (1 << 1)
#define ILPS22QS_INTCFG_PHE       (1 << 0)

/* ---- Enumerations ---- */

/** Output data rate selection (CTRL_REG1 ODR[3:0], bits 6:3). */
typedef enum {
    SENSE_BP_ODR_POWERDOWN = 0x00,  /**< Power-down / one-shot */
    SENSE_BP_ODR_1HZ       = 0x01,  /**< 1 Hz @studio value=1 */
    SENSE_BP_ODR_4HZ       = 0x02,  /**< 4 Hz @studio value=4 */
    SENSE_BP_ODR_10HZ      = 0x03,  /**< 10 Hz @studio value=10 */
    SENSE_BP_ODR_25HZ      = 0x04,  /**< 25 Hz @studio value=25 */
    SENSE_BP_ODR_50HZ      = 0x05,  /**< 50 Hz @studio value=50 */
    SENSE_BP_ODR_75HZ      = 0x06,  /**< 75 Hz @studio value=75 */
    SENSE_BP_ODR_100HZ     = 0x07,  /**< 100 Hz @studio value=100 */
    SENSE_BP_ODR_200HZ     = 0x08,  /**< 200 Hz @studio value=200 */
} sense_bp_odr_t;

/**
 * Averaging selection (CTRL_REG1 AVG[2:0], bits 2:0). More averaging means
 * less noise and more current, and it caps the data rate (datasheet
 * Table 22): 512 samples up to 25 Hz, 128 up to 75 Hz, 64 up to 100 Hz.
 * Code 0b110 is not defined.
 */
typedef enum {
    SENSE_BP_AVG_4   = 0x00,  /**< 4 samples */
    SENSE_BP_AVG_8   = 0x01,  /**< 8 samples */
    SENSE_BP_AVG_16  = 0x02,  /**< 16 samples */
    SENSE_BP_AVG_32  = 0x03,  /**< 32 samples */
    SENSE_BP_AVG_64  = 0x04,  /**< 64 samples */
    SENSE_BP_AVG_128 = 0x05,  /**< 128 samples */
    SENSE_BP_AVG_512 = 0x07,  /**< 512 samples */
} sense_bp_avg_t;

/** Full-scale mode selection (CTRL_REG2 FS_MODE, bit 6). */
typedef enum {
    SENSE_BP_FS_1260HPA = 0,  /**< Mode 1: 260–1260 hPa, 4096 LSB/hPa */
    SENSE_BP_FS_4060HPA = 1,  /**< Mode 2: 260–4060 hPa, 2048 LSB/hPa */
} sense_bp_fs_t;

/** Low-pass filter bandwidth (CTRL_REG2 LFPF_CFG, bit 5). */
typedef enum {
    SENSE_BP_LPF_ODR_4 = 0,  /**< Bandwidth = ODR/4 @studio value=4 */
    SENSE_BP_LPF_ODR_9 = 1,  /**< Bandwidth = ODR/9 @studio value=9 */
} sense_bp_lpf_bw_t;

/** FIFO mode selection (FIFO_CTRL F_MODE[1:0] + TRIG_MODES). */
typedef enum {
    SENSE_BP_FIFO_BYPASS     = 0x00,  /**< FIFO disabled */
    SENSE_BP_FIFO_FIFO       = 0x01,  /**< Stop when full */
    SENSE_BP_FIFO_CONTINUOUS = 0x02,  /**< Dynamic stream, overwrites oldest */
    SENSE_BP_FIFO_BYP2FIFO   = 0x05,  /**< Bypass-to-FIFO on trigger */
    SENSE_BP_FIFO_BYP2CONT   = 0x06,  /**< Bypass-to-continuous on trigger */
    SENSE_BP_FIFO_CONT2FIFO  = 0x07,  /**< Continuous-to-FIFO on trigger */
} sense_bp_fifo_mode_t;

/* ---- Configuration struct ---- */

/**
 * @brief Optional init-time configuration for Sense.BP.
 *
 * Pass NULL to tile_sense_bp_init() for defaults:
 * ODR=25 Hz, AVG=4, FS=1260 hPa, LPF enabled at ODR/4, BDU enabled.
 */
typedef struct {
    uint8_t odr;       /**< Output data rate (sense_bp_odr_t). Default: SENSE_BP_ODR_25HZ. */
    uint8_t avg;       /**< Averaging depth (sense_bp_avg_t). Default: SENSE_BP_AVG_4. */
    uint8_t fs;        /**< Full-scale mode (sense_bp_fs_t). Default: SENSE_BP_FS_1260HPA. */
    uint8_t lpf;       /**< Low-pass filter: 1 = enabled, 0 = disabled. cfg = NULL
                            enables it; a zeroed struct turns it off. */
    uint8_t lpf_bw;    /**< LPF bandwidth (sense_bp_lpf_bw_t). Default: SENSE_BP_LPF_ODR_4. */
    uint8_t bdu;       /**< Block data update: 1 = enabled, 0 = continuous. cfg = NULL
                            enables it; a zeroed struct turns it off. */
} sense_bp_cfg_t;

/* ---- Lifecycle ---- */

/**
 * @brief  Check if a Sense.BP is present on the bus.
 * @param  hal      Platform HAL handle.
 * @param  instance 0 = default address (0x5D), 1 = alternate (0x5C).
 * @return 1 if device ACKs and WHO_AM_I matches, 0 otherwise.
 */
uint8_t tile_sense_bp_find(tiles_pal_t *hal, uint8_t instance);

/**
 * @brief  Initialise a Sense.BP tile.
 *
 * Probes the device, verifies WHO_AM_I, configures ODR/AVG/FS/LPF/BDU,
 * disables AH/Qvar for lower power, and sets tile state to READY.
 *
 * @param  hal      Platform HAL handle.
 * @param  instance 0 = default (0x5D), 1 = alternate (0x5C).
 * @param  tile     Tile handle to initialise.
 * @param  cfg      Configuration (NULL for defaults).
 */
void tile_sense_bp_init(tiles_pal_t *hal, uint8_t instance,
                        tile_t *tile, const sense_bp_cfg_t *cfg);

/**
 * @brief  Enter power-down mode (ODR = 0).
 * @studio expose category=tile name=sleep section=lifecycle
 */
void tile_sense_bp_sleep(tile_t *tile);

/**
 * @brief  Resume from power-down using cached ODR/AVG settings.
 * @studio expose category=tile name=wake section=lifecycle
 */
void tile_sense_bp_wake(tile_t *tile);

/**
 * @brief  Software-reset the device.
 *
 * All registers return to defaults. Call init() again after reset.
 *
 * @studio expose category=tile name=reset section=lifecycle
 */
void tile_sense_bp_reset(tile_t *tile);

/* ---- Configuration ---- */

/**
 * @brief  Set the output data rate.
 *
 * SENSE_BP_ODR_POWERDOWN stops conversions (use oneshot() to take a
 * reading), so Studio offers only the running rates. init() default: 25 Hz.
 *
 * @studio expose category=tile name=set_odr section=config
 * @studio control odr label="Output data rate" tier=basic default=SENSE_BP_ODR_25HZ role=sample_rate allow=SENSE_BP_ODR_1HZ,SENSE_BP_ODR_4HZ,SENSE_BP_ODR_10HZ,SENSE_BP_ODR_25HZ,SENSE_BP_ODR_50HZ,SENSE_BP_ODR_75HZ,SENSE_BP_ODR_100HZ,SENSE_BP_ODR_200HZ
 * @param  odr   Desired output data rate.
 */
void tile_sense_bp_set_odr(tile_t *tile, sense_bp_odr_t odr);

/**
 * @brief  Set the averaging filter depth.
 *
 * Trades noise for current: at 25 Hz with the LPF at ODR/4 the pressure
 * noise falls from 2.87 Pa RMS (4 samples) to 0.44 Pa (512), while the
 * supply current rises from 23 µA to 784 µA (datasheet Tables 22, 24).
 * init() default: 4 samples.
 *
 * @studio expose category=tile name=set_avg section=config
 * @studio control avg label="Averaging" tier=advanced default=SENSE_BP_AVG_4
 * @studio require expr="value(set_odr.odr) <= 25" when="avg == SENSE_BP_AVG_512" message="Averaging 512 samples limits the data rate to 25 Hz. Lower the data rate or the averaging."
 * @studio require expr="value(set_odr.odr) <= 75" when="avg == SENSE_BP_AVG_128" message="Averaging 128 samples limits the data rate to 75 Hz. Lower the data rate or the averaging."
 * @studio require expr="value(set_odr.odr) <= 100" when="avg == SENSE_BP_AVG_64" message="Averaging 64 samples limits the data rate to 100 Hz. Lower the data rate or the averaging."
 * @param  avg   Desired averaging.
 */
void tile_sense_bp_set_avg(tile_t *tile, sense_bp_avg_t avg);

/**
 * @brief  Set the full-scale mode.
 *
 * Mode 1 (up to 1260 hPa) has twice the resolution and about half the
 * noise of mode 2 (up to 4060 hPa). init() default: mode 1.
 *
 * @studio expose category=tile name=set_fullscale section=config
 * @studio control fs label="Pressure range" tier=advanced default=SENSE_BP_FS_1260HPA
 * @param  fs    Full-scale selection.
 */
void tile_sense_bp_set_fullscale(tile_t *tile, sense_bp_fs_t fs);

/**
 * @brief  Enable or disable the low-pass filter.
 *
 * The filter on the pressure output (CTRL_REG2 EN_LPFP / LFPF_CFG) cuts
 * the noise further at no current cost: at 25 Hz with 4-sample
 * averaging, 4.26 Pa RMS unfiltered, 2.87 at ODR/4, 2.20 at ODR/9
 * (datasheet Table 24). init() default: on, at ODR/4.
 *
 * @studio expose category=tile name=set_lpf section=config
 * @studio control enable label="Low-pass filter" tier=advanced type=bool default=1
 * @studio control bw label="Low-pass filter bandwidth" tier=advanced default=SENSE_BP_LPF_ODR_4 show="value(set_odr.odr) / value(bw)" unit=Hz when="enable == 1"
 * @param  enable  [0..1] 1 = enable, 0 = disable.
 * @param  bw      Bandwidth selection (only used if enable = 1).
 */
void tile_sense_bp_set_lpf(tile_t *tile, uint8_t enable, sense_bp_lpf_bw_t bw);

/* ---- Pressure data ---- */

/**
 * @brief  Read the raw 24-bit pressure output (two's complement).
 * @studio expose category=tile name=get_pressure_raw returns=int section=runtime
 * @return Raw 24-bit signed value, sign-extended to int32_t.
 */
int32_t tile_sense_bp_get_pressure_raw(tile_t *tile);

/**
 * @brief  Read pressure in milli-hectopascals (integer, no float).
 * @studio expose category=tile name=get_pressure_mhpa returns=int section=runtime
 *
 * Returns pressure * 1000 in mhPa units. For example, 1013250 = 1013.250 hPa.
 * Accounts for the current full-scale mode setting.
 *
 * @return Pressure in milli-hPa (mhPa).
 */
int32_t tile_sense_bp_get_pressure_mhpa(tile_t *tile);

/* ---- Temperature data ---- */

/**
 * @brief  Read the raw 16-bit temperature output (two's complement).
 * @studio expose category=tile name=get_temp_raw returns=int section=runtime
 * @return Raw 16-bit signed value, sign-extended to int16_t.
 */
int16_t tile_sense_bp_get_temp_raw(tile_t *tile);

/**
 * @brief  Read temperature in centi-degrees Celsius (integer, no float).
 * @studio expose category=tile name=get_temp_cdeg returns=int section=runtime
 *
 * Returns temperature * 100. For example, 2534 = 25.34 °C.
 * Sensor sensitivity is 100 LSB/°C, so this is (raw * 100) / 100 = raw.
 *
 * @return Temperature in centi-°C.
 */
int32_t tile_sense_bp_get_temp_cdeg(tile_t *tile);

/* ---- One-shot mode ---- */

/**
 * @brief  Trigger a single measurement in power-down mode.
 * @studio expose category=tile name=oneshot section=runtime
 *
 * ODR must be POWERDOWN. Sets the ONESHOT bit in CTRL_REG2.
 * The bit self-clears when the measurement is complete.
 *
 */
void tile_sense_bp_oneshot(tile_t *tile);

/* ---- Status ---- */

/**
 * @brief  Read the STATUS register.
 * @studio expose category=tile name=get_status returns=int section=runtime
 * @return Raw STATUS byte (use ILPS22QS_STATUS_* masks).
 */
uint8_t tile_sense_bp_get_status(tile_t *tile);

/**
 * @brief  Check if new pressure data is available.
 * @studio expose category=tile name=pressure_ready returns=bool section=runtime
 * @return 1 if P_DA is set, 0 otherwise.
 */
uint8_t tile_sense_bp_pressure_ready(tile_t *tile);

/**
 * @brief  Check if new temperature data is available.
 * @studio expose category=tile name=temp_ready returns=bool section=runtime
 * @return 1 if T_DA is set, 0 otherwise.
 */
uint8_t tile_sense_bp_temp_ready(tile_t *tile);

/* ---- FIFO ---- */

/**
 * @brief  Configure the FIFO mode.
 * @studio expose category=tile name=set_fifo_mode section=fifo
 * @param  mode  FIFO mode selection.
 */
void tile_sense_bp_set_fifo_mode(tile_t *tile, sense_bp_fifo_mode_t mode);

/**
 * @brief  Set the FIFO watermark threshold (0–127).
 * @studio expose category=tile name=set_fifo_watermark section=fifo
 * @param  watermark  [0..127] Threshold level.
 */
void tile_sense_bp_set_fifo_watermark(tile_t *tile, uint8_t watermark);

/**
 * @brief  Read the number of unread FIFO samples.
 * @studio expose category=tile name=get_fifo_level returns=int section=fifo
 * @return Number of unread samples (0–128).
 */
uint8_t tile_sense_bp_get_fifo_level(tile_t *tile);

/**
 * @brief  Read FIFO status flags.
 * @studio expose category=tile name=get_fifo_status returns=int section=fifo
 * @return Raw FIFO_STATUS2 byte (use ILPS22QS_FIFO_* masks).
 */
uint8_t tile_sense_bp_get_fifo_status(tile_t *tile);

/**
 * @brief  Read one raw 24-bit pressure sample from the FIFO.
 * @studio expose category=tile name=read_fifo_raw returns=int section=fifo
 * @return Raw 24-bit signed pressure value, sign-extended to int32_t.
 */
int32_t tile_sense_bp_read_fifo_raw(tile_t *tile);

/**
 * @brief  Read multiple raw pressure samples from the FIFO.
 *
 * @studio expose category=tile name=read_fifo_batch returns=int section=fifo
 * @studio out_buffer buf type=int32_t cap_param=count
 * @param  buf    Caller-allocated buffer the driver fills with raw
 *                24-bit signed values (sign-extended to int32_t).
 * @param  count  Capacity of `buf` — the driver fills up to this
 *                many samples and returns the actual count.
 * @return Number of samples actually read.
 */
uint8_t tile_sense_bp_read_fifo_batch(tile_t *tile, int32_t *buf,
                                      uint8_t count);

/* ---- Interrupt / threshold ---- */

/**
 * @brief  Set the pressure interrupt threshold in hPa.
 * @studio expose category=tile name=set_threshold_hpa section=config
 *
 * The threshold is applied to the differential pressure (P_DIFF_IN), so
 * it needs AUTOREFP or AUTOZERO. Enable PHE/PLE bits in INTERRUPT_CFG to
 * generate interrupts. THS_P is 15 bits (hPa × 16 in mode 1, × 8 in
 * mode 2); values past 2047 hPa (mode 1) / 4095 hPa (mode 2) saturate.
 *
 * @param  ths_hpa [0..2047] hPa Threshold (unsigned, applied symmetrically).
 */
void tile_sense_bp_set_threshold_hpa(tile_t *tile, uint16_t ths_hpa);

/**
 * @brief  Configure the interrupt source register.
 * @studio expose category=tile name=set_interrupt_cfg section=config
 * @param  cfg   [0..255] Raw INTERRUPT_CFG byte (use ILPS22QS_INTCFG_* masks).
 */
void tile_sense_bp_set_interrupt_cfg(tile_t *tile, uint8_t cfg);

/**
 * @brief  Read the interrupt source register (clears latched flags).
 * @studio expose category=tile name=get_int_source returns=int section=config
 * @return Raw INT_SOURCE byte (use ILPS22QS_INT_SRC_* masks).
 */
uint8_t tile_sense_bp_get_int_source(tile_t *tile);

/**
 * @brief  Check whether the chip's power-on boot sequence is complete.
 *
 * @studio expose category=tile name=is_boot_complete returns=int section=runtime
 *
 * After VDD ramps, the ILPS22QS reloads its trim parameters from NVM.
 * The BOOT_ON bit in INT_SOURCE reads 1 during this phase and 0 once
 * the chip is ready to be configured. Reads of this getter are
 * non-clearing (the BOOT_ON bit is preserved across the read; only
 * the threshold/IA flags clear on read).
 *
 * Useful when interrogating the chip immediately after VDD comes up,
 * or after software reset. A typical pattern:
 * @code
 *   tile_sense_bp_reset(&baro);
 *   while (!tile_sense_bp_is_boot_complete(&baro)) core_delay_ms(1);
 *   // chip is now ready for re-configuration
 * @endcode
 *
 * @return 1 if boot complete (BOOT_ON cleared), 0 if still booting.
 */
uint8_t tile_sense_bp_is_boot_complete(tile_t *tile);

/* ---- Reference / offset calibration ---- */

/**
 * @brief  Enable autozero mode.
 * @studio expose category=tile name=set_autozero section=config
 *
 * Captures current pressure as REF_P. Output registers then show
 * the difference from reference. Reset with reset_autozero().
 *
 */
void tile_sense_bp_set_autozero(tile_t *tile);

/**
 * @brief  Reset autozero mode to normal operation.
 * @studio expose category=tile name=reset_autozero section=config
 */
void tile_sense_bp_reset_autozero(tile_t *tile);

/**
 * @brief  Enable autorefp mode.
 * @studio expose category=tile name=set_autorefp section=config
 *
 * Captures current pressure as REF_P for interrupt threshold comparison.
 * Output registers are not affected. Reset with reset_autorefp().
 *
 */
void tile_sense_bp_set_autorefp(tile_t *tile);

/**
 * @brief  Reset autorefp mode to normal operation.
 * @studio expose category=tile name=reset_autorefp section=config
 */
void tile_sense_bp_reset_autorefp(tile_t *tile);

/**
 * @brief  Set pressure offset for one-point calibration (RPDS registers).
 * @studio expose category=tile name=set_pressure_offset section=config
 *
 * The offset is in raw LSB units (signed 16-bit) and is subtracted from
 * the measured pressure before output.
 *
 * @param  offset  [-32768..32767] Raw offset value (two's complement).
 */
void tile_sense_bp_set_pressure_offset(tile_t *tile, int16_t offset);

/**
 * @brief  Read the reference pressure registers (REF_P).
 * @studio expose category=tile name=get_ref_pressure returns=int section=config
 * @return 16-bit signed reference pressure value.
 */
int16_t tile_sense_bp_get_ref_pressure(tile_t *tile);

/* ---- Tier-2 helpers ---- */

/**
 * @brief  Estimate altitude (mm) above the supplied sea-level pressure.
 *
 * @studio expose category=tile name=read_altitude_mm returns=int section=runtime
 *
 * Pure-integer linear approximation around the reference pressure:
 *   h_mm = 8430 * (P0_pa - P_pa) / 100
 * which corresponds to ~8.43 mm of altitude per pascal of pressure
 * decrease — the standard-atmosphere lapse-rate slope near sea level.
 *
 * Accuracy regime: this is a first-order linearisation valid for
 * roughly ±1500 m around the supplied reference (i.e. the kind of
 * range you care about for indoor floor detection, ascent-rate
 * estimation, or short-baseline relative-altitude tracking). Error
 * grows with the cube of altitude — at ±3000 m the underlying
 * exponential law diverges from the linear fit by tens of metres,
 * and outside the ILPS22QS's pressure range (260–1260 hPa or
 * 260–4060 hPa depending on FS mode) the result is meaningless.
 * For absolute geodetic altitude over wide ranges, use a full
 * barometric-formula library on a host with floats.
 *
 * The reference is supplied in pascals (not hPa) for the convenience
 * of callers that already have a recent QNH/QFE figure handy. Pass
 * `101325` for the ICAO standard sea level reference.
 *
 * @param  sea_level_pa [26000..406000] Pa Reference pressure (e.g. 101325).
 * @return Altitude in millimetres above (positive) or below (negative)
 *         the reference. Negative when the measured pressure exceeds
 *         the reference (deeper than the reference altitude).
 */
int32_t tile_sense_bp_read_altitude_mm(tile_t *tile, uint32_t sea_level_pa);

/**
 * @brief  Block until measured pressure deviates from a captured baseline.
 *
 * @studio expose category=tile name=wait_for_pressure_change returns=int section=runtime
 *
 * Captures the current pressure at call time, then polls at the chip's
 * configured ODR cadence until either:
 *   - |pressure - baseline| >= threshold_hpa, or
 *   - timeout_ms elapses with no qualifying change.
 *
 * This is a blocking helper. The polling cadence is derived from the
 * cached CTRL_REG1 ODR field — the function sleeps for one ODR period
 * between samples (or 10 ms as a fallback if ODR is power-down or
 * unrecognised). For one-shot users, configure a non-power-down ODR
 * before calling.
 *
 * Useful for "tap-to-wake" altitude triggers, ascent/descent detection,
 * or pressure-step-driven UI events — without committing to the full
 * threshold-interrupt configuration in CTRL_REG3 / INTERRUPT_CFG.
 *
 * @param  threshold_hpa  [1..4060] hPa Absolute deviation that triggers return.
 * @param  timeout_ms     [0..600000] ms Maximum wait.
 * @return 1 if the threshold was crossed, 0 if the call timed out.
 */
uint8_t tile_sense_bp_wait_for_pressure_change(tile_t *tile,
                                               uint16_t threshold_hpa,
                                               uint32_t timeout_ms);

#endif /* TILE_SENSE_BP_H */
