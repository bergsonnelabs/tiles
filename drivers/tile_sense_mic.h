/**
 * @file   tile_sense_mic.h
 * @brief  Complete driver for the Sense.MIC tile (analog MEMS mic + amp + ADC).
 *         I2C-only, command-based protocol via tiles_pal_t raw I2C.
 * @version 2.3.0
 *
 * Analog signal chain: an analog MEMS microphone is AC-coupled into an
 * AD8605 op-amp gain stage, whose output drives the MAX11645 ADC (and is
 * also tapped out to a pad). Components:
 *   - Same Sky / CUI CMM-2718AT-42316-TR: omnidirectional ANALOG MEMS mic,
 *     −42 dBV/Pa sensitivity, 57 dBA SNR, 130 dB SPL AOP, 300 Ω output.
 *   - Analog Devices AD8605: precision RRIO op-amp, non-inverting gain
 *     ~48× (R4 47k / R3 1k → 1 + 47). The ADC therefore sees ~48× the
 *     bare mic voltage — the mV→SPL conversion divides this back out.
 *   - Maxim MAX11645: 12-bit 2-channel ADC, up to 94.4 ksps,
 *     I2C up to 1.7 MHz, internal 2.048V reference.
 *
 * The amplified analog output is also brought out to a tile pad (the
 * schematic routes it to pad 8). NOTE: as of this writing the canonical
 * tile JSON is mid-reconciliation — it lists the older PUI AMM-2742 mic
 * and labels pad 6 (not 8) as the analog out, and omits the AD8605. The
 * driver here reflects the real schematic; the product DB still needs the
 * mic part, the AD8605, and the analog-out pad (8) updated to match.
 *
 * The MAX11645 uses a command-based I2C protocol (no register addresses).
 * All bus access goes through tiles_pal_t i2c_write_raw / i2c_read_raw.
 *
 * Platform-agnostic: uses tiles_pal_t for all bus access.
 *
 * Quick start — polling:
 * @code
 *   tile_t mic;
 *   tile_sense_mic_init(core_tiles_pal(&core_i2c3), 0, &mic, NULL);
 *   uint16_t raw = tile_sense_mic_get_raw(&mic);
 *   uint16_t mv  = tile_sense_mic_get_raw_mv(&mic);
 * @endcode
 *
 * Quick start — continuous sampling:
 * @code
 *   tile_t mic;
 *   tile_sense_mic_init(core_tiles_pal(&core_i2c3), 0, &mic, NULL);
 *
 *   uint16_t samples[256];
 *   tile_sense_mic_get_samples(&mic, samples, 256);
 *
 *   uint16_t dc  = tile_sense_mic_dc_level(samples, 256);
 *   uint16_t pp  = tile_sense_mic_peak_to_peak(samples, 256);
 * @endcode
 *
 * Quick start — with internal reference for precise mV:
 * @code
 *   sense_mic_cfg_t cfg = { .ref = SENSE_MIC_REF_INTERNAL };
 *   tile_t mic;
 *   tile_sense_mic_init(core_tiles_pal(&core_i2c3), 0, &mic, &cfg);
 *   // Now get_raw_mv() uses 2048 mV full-scale (0.5 mV/LSB)
 * @endcode
 *
 * Datasheet: Maxim MAX11644/MAX11645, 19-4544; Rev 3, 9/09
 *
 * @studio tile label=Sense.MIC icon=◉
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=advanced category="External reference voltage on REF pin" section=config
 *   The MAX11645 supports an external reference on its REF/AIN1
 *   pin, but the Sense.MIC tile does not route REF/AIN1 to any
 *   pad (pad 6 carries the chip's analog audio output, not the
 *   ADC reference). Closing this gap requires a tile hardware
 *   revision that breaks REF out to a connector pad. Until then,
 *   the SENSE_MIC_REF_EXTERNAL* enum values configure the chip
 *   but have no usable external pin.
 */

#ifndef INC_TILE_SENSE_MIC_H_
#define INC_TILE_SENSE_MIC_H_

#include "tiles.h"
#include <stdint.h>

/* ================================================================
 * Driver version
 * ================================================================ */

#define TILE_SENSE_MIC_VERSION_MAJOR  2
#define TILE_SENSE_MIC_VERSION_MINOR  3
#define TILE_SENSE_MIC_VERSION_PATCH  0

TILES_CHECK_VERSION(1, 0);

/* ================================================================
 * Instance mapping
 * ================================================================ */

/**
 * | Instance | ID   | Hardware config                       |
 * |----------|------|---------------------------------------|
 * | 0        | 0x36 | Fixed address — no address pin        |
 *
 * The MAX11645 has a factory-set I2C address of 0x36. Only one
 * instance is supported per bus.
 */
#define MAX11645_I2C_ADDR  0x36

/* ================================================================
 * MAX11645 protocol constants
 *
 * The MAX11645 uses a command-based I2C protocol. Two command
 * byte types are distinguished by bit 7:
 *   - Setup byte   (bit 7 = 1): configures Vref, clock, polarity
 *   - Config byte  (bit 7 = 0): configures scan mode, channel, mode
 *
 * Conversion data is returned as 2 bytes (big-endian, 12-bit
 * right-aligned) via a raw I2C read with no preceding address.
 * ================================================================ */

/* --- Setup byte (bit 7 = 1) --- */

#define MAX11645_SETUP_REG       (1 << 7)   /**< Bit 7 = 1 → setup byte */

/* SEL[2:0]: bits 6:4 — reference voltage selection.
 * Datasheet 19-4544 Rev 3, Table 6 (SEL2 SEL1 SEL0):
 *   0 0 X  VDD                  REF pin not connected   internal ref off
 *   0 1 X  external reference   REF pin = input         internal ref off
 *   1 0 0  internal reference   REF pin not connected   ref off between conversions
 *   1 0 1  internal reference   REF pin not connected   ref always on   (preferred)
 *   1 1 0  internal reference   REF pin = output        ref off between conversions
 *   1 1 1  internal reference   REF pin = output        ref always on
 * v2.2.0 and earlier had these wrong: "internal" was 0x02 (= 0 1 0), which
 * selects the EXTERNAL reference — and this tile does not route the REF pin, so
 * the ADC referenced a floating input while the driver assumed 2048 mV. */
#define MAX11645_SEL_VDD         (0x00 << 4)  /**< VDD as reference (default) */
#define MAX11645_SEL_EXT         (0x02 << 4)  /**< External reference on the REF pin */
#define MAX11645_SEL_INT_AUTO    (0x04 << 4)  /**< Internal 2.048 V, off between conversions */
#define MAX11645_SEL_INT_ON      (0x05 << 4)  /**< Internal 2.048 V, always on (preferred) */
#define MAX11645_SEL_INT_AUTO_BUF (0x06 << 4) /**< Internal 2.048 V on the REF pin, off between conversions */
#define MAX11645_SEL_INT_ON_BUF  (0x07 << 4)  /**< Internal 2.048 V on the REF pin, always on */

/* CLK: bit 3 */
#define MAX11645_CLK_INTERNAL    (0 << 3)     /**< Internal clock (default) */
#define MAX11645_CLK_EXTERNAL    (1 << 3)     /**< External clock on AIN1 */

/* BIP/UNI: bit 2 */
#define MAX11645_UNI             (0 << 2)     /**< Unipolar output (default) */
#define MAX11645_BIP             (1 << 2)     /**< Bipolar output (two's comp) */

/* RST: bit 1 */
#define MAX11645_RST_NORESET     (1 << 1)     /**< No action on config register */
#define MAX11645_RST_RESET       (0 << 1)     /**< Reset config register to default */

/* --- Configuration byte (bit 7 = 0) --- */

#define MAX11645_CONFIG_REG      (0 << 7)     /**< Bit 7 = 0 → config byte */

/* SCAN[1:0]: bits 6:5 — scan mode */
#define MAX11645_SCAN_UP         (0x00 << 5)  /**< Scan from AIN0 up to CS0 channel */
#define MAX11645_SCAN_8X         (0x01 << 5)  /**< Convert CS0 channel 8 times */
#define MAX11645_SCAN_UPPER      (0x02 << 5)  /**< Scan from CS0 up to highest channel */
#define MAX11645_SCAN_SINGLE     (0x03 << 5)  /**< Convert CS0 channel only */

/* CS0: bit 1 — channel select (2-channel MAX11645) */
#define MAX11645_CS0_AIN0        (0 << 1)     /**< Select AIN0 (mic input) */
#define MAX11645_CS0_AIN1        (1 << 1)     /**< Select AIN1 */

/* SGL/DIF: bit 0 */
#define MAX11645_SINGLE_ENDED    (1 << 0)     /**< Single-ended (default) */
#define MAX11645_DIFFERENTIAL    (0 << 0)     /**< Differential */

/* --- ADC resolution --- */

#define MAX11645_ADC_BITS        12
#define MAX11645_ADC_MAX         ((1 << MAX11645_ADC_BITS) - 1)  /**< 4095 */

/* ================================================================
 * Configuration enums
 * ================================================================ */

/**
 * @brief  Reference voltage selection.
 *
 * | Enum value        | Vref     | Notes                                        |
 * |-------------------|----------|----------------------------------------------|
 * | REF_VDD           | VDD      | Simple, full-range (default)                 |
 * | REF_INTERNAL      | 2.048 V  | Precise, always on (datasheet's preferred)   |
 * | REF_INTERNAL_BUF  | 2.048 V  | Also driven out on the REF pin (not routed)  |
 * | REF_EXTERNAL      | REF pin  | NOT USABLE on this tile: REF is not routed   |
 *
 * Values are the MAX11645's SEL[2:0] field (Table 6). They changed in v2.3.0 —
 * see the MAX11645_SEL_* note above; the names did not.
 */
typedef enum {
    SENSE_MIC_REF_VDD          = 0x00,  /**< Supply voltage (3.3 V) */
    SENSE_MIC_REF_EXTERNAL     = 0x02,  /**< External reference on the REF pin (not routed on this tile) */
    SENSE_MIC_REF_INTERNAL     = 0x05,  /**< Internal 2.048 V */
    SENSE_MIC_REF_INTERNAL_BUF = 0x07,  /**< Internal 2.048 V, also on the REF pin */
} sense_mic_ref_t;

/**
 * @brief  ADC channel selection.
 *
 * | Enum value | Channel | Tile signal                         |
 * |------------|---------|-------------------------------------|
 * | CH_AIN0    | AIN0    | MEMS microphone output (default)    |
 * | CH_AIN1    | AIN1    | AIN1 / reference input              |
 */
typedef enum {
    SENSE_MIC_CH_AIN0 = 0,  /**< AIN0 — microphone (default) */
    SENSE_MIC_CH_AIN1 = 1,  /**< AIN1 */
} sense_mic_channel_t;

/**
 * @brief  Scan mode.
 *
 * | Enum value   | Behavior                                 |
 * |--------------|------------------------------------------|
 * | SCAN_UP      | Scan from AIN0 up to selected channel (default) |
 * | SCAN_SINGLE  | Convert selected channel once            |
 * | SCAN_8X      | Convert selected channel 8 times         |
 */
typedef enum {
    SENSE_MIC_SCAN_SINGLE = 0x03,  /**< Single channel conversion (default) */
    SENSE_MIC_SCAN_UP     = 0x00,  /**< Scan from AIN0 up to CS0 */
    SENSE_MIC_SCAN_8X     = 0x01,  /**< Convert CS0 8 times (averaging) */
} sense_mic_scan_t;

/**
 * @brief  Conversion-clock source.
 *
 * | Enum value | Behavior                                          |
 * |------------|---------------------------------------------------|
 * | INTERNAL   | Chip's 2.8 MHz internal oscillator (default)      |
 * | EXTERNAL   | Conversions clocked from the I2C SCL line         |
 *
 * External-clock mode lets the host drive conversion timing exactly
 * by holding SCL — useful for tightly synchronized multi-ADC sampling.
 * In external-clock mode the effective sample rate is set by the I2C
 * bus speed; SCL must remain active for the full conversion window.
 */
typedef enum {
    SENSE_MIC_CLOCK_INTERNAL = 0,  /**< Internal 2.8 MHz oscillator (default) */
    SENSE_MIC_CLOCK_EXTERNAL = 1,  /**< Host-clocked via SCL */
} sense_mic_clock_t;

/**
 * @brief  Output coding (unipolar vs bipolar).
 *
 * | Enum value | Output range          | Encoding                  |
 * |------------|-----------------------|---------------------------|
 * | UNIPOLAR   | 0 .. VREF             | Straight binary (default) |
 * | BIPOLAR    | -VREF/2 .. +VREF/2    | Two's complement          |
 *
 * @note  The Sense.MIC's MEMS mic is single-ended around a positive
 *        DC bias, so unipolar mode is the natural fit. Bipolar mode
 *        re-encodes the same single-ended sample as a signed value
 *        centred on VREF/2 — useful if your code wants signed audio
 *        directly out of get_raw(), but does NOT enable true
 *        differential reads (the tile has no second analog input
 *        wired to AIN1).
 */
typedef enum {
    SENSE_MIC_POLARITY_UNIPOLAR = 0,  /**< 0 .. VREF, straight binary (default) */
    SENSE_MIC_POLARITY_BIPOLAR  = 1,  /**< ±VREF/2, two's complement */
} sense_mic_polarity_t;

/* ================================================================
 * Configuration
 * ================================================================ */

/**
 * Optional init config. Pass NULL for defaults (VDD ref, AIN0,
 * single-ended, internal clock, unipolar coding).
 */
typedef struct {
    uint8_t  ref;       /**< Voltage reference (sense_mic_ref_t). Default: SENSE_MIC_REF_VDD. */
    uint8_t  channel;   /**< ADC channel (sense_mic_channel_t). Default: SENSE_MIC_CH_AIN0. */
    uint8_t  scan;      /**< Scan mode (sense_mic_scan_t). Default: SENSE_MIC_SCAN_SINGLE. */
    uint8_t  clock;     /**< Conversion clock (sense_mic_clock_t). Default: SENSE_MIC_CLOCK_INTERNAL. */
    uint8_t  polarity;  /**< Output coding (sense_mic_polarity_t). Default: SENSE_MIC_POLARITY_UNIPOLAR. */
    uint16_t vref_mv;   /**< Reference voltage in mV. 0 = auto (3300 for VDD, 2048 for internal). Set manually only for external ref. */
} sense_mic_cfg_t;

/* ================================================================
 * Public API — Lifecycle
 * ================================================================ */

/**
 * @brief  Check if a Sense.MIC is present on the bus (address probe).
 *
 * Lightweight probe — does not configure the device.
 * The MAX11645 has no WHO_AM_I register; this checks for an ACK at 0x36.
 */
uint8_t tile_sense_mic_find(tiles_pal_t *hal, uint8_t instance);

/**
 * @brief  Initialize the MAX11645 ADC.
 *
 * Sends setup byte (reference, clock, polarity) and configuration byte
 * (scan mode, channel, single-ended). Verifies the device responds by
 * performing a test read.
 *
 * Pass cfg=NULL for defaults: VDD reference, AIN0 (mic), single-ended,
 * unipolar, internal clock, single-channel scan.
 *
 * @note   Blocks for ~5 ms (setup settling + DC offset calibration).
 *         Call once at startup.
 */
void tile_sense_mic_init(tiles_pal_t *hal, uint8_t instance,
                         tile_t *tile, const sense_mic_cfg_t *cfg);

/**
 * @brief  Enter low-power mode (no conversions).
 *
 * @studio expose category=tile name=sleep section=lifecycle
 */
void tile_sense_mic_sleep(tile_t *tile);

/**
 * @brief  Wake from sleep, restore previous configuration.
 *
 * @studio expose category=tile name=wake section=lifecycle
 */
void tile_sense_mic_wake(tile_t *tile);

/**
 * @brief  Reset the config register to power-on defaults. Must call init() again.
 *
 * @studio expose category=tile name=reset section=lifecycle
 */
void tile_sense_mic_reset(tile_t *tile);

/* ================================================================
 * Public API — Configuration
 * ================================================================ */

/**
 * @brief  Change the reference voltage source.
 *
 * @note   When switching to internal ref, allow ~10 µs for settling.
 *         This function inserts a 1 ms delay automatically.
 *
 * @studio expose category=tile name=set_reference section=config
 * @studio control ref label="Voltage reference" tier=advanced default=SENSE_MIC_REF_VDD allow=SENSE_MIC_REF_VDD,SENSE_MIC_REF_INTERNAL
 * @param  ref  One of the sense_mic_ref_t values (VDD, INTERNAL, etc.)
 */
void tile_sense_mic_set_reference(tile_t *tile, sense_mic_ref_t ref);

/**
 * @brief  Change the active ADC channel.
 *
 * @param  ch  SENSE_MIC_CH_AIN0 (mic) or SENSE_MIC_CH_AIN1
 *
 * @studio expose category=tile name=set_channel section=config
 */
void tile_sense_mic_set_channel(tile_t *tile, sense_mic_channel_t ch);

/**
 * @brief  Change the scan mode.
 *
 * @param  scan  SENSE_MIC_SCAN_SINGLE, SENSE_MIC_SCAN_UP, or SENSE_MIC_SCAN_8X
 *
 * @studio expose category=tile name=set_scan_mode section=config
 */
void tile_sense_mic_set_scan_mode(tile_t *tile, sense_mic_scan_t scan);

/**
 * @brief  Switch the conversion-clock source.
 *
 * Selects whether the MAX11645 clocks conversions from its internal
 * 2.8 MHz oscillator (default) or from the I2C SCL line.
 *
 * In external-clock mode, conversion timing is locked to the host
 * I2C bus, which makes it possible to synchronise multiple Sense.MIC
 * tiles or to align ADC sampling with another time-base.
 *
 * @note  External-clock mode only takes effect during the next read
 *        transaction; SCL must remain active for the full conversion
 *        window or the result will be invalid.
 *
 * @param  clk  SENSE_MIC_CLOCK_INTERNAL or SENSE_MIC_CLOCK_EXTERNAL
 *
 * @studio expose category=tile name=set_clock_mode section=config
 */
void tile_sense_mic_set_clock_mode(tile_t *tile, sense_mic_clock_t clk);

/**
 * @brief  Switch the output coding (unipolar vs bipolar).
 *
 * Unipolar (default) returns 0–4095 straight binary, mid-bias near
 * 2048. Bipolar returns the same sample re-encoded as a signed
 * 12-bit two's-complement value centred on VREF/2.
 *
 * Both modes use the chip's single-ended input on AIN0 — bipolar
 * does NOT enable true differential reads (the tile does not route
 * an opposing analog input to AIN1). It just changes how get_raw()
 * encodes the same physical sample.
 *
 * After switching to bipolar, treat get_raw() output as a signed
 * 12-bit value (sign-extend before use): see get_audio_sample()
 * for an alternative that subtracts the calibrated DC offset.
 *
 * @param  pol  SENSE_MIC_POLARITY_UNIPOLAR or SENSE_MIC_POLARITY_BIPOLAR
 *
 * @studio expose category=tile name=set_polarity section=config
 */
void tile_sense_mic_set_polarity(tile_t *tile, sense_mic_polarity_t pol);

/**
 * @brief  Get the currently configured reference voltage in millivolts.
 *
 * Returns the Vref used for millivolt conversions (3300 for VDD, 2048
 * for internal, or the user-specified value for external).
 *
 * @studio expose category=tile name=get_vref_mv returns=int section=runtime
 */
uint16_t tile_sense_mic_get_vref_mv(tile_t *tile);

/* ================================================================
 * Public API — Data reads
 * ================================================================ */

/**
 * @brief  Read a single 12-bit ADC sample (0–4095).
 *
 * The MAX11645 auto-converts on every read — no trigger needed.
 * Conversion time is ~3.5 µs (internal clock); the I2C transaction
 * itself dominates the timing (~55 µs at 400 kHz).
 *
 * @studio expose category=tile name=get_raw returns=int section=runtime
 */
uint16_t tile_sense_mic_get_raw(tile_t *tile);

/**
 * @brief  Read a single sample and convert to millivolts.
 *
 * Conversion: mv = raw * vref_mv / 4096
 *
 * @studio expose category=tile name=get_raw_mv returns=int section=runtime
 */
uint16_t tile_sense_mic_get_raw_mv(tile_t *tile);

/**
 * @brief  Read a single AC-coupled audio sample (signed, relative to DC offset).
 *
 * Returns: (raw - dc_offset) where dc_offset is auto-calibrated during init().
 * Useful for audio waveform capture where DC bias is removed.
 *
 * @studio expose category=tile name=get_audio_sample returns=int section=runtime
 * @return Signed 16-bit value. 0 = silence.
 */
int16_t tile_sense_mic_get_audio_sample(tile_t *tile);

/**
 * @brief  Get the auto-calibrated DC offset (mic bias point).
 *
 * Measured during init() by averaging 64 samples. Typically 600–900
 * with VDD reference, depending on supply voltage and PCB layout.
 *
 * @studio expose category=tile name=get_dc_offset returns=int section=runtime
 */
uint16_t tile_sense_mic_get_dc_offset(tile_t *tile);

/**
 * @brief  Re-calibrate the DC offset (e.g., after changing reference).
 *
 * Averages 64 samples to update the stored bias point.
 * @note   Call in a quiet environment for best results.
 *
 * @studio expose category=tile name=calibrate section=advanced
 */
void tile_sense_mic_calibrate(tile_t *tile);

/**
 * @brief  Burst-read N samples into a buffer.
 *
 * Reads are back-to-back; effective sample rate depends on I2C bus speed:
 *   - 400 kHz: ~12.5 ksps
 *   - 1 MHz:   ~16 ksps
 *
 * @studio expose category=tile name=get_samples section=runtime
 * @studio out_buffer buf type=uint16_t cap_param=count
 * @param  buf    Output buffer (caller-allocated, min count entries)
 * @param  count  Number of samples to read
 */
void tile_sense_mic_get_samples(tile_t *tile, uint16_t *buf, uint16_t count);

/* ================================================================
 * Public API — Audio analysis utilities
 *
 * These operate on sample buffers and are pure computation — no I2C.
 * Pass buffers obtained from tile_sense_mic_get_samples().
 * ================================================================ */

/**
 * @brief  Compute the DC level (mean) of a sample buffer.
 *
 * Useful for determining the mic bias point. Varies with supply
 * voltage and PCB bias circuit (typically 600–900 with VDD ref).
 *
 * @studio expose category=tile name=dc_level returns=int section=runtime
 * @studio in_buffer samples type=uint16_t length_param=count
 *
 * @param  tile     Tile handle (unused — accepted for DSL-binding
 *                  symmetry with the rest of the driver surface).
 * @param  samples  Sample buffer (typically from get_samples()).
 * @param  count    Number of samples in the buffer.
 * @return DC offset in raw ADC counts.
 */
uint16_t tile_sense_mic_dc_level(tile_t *tile, const uint16_t *samples, uint16_t count);

/**
 * @brief  Compute peak-to-peak amplitude of a sample buffer (raw counts).
 *
 * Returns max - min across all samples. Silence ≈ 5–20 counts (noise floor).
 *
 * @studio expose category=tile name=peak_to_peak returns=int section=runtime
 * @studio in_buffer samples type=uint16_t length_param=count
 *
 * @param  tile     Tile handle (unused — accepted for DSL-binding symmetry).
 * @param  samples  Sample buffer.
 * @param  count    Number of samples.
 * @return Peak-to-peak amplitude in raw ADC counts.
 */
uint16_t tile_sense_mic_peak_to_peak(tile_t *tile, const uint16_t *samples, uint16_t count);

/**
 * @brief  Compute RMS amplitude relative to a DC offset (raw counts).
 *
 * @studio expose category=tile name=rms returns=int section=runtime
 * @studio in_buffer samples type=uint16_t length_param=count
 *
 * @param  tile       Tile handle (unused — accepted for DSL-binding symmetry).
 * @param  samples    Sample buffer.
 * @param  count      Number of samples.
 * @param  dc_offset  DC bias point (use dc_level() to measure, or pass 2048).
 * @return RMS of AC component in raw ADC counts.
 */
uint16_t tile_sense_mic_rms(tile_t *tile, const uint16_t *samples, uint16_t count,
                            uint16_t dc_offset);

/**
 * @brief  Convert peak-to-peak raw amplitude to millivolts.
 *
 * Uses the configured reference voltage to scale a peak-to-peak raw
 * ADC count into millivolts: mv = pp_raw * vref_mv / 4096.
 *
 * @studio expose category=tile name=amplitude_mv returns=int section=runtime
 * @param  pp_raw  [0..4095] Peak-to-peak amplitude in raw ADC counts.
 * @return Amplitude in millivolts.
 */
uint16_t tile_sense_mic_amplitude_mv(tile_t *tile, uint16_t pp_raw);

/* ================================================================
 * Runtime — tier-2 idiomatic helpers
 *
 * These compose the tier-1 surface above into "did the thing happen"
 * calls. They take care of buffer capture, RMS computation, and
 * mV→dB SPL mapping internally so callers don't need to read the
 * CMM-2718AT datasheet to write a clap detector.
 *
 * SPL accuracy regime
 * -------------------
 * The CMM-2718AT sensitivity (−42 dBV/Pa typ.) — through the AD8605's
 * ~48× gain — and the integer mV→dB lookup that backs read_spl_db() /
 * is_loud() / wait_for_sound() are tuned for "did something happen"
 * event detection — knock, clap, voice presence, ambient quiet vs.
 * busy. They are NOT calibrated for studio metering: expect roughly
 * ±5 dB absolute accuracy in the 50–100 dB SPL window, more error
 * toward the extremes (sub-50 dB falls into the noise floor of the
 * ADC, above ~110 dB the mic clips at its 130 dB AOP). A-weighting is
 * approximated as flat — the CMM-2718AT has its own frequency response
 * (50 Hz–10 kHz) and we don't apply any weighting filter.
 * ================================================================ */

/**
 * @brief  Quick "is the room loud right now?" check.
 *
 * @studio expose category=tile name=is_loud returns=bool section=runtime
 *
 * Captures a short sample buffer (~64 samples / ~5 ms at 12.5 ksps)
 * via @ref tile_sense_mic_get_samples, computes the RMS amplitude
 * relative to the calibrated DC offset, converts it to dB SPL via
 * the CMM-2718AT sensitivity model (with the AD8605 gain divided
 * out), and compares against `threshold_db` (0.1 dB units, e.g. 700 = 70.0 dB).
 *
 * @note  Blocking. Takes ~5 ms while sampling.
 *
 * @param  tile          Initialised tile handle.
 * @param  threshold_db  SPL threshold in 0.1 dB units (e.g. 700 = 70 dB).
 * @return 1 if measured SPL > threshold, 0 otherwise.
 */
uint8_t tile_sense_mic_is_loud(tile_t *tile, int16_t threshold_db);

/**
 * @brief  Read instantaneous SPL in 0.1 dB units.
 *
 * @studio expose category=tile name=read_spl_db returns=int section=runtime
 *
 * Captures a short sample buffer (~64 samples), computes the RMS
 * amplitude relative to the calibrated DC offset, scales to mV
 * via the configured Vref, then maps mV→dB SPL using the CMM-2718AT's
 * −42 dBV/Pa typical sensitivity AND the AD8605's ~48× gain (1 Pa ≈
 * 7.94 mV RMS at the mic → ~381 mV RMS at the ADC input, where 1 Pa
 * SPL = 94 dB). The conversion is integer-only with a 32-entry log10
 * lookup table (1..32 mV).
 *
 * Returns dB SPL in 0.1 dB units (e.g. 700 = 70.0 dB).
 *
 * @note  Blocking. Takes ~5 ms while sampling.
 * @note  Accuracy regime documented in the section comment above.
 *
 * @param  tile  Initialised tile handle.
 * @return SPL in 0.1 dB units. Floor at 300 (~30 dB) when below
 *         the noise floor. Negative values are not produced.
 */
int16_t tile_sense_mic_read_spl_db(tile_t *tile);

/**
 * @brief  Block until ambient SPL crosses a threshold (or timeout).
 *
 * @studio expose category=tile name=wait_for_sound returns=bool section=runtime
 *
 * Polls @ref tile_sense_mic_read_spl_db every ~5 ms until the
 * computed SPL (in 0.1 dB units) exceeds `threshold_db`, or the
 * elapsed time exceeds `timeout_ms`. Useful for "wake on sound"
 * patterns — e.g., wait for a door slam, then fire an action.
 *
 * @note  Blocking until threshold or timeout.
 *
 * @studio control timeout_ms label="Sound wait timeout" tier=advanced scope=usage
 * @param  tile          Initialised tile handle.
 * @param  threshold_db  SPL threshold in 0.1 dB units.
 * @param  timeout_ms    [1..60000] ms Maximum wait, in milliseconds.
 * @return 1 if threshold was crossed, 0 on timeout.
 */
uint8_t tile_sense_mic_wait_for_sound(tile_t *tile, int16_t threshold_db,
                                      uint32_t timeout_ms);

/**
 * @brief  Detect a clap pattern (two peaks, quiet brackets).
 *
 * @studio expose category=tile name=detect_clap returns=bool section=runtime
 *
 * Watches the SPL stream looking for the canonical clap signature:
 *   1. Quiet bracket (≥100 ms below ~50 dB SPL) — establishes baseline
 *   2. First peak  (>~70 dB SPL spike, <50 ms wide)
 *   3. Quiet gap   (50–500 ms below threshold)
 *   4. Second peak (>~70 dB SPL spike, <50 ms wide)
 *   5. Quiet bracket (≥100 ms below threshold) — ensures it's really a clap-clap
 *
 * Thresholds are integer-fixed: peak ≈ 700 (70.0 dB), quiet floor
 * ≈ 500 (50.0 dB). The pattern detector samples SPL every ~5 ms.
 *
 * @note  Blocking until pattern detected or timeout. False positives
 *        on door knocks / drawer slams are expected — this is a
 *        coarse pattern detector, not a trained classifier.
 *
 * @studio control timeout_ms label="Clap wait timeout" tier=advanced scope=usage
 * @param  tile        Initialised tile handle.
 * @param  timeout_ms  [1..60000] ms Maximum wait, in milliseconds.
 * @return 1 if a clap pattern was detected, 0 on timeout.
 */
uint8_t tile_sense_mic_detect_clap(tile_t *tile, uint32_t timeout_ms);

#endif /* INC_TILE_SENSE_MIC_H_ */
