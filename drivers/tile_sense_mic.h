/**
 * @file   tile_sense_mic.h
 * @brief  Complete driver for the Sense.MIC tile (analog MEMS mic + amp + ADC).
 *         I2C-only, command-based protocol via tiles_pal_t raw I2C.
 * @version 2.4.0
 *
 * Signal chain (production schematic):
 *   - Same Sky CMM-2718AT-38164W-TR: analog, omnidirectional, top-port MEMS
 *     mic. −38 dBV/Pa (±1 dB), 64 dBA SNR, 128 dB SPL overload, 170 Ω
 *     output, 0.75 V DC output, 175 µA.
 *   - AC-coupled through C2 (100 nF) into the + input of an Analog Devices
 *     AD8605 (rail-to-rail CMOS op-amp, 1 mA). C3 (100 nF) also sits from
 *     that input to GND, so C2/C3 halve the audio on the way in.
 *   - Non-inverting gain 1 + R4/R3 = 1 + 47k/1k = 48x (C4 100 pF across
 *     R4: ~34 kHz low-pass). Mic to ADC: 0.5 x 48 = 24x.
 *   - The amp output drives the MAX11645's AIN0 and pad 8. AIN1 is not
 *     connected.
 *   - Maxim MAX11645: 12-bit, 2-channel, up to 94.4 ksps, I2C to 1.7 MHz,
 *     internal 2.048 V reference.
 *
 * Bias: the + input is biased from the MAX11645's REF pin through R2/R1
 * (330k/330k, with C6 100 nF on REF), and R3 returns to GND with no
 * DC-blocking capacitor, so the bias is amplified 48x as well. REF is
 * driven only by SENSE_MIC_REF_INTERNAL_BUF (Table 6); then the bias is
 * 1.024 V x 48 and the output sits at the V+ rail — no audio. In every
 * other mode REF is not connected, the bias is ~0 V, and the output rests
 * at 0 V: only the positive half-cycles of the sound reach the ADC. The
 * driver therefore defaults to the VDD reference, measures the resting
 * level at init(), and corrects its SPL figure for the half-wave signal.
 * (A board revision with a capacitor in series with R3 would centre the
 * signal on 1.024 V in the REF_INTERNAL_BUF mode.)
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
 * @studio unsupported severity=common category="Full-wave audio" section=runtime
 *   Hardware-gated. The amp rests at 0 V in every usable mode (its bias
 *   comes from the undriven REF pin and is gained 48x), so only positive
 *   half-cycles reach the ADC: fine for levels and events, not for
 *   recording a waveform. Needs a board revision (a capacitor in series
 *   with R3, then the REF_INTERNAL_BUF mode centres the signal).
 *
 * @studio unsupported severity=niche category="Differential / bipolar input" section=config
 *   Hardware-gated: AIN1 is not connected, so the ADC runs single-ended,
 *   where the MAX11645 ignores the bipolar setting.
 *
 * @studio unsupported severity=advanced category="External reference voltage on REF pin" section=config
 *   Hardware-gated. The MAX11645 accepts an external reference on
 *   its REF pin, but on this tile REF only feeds the on-board amp
 *   bias divider (R2/R1) and its 100 nF decoupling cap (C6); it is
 *   not on any pad, and AIN1 is unconnected. Closing this gap needs
 *   a tile revision. Until then SENSE_MIC_REF_EXTERNAL configures
 *   the chip but has no usable reference source.
 */

#ifndef INC_TILE_SENSE_MIC_H_
#define INC_TILE_SENSE_MIC_H_

#include "tiles.h"
#include <stdint.h>

/* ================================================================
 * Driver version
 * ================================================================ */

#define TILE_SENSE_MIC_VERSION_MAJOR  2
#define TILE_SENSE_MIC_VERSION_MINOR  4
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
#define MAX11645_CLK_EXTERNAL    (1 << 3)     /**< External clock: SCL clocks the conversion */

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
 * | REF_VDD           | VDD      | Default. 0.8 mV/count; tracks the supply     |
 * | REF_INTERNAL      | 2.048 V  | 0.5 mV/count, always on (+330 µA)            |
 * | REF_INTERNAL_BUF  | 2.048 V  | NOT USABLE on this tile: driving REF biases  |
 * |                   |          | the amp to the rail (see the header)         |
 * | REF_EXTERNAL      | REF pin  | NOT USABLE on this tile: REF is not on a pad |
 *
 * Values are the MAX11645's SEL[2:0] field (Table 6). They changed in v2.3.0 —
 * see the MAX11645_SEL_* note above; the names did not.
 */
typedef enum {
    SENSE_MIC_REF_VDD          = 0x00,  /**< Supply voltage (V+) */
    SENSE_MIC_REF_EXTERNAL     = 0x02,  /**< External reference on the REF pin (not routed on this tile) */
    SENSE_MIC_REF_INTERNAL     = 0x05,  /**< Internal 2.048 V */
    SENSE_MIC_REF_INTERNAL_BUF = 0x07,  /**< Internal 2.048V, also on the REF pin (saturates this board's amp) */
} sense_mic_ref_t;

/**
 * @brief  ADC channel selection.
 *
 * | Enum value | Channel | Tile signal                         |
 * |------------|---------|-------------------------------------|
 * | CH_AIN0    | AIN0    | MEMS microphone output (default)    |
 * | CH_AIN1    | AIN1    | Not connected on this tile          |
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
 * | SCAN_SINGLE  | Convert selected channel once (default)  |
 * | SCAN_UP      | Scan from AIN0 up to selected channel    |
 * | SCAN_8X      | Convert selected channel 8 times, returning 8 results |
 *
 * SCAN_8X is not averaging: each conversion comes back as its own result.
 * get_samples() uses it to fetch eight samples per I2C transaction (about
 * twice the burst rate); single reads take the first of the eight.
 */
typedef enum {
    SENSE_MIC_SCAN_SINGLE = 0x03,  /**< Single channel conversion (default) */
    SENSE_MIC_SCAN_UP     = 0x00,  /**< Scan from AIN0 up to CS0 */
    SENSE_MIC_SCAN_8X     = 0x01,  /**< Convert CS0 8 times, 8 results per read */
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
 * @note  Bipolar coding applies to differential inputs only. The driver
 *        always converts single-ended (AIN1 is not connected on this
 *        tile), and the MAX11645 then works unipolar whatever this bit
 *        says (datasheet "Unipolar/Bipolar"). So on this tile the two
 *        settings read the same.
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
    uint8_t  scan;      /**< Scan mode (sense_mic_scan_t). 0 = default, SENSE_MIC_SCAN_SINGLE. */
    uint8_t  clock;     /**< Conversion clock (sense_mic_clock_t). Default: SENSE_MIC_CLOCK_INTERNAL. */
    uint8_t  polarity;  /**< Output coding (sense_mic_polarity_t). Default: SENSE_MIC_POLARITY_UNIPOLAR. */
    uint16_t vref_mv;   /**< Reference voltage in mV. 0 = auto (3300 for VDD, 2048 for internal). Set it to the real supply when using the VDD reference off 3.3 V. */
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
 * @note   Blocks for ~5 ms (~15 ms with the internal reference, which
 *         takes 10 ms to wake). Call once at startup.
 */
void tile_sense_mic_init(tiles_pal_t *hal, uint8_t instance,
                         tile_t *tile, const sense_mic_cfg_t *cfg);

/**
 * @brief  Enter low-power mode (no conversions).
 *
 * The ADC powers down between conversions on its own; sleep also turns an
 * internal reference off (~330 µA). The mic and amp run from V+ and keep
 * drawing ~1.2 mA: the tile has no switch for them.
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
 * @note   The internal reference takes 10 ms to wake; this function waits
 *         for it. Don't select REF_INTERNAL_BUF or REF_EXTERNAL on this
 *         tile (see sense_mic_ref_t). Call calibrate() afterwards: the
 *         resting level in counts changes with the reference.
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
 * No effect on this tile: bipolar coding applies only to differential
 * inputs, and the driver always converts single-ended (AIN1 is not
 * connected). get_raw() stays 0-4095 straight binary either way. Kept for
 * API compatibility. For signed audio use get_audio_sample(), which
 * subtracts the calibrated resting level.
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
 * Measured during init() by averaging 64 samples. Near 0 on this board
 * (the amp rests at 0 V; see the header), a few counts of amp offset.
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
 * Useful for determining the resting level (near 0 on this board).
 *
 * @studio expose category=tile name=dc_level returns=int section=runtime
 * @studio in_buffer samples type=uint16_t length_param=count
 *
 * @param  samples  Sample buffer (typically from get_samples()).
 * @param  count    Number of samples in the buffer.
 * @return DC offset in raw ADC counts.
 */
uint16_t tile_sense_mic_dc_level(tile_t *tile, const uint16_t *samples, uint16_t count);

/**
 * @brief  Compute peak-to-peak amplitude of a sample buffer (raw counts).
 *
 * Returns max - min across all samples. Silence is a few counts. On this
 * board only the positive half of the signal is captured, so this is the
 * positive peak, not the full swing.
 *
 * @studio expose category=tile name=peak_to_peak returns=int section=runtime
 * @studio in_buffer samples type=uint16_t length_param=count
 *
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
 * @param  samples    Sample buffer.
 * @param  count      Number of samples.
 * @param  dc_offset  Resting level (get_dc_offset(), or dc_level() of a quiet buffer).
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
 * read_spl_db() / is_loud() / wait_for_sound() use the chain as built:
 * the CMM-2718AT-38164W's −38 dBV/Pa, 24x from mic to ADC, and a half-wave
 * correction (+3 dB) when the signal rests at 0 V, as it does on this
 * board. They are for "did something happen" detection — knock, clap,
 * voice, quiet vs busy — not metering: a few dB absolute, flat weighting
 * (no A-weighting), not yet checked against a reference meter. The usable
 * window is about 45 dB (one ADC count of RMS) to ~112 dB (half-wave
 * clipping at the VDD reference; ~108 dB with the internal 2.048 V),
 * below the mic's 128 dB overload point. Quieter than the floor reads
 * 30.0 dB.
 * ================================================================ */

/**
 * @brief  Quick "is the room loud right now?" check.
 *
 * @studio expose category=tile name=is_loud returns=bool section=runtime
 *
 * Captures a short sample buffer (~64 samples / ~5 ms at 12.5 ksps)
 * via @ref tile_sense_mic_get_samples, computes the RMS amplitude
 * relative to the calibrated DC offset, converts it to dB SPL via
 * the signal-chain model above, and compares against `threshold_db`
 * (0.1 dB units, e.g. 700 = 70.0 dB).
 *
 * @note  Blocking. Takes ~5 ms while sampling.
 *
 * @studio control threshold_db label="Loudness threshold" tier=basic scope=usage scale=0.1 unit=dB default=700
 * @param  threshold_db  [300..1100] SPL threshold in 0.1 dB units (e.g. 700 = 70 dB). Readings start at ~45 dB and clip near 112 dB, so thresholds outside that never change.
 * @return 1 if measured SPL > threshold, 0 otherwise.
 */
uint8_t tile_sense_mic_is_loud(tile_t *tile, int16_t threshold_db);

/**
 * @brief  Read instantaneous SPL in 0.1 dB units.
 *
 * @studio expose category=tile name=read_spl_db returns=int section=runtime
 *
 * Captures a short sample buffer (~64 samples), computes the RMS
 * amplitude relative to the calibrated resting level, scales it to
 * 0.1 mV via the configured Vref, then maps to dB SPL: −38 dBV/Pa
 * (1 Pa = 94 dB = 12.59 mV RMS at the mic, ~302 mV RMS at the ADC after
 * the 24x chain), +3 dB for the half-wave signal. Integer-only, with a
 * 32-entry log10 table.
 *
 * Returns dB SPL in 0.1 dB units (e.g. 700 = 70.0 dB).
 *
 * @note  Blocking. Takes ~5 ms while sampling.
 * @note  Accuracy regime documented in the section comment above.
 *
 * @return SPL in 0.1 dB units. 300 (30.0 dB) when below the ~45 dB
 *         measurement floor. Negative values are not produced.
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
 * @studio control threshold_db label="Sound wait threshold" tier=basic scope=usage scale=0.1 unit=dB default=700
 * @studio control timeout_ms label="Sound wait timeout" tier=advanced scope=usage default=5000
 * @param  threshold_db  [300..1100] SPL threshold in 0.1 dB units.
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
 * @studio control timeout_ms label="Clap wait timeout" tier=advanced scope=usage default=5000
 * @param  timeout_ms  [1..60000] ms Maximum wait, in milliseconds.
 * @return 1 if a clap pattern was detected, 0 on timeout.
 */
uint8_t tile_sense_mic_detect_clap(tile_t *tile, uint32_t timeout_ms);

#endif /* INC_TILE_SENSE_MIC_H_ */
