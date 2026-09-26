/**
 * @file   tile_drive_a_2.h
 * @brief  Dual-channel audio output driver for the Drive.A.2 tile
 *         (DAC63202W smart DAC + 2x TPA2028D1 Class-D amplifiers).
 *         Supports I2C and SPI bus access via tiles_pal_t.
 * @version 3.3.0
 *
 * The Drive.A.2 tile provides two independent audio output channels,
 * each consisting of a 12-bit DAC channel feeding a 3W Class-D amplifier.
 *
 * DAC features (DAC63202W):
 *   - 12-bit resolution, dual channel
 *   - Voltage output with configurable gain (1x, 1.5x, 2x, 3x, 4x)
 *   - Internal 1.21V reference or VDD/external reference
 *   - Built-in waveform generation: sine, triangle, sawtooth
 *   - I2C (up to 1 MHz) or SPI (up to 50 MHz), auto-detected at boot
 *
 * Amplifier features (TPA2028D1, x2):
 *   - 3W into 4 ohm, 880 mW into 8 ohm per channel
 *   - I2C-programmable gain: -28 dB to +30 dB in 1 dB steps
 *   - AGC with configurable compression, attack/release/hold times
 *   - Both amplifiers share I2C address 0x58 — writes affect both
 *
 * Speaker safety: init() leaves the TPA2028D1 output limiter ON at
 * DRIVE_A_2_LIMITER_DEFAULT (3 dBV, 2.0 V peak, 0.25 W into 8 ohm),
 * with AGC compression off. Without it, +30 dB of gain on a full-scale
 * DAC signal drives a clipped square wave at the V+ rail (about 1.4 W
 * into 8 ohm at 3.3 V), enough to burn a small speaker. Raise or
 * disable it with tile_drive_a_2_amp_set_limiter().
 *
 * In SPI mode, only the DAC is controllable; amplifier functions
 * become no-ops since the I2C bus pins are repurposed for SPI.
 * The DAC's SPI is 3-wire (write-only) until INTERFACE-CONFIG.SDO-EN
 * is set, and a read takes two access cycles (SLASF73A §6.5.1), so
 * the DEVICE-ID check in init and every read-modify-write setter
 * need that 4-wire readback; this driver does not enable it yet.
 *
 * Quick start (I2C — DAC + amplifiers):
 * @code
 *   #include "core_tiles.h"  // provides core_tiles_pal()
 *
 *   tile_t dac;
 *   tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
 *   tile_drive_a_2_init(hal, 0, &dac, NULL);
 *   if (tile_is_ready(&dac)) {
 *       tile_drive_a_2_set(&dac, 0, 2048);       // Ch 0 mid-scale
 *       tile_drive_a_2_amp_set_gain(&dac, 12);    // +12 dB both amps
 *   }
 * @endcode
 *
 * Quick start (SPI — DAC only, no amplifier control):
 * @code
 *   #include "core_tiles.h"
 *
 *   tile_t dac;
 *   tiles_pal_t *hal = core_tiles_pal(&core_spi1);
 *   tile_drive_a_2_init(hal, 0, &dac, NULL);  // instance = CS index
 *   tile_drive_a_2_set_mv(&dac, 0, 1500);     // 1.5 V on channel 0
 * @endcode
 *
 * Datasheet (DAC): https://www.ti.com/product/DAC63202W
 * Datasheet (Amp): https://www.ti.com/product/TPA2028D1
 *
 * @studio tile label=Drive.A.2 icon=♬
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=advanced category="DAC PWM-output mode (FBx pins)"
 *   The DAC63202W FB0 / FB1 pins (package balls B2 / C2) can be
 *   configured as comparator / PWM outputs for driving external
 *   power-stage gates. On the Drive.A.2 tile these pins are tied
 *   to the on-board TPA2028D1 amplifier inputs (closed-loop voltage-
 *   output feedback) and are NOT routed to any tile-connector pad —
 *   so even if the driver enabled PWM mode, no Core GPIO could
 *   observe or use the output. Closing this gap requires a tile
 *   hardware revision that breaks FBx out to a pad.
 *
 * @studio unsupported severity=niche category="Per-amp independent control"
 *   The tile carries 2× TPA2028D1 amps wired to a single shared I²C
 *   bus, and the TPA2028D1 has a fixed factory I²C address (0x58) —
 *   any write at 0x58 is acknowledged by both amps simultaneously.
 *   Independent left / right gain, AGC, or shutdown therefore can't
 *   be achieved in software; closing this gap requires either an
 *   I²C-mux on the tile or amp ICs with selectable addresses.
 */

#ifndef INC_TILE_DRIVE_A_2_H_
#define INC_TILE_DRIVE_A_2_H_

#include "tiles.h"
#include <stdint.h>

/* -------------------------------------------------------------- */
/* Driver version                                                  */
/* -------------------------------------------------------------- */

#define TILE_DRIVE_A_2_VERSION_MAJOR  3
#define TILE_DRIVE_A_2_VERSION_MINOR  3
#define TILE_DRIVE_A_2_VERSION_PATCH  0

TILES_CHECK_VERSION(1, 0);

/* -------------------------------------------------------------- */
/* Instance mapping                                                */
/* -------------------------------------------------------------- */

/**
 * @brief  Instance-to-address mapping for Drive.A.2.
 *
 * | Instance | DAC addr | A0 pin config             |
 * |----------|----------|---------------------------|
 * | 0        | 0x49     | Float / VDD (default)     |
 * | 1        | 0x48     | GND                       |
 * | 2        | 0x4A     | SDA                       |
 * | 3        | 0x4B     | SCL                       |
 *
 * For SPI, instance selects the CS index (0–7).
 *
 * @note  Tile pads 2 and 3 have weak pull-ups that set I2C mode
 *        and the default address (0x49) at power-on.
 */
#define DAC63202W_I2C_ADDR_DEFAULT   0x49  /**< A0 → VDD (default) */
#define DAC63202W_I2C_ADDR_GND       0x48  /**< A0 → GND */
#define DAC63202W_I2C_ADDR_SDA       0x4A  /**< A0 → SDA */
#define DAC63202W_I2C_ADDR_SCL       0x4B  /**< A0 → SCL */

#define TPA2028D1_I2C_ADDR           0x58  /**< Fixed (both amps) */

/* -------------------------------------------------------------- */
/* DAC63202W register map                                          */
/* -------------------------------------------------------------- */

/* Per-channel registers (X = 0 uses 0x13–0x1C, X = 1 uses 0x01–0x06/0x19) */
#define DAC63202W_REG_DAC_0_MARGIN_HIGH   0x13
#define DAC63202W_REG_DAC_0_MARGIN_LOW    0x14
#define DAC63202W_REG_DAC_0_VOUT_CMP_CFG  0x15
#define DAC63202W_REG_DAC_0_IOUT_MISC_CFG 0x16
#define DAC63202W_REG_DAC_0_CMP_MODE_CFG  0x17
#define DAC63202W_REG_DAC_0_FUNC_CFG      0x18
#define DAC63202W_REG_DAC_0_DATA          0x1C

#define DAC63202W_REG_DAC_1_MARGIN_HIGH   0x01
#define DAC63202W_REG_DAC_1_MARGIN_LOW    0x02
#define DAC63202W_REG_DAC_1_VOUT_CMP_CFG  0x03
#define DAC63202W_REG_DAC_1_IOUT_MISC_CFG 0x04
#define DAC63202W_REG_DAC_1_CMP_MODE_CFG  0x05
#define DAC63202W_REG_DAC_1_FUNC_CFG      0x06
#define DAC63202W_REG_DAC_1_DATA          0x19

/* Shared registers */
#define DAC63202W_REG_COMMON_CONFIG       0x1F
#define DAC63202W_REG_COMMON_TRIGGER      0x20
#define DAC63202W_REG_COMMON_DAC_TRIG     0x21
#define DAC63202W_REG_GENERAL_STATUS      0x22

/** @brief  Expected DEVICE-ID value in GENERAL-STATUS bits[7:2]. */
#define DAC63202W_DEVICE_ID               0x06

/** @brief  Internal reference voltage in mV (SLASF73A §6.4.1.1.1). */
#define DAC63202W_VREF_INT_MV             1210

/** @brief  Tile V+ range in mV (Drive-A-2 power rail; also the DAC's VDD). */
#define DRIVE_A_2_VDD_MIN_MV              2500
#define DRIVE_A_2_VDD_MAX_MV              5500
/** @brief  V+ assumed when the app does not give one (cfg->vdd_mv = 0). */
#define DRIVE_A_2_VDD_DEFAULT_MV          3300

/** @brief  DAC resolution in bits. */
#define DAC63202W_DAC_BITS                12
#define DAC63202W_DAC_MAX                 ((1 << DAC63202W_DAC_BITS) - 1)

/* -------------------------------------------------------------- */
/* TPA2028D1 register map                                          */
/* -------------------------------------------------------------- */

#define TPA2028D1_REG_FUNC_CTRL           0x01  /**< EN, SWS, FAULT, Thermal, NG_EN */
#define TPA2028D1_REG_AGC_ATTACK          0x02  /**< Attack time [5:0] */
#define TPA2028D1_REG_AGC_RELEASE         0x03  /**< Release time [5:0] */
#define TPA2028D1_REG_AGC_HOLD            0x04  /**< Hold time [5:0] */
#define TPA2028D1_REG_AGC_GAIN            0x05  /**< Fixed gain [5:0], two's complement */
#define TPA2028D1_REG_AGC_CTRL1           0x06  /**< Limiter disable, noise gate, limiter level */
#define TPA2028D1_REG_AGC_CTRL2           0x07  /**< Max gain [7:4], compression ratio [1:0] */

/**
 * @brief  Output limiter level init() programs (AGC_CTRL1[4:0]).
 *
 * Level n is -6.5 + 0.5 n dBV (SLOS660C Table 11). 19 = 3.0 dBV:
 * 1.41 V rms, 2.0 V peak, 0.25 W into 8 ohm (0.5 W into 4 ohm). That
 * is within the rating of common 0.25 W+ micro speakers, and 2.0 V
 * peak stays below the tile's 2.5 V minimum V+, so the limited output
 * never clips at any legal supply (the datasheet's second limiter
 * constraint, §9.3.1). The chip's own default is 26 (6.5 dBV, 3.0 V
 * peak, 0.56 W into 8 ohm).
 */
#define DRIVE_A_2_LIMITER_DEFAULT         19
/** @brief  Highest limiter level: 31 = 9 dBV, 3.99 V peak, 0.99 W into 8 ohm. */
#define DRIVE_A_2_LIMITER_MAX             31

/* -------------------------------------------------------------- */
/* DAC gain / reference selection                                  */
/* -------------------------------------------------------------- */

/**
 * @brief  DAC output voltage gain setting.
 *
 * Maps directly to VOUT-GAIN-X bits[12:10] in DAC-X-VOUT-CMP-CONFIG.
 * Internal reference gains require EN-INT-REF = 1 in COMMON-CONFIG.
 */
typedef enum {
    DRIVE_A_2_GAIN_1X_EXT   = 0,  /**< 1x, external VREF pin */
    DRIVE_A_2_GAIN_1X_VDD   = 1,  /**< 1x, VDD as reference (default) */
    DRIVE_A_2_GAIN_1P5X_INT = 2,  /**< 1.5x, internal 1.21V → 0–1.815V */
    DRIVE_A_2_GAIN_2X_INT   = 3,  /**< 2x, internal 1.21V → 0–2.42V */
    DRIVE_A_2_GAIN_3X_INT   = 4,  /**< 3x, internal 1.21V → 0–3.63V */
    DRIVE_A_2_GAIN_4X_INT   = 5,  /**< 4x, internal 1.21V → 0–4.84V */
} drive_a_2_gain_t;

/* -------------------------------------------------------------- */
/* Waveform generation                                             */
/* -------------------------------------------------------------- */

/**
 * @brief  Built-in waveform shapes for the DAC function generator.
 *
 * Maps to FUNC-CONFIG-X bits[10:8] in DAC-X-FUNC-CONFIG.
 * Use with set_waveform() / start_waveform() / stop_waveform().
 */
typedef enum {
    DRIVE_A_2_WAVE_TRIANGLE = 0,  /**< Triangular wave */
    DRIVE_A_2_WAVE_SAWTOOTH = 1,  /**< Sawtooth (ramp up) */
    DRIVE_A_2_WAVE_INV_SAW  = 2,  /**< Inverse sawtooth (ramp down) */
    DRIVE_A_2_WAVE_SINE     = 4,  /**< Sine wave */
    DRIVE_A_2_WAVE_OFF      = 7,  /**< Function generation disabled */
} drive_a_2_wave_t;

/**
 * @brief  Slew-rate control (time per code step) for the DAC.
 *
 * Maps to SLEW-RATE-X bits[3:0] in DAC-X-FUNC-CONFIG. Combined with
 * CODE-STEP-X this sets the slewed-update ramp time (set_slew_rate /
 * set_code_step) and the triangle / sawtooth frequency. The sine
 * generator uses SLEW-RATE-X alone: f = 1 / (24 × time_step)
 * (SLASF73A Eq 8), e.g. 41_US → 1029 Hz.
 *
 * Time per step, SLASF73A Table 6-30 (linear-slew mode). NONE is
 * invalid for waveform generation.
 */
typedef enum {
    DRIVE_A_2_SLEW_NONE     = 0x0,  /**< No slew (default): immediate update. Invalid for waveforms */
    DRIVE_A_2_SLEW_4_US     = 0x1,  /**< 4 µs/step (sine 10417 Hz) */
    DRIVE_A_2_SLEW_8_US     = 0x2,  /**< 8 µs/step (sine 5208 Hz) */
    DRIVE_A_2_SLEW_12_US    = 0x3,  /**< 12 µs/step (sine 3472 Hz) */
    DRIVE_A_2_SLEW_18_US    = 0x4,  /**< 18 µs/step (sine 2315 Hz) */
    DRIVE_A_2_SLEW_27_US    = 0x5,  /**< 27.04 µs/step (sine 1541 Hz) */
    DRIVE_A_2_SLEW_41_US    = 0x6,  /**< 40.48 µs/step (sine 1029 Hz) */
    DRIVE_A_2_SLEW_61_US    = 0x7,  /**< 60.72 µs/step (sine 686 Hz) */
    DRIVE_A_2_SLEW_91_US    = 0x8,  /**< 91.12 µs/step (sine 457 Hz) */
    DRIVE_A_2_SLEW_137_US   = 0x9,  /**< 136.72 µs/step (sine 305 Hz) */
    DRIVE_A_2_SLEW_239_US   = 0xA,  /**< 239.2 µs/step (sine 174 Hz) */
    DRIVE_A_2_SLEW_419_US   = 0xB,  /**< 418.64 µs/step (sine 99.5 Hz) */
    DRIVE_A_2_SLEW_733_US   = 0xC,  /**< 732.56 µs/step (sine 56.9 Hz) */
    DRIVE_A_2_SLEW_1282_US  = 0xD,  /**< 1282 µs/step (sine 32.5 Hz) */
    DRIVE_A_2_SLEW_2564_US  = 0xE,  /**< 2563.96 µs/step (sine 16.3 Hz) */
    DRIVE_A_2_SLEW_5128_US  = 0xF,  /**< 5127.92 µs/step (sine 8.1 Hz) */
} drive_a_2_slew_t;

/**
 * @brief  Code-step size (LSBs per slewed update tick) for the DAC.
 *
 * Maps to CODE-STEP-X bits[6:4] in DAC-X-FUNC-CONFIG. With slew
 * enabled, the DAC moves toward DAC-X-DATA (or between MARGIN-LOW
 * and MARGIN-HIGH for waveforms) by this many LSBs per slew tick.
 */
typedef enum {
    DRIVE_A_2_STEP_1_LSB   = 0x0,  /**< 1 LSB per step (default, finest) */
    DRIVE_A_2_STEP_2_LSB   = 0x1,  /**< 2 LSB per step */
    DRIVE_A_2_STEP_3_LSB   = 0x2,  /**< 3 LSB per step */
    DRIVE_A_2_STEP_4_LSB   = 0x3,  /**< 4 LSB per step */
    DRIVE_A_2_STEP_6_LSB   = 0x4,  /**< 6 LSB per step */
    DRIVE_A_2_STEP_8_LSB   = 0x5,  /**< 8 LSB per step */
    DRIVE_A_2_STEP_16_LSB  = 0x6,  /**< 16 LSB per step */
    DRIVE_A_2_STEP_32_LSB  = 0x7,  /**< 32 LSB per step (coarsest) */
} drive_a_2_step_t;

/**
 * @brief  Phase offset for the function generator (sine / triangle).
 *
 * Maps to PHASE-SEL-X bits[12:11] in DAC-X-FUNC-CONFIG. Useful for
 * driving two channels in quadrature (e.g., 0° + 90°) for stereo
 * effects or two-phase actuators.
 */
typedef enum {
    DRIVE_A_2_PHASE_0    = 0,  /**< 0° (default) */
    DRIVE_A_2_PHASE_120  = 1,  /**< 120° */
    DRIVE_A_2_PHASE_240  = 2,  /**< 240° */
    DRIVE_A_2_PHASE_90   = 3,  /**< 90° (quadrature) */
} drive_a_2_phase_t;

/* -------------------------------------------------------------- */
/* Amplifier AGC / DRC                                             */
/* -------------------------------------------------------------- */

/**
 * @brief  Amplifier compression ratio.
 *
 * Maps to bits[1:0] of TPA2028D1 register 0x07.
 */
typedef enum {
    DRIVE_A_2_COMP_1_1 = 0,  /**< 1:1 — compression off */
    DRIVE_A_2_COMP_2_1 = 1,  /**< 2:1 */
    DRIVE_A_2_COMP_4_1 = 2,  /**< 4:1 (default) */
    DRIVE_A_2_COMP_8_1 = 3,  /**< 8:1 */
} drive_a_2_comp_t;

/**
 * @brief  Full AGC/DRC configuration for the TPA2028D1 amplifiers.
 *
 * Pass to tile_drive_a_2_amp_set_agc() to configure all AGC parameters.
 * Every field is written as given: a zeroed struct is NOT the chip's
 * power-on setup (that is compression 4:1, fixed gain 6 dB, max gain
 * 30 dB, limiter 6.5 dBV, attack 5, release 11, hold 0, noise gate 1;
 * SLOS660C Table 5). With compression 1:1 the fixed gain is only valid
 * from 0 to +30 dB (clamped). The output limiter stays ON unless
 * `limiter_disable` is set, which the chip honours only with
 * compression 1:1 (Table 11). Note that a zeroed limiter_level is the
 * lowest level (-6.5 dBV, 0.03 W into 8 ohm), not the default.
 */
typedef struct {
    uint8_t compression;     /**< drive_a_2_comp_t. 0 = 1:1 (off). */
    int8_t  fixed_gain_db;   /**< -28 to +30 dB (0 to +30 with compression 1:1). Clamped. */
    uint8_t max_gain_db;     /**< 18 to 30 dB. 0 = 18 dB (use 30 for default). */
    uint8_t limiter_level;   /**< 0–31 (0.5 dB steps from -6.5 dBV to 9 dBV; 26 = 6.5 dBV chip default). */
    uint8_t attack;          /**< 0–63 (0.1067 ms per step). 0 = fastest. */
    uint8_t release;         /**< 0–63 (0.0137 s per step). 0 = fastest. */
    uint8_t hold;            /**< 0–63 (0.0137 s per step). 0 = disabled. */
    uint8_t noise_gate;      /**< 0–3 (0=1mV, 1=4mV, 2=10mV, 3=20mV rms). */
    uint8_t limiter_disable; /**< 1 = limiter off (compression 1:1 only). UNSAFE for small speakers. 0 = on. */
} drive_a_2_agc_cfg_t;

/* -------------------------------------------------------------- */
/* Tier-2 channel selector                                         */
/* -------------------------------------------------------------- */

/**
 * @brief  Logical channel selector for the tier-2 runtime helpers
 *         (`play_tone`, `set_volume_pct`, `mute`, …).
 *
 * Channel 0 maps to the DAC's VOUT0 → left amp; channel 1 to VOUT1
 * → right amp. The two TPA2028D1 amps share I²C address 0x58 (see
 * the per-amp constraint at the top of this file), so any
 * amp-side write actually lands on both physical amps regardless
 * of which logical channel was requested. Use BOTH when you want
 * symmetric stereo behaviour and don't need the channel
 * distinction at the API level.
 */
typedef enum {
    DRIVE_A_2_CH_LEFT  = 0,  /**< DAC0 / left amp logically */
    DRIVE_A_2_CH_RIGHT = 1,  /**< DAC1 / right amp logically */
    DRIVE_A_2_CH_BOTH  = 2,  /**< Both DAC channels (amp writes still hit both) */
} drive_a_2_channel_t;

/* -------------------------------------------------------------- */
/* Init configuration                                              */
/* -------------------------------------------------------------- */

/**
 * @brief  Optional configuration for tile_drive_a_2_init().
 *
 * Pass NULL for defaults: 1x external-VREF gain on both channels (the
 * tile ties VREF to V+), V+ taken as 3.3 V, 6 dB amp gain, AGC
 * compression off, output limiter on at DRIVE_A_2_LIMITER_DEFAULT.
 */
typedef struct {
    uint8_t  gain;           /**< DAC gain (drive_a_2_gain_t). Default: DRIVE_A_2_GAIN_1X_EXT. */
    int8_t   amp_gain_db;    /**< Amplifier fixed gain in dB (0 to +30; init turns compression off). 0 = default 6 dB. */
    uint16_t vdd_mv;         /**< Tile V+ in mV (2500–5500), the DAC's reference in the 1x gains. 0 = default 3300. */
} drive_a_2_cfg_t;

/* -------------------------------------------------------------- */
/* Public API — Lifecycle                                          */
/* -------------------------------------------------------------- */

/**
 * @brief  Check whether a DAC63202W is present on the bus.
 *
 * @param  hal       Platform HAL handle
 * @param  instance  Instance index (see mapping table)
 * @return 1 if device ACKs (I2C) or responds to WHO_AM_I (SPI), 0 otherwise
 */
uint8_t tile_drive_a_2_find(tiles_pal_t *hal, uint8_t instance);

/**
 * @brief  Initialize the Drive.A.2 tile.
 *
 * Verifies DEVICE-ID, powers up both VOUT channels at mid-scale,
 * configures gain, and (in I2C mode) probes and configures the
 * amplifiers: shutdown, fixed gain, compression 1:1, output limiter on
 * at DRIVE_A_2_LIMITER_DEFAULT with the chip-default attack / release /
 * hold times, then wake. Init starts unmuted: it clears the mute state.
 *
 * @param  hal       Platform HAL handle (see core_tiles.h).
 * @param  instance  Instance index (I2C: address variant, SPI: CS index)
 * @param  tile      Pointer to tile handle (populated by this function)
 * @param  cfg       Optional config, or NULL for defaults (1x ext VREF, 6 dB amp)
 */
void tile_drive_a_2_init(tiles_pal_t *hal, uint8_t instance,
                         tile_t *tile, const drive_a_2_cfg_t *cfg);

/**
 * @brief  Enter low-power sleep.
 * @studio expose category=tile name=sleep section=lifecycle
 *
 * Powers down both DAC VOUT channels (Hi-Z) and puts the amplifiers
 * into software shutdown.
 */
void tile_drive_a_2_sleep(tile_t *tile);

/**
 * @brief  Wake from sleep.
 * @studio expose category=tile name=wake section=lifecycle
 *
 * Powers up both VOUT channels, parks them at mid-scale, restores the
 * cached DAC gains, then brings the amplifiers back to the state they
 * were in before sleep: a pair that was muted (mute()) or disabled
 * (amp_disable()) stays in software shutdown; otherwise it is enabled
 * once the DAC has settled.
 */
void tile_drive_a_2_wake(tile_t *tile);

/**
 * @brief  Perform a software reset of the DAC.
 *
 * All DAC registers return to defaults. tile->state becomes TILE_STATE_NONE;
 * call init() again to reconfigure.
 */
void tile_drive_a_2_reset(tile_t *tile);

/* -------------------------------------------------------------- */
/* Public API — DAC output                                         */
/* -------------------------------------------------------------- */

/**
 * @brief  Set a DAC channel output by raw 12-bit code (0–4095).
 * @studio expose category=tile name=set section=runtime
 *
 * @param  channel  0 or 1
 * @param  value    12-bit DAC code (0 = 0V, 4095 = full-scale)
 */
void tile_drive_a_2_set(tile_t *tile, uint8_t channel, uint16_t value);

/**
 * @brief  Set a DAC channel output in millivolts.
 * @studio expose category=tile name=set_mv section=runtime
 *
 * Computes the DAC code from this channel's gain: full scale is V+
 * (set_supply_mv() / cfg->vdd_mv) in the 1x gains (the VDD reference,
 * or the VREF pin, which the tile ties to V+), or 1.21 V times 1.5,
 * 2, 3 or 4 with the internal reference (SLASF73A Eq 1-3). The output
 * cannot exceed V+, so requests above it are clamped to V+.
 *
 * @param  channel  0 or 1
 * @param  mv       Desired output in millivolts
 */
void tile_drive_a_2_set_mv(tile_t *tile, uint8_t channel, uint16_t mv);

/**
 * @brief  Read back the current DAC code for a channel.
 * @studio expose category=tile name=get returns=int section=runtime
 *
 * @param  channel  0 or 1
 * @return 12-bit DAC code currently loaded
 */
uint16_t tile_drive_a_2_get(tile_t *tile, uint8_t channel);

/* -------------------------------------------------------------- */
/* Public API — DAC configuration                                  */
/* -------------------------------------------------------------- */

/**
 * @brief  Set the voltage output gain for a DAC channel.
 * @studio expose category=tile name=set_gain section=config
 *
 * Automatically enables the internal reference when an internal-reference
 * gain is selected, and records the gain for this channel so set_mv()
 * uses the right full scale (each channel keeps its own).
 *
 * @param  channel  0 or 1
 * @param  gain     One of the drive_a_2_gain_t values
 */
void tile_drive_a_2_set_gain(tile_t *tile, uint8_t channel,
                             drive_a_2_gain_t gain);

/**
 * @brief  Tell the driver the tile's V+ supply voltage.
 * @studio expose category=tile name=set_supply_mv section=config
 * @studio control mv label="V+ supply" tier=advanced default=3300 unit=mV
 *
 * The DAC63202W's VDD is the tile's V+ (2.5–5.5 V). In the 1x gains
 * the DAC's full scale is V+, so set_mv() needs it to pick the right
 * code; the internal-reference gains need it to clamp requests above
 * V+. No bus traffic. Default 3300 mV (or cfg->vdd_mv).
 *
 * @param  mv  [2500..5500] V+ in millivolts (clamped).
 */
void tile_drive_a_2_set_supply_mv(tile_t *tile, uint16_t mv);

/* -------------------------------------------------------------- */
/* Public API — Waveform generation                                */
/* -------------------------------------------------------------- */

/**
 * @brief  Configure the waveform shape for a channel.
 * @studio expose category=tile name=set_waveform section=runtime
 *
 * Sets FUNC-CONFIG-X in DAC-X-FUNC-CONFIG. Triangle and sawtooth
 * waves oscillate between DAC-X-MARGIN-LOW and DAC-X-MARGIN-HIGH at
 * the configured slew rate and code step. The sine uses 24 fixed
 * codes (0x19A to 0xE66, about 80 % of full scale) and ignores the
 * margins and code step. Set a non-zero slew rate first, then call
 * start_waveform() to begin output.
 *
 * @param  channel  0 or 1
 * @param  wave     Waveform shape
 */
void tile_drive_a_2_set_waveform(tile_t *tile, uint8_t channel,
                                 drive_a_2_wave_t wave);

/**
 * @brief  Start waveform generation on a channel.
 * @studio expose category=tile name=start_waveform section=runtime
 *
 * Sets START-FUNC-X in COMMON-DAC-TRIG (read-modify-write, so the
 * other channel's generator keeps running).
 *
 * @param  channel  0 or 1
 */
void tile_drive_a_2_start_waveform(tile_t *tile, uint8_t channel);

/**
 * @brief  Stop waveform generation on a channel.
 * @studio expose category=tile name=stop_waveform section=runtime
 *
 * Clears START-FUNC-X and sets FUNC-CONFIG-X to "disabled".
 *
 * @param  channel  0 or 1
 */
void tile_drive_a_2_stop_waveform(tile_t *tile, uint8_t channel);

/**
 * @brief  Set the slew rate (time per code step) for a DAC channel.
 * @studio expose category=tile name=set_slew_rate section=config
 *
 * Programs SLEW-RATE-X bits[3:0] in DAC-X-FUNC-CONFIG. Affects both
 * slewed direct-output updates and the on-chip function generator's
 * frequency. The default is no-slew — outputs settle as fast as the
 * analog stage allows. Use a slower slew to soften zero-crossings
 * for capacitive / inductive loads, or to dial in a target waveform
 * frequency together with set_code_step() and set_margins().
 *
 * @param  channel  0 or 1
 * @param  slew     Slew-rate code (drive_a_2_slew_t value 0–15)
 */
void tile_drive_a_2_set_slew_rate(tile_t *tile, uint8_t channel,
                                  drive_a_2_slew_t slew);

/**
 * @brief  Set the code step (LSBs per slew tick) for a DAC channel.
 * @studio expose category=tile name=set_code_step section=config
 *
 * Programs CODE-STEP-X bits[6:4] in DAC-X-FUNC-CONFIG. Larger steps
 * give faster ramps / higher waveform frequencies at the cost of
 * coarser resolution.
 *
 * @param  channel  0 or 1
 * @param  step     Code-step code (drive_a_2_step_t value 0–7)
 */
void tile_drive_a_2_set_code_step(tile_t *tile, uint8_t channel,
                                  drive_a_2_step_t step);

/**
 * @brief  Set the upper / lower bounds for waveform & window-comparator modes.
 * @studio expose category=tile name=set_margins section=config
 *
 * Writes DAC-X-MARGIN-HIGH and DAC-X-MARGIN-LOW. The function
 * generator oscillates between these levels, and they also serve
 * as the thresholds for the chip's window / hysteresis comparator
 * modes. Values are 12-bit DAC codes; high must be > low.
 *
 * @param  channel  0 or 1
 * @param  low      12-bit DAC code for the lower bound (0–4095)
 * @param  high     12-bit DAC code for the upper bound (0–4095)
 */
void tile_drive_a_2_set_margins(tile_t *tile, uint8_t channel,
                                uint16_t low, uint16_t high);

/**
 * @brief  Set the phase offset for the function generator.
 * @studio expose category=tile name=set_phase section=config
 *
 * Programs PHASE-SEL-X bits[12:11] in DAC-X-FUNC-CONFIG. Take effect
 * the next time start_waveform() is called. Use phase = 90° on one
 * channel and 0° on the other to drive a quadrature pair.
 *
 * @param  channel  0 or 1
 * @param  phase    Phase offset (drive_a_2_phase_t)
 */
void tile_drive_a_2_set_phase(tile_t *tile, uint8_t channel,
                              drive_a_2_phase_t phase);

/**
 * @brief  Configure the parametric on-chip waveform generator.
 *
 * Convenience helper that sets up the waveform shape, full-scale
 * margins (0 → 4095), code step, and slew rate in one call. The
 * resulting frequency is (SLASF73A Eq 6-8):
 *
 *   f_triangle = 1 / (2 × time_step × ceil((margin_high − margin_low) / code_step))
 *   f_sawtooth = 1 / (    time_step × ceil((margin_high − margin_low + 1) / code_step))
 *   f_sine     = 1 / (24 × time_step)
 *
 * Call start_waveform() to begin output once configured. For
 * fine-grained control, call set_margins() / set_code_step() /
 * set_slew_rate() / set_waveform() / set_phase() individually.
 *
 * @studio expose category=tile name=set_waveform_params section=config
 * @param  channel  0 or 1
 * @param  wave     Waveform shape (drive_a_2_wave_t)
 * @param  step     Code-step (drive_a_2_step_t)
 * @param  slew     Time per step (drive_a_2_slew_t)
 */
void tile_drive_a_2_set_waveform_params(tile_t *tile, uint8_t channel,
                                        drive_a_2_wave_t wave,
                                        drive_a_2_step_t step,
                                        drive_a_2_slew_t slew);

/* -------------------------------------------------------------- */
/* Public API — Amplifier control                                  */
/* -------------------------------------------------------------- */

/**
 * @brief  Set the amplifier fixed gain.
 * @studio expose category=tile name=amp_set_gain section=runtime
 * @studio control gain_db label="Amplifier gain" tier=basic default=6 unit=dB
 * @studio require expr="gain_db >= 0" message="With AGC compression off (init's default) the amplifier's gain is only specified from 0 to +30 dB. Use 0 dB or more, or turn compression on first."
 *
 * Both TPA2028D1 amplifiers share I2C address 0x58, so this write
 * affects both channels simultaneously.  No-op in SPI mode.
 *
 * @note  init() turns AGC compression off (1:1), and with compression
 *        off the TPA2028D1 fixed gain is only specified from 0 to
 *        +30 dB (SLOS660C Table 10), so negative gains are raised to
 *        0 dB. Negative gains need compression on (amp_set_agc()).
 *        The output limiter still caps the output level.
 *
 * @param  gain_db   [-28..30] Gain in dB. Clamped if out of range.
 */
void tile_drive_a_2_amp_set_gain(tile_t *tile, int8_t gain_db);

/**
 * @brief  Read the current amplifier fixed gain.
 * @studio expose category=tile name=amp_get_gain returns=int section=runtime
 *
 * @return Gain in dB (-28 to +30), or 0 if amp not available
 */
int8_t tile_drive_a_2_amp_get_gain(tile_t *tile);

/**
 * @brief  Enable the amplifiers (clear software shutdown).
 * @studio expose category=tile name=amp_enable section=runtime
 *
 * While the tile is asleep this only records the request, and wake()
 * applies it once the DAC has settled at mid-scale. No-op in SPI mode.
 */
void tile_drive_a_2_amp_enable(tile_t *tile);

/**
 * @brief  Disable the amplifiers (enter software shutdown).
 * @studio expose category=tile name=amp_disable section=runtime
 *
 * The amplifiers stay off across sleep() / wake(). No-op in SPI mode.
 */
void tile_drive_a_2_amp_disable(tile_t *tile);

/**
 * @brief  Configure the full AGC/DRC parameters.
 * @studio expose category=tile name=amp_set_agc section=runtime
 *
 * Writes all AGC registers (attack, release, hold, fixed gain,
 * limiter, compression, noise gate, max gain).  Affects both amps.
 * No-op in SPI mode. The compression register is written before the
 * limiter register, because the limiter can only be disabled while
 * compression is 1:1 (SLOS660C Table 11). The limiter stays on unless
 * cfg->limiter_disable is set. The compression ratio also sets the
 * valid fixed-gain range that amp_set_gain() and set_volume_pct() use.
 *
 * @param  cfg   AGC configuration
 */
void tile_drive_a_2_amp_set_agc(tile_t *tile, const drive_a_2_agc_cfg_t *cfg);

/**
 * @brief  Set, or disable, the amplifiers' output limiter.
 * @studio expose category=tile name=amp_set_limiter section=config
 * @studio control enabled label="Output limiter" tier=advanced type=bool default=1
 * @studio control level label="Limiter level" tier=advanced default=19
 * @studio require expr="enabled == 1" message="With the output limiter off, full gain on a loud signal drives a clipped square wave at the V+ rail (about 1.4 W into 8 ohm at 3.3 V), which can burn a small speaker. Keep it on unless the speaker is rated for that."
 *
 * The limiter caps the amplifier output at level n = -6.5 + 0.5 n dBV
 * (SLOS660C Table 11; power figures are for 8 ohm): 0 = 0.67 V peak,
 * 0.03 W; 19 = 2.0 V peak, 0.25 W (init's default); 26 = 3.0 V peak,
 * 0.56 W (chip default); 31 = 3.99 V peak, 0.99 W. A level whose peak
 * is above V+ cannot be reached, so the output clips first. Read-
 * modify-write of AGC_CTRL1 (the noise-gate bits are kept). Affects
 * both amps. No-op in SPI mode.
 *
 * @warning Disabling the limiter (enabled = 0), or a level above the
 *          speaker's rating, is unsafe for small speakers. The chip
 *          only honours a disable while AGC compression is 1:1 (init's
 *          default); with compression on the driver keeps the limiter
 *          enabled and only sets the level.
 *
 * @param  enabled  1 = limiter on (default), 0 = off (unsafe)
 * @param  level    [0..31] Limiter level (clamped)
 */
void tile_drive_a_2_amp_set_limiter(tile_t *tile, uint8_t enabled, uint8_t level);

/**
 * @brief  Read the amplifier status register.
 * @studio expose category=tile name=amp_read_status returns=int section=runtime
 *
 * Check bit 3 (FAULT) for short-circuit and bit 2 (Thermal) for
 * over-temperature.  Write 0 to the respective bit to clear.
 *
 * @return Raw register 0x01 value, or 0 if amp not available
 */
uint8_t tile_drive_a_2_amp_read_status(tile_t *tile);

/* -------------------------------------------------------------- */
/* Public API — Status                                             */
/* -------------------------------------------------------------- */

/**
 * @brief  Read the DAC GENERAL-STATUS register.
 * @studio expose category=tile name=read_status returns=int section=runtime
 *
 * Contains DEVICE-ID, VERSION-ID, NVM CRC status, and DAC busy flags.
 *
 * @return 16-bit GENERAL-STATUS value
 */
uint16_t tile_drive_a_2_read_status(tile_t *tile);

/* -------------------------------------------------------------- */
/* Public API — NVM (shadow flash for power-on defaults)           */
/* -------------------------------------------------------------- */

/**
 * @brief  Save the DAC's current register state into shadow NVM.
 * @studio expose category=tile name=nvm_save section=config
 *
 * Triggers NVM-PROG in COMMON-TRIGGER. The DAC's user-programmable
 * registers (gain, margins, slew, waveform shape, COMMON-CONFIG,
 * etc. — see datasheet "highlighted gray" rows) are committed to
 * non-volatile storage and become the new power-on defaults.
 *
 * Blocking: holds the bus busy for the NVM write cycle. Limited
 * write endurance — TI specs ~1000 cycles. Use only for one-time
 * factory tuning, not for runtime configuration storage.
 *
 */
void tile_drive_a_2_nvm_save(tile_t *tile);

/**
 * @brief  Reload all DAC registers from shadow NVM.
 * @studio expose category=tile name=nvm_reload section=config
 *
 * Triggers NVM-RELOAD in COMMON-TRIGGER. Restores the saved
 * power-on configuration without a full reset — equivalent to
 * the load that happens automatically on POR. Blocks until the
 * reload completes, then re-reads both channels' VOUT-GAIN so
 * set_mv() follows whatever gain the NVM held.
 *
 */
void tile_drive_a_2_nvm_reload(tile_t *tile);

/* -------------------------------------------------------------- */
/* Public API — Raw register access (escape hatches)               */
/* -------------------------------------------------------------- */

/**
 * @brief  Read any 16-bit DAC63202W register.
 * @studio expose category=tile name=read_reg returns=int section=advanced
 *
 * Escape hatch for advanced users wanting to touch registers the
 * driver doesn't expose. Caller is responsible for not bricking
 * the chip — most useful registers have typed setters above.
 *
 * @param  reg   Register address (7-bit)
 * @return 16-bit register value
 */
uint16_t tile_drive_a_2_read_reg(tile_t *tile, uint8_t reg);

/**
 * @brief  Write any 16-bit DAC63202W register.
 * @studio expose category=tile name=write_reg section=advanced
 *
 * @param  reg    Register address (7-bit)
 * @param  value  16-bit value to write (big-endian on the wire)
 */
void tile_drive_a_2_write_reg(tile_t *tile, uint8_t reg, uint16_t value);

/* ============================================================== */
/* Runtime — tier-2 idiomatic helpers                              */
/*                                                                  */
/* These compose the tier-1 surface above into "do the thing the   */
/* user wants to do" calls. Mode transitions, sample synthesis,    */
/* and amp gating are handled internally so a caller can play a    */
/* tone or sweep without reading the DAC63202W datasheet.          */
/*                                                                  */
/* Channel semantics: LEFT/RIGHT pick a single DAC channel; BOTH   */
/* drives both DAC channels. The TPA2028D1 amp pair shares an I²C  */
/* address, so amp-side writes (mute, volume, enable) affect both  */
/* physical amps regardless of the requested channel.              */
/* ============================================================== */

/**
 * @brief  Play a sine tone on `channel` for `ms` milliseconds.
 * @studio expose category=tile name=play_tone section=runtime
 *
 * Configures DAC margins for full-scale swing, sets a sine
 * waveform, and uses the on-chip parametric generator together
 * with a software-tuned slew rate / code step to approximate
 * `freq_hz`. The chip's sine generator runs at 1 / (24 × slew
 * step), so only 15 pitches exist (10417, 5208, 3472, 2315, 1541,
 * 1029, 686, 457, 305, 174, 99.5, 56.9, 32.5, 16.3, 8.1 Hz); the
 * nearest one is played. The sine spans about 80 % of the DAC's
 * range. The amplifier is unmuted for the duration of the tone and
 * restored to its prior mute state on return.
 *
 * @note  Blocks for `ms` milliseconds via `hal->delay_ms`.
 *
 * @param  channel  DRIVE_A_2_CH_LEFT, _RIGHT, or _BOTH
 * @param  freq_hz  [8..10417] Tone frequency in Hz (rounded to the nearest generator pitch)
 * @param  ms       Duration in milliseconds
 */
void tile_drive_a_2_play_tone(tile_t *tile, drive_a_2_channel_t channel,
                              uint16_t freq_hz, uint16_t ms);

/**
 * @brief  Hold the channel at silence (DAC mid-scale) for `ms` ms.
 * @studio expose category=tile name=play_silence section=runtime
 *
 * Stops any active waveform on `channel`, parks the DAC code at
 * mid-scale (= zero differential at the amp input), and blocks
 * for `ms` milliseconds. Useful to insert a precise gap between
 * tones without toggling the amp shutdown — the amp stays alive
 * but draws minimal idle current with no signal.
 *
 * @note  Blocks for `ms` milliseconds via `hal->delay_ms`.
 *
 * @param  channel  DRIVE_A_2_CH_LEFT, _RIGHT, or _BOTH
 * @param  ms       Duration in milliseconds
 */
void tile_drive_a_2_play_silence(tile_t *tile, drive_a_2_channel_t channel,
                                 uint16_t ms);

/**
 * @brief  Linear frequency sweep on `channel` from `start_hz` to `end_hz`.
 * @studio expose category=tile name=play_chirp section=runtime
 *
 * Software sine-LUT chirp delivered through `set` (raw DAC code)
 * at ~8 kHz update rate. Frequency advances linearly across `ms`.
 * For ascending sweeps pass start < end; for descending, start >
 * end. The amp is unmuted for the duration and restored to its
 * prior mute state on return.
 *
 * @note  Blocks for `ms` milliseconds via `hal->delay_ms`.
 *
 * @param  channel   DRIVE_A_2_CH_LEFT, _RIGHT, or _BOTH
 * @param  start_hz  Initial frequency in Hz
 * @param  end_hz    Final frequency in Hz
 * @param  ms        Sweep duration in milliseconds
 */
void tile_drive_a_2_play_chirp(tile_t *tile, drive_a_2_channel_t channel,
                               uint16_t start_hz, uint16_t end_hz,
                               uint16_t ms);

/**
 * @brief  Map a 0–100 percent volume to amplifier fixed gain.
 * @studio expose category=tile name=set_volume_pct section=runtime
 *
 * Linear-in-dB mapping from `pct` onto the fixed-gain range the
 * TPA2028D1 specifies for the active compression ratio (SLOS660C
 * Table 10):
 *
 *   compression 1:1 (init's default):  gain_db = (pct × 30) / 100
 *   compression 2:1, 4:1 or 8:1:       gain_db = -28 + (pct × 58) / 100
 *
 * With compression off, 0 % → 0 dB (the lowest valid gain, not
 * silence; use mute() for silence), 50 % → +15 dB, 100 % → +30 dB.
 * The output limiter still caps the level. The
 * mapping is intentionally linear-in-dB rather than perceptually
 * weighted; for finer dB control use @ref
 * tile_drive_a_2_amp_set_gain directly. Because both amps share
 * an I²C address, the channel argument is accepted for symmetry
 * with the rest of the tier-2 API but the write lands on both
 * physical amps.
 *
 * @param  channel  DRIVE_A_2_CH_LEFT, _RIGHT, or _BOTH (advisory)
 * @param  pct      [0..100] Percent (clamped if out of range)
 */
void tile_drive_a_2_set_volume_pct(tile_t *tile, drive_a_2_channel_t channel,
                                   uint8_t pct);

/**
 * @brief  Mute `channel` by entering amp software shutdown.
 * @studio expose category=tile name=mute section=runtime
 *
 * Stashes the current fixed-gain register so a later @ref
 * tile_drive_a_2_unmute restores it byte-exact, then puts the amp
 * pair into software shutdown (SWS=1). Both physical amps mute
 * regardless of `channel` (shared I²C address). The mute survives
 * sleep() / wake(); only unmute(), amp_enable() or init() clear it.
 *
 * @param  channel  DRIVE_A_2_CH_LEFT, _RIGHT, or _BOTH (advisory)
 */
void tile_drive_a_2_mute(tile_t *tile, drive_a_2_channel_t channel);

/**
 * @brief  Restore audio output after a previous `mute` call.
 * @studio expose category=tile name=unmute section=runtime
 *
 * Re-applies the gain that was active at the time of @ref
 * tile_drive_a_2_mute and clears software shutdown (SWS=0). If
 * the channel was never muted by this driver instance the call
 * still wakes the amp using whatever gain is currently in the
 * register. While the tile is asleep the amp is woken by wake().
 *
 * @param  channel  DRIVE_A_2_CH_LEFT, _RIGHT, or _BOTH (advisory)
 */
void tile_drive_a_2_unmute(tile_t *tile, drive_a_2_channel_t channel);

#endif /* INC_TILE_DRIVE_A_2_H_ */
