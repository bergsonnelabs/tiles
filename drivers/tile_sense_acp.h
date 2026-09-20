/**
 * @file  tile_sense_acp.h
 * @brief TMD3725 ambient-light, RGB colour & proximity sensor.
 *
 * Platform-agnostic driver for the ams/OSRAM TMD3725 ALS/Color/Proximity
 * module on Sense.ACP. Provides RGBC ambient-light sensing (four 16-bit
 * data converters — Clear/Red/Green/Blue) plus an integrated IR-LED
 * proximity engine with an 8-bit result.
 *
 * Key specifications:
 *  - RGBC ambient light: 16-bit Clear/Red/Green/Blue channels
 *  - Programmable ALS gain (1/4/16/64x) and integration time (2.8-719 ms)
 *  - Proximity: integrated, factory-trimmed IR LED, 8-bit output
 *  - Programmable proximity gain (1/2/4/8x) and LED drive (6-192 mA)
 *  - 1.8 V supply and 1.8 V I2C (up to 400 kHz), fixed address 0x39
 *
 * Board note: on Sense.ACP the proximity IR LED anode (LEDA) is fed from
 * an on-tile TPS61099 boost converter, so the LED drive current is drawn
 * from a boosted rail rather than the 1.8 V input directly. The boost is
 * always-on hardware; this driver only sets the LED drive strength via
 * PLDRIVE.
 *
 * Quick start:
 * @code
 *   #include "core.h"
 *   #include "core_tiles.h"
 *   #include "tile_sense_acp.h"
 *
 *   tile_t acp;
 *   tile_sense_acp_init(core_tiles_pal(&core_i2c1), 0, &acp, NULL);
 *
 *   uint16_t clear = tile_sense_acp_get_clear(&acp);
 *   uint8_t  prox  = tile_sense_acp_get_proximity(&acp);
 * @endcode
 *
 * Datasheet: ams TMD3725, "ALS, Color and Proximity Sensor Module"
 * (v2-00, 2023-Mar-21).
 *
 * @studio tile label=Sense.ACP icon=🎨
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=common category="Threshold interrupts (ALS / proximity INT pin)" section=config
 *   Driver-deferred AND definition-gated. The TMD3725 asserts its INT pin
 *   (open-drain, active-low) when the Clear channel or the proximity
 *   result crosses the AILT/AIHT or PILT/PIHT thresholds past the PERS
 *   persistence count. The tile routes INT to pad 7, but Sense-ACP-b.json
 *   does not declare that pad, so Studio cannot wire it as an event
 *   source. Polled reads (get_clear / get_proximity) cover the same ground
 *   for v1 — a DSL comparison yields "near/far" without the INT path.
 *   Closing this needs the tile definition updated to declare pad 7 = INT,
 *   then a follow-up pass to expose the threshold/PERS/INTENAB registers
 *   and a tile event.
 *
 * @studio unsupported severity=advanced category="Illuminance (lux) and correlated colour temperature"
 *   Out of scope for an integer driver. Converting RGBC counts to lux or
 *   CCT requires the device-specific calibration coefficients from the ams
 *   application note plus floating-point maths. This driver exposes the raw
 *   per-channel counts (clear/red/green/blue); compute lux/CCT on the host
 *   where floats and calibration data are available.
 *
 * @studio unsupported severity=niche category="Wait state, proximity offset calibration, IR-channel mux"
 *   Deferred, not gated. The wait feature (WEN/WTIME/WLONG idle timing),
 *   the proximity offset-cancellation engine (POFFSET + CALIB autozero),
 *   and the IR_TO_GREEN mux (route the IR photodiode into the green data
 *   converter) are left at sane power-on defaults and not surfaced. A later
 *   pass can expose them if a use-case needs them.
 *
 * @note All bus I/O is routed through tiles_pal_t function pointers.
 *       This driver contains no platform-specific code.
 */

#ifndef TILE_SENSE_ACP_H
#define TILE_SENSE_ACP_H

#include "tiles.h"

/* ---- Driver version ---- */

#define TILE_SENSE_ACP_VERSION_MAJOR  1
#define TILE_SENSE_ACP_VERSION_MINOR  0
#define TILE_SENSE_ACP_VERSION_PATCH  0

TILES_CHECK_VERSION(1, 0);

/* ---- I2C address ---- */

#define TMD3725_I2C_ADDR    0x39  /**< Fixed 7-bit I2C address (only value). */

/* ---- Register map ---- */

#define TMD3725_REG_ENABLE  0x80  /**< Enables states and functions. */
#define TMD3725_REG_ATIME   0x81  /**< ALS integration time. */
#define TMD3725_REG_PRATE   0x82  /**< Proximity sample time (PTIME). */
#define TMD3725_REG_WTIME   0x83  /**< Wait time between ALS cycles. */
#define TMD3725_REG_AILTL   0x84  /**< ALS interrupt low threshold, low byte. */
#define TMD3725_REG_AILTH   0x85  /**< ALS interrupt low threshold, high byte. */
#define TMD3725_REG_AIHTL   0x86  /**< ALS interrupt high threshold, low byte. */
#define TMD3725_REG_AIHTH   0x87  /**< ALS interrupt high threshold, high byte. */
#define TMD3725_REG_PILT    0x88  /**< Proximity interrupt low threshold. */
#define TMD3725_REG_PIHT    0x8A  /**< Proximity interrupt high threshold. */
#define TMD3725_REG_PERS    0x8C  /**< Interrupt persistence filters. */
#define TMD3725_REG_CFG0    0x8D  /**< Configuration register zero (WLONG). */
#define TMD3725_REG_PCFG0   0x8E  /**< Proximity config zero (pulse len/count). */
#define TMD3725_REG_PCFG1   0x8F  /**< Proximity config one (PGAIN, PLDRIVE). */
#define TMD3725_REG_CFG1    0x90  /**< Configuration register one (AGAIN). */
#define TMD3725_REG_REVID   0x91  /**< Revision ID. */
#define TMD3725_REG_ID      0x92  /**< Device ID. */
#define TMD3725_REG_STATUS  0x93  /**< Device status register. */
#define TMD3725_REG_CDATAL  0x94  /**< Clear channel data, low byte. */
#define TMD3725_REG_RDATAL  0x96  /**< Red channel data, low byte. */
#define TMD3725_REG_GDATAL  0x98  /**< Green channel data, low byte. */
#define TMD3725_REG_BDATAL  0x9A  /**< Blue channel data, low byte. */
#define TMD3725_REG_PDATA   0x9C  /**< Proximity channel data (8-bit). */
#define TMD3725_REG_CFG3    0xAB  /**< Configuration register three. */

/* ---- Device ID ---- */

#define TMD3725_ID_VALUE    0xE4  /**< ID register (0x92) reset/expected value. */

/* ---- ENABLE (0x80) bit masks ---- */

#define TMD3725_EN_PON      (1 << 0)  /**< Power on: internal oscillator + ADCs. */
#define TMD3725_EN_AEN      (1 << 1)  /**< ALS enable. */
#define TMD3725_EN_PEN      (1 << 2)  /**< Proximity enable. */
#define TMD3725_EN_WEN      (1 << 3)  /**< Wait enable. */

/* ---- STATUS (0x93) bit masks ---- */

#define TMD3725_ST_PSAT_AMBIENT  (1 << 0)  /**< AFE saturated, IR-inactive phase. */
#define TMD3725_ST_PSAT_REFLECT  (1 << 1)  /**< AFE saturated, IR-active phase. */
#define TMD3725_ST_CINT          (1 << 3)  /**< Calibration interrupt. */
#define TMD3725_ST_AINT          (1 << 4)  /**< ALS threshold interrupt. */
#define TMD3725_ST_PINT          (1 << 5)  /**< Proximity threshold interrupt. */
#define TMD3725_ST_PSAT          (1 << 6)  /**< Proximity saturation. */
#define TMD3725_ST_ASAT          (1 << 7)  /**< ALS/Color analog saturation. */

/* ---- Enumerations ---- */

/** ALS/Color gain (CFG1 AGAIN[1:0]). */
typedef enum {
    SENSE_ACP_ALS_GAIN_1X  = 0x00,  /**< 1x. */
    SENSE_ACP_ALS_GAIN_4X  = 0x01,  /**< 4x. */
    SENSE_ACP_ALS_GAIN_16X = 0x02,  /**< 16x (default). */
    SENSE_ACP_ALS_GAIN_64X = 0x03,  /**< 64x. */
} sense_acp_als_gain_t;

/** Proximity IR-sensor gain (PCFG1 PGAIN[7:6]). */
typedef enum {
    SENSE_ACP_PROX_GAIN_1X = 0x00,  /**< 1x. */
    SENSE_ACP_PROX_GAIN_2X = 0x01,  /**< 2x. */
    SENSE_ACP_PROX_GAIN_4X = 0x02,  /**< 4x (default). */
    SENSE_ACP_PROX_GAIN_8X = 0x03,  /**< 8x. */
} sense_acp_prox_gain_t;

/* ---- Configuration struct ---- */

/**
 * @brief Optional init-time configuration for Sense.ACP.
 *
 * Pass NULL to tile_sense_acp_init() for defaults: ALS gain 16x,
 * integration ~135 ms (ATIME 0x2F), proximity gain 4x, LED drive 12 mA,
 * both the ALS and proximity engines enabled.
 */
typedef struct {
    uint8_t als_gain;    /**< ALS gain (sense_acp_als_gain_t). Default 16x. */
    uint8_t atime;       /**< ALS integration: (atime+1) x 2.78 ms. Default 0x2F. */
    uint8_t prox_gain;   /**< Proximity gain (sense_acp_prox_gain_t). Default 4x. */
    uint8_t prox_drive_ma; /**< IR LED drive current in mA (6-192). Default 12. */
    uint8_t enable_als;  /**< 1 = enable ALS at init (default), 0 = leave off. */
    uint8_t enable_prox; /**< 1 = enable proximity at init (default), 0 = off. */
} sense_acp_cfg_t;

/* ---- Lifecycle ---- */

/**
 * @brief  Check if a Sense.ACP is present on the bus.
 * @param  hal      Platform HAL handle.
 * @param  instance Address selector — only 0 is valid (fixed address 0x39).
 * @return 1 if the device ACKs and its ID register reads 0xE4, else 0.
 */
uint8_t tile_sense_acp_find(tiles_pal_t *hal, uint8_t instance);

/**
 * @brief  Initialise a Sense.ACP tile.
 *
 * Probes the device, verifies the ID register, programs ALS gain +
 * integration time and proximity gain + LED drive, then powers on the
 * requested engines (PON is asserted together with AEN so the chip's
 * auto-zero runs before the first ALS measurement). Sets tile state to
 * READY on success, ERROR otherwise.
 *
 * @param  hal      Platform HAL handle.
 * @param  instance Address selector — only 0 is valid (fixed address 0x39).
 * @param  tile     Tile handle to initialise.
 * @param  cfg      Configuration (NULL for defaults).
 */
void tile_sense_acp_init(tiles_pal_t *hal, uint8_t instance,
                         tile_t *tile, const sense_acp_cfg_t *cfg);

/**
 * @brief  Power down the sensor (clears PON/AEN/PEN).
 * @studio expose category=tile name=sleep section=lifecycle
 * @param  tile  Initialised tile handle.
 */
void tile_sense_acp_sleep(tile_t *tile);

/**
 * @brief  Resume from sleep, restoring the engines enabled at init.
 * @studio expose category=tile name=wake section=lifecycle
 * @param  tile  Sleeping tile handle.
 */
void tile_sense_acp_wake(tile_t *tile);

/* ---- Configuration ---- */

/**
 * @brief  Set the ALS/Color gain.
 * @studio expose category=tile name=set_als_gain section=config
 * @studio control gain label="Light sensor gain" tier=basic default=SENSE_ACP_ALS_GAIN_16X
 * @param  tile  Initialised tile handle.
 * @param  gain  ALS gain selection (sense_acp_als_gain_t).
 */
void tile_sense_acp_set_als_gain(tile_t *tile, sense_acp_als_gain_t gain);

/**
 * @brief  Set the ALS integration time.
 *
 * @studio expose category=tile name=set_integration_time section=config
 * @studio control atime label="Light integration time" tier=basic default=47 show="(atime + 1) * 2.81" unit=ms
 *
 * Integration time is (atime + 1) steps of nominally 2.8 ms; the maximum ALS
 * count scales with it (1024 per step, saturating at 65535). The datasheet
 * prose says 2.78 ms per step, but its own table (0x3F = 180 ms, 0xFF = 719 ms)
 * and clock arithmetic work out to 2.81 ms, which is what is shown to the user.
 * For example 0x00 = 2.8 ms, 0x2F = ~135 ms (default), 0xFF = ~719 ms.
 *
 * @param  tile   Initialised tile handle.
 * @param  atime  [0..255] Raw ATIME register value.
 */
void tile_sense_acp_set_integration_time(tile_t *tile, uint8_t atime);

/**
 * @brief  Set the proximity IR-sensor gain.
 * @studio expose category=tile name=set_prox_gain section=config
 * @studio control gain label="Proximity sensor gain" tier=advanced default=SENSE_ACP_PROX_GAIN_4X
 * @param  tile  Initialised tile handle.
 * @param  gain  Proximity gain selection (sense_acp_prox_gain_t).
 */
void tile_sense_acp_set_prox_gain(tile_t *tile, sense_acp_prox_gain_t gain);

/**
 * @brief  Set the proximity IR-LED drive current.
 *
 * @studio expose category=tile name=set_prox_drive_ma section=config
 * @studio control ma label="Proximity LED current" tier=advanced default=12
 *
 * The LED current is programmed in ~6 mA steps: i_LED = 6 x (PLDRIVE + 1),
 * so the request is clamped to 6-192 mA and rounded down to the nearest
 * step. Actual current is factory-trimmed to normalise IR intensity, so
 * treat the value as approximate.
 *
 * @param  tile  Initialised tile handle.
 * @param  ma    [6..192] mA Desired LED drive current, in milliamps.
 */
void tile_sense_acp_set_prox_drive_ma(tile_t *tile, uint8_t ma);

/* ---- Ambient light / colour data ---- */

/**
 * @brief  Read the Clear (unfiltered) channel — overall ambient light.
 * @studio expose category=tile name=get_clear returns=int section=runtime
 * @param  tile  Initialised tile handle.
 * @return 16-bit Clear channel count.
 */
uint16_t tile_sense_acp_get_clear(tile_t *tile);

/**
 * @brief  Read the Red channel count.
 * @studio expose category=tile name=get_red returns=int section=runtime
 * @param  tile  Initialised tile handle.
 * @return 16-bit Red channel count.
 */
uint16_t tile_sense_acp_get_red(tile_t *tile);

/**
 * @brief  Read the Green channel count.
 * @studio expose category=tile name=get_green returns=int section=runtime
 * @param  tile  Initialised tile handle.
 * @return 16-bit Green channel count.
 */
uint16_t tile_sense_acp_get_green(tile_t *tile);

/**
 * @brief  Read the Blue channel count.
 * @studio expose category=tile name=get_blue returns=int section=runtime
 * @param  tile  Initialised tile handle.
 * @return 16-bit Blue channel count.
 */
uint16_t tile_sense_acp_get_blue(tile_t *tile);

/* ---- Proximity data ---- */

/**
 * @brief  Read the proximity result (larger = closer object).
 * @studio expose category=tile name=get_proximity returns=int section=runtime
 * @param  tile  Initialised tile handle.
 * @return 8-bit proximity count.
 */
uint8_t tile_sense_acp_get_proximity(tile_t *tile);

/* ---- Status ---- */

/**
 * @brief  Read the STATUS register.
 *
 * @studio expose category=tile name=get_status returns=int section=runtime
 *
 * Use the TMD3725_ST_* masks — e.g. ASAT (ALS saturation), PSAT
 * (proximity saturation), AINT/PINT (threshold interrupts pending).
 *
 * @param  tile  Initialised tile handle.
 * @return Raw STATUS byte.
 */
uint8_t tile_sense_acp_get_status(tile_t *tile);

#endif /* TILE_SENSE_ACP_H */
