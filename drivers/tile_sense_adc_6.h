/**
 * @file   tile_sense_adc_6.h
 * @brief  Six-channel ADC input driver for the Sense.ADC.6 tile.
 * @version 1.0.0
 *
 * Six analog inputs sampled continuously and published as calibrated
 * millivolts over I2C. The tile filters and paces the sampling itself,
 * so the host just reads whenever it likes and always gets a coherent
 * set: all six channels come from the same instant, never a mix of old
 * and new.
 *
 * Channels map to pads in order:
 *
 *   CH0 pad 2   CH1 pad 3   CH2 pad 6
 *   CH3 pad 7   CH4 pad 8   CH5 pad 9
 *
 * Inputs read 0 V to the tile's own supply. Sources up to roughly
 * 30-50 kOhm are fine; above that, add a buffer or an RC at the input.
 *
 * Typical use (Cores SDK):
 * @code
 *   tile_t adc;
 *   tile_sense_adc_6_init(core_tiles_pal(&core_i2c1), 0, &adc, NULL);
 *   while (1) {
 *       if (tile_sense_adc_6_update(&adc)) {
 *           int mv = tile_sense_adc_6_read_mv(&adc, 0);
 *           ...
 *       }
 *       core_delay_ms(10);
 *   }
 * @endcode
 *
 * Output rate and filtering are configurable at runtime. The averaging
 * window doubles as mains rejection, because a boxcar over N periods
 * nulls every multiple of rate/N:
 *
 *   100 Hz, window 1   default, 10 ms average
 *   100 Hz, window 2   20 ms average, nulls 50 Hz
 *   120 Hz, window 2   16.7 ms average, nulls 60 Hz
 *    10 Hz, window 1   100 ms average, nulls both
 *
 * The default (100 Hz, window 1) passes 50 Hz at only -3.9 dB, so pick
 * a rejection setting if the sensor leads can pick up mains.
 *
 * Several tiles share a bus. Each needs its own address, stored on the
 * tile with tile_sense_adc_6_set_address(), which takes effect at its
 * next power-up. Read the settings back any time: they are live values,
 * not a copy of what you asked for.
 *
 * Poll a little faster than the output rate and use the sequence number
 * to spot repeats, since the tile's clock and yours drift apart by a
 * percent or so.
 *
 * @studio tile label=Sense.ADC.6 icon=∿
 *
 * @studio unsupported severity=common category="Raw counts / ratiometric output" section=runtime
 *   Readings are calibrated millivolts only. For a divider powered from
 *   the tile's own rail, a ratio of full scale would cancel supply
 *   variation exactly and be more accurate than millivolts. Needs a
 *   format setting on the tile.
 *
 * @studio unsupported severity=advanced category="Sampling-time control" section=config
 *   The ADC's per-sample acquisition time is fixed. Exposing it would
 *   let high-impedance sources (above ~50 kOhm) settle properly.
 *
 * @studio unsupported severity=advanced category="Data-ready output" section=events
 *   No interrupt line: every pad is taken by the six inputs, the bus and
 *   power, so a host must poll. Reading the sequence number tells you
 *   whether a set is new.
 *
 * @studio unsupported severity=niche category="Synchronized sampling across tiles" section=advanced
 *   Each tile free-runs on its own clock, so several tiles on one bus
 *   drift apart by a percent or so and their samples are not aligned in
 *   time. An I2C general-call latch would fix it.
 *
 * @studio unsupported severity=niche category="Per-channel enable" section=config
 *   All six channels are always sampled. Disabling unused ones would
 *   shorten each sweep and save power.
 */

#ifndef TILE_SENSE_ADC_6_H
#define TILE_SENSE_ADC_6_H

#include "tiles.h"

TILES_CHECK_VERSION(1, 0);

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * Version
 * ================================================================ */

#define TILE_SENSE_ADC_6_VERSION_MAJOR  1
#define TILE_SENSE_ADC_6_VERSION_MINOR  0
#define TILE_SENSE_ADC_6_VERSION_PATCH  0

/* ================================================================
 * Constants
 * ================================================================ */

/** Channels on the tile. */
#define SENSE_ADC_6_CHANNELS      6

/** Default I2C address (instance 0). */
#define SENSE_ADC_6_I2C_ADDR      0x28

/** Address range the I2C spec leaves to devices. */
#define SENSE_ADC_6_ADDR_MIN      0x08
#define SENSE_ADC_6_ADDR_MAX      0x77

/** Limits on the acquisition settings. */
#define SENSE_ADC_6_RATE_MAX      250   /**< output rate, Hz */
#define SENSE_ADC_6_OVERSAMP_MAX  32    /**< sweeps averaged per period */
#define SENSE_ADC_6_WINDOW_MAX    2     /**< periods averaged */

/** Status bits from tile_sense_adc_6_status(). */
#define SENSE_ADC_6_ST_VALID      (1U << 0)  /**< readings are current */
#define SENSE_ADC_6_ST_SAMPLING   (1U << 1)  /**< acquisition running */
#define SENSE_ADC_6_ST_CALIBRATED (1U << 2)  /**< supply reference measured */

/* ================================================================
 * Configuration
 * ================================================================ */

/**
 * @brief  Optional init config. Pass NULL for defaults.
 *
 * Any field left zero keeps the tile's stored setting, so a tile
 * configured once and saved comes up the same way without the host
 * having to restate it.
 */
typedef struct {
    uint8_t address;    /**< override the address for this instance, 0 = default */
    uint8_t rate_hz;    /**< output rate, 0 = leave as stored */
    uint8_t oversamp;   /**< sweeps averaged per period, 0 = leave as stored */
    uint8_t window;     /**< periods averaged (1 or 2), 0 = leave as stored */
} sense_adc_6_cfg_t;

/* ================================================================
 * Lifecycle
 * ================================================================ */

/**
 * @brief  Check whether a tile is present.
 * @param  hal       Tiles HAL handle (I2C bus)
 * @param  instance  Device instance (0-3, selects the default addresses)
 * @return 1 if a Sense.ADC.6 answered, 0 otherwise
 */
uint8_t tile_sense_adc_6_find(tiles_pal_t *hal, uint8_t instance);

/**
 * @brief  Initialize the tile.
 *
 * Verifies the tile's identity and reads its current settings. Applies
 * any rate/oversamp/window given in cfg; otherwise leaves the tile's
 * stored settings alone.
 *
 * @param  hal       Tiles HAL handle (I2C bus)
 * @param  instance  Device instance (0-3)
 * @param  tile      Tile handle to initialize
 * @param  cfg       Optional config, NULL for defaults
 */
void tile_sense_adc_6_init(tiles_pal_t *hal, uint8_t instance,
                           tile_t *tile, const sense_adc_6_cfg_t *cfg);

/* ================================================================
 * Runtime
 * ================================================================ */

/**
 * @brief  Fetch one coherent set of readings from the tile.
 *
 * One bus transaction collects all six channels, the status and the
 * sequence number from the same instant. Call this, then read the
 * channels with read_mv(), which costs no bus traffic.
 *
 * @studio expose category=tile name=update returns=bool section=runtime
 * @return 1 on success, 0 if the read failed
 */
uint8_t tile_sense_adc_6_update(tile_t *tile);

/**
 * @brief  Millivolts on one channel, from the last update().
 *
 * @studio expose category=tile name=read_mv returns=int section=runtime
 * @param  ch    [0..5] Channel. CH0 is pad 2, CH1 pad 3, CH2 pad 6, CH3 pad 7, CH4 pad 8, CH5 pad 9.
 * @return Millivolts, or 0 if the channel is out of range
 */
uint16_t tile_sense_adc_6_read_mv(tile_t *tile, uint8_t ch);

/**
 * @brief  Copy all six channels from the last update() into an array.
 * @param  out   Destination, at least SENSE_ADC_6_CHANNELS entries
 */
void tile_sense_adc_6_read_all(tile_t *tile, uint16_t *out);

/**
 * @brief  Sequence number of the last set, which advances once per
 *         output period and wraps at 255.
 *
 * Unchanged between two updates means you polled faster than the tile
 * publishes, not that it stopped.
 *
 * @studio expose category=tile name=sequence returns=int section=runtime
 */
uint8_t tile_sense_adc_6_sequence(tile_t *tile);

/**
 * @brief  True when the last set is current: sampling is running and
 *         settled after any settings change.
 *
 * @studio expose category=tile name=is_valid returns=bool section=runtime
 */
uint8_t tile_sense_adc_6_is_valid(tile_t *tile);

/**
 * @brief  Status bits (SENSE_ADC_6_ST_*) from the last update().
 * @studio expose category=tile name=status returns=int section=runtime
 */
uint8_t tile_sense_adc_6_status(tile_t *tile);

/**
 * @brief  The tile's measured supply in millivolts, which is also the
 *         full-scale input voltage.
 *
 * @studio expose category=tile name=supply_mv returns=int section=runtime
 */
uint16_t tile_sense_adc_6_supply_mv(tile_t *tile);

/* ================================================================
 * Configuration
 * ================================================================ */

/**
 * @brief  Set output rate and averaging.
 *
 * Applied as a set, because the three only make sense together. Takes
 * effect within one output period; readings are marked invalid until
 * the filter has refilled, so you never average across the change.
 *
 * Limits: oversamp x window at most 32 sweeps, and rate x oversamp at
 * most 8000 sweeps per second. Out-of-range combinations are rejected
 * and the tile keeps running as it was.
 *
 * @studio expose category=tile name=set_rate returns=bool section=config
 * @param  rate_hz   [1..250] Output rate in Hz.
 * @param  oversamp  [1..32] Sweeps averaged per output period.
 * @param  window    [1..2] Output periods averaged. 2 halves the null frequency.
 * @return 1 if applied, 0 if rejected
 */
uint8_t tile_sense_adc_6_set_rate(tile_t *tile, uint8_t rate_hz,
                                  uint8_t oversamp, uint8_t window);

/**
 * @brief  The tile's current output rate in Hz.
 * @studio expose category=tile name=get_rate returns=int section=config
 */
uint8_t tile_sense_adc_6_get_rate(tile_t *tile);

/**
 * @brief  Sweeps averaged per output period.
 * @studio expose category=tile name=get_oversamp returns=int section=config
 */
uint8_t tile_sense_adc_6_get_oversamp(tile_t *tile);

/**
 * @brief  Output periods averaged (1 or 2).
 * @studio expose category=tile name=get_window returns=int section=config
 */
uint8_t tile_sense_adc_6_get_window(tile_t *tile);

/**
 * @brief  Store the current settings on the tile so they survive a
 *         power cycle.
 *
 * Writes the tile's non-volatile memory, which holds the bus for up to
 * about 30 ms and only writes what actually changed. Do it on a bench,
 * not while other tiles are being polled on the same bus.
 *
 * @studio expose category=tile name=save returns=bool section=config
 * @return 1 on success, 0 on failure
 */
uint8_t tile_sense_adc_6_save(tile_t *tile);

/* ================================================================
 * Advanced
 * ================================================================ */

/**
 * @brief  Give the tile a new I2C address, stored permanently.
 *
 * For several tiles on one bus. The new address takes effect at the
 * tile's NEXT POWER-UP, not immediately, so this handle keeps working
 * until then. Address one tile at a time: they all ship on the same
 * default address, so give each its own before putting them on a shared
 * bus.
 *
 * @studio expose category=tile name=set_address returns=bool section=advanced
 * @param  addr  [8..119] New 7-bit address. 0x08 to 0x77.
 * @return 1 if stored, 0 if rejected or the write failed
 */
uint8_t tile_sense_adc_6_set_address(tile_t *tile, uint8_t addr);

/**
 * @brief  The pad number a channel is wired to (CH0 is pad 2, and so on).
 * @param  ch  [0..5] Channel.
 * @return Pad number, or 0 if the channel is out of range
 */
uint8_t tile_sense_adc_6_channel_pad(uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* TILE_SENSE_ADC_6_H */
