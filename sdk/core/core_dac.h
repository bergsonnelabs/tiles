/**
 * core_dac.h — DAC output (analog voltage generation)
 *
 * Output a 12-bit analog voltage on a DAC-capable pad.
 * The DAC instance is configured by coregen based on config.json.
 *
 * Only available on Core.ST.H5 (STM32H523).
 *
 * Usage:
 *   core_dac_init();
 *   core_dac_write_mv(1650);   // Output 1.65V
 *   core_dac_write(2048);      // Raw 12-bit value (0–4095)
 *
 * @studio category dac label=Core.DAC icon=◓
 *
 * @studio coverage
 *   id:    dac
 *   name:  DAC — digital-to-analog output
 *   blurb: Core.ST.H5-only API for the on-chip 12-bit DAC. Tier 2 helpers
 *          init the default instance (resolved by coregen from
 *          config.json) and write a raw count, write millivolts, or
 *          read back the current output register. No waveform / DMA
 *          path — single-sample writes only.
 */

#ifndef CORE_DAC_H
#define CORE_DAC_H

#if !defined(STM32H523xx)
#error "core_dac.h: DAC is not available on this Core tile. Only Core.ST.H5 (STM32H523) has a DAC peripheral."
#endif

#include "hal_dac.h"

/* DAC instance — defined by coregen in core_init.c based on config.json.
 * Stores the GPIO port/pin and DAC channel from the tile definition. */
extern hal_dac_t core_dac;

/**
 * Initialize the DAC (clock, GPIO, peripheral). Call once from `on start`
 * before any writes.
 *
 * @studio expose category=dac name=init availability=Core.ST.H5
 * @studio twin full
 */
static inline void core_dac_init(void)
{
    hal_dac_init(&core_dac);
}

/**
 * Write a raw 12-bit value. Output = val / 4095 × VREF+.
 *
 * @studio expose category=dac name=write availability=Core.ST.H5
 * @studio twin full
 * @param val [0..4095] Raw DAC value.
 */
static inline void core_dac_write(uint16_t val)
{
    hal_dac_write(&core_dac, val);
}

/**
 * Write a voltage in millivolts.
 *
 * @studio expose category=dac name=write_mv availability=Core.ST.H5
 * @studio twin full
 * @param mv [0..3300] Output voltage in millivolts.
 */
static inline void core_dac_write_mv(uint16_t mv)
{
    hal_dac_write_mv(&core_dac, mv);
}

/**
 * Read back the current DAC output register value (12-bit).
 *
 * @studio expose category=dac name=read returns=int availability=Core.ST.H5
 * @studio twin full
 */
static inline uint16_t core_dac_read(void)
{
    return hal_dac_read(&core_dac);
}

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=1 value=M title="No waveform / DMA generation"
//   Single-sample writes only. STM32H5's DAC supports DMA-fed sample
//   buffers + the internal noise / triangle wave generators — none
//   of which are wrapped. Audio + arbitrary-waveform tiles need this.
//
// @studio unsupported tier=1 value=L title="No dual-channel synchronization"
//   The DAC peripheral has two channels that can be triggered together
//   for stereo / I-Q output. Only single-channel writes are exposed.

#endif /* CORE_DAC_H */
