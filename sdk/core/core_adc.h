/**
 * core_adc.h — Pad-level ADC reads
 *
 * The primary ADC API. All functions use tile pad numbers.
 * Channels, instances, and clocks are resolved automatically
 * from the project configuration.
 *
 * @studio category adc label=Core.ADC icon=◐
 *
 * @studio coverage
 *   id:    adc
 *   name:  ADC — analog-to-digital conversion
 *   page:  /docs/sdk/adc
 *   blurb: Pad-numbered ADC reads in raw counts and calibrated
 *          millivolts, plus internal channels (die temperature, VDD).
 *          Tier 2 dispatches to the coregen-emitted default instance
 *          (`core_adc1`); Tier 1 (the explicit-handle API) covers
 *          channel registration, sampling-speed control, and DMA
 *          continuous mode for advanced callers.
 */

#ifndef CORE_ADC_H
#define CORE_ADC_H

#include "tal_adc.h"
#include "core_types.h"

/** Core-level ADC handle. Alias for hal_adc_t — use this in application code. */
typedef hal_adc_t core_adc_t;

/** Default ADC instance — emitted by coregen into core_init.c when the
 * project declares any ADC pad. Today this is always `core_adc1`; multi-ADC
 * (Core.ST.H5) lands when tile JSON grows per-pad peripheral tagging. Without
 * an ADC pad, core_adc.c supplies a weak, uninitialised one. */
extern core_adc_t core_adc1;

/** `core_adc1`, initialised on first use if no ADC pad set it up (core_adc.c):
 * the internal channels (temperature, VDD) need no pad. */
core_adc_t *core_adc_default(void);

/* ============================================================
 * Sampling speed (core_adc_add)
 * ============================================================ */

#define CORE_ADC_SAMP_FAST       0   /**< Shortest: op-amp outputs, DAC, low impedance */
#define CORE_ADC_SAMP_MED        1   /**< General purpose */
#define CORE_ADC_SAMP_SLOW       2   /**< Resistive dividers, NTCs */
#define CORE_ADC_SAMP_VERY_SLOW  3   /**< Longest: high-impedance sources, best accuracy */

/* ============================================================
 * Resolution (core_adc_init)
 * ============================================================ */

#define CORE_ADC_RES_6BIT   6    /**< 6-bit conversions */
#define CORE_ADC_RES_8BIT   8    /**< 8-bit conversions */
#define CORE_ADC_RES_10BIT  10   /**< 10-bit conversions */
#define CORE_ADC_RES_12BIT  12   /**< 12-bit conversions */

/* ============================================================
 * External trigger edge (core_adc_set_trigger)
 * ============================================================ */

#define CORE_ADC_TRIG_NONE     0   /**< Free-running: no hardware trigger */
#define CORE_ADC_TRIG_RISING   1   /**< Convert on the trigger's rising edge */
#define CORE_ADC_TRIG_FALLING  2   /**< Convert on the falling edge */
#define CORE_ADC_TRIG_BOTH     3   /**< Convert on both edges */

/* Deprecated short names, kept so existing code builds: use the CORE_ADC_ ones. */
#define SAMP_FAST      CORE_ADC_SAMP_FAST
#define SAMP_MED       CORE_ADC_SAMP_MED
#define SAMP_SLOW      CORE_ADC_SAMP_SLOW
#define SAMP_VERY_SLOW CORE_ADC_SAMP_VERY_SLOW
#define ADC_6BIT       CORE_ADC_RES_6BIT
#define ADC_8BIT       CORE_ADC_RES_8BIT
#define ADC_10BIT      CORE_ADC_RES_10BIT
#define ADC_12BIT      CORE_ADC_RES_12BIT
#define ADC_TRIG_NONE     CORE_ADC_TRIG_NONE
#define ADC_TRIG_RISING   CORE_ADC_TRIG_RISING
#define ADC_TRIG_FALLING  CORE_ADC_TRIG_FALLING
#define ADC_TRIG_BOTH     CORE_ADC_TRIG_BOTH

/* ============================================================
 * Init
 * ============================================================ */

/** Initialise the ADC at the given resolution. */
CORE_INLINE core_status_t core_adc_init(core_adc_t *adc, uint32_t resolution);

/* ============================================================
 * Channel registration
 * ============================================================ */

/** Register a pad as an ADC input with the given sampling speed. */
CORE_INLINE core_status_t core_adc_add(core_adc_t *adc, uint8_t pad,
                                        uint32_t samp);

/* ============================================================
 * Blocking reads
 * ============================================================ */

/** Single-shot read — returns raw ADC count. */
CORE_INLINE uint16_t core_adc_read(core_adc_t *adc, uint8_t pad);

/** Single-shot read — returns calibrated millivolts. */
CORE_INLINE uint32_t core_adc_read_mv(core_adc_t *adc, uint8_t pad);

/* ============================================================
 * Pad-oriented wrappers (default-instance convention)
 * ============================================================
 *
 * These dispatch to the coregen-emitted default handle (`core_adc1`) so
 * the caller never has to pick an ADC peripheral. config.json drives the
 * handle into existence via `build_adc_config()` + `core_pads_init()`;
 * if no ADC pad is configured, calling these wrappers fails to link
 * — the correct outcome.
 */

/**
 * Read a pad as raw ADC counts (0–4095 at 12-bit resolution).
 *
 * @studio expose category=adc name=read returns=int
 * @studio twin full
 * @param pad [1..64] Tile pad number configured as an ADC input in config.json.
 */
CORE_INLINE int core_adc_read_pad(uint8_t pad);

/**
 * Read a pad as calibrated millivolts (uses VREFINT for per-chip accuracy).
 *
 * @studio expose category=adc name=read_mv returns=int
 * @studio twin full
 * @param pad [1..64] Tile pad number configured as an ADC input in config.json.
 */
CORE_INLINE int core_adc_read_mv_pad(uint8_t pad);

/**
 * Die temperature in tenths of deg C (e.g. 253 = 25.3 C).
 * Dispatches to the default ADC instance.
 *
 * @studio expose category=adc name=temp_decidegc returns=int
 * @studio twin full
 */
CORE_INLINE int core_adc_temp_decidegc(void);

/**
 * VDD supply voltage in millivolts (via VREFINT).
 * Dispatches to the default ADC instance.
 *
 * @studio expose category=adc name=vdd_mv returns=int
 * @studio twin full
 */
CORE_INLINE int core_adc_vdd_mv(void);

/* ============================================================
 * Internal channels
 * ============================================================ */

/** Die temperature in tenths of deg C (e.g. 253 = 25.3 C). */
CORE_INLINE int32_t core_adc_temp(core_adc_t *adc);

/** Actual VDD supply voltage in millivolts via VREFINT. */
CORE_INLINE uint32_t core_adc_vdd(core_adc_t *adc);

/* ============================================================
 * DMA / continuous mode
 * ============================================================ */

/** Start continuous conversion with DMA circular buffer.
 *  @param cb   Called on half-transfer and transfer-complete; may be NULL
 *  @param ctx  User context passed to callback; may be NULL
 */
CORE_INLINE core_status_t core_adc_start_dma(core_adc_t *adc, uint16_t *buf,
                                              uint16_t len,
                                              core_callback_t cb, void *ctx);

/** Stop continuous DMA conversion. */
CORE_INLINE void core_adc_stop_dma(core_adc_t *adc);

/** Read most recent DMA result for a pad. */
CORE_INLINE uint16_t core_adc_dma_read(core_adc_t *adc, uint8_t pad);

/** Read most recent DMA result for a pad, in calibrated millivolts.
 *  Prime VDDA before starting DMA (call core_adc_vdd(adc) once),
 *  since measuring VREFINT needs the regular sequence that DMA owns. */
CORE_INLINE uint32_t core_adc_dma_read_mv(core_adc_t *adc, uint8_t pad);

/** Convert a raw count from a DMA buffer to millivolts. */
CORE_INLINE uint32_t core_adc_raw_to_mv(core_adc_t *adc, uint16_t raw);

/* ---- External trigger ---- */

/**
 * Pace DMA conversions from a hardware trigger instead of free-running.
 * Call before core_adc_start_dma. See hal_adc_set_trigger for why this
 * matters to any slow output rate: it is what makes a DMA ring span the
 * whole output period, turning the average into a real anti-alias filter
 * instead of a burst average.
 *
 *   core_timer_init_freq(&t, TIM2, 800);
 *   core_timer_set_trgo(&t, LL_TIM_MMS_UPDATE);
 *   core_adc_set_trigger(adc, LL_ADC_L0_TRG_TIM2_TRGO, CORE_ADC_TRIG_RISING);
 *   core_adc_start_dma(adc, buf, len, 0, 0);
 *   core_timer_start(&t);
 */
CORE_INLINE core_status_t core_adc_set_trigger(core_adc_t *adc,
                                               uint8_t extsel, uint32_t edge);

/** Buffer slot a pad occupies in one DMA scan (-1 if not registered).
 *  Use the stride when the buffer holds several scans for averaging. */
CORE_INLINE int core_adc_dma_slot(core_adc_t *adc, uint8_t pad);

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=L title="Twin VDD / die-temp are constants"
//   read_pad / read_mv_pad use the per-pad slider on the chip view, but
//   temp_decidegc and vdd_mv return fixed values (25.0 °C / 3300 mV).
//   No UI affordance to vary them yet — DSL programs that branch on
//   under-/over-temp or low-VDD logic always see nominal in the IDE.
//
// @studio unsupported tier=2 value=M title="No DSL access to sampling speed / resolution"
//   Tier 2 calls the default instance at its compile-time resolution
//   (12-bit) and per-pad sampling speed set during init. SAMP_FAST /
//   MED / SLOW / VERY_SLOW and ADC_6/8/10/12BIT aliases are reachable
//   only via the explicit-handle Tier 1 API.
//
// @studio unsupported tier=2 value=M title="No multi-instance ADC dispatch"
//   Tier 2 wrappers always hit core_adc1. Cores with multiple ADCs
//   (Core.ST.H5 has ADC1 + ADC2) need per-pad peripheral tagging in tile
//   JSON before coregen can emit the dispatch. Until then DSL programs
//   are limited to whichever pads coregen routed to ADC1.
//
// @studio unsupported tier=2 value=L title="No core_adc / DSL oversampling wrapper"
//   Hardware oversampling (2x to 256x; 1024x on Core.ST.H5) works through
//   the HAL on the same handle: hal_adc_set_oversample(&core_adc1, ratio)
//   averages, hal_adc_set_oversample_ex() also widens the result. There is
//   no core_adc_* wrapper or DSL surface for it.
//
// @studio unsupported tier=1 value=L title="No injected channels"
//   Regular-group external triggers ARE supported: core_adc_set_trigger()
//   paces DMA conversions from a TIMx TRGO or EXTI line. Injected groups
//   (a second, higher-priority conversion sequence that preempts the
//   regular one) are not surfaced. Bring-your-own register writes if you
//   need a preempting sample inside a control loop.

/* The port: checks that the CORE_ADC_* values above are its own, and the
 * bodies of the functions above. */
#include "port_core_adc.h"

#endif /* CORE_ADC_H */
