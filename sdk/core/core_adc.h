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

/** Core-level ADC handle. Alias for hal_adc_t — use this in application code. */
typedef hal_adc_t core_adc_t;

/** Default ADC instance — emitted by coregen into core_init.c when the
 * project declares any ADC pad. Today this is always `core_adc1`; multi-ADC
 * (Core.ST.H5) lands when tile JSON grows per-pad peripheral tagging. */
extern core_adc_t core_adc1;

/* ============================================================
 * Sampling speed aliases
 * ============================================================ */

#define SAMP_FAST      HAL_ADC_SAMP_FAST
#define SAMP_MED       HAL_ADC_SAMP_MED
#define SAMP_SLOW      HAL_ADC_SAMP_SLOW
#define SAMP_VERY_SLOW HAL_ADC_SAMP_VERY_SLOW

/* ============================================================
 * Resolution aliases
 * ============================================================ */

#define ADC_6BIT   HAL_ADC_RES_6BIT
#define ADC_8BIT   HAL_ADC_RES_8BIT
#define ADC_10BIT  HAL_ADC_RES_10BIT
#define ADC_12BIT  HAL_ADC_RES_12BIT

/* ============================================================
 * Init
 * ============================================================ */

/** Initialise the ADC at the given resolution. */
static inline hal_status_t core_adc_init(core_adc_t *adc, uint32_t resolution)
{
    return tal_adc_init(adc, (hal_adc_res_t)resolution);
}

/* ============================================================
 * Channel registration
 * ============================================================ */

/** Register a pad as an ADC input with the given sampling speed. */
static inline hal_status_t core_adc_add(core_adc_t *adc, uint8_t pad,
                                         uint32_t samp)
{
    return tal_adc_add_pad(adc, pad, (hal_adc_samp_t)samp);
}

/* ============================================================
 * Blocking reads
 * ============================================================ */

/** Single-shot read — returns raw ADC count. */
static inline uint16_t core_adc_read(core_adc_t *adc, uint8_t pad)
{
    return tal_adc_read_pad(adc, pad);
}

/** Single-shot read — returns calibrated millivolts. */
static inline uint32_t core_adc_read_mv(core_adc_t *adc, uint8_t pad)
{
    return tal_adc_read_pad_mv(adc, pad);
}

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
static inline int core_adc_read_pad(uint8_t pad)
{
    return (int)tal_adc_read_pad(&core_adc1, pad);
}

/**
 * Read a pad as calibrated millivolts (uses VREFINT for per-chip accuracy).
 *
 * @studio expose category=adc name=read_mv returns=int
 * @studio twin full
 * @param pad [1..64] Tile pad number configured as an ADC input in config.json.
 */
static inline int core_adc_read_mv_pad(uint8_t pad)
{
    return (int)tal_adc_read_pad_mv(&core_adc1, pad);
}

/**
 * Die temperature in tenths of deg C (e.g. 253 = 25.3 C).
 * Dispatches to the default ADC instance.
 *
 * @studio expose category=adc name=temp_decidegc returns=int
 * @studio twin full
 */
static inline int core_adc_temp_decidegc(void)
{
    return (int)tal_adc_read_temp_decidegc(&core_adc1);
}

/**
 * VDD supply voltage in millivolts (via VREFINT).
 * Dispatches to the default ADC instance.
 *
 * @studio expose category=adc name=vdd_mv returns=int
 * @studio twin full
 */
static inline int core_adc_vdd_mv(void)
{
    return (int)tal_adc_read_vdda_mv(&core_adc1);
}

/* ============================================================
 * Internal channels
 * ============================================================ */

/** Die temperature in tenths of deg C (e.g. 253 = 25.3 C). */
static inline int32_t core_adc_temp(core_adc_t *adc)
{
    return tal_adc_read_temp_decidegc(adc);
}

/** Actual VDD supply voltage in millivolts via VREFINT. */
static inline uint32_t core_adc_vdd(core_adc_t *adc)
{
    return tal_adc_read_vdda_mv(adc);
}

/* ============================================================
 * DMA / continuous mode
 * ============================================================ */

/** Start continuous conversion with DMA circular buffer.
 *  @param cb   Called on half-transfer and transfer-complete; may be NULL
 *  @param ctx  User context passed to callback; may be NULL
 */
static inline hal_status_t core_adc_start_dma(core_adc_t *adc, uint16_t *buf,
                                               uint16_t len,
                                               hal_callback_t cb, void *ctx)
{
    return tal_adc_start_dma(adc, buf, len, cb, ctx);
}

/** Stop continuous DMA conversion. */
static inline void core_adc_stop_dma(core_adc_t *adc)
{
    tal_adc_stop_dma(adc);
}

/** Read most recent DMA result for a pad. */
static inline uint16_t core_adc_dma_read(core_adc_t *adc, uint8_t pad)
{
    return tal_adc_dma_read_pad(adc, pad);
}

/** Read most recent DMA result for a pad, in calibrated millivolts.
 *  Prime VDDA before starting DMA (call core_adc_vdd(adc) once),
 *  since measuring VREFINT needs the regular sequence that DMA owns. */
static inline uint32_t core_adc_dma_read_mv(core_adc_t *adc, uint8_t pad)
{
    return tal_adc_dma_read_pad_mv(adc, pad);
}

/** Convert a raw count from a DMA buffer to millivolts. */
static inline uint32_t core_adc_raw_to_mv(core_adc_t *adc, uint16_t raw)
{
    return hal_adc_raw_to_mv(adc, raw);
}

/* ---- External trigger ---- */

#define ADC_TRIG_NONE     HAL_ADC_TRIG_NONE
#define ADC_TRIG_RISING   HAL_ADC_TRIG_RISING
#define ADC_TRIG_FALLING  HAL_ADC_TRIG_FALLING
#define ADC_TRIG_BOTH     HAL_ADC_TRIG_BOTH

/**
 * Pace DMA conversions from a hardware trigger instead of free-running.
 * Call before core_adc_start_dma. See hal_adc_set_trigger for why this
 * matters to any slow output rate: it is what makes a DMA ring span the
 * whole output period, turning the average into a real anti-alias filter
 * instead of a burst average.
 *
 *   core_timer_init_freq(&t, TIM2, 800);
 *   core_timer_set_trgo(&t, LL_TIM_MMS_UPDATE);
 *   core_adc_set_trigger(adc, LL_ADC_L0_TRG_TIM2_TRGO, ADC_TRIG_RISING);
 *   core_adc_start_dma(adc, buf, len, 0, 0);
 *   core_timer_start(&t);
 */
static inline hal_status_t core_adc_set_trigger(core_adc_t *adc,
                                                uint8_t extsel, uint32_t edge)
{
    return hal_adc_set_trigger(adc, extsel, (hal_adc_trig_edge_t)edge);
}

/** Buffer slot a pad occupies in one DMA scan (-1 if not registered).
 *  Use the stride when the buffer holds several scans for averaging. */
static inline int core_adc_dma_slot(core_adc_t *adc, uint8_t pad)
{
    return tal_adc_dma_slot(adc, pad);
}

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
// @studio unsupported tier=1 value=M title="No oversampling / averaging helper"
//   STM32 ADCs can hardware-oversample 2×–256× for extra effective
//   resolution. The HAL exposes the registers but core_adc has no
//   convenience wrapper — callers reach into tal_adc / hal_adc to set
//   OVSR/OVSS bits manually.
//
// @studio unsupported tier=1 value=L title="No injected channels"
//   Regular-group external triggers ARE supported: core_adc_set_trigger()
//   paces DMA conversions from a TIMx TRGO or EXTI line. Injected groups
//   (a second, higher-priority conversion sequence that preempts the
//   regular one) are not surfaced. Bring-your-own register writes if you
//   need a preempting sample inside a control loop.

#endif /* CORE_ADC_H */
