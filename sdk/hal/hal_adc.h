/**
 * hal_adc.h — ADC HAL driver (Tier 1)
 *
 * Supports all four Core families: L0 (Core.ST.L0), L4 (Core.ST.L4),
 * WBA (Core.ST.W5), H5 (Core.ST.H5).
 *
 * Features:
 *   - Configurable resolution (6/8/10/12-bit)
 *   - Per-channel sampling time
 *   - Hardware oversampling (up to 256×, 1024× on H5)
 *   - Single-shot blocking reads
 *   - VREFINT-based millivolt conversion (supply-independent)
 *   - Die temperature via factory TS_CAL calibration
 *   - DMA circular buffer mode with half/complete callback
 *   - hal_adc_read_all() convenience burst
 */

#ifndef HAL_ADC_H
#define HAL_ADC_H

#include "hal_common.h"
#include "ll_adc.h"
#include <stdbool.h>

/* ============================================================
 * Enums
 * ============================================================ */

/**
 * ADC resolution.
 * All Core families support 6/8/10/12-bit.
 */
typedef enum {
    HAL_ADC_RES_6BIT   = 6,
    HAL_ADC_RES_8BIT   = 8,
    HAL_ADC_RES_10BIT  = 10,
    HAL_ADC_RES_12BIT  = 12,
} hal_adc_res_t;

/**
 * Sampling time presets (approximate ADC clock cycles).
 * Exact cycle count varies by family — see implementation.
 *
 *   FAST:      ~2–4 cycles   — op-amp outputs, DAC, low-impedance
 *   MED:       ~28–48 cycles — general purpose
 *   SLOW:      ~160 cycles   — resistive dividers, NTCs
 *   VERY_SLOW: ~247–640 cycles — high-Z sources, max accuracy
 */
typedef enum {
    HAL_ADC_SAMP_FAST,
    HAL_ADC_SAMP_MED,
    HAL_ADC_SAMP_SLOW,
    HAL_ADC_SAMP_VERY_SLOW,
} hal_adc_samp_t;

/**
 * Oversampling ratio.
 * Enum value = log2(N).  Each doubling adds ~0.5 effective bits
 * in noise-reduction mode, or +1 raw bit in extend mode.
 *
 * HAL_ADC_OVERSAMPLE_1024X is H5 only.
 */
typedef enum {
    HAL_ADC_OVERSAMPLE_1X    = 0,
    HAL_ADC_OVERSAMPLE_2X    = 1,
    HAL_ADC_OVERSAMPLE_4X    = 2,
    HAL_ADC_OVERSAMPLE_8X    = 3,
    HAL_ADC_OVERSAMPLE_16X   = 4,
    HAL_ADC_OVERSAMPLE_32X   = 5,
    HAL_ADC_OVERSAMPLE_64X   = 6,
    HAL_ADC_OVERSAMPLE_128X  = 7,
    HAL_ADC_OVERSAMPLE_256X  = 8,
    HAL_ADC_OVERSAMPLE_1024X = 10,   /* H5 only */
} hal_adc_oversample_t;

/* ============================================================
 * Channel descriptor
 * ============================================================ */

typedef struct {
    uint8_t        channel;   /* ADC channel number (0–18) */
    hal_adc_samp_t samp;
    bool           enabled;
} hal_adc_channel_t;

#define HAL_ADC_MAX_CHANNELS 16

/* ============================================================
 * Main handle
 * ============================================================ */

typedef struct {
    ADC_TypeDef          *instance;
    hal_adc_res_t         resolution;
    hal_adc_oversample_t  oversample;
    hal_adc_channel_t     channels[HAL_ADC_MAX_CHANNELS];
    uint8_t               n_channels;
    uint32_t              sysclk_hz;

    /* DMA circular buffer */
    uint16_t             *dma_buf;
    uint16_t              dma_len;
    hal_callback_t        dma_callback;          /* called on HT and TC */
    void                 *dma_callback_ctx;
    bool                  dma_active;

    /* VREFINT calibration cache — computed lazily on first read_mv call */
    uint32_t              vdda_mv;       /* 0 = not yet measured */

    /* External trigger, applied by hal_adc_start_dma. Edge NONE means
     * free-running continuous mode, which is the historic behavior. */
    uint8_t               trig_extsel;
    uint8_t               trig_edge;

    /* Effective output bit depth — equals resolution bits normally, but
     * increases when hal_adc_set_oversample_ex is used with shift < log2(N).
     * hal_adc_read_mv uses this as the full-scale denominator. */
    uint8_t               effective_bits;
} hal_adc_t;

/* ============================================================
 * API
 * ============================================================ */

/**
 * Initialise an ADC instance.
 *
 * Enables the peripheral clock, exits deep power-down (L4/H5),
 * runs self-calibration, and enables the ADC.
 *
 * Must be called before add_channel or read.
 * ADC input pins must already be configured analog (hal_pad_analog).
 *
 * @param adc       Handle to zero or uninitialised memory
 * @param instance  ADC peripheral (ADC1, ADC4, …)
 * @param sysclk_hz System clock in Hz (used to choose ADC clock divisor)
 * @param res       Conversion resolution
 * @return HAL_OK, or HAL_ERROR if resolution is unsupported on this family
 */
hal_status_t hal_adc_init(hal_adc_t *adc, ADC_TypeDef *instance,
                          uint32_t sysclk_hz, hal_adc_res_t res);

/**
 * Add a channel to the conversion sequence.
 * Channels are converted in the order they are added.
 * Call after hal_adc_init, before hal_adc_read or hal_adc_start_dma.
 */
hal_status_t hal_adc_add_channel(hal_adc_t *adc, uint8_t channel,
                                 hal_adc_samp_t samp);

/**
 * Set hardware oversampling ratio for noise reduction (applies to all channels).
 * The right-shift equals log2(N), so the output word stays at the configured
 * resolution. Can be called after init; takes effect on the next conversion.
 */
hal_status_t hal_adc_set_oversample(hal_adc_t *adc, hal_adc_oversample_t ratio);

/**
 * Set hardware oversampling with explicit right-shift for resolution extension.
 *
 * shift controls how many bits the hardware divides the accumulator by before
 * writing to DR.  Valid range: 0 – log2(N) where N is the oversampling ratio.
 *
 *   shift = log2(N)      → same as hal_adc_set_oversample (noise reduction only)
 *   shift < log2(N)      → output word is wider; effective_bits increases
 *   shift = 0            → raw accumulator in DR; max bit depth
 *
 * Example — 12-bit ADC, 256× oversampling, shift = 4:
 *   accumulator = up to 20 bits; DR = accumulator >> 4 = 16-bit result
 *   effective_bits = 16; hal_adc_read_mv scales against 65535 automatically.
 *
 * @param adc    Initialised handle
 * @param ratio  Oversampling ratio (HAL_ADC_OVERSAMPLE_4X … _256X)
 * @param shift  Right-shift applied by hardware (0 = none, ratio/2 = full)
 */
hal_status_t hal_adc_set_oversample_ex(hal_adc_t *adc, hal_adc_oversample_t ratio,
                                        uint8_t shift);

/**
 * Single-shot blocking read of one channel.
 * Returns the raw ADC count (range depends on resolution).
 *
 * The channel must have been added via hal_adc_add_channel.
 */
uint16_t hal_adc_read(hal_adc_t *adc, uint8_t channel);

/**
 * Read a channel and convert to millivolts.
 *
 * Uses the VREFINT factory calibration to derive the actual VDDA
 * supply voltage, so the result is correct even when VDD ≠ 3.3 V.
 * The VDDA measurement is cached after the first call.
 */
uint32_t hal_adc_read_mv(hal_adc_t *adc, uint8_t channel);

/**
 * Convert a raw count to millivolts using the VREFINT-derived VDDA.
 *
 * Does no conversion of its own, so unlike hal_adc_read_mv it is safe to
 * call while continuous DMA owns the regular sequence. That is the point
 * of it: hal_adc_start_dma fills the buffer with raw counts and there is
 * otherwise no supported way to scale them.
 *
 * VDDA is measured on first use, and measuring VREFINT DOES need the
 * regular sequence. So prime it (call this once, or hal_adc_read_vdda_mv)
 * BEFORE hal_adc_start_dma; priming it afterwards returns 0.
 */
uint32_t hal_adc_raw_to_mv(hal_adc_t *adc, uint16_t raw);

/* ============================================================
 * External trigger
 * ============================================================ */

typedef enum {
    HAL_ADC_TRIG_NONE    = 0,   /* free-running continuous conversion */
    HAL_ADC_TRIG_RISING  = 1,
    HAL_ADC_TRIG_FALLING = 2,
    HAL_ADC_TRIG_BOTH    = 3,
} hal_adc_trig_edge_t;

/**
 * Pace conversions from a hardware trigger instead of free-running.
 *
 * Call BEFORE hal_adc_start_dma; it only records the setting, and
 * start_dma applies it. With a trigger set, start_dma clears CONT and
 * one scan of the whole channel list runs per trigger edge.
 *
 * This is what lets a slow output rate average over its whole period
 * rather than over one burst. Free-running fills the DMA ring as fast
 * as the ADC can convert, so a ring of N scans only ever holds the last
 * N conversion times worth of signal, however long the period is. That
 * averages noise but does nothing about aliasing, and burns power
 * converting samples nobody reads. Pacing the scans so the ring spans
 * the full output period turns the same average into a boxcar filter,
 * with nulls at the output rate and its multiples.
 *
 * `extsel` is the family-specific trigger encoding. On Core.ST.L0 use
 * the LL_ADC_L0_TRG_* constants (RM0377 Table 58); LL_ADC_L0_TRG_TIM2_TRGO
 * paired with ll_tim_set_trgo(TIM2, LL_TIM_MMS_UPDATE) is the usual pair.
 */
hal_status_t hal_adc_set_trigger(hal_adc_t *adc, uint8_t extsel,
                                 hal_adc_trig_edge_t edge);

/**
 * Read die temperature in tenths of a degree C.
 * Example: returns 253 for 25.3 °C.
 *
 * Uses TS_CAL1 / TS_CAL2 factory calibration values.
 * Automatically adds the internal temperature sensor channel on first call.
 */
int32_t hal_adc_read_temp_decidegc(hal_adc_t *adc);

/**
 * Disable an ADC initialised with hal_adc_init() and power it down, so a
 * one-off measurement (the status LED's VDD read) leaves no standing
 * current behind.
 * L0: ADEN and the voltage regulator off (RM0377 ADC_CR bit 28). L4: also
 * back into deep power-down (RM0394 ADC_CR bit 29). Other families: ADEN
 * off only. The peripheral clock stays enabled.
 */
void hal_adc_deinit(hal_adc_t *adc);

/**
 * Read the actual VDD supply voltage in millivolts via VREFINT.
 * Result is accurate regardless of nominal supply.
 */
uint32_t hal_adc_read_vdda_mv(hal_adc_t *adc);

/**
 * Read all configured channels into buf[].
 * Buffer order matches the order channels were added via hal_adc_add_channel.
 * buf must be at least adc->n_channels elements.
 */
void hal_adc_read_all(hal_adc_t *adc, uint16_t *buf);

/**
 * Start continuous ADC conversion with DMA circular buffer.
 *
 * callback is invoked at both half-complete and complete events,
 * supporting a double-buffered processing pattern:
 *   - At half-complete: process buf[0 .. len/2 - 1]
 *   - At complete:      process buf[len/2 .. len - 1]
 *
 * Channels must already be configured via hal_adc_add_channel.
 * DMA and ADC peripheral clocks are enabled automatically.
 *
 * @param adc      Handle
 * @param buf      uint16_t buffer for DMA destination (must stay valid)
 * @param len      Number of uint16_t elements (multiple of n_channels)
 * @param callback Called on HT and TC interrupts (may be NULL)
 */
hal_status_t hal_adc_start_dma(hal_adc_t *adc, uint16_t *buf, uint16_t len,
                                hal_callback_t callback, void *ctx);

/**
 * Stop DMA conversion and disable the DMA channel.
 */
void hal_adc_stop_dma(hal_adc_t *adc);

/* ---- Legacy / compatibility shims (deprecated) ---- */

/** @deprecated Use hal_adc_init(adc, instance, sysclk_hz, HAL_ADC_RES_12BIT) */
static inline hal_status_t hal_adc_init_simple(hal_adc_t *adc, ADC_TypeDef *instance)
{
    return hal_adc_init(adc, instance, 80000000UL, HAL_ADC_RES_12BIT);
}

/** @deprecated Pad-based read; use hal_pad_analog + hal_adc_add_channel + hal_adc_read */
uint16_t hal_adc_read_pad(hal_adc_t *adc, uint8_t pad);

/** @deprecated */
uint32_t hal_adc_read_pad_mv(hal_adc_t *adc, uint8_t pad);

#endif /* HAL_ADC_H */
