/**
 * port_core_adc.h — STM32 port: value checks and bodies for core_adc.h
 *
 * Included at the end of core_adc.h, after its prototypes; not for direct
 * use. core_adc_init / _add / _set_trigger cast their argument straight to
 * the HAL enum, so each CORE_ADC_* value must equal the HAL's. The bodies go
 * through tal_adc (pad -> channel) and hal_adc.
 */

#ifndef PORT_CORE_ADC_H
#define PORT_CORE_ADC_H

#include "hal_adc.h"
#include "tal_adc.h"

#ifndef __cplusplus
_Static_assert(CORE_ADC_SAMP_FAST      == HAL_ADC_SAMP_FAST,      "CORE_ADC_SAMP_FAST");
_Static_assert(CORE_ADC_SAMP_MED       == HAL_ADC_SAMP_MED,       "CORE_ADC_SAMP_MED");
_Static_assert(CORE_ADC_SAMP_SLOW      == HAL_ADC_SAMP_SLOW,      "CORE_ADC_SAMP_SLOW");
_Static_assert(CORE_ADC_SAMP_VERY_SLOW == HAL_ADC_SAMP_VERY_SLOW, "CORE_ADC_SAMP_VERY_SLOW");
_Static_assert(CORE_ADC_RES_6BIT  == HAL_ADC_RES_6BIT,  "CORE_ADC_RES_6BIT");
_Static_assert(CORE_ADC_RES_8BIT  == HAL_ADC_RES_8BIT,  "CORE_ADC_RES_8BIT");
_Static_assert(CORE_ADC_RES_10BIT == HAL_ADC_RES_10BIT, "CORE_ADC_RES_10BIT");
_Static_assert(CORE_ADC_RES_12BIT == HAL_ADC_RES_12BIT, "CORE_ADC_RES_12BIT");
_Static_assert(CORE_ADC_TRIG_NONE    == HAL_ADC_TRIG_NONE,    "CORE_ADC_TRIG_NONE");
_Static_assert(CORE_ADC_TRIG_RISING  == HAL_ADC_TRIG_RISING,  "CORE_ADC_TRIG_RISING");
_Static_assert(CORE_ADC_TRIG_FALLING == HAL_ADC_TRIG_FALLING, "CORE_ADC_TRIG_FALLING");
_Static_assert(CORE_ADC_TRIG_BOTH    == HAL_ADC_TRIG_BOTH,    "CORE_ADC_TRIG_BOTH");
#endif

static inline core_status_t core_adc_init(core_adc_t *adc, uint32_t resolution)
{
    return tal_adc_init(adc, (hal_adc_res_t)resolution);
}

static inline core_status_t core_adc_add(core_adc_t *adc, uint8_t pad,
                                          uint32_t samp)
{
    return tal_adc_add_pad(adc, pad, (hal_adc_samp_t)samp);
}

static inline uint16_t core_adc_read(core_adc_t *adc, uint8_t pad)
{
    return tal_adc_read_pad(adc, pad);
}

static inline uint32_t core_adc_read_mv(core_adc_t *adc, uint8_t pad)
{
    return tal_adc_read_pad_mv(adc, pad);
}

static inline int core_adc_read_pad(uint8_t pad)
{
    return (int)tal_adc_read_pad(&core_adc1, pad);
}

static inline int core_adc_read_mv_pad(uint8_t pad)
{
    return (int)tal_adc_read_pad_mv(&core_adc1, pad);
}

static inline int core_adc_temp_decidegc(void)
{
    return (int)tal_adc_read_temp_decidegc(core_adc_default());
}

static inline int core_adc_vdd_mv(void)
{
    return (int)tal_adc_read_vdda_mv(core_adc_default());
}

static inline int32_t core_adc_temp(core_adc_t *adc)
{
    return tal_adc_read_temp_decidegc(adc);
}

static inline uint32_t core_adc_vdd(core_adc_t *adc)
{
    return tal_adc_read_vdda_mv(adc);
}

static inline core_status_t core_adc_start_dma(core_adc_t *adc, uint16_t *buf,
                                                uint16_t len,
                                                core_callback_t cb, void *ctx)
{
    return tal_adc_start_dma(adc, buf, len, cb, ctx);
}

static inline void core_adc_stop_dma(core_adc_t *adc)
{
    tal_adc_stop_dma(adc);
}

static inline uint16_t core_adc_dma_read(core_adc_t *adc, uint8_t pad)
{
    return tal_adc_dma_read_pad(adc, pad);
}

static inline uint32_t core_adc_dma_read_mv(core_adc_t *adc, uint8_t pad)
{
    return tal_adc_dma_read_pad_mv(adc, pad);
}

static inline uint32_t core_adc_raw_to_mv(core_adc_t *adc, uint16_t raw)
{
    return hal_adc_raw_to_mv(adc, raw);
}

static inline core_status_t core_adc_set_trigger(core_adc_t *adc,
                                                 uint8_t extsel, uint32_t edge)
{
    return hal_adc_set_trigger(adc, extsel, (hal_adc_trig_edge_t)edge);
}

static inline int core_adc_dma_slot(core_adc_t *adc, uint8_t pad)
{
    return tal_adc_dma_slot(adc, pad);
}

#endif /* PORT_CORE_ADC_H */
