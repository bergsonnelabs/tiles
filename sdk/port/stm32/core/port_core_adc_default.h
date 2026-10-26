/**
 * port_core_adc_default.h — STM32 port: the default ADC behind core_adc.c
 *
 * Included by core_adc.c only; not for direct use.
 * port_core_adc_default_init() brings up the handle on the ADC that has the
 * internal channels (VREFINT, the temperature sensor), 12-bit, unless
 * something already initialised it.
 */

#ifndef PORT_CORE_ADC_DEFAULT_H
#define PORT_CORE_ADC_DEFAULT_H

#include "core_adc.h"
#include "core_config.h"   /* SYSCLK_HZ */
#include "hal_adc.h"

/* VREFINT and the temperature sensor: ADC4 on the WBA, ADC1 on L0 / L4 / H5
 * (as core_led.c's VDD read, port_core_led_vdd.h). */
#if defined(STM32WBA55xx)
#define CORE_ADC_DEFAULT_PERIPH ADC4
#else
#define CORE_ADC_DEFAULT_PERIPH ADC1
#endif

/* always_inline: -Og does not inline a function called once, and this one
 * always sat inside core_adc_default(). */
static inline __attribute__((always_inline)) void port_core_adc_default_init(core_adc_t *adc)
{
    if (adc->instance == 0) {
        (void)hal_adc_init(adc, CORE_ADC_DEFAULT_PERIPH, SYSCLK_HZ, HAL_ADC_RES_12BIT);
    }
}

#endif /* PORT_CORE_ADC_DEFAULT_H */
