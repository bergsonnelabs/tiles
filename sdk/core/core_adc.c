/**
 * core_adc.c — the default ADC instance for projects that declare no ADC pad.
 *
 * coregen defines `core_adc1` (and initialises it in core_init.c) only when the
 * project assigns an ADC pad. The internal channels (die temperature, VDD via
 * VREFINT) need no pad, so a program that only reads those used to fail to
 * link ("undefined reference to core_adc1"). This file supplies a weak, zeroed
 * `core_adc1` that coregen's strong one replaces, and core_adc_default(), which
 * initialises it on first use when nothing else has.
 */
#include "core.h"
#include "core_adc.h"

core_adc_t core_adc1 __attribute__((weak));

/* VREFINT and the temperature sensor: ADC4 on the WBA, ADC1 on L0 / L4 / H5
 * (as core_led.c's VDD read). */
#if defined(STM32WBA55xx)
#define CORE_ADC_DEFAULT_PERIPH ADC4
#else
#define CORE_ADC_DEFAULT_PERIPH ADC1
#endif

core_adc_t *core_adc_default(void)
{
    if (core_adc1.instance == 0) {
        (void)hal_adc_init(&core_adc1, CORE_ADC_DEFAULT_PERIPH, SYSCLK_HZ, HAL_ADC_RES_12BIT);
    }
    return &core_adc1;
}
