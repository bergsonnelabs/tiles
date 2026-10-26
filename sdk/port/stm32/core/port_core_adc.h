/**
 * port_core_adc.h — STM32 port: value checks for core_adc.h
 *
 * Included by core_adc.h after its neutral constants; not for direct use.
 * core_adc_init / _add / _set_trigger cast their argument straight to the HAL
 * enum, so each CORE_ADC_* value must equal the HAL's.
 */

#ifndef PORT_CORE_ADC_H
#define PORT_CORE_ADC_H

#include "hal_adc.h"

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

#endif /* PORT_CORE_ADC_H */
