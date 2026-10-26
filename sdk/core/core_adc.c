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

/* The ADC with the internal channels and its bring-up are the port's
 * (port_core_adc_default.h). */
#include "port_core_adc_default.h"

core_adc_t *core_adc_default(void)
{
    port_core_adc_default_init(&core_adc1);
    return &core_adc1;
}
