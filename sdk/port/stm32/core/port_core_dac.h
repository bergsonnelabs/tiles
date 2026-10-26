/**
 * port_core_dac.h — STM32 port: DAC types for core_dac.h
 *
 * Included by core_dac.h; not for direct use.
 */

#ifndef PORT_CORE_DAC_H
#define PORT_CORE_DAC_H

#include "hal_dac.h"

/** The DAC output handle (coregen defines one, core_dac, from config.json). */
typedef hal_dac_t core_dac_t;

#endif /* PORT_CORE_DAC_H */
