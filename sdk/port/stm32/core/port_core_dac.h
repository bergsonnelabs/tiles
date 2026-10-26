/**
 * port_core_dac.h — STM32 port: the bodies of core_dac.h
 *
 * Included at the end of core_dac.h, after its prototypes; not for direct
 * use. The types are in port_core_dac_types.h.
 */

#ifndef PORT_CORE_DAC_H
#define PORT_CORE_DAC_H

#include "hal_dac.h"
#include "port_core_dac_types.h"

static inline void core_dac_init(void)
{
    hal_dac_init(&core_dac);
}

static inline void core_dac_write(uint16_t val)
{
    hal_dac_write(&core_dac, val);
}

static inline void core_dac_write_mv(uint16_t mv)
{
    hal_dac_write_mv(&core_dac, mv);
}

static inline uint16_t core_dac_read(void)
{
    return hal_dac_read(&core_dac);
}

#endif /* PORT_CORE_DAC_H */
