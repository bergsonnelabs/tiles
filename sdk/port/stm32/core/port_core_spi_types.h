/**
 * port_core_spi_types.h — STM32 port: SPI types for core_spi.h
 *
 * Included by core_spi.h before its prototypes; not for direct use. The
 * bodies of its functions are in port_core_spi.h.
 */

#ifndef PORT_CORE_SPI_TYPES_H
#define PORT_CORE_SPI_TYPES_H

#include "ll_spi.h"

/** An SPI peripheral instance: SPI1, SPI3, ... */
typedef SPI_TypeDef core_spi_instance_t;

#endif /* PORT_CORE_SPI_TYPES_H */
