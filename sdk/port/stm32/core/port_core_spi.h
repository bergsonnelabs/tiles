/**
 * port_core_spi.h — STM32 port: SPI types for core_spi.h
 *
 * Included by core_spi.h; not for direct use.
 */

#ifndef PORT_CORE_SPI_H
#define PORT_CORE_SPI_H

#include "ll_spi.h"

/** An SPI peripheral instance: SPI1, SPI3, ... */
typedef SPI_TypeDef core_spi_instance_t;

#endif /* PORT_CORE_SPI_H */
