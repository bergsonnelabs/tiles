/**
 * port_core_serial_types.h — STM32 port: UART types for core_serial.h
 *
 * Included by core_serial.h before its prototypes; not for direct use. The
 * bodies of its functions are in port_core_serial.h.
 */

#ifndef PORT_CORE_SERIAL_TYPES_H
#define PORT_CORE_SERIAL_TYPES_H

#include "hal_uart.h"

/** A UART peripheral instance: USART1, LPUART1, ... */
typedef USART_TypeDef core_uart_instance_t;

/** A serial (UART) handle: what core_serial_init() fills and the calls take. */
typedef hal_uart_t core_serial_t;

/** Serial configuration for core_serial_init(): `baud`, `rx_interrupt`. */
typedef hal_uart_config_t core_serial_config_t;

#endif /* PORT_CORE_SERIAL_TYPES_H */
