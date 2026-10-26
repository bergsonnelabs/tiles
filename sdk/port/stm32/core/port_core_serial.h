/**
 * port_core_serial.h — STM32 port: the bodies of core_serial.h
 *
 * Included at the end of core_serial.h, after its prototypes; not for direct
 * use. The types are in port_core_serial_types.h.
 */

#ifndef PORT_CORE_SERIAL_H
#define PORT_CORE_SERIAL_H

#include "hal_uart.h"
#include "port_core_serial_types.h"

static inline core_status_t core_serial_init(core_serial_t *h,
                                              core_uart_instance_t *instance,
                                              const core_serial_config_t *cfg)
{
    return hal_uart_init(h, instance, PCLK1_HZ, cfg);
}

static inline core_status_t core_serial_init_clk(core_serial_t *h,
                                                  core_uart_instance_t *instance,
                                                  uint32_t pclk_hz,
                                                  const core_serial_config_t *cfg)
{
    return hal_uart_init(h, instance, pclk_hz, cfg);
}

static inline void core_serial_write(core_serial_t *h, const uint8_t *data,
                                      uint32_t len)
{
    hal_uart_tx(h, data, len);
}

static inline void core_serial_print(core_serial_t *h, const char *str)
{
    hal_uart_tx_str(h, str);
}

/** Printf over UART (blocking). */
#define core_serial_printf  hal_uart_printf

static inline void core_serial_putc(core_serial_t *h, uint8_t byte)
{
    hal_uart_putc(h, byte);
}

static inline uint16_t core_serial_available(core_serial_t *h)
{
    return hal_uart_available(h);
}

static inline uint8_t core_serial_getc(core_serial_t *h)
{
    return hal_uart_rx(h);
}

static inline uint16_t core_serial_read(core_serial_t *h, uint8_t *buf,
                                         uint16_t max_len)
{
    return hal_uart_read(h, buf, max_len);
}

static inline int core_serial_print_bus(uint8_t bus, const char *str)
{
    hal_uart_t *h = core_serial_handle_for_bus(bus);
    if (!h) return -1;
    hal_uart_tx_str(h, str);
    return 0;
}

static inline int core_serial_putc_bus(uint8_t bus, uint8_t byte)
{
    hal_uart_t *h = core_serial_handle_for_bus(bus);
    if (!h) return -1;
    hal_uart_putc(h, byte);
    return 0;
}

#endif /* PORT_CORE_SERIAL_H */
