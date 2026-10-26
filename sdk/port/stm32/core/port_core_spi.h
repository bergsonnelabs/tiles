/**
 * port_core_spi.h — STM32 port: the bodies of core_spi.h
 *
 * Included at the end of core_spi.h, after its prototypes; not for direct
 * use. The types are in port_core_spi_types.h.
 */

#ifndef PORT_CORE_SPI_H
#define PORT_CORE_SPI_H

#include "hal_spi.h"
#include "hal_gpio.h"   /* hal_pad_lookup: the CS pad */
#include "port_core_spi_types.h"

static inline core_status_t core_spi_init(core_spi_t *h,
                                           core_spi_instance_t *instance,
                                           const core_spi_config_t *cfg)
{
    return hal_spi_init(h, instance, cfg);
}

static inline core_status_t core_spi_configure(core_spi_t *h,
                                                const core_spi_config_t *cfg)
{
    return hal_spi_configure(h, cfg);
}

static inline uint32_t core_spi_sck_hz(const core_spi_t *h)
{
    return hal_spi_sck_hz(h);
}

static inline void core_spi_set_cs(core_spi_t *h, uint8_t pad)
{
    hal_pad_gpio_t g = hal_pad_lookup(pad);
    if (g.port)
        hal_spi_set_cs(h, g.port, g.pin);
}

static inline void core_spi_select(core_spi_t *h)
{
    hal_spi_select(h);
}

static inline void core_spi_deselect(core_spi_t *h)
{
    hal_spi_deselect(h);
}

static inline uint8_t core_spi_transfer(core_spi_t *h, uint8_t tx)
{
    return hal_spi_transfer(h, tx);
}

static inline core_status_t core_spi_exchange(core_spi_t *h, const uint8_t *tx,
                                               uint8_t *rx, uint32_t len)
{
    return hal_spi_exchange(h, tx, rx, len);
}

static inline core_status_t core_spi_write(core_spi_t *h, const uint8_t *data,
                                            uint32_t len)
{
    return hal_spi_write(h, data, len);
}

static inline core_status_t core_spi_read(core_spi_t *h, uint8_t *buf, uint32_t len)
{
    return hal_spi_read(h, buf, len);
}

static inline core_status_t core_spi_xfer(core_spi_t *h, const uint8_t *tx,
                                           uint8_t *rx, uint32_t len)
{
    return hal_spi_xfer(h, tx, rx, len);
}

static inline core_status_t core_spi_write_read(core_spi_t *h, const uint8_t *tx,
                                                 uint32_t tx_len, uint8_t *rx,
                                                 uint32_t rx_len)
{
    return hal_spi_write_read(h, tx, tx_len, rx, rx_len);
}

static inline core_status_t core_spi_exchange_dma(core_spi_t *h, const uint8_t *tx,
                                                   uint8_t *rx, uint32_t len)
{
    return hal_spi_exchange_dma(h, tx, rx, len);
}

static inline core_status_t core_spi_xfer_dma(core_spi_t *h,
                                               const uint8_t *tx,
                                               uint8_t *rx,
                                               uint32_t len,
                                               core_callback_t cb, void *ctx)
{
    return hal_spi_xfer_dma(h, tx, rx, len, cb, ctx);
}

static inline core_status_t core_spi_dma_wait(core_spi_t *h)
{
    return hal_spi_dma_wait(h);
}

static inline int core_spi_busy(core_spi_t *h)
{
    return hal_spi_busy(h);
}

static inline int core_spi_xfer_byte_bus(uint8_t bus, uint8_t cs_pad, uint8_t tx)
{
    hal_spi_t *h = core_spi_handle_for_bus(bus);
    if (!h) return -1;
    hal_pad_gpio_t cs = hal_pad_lookup(cs_pad);
    if (!cs.port) return -1;
    if (h->cs_port != cs.port || h->cs_pin != cs.pin)
        hal_spi_set_cs(h, cs.port, cs.pin);
    uint8_t rx = 0xFF;
    if (hal_spi_xfer(h, &tx, &rx, 1) != HAL_OK) return -1;
    return (int)rx;
}

#endif /* PORT_CORE_SPI_H */
