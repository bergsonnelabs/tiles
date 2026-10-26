/**
 * port_core_tiles.h — STM32 port: the bus adapters behind core_tiles.h
 *
 * Included at the end of core_tiles.h, after its prototypes; not for direct
 * use. Fills a tiles_pal_t with adapters over core_i2c / core_spi / core_i3c
 * and core_pad; the SPI chip select goes through hal_spi, the I3C transfers
 * through hal_i3c, and the PAL's delay_ms is ll_delay_ms.
 */

#ifndef PORT_CORE_TILES_H
#define PORT_CORE_TILES_H

#include "ll_systick.h"

/* ---- Internal: I2C adapters ---- */

static inline int _ct_i2c_read(void *h, uint8_t addr, uint16_t reg,
                               uint8_t *data, uint16_t len)
{
    return core_i2c_read_reg((core_i2c_t *)h, addr, reg, data, len);
}

static inline int _ct_i2c_write(void *h, uint8_t addr, uint16_t reg,
                                const uint8_t *data, uint16_t len)
{
    return core_i2c_write_reg((core_i2c_t *)h, addr, reg, data, len);
}

static inline int _ct_i2c_ready(void *h, uint8_t addr)
{
    return core_i2c_probe((core_i2c_t *)h, addr);
}

static inline int _ct_i2c_write_raw(void *h, uint8_t addr,
                                    const uint8_t *data, uint16_t len)
{
    return core_i2c_write((core_i2c_t *)h, addr, data, len);
}

static inline int _ct_i2c_read_raw(void *h, uint8_t addr,
                                   uint8_t *data, uint16_t len)
{
    return core_i2c_read((core_i2c_t *)h, addr, data, len);
}

/* ---- Internal: SPI adapters (the chip-select rules are in core_tiles.h) ---- */
#ifdef _CORE_TILES_HAS_SPI

/* Select `cs`, clock out a then b (received bytes discarded), then clock in
 * rn bytes (sending the fill byte), release. */
static inline int _ct_spi_txn(core_spi_t *spi, uint8_t cs,
                              const uint8_t *a, uint16_t an,
                              const uint8_t *b, uint16_t bn,
                              uint8_t *rx, uint16_t rn)
{
    if (hal_spi_select_id(spi, cs) != 0)
        return -1;
    hal_status_t s = core_spi_exchange(spi, a, NULL, an);
    if (s == HAL_OK) s = core_spi_exchange(spi, b, NULL, bn);
    if (s == HAL_OK) s = core_spi_exchange(spi, NULL, rx, rn);
    hal_spi_deselect_id(spi, cs);
    return s == HAL_OK ? 0 : -1;
}

static inline int _ct_spi_read(void *h, uint8_t cs, uint8_t reg,
                               uint8_t *data, uint16_t len)
{
    uint8_t cmd = reg | 0x80;
    return _ct_spi_txn((core_spi_t *)h, cs, &cmd, 1, NULL, 0, data, len);
}

static inline int _ct_spi_write(void *h, uint8_t cs, uint8_t reg,
                                const uint8_t *data, uint16_t len)
{
    uint8_t cmd = reg & 0x7F;
    return _ct_spi_txn((core_spi_t *)h, cs, &cmd, 1, data, len, NULL, 0);
}

/* Raw transaction (tiles_pal.h spi_transfer): one CS assertion, send tx_len
 * bytes, then clock in rx_len bytes (0xFF on MOSI), release CS. For command +
 * address + data protocols such as SPI-NOR (Store.O.128). */
static inline int _ct_spi_transfer(void *h, uint8_t cs,
                                   const uint8_t *tx, uint16_t tx_len,
                                   uint8_t *rx, uint16_t rx_len)
{
    if ((tx_len && !tx) || (rx_len && !rx))
        return -1;
    return _ct_spi_txn((core_spi_t *)h, cs, tx, tx_len, NULL, 0, rx, rx_len);
}

#endif /* _CORE_TILES_HAS_SPI */

/* ---- Internal: I3C adapters (Core.ST.H5; behavior in core_tiles.h) ---- */
#ifdef _CORE_TILES_HAS_I3C

static inline uint8_t _ct_i3c_da(core_i3c_t *bus, uint8_t addr)
{
    return core_i3c_attach(bus, addr);
}

static inline int _ct_i3c_reattach(core_i3c_t *bus, uint8_t addr)
{
    hal_i3c_target_t *t = hal_i3c_target_by_static(bus, addr);
    return t && hal_i3c_setdasa(bus, addr, t->dyn_addr) == HAL_OK;
}

static inline int _ct_i3c_read(void *h, uint8_t addr, uint16_t reg,
                               uint8_t *data, uint16_t len)
{
    core_i3c_t *bus = (core_i3c_t *)h;
    uint8_t da = _ct_i3c_da(bus, addr);
    if (!da) return -1;
    hal_status_t s = hal_i3c_read_reg(bus, da, reg, data, len);
    if (s == HAL_NACK && _ct_i3c_reattach(bus, addr))
        s = hal_i3c_read_reg(bus, da, reg, data, len);
    return s == HAL_OK ? 0 : -1;
}

static inline int _ct_i3c_write(void *h, uint8_t addr, uint16_t reg,
                                const uint8_t *data, uint16_t len)
{
    core_i3c_t *bus = (core_i3c_t *)h;
    uint8_t da = _ct_i3c_da(bus, addr);
    if (!da) return -1;
    hal_status_t s = hal_i3c_write_reg(bus, da, reg, data, len);
    if (s == HAL_NACK && _ct_i3c_reattach(bus, addr))
        s = hal_i3c_write_reg(bus, da, reg, data, len);
    return s == HAL_OK ? 0 : -1;
}

/* Presence only: the target acknowledged its address. A GETSTATUS it cut
 * short (CE0, SER.PERR with CODERR 0) still counts, since before its driver
 * has configured it a target may not return clean read data yet (the
 * ICM-42686-P at its reset SDA slew, see tile_sense_i_6p6.c). */
static inline int _ct_i3c_ready(void *h, uint8_t addr)
{
    core_i3c_t *bus = (core_i3c_t *)h;
    uint8_t da = _ct_i3c_da(bus, addr);
    if (!da) return -1;
    uint16_t st;
    hal_status_t s = hal_i3c_get_status(bus, da, &st);
    if (s == HAL_NACK && _ct_i3c_reattach(bus, addr))
        s = hal_i3c_get_status(bus, da, &st);
    if (s == HAL_ERROR && (bus->last_ser & LL_I3C_SER_PERR) &&
        (bus->last_ser & LL_I3C_SER_CODERR_MASK) == 0)
        s = HAL_OK;
    return s == HAL_OK ? 0 : -1;
}

static inline int _ct_i3c_write_raw(void *h, uint8_t addr,
                                    const uint8_t *data, uint16_t len)
{
    core_i3c_t *bus = (core_i3c_t *)h;
    uint8_t da = _ct_i3c_da(bus, addr);
    if (!da) return -1;
    hal_status_t s = hal_i3c_write(bus, da, data, len);
    if (s == HAL_NACK && _ct_i3c_reattach(bus, addr))
        s = hal_i3c_write(bus, da, data, len);
    return s == HAL_OK ? 0 : -1;
}

static inline int _ct_i3c_read_raw(void *h, uint8_t addr,
                                   uint8_t *data, uint16_t len)
{
    core_i3c_t *bus = (core_i3c_t *)h;
    uint8_t da = _ct_i3c_da(bus, addr);
    if (!da) return -1;
    hal_status_t s = hal_i3c_read(bus, da, data, len, 0);
    if (s == HAL_NACK && _ct_i3c_reattach(bus, addr))
        s = hal_i3c_read(bus, da, data, len, 0);
    return s == HAL_OK ? 0 : -1;
}

#endif /* _CORE_TILES_HAS_I3C */

/* ---- Internal: shared adapters ---- */

static inline int _ct_gpio_irq_enable(void *h, uint8_t pin, uint8_t edge,
                                      void (*cb)(void *), void *ctx)
{
    (void)h;
    uint32_t e = (edge == TILES_GPIO_EDGE_FALLING) ? EDGE_FALLING
               : (edge == TILES_GPIO_EDGE_RISING)  ? EDGE_RISING
               : EDGE_BOTH;
    return core_pad_on_change(pin, e, cb, ctx);
}

static inline void _ct_gpio_irq_disable(void *h, uint8_t pin)
{
    (void)h;
    core_pad_on_change_stop(pin);
}

/* ---- Internal: typed constructors ---- */

static inline tiles_pal_t *_core_tiles_pal_i2c(core_i2c_t *bus)
{
    enum { CT_MAX = 4 };
    static tiles_pal_t hals[CT_MAX];
    static void *keys[CT_MAX];
    static uint8_t count = 0;

    for (uint8_t i = 0; i < count; i++)
        if (keys[i] == bus)
            return &hals[i];

    if (count >= CT_MAX)
        return &hals[0];

    uint8_t i = count++;
    keys[i] = bus;
    hals[i].i2c_read        = _ct_i2c_read;
    hals[i].i2c_write       = _ct_i2c_write;
    hals[i].i2c_is_ready    = _ct_i2c_ready;
    hals[i].i2c_write_raw   = _ct_i2c_write_raw;
    hals[i].i2c_read_raw    = _ct_i2c_read_raw;
    hals[i].gpio_irq_enable = _ct_gpio_irq_enable;
    hals[i].gpio_irq_disable = _ct_gpio_irq_disable;
    hals[i].delay_ms        = ll_delay_ms;
    hals[i].buses           = TILES_BUS_I2C;
    hals[i].handle          = bus;
    return &hals[i];
}

#ifdef _CORE_TILES_HAS_I3C
static inline tiles_pal_t *_core_tiles_pal_i3c(core_i3c_t *bus)
{
    enum { CT_MAX = 4 };
    static tiles_pal_t hals[CT_MAX];
    static void *keys[CT_MAX];
    static uint8_t count = 0;

    for (uint8_t i = 0; i < count; i++)
        if (keys[i] == bus)
            return &hals[i];

    if (count >= CT_MAX)
        return &hals[0];

    uint8_t i = count++;
    keys[i] = bus;
    hals[i].i2c_read        = _ct_i3c_read;
    hals[i].i2c_write       = _ct_i3c_write;
    hals[i].i2c_is_ready    = _ct_i3c_ready;
    hals[i].i2c_write_raw   = _ct_i3c_write_raw;
    hals[i].i2c_read_raw    = _ct_i3c_read_raw;
    hals[i].gpio_irq_enable = _ct_gpio_irq_enable;
    hals[i].gpio_irq_disable = _ct_gpio_irq_disable;
    hals[i].delay_ms        = ll_delay_ms;
    hals[i].buses           = TILES_BUS_I2C | TILES_BUS_I3C;
    hals[i].handle          = bus;
    return &hals[i];
}
#endif /* _CORE_TILES_HAS_I3C */

#ifdef _CORE_TILES_HAS_SPI
static inline tiles_pal_t *_core_tiles_pal_spi(core_spi_t *bus)
{
    enum { CT_MAX = 4 };
    static tiles_pal_t hals[CT_MAX];
    static void *keys[CT_MAX];
    static uint8_t count = 0;

    for (uint8_t i = 0; i < count; i++)
        if (keys[i] == bus)
            return &hals[i];

    if (count >= CT_MAX)
        return &hals[0];

    uint8_t i = count++;
    keys[i] = bus;
    hals[i].spi_read        = _ct_spi_read;
    hals[i].spi_write       = _ct_spi_write;
    hals[i].spi_transfer    = _ct_spi_transfer;
    hals[i].gpio_irq_enable = _ct_gpio_irq_enable;
    hals[i].gpio_irq_disable = _ct_gpio_irq_disable;
    hals[i].delay_ms        = ll_delay_ms;
    hals[i].buses           = TILES_BUS_SPI;
    hals[i].handle          = bus;
    return &hals[i];
}

/* ---- Internal: dual-bus (I2C + SPI) adapters: unpack core_tiles_dual_t ---- */
static inline int _ct2_i2c_read(void *h, uint8_t a, uint16_t r, uint8_t *d, uint16_t n)
{ return _ct_i2c_read(((core_tiles_dual_t *)h)->i2c, a, r, d, n); }
static inline int _ct2_i2c_write(void *h, uint8_t a, uint16_t r, const uint8_t *d, uint16_t n)
{ return _ct_i2c_write(((core_tiles_dual_t *)h)->i2c, a, r, d, n); }
static inline int _ct2_i2c_ready(void *h, uint8_t a)
{ return _ct_i2c_ready(((core_tiles_dual_t *)h)->i2c, a); }
static inline int _ct2_i2c_write_raw(void *h, uint8_t a, const uint8_t *d, uint16_t n)
{ return _ct_i2c_write_raw(((core_tiles_dual_t *)h)->i2c, a, d, n); }
static inline int _ct2_i2c_read_raw(void *h, uint8_t a, uint8_t *d, uint16_t n)
{ return _ct_i2c_read_raw(((core_tiles_dual_t *)h)->i2c, a, d, n); }
static inline int _ct2_spi_read(void *h, uint8_t cs, uint8_t r, uint8_t *d, uint16_t n)
{ return _ct_spi_read(((core_tiles_dual_t *)h)->spi, cs, r, d, n); }
static inline int _ct2_spi_write(void *h, uint8_t cs, uint8_t r, const uint8_t *d, uint16_t n)
{ return _ct_spi_write(((core_tiles_dual_t *)h)->spi, cs, r, d, n); }
static inline int _ct2_spi_transfer(void *h, uint8_t cs, const uint8_t *tx, uint16_t tn,
                                    uint8_t *rx, uint16_t rn)
{ return _ct_spi_transfer(((core_tiles_dual_t *)h)->spi, cs, tx, tn, rx, rn); }

static inline tiles_pal_t *core_tiles_pal2(core_i2c_t *i2c, core_spi_t *spi)
{
    enum { CT_MAX = 4 };
    static tiles_pal_t hals[CT_MAX];
    static core_tiles_dual_t pairs[CT_MAX];
    static uint8_t count = 0;

    for (uint8_t i = 0; i < count; i++)
        if (pairs[i].i2c == i2c && pairs[i].spi == spi)
            return &hals[i];

    if (count >= CT_MAX)
        return &hals[0];

    uint8_t i = count++;
    pairs[i].i2c = i2c;
    pairs[i].spi = spi;
    hals[i].i2c_read        = _ct2_i2c_read;
    hals[i].i2c_write       = _ct2_i2c_write;
    hals[i].i2c_is_ready    = _ct2_i2c_ready;
    hals[i].i2c_write_raw   = _ct2_i2c_write_raw;
    hals[i].i2c_read_raw    = _ct2_i2c_read_raw;
    hals[i].spi_read        = _ct2_spi_read;
    hals[i].spi_write       = _ct2_spi_write;
    hals[i].spi_transfer    = _ct2_spi_transfer;
    hals[i].gpio_irq_enable = _ct_gpio_irq_enable;
    hals[i].gpio_irq_disable = _ct_gpio_irq_disable;
    hals[i].delay_ms        = ll_delay_ms;
    hals[i].buses           = TILES_BUS_I2C | TILES_BUS_SPI;
    hals[i].handle          = &pairs[i];
    return &hals[i];
}
#endif /* _CORE_TILES_HAS_SPI */

#endif /* PORT_CORE_TILES_H */
