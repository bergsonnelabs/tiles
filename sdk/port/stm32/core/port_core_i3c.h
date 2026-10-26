/**
 * port_core_i3c.h — STM32 port: the bodies of core_i3c.h
 *
 * Included at the end of core_i3c.h, after its prototypes; not for direct
 * use. The types are in port_core_i3c_types.h.
 */

#ifndef PORT_CORE_I3C_H
#define PORT_CORE_I3C_H

#include "hal_i3c.h"
#include "port_core_i3c_types.h"
#include "core_config.h"   /* HCLK_HZ, PCLK1_HZ: the I3C kernel clock */

static inline core_status_t core_i3c_init(core_i3c_t *h, core_i3c_instance_t *instance, uint32_t scl_hz)
{
    hal_i3c_config_t cfg = {
        .scl_hz     = scl_hz,
        .od_low_ns  = 2000,
        .kernel     = HAL_I3C_KERNEL_PCLK,
        .pclk_hz    = (instance == I3C2) ? HCLK_HZ : PCLK1_HZ,
        .timeout_ms = 10,
    };
    return hal_i3c_init(h, instance, &cfg);
}

static inline void core_i3c_deinit(core_i3c_t *h)
{
    hal_i3c_deinit(h);
}

static inline core_status_t core_i3c_rstdaa(core_i3c_t *h)
{
    return hal_i3c_rstdaa(h);
}

static inline core_status_t core_i3c_setdasa(core_i3c_t *h, uint8_t static_addr, uint8_t dyn_addr)
{
    return hal_i3c_setdasa(h, static_addr, dyn_addr);
}

static inline core_status_t core_i3c_entdaa(core_i3c_t *h, uint8_t *found)
{
    return hal_i3c_entdaa(h, found);
}

static inline uint8_t core_i3c_attach(core_i3c_t *h, uint8_t static_addr)
{
    hal_i3c_target_t *t = hal_i3c_target_by_static(h, static_addr);
    if (!t && hal_i3c_setdasa(h, static_addr, 0) == HAL_OK)
        t = hal_i3c_target_by_static(h, static_addr);
    return t ? t->dyn_addr : 0;
}

static inline core_status_t core_i3c_get_pid(core_i3c_t *h, uint8_t dyn_addr, uint8_t pid[6])
{
    return hal_i3c_get_pid(h, dyn_addr, pid);
}

static inline core_status_t core_i3c_get_bcr(core_i3c_t *h, uint8_t dyn_addr, uint8_t *bcr)
{
    return hal_i3c_get_bcr(h, dyn_addr, bcr);
}

static inline core_status_t core_i3c_get_dcr(core_i3c_t *h, uint8_t dyn_addr, uint8_t *dcr)
{
    return hal_i3c_get_dcr(h, dyn_addr, dcr);
}

static inline core_status_t core_i3c_get_status(core_i3c_t *h, uint8_t dyn_addr, uint16_t *status)
{
    return hal_i3c_get_status(h, dyn_addr, status);
}

static inline core_status_t core_i3c_enec(core_i3c_t *h, uint8_t dyn_addr, uint8_t events)
{
    return hal_i3c_enec(h, dyn_addr, events);
}

static inline core_status_t core_i3c_disec(core_i3c_t *h, uint8_t dyn_addr, uint8_t events)
{
    return hal_i3c_disec(h, dyn_addr, events);
}

static inline core_status_t core_i3c_write(core_i3c_t *h, uint8_t dyn_addr,
                                           const uint8_t *data, uint16_t len)
{
    return hal_i3c_write(h, dyn_addr, data, len);
}

static inline core_status_t core_i3c_read(core_i3c_t *h, uint8_t dyn_addr,
                                          uint8_t *buf, uint16_t len)
{
    return hal_i3c_read(h, dyn_addr, buf, len, 0);
}

static inline core_status_t core_i3c_write_reg(core_i3c_t *h, uint8_t dyn_addr, uint16_t reg,
                                               const uint8_t *data, uint16_t len)
{
    return hal_i3c_write_reg(h, dyn_addr, reg, data, len);
}

static inline core_status_t core_i3c_read_reg(core_i3c_t *h, uint8_t dyn_addr, uint16_t reg,
                                              uint8_t *buf, uint16_t len)
{
    return hal_i3c_read_reg(h, dyn_addr, reg, buf, len);
}

static inline core_status_t core_i3c_ibi_enable(core_i3c_t *h, uint8_t dyn_addr,
                                                core_i3c_ibi_cb_t cb, void *ctx)
{
    return hal_i3c_ibi_enable(h, dyn_addr, cb, ctx);
}

static inline core_status_t core_i3c_ibi_disable(core_i3c_t *h, uint8_t dyn_addr)
{
    return hal_i3c_ibi_disable(h, dyn_addr);
}

#endif /* PORT_CORE_I3C_H */
