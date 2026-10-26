/**
 * port_core_i2c.h — STM32 port: value checks and bodies for core_i2c.h
 *
 * Included at the end of core_i2c.h, after its prototypes; not for direct
 * use. The types are in port_core_i2c_types.h.
 */

#ifndef PORT_CORE_I2C_H
#define PORT_CORE_I2C_H

#include "hal_i2c.h"
#include "port_core_i2c_types.h"
#include "core_config.h"   /* I2C_KERNEL_CLK_MHZ: the TIMINGR lookup */

#ifndef __cplusplus
_Static_assert(CORE_I2C_TARGET_READ_START == HAL_I2C_TARGET_READ_START, "CORE_I2C_TARGET_READ_START");
_Static_assert(CORE_I2C_TARGET_WRITE      == HAL_I2C_TARGET_WRITE,      "CORE_I2C_TARGET_WRITE");
_Static_assert(CORE_I2C_TARGET_STOP       == HAL_I2C_TARGET_STOP,       "CORE_I2C_TARGET_STOP");
#endif

static inline uint32_t _core_i2c_timing(uint32_t speed_hz)
{
    switch (speed_hz) {
        case I2C_100K: return ll_i2c_timing_100k(I2C_KERNEL_CLK_MHZ);
        case I2C_400K: return ll_i2c_timing_400k(I2C_KERNEL_CLK_MHZ);
        case I2C_1M:   return ll_i2c_timing_1m(I2C_KERNEL_CLK_MHZ);
        default:       return 0;
    }
}

static inline core_status_t core_i2c_init(core_i2c_t *h,
                                           core_i2c_instance_t *instance,
                                           uint32_t speed_hz)
{
    uint32_t timing = _core_i2c_timing(speed_hz);
    if (!timing) return HAL_ERROR;
    hal_i2c_config_t cfg = {
        .timing = timing,
        .timeout_ms = 100,
        .fmp = (speed_hz >= I2C_1M) ? 1 : 0,
    };
    return hal_i2c_init(h, instance, &cfg);
}

static inline core_status_t core_i2c_init_cfg(core_i2c_t *h,
                                               core_i2c_instance_t *instance,
                                               const core_i2c_config_t *cfg)
{
    return hal_i2c_init(h, instance, cfg);
}

static inline core_status_t core_i2c_write(core_i2c_t *h, uint8_t addr,
                                            const uint8_t *data, uint32_t len)
{
    return hal_i2c_write(h, addr, data, len);
}

static inline core_status_t core_i2c_read(core_i2c_t *h, uint8_t addr,
                                           uint8_t *buf, uint32_t len)
{
    return hal_i2c_read(h, addr, buf, len);
}

static inline core_status_t core_i2c_write_reg(core_i2c_t *h, uint8_t addr,
                                                uint16_t reg,
                                                const uint8_t *data,
                                                uint32_t len)
{
    return hal_i2c_write_reg(h, addr, reg, data, len);
}

static inline core_status_t core_i2c_read_reg(core_i2c_t *h, uint8_t addr,
                                               uint16_t reg, uint8_t *buf,
                                               uint32_t len)
{
    return hal_i2c_read_reg(h, addr, reg, buf, len);
}

static inline core_status_t core_i2c_write_byte(core_i2c_t *h, uint8_t addr,
                                                 uint16_t reg, uint8_t value)
{
    return hal_i2c_write_byte(h, addr, reg, value);
}

static inline core_status_t core_i2c_read_byte(core_i2c_t *h, uint8_t addr,
                                                uint16_t reg, uint8_t *value)
{
    return hal_i2c_read_byte(h, addr, reg, value);
}

static inline core_status_t core_i2c_probe(core_i2c_t *h, uint8_t addr)
{
    return hal_i2c_probe(h, addr);
}

static inline void core_i2c_scan(core_i2c_t *h, uint8_t *found,
                                  uint8_t *count, uint8_t max_count)
{
    hal_i2c_scan(h, found, count, max_count);
}

static inline core_status_t core_i2c_write_byte_bus(uint8_t bus, uint8_t addr,
                                                     uint16_t reg, uint8_t value)
{
    hal_i2c_t *h = core_i2c_handle_for_bus(bus);
    if (!h) return HAL_ERROR;
    return hal_i2c_write_byte(h, addr, reg, value);
}

static inline int core_i2c_read_byte_bus(uint8_t bus, uint8_t addr, uint16_t reg)
{
    hal_i2c_t *h = core_i2c_handle_for_bus(bus);
    if (!h) return -1;
    uint8_t v = 0;
    if (hal_i2c_read_byte(h, addr, reg, &v) != HAL_OK) return -1;
    return (int)v;
}

static inline int core_i2c_probe_bus(uint8_t bus, uint8_t addr)
{
    hal_i2c_t *h = core_i2c_handle_for_bus(bus);
    if (!h) return 0;
    return hal_i2c_probe(h, addr) == HAL_OK ? 1 : 0;
}

static inline core_status_t core_i2c_target_init(core_i2c_target_t *t,
                                                 core_i2c_instance_t *instance,
                                                 uint8_t addr, uint32_t speed,
                                                 uint8_t *regs, uint16_t n_regs,
                                                 core_i2c_target_cb_t cb,
                                                 void *ctx)
{
    uint32_t timing = _core_i2c_timing(speed);
    if (!timing) return HAL_ERROR;
    hal_i2c_target_config_t cfg = {
        .addr           = addr,
        .timing         = timing,
        .regs           = regs,
        .n_regs         = n_regs,
        .writable_first = 0,
        .writable_count = 0,
        .cb             = cb,
        .ctx            = ctx,
    };
    return hal_i2c_target_init(t, instance, &cfg);
}

static inline core_status_t core_i2c_target_init_rw(core_i2c_target_t *t,
                                                    core_i2c_instance_t *instance,
                                                    uint8_t addr, uint32_t speed,
                                                    uint8_t *regs, uint16_t n_regs,
                                                    uint16_t first, uint16_t count,
                                                    core_i2c_target_cb_t cb,
                                                    void *ctx)
{
    uint32_t timing = _core_i2c_timing(speed);
    if (!timing) return HAL_ERROR;
    hal_i2c_target_config_t cfg = {
        .addr           = addr,
        .timing         = timing,
        .regs           = regs,
        .n_regs         = n_regs,
        .writable_first = first,
        .writable_count = count,
        .cb             = cb,
        .ctx            = ctx,
    };
    return hal_i2c_target_init(t, instance, &cfg);
}

static inline void core_i2c_target_deinit(core_i2c_target_t *t)
{
    hal_i2c_target_deinit(t);
}

#endif /* PORT_CORE_I2C_H */
