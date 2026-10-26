/**
 * port_core_i2c_types.h — STM32 port: I2C types for core_i2c.h
 *
 * Included by core_i2c.h before its prototypes; not for direct use. The
 * value checks and the bodies of its functions are in port_core_i2c.h.
 */

#ifndef PORT_CORE_I2C_TYPES_H
#define PORT_CORE_I2C_TYPES_H

#include "hal_i2c.h"

/** An I2C peripheral instance: I2C1, I2C3, ... */
typedef I2C_TypeDef core_i2c_instance_t;

/** Explicit bus configuration for core_i2c_init_cfg() (TIMINGR, timeout, FM+). */
typedef hal_i2c_config_t core_i2c_config_t;

/** A target-mode event, one of CORE_I2C_TARGET_*. */
typedef hal_i2c_target_event_t core_i2c_target_event_t;

#endif /* PORT_CORE_I2C_TYPES_H */
