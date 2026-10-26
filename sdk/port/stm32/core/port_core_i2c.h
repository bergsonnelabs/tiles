/**
 * port_core_i2c.h — STM32 port: I2C types and value checks for core_i2c.h
 *
 * Included by core_i2c.h after its neutral constants; not for direct use.
 */

#ifndef PORT_CORE_I2C_H
#define PORT_CORE_I2C_H

#include "hal_i2c.h"

/** An I2C peripheral instance: I2C1, I2C3, ... */
typedef I2C_TypeDef core_i2c_instance_t;

/** Explicit bus configuration for core_i2c_init_cfg() (TIMINGR, timeout, FM+). */
typedef hal_i2c_config_t core_i2c_config_t;

/** A target-mode event, one of CORE_I2C_TARGET_*. */
typedef hal_i2c_target_event_t core_i2c_target_event_t;

#ifndef __cplusplus
_Static_assert(CORE_I2C_TARGET_READ_START == HAL_I2C_TARGET_READ_START, "CORE_I2C_TARGET_READ_START");
_Static_assert(CORE_I2C_TARGET_WRITE      == HAL_I2C_TARGET_WRITE,      "CORE_I2C_TARGET_WRITE");
_Static_assert(CORE_I2C_TARGET_STOP       == HAL_I2C_TARGET_STOP,       "CORE_I2C_TARGET_STOP");
#endif

#endif /* PORT_CORE_I2C_H */
