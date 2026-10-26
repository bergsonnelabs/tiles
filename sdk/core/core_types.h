/**
 * core_types.h — Vendor-neutral types shared by the core_ API
 *
 * What every port provides under the same names, so a core_ prototype never
 * names a vendor's type:
 *
 *   core_status_t     what a fallible core_ call returns: CORE_OK (0) or a
 *                     negative CORE_ERROR / CORE_BUSY / CORE_TIMEOUT /
 *                     CORE_NACK
 *   core_callback_t   generic `void cb(void *ctx)` completion / ISR callback
 *   CORE_INLINE       the storage class of a core_ function defined in a
 *                     header (`static inline` on STM32)
 *
 * Naming across the core_ headers: core_<x>_t is a handle (core_i2c_t,
 * core_serial_t), core_<x>_instance_t a peripheral instance (core_i2c_instance_t,
 * what I2C1 points to), core_<x>_cb_t a callback. Subsystem types live with
 * their subsystem: the neutral header declares what it can (callbacks with
 * plain C signatures), the port's port_core_<x>.h the rest.
 *
 * The values come from the port: this header includes port_core_types.h,
 * which each port ships in its own include directory (the STM32 one is
 * sdk/port/stm32/core/port_core_types.h, aliases of the hal_ types).
 */

#ifndef CORE_TYPES_H
#define CORE_TYPES_H

#include <stdint.h>

/** Generic callback: completion or interrupt hook with the caller's context. */
typedef void (*core_callback_t)(void *ctx);

#include "port_core_types.h"

#ifndef CORE_INLINE
#define CORE_INLINE static inline
#endif

#endif /* CORE_TYPES_H */
