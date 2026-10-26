/**
 * port_core_types.h — STM32 port: the types behind core_types.h
 *
 * Include core_types.h, not this file. Each name here is a typedef or alias
 * of the STM32 HAL's own, so the core_ API keeps the exact ABI and code it
 * had: core_status_t is hal_status_t, CORE_OK is HAL_OK, and so on. Another
 * port (nRF) ships its own port_core_types.h with the same names.
 *
 * Per-subsystem types (peripheral instances, handles, callbacks with HAL
 * types in their signature) and the checks that a neutral constant equals
 * the LL / HAL value it replaces live in port_core_<subsystem>.h beside this
 * file, included by the matching core_<subsystem>.h.
 */

#ifndef PORT_CORE_TYPES_H
#define PORT_CORE_TYPES_H

#include "hal_common.h"

/** Storage class of the core_ functions defined in headers. */
#define CORE_INLINE static inline

/* ---- Status ---- */

typedef hal_status_t core_status_t;

#define CORE_OK       HAL_OK        /*  0 */
#define CORE_ERROR    HAL_ERROR     /* -1 */
#define CORE_BUSY     HAL_BUSY      /* -2 */
#define CORE_TIMEOUT  HAL_TIMEOUT   /* -3 */
#define CORE_NACK     HAL_NACK      /* -4 */

/* ---- Checks: the neutral types are the HAL's ---- */

#ifndef __cplusplus
_Static_assert(__builtin_types_compatible_p(core_callback_t, hal_callback_t),
               "core_callback_t must be hal_callback_t");
#endif

#endif /* PORT_CORE_TYPES_H */
