/**
 * port_core_pad.h — STM32 port: value checks and bodies for core_pad.h
 *
 * Included at the end of core_pad.h, after its prototypes; not for direct
 * use. The checks hold the neutral CORE_PULL_* / CORE_OTYPE_* / CORE_SPEED_* /
 * CORE_EDGE_* values to the LL values they are passed straight through as.
 * The bodies go through hal_gpio (pad -> port/pin), the GPIO BSRR/IDR
 * registers (the fast path) and tal_exti. The types are in
 * port_core_pad_types.h.
 */

#ifndef PORT_CORE_PAD_H
#define PORT_CORE_PAD_H

#include "hal_gpio.h"
#include "port_core_pad_types.h"
#include "ll_common.h"   /* LL_GPIO_PULL_* / _OTYPE_* / _SPEED_* */
#include "ll_exti.h"     /* LL_EXTI_RISING / _FALLING / _BOTH */

#ifndef __cplusplus
_Static_assert(CORE_PULL_NONE == LL_GPIO_PULL_NONE, "CORE_PULL_NONE");
_Static_assert(CORE_PULL_UP   == LL_GPIO_PULL_UP,   "CORE_PULL_UP");
_Static_assert(CORE_PULL_DOWN == LL_GPIO_PULL_DOWN, "CORE_PULL_DOWN");
_Static_assert(CORE_OTYPE_PP  == LL_GPIO_OTYPE_PP,  "CORE_OTYPE_PP");
_Static_assert(CORE_OTYPE_OD  == LL_GPIO_OTYPE_OD,  "CORE_OTYPE_OD");
_Static_assert(CORE_SPEED_LOW   == LL_GPIO_SPEED_LOW,   "CORE_SPEED_LOW");
_Static_assert(CORE_SPEED_MED   == LL_GPIO_SPEED_MED,   "CORE_SPEED_MED");
_Static_assert(CORE_SPEED_HIGH  == LL_GPIO_SPEED_HIGH,  "CORE_SPEED_HIGH");
_Static_assert(CORE_SPEED_VHIGH == LL_GPIO_SPEED_VHIGH, "CORE_SPEED_VHIGH");
_Static_assert(CORE_EDGE_RISING  == LL_EXTI_RISING,  "CORE_EDGE_RISING");
_Static_assert(CORE_EDGE_FALLING == LL_EXTI_FALLING, "CORE_EDGE_FALLING");
_Static_assert(CORE_EDGE_BOTH    == LL_EXTI_BOTH,    "CORE_EDGE_BOTH");
#endif

static inline void core_pad_output(uint8_t pad)
{
    hal_pad_output(pad);
}

static inline void core_pad_output_od(uint8_t pad, uint32_t pull)
{
    hal_pad_output_od(pad, pull);
}

static inline void core_pad_input(uint8_t pad, uint32_t pull)
{
    hal_pad_input(pad, pull);
}

static inline void core_pad_write(uint8_t pad, int state)
{
    hal_pad_write(pad, state);
}

static inline int core_pad_read(uint8_t pad)
{
    return hal_pad_read(pad);
}

static inline void core_pad_toggle(uint8_t pad)
{
    hal_pad_toggle(pad);
}

static inline core_pad_fast_t core_pad_resolve(uint8_t pad)
{
    hal_pad_gpio_t g = hal_pad_lookup(pad);
    core_pad_fast_t h = { g.port, g.port ? (1UL << g.pin) : 0UL };
    return h;
}

static inline void core_pad_fast_set(core_pad_fast_t h)   { h.port->BSRR = h.mask; }
static inline void core_pad_fast_clear(core_pad_fast_t h) { h.port->BSRR = h.mask << 16; }
static inline void core_pad_fast_write(core_pad_fast_t h, int state)
{
    h.port->BSRR = state ? h.mask : (h.mask << 16);
}
static inline int core_pad_fast_read(core_pad_fast_t h)
{
    return (h.port->IDR & h.mask) ? 1 : 0;
}

static inline void core_pad_bulk_write(core_pad_port_t *port,
                                       uint32_t set_mask, uint32_t clear_mask)
{
    port->BSRR = set_mask | (clear_mask << 16);
}

static inline void core_pad_analog(uint8_t pad)
{
    hal_pad_analog(pad);
}

static inline void core_pad_speed(uint8_t pad, uint32_t speed)
{
    hal_pad_gpio_t g = hal_pad_lookup(pad);
    if (!g.port) return;
    ll_gpio_set_speed(g.port, g.pin, speed);
}

#include "tal_exti.h"

static inline core_status_t core_pad_on_change(uint8_t pad, uint32_t edge,
                                                core_pad_cb_t cb, void *ctx)
{
    return tal_exti_enable(pad, edge, cb, ctx);
}

static inline void core_pad_on_change_stop(uint8_t pad)
{
    tal_exti_disable(pad);
}

#endif /* PORT_CORE_PAD_H */
