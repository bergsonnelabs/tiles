/**
 * core_pad.h — Pad-level GPIO for tile pad numbers
 *
 * The primary GPIO API. All functions take tile pad numbers
 * (matching silkscreen) and resolve to hardware automatically.
 *
 * Edge-triggered events fire for any pad whose config.json
 * `gpio.<pad>.exti` is set to `rising`, `falling`, or `both`. Coregen
 * wires the EXTI interrupts into a pending-bit bitmap; the Studio
 * main loop drains it and calls the DSL `on Core.Pad.*` handler once
 * per pending pad with the pad number as payload.
 *
 * Fires once per rising edge on any pad whose config.json gpio.<pad>.exti
 * is set to "rising" or "both". Likewise for falling. Coregen wires the
 * EXTI callbacks; the Studio main loop drains the pending bitmap and
 * calls the DSL handler with the pad number as payload.
 *
 * @studio category pad label=Core.Pad icon=◼
 * @studio event name=rising payload=pad:int
 * @studio event name=falling payload=pad:int
 *
 * @studio coverage
 *   id:    gpio
 *   name:  GPIO — pad-level I/O
 *   page:  /docs/sdk/gpio
 *   blurb: Pad-numbered GPIO API. Tier 2 helpers (write/read/toggle) take
 *          a pad number and resolve the port/pin from coregen's tables.
 *          Tier 1 helpers configure pad mode (output / open-drain / input
 *          / analog / speed) and EXTI callbacks — they sit below the DSL
 *          surface, called by the framework at init and by escape-to-C
 *          users who want explicit control.
 */

#ifndef CORE_PAD_H
#define CORE_PAD_H

#include "core_types.h"

/* ---- On/Off convenience ---- */

#ifndef ON
#define ON  1
#endif
#ifndef OFF
#define OFF 0
#endif

/* ---- Pull resistor (core_pad_input, core_pad_output_od) ---- */

#define CORE_PULL_NONE  0x0UL   /**< No pull resistor */
#define CORE_PULL_UP    0x1UL   /**< Pull-up */
#define CORE_PULL_DOWN  0x2UL   /**< Pull-down */

/* ---- Output type ---- */

#define CORE_OTYPE_PP   0x0UL   /**< Push-pull */
#define CORE_OTYPE_OD   0x1UL   /**< Open-drain */

/* ---- Output speed (core_pad_speed) ---- */

#define CORE_SPEED_LOW    0x0UL   /**< ~2 MHz: LEDs, relays */
#define CORE_SPEED_MED    0x1UL   /**< ~10 MHz: general GPIO (default) */
#define CORE_SPEED_HIGH   0x2UL   /**< ~50 MHz: SPI, UART */
#define CORE_SPEED_VHIGH  0x3UL   /**< ~100 MHz: SDMMC, high-speed */

/* ---- Edge (core_pad_on_change, core_stop_until_on_change) ---- */

#define CORE_EDGE_RISING   0x01   /**< Rising edge */
#define CORE_EDGE_FALLING  0x02   /**< Falling edge */
#define CORE_EDGE_BOTH     0x03   /**< Both edges */

/* Deprecated short names, kept so existing code builds: use the CORE_ ones. */
#define PULL_NONE     CORE_PULL_NONE
#define PULL_UP       CORE_PULL_UP
#define PULL_DOWN     CORE_PULL_DOWN
#define PUSH_PULL     CORE_OTYPE_PP
#define OPEN_DRAIN    CORE_OTYPE_OD
#define SPEED_LOW     CORE_SPEED_LOW
#define SPEED_MED     CORE_SPEED_MED
#define SPEED_HIGH    CORE_SPEED_HIGH
#define SPEED_VHIGH   CORE_SPEED_VHIGH
#define EDGE_RISING   CORE_EDGE_RISING
#define EDGE_FALLING  CORE_EDGE_FALLING
#define EDGE_BOTH     CORE_EDGE_BOTH

/* The port's pad types (core_pad_port_t). */
#include "port_core_pad_types.h"

/* ---- Pad GPIO API ---- */

/** Configure a pad as push-pull output (default). */
CORE_INLINE void core_pad_output(uint8_t pad);

/** Configure a pad as open-drain output with optional pull resistor.
 *  Use CORE_PULL_UP for a wired-AND / I2C-style bus, CORE_PULL_NONE for external pull. */
CORE_INLINE void core_pad_output_od(uint8_t pad, uint32_t pull);

/** Configure a pad as input with pull resistor. */
CORE_INLINE void core_pad_input(uint8_t pad, uint32_t pull);

/**
 * Set a pad high (ON) or low (OFF).
 *
 * @studio expose category=pad name=write
 * @studio twin full
 * @param pad [1..64] Tile pad number.
 * @param state [0..1] 0 for low, 1 for high.
 */
CORE_INLINE void core_pad_write(uint8_t pad, int state);

/**
 * Read a pad. Returns 0 or 1.
 *
 * @studio expose category=pad name=read returns=bool
 * @studio twin full
 * @param pad [1..64] Tile pad number.
 */
CORE_INLINE int core_pad_read(uint8_t pad);

/**
 * Toggle a pad output.
 *
 * @studio expose category=pad name=toggle
 * @studio twin full
 * @param pad [1..64] Tile pad number.
 */
CORE_INLINE void core_pad_toggle(uint8_t pad);

/* ---- Fast path (resolved handle + bulk BSRR) ----
 *
 * core_pad_write/read do a pad→port/pin table lookup every call. For tight
 * bitbang inner loops (e.g. a software SWD/CMSIS-DAP clock) that's too slow:
 * resolve the pad ONCE to a {port, mask} handle, then hit BSRR/IDR directly —
 * each op compiles to a single store/load. C-only (struct ABI), no DSL surface.
 */

typedef struct {
    core_pad_port_t *port;  /* NULL if the pad has no GPIO (power/reset/etc.) */
    uint32_t         mask;  /* (1u << pin) */
} core_pad_fast_t;

/** Resolve a pad to a fast handle once, outside the hot loop. */
CORE_INLINE core_pad_fast_t core_pad_resolve(uint8_t pad);

/** Drive the resolved pad high — single BSRR write. */
CORE_INLINE void core_pad_fast_set(core_pad_fast_t h);
/** Drive the resolved pad low — single BSRR write. */
CORE_INLINE void core_pad_fast_clear(core_pad_fast_t h);
/** Drive the resolved pad to `state` (0/1) — single BSRR write. */
CORE_INLINE void core_pad_fast_write(core_pad_fast_t h, int state);
/** Read the resolved pad — single IDR load. Returns 0 or 1. */
CORE_INLINE int core_pad_fast_read(core_pad_fast_t h);

/**
 * Bulk update many pins on ONE port in a single atomic BSRR write: pins in
 * `set_mask` go high, pins in `clear_mask` go low (set wins on overlap, per
 * BSRR). Build the masks from the generated PAD_n_MASK defines. Useful for
 * synchronized buses / LED matrices without per-pad jitter.
 */
CORE_INLINE void core_pad_bulk_write(core_pad_port_t *port,
                                     uint32_t set_mask, uint32_t clear_mask);

/** Configure a pad as analog (for ADC, DAC, comparator). */
CORE_INLINE void core_pad_analog(uint8_t pad);

/** Set the output speed (slew rate) of a pad.
 *  Use CORE_SPEED_LOW / _MED / _HIGH / _VHIGH.
 *  Only affects outputs — input pads ignore this setting. */
CORE_INLINE void core_pad_speed(uint8_t pad, uint32_t speed);

/* ---- Edge-triggered interrupts (EXTI) ---- */

/** Pad edge callback, run in interrupt context with the ctx given at registration. */
typedef core_callback_t core_pad_cb_t;

/**
 * Register an edge-triggered callback on a pad.
 *
 * The pad is automatically configured as input with a sensible
 * pull resistor based on edge direction:
 *   rising  → pull-down (idle low)
 *   falling → pull-up   (idle high)
 *   both    → no pull   (external bias expected)
 *
 * @param pad   Tile pad number
 * @param edge  CORE_EDGE_RISING, CORE_EDGE_FALLING, or CORE_EDGE_BOTH
 * @param cb    Callback (called from ISR context)
 * @param ctx   Opaque pointer passed to cb
 */
CORE_INLINE core_status_t core_pad_on_change(uint8_t pad, uint32_t edge,
                                              core_pad_cb_t cb, void *ctx);

/** Stop the interrupt on a pad. */
CORE_INLINE void core_pad_on_change_stop(uint8_t pad);

/* ---- Backward compatibility ---- */
#define core_on_change       core_pad_on_change
#define core_on_change_stop  core_pad_on_change_stop

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=M title="No DSL access to slew-rate / pull / drive-strength"
//   Tier 2 exposes write/read/toggle only. core_pad_speed,
//   core_pad_output_od pull-config, and the analog-mode setter are
//   reachable only by escaping to C. A Tier 2 wrapper (or @studio expose
//   annotation) for slew-rate would let DSL programs tune SPI/UART rise
//   times without leaving the IDE.
//
// @studio unsupported tier=2 value=L title="Bulk / fast pad write is C-only"
//   The single-pad DSL ops do a pad→port/pin lookup each call. A C fast
//   path now exists — core_pad_resolve() + core_pad_fast_* (resolved
//   {port,mask} handle, direct BSRR/IDR) and core_pad_bulk_write() (one
//   atomic BSRR for many pins) — used by bitbang inner loops like the
//   CMSIS-DAP probe. Still no DSL surface (pointer/struct ABI), so
//   synchronized-bus / LED-matrix updates from the IDE remain a gap.

/* The port's value checks and the bodies of the functions above. */
#include "port_core_pad.h"

#endif /* CORE_PAD_H */
