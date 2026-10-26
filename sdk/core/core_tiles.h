/**
 * core_tiles.h — Bridge between Cores SDK and tile drivers.
 *
 * Provides core_tiles_pal() which accepts either an I2C or SPI bus
 * handle and returns a tiles_pal_t* wired to the Cores SDK:
 *
 *   tiles_pal_t *hal = core_tiles_pal(&core_i2c1);   // I2C bus
 *   tiles_pal_t *hal = core_tiles_pal(&core_spi1);   // SPI bus
 *
 * and core_tiles_pal2() for a tile that uses an I2C and an SPI bus together:
 *
 *   tiles_pal_t *hal = core_tiles_pal2(&core_i2c1, &core_spi1);
 *
 * On Core.ST.H5 it also takes an I3C bus, and I2C tile drivers run over it
 * unchanged (see "I3C adapters" below):
 *
 *   tiles_pal_t *hal = core_tiles_pal(&core_i3c1);   // I3C bus (H5)
 *
 * The correct bus type is resolved at compile time via C11 _Generic.
 * Passing the wrong type is a compile error.
 *
 * @studio coverage
 *   id:    tiles
 *   name:  Tiles — SDK ↔ tile-driver bridge
 *   blurb: Tier 1 only. Header-only adapter that wraps a core_i2c_t* /
 *          core_spi_t* into the tiles_pal_t* shape tile drivers
 *          consume. C11 _Generic resolves the bus type at compile
 *          time; the resulting hal struct is cached per bus pointer
 *          (max 4 per bus type). Used by every tile val / hardware
 *          test in the SDK; not user-facing in the DSL.
 */

#ifndef CORE_TILES_H
#define CORE_TILES_H

#include "core_i2c.h"
#include "core_caps.h"
/* SPI is unavailable on Core.ST.L0 (STM32L011) — core_spi.h #errors there.
 * Guard all SPI bridging below so I2C-only tile projects still build on L0. */
#if CORE_HAS_SPI
#define _CORE_TILES_HAS_SPI 1
#include "core_spi.h"
#endif
/* I3C exists only on the H523 (Core.ST.H5). */
#if CORE_HAS_I3C
#define _CORE_TILES_HAS_I3C 1
#include "core_i3c.h"
#endif
#include "core_pad.h"
#include "tiles_pal.h"
#include "core_types.h"   /* CORE_INLINE */

/* ---- Bus adapters ----
 * core_tiles_pal() / core_tiles_pal2() fill a tiles_pal_t with the port's
 * adapters (port_core_tiles.h), which forward to core_i2c / core_spi /
 * core_i3c and core_pad. */

/* ---- SPI adapters ----
 * Every SPI call is one transaction under one chip-select assertion, chosen by
 * the driver's `cs` argument: tile->id, which for an SPI tile is its instance,
 * or for a dual-bus tile (core_tiles_pal2; Sense.CAM.P) its cfg.spi_cs, which
 * is the tile's tiles[].cs_id in config.json:
 *   - a bus with a chip-select map (coregen, several tiles with their own
 *     tiles[].cs_pad) asserts the map entry whose id is `cs`. A `cs` that is
 *     not in the map returns -1 and drives nothing: it never selects some
 *     other device.
 *   - a bus without one (one tile) asserts the bus's own CS pad, whatever
 *     `cs` is.
 * Bounded: each phase is a bounded core_spi_exchange(), so a stalled bus
 * returns -1, and the chip select is released on every path. */

/* ---- I3C adapters (Core.ST.H5) ----
 * Tile drivers speak I2C: every call carries the static 7-bit address from
 * the driver's instance table. On an I3C bus that address is only used once,
 * to hand the target its dynamic address (SETDASA, from 0x09 up; never the
 * static address itself). Every transfer then goes to the dynamic address as
 * an I3C SDR private message:
 *   i2c_read / i2c_write        -> register read / write (16-bit reg MSB first)
 *   i2c_read_raw / i2c_write_raw -> private read / write
 *   i2c_is_ready                -> GETSTATUS, a CCC every I3C target answers
 * Targets not assigned yet (coregen assigns the tiles it knows at init) are
 * assigned on first use. A target that NACKs its dynamic address has usually
 * lost it: a device reset clears it (the ICM-42686-P does, on its soft reset;
 * benched 2026-09-27), and it then answers only its static address, as an
 * I2C device. The adapter repeats SETDASA with the same dynamic address and
 * retries the transfer once. */

/* ---- Internal: typed constructors ---- */

CORE_INLINE tiles_pal_t *_core_tiles_pal_i2c(core_i2c_t *bus);

#ifdef _CORE_TILES_HAS_I3C
/* An I3C bus looks like an I2C bus to the driver (TILES_BUS_I2C, so the
 * I2C code paths run), flagged TILES_BUS_I3C for the drivers that need an
 * I3C-specific step (Sense.I.6P6 slows its SDA output slew). */
CORE_INLINE tiles_pal_t *_core_tiles_pal_i3c(core_i3c_t *bus);
#endif /* _CORE_TILES_HAS_I3C */

#ifdef _CORE_TILES_HAS_SPI
CORE_INLINE tiles_pal_t *_core_tiles_pal_spi(core_spi_t *bus);

/* ---- Internal: dual-bus (I2C + SPI) PAL ----
 * tiles_pal_t has ONE handle that both bus families receive, so a tile that
 * configures over I2C and streams over SPI (Sense.CAM.P) needs a handle that
 * holds both buses. The port's dual-bus adapters unpack it and forward to
 * the single-bus adapters. The SPI chip select follows the single-bus rules above: the
 * SPI bus's own CS pad, or the map entry named by `cs`. */
typedef struct {
    core_i2c_t *i2c;
    core_spi_t *spi;
} core_tiles_dual_t;

/**
 * Get a tiles_pal_t* carrying BOTH an I2C and an SPI bus, for tiles that use
 * the two together (Sense.CAM.P: I2C configuration, SPI image readout).
 * Cached per (i2c, spi) pair, up to 4 pairs.
 *
 * @code
 *   tiles_pal_t *hal = core_tiles_pal2(&core_i2c1, &core_spi1);
 * @endcode
 */
CORE_INLINE tiles_pal_t *core_tiles_pal2(core_i2c_t *i2c, core_spi_t *spi);
#endif /* _CORE_TILES_HAS_SPI */

/* ---- Public API ---- */

/**
 * Get a tiles_pal_t* for any Cores SDK bus handle.
 *
 * Accepts a core_i2c_t*, a core_spi_t* or (Core.ST.H5) a core_i3c_t* — the
 * correct bus type is resolved at compile time. Each unique bus pointer gets
 * its own cached slot (up to 4 per bus type).
 *
 * @code
 *   tiles_pal_t *hal = core_tiles_pal(&core_i2c1);   // I2C
 *   tiles_pal_t *hal = core_tiles_pal(&core_spi1);   // SPI
 *   tiles_pal_t *hal = core_tiles_pal(&core_i3c1);   // I3C (H5)
 * @endcode
 */
#if defined(_CORE_TILES_HAS_I3C)
#define core_tiles_pal(bus) _Generic((bus), \
    core_i2c_t*: _core_tiles_pal_i2c,      \
    core_spi_t*: _core_tiles_pal_spi,      \
    core_i3c_t*: _core_tiles_pal_i3c       \
)(bus)
#elif defined(_CORE_TILES_HAS_SPI)
#define core_tiles_pal(bus) _Generic((bus), \
    core_i2c_t*: _core_tiles_pal_i2c,      \
    core_spi_t*: _core_tiles_pal_spi       \
)(bus)
#else
/* L0 (no SPI): only the I2C bridge exists. */
#define core_tiles_pal(bus) _Generic((bus), \
    core_i2c_t*: _core_tiles_pal_i2c       \
)(bus)
#endif

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=1 value=M title="Hard cap of 4 cached PALs per bus type"
//   Each bus type keeps its own table of 4 PALs. A fifth distinct bus of
//   the same type gets the first bus's PAL back, so its tile driver
//   silently talks to the first bus. No current Core has more than 4
//   buses of one type.
//
// @studio unsupported tier=1 value=L title="No bus / address discovery"
//   Tile drivers receive a bare tiles_pal_t* and a tile JSON; there's
//   no wrapper here that walks coregen's tile_handles.h to enumerate
//   "what tiles are on what bus." Each test builds the wiring by hand.

/* The port's bodies of the functions above. */
#include "port_core_tiles.h"

#endif /* CORE_TILES_H */
