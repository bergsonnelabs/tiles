/**
 * core_spi.h — SPI master bus communication
 *
 * Master-mode SPI with software CS management, polling and DMA
 * transfers. Wraps hal_spi with pad-based convenience.
 *
 * Quick start (polling):
 * @code
 *   hal_spi_t spi;
 *   hal_spi_config_t cfg = { .prescaler = LL_SPI_PRESCALER_8 };
 *   core_spi_init(&spi, SPI1, &cfg);
 *   core_spi_set_cs(&spi, 11);         // Pad 11 as CS
 *
 *   core_spi_select(&spi);
 *   uint8_t who = core_spi_transfer(&spi, 0xF5);  // Read reg 0x75
 *   core_spi_deselect(&spi);
 * @endcode
 *
 * Available on: Core.ST.L4, Core.ST.W5, Core.ST.H5 (not Core.ST.L0 — no SPI peripheral).
 *
 * @studio category spi label=Core.SPI icon=⇆
 *
 * @studio coverage
 *   id:    spi
 *   name:  SPI — bus communication
 *   page:  /docs/sdk/spi
 *   blurb: Master-mode SPI: polled byte / buffer transfer, software CS
 *          via tile pads, and DMA non-blocking transfers (Core.ST.L4
 *          only, not yet run on hardware; Core.ST.W5/H5 DMA is WIP). Tier 2 exposes a
 *          single-byte full-duplex transfer against a bus id + CS pad —
 *          coregen resolves the handle via core_spi_handle_for_bus().
 *          Tier 1 keeps the explicit-handle forms for buffer transfers,
 *          DMA, and persistent CS control.
 */

#ifndef CORE_SPI_H
#define CORE_SPI_H

/* SPI is not available on Core.ST.L0 (STM32L011) */
#if defined(STM32L011xx)
#error "core_spi.h: SPI is not available on Core.ST.L0. Only Core.ST.L4, Core.ST.W5, and Core.ST.H5 have SPI."
#endif

#include "hal_spi.h"
#include "hal_gpio.h"

/** SPI handle type — same as hal_spi_t under the hood. */
typedef hal_spi_t core_spi_t;

/** SPI configuration — prescaler, clock polarity, clock phase. */
typedef hal_spi_config_t core_spi_config_t;

/* ---- Init ---- */

/** Initialize SPI in master mode. Same signature as hal_spi_init. */
static inline hal_status_t core_spi_init(hal_spi_t *h,
                                          SPI_TypeDef *instance,
                                          const hal_spi_config_t *cfg)
{
    return hal_spi_init(h, instance, cfg);
}

/* ---- CS management ---- */

/**
 * Assign a CS pin using a tile pad number.
 * Resolves pad to port/pin via hal_pad_lookup, then calls
 * hal_spi_set_cs to configure and deassert the pin.
 */
static inline void core_spi_set_cs(hal_spi_t *h, uint8_t pad)
{
    hal_pad_gpio_t g = hal_pad_lookup(pad);
    if (g.port)
        hal_spi_set_cs(h, g.port, g.pin);
}

/** Assert CS (drive low). */
static inline void core_spi_select(hal_spi_t *h)
{
    hal_spi_select(h);
}

/** Deassert CS (drive high). */
static inline void core_spi_deselect(hal_spi_t *h)
{
    hal_spi_deselect(h);
}

/* ---- Polling transfer ---- */

/** Full-duplex single byte transfer. Returns received byte. */
static inline uint8_t core_spi_transfer(hal_spi_t *h, uint8_t tx)
{
    return hal_spi_transfer(h, tx);
}

/** Write-only (discard received data). */
static inline void core_spi_write(hal_spi_t *h, const uint8_t *data,
                                   uint32_t len)
{
    hal_spi_write(h, data, len);
}

/** Read-only (send zeros). */
static inline void core_spi_read(hal_spi_t *h, uint8_t *buf, uint32_t len)
{
    hal_spi_read(h, buf, len);
}

/**
 * Convenience: select + full-duplex transfer + deselect.
 * tx and rx can be the same buffer. Either can be NULL.
 */
static inline void core_spi_xfer(hal_spi_t *h, const uint8_t *tx,
                                  uint8_t *rx, uint32_t len)
{
    hal_spi_xfer(h, tx, rx, len);
}

/* ---- DMA transfer (non-blocking) ---- */

/**
 * Start a DMA-based SPI transfer (non-blocking).
 *
 * Full-duplex: tx bytes are sent while rx bytes are received
 * simultaneously. Either tx or rx can be NULL for write-only or
 * read-only transfers. The callback fires from DMA ISR context
 * when the transfer completes.
 *
 * Caller must manage CS: assert before calling, deassert in
 * the callback. The command/address byte should be sent via
 * polling (core_spi_transfer) before starting DMA.
 *
 * @param h    SPI handle
 * @param tx   TX buffer (NULL = send zeros)
 * @param rx   RX buffer (NULL = discard received data)
 * @param len  Number of bytes to transfer
 * @param cb   Completion callback (called from DMA ISR)
 * @param ctx  User context for callback
 * @return HAL_OK on success, HAL_BUSY if a DMA transfer is active,
 *         HAL_ERROR if DMA is not available on this platform
 */
static inline hal_status_t core_spi_xfer_dma(hal_spi_t *h,
                                              const uint8_t *tx,
                                              uint8_t *rx,
                                              uint32_t len,
                                              hal_callback_t cb, void *ctx)
{
    return hal_spi_xfer_dma(h, tx, rx, len, cb, ctx);
}

/** Returns 1 if a DMA transfer is in progress. */
static inline int core_spi_busy(hal_spi_t *h)
{
    return hal_spi_busy(h);
}

/* ---- Tier 2 — default-instance bus helpers ---------------------------- */

/* These wrappers take a bus id (the SPI peripheral number — 1, 2, 3 …
 * matching how config.json declares it) plus the CS pad, instead of
 * a hal_spi_t handle + persistent CS state. The dispatcher
 * `core_spi_handle_for_bus` is emitted by coregen alongside the per-
 * bus extern handles (core_spi1, core_spi2 …). DSL programs reach
 * for these; escape-to-C drops back to the Tier 1 handle-based forms
 * when buffer transfers, DMA, or persistent CS control are needed.
 */

/**
 * The SPI handle for a bus id declared in config.json, or NULL if the
 * project doesn't declare that bus. Emitted per project by coregen into
 * core_init.c (when the project declares any SPI bus); the *_bus helpers below
 * resolve their handle through it.
 *
 * Forward-declared here rather than `#include "core_init.h"` so this header
 * compiles without a project (val tests, examples without config.json). The
 * natives-side caller is gated on CORE_HAS_SPI_BUSES, so the linker never asks for
 * the symbol unless the dispatcher exists.
 *
 * @param bus Bus id (1 = the first instance, ...).
 * @return The coregen-initialized handle, or NULL.
 */
hal_spi_t *core_spi_handle_for_bus(uint8_t bus);

/**
 * Single-byte full-duplex transfer over `bus`, with CS auto-managed
 * around the call (asserted before, deasserted after). Returns the
 * received byte (0..255) on success or -1 on any error (bus undeclared,
 * cs_pad undefined). The signed return lets DSL programs branch on
 * `< 0` without an out-pointer.
 *
 * Most chip protocols pair two of these (write a register address,
 * then read or write the value). Multi-byte sequences that need CS
 * held across them — display init streams, audio frame transfers —
 * still need Tier 1 with manual select/deselect.
 *
 * @studio expose category=spi name=xfer_byte returns=int
 * @studio twin full
 * @param bus SPI bus id as declared in config.json (1 = SPI1, ...).
 * @param cs_pad Tile pad driving chip select (asserted around the transfer).
 * @param tx Byte to send.
 */
static inline int core_spi_xfer_byte_bus(uint8_t bus, uint8_t cs_pad, uint8_t tx)
{
    hal_spi_t *h = core_spi_handle_for_bus(bus);
    if (!h) return -1;
    hal_pad_gpio_t cs = hal_pad_lookup(cs_pad);
    if (!cs.port) return -1;
    hal_spi_set_cs(h, cs.port, cs.pin);
    hal_spi_select(h);
    uint8_t rx = hal_spi_transfer(h, tx);
    hal_spi_deselect(h);
    return (int)rx;
}

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=M title="Tier 2 is single-byte xfer only — no bulk / persistent CS"
//   The Tier 2 surface is core_spi_xfer_byte_bus: one byte, CS auto-
//   managed around the call. DSL programs that need to push a
//   multi-byte payload with CS held (display init streams, SD-card
//   sectors, audio frame transfers) drop back to Tier 1 with a
//   hal_spi_t handle and manual select/deselect. Bulk variants need
//   the array-IN / array-OUT host-call ABI prototyped on the tile-
//   driver side — track with the DSL Capability Coverage close.
//
// @studio unsupported tier=1 value=H title="core_spi is not bench-verified on any Core; broken on Core.ST.H5"
//   Core.ST.W5: the SPI v2 bring-up is done and the LL half-duplex + GPDMA
//   path ran against a camera, but the core_spi full-duplex path hasn't been
//   bench-verified. Core.ST.H5: SPI has no kernel clock (PLL1Q is never
//   enabled) and the transfer loop has no timeout, so it hangs until the H5
//   fix session. Core.ST.L4: builds, has not run on hardware yet. Tile
//   drivers that need SPI should expect rough edges.
//
// @studio unsupported tier=2 value=M title="DMA implemented only on Core.ST.L4"
//   core_spi_xfer_dma is implemented for the L4 (classic DMA) but not yet
//   run on hardware. On Core.ST.W5 / Core.ST.H5 it returns HAL_ERROR (SPI v2
//   needs TSIZE set while SPE=0); the W5's LL-level ll_spi_dma_* path works.
//   Long buffer transfers (display refresh, audio) are slower on W5 / H5.
//
// @studio unsupported tier=1 value=M title="Slave mode missing"
//   Master-only. No path for a Core to act as a SPI peripheral on
//   another host's bus.
//
// @studio unsupported tier=1 value=L title="Quad / Octo SPI / OctoSPI"
//   SDK roadmap Tier 2: QUADSPI / OctoSPI (memory-mapped external
//   flash for NOR / PSRAM tiles) is wired only on Core.ST.L4 + Core.ST.H5
//   silicon and isn't wrapped here.

#endif /* CORE_SPI_H */
