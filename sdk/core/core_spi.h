/**
 * core_spi.h — SPI master bus communication
 *
 * Master-mode SPI, 8-bit frames, software CS on a tile pad: bounded polled
 * transfers (full-duplex, write-only, read-only, CS-held command + data) and
 * DMA transfers (blocking or with a completion callback). Wraps hal_spi.
 *
 * Quick start (polling):
 * @code
 *   hal_spi_t spi;
 *   hal_spi_config_t cfg = { .prescaler = LL_SPI_PRESCALER_8 };  // mode 0, MSB first
 *   core_spi_init(&spi, SPI1, &cfg);
 *   core_spi_set_cs(&spi, 9);          // Pad 9 as CS
 *
 *   uint8_t cmd[4] = { 0x03, 0x00, 0x10, 0x00 };   // SPI-NOR READ @ 0x001000
 *   uint8_t page[256];
 *   if (core_spi_write_read(&spi, cmd, 4, page, sizeof page) != HAL_OK) { ... }
 * @endcode
 *
 * Projects that declare the bus in config.json get a ready handle
 * (core_spi1 / core_spi3) with its CS pad already attached.
 *
 * Conventions for every buffer call: a NULL tx buffer clocks out the handle's
 * fill byte (0xFF, `spi.fill`), a NULL rx buffer discards what comes back,
 * and tx may equal rx. Every call is bounded: if no frame completes for the
 * stall budget (2 ms plus a few frame times at the configured SCK, see
 * ll_spi_stall_cycles) it returns HAL_TIMEOUT and the peripheral is reset, so
 * the next call starts clean.
 *
 * Fastest SCK is the kernel clock / 2 (the kernel clock is SYSCLK at every
 * coregen clock level on the L4 / W5; on the H5 it is the HSI, 64 MHz, or
 * 16 MHz at "low"):
 *   Core.ST.L4: 8 MHz (low/medium, 16 MHz), 24 MHz (high), 40 MHz (max).
 *               Above 16 MHz needs VDD >= 2.7 V (STM32L422 DS Table 80).
 *   Core.ST.W5: 8 MHz (low), 16 MHz (medium), 32 MHz (high), 50 MHz (max).
 *               Above 33 MHz needs VDD >= 2.7 V (DS14127 Table 94): at 1.8 V
 *               use /4 (25 MHz) at max.
 *
 * Available on: Core.ST.L4, Core.ST.W5, Core.ST.H5 (not Core.ST.L0 — no SPI peripheral).
 *
 * @studio category spi label=Core.SPI icon=⇆
 *
 * @studio coverage
 *   id:    spi
 *   name:  SPI — bus communication
 *   page:  /docs/sdk/spi
 *   blurb: Master-mode SPI, 8-bit frames, modes 0-3, MSB or LSB first:
 *          bounded polled transfers (full-duplex, write, read, CS-held
 *          command + data) and DMA transfers, blocking or with a callback
 *          (Core.ST.L4 and Core.ST.W5). Bench-verified on Core.ST.L4 by
 *          tests/hw-spi-loopback. Tier 2 exposes a single-byte full-duplex
 *          transfer against a bus id + CS pad — coregen resolves the handle
 *          via core_spi_handle_for_bus(). Tier 1 keeps the explicit-handle
 *          forms for buffer transfers, DMA, and persistent CS control.
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

/** SPI configuration — prescaler, clock polarity, clock phase, bit order. */
typedef hal_spi_config_t core_spi_config_t;

/* ---- Init ---- */

/**
 * @brief  Initialize SPI in master mode (8-bit frames).
 *
 * Enables and resets the peripheral, then applies `cfg`. SCK/MOSI/MISO pads
 * must already be in their SPI alternate function (coregen does this for the
 * pads config.json assigns).
 *
 * @param  h         Handle (zeroed by this call; attach CS afterwards)
 * @param  instance  SPI1 (Core.ST.L4 / Core.ST.W5) or SPI3 (Core.ST.W5)
 * @param  cfg       Prescaler (LL_SPI_PRESCALER_2..256), cpol, cpha, lsb_first
 * @return HAL_OK, or HAL_ERROR on a NULL argument
 */
static inline hal_status_t core_spi_init(hal_spi_t *h,
                                          SPI_TypeDef *instance,
                                          const hal_spi_config_t *cfg)
{
    return hal_spi_init(h, instance, cfg);
}

/**
 * @brief  Change prescaler, SPI mode or bit order, keeping the CS pad.
 *
 * Use it to talk to devices with different modes on one bus, or to start
 * slow and speed up after identification.
 *
 * @param  h    SPI handle
 * @param  cfg  New configuration
 * @return HAL_OK, HAL_BUSY while a DMA transfer runs, HAL_ERROR on NULL
 */
static inline hal_status_t core_spi_configure(hal_spi_t *h,
                                               const hal_spi_config_t *cfg)
{
    return hal_spi_configure(h, cfg);
}

/**
 * @brief  The SCK frequency the handle is set to, in Hz.
 * @param  h  SPI handle
 * @return SYSCLK / (2..256); 0 for an uninitialized handle
 */
static inline uint32_t core_spi_sck_hz(const hal_spi_t *h)
{
    return hal_spi_sck_hz(h);
}

/* ---- CS management ---- */

/**
 * @brief  Assign a CS pin by tile pad number.
 *
 * Configures the pad as a push-pull output at its inactive level (high, or
 * low when `h->cs_active_low` is 0).
 *
 * @param  h    SPI handle
 * @param  pad  Tile pad driving chip select
 */
static inline void core_spi_set_cs(hal_spi_t *h, uint8_t pad)
{
    hal_pad_gpio_t g = hal_pad_lookup(pad);
    if (g.port)
        hal_spi_set_cs(h, g.port, g.pin);
}

/**
 * @brief  Assert CS. Everything until core_spi_deselect() is one transaction.
 * @param  h  SPI handle
 */
static inline void core_spi_select(hal_spi_t *h)
{
    hal_spi_select(h);
}

/**
 * @brief  Deassert CS, ending the transaction.
 * @param  h  SPI handle
 */
static inline void core_spi_deselect(hal_spi_t *h)
{
    hal_spi_deselect(h);
}

/* ---- Polling transfer ---- */

/**
 * @brief  Full-duplex single byte. CS untouched.
 * @param  h   SPI handle
 * @param  tx  Byte to send
 * @return The byte received (0xFF if the transfer failed)
 */
static inline uint8_t core_spi_transfer(hal_spi_t *h, uint8_t tx)
{
    return hal_spi_transfer(h, tx);
}

/**
 * @brief  Full-duplex transfer of `len` bytes. CS untouched.
 *
 * Polled and FIFO-pipelined, so SCK runs back to back. Build a CS-held
 * transaction from several calls between core_spi_select() and
 * core_spi_deselect().
 *
 * @param  h    SPI handle
 * @param  tx   Bytes to send, or NULL to send the fill byte (0xFF)
 * @param  rx   Receive buffer, or NULL to discard (may equal tx)
 * @param  len  Number of bytes
 * @return HAL_OK, HAL_TIMEOUT (stalled), HAL_ERROR (overrun / mode fault),
 *         HAL_BUSY (a DMA transfer is running)
 */
static inline hal_status_t core_spi_exchange(hal_spi_t *h, const uint8_t *tx,
                                              uint8_t *rx, uint32_t len)
{
    return hal_spi_exchange(h, tx, rx, len);
}

/**
 * @brief  Write-only: send `len` bytes, discard what comes back. CS untouched.
 * @param  h     SPI handle
 * @param  data  Bytes to send
 * @param  len   Number of bytes
 * @return HAL_OK, HAL_TIMEOUT, HAL_ERROR or HAL_BUSY (see core_spi_exchange)
 */
static inline hal_status_t core_spi_write(hal_spi_t *h, const uint8_t *data,
                                           uint32_t len)
{
    return hal_spi_write(h, data, len);
}

/**
 * @brief  Read-only: clock out the fill byte (0xFF) and capture `len` bytes.
 *         CS untouched.
 * @param  h    SPI handle
 * @param  buf  Receive buffer
 * @param  len  Number of bytes
 * @return HAL_OK, HAL_TIMEOUT, HAL_ERROR or HAL_BUSY (see core_spi_exchange)
 */
static inline hal_status_t core_spi_read(hal_spi_t *h, uint8_t *buf, uint32_t len)
{
    return hal_spi_read(h, buf, len);
}

/**
 * @brief  One CS-framed full-duplex transaction: select, exchange, deselect.
 * @param  h    SPI handle
 * @param  tx   Bytes to send, or NULL to send the fill byte
 * @param  rx   Receive buffer, or NULL to discard (may equal tx)
 * @param  len  Number of bytes
 * @return HAL_OK, HAL_TIMEOUT, HAL_ERROR or HAL_BUSY (see core_spi_exchange)
 */
static inline hal_status_t core_spi_xfer(hal_spi_t *h, const uint8_t *tx,
                                          uint8_t *rx, uint32_t len)
{
    return hal_spi_xfer(h, tx, rx, len);
}

/**
 * @brief  One CS-framed command + data transaction.
 *
 * Select, send `tx_len` bytes (command, address, dummy bytes, or write data;
 * what comes back is discarded), clock in `rx_len` bytes, deselect. CS stays
 * asserted across both phases, as SPI-NOR flash (Store.O.128) and register
 * reads need.
 *
 * @param  h       SPI handle
 * @param  tx      Command bytes
 * @param  tx_len  Number of command bytes
 * @param  rx      Receive buffer (may be NULL when rx_len is 0)
 * @param  rx_len  Number of bytes to read after the command
 * @return HAL_OK, HAL_TIMEOUT, HAL_ERROR or HAL_BUSY (see core_spi_exchange)
 */
static inline hal_status_t core_spi_write_read(hal_spi_t *h, const uint8_t *tx,
                                                uint32_t tx_len, uint8_t *rx,
                                                uint32_t rx_len)
{
    return hal_spi_write_read(h, tx, tx_len, rx, rx_len);
}

/* ---- DMA transfer ---- */

/**
 * @brief  Full-duplex DMA transfer, blocking until the last frame is off the
 *         bus. CS untouched.
 *
 * Same buffers and NULL conventions as core_spi_exchange(); worth it from a
 * few dozen bytes up, where it keeps SCK saturated and the CPU free of
 * per-byte work. Core.ST.L4: DMA1 CH2/CH3. Core.ST.W5: GPDMA1 CH6/CH7.
 *
 * @param  h    SPI handle
 * @param  tx   Bytes to send, or NULL to send the fill byte
 * @param  rx   Receive buffer, or NULL to discard (may equal tx)
 * @param  len  Number of bytes (any length; long transfers are chunked)
 * @return HAL_OK, HAL_TIMEOUT (stalled; channels aborted, SPI reset),
 *         HAL_BUSY (another DMA transfer is running), HAL_ERROR (DMA error,
 *         or Core.ST.H5, which has no SPI DMA yet)
 */
static inline hal_status_t core_spi_exchange_dma(hal_spi_t *h, const uint8_t *tx,
                                                  uint8_t *rx, uint32_t len)
{
    return hal_spi_exchange_dma(h, tx, rx, len);
}

/**
 * @brief  Start a full-duplex DMA transfer and return at once.
 *
 * The callback runs from interrupt context when the last frame is received
 * (Core.ST.L4: DMA RX complete; Core.ST.W5: the SPI's end-of-transfer).
 * Manage CS yourself: select before, deselect in the callback or after
 * core_spi_dma_wait(). One DMA transfer at a time across all SPI handles.
 *
 * @param  h    SPI handle
 * @param  tx   TX buffer (NULL = send the fill byte, 0xFF)
 * @param  rx   RX buffer (NULL = discard received data)
 * @param  len  Number of bytes: up to 65535 (Core.ST.L4, Core.ST.W5 SPI1),
 *              1023 (Core.ST.W5 SPI3)
 * @param  cb   Completion callback (interrupt context), or NULL
 * @param  ctx  User context for the callback
 * @return HAL_OK, HAL_BUSY if a DMA transfer is running, HAL_ERROR for a bad
 *         length or on Core.ST.H5
 */
static inline hal_status_t core_spi_xfer_dma(hal_spi_t *h,
                                              const uint8_t *tx,
                                              uint8_t *rx,
                                              uint32_t len,
                                              hal_callback_t cb, void *ctx)
{
    return hal_spi_xfer_dma(h, tx, rx, len, cb, ctx);
}

/**
 * @brief  Wait for a core_spi_xfer_dma() transfer to finish (bounded).
 * @param  h  SPI handle
 * @return The transfer's result (HAL_OK if none was running), or HAL_TIMEOUT
 *         after aborting a stalled transfer
 */
static inline hal_status_t core_spi_dma_wait(hal_spi_t *h)
{
    return hal_spi_dma_wait(h);
}

/**
 * @brief  Whether a DMA transfer is in progress on this handle.
 * @param  h  SPI handle
 * @return 1 while core_spi_xfer_dma() is running, else 0
 */
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
 * cs_pad undefined, transfer timed out). The signed return lets DSL
 * programs branch on `< 0` without an out-pointer.
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
    if (h->cs_port != cs.port || h->cs_pin != cs.pin)
        hal_spi_set_cs(h, cs.port, cs.pin);
    uint8_t rx = 0xFF;
    if (hal_spi_xfer(h, &tx, &rx, 1) != HAL_OK) return -1;
    return (int)rx;
}

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=M title="Tier 2 is single-byte xfer only — no bulk / persistent CS"
//   The Tier 2 surface is core_spi_xfer_byte_bus: one byte, CS auto-
//   managed around the call. DSL programs that need to push a
//   multi-byte payload with CS held (display init streams, SD-card
//   sectors, audio frame transfers) drop back to Tier 1 (core_spi_xfer,
//   core_spi_write_read, core_spi_exchange_dma). Bulk variants need
//   the array-IN / array-OUT host-call ABI prototyped on the tile-
//   driver side — track with the DSL Capability Coverage close.
//
// @studio unsupported tier=1 value=M title="Core.ST.W5 SPI not yet bench-verified; Core.ST.H5 SPI still stalls"
//   Core.ST.L4 passes tests/hw-spi-loopback (polled + DMA, modes 0-3,
//   1-4096 bytes, timeouts). Core.ST.W5 builds the same code (TSIZE
//   sessions, GPDMA) but has not run it on hardware yet; the camera tile's
//   LL half-duplex path is the only W5 SPI use proven on silicon.
//   Core.ST.H5: the kernel clock is now per_ck (the HSI), but on the bench
//   SPI1 still never clocks a frame and a transfer returns HAL_TIMEOUT
//   (2026-09-26, only tried on a board started by the ROM bootloader).
//
// @studio unsupported tier=1 value=L title="SPI DMA not on Core.ST.H5"
//   core_spi_exchange_dma / core_spi_xfer_dma run on Core.ST.L4 (DMA1)
//   and Core.ST.W5 (GPDMA1 CH6/CH7) and return HAL_ERROR on Core.ST.H5,
//   where polled transfers are the only path.
//
// @studio unsupported tier=1 value=L title="8-bit Motorola frames only"
//   Frames are 8 bits. No 16-bit frames, hardware CRC, TI frame format or
//   hardware NSS at the core level; the LL layer has the Core.ST.W5
//   half-duplex (1-line) mode.
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
