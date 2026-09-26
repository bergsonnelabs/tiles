/**
 * hal_spi.h — SPI HAL driver
 *
 * Master-mode SPI with software CS pin management, polled and DMA
 * transfers. Supports the old SPI IP (L0/L4) and SPI v2 (WBA/H5).
 *
 * CS is managed via GPIO (not hardware NSS), so a transaction can span
 * several calls: select, any number of transfers, deselect. Assign a CS pin
 * with hal_spi_set_cs() or manage CS externally via the tile driver CS array.
 *
 * Every call is bounded. A transfer that stops making progress (no frame
 * completes for the stall budget, see ll_spi_stall_cycles: about 2 ms plus a
 * few frame times) returns HAL_TIMEOUT, and the peripheral is reset and
 * reconfigured so the next call starts clean.
 *
 * DMA: hal_spi_exchange_dma() (blocking) and hal_spi_xfer_dma() (completion
 * callback) on the L4 (DMA1 CH2/CH3) and the WBA (GPDMA1 CH6/CH7). The H5
 * returns HAL_ERROR for DMA.
 */

#ifndef HAL_SPI_H
#define HAL_SPI_H

#include "hal_common.h"
#include "ll_spi.h"
#include "ll_gpio.h"

/* ============================================================
 * Types
 * ============================================================ */

typedef struct {
    uint32_t prescaler;     /* LL_SPI_PRESCALER_* (SCK = kernel clock / 2..256) */
    uint8_t  cpol;          /* LL_SPI_CPOL_LOW or _HIGH */
    uint8_t  cpha;          /* LL_SPI_CPHA_1EDGE or _2EDGE */
    uint8_t  lsb_first;     /* 0 = MSB first (default), 1 = LSB first */
} hal_spi_config_t;

typedef struct hal_spi {
    SPI_TypeDef  *instance;

    /* CS pin (optional — NULL means no CS management) */
    GPIO_TypeDef *cs_port;
    uint32_t      cs_pin;
    uint8_t       cs_active_low;  /* 1 = CS active low (default) */

    uint8_t       fill;           /* byte clocked out when tx is NULL (0xFF) */
    hal_spi_config_t cfg;         /* kept for hal_spi_configure() and recovery */

    /* DMA state */
    volatile uint8_t busy;
    volatile int8_t  dma_status;  /* hal_status_t of the last async DMA transfer */
    hal_callback_t   complete_cb;
    void            *complete_ctx;
    uint8_t         *dma_rx_buf;   /* DMA destination buffer */
    uint32_t         dma_len;      /* DMA transfer length */
} hal_spi_t;

/* ============================================================
 * API declarations (implemented in hal_spi.c)
 * ============================================================ */

/**
 * Initialize SPI in master mode, 8-bit frames.
 * The peripheral clock is auto-enabled and the peripheral reset first.
 * SCK/MOSI/MISO pins must be configured for AF (via coregen or manually).
 *
 * @param h         Handle (zero-initialized by this function)
 * @param instance  SPI peripheral (SPI1, SPI2, SPI3)
 * @param cfg       Prescaler, polarity, phase and bit order
 * @return HAL_OK on success, HAL_ERROR on a NULL argument
 */
hal_status_t hal_spi_init(hal_spi_t *h, SPI_TypeDef *instance,
                          const hal_spi_config_t *cfg);

/**
 * Change prescaler, mode or bit order on an initialized handle. Keeps the CS
 * pin. Waits (bounded) for the bus to go idle first.
 *
 * @param h    SPI handle
 * @param cfg  New configuration
 * @return HAL_OK, HAL_BUSY if a DMA transfer is running, HAL_ERROR on NULL
 */
hal_status_t hal_spi_configure(hal_spi_t *h, const hal_spi_config_t *cfg);

/** Disable the SPI peripheral and release the handle. */
void hal_spi_deinit(hal_spi_t *h);

/**
 * Assign a CS GPIO pin for automatic chip-select management.
 * The pin is configured as push-pull output and deasserted (high).
 *
 * @param h     SPI handle
 * @param port  GPIO port (e.g. GPIOA)
 * @param pin   Pin number (0–15)
 */
void hal_spi_set_cs(hal_spi_t *h, GPIO_TypeDef *port, uint32_t pin);

/* ---- CS control ---- */

/** Assert CS (drive active — low by default). */
void hal_spi_select(hal_spi_t *h);

/** Deassert CS (drive inactive — high by default). */
void hal_spi_deselect(hal_spi_t *h);

/**
 * The SCK frequency the handle's prescaler gives, in Hz: the SPI kernel
 * clock (SYSCLK at every coregen clock level, read from the SysTick reload)
 * divided by 2..256.
 */
uint32_t hal_spi_sck_hz(const hal_spi_t *h);

/* ---- Polled transfers (blocking, bounded) ---- */

/**
 * Full-duplex transfer of `len` bytes without touching CS.
 * NULL tx clocks out the handle's fill byte (0xFF); NULL rx discards what
 * comes back. tx and rx may be the same buffer.
 *
 * @return HAL_OK, HAL_TIMEOUT (stalled; the peripheral is reset),
 *         HAL_ERROR (overrun / mode fault / bad handle), HAL_BUSY (DMA running)
 */
hal_status_t hal_spi_exchange(hal_spi_t *h, const uint8_t *tx, uint8_t *rx,
                              uint32_t len);

/** Full-duplex single byte: send tx, return received byte (0xFF on error). */
uint8_t hal_spi_transfer(hal_spi_t *h, uint8_t tx);

/** Full-duplex buffer transfer (in-place: rx overwrites buf). */
void hal_spi_transfer_buf(hal_spi_t *h, uint8_t *buf, uint32_t len);

/** Write-only: send data, discard received bytes. CS untouched. */
hal_status_t hal_spi_write(hal_spi_t *h, const uint8_t *data, uint32_t len);

/** Read-only: clock out the fill byte (0xFF), capture received bytes. CS untouched. */
hal_status_t hal_spi_read(hal_spi_t *h, uint8_t *buf, uint32_t len);

/**
 * One CS-framed full-duplex transaction: select → exchange → deselect.
 * tx and rx can be the same buffer. Either can be NULL.
 */
hal_status_t hal_spi_xfer(hal_spi_t *h, const uint8_t *tx, uint8_t *rx, uint32_t len);

/**
 * One CS-framed command + data transaction: select, send tx_len bytes
 * (received bytes discarded), then clock in rx_len bytes (sending the fill
 * byte), deselect. CS stays asserted across both phases (SPI-NOR reads:
 * command + 3-byte address + data).
 */
hal_status_t hal_spi_write_read(hal_spi_t *h, const uint8_t *tx, uint32_t tx_len,
                                uint8_t *rx, uint32_t rx_len);

/* ---- DMA transfers ---- */

/**
 * Full-duplex DMA transfer, blocking until the last frame is off the bus.
 * CS untouched. Same NULL conventions as hal_spi_exchange(). Bounded: a
 * stall (no DMA progress for the stall budget) aborts both channels, resets
 * the peripheral and returns HAL_TIMEOUT.
 *
 * @return HAL_OK, HAL_TIMEOUT, HAL_BUSY (another DMA transfer is running),
 *         HAL_ERROR (DMA error, or no DMA on this platform)
 */
hal_status_t hal_spi_exchange_dma(hal_spi_t *h, const uint8_t *tx, uint8_t *rx,
                                  uint32_t len);

/**
 * Start a full-duplex DMA transfer (non-blocking).
 *
 * @param h    SPI handle
 * @param tx   TX buffer (NULL = send the fill byte, 0xFF)
 * @param rx   RX buffer (NULL = discard received data)
 * @param len  Number of bytes (L4: up to 65535; WBA: up to 65535 on SPI1,
 *             1023 on SPI3)
 * @param cb   Completion callback (from interrupt context)
 * @param ctx  User context for callback
 * @return HAL_OK, HAL_BUSY, or HAL_ERROR (bad length, or no DMA on this platform)
 */
hal_status_t hal_spi_xfer_dma(hal_spi_t *h, const uint8_t *tx, uint8_t *rx,
                               uint32_t len, hal_callback_t cb, void *ctx);

/**
 * Wait (bounded) for an async DMA transfer started with hal_spi_xfer_dma().
 * On a stall it aborts the transfer and resets the peripheral.
 *
 * @return The transfer's status (HAL_OK if none was running), or HAL_TIMEOUT
 */
hal_status_t hal_spi_dma_wait(hal_spi_t *h);

/** Returns 1 if a DMA transfer is in progress. */
int hal_spi_busy(hal_spi_t *h);

#endif /* HAL_SPI_H */
