/**
 * hal_uart.h — UART HAL driver
 *
 * Provides polling TX, interrupt-driven RX with ring buffer,
 * DMA TX, and printf support over USART/LPUART.
 */

#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdarg.h>
#include "hal_common.h"
#include "ll_uart.h"

/* ============================================================
 * Configuration
 * ============================================================ */

#ifndef HAL_UART_RX_BUF_SIZE
  #define HAL_UART_RX_BUF_SIZE   128   /* Must be power of 2 */
#endif

#ifndef HAL_UART_PRINTF_BUF_SIZE
  #define HAL_UART_PRINTF_BUF_SIZE 128
#endif

/* Parity options */
#define HAL_UART_PARITY_NONE    0
#define HAL_UART_PARITY_EVEN    1
#define HAL_UART_PARITY_ODD     2

/* ============================================================
 * Types
 * ============================================================ */

typedef struct {
    uint32_t baud;
    uint8_t  rx_interrupt;      /* 1 = enable interrupt RX with ring buffer */
} hal_uart_config_t;

typedef struct {
    USART_TypeDef    *instance;
    uint32_t          pclk_hz;

    /* Interrupt-driven RX */
    hal_ringbuf_t     rx_ring;
    uint8_t           rx_buf[HAL_UART_RX_BUF_SIZE];
    uint8_t           rx_irq_enabled;

    /* DMA TX state */
    volatile uint8_t  tx_busy;
    hal_callback_t    tx_complete_cb;
    void             *tx_complete_ctx;

    /* Error counters */
    volatile uint32_t rx_overrun;
    volatile uint32_t rx_framing_err;
} hal_uart_t;

/* ============================================================
 * API declarations (implemented in hal_uart.c)
 * ============================================================ */

/**
 * Initialize a UART instance.
 *   h:        handle (caller provides storage)
 *   instance: USART peripheral (USART1, USART2, LPUART1)
 *   pclk_hz:  peripheral clock frequency feeding this USART
 *   cfg:      configuration (baud rate, interrupt RX enable)
 *
 * The peripheral clock is auto-enabled. TX/RX pins must be
 * configured for AF (via tile_init or manually).
 */
hal_status_t hal_uart_init(hal_uart_t *h, USART_TypeDef *instance,
                           uint32_t pclk_hz, const hal_uart_config_t *cfg);

/** Disable the UART and its interrupts */
void hal_uart_deinit(hal_uart_t *h);

/* ---- Blocking TX ---- */

/** Transmit a single byte (blocking) */
void hal_uart_putc(hal_uart_t *h, uint8_t byte);

/** Transmit a buffer (blocking) */
void hal_uart_tx(hal_uart_t *h, const uint8_t *data, uint32_t len);

/** Transmit a null-terminated string (blocking) */
void hal_uart_tx_str(hal_uart_t *h, const char *str);

/** Printf over UART (blocking, uses vsnprintf from newlib-nano) */
int hal_uart_printf(hal_uart_t *h, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

/* ---- RX (polling or interrupt ring buffer) ---- */

/** Check if data is available (works for both polling and interrupt mode) */
int hal_uart_rx_ready(hal_uart_t *h);

/** Receive a single byte (blocking — waits for data) */
uint8_t hal_uart_rx(hal_uart_t *h);

/** Try to receive a byte without blocking. Returns 1 if got one. */
int hal_uart_rx_try(hal_uart_t *h, uint8_t *byte);

/**
 * Read available bytes from the RX ring buffer (interrupt mode).
 * Non-blocking — returns immediately with however many bytes are available.
 * Returns number of bytes read.
 */
uint16_t hal_uart_read(hal_uart_t *h, uint8_t *buf, uint16_t max_len);

/** Number of bytes available in the RX ring buffer */
uint16_t hal_uart_available(hal_uart_t *h);

/* ---- DMA TX (not implemented yet) ---- */

/**
 * Transmit a buffer via DMA (non-blocking). Not implemented yet: returns
 * HAL_ERROR on every Core and never calls the callback. It needs a DMA channel
 * assignment from coregen and per-family DMA setup. Use hal_uart_tx()
 * (polled) until then.
 */
hal_status_t hal_uart_tx_dma(hal_uart_t *h, const uint8_t *data, uint32_t len,
                              hal_callback_t cb, void *ctx);

/** Check if a DMA TX is in progress (always 0 while DMA TX is unimplemented). */
int hal_uart_tx_busy(hal_uart_t *h);

#endif /* HAL_UART_H */
