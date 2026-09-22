/**
 * hal_usb_cdc.h — USB CDC (Virtual Serial Port) driver
 *
 * Implements USB Device with CDC-ACM class on STM32L422.
 * When initialized, the device appears as /dev/tty.usbmodem*
 * on the host. Provides printf-style output and byte-level RX.
 *
 * Crystal-less operation via HSI48 + CRS (synced to USB SOF).
 */

#ifndef HAL_USB_CDC_H
#define HAL_USB_CDC_H

#include "hal_common.h"
#include <stdint.h>
#include <stdarg.h>

#if defined(STM32L422xx) || defined(STM32H523xx)

/* ============================================================
 * Configuration
 * ============================================================ */

#ifndef HAL_USB_CDC_TX_QUEUE_SIZE
  #define HAL_USB_CDC_TX_QUEUE_SIZE 512   /* Must be power of 2; try_write only */
#endif

#ifndef HAL_USB_CDC_RX_BUF_SIZE
  #define HAL_USB_CDC_RX_BUF_SIZE   256   /* Must be power of 2 */
#endif

#ifndef HAL_USB_CDC_PRINTF_BUF_SIZE
  #define HAL_USB_CDC_PRINTF_BUF_SIZE 256
#endif

/* ============================================================
 * RX callback type
 * ============================================================ */

/** Called from USB ISR when data is received on EP1 OUT.
 *  @param data  Received bytes
 *  @param len   Number of bytes
 *  @param ctx   User context (passed at registration time)
 */
typedef void (*hal_usb_cdc_rx_cb_t)(const uint8_t *data, uint16_t len, void *ctx);

/** Called from USB ISR when a HID OUT report arrives (EP3 OUT or
 *  HID SET_REPORT). Delivers one report per call (up to 64 bytes).
 *  @param data  Report bytes copied from PMA
 *  @param len   Number of bytes received
 *  @param ctx   User context (passed at registration time)
 */
typedef void (*hal_usb_hid_rx_cb_t)(const uint8_t *data, uint16_t len, void *ctx);

/* ============================================================
 * API
 * ============================================================ */

/**
 * Initialize USB CDC.
 *
 * This function:
 *   - Enables HSI48 + CRS (crystal-less 48MHz for USB)
 *   - Selects HSI48 as USB clock source
 *   - Enables VDDUSB power supply
 *   - Configures PA11/PA12 as AF10
 *   - Enables USB peripheral clock
 *   - Configures USB device and endpoints
 *   - Enables USB interrupt
 *   - Connects DP pull-up (host sees device)
 */
void hal_usb_cdc_init(void);

/**
 * Check if the host has opened the virtual COM port.
 * Returns 1 if configured and DTR is set (terminal connected).
 */
int hal_usb_cdc_connected(void);

/* ---- TX (device -> host) ---- */

/**
 * Transmit data over USB CDC (blocking).
 * Waits for the host to consume data if the EP is busy.
 * Returns number of bytes sent, or -1 if not configured.
 */
int hal_usb_cdc_write(const uint8_t *buf, uint16_t len);

/**
 * Transmit without waiting (non-blocking).
 * Queues the WHOLE buffer or none of it and returns at once; the USB
 * interrupt sends it. Returns `len` if queued, 0 if the queue has no room
 * right now (offer it again later), -1 if not configured / no terminal.
 * A queued buffer reaches the host contiguously and in call order relative
 * to hal_usb_cdc_write(), which waits for the queue to empty before sending.
 * Call from one context only (the main loop). len <= HAL_USB_CDC_TX_QUEUE_SIZE.
 */
int hal_usb_cdc_try_write(const uint8_t *buf, uint16_t len);

/**
 * Printf over USB CDC (blocking).
 * Returns number of characters written (from vsnprintf).
 */
int hal_usb_cdc_printf(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));

/* ---- RX ---- */

/**
 * Set a callback for received data.
 * Called from the USB ISR with a pointer to data copied from PMA.
 * If set, data is NOT stored in the ring buffer.
 * If not set, data goes into the ring buffer for polling reads.
 */
void hal_usb_cdc_set_rx_callback(hal_usb_cdc_rx_cb_t cb, void *ctx);

/**
 * Check if data is available to read (ring buffer mode).
 */
int hal_usb_cdc_rx_ready(void);

/**
 * Read a single byte (blocking — waits for data).
 */
uint8_t hal_usb_cdc_getc(void);

/**
 * Try to read a byte without blocking.
 * Returns 1 if a byte was read, 0 if no data available.
 */
int hal_usb_cdc_rx_try(uint8_t *byte);

/**
 * Read available bytes into buffer (non-blocking).
 * Returns number of bytes actually read.
 */
uint16_t hal_usb_cdc_read(uint8_t *buf, uint16_t max_len);

/**
 * Number of bytes available in the RX buffer.
 */
uint16_t hal_usb_cdc_available(void);

/**
 * Poll USB events (call from main loop).
 * Use this as an alternative to interrupt-driven operation.
 */
void hal_usb_cdc_poll(void);

/* ---- HID ---- */

/**
 * Send a HID report (up to 64 bytes) via EP3 interrupt IN.
 * Blocking — waits for previous report to complete.
 * Returns bytes sent, or -1 if not configured.
 */
int hal_usb_hid_send_report(const uint8_t *buf, uint16_t len);

/**
 * Set a callback for HID OUT reports (host -> device).
 * Called from the USB ISR with one report per call, copied from PMA.
 * Reception arrives via the EP3 OUT interrupt endpoint and via the
 * HID SET_REPORT control request. If no callback is set, OUT reports
 * are dropped (the endpoint is still re-armed).
 */
void hal_usb_hid_set_rx_callback(hal_usb_hid_rx_cb_t cb, void *ctx);

#endif /* STM32L422xx || STM32H523xx */

#endif /* HAL_USB_CDC_H */
