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
  #define HAL_USB_CDC_TX_QUEUE_SIZE 512   /* Must be power of 2. L4: every write;
                                             H5: try_write only */
#endif

/* L4: longest a write waits for a terminal that has stopped reading (no
 * packet collected for this long) before it drops, and a HID report for the
 * previous one. After one timeout, writes drop at once until the host reads. */
#ifndef HAL_USB_CDC_TX_TIMEOUT_MS
  #define HAL_USB_CDC_TX_TIMEOUT_MS 50
#endif
#ifndef HAL_USB_HID_TX_TIMEOUT_MS
  #define HAL_USB_HID_TX_TIMEOUT_MS 50
#endif

/* L4: after a terminal raises DTR, output waits this long (USB frames, ms)
 * before it flows, so the host's open-time input flush (pyserial, Windows
 * PurgeComm) doesn't discard what was printed before the port opened. */
#ifndef HAL_USB_CDC_OPEN_SETTLE_MS
  #define HAL_USB_CDC_OPEN_SETTLE_MS 100
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
 *
 * L4: core_init() already calls it; later calls do nothing.
 */
void hal_usb_cdc_init(void);

/**
 * Check if the host has opened the virtual COM port.
 * Returns 1 if configured and DTR is set (terminal connected).
 */
int hal_usb_cdc_connected(void);

/* ---- TX (device -> host) ---- */

/**
 * Transmit data over USB CDC.
 *
 * L4: queues into the TX FIFO (HAL_USB_CDC_TX_QUEUE_SIZE) and returns once the
 * bytes are queued. Before a terminal opens the port (DTR) the bytes wait in
 * the FIFO and go out when it does; what doesn't fit is dropped (the oldest
 * text is kept). With a terminal that isn't reading, it waits at most
 * HAL_USB_CDC_TX_TIMEOUT_MS, then drops. Never waits in an interrupt handler.
 * Returns the number of bytes queued (short = the rest was dropped), or -1
 * before hal_usb_cdc_init().
 *
 * H5: blocking; waits for each packet. Returns bytes sent, or -1 if no
 * terminal is connected.
 */
int hal_usb_cdc_write(const uint8_t *buf, uint16_t len);

/**
 * Transmit without waiting (non-blocking).
 * Queues the WHOLE buffer or none of it and returns at once; the USB
 * interrupt sends it. Returns `len` if queued, 0 if the queue has no room
 * right now (offer it again later), -1 if not configured / no terminal.
 * A queued buffer reaches the host contiguously and in call order relative
 * to hal_usb_cdc_write() (L4: the same FIFO; H5: write waits for it to empty).
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

#if defined(STM32L422xx)
/**
 * 1 while the bus is suspended (no SOF for 3 ms: the host is asleep, has
 * suspended the port, or there is no host). Writes don't wait meanwhile.
 */
int hal_usb_cdc_suspended(void);

/* ---- Low power (used by core_power.h) ---- */

/** 1 once hal_usb_cdc_init() has run: after Stop the clocks (HSI48 included)
 *  must be restored before the USB interrupt runs. */
int hal_usb_cdc_started(void);

/**
 * May the chip enter Stop now? 1 when USB can't be hurt by it: not started,
 * or the bus is suspended, in which case this also puts the transceivers in
 * low-power mode; the host's resume or reset then ends Stop (USB wakeup on
 * EXTI line 17; Stop 0/1 only). 0 while a host has the bus awake: HSI48 stops
 * in Stop and the Core would stop answering, so wait in Sleep instead.
 * Call with interrupts masked, right before entering Stop.
 */
int hal_usb_cdc_stop_allowed(void);

#ifdef HAL_USB_CDC_TEST_HOOKS
/* Bench only (tests/hw-usb-robust): Stops taken with the bus suspended and
 * USB wakeups, since boot. */
void hal_usb_cdc_test_stats(uint32_t *stops, uint32_t *wakes);
uint32_t hal_usb_cdc_test_log(const uint32_t **log);   /* (ms << 8) | event */
void hal_usb_cdc_test_force_suspend(void);   /* next Stop as if suspended */
#endif
#endif

/**
 * Poll USB events (call from main loop).
 * Use this as an alternative to interrupt-driven operation.
 */
void hal_usb_cdc_poll(void);

/* ---- HID ---- */

/**
 * Send a HID report (up to 64 bytes) via EP3 interrupt IN.
 * Waits for the previous report to complete (L4: at most
 * HAL_USB_HID_TX_TIMEOUT_MS, never in an interrupt handler or while the bus
 * is suspended; a report that can't go is dropped).
 * Returns bytes sent, or -1 if not configured or dropped.
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
