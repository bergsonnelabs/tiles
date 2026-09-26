/**
 * core_serial.h -- UART serial communication
 *
 * Wraps hal_uart with friendlier names. Instance and clock
 * resolution still requires explicit init (auto-resolve from
 * config.json is planned).
 *
 * @studio category serial label=Core.Serial icon=⌨
 *
 * @studio coverage
 *   id:    serial
 *   name:  Serial — UART
 *   page:  /docs/sdk/uart
 *   blurb: Polled UART send / receive (write, print, putc, getc, read,
 *          available) over hal_uart. Tier 2 exposes the write side —
 *          print_bus + putc_bus against a bus id (1, 2, 6 …) —
 *          coregen resolves the handle via core_serial_handle_for_bus().
 *          Read side (getc/read/available) stays Tier 1 until there's
 *          a twin input affordance. Apps that want printf-style output
 *          to a host should prefer Core.USB; this header is for
 *          hardware UART pins (RS-485, GPS, side-channel debug).
 */

#ifndef CORE_SERIAL_H
#define CORE_SERIAL_H

#include "hal_uart.h"

/* ---- Init ---- */

/** Initialize a UART instance.
 *  Clock is auto-resolved from PCLK1_HZ (core_config.h).
 */
static inline hal_status_t core_serial_init(hal_uart_t *h,
                                             USART_TypeDef *instance,
                                             const hal_uart_config_t *cfg)
{
    return hal_uart_init(h, instance, PCLK1_HZ, cfg);
}

/** @deprecated Use core_serial_init(h, instance, cfg) — clock auto-resolved. */
static inline hal_status_t core_serial_init_clk(hal_uart_t *h,
                                                 USART_TypeDef *instance,
                                                 uint32_t pclk_hz,
                                                 const hal_uart_config_t *cfg)
{
    return hal_uart_init(h, instance, pclk_hz, cfg);
}

/* ---- TX ---- */

/** Transmit a buffer (blocking). */
static inline void core_serial_write(hal_uart_t *h, const uint8_t *data,
                                      uint32_t len)
{
    hal_uart_tx(h, data, len);
}

/** Transmit a null-terminated string (blocking). */
static inline void core_serial_print(hal_uart_t *h, const char *str)
{
    hal_uart_tx_str(h, str);
}

/** Printf over UART (blocking). */
#define core_serial_printf  hal_uart_printf

/** Transmit a single byte (blocking). */
static inline void core_serial_putc(hal_uart_t *h, uint8_t byte)
{
    hal_uart_putc(h, byte);
}

/* ---- RX ---- */

/** Number of bytes available in the RX buffer. */
static inline uint16_t core_serial_available(hal_uart_t *h)
{
    return hal_uart_available(h);
}

/** Receive a single byte (blocking -- waits for data). */
static inline uint8_t core_serial_getc(hal_uart_t *h)
{
    return hal_uart_rx(h);
}

/**
 * Read available bytes from the RX ring buffer (non-blocking).
 * Returns number of bytes read.
 */
static inline uint16_t core_serial_read(hal_uart_t *h, uint8_t *buf,
                                         uint16_t max_len)
{
    return hal_uart_read(h, buf, max_len);
}

/* ---- Tier 2 — default-instance bus helpers ---------------------------- */

/**
 * The UART handle for a bus id declared in config.json, or NULL if the
 * project doesn't declare that bus. Emitted per project by coregen into
 * core_init.c (when the project declares any USART); the *_bus helpers below
 * resolve their handle through it.
 *
 * Forward-declared here rather than `#include "core_init.h"` so this header
 * compiles without a project (val tests, examples without config.json). The
 * natives-side caller is gated on CORE_HAS_SERIAL_BUSES, so the linker never asks for
 * the symbol unless the dispatcher exists.
 *
 * @param bus Bus id (1 = the first instance, ...).
 * @return The coregen-initialized handle, or NULL.
 */
hal_uart_t *core_serial_handle_for_bus(uint8_t bus);

/**
 * Write a null-terminated string to `bus` (blocking). Returns 0 on
 * success or -1 on undeclared bus. The most-common DSL pattern —
 * "drop a debug line on UART2" — is one line:
 *   serial.print(2, "ready\\n");
 *
 * @studio expose category=serial name=print returns=int
 * @studio twin full
 * @param bus UART bus id as declared in config.json (1 = USART1, 2 = USART2, ...).
 * @param str NUL-terminated string to send.
 */
static inline int core_serial_print_bus(uint8_t bus, const char *str)
{
    hal_uart_t *h = core_serial_handle_for_bus(bus);
    if (!h) return -1;
    hal_uart_tx_str(h, str);
    return 0;
}

/**
 * Write a single byte to `bus` (blocking). Returns 0 on success or
 * -1 on undeclared bus.
 *
 * @studio expose category=serial name=putc returns=int
 * @studio twin full
 * @param bus UART bus id as declared in config.json (1 = USART1, 2 = USART2, ...).
 * @param byte Byte to send.
 */
static inline int core_serial_putc_bus(uint8_t bus, uint8_t byte)
{
    hal_uart_t *h = core_serial_handle_for_bus(bus);
    if (!h) return -1;
    hal_uart_putc(h, byte);
    return 0;
}

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=M title="No Tier 2 read surface"
//   Tier 2 wraps the write side (print_bus, putc_bus) only. getc_bus,
//   read_bus, available_bus need a Twin input affordance to be
//   meaningful — without one the read functions would just return
//   -1/0 always, defeating the point. Adding a "serial input" pane
//   (analog of the GPIO toggle / ADC slider) closes both ends.
//
// @studio unsupported tier=2 value=M title="No coregen auto-init for serial"
//   Tier 2 wrappers assume the user has called core_serial_init on the
//   matching handle before any send. Coregen doesn't yet read a
//   `serial: { 1: { baud: 115200 } }` block out of config.json and
//   emit the init in core_init(). Until it does, the bus has to be
//   primed by hand even when using Tier 2.
//
// @studio unsupported tier=1 value=H title="No interrupt / DMA driven path"
//   All calls block. SDK roadmap Tier 1 item: "UART IRQ + coregen"
//   for non-blocking serial (GPS, RS-485). Long reads currently stall
//   the main loop until the buffer fills or times out.
//
// @studio unsupported tier=1 value=M title="No flow control / parity / 9-bit modes"
//   The thin wrappers expose only the basics — RTS/CTS, parity, stop
//   bits, 9-bit data, and inversion are reachable only via the
//   hal_uart_config_t struct passed to init.
//
// @studio unsupported tier=1 value=M title="No LPUART (low-power UART)"
//   SDK roadmap Tier 2: LPUART works in Stop mode for wake-on-serial.
//   Compile-only on every Core today; not surfaced through this header.

#endif /* CORE_SERIAL_H */
