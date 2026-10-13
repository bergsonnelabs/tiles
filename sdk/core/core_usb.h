/**
 * core_usb.h — USB virtual serial port (CDC)
 *
 * Printf-style output, byte-level TX/RX, and receive callbacks
 * over the USB CDC (virtual COM port) interface.
 *
 * Received bytes are drained from the ring buffer by coregen's
 * `studio_dispatch_core_usb()` each main-loop iteration (capped to
 * 64 bytes per iteration so burst traffic doesn't starve the DSL
 * loop). The DSL-facing `Core.USB.receive(byte)` event fires once
 * per byte. The same pass feeds every byte to a line assembler
 * (core_usb_line_feed), and `Core.USB.line(text)` fires once per
 * complete line ("led 1\n" arrives as "led 1"). The two events
 * coexist: a program may subscribe to either or both, and with both
 * it sees every byte and then the line it completes. Gated on
 * `usb.enabled` in config.json — emits a no-op stub when USB isn't
 * enabled.
 *
 * @studio category usb label=Core.USB icon=❝
 * @studio event name=receive payload=byte:int
 * @studio event name=line payload=text:string
 *
 * @studio coverage
 *   id:    usb
 *   name:  USB — virtual serial port (CDC)
 *   page:  /docs/sdk/usb
 *   blurb: Core.ST.L4 / Core.ST.H5 only. CDC virtual COM port — printf-style
 *          output (print, print_int, print_float, print_bool) with
 *          full Twin coverage (the simulator's CDC pane is a real-time
 *          mirror), plus byte-level TX/RX, ring-buffer reads, and a
 *          receive callback, plus a line assembler for text
 *          commands. The DSL `Core.USB.receive(byte:int)` and
 *          `Core.USB.line(text:string)` events are wired through
 *          coregen's USB dispatch when the program subscribes.
 */

#ifndef CORE_USB_H
#define CORE_USB_H

/* USB CDC requires a USB peripheral: Core.ST.L4 (STM32L422) or Core.ST.H5 (STM32H523). */
#if !defined(STM32L422xx) && !defined(STM32H523xx)
#error "core_usb.h: USB is not available on this Core tile. Only Core.ST.L4 and Core.ST.H5 have USB hardware."
#endif

#include "hal_usb_cdc.h"
#include "ll_iwdg.h"
#include <stdio.h>    /* snprintf, printf — commonly used with USB serial output */

/** Initialize USB CDC. Device appears as /dev/tty.usbmodem* on the host. */
static inline void core_usb_init(void)
{
    hal_usb_cdc_init();
}

/** Returns 1 if a host terminal is connected (DTR set). */
static inline int core_usb_connected(void)
{
    return hal_usb_cdc_connected();
}

/**
 * Wait up to `timeout_ms` for a host terminal to open the port (DTR set).
 * Feeds the watchdog while it waits. Not needed just to see early output:
 * text printed before the port opens is kept (up to
 * HAL_USB_CDC_TX_QUEUE_SIZE bytes) and sent when it does.
 *
 * @param timeout_ms Longest wait in milliseconds; 0 only checks.
 * @return 1 if a terminal is connected, 0 on timeout.
 */
static inline int core_usb_wait_host(uint32_t timeout_ms)
{
    uint32_t t0 = hal_tick();
    while (!hal_usb_cdc_connected()) {
        if ((uint32_t)(hal_tick() - t0) >= timeout_ms) return 0;
        ll_iwdg_refresh();                /* harmless if the watchdog isn't running */
    }
    return 1;
}

/** Transmit data: queued, never waits long (see hal_usb_cdc_write); returns
 * bytes queued, -1 before init. */
static inline int core_usb_write(const uint8_t *buf, uint16_t len)
{
    return hal_usb_cdc_write(buf, len);
}

/** Transmit without waiting: queues the whole buffer or none of it.
 * Returns len if queued, 0 if there is no room right now (offer it again
 * later), -1 if no terminal is connected. Main loop only. */
static inline int core_usb_try_write(const uint8_t *buf, uint16_t len)
{
    return hal_usb_cdc_try_write(buf, len);
}

/** Printf over USB CDC: queued like core_usb_write(); waits at most 50 ms for a
 *  stalled terminal, then drops. Returns the formatted length. */
#define core_usb_printf  hal_usb_cdc_printf

/**
 * Print a string followed by a newline. Thin wrapper for DSL-style callers.
 *
 * @studio expose category=usb icon=❝ name=print availability=Core.ST.L4,Core.ST.H5
 * @studio twin full
 * @param text The message to print.
 */
static inline void core_usb_print(const char *s)
{
    core_usb_printf("%s\n", s ? s : "");
}

/**
 * Print a signed integer followed by a newline.
 *
 * @studio expose category=usb name=print_int availability=Core.ST.L4,Core.ST.H5
 * @studio twin full
 * @param value The integer to print (decimal).
 */
static inline void core_usb_print_int(int v)
{
    core_usb_printf("%d\n", v);
}

/**
 * Print a double with a newline. %g trims trailing zeros for readability.
 *
 * @studio expose category=usb name=print_float availability=Core.ST.L4,Core.ST.H5
 * @studio twin full
 * @param value The float to print.
 */
static inline void core_usb_print_float(double v)
{
    core_usb_printf("%g\n", v);
}

/**
 * Print "true" / "false" followed by a newline.
 *
 * @studio expose category=usb name=print_bool availability=Core.ST.L4,Core.ST.H5
 * @studio twin full
 * @param value {bool} The boolean to print.
 */
static inline void core_usb_print_bool(int v)
{
    core_usb_printf("%s\n", v ? "true" : "false");
}

/**
 * Set a callback for received data.
 * Called from USB ISR. When set, data is NOT buffered for polling reads.
 * @param cb   Callback: void cb(const uint8_t *data, uint16_t len, void *ctx)
 * @param ctx  User context passed to callback; may be NULL
 */
static inline void core_usb_on_receive(hal_usb_cdc_rx_cb_t cb, void *ctx)
{
    hal_usb_cdc_set_rx_callback(cb, ctx);
}

/** Returns the number of bytes available to read (ring buffer mode). */
static inline uint16_t core_usb_available(void)
{
    return hal_usb_cdc_available();
}

/** Read a single byte (blocking — waits for data). */
static inline uint8_t core_usb_getc(void)
{
    return hal_usb_cdc_getc();
}

/** Read available bytes into buf (non-blocking). Returns bytes read. */
static inline uint16_t core_usb_read(uint8_t *buf, uint16_t max)
{
    return hal_usb_cdc_read(buf, max);
}

/** Non-blocking single byte read. Returns 1 if a byte was read, 0 if empty. */
static inline int core_usb_try_read(uint8_t *byte)
{
    return hal_usb_cdc_rx_try(byte);
}

/* ---- Line reader ---- */

/** Longest line core_usb_line_feed() assembles, not counting the NUL. */
#define CORE_USB_LINE_MAX  127

/** Line-assembler state for core_usb_line_feed() / core_usb_poll_line().
 * Zero-initialize it (a static or global is). */
typedef struct {
    char    buf[CORE_USB_LINE_MAX + 1];  /**< Line so far, then the NUL-terminated line. */
    uint8_t len;                         /**< Characters collected. */
    uint8_t overflow;                    /**< Set while discarding an over-long line. */
} core_usb_line_t;

/**
 * Feed one received byte to a line assembler.
 *
 * A line ends at '\n'. '\r' and NUL bytes are dropped, so "\r\n" and "\n"
 * endings both work. Empty lines are ignored. A line longer than
 * CORE_USB_LINE_MAX characters is dropped whole: the assembler discards
 * everything up to the next '\n' and starts clean after it.
 *
 * @param lr    Assembler state (zero-initialized before first use).
 * @param byte  The received byte.
 * @return The completed line, NUL-terminated and without its ending, or NULL
 *         if this byte did not complete one. The text stays valid until the
 *         next call with the same `lr`.
 */
static inline const char *core_usb_line_feed(core_usb_line_t *lr, uint8_t byte)
{
    if (byte == '\r' || byte == '\0') return NULL;
    if (byte == '\n') {
        uint8_t n = lr->len, dropped = lr->overflow;
        lr->len = 0;
        lr->overflow = 0;
        if (dropped || n == 0) return NULL;
        lr->buf[n] = '\0';
        return lr->buf;
    }
    if (lr->overflow) return NULL;
    if (lr->len >= CORE_USB_LINE_MAX) {
        lr->overflow = 1;               /* too long: drop it, resync on '\n' */
        lr->len = 0;
        return NULL;
    }
    lr->buf[lr->len++] = (char)byte;
    return NULL;
}

/**
 * Read received bytes until a line completes (non-blocking).
 *
 * Takes bytes from the receive ring with core_usb_try_read(), at most 64 per
 * call so a flood can't stall the main loop, and stops at the first complete
 * line; later bytes stay in the ring for the next call. Call it from the main
 * loop. Not for use with core_usb_on_receive(), which bypasses the ring, and
 * not alongside other ring reads, which would steal bytes from the line.
 *
 * Studio programs don't call this: `on Core.USB.line(text)` gets each line
 * from coregen's dispatcher, which feeds core_usb_line_feed() directly.
 *
 * @param lr  Assembler state (zero-initialized before first use).
 * @return The completed line, NUL-terminated and without its ending, or NULL
 *         if no line is complete yet. Valid until the next call with `lr`.
 */
static inline const char *core_usb_poll_line(core_usb_line_t *lr)
{
    uint8_t b;
    for (int i = 0; i < 64 && core_usb_try_read(&b); i++) {
        const char *line = core_usb_line_feed(lr, b);
        if (line) return line;
    }
    return NULL;
}

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=M title="No DSL surface for receive (byte read / available)"
//   The Core.USB.receive event handles inbound bytes one at a time,
//   and Core.USB.line delivers newline-terminated text commands, but
//   DSL programs can't poll core_usb_available() / core_usb_read().
//   That makes binary message-framed protocols (read N bytes after a
//   header) awkward — the workaround is to accumulate bytes in a DSL
//   array inside the receive handler.
//
// @studio unsupported tier=2 value=L title="No DSL printf-with-formatting"
//   print_int / float / bool emit one value per call, each on its own
//   line. For multi-field log lines DSL programs concatenate with
//   string ops (or call print multiple times). A `print_fmt(template,
//   ...args)` host call would be the next step.
//
// @studio unsupported tier=1 value=M title="USB MSC (mass storage)"
//   SDK roadmap Tier 3: drag-and-drop firmware / datalog volume needs
//   SCSI + FAT. A feature gap, not a workflow blocker — projects that
//   need persistent storage today reach for NVM or SD via SPI.
//
// @studio unsupported tier=1 value=M title="USB Host mode"
//   Core.ST.H5 has DRD silicon but the wrapper is device-only. Most
//   projects don't need host mode; the few that do (USB-keyboard /
//   thumb-drive readers) drop into the lower-layer USB stack.
//
// @studio unsupported tier=1 value=M title="No CDC line-coding callback"
//   Host-side baud changes (1200-touch bootloader trigger included)
//   land in the lower stack. The header doesn't expose a hook for
//   programs that want to react to baud / DTR changes.

#endif /* CORE_USB_H */
