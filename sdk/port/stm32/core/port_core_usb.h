/**
 * port_core_usb.h — STM32 port: the bodies of core_usb.h
 *
 * Included at the end of core_usb.h, after its prototypes; not for direct
 * use. USB CDC over hal_usb_cdc; core_usb_wait_host() feeds the IWDG
 * (ll_iwdg) while it waits.
 */

#ifndef PORT_CORE_USB_H
#define PORT_CORE_USB_H

#include "hal_usb_cdc.h"
#include "ll_iwdg.h"

static inline void core_usb_init(void)
{
    hal_usb_cdc_init();
}

static inline int core_usb_connected(void)
{
    return hal_usb_cdc_connected();
}

static inline int core_usb_wait_host(uint32_t timeout_ms)
{
    uint32_t t0 = hal_tick();
    while (!hal_usb_cdc_connected()) {
        if ((uint32_t)(hal_tick() - t0) >= timeout_ms) return 0;
        ll_iwdg_refresh();                /* harmless if the watchdog isn't running */
    }
    return 1;
}

static inline int core_usb_write(const uint8_t *buf, uint16_t len)
{
    return hal_usb_cdc_write(buf, len);
}

static inline int core_usb_try_write(const uint8_t *buf, uint16_t len)
{
    return hal_usb_cdc_try_write(buf, len);
}

/** Printf over USB CDC: queued like core_usb_write(); waits at most 50 ms for a
 *  stalled terminal, then drops. Returns the formatted length. */
#define core_usb_printf  hal_usb_cdc_printf

static inline void core_usb_print(const char *s)
{
    core_usb_printf("%s\n", s ? s : "");
}

static inline void core_usb_print_int(int v)
{
    core_usb_printf("%d\n", v);
}

static inline void core_usb_print_float(double v)
{
    core_usb_printf("%g\n", v);
}

static inline void core_usb_print_bool(int v)
{
    core_usb_printf("%s\n", v ? "true" : "false");
}

static inline void core_usb_on_receive(core_usb_cdc_rx_cb_t cb, void *ctx)
{
    hal_usb_cdc_set_rx_callback(cb, ctx);
}

static inline uint16_t core_usb_available(void)
{
    return hal_usb_cdc_available();
}

static inline uint8_t core_usb_getc(void)
{
    return hal_usb_cdc_getc();
}

static inline uint16_t core_usb_read(uint8_t *buf, uint16_t max)
{
    return hal_usb_cdc_read(buf, max);
}

static inline int core_usb_try_read(uint8_t *byte)
{
    return hal_usb_cdc_rx_try(byte);
}

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

static inline const char *core_usb_poll_line(core_usb_line_t *lr)
{
    uint8_t b;
    for (int i = 0; i < 64 && core_usb_try_read(&b); i++) {
        const char *line = core_usb_line_feed(lr, b);
        if (line) return line;
    }
    return NULL;
}

#endif /* PORT_CORE_USB_H */
