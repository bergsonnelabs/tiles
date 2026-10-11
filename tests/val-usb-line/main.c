/**
 * val-usb-line -- Validation: USB line reader + Core.USB.line dispatch
 *
 * Core.ST.L4 / Core.ST.H5, USB enabled. Build-only (no hardware checks).
 *
 * Exercises: core_usb_line_feed, core_usb_poll_line, and coregen's
 *            studio_dispatch_core_usb() with both studio_evt_Core_USB_receive
 *            and studio_evt_Core_USB_line overridden, the way a Studio
 *            program that subscribes to both events links.
 */

#include "core.h"
#include "core_usb.h"

static volatile uint32_t rx_bytes;
static volatile uint32_t rx_lines;

/* Strong overrides of coregen's weak defaults (what the DSL emits for
 * `on Core.USB.receive(byte: int)` and `on Core.USB.line(text: string)`). */
void studio_evt_Core_USB_receive(int byte)
{
    (void)byte;
    rx_bytes++;
}

void studio_evt_Core_USB_line(const char *text)
{
    rx_lines++;
    core_usb_print(text);
}

int main(void)
{
    core_init();
    core_usb_init();

    /* Direct assembler use, fed by hand: "ok\r\n" completes "ok". */
    static core_usb_line_t manual;
    const char *line = NULL;
    const char *msg = "ok\r\n";
    for (const char *p = msg; *p; p++) {
        const char *l = core_usb_line_feed(&manual, (uint8_t)*p);
        if (l) line = l;
    }
    (void)line;

    /* C-side polling reader: a C program would loop on this. Here it runs
     * once, before the dispatcher takes over the ring (the two must not
     * share it). */
    static core_usb_line_t polled;
    const char *first = core_usb_poll_line(&polled);
    if (first) core_usb_printf("polled: %s\r\n", first);

    while (1) {
        /* Coregen dispatcher: fires receive per byte, line per line. */
        studio_dispatch_core_usb();
        core_delay_ms(10);
    }
}
