/**
 * hw-swd-attach: a quiet image for SWD bench work (Core.ST.H5 by default).
 *
 * The point is a target SWD can attach to at any moment and tell "alive and
 * not resetting" from the outside:
 *   - never resets itself, runs no watchdog ("iwdg" is absent from
 *     config.json), never sleeps or changes clocks after core_init()
 *   - keeps USB CDC and serial update (core_init() brings them up on every
 *     USB-capable Core), so the board goes back to USB-only use with
 *     `make flash-serial` / `make flash-dfu` and no probe
 *
 * SWD-visible state (addresses from the ELF; swd_attach.py reads them):
 *   g_swd.heartbeat  +1 per main-loop pass (millions per second)
 *   g_swd.ms         core_millis(), copied every pass
 *   g_swd.boot       copy of g_boot.boots for this boot
 *   g_boot           .noinit, kept across resets: magic + boots since power-on
 *                    (a new magic = first boot after a power-on)
 *
 * Over CDC it prints "hb <heartbeat> ms <ms> boots <n>" once a second while a
 * terminal has the port open; "V" answers "HELLO hw-swd-attach".
 *
 *   make                          # build (Core.ST.H5)
 *   make flash-serial             # over USB, or over SWD:
 *   probe-rs download --chip STM32H523HEYxT --binary-format elf build/hw-swd-attach.elf
 *   probe-rs reset --chip STM32H523HEYxT
 *   python3 swd_attach.py         # hot-attach series, see its docstring
 *
 * Power the board before `make flash-coreprobe`: this config declares no
 * "probe" block on purpose. A bare Core.ST.H5 takes 1.71-3.6 V on pad 10
 * (definitions/Core-ST-H5-a.json), so "target_power": "5v" is only right for a
 * carrier that regulates, and that belongs in the bench's own config.
 */

#include "core.h"

#define BOOT_MAGIC 0x5D0B007UL

typedef struct {
    uint32_t magic;
    uint32_t boots;
} swd_boot_t;

typedef struct {
    volatile uint32_t heartbeat;
    volatile uint32_t ms;
    volatile uint32_t boot;
} swd_state_t;

__attribute__((section(".noinit"))) swd_boot_t g_boot;
swd_state_t g_swd;

int main(void)
{
    if (g_boot.magic != BOOT_MAGIC) {
        g_boot.magic = BOOT_MAGIC;
        g_boot.boots = 0;
    }
    g_boot.boots++;
    g_swd.boot = g_boot.boots;

    core_init();                 /* clock, USB CDC + serial update; no watchdog */

    uint32_t last_print = 0;
    for (;;) {
        g_swd.heartbeat++;
        uint32_t now = core_millis();
        g_swd.ms = now;

        if (core_usb_connected() && (uint32_t)(now - last_print) >= 1000u) {
            last_print = now;
            core_usb_printf("hb %lu ms %lu boots %lu\n", (unsigned long)g_swd.heartbeat,
                            (unsigned long)now, (unsigned long)g_boot.boots);
        }
        uint8_t b;
        while (core_usb_try_read(&b)) {
            if (b == 'V') core_usb_printf("HELLO hw-swd-attach\n");
        }
    }
}
