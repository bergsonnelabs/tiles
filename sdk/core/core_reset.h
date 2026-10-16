/**
 * core_reset.h — Software reset, and why the Core last reset
 *
 *   core_reset()         restart the Core: drop a BLE link cleanly, flush
 *                        USB serial output, then a system reset. Never returns.
 *   core_reset_reason()  what caused the reset this program started from.
 *
 * Use core_reset() when the program has to start over, e.g. after a tile was
 * swapped while running (only init finds and configures it). Studio's reset
 * link calls it remotely: over USB serial (docs/serial-update-protocol.md
 * §10) and over Bluetooth (docs/ble-update-protocol.md, frame R). Over SWD
 * Studio resets the chip itself (AIRCR), which reads here as SOFTWARE too.
 *
 * Reset reason: core_init() reads the RCC reset flags first thing, before
 * anything else can clear them, keeps the result for core_reset_reason(),
 * and clears the flags (RMVF) so the next boot reads only its own reset.
 * Every reset also pulls NRST low, so PIN is set alongside the real cause,
 * and a power-on sets POR/BOR and PIN together. Priority, first match wins:
 *
 *   1. WATCHDOG  IWDGRSTF or WWDGRSTF
 *   2. SOFTWARE  SFTRSTF (AIRCR SYSRESETREQ), or OBLRSTF (option-byte reload,
 *                which only the SDK's L4 flasher and core_reset() trigger)
 *   3. OTHER     LPWRRSTF (illegal Stop/Standby entry), FWRSTF (firewall)
 *   4. POWER_ON  PORRSTF (L0) / BORRSTF (L4, W5, H5)
 *   5. PIN       PINRSTF alone: the NRST pin (a reset button or a probe)
 *   6. OTHER     no flag at all
 *
 * A flag set together with POR/BOR happened after the power-on (a power-on
 * clears the others), so the specific flags rank first.
 *
 * Brownout reads as POWER_ON: every Core sets the same flag for a power-on
 * and a brownout (BORRSTF on the L4, W5 and H5; the L0's PORRSTF covers
 * POR/PDR, and its BOR is off in the factory option bytes). A wake from
 * Standby is not a reset reason here; use core_woke_from_standby().
 *
 * Registers, bits 31..24 on every part (RCC flags, same positions):
 *   Core.ST.L0  RCC_CSR 0x50  (RM0377 §7.3.20)
 *   Core.ST.L4  RCC_CSR 0x94  (RM0394 §6.4.29)
 *   Core.ST.W5  RCC_CSR 0xF4  (RM0493 §12.8.40)
 *   Core.ST.H5  RCC_RSR 0xF4  (RM0481 §11.8.46)
 *
 * @studio category power label=Core.Power icon=☾
 *
 * @studio coverage
 *   id:    reset
 *   name:  Reset: software reset and reset reason
 *   page:  /docs/sdk/system
 *   blurb: core_reset() restarts the Core cleanly (BLE link dropped, USB
 *          serial flushed); core_reset_reason() says what caused the last
 *          reset. Both Tier 2 (Core.Power.reset, Core.Power.reset_reason).
 *          Studio resets a board remotely over USB serial and Bluetooth.
 */

#ifndef CORE_RESET_H
#define CORE_RESET_H

#include <stdint.h>

/** Cause of the reset this program started from (core_reset_reason()). */
typedef enum {
    CORE_RESET_POWER_ON = 0,   /**< Power-on, or a brownout (one flag for both) */
    CORE_RESET_PIN      = 1,   /**< NRST pin: a reset button or a debug probe */
    CORE_RESET_WATCHDOG = 2,   /**< Independent or window watchdog timeout */
    CORE_RESET_SOFTWARE = 3,   /**< core_reset(), a remote reset, the DFU touch, an update's restart, SWD */
    CORE_RESET_OTHER    = 4,   /**< Illegal low-power entry, firewall, or no flag */
} core_reset_reason_t;

/* Raw RCC flag bits (core_reset_flags()): the same positions on every Core. */
#define CORE_RESET_FLAG_LPWR   (1UL << 31)  /**< Illegal Stop/Standby entry */
#define CORE_RESET_FLAG_WWDG   (1UL << 30)  /**< Window watchdog */
#define CORE_RESET_FLAG_IWDG   (1UL << 29)  /**< Independent watchdog */
#define CORE_RESET_FLAG_SFT    (1UL << 28)  /**< Software (AIRCR SYSRESETREQ) */
#define CORE_RESET_FLAG_POR    (1UL << 27)  /**< POR/PDR (L0), BOR (L4, W5, H5) */
#define CORE_RESET_FLAG_PIN    (1UL << 26)  /**< NRST pin (set by every reset) */
#define CORE_RESET_FLAG_OBL    (1UL << 25)  /**< Option-byte reload (not on the H5) */
#define CORE_RESET_FLAG_FW     (1UL << 24)  /**< Firewall (L0, L4 only) */

/**
 * Reset the Core. Does not return.
 *
 * With a BLE central connected (Core.ST.W5), the link is dropped first so the
 * central sees a clean disconnect (waits at most 200 ms). Pending USB serial
 * output is flushed (Core.ST.L4 / H5: waits at most 50 ms for a terminal that
 * stopped reading, longer only while a just-opened port settles). The
 * watchdog is fed while it waits. Then a system reset (AIRCR SYSRESETREQ).
 *
 * Core.ST.L4: if the chip still believes its flash is empty (it was flashed
 * blank by DFU or SWD and not power-cycled since), that stale verdict is
 * corrected first, so the reset starts this program and not the ROM
 * bootloader. Core.ST.H5 with BOOT0 held high resets into the ROM bootloader.
 *
 * Safe from an interrupt handler; the BLE disconnect is then left to the
 * central's supervision timeout.
 *
 * @studio expose category=power name=reset
 */
void core_reset(void) __attribute__((noreturn));

/**
 * Why the Core last reset: power-on (or brownout), the reset pin, the
 * watchdog, a software reset (core_reset(), a remote reset from Studio, an
 * update), or other. Read once by core_init() and kept, so it answers the
 * same for the whole run.
 *
 * @studio expose category=power name=reset_reason returns=int
 * @return core_reset_reason_t: 0 power-on, 1 pin, 2 watchdog, 3 software, 4 other.
 */
core_reset_reason_t core_reset_reason(void);

/**
 * The raw RCC reset flags read at boot (CORE_RESET_FLAG_*; bits 31..24).
 * 0 if no flag was set.
 */
uint32_t core_reset_flags(void);

/**
 * Read and keep the reset flags, then clear them in RCC. core_init() calls
 * this first thing; later calls do nothing. core_reset_reason() calls it too,
 * for a program that never ran core_init().
 */
void core_reset_capture(void);

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=1 value=L title="Brownout reads as power-on"
//   The four Cores set one flag for a power-on and a brownout (BORRSTF on the
//   L4, W5 and H5, PORRSTF on the L0), so core_reset_reason() can't tell them
//   apart and reports both as power-on. Hardware limit, not a driver gap.
//
// @studio unsupported tier=2 value=M title="Studio's simulator doesn't model reset"
//   Core.Power.reset() and Core.Power.reset_reason() run on the board; the
//   simulator needs its own handlers (restart the program, report power-on)
//   in web apps/studio/src/lib/simulator/runtime.ts.
//
// @studio unsupported tier=1 value=L title="Remote and app resets not bench-verified"
//   core_reset(), the USB reset frame and the BLE R frame compile on every
//   Core and pass the host tests (make -C sdk/serial_update test); none has
//   run on a board yet (2026-10-16). tests/hw-reset is the L4 bench test.

#endif /* CORE_RESET_H */
