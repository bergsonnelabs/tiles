/**
 * core_usb_hid.h — USB HID (generic vendor reports)
 *
 * Send and receive raw binary reports over USB HID. No drivers
 * needed on the host — works with hidapi, pyusb, or any HID reader.
 *
 * Reports are automatically zero-padded to 64 bytes. You can send
 * any struct up to 64 bytes — just cast to uint8_t* and pass its size.
 * Inbound reports (host -> device) are delivered one at a time to a
 * callback registered with core_usb_hid_set_rx_callback().
 *
 * The HID interface is part of the composite CDC+HID device —
 * call core_usb_init() first, then use core_usb_hid_send() /
 * core_usb_hid_set_rx_callback().
 *
 * Only available on Core.ST.L4 (STM32L422) and Core.ST.H5 (STM32H523).
 *
 * VID:1209 PID:0001, Interface 2 = HID (vendor-defined, usage page 0xFF00).
 *
 * @studio category usb label=Core.USB icon=❝
 *
 * @studio coverage
 *   id:    usb_hid
 *   name:  USB HID — vendor reports
 *   page:  /docs/sdk/usb
 *   blurb: Tier 1 only. Bidirectional 64-byte vendor HID reports over the
 *          composite CDC+HID device on Core.ST.L4 / Core.ST.H5 — send via
 *          core_usb_hid_send(), receive via a callback. Useful for
 *          high-throughput driverless host I/O (hidapi / pyusb / Web HID),
 *          e.g. the CMSIS-DAP probe transport. No DSL surface —
 *          pointer-buffer ABI doesn't fit the current host-call shape.
 */

#ifndef CORE_USB_HID_H
#define CORE_USB_HID_H

#if !defined(STM32L422xx) && !defined(STM32H523xx)
#error "core_usb_hid.h: USB HID is not available on this Core tile. Only Core.ST.L4 and Core.ST.H5 have USB hardware."
#endif

#include "hal_usb_cdc.h"

/**
 * Send a HID report (up to 64 bytes, zero-padded automatically).
 * Blocking — waits for the previous report to be read by the host.
 * Returns bytes of user data sent, or -1 if USB not configured.
 */
static inline int core_usb_hid_send(const uint8_t *buf, uint16_t len)
{
    return hal_usb_hid_send_report(buf, len);
}

/**
 * Register a callback for inbound HID reports (host -> device).
 * Fires from the USB ISR with one report per call (up to 64 bytes),
 * delivered via the EP3 OUT interrupt endpoint or HID SET_REPORT.
 * Pass NULL to disable; unhandled reports are dropped.
 */
static inline void core_usb_hid_set_rx_callback(hal_usb_hid_rx_cb_t cb, void *ctx)
{
    hal_usb_hid_set_rx_callback(cb, ctx);
}

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=M title="No DSL surface for HID"
//   Sending a HID report needs a buffer pointer + length. The DSL's
//   array-host ABI (used for tile reads) could carry it, but no
//   default-instance wrapper is wired up.
//
// @studio unsupported tier=1 value=L title="Vendor descriptor only"
//   The HID descriptor is hard-coded as a 64-byte vendor report with
//   usage page 0xFF00. No way to register custom reports (keyboard,
//   mouse, gamepad) — that needs the full HID descriptor tooling.

#endif /* CORE_USB_HID_H */
