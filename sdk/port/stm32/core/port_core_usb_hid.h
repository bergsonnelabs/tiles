/**
 * port_core_usb_hid.h — STM32 port: the bodies of core_usb_hid.h
 *
 * Included at the end of core_usb_hid.h, after its prototypes; not for
 * direct use. HID reports over the composite CDC+HID device (hal_usb_cdc).
 */

#ifndef PORT_CORE_USB_HID_H
#define PORT_CORE_USB_HID_H

#include "hal_usb_cdc.h"

static inline int core_usb_hid_send(const uint8_t *buf, uint16_t len)
{
    return hal_usb_hid_send_report(buf, len);
}

static inline void core_usb_hid_set_rx_callback(core_usb_hid_rx_cb_t cb, void *ctx)
{
    hal_usb_hid_set_rx_callback(cb, ctx);
}

#endif /* PORT_CORE_USB_HID_H */
