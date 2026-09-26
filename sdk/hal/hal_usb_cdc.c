/**
 * hal_usb_cdc.c — USB CDC (Virtual Serial Port) implementation
 *
 * Self-contained USB Device core + CDC-ACM class.
 * Handles enumeration, control transfers, and bulk data entirely
 * in the USB interrupt handler.
 *
 * Supported peripherals:
 *   - STM32L422: USB Device FS (16-bit registers, 1KB PMA)
 *   - STM32H523: USB DRD Full-Speed (32-bit registers, 2KB PMA)
 */

#include "hal_usb_cdc.h"

/* Per-project USB identity. coregen emits CORE_USB_* into core_config.h from
 * the config.json "usb" block; anything unset falls back to the defaults
 * below. CMSIS-DAP hosts (pyOCD/OpenOCD) auto-detect by matching "CMSIS-DAP"
 * in the product string, so a probe sets usb.product accordingly. */
#if defined(__has_include)
#  if __has_include("core_config.h")
#    include "core_config.h"
#  endif
#endif
#ifndef CORE_USB_VID
#  define CORE_USB_VID 0x1209  /* pid.codes open-source VID */
#endif
#ifndef CORE_USB_PID
#  define CORE_USB_PID 0x0001  /* pid.codes' shared test PID: a real PID is a
                                * business decision, not made here */
#endif
#ifndef CORE_USB_MANUFACTURER
#  define CORE_USB_MANUFACTURER "Bergsonne"
#endif
#ifndef CORE_USB_PRODUCT
#  define CORE_USB_PRODUCT "Core Tile"
#endif
/* Serial number. A fixed one from config.json wins; otherwise the L4 reports
 * its 96-bit factory UID as 24 hex digits (built at init), so a host can tell
 * Cores apart (Windows COM numbers, Web Serial permissions, macOS tty names).
 * The H5 keeps the old fixed string until its own USB pass. */
#if defined(CORE_USB_SERIAL)
#  define HAL_USB_SERIAL_FIXED 1
#else
#  define HAL_USB_SERIAL_FIXED 0
#  if !defined(STM32L422xx)
#    define CORE_USB_SERIAL "000001"
#  endif
#endif
/* bMaxPower. 100 mA (one unit load) is what every port, bus-powered hubs
 * included, must grant, so the Core configures anywhere. The Core itself draws
 * ~10-20 mA; a project that knowingly pulls more through USB can raise it. */
#ifndef CORE_USB_MAX_POWER_MA
#  define CORE_USB_MAX_POWER_MA 100
#endif
#if CORE_USB_MAX_POWER_MA > 500 || CORE_USB_MAX_POWER_MA < 2
#  error "CORE_USB_MAX_POWER_MA must be 2..500"
#endif

#if defined(STM32L422xx)

#include "ll_usb.h"
#include "ll_rcc.h"
#include "ll_crs.h"
#include "ll_gpio.h"
#include "ll_exti.h"
#include "ll_systick.h"
#include "hal_dfu.h"
#include "core_uid.h"      /* 96-bit UID base, RM0394 §48.1 (0x1FFF7590) */
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* Serial update (docs/serial-update-protocol.md): opening the port at
 * SU_TRIGGER_BAUD hands the Core to a flasher that runs from SRAM. Only in the
 * layout it rewrites — app at 0x08000000 — so not under the parked custom
 * bootloader (APP_OFFSET). */
#if !defined(APP_OFFSET) && !defined(HAL_SERIAL_UPDATE_DISABLE)
#  define HAL_SERIAL_UPDATE 1
#  include "../serial_update/su_protocol.h"
#  include "hal_serial_update_blob_l4.h"
#else
#  define HAL_SERIAL_UPDATE 0
#endif

/* ============================================================
 * PMA buffer layout (1024 bytes total, BTABLE=0, PMA_ACCESS=1)
 *
 * BDT:  0x00 - 0x27  (5 endpoints x 8 bytes = 40 bytes; EP3 slot incl.)
 * EP0 TX: 0x28 - 0x67  (64 bytes)
 * EP0 RX: 0x68 - 0xA7  (64 bytes)
 * EP1 TX: 0xA8 - 0xE7  (64 bytes)
 * EP1 RX: 0xE8 - 0x127 (64 bytes)
 * EP2 TX: 0x128 - 0x137 (16 bytes)
 * EP3 TX: 0x138 - 0x177 (64 bytes) — HID reports IN  (device -> host)
 * EP3 RX: 0x178 - 0x1B7 (64 bytes) — HID reports OUT (host -> device)
 * ============================================================ */

#define PMA_EP0_TX          0x28
#define PMA_EP0_RX          0x68
#define PMA_EP1_TX          0xA8
#define PMA_EP1_RX          0xE8
#define PMA_EP2_TX          0x128
#define PMA_EP3_TX          0x138
#define PMA_EP3_RX          0x178

#define EP0_MAX_PACKET      64
#define EP1_MAX_PACKET      64
#define EP2_MAX_PACKET      8
#define EP3_MAX_PACKET      64

/* ============================================================
 * USB descriptor data
 * ============================================================ */

/* Standard USB request types */
#define USB_REQ_GET_STATUS          0x00
#define USB_REQ_CLEAR_FEATURE       0x01
#define USB_REQ_SET_FEATURE         0x03
#define USB_REQ_SET_ADDRESS         0x05
#define USB_REQ_GET_DESCRIPTOR      0x06
#define USB_REQ_SET_DESCRIPTOR      0x07
#define USB_REQ_GET_CONFIGURATION   0x08
#define USB_REQ_SET_CONFIGURATION   0x09

/* Descriptor types */
#define USB_DESC_DEVICE             0x01
#define USB_DESC_CONFIGURATION      0x02
#define USB_DESC_STRING             0x03
#define USB_DESC_INTERFACE          0x04
#define USB_DESC_ENDPOINT           0x05
#define USB_DESC_DEVICE_QUALIFIER   0x06
#define USB_DESC_CS_INTERFACE       0x24

/* CDC class requests */
#define CDC_SET_LINE_CODING         0x20
#define CDC_GET_LINE_CODING         0x21
#define CDC_SET_CONTROL_LINE_STATE  0x22
#define CDC_SEND_BREAK              0x23

/* CDC subclass/protocol */
#define CDC_ACM_SUBCLASS            0x02
#define CDC_AT_PROTOCOL             0x01

/* Composite device: CDC-ACM (serial) + HID (generic reports).
 * Uses IAD (Interface Association Descriptor) to group CDC interfaces. */
#define USB_DESC_IAD            0x0B
#define USB_DESC_HID            0x21
#define USB_DESC_HID_REPORT     0x22

/* HID class requests */
#define HID_GET_REPORT          0x01
#define HID_GET_IDLE            0x02
#define HID_SET_REPORT          0x09
#define HID_SET_IDLE            0x0A

/* Device descriptor
 * VID=0x1209 (pid.codes open source), PID=0x0001 (placeholder) */
static const uint8_t dev_desc[] = {
    18,                     /* bLength */
    USB_DESC_DEVICE,        /* bDescriptorType */
    0x00, 0x02,             /* bcdUSB = 2.00 */
    0xEF,                   /* bDeviceClass = Miscellaneous (composite) */
    0x02,                   /* bDeviceSubClass = Common Class */
    0x01,                   /* bDeviceProtocol = IAD */
    EP0_MAX_PACKET,         /* bMaxPacketSize0 */
    (CORE_USB_VID & 0xFF), ((CORE_USB_VID >> 8) & 0xFF),       /* idVendor */
    (CORE_USB_PID & 0xFF), ((CORE_USB_PID >> 8) & 0xFF),       /* idProduct */
    0x00, 0x01,             /* bcdDevice = 1.00 */
    1,                      /* iManufacturer (string index 1) */
    2,                      /* iProduct (string index 2) */
    3,                      /* iSerialNumber (string index 3) */
    1,                      /* bNumConfigurations */
};

/* HID Report Descriptor — generic vendor-defined, bidirectional 64-byte
 * reports (Input + Output). Identical to the H523 reference. */
static const uint8_t hid_report_desc[] = {
    0x06, 0x00, 0xFF,       /* Usage Page (Vendor Defined 0xFF00) */
    0x09, 0x01,             /* Usage (Vendor Usage 1) */
    0xA1, 0x01,             /* Collection (Application) */
    0x15, 0x00,             /*   Logical Minimum (0) */
    0x26, 0xFF, 0x00,       /*   Logical Maximum (255) */
    0x75, 0x08,             /*   Report Size (8 bits) */
    0x95, 0x40,             /*   Report Count (64) */
    0x09, 0x01,             /*   Usage (Vendor Usage 1) */
    0x81, 0x02,             /*   Input (Data, Variable, Absolute) */
    0x09, 0x02,             /*   Usage (Vendor Usage 2) */
    0x91, 0x02,             /*   Output (Data, Variable, Absolute) */
    0xC0,                   /* End Collection */
};

#define HID_REPORT_DESC_LEN  sizeof(hid_report_desc)

/* Configuration descriptor — composite CDC-ACM (serial) + HID (vendor reports).
 * Config(9) + IAD(8) + CDC_IF0(9) + Func(5+4+5+5=19) + EP2(7) +
 * CDC_IF1(9) + EP1_OUT(7) + EP1_IN(7) + HID_IF2(9) + HID_Desc(9) +
 * EP3_IN(7) + EP3_OUT(7) = 107 bytes */
#define CONFIG_DESC_TOTAL_LEN   107

static const uint8_t cfg_desc[] = {
    /* Configuration descriptor */
    9,                      /* bLength */
    USB_DESC_CONFIGURATION, /* bDescriptorType */
    CONFIG_DESC_TOTAL_LEN, 0x00, /* wTotalLength */
    3,                      /* bNumInterfaces (CDC control + data + HID) */
    1,                      /* bConfigurationValue */
    0,                      /* iConfiguration */
    0x80,                   /* bmAttributes: bus-powered */
    CORE_USB_MAX_POWER_MA / 2, /* bMaxPower, 2 mA units */

    /* ---- IAD: group CDC interfaces 0+1 ---- */
    8,                      /* bLength */
    USB_DESC_IAD,           /* bDescriptorType */
    0,                      /* bFirstInterface */
    2,                      /* bInterfaceCount */
    0x02,                   /* bFunctionClass: Communications */
    CDC_ACM_SUBCLASS,       /* bFunctionSubClass: ACM */
    CDC_AT_PROTOCOL,        /* bFunctionProtocol: AT commands */
    0,                      /* iFunction */

    /* ---- CDC Control Interface (Interface 0) ---- */
    9,                      /* bLength */
    USB_DESC_INTERFACE,     /* bDescriptorType */
    0,                      /* bInterfaceNumber */
    0,                      /* bAlternateSetting */
    1,                      /* bNumEndpoints (EP2 IN interrupt) */
    0x02,                   /* bInterfaceClass: Communications */
    CDC_ACM_SUBCLASS,       /* bInterfaceSubClass: ACM */
    CDC_AT_PROTOCOL,        /* bInterfaceProtocol: AT commands */
    0,                      /* iInterface */

    /* CDC Header Functional Descriptor */
    5, USB_DESC_CS_INTERFACE, 0x00, 0x10, 0x01,

    /* CDC ACM Functional Descriptor */
    4, USB_DESC_CS_INTERFACE, 0x02, 0x02,

    /* CDC Union Functional Descriptor */
    5, USB_DESC_CS_INTERFACE, 0x06, 0, 1,   /* control=0, data=1 */

    /* CDC Call Management Functional Descriptor */
    5, USB_DESC_CS_INTERFACE, 0x01, 0x00, 1,

    /* EP2 IN — CDC notification (interrupt) */
    7,                      /* bLength */
    USB_DESC_ENDPOINT,      /* bDescriptorType */
    0x82,                   /* bEndpointAddress: EP2 IN */
    0x03,                   /* bmAttributes: Interrupt */
    EP2_MAX_PACKET, 0x00,   /* wMaxPacketSize */
    255,                    /* bInterval: 255ms */

    /* ---- CDC Data Interface (Interface 1) ---- */
    9,                      /* bLength */
    USB_DESC_INTERFACE,     /* bDescriptorType */
    1,                      /* bInterfaceNumber */
    0,                      /* bAlternateSetting */
    2,                      /* bNumEndpoints (EP1 IN + EP1 OUT) */
    0x0A,                   /* bInterfaceClass: Data */
    0x00,                   /* bInterfaceSubClass */
    0x00,                   /* bInterfaceProtocol */
    0,                      /* iInterface */

    /* EP1 OUT — CDC bulk data */
    7,                      /* bLength */
    USB_DESC_ENDPOINT,      /* bDescriptorType */
    0x01,                   /* bEndpointAddress: EP1 OUT */
    0x02,                   /* bmAttributes: Bulk */
    EP1_MAX_PACKET, 0x00,   /* wMaxPacketSize */
    0,                      /* bInterval */

    /* EP1 IN — CDC bulk data */
    7,                      /* bLength */
    USB_DESC_ENDPOINT,      /* bDescriptorType */
    0x81,                   /* bEndpointAddress: EP1 IN */
    0x02,                   /* bmAttributes: Bulk */
    EP1_MAX_PACKET, 0x00,   /* wMaxPacketSize */
    0,                      /* bInterval */

    /* ---- HID Interface (Interface 2) ---- */
    9,                      /* bLength */
    USB_DESC_INTERFACE,     /* bDescriptorType */
    2,                      /* bInterfaceNumber */
    0,                      /* bAlternateSetting */
    2,                      /* bNumEndpoints (EP3 IN + EP3 OUT) */
    0x03,                   /* bInterfaceClass: HID */
    0x00,                   /* bInterfaceSubClass: none */
    0x00,                   /* bInterfaceProtocol: none */
    0,                      /* iInterface */

    /* HID Descriptor */
    9,                      /* bLength */
    USB_DESC_HID,           /* bDescriptorType */
    0x11, 0x01,             /* bcdHID = 1.11 */
    0x00,                   /* bCountryCode = 0 */
    1,                      /* bNumDescriptors */
    USB_DESC_HID_REPORT,    /* bDescriptorType = Report */
    HID_REPORT_DESC_LEN, 0, /* wDescriptorLength */

    /* EP3 IN — HID reports (interrupt, device -> host) */
    7,                      /* bLength */
    USB_DESC_ENDPOINT,      /* bDescriptorType */
    0x83,                   /* bEndpointAddress: EP3 IN */
    0x03,                   /* bmAttributes: Interrupt */
    EP3_MAX_PACKET, 0x00,   /* wMaxPacketSize */
    1,                      /* bInterval: 1ms for fastest polling */

    /* EP3 OUT — HID reports (interrupt, host -> device) */
    7,                      /* bLength */
    USB_DESC_ENDPOINT,      /* bDescriptorType */
    0x03,                   /* bEndpointAddress: EP3 OUT */
    0x03,                   /* bmAttributes: Interrupt */
    EP3_MAX_PACKET, 0x00,   /* wMaxPacketSize */
    1,                      /* bInterval: 1ms */
};

/* Offset of the HID descriptor within cfg_desc (for GET_DESCRIPTOR HID).
 * Config(9) + IAD(8) + CDC_IF0(9) + Func(19) + EP2(7) + CDC_IF1(9)
 * + EP1out(7) + EP1in(7) + HID_IF(9) = 84 */
#define CFG_DESC_HID_OFFSET  84

/* Guard the hand-computed descriptor length against future edits. */
_Static_assert(sizeof(cfg_desc) == CONFIG_DESC_TOTAL_LEN,
               "cfg_desc length != CONFIG_DESC_TOTAL_LEN");

/* String descriptor 0: Language ID */
static const uint8_t str0_desc[] = { 4, USB_DESC_STRING, 0x09, 0x04 };

/* Helper: convert ASCII string to USB string descriptor in-place.
 * Returns total descriptor length. */
static uint8_t _str_to_desc(const char *str, uint8_t *buf, uint8_t max_len)
{
    uint8_t slen = 0;
    while (str[slen]) slen++;
    uint8_t dlen = 2 + slen * 2;
    if (dlen > max_len) dlen = max_len;
    buf[0] = dlen;
    buf[1] = USB_DESC_STRING;
    for (uint8_t i = 0; i < (dlen - 2) / 2; i++) {
        buf[2 + i * 2] = str[i];
        buf[3 + i * 2] = 0;
    }
    return dlen;
}

/* ============================================================
 * Internal state
 * ============================================================ */

/* USB device state */
typedef enum {
    USB_STATE_DEFAULT = 0,
    USB_STATE_ADDRESS,
    USB_STATE_CONFIGURED,
} usb_state_t;

/* EP0 control transfer state */
typedef enum {
    EP0_IDLE = 0,
    EP0_DATA_IN,
    EP0_DATA_OUT,
    EP0_STATUS_IN,
    EP0_STATUS_OUT,
} ep0_state_t;

/* Setup packet */
typedef struct {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb_setup_t;

/* CDC line coding (baud, stop, parity, data bits) */
typedef struct {
    uint32_t dwDTERate;
    uint8_t  bCharFormat;
    uint8_t  bParityType;
    uint8_t  bDataBits;
} __attribute__((packed)) cdc_line_coding_t;

/* Global CDC state */
static struct {
    volatile usb_state_t state;
    volatile uint8_t     address_pending;   /* Set after SET_ADDRESS, applied after STATUS */
    volatile uint8_t     configured;
    volatile uint8_t     dtr;               /* DTR line state from host */

    /* EP0 control transfer */
    ep0_state_t          ep0_state;
    const uint8_t       *ep0_tx_ptr;
    uint16_t             ep0_tx_remain;

    /* EP0 OUT data receive (sized for a full 64-byte HID SET_REPORT payload) */
    uint8_t              ep0_rx_buf[EP0_MAX_PACKET];
    /* What the pending EP0 DATA OUT stage is carrying (0 = CDC line coding,
     * 1 = HID SET_REPORT). Set by _handle_setup(), consumed on DATA OUT. */
    volatile uint8_t     ep0_out_is_hid;

    /* CDC line coding (host-set baud rate etc.) */
    cdc_line_coding_t    line_coding;

    /* TX state */
    volatile uint8_t     tx_busy;           /* an EP1 IN packet is armed */
    volatile uint8_t     tx_stalled;        /* a write timed out; don't wait again until
                                             * the host collects a packet */
    volatile uint8_t     tx_hold;           /* port just opened: settling, see below */
    uint16_t             hold_fn;           /* USB frame number when DTR rose */

    /* Bus state */
    volatile uint8_t     started;           /* hal_usb_cdc_init() has run */
    volatile uint8_t     suspended;         /* no SOF for 3 ms: host asleep or gone */

    /* RX flow control: EP1 OUT holds a packet the ring had no room for (NAK) */
    volatile uint8_t     rx_held;

    /* RX callback (optional) */
    hal_usb_cdc_rx_cb_t  rx_cb;
    void                *rx_cb_ctx;

    /* RX ring buffer (used when no callback is set) */
    hal_ringbuf_t        rx_ring;
    uint8_t              rx_buf[HAL_USB_CDC_RX_BUF_SIZE];

    /* HID state */
    volatile uint8_t     hid_tx_busy;
    volatile uint8_t     hid_stalled;       /* a report timed out; don't wait again */
    uint8_t              hid_idle_rate;
    hal_usb_hid_rx_cb_t  hid_rx_cb;       /* HID OUT report sink (EP3 OUT + SET_REPORT) */
    void                *hid_rx_cb_ctx;

    /* Endpoint GET_STATUS reply (halt bit) */
    uint8_t              ep_status[2];

    /* Serial number string (UID hex, or CORE_USB_SERIAL) */
    char                 serial[26];
} _cdc;

/* ============================================================
 * EP0 control transfer helpers
 * ============================================================ */

static void _ep0_tx_packet(void)
{
    uint16_t len = _cdc.ep0_tx_remain;
    if (len > EP0_MAX_PACKET) len = EP0_MAX_PACKET;

    ll_usb_pma_write(PMA_EP0_TX, _cdc.ep0_tx_ptr, len);
    ll_usb_bdt_set_tx_count(0, len);

    _cdc.ep0_tx_ptr += len;
    _cdc.ep0_tx_remain -= len;

    ll_usb_ep_set_stat_tx(0, USB_EP_STAT_VALID);
}

static void _ep0_send(const uint8_t *data, uint16_t len, uint16_t max_len)
{
    if (len > max_len) len = max_len;
    _cdc.ep0_tx_ptr = data;
    _cdc.ep0_tx_remain = len;
    _cdc.ep0_state = EP0_DATA_IN;
    _ep0_tx_packet();
}

static void _ep0_send_status(void)
{
    ll_usb_bdt_set_tx_count(0, 0);
    _cdc.ep0_state = EP0_STATUS_IN;
    ll_usb_ep_set_stat_tx(0, USB_EP_STAT_VALID);
}

static void _ep0_stall(void)
{
    ll_usb_ep_set_stat(0, USB_EP_STAT_STALL, USB_EP_STAT_STALL);
    _cdc.ep0_state = EP0_IDLE;
}

/* Defined with the TX queue below. */
static void _txq_send_next(void);

/* ENDPOINT_HALT on a data endpoint (USB 2.0 §9.4.1, §9.4.5). SET stalls it.
 * CLEAR resets its data toggle to DATA0 whether or not it was halted (the
 * host resets its own toggle on every CLEAR_FEATURE(HALT), so a device that
 * doesn't loses the next packet as a "duplicate"), then un-stalls it. */
static void _ep_halt(uint16_t windex, int halt)
{
    uint8_t ep = windex & 0x0F;
    if (ep == 0 || ep > 3) return;          /* EP0 un-stalls on the next SETUP */

    if (windex & 0x80) {                    /* IN */
        if (halt) {
            ll_usb_ep_set_stat_tx(ep, USB_EP_STAT_STALL);
            if (ep == 1) _cdc.tx_busy = 1;  /* hold the queue while halted */
            if (ep == 3) _cdc.hid_tx_busy = 1;
            return;
        }
        ll_usb_ep_clr_dtog_tx(ep);
        if (ll_usb_ep_stat_tx(ep) == USB_EP_STAT_STALL) {
            ll_usb_ep_set_stat_tx(ep, USB_EP_STAT_NAK);
            if (ep == 1) { _cdc.tx_busy = 0; _txq_send_next(); }
            if (ep == 3) _cdc.hid_tx_busy = 0;
        }
    } else {                                /* OUT */
        if (halt) {
            ll_usb_ep_set_stat_rx(ep, USB_EP_STAT_STALL);
            return;
        }
        ll_usb_ep_clr_dtog_rx(ep);
        if (ll_usb_ep_stat_rx(ep) == USB_EP_STAT_STALL)
            ll_usb_ep_set_stat_rx(ep, (ep == 1 && _cdc.rx_held) ? USB_EP_STAT_NAK
                                                                 : USB_EP_STAT_VALID);
    }
}

/* ============================================================
 * Setup packet handler
 * ============================================================ */

static void _handle_setup(void)
{
    usb_setup_t setup;
    uint8_t buf[8];

    ll_usb_pma_read(PMA_EP0_RX, buf, 8);
    setup.bmRequestType = buf[0];
    setup.bRequest      = buf[1];
    setup.wValue        = buf[2] | (buf[3] << 8);
    setup.wIndex        = buf[4] | (buf[5] << 8);
    setup.wLength       = buf[6] | (buf[7] << 8);

    uint8_t type    = setup.bmRequestType & 0x60;  /* Type field [6:5] */
    uint8_t dir_in  = setup.bmRequestType & 0x80;

    /* ---- Standard requests ---- */
    if (type == 0x00) {
        switch (setup.bRequest) {

        case USB_REQ_GET_DESCRIPTOR: {
            uint8_t desc_type = setup.wValue >> 8;
            uint8_t desc_idx  = setup.wValue & 0xFF;

            switch (desc_type) {
            case USB_DESC_DEVICE:
                _ep0_send(dev_desc, sizeof(dev_desc), setup.wLength);
                return;

            case USB_DESC_CONFIGURATION:
                _ep0_send(cfg_desc, sizeof(cfg_desc), setup.wLength);
                return;

            case USB_DESC_STRING: {
                static uint8_t str_buf[64];
                uint8_t len;
                switch (desc_idx) {
                case 0:
                    _ep0_send(str0_desc, sizeof(str0_desc), setup.wLength);
                    return;
                case 1:
                    len = _str_to_desc(CORE_USB_MANUFACTURER, str_buf, sizeof(str_buf));
                    _ep0_send(str_buf, len, setup.wLength);
                    return;
                case 2:
                    len = _str_to_desc(CORE_USB_PRODUCT, str_buf, sizeof(str_buf));
                    _ep0_send(str_buf, len, setup.wLength);
                    return;
                case 3:
                    len = _str_to_desc(_cdc.serial, str_buf, sizeof(str_buf));
                    _ep0_send(str_buf, len, setup.wLength);
                    return;
                }
                break;
            }

            case USB_DESC_DEVICE_QUALIFIER:
                /* Full-speed only — STALL qualifier requests */
                _ep0_stall();
                return;

            case USB_DESC_HID:
                /* HID class descriptor (9 bytes within cfg_desc) */
                if (setup.wIndex == 2) {  /* Interface 2 = HID */
                    _ep0_send(cfg_desc + CFG_DESC_HID_OFFSET, 9, setup.wLength);
                    return;
                }
                break;

            case USB_DESC_HID_REPORT:
                if (setup.wIndex == 2) {
                    _ep0_send(hid_report_desc, sizeof(hid_report_desc), setup.wLength);
                    return;
                }
                break;
            }
            break;
        }

        case USB_REQ_SET_ADDRESS:
            _cdc.address_pending = setup.wValue & 0x7F;
            _ep0_send_status();
            return;

        case USB_REQ_SET_CONFIGURATION:
            if (setup.wValue == 1) {
                _cdc.state = USB_STATE_CONFIGURED;
                _cdc.configured = 1;

                /* Configure EP1 (CDC bulk data) */
                ll_usb_ep_config(1, USB_EP_BULK, 1);
                ll_usb_bdt_set_tx_addr(1, PMA_EP1_TX);
                ll_usb_bdt_set_tx_count(1, 0);
                ll_usb_bdt_set_rx_addr(1, PMA_EP1_RX);
                ll_usb_bdt_set_rx_count(1, EP1_MAX_PACKET);
                ll_usb_ep_set_stat(1, USB_EP_STAT_NAK, USB_EP_STAT_VALID);

                /* Configure EP2 (CDC interrupt notification) */
                ll_usb_ep_config(2, USB_EP_INTERRUPT, 2);
                ll_usb_bdt_set_tx_addr(2, PMA_EP2_TX);
                ll_usb_bdt_set_tx_count(2, 0);
                ll_usb_ep_set_stat_tx(2, USB_EP_STAT_NAK);

                /* Configure EP3 (HID interrupt, bidirectional):
                 * IN  = report device -> host, armed on demand (NAK until sent)
                 * OUT = report host -> device, armed VALID to receive */
                ll_usb_ep_config(3, USB_EP_INTERRUPT, 3);
                ll_usb_bdt_set_tx_addr(3, PMA_EP3_TX);
                ll_usb_bdt_set_tx_count(3, 0);
                ll_usb_bdt_set_rx_addr(3, PMA_EP3_RX);
                ll_usb_bdt_set_rx_count(3, EP3_MAX_PACKET);
                ll_usb_ep_set_stat(3, USB_EP_STAT_NAK, USB_EP_STAT_VALID);
                _cdc.hid_tx_busy = 0;
                _cdc.hid_stalled = 0;
                _cdc.tx_busy = 0;           /* EP1 reconfigured: nothing armed */
                _cdc.rx_held = 0;
                _txq_send_next();           /* a re-configure keeps DTR */
            } else {
                _cdc.state = USB_STATE_ADDRESS;
                _cdc.configured = 0;
            }
            _ep0_send_status();
            return;

        case USB_REQ_GET_CONFIGURATION: {
            static const uint8_t one = 1;
            static const uint8_t zero = 0;
            _ep0_send(_cdc.configured ? &one : &zero, 1, setup.wLength);
            return;
        }

        case USB_REQ_GET_STATUS: {
            _cdc.ep_status[0] = 0;
            _cdc.ep_status[1] = 0;
            if ((setup.bmRequestType & 0x1F) == 0x02) {     /* endpoint: bit 0 = halted */
                uint8_t ep = setup.wIndex & 0x0F;
                if (ep <= 3) {
                    uint16_t st = (setup.wIndex & 0x80) ? ll_usb_ep_stat_tx(ep)
                                                        : ll_usb_ep_stat_rx(ep);
                    _cdc.ep_status[0] = (st == USB_EP_STAT_STALL) ? 1 : 0;
                }
            }
            _ep0_send(_cdc.ep_status, 2, setup.wLength);
            return;
        }

        case USB_REQ_CLEAR_FEATURE:
        case USB_REQ_SET_FEATURE:
            if ((setup.bmRequestType & 0x1F) == 0x02 && setup.wValue == 0)  /* ENDPOINT_HALT */
                _ep_halt(setup.wIndex, setup.bRequest == USB_REQ_SET_FEATURE);
            _ep0_send_status();
            return;
        }
    }

    /* ---- Class requests — route by interface (wIndex) ---- */
    if (type == 0x20) {
        uint16_t iface = setup.wIndex;

        /* CDC class requests (interfaces 0 and 1) */
        if (iface <= 1) {
            switch (setup.bRequest) {

            case CDC_SET_LINE_CODING:
                /* Host sends 7 bytes of line coding data in DATA phase */
                _cdc.ep0_out_is_hid = 0;
                _cdc.ep0_state = EP0_DATA_OUT;
                ll_usb_ep_set_stat_rx(0, USB_EP_STAT_VALID);
                return;

            case CDC_GET_LINE_CODING:
                _ep0_send((const uint8_t *)&_cdc.line_coding, 7, setup.wLength);
                return;

            case CDC_SET_CONTROL_LINE_STATE: {
                uint8_t new_dtr = (setup.wValue & 0x01) ? 1 : 0;

                /* 1200-baud touch: DTR drop while baud=1200 triggers DFU reboot.
                 * This is the Arduino convention — host opens port at 1200 baud
                 * then closes it. make flash-dfu uses this to auto-enter DFU. */
                if (_cdc.dtr && !new_dtr && _cdc.line_coding.dwDTERate == 1200) {
                    _ep0_send_status();
                    /* Small delay so the ZLP status reaches the host */
                    for (volatile int i = 0; i < 100000; i++)
                        ;
                    hal_dfu_reboot();
                }

                /* A terminal (re)opening gets a fresh chance to read before
                 * writes are dropped, and what was written before it opened,
                 * after a short settle: pyserial (and Windows' PurgeComm)
                 * flush the input buffer right after raising DTR, and would
                 * throw away text sent the moment it rose. Timed on SOFs. */
                if (new_dtr && !_cdc.dtr) {
                    _cdc.tx_stalled = 0;
#if HAL_USB_CDC_OPEN_SETTLE_MS > 0
                    _cdc.tx_hold = 1;
                    _cdc.hold_fn = (uint16_t)(USB_FNR & 0x7FFu);
                    USB_CNTR |= USB_CNTR_SOFM;
#endif
                }
                if (!new_dtr) {
                    _cdc.tx_hold = 0;
                    USB_CNTR &= (uint16_t)~USB_CNTR_SOFM;
                }
                _cdc.dtr = new_dtr;
                _ep0_send_status();
                if (new_dtr) _txq_send_next();
                return;
            }

            case CDC_SEND_BREAK:
                _ep0_send_status();
                return;
            }
        }

        /* HID class requests (interface 2) */
        if (iface == 2) {
            switch (setup.bRequest) {

            case HID_SET_IDLE:
                _cdc.hid_idle_rate = (setup.wValue >> 8) & 0xFF;
                _ep0_send_status();
                return;

            case HID_GET_IDLE: {
                static uint8_t idle;
                idle = _cdc.hid_idle_rate;
                _ep0_send(&idle, 1, setup.wLength);
                return;
            }

            case HID_GET_REPORT:
                /* Not implemented — host should read via EP3 interrupt IN */
                _ep0_stall();
                return;

            case HID_SET_REPORT:
                /* Accept the report payload in the EP0 DATA OUT stage and
                 * route it to the HID OUT sink (alongside the EP3 OUT path). */
                _cdc.ep0_out_is_hid = 1;
                _cdc.ep0_state = EP0_DATA_OUT;
                ll_usb_ep_set_stat_rx(0, USB_EP_STAT_VALID);
                return;
            }
        }
    }

    /* Unknown request — STALL */
    if (dir_in) {
        _ep0_stall();
    } else {
        _ep0_send_status();
    }
}

/* ============================================================
 * USB reset handler
 * ============================================================ */

/* TX queue: every write goes through it, see the TX section below. */
#define TXQ_MASK  (HAL_USB_CDC_TX_QUEUE_SIZE - 1u)
_Static_assert((HAL_USB_CDC_TX_QUEUE_SIZE & TXQ_MASK) == 0u &&
               HAL_USB_CDC_TX_QUEUE_SIZE >= 64u && HAL_USB_CDC_TX_QUEUE_SIZE <= 32768u,
               "HAL_USB_CDC_TX_QUEUE_SIZE must be a power of 2, 64..32768");
static uint8_t           _txq[HAL_USB_CDC_TX_QUEUE_SIZE];
static volatile uint16_t _txq_head;   /* written by writers, IRQs masked */
static volatile uint16_t _txq_tail;   /* written by _txq_send_next        */

#ifdef HAL_USB_CDC_TEST_HOOKS
static volatile uint8_t  _test_force;           /* hal_usb_cdc_test_force_suspend() */
static volatile uint32_t _lp_stops, _lp_wakes;  /* Stops taken while suspended; WKUPs */
static volatile uint32_t _lp_log[16], _lp_log_n;  /* (ms << 8) | event, first 16 */
#  define _LP_COUNT(v) ((v)++)
#  define _LP_LOG(ev) do { if (_lp_log_n < 16u) \
        _lp_log[_lp_log_n++] = (_systick_ticks << 8) | (uint32_t)(ev); } while (0)
#else
#  define _LP_COUNT(v) ((void)0)
#  define _LP_LOG(ev)  ((void)0)
#endif

static void _handle_reset(void)
{
    /* Configure EP0 (control) */
    ll_usb_ep_config(0, USB_EP_CONTROL, 0);
    ll_usb_bdt_set_tx_addr(0, PMA_EP0_TX);
    ll_usb_bdt_set_tx_count(0, 0);
    ll_usb_bdt_set_rx_addr(0, PMA_EP0_RX);
    ll_usb_bdt_set_rx_count(0, EP0_MAX_PACKET);

    /* Enable EP0 RX */
    ll_usb_ep_set_stat(0, USB_EP_STAT_NAK, USB_EP_STAT_VALID);

    /* Set address 0 with function enabled */
    ll_usb_set_address(0);

    _cdc.state = USB_STATE_DEFAULT;
    _cdc.configured = 0;
    _cdc.address_pending = 0;
    _cdc.dtr = 0;
    _cdc.tx_busy = 0;               /* the armed packet (if any) is gone */
    _cdc.tx_stalled = 0;
    _cdc.tx_hold = 0;               /* SOFM is off in the CNTR write below */
    _cdc.rx_held = 0;
    _cdc.hid_tx_busy = 0;
    _cdc.hid_stalled = 0;
    _cdc.suspended = 0;
    _cdc.ep0_state = EP0_IDLE;
    /* The TX queue is kept: text written before enumeration (or before a
     * host-side reset) still reaches the next terminal. */

    /* Enable interrupts: CTR + RESET + SUSPEND + WAKEUP. Also clears FSUSP and
     * LP_MODE, which a reset out of suspend leaves behind. */
    USB_CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
}

#if HAL_SERIAL_UPDATE
/* ============================================================
 * Serial update handoff
 *
 * Called from the USB interrupt once the host has set the trigger line coding
 * and the status stage is armed. Stops everything else that could touch SRAM,
 * leaves the USB peripheral exactly as it is, copies the flasher to
 * SU_FLASHER_BASE and jumps to it. The app does not come back: the flasher
 * resets into the new image (or the old one, if nothing was written).
 * ============================================================ */

static void _serial_update_enter(void)
{
    /* The handoff block and flasher overwrite low SRAM. Refuse (and stay in
     * the app) if this interrupt's stack has grown down into that range. */
    uint32_t msp;
    __asm volatile ("mrs %0, msp" : "=r" (msp));
    if (msp < SU_HANDOFF_ADDR + 0x400UL) return;

    __asm volatile ("cpsid i" ::: "memory");
    REG32(0xE000E010UL) = 0;                  /* SysTick off */
    REG32(0xE000ED94UL) = 0;                  /* MPU off */
    REG32(0x40021028UL) |= 0x3UL;             /* RCC_AHB1RSTR: reset DMA1 + DMA2 so */
    REG32(0x40021028UL) &= ~0x3UL;            /* no transfer lands in the flasher */
    REG32(0x40003000UL) = 0xAAAAUL;           /* feed the IWDG */

    /* An OUT packet held back for lack of ring space belongs to the old
     * session: drop it, so the flasher's first EP1 read is the host's. */
    if (_cdc.rx_held) ll_usb_ep_set_stat_rx(1, USB_EP_STAT_VALID);

    /* Read our own state before anything is overwritten: _cdc lives in SRAM. */
    uint8_t lc[7];
    memcpy(lc, &_cdc.line_coding, 7);
    uint8_t configured = _cdc.configured;

    su_handoff_t *h = (su_handoff_t *)SU_HANDOFF_ADDR;
    h->magic               = SU_HANDOFF_MAGIC;
    h->dev_desc            = dev_desc;
    h->cfg_desc            = cfg_desc;
    h->hid_report_desc     = hid_report_desc;
    h->strings[0]          = CORE_USB_MANUFACTURER;
    h->strings[1]          = CORE_USB_PRODUCT;
    /* The serial is built in SRAM, which the flasher copy overwrites: pass a
     * copy inside the handoff block, which it doesn't. */
    for (unsigned i = 0; i < sizeof(h->serial); i++)
        h->serial[i] = (i + 1u < sizeof(h->serial)) ? _cdc.serial[i] : '\0';
    h->strings[2]          = h->serial;
    h->dev_desc_len        = sizeof(dev_desc);
    h->cfg_desc_len        = sizeof(cfg_desc);
    h->cfg_hid_offset      = CFG_DESC_HID_OFFSET;
    h->hid_report_desc_len = sizeof(hid_report_desc);
    for (int i = 0; i < 7; i++) h->line_coding[i] = lc[i];
    h->configured          = configured;

    const uint32_t *src = (const uint32_t *)(const void *)su_flasher_l4_bytes;
    uint32_t       *dst = (uint32_t *)SU_FLASHER_BASE;
    for (uint32_t i = 0; i < (SU_FLASHER_L4_LEN + 3u) / 4u; i++) dst[i] = src[i];

    __asm volatile ("dsb\n isb" ::: "memory");
    uint32_t sp = ((const uint32_t *)SU_FLASHER_BASE)[0];
    uint32_t pc = ((const uint32_t *)SU_FLASHER_BASE)[1];
    __asm volatile (
        "msr msp, %0 \n"
        "bx  %1      \n"
        :
        : "r" (sp), "r" (pc)
        : "memory"
    );
    __builtin_unreachable();
}
#endif /* HAL_SERIAL_UPDATE */

/* ============================================================
 * RX flow control
 *
 * After a correct OUT transfer the hardware sets EP1's STAT_RX to NAK, so the
 * host retries (without loss) until the endpoint is re-armed. With no RX
 * callback, the packet is copied into the ring only when it fits whole; if
 * it doesn't, the endpoint stays NAK (rx_held) and the next read that frees
 * room pulls it in. A host burst larger than the ring is paced, never lost.
 * ============================================================ */

/* ISR, or thread mode with interrupts masked. */
static void _rx_pull(void)
{
    uint16_t count = ll_usb_bdt_get_rx_count(1);
    if (count > EP1_MAX_PACKET) count = EP1_MAX_PACKET;

    if (!_cdc.rx_cb) {
        uint16_t space = (uint16_t)(HAL_USB_CDC_RX_BUF_SIZE - 1u
                                    - hal_ringbuf_count(&_cdc.rx_ring));
        if (count > space) {
            _cdc.rx_held = 1;               /* stay NAK: the host retries */
            return;
        }
    }

    uint8_t tmp[EP1_MAX_PACKET];
    ll_usb_pma_read(PMA_EP1_RX, tmp, count);
    _cdc.rx_held = 0;
    ll_usb_ep_set_stat_rx(1, USB_EP_STAT_VALID);   /* the PMA copy is done */

    if (_cdc.rx_cb) {
        if (count) _cdc.rx_cb(tmp, count, _cdc.rx_cb_ctx);
    } else {
        for (uint16_t i = 0; i < count; i++)
            (void)hal_ringbuf_put(&_cdc.rx_ring, tmp[i]);   /* room checked above */
    }
}

/* Thread mode, after a read freed ring space. */
static void _rx_release(void)
{
    if (!_cdc.rx_held) return;
    uint32_t irq = ll_irq_save();
    if (_cdc.rx_held) _rx_pull();
    ll_irq_restore(irq);
}

/* ============================================================
 * Correct Transfer (CTR) handler
 * ============================================================ */

static void _handle_ctr(void)
{
    uint16_t istr = USB_ISTR;
    uint8_t  ep   = istr & USB_ISTR_EP_ID_MASK;

    if (ep == 0) {
        uint16_t ep0r = ll_usb_ep_read(0);

        /* ---- EP0 RX (OUT/SETUP from host) ---- */
        if (ep0r & USB_EP_CTR_RX) {
            ll_usb_ep_clr_ctr_rx(0);

            if (ep0r & USB_EP_SETUP) {
                _handle_setup();
            } else {
                /* DATA OUT phase */
                if (_cdc.ep0_state == EP0_DATA_OUT) {
                    uint16_t count = ll_usb_bdt_get_rx_count(0);
                    if (count > sizeof(_cdc.ep0_rx_buf))
                        count = sizeof(_cdc.ep0_rx_buf);
                    ll_usb_pma_read(PMA_EP0_RX, _cdc.ep0_rx_buf, count);

                    if (_cdc.ep0_out_is_hid) {
                        /* HID SET_REPORT payload — deliver to the OUT sink */
                        if (_cdc.hid_rx_cb)
                            _cdc.hid_rx_cb(_cdc.ep0_rx_buf, count, _cdc.hid_rx_cb_ctx);
                        _cdc.ep0_out_is_hid = 0;
                    } else if (count == 7) {
                        /* CDC SET_LINE_CODING — copy the 7 bytes */
                        memcpy(&_cdc.line_coding, _cdc.ep0_rx_buf, 7);
#if HAL_SERIAL_UPDATE
                        if (_cdc.line_coding.dwDTERate == SU_TRIGGER_BAUD) {
                            _ep0_send_status();   /* the flasher completes this stage */
                            _serial_update_enter();
                            return;               /* only if the handoff was refused */
                        }
#endif
                    }
                    _ep0_send_status();
                } else if (_cdc.ep0_state == EP0_STATUS_OUT) {
                    _cdc.ep0_state = EP0_IDLE;
                    ll_usb_ep_set_stat_rx(0, USB_EP_STAT_VALID);
                } else {
                    /* Unexpected OUT — re-arm RX */
                    ll_usb_ep_set_stat_rx(0, USB_EP_STAT_VALID);
                }
            }
        }

        /* ---- EP0 TX (IN to host) ---- */
        if (ep0r & USB_EP_CTR_TX) {
            ll_usb_ep_clr_ctr_tx(0);

            if (_cdc.ep0_state == EP0_DATA_IN) {
                if (_cdc.ep0_tx_remain > 0) {
                    _ep0_tx_packet();
                } else {
                    /* All data sent — wait for STATUS OUT from host */
                    _cdc.ep0_state = EP0_STATUS_OUT;
                    ll_usb_ep_set_stat_rx(0, USB_EP_STAT_VALID);
                }
            } else if (_cdc.ep0_state == EP0_STATUS_IN) {
                /* Status phase complete */
                if (_cdc.address_pending) {
                    ll_usb_set_address(_cdc.address_pending);
                    _cdc.state = USB_STATE_ADDRESS;
                    _cdc.address_pending = 0;
                }
                _cdc.ep0_state = EP0_IDLE;
                ll_usb_ep_set_stat_rx(0, USB_EP_STAT_VALID);
            }
        }
    }

    else if (ep == 1) {
        uint16_t ep1r = ll_usb_ep_read(1);

        /* ---- EP1 RX (bulk OUT — host -> device data) ---- */
        if (ep1r & USB_EP_CTR_RX) {
            ll_usb_ep_clr_ctr_rx(1);
            _rx_pull();                     /* re-arms only if the data fit */
        }

        /* ---- EP1 TX (bulk IN — device -> host complete) ---- */
        if (ep1r & USB_EP_CTR_TX) {
            ll_usb_ep_clr_ctr_tx(1);
            _cdc.tx_busy = 0;
            _cdc.tx_stalled = 0;            /* the host is reading again */
            _txq_send_next();
        }
    }

    else if (ep == 3) {
        uint16_t ep3r = ll_usb_ep_read(3);

        /* ---- EP3 RX (HID OUT report — host -> device) ---- */
        if (ep3r & USB_EP_CTR_RX) {
            ll_usb_ep_clr_ctr_rx(3);

            uint16_t count = ll_usb_bdt_get_rx_count(3);
            if (count > EP3_MAX_PACKET) count = EP3_MAX_PACKET;
            if (count > 0 && _cdc.hid_rx_cb) {
                uint8_t tmp[EP3_MAX_PACKET];
                ll_usb_pma_read(PMA_EP3_RX, tmp, count);
                _cdc.hid_rx_cb(tmp, count, _cdc.hid_rx_cb_ctx);
            }
            /* Re-arm RX for the next report */
            ll_usb_ep_set_stat_rx(3, USB_EP_STAT_VALID);
        }

        /* ---- EP3 TX (HID report sent) ---- */
        if (ep3r & USB_EP_CTR_TX) {
            ll_usb_ep_clr_ctr_tx(3);
            _cdc.hid_tx_busy = 0;
            _cdc.hid_stalled = 0;
        }
    }

    else {
        /* EP2 (notifications, never armed) or a stray: just acknowledge,
         * or ISTR.CTR stays set and the interrupt never ends. */
        uint16_t r = ll_usb_ep_read(ep);
        if (r & USB_EP_CTR_RX) ll_usb_ep_clr_ctr_rx(ep);
        if (r & USB_EP_CTR_TX) ll_usb_ep_clr_ctr_tx(ep);
    }
}

/* ============================================================
 * USB Interrupt Handler (vector 67: USB FS, also the wakeup from Stop
 * through EXTI line 17, RM0394 Table 51 / Table 52)
 * ============================================================ */

void USB_IRQHandler(void)
{
    uint16_t istr = USB_ISTR;

    if (istr & USB_ISTR_RESET) {
        USB_ISTR = (uint16_t)~USB_ISTR_RESET;
        _handle_reset();
        return;
    }

    /* CTR is cleared through each endpoint's CTR_RX/CTR_TX; ISTR.CTR is
     * read-only and stays set while any endpoint has one pending. */
    for (int n = 0; n < 8 && (USB_ISTR & USB_ISTR_CTR); n++)
        _handle_ctr();

    /* Start of frame: only enabled while a newly opened port settles. */
    if (istr & USB_ISTR_SOF) {
        USB_ISTR = (uint16_t)~USB_ISTR_SOF;
        _txq_send_next();                   /* ends the hold once it has run */
    }

    /* Suspend (RM0394 §46.5.5): no SOF for 3 ms. The host is asleep, has
     * suspended the port, or is gone. FSUSP stops the SOF check; LP_MODE is
     * set only on the way into Stop (hal_usb_cdc_stop_allowed). */
    if (istr & USB_ISTR_SUSP) {
        USB_ISTR = (uint16_t)~USB_ISTR_SUSP;
        USB_CNTR |= USB_CNTR_FSUSP;
        _cdc.suspended = 1;
        _LP_LOG('S' | ((istr & USB_ISTR_WKUP) ? 0x80u : 0u));
    }

    /* Resume or reset signalling. The hardware clears LP_MODE when it sees
     * the event, but not if software set it after; clear both. Noise that
     * woke us leaves no SOFs behind, so SUSP fires again 3 ms later. */
    if (istr & USB_ISTR_WKUP) {
        USB_ISTR = (uint16_t)~USB_ISTR_WKUP;
        USB_CNTR &= (uint16_t)~(USB_CNTR_FSUSP | USB_CNTR_LP_MODE);
        _cdc.suspended = 0;
        _LP_COUNT(_lp_wakes);
        _LP_LOG('W');
        _txq_send_next();
    }
}

/* ============================================================
 * Public API
 * ============================================================ */

void hal_usb_cdc_init(void)
{
    /* core_init() already starts USB on every L4 project; a second call (an
     * explicit core_usb_init()) used to wipe this state and reset the
     * peripheral under a host that may have enumerated it. Once is enough. */
    if (_cdc.started) return;

    /* Initialize state */
    memset(&_cdc, 0, sizeof(_cdc));
    hal_ringbuf_init(&_cdc.rx_ring, _cdc.rx_buf, HAL_USB_CDC_RX_BUF_SIZE);

    /* Default line coding: 115200 8N1 */
    _cdc.line_coding.dwDTERate   = 115200;
    _cdc.line_coding.bCharFormat = 0;   /* 1 stop bit */
    _cdc.line_coding.bParityType = 0;   /* None */
    _cdc.line_coding.bDataBits   = 8;

    /* Serial number: the factory UID unless config.json fixed one. */
#if HAL_USB_SERIAL_FIXED
    strncpy(_cdc.serial, CORE_USB_SERIAL, sizeof(_cdc.serial) - 1u);
#else
    (void)core_uid_hex(_cdc.serial, (uint16_t)sizeof(_cdc.serial));
#endif

    /* Enable HSI48 for crystal-less USB */
    ll_rcc_hsi48_enable();
    while (!ll_rcc_hsi48_ready())
        ;

    /* Select HSI48 as USB clock source */
    ll_rcc_set_usb_clk_source(LL_RCC_USB48_HSI48);

    /* Enable CRS clock and sync to USB SOF */
    ll_rcc_apb1_clk_enable(LL_APB1_CRS);
    ll_crs_usb_sync_enable();

    /* Enable USB peripheral clock */
    ll_rcc_apb1_clk_enable(LL_APB1_USB);

    /* Enable VDDUSB power supply.
     * On STM32L4, USB power is isolated by default. The USV bit in
     * PWR_CR2 must be set to remove isolation before USB can work. */
    {
        /* Enable PWR peripheral clock (APB1 bit 28) */
        ll_rcc_apb1_clk_enable(1UL << 28);
        /* PWR_CR2 at PWR_BASE + 0x04, USV = bit 10 */
        #define PWR_BASE_ADDR  0x40007000UL
        SET_BITS(REG32(PWR_BASE_ADDR + 0x04UL), (1UL << 10));
    }

    /* Configure USB pins: PA11 (DM) and PA12 (DP) as AF10 */
    ll_rcc_gpio_clk_enable(GPIOA);
    ll_gpio_config_af(GPIOA, 11, 10, LL_GPIO_OTYPE_PP, LL_GPIO_SPEED_VHIGH, LL_GPIO_PULL_NONE);
    ll_gpio_config_af(GPIOA, 12, 10, LL_GPIO_OTYPE_PP, LL_GPIO_SPEED_VHIGH, LL_GPIO_PULL_NONE);

    /* Power on USB peripheral */
    ll_usb_power_on();

    /* Enable USB interrupt */
    hal_nvic_set_priority(HAL_IRQ_USB, 3);   /* level 3 (the helper shifts it) */
    hal_nvic_enable_irq(HAL_IRQ_USB);

    /* Initial interrupt mask: RESET, plus SUSP/WKUP so a Core with no host
     * (on battery, or a charger) sees the idle bus as suspended and may use
     * Stop. _handle_reset() adds CTR. */
    USB_CNTR = USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
    _cdc.started = 1;

    /* Connect DP pull-up — host will see us */
    ll_usb_connect();
}

int hal_usb_cdc_connected(void)
{
    return _cdc.configured && _cdc.dtr;
}

int hal_usb_cdc_suspended(void)
{
    return _cdc.started && _cdc.suspended;
}

void hal_usb_cdc_set_rx_callback(hal_usb_cdc_rx_cb_t cb, void *ctx)
{
    uint32_t irq = ll_irq_save();
    _cdc.rx_cb     = cb;
    _cdc.rx_cb_ctx = ctx;
    if (_cdc.rx_held) _rx_pull();       /* a callback takes any held packet */
    ll_irq_restore(irq);
}

/* ---- Low power (core_power.h) ---- */

#ifdef HAL_USB_CDC_TEST_HOOKS
/* Bench-only (tests/hw-usb-robust): Stops taken with the bus suspended, and
 * USB wakeups (WKUP interrupts), since boot. */
/* Take the next Stop as if the bus were suspended (FSUSP + LP_MODE). With a
 * live host the SOF traffic only sometimes looks like resume to the wake
 * detector, so this proves the wake chain (EXTI 17, clocks, WKUP), not the
 * host-resume trigger, and it must not be used with the port open. */
void hal_usb_cdc_test_force_suspend(void) { _test_force = 1; }

uint32_t hal_usb_cdc_test_log(const uint32_t **log)
{
    *log = (const uint32_t *)_lp_log;
    return _lp_log_n;
}

void hal_usb_cdc_test_stats(uint32_t *stops, uint32_t *wakes)
{
    *stops = _lp_stops;
    *wakes = _lp_wakes;
}
#endif

int hal_usb_cdc_started(void)
{
    return _cdc.started;
}

int hal_usb_cdc_stop_allowed(void)
{
    if (!_cdc.started) return 1;
#ifdef HAL_USB_CDC_TEST_HOOKS
    if (_test_force) {
        _test_force = 0;
        USB_CNTR |= USB_CNTR_FSUSP;
        _cdc.suspended = 1;
        USB_ISTR = (uint16_t)~USB_ISTR_WKUP;
    }
#endif
    /* A host that has the bus awake can send a token at any time; in Stop
     * HSI48 is off (RCC_CRRCR.HSI48ON is cleared on entry, RM0394 §6.4.30) and
     * the device would stop answering. Wait in Sleep instead. */
    if (!_cdc.suspended) return 0;
    if (USB_ISTR & (USB_ISTR_WKUP | USB_ISTR_RESET)) {           /* ISR pending */
        _LP_LOG('P');
        return 0;
    }

    /* Suspended (RM0394 §46.5.5): transceivers to low power, still able to
     * see resume. A host resume or reset then clears LP_MODE and raises the
     * USB wakeup on EXTI line 17 (a direct line, unmasked at reset, wakes
     * Stop 0 and Stop 1 but not Stop 2: Table 52), which pends the USB
     * interrupt and ends Stop. */
    SET_BITS(REG32(EXTI_BASE + 0x00UL), (1UL << 17));   /* EXTI_IMR1.IM17 */
    USB_CNTR |= USB_CNTR_LP_MODE;
    _LP_COUNT(_lp_stops);
    _LP_LOG('T');
    return 1;
}

/* ---- TX ----
 *
 * Every write goes through one FIFO (_txq) that the USB interrupt drains a
 * packet at a time, so bytes reach the host in call order and a buffer queued
 * by hal_usb_cdc_try_write() (a Scope frame) is never split by print text
 * written from the same context.
 *
 * Packets are capped one byte short of full: a transfer ending on a full
 * 64-byte packet is not complete until a zero-length packet follows, and hosts
 * hold the data until then. No packet is ever full, so no ZLP is needed.
 *
 * Nothing leaves the FIFO until a terminal is connected (configured + DTR):
 * text written before that (a banner printed right after core_init()) waits
 * and goes out when the port opens. When the FIFO is full:
 *   - no terminal: a write that doesn't fit whole is dropped whole
 *     (newest-drop, so the oldest text, the boot banner, survives, and the
 *     backlog never ends in a torn line);
 *   - the bus is suspended: what doesn't fit is dropped at once;
 *   - a terminal that is reading: the writer waits for room;
 *   - a terminal that stopped reading: the writer waits at most
 *     HAL_USB_CDC_TX_TIMEOUT_MS without the host collecting a packet, then
 *     drops, and later writes drop at once until the host reads again.
 * Writes from an interrupt handler, or with interrupts masked, never wait. */

/* May this caller wait? Thread mode with interrupts on (so the USB interrupt
 * can drain and SysTick can time the wait). */
static int _usb_can_wait(void)
{
    uint32_t ipsr, primask, basepri;
    __asm volatile ("mrs %0, ipsr"    : "=r" (ipsr));
    __asm volatile ("mrs %0, primask" : "=r" (primask));
    __asm volatile ("mrs %0, basepri" : "=r" (basepri));
    return ipsr == 0 && (primask & 1u) == 0 && basepri == 0 &&
           (SYSTICK_CSR & (SYSTICK_CSR_ENABLE | SYSTICK_CSR_TICKINT)) ==
               (SYSTICK_CSR_ENABLE | SYSTICK_CSR_TICKINT);
}

/* Arm the next packet if EP1 IN is free and a terminal is there. ISR, or
 * thread mode with interrupts masked. */
static void _txq_send_next(void)
{
    if (_cdc.tx_busy || !_cdc.configured || !_cdc.dtr) return;
    if (_cdc.tx_hold) {
        if (((USB_FNR - _cdc.hold_fn) & 0x7FFu) < HAL_USB_CDC_OPEN_SETTLE_MS) return;
        _cdc.tx_hold = 0;
        USB_CNTR &= (uint16_t)~USB_CNTR_SOFM;
    }
    uint16_t n = (uint16_t)(_txq_head - _txq_tail);
    if (n == 0) return;
    if (n > EP1_MAX_PACKET - 1) n = EP1_MAX_PACKET - 1;

    uint8_t pkt[EP1_MAX_PACKET];
    for (uint16_t i = 0; i < n; i++) {
        pkt[i] = _txq[(uint16_t)(_txq_tail + i) & TXQ_MASK];
    }
    _txq_tail = (uint16_t)(_txq_tail + n);

    ll_usb_pma_write(PMA_EP1_TX, pkt, n);
    ll_usb_bdt_set_tx_count(1, n);
    _cdc.tx_busy = 1;
    ll_usb_ep_set_stat_tx(1, USB_EP_STAT_VALID);
}

/* Copy up to `len` bytes into the FIFO; returns how many fit. IRQs masked. */
static uint16_t _txq_put(const uint8_t *buf, uint16_t len)
{
    uint16_t room = (uint16_t)(HAL_USB_CDC_TX_QUEUE_SIZE - (uint16_t)(_txq_head - _txq_tail));
    if (len > room) len = room;
    uint16_t head = _txq_head;
    for (uint16_t i = 0; i < len; i++) {
        _txq[(uint16_t)(head + i) & TXQ_MASK] = buf[i];
    }
    _txq_head = (uint16_t)(head + len);
    return len;
}

int hal_usb_cdc_try_write(const uint8_t *buf, uint16_t len)
{
    if (!_cdc.configured || !_cdc.dtr) return -1;
    if (len == 0) return 0;
    if (len > HAL_USB_CDC_TX_QUEUE_SIZE) return 0;

    int ok = 0;
    uint32_t irq = ll_irq_save();
    uint16_t used = (uint16_t)(_txq_head - _txq_tail);
    if (len <= (uint16_t)(HAL_USB_CDC_TX_QUEUE_SIZE - used)) {
        (void)_txq_put(buf, len);
        ok = 1;
    }
    _txq_send_next();
    ll_irq_restore(irq);
    return ok ? (int)len : 0;
}

int hal_usb_cdc_write(const uint8_t *buf, uint16_t len)
{
    if (!_cdc.started) return -1;

    /* No terminal: the write is kept whole or dropped whole, so the text
     * waiting for the port never ends in a torn line. */
    if (!_cdc.configured || !_cdc.dtr) {
        uint32_t irq = ll_irq_save();
        uint16_t room = (uint16_t)(HAL_USB_CDC_TX_QUEUE_SIZE
                                   - (uint16_t)(_txq_head - _txq_tail));
        uint16_t n = (len <= room) ? _txq_put(buf, len) : 0;
        ll_irq_restore(irq);
        return (int)n;
    }

    int      can_wait  = _usb_can_wait();
    uint16_t done      = 0;
    uint16_t seen_tail = _txq_tail;
    uint32_t since     = can_wait ? _systick_ticks : 0;

    while (done < len) {
        /* A packet's worth at a time, so interrupts are masked only briefly
         * (and an interrupt's own print can slip in between, as on a UART). */
        uint16_t n = (uint16_t)(len - done);
        if (n > EP1_MAX_PACKET) n = EP1_MAX_PACKET;
        uint32_t irq = ll_irq_save();
        n = _txq_put(buf + done, n);
        _txq_send_next();
        ll_irq_restore(irq);
        done = (uint16_t)(done + n);
        if (n) continue;

        /* FIFO full. Wait only for a terminal that is there and reading. */
        if (!can_wait || !_cdc.configured || !_cdc.dtr || _cdc.suspended || _cdc.tx_stalled)
            break;
        uint16_t tail = _txq_tail;
        if (tail != seen_tail || _cdc.tx_hold) {   /* the settle isn't a stall */
            seen_tail = tail;               /* the host took a packet */
            since = _systick_ticks;
        } else if ((uint32_t)(_systick_ticks - since) >= HAL_USB_CDC_TX_TIMEOUT_MS) {
            _cdc.tx_stalled = 1;            /* cleared when it reads again */
            break;
        }
    }
    return (int)done;
}

int hal_usb_cdc_printf(const char *fmt, ...)
{
    char buf[HAL_USB_CDC_PRINTF_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n > 0) {
        /* vsnprintf returns the length the text WOULD have had; a message
         * longer than the buffer was truncated. Send only what was formatted:
         * sending `n` bytes reads past the stack buffer (bus fault at the top
         * of RAM, seen 2026-09-08 with a ~450-byte help text). */
        if ((size_t)n > sizeof(buf) - 1) n = (int)(sizeof(buf) - 1);
        hal_usb_cdc_write((const uint8_t *)buf, (uint16_t)n);
    }
    return n;
}

/* ---- RX (ring buffer mode) ---- */

int hal_usb_cdc_rx_ready(void)
{
    return !hal_ringbuf_empty(&_cdc.rx_ring);
}

uint8_t hal_usb_cdc_getc(void)
{
    uint8_t byte;
    while (!hal_ringbuf_get(&_cdc.rx_ring, &byte))
        _rx_release();
    _rx_release();
    return byte;
}

int hal_usb_cdc_rx_try(uint8_t *byte)
{
    int got = hal_ringbuf_get(&_cdc.rx_ring, byte);
    _rx_release();
    return got;
}

uint16_t hal_usb_cdc_read(uint8_t *buf, uint16_t max_len)
{
    uint16_t n = hal_ringbuf_read(&_cdc.rx_ring, buf, max_len);
    _rx_release();
    return n;
}

uint16_t hal_usb_cdc_available(void)
{
    _rx_release();
    return hal_ringbuf_count(&_cdc.rx_ring);
}

/* ============================================================
 * HID API
 * ============================================================ */

void hal_usb_hid_set_rx_callback(hal_usb_hid_rx_cb_t cb, void *ctx)
{
    _cdc.hid_rx_cb     = cb;
    _cdc.hid_rx_cb_ctx = ctx;
}

int hal_usb_hid_send_report(const uint8_t *buf, uint16_t len)
{
    if (!_cdc.configured) return -1;
    if (len > EP3_MAX_PACKET) len = EP3_MAX_PACKET;

    /* Wait for the previous report, with the same policy as CDC writes: never
     * from an interrupt, never while suspended, at most
     * HAL_USB_HID_TX_TIMEOUT_MS, and not at all again after a timeout until
     * the host collects a report. */
    if (_cdc.hid_tx_busy) {
        if (!_usb_can_wait() || _cdc.suspended || _cdc.hid_stalled) return -1;
        uint32_t t0 = _systick_ticks;
        while (_cdc.hid_tx_busy) {
            if (!_cdc.configured || _cdc.suspended) return -1;
            if ((uint32_t)(_systick_ticks - t0) >= HAL_USB_HID_TX_TIMEOUT_MS) {
                _cdc.hid_stalled = 1;
                return -1;
            }
        }
    }

    /* Zero-pad to 64 bytes — HID report descriptor declares fixed-size reports.
     * macOS (and some other hosts) ignore reports shorter than declared size. */
    uint8_t padded[EP3_MAX_PACKET];
    __builtin_memcpy(padded, buf, len);
    if (len < EP3_MAX_PACKET)
        __builtin_memset(padded + len, 0, EP3_MAX_PACKET - len);

    uint32_t irq = ll_irq_save();
    if (_cdc.hid_tx_busy) {                 /* an interrupt's report got there first */
        ll_irq_restore(irq);
        return -1;
    }
    ll_usb_pma_write(PMA_EP3_TX, padded, EP3_MAX_PACKET);
    ll_usb_bdt_set_tx_count(3, EP3_MAX_PACKET);
    _cdc.hid_tx_busy = 1;
    ll_usb_ep_set_stat_tx(3, USB_EP_STAT_VALID);
    ll_irq_restore(irq);

    return (int)len;
}

/* ################################################################
 * STM32H523 — USB DRD Full-Speed (32-bit registers, 2KB PMA)
 *
 * Same CDC protocol as L4, different register-level access:
 *   - 32-bit CHEP registers (vs 16-bit EPnR)
 *   - 32-bit PMA access (vs 16-bit)
 *   - CNTR/ISTR bits shifted up ~2 positions
 *   - DPPU_DPD at bit 16 (vs bit 15)
 *   - IRQ: USB_DRD_FS_IRQHandler (vector 73)
 * ################################################################ */

#elif defined(STM32H523xx)

#include "ll_usb_drd.h"
#include "ll_rcc.h"
#include "ll_crs.h"
#include "ll_gpio.h"
#include "ll_systick.h"
#include "hal_dfu.h"
#include <string.h>
#include <stdio.h>

/* ============================================================
 * PMA buffer layout (2048 bytes total)
 *
 * BDT:     0x00 - 0x3F  (8 endpoints x 8 bytes = 64 bytes)
 * EP0 TX:  0x40 - 0x7F  (64 bytes)
 * EP0 RX:  0x80 - 0xBF  (64 bytes)
 * EP1 TX:  0xC0 - 0xFF  (64 bytes)  — CDC bulk
 * EP1 RX:  0x100 - 0x13F (64 bytes) — CDC bulk
 * EP2 TX:  0x140 - 0x14F (16 bytes) — CDC notification
 * EP3 TX:  0x150 - 0x18F (64 bytes) — HID reports IN  (device -> host)
 * EP3 RX:  0x190 - 0x1CF (64 bytes) — HID reports OUT (host -> device)
 * ============================================================ */

#define PMA_EP0_TX          0x40
#define PMA_EP0_RX          0x80
#define PMA_EP1_TX          0xC0
#define PMA_EP1_RX          0x100
#define PMA_EP2_TX          0x140
#define PMA_EP3_TX          0x150
#define PMA_EP3_RX          0x190

#define EP0_MAX_PACKET      64
#define EP1_MAX_PACKET      64
#define EP2_MAX_PACKET      8
#define EP3_MAX_PACKET      64

/* ============================================================
 * USB descriptor data (identical to L4)
 * ============================================================ */

#define USB_REQ_GET_STATUS          0x00
#define USB_REQ_CLEAR_FEATURE       0x01
#define USB_REQ_SET_FEATURE         0x03
#define USB_REQ_SET_ADDRESS         0x05
#define USB_REQ_GET_DESCRIPTOR      0x06
#define USB_REQ_SET_DESCRIPTOR      0x07
#define USB_REQ_GET_CONFIGURATION   0x08
#define USB_REQ_SET_CONFIGURATION   0x09

#define USB_DESC_DEVICE             0x01
#define USB_DESC_CONFIGURATION      0x02
#define USB_DESC_STRING             0x03
#define USB_DESC_INTERFACE          0x04
#define USB_DESC_ENDPOINT           0x05
#define USB_DESC_DEVICE_QUALIFIER   0x06
#define USB_DESC_CS_INTERFACE       0x24

#define CDC_SET_LINE_CODING         0x20
#define CDC_GET_LINE_CODING         0x21
#define CDC_SET_CONTROL_LINE_STATE  0x22
#define CDC_SEND_BREAK              0x23

#define CDC_ACM_SUBCLASS            0x02
#define CDC_AT_PROTOCOL             0x01

/* Composite device: CDC-ACM (serial) + HID (generic reports).
 * Uses IAD (Interface Association Descriptor) to group CDC interfaces. */

#define USB_DESC_IAD            0x0B
#define USB_DESC_HID            0x21
#define USB_DESC_HID_REPORT     0x22

/* HID class requests */
#define HID_GET_REPORT          0x01
#define HID_GET_IDLE            0x02
#define HID_SET_REPORT          0x09
#define HID_SET_IDLE            0x0A

static const uint8_t dev_desc[] = {
    18,                     /* bLength */
    USB_DESC_DEVICE,        /* bDescriptorType */
    0x00, 0x02,             /* bcdUSB = 2.00 */
    0xEF,                   /* bDeviceClass = Miscellaneous (composite) */
    0x02,                   /* bDeviceSubClass = Common Class */
    0x01,                   /* bDeviceProtocol = IAD */
    EP0_MAX_PACKET,         /* bMaxPacketSize0 */
    (CORE_USB_VID & 0xFF), ((CORE_USB_VID >> 8) & 0xFF),       /* idVendor */
    (CORE_USB_PID & 0xFF), ((CORE_USB_PID >> 8) & 0xFF),       /* idProduct */
    0x00, 0x01,             /* bcdDevice = 1.00 */
    1,                      /* iManufacturer (string index 1) */
    2,                      /* iProduct (string index 2) */
    3,                      /* iSerialNumber (string index 3) */
    1,                      /* bNumConfigurations */
};

/* HID Report Descriptor — generic vendor-defined, bidirectional 64-byte
 * reports (Input + Output). */
static const uint8_t hid_report_desc[] = {
    0x06, 0x00, 0xFF,       /* Usage Page (Vendor Defined 0xFF00) */
    0x09, 0x01,             /* Usage (Vendor Usage 1) */
    0xA1, 0x01,             /* Collection (Application) */
    0x15, 0x00,             /*   Logical Minimum (0) */
    0x26, 0xFF, 0x00,       /*   Logical Maximum (255) */
    0x75, 0x08,             /*   Report Size (8 bits) */
    0x95, 0x40,             /*   Report Count (64) */
    0x09, 0x01,             /*   Usage (Vendor Usage 1) */
    0x81, 0x02,             /*   Input (Data, Variable, Absolute) */
    0x09, 0x02,             /*   Usage (Vendor Usage 2) */
    0x91, 0x02,             /*   Output (Data, Variable, Absolute) */
    0xC0,                   /* End Collection */
};

#define HID_REPORT_DESC_LEN  sizeof(hid_report_desc)

/* Configuration descriptor:
 * Config(9) + IAD(8) + CDC_IF0(9) + CDC_Func(5+4+5+5) + EP2(7) +
 * CDC_IF1(9) + EP1_OUT(7) + EP1_IN(7) + HID_IF2(9) + HID_Desc(9) +
 * EP3_IN(7) + EP3_OUT(7)
 * = 9 + 8 + 9 + 19 + 7 + 9 + 7 + 7 + 9 + 9 + 7 + 7 = 107 */
#define CONFIG_DESC_TOTAL_LEN   107

static const uint8_t cfg_desc[] = {
    /* Configuration descriptor */
    9, USB_DESC_CONFIGURATION,
    CONFIG_DESC_TOTAL_LEN, 0x00, 3, 1, 0, 0x80, CORE_USB_MAX_POWER_MA / 2, /* 3 interfaces */

    /* ---- IAD: group CDC interfaces 0+1 ---- */
    8, USB_DESC_IAD, 0, 2, 0x02, CDC_ACM_SUBCLASS, CDC_AT_PROTOCOL, 0,

    /* CDC Control Interface (Interface 0) */
    9, USB_DESC_INTERFACE, 0, 0, 1, 0x02, CDC_ACM_SUBCLASS, CDC_AT_PROTOCOL, 0,

    /* CDC Functional Descriptors */
    5, USB_DESC_CS_INTERFACE, 0x00, 0x10, 0x01,
    4, USB_DESC_CS_INTERFACE, 0x02, 0x02,
    5, USB_DESC_CS_INTERFACE, 0x06, 0, 1,
    5, USB_DESC_CS_INTERFACE, 0x01, 0x00, 1,

    /* EP2 IN — CDC notification (interrupt) */
    7, USB_DESC_ENDPOINT, 0x82, 0x03, EP2_MAX_PACKET, 0x00, 255,

    /* CDC Data Interface (Interface 1) */
    9, USB_DESC_INTERFACE, 1, 0, 2, 0x0A, 0x00, 0x00, 0,

    /* EP1 OUT — CDC bulk data */
    7, USB_DESC_ENDPOINT, 0x01, 0x02, EP1_MAX_PACKET, 0x00, 0,

    /* EP1 IN — CDC bulk data */
    7, USB_DESC_ENDPOINT, 0x81, 0x02, EP1_MAX_PACKET, 0x00, 0,

    /* ---- HID Interface (Interface 2) ---- */
    9, USB_DESC_INTERFACE, 2, 0, 2, 0x03, 0x00, 0x00, 0,
    /* bNumEndpoints=2 (EP3 IN + EP3 OUT); class=0x03 (HID), no subclass/protocol */

    /* HID Descriptor */
    9, USB_DESC_HID,
    0x11, 0x01,             /* bcdHID = 1.11 */
    0x00,                   /* bCountryCode = 0 */
    1,                      /* bNumDescriptors */
    USB_DESC_HID_REPORT,    /* bDescriptorType = Report */
    HID_REPORT_DESC_LEN, 0,/* wDescriptorLength */

    /* EP3 IN — HID reports (interrupt, device -> host) */
    7, USB_DESC_ENDPOINT, 0x83, 0x03, EP3_MAX_PACKET, 0x00, 1,
    /* bInterval=1ms for fastest polling */

    /* EP3 OUT — HID reports (interrupt, host -> device) */
    7, USB_DESC_ENDPOINT, 0x03, 0x03, EP3_MAX_PACKET, 0x00, 1,
};

/* Offset of the HID descriptor within cfg_desc (for GET_DESCRIPTOR HID).
 * Config(9) + IAD(8) + CDC_IF0(9) + Func(5+4+5+5=19) + EP2(7) +
 * CDC_IF1(9) + EP1out(7) + EP1in(7) + HID_IF(9) = 84 */
#define CFG_DESC_HID_OFFSET  84

/* Guard the hand-computed descriptor length against future edits. */
_Static_assert(sizeof(cfg_desc) == CONFIG_DESC_TOTAL_LEN,
               "cfg_desc length != CONFIG_DESC_TOTAL_LEN");

static const uint8_t str0_desc[] = { 4, USB_DESC_STRING, 0x09, 0x04 };

static uint8_t _str_to_desc(const char *str, uint8_t *buf, uint8_t max_len)
{
    uint8_t slen = 0;
    while (str[slen]) slen++;
    uint8_t dlen = 2 + slen * 2;
    if (dlen > max_len) dlen = max_len;
    buf[0] = dlen;
    buf[1] = USB_DESC_STRING;
    for (uint8_t i = 0; i < (dlen - 2) / 2; i++) {
        buf[2 + i * 2] = str[i];
        buf[3 + i * 2] = 0;
    }
    return dlen;
}

/* ============================================================
 * Internal state (identical to L4)
 * ============================================================ */

typedef enum {
    USB_STATE_DEFAULT = 0,
    USB_STATE_ADDRESS,
    USB_STATE_CONFIGURED,
} usb_state_t;

typedef enum {
    EP0_IDLE = 0,
    EP0_DATA_IN,
    EP0_DATA_OUT,
    EP0_STATUS_IN,
    EP0_STATUS_OUT,
} ep0_state_t;

typedef struct {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb_setup_t;

typedef struct {
    uint32_t dwDTERate;
    uint8_t  bCharFormat;
    uint8_t  bParityType;
    uint8_t  bDataBits;
} __attribute__((packed)) cdc_line_coding_t;

static struct {
    volatile usb_state_t state;
    volatile uint8_t     address_pending;
    volatile uint8_t     configured;
    volatile uint8_t     dtr;

    ep0_state_t          ep0_state;
    const uint8_t       *ep0_tx_ptr;
    uint16_t             ep0_tx_remain;

    /* EP0 OUT data receive (sized for a full 64-byte HID SET_REPORT payload) */
    uint8_t              ep0_rx_buf[EP0_MAX_PACKET];
    /* Pending EP0 DATA OUT target (0 = CDC line coding, 1 = HID SET_REPORT) */
    volatile uint8_t     ep0_out_is_hid;

    cdc_line_coding_t    line_coding;

    volatile uint8_t     tx_busy;

    hal_usb_cdc_rx_cb_t  rx_cb;
    void                *rx_cb_ctx;

    hal_ringbuf_t        rx_ring;
    uint8_t              rx_buf[HAL_USB_CDC_RX_BUF_SIZE];

    /* HID state */
    volatile uint8_t     hid_tx_busy;
    uint8_t              hid_idle_rate;
    hal_usb_hid_rx_cb_t  hid_rx_cb;       /* HID OUT report sink (EP3 OUT + SET_REPORT) */
    void                *hid_rx_cb_ctx;
} _cdc;

/* ============================================================
 * EP0 control transfer helpers (DRD LL calls)
 * ============================================================ */

static void _ep0_tx_packet(void)
{
    uint16_t len = _cdc.ep0_tx_remain;
    if (len > EP0_MAX_PACKET) len = EP0_MAX_PACKET;

    ll_usb_drd_pma_write(PMA_EP0_TX, _cdc.ep0_tx_ptr, len);
    ll_usb_drd_bdt_set_tx_count(0, len);

    _cdc.ep0_tx_ptr += len;
    _cdc.ep0_tx_remain -= len;

    ll_usb_drd_chep_set_stat_tx(0, USB_CHEP_STAT_VALID);
}

static void _ep0_send(const uint8_t *data, uint16_t len, uint16_t max_len)
{
    if (len > max_len) len = max_len;
    _cdc.ep0_tx_ptr = data;
    _cdc.ep0_tx_remain = len;
    _cdc.ep0_state = EP0_DATA_IN;
    _ep0_tx_packet();
}

static void _ep0_send_status(void)
{
    ll_usb_drd_bdt_set_tx_count(0, 0);
    _cdc.ep0_state = EP0_STATUS_IN;
    ll_usb_drd_chep_set_stat_tx(0, USB_CHEP_STAT_VALID);
}

static void _ep0_stall(void)
{
    ll_usb_drd_chep_set_stat(0, USB_CHEP_STAT_STALL, USB_CHEP_STAT_STALL);
    _cdc.ep0_state = EP0_IDLE;
}

/* ============================================================
 * Setup packet handler
 * ============================================================ */

static void _handle_setup(void)
{
    usb_setup_t setup;
    uint8_t buf[8];

    ll_usb_drd_pma_read(PMA_EP0_RX, buf, 8);
    setup.bmRequestType = buf[0];
    setup.bRequest      = buf[1];
    setup.wValue        = buf[2] | (buf[3] << 8);
    setup.wIndex        = buf[4] | (buf[5] << 8);
    setup.wLength       = buf[6] | (buf[7] << 8);

    uint8_t type    = setup.bmRequestType & 0x60;
    uint8_t dir_in  = setup.bmRequestType & 0x80;

    /* ---- Standard requests ---- */
    if (type == 0x00) {
        switch (setup.bRequest) {

        case USB_REQ_GET_DESCRIPTOR: {
            uint8_t desc_type = setup.wValue >> 8;
            uint8_t desc_idx  = setup.wValue & 0xFF;

            switch (desc_type) {
            case USB_DESC_DEVICE:
                _ep0_send(dev_desc, sizeof(dev_desc), setup.wLength);
                return;

            case USB_DESC_CONFIGURATION:
                _ep0_send(cfg_desc, sizeof(cfg_desc), setup.wLength);
                return;

            case USB_DESC_STRING: {
                static uint8_t str_buf[64];
                uint8_t len;
                switch (desc_idx) {
                case 0:
                    _ep0_send(str0_desc, sizeof(str0_desc), setup.wLength);
                    return;
                case 1:
                    len = _str_to_desc(CORE_USB_MANUFACTURER, str_buf, sizeof(str_buf));
                    _ep0_send(str_buf, len, setup.wLength);
                    return;
                case 2:
                    len = _str_to_desc(CORE_USB_PRODUCT, str_buf, sizeof(str_buf));
                    _ep0_send(str_buf, len, setup.wLength);
                    return;
                case 3:
                    len = _str_to_desc(CORE_USB_SERIAL, str_buf, sizeof(str_buf));
                    _ep0_send(str_buf, len, setup.wLength);
                    return;
                }
                break;
            }

            case USB_DESC_DEVICE_QUALIFIER:
                _ep0_stall();
                return;

            case USB_DESC_HID:
                /* HID descriptor (9 bytes within the config descriptor).
                 * Offset: find HID descriptor start in cfg_desc. */
                if (setup.wIndex == 2) {  /* Interface 2 = HID */
                    /* HID descriptor is at a fixed offset in cfg_desc.
                     * Config(9) + IAD(8) + CDC_IF0(9) + Func(19) + EP2(7)
                     * + CDC_IF1(9) + EP1out(7) + EP1in(7) + HID_IF(9) = 84 */
                    _ep0_send(cfg_desc + CFG_DESC_HID_OFFSET, 9, setup.wLength);
                    return;
                }
                break;

            case USB_DESC_HID_REPORT:
                if (setup.wIndex == 2) {
                    _ep0_send(hid_report_desc, sizeof(hid_report_desc), setup.wLength);
                    return;
                }
                break;
            }
            break;
        }

        case USB_REQ_SET_ADDRESS:
            _cdc.address_pending = setup.wValue & 0x7F;
            _ep0_send_status();
            return;

        case USB_REQ_SET_CONFIGURATION:
            if (setup.wValue == 1) {
                _cdc.state = USB_STATE_CONFIGURED;
                _cdc.configured = 1;

                /* Configure EP1 (CDC bulk data) */
                ll_usb_drd_chep_config(1, USB_CHEP_BULK, 1);
                ll_usb_drd_bdt_set_tx(1, PMA_EP1_TX, 0);
                ll_usb_drd_bdt_set_rx(1, PMA_EP1_RX, EP1_MAX_PACKET);
                ll_usb_drd_chep_set_stat(1, USB_CHEP_STAT_NAK, USB_CHEP_STAT_VALID);

                /* Configure EP2 (CDC interrupt notification) */
                ll_usb_drd_chep_config(2, USB_CHEP_INTERRUPT, 2);
                ll_usb_drd_bdt_set_tx(2, PMA_EP2_TX, 0);
                ll_usb_drd_chep_set_stat_tx(2, USB_CHEP_STAT_NAK);

                /* Configure EP3 (HID interrupt, bidirectional):
                 * IN  = report device -> host, NAK until a report is queued
                 * OUT = report host -> device, armed VALID to receive */
                ll_usb_drd_chep_config(3, USB_CHEP_INTERRUPT, 3);
                ll_usb_drd_bdt_set_tx(3, PMA_EP3_TX, 0);
                ll_usb_drd_bdt_set_rx(3, PMA_EP3_RX, EP3_MAX_PACKET);
                ll_usb_drd_chep_set_stat(3, USB_CHEP_STAT_NAK, USB_CHEP_STAT_VALID);
                _cdc.hid_tx_busy = 0;
            } else {
                _cdc.state = USB_STATE_ADDRESS;
                _cdc.configured = 0;
            }
            _ep0_send_status();
            return;

        case USB_REQ_GET_CONFIGURATION: {
            static const uint8_t one = 1;
            static const uint8_t zero = 0;
            _ep0_send(_cdc.configured ? &one : &zero, 1, setup.wLength);
            return;
        }

        case USB_REQ_GET_STATUS: {
            static const uint8_t status[2] = { 0, 0 };
            _ep0_send(status, 2, setup.wLength);
            return;
        }

        case USB_REQ_CLEAR_FEATURE:
        case USB_REQ_SET_FEATURE:
            _ep0_send_status();
            return;
        }
    }

    /* ---- Class requests — route by interface (wIndex) ---- */
    if (type == 0x20) {
        uint16_t iface = setup.wIndex;

        /* CDC class requests (interfaces 0 and 1) */
        if (iface <= 1) {
            switch (setup.bRequest) {

            case CDC_SET_LINE_CODING:
                _cdc.ep0_out_is_hid = 0;
                _cdc.ep0_state = EP0_DATA_OUT;
                ll_usb_drd_chep_set_stat_rx(0, USB_CHEP_STAT_VALID);
                return;

            case CDC_GET_LINE_CODING:
                _ep0_send((const uint8_t *)&_cdc.line_coding, 7, setup.wLength);
                return;

            case CDC_SET_CONTROL_LINE_STATE: {
                uint8_t new_dtr = (setup.wValue & 0x01) ? 1 : 0;

                /* 1200-baud touch: DTR drop while baud=1200 triggers DFU reboot. */
                if (_cdc.dtr && !new_dtr && _cdc.line_coding.dwDTERate == 1200) {
                    _ep0_send_status();
                    for (volatile int i = 0; i < 100000; i++)
                        ;
                    hal_dfu_reboot();
                }

                _cdc.dtr = new_dtr;
                _ep0_send_status();
                return;
            }

            case CDC_SEND_BREAK:
                _ep0_send_status();
                return;
            }
        }

        /* HID class requests (interface 2) */
        if (iface == 2) {
            switch (setup.bRequest) {

            case HID_SET_IDLE:
                _cdc.hid_idle_rate = (setup.wValue >> 8) & 0xFF;
                _ep0_send_status();
                return;

            case HID_GET_IDLE: {
                static uint8_t idle;
                idle = _cdc.hid_idle_rate;
                _ep0_send(&idle, 1, setup.wLength);
                return;
            }

            case HID_GET_REPORT:
                /* Not implemented — host should read via EP3 interrupt IN */
                _ep0_stall();
                return;

            case HID_SET_REPORT:
                /* Accept the report payload in the EP0 DATA OUT stage and
                 * route it to the HID OUT sink (alongside the EP3 OUT path). */
                _cdc.ep0_out_is_hid = 1;
                _cdc.ep0_state = EP0_DATA_OUT;
                ll_usb_drd_chep_set_stat_rx(0, USB_CHEP_STAT_VALID);
                return;
            }
        }
    }

    /* Unknown request */
    if (dir_in) {
        _ep0_stall();
    } else {
        _ep0_send_status();
    }
}

/* ============================================================
 * USB reset handler
 * ============================================================ */

/* TX queue for hal_usb_cdc_try_write() — see the TX section below. */
static uint8_t           _txq[HAL_USB_CDC_TX_QUEUE_SIZE];
static volatile uint16_t _txq_head;   /* written by try_write (main) */
static volatile uint16_t _txq_tail;   /* written by _txq_send_next   */
/* Set by the first try_write. The ISR chains through this pointer rather than
 * calling _txq_send_next directly, so a project that never uses try_write
 * links neither it nor the queue buffer (--gc-sections). */
static void (*volatile _txq_chain)(void);

static void _handle_reset(void)
{
    /* Configure EP0 (control) */
    ll_usb_drd_chep_config(0, USB_CHEP_CONTROL, 0);
    ll_usb_drd_bdt_set_tx(0, PMA_EP0_TX, 0);
    ll_usb_drd_bdt_set_rx(0, PMA_EP0_RX, EP0_MAX_PACKET);

    /* Enable EP0 RX */
    ll_usb_drd_chep_set_stat(0, USB_CHEP_STAT_NAK, USB_CHEP_STAT_VALID);

    /* Set address 0 with function enabled */
    ll_usb_drd_set_address(0);

    _cdc.state = USB_STATE_DEFAULT;
    _cdc.configured = 0;
    _cdc.dtr = 0;
    _cdc.tx_busy = 0;
    _txq_tail = _txq_head;          /* drop anything queued for the old session */
    _cdc.ep0_state = EP0_IDLE;

    /* Enable interrupts: CTR + RESET + SUSPEND + WAKEUP */
    USB_CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
}

/* ============================================================
 * Correct Transfer (CTR) handler
 * ============================================================ */

static void _handle_ctr(void)
{
    uint32_t istr = USB_ISTR;
    uint8_t  ep   = istr & USB_ISTR_IDN_MASK;

    if (ep == 0) {
        uint32_t chep0 = ll_usb_drd_chep_read(0);

        /* ---- EP0 RX (OUT/SETUP from host) ---- */
        if (chep0 & USB_CHEP_VTRX) {
            ll_usb_drd_chep_clr_vtrx(0);

            if (chep0 & USB_CHEP_SETUP) {
                _handle_setup();
            } else {
                if (_cdc.ep0_state == EP0_DATA_OUT) {
                    uint16_t count = ll_usb_drd_bdt_get_rx_count(0);
                    if (count > sizeof(_cdc.ep0_rx_buf))
                        count = sizeof(_cdc.ep0_rx_buf);
                    ll_usb_drd_pma_read(PMA_EP0_RX, _cdc.ep0_rx_buf, count);

                    if (_cdc.ep0_out_is_hid) {
                        /* HID SET_REPORT payload — deliver to the OUT sink */
                        if (_cdc.hid_rx_cb)
                            _cdc.hid_rx_cb(_cdc.ep0_rx_buf, count, _cdc.hid_rx_cb_ctx);
                        _cdc.ep0_out_is_hid = 0;
                    } else if (count == 7) {
                        /* CDC SET_LINE_CODING — copy the 7 bytes */
                        memcpy(&_cdc.line_coding, _cdc.ep0_rx_buf, 7);
                    }
                    _ep0_send_status();
                } else if (_cdc.ep0_state == EP0_STATUS_OUT) {
                    _cdc.ep0_state = EP0_IDLE;
                    ll_usb_drd_chep_set_stat_rx(0, USB_CHEP_STAT_VALID);
                } else {
                    ll_usb_drd_chep_set_stat_rx(0, USB_CHEP_STAT_VALID);
                }
            }
        }

        /* ---- EP0 TX (IN to host) ---- */
        if (chep0 & USB_CHEP_VTTX) {
            ll_usb_drd_chep_clr_vttx(0);

            if (_cdc.ep0_state == EP0_DATA_IN) {
                if (_cdc.ep0_tx_remain > 0) {
                    _ep0_tx_packet();
                } else {
                    _cdc.ep0_state = EP0_STATUS_OUT;
                    ll_usb_drd_chep_set_stat_rx(0, USB_CHEP_STAT_VALID);
                }
            } else if (_cdc.ep0_state == EP0_STATUS_IN) {
                if (_cdc.address_pending) {
                    ll_usb_drd_set_address(_cdc.address_pending);
                    _cdc.state = USB_STATE_ADDRESS;
                    _cdc.address_pending = 0;
                }
                _cdc.ep0_state = EP0_IDLE;
                ll_usb_drd_chep_set_stat_rx(0, USB_CHEP_STAT_VALID);
            }
        }
    }

    else if (ep == 1) {
        uint32_t chep1 = ll_usb_drd_chep_read(1);

        /* ---- EP1 RX (bulk OUT — host -> device data) ---- */
        if (chep1 & USB_CHEP_VTRX) {
            ll_usb_drd_chep_clr_vtrx(1);

            uint16_t count = ll_usb_drd_bdt_get_rx_count(1);
            if (count > 0) {
                uint8_t tmp[EP1_MAX_PACKET];
                ll_usb_drd_pma_read(PMA_EP1_RX, tmp, count);

                if (_cdc.rx_cb) {
                    _cdc.rx_cb(tmp, count, _cdc.rx_cb_ctx);
                } else {
                    for (uint16_t i = 0; i < count; i++) {
                        hal_ringbuf_put(&_cdc.rx_ring, tmp[i]);
                    }
                }
            }
            ll_usb_drd_chep_set_stat_rx(1, USB_CHEP_STAT_VALID);
        }

        /* ---- EP1 TX (bulk IN — device -> host complete) ---- */
        if (chep1 & USB_CHEP_VTTX) {
            ll_usb_drd_chep_clr_vttx(1);
            _cdc.tx_busy = 0;
            if (_txq_chain) _txq_chain();
        }
    }

    else if (ep == 3) {
        uint32_t chep3 = ll_usb_drd_chep_read(3);

        /* ---- EP3 RX (HID OUT report — host -> device) ---- */
        if (chep3 & USB_CHEP_VTRX) {
            ll_usb_drd_chep_clr_vtrx(3);

            uint16_t count = ll_usb_drd_bdt_get_rx_count(3);
            if (count > EP3_MAX_PACKET) count = EP3_MAX_PACKET;
            if (count > 0 && _cdc.hid_rx_cb) {
                uint8_t tmp[EP3_MAX_PACKET];
                ll_usb_drd_pma_read(PMA_EP3_RX, tmp, count);
                _cdc.hid_rx_cb(tmp, count, _cdc.hid_rx_cb_ctx);
            }
            /* Re-arm RX for the next report */
            ll_usb_drd_chep_set_stat_rx(3, USB_CHEP_STAT_VALID);
        }

        /* ---- EP3 TX (HID report sent) ---- */
        if (chep3 & USB_CHEP_VTTX) {
            ll_usb_drd_chep_clr_vttx(3);
            _cdc.hid_tx_busy = 0;
        }
    }
}

/* ============================================================
 * USB Interrupt Handler (H5 vector name)
 * ============================================================ */

void USB_DRD_FS_IRQHandler(void)
{
    uint32_t istr = USB_ISTR;

    if (istr & USB_ISTR_RESET) {
        USB_ISTR = ~USB_ISTR_RESET;
        _handle_reset();
        return;
    }

    if (istr & USB_ISTR_CTR) {
        _handle_ctr();
    }

    if (istr & USB_ISTR_SUSP) {
        USB_ISTR = ~USB_ISTR_SUSP;
        USB_CNTR |= USB_CNTR_SUSPEN;
    }

    if (istr & USB_ISTR_WKUP) {
        USB_ISTR = ~USB_ISTR_WKUP;
        USB_CNTR &= ~USB_CNTR_SUSPEN;
    }
}

/**
 * Poll USB events — call from main loop as ISR alternative.
 * Same logic as the interrupt handler.
 */
void hal_usb_cdc_poll(void)
{
    USB_DRD_FS_IRQHandler();
}

/* ============================================================
 * Public API
 * ============================================================ */

void hal_usb_cdc_init(void)
{
    memset(&_cdc, 0, sizeof(_cdc));
    hal_ringbuf_init(&_cdc.rx_ring, _cdc.rx_buf, HAL_USB_CDC_RX_BUF_SIZE);

    _cdc.line_coding.dwDTERate   = 115200;
    _cdc.line_coding.bCharFormat = 0;
    _cdc.line_coding.bParityType = 0;
    _cdc.line_coding.bDataBits   = 8;

    /* USB requires SYSCLK > 4 MHz for reliable enumeration.
     * If running from the low-speed CSI (4 MHz), automatically
     * switch to HSI/2 (32 MHz) before enabling USB. */
    {
        /* RCC_CFGR1 SWS bits [4:3] indicate current SYSCLK source.
         * 0=HSI, 1=CSI, 2=HSE, 3=PLL1. CSI (4MHz) is too slow. */
        uint32_t sws = (REG32(RCC_BASE + 0x1CUL) >> 3) & 0x3UL;
        if (sws == 0x1UL) {  /* CSI active — too slow for USB */
            /* Set flash latency for 32 MHz (LATENCY=1) */
            MOD_BITS(REG32(0x40022000UL), 0xFUL, 1UL);
            while ((REG32(0x40022000UL) & 0xF) != 1) ;

            /* Enable HSI (64 MHz on H5), divide by 2 → 32 MHz */
            ll_rcc_hsi16_enable();
            while (!ll_rcc_hsi16_ready()) ;
            MOD_BITS(REG32(RCC_BASE + 0x00UL), 3UL << 5, 1UL << 5);

            /* Switch SYSCLK to HSI */
            ll_rcc_set_sysclk(LL_RCC_SYSCLK_HSI16);

            /* Update SysTick for new clock speed */
            ll_systick_init(32000000UL);
        }
    }

    /* Enable HSI48 for crystal-less USB */
    ll_rcc_hsi48_enable();
    while (!ll_rcc_hsi48_ready())
        ;

    /* Select HSI48 as USB clock source (RCC_CCIPR4 USBSEL=0b11) */
    ll_rcc_set_usb_clk_source(LL_RCC_USB_HSI48);

    /* Enable CRS clock and sync to USB SOF */
    ll_rcc_apb1_clk_enable(LL_APB1_CRS);
    ll_crs_usb_sync_enable();

    /* Enable USB peripheral clock (APB2) */
    ll_rcc_apb2_clk_enable(LL_APB2_USB);

    /* Enable VDDUSB power supply.
     * PWR_USBSCR at PWR_BASE + 0x38:
     *   bit 24: USB33DEN — enable USB 3.3V regulator
     *   bit 25: USB33SV  — supply valid (removes VDDUSB isolation)
     * PWR_VMSR at PWR_BASE + 0x3C:
     *   bit 24: USB33RDY — USB 3.3V supply ready */
    SET_BITS(REG32(PWR_BASE + 0x38UL), (1UL << 24));   /* USB33DEN */
    while (!(REG32(PWR_BASE + 0x3CUL) & (1UL << 24)))  /* Wait USB33RDY */
        ;
    SET_BITS(REG32(PWR_BASE + 0x38UL), (1UL << 25));   /* USB33SV */

    /* Power on USB peripheral */
    ll_usb_drd_power_on();

    /* Enable USB interrupt */
    hal_nvic_set_priority(HAL_IRQ_USB, 3);   /* level 3 (the helper shifts it) */
    hal_nvic_enable_irq(HAL_IRQ_USB);

    /* Initial interrupt mask — just RESET for now */
    USB_CNTR = USB_CNTR_RESETM;

    /* Connect DP pull-up — host will see us */
    ll_usb_drd_connect();
}

int hal_usb_cdc_connected(void)
{
    return _cdc.configured && _cdc.dtr;
}

void hal_usb_cdc_set_rx_callback(hal_usb_cdc_rx_cb_t cb, void *ctx)
{
    _cdc.rx_cb     = cb;
    _cdc.rx_cb_ctx = ctx;
}

/* ---- TX queue (non-blocking writes) ----
 *
 * hal_usb_cdc_try_write() queues a buffer all-or-nothing and returns; the
 * TX-complete interrupt drains the queue a packet at a time. The blocking
 * write path does not use the queue, but it waits for the queue to empty
 * first — so bytes reach the host in call order, and a queued buffer (a
 * Scope frame) is never split by print text. With nothing queued the
 * blocking path behaves exactly as it did before the queue existed. */

/* Send the next queued packet if the endpoint is idle. Runs in the USB
 * interrupt, and from try_write with interrupts masked. Packets are capped
 * one byte short of full: a transfer ending on a full 64-byte packet is not
 * complete until a ZLP follows, and some hosts hold the data until then. */
static void _txq_send_next(void)
{
    if (_cdc.tx_busy) return;
    uint16_t n = (uint16_t)(_txq_head - _txq_tail);
    if (n == 0) return;
    if (n > EP1_MAX_PACKET - 1) n = EP1_MAX_PACKET - 1;

    uint8_t pkt[EP1_MAX_PACKET];
    for (uint16_t i = 0; i < n; i++) {
        pkt[i] = _txq[(uint16_t)(_txq_tail + i) & (HAL_USB_CDC_TX_QUEUE_SIZE - 1)];
    }
    _txq_tail = (uint16_t)(_txq_tail + n);

    ll_usb_drd_pma_write(PMA_EP1_TX, pkt, n);
    ll_usb_drd_bdt_set_tx_count(1, n);
    _cdc.tx_busy = 1;
    ll_usb_drd_chep_set_stat_tx(1, USB_CHEP_STAT_VALID);
}

int hal_usb_cdc_try_write(const uint8_t *buf, uint16_t len)
{
    if (!_cdc.configured || !_cdc.dtr) return -1;
    if (len == 0) return 0;

    uint16_t used = (uint16_t)(_txq_head - _txq_tail);
    if (len > (uint16_t)(HAL_USB_CDC_TX_QUEUE_SIZE - used)) return 0;

    uint16_t head = _txq_head;
    for (uint16_t i = 0; i < len; i++) {
        _txq[(uint16_t)(head + i) & (HAL_USB_CDC_TX_QUEUE_SIZE - 1)] = buf[i];
    }
    _txq_head = (uint16_t)(head + len);
    _txq_chain = _txq_send_next;

    uint32_t irq = ll_irq_save();
    _txq_send_next();
    ll_irq_restore(irq);
    return (int)len;
}

int hal_usb_cdc_write(const uint8_t *buf, uint16_t len)
{
    if (!_cdc.configured || !_cdc.dtr) return -1;

    uint16_t sent = 0;
    while (sent < len) {
        /* Anything queued by try_write goes out first. */
        while (_cdc.tx_busy || _txq_head != _txq_tail) {
            if (!_cdc.dtr) return (int)sent;
        }

        uint16_t chunk = len - sent;
        if (chunk > EP1_MAX_PACKET) chunk = EP1_MAX_PACKET;

        ll_usb_drd_pma_write(PMA_EP1_TX, buf + sent, chunk);
        ll_usb_drd_bdt_set_tx_count(1, chunk);
        _cdc.tx_busy = 1;
        ll_usb_drd_chep_set_stat_tx(1, USB_CHEP_STAT_VALID);

        sent += chunk;
    }

    return (int)sent;
}

int hal_usb_cdc_printf(const char *fmt, ...)
{
    char buf[HAL_USB_CDC_PRINTF_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n > 0) {
        hal_usb_cdc_write((const uint8_t *)buf, (uint16_t)n);
    }
    return n;
}

int hal_usb_cdc_rx_ready(void)
{
    return !hal_ringbuf_empty(&_cdc.rx_ring);
}

uint8_t hal_usb_cdc_getc(void)
{
    uint8_t byte;
    while (!hal_ringbuf_get(&_cdc.rx_ring, &byte))
        ;
    return byte;
}

int hal_usb_cdc_rx_try(uint8_t *byte)
{
    return hal_ringbuf_get(&_cdc.rx_ring, byte);
}

uint16_t hal_usb_cdc_read(uint8_t *buf, uint16_t max_len)
{
    return hal_ringbuf_read(&_cdc.rx_ring, buf, max_len);
}

uint16_t hal_usb_cdc_available(void)
{
    return hal_ringbuf_count(&_cdc.rx_ring);
}

/* ============================================================
 * HID API
 * ============================================================ */

void hal_usb_hid_set_rx_callback(hal_usb_hid_rx_cb_t cb, void *ctx)
{
    _cdc.hid_rx_cb     = cb;
    _cdc.hid_rx_cb_ctx = ctx;
}

int hal_usb_hid_send_report(const uint8_t *buf, uint16_t len)
{
    if (!_cdc.configured) return -1;
    if (len > EP3_MAX_PACKET) len = EP3_MAX_PACKET;

    /* Wait for previous report to complete */
    while (_cdc.hid_tx_busy) {
        if (!_cdc.configured) return -1;
    }

    /* Zero-pad to 64 bytes — HID report descriptor declares fixed-size reports.
     * macOS (and some other hosts) ignore reports shorter than declared size. */
    uint8_t padded[EP3_MAX_PACKET];
    __builtin_memcpy(padded, buf, len);
    if (len < EP3_MAX_PACKET)
        __builtin_memset(padded + len, 0, EP3_MAX_PACKET - len);

    ll_usb_drd_pma_write(PMA_EP3_TX, padded, EP3_MAX_PACKET);
    ll_usb_drd_bdt_set_tx_count(3, EP3_MAX_PACKET);
    _cdc.hid_tx_busy = 1;
    ll_usb_drd_chep_set_stat_tx(3, USB_CHEP_STAT_VALID);

    return (int)len;
}

#endif /* STM32H523xx */
