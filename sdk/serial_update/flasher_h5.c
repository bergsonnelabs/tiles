/**
 * flasher_h5.c — Serial-update flasher for STM32H523 (Core.ST.H5)
 *
 * Runs from SRAM at SU_FLASHER_BASE while the application's flash is rewritten.
 * The app (hal_usb_cdc.c, STM32H523 branch) jumps here from its USB interrupt
 * when the host sets the CDC line coding to SU_TRIGGER_BAUD, having left the
 * USB peripheral configured, the ICACHE off and the status stage of that
 * SET_LINE_CODING armed.
 *
 * The same design as flasher_l4.c (read that first): the flasher takes the
 * connection over as-is (address, endpoints, USBSRAM layout, data toggles),
 * polls the peripheral with interrupts off, answers every control request the
 * app would, and feeds the IWDG throughout. What differs on the H5:
 *
 *   - USB FS "DRD" registers (RM0481 §55): 32-bit CHEPnR and USBSRAM.
 *   - Flash (RM0481 §7): 8 KB sectors in two banks, programmed in 128-bit
 *     flash words that must each be written exactly once (SU_PROG_UNIT 16),
 *     so a "page" is a sector and a DATA frame carries 8 KB.
 *   - Leaving: the H5 has no empty-flash check (RM0481 §4.1: BOOT0 low always
 *     boots NSBOOTADD, 0x08000000), and with BOOT0 high any reset lands in the
 *     ROM bootloader. So the flasher never resets to start an image: it
 *     detaches from the host, resets the USB peripheral and jumps to the
 *     image's reset vector, which starts it as the ST ROM's DFU "leave" does.
 *     Only a fault resets (with page 0 erased, a BOOT0-low Core then needs
 *     BOOT0 to recover: docs/serial-update-protocol.md §5).
 *
 * Built -ffreestanding and linked -nostdlib (see Makefile): nothing here may
 * call into the flash being rewritten.
 */

#include <stdint.h>
#include "ll_usb_drd.h"
#include "ll_rcc.h"
#include "ll_flash.h"
#include "su_core.h"

/* ============================================================
 * Registers (RM0481)
 * ============================================================ */

#define SCB_VTOR            REG32(0xE000ED08UL)
#define SCB_AIRCR           REG32(0xE000ED0CUL)
#define IWDG_KR             REG32(0x40003000UL)          /* §44.7.1 */
#define DBGMCU_IDCODE       REG32(0x44024000UL)          /* §59.12.4, software base; DEV_ID 0x478 */
#define FLASHSIZE_KB        (*(volatile uint16_t *)0x08FFF80CUL)  /* §60.2, 16-bit read */
#define RCC_APB2RSTR        REG32(RCC_BASE + 0x7CUL)     /* §11.8.24, USBRST = bit 24 */

#define SU_FLASH_START      0x08000000UL
#define SU_SRAM_START       0x20000000UL
#define SU_SRAM_END         0x20044000UL                 /* 272 KB */

/* ============================================================
 * USB — must match hal_usb_cdc.c (STM32H523 branch) exactly: the flasher
 * inherits the endpoints and USBSRAM buffers the app configured.
 * ============================================================ */

#define PMA_EP0_TX          0x040
#define PMA_EP0_RX          0x080
#define PMA_EP1_TX          0x0C0
#define PMA_EP1_RX          0x100
#define PMA_EP2_TX          0x140
#define PMA_EP3_TX          0x150
#define PMA_EP3_RX          0x190
#define EP0_MAX_PACKET      64
#define EP1_MAX_PACKET      64
#define EP3_MAX_PACKET      64

#define REQ_GET_STATUS          0x00
#define REQ_CLEAR_FEATURE       0x01
#define REQ_SET_FEATURE         0x03
#define REQ_SET_ADDRESS         0x05
#define REQ_GET_DESCRIPTOR      0x06
#define REQ_GET_CONFIGURATION   0x08
#define REQ_SET_CONFIGURATION   0x09
#define DESC_DEVICE             0x01
#define DESC_CONFIGURATION      0x02
#define DESC_STRING             0x03
#define DESC_DEVICE_QUALIFIER   0x06
#define DESC_HID                0x21
#define DESC_HID_REPORT         0x22
#define CDC_SET_LINE_CODING     0x20
#define CDC_GET_LINE_CODING     0x21
#define CDC_SET_CONTROL_LINE    0x22
#define CDC_SEND_BREAK          0x23
#define HID_GET_REPORT          0x01
#define HID_GET_IDLE            0x02
#define HID_SET_REPORT          0x09
#define HID_SET_IDLE            0x0A

typedef enum { EP0_IDLE = 0, EP0_DATA_IN, EP0_DATA_OUT, EP0_STATUS_IN, EP0_STATUS_OUT } ep0_state_t;

static struct {
    uint8_t  dev[18];
    uint8_t  cfg[160];
    uint16_t cfg_len;
    uint16_t hid_off;
    uint8_t  hid[96];
    uint16_t hid_len;
    uint8_t  str[3][64];            /* ready-made string descriptors 1..3 */
    uint8_t  str0[4];

    uint8_t  configured;
    uint8_t  address_pending;
    ep0_state_t ep0_state;
    const uint8_t *ep0_tx_ptr;
    uint16_t ep0_tx_remain;
    uint8_t  ep0_out_is_line_coding;
    uint8_t  ep0_rx[EP0_MAX_PACKET];
    uint8_t  line_coding[7];
    uint8_t  hid_idle;
    uint8_t  one, zero, status[2];

    uint16_t last_fn;
} U;

extern uint32_t _sbss, _ebss;

void su_main(void) __attribute__((noreturn, used));
void su_entry(void) __attribute__((naked, noreturn, used));
static void su_fault(void) __attribute__((noreturn));

/* Minimal vector table at SU_FLASHER_BASE. The app reads words 0/1 to enter
 * here (by an exception return from its USB interrupt, hal_usb_cdc.c); the
 * rest route any fault to a reset. */
__attribute__((section(".vectors"), used))
const uintptr_t su_vectors[16] = {
    SU_STACK_TOP,
    (uintptr_t)su_entry,
    (uintptr_t)su_fault, (uintptr_t)su_fault, (uintptr_t)su_fault,
    (uintptr_t)su_fault, (uintptr_t)su_fault, (uintptr_t)su_fault, 0, 0, 0,
    (uintptr_t)su_fault, (uintptr_t)su_fault, 0,
    (uintptr_t)su_fault, (uintptr_t)su_fault,
};

/* Entered in Thread mode on whatever stack the app's frame left (or, from a
 * polled handoff, SU_STACK_TOP): take our own stack, on MSP, privileged. */
void su_entry(void)
{
    __asm volatile (
        "ldr  r0, =0x20009FE0 \n"      /* SU_STACK_TOP (su_protocol.h) */
        "msr  msp, r0         \n"
        "movs r0, #0          \n"
        "msr  control, r0     \n"
        "isb                  \n"
        "b    su_main         \n"
    );
}
_Static_assert(SU_STACK_TOP == 0x20009FE0UL, "su_entry's stack literal");

/* ============================================================
 * Housekeeping
 * ============================================================ */

static inline void iwdg_feed(void) { IWDG_KR = 0xAAAAUL; }

static void sys_reset(void) __attribute__((noreturn));
static void sys_reset(void)
{
    __asm volatile ("dsb" ::: "memory");
    SCB_AIRCR = (0x05FAUL << 16) | (1UL << 2);
    __asm volatile ("dsb" ::: "memory");
    for (;;) ;
}

static void su_fault(void)
{
    sys_reset();
}

/* Milliseconds from the USB frame counter (one SOF per ms while the host is
 * there). Clock-independent: the app may have left SYSCLK anywhere. */
static uint32_t ms_elapsed(void)
{
    uint16_t fn = (uint16_t)(USB_FNR & 0x7FFu);
    uint32_t d  = (uint32_t)(fn - U.last_fn) & 0x7FFu;
    U.last_fn = fn;
    return d;
}

static void busy_wait(uint32_t loops)
{
    for (volatile uint32_t i = 0; i < loops; i++)
        if ((i & 0x3FFFu) == 0) iwdg_feed();
}

/* ============================================================
 * Endpoint register writes, race-free (see flasher_l4.c): the VTRX/VTTX
 * flags we don't mean to clear are always written 1, and "packet waiting" /
 * "IN busy" come from the STAT bits the hardware owns. The upper half
 * (host-mode fields) is written back as read, as USB_CHEP_REG_MASK does.
 * ============================================================ */

#define EP_VT_BOTH    (USB_CHEP_VTRX | USB_CHEP_VTTX)
#define EP_KEEP       (USB_CHEP_REG_MASK & ~EP_VT_BOTH)   /* rw + upper fields */

static void ep_clear_vt(uint8_t ep, uint32_t which)
{
    uint32_t r = USB_CHEPnR(ep);
    USB_CHEPnR(ep) = (r & EP_KEEP) | (EP_VT_BOTH & ~which);
}

static void ep_set_stat(uint8_t ep, int set_tx, uint32_t tx, int set_rx, uint32_t rx)
{
    uint32_t r = USB_CHEPnR(ep);
    uint32_t w = (r & EP_KEEP) | EP_VT_BOTH;
    if (set_tx) w |= (r ^ (tx << 4)) & USB_CHEP_STATTX_MASK;
    if (set_rx) w |= (r ^ (rx << 12)) & USB_CHEP_STATRX_MASK;
    USB_CHEPnR(ep) = w;
}

#define ep_clear_vtrx(ep)          ep_clear_vt((ep), USB_CHEP_VTRX)
#define ep_clear_vttx(ep)          ep_clear_vt((ep), USB_CHEP_VTTX)
#define ep_set_stat_tx(ep, s)      ep_set_stat((ep), 1, (s), 0, 0)
#define ep_set_stat_rx(ep, s)      ep_set_stat((ep), 0, 0, 1, (s))
#define ep_set_stat_both(ep, t, r) ep_set_stat((ep), 1, (t), 1, (r))

static uint32_t ep_stat_tx(uint8_t ep) { return (USB_CHEPnR(ep) & USB_CHEP_STATTX_MASK) >> 4; }
static uint32_t ep_stat_rx(uint8_t ep) { return (USB_CHEPnR(ep) & USB_CHEP_STATRX_MASK) >> 12; }

/* ============================================================
 * EP0
 * ============================================================ */

static void ep0_tx_packet(void)
{
    uint16_t len = U.ep0_tx_remain;
    if (len > EP0_MAX_PACKET) len = EP0_MAX_PACKET;
    ll_usb_drd_pma_write(PMA_EP0_TX, U.ep0_tx_ptr, len);
    ll_usb_drd_bdt_set_tx_count(0, len);
    U.ep0_tx_ptr += len;
    U.ep0_tx_remain -= len;
    ep_set_stat_tx(0, USB_CHEP_STAT_VALID);
}

static void ep0_send(const uint8_t *data, uint16_t len, uint16_t max_len)
{
    if (len > max_len) len = max_len;
    U.ep0_tx_ptr = data;
    U.ep0_tx_remain = len;
    U.ep0_state = EP0_DATA_IN;
    ep0_tx_packet();
}

static void ep0_send_status(void)
{
    ll_usb_drd_bdt_set_tx_count(0, 0);
    U.ep0_state = EP0_STATUS_IN;
    ep_set_stat_tx(0, USB_CHEP_STAT_VALID);
}

static void ep0_stall(void)
{
    ep_set_stat_both(0, USB_CHEP_STAT_STALL, USB_CHEP_STAT_STALL);
    U.ep0_state = EP0_IDLE;
}

static void configure_data_endpoints(void)
{
    ll_usb_drd_chep_config(1, USB_CHEP_BULK, 1);
    ll_usb_drd_bdt_set_tx(1, PMA_EP1_TX, 0);
    ll_usb_drd_bdt_set_rx(1, PMA_EP1_RX, EP1_MAX_PACKET);
    ep_set_stat_both(1, USB_CHEP_STAT_NAK, USB_CHEP_STAT_VALID);

    ll_usb_drd_chep_config(2, USB_CHEP_INTERRUPT, 2);
    ll_usb_drd_bdt_set_tx(2, PMA_EP2_TX, 0);
    ep_set_stat_tx(2, USB_CHEP_STAT_NAK);

    ll_usb_drd_chep_config(3, USB_CHEP_INTERRUPT, 3);
    ll_usb_drd_bdt_set_tx(3, PMA_EP3_TX, 0);
    ll_usb_drd_bdt_set_rx(3, PMA_EP3_RX, EP3_MAX_PACKET);
    ep_set_stat_both(3, USB_CHEP_STAT_NAK, USB_CHEP_STAT_VALID);
}

static void handle_setup(void)
{
    uint8_t b[8];
    ll_usb_drd_pma_read(PMA_EP0_RX, b, 8);
    uint8_t  req_type = b[0];
    uint8_t  req      = b[1];
    uint16_t wValue   = (uint16_t)(b[2] | (b[3] << 8));
    uint16_t wIndex   = (uint16_t)(b[4] | (b[5] << 8));
    uint16_t wLength  = (uint16_t)(b[6] | (b[7] << 8));
    uint8_t  type     = req_type & 0x60;

    if (type == 0x00) {
        switch (req) {
        case REQ_GET_DESCRIPTOR: {
            uint8_t dt = wValue >> 8, di = wValue & 0xFF;
            if (dt == DESC_DEVICE)        { ep0_send(U.dev, 18, wLength); return; }
            if (dt == DESC_CONFIGURATION) { ep0_send(U.cfg, U.cfg_len, wLength); return; }
            if (dt == DESC_STRING) {
                if (di == 0)             { ep0_send(U.str0, 4, wLength); return; }
                if (di >= 1 && di <= 3)  { ep0_send(U.str[di - 1], U.str[di - 1][0], wLength); return; }
            }
            if (dt == DESC_DEVICE_QUALIFIER) { ep0_stall(); return; }
            if (dt == DESC_HID && wIndex == 2)        { ep0_send(U.cfg + U.hid_off, 9, wLength); return; }
            if (dt == DESC_HID_REPORT && wIndex == 2) { ep0_send(U.hid, U.hid_len, wLength); return; }
            break;
        }
        case REQ_SET_ADDRESS:
            U.address_pending = wValue & 0x7F;
            ep0_send_status();
            return;
        case REQ_SET_CONFIGURATION:
            U.configured = (wValue == 1);
            if (U.configured) configure_data_endpoints();
            ep0_send_status();
            return;
        case REQ_GET_CONFIGURATION:
            ep0_send(U.configured ? &U.one : &U.zero, 1, wLength);
            return;
        case REQ_GET_STATUS:
            ep0_send(U.status, 2, wLength);
            return;
        case REQ_CLEAR_FEATURE:
        case REQ_SET_FEATURE:
            ep0_send_status();
            return;
        }
    }

    if (type == 0x20 && wIndex <= 1) {
        switch (req) {
        case CDC_SET_LINE_CODING:
            U.ep0_out_is_line_coding = 1;
            U.ep0_state = EP0_DATA_OUT;
            ep_set_stat_rx(0, USB_CHEP_STAT_VALID);
            return;
        case CDC_GET_LINE_CODING:
            ep0_send(U.line_coding, 7, wLength);
            return;
        case CDC_SET_CONTROL_LINE:   /* DTR / RTS: ignored — no 1200-touch here */
        case CDC_SEND_BREAK:
            ep0_send_status();
            return;
        }
    }

    if (type == 0x20 && wIndex == 2) {
        switch (req) {
        case HID_SET_IDLE:
            U.hid_idle = (uint8_t)(wValue >> 8);
            ep0_send_status();
            return;
        case HID_GET_IDLE:
            ep0_send(&U.hid_idle, 1, wLength);
            return;
        case HID_GET_REPORT:
            ep0_stall();
            return;
        case HID_SET_REPORT:           /* accepted and dropped */
            U.ep0_out_is_line_coding = 0;
            U.ep0_state = EP0_DATA_OUT;
            ep_set_stat_rx(0, USB_CHEP_STAT_VALID);
            return;
        }
    }

    if (req_type & 0x80) ep0_stall();
    else ep0_send_status();
}

static void handle_reset(void)
{
    ll_usb_drd_chep_config(0, USB_CHEP_CONTROL, 0);
    ll_usb_drd_bdt_set_tx(0, PMA_EP0_TX, 0);
    ll_usb_drd_bdt_set_rx(0, PMA_EP0_RX, EP0_MAX_PACKET);
    ep_set_stat_both(0, USB_CHEP_STAT_NAK, USB_CHEP_STAT_VALID);
    ll_usb_drd_set_address(0);

    U.configured = 0;
    U.address_pending = 0;
    U.ep0_state = EP0_IDLE;
}

static void handle_ctr(void)
{
    uint8_t ep = (uint8_t)(USB_ISTR & USB_ISTR_IDN_MASK);
    uint32_t r = USB_CHEPnR(ep);

    if (ep == 0) {
        if (r & USB_CHEP_VTRX) {
            ep_clear_vtrx(0);
            if (r & USB_CHEP_SETUP) {
                handle_setup();
            } else if (U.ep0_state == EP0_DATA_OUT) {
                uint16_t n = ll_usb_drd_bdt_get_rx_count(0);
                if (n > EP0_MAX_PACKET) n = EP0_MAX_PACKET;
                ll_usb_drd_pma_read(PMA_EP0_RX, U.ep0_rx, n);
                if (U.ep0_out_is_line_coding && n == 7)
                    for (int i = 0; i < 7; i++) U.line_coding[i] = U.ep0_rx[i];
                ep0_send_status();
            } else {
                U.ep0_state = EP0_IDLE;
                ep_set_stat_rx(0, USB_CHEP_STAT_VALID);
            }
        }
        if (r & USB_CHEP_VTTX) {
            ep_clear_vttx(0);
            if (U.ep0_state == EP0_DATA_IN) {
                if (U.ep0_tx_remain > 0) {
                    ep0_tx_packet();
                } else {
                    U.ep0_state = EP0_STATUS_OUT;
                    ep_set_stat_rx(0, USB_CHEP_STAT_VALID);
                }
            } else if (U.ep0_state == EP0_STATUS_IN) {
                if (U.address_pending) {
                    ll_usb_drd_set_address(U.address_pending);
                    U.address_pending = 0;
                }
                U.ep0_state = EP0_IDLE;
                ep_set_stat_rx(0, USB_CHEP_STAT_VALID);
            }
        }
        return;
    }

    if (ep == 1) {
        /* Just acknowledge: pump_rx / op_send read EP1's STAT bits. */
        if (r & USB_CHEP_VTRX) ep_clear_vtrx(1);
        if (r & USB_CHEP_VTTX) ep_clear_vttx(1);
        return;
    }

    /* EP2 notifications / EP3 HID: nothing to say while flashing. */
    if (r & USB_CHEP_VTRX) {
        ep_clear_vtrx(ep);
        ep_set_stat_rx(ep, USB_CHEP_STAT_VALID);
    }
    if (r & USB_CHEP_VTTX) ep_clear_vttx(ep);
}

static void usb_poll(void)
{
    uint32_t istr = USB_ISTR;
    if (istr & USB_ISTR_RESET) {
        USB_ISTR = ~USB_ISTR_RESET;
        handle_reset();
        return;
    }
    for (int n = 0; n < 8 && (USB_ISTR & USB_ISTR_CTR); n++) handle_ctr();
    if (istr & USB_ISTR_SUSP) {
        USB_ISTR = ~USB_ISTR_SUSP;
        USB_CNTR |= USB_CNTR_SUSPEN;
    }
    if (istr & USB_ISTR_WKUP) {
        USB_ISTR = ~USB_ISTR_WKUP;
        USB_CNTR &= ~USB_CNTR_SUSPEN;
    }
}

/* EP1 OUT holds a packet when the hardware has flipped it from VALID to NAK.
 * Move it into the protocol core only when there is room — until then the
 * endpoint stays NAK and the host simply waits. */
static int ep1_rx_ready(void) { return U.configured && ep_stat_rx(1) == USB_CHEP_STAT_NAK; }
static int ep1_tx_busy(void)  { return U.configured && ep_stat_tx(1) == USB_CHEP_STAT_VALID; }

static void pump_rx(void)
{
    if (!ep1_rx_ready() || su_core_rx_space() < EP1_MAX_PACKET) return;
    uint8_t  tmp[EP1_MAX_PACKET];
    uint16_t n = ll_usb_drd_bdt_get_rx_count(1);
    if (n > EP1_MAX_PACKET) n = EP1_MAX_PACKET;
    ll_usb_drd_pma_read(PMA_EP1_RX, tmp, n);
    ep_set_stat_rx(1, USB_CHEP_STAT_VALID);
    su_core_rx(tmp, n);
}

/* Wait (bounded) for the previous bulk IN packet to be collected. */
static int wait_tx_free(uint32_t timeout_ms)
{
    uint32_t waited = 0;
    ms_elapsed();
    while (ep1_tx_busy() && waited < timeout_ms) {
        iwdg_feed();
        usb_poll();
        waited += ms_elapsed();
    }
    return !ep1_tx_busy();
}

/* ============================================================
 * su_core operations
 * ============================================================ */

static void op_send(const char *line, uint32_t len)
{
    if (!U.configured || len >= EP1_MAX_PACKET) return;
    if (!wait_tx_free(500)) return;          /* host not reading: drop the reply */
    ll_usb_drd_pma_write(PMA_EP1_TX, (const uint8_t *)line, (uint16_t)len);
    ll_usb_drd_bdt_set_tx_count(1, (uint16_t)len);
    ep_set_stat_tx(1, USB_CHEP_STAT_VALID);
}

/* An 8 KB sector erase stalls the bus for milliseconds (the code runs from
 * SRAM, so only USB goes unanswered meanwhile; the host retries). */
static int op_erase_page(uint32_t page)
{
    iwdg_feed();
    ll_flash_unlock();
    int rc = ll_flash_erase_page(page);
    iwdg_feed();
    return rc;
}

static int op_program(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    ll_flash_unlock();
    for (uint32_t i = 0; i < len; i += 16) {
        if ((i & 0x3FFu) == 0) { iwdg_feed(); usb_poll(); }
        const uint8_t *p = buf + i;
        uint32_t w[4];
        for (int k = 0; k < 4; k++)
            w[k] = (uint32_t)p[4 * k] | ((uint32_t)p[4 * k + 1] << 8) |
                   ((uint32_t)p[4 * k + 2] << 16) | ((uint32_t)p[4 * k + 3] << 24);
        if (ll_flash_program_qword(addr + i, w) != 0) return -1;
    }
    return 0;
}

/* Start the image at 0x08000000 in place (the H5 can't reset into it with
 * BOOT0 high, see the top of this file): detach, give the USB peripheral back
 * in its reset state (RCC_APB2RSTR.USBRST, which also drops the DP pull-up),
 * lock the flash, and jump to the image's reset vector with its stack. The
 * image's Reset_Handler sets VTOR and clears the NVIC, and its clock setup
 * works from whatever state it finds. */
static void start_image(void) __attribute__((noreturn));
static void start_image(void)
{
    USB_BCDR &= ~USB_BCDR_DPPU_DPD;          /* the host sees a clean detach */
    busy_wait(2000000);                      /* ~80-300 ms, SYSCLK 250-64 MHz */
    RCC_APB2RSTR |= (1UL << 24);
    RCC_APB2RSTR &= ~(1UL << 24);
    ll_flash_lock();
    iwdg_feed();

    const uint32_t *vt = (const uint32_t *)SU_FLASH_START;
    uint32_t sp = vt[0], pc = vt[1];
    if (sp <= SU_SRAM_START || sp > SU_SRAM_END || !(pc & 1u)) sys_reset();
    SCB_VTOR = SU_FLASH_START;
    __asm volatile ("dsb\n isb" ::: "memory");
    __asm volatile (
        "msr msp, %0 \n"
        "bx  %1      \n"
        :
        : "r" (sp), "r" (pc)
        : "memory"
    );
    __builtin_unreachable();
}

static void op_reboot(void)
{
    wait_tx_free(200);                       /* let the last reply (DONE) go out */
    busy_wait(20000);
    if (su_core_page0_erased()) sys_reset(); /* never: su_core only asks with page 0 whole */
    start_image();
}

static su_ops_t ops;

/* ============================================================
 * Handoff
 * ============================================================ */

static int in_flash(const void *p)
{
    uint32_t a = (uint32_t)(uintptr_t)p;
    return a >= SU_FLASH_START && a < SU_FLASH_START + (uint32_t)FLASHSIZE_KB * 1024u;
}

/* A string the app built at run time (the UID serial) travels inside the
 * handoff block itself, which nothing overwrites before take_handoff(). */
static int in_handoff_serial(const su_handoff_t *h, const void *p)
{
    return (const char *)p == h->serial && h->serial[sizeof(h->serial) - 1] == '\0';
}

static void make_string(uint8_t *dst, const char *s)
{
    uint8_t n = 0;
    while (s[n] && n < 31) n++;
    dst[0] = (uint8_t)(2 + n * 2);
    dst[1] = DESC_STRING;
    for (uint8_t i = 0; i < n; i++) {
        dst[2 + i * 2] = (uint8_t)s[i];
        dst[3 + i * 2] = 0;
    }
}

/* Copy everything the app pointed at into our RAM — its flash is about to go. */
static int take_handoff(const su_handoff_t *h)
{
    if (h->magic != SU_HANDOFF_MAGIC) return 0;
    if (h->dev_desc_len != 18 || h->cfg_desc_len > sizeof(U.cfg) ||
        h->hid_report_desc_len > sizeof(U.hid) || h->cfg_hid_offset + 9u > h->cfg_desc_len)
        return 0;
    if (!in_flash(h->dev_desc) || !in_flash(h->cfg_desc) || !in_flash(h->hid_report_desc))
        return 0;
    for (int i = 0; i < 3; i++)
        if (!in_flash(h->strings[i]) && !in_handoff_serial(h, h->strings[i])) return 0;

    for (int i = 0; i < 18; i++) U.dev[i] = h->dev_desc[i];
    for (int i = 0; i < h->cfg_desc_len; i++) U.cfg[i] = h->cfg_desc[i];
    for (int i = 0; i < h->hid_report_desc_len; i++) U.hid[i] = h->hid_report_desc[i];
    U.cfg_len = h->cfg_desc_len;
    U.hid_off = h->cfg_hid_offset;
    U.hid_len = h->hid_report_desc_len;
    for (int i = 0; i < 3; i++) make_string(U.str[i], h->strings[i]);
    U.str0[0] = 4; U.str0[1] = DESC_STRING; U.str0[2] = 0x09; U.str0[3] = 0x04;
    for (int i = 0; i < 7; i++) U.line_coding[i] = h->line_coding[i];
    U.configured = h->configured;
    U.one = 1;
    return 1;
}

void su_main(void)
{
    __asm volatile ("cpsid i" ::: "memory");
    for (uint32_t *p = &_sbss; p < &_ebss; ) *p++ = 0;
    SCB_VTOR = SU_FLASHER_BASE;
    iwdg_feed();

    /* The app turned the ICACHE off before the jump (read-back must see
     * flash, and the cache flags flash writes as errors); make sure. */
    (void)ll_icache_disable();

    if (!take_handoff((const su_handoff_t *)SU_HANDOFF_ADDR))
        start_image();                      /* nothing erased: back to the app */

    /* Hot takeover: the app armed the status stage of the SET_LINE_CODING that
     * brought us here. A bulk IN packet may still be in flight: op_send waits
     * for EP1's STAT_TX to leave VALID either way. */
    U.ep0_state = EP0_STATUS_IN;
    U.last_fn   = (uint16_t)(USB_FNR & 0x7FFu);

    ops.flash_base = SU_FLASH_START;
    ops.flash_size = (uint32_t)FLASHSIZE_KB * 1024u;
    ops.image_limit = ops.flash_size - SU_NVM_RESERVED_H5;
    ops.page_size  = FLASH_PAGE_SIZE;
    ops.dev_id     = DBGMCU_IDCODE & 0xFFFu;
    ops.sram_start = SU_SRAM_START;
    ops.sram_end   = SU_SRAM_END;
    ops.flash_read = (const uint8_t *)SU_FLASH_START;
    ops.erase_page = op_erase_page;
    ops.program    = op_program;
    ops.send       = op_send;
    ops.reboot     = op_reboot;
    su_core_init(&ops);

    for (;;) {
        iwdg_feed();
        usb_poll();
        pump_rx();
        su_core_tick(ms_elapsed());
    }
}
