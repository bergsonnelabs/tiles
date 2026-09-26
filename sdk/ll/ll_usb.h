/**
 * ll_usb.h — Low-level USB Device Full-Speed peripheral
 *
 * Register definitions, PMA (Packet Memory Area) access helpers,
 * and endpoint register manipulation for STM32 USB Device FS.
 *
 * This peripheral is present on STM32L4 (L422, etc.).
 * STM32WBA and H5 use a different USB IP (USB OTG/DRD) and are
 * NOT covered by this driver.
 *
 * Key quirks:
 *   1. EP registers MUST use 16-bit access (volatile uint16_t *).
 *      Using 32-bit access causes incorrect toggle-bit behavior.
 *      All USB registers (CNTR, ISTR, etc.) also use 16-bit access
 *      for consistency with the peripheral design.
 *
 *   2. PMA uses 16-bit access at byte-addressed offsets on L4x2
 *      (PMA_ACCESS=1, 2x16 scheme — no stride doubling).
 *      Use ll_usb_pma_write/read() — do not access PMA directly.
 *
 *   3. EP register writes must preserve the rw bits, write 1 to the rc_w0
 *      CTR flags they don't mean to clear, and set toggle bits by XOR.
 *      Use the ll_usb_ep_* helpers below.
 */

#ifndef LL_USB_H
#define LL_USB_H

#include "ll_common.h"

#if defined(STM32L422xx)

/* ============================================================
 * USB peripheral base addresses
 * ============================================================ */

#define USB_BASE            0x40006800UL
#define USB_PMA_BASE        0x40006C00UL
#define USB_PMA_SIZE        1024        /* bytes */

/* ============================================================
 * USB register access — ALL 16-bit (volatile uint16_t *)
 *
 * The STM32L4 USB FS peripheral requires 16-bit register access.
 * Using 32-bit (REG32) access corrupts toggle bits in EP registers.
 * ============================================================ */

/* Endpoint registers EP0R–EP7R at offsets 0x00–0x1C (4 bytes apart) */
#define USB_EPnR(n)         (*(volatile uint16_t *)(USB_BASE + ((n) * 4)))

/* Common registers — also 16-bit access */
#define USB_CNTR            (*(volatile uint16_t *)(USB_BASE + 0x40UL))
#define USB_ISTR            (*(volatile uint16_t *)(USB_BASE + 0x44UL))
#define USB_FNR             (*(volatile uint16_t *)(USB_BASE + 0x48UL))
#define USB_DADDR           (*(volatile uint16_t *)(USB_BASE + 0x4CUL))
#define USB_BTABLE          (*(volatile uint16_t *)(USB_BASE + 0x50UL))
#define USB_LPMCSR          (*(volatile uint16_t *)(USB_BASE + 0x54UL))
#define USB_BCDR            (*(volatile uint16_t *)(USB_BASE + 0x58UL))

/* ============================================================
 * USB_CNTR bits (Control Register)
 * ============================================================ */

#define USB_CNTR_CTRM       (1U << 15)   /* Correct transfer interrupt mask */
#define USB_CNTR_PMAOVRM    (1U << 14)   /* PMA overrun interrupt mask */
#define USB_CNTR_ERRM       (1U << 13)   /* Error interrupt mask */
#define USB_CNTR_WKUPM      (1U << 12)   /* Wakeup interrupt mask */
#define USB_CNTR_SUSPM      (1U << 11)   /* Suspend interrupt mask */
#define USB_CNTR_RESETM     (1U << 10)   /* Reset interrupt mask */
#define USB_CNTR_SOFM       (1U << 9)    /* SOF interrupt mask */
#define USB_CNTR_ESOFM      (1U << 8)    /* Expected SOF interrupt mask */
#define USB_CNTR_RESUME     (1U << 4)    /* Resume request */
#define USB_CNTR_FSUSP      (1U << 3)    /* Force suspend */
#define USB_CNTR_LP_MODE    (1U << 2)    /* Low-power mode */
#define USB_CNTR_PDWN       (1U << 1)    /* Power down */
#define USB_CNTR_FRES       (1U << 0)    /* Force USB reset */

/* ============================================================
 * USB_ISTR bits (Interrupt Status Register)
 * ============================================================ */

#define USB_ISTR_CTR        (1U << 15)   /* Correct transfer */
#define USB_ISTR_PMAOVR     (1U << 14)   /* PMA overrun */
#define USB_ISTR_ERR        (1U << 13)   /* Error */
#define USB_ISTR_WKUP       (1U << 12)   /* Wakeup */
#define USB_ISTR_SUSP       (1U << 11)   /* Suspend */
#define USB_ISTR_RESET      (1U << 10)   /* USB reset */
#define USB_ISTR_SOF        (1U << 9)    /* Start of frame */
#define USB_ISTR_ESOF       (1U << 8)    /* Expected SOF */
#define USB_ISTR_DIR        (1U << 4)    /* Direction (0=TX/IN, 1=RX/OUT) */
#define USB_ISTR_EP_ID_MASK 0x0FU        /* Endpoint ID [3:0] */

/* ============================================================
 * USB_DADDR bits
 * ============================================================ */

#define USB_DADDR_EF        (1U << 7)    /* Enable function */
#define USB_DADDR_ADD_MASK  0x7FU        /* Device address [6:0] */

/* ============================================================
 * USB_BCDR bits (Battery Charging Detector)
 * ============================================================ */

#define USB_BCDR_DPPU       (1U << 15)   /* DP pull-up enable */

/* ============================================================
 * Endpoint register (EPnR) bits
 *
 * Bit types:
 *   rw:    normal read/write
 *   t:     toggle — write 1 to toggle, write 0 to leave unchanged
 *   rc_w0: read, write-0-to-clear — write 1 to leave unchanged
 *
 * [15]    CTR_RX   (rc_w0)  Correct transfer RX
 * [14]    DTOG_RX  (t)      Data toggle RX
 * [13:12] STAT_RX  (t,t)    Status RX (00=DIS, 01=STALL, 10=NAK, 11=VALID)
 * [11]    SETUP    (r)      Setup packet received
 * [10:9]  EP_TYPE  (rw)     Endpoint type
 * [8]     EP_KIND  (rw)     Endpoint kind (DBL_BUF for bulk, STATUS_OUT for control)
 * [7]     CTR_TX   (rc_w0)  Correct transfer TX
 * [6]     DTOG_TX  (t)      Data toggle TX
 * [5:4]   STAT_TX  (t,t)    Status TX (00=DIS, 01=STALL, 10=NAK, 11=VALID)
 * [3:0]   EA       (rw)     Endpoint address
 * ============================================================ */

/* EP type values */
#define USB_EP_BULK         (0x0U << 9)
#define USB_EP_CONTROL      (0x1U << 9)
#define USB_EP_ISO          (0x2U << 9)
#define USB_EP_INTERRUPT    (0x3U << 9)

/* EP status values (for STAT_TX and STAT_RX) */
#define USB_EP_STAT_DIS     0x0U
#define USB_EP_STAT_STALL   0x1U
#define USB_EP_STAT_NAK     0x2U
#define USB_EP_STAT_VALID   0x3U

/* EP register bit masks */
#define USB_EP_CTR_RX       (1U << 15)
#define USB_EP_DTOG_RX      (1U << 14)
#define USB_EP_STAT_RX_MASK (3U << 12)
#define USB_EP_SETUP        (1U << 11)
#define USB_EP_TYPE_MASK    (3U << 9)
#define USB_EP_KIND         (1U << 8)
#define USB_EP_CTR_TX       (1U << 7)
#define USB_EP_DTOG_TX      (1U << 6)
#define USB_EP_STAT_TX_MASK (3U << 4)
#define USB_EP_EA_MASK      0x0FU

/* Bits that are "invariant" — must be preserved on write. */
#define USB_EP_RW_MASK      (USB_EP_TYPE_MASK | USB_EP_KIND | USB_EP_EA_MASK)

/* CTR_RX / CTR_TX are rc_w0: writing 0 clears, writing 1 leaves them as they
 * are. Every write below sets both to 1 except the one it means to clear. */
#define USB_EP_CTR_BOTH     (USB_EP_CTR_RX | USB_EP_CTR_TX)

/* ============================================================
 * Endpoint register helpers (RM0394 §46.6.2, USB_EPnR)
 *
 * Write rules, one register write each:
 *   rw    (EP_TYPE, EP_KIND, EA)          write back what was read
 *   rc_w0 (CTR_RX, CTR_TX)                write 1, except to clear
 *   t     (DTOG_RX/TX, STAT_RX/TX)        write 1 to toggle: XOR the wanted
 *                                         value with the current one
 *
 * The CTR flags are never written back from the value read: a transfer that
 * completes between the read and the write would have its flag cleared and its
 * interrupt lost (the serial-update flasher stalled on exactly that, 2026-09).
 * ALL functions use uint16_t throughout for 16-bit register access.
 * ============================================================ */

/**
 * Read an endpoint register.
 */
static inline uint16_t ll_usb_ep_read(uint8_t ep)
{
    return USB_EPnR(ep);
}

/**
 * Change rw bits and/or clear CTR flags, leaving every toggle bit alone.
 *
 * 'mask' = bits you want to change (rw bits, or CTR_RX / CTR_TX to clear)
 * 'val'  = desired values for those bits (a CTR flag in mask is cleared when
 *          its val bit is 0; it can't be set)
 */
static inline void ll_usb_ep_write(uint8_t ep, uint16_t mask, uint16_t val)
{
    uint16_t reg = USB_EPnR(ep);
    uint16_t w = reg & USB_EP_RW_MASK;

    w = (uint16_t)((w & ~(mask & USB_EP_RW_MASK)) | (val & mask & USB_EP_RW_MASK));
    w = (uint16_t)(w | (USB_EP_CTR_BOTH & ~(mask & ~val)));

    USB_EPnR(ep) = w;
}

/**
 * Clear CTR_RX (CTR_TX is written 1: unchanged).
 */
static inline void ll_usb_ep_clr_ctr_rx(uint8_t ep)
{
    uint16_t reg = USB_EPnR(ep);
    USB_EPnR(ep) = (uint16_t)((reg & USB_EP_RW_MASK) | USB_EP_CTR_TX);
}

/**
 * Clear CTR_TX (CTR_RX is written 1: unchanged).
 */
static inline void ll_usb_ep_clr_ctr_tx(uint8_t ep)
{
    uint16_t reg = USB_EPnR(ep);
    USB_EPnR(ep) = (uint16_t)((reg & USB_EP_RW_MASK) | USB_EP_CTR_RX);
}

/**
 * Set STAT_TX to a specific value (toggle bits: XOR with the current value).
 */
static inline void ll_usb_ep_set_stat_tx(uint8_t ep, uint16_t stat)
{
    uint16_t reg = USB_EPnR(ep);
    USB_EPnR(ep) = (uint16_t)((reg & USB_EP_RW_MASK) | USB_EP_CTR_BOTH
                              | ((reg ^ (uint16_t)(stat << 4)) & USB_EP_STAT_TX_MASK));
}

/**
 * Set STAT_RX to a specific value.
 */
static inline void ll_usb_ep_set_stat_rx(uint8_t ep, uint16_t stat)
{
    uint16_t reg = USB_EPnR(ep);
    USB_EPnR(ep) = (uint16_t)((reg & USB_EP_RW_MASK) | USB_EP_CTR_BOTH
                              | ((reg ^ (uint16_t)(stat << 12)) & USB_EP_STAT_RX_MASK));
}

/**
 * Set both STAT_TX and STAT_RX at once.
 */
static inline void ll_usb_ep_set_stat(uint8_t ep, uint16_t stat_tx, uint16_t stat_rx)
{
    uint16_t reg = USB_EPnR(ep);
    USB_EPnR(ep) = (uint16_t)((reg & USB_EP_RW_MASK) | USB_EP_CTR_BOTH
                              | ((reg ^ (uint16_t)(stat_tx << 4)) & USB_EP_STAT_TX_MASK)
                              | ((reg ^ (uint16_t)(stat_rx << 12)) & USB_EP_STAT_RX_MASK));
}

/** Current STAT_TX / STAT_RX (USB_EP_STAT_*). The hardware moves VALID to NAK
 *  when a transfer completes, so these say whether a packet is still pending. */
static inline uint16_t ll_usb_ep_stat_tx(uint8_t ep)
{
    return (uint16_t)((USB_EPnR(ep) & USB_EP_STAT_TX_MASK) >> 4);
}

static inline uint16_t ll_usb_ep_stat_rx(uint8_t ep)
{
    return (uint16_t)((USB_EPnR(ep) & USB_EP_STAT_RX_MASK) >> 12);
}

/**
 * Clear data toggle TX (force DATA0).
 */
static inline void ll_usb_ep_clr_dtog_tx(uint8_t ep)
{
    uint16_t reg = USB_EPnR(ep);
    if (reg & USB_EP_DTOG_TX)
        USB_EPnR(ep) = (uint16_t)((reg & USB_EP_RW_MASK) | USB_EP_CTR_BOTH | USB_EP_DTOG_TX);
}

/**
 * Clear data toggle RX (force DATA0).
 */
static inline void ll_usb_ep_clr_dtog_rx(uint8_t ep)
{
    uint16_t reg = USB_EPnR(ep);
    if (reg & USB_EP_DTOG_RX)
        USB_EPnR(ep) = (uint16_t)((reg & USB_EP_RW_MASK) | USB_EP_CTR_BOTH | USB_EP_DTOG_RX);
}

/* ============================================================
 * PMA (Packet Memory Area) access
 *
 * PMA access on STM32L4x2:
 *   PMA_ACCESS=1 in ST HAL (2x16 scheme).
 *   Address = USB_PMA_BASE + byte_offset (no doubling).
 *   16-bit access only.
 *
 * BUT: BDT entries and buffer addresses in the BDT are PMA
 * "local" addresses. The USB peripheral uses these directly.
 * CPU access to PMA is at USB_PMA_BASE + local_address.
 * ============================================================ */

/**
 * Read a 16-bit word from PMA at the given byte offset.
 */
static inline uint16_t ll_usb_pma_rd16(uint16_t offset)
{
    return *(volatile uint16_t *)(USB_PMA_BASE + (uint32_t)offset);
}

/**
 * Write a 16-bit word to PMA at the given byte offset.
 */
static inline void ll_usb_pma_wr16(uint16_t offset, uint16_t val)
{
    *(volatile uint16_t *)(USB_PMA_BASE + (uint32_t)offset) = val;
}

/**
 * Copy data from user buffer to PMA.
 *   pma_offset: byte offset in PMA (must be even)
 *   buf:        source data
 *   len:        number of bytes to copy
 */
static inline void ll_usb_pma_write(uint16_t pma_offset, const uint8_t *buf, uint16_t len)
{
    volatile uint16_t *pma = (volatile uint16_t *)(USB_PMA_BASE + (uint32_t)pma_offset);
    uint16_t i;

    for (i = 0; i + 1 < len; i += 2) {
        *pma++ = (uint16_t)buf[i] | ((uint16_t)buf[i + 1] << 8);
    }
    /* Handle odd trailing byte */
    if (i < len) {
        *pma = (uint16_t)buf[i];
    }
}

/**
 * Copy data from PMA to user buffer.
 */
static inline void ll_usb_pma_read(uint16_t pma_offset, uint8_t *buf, uint16_t len)
{
    volatile uint16_t *pma = (volatile uint16_t *)(USB_PMA_BASE + (uint32_t)pma_offset);
    uint16_t i;

    for (i = 0; i + 1 < len; i += 2) {
        uint16_t w = *pma++;
        buf[i]     = (uint8_t)(w & 0xFF);
        buf[i + 1] = (uint8_t)(w >> 8);
    }
    if (i < len) {
        buf[i] = (uint8_t)(*pma & 0xFF);
    }
}

/* ============================================================
 * Buffer Descriptor Table (BDT)
 *
 * Located at the start of PMA (offset set by USB_BTABLE).
 * Each endpoint has 4 entries x 2 bytes = 8 bytes:
 *   +0: ADDR_TX   — TX buffer start (PMA byte offset)
 *   +2: COUNT_TX  — TX byte count [9:0]
 *   +4: ADDR_RX   — RX buffer start (PMA byte offset)
 *   +6: COUNT_RX  — [14:10] BL_SIZE+NUM_BLOCK, [9:0] received count
 * ============================================================ */

#define USB_BDT_ENTRY_SIZE  8   /* bytes per endpoint in BDT */

/** Set TX buffer address for an endpoint */
static inline void ll_usb_bdt_set_tx_addr(uint8_t ep, uint16_t pma_addr)
{
    uint16_t bdt_offset = USB_BDT_ENTRY_SIZE * ep;
    ll_usb_pma_wr16(bdt_offset + 0, pma_addr);
}

/** Set TX byte count for an endpoint */
static inline void ll_usb_bdt_set_tx_count(uint8_t ep, uint16_t count)
{
    uint16_t bdt_offset = USB_BDT_ENTRY_SIZE * ep;
    ll_usb_pma_wr16(bdt_offset + 2, count);
}

/** Set RX buffer address for an endpoint */
static inline void ll_usb_bdt_set_rx_addr(uint8_t ep, uint16_t pma_addr)
{
    uint16_t bdt_offset = USB_BDT_ENTRY_SIZE * ep;
    ll_usb_pma_wr16(bdt_offset + 4, pma_addr);
}

/**
 * Set RX buffer size allocation.
 *   max_bytes: maximum packet size this buffer can receive.
 *
 * Encoding in COUNT_RX [14:10]:
 *   BL_SIZE=0: NUM_BLOCK = max_bytes/2, capacity = NUM_BLOCK * 2
 *   BL_SIZE=1: NUM_BLOCK = max_bytes/32, capacity = NUM_BLOCK * 32
 *              (NUM_BLOCK=0 means 32 bytes)
 */
static inline void ll_usb_bdt_set_rx_count(uint8_t ep, uint16_t max_bytes)
{
    uint16_t bdt_offset = USB_BDT_ENTRY_SIZE * ep;
    uint16_t val;

    if (max_bytes <= 62) {
        val = (uint16_t)((max_bytes / 2) << 10);  /* BL_SIZE=0 */
    } else {
        val = (uint16_t)((max_bytes / 32) << 10) | (1U << 15);  /* BL_SIZE=1 */
    }
    ll_usb_pma_wr16(bdt_offset + 6, val);
}

/** Get the number of bytes received (from COUNT_RX [9:0]) */
static inline uint16_t ll_usb_bdt_get_rx_count(uint8_t ep)
{
    uint16_t bdt_offset = USB_BDT_ENTRY_SIZE * ep;
    return ll_usb_pma_rd16(bdt_offset + 6) & 0x3FF;
}

/* ============================================================
 * USB peripheral init/deinit helpers
 * ============================================================ */

/**
 * Power-on and reset the USB peripheral.
 * Call after enabling the USB clock (ll_rcc_apb1_clk_enable(LL_APB1_USB)).
 */
static inline void ll_usb_power_on(void)
{
    /* Exit power-down */
    USB_CNTR = USB_CNTR_FRES;           /* Clear PDWN, keep FRES */
    /* Startup delay — datasheet says ~1us, use a short loop */
    for (volatile int i = 0; i < 100; i++)
        ;
    /* Release reset */
    USB_CNTR = 0;
    /* Clear any pending interrupts */
    USB_ISTR = 0;
    /* Set buffer table at PMA offset 0 */
    USB_BTABLE = 0;
}

/**
 * Enable the DP pull-up resistor (signals device connection to host).
 */
static inline void ll_usb_connect(void)
{
    USB_BCDR |= USB_BCDR_DPPU;
}

/**
 * Disable the DP pull-up (disconnect from host).
 */
static inline void ll_usb_disconnect(void)
{
    USB_BCDR &= (uint16_t)~USB_BCDR_DPPU;
}

/**
 * Set the USB device address and enable function.
 */
static inline void ll_usb_set_address(uint8_t addr)
{
    USB_DADDR = (uint16_t)(USB_DADDR_EF | (addr & USB_DADDR_ADD_MASK));
}

/**
 * Configure an endpoint: type and address, EP_KIND clear, both data toggles
 * DATA0 and both directions NAK, in one write. A USB reset already zeroes the
 * toggles; SET_CONFIGURATION without one does not, and RM0394 §46.6.2 makes
 * initializing DTOG mandatory for non-control endpoints. Stale CTR flags are
 * cleared (written 0).
 */
static inline void ll_usb_ep_config(uint8_t ep, uint16_t type, uint8_t addr)
{
    uint16_t reg = USB_EPnR(ep);
    USB_EPnR(ep) = (uint16_t)(type | (addr & USB_EP_EA_MASK)
                              | (reg & (USB_EP_DTOG_RX | USB_EP_DTOG_TX))
                              | ((reg ^ (uint16_t)(USB_EP_STAT_NAK << 12)) & USB_EP_STAT_RX_MASK)
                              | ((reg ^ (uint16_t)(USB_EP_STAT_NAK << 4)) & USB_EP_STAT_TX_MASK));
}

#endif /* STM32L422xx */

#endif /* LL_USB_H */
