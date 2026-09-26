/**
 * ll_usb_drd.h — Low-level USB DRD (Dual Role Device) peripheral
 *
 * Register definitions, PMA (Packet Memory Area / USBSRAM) access helpers,
 * and endpoint register manipulation for STM32H5 USB DRD Full-Speed.
 *
 * This peripheral is present on STM32H523 (Core.ST.H5).
 * STM32L4 uses a different USB IP (USB Device FS) — see ll_usb.h.
 *
 * Key differences from L4 USB Device FS:
 *   1. All registers use 32-bit access (not 16-bit).
 *   2. CHEP registers replace EPnR — lower 16 bits same layout,
 *      upper 16 bits add host-mode fields (DEVADDR, NAK, etc.)
 *   3. PMA is 2048 bytes (vs 1024) with 32-bit access.
 *   4. BDT entries are 32-bit packed (TX: COUNT|ADDR, RX: BLSIZE|NUM|COUNT|ADDR).
 *   5. CNTR/ISTR interrupt bits shifted up ~2 positions vs L4.
 *   6. DPPU_DPD at bit 16 in BCDR (was bit 15 on L4).
 *
 * Toggle/rc_w0 semantics on CHEP registers are identical to L4 EPnR.
 */

#ifndef LL_USB_DRD_H
#define LL_USB_DRD_H

#include "ll_common.h"

#if defined(STM32H523xx)

/* ============================================================
 * USB DRD peripheral base addresses: USB_FS 0x4001_6000, USB_FS RAM
 * (USBSRAM) 0x4001_6400-0x4001_6BFF (RM0481 Table 6, non-secure aliases).
 * ============================================================ */

#define USB_DRD_BASE        0x40016000UL
#define USB_DRD_PMA_BASE    0x40016400UL    /* USBSRAM — 2048 bytes */
#define USB_DRD_PMA_SIZE    2048

/* ============================================================
 * CHEP registers (Channel/Endpoint) — 32-bit access
 *
 * Lower 16 bits identical layout to L4 EPnR:
 *   [15]    VTRX      (rc_w0)  Valid Transfer RX  (was CTR_RX)
 *   [14]    DTOGRX    (t)      Data toggle RX
 *   [13:12] STATRX    (t,t)    Status RX
 *   [11]    SETUP     (r)      Setup packet received
 *   [10:9]  UTYPE     (rw)     USB endpoint type  (was EP_TYPE)
 *   [8]     EPKIND    (rw)     Endpoint kind      (was EP_KIND)
 *   [7]     VTTX      (rc_w0)  Valid Transfer TX   (was CTR_TX)
 *   [6]     DTOGTX    (t)      Data toggle TX
 *   [5:4]   STATTX    (t,t)    Status TX
 *   [3:0]   EA        (rw)     Endpoint address
 *
 * Upper 16 bits (host-mode, not used for device CDC):
 *   [22:16] DEVADDR   (rw)     Device address (host)
 *   [23]    NAK       (rw)     NAK (host)
 *   [24]    LS_EP     (rw)     Low-speed endpoint
 *   [25]    ERR_TX    (?)
 *   [26]    ERR_RX    (?)
 *   [28:27] THREE_ERR_TX (?)
 *   [30:29] THREE_ERR_RX (?)
 * ============================================================ */

/* CHEP register access — 32-bit, 8 endpoints at offsets 0x00–0x1C */
#define USB_CHEPnR(n)       REG32(USB_DRD_BASE + ((n) * 4))

/* CHEP type values (bits [10:9], same encoding as L4) */
#define USB_CHEP_BULK       (0x0U << 9)
#define USB_CHEP_CONTROL    (0x1U << 9)
#define USB_CHEP_ISO        (0x2U << 9)
#define USB_CHEP_INTERRUPT  (0x3U << 9)

/* CHEP status values (for STATTX [5:4] and STATRX [13:12]) */
#define USB_CHEP_STAT_DIS   0x0U
#define USB_CHEP_STAT_STALL 0x1U
#define USB_CHEP_STAT_NAK   0x2U
#define USB_CHEP_STAT_VALID 0x3U

/* CHEP register bit masks (lower 16 bits) */
#define USB_CHEP_VTRX           (1U << 15)
#define USB_CHEP_DTOGRX         (1U << 14)
#define USB_CHEP_STATRX_MASK    (3U << 12)
#define USB_CHEP_SETUP          (1U << 11)
#define USB_CHEP_UTYPE_MASK     (3U << 9)
#define USB_CHEP_EPKIND         (1U << 8)
#define USB_CHEP_VTTX           (1U << 7)
#define USB_CHEP_DTOGTX         (1U << 6)
#define USB_CHEP_STATTX_MASK    (3U << 4)
#define USB_CHEP_EA_MASK        0x0FU

/* All non-toggle, non-rc_w0 bits that must be preserved on write.
 * Includes upper 16-bit fields (ERR, LSEP, DEVADDR) for H5 DRD.
 * From CMSIS: USB_CHEP_REG_MASK = ERRRX|ERRTX|LSEP|DEVADDR|VTRX|SETUP|UTYPE|KIND|VTTX|ADDR
 * Note: VTRX and VTTX are rc_w0 (write 1 to preserve), included here
 * so they can be selectively preserved/cleared in write helpers. */
#define USB_CHEP_RW_MASK    (USB_CHEP_UTYPE_MASK | USB_CHEP_EPKIND | USB_CHEP_EA_MASK)

/* Full register mask matching ST HAL — preserves all invariant bits including upper 16.
 * Includes: ERRRX(26)|ERRTX(25)|LSEP(24)|DEVADDR(22:16)|NAK(23)
 *           |VTRX(15)|SETUP(11)|UTYPE(10:9)|KIND(8)|VTTX(7)|ADDR(3:0)
 * = 0x07FF8F8F */
#define USB_CHEP_REG_MASK   0x07FF8F8FUL

/* ============================================================
 * USB_CNTR — Control register (offset 0x40)
 *
 * Interrupt mask and control bits. Bit positions differ from L4.
 * ============================================================ */

#define USB_CNTR            REG32(USB_DRD_BASE + 0x40UL)

#define USB_CNTR_HOST       (1UL << 31)   /* Host mode enable */
#define USB_CNTR_DDISCM     (1UL << 17)   /* Device disconnect mask (host) */
#define USB_CNTR_THR512M    (1UL << 16)   /* 512-byte threshold mask */
#define USB_CNTR_CTRM       (1UL << 15)   /* Correct transfer interrupt mask */
#define USB_CNTR_PMAOVRM    (1UL << 14)   /* PMA overrun interrupt mask */
#define USB_CNTR_ERRM       (1UL << 13)   /* Error interrupt mask */
#define USB_CNTR_WKUPM      (1UL << 12)   /* Wakeup interrupt mask */
#define USB_CNTR_SUSPM      (1UL << 11)   /* Suspend interrupt mask */
#define USB_CNTR_RESETM     (1UL << 10)   /* Reset interrupt mask (device) */
#define USB_CNTR_SOFM       (1UL << 9)    /* SOF interrupt mask */
#define USB_CNTR_ESOFM      (1UL << 8)    /* Expected SOF interrupt mask */
#define USB_CNTR_L1REQM     (1UL << 7)    /* LPM L1 request mask */
#define USB_CNTR_L1XACT     (1UL << 6)    /* Host LPM L1 transaction request */
#define USB_CNTR_L2RES      (1UL << 5)    /* L2 resume request */
#define USB_CNTR_SUSPEN     (1UL << 3)    /* Suspend enable */
#define USB_CNTR_SUSPRDY    (1UL << 2)    /* Suspend ready (ro) */
#define USB_CNTR_PDWN       (1UL << 1)    /* Power down */
#define USB_CNTR_USBRST     (1UL << 0)    /* Force USB reset */

/* ============================================================
 * USB_ISTR — Interrupt status register (offset 0x44)
 * ============================================================ */

#define USB_ISTR            REG32(USB_DRD_BASE + 0x44UL)

#define USB_ISTR_LS_DCON    (1UL << 30)   /* LS device connected (host, ro) */
#define USB_ISTR_DCON_STAT  (1UL << 29)   /* Device connection status (host, ro) */
#define USB_ISTR_DDISC      (1UL << 17)   /* Device disconnect (host) */
#define USB_ISTR_THR512     (1UL << 16)   /* 512-byte threshold */
#define USB_ISTR_CTR        (1UL << 15)   /* Correct transfer */
#define USB_ISTR_PMAOVR     (1UL << 14)   /* PMA overrun */
#define USB_ISTR_ERR        (1UL << 13)   /* Error */
#define USB_ISTR_WKUP       (1UL << 12)   /* Wakeup */
#define USB_ISTR_SUSP       (1UL << 11)   /* Suspend */
#define USB_ISTR_RESET      (1UL << 10)   /* USB reset (device) */
#define USB_ISTR_SOF        (1UL << 9)    /* Start of frame */
#define USB_ISTR_ESOF       (1UL << 8)    /* Expected SOF */
#define USB_ISTR_L1REQ      (1UL << 7)    /* LPM L1 request */
#define USB_ISTR_DIR        (1UL << 4)    /* Direction (0=IN/TX, 1=OUT/RX) */
#define USB_ISTR_IDN_MASK   0x0FUL        /* Endpoint ID [3:0] */

/* ============================================================
 * USB_FNR — Frame number register (offset 0x48, read-only)
 * ============================================================ */

#define USB_FNR             REG32(USB_DRD_BASE + 0x48UL)

/* ============================================================
 * USB_DADDR — Device address register (offset 0x4C)
 * ============================================================ */

#define USB_DADDR           REG32(USB_DRD_BASE + 0x4CUL)

#define USB_DADDR_EF        (1UL << 7)    /* Enable function */
#define USB_DADDR_ADD_MASK  0x7FUL        /* Device address [6:0] */

/* ============================================================
 * USB_LPMCSR — LPM control/status (offset 0x54)
 * ============================================================ */

#define USB_LPMCSR          REG32(USB_DRD_BASE + 0x54UL)

/* ============================================================
 * USB_BCDR — Battery charging detector (offset 0x58)
 * ============================================================ */

#define USB_BCDR            REG32(USB_DRD_BASE + 0x58UL)

#define USB_BCDR_DPPU_DPD   (1UL << 15)   /* DP pull-up (device) / pull-down (host), §55.6.6 */

/* ============================================================
 * CHEP register helpers
 *
 * Toggle/rc_w0 semantics identical to L4:
 *   - VTRX, VTTX are rc_w0: write 1 to keep, write 0 to clear
 *   - STATTX, STATRX, DTOGTX, DTOGRX are toggle: write 1 to flip
 *   - UTYPE, EPKIND, EA are normal rw
 *
 * All functions use 32-bit access. Upper 16 bits (host fields)
 * are written as 0 — safe for device mode.
 * ============================================================ */

/**
 * Read a CHEP register.
 */
static inline uint32_t ll_usb_drd_chep_read(uint8_t ep)
{
    return USB_CHEPnR(ep);
}

/* ll_usb_drd_chep_write is replaced by direct ST HAL-style manipulation.
 * All helpers below use USB_CHEP_REG_MASK to preserve invariant bits. */

/**
 * Clear VTRX flag (write 0 to clear, preserve VTTX and all REG_MASK bits).
 */
static inline void ll_usb_drd_chep_clr_vtrx(uint8_t ep)
{
    uint32_t reg = USB_CHEPnR(ep);
    USB_CHEPnR(ep) = (reg & USB_CHEP_REG_MASK & ~USB_CHEP_VTRX) | USB_CHEP_VTTX;
}

/**
 * Clear VTTX flag (preserve VTRX and all REG_MASK bits).
 */
static inline void ll_usb_drd_chep_clr_vttx(uint8_t ep)
{
    uint32_t reg = USB_CHEPnR(ep);
    USB_CHEPnR(ep) = (reg & USB_CHEP_REG_MASK & ~USB_CHEP_VTTX) | USB_CHEP_VTRX;
}

/**
 * Set STATTX to a specific value (0=DIS, 1=STALL, 2=NAK, 3=VALID).
 * Uses XOR toggle. Preserves all REG_MASK bits + VTRX + VTTX.
 */
static inline void ll_usb_drd_chep_set_stat_tx(uint8_t ep, uint32_t stat)
{
    uint32_t reg = USB_CHEPnR(ep) & (USB_CHEP_STATTX_MASK | USB_CHEP_REG_MASK);
    if (stat & 1) reg ^= (1UL << 4);   /* Toggle STATTX bit 0 */
    if (stat & 2) reg ^= (1UL << 5);   /* Toggle STATTX bit 1 */
    USB_CHEPnR(ep) = reg | USB_CHEP_VTRX | USB_CHEP_VTTX;
}

/**
 * Set STATRX to a specific value.
 */
static inline void ll_usb_drd_chep_set_stat_rx(uint8_t ep, uint32_t stat)
{
    uint32_t reg = USB_CHEPnR(ep) & (USB_CHEP_STATRX_MASK | USB_CHEP_REG_MASK);
    if (stat & 1) reg ^= (1UL << 12);  /* Toggle STATRX bit 0 */
    if (stat & 2) reg ^= (1UL << 13);  /* Toggle STATRX bit 1 */
    USB_CHEPnR(ep) = reg | USB_CHEP_VTRX | USB_CHEP_VTTX;
}

/**
 * Set both STATTX and STATRX at once.
 */
static inline void ll_usb_drd_chep_set_stat(uint8_t ep, uint32_t stat_tx, uint32_t stat_rx)
{
    uint32_t reg = USB_CHEPnR(ep) & (USB_CHEP_STATTX_MASK | USB_CHEP_STATRX_MASK | USB_CHEP_REG_MASK);
    if (stat_tx & 1) reg ^= (1UL << 4);
    if (stat_tx & 2) reg ^= (1UL << 5);
    if (stat_rx & 1) reg ^= (1UL << 12);
    if (stat_rx & 2) reg ^= (1UL << 13);
    USB_CHEPnR(ep) = reg | USB_CHEP_VTRX | USB_CHEP_VTTX;
}

/** Current STATTX / STATRX (USB_CHEP_STAT_*). The hardware moves VALID to NAK
 *  when a transfer completes, so these say whether a packet is still pending. */
static inline uint32_t ll_usb_drd_chep_stat_tx(uint8_t ep)
{
    return (USB_CHEPnR(ep) & USB_CHEP_STATTX_MASK) >> 4;
}

static inline uint32_t ll_usb_drd_chep_stat_rx(uint8_t ep)
{
    return (USB_CHEPnR(ep) & USB_CHEP_STATRX_MASK) >> 12;
}

/**
 * Clear data toggle TX (force DATA0).
 */
static inline void ll_usb_drd_chep_clr_dtog_tx(uint8_t ep)
{
    uint32_t reg = USB_CHEPnR(ep);
    if (reg & USB_CHEP_DTOGTX) {
        reg = (reg & USB_CHEP_REG_MASK) | USB_CHEP_DTOGTX | USB_CHEP_VTRX | USB_CHEP_VTTX;
        USB_CHEPnR(ep) = reg;
    }
}

/**
 * Clear data toggle RX (force DATA0).
 */
static inline void ll_usb_drd_chep_clr_dtog_rx(uint8_t ep)
{
    uint32_t reg = USB_CHEPnR(ep);
    if (reg & USB_CHEP_DTOGRX) {
        reg = (reg & USB_CHEP_REG_MASK) | USB_CHEP_DTOGRX | USB_CHEP_VTRX | USB_CHEP_VTTX;
        USB_CHEPnR(ep) = reg;
    }
}

/* ============================================================
 * PMA (Packet Memory Area / USBSRAM) access — 32-bit
 *
 * PMA is 2048 bytes at USB_DRD_PMA_BASE.
 * 32-bit word access, byte-addressed offsets.
 *
 * BDT (Buffer Descriptor Table) lives at the start of PMA.
 * Each endpoint has 2 x 32-bit descriptors = 8 bytes:
 *
 *   TXRXBD at ep*8 + 0:
 *     [31:28] Reserved
 *     [27:16] COUNT_TX[9:0]  (only [25:16] for count, [27:26] reserved)
 *     [15:0]  ADDR_TX
 *
 *   RXTXBD at ep*8 + 4:
 *     [31]    BLSIZE
 *     [30:26] NUM_BLOCK[4:0]
 *     [25:16] COUNT_RX[9:0]
 *     [15:0]  ADDR_RX
 *
 * Buffer data starts after the BDT (8 endpoints x 8 bytes = 64).
 * ============================================================ */

#define USB_DRD_BDT_ENTRY_SIZE  8   /* bytes per endpoint in BDT */

/**
 * Read a 32-bit word from PMA.
 */
static inline uint32_t ll_usb_drd_pma_rd32(uint32_t offset)
{
    return *(volatile uint32_t *)(USB_DRD_PMA_BASE + offset);
}

/**
 * Write a 32-bit word to PMA.
 */
static inline void ll_usb_drd_pma_wr32(uint32_t offset, uint32_t val)
{
    *(volatile uint32_t *)(USB_DRD_PMA_BASE + offset) = val;
}

/**
 * Copy data from user buffer to PMA.
 *   pma_offset: byte offset in PMA (should be 4-byte aligned)
 *   buf:        source data
 *   len:        number of bytes to copy
 */
static inline void ll_usb_drd_pma_write(uint32_t pma_offset, const uint8_t *buf, uint16_t len)
{
    volatile uint32_t *pma = (volatile uint32_t *)(USB_DRD_PMA_BASE + pma_offset);
    uint16_t i;

    /* Copy 4 bytes at a time */
    for (i = 0; i + 3 < len; i += 4) {
        *pma++ = (uint32_t)buf[i]
               | ((uint32_t)buf[i + 1] << 8)
               | ((uint32_t)buf[i + 2] << 16)
               | ((uint32_t)buf[i + 3] << 24);
    }
    /* Handle trailing 1–3 bytes */
    if (i < len) {
        uint32_t w = 0;
        for (uint16_t j = 0; j < (len - i); j++)
            w |= (uint32_t)buf[i + j] << (j * 8);
        *pma = w;
    }
}

/**
 * Copy data from PMA to user buffer.
 */
static inline void ll_usb_drd_pma_read(uint32_t pma_offset, uint8_t *buf, uint16_t len)
{
    volatile uint32_t *pma = (volatile uint32_t *)(USB_DRD_PMA_BASE + pma_offset);
    uint16_t i;

    for (i = 0; i + 3 < len; i += 4) {
        uint32_t w = *pma++;
        buf[i]     = (uint8_t)(w & 0xFF);
        buf[i + 1] = (uint8_t)((w >> 8) & 0xFF);
        buf[i + 2] = (uint8_t)((w >> 16) & 0xFF);
        buf[i + 3] = (uint8_t)((w >> 24) & 0xFF);
    }
    if (i < len) {
        uint32_t w = *pma;
        for (uint16_t j = 0; j < (len - i); j++)
            buf[i + j] = (uint8_t)((w >> (j * 8)) & 0xFF);
    }
}

/* ============================================================
 * BDT (Buffer Descriptor Table) helpers
 *
 * TX descriptor at PMA offset ep*8:
 *   [27:16] COUNT_TX, [15:0] ADDR_TX
 *
 * RX descriptor at PMA offset ep*8 + 4:
 *   [31] BLSIZE, [30:26] NUM_BLOCK, [25:16] COUNT_RX, [15:0] ADDR_RX
 * ============================================================ */

/** Set TX buffer address and clear count for an endpoint */
static inline void ll_usb_drd_bdt_set_tx(uint8_t ep, uint16_t pma_addr, uint16_t count)
{
    uint32_t bdt_offset = USB_DRD_BDT_ENTRY_SIZE * ep;
    ll_usb_drd_pma_wr32(bdt_offset, ((uint32_t)(count & 0x3FF) << 16) | (pma_addr & 0xFFFF));
}

/** Set TX buffer address */
static inline void ll_usb_drd_bdt_set_tx_addr(uint8_t ep, uint16_t pma_addr)
{
    uint32_t bdt_offset = USB_DRD_BDT_ENTRY_SIZE * ep;
    uint32_t val = ll_usb_drd_pma_rd32(bdt_offset);
    val = (val & 0xFFFF0000UL) | (pma_addr & 0xFFFF);
    ll_usb_drd_pma_wr32(bdt_offset, val);
}

/** Set TX byte count */
static inline void ll_usb_drd_bdt_set_tx_count(uint8_t ep, uint16_t count)
{
    uint32_t bdt_offset = USB_DRD_BDT_ENTRY_SIZE * ep;
    uint32_t val = ll_usb_drd_pma_rd32(bdt_offset);
    val = (val & 0x0000FFFFUL) | ((uint32_t)(count & 0x3FF) << 16);
    ll_usb_drd_pma_wr32(bdt_offset, val);
}

/** Set RX buffer address and allocate size (RM0481 §55.7.3, Table 622:
 *  BLSIZE = 0 allocates NUM_BLOCK x 2 bytes, BLSIZE = 1 (NUM_BLOCK + 1) x 32
 *  bytes). This wrote max/32 with BLSIZE = 1, which allocated 96 bytes for a
 *  64-byte buffer, so a babbling host could overrun into the next buffer. */
static inline void ll_usb_drd_bdt_set_rx(uint8_t ep, uint16_t pma_addr, uint16_t max_bytes)
{
    uint32_t bdt_offset = USB_DRD_BDT_ENTRY_SIZE * ep + 4;
    uint32_t alloc;

    if (max_bytes <= 62) {
        alloc = (uint32_t)(max_bytes / 2) << 26;
    } else {
        alloc = ((uint32_t)(max_bytes / 32 - 1) << 26) | (1UL << 31);
    }
    ll_usb_drd_pma_wr32(bdt_offset, alloc | (pma_addr & 0xFFFC));
}

/**
 * Get the number of bytes received (COUNT_RX [25:16]).
 *
 * The descriptor is written back a little after VTRX is set: read at once
 * from a fast interrupt, COUNT_RX is still short. Seen on the bench
 * (2026-09-26, 248 MHz): 11-byte packets arrived as 7, 19 as 11, the tail of
 * every command line lost. ST's own H5 driver waits "a few cycles for RX PMA
 * descriptor to update" (PCD_GET_EP_RX_CNT, PCD_RX_PMA_CNT = 10 loops in
 * STM32CubeH5 stm32h5xx_hal_pcd.h). This waits longer (~1 us at 250 MHz),
 * then re-reads until two reads agree.
 */
static inline uint16_t ll_usb_drd_bdt_get_rx_count(uint8_t ep)
{
    uint32_t bdt_offset = USB_DRD_BDT_ENTRY_SIZE * ep + 4;
    for (volatile uint32_t i = 0; i < 64; i++)
        ;
    uint32_t a = ll_usb_drd_pma_rd32(bdt_offset), b;
    for (int n = 0; n < 8 && (b = ll_usb_drd_pma_rd32(bdt_offset)) != a; n++)
        a = b;
    return (uint16_t)((a >> 16) & 0x3FF);
}

/* ============================================================
 * USB peripheral init/deinit helpers
 * ============================================================ */

/**
 * Power-on and reset the USB peripheral.
 * Call after enabling the USB clock.
 */
static inline void ll_usb_drd_power_on(void)
{
    /* RM0481 §55.5.2: clear PDWN (CNTR resets to PDWN | USBRST, §55.6.1),
     * wait tSTARTUP (1 us max, DS14540 Table 112), release USBRST,
     * clear ISTR. ~2000 loop iterations is >= 8 us at 250 MHz. */
    USB_CNTR = USB_CNTR_USBRST;        /* PDWN=0, USBRST=1 */
    for (volatile int i = 0; i < 2000; i++)
        ;
    USB_CNTR = 0;                       /* USBRST=0, HOST=0 */
    USB_ISTR = 0;
}

/**
 * Enable the DP pull-up resistor (signals device connection to host).
 */
static inline void ll_usb_drd_connect(void)
{
    SET_BITS(USB_BCDR, USB_BCDR_DPPU_DPD);
}

/**
 * Disable the DP pull-up (disconnect from host).
 */
static inline void ll_usb_drd_disconnect(void)
{
    CLR_BITS(USB_BCDR, USB_BCDR_DPPU_DPD);
}

/**
 * Set the USB device address and enable function.
 */
static inline void ll_usb_drd_set_address(uint8_t addr)
{
    USB_DADDR = USB_DADDR_EF | (addr & USB_DADDR_ADD_MASK);
}

/**
 * Configure an endpoint: type and address, EPKIND clear, both data toggles
 * DATA0 and both directions NAK, in one write; stale VTRX/VTTX are cleared
 * (written 0) and the host-mode upper half is written 0. A USB reset zeroes
 * the toggles, SET_CONFIGURATION without one does not, and RM0481 §55.6.7
 * makes initializing DTOG mandatory for non-control endpoints. (This used to
 * leave the toggles as they were: a re-configure, or an app started after
 * the ROM bootloader, could lose the first packet as a "duplicate".)
 */
static inline void ll_usb_drd_chep_config(uint8_t ep, uint32_t type, uint8_t addr)
{
    uint32_t reg = USB_CHEPnR(ep);
    USB_CHEPnR(ep) = type | (addr & USB_CHEP_EA_MASK)
                   | (reg & (USB_CHEP_DTOGRX | USB_CHEP_DTOGTX))
                   | ((reg ^ (USB_CHEP_STAT_NAK << 12)) & USB_CHEP_STATRX_MASK)
                   | ((reg ^ (USB_CHEP_STAT_NAK << 4)) & USB_CHEP_STATTX_MASK);
}

#endif /* STM32H523xx */

#endif /* LL_USB_DRD_H */
