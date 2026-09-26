/**
 * ll_spi.h — Low-level SPI operations
 *
 * Polling master-mode SPI. The SPI IP differs between families:
 *   - L0/L4: "old" SPI (single DR register, SR-based status, 32-bit FIFOs)
 *     — RM0394 §41 on the L4.
 *   - WBA/H5: "new" SPI v2 (separate TXDR/RXDR, TSIZE data counter,
 *     CSTART / EOT) — RM0493 §41 on the WBA.
 *
 * This driver abstracts both behind a common inline API. Every wait is
 * bounded: the multi-byte transfer (ll_spi_xfer_poll) fails with
 * LL_SPI_TIMEOUT when no frame completes for a "stall" budget derived from
 * the configured baud rate, so a stuck bus returns instead of hanging until
 * the watchdog.
 */

#ifndef LL_SPI_H
#define LL_SPI_H

#include "ll_common.h"
#include "ll_systick.h"   /* SysTick registers: the cycle clock for bounded waits */

/* ============================================================
 * Register structures (two different IPs)
 * ============================================================ */

#if defined(STM32L011xx) || defined(STM32L422xx)

/* ---- Old SPI IP (L0, L4) ---- */

typedef struct {
    volatile uint32_t CR1;      /* 0x00: Control register 1 */
    volatile uint32_t CR2;      /* 0x04: Control register 2 */
    volatile uint32_t SR;       /* 0x08: Status register */
    volatile uint32_t DR;       /* 0x0C: Data register */
    volatile uint32_t CRCPR;    /* 0x10: CRC polynomial */
    volatile uint32_t RXCRCR;   /* 0x14: RX CRC */
    volatile uint32_t TXCRCR;   /* 0x18: TX CRC */
} SPI_TypeDef;

/* CR1 bits (RM0394 §41.6.1) */
#define LL_SPI_CR1_CPHA         (1UL << 0)
#define LL_SPI_CR1_CPOL         (1UL << 1)
#define LL_SPI_CR1_MSTR         (1UL << 2)
#define LL_SPI_CR1_BR_SHIFT     3            /* BR[2:0] = prescaler */
#define LL_SPI_CR1_BR_MASK      (0x7UL << 3)
#define LL_SPI_CR1_SPE          (1UL << 6)
#define LL_SPI_CR1_LSBFIRST     (1UL << 7)
#define LL_SPI_CR1_SSI          (1UL << 8)
#define LL_SPI_CR1_SSM          (1UL << 9)

/* CR2 bits (RM0394 §41.6.2) */
#define LL_SPI_CR2_RXDMAEN     (1UL << 0)   /* RX DMA enable */
#define LL_SPI_CR2_TXDMAEN     (1UL << 1)   /* TX DMA enable */
#define LL_SPI_CR2_DS_SHIFT     8            /* DS[3:0] data size */
#define LL_SPI_CR2_FRXTH        (1UL << 12)  /* FIFO threshold: 1=8-bit */

/* SR bits (RM0394 §41.6.3) */
#define LL_SPI_SR_RXNE          (1UL << 0)
#define LL_SPI_SR_TXE           (1UL << 1)
#define LL_SPI_SR_MODF          (1UL << 5)
#define LL_SPI_SR_OVR           (1UL << 6)
#define LL_SPI_SR_BSY           (1UL << 7)
#define LL_SPI_SR_FRLVL_MASK    (0x3UL << 9)  /* RX FIFO level */
#define LL_SPI_SR_FTLVL_MASK    (0x3UL << 11) /* TX FIFO level */

#elif defined(STM32WBA55xx) || defined(STM32H523xx)

/* ---- New SPI v2 IP (WBA, H5) ---- */

typedef struct {
    volatile uint32_t CR1;      /* 0x00: Control register 1 */
    volatile uint32_t CR2;      /* 0x04: Control register 2 */
    volatile uint32_t CFG1;     /* 0x08: Configuration 1 */
    volatile uint32_t CFG2;     /* 0x0C: Configuration 2 */
    volatile uint32_t IER;      /* 0x10: Interrupt enable */
    volatile uint32_t SR;       /* 0x14: Status register */
    volatile uint32_t IFCR;     /* 0x18: Interrupt flag clear */
    volatile uint32_t _RESERVED0; /* 0x1C */
    volatile uint32_t TXDR;     /* 0x20: Transmit data */
    volatile uint32_t _RESERVED1[3];
    volatile uint32_t RXDR;     /* 0x30: Receive data */
    volatile uint32_t _RESERVED2[3];
    volatile uint32_t CRCPOLY;  /* 0x40: CRC polynomial */
    volatile uint32_t TXCRCR;   /* 0x44: TX CRC */
    volatile uint32_t RXCRCR;   /* 0x48: RX CRC */
    volatile uint32_t UDRDR;    /* 0x4C: Underrun data */
} SPI_TypeDef;

/* CR1 bits (RM0493 §41.8.1) */
#define LL_SPI_CR1_SPE          (1UL << 0)
#define LL_SPI_CR1_MASRX        (1UL << 8)   /* Master auto-suspend (RX) */
#define LL_SPI_CR1_CSTART       (1UL << 9)   /* Master transfer start */
#define LL_SPI_CR1_CSUSP        (1UL << 10)  /* Master suspend request */
#define LL_SPI_CR1_HDDIR        (1UL << 11)  /* Half-duplex dir: 1=Tx, 0=Rx */
#define LL_SPI_CR1_SSI          (1UL << 12)  /* Internal SS level */

/* CR2 bits (RM0493 §41.8.2) */
#define LL_SPI_CR2_TSIZE_SHIFT  0            /* TSIZE[15:0]: frame count (0=endless) */

/* CFG1 bits (RM0493 §41.8.3). Write-protected while SPE=1, except the DMA enables. */
#define LL_SPI_CFG1_RXDMAEN    (1UL << 14)  /* RX DMA enable */
#define LL_SPI_CFG1_TXDMAEN    (1UL << 15)  /* TX DMA enable */
#define LL_SPI_CFG1_DSIZE_SHIFT 0            /* DSIZE[4:0] */
#define LL_SPI_CFG1_FTHLV_SHIFT 5            /* FTHLV[3:0]: frames per packet - 1 */
#define LL_SPI_CFG1_MBR_SHIFT   28           /* MBR[2:0] prescaler */
#define LL_SPI_CFG1_MBR_MASK    (0x7UL << 28)

/* CFG2 bits (RM0493 §41.8.4; the same layout in RM0481 on the H5) */
#define LL_SPI_CFG2_CPHA        (1UL << 24)
#define LL_SPI_CFG2_CPOL        (1UL << 25)
#define LL_SPI_CFG2_MASTER      (1UL << 22)
#define LL_SPI_CFG2_LSBFRST     (1UL << 23)
#define LL_SPI_CFG2_SSM         (1UL << 26)
#define LL_SPI_CFG2_SSOE        (1UL << 29)  /* SS output enable */
#define LL_SPI_CFG2_SSOM        (1UL << 30)  /* SS output management in master mode */
#define LL_SPI_CFG2_AFCNTR      (1UL << 31)
/* COMM[18:17]: communication mode */
#define LL_SPI_CFG2_COMM_SHIFT  17
#define LL_SPI_CFG2_COMM_FULL   (0x0UL << 17)  /* full-duplex */
#define LL_SPI_CFG2_COMM_TX     (0x1UL << 17)  /* simplex transmitter */
#define LL_SPI_CFG2_COMM_RX     (0x2UL << 17)  /* simplex receiver */
#define LL_SPI_CFG2_COMM_HALF   (0x3UL << 17)  /* half-duplex (1-line, bidir) */

/* IER bits */
#define LL_SPI_IER_EOTIE        (1UL << 3)   /* EOT / SUSP / TXC interrupt enable */

/* SR bits (RM0493 §41.8.6) */
#define LL_SPI_SR_RXP           (1UL << 0)   /* RX packet available */
#define LL_SPI_SR_TXP           (1UL << 1)   /* TX packet space available */
#define LL_SPI_SR_EOT           (1UL << 3)   /* End of transfer */
#define LL_SPI_SR_TXTF          (1UL << 4)   /* All TSIZE frames pushed to the TxFIFO */
#define LL_SPI_SR_OVR           (1UL << 6)   /* Overrun */
#define LL_SPI_SR_MODF          (1UL << 9)   /* Mode fault */
#define LL_SPI_SR_SUSP          (1UL << 11)  /* Master suspended */
#define LL_SPI_SR_TXC           (1UL << 12)  /* TX complete (FIFO empty + idle) */
#define LL_SPI_SR_RXPLVL_MASK   (0x3UL << 13) /* frames left in RxFIFO (< 4 bytes) */
#define LL_SPI_SR_RXWNE         (1UL << 15)  /* >= 4 bytes in RxFIFO */

/* IFCR bits (RM0493 §41.8.7) */
#define LL_SPI_IFCR_EOTC        (1UL << 3)
#define LL_SPI_IFCR_TXTFC       (1UL << 4)
#define LL_SPI_IFCR_UDRC        (1UL << 5)
#define LL_SPI_IFCR_OVRC        (1UL << 6)
#define LL_SPI_IFCR_MODFC       (1UL << 9)
#define LL_SPI_IFCR_SUSPC       (1UL << 11)
#define LL_SPI_IFCR_ALL         (0xBF8UL)    /* every clearable flag, bits 3-9 and 11 */

#endif /* SPI IP version */

/* ---- Instance base addresses ---- */

#if defined(STM32L011xx)
  #define SPI1      ((SPI_TypeDef *)0x40013000UL)

#elif defined(STM32L422xx)
  /* The L422 has SPI1 and SPI2 only (RM0394 Table 1); the Cores route SPI1. */
  #define SPI1      ((SPI_TypeDef *)0x40013000UL)

#elif defined(STM32WBA55xx)
  #define SPI1      ((SPI_TypeDef *)0x40013000UL)
  #define SPI3      ((SPI_TypeDef *)0x46002000UL)

#elif defined(STM32H523xx)
  #define SPI1      ((SPI_TypeDef *)0x40013000UL)
  #define SPI2      ((SPI_TypeDef *)0x40003800UL)
  #define SPI3      ((SPI_TypeDef *)0x40003C00UL)
#endif

/* ---- Prescaler values ---- */
/* Divides the SPI kernel clock by 2, 4, 8, 16, 32, 64, 128, 256. The kernel
 * clock is PCLK2 on the L4 (SPI1), PCLK2 / PCLK7 on the WBA (SPI1 / SPI3
 * at reset, RCC_CCIPR1/3), which is SYSCLK at every coregen clock level. */

#define LL_SPI_PRESCALER_2      0
#define LL_SPI_PRESCALER_4      1
#define LL_SPI_PRESCALER_8      2
#define LL_SPI_PRESCALER_16     3
#define LL_SPI_PRESCALER_32     4
#define LL_SPI_PRESCALER_64     5
#define LL_SPI_PRESCALER_128    6
#define LL_SPI_PRESCALER_256    7

/* ---- Clock polarity / phase ---- */

#define LL_SPI_CPOL_LOW         0
#define LL_SPI_CPOL_HIGH        1
#define LL_SPI_CPHA_1EDGE       0   /* Data on first clock edge */
#define LL_SPI_CPHA_2EDGE       1   /* Data on second clock edge */

/* ---- Return codes of the bounded transfer ---- */

#define LL_SPI_OK               0
#define LL_SPI_TIMEOUT          (-1)   /* no frame completed within the stall budget */
#define LL_SPI_ERR              (-2)   /* overrun or mode fault */

/* ============================================================
 * Bounded waits (internal)
 *
 * A deadline counted in CPU cycles on the SysTick down-counter. It reads the
 * counter itself rather than the millisecond tick, so it keeps counting with
 * interrupts masked or inside an ISR. A poll budget equal to the cycle budget
 * backs it up in case SysTick is stopped: every poll costs at least one
 * cycle, so that bound can only be later, never earlier.
 * ============================================================ */

typedef struct {
    uint32_t last;    /* SysTick CVR at the previous poll */
    uint32_t left;    /* cycles still allowed */
    uint32_t spins;   /* polls still allowed */
} ll_spi_deadline_t;

/* CPU cycles per millisecond, from the SysTick reload ll_systick_init() set
 * (sysclk / 1000). 100000 (a 100 MHz part) before SysTick is configured. */
static inline uint32_t _ll_spi_cycles_per_ms(void)
{
    uint32_t rvr = SYSTICK_RVR & 0x00FFFFFFUL;
    return rvr ? rvr + 1UL : 100000UL;
}

static inline void _ll_spi_deadline_start(ll_spi_deadline_t *d, uint32_t cycles)
{
    d->last  = SYSTICK_CVR & 0x00FFFFFFUL;
    d->left  = cycles;
    d->spins = cycles;
}

static inline int _ll_spi_deadline_expired(ll_spi_deadline_t *d)
{
    if (d->spins == 0) return 1;
    d->spins--;
    uint32_t rvr = SYSTICK_RVR & 0x00FFFFFFUL;
    if (rvr) {
        uint32_t now = SYSTICK_CVR & 0x00FFFFFFUL;
        uint32_t el  = (d->last >= now) ? d->last - now : d->last + rvr + 1UL - now;
        d->last = now;
        if (el >= d->left) return 1;
        d->left -= el;
    }
    return 0;
}

/* ============================================================
 * Instance properties
 * ============================================================ */

/* Frames allowed "in flight" (written but not yet read back) by the polled
 * transfer. Keeping it at or below the RxFIFO size means the RxFIFO can
 * never overrun, however late the CPU gets back to it (an ISR, say).
 *   L4:  32-bit RxFIFO = 4 x 8-bit frames (RM0394 §41.4.9).
 *   WBA: SPI1 16 x 8 bits, SPI3 8 x 8 bits (RM0493 Table 394).
 *   H5:  8 (the smaller FIFO; not checked per instance on the H5).
 *   L0:  1 (no FIFO: a single RX buffer; no Core routes SPI on the L0). */
static inline uint32_t _ll_spi_fifo_frames(SPI_TypeDef *spi)
{
#if defined(STM32L011xx)
    (void)spi;
    return 1;
#elif defined(STM32L422xx)
    (void)spi;
    return 4;
#elif defined(STM32WBA55xx)
    return (spi == SPI1) ? 16 : 8;
#else
    (void)spi;
    return 8;
#endif
}

/* Largest TSIZE a single SPI v2 session can count (RM0493 §41.8.2: TSIZE[15:10]
 * are reserved on the limited-feature SPI3). Longer transfers are chunked;
 * the chip select is a GPIO, so it stays asserted across chunks. */
static inline uint32_t _ll_spi_max_frames(SPI_TypeDef *spi)
{
#if defined(STM32WBA55xx)
    return (spi == SPI1) ? 0xFFFFUL : 0x3FFUL;
#elif defined(STM32H523xx)
    (void)spi;
    return 0x3FFUL;
#else
    (void)spi;
    return 0xFFFFUL;
#endif
}

/* Prescaler exponent currently programmed: SCK = kernel / (2 << code). */
static inline uint32_t _ll_spi_prescaler_code(SPI_TypeDef *spi)
{
#if defined(STM32L011xx) || defined(STM32L422xx)
    return (spi->CR1 & LL_SPI_CR1_BR_MASK) >> LL_SPI_CR1_BR_SHIFT;
#else
    return (spi->CFG1 & LL_SPI_CFG1_MBR_MASK) >> LL_SPI_CFG1_MBR_SHIFT;
#endif
}

/**
 * @brief  Stall budget, in CPU cycles, for the SPI's current baud rate.
 *
 * A transfer fails with LL_SPI_TIMEOUT when no frame completes for this long:
 * 2 ms plus 4x the time to move a full FIFO of 8-bit frames (a margin that
 * also covers an APB divider of up to 4 between the CPU and the SPI kernel
 * clock). At /256 on a 16 MHz Core that is about 2.5 ms (L4) to 3 ms (WBA);
 * at /2 it is essentially 2 ms. A whole transfer is therefore bounded by
 * (frames x frame time) + this budget per stall, so it scales with length and
 * clock.
 *
 * @param  spi  SPI instance (configured)
 * @return Cycle budget
 */
static inline uint32_t ll_spi_stall_cycles(SPI_TypeDef *spi)
{
    uint32_t frame = 8UL * (2UL << _ll_spi_prescaler_code(spi));
    return 2UL * _ll_spi_cycles_per_ms() + frame * _ll_spi_fifo_frames(spi) * 4UL;
}

/* ============================================================
 * Configuration
 * ============================================================ */

/**
 * @brief  Configure SPI as an 8-bit master with software CS.
 *
 * Programs the whole configuration (prescaler, mode, bit order) with the
 * peripheral disabled. On the L4 the SPI is left enabled (SPE=1, idle until
 * data is written); on the WBA/H5 SPI v2 it is left enabled with TSIZE=0,
 * and each transfer reprograms TSIZE with SPE=0 as RM0493 §41.8.2 requires.
 * AFCNTR keeps the v2 outputs driven while SPE toggles, so SCK holds its idle
 * level between transfers.
 *
 * @param  spi        SPI instance
 * @param  prescaler  LL_SPI_PRESCALER_* (kernel clock divider)
 * @param  cpol       LL_SPI_CPOL_LOW or _HIGH
 * @param  cpha       LL_SPI_CPHA_1EDGE or _2EDGE
 * @param  lsb_first  0 = MSB first (the common case), 1 = LSB first
 */
static inline void ll_spi_config(SPI_TypeDef *spi, uint32_t prescaler,
                                 uint32_t cpol, uint32_t cpha, uint32_t lsb_first)
{
#if defined(STM32L011xx) || defined(STM32L422xx)
    /* Old SPI IP: RM0394 §41.4.7 order — CR1 (BR, CPOL/CPHA, LSBFIRST, SSM/SSI,
     * MSTR), then CR2 (DS, FRXTH), then SPE. */
    spi->CR1 = 0;

    spi->CR1 = LL_SPI_CR1_MSTR                       /* Master mode */
             | LL_SPI_CR1_SSM                         /* Software CS */
             | LL_SPI_CR1_SSI                         /* Internal CS high */
             | ((prescaler & 7UL) << LL_SPI_CR1_BR_SHIFT)
             | (cpol ? LL_SPI_CR1_CPOL : 0)
             | (cpha ? LL_SPI_CR1_CPHA : 0)
             | (lsb_first ? LL_SPI_CR1_LSBFIRST : 0);

    /* 8-bit frames; FRXTH=1 so RXNE fires per byte and DR is read byte-wide
     * (RM0394 §41.4.9: reads must match the FRXTH threshold). */
    spi->CR2 = (0x7UL << LL_SPI_CR2_DS_SHIFT)        /* DS = 0111 → 8-bit */
             | LL_SPI_CR2_FRXTH;

    spi->CR1 |= LL_SPI_CR1_SPE;                      /* Enable */

#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    /* New SPI v2 IP */
    spi->CR1 = 0;

    /* Raise SSI (internal SS) high BEFORE enabling MASTER: with software SS,
     * setting MASTER while SSI=0 makes the internal SS read low, which trips a
     * mode fault (MODF) that auto-clears MASTER and silently drops to slave. */
    spi->CR1 = LL_SPI_CR1_SSI;

    spi->CFG1 = (7UL << LL_SPI_CFG1_DSIZE_SHIFT)     /* 8-bit data */
              | (0UL << LL_SPI_CFG1_FTHLV_SHIFT)      /* 1 frame per packet */
              | ((prescaler & 7UL) << LL_SPI_CFG1_MBR_SHIFT);

    spi->CFG2 = LL_SPI_CFG2_MASTER                    /* Master mode */
              | LL_SPI_CFG2_SSM                        /* Software CS */
              | LL_SPI_CFG2_AFCNTR                     /* Keep AF control */
              | (cpol ? LL_SPI_CFG2_CPOL : 0)
              | (cpha ? LL_SPI_CFG2_CPHA : 0)
              | (lsb_first ? LL_SPI_CFG2_LSBFRST : 0);

    spi->IFCR = LL_SPI_IFCR_ALL;                       /* Clear any latched MODF etc. */
    spi->CR1 = LL_SPI_CR1_SSI | LL_SPI_CR1_SPE;        /* SSI + Enable */
#endif
}

/**
 * Initialize SPI in master mode, 8-bit, MSB-first.
 *   spi:       SPI instance
 *   prescaler: LL_SPI_PRESCALER_* (clock divider)
 *   cpol:      LL_SPI_CPOL_LOW or _HIGH
 *   cpha:      LL_SPI_CPHA_1EDGE or _2EDGE
 *
 * Prerequisites:
 *   - Peripheral clock enabled via ll_rcc_apb1/2_clk_enable()
 *   - SCK/MOSI/MISO pins configured for AF via ll_gpio_config_af()
 *   - CS managed manually via GPIO (not hardware NSS)
 */
static inline void ll_spi_init(SPI_TypeDef *spi, uint32_t prescaler,
                               uint32_t cpol, uint32_t cpha)
{
    ll_spi_config(spi, prescaler, cpol, cpha, 0);
}

/* ============================================================
 * Polling transfer (bounded)
 * ============================================================ */

#if defined(STM32WBA55xx) || defined(STM32H523xx)
/* Stop a v2 session cleanly and leave SPE=0 (RM0493 §41.4.13): a running
 * master is suspended first (CSUSP completes the current frame; the wait is
 * bounded to a few frame times), then SPE=0 flushes both FIFOs and restarts
 * the internal state machine. */
static inline void _ll_spi_v2_stop(SPI_TypeDef *spi)
{
    if ((spi->CR1 & LL_SPI_CR1_CSTART) && (spi->CR1 & LL_SPI_CR1_SPE)
        && (spi->CFG2 & LL_SPI_CFG2_MASTER)) {
        SET_BITS(spi->CR1, LL_SPI_CR1_CSUSP);
        ll_spi_deadline_t dl;
        _ll_spi_deadline_start(&dl, 32UL * (2UL << _ll_spi_prescaler_code(spi)) + 2000UL);
        while (!(spi->SR & LL_SPI_SR_SUSP) && !_ll_spi_deadline_expired(&dl)) { }
    }
    CLR_BITS(spi->CR1, LL_SPI_CR1_SPE);
    spi->IFCR = LL_SPI_IFCR_ALL;
}

/* One TSIZE session of n (1..max) frames. */
static inline int _ll_spi_v2_session(SPI_TypeDef *spi, const uint8_t *tx,
                                     uint8_t *rx, uint32_t n, uint8_t fill,
                                     uint32_t stall)
{
    const uint32_t depth = _ll_spi_fifo_frames(spi);
    ll_spi_deadline_t dl;

    CLR_BITS(spi->CR1, LL_SPI_CR1_SPE);          /* TSIZE writable only when off */
    spi->CR2 = n;
    spi->IFCR = LL_SPI_IFCR_ALL;
    SET_BITS(spi->CR1, LL_SPI_CR1_SPE);
    SET_BITS(spi->CR1, LL_SPI_CR1_CSTART);       /* clocks once the TxFIFO has data */

    uint32_t ti = 0, ri = 0;
    _ll_spi_deadline_start(&dl, stall);
    while (ri < n) {
        uint32_t sr = spi->SR;
        if (sr & (LL_SPI_SR_OVR | LL_SPI_SR_MODF)) { _ll_spi_v2_stop(spi); return LL_SPI_ERR; }
        if (ti < n && (sr & LL_SPI_SR_TXP) && (ti - ri) < depth) {
            *(volatile uint8_t *)&spi->TXDR = tx ? tx[ti] : fill;
            ti++;
        }
        /* RXP is not raised for the last packet of a TSIZE session
         * (RM0493 §41.4.12, "Hardware data counter"), so RXPLVL/RXWNE count too. */
        if (sr & (LL_SPI_SR_RXP | LL_SPI_SR_RXWNE | LL_SPI_SR_RXPLVL_MASK)) {
            uint8_t b = *(volatile uint8_t *)&spi->RXDR;
            if (rx) rx[ri] = b;
            ri++;
            _ll_spi_deadline_start(&dl, stall);
        } else if (_ll_spi_deadline_expired(&dl)) {
            _ll_spi_v2_stop(spi);
            return LL_SPI_TIMEOUT;
        }
    }

    /* Every frame is back, so EOT is at most a few kernel clocks away. */
    _ll_spi_deadline_start(&dl, stall);
    while (!(spi->SR & LL_SPI_SR_EOT)) {
        if (_ll_spi_deadline_expired(&dl)) { _ll_spi_v2_stop(spi); return LL_SPI_TIMEOUT; }
    }
    spi->IFCR = LL_SPI_IFCR_ALL;
    CLR_BITS(spi->CR1, LL_SPI_CR1_SPE);          /* restart the state machine (§41.4.12) */
    return LL_SPI_OK;
}
#endif

/**
 * @brief  Bounded full-duplex transfer of `len` bytes (polled, FIFO-pipelined).
 *
 * Sends `tx` (or `fill` for every byte when tx is NULL) while receiving into
 * `rx` (discarded when NULL). Up to a FIFO's worth of frames is kept in
 * flight, so SCK runs back to back; the in-flight count never exceeds the
 * RxFIFO, so an interrupt can delay the loop but never cause an overrun.
 * The chip select is not touched (it is a GPIO; see hal_spi_select).
 *
 * L4 (RM0394 §41.4.9): byte-wide DR access with FRXTH=1; the transfer ends
 * with the standard disable wait (FTLVL=0, then BSY=0) so CS can be released
 * right after. WBA/H5 (RM0493 §41.4.12): one TSIZE session per chunk of up
 * to 65535 frames (1023 on the WBA's SPI3), each closed on EOT with SPE=0.
 *
 * @param  spi     SPI instance (configured with ll_spi_config)
 * @param  tx      Bytes to send, or NULL to send `fill`
 * @param  rx      Buffer for received bytes, or NULL to discard (may equal tx)
 * @param  len     Number of bytes (0 is a no-op)
 * @param  fill    Byte sent when tx is NULL (0xFF is the usual idle level)
 * @param  stall   Stall budget in CPU cycles (ll_spi_stall_cycles(spi))
 * @return LL_SPI_OK, LL_SPI_TIMEOUT (no progress within `stall`), or
 *         LL_SPI_ERR (overrun / mode fault)
 */
static inline int ll_spi_xfer_poll(SPI_TypeDef *spi, const uint8_t *tx,
                                   uint8_t *rx, uint32_t len, uint8_t fill,
                                   uint32_t stall)
{
    if (len == 0) return LL_SPI_OK;

#if defined(STM32L011xx) || defined(STM32L422xx)
    const uint32_t depth = _ll_spi_fifo_frames(spi);
    ll_spi_deadline_t dl;

    if (!(spi->CR1 & LL_SPI_CR1_SPE)) SET_BITS(spi->CR1, LL_SPI_CR1_SPE);
    /* Drop stale bytes a previous caller left behind (at most a FIFO's worth). */
    for (uint32_t i = 0; i < 4 && (spi->SR & LL_SPI_SR_RXNE); i++)
        (void)*(volatile uint8_t *)&spi->DR;

    uint32_t ti = 0, ri = 0;
    _ll_spi_deadline_start(&dl, stall);
    while (ri < len) {
        uint32_t sr = spi->SR;
        if (sr & (LL_SPI_SR_OVR | LL_SPI_SR_MODF)) return LL_SPI_ERR;
        if (ti < len && (sr & LL_SPI_SR_TXE) && (ti - ri) < depth) {
            *(volatile uint8_t *)&spi->DR = tx ? tx[ti] : fill;
            ti++;
        }
        if (sr & LL_SPI_SR_RXNE) {
            uint8_t b = *(volatile uint8_t *)&spi->DR;
            if (rx) rx[ri] = b;
            ri++;
            _ll_spi_deadline_start(&dl, stall);
        } else if (_ll_spi_deadline_expired(&dl)) {
            return LL_SPI_TIMEOUT;
        }
    }

    /* RM0394 §41.4.9 "Procedure for disabling the SPI", steps 1-2: the last
     * frame is fully off the bus before the caller releases CS. */
    _ll_spi_deadline_start(&dl, stall);
    while (spi->SR & (LL_SPI_SR_FTLVL_MASK | LL_SPI_SR_BSY)) {
        if (_ll_spi_deadline_expired(&dl)) return LL_SPI_TIMEOUT;
    }
    return LL_SPI_OK;

#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    const uint32_t max = _ll_spi_max_frames(spi);
    while (len) {
        uint32_t n = len > max ? max : len;
        int rc = _ll_spi_v2_session(spi, tx, rx, n, fill, stall);
        if (rc != LL_SPI_OK) return rc;
        if (tx) tx += n;
        if (rx) rx += n;
        len -= n;
    }
    return LL_SPI_OK;
#else
    (void)spi; (void)tx; (void)rx; (void)fill; (void)stall;
    return LL_SPI_ERR;
#endif
}

/**
 * Transfer a single byte (full-duplex): send tx_data, return received byte.
 * Bounded: returns 0xFF if the transfer stalls (use ll_spi_xfer_poll to see
 * the error).
 */
static inline uint8_t ll_spi_transfer(SPI_TypeDef *spi, uint8_t tx_data)
{
    uint8_t rx = 0xFF;
    if (ll_spi_xfer_poll(spi, &tx_data, &rx, 1, 0xFF, ll_spi_stall_cycles(spi)) != LL_SPI_OK)
        return 0xFF;
    return rx;
}

/**
 * Transfer a buffer (full-duplex, in-place).
 * tx_buf is sent; received data overwrites tx_buf.
 * Pass NULL for tx_buf to clock out 0xFF and discard what comes back.
 */
static inline void ll_spi_transfer_buf(SPI_TypeDef *spi,
                                       uint8_t *buf, uint32_t len)
{
    (void)ll_spi_xfer_poll(spi, buf, buf, len, 0xFF, ll_spi_stall_cycles(spi));
}

/**
 * Write-only transfer (discard received data).
 */
static inline void ll_spi_write(SPI_TypeDef *spi, uint8_t data)
{
    (void)ll_spi_transfer(spi, data);
}

/**
 * Read-only transfer (send 0xFF, return received byte).
 */
static inline uint8_t ll_spi_read(SPI_TypeDef *spi)
{
    return ll_spi_transfer(spi, 0xFF);
}

/* ============================================================
 * Enable / Disable
 * ============================================================ */

static inline void ll_spi_enable(SPI_TypeDef *spi)
{
#if defined(STM32L011xx) || defined(STM32L422xx)
    SET_BITS(spi->CR1, LL_SPI_CR1_SPE);
#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    SET_BITS(spi->CR1, LL_SPI_CR1_SPE);
#endif
}

/**
 * Disable the SPI after the bus goes idle (bounded wait: FTLVL=0, BSY=0 on
 * the L4, RM0394 §41.4.9; a clean suspend-then-disable on SPI v2).
 */
static inline void ll_spi_disable(SPI_TypeDef *spi)
{
#if defined(STM32L011xx) || defined(STM32L422xx)
    ll_spi_deadline_t dl;
    _ll_spi_deadline_start(&dl, ll_spi_stall_cycles(spi));
    while ((spi->SR & (LL_SPI_SR_FTLVL_MASK | LL_SPI_SR_BSY)) && !_ll_spi_deadline_expired(&dl))
        ;
    CLR_BITS(spi->CR1, LL_SPI_CR1_SPE);
    for (uint32_t i = 0; i < 4 && (spi->SR & LL_SPI_SR_RXNE); i++)
        (void)*(volatile uint8_t *)&spi->DR;          /* step 4: empty the RxFIFO */
#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    _ll_spi_v2_stop(spi);
#endif
}

/* ============================================================
 * Busy check
 * ============================================================ */

static inline int ll_spi_busy(SPI_TypeDef *spi)
{
#if defined(STM32L011xx) || defined(STM32L422xx)
    return (spi->SR & LL_SPI_SR_BSY) != 0;
#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    return !(spi->SR & LL_SPI_SR_EOT);
#endif
}

/* ============================================================
 * DMA enable / disable
 * ============================================================ */

/** Enable DMA requests for RX */
static inline void ll_spi_enable_dma_rx(SPI_TypeDef *spi)
{
#if defined(STM32L011xx) || defined(STM32L422xx)
    SET_BITS(spi->CR2, LL_SPI_CR2_RXDMAEN);
#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    SET_BITS(spi->CFG1, LL_SPI_CFG1_RXDMAEN);
#endif
}

/** Disable DMA requests for RX */
static inline void ll_spi_disable_dma_rx(SPI_TypeDef *spi)
{
#if defined(STM32L011xx) || defined(STM32L422xx)
    CLR_BITS(spi->CR2, LL_SPI_CR2_RXDMAEN);
#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    CLR_BITS(spi->CFG1, LL_SPI_CFG1_RXDMAEN);
#endif
}

/** Enable DMA requests for TX */
static inline void ll_spi_enable_dma_tx(SPI_TypeDef *spi)
{
#if defined(STM32L011xx) || defined(STM32L422xx)
    SET_BITS(spi->CR2, LL_SPI_CR2_TXDMAEN);
#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    SET_BITS(spi->CFG1, LL_SPI_CFG1_TXDMAEN);
#endif
}

/** Disable DMA requests for TX */
static inline void ll_spi_disable_dma_tx(SPI_TypeDef *spi)
{
#if defined(STM32L011xx) || defined(STM32L422xx)
    CLR_BITS(spi->CR2, LL_SPI_CR2_TXDMAEN);
#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    CLR_BITS(spi->CFG1, LL_SPI_CFG1_TXDMAEN);
#endif
}

/* ============================================================
 * Half-duplex (1-line bidirectional) — SPI v2 only (WBA / H5)
 *
 * One data line (MOSI) carries both directions; HDDIR selects Tx vs Rx. This
 * is what the NanEyeC SEIM interface needs (SCLK on SCK, SDAT on MOSI). AFCNTR
 * is kept set so the AF pins don't glitch while SPE/HDDIR are toggled — which
 * is exactly what direction flips between transfers require.
 *
 * SPE is left disabled by init: on SPI v2, TSIZE must be programmed while
 * SPE=0, so each transfer (or DMA arm) sets TSIZE then enables SPE itself.
 * ============================================================ */
#if defined(STM32WBA55xx) || defined(STM32H523xx)

/**
 * Initialize SPI v2 as a half-duplex (1-line) master, 8-bit, MSB-first,
 * starting in transmit direction. Leaves SPE disabled.
 * Pin/clock prerequisites are the same as ll_spi_init().
 */
static inline void ll_spi_init_halfduplex(SPI_TypeDef *spi, uint32_t prescaler,
                                          uint32_t cpol, uint32_t cpha)
{
    /* Raise SSI (internal SS) high BEFORE enabling MASTER: with software SS,
     * setting MASTER while SSI=0 makes the internal SS read low, which trips a
     * mode fault (MODF) that auto-clears MASTER. Start as transmitter (HDDIR). */
    spi->CR1  = LL_SPI_CR1_SSI | LL_SPI_CR1_HDDIR;

    spi->CFG1 = (7UL << LL_SPI_CFG1_DSIZE_SHIFT)     /* 8-bit data */
              | (prescaler << LL_SPI_CFG1_MBR_SHIFT);

    spi->CFG2 = LL_SPI_CFG2_MASTER
              | LL_SPI_CFG2_SSM                       /* software CS */
              | LL_SPI_CFG2_AFCNTR                    /* hold AF pins across SPE toggles */
              | LL_SPI_CFG2_COMM_HALF                 /* 1-line bidirectional */
              | (cpol ? LL_SPI_CFG2_CPOL : 0)
              | (cpha ? LL_SPI_CFG2_CPHA : 0);

    spi->IFCR = (1UL << 9);                            /* clear any latched MODF */
    /* SPE left disabled: TSIZE must be programmed per-transfer while SPE=0. */
}

/**
 * Select half-duplex data direction (1 = transmit, 0 = receive).
 * Must be called with SPE=0 (HDDIR is config-protected while enabled).
 */
static inline void ll_spi_halfduplex_set_dir(SPI_TypeDef *spi, int tx)
{
    if (tx) SET_BITS(spi->CR1, LL_SPI_CR1_HDDIR);
    else    CLR_BITS(spi->CR1, LL_SPI_CR1_HDDIR);
}

/**
 * Polled half-duplex master transmit of `len` bytes. Programs TSIZE (with SPE
 * disabled), enables the peripheral, starts the master clock, feeds the FIFO,
 * and waits for end-of-transfer. Leaves SPE disabled so the next call can
 * reprogram TSIZE. Returns 1 on success, 0 on overrun/timeout.
 */
static inline int ll_spi_halfduplex_write(SPI_TypeDef *spi,
                                          const uint8_t *data, uint32_t len)
{
    if (len == 0) return 1;

    CLR_BITS(spi->CR1, LL_SPI_CR1_SPE);               /* TSIZE writable only when off */
    SET_BITS(spi->CR1, LL_SPI_CR1_HDDIR);             /* transmitter */
    spi->CR2 = (len & 0xFFFFUL);                       /* TSIZE = frame count */
    SET_BITS(spi->CR1, LL_SPI_CR1_SPE);
    SET_BITS(spi->CR1, LL_SPI_CR1_CSTART);            /* begin clocking */

    for (uint32_t i = 0; i < len; i++) {
        uint32_t guard = 1000000;
        while (!(spi->SR & LL_SPI_SR_TXP) && --guard) { }
        if (guard == 0) { CLR_BITS(spi->CR1, LL_SPI_CR1_SPE); return 0; }
        *(volatile uint8_t *)&spi->TXDR = data[i];
    }

    uint32_t guard = 1000000;
    while (!(spi->SR & LL_SPI_SR_EOT) && --guard) { }
    int ok = (guard != 0) && !(spi->SR & LL_SPI_SR_OVR);

    spi->IFCR = LL_SPI_IFCR_EOTC | LL_SPI_IFCR_TXTFC | LL_SPI_IFCR_OVRC;
    CLR_BITS(spi->CR1, LL_SPI_CR1_SPE);
    return ok;
}

/* ---- DMA-driven master transfers (SPI v2) ----
 * The GPDMA channel must be configured AND enabled to feed TXDR / drain RXDR
 * (CTR2 REQSEL = spi<n>_tx / spi<n>_rx with DREQ for TX, hardware request)
 * BEFORE calling start — the channel then waits for the SPI's TXP/RXP requests.
 * tsize = frame count, or 0 for endless streaming (stop via ll_spi_dma_stop).
 *
 * NOTE (NanEyeC M3 direction flips): there is no SPI-EOT GPDMA trigger on WBA
 * (RM0493 Table 127), so phase-boundary HDDIR flips are driven by the SPI EOT
 * interrupt in a CPU ISR (approach A). If that proves too tight, the fallback
 * is gpdma1_chX_tc cross-triggers (TRIGSEL 20-27) to chain a control channel
 * — keeping in mind DMA-TC fires at FIFO-fill, not bus EOT.
 */
static inline void ll_spi_dma_tx_start(SPI_TypeDef *spi, uint16_t tsize)
{
    CLR_BITS(spi->CR1, LL_SPI_CR1_SPE);       /* TSIZE/HDDIR writable only when off */
    SET_BITS(spi->CR1, LL_SPI_CR1_HDDIR);     /* transmitter (ignored in full-duplex) */
    spi->CR2 = tsize;
    SET_BITS(spi->CFG1, LL_SPI_CFG1_TXDMAEN);
    SET_BITS(spi->CR1, LL_SPI_CR1_SPE);
    SET_BITS(spi->CR1, LL_SPI_CR1_CSTART);
}

static inline void ll_spi_dma_rx_start(SPI_TypeDef *spi, uint16_t tsize)
{
    CLR_BITS(spi->CR1, LL_SPI_CR1_SPE);
    CLR_BITS(spi->CR1, LL_SPI_CR1_HDDIR);     /* receiver */
    spi->CR2 = tsize;
    SET_BITS(spi->CFG1, LL_SPI_CFG1_RXDMAEN);
    SET_BITS(spi->CR1, LL_SPI_CR1_SPE);
    SET_BITS(spi->CR1, LL_SPI_CR1_CSTART);
}

/** Stop a DMA transfer: disable the peripheral + DMA request generation, clear flags. */
static inline void ll_spi_dma_stop(SPI_TypeDef *spi)
{
    CLR_BITS(spi->CR1, LL_SPI_CR1_SPE);
    CLR_BITS(spi->CFG1, LL_SPI_CFG1_TXDMAEN | LL_SPI_CFG1_RXDMAEN);
    spi->IFCR = LL_SPI_IFCR_EOTC | LL_SPI_IFCR_TXTFC | LL_SPI_IFCR_OVRC;
}

#endif /* SPI v2 half-duplex */

#endif /* LL_SPI_H */
