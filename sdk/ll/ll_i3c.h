/**
 * ll_i3c.h — Low-level I3C (STM32H5 "improved inter-integrated circuit")
 *
 * Register map, bit fields and the clock/reset/kernel-clock plumbing for the
 * I3C peripheral of the STM32H523 (Core.ST.H5), controller role. Everything
 * here is from RM0481 Rev 4 chapter 49 ("Improved inter-integrated circuit
 * (I3C)", §49.16 registers, Table 547 register map) and chapter 11 (RCC).
 *
 * Only the H523 has an I3C peripheral among the Cores. On every other part
 * this header defines nothing but LL_I3C_AVAILABLE = 0, so code can test it.
 */

#ifndef LL_I3C_H
#define LL_I3C_H

#include "ll_common.h"
#include "ll_rcc.h"

#if defined(STM32H523xx)

#define LL_I3C_AVAILABLE 1

/* ---- Register block (RM0481 Table 547; offsets in the comments) ---- */

typedef struct {
    volatile uint32_t CR;          /* 0x000 message control (write: pushes the C-FIFO) */
    volatile uint32_t CFGR;        /* 0x004 configuration */
    uint32_t          _r0[2];      /* 0x008-0x00C */
    volatile uint32_t RDR;         /* 0x010 receive data byte */
    volatile uint32_t RDWR;        /* 0x014 receive data word */
    volatile uint32_t TDR;         /* 0x018 transmit data byte */
    volatile uint32_t TDWR;        /* 0x01C transmit data word */
    volatile uint32_t IBIDR;       /* 0x020 IBI payload data */
    volatile uint32_t TGTTDR;      /* 0x024 target transmit configuration */
    uint32_t          _r1[2];      /* 0x028-0x02C */
    volatile uint32_t SR;          /* 0x030 status */
    volatile uint32_t SER;         /* 0x034 status error */
    uint32_t          _r2[2];      /* 0x038-0x03C */
    volatile uint32_t RMR;         /* 0x040 received message */
    uint32_t          _r3[3];      /* 0x044-0x04C */
    volatile uint32_t EVR;         /* 0x050 event (flags) */
    volatile uint32_t IER;         /* 0x054 interrupt enable */
    volatile uint32_t CEVR;        /* 0x058 clear event */
    uint32_t          _r4;         /* 0x05C */
    volatile uint32_t DEVR0;       /* 0x060 own device characteristics */
    volatile uint32_t DEVR[4];     /* 0x064-0x070 device 1..4 characteristics (DEVR[0] = DEVR1) */
    uint32_t          _r5[7];      /* 0x074-0x08C */
    volatile uint32_t MAXRLR;      /* 0x090 maximum read length */
    volatile uint32_t MAXWLR;      /* 0x094 maximum write length */
    uint32_t          _r6[2];      /* 0x098-0x09C */
    volatile uint32_t TIMINGR0;    /* 0x0A0 timing 0 */
    volatile uint32_t TIMINGR1;    /* 0x0A4 timing 1 */
    volatile uint32_t TIMINGR2;    /* 0x0A8 timing 2 */
    uint32_t          _r7[5];      /* 0x0AC-0x0BC */
    volatile uint32_t BCR;         /* 0x0C0 bus characteristics (target) */
    volatile uint32_t DCR;         /* 0x0C4 device characteristics (target) */
    volatile uint32_t GETCAPR;     /* 0x0C8 get capability (target) */
    volatile uint32_t CRCAPR;      /* 0x0CC controller-role capability (target) */
    volatile uint32_t GETMXDSR;    /* 0x0D0 get max data speed (target) */
    volatile uint32_t EPIDR;       /* 0x0D4 extended provisioned ID (target) */
} I3C_TypeDef;

/* Non-secure aliases (RM0481 Table 3 memory map: I3C1 APB1 0x4000 5C00,
 * I3C2 APB3 0x4400 3000). */
#define I3C1  ((I3C_TypeDef *)0x40005C00UL)
#define I3C2  ((I3C_TypeDef *)0x44003000UL)

/* ---- I3C_CR (§49.16.1 / §49.16.2) ---- */
#define LL_I3C_CR_MEND            (1UL << 31)   /* last message: stop, else Sr */
#define LL_I3C_CR_MTYPE_SHIFT     27
#define LL_I3C_CR_MTYPE_SCL_STOP  (0x0UL << 27) /* force SCL stop (CE1 recovery) */
#define LL_I3C_CR_MTYPE_HEADER    (0x1UL << 27)
#define LL_I3C_CR_MTYPE_PRIVATE   (0x2UL << 27) /* I3C SDR private read/write */
#define LL_I3C_CR_MTYPE_DIRECT    (0x3UL << 27) /* 2nd part of a direct CCC */
#define LL_I3C_CR_MTYPE_I2C       (0x4UL << 27) /* legacy I2C message */
#define LL_I3C_CR_MTYPE_CCC       (0x6UL << 27) /* broadcast CCC / 1st part of a direct CCC */
#define LL_I3C_CR_ADD_SHIFT       17            /* ADD[6:0] = bits 23:17 */
#define LL_I3C_CR_CCC_SHIFT       16            /* CCC[7:0] = bits 23:16 (MTYPE 0110) */
#define LL_I3C_CR_RNW             (1UL << 16)   /* 1 = read */
#define LL_I3C_CR_DCNT_MASK       0xFFFFUL

/* ---- I3C_CFGR (§49.16.3) ---- */
#define LL_I3C_CFGR_EN            (1UL << 0)
#define LL_I3C_CFGR_CRINIT        (1UL << 1)    /* 1 = controller role */
#define LL_I3C_CFGR_NOARBH        (1UL << 2)    /* 1 = no 0x7E arbitrable header after a start */
#define LL_I3C_CFGR_RSTPTRN       (1UL << 3)
#define LL_I3C_CFGR_EXITPTRN      (1UL << 4)
#define LL_I3C_CFGR_HKSDAEN       (1UL << 5)    /* SDA high keeper */
#define LL_I3C_CFGR_HJACK         (1UL << 7)
#define LL_I3C_CFGR_RXDMAEN       (1UL << 8)
#define LL_I3C_CFGR_RXFLUSH       (1UL << 9)
#define LL_I3C_CFGR_RXTHRES       (1UL << 10)   /* 1 = word threshold */
#define LL_I3C_CFGR_TXDMAEN       (1UL << 12)
#define LL_I3C_CFGR_TXFLUSH       (1UL << 13)
#define LL_I3C_CFGR_TXTHRES       (1UL << 14)
#define LL_I3C_CFGR_SDMAEN        (1UL << 16)
#define LL_I3C_CFGR_SFLUSH        (1UL << 17)
#define LL_I3C_CFGR_SMODE         (1UL << 18)   /* S-FIFO on */
#define LL_I3C_CFGR_TMODE         (1UL << 19)   /* C-/TX-FIFO preload before the start */
#define LL_I3C_CFGR_CDMAEN        (1UL << 20)
#define LL_I3C_CFGR_CFLUSH        (1UL << 21)
#define LL_I3C_CFGR_TSFSET        (1UL << 30)

/* ---- I3C_SR (§49.16.10) ---- */
#define LL_I3C_SR_XDCNT_MASK      0xFFFFUL
#define LL_I3C_SR_ABT             (1UL << 17)   /* private read ended early by the target */
#define LL_I3C_SR_DIR             (1UL << 18)
#define LL_I3C_SR_MID_SHIFT       24

/* ---- I3C_SER (§49.16.11) ---- */
#define LL_I3C_SER_CODERR_MASK    0xFUL         /* CE0..CE3 = 0..3 when controller */
#define LL_I3C_SER_PERR           (1UL << 4)
#define LL_I3C_SER_STALL          (1UL << 5)
#define LL_I3C_SER_DOVR           (1UL << 6)    /* RX-FIFO overrun / TX-FIFO underrun */
#define LL_I3C_SER_COVR           (1UL << 7)    /* C-FIFO underrun / S-FIFO overrun */
#define LL_I3C_SER_ANACK          (1UL << 8)    /* address NACK (2nd try on direct CCC reads) */
#define LL_I3C_SER_DNACK          (1UL << 9)    /* data NACK (I2C write, ENTDAA 2nd try) */
#define LL_I3C_SER_DERR           (1UL << 10)

/* ---- I3C_RMR (§49.16.12) ---- */
#define LL_I3C_RMR_IBIRDCNT_MASK  0x7UL
#define LL_I3C_RMR_RADD_SHIFT     17            /* RADD[6:0] = bits 23:17 */

/* ---- I3C_EVR / I3C_IER / I3C_CEVR (§49.16.13-15): same bit positions ---- */
#define LL_I3C_EVR_CFEF           (1UL << 0)
#define LL_I3C_EVR_TXFEF          (1UL << 1)
#define LL_I3C_EVR_CFNFF          (1UL << 2)
#define LL_I3C_EVR_SFNEF          (1UL << 3)
#define LL_I3C_EVR_TXFNFF         (1UL << 4)
#define LL_I3C_EVR_RXFNEF         (1UL << 5)
#define LL_I3C_EVR_TXLASTF        (1UL << 6)
#define LL_I3C_EVR_RXLASTF        (1UL << 7)
#define LL_I3C_EVR_FCF            (1UL << 9)
#define LL_I3C_EVR_RXTGTENDF      (1UL << 10)
#define LL_I3C_EVR_ERRF           (1UL << 11)
#define LL_I3C_EVR_IBIF           (1UL << 15)
#define LL_I3C_EVR_CRF            (1UL << 17)
#define LL_I3C_EVR_HJF            (1UL << 19)
#define LL_I3C_IER_IBIIE          LL_I3C_EVR_IBIF
#define LL_I3C_IER_ERRIE          LL_I3C_EVR_ERRF
#define LL_I3C_CEVR_ALL_CTRL      (LL_I3C_EVR_FCF | LL_I3C_EVR_RXTGTENDF | LL_I3C_EVR_ERRF \
                                   | LL_I3C_EVR_IBIF | LL_I3C_EVR_CRF | LL_I3C_EVR_HJF)

/* ---- I3C_DEVR0 / I3C_DEVRx (§49.16.16-17) ---- */
#define LL_I3C_DEVR_DA_SHIFT      1             /* DA[6:0] = bits 7:1 */
#define LL_I3C_DEVR0_DAVAL        (1UL << 0)
#define LL_I3C_DEVRX_IBIACK       (1UL << 16)
#define LL_I3C_DEVRX_CRACK        (1UL << 17)
#define LL_I3C_DEVRX_IBIDEN       (1UL << 18)   /* the target's BCR[2]: IBI carries a payload */
#define LL_I3C_DEVRX_SUSP         (1UL << 19)
#define LL_I3C_DEVRX_DIS          (1UL << 31)   /* DA/IBIDEN locked while set (read-only) */

/* ---- I3C_TIMINGR0/1/2 (§49.16.20-22) ---- */
#define LL_I3C_TIMINGR0(sclh_i2c, scll_od, sclh_i3c, scll_pp) \
    (((uint32_t)(sclh_i2c) << 24) | ((uint32_t)(scll_od) << 16) | \
     ((uint32_t)(sclh_i3c) << 8) | (uint32_t)(scll_pp))
#define LL_I3C_TIMINGR1(sda_hd, free, aval) \
    (((uint32_t)(sda_hd) << 28) | ((uint32_t)(free) << 16) | (uint32_t)(aval))
#define LL_I3C_TIMINGR2_STALLT    (1UL << 0)
#define LL_I3C_TIMINGR2_STALLD    (1UL << 1)
#define LL_I3C_TIMINGR2_STALLC    (1UL << 2)
#define LL_I3C_TIMINGR2_STALLA    (1UL << 3)
#define LL_I3C_TIMINGR2_STALL_SHIFT 8

/* ---- CCC codes used by the controller (§49.9.1 Table 542; MIPI I3C v1.1) ---- */
#define LL_I3C_CCC_ENEC_BC        0x00U
#define LL_I3C_CCC_DISEC_BC       0x01U
#define LL_I3C_CCC_RSTDAA         0x06U
#define LL_I3C_CCC_ENTDAA         0x07U
#define LL_I3C_CCC_ENEC           0x80U
#define LL_I3C_CCC_DISEC          0x81U
#define LL_I3C_CCC_SETDASA        0x87U
#define LL_I3C_CCC_SETNEWDA       0x88U
#define LL_I3C_CCC_SETMWL         0x89U
#define LL_I3C_CCC_SETMRL         0x8AU
#define LL_I3C_CCC_GETMWL         0x8BU
#define LL_I3C_CCC_GETMRL         0x8CU
#define LL_I3C_CCC_GETPID         0x8DU
#define LL_I3C_CCC_GETBCR         0x8EU
#define LL_I3C_CCC_GETDCR         0x8FU
#define LL_I3C_CCC_GETSTATUS      0x90U
#define LL_I3C_CCC_GETMXDS        0x94U

/* ENEC/DISEC event byte (MIPI I3C v1.1 §5.1.9.3.1) */
#define LL_I3C_EVT_INT            0x01U         /* in-band interrupts */
#define LL_I3C_EVT_CR             0x02U         /* controller-role requests */
#define LL_I3C_EVT_HJ             0x08U         /* hot-join */

/* ---- Interrupt lines (RM0481 Table 147) ---- */
#define LL_I3C1_EV_IRQn           123U
#define LL_I3C1_ER_IRQn           124U
#define LL_I3C2_EV_IRQn           131U
#define LL_I3C2_ER_IRQn           132U

/* ---- RCC plumbing (RM0481 §11.8) ----
 * I3C1: RCC_APB1LENR bit 23 (I3C1EN, §11.8.29), RCC_APB1LRSTR bit 23 (I3C1RST,
 * §11.8.22), kernel clock RCC_CCIPR4.I3C1SEL[25:24] (§11.8.43).
 * I3C2: RCC_APB3ENR bit 9 (I3C2EN, §11.8.32), RCC_APB3RSTR bit 9 (I3C2RST,
 * §11.8.25, offset 0x080), RCC_CCIPR4.I3C2SEL[27:26].
 * Kernel clock choices: 00 rcc_pclk1 (reset), 01 pll3_r_ck, 10 hsi_ker_ck. */
#define LL_I3C_RCC_CR             REG32(RCC_BASE + 0x000UL)
#define LL_I3C_RCC_APB1LRSTR      REG32(RCC_BASE + 0x074UL)
#define LL_I3C_RCC_APB3RSTR       REG32(RCC_BASE + 0x080UL)
#define LL_I3C_RCC_APB1LENR       REG32(RCC_BASE + 0x09CUL)
#define LL_I3C_RCC_APB3ENR        REG32(RCC_BASE + 0x0A8UL)
#define LL_I3C_RCC_CCIPR4         REG32(RCC_BASE + 0x0E4UL)

#define LL_I3C_KER_PCLK           0x0UL
#define LL_I3C_KER_PLL3R          0x1UL
#define LL_I3C_KER_HSI            0x2UL

/** Enable the bus clock of an I3C instance and pulse its RCC reset. */
static inline void ll_i3c_clk_enable_reset(I3C_TypeDef *i3c)
{
    if (i3c == I3C1) {
        LL_I3C_RCC_APB1LENR |= (1UL << 23);
        (void)LL_I3C_RCC_APB1LENR;
        LL_I3C_RCC_APB1LRSTR |= (1UL << 23);
        LL_I3C_RCC_APB1LRSTR &= ~(1UL << 23);
    } else if (i3c == I3C2) {
        LL_I3C_RCC_APB3ENR |= (1UL << 9);
        (void)LL_I3C_RCC_APB3ENR;
        LL_I3C_RCC_APB3RSTR |= (1UL << 9);
        LL_I3C_RCC_APB3RSTR &= ~(1UL << 9);
    }
}

/** Stop the bus clock of an I3C instance. */
static inline void ll_i3c_clk_disable(I3C_TypeDef *i3c)
{
    if (i3c == I3C1)      LL_I3C_RCC_APB1LENR &= ~(1UL << 23);
    else if (i3c == I3C2) LL_I3C_RCC_APB3ENR  &= ~(1UL << 9);
}

/** Select the kernel clock of an I3C instance (LL_I3C_KER_*). */
static inline void ll_i3c_set_kernel_clk(I3C_TypeDef *i3c, uint32_t sel)
{
    uint32_t shift = (i3c == I3C2) ? 26U : 24U;
    LL_I3C_RCC_CCIPR4 = (LL_I3C_RCC_CCIPR4 & ~(3UL << shift)) | ((sel & 3UL) << shift);
}

/** hsi_ker_ck in Hz: the 64 MHz HSI through RCC_CR.HSIDIV[4:3] (/1 /2 /4 /8, §11.8.1). */
static inline uint32_t ll_i3c_hsi_hz(void)
{
    return 64000000UL >> ((LL_I3C_RCC_CR >> 3) & 3UL);
}

#else  /* no I3C on this part */

#define LL_I3C_AVAILABLE 0

#endif /* STM32H523xx */

#endif /* LL_I3C_H */
