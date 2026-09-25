/**
 * ll_sai.h — SAI (Serial Audio Interface) register layer
 *
 * Register structs + bit definitions for the STM32 SAI, plus thin inline
 * accessors. Procedures (PDM init, DMA capture) live in hal_sai; this layer
 * is just the registers. Transcribed from the CMSIS device header
 * (stm32wba55xx.h) — offsets/masks are authoritative.
 *
 * SAI is available on Core.ST.W5 (one SAI) only. The L011, L422 and H523
 * have no SAI peripheral.
 *
 * On the WBA55 only **SAI1 sub-block A** can run the PDM controller
 * (HAL_SAI_Init enforces SAI1_Block_A + MASTER_RX + FREE_PROTOCOL for PDM).
 */
#ifndef LL_SAI_H
#define LL_SAI_H

#include "ll_common.h"

/* -------------------------------------------------------------- */
/* Register structures                                             */
/* -------------------------------------------------------------- */

/** Per-block registers (SAI block A or B). */
typedef struct {
    volatile uint32_t CR1;   /* 0x00: configuration register 1 */
    volatile uint32_t CR2;   /* 0x04: configuration register 2 (FIFO) */
    volatile uint32_t FRCR;  /* 0x08: frame configuration */
    volatile uint32_t SLOTR; /* 0x0C: slot configuration */
    volatile uint32_t IMR;   /* 0x10: interrupt mask */
    volatile uint32_t SR;    /* 0x14: status */
    volatile uint32_t CLRFR; /* 0x18: clear flag */
    volatile uint32_t DR;    /* 0x1C: data */
} SAI_Block_TypeDef;

/** SAI global registers (GCR + the PDM controller). */
typedef struct {
    volatile uint32_t GCR;        /* 0x00: global configuration */
    uint32_t          RESERVED[16];
    volatile uint32_t PDMCR;      /* 0x44: PDM control */
    volatile uint32_t PDMDLY;     /* 0x48: PDM delay */
} SAI_TypeDef;

/* -------------------------------------------------------------- */
/* Instances (SAI1 on APB2 @ 0x40015400 for WBA55)                 */
/* -------------------------------------------------------------- */

/* Only the WBA55 has an SAI. The STM32L422 (RM0394: SAI1 is on the L43x/L44x
 * and up, e.g. the RCC_CCIPR.SAI1SEL footnote) and the STM32H523 have none,
 * and 0x40015400 is not an SAI on either, so no SAI1 is defined there: code
 * that uses it fails to compile instead of writing to whatever lives there. */
#if defined(STM32WBA55xx)
  #define SAI1          ((SAI_TypeDef *)0x40015400UL)
  #define SAI1_Block_A  ((SAI_Block_TypeDef *)0x40015404UL)
  #define SAI1_Block_B  ((SAI_Block_TypeDef *)0x40015424UL)
  #define LL_SAI_AVAILABLE 1
#else
  #define LL_SAI_AVAILABLE 0
#endif

/* -------------------------------------------------------------- */
/* CR1 — block configuration 1                                     */
/* -------------------------------------------------------------- */

#define SAI_CR1_MODE          (0x3UL << 0)   /**< Audio block mode */
#define SAI_CR1_MODE_MASTER_RX (0x1UL << 0)  /**< Master receiver (MODE=01) */
#define SAI_CR1_PRTCFG        (0x3UL << 2)   /**< Protocol cfg (00 = free) */
#define SAI_CR1_PRTCFG_FREE   (0x0UL << 2)
#define SAI_CR1_DS            (0x7UL << 5)   /**< Data size */
#define SAI_CR1_DS_32         (0x7UL << 5)   /**< 32-bit (DS=111) */
#define SAI_CR1_DS_16         (0x4UL << 5)   /**< 16-bit (DS=100) */
#define SAI_CR1_LSBFIRST      (0x1UL << 8)   /**< 1=LSB first, 0=MSB first */
#define SAI_CR1_CKSTR         (0x1UL << 9)   /**< Clock strobing edge */
#define SAI_CR1_SYNCEN        (0x3UL << 10)  /**< Sync enable (00 = async) */
#define SAI_CR1_MONO          (0x1UL << 12)  /**< Mono mode */
#define SAI_CR1_OUTDRIV       (0x1UL << 13)  /**< Output drive */
#define SAI_CR1_SAIEN         (0x1UL << 16)  /**< Audio block enable */
#define SAI_CR1_DMAEN         (0x1UL << 17)  /**< DMA enable */
#define SAI_CR1_NODIV         (0x1UL << 19)  /**< 1=no divider (bypass /256) */
#define SAI_CR1_MCKDIV_Pos    20
#define SAI_CR1_MCKDIV        (0x3FUL << 20) /**< Master clock divider [5:0] */
#define SAI_CR1_OSR           (0x1UL << 26)  /**< Oversampling ratio */
#define SAI_CR1_MCKEN         (0x1UL << 27)  /**< Master clock output enable */

/* -------------------------------------------------------------- */
/* CR2 — FIFO                                                       */
/* -------------------------------------------------------------- */

#define SAI_CR2_FTH           (0x7UL << 0)   /**< FIFO threshold */
#define SAI_CR2_FTH_EMPTY     (0x0UL << 0)
#define SAI_CR2_FTH_QF        (0x1UL << 0)   /**< 1/4 full */
#define SAI_CR2_FTH_HF        (0x2UL << 0)   /**< 1/2 full (DMA default) */
#define SAI_CR2_FTH_3QF       (0x3UL << 0)   /**< 3/4 full */
#define SAI_CR2_FTH_FULL      (0x4UL << 0)
#define SAI_CR2_FFLUSH        (0x1UL << 3)   /**< FIFO flush (self-clearing) */
#define SAI_CR2_COMP          (0x3UL << 14)  /**< Companding (00 = none) */

/* -------------------------------------------------------------- */
/* FRCR — frame configuration                                      */
/* -------------------------------------------------------------- */

#define SAI_FRCR_FRL_Pos      0
#define SAI_FRCR_FRL          (0xFFUL << 0)  /**< Frame length - 1 */
#define SAI_FRCR_FSALL_Pos    8
#define SAI_FRCR_FSALL        (0x7FUL << 8)  /**< FS active level length - 1 */
#define SAI_FRCR_FSDEF        (0x1UL << 16)  /**< 1=FS is start-of-frame + channel ID */
#define SAI_FRCR_FSPOL        (0x1UL << 17)  /**< FS polarity: 1 = active high */
#define SAI_FRCR_FSOFF        (0x1UL << 18)  /**< FS offset */

/* -------------------------------------------------------------- */
/* SLOTR — slot configuration                                      */
/* -------------------------------------------------------------- */

#define SAI_SLOTR_FBOFF       (0x1FUL << 0)  /**< First-bit offset */
#define SAI_SLOTR_SLOTSZ      (0x3UL << 6)   /**< Slot size */
#define SAI_SLOTR_SLOTSZ_32   (0x2UL << 6)   /**< 32-bit slot */
#define SAI_SLOTR_NBSLOT_Pos  8
#define SAI_SLOTR_NBSLOT      (0xFUL << 8)   /**< Number of slots - 1 */
#define SAI_SLOTR_SLOTEN_Pos  16
#define SAI_SLOTR_SLOTEN      (0xFFFFUL << 16) /**< Slot enable bitmap */

/* -------------------------------------------------------------- */
/* SR / CLRFR — status & clear                                     */
/* -------------------------------------------------------------- */

#define SAI_SR_OVRUDR         (0x1UL << 0)   /**< Overrun / underrun */
#define SAI_SR_FREQ           (0x1UL << 3)   /**< FIFO request */
#define SAI_SR_FLVL           (0x7UL << 16)  /**< FIFO level */
#define SAI_CLRFR_COVRUDR     (0x1UL << 0)   /**< Clear overrun/underrun flag */

/* -------------------------------------------------------------- */
/* GCR — global                                                    */
/* -------------------------------------------------------------- */

#define SAI_GCR_SYNCIN        (0x3UL << 0)
#define SAI_GCR_SYNCOUT       (0x3UL << 4)

/* -------------------------------------------------------------- */
/* PDMCR / PDMDLY — PDM controller                                 */
/* -------------------------------------------------------------- */

#define SAI_PDMCR_PDMEN       (0x1UL << 0)   /**< PDM enable (set LAST) */
#define SAI_PDMCR_MICNBR_Pos  4
#define SAI_PDMCR_MICNBR      (0x3UL << 4)   /**< (mic pairs - 1) */
#define SAI_PDMCR_CKEN1       (0x1UL << 8)   /**< PDM clock 1 (CK1/D1 group) */
#define SAI_PDMCR_CKEN2       (0x1UL << 9)   /**< PDM clock 2 (CK2/D2 group) */
#define SAI_PDMCR_CKEN3       (0x1UL << 10)
#define SAI_PDMCR_CKEN4       (0x1UL << 11)

/* -------------------------------------------------------------- */
/* Thin inline accessors                                           */
/* -------------------------------------------------------------- */

/** Disable a block (clear SAIEN) and wait for it to read back 0. */
static inline void ll_sai_block_disable(SAI_Block_TypeDef *b)
{
    b->CR1 &= ~SAI_CR1_SAIEN;
    uint32_t t = 100000;
    while ((b->CR1 & SAI_CR1_SAIEN) && --t) { }
}

/** Enable a block — for master RX this immediately starts the bit clock. */
static inline void ll_sai_block_enable(SAI_Block_TypeDef *b) { b->CR1 |= SAI_CR1_SAIEN; }

/** Enable peripheral DMA requests (must precede SAIEN). */
static inline void ll_sai_block_enable_dma(SAI_Block_TypeDef *b) { b->CR1 |= SAI_CR1_DMAEN; }

/** Flush the FIFO (one-shot, self-clearing). */
static inline void ll_sai_flush(SAI_Block_TypeDef *b) { b->CR2 |= SAI_CR2_FFLUSH; }

/** Read one data word from the FIFO. */
static inline uint32_t ll_sai_read(SAI_Block_TypeDef *b) { return b->DR; }

/** True if an overrun has been latched. */
static inline int ll_sai_overrun(SAI_Block_TypeDef *b) { return (b->SR & SAI_SR_OVRUDR) != 0; }

/** Clear the overrun flag. */
static inline void ll_sai_clear_overrun(SAI_Block_TypeDef *b) { b->CLRFR = SAI_CLRFR_COVRUDR; }

#endif /* LL_SAI_H */
