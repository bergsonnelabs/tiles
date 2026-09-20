/**
 * ll_i2c.h — Low-level I2C operations
 *
 * Polling master-mode I2C. All four STM32 families use the same
 * I2C IP with TIMINGR-based configuration, so operations are
 * generic — only base addresses differ.
 */

#ifndef LL_I2C_H
#define LL_I2C_H

#include "ll_common.h"

/* ---- I2C register structure ---- */

typedef struct {
    volatile uint32_t CR1;      /* 0x00: Control register 1 */
    volatile uint32_t CR2;      /* 0x04: Control register 2 */
    volatile uint32_t OAR1;     /* 0x08: Own address 1 */
    volatile uint32_t OAR2;     /* 0x0C: Own address 2 */
    volatile uint32_t TIMINGR;  /* 0x10: Timing register */
    volatile uint32_t TIMEOUTR; /* 0x14: Timeout register */
    volatile uint32_t ISR;      /* 0x18: Interrupt and status */
    volatile uint32_t ICR;      /* 0x1C: Interrupt flag clear */
    volatile uint32_t PECR;     /* 0x20: PEC register */
    volatile uint32_t RXDR;     /* 0x24: Receive data */
    volatile uint32_t TXDR;     /* 0x28: Transmit data */
} I2C_TypeDef;

/* ---- Instance base addresses ---- */

#if defined(STM32L011xx)
  #define I2C1      ((I2C_TypeDef *)0x40005400UL)

#elif defined(STM32L422xx)
  #define I2C1      ((I2C_TypeDef *)0x40005400UL)
  #define I2C3      ((I2C_TypeDef *)0x40005C00UL)

#elif defined(STM32WBA55xx)
  #define I2C1      ((I2C_TypeDef *)0x40005400UL)
  #define I2C3      ((I2C_TypeDef *)0x46002800UL)

#elif defined(STM32H523xx)
  #define I2C1      ((I2C_TypeDef *)0x40005400UL)
  #define I2C2      ((I2C_TypeDef *)0x40005800UL)
  #define I2C3      ((I2C_TypeDef *)0x44002800UL)
#endif

/* ---- CR1 bit definitions ---- */

#define LL_I2C_CR1_PE           (1UL << 0)    /* Peripheral enable */
#define LL_I2C_CR1_TXIE         (1UL << 1)    /* TX interrupt enable */
#define LL_I2C_CR1_RXIE         (1UL << 2)    /* RX interrupt enable */
#define LL_I2C_CR1_NACKIE       (1UL << 4)    /* NACK interrupt enable */
#define LL_I2C_CR1_STOPIE       (1UL << 5)    /* STOP interrupt enable */
#define LL_I2C_CR1_ANFOFF       (1UL << 12)   /* Analog filter off */
#define LL_I2C_CR1_DNF_SHIFT    8             /* Digital noise filter [3:0] */
#define LL_I2C_CR1_FMP          (1UL << 24)   /* Fast-mode plus enable (WBA55, H523) */
#define LL_I2C_CR1_NOSTRETCH    (1UL << 17)   /* Clock stretching disable (slave mode) */
#define LL_I2C_CR1_GCEN         (1UL << 19)   /* General call enable */
#define LL_I2C_CR1_ADDRIE       (1UL << 3)    /* Address-match interrupt enable (target) */
#define LL_I2C_CR1_TCIE         (1UL << 6)    /* Transfer-complete interrupt enable */
#define LL_I2C_CR1_ERRIE        (1UL << 7)    /* Error interrupt enable */
#define LL_I2C_CR1_SBC          (1UL << 16)   /* Slave byte control (target, needs RELOAD) */

/* ---- OAR1 bit definitions (target-mode own address) ---- */

#define LL_I2C_OAR1_OA1_SHIFT   1             /* Own address 1 [7:1] in 7-bit mode */
#define LL_I2C_OAR1_OA1MODE     (1UL << 10)   /* 0 = 7-bit address, 1 = 10-bit */
#define LL_I2C_OAR1_OA1EN       (1UL << 15)   /* Own address 1 enable */

/* ---- CR2 bit definitions ---- */

#define LL_I2C_CR2_SADD_SHIFT   1             /* Slave address [7:1] for 7-bit */
#define LL_I2C_CR2_RD_WRN       (1UL << 10)   /* Transfer direction: 1=read */
#define LL_I2C_CR2_ADD10        (1UL << 11)   /* 10-bit addressing */
#define LL_I2C_CR2_START        (1UL << 13)   /* Generate START */
#define LL_I2C_CR2_STOP         (1UL << 14)   /* Generate STOP */
#define LL_I2C_CR2_NACK         (1UL << 15)   /* Generate NACK */
#define LL_I2C_CR2_NBYTES_SHIFT 16            /* Number of bytes [7:0] */
#define LL_I2C_CR2_RELOAD       (1UL << 24)   /* Reload mode */
#define LL_I2C_CR2_AUTOEND      (1UL << 25)   /* Auto-end mode */

/* ---- ISR bit definitions ---- */

#define LL_I2C_ISR_TXE          (1UL << 0)    /* TX register empty */
#define LL_I2C_ISR_TXIS         (1UL << 1)    /* TX interrupt status */
#define LL_I2C_ISR_RXNE         (1UL << 2)    /* RX register not empty */
#define LL_I2C_ISR_ADDR         (1UL << 3)    /* Address matched (slave) */
#define LL_I2C_ISR_NACKF        (1UL << 4)    /* NACK received */
#define LL_I2C_ISR_STOPF        (1UL << 5)    /* STOP detected */
#define LL_I2C_ISR_TC           (1UL << 6)    /* Transfer complete */
#define LL_I2C_ISR_TCR          (1UL << 7)    /* Transfer complete reload */
#define LL_I2C_ISR_BERR         (1UL << 8)    /* Bus error */
#define LL_I2C_ISR_ARLO         (1UL << 9)    /* Arbitration lost */
#define LL_I2C_ISR_OVR          (1UL << 10)   /* Overrun/underrun */
#define LL_I2C_ISR_BUSY         (1UL << 15)   /* Bus busy */
#define LL_I2C_ISR_DIR          (1UL << 16)   /* Transfer direction (target): 1 = controller reads */
#define LL_I2C_ISR_ADDCODE_SHIFT 17           /* Matched address [7:1] (target) */
#define LL_I2C_ISR_ADDCODE_MASK 0x7FUL

/* ---- ICR bit definitions ---- */

#define LL_I2C_ICR_ADDRCF       (1UL << 3)
#define LL_I2C_ICR_NACKCF       (1UL << 4)
#define LL_I2C_ICR_STOPCF       (1UL << 5)
#define LL_I2C_ICR_BERRCF       (1UL << 8)
#define LL_I2C_ICR_ARLOCF       (1UL << 9)
#define LL_I2C_ICR_OVRCF        (1UL << 10)

/* ---- Error mask ---- */

#define LL_I2C_ERR_MASK         (LL_I2C_ISR_NACKF | LL_I2C_ISR_BERR \
                               | LL_I2C_ISR_ARLO | LL_I2C_ISR_OVR)

/* ============================================================
 * Pre-computed TIMINGR values
 *
 * The I2C TIMINGR register is a 32-bit value that controls SCL
 * timing. Its fields are:
 *
 *   [31:28] PRESC   — prescaler (divides the I2C kernel clock)
 *   [23:20] SCLDEL  — data setup time (in prescaled ticks)
 *   [19:16] SDADEL  — data hold time (in prescaled ticks)
 *   [15:8]  SCLH    — SCL high period (in prescaled ticks)
 *   [7:0]   SCLL    — SCL low period (in prescaled ticks)
 *
 * The I2C kernel clock is NOT necessarily SYSCLK — it depends
 * on the RCC_CCIPR mux for each I2C instance:
 *
 *   STM32WBA55:
 *     I2C1 kernel clock = PCLK1 (APB1 clock) by default
 *     I2C3 kernel clock = PCLK7 (APB7 clock) by default
 *     Both default to SYSCLK when APB prescaler = 1
 *     Can be switched to HSI16 via RCC_CCIPR1[13:12] / [15:14]
 *
 *   STM32L422:
 *     I2C1/I2C3 kernel clock = PCLK1 by default
 *
 * Choose the timing constant that matches your actual I2C
 * kernel clock, NOT your SYSCLK (unless APB prescaler = 1).
 *
 * To generate values for other frequencies, use CubeMX or the
 * AN4235 "I2C timing configuration tool" spreadsheet.
 * ============================================================ */

/* Standard mode (100kHz) from various peripheral clocks.
 * Values generated with STM32CubeMX I2C Timing Configuration tool
 * (250ns rise, 100ns fall for standard mode). */
#define LL_I2C_TIMING_100K_1MHZ     0x00000305UL
#define LL_I2C_TIMING_100K_2MHZ     0x00000509UL  /* CubeMX: Core.ST.L0 default (MSI 2MHz) */
#define LL_I2C_TIMING_100K_4MHZ     0x00100F16UL
#define LL_I2C_TIMING_100K_8MHZ     0x00201F2CUL
#define LL_I2C_TIMING_100K_16MHZ    0x00503D5AUL
#define LL_I2C_TIMING_100K_32MHZ    0x10707DBFUL
#define LL_I2C_TIMING_100K_48MHZ    0x20602938UL
#define LL_I2C_TIMING_100K_64MHZ    0x10707DBCUL  /* CubeMX: Core.ST.H5 default (HSI 64MHz) */
#define LL_I2C_TIMING_100K_80MHZ    0x30A0A7FBUL
#define LL_I2C_TIMING_100K_128MHZ   0x20A0C4DFUL  /* CubeMX: Core.ST.H5 high (PLL 128MHz) */
#define LL_I2C_TIMING_100K_144MHZ   0x80602938UL
#define LL_I2C_TIMING_100K_240MHZ   0xE0602938UL
#define LL_I2C_TIMING_100K_248MHZ   0x40C0E9FFUL  /* CubeMX: Core.ST.H5 max (PLL 248MHz) */

/* Fast mode (400kHz) from various peripheral clocks.
 * Minimum kernel clock: 4 MHz. Values from CubeMX (100ns rise, 10ns fall). */
#define LL_I2C_TIMING_400K_4MHZ     0x00100206UL
#define LL_I2C_TIMING_400K_8MHZ     0x0010030BUL
#define LL_I2C_TIMING_400K_16MHZ    0x00300617UL
#define LL_I2C_TIMING_400K_32MHZ    0x00701737UL  /* Verified: Ring Demo @ HSE 32MHz */
#define LL_I2C_TIMING_400K_48MHZ    0x00B01A4BUL
#define LL_I2C_TIMING_400K_64MHZ    0x00602173UL  /* CubeMX: Core.ST.H5 default (HSI 64MHz) */
#define LL_I2C_TIMING_400K_80MHZ    0x00B01B59UL
#define LL_I2C_TIMING_400K_128MHZ   0x2040184BUL  /* CubeMX: Core.ST.H5 high (PLL 128MHz) */
#define LL_I2C_TIMING_400K_144MHZ   0x20B01A4BUL
#define LL_I2C_TIMING_400K_240MHZ   0x40B01A4BUL
#define LL_I2C_TIMING_400K_248MHZ   0x20802C97UL  /* CubeMX: Core.ST.H5 max (PLL 248MHz) */

/* Fast mode plus (1MHz) from various peripheral clocks.
 * Minimum kernel clock: 16 MHz. Values from CubeMX (120ns rise, 120ns fall). */
#define LL_I2C_TIMING_1M_16MHZ      0x00100306UL  /* tight margins, short bus only */
#define LL_I2C_TIMING_1M_32MHZ      0x00200B0EUL
#define LL_I2C_TIMING_1M_48MHZ      0x00300B29UL
#define LL_I2C_TIMING_1M_64MHZ      0x00300829UL  /* CubeMX: Core.ST.H5 default (HSI 64MHz) */
#define LL_I2C_TIMING_1M_80MHZ      0x00300F33UL
#define LL_I2C_TIMING_1M_128MHZ     0x00601C51UL  /* CubeMX: Core.ST.H5 high (PLL 128MHz) */
#define LL_I2C_TIMING_1M_144MHZ     0x20300B29UL
#define LL_I2C_TIMING_1M_240MHZ     0x40300B29UL
#define LL_I2C_TIMING_1M_248MHZ     0x00C032A7UL  /* CubeMX: Core.ST.H5 max (PLL 248MHz) */

/**
 * Select appropriate timing constant based on kernel clock MHz.
 * Returns a 100kHz (standard mode) timing value.
 * Returns 0 if no pre-computed value exists for the given frequency.
 */
static inline uint32_t ll_i2c_timing_100k(uint32_t kernel_mhz)
{
    switch (kernel_mhz) {
        case   1: return LL_I2C_TIMING_100K_1MHZ;
        case   2: return LL_I2C_TIMING_100K_2MHZ;
        case   4: return LL_I2C_TIMING_100K_4MHZ;
        case   8: return LL_I2C_TIMING_100K_8MHZ;
        case  16: return LL_I2C_TIMING_100K_16MHZ;
        case  32: return LL_I2C_TIMING_100K_32MHZ;
        case  48: return LL_I2C_TIMING_100K_48MHZ;
        case  64: return LL_I2C_TIMING_100K_64MHZ;
        case  80: return LL_I2C_TIMING_100K_80MHZ;
        case 128: return LL_I2C_TIMING_100K_128MHZ;
        case 144: return LL_I2C_TIMING_100K_144MHZ;
        case 240: return LL_I2C_TIMING_100K_240MHZ;
        case 248: return LL_I2C_TIMING_100K_248MHZ;
        default:  return 0;
    }
}

/**
 * Select appropriate timing constant based on kernel clock MHz.
 * Returns a 400kHz (fast mode) timing value.
 * Minimum kernel clock: 4 MHz. Returns 0 if unsupported.
 */
static inline uint32_t ll_i2c_timing_400k(uint32_t kernel_mhz)
{
    switch (kernel_mhz) {
        case   4: return LL_I2C_TIMING_400K_4MHZ;
        case   8: return LL_I2C_TIMING_400K_8MHZ;
        case  16: return LL_I2C_TIMING_400K_16MHZ;
        case  32: return LL_I2C_TIMING_400K_32MHZ;
        case  48: return LL_I2C_TIMING_400K_48MHZ;
        case  64: return LL_I2C_TIMING_400K_64MHZ;
        case  80: return LL_I2C_TIMING_400K_80MHZ;
        case 128: return LL_I2C_TIMING_400K_128MHZ;
        case 144: return LL_I2C_TIMING_400K_144MHZ;
        case 240: return LL_I2C_TIMING_400K_240MHZ;
        case 248: return LL_I2C_TIMING_400K_248MHZ;
        default:  return 0;
    }
}

/**
 * Select appropriate timing constant based on kernel clock MHz.
 * Returns a 1MHz (fast mode plus) timing value.
 * Minimum kernel clock: 16 MHz. Returns 0 if unsupported.
 */
static inline uint32_t ll_i2c_timing_1m(uint32_t kernel_mhz)
{
    switch (kernel_mhz) {
        case  16: return LL_I2C_TIMING_1M_16MHZ;
        case  32: return LL_I2C_TIMING_1M_32MHZ;
        case  48: return LL_I2C_TIMING_1M_48MHZ;
        case  64: return LL_I2C_TIMING_1M_64MHZ;
        case  80: return LL_I2C_TIMING_1M_80MHZ;
        case 128: return LL_I2C_TIMING_1M_128MHZ;
        case 144: return LL_I2C_TIMING_1M_144MHZ;
        case 240: return LL_I2C_TIMING_1M_240MHZ;
        case 248: return LL_I2C_TIMING_1M_248MHZ;
        default:  return 0;
    }
}

/* ============================================================
 * Configuration
 * ============================================================ */

/**
 * Initialize I2C in master mode.
 *   i2c:     I2C instance
 *   timing:  TIMINGR value (use LL_I2C_TIMING_* defines)
 *
 * Prerequisites:
 *   - Peripheral clock enabled via ll_rcc_apb1_clk_enable()
 *   - SDA/SCL pins configured for AF, open-drain, pull-up
 */
/**
 * Initialize I2C in master mode.
 *   i2c:     I2C instance
 *   timing:  TIMINGR value (use LL_I2C_TIMING_* defines)
 *   fmp:     set non-zero to enable Fast-mode Plus (CR1.FMP bit)
 *            Only effective on STM32WBA55 and STM32H523.
 *            On L422, FMP is enabled via SYSCFG — see hal_i2c.c.
 *            On L011, FMP is not supported.
 */
static inline void ll_i2c_init(I2C_TypeDef *i2c, uint32_t timing)
{
    /* Disable I2C while configuring */
    i2c->CR1 = 0;

    /* Set timing */
    i2c->TIMINGR = timing;

    /* Enable analog noise filter (ANFOFF=0 is enabled) */
    /* No digital filter (DNF=0000) */

    /* Enable I2C */
    i2c->CR1 = LL_I2C_CR1_PE;
}

/**
 * Initialize I2C in Fast-mode Plus (1 MHz).
 * Sets the CR1.FMP bit for 20 mA output drive on WBA55 / H523.
 */
static inline void ll_i2c_init_fmp(I2C_TypeDef *i2c, uint32_t timing)
{
    i2c->CR1 = 0;
    i2c->TIMINGR = timing;
    i2c->CR1 = LL_I2C_CR1_PE | LL_I2C_CR1_FMP;
}

/* ============================================================
 * Polling master write
 * ============================================================ */

/**
 * Return codes for I2C operations.
 */
#define LL_I2C_OK       0
#define LL_I2C_NACK     -1
#define LL_I2C_ERROR    -2
#define LL_I2C_TIMEOUT  -3

/* Default timeout for I2C wait loops (~100k iterations at typical clock) */
#ifndef LL_I2C_DEFAULT_TIMEOUT
#define LL_I2C_DEFAULT_TIMEOUT  100000UL
#endif

/* Bounded STOPF wait — used in error/NACK cleanup paths where a bare
 * `while (!(STOPF));` would hang forever if the bus is stuck.  Clears
 * STOPCF on success.  On timeout the bus is left in a dirty state, but
 * that's better than freezing the CPU. */
static inline void _ll_i2c_wait_stopf(I2C_TypeDef *i2c)
{
    volatile uint32_t t = LL_I2C_DEFAULT_TIMEOUT;
    while (!(i2c->ISR & LL_I2C_ISR_STOPF)) {
        if (--t == 0) break;
    }
    i2c->ICR = LL_I2C_ICR_STOPCF;
}

/**
 * Master write to a 7-bit address device.
 *   addr:  7-bit slave address (unshifted, e.g. 0x68 for MPU6050)
 *   data:  pointer to bytes to send
 *   len:   number of bytes
 *
 * Returns LL_I2C_OK on success, LL_I2C_NACK, LL_I2C_ERROR, or
 * LL_I2C_TIMEOUT on failure.
 */
static inline int ll_i2c_write(I2C_TypeDef *i2c, uint8_t addr,
                               const uint8_t *data, uint32_t len)
{
    volatile uint32_t timeout;

    /* Wait for bus idle (matches ST HAL behavior) */
    timeout = LL_I2C_DEFAULT_TIMEOUT;
    while (i2c->ISR & LL_I2C_ISR_BUSY) {
        if (--timeout == 0) return LL_I2C_TIMEOUT;
    }

    /* Clear any pending flags */
    i2c->ICR = LL_I2C_ICR_NACKCF | LL_I2C_ICR_STOPCF
             | LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;

    /* Configure transfer: 7-bit addr, write, NBYTES, AUTOEND, START */
    i2c->CR2 = ((uint32_t)addr << LL_I2C_CR2_SADD_SHIFT)
             | (len << LL_I2C_CR2_NBYTES_SHIFT)
             | LL_I2C_CR2_AUTOEND
             | LL_I2C_CR2_START;

    for (uint32_t i = 0; i < len; i++) {
        /* Wait for TXIS or error */
        timeout = LL_I2C_DEFAULT_TIMEOUT;
        while (1) {
            uint32_t isr = i2c->ISR;
            if (isr & LL_I2C_ISR_NACKF) {
                i2c->ICR = LL_I2C_ICR_NACKCF;
                /* AUTOEND generates STOP after NACK — wait for it */
                _ll_i2c_wait_stopf(i2c);
                return LL_I2C_NACK;
            }
            if (isr & (LL_I2C_ISR_BERR | LL_I2C_ISR_ARLO | LL_I2C_ISR_OVR)) {
                i2c->ICR = LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;
                _ll_i2c_wait_stopf(i2c);
                return LL_I2C_ERROR;
            }
            if (isr & LL_I2C_ISR_TXIS)
                break;
            if (--timeout == 0)
                return LL_I2C_TIMEOUT;
        }
        i2c->TXDR = data[i];
    }

    /* Wait for STOP (AUTOEND generates it after last byte) */
    timeout = LL_I2C_DEFAULT_TIMEOUT;
    while (!(i2c->ISR & LL_I2C_ISR_STOPF)) {
        if (--timeout == 0)
            return LL_I2C_TIMEOUT;
    }
    i2c->ICR = LL_I2C_ICR_STOPCF;

    return LL_I2C_OK;
}

/* ============================================================
 * Polling master read
 * ============================================================ */

/**
 * Master read from a 7-bit address device.
 *   addr:  7-bit slave address (unshifted)
 *   buf:   buffer to store received bytes
 *   len:   number of bytes to read
 *
 * Returns LL_I2C_OK on success, LL_I2C_NACK, LL_I2C_ERROR, or
 * LL_I2C_TIMEOUT on failure.
 */
static inline int ll_i2c_read(I2C_TypeDef *i2c, uint8_t addr,
                              uint8_t *buf, uint32_t len)
{
    volatile uint32_t timeout;

    /* NOTE: no BUSY wait here — this function is called from
     * ll_i2c_read_reg Phase 2 where the bus is intentionally
     * busy (waiting for restart). Standalone callers must ensure
     * the bus is idle before calling. */

    i2c->ICR = LL_I2C_ICR_NACKCF | LL_I2C_ICR_STOPCF
             | LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;

    /* Configure transfer: 7-bit addr, READ, NBYTES, AUTOEND, START */
    i2c->CR2 = ((uint32_t)addr << LL_I2C_CR2_SADD_SHIFT)
             | LL_I2C_CR2_RD_WRN
             | (len << LL_I2C_CR2_NBYTES_SHIFT)
             | LL_I2C_CR2_AUTOEND
             | LL_I2C_CR2_START;

    for (uint32_t i = 0; i < len; i++) {
        timeout = LL_I2C_DEFAULT_TIMEOUT;
        while (1) {
            uint32_t isr = i2c->ISR;
            if (isr & LL_I2C_ISR_NACKF) {
                i2c->ICR = LL_I2C_ICR_NACKCF;
                /* AUTOEND generates STOP after NACK — wait for it */
                _ll_i2c_wait_stopf(i2c);
                return LL_I2C_NACK;
            }
            if (isr & (LL_I2C_ISR_BERR | LL_I2C_ISR_ARLO | LL_I2C_ISR_OVR)) {
                i2c->ICR = LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;
                _ll_i2c_wait_stopf(i2c);
                return LL_I2C_ERROR;
            }
            if (isr & LL_I2C_ISR_RXNE)
                break;
            if (--timeout == 0)
                return LL_I2C_TIMEOUT;
        }
        buf[i] = (uint8_t)i2c->RXDR;
    }

    timeout = LL_I2C_DEFAULT_TIMEOUT;
    while (!(i2c->ISR & LL_I2C_ISR_STOPF)) {
        if (--timeout == 0)
            return LL_I2C_TIMEOUT;
    }
    i2c->ICR = LL_I2C_ICR_STOPCF;

    return LL_I2C_OK;
}

/* ============================================================
 * Register write/read helpers (common I2C device patterns)
 * ============================================================ */

/**
 * Write to a register on an I2C device.
 *   addr:  7-bit slave address
 *   reg:   register address (8-bit or 16-bit; if > 0xFF, sends MSB first)
 *   data:  pointer to data bytes
 *   len:   number of data bytes
 *
 * Returns LL_I2C_OK on success, LL_I2C_NACK, LL_I2C_ERROR, or
 * LL_I2C_TIMEOUT on failure.
 */
static inline int ll_i2c_write_reg(I2C_TypeDef *i2c, uint8_t addr,
                                   uint16_t reg, const uint8_t *data, uint32_t len)
{
    volatile uint32_t timeout;

    /* Wait for bus idle */
    timeout = LL_I2C_DEFAULT_TIMEOUT;
    while (i2c->ISR & LL_I2C_ISR_BUSY) {
        if (--timeout == 0) return LL_I2C_TIMEOUT;
    }

    i2c->ICR = LL_I2C_ICR_NACKCF | LL_I2C_ICR_STOPCF
             | LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;

    /* Register address is 1 byte if <= 0xFF, 2 bytes otherwise */
    uint32_t reg_len = (reg > 0xFF) ? 2 : 1;
    uint32_t total = reg_len + len;
    i2c->CR2 = ((uint32_t)addr << LL_I2C_CR2_SADD_SHIFT)
             | (total << LL_I2C_CR2_NBYTES_SHIFT)
             | LL_I2C_CR2_AUTOEND
             | LL_I2C_CR2_START;

    /* Send register address (MSB first if 16-bit) */
    if (reg > 0xFF) {
        timeout = LL_I2C_DEFAULT_TIMEOUT;
        while (!(i2c->ISR & LL_I2C_ISR_TXIS)) {
            uint32_t isr = i2c->ISR;
            if (isr & LL_I2C_ISR_NACKF) {
                i2c->ICR = LL_I2C_ICR_NACKCF;
                _ll_i2c_wait_stopf(i2c);
                return LL_I2C_NACK;
            }
            if (isr & (LL_I2C_ISR_BERR | LL_I2C_ISR_ARLO | LL_I2C_ISR_OVR)) {
                i2c->ICR = LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;
                _ll_i2c_wait_stopf(i2c);
                return LL_I2C_ERROR;
            }
            if (--timeout == 0)
                return LL_I2C_TIMEOUT;
        }
        i2c->TXDR = (uint8_t)(reg >> 8);
    }
    timeout = LL_I2C_DEFAULT_TIMEOUT;
    while (!(i2c->ISR & LL_I2C_ISR_TXIS)) {
        uint32_t isr = i2c->ISR;
        if (isr & LL_I2C_ISR_NACKF) {
            i2c->ICR = LL_I2C_ICR_NACKCF;
            _ll_i2c_wait_stopf(i2c);
            return LL_I2C_NACK;
        }
        if (isr & (LL_I2C_ISR_BERR | LL_I2C_ISR_ARLO | LL_I2C_ISR_OVR)) {
            i2c->ICR = LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;
            _ll_i2c_wait_stopf(i2c);
            return LL_I2C_ERROR;
        }
        if (--timeout == 0)
            return LL_I2C_TIMEOUT;
    }
    i2c->TXDR = (uint8_t)(reg & 0xFF);

    /* Send data */
    for (uint32_t i = 0; i < len; i++) {
        timeout = LL_I2C_DEFAULT_TIMEOUT;
        while (!(i2c->ISR & LL_I2C_ISR_TXIS)) {
            uint32_t isr = i2c->ISR;
            if (isr & LL_I2C_ISR_NACKF) {
                i2c->ICR = LL_I2C_ICR_NACKCF;
                _ll_i2c_wait_stopf(i2c);
                return LL_I2C_NACK;
            }
            if (isr & (LL_I2C_ISR_BERR | LL_I2C_ISR_ARLO | LL_I2C_ISR_OVR)) {
                i2c->ICR = LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;
                _ll_i2c_wait_stopf(i2c);
                return LL_I2C_ERROR;
            }
            if (--timeout == 0)
                return LL_I2C_TIMEOUT;
        }
        i2c->TXDR = data[i];
    }

    timeout = LL_I2C_DEFAULT_TIMEOUT;
    while (!(i2c->ISR & LL_I2C_ISR_STOPF)) {
        if (--timeout == 0)
            return LL_I2C_TIMEOUT;
    }
    i2c->ICR = LL_I2C_ICR_STOPCF;

    return LL_I2C_OK;
}

/**
 * Read from a register on an I2C device (write reg addr, then read).
 *   addr:  7-bit slave address
 *   reg:   register address (8-bit or 16-bit; if > 0xFF, sends MSB first)
 *   buf:   buffer for received data
 *   len:   number of bytes to read
 *
 * Returns LL_I2C_OK on success, LL_I2C_NACK, LL_I2C_ERROR, or
 * LL_I2C_TIMEOUT on failure.
 */
static inline int ll_i2c_read_reg(I2C_TypeDef *i2c, uint8_t addr,
                                  uint16_t reg, uint8_t *buf, uint32_t len)
{
    volatile uint32_t timeout;
    uint32_t isr;

    /* Wait for bus idle */
    timeout = LL_I2C_DEFAULT_TIMEOUT;
    while (i2c->ISR & LL_I2C_ISR_BUSY) {
        if (--timeout == 0) return LL_I2C_TIMEOUT;
    }

    i2c->ICR = LL_I2C_ICR_NACKCF | LL_I2C_ICR_STOPCF
             | LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;

    /* Phase 1: Write register address (no AUTOEND — we need a restart) */
    uint32_t reg_len = (reg > 0xFF) ? 2 : 1;
    i2c->CR2 = ((uint32_t)addr << LL_I2C_CR2_SADD_SHIFT)
             | (reg_len << LL_I2C_CR2_NBYTES_SHIFT)
             | LL_I2C_CR2_START;

    if (reg > 0xFF) {
        timeout = LL_I2C_DEFAULT_TIMEOUT;
        while (!(i2c->ISR & LL_I2C_ISR_TXIS)) {
            isr = i2c->ISR;
            if (isr & LL_I2C_ISR_NACKF) {
                i2c->ICR = LL_I2C_ICR_NACKCF;
                i2c->CR2 = LL_I2C_CR2_STOP;
                _ll_i2c_wait_stopf(i2c);
                return LL_I2C_NACK;
            }
            if (isr & (LL_I2C_ISR_BERR | LL_I2C_ISR_ARLO | LL_I2C_ISR_OVR)) {
                i2c->ICR = LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;
                i2c->CR2 = LL_I2C_CR2_STOP;
                _ll_i2c_wait_stopf(i2c);
                return LL_I2C_ERROR;
            }
            if (--timeout == 0) {
                i2c->CR2 = LL_I2C_CR2_STOP;
                _ll_i2c_wait_stopf(i2c);
                return LL_I2C_TIMEOUT;
            }
        }
        i2c->TXDR = (uint8_t)(reg >> 8);
    }
    timeout = LL_I2C_DEFAULT_TIMEOUT;
    while (!(i2c->ISR & LL_I2C_ISR_TXIS)) {
        isr = i2c->ISR;
        if (isr & LL_I2C_ISR_NACKF) {
            i2c->ICR = LL_I2C_ICR_NACKCF;
            i2c->CR2 = LL_I2C_CR2_STOP;
            _ll_i2c_wait_stopf(i2c);
            return LL_I2C_NACK;
        }
        if (isr & (LL_I2C_ISR_BERR | LL_I2C_ISR_ARLO | LL_I2C_ISR_OVR)) {
            i2c->ICR = LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;
            i2c->CR2 = LL_I2C_CR2_STOP;
            _ll_i2c_wait_stopf(i2c);
            return LL_I2C_ERROR;
        }
        if (--timeout == 0) {
            i2c->CR2 = LL_I2C_CR2_STOP;
            _ll_i2c_wait_stopf(i2c);
            return LL_I2C_TIMEOUT;
        }
    }
    i2c->TXDR = (uint8_t)(reg & 0xFF);

    /* Wait for transfer complete (TC, not TCR since no RELOAD) */
    timeout = LL_I2C_DEFAULT_TIMEOUT;
    while (!(i2c->ISR & LL_I2C_ISR_TC)) {
        isr = i2c->ISR;
        if (isr & LL_I2C_ISR_NACKF) {
            i2c->ICR = LL_I2C_ICR_NACKCF;
            i2c->CR2 = LL_I2C_CR2_STOP;
            _ll_i2c_wait_stopf(i2c);
            return LL_I2C_NACK;
        }
        if (isr & (LL_I2C_ISR_BERR | LL_I2C_ISR_ARLO | LL_I2C_ISR_OVR)) {
            i2c->ICR = LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;
            i2c->CR2 = LL_I2C_CR2_STOP;
            _ll_i2c_wait_stopf(i2c);
            return LL_I2C_ERROR;
        }
        if (--timeout == 0) {
            i2c->CR2 = LL_I2C_CR2_STOP;
            _ll_i2c_wait_stopf(i2c);
            return LL_I2C_TIMEOUT;
        }
    }

    /* Phase 2: Read data with repeated START */
    return ll_i2c_read(i2c, addr, buf, len);
}

/* ============================================================
 * Bus scan (useful for debugging)
 * ============================================================ */

/**
 * Check if a device responds at the given 7-bit address.
 * Returns LL_I2C_OK if ACK received, LL_I2C_NACK otherwise.
 */
static inline int ll_i2c_probe(I2C_TypeDef *i2c, uint8_t addr)
{
    /* Matches the ST HAL HAL_I2C_IsDeviceReady approach:
     * Send START + address (write, 0 bytes), NO AUTOEND.
     * Wait for either NACK (device absent) or TC (transfer complete,
     * meaning device ACKed). Then manually send STOP. */

    i2c->ICR = LL_I2C_ICR_NACKCF | LL_I2C_ICR_STOPCF
             | LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;

    /* Wait for any previous transaction to complete */
    volatile uint32_t timeout = 10000;
    while (i2c->ISR & LL_I2C_ISR_BUSY) {
        if (--timeout == 0) {
            i2c->CR1 = 0;
            for (volatile int d = 0; d < 100; d++);
            i2c->CR1 = LL_I2C_CR1_PE;
            return LL_I2C_TIMEOUT;
        }
    }

    /* START + address, write direction, 0 bytes, NO AUTOEND */
    i2c->CR2 = ((uint32_t)addr << LL_I2C_CR2_SADD_SHIFT)
             | (0UL << LL_I2C_CR2_NBYTES_SHIFT)
             | LL_I2C_CR2_START;

    /* Wait for NACK or TC (transfer complete = address ACKed) */
    timeout = 100000;
    while (1) {
        uint32_t isr = i2c->ISR;
        if (isr & LL_I2C_ISR_NACKF) {
            /* Device NACKed — clear flags, send STOP */
            i2c->ICR = LL_I2C_ICR_NACKCF;
            i2c->CR2 = LL_I2C_CR2_STOP;
            _ll_i2c_wait_stopf(i2c);
            return LL_I2C_NACK;
        }
        if (isr & (LL_I2C_ISR_BERR | LL_I2C_ISR_ARLO | LL_I2C_ISR_OVR)) {
            /* Bus error — clear flags, send STOP, reset PE */
            i2c->ICR = LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;
            i2c->CR2 = LL_I2C_CR2_STOP;
            _ll_i2c_wait_stopf(i2c);
            return LL_I2C_ERROR;
        }
        if (isr & LL_I2C_ISR_TC) {
            /* Device ACKed — send STOP */
            i2c->CR2 = LL_I2C_CR2_STOP;
            _ll_i2c_wait_stopf(i2c);
            return LL_I2C_OK;
        }
        if (--timeout == 0) {
            i2c->CR2 = LL_I2C_CR2_STOP;
            _ll_i2c_wait_stopf(i2c);
            i2c->CR1 = 0;
            for (volatile int d = 0; d < 100; d++);
            i2c->CR1 = LL_I2C_CR1_PE;
            return LL_I2C_TIMEOUT;
        }
    }
}

/* ============================================================
 * Target (device / slave) mode
 * ============================================================
 *
 * The same I2C IP on all four families can answer as a device on a
 * host's bus. Target mode is interrupt-driven by nature: the controller
 * sets the pace, and we answer inside the ISR.
 *
 * Clock stretching is deliberately left ENABLED (NOSTRETCH = 0). With
 * it on, the peripheral holds SCL low at each address match and at each
 * byte boundary until our ISR responds, so a late interrupt costs bus
 * throughput rather than corrupting data. Setting NOSTRETCH turns every
 * missed deadline into a silently wrong byte, so only disable it if the
 * controller cannot tolerate stretching AND every deadline is provably
 * met.
 *
 * See hal_i2c.c for the register-file target driver built on these.
 * ============================================================ */

/**
 * Initialize I2C as a target (device) at a 7-bit own address.
 *
 *   i2c:     I2C instance
 *   timing:  TIMINGR value (use LL_I2C_TIMING_* defines). A target does
 *            not drive SCL, but TIMINGR still sets SDADEL/SCLDEL, which
 *            govern data setup and hold. Use the constant matching the
 *            controller's speed.
 *   addr7:   7-bit own address, unshifted (e.g. 0x28)
 *
 * Enables address-match, RX, TX, NACK, STOP and error interrupts. The
 * caller still has to unmask the IRQ in the NVIC.
 */
static inline void ll_i2c_target_init(I2C_TypeDef *i2c, uint32_t timing,
                                      uint8_t addr7)
{
    /* PE must be 0 to write TIMINGR, and OA1EN must be 0 to change OA1. */
    i2c->CR1 = 0;
    i2c->TIMINGR = timing;

    i2c->OAR1 = 0;
    i2c->OAR1 = LL_I2C_OAR1_OA1EN
              | (((uint32_t)addr7 & LL_I2C_ISR_ADDCODE_MASK)
                 << LL_I2C_OAR1_OA1_SHIFT);
    i2c->OAR2 = 0;

    /* Drop anything latched from a previous life on the bus. */
    i2c->ICR = LL_I2C_ICR_ADDRCF | LL_I2C_ICR_NACKCF | LL_I2C_ICR_STOPCF
             | LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;

    i2c->CR1 = LL_I2C_CR1_PE
             | LL_I2C_CR1_ADDRIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_TXIE
             | LL_I2C_CR1_NACKIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_ERRIE;
}

/** Stop answering on the bus and release the own address. */
static inline void ll_i2c_target_disable(I2C_TypeDef *i2c)
{
    i2c->CR1 = 0;
    i2c->OAR1 = 0;
}

/**
 * Direction of the transfer the controller just addressed us for.
 * Returns 1 when the controller intends to READ from us (we transmit),
 * 0 when it intends to write. Only meaningful while ADDR is set.
 */
static inline uint32_t ll_i2c_target_is_read(I2C_TypeDef *i2c)
{
    return (i2c->ISR & LL_I2C_ISR_DIR) ? 1UL : 0UL;
}

/**
 * The 7-bit address that actually matched. Worth reading when OAR2 or
 * address masking puts more than one address on the same peripheral.
 */
static inline uint8_t ll_i2c_target_addcode(I2C_TypeDef *i2c)
{
    return (uint8_t)((i2c->ISR >> LL_I2C_ISR_ADDCODE_SHIFT)
                     & LL_I2C_ISR_ADDCODE_MASK);
}

/**
 * Discard a byte still sitting in TXDR. TXE is write-1-to-flush.
 *
 * Always do this before answering a read. A controller that abandoned a
 * previous read (or NACKed early) can leave a stale byte loaded, and
 * without a flush that byte is the first thing the next read receives,
 * shifting the whole transfer by one.
 */
static inline void ll_i2c_target_flush_tx(I2C_TypeDef *i2c)
{
    i2c->ISR = LL_I2C_ISR_TXE;
}

/** Clear the address match, which releases the stretched SCL. */
static inline void ll_i2c_target_clear_addr(I2C_TypeDef *i2c)
{
    i2c->ICR = LL_I2C_ICR_ADDRCF;
}

/* ============================================================
 * Enable / Disable
 * ============================================================ */

static inline void ll_i2c_enable(I2C_TypeDef *i2c)
{
    SET_BITS(i2c->CR1, LL_I2C_CR1_PE);
}

static inline void ll_i2c_disable(I2C_TypeDef *i2c)
{
    CLR_BITS(i2c->CR1, LL_I2C_CR1_PE);
}

#endif /* LL_I2C_H */
