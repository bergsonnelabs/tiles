/**
 * ll_itm.h — ITM (Instrumentation Trace Macrocell) for SWO printf
 *
 * Sends debug text over the SWD trace pin (SWO) to the debug
 * probe. No UART adapter needed — uses the same cable you flash with.
 *
 * Available on Cortex-M3, M4, M33 (Core.ST.L4, Core.ST.W5, Core.ST.H5).
 * NOT available on Cortex-M0+ (Core.ST.L0 — no ITM hardware).
 *
 * Usage:
 *   ll_itm_init(80000000);       // Call once with SYSCLK
 *   ll_itm_putc('H');            // Send a character
 *   ll_itm_puts("Hello\r\n");   // Send a string
 *
 * To capture output, run OpenOCD with SWO enabled, or use
 * STM32CubeIDE's SWV console.
 */

#ifndef LL_ITM_H
#define LL_ITM_H

#include "ll_common.h"

#if !defined(STM32L011xx)  /* M0+ has no ITM */

/* ---- Core Debug / ITM registers ---- */

#define ITM_BASE            0xE0000000UL
#define ITM_STIM0           REG32(ITM_BASE + 0x000UL)  /* Stimulus port 0 */
#define ITM_TER             REG32(ITM_BASE + 0xE00UL)  /* Trace enable */
#define ITM_TCR             REG32(ITM_BASE + 0xE80UL)  /* Trace control */
#define ITM_LAR             REG32(ITM_BASE + 0xFB0UL)  /* Lock access */

#define TPIU_BASE           0xE0040000UL
#define TPIU_ACPR           REG32(TPIU_BASE + 0x010UL) /* Async clock prescaler */
#define TPIU_SPPR           REG32(TPIU_BASE + 0x0F0UL) /* Selected pin protocol */
#define TPIU_FFCR           REG32(TPIU_BASE + 0x304UL) /* Formatter and flush */

/* The trace-pin enable in DBGMCU differs per Core, and so does DBGMCU's
 * address: 0xE0042000 is DBGMCU only on the Cortex-M4 (L4); on both M33 parts
 * it is the CoreSight CTI block.
 *   L4  (RM0394 §47.16.3): DBGMCU_CR at 0xE0042004, TRACE_IOEN = bit 5,
 *       TRACE_MODE[7:6] = 00 (asynchronous, TRACESWO).
 *   H5  (RM0481 §59.12.4): DBGMCU_CR at 0x44024004 (the software base; the
 *       debugger sees it at 0xE00E4004). TRACE_IOEN = bit 4, TRACE_EN = bit 5
 *       (trace port clock), TRACE_MODE[7:6] = 00 asynchronous. ST's own SWO
 *       setup for the H5 writes 0x30 (both bits).
 *   W5  (RM0493 §43.12.7): DBGMCU_SCR has no trace bits (bit 0 reserved, 1
 *       DBG_STOP, 2 DBG_STANDBY), and PB3 is JTDO/TRACESWO from reset
 *       (§43.2.2, §14.4.1), so there is nothing to enable. */
#if defined(STM32L422xx)
  #define LL_ITM_DBGMCU_CR    REG32(0xE0042004UL)
  #define LL_ITM_TRACE_BITS   (1UL << 5)                  /* TRACE_IOEN */
#elif defined(STM32H523xx)
  #define LL_ITM_DBGMCU_CR    REG32(0x44024004UL)
  #define LL_ITM_TRACE_BITS   ((1UL << 4) | (1UL << 5))   /* TRACE_IOEN | TRACE_EN */
#endif

/* ============================================================
 * Initialization
 * ============================================================ */

/**
 * Initialize ITM for SWO printf output.
 *   sysclk_hz:  system clock frequency (e.g., 80000000)
 *   swo_baud:   desired SWO baud rate (2000000 is typical)
 *
 * The SWO pin (TRACESWO) must be available — on Core.ST.L4.2 this
 * is PB3 (Pad 12), but it's directly connected to the ST-Link
 * trace input, so no GPIO config is needed.
 *
 * Note: The debugger (OpenOCD / ST-Link) must also be configured
 * to capture SWO at the matching baud rate.
 */
static inline void ll_itm_init(uint32_t sysclk_hz, uint32_t swo_baud)
{
    /* Enable trace output in DBGMCU (TRACE_MODE stays 00: asynchronous) */
#ifdef LL_ITM_DBGMCU_CR
    SET_BITS(LL_ITM_DBGMCU_CR, LL_ITM_TRACE_BITS);
#endif

    /* Unlock ITM */
    ITM_LAR = 0xC5ACCE55UL;

    /* Configure TPIU */
    TPIU_ACPR = (sysclk_hz / swo_baud) - 1;  /* Async clock prescaler */
    TPIU_SPPR = 2;                             /* NRZ (UART-like) protocol */
    TPIU_FFCR = 0x100;                        /* ENFCONT = 0: formatter bypassed (ITM only) */

    /* Enable ITM */
    ITM_TCR = (1UL << 0)    /* ITMENA: enable ITM */
            | (1UL << 3)    /* TXENA: enable hardware event forwarding */
            | (1UL << 16);  /* TraceBusID = 1 */

    /* Enable stimulus port 0 */
    ITM_TER = 1;
}

/**
 * Convenience: init with default 2MHz SWO baud rate.
 */
static inline void ll_itm_init_default(uint32_t sysclk_hz)
{
    ll_itm_init(sysclk_hz, 2000000);
}

/* ============================================================
 * Output
 * ============================================================ */

/**
 * Send a single character via ITM stimulus port 0.
 * Blocks if the port is busy (very briefly — SWO runs at MHz).
 */
static inline void ll_itm_putc(char c)
{
    /* Skip the byte unless the trace block is on (DEMCR.TRCENA, set by a
     * debugger or by core_timing's DWT setup) and ITM and port 0 are enabled,
     * as CMSIS ITM_SendChar does. Otherwise STIM0 never reports ready and
     * debug prints hang a Core with no probe attached (seen on the H5,
     * 2026-09-29: STIM0 read 0x2 with TRCENA clear). */
    if (!(REG32(0xE000EDFCUL) & (1UL << 24)) || !(ITM_TCR & 1UL) || !(ITM_TER & 1UL))
        return;
    /* Wait for stimulus port 0 to be ready (bit 0 of STIM0) */
    while (!(ITM_STIM0 & 1))
        ;
    /* Write character (only the low byte matters) */
    *(volatile uint8_t *)&ITM_STIM0 = (uint8_t)c;
}

/** Send a null-terminated string via ITM. */
static inline void ll_itm_puts(const char *str)
{
    while (*str) {
        ll_itm_putc(*str++);
    }
}

/** Send a buffer of bytes via ITM. */
static inline void ll_itm_write(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        ll_itm_putc((char)data[i]);
    }
}

#endif /* !STM32L011xx */
#endif /* LL_ITM_H */
