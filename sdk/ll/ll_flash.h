/**
 * ll_flash.h — Low-level internal flash erase/program
 *
 * STM32L4 flash programs in double-words (8 bytes). Addresses must
 * be 8-byte aligned. Pages are 2KB on STM32L422 (64 pages, 128KB).
 *
 * Usage:
 *   ll_flash_unlock();
 *   ll_flash_erase_page(page_num);
 *   ll_flash_program_dword(addr, word0, word1);
 *   ll_flash_lock();
 */

#ifndef LL_FLASH_H
#define LL_FLASH_H

#include "ll_common.h"
#include "ll_rcc.h"  /* for FLASH_BASE */

/* ---- Register offsets ---- */

#define FLASH_KEYR          REG32(FLASH_BASE + 0x08UL)
#define FLASH_SR            REG32(FLASH_BASE + 0x10UL)
#define FLASH_CR            REG32(FLASH_BASE + 0x14UL)

/* ---- Unlock keys ---- */

#define FLASH_KEY1          0x45670123UL
#define FLASH_KEY2          0xCDEF89ABUL

/* ---- SR bits ---- */

#define FLASH_SR_EOP        (1UL << 0)
#define FLASH_SR_OPERR      (1UL << 1)
#define FLASH_SR_PROGERR    (1UL << 3)
#define FLASH_SR_WRPERR     (1UL << 4)
#define FLASH_SR_PGAERR     (1UL << 5)
#define FLASH_SR_SIZERR     (1UL << 6)
#define FLASH_SR_PGSERR     (1UL << 7)
#define FLASH_SR_MISERR     (1UL << 8)
#define FLASH_SR_FASTERR    (1UL << 9)
#define FLASH_SR_BSY        (1UL << 16)

#define FLASH_SR_ERR_MASK   (FLASH_SR_OPERR | FLASH_SR_PROGERR | \
                             FLASH_SR_WRPERR | FLASH_SR_PGAERR | \
                             FLASH_SR_SIZERR | FLASH_SR_PGSERR | \
                             FLASH_SR_MISERR | FLASH_SR_FASTERR)

/* ---- CR bits ---- */

#define FLASH_CR_PG         (1UL << 0)
#define FLASH_CR_PER        (1UL << 1)
#define FLASH_CR_MER1       (1UL << 2)
#define FLASH_CR_PNB_SHIFT  3
#define FLASH_CR_PNB_MASK   (0xFFUL << FLASH_CR_PNB_SHIFT)
#define FLASH_CR_STRT       (1UL << 16)
#define FLASH_CR_LOCK       (1UL << 31)

/* ---- Page geometry ---- */

#if defined(STM32L422xx)
  #define FLASH_PAGE_SIZE   2048
  #define FLASH_START       0x08000000UL
#elif defined(STM32WBA55xx)
  #define FLASH_PAGE_SIZE   8192     /* 8 KB pages */
  #define FLASH_START       0x08000000UL
  #define FLASH_TOTAL_SIZE  (1024 * 1024)  /* 1 MB */
  #define FLASH_PAGE_COUNT  (FLASH_TOTAL_SIZE / FLASH_PAGE_SIZE)  /* 128 pages */

  /* WBA55 uses non-secure registers at different offsets */
  #undef  FLASH_KEYR
  #undef  FLASH_SR
  #undef  FLASH_CR
  #define FLASH_KEYR        REG32(FLASH_BASE + 0x08UL)  /* NSKEYR */
  #define FLASH_SR          REG32(FLASH_BASE + 0x20UL)  /* NSSR */
  #define FLASH_CR          REG32(FLASH_BASE + 0x28UL)  /* NSCR1 */

  /* NSSR (RM0493 §7.9.8): bits 8 and 9 (MISERR / FASTERR on the L4) are
   * reserved here, and OPTWERR is bit 13. WDW (bit 17) is set while a
   * quad-word is only partly written into the write buffer. */
  #undef  FLASH_SR_MISERR
  #undef  FLASH_SR_FASTERR
  #undef  FLASH_SR_ERR_MASK
  #define FLASH_SR_OPTWERR  (1UL << 13)
  #define FLASH_SR_WDW      (1UL << 17)
  #define FLASH_SR_ERR_MASK (FLASH_SR_OPERR | FLASH_SR_PROGERR | \
                             FLASH_SR_WRPERR | FLASH_SR_PGAERR | \
                             FLASH_SR_SIZERR | FLASH_SR_PGSERR | \
                             FLASH_SR_OPTWERR)

  /* NSCR1 PNB is 7 bits, [9:3] (RM0493 §7.9.10) */
  #undef  FLASH_CR_PNB_MASK
  #define FLASH_CR_PNB_MASK (0x7FUL << FLASH_CR_PNB_SHIFT)
#elif defined(STM32H523xx)
  #define FLASH_PAGE_SIZE   8192     /* 8 KB sectors */
  #define FLASH_START       0x08000000UL
  #define FLASH_TOTAL_SIZE  (512 * 1024)   /* 512 KB */
  #define FLASH_PAGE_COUNT  (FLASH_TOTAL_SIZE / FLASH_PAGE_SIZE)  /* 64 sectors */

  /* H5 non-secure flash registers (different offsets + bit positions) */
  #undef  FLASH_KEYR
  #undef  FLASH_SR
  #undef  FLASH_CR

  #define FLASH_KEYR        REG32(FLASH_BASE + 0x04UL)  /* NSKEYR */
  #define FLASH_SR          REG32(FLASH_BASE + 0x20UL)  /* NSSR */
  #define FLASH_CR          REG32(FLASH_BASE + 0x28UL)  /* NSCR */
  #define FLASH_CCR         REG32(FLASH_BASE + 0x30UL)  /* NSCCR (clear flags) */

  /* H5 SR bits are different from L4 */
  #undef  FLASH_SR_EOP
  #undef  FLASH_SR_OPERR
  #undef  FLASH_SR_PROGERR
  #undef  FLASH_SR_WRPERR
  #undef  FLASH_SR_PGAERR
  #undef  FLASH_SR_SIZERR
  #undef  FLASH_SR_PGSERR
  #undef  FLASH_SR_MISERR
  #undef  FLASH_SR_FASTERR
  #undef  FLASH_SR_BSY
  #undef  FLASH_SR_ERR_MASK

  #define FLASH_SR_BSY        (1UL << 0)
  #define FLASH_SR_EOP        (1UL << 16)
  #define FLASH_SR_WRPERR     (1UL << 17)
  #define FLASH_SR_PGSERR     (1UL << 18)
  #define FLASH_SR_STRBERR    (1UL << 19)
  #define FLASH_SR_INCERR     (1UL << 20)
  #define FLASH_SR_ERR_MASK   (FLASH_SR_WRPERR | FLASH_SR_PGSERR | \
                               FLASH_SR_STRBERR | FLASH_SR_INCERR)

  /* H5 CR bits are different from L4 */
  #undef  FLASH_CR_PG
  #undef  FLASH_CR_PER
  #undef  FLASH_CR_MER1
  #undef  FLASH_CR_PNB_SHIFT
  #undef  FLASH_CR_PNB_MASK
  #undef  FLASH_CR_STRT
  #undef  FLASH_CR_LOCK

  #define FLASH_CR_LOCK       (1UL << 0)
  #define FLASH_CR_PG         (1UL << 1)
  #define FLASH_CR_SER        (1UL << 2)    /* Sector erase (was PER on L4) */
  #define FLASH_CR_PER        FLASH_CR_SER  /* Alias for compatibility */
  #define FLASH_CR_FW         (1UL << 4)    /* Force write */
  #define FLASH_CR_START      (1UL << 5)    /* Start erase (was STRT on L4) */
  #define FLASH_CR_STRT       FLASH_CR_START /* Alias for compatibility */
  #define FLASH_CR_SNB_SHIFT  6
  #define FLASH_CR_SNB_MASK   (0x7FUL << FLASH_CR_SNB_SHIFT)
  #define FLASH_CR_PNB_SHIFT  FLASH_CR_SNB_SHIFT  /* Alias */
  #define FLASH_CR_PNB_MASK   FLASH_CR_SNB_MASK   /* Alias */
#endif

/* ============================================================
 * Functions
 * ============================================================ */

/** Unlock flash for erase/program. */
static inline void ll_flash_unlock(void)
{
    if (FLASH_CR & FLASH_CR_LOCK) {
        FLASH_KEYR = FLASH_KEY1;
        FLASH_KEYR = FLASH_KEY2;
    }
}

/** Lock flash (re-enable write protection). */
static inline void ll_flash_lock(void)
{
    SET_BITS(FLASH_CR, FLASH_CR_LOCK);
}

/** Spin until flash is not busy. */
static inline void ll_flash_wait_bsy(void)
{
    while (FLASH_SR & FLASH_SR_BSY)
        ;
}

/** Clear all error flags. */
static inline void ll_flash_clear_errors(void)
{
#if defined(STM32H523xx)
    /* H5 uses a separate clear register (NSCCR) */
    FLASH_CCR = FLASH_SR_ERR_MASK | FLASH_SR_EOP;
#else
    FLASH_SR = FLASH_SR_ERR_MASK | FLASH_SR_EOP;
#endif
}

/**
 * Erase a single flash page.
 * Returns 0 on success, -1 on error.
 */
static inline int ll_flash_erase_page(uint32_t page)
{
    ll_flash_wait_bsy();
    ll_flash_clear_errors();

#if defined(STM32WBA55xx)
    /* RM0493 §7.3.6: PER + PNB first, then STRT. NSCR1 can't be written while
     * BSY is set (a bus error), hence the wait above. */
    MOD_BITS(FLASH_CR, FLASH_CR_PG | FLASH_CR_MER1 | FLASH_CR_PNB_MASK,
             FLASH_CR_PER | ((page << FLASH_CR_PNB_SHIFT) & FLASH_CR_PNB_MASK));
    SET_BITS(FLASH_CR, FLASH_CR_STRT);
#else
    /* Set sector/page erase + number, then start */
    FLASH_CR = FLASH_CR_PER | (page << FLASH_CR_PNB_SHIFT) | FLASH_CR_STRT;
#endif

    ll_flash_wait_bsy();

    /* Clear erase bits */
    CLR_BITS(FLASH_CR, FLASH_CR_PER | FLASH_CR_STRT);

    if (FLASH_SR & FLASH_SR_ERR_MASK)
        return -1;

    return 0;
}

/**
 * Program a double-word (8 bytes) at an 8-byte aligned address.
 * Returns 0 on success, -1 on error.
 */
static inline int ll_flash_program_dword(uint32_t addr, uint32_t word0, uint32_t word1)
{
    ll_flash_wait_bsy();
    ll_flash_clear_errors();

    /* Enable programming */
    SET_BITS(FLASH_CR, FLASH_CR_PG);

    /* Write the two 32-bit words (must be in order, low then high) */
    *(volatile uint32_t *)(addr)     = word0;
    *(volatile uint32_t *)(addr + 4) = word1;

    ll_flash_wait_bsy();

    /* Clear PG */
    CLR_BITS(FLASH_CR, FLASH_CR_PG);

    if (FLASH_SR & FLASH_SR_ERR_MASK)
        return -1;

    return 0;
}

#if defined(STM32WBA55xx) || defined(STM32H523xx)
/**
 * Program a quad-word (128-bit / 16 bytes) at a 16-byte aligned address.
 *
 * WBA55 and H5 program in 128-bit lines: the flash controller only completes
 * (and clears BSY) once all four 32-bit words have been written, so all four
 * must be issued before the busy wait — a partial write never completes. This
 * is also the native granularity of the OTP region, so it serves both main
 * flash and OTP (same PG bit + NSKEYR unlock; only the target address differs).
 *
 * Returns 0 on success, -1 on error.
 */
static inline int ll_flash_program_qword(uint32_t addr, const uint32_t w[4])
{
    ll_flash_wait_bsy();
    ll_flash_clear_errors();

    /* Enable programming */
    SET_BITS(FLASH_CR, FLASH_CR_PG);

    /* Write the full 128-bit line, low word first; the program completes on
     * the fourth write. */
    *(volatile uint32_t *)(addr + 0)  = w[0];
    *(volatile uint32_t *)(addr + 4)  = w[1];
    *(volatile uint32_t *)(addr + 8)  = w[2];
    *(volatile uint32_t *)(addr + 12) = w[3];

#if defined(STM32WBA55xx)
    /* RM0493 §7.3.7 step 7: WDW clears first (the buffer is handed to the
     * array and BSY rises), then BSY. Checking BSY alone can catch the gap. */
    while (FLASH_SR & FLASH_SR_WDW)
        ;
#endif
    ll_flash_wait_bsy();

    /* Clear PG */
    CLR_BITS(FLASH_CR, FLASH_CR_PG);

    if (FLASH_SR & FLASH_SR_ERR_MASK)
        return -1;

    return 0;
}
#endif

#endif /* LL_FLASH_H */
