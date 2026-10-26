/**
 * port_core_nvm_hw_prog.h — STM32 port: the hardware behind core_nvm.c (2 of 3)
 *
 * Included by core_nvm.c only, after its CRC-32 (see port_core_nvm_hw.h);
 * not for direct use. The flash emulation's erase of one page and program of
 * whole units: ll_flash, or ST's flash manager on a W5 running BLE or Thread
 * (sdk/wpan/wpan_flash.c), the 16-bit data-area programs on the H5, and the
 * cache upkeep after each (L4 data cache, W5 ICACHE).
 */

#ifndef PORT_CORE_NVM_HW_PROG_H
#define PORT_CORE_NVM_HW_PROG_H

#include "port_core_nvm_hw.h"

#if CORE_NVM_FLASH_EMU

/* ============================================================
 * Hardware: erase one page, program whole units
 * ============================================================ */

static void port_core_nvm_caches_after_erase(void)
{
#if defined(STM32L422xx)
    /* RM0394 §3.3.7 "Programming and caches": an erase is not reflected in the
     * data cache. Flush it (only allowed while it is disabled). The pages
     * hold no code, so the instruction cache never has them. */
    uint32_t acr = FLASH_ACR;
    if (acr & (1UL << 10)) {                        /* DCEN */
        FLASH_ACR = acr & ~(1UL << 10);
        FLASH_ACR = (acr & ~(1UL << 10)) | (1UL << 12);   /* DCRST */
        FLASH_ACR = acr & ~(1UL << 10);
        FLASH_ACR = acr;
    }
#endif
}

#if defined(STM32WBA55xx)
/* The ICACHE (enabled by BLE builds) sits on C-AHB and also caches data reads
 * of flash: invalidate it so a read-back sees the array (RM0493 §8.4.8). */
#define ICACHE_CR   REG32(0x40030400UL)
#define ICACHE_SR   REG32(0x40030404UL)
#define ICACHE_FCR  REG32(0x4003040CUL)
static void port_core_nvm_icache_invalidate(void)
{
    if (!(ICACHE_CR & 1UL)) return;                 /* EN */
    ICACHE_FCR = (1UL << 1);                        /* CBSYENDF */
    ICACHE_CR |= (1UL << 1);                        /* CACHEINV */
    while (!(ICACHE_SR & (1UL << 1)))               /* BSYENDF */
        ;
}
#else
static inline void port_core_nvm_icache_invalidate(void) { }
#endif

static int port_core_nvm_erase(uint32_t page)
{
    /* The L4 stalls ~22 ms here with every interrupt held off: feed the
     * watchdog first (a no-op if it isn't running). */
    ll_iwdg_refresh();
    int rc;
#if NVM_USE_FLASH_MANAGER
    if (wpan_flash_ready()) {
        rc = wpan_flash_erase_page(page);
    } else
#endif
    {
        ll_flash_unlock();
        rc = ll_flash_erase_page(page);
        ll_flash_lock();
    }
    port_core_nvm_caches_after_erase();
    port_core_nvm_icache_invalidate();
    return rc;
}

/* Program len bytes (a whole number of units) at a unit-aligned address. */
static int port_core_nvm_program(uint32_t addr, const uint8_t *src, uint32_t len)
{
    int rc = 0;
#if CORE_NVM_EDATA
    /* 16-bit words, each its own program (RM0481 §7.3.5). No interrupt
     * masking: nothing is left half-written between two words (there is no
     * write buffer for the data area), and a reset between them is a torn
     * record like a power cut. Code keeps running from bank 1 meanwhile. */
    ll_flash_unlock();
    for (uint32_t i = 0; i < len && rc == 0; i += 2u)
        rc = ll_flash_edata_program_half(addr + i, (uint16_t)(src[i] | ((uint16_t)src[i + 1u] << 8)));
    ll_flash_lock();
    return rc;
#else
#if NVM_USE_FLASH_MANAGER
    if (wpan_flash_ready()) {
        rc = wpan_flash_program(addr, (const uint32_t *)(const void *)src, len / 4u);
        port_core_nvm_icache_invalidate();
        return rc;
    }
#endif
    ll_flash_unlock();
    for (uint32_t i = 0; i < len && rc == 0; i += CORE_NVM_PROG_UNIT) {
        uint32_t w[CORE_NVM_PROG_UNIT / 4u];
        memcpy(w, src + i, CORE_NVM_PROG_UNIT);
        /* One unit is one program sequence: keep an interrupt from landing
         * between its words (an ISR could reset into the serial-update
         * flasher, for instance). ~82 us (L4) / ~118 us (W5). */
        uint32_t p = port_core_nvm_irq_save();
#if defined(STM32L422xx)
        rc = ll_flash_program_dword(addr + i, w[0], w[1]);
#else
        rc = ll_flash_program_qword(addr + i, w);
#endif
        port_core_nvm_irq_restore(p);
    }
    ll_flash_lock();
    port_core_nvm_icache_invalidate();
    return rc;
#endif
}

#endif /* CORE_NVM_FLASH_EMU */

#endif /* PORT_CORE_NVM_HW_PROG_H */
