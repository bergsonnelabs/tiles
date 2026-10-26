/**
 * port_core_uid.h — STM32 port: the bodies of core_uid.h
 *
 * Included at the end of core_uid.h, after its prototypes; not for direct
 * use. Reads the factory 96-bit unique ID from each MCU's UID_BASE.
 */

#ifndef PORT_CORE_UID_H
#define PORT_CORE_UID_H

#include <stdint.h>

/* Base address of the 96-bit (3 x 32-bit) unique device ID, per MCU. Each
 * value is taken from that part's CMSIS device header (UID_BASE) / reference
 * manual — do not guess these. */
#if defined(STM32L422xx)
#define CORE_UID_BASE 0x1FFF7590UL   /* RM0394 */
#elif defined(STM32WBA55xx)
#define CORE_UID_BASE 0x0BF90700UL   /* RM0493 */
#elif defined(STM32H523xx)
#define CORE_UID_BASE 0x08FFF800UL   /* RM0481 */
#elif defined(STM32L011xx)
#define CORE_UID_BASE 0x1FF80050UL   /* RM0377 */
#else
#error "core_uid.h: UID base address not defined for this Core MCU"
#endif

static inline void core_uid_read(uint32_t out[3])
{
    const volatile uint32_t *uid = (const volatile uint32_t *)CORE_UID_BASE;
#if defined(STM32H523xx)
    /* The H5's UID is in the flash's read-only area (RM0481 §7.3.9: 16/32-bit
     * reads only, parsed from 137-bit ECC words). That area is in the code
     * region the ICACHE caches (cacheability is the MPU's call, RM0481
     * §7.6.12, and the MPU is off), and a read of it through the ICACHE is a
     * precise bus fault: seen on the bench 2026-09-26 at 0x08FFF80C (BFAR).
     * So read it with the ICACHE off. ICACHE_CR (0x40030400) bit 0 = EN; clearing
     * it invalidates, and re-enabling waits for that to end (ICACHE_SR.BUSYF). */
    volatile uint32_t *icr = (volatile uint32_t *)0x40030400UL;
    uint32_t ic = *icr & 1UL;
    if (ic) *icr &= ~1UL;
    out[0] = uid[0];
    out[1] = uid[1];
    out[2] = uid[2];
    if (ic) {
        for (uint32_t t = 100000UL; (*(volatile uint32_t *)0x40030404UL & 1UL) && t; t--) { }
        *icr |= 1UL;
    }
    return;
#endif
    out[0] = uid[0];
    out[1] = uid[1];
#if defined(STM32L011xx)
    /* The L0's third word is not contiguous: offsets 0x00, 0x04 and 0x14
     * (0x1FF80064), RM0377 §28.2. Offset 0x08 is not part of the ID. */
    out[2] = uid[5];
#else
    out[2] = uid[2];
#endif
}

static inline uint32_t core_uid_hash(void)
{
    uint32_t w[3];
    core_uid_read(w);
    return w[0] ^ w[1] ^ w[2];
}

static inline int core_uid_hex(char *buf, uint16_t buflen)
{
    static const char hexd[] = "0123456789ABCDEF";
    uint32_t w[3];
    int p = 0;

    if (!buf || buflen < 25) return -1;

    core_uid_read(w);
    for (int i = 0; i < 3; i++)
        for (int shift = 28; shift >= 0; shift -= 4)
            buf[p++] = hexd[(w[i] >> shift) & 0xF];
    buf[p] = '\0';
    return p;
}

#endif /* PORT_CORE_UID_H */
