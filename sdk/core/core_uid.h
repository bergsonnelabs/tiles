/**
 * core_uid.h — Unique device identifier (96-bit factory UID)
 *
 * Reads the STM32 factory-programmed 96-bit unique device ID. It is set at
 * manufacture, identical for the life of the chip, and read-only — so it makes
 * a good per-device serial number, a stable seed for a unique advertised name,
 * or a basis for per-device keys. No initialization is required; the ID lives
 * in a read-only region of the system memory.
 *
 * Available on every Core (Core.ST.L0 / L4 / W5 / H5) — each MCU exposes the
 * same 96-bit ID, just at a different base address (handled internally).
 *
 * Usage:
 *   uint32_t w[3];
 *   core_uid_read(w);                       // the three 32-bit ID words
 *   uint32_t tok = core_uid_hash();         // stable 32-bit fold of the full ID
 *   char serial[25];
 *   core_uid_hex(serial, sizeof serial);    // "1A2B3C4D…" — 24 hex chars + NUL
 *
 * @studio category uid label=Core.UID icon=🔖
 *
 * @studio coverage
 *   id:    uid
 *   name:  UID — unique device identifier
 *   blurb: Reads the STM32 factory 96-bit unique device ID (read-only, no init)
 *          on every Core. Handy for per-device serial numbers, unique BLE
 *          advertised names, and per-device key/seed derivation. Tier 1 only —
 *          a plain register read with no DSL binding or Twin model yet.
 */

#ifndef CORE_UID_H
#define CORE_UID_H

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

/**
 * Read the 96-bit unique device ID as three 32-bit words.
 *
 * @param out  Destination for the three words: out[0] is the first UID word,
 *             out[2] the last. Must point to at least 3 uint32_t.
 */
static inline void core_uid_read(uint32_t out[3])
{
    const volatile uint32_t *uid = (const volatile uint32_t *)CORE_UID_BASE;
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

/**
 * Fold the full 96-bit ID into a single stable 32-bit value (the three words
 * XORed together). Better mixed than any single word — use it to derive short
 * tokens (e.g. a 4-hex-digit name suffix) where the raw low bytes might be
 * correlated across a manufacturing batch.
 *
 * @return  A 32-bit value, stable for the life of the chip.
 */
static inline uint32_t core_uid_hash(void)
{
    uint32_t w[3];
    core_uid_read(w);
    return w[0] ^ w[1] ^ w[2];
}

/**
 * Format the full 96-bit ID as an uppercase hex string (24 hex characters,
 * word 0 first, most-significant nibble first within each word) plus a NUL
 * terminator. The canonical form for a device serial number.
 *
 * @param buf     Destination buffer.
 * @param buflen  Size of buf; must be at least 25.
 * @return        Number of hex characters written (24), or -1 if buf is NULL
 *                or buflen is too small.
 */
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

#endif /* CORE_UID_H */
