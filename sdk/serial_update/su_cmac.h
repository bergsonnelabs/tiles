/**
 * su_cmac.h — AES-128 (encrypt only) and AES-CMAC (RFC 4493), portable C
 *
 * The BLE update (docs/ble-update-protocol.md §5) authenticates the host and
 * the image with a per-board key. Software, not the WBA55's AES peripheral:
 * the peripheral is shared with the BLE stack (sdk/ble/baes.h), the SDK's
 * driver for it (sdk/ble/hw_aes.c) still guesses its byte order at run time,
 * and the same code then runs in the host tests. ~10-15 ms per 2 KB chunk at
 * 32 MHz is affordable.
 *
 * No libc, no tables in RAM beyond the context. Not hardened against timing
 * or power analysis (the S-box is a table lookup): it protects the radio
 * path, and anyone with the board in hand has SWD anyway.
 */

#ifndef SU_CMAC_H
#define SU_CMAC_H

#include <stdint.h>

typedef struct {
    uint8_t rk[176];                 /* expanded key, 11 round keys */
} su_aes128_t;

void su_aes128_init(su_aes128_t *a, const uint8_t key[16]);
void su_aes128_encrypt(const su_aes128_t *a, const uint8_t in[16], uint8_t out[16]);

typedef struct {
    su_aes128_t aes;
    uint8_t     k1[16], k2[16];      /* subkeys */
    uint8_t     x[16];               /* CBC-MAC chaining value */
    uint8_t     buf[16];             /* pending block, never processed until more data */
    uint32_t    n;                   /* bytes in buf, 0..16 */
} su_cmac_t;

void su_cmac_init(su_cmac_t *c, const uint8_t key[16]);
void su_cmac_update(su_cmac_t *c, const uint8_t *p, uint32_t len);
/** Finish and write the 16-byte tag. The context must be re-initialised after. */
void su_cmac_final(su_cmac_t *c, uint8_t tag[16]);

/* ---- BLE update tags (docs/ble-update-protocol.md §5) ---- */

/** auth_tag = CMAC(K, "SUA1" ‖ nonce) */
void su_tag_auth(const uint8_t key[16], const uint8_t nonce[16], uint8_t out[16]);
/** Start image_tag = CMAC(K, "SUI1" ‖ LE32(size) ‖ LE32(crc) ‖ image): feed
 *  the image with su_cmac_update(), finish with su_cmac_final(). */
void su_tag_image_start(su_cmac_t *c, const uint8_t key[16], uint32_t size, uint32_t crc);
/** end_tag = CMAC(K, "SUE1" ‖ nonce ‖ image_tag) */
void su_tag_end(const uint8_t key[16], const uint8_t nonce[16], const uint8_t image_tag[16], uint8_t out[16]);

/** Constant-time compare: 1 if equal. */
int su_ct_equal(const uint8_t *a, const uint8_t *b, uint32_t n);

/** Overwrite n bytes with zeros (not optimised away). */
void su_wipe(void *p, uint32_t n);

#endif /* SU_CMAC_H */
