/**
 * su_cmac.c — AES-128 (FIPS 197, encrypt only) and AES-CMAC (RFC 4493)
 *
 * See su_cmac.h. Byte-oriented and table-light (the S-box only): small, and
 * the same on the host tests and the Core. Checked against FIPS 197
 * Appendix C.1 and the four RFC 4493 §4 vectors (sdk/serial_update/test).
 */

#include "su_cmac.h"

static const uint8_t sbox[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16,
};

static uint8_t xtime(uint8_t b)
{
    return (uint8_t)((b << 1) ^ ((b & 0x80u) ? 0x1Bu : 0x00u));
}

void su_aes128_init(su_aes128_t *a, const uint8_t key[16])
{
    uint8_t *w = a->rk;
    uint8_t rcon = 0x01;
    for (int i = 0; i < 16; i++) w[i] = key[i];
    for (int i = 16; i < 176; i += 4) {
        uint8_t t0 = w[i - 4], t1 = w[i - 3], t2 = w[i - 2], t3 = w[i - 1];
        if ((i & 15) == 0) {
            /* RotWord, SubWord, Rcon */
            uint8_t u = t0;
            t0 = (uint8_t)(sbox[t1] ^ rcon);
            t1 = sbox[t2];
            t2 = sbox[t3];
            t3 = sbox[u];
            rcon = xtime(rcon);
        }
        w[i + 0] = (uint8_t)(w[i - 16] ^ t0);
        w[i + 1] = (uint8_t)(w[i - 15] ^ t1);
        w[i + 2] = (uint8_t)(w[i - 14] ^ t2);
        w[i + 3] = (uint8_t)(w[i - 13] ^ t3);
    }
}

void su_aes128_encrypt(const su_aes128_t *a, const uint8_t in[16], uint8_t out[16])
{
    uint8_t s[16];
    const uint8_t *rk = a->rk;
    for (int i = 0; i < 16; i++) s[i] = (uint8_t)(in[i] ^ rk[i]);

    for (int round = 1; round <= 10; round++) {
        uint8_t t[16];
        /* SubBytes + ShiftRows: state is column-major, s[c*4 + r]. */
        for (int c = 0; c < 4; c++)
            for (int r = 0; r < 4; r++)
                t[c * 4 + r] = sbox[s[((c + r) & 3) * 4 + r]];
        if (round < 10) {
            /* MixColumns */
            for (int c = 0; c < 4; c++) {
                uint8_t a0 = t[c * 4], a1 = t[c * 4 + 1], a2 = t[c * 4 + 2], a3 = t[c * 4 + 3];
                uint8_t all = (uint8_t)(a0 ^ a1 ^ a2 ^ a3);
                s[c * 4 + 0] = (uint8_t)(a0 ^ all ^ xtime((uint8_t)(a0 ^ a1)));
                s[c * 4 + 1] = (uint8_t)(a1 ^ all ^ xtime((uint8_t)(a1 ^ a2)));
                s[c * 4 + 2] = (uint8_t)(a2 ^ all ^ xtime((uint8_t)(a2 ^ a3)));
                s[c * 4 + 3] = (uint8_t)(a3 ^ all ^ xtime((uint8_t)(a3 ^ a0)));
            }
        } else {
            for (int i = 0; i < 16; i++) s[i] = t[i];
        }
        rk += 16;
        for (int i = 0; i < 16; i++) s[i] ^= rk[i];
    }
    for (int i = 0; i < 16; i++) out[i] = s[i];
    su_wipe(s, sizeof(s));
}

/* ============================================================
 * CMAC (RFC 4493)
 * ============================================================ */

static void dbl(const uint8_t in[16], uint8_t out[16])
{
    uint8_t carry = (uint8_t)(in[0] >> 7);
    for (int i = 0; i < 15; i++) out[i] = (uint8_t)((in[i] << 1) | (in[i + 1] >> 7));
    out[15] = (uint8_t)((in[15] << 1) ^ (carry ? 0x87u : 0x00u));
}

void su_cmac_init(su_cmac_t *c, const uint8_t key[16])
{
    uint8_t l[16];
    su_aes128_init(&c->aes, key);
    for (int i = 0; i < 16; i++) { l[i] = 0; c->x[i] = 0; c->buf[i] = 0; }
    su_aes128_encrypt(&c->aes, l, l);
    dbl(l, c->k1);
    dbl(c->k1, c->k2);
    su_wipe(l, sizeof(l));
    c->n = 0;
}

static void absorb(su_cmac_t *c, const uint8_t blk[16])
{
    for (int i = 0; i < 16; i++) c->x[i] ^= blk[i];
    su_aes128_encrypt(&c->aes, c->x, c->x);
}

void su_cmac_update(su_cmac_t *c, const uint8_t *p, uint32_t len)
{
    while (len) {
        /* A full pending block is processed only once more data arrives: the
         * last block of the message gets the subkey treatment instead. */
        if (c->n == 16) {
            absorb(c, c->buf);
            c->n = 0;
        }
        uint32_t take = 16u - c->n;
        if (take > len) take = len;
        for (uint32_t i = 0; i < take; i++) c->buf[c->n + i] = p[i];
        c->n += take;
        p += take;
        len -= take;
    }
}

void su_cmac_final(su_cmac_t *c, uint8_t tag[16])
{
    uint8_t last[16];
    if (c->n == 16) {
        for (int i = 0; i < 16; i++) last[i] = (uint8_t)(c->buf[i] ^ c->k1[i]);
    } else {
        for (int i = 0; i < 16; i++) {
            uint8_t b = (i < (int)c->n) ? c->buf[i] : (i == (int)c->n ? 0x80u : 0x00u);
            last[i] = (uint8_t)(b ^ c->k2[i]);
        }
    }
    absorb(c, last);
    for (int i = 0; i < 16; i++) tag[i] = c->x[i];
    su_wipe(last, sizeof(last));
    su_wipe(c, sizeof(*c));
}

/* ============================================================
 * BLE update tags
 * ============================================================ */

void su_tag_auth(const uint8_t key[16], const uint8_t nonce[16], uint8_t out[16])
{
    su_cmac_t c;
    su_cmac_init(&c, key);
    su_cmac_update(&c, (const uint8_t *)"SUA1", 4);
    su_cmac_update(&c, nonce, 16);
    su_cmac_final(&c, out);
}

void su_tag_image_start(su_cmac_t *c, const uint8_t key[16], uint32_t size, uint32_t crc)
{
    uint8_t h[12] = { 'S', 'U', 'I', '1',
                      (uint8_t)size, (uint8_t)(size >> 8), (uint8_t)(size >> 16), (uint8_t)(size >> 24),
                      (uint8_t)crc,  (uint8_t)(crc >> 8),  (uint8_t)(crc >> 16),  (uint8_t)(crc >> 24) };
    su_cmac_init(c, key);
    su_cmac_update(c, h, sizeof(h));
}

void su_tag_end(const uint8_t key[16], const uint8_t nonce[16], const uint8_t image_tag[16], uint8_t out[16])
{
    su_cmac_t c;
    su_cmac_init(&c, key);
    su_cmac_update(&c, (const uint8_t *)"SUE1", 4);
    su_cmac_update(&c, nonce, 16);
    su_cmac_update(&c, image_tag, 16);
    su_cmac_final(&c, out);
}

int su_ct_equal(const uint8_t *a, const uint8_t *b, uint32_t n)
{
    uint8_t d = 0;
    for (uint32_t i = 0; i < n; i++) d |= (uint8_t)(a[i] ^ b[i]);
    return d == 0;
}

void su_wipe(void *p, uint32_t n)
{
    volatile uint8_t *v = (volatile uint8_t *)p;
    while (n--) *v++ = 0;
}
