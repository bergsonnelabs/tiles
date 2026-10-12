/**
 * thread_crypto_hal.c — the ST HAL calls the prebuilt mbedTLS library makes
 *
 * ST built stm32wba_mbedtls_ftd_lib.a with its hardware "alt" modules, which
 * call the STM32 HAL: aes_alt.o (HAL_CRYP_*, the AES peripheral) and
 * ecp_alt.o (HAL_PKA_*, the PKA). The SDK has no ST HAL, so this file supplies
 * those entry points itself:
 *
 *   HAL_CRYP_*  AES-ECB on the AES peripheral (0x420C0000), 128- and 256-bit
 *               keys, byte-swapped data: everything aes_alt.o asks for (it
 *               uses ECB only; mbedTLS builds CCM and CMAC on top of it).
 *               This is the path Thread's MAC and MLE security run on, so it
 *               is self-tested at init (FIPS-197 vectors, thread_app.c).
 *   HAL_HASH_*  SHA-224 / SHA-256 in software (sha256_alt.o). The library
 *               brackets every step with HAL_HASH_Resume / _Suspend, which
 *               save and restore the hardware context into a 424-byte buffer
 *               in each mbedtls_sha256_context; the software state (112 B)
 *               lives in that buffer instead, so interleaved contexts (HMAC)
 *               work the same. Software rather than the HASH peripheral: no
 *               new register path to bring up, and the HASH IRQ line stays
 *               the link layer's SW low interrupt (as in BLE builds).
 *               Self-tested at init (FIPS 180-2 "abc").
 *   HAL_PKA_*   not implemented: every call returns HAL_ERROR, so mbedTLS
 *               reports its ECC operations as failed. ECC is used by
 *               commissioning (ECJPAKE: joiner / commissioner), SRP client
 *               keys (ECDSA) and DTLS (CoAP secure), none of which milestone 1
 *               uses: it joins a fixed network from config.json.
 *
 * Only the fields aes_alt.o touches are declared, at the offsets of ST's
 * CRYP_HandleTypeDef (STM32CubeWBA V1.6.1 stm32wbaxx_hal_cryp.h), checked
 * below and against the library's disassembly: aes_set_key() stores
 * DataType at handle+4, KeySize +8, pKey +12, DataWidthUnit +36, KeyMode +48.
 * HASH handle: Init.DataType at +4, Init.Algorithm at +16 (mbedtls_sha256_starts
 * stores 0x20 / 0x60000 there).
 * The register sequences follow RM0493 §25.4.5 (ECB/CBC encryption and
 * decryption processes) and ST's HAL_CRYP code.
 *
 * The AES peripheral is shared with BLE (sdk/wpan/hw_aes.c: SMP, privacy, and
 * its ble_app_init() test vector) in BLE + Thread builds. Both run from the
 * sequencer, never from an interrupt, so they never interleave, but neither
 * may assume the other left the peripheral clean: hw_aes.c leaves ISR.CCF set
 * after every block (it clears the flags before its next block instead). So
 * every operation here starts from scratch (start_op: EN off, CCF / RWEIF /
 * KEIF cleared, CR rewritten whole) and ends with EN off and CCF cleared. With
 * a stale CCF, wait_ccf() returned at once and DOUTR was read mid-computation:
 * zeros, RDERRF set, the start-up self-test failed (THREAD_FATAL_CRYPTO, the
 * BLE + Thread boot loop, measured 2026-10-12: SR 0x81 / ISR 0x1 before it).
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "thread_app.h"

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;

/* ---- AES peripheral (RM0493 §25.7; CMSIS AES_TypeDef) ---- */

typedef struct {
    volatile uint32_t CR, SR, DINR, DOUTR;
    volatile uint32_t KEYR0, KEYR1, KEYR2, KEYR3;
    volatile uint32_t IVR0, IVR1, IVR2, IVR3;
    volatile uint32_t KEYR4, KEYR5, KEYR6, KEYR7;
    volatile uint32_t SUSPR[8];
    uint32_t reserved[168];
    volatile uint32_t IER, ISR, ICR;
} aes_regs_t;

_Static_assert(offsetof(aes_regs_t, KEYR4) == 0x30, "AES KEYR4 offset");
_Static_assert(offsetof(aes_regs_t, ISR) == 0x304, "AES ISR offset");

#define AES_BASE_NS       0x420C0000UL
#define AES_CR_EN         (1UL << 0)
#define AES_CR_DATATYPE   (3UL << 1)
#define AES_CR_MODE       (3UL << 3)
#define AES_CR_MODE_ENC   (0UL << 3)
#define AES_CR_MODE_KEYD  (1UL << 3)      /* key derivation (decryption key) */
#define AES_CR_MODE_DEC   (2UL << 3)
#define AES_CR_CHMOD      ((3UL << 5) | (1UL << 16))
#define AES_CR_KEYSIZE    (1UL << 18)
#define AES_CR_KMOD       (3UL << 24)
#define AES_SR_KEYVALID   (1UL << 7)
#define AES_ISR_CCF       (1UL << 0)
#define AES_ISR_RWEIF     (1UL << 1)
#define AES_ICR_CCF       (1UL << 0)
#define AES_ICR_RWEIF     (1UL << 1)      /* also clears SR.RDERRF / WRERRF */
#define AES_ICR_KEIF      (1UL << 2)

/* RCC is at 0x46020C00 (RM0493 §2.3, AHB4 + 0xC00); AHB2ENR at offset 0x08C.
 * This was 0x4602008C (no 0xC00), an unmapped address: a precise BusFault in
 * HAL_CRYP_Init during the start-up self-test, escalated to HardFault. */
#define RCC_AHB2ENR       (*(volatile uint32_t *)(0x46020C00UL + 0x08CUL))
#define RCC_AHB2ENR_AESEN (1UL << 16)

/* ---- The part of ST's CRYP handle that aes_alt.o uses ---- */

#define CRYP_KEYSIZE_128B  0x00000000UL
#define CRYP_KEYSIZE_256B  AES_CR_KEYSIZE
#define CRYP_AES_ECB       0x00000000UL

typedef struct {
    uint32_t  DataType;          /* CR.DATATYPE value (aes_alt: byte swap) */
    uint32_t  KeySize;           /* CRYP_KEYSIZE_128B / _256B */
    uint32_t *pKey;              /* key words, most significant first */
    uint32_t *pInitVect;
    uint32_t  Algorithm;         /* CR.CHMOD value; ECB = 0 */
    uint32_t *Header;
    uint32_t  HeaderSize;
    uint32_t *B0;
    uint32_t  DataWidthUnit;     /* 1 = sizes in bytes */
    uint32_t  HeaderWidthUnit;
    uint32_t  KeyIVConfigSkip;   /* 0 = load the key for every operation */
    uint32_t  KeyMode;           /* CR.KMOD value; normal = 0 */
    uint32_t  KeySelect;
    uint32_t  KeyProtection;
} cryp_config_t;

typedef struct {
    aes_regs_t   *Instance;
    cryp_config_t Init;
    /* ST's handle continues (buffers, counters, state, lock); nothing here
     * reads or writes past Init. */
} cryp_handle_t;

_Static_assert(offsetof(cryp_handle_t, Init.DataType) == 4, "CRYP Init.DataType");
_Static_assert(offsetof(cryp_handle_t, Init.KeySize) == 8, "CRYP Init.KeySize");
_Static_assert(offsetof(cryp_handle_t, Init.pKey) == 12, "CRYP Init.pKey");
_Static_assert(offsetof(cryp_handle_t, Init.Algorithm) == 20, "CRYP Init.Algorithm");
_Static_assert(offsetof(cryp_handle_t, Init.DataWidthUnit) == 36, "CRYP Init.DataWidthUnit");
_Static_assert(offsetof(cryp_handle_t, Init.KeyMode) == 48, "CRYP Init.KeyMode");

#define AES_TIMEOUT_LOOPS  100000u     /* one block takes ~50 AHB cycles */

static int wait_ccf(aes_regs_t *a)
{
    /* CCF from this block only: start_op() cleared any earlier one. */
    for (uint32_t t = AES_TIMEOUT_LOOPS; t; t--) {
        if (a->ISR & AES_ISR_CCF) {
            a->ICR = AES_ICR_CCF;
            return 0;
        }
    }
    return -1;
}

/* Write the key (EN clear) and wait for KEYVALID (RM0493 §25.4.5 step 4). */
static int load_key(aes_regs_t *a, const cryp_config_t *c)
{
    const uint32_t *k = c->pKey;
    if (c->KeySize == CRYP_KEYSIZE_256B) {
        a->KEYR7 = k[0]; a->KEYR6 = k[1]; a->KEYR5 = k[2]; a->KEYR4 = k[3];
        a->KEYR3 = k[4]; a->KEYR2 = k[5]; a->KEYR1 = k[6]; a->KEYR0 = k[7];
    } else {
        a->KEYR3 = k[0]; a->KEYR2 = k[1]; a->KEYR1 = k[2]; a->KEYR0 = k[3];
    }
    for (uint32_t t = AES_TIMEOUT_LOOPS; t; t--)
        if (a->SR & AES_SR_KEYVALID)
            return 0;
    return -1;
}

static int check(const cryp_handle_t *h)
{
    if (h == NULL || h->Instance == NULL || h->Init.pKey == NULL)
        return -1;
    if (h->Init.Algorithm != CRYP_AES_ECB)
        return -1;                       /* aes_alt only ever asks for ECB */
    if (h->Init.KeySize != CRYP_KEYSIZE_128B && h->Init.KeySize != CRYP_KEYSIZE_256B)
        return -1;
    return 0;
}

/* CR rewritten whole, EN and MODE clear: nothing another user (or an earlier
 * operation) set survives (GCMPH, NPBLB, DMA enables, MODE). */
static void configure(cryp_handle_t *h)
{
    aes_regs_t *a = h->Instance;
    a->CR = (h->Init.DataType & AES_CR_DATATYPE) | h->Init.KeySize |
            (h->Init.Algorithm & AES_CR_CHMOD) | (h->Init.KeyMode & AES_CR_KMOD);
}

/* Take the peripheral from whatever state its last user left: disable, clear
 * the flags (KEIF before reconfiguring, RM0493 §25.4.18), configure. */
static void start_op(cryp_handle_t *h)
{
    aes_regs_t *a = h->Instance;
    a->CR &= ~AES_CR_EN;
    a->ICR = AES_ICR_CCF | AES_ICR_RWEIF | AES_ICR_KEIF;
    configure(h);
}

HAL_StatusTypeDef HAL_CRYP_Init(cryp_handle_t *hcryp);
HAL_StatusTypeDef HAL_CRYP_DeInit(cryp_handle_t *hcryp);
HAL_StatusTypeDef HAL_CRYP_SetConfig(cryp_handle_t *hcryp, cryp_config_t *pConf);
HAL_StatusTypeDef HAL_CRYP_Encrypt(cryp_handle_t *hcryp, uint32_t *Input, uint16_t Size,
                                   uint32_t *Output, uint32_t Timeout);
HAL_StatusTypeDef HAL_CRYP_Decrypt(cryp_handle_t *hcryp, uint32_t *Input, uint16_t Size,
                                   uint32_t *Output, uint32_t Timeout);

HAL_StatusTypeDef HAL_CRYP_Init(cryp_handle_t *hcryp)
{
    if (check(hcryp) != 0)
        return HAL_ERROR;
    RCC_AHB2ENR |= RCC_AHB2ENR_AESEN;
    (void)RCC_AHB2ENR;
    start_op(hcryp);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CRYP_DeInit(cryp_handle_t *hcryp)
{
    if (hcryp == NULL)
        return HAL_ERROR;
    /* ST's DeInit only resets the handle state (and its MSP gates the clock,
     * which aes_alt turns straight back on): nothing to do on the hardware. */
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CRYP_SetConfig(cryp_handle_t *hcryp, cryp_config_t *pConf)
{
    if (hcryp == NULL || pConf == NULL)
        return HAL_ERROR;
    if (pConf != &hcryp->Init)
        hcryp->Init = *pConf;
    if (check(hcryp) != 0)
        return HAL_ERROR;
    start_op(hcryp);
    return HAL_OK;
}

/* Run Size bytes (whole 16-byte blocks) through the enabled peripheral. */
static int process(aes_regs_t *a, const uint32_t *in, uint32_t *out, uint32_t size)
{
    for (uint32_t off = 0; off < size; off += 16u) {
        uint32_t w[4];
        memcpy(w, (const uint8_t *)in + off, 16u);    /* the buffers may be unaligned */
        a->DINR = w[0];
        a->DINR = w[1];
        a->DINR = w[2];
        a->DINR = w[3];
        if (wait_ccf(a) != 0)
            return -1;
        w[0] = a->DOUTR;
        w[1] = a->DOUTR;
        w[2] = a->DOUTR;
        w[3] = a->DOUTR;
        if (a->ISR & AES_ISR_RWEIF) {           /* a DOUTR read returned zeros */
            a->ICR = AES_ICR_RWEIF;
            return -1;
        }
        memcpy((uint8_t *)out + off, w, 16u);
    }
    return 0;
}

static uint32_t size_bytes(const cryp_handle_t *h, uint16_t size)
{
    return (h->Init.DataWidthUnit == 1u) ? size : (uint32_t)size * 4u;
}

HAL_StatusTypeDef HAL_CRYP_Encrypt(cryp_handle_t *hcryp, uint32_t *Input, uint16_t Size,
                                   uint32_t *Output, uint32_t Timeout)
{
    (void)Timeout;
    if (check(hcryp) != 0 || Input == NULL || Output == NULL)
        return HAL_ERROR;
    uint32_t n = size_bytes(hcryp, Size);
    if (n == 0u || (n & 15u))
        return HAL_ERROR;

    aes_regs_t *a = hcryp->Instance;
    start_op(hcryp);
    a->CR = (a->CR & ~AES_CR_MODE) | AES_CR_MODE_ENC;
    if (load_key(a, &hcryp->Init) != 0)
        return HAL_TIMEOUT;
    a->CR |= AES_CR_EN;
    int rc = process(a, Input, Output, n);
    a->CR &= ~AES_CR_EN;
    return rc == 0 ? HAL_OK : HAL_TIMEOUT;
}

HAL_StatusTypeDef HAL_CRYP_Decrypt(cryp_handle_t *hcryp, uint32_t *Input, uint16_t Size,
                                   uint32_t *Output, uint32_t Timeout)
{
    (void)Timeout;
    if (check(hcryp) != 0 || Input == NULL || Output == NULL)
        return HAL_ERROR;
    uint32_t n = size_bytes(hcryp, Size);
    if (n == 0u || (n & 15u))
        return HAL_ERROR;

    aes_regs_t *a = hcryp->Instance;
    start_op(hcryp);
    /* ECB decryption: derive the decryption key first (MODE 01), then
     * decrypt (MODE 10) with the derived key left in KEYRx. */
    a->CR = (a->CR & ~(AES_CR_MODE | AES_CR_KMOD)) | AES_CR_MODE_KEYD;
    if (load_key(a, &hcryp->Init) != 0)
        return HAL_TIMEOUT;
    a->CR |= AES_CR_EN;
    if (wait_ccf(a) != 0) {
        a->CR &= ~AES_CR_EN;
        return HAL_TIMEOUT;
    }
    /* The derived key stays in the core and EN clears itself (§25.4.5 step 7);
     * clearing it here as well is harmless. Then decrypt mode, EN (steps 8, 10). */
    a->CR = (a->CR & ~(AES_CR_MODE | AES_CR_EN)) | AES_CR_MODE_DEC;
    a->CR |= AES_CR_EN;
    int rc = process(a, Input, Output, n);
    a->CR &= ~AES_CR_EN;
    return rc == 0 ? HAL_OK : HAL_TIMEOUT;
}

/* ST HAL callbacks the library may reference */
void HAL_CRYP_ErrorCallback(cryp_handle_t *hcryp);
void HAL_CRYP_ErrorCallback(cryp_handle_t *hcryp) { (void)hcryp; }

/* ============================================================
 * HASH: SHA-224 / SHA-256 in software (see the header comment)
 * ============================================================ */

#define HASH_ALGO_MASK    (3UL << 17)          /* HASH_CR_ALGO */
#define HASH_ALGO_SHA224  (2UL << 17)

typedef struct {
    void    *Instance;
    uint32_t DataType;
    uint32_t KeySize;
    uint8_t *pKey;
    uint32_t Algorithm;
    /* ST's handle continues; nothing here touches the rest. */
} hash_handle_t;

_Static_assert(offsetof(hash_handle_t, Algorithm) == 16, "HASH Init.Algorithm");

typedef struct {
    uint32_t h[8];
    uint64_t total;        /* bytes absorbed */
    uint32_t buflen;
    uint32_t is224;
    uint8_t  buf[64];
} sha256_state_t;

/* sha256_alt.h: ctx_save_regs[ST_SHA256_NB_HASH_REG * 4] = 424 bytes */
_Static_assert(sizeof(sha256_state_t) <= 424u, "SHA state must fit the library's context buffer");

static sha256_state_t s_sha;    /* the "hardware" state between Resume and Suspend */

static const uint32_t K256[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

#define ROR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))

static void sha256_block(uint32_t h[8], const uint8_t *p)
{
    uint32_t w[64];
    for (int i = 0; i < 16; i++)
        w[i] = ((uint32_t)p[4 * i] << 24) | ((uint32_t)p[4 * i + 1] << 16) |
               ((uint32_t)p[4 * i + 2] << 8) | p[4 * i + 3];
    for (int i = 16; i < 64; i++) {
        uint32_t s0 = ROR(w[i - 15], 7) ^ ROR(w[i - 15], 18) ^ (w[i - 15] >> 3);
        uint32_t s1 = ROR(w[i - 2], 17) ^ ROR(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; i++) {
        uint32_t t1 = hh + (ROR(e, 6) ^ ROR(e, 11) ^ ROR(e, 25)) + ((e & f) ^ (~e & g)) + K256[i] + w[i];
        uint32_t t2 = (ROR(a, 2) ^ ROR(a, 13) ^ ROR(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
        hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
}

static void sha256_reset(sha256_state_t *st, int is224)
{
    static const uint32_t iv256[8] = { 0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                       0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19 };
    static const uint32_t iv224[8] = { 0xc1059ed8, 0x367cd507, 0x3070dd17, 0xf70e5939,
                                       0xffc00b31, 0x68581511, 0x64f98fa7, 0xbefa4fa4 };
    memcpy(st->h, is224 ? iv224 : iv256, sizeof st->h);
    st->total = 0;
    st->buflen = 0;
    st->is224 = (uint32_t)is224;
}

static void sha256_absorb(sha256_state_t *st, const uint8_t *p, uint32_t n)
{
    st->total += n;
    while (n) {
        uint32_t take = 64u - st->buflen;
        if (take > n)
            take = n;
        memcpy(&st->buf[st->buflen], p, take);
        st->buflen += take;
        p += take;
        n -= take;
        if (st->buflen == 64u) {
            sha256_block(st->h, st->buf);
            st->buflen = 0;
        }
    }
}

static void sha256_finish(sha256_state_t *st, uint8_t *out)
{
    uint64_t bits = st->total * 8u;
    uint8_t pad = 0x80;
    sha256_absorb(st, &pad, 1);
    pad = 0;
    while (st->buflen != 56u)
        sha256_absorb(st, &pad, 1);
    uint8_t len[8];
    for (int i = 0; i < 8; i++)
        len[i] = (uint8_t)(bits >> (56 - 8 * i));
    sha256_absorb(st, len, 8);
    unsigned words = st->is224 ? 7u : 8u;
    for (unsigned i = 0; i < words; i++) {
        out[4 * i]     = (uint8_t)(st->h[i] >> 24);
        out[4 * i + 1] = (uint8_t)(st->h[i] >> 16);
        out[4 * i + 2] = (uint8_t)(st->h[i] >> 8);
        out[4 * i + 3] = (uint8_t)(st->h[i]);
    }
}

HAL_StatusTypeDef HAL_HASH_Init(hash_handle_t *hhash);
HAL_StatusTypeDef HAL_HASH_DeInit(hash_handle_t *hhash);
void HAL_HASH_Suspend(hash_handle_t *hhash, uint8_t *pMemBuffer);
void HAL_HASH_Resume(hash_handle_t *hhash, uint8_t *pMemBuffer);
HAL_StatusTypeDef HAL_HASH_Accumulate(hash_handle_t *hhash, const uint8_t *const pInBuffer,
                                      uint32_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_HASH_AccumulateLast(hash_handle_t *hhash, const uint8_t *const pInBuffer,
                                          uint32_t Size, uint8_t *const pOutBuffer, uint32_t Timeout);

HAL_StatusTypeDef HAL_HASH_Init(hash_handle_t *hhash)
{
    if (hhash == NULL)
        return HAL_ERROR;
    sha256_reset(&s_sha, (hhash->Algorithm & HASH_ALGO_MASK) == HASH_ALGO_SHA224);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_HASH_DeInit(hash_handle_t *hhash)
{
    return hhash == NULL ? HAL_ERROR : HAL_OK;
}

void HAL_HASH_Suspend(hash_handle_t *hhash, uint8_t *pMemBuffer)
{
    (void)hhash;
    if (pMemBuffer != NULL)
        memcpy(pMemBuffer, &s_sha, sizeof s_sha);
}

void HAL_HASH_Resume(hash_handle_t *hhash, uint8_t *pMemBuffer)
{
    (void)hhash;
    if (pMemBuffer != NULL)
        memcpy(&s_sha, pMemBuffer, sizeof s_sha);
}

HAL_StatusTypeDef HAL_HASH_Accumulate(hash_handle_t *hhash, const uint8_t *const pInBuffer,
                                      uint32_t Size, uint32_t Timeout)
{
    (void)Timeout;
    if (hhash == NULL || (pInBuffer == NULL && Size))
        return HAL_ERROR;
    sha256_absorb(&s_sha, pInBuffer, Size);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_HASH_AccumulateLast(hash_handle_t *hhash, const uint8_t *const pInBuffer,
                                          uint32_t Size, uint8_t *const pOutBuffer, uint32_t Timeout)
{
    (void)Timeout;
    if (hhash == NULL || pOutBuffer == NULL || (pInBuffer == NULL && Size))
        return HAL_ERROR;
    sha256_absorb(&s_sha, pInBuffer, Size);
    sha256_finish(&s_sha, pOutBuffer);
    return HAL_OK;
}

/* ============================================================
 * PKA: not supported (see the header comment)
 * ============================================================ */

#define PKA_STUB(name) \
    HAL_StatusTypeDef name(void *h, void *in, uint32_t t); \
    HAL_StatusTypeDef name(void *h, void *in, uint32_t t) { (void)h; (void)in; (void)t; return HAL_ERROR; }

HAL_StatusTypeDef HAL_PKA_Init(void *hpka);
HAL_StatusTypeDef HAL_PKA_Init(void *hpka) { (void)hpka; return HAL_ERROR; }
HAL_StatusTypeDef HAL_PKA_DeInit(void *hpka);
HAL_StatusTypeDef HAL_PKA_DeInit(void *hpka) { (void)hpka; return HAL_OK; }
HAL_StatusTypeDef HAL_PKA_RAMReset(void *hpka);
HAL_StatusTypeDef HAL_PKA_RAMReset(void *hpka) { (void)hpka; return HAL_ERROR; }
PKA_STUB(HAL_PKA_ECCMulEx)
PKA_STUB(HAL_PKA_MontgomeryParam)
PKA_STUB(HAL_PKA_PointCheck)

void HAL_PKA_ECCMul_GetResult(void *hpka, void *out);
void HAL_PKA_ECCMul_GetResult(void *hpka, void *out) { (void)hpka; (void)out; }
void HAL_PKA_MontgomeryParam_GetResult(void *hpka, uint32_t *pRes);
void HAL_PKA_MontgomeryParam_GetResult(void *hpka, uint32_t *pRes) { (void)hpka; (void)pRes; }
uint32_t HAL_PKA_PointCheck_IsOnCurve(void const *hpka);
uint32_t HAL_PKA_PointCheck_IsOnCurve(void const *hpka) { (void)hpka; return 0u; }

/* ============================================================
 * Self-tests: FIPS-197 Appendix C.1 (AES-128) and C.3 (AES-256), through
 * the same calls and data format aes_alt.o uses; FIPS 180-2 SHA-256.
 * ============================================================ */

static void be_words(uint32_t *w, const uint8_t *b, unsigned nwords)
{
    for (unsigned i = 0; i < nwords; i++)
        w[i] = ((uint32_t)b[4 * i] << 24) | ((uint32_t)b[4 * i + 1] << 16) |
               ((uint32_t)b[4 * i + 2] << 8) | b[4 * i + 3];
}

static int selftest_one(const uint8_t *key, unsigned keybits, const uint8_t *ct_expect)
{
    static const uint8_t pt[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                                    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff };
    uint32_t kw[8];
    uint8_t ct[16], back[16];
    cryp_handle_t h;

    memset(&h, 0, sizeof h);
    be_words(kw, key, keybits / 32u);
    h.Instance = (aes_regs_t *)AES_BASE_NS;
    h.Init.DataType = 2UL << 1;                       /* CRYP_BYTE_SWAP, as aes_alt */
    h.Init.KeySize = (keybits == 256u) ? CRYP_KEYSIZE_256B : CRYP_KEYSIZE_128B;
    h.Init.pKey = kw;
    h.Init.Algorithm = CRYP_AES_ECB;
    h.Init.DataWidthUnit = 1u;

    if (HAL_CRYP_Init(&h) != HAL_OK)
        return -1;
    if (HAL_CRYP_Encrypt(&h, (uint32_t *)(uintptr_t)pt, 16, (uint32_t *)ct, 0) != HAL_OK)
        return -1;
    if (memcmp(ct, ct_expect, 16) != 0)
        return -1;
    if (HAL_CRYP_Decrypt(&h, (uint32_t *)ct, 16, (uint32_t *)back, 0) != HAL_OK)
        return -1;
    return memcmp(back, pt, 16) == 0 ? 0 : -1;
}

/* FIPS 180-2 B.1: SHA-256("abc"), fed through the library's call pattern
 * (Init, Suspend, Resume, Accumulate, Suspend, Resume, AccumulateLast). */
static int selftest_sha256(void)
{
    static const uint8_t expect[32] = {
        0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
        0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad };
    hash_handle_t h;
    uint8_t ctx[424], out[32];
    memset(&h, 0, sizeof h);
    h.Algorithm = 3UL << 17;                         /* HASH_ALGOSELECTION_SHA256 */
    if (HAL_HASH_Init(&h) != HAL_OK)
        return -1;
    HAL_HASH_Suspend(&h, ctx);
    sha256_reset(&s_sha, 1);                         /* another context runs meanwhile */
    HAL_HASH_Resume(&h, ctx);
    (void)HAL_HASH_Accumulate(&h, (const uint8_t *)"ab", 2, 0);
    HAL_HASH_Suspend(&h, ctx);
    HAL_HASH_Resume(&h, ctx);
    if (HAL_HASH_AccumulateLast(&h, (const uint8_t *)"c", 1, out, 0) != HAL_OK)
        return -1;
    return memcmp(out, expect, sizeof expect) == 0 ? 0 : -1;
}

int thread_crypto_selftest(void)
{
    if (selftest_sha256() != 0)
        return -3;
    static const uint8_t k128[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                      0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
    static const uint8_t c128[16] = { 0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
                                      0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a };
    static const uint8_t k256[32] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                      0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
                                      0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                                      0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f };
    static const uint8_t c256[16] = { 0x8e, 0xa2, 0xb7, 0xca, 0x51, 0x67, 0x45, 0xbf,
                                      0xea, 0xfc, 0x49, 0x90, 0x4b, 0x49, 0x60, 0x89 };
    if (selftest_one(k128, 128u, c128) != 0)
        return -1;
    if (selftest_one(k256, 256u, c256) != 0)
        return -2;
    return 0;
}
