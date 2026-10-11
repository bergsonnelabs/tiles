/**
 * hw_aes.c — AES-ECB encryption using STM32WBA55 AES hardware
 *
 * The AES peripheral is at 0x420C0000 (AHB2 non-secure domain).
 * Register layout: CR(0x00), SR(0x04), DINR(0x08), DOUTR(0x0C),
 *                  KEYR0-3(0x10-0x1C).
 *
 * NIST test vector (FIPS 197 Appendix B):
 *   Key:        2b7e1516 28aed2a6 abf71588 09cf4f3c
 *   Plaintext:  6bc1bee2 2e409f96 e93d7e11 7393172a
 *   Ciphertext: 3ad77bb4 0d7a3660 a89ecaf3 2466ef97
 */

#include <stdint.h>
#include <string.h>
#include "baes.h"
#include "ll_common.h"
#include "ll_rcc.h"

/* AES peripheral registers */
#define AES_BASE  (PERIPH_BASE + 0x020C0000UL)

#define AES_CR    REG32(AES_BASE + 0x00UL)
#define AES_SR    REG32(AES_BASE + 0x04UL)
#define AES_DINR  REG32(AES_BASE + 0x08UL)
#define AES_DOUTR REG32(AES_BASE + 0x0CUL)
#define AES_KEYR0 REG32(AES_BASE + 0x10UL)
#define AES_KEYR1 REG32(AES_BASE + 0x14UL)
#define AES_KEYR2 REG32(AES_BASE + 0x18UL)
#define AES_KEYR3 REG32(AES_BASE + 0x1CUL)

#define AES_CR_EN       (1UL << 0)
#define AES_SR_CCF      (1UL << 0)
#define AES_SR_KEYVALID (1UL << 7)
#define LL_AHB2_AES     (1UL << 16)

/* Debug diagnostics */
volatile uint32_t aes_test_result __attribute__((used));
volatile uint32_t aes_test_output[4] __attribute__((used));
volatile uint32_t aes_test_method __attribute__((used));
volatile uint32_t aes_diag[8] __attribute__((used));

/* ---- Raw hardware encrypt (word-level) ---- */

static void hw_aes_encrypt(const uint32_t key[4], const uint32_t in[4], uint32_t out[4])
{
    ll_rcc_ahb2_clk_enable(LL_AHB2_AES);

    /* Disable and reset */
    AES_CR = 0;

    /* Clear stale CCF flag via ICR (offset 0x308) */
    REG32(AES_BASE + 0x308UL) = 0x07;  /* clear CCF + RWEIF + KEIF */

    /* Load key (while disabled) */
    AES_KEYR0 = key[0];
    AES_KEYR1 = key[1];
    AES_KEYR2 = key[2];
    AES_KEYR3 = key[3];

    aes_diag[0] = AES_SR;  /* SR after key load */
    aes_diag[1] = AES_CR;  /* CR before enable */

    /* Enable: ECB encrypt, DATATYPE=00 (32-bit, no swap) — matches ST hw_aes.c */
    AES_CR = AES_CR_EN;

    aes_diag[2] = AES_CR;  /* CR after enable */
    aes_diag[3] = AES_SR;  /* SR after enable */

    /* Load input */
    AES_DINR = in[0];
    AES_DINR = in[1];
    AES_DINR = in[2];
    AES_DINR = in[3];

    aes_diag[4] = AES_SR;  /* SR after input load */

    /* Wait for CCF (timeout protected) */
    uint32_t loops = 0;
    for (volatile uint32_t t = 100000; t; t--) {
        loops++;
        if (AES_SR & AES_SR_CCF) break;
    }
    aes_diag[5] = loops;   /* iterations waited */
    aes_diag[6] = AES_SR;  /* SR after wait */

    /* Read output */
    out[0] = AES_DOUTR;
    out[1] = AES_DOUTR;
    out[2] = AES_DOUTR;
    out[3] = AES_DOUTR;

    aes_diag[7] = AES_SR;  /* SR after read */

    /* Disable */
    AES_CR = 0;
}

/* ---- NIST test vector ---- */

/* (declared above) */

void baes_run_test(void)
{
    /* NIST FIPS 197 Appendix B test vector */
    static const uint8_t nist_key[16] = {
        0x2b,0x7e,0x15,0x16, 0x28,0xae,0xd2,0xa6,
        0xab,0xf7,0x15,0x88, 0x09,0xcf,0x4f,0x3c
    };
    static const uint8_t nist_pt[16] = {
        0x6b,0xc1,0xbe,0xe2, 0x2e,0x40,0x9f,0x96,
        0xe9,0x3d,0x7e,0x11, 0x73,0x93,0x17,0x2a
    };
    static const uint8_t nist_ct[16] = {
        0x3a,0xd7,0x7b,0xb4, 0x0d,0x7a,0x36,0x60,
        0xa8,0x9e,0xca,0xf3, 0x24,0x66,0xef,0x97
    };

    uint32_t k[4], pt[4], ct[4];
    uint8_t result[16];

    /* Try multiple byte orderings to find the correct one.
     * The BLE stack's BAES_SWAP reverses word order [0,1,2,3] → [3,2,1,0].
     * The ST HW_AES_REV loads key directly.
     * We test each combination. */

    /* Method 1: Direct (no swap) */
    memcpy(k, nist_key, 16);
    memcpy(pt, nist_pt, 16);
    hw_aes_encrypt(k, pt, ct);
    memcpy(result, ct, 16);
    if (memcmp(result, nist_ct, 16) == 0) {
        aes_test_method = 1;
        aes_test_result = 1;
        memcpy((void*)aes_test_output, ct, 16);
        return;
    }

    /* Method 2: Swap input/output word order (BAES_SWAP), key direct */
    memcpy(k, nist_key, 16);
    memcpy(pt, nist_pt, 16);
    { uint32_t t; t=pt[0]; pt[0]=pt[3]; pt[3]=t; t=pt[1]; pt[1]=pt[2]; pt[2]=t; }
    hw_aes_encrypt(k, pt, ct);
    { uint32_t t; t=ct[0]; ct[0]=ct[3]; ct[3]=t; t=ct[1]; ct[1]=ct[2]; ct[2]=t; }
    memcpy(result, ct, 16);
    if (memcmp(result, nist_ct, 16) == 0) {
        aes_test_method = 2;
        aes_test_result = 1;
        memcpy((void*)aes_test_output, ct, 16);
        return;
    }

    /* Method 3: Swap key AND input/output word order */
    memcpy(k, nist_key, 16);
    { uint32_t t; t=k[0]; k[0]=k[3]; k[3]=t; t=k[1]; k[1]=k[2]; k[2]=t; }
    memcpy(pt, nist_pt, 16);
    { uint32_t t; t=pt[0]; pt[0]=pt[3]; pt[3]=t; t=pt[1]; pt[1]=pt[2]; pt[2]=t; }
    hw_aes_encrypt(k, pt, ct);
    { uint32_t t; t=ct[0]; ct[0]=ct[3]; ct[3]=t; t=ct[1]; ct[1]=ct[2]; ct[2]=t; }
    memcpy(result, ct, 16);
    if (memcmp(result, nist_ct, 16) == 0) {
        aes_test_method = 3;
        aes_test_result = 1;
        memcpy((void*)aes_test_output, ct, 16);
        return;
    }

    /* Method 4: Byte-swap each word of input/output, key direct */
    memcpy(k, nist_key, 16);
    memcpy(pt, nist_pt, 16);
    for (int i=0; i<4; i++) pt[i] = __builtin_bswap32(pt[i]);
    hw_aes_encrypt(k, pt, ct);
    for (int i=0; i<4; i++) ct[i] = __builtin_bswap32(ct[i]);
    memcpy(result, ct, 16);
    if (memcmp(result, nist_ct, 16) == 0) {
        aes_test_method = 4;
        aes_test_result = 1;
        memcpy((void*)aes_test_output, ct, 16);
        return;
    }

    /* Method 5: Byte-swap each word of key AND input/output */
    memcpy(k, nist_key, 16);
    for (int i=0; i<4; i++) k[i] = __builtin_bswap32(k[i]);
    memcpy(pt, nist_pt, 16);
    for (int i=0; i<4; i++) pt[i] = __builtin_bswap32(pt[i]);
    hw_aes_encrypt(k, pt, ct);
    for (int i=0; i<4; i++) ct[i] = __builtin_bswap32(ct[i]);
    memcpy(result, ct, 16);
    if (memcmp(result, nist_ct, 16) == 0) {
        aes_test_method = 5;
        aes_test_result = 1;
        memcpy((void*)aes_test_output, ct, 16);
        return;
    }

    /* Method 6: BAES_SWAP + byte-swap each word (both transformations) */
    memcpy(k, nist_key, 16);
    memcpy(pt, nist_pt, 16);
    { uint32_t t; t=pt[0]; pt[0]=pt[3]; pt[3]=t; t=pt[1]; pt[1]=pt[2]; pt[2]=t; }
    for (int i=0; i<4; i++) pt[i] = __builtin_bswap32(pt[i]);
    hw_aes_encrypt(k, pt, ct);
    for (int i=0; i<4; i++) ct[i] = __builtin_bswap32(ct[i]);
    { uint32_t t; t=ct[0]; ct[0]=ct[3]; ct[3]=t; t=ct[1]; ct[1]=ct[2]; ct[2]=t; }
    memcpy(result, ct, 16);
    if (memcmp(result, nist_ct, 16) == 0) {
        aes_test_method = 6;
        aes_test_result = 1;
        memcpy((void*)aes_test_output, ct, 16);
        return;
    }

    /* Method 7: bswap each word + reverse order for KEYR3=MSB, DINR MSB first
     * KEYR3=bswap(k[0]), KEYR2=bswap(k[1]), KEYR1=bswap(k[2]), KEYR0=bswap(k[3])
     * Same for DINR. This puts big-endian key/data into the registers correctly. */
    memcpy(k, nist_key, 16);
    memcpy(pt, nist_pt, 16);
    {
        /* Reverse word order AND bswap each word = full byte reversal of 16 bytes */
        uint32_t rk[4] = { __builtin_bswap32(k[3]), __builtin_bswap32(k[2]),
                           __builtin_bswap32(k[1]), __builtin_bswap32(k[0]) };
        uint32_t rpt[4] = { __builtin_bswap32(pt[3]), __builtin_bswap32(pt[2]),
                            __builtin_bswap32(pt[1]), __builtin_bswap32(pt[0]) };
        hw_aes_encrypt(rk, rpt, ct);
        /* Reverse output the same way */
        uint32_t rct[4] = { __builtin_bswap32(ct[3]), __builtin_bswap32(ct[2]),
                            __builtin_bswap32(ct[1]), __builtin_bswap32(ct[0]) };
        memcpy(result, rct, 16);
    }
    if (memcmp(result, nist_ct, 16) == 0) {
        aes_test_method = 7;
        aes_test_result = 1;
        memcpy((void*)aes_test_output, result, 16);
        return;
    }

    /* Method 8: Key = bswap + reverse (KEYR3=MSB), Data = bswap only (DINR MSB first)
     * Key: KEYR0=bswap(k[3])..KEYR3=bswap(k[0])  (reverse order, each word bswap'd)
     * Data: DINR[0]=bswap(pt[0])..DINR[3]=bswap(pt[3])  (natural order, each word bswap'd)
     * Output: result = bswap(DOUTR[0..3])  (natural order, each word bswap'd) */
    memcpy(k, nist_key, 16);
    memcpy(pt, nist_pt, 16);
    {
        uint32_t rk[4] = { __builtin_bswap32(k[3]), __builtin_bswap32(k[2]),
                           __builtin_bswap32(k[1]), __builtin_bswap32(k[0]) };
        uint32_t bpt[4] = { __builtin_bswap32(pt[0]), __builtin_bswap32(pt[1]),
                            __builtin_bswap32(pt[2]), __builtin_bswap32(pt[3]) };
        hw_aes_encrypt(rk, bpt, ct);
        uint32_t bct[4] = { __builtin_bswap32(ct[0]), __builtin_bswap32(ct[1]),
                            __builtin_bswap32(ct[2]), __builtin_bswap32(ct[3]) };
        memcpy(result, bct, 16);
    }
    if (memcmp(result, nist_ct, 16) == 0) {
        aes_test_method = 8;
        aes_test_result = 1;
        memcpy((void*)aes_test_output, result, 16);
        return;
    }

    /* None matched */
    aes_test_result = 2;
    memcpy((void*)aes_test_output, ct, 16);
}

/* ---- BAES API (used by BLE stack) ---- */

void BAES_Reset(void)
{
    ll_rcc_ahb2_clk_enable(LL_AHB2_AES);
    AES_CR = 0;
}

void BAES_EcbCrypt(const uint8_t *key, const uint8_t *input,
                   uint8_t *output, int encrypt)
{
    (void)encrypt;

    /* Match ST's baes_ecb.c exactly:
     * 1. Key loaded with HW_AES_REV: KEYR0=k[0]..KEYR3=k[3] (direct memcpy order)
     * 2. Input BAES_SWAP'd: word order reversed [0,1,2,3] → [3,2,1,0]
     * 3. DATATYPE=00 (32-bit, no byte swap)
     * 4. Output BAES_SWAP'd back: [3,2,1,0] → [0,1,2,3]
     *
     * This is NOT the same as standard big-endian AES (which is method 8).
     * The BLE stack's SMP code expects this specific byte transformation. */
    uint32_t k[4], in[4], ct[4];

    memcpy(k, key, 16);
    memcpy(in, input, 16);

    /* BAES_SWAP input: reverse word order */
    { uint32_t t; t=in[0]; in[0]=in[3]; in[3]=t; t=in[1]; in[1]=in[2]; in[2]=t; }

    /* Key goes directly (HW_AES_REV mode) */
    hw_aes_encrypt(k, in, ct);

    /* BAES_SWAP output: reverse word order */
    { uint32_t t; t=ct[0]; ct[0]=ct[3]; ct[3]=t; t=ct[1]; ct[1]=ct[2]; ct[2]=t; }

    memcpy(output, ct, 16);
}

/* ---- CMAC / CCM stubs ---- */

void BAES_CmacSetKey(const uint8_t *key) { (void)key; }

void BAES_CmacCompute(const uint8_t *input, uint32_t input_length,
                      uint8_t *output_tag)
{
    (void)input; (void)input_length;
    memset(output_tag, 0, 16);
}

int BAES_CcmCrypt(uint8_t mode, const uint8_t *key, uint8_t iv_length,
                  const uint8_t *iv, uint16_t add_length, const uint8_t *add,
                  uint32_t input_length, const uint8_t *input,
                  uint8_t tag_length, uint8_t *tag, uint8_t *output)
{
    (void)mode; (void)key; (void)iv_length; (void)iv;
    (void)add_length; (void)add;
    (void)input_length; (void)input;
    (void)tag_length; (void)tag; (void)output;
    return 0;
}
