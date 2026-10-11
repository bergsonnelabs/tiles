/**
 * baes.h -- BLE AES stub declarations
 */
#ifndef BAES_H
#define BAES_H

#include <stdint.h>

void BAES_Reset(void);
void BAES_EcbCrypt(const uint8_t *key, const uint8_t *input, uint8_t *output, int encrypt);
void BAES_CmacSetKey(const uint8_t *key);
void BAES_CmacCompute(const uint8_t *input, uint32_t input_length, uint8_t *output_tag);
int  BAES_CcmCrypt(uint8_t mode, const uint8_t *key, uint8_t iv_length, const uint8_t *iv,
                    uint16_t add_length, const uint8_t *add, uint32_t input_length,
                    const uint8_t *input, uint8_t tag_length, uint8_t *tag, uint8_t *output);

#endif /* BAES_H */
