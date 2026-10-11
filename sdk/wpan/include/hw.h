/**
 * hw.h -- Minimal hardware abstraction shim
 *
 * The ST app_common.h includes "hw.h". We just provide
 * the HW_RNG_Get declaration that bleplat.c needs.
 */
#ifndef HW_H
#define HW_H

#include <stdint.h>

void HW_RNG_Get(uint8_t n, uint32_t *val);

#endif /* HW_H */
