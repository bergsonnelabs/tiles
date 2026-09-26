/* Host stand-in for sdk/ll/ll_flash_edata.h (tests/host-nvm-sim): the H5
 * data area's 16-bit words live in nvm_sim.c, which can cut the power. */
#ifndef LL_FLASH_EDATA_H
#define LL_FLASH_EDATA_H
#include "ll_flash.h"
typedef struct { uint8_t unused; } ll_flash_edata_ctx_t;
static inline void ll_flash_edata_begin(ll_flash_edata_ctx_t *c) { (void)c; }
static inline void ll_flash_edata_end(const ll_flash_edata_ctx_t *c) { (void)c; }
int ll_flash_edata_read_half(uint32_t addr, uint16_t *v);
int ll_flash_edata_program_half(uint32_t addr, uint16_t v);
#endif
