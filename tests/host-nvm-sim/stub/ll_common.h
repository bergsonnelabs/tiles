/* Host stand-in for sdk/ll/ll_common.h (tests/host-nvm-sim): registers read 0
 * and swallow writes, so core_nvm.c's cache / ECC housekeeping is a no-op. */
#ifndef LL_COMMON_H
#define LL_COMMON_H
#include <stdint.h>
static inline volatile uint32_t *sim_reg(void) { static uint32_t w; w = 0; return &w; }
#define REG32(addr)              (*((void)(addr), sim_reg()))
#define SET_BITS(reg, mask)      ((reg) |= (mask))
#define CLR_BITS(reg, mask)      ((reg) &= ~(mask))
#define MOD_BITS(reg, mask, val) ((reg) = ((reg) & ~(mask)) | (val))
#endif
