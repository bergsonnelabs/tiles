/* Host stand-in for sdk/ll/ll_flash.h (tests/host-nvm-sim): erase and program
 * go to the simulated flash in nvm_sim.c, which can cut the power mid-way. */
#ifndef LL_FLASH_H
#define LL_FLASH_H
#include "ll_common.h"
#define FLASH_START  0x08000000UL
#define FLASH_BASE   0x40022000UL
#define FLASH_ACR    REG32(FLASH_BASE)
static inline void ll_flash_unlock(void) { }
static inline void ll_flash_lock(void) { }
int ll_flash_erase_page(uint32_t page);
int ll_flash_program_dword(uint32_t addr, uint32_t w0, uint32_t w1);
int ll_flash_program_qword(uint32_t addr, const uint32_t w[4]);
#endif
