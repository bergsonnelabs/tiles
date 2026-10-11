/* Host stand-in for sdk/ll/ll_common.h (tests/host-timer-sim): LPTIM2's
 * registers are a word array timer_sim.c models; anything else is a scratch
 * word that reads 0 and swallows writes. */
#ifndef LL_COMMON_H
#define LL_COMMON_H
#include <stdint.h>
extern uint32_t sim_lptim2[8];
extern uint32_t sim_scratch;
static inline volatile uint32_t *sim_reg(uintptr_t a)
{
    if (a >= 0x40009400UL && a < 0x40009420UL)
        return &sim_lptim2[(a - 0x40009400UL) / 4u];
    sim_scratch = 0;
    return &sim_scratch;
}
#define REG32(addr)              (*sim_reg((uintptr_t)(addr)))
#define SET_BITS(reg, mask)      ((reg) |= (mask))
#define CLR_BITS(reg, mask)      ((reg) &= ~(mask))
#endif
