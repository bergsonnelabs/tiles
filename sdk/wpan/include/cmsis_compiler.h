/**
 * cmsis_compiler.h — Minimal shim for Cores SDK
 *
 * Provides the CMSIS intrinsics that stm_list.h, stm32_wpan_common.h,
 * and stm32_timer.h expect, without pulling in full CMSIS headers.
 */

#ifndef CMSIS_COMPILER_H
#define CMSIS_COMPILER_H

#include <stdint.h>

#ifndef __STATIC_INLINE
#define __STATIC_INLINE  static inline
#endif

#ifndef __WEAK
#define __WEAK  __attribute__((weak))
#endif

#ifndef __PACKED_STRUCT
#define __PACKED_STRUCT  struct __attribute__((packed))
#endif

#ifndef __PACKED_UNION
#define __PACKED_UNION  union __attribute__((packed))
#endif

static inline void __disable_irq(void)
{
    __asm volatile ("cpsid i" ::: "memory");
}

static inline void __enable_irq(void)
{
    __asm volatile ("cpsie i" ::: "memory");
}

static inline uint32_t __get_PRIMASK(void)
{
    uint32_t result;
    __asm volatile ("MRS %0, primask" : "=r" (result));
    return result;
}

static inline void __set_PRIMASK(uint32_t val)
{
    __asm volatile ("MSR primask, %0" :: "r" (val) : "memory");
}

#endif /* CMSIS_COMPILER_H */
