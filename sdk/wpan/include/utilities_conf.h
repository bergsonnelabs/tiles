/**
 * utilities_conf.h -- Configuration for ST utilities (sequencer, AMM, timer)
 *
 * Provides the critical section macros and memory utilities needed
 * by stm32_seq.c, advanced_memory_manager.c, stm32_timer.c without
 * depending on the ST HAL.
 */

#ifndef UTILITIES_CONF_H
#define UTILITIES_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_conf.h"
#include <stdint.h>
#include <string.h>

/* ============================================================
 * Critical section macros (PRIMASK-based)
 * Used by sequencer, AMM, and timer server.
 * ============================================================ */

#define UTIL_SEQ_INIT_CRITICAL_SECTION()

#define UTIL_SEQ_ENTER_CRITICAL_SECTION() \
    uint32_t primask_bit; \
    __asm volatile ("MRS %0, primask" : "=r" (primask_bit)); \
    __asm volatile ("cpsid i" ::: "memory")

#define UTIL_SEQ_EXIT_CRITICAL_SECTION() \
    __asm volatile ("MSR primask, %0" :: "r" (primask_bit) : "memory")

/* UTILS_* macros -- used by advanced_memory_manager.c and stm32_timer.c */
#define UTILS_INIT_CRITICAL_SECTION()

#define UTILS_ENTER_CRITICAL_SECTION() \
    uint32_t primask_bit; \
    __asm volatile ("MRS %0, primask" : "=r" (primask_bit)); \
    __asm volatile ("cpsid i" ::: "memory")

#define UTILS_EXIT_CRITICAL_SECTION() \
    __asm volatile ("MSR primask, %0" :: "r" (primask_bit) : "memory")

/* ============================================================
 * Memory utilities
 * ============================================================ */

#define UTIL_SEQ_MEMSET8(dest, value, size)  memset((dest), (value), (size))

/* ============================================================
 * Placement / alignment macros (used by some ST utilities)
 * ============================================================ */

#define UTIL_PLACE_IN_SECTION(__x__)  __attribute__((section(__x__)))

#ifndef ALIGN
#define ALIGN(n)  __attribute__((aligned(n)))
#endif

/* ============================================================
 * AMM configuration (memory statistics disabled for simplicity)
 * ============================================================ */

#define AMM_USE_MEMORY_STATISTICS  0

/* ============================================================
 * Log/trace stubs
 * ============================================================ */

#define VLEVEL_OFF    0
#define VLEVEL_ALWAYS 0
#define VLEVEL_L 1
#define VLEVEL_M 2
#define VLEVEL_H 3

#define TS_OFF 0
#define TS_ON 1
#define T_REG_OFF  0

#ifdef __cplusplus
}
#endif

#endif /* UTILITIES_CONF_H */
