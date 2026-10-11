/**
 * stm32_rtos.h -- Minimal shim for bare-metal (no RTOS)
 */

#ifndef STM32_RTOS_H
#define STM32_RTOS_H

#include "stm32_seq.h"

#define TASK_PRIO_RNG           CFG_SEQ_PRIO_0
#define TASK_PRIO_LINK_LAYER    CFG_SEQ_PRIO_0

#endif /* STM32_RTOS_H */
