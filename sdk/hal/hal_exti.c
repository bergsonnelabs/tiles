/**
 * hal_exti.c — EXTI HAL driver implementation
 *
 * Static callback registry with per-family ISR handlers.
 * Handles grouped IRQs on L0 (lines 0-1, 2-3, 4-15) and
 * L4 (lines 5-9, 10-15), plus individual IRQs on WBA/H5.
 */

#include "hal_exti.h"

/* ============================================================
 * Callback registry — one slot per EXTI line (pin 0-15)
 * ============================================================ */

static struct {
    hal_callback_t cb;
    void          *ctx;
} _exti_slots[16];

/* ============================================================
 * IRQ number resolution
 * ============================================================ */

static uint32_t _exti_irqn(uint32_t line)
{
#if defined(STM32L011xx)
    /* L0: grouped — lines 0-1, 2-3, 4-15 */
    if (line <= 1)  return EXTI0_1_IRQn;
    if (line <= 3)  return EXTI2_3_IRQn;
    return EXTI4_15_IRQn;

#elif defined(STM32L422xx)
    /* L4: individual 0-4, grouped 5-9, grouped 10-15 */
    if (line == 0)  return EXTI0_IRQn;
    if (line == 1)  return EXTI1_IRQn;
    if (line == 2)  return EXTI2_IRQn;
    if (line == 3)  return EXTI3_IRQn;
    if (line == 4)  return EXTI4_IRQn;
    if (line <= 9)  return EXTI9_5_IRQn;
    return EXTI15_10_IRQn;

#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    /* WBA/H5: individual lines 0-15 */
    return EXTI0_IRQn + line;
#endif
}

/* ============================================================
 * Grouped IRQ line ranges (for safe NVIC disable)
 * ============================================================ */

/**
 * Get the first and last EXTI line that share an IRQ with `line`.
 */
static void _exti_irq_range(uint32_t line, uint32_t *first, uint32_t *last)
{
#if defined(STM32L011xx)
    if (line <= 1)       { *first = 0;  *last = 1;  }
    else if (line <= 3)  { *first = 2;  *last = 3;  }
    else                 { *first = 4;  *last = 15; }

#elif defined(STM32L422xx)
    if (line <= 4)       { *first = line; *last = line; }
    else if (line <= 9)  { *first = 5;    *last = 9;   }
    else                 { *first = 10;   *last = 15;  }

#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    *first = line;
    *last  = line;
#endif
}

/**
 * Check if any lines in the given range have active callbacks.
 */
static int _exti_group_active(uint32_t first, uint32_t last)
{
    for (uint32_t i = first; i <= last; i++) {
        if (_exti_slots[i].cb) return 1;
    }
    return 0;
}

/* ============================================================
 * Public API
 * ============================================================ */

hal_status_t hal_exti_enable(uint8_t pad, uint32_t edge,
                              hal_callback_t cb, void *ctx)
{
    hal_pad_gpio_t g = hal_pad_lookup(pad);
    if (!g.port) return HAL_ERROR;

    uint32_t line = g.pin;
    if (line > 15) return HAL_ERROR;

    /* Store callback */
    _exti_slots[line].cb  = cb;
    _exti_slots[line].ctx = ctx;

    /* Configure EXTI: SYSCFG mux + edge detection + interrupt mask */
    ll_exti_gpio_config(g.port, g.pin, edge);

    /* Enable NVIC */
    uint32_t irqn = _exti_irqn(line);
    hal_nvic_set_priority(irqn, 4);   /* raw NVIC 0x40 (the helper shifts <<4) */
    hal_nvic_enable_irq(irqn);

    return HAL_OK;
}

void hal_exti_disable(uint8_t pad)
{
    hal_pad_gpio_t g = hal_pad_lookup(pad);
    if (!g.port) return;

    uint32_t line = g.pin;
    if (line > 15) return;

    /* Clear callback */
    _exti_slots[line].cb  = (hal_callback_t)0;
    _exti_slots[line].ctx = (void *)0;

    /* Disable EXTI line */
    ll_exti_disable(line);

    /* Only disable NVIC if no other lines share this IRQ */
    uint32_t first, last;
    _exti_irq_range(line, &first, &last);
    if (!_exti_group_active(first, last)) {
        hal_nvic_disable_irq(_exti_irqn(line));
    }
}

/* ============================================================
 * ISR dispatch
 * ============================================================ */

static void _exti_dispatch(uint32_t first, uint32_t last)
{
    for (uint32_t line = first; line <= last; line++) {
        if (ll_exti_pending(line)) {
            ll_exti_clear_pending(line);
            if (_exti_slots[line].cb) {
                _exti_slots[line].cb(_exti_slots[line].ctx);
            }
        }
    }
}

/* ---- ISR handlers per family ---- */

#if defined(STM32L011xx)

void EXTI0_1_IRQHandler(void)   { _exti_dispatch(0, 1);  }
void EXTI2_3_IRQHandler(void)   { _exti_dispatch(2, 3);  }
void EXTI4_15_IRQHandler(void)  { _exti_dispatch(4, 15); }

#elif defined(STM32L422xx)

void EXTI0_IRQHandler(void)     { _exti_dispatch(0, 0);  }
void EXTI1_IRQHandler(void)     { _exti_dispatch(1, 1);  }
void EXTI2_IRQHandler(void)     { _exti_dispatch(2, 2);  }
void EXTI3_IRQHandler(void)     { _exti_dispatch(3, 3);  }
void EXTI4_IRQHandler(void)     { _exti_dispatch(4, 4);  }
void EXTI9_5_IRQHandler(void)   { _exti_dispatch(5, 9);  }
void EXTI15_10_IRQHandler(void) { _exti_dispatch(10, 15); }

#elif defined(STM32WBA55xx)

void EXTI0_IRQHandler(void)     { _exti_dispatch(0, 0);  }
void EXTI1_IRQHandler(void)     { _exti_dispatch(1, 1);  }
void EXTI2_IRQHandler(void)     { _exti_dispatch(2, 2);  }
void EXTI3_IRQHandler(void)     { _exti_dispatch(3, 3);  }
void EXTI4_IRQHandler(void)     { _exti_dispatch(4, 4);  }
void EXTI5_IRQHandler(void)     { _exti_dispatch(5, 5);  }
void EXTI6_IRQHandler(void)     { _exti_dispatch(6, 6);  }
void EXTI7_IRQHandler(void)     { _exti_dispatch(7, 7);  }
void EXTI8_IRQHandler(void)     { _exti_dispatch(8, 8);  }
void EXTI9_IRQHandler(void)     { _exti_dispatch(9, 9);  }
void EXTI10_IRQHandler(void)    { _exti_dispatch(10, 10); }
void EXTI11_IRQHandler(void)    { _exti_dispatch(11, 11); }
void EXTI12_IRQHandler(void)    { _exti_dispatch(12, 12); }
void EXTI13_IRQHandler(void)    { _exti_dispatch(13, 13); }
void EXTI14_IRQHandler(void)    { _exti_dispatch(14, 14); }
void EXTI15_IRQHandler(void)    { _exti_dispatch(15, 15); }

#elif defined(STM32H523xx)

void EXTI0_IRQHandler(void)     { _exti_dispatch(0, 0);  }
void EXTI1_IRQHandler(void)     { _exti_dispatch(1, 1);  }
void EXTI2_IRQHandler(void)     { _exti_dispatch(2, 2);  }
void EXTI3_IRQHandler(void)     { _exti_dispatch(3, 3);  }
void EXTI4_IRQHandler(void)     { _exti_dispatch(4, 4);  }
void EXTI5_IRQHandler(void)     { _exti_dispatch(5, 5);  }
void EXTI6_IRQHandler(void)     { _exti_dispatch(6, 6);  }
void EXTI7_IRQHandler(void)     { _exti_dispatch(7, 7);  }
void EXTI8_IRQHandler(void)     { _exti_dispatch(8, 8);  }
void EXTI9_IRQHandler(void)     { _exti_dispatch(9, 9);  }
void EXTI10_IRQHandler(void)    { _exti_dispatch(10, 10); }
void EXTI11_IRQHandler(void)    { _exti_dispatch(11, 11); }
void EXTI12_IRQHandler(void)    { _exti_dispatch(12, 12); }
void EXTI13_IRQHandler(void)    { _exti_dispatch(13, 13); }
void EXTI14_IRQHandler(void)    { _exti_dispatch(14, 14); }
void EXTI15_IRQHandler(void)    { _exti_dispatch(15, 15); }

#endif
