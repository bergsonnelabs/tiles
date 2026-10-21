/**
 * wpan_irq.c — 2.4 GHz radio interrupt handlers for STM32WBA55 (BLE and Thread)
 *
 * Formerly sdk/ble/ble_irq.c (unchanged code; it keeps ble_irq's place in the
 * BLE link order, see the Makefile).
 *
 * RADIO_IRQHandler (IRQ 66) — high-priority radio ISR
 * HASH_IRQHandler  (IRQ 61) — repurposed as software low-priority radio ISR
 *
 * MUST match the working project's stm32wbaxx_it.c exactly.
 */

#include <stdint.h>
#include <stddef.h>
#include "hal_common.h"
#include "app_conf.h"

/* Callbacks installed by the link layer — defined here, extern'd elsewhere */
void (*radio_callback)(void) = NULL;
void (*low_isr_callback)(void) = NULL;
volatile uint8_t radio_sw_low_isr_is_running_high_prio = 0;

/* RCC_RADIOENR register — STRADIOCLKON is bit 16 */
#define _RCC_RADIOENR_ADDR  (*(volatile uint32_t *)(0x46020C00UL + 0x208UL))
#define _STRADIOCLKON       (1UL << 16)

void RADIO_IRQHandler(void)
{
    if (radio_callback != NULL)
    {
        radio_callback();
    }

    /* Disable sleep timer auto-clock — CRITICAL for proper advertising timing.
     * Without this, the auto-wakeup mechanism interferes with the radio scheduler.
     * This matches the working project's RADIO_IRQHandler exactly. */
    _RCC_RADIOENR_ADDR &= ~_STRADIOCLKON;

    __asm volatile ("isb 0xF" ::: "memory");
}

extern void LINKLAYER_PLAT_LowIsrEnter(void);    /* linklayer_plat.c */
extern void LINKLAYER_PLAT_LowIsrExit(void);

void HASH_IRQHandler(void)
{
    /* Disable SW radio low interrupt to prevent nested calls */
    hal_nvic_disable_irq(RADIO_SW_LOW_INTR_NUM);

    /* Guard against the link layer's RCO-compensation assert: a live bus,
     * past the previous exit tick and the library's reference (linklayer_plat.c). */
    LINKLAYER_PLAT_LowIsrEnter();

    if (low_isr_callback != NULL)
    {
        low_isr_callback();
    }

    /* Check if nested SW radio low interrupt has been requested */
    if (radio_sw_low_isr_is_running_high_prio != 0)
    {
        hal_nvic_set_priority(RADIO_SW_LOW_INTR_NUM, RADIO_INTR_PRIO_LOW);
        radio_sw_low_isr_is_running_high_prio = 0;
    }

    LINKLAYER_PLAT_LowIsrExit();

    /* Re-enable SW radio low interrupt */
    hal_nvic_enable_irq(RADIO_SW_LOW_INTR_NUM);
}
