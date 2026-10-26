/**
 * port_core_led_vdd.h — STM32 port: the supply read behind core_led.c
 *
 * Included by core_led.c only (after its CORE_LED_* constants); not for
 * direct use. port_core_led_vdd_mv() returns VDD in mV through VREFINT, which
 * core_led.c turns into the on-time stretch for short LED blips. A Core with
 * no supply read returns CORE_LED_REF_MV (a scale of 1).
 */

#ifndef PORT_CORE_LED_VDD_H
#define PORT_CORE_LED_VDD_H

#include "hal_adc.h"

/* The project's ADC handle, when it declares ADC pads (coregen). Weak, so a
 * project without one links, and we can tell. */
extern hal_adc_t core_adc1 __attribute__((weak));

/* VDD in mV through VREFINT: L0 and L4 (ADC1), W5 (ADC4). The H5 is not
 * wired in yet and returns a nominal 3300 mV, which gives a scale of 1 (no
 * change). Its VREFINT read itself works (adc.mv verified 2026-09-22); what
 * is missing is this path on the H5 and a bench check of it. */
#if defined(STM32WBA55xx)
#define LED_VDD_ADC  ADC4
#else
#define LED_VDD_ADC  ADC1
#endif
static uint32_t port_core_led_vdd_mv(void)
{
#if defined(STM32L011xx) || defined(STM32L422xx) || defined(STM32WBA55xx)
    if (&core_adc1 != 0 && core_adc1.instance != 0) {
        /* The project owns ADC1: never re-sequence it under running DMA. */
        if (core_adc1.dma_active) {
            return core_adc1.vdda_mv ? core_adc1.vdda_mv : CORE_LED_REF_MV;
        }
        return hal_adc_read_vdda_mv(&core_adc1);
    }
    /* No project ADC: bring one up, read VREFINT, and power it back down. */
#if defined(STM32L011xx)
    /* Put VREFINT's ADC buffers and the temperature sensor back the way they
     * were: left on they draw ~17 uA typ (DS DocID027973 Table 21 I_BUF_ADC
     * 13.5 uA, Table 58 I_DDA(TEMP) 3.4 uA), in Stop too. Bench, 2026-09-26:
     * one heartbeat left SYSCFG_CFGR3 = 0x40000300, ADC_CCR VREFEN | TSEN. */
    const uint32_t bufs = SYSCFG_CFGR3_ENBUF_VREFINT | SYSCFG_CFGR3_ENBUF_SENSOR;
    const uint32_t ens = (1UL << 22) | (1UL << 23);           /* VREFEN, TSEN */
    uint32_t apb2 = REG32(RCC_BASE + 0x34UL);                 /* APB2ENR */
    uint32_t bufs_was = (apb2 & 1UL) ? (SYSCFG_CFGR3 & bufs) : 0u;
    uint32_t ens_was = (apb2 & (1UL << 9)) ? (ADC_CCR & ens) : 0u;
#endif
    hal_adc_t adc;
    uint32_t mv = CORE_LED_REF_MV;
    if (hal_adc_init(&adc, LED_VDD_ADC, SYSCLK_HZ, HAL_ADC_RES_12BIT) == HAL_OK) {
        mv = hal_adc_read_vdda_mv(&adc);
        hal_adc_deinit(&adc);
    }
#if defined(STM32L011xx)
    MOD_BITS(ADC_CCR, ens, ens_was);                          /* ADC is off: writable */
    MOD_BITS(SYSCFG_CFGR3, bufs, bufs_was);
    CLR_BITS(REG32(RCC_BASE + 0x34UL), ~apb2 & ((1UL << 9) | 1UL));  /* ADCEN, SYSCFGEN we set */
#endif
    return mv;
#else
    return CORE_LED_REF_MV;
#endif
}

#endif /* PORT_CORE_LED_VDD_H */
