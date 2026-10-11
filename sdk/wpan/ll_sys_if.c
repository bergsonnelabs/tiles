/**
 * ll_sys_if.c — Link Layer system interface
 *
 * Adapted from ST's reference implementation for use without the ST HAL.
 * Uses our ll_rcc.h register-level drivers instead.
 *
 * Original: STM32_WPAN/Target/ll_sys_if.c
 * Copyright (c) 2025 STMicroelectronics. Licensed under ST terms.
 */

#include <stdint.h>
#include "app_conf.h"
#include "ll_sys.h"
#include "ll_sys_if.h"
#include "ll_intf_cmn.h"
#include "ll_rcc.h"
#include "stm32_seq.h"

/* Private defines */
#define USE_RADIO_LOW_ISR              (1)
#define NEXT_EVENT_SCHEDULING_FROM_ISR (1)

/* Sleep clock source values expected by the link layer */
#define RTC_SLPTMR                     0
#define RCO_SLPTMR                     1
#define CRYSTAL_OSCILLATOR_SLPTMR      2

/* SCA ranges (default 500ppm) */
#define STM32WBA5x_DEFAULT_SCA_RANGE   0

/* Private function prototypes */
static void ll_sys_sleep_clock_source_selection(void);
static uint8_t ll_sys_BLE_sleep_clock_accuracy_selection(void);

/* ============================================================
 * Link Layer background process management
 * ============================================================ */

void ll_sys_bg_process_init(void)
{
    UTIL_SEQ_RegTask(1U << CFG_TASK_LINK_LAYER, UTIL_SEQ_RFU, ll_sys_bg_process);
}

void ll_sys_schedule_bg_process(void)
{
    UTIL_SEQ_SetTask(1U << CFG_TASK_LINK_LAYER, CFG_SEQ_PRIO_0);
}

void ll_sys_schedule_bg_process_isr(void)
{
    UTIL_SEQ_SetTask(1U << CFG_TASK_LINK_LAYER, CFG_SEQ_PRIO_0);
}

/* ============================================================
 * Link Layer configuration
 * ============================================================ */

void ll_sys_config_params(void)
{
    /* Configure link layer behavior:
     * - SW low ISR is used.
     * - Next event is scheduled from ISR.
     */
    ll_intf_cmn_config_ll_ctx_params(USE_RADIO_LOW_ISR, NEXT_EVENT_SCHEDULING_FROM_ISR);

    /* Apply the selected link layer sleep timer source */
    ll_sys_sleep_clock_source_selection();

    /* Link Layer power table */
    ll_intf_cmn_select_tx_power_table(CFG_RF_TX_POWER_TABLE_ID);
}

/* ============================================================
 * Sleep clock accuracy
 * ============================================================ */

static uint8_t ll_sys_BLE_sleep_clock_accuracy_selection(void)
{
    uint32_t linklayer_slp_clk_src = ll_rcc_get_radio_sleep_clk();

    if (linklayer_slp_clk_src == LL_RCC_RADIOSLEEPSOURCE_LSE)
    {
        /* LSE: use default 500ppm for our bring-up */
        return STM32WBA5x_DEFAULT_SCA_RANGE;
    }
    else
    {
        /* Not LSE — default 500ppm */
        return STM32WBA5x_DEFAULT_SCA_RANGE;
    }
}

/* ============================================================
 * Sleep clock source selection
 * ============================================================ */

static void ll_sys_sleep_clock_source_selection(void)
{
    uint16_t freq_value = 0;
    uint32_t linklayer_slp_clk_src;

    linklayer_slp_clk_src = ll_rcc_get_radio_sleep_clk();
    switch (linklayer_slp_clk_src)
    {
    case LL_RCC_RADIOSLEEPSOURCE_LSE:
        linklayer_slp_clk_src = RTC_SLPTMR;
        break;
    case LL_RCC_RADIOSLEEPSOURCE_LSI:
        linklayer_slp_clk_src = RCO_SLPTMR;
        break;
    case LL_RCC_RADIOSLEEPSOURCE_HSE_DIV:
        linklayer_slp_clk_src = CRYSTAL_OSCILLATOR_SLPTMR;
        break;
    case LL_RCC_RADIOSLEEPSOURCE_NONE:
    default:
        /* No clock source — should not happen */
        while (1) ;
        break;
    }
    ll_intf_cmn_le_select_slp_clk_src((uint8_t)linklayer_slp_clk_src, &freq_value);
}

/* ============================================================
 * Link Layer reset
 * ============================================================ */

void ll_sys_reset(void)
{
    uint8_t bsca;
    uint8_t drift_time = DRIFT_TIME_DEFAULT;
    uint8_t exec_time = EXEC_TIME_DEFAULT;

    /* Apply the selected link layer sleep timer source */
    ll_sys_sleep_clock_source_selection();

    /* Configure the link layer sleep clock accuracy */
    bsca = ll_sys_BLE_sleep_clock_accuracy_selection();
    ll_intf_le_set_sleep_clock_accuracy(bsca);

    /* Update link layer timings for LSI2 */
    if (ll_rcc_get_radio_sleep_clk() == LL_RCC_RADIOSLEEPSOURCE_LSI)
    {
        drift_time += DRIFT_TIME_EXTRA_LSI2;
        exec_time += EXEC_TIME_EXTRA_LSI2;
    }
    else
    {
#if defined(__GNUC__)
        /* GCC builds at -O0 execute slower — add extra timing margin
           to prevent scheduler deadline misses. Matches working project's
           GCC debug path. Our Makefile always uses -O0. */
        drift_time += DRIFT_TIME_EXTRA_GCC_DEBUG;
        exec_time += EXEC_TIME_EXTRA_GCC_DEBUG;
#endif
    }

    if ((drift_time != DRIFT_TIME_DEFAULT) || (exec_time != EXEC_TIME_DEFAULT))
    {
        ll_sys_config_BLE_schldr_timings(drift_time, exec_time);
    }
}
