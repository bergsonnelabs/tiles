/**
 * stm32_timer_if.c -- BLE timer server interface using LPTIM2 interrupt
 *
 * Uses LPTIM2 as a one-shot hardware timer for the BLE timer server.
 * When the BLE stack sets an alarm, LPTIM2 counts and fires an
 * interrupt at the exact timeout. This provides deterministic timing
 * for BLE connection event scheduling.
 *
 * LPTIM2 is not exposed on any Core.ST.W5 pads, so it's safe to reserve
 * for BLE. Timer ticks = milliseconds.
 */

#include <stdint.h>
#include "stm32_timer.h"
#include "ll_common.h"
#include "ll_rcc.h"
#include "hal_common.h"
#include "core_config.h"

/* LPTIM2 registers (APB1 domain, 0x40009400) */
#define LPTIM2_BASE_ADDR  0x40009400UL
#define LPTIM2_ISR   REG32(LPTIM2_BASE_ADDR + 0x00UL)
#define LPTIM2_ICR   REG32(LPTIM2_BASE_ADDR + 0x04UL)
#define LPTIM2_DIER  REG32(LPTIM2_BASE_ADDR + 0x08UL)
#define LPTIM2_CFGR  REG32(LPTIM2_BASE_ADDR + 0x0CUL)
#define LPTIM2_CR    REG32(LPTIM2_BASE_ADDR + 0x10UL)
#define LPTIM2_CCR1  REG32(LPTIM2_BASE_ADDR + 0x14UL)
#define LPTIM2_ARR   REG32(LPTIM2_BASE_ADDR + 0x18UL)
#define LPTIM2_CNT   REG32(LPTIM2_BASE_ADDR + 0x1CUL)

/* ISR/ICR bits */
#define LPTIM_ISR_ARRM   (1UL << 1)   /* Auto-reload match */

/* DIER bits */
#define LPTIM_DIER_ARRMIE (1UL << 1)  /* ARR match interrupt enable */

/* CR bits */
#define LPTIM_CR_ENABLE  (1UL << 0)
#define LPTIM_CR_SNGSTRT (1UL << 1)   /* Single start (one-shot) */

/* CFGR prescaler bits [11:9] */
#define LPTIM_CFGR_PRESC_DIV1   (0x0UL << 9)
#define LPTIM_CFGR_PRESC_DIV32  (0x5UL << 9)

/* SysTick for elapsed time tracking */
extern volatile uint32_t _systick_ticks;

/* Timer context */
static uint32_t timer_context;

/* ---- LPTIM2 ISR — fires when one-shot timer expires ---- */

void LPTIM2_IRQHandler(void)
{
    /* Clear ARR match flag */
    LPTIM2_ICR = LPTIM_ISR_ARRM;

    /* Disable timer */
    LPTIM2_CR = 0;

    /* Call BLE timer server handler */
    UTIL_TIMER_IRQ_Handler();
}

/* ---- Low-level driver functions ---- */

static UTIL_TIMER_Status_t TimerIF_Init(void)
{
    /* Enable LPTIM2 clock (APB1ENR2 bit 5) */
    SET_BITS(REG32(RCC_BASE + 0xA0UL), (1UL << 5));
    (void)REG32(RCC_BASE + 0xA0UL);

    /* Reset LPTIM2 */
    LPTIM2_CR = 0;

    /* Prescaler /32: effective clock = SYSCLK/32.
     * At 32MHz: 1ms = 1000 ticks. Max timeout = 65ms (16-bit ARR). */
    LPTIM2_CFGR = LPTIM_CFGR_PRESC_DIV32;

    /* Enable LPTIM2 interrupt in NVIC */
    hal_nvic_set_priority(HAL_IRQ_LPTIM2, 8);
    hal_nvic_enable_irq(HAL_IRQ_LPTIM2);

    timer_context = 0;
    return UTIL_TIMER_OK;
}

static UTIL_TIMER_Status_t TimerIF_DeInit(void)
{
    LPTIM2_CR = 0;
    hal_nvic_disable_irq(HAL_IRQ_LPTIM2);
    return UTIL_TIMER_OK;
}

static UTIL_TIMER_Status_t TimerIF_StartTimer(uint32_t timeout)
{
    /* Disable before reconfiguring */
    LPTIM2_CR = 0;

    /* Clear pending flags */
    LPTIM2_ICR = LPTIM_ISR_ARRM;

    /* Enable the timer (must be done before writing ARR/CCR on LPTIM) */
    LPTIM2_CR = LPTIM_CR_ENABLE;

    /* Convert ms to LPTIM ticks: ticks = ms * (SYSCLK/32/1000) */
    uint32_t ticks_per_ms = SYSCLK_HZ / 32 / 1000;
    uint32_t arr = timeout * ticks_per_ms;
    if (arr == 0) arr = 1;
    if (arr > 0xFFFF) arr = 0xFFFF;
    LPTIM2_ARR = (uint16_t)(arr - 1);

    /* Enable ARR match interrupt */
    LPTIM2_DIER = LPTIM_DIER_ARRMIE;

    /* Start single-shot countdown */
    LPTIM2_CR = LPTIM_CR_ENABLE | LPTIM_CR_SNGSTRT;

    return UTIL_TIMER_OK;
}

static UTIL_TIMER_Status_t TimerIF_StopTimer(void)
{
    LPTIM2_CR = 0;
    LPTIM2_DIER = 0;
    LPTIM2_ICR = LPTIM_ISR_ARRM;
    return UTIL_TIMER_OK;
}

static uint32_t TimerIF_SetTimerContext(void)
{
    timer_context = _systick_ticks;
    return timer_context;
}

static uint32_t TimerIF_GetTimerContext(void)
{
    return timer_context;
}

static uint32_t TimerIF_GetTimerElapsedTime(void)
{
    return _systick_ticks - timer_context;
}

static uint32_t TimerIF_GetTimerValue(void)
{
    return _systick_ticks;
}

static uint32_t TimerIF_GetMinimumTimeout(void)
{
    return 1;
}

static uint32_t TimerIF_ms2Tick(uint32_t ms)
{
    return ms;
}

static uint32_t TimerIF_Tick2ms(uint32_t tick)
{
    return tick;
}

/* ---- The driver struct ---- */

const UTIL_TIMER_Driver_s UTIL_TimerDriver =
{
    .InitTimer          = TimerIF_Init,
    .DeInitTimer        = TimerIF_DeInit,
    .StartTimerEvt      = TimerIF_StartTimer,
    .StopTimerEvt       = TimerIF_StopTimer,
    .SetTimerContext     = TimerIF_SetTimerContext,
    .GetTimerContext     = TimerIF_GetTimerContext,
    .GetTimerElapsedTime = TimerIF_GetTimerElapsedTime,
    .GetTimerValue       = TimerIF_GetTimerValue,
    .GetMinimumTimeout   = TimerIF_GetMinimumTimeout,
    .ms2Tick             = TimerIF_ms2Tick,
    .Tick2ms             = TimerIF_Tick2ms,
};
