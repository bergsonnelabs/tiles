/**
 * stm32_timer_if.c -- timer server (stm32_timer / UTIL_TIMER) interface on
 * LPTIM2 + SysTick
 *
 * Time base: SysTick milliseconds (_systick_ticks). UTIL_TIMER's ticks are
 * milliseconds; its context, elapsed time and current time all read SysTick.
 *
 * Wake-up: LPTIM2 as a one-shot, clocked from PCLK1 / 32. LPTIM2 is 16 bits,
 * so one shot reaches at most 0xFFFF ticks: ~131 ms at 16 MHz, ~65 ms at
 * 32 MHz, ~33 ms at 64 MHz, ~21 ms at 100 MHz. Longer timeouts are chained:
 * StartTimerEvt records the absolute deadline (timer context + timeout, the
 * way ST's timer_if.c adds RtcTimerContext to the alarm), arms the largest
 * chunk that fits, and the LPTIM2 interrupt re-arms the remainder from
 * SysTick until the deadline is reached. Only then does it call
 * UTIL_TIMER_IRQ_Handler, so UTIL_TIMER only ever sees an expired head timer.
 *
 * Before 2026-10-11 the interrupt called UTIL_TIMER_IRQ_Handler on every
 * shot. A timeout past one shot fired early, UTIL_TIMER found nothing expired,
 * and because the head was still marked pending it never re-armed: any timer
 * over ~65 ms (at 32 MHz) never fired. The timeout was also taken as "from
 * now" rather than "from the timer context", so a timer inserted as the new
 * head while others ran was armed late by the time since the context was set.
 * The same stuck-pending path caught a shot that ended a hair before SysTick
 * counted the last millisecond; that case now re-arms one more millisecond.
 *
 * Timeouts up to 2^32 ms are exact to SysTick's 1 ms resolution, as before.
 * LPTIM2 is not routed to any Core.ST.W5 pad, so it is reserved for this.
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

/* LPTIM2's kernel clock is PCLK1 (RCC_CCIPR1.LPTIM2SEL reset value), / 32. */
#define TIMER_IF_TICKS_PER_MS   (PCLK1_HZ / 32UL / 1000UL)
#define TIMER_IF_MAX_SHOT       0xFFFFUL      /* 16-bit ARR */
#define TIMER_IF_MAX_SHOT_MS    (TIMER_IF_MAX_SHOT / TIMER_IF_TICKS_PER_MS)

_Static_assert(TIMER_IF_TICKS_PER_MS >= 1UL, "LPTIM2: PCLK1 / 32 below 1 tick per ms");
_Static_assert(TIMER_IF_MAX_SHOT_MS >= 1UL, "LPTIM2: one shot shorter than 1 ms");

/* Masks interrupts around the deadline bookkeeping: the LPTIM2 interrupt
 * re-arms from it, and timers are started from higher-priority interrupts too
 * (the link layer's RFTS window callback). The host test overrides it. */
#ifndef TIMER_IF_ENTER_CRITICAL
#define TIMER_IF_ENTER_CRITICAL()  UTILS_ENTER_CRITICAL_SECTION()
#define TIMER_IF_EXIT_CRITICAL()   UTILS_EXIT_CRITICAL_SECTION()
#endif

/* SysTick for elapsed time tracking */
extern volatile uint32_t _systick_ticks;

/* Timer context (SysTick ms) */
static uint32_t timer_context;

/* The armed deadline: s_arm_timeout ms after s_arm_ctx. */
static volatile uint32_t s_arm_ctx;
static volatile uint32_t s_arm_timeout;
static volatile uint8_t  s_armed;

/* Start one LPTIM2 shot of `ticks` (1..0xFFFF) LPTIM ticks. */
static void lptim2_shot(uint32_t ticks)
{
    /* Disable before reconfiguring */
    LPTIM2_CR = 0;

    /* Clear pending flags */
    LPTIM2_ICR = LPTIM_ISR_ARRM;

    /* Enable the timer (must be done before writing ARR/CCR on LPTIM) */
    LPTIM2_CR = LPTIM_CR_ENABLE;

    if (ticks < 2UL) ticks = 2UL;     /* ARR >= 1 */
    LPTIM2_ARR = (uint16_t)(ticks - 1UL);

    /* Enable ARR match interrupt */
    LPTIM2_DIER = LPTIM_DIER_ARRMIE;

    /* Start single-shot countdown */
    LPTIM2_CR = LPTIM_CR_ENABLE | LPTIM_CR_SNGSTRT;
}

/* Arm the next shot toward the deadline, `remaining_ms` (>= 1) away. */
static void lptim2_arm_ms(uint32_t remaining_ms)
{
    if (remaining_ms > TIMER_IF_MAX_SHOT_MS)
        lptim2_shot(TIMER_IF_MAX_SHOT);
    else
        lptim2_shot(remaining_ms * TIMER_IF_TICKS_PER_MS);   /* <= 0xFFFF, no overflow */
}

/* ---- LPTIM2 ISR: one shot ended ---- */

void LPTIM2_IRQHandler(void)
{
    int expired = 0;

    TIMER_IF_ENTER_CRITICAL();
    /* Clear ARR match flag, disable timer */
    LPTIM2_ICR = LPTIM_ISR_ARRM;
    LPTIM2_CR = 0;

    if (s_armed) {
        uint32_t elapsed = _systick_ticks - s_arm_ctx;   /* intentional wrap */
        if (elapsed < s_arm_timeout) {
            lptim2_arm_ms(s_arm_timeout - elapsed);      /* not there yet: next chunk */
        } else {
            s_armed = 0;
            expired = 1;
        }
    }
    /* else: a stale interrupt from a shot that was stopped or replaced */
    TIMER_IF_EXIT_CRITICAL();

    if (expired)
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

    /* Prescaler /32: effective clock = PCLK1/32 (32 MHz: 1 ms = 1000 ticks). */
    LPTIM2_CFGR = LPTIM_CFGR_PRESC_DIV32;

    /* Enable LPTIM2 interrupt in NVIC */
    hal_nvic_set_priority(HAL_IRQ_LPTIM2, 8);
    hal_nvic_enable_irq(HAL_IRQ_LPTIM2);

    timer_context = 0;
    s_armed = 0;
    return UTIL_TIMER_OK;
}

static UTIL_TIMER_Status_t TimerIF_DeInit(void)
{
    LPTIM2_CR = 0;
    s_armed = 0;
    hal_nvic_disable_irq(HAL_IRQ_LPTIM2);
    return UTIL_TIMER_OK;
}

/* `timeout` is in ms from the timer context (UTIL_TIMER keeps every
 * Timestamp relative to it), not from now. */
static UTIL_TIMER_Status_t TimerIF_StartTimer(uint32_t timeout)
{
    TIMER_IF_ENTER_CRITICAL();
    s_arm_ctx = timer_context;
    s_arm_timeout = timeout;
    s_armed = 1;

    uint32_t elapsed = _systick_ticks - timer_context;   /* intentional wrap */
    if (elapsed < timeout)
        lptim2_arm_ms(timeout - elapsed);
    else
        lptim2_shot(2UL);                                /* already due: fire now */
    TIMER_IF_EXIT_CRITICAL();

    return UTIL_TIMER_OK;
}

static UTIL_TIMER_Status_t TimerIF_StopTimer(void)
{
    TIMER_IF_ENTER_CRITICAL();
    s_armed = 0;
    LPTIM2_CR = 0;
    LPTIM2_DIER = 0;
    LPTIM2_ICR = LPTIM_ISR_ARRM;
    TIMER_IF_EXIT_CRITICAL();
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
