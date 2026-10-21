/**
 * linklayer_plat.c — Link Layer platform adaptation layer
 *
 * Adapted from ST's reference implementation for use without the ST HAL.
 * Uses our ll_rcc.h / hal_common.h register-level drivers instead.
 *
 * Original: STM32_WPAN/Target/linklayer_plat.c
 * Copyright (c) 2025 STMicroelectronics. Licensed under ST terms.
 */

#include <stdint.h>
#include <string.h>
#include "hal_common.h"
#include "ll_rcc.h"
#include "ll_pwr.h"
#include "app_conf.h"
#include "linklayer_plat.h"

/* SystemCoreClock — defined in system_stm32wbaxx.c (sdk/stm/).
   Needed by DelayUs for cycle counting. */
extern uint32_t SystemCoreClock;

/* Forward decl — provided by the link-layer library (BLE or Thread) */
extern uint32_t ll_intf_cmn_get_slptmr_value(void);

/* HW_RNG_Get — provided by hw_rng.c (sdk/stm/) */
extern void HW_RNG_Get(uint8_t n, uint32_t *val);

#define max(a,b) ((a) > (b) ? (a) : (b))

/* ---- RCC register helpers ---- */

/* RCC_RADIOENR at offset 0x208 */
#define _RCC_RADIOENR          REG32(RCC_BASE + 0x208UL)
#define _RADIOENR_BBCLKEN      (1UL << 1)
#define _RADIOENR_STRADIOCLKON (1UL << 16)
#define _RADIOENR_RADIOCLKRDY  (1UL << 17)

/* RCC_AHB5SMENR at offset 0x0C0 (NOT 0x0D8!) */
#define _RCC_AHB5SMENR         REG32(RCC_BASE + 0x0C0UL)

/* RCC_CR at offset 0x00 — HSEON bit 16, HSERDY bit 17 */
#define _RCC_CR                REG32(RCC_BASE + 0x00UL)

/* ---- NVIC helpers ---- */

static inline uint32_t _nvic_get_active(uint32_t irqn)
{
    /* NVIC_IABR registers at 0xE000E300, 32 IRQs per register */
    volatile uint32_t *iabr = (volatile uint32_t *)(0xE000E300UL + ((irqn >> 5) * 4));
    return (*iabr >> (irqn & 0x1F)) & 1UL;
}

/* ---- 2.4GHz RADIO ISR callbacks (defined in wpan_irq.c) ---- */
extern void (*radio_callback)(void);
extern void (*low_isr_callback)(void);
extern volatile uint8_t radio_sw_low_isr_is_running_high_prio;

/* Radio critical sections */
static uint32_t primask_bit = 0;
volatile int32_t prio_high_isr_counter = 0;
volatile int32_t prio_low_isr_counter = 0;
volatile int32_t prio_sys_isr_counter = 0;
volatile int32_t irq_counter = 0;
volatile uint32_t local_basepri_value = 0;

/* Radio bus clock control variables */
uint8_t AHB5_SwitchedOff = 0;
uint32_t radio_sleep_timer_val = 0;

/* Statistics for the radio-bus guards below (initialized, so .data / SRAM1:
 * readable over SWD while the Core sleeps in the BLE idle loop). */
volatile wpan_busclk_stats_t g_wpan_busclk_stats = { .magic = 0x42554C4Bu };

#define BUSCLK_WAIT_BOUND  2000000u   /* loops; far beyond the ~1.6 ms resync seen 2026-09-25 */

/* Spin until the radio sleep timer has moved twice from `from`: the first
 * change can be the frozen value giving way to the live one, the second
 * proves the timer is ticking (one tick is ~32 us at HSE/1000). Bounded, and
 * counted, so a dead bus shows up in the statistics instead of hanging here. */
static void _busclk_wait_ticking(uint32_t from, volatile uint32_t *loops_max,
                                 volatile uint32_t *timeouts)
{
    uint32_t loops = 0, v = from;
    for (int ticks = 0; ticks < 2; ticks++) {
        uint32_t ref = v;
        do {
            v = ll_intf_cmn_get_slptmr_value();
            loops++;
        } while (v == ref && loops < BUSCLK_WAIT_BOUND);
        if (loops >= BUSCLK_WAIT_BOUND) { (*timeouts)++; break; }
    }
    if (loops > *loops_max) *loops_max = loops;
}

/* Low-priority radio ISR entry and exit: the guard against the link layer's
 * RCO-compensation assert.
 *
 * llhwc_cmn_hw_rco_compnstn_enable (LinkLayer_BLE_Basic_lib.a, llhwc_cmn.o,
 * run from llhwc_cmn_rco_isr in the low ISR) reads the sleep timer at
 * 0x48020014 directly, without the bus-ready hook the rest of the library
 * uses, and asserts that it is strictly past g_rco_compnstn_enable_time_ref,
 * the value it stored at a tick edge when it last enabled the timer's
 * hardware compensation. With that compensation on, the counter is not
 * monotonic from one read to the next: on the bench (Core.ST.W5, HSE/1000
 * sleep clock, 2026-10-20) it read 0 to 2 ticks below the reference a few
 * microseconds after reading above it, and the assert, an infinite loop in
 * ll_sys_assert, showed up as watchdog resets after 1 to 107 minutes of
 * advertising. So before the link layer runs here: the timer must move (live
 * bus), be past the tick the previous low ISR exited in, and be at least
 * LOWISR_REF_MARGIN_TICKS past the reference. Cost: a few poll loops as a
 * rule, up to ~250 us when the margin wait engages (about every 3 minutes
 * while advertising, counted in low_isr_behind_ref). Bench: no hang in 6 h 43
 * min advertising and 2 h 10 min streaming with the guard, against hangs at
 * 24, 6, 107, 8, 58, 1 and 29 minutes without it. Evidence and register
 * values: portal page "KINETIK", bench log of 20 October 2026. */
static uint32_t s_low_isr_exit_tick = 0xFFFFFFFFu;

/* The link layer's reference, exported by llhwc_cmn.o (64-bit, timer value in
 * the low word). Read only. */
extern volatile uint64_t g_rco_compnstn_enable_time_ref;

#define LOWISR_REF_WAIT_BOUND  20000u   /* loops, ~10 ms: far past the ~1.6 ms resync */
#define LOWISR_REF_MARGIN_TICKS 8        /* ~250 us at 32 kHz */

void LINKLAYER_PLAT_LowIsrEnter(void)
{
    g_wpan_busclk_stats.low_isr++;
    uint32_t loops = 0, v, v0 = ll_intf_cmn_get_slptmr_value();
    do {                                   /* live bus: the timer moves */
        v = ll_intf_cmn_get_slptmr_value();
        loops++;
    } while (v == v0 && loops < BUSCLK_WAIT_BOUND);
    if (v == s_low_isr_exit_tick) {        /* same tick as the previous exit */
        g_wpan_busclk_stats.low_isr_same_tick++;
        while (v == s_low_isr_exit_tick && loops < BUSCLK_WAIT_BOUND) {
            v = ll_intf_cmn_get_slptmr_value();
            loops++;
        }
    }
    if (loops >= BUSCLK_WAIT_BOUND) g_wpan_busclk_stats.low_isr_timeouts++;
    if (loops > g_wpan_busclk_stats.low_isr_loops_max) g_wpan_busclk_stats.low_isr_loops_max = loops;

    /* The precondition the RCO compensation asserts, with margin (wrap-safe
     * on the low word). */
    uint32_t ref = (uint32_t)g_rco_compnstn_enable_time_ref;
    uint32_t rloops = 0;
    if ((int32_t)(v - ref) < LOWISR_REF_MARGIN_TICKS) {
        g_wpan_busclk_stats.low_isr_behind_ref++;
        while ((int32_t)(v - ref) < LOWISR_REF_MARGIN_TICKS && rloops < LOWISR_REF_WAIT_BOUND) {
            v = ll_intf_cmn_get_slptmr_value();
            rloops++;
        }
        if (rloops >= LOWISR_REF_WAIT_BOUND) g_wpan_busclk_stats.low_isr_ref_timeouts++;
        if (rloops > g_wpan_busclk_stats.low_isr_ref_loops_max) g_wpan_busclk_stats.low_isr_ref_loops_max = rloops;
    }
}

void LINKLAYER_PLAT_LowIsrExit(void)
{
    s_low_isr_exit_tick = ll_intf_cmn_get_slptmr_value();
}

/* ---- CMSIS intrinsics ---- */
static inline uint32_t _get_PRIMASK(void) {
    uint32_t r;
    __asm volatile ("MRS %0, primask" : "=r" (r));
    return r;
}
static inline void _set_PRIMASK(uint32_t v) {
    __asm volatile ("MSR primask, %0" :: "r" (v) : "memory");
}
static inline void _disable_irq(void) {
    __asm volatile ("cpsid i" ::: "memory");
}
static inline uint32_t _get_BASEPRI(void) {
    uint32_t r;
    __asm volatile ("MRS %0, basepri" : "=r" (r));
    return r;
}
static inline void _set_BASEPRI(uint32_t v) {
    __asm volatile ("MSR basepri, %0" :: "r" (v) : "memory");
}
static inline void _set_BASEPRI_MAX(uint32_t v) {
    __asm volatile ("MSR basepri_max, %0" :: "r" (v) : "memory");
}

/* ============================================================
 * Clock initialization
 * ============================================================ */

void LINKLAYER_PLAT_ClockInit(void)
{
    /* Verify sleep timer clock source is configured */
    if (ll_rcc_get_radio_sleep_clk() == LL_RCC_RADIOSLEEPSOURCE_NONE)
    {
        /* Should be selected before — fallback to HSE/1024 */
        ll_pwr_enable_backup_access();
        ll_rcc_set_radio_sleep_clk(LL_RCC_RADIOSLEEPSOURCE_HSE_DIV);
    }

    /* Enable AHB5ENR peripheral clock (bus CLK) */
    ll_rcc_ahb5_clk_enable(LL_AHB5_RADIO);

    /* Enable AHB5 radio clock in sleep mode */
    ll_rcc_ahb5_clk_sleep_enable();
}

/* ============================================================
 * Delay / Assert
 * ============================================================ */

void LINKLAYER_PLAT_DelayUs(uint32_t delay)
{
    volatile uint32_t count = delay * (SystemCoreClock / 1000000U);
    do {
        __asm volatile ("nop");
    } while (count--);
}

void LINKLAYER_PLAT_Assert(uint8_t condition)
{
    if (!condition) {
        while (1) ;
    }
}

/* ============================================================
 * AHB5 bus clock management
 * ============================================================ */

void LINKLAYER_PLAT_WaitHclkRdy(void)
{
    /* Wait for radio bus clock readiness after a WFI: the sleep timer must
     * tick, not merely differ from the value sampled at the wake. */
    if (AHB5_SwitchedOff == 1)
    {
        AHB5_SwitchedOff = 0;
        g_wpan_busclk_stats.wait_calls++;
        _busclk_wait_ticking(radio_sleep_timer_val, &g_wpan_busclk_stats.wait_loops_max,
                             &g_wpan_busclk_stats.wait_timeouts);
    }
}

void LINKLAYER_PLAT_NotifyWFIEnter(void)
{
    /* CubeWBA predicts whether AHB5 will be cut during WFI (radio not
     * ACTIVE, or neither RADIOSMEN nor STRADIOCLKON set) and resyncs only
     * then. The prediction is kept for the statistics, but every WFI is
     * treated as a possible cut: the wake then always samples the sleep
     * timer and waits for it to tick (at most ~64 us when the bus was never
     * cut). Part of the RCO-compensation guard, see LINKLAYER_PLAT_LowIsrEnter. */
    g_wpan_busclk_stats.wfi_enter++;
    if ((ll_pwr_get_radio_mode() != LL_PWR_RADIO_ACTIVE_MODE) ||
        ((_RCC_AHB5SMENR & LL_AHB5_RADIO) == 0 && (_RCC_RADIOENR & _RADIOENR_STRADIOCLKON) == 0))
    {
        g_wpan_busclk_stats.wfi_predicted_cut++;
    }
    AHB5_SwitchedOff = 1;
}

void LINKLAYER_PLAT_NotifyWFIExit(void)
{
    /* As CubeWBA: if AHB5 was cut, sample the (frozen) sleep timer as early
     * as possible; LINKLAYER_PLAT_WaitHclkRdy() then waits for it to move. */
    if (AHB5_SwitchedOff)
        radio_sleep_timer_val = ll_intf_cmn_get_slptmr_value();
}

/* ============================================================
 * Active clock (baseband clock) control
 * ============================================================ */

void LINKLAYER_PLAT_AclkCtrl(uint8_t enable)
{
    if (enable != 0u)
    {
        /* Enable RADIO baseband clock (BBCLKEN = bit 1 of RCC_RADIOENR)
         * — matches HAL_RCCEx_EnableRadioBBClock() in working project */
        ll_rcc_radio_bb_clk_enable();

        /* Wait for HSE to be ready */
        while (!ll_rcc_hse_ready()) ;
    }
    else
    {
        /* Disable RADIO baseband clock */
        ll_rcc_radio_bb_clk_disable();
    }
}

/* ============================================================
 * RNG
 * ============================================================ */

void LINKLAYER_PLAT_GetRNG(uint8_t *ptr_rnd, uint32_t len)
{
    uint32_t nb_remaining_rng = len;
    uint32_t generated_rng;

    while (nb_remaining_rng >= 4)
    {
        generated_rng = 0;
        HW_RNG_Get(1, &generated_rng);
        memcpy((ptr_rnd + (len - nb_remaining_rng)), &generated_rng, 4);
        nb_remaining_rng -= 4;
    }
    if (nb_remaining_rng > 0)
    {
        generated_rng = 0;
        HW_RNG_Get(1, &generated_rng);
        memcpy((ptr_rnd + (len - nb_remaining_rng)), &generated_rng, nb_remaining_rng);
    }
}

/* ============================================================
 * Interrupt setup
 * ============================================================ */

void LINKLAYER_PLAT_SetupRadioIT(void (*intr_cb)())
{
    radio_callback = intr_cb;
    hal_nvic_set_priority(RADIO_INTR_NUM, RADIO_INTR_PRIO_HIGH);
    hal_nvic_enable_irq(RADIO_INTR_NUM);
}

void LINKLAYER_PLAT_SetupSwLowIT(void (*intr_cb)())
{
    low_isr_callback = intr_cb;
    hal_nvic_set_priority(RADIO_SW_LOW_INTR_NUM, RADIO_SW_LOW_INTR_PRIO);
    hal_nvic_enable_irq(RADIO_SW_LOW_INTR_NUM);
}

void LINKLAYER_PLAT_TriggerSwLowIT(uint8_t priority)
{
    uint8_t low_isr_priority = RADIO_INTR_PRIO_LOW;

    if (_nvic_get_active(RADIO_SW_LOW_INTR_NUM) == 0)
    {
        if (priority == 0)
        {
            low_isr_priority = RADIO_SW_LOW_INTR_PRIO;
        }
        hal_nvic_set_priority(RADIO_SW_LOW_INTR_NUM, low_isr_priority);
    }
    else
    {
        if (priority != 0)
        {
            radio_sw_low_isr_is_running_high_prio = 1;
        }
    }

    ll_nvic_set_pending(RADIO_SW_LOW_INTR_NUM);
}

/* ============================================================
 * Global interrupt control
 * ============================================================ */

void LINKLAYER_PLAT_EnableIRQ(void)
{
    irq_counter = max(0, irq_counter - 1);
    if (irq_counter == 0)
    {
        _set_PRIMASK(primask_bit);
    }
}

void LINKLAYER_PLAT_DisableIRQ(void)
{
    if (irq_counter == 0)
    {
        primask_bit = _get_PRIMASK();
    }
    _disable_irq();
    irq_counter++;
}

/* ============================================================
 * Specific interrupt group control
 * ============================================================ */

void LINKLAYER_PLAT_EnableSpecificIRQ(uint8_t isr_type)
{
    if ((isr_type & LL_HIGH_ISR_ONLY) != 0)
    {
        prio_high_isr_counter--;
        if (prio_high_isr_counter == 0)
        {
            hal_nvic_enable_irq(RADIO_INTR_NUM);
        }
    }
    if ((isr_type & LL_LOW_ISR_ONLY) != 0)
    {
        prio_low_isr_counter--;
        if (prio_low_isr_counter == 0)
        {
            hal_nvic_enable_irq(RADIO_SW_LOW_INTR_NUM);
        }
    }
    if ((isr_type & SYS_LOW_ISR) != 0)
    {
        prio_sys_isr_counter--;
        if (prio_sys_isr_counter == 0)
        {
            _set_BASEPRI(local_basepri_value);
        }
    }
}

void LINKLAYER_PLAT_DisableSpecificIRQ(uint8_t isr_type)
{
    if ((isr_type & LL_HIGH_ISR_ONLY) != 0)
    {
        prio_high_isr_counter++;
        if (prio_high_isr_counter == 1)
        {
            hal_nvic_disable_irq(RADIO_INTR_NUM);
        }
    }
    if ((isr_type & LL_LOW_ISR_ONLY) != 0)
    {
        prio_low_isr_counter++;
        if (prio_low_isr_counter == 1)
        {
            hal_nvic_disable_irq(RADIO_SW_LOW_INTR_NUM);
        }
    }
    if ((isr_type & SYS_LOW_ISR) != 0)
    {
        prio_sys_isr_counter++;
        if (prio_sys_isr_counter == 1)
        {
            local_basepri_value = _get_BASEPRI();
            _set_BASEPRI_MAX(RADIO_INTR_PRIO_LOW << 4);
        }
    }
}

/* ============================================================
 * Radio IT enable/disable
 * ============================================================ */

void LINKLAYER_PLAT_EnableRadioIT(void)
{
    hal_nvic_enable_irq(RADIO_INTR_NUM);
}

void LINKLAYER_PLAT_DisableRadioIT(void)
{
    hal_nvic_disable_irq(RADIO_INTR_NUM);
}

/* ============================================================
 * Radio event start/stop
 * ============================================================ */

void LINKLAYER_PLAT_StartRadioEvt(void)
{
    ll_rcc_ahb5_clk_sleep_enable();
    hal_nvic_set_priority(RADIO_INTR_NUM, RADIO_INTR_PRIO_HIGH);
}

void LINKLAYER_PLAT_StopRadioEvt(void)
{
    ll_rcc_ahb5_clk_sleep_disable();
    hal_nvic_set_priority(RADIO_INTR_NUM, RADIO_INTR_PRIO_LOW);
}

/* ============================================================
 * RCO calibration
 * ============================================================ */

void LINKLAYER_PLAT_RCOStartClbr(void)
{
    /* RCO calibration needs HSE — ensure it's running */
    while (!ll_rcc_hse_ready()) ;
}

void LINKLAYER_PLAT_RCOStopClbr(void)
{
    /* Nothing needed without SCM or LPM */
}

/* ============================================================
 * Temperature (not used without calibration)
 * ============================================================ */

void LINKLAYER_PLAT_RequestTemperature(void)
{
    /* Temperature-based radio calibration disabled */
}

/* ============================================================
 * OS context switch (no RTOS)
 * ============================================================ */

void LINKLAYER_PLAT_EnableOSContextSwitch(void)
{
}

void LINKLAYER_PLAT_DisableOSContextSwitch(void)
{
}

/* ============================================================
 * Scheduler timing update notification
 * ============================================================ */

void LINKLAYER_PLAT_SCHLDR_TIMING_UPDATE_NOT(Evnt_timing_t *p_evnt_timing)
{
    (void)p_evnt_timing;
}

/* ============================================================
 * Device identification
 * ============================================================ */

/* UID64 at 0x0BF90A00 (CubeWBA stm32wba55xx.h UID64_BASE = FLASH_ENGY_BASE
 * 0x0BF90500 + 0x500): word 0 the device number, word 1 [31:8] ST's company
 * ID (0x0080E1). As ST's LL_FLASH_GetUDN / LL_FLASH_GetSTCompanyID. These
 * used to read 0x0BFA07xx, which is unmapped: OpenThread's EUI-64 read
 * (otPlatRadioGetIeeeEui64) bus-faulted during otInstanceInit. */
#define WPAN_UID64_BASE  0x0BF90A00UL

uint32_t LINKLAYER_PLAT_GetSTCompanyID(void)
{
    return (*(volatile uint32_t *)(WPAN_UID64_BASE + 4UL) >> 8) & 0x00FFFFFFUL;
}

uint32_t LINKLAYER_PLAT_GetUDN(void)
{
    return *(volatile uint32_t *)WPAN_UID64_BASE;
}
