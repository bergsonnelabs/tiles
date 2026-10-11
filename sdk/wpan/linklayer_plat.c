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

/* Forward decl — provided by LinkLayer_BLE_Basic_lib.a */
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

/* ---- 2.4GHz RADIO ISR callbacks (defined in ble_irq.c) ---- */
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
    /* Wait for radio bus clock readiness if it was turned off during sleep */
    if (AHB5_SwitchedOff == 1)
    {
        AHB5_SwitchedOff = 0;
        /* Wait for sleep timer to tick — proves bus clock is back */
        uint32_t timeout = 100000;
        while (radio_sleep_timer_val == ll_intf_cmn_get_slptmr_value() && --timeout)
            ;
    }
}

void LINKLAYER_PLAT_NotifyWFIEnter(void)
{
    /* Check if AHB5 clock will be cut during WFI:
     * - Radio not in ACTIVE mode, OR
     * - Neither RADIOSMEN nor STRADIOCLKON is set */
    if ((ll_pwr_get_radio_mode() != LL_PWR_RADIO_ACTIVE_MODE) ||
        ((_RCC_AHB5SMENR & LL_AHB5_RADIO) == 0 && (_RCC_RADIOENR & _RADIOENR_STRADIOCLKON) == 0))
    {
        AHB5_SwitchedOff = 1;
    }
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

uint32_t LINKLAYER_PLAT_GetSTCompanyID(void)
{
    /* UID96[95:64] at 0x0BFA0700 — company ID in lower 24 bits */
    return *(volatile uint32_t *)0x0BFA0700UL & 0x00FFFFFFUL;
}

uint32_t LINKLAYER_PLAT_GetUDN(void)
{
    /* UID96[63:32] at 0x0BFA0704 */
    return *(volatile uint32_t *)0x0BFA0704UL;
}
