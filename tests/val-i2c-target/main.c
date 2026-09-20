/**
 * val-i2c-target — compile validation for I2C target (device) mode and the
 * ADC DMA millivolt helpers.
 *
 * Calls every public entry point added alongside the l0-adc-mux sensor hub
 * and casts the returns away. It proves the API compiles and links on all
 * four Cores; it does not prove anything about behavior on a bus. Hardware
 * validation lives with the l0-adc-mux project.
 *
 * No pads are assigned, so nothing here is expected to run correctly on a
 * board. Build only:
 *   make TILE=Core.ST.L0.1 && make TILE=Core.ST.L4.1
 *   make TILE=Core.ST.W5   && make TILE=Core.ST.H5.1
 */

#include "core.h"
#include "core_adc.h"
#include "core_i2c.h"
#include "core_timer.h"
#include "core_nvm.h"

static uint8_t regs[16];
static core_i2c_target_t tgt;
static uint16_t dma_buf[8];

static void on_evt(void *ctx, hal_i2c_target_event_t evt,
                   uint16_t reg, uint8_t value)
{
    (void)ctx; (void)evt; (void)reg; (void)value;
}

int main(void)
{
    core_init();

    /* ---- Critical section primitive ---- */
    uint32_t s = ll_irq_save();
    ll_irq_restore(s);

    /* ---- I2C target mode ---- */

    /* Read-only register file. */
    (void)core_i2c_target_init(&tgt, I2C1, 0x28, I2C_400K,
                               regs, sizeof(regs), on_evt, (void *)0);

    /* Diagnostics counters are part of the contract. */
    (void)tgt.n_reads;
    (void)tgt.n_writes;
    (void)tgt.n_errors;

    /* Servicing the vector by hand is supported for custom vector tables. */
    hal_i2c_target_irq(&tgt);
    core_i2c_target_deinit(&tgt);

    /* Writable window: regs[8..11] accept host writes, the rest stay
     * read-only. */
    (void)core_i2c_target_init_rw(&tgt, I2C1, 0x29, I2C_100K,
                                  regs, sizeof(regs), 8, 4, on_evt, (void *)0);
    core_i2c_target_deinit(&tgt);

    /* A window running past the end of the register file must be refused
     * rather than allowing a host write off the end of the buffer. */
    if (core_i2c_target_init_rw(&tgt, I2C1, 0x2A, I2C_100K,
                                regs, sizeof(regs), 14, 8,
                                (hal_i2c_target_cb_t)0, (void *)0) == HAL_OK) {
        core_i2c_target_deinit(&tgt);
    }

    /* The low-level primitives underneath, exercised directly. */
    ll_i2c_target_init(I2C1, LL_I2C_TIMING_100K_16MHZ, 0x28);
    (void)ll_i2c_target_is_read(I2C1);
    (void)ll_i2c_target_addcode(I2C1);
    ll_i2c_target_flush_tx(I2C1);
    ll_i2c_target_clear_addr(I2C1);
    ll_i2c_target_disable(I2C1);

    /* ---- ADC: DMA millivolts ---- */

    core_adc_t adc;
    (void)core_adc_init(&adc, ADC_12BIT);

    /* Prime VDDA before DMA claims the regular sequence. */
    (void)core_adc_vdd(&adc);

    (void)core_adc_start_dma(&adc, dma_buf,
                             (uint16_t)(sizeof(dma_buf) / sizeof(dma_buf[0])),
                             (hal_callback_t)0, (void *)0);

    (void)core_adc_dma_slot(&adc, 5);
    (void)core_adc_dma_read(&adc, 5);
    (void)core_adc_dma_read_mv(&adc, 5);
    (void)core_adc_raw_to_mv(&adc, 2048);
    (void)hal_adc_raw_to_mv(&adc, 2048);
    (void)tal_adc_dma_slot(&adc, 5);
    (void)tal_adc_dma_read_pad_mv(&adc, 5);

    core_adc_stop_dma(&adc);

    /* ---- Timer-paced ADC (anti-aliasing path) ---- */

    core_timer_t pacer;
    core_timer_init_freq(&pacer, TIM2, 800);
    core_timer_set_trgo(&pacer, LL_TIM_MMS_UPDATE);
    ll_tim_set_trgo(TIM2, LL_TIM_MMS_UPDATE);

    /* Trigger encoding is family specific; LL_ADC_L0_TRG_* is the L0 set,
     * and 0x2 is in range on every family's EXTSEL field. */
    (void)core_adc_set_trigger(&adc, 0x2, ADC_TRIG_RISING);
    (void)hal_adc_set_trigger(&adc, 0x2, HAL_ADC_TRIG_FALLING);

    /* Out-of-range encodings and edges must be refused, not truncated. */
    if (hal_adc_set_trigger(&adc, 0xFF, HAL_ADC_TRIG_RISING) == HAL_OK) {
        for (;;) { }   /* unreachable: 0xFF exceeds EXTSEL on every family */
    }

    (void)core_adc_start_dma(&adc, dma_buf,
                             (uint16_t)(sizeof(dma_buf) / sizeof(dma_buf[0])),
                             (hal_callback_t)0, (void *)0);
    core_timer_start(&pacer);
    core_adc_stop_dma(&adc);
    core_timer_stop(&pacer);

    /* Back to free-running. */
    (void)core_adc_set_trigger(&adc, 0, ADC_TRIG_NONE);

    /* ---- NVM bounds ---- */

    uint8_t nvbuf[4];
    (void)core_nvm_size();
    (void)core_nvm_read(0, nvbuf, sizeof(nvbuf));

    /* A near-max offset with a small length must be refused. The old
     * `offset + len > SIZE` check wrapped and let this through. */
    if (core_nvm_read(0xFFFFFFF0UL, nvbuf, sizeof(nvbuf)) == 0) {
        for (;;) { }   /* unreachable */
    }
    if (core_nvm_write(0xFFFFFFF0UL, nvbuf, sizeof(nvbuf)) == 0) {
        for (;;) { }   /* unreachable */
    }

    for (;;) { }
}
