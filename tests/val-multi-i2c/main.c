/**
 * val-multi-i2c -- Validation: Dual I2C buses + ADC DMA on Core.ST.H5
 *
 * Core.ST.H5, clock=high (128 MHz)
 * Pad 4 = I2C1.CLK, Pad 5 = I2C1.DAT (400 kHz)
 * Pad 2 = I2C3.CLK, Pad 3 = I2C3.DAT (100 kHz)
 * Pad 8 = ADC7+
 *
 * Exercises: core_init, core_i2c_init (h, I2Cx, speed_hz) for I2C1 and I2C3,
 *            core_i2c_probe, core_adc_t, core_adc_init, core_adc_add,
 *            core_adc_start_dma (adc, buf, len, cb, ctx)
 *
 * DMA callback signature: void(void *ctx) -- hal_callback_t
 */

#include "core.h"
#include "core_i2c.h"
#include "core_adc.h"

static volatile uint32_t dma_count;

/* DMA callback -- hal_callback_t signature: void(void *ctx) */
static void dma_cb(void *ctx)
{
    (void)ctx;
    dma_count++;
}

int main(void)
{
    core_init();
    core_led_init();

    /* I2C1 at 400 kHz -- core_i2c_init(h, I2C1, speed_hz) */
    core_i2c_t i2c1;
    core_i2c_init(&i2c1, I2C1, I2C_400K);

    /* I2C3 at 100 kHz */
    core_i2c_t i2c3;
    core_i2c_init(&i2c3, I2C3, I2C_100K);

    /* Probe a device on each bus */
    hal_status_t probe1 = core_i2c_probe(&i2c1, 0x48);
    hal_status_t probe3 = core_i2c_probe(&i2c3, 0x68);
    (void)probe1;
    (void)probe3;

    /* ADC on pad 8 with DMA */
    core_adc_t adc;
    core_adc_init(&adc, ADC_12BIT);
    core_adc_add(&adc, 8, SAMP_MED);

    /* Start continuous DMA -- core_adc_start_dma(adc, buf, len, cb, ctx) */
    static uint16_t dma_buf[16];
    core_adc_start_dma(&adc, dma_buf, 16, dma_cb, NULL);

    while (1) {
        LED_TOGGLE();

        /* Read latest DMA result */
        uint16_t latest = core_adc_dma_read(&adc, 8);
        (void)latest;

        core_delay_ms(250);
    }
}
