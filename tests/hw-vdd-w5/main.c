/**
 * hw-vdd-w5: bench check for the WBA55 ADC4 VREFINT / temperature fix.
 *
 * What it proves:
 *   - the factory calibration words (RM0493 Tables 144 / 145, 0x0BF907xx)
 *     are readable from the application: if they were not, this would fault
 *     before `stage` reaches 3;
 *   - VDD through VREFINT (ADC4 VIN[0]) tracks the real supply;
 *   - the die temperature (VIN[13]) is plausible;
 *   - the status-LED blip is stretched for the supply (core_led).
 *
 * Flash over SWD (CoreProbe), then read `g_vdd_probe` without resetting:
 *   make
 *   probe-rs download --chip STM32WBA55HG build/hw-vdd-w5.elf && probe-rs reset --chip STM32WBA55HG
 *   arm-none-eabi-nm build/hw-vdd-w5.elf | grep g_vdd_probe      # its address
 *   probe-rs read --chip STM32WBA55HG b32 <address> 8
 * Words: magic 0x56444431, stage (3 = done), VREFINT_CAL, TS_CAL1, TS_CAL2,
 * VDD mV, temperature in 0.1 °C, the 2 ms blip as scaled (ms).
 * Expect VDD within ~50 mV of a meter on V+, a room-ish temperature, and at
 * 3.3 V a 2 ms blip; power it lower (e.g. 2.0 V) and the blip grows.
 *
 * First run, 2026-09-25 (Core.ST.W5 on its 1.8 V rail, probe 5 V in):
 *   stage 3, VREFINT_CAL 1660, TS_CAL1 1017, TS_CAL2 1348, VDD 1806 mV
 *   (probe sensed 1800), 23.1 °C, blip 30 ms.
 */
#include "core.h"
#include "hal_adc.h"

typedef struct {
    uint32_t magic;
    uint32_t stage;
    uint32_t vrefint_cal;
    uint32_t ts_cal1;
    uint32_t ts_cal2;
    uint32_t vdd_mv;
    int32_t  temp_decidegc;
    uint32_t led_blip_ms;
} vdd_probe_t;

volatile vdd_probe_t g_vdd_probe;

static uint32_t cal_word(uintptr_t addr)
{
    volatile const uint8_t *b = (volatile const uint8_t *)addr;
    return (uint32_t)(b[0] | ((uint32_t)b[1] << 8));
}

int main(void)
{
    core_init();
    core_led_init();
    g_vdd_probe.magic = 0x56444431u;           /* "VDD1" */

    g_vdd_probe.stage = 1;                      /* about to touch the cal block */
    g_vdd_probe.vrefint_cal = cal_word(0x0BF907A5u);
    g_vdd_probe.ts_cal1     = cal_word(0x0BF90710u);
    g_vdd_probe.ts_cal2     = cal_word(0x0BF90742u);

    g_vdd_probe.stage = 2;                      /* cal readable; now the ADC */
    hal_adc_t adc;
    if (hal_adc_init(&adc, ADC4, SYSCLK_HZ, HAL_ADC_RES_12BIT) == HAL_OK) {
        g_vdd_probe.vdd_mv        = hal_adc_read_vdda_mv(&adc);
        g_vdd_probe.temp_decidegc = hal_adc_read_temp_decidegc(&adc);
        hal_adc_deinit(&adc);
    }
    g_vdd_probe.led_blip_ms = core_led_scaled_on_ms(2);
    g_vdd_probe.stage = 3;

    core_led_heartbeat(1000, 2);                /* 1 Hz, 2 ms at 3.3 V */
    while (1) {
        ll_delay_ms(1000);
    }
}
