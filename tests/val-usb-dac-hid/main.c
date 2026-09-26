/**
 * val-usb-dac-hid -- Validation: USB CDC + HID, DAC output, ADC read
 *
 * Core.ST.H5, clock=default, USB enabled
 * Pad 9 = DAC1.OUT, Pad 8 = ADC7+
 *
 * Exercises: core_init, core_usb_init, core_usb_printf,
 *            core_usb_hid_send, core_dac_init, core_dac_write,
 *            core_dac_write_mv, core_dac_read,
 *            core_adc_t, core_adc_init, core_adc_read
 *
 * Note: core_dac.h has #error if not STM32H523xx -- this is Core.ST.H5, so OK.
 *       core_usb.h has #error if not STM32L422xx or STM32H523xx -- also OK.
 */

#include "core.h"
#include "core_usb.h"
#include "core_usb_hid.h"
#include "core_dac.h"
#include "core_adc.h"

int main(void)
{
    core_init();
    core_usb_init();

    /* DAC on pad 9 */
    core_dac_init();
    core_dac_write(2048);          /* Raw 12-bit mid-scale */
    core_dac_write_mv(1650);       /* 1.65V */
    uint16_t dac_val = core_dac_read();
    (void)dac_val;

    /* ADC on pad 8 */
    core_adc_t adc;
    core_adc_init(&adc, ADC_12BIT);
    core_adc_add(&adc, 8, SAMP_MED);

    while (1) {
        if (core_usb_connected()) {
            uint16_t raw = core_adc_read(&adc, 8);
            core_usb_printf("ADC=%u DAC=%u\r\n", raw, core_dac_read());

            /* HID report -- 8-byte payload */
            uint8_t report[8] = { 0x01, 0x02, 0x03, 0x04,
                                  0x05, 0x06, 0x07, 0x08 };
            core_usb_hid_send(report, sizeof(report));
        }
        core_delay_ms(500);
    }
}
