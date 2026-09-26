/**
 * val-adc-uart -- Validation: ADC reads + UART serial output
 *
 * Core.ST.L0, clock=high
 * Pad 6 = ADC1, Pad 7 = USART2.TX, Pad 2 = USART2.RX
 *
 * Exercises: core_init, hal_uart_t, core_serial_init (no pclk_hz),
 *            core_adc_t, core_adc_init, core_adc_read, core_adc_read_mv,
 *            core_adc_temp, core_serial_printf
 */

#include "core.h"
#include "core_serial.h"
#include "core_adc.h"

int main(void)
{
    core_init();

    /* UART init -- no pclk_hz parameter (auto-resolved) */
    hal_uart_t uart;
    hal_uart_config_t uart_cfg = { .baud = 115200, .rx_interrupt = 0 };
    core_serial_init(&uart, USART2, &uart_cfg);

    /* ADC init at 12-bit resolution */
    core_adc_t adc;
    core_adc_init(&adc, ADC_12BIT);
    core_adc_add(&adc, 6, SAMP_MED);

    while (1) {
        uint16_t raw = core_adc_read(&adc, 6);
        uint32_t mv  = core_adc_read_mv(&adc, 6);
        int32_t  temp = core_adc_temp(&adc);

        core_serial_printf(&uart, "ADC raw=%u mv=%lu temp=%ld\r\n",
                           raw, mv, temp);

        core_delay_ms(500);
    }
}
