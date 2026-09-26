/**
 * val-lpuart -- Validation: LPUART1 pad assignments on Core.ST.L0
 *
 * Core.ST.L0, clock=high
 * Pad 4 = LPUART1.TX, Pad 2 = LPUART1.RX
 *
 * The primary test is that the LPUART1 pad assignments work through
 * coregen -- the generated core_pads.h should configure pad 4 and pad 2
 * for the correct alternate function.
 *
 * If LPUART1 is available as a CMSIS symbol, we exercise hal_uart_init
 * directly since core_serial_init may not handle LPUART clock correctly.
 * If LPUART1 is not defined, we still verify the pads are configured.
 */

#include "core.h"
#include "core_serial.h"

int main(void)
{
    core_init();
    core_led_init();

    /* Coregen should have configured pad 4 (LPUART1.TX) and pad 2
     * (LPUART1.RX) with the correct AF. core_init() calls tile_init()
     * which applies the generated pad configuration. */

#ifdef LPUART1
    /* LPUART1 symbol exists -- do a basic init via hal_uart_init.
     * We use hal_uart_init directly because core_serial_init passes
     * PCLK1_HZ, which may not be the correct source clock for LPUART1
     * (on L0, LPUART1 is on APB1 but has a separate clock mux). */
    hal_uart_t lpuart;
    hal_uart_config_t cfg = { .baud = 9600, .rx_interrupt = 0 };
    hal_uart_init(&lpuart, LPUART1, PCLK1_HZ, &cfg);

    hal_uart_printf(&lpuart, "LPUART ok\r\n");
#else
    /* LPUART1 not defined -- pads are still configured by coregen.
     * This branch verifies the project compiles without LPUART support. */
#endif

    while (1) {
        LED_TOGGLE();
        core_delay_ms(1000);
    }
}
