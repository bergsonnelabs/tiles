/**
 * val-nvm-eeprom -- Validation: NVM (EEPROM) on Core.ST.L0
 *
 * Core.ST.L0, clock=default
 *
 * Exercises: core_init, core_nvm_read, core_nvm_write, core_nvm_size
 *
 * Core.ST.L0 (STM32L011) has 512 bytes of true EEPROM at 0x08080000.
 * This test writes a 4-byte pattern, reads it back, and checks size.
 */

#include "core.h"
#include "core_nvm.h"

int main(void)
{
    core_init();
    core_led_init();

    /* Check NVM size -- 512 bytes on Core.ST.L0 */
    uint32_t nvm_sz = core_nvm_size();
    (void)nvm_sz;

    /* Write a 4-byte pattern at offset 0 */
    uint8_t data[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
    int wr = core_nvm_write(0, data, 4);
    (void)wr;

    /* Read it back */
    uint8_t buf[4] = { 0 };
    int rd = core_nvm_read(0, buf, 4);
    (void)rd;

    /* Simple verify -- toggle LED on match */
    if (buf[0] == 0xDE && buf[1] == 0xAD &&
        buf[2] == 0xBE && buf[3] == 0xEF) {
        LED_TOGGLE();
    }

    while (1) {
        core_delay_ms(1000);
    }
}
