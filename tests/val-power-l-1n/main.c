/**
 * val-power-l-1n — Compile-only validation for the Power.L.1N tile driver.
 *
 * Exercises every public API function in tile_power_l_1n.h to verify
 * compilation. Does not require hardware — all results are cast to void.
 *
 * Core.ST.L4, clock=max, I2C1 at 400 kHz
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_power_l_1n.h"

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_t pmic;

    /* ---- Lifecycle ---- */
    uint8_t found = tile_power_l_1n_find(hal, 0);
    (void)found;

    tile_power_l_1n_init(hal, 0, &pmic, NULL);

    /* Explicit config, every field (vbus_limit_auto last) */
    power_l_1n_cfg_t cfg = {
        .charge_current_ma = 200,
        .term_mv           = 4200,
        .enable_charging   = 1,
        .vbus_limit_ma     = 500,
        .vbus_limit_auto   = 1,
    };
    tile_power_l_1n_init(hal, 0, &pmic, &cfg);

    /* ---- Charger ---- */
    tile_power_l_1n_charger_enable(&pmic, 0);
    tile_power_l_1n_charger_enable(&pmic, 1);
    tile_power_l_1n_set_vbus_limit_ma(&pmic, 100);
    tile_power_l_1n_set_vbus_limit_ma(&pmic, 1500);
    tile_power_l_1n_set_charge_current_ma(&pmic, 100);
    tile_power_l_1n_set_term_mv(&pmic, 4200);
    tile_power_l_1n_set_term_mv(&pmic, 3600);  /* LiFePO4 */
    uint8_t status   = tile_power_l_1n_get_charge_status(&pmic);
    uint8_t err      = tile_power_l_1n_get_charge_error(&pmic);
    uint8_t charging = tile_power_l_1n_is_charging(&pmic);
    uint8_t complete = tile_power_l_1n_is_charge_complete(&pmic);
    uint8_t battery  = tile_power_l_1n_battery_present(&pmic);
    (void)status; (void)err; (void)charging; (void)complete; (void)battery;

    /* ---- USB-C source detection (CC1 / CC2) ---- */
    power_l_1n_usbc_t src = tile_power_l_1n_get_usbc_source(&pmic);
    uint16_t src_ma = tile_power_l_1n_get_usbc_current_ma(&pmic);
    (void)src; (void)src_ma;
    tile_power_l_1n_set_vbus_limit_auto(&pmic, 1);
    uint16_t ilim = tile_power_l_1n_update_vbus_limit(&pmic);
    (void)ilim;
    tile_power_l_1n_set_vbus_limit_auto(&pmic, 0);

    /* ---- Measurements ---- */
    uint16_t vbat = tile_power_l_1n_get_vbat_mv(&pmic);
    uint16_t vsys = tile_power_l_1n_get_vsys_mv(&pmic);
    int16_t  temp = tile_power_l_1n_get_die_temp_c(&pmic);
    (void)vbat; (void)vsys; (void)temp;

    /* ---- Buck regulators ---- */
    tile_power_l_1n_buck_enable(&pmic, 1, 1);
    tile_power_l_1n_buck_enable(&pmic, 2, 1);
    tile_power_l_1n_buck_set_mv(&pmic, 1, 1800);
    tile_power_l_1n_buck_set_mv(&pmic, 2, 3300);

    /* ---- Indicator LEDs ---- */
    tile_power_l_1n_led_set_mode(&pmic, 2, NPM1300_LED_HOST);
    tile_power_l_1n_led_set(&pmic, 2, 1);
    tile_power_l_1n_led_set(&pmic, 2, 0);

    /* ---- Misc ---- */
    uint8_t rst = tile_power_l_1n_get_reset_cause(&pmic);
    (void)rst;

    while (1) {
        core_delay_ms(1000);
    }
}
