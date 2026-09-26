/**
 * val-power-l-1t — Compile-only validation for the Power.L.1T tile driver.
 *
 * Exercises every public API function in tile_power_l_1t.h to verify
 * compilation. Does not require hardware — all results are cast to void.
 *
 * Core.ST.L4, clock=max, I2C1 at 400 kHz
 */

#include "core.h"
#include "core_tiles.h"
#include "tile_power_l_1t.h"

int main(void)
{
    core_init();

    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_t battery;

    /* ---- Lifecycle ---- */
    uint8_t found = tile_power_l_1t_find(hal, 0);
    (void)found;

    tile_power_l_1t_init(hal, 0, &battery, NULL);

    /* ---- Charge configuration ---- */
    tile_power_l_1t_set_charge_current_ma(&battery, 100);
    tile_power_l_1t_set_charge_current_ma(&battery, 400);  /* triggers range 1 */
    tile_power_l_1t_set_charge_voltage_mv(&battery, 4200);
    tile_power_l_1t_set_charge_voltage_mv(&battery, 3650); /* LFP */
    tile_power_l_1t_set_pre_charge_ma(&battery, 20);
    tile_power_l_1t_set_termination_percent(&battery, 10);
    tile_power_l_1t_set_termination_percent(&battery, 0);  /* disable */
    tile_power_l_1t_set_input_current_limit_ma(&battery, 250);

    /* ---- ADC reads ---- */
    uint16_t vbat   = tile_power_l_1t_get_vbat_mv(&battery);
    uint16_t vin    = tile_power_l_1t_get_vin_mv(&battery);
    uint16_t pmid   = tile_power_l_1t_get_pmid_mv(&battery);
    uint16_t ichg   = tile_power_l_1t_get_charge_current_ma(&battery);
    uint16_t iin    = tile_power_l_1t_get_input_current_ma(&battery);
    uint16_t ts     = tile_power_l_1t_get_ts_mv(&battery);
    uint16_t adcin  = tile_power_l_1t_get_adcin_mv(&battery);
    uint8_t  pct    = tile_power_l_1t_get_percent(&battery);
    (void)vbat; (void)vin; (void)pmid; (void)ichg;
    (void)iin; (void)ts; (void)adcin; (void)pct;

    /* ---- NTC thresholds (raw codes from datasheet §8.5.1.49–52) ---- */
    tile_power_l_1t_set_ts_cold(&battery, 0x7C);  /* default */
    tile_power_l_1t_set_ts_cool(&battery, 0x44);  /* example */
    tile_power_l_1t_set_ts_warm(&battery, 0x24);
    tile_power_l_1t_set_ts_hot(&battery, 0x10);
    tile_power_l_1t_set_ts_enabled(&battery, 1);
    tile_power_l_1t_set_ts_enabled(&battery, 0);

    /* ---- LDO output ---- */
    tile_power_l_1t_set_ldo_voltage_mv(&battery, 1800);
    tile_power_l_1t_set_ldo_voltage_mv(&battery, 3300);
    tile_power_l_1t_set_ldo_mode(&battery, POWER_L_1T_LDO_MODE_LDO);
    tile_power_l_1t_set_ldo_mode(&battery, POWER_L_1T_LDO_MODE_LOAD_SWITCH);
    tile_power_l_1t_set_ldo_enabled(&battery, 0);
    tile_power_l_1t_set_ldo_enabled(&battery, 1);

    /* ---- Status / fault snapshot ---- */
    power_l_1t_status_t status;
    tile_power_l_1t_get_charge_status(&battery, &status);
    (void)status.vin_pgood;
    (void)status.charging;
    (void)status.cv_mode;
    (void)status.charge_done;
    (void)status.iinlim_active;
    (void)status.vindpm_active;
    (void)status.thermreg_active;
    (void)status.vin_ovp;
    (void)status.bat_ocp;
    (void)status.bat_uvlo;
    (void)status.safety_timer;
    (void)status.watchdog;
    (void)status.ts_cold;
    (void)status.ts_cool;
    (void)status.ts_warm;
    (void)status.ts_hot;

    /* ---- Raw register access (escape hatches) ---- */
    uint8_t devid = tile_power_l_1t_read_status(&battery, BQ25150_REG_DEVICE_ID);
    (void)devid;
    tile_power_l_1t_write_reg(&battery, BQ25150_REG_VBAT_CTRL, 0x3C);

    /* ---- State checks ---- */
    uint8_t ready = tile_is_ready(&battery);
    (void)ready;
    tile_state_t state = tile_state(&battery);
    (void)state;

    /* Skip enter_ship_mode in val test — would disconnect the chip
     * for the rest of the run. Keep it referenced for compile check. */
    (void)tile_power_l_1t_enter_ship_mode;

    /* ---- v3.1 tier-2 runtime helpers ---- */
    uint8_t charging  = tile_power_l_1t_is_charging(&battery);
    uint8_t done      = tile_power_l_1t_is_charge_done(&battery);
    uint8_t low       = tile_power_l_1t_is_battery_low(&battery, 10);
    uint8_t powered   = tile_power_l_1t_is_powered(&battery);
    uint8_t wait_done = tile_power_l_1t_wait_for_charge_done(&battery, 5);
    (void)charging; (void)done; (void)low; (void)powered; (void)wait_done;

    /* ---- v3.2 capability additions ---- */
    tile_power_l_1t_charger_enable(&battery, 0);
    tile_power_l_1t_charger_enable(&battery, 1);
    tile_power_l_1t_set_battery_uvlo_mv(&battery, 3000);
    tile_power_l_1t_set_battery_uvlo_mv(&battery, 2400);
    tile_power_l_1t_set_safety_timer(&battery, POWER_L_1T_SAFETY_6H);
    tile_power_l_1t_set_safety_timer(&battery, POWER_L_1T_SAFETY_OFF);
    tile_power_l_1t_set_pmid_mode(&battery, POWER_L_1T_PMID_AUTO);
    tile_power_l_1t_set_pmid_mode(&battery, POWER_L_1T_PMID_BAT_ONLY);
    tile_power_l_1t_set_adc_comparator(&battery, 1, POWER_L_1T_ADC_CH_VBAT, 0x8000);
    tile_power_l_1t_set_adc_comparator(&battery, 2, POWER_L_1T_ADC_CH_VIN, 0xC000);
    tile_power_l_1t_set_adc_comparator(&battery, 3, POWER_L_1T_ADC_CH_DISABLED, 0);
    uint8_t alarms = tile_power_l_1t_get_adc_comparators(&battery);
    (void)alarms;

    while (1) {
        core_delay_ms(1000);
    }
}
