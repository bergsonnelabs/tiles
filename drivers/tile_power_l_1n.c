/**
 * @file   tile_power_l_1n.c
 * @brief  Power.L.1N tile driver implementation — Nordic nPM1300 PMIC.
 */

#include "tile_power_l_1n.h"
#include <stddef.h>

/* -------------------------------------------------------------- */
/* Instance mapping                                                */
/* -------------------------------------------------------------- */

static const uint8_t id_table[] = {
    NPM1300_I2C_ADDR,   /* instance 0 — fixed address (0x6B) */
};

#define ID_TABLE_LEN  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    if (instance >= ID_TABLE_LEN) return 0x00;
    return id_table[instance];
}

/* -------------------------------------------------------------- */
/* Private helpers                                                 */
/* -------------------------------------------------------------- */

/* The nPM1300 uses 16-bit register addresses (base << 8 | offset). The HAL
 * sends two address bytes MSB-first whenever reg > 0xFF, which every register
 * here is, so the addresses below go on the wire verbatim. */
static void pmic_write(tile_t* tile, uint16_t reg, uint8_t value)
{
    tile->hal->i2c_write(tile->hal->handle, tile->id, reg, &value, 1);
}

static uint8_t pmic_read(tile_t* tile, uint16_t reg)
{
    uint8_t val = 0;
    tile->hal->i2c_read(tile->hal->handle, tile->id, reg, &val, 1);
    return val;
}

/* Trigger a single-shot ADC conversion and read back the 10-bit result.
 * Each channel's high 8 bits live in its own MSB register; the low 2 bits are
 * packed into the shared ADCGP0RESULTLSBS register at the given bit offset. */
static uint16_t adc_read10(tile_t* tile, uint16_t task_reg, uint16_t msb_reg,
                           uint8_t lsb_shift)
{
    pmic_write(tile, task_reg, 1);
    tile->hal->delay_ms(2);          /* conversion completes in ~250 us */
    uint8_t msb  = pmic_read(tile, msb_reg);
    uint8_t lsbs = pmic_read(tile, NPM1300_REG_ADCGP0RESULTLSBS);
    uint8_t lo   = (uint8_t)((lsbs >> lsb_shift) & 0x03);
    return (uint16_t)(((uint16_t)msb << 2) | lo);
}

/* Charge-termination voltage → BCHGVTERM code. The nPM1300 supports two
 * linear bands (datasheet "Charger" §): 3.50-3.65 V (codes 0-3) and
 * 4.00-4.45 V (codes 4-13), both in 50 mV steps; 3.70-3.95 V is not
 * selectable, so requests in the gap clamp down to 3.65 V. */
static uint8_t vterm_code(uint16_t mv)
{
    if (mv < 3500) mv = 3500;
    if (mv <= 3650) return (uint8_t)((mv - 3500) / 50);   /* codes 0-3   */
    if (mv < 4000)  return 3;                              /* gap → 3.65V */
    if (mv > 4450)  mv = 4450;
    return (uint8_t)(4 + (mv - 4000) / 50);               /* codes 4-13  */
}

/* -------------------------------------------------------------- */
/* Public API — discovery + init                                  */
/* -------------------------------------------------------------- */

uint8_t tile_power_l_1n_find(tiles_pal_t* hal, uint8_t instance)
{
    uint8_t id = resolve_id(instance);
    if (id == 0x00) return 0;
    return (hal->i2c_is_ready(hal->handle, id) == 0) ? 1 : 0;
}

void tile_power_l_1n_init(tiles_pal_t* hal, uint8_t instance, tile_t* tile,
                          const power_l_1n_cfg_t* cfg)
{
    tile->hal      = NULL;
    tile->id       = 0;
    tile->state    = TILE_STATE_NONE;
    tile->flags    = 0;
    tile->callback = NULL;
    tile->cb_ctx   = NULL;

    uint8_t id = resolve_id(instance);
    if (id == 0x00) {
        TILE_ON_ERROR(tile, "init: invalid instance");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    tile->hal = hal;
    tile->id  = id;

    if (hal->i2c_is_ready(hal->handle, id) != 0) {
        TILE_ON_ERROR(tile, "init: device not found on bus");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    /* Indicator LEDs (physical colours per the tile schematic):
     *   LED0 green  → HOST    (firmware-driven status)
     *   LED1 yellow → CHARGING (auto: on while charging)
     *   LED2 red    → ERROR    (auto: on for charger faults) */
    pmic_write(tile, NPM1300_REG_LEDDRV0MODESEL, NPM1300_LED_HOST);
    pmic_write(tile, NPM1300_REG_LEDDRV1MODESEL, NPM1300_LED_CHARGING);
    pmic_write(tile, NPM1300_REG_LEDDRV2MODESEL, NPM1300_LED_ERROR);

    /* No NTC thermistor is fitted on this tile (NTC pin tied to GND), so
     * disable NTC monitoring — otherwise the charger would fault on an
     * out-of-range thermistor reading. */
    pmic_write(tile, NPM1300_REG_ADCNTCRSEL, 0x00);

    /* Charger configuration — defaults: 100 mA, 4.20 V, charging enabled. */
    uint16_t charge_ma = 100;
    uint16_t term_mv   = 4200;
    uint8_t  enable    = 1;
    if (cfg != NULL) {
        if (cfg->charge_current_ma != 0) charge_ma = cfg->charge_current_ma;
        if (cfg->term_mv != 0)           term_mv   = cfg->term_mv;
        enable = cfg->enable_charging ? 1 : 0;
    }

    tile_power_l_1n_set_charge_current_ma(tile, charge_ma);
    tile_power_l_1n_set_term_mv(tile, term_mv);
    tile_power_l_1n_charger_enable(tile, enable);

    tile->state = TILE_STATE_READY;
}

/* -------------------------------------------------------------- */
/* Charger                                                        */
/* -------------------------------------------------------------- */

void tile_power_l_1n_charger_enable(tile_t* tile, uint8_t on)
{
    /* ENABLE/CLR are write-1-to-action registers (bit0 = ENABLECHARGING). */
    if (on) pmic_write(tile, NPM1300_REG_BCHGENABLESET, 0x01);
    else    pmic_write(tile, NPM1300_REG_BCHGENABLECLR, 0x01);
}

void tile_power_l_1n_set_vbus_limit_ma(tile_t* tile, uint16_t ma)
{
    /* VBUSINILIM0 holds the limit as a 4-bit code = mA / 100, 1 (100 mA) to
     * 15 (1500 mA); code 0 is a second spelling of 500 mA, never written here.
     * Integer division rounds DOWN, so the limit is never more than asked.
     * The new value only takes effect when TASKUPDATEILIMSW is written, and
     * the part falls back to its start-up limit (100 mA) when VBUS is removed
     * (datasheet 6.1.1, 6.1.8). */
    if (ma < 100)  ma = 100;
    if (ma > 1500) ma = 1500;
    pmic_write(tile, NPM1300_REG_VBUSINILIM0, (uint8_t)(ma / 100));
    pmic_write(tile, NPM1300_REG_TASKUPDATEILIMSW, 0x01);
}

void tile_power_l_1n_set_charge_current_ma(tile_t* tile, uint16_t ma)
{
    /* nPM1300 charge current is programmable 32-800 mA in 2 mA steps.
     * The 10-bit setting splits across two registers: ISETMSB holds the high
     * 8 bits, ISETLSB the low bit (matches Nordic's npmx: idx = mA/2,
     * MSB = idx>>1, LSB = idx&1 → 4 mA per MSB count, 2 mA via the LSB).
     * Example: 100 mA → idx 50 → MSB 25, LSB 0. */
    if (ma < 32)  ma = 32;
    if (ma > 800) ma = 800;
    uint16_t idx = (uint16_t)(ma / 2);
    pmic_write(tile, NPM1300_REG_BCHGISETMSB, (uint8_t)(idx >> 1));
    pmic_write(tile, NPM1300_REG_BCHGISETLSB, (uint8_t)(idx & 0x01));
}

void tile_power_l_1n_set_term_mv(tile_t* tile, uint16_t mv)
{
    uint8_t code = vterm_code(mv);
    pmic_write(tile, NPM1300_REG_BCHGVTERM,  code);
    /* VTERMR (warm-region termination) tracks VTERM so the charge target is
     * consistent regardless of the JEITA region the charger believes it's in. */
    pmic_write(tile, NPM1300_REG_BCHGVTERMR, code);
}

uint8_t tile_power_l_1n_get_charge_status(tile_t* tile)
{
    return pmic_read(tile, NPM1300_REG_BCHGCHARGESTATUS);
}

uint8_t tile_power_l_1n_get_charge_error(tile_t* tile)
{
    return pmic_read(tile, NPM1300_REG_BCHGERRREASON);
}

uint8_t tile_power_l_1n_is_charging(tile_t* tile)
{
    uint8_t s = pmic_read(tile, NPM1300_REG_BCHGCHARGESTATUS);
    return (s & (NPM1300_CHG_TRICKLECHARGE |
                 NPM1300_CHG_CONSTANTCURRENT |
                 NPM1300_CHG_CONSTANTVOLTAGE)) ? 1 : 0;
}

uint8_t tile_power_l_1n_is_charge_complete(tile_t* tile)
{
    return (pmic_read(tile, NPM1300_REG_BCHGCHARGESTATUS)
            & NPM1300_CHG_COMPLETED) ? 1 : 0;
}

uint8_t tile_power_l_1n_battery_present(tile_t* tile)
{
    return (pmic_read(tile, NPM1300_REG_BCHGCHARGESTATUS)
            & NPM1300_CHG_BATTERYDETECTED) ? 1 : 0;
}

/* -------------------------------------------------------------- */
/* Measurements (ADC)                                            */
/* -------------------------------------------------------------- */

uint16_t tile_power_l_1n_get_vbat_mv(tile_t* tile)
{
    /* VBAT ADC full scale = 5.0 V over 10 bits (1023). */
    uint16_t raw = adc_read10(tile, NPM1300_REG_TASKVBATMEASURE,
                              NPM1300_REG_ADCVBATRESULTMSB, 0);
    return (uint16_t)((uint32_t)raw * 5000u / 1023u);
}

uint16_t tile_power_l_1n_get_vsys_mv(tile_t* tile)
{
    /* VSYS ADC full scale = 6.375 V over 10 bits (1023). */
    uint16_t raw = adc_read10(tile, NPM1300_REG_TASKVSYSMEASURE,
                              NPM1300_REG_ADCVSYSRESULTMSB, 6);
    return (uint16_t)((uint32_t)raw * 6375u / 1023u);
}

int16_t tile_power_l_1n_get_die_temp_c(tile_t* tile)
{
    /* Datasheet die-temperature transfer: T(°C) = 394.67 − 0.7926 × code.
     * Integer form: T = (394670 − 793 × code) / 1000. */
    uint16_t raw = adc_read10(tile, NPM1300_REG_TASKTEMPMEASURE,
                              NPM1300_REG_ADCTEMPRESULTMSB, 4);
    int32_t t = (394670 - 793 * (int32_t)raw) / 1000;
    return (int16_t)t;
}

/* -------------------------------------------------------------- */
/* Buck regulators                                               */
/* -------------------------------------------------------------- */

void tile_power_l_1n_buck_enable(tile_t* tile, uint8_t buck, uint8_t on)
{
    uint16_t reg;
    if (buck == 1)      reg = on ? NPM1300_REG_BUCK1ENASET : NPM1300_REG_BUCK1ENACLR;
    else if (buck == 2) reg = on ? NPM1300_REG_BUCK2ENASET : NPM1300_REG_BUCK2ENACLR;
    else return;
    pmic_write(tile, reg, 0x01);
}

void tile_power_l_1n_buck_set_mv(tile_t* tile, uint8_t buck, uint16_t mv)
{
    /* Output range 1.0-3.3 V in 100 mV steps: code = (mv − 1000) / 100. */
    if (buck != 1 && buck != 2) return;
    if (mv < 1000) mv = 1000;
    if (mv > 3300) mv = 3300;
    uint8_t code = (uint8_t)((mv - 1000) / 100);

    pmic_write(tile, buck == 1 ? NPM1300_REG_BUCK1NORMVOUT
                               : NPM1300_REG_BUCK2NORMVOUT, code);

    /* The NORMVOUT register is only honoured when software control is
     * selected; otherwise the buck follows its VSETx resistor. Set the
     * per-buck SW-control bit so the new voltage takes effect. */
    uint8_t sel = pmic_read(tile, NPM1300_REG_BUCKSWCTRLSEL);
    sel |= (buck == 1) ? 0x01 : 0x02;
    pmic_write(tile, NPM1300_REG_BUCKSWCTRLSEL, sel);
}

/* -------------------------------------------------------------- */
/* Indicator LEDs                                                */
/* -------------------------------------------------------------- */

void tile_power_l_1n_led_set_mode(tile_t* tile, uint8_t led,
                                  power_l_1n_led_mode_t mode)
{
    uint16_t reg;
    switch (led) {
    case 0:  reg = NPM1300_REG_LEDDRV0MODESEL; break;
    case 1:  reg = NPM1300_REG_LEDDRV1MODESEL; break;
    case 2:  reg = NPM1300_REG_LEDDRV2MODESEL; break;
    default: return;
    }
    pmic_write(tile, reg, (uint8_t)mode);
}

void tile_power_l_1n_led_set(tile_t* tile, uint8_t led, uint8_t on)
{
    /* Only effective when the LED is in HOST mode; SET/CLR are
     * write-1-to-action. */
    uint16_t reg;
    switch (led) {
    case 0:  reg = on ? NPM1300_REG_LEDDRV0SET : NPM1300_REG_LEDDRV0CLR; break;
    case 1:  reg = on ? NPM1300_REG_LEDDRV1SET : NPM1300_REG_LEDDRV1CLR; break;
    case 2:  reg = on ? NPM1300_REG_LEDDRV2SET : NPM1300_REG_LEDDRV2CLR; break;
    default: return;
    }
    pmic_write(tile, reg, 0x01);
}

/* -------------------------------------------------------------- */
/* Misc                                                          */
/* -------------------------------------------------------------- */

uint8_t tile_power_l_1n_get_reset_cause(tile_t* tile)
{
    return pmic_read(tile, NPM1300_REG_RSTCAUSE);
}
