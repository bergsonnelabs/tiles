/**
 * @file   tile_power_l_1t.c
 * @brief  Li-Ion charge controller implementation (BQ25150).
 */

#include "tile_power_l_1t.h"
#include <stddef.h>

/* -------------------------------------------------------------- */
/* Instance mapping                                                */
/* -------------------------------------------------------------- */

static const uint8_t id_table[] = {
    BQ25150_I2C_ADDR_DEFAULT,   /* instance 0 — fixed address (0x6B) */
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

static void bq_write(tile_t* tile, uint8_t reg, uint8_t value)
{
    tile->hal->i2c_write(tile->hal->handle, tile->id, reg, &value, 1);
}

static uint8_t bq_read(tile_t* tile, uint8_t reg)
{
    uint8_t value = 0;
    tile->hal->i2c_read(tile->hal->handle, tile->id, reg, &value, 1);
    return value;
}

/** Read a 16-bit ADC pair (MSB:LSB) as a single burst. The MSB
 *  register is at `msb_reg`, LSB at `msb_reg + 1`. Returns 0 on
 *  bus error. Bursting avoids torn samples between the two halves
 *  of a single conversion. */
static uint16_t adc_read16(tile_t* tile, uint8_t msb_reg)
{
    uint8_t buf[2] = { 0, 0 };
    if (tile->hal->i2c_read(tile->hal->handle, tile->id,
                            msb_reg, buf, 2) != 0) {
        return 0;
    }
    return ((uint16_t)buf[0] << 8) | buf[1];
}

/** Scale a 16-bit ADC reading to milli-units against a full-scale
 *  range. ADC formulas in datasheet §8.3.3.3:
 *    VIN/PMID/VBAT: V = (raw / 2^16) × 6 V → mV = (raw × 6000) >> 16
 *    TS/ADCIN:      V = (raw / 2^16) × 1.2 V → mV = (raw × 1200) >> 16
 *    IIN: I = (raw / 2^16) × 375 mA (≤150 mA range) or × 750 mA (>150 mA) */
static uint16_t adc_scale(uint16_t raw, uint32_t fs_milli)
{
    return (uint16_t)(((uint32_t)raw * fs_milli) >> 16);
}

/* -------------------------------------------------------------- */
/* Public API — discovery + init                                   */
/* -------------------------------------------------------------- */

uint8_t tile_power_l_1t_find(tiles_pal_t* hal, uint8_t instance)
{
    uint8_t id = resolve_id(instance);
    if (id == 0x00) return 0;
    return (hal->i2c_is_ready(hal->handle, id) == 0) ? 1 : 0;
}

void tile_power_l_1t_init(tiles_pal_t* hal, uint8_t instance, tile_t* tile,
                          const power_l_1t_cfg_t *cfg)
{
    (void)cfg;  /* Reserved for future use */
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

    /* Verify device is on bus */
    if (hal->i2c_is_ready(hal->handle, id) != 0) {
        TILE_ON_ERROR(tile, "init: device not found on bus");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    /* Verify device ID */
    uint8_t dev_id = bq_read(tile, BQ25150_REG_DEVICE_ID);
    if (dev_id != BQ25150_DEVICE_ID_DEFAULT) {
        TILE_ON_ERROR(tile, "init: unexpected device ID");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    /* Disable the I²C watchdog. Default state of CHARGERCTRL0 is
     * 0x82 (TS_EN=1, SAFETY_TIMER=01); set bit 4 (WATCHDOG_DISABLE)
     * → 0x92. Without this, the chip silently resets all charge
     * parameters every 50 s if the host doesn't write within that
     * window. The watchdog only matters for hosted-system crash
     * recovery — not a use case Cores cares about. */
    bq_write(tile, BQ25150_REG_CHARGERCTRL0, 0x92);

    /* Fast charge current: 80 mA (range 0, 1.25 mA × 64 = 80 mA).
     * Conservative default suitable for small Li-Ion cells. Users
     * tune via tile_power_l_1t_set_charge_current_ma(). */
    bq_write(tile, BQ25150_REG_PCHRGCTRL, 0x0F);   /* range 0, IPRECHG = 15 → 18.75 mA */
    bq_write(tile, BQ25150_REG_ICHG_CTRL, 0x40);   /* 64 × 1.25 = 80 mA */

    /* Battery regulation (charge-to) voltage: 4.20 V, written explicitly.
     * 0x3C is the chip's reset value, but the BQ25150 is battery-powered and
     * keeps its registers across a host reset or re-flash: without this, a
     * VBATREG raised by an earlier program (up to 4.6 V) would survive into a
     * program that expects the documented 4.20 V default. VBATREG = 3.6 V +
     * code x 10 mV (datasheet 8.5.1.12). */
    bq_write(tile, BQ25150_REG_VBAT_CTRL, 0x3C);

    /* Battery undervoltage lockout: 2.6 V */
    bq_write(tile, BQ25150_REG_BUVLO, 0x04);

    /* Enable all 6 ADC channels (IIN/PMID/ICHG/VIN/VBAT/TS/ADCIN).
     * Bit 0 is RESERVED, leave 0. */
    bq_write(tile, BQ25150_REG_ADC_READ_EN, 0xFE);

    /* Prime the ADC: on battery the auto rate is only 1 Hz, so the first
     * VBAT/VIN read after init would otherwise return a stale register until
     * the first auto tick (up to ~1 s later). Force conversions in Manual mode
     * and wait for ADC_READY (FLAG2 bit7, clear-on-read) each time, then resume
     * the low-power 1 s rate. Two conversions (~50 ms) so the data registers
     * hold a settled post-init sample, not the first mid-startup one. (With VIN
     * present the ADC runs continuously anyway; the manual trigger is harmless
     * there.) */
    for (int conv = 0; conv < 2; conv++) {
        (void)bq_read(tile, BQ25150_REG_FLAG2);  /* clear any stale ADC_READY */
        bq_write(tile, BQ25150_REG_ADCCTRL0,
                 BQ25150_ADCCTRL0_RATE_MANUAL | BQ25150_ADCCTRL0_CONV_START |
                 BQ25150_ADCCTRL0_COMP1_DEFAULT);
        for (int i = 0; i < 50; i++) {  /* ~24 ms conversion */
            if (bq_read(tile, BQ25150_REG_FLAG2) & BQ25150_FLAG2_ADC_READY) break;
            hal->delay_ms(1);
        }
    }
    /* ADC: 1-second update rate in battery mode, 24 ms conv speed. */
    bq_write(tile, BQ25150_REG_ADCCTRL0,
             BQ25150_ADCCTRL0_RATE_1S | BQ25150_ADCCTRL0_COMP1_DEFAULT);

    /* Disable ship mode (clear bit 7 of ICCTRL0). Reset value is
     * 0x10 (AUTOWAKE = 1.2 s); preserving that. */
    bq_write(tile, BQ25150_REG_ICCTRL0, 0x10);

    tile->state = TILE_STATE_READY;
}

/* -------------------------------------------------------------- */
/* Charge configuration                                            */
/* -------------------------------------------------------------- */

void tile_power_l_1t_set_charge_current_ma(tile_t* tile, uint16_t ma)
{
    if (tile->state != TILE_STATE_READY) return;
    if (ma > 500) ma = 500;

    /* ICHG_CTRL is 8 bits, ICHARGE_RANGE is bit 7 of PCHRGCTRL.
     * Range 0: 1.25 mA × code (max 318.75 mA at code 255).
     * Range 1: 2.5 mA × code (max 637.5 mA, clamped here to 500). */
    uint8_t use_range_1 = (ma > 318) ? 1 : 0;
    uint16_t code = use_range_1 ? ((ma * 2u) / 5u)   /* /2.5 */
                                : ((ma * 4u) / 5u); /* /1.25 */
    if (code > 255) code = 255;

    /* Update ICHARGE_RANGE in PCHRGCTRL bit 7, preserving the
     * IPRECHG field below. Re-program ICHG_CTRL for the new code. */
    uint8_t pchrg = bq_read(tile, BQ25150_REG_PCHRGCTRL);
    pchrg = (pchrg & 0x7F) | (use_range_1 ? 0x80 : 0x00);
    bq_write(tile, BQ25150_REG_PCHRGCTRL, pchrg);
    bq_write(tile, BQ25150_REG_ICHG_CTRL, (uint8_t)code);
}

void tile_power_l_1t_set_charge_voltage_mv(tile_t* tile, uint16_t mv)
{
    if (tile->state != TILE_STATE_READY) return;
    if (mv < 3600) mv = 3600;
    if (mv > 4600) mv = 4600;

    /* VBATREG = 3.6 V + code × 10 mV. Bit 7 reserved (keep 0). */
    uint8_t code = (uint8_t)((mv - 3600u) / 10u);
    bq_write(tile, BQ25150_REG_VBAT_CTRL, code & 0x7F);
}

void tile_power_l_1t_set_pre_charge_ma(tile_t* tile, uint8_t ma)
{
    if (tile->state != TILE_STATE_READY) return;

    /* IPRECHG step depends on ICHARGE_RANGE; preserve that bit. */
    uint8_t pchrg = bq_read(tile, BQ25150_REG_PCHRGCTRL);
    uint8_t range_1 = (pchrg & 0x80) ? 1 : 0;
    uint16_t code;
    if (range_1) {
        if (ma > 77) ma = 77;
        code = (ma * 2u) / 5u;        /* /2.5 */
    } else {
        if (ma > 38) ma = 38;
        code = (ma * 4u) / 5u;        /* /1.25 */
    }
    if (code > 31) code = 31;

    /* Preserve bits 7:5 (ICHARGE_RANGE + RESERVED), set bits 4:0. */
    pchrg = (pchrg & 0xE0) | (uint8_t)(code & 0x1F);
    bq_write(tile, BQ25150_REG_PCHRGCTRL, pchrg);
}

void tile_power_l_1t_set_termination_percent(tile_t* tile, uint8_t pct)
{
    if (tile->state != TILE_STATE_READY) return;

    /* Bits 5:1 = ITERM (% of ICHG, 1–31), bit 0 = TERM_DISABLE. */
    uint8_t val;
    if (pct == 0) {
        val = 0x01;                          /* TERM_DISABLE = 1 */
    } else {
        if (pct > 31) pct = 31;
        val = (uint8_t)((pct & 0x1F) << 1);  /* TERM_DISABLE = 0 */
    }
    bq_write(tile, BQ25150_REG_TERMCTRL, val);
}

void tile_power_l_1t_set_input_current_limit_ma(tile_t* tile, uint16_t ma)
{
    if (tile->state != TILE_STATE_READY) return;

    /* ILIMCTRL[2:0] selects from a discrete table (50/100/150/200/
     * 300/400/500/600 mA). Pick the largest level ≤ requested so
     * we never draw more than the caller asked for. */
    uint8_t code;
    if      (ma >= 600) code = 7;
    else if (ma >= 500) code = 6;
    else if (ma >= 400) code = 5;
    else if (ma >= 300) code = 4;
    else if (ma >= 200) code = 3;
    else if (ma >= 150) code = 2;
    else if (ma >= 100) code = 1;
    else                code = 0;            /* 50 mA floor */
    bq_write(tile, BQ25150_REG_ILIMCTRL, code);
}

/* -------------------------------------------------------------- */
/* ADC reads                                                       */
/* -------------------------------------------------------------- */

uint16_t tile_power_l_1t_get_vbat_mv(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) return 0;
    return adc_scale(adc_read16(tile, BQ25150_REG_ADC_DATA_VBAT_M), 6000);
}

uint16_t tile_power_l_1t_get_vin_mv(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) return 0;
    return adc_scale(adc_read16(tile, BQ25150_REG_ADC_DATA_VIN_M), 6000);
}

uint16_t tile_power_l_1t_get_pmid_mv(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) return 0;
    return adc_scale(adc_read16(tile, BQ25150_REG_ADC_DATA_PMID_M), 6000);
}

uint16_t tile_power_l_1t_get_charge_current_ma(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) return 0;

    /* ICHG_RAW reports % of the configured fast-charge current,
     * scaled so that 0.8 × 2^16 = 100 %. Convert to mA by reading
     * ICHG_CTRL + ICHARGE_RANGE to recover the absolute scale.
     *
     *   I_charge = (raw / (0.8 × 2^16)) × 100 % × ICHG_max
     *            = (raw × ICHG_max) / 52429
     */
    uint8_t pchrg = bq_read(tile, BQ25150_REG_PCHRGCTRL);
    uint8_t ichg_code = bq_read(tile, BQ25150_REG_ICHG_CTRL);
    uint16_t ichg_max_ma = (pchrg & 0x80) ? (uint16_t)((ichg_code * 5u) / 2u)
                                          : (uint16_t)((ichg_code * 5u) / 4u);
    uint16_t raw = adc_read16(tile, BQ25150_REG_ADC_DATA_ICHG_M);
    return (uint16_t)(((uint32_t)raw * ichg_max_ma) / 52429u);
}

uint16_t tile_power_l_1t_get_input_current_ma(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) return 0;

    /* IIN scales with ILIMCTRL: ≤150 mA range gives 375 mA full
     * scale, >150 mA range gives 750 mA. ILIM codes 0–2 = ≤150 mA. */
    uint8_t ilim = bq_read(tile, BQ25150_REG_ILIMCTRL) & 0x07;
    uint32_t fs = (ilim <= 2) ? 375u : 750u;
    return adc_scale(adc_read16(tile, BQ25150_REG_ADC_DATA_IIN_M), fs);
}

uint16_t tile_power_l_1t_get_ts_mv(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) return 0;
    return adc_scale(adc_read16(tile, BQ25150_REG_ADC_DATA_TS_M), 1200);
}

uint16_t tile_power_l_1t_get_adcin_mv(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) return 0;
    return adc_scale(adc_read16(tile, BQ25150_REG_ADC_DATA_ADCIN_M), 1200);
}

/* -------------------------------------------------------------- */
/* Battery percent (derived)                                       */
/* -------------------------------------------------------------- */

uint8_t tile_power_l_1t_get_percent(tile_t* tile)
{
    /* Linear curve: 3000 mV = 0 %, 4200 mV = 100 % (1200 mV span).
     * Coarse — a real fuel gauge would integrate Coulombs. */
    uint16_t mv = tile_power_l_1t_get_vbat_mv(tile);
    if (mv <= 3000) return 0;
    if (mv >= 4200) return 100;
    return (uint8_t)(((uint32_t)(mv - 3000u) * 100u) / 1200u);
}

/* -------------------------------------------------------------- */
/* NTC thresholds                                                  */
/* -------------------------------------------------------------- */

void tile_power_l_1t_set_ts_cold(tile_t* tile, uint8_t code)
{
    if (tile->state != TILE_STATE_READY) return;
    bq_write(tile, BQ25150_REG_TS_COLD, code);
}

void tile_power_l_1t_set_ts_cool(tile_t* tile, uint8_t code)
{
    if (tile->state != TILE_STATE_READY) return;
    bq_write(tile, BQ25150_REG_TS_COOL, code);
}

void tile_power_l_1t_set_ts_warm(tile_t* tile, uint8_t code)
{
    if (tile->state != TILE_STATE_READY) return;
    bq_write(tile, BQ25150_REG_TS_WARM, code);
}

void tile_power_l_1t_set_ts_hot(tile_t* tile, uint8_t code)
{
    if (tile->state != TILE_STATE_READY) return;
    bq_write(tile, BQ25150_REG_TS_HOT, code);
}

void tile_power_l_1t_set_ts_enabled(tile_t* tile, uint8_t enabled)
{
    if (tile->state != TILE_STATE_READY) return;

    /* TS_EN is bit 7 of CHARGERCTRL0. Preserve other bits including
     * WATCHDOG_DISABLE (bit 4). */
    uint8_t cur = bq_read(tile, BQ25150_REG_CHARGERCTRL0);
    cur = enabled ? (cur | 0x80) : (cur & 0x7F);
    bq_write(tile, BQ25150_REG_CHARGERCTRL0, cur);
}

/* -------------------------------------------------------------- */
/* LDO output                                                      */
/* -------------------------------------------------------------- */

void tile_power_l_1t_set_ldo_voltage_mv(tile_t* tile, uint16_t mv)
{
    if (tile->state != TILE_STATE_READY) return;
    if (mv < 600)  mv = 600;
    if (mv > 3700) mv = 3700;

    /* VLDO_4:0 in bits 6:2; VLDO = 600 + code × 100 mV. Preserve
     * EN_LS_LDO (bit 7) and LDO_SWITCH_CONFG (bit 1). */
    uint8_t code = (uint8_t)((mv - 600u) / 100u) & 0x1F;
    uint8_t cur = bq_read(tile, BQ25150_REG_LDOCTRL);
    cur = (cur & 0x83) | (uint8_t)(code << 2);
    bq_write(tile, BQ25150_REG_LDOCTRL, cur);
}

void tile_power_l_1t_set_ldo_mode(tile_t* tile, power_l_1t_ldo_mode_t mode)
{
    if (tile->state != TILE_STATE_READY) return;

    /* LDO_SWITCH_CONFG is bit 1; preserve all other bits. */
    uint8_t cur = bq_read(tile, BQ25150_REG_LDOCTRL);
    cur = (mode == POWER_L_1T_LDO_MODE_LOAD_SWITCH)
        ? (cur | 0x02)
        : (cur & ~0x02);
    bq_write(tile, BQ25150_REG_LDOCTRL, cur);
}

void tile_power_l_1t_set_ldo_enabled(tile_t* tile, uint8_t enabled)
{
    if (tile->state != TILE_STATE_READY) return;

    /* EN_LS_LDO is bit 7; preserve all other bits. */
    uint8_t cur = bq_read(tile, BQ25150_REG_LDOCTRL);
    cur = enabled ? (cur | 0x80) : (cur & 0x7F);
    bq_write(tile, BQ25150_REG_LDOCTRL, cur);
}

/* -------------------------------------------------------------- */
/* Status / fault read                                             */
/* -------------------------------------------------------------- */

void tile_power_l_1t_get_charge_status(tile_t* tile,
                                       power_l_1t_status_t *out)
{
    if (!out) return;
    /* Zero the struct first so any short-read leaves consistent state. */
    *(uint16_t *)out = 0;

    if (tile->state != TILE_STATE_READY) return;

    uint8_t s0 = bq_read(tile, BQ25150_REG_STAT0);
    uint8_t s1 = bq_read(tile, BQ25150_REG_STAT1);
    uint8_t f3 = bq_read(tile, BQ25150_REG_FLAG3);  /* clear-on-read */

    /* STAT0: bit 6 CHRG_CV, 5 CHARGE_DONE, 4 IINLIM, 3 VDPPM,
     *        2 VINDPM, 1 THERMREG, 0 VIN_PGOOD */
    out->vin_pgood       = (s0 & 0x01) ? 1 : 0;
    out->thermreg_active = (s0 & 0x02) ? 1 : 0;
    out->vindpm_active   = (s0 & 0x04) ? 1 : 0;
    /* VDPPM = "DPPM"; map to vindpm_active too if you want, here we
     * fold it into vindpm_active to keep the struct compact — both
     * loops indicate the input is being throttled. */
    out->vindpm_active  |= (s0 & 0x08) ? 1 : 0;
    out->iinlim_active   = (s0 & 0x10) ? 1 : 0;
    out->charge_done     = (s0 & 0x20) ? 1 : 0;
    out->cv_mode         = (s0 & 0x40) ? 1 : 0;

    /* Derived: charging when VIN good and not done / not faulted. The
     * BQ25150 has no single "charging" status bit, so the software gates are
     * folded in too: ICCTRL2.CHARGER_DISABLE (bit0) stops charging, and any
     * PMID_MODE other than 00 (ICCTRL1[1:0]) stops it as well (datasheet
     * 8.3.6: BAT_ONLY "charging will be stopped"; FLOAT / PULLDOWN disconnect
     * PMID). Without these, is_charging() said 1 after charger_enable(0). */
    uint8_t icctrl2 = bq_read(tile, BQ25150_REG_ICCTRL2);
    uint8_t icctrl1 = bq_read(tile, BQ25150_REG_ICCTRL1);
    out->charging = (out->vin_pgood && !out->charge_done &&
                     !(icctrl2 & 0x01) && (icctrl1 & 0x03) == 0) ? 1 : 0;

    /* STAT1: bit 7 VIN_OVP, 5 BAT_OCP, 4 BAT_UVLO,
     *        3 TS_COLD, 2 TS_COOL, 1 TS_WARM, 0 TS_HOT */
    out->ts_hot    = (s1 & 0x01) ? 1 : 0;
    out->ts_warm   = (s1 & 0x02) ? 1 : 0;
    out->ts_cool   = (s1 & 0x04) ? 1 : 0;
    out->ts_cold   = (s1 & 0x08) ? 1 : 0;
    out->bat_uvlo  = (s1 & 0x10) ? 1 : 0;
    out->bat_ocp   = (s1 & 0x20) ? 1 : 0;
    out->vin_ovp   = (s1 & 0x80) ? 1 : 0;

    /* If BAT_UVLO or VIN_OVP active, can't actually be charging. */
    if (out->bat_uvlo || out->vin_ovp || out->ts_cold || out->ts_hot) {
        out->charging = 0;
    }

    /* FLAG3: bit 6 WD_FAULT_FLAG, 5 SAFETY_TMR_FAULT_FLAG */
    out->safety_timer = (f3 & 0x20) ? 1 : 0;
    out->watchdog     = (f3 & 0x40) ? 1 : 0;
}

/* -------------------------------------------------------------- */
/* Power management                                                */
/* -------------------------------------------------------------- */

void tile_power_l_1t_enter_ship_mode(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) return;

    /* Set bit 7 EN_SHIP_MODE; preserve other bits. Once set, the
     * chip enters ship mode when VIN is removed. Recovery requires
     * MR press or VIN insertion. */
    uint8_t cur = bq_read(tile, BQ25150_REG_ICCTRL0);
    bq_write(tile, BQ25150_REG_ICCTRL0, cur | 0x80);
}

/* -------------------------------------------------------------- */
/* Raw register access                                             */
/* -------------------------------------------------------------- */

uint8_t tile_power_l_1t_read_status(tile_t* tile, uint8_t reg)
{
    return bq_read(tile, reg);
}

void tile_power_l_1t_write_reg(tile_t* tile, uint8_t reg, uint8_t value)
{
    bq_write(tile, reg, value);
}

/* -------------------------------------------------------------- */
/* Runtime — tier-2 idiomatic helpers                              */
/* -------------------------------------------------------------- */

uint8_t tile_power_l_1t_is_charging(tile_t* tile)
{
    power_l_1t_status_t s;
    tile_power_l_1t_get_charge_status(tile, &s);
    return s.charging ? 1 : 0;
}

uint8_t tile_power_l_1t_is_charge_done(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) return 0;
    /* Read STAT0 directly so we don't disturb the clear-on-read
     * FLAG3 register. CHARGE_DONE is bit 5 of STAT0. */
    return (bq_read(tile, BQ25150_REG_STAT0) & 0x20) ? 1 : 0;
}

uint8_t tile_power_l_1t_is_battery_low(tile_t* tile, uint8_t threshold_pct)
{
    return (tile_power_l_1t_get_percent(tile) < threshold_pct) ? 1 : 0;
}

uint8_t tile_power_l_1t_is_powered(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) return 0;
    /* VIN_PGOOD is bit 0 of STAT0. Read STAT0 directly to leave
     * FLAG3 (clear-on-read) alone for callers that also poll
     * get_charge_status. */
    return (bq_read(tile, BQ25150_REG_STAT0) & 0x01) ? 1 : 0;
}

uint8_t tile_power_l_1t_wait_for_charge_done(tile_t* tile, uint32_t timeout_ms)
{
    if (tile->state != TILE_STATE_READY) return 0;

    /* Poll cadence: 1 s. Charge state evolves on the order of
     * minutes, so polling faster gains nothing. We always check
     * once before sleeping so a zero timeout still does the
     * right thing if the cycle is already complete. */
    const uint32_t poll_ms = 1000u;
    uint32_t elapsed = 0;
    while (1) {
        if (tile_power_l_1t_is_charge_done(tile)) return 1;
        if (elapsed >= timeout_ms) return 0;
        uint32_t step = (timeout_ms - elapsed < poll_ms)
                          ? (timeout_ms - elapsed)
                          : poll_ms;
        tile->hal->delay_ms(step);
        elapsed += step;
    }
}

/* ============================================================== */
/* v3.2 capability additions                                       */
/* ============================================================== */

void tile_power_l_1t_charger_enable(tile_t* tile, uint8_t on)
{
    /* ICCTRL2.CHARGER_DISABLE (bit 0): 1 = charging disabled. */
    uint8_t v = bq_read(tile, BQ25150_REG_ICCTRL2);
    if (on) v &= (uint8_t)~0x01;
    else    v |= 0x01;
    bq_write(tile, BQ25150_REG_ICCTRL2, v);
}

void tile_power_l_1t_set_battery_uvlo_mv(tile_t* tile, uint16_t mv)
{
    /* BUVLO[2:0]: 3.0V=000, 2.8V=011, 2.6V=100, 2.4V=101, 2.2V=110.
     * Pick the highest code whose voltage is <= the request (so the
     * cutoff never sits above what the caller asked for). */
    uint8_t code;
    if (mv >= 3000)      code = 0x0;  /* 3.0 V */
    else if (mv >= 2800) code = 0x3;  /* 2.8 V */
    else if (mv >= 2600) code = 0x4;  /* 2.6 V */
    else if (mv >= 2400) code = 0x5;  /* 2.4 V */
    else                 code = 0x6;  /* 2.2 V */

    uint8_t v = bq_read(tile, BQ25150_REG_BUVLO);
    v = (uint8_t)((v & ~0x07) | code);  /* preserve VLOWV_SEL + IBAT_OCP */
    bq_write(tile, BQ25150_REG_BUVLO, v);
}

void tile_power_l_1t_set_safety_timer(tile_t* tile, power_l_1t_safety_timer_t mode)
{
    /* CHARGERCTRL0.SAFETY_TIMER_LIMIT[2:1]. Preserve TS_EN and the rest. */
    uint8_t v = bq_read(tile, BQ25150_REG_CHARGERCTRL0);
    v = (uint8_t)((v & ~0x06) | (((uint8_t)mode & 0x03) << 1));
    bq_write(tile, BQ25150_REG_CHARGERCTRL0, v);
}

void tile_power_l_1t_set_pmid_mode(tile_t* tile, power_l_1t_pmid_mode_t mode)
{
    /* ICCTRL1.PMID_MODE[1:0]. Preserve PG_MODE / ADCIN_MODE / MR_LPRESS. */
    uint8_t v = bq_read(tile, BQ25150_REG_ICCTRL1);
    v = (uint8_t)((v & ~0x03) | ((uint8_t)mode & 0x03));
    bq_write(tile, BQ25150_REG_ICCTRL1, v);
}

void tile_power_l_1t_set_adc_comparator(tile_t* tile, uint8_t comp,
                                        power_l_1t_adc_channel_t channel,
                                        uint16_t threshold)
{
    uint8_t ch = (uint8_t)channel & 0x07;
    /* The threshold is 12 bits, left-justified: ADCALARM_COMPn_L[7:4] holds
     * its low nibble, bit3 is ADCALARM_ABOVE (polarity) and bits 2:0 are
     * reserved (datasheet 8.5.1.42). Mask the low nibble off so a threshold
     * never flips the polarity or writes reserved bits; polarity stays 0
     * ("flag when the measurement falls below"). */
    switch (comp) {
    case 1: {
        /* Channel in ADCCTRL0[2:0]; threshold at 0x52/0x53. */
        uint8_t v = bq_read(tile, BQ25150_REG_ADCCTRL0);
        v = (uint8_t)((v & ~0x07) | ch);
        bq_write(tile, BQ25150_REG_ADCCTRL0, v);
        bq_write(tile, BQ25150_REG_ADCALARM_C1_M, (uint8_t)(threshold >> 8));
        bq_write(tile, BQ25150_REG_ADCALARM_C1_L, (uint8_t)(threshold & 0xF0));
        break;
    }
    case 2: {
        /* Channel in ADCCTRL1[7:5]; threshold at 0x54/0x55. */
        uint8_t v = bq_read(tile, BQ25150_REG_ADCCTRL1);
        v = (uint8_t)((v & ~0xE0) | (ch << 5));
        bq_write(tile, BQ25150_REG_ADCCTRL1, v);
        bq_write(tile, BQ25150_REG_ADCALARM_C2_M, (uint8_t)(threshold >> 8));
        bq_write(tile, BQ25150_REG_ADCALARM_C2_L, (uint8_t)(threshold & 0xF0));
        break;
    }
    case 3: {
        /* Channel in ADCCTRL1[4:2]; threshold at 0x56/0x57. */
        uint8_t v = bq_read(tile, BQ25150_REG_ADCCTRL1);
        v = (uint8_t)((v & ~0x1C) | (ch << 2));
        bq_write(tile, BQ25150_REG_ADCCTRL1, v);
        bq_write(tile, BQ25150_REG_ADCALARM_C3_M, (uint8_t)(threshold >> 8));
        bq_write(tile, BQ25150_REG_ADCALARM_C3_L, (uint8_t)(threshold & 0xF0));
        break;
    }
    default:
        break;
    }
}

uint8_t tile_power_l_1t_get_adc_comparators(tile_t* tile)
{
    /* FLAG2[6:4] = COMP1/2/3 alarm (clear on read). */
    return (uint8_t)(bq_read(tile, BQ25150_REG_FLAG2) &
                     (BQ25150_FLAG2_COMP1_ALARM |
                      BQ25150_FLAG2_COMP2_ALARM |
                      BQ25150_FLAG2_COMP3_ALARM));
}
