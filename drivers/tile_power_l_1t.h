/**
 * @file   tile_power_l_1t.h
 * @brief  Li-Ion charge controller driver for the Power.L.1T tile (rev b).
 *
 * Embeds the Texas Instruments BQ25150, a single-cell Li-Ion charge
 * controller with programmable 1.8 V LDO output and 12-bit ADC for
 * battery and system monitoring.
 *
 * Key specifications:
 *   - Charge input:     3.4–5.5 V (VUVLO / VOVP), input limit 50–600 mA
 *   - Battery voltage:  3.6–4.6 V regulation (programmable in 10 mV steps)
 *   - LDO output:       V+ (pad 10), 1.8 V at power-on, 0.6–3.7 V
 *                       programmable, 100 mA max (datasheet 7.3 ILDO)
 *   - ADC:              12-bit, 6 input channels
 *   - Charge current:   1.25–500 mA programmable
 *   - No battery NTC:   TS is strapped to a fixed 5 kΩ (see below)
 *
 * Tile wiring that matters to firmware (Power-L-1T-b schematic):
 *   - I²C only answers on battery when LP (pad 2) is driven > 1.35 V; with
 *     LP low (its 900 kΩ default) and no SUPPLY+, the chip is in low-power
 *     mode with I²C and the ADC off. With SUPPLY+ present LP has no effect.
 *   - GND (pad 1) is a SWITCHED ground: an SI8806 N-FET joins it to
 *     BATT-/SUPPLY- (pads 6/9), its gate fed from V+ through 220 kΩ. SW
 *     (pad 3) to ground, LDO disabled, PMID off or ship mode all open it.
 *   - IMAX = 10 kΩ (no hardware charge-current cap below 500 mA).
 *   - /CE, /MR, /PG and INT are not routed (/CE's internal pull-down keeps
 *     charging enabled).
 *
 * Datasheet: https://www.bergsonne.io/tiles/power/l1t
 * IC datasheet: https://www.ti.com/lit/ds/symlink/bq25150.pdf
 *
 * @studio tile label=Power.L.1T icon=☷
 *
 * Quick start:
 * @code
 *   #include "core_tiles.h"
 *
 *   tile_t battery;
 *   tile_power_l_1t_init(core_tiles_pal(&core_i2c1), 0, &battery, NULL);
 *   if (tile_is_ready(&battery)) {
 *       tile_power_l_1t_set_charge_current_ma(&battery, 100);
 *       tile_power_l_1t_set_charge_voltage_mv(&battery, 4200);
 *       uint16_t vbat = tile_power_l_1t_get_vbat_mv(&battery);
 *       if (tile_power_l_1t_is_battery_low(&battery, 10)) {
 *           // go to sleep, save the cell
 *       }
 *   }
 * @endcode
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=advanced category="Battery NTC / JEITA"
 *   Hardware-gated (not a driver gap). The BQ25150 can pause or derate
 *   charging from a battery thermistor on TS, but Power-L-1T-b straps TS to
 *   a fixed 5 kΩ resistor (the datasheet's "TS not used" connection), which
 *   reads a constant ~0.40 V: always the normal band. Battery temperature is
 *   therefore NOT monitored; set_ts_* and set_ts_enabled have no practical
 *   effect. The chip's die thermal foldback (80 °C) still applies.
 *
 * @studio unsupported severity=advanced category="MR button + INT pin handling"
 *   The chip's MR (push-button) and INT (interrupt) pins aren't
 *   routed to tile pads on the current revision — nothing for a
 *   Core GPIO to attach to. Closing this gap requires a tile
 *   hardware revision that routes at least INT to a connector
 *   pad. Until then, MASK0–3 / MRCTRL register writes have no
 *   external effect.
 */

#ifndef INC_TILE_POWER_L_1T_H_
#define INC_TILE_POWER_L_1T_H_

#include "tiles.h"
#include <stdint.h>

/* -------------------------------------------------------------- */
/* Driver version                                                  */
/* -------------------------------------------------------------- */

#define TILE_POWER_L_1T_VERSION_MAJOR  3
#define TILE_POWER_L_1T_VERSION_MINOR  3
#define TILE_POWER_L_1T_VERSION_PATCH  0

TILES_CHECK_VERSION(1, 0);  /* requires tiles.h >= 1.0 */

/* -------------------------------------------------------------- */
/* Instance mapping                                                */
/* -------------------------------------------------------------- */

/**
 * @brief  Instance-to-address mapping for Power.L.1T.
 *
 * | Instance | ID   | Bus  | Hardware config      |
 * |----------|------|------|----------------------|
 * | 0        | 0x6B | I2C  | Fixed address        |
 *
 * @note  The BQ25150 has a single fixed I2C address. Multiple
 *        Power.L.1T tiles require separate I2C buses.
 */
#define BQ25150_I2C_ADDR_DEFAULT    0x6B

/* -------------------------------------------------------------- */
/* BQ25150 register map (subset used by this driver)               */
/* -------------------------------------------------------------- */

#define BQ25150_REG_STAT0           0x00  /**< Charger Status 0 */
#define BQ25150_REG_STAT1           0x01  /**< Charger Status 1 */
#define BQ25150_REG_STAT2           0x02  /**< ADC Status */
#define BQ25150_REG_FLAG0           0x03  /**< Charger Flags 0 */
#define BQ25150_REG_FLAG1           0x04  /**< Charger Flags 1 */
#define BQ25150_REG_FLAG2           0x05  /**< ADC Flags */
#define BQ25150_FLAG2_ADC_READY     0x80  /**< FLAG2 bit7: ADC conversion completed (clear-on-read) */
#define BQ25150_REG_FLAG3           0x06  /**< Timer Flags */
#define BQ25150_REG_VBAT_CTRL       0x12  /**< Battery Voltage Control */
#define BQ25150_REG_ICHG_CTRL       0x13  /**< Fast Charge Current Control */
#define BQ25150_REG_PCHRGCTRL       0x14  /**< Pre-Charge Current Control */
#define BQ25150_REG_TERMCTRL        0x15  /**< Termination Current Control */
#define BQ25150_REG_BUVLO           0x16  /**< Battery UVLO and Current Limit */
#define BQ25150_REG_CHARGERCTRL0    0x17  /**< Charger Control 0 (TS, watchdog) */
#define BQ25150_REG_ILIMCTRL        0x19  /**< Input Current Limit Control */
#define BQ25150_REG_LDOCTRL         0x1D  /**< LDO Control (enable, voltage, mode) */
#define BQ25150_REG_ICCTRL0         0x35  /**< IC Control 0 (ship mode) */
#define BQ25150_REG_ADCCTRL0        0x40  /**< ADC Control 0 */
#define BQ25150_ADCCTRL0_RATE_1S    0x80  /**< ADC_READ_RATE = every 1 s (battery mode) */
#define BQ25150_ADCCTRL0_RATE_MANUAL 0x00 /**< ADC_READ_RATE = manual (convert on CONV_START) */
#define BQ25150_ADCCTRL0_CONV_START 0x20  /**< bit5: trigger one ADC conversion */
#define BQ25150_ADCCTRL0_COMP1_DEFAULT 0x02 /**< ADC_COMP1[2:0] reset value */
#define BQ25150_REG_ADC_DATA_VBAT_M 0x42  /**< ADC VBAT MSB */
#define BQ25150_REG_ADC_DATA_VBAT_L 0x43  /**< ADC VBAT LSB */
#define BQ25150_REG_ADC_DATA_TS_M   0x44  /**< ADC TS MSB */
#define BQ25150_REG_ADC_DATA_ICHG_M 0x46  /**< ADC ICHG MSB */
#define BQ25150_REG_ADC_DATA_ADCIN_M 0x48 /**< ADC ADCIN MSB */
#define BQ25150_REG_ADC_DATA_VIN_M  0x4A  /**< ADC VIN MSB */
#define BQ25150_REG_ADC_DATA_PMID_M 0x4C  /**< ADC PMID MSB */
#define BQ25150_REG_ADC_DATA_IIN_M  0x4E  /**< ADC IIN MSB */
#define BQ25150_REG_ADC_READ_EN     0x58  /**< ADC Channel Enable */
#define BQ25150_REG_TS_FASTCHGCTRL  0x61  /**< TS Charge Control */
#define BQ25150_REG_TS_COLD         0x62  /**< TS Cold Threshold */
#define BQ25150_REG_TS_COOL         0x63  /**< TS Cool Threshold */
#define BQ25150_REG_TS_WARM         0x64  /**< TS Warm Threshold */
#define BQ25150_REG_TS_HOT          0x65  /**< TS Hot Threshold */
#define BQ25150_REG_DEVICE_ID       0x6F  /**< Device identification */

/* ---- Registers added for the v3.2 capability set ---- */
#define BQ25150_REG_CHARGERCTRL1    0x18  /**< Charger Control 1 (VINDPM, DPPM, THERM_REG) */
#define BQ25150_REG_ICCTRL1         0x36  /**< IC Control 1 (PG mode, PMID mode) */
#define BQ25150_REG_ICCTRL2         0x37  /**< IC Control 2 (charger disable) */
#define BQ25150_REG_ADCCTRL1        0x41  /**< ADC Control 1 (COMP2/COMP3 channel) */
#define BQ25150_REG_ADCALARM_C1_M   0x52  /**< ADC comparator 1 threshold MSB */
#define BQ25150_REG_ADCALARM_C1_L   0x53  /**< ADC comparator 1 threshold LSB */
#define BQ25150_REG_ADCALARM_C2_M   0x54  /**< ADC comparator 2 threshold MSB */
#define BQ25150_REG_ADCALARM_C2_L   0x55  /**< ADC comparator 2 threshold LSB */
#define BQ25150_REG_ADCALARM_C3_M   0x56  /**< ADC comparator 3 threshold MSB */
#define BQ25150_REG_ADCALARM_C3_L   0x57  /**< ADC comparator 3 threshold LSB */

/* FLAG2 (0x05) ADC-comparator alarm bits (clear on read) */
#define BQ25150_FLAG2_COMP1_ALARM   0x40  /**< Comparator 1 threshold crossed */
#define BQ25150_FLAG2_COMP2_ALARM   0x20  /**< Comparator 2 threshold crossed */
#define BQ25150_FLAG2_COMP3_ALARM   0x10  /**< Comparator 3 threshold crossed */

/** Charge safety-timer limit (CHARGERCTRL0[2:1]). */
typedef enum {
    POWER_L_1T_SAFETY_3H   = 0,  /**< 3-hour fast-charge timer */
    POWER_L_1T_SAFETY_6H   = 1,  /**< 6-hour (default)         */
    POWER_L_1T_SAFETY_12H  = 2,  /**< 12-hour                  */
    POWER_L_1T_SAFETY_OFF  = 3,  /**< disabled                 */
} power_l_1t_safety_timer_t;

/** PMID power-path mode (ICCTRL1[1:0]). */
typedef enum {
    POWER_L_1T_PMID_AUTO     = 0,  /**< powered from BAT or VIN (default) */
    POWER_L_1T_PMID_BAT_ONLY = 1,  /**< forced from BAT even when VIN present */
    POWER_L_1T_PMID_FLOAT    = 2,  /**< disconnected, floating */
    POWER_L_1T_PMID_PULLDOWN = 3,  /**< disconnected, pulled down */
} power_l_1t_pmid_mode_t;

/** ADC channel selector for the programmable comparators (ADC_COMPn). */
typedef enum {
    POWER_L_1T_ADC_CH_DISABLED = 0,
    POWER_L_1T_ADC_CH_ADCIN    = 1,
    POWER_L_1T_ADC_CH_TS       = 2,
    POWER_L_1T_ADC_CH_VBAT     = 3,
    POWER_L_1T_ADC_CH_ICHARGE  = 4,
    POWER_L_1T_ADC_CH_VIN      = 5,
    POWER_L_1T_ADC_CH_PMID     = 6,
    POWER_L_1T_ADC_CH_IIN      = 7,
} power_l_1t_adc_channel_t;

/** @brief  Expected DEVICE_ID register value. */
#define BQ25150_DEVICE_ID_DEFAULT   0x20

/* -------------------------------------------------------------- */
/* Status struct                                                   */
/* -------------------------------------------------------------- */

/**
 * @brief  Snapshot of the BQ25150's current state and latched faults.
 *
 * Populated by tile_power_l_1t_get_charge_status(). Mixes live state
 * (vin_pgood, charging, *_active) and clear-on-read event flags
 * (faults). Reading the flags clears them — call once per polling
 * cycle, not multiple times.
 */
typedef struct {
    /* Live state (from STAT0 / STAT1) */
    uint16_t vin_pgood       : 1;  /**< 1 = VIN within valid range */
    uint16_t charging        : 1;  /**< 1 = charging active */
    uint16_t cv_mode         : 1;  /**< 1 = CV (taper) phase */
    uint16_t charge_done     : 1;  /**< 1 = termination reached */
    uint16_t iinlim_active   : 1;  /**< 1 = input current limit reducing charge */
    uint16_t vindpm_active   : 1;  /**< 1 = VIN dynamic power management active */
    uint16_t thermreg_active : 1;  /**< 1 = thermal regulation foldback active */
    /* Faults (from FLAG1 / FLAG3 — clear on read) */
    uint16_t vin_ovp         : 1;  /**< VIN above OVP threshold */
    uint16_t bat_ocp         : 1;  /**< Battery over-current detected */
    uint16_t bat_uvlo        : 1;  /**< Battery under-voltage lockout */
    uint16_t safety_timer    : 1;  /**< Charge safety timer expired */
    uint16_t watchdog        : 1;  /**< I²C watchdog expired (init disables WD, so should be 0) */
    /* TS thresholds (from STAT1 / FLAG1) */
    uint16_t ts_cold         : 1;  /**< NTC reads below TS_COLD (charging paused) */
    uint16_t ts_cool         : 1;  /**< NTC in cool band (reduced charge current) */
    uint16_t ts_warm         : 1;  /**< NTC in warm band (reduced charge voltage) */
    uint16_t ts_hot          : 1;  /**< NTC reads above TS_HOT (charging stopped) */
} power_l_1t_status_t;

/* -------------------------------------------------------------- */
/* Public API                                                      */
/* -------------------------------------------------------------- */

/**
 * @brief  Check whether a BQ25150 is present on the I2C bus.
 *
 * @param  hal       Platform HAL handle
 * @param  instance  Instance index (0 = default, see mapping table)
 * @return 1 if device ACKs, 0 otherwise
 */
uint8_t tile_power_l_1t_find(tiles_pal_t* hal, uint8_t instance);

/**
 * Optional init config. Pass NULL for defaults.
 * Reserved for future use.
 */
typedef struct {
    uint8_t reserved;   /**< Placeholder — no options yet. */
} power_l_1t_cfg_t;

/**
 * @brief  Initialize the BQ25150 charge controller.
 *
 * Verifies the device ID, disables the I²C watchdog (the chip
 * otherwise resets all charge parameters every 50 s), enables all
 * 6 ADC channels, configures charge defaults (80 mA fast charge,
 * 18.75 mA pre-charge, 4.20 V regulation, 2.6 V battery UVLO, TS on,
 * 6 h safety timer) and clears the ship-mode request. Input limit,
 * termination, LDO and charger-enable are left as the chip holds them.
 * Pass cfg=NULL for defaults.
 *
 * On battery alone, LP (pad 2) must be high for this to find the chip
 * (I²C is off in low-power mode).
 *
 * @param  hal       Platform HAL handle
 * @param  instance  Instance index (0 = default, see mapping table)
 * @param  tile      Pointer to tile handle (populated by this function)
 * @param  cfg       Optional config, or NULL for defaults
 */
void tile_power_l_1t_init(tiles_pal_t* hal, uint8_t instance, tile_t* tile,
                          const power_l_1t_cfg_t *cfg);

/* ---- Charge configuration ---------------------------------------- */

/**
 * @brief  Set the fast-charge current.
 *
 * Programs ICHG_CTRL with the appropriate ICHARGE_RANGE bit so the
 * resolution scales: ≤318 mA uses 1.25 mA steps, >318 mA uses
 * 2.5 mA steps (rounded down). Values clamp to 500 mA. Crossing 318 mA
 * also doubles the pre-charge step, so the pre-charge current doubles.
 * Keep it at or below 1C for the cell fitted; there is no thermistor.
 *
 * @studio expose category=tile name=set_charge_current_ma section=config
 * @studio control ma label="Charge current" tier=basic default=80
 * @param  ma    [2..500] mA Target charge current in mA (below 2 the code is 0: no charge).
 */
void tile_power_l_1t_set_charge_current_ma(tile_t* tile, uint16_t ma);

/**
 * @brief  Set the battery regulation voltage.
 *
 * VBATREG = 3600 + code × 10 mV. Values outside 3600–4600 mV clamp
 * to that range. Use 4200 for typical Li-Ion, 3650 for LFP, 4350+
 * for high-voltage NMC variants.
 *
 * @studio expose category=tile name=set_charge_voltage_mv section=config
 * @studio control mv label="Charge to" tier=advanced default=4200 scale=0.001 unit=V
 * @param  mv    [3600..4600] mV Target battery voltage in millivolts.
 */
void tile_power_l_1t_set_charge_voltage_mv(tile_t* tile, uint16_t mv);

/**
 * @brief  Set the pre-charge current (used when VBAT < VLOW).
 *
 * Pre-charge applies to deeply-discharged cells; usually 10–20 % of
 * the fast-charge rate. Values clamp to 1.25–77.5 mA.
 *
 * @studio expose category=tile name=set_pre_charge_ma section=config
 * @param  ma   [2..77] mA Target pre-charge current (38 mA max below 318 mA fast charge).
 */
void tile_power_l_1t_set_pre_charge_ma(tile_t* tile, uint8_t ma);

/**
 * @brief  Set the termination current as a percentage of ICHG.
 *
 * Charging ends when the cell's current draw drops below this
 * threshold during the CV phase. 0 disables termination (charges
 * continuously until VBATREG is reached). Values clamp to 1–31 %.
 *
 * @studio expose category=tile name=set_termination_percent section=config
 * @param  pct   [0..31] % Termination current as % of ICHG (0 = disabled).
 */
void tile_power_l_1t_set_termination_percent(tile_t* tile, uint8_t pct);

/**
 * @brief  Set the input (SUPPLY+) current limit.
 *
 * ILIMCTRL is a discrete table: 50, 100, 150, 200, 300, 400, 500 or
 * 600 mA. The largest level not above the request is used (50 mA
 * floor). The chip powers up at 100 mA and init() leaves it there;
 * when the load plus charge current reach the limit, charge current
 * folds back first. The tile is rated for 500 mA on SUPPLY+.
 *
 * @studio expose category=tile name=set_input_current_limit_ma section=config
 * @studio control ma label="Input current limit" tier=basic default=100
 * @param  ma   [50..600] mA Input current limit.
 */
void tile_power_l_1t_set_input_current_limit_ma(tile_t* tile, uint16_t ma);

/* ---- ADC reads (milli-units) ------------------------------------- */

/**
 * @brief  Read battery voltage from the ADC.
 *
 * Replaces the deprecated raw-counts API; performs a 2-byte burst
 * read to avoid torn samples.
 *
 * @studio expose category=tile name=get_vbat_mv returns=int section=runtime
 * @return Battery voltage in mV (0–6000)
 */
uint16_t tile_power_l_1t_get_vbat_mv(tile_t* tile);

/**
 * @brief  Read input (VIN) voltage from the ADC.
 * @studio expose category=tile name=get_vin_mv returns=int section=runtime
 * @return VIN in mV (0–6000)
 */
uint16_t tile_power_l_1t_get_vin_mv(tile_t* tile);

/**
 * @brief  Read PMID (system rail) voltage from the ADC.
 * @studio expose category=tile name=get_pmid_mv returns=int section=runtime
 * @return PMID in mV (0–6000)
 */
uint16_t tile_power_l_1t_get_pmid_mv(tile_t* tile);

/**
 * @brief  Read charge current (into the battery) from the ADC.
 *
 * Scaled relative to the configured ICHG fast-charge limit; returns
 * actual mA flowing into the cell.
 *
 * @studio expose category=tile name=get_charge_current_ma returns=int section=runtime
 * @return Charge current in mA (0–500)
 */
uint16_t tile_power_l_1t_get_charge_current_ma(tile_t* tile);

/**
 * @brief  Read input current (from VIN) from the ADC.
 *
 * Range scales with ILIMCTRL: ≤150 mA range gives 0–375 mA full
 * scale; >150 mA range gives 0–750 mA full scale.
 *
 * @studio expose category=tile name=get_input_current_ma returns=int section=runtime
 * @return Input current in mA (0–750)
 */
uint16_t tile_power_l_1t_get_input_current_ma(tile_t* tile);

/**
 * @brief  Read raw TS pin voltage from the ADC.
 *
 * Returns the millivolt reading on the TS pin (0–1200 mV). On
 * Power-L-1T-b TS is a fixed 5 kΩ to ground (no thermistor), so this
 * reads a constant ~400 mV (5 kΩ x 80 µA bias); it is not a
 * temperature. Not measured in low-power mode.
 *
 * @studio expose category=tile name=get_ts_mv returns=int section=runtime
 * @return TS voltage in mV (0–1200)
 */
uint16_t tile_power_l_1t_get_ts_mv(tile_t* tile);

/**
 * @brief  Read raw ADCIN pin voltage from the ADC.
 *
 * Optional auxiliary input (0–1.2 V range). Useful for monitoring
 * an external sensor (e.g., separate NTC, voltage divider).
 *
 * @studio expose category=tile name=get_adcin_mv returns=int section=runtime
 * @return ADCIN voltage in mV (0–1200)
 */
uint16_t tile_power_l_1t_get_adcin_mv(tile_t* tile);

/* ---- Battery percent (derived) ---------------------------------- */

/**
 * @brief  Read battery state-of-charge as a percentage.
 *
 * Derived from VBAT using a linear curve (3000 mV = 0%, 4200 mV =
 * 100%). Coarse — a real fuel gauge would integrate Coulombs.
 *
 * @studio expose category=tile name=get_percent returns=int section=runtime
 * @return Battery percentage (0–100)
 */
uint8_t tile_power_l_1t_get_percent(tile_t* tile);

/* ---- NTC (TS pin) thresholds ------------------------------------- */

/**
 * @brief  Set the cold-temperature TS threshold (raw register value).
 *
 * The TS register codes are bit-positional (1, 2, 4, 8, 16, ...
 * encode multiples of 4.688 mV up to 600 mV). Datasheet section
 * 8.5.1.49–52 describes the exact mapping. No practical effect on
 * Power-L-1T-b: TS is a fixed 5 kΩ, not a thermistor.
 *
 * @studio expose category=tile name=set_ts_cold section=config
 * @param  code  Raw 8-bit TS_COLD register value
 */
void tile_power_l_1t_set_ts_cold(tile_t* tile, uint8_t code);

/**
 * @brief  Set the cool-temperature TS threshold (raw register value).
 * @studio expose category=tile name=set_ts_cool section=config
 * @param  code  Raw 8-bit TS_COOL register value
 */
void tile_power_l_1t_set_ts_cool(tile_t* tile, uint8_t code);

/**
 * @brief  Set the warm-temperature TS threshold (raw register value).
 * @studio expose category=tile name=set_ts_warm section=config
 * @param  code  Raw 8-bit TS_WARM register value
 */
void tile_power_l_1t_set_ts_warm(tile_t* tile, uint8_t code);

/**
 * @brief  Set the hot-temperature TS threshold (raw register value).
 * @studio expose category=tile name=set_ts_hot section=config
 * @param  code  Raw 8-bit TS_HOT register value
 */
void tile_power_l_1t_set_ts_hot(tile_t* tile, uint8_t code);

/**
 * @brief  Enable or disable TS-based thermal protection.
 *
 * When disabled, the chip ignores the TS pin for charge control
 * (monitoring continues). Default at init: enabled. No practical
 * effect on Power-L-1T-b: TS is a fixed 5 kΩ, not a thermistor.
 *
 * @studio expose category=tile name=set_ts_enabled section=config
 * @param  enabled  1 = TS thermal protection on, 0 = off
 */
void tile_power_l_1t_set_ts_enabled(tile_t* tile, uint8_t enabled);

/* ---- LDO output --------------------------------------------------- */

/** LDO output mode — regulated voltage vs pass-through load switch. */
typedef enum {
    POWER_L_1T_LDO_MODE_LDO         = 0,  /**< Regulated LDO output */
    POWER_L_1T_LDO_MODE_LOAD_SWITCH = 1,  /**< Pass-through load switch */
} power_l_1t_ldo_mode_t;

/**
 * @brief  Set the LDO output voltage.
 *
 * VLDO = 600 + code × 100 mV (rounded down). Values clamp to
 * 600–3700 mV.
 *
 * @warning The datasheet (8.3.5) says the output voltage "can only be
 * changed when the EN_LS_LDO ... have disabled the output". This call
 * writes the code without disabling the LDO, so a change made while
 * V+ is on may not apply until the LDO is next disabled and enabled.
 * V+ also drives the pad 1 ground switch gate (SI8806, VGS(th) up to
 * 1.0 V, RDS(on) specified from 1.8 V): below ~1.8 V the downstream
 * ground is not reliably closed.
 *
 * @studio expose category=tile name=set_ldo_voltage_mv section=config
 * @param  mv    [600..3700] mV Target LDO voltage.
 */
void tile_power_l_1t_set_ldo_voltage_mv(tile_t* tile, uint16_t mv);

/**
 * @brief  Set the LS/LDO output mode (regulated LDO vs load switch).
 *
 * In LDO mode the chip regulates pad 10 to the configured voltage;
 * in load-switch mode it connects VINLS straight through. VINLS is
 * tied to PMID on this tile, so pad 10 then carries PMID (VIN or
 * VBAT, up to 5.5 V). 100 mA max either way (datasheet 7.3). The
 * datasheet (8.3.5) requires the LDO to be disabled before the mode
 * changes; this call does not do that.
 *
 * @studio expose category=tile name=set_ldo_mode section=config
 * @param  mode  LDO mode (POWER_L_1T_LDO_MODE_LDO or _LOAD_SWITCH)
 */
void tile_power_l_1t_set_ldo_mode(tile_t* tile, power_l_1t_ldo_mode_t mode);

/**
 * @brief  Enable or disable the LS/LDO output.
 *
 * When disabled, pad 10 (V+) is pulled down by the chip, and because
 * V+ feeds the pad 1 ground-switch gate, the downstream ground (pad 1)
 * opens too: everything on this tile's output powers off. Default:
 * enabled (chip reset value; init does not touch it).
 *
 * @studio expose category=tile name=set_ldo_enabled section=config
 * @param  enabled  1 = output on, 0 = output off (high-Z)
 */
void tile_power_l_1t_set_ldo_enabled(tile_t* tile, uint8_t enabled);

/* ---- Status / fault read ----------------------------------------- */

/**
 * @brief  Snapshot the chip's current state + latched faults.
 *
 * Reads STAT0/STAT1 (live state) and FLAG0/FLAG1/FLAG3 (event flags
 * which clear on read) into the supplied struct. Call once per
 * polling cycle — multiple reads will lose flag transitions.
 *
 * @studio expose category=tile name=get_charge_status section=runtime
 * @param  out   Caller-allocated status struct (zeroed on entry)
 */
void tile_power_l_1t_get_charge_status(tile_t* tile,
                                       power_l_1t_status_t *out);

/* ---- Power management -------------------------------------------- */

/**
 * @brief  Enter ship mode (~10 nA quiescent).
 *
 * Sets EN_SHIP_MODE. If SUPPLY+ is present the chip waits until it is
 * removed, then disconnects the battery (10 nA typ). /MR is not routed
 * on this tile, so the only way out is applying SUPPLY+ again; the
 * output (pad 10, and the pad 1 ground) stays off until then. Registers
 * return to reset values on exit. Use only for storage / shipping.
 *
 * @warning Destructive: requires physical user action (MR press or VIN
 * re-insertion) to recover. Marked `section=advanced` for the same
 * posture as other one-way / hardware-gated operations.
 *
 * @studio expose category=tile name=enter_ship_mode section=advanced
 */
void tile_power_l_1t_enter_ship_mode(tile_t* tile);

/* ---- Raw register access (escape hatches) ------------------------ */

/**
 * @brief  Read any 8-bit BQ25150 register.
 *
 * @studio expose category=tile name=read_status returns=int section=advanced
 * @param  reg   Register address
 * @return 8-bit register value
 */
uint8_t tile_power_l_1t_read_status(tile_t* tile, uint8_t reg);

/**
 * @brief  Write any 8-bit BQ25150 register.
 *
 * Escape hatch for advanced users wanting to touch registers the
 * driver doesn't expose. Caller is responsible for not bricking the
 * chip — most useful registers have typed setters above.
 *
 * @studio expose category=tile name=write_reg section=advanced
 * @param  reg    Register address
 * @param  value  Value to write
 */
void tile_power_l_1t_write_reg(tile_t* tile, uint8_t reg, uint8_t value);

/* ============================================================== */
/* Runtime — tier-2 idiomatic helpers                              */
/*                                                                  */
/* These compose the tier-1 surface above into "is the battery     */
/* doing the thing I care about?" calls. All the boolean checks    */
/* are quick reads (one or two register accesses); only            */
/* wait_for_charge_done blocks, and even that polls at a slow      */
/* 1 s cadence because charge state changes on the order of        */
/* minutes — there's no value in spinning faster.                  */
/* ============================================================== */

/**
 * @brief  Is the cell currently being charged?
 *
 * @studio expose category=tile name=is_charging returns=bool section=runtime
 *
 * Convenience over @ref tile_power_l_1t_get_charge_status — returns
 * the `charging` field as a bool: VIN good, not done, charger not
 * disabled over I²C, PMID in AUTO, no UVLO / OVP / TS cold / TS hot.
 * Note this reads (and clears) FLAG3 as a side effect; if you also poll
 * the full status struct, alternate with this rather than calling both
 * per cycle.
 *
 * @return 1 if charging, 0 otherwise
 */
uint8_t tile_power_l_1t_is_charging(tile_t* tile);

/**
 * @brief  Has charge termination been reached?
 *
 * @studio expose category=tile name=is_charge_done returns=bool section=runtime
 *
 * Returns the `charge_done` bit from STAT0 — set when the cell
 * reaches VBATREG and current tapers below the termination threshold.
 *
 * @return 1 if charge cycle has finished, 0 otherwise
 */
uint8_t tile_power_l_1t_is_charge_done(tile_t* tile);

/**
 * @brief  Is the battery below `threshold_pct`?
 *
 * @studio expose category=tile name=is_battery_low returns=bool section=runtime
 *
 * The classic "go to sleep" trigger. Equivalent to
 * `get_percent() < threshold_pct`. Quick: one ADC read.
 *
 * @param  threshold_pct  Threshold in percent (0–100)
 * @return 1 if battery percent is strictly below threshold, 0 otherwise
 */
uint8_t tile_power_l_1t_is_battery_low(tile_t* tile, uint8_t threshold_pct);

/**
 * @brief  Is external power present (running off VIN)?
 *
 * @studio expose category=tile name=is_powered returns=bool section=runtime
 *
 * Returns the `vin_pgood` bit — true when VIN is in the valid
 * operating range (3.4–5.5 V). Use to branch behaviour between
 * "plugged in" and "battery only" modes.
 *
 * @return 1 if VIN is good, 0 if running off battery only
 */
uint8_t tile_power_l_1t_is_powered(tile_t* tile);

/**
 * @brief  Block until charge_done is observed, or timeout.
 *
 * @studio expose category=tile name=wait_for_charge_done returns=bool section=runtime
 *
 * Polls the charge_done bit at a 1 s cadence (charge state changes
 * on the order of minutes for a typical cell — faster polling wastes
 * MCU cycles and bus bandwidth without finer-grained answers).
 * Returns 1 immediately if the cycle is already finished.
 *
 * @param  timeout_ms  Maximum time to wait, in milliseconds
 * @return 1 if charge_done was observed, 0 on timeout
 */
uint8_t tile_power_l_1t_wait_for_charge_done(tile_t* tile, uint32_t timeout_ms);

/* ============================================================== */
/* v3.2 capability additions                                       */
/* ============================================================== */

/**
 * @brief  Enable or disable battery charging over I²C.
 *
 * Sets/clears ICCTRL2.CHARGER_DISABLE. Note the hardware /CE pin still
 * gates charging: when /CE is high, charging is off regardless of this
 * bit; this only takes effect when /CE is held low (the tile's default).
 *
 * @studio expose category=tile name=charger_enable section=config
 * @studio control on label="Charge the battery" tier=basic type=bool default=1
 * @param  on  1 = allow charging, 0 = disable charging.
 */
void tile_power_l_1t_charger_enable(tile_t* tile, uint8_t on);

/**
 * @brief  Set the battery under-voltage lockout (discharge cutoff) threshold.
 *
 * BUVLO[2:0]: 3.0 / 2.8 / 2.6 / 2.4 / 2.2 V (nearest below `mv` is chosen);
 * deeper cutoff trades battery life for runtime. Other BUVLO fields
 * (precharge threshold, OCP limit) are preserved.
 *
 * @studio expose category=tile name=set_battery_uvlo_mv section=config
 * @param  mv  [2200..3000] mV Desired UVLO.
 */
void tile_power_l_1t_set_battery_uvlo_mv(tile_t* tile, uint16_t mv);

/**
 * @brief  Set the charge safety-timer limit.
 *
 * The timer stops a charge that has not terminated in time; with no
 * battery thermistor on this tile it is one of the few backstops, so
 * prefer not to choose OFF. Changing it while charging restarts it.
 *
 * @studio expose category=tile name=set_safety_timer section=config
 * @param  mode  power_l_1t_safety_timer_t (3h / 6h / 12h / off).
 */
void tile_power_l_1t_set_safety_timer(tile_t* tile, power_l_1t_safety_timer_t mode);

/**
 * @brief  Set the PMID power-path mode (ICCTRL1[1:0]).
 *
 * BAT_ONLY stops charging. FLOAT and PULLDOWN disconnect PMID and turn
 * the LDO off, so V+ (pad 10) and the pad 1 ground go off; the datasheet
 * (8.3.6) says those modes exit only through I²C or an /MR reset, and
 * /MR is not routed here. Do not use them if the Core runs from V+.
 *
 * @studio expose category=tile name=set_pmid_mode section=config
 * @param  mode  power_l_1t_pmid_mode_t.
 */
void tile_power_l_1t_set_pmid_mode(tile_t* tile, power_l_1t_pmid_mode_t mode);

/**
 * @brief  Configure a programmable ADC comparator (threshold alarm).
 *
 * Routes ADC channel `channel` to comparator `comp` (1-3) and sets its
 * 16-bit threshold (left-justified, same scale as the channel's ADC
 * result). When the measurement crosses the threshold, the matching
 * COMPn_ALARM bit latches in FLAG2 (read via get_adc_comparators()).
 * Pass channel = POWER_L_1T_ADC_CH_DISABLED to turn a comparator off.
 *
 * Note: the chip's above/below polarity bit (ADCALARM_ABOVE) is not
 * exposed by this driver yet — the comparator uses its default sense
 * (flag when the measurement falls below the threshold).
 *
 * @studio expose category=tile name=set_adc_comparator section=config
 * @param  comp       [1..3] Comparator index.
 * @param  channel    power_l_1t_adc_channel_t to monitor.
 * @param  threshold  Raw left-justified ADC threshold (top 12 bits used).
 */
void tile_power_l_1t_set_adc_comparator(tile_t* tile, uint8_t comp,
                                        power_l_1t_adc_channel_t channel,
                                        uint16_t threshold);

/**
 * @brief  Read (and clear) the ADC comparator alarm flags.
 * @studio expose category=tile name=get_adc_comparators section=runtime
 * @return BQ25150_FLAG2_COMPn_ALARM bits that have tripped since last read.
 */
uint8_t tile_power_l_1t_get_adc_comparators(tile_t* tile);

#endif /* INC_TILE_POWER_L_1T_H_ */
