/**
 * @file   tile_power_l_1n.h
 * @brief  Power.L.1N tile driver — Nordic nPM1300 PMIC.
 *
 * Battery charger + 2 buck regulators + system supply (VSYS) + 3 indicator
 * LEDs. Platform-agnostic: all bus access goes through tiles_pal_t.
 *
 * The bucks come up at fixed boot voltages chosen by the tile's VSET pulldown
 * resistors (no I2C needed): VSET1 = 47 kΩ → VOUT1 = 1.8 V, VSET2 = 330 kΩ →
 * VOUT2 = 3.3 V. buck_set_mv() switches a buck to software control to override
 * its VSET default at runtime.
 *
 * Tile LEDs. Three discrete 0201 parts (Power-L1-N-a BOM), not an RGB package:
 *   U3 = APG015SURKKC-TT  631 nm red
 *   U4 = APG015SEKKC-TT   605 nm ORANGE   (previously documented as "yellow")
 *   U5 = APG015CGKKC-TT   571 nm green
 *
 * Which LED is which (confirmed 2026-09-24): red, orange, green sit left to
 * right on the nPM1300's A1, A2, A3 balls, which are LED0, LED1, LED2 (WLCSP
 * pin table). So LED0 = red (U3), LED1 = orange (U4), LED2 = green (U5).
 *
 * Driver mode assignment made by init() (the chip's own reset assignment):
 *   LED0 red    → ERROR mode    (auto, on for charger faults)
 *   LED1 orange → CHARGING mode (auto, on while charging)
 *   LED2 green  → HOST mode     (firmware "ready"/status, via led_set())
 *
 * Before 1.2.0 init() had this backwards (LED0 HOST, LED2 ERROR, from a
 * misread schematic), so a charger fault lit the green LED and firmware
 * status the red one.
 *
 * @note Prefer conveying state by blink PATTERN rather than by colour. A
 * misstuffed or single-colour tile still communicates correctly that way.
 *
 * Quick start:
 * @code
 *   #include "core_tiles.h"
 *
 *   tile_t pmic;
 *   tile_power_l_1n_init(core_tiles_pal(&core_i2c1), 0, &pmic, NULL);
 *   if (tile_is_ready(&pmic)) {
 *       tile_power_l_1n_set_charge_current_ma(&pmic, 200);
 *       tile_power_l_1n_charger_enable(&pmic, 1);
 *       uint16_t vbat = tile_power_l_1n_get_vbat_mv(&pmic);
 *       (void)vbat;
 *   }
 * @endcode
 *
 * Datasheet: Nordic nPM1300 Product Specification v1.1 (4490_483).
 *
 * @studio tile label=Power.L.1N icon=🔋
 * @studio event name=charge_complete mask=NPM1300_CHG_COMPLETED
 * @studio event name=charging mask=NPM1300_CHG_CONSTANTCURRENT
 *
 * Unsupported capabilities (gating noted per item — a HARDWARE gap means the
 * chip can do it but the tile doesn't wire the pin out, NOT a driver omission):
 *
 * @studio unsupported severity=advanced category="NTC battery thermistor"
 *   Hardware-gated (not a driver gap). The nPM1300 can sense battery temperature
 *   via an NTC thermistor for JEITA charge regulation. NTC is tied to GND on
 *   Power-L-1N-a (no thermistor fitted), so the driver disables NTC monitoring
 *   (ADCNTCRSEL=0, DISABLENTC). Closing requires a tile hardware revision.
 *
 * @studio unsupported severity=niche category="Load switches / LDOs (LSIN/LSOUT)"
 *   Hardware-gated (not a driver gap). The nPM1300's two load-switch/LDO outputs
 *   are tied off (LSIN1/2, LSOUT1/2 grounded) on this tile, so there's nothing
 *   to control.
 *
 * @studio unsupported severity=niche category="GPIO0-3"
 *   Hardware-gated (not a driver gap). The nPM1300 GPIOs aren't routed to tile
 *   pads on Power-L-1N-a, so there's no externally useful GPIO function.
 *
 * @studio unsupported severity=advanced category="Ship / hibernate mode"
 *   Driver-deferred. The chip supports ultra-low-power ship (370 nA) and
 *   hibernate modes that isolate the battery for storage (datasheet §7.4:
 *   TASKENTERSHIPMODE 0x0B02, TASKENTERHIBERNATE 0x0B00). Not yet exposed. The
 *   SHPHLD wake button is not routed to a pad on Power-L-1N-a, so the only way
 *   back out of ship mode on this tile is applying CHG (VBUS), which needs a
 *   bench check before it is offered as a Studio call.
 *
 * @studio unsupported severity=niche category="POF warning + buck retention / forced-PWM"
 *   Driver-deferred. Power-fail early-warning (via GPIO) and buck
 *   retention-voltage / forced-PWM modes aren't exposed yet.
 *
 * @studio unsupported severity=advanced category="USB-C source detection (CC1 / CC2)"
 *   Driver-deferred. The nPM1300 reports what the attached source can supply
 *   (USBCDETECTSTATUS: default / 1.5 A / 3 A) and the tile routes CC1 / CC2 to
 *   pads 2 / 3, but the driver does not read it. set_vbus_limit_ma() therefore
 *   takes the caller's word for what the source can give: 500 mA is safe for
 *   any USB port, more is only safe for a supply known to deliver it.
 */

#ifndef INC_TILE_POWER_L_1N_H_
#define INC_TILE_POWER_L_1N_H_

#include "tiles.h"
#include <stdint.h>

/* -------------------------------------------------------------- */
/* Driver version                                                  */
/* -------------------------------------------------------------- */

#define TILE_POWER_L_1N_VERSION_MAJOR  1
#define TILE_POWER_L_1N_VERSION_MINOR  2
#define TILE_POWER_L_1N_VERSION_PATCH  0

TILES_CHECK_VERSION(1, 0);  /* requires tiles.h >= 1.0 */

#define NPM1300_I2C_ADDR                0x6B

/* ── VBUS input (VBUSIN, base 0x02) ────────────────────────────────────────── */
#define NPM1300_REG_TASKUPDATEILIMSW    0x0200  /**< Write 1: VBUSINILIM0 takes effect */
#define NPM1300_REG_VBUSINILIM0         0x0201  /**< Input limit, code = mA / 100 (1-15) */
#define NPM1300_REG_VBUSINILIMSTARTUP   0x0202  /**< Limit VBUS re-plug falls back to, same code */

/* ── charger (BCHARGER, base 0x03) ─────────────────────────────────────────── */
#define NPM1300_REG_BCHGENABLESET       0x0304
#define NPM1300_REG_BCHGENABLECLR       0x0305
#define NPM1300_REG_BCHGDISABLESET      0x0306
#define NPM1300_REG_BCHGISETMSB         0x0308
#define NPM1300_REG_BCHGISETLSB         0x0309
#define NPM1300_REG_BCHGVTERM           0x030C
#define NPM1300_REG_BCHGVTERMR          0x030D
#define NPM1300_REG_BCHGCHARGESTATUS    0x0334
#define NPM1300_REG_BCHGERRREASON       0x0336

/* charge-status bits — the return value of get_charge_status() */
#define NPM1300_CHG_BATTERYDETECTED     (1 << 0)  /**< Battery detected             */
#define NPM1300_CHG_COMPLETED           (1 << 1)  /**< Charge complete (full)       */
#define NPM1300_CHG_TRICKLECHARGE       (1 << 2)  /**< Trickle-charge phase         */
#define NPM1300_CHG_CONSTANTCURRENT     (1 << 3)  /**< Constant-current phase       */
#define NPM1300_CHG_CONSTANTVOLTAGE     (1 << 4)  /**< Constant-voltage phase       */
#define NPM1300_CHG_RECHARGE            (1 << 5)  /**< Recharge (top-up) in progress*/
#define NPM1300_CHG_DIETEMPHIGHPAUSED   (1 << 6)  /**< Paused: die too hot          */
#define NPM1300_CHG_SUPPLEMENTACTIVE    (1 << 7)  /**< Battery supplementing VSYS    */

/* ── bucks (BUCK, base 0x04) ───────────────────────────────────────────────── */
#define NPM1300_REG_BUCK1ENASET         0x0400
#define NPM1300_REG_BUCK1ENACLR         0x0401
#define NPM1300_REG_BUCK2ENASET         0x0402
#define NPM1300_REG_BUCK2ENACLR         0x0403
#define NPM1300_REG_BUCK1NORMVOUT       0x0408
#define NPM1300_REG_BUCK2NORMVOUT       0x040A
#define NPM1300_REG_BUCKSWCTRLSEL       0x040F  /* bit0=BUCK1, bit1=BUCK2: 1=use NORMVOUT, not VSET pin */

/* ── ADC (base 0x05) ───────────────────────────────────────────────────────── */
#define NPM1300_REG_TASKVBATMEASURE     0x0500
#define NPM1300_REG_TASKTEMPMEASURE     0x0502
#define NPM1300_REG_TASKVSYSMEASURE     0x0503
#define NPM1300_REG_ADCCONFIG           0x0509
#define NPM1300_REG_ADCNTCRSEL          0x050A
#define NPM1300_REG_ADCVBATRESULTMSB    0x0511
#define NPM1300_REG_ADCTEMPRESULTMSB    0x0513
#define NPM1300_REG_ADCVSYSRESULTMSB    0x0514
#define NPM1300_REG_ADCGP0RESULTLSBS    0x0515  /* packed low 2 bits: [1:0]VBAT [3:2]NTC [5:4]TEMP [7:6]VSYS */

/* ── LED drivers (base 0x0A) ───────────────────────────────────────────────── */
#define NPM1300_REG_LEDDRV0MODESEL      0x0A00
#define NPM1300_REG_LEDDRV1MODESEL      0x0A01
#define NPM1300_REG_LEDDRV2MODESEL      0x0A02
#define NPM1300_REG_LEDDRV0SET          0x0A03
#define NPM1300_REG_LEDDRV0CLR          0x0A04
#define NPM1300_REG_LEDDRV1SET          0x0A05
#define NPM1300_REG_LEDDRV1CLR          0x0A06
#define NPM1300_REG_LEDDRV2SET          0x0A07
#define NPM1300_REG_LEDDRV2CLR          0x0A08

#define NPM1300_REG_RSTCAUSE            0x0E03

/** LED indicator mode (LEDDRVxMODESEL). */
typedef enum {
    NPM1300_LED_ERROR    = 0,  /**< auto: on for charger error        */
    NPM1300_LED_CHARGING = 1,  /**< auto: on while charging           */
    NPM1300_LED_HOST     = 2,  /**< software-controlled via led_set() */
    NPM1300_LED_NOTUSED  = 3,  /**< disabled                          */
} power_l_1n_led_mode_t;

/** Optional init config (pass NULL for defaults: 500 mA USB input limit, 100 mA
 *  charge, 4.20 V, charging on). New fields go at the END, so an existing
 *  initialiser keeps meaning what it meant. */
typedef struct {
    uint16_t charge_current_ma;  /**< 32-800 mA. 0 = default (100 mA).     */
    uint16_t term_mv;            /**< 3500-4450 mV. 0 = default (4200 mV). */
    uint8_t  enable_charging;    /**< 0 = leave charging off, 1 = enable.  */
    uint16_t vbus_limit_ma;      /**< 100-1500 mA. 0 = default (500 mA). Use 100
                                  *   for a source that has not granted more. */
} power_l_1n_cfg_t;

/**
 * @brief  Check whether an nPM1300 is present on the I2C bus.
 *
 * @param  hal       Platform HAL handle
 * @param  instance  Instance index (0 = default, see mapping table)
 * @return 1 if the device ACKs, 0 otherwise
 */
uint8_t tile_power_l_1n_find(tiles_pal_t* hal, uint8_t instance);

/**
 * @brief  Initialize the nPM1300 PMIC.
 *
 * Sets the indicator LED modes (LED0 red = ERROR, LED1 orange = CHARGING,
 * LED2 green = HOST),
 * disables NTC monitoring (no thermistor on this tile), and applies the
 * charger settings. The buck regulators are left at their boot voltages
 * (1.8 V / 3.3 V, fixed by the VSET resistors) — they are already up.
 * Pass cfg=NULL for defaults (500 mA USB limit, 100 mA, 4.20 V, charging
 * enabled). A zero-initialised cfg is NOT the same as NULL: it leaves
 * charging OFF (enable_charging = 0).
 *
 * @warning The NULL defaults suit a single Li-ion / Li-poly cell rated
 * 4.20 V and at least ~100 mAh (100 mA is 1C for a 100 mAh cell). They are
 * NOT safe for a LiFePO4 cell (charge to 3.60 V: pass term_mv = 3600) or for
 * a smaller cell (lower charge_current_ma to <= 1C). The chip itself powers
 * up with charging disabled at 3.60 V / 32 mA; init() is what raises it.
 *
 * @param  hal       Platform HAL handle
 * @param  instance  Instance index (0 = default, see mapping table)
 * @param  tile      Pointer to tile handle (populated by this function)
 * @param  cfg       Optional config, or NULL for defaults
 */
void tile_power_l_1n_init(tiles_pal_t* hal, uint8_t instance, tile_t* tile,
                          const power_l_1n_cfg_t* cfg);

/* ── charger ───────────────────────────────────────────────────────────────── */

/**
 * @brief  Enable or disable battery charging.
 * @studio expose category=tile name=charger_enable section=config
 * @studio control on label="Charge the battery" tier=basic type=bool default=1
 * @param  on  1 = enable charging, 0 = disable.
 */
void tile_power_l_1n_charger_enable(tile_t* tile, uint8_t on);

/**
 * @brief  Set the VBUS (USB) input current limit.
 *
 * The nPM1300 powers up with a 100 mA input limit; init() raises it to 500 mA
 * (what any USB port gives) unless the config says otherwise. The limit caps
 * what is drawn from USB for the system AND the charger together. It does not
 * regulate VSYS: with a battery attached, asking for more than the limit gives
 * cuts the charge back and lets VSYS fall to just above the battery voltage
 * (measured: a 100 mA charge on the 100 mA limit took VSYS from 5.15 V to
 * ~3.65 V; both bucks held, but with only ~350 mV of headroom on 3.3 V, which a
 * load with a start-up surge can dip). With no battery, VSYS simply sags.
 *
 * The chip falls back to its START-UP limit whenever VBUS is removed, which
 * matters for exactly the system that charges: one with a battery keeps running
 * through an unplug, and would otherwise be back at 100 mA on re-plug. So this
 * writes the start-up limit too, and the value holds until the chip is RESET
 * (when it returns to 100 mA, and init() sets it again).
 *
 * Settable 100-1500 mA in 100 mA steps; a request is rounded DOWN to a step
 * (never more than asked) and clamped to that range. 100 and 500 mA are the
 * USB-compliant, accurately trimmed levels; 500 mA is safe for any USB port.
 *
 * @studio expose category=tile name=set_vbus_limit_ma section=config
 * @studio control ma label="USB input limit" tier=basic default=500
 * @param  ma  [100..1500] mA Input current limit, in milliamps.
 */
void tile_power_l_1n_set_vbus_limit_ma(tile_t* tile, uint16_t ma);

/**
 * @brief  Set the constant-current charge level.
 *
 * Programmable 32-800 mA in 2 mA steps (rounded down); out-of-range values
 * clamp. The chip only accepts a new current while the charger is disabled
 * (datasheet §6.2.4), so if charging is on this pauses it for the write and
 * re-enables it. Keep it at or below 1C for the cell fitted.
 *
 * @studio expose category=tile name=set_charge_current_ma section=config
 * @studio control ma label="Charge current" tier=basic default=100
 * @param  ma  [32..800] mA Charge current in milliamps.
 */
void tile_power_l_1n_set_charge_current_ma(tile_t* tile, uint16_t ma);

/**
 * @brief  Set the charge-termination (full) voltage.
 *
 * Selectable 3.50-3.65 V or 4.00-4.45 V in 50 mV steps; the 3.70-3.95 V
 * gap is not supported and clamps to 3.65 V.
 *
 * @studio expose category=tile name=set_term_mv section=config
 * @studio control mv label="Charge to" tier=advanced default=4200 scale=0.001 unit=V
 * @param  mv  [3500..4450] mV Termination voltage in millivolts (e.g. 4200).
 */
void tile_power_l_1n_set_term_mv(tile_t* tile, uint16_t mv);

/**
 * @brief  Read the raw charge-status register.
 * @studio expose category=tile name=get_charge_status section=runtime
 * @return BCHGCHARGESTATUS bits (NPM1300_CHG_* mask).
 */
uint8_t tile_power_l_1n_get_charge_status(tile_t* tile);

/**
 * @brief  Read the charger error-reason register.
 * @studio expose category=tile name=get_charge_error section=runtime
 * @return BCHGERRREASON bits (0 = no error).
 */
uint8_t tile_power_l_1n_get_charge_error(tile_t* tile);

/**
 * @brief  Whether the charger is actively charging (trickle / CC / CV).
 * @studio expose category=tile name=is_charging returns=bool section=runtime
 * @return 1 if charging, 0 otherwise.
 */
uint8_t tile_power_l_1n_is_charging(tile_t* tile);

/**
 * @brief  Whether charging has completed (battery full).
 * @studio expose category=tile name=is_charge_complete returns=bool section=runtime
 * @return 1 if charge complete, 0 otherwise.
 */
uint8_t tile_power_l_1n_is_charge_complete(tile_t* tile);

/**
 * @brief  Whether a battery is detected.
 *
 * @warning Only meaningful while charging is ENABLED. This reads
 * BATTERYDETECTED in BCHGCHARGESTATUS, a charger-domain bit: the datasheet
 * states "CHARGER waits until a battery is detected before charging", i.e.
 * detection runs as part of the charge cycle. With the charger disabled the
 * bit reads 0 whether or not a cell is fitted — measured on the bench with a
 * battery physically attached (0 before enable, 1 after).
 *
 * Do NOT gate a charge-enable decision on this; that deadlocks. Enable
 * charging first, poll this for a second or two, and disable again if nothing
 * appears. Note also that get_vbat_mv() cannot substitute: an open battery
 * terminal reads a plausible ~3 V once charger current has pushed the VBAT
 * decoupling cap up.
 *
 * @studio expose category=tile name=battery_present returns=bool section=runtime
 * @return 1 if a battery is present, 0 otherwise.
 */
uint8_t tile_power_l_1n_battery_present(tile_t* tile);

/* ── measurements (ADC) ────────────────────────────────────────────────────── */

/**
 * @brief  Measure the battery voltage.
 * @studio expose category=tile name=get_vbat_mv section=runtime
 * @return VBAT in millivolts (0-5000).
 */
uint16_t tile_power_l_1n_get_vbat_mv(tile_t* tile);

/**
 * @brief  Measure the system (VSYS) voltage.
 * @studio expose category=tile name=get_vsys_mv section=runtime
 * @return VSYS in millivolts (0-6375).
 */
uint16_t tile_power_l_1n_get_vsys_mv(tile_t* tile);

/**
 * @brief  Measure the PMIC die temperature.
 * @studio expose category=tile name=get_die_temp_c section=runtime
 * @return Die temperature in degrees Celsius.
 */
int16_t  tile_power_l_1n_get_die_temp_c(tile_t* tile);

/* ── buck regulators (buck = 1 or 2) ───────────────────────────────────────── */

/**
 * @brief  Enable or disable a buck regulator.
 * @studio expose category=tile name=buck_enable section=config
 * @param  buck  1 (VOUT1) or 2 (VOUT2).
 * @param  on    1 = enable, 0 = disable.
 */
void tile_power_l_1n_buck_enable(tile_t* tile, uint8_t buck, uint8_t on);

/**
 * @brief  Set a buck regulator's output voltage.
 *
 * Selects software voltage control for that buck (overriding its VSET
 * resistor) and applies the new target.
 *
 * @studio expose category=tile name=buck_set_mv section=config
 * @param  buck  1 (VOUT1) or 2 (VOUT2).
 * @param  mv    [1000..3300] Output voltage in mV, 100 mV steps.
 */
void tile_power_l_1n_buck_set_mv(tile_t* tile, uint8_t buck, uint16_t mv);

/* ── indicator LEDs (led = 0/1/2) ──────────────────────────────────────────── */

/**
 * @brief  Set an indicator LED's drive mode.
 *
 * In auto modes (ERROR/CHARGING) the charger drives the LED directly; HOST
 * mode hands control to led_set().
 *
 * LED0 is red, LED1 orange, LED2 green (see the file header). init() leaves
 * LED2 in HOST mode for firmware status.
 *
 * @studio expose category=tile name=led_set_mode section=config
 * @param  led   0, 1 or 2.
 * @param  mode  power_l_1n_led_mode_t.
 */
void tile_power_l_1n_led_set_mode(tile_t* tile, uint8_t led, power_l_1n_led_mode_t mode);

/**
 * @brief  Turn a host-controlled LED on or off.
 *
 * Only effective when the LED is in HOST mode (see led_set_mode).
 *
 * @studio expose category=tile name=led_set section=runtime
 * @param  led  0, 1 or 2.
 * @param  on   1 = on, 0 = off.
 */
void tile_power_l_1n_led_set(tile_t* tile, uint8_t led, uint8_t on);

/**
 * @brief  Read the reset-cause register (why the PMIC last reset).
 * @studio expose category=tile name=get_reset_cause section=advanced
 * @return RSTCAUSE bits.
 */
uint8_t tile_power_l_1n_get_reset_cause(tile_t* tile);

#endif /* INC_TILE_POWER_L_1N_H_ */
