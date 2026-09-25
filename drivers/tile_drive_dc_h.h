/**
 * @file   tile_drive_dc_h.h
 * @brief  H-bridge DC motor driver for the Drive.DC.H tile (rev a).
 *
 * Embeds the TI DRV8214, a full H-bridge motor driver with integrated
 * current sensing, stall detection, and voltage/speed regulation.
 *
 * Key specifications:
 *   - Output:        full H-bridge, 4 A peak / 2 A RMS continuous
 *   - Supply:        1.65-11 V motor supply (VM), 1.65-5.5 V logic (VCC)
 *   - R_DS(on):      240 mOhm (high + low side total)
 *   - Regulation:    voltage or speed, via internal PI loop
 *   - Current sense: internal current mirror (IPROPI), 3 gain settings
 *   - Stall detect:  hardware, configurable via IPROPI/VREF threshold
 *   - Ripple count:  sensorless position/speed via commutation ripples
 *   - Protection:    UVLO, OCP, OVP, thermal shutdown
 *
 * The tile also exposes EN/PH (or IN1/IN2) control pads (pads 2, 3) for
 * direct GPIO/PWM-based motor control without I²C, and an analog
 * current-proportional output on pad 6 (NPROP/IPROPI) that mirrors the
 * motor winding current scaled by the CS_GAIN_SEL setting. Switch the
 * bridge control source at runtime with set_control_mode().
 *
 * The DRV8214 integrates a sensorless ripple counting algorithm
 * that counts commutation ripples in the motor current waveform
 * to estimate rotor position and speed without external encoders
 * or Hall sensors. In speed regulation mode (DRIVE_DC_H_MODE_SPEED),
 * the chip's internal PI loop maintains constant motor speed across
 * varying supply voltages and loads. The ripple count and estimated
 * speed can be read at any time via get_ripple_count() / get_speed().
 * Accurate ripple counting requires motor-specific tuning of the
 * INV_R, KMC, and filter parameters (see DRV8214 datasheet
 * section 9.2.3.1).
 *
 * Datasheet: https://www.bergsonne.io/tiles/drive/dc-h
 * IC datasheet: https://www.ti.com/lit/ds/symlink/drv8214.pdf
 *
 * @studio tile label=Drive.DC.H icon=⚙
 *
 * Quick start:
 * @code
 *   #include "core_tiles.h"
 *
 *   tile_t motor;
 *   tile_drive_dc_h_init(core_tiles_pal(&core_i2c3), 0, &motor, NULL);
 *   if (tile_is_ready(&motor)) {
 *       tile_drive_dc_h_forward(&motor);   // drive forward
 *       core_delay_ms(2000);
 *       tile_drive_dc_h_brake(&motor);     // slow-decay brake
 *   }
 * @endcode
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=advanced category="nSLEEP pin (hardware-gated)" section=advanced
 *   Chip's nSLEEP pin (~100 nA quiescent in sleep, ~1.3 mA active)
 *   is not routed to any tile pad in the Drive.DC.H rev-a layout
 *   (verify in definitions/Drive-DC-H-a.json — pads 1–10 are
 *   GND, EN/IN1, PH/IN2, I²C.CLK, I²C.DAT, NPROP, OUT1, OUT2, VM,
 *   V+; nSLEEP is strapped active on the PCB). sleep()/wake() only
 *   toggle the I²C-side EN_OUT bit; the chip itself can't reach its
 *   true quiescent floor. Closing this gap requires a tile hardware
 *   revision that routes nSLEEP to a connector pad.
 *
 * @studio unsupported severity=niche category="Address selection (factory variants)"
 *   The DRV8214 derives its 7-bit I²C address from tri-level pins
 *   A0/A1, but the Drive.DC.H tile straps these to fixed values per
 *   factory variant: A30 (0x30), A31 (0x31), A33 (0x33), A34 (0x34
 *   default). Address is chip-gated at order time, not runtime-
 *   settable. The 9-entry instance table in this driver covers the
 *   full chip address space, but only those four straps are sold —
 *   pick a tile variant per Drive.DC.H tile sharing a bus.
 */

#ifndef INC_TILE_DRIVE_DC_H_H_
#define INC_TILE_DRIVE_DC_H_H_

#include "tiles.h"
#include <stdint.h>

/* -------------------------------------------------------------- */
/* Driver version                                                  */
/* -------------------------------------------------------------- */

#define TILE_DRIVE_DC_H_VERSION_MAJOR  4
#define TILE_DRIVE_DC_H_VERSION_MINOR  3
#define TILE_DRIVE_DC_H_VERSION_PATCH  0

TILES_CHECK_VERSION(1, 0);  /* requires tiles.h >= 1.0 */

/* -------------------------------------------------------------- */
/* Instance mapping                                                */
/* -------------------------------------------------------------- */

/**
 * @brief  Instance-to-address mapping for Drive.DC.H.
 *
 * The DRV8214 derives its I2C address from tri-level pins A0 and A1
 * (tied to VCC, GND, or left floating). Up to 9 devices per bus.
 *
 * | Instance | A1   | A0   | 7-bit addr |
 * |----------|------|------|------------|
 * | 0        | Hi-Z | Hi-Z | 0x34       |
 * | 1        | 0    | 0    | 0x30       |
 * | 2        | 0    | Hi-Z | 0x31       |
 * | 3        | 0    | 1    | 0x32       |
 * | 4        | Hi-Z | 0    | 0x33       |
 * | 5        | Hi-Z | 1    | 0x35       |
 * | 6        | 1    | 0    | 0x36       |
 * | 7        | 1    | Hi-Z | 0x37       |
 * | 8        | 1    | 1    | 0x38       |
 */
#define DRV8214_I2C_ADDR_DEFAULT    0x34

/* -------------------------------------------------------------- */
/* DRV8214 register map                                            */
/* -------------------------------------------------------------- */

/* Status registers (read-only) */
#define DRV8214_REG_FAULT           0x00  /**< Fault status */
#define DRV8214_REG_RC_STATUS1      0x01  /**< Ripple speed estimate */
#define DRV8214_REG_RC_STATUS2      0x02  /**< Ripple count [7:0] */
#define DRV8214_REG_RC_STATUS3      0x03  /**< Ripple count [15:8] */
#define DRV8214_REG_REG_STATUS1     0x04  /**< Motor terminal voltage */
#define DRV8214_REG_REG_STATUS2     0x05  /**< Motor current */
#define DRV8214_REG_REG_STATUS3     0x06  /**< Internal duty cycle */

/* Configuration registers */
#define DRV8214_REG_CONFIG0         0x09  /**< Output/fault enables */
#define DRV8214_REG_CONFIG1         0x0A  /**< TINRUSH [7:0] */
#define DRV8214_REG_CONFIG2         0x0B  /**< TINRUSH [15:8] */
#define DRV8214_REG_CONFIG3         0x0C  /**< IMODE, SMODE, INT_VREF */
#define DRV8214_REG_CONFIG4         0x0D  /**< Bridge ctrl, reporting */

/* Regulation control registers */
#define DRV8214_REG_CTRL0           0x0E  /**< Reg mode, soft-start, PWM */
#define DRV8214_REG_CTRL1           0x0F  /**< WSET_VSET target */
#define DRV8214_REG_CTRL2           0x10  /**< OUT_FLT, EXT_DUTY */

/* Ripple counting control registers */
#define DRV8214_REG_RC_CTRL0        0x11  /**< EN_RC, CS_GAIN_SEL */
#define DRV8214_REG_RC_CTRL1        0x12  /**< RC_THR [7:0] */
#define DRV8214_REG_RC_CTRL2        0x13  /**< INV_R_SCALE, KMC_SCALE, RC_THR_SCALE, RC_THR[9:8] */
#define DRV8214_REG_RC_CTRL3        0x14  /**< INV_R */
#define DRV8214_REG_RC_CTRL4        0x15  /**< KMC */

/** @brief  Expected CONFIG3 register value at power-on. */
#define DRV8214_CONFIG3_DEFAULT     0x63

/* -------------------------------------------------------------- */
/* Fault register bit masks                                        */
/* -------------------------------------------------------------- */

#define DRV8214_FAULT_FAULT         0x80  /**< Bit 7 — any fault active */
#define DRV8214_FAULT_STALL         0x20  /**< Bit 5 — motor stall */
#define DRV8214_FAULT_OCP           0x10  /**< Bit 4 — overcurrent */
#define DRV8214_FAULT_OVP           0x08  /**< Bit 3 — overvoltage */
#define DRV8214_FAULT_TSD           0x04  /**< Bit 2 — thermal shutdown */
#define DRV8214_FAULT_NPOR          0x02  /**< Bit 1 — power-on reset */
#define DRV8214_FAULT_CNT_DONE      0x01  /**< Bit 0 — ripple count done */

/* -------------------------------------------------------------- */
/* Init-time mode selection                                        */
/* -------------------------------------------------------------- */

/** Voltage regulation — internal PI loop maintains target motor voltage.
 *  WSET_VSET sets the target; VM_GAIN_SEL selects the voltage range.
 *  Ripple counting is disabled. */
#define DRIVE_DC_H_MODE_VOLTAGE      0

/** Speed regulation — internal PI loop maintains target motor speed
 *  using the ripple counting algorithm. Requires motor_mohm,
 *  ripples_per_rev, and kv_uv_per_rpm in the config struct. */
#define DRIVE_DC_H_MODE_SPEED        1

/** Voltage regulation with ripple counting enabled. Motor runs at the
 *  target voltage while counting commutation ripples for position
 *  tracking. Use get_ripple_count() to read position, get_speed() for
 *  speed estimate. Provide motor_mohm and ripples_per_rev for accurate
 *  counting; kv_uv_per_rpm improves speed estimation but is optional. */
#define DRIVE_DC_H_MODE_RIPPLE_COUNT 2

/** External pad control in PH/EN mode. The DRV8214 is configured
 *  via I2C (EN_OUT=1, PMODE=0, I2C_BC=0) then the motor is
 *  controlled entirely through the EN and PH pads:
 *    - EN (tile pad 2): PWM for speed control, HIGH=drive, LOW=brake
 *    - PH (tile pad 3): direction — HIGH=forward, LOW=reverse
 *  After init, the I2C bridge control bits are ignored. Motor
 *  monitoring (get_voltage_mv, get_current_ma, get_fault, etc.)
 *  still works over I2C. forward/reverse/brake/coast functions
 *  are NOT available in this mode — use GPIO/PWM instead. */
#define DRIVE_DC_H_MODE_PAD_PHEN     3

/** External pad control in IN1/IN2 mode. Same as PAD_PHEN but
 *  PMODE=1 (PWM mode), so each pad drives one half-bridge:
 *    - IN1 (tile pad 2): PWM for OUT1 (HIGH = OUT1 high)
 *    - IN2 (tile pad 3): PWM for OUT2 (HIGH = OUT2 high)
 *  Both 0 = coast, both 1 = brake, complementary = drive. */
#define DRIVE_DC_H_MODE_PAD_IN1IN2   4

/* -------------------------------------------------------------- */
/* Runtime mode setters (control modes / regulation / current /    */
/* stall — closes the corresponding driver gaps)                   */
/* -------------------------------------------------------------- */

/** Bridge control source — value passed to set_control_mode().
 *  Mirrors the four init-time modes but switchable at runtime.   */
typedef enum {
    DRIVE_DC_H_CTRL_I2C        = 0,  /**< I²C registers control bridge (PWM mode). */
    DRIVE_DC_H_CTRL_PAD_PHEN   = 1,  /**< Pads 2/3 = EN/PH (one PWM + dir). */
    DRIVE_DC_H_CTRL_PAD_IN1IN2 = 2,  /**< Pads 2/3 = IN1/IN2 (independent half-bridge PWM). */
} drive_dc_h_control_mode_t;

/** Regulation loop selection — value passed to set_regulation_mode().
 *  Maps to REG_CTRL[1:0] in REG_CTRL0 (offset 0x0E).             */
typedef enum {
    DRIVE_DC_H_REG_OPEN_LOOP      = 0,  /**< 00b: fixed-off-time current reg only (open-loop drive). */
    DRIVE_DC_H_REG_CYCLE_BY_CYCLE = 1,  /**< 01b: cycle-by-cycle current reg. */
    DRIVE_DC_H_REG_SPEED          = 2,  /**< 10b: speed regulation (requires ripple counting). */
    DRIVE_DC_H_REG_VOLTAGE        = 3,  /**< 11b: voltage regulation (default). */
} drive_dc_h_reg_mode_t;

/** Current regulation mode — value passed to set_current_regulation_mode().
 *  Maps to IMODE[1:0] in CONFIG3 (offset 0x0C).                   */
typedef enum {
    DRIVE_DC_H_IMODE_DISABLED     = 0,  /**< 00b: no current regulation. */
    DRIVE_DC_H_IMODE_INRUSH       = 1,  /**< 01b: regulate during tINRUSH only (default for I²C). */
    DRIVE_DC_H_IMODE_ALWAYS       = 2,  /**< 10b: always regulate at ITRIP threshold. */
    DRIVE_DC_H_IMODE_DISABLED_ALT = 3,  /**< 11b: disabled (alt encoding). */
} drive_dc_h_imode_t;

/** Stall recovery behavior — value passed to set_stall_recovery().
 *  Maps to SMODE bit in CONFIG3 (offset 0x0C, bit 5).             */
typedef enum {
    DRIVE_DC_H_STALL_LATCH    = 0,  /**< 0b: latch outputs off on stall. */
    DRIVE_DC_H_STALL_REPORT   = 1,  /**< 1b: report only, keep driving (default). */
} drive_dc_h_stall_recovery_t;

/* -------------------------------------------------------------- */
/* Public API                                                      */
/* -------------------------------------------------------------- */

/**
 * @brief  Check whether a DRV8214 is present on the I2C bus.
 *
 * @param  hal       Platform HAL handle
 * @param  instance  Instance index (0-8, see mapping table)
 * @return 1 if device ACKs, 0 otherwise
 */
uint8_t tile_drive_dc_h_find(tiles_pal_t* hal, uint8_t instance);

/**
 * Optional init config. Pass NULL for defaults:
 *   - Voltage regulation mode (no ripple counting)
 *   - 0-3.92 V range (VM_GAIN_SEL=1)
 *   - 4 A max current sense (CS_GAIN_SEL=0)
 *   - Full target voltage (WSET_VSET=0xFF)
 *   - Soft-start enabled, 25 kHz PWM
 *   - I2C bridge control, PWM mode
 *
 * For ripple counting, set mode to RIPPLE_COUNT or SPEED and
 * provide motor parameters. The driver computes register values
 * (INV_R, KMC, scaling factors) automatically.
 */
typedef struct {
    uint8_t  mode;     /**< DRIVE_DC_H_MODE_VOLTAGE (0), _SPEED (1),
                            _RIPPLE_COUNT (2), _PAD_PHEN (3),
                            or _PAD_IN1IN2 (4). */
    uint8_t  vm_gain;  /**< Voltage range: 0 = 0-15.7 V, 1 = 0-3.92 V.
                            Use 1 for better resolution at low voltages. */
    uint8_t  cs_gain;  /**< Current sense gain (CS_GAIN_SEL, 0-5):
                            0=4A, 1=2A, 2=1A, 3=0.5A, 4=0.25A, 5=0.125A.
                            Lower gain = lower R_DS(on) = higher efficiency. */
    uint8_t  target;   /**< WSET_VSET: target voltage or speed (0-255).
                            In voltage mode with VM_GAIN_SEL=1:
                              WSET_VSET = target_mV * 255 / 3920.
                            0xFF = full scale. */

    /* ---- Ripple counting (set motor_mohm > 0 to enable tuning) ---- */
    uint16_t motor_mohm;       /**< Motor winding resistance in milliohms.
                                    0 = skip ripple tuning (use chip defaults).
                                    Typical small DC motor: 1000-50000 mohm. */
    uint8_t  ripples_per_rev;  /**< Commutation ripples per revolution.
                                    = LCM(brushes, commutator_segments).
                                    Common values: 3, 5, 6, 7, 12.
                                    0 = default (12). */
    uint16_t kv_uv_per_rpm;    /**< Back-EMF constant in uV/RPM.
                                    Improves speed estimation accuracy.
                                    0 = skip KMC tuning.
                                    Typical small motor: 100-2000 uV/RPM. */
} drive_dc_h_cfg_t;

/**
 * @brief  Initialize the DRV8214 motor driver.
 *
 * Verifies device presence, configures I2C bridge control in PWM
 * mode, sets regulation parameters, and enables the output stage
 * in coast state (motor not spinning). Call forward() or reverse()
 * to start the motor.
 *
 * @param  hal       Platform HAL handle
 * @param  instance  Instance index (0-8, see mapping table)
 * @param  tile      Pointer to tile handle (populated by this function)
 * @param  cfg       Optional config, or NULL for defaults
 */
void tile_drive_dc_h_init(tiles_pal_t* hal, uint8_t instance, tile_t* tile,
                          const drive_dc_h_cfg_t *cfg);

/* ---- Motor control ---- */

/**
 * @brief  Drive motor forward (OUT1=H, OUT2=L).
 * @studio expose category=tile name=forward section=runtime
 *
 */
void tile_drive_dc_h_forward(tile_t* tile);

/**
 * @brief  Drive motor in reverse (OUT1=L, OUT2=H).
 * @studio expose category=tile name=reverse section=runtime
 *
 */
void tile_drive_dc_h_reverse(tile_t* tile);

/**
 * @brief  Active brake (slow-decay, both low-side FETs on).
 * @studio expose category=tile name=brake section=runtime
 *
 * Motor is actively held. Current recirculates through the
 * low-side FETs, providing strong braking force.
 *
 */
void tile_drive_dc_h_brake(tile_t* tile);

/**
 * @brief  Coast (Hi-Z, all FETs off).
 * @studio expose category=tile name=coast section=runtime
 *
 * Motor freewheels. No braking force is applied. This is the
 * initial state after init().
 *
 */
void tile_drive_dc_h_coast(tile_t* tile);

/* ---- Bridge control source (closes "PMODE / I2C_BC" gap) ---- */

/**
 * @brief  Switch the bridge control source between I²C and the EN/PH
 *         or IN1/IN2 tile pads.
 * @studio expose category=tile name=set_control_mode section=config
 *
 * In I²C mode (default after init), forward/reverse/brake/coast control
 * the bridge. In either pad-control mode, the chip ignores I²C bridge
 * bits and tracks pads 2/3 directly:
 *   - PAD_PHEN     — pad 2 = EN (PWM), pad 3 = PH (direction)
 *   - PAD_IN1IN2   — pad 2 = IN1, pad 3 = IN2 (independent half-bridges)
 * Monitoring (voltage, current, fault, ripple count) keeps working over
 * I²C in all modes. Switching to a pad mode disables I²C bridge calls
 * (forward/reverse/etc) — they'll log an error and no-op.
 *
 * @param  mode  Bridge control source (DRIVE_DC_H_CTRL_*)
 */
void tile_drive_dc_h_set_control_mode(tile_t* tile,
                                      drive_dc_h_control_mode_t mode);

/* ---- Regulation ---- */

/**
 * @brief  Set the regulation target (WSET_VSET).
 * @studio expose category=tile name=set_target section=runtime
 *
 * In voltage mode: sets target motor terminal voltage.
 *   With VM_GAIN_SEL=0: value * 15700/255 mV (61.6 mV/bit)
 *   With VM_GAIN_SEL=1: value * 3920/255 mV  (15.4 mV/bit)
 *
 * In speed mode: sets target ripple speed.
 *   Target ripple speed = value * W_SCALE rad/s (W_SCALE 16/32/64/128;
 *   shaft RPM = that × 60 / (2π × ripples per rev)).
 *
 * @param  value  Target setpoint (0-255)
 */
void tile_drive_dc_h_set_target(tile_t* tile, uint8_t value);

/**
 * @brief  Select the regulation loop (open-loop / current / voltage / speed).
 * @studio expose category=tile name=set_regulation_mode section=config
 * @studio control mode label="Regulation" tier=advanced default=DRIVE_DC_H_REG_VOLTAGE
 *
 * Writes REG_CTRL bits in REG_CTRL0. After switching, set_target() is
 * interpreted in the new loop's units (voltage code, speed code, etc.).
 * Speed mode requires ripple counting to be enabled — the driver
 * automatically sets EN_RC=1 when SPEED is selected.
 *
 * @note  REG_CTRL is only writable with the outputs off (datasheet
 *        Table 8-29), so this briefly clears EN_OUT around the write.
 *
 * @param  mode  Regulation mode (DRIVE_DC_H_REG_*)
 */
void tile_drive_dc_h_set_regulation_mode(tile_t* tile,
                                         drive_dc_h_reg_mode_t mode);

/* ---- Current regulation (closes "IMODE / ITRIP" gap) ---- */

/**
 * @brief  Set the current-regulation mode (IMODE in CONFIG3).
 * @studio expose category=tile name=set_current_regulation_mode section=config
 *
 * Selects when the chip's internal current loop folds back to keep
 * motor current under the ITRIP threshold. ITRIP itself is set by
 * the combination of CS_GAIN_SEL (set_current_sense_gain) and the
 * internal 500 mV VREF (INT_VREF=1, fixed by this driver since the
 * external VREF pin isn't routed on the Drive.DC.H tile).
 *
 * @param  mode  Current regulation mode (DRIVE_DC_H_IMODE_*)
 */
void tile_drive_dc_h_set_current_regulation_mode(tile_t* tile,
                                                 drive_dc_h_imode_t mode);

/**
 * @brief  Set the current-sense gain / max-current range.
 * @studio expose category=tile name=set_current_sense_gain section=config
 * @studio control code label="Current range (0 = 4 A … 5 = 0.125 A)" tier=advanced default=0
 *
 * Programs CS_GAIN_SEL[2:0] in RC_CTRL0. Lower max-current ranges
 * give finer current resolution but lower R_DS(on) headroom; higher
 * ranges support larger motors but coarser get_current_ma() steps.
 * Codes:
 *   0 = 4 A,    1 = 2 A,    2 = 1 A,
 *   3 = 0.5 A,  4 = 0.25 A, 5 = 0.125 A
 * Also affects ITRIP when current regulation is enabled.
 *
 * @param  code  [0..5] CS_GAIN_SEL code
 */
void tile_drive_dc_h_set_current_sense_gain(tile_t* tile, uint8_t code);

/* ---- Stall detection (closes "EN_STALL / TINRUSH / SMODE" gap) ---- */

/**
 * @brief  Enable or disable hardware stall detection.
 * @studio expose category=tile name=set_stall_enabled section=config
 * @studio control enabled label="Stall detection" tier=advanced type=bool default=1
 *
 * Toggles EN_STALL in CONFIG0. When disabled, the STALL bit in the
 * fault register won't latch and is_stalled() always returns 0.
 * Useful while tuning the inrush time for a new motor.
 *
 * @param  enabled  1 = stall detection on, 0 = off
 */
void tile_drive_dc_h_set_stall_enabled(tile_t* tile, uint8_t enabled);

/**
 * @brief  Set the inrush blanking time (TINRUSH).
 * @studio expose category=tile name=set_inrush_time_ms section=config
 * @studio control ms label="Inrush time" tier=advanced default=100 unit=ms
 *
 * Programs CONFIG1/CONFIG2 with a 16-bit count of 102.4 µs ticks,
 * giving up to ~6.7 s. During the blanking window after a drive
 * command, the stall detector ignores motor current — necessary
 * because real motors draw several × steady-state during startup.
 * Tune for the slowest motor you want to drive.
 *
 * @note  With soft-start on (EN_SS=1, init's default) and voltage or
 *        speed regulation active, the chip stretches the window to
 *        TINRUSH × WSET_VSET (datasheet §8.3.7.3.1): 100 ms at
 *        WSET_VSET=255 is a ~25 s ramp.
 *
 * @param  ms    [0..6710] ms Inrush blanking time
 */
void tile_drive_dc_h_set_inrush_time_ms(tile_t* tile, uint16_t ms);

/**
 * @brief  Set the stall-detection recovery behavior (SMODE in CONFIG3).
 * @studio expose category=tile name=set_stall_recovery section=config
 *
 * In LATCH mode, hitting a stall turns the bridge off until either
 * clear_fault() is called or the chip is power-cycled. In REPORT mode
 * the chip flags STALL but keeps driving — useful for haptics or
 * actuators that legitimately stall against an end-stop.
 *
 * @param  mode  DRIVE_DC_H_STALL_LATCH or DRIVE_DC_H_STALL_REPORT
 */
void tile_drive_dc_h_set_stall_recovery(tile_t* tile,
                                        drive_dc_h_stall_recovery_t mode);

/* ---- Ripple-counter tuning (closes "Ripple counter tuning" gap) ---- */

/**
 * @brief  Set the ripple-count threshold that fires CNT_DONE.
 * @studio expose category=tile name=set_ripple_threshold section=config
 *
 * The chip raises CNT_DONE when the running ripple count reaches
 * (count × scale). Useful for "rotate N counts then stop" patterns:
 * watch the CNT_DONE bit in get_fault(), then call coast(). Internal
 * threshold is 10-bit; the driver picks the smallest scale (×2, ×8,
 * ×16, or ×64) that fits `count`.
 *
 * @param  count  Threshold count (0 - 65472, larger = coarser scale)
 */
void tile_drive_dc_h_set_ripple_threshold(tile_t* tile, uint16_t count);

/**
 * @brief  Set the ripple filter input scaling factor.
 * @studio expose category=tile name=set_ripple_filter_gain section=config
 *
 * Programs FLT_GAIN_SEL[1:0] in RC_CTRL0. Scales the magnitude of
 * detected ripples before the counter — increase if get_ripple_count()
 * undercounts, decrease if it spuriously counts noise. Codes:
 *   0 = ×2,  1 = ×4 (default),  2 = ×8,  3 = ×16
 *
 * @param  code  FLT_GAIN_SEL code (0-3)
 */
void tile_drive_dc_h_set_ripple_filter_gain(tile_t* tile, uint8_t code);

/* ---- Monitoring ---- */

/**
 * @brief  Read the raw FAULT register (0x00).
 * @studio expose category=tile name=get_fault returns=int section=runtime
 *
 * Contains FAULT[7], STALL[5], OCP[4], OVP[3], TSD[2],
 * NPOR[1], CNT_DONE[0]. Use the DRV8214_FAULT_* masks to
 * decode individual bits.
 *
 * @return Raw fault byte (0 = no faults)
 */
uint8_t tile_drive_dc_h_get_fault(tile_t* tile);

/**
 * @brief  Clear all latched faults.
 * @studio expose category=tile name=clear_fault section=runtime
 *
 * Sets CLR_FLT in CONFIG0. Clears FAULT, OCP, OVP, TSD,
 * and NPOR bits. The CLR_FLT bit is self-clearing.
 *
 */
void tile_drive_dc_h_clear_fault(tile_t* tile);

/**
 * @brief  Check if a motor stall condition is active.
 * @studio expose category=tile name=is_stalled returns=bool section=runtime
 *
 * @return 1 if STALL bit is set, 0 otherwise
 */
uint8_t tile_drive_dc_h_is_stalled(tile_t* tile);

/**
 * @brief  Read motor terminal voltage in millivolts.
 * @studio expose category=tile name=get_voltage_mv returns=int section=runtime
 *
 * Reads the VMTR register. The reading is proportional to the
 * voltage across OUT1-OUT2 terminals. Valid while driving.
 *
 * @return Motor voltage in mV (e.g. 3300 = 3.3 V)
 */
uint16_t tile_drive_dc_h_get_voltage_mv(tile_t* tile);

/**
 * @brief  Read motor current in milliamps.
 * @studio expose category=tile name=get_current_ma returns=int section=runtime
 *
 * Reads the IMTR register and converts using the current
 * CS_GAIN_SEL setting. The reading reflects the current
 * flowing through the low-side FETs during drive or brake.
 *
 * @return Motor current in mA
 */
uint16_t tile_drive_dc_h_get_current_ma(tile_t* tile);

/**
 * @brief  Read the ripple speed estimate.
 * @studio expose category=tile name=get_speed returns=int section=runtime
 *
 * Returns the raw SPEED register from the ripple counting
 * algorithm. Value is proportional to motor speed but requires
 * motor-specific calibration for RPM conversion.
 *
 * @return Raw speed estimate (0-255)
 */
uint8_t tile_drive_dc_h_get_speed(tile_t* tile);

/**
 * @brief  Read the estimated motor speed in RPM.
 * @studio expose category=tile name=get_speed_rpm returns=int section=runtime
 *
 * Converts the chip's ripple-speed estimate to shaft RPM using the
 * live W_SCALE setting and the configured ripples-per-rev, making it
 * the exact inverse of the set_speed_rpm() target conversion.
 * Accuracy depends on the motor profile (see set_motor_params());
 * without INV_R/KMC calibration the estimate is approximate.
 *
 * @return Estimated shaft speed in revolutions per minute
 */
uint32_t tile_drive_dc_h_get_speed_rpm(tile_t* tile);

/**
 * @brief  Read the 16-bit ripple count.
 * @studio expose category=tile name=get_ripple_count returns=int section=runtime
 *
 * Returns the total number of commutation ripples counted
 * since the last clear. Proportional to rotor position.
 *
 * @return Ripple count (0-65535)
 */
uint16_t tile_drive_dc_h_get_ripple_count(tile_t* tile);

/**
 * @brief  Reset the ripple counter to zero.
 * @studio expose category=tile name=clear_ripple_count section=runtime
 *
 * Sets CLR_CNT in CONFIG0. Also clears CNT_DONE flag.
 * The CLR_CNT bit is self-clearing.
 *
 */
void tile_drive_dc_h_clear_ripple_count(tile_t* tile);

/* ---- Power management ---- */

/**
 * @brief  Disable the output stage (all FETs Hi-Z).
 * @studio expose category=tile name=sleep section=lifecycle
 *
 * Clears EN_OUT in CONFIG0. The device remains on the I2C bus
 * and registers are accessible, but no current flows through
 * the motor. OVP protection remains active in sleep.
 *
 */
void tile_drive_dc_h_sleep(tile_t* tile);

/**
 * @brief  Re-enable the output stage.
 * @studio expose category=tile name=wake section=lifecycle
 *
 * Sets EN_OUT in CONFIG0. The bridge returns to the last
 * commanded state (coast/brake/forward/reverse).
 *
 */
void tile_drive_dc_h_wake(tile_t* tile);

/* ============================================================== */
/* Runtime — tier-2 idiomatic helpers                              */
/*                                                                  */
/* These compose the tier-1 surface above into "do the thing the   */
/* user wants to do" calls. Direction-aware drive helpers, closed- */
/* loop ripple-counted moves, and run/stop status checks — so      */
/* callers don't need to juggle CONFIG4 bridge bits + RC_STATUS    */
/* polling to do the obvious things.                               */
/* ============================================================== */

/**
 * @brief  Drive direction for the tier-2 helpers.
 *
 * The chip's bridge bits encode forward as IN1=1/IN2=0 and reverse
 * as IN1=0/IN2=1. The enum is symbolic so callers don't need to
 * remember the polarity — see @ref tile_drive_dc_h_set_speed_rpm
 * and @ref tile_drive_dc_h_move_distance.
 */
typedef enum {
    DRIVE_DC_H_DIR_FORWARD = 0,  /**< OUT1=H, OUT2=L (matches forward()). */
    DRIVE_DC_H_DIR_REVERSE = 1,  /**< OUT1=L, OUT2=H (matches reverse()). */
} drive_dc_h_direction_t;

/**
 * @brief  Drive at a target speed (RPM) with closed-loop regulation.
 *
 * @studio expose category=tile name=set_speed_rpm section=runtime
 *
 * Switches the chip into speed-regulation mode (REG_CTRL=10b),
 * writes WSET_VSET so the internal PI loop targets `rpm` for the
 * given motor, and engages the bridge in `direction`. Returns
 * immediately — the chip ramps the motor to the target speed
 * autonomously and is_running() / get_speed() report progress.
 *
 * @note  The driver picks the finest W_SCALE (16/32/64/128 rad/s,
 *        datasheet Table 8-24) whose 8-bit WSET range still reaches
 *        the requested speed, so low targets get the best granularity
 *        the chip offers. W_SCALE is in ripple rad/s (Eq. 8, 11), so
 *        the RPM step is `60 × W_SCALE / (2π × ripples_per_rev)`:
 *        ~12.7 RPM at ripples_per_rev=12 for targets up to ~3 250 RPM,
 *        ~102 RPM per step at the coarsest scale (to ~26 000 RPM).
 *        WSET is clamped at 0xFF for targets beyond the coarsest
 *        scale, and a nonzero request never rounds down to a stop.
 *
 * @note  The conversion depends on the ripples-per-rev value from the
 *        init config (or set_motor_params()), and closed-loop accuracy
 *        depends on the ripple-counter calibration (INV_R, KMC) from
 *        `motor_mohm` / `kv_uv_per_rpm`. If those aren't set, speed
 *        regulation falls back to chip defaults and the RPM target is
 *        approximate. Provide a full motor profile — at init or via
 *        set_motor_params() — for accurate low-RPM control.
 *
 * @param  rpm        Target shaft speed in revolutions per minute
 * @param  direction  DRIVE_DC_H_DIR_FORWARD or DRIVE_DC_H_DIR_REVERSE
 */
void tile_drive_dc_h_set_speed_rpm(tile_t* tile, uint32_t rpm,
                                   drive_dc_h_direction_t direction);

/**
 * @brief  Program the motor profile for ripple counting and speed regulation.
 *
 * @studio expose category=tile name=set_motor_params section=config
 * @studio control ripples_per_rev label="Ripples per revolution" tier=basic default=12
 * @studio control motor_mohm label="Motor winding resistance" tier=advanced default=0 scale=0.001 unit=Ω
 * @studio control kv_uv_per_rpm label="Motor Kv (back-EMF)" tier=advanced default=0 unit=µV/RPM
 *
 * Writes the DRV8214's ripple-counter calibration registers (INV_R,
 * KMC and their scale fields) from physical motor parameters, and
 * caches ripples-per-rev for the set_speed_rpm()/get_speed_rpm()
 * conversions. Equivalent to supplying `motor_mohm`,
 * `ripples_per_rev` and `kv_uv_per_rpm` in the init config — use this
 * when the driver was initialised with defaults (e.g. from Studio)
 * and you want accurate closed-loop speed control.
 *
 * Example (RS PRO 834-7644, direct drive): motor_mohm=6000,
 * ripples_per_rev=12, kv_uv_per_rpm=187.
 *
 * @param  motor_mohm       [0..65535] mΩ Winding resistance. 0 = leave
 *                          chip defaults (no-op for INV_R/KMC).
 * @param  ripples_per_rev  [0..255] Commutation ripples per shaft revolution
 *                          (poles × brush pairs; common: 3, 5, 6, 7,
 *                          12). 0 = default (12); clamped to 255.
 * @param  kv_uv_per_rpm    [0..5000] µV/RPM Back-EMF constant (typical small
 *                          motor: 100-2000). 0 = skip KMC tuning.
 */
void tile_drive_dc_h_set_motor_params(tile_t* tile, uint16_t motor_mohm,
                                      uint16_t ripples_per_rev,
                                      uint16_t kv_uv_per_rpm);

/**
 * @brief  Drive `ripples` commutation ripples in `direction`, then brake.
 *
 * @studio expose category=tile name=move_distance section=runtime
 *
 * Closed-loop step: programs RC_THR for the requested ripple count,
 * clears the ripple counter, engages the bridge in `direction`,
 * then polls until either CNT_DONE fires or the motor stalls. On
 * completion the bridge is put into slow-decay brake — the motor
 * is actively held at the new position, not coasting.
 *
 * @note  This call BLOCKS until the move completes or the motor
 *        stalls. Worst-case wall-time depends on motor speed +
 *        load; budget accordingly. There is no built-in timeout
 *        beyond the chip's own stall detector — use a separate
 *        watchdog or @ref tile_drive_dc_h_wait_for_stop with a
 *        timeout if you need a hard upper bound.
 *
 * @note  On completion the bridge is left in BRAKE (slow decay,
 *        both low-side FETs on). Call @ref tile_drive_dc_h_coast
 *        afterwards if you'd rather let the motor freewheel.
 *
 * @note  Ripple counts are coarse (integer commutation ticks, not
 *        encoder counts). Resolution = 1/ripples_per_rev of a
 *        revolution — typically 1/12 turn. Don't expect sub-degree
 *        positioning.
 *
 * @param  ripples    Number of ripples to advance (0–65472)
 * @param  direction  DRIVE_DC_H_DIR_FORWARD or DRIVE_DC_H_DIR_REVERSE
 */
void tile_drive_dc_h_move_distance(tile_t* tile, uint16_t ripples,
                                   drive_dc_h_direction_t direction);

/**
 * @brief  Check whether the bridge is actively driving the motor.
 *
 * @studio expose category=tile name=is_running returns=bool section=runtime
 *
 * Returns 1 when the bridge is in forward or reverse drive (any
 * non-zero, non-brake combination of IN1/IN2 in I²C bridge mode).
 * Returns 0 when braking, coasting, sleeping, in pad-control mode,
 * or not yet ready.
 *
 * @return 1 if driving, 0 otherwise
 */
uint8_t tile_drive_dc_h_is_running(tile_t* tile);

/**
 * @brief  Block until the ripple counter stops advancing or `timeout_ms` elapses.
 *
 * @studio expose category=tile name=wait_for_stop returns=bool section=runtime
 *
 * Polls the 16-bit ripple counter at ~10 ms cadence. Returns 1
 * when two successive samples (≥30 ms apart) report the same
 * count — i.e., the rotor is mechanically stationary. Returns 0
 * if `timeout_ms` elapses first. Useful after @ref
 * tile_drive_dc_h_brake when you want to know the motor has
 * actually settled rather than just "the bridge has been
 * commanded to brake".
 *
 * @param  timeout_ms  Maximum time to wait, in milliseconds
 * @return 1 if the rotor settled before timeout, 0 on timeout
 */
uint8_t tile_drive_dc_h_wait_for_stop(tile_t* tile, uint32_t timeout_ms);

#endif /* INC_TILE_DRIVE_DC_H_H_ */
