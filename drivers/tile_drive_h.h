/**
 * @file   tile_drive_h.h
 * @brief  LRA/ERM haptic driver for the Drive.H tile (rev a).
 *
 * Embeds the TI DRV2605 (DEVICE_ID 3 — the non-L part), a haptic
 * driver for LRA (Linear Resonant Actuator) and ERM actuators with
 * a built-in waveform library of 123 effects. The actuator is
 * external, connected via the tile's OUT+ / OUT- pads; V_MOTOR
 * accepts 2.5-5.5 V.
 *
 * The chip regulates output amplitude itself: the supply rail only
 * sets headroom. Full-scale drive level comes from RATED_VOLTAGE /
 * OD_CLAMP (see tile_drive_h_set_actuator_voltage()), NOT from the
 * voltage applied to V_MOTOR. V_MOTOR is the DRV2605's only supply
 * (VDD, 2.5-5.5 V); the output cannot exceed it. The datasheet's
 * minimum load is 8 Ω at VDD = 5.2 V.
 *
 * V+ (pad 10) powers nothing but R2, a 100 kΩ pull-up on EN (pad 3).
 * Power V+ from a logic rail, or drive pad 3 high from a Core pin:
 * with neither, EN floats, the chip may stay in shutdown, and init()
 * fails its device-ID check. Holding pad 3 low draws V+ / 100 kΩ
 * (33 µA at 3.3 V). TRIG (pad 2) has a 100 kΩ pull-down (R1). The
 * tile has no I2C pull-ups.
 *
 * Key specifications:
 *   - Output:       full-bridge, amplitude regulated per config
 *   - Waveform lib: 123 haptic effects (6 libraries)
 *   - Smart-loop:   automatic resonance tracking for LRA (closed loop)
 *   - Modes:        Internal trigger, RTP, PWM, audio-to-vibe
 *
 * Datasheet: https://www.bergsonne.io/tiles/drive/h
 * IC datasheet: https://www.ti.com/lit/ds/symlink/drv2605.pdf
 *
 * Quick start:
 * @code
 *   #include "core_tiles.h"
 *
 *   tile_t haptic;
 *   tile_drive_h_init(core_tiles_pal(&core_i2c1), 0, &haptic, NULL);
 *   if (tile_is_ready(&haptic)) {
 *       tile_drive_h_play(&haptic, 1, 1);  // play effect #1 once
 *   }
 * @endcode
 *
 * @studio tile label=Drive.H icon=≋
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=niche category="OTP waveform burning"
 *   The DRV2605 can burn custom waveforms into one-time-programmable
 *   memory. This is implemented in firmware (tile_drive_h_program_otp())
 *   but intentionally NOT exposed to Studio: it permanently and
 *   irreversibly modifies the chip. Driver-deferred by policy, not a
 *   hardware gap — call it from C if you really need it.
 *
 * @studio unsupported severity=niche category="Minor control bits"
 *   Driver-deferred: LRA_DRIVE_MODE (drive once or twice per cycle),
 *   SUPPLY_COMP_DIS (supply-voltage compensation), NG_THRESH (PWM /
 *   analog noise gate), BRAKE_STABILIZER, STARTUP_BOOST, BEMF_GAIN,
 *   LIBRARY_SEL.HI_Z (outputs high-impedance while idle) and
 *   MODE.DEV_RESET have no setter. Reachable only by editing the
 *   driver; none is hardware-gated.
 *
 * @studio unsupported severity=niche category="Hardware EN control"
 *   Hardware-gated in part: the DRV2605 EN pin is on pad 3 with a
 *   100 kΩ pull-up to V+ (pad 10), so the driver cannot toggle it over I2C. Use the
 *   STANDBY bit (tile_drive_h_standby()) or drive pad 3 from a Core
 *   GPIO. With EN low the chip still ACKs its address but ignores
 *   reads and writes (SLOS825E §7.4.1.3), so init fails on the
 *   device-ID check; registers are kept.
 */

#ifndef INC_TILE_DRIVE_H_H_
#define INC_TILE_DRIVE_H_H_

#include "tiles.h"
#include <stdint.h>

/* -------------------------------------------------------------- */
/* Driver version                                                  */
/* -------------------------------------------------------------- */

#define TILE_DRIVE_H_VERSION_MAJOR  4
#define TILE_DRIVE_H_VERSION_MINOR  3
#define TILE_DRIVE_H_VERSION_PATCH  0

TILES_CHECK_VERSION(1, 0);  /* requires tiles.h >= 1.0 */

/* -------------------------------------------------------------- */
/* Instance mapping                                                */
/* -------------------------------------------------------------- */

/**
 * @brief  Instance-to-address mapping for Drive.H.
 *
 * | Instance | ID   | Bus  | Hardware config      |
 * |----------|------|------|----------------------|
 * | 0        | 0x5A | I2C  | Fixed address        |
 *
 * @note  The DRV2605 has a single fixed I2C address. Multiple
 *        Drive.H tiles require separate I2C buses.
 */
#define DRV2605L_I2C_ADDR_DEFAULT   0x5A

/* -------------------------------------------------------------- */
/* DRV2605 register map                                           */
/* -------------------------------------------------------------- */

#define DRV2605L_REG_STATUS         0x00  /**< Status register */
#define DRV2605L_REG_MODE           0x01  /**< Mode register */
#define DRV2605L_REG_RTP            0x02  /**< Real-time playback input */
#define DRV2605L_REG_LIBRARY_SEL    0x03  /**< Waveform library selection */
#define DRV2605L_REG_WAVE_SEQ_0     0x04  /**< Waveform sequence slot 0 */
#define DRV2605L_REG_WAVE_SEQ_1     0x05  /**< Waveform sequence slot 1 */
#define DRV2605L_REG_WAVE_SEQ_2     0x06  /**< Waveform sequence slot 2 */
#define DRV2605L_REG_WAVE_SEQ_3     0x07  /**< Waveform sequence slot 3 */
#define DRV2605L_REG_WAVE_SEQ_4     0x08  /**< Waveform sequence slot 4 */
#define DRV2605L_REG_WAVE_SEQ_5     0x09  /**< Waveform sequence slot 5 */
#define DRV2605L_REG_WAVE_SEQ_6     0x0A  /**< Waveform sequence slot 6 */
#define DRV2605L_REG_WAVE_SEQ_7     0x0B  /**< Waveform sequence slot 7 */
#define DRV2605L_REG_GO             0x0C  /**< Go register (trigger playback) */
#define DRV2605L_REG_ODT            0x0D  /**< Overdrive time offset */
#define DRV2605L_REG_SPT            0x0E  /**< Sustain time offset (positive) */
#define DRV2605L_REG_SNT            0x0F  /**< Sustain time offset (negative) */
#define DRV2605L_REG_BRT            0x10  /**< Brake time offset */
#define DRV2605L_REG_ATV_CTRL       0x11  /**< Audio-to-vibe control */
#define DRV2605L_REG_ATV_MIN_INPUT  0x12  /**< Audio-to-vibe min input level */
#define DRV2605L_REG_ATV_MAX_INPUT  0x13  /**< Audio-to-vibe max input level */
#define DRV2605L_REG_ATV_MIN_DRIVE  0x14  /**< Audio-to-vibe min output drive */
#define DRV2605L_REG_ATV_MAX_DRIVE  0x15  /**< Audio-to-vibe max output drive */
#define DRV2605L_REG_RATED_VOLTAGE  0x16  /**< Rated voltage for actuator */
#define DRV2605L_REG_OD_CLAMP       0x17  /**< Overdrive clamp voltage */
#define DRV2605L_REG_A_CAL_COMP     0x18  /**< Auto-cal compensation result */
#define DRV2605L_REG_A_CAL_BEMF     0x19  /**< Auto-cal back-EMF result */
#define DRV2605L_REG_FEEDBACK_CTRL  0x1A  /**< Feedback control register */
#define DRV2605L_REG_CONTROL1       0x1B  /**< Control1 (startup boost, AC couple, drive time) */
#define DRV2605L_REG_CONTROL2       0x1C  /**< Control2 (bidir, sample/blank/idiss time) */
#define DRV2605L_REG_CONTROL3       0x1D  /**< Control3 (open loop, RTP format, N_PWM_ANALOG) */
#define DRV2605L_REG_CONTROL4       0x1E  /**< Control4 (auto-cal time, OTP) */
#define DRV2605L_REG_VBAT           0x21  /**< Battery voltage monitor */
#define DRV2605L_REG_LRA_PERIOD     0x22  /**< LRA resonance period */

/** @brief  Expected default STATUS register value (DEVICE_ID = 3). */
#define DRV2605L_STATUS_DEFAULT     0x60

/* -------------------------------------------------------------- */
/* Status register bit masks                                       */
/* -------------------------------------------------------------- */

#define DRV2605L_STATUS_DEVICE_ID   0xE0  /**< Bits 7:5 — device ID */
#define DRV2605L_STATUS_DIAG_RESULT 0x08  /**< Bit 3 — diagnostic result */
#define DRV2605L_STATUS_OVER_TEMP   0x02  /**< Bit 1 — over-temperature */
#define DRV2605L_STATUS_OC_DETECT   0x01  /**< Bit 0 — overcurrent detect */

/* -------------------------------------------------------------- */
/* Mode register values                                            */
/* -------------------------------------------------------------- */

#define DRV2605L_MODE_INTERNAL_TRIG 0x00  /**< Internal trigger (I2C GO) */
#define DRV2605L_MODE_EXT_EDGE      0x01  /**< External trigger, edge mode */
#define DRV2605L_MODE_EXT_LEVEL     0x02  /**< External trigger, level mode */
#define DRV2605L_MODE_PWM_ANALOG    0x03  /**< PWM or analog input on TRIG */
#define DRV2605L_MODE_AUDIO         0x04  /**< Audio-to-vibe on TRIG */
#define DRV2605L_MODE_RTP           0x05  /**< Real-time playback */
#define DRV2605L_MODE_DIAGNOSTICS   0x06  /**< Actuator diagnostics */
#define DRV2605L_MODE_CALIBRATION   0x07  /**< Auto-calibration */
#define DRV2605L_MODE_STANDBY       0x40  /**< STANDBY bit (bit 6) */

/* -------------------------------------------------------------- */
/* Trigger mode selection                                          */
/* -------------------------------------------------------------- */

/** Internal trigger — waveforms fired via I2C GO bit (default). */
#define DRIVE_H_TRIG_INTERNAL  0
/** Edge trigger — rising edge on IN/TRIG pin fires the sequencer.
 *  A second rising edge while GO is high cancels playback.
 *  Pulse width must be >= 1 µs. */
#define DRIVE_H_TRIG_EDGE      1
/** Level trigger — GO bit follows IN/TRIG pin level.
 *  High = playing, falling edge = cancel. */
#define DRIVE_H_TRIG_LEVEL     2

/* -------------------------------------------------------------- */
/* Sequence limits                                                 */
/* -------------------------------------------------------------- */

/** Maximum effects in a single waveform sequence. */
#define DRV2605L_SEQ_MAX            8

/* -------------------------------------------------------------- */
/* Library selection                                               */
/* -------------------------------------------------------------- */

/** ROM waveform library, which also sets the actuator type (ERM vs LRA). */
typedef enum {
    DRIVE_H_LIB_EMPTY = 0,  /**< Empty library (silence). */
    DRIVE_H_LIB_ERM_A = 1,  /**< ERM motor: TS2200 Library A (fixed open-loop overdrive). */
    DRIVE_H_LIB_ERM_B = 2,  /**< ERM motor: TS2200 Library B (no overdrive). */
    DRIVE_H_LIB_ERM_C = 3,  /**< ERM motor: TS2200 Library C (no overdrive). */
    DRIVE_H_LIB_ERM_D = 4,  /**< ERM motor: TS2200 Library D (no overdrive). */
    DRIVE_H_LIB_ERM_E = 5,  /**< ERM motor: TS2200 Library E (no overdrive). */
    DRIVE_H_LIB_LRA   = 6,  /**< LRA (coin / linear actuator): LRA library, auto-resonance. */
} drive_h_library_t;

/* -------------------------------------------------------------- */
/* Public API                                                      */
/* -------------------------------------------------------------- */

/**
 * @brief  Check whether a DRV2605 is present on the I2C bus.
 *
 * @param  hal       Platform HAL handle
 * @param  instance  Instance index (0 = default, see mapping table)
 * @return 1 if device ACKs, 0 otherwise
 */
uint8_t tile_drive_h_find(tiles_pal_t* hal, uint8_t instance);

/**
 * Optional init config. Pass NULL for defaults: LRA closed-loop,
 * library 6, RATED_VOLTAGE 0x56 / OD_CLAMP 0x8C (the actuator is
 * external, via the OUT± pads). At init's ~238 Hz DRIVE_TIME and the
 * reset SAMPLE_TIME (300 µs), 0x56 is ~2.2 Vrms closed-loop
 * (SLOS825E Eq 3) and 0x8C is a 3.07 V peak clamp (Eq 7), ~2.7 Vrms
 * full scale in open loop (Eq 5). That suits a 2.0 Vrms-class coin
 * LRA (bench-verified on a Vybronix V4CHA1) and over-drives smaller
 * ones.
 *
 * For any other actuator, set rated_voltage and od_clamp from its
 * datasheet with Eq 2-5 (SLOS825E §7.5.2.1-7.5.2.2), or call
 * tile_drive_h_set_actuator_voltage() with millivolts. Values at
 * 238 Hz / 300 µs, open-loop clamp = rated:
 *
 * | LRA rated voltage | rated_voltage | od_clamp |
 * |-------------------|---------------|----------|
 * | 0.5 Vrms          | 0x13          | 0x1A     |
 * | 0.7 Vrms          | 0x1B          | 0x24     |
 * | 1.0 Vrms          | 0x26          | 0x34     |
 * | 1.8 Vrms          | 0x45          | 0x5D     |
 * | 2.0 Vrms          | 0x4D          | 0x68     |
 *
 * Note: in open-loop mode the chip ignores RATED_VOLTAGE and derives
 * full-scale output from OD_CLAMP alone (and LRA playback does not
 * resonance-track on this part) — prefer closed loop for LRAs.
 */
typedef struct {
    uint8_t library;       /**< Waveform library: 1-5 = ERM (A-E), 6 = LRA.
                                0 = use default (6). */
    uint8_t closed_loop;   /**< 0 = open-loop, 1 = closed-loop.
                                (cfg=NULL defaults to closed-loop.) */
    uint8_t rated_voltage; /**< RATED_VOLTAGE register (0x16). 0 = default
                                (0x56, ~2.2 Vrms at 238 Hz). Closed-loop reference. */
    uint8_t od_clamp;      /**< OD_CLAMP register (0x17). 0 = default
                                (0x8C, 3.07 V peak). Overdrive clamp / open-loop ref. */
} drive_h_cfg_t;

/**
 * @brief  Initialize the DRV2605 haptic driver.
 *
 * Verifies the device ID, exits standby, and configures the
 * actuator drive mode. Pass cfg=NULL for defaults (LRA closed-loop,
 * library 6, RATED_VOLTAGE 0x56 / OD_CLAMP 0x8C: see drive_h_cfg_t).
 *
 * @param  hal       Platform HAL handle
 * @param  instance  Instance index (0 = default, see mapping table)
 * @param  tile      Pointer to tile handle (populated by this function)
 * @param  cfg       Optional config, or NULL for defaults
 *
 * @note   Blocks for ~500 ms during init. Call once at startup.
 */
void tile_drive_h_init(tiles_pal_t* hal, uint8_t instance, tile_t* tile,
                       const drive_h_cfg_t *cfg);

/**
 * @brief  Play a waveform effect from the built-in library.
 *
 * Loads the effect index into sequence slot 0, terminates
 * the sequence, and triggers playback. For repeats > 1,
 * re-triggers with a 200ms gap between plays.
 *
 * @studio expose category=tile name=play section=runtime
 * @param  index    [1..123] Library effect index (see datasheet).
 * @param  repeats  [1..20] Number of times to play (1 = once).
 */
void tile_drive_h_play(tile_t* tile, uint8_t index, uint8_t repeats);

/**
 * @brief  Play a sequence of up to 8 waveform effects.
 *
 * Loads effects into the waveform sequencer registers (0x04-0x0B),
 * terminates the sequence, and triggers playback. Returns
 * immediately — use tile_drive_h_is_playing() to poll for
 * completion. DSL pads short sequences with 0 (DRV2605 effect 0
 * is "stop"), so a 3-effect sequence written as `[5, 10, 15, 0,
 * 0, 0, 0, 0]` plays effects 5/10/15 then halts.
 *
 * @studio expose category=tile name=play_sequence section=runtime
 * @studio in_buffer effects type=uint8_t length=8 length_param=count
 * @param  effects  Array of effect indices (1-123, 0 = stop)
 * @param  count    Number of effects (1-8)
 */
void tile_drive_h_play_sequence(tile_t* tile, const uint8_t *effects,
                                uint8_t count);

/**
 * @brief  Load effects into the waveform sequencer without triggering.
 *
 * Identical to tile_drive_h_play_sequence() but does NOT assert
 * the GO bit. Use this to pre-load effects before switching to
 * external trigger mode (edge or level).
 *
 * @studio expose category=tile name=load_sequence section=runtime
 * @studio in_buffer effects type=uint8_t length=8 length_param=count
 * @param  effects  Array of effect indices (1-123, 0 = stop)
 * @param  count    Number of effects (1-8)
 */
void tile_drive_h_load_sequence(tile_t* tile, const uint8_t *effects,
                                uint8_t count);

/**
 * @brief  Set a per-slot wait delay in the loaded sequence.
 *
 * The 8 sequencer slots can each be marked as either an effect
 * (MSB=0, low 7 bits = waveform 1..123) or a wait time (MSB=1,
 * low 7 bits × 10 ms = pause). This call rewrites slot @p slot
 * with a wait of @p delay_steps × 10 ms (max 1270 ms). Use it
 * after tile_drive_h_load_sequence() to insert pauses between
 * effects, e.g. play 1, wait 200 ms, play 47.
 *
 * Slot is silently ignored if >= 8; delay_steps is clipped to 0x7F.
 *
 * @studio expose category=tile name=set_sequence_wait
 * @param  slot         Slot index 0..7
 * @param  delay_steps  Wait length in 10 ms steps (0 = no wait, 0x7F = 1.27 s)
 */
void tile_drive_h_set_sequence_wait(tile_t* tile, uint8_t slot,
                                    uint8_t delay_steps);

/**
 * @brief  Set the trigger mode.
 *
 * Selects how the waveform sequencer is triggered:
 *   - DRIVE_H_TRIG_INTERNAL (default): I2C GO bit, used by play()
 *     and play_sequence().
 *   - DRIVE_H_TRIG_EDGE: a rising edge on the IN/TRIG pin sets
 *     GO. A second rising edge while playing cancels. Pre-load
 *     effects with tile_drive_h_load_sequence() before entering
 *     this mode.
 *   - DRIVE_H_TRIG_LEVEL: GO follows the IN/TRIG pin level.
 *     High = playing, low = idle. Falling edge cancels.
 *
 * @studio expose category=tile name=set_trigger section=runtime
 * @param  mode  [0..2] One of DRIVE_H_TRIG_INTERNAL (0), DRIVE_H_TRIG_EDGE (1),
 *               DRIVE_H_TRIG_LEVEL (2)
 */
void tile_drive_h_set_trigger(tile_t* tile, uint8_t mode);

/**
 * @brief  Check whether an effect or sequence is still playing.
 *
 * Reads the GO bit in register 0x0C. The GO bit remains high
 * until playback completes.
 *
 * @studio expose category=tile name=is_playing returns=bool section=runtime
 * @return 1 if playing, 0 if idle
 */
uint8_t tile_drive_h_is_playing(tile_t* tile);

/**
 * @brief  Stop any currently playing effect.
 *
 * @studio expose category=tile name=stop section=runtime
 */
void tile_drive_h_stop(tile_t* tile);

/* ---- ROM library / actuator tuning -------------------------------- */

/**
 * @brief  Switch the active waveform library at runtime.
 *
 * The DRV2605 ships with 6 ROM libraries: 1–5 are TS2200 ERM
 * libraries (A–E), 6 is the LRA library. Library 0 is empty
 * (silence). This call also updates the FEEDBACK_CTRL N_ERM_LRA
 * bit so the chip drives the correct actuator type.
 *
 * The actuator is external (OUT± pads), so this must match what is
 * wired: an LRA (coin / linear) uses DRIVE_H_LIB_LRA, an ERM (spinning
 * mass) motor one of the ERM libraries.
 *
 * @studio expose category=tile name=set_library
 * @studio control library label="Actuator type" tier=basic default=DRIVE_H_LIB_LRA allow=DRIVE_H_LIB_LRA,DRIVE_H_LIB_ERM_A,DRIVE_H_LIB_ERM_B,DRIVE_H_LIB_ERM_C,DRIVE_H_LIB_ERM_D,DRIVE_H_LIB_ERM_E
 * @param  library  Library (use DRIVE_H_LIB_* constants)
 */
void tile_drive_h_set_library(tile_t* tile, drive_h_library_t library);

/**
 * @brief  Tune the actuator drive parameters.
 *
 * Writes the four registers that the auto-calibration engine and
 * smart-loop architecture consume:
 *   - rated_voltage (0x16): full-scale RMS drive level (closed-loop ref).
 *   - od_clamp      (0x17): peak overdrive ceiling (also open-loop ref).
 *   - fb_brake      (0x1A bits 6:4): feedback brake factor (0=1× .. 6=16×, 7=off).
 *   - loop_gain     (0x1A bits 3:2): closed-loop gain (0=Low .. 3=Very high).
 *
 * Pass 0 for rated_voltage / od_clamp to leave that register untouched.
 * Pass 0xFF for fb_brake / loop_gain to leave them untouched. Run
 * tile_drive_h_calibrate() afterwards to update A_CAL_COMP / A_CAL_BEMF.
 *
 * @studio expose category=tile name=set_actuator_params
 * @param  rated_voltage  RATED_VOLTAGE byte (0 = no change)
 * @param  od_clamp       OD_CLAMP byte (0 = no change)
 * @param  fb_brake       Feedback brake factor 0..7 (0xFF = no change)
 * @param  loop_gain      Loop gain 0..3 (0xFF = no change)
 */
void tile_drive_h_set_actuator_params(tile_t* tile,
                                      uint8_t rated_voltage,
                                      uint8_t od_clamp,
                                      uint8_t fb_brake,
                                      uint8_t loop_gain);

/**
 * @brief  Switch between open-loop and closed-loop drive at runtime.
 *
 * Closed loop (recommended for LRAs) enables smart-loop back-EMF
 * feedback: automatic resonance tracking, overdrive, and braking,
 * with full-scale amplitude referenced to RATED_VOLTAGE. Open loop
 * drives blind: amplitude is referenced to OD_CLAMP, RATED_VOLTAGE
 * is ignored, and LRA playback does NOT resonance-track on this
 * part — the commutation frequency comes from DRIVE_TIME (see
 * tile_drive_h_set_resonance_hz()).
 *
 * Applies to whichever actuator type is currently selected
 * (set_library() / FEEDBACK_CTRL N_ERM_LRA).
 *
 * @note  Switching to open loop makes OD_CLAMP the full-scale level:
 *        with init's 0x8C that is ~2.7 Vrms, stronger than closed loop.
 *
 * @studio expose category=tile name=set_loop_mode section=config
 * @studio control closed label="Closed-loop drive" tier=advanced type=bool default=1
 * @param  closed  1 = closed-loop (smart-loop), 0 = open-loop
 */
void tile_drive_h_set_loop_mode(tile_t* tile, uint8_t closed);

/**
 * @brief  Set the actuator drive levels in millivolts.
 *
 * The friendly-units version of tile_drive_h_set_actuator_params():
 * converts voltages to the RATED_VOLTAGE (0x16) and OD_CLAMP (0x17)
 * register values using the DRV2605 datasheet equations (section
 * 7.5.2, Eq. 2-5), honouring the currently selected actuator type
 * (ERM vs LRA) and, for LRAs, the drive frequency currently
 * programmed via tile_drive_h_set_resonance_hz().
 *
 *   - rated_mv:     steady-state full-scale level (RMS for LRA,
 *                   average for ERM). Closed-loop reference.
 *   - overdrive_mv: peak ceiling for overdrive/braking; also the
 *                   full-scale reference in open-loop mode. Must be
 *                   >= rated_mv; typically 1.3-1.5x for LRAs.
 *
 * Run tile_drive_h_calibrate() afterwards — the datasheet requires
 * recalibration whenever these references change.
 *
 * @warning Set rated_mv from the actuator's datasheet. Driving an LRA
 *          above its rated voltage for long effects (play_buzz, RTP)
 *          heats it and shortens its life. The output never exceeds
 *          V_MOTOR, whatever is set here.
 *
 * The control defaults (2230 / 2700 mV) reproduce init's 0x56 / 0x8C
 * at init's 238 Hz DRIVE_TIME.
 *
 * @studio expose category=tile name=set_actuator_voltage section=config
 * @studio control rated_mv label="Actuator rated voltage" tier=basic default=2230 scale=0.001 unit=V
 * @studio control overdrive_mv label="Overdrive clamp" tier=advanced default=2700 scale=0.001 unit=V
 * @param  rated_mv      [300..3600] Rated drive level in mV
 * @param  overdrive_mv  [300..5000] Overdrive clamp in mV
 */
void tile_drive_h_set_actuator_voltage(tile_t* tile, uint16_t rated_mv,
                                       uint16_t overdrive_mv);

/**
 * @brief  Set the LRA resonant frequency the driver targets.
 *
 * Programs CONTROL1 DRIVE_TIME to half the LRA period — the
 * datasheet-optimal value. In closed-loop mode this seeds the
 * auto-resonance tracker (which then follows the real resonance);
 * in open-loop mode it directly sets the commutation frequency.
 * Read the actuator's actual resonance with
 * tile_drive_h_get_resonance_hz() while driving in closed loop.
 *
 * @studio expose category=tile name=set_resonance_hz section=config
 * @studio control hz label="LRA resonance" tier=advanced default=238 unit=Hz
 * @param  hz    [125..300] LRA resonant frequency in Hz
 */
void tile_drive_h_set_resonance_hz(tile_t* tile, uint16_t hz);

/**
 * @brief  Tune the LRA auto-resonance tracker.
 *
 * Writes Control2 (0x1C) bits for sample / blanking / current-
 * dissipation timing. Default values (3 / 1 / 1) work for most
 * coin-type LRAs at 175–235 Hz. Tune only if calibration fails to
 * converge or the actuator runs at the edges of the 125–300 Hz
 * window.
 *
 * Pass 0xFF for any field to leave that register slice untouched.
 *
 * @studio expose category=tile name=set_resonance_params
 * @param  sample_time    Sample time 0..3 (0=150µs, 3=300µs; 0xFF = no change)
 * @param  blanking_time  Blanking time 0..3 (0xFF = no change)
 * @param  idiss_time     Current-dissipation time 0..3 (0xFF = no change)
 */
void tile_drive_h_set_resonance_params(tile_t* tile,
                                       uint8_t sample_time,
                                       uint8_t blanking_time,
                                       uint8_t idiss_time);

/* ---- Library waveform timing offsets ------------------------------ */

/**
 * @brief  Tune the open-loop library waveform timing.
 *
 * Adds signed offsets (in 5 ms steps) to the overdrive, sustain,
 * and brake portions of every library effect. Offsets are 8-bit
 * two's complement, so range is -640..+635 ms per knob.
 *
 * These offsets are only honoured in open-loop mode — closed-loop
 * mode generates them automatically from back-EMF feedback.
 *
 * @studio expose category=tile name=set_waveform_timing
 * @param  overdrive     Overdrive Time Offset (0x0D, signed × 5 ms)
 * @param  sustain_pos   Sustain-Time Positive Offset (0x0E, signed × 5 ms)
 * @param  sustain_neg   Sustain-Time Negative Offset (0x0F, signed × 5 ms)
 * @param  brake         Brake Time Offset (0x10, signed × 5 ms)
 */
void tile_drive_h_set_waveform_timing(tile_t* tile,
                                      int8_t overdrive,
                                      int8_t sustain_pos,
                                      int8_t sustain_neg,
                                      int8_t brake);

/* ---- RTP (real-time playback) ------------------------------------- */

/**
 * @brief  Enter RTP (Real-Time Playback) mode.
 *
 * Sets DRV2605 MODE register to 0x05 (RTP). The chip drives
 * the LRA at its resonant frequency with amplitude controlled
 * by tile_drive_h_rtp_write(). Call tile_drive_h_rtp_stop()
 * to return to internal trigger mode.
 *
 * @studio expose category=tile name=rtp_start section=runtime
 */
void tile_drive_h_rtp_start(tile_t* tile);

/**
 * @brief  Write an amplitude value in RTP mode.
 *
 * @studio expose category=tile name=rtp_write section=runtime
 * @param  amplitude  [0..127] Amplitude (0 = off, 127 = max).
 */
void tile_drive_h_rtp_write(tile_t* tile, uint8_t amplitude);

/**
 * @brief  Set the RTP data format.
 *
 * Selects how RTP_INPUT bytes are interpreted:
 *   - signed (default for closed-loop bidirectional): 0x80 = full
 *     reverse, 0x00 = mid-scale (no drive), 0x7F = full forward.
 *   - unsigned (recommended for closed-loop unidirectional and
 *     open-loop): 0x00 = no drive, 0xFF = full drive.
 *
 * Also exposes the BIDIR_INPUT bit (CONTROL2[7]) which selects
 * unidirectional vs bidirectional input interpretation; tie this
 * to the same convention as the format flag.
 *
 * @studio expose category=tile name=set_rtp_format
 * @param  unsigned_   1 = unsigned data format, 0 = signed (default)
 * @param  bidir       1 = bidirectional input (default), 0 = unidirectional
 */
void tile_drive_h_set_rtp_format(tile_t* tile, uint8_t unsigned_, uint8_t bidir);

/**
 * @brief  Exit RTP mode and return to internal trigger mode.
 *
 * @studio expose category=tile name=rtp_stop section=runtime
 */
void tile_drive_h_rtp_stop(tile_t* tile);

/* ---- PWM / analog input modes ------------------------------------- */

/**
 * @brief  Drive the actuator from a PWM signal on pad 2 (TRIG).
 *
 * Sets MODE=3 with N_PWM_ANALOG=0. The chip accepts a 10–250 kHz
 * PWM signal: duty cycle directly modulates output amplitude
 * (50 % = no drive in bidirectional mode; 0 % = no drive in
 * unidirectional mode). Useful for offloading waveform generation
 * to a hardware timer or audio rendering pipeline.
 *
 * Call tile_drive_h_set_rtp_format() first if you need to change
 * the unidirectional / bidirectional interpretation.
 *
 * @studio expose category=tile name=pwm_input_start
 */
void tile_drive_h_pwm_input_start(tile_t* tile);

/**
 * @brief  Drive the actuator from an analog voltage on pad 2 (TRIG).
 *
 * Sets MODE=3 with N_PWM_ANALOG=1. Reference voltage is 1.8 V:
 * 0 V → 0 % drive, 0.9 V → 50 %, 1.8 V → 100 %. Useful for waveform
 * synthesis from a DAC or analog signal source.
 *
 * @note Pad 2 (TRIG) is a digital pad on the tile. The chip pin
 *       itself accepts 0–1.8 V analog, but the driving Core must
 *       output a clean analog level — Drive.H has no input filter
 *       or AC-coupling cap on TRIG.
 *
 * @studio expose category=tile name=analog_input_start
 */
void tile_drive_h_analog_input_start(tile_t* tile);

/**
 * @brief  Exit PWM / analog input mode.
 *
 * Returns to internal-trigger mode (MODE=0). Same effect as
 * tile_drive_h_audio_stop(); both modes share the IN/TRIG pin.
 *
 * @studio expose category=tile name=pwm_input_stop
 */
void tile_drive_h_pwm_input_stop(tile_t* tile);

/* ---- Audio-to-vibe ------------------------------------------------ */

/**
 * @brief  Drive the actuator from an AC-coupled audio signal.
 *
 * Sets MODE=4 (audio-to-vibe), N_PWM_ANALOG=1 (analog), and
 * AC_COUPLE=1 (0.9 V common-mode bias on IN/TRIG). The chip
 * envelope-detects the audio and drives haptic vibration at
 * matching intensity.
 *
 * @note The DRV2605 expects an AC-coupled line-level audio
 *       source (1.8 Vpp full-scale). Drive.H rev a does not
 *       include a series capacitor on pad 2 — users wanting
 *       audio-to-vibe must add their own external 1 µF AC-coupling
 *       cap between the audio source and pad 2.
 *
 * @studio expose category=tile name=audio_start
 */
void tile_drive_h_audio_start(tile_t* tile);

/**
 * @brief  Tune the audio-to-vibe envelope detector.
 *
 * Configures the four ATV control registers:
 *   - ATV_CTRL  (0x11): peak-detect time (0=10ms..3=40ms) and
 *     low-pass filter cutoff (0=100Hz..3=200Hz).
 *   - ATV_MIN_INPUT (0x12): minimum input level below which the
 *     envelope is gated (output silent). raw × 1.8/255 V.
 *   - ATV_MAX_INPUT (0x13): full-scale input level. raw × 1.8/255 V.
 *   - ATV_MIN_DRIVE (0x14): minimum output drive once unmuted.
 *     raw / 255 × 100 %.
 *   - ATV_MAX_DRIVE (0x15): maximum output drive at full input.
 *     raw / 255 × 100 %.
 *
 * Pass 0xFF for any field to leave it untouched.
 *
 * @studio expose category=tile name=set_audio_params
 * @param  peak_time    ATH_PEAK_TIME 0..3 (10/20/30/40 ms; 0xFF = no change)
 * @param  filter       ATH_FILTER 0..3 (100/125/150/200 Hz; 0xFF = no change)
 * @param  min_input    Minimum input gate (0xFF = no change)
 * @param  max_input    Full-scale input level (0xFF = no change)
 * @param  min_drive    Minimum output drive once active (0xFF = no change)
 * @param  max_drive    Full-scale output drive (0xFF = no change)
 */
void tile_drive_h_set_audio_params(tile_t* tile,
                                   uint8_t peak_time,
                                   uint8_t filter,
                                   uint8_t min_input,
                                   uint8_t max_input,
                                   uint8_t min_drive,
                                   uint8_t max_drive);

/**
 * @brief  Exit audio-to-vibe mode.
 *
 * Returns to internal-trigger mode (MODE=0) and clears the
 * AC_COUPLE bit. Same physical effect as tile_drive_h_pwm_input_stop().
 *
 * @studio expose category=tile name=audio_stop
 */
void tile_drive_h_audio_stop(tile_t* tile);

/* ---- Status / diagnostics / calibration --------------------------- */

/**
 * @brief  Read the raw STATUS register (0x00).
 *
 * Contains DEVICE_ID[7:5], DIAG_RESULT[3], FB_STS[2],
 * OVER_TEMP[1], OC_DETECT[0]. Status bits clear on read.
 *
 * @studio expose category=tile name=get_status returns=int section=runtime
 * @return Raw status byte
 */
uint8_t tile_drive_h_get_status(tile_t* tile);

/**
 * @brief  Run actuator diagnostics.
 *
 * Enters diagnostic mode (MODE=6), triggers GO, and polls
 * for completion. The DRV2605 checks whether the actuator is
 * present, open, or shorted.
 *
 * @studio expose category=tile name=diagnose returns=bool section=runtime
 * @return 1 if actuator passed diagnostics, 0 if fault detected
 */
uint8_t tile_drive_h_diagnose(tile_t* tile);

/**
 * @brief  Run auto-calibration for the connected actuator.
 *
 * Enters calibration mode (MODE=7) with datasheet-recommended
 * parameters, triggers GO, and polls for completion. On success,
 * the DRV2605 stores optimized A_CAL_COMP and A_CAL_BEMF values
 * that improve playback fidelity.
 *
 * @studio expose category=tile name=calibrate returns=bool section=runtime
 * @return 1 if calibration passed, 0 if it failed to converge
 */
uint8_t tile_drive_h_calibrate(tile_t* tile);

/**
 * @brief  Read supply voltage in millivolts.
 *
 * Reads the VBAT register (0x21). The reading is only valid
 * while the device is actively driving a waveform (RTP, library
 * playback, etc.).
 *
 * @studio expose category=tile name=get_vbat_mv returns=int section=runtime
 * @return Battery voltage in mV (e.g. 3300 = 3.3 V), 0 if idle
 */
uint16_t tile_drive_h_get_vbat_mv(tile_t* tile);

/**
 * @brief  Read LRA resonant frequency in Hz.
 *
 * Reads the LRA_PERIOD register (0x22). The reading is only
 * valid while the device is actively driving a waveform and
 * must not be polled during braking.
 *
 * @studio expose category=tile name=get_resonance_hz returns=int section=runtime
 * @return Resonant frequency in Hz (e.g. 235), 0 if unavailable
 */
uint16_t tile_drive_h_get_resonance_hz(tile_t* tile);

/* ---- Power management --------------------------------------------- */

/**
 * @brief  Enter low-power standby.
 *
 * Sets the STANDBY bit in the MODE register. The device retains
 * register values and can be woken quickly with
 * tile_drive_h_wake().
 *
 * @studio expose category=tile name=standby section=runtime
 */
void tile_drive_h_standby(tile_t* tile);

/**
 * @brief  Wake from standby.
 *
 * Clears the STANDBY bit in the MODE register. The device
 * returns to the active state, ready for playback.
 *
 * @studio expose category=tile name=wake section=runtime
 */
void tile_drive_h_wake(tile_t* tile);

/* ---- One-time programmable memory (DESTRUCTIVE) ------------------- */

/**
 * @brief  Burn the current calibration into on-chip OTP.
 *
 * Writes the current contents of registers 0x16–0x1A
 * (RATED_VOLTAGE, OD_CLAMP, A_CAL_COMP, A_CAL_BEMF, FEEDBACK_CTRL)
 * to the DRV2605's nonvolatile OTP cells. After this, those values
 * become the power-on defaults — the chip skips run-time calibration
 * on subsequent boots.
 *
 * @warning IRREVERSIBLE. The DRV2605 OTP can be programmed exactly
 *          once per device. A bad programming run permanently
 *          mistunes the chip. NOT exposed to Studio by design.
 *
 * Pre-conditions:
 *   - Run tile_drive_h_calibrate() first and confirm pass.
 *   - VDD must be 4.0–4.4 V at the moment of programming
 *     (datasheet section 7.5.7). Lower VDD silently corrupts the cells.
 *
 * Returns 1 if the OTP_STATUS bit reads 1 after programming, 0 if
 * the burn failed or OTP was already programmed.
 *
 * @return 1 on success, 0 on failure / already-programmed
 */
uint8_t tile_drive_h_program_otp(tile_t* tile);

/**
 * @brief  Read the OTP_STATUS bit.
 *
 * Returns 1 if the chip's OTP cells have already been programmed
 * (registers 0x16–0x1A boot from OTP rather than chip defaults).
 *
 * @studio expose category=tile name=get_otp_status returns=bool
 * @return 1 if OTP is programmed, 0 if unprogrammed
 */
uint8_t tile_drive_h_get_otp_status(tile_t* tile);

/* ============================================================== */
/* Runtime — tier-2 idiomatic helpers                              */
/*                                                                  */
/* These compose the tier-1 surface above into "do the thing the   */
/* user wants to do" calls. They wrap effect IDs from the          */
/* TS2200 / Immersion ROM library and the sequencer / RTP paths   */
/* so callers don't need to memorize effect numbers or mode        */
/* transitions for the common idioms.                              */
/* ============================================================== */

/**
 * @brief  Fire a single sharp tactile click.
 *
 * @studio expose category=tile name=play_click section=runtime
 *
 * Plays ROM library effect 1 ("Strong Click — 100%") once. Returns
 * immediately; the chip drives the click after the call returns.
 * Use @ref tile_drive_h_is_playing to poll for completion.
 *
 */
void tile_drive_h_play_click(tile_t* tile);

/**
 * @brief  Fire a double tap.
 *
 * @studio expose category=tile name=play_double_tap section=runtime
 *
 * Loads the sequencer with ROM library effect 10 ("Double Click —
 * 100%"), a two-click effect, and triggers playback. The
 * chip handles the inter-tap timing internally. Returns
 * immediately; use @ref tile_drive_h_is_playing to poll.
 *
 */
void tile_drive_h_play_double_tap(tile_t* tile);

/**
 * @brief  Play an alert pattern.
 *
 * @studio expose category=tile name=play_alert section=runtime
 *
 * Sequences three ROM-library effects: effect 14 ("Strong Buzz —
 * 100%") → effect 56 ("Pulsing Sharp 1 — 100%") → effect 14.
 * Returns immediately; use @ref tile_drive_h_is_playing to poll.
 *
 */
void tile_drive_h_play_alert(tile_t* tile);

/**
 * @brief  Play a sustained vibration for `ms` milliseconds.
 *
 * @studio expose category=tile name=play_buzz section=runtime
 *
 * Switches into RTP (Real-Time Playback) mode, drives a constant
 * full-amplitude signal for `ms` milliseconds, then stops the RTP
 * stream and returns to internal-trigger mode.
 *
 * @note  Blocking call — does not return until the buzz completes.
 *        For longer non-blocking patterns, drive RTP directly via
 *        @ref tile_drive_h_rtp_start / @ref tile_drive_h_rtp_write
 *        / @ref tile_drive_h_rtp_stop.
 *
 * @param  ms    [0..65535] Duration in milliseconds
 */
void tile_drive_h_play_buzz(tile_t* tile, uint16_t ms);

/**
 * @brief  Check whether smart-loop calibration converged.
 *
 * @studio expose category=tile name=is_calibrated returns=bool section=runtime
 *
 * Reads the STATUS register's DIAG_RESULT bit, which the auto-
 * calibration engine populates on completion: 0 = converged
 * (calibration produced valid A_CAL_COMP / A_CAL_BEMF values),
 * 1 = failed. Only meaningful after a prior call to @ref
 * tile_drive_h_calibrate.
 *
 * @note  STATUS bits clear on read, so the result reflects the
 *        most recent calibration / diagnostic run.
 *
 * @return 1 if calibration converged, 0 otherwise
 */
uint8_t tile_drive_h_is_calibrated(tile_t* tile);

#endif /* INC_TILE_DRIVE_H_H_ */
