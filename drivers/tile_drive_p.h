/**
 * @file   tile_drive_p.h
 * @brief  Piezoelectric haptic driver for the Drive.P tile (rev a).
 *
 * Embeds the Boréas Technologies BOS1921, a piezoelectric driver
 * with integrated high-voltage boost (190 Vpp differential),
 * waveform synthesizer, 1024-sample FIFO, and piezo sensing.
 *
 * Key specifications:
 *   - Output:      190 Vpp differential (high range, ±95 V), up to 820 nF
 *                  capacitive load. The driver STARTS IN THE LOW RANGE
 *                  (±13.28 V); ±95 V needs an explicit
 *                  tile_drive_p_set_output_range(tile, DRIVE_P_OUTPUT_HIGH_V).
 *   - Sensing:     7.6 mV resolution (fine), 54.5 mV (coarse)
 *   - Play modes:  Direct, FIFO, RAM, RAM Synth
 *   - Sample rate:  8 ksps – 1024 ksps (configurable)
 *
 * Datasheet: https://www.bergsonne.io/tiles/drive/p
 * IC datasheet: https://mosaic-component-datasheets.s3.eu-north-1.amazonaws.com/5/Bor_as_Technologies-BOS1921.pdf
 *
 * @studio tile label=Drive.P icon=♪
 *
 * Quick start:
 * @code
 *   #include "core_tiles.h"
 *
 *   tile_t piezo;
 *   tile_drive_p_init(core_tiles_pal(&core_i2c1), 0, &piezo, NULL);
 *   if (tile_is_ready(&piezo)) {
 *       // init leaves the output in the LOW range (±13.28 V). Opt in to
 *       // ±95 V explicitly, and only for a piezo rated for it:
 *       // tile_drive_p_set_output_range(&piezo, DRIVE_P_OUTPUT_HIGH_V);
 *       // Every sample is limited to +95 V / -10 V by default (PowerHap-safe;
 *       // see tile_drive_p_set_voltage_limits), flattened at the limit.
 *       tile_drive_p_play_click(&piezo, 80);
 *   }
 * @endcode
 *
 * Operating notes / gotchas (learned bringing up Drive.P haptics on Ring):
 *
 *   - UPI IS MANDATORY ON BATTERY / NON-SINKING SUPPLIES. The BOS1921 recovers
 *     piezo-discharge energy back toward VDD (bidirectional power). If the PDN
 *     can't sink that current, recovered charge piles onto CVDD and VDDP climbs
 *     (must stay < 5.5 V), skewing the Rsense current sense → MXPWR/IDAC faults.
 *     Call tile_drive_p_set_upi(tile, 1) right after init on such boards
 *     (datasheet §6.2.13). Leave UPI = 0 only when the supply can sink current.
 *
 *   - DISCHARGE TO 0 V BEFORE PLAYING into possible residual charge. Output-bridge
 *     protection (§6.2.18) blocks a polarity change when > 4.4 V of the OPPOSITE
 *     sign sits on the piezo, and can play the waveform in the wrong polarity —
 *     with or WITHOUT setting an IDAC error (so it just feels like "nothing
 *     happened"). Residual builds from a prior effect AND from sensing (the
 *     ceramic self-generates voltage when deformed). Prepend a few 0 V samples
 *     (or a small ramp to 0) to each waveform.
 *
 *   - IDAC FAULT DOES NOT SELF-CLEAR. Current-detection / over-current (MXPWR→IDAC)
 *     latches the chip in ERROR until a software reset. Call
 *     tile_drive_p_check_and_recover(tile, mode) after effects (or on any ERROR)
 *     to reset + restore; otherwise the driver stays dead until a power cycle.
 *
 *   - INIT STARTS IN THE LOW RANGE (±13.28 V, CONFIG.GAIND = 1) since v3.6.
 *     Before v3.6 init left the chip's reset value GAIND = 0, i.e. ±95 V. Firmware
 *     that drives a high-voltage actuator must now call
 *     tile_drive_p_set_output_range(tile, DRIVE_P_OUTPUT_HIGH_V) after init.
 *
 *   - EVERY OUTPUT SAMPLE IS VOLTAGE-LIMITED (since v3.6), separately per
 *     polarity: by default +the range's maximum / -10 V, because PowerHap
 *     actuators take a high positive voltage but only ~10 V negative. A
 *     waveform that exceeds a limit still plays, flattened at it; read
 *     tile_drive_p_read_clamp_count() to find out it happened. Widen the
 *     negative limit only for an actuator rated for it
 *     (tile_drive_p_set_voltage_limits). The limits are not applied to RAM
 *     Playback / RAM Synthesis waveforms (WFS commands), see wfs_write().
 *
 *   - THE CURRENT LIMIT IS COMPONENT-DEPENDENT. The parcap / SUP_RISE values set
 *     in init are tuned for a 260 nF piezo, L1 = 10 µH, Rsense = 0.2 Ω. A board
 *     with different components hits MXPWR/IDAC far below the rated ±95 V (Ring
 *     faulted ~33 V). If you see early MXPWR/IDAC at low voltage, suspect sizing:
 *     retune parcap / SUP_RISE and the Rsense current limit (§7.5.3) to the actual
 *     BOM, or cap the drive amplitude.
 *
 *   - play_samples() RETURNS WHEN THE LAST SAMPLE IS QUEUED, NOT PLAYED — up to
 *     1024 samples (128 ms at 8 ksps) are still in the FIFO when it returns.
 *     Account for that in teardown / discharge timing (don't switch to SENSE or
 *     read status assuming playback is done). Streaming paces on FIFO_SPACE
 *     and writes 32-sample bursts; gapless 8 ksps needs a 400 kHz (or faster)
 *     bus — at 100 kHz two bytes per sample top out near 5.5 ksps.
 *
 *   - END EVERY WAVEFORM AT 0. When the FIFO drains, the last sample stays on
 *     the output (datasheet §6.6). The click / sine / buzz / pulse-train
 *     helpers end at exactly 0 V; play_samples() plays the caller's buffer
 *     as-is, so make its last sample 0.
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=advanced category="Multi-device SYNC pin" section=advanced
 *   SYNC pin coordinates phase between cascaded BOS1921s
 *   (< 2 µs delay). The Drive.P tile has 10 pads (I2C/I3C, OUT±,
 *   GPIO, V+, V_DRIVE, GND); the SYNC pin on the IC is not routed
 *   to a pad, so multi-tile cascading is hardware-gated to a
 *   future tile rev.
 *
 * @studio unsupported severity=advanced category="I3C alternate bus mode"
 *   Pads 4/5 are bus-shared between I²C (default) and I3C SDR
 *   (≤12.5 Mbps with in-band interrupts). The cores tile-driver
 *   framework only ships an I²C PAL; I3C support is an
 *   ecosystem-wide gap, not BOS1921-specific.
 */

#ifndef INC_TILE_DRIVE_P_H_
#define INC_TILE_DRIVE_P_H_

#include "tiles.h"
#include <stdint.h>

/* -------------------------------------------------------------- */
/* Driver version                                                  */
/* -------------------------------------------------------------- */

#define TILE_DRIVE_P_VERSION_MAJOR  3
#define TILE_DRIVE_P_VERSION_MINOR  6
#define TILE_DRIVE_P_VERSION_PATCH  0

TILES_CHECK_VERSION(1, 0);  /* requires tiles.h >= 1.0 */

/* -------------------------------------------------------------- */
/* Instance mapping                                                */
/* -------------------------------------------------------------- */

/**
 * @brief  Instance-to-address mapping for Drive.P.
 *
 * | Instance | ID   | Bus  | Hardware config                    |
 * |----------|------|------|------------------------------------|
 * | 0        | 0x44 | I2C  | Power-on default (single tile)     |
 * | 1        | 0x45 | I2C  | First of a pair, after reassign    |
 * | 2        | 0x46 | I2C  | Second of a pair, after reassign   |
 *
 * @note  Every BOS1921 powers up at the same static address 0x44, so two
 *        on one bus collide. They are re-addressed via the GPIO write-gate
 *        (datasheet §6.3.3): hold the target chip's GPIO low and only that
 *        chip accepts a new address. See @ref tile_drive_p_reassign_address.
 *        Only the low nibble is programmable (I2C_ADDR[3:0]), so addresses
 *        lie in 0x40..0x4F.
 *
 *        For a PAIR sharing one bus, move BOTH off 0x44 — instance 1→0x45,
 *        instance 2→0x46 — each gated by its own GPIO. (Keeping one chip at
 *        0x44 is unreliable in practice; move both. Verified on Ring_Av2.)
 *
 * @note  The reassigned address is held until power-on reset. A *software*
 *        reset does NOT revert it (the operating address only changes via a
 *        GPIO-gated SUP_RISE write), so a reassigned chip can be soft-reset
 *        and re-init'd normally — only a power cycle returns it to 0x44.
 */
#define BOS1921_I2C_ADDR_DEFAULT    0x44
#define BOS1921_I2C_ADDR_SECOND     0x45  /**< Instance 1 — first of a pair */
#define BOS1921_I2C_ADDR_THIRD      0x46  /**< Instance 2 — second of a pair */

/* -------------------------------------------------------------- */
/* BOS1921 register map                                            */
/* -------------------------------------------------------------- */

#define BOS1921_REG_REFERENCE       0x00  /**< Waveform reference / FIFO write */
#define BOS1921_REG_ION_BL          0x01  /**< HS turn-on current (I_ON_SCALE), blanking, FSWMAX */
#define BOS1921_REG_CONFIG          0x05  /**< Main configuration register */
#define BOS1921_REG_PARCAP          0x06  /**< Parasitic capacitance trim */
#define BOS1921_REG_SUP_RISE        0x07  /**< Supply rise time */
#define BOS1921_REG_COMM            0x0B  /**< Communication / return register select */
#define BOS1921_REG_IC_STATUS       0x10  /**< IC status register */
#define BOS1921_REG_FIFO_STATE      0x11  /**< FIFO state: ERROR / FULL / EMPTY / FIFO_SPACE */
#define BOS1921_REG_SENSE_VAL       0x18  /**< Sensed piezo voltage */
#define BOS1921_REG_CHIP_ID         0x1E  /**< Chip identification */

/** @brief  Expected lower 12 bits of CHIP_ID register. */
#define BOS1921_CHIP_ID_DEFAULT     0x0781

/** @brief  Largest REFERENCE sample magnitude in Direct / FIFO mode.
 *  Vpk = REFERENCE / 2047 × 3.6 V × FBratio, so ±1743 is ±95 V (high
 *  range) or ±13.28 V (low range); larger codes ask for more than the
 *  device's rated output (BOS1921 §6.10.1). The tier-2 helpers never
 *  exceed it. */
#define BOS1921_REFERENCE_MAX       1743

/** @brief  Output voltage limits (@ref tile_drive_p_set_voltage_limits), in volts.
 *  The defaults are what init leaves in force: PowerHap actuators take a
 *  high positive voltage but only about 10 V negative. */
#define DRIVE_P_VOLTAGE_LIMIT_MAX_V   95u  /**< Largest limit; at or above a range's maximum means that maximum */
#define DRIVE_P_POS_LIMIT_DEFAULT_V   95u  /**< Positive limit after init: the active range's maximum */
#define DRIVE_P_NEG_LIMIT_DEFAULT_V   10u  /**< Negative limit after init: 10 V (PowerHap-safe) */

/* -------------------------------------------------------------- */
/* Status masks                                                    */
/* -------------------------------------------------------------- */

/** @brief  STATE field in IC_STATUS (bits 9:8). */
#define BOS_STATUS_STATE_MASK       0x0300
#define BOS_STATUS_STATE_IDLE       0x0000
#define BOS_STATUS_STATE_CALIB      0x0100
#define BOS_STATUS_STATE_RUNNING    0x0200
#define BOS_STATUS_STATE_ERROR      0x0300

/** @brief  Fault bits in IC_STATUS (bits 7:2, excluding FULL and PLAYST). */
#define BOS_STATUS_FAULT_MASK       0x00FC

/** @brief  FIFO_STATE register fields (see datasheet §6.10.14). */
#define BOS_FIFO_STATE_ERROR_BIT    (1u << 12)  /**< 1=device in ERROR state (OVV/OVT/IDAC/UVLO/SC) */
#define BOS_FIFO_STATE_FULL_BIT     (1u << 11)  /**< 1=FIFO full */
#define BOS_FIFO_STATE_EMPTY_BIT    (1u << 10)  /**< 1=FIFO empty (same as PLAYST) */
#define BOS_FIFO_STATE_SPACE_MASK   0x03FFu     /**< FIFO_SPACE[9:0]; 0 with EMPTY=1 means 1024 free */

/** @brief  FIFO depth in samples (chip default; the driver never changes it). */
#define BOS1921_FIFO_DEPTH          1024

/** @brief  CONFIG register bit fields (see datasheet §6.10.6). */
#define BOS_CONFIG_GAINS_BIT        (1u << 12)  /**< 0=54.5 mV LSB, 1=7.6 mV LSB (default 1) */
#define BOS_CONFIG_GAIND_BIT        (1u << 11)  /**< 0=±95 V output, 1=±13.28 V output (default 0) */
#define BOS_CONFIG_RET_BIT          (1u << 8)   /**< 1=clear RAM/regs in SLEEP, 0=retain (default 0) */
#define BOS_CONFIG_PLAY_MODE_MASK   0x0600u     /**< PLAY_MODE[1:0] (bits 10:9): 0=Direct, 1=FIFO, 2=RAM Playback, 3=RAM Synthesis */
#define BOS_CONFIG_RST_BIT          (1u << 6)   /**< 1=software reset (self-clears) */
#define BOS_CONFIG_OE_BIT           (1u << 4)   /**< 1=output (playback or sensing) enabled */
#define BOS_CONFIG_DS_BIT           (1u << 3)   /**< power mode while OE=0: 1=SLEEP, 0=IDLE */

/** @brief  COMM register bit fields (see datasheet §6.10.12). */
#define BOS_COMM_TOUT_BIT           (1u << 5)   /**< 1=auto-sleep after 4 ms idle in Direct/FIFO */
#define BOS_COMM_GPIODIR_BIT        (1u << 6)   /**< 1=GPIO is input (write-gate for I2C readdress), 0=output */

/** @brief  SUP_RISE register fields (see datasheet §6.10.8, default 0x4967).
 *  I2C_ADDR[3:0] occupies bits [15:12] and sets the low nibble of the
 *  7-bit address (0b100_XXXX); the remaining bits are supply-rise timing. */
#define BOS_SUP_RISE_I2C_ADDR_POS   12
#define BOS_SUP_RISE_TIMING_DEFAULT 0x0967  /**< default with I2C_ADDR nibble cleared */

/** @brief  PARCAP register bit fields (see datasheet §6.10.7). */
#define BOS_PARCAP_CCM_BIT          (1u << 10)  /**< 1=chip may choose CCM or DCM (10 µH L1) */
#define BOS_PARCAP_UPI_BIT          (1u << 9)   /**< 1=Unidirectional Power Input (sink-only) */

/* -------------------------------------------------------------- */
/* Operating modes                                                 */
/* -------------------------------------------------------------- */

/**
 * @brief  Drive.P operating mode.
 *
 * Each mode configures the CONFIG register for a specific use case.
 * After init, the device is in IDLE mode.
 */
typedef enum {
    DRIVE_P_MODE_IDLE           = 0,  /**< Output disabled, status readback */
    DRIVE_P_MODE_SENSE_FINE     = 1,  /**< Piezo sensing, 7.6 mV/LSB resolution */
    DRIVE_P_MODE_SENSE_COARSE   = 2,  /**< Piezo sensing, 54.5 mV/LSB resolution */
    DRIVE_P_MODE_PLAY_DIRECT    = 3,  /**< Direct waveform output */
    DRIVE_P_MODE_PLAY_FIFO      = 4,  /**< FIFO-buffered playback, 8 ksps */
    DRIVE_P_MODE_PLAY_RAM_SYNTH = 5,  /**< RAM Synthesis waveform playback */
} drive_p_mode_t;

/**
 * @brief  Output voltage range (CONFIG.GAIND).
 *
 * High range (±95 V) is FBratio 31, the chip's reset value; low range
 * (±13.28 V) is FBratio 4.33. **The driver initializes the LOW range**;
 * the high range is only ever entered by an explicit
 * tile_drive_p_set_output_range(tile, DRIVE_P_OUTPUT_HIGH_V).
 * PARCAP, SUP_RISE.TI_RISE and ION_BL.I_ON_SCALE depend on FBratio;
 * init and set_output_range write the values for the selected range.
 */
typedef enum {
    DRIVE_P_OUTPUT_HIGH_V = 0,  /**< ±95 V (high voltage) */
    DRIVE_P_OUTPUT_LOW_V  = 1,  /**< ±13.28 V (init default) */
} drive_p_output_range_t;

/**
 * @brief  Sense-channel resolution (CONFIG.GAINS).
 *
 * Coarse is FBratio 31; fine is FBratio 4.33 (the chip default).
 */
typedef enum {
    DRIVE_P_SENSE_COARSE_GAIN = 0,  /**< Coarse (54.5 mV per step) */
    DRIVE_P_SENSE_FINE_GAIN   = 1,  /**< Fine (7.6 mV per step) */
} drive_p_sense_gain_t;

/* -------------------------------------------------------------- */
/* Public API                                                      */
/* -------------------------------------------------------------- */

/**
 * @brief  Check whether a BOS1921 is present on the I2C bus.
 *
 * @param  hal       Platform HAL handle
 * @param  instance  Instance index (0 = default, see mapping table)
 * @return 1 if device ACKs, 0 otherwise
 */
uint8_t tile_drive_p_find(tiles_pal_t* hal, uint8_t instance);

/**
 * Optional init config. Pass NULL for defaults.
 */
typedef struct {
    uint8_t reserved;   /**< Placeholder — no options yet. */
} drive_p_cfg_t;

/**
 * @brief  Initialize the BOS1921 piezoelectric driver.
 *
 * Wakes the device, performs a software reset, verifies the chip ID, and
 * configures parasitic capacitance and supply parameters for a 260nF piezo
 * on a 3.7V LiPo supply. The output is left in IDLE in the LOW range
 * (±13.28 V, CONFIG.GAIND = 1) with PARCAP / TI_RISE / I_ON_SCALE tuned for
 * it; call tile_drive_p_set_output_range(tile, DRIVE_P_OUTPUT_HIGH_V) to
 * opt in to ±95 V. The SUP_RISE I2C_ADDR nibble is derived from the
 * instance's address, so a reassigned chip (0x45/0x46) keeps its address —
 * a soft reset doesn't revert it, so init may reset such a chip normally.
 * Pass cfg=NULL for defaults.
 *
 * @param  hal       Platform HAL handle
 * @param  instance  Instance index (0=0x44, 1=0x45, 2=0x46; see mapping table)
 * @param  tile      Pointer to tile handle (populated by this function)
 * @param  cfg       Optional config, or NULL for defaults
 */
void tile_drive_p_init(tiles_pal_t* hal, uint8_t instance, tile_t* tile,
                       const drive_p_cfg_t *cfg);

/**
 * @brief  Initialize a BOS1921 at an explicit I2C address.
 *
 * The address-explicit sibling of tile_drive_p_init(): identical bring-up, but
 * you name the operating address directly instead of an instance index. This
 * decouples a chip from the fixed instance→address table, which a topology-
 * driven bringup needs — e.g. the v1 Ring runs two Drive.P on separate buses,
 * BOTH at 0x44 (no readdress), which the instance map can't express. Shadow
 * state is keyed per-tile, so two chips at the same address on different buses
 * don't collide. Pass cfg=NULL for defaults.
 *
 * @param  hal   Platform HAL handle (the bus this chip lives on)
 * @param  addr  Operating I2C address (e.g. 0x44 / 0x45 / 0x46)
 * @param  tile  Pointer to tile handle (populated by this function)
 * @param  cfg   Optional config, or NULL for defaults
 */
void tile_drive_p_init_at(tiles_pal_t* hal, uint8_t addr, tile_t* tile,
                          const drive_p_cfg_t *cfg);

/**
 * @brief  Reassign one BOS1921's I2C address (GPIO-gated, datasheet §6.3.3).
 *
 * For two BOS1921 sharing a bus (both at 0x44), move each to a unique
 * address. The call wakes the chip(s) still at cur_addr, sets COMM.GPIODIR=1
 * (GPIO becomes a write-gate), writes the new address into SUP_RISE.I2C_ADDR,
 * and verifies CHIP_ID at new_addr. Only the chip whose GPIO the caller holds
 * low latches the change; the others are untouched.
 *
 * The driver performs the I2C register sequence only; the **caller owns the
 * GPIO** (a board-specific Core pad, not reachable through the tile PAL).
 * Move BOTH chips of a pair off 0x44 (→ 0x45 and 0x46); each call gates a
 * different chip:
 *
 * @code
 *   // chip A → 0x45 : hold A's GPIO low, B's high
 *   core_pad_write(GPIO_A, 0); core_pad_write(GPIO_B, 1);
 *   tile_drive_p_reassign_address(hal, 0x44, 0x45);
 *   // chip B → 0x46 : hold B's GPIO low, A's high (A has left 0x44)
 *   core_pad_write(GPIO_A, 1); core_pad_write(GPIO_B, 0);
 *   tile_drive_p_reassign_address(hal, 0x44, 0x46);
 *   core_pad_input(GPIO_A); core_pad_input(GPIO_B);   // release
 *   // then init instance 1 (0x45) and 2 (0x46)
 * @endcode
 *
 * @note  The new address persists until power-on reset; a soft reset will
 *        not revert it. Re-running this from 0x44 after a reflash (no power
 *        cycle) is a no-op — the chips already left 0x44.
 *
 * @param  hal       Platform HAL handle
 * @param  cur_addr  the chip's current 7-bit address (0x44 at power-up)
 * @param  new_addr  desired 7-bit address; only the low nibble is settable,
 *                   so it must be in 0x40..0x4F (BOS1921_I2C_ADDR_SECOND/THIRD)
 * @return 1 if the chip answers at new_addr with the correct CHIP_ID, else 0
 */
uint8_t tile_drive_p_reassign_address(tiles_pal_t* hal, uint8_t cur_addr,
                                      uint8_t new_addr);

/**
 * @brief  Perform a software reset.
 *
 * Follows the datasheet's safe order (§6.2.8): output to 0 V (Direct / FIFO
 * modes) and OE = 0, wait for IC_STATUS.STATE to leave RUN (bounded at
 * 150 ms: a full FIFO keeps playing 128 ms after OE = 0), then RST. The
 * chip and the driver's shadow return to power-on defaults (the I2C address
 * is kept) and the tile goes to NONE: re-run init() afterwards. To reset and
 * keep your configuration, use @ref tile_drive_p_check_and_recover.
 *
 * @param  tile  Pointer to tile handle
 */
void tile_drive_p_reset(tile_t* tile);

/**
 * @brief  Set the operating mode.
 * @studio expose category=tile name=set_mode section=runtime
 *
 * @param  mode  One of the drive_p_mode_t values
 */
void tile_drive_p_set_mode(tile_t* tile, drive_p_mode_t mode);

/**
 * @brief  Read the current return register value.
 * @studio expose category=tile name=read returns=int section=runtime
 *
 * @return 16-bit value from the currently selected return register
 */
uint16_t tile_drive_p_read(tile_t* tile);

/**
 * @brief  Read the sensed piezo voltage.
 * @studio expose category=tile name=read_sense returns=int section=runtime
 *
 * Must be in SENSE_FINE or SENSE_COARSE mode.
 *
 * @return Signed 16-bit sense value (−2048 to +2047)
 */
int16_t tile_drive_p_read_sense(tile_t* tile);

/**
 * @brief  Read the IC status register.
 * @studio expose category=tile name=read_status returns=int section=runtime
 *
 * @return 16-bit IC_STATUS value
 */
uint16_t tile_drive_p_read_status(tile_t* tile);

/**
 * @brief  Write a sample to the FIFO.
 * @studio expose category=tile name=write_fifo section=advanced
 *
 * One REFERENCE write. In Direct / FIFO mode (and IDLE / sense, whose
 * PLAY_MODE is Direct) the sample is limited to the output voltage
 * limits (@ref tile_drive_p_set_voltage_limits) and counted if clamped;
 * a value beyond 12 bits clamps rather than wrapping. In the RAM modes
 * the same register takes WFS command words, which go out unchanged.
 *
 * @param  sample  Signed waveform sample in REFERENCE codes (±1743 = full scale of the range)
 */
void tile_drive_p_write_fifo(tile_t* tile, int16_t sample);

/**
 * @brief  Write a raw 16-bit value to any BOS1921 register.
 *
 * Two registers are not entirely raw. A REFERENCE (0x00) write in Direct /
 * FIFO mode is a sample: REFERENCE[11:0] is limited like write_fifo()'s.
 * A CONFIG (0x05) write updates the driver's view of the output range and
 * play mode, so the voltage limits stay correct in volts. Changing the
 * range this way does not retune PARCAP / TI_RISE / I_ON_SCALE; use
 * @ref tile_drive_p_set_output_range for that.
 *
 * @param  tile   Pointer to tile handle
 * @param  reg    8-bit register address
 * @param  value  16-bit value (sent big-endian on the wire)
 */
void tile_drive_p_write_reg(tile_t* tile, uint8_t reg, uint16_t value);

/**
 * @brief  Write a multi-word WFS command to the BOS1921.
 *
 * @studio expose category=tile name=wfs_write section=advanced
 * @studio in_buffer words type=uint16_t length_param=count
 *
 * In the RAM modes the words are WFS commands and go out unchanged, so
 * the output voltage limits do NOT cover RAM Playback / RAM Synthesis
 * waveforms: keep their samples and AMPLITUDE fields within the limits
 * yourself. In Direct / FIFO mode the words are samples (REFERENCE[11:0])
 * and are limited like write_fifo()'s.
 *
 * @param  words  Array of 16-bit words (big-endian on wire)
 * @param  count  Number of words (max 8)
 */
void tile_drive_p_wfs_write(tile_t* tile, const uint16_t* words, uint16_t count);

/**
 * @brief  Enter low-power sleep mode.
 * @studio expose category=tile name=sleep section=lifecycle
 *
 * Writes CONFIG with DS = 1 and OE = 0 (datasheet §6.2.6): the output stops
 * and the chip drops to SLEEP — ~2.4 µA with retention, ~0.6 µA without
 * (see @ref tile_drive_p_set_sleep_retention). Any I2C traffic wakes the
 * chip, so leave the bus alone until @ref tile_drive_p_wake; set_mode() and
 * the play / sense helpers call wake() for you.
 */
void tile_drive_p_sleep(tile_t* tile);

/**
 * @brief  Wake the chip from SLEEP and restore the driver's configuration.
 * @studio expose category=tile name=wake returns=bool section=lifecycle
 *
 * Follows the datasheet start-up sequence (§7.4.1 steps 3-6, §7.4.2): a
 * dummy write wakes the chip (its data is ignored, §6.2.6), a 1 ms settle
 * covers the 50 µs SLEEP→IDLE time, then CONFIG (DS = 0, OE = 0, GAINS /
 * GAIND / RET), PARCAP (with UPI), SUP_RISE and COMM (TOUT) are rewritten
 * from the driver's shadow, and CHIP_ID is read back. Rewriting everything
 * covers both retention settings: with retention off (RET = 1) the chip loses
 * its registers in SLEEP; its I2C address survives either way (§6.10.8).
 * RAM contents (RAM-playback / synthesis waveforms) are not restored — re-load
 * them after a no-retention sleep. The tile returns to IDLE mode.
 *
 * Also use it after a COMM.TOUT auto-sleep (@ref tile_drive_p_set_auto_sleep),
 * which leaves the tile READY but the chip asleep. Harmless on an awake chip.
 *
 * @return 1 if the chip answered with the expected CHIP_ID (tile READY),
 *         0 otherwise (tile ERROR, or the tile was never initialized)
 */
uint8_t tile_drive_p_wake(tile_t* tile);

/**
 * @brief  Check status and recover from error/fault states.
 * @studio expose category=tile name=check_and_recover returns=bool section=advanced
 *
 * If IC_STATUS shows STATE = ERROR or any fault bit, resets the chip in the
 * §6.2.8 order (see @ref tile_drive_p_reset; blocks up to ~150 ms while a
 * FIFO drains), then rewrites the full configuration — init's tuned PARCAP
 * (with UPI), SUP_RISE, output range, sense gain, sleep retention and
 * auto-sleep — and re-enters restore_mode. A sleeping tile is woken first.
 *
 * @param  restore_mode  Mode to re-enter after recovery
 * @return 1 if recovery was performed, 0 if device was healthy
 */
uint8_t tile_drive_p_check_and_recover(tile_t* tile, drive_p_mode_t restore_mode);

/**
 * @brief  Select the output voltage range (CONFIG.GAIND).
 * @studio expose category=tile name=set_output_range section=config
 * @studio control range label="Output voltage range" tier=basic default=DRIVE_P_OUTPUT_LOW_V hazard="high voltage (±95 V)" hazard_values=DRIVE_P_OUTPUT_HIGH_V
 *
 * The tile starts in the LOW range (±13.28 V): init sets it. The HIGH
 * range is ±95 V (190 Vpp across OUT+/OUT-), a shock and
 * piezo-damage hazard, and is only entered by calling this with
 * DRIVE_P_OUTPUT_HIGH_V. Use it only for an actuator rated for that
 * swing. (The chip's own reset value is the high range; the driver
 * overrides it at init.)
 *
 * Writes CONFIG with OE = 0 (output stops; datasheet: set OE = 0
 * before changing gain), then rewrites PARCAP, SUP_RISE.TI_RISE and
 * ION_BL.I_ON_SCALE for the new FBratio. Use from IDLE before setting
 * a play mode. On a sleeping tile it only updates the driver's shadow;
 * wake() applies it.
 *
 * @param  range  DRIVE_P_OUTPUT_HIGH_V or DRIVE_P_OUTPUT_LOW_V
 */
void tile_drive_p_set_output_range(tile_t* tile, drive_p_output_range_t range);

/**
 * @brief  Select the sense-channel resolution (CONFIG.GAINS).
 * @studio expose category=tile name=set_sense_gain section=config
 * @studio control gain label="Touch sensing resolution" tier=advanced default=DRIVE_P_SENSE_FINE_GAIN
 *
 * Fine gain (7.6 mV LSB) is the BOS1921 default and gives the
 * highest sensing resolution. Coarse gain (54.5 mV LSB) widens the
 * input range — useful when sensing high-amplitude press events
 * that would otherwise saturate at fine gain. Use from IDLE before
 * entering a sense mode.
 *
 * @param  gain  DRIVE_P_SENSE_FINE_GAIN or DRIVE_P_SENSE_COARSE_GAIN
 */
void tile_drive_p_set_sense_gain(tile_t* tile, drive_p_sense_gain_t gain);

/**
 * @brief  Configure register and RAM retention during SLEEP (CONFIG.RET).
 * @studio expose category=tile name=set_sleep_retention section=config
 * @studio control retain label="Keep settings while asleep" tier=advanced type=bool default=1
 *
 * Default is retain (~2.4 µA quiescent) so that RAM contents and
 * register configuration survive a sleep cycle. Disabling retention
 * (~0.6 µA) is useful for ultra-low-power applications; wake() rewrites
 * the driver's register configuration either way, but RAM contents are
 * lost. Set this before calling sleep().
 *
 * @param  retain  1 = retain (default), 0 = clear on sleep
 */
void tile_drive_p_set_sleep_retention(tile_t* tile, uint8_t retain);

/**
 * @brief  Enable or disable the auto-sleep timeout (COMM.TOUT).
 * @studio expose category=tile name=set_auto_sleep section=config
 * @studio control enabled label="Sleep after 4 ms idle" tier=advanced type=bool default=0
 *
 * When enabled, the device drops into SLEEP after 4 ms of bus
 * inactivity during Direct or FIFO playback. Useful for unattended
 * one-shot waveforms; harmful for long streaming playback where
 * a host gap would unexpectedly stop the output.
 *
 * @note  After a timeout-triggered sleep, PLAY_SRATE is reset to
 *        0x7 (8 ksps) — re-set the sample rate before the next
 *        playback if you were using a faster rate. The driver's tile
 *        state stays READY; the first I2C write after the timeout only
 *        wakes the chip and is discarded, so call @ref tile_drive_p_wake
 *        before the next command.
 *
 * @param  enabled  1 = auto-sleep on idle, 0 = stay awake
 */
void tile_drive_p_set_auto_sleep(tile_t* tile, uint8_t enabled);

/**
 * @brief  Enable or disable the Unidirectional Power Input (PARCAP.UPI).
 * @studio expose category=tile name=set_upi section=config
 * @studio control enabled label="Never return energy to the supply" tier=advanced type=bool default=0
 *
 * UPI forces the BOS1921 into sink-only operation: energy
 * recovered from piezo discharge is dumped instead of pushed back
 * into the supply. Useful for battery-powered designs where the
 * supply rail can't safely absorb returned energy.
 *
 * @param  enabled  1 = sink-only (UPI on), 0 = energy recovery (default)
 */
void tile_drive_p_set_upi(tile_t* tile, uint8_t enabled);

/**
 * @brief  Limit the output voltage, separately for each polarity.
 * @studio expose category=tile name=set_voltage_limits section=config
 * @studio control pos_v label="Positive voltage limit" tier=basic default=95
 * @studio control neg_v label="Negative voltage limit" tier=basic default=10 hazard="more than 10 V negative can damage PowerHap actuators" hazard_above=10
 *
 * Every output sample the driver sends (click, sine, buzz, pulse train,
 * play_samples, write_fifo, and REFERENCE samples through write_reg /
 * wfs_write in Direct / FIFO mode) is clamped to [-neg_v, +pos_v]: a
 * waveform that exceeds a limit still plays, flattened at it, and each
 * flattened sample is counted (@ref tile_drive_p_read_clamp_count).
 * RAM Playback / RAM Synthesis waveforms are not covered (see wfs_write).
 *
 * Init sets +95 V / -10 V: positive up to the range's maximum, negative
 * 10 V, because PowerHap actuators take a high positive voltage but only
 * ~10 V negative. Widen the negative limit only for an actuator rated
 * for it.
 *
 * The limits are kept in volts and converted to REFERENCE codes for the
 * active output range, rounded down so the limit in force never exceeds
 * the one asked for; set_output_range re-derives them. A limit at or
 * above the range's maximum is that maximum (±1743). Resolution: one
 * code is 54.5 mV on the ±95 V range (1 V = ~18.3 codes) and 7.6 mV on
 * the ±13.28 V range (1 V = ~131 codes); e.g. 10 V is code 183 (9.98 V)
 * high, 1313 (10.00 V) low. Driver-side only; nothing is written to the
 * chip. The limits survive set_output_range, sleep and reset; init
 * restores the defaults.
 *
 * @param  pos_v  [0..95] V Largest positive output; values above 95 mean 95
 * @param  neg_v  [0..95] V Largest negative output, as a magnitude; values above 95 mean 95
 */
void tile_drive_p_set_voltage_limits(tile_t* tile, uint8_t pos_v, uint8_t neg_v);

/**
 * @brief  The positive output voltage limit, in volts.
 * @studio expose category=tile name=get_pos_limit_v returns=int section=advanced
 *
 * As set (default 95). The limit in force is the smaller of this and the
 * active range's maximum (±95 V or ±13.28 V).
 *
 * @return Positive limit in volts (0..95)
 */
uint8_t tile_drive_p_get_pos_limit_v(tile_t* tile);

/**
 * @brief  The negative output voltage limit, in volts (a magnitude).
 * @studio expose category=tile name=get_neg_limit_v returns=int section=advanced
 *
 * As set (default 10). The limit in force is the smaller of this and the
 * active range's maximum (±95 V or ±13.28 V).
 *
 * @return Negative limit in volts, as a magnitude (0..95)
 */
uint8_t tile_drive_p_get_neg_limit_v(tile_t* tile);

/**
 * @brief  How many output samples the voltage limits flattened, then reset the count.
 * @studio expose category=tile name=read_clamp_count returns=int section=runtime
 *
 * Counts every sample clamped to +pos / -neg (@ref
 * tile_drive_p_set_voltage_limits) since init or the previous call, and
 * resets it to 0. Nonzero means the waveform hit a limit and was played
 * flattened. Saturates at 4294967295.
 *
 * @return Clamped samples since the last call
 */
uint32_t tile_drive_p_read_clamp_count(tile_t* tile);

/* ============================================================== */
/* Runtime — tier-2 idiomatic helpers                              */
/*                                                                  */
/* These compose the tier-1 surface above into "do the thing the   */
/* user wants to do" calls. They take care of mode transitions     */
/* and waveform shaping internally so callers don't need to read   */
/* the BOS1921 datasheet to get a click out of a piezo.            */
/* ============================================================== */

/**
 * @brief  Fire a single sharp tactile click.
 *
 * @studio expose category=tile name=play_click section=runtime
 *
 * Streams a 2 ms half-sine pulse through the FIFO at 8 ksps in one
 * burst, framed by 0 V samples: two before it (discharges residual
 * piezo charge, §6.2.18) and the pulse itself ends at exactly 0 V, so
 * the output rests at 0 V when the FIFO drains (§6.6). Intensity
 * scales the peak output amplitude in the configured voltage range
 * (±13.28 V after init; see @ref tile_drive_p_set_output_range to opt
 * in to ±95 V). Returns when the FIFO has
 * been written; the chip continues playing the click after the
 * call returns. The output is switched off (OE = 0) right after the
 * last sample is queued; the chip finishes the FIFO first (§6.6), then
 * idles. The next play / set_mode call turns it back on.
 *
 * @param  intensity_pct  [0..100] Percent of full-scale output (100 = ±95 V, or ±13.28 V in the low range), clamped to the voltage limits
 */
void tile_drive_p_play_click(tile_t* tile, uint8_t intensity_pct);

/**
 * @brief  Play a continuous sine wave for `ms` milliseconds.
 *
 * @studio expose category=tile name=play_sine section=runtime
 *
 * Generates and streams sine samples at 8 ksps. Frequency is
 * software-quantised to the sample rate (max useful ~3 kHz). The
 * last 2 ms fade linearly to exactly 0 V, so the output never parks
 * at a DC level (§6.6). Streaming paces on FIFO_STATE.FIFO_SPACE and
 * writes 32-sample bursts: the call returns once the last sample is
 * queued, i.e. it blocks for about (ms − 128) ms on long tones and
 * the chip plays out the final ≤ 128 ms after it returns. Gapless
 * output needs a ≥ 400 kHz bus. Stops early if the chip reports ERROR
 * or stops draining for 20 ms. Like play_click(), turns the output off
 * (OE = 0) after queuing the last sample; the chip idles once the FIFO
 * has drained.
 *
 * @param  freq_hz        [1..4000] Sine frequency in Hz (50–3000 useful range)
 * @param  intensity_pct  [0..100] Percent of full-scale output (100 = ±95 V, or ±13.28 V in the low range), clamped to the voltage limits
 * @param  ms             Duration in milliseconds
 */
void tile_drive_p_play_sine(tile_t* tile, uint16_t freq_hz,
                            uint8_t intensity_pct, uint16_t ms);

/**
 * @brief  Play a buzz (sustained mid-frequency vibration).
 *
 * @studio expose category=tile name=play_buzz section=runtime
 *
 * Convenience wrapper for @ref tile_drive_p_play_sine at 150 Hz —
 * a frequency typical small-form-factor piezo actuators feel
 * strongly at. Use `play_sine` directly if you need a specific
 * frequency.
 *
 * @param  intensity_pct  [0..100] Percent of full-scale output (100 = ±95 V, or ±13.28 V in the low range), clamped to the voltage limits
 * @param  ms             Duration in milliseconds
 */
void tile_drive_p_play_buzz(tile_t* tile, uint8_t intensity_pct, uint16_t ms);

/**
 * @brief  Play N clicks separated by gaps.
 *
 * @studio expose category=tile name=play_pulse_train section=runtime
 *
 * The classic "tick-tick-tick" pattern. Composes @ref
 * tile_drive_p_play_click with `core_delay_ms` between clicks. The
 * output stays on between clicks and goes off (OE = 0) once, after the
 * last click is queued.
 *
 * @param  intensity_pct  [0..100] Percent of full-scale output (100 = ±95 V, or ±13.28 V in the low range), clamped to the voltage limits
 * @param  count          [1..255] Number of clicks
 * @param  gap_ms         Milliseconds between successive clicks
 */
void tile_drive_p_play_pulse_train(tile_t* tile, uint8_t intensity_pct,
                                   uint8_t count, uint16_t gap_ms);

/**
 * @brief  Check whether the piezo is being touched / pressed.
 *
 * @studio expose category=tile name=is_touched returns=bool section=runtime
 *
 * Switches into sense mode (fine resolution), reads one sense
 * sample, compares the absolute value against `threshold_mv`, and
 * returns the boolean result. Leaves the chip in sense mode after
 * the call — call `tile_drive_p_set_mode(tile, DRIVE_P_MODE_IDLE)`
 * (or any play mode) to return to driving the actuator.
 *
 * @param  threshold_mv  Absolute sense voltage threshold in mV
 * @return 1 if sense > threshold (touched), 0 otherwise
 */
uint8_t tile_drive_p_is_touched(tile_t* tile, uint16_t threshold_mv);

/**
 * @brief  Block until touch detected, then fire a click.
 *
 * @studio expose category=tile name=play_on_touch section=runtime
 *
 * Polls the sense channel until `threshold_mv` is exceeded, then
 * switches into FIFO mode and plays a click via @ref
 * tile_drive_p_play_click. The classic closed-loop tactile-feedback
 * idiom — press the piezo, feel the click. Polling polls every
 * ~1 ms; returns 0 if `timeout_ms` elapses without detection.
 *
 * @param  intensity_pct  [0..100] Percent of full-scale output (100 = ±95 V, or ±13.28 V in the low range), clamped to the voltage limits
 * @param  threshold_mv   Touch threshold in mV
 * @param  timeout_ms     Maximum time to wait
 * @return 1 if a touch fired the click, 0 on timeout
 */
uint8_t tile_drive_p_play_on_touch(tile_t* tile, uint8_t intensity_pct,
                                   uint16_t threshold_mv,
                                   uint32_t timeout_ms);

/**
 * @brief  Stream a buffer of pre-computed samples through the FIFO.
 *
 * @studio expose category=tile name=play_samples section=runtime
 *
 * Switches into FIFO play mode (if not already there) and writes
 * `count` samples to REFERENCE[11:0] (signed 12-bit, two's
 * complement), 8 ksps. Each sample is clamped to the output voltage
 * limits (@ref tile_drive_p_set_voltage_limits; by default +full scale
 * / -10 V) and counted (@ref tile_drive_p_read_clamp_count); ±1743 is
 * full scale of the range. Start and end the waveform at
 * 0: the last sample stays on the output when the FIFO drains, and
 * the driver does not append one (so back-to-back calls can stream a
 * long waveform in pieces). Paced on FIFO_SPACE with 32-sample burst
 * writes; returns once the last sample is queued (up to 128 ms of it
 * still playing). Stops early if the chip reports ERROR or stops
 * draining for 20 ms. Leaves the output on (OE = 1) so calls chain;
 * call set_mode(DRIVE_P_MODE_IDLE) when done to switch it off. Use
 * this for arbitrary waveforms that don't fit the click / sine /
 * buzz / pulse-train idioms — e.g., recorded waveforms or
 * DSP-generated patterns.
 *
 * @studio in_buffer samples type=int16_t length_param=count
 * @param  samples  Buffer of signed samples in REFERENCE codes (±1743 = full scale)
 * @param  count    Number of samples to write
 */
void tile_drive_p_play_samples(tile_t* tile, const int16_t* samples,
                               uint16_t count);

/**
 * @brief  Read a buffer of sense samples.
 *
 * @studio expose category=tile name=read_sense_samples section=runtime
 * @studio out_buffer buf type=int16_t cap_param=count
 *
 * Switches into fine-resolution sense mode (if not already there)
 * and reads `count` consecutive samples into the caller's buffer.
 * Each sample takes ~125 µs to acquire (8 ksps native). Use for
 * impedance characterisation, multi-touch pattern detection, or
 * piezo-as-mic experiments beyond the simple `is_touched` API.
 *
 * @param  buf    Output buffer for signed 12-bit sense samples (7.6 mV/LSB)
 * @param  count  Number of samples to read
 */
void tile_drive_p_read_sense_samples(tile_t* tile, int16_t* buf,
                                     uint16_t count);

#endif /* INC_TILE_DRIVE_P_H_ */
