/**
 * @file   tile_drive_p.h
 * @brief  Piezoelectric haptic driver for the Drive.P tile (rev a).
 *
 * Embeds the Boréas Technologies BOS1921, a piezoelectric driver
 * with integrated high-voltage boost (190 Vpp differential),
 * waveform synthesizer, 1024-sample FIFO, and piezo sensing.
 *
 * Key specifications:
 *   - Output:      190 Vpp differential, up to 820 nF capacitive load
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
 *       tile_drive_p_set_mode(&piezo, DRIVE_P_MODE_PLAY_FIFO);
 *       tile_drive_p_write_fifo(&piezo, 0x7FFF);
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
 *   - THE CURRENT LIMIT IS COMPONENT-DEPENDENT. The parcap / SUP_RISE values set
 *     in init are tuned for a 260 nF piezo, L1 = 10 µH, Rsense = 0.2 Ω. A board
 *     with different components hits MXPWR/IDAC far below the rated ±95 V (Ring
 *     faulted ~33 V). If you see early MXPWR/IDAC at low voltage, suspect sizing:
 *     retune parcap / SUP_RISE and the Rsense current limit (§7.5.3) to the actual
 *     BOM, or cap the drive amplitude.
 *
 *   - play_samples() RETURNS WHEN THE FIFO IS FILLED, NOT DRAINED — the waveform
 *     keeps playing after it returns. Account for that in teardown / discharge
 *     timing (don't switch to SENSE or read status assuming playback is done).
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
#define TILE_DRIVE_P_VERSION_MINOR  4
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
#define BOS1921_REG_CONFIG          0x05  /**< Main configuration register */
#define BOS1921_REG_PARCAP          0x06  /**< Parasitic capacitance trim */
#define BOS1921_REG_SUP_RISE        0x07  /**< Supply rise time */
#define BOS1921_REG_COMM            0x0B  /**< Communication / return register select */
#define BOS1921_REG_IC_STATUS       0x10  /**< IC status register */
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

/** @brief  CONFIG register bit fields (see datasheet §6.10.6). */
#define BOS_CONFIG_GAINS_BIT        (1u << 12)  /**< 0=54.5 mV LSB, 1=7.6 mV LSB (default 1) */
#define BOS_CONFIG_GAIND_BIT        (1u << 11)  /**< 0=±95 V output, 1=±13.28 V output (default 0) */
#define BOS_CONFIG_RET_BIT          (1u << 8)   /**< 1=clear RAM/regs in SLEEP, 0=retain (default 0) */

/** @brief  COMM register bit fields (see datasheet §6.10.12). */
#define BOS_COMM_TOUT_BIT           (1u << 5)   /**< 1=auto-sleep after 4 ms idle in Direct/FIFO */
#define BOS_COMM_GPIODIR_BIT        (1u << 6)   /**< 1=GPIO is input (write-gate for I2C readdress), 0=output */

/** @brief  SUP_RISE register fields (see datasheet §6.10.8, default 0x4967).
 *  I2C_ADDR[3:0] occupies bits [15:12] and sets the low nibble of the
 *  7-bit address (0b100_XXXX); the remaining bits are supply-rise timing. */
#define BOS_SUP_RISE_I2C_ADDR_POS   12
#define BOS_SUP_RISE_TIMING_DEFAULT 0x0967  /**< default with I2C_ADDR nibble cleared */

/** @brief  PARCAP register bit fields (see datasheet §6.10.7). */
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
 * High range is FBratio 31 (the chip default); low range is FBratio 4.33.
 *
 * @note   Switching range invalidates the PARCAP / TI_RISE values
 *         set during init (those are computed against the configured
 *         FBratio, which is determined by GAIND). After changing the
 *         range, recompute and rewrite PARCAP and TI_RISE for the
 *         new FBratio if precise output behaviour matters.
 */
typedef enum {
    DRIVE_P_OUTPUT_HIGH_V = 0,  /**< ±95V */
    DRIVE_P_OUTPUT_LOW_V  = 1,  /**< ±13.28V */
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
 * on a 3.7V LiPo supply. The SUP_RISE I2C_ADDR nibble is derived from the
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
 * Raw REFERENCE write, no clamping (the same register takes WFS
 * command words in the RAM modes). In Direct / FIFO mode keep samples
 * within ±BOS1921_REFERENCE_MAX (±1743 = ±95 V).
 *
 * @param  sample  Signed 12-bit waveform sample (REFERENCE[11:0], two's complement)
 */
void tile_drive_p_write_fifo(tile_t* tile, int16_t sample);

/**
 * @brief  Write a raw 16-bit value to any BOS1921 register.
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
 * @param  words  Array of 16-bit words (big-endian on wire)
 * @param  count  Number of words (max 8)
 */
void tile_drive_p_wfs_write(tile_t* tile, const uint16_t* words, uint16_t count);

/**
 * @brief  Enter low-power sleep mode.
 * @studio expose category=tile name=sleep section=lifecycle
 *
 * @note  There is no wake() yet: after sleep() set_mode() refuses
 *        (tile not READY). Re-run init() to use the tile again; with
 *        retention off (RET = 1) the chip also loses PARCAP / SUP_RISE.
 *
 */
void tile_drive_p_sleep(tile_t* tile);

/**
 * @brief  Check status and recover from error/fault states.
 * @studio expose category=tile name=check_and_recover returns=bool section=advanced
 *
 * @param  restore_mode  Mode to re-enter after recovery
 * @return 1 if recovery was performed, 0 if device was healthy
 */
uint8_t tile_drive_p_check_and_recover(tile_t* tile, drive_p_mode_t restore_mode);

/**
 * @brief  Select the output voltage range (CONFIG.GAIND).
 * @studio expose category=tile name=set_output_range section=config
 * @studio control range label="Output voltage range" tier=basic default=DRIVE_P_OUTPUT_HIGH_V
 *
 * High-V (±95 V) is the BOS1921 default and suits most piezo
 * actuators. Low-V (±13.28 V) is for low-voltage piezos where the
 * full ±95 V swing would be wasteful or destructive. Use this from
 * IDLE before setting a play mode — the change takes effect on the
 * next OE-enable.
 *
 * @note  Changing range invalidates the PARCAP / TI_RISE tuning set
 *        at init. If accurate output behaviour matters, recompute
 *        and rewrite those registers for the new FBratio (see
 *        datasheet §7.5).
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
 * (~0.6 µA) is useful for ultra-low-power applications that re-init
 * on every wake anyway. Set this before calling sleep().
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
 *        playback if you were using a faster rate.
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
 * Streams a half-sine pulse through the FIFO at 8 ksps. Intensity
 * scales the peak output amplitude in the configured voltage range
 * (default ±95 V; see @ref tile_drive_p_set_output_range to switch
 * to ±13.28 V for low-voltage piezos). Returns when the FIFO has
 * been written; the chip continues playing the click after the
 * call returns.
 *
 * @param  intensity_pct  [0..100] Percent of full-scale output (100 = ±95 V, or ±13.28 V in the low range)
 */
void tile_drive_p_play_click(tile_t* tile, uint8_t intensity_pct);

/**
 * @brief  Play a continuous sine wave for `ms` milliseconds.
 *
 * @studio expose category=tile name=play_sine section=runtime
 *
 * Generates and streams sine samples at 8 ksps. Frequency is
 * software-quantised to the sample rate (max useful ~3 kHz). The
 * call blocks until the FIFO is filled; for streams longer than
 * the 1024-sample FIFO depth (~128 ms at 8 ksps) the call refills
 * as the chip drains.
 *
 * @param  freq_hz        [1..4000] Sine frequency in Hz (50–3000 useful range)
 * @param  intensity_pct  [0..100] Percent of full-scale output (100 = ±95 V, or ±13.28 V in the low range)
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
 * @param  intensity_pct  [0..100] Percent of full-scale output (100 = ±95 V, or ±13.28 V in the low range)
 * @param  ms             Duration in milliseconds
 */
void tile_drive_p_play_buzz(tile_t* tile, uint8_t intensity_pct, uint16_t ms);

/**
 * @brief  Play N clicks separated by gaps.
 *
 * @studio expose category=tile name=play_pulse_train section=runtime
 *
 * The classic "tick-tick-tick" pattern. Composes @ref
 * tile_drive_p_play_click with `core_delay_ms` between clicks.
 *
 * @param  intensity_pct  [0..100] Percent of full-scale output (100 = ±95 V, or ±13.28 V in the low range)
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
 * @param  intensity_pct  [0..100] Percent of full-scale output (100 = ±95 V, or ±13.28 V in the low range)
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
 * complement), 8 ksps. Values beyond ±BOS1921_REFERENCE_MAX (±1743,
 * the ±95 V rated output) are clamped. Start and end the waveform at
 * 0: the last sample stays on the output when the FIFO drains. Use
 * this for arbitrary waveforms that don't fit the click / sine /
 * buzz / pulse-train idioms — e.g., recorded waveforms or
 * DSP-generated patterns.
 *
 * @studio in_buffer samples type=int16_t length_param=count
 * @param  samples  Buffer of signed samples, within ±1743
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
