/**
 * @file   tile_sense_cap.h
 * @brief  Capacitive trackpad driver for the Sense.CAP tile (IQS7211A).
 * @version 1.0.0
 *
 * Azoteq IQS7211A mutual-capacitance trackpad controller: up to 2-finger
 * absolute XY tracking, relative XY, per-channel touch status, an
 * on-chip gesture engine (tap, press-and-hold, four-way swipe) and an
 * Alternate Low-Power (ALP) channel for presence wake-up.
 *
 * **Tile and surface.** The driver owns the rules the tile's PCB fixes;
 * the application supplies the electrode surface. The tile routes five
 * chip electrodes to pads: RX3 (pad 2, prox block A), RX6 (pad 9,
 * block B), TX8 (pad 8), TX9 (pad 7) and TX11 (pad 6). RX3 and RX6 are
 * RXn/TXn pins (datasheet Table 2.4) and may also act as Tx. Nothing
 * else is routed, so the largest surface is 2 Rx x 3 Tx = 6 channels
 * (or 1 Rx x 4 Tx with RX6 as a Tx). RX0/TX0 (ball F4) is tied to GND on
 * this layout and is never used; the WLCSP18 ball A1 bonds TX9 and TX10
 * together (Table 2.2 note i), so the driver only ever selects TX9.
 * A surface is described by sense_cap_surface_t (C) or chosen from the
 * layout presets with tile_sense_cap_set_layout() (Studio).
 *
 * **Baseline.** init() starts writing a tile baseline: main oscillator locked
 * to 14 MHz (18 MHz needs VDD >= 2.2 V, Table 3.2, and the tile cannot
 * know its rail), internal RF filters on (no series resistors on the
 * rev a electrodes), ALP electrodes limited to routed pins with the ALP
 * count filter and debounce on, Comms Request mode, event mode with no
 * event sources (the polled host asks for every window), no trackpad
 * cycles until a surface is set, and the power-on reset acknowledged.
 * init() never software-resets the chip. Set cfg.skip_baseline for a
 * part pre-programmed with GUI settings (datasheet §10.1.1).
 *
 * **Lifecycle (nothing blocks for long).** init() only identifies the chip
 * and returns (see its worst case). The baseline, a layout and its ATI
 * run as one job that surface_poll() or process() advance: each call
 * makes at most one register transaction that may wait for a
 * communication window, and that wait is capped at one Active report
 * period + 2 ms (never more than 20 ms), so no call blocks longer than
 * ~12 ms at the 10 ms Active rate (20 ms at most) plus one I2C
 * transaction. A window that opens later is caught by the next call. The
 * one exception is the first transactions after a chip reset, before the
 * job sets Comms Request EN: the chip clock-stretches each of them up to
 * one report period at its current rate. The job is done when
 * is_surface_ready() returns 1 (surface or not). Steady-state process()
 * makes one read attempt and returns SENSE_CAP_BUSY if no window is open.
 * Blocking helpers (configure_surface(), the setters, the live reads,
 * reset(), hw_reset()) wait within real-time budgets instead.
 *
 * **Communication (polled, Comms Request).** Rev a fits the IQS7211A on
 * an IQS7211E layout: tile pad 3 is the chip's MCLR (active-low reset,
 * 4.7k pull-up plus a capacitor on the tile) and the chip's RDY (ball
 * C3) is not connected. The host therefore polls. The baseline sets
 * Comms Request EN (Config Settings bit 4, Table A.7): instead of
 * clock-stretching an early master, the chip answers 0xEEEE outside a
 * communication window, and the host asks for the next window by
 * writing 0x00 to address 0xFF (§11.9.2). No transaction ever holds the
 * bus waiting for the chip, which matters next to a radio. One window
 * serves one transaction: the STOP ends it (§11.7), and a window the
 * host does not service closes after the I2C timeout (register 0x4A,
 * §11.6). Non-blocking calls (process(), ati_poll()) return
 * SENSE_CAP_BUSY while a window is pending; blocking helpers retry
 * within real-time budgets. A register write lands only inside a
 * window, so every setter reads its register back.
 *
 * **Resets.** process() watches Show Reset (Info Flags bit 7). After a
 * brown-out, a watchdog reset, tile_sense_cap_reset() or
 * tile_sense_cap_hw_reset(), the driver re-applies the baseline and the
 * stored surface on its own and raises SENSE_CAP_EV_RESET. A hardware
 * reset needs a host pin on tile pad 3: pass cfg.mclr (and
 * cfg.reset_on_init to pulse it at every init).
 *
 * Polling example (Cores SDK):
 * @code
 *   tile_t pad;
 *   sense_cap_cfg_t cfg = { .millis = my_millis, .mclr = my_mclr, .reset_on_init = 1,
 *                           .layout = SENSE_CAP_LAYOUT_GRID_2X3 };
 *   tile_sense_cap_init(core_tiles_pal(&core_i2c1), 0, &pad, &cfg);   // ~0.1 s
 *   while (1) {
 *       tile_sense_cap_process(&pad);  // finishes baseline, layout and ATI first
 *       if (tile_sense_cap_get_num_fingers(&pad) > 0)
 *           printf("x=%u y=%u\n", tile_sense_cap_get_finger_x(&pad, 0),
 *                                 tile_sense_cap_get_finger_y(&pad, 0));
 *       core_delay_ms(10);
 *   }
 * @endcode
 *
 * Custom surface example (blocking):
 * @code
 *   sense_cap_surface_t s = SENSE_CAP_SURFACE_DEFAULTS;  // the 2x3 grid
 *   s.tuning.touch_set_mult = 4;  s.tuning.touch_clear_mult = 2;
 *   s.alp.enable = 0;
 *   if (tile_sense_cap_configure_surface(&pad, &s) != SENSE_CAP_OK) { ... }
 * @endcode
 *
 * Datasheet: Azoteq IQS7211A, Rev v1.3, April 2025
 *
 * @studio tile label=Sense.CAP icon=⬚
 *
 * @studio event name=touch_down mask=SENSE_CAP_EV_TOUCH_DOWN
 * @studio event name=touch_up mask=SENSE_CAP_EV_TOUCH_UP
 * @studio event name=tap mask=SENSE_CAP_EV_TAP
 * @studio event name=double_tap mask=SENSE_CAP_EV_DOUBLE_TAP
 * @studio event name=long_press mask=SENSE_CAP_EV_LONG_PRESS
 * @studio event name=swipe mask=SENSE_CAP_EV_SWIPE
 * @studio event name=drag mask=SENSE_CAP_EV_DRAG
 * @studio event name=reset mask=SENSE_CAP_EV_RESET
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=advanced category="RDY / event-driven operation" section=lifecycle
 *   HW-gated (rev a): the IQS7211A's RDY (ball C3) is not routed, and tile
 *   pad 3 carries MCLR instead (the layout was drawn for the IQS7211E, whose
 *   A5 ball combines RDY and MCLR). The driver runs polled only: Comms
 *   Request mode with event mode on and every event source off, so the chip
 *   opens a window only when the host asks. Interrupt-driven wake on touch
 *   needs a board revision that routes RDY.
 *
 * @studio unsupported severity=niche category="18 MHz main oscillator" section=advanced
 *   Driver policy: the baseline locks the main oscillator to 14 MHz (Other
 *   Settings bit 4 = 0). 18 MHz needs VDD >= 2.2 V (Table 3.2) and the tile
 *   runs from 1.8-3.3 V, which the driver cannot measure. Main Osc Adj and
 *   the calibration capacitor (the rest of 0x52) are left as the chip has
 *   them.
 *
 * @studio unsupported severity=niche category="Tx short test" section=advanced
 *   Forbidden, HW-gated: RX0/TX0 (ball F4) is tied to GND on this layout, so
 *   the Tx short-test configuration (System Control bit 15, Table A.6) would
 *   drive a transmitter into ground. The driver never sets it, and
 *   write_reg refuses it, along with RX0/TX0 or TX10 in the ALP masks or the
 *   Rx/Tx map.
 *
 * @studio unsupported severity=niche category="Analog front-end settings" section=advanced
 *   Driver-deferred: charge-transfer frequency (0x58/0x59, Table A.9), Max
 *   Count, projected opamp bias, pool capacitor, NM-in-static, Cs discharge
 *   and init delay (0x5A/0x5B, Table A.10) stay at chip values. They are
 *   GUI-derived settings for a particular stack-up (§9: "only adjusted under
 *   guidance of Azoteq"). The driver only sets the RF filter bit in 0x5A and
 *   0x5B and, when asked, the ALP auto-prox cycles. write_reg reaches the
 *   rest.
 *
 * @studio unsupported severity=advanced category="Electrodes beyond the routed five" section=config
 *   HW-gated: RX0-RX2, RX4, RX5, RX7 and TX10 are not routed to pads, so
 *   the largest surface is 2 Rx x 3 Tx (6 channels) or 1 Rx x 4 Tx with RX6
 *   as a Tx. surface_begin() rejects any other pin.
 *
 * @studio unsupported severity=niche category="Hardware reset from Studio" section=lifecycle
 *   Driver-deferred: the tile's pad 3 is the chip's MCLR (4.7k pull-up on
 *   the tile, chip pull-up 180-240k, Table 4.2). The platform layer has no
 *   GPIO write, so a C program passes cfg.mclr (drive low / release) and
 *   calls hw_reset() or sets cfg.reset_on_init; a Studio program cannot
 *   supply the pin yet, so
 *   hw_reset() returns SENSE_CAP_ERR_NO_CALLBACK there. Software reset
 *   (reset()) covers every case where the chip still answers I2C.
 *
 * @studio unsupported severity=niche category="Touch-status and XY event sources" section=advanced
 *   Driver policy: with no RDY line, an event window would stall the chip
 *   for the whole I2C timeout while nobody services it, so the gesture event
 *   is the only source the driver ever enables (and only with the chip
 *   gesture engine selected). Touch-status (0x20/0x21) is read live, not
 *   cached by process().
 *
 * @studio unsupported severity=niche category="OTP settings programming" section=advanced
 *   Out of scope: programming a settings image into the part (Azoteq GUI
 *   export, §10.1.1). A pre-programmed part is supported: set
 *   cfg.skip_baseline and read get_settings_version().
 */

#ifndef INC_TILE_SENSE_CAP_H_
#define INC_TILE_SENSE_CAP_H_

#include "tiles.h"
#include <stdint.h>

/* ================================================================
 * Driver version
 * ================================================================ */

#define TILE_SENSE_CAP_VERSION_MAJOR  1
#define TILE_SENSE_CAP_VERSION_MINOR  0
#define TILE_SENSE_CAP_VERSION_PATCH  0

TILES_CHECK_VERSION(1, 0);

/* ================================================================
 * Instance mapping
 * ================================================================ */

/**
 * | Instance | Address | Part      |
 * |----------|---------|-----------|
 * | 0        | 0x56    | IQS7211A  |
 *
 * The address is fixed in silicon (datasheet §11.2), so only instance 0
 * exists on a bus.
 */
#define IQS7211A_I2C_ADDR        0x56

/** Product number reported at register 0x00 (Table A.1). */
#define IQS7211A_PRODUCT_NUMBER  763

/**
 * What the chip clocks out when the host reads outside a communication
 * window with Comms Request EN set. It is not an error code in the
 * datasheet but the chip's "no window" marker, observed on hardware.
 * Treat it as "read failed" and never write a value derived from it:
 * as Config Settings (0x51) it would set Manual Control, WDT, both
 * automatic re-ATI bits and every event source while clearing Comms
 * Request EN and Event Mode (Table A.7); as System Control (0x50) it
 * would request a software reset and the Tx short test (Table A.6).
 * write_reg() refuses to write this value.
 */
#define IQS7211A_INVALID_RESPONSE  0xEEEE

/* ================================================================
 * Register map -- 8-bit addresses, 16-bit little-endian data (§11.5)
 * ================================================================ */

/* Version / ID (read-only, Table A.1) */
#define IQS7211A_REG_PRODUCT_NUM     0x00
#define IQS7211A_REG_MAJOR_VER       0x01
#define IQS7211A_REG_MINOR_VER       0x02

/* Trackpad data (read-only) */
#define IQS7211A_REG_INFO_FLAGS      0x10
#define IQS7211A_REG_GESTURES        0x11
#define IQS7211A_REG_RELATIVE_X      0x12
#define IQS7211A_REG_RELATIVE_Y      0x13
#define IQS7211A_REG_FINGER1_X       0x14
#define IQS7211A_REG_FINGER1_Y       0x15
#define IQS7211A_REG_FINGER1_STRENGTH 0x16
#define IQS7211A_REG_FINGER1_AREA    0x17
#define IQS7211A_REG_FINGER2_X       0x18
#define IQS7211A_REG_FINGER2_Y       0x19
#define IQS7211A_REG_FINGER2_STRENGTH 0x1A
#define IQS7211A_REG_FINGER2_AREA    0x1B

/* Channel status / ALP data (read-only) */
#define IQS7211A_REG_TOUCH_STATUS_0  0x20  /**< CH15..CH0  */
#define IQS7211A_REG_TOUCH_STATUS_1  0x21  /**< CH31..CH16 */
#define IQS7211A_REG_ALP_COUNT       0x23  /**< ALP count (A + B, §5.3.2) */
#define IQS7211A_REG_ALP_LTA         0x24  /**< ALP long-term average (§5.4.2) */
#define IQS7211A_REG_ALP_COUNT_A     0x25  /**< ALP count, prox block A */
#define IQS7211A_REG_ALP_COUNT_B     0x26  /**< ALP count, prox block B */

/* ATI (§5.6) */
#define IQS7211A_REG_TP_ATI_MULTDIV  0x30  /**< Trackpad ATI multipliers/dividers (Table A.5) */
#define IQS7211A_REG_TP_ATI_COMP_DIV 0x31  /**< Trackpad ATI compensation divider (§5.6.4) */
#define IQS7211A_REG_TP_ATI_TARGET   0x32  /**< Trackpad ATI target (counts) */
#define IQS7211A_REG_TP_REF_DRIFT    0x33  /**< Reference drift limit (§5.7.2) */
#define IQS7211A_REG_TP_MIN_COUNT    0x34  /**< Minimum count re-ATI value (§5.7.2) */
#define IQS7211A_REG_REATI_RETRY     0x35  /**< Re-ATI retry time, s, max 60 (§5.7.3) */
#define IQS7211A_REG_ALP_ATI_MULTDIV 0x36  /**< ALP ATI multipliers/dividers (Table A.5) */
#define IQS7211A_REG_ALP_ATI_COMP_DIV 0x37 /**< ALP ATI compensation divider (§5.6.4) */
#define IQS7211A_REG_ALP_ATI_TARGET  0x38  /**< ALP ATI target, per prox block (§5.6.3) */
#define IQS7211A_REG_ALP_LTA_DRIFT   0x39  /**< ALP LTA drift limit (§5.7.2) */
#define IQS7211A_REG_ALP_ATI_COMP_A  0x3A  /**< ALP ATI compensation, block A (set by ATI) */
#define IQS7211A_REG_ALP_ATI_COMP_B  0x3B  /**< ALP ATI compensation, block B (set by ATI) */

/* Report rates (ms) and mode timeouts (s), §6.1 / §6.2 */
#define IQS7211A_REG_RATE_ACTIVE     0x40
#define IQS7211A_REG_RATE_IDLE_TOUCH 0x41
#define IQS7211A_REG_RATE_IDLE       0x42
#define IQS7211A_REG_RATE_LP1        0x43
#define IQS7211A_REG_RATE_LP2        0x44
#define IQS7211A_REG_TIMEOUT_ACTIVE  0x45
#define IQS7211A_REG_TIMEOUT_IDLE_TOUCH 0x46
#define IQS7211A_REG_TIMEOUT_IDLE    0x47
#define IQS7211A_REG_TIMEOUT_LP1     0x48
#define IQS7211A_REG_REF_UPDATE_TIME 0x49  /**< Reference update time, s, max 60 (§5.4.1) */
#define IQS7211A_REG_I2C_TIMEOUT     0x4A  /**< Unserviced-window timeout, ms (§11.6) */

/* Control / configuration */
#define IQS7211A_REG_SYSTEM_CONTROL  0x50  /**< Table A.6 */
#define IQS7211A_REG_CONFIG_SETTINGS 0x51  /**< Table A.7 */
#define IQS7211A_REG_OTHER_SETTINGS  0x52  /**< Table A.8 */
#define IQS7211A_REG_TOUCH_MULT      0x53  /**< high: clear mult, low: set mult (§5.5.1) */
#define IQS7211A_REG_ALP_THRESHOLD   0x54  /**< ALP output delta threshold (§5.5.2) */
#define IQS7211A_REG_ALP_DEBOUNCE    0x56  /**< high: clear debounce, low: set debounce (§5.5.3) */
#define IQS7211A_REG_TP_CONV_FREQ    0x58  /**< Trackpad conversion frequency (Table A.9) */
#define IQS7211A_REG_ALP_CONV_FREQ   0x59  /**< ALP conversion frequency (Table A.9) */
#define IQS7211A_REG_TP_HW_SETTINGS  0x5A  /**< Trackpad hardware settings (Table A.10) */
#define IQS7211A_REG_ALP_HW_SETTINGS 0x5B  /**< ALP hardware settings (Table A.10) */

/* Trackpad sizing and XY processing */
#define IQS7211A_REG_TP_SETTINGS     0x60  /**< high: total Rxs, low: trackpad settings (Table A.11) */
#define IQS7211A_REG_TP_TOUCHES      0x61  /**< high: max multi-touches, low: total Txs */
#define IQS7211A_REG_X_RESOLUTION    0x62
#define IQS7211A_REG_Y_RESOLUTION    0x63
#define IQS7211A_REG_DYN_BOTTOM_SPEED 0x64 /**< XY dynamic filter bottom speed (§7.8.2) */
#define IQS7211A_REG_DYN_TOP_SPEED   0x65  /**< XY dynamic filter top speed (§7.8.2) */
#define IQS7211A_REG_FILTER_BETAS    0x66  /**< high: static beta, low: dynamic bottom beta */
#define IQS7211A_REG_SPLIT_STATIONARY 0x67 /**< high: finger split factor, low: stationary threshold */
#define IQS7211A_REG_X_TRIM          0x68  /**< §7.9 */
#define IQS7211A_REG_Y_TRIM          0x69  /**< §7.9 */

/* ALP channel setup */
#define IQS7211A_REG_ALP_COUNT_BETA  0x70  /**< ALP count filter beta, fraction of 256 (§5.3.2) */
#define IQS7211A_REG_ALP_LTA_BETAS   0x71  /**< high: LP2 beta, low: LP1 beta, 1/2^x (§5.4.2) */
#define IQS7211A_REG_ALP_SETUP       0x72  /**< Table A.12: filter, sensing method, Rx enables */
#define IQS7211A_REG_ALP_TX_ENABLE   0x73  /**< Table A.13: Tx enable bits, one per TXn */
#define IQS7211A_REG_SETTINGS_VER    0x74  /**< high: settings major, low: minor (§10.1.1) */

/* Gestures (§8) */
#define IQS7211A_REG_GESTURE_ENABLE  0x80  /**< Table A.14 */
#define IQS7211A_REG_TAP_TIME        0x81  /**< ms   */
#define IQS7211A_REG_TAP_DISTANCE    0x82  /**< pixels */
#define IQS7211A_REG_HOLD_TIME       0x83  /**< ms, added to tap time (§8.2) */
#define IQS7211A_REG_SWIPE_TIME      0x84  /**< ms   */
#define IQS7211A_REG_SWIPE_X_DIST    0x85  /**< pixels */
#define IQS7211A_REG_SWIPE_Y_DIST    0x86  /**< pixels */
#define IQS7211A_REG_SWIPE_ANGLE     0x87  /**< low byte: 64*tan(deg) */

/* Surface geometry */
#define IQS7211A_REG_RXTX_MAP_BASE   0x90  /**< RxTx mapping <1..0> .. <11..10> (§7.1.5) */
#define IQS7211A_REG_CYCLE_BASE_0    0xA0  /**< Cycles 0-9: 3 bytes each, packed (§7.1.2) */
#define IQS7211A_REG_CYCLE_BASE_10   0xB0  /**< Cycles 10-17: 3 bytes each, packed */
#define IQS7211A_CYCLE_PROX_BYTE     0x05  /**< Fixed first byte of every cycle record */
#define IQS7211A_CHANNEL_NONE        0xFF  /**< "No channel allocated" (§7.1.2) */

/* Extended memory map (16-bit addresses, sent MSB first, §11.4.2). One
 * 16-bit value per channel, indexed by channel number. */
#define IQS7211A_REG_EXT_COUNTS      0xE000  /**< Per-channel count values */
#define IQS7211A_REG_EXT_REFS        0xE100  /**< Per-channel reference values */
#define IQS7211A_REG_EXT_DELTAS      0xE200  /**< Per-channel delta values (signed) */
#define IQS7211A_REG_EXT_ATI_COMP    0xE300  /**< Per-channel ATI compensation */

/** Comms request / terminate: write 0x00 here (§11.7, §11.9.2). */
#define IQS7211A_REG_END_COMMS       0xFF

/* ================================================================
 * Info Flags (0x10) bit masks -- Table A.2
 * ================================================================ */

#define IQS7211A_INFO_MODE_MASK      (7U << 0)   /**< Charging (power) mode */
#define IQS7211A_INFO_MODE_SHIFT     0
#define IQS7211A_INFO_ATI_ERROR      (1U << 3)   /**< Trackpad ATI failed */
#define IQS7211A_INFO_RE_ATI_OCCURRED (1U << 4)  /**< Trackpad re-ATI just completed (momentary) */
#define IQS7211A_INFO_ALP_ATI_ERROR  (1U << 5)   /**< ALP ATI failed */
#define IQS7211A_INFO_ALP_RE_ATI_OCCURRED (1U << 6) /**< ALP re-ATI just completed (momentary) */
#define IQS7211A_INFO_SHOW_RESET     (1U << 7)   /**< Reset seen, not yet acknowledged */
#define IQS7211A_INFO_NUM_FINGERS_MASK  (3U << 8) /**< 0, 1 or 2 fingers */
#define IQS7211A_INFO_NUM_FINGERS_SHIFT 8
#define IQS7211A_INFO_TP_MOVEMENT    (1U << 10)  /**< Finger movement detected */
#define IQS7211A_INFO_TOO_MANY_FINGERS (1U << 12) /**< More fingers than allowed */
#define IQS7211A_INFO_ALP_OUTPUT     (1U << 14)  /**< ALP prox/touch detected */

/* ================================================================
 * Gestures (0x11 status / 0x80 enable) bit masks -- Tables A.3, A.14
 * ================================================================ */

#define SENSE_CAP_GESTURE_SINGLE_TAP    (1U << 0)  /**< Single tap */
#define SENSE_CAP_GESTURE_PRESS_HOLD    (1U << 1)  /**< Press and hold */
#define SENSE_CAP_GESTURE_SWIPE_X_NEG   (1U << 2)  /**< Swipe in -X */
#define SENSE_CAP_GESTURE_SWIPE_X_POS   (1U << 3)  /**< Swipe in +X */
#define SENSE_CAP_GESTURE_SWIPE_Y_POS   (1U << 4)  /**< Swipe in +Y */
#define SENSE_CAP_GESTURE_SWIPE_Y_NEG   (1U << 5)  /**< Swipe in -Y */
#define SENSE_CAP_GESTURE_ALL           0x3FU      /**< Every supported gesture */

/* ================================================================
 * System Control (0x50) bits -- Table A.6
 * ================================================================ */

#define IQS7211A_CTRL_MODE_MASK      (7U << 0)   /**< Mode select (manual control only) */
#define IQS7211A_CTRL_MODE_SHIFT     0
#define IQS7211A_CTRL_TP_RESEED      (1U << 3)   /**< Reseed trackpad references */
#define IQS7211A_CTRL_ALP_RESEED     (1U << 4)   /**< Reseed the ALP LTA */
#define IQS7211A_CTRL_TP_RE_ATI      (1U << 5)   /**< Queue trackpad re-ATI; clears when done */
#define IQS7211A_CTRL_ALP_RE_ATI     (1U << 6)   /**< Queue ALP re-ATI; clears when done */
#define IQS7211A_CTRL_ACK_RESET      (1U << 7)   /**< Clear Show Reset */
#define IQS7211A_CTRL_SW_RESET       (1U << 9)   /**< Reset once the window closes (§9.3.2) */
#define IQS7211A_CTRL_TX_TEST        (1U << 15)  /**< Tx short test: forbidden on this tile */

/* ================================================================
 * Config Settings (0x51) bits -- Table A.7
 * ================================================================ */

#define IQS7211A_CFG_TP_RE_ATI_EN    (1U << 2)   /**< Auto re-ATI, trackpad */
#define IQS7211A_CFG_ALP_RE_ATI_EN   (1U << 3)   /**< Auto re-ATI, ALP */
#define IQS7211A_CFG_COMMS_REQUEST_EN (1U << 4)  /**< Request a window instead of clock stretching */
#define IQS7211A_CFG_WDT_EN          (1U << 5)   /**< Device watchdog timer */
#define IQS7211A_CFG_MANUAL_CONTROL  (1U << 7)   /**< Host drives mode switching */
#define IQS7211A_CFG_EVENT_MODE      (1U << 8)   /**< Windows only on enabled events (or requests) */
#define IQS7211A_CFG_GESTURE_EVENT   (1U << 9)   /**< Gestures raise an event */
#define IQS7211A_CFG_TP_EVENT        (1U << 10)  /**< XY change / finger up-down raises an event */
#define IQS7211A_CFG_RE_ATI_EVENT    (1U << 11)  /**< Re-ATI raises an event */
#define IQS7211A_CFG_ALP_EVENT       (1U << 13)  /**< ALP state change raises an event */
#define IQS7211A_CFG_TP_TOUCH_EVENT  (1U << 14)  /**< Channel touch-state change raises an event */

/* ================================================================
 * Other Settings (0x52), hardware settings (0x5A/0x5B), trackpad
 * settings (0x60) and ALP setup (0x72) bits -- Tables A.8, A.10-A.12
 * ================================================================ */

#define IQS7211A_OTHER_18MHZ         (1U << 4)   /**< 1 = 18 MHz main oscillator (never set here) */
#define IQS7211A_HW_RF_FILTER        (1U << 13)  /**< Internal RF filters */
#define IQS7211A_HW_LP2_AUTOPROX_SHIFT 5         /**< 0x5B bits 7-5 */
#define IQS7211A_HW_LP1_AUTOPROX_SHIFT 2         /**< 0x5B bits 4-2 */

#define IQS7211A_TP_FLIP_X           (1U << 0)  /**< Invert X output */
#define IQS7211A_TP_FLIP_Y           (1U << 1)  /**< Invert Y output */
#define IQS7211A_TP_SWITCH_XY        (1U << 2)  /**< X along Txs, Y along Rxs */
#define IQS7211A_TP_IIR_FILTER       (1U << 3)  /**< XY IIR filter (recommended on) */
#define IQS7211A_TP_IIR_STATIC       (1U << 4)  /**< Fixed (vs dynamic) IIR damping */
#define IQS7211A_TP_MAV_FILTER       (1U << 5)  /**< XY moving-average filter (recommended on) */

#define IQS7211A_ALP_SETUP_MUTUAL    (1U << 8)  /**< 1 = mutual-capacitive ALP sensing */
#define IQS7211A_ALP_SETUP_FILTER    (1U << 9)  /**< 1 = ALP count filter enabled */

/* ================================================================
 * Tile electrodes (chip pin numbers as used by the Rx/Tx map, §7.1.5)
 * ================================================================ */

#define SENSE_CAP_PIN_RX3   3   /**< Tile pad 2, prox block A; RX3/TX3 pin, usable as Tx */
#define SENSE_CAP_PIN_RX6   6   /**< Tile pad 9, prox block B; RX6/TX6 pin, usable as Tx */
#define SENSE_CAP_PIN_TX8   8   /**< Tile pad 8 */
#define SENSE_CAP_PIN_TX9   9   /**< Tile pad 7 (ball A1, bonded to TX10: only TX9 is used) */
#define SENSE_CAP_PIN_TX11  11  /**< Tile pad 6 */

/** Pins that may be an Rx on this tile (bit n = RXn). */
#define SENSE_CAP_ROUTED_RX_MASK  ((1U << 3) | (1U << 6))
/** Pins that may be a Tx on this tile (bit n = TXn). */
#define SENSE_CAP_ROUTED_TX_MASK  ((1U << 3) | (1U << 6) | (1U << 8) | (1U << 9) | (1U << 11))

#define SENSE_CAP_MAX_RX        2   /**< Routed Rx pins */
#define SENSE_CAP_MAX_TX        4   /**< Routed Tx pins when one Rx pin acts as a Tx */
#define SENSE_CAP_MAX_CHANNELS  6   /**< Largest surface: 2 Rx x 3 Tx */
#define SENSE_CAP_NUM_FINGERS   2   /**< Finger slots (§7.3) */

/* Trackpad XY filter bits for sense_cap_tuning_t.filters (0x60 bits 5-3). */
#define SENSE_CAP_FILTER_IIR         IQS7211A_TP_IIR_FILTER  /**< IIR filter on */
#define SENSE_CAP_FILTER_IIR_STATIC  IQS7211A_TP_IIR_STATIC  /**< Static (not dynamic) IIR damping */
#define SENSE_CAP_FILTER_MAV         IQS7211A_TP_MAV_FILTER  /**< Moving-average filter on */

/* ================================================================
 * Touch event bits (get_touch_events, on_event)
 *
 * The low byte is what the on_event callback delivers (the Studio event
 * dispatcher takes a uint8_t); the high byte adds the per-direction
 * swipe and pinch detail for get_touch_events().
 * ================================================================ */

#define SENSE_CAP_EV_TOUCH_DOWN   (1U << 0)   /**< A finger arrived */
#define SENSE_CAP_EV_TOUCH_UP     (1U << 1)   /**< A finger left */
#define SENSE_CAP_EV_TAP          (1U << 2)   /**< Single tap completed */
#define SENSE_CAP_EV_DOUBLE_TAP   (1U << 3)   /**< Two taps in quick succession */
#define SENSE_CAP_EV_LONG_PRESS   (1U << 4)   /**< Held still past tap time + hold time */
#define SENSE_CAP_EV_SWIPE        (1U << 5)   /**< A swipe in any direction */
#define SENSE_CAP_EV_DRAG         (1U << 6)   /**< Finger moving beyond the tap radius */
#define SENSE_CAP_EV_RESET        (1U << 7)   /**< The chip reset; baseline + surface re-applied */
#define SENSE_CAP_EV_SWIPE_LEFT   (1U << 8)   /**< Swipe toward -X */
#define SENSE_CAP_EV_SWIPE_RIGHT  (1U << 9)   /**< Swipe toward +X */
#define SENSE_CAP_EV_SWIPE_UP     (1U << 10)  /**< Swipe toward -Y */
#define SENSE_CAP_EV_SWIPE_DOWN   (1U << 11)  /**< Swipe toward +Y */
#define SENSE_CAP_EV_PINCH        (1U << 12)  /**< Two-finger spread changed */
#define SENSE_CAP_EV_SWIPE_ANY    (SENSE_CAP_EV_SWIPE_LEFT | SENSE_CAP_EV_SWIPE_RIGHT | \
                                   SENSE_CAP_EV_SWIPE_UP | SENSE_CAP_EV_SWIPE_DOWN)

/* ---- Swipe directions (sense_cap_swipe_cb_t) ---- */

#define SENSE_CAP_DIR_LEFT   0  /**< Toward -X */
#define SENSE_CAP_DIR_RIGHT  1  /**< Toward +X */
#define SENSE_CAP_DIR_UP     2  /**< Toward -Y */
#define SENSE_CAP_DIR_DOWN   3  /**< Toward +Y */

/* ================================================================
 * Enums
 * ================================================================ */

/** Result of a lifecycle call, a surface/ATI step or a guarded write. */
typedef enum {
    SENSE_CAP_OK              = 0,   /**< Done */
    SENSE_CAP_BUSY            = 1,   /**< In progress; call the poll function again */
    SENSE_CAP_ERR_STATE       = 2,   /**< Tile not initialised, or another job is running */
    SENSE_CAP_ERR_ARG         = 3,   /**< Argument out of range */
    SENSE_CAP_ERR_PIN         = 4,   /**< Electrode not routed on this tile */
    SENSE_CAP_ERR_FORBIDDEN   = 5,   /**< RX0/TX0, TX10 or the Tx short test */
    SENSE_CAP_ERR_GEOMETRY    = 6,   /**< Duplicate electrode or impossible surface */
    SENSE_CAP_ERR_COMMS       = 7,   /**< No communication window within the budget */
    SENSE_CAP_ERR_VERIFY      = 8,   /**< A register read back differently */
    SENSE_CAP_ERR_ATI         = 9,   /**< ATI did not converge */
    SENSE_CAP_ERR_TIMEOUT     = 10,  /**< Real-time budget exceeded */
    SENSE_CAP_ERR_RESET       = 11,  /**< The chip reset mid-operation (it is being re-applied) */
    SENSE_CAP_ERR_NO_CALLBACK = 12,  /**< hw_reset() without a cfg.mclr callback */
    SENSE_CAP_ERR_NOT_FOUND   = 13,  /**< No IQS7211A answered */
} sense_cap_result_t;

/** Electrode layout presets, all physically valid on the routed pins. */
typedef enum {
    SENSE_CAP_LAYOUT_NONE        = 0,  /**< No surface: nothing sensed */
    SENSE_CAP_LAYOUT_GRID_2X3    = 1,  /**< 2x3 trackpad (bench-tuned) */
    SENSE_CAP_LAYOUT_BUTTONS_2X3 = 2,  /**< 2x3 as six buttons */
    SENSE_CAP_LAYOUT_SLIDER_1X3  = 3,  /**< 3-segment slider on RX3 */
    SENSE_CAP_LAYOUT_SLIDER_1X4  = 4,  /**< 4-segment slider, RX6 as Tx */
} sense_cap_layout_t;

/** Touch sensitivity, relative to the surface's tuned multipliers. */
typedef enum {
    SENSE_CAP_SENS_LOWEST  = 0,  /**< Firm touches only (multipliers x2) */
    SENSE_CAP_SENS_LOW     = 1,  /**< Less sensitive (x1.5) */
    SENSE_CAP_SENS_NORMAL  = 2,  /**< As tuned for the surface */
    SENSE_CAP_SENS_HIGH    = 3,  /**< More sensitive (x0.75) */
    SENSE_CAP_SENS_HIGHEST = 4,  /**< Light touches (x0.5) */
} sense_cap_sensitivity_t;

/** Charging (power) mode, as reported in Info Flags (Table A.2). */
typedef enum {
    SENSE_CAP_MODE_ACTIVE     = 0,  /**< Active */
    SENSE_CAP_MODE_IDLE_TOUCH = 1,  /**< Idle-touch */
    SENSE_CAP_MODE_IDLE       = 2,  /**< Idle */
    SENSE_CAP_MODE_LP1        = 3,  /**< Low power 1 */
    SENSE_CAP_MODE_LP2        = 4,  /**< Low power 2 */
} sense_cap_mode_t;

/** Who switches the chip's power modes (§6.3). */
typedef enum {
    SENSE_CAP_POWER_AUTO         = 0,  /**< Chip switches modes itself */
    SENSE_CAP_POWER_FORCE_ACTIVE = 1,  /**< Held in Active (manual control) */
    SENSE_CAP_POWER_FORCE_LP1    = 2,  /**< Held in LP1: ALP only */
    SENSE_CAP_POWER_FORCE_LP2    = 3,  /**< Held in LP2: ALP only */
} sense_cap_power_mode_t;

/** Report-rate and timeout bundles for the automatic power modes. */
typedef enum {
    SENSE_CAP_POWER_ALWAYS_ACTIVE = 0,  /**< Always active (10 ms in every mode) */
    SENSE_CAP_POWER_RESPONSIVE    = 1,  /**< Responsive */
    SENSE_CAP_POWER_BALANCED      = 2,  /**< Balanced */
    SENSE_CAP_POWER_LOW_POWER     = 3,  /**< Low power */
} sense_cap_power_profile_t;

/** Which engine recognizes tap / hold / swipe. */
typedef enum {
    SENSE_CAP_GESTURES_DRIVER = 0,  /**< Driver recognizers */
    SENSE_CAP_GESTURES_CHIP   = 1,  /**< Chip gesture engine */
} sense_cap_gesture_source_t;

/** What ati_start() tunes. */
typedef enum {
    SENSE_CAP_ATI_TRACKPAD = 1,  /**< Trackpad channels */
    SENSE_CAP_ATI_ALP      = 2,  /**< ALP wake channel (runs in LP1) */
    SENSE_CAP_ATI_BOTH     = 3,  /**< Trackpad, then ALP */
} sense_cap_ati_target_t;

/** ALP auto-prox cycles per low-power report (0x5B, Table A.10). */
typedef enum {
    SENSE_CAP_AUTOPROX_KEEP = 0,  /**< Leave the chip's setting */
    SENSE_CAP_AUTOPROX_4    = 1,  /**< 4 cycles */
    SENSE_CAP_AUTOPROX_8    = 2,  /**< 8 cycles */
    SENSE_CAP_AUTOPROX_16   = 3,  /**< 16 cycles */
    SENSE_CAP_AUTOPROX_32   = 4,  /**< 32 cycles */
    SENSE_CAP_AUTOPROX_OFF  = 5,  /**< Auto-prox disabled */
} sense_cap_auto_prox_t;

/** Finger slot for the absolute-XY getters. */
typedef enum {
    SENSE_CAP_FINGER_1 = 0,  /**< First tracked finger */
    SENSE_CAP_FINGER_2 = 1,  /**< Second tracked finger */
} sense_cap_finger_t;

/** Phase of a touch event, iOS/Android style. */
typedef enum {
    SENSE_CAP_TOUCH_DOWN  = 0,  /**< Finger arrived */
    SENSE_CAP_TOUCH_MOVED = 1,  /**< Position changed while down */
    SENSE_CAP_TOUCH_UP    = 2,  /**< Finger left */
} sense_cap_phase_t;

/* ================================================================
 * Callbacks
 * ================================================================ */

/**
 * Event callback fired by tile_sense_cap_process() when a read produced
 * events. `events` is the low byte of the SENSE_CAP_EV_* bits (touch
 * down/up, tap, double tap, long press, swipe, drag, reset); the full
 * set, with swipe directions and pinch, is latched for
 * tile_sense_cap_get_touch_events(). Runs in main-loop context.
 */
typedef void (*sense_cap_event_cb_t)(tile_t *tile, uint8_t events, void *ctx);

/**
 * One touch event, produced by process() from finger-state transitions
 * and consumed via on_touch() or next_touch_event(). Velocities are in
 * resolution pixels per second from the cfg.millis clock (or a report-
 * rate estimate when none is given).
 */
typedef struct {
    uint8_t  phase;     /**< sense_cap_phase_t value. */
    uint8_t  finger;    /**< Finger slot, 0 or 1. */
    uint16_t x;         /**< Position in the configured X resolution. */
    uint16_t y;         /**< Position in the configured Y resolution. */
    int16_t  dx;        /**< X movement since the previous event. */
    int16_t  dy;        /**< Y movement since the previous event. */
    int16_t  vx;        /**< X velocity, pixels/second. */
    int16_t  vy;        /**< Y velocity, pixels/second. */
    uint16_t strength;  /**< Touch strength at this event. */
    uint32_t t_ms;      /**< Timestamp, driver clock. */
} sense_cap_touch_t;

/** Raw touch-event stream callback (every DOWN/MOVED/UP). */
typedef void (*sense_cap_touch_cb_t)(tile_t *tile,
                                     const sense_cap_touch_t *ev, void *ctx);
/** Tap callback. taps = 1 (single) or 2 (double). */
typedef void (*sense_cap_tap_cb_t)(tile_t *tile, uint8_t taps,
                                   uint16_t x, uint16_t y, void *ctx);
/** Long-press callback, fired once per hold. */
typedef void (*sense_cap_hold_cb_t)(tile_t *tile,
                                    uint16_t x, uint16_t y, void *ctx);
/** Swipe callback. direction = SENSE_CAP_DIR_*. */
typedef void (*sense_cap_swipe_cb_t)(tile_t *tile, uint8_t direction,
                                     int16_t vx, int16_t vy, void *ctx);
/** Drag (pan) callback, fired on every movement while dragging. */
typedef void (*sense_cap_drag_cb_t)(tile_t *tile, int16_t dx, int16_t dy,
                                    uint16_t x, uint16_t y, void *ctx);
/** Two-finger pinch callback. delta > 0 = spreading, < 0 = pinching in. */
typedef void (*sense_cap_pinch_cb_t)(tile_t *tile, int16_t delta, void *ctx);

/* ================================================================
 * Surface description
 * ================================================================ */

/**
 * Which routed electrodes form the surface, and how XY is reported.
 * Channel numbers follow §5.1.1: along the Rxs first, then the next Tx
 * (channel = tx_index * n_rx + rx_index).
 */
typedef struct {
    uint8_t  n_rx;                      /**< Rx electrodes, 1-2. */
    uint8_t  n_tx;                      /**< Tx electrodes, 1-4. */
    uint8_t  rx[SENSE_CAP_MAX_RX];      /**< Rx pins in surface order (SENSE_CAP_PIN_RX3/RX6). */
    uint8_t  tx[SENSE_CAP_MAX_TX];      /**< Tx pins in surface order (TX8/TX9/TX11, or RX3/RX6 as Tx). */
    uint8_t  disabled_channels;         /**< Bit n = channel n is not sensed (§7.1.4). */
    uint8_t  switch_xy;                 /**< 1 = X along the Txs, Y along the Rxs (§7.7). */
    uint8_t  flip_x;                    /**< 1 = invert X (§7.7). */
    uint8_t  flip_y;                    /**< 1 = invert Y (§7.7). */
    uint16_t x_res;                     /**< X output range; 0 = 256 per electrode pitch (§7.4). */
    uint16_t y_res;                     /**< Y output range; 0 = 256 per electrode pitch. */
    uint16_t x_trim;                    /**< X edge trim (§7.9); 0 = keep the chip's. */
    uint16_t y_trim;                    /**< Y edge trim; 0 = keep the chip's. */
} sense_cap_geometry_t;

/**
 * Trackpad tuning. A field left 0 keeps the chip's value, except the
 * multipliers, filters and max touches, which are always written.
 */
typedef struct {
    uint16_t ati_target;        /**< Trackpad ATI target, counts (0x32, §5.6.3). */
    uint8_t  ati_fine_div;      /**< ATI fine divider 1-31 (0x30 [13:9]); fine/mult/div all 0 = keep. */
    uint8_t  ati_coarse_mult;   /**< ATI coarse multiplier 1-15 (0x30 [8:5]). */
    uint8_t  ati_coarse_div;    /**< ATI coarse divider 1-31 (0x30 [4:0]). */
    uint16_t ati_comp_div;      /**< ATI compensation divider (0x31, §5.6.4). */
    uint8_t  touch_set_mult;    /**< Touch-set multiplier 1-255: threshold = ref x (1 + m/128) (§5.5.1). */
    uint8_t  touch_clear_mult;  /**< Touch-clear multiplier, <= set (hysteresis). */
    uint8_t  filters;           /**< SENSE_CAP_FILTER_* bits (0x60 [5:3]); MAV + IIR recommended (§7.8). */
    uint8_t  max_touches;       /**< Fingers tracked, 1-2 (0x61 high byte, §7.3). */
    uint16_t dyn_bottom_speed;  /**< Dynamic IIR bottom speed, px/cycle (0x64). */
    uint16_t dyn_top_speed;     /**< Dynamic IIR top speed, px/cycle (0x65). */
    uint8_t  dyn_bottom_beta;   /**< Dynamic IIR bottom beta /256 (0x66 low). */
    uint8_t  static_beta;       /**< Static IIR beta /256 (0x66 high). */
    uint16_t ref_drift_limit;   /**< Re-ATI reference drift limit (0x33, §5.7.2). */
    uint16_t min_count_reati;   /**< Minimum-count re-ATI value (0x34, §5.7.2). */
    uint16_t reati_retry_s;     /**< Re-ATI retry time after an ATI error, 1-60 s (0x35). */
    uint16_t ref_update_s;      /**< Reference update time, 1-60 s (0x49, §5.4.1). */
    uint8_t  finger_split;      /**< Finger split factor (0x67 high, §7.6). */
    uint8_t  stationary_thr;    /**< Stationary-touch movement threshold, px (0x67 low, §7.5). */
} sense_cap_tuning_t;

/**
 * Alternate low-power (ALP) wake channel (§5.2). The chip always senses
 * the ALP in LP1/LP2 and has no "off" switch, so the driver always
 * limits its electrodes to routed pins; `enable` says whether the ALP
 * is tuned and may be used for wake. A field left 0 keeps the chip's
 * value. The count filter is always on (rev a has no series resistors).
 */
typedef struct {
    uint8_t  enable;            /**< 1 = tune the ALP and allow ALP wake. */
    uint8_t  mutual;            /**< 1 = mutual (Rx + Tx), 0 = self-capacitive (Rx only). */
    uint8_t  rx_mask;           /**< Bit n = RXn in the ALP (RX3/RX6 only); 0 = the surface's Rxs. */
    uint16_t tx_mask;           /**< Bit n = TXn in the ALP; 0 = the surface's Txs (mutual only). */
    uint16_t ati_target;        /**< ALP ATI target per prox block, counts (0x38). */
    uint8_t  ati_fine_div;      /**< ALP ATI fine divider (0x36); all three 0 = the trackpad's. */
    uint8_t  ati_coarse_mult;   /**< ALP ATI coarse multiplier (0x36). */
    uint8_t  ati_coarse_div;    /**< ALP ATI coarse divider (0x36). */
    uint16_t ati_comp_div;      /**< ALP ATI compensation divider (0x37). */
    uint16_t lta_drift_limit;   /**< ALP LTA drift limit for re-ATI (0x39). */
    uint16_t threshold;         /**< ALP output threshold, delta counts (0x54, §5.5.2). */
    uint8_t  set_debounce;      /**< ALP set debounce (0x56 low). */
    uint8_t  clear_debounce;    /**< ALP clear debounce (0x56 high). */
    uint16_t count_beta;        /**< ALP count filter beta /256 (0x70). */
    uint8_t  lp1_lta_beta;      /**< LTA beta in LP1, 1/2^x (0x71 low). */
    uint8_t  lp2_lta_beta;      /**< LTA beta in LP2, 1/2^x (0x71 high). */
    uint8_t  lp1_auto_prox;     /**< sense_cap_auto_prox_t for LP1 (0x5B). */
    uint8_t  lp2_auto_prox;     /**< sense_cap_auto_prox_t for LP2 (0x5B). */
} sense_cap_alp_t;

/** A complete electrode surface: geometry, trackpad tuning and ALP. */
typedef struct {
    sense_cap_geometry_t geometry;  /**< Which electrodes, XY orientation and range. */
    sense_cap_tuning_t   tuning;    /**< ATI, thresholds, filters. */
    sense_cap_alp_t      alp;       /**< ALP wake channel. */
} sense_cap_surface_t;

/**
 * Initializer for sense_cap_surface_t: the SENSE_CAP_LAYOUT_GRID_2X3
 * preset. Two Rx strips (RX3 top, RX6 bottom) across three Tx columns
 * (TX11, TX9, TX8 = tile pads 6, 7, 8, left to right), X along the Txs.
 * Trackpad tuning characterised on hardware 2026-08-06 (socket and
 * breadboard attachments): ATI base fine 1 / mult 1 / div 31 with target
 * 900, where a finger gives ~100-190 delta counts; thresholds 8/5. The
 * ALP numbers (target 200, the trackpad's base, threshold 20, debounce
 * 4/4) are derived, not bench-tuned.
 */
#define SENSE_CAP_SURFACE_DEFAULTS {                                          \
    .geometry = {                                                             \
        .n_rx = 2, .n_tx = 3,                                                 \
        .rx = { SENSE_CAP_PIN_RX3, SENSE_CAP_PIN_RX6 },                       \
        .tx = { SENSE_CAP_PIN_TX11, SENSE_CAP_PIN_TX9, SENSE_CAP_PIN_TX8, 0 },\
        .disabled_channels = 0, .switch_xy = 1, .flip_x = 0, .flip_y = 0,     \
        .x_res = 0, .y_res = 0, .x_trim = 0, .y_trim = 0,                     \
    },                                                                        \
    .tuning = {                                                               \
        .ati_target = 900,                                                    \
        .ati_fine_div = 1, .ati_coarse_mult = 1, .ati_coarse_div = 31,        \
        .ati_comp_div = 0,                                                    \
        .touch_set_mult = 8, .touch_clear_mult = 5,                           \
        .filters = SENSE_CAP_FILTER_MAV | SENSE_CAP_FILTER_IIR,               \
        .max_touches = 2,                                                     \
    },                                                                        \
    .alp = {                                                                  \
        .enable = 1, .mutual = 1, .rx_mask = 0, .tx_mask = 0,                 \
        .ati_target = 200, .threshold = 20,                                   \
        .set_debounce = 4, .clear_debounce = 4,                               \
    },                                                                        \
}

/* ================================================================
 * Configuration
 * ================================================================ */

/**
 * Optional init config. Pass NULL for defaults: no callbacks, the tile
 * baseline started, no surface, no MCLR.
 */
typedef struct {
    sense_cap_event_cb_t on_event;  /**< Event callback (SENSE_CAP_EV_* low byte). NULL = none. */
    void *event_ctx;                /**< User context for on_event. */

    /** Monotonic ms clock (e.g. wrap core_millis()). Gives real-time
     *  timeouts and honest velocities; NULL = the driver counts its own
     *  delays and estimates one report period per successful read. */
    uint32_t (*millis)(void *ctx);
    void *millis_ctx;               /**< Context for millis. */

    /** Drive tile pad 3 (the chip's MCLR): level 0 = pull low, 1 =
     *  release. Use an open-drain pin; the tile pulls the line up.
     *  NULL = no hardware reset (hw_reset() returns ERR_NO_CALLBACK). */
    void (*mclr)(void *ctx, uint8_t level);
    void *mclr_ctx;                 /**< Context for mclr. */

    uint8_t skip_baseline;          /**< 1 = leave a GUI-programmed part's settings alone (§10.1.1). */
    sense_cap_layout_t layout;      /**< Layout to begin after init (process() finishes it). */

    /** 1 = init() pulses MCLR through cfg.mclr before looking for the
     *  chip, so it starts from its power-on state whatever the MCU did
     *  before (a warm MCU reset leaves the chip configured). Costs the
     *  boot wait (~0.1 s). Needs cfg.mclr; ignored without it. 0 = pulse
     *  only if the chip does not answer. */
    uint8_t reset_on_init;
} sense_cap_cfg_t;

/* ================================================================
 * Lifecycle
 * ================================================================ */

/**
 * @brief  Look for an IQS7211A on the bus.
 *
 * Reads the product number, asking for a communication window when the
 * chip answers 0xEEEE (a bare address probe goes unanswered outside a
 * window). Blocks up to ~0.4 s.
 *
 * @param  hal       Tiles HAL handle (I2C bus)
 * @param  instance  Device instance (only 0 exists)
 * @return 1 if the chip reported product 763, or answered with the
 *         no-window marker; 0 if nothing answered.
 */
uint8_t tile_sense_cap_find(tiles_pal_t *hal, uint8_t instance);

/**
 * @brief  Initialize the trackpad controller and start the tile baseline.
 *
 * Identifies the chip (product 763) and returns: the tile is READY, and
 * the baseline (see the file header) is a job that surface_poll() or
 * process() advance, one register transaction per call, until
 * is_surface_ready() returns 1. With cfg.layout set, the same job also
 * applies and tunes that layout. Setters called meanwhile are queued into
 * the job. A baseline that does not verify is reported by surface_poll()
 * / process() (the error, and TILE_ON_ERROR) and leaves is_surface_ready()
 * at 0; the tile stays READY (only a chip that is not found sets ERROR).
 * Never software-resets the chip.
 *
 * Blocking: the product read, which waits at most ~0.4 s for a
 * communication window (typically one transaction). With
 * cfg.reset_on_init it pulses MCLR first and waits for the boot instead
 * (0.1 s, at most ~0.7 s). Without it, a chip that does not answer gets
 * one MCLR pulse if cfg.mclr is set (worst case ~1.1 s, chip absent).
 *
 * Blocking use: `while (tile_sense_cap_surface_poll(&pad) == SENSE_CAP_BUSY)
 * core_delay_ms(2);` or configure_surface(), which also finishes the
 * baseline.
 *
 * @param  hal       Tiles HAL handle (I2C bus)
 * @param  instance  Device instance (only 0 exists)
 * @param  tile      Tile handle to initialize
 * @param  cfg       Optional config. NULL for defaults.
 */
void tile_sense_cap_init(tiles_pal_t *hal, uint8_t instance,
                         tile_t *tile, const sense_cap_cfg_t *cfg);

/**
 * @brief  Fill a surface struct with a layout preset.
 *
 * For customising a preset before surface_begin(). SENSE_CAP_LAYOUT_NONE
 * gives the routed 2x3 geometry with every channel disabled.
 *
 * @param  layout  Preset
 * @param  out     Filled with the preset
 * @return SENSE_CAP_OK, or SENSE_CAP_ERR_ARG for an unknown layout.
 */
sense_cap_result_t tile_sense_cap_layout_preset(sense_cap_layout_t layout,
                                                sense_cap_surface_t *out);

/**
 * @brief  Validate a surface against the tile and start applying it.
 *
 * Checks the tile rules (routed pins only; never RX0/TX0 or TX10; no pin
 * both Rx and Tx; ALP masks on routed pins; tuning ranges), copies the
 * surface and starts the apply job: register sync (only registers that
 * differ are written, cycles cleared before a geometry change), then
 * trackpad ATI, then ALP ATI if ALP wake is on. Non-blocking: advance it
 * with surface_poll() or process() until is_surface_ready(). Replaces a
 * job still running (the baseline from init included; the new job
 * carries the whole register image). The copy is re-applied after any
 * chip reset.
 *
 * @param  tile  Initialised tile handle
 * @param  surf  Surface description
 * @return SENSE_CAP_OK when accepted, or the validation error.
 */
sense_cap_result_t tile_sense_cap_surface_begin(tile_t *tile,
                                                const sense_cap_surface_t *surf);

/**
 * @brief  Apply a surface and wait until it is tuned (blocking).
 *
 * surface_begin() plus surface_poll() until done, bounded by 20 s of
 * real time. Also finishes a baseline still pending from init().
 *
 * @param  tile  Initialised tile handle
 * @param  surf  Surface description
 * @return SENSE_CAP_OK when applied and ATI converged; otherwise the error.
 */
sense_cap_result_t tile_sense_cap_configure_surface(tile_t *tile,
                                                    const sense_cap_surface_t *surf);

/**
 * @brief  Choose the electrode layout (non-blocking).
 * @studio expose category=tile name=set_layout returns=int section=lifecycle
 * @studio control layout label="Surface layout" tier=basic default=SENSE_CAP_LAYOUT_NONE
 *
 * Starts applying a preset: GRID_2X3 is the bench-tuned 2x3 trackpad
 * (X 0-511, Y 0-255); BUTTONS_2X3 is the same wiring read as six channel
 * touch states; SLIDER_1X3 uses RX3 across TX11/TX9/TX8 (X 0-511);
 * SLIDER_1X4 adds RX6 as a fourth Tx (X 0-767). Only GRID_2X3 is
 * bench-tuned. Call process() (or surface_poll()) until
 * is_surface_ready(); the ATI takes a second or two. NONE stops sensing;
 * it is ready once its registers verify.
 *
 * @param  layout  Preset (init: NONE)
 * @return SENSE_CAP_OK when started, or an error.
 */
sense_cap_result_t tile_sense_cap_set_layout(tile_t *tile, sense_cap_layout_t layout);

/**
 * @brief  Advance the baseline / surface job: registers, then ATI steps.
 * @studio expose category=tile name=surface_poll returns=int section=lifecycle
 *
 * Each call makes at most one register transaction, whose wait for a
 * communication window is capped at one Active report period + 2 ms
 * (20 ms at most): ~12 ms plus the transaction at the 10 ms Active rate.
 * Returns SENSE_CAP_BUSY until the job is done (baseline verified; with a
 * surface, applied and tuned). process() calls it for you while a job
 * runs. Nothing to do: SENSE_CAP_OK.
 *
 * @return SENSE_CAP_OK when done, SENSE_CAP_BUSY, or the job's error.
 */
sense_cap_result_t tile_sense_cap_surface_poll(tile_t *tile);

/**
 * @brief  Is the tile ready: no job pending, baseline applied, surface tuned?
 * @studio expose category=tile name=is_surface_ready returns=bool section=lifecycle
 *
 * 1 once no baseline / surface job or ATI is pending and the last job
 * verified the register image; with a surface (any layout but NONE) it
 * must also have been applied with the ATI converged. With layout NONE it
 * becomes 1 as soon as the baseline from init() is applied. 0 again while
 * a new job (set_layout(), a setter's re-sync after a chip reset) or an
 * ATI runs, and after a failed job.
 *
 * @return 1 if ready, 0 otherwise.
 */
uint8_t tile_sense_cap_is_surface_ready(tile_t *tile);

/**
 * @brief  Queue auto-tuning (ATI) and return at once.
 * @studio expose category=tile name=ati_start returns=int section=lifecycle
 *
 * Trackpad ATI runs when the trackpad is next sensed; ALP ATI only runs
 * in LP1/LP2 (§5.6.3), so the driver holds the chip in LP1 under manual
 * control for it and then hands mode control back. Re-run after any
 * mechanical change (cover, enclosure). Advance with ati_poll().
 *
 * @param  target  Trackpad, ALP or both
 * @return SENSE_CAP_OK when queued, or SENSE_CAP_ERR_STATE if a job runs.
 */
sense_cap_result_t tile_sense_cap_ati_start(tile_t *tile, sense_cap_ati_target_t target);

/**
 * @brief  Advance an ATI started with ati_start().
 * @studio expose category=tile name=ati_poll returns=int section=lifecycle
 *
 * Stays off the bus for 100 ms after queuing, then checks at most every
 * 50 ms whether the re-ATI bit has cleared (the chip clears it when the
 * algorithm completes, §5.6.3), so it never starves the ATI. The verdict
 * is every sensed channel within 25% of the target; up to three rounds.
 * One register transaction per call, like surface_poll().
 *
 * @return SENSE_CAP_OK, SENSE_CAP_BUSY, SENSE_CAP_ERR_ATI or another error.
 */
sense_cap_result_t tile_sense_cap_ati_poll(tile_t *tile);

/**
 * @brief  Hardware-reset the chip through tile pad 3 (MCLR).
 * @studio expose category=tile name=hw_reset returns=int section=lifecycle
 *
 * Pulls MCLR low for 2 ms (at least 250 ns needed, Table 4.2), releases
 * it and waits for the chip to boot (0.1 s, at most ~0.7 s), then starts
 * re-applying the baseline and the stored surface (finish with
 * surface_poll() or process()). Recovers a chip that no longer answers
 * I2C. Needs a tile that init() has seen (READY, or ERROR after a failed
 * init); to reset before init, set cfg.reset_on_init.
 *
 * @return SENSE_CAP_OK once the chip is back, SENSE_CAP_ERR_NO_CALLBACK
 *         without cfg.mclr, SENSE_CAP_ERR_STATE before init(), or
 *         SENSE_CAP_ERR_NOT_FOUND.
 */
sense_cap_result_t tile_sense_cap_hw_reset(tile_t *tile);

/**
 * @brief  Software-reset the chip (System Control SW Reset, §9.3.2).
 * @studio expose category=tile name=reset returns=int section=lifecycle
 *
 * The reset takes effect when the window closes. Waits for the reboot,
 * clears the cached data, then starts re-applying the baseline and the
 * stored surface (finish with surface_poll() or process()).
 *
 * @return SENSE_CAP_OK once the chip is back, or an error.
 */
sense_cap_result_t tile_sense_cap_reset(tile_t *tile);

/**
 * @brief  Firmware major version (cached at init).
 * @studio expose category=tile name=get_version_major returns=int section=lifecycle
 * @return Major version from register 0x01
 */
uint16_t tile_sense_cap_get_version_major(tile_t *tile);

/**
 * @brief  Firmware minor version (cached at init).
 * @studio expose category=tile name=get_version_minor returns=int section=lifecycle
 * @return Minor version from register 0x02
 */
uint16_t tile_sense_cap_get_version_minor(tile_t *tile);

/**
 * @brief  Settings-version label of a GUI-programmed part (register 0x74).
 * @studio expose category=tile name=get_settings_version returns=int section=lifecycle
 *
 * High byte settings major, low byte minor (§10.1.1). Live read.
 *
 * @return Raw 16-bit settings version, or 0 on failure
 */
uint16_t tile_sense_cap_get_settings_version(tile_t *tile);

/* ================================================================
 * Event processing
 * ================================================================ */

/**
 * @brief  Read the trackpad data and update state (non-blocking).
 * @studio expose category=tile name=process returns=int section=runtime
 *
 * Call from the main loop, ideally once per report period. One read
 * attempt of 0x10-0x1B: when the window is not open yet it asks for one
 * and returns SENSE_CAP_BUSY without waiting. After a good read it asks
 * for the next window straight away (in the awake modes), so the next
 * call usually lands. Runs the touch recognizers, fires callbacks and
 * handles a chip reset. A Show Reset flag the chip will not clear costs
 * one more capped transaction (a Config Settings check, or a re-ack once
 * a second). While the baseline / surface job or an ATI runs it advances
 * that instead (surface_poll(): one transaction, capped wait) and returns
 * SENSE_CAP_BUSY until it finishes.
 *
 * Worst case per call: ~12 ms plus one I2C transaction at the 10 ms
 * Active rate (window wait capped at 20 ms whatever the rates), except
 * right after a chip reset (see the file header).
 *
 * @return SENSE_CAP_OK (new data, or a job just finished), SENSE_CAP_BUSY,
 *         SENSE_CAP_ERR_RESET (chip reset seen; re-applying), or an error.
 */
sense_cap_result_t tile_sense_cap_process(tile_t *tile);

/**
 * @brief  Register or change the event callback.
 * @studio expose category=tile name=on_event section=runtime
 * @param  cb    Callback (NULL to disable)
 * @param  ctx   User context passed to the callback
 */
void tile_sense_cap_on_event(tile_t *tile, sense_cap_event_cb_t cb, void *ctx);

/**
 * @brief  Register the raw touch-event stream callback (DOWN/MOVED/UP).
 * @param  tile  Tile handle
 * @param  cb    Callback (NULL to disable)
 * @param  ctx   User context
 */
void tile_sense_cap_on_touch(tile_t *tile, sense_cap_touch_cb_t cb, void *ctx);

/**
 * @brief  Pop the oldest queued touch event (8-deep ring, oldest dropped).
 * @param  tile  Tile handle
 * @param  ev    Filled with the event when one was available
 * @return 1 if an event was returned, 0 if the queue is empty.
 */
uint8_t tile_sense_cap_next_touch_event(tile_t *tile, sense_cap_touch_t *ev);

/**
 * @brief  Register the tap callback (single and double).
 * @param  tile  Tile handle
 * @param  cb    Callback (NULL to disable)
 * @param  ctx   User context
 */
void tile_sense_cap_on_tap(tile_t *tile, sense_cap_tap_cb_t cb, void *ctx);

/**
 * @brief  Register the long-press callback.
 * @param  tile  Tile handle
 * @param  cb    Callback (NULL to disable)
 * @param  ctx   User context
 */
void tile_sense_cap_on_long_press(tile_t *tile, sense_cap_hold_cb_t cb, void *ctx);

/**
 * @brief  Register the swipe callback.
 * @param  tile  Tile handle
 * @param  cb    Callback (NULL to disable)
 * @param  ctx   User context
 */
void tile_sense_cap_on_swipe(tile_t *tile, sense_cap_swipe_cb_t cb, void *ctx);

/**
 * @brief  Register the drag (pan) callback.
 * @param  tile  Tile handle
 * @param  cb    Callback (NULL to disable)
 * @param  ctx   User context
 */
void tile_sense_cap_on_drag(tile_t *tile, sense_cap_drag_cb_t cb, void *ctx);

/**
 * @brief  Register the two-finger pinch callback.
 * @param  tile  Tile handle
 * @param  cb    Callback (NULL to disable)
 * @param  ctx   User context
 */
void tile_sense_cap_on_pinch(tile_t *tile, sense_cap_pinch_cb_t cb, void *ctx);

/* ================================================================
 * Runtime data (cached from the last successful process())
 * ================================================================ */

/**
 * @brief  Read and clear the accumulated touch-event bits.
 * @studio expose category=tile name=get_touch_events returns=int section=runtime
 * @return Latched SENSE_CAP_EV_* bits since the previous call.
 */
uint16_t tile_sense_cap_get_touch_events(tile_t *tile);

/**
 * @brief  Did a tap (single or double) complete since the last check?
 * @studio expose category=tile name=was_tapped returns=bool section=runtime
 * @return 1 if a tap completed (consumes the tap bits), 0 otherwise.
 */
uint8_t tile_sense_cap_was_tapped(tile_t *tile);

/**
 * @brief  Which channel (zone) is under the first finger?
 * @studio expose category=tile name=get_zone returns=int section=runtime
 * @return Channel number (§5.1.1), or -1 with no finger or no surface.
 */
int8_t tile_sense_cap_get_zone(tile_t *tile);

/**
 * @brief  Which channel (zone) is nearest a position?
 * @studio expose category=tile name=zone_at returns=int section=runtime
 *
 * Electrode centres sit 256 points apart in chip units (§7.4), i.e. at
 * k * res / (n - 1) in the configured resolution; the nearest centre on
 * each axis wins, with Flip X/Y undone (§7.7). Use it in tap callbacks,
 * where the finger has already lifted.
 *
 * @param  x  [0..65535] X position in the configured resolution
 * @param  y  [0..65535] Y position in the configured resolution
 * @return Channel number, or -1 without a surface.
 */
int8_t tile_sense_cap_zone_at(tile_t *tile, uint16_t x, uint16_t y);

/**
 * @brief  First finger's X as a percentage of the X range.
 * @studio expose category=tile name=get_x_pct returns=int section=runtime
 * @return 0-100 (truncated), or -1 with no finger.
 */
int16_t tile_sense_cap_get_x_pct(tile_t *tile);

/**
 * @brief  First finger's Y as a percentage of the Y range.
 * @studio expose category=tile name=get_y_pct returns=int section=runtime
 * @return 0-100 (truncated), or -1 with no finger or a one-row surface.
 */
int16_t tile_sense_cap_get_y_pct(tile_t *tile);

/**
 * @brief  Block until a finger touches the surface.
 * @studio expose category=tile name=wait_for_touch returns=bool section=runtime
 *
 * Runs process() every 5 ms. Real time comes from cfg.millis when given
 * (otherwise the driver's own delay count).
 *
 * @param  timeout_ms  [0..600000] ms Maximum wait
 * @return 1 on touch, 0 on timeout.
 */
uint8_t tile_sense_cap_wait_for_touch(tile_t *tile, uint32_t timeout_ms);

/**
 * @brief  Cached Info Flags word (Table A.2).
 * @studio expose category=tile name=get_info_flags returns=int section=runtime
 * @return Info Flags (IQS7211A_INFO_* masks)
 */
uint16_t tile_sense_cap_get_info_flags(tile_t *tile);

/**
 * @brief  Cached chip gesture bits (Table A.3).
 * @studio expose category=tile name=get_gestures returns=int section=runtime
 *
 * Only non-zero with the chip gesture engine selected; each bit lasts one
 * report cycle.
 *
 * @return SENSE_CAP_GESTURE_* bits from the last read
 */
uint16_t tile_sense_cap_get_gestures(tile_t *tile);

/**
 * @brief  Number of fingers on the trackpad (0, 1 or 2).
 * @studio expose category=tile name=get_num_fingers returns=int section=runtime
 * @return Finger count from the cached Info Flags
 */
uint8_t tile_sense_cap_get_num_fingers(tile_t *tile);

/**
 * @brief  Absolute X of a finger (§7.2.3).
 * @studio expose category=tile name=get_finger_x returns=int section=runtime
 *
 * Meaningful only while that slot holds a finger (gate on
 * get_num_fingers); slots are stable while a finger stays down (§7.2.6).
 * An empty slot reads 0xFFFF (observed on hardware); before the first
 * successful process() read, and right after a chip reset, the cache
 * reads 0.
 *
 * @param  finger  [0..1] Finger slot
 * @return X in the configured resolution, 0xFFFF for an empty slot
 */
uint16_t tile_sense_cap_get_finger_x(tile_t *tile, uint8_t finger);

/**
 * @brief  Absolute Y of a finger (§7.2.3).
 * @studio expose category=tile name=get_finger_y returns=int section=runtime
 *
 * An empty slot reads 0xFFFF, as for get_finger_x().
 *
 * @param  finger  [0..1] Finger slot
 * @return Y in the configured resolution, 0xFFFF for an empty slot
 */
uint16_t tile_sense_cap_get_finger_y(tile_t *tile, uint8_t finger);

/**
 * @brief  Touch strength of a finger: the sum of its deltas (§7.2.4).
 * @studio expose category=tile name=get_finger_strength returns=int section=runtime
 * @param  finger  [0..1] Finger slot
 * @return Strength in counts (relative, scales with tuning)
 */
uint16_t tile_sense_cap_get_finger_strength(tile_t *tile, uint8_t finger);

/**
 * @brief  Contact area of a finger, in channels (§7.2.5).
 * @studio expose category=tile name=get_finger_area returns=int section=runtime
 * @param  finger  [0..1] Finger slot
 * @return Number of channels associated with the finger
 */
uint16_t tile_sense_cap_get_finger_area(tile_t *tile, uint8_t finger);

/**
 * @brief  Relative X movement since the previous report (§7.2.2).
 * @studio expose category=tile name=get_relative_x returns=int section=runtime
 * @return Signed X delta in the configured resolution
 */
int16_t tile_sense_cap_get_relative_x(tile_t *tile);

/**
 * @brief  Relative Y movement since the previous report (§7.2.2).
 * @studio expose category=tile name=get_relative_y returns=int section=runtime
 * @return Signed Y delta in the configured resolution
 */
int16_t tile_sense_cap_get_relative_y(tile_t *tile);

/**
 * @brief  Per-channel touch status CH0..CH31 (Table A.4), live read.
 * @studio expose category=tile name=get_touch_status returns=int section=runtime
 *
 * Waits for one communication window (at most about one report period).
 * The outputs of the BUTTONS_2X3 layout.
 *
 * @return Bit n set = channel n touched; 0 on failure
 */
uint32_t tile_sense_cap_get_touch_status(tile_t *tile);

/**
 * @brief  Does one channel report touch? (live read)
 * @studio expose category=tile name=is_touched returns=bool section=runtime
 * @param  channel  [0..5] Channel number
 * @return 1 if touched, 0 if not
 */
uint8_t tile_sense_cap_is_touched(tile_t *tile, uint8_t channel);

/**
 * @brief  Number of channels in the applied surface.
 * @studio expose category=tile name=get_num_channels returns=int section=runtime
 * @return n_rx * n_tx, or 0 without a surface.
 */
uint8_t tile_sense_cap_get_num_channels(tile_t *tile);

/**
 * @brief  One channel's raw count (extended map 0xE000, live read).
 * @studio expose category=tile name=get_channel_count returns=int section=runtime
 * @param  channel  [0..5] Channel number
 * @return Count value, or 0 on failure / bad channel.
 */
uint16_t tile_sense_cap_get_channel_count(tile_t *tile, uint8_t channel);

/**
 * @brief  One channel's signed delta, count minus reference (§5.3.4).
 * @studio expose category=tile name=get_channel_delta returns=int section=runtime
 * @param  channel  [0..5] Channel number
 * @return Delta value, or 0 on failure / bad channel.
 */
int16_t tile_sense_cap_get_channel_delta(tile_t *tile, uint8_t channel);

/**
 * @brief  Read the first n channel counts in one transaction.
 * @studio expose category=tile name=read_channel_counts returns=int section=runtime
 * @studio out_buffer counts type=uint16_t cap_param=n
 *
 * The extended map auto-increments (§11.4.2), so one window serves the
 * whole surface.
 *
 * @param  counts  Out: n counts
 * @param  n       Channels to read, 1-32
 * @return Channels read (n), or 0 on failure (counts untouched).
 */
uint8_t tile_sense_cap_read_channel_counts(tile_t *tile, uint16_t *counts, uint8_t n);

/**
 * @brief  Read the first n signed channel deltas in one transaction.
 * @studio expose category=tile name=read_channel_deltas returns=int section=runtime
 * @studio out_buffer deltas type=int16_t cap_param=n
 * @param  deltas  Out: n deltas (count - reference, §5.3.4)
 * @param  n       Channels to read, 1-32
 * @return Channels read (n), or 0 on failure (deltas untouched).
 */
uint8_t tile_sense_cap_read_channel_deltas(tile_t *tile, int16_t *deltas, uint8_t n);

/**
 * @brief  Counts and deltas together (two windows). C only.
 * @param  tile    Tile handle
 * @param  counts  Out: n counts, or NULL to skip
 * @param  deltas  Out: n signed deltas, or NULL to skip
 * @param  n       Channels to read, 1-32
 * @return 1 if every requested block came back valid, 0 otherwise.
 */
uint8_t tile_sense_cap_read_channels(tile_t *tile, uint16_t *counts,
                                     int16_t *deltas, uint8_t n);

/**
 * @brief  Does the ALP channel report presence? (cached Info Flags)
 * @studio expose category=tile name=is_alp_active returns=bool section=runtime
 * @return 1 if the ALP output bit is set
 */
uint8_t tile_sense_cap_is_alp_active(tile_t *tile);

/**
 * @brief  Raw ALP count (register 0x23, live read).
 * @studio expose category=tile name=get_alp_count returns=int section=runtime
 * @return ALP count, or 0 on failure
 */
uint16_t tile_sense_cap_get_alp_count(tile_t *tile);

/**
 * @brief  ALP long-term average (register 0x24, live read).
 * @studio expose category=tile name=get_alp_lta returns=int section=runtime
 * @return ALP LTA, or 0 on failure
 */
uint16_t tile_sense_cap_get_alp_lta(tile_t *tile);

/**
 * @brief  Current charging mode (cached Info Flags).
 * @studio expose category=tile name=get_mode returns=int section=runtime
 * @return sense_cap_mode_t value
 */
uint8_t tile_sense_cap_get_mode(tile_t *tile);

/**
 * @brief  Did the last ATI fail?
 * @studio expose category=tile name=has_ati_error returns=bool section=runtime
 *
 * The trackpad ATI error flag, plus the ALP one only while ALP wake is in
 * use: the ALP ATI runs only in LP1/LP2 (§5.6.3), so its flag is stale
 * otherwise.
 *
 * @return 1 if a relevant ATI error flag is set
 */
uint8_t tile_sense_cap_has_ati_error(tile_t *tile);

/* ================================================================
 * Configuration
 * ================================================================ */

/**
 * @brief  Touch sensitivity relative to the surface's tuned multipliers.
 * @studio expose category=tile name=set_sensitivity returns=bool section=config
 * @studio control level label="Touch sensitivity" tier=basic default=SENSE_CAP_SENS_NORMAL
 *
 * NORMAL writes the surface's own set/clear multipliers; the other levels
 * scale both by x2, x1.5, x0.75 or x0.5, keeping their ratio. Replaces a
 * set_touch_multipliers() value, and is kept across set_layout().
 *
 * @param  level  Sensitivity level
 * @return 1 if written and verified (or queued behind a running job)
 */
uint8_t tile_sense_cap_set_sensitivity(tile_t *tile, sense_cap_sensitivity_t level);

/**
 * @brief  Set the touch set/clear multipliers directly (§5.5.1).
 * @studio expose category=tile name=set_touch_multipliers returns=bool section=config
 * @studio control set_mult label="Touch-set multiplier" tier=advanced default=8
 * @studio control clear_mult label="Touch-clear multiplier" tier=advanced default=5
 *
 * A channel reports touch when its count rises above Reference x
 * (1 + set/128) and releases below Reference x (1 + clear/128). Smaller =
 * more sensitive; clear below set gives hysteresis. Replaces the
 * sensitivity level until the next set_sensitivity() or set_layout().
 *
 * @param  set_mult    [1..255] Touch-set multiplier
 * @param  clear_mult  [0..255] Touch-clear multiplier (clamped to <= set)
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_touch_multipliers(tile_t *tile, uint8_t set_mult,
                                             uint8_t clear_mult);

/**
 * @brief  Choose a report-rate / timeout bundle for the power modes.
 * @studio expose category=tile name=set_power_profile returns=bool section=config
 * @studio control profile label="Power profile" tier=basic default=SENSE_CAP_POWER_RESPONSIVE
 *
 * Report periods Active / Idle-touch / Idle / LP1 / LP2 (ms), and
 * timeouts Active / Idle-touch / Idle / LP1 (s):
 *   - ALWAYS_ACTIVE: 10/10/10/10/10, timeouts never.
 *   - RESPONSIVE:    10/20/20/50/100, 10/30/20/60.
 *   - BALANCED:      10/50/50/100/200 (the Table 3.4 setup), 5/30/10/30.
 *   - LOW_POWER:     20/100/100/160/320, 2/20/3/10.
 * The chip leaves Idle for LP1/LP2 only while ALP wake is on; otherwise
 * the Idle timeout is written as 0 (never, §6.2), because only the ALP can
 * wake it from there.
 *
 * @param  profile  Profile
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_power_profile(tile_t *tile, sense_cap_power_profile_t profile);

/**
 * @brief  Automatic power modes, or hold one mode (manual control, §6.3).
 * @studio expose category=tile name=set_power_mode returns=bool section=config
 *
 * FORCE_LP1/FORCE_LP2 sense only the ALP and never wake on their own:
 * poll is_alp_active() and switch back. Under manual control the chip
 * does not update trackpad references (§5.4.1): call reseed() when needed.
 *
 * @param  mode  AUTO (init) or a forced mode
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_power_mode(tile_t *tile, sense_cap_power_mode_t mode);

/**
 * @brief  Active-mode report period.
 * @studio expose category=tile name=set_active_rate returns=bool section=config
 * @studio control ms label="Active report period" tier=advanced default=10
 *
 * Overrides one field of the power profile. The coordinate report rate
 * tops out at 100 Hz (§1.1).
 *
 * @param  ms  [10..1000] ms Report period in Active mode
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_active_rate(tile_t *tile, uint16_t ms);

/**
 * @brief  Time in Idle before dropping to LP1 (used while ALP wake is on).
 * @studio expose category=tile name=set_idle_timeout returns=bool section=config
 * @studio control seconds label="Idle time before low power" tier=advanced default=20
 *
 * Overrides one field of the power profile. 0 = never leave Idle.
 *
 * @param  seconds  [0..3600] s Idle timeout
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_idle_timeout(tile_t *tile, uint16_t seconds);

/**
 * @brief  Report period of one charging mode (§6.1).
 * @studio expose category=tile name=set_report_rate returns=bool section=config
 * @param  mode  Which mode's period
 * @param  ms    [1..65535] ms Report period
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_report_rate(tile_t *tile, sense_cap_mode_t mode, uint16_t ms);

/**
 * @brief  Timeout before the chip steps down from a mode (§6.2).
 * @studio expose category=tile name=set_mode_timeout returns=bool section=config
 *
 * LP2 has no timeout. The Idle timeout is only written while ALP wake is
 * on (see set_power_profile).
 *
 * @param  mode     Active, Idle-touch, Idle or LP1
 * @param  seconds  [0..65535] s Timeout, 0 = never
 * @return 1 if written and verified, 0 for LP2 or on failure
 */
uint8_t tile_sense_cap_set_mode_timeout(tile_t *tile, sense_cap_mode_t mode, uint16_t seconds);

/**
 * @brief  Allow the chip to sleep in LP1/LP2 and wake on the ALP channel.
 * @studio expose category=tile name=set_alp_wake returns=bool section=config
 * @studio control enable label="Wake on proximity" tier=advanced type=bool default=0
 *
 * On: the Idle timeout of the power profile applies, ALP automatic re-ATI
 * is enabled and the ALP counts in has_ati_error(); run ati_start(ALP)
 * once so the ALP is tuned before relying on it. Off (init): the chip
 * never times out of Idle, so it never goes deaf in LP with an untuned
 * ALP. Needs a surface whose alp.enable is set.
 *
 * @param  enable  1 = allow ALP wake
 * @return 1 if applied, 0 if the surface has no ALP or on failure
 */
uint8_t tile_sense_cap_set_alp_wake(tile_t *tile, uint8_t enable);

/**
 * @brief  ALP output threshold (§5.5.2).
 * @studio expose category=tile name=set_alp_threshold returns=bool section=config
 * @studio control threshold label="Proximity wake threshold" tier=advanced default=20
 *
 * The ALP output sets when the ALP count deviates from its LTA by more
 * than this. Lower = wakes on lighter proximity.
 *
 * @param  threshold  [1..1000] Delta counts
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_alp_threshold(tile_t *tile, uint16_t threshold);

/**
 * @brief  Mirror X relative to the layout (§7.7).
 * @studio expose category=tile name=set_flip_x returns=bool section=config
 * @studio control enable label="Mirror X" tier=advanced type=bool default=0
 * @param  enable  1 = mirror
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_flip_x(tile_t *tile, uint8_t enable);

/**
 * @brief  Mirror Y relative to the layout (§7.7).
 * @studio expose category=tile name=set_flip_y returns=bool section=config
 * @studio control enable label="Mirror Y" tier=advanced type=bool default=0
 * @param  enable  1 = mirror
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_flip_y(tile_t *tile, uint8_t enable);

/**
 * @brief  Swap the X and Y axes relative to the layout (§7.7).
 * @studio expose category=tile name=set_switch_xy returns=bool section=config
 * @studio control enable label="Swap X and Y" tier=advanced type=bool default=0
 *
 * Channel numbering is unaffected (§7.7). A resolution set with
 * set_resolution() stays with its axis name.
 *
 * @param  enable  1 = swap
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_switch_xy(tile_t *tile, uint8_t enable);

/**
 * @brief  XY output range (§7.4).
 * @studio expose category=tile name=set_resolution returns=bool section=config
 * @studio control x_res label="X resolution" tier=advanced default=0
 * @studio control y_res label="Y resolution" tier=advanced default=0
 *
 * 0 = the layout's own (256 per electrode pitch). Gesture distances scale
 * with the resolution.
 *
 * @param  x_res  [0..65535] X range, 0 = automatic
 * @param  y_res  [0..65535] Y range, 0 = automatic
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_resolution(tile_t *tile, uint16_t x_res, uint16_t y_res);

/**
 * @brief  Maximum tap duration (§8.1).
 * @studio expose category=tile name=set_tap_time returns=bool section=config
 * @studio control ms label="Tap time" tier=advanced default=300
 *
 * Used by both gesture engines: a touch released sooner, without moving
 * beyond the tap distance, is a tap.
 *
 * @param  ms  [50..2000] ms Maximum tap duration
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_tap_time(tile_t *tile, uint16_t ms);

/**
 * @brief  Press-and-hold time, counted after the tap time (§8.2).
 * @studio expose category=tile name=set_hold_time returns=bool section=config
 * @studio control ms label="Hold time" tier=advanced default=300
 *
 * A long press fires at tap time + hold time from touch-down (600 ms with
 * the defaults), in both gesture engines.
 *
 * @param  ms  [0..5000] ms Hold time after the tap time
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_hold_time(tile_t *tile, uint16_t ms);

/**
 * @brief  Maximum swipe duration for the chip gesture engine (§8.3).
 * @studio expose category=tile name=set_swipe_time returns=bool section=config
 * @param  ms  [50..2000] ms Swipe must cover its distance within this
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_swipe_time(tile_t *tile, uint16_t ms);

/**
 * @brief  Minimum swipe travel (§8.3), both axes.
 * @studio expose category=tile name=set_swipe_distance returns=bool section=config
 * @studio control px label="Swipe distance" tier=advanced default=0
 *
 * 0 = automatic: 48 px per 256 px of electrode pitch. The driver
 * recognizer also needs a release faster than 400 px/s per 256 px pitch.
 *
 * @param  px  [0..65535] Pixels in the configured resolution, 0 = automatic
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_swipe_distance(tile_t *tile, uint16_t px);

/**
 * @brief  Maximum swipe angle from its axis (§8.3).
 * @studio expose category=tile name=set_swipe_angle returns=bool section=config
 * @studio control degrees label="Swipe angle" tier=advanced default=45
 *
 * Written as 64 * tan(angle) (register 0x87). 45 accepts every swipe
 * along its dominant axis.
 *
 * @param  degrees  [1..75] deg Largest angle from the X or Y axis
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_swipe_angle(tile_t *tile, uint8_t degrees);

/**
 * @brief  Choose the gesture engine.
 * @studio expose category=tile name=set_gesture_source returns=bool section=config
 * @studio control source label="Gesture engine" tier=advanced default=SENSE_CAP_GESTURES_DRIVER
 *
 * DRIVER (init): the driver's own tap / double tap / hold / swipe
 * recognizers, and the chip's gesture engine is switched off so nothing
 * fires twice. CHIP: the chip's engine (§8) with its gesture event
 * enabled, so a gesture holds its window open until the host reads it.
 * Double tap is paired by the driver either way; drag and pinch are
 * always the driver's.
 *
 * @param  source  Gesture engine
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_gesture_source(tile_t *tile, sense_cap_gesture_source_t source);

/**
 * @brief  Which gestures fire (either engine).
 * @studio expose category=tile name=enable_gestures returns=bool section=config
 * @param  mask  [0..63] OR of SENSE_CAP_GESTURE_* bits (init: all)
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_enable_gestures(tile_t *tile, uint16_t mask);

/**
 * @brief  Maximum simultaneous fingers (§7.3).
 * @studio expose category=tile name=set_max_touches returns=bool section=config
 * @param  fingers  [1..2] Fingers tracked
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_max_touches(tile_t *tile, uint8_t fingers);

/**
 * @brief  Reference update interval (§5.4.1).
 * @studio expose category=tile name=set_reference_update returns=bool section=config
 *
 * Trackpad references refresh from LP1/LP2 under automatic mode control.
 * The datasheet gives a 60 s maximum and does not say what 0 means.
 *
 * @param  seconds  [0..60] s Update interval
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_reference_update(tile_t *tile, uint16_t seconds);

/**
 * @brief  Unserviced-window timeout (§11.6).
 * @studio expose category=tile name=set_i2c_timeout returns=bool section=config
 *
 * How long a requested window waits for the host before the chip moves on
 * (the chip pauses while it waits). Init: 50 ms. Poll process() faster
 * than report period + this, or reads alternate with BUSY.
 *
 * @param  ms  [2..1000] ms Window timeout
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_i2c_timeout(tile_t *tile, uint16_t ms);

/**
 * @brief  Enable or disable the chip's watchdog (§10.2). Init: enabled.
 * @studio expose category=tile name=set_watchdog returns=bool section=config
 * @param  enable  1 = enable
 * @return 1 if written and verified
 */
uint8_t tile_sense_cap_set_watchdog(tile_t *tile, uint8_t enable);

/**
 * @brief  Reseed the trackpad references and the ALP LTA (§5.4.3).
 * @studio expose category=tile name=reseed returns=bool section=config
 * @return 1 if the command was sent
 */
uint8_t tile_sense_cap_reseed(tile_t *tile);

/* ================================================================
 * Advanced / escape hatch
 * ================================================================ */

/**
 * @brief  Read a raw 16-bit register (waits for a window).
 * @studio expose category=tile name=read_reg returns=int section=advanced
 * @param  reg  [0..255] Register address
 * @return Register value, or 0xEEEE if no window opened
 */
uint16_t tile_sense_cap_read_reg(tile_t *tile, uint8_t reg);

/**
 * @brief  Write a raw 16-bit register, guarded and verified.
 * @studio expose category=tile name=write_reg returns=int section=advanced
 *
 * Refuses the Tx short test (0x50 bit 15), RX0/TX0 or TX10 in the ALP
 * masks (0x72/0x73) or the Rx/Tx map (0x90-0x95), and the value 0xEEEE.
 * A write to 0x51 updates the driver's comms settings. Not re-applied
 * after a chip reset, and a later driver call may overwrite it. 0x50 is
 * sent as a command without read-back.
 *
 * @param  reg    [0..255] Register address
 * @param  value  [0..65535] Value
 * @return SENSE_CAP_OK, SENSE_CAP_ERR_FORBIDDEN, or a comms/verify error
 */
sense_cap_result_t tile_sense_cap_write_reg(tile_t *tile, uint8_t reg, uint16_t value);

#endif /* INC_TILE_SENSE_CAP_H_ */
