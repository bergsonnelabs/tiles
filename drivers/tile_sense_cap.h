/**
 * @file   tile_sense_cap.h
 * @brief  Capacitive trackpad driver for the Sense.CAP tile (IQS7211A).
 * @version 0.4.0
 *
 * Azoteq IQS7211A mutual-capacitance trackpad controller: up to 2-finger
 * absolute XY tracking, relative XY, per-channel touch status, built-in
 * gesture engine (tap, press-and-hold, four-way swipe), and an Alternate
 * Low-Power (ALP) channel for presence wake-up.
 *
 * **Surface configuration.** The trackpad's electrical geometry (Rx/Tx
 * count and mapping, conversion cycles, XY resolution, ATI target) is a
 * property of the physical electrode surface. init() deliberately does
 * NOT overwrite the chip's memory map — call
 * tile_sense_cap_configure_surface() after init with a
 * sense_cap_surface_t describing the attached surface, which writes the
 * geometry, packs the sensing cycles, sets the ATI target and runs
 * re-ATI. The r0 2x3 surface (2 Rx strips x 3 Tx blocks) ships as the
 * built-in preset sense_cap_surface_2x3; setup_2x3() is the one-call
 * form. Without a surface config the device runs on factory defaults
 * and XY output is not meaningful.
 *
 * Communication windows: the device only serves register data inside a
 * communication window. In its default streaming mode a window opens every
 * report cycle and the part clock-stretches a master that arrives early
 * (datasheet §11.9.2), so plain reads simply work. In event mode, in the
 * low-power modes, or with Comms Request enabled, they do not — the read
 * returns @ref IQS7211A_INVALID_RESPONSE (0xEEEE) instead. The driver
 * therefore retries every access behind an explicit comms request (write
 * 0x00 to 0xFF, wait, retry), the same shape as the IQS323 force-comms
 * dance on Sense.T.C. One window serves one transaction — an I2C STOP
 * closes it again.
 *
 * The 0xEEEE response is the sharp edge on this part: it is valid-looking
 * data with Comms Request, Event Mode, Manual Control and WDT bits all
 * set. A read-modify-write that trusts it will write those bits back and
 * strand the device in request-only mode — recoverable only by power
 * cycling the tile. Every helper here checks for it before writing.
 *
 * Two communication modes:
 *   - **Polled mode** (default): call process() from your main loop.
 *     No RDY pin wiring required. **This is the required mode on r0
 *     tiles fitted with the IQS7211A** — the board was designed for the
 *     IQS7211E, whose A5 ball combines RDY+MCLR; on the A variant that
 *     ball is MCLR only, so tile pad 3 is a hardware reset line (100k
 *     pull-up on board), and the A's real RDY (ball C3) is unrouted.
 *   - **RDY mode**: provide a rdy_pin in config. Usable once IQS7211E
 *     parts are fitted (pad 3 then carries the combined RDY/MCLR line);
 *     wire it to a Core pad and pass that pad number. An EXTI
 *     falling-edge ISR sets a flag; process() only does I2C when set.
 *
 * Polling example (Cores SDK):
 * @code
 *   tile_t pad;
 *   tile_sense_cap_init(core_tiles_pal(&core_i2c1), 0, &pad, NULL);
 *   while (1) {
 *       tile_sense_cap_process(&pad);
 *       if (tile_sense_cap_get_num_fingers(&pad) > 0)
 *           printf("x=%u y=%u\n", tile_sense_cap_get_finger_x(&pad, 0),
 *                                 tile_sense_cap_get_finger_y(&pad, 0));
 *       core_delay_ms(20);
 *   }
 * @endcode
 *
 * Gesture callback example:
 * @code
 *   void on_pad(tile_t *t, uint16_t info, void *ctx) {
 *       uint16_t g = tile_sense_cap_get_gestures(t);
 *       if (g & SENSE_CAP_GESTURE_SINGLE_TAP) tap();
 *       if (g & SENSE_CAP_GESTURE_SWIPE_X_POS) next();
 *   }
 *
 *   sense_cap_cfg_t cfg = {
 *       .gestures = SENSE_CAP_GESTURE_SINGLE_TAP | SENSE_CAP_GESTURE_SWIPE_X_POS,
 *       .on_event = on_pad,
 *   };
 *   tile_sense_cap_init(core_tiles_pal(&core_i2c1), 0, &pad, &cfg);
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
 * @studio event name=swipe mask=SENSE_CAP_EV_SWIPE_ANY
 * @studio event name=drag mask=SENSE_CAP_EV_DRAG
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=advanced category="ATI fine tuning" section=config
 *   The ATI *target* is part of the surface config, and re_ati() /
 *   reseed() re-run auto-tuning at any time. The finer knobs — trackpad
 *   and ALP ATI multipliers/dividers, compensation dividers and drift
 *   limits (0x30-0x31, 0x33-0x3B) — are left at chip defaults, to be
 *   revisited if re-ATI against the real surface proves insufficient.
 *   Reachable through write_reg meanwhile.
 *
 * @studio unsupported severity=niche category="ALP fine tuning" section=config
 *   The ALP wake channel is configured by the surface config (electrode
 *   selection, mutual sensing, count filter, ATI target) and its
 *   threshold is settable at runtime. Still at chip defaults: the set/
 *   clear debounce counts (0x56), the count-filter betas (0x70/0x71),
 *   and the LPX auto-prox cycle setting. ALP count, LTA and output
 *   status are readable at runtime.
 *
 * @studio unsupported severity=advanced category="XY filtering and trim" section=config
 *   Axis switch and X/Y flips are part of the surface config, and the
 *   recommended MAV + dynamic-IIR filters are enabled there. The filter
 *   *parameters* (0x64-0x66), finger-split factor and stationary-touch
 *   threshold (0x67), and X/Y trim (0x68, 0x69) are not exposed —
 *   surface-tuning knobs, left at chip defaults until real-finger
 *   testing says otherwise.
 *
 * @studio unsupported severity=niche category="Analog hardware settings" section=advanced
 *   Main oscillator selection and adjustment, calibration-capacitor
 *   selection (0x52), and trackpad/ALP charge-transfer frequency
 *   (0x58/0x59) are not exposed. These are GUI-derived analog settings
 *   for a specific electrode stack-up.
 *
 * @studio unsupported severity=niche category="Comms-request mode selection" section=advanced
 *   Config bit "Comms Request EN" (0x51 bit 4) chooses whether the device
 *   clock-stretches a early master or requires an explicit window
 *   request. The driver speaks both — it retries through a comms request
 *   whenever a read comes back invalid — but does not expose a setter for
 *   the bit itself, since the automatic fallback covers both settings.
 *
 * @studio unsupported severity=niche category="MCLR hardware reset" section=lifecycle
 *   Driver-deferred: on r0 tiles with the IQS7211A fitted, tile pad 3
 *   carries the chip's MCLR (active-low reset, 100k pull-up on board) —
 *   a Core pad wired there could hard-reset a comms-locked device, the
 *   one failure the software reset cannot reach. Not yet exposed as an
 *   API; software reset covers normal operation. (There is no
 *   definitions/Sense-CAP-*.json yet; when one lands, this note should
 *   be re-checked against the real pad table.)
 *
 * @studio unsupported severity=niche category="Tx short test" section=advanced
 *   The Tx short-test configuration (0x50 bit 15) is a production-line
 *   electrode-fault test, out of scope for a runtime driver.
 */

#ifndef INC_TILE_SENSE_CAP_H_
#define INC_TILE_SENSE_CAP_H_

#include "tiles.h"
#include <stdint.h>

/* ================================================================
 * Driver version
 * ================================================================ */

#define TILE_SENSE_CAP_VERSION_MAJOR  0
#define TILE_SENSE_CAP_VERSION_MINOR  4
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
 * The address is fixed in silicon (datasheet §11.2) — there are no
 * strap options, so only instance 0 exists on a bus.
 */
#define IQS7211A_I2C_ADDR        0x56

/** Product number reported at register 0x00 (Table A.1). */
#define IQS7211A_PRODUCT_NUMBER  763

/**
 * Returned by every register when the device has no communication window
 * open for the master. It is data, not an error — the part ACKs its
 * address and clocks out 0xEEEE. Treat it as "read failed", and NEVER
 * write a value derived from it: 0xEEEE has Comms Request, Event Mode,
 * Manual Control and WDT all set, so a read-modify-write on an invalid
 * read latches the device into a state it cannot be talked out of
 * without a power cycle.
 */
#define IQS7211A_INVALID_RESPONSE  0xEEEE

/* ================================================================
 * Register map -- 8-bit addresses, 16-bit LE data
 * ================================================================ */

/* Version / ID (read-only) */
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
#define IQS7211A_REG_ALP_COUNT       0x23
#define IQS7211A_REG_ALP_LTA         0x24

/* Report rates (ms) and mode timeouts (s) */
#define IQS7211A_REG_RATE_ACTIVE     0x40
#define IQS7211A_REG_RATE_IDLE_TOUCH 0x41
#define IQS7211A_REG_RATE_IDLE       0x42
#define IQS7211A_REG_RATE_LP1        0x43
#define IQS7211A_REG_RATE_LP2        0x44
#define IQS7211A_REG_TIMEOUT_ACTIVE  0x45
#define IQS7211A_REG_TIMEOUT_IDLE_TOUCH 0x46
#define IQS7211A_REG_TIMEOUT_IDLE    0x47
#define IQS7211A_REG_TIMEOUT_LP1     0x48
#define IQS7211A_REG_REF_UPDATE_TIME 0x49
#define IQS7211A_REG_I2C_TIMEOUT     0x4A

/* Trackpad ATI (0x30-0x35; ALP equivalents 0x36-0x3B untouched) */
#define IQS7211A_REG_TP_ATI_MULTDIV  0x30  /**< ATI multipliers/dividers (Table A.5) */
#define IQS7211A_REG_TP_ATI_COMP_DIV 0x31  /**< ATI compensation divider */
#define IQS7211A_REG_TP_ATI_TARGET   0x32  /**< ATI target (counts) */
#define IQS7211A_REG_TP_REF_DRIFT    0x33  /**< Reference drift limit */
#define IQS7211A_REG_TP_MIN_COUNT    0x34  /**< Minimum count re-ATI value */
#define IQS7211A_REG_REATI_RETRY     0x35  /**< Re-ATI retry time (s) */
#define IQS7211A_REG_ALP_ATI_TARGET  0x38  /**< ALP ATI target (counts) */

/* Control / configuration */
#define IQS7211A_REG_SYSTEM_CONTROL  0x50
#define IQS7211A_REG_CONFIG_SETTINGS 0x51
#define IQS7211A_REG_OTHER_SETTINGS  0x52
#define IQS7211A_REG_TOUCH_MULT      0x53  /**< high: clear mult, low: set mult (§5.5.1) */
#define IQS7211A_REG_ALP_THRESHOLD   0x54  /**< ALP output delta threshold (§5.5.2) */

/* Trackpad sizing */
#define IQS7211A_REG_TP_SETTINGS     0x60  /**< high: total Rxs, low: trackpad settings */
#define IQS7211A_REG_TP_TOUCHES      0x61  /**< high: max multi-touches, low: total Txs */
#define IQS7211A_REG_X_RESOLUTION    0x62
#define IQS7211A_REG_Y_RESOLUTION    0x63

/* Trackpad settings bits (0x60 low byte, Table A.11) */
#define IQS7211A_TP_FLIP_X           (1U << 0)  /**< Invert X output */
#define IQS7211A_TP_FLIP_Y           (1U << 1)  /**< Invert Y output */
#define IQS7211A_TP_SWITCH_XY        (1U << 2)  /**< X along Txs, Y along Rxs */
#define IQS7211A_TP_IIR_FILTER       (1U << 3)  /**< XY IIR filter (recommended on) */
#define IQS7211A_TP_IIR_STATIC       (1U << 4)  /**< Fixed (vs dynamic) IIR damping */
#define IQS7211A_TP_MAV_FILTER       (1U << 5)  /**< XY moving-average filter (recommended on) */

/* Surface geometry (write via configure_surface) */
#define IQS7211A_REG_RXTX_MAP_BASE   0x90  /**< RxTx mapping <1..0> .. <11..10> */
#define IQS7211A_REG_CYCLE_BASE_0    0xA0  /**< Cycles 0-9: 3 bytes each, packed */
#define IQS7211A_REG_CYCLE_BASE_10   0xB0  /**< Cycles 10-17: 3 bytes each, packed */
#define IQS7211A_CYCLE_PROX_BYTE     0x05  /**< Fixed first byte of every cycle record */
#define IQS7211A_CHANNEL_NONE        0xFF  /**< "No channel allocated" in a cycle slot */

/* Extended memory map (16-bit register addresses; the PAL sends two
 * address bytes MSB-first for regs above 0xFF). One 16-bit value per
 * channel, indexed by channel number. */
#define IQS7211A_REG_EXT_COUNTS      0xE000  /**< Per-channel count values */
#define IQS7211A_REG_EXT_REFS        0xE100  /**< Per-channel reference values */
#define IQS7211A_REG_EXT_DELTAS      0xE200  /**< Per-channel delta values */
#define IQS7211A_REG_EXT_ATI_COMP    0xE300  /**< Per-channel ATI compensation */

/* ALP channel setup */
#define IQS7211A_REG_ALP_SETUP       0x72  /**< Table A.12: filter, sensing method, Rx enables */
#define IQS7211A_REG_ALP_TX_ENABLE   0x73  /**< Table A.13: Tx enable bits, one per TXn */
#define IQS7211A_ALP_SETUP_MUTUAL    (1U << 8)  /**< 1 = mutual-capacitive ALP sensing */
#define IQS7211A_ALP_SETUP_FILTER    (1U << 9)  /**< 1 = ALP count filter enabled */

/* Settings version label (0x74): high byte major, low byte minor */
#define IQS7211A_REG_SETTINGS_VER    0x74

/* Gestures */
#define IQS7211A_REG_GESTURE_ENABLE  0x80
#define IQS7211A_REG_TAP_TIME        0x81  /**< ms   */
#define IQS7211A_REG_TAP_DISTANCE    0x82  /**< pixels */
#define IQS7211A_REG_HOLD_TIME       0x83  /**< ms   */
#define IQS7211A_REG_SWIPE_TIME      0x84  /**< ms   */
#define IQS7211A_REG_SWIPE_X_DIST    0x85  /**< pixels */
#define IQS7211A_REG_SWIPE_Y_DIST    0x86  /**< pixels */
#define IQS7211A_REG_SWIPE_ANGLE     0x87  /**< low byte: 64·tan(deg) */

/* Terminate the communication window when the comms-end command mode is
 * enabled: write 0x00 here, then STOP (datasheet §11.7). */
#define IQS7211A_REG_END_COMMS       0xFF

/* ================================================================
 * Info Flags (0x10) bit masks — Table A.2
 * ================================================================ */

#define IQS7211A_INFO_MODE_MASK      (7U << 0)   /**< Charging (power) mode */
#define IQS7211A_INFO_MODE_SHIFT     0
#define IQS7211A_INFO_ATI_ERROR      (1U << 3)   /**< Trackpad ATI failed */
#define IQS7211A_INFO_RE_ATI_OCCURRED (1U << 4)  /**< Trackpad re-ATI completed */
#define IQS7211A_INFO_ALP_ATI_ERROR  (1U << 5)   /**< ALP ATI failed */
#define IQS7211A_INFO_ALP_RE_ATI_OCCURRED (1U << 6) /**< ALP re-ATI completed */
#define IQS7211A_INFO_SHOW_RESET     (1U << 7)   /**< Reset seen, not yet acknowledged */
#define IQS7211A_INFO_NUM_FINGERS_MASK  (3U << 8) /**< 0, 1 or 2 fingers */
#define IQS7211A_INFO_NUM_FINGERS_SHIFT 8
#define IQS7211A_INFO_TP_MOVEMENT    (1U << 10)  /**< Finger movement detected */
#define IQS7211A_INFO_TOO_MANY_FINGERS (1U << 12) /**< More fingers than allowed */
#define IQS7211A_INFO_ALP_OUTPUT     (1U << 14)  /**< ALP prox/touch detected */

/* ================================================================
 * Gestures (0x11 status / 0x80 enable) bit masks — Tables A.3, A.14
 * ================================================================ */

#define SENSE_CAP_GESTURE_SINGLE_TAP    (1U << 0)  /**< Single tap */
#define SENSE_CAP_GESTURE_PRESS_HOLD    (1U << 1)  /**< Press and hold */
#define SENSE_CAP_GESTURE_SWIPE_X_NEG   (1U << 2)  /**< Swipe in -X */
#define SENSE_CAP_GESTURE_SWIPE_X_POS   (1U << 3)  /**< Swipe in +X */
#define SENSE_CAP_GESTURE_SWIPE_Y_POS   (1U << 4)  /**< Swipe in +Y */
#define SENSE_CAP_GESTURE_SWIPE_Y_NEG   (1U << 5)  /**< Swipe in -Y */
#define SENSE_CAP_GESTURE_ALL           0x3FU      /**< Every supported gesture */

/* ---- High-level touch event bits (get_touch_events) ---- */

#define SENSE_CAP_EV_TOUCH_DOWN   (1U << 0)   /**< A finger arrived */
#define SENSE_CAP_EV_TOUCH_UP     (1U << 1)   /**< A finger left */
#define SENSE_CAP_EV_TAP          (1U << 2)   /**< Single tap completed */
#define SENSE_CAP_EV_DOUBLE_TAP   (1U << 3)   /**< Two taps in quick succession */
#define SENSE_CAP_EV_LONG_PRESS   (1U << 4)   /**< Stationary hold past the hold time */
#define SENSE_CAP_EV_SWIPE_LEFT   (1U << 5)   /**< Swipe toward -X */
#define SENSE_CAP_EV_SWIPE_RIGHT  (1U << 6)   /**< Swipe toward +X */
#define SENSE_CAP_EV_SWIPE_UP     (1U << 7)   /**< Swipe toward -Y */
#define SENSE_CAP_EV_SWIPE_DOWN   (1U << 8)   /**< Swipe toward +Y */
#define SENSE_CAP_EV_DRAG         (1U << 9)   /**< Finger moving beyond the tap radius */
#define SENSE_CAP_EV_PINCH        (1U << 10)  /**< Two-finger spread changed */
#define SENSE_CAP_EV_SWIPE_ANY    (SENSE_CAP_EV_SWIPE_LEFT | SENSE_CAP_EV_SWIPE_RIGHT | \
                                   SENSE_CAP_EV_SWIPE_UP | SENSE_CAP_EV_SWIPE_DOWN)

/* ---- Swipe directions (sense_cap_swipe_cb_t) ---- */

#define SENSE_CAP_DIR_LEFT   0  /**< Toward -X */
#define SENSE_CAP_DIR_RIGHT  1  /**< Toward +X */
#define SENSE_CAP_DIR_UP     2  /**< Toward -Y */
#define SENSE_CAP_DIR_DOWN   3  /**< Toward +Y */

/* ================================================================
 * System Control (0x50) bits — Table A.6
 * ================================================================ */

#define IQS7211A_CTRL_MODE_MASK      (7U << 0)   /**< Mode select (manual control only) */
#define IQS7211A_CTRL_MODE_SHIFT     0
#define IQS7211A_CTRL_TP_RESEED      (1U << 3)
#define IQS7211A_CTRL_ALP_RESEED     (1U << 4)
#define IQS7211A_CTRL_TP_RE_ATI      (1U << 5)
#define IQS7211A_CTRL_ALP_RE_ATI     (1U << 6)
#define IQS7211A_CTRL_ACK_RESET      (1U << 7)
#define IQS7211A_CTRL_SW_RESET       (1U << 9)   /**< Resets once the window closes */
#define IQS7211A_CTRL_TX_TEST        (1U << 15)

/* ================================================================
 * Config Settings (0x51) bits — Table A.7
 * ================================================================ */

#define IQS7211A_CFG_TP_RE_ATI_EN    (1U << 2)   /**< Auto re-ATI, trackpad */
#define IQS7211A_CFG_ALP_RE_ATI_EN   (1U << 3)   /**< Auto re-ATI, ALP */
#define IQS7211A_CFG_COMMS_REQUEST_EN (1U << 4)  /**< Request-window instead of stretching */
#define IQS7211A_CFG_WDT_EN          (1U << 5)   /**< Device watchdog timer */
#define IQS7211A_CFG_MANUAL_CONTROL  (1U << 7)   /**< Host drives mode switching */
#define IQS7211A_CFG_EVENT_MODE      (1U << 8)   /**< Comms only on enabled events */
#define IQS7211A_CFG_GESTURE_EVENT   (1U << 9)   /**< Gestures raise an event */
#define IQS7211A_CFG_TP_EVENT        (1U << 10)  /**< XY change / finger up-down raises an event */
#define IQS7211A_CFG_RE_ATI_EVENT    (1U << 11)  /**< Re-ATI raises an event */
#define IQS7211A_CFG_ALP_EVENT       (1U << 13)  /**< ALP state change raises an event */
#define IQS7211A_CFG_TP_TOUCH_EVENT  (1U << 14)  /**< Channel touch-state change raises an event */

/** Every event-source bit in Config Settings — handy as an enable mask. */
#define IQS7211A_CFG_EVENT_MASK \
    (IQS7211A_CFG_GESTURE_EVENT | IQS7211A_CFG_TP_EVENT | \
     IQS7211A_CFG_RE_ATI_EVENT  | IQS7211A_CFG_ALP_EVENT | \
     IQS7211A_CFG_TP_TOUCH_EVENT)

/* ================================================================
 * Enums
 * ================================================================ */

/** Charging (power) mode — reported in Info Flags, selectable in manual control. */
typedef enum {
    SENSE_CAP_MODE_ACTIVE     = 0,  /**< Full report rate */
    SENSE_CAP_MODE_IDLE_TOUCH = 1,  /**< Touch held, reduced rate */
    SENSE_CAP_MODE_IDLE       = 2,  /**< No touch, reduced rate */
    SENSE_CAP_MODE_LP1        = 3,  /**< Low power 1 */
    SENSE_CAP_MODE_LP2        = 4,  /**< Low power 2 (lowest) */
} sense_cap_mode_t;

/** Finger slot for the absolute-XY getters. */
typedef enum {
    SENSE_CAP_FINGER_1 = 0,
    SENSE_CAP_FINGER_2 = 1,
} sense_cap_finger_t;

#define SENSE_CAP_NUM_FINGERS  2

/* ================================================================
 * Event callback
 * ================================================================ */

/**
 * Event callback fired by tile_sense_cap_process() after a successful
 * read of the trackpad data block.
 *
 * @param tile   The tile instance
 * @param info   Info Flags register value (IQS7211A_INFO_* masks)
 * @param ctx    User context (from config or on_event registration)
 *
 * Always runs in main-loop context (never ISR). I2C is safe to call.
 */
typedef void (*sense_cap_event_cb_t)(tile_t *tile, uint16_t info, void *ctx);

/* ================================================================
 * Touch events (mobile-style)
 * ================================================================ */

/** Phase of a touch event, iOS/Android style. */
typedef enum {
    SENSE_CAP_TOUCH_DOWN  = 0,  /**< Finger arrived */
    SENSE_CAP_TOUCH_MOVED = 1,  /**< Position changed while down */
    SENSE_CAP_TOUCH_UP    = 2,  /**< Finger left */
} sense_cap_phase_t;

/**
 * One touch event. Produced by process() from finger-state transitions;
 * consumed via the on_touch() callback or by polling next_touch_event().
 * Velocities are in resolution-pixels per second, computed from the
 * config-supplied millisecond clock (or an internal report-rate estimate
 * when none was provided).
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
/** Tap recognizer callback. taps = 1 (single) or 2 (double). */
typedef void (*sense_cap_tap_cb_t)(tile_t *tile, uint8_t taps,
                                   uint16_t x, uint16_t y, void *ctx);
/** Long-press recognizer callback, fired once per hold. */
typedef void (*sense_cap_hold_cb_t)(tile_t *tile,
                                    uint16_t x, uint16_t y, void *ctx);
/** Swipe recognizer callback. direction = SENSE_CAP_DIR_*. */
typedef void (*sense_cap_swipe_cb_t)(tile_t *tile, uint8_t direction,
                                     int16_t vx, int16_t vy, void *ctx);
/** Drag (pan) recognizer callback, fired on every movement while dragging. */
typedef void (*sense_cap_drag_cb_t)(tile_t *tile, int16_t dx, int16_t dy,
                                    uint16_t x, uint16_t y, void *ctx);
/** Two-finger pinch callback. delta > 0 = spreading, < 0 = pinching in. */
typedef void (*sense_cap_pinch_cb_t)(tile_t *tile, int16_t delta, void *ctx);

/* ================================================================
 * Configuration
 * ================================================================ */

/**
 * Optional init config. Pass NULL for defaults: polled mode, no callback,
 * and the device's own settings left exactly as they are.
 *
 * Every field is opt-in — a zero field means "don't touch that register".
 * This matters while the sensor surface is undefined: the driver must not
 * impose geometry it cannot know.
 */
typedef struct {
    /* RDY pin — set to enable interrupt-driven mode. The device's RDY is
     * open-drain active-low. Uses hal->gpio_irq_enable() to register a
     * falling-edge ISR. 0 = polled mode (the current tile wiring). */
    uint8_t rdy_pin;                /**< Core pad for RDY. 0 = polled mode. */

    sense_cap_event_cb_t on_event;  /**< Callback. NULL = no callback. */
    void *event_ctx;                /**< User context passed to callback. */

    uint16_t gestures;              /**< Gesture enable mask (SENSE_CAP_GESTURE_*). 0 = leave as-is. */
    uint16_t active_rate_ms;        /**< Active-mode report rate. 0 = leave as-is. */
    uint8_t  max_touches;           /**< Max simultaneous fingers, 1 or 2. 0 = leave as-is. */
    uint8_t  event_mode;            /**< 1 = event mode (comms only on events), 0 = leave as-is. */

    /* Millisecond clock for touch-event timestamps and velocities (e.g.
     * wrap core_millis()). NULL = timestamps estimated from the report
     * rate — events still flow, velocities are approximate. */
    uint32_t (*millis)(void *ctx);  /**< Monotonic ms clock, or NULL. */
    void *millis_ctx;               /**< Context for the clock callback. */
} sense_cap_cfg_t;

/* ================================================================
 * Surface description
 * ================================================================ */

/** Maximum Rx electrodes the IQS7211A can sense (RX0-RX7). */
#define SENSE_CAP_MAX_RX  8
/** Maximum Tx electrodes the IQS7211A can drive (TX0-TX11). */
#define SENSE_CAP_MAX_TX  12
/** RxTx-mapping slots in hardware — total_rx + total_tx must fit. */
#define SENSE_CAP_MAX_MAP 12

/**
 * Electrical description of an electrode surface attached to the tile.
 *
 * Pin numbers are the chip's Rx/Tx numbers, in trackpad order: rx_pins[0]
 * is Rx row/column 0, and so on. Channel numbers follow the datasheet
 * rule (§5.1.1): along the Rxs first, then to the next Tx. The sensing
 * cycles are derived automatically — each cycle pairs one channel from
 * prox block A (RX0-3) with one from block B (RX4-7) on the same Tx,
 * so a surface that mixes blocks packs two channels per cycle.
 */
typedef struct {
    uint8_t  total_rx;                    /**< Rx electrode count, 1-8. */
    uint8_t  total_tx;                    /**< Tx electrode count, 1-12. */
    uint8_t  rx_pins[SENSE_CAP_MAX_RX];   /**< Chip RXn numbers, surface order. */
    uint8_t  tx_pins[SENSE_CAP_MAX_TX];   /**< Chip TXn numbers, surface order. */
    uint8_t  switch_xy;                   /**< 1 = X along Txs, Y along Rxs. */
    uint8_t  flip_x;                      /**< 1 = invert X output. */
    uint8_t  flip_y;                      /**< 1 = invert Y output. */
    uint16_t x_res;                       /**< X output resolution (pixels). */
    uint16_t y_res;                       /**< Y output resolution (pixels). */
    uint16_t ati_target;                  /**< Trackpad ATI target (counts). */
    uint8_t  alp_enable;                  /**< 1 = configure the ALP wake channel
                                               over the same electrodes (mutual,
                                               filtered) and ATI it. */
    uint16_t alp_ati_target;              /**< ALP ATI target (counts), applied
                                               per prox block (§5.6.3). */
    uint16_t ati_base;                    /**< ATI multiplier/divider word (0x30,
                                               Table A.5): [13:9] fine divider,
                                               [8:5] coarse multiplier, [4:0]
                                               coarse divider. 0 = chip default.
                                               High-capacitance attachments need
                                               a reduced base (e.g. 0x023F for a
                                               breadboard bench harness). */
    uint8_t  touch_set_mult;              /**< Touch-set multiplier (§5.5.1).
                                               0 = chip default. */
    uint8_t  touch_clear_mult;            /**< Touch-clear multiplier.
                                               0 = chip default. */
} sense_cap_surface_t;

/**
 * The r0 Sense.CAP 2x3 surface: two Rx strips (tile pads 2, 9 = RX3,
 * RX6) across three Tx blocks (tile pads 6, 7, 8 = TX11, TX9, TX8).
 * X runs along the three Tx columns (switch_xy), 256 points between
 * electrodes: X 0-511, Y 0-255.
 *
 * Tuning (hardware-characterised 2026-08-06, socket and breadboard
 * attachments): ATI base 0x023F (coarse divider 31) with target 900 —
 * the working point where a finger produces ~100-190 delta counts.
 * The chip-default base converges at target 300 but yields almost no
 * touch signal on this surface. Touch thresholds 8/5 (~56/35 counts).
 * Re-run ATI (or power cycle) after any mechanical change: cover
 * bonding, mounting surface, enclosure.
 */
extern const sense_cap_surface_t sense_cap_surface_2x3;

/* ================================================================
 * Public API
 * ================================================================ */

/**
 * @brief  Probe the bus for an IQS7211A trackpad controller.
 * @param  hal       Tiles HAL handle (I2C bus)
 * @param  instance  Device instance (only 0 exists — the address is fixed)
 * @return 1 if a device ACKs at 0x56, 0 if not
 */
uint8_t tile_sense_cap_find(tiles_pal_t *hal, uint8_t instance);

/**
 * @brief  Initialize the trackpad controller.
 *
 * Probes the bus, verifies the product number (763), caches the firmware
 * version, and acknowledges the power-on reset flag. Applies only the
 * optional settings supplied in @p cfg — the chip's own configuration is
 * otherwise left untouched. Follow with configure_surface() (or
 * setup_2x3()) to program the electrode geometry.
 *
 * @param  hal       Tiles HAL handle (I2C bus)
 * @param  instance  Device instance (only 0 exists)
 * @param  tile      Tile handle to initialize
 * @param  cfg       Optional config. NULL for defaults.
 */
void tile_sense_cap_init(tiles_pal_t *hal, uint8_t instance,
                         tile_t *tile, const sense_cap_cfg_t *cfg);

/* ---- Surface configuration ---- */

/**
 * @brief  Program the electrode-surface geometry into the device.
 *
 * Writes the Rx/Tx totals and pin mapping, packs and writes the sensing-
 * cycle allocation table (unused cycles cleared), sets XY resolution,
 * axis orientation and the recommended XY filters, sets the ATI target,
 * and finishes with a re-ATI against the new geometry. Every write is
 * window-managed and verified.
 *
 * Call after init, before trusting any XY or touch output. The verified
 * geometry survives in the device until reset, so calling once per boot
 * is sufficient.
 *
 * @param  tile  Initialised tile handle
 * @param  surf  Surface description (see sense_cap_surface_2x3)
 * @return 1 if every register verified and re-ATI completed, 0 otherwise.
 */
uint8_t tile_sense_cap_configure_surface(tile_t *tile,
                                         const sense_cap_surface_t *surf);

/**
 * @brief  Configure the built-in r0 2x3 surface and run ATI.
 * @studio expose category=tile name=setup_2x3 returns=bool section=lifecycle
 *
 * One-call form of configure_surface() with sense_cap_surface_2x3 —
 * the surface shipped with the r0 tile (2 Rx strips x 3 Tx blocks).
 *
 * @param  tile  Initialised tile handle
 * @return 1 if configuration verified and re-ATI completed, 0 otherwise.
 */
uint8_t tile_sense_cap_setup_2x3(tile_t *tile);

/* ---- Touch events (mobile-style) ---- */

/**
 * @brief  Register the raw touch-event stream callback.
 *
 * Fires from process() for every DOWN / MOVED / UP transition, in
 * main-loop context. Pass NULL to disable.
 *
 * @param  tile  Tile handle
 * @param  cb    Callback (NULL to disable)
 * @param  ctx   User context passed to the callback
 */
void tile_sense_cap_on_touch(tile_t *tile, sense_cap_touch_cb_t cb, void *ctx);

/**
 * @brief  Pop the oldest queued touch event.
 *
 * Events queue in an 8-deep ring so a polling loop cannot miss the
 * transitions that happened between process() calls. Oldest events are
 * dropped first on overflow.
 *
 * @param  tile  Tile handle
 * @param  ev    Filled with the event when one was available
 * @return 1 if an event was returned, 0 if the queue is empty.
 */
uint8_t tile_sense_cap_next_touch_event(tile_t *tile, sense_cap_touch_t *ev);

/**
 * @brief  Register the tap recognizer (single and double).
 * @param  tile  Tile handle
 * @param  cb    Callback (NULL to disable)
 * @param  ctx   User context
 */
void tile_sense_cap_on_tap(tile_t *tile, sense_cap_tap_cb_t cb, void *ctx);

/**
 * @brief  Register the long-press recognizer.
 * @param  tile  Tile handle
 * @param  cb    Callback (NULL to disable)
 * @param  ctx   User context
 */
void tile_sense_cap_on_long_press(tile_t *tile, sense_cap_hold_cb_t cb, void *ctx);

/**
 * @brief  Register the swipe recognizer.
 *
 * Uses the chip's four-way swipe engine (enable the swipe gestures in
 * the init config), enriched with the tracked velocity.
 *
 * @param  tile  Tile handle
 * @param  cb    Callback (NULL to disable)
 * @param  ctx   User context
 */
void tile_sense_cap_on_swipe(tile_t *tile, sense_cap_swipe_cb_t cb, void *ctx);

/**
 * @brief  Register the drag (pan) recognizer.
 *
 * Fires on every movement once the finger travels beyond the tap
 * radius, with per-event deltas — the Android onScroll / iOS pan.
 *
 * @param  tile  Tile handle
 * @param  cb    Callback (NULL to disable)
 * @param  ctx   User context
 */
void tile_sense_cap_on_drag(tile_t *tile, sense_cap_drag_cb_t cb, void *ctx);

/**
 * @brief  Register the two-finger pinch recognizer.
 * @param  tile  Tile handle
 * @param  cb    Callback (NULL to disable)
 * @param  ctx   User context
 */
void tile_sense_cap_on_pinch(tile_t *tile, sense_cap_pinch_cb_t cb, void *ctx);

/**
 * @brief  Read and clear the accumulated touch-event bits.
 *
 * Every recognizer result since the previous call, as SENSE_CAP_EV_*
 * bits — the polling counterpart to the callbacks, latched so nothing
 * is missed between reads.
 *
 * @studio expose category=tile name=get_touch_events returns=int section=runtime
 * @param  tile  Tile handle
 * @return Latched SENSE_CAP_EV_* bits; reading clears them.
 */
uint16_t tile_sense_cap_get_touch_events(tile_t *tile);

/**
 * @brief  Did a tap (single or double) complete since the last check?
 *
 * Consumes the tap bits from the same latch as get_touch_events().
 *
 * @studio expose category=tile name=was_tapped returns=bool section=runtime
 * @param  tile  Tile handle
 * @return 1 if a tap completed, 0 otherwise.
 */
uint8_t tile_sense_cap_was_tapped(tile_t *tile);

/* ---- Zones (the 2x3 surface as six buttons) ---- */

/**
 * @brief  Which zone is currently touched?
 *
 * Zones are channel numbers (§5.1.1): for the 2x3 surface, column c /
 * row r is zone c*2 + r — top row 0/2/4 left to right, bottom row
 * 1/3/5. Computed from the tracked finger position.
 *
 * @studio expose category=tile name=get_zone returns=int section=runtime
 * @param  tile  Tile handle
 * @return Zone number, or -1 when nothing is touched.
 */
int8_t tile_sense_cap_get_zone(tile_t *tile);

/**
 * @brief  Which zone contains a given position?
 *
 * Pure geometry over the configured surface — use it inside tap/touch
 * callbacks, where the finger has already lifted and get_zone() would
 * report none ("which button was tapped?").
 *
 * @studio expose category=tile name=zone_at returns=int section=runtime
 * @param  tile  Tile handle
 * @param  x     X position in the configured resolution
 * @param  y     Y position in the configured resolution
 * @return Zone (channel) number, or -1 before configure_surface().
 */
int8_t tile_sense_cap_zone_at(tile_t *tile, uint16_t x, uint16_t y);

/**
 * @brief  Is a specific zone's channel reporting touch?
 * @studio expose category=tile name=is_zone_touched returns=bool section=runtime
 * @param  tile  Tile handle
 * @param  zone  Zone (channel) number, 0-31
 * @return 1 if that channel's touch bit is set (live read).
 */
uint8_t tile_sense_cap_is_zone_touched(tile_t *tile, uint8_t zone);

/**
 * @brief  Finger position as percentages, resolution-independent.
 * @studio expose category=tile name=get_position_pct section=runtime
 * @param  tile   Tile handle
 * @param  x_pct  0-100 across X (or -1 with no finger); NULL ok
 * @param  y_pct  0-100 across Y (or -1 with no finger); NULL ok
 */
void tile_sense_cap_get_position_pct(tile_t *tile,
                                     int32_t *x_pct, int32_t *y_pct);

/**
 * @brief  Block until a finger touches the surface.
 *
 * Runs process() internally while waiting.
 *
 * @studio expose category=tile name=wait_for_touch returns=bool section=runtime
 * @param  tile        Tile handle
 * @param  timeout_ms  Maximum wait
 * @return 1 on touch, 0 on timeout.
 */
uint8_t tile_sense_cap_wait_for_touch(tile_t *tile, uint32_t timeout_ms);

/**
 * @brief  One-knob touch sensitivity.
 *
 * Maps 1 (least sensitive, firm touches only) to 5 (most sensitive,
 * light touches) onto the touch set/clear multiplier pairs.
 *
 * @studio expose category=tile name=set_sensitivity section=config
 * @param  tile   Tile handle
 * @param  level  1-5; values outside the range are clamped.
 */
void tile_sense_cap_set_sensitivity(tile_t *tile, uint8_t level);

/* ---- Event processing ---- */

/**
 * @brief  Read the trackpad data block and update the cached state.
 * @studio expose category=tile name=process section=runtime
 *
 * Call from your main loop. Reads info flags, gestures, relative XY and
 * both finger slots in one burst, then fires the event callback.
 * In RDY mode returns immediately unless the RDY line has fired.
 *
 * @param  tile  Tile handle
 */
void tile_sense_cap_process(tile_t *tile);

/**
 * @brief  Register or change the event callback.
 * @studio expose category=tile name=on_event section=runtime
 * @param  tile  Tile handle
 * @param  cb    Callback function (NULL to disable)
 * @param  ctx   User context passed to the callback
 */
void tile_sense_cap_on_event(tile_t *tile, sense_cap_event_cb_t cb, void *ctx);

/* ---- Identification ---- */

/**
 * @brief  Read the device's firmware major version (cached at init).
 * @studio expose category=tile name=get_version_major returns=int section=lifecycle
 * @param  tile  Tile handle
 * @return Major version from register 0x01
 */
uint16_t tile_sense_cap_get_version_major(tile_t *tile);

/**
 * @brief  Read the device's firmware minor version (cached at init).
 * @studio expose category=tile name=get_version_minor returns=int section=lifecycle
 * @param  tile  Tile handle
 * @return Minor version from register 0x02
 */
uint16_t tile_sense_cap_get_version_minor(tile_t *tile);

/**
 * @brief  Read the settings-version label the device was programmed with.
 *
 * Register 0x74: high byte is the settings major version, low byte the
 * minor. A designer stamps this when exporting a configured firmware from
 * the Azoteq GUI, so the host can confirm the part carries the intended
 * surface configuration (datasheet §10.1.1).
 *
 * @studio expose category=tile name=get_settings_version returns=int section=lifecycle
 * @param  tile  Tile handle
 * @return Raw 16-bit settings version, or 0 if the tile is not ready
 */
uint16_t tile_sense_cap_get_settings_version(tile_t *tile);

/* ---- Lifecycle ---- */

/**
 * @brief  Software-reset the device.
 *
 * The reset takes effect once the communication window closes
 * (datasheet §9.3.2). Blocks for the device's boot time, then clears the
 * reset indication. The tile returns to TILE_STATE_READY.
 *
 * @studio expose category=tile name=reset section=lifecycle
 * @param  tile  Tile handle
 */
void tile_sense_cap_reset(tile_t *tile);

/**
 * @brief  Clear the reset indication (Show Reset) flag.
 *
 * The device sets Show Reset on every boot. Acknowledging it means a
 * later Show Reset tells the host the part rebooted unexpectedly — a
 * brown-out or a device-watchdog reset.
 *
 * @studio expose category=tile name=ack_reset section=lifecycle
 * @param  tile  Tile handle
 */
void tile_sense_cap_ack_reset(tile_t *tile);

/**
 * @brief  Force the device into its lowest-power mode (LP2).
 *
 * Takes manual control of mode switching. The trackpad keeps sensing at
 * the LP2 report rate; call wake() to hand control back to the device's
 * automatic mode machine. Sets TILE_STATE_SLEEPING.
 *
 * @studio expose category=tile name=sleep section=lifecycle
 * @param  tile  Tile handle
 */
void tile_sense_cap_sleep(tile_t *tile);

/**
 * @brief  Return to automatic mode switching, starting in Active mode.
 * @studio expose category=tile name=wake section=lifecycle
 * @param  tile  Tile handle
 */
void tile_sense_cap_wake(tile_t *tile);

/* ---- Runtime data (from the last process() call) ---- */

/**
 * @brief  Read the cached Info Flags word.
 * @studio expose category=tile name=get_info_flags returns=int section=runtime
 * @param  tile  Tile handle
 * @return Info Flags (IQS7211A_INFO_* masks)
 */
uint16_t tile_sense_cap_get_info_flags(tile_t *tile);

/**
 * @brief  Read the cached gesture flags.
 *
 * Gesture bits are latched by the device for one report cycle, so read
 * them every process() call or they are missed.
 *
 * @studio expose category=tile name=get_gestures returns=int section=runtime
 * @param  tile  Tile handle
 * @return Gesture bits (SENSE_CAP_GESTURE_* masks)
 */
uint16_t tile_sense_cap_get_gestures(tile_t *tile);

/**
 * @brief  Number of fingers currently on the trackpad (0, 1 or 2).
 * @studio expose category=tile name=get_num_fingers returns=int section=runtime
 * @param  tile  Tile handle
 * @return Finger count from the cached Info Flags
 */
uint8_t tile_sense_cap_get_num_fingers(tile_t *tile);

/**
 * @brief  Absolute X coordinate of a finger.
 *
 * Only meaningful while that slot holds a tracked finger — gate on
 * @ref tile_sense_cap_get_num_fingers. Fingers keep their slot from one
 * cycle to the next (datasheet §7.2.6), so slot 0 stays slot 0 while it
 * is down.
 *
 * The raw register contents are returned as-is. An empty slot reads
 * 0xFFFF in practice (observed on hardware, 2026-08-04) but the
 * datasheet does not document that sentinel, so gate on the finger
 * count rather than testing for it.
 *
 * @studio expose category=tile name=get_finger_x returns=int section=runtime
 * @param  tile    Tile handle
 * @param  finger  0 or 1 (SENSE_CAP_FINGER_1 / _2)
 * @return X in trackpad units (0..X resolution)
 */
uint16_t tile_sense_cap_get_finger_x(tile_t *tile, uint8_t finger);

/**
 * @brief  Absolute Y coordinate of a finger.
 *
 * Same slot caveat as @ref tile_sense_cap_get_finger_x.
 *
 * @studio expose category=tile name=get_finger_y returns=int section=runtime
 * @param  tile    Tile handle
 * @param  finger  0 or 1 (SENSE_CAP_FINGER_1 / _2)
 * @return Y in trackpad units (0..Y resolution)
 */
uint16_t tile_sense_cap_get_finger_y(tile_t *tile, uint8_t finger);

/**
 * @brief  Touch strength of a finger — the sum of its channels' deltas.
 *
 * Scales with the sensitivity setup, so it is a relative measure, not a
 * force reading. Same slot caveat as @ref tile_sense_cap_get_finger_x.
 *
 * @studio expose category=tile name=get_finger_strength returns=int section=runtime
 * @param  tile    Tile handle
 * @param  finger  0 or 1
 * @return Touch strength in device units
 */
uint16_t tile_sense_cap_get_finger_strength(tile_t *tile, uint8_t finger);

/**
 * @brief  Contact area of a finger, in channels.
 *
 * Same slot caveat as @ref tile_sense_cap_get_finger_x.
 *
 * @studio expose category=tile name=get_finger_area returns=int section=runtime
 * @param  tile    Tile handle
 * @param  finger  0 or 1
 * @return Number of channels associated with the finger
 */
uint16_t tile_sense_cap_get_finger_area(tile_t *tile, uint8_t finger);

/**
 * @brief  Relative X movement since the previous report.
 * @studio expose category=tile name=get_relative_x returns=int section=runtime
 * @param  tile  Tile handle
 * @return Signed X delta in trackpad units
 */
int16_t tile_sense_cap_get_relative_x(tile_t *tile);

/**
 * @brief  Relative Y movement since the previous report.
 * @studio expose category=tile name=get_relative_y returns=int section=runtime
 * @param  tile  Tile handle
 * @return Signed Y delta in trackpad units
 */
int16_t tile_sense_cap_get_relative_y(tile_t *tile);

/**
 * @brief  Read the 32-bit per-channel touch status (CH0..CH31).
 *
 * Live read, not cached — the touch-status words sit outside the data
 * block that process() burst-reads. Useful when unused trackpad channels
 * are repurposed as discrete buttons (datasheet §10.4).
 *
 * @studio expose category=tile name=get_touch_status returns=int section=runtime
 * @param  tile  Tile handle
 * @return Bit n set = channel n touched
 */
uint32_t tile_sense_cap_get_touch_status(tile_t *tile);

/**
 * @brief  Check whether a single trackpad channel reports touch.
 * @studio expose category=tile name=is_touched returns=bool section=runtime
 * @param  tile     Tile handle
 * @param  channel  Channel number, 0..31
 * @return 1 if touched, 0 if not
 */
uint8_t tile_sense_cap_is_touched(tile_t *tile, uint8_t channel);

/**
 * @brief  Number of trackpad channels in the configured surface.
 * @studio expose category=tile name=get_num_channels returns=int section=runtime
 * @param  tile  Tile handle
 * @return total_rx * total_tx, or 0 before configure_surface().
 */
uint8_t tile_sense_cap_get_num_channels(tile_t *tile);

/**
 * @brief  Read one channel's raw count from the extended memory map.
 *
 * Live read. Channel numbers follow the datasheet rule: along the Rxs
 * first, then to the next Tx (for the 2x3 surface: Tx column n has
 * channels 2n and 2n+1, top and bottom row).
 *
 * @studio expose category=tile name=get_channel_count returns=int section=runtime
 * @param  tile     Tile handle
 * @param  channel  Channel number, 0-31
 * @return Count value, or 0 for an out-of-range channel.
 */
uint16_t tile_sense_cap_get_channel_count(tile_t *tile, uint8_t channel);

/**
 * @brief  Read one channel's delta (count minus reference).
 *
 * Live read from the extended memory map. The delta is what the touch
 * threshold acts on — the primary signal for surface bring-up.
 *
 * @studio expose category=tile name=get_channel_delta returns=int section=runtime
 * @param  tile     Tile handle
 * @param  channel  Channel number, 0-31
 * @return Delta value, or 0 for an out-of-range channel.
 */
uint16_t tile_sense_cap_get_channel_delta(tile_t *tile, uint8_t channel);

/**
 * @brief  Check whether the ALP channel detects presence.
 * @studio expose category=tile name=is_alp_active returns=bool section=runtime
 * @param  tile  Tile handle
 * @return 1 if the ALP channel reports prox/touch in the cached info flags
 */
uint8_t tile_sense_cap_is_alp_active(tile_t *tile);

/**
 * @brief  Read every channel's count and delta in two bus transactions.
 *
 * get_channel_count() / get_channel_delta() each open their own
 * communication window, so a per-channel sweep of a 6-channel surface
 * costs twelve window waits (~10 ms each at the fastest report rate).
 * This reads the count block and the delta block as two auto-incrementing
 * bursts: two windows for the whole surface. Use it for any live view.
 *
 * @studio expose category=tile name=read_channels returns=bool section=runtime
 * @param  tile    Tile handle
 * @param  counts  Out: `n` counts, or NULL to skip
 * @param  deltas  Out: `n` deltas (signed, cast to int16_t), or NULL to skip
 * @param  n       Channels to read, 1-32
 * @return 1 if every requested block came back valid, 0 otherwise.
 */
uint8_t tile_sense_cap_read_channels(tile_t *tile, uint16_t *counts,
                                     uint16_t *deltas, uint8_t n);

/**
 * @brief  Read the raw ALP channel count.
 * @studio expose category=tile name=get_alp_count returns=int section=runtime
 * @param  tile  Tile handle
 * @return ALP count value (live read)
 */
uint16_t tile_sense_cap_get_alp_count(tile_t *tile);

/**
 * @brief  Read the ALP channel long-term average.
 * @studio expose category=tile name=get_alp_lta returns=int section=runtime
 * @param  tile  Tile handle
 * @return ALP LTA value (live read)
 */
uint16_t tile_sense_cap_get_alp_lta(tile_t *tile);

/**
 * @brief  Current charging (power) mode of the device.
 * @studio expose category=tile name=get_mode returns=int section=runtime
 * @param  tile  Tile handle
 * @return sense_cap_mode_t value from the cached info flags
 */
uint8_t tile_sense_cap_get_mode(tile_t *tile);

/**
 * @brief  Report whether the most recent ATI run failed.
 *
 * An ATI error means the auto-tuning could not reach its target — with
 * no sensor surface attached, this is the expected state.
 *
 * @studio expose category=tile name=has_ati_error returns=bool section=runtime
 * @param  tile  Tile handle
 * @return 1 if either the trackpad or the ALP ATI error flag is set
 */
uint8_t tile_sense_cap_has_ati_error(tile_t *tile);

/* ---- Configuration ---- */

/**
 * @brief  Set which gestures the device detects.
 * @studio expose category=tile name=enable_gestures section=config
 * @param  tile  Tile handle
 * @param  mask  OR of SENSE_CAP_GESTURE_* bits (0 disables all gestures)
 */
void tile_sense_cap_enable_gestures(tile_t *tile, uint16_t mask);

/**
 * @brief  Set the tap and press-and-hold timing.
 *
 * A tap must lift within @p tap_ms and move less than the tap distance;
 * a press that stays down past @p hold_ms raises press-and-hold
 * (datasheet §8.1).
 *
 * @studio expose category=tile name=set_tap_timing section=config
 * @param  tile     Tile handle
 * @param  tap_ms   Maximum tap duration in ms
 * @param  hold_ms  Press-and-hold duration in ms
 */
void tile_sense_cap_set_tap_timing(tile_t *tile, uint16_t tap_ms, uint16_t hold_ms);

/**
 * @brief  Set the swipe timing and distance thresholds.
 *
 * A swipe must cover @p x_dist or @p y_dist trackpad units within
 * @p swipe_ms (datasheet §8.2, §8.3).
 *
 * @studio expose category=tile name=set_swipe_timing section=config
 * @param  tile      Tile handle
 * @param  swipe_ms  Maximum swipe duration in ms
 * @param  x_dist    X distance threshold in trackpad units
 * @param  y_dist    Y distance threshold in trackpad units
 */
void tile_sense_cap_set_swipe_timing(tile_t *tile, uint16_t swipe_ms,
                                     uint16_t x_dist, uint16_t y_dist);

/**
 * @brief  Set the report rate for one charging mode.
 * @studio expose category=tile name=set_report_rate section=config
 * @param  tile  Tile handle
 * @param  mode  Which mode's rate to set (sense_cap_mode_t)
 * @param  ms    Report period in milliseconds
 */
void tile_sense_cap_set_report_rate(tile_t *tile, uint8_t mode, uint16_t ms);

/**
 * @brief  Bound how long a polled read can block.
 *
 * Without a RDY line the host cannot know when a communication window is
 * open, so every polled read clock-stretches until the part's NEXT report
 * cycle. That is ~10 ms in Active but grows with each power mode, to
 * ~160 ms in LP2 — measured on hardware as a 130 ms stall per process()
 * call once an untouched surface had drifted down to LP2. A host with
 * anything else to do (a radio, audio DMA, other tiles on the bus) cannot
 * afford that.
 *
 * This sets one report period for every mode, LP2 included, so a read
 * blocks for at most `ms` whatever mode the part is in. The cost is
 * power: the low-power modes stop saving anything, because their saving
 * IS the slow cycle. Use it while polling responsively and restore the
 * per-mode rates (set_report_rate) when idle — or fit RDY and use event
 * mode, which needs neither.
 *
 * The host's I2C timeout must exceed `ms`, or the stretch is cut short
 * and the read fails into the driver's window-request retries (8 x 16 ms).
 *
 * @studio expose category=tile name=set_poll_latency returns=bool section=config
 * @param  tile  Tile handle
 * @param  ms    Report period for all modes, in ms (minimum 5)
 * @return 1 if every mode's rate verified, 0 otherwise.
 */
uint8_t tile_sense_cap_set_poll_latency(tile_t *tile, uint16_t ms);

/**
 * @brief  Set the inactivity timeout that drops the device to the next mode.
 * @studio expose category=tile name=set_mode_timeout section=config
 * @param  tile     Tile handle
 * @param  mode     Mode to set the timeout for (Active, Idle-Touch, Idle or LP1)
 * @param  seconds  Timeout in seconds (0 = never time out of that mode)
 * @return 1 if the write verified, 0 otherwise (bad mode, or no window).
 */
uint8_t tile_sense_cap_set_mode_timeout(tile_t *tile, uint8_t mode, uint16_t seconds);

/**
 * @brief  Set the maximum number of simultaneous fingers tracked (1 or 2).
 * @studio expose category=tile name=set_max_touches section=config
 * @param  tile     Tile handle
 * @param  fingers  1 or 2
 */
void tile_sense_cap_set_max_touches(tile_t *tile, uint8_t fingers);

/**
 * @brief  Set the reported XY resolution of the trackpad.
 *
 * Absolute finger coordinates are scaled to 0..x_res and 0..y_res
 * (datasheet §7.4). Meaningful only once a real surface exists.
 *
 * @studio expose category=tile name=set_resolution section=config
 * @param  tile   Tile handle
 * @param  x_res  X resolution in units
 * @param  y_res  Y resolution in units
 */
void tile_sense_cap_set_resolution(tile_t *tile, uint16_t x_res, uint16_t y_res);

/**
 * @brief  Set the touch set/clear threshold multipliers.
 * @studio expose category=tile name=set_touch_multipliers section=config
 *
 * A channel reports touch when its count rises above
 * Reference x (1 + multiplier/128) — §5.5.1. Smaller multiplier =
 * more sensitive. Distinct set and clear values give hysteresis.
 *
 * @param  tile        Tile handle
 * @param  set_mult    Touch-set multiplier, 0-255
 * @param  clear_mult  Touch-clear multiplier, 0-255
 */
void tile_sense_cap_set_touch_multipliers(tile_t *tile, uint8_t set_mult,
                                          uint8_t clear_mult);

/**
 * @brief  Set the ALP wake-channel output threshold.
 * @studio expose category=tile name=set_alp_threshold section=config
 *
 * The ALP output asserts when the ALP count deviates from its LTA by
 * more than this delta (§5.5.2). Lower = wakes on lighter proximity.
 *
 * @param  tile       Tile handle
 * @param  threshold  Count-delta threshold
 */
void tile_sense_cap_set_alp_threshold(tile_t *tile, uint16_t threshold);

/**
 * @brief  Enable or disable event-mode communication.
 *
 * In event mode the device only opens a communication window when an
 * enabled event occurs, instead of every cycle. Pair it with a RDY pin
 * for interrupt-driven operation; in polled mode the driver reads through
 * the device's clock stretching either way.
 *
 * @studio expose category=tile name=set_event_mode section=config
 * @param  tile    Tile handle
 * @param  enable  1 to enable event mode, 0 for streaming
 * @param  events  Event sources to enable (IQS7211A_CFG_*_EVENT bits).
 *                 Pass 0 to leave the current event sources alone.
 */
void tile_sense_cap_set_event_mode(tile_t *tile, uint8_t enable, uint16_t events);

/**
 * @brief  Enable or disable the device's internal watchdog timer.
 *
 * The device watchdog triggers a full cold boot if its main loop stalls
 * (datasheet §10.2). Independent of the Core's own watchdog.
 *
 * @studio expose category=tile name=set_watchdog section=config
 * @param  tile    Tile handle
 * @param  enable  1 to enable, 0 to disable
 */
void tile_sense_cap_set_watchdog(tile_t *tile, uint8_t enable);

/**
 * @brief  Queue a re-ATI (auto-tuning) run and wait for it to finish.
 *
 * Re-runs auto-tuning on both the trackpad and the ALP channel against
 * whatever settings the device currently holds, then reports whether it
 * converged. Run this after the sensing environment changes.
 *
 * @studio expose category=tile name=re_ati returns=bool section=config
 * @param  tile  Tile handle
 * @return 1 if ATI completed without an error flag, 0 on error or timeout
 */
uint8_t tile_sense_cap_re_ati(tile_t *tile);

/**
 * @brief  Reseed the trackpad and ALP reference values.
 *
 * Snaps the references to the present counts, discarding any drift the
 * long-term average has accumulated (datasheet §5.4.3).
 *
 * @studio expose category=tile name=reseed section=config
 * @param  tile  Tile handle
 */
void tile_sense_cap_reseed(tile_t *tile);

/* ---- Advanced / escape hatch ---- */

/**
 * @brief  Read a raw 16-bit register.
 *
 * The escape hatch for everything in the `@studio unsupported` list —
 * notably the geometry and ATI registers that a surface bring-up needs
 * before this driver grows a typed API for them.
 *
 * @studio expose category=tile name=read_reg returns=int section=advanced
 * @param  tile  Tile handle
 * @param  reg   Register address (0x00..0xFF)
 * @return Register value, little-endian, 0 if the tile is not ready
 */
uint16_t tile_sense_cap_read_reg(tile_t *tile, uint8_t reg);

/**
 * @brief  Write a raw 16-bit register.
 * @studio expose category=tile name=write_reg section=advanced
 * @param  tile   Tile handle
 * @param  reg    Register address (0x00..0xFF)
 * @param  value  Value to write, little-endian on the wire
 */
void tile_sense_cap_write_reg(tile_t *tile, uint8_t reg, uint16_t value);

#endif /* INC_TILE_SENSE_CAP_H_ */
