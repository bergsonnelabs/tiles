/**
 * @file   tile_sense_t_c.h
 * @brief  Capacitive touch/proximity driver for the Sense.T.C tile (IQS323).
 * @version 1.4.0
 *
 * Azoteq IQS323 ProxFusion controller with self-capacitive touch sensing.
 * The tile's entire top surface is a touch electrode (CH1/CRx1), and a
 * second channel (CH0/CRx0) is available via pad 8 for external sensors.
 *
 * init() soft-resets the chip and configures it: CH1 on the surface
 * electrode (CRx1/CTx1) and CH0 on pad 8 (CRx0/CTx0), self-capacitance,
 * prox 20 / touch 40, Azoteq's EV-kit filters and report rates (NP 16 ms,
 * LP 60, ULP 160, Halt 3000, auto step-down after 2 s), AUTO power, ATI,
 * then I2C event mode. The chip's own reset values leave every threshold,
 * rate and filter at 0 and point all three channels at CRx0, so none of
 * this is optional. If the chip resets later (brown-out, watchdog),
 * process() sees the reset event and configures it again.
 *
 * Two ways to drive process():
 *   - **Polled mode** (default): call process() from your main loop.
 *     No RDY pin wiring required. Each call forces a communication
 *     window (about 15 ms).
 *   - **RDY mode**: provide a rdy_pin in config. EXTI falling-edge ISR
 *     sets a flag; process() only does I2C when the flag is set, which in
 *     event mode means a touch or prox change. Lower latency, far less
 *     I2C traffic.
 *
 * Event callback: register an on_event callback to be notified of
 * touch, proximity, slider, and gesture changes without polling
 * individual status functions.
 *
 * Simple polling example (Cores SDK):
 * @code
 *   tile_t touch;
 *   tile_sense_t_c_init(core_tiles_pal(&core_i2c3), 0, &touch, NULL);
 *   while (1) {
 *       tile_sense_t_c_process(&touch);
 *       if (tile_sense_t_c_is_touched(&touch, SENSE_T_C_CH_SURFACE))
 *           led_on();
 *       else
 *           led_off();
 *       core_delay_ms(30);
 *   }
 * @endcode
 *
 * Callback example (polled):
 * @code
 *   void on_touch(tile_t *t, uint16_t status, void *ctx) {
 *       if (status & SENSE_T_C_SURFACE_TOUCH)
 *           led_green();
 *       else
 *           led_off();
 *   }
 *
 *   sense_t_c_cfg_t cfg = { .on_event = on_touch };
 *   tile_sense_t_c_init(core_tiles_pal(&core_i2c3), 0, &touch, &cfg);
 *   while (1) {
 *       tile_sense_t_c_process(&touch);
 *       core_delay_ms(30);
 *   }
 * @endcode
 *
 * Callback example (RDY interrupt):
 * @code
 *   sense_t_c_cfg_t cfg = {
 *       .rdy_pin = 3,
 *       .on_event = on_touch,
 *   };
 *   tile_sense_t_c_init(core_tiles_pal(&core_i2c3), 0, &touch, &cfg);
 *   while (1) {
 *       tile_sense_t_c_process(&touch);
 *       // ... other work — process() returns fast if no RDY event ...
 *   }
 * @endcode
 *
 * Slider note: a slider needs at least two channels on the SAME IQS323
 * (the surface CH1 plus an electrode on pad 8 / CH0; CH2 is not routed),
 * configured through the slider registers (0x90-0x98) with write_reg.
 * Two separate tiles cannot form one slider: each has its own controller.
 * A single tile surface alone cannot produce a slider output.
 *
 * Datasheet: Azoteq IQS323, Rev v1.11, August 2025
 *
 * @studio tile label=Sense.T.C icon=◕
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=advanced category="Per-gesture threshold tuning" section=config
 *   IQS323 gesture timing (tap touch/release time, hold duration,
 *   swipe distance, flick velocity) is packed across several
 *   GESTURE_* registers with chip-specific encodings. The driver
 *   reports gesture events but doesn't surface a typed API for
 *   timing / threshold tuning — closing this properly needs a
 *   dedicated pass with Azoteq's gesture-config guide. Raw
 *   write_reg works for advanced users in the interim.
 *
 * @studio unsupported severity=niche category="Per-channel timeout / OutA event-indicator" section=advanced
 *   Channel-timeout disable bits (auto-reseed on timeout) and the
 *   OutA pin's event-indicator output mode aren't exposed. Most
 *   applications don't need them; raw write_reg covers the niche
 *   cases that do. OutA is also not routed on this tile (HW-gated).
 *
 * @studio unsupported severity=niche category="Mutual-capacitance / inductive sensing" section=advanced
 *   The chip can run mutual-capacitance (Tx/Rx pairs) and inductive
 *   channels. HW-gated: the tile routes one self-cap electrode per
 *   channel (surface on CRx1, pad 8 on CRx0) and no TxA; CRx2 is not
 *   routed. The driver configures self-capacitance only.
 *
 * @studio unsupported severity=niche category="Hardware reset (MCLR)" section=advanced
 *   Pad 3 is RDY/MCLR: pulling it low outside a window hard-resets the
 *   chip. Driver-deferred: the driver uses the soft reset and the pad
 *   only as the RDY input. The on-tile 100 nF on the pin slows any
 *   host-driven pulse.
 *
 * @studio unsupported severity=advanced category="Release UI (long-touch release)" section=advanced
 *   The tile's IQS323-001 includes the Release UI: an activation LTA
 *   that lets a long-held touch or prox release cleanly (registers
 *   0x20-0x25, 0xD4, Sensor Setup bit 6). Driver-deferred; write_reg
 *   reaches it. The Movement UI is a different build (IQS323-A01), not
 *   on this tile.
 */

#ifndef INC_TILE_SENSE_T_C_H_
#define INC_TILE_SENSE_T_C_H_

#include "tiles.h"
#include <stdint.h>

/* ================================================================
 * Driver version
 * ================================================================ */

#define TILE_SENSE_T_C_VERSION_MAJOR  1
#define TILE_SENSE_T_C_VERSION_MINOR  4
#define TILE_SENSE_T_C_VERSION_PATCH  0

TILES_CHECK_VERSION(1, 0);

/* ================================================================
 * Instance mapping
 * ================================================================ */

/**
 * | Instance | Address | Order code  |
 * |----------|---------|-------------|
 * | 0        | 0x44    | IQS323-001  |
 * | 1        | 0x58    | IQS323-002  |
 */
#define IQS323_I2C_ADDR_001   0x44
#define IQS323_I2C_ADDR_002   0x58

/* ================================================================
 * Channel identifiers
 * ================================================================ */

#define SENSE_T_C_CH0           0   /**< CRx0/CTx0 -- pad 8 (external) */
#define SENSE_T_C_CH1           1   /**< CRx1/CTx1 -- tile top surface  */
#define SENSE_T_C_CH2           2   /**< CRx2/CTx2 -- not routed on this tile; always disabled */
#define SENSE_T_C_CH_SURFACE    SENSE_T_C_CH1
#define SENSE_T_C_CH_EXTERNAL   SENSE_T_C_CH0
#define SENSE_T_C_NUM_CHANNELS  3
/** Channels with an electrode on this tile: CH0 (pad 8) and CH1 (surface). */
#define SENSE_T_C_CHANNELS_ROUTED ((1U << SENSE_T_C_CH0) | (1U << SENSE_T_C_CH1))

/** Status bit masks for the tile surface (CH1) — use with status from callback. */
#define SENSE_T_C_SURFACE_TOUCH  IQS323_STATUS_CH_TOUCH(SENSE_T_C_CH_SURFACE)
#define SENSE_T_C_SURFACE_PROX   IQS323_STATUS_CH_PROX(SENSE_T_C_CH_SURFACE)

/** Status bit masks for the external pad (CH0) — pad 8. */
#define SENSE_T_C_EXTERNAL_TOUCH IQS323_STATUS_CH_TOUCH(SENSE_T_C_CH_EXTERNAL)
#define SENSE_T_C_EXTERNAL_PROX  IQS323_STATUS_CH_PROX(SENSE_T_C_CH_EXTERNAL)

/* ================================================================
 * Register map -- 8-bit addresses, 16-bit LE data
 * ================================================================ */

/* Version / ID */
#define IQS323_REG_PRODUCT_NUM       0x00
#define IQS323_REG_MAJOR_VER        0x01
#define IQS323_REG_MINOR_VER        0x02
#define IQS323_REG_HARDWARE_ID      0xE1

/* System (read-only) */
#define IQS323_REG_SYSTEM_STATUS    0x10
#define IQS323_REG_GESTURE_STATUS   0x11
#define IQS323_REG_SLIDER_POSITION  0x12

/* Per-channel data (read-only) */
#define IQS323_REG_CH0_COUNTS       0x13
#define IQS323_REG_CH0_LTA          0x14
#define IQS323_REG_CH1_COUNTS       0x15
#define IQS323_REG_CH1_LTA          0x16
#define IQS323_REG_CH2_COUNTS       0x17
#define IQS323_REG_CH2_LTA          0x18

/* Sensor setup per channel (CH0=0x3x, CH1=0x4x, CH2=0x5x) */
#define IQS323_REG_SENSOR_SETUP(ch) (0x30 + (ch) * 0x10)
#define IQS323_REG_CONV_FREQ(ch)    (0x31 + (ch) * 0x10)
#define IQS323_REG_PROX_CTRL(ch)    (0x32 + (ch) * 0x10)
#define IQS323_REG_PROX_INPUT(ch)   (0x33 + (ch) * 0x10)
#define IQS323_REG_PATTERN_DEF(ch)  (0x34 + (ch) * 0x10)
#define IQS323_REG_PATTERN_SEL(ch)  (0x35 + (ch) * 0x10)
#define IQS323_REG_ATI_SETUP(ch)    (0x36 + (ch) * 0x10)
#define IQS323_REG_ATI_BASE(ch)     (0x37 + (ch) * 0x10)
#define IQS323_REG_ATI_MULTI(ch)    (0x38 + (ch) * 0x10)  /* ATI multipliers/dividers (A.13) */
#define IQS323_REG_COMPENSATION(ch) (0x39 + (ch) * 0x10)  /* per-channel offset (A.14) */

/* Channel UI setup (CH0=0x6x, CH1=0x7x, CH2=0x8x) */
#define IQS323_REG_CH_SETUP(ch)          (0x60 + (ch) * 0x10)
#define IQS323_REG_PROX_SETTINGS(ch)     (0x61 + (ch) * 0x10)
#define IQS323_REG_TOUCH_SETTINGS(ch)    (0x62 + (ch) * 0x10)
#define IQS323_REG_FOLLOWER_WEIGHT(ch)   (0x63 + (ch) * 0x10)

/* Slider config */
#define IQS323_REG_SLIDER_SETUP     0x90
#define IQS323_REG_SLIDER_CALIB     0x91
#define IQS323_REG_SLIDER_RESOLUTION 0x93  /**< Slider output range: position runs 0..this (0 = no slider) */
#define IQS323_REG_ENABLE_MASK      0x94
#define IQS323_REG_DELTA_LINK(ch)   (0x96 + (ch))

/* Gesture config */
#define IQS323_REG_GESTURE_ENABLE   0xA0

/* Filter betas */
#define IQS323_REG_COUNTS_FILTER    0xB0
#define IQS323_REG_LTA_FILTER       0xB1
#define IQS323_REG_LTA_FAST_FILTER  0xB2
#define IQS323_REG_FAST_FILTER_BAND 0xB4

/* System control */
#define IQS323_REG_SYSTEM_CONTROL   0xC0
#define IQS323_REG_NP_REPORT_RATE   0xC1
#define IQS323_REG_LP_REPORT_RATE   0xC2
#define IQS323_REG_ULP_REPORT_RATE  0xC3
#define IQS323_REG_HALT_REPORT_RATE 0xC4
#define IQS323_REG_POWER_TIMEOUT    0xC5

/* General */
#define IQS323_REG_I2C_TIMEOUT      0xD1
#define IQS323_REG_EVENT_TIMEOUTS   0xD2
#define IQS323_REG_EVENTS_ENABLE    0xD3
#define IQS323_REG_I2C_SETTINGS     0xE0

/* ================================================================
 * System Status (0x10) bit masks
 * ================================================================ */

#define IQS323_STATUS_PROX_EVENT    (1U << 0)
#define IQS323_STATUS_TOUCH_EVENT   (1U << 1)
#define IQS323_STATUS_SLIDER_EVENT  (1U << 2)
#define IQS323_STATUS_POWER_EVENT   (1U << 3)
#define IQS323_STATUS_ATI_EVENT     (1U << 4)
#define IQS323_STATUS_ATI_ACTIVE    (1U << 5)
#define IQS323_STATUS_ATI_ERROR     (1U << 6)
#define IQS323_STATUS_RESET_EVENT   (1U << 7)

#define IQS323_STATUS_CH0_PROX      (1U << 8)
#define IQS323_STATUS_CH0_TOUCH     (1U << 9)
#define IQS323_STATUS_CH1_PROX      (1U << 10)
#define IQS323_STATUS_CH1_TOUCH     (1U << 11)
#define IQS323_STATUS_CH2_PROX      (1U << 12)
#define IQS323_STATUS_CH2_TOUCH     (1U << 13)
#define IQS323_STATUS_POWER_MASK    (3U << 14)
#define IQS323_STATUS_POWER_SHIFT   14

/** Per-channel touch/prox masks indexed by channel number. */
#define IQS323_STATUS_CH_PROX(ch)   (1U << (8 + (ch) * 2))
#define IQS323_STATUS_CH_TOUCH(ch)  (1U << (9 + (ch) * 2))

/** Events Enable (0xD3) bits the datasheet defines (A.33): 0-4 and 6. */
#define IQS323_EVENTS_DEFINED       0x005F

/* Invalid communication response (read outside window) */
#define IQS323_INVALID_RESPONSE     0xEEEE

/* ================================================================
 * Gesture Status (0x11) bit masks
 * ================================================================ */

#define IQS323_GESTURE_TAP          (1U << 0)
#define IQS323_GESTURE_SWIPE_POS    (1U << 1)
#define IQS323_GESTURE_SWIPE_NEG    (1U << 2)
#define IQS323_GESTURE_FLICK_POS    (1U << 3)
#define IQS323_GESTURE_FLICK_NEG    (1U << 4)
#define IQS323_GESTURE_HOLD         (1U << 5)
#define IQS323_GESTURE_EVENT        (1U << 6)

/* ================================================================
 * System Control (0xC0) bits
 * ================================================================ */

#define IQS323_CTRL_ACK_RESET       (1U << 0)
#define IQS323_CTRL_SOFT_RESET      (1U << 1)
#define IQS323_CTRL_RE_ATI          (1U << 2)
#define IQS323_CTRL_RESEED          (1U << 3)
#define IQS323_CTRL_POWER_SHIFT     4
#define IQS323_CTRL_POWER_MASK      (7U << 4)
#define IQS323_CTRL_INTERFACE_SEL   (1U << 7)

/* ================================================================
 * Enums
 * ================================================================ */

typedef enum {
    SENSE_T_C_POWER_NORMAL       = 0,  /**< Normal power mode */
    SENSE_T_C_POWER_LOW          = 1,  /**< Low power mode */
    SENSE_T_C_POWER_ULTRA_LOW    = 2,  /**< Ultra-low power mode */
    SENSE_T_C_POWER_HALT         = 3,  /**< Halt (lowest power) */
    SENSE_T_C_POWER_AUTO         = 4,  /**< Auto power mode switching */
    SENSE_T_C_POWER_AUTO_NO_ULP  = 5,  /**< Auto mode, skip ultra-low power */
} sense_t_c_power_mode_t;

/* ================================================================
 * Event callback
 * ================================================================ */

/**
 * Event callback fired by tile_sense_t_c_process() when the device
 * reports new touch, proximity, or gesture events.
 *
 * @param tile    The tile instance
 * @param status  System Status register value (use SENSE_T_C_SURFACE_TOUCH etc.)
 * @param ctx     User context (from config or on_event registration)
 *
 * Always runs in main-loop context (never ISR). I2C is safe to call.
 */
typedef void (*sense_t_c_event_cb_t)(tile_t *tile, uint16_t status, void *ctx);

/* ================================================================
 * Configuration
 * ================================================================ */

/**
 * Optional init config. Pass NULL for defaults (polled mode, no callback).
 */
typedef struct {
    /* RDY pin — set to enable interrupt-driven mode.
     * The RDY pin (pad 3) has a 4.7k pull-up and 100 nF to GND on the tile.
     * Uses hal->gpio_irq_enable() to register the falling-edge ISR.
     * Set to 0 for polled mode (no RDY pin needed). */
    uint8_t rdy_pin;              /**< Platform pin for RDY. 0 = polled mode. */

    /* Event callback — fired by process() on touch/prox/gesture changes */
    sense_t_c_event_cb_t on_event;   /**< Callback. NULL = no callback. */
    void *event_ctx;              /**< User context passed to callback. */

    uint8_t channels;             /**< Channel enable bitmask (bit 0 = CH0 pad 8, bit 1 = CH1 surface). 0 = both. CH2 is not routed and always off. */

    /* Thresholds for every enabled channel. 0 = the driver default,
     * Azoteq's EV-kit values (prox 20, touch 40); the chip's own reset
     * value is 0, which would trigger on noise. */
    uint8_t prox_threshold;       /**< Proximity threshold, delta counts (0 = default 20). */
    uint8_t touch_threshold;      /**< Touch threshold, x/256 of the LTA (0 = default 40). */
} sense_t_c_cfg_t;

/* Hardware ID values */
#define IQS323_HW_ID_3DD   0xF003
#define IQS323_HW_ID_3ED   0xF004

/* ================================================================
 * Public API
 * ================================================================ */

/**
 * @brief  Probe the bus for an IQS323 touch controller.
 *
 * Instance 0 is 0x44 (order code IQS323-001, this tile); instance 1 is
 * 0x58 (IQS323-002). The address is fixed by the order code, not a strap.
 * @param  hal       Tiles HAL handle (I2C bus)
 * @param  instance  Device instance (0 or 1, selects I2C address)
 * @return 1 if found, 0 if not
 */
uint8_t tile_sense_t_c_find(tiles_pal_t *hal, uint8_t instance);

/**
 * @brief  Initialize the touch controller.
 *
 * Soft resets, then configures the chip (see the file header): channels,
 * thresholds, filters, report rates, AUTO power, ATI, event mode. Takes
 * roughly 0.5-2 s, most of it ATI and the forced communication windows.
 *
 * @param  hal       Tiles HAL handle (I2C bus)
 * @param  instance  Device instance (0 or 1, selects I2C address)
 * @param  tile      Tile handle to initialize
 * @param  cfg       Optional config (thresholds, RDY pin, callback). NULL for defaults.
 */
void tile_sense_t_c_init(tiles_pal_t *hal, uint8_t instance,
                         tile_t *tile, const sense_t_c_cfg_t *cfg);

/* ---- Event processing ---- */

/**
 * @brief  Read pending touch/proximity events and update internal state.
 * @studio expose category=tile name=process section=events
 *
 * Call from your main loop.
 * In polled mode: reads status register, fires callback if events present.
 * In RDY mode: returns immediately if no RDY interrupt; reads + fires
 * callback only when the device has new data.
 *
 */
void tile_sense_t_c_process(tile_t *tile);

/**
 * @brief  Register or change the event callback for touch/proximity events.
 * @studio expose category=tile name=on_event section=events
 * @param  cb    Callback function (NULL to disable)
 * @param  ctx   User context passed to callback
 */
void tile_sense_t_c_on_event(tile_t *tile, sense_t_c_event_cb_t cb, void *ctx);

/* ---- Lifecycle ---- */

/**
 * @brief  Enter low-power sleep mode (Halt, about 2 µA).
 * @studio expose category=tile name=sleep section=lifecycle
 *
 * The chip stops sensing; process() does nothing until wake().
 */
void tile_sense_t_c_sleep(tile_t *tile);

/**
 * @brief  Wake from sleep mode.
 * @studio expose category=tile name=wake section=lifecycle
 *
 * Restores the power mode last chosen with set_power_mode() (init: AUTO).
 */
void tile_sense_t_c_wake(tile_t *tile);

/**
 * @brief  Soft-reset the IQS323 (System Control bit 1).
 *
 * All registers return to their reset values (thresholds, report rates
 * and filters 0, every channel on CRx0) and the tile leaves the READY
 * state. Call init() again afterwards.
 *
 */
void tile_sense_t_c_reset(tile_t *tile);

/* ---- Status (from last process() call) ---- */

/**
 * @brief  Read the cached System Status word from the last process() call.
 * @studio expose category=tile name=get_status returns=int section=runtime
 */
uint16_t tile_sense_t_c_get_status(tile_t *tile);

/**
 * @brief  Read the Gesture Status word (a live read, not cached).
 * @studio expose category=tile name=get_gestures returns=int section=runtime
 */
uint16_t tile_sense_t_c_get_gestures(tile_t *tile);

/**
 * @brief  Check if the given channel is currently touched.
 * @studio expose category=tile name=is_touched returns=bool section=runtime
 *
 * Reads the cached System Status from the last process() call.
 *
 * @param  channel  [0..2] 0 = pad 8 electrode, 1 = tile surface, 2 = not routed
 * @return 1 if touched, 0 otherwise
 */
uint8_t  tile_sense_t_c_is_touched(tile_t *tile, uint8_t channel);

/**
 * @brief  Check if the given channel currently reports proximity.
 * @studio expose category=tile name=is_prox returns=bool section=runtime
 *
 * Reads the cached System Status from the last process() call.
 *
 * @param  channel  [0..2] 0 = pad 8 electrode, 1 = tile surface, 2 = not routed
 * @return 1 if in proximity, 0 otherwise
 */
uint8_t  tile_sense_t_c_is_prox(tile_t *tile, uint8_t channel);

/* ---- Data ---- */

/**
 * @brief  Read the filtered counts value for a channel (0x13/0x15/0x17).
 * @studio expose category=tile name=get_counts returns=int section=runtime
 *
 * Self-capacitance counts DROP when a finger approaches.
 *
 * @param  channel  [0..2] Channel
 * @return Filtered counts, or 0 for a bad channel
 */
uint16_t tile_sense_t_c_get_counts(tile_t *tile, uint8_t channel);

/**
 * @brief  Read the long-term average (LTA) for a channel.
 * @studio expose category=tile name=get_lta returns=int section=runtime
 * @param  channel  [0..2] Channel
 * @return LTA baseline in counts
 */
uint16_t tile_sense_t_c_get_lta(tile_t *tile, uint8_t channel);

/**
 * @brief  Read the delta (counts - LTA) for a channel.
 * @studio expose category=tile name=get_delta returns=int section=runtime
 * @param  channel  [0..2] Channel
 * @return counts - LTA; negative when a finger is near a self-cap channel
 */
int16_t  tile_sense_t_c_get_delta(tile_t *tile, uint8_t channel);

/**
 * @brief  Read the slider position (if a slider is configured).
 * @studio expose category=tile name=get_slider returns=int section=runtime
 *
 * Raw Slider Position register (0x12): 0..Slider Resolution (0x93).
 * Stays 0 until a slider is configured (see the slider note above).
 *
 * @return Slider position
 */
uint16_t tile_sense_t_c_get_slider(tile_t *tile);

/* ---- Configuration ---- */

/**
 * @brief  Set proximity and touch thresholds for a channel.
 * @studio expose category=tile name=set_thresholds section=runtime
 *
 * Prox is an absolute delta in counts (A.16, written with 4/4 debounce);
 * touch is relative, threshold/256 of the LTA (A.17). A 0 leaves that
 * register unchanged. Azoteq's EV-kit uses prox 20, touch 40. These
 * per-channel values are lost if the chip resets (process() re-applies the
 * all-channel thresholds from init / set_touch_threshold / set_prox_threshold).
 *
 * @param  channel       [0..2] Channel
 * @param  prox_thresh   [0..255] Prox threshold, delta counts (0 = unchanged)
 * @param  touch_thresh  [0..255] Touch threshold, x/256 of LTA (0 = unchanged)
 */
void tile_sense_t_c_set_thresholds(tile_t *tile, uint8_t channel,
                                   uint8_t prox_thresh, uint8_t touch_thresh);

/**
 * @brief  Set the touch threshold on every enabled channel.
 * @studio expose category=tile name=set_touch_threshold section=config
 * @studio control threshold label="Touch threshold" tier=basic default=40
 *
 * The touch threshold is relative: a touch registers when the counts drop
 * by threshold/256 of the channel's baseline (A.17). Lower is more
 * sensitive (a touch through thicker covers); higher rejects near misses.
 * Kept by the driver, so it survives a chip reset. 0 is ignored.
 *
 * @param  threshold  [1..255] x/256 of the LTA (default 40, about 16 %)
 */
void tile_sense_t_c_set_touch_threshold(tile_t *tile, uint8_t threshold);

/**
 * @brief  Set the proximity threshold on every enabled channel.
 * @studio expose category=tile name=set_prox_threshold section=config
 * @studio control threshold label="Proximity threshold" tier=advanced default=20
 *
 * Proximity is an absolute drop in counts (A.16), written with Azoteq's
 * 4/4 debounce. Lower detects a hand from further away. Kept by the
 * driver, so it survives a chip reset. 0 is ignored.
 *
 * @param  threshold  [1..255] delta counts (default 20)
 */
void tile_sense_t_c_set_prox_threshold(tile_t *tile, uint8_t threshold);

/**
 * @brief  Set the device power mode.
 * @studio expose category=tile name=set_power_mode section=config
 * @studio control mode label="Power mode" tier=advanced default=SENSE_T_C_POWER_AUTO
 *
 * AUTO steps from normal to low and ultra-low power after the power
 * timeout without activity, and back on a touch (datasheet §5.3).
 * Typical draw (3 self-cap channels, EV-kit rates): normal 125 µA, low
 * 37.5 µA, ultra-low 4 µA, halt 2 µA. Values above 5 are ignored.
 *
 * @param  mode  Power mode (System Control bits 6-4); init selects AUTO
 */
void tile_sense_t_c_set_power_mode(tile_t *tile, sense_t_c_power_mode_t mode);

/**
 * @brief  Set the events-enable mask (which events drive RDY pulses).
 * @studio expose category=tile name=enable_events section=config
 * @param  mask  Event bits (IQS323_STATUS_*_EVENT); init enables touch + prox
 */
void tile_sense_t_c_enable_events(tile_t *tile, uint16_t mask);

/**
 * @brief  Trigger an ATI (auto-tuning) cycle and wait for completion.
 * @studio expose category=tile name=ati section=config
 */
void tile_sense_t_c_ati(tile_t *tile);

/* ---- ATI fine-tuning ---- */

/**
 * @brief  Set the per-channel ATI setup register (0x36 + ch*0x10).
 *
 * Writes the raw 16-bit ATI Setup register (0x36 + ch·0x10) for the
 * channel. Bit layout (datasheet §A.12):
 *   - Bits 15-4: ATI Target / Resolution Factor
 *   - Bit  3:    ATI Band (0 = 1/16, 1 = 1/8)
 *   - Bits 2-0:  ATI Mode (0 disabled, 1 comp-only, 2 from-comp-divider,
 *                3 partial, 4 full)
 * (The coarse/fine multipliers and dividers live in the separate ATI
 * Multipliers/Dividers register at 0x38 — see IQS323_REG_ATI_MULTI;
 * the compensation divider lives at 0x39 — see set_compensation.)
 *
 * After changing ATI parameters, call @ref tile_sense_t_c_ati to
 * re-run the auto-tuning sequence with the new values. Useful when
 * ambient capacitance shifts (cover-glass thickness, mounting
 * substrate) push the working point off the chip's defaults.
 *
 * @studio expose category=tile name=set_ati_setup section=config
 * @param  channel  0–2
 * @param  value    Raw 16-bit ATI Setup register value
 */
void tile_sense_t_c_set_ati_setup(tile_t *tile, uint8_t channel, uint16_t value);

/* ---- Filter / conversion frequency tuning ---- */

/**
 * @brief  Set the global counts-filter beta register (0xB0).
 *
 * Datasheet §A.26. Higher beta = more aggressive smoothing on the
 * raw counts (lower noise, slower response). Default is tuned for
 * typical electrode geometries; tune up for noisy environments,
 * tune down for fast-response applications.
 *
 * @studio expose category=tile name=set_counts_filter section=config
 * @param  beta  Raw 16-bit Counts Filter Betas register value
 */
void tile_sense_t_c_set_counts_filter(tile_t *tile, uint16_t beta);

/**
 * @brief  Set per-channel conversion frequency (0x31 + ch*0x10).
 *
 * Datasheet §A.4. Controls charge-transfer frequency and dead time.
 * Tune for unusual electrode geometries, large capacitance loads,
 * or when self-capacitance interactions push beyond the chip's
 * default Conversion Frequency Fraction.
 *
 * @studio expose category=tile name=set_conversion_freq section=config
 * @param  channel  0–2
 * @param  value    Raw 16-bit Conversion Frequency register value
 */
void tile_sense_t_c_set_conversion_freq(tile_t *tile, uint8_t channel,
                                        uint16_t value);

/* ---- Reference channel compensation ---- */

/** Channel Mode for the Reference UI (datasheet §A.15 bits [3:0]). */
typedef enum {
    SENSE_T_C_CHANNEL_INDEPENDENT = 0x00,  /**< Stand-alone sensing channel */
    SENSE_T_C_CHANNEL_FOLLOWER    = 0x01,  /**< Subtracts a reference channel's LTA */
    SENSE_T_C_CHANNEL_REFERENCE   = 0x02,  /**< Acts as ambient reference for followers */
} sense_t_c_channel_mode_t;

/**
 * @brief  Configure a channel's role in the Reference UI.
 *
 * Reference channels measure ambient capacitance (e.g., temperature
 * / humidity drift on the substrate). Follower channels subtract
 * the reference's LTA delta from their own, eliminating common-mode
 * drift. Datasheet §7.3 describes the design pattern.
 *
 * For follower channels, `reference_id` selects which channel to
 * follow (encoded in Channel Setup bits [7:4]). For independent /
 * reference channels, `reference_id` is unused — pass 0.
 *
 * Apply to all participating channels (set the reference channel to
 * REFERENCE first, then set followers). Re-run @ref tile_sense_t_c_ati
 * after changing channel modes.
 *
 * @studio expose category=tile name=set_channel_mode section=config
 * @param  channel       0–2
 * @param  mode          Channel role
 * @param  reference_id  Which channel to follow (only used for FOLLOWER)
 */
void tile_sense_t_c_set_channel_mode(tile_t *tile, uint8_t channel,
                                     sense_t_c_channel_mode_t mode,
                                     uint8_t reference_id);

/* ---- I²C communication mode ---- */

/** I²C communication mode (System Control register §A.30 bit 7). */
typedef enum {
    SENSE_T_C_COMM_STREAM = 0,  /**< Continuous reporting at the configured power-mode rate (the chip's reset default) */
    SENSE_T_C_COMM_EVENT  = 1,  /**< RDY pulses only on enabled events (what init() selects) */
} sense_t_c_comm_mode_t;

/**
 * @brief  Switch between event-mode and streaming-mode I²C.
 *
 * Event mode opens a communication window (RDY low) only for an enabled
 * event (touch or prox change, a reset); lowest power, lowest CPU
 * overhead. init() selects event mode. Streaming mode opens one every
 * report cycle; the chip then waits up to the I2C timeout (200 ms) for
 * the host each time, so use it only with the RDY pin and a host that
 * services every window (continuous logging of counts).
 *
 * @studio expose category=tile name=set_comm_mode section=config
 * @studio control mode label="I2C reporting" tier=advanced default=SENSE_T_C_COMM_EVENT
 * @param  mode  Event or streaming
 */
void tile_sense_t_c_set_comm_mode(tile_t *tile, sense_t_c_comm_mode_t mode);

/* ============================================================== */
/* Runtime — tier-2 idiomatic helpers                              */
/*                                                                  */
/* These compose the tier-1 surface above into "do the thing the   */
/* user wants" calls. They handle process()-driven status caching, */
/* RDY-pin awareness, and integer-only slider scaling internally   */
/* so callers don't need to mind the IQS323's communication-window */
/* protocol or status-bit layout.                                  */
/* ============================================================== */

/**
 * @brief  Check whether *any* channel is currently touched.
 *
 * @studio expose category=tile name=is_touched_any returns=bool section=runtime
 *
 * Reflects the cached System Status from the most recent process()
 * call — it does not perform an I²C read itself, so it's safe to
 * call from any rate. Returns 1 if any of CH0/CH1/CH2 currently
 * reports touch, 0 otherwise. Pair with @ref tile_sense_t_c_process
 * in your main loop.
 *
 * @return 1 if any channel is touched, 0 otherwise
 */
uint8_t tile_sense_t_c_is_touched_any(tile_t *tile);

/**
 * @brief  Block until any channel is touched (or timeout elapses).
 *
 * @studio expose category=tile name=wait_for_touch returns=bool section=runtime
 *
 * Polls process() at ~5 ms cadence and returns as soon as any
 * channel reports touch. If RDY-mode is configured, process() is a
 * no-op until RDY fires, so the poll is effectively interrupt-paced.
 * In polled mode the cadence determines latency. Returns 0 if
 * `timeout_ms` elapses without a touch event.
 *
 * @param  timeout_ms  Maximum time to wait, in milliseconds
 * @return 1 if a touch was detected, 0 on timeout
 */
uint8_t tile_sense_t_c_wait_for_touch(tile_t *tile, uint32_t timeout_ms);

/**
 * @brief  Block until any gesture fires (or timeout elapses).
 *
 * @studio expose category=tile name=wait_for_gesture returns=int section=runtime
 *
 * Polls the Gesture Status register at ~5 ms cadence. Returns the
 * raw 16-bit gesture word as soon as any gesture bit is set
 * (IQS323_GESTURE_TAP / SWIPE_POS / SWIPE_NEG / FLICK_POS /
 * FLICK_NEG / HOLD). Returns 0 on timeout.
 *
 * Gestures require the slider/wheel UI to be configured — a single
 * tile surface alone does not produce gesture events. See the
 * datasheet §7.4 for the slider-config requirements.
 *
 * @param  timeout_ms  Maximum time to wait, in milliseconds
 * @return Raw Gesture Status word (non-zero) on detection, 0 on timeout
 */
uint16_t tile_sense_t_c_wait_for_gesture(tile_t *tile, uint32_t timeout_ms);

/**
 * @brief  Read the slider position scaled to 0–100 percent.
 *
 * @studio expose category=tile name=read_slider_pct returns=bool section=runtime
 * @studio out_scalar out_pct type=uint8_t
 *
 * The IQS323 slider position runs 0..Slider Resolution (register
 * 0x93, datasheet §7.4). This helper reads both and integer-scales
 * the position to 0–100 through `out_pct`. Returns 0 if either read
 * returned an invalid (out-of-window) response, or if the resolution
 * is 0 (no slider configured): `out_pct` is untouched in that case so
 * callers can keep the previous value.
 *
 * Like @ref tile_sense_t_c_get_slider, this requires a valid slider
 * configuration on this chip (the surface plus an electrode on pad 8).
 * A single tile surface alone cannot produce a slider output.
 *
 * @param  out_pct  Pointer to receive the percentage (0–100)
 * @return 1 on success, 0 on read failure (out_pct unmodified)
 */
uint8_t tile_sense_t_c_read_slider_pct(tile_t *tile, uint8_t *out_pct);

/* ---- Low-level ---- */

/**
 * @brief  Read a raw 16-bit IQS323 register.
 * @studio expose category=tile name=read_reg returns=int section=advanced
 * @param  reg   [0..255] Register address (datasheet §9 memory map)
 * @return Register value, 0xEEEE if no comms window opened
 */
uint16_t tile_sense_t_c_read_reg(tile_t *tile, uint8_t reg);

/**
 * @brief  Write a raw 16-bit IQS323 register.
 * @studio expose category=tile name=write_reg section=advanced
 * @param  reg    [0..255] Register address (datasheet §9 memory map)
 * @param  value  16-bit value
 */
void tile_sense_t_c_write_reg(tile_t *tile, uint8_t reg, uint16_t value);

/* ================================================================
 * v1.3 capability additions
 * ================================================================ */

/**
 * @brief  Force the long-term-average baseline to re-snap to current counts.
 *
 * Sets SYSTEM_CONTROL.RESEED (self-clearing). Use after a known
 * environmental change (object placed/removed, cover added) to discard
 * the slow baseline and re-zero the channels immediately, instead of
 * waiting for the LTA filter to drift there.
 *
 * @studio expose category=tile name=reseed section=runtime
 */
void tile_sense_t_c_reseed(tile_t *tile);

/**
 * @brief  Set the report rate (conversion interval) for a power mode.
 *
 * Writes the per-mode report-rate register (NP 0xC1 / LP 0xC2 / ULP 0xC3
 * / HALT 0xC4) in plain milliseconds (clamped 0-3000). This is the tile's
 * headline power/latency knob — faster rate = lower latency, higher
 * current. Only the four reporting modes have a rate; AUTO/AUTO_NO_ULP
 * are ignored. init() writes Azoteq's EV-kit values, NP 16 ms, LP 60 ms,
 * ULP 160 ms, Halt 3000 ms (the chip's reset value is 0 for all four).
 * These are lost if the chip resets; process() restores the defaults.
 *
 * @studio expose category=tile name=set_report_rate section=config
 * @param  mode  Which power mode's rate to set (NORMAL/LOW/ULTRA_LOW/HALT).
 * @param  ms    Report interval in milliseconds (0-3000).
 */
void tile_sense_t_c_set_report_rate(tile_t *tile, sense_t_c_power_mode_t mode,
                                    uint16_t ms);

/**
 * @brief  Set the auto power-mode step-down timeout.
 *
 * Writes POWER_TIMEOUT (0xC5) in milliseconds. In AUTO / AUTO_NO_ULP the
 * chip steps down a power mode after this much inactivity; 0 disables
 * auto step-down. init() writes Azoteq's EV-kit 2000 ms (the chip's reset
 * value is 0).
 *
 * @studio expose category=tile name=set_power_timeout section=config
 * @studio control ms label="Idle time before low power" tier=advanced default=2000 unit=ms
 * @param  ms    [0..65000] Inactivity timeout in milliseconds (0 = off).
 */
void tile_sense_t_c_set_power_timeout(tile_t *tile, uint16_t ms);

/**
 * @brief  Read a channel's compensation (CAPDAC-equivalent offset).
 *
 * Returns the raw Compensation register (0x39 + ch·0x10). The low 10
 * bits are the compensation value; bits 15-11 are the divider. Normally
 * set by ATI — read it to capture a tuned working point.
 *
 * @studio expose category=tile name=get_compensation returns=int section=advanced
 * @param  channel  0-2.
 * @return Raw 16-bit compensation register, or 0 on a bad channel.
 */
uint16_t tile_sense_t_c_get_compensation(tile_t *tile, uint8_t channel);

/**
 * @brief  Override a channel's compensation value + divider.
 *
 * Writes the Compensation register (0x39 + ch·0x10): value (0-1023) in
 * bits 9-0, divider (0-31) in bits 15-11. NOTE: a subsequent full ATI
 * overwrites this — to hold a fixed offset, set the channel's ATI Mode
 * (set_ati_setup) to a non-full mode first.
 *
 * @studio expose category=tile name=set_compensation section=advanced
 * @param  channel  0-2.
 * @param  value    Compensation value (0-1023).
 * @param  divider  Compensation divider (0-31).
 */
void tile_sense_t_c_set_compensation(tile_t *tile, uint8_t channel,
                                     uint16_t value, uint8_t divider);

#endif /* INC_TILE_SENSE_T_C_H_ */
