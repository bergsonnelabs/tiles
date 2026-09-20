/**
 * @file   tile_display_rgbw.h
 * @brief  RGBW LED driver for the Display.RGBW tile (LP5811).
 * @version 2.4.1
 *
 * 4-channel LED driver with independent PWM + current control.
 * Channels: R (LED0), G (LED2), B (LED1), W (LED3).
 *
 * Quick start:
 * @code
 *   tile_t led;
 *   tile_display_rgbw_init(&hal, 0, &led, NULL);
 *   tile_display_rgbw_set_color(&led, 255, 0, 0);    // red
 *   tile_display_rgbw_pulse(&led, 0, 255, 0, 200);   // 200ms green flash
 *   tile_display_rgbw_off(&led);                     // all off
 * @endcode
 *
 * Version history:
 *   v2.4.1 — Documentation only, no behavior change. init()'s own doc
 *            block still claimed "current limits to 50%" after v2.4.0
 *            changed it to ~1 mA; @version had been stale at 2.1.0
 *            since v2.2.0, and the generator emits it as the docs
 *            page's first line; and the eight-address note added in
 *            v2.4.0 sat in the file blurb, which the docs site does
 *            not render, so it reached no reader. Moved to find().
 *   v2.4.0 — Safe-by-default drive. init() now leaves the per-channel
 *            current limit at LP5811_DC_DEFAULT (~1 mA at full PWM)
 *            instead of 0x80 (~25.6 mA). The old default was both
 *            painful to look at on a bare, undiffused LED and roughly
 *            2x above the level at which an abrupt switch-on browns
 *            out a bench fixture. Raise it with set_current() when you
 *            actually want the light. Also documents the PWM > 25
 *            precondition on LOD/LSD, and corrects the multi-address
 *            note below (page bits vs variant bits).
 *   v2.3.0 — init() ramps the boost 3.0 -> 4.5 V in 0.1 V committed
 *            steps instead of one slam. The single-step commit's
 *            inrush can brown-out a marginal supply (long leads,
 *            loaded USB rail) and reset the host MCU.
 *   v2.1.0 — Tier-2 idiomatic helpers (set_color, pulse, breathe,
 *            flash, is_faulted) + section= tagging for Coverage Table.
 *   v2.0.0 — Initial tier-1 surface (set, off, current, faults).
 *
 * @studio tile label=Display.RGBW icon=◑
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=niche category="Multi-address support (0x50/0x54/0x58/0x5C)"
 *   Chip-gated. Per datasheet Table 7-4, Address Byte 1 carries the
 *   chip address AND the top two bits of the 10-bit register address,
 *   so the 7-bit address a bus scan sees is:
 *
 *     independent:  1 0 1 Bit4 Bit3 RA9 RA8
 *     broadcast:    1 1 0  1    1   RA9 RA8
 *
 *   Bit4:Bit3 select the material variant (LP5811A/B/C/D, datasheet
 *   §4 Device Comparison) and move the base address in steps of 4:
 *   A = 0x50, then 0x54, 0x58, 0x5C. RA9:RA8 are the REGISTER PAGE,
 *   which is why one A-variant part answers across 0x50-0x53 — that
 *   is the mechanism lp_read_page() relies on, not four devices.
 *   (An earlier version of this note claimed the variants sat at
 *   0x50/0x51/0x52/0x53. They do not; that range is one chip's four
 *   pages, and a second part placed at 0x51 would collide with it.)
 *
 *   The variant bits are fixed at the factory — not pin-strapped or
 *   register-configurable — and Display.RGBW (rev a) ships only the
 *   A variant, so alternate addresses need a tile hardware revision.

 */

#ifndef INC_TILE_DISP_RGBW_H_
#define INC_TILE_DISP_RGBW_H_

#include "tiles.h"
#include <stdint.h>

/* ---- Driver version ---- */

#define TILE_DISP_RGBW_VERSION_MAJOR  2
#define TILE_DISP_RGBW_VERSION_MINOR  4
#define TILE_DISP_RGBW_VERSION_PATCH  1

TILES_CHECK_VERSION(1, 0);

/* ---- Instance mapping ---- */

/**
 * | Instance | ID   | Hardware config         |
 * |----------|------|-------------------------|
 * | 0        | 0x50 | LP5811A (Bit4=0,Bit3=0) |
 *
 * @note  The LP5811 has four factory-strapped material variants
 *        (LP5811A/B/C/D) with hard-wired I2C addresses 0x50/0x51/
 *        0x52/0x53. The Display.RGBW tile (rev a) ships with the A
 *        variant only — see the chip-gated note in the multi-address
 *        unsupported annotation.
 */
#define LP5811_I2C_ADDR_DEFAULT  0x50

/* ---- LP5811 registers (page-0 offsets unless noted) ---- */

#define LP5811_REG_CHIP_EN      0x00
#define LP5811_REG_CONFIG_0     0x01  /* bits[5:1] boost_vout (3.0V + 0.1V*code),
                                         bit[0] max_current (1 = 51 mA) */
#define LP5811_REG_CONFIG_2     0x03
#define LP5811_REG_CONFIG_12    0x0D
#define LP5811_REG_CMD_UPDATE   0x10
#define LP5811_REG_LED_EN       0x20
#define LP5811_REG_FAULT_CLEAR  0x22
#define LP5811_REG_RESET        0x23
#define LP5811_REG_DC_0         0x30  /* Current limit channel 0 (R) */
#define LP5811_REG_DC_1         0x31  /* Current limit channel 1 (B) */
#define LP5811_REG_DC_2         0x32  /* Current limit channel 2 (G) */
#define LP5811_REG_DC_3         0x33  /* Current limit channel 3 (W) */
#define LP5811_REG_PWM_0        0x40  /* PWM channel 0 (R) */
#define LP5811_REG_PWM_1        0x41  /* PWM channel 1 (B) */
#define LP5811_REG_PWM_2        0x42  /* PWM channel 2 (G) */
#define LP5811_REG_PWM_3        0x43  /* PWM channel 3 (W) */

/* Page-3 status registers (0x300+ — accessed via address-bump trick) */
#define LP5811_REG_TSD_STATUS   0x00  /* page 3, offset 0x00 (=0x300) */
#define LP5811_REG_LOD_STATUS_0 0x01  /* page 3, offset 0x01 (=0x301) */
#define LP5811_REG_LSD_STATUS_0 0x03  /* page 3, offset 0x03 (=0x303) */

#define LP5811_CONFIG_2_DEFAULT 0xE4  /* Used to verify chip is alive */

/* ---- Boost bring-up (Dev_Config_0) ---- */

#define LP5811_BOOST_VOUT_CODE_4V5  15u   /**< boost_vout code: 3.0 + 15*0.1 = 4.5 V */
#define LP5811_CONFIG_0_MC_51MA     0x01u /**< max_current bit: 51 mA full-scale */

/**
 * Boost ramp step delay used by init(). One CMD_Update per 0.1 V code,
 * 16 commits total (~16 * LP5811_BOOST_RAMP_STEP_MS of init time).
 * Stepping avoids the single-slam inrush that can collapse a marginal
 * supply — see the v2.3.0 note in the version history.
 *
 * 100 ms/step (~1.6 s init) is the bench-proven conservative value
 * from the 2026-07 panel bring-up rig (long soldered leads). Faster
 * steps likely work on solid supplies but are unvalidated — tune down
 * once socketed-tile testing confirms margin.
 */
#define LP5811_BOOST_RAMP_STEP_MS   100u

/* ---- Safe-by-default drive ---- */

/**
 * Per-channel DC current limit left in place by init(). Current is
 * full_scale * (DC/255) * (PWM/255), so at the 51 mA full scale this
 * is ~1 mA per channel with PWM at maximum.
 *
 * Chosen against two measured limits on a Display.RGBW rev a
 * (bench fixture, Core.ST.L4.1, USB-fed rail, 2026-09-04):
 *
 *   - Comfort. ~6.4 mA on a bare, undiffused LED is genuinely painful
 *     to look at from bench distance; ~2 mA on all four channels at
 *     once still reads as "flashlight bright". The tile carries no
 *     diffuser, so during bring-up you look straight at the die.
 *     Note the asymmetry: this limit is PER CHANNEL, and a white
 *     set(255,255,255,255) is four of them at once.
 *   - Supply. An abrupt PWM 0 -> 255 switch-on survived up to ~9.6 mA
 *     and browned the host MCU out at ~12.8 mA. The v2.3.0 boost ramp
 *     does not help here: it fixes the slew *to* 4.5 V, not the load
 *     step once a sink turns on.
 *
 * The previous default (0x80, ~25.6 mA at 51 mA scale) sat above both.
 * ~1 mA per channel is clearly legible as an indicator — it is the
 * level at which all four colors were correctly identified by eye
 * during post-saw acceptance — and leaves ~10x margin to the measured
 * brown-out. Call set_current() when you want the light —
 * that is the intended escape hatch, and it is worth knowing that
 * driving hard AND switching on abruptly is what actually bites.
 */
#define LP5811_DC_DEFAULT            0x05u

/**
 * Minimum PWM at which the chip will perform open/short detection.
 *
 * Per datasheet §7.3.5.3 and §7.3.5.4, LOD and LSD "can only be
 * performed when the PWM setting of this LED is above 25" — the chip
 * needs the on-time to make the measurement. Note this gates on PWM,
 * NOT on current, so backing the DC limit down for brightness does not
 * cost you fault coverage; running a channel at a low duty does.
 *
 * A channel sitting below this reports clean regardless of its true
 * state. See the warning on read_faults().
 */
#define LP5811_PWM_FAULT_DETECT_MIN   25u

/* ---- Autonomous-animation config registers ---- */
#define LP5811_REG_CONFIG_3     0x04  /* auto_en[3:0] — per-LED autonomous enable */
#define LP5811_REG_CONFIG_5     0x06  /* exp_en[3:0]  — per-LED exponential dimming */
#define LP5811_REG_CONFIG_7     0x08  /* phase_align — 2 bits per LED */

/* ---- Animation command registers (write the magic byte to trigger) ---- */
#define LP5811_REG_CMD_START    0x11  /* 0xFF = start / restart */
#define LP5811_REG_CMD_STOP     0x12  /* 0xAA = stop → INITIAL state */
#define LP5811_REG_CMD_PAUSE    0x13  /* 0x33 = pause */
#define LP5811_REG_CMD_CONTINUE 0x14  /* 0xCC = continue */

/* ---- Autonomous animation register block (page 0, 0x80–0xE7) ----
 * Per-LED block of 0x1A bytes: base = 0x80 + channel*0x1A.
 *   +0x00 Auto_Pause    (tp_ts[7:4] start pause, tp_te[3:0] end pause)
 *   +0x01 Auto_Playback (aeu_num[5:4], pt[3:0] repeat: 0-14, Fh = infinite)
 *   AEUk (k = 1..3) at +0x02 + (k-1)*0x08:
 *     +0..+4  PWM1..PWM5 (0-255 = 0-100%, keyframe levels)
 *     +5      T12 (t2[7:4], t1[3:0])  slope-time codes (PWM1→2, 2→3)
 *     +6      T34 (t4[7:4], t3[3:0])  slope-time codes (PWM3→4, 4→5)
 *     +7      Playback (pt[1:0], 3 = infinite) */
#define LP5811_LED_ANIM_BASE(ch)  (uint8_t)(0x80 + (ch) * 0x1A)
#define LP5811_AEU_OFFSET(aeu)    (uint8_t)(0x02 + ((aeu) - 1) * 0x08)

/* ---- Maximum-current selection (MC bit) ---- */

/**
 * @brief  Per-channel maximum-current selector (LP5811 MC bit).
 *
 * The LP5811 has one global "max current" bit that gates the upper
 * limit for every channel's current sink. Switching the bit also
 * rescales the 8-bit DC code (`tile_display_rgbw_set_current`) over
 * the new range — DC=255 always means full-scale. Default at init: 51 mA.
 */
typedef enum {
    DISP_RGBW_MAX_CURRENT_25_5_MA = 0,  /**< 25.5 mA full scale per channel */
    DISP_RGBW_MAX_CURRENT_51_MA   = 1,  /**< 51 mA full scale per channel */
} disp_rgbw_max_current_t;

/* ---- LED fault status ---- */

/**
 * @brief  LED open / short fault snapshot.
 *
 * Bit N of each mask corresponds to LED channel N. Mapping:
 *   bit0 = R, bit1 = B, bit2 = G, bit3 = W (matches `set()` order
 *   inside the chip). Faults are sticky in the chip — read once,
 *   then call `clear_faults()` to reset the latches.
 */
typedef struct {
    uint8_t open_mask;       /**< Bits 3:0 — channels with open-circuit fault. */
    uint8_t short_mask;      /**< Bits 3:0 — channels with short-circuit fault. */
    uint8_t thermal_shutdown;/**< 1 = chip in thermal shutdown (TSD). */
    uint8_t config_error;    /**< 1 = configuration error reported by chip. */
} disp_rgbw_faults_t;

/**
 * @brief  Short-circuit detection threshold (fraction of VOUT).
 */
typedef enum {
    DISP_RGBW_LSD_TH_0_35 = 0,  /**< 0.35 × VOUT (most sensitive) */
    DISP_RGBW_LSD_TH_0_45 = 1,  /**< 0.45 × VOUT */
    DISP_RGBW_LSD_TH_0_55 = 2,  /**< 0.55 × VOUT */
    DISP_RGBW_LSD_TH_0_65 = 3,  /**< 0.65 × VOUT (least sensitive — driver default) */
} disp_rgbw_lsd_threshold_t;

/* ---- Public API ---- */

/**
 * @brief  Check if a Disp.RGBW is present on the bus.
 *
 * Probes the instance's independent-mode base address only (0x50 for
 * instance 0).
 *
 * @note A healthy Display.RGBW ACKs on EIGHT addresses, and all eight
 *       are the same chip. Per datasheet Table 7-4 the 7-bit address
 *       carries the chip address AND the top two bits of the 10-bit
 *       register address:
 *
 *         independent:  1 0 1 Bit4 Bit3 RA9 RA8   -> 0x50-0x53
 *         broadcast:    1 1 0  1    1   RA9 RA8   -> 0x6C-0x6F
 *
 *       So 0x50-0x53 are the four register pages in independent mode
 *       and 0x6C-0x6F are the same pages under the broadcast address.
 *       A bus scan showing all eight is one part behaving correctly —
 *       not a second device, and not a board fault. Do not go hunting
 *       for it; it has cost bring-up time before.
 *
 * @param  hal       Platform abstraction handle
 * @param  instance  Device instance (0 = default address 0x50)
 * @return 1 if the device ACKs, 0 otherwise
 */
uint8_t tile_display_rgbw_find(tiles_pal_t *hal, uint8_t instance);

/**
 * Optional init config. Pass NULL for defaults.
 * Reserved for future use (e.g., initial brightness, current limits).
 */
typedef struct {
    uint8_t reserved;   /**< Placeholder — no options yet. */
} disp_rgbw_cfg_t;

/**
 * @brief  Initialize the LP5811 LED driver.
 *
 * Enables the chip, ramps the boost to 4.5 V in 0.1 V committed
 * steps (~1.6 s — avoids the single-step inrush that can brown-out
 * a marginal supply; see version history v2.3.0), sets max current
 * full scale to 51 mA, enables all 4 LED channels, and sets the
 * per-channel current limit to LP5811_DC_DEFAULT — ~1 mA at full
 * PWM, NOT half scale. See that constant for why it is deliberately
 * low and how to raise it. LSD action is left at "no shutdown"
 * (driver-level choice) so
 * a transient short doesn't latch the device into OFAF state without
 * firmware seeing it. Pass cfg=NULL for defaults.
 *
 * @param  hal       Platform abstraction handle
 * @param  instance  Device instance (0 = default address 0x50)
 * @param  tile      Tile handle to populate
 * @param  cfg       Optional config, or NULL for defaults
 */
void tile_display_rgbw_init(tiles_pal_t *hal, uint8_t instance, tile_t *tile,
                         const disp_rgbw_cfg_t *cfg);

/**
 * @brief Set RGBW output levels.
 *
 * @studio expose category=tile icon=◑ name=set section=runtime
 * @param r [0..255] Red PWM.
 * @param g [0..255] Green PWM.
 * @param b [0..255] Blue PWM.
 * @param w [0..255] White PWM.
 */
void tile_display_rgbw_set(tile_t *tile, uint8_t r, uint8_t g, uint8_t b, uint8_t w);

/**
 * @brief Turn all LEDs off (PWM = 0).
 *
 * @studio expose category=tile icon=◑ name=off section=runtime
 */
void tile_display_rgbw_off(tile_t *tile);

/**
 * @brief Set per-channel current limit.
 *
 * @studio expose category=tile icon=◑ name=set_current section=runtime
 * @param r [0..255] Red current (fraction of full-scale max).
 * @param g [0..255] Green current.
 * @param b [0..255] Blue current.
 * @param w [0..255] White current.
 */
void tile_display_rgbw_set_current(tile_t *tile, uint8_t r, uint8_t g, uint8_t b, uint8_t w);

/**
 * @brief Set the global maximum-current range (MC bit).
 *
 * Selects 25.5 mA or 51 mA full-scale per channel. After changing,
 * the 8-bit per-channel DC codes (set via `set_current()`) re-scale
 * automatically — DC=255 always means full-scale current. Useful
 * when wiring lower-rated LEDs (drop to 25.5 mA to keep DC resolution
 * fine) or when running cooler / saving power.
 *
 * The driver writes `Dev_Config_0` and re-issues the `CMD_Update`
 * latch (0x55) the chip requires for config-register writes to
 * actually take effect.
 *
 * @studio expose category=tile icon=◑ name=set_max_current section=config
 * @param  tile  Initialised tile handle
 * @param  mode  25.5 mA (0) or 51 mA (1)
 */
void tile_display_rgbw_set_max_current(tile_t *tile, disp_rgbw_max_current_t mode);

/**
 * @brief Read the per-channel open / short / thermal fault state.
 *
 * Pulls TSD_Config_Status, LOD_Status_0, and LSD_Status_0 from the
 * chip's page-3 register space (the LP5811 multiplexes register
 * pages onto two extra address bits — the driver handles this
 * transparently). Faults latch in the chip until cleared via
 * `clear_faults()`.
 *
 * Open-circuit threshold (VLOD_TH) is fixed by the chip at ~70 mV
 * (25.5 mA mode) or ~180 mV (51 mA mode). Short-circuit threshold
 * (VLSD_TH) is configurable via `set_short_threshold()`.
 *
 * @warning Detection requires that channel's PWM to be above
 *          LP5811_PWM_FAULT_DETECT_MIN (25) — datasheet §7.3.5.3 and
 *          §7.3.5.4. A channel that is dark, or driven at a very low
 *          duty, reports NO fault whether or not one exists. Light the
 *          channel you care about, give it a moment to settle, and read
 *          faults while it is still lit. Reading them on an idle part
 *          returns a clean bill of health that means nothing.
 *
 * @studio expose category=tile icon=◑ name=read_faults section=runtime
 * @param  tile  Initialised tile handle
 * @param  out   Caller-allocated fault snapshot (zeroed on entry)
 */
void tile_display_rgbw_read_faults(tile_t *tile, disp_rgbw_faults_t *out);

/**
 * @brief Clear all latched LED open / short / TSD fault flags.
 *
 * Writes 0x07 to Fault_Clear (W1C — write 1 to clear). After this
 * call, `read_faults()` reflects only currently-active faults.
 *
 * @studio expose category=tile icon=◑ name=clear_faults section=runtime
 * @param  tile  Initialised tile handle
 */
void tile_display_rgbw_clear_faults(tile_t *tile);

/**
 * @brief Set the short-circuit detection threshold (fraction of VOUT).
 *
 * Lower thresholds catch milder partial-shorts; higher thresholds
 * tolerate more LED forward-voltage variation without false alarms.
 * Driver default (init) is 0.65 × VOUT — most permissive, least
 * likely to mis-fire on cold LEDs whose Vf hasn't settled.
 *
 * @studio expose category=tile icon=◑ name=set_short_threshold section=config
 * @param  tile       Initialised tile handle
 * @param  threshold  One of DISP_RGBW_LSD_TH_*
 */
void tile_display_rgbw_set_short_threshold(tile_t *tile,
                                           disp_rgbw_lsd_threshold_t threshold);

/**
 * @brief Configure whether a short-circuit fault auto-disables outputs.
 *
 * When enabled, an LSD fault sends the chip into OFAF (one-fail-all-
 * fail) state — every channel turns off until LSD_Clear is written.
 * When disabled (driver default), the chip flags the fault but keeps
 * driving — firmware decides what to do.
 *
 * @studio expose category=tile icon=◑ name=set_short_shutdown section=config
 * @param  tile     Initialised tile handle
 * @param  enabled  1 = chip auto-shuts-down on LSD, 0 = report only
 */
void tile_display_rgbw_set_short_shutdown(tile_t *tile, uint8_t enabled);

/**
 * @brief Configure whether an open-circuit fault auto-disables a sink.
 *
 * When enabled (chip default), a per-channel LOD fault turns off
 * that single current sink. When disabled, the chip flags the fault
 * via `read_faults()` but keeps driving the channel — useful when
 * the open is intermittent (e.g., a flexing wire) and firmware
 * wants to retry rather than relying on the chip to recover.
 *
 * @studio expose category=tile icon=◑ name=set_open_shutdown section=config
 * @param  tile     Initialised tile handle
 * @param  enabled  1 = chip auto-shuts-down a single sink, 0 = report only
 */
void tile_display_rgbw_set_open_shutdown(tile_t *tile, uint8_t enabled);

/**
 * @brief Enter sleep (disable chip).
 *
 * @studio expose category=tile icon=◑ name=sleep section=lifecycle
 */
void tile_display_rgbw_sleep(tile_t *tile);

/**
 * @brief Wake (re-enable chip, LEDs retain previous state).
 *
 * @studio expose category=tile icon=◑ name=wake section=lifecycle
 */
void tile_display_rgbw_wake(tile_t *tile);

/**
 * @brief  Software reset. Must call init() again after.
 *
 * @studio expose category=tile icon=◑ name=reset section=lifecycle
 */
void tile_display_rgbw_reset(tile_t *tile);

/* ============================================================== */
/* Runtime — tier-2 idiomatic helpers                              */
/*                                                                  */
/* These compose the tier-1 surface above into "do the thing the   */
/* user wants to do" calls. Most apps drive RGB only and want      */
/* simple verbs — set_color / pulse / breathe / flash — not raw    */
/* PWM channel writes. White is left at zero in these helpers; if  */
/* you need RGBW, drop down to the tier-1 `set()` call.            */
/* ============================================================== */

/**
 * @brief  Set a solid RGB colour (W channel forced off).
 *
 * @studio expose category=tile icon=◑ name=set_color section=runtime
 *
 * Convenience wrapper around @ref tile_display_rgbw_set for the
 * common case where only the RGB channels matter. White is held at
 * zero — drop to `set()` directly if you want to mix white in.
 *
 * @param  tile  Initialised tile handle
 * @param  r     Red PWM   [0..255]
 * @param  g     Green PWM [0..255]
 * @param  b     Blue PWM  [0..255]
 */
void tile_display_rgbw_set_color(tile_t *tile, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief  Show a colour for `ms` milliseconds, then turn off.
 *
 * @studio expose category=tile icon=◑ name=pulse section=runtime
 *
 * Simple alert / acknowledgement idiom. Blocking — the call returns
 * after `ms` has elapsed and the LEDs have been turned back off. White
 * is forced off (RGB-only); use @ref tile_display_rgbw_set + manual
 * delay if you need full RGBW pulse control.
 *
 * @param  tile  Initialised tile handle
 * @param  r     Red PWM   [0..255]
 * @param  g     Green PWM [0..255]
 * @param  b     Blue PWM  [0..255]
 * @param  ms    Duration in milliseconds
 */
void tile_display_rgbw_pulse(tile_t *tile, uint8_t r, uint8_t g, uint8_t b,
                             uint16_t ms);

/**
 * @brief  Continuous fade in/out (one breath cycle).
 *
 * @studio expose category=tile icon=◑ name=breathe section=runtime
 *
 * The "thinking" indicator. Plays a single up-and-down PWM ramp over
 * `period_ms` (i.e., dark → peak → dark = one full breath). Loop in
 * the caller for sustained breathing. Implementation is a software
 * loop — the LP5811 has on-chip animation engines (AEU) that could
 * run this autonomously, but the AEU bytecode/timing semantics aren't
 * fully documented in the public datasheet (see @ref tile_display_rgbw.h
 * "unsupported AEU" annotation). The software loop is fine for
 * indicator-grade breathing at v2.1; revisit when AEU lands.
 *
 * @note  Blocking. Spends `period_ms` in `delay_ms()`. Call from a
 *        dedicated task or accept the stall — the function does not
 *        yield to other peripherals.
 *
 * @note  White is forced off (RGB-only). Step granularity is fixed
 *        at 32 PWM levels per half-cycle; with `period_ms < 64` the
 *        delay-per-step rounds to 1 ms and the breath becomes choppy.
 *
 * @param  tile        Initialised tile handle
 * @param  r           Peak red PWM   [0..255]
 * @param  g           Peak green PWM [0..255]
 * @param  b           Peak blue PWM  [0..255]
 * @param  period_ms   One full breath duration in milliseconds (>= 64 recommended)
 */
void tile_display_rgbw_breathe(tile_t *tile, uint8_t r, uint8_t g, uint8_t b,
                               uint16_t period_ms);

/**
 * @brief  N quick on/off blinks at the given colour.
 *
 * @studio expose category=tile icon=◑ name=flash section=runtime
 *
 * Notification idiom — alarm, error indication, "got it" ack. Each
 * blink is 100 ms on, 100 ms off (200 ms per cycle). White is forced
 * off (RGB-only). Leaves the LEDs off after the final blink.
 *
 * @note  Blocking. Total runtime is approximately `count * 200` ms.
 *        Call from a dedicated task or accept the stall.
 *
 * @param  tile   Initialised tile handle
 * @param  r      Red PWM   [0..255]
 * @param  g      Green PWM [0..255]
 * @param  b      Blue PWM  [0..255]
 * @param  count  Number of on/off cycles (1..255)
 */
void tile_display_rgbw_flash(tile_t *tile, uint8_t r, uint8_t g, uint8_t b,
                             uint8_t count);

/**
 * @brief  Quick yes/no on whether any LP5811 fault is currently latched.
 *
 * @studio expose category=tile icon=◑ name=is_faulted returns=bool section=runtime
 *
 * Wraps @ref tile_display_rgbw_read_faults and returns 1 if any fault
 * bit is set — open-circuit (LOD), short-circuit (LSD), thermal
 * shutdown (TSD), or configuration error. For per-channel detail
 * (which LED is open / shorted), use `read_faults()` directly. Faults
 * stay latched until cleared via @ref tile_display_rgbw_clear_faults.
 *
 * @param  tile  Initialised tile handle
 * @return 1 if any fault bit is set, 0 if healthy
 */
uint8_t tile_display_rgbw_is_faulted(tile_t *tile);

/* ============================================================== */
/* Autonomous animation engine (AEU)                               */
/* ============================================================== */

/**
 * @brief  One animation sub-engine (AEU) program for a channel.
 *
 * An AEU plays a 5-keyframe ramp PWM1→PWM2→PWM3→PWM4→PWM5 with four
 * slope times between the keyframes, repeated `repeats` times. Three
 * AEUs (1..3) can be chained per channel (see set_animation).
 */
typedef struct {
    uint8_t pwm[5];   /**< Keyframe brightness levels, 0-255 = 0-100%.   */
    uint8_t t[4];     /**< Slope-time codes T1..T4 (0-15, see ms_to_slope).
                           0 = no time (instant), 0xF ≈ 8.05 s.          */
    uint8_t repeats;  /**< AEU pattern repeat: 0-2, 3 = infinite.        */
} display_rgbw_aeu_t;

/**
 * @brief  Convert a duration in milliseconds to the nearest AEU slope/pause code.
 * @studio expose category=tile name=ms_to_slope section=config
 * @param  ms  Duration in milliseconds (0-8050).
 * @return 4-bit time code (0-15) for an AEU T-field or Auto_Pause field.
 */
uint8_t tile_display_rgbw_ms_to_slope(uint16_t ms);

/**
 * @brief  Put a channel into (or out of) autonomous animation mode.
 *
 * In autonomous mode the on-chip engine runs the channel's AEU program
 * with no MCU intervention; in manual mode the channel follows the PWM
 * registers (set / set_color). Call update() after configuring.
 *
 * @studio expose category=tile name=set_autonomous section=config
 * @param  channel  0-3 (R, B, G, W).
 * @param  enabled  1 = autonomous, 0 = manual.
 */
void tile_display_rgbw_set_autonomous(tile_t *tile, uint8_t channel, uint8_t enabled);

/**
 * @brief  Program one AEU sub-engine of a channel.
 * @studio expose category=tile name=set_aeu section=config
 * @param  channel  0-3.
 * @param  aeu      Sub-engine index 1-3.
 * @param  prog     5-keyframe ramp + slope times + repeat.
 */
void tile_display_rgbw_set_aeu(tile_t *tile, uint8_t channel, uint8_t aeu,
                               const display_rgbw_aeu_t *prog);

/**
 * @brief  Set a channel's overall autonomous playback.
 * @studio expose category=tile name=set_animation section=config
 * @param  channel      0-3.
 * @param  num_aeu      How many AEUs to chain, 1-3.
 * @param  pause_start  Start-of-pattern pause code (0-15, see ms_to_slope).
 * @param  pause_end    End-of-pattern pause code (0-15).
 * @param  repeats      Whole-pattern repeat: 0-14, 15 = infinite.
 */
void tile_display_rgbw_set_animation(tile_t *tile, uint8_t channel, uint8_t num_aeu,
                                     uint8_t pause_start, uint8_t pause_end, uint8_t repeats);

/**
 * @brief  Enable/disable exponential PWM dimming for a channel (Dev_Config_5).
 * @studio expose category=tile name=set_exp_dimming section=config
 * @param  channel  0-3.
 * @param  enabled  1 = exponential curve, 0 = linear.
 */
void tile_display_rgbw_set_exp_dimming(tile_t *tile, uint8_t channel, uint8_t enabled);

/**
 * @brief  Set a channel's PWM phase-align method (Dev_Config_7).
 * @studio expose category=tile name=set_phase_align section=config
 * @param  channel  0-3.
 * @param  mode     0/1 = forward, 2 = middle, 3 = backward align.
 */
void tile_display_rgbw_set_phase_align(tile_t *tile, uint8_t channel, uint8_t mode);

/**
 * @brief  Latch pending config/animation register writes (CMD_Update = 0x55).
 *
 * The LP5811 only acts on Dev_Config / animation writes once this is
 * issued. Call after set_aeu / set_animation / set_autonomous and
 * before animate_start.
 *
 * @studio expose category=tile name=update section=runtime
 */
void tile_display_rgbw_update(tile_t *tile);

/**
 * @brief  Start (or restart) autonomous animation on all enabled channels.
 * @studio expose category=tile name=animate_start section=runtime
 */
void tile_display_rgbw_animate_start(tile_t *tile);

/**
 * @brief  Stop autonomous animation and return to the INITIAL state.
 * @studio expose category=tile name=animate_stop section=runtime
 */
void tile_display_rgbw_animate_stop(tile_t *tile);

/**
 * @brief  Pause autonomous animation, holding the current output.
 * @studio expose category=tile name=animate_pause section=runtime
 */
void tile_display_rgbw_animate_pause(tile_t *tile);

/**
 * @brief  Resume autonomous animation after a pause.
 * @studio expose category=tile name=animate_continue section=runtime
 */
void tile_display_rgbw_animate_continue(tile_t *tile);

/**
 * @brief  Configure + start an autonomous breathe on one channel.
 *
 * Builds a single-AEU symmetric ramp (0 → peak → 0) on the channel,
 * enables autonomous mode, latches, and starts. Convenience wrapper
 * over the AEU API for the common case.
 *
 * @studio expose category=tile name=breathe_auto section=runtime
 * @param  channel    0-3.
 * @param  peak       Peak brightness 0-255.
 * @param  period_ms  Full breathe period (up+down) in ms.
 * @param  repeats    Whole-pattern repeat: 0-14, 15 = infinite.
 */
void tile_display_rgbw_breathe_auto(tile_t *tile, uint8_t channel, uint8_t peak,
                                    uint16_t period_ms, uint8_t repeats);

#endif /* INC_TILE_DISP_RGBW_H_ */
