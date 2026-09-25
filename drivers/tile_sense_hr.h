/**
 * @file  tile_sense_hr.h
 * @brief MAX86174A optical heart-rate and SpO₂ front end (PPG).
 *
 * Platform-agnostic driver for the Analog Devices MAX86174A dual-channel
 * photoplethysmography (PPG) analog front end, paired on Sense.HR with
 * two Vishay VEMD1060X01 silicon PIN photodiodes and four emitters
 * (green ×2, red 645 nm, infrared 940 nm) for reflective optical sensing.
 *
 * Key specifications:
 *  - 20-bit PPG ADC, 4 / 8 / 16 / 32 µA full-scale photocurrent ranges
 *  - Two independent readout channels (PPG1, PPG2), two photodiode inputs
 *  - Six measurement slots per frame, each driving one LED at 0–100 mA
 *    (the chip goes to 127.5 mA; the driver caps at the emitters' 100 mA pulse rating)
 *  - Frame rates 1 to 2048 fps from an internal 32.768 kHz oscillator
 *  - 256-sample FIFO, tagged per measurement slot and channel
 *  - Analog ambient-light cancellation plus a 0–28 µA offset DAC
 *  - Automatic proximity mode drops to 8 fps until skin is detected
 *  - Configurable INTB output (frame ready, FIFO almost full, thresholds)
 *
 * Quick start (single-wavelength heart rate, polled):
 * @code
 *   #include "core.h"
 *   #include "core_tiles.h"
 *   #include "tile_sense_hr.h"
 *
 *   tile_t hr;
 *   tile_sense_hr_init(core_tiles_pal(&core_i2c1), 0, &hr, NULL);
 *   tile_sense_hr_auto_gain(&hr, 0, 0);  // find LED current + offset for slot 0 (0 = 10 mA cap)
 *   tile_sense_hr_start(&hr);
 *   while (1) {
 *       if (tile_sense_hr_frame_ready(&hr)) {
 *           int32_t ppg = tile_sense_hr_read_ppg(&hr, 0);   // green, PPG1
 *           // pulsatile AC rides on a large DC — band-pass on the host
 *       }
 *   }
 * @endcode
 *
 * Multi-wavelength (green + red + IR interleaved in one frame):
 * @code
 *   sense_hr_cfg_t cfg = { .fps = 128 };
 *   tile_sense_hr_init(core_tiles_pal(&core_i2c1), 0, &hr, &cfg);
 *   tile_sense_hr_set_slot(&hr, 0, SENSE_HR_LED_GREEN, 5000);
 *   tile_sense_hr_set_slot(&hr, 1, SENSE_HR_LED_RED,   5000);
 *   tile_sense_hr_set_slot(&hr, 2, SENSE_HR_LED_IR,    5000);
 *   for (uint8_t s = 0; s < 3; s++) tile_sense_hr_auto_gain(&hr, s, 0);
 *   tile_sense_hr_start(&hr);
 *   int32_t frame[SENSE_HR_FRAME_LEN];
 *   if (tile_sense_hr_read_frame(&hr, frame)) {
 *       int32_t green = frame[0], red = frame[2], ir = frame[4];  // PPG1
 *   }
 * @endcode
 *
 * Electrical notes (from Sense-HR-a bring-up, August 2026):
 *  - V+ (pad 10) is a 1.8 V rail with a 2.2 V absolute maximum. VLED
 *    (pad 9, 2.7–5.5 V) feeds the emitter anodes and must not come up
 *    before V+ (datasheet: "Power up VDD before VLED"). Prefer 5 V for
 *    VLED: green needs ~3.3 V forward plus driver headroom.
 *  - Pads 2 (address) and 3 (I²C/SPI select) are held high by pull-ups (R1, R2) to 1.8 V on
 *    the tile. Strap them at power-up; never drive pad 2 above 2.1 V.
 *  - INTB (pad 8) is left open-drain, active-low, so the host's own
 *    pull-up sets its level (the pin tolerates 6 V). Its push-pull modes
 *    drive only to 1.8 V, below many 3.3 V MCUs' input-high threshold.
 *  - Emitter-to-detector crosstalk across the tile puts a large DC
 *    pedestal on every channel. tile_sense_hr_auto_gain() cancels it with
 *    the offset DAC; without that step most settings overflow the ADC.
 *
 * Datasheet: MAX86174A/B, Analog Devices, Rev 2 (7/23)
 *
 * @studio tile label=Sense.HR icon=♥
 * @studio event name=frame_ready mask=MAX86174A_STATUS1_FRAME_RDY
 * @studio event name=fifo_almost_full mask=MAX86174A_STATUS1_A_FULL
 * @studio event name=exposure_overflow mask=MAX86174A_STATUS1_EXP_OVF
 * @studio event name=proximity mask=MAX86174A_STATUS1_THRESH1_HILO
 *
 * Driver gaps (chip capabilities not exposed by this driver):
 *
 * @studio unsupported severity=advanced category="SPI bus mode"
 *   Hardware-gated on Sense-HR-a. The MAX86174A supports 4-wire SPI to
 *   24 MHz (pad 3 to GND selects it), but its SDO/MISO output (pad 2) is
 *   push-pull to the 1.8 V VDD (V_OH >= VDD - 0.4 V at 4 mA), below a
 *   3.3 V Core's input-high threshold, and the pad's 2.1 V absolute
 *   maximum (VDD + 0.3 V) forbids a
 *   pull-up to 3.3 V. SPI needs a level translator on MISO that the
 *   tile does not carry. I²C is open-drain and unaffected.
 *
 * @studio unsupported severity=niche category="External frame sync (TRIG input)"
 *   Hardware-gated. SYNC_MODE 1/2 take an external frame trigger or
 *   frame clock on the LED4_DRV/TRIG pin. On Sense-HR-a that pin drives
 *   the second green LED, so the driver pins TRIGLED4_SEL = 1 and runs
 *   the internal 32.768 kHz frame oscillator only.
 *
 * @studio unsupported severity=advanced category="Dual-LED exposures (driver B)"
 *   Driver-deferred. Each measurement slot can pulse two LEDs at once
 *   (MEASx_DRVA + MEASx_DRVB) to sum red and IR into one sample. This
 *   driver uses driver A only — one LED per slot — because the SpO₂
 *   ratio needs the wavelengths separated, not summed. Driver B is
 *   reachable through tile_sense_hr_write_reg() on MEASx_DRVB_PA.
 *
 * @studio unsupported severity=advanced category="Raw exposure/ambient capture (COLLECT_RAW_DATA)"
 *   Driver-deferred. Setting COLLECT_RAW_DATA pushes the LED-on and
 *   dark samples to the FIFO separately (DARK tag 0xC) so the host can
 *   run its own ambient rejection. The driver's FIFO decoder assumes
 *   the chip's computed samples; raw mode changes the frame layout and
 *   would need a second decoder.
 *
 * @studio unsupported severity=advanced category="Digital low-pass / IIR filter (DLPF_EN)"
 *   Driver-deferred. The chip can average 2–32 adjacent samples (DEC_AVE)
 *   or run an IIR low-pass (IIR_CFG selects the cut-off; the IIR supports
 *   100 and 200 fps) before the FIFO. Left disabled: the host-side band-pass a heart-rate algorithm
 *   already needs does the same job with fewer constraints.
 *
 * @studio unsupported severity=niche category="Threshold instance 2 and hysteresis"
 *   Driver-deferred. tile_sense_hr_set_threshold() programs THRESHOLD1
 *   only, with no time or level hysteresis. THRESHOLD2 (used for a
 *   "left proximity mode" interrupt) and TIME_HYST / LEVEL_HYST are
 *   reachable through tile_sense_hr_write_reg() on 0x50–0x55.
 *
 * @studio unsupported severity=niche category="FIFO marker, I²C broadcast address, SINC3 / 2nd-order decimation"
 *   Driver-deferred. FIFO_MARK pushes a 0xFFFFFE tag to separate data
 *   across a configuration change; I2C_BCAST_ADDR lets several parts
 *   share one write address; PPG_FILT2_SEL / MEASx_SINC3_SEL swap the
 *   ADC decimation filter. None affect a normal measurement; all are
 *   reachable through tile_sense_hr_write_reg().
 *
 * @note All bus I/O is routed through tiles_pal_t function pointers.
 *       This driver contains no platform-specific code.
 */

#ifndef TILE_SENSE_HR_H
#define TILE_SENSE_HR_H

#include "tiles.h"

/* ---- Driver version ---- */

#define TILE_SENSE_HR_VERSION_MAJOR  1
#define TILE_SENSE_HR_VERSION_MINOR  0
#define TILE_SENSE_HR_VERSION_PATCH  0

TILES_CHECK_VERSION(1, 0);

/* ---- Instance mapping ----
 *
 *  Instance  |  I2C address  |  Pad 2 (ADDR) state
 * -----------|---------------|----------------------------------------
 *     0      |     0x6B      |  Open (tile pull-up, default)
 *     1      |     0x6A      |  Connected to GND
 *
 * Datasheet Table 7 gives 8-bit write/read pairs 0xD6/0xD7 (ADDR = 1)
 * and 0xD4/0xD5 (ADDR = 0); tiles_pal takes the 7-bit form.
 */

/* ---- I2C addresses ---- */

#define MAX86174A_I2C_ADDR_DEFAULT  0x6B  /**< ADDR high (pad 2 open, default) */
#define MAX86174A_I2C_ADDR_ALT      0x6A  /**< ADDR low (pad 2 to GND) */

/* ---- Register map ---- */

#define MAX86174A_REG_STATUS1        0x00
#define MAX86174A_REG_STATUS2        0x01
#define MAX86174A_REG_FIFO_WR_PTR    0x03
#define MAX86174A_REG_FIFO_RD_PTR    0x04
#define MAX86174A_REG_FIFO_CNT1      0x05  /**< [7] DATA_COUNT[8], [6:0] OVF_COUNTER */
#define MAX86174A_REG_FIFO_CNT2      0x06  /**< DATA_COUNT[7:0] */
#define MAX86174A_REG_FIFO_DATA      0x07  /**< Burst-read only, 3 bytes per sample */
#define MAX86174A_REG_FIFO_CFG1      0x08  /**< FIFO_A_FULL: free slots before A_FULL */
#define MAX86174A_REG_FIFO_CFG2      0x09
#define MAX86174A_REG_SYS_SYNC       0x10
#define MAX86174A_REG_SYS_CFG1       0x11
#define MAX86174A_REG_SYS_CFG2       0x12  /**< [5:0] MEAS6_EN..MEAS1_EN */
#define MAX86174A_REG_SYS_CFG3       0x13
#define MAX86174A_REG_PPG_FILTER     0x14
#define MAX86174A_REG_PD_BIAS        0x15
#define MAX86174A_REG_PIN_FUNC       0x16
#define MAX86174A_REG_PIN_OUT        0x17
#define MAX86174A_REG_I2C_BCAST      0x18
#define MAX86174A_REG_FR_CLK_FINE    0x1A
#define MAX86174A_REG_FR_CLK_DIV_H   0x1B  /**< FR_CLK_DIV[14:8] */
#define MAX86174A_REG_FR_CLK_DIV_L   0x1C  /**< FR_CLK_DIV[7:0] */
#define MAX86174A_REG_MEAS1_BASE     0x20  /**< MEASx block = 0x20 + 8·x, x = 0..5 */
#define MAX86174A_REG_THRESH_SEL     0x50
#define MAX86174A_REG_THRESH_HYST    0x51
#define MAX86174A_REG_THRESH1_HI     0x52
#define MAX86174A_REG_THRESH1_LO     0x53
#define MAX86174A_REG_THRESH2_HI     0x54
#define MAX86174A_REG_THRESH2_LO     0x55
#define MAX86174A_REG_INT_EN1        0x58
#define MAX86174A_REG_INT_EN2        0x59
#define MAX86174A_REG_REV_ID         0xFE
#define MAX86174A_REG_PART_ID        0xFF

/* MEASx block offsets (add to 0x20 + 8·slot) */
#define MAX86174A_MEAS_SEL           0     /**< [6] AMB, [3:2] DRVB, [1:0] DRVA */
#define MAX86174A_MEAS_CFG1          1     /**< [7:6] PDSEL, [4:3] TINT, [2:0] AVER */
#define MAX86174A_MEAS_CFG2          2     /**< [5:4] LED_RGE, [3:2] PPG2_ADC_RGE, [1:0] PPG1_ADC_RGE */
#define MAX86174A_MEAS_CFG3          3     /**< [7:5] PD_SETLNG, [4:3] LED_SETLNG */
#define MAX86174A_MEAS_CFG4          4     /**< [6:4] PPG2_DACOFF, [2:0] PPG1_DACOFF */
#define MAX86174A_MEAS_DRVA_PA       5     /**< LED driver A current code */
#define MAX86174A_MEAS_DRVB_PA       6     /**< LED driver B current code */

/* ---- Device ID ---- */

#define MAX86174A_PART_ID_VALUE      0x40  /**< MAX86174A (dual channel); 0x41 = MAX86174B */

/* ---- STATUS1 bit masks (also the event bitmask handed to on_event) ---- */

#define MAX86174A_STATUS1_A_FULL       (1 << 7)  /**< FIFO reached the almost-full watermark */
#define MAX86174A_STATUS1_FRAME_RDY    (1 << 6)  /**< A complete frame is in the FIFO */
#define MAX86174A_STATUS1_FIFO_DATA_RDY (1 << 5) /**< New sample in the FIFO */
#define MAX86174A_STATUS1_ALC_OVF      (1 << 4)  /**< Ambient-light cancellation saturated */
#define MAX86174A_STATUS1_EXP_OVF      (1 << 3)  /**< Exposure over/under range on some slot */
#define MAX86174A_STATUS1_THRESH2_HILO (1 << 2)  /**< Threshold instance 2 crossed */
#define MAX86174A_STATUS1_THRESH1_HILO (1 << 1)  /**< Threshold instance 1 crossed (proximity) */
#define MAX86174A_STATUS1_PWR_RDY      (1 << 0)  /**< VDD dipped below UVLO since last read */

/* ---- STATUS2 bit masks ---- */

#define MAX86174A_STATUS2_INVALID_CFG  (1 << 7)  /**< Frame rate too fast for the enabled slots */
#define MAX86174A_STATUS2_LED4_COMPB   (1 << 3)  /**< LED4_DRV below compliance voltage */
#define MAX86174A_STATUS2_LED3_COMPB   (1 << 2)  /**< LED3_DRV below compliance voltage */
#define MAX86174A_STATUS2_LED2_COMPB   (1 << 1)  /**< LED2_DRV below compliance voltage */
#define MAX86174A_STATUS2_LED1_COMPB   (1 << 0)  /**< LED1_DRV below compliance voltage */

/* ---- SYS_CFG1 bit masks ---- */

#define MAX86174A_CFG1_DISABLE_I2C   (1 << 7)
#define MAX86174A_CFG1_PPG2_PWRDN    (1 << 3)
#define MAX86174A_CFG1_PPG1_PWRDN    (1 << 2)
#define MAX86174A_CFG1_SHDN          (1 << 1)
#define MAX86174A_CFG1_RESET         (1 << 0)

/* ---- SYS_CFG3 bit masks ---- */

#define MAX86174A_CFG3_ALC_DISABLE      (1 << 4)
#define MAX86174A_CFG3_PROX_DATA_EN     (1 << 3)
#define MAX86174A_CFG3_PROX_AUTO        (1 << 2)
#define MAX86174A_CFG3_COLLECT_RAW_DATA (1 << 1)
#define MAX86174A_CFG3_MEAS1_CONFIG_SEL (1 << 0)

/* ---- FIFO_CFG2 bit masks ---- */

#define MAX86174A_FIFO_MARK          (1 << 5)
#define MAX86174A_FIFO_FLUSH         (1 << 4)
#define MAX86174A_FIFO_STAT_CLR      (1 << 3)
#define MAX86174A_FIFO_A_FULL_TYPE   (1 << 2)
#define MAX86174A_FIFO_RO            (1 << 1)  /**< Roll over (drop oldest) when full */

/* ---- FIFO sample encoding ---- */

#define MAX86174A_FIFO_TAG_SHIFT     20
#define MAX86174A_FIFO_DATA_MASK     0x000FFFFFul
#define MAX86174A_FIFO_TAG_DARK      0xC   /**< Dark sample (COLLECT_RAW_DATA only) */
#define MAX86174A_FIFO_TAG_ALC_OVF   0xD   /**< ALC overflow replaced this sample */
#define MAX86174A_FIFO_TAG_EXP_OVF   0xE   /**< Exposure overflow replaced this sample */
#define MAX86174A_FIFO_MARKER        0xFFFFEul  /**< 20-bit marker payload */
#define MAX86174A_FIFO_INVALID       0xFFFFFul  /**< Read from an empty FIFO */

/* ---- Constants ---- */

#define SENSE_HR_SLOTS               6     /**< Measurement slots per frame (MEAS1..6) */
#define SENSE_HR_CHANNELS            2     /**< Readout channels (PPG1, PPG2) */
#define SENSE_HR_FRAME_LEN           (SENSE_HR_SLOTS * SENSE_HR_CHANNELS)
#define SENSE_HR_FIFO_DEPTH          256
#define SENSE_HR_ADC_FULL_SCALE      524287L  /**< 20-bit two's complement, positive limit */
#define SENSE_HR_FR_CLK_HZ           32768u   /**< Internal frame-rate oscillator */
#define SENSE_HR_FR_DIV_MIN          16
#define SENSE_HR_FR_DIV_MAX          32766
#define SENSE_HR_LED_CHIP_MAX_UA     127500u  /**< The chip's ceiling: 0xFF at the 128 mA range */
#define SENSE_HR_LED_MAX_UA          100000u  /**< The driver's cap: the IN-S42AT emitters' pulse rating (100 mA, <=1/10 duty, <=0.1 ms) */
#define SENSE_HR_DACOFF_MAX_UA       28
#define SENSE_HR_NO_SAMPLE           ((int32_t)0x80000000)  /**< Frame entry with no valid sample */

/* ---- Enumerations ---- */

/**
 * Emitter selection — which LEDn_DRV pin a slot pulses (MEASx_DRVA[1:0]).
 *
 * Mapping measured on Sense-HR-a hardware (2026-08-10) by driving each pin
 * in turn. It does NOT match the BOM, which lists LED2 as red and LED3 as
 * green; on the board those two are swapped.
 */
typedef enum {
    SENSE_HR_LED_IR     = 0x0,  /**< LED1_DRV — 940 nm infrared */
    SENSE_HR_LED_GREEN  = 0x1,  /**< LED2_DRV — 525 nm green (primary) */
    SENSE_HR_LED_RED    = 0x2,  /**< LED3_DRV — 645 nm red (IN-S42ATR, λP) */
    SENSE_HR_LED_GREEN2 = 0x3,  /**< LED4_DRV — 525 nm green (second) */
} sense_hr_led_t;

/** ADC integration time per pulse (MEASx_TINT[1:0], 3rd-order decimation). */
typedef enum {
    SENSE_HR_TINT_14US  = 0x0,  /**< 14.6 µs */
    SENSE_HR_TINT_29US  = 0x1,  /**< 29.2 µs */
    SENSE_HR_TINT_58US  = 0x2,  /**< 58.6 µs */
    SENSE_HR_TINT_117US = 0x3,  /**< 117.1 µs (default, most signal per exposure) */
} sense_hr_tint_t;

/** LED pulses averaged per sample (MEASx_AVER[2:0]). */
typedef enum {
    SENSE_HR_AVER_1   = 0x0,  /**< 1 pulse */
    SENSE_HR_AVER_2   = 0x1,  /**< 2 pulses */
    SENSE_HR_AVER_4   = 0x2,  /**< 4 pulses (default) */
    SENSE_HR_AVER_8   = 0x3,  /**< 8 pulses */
    SENSE_HR_AVER_16  = 0x4,  /**< 16 pulses */
    SENSE_HR_AVER_32  = 0x5,  /**< 32 pulses */
    SENSE_HR_AVER_64  = 0x6,  /**< 64 pulses */
    SENSE_HR_AVER_128 = 0x7,  /**< 128 pulses */
} sense_hr_aver_t;

/** PPG ADC full-scale photocurrent (MEASx_PPGy_ADC_RGE[1:0]). */
typedef enum {
    SENSE_HR_ADC_4UA  = 0x0,  /**< 4 µA full scale, 7.6 pA/LSB */
    SENSE_HR_ADC_8UA  = 0x1,  /**< 8 µA full scale, 15.3 pA/LSB */
    SENSE_HR_ADC_16UA = 0x2,  /**< 16 µA full scale, 30.5 pA/LSB */
    SENSE_HR_ADC_32UA = 0x3,  /**< 32 µA full scale, 61 pA/LSB (default) */
} sense_hr_adc_range_t;

/** Photodiode routing to the two readout channels (MEASx_PDSEL[1:0]). */
typedef enum {
    SENSE_HR_PD_BOTH_PPG1 = 0x0,  /**< PD1 + PD2 summed onto PPG1 (default, best SNR) */
    SENSE_HR_PD_2_1       = 0x1,  /**< PD2 → PPG1, PD1 → PPG2 */
    SENSE_HR_PD_1_2       = 0x2,  /**< PD1 → PPG1, PD2 → PPG2 (per-diode readout) */
    SENSE_HR_PD_BOTH_PPG2 = 0x3,  /**< PD1 + PD2 summed onto PPG2 */
} sense_hr_pdsel_t;

/* ---- Event callback ---- */

/**
 * Event callback fired by tile_sense_hr_process() when STATUS1 reports
 * activity. `events` is the raw STATUS1 byte (MAX86174A_STATUS1_* masks).
 *
 * Always runs in main-loop context (never ISR). I2C is safe to call.
 */
typedef void (*sense_hr_event_cb_t)(tile_t *tile, uint8_t events, void *ctx);

/* ---- Configuration struct ---- */

/**
 * @brief Optional init-time configuration for Sense.HR.
 *
 * Pass NULL to tile_sense_hr_init() for defaults: 128 fps, slot 0 = green
 * at 1 mA, 117 µs integration, 4 pulses averaged, 32 µA ADC range, both
 * photodiodes onto PPG1, PPG2 powered down, polled (no INTB wiring).
 * Zero fields fall back to those defaults, so the zero-valued choices
 * (LED1 emitter, 14.6 µs, 1 pulse, the 4 µA range) are set after init
 * with set_slot(), set_integration(), set_averaging(), set_adc_range().
 */
typedef struct {
    uint16_t fps;          /**< Frame rate, 1–2048 fps. Default 128. */
    uint8_t  led;          /**< Slot-0 emitter (sense_hr_led_t). Default green. */
    uint32_t led_ua;       /**< Slot-0 LED current in µA. Default 1000. */
    uint8_t  tint;         /**< Integration time (sense_hr_tint_t). 0 = default 117 µs. */
    uint8_t  aver;         /**< Pulses averaged (sense_hr_aver_t). 0 = default 4. */
    uint8_t  adc_range;    /**< ADC range (sense_hr_adc_range_t). 0 = default 32 µA. */
    uint8_t  pdsel;        /**< Photodiode routing (sense_hr_pdsel_t). Default both → PPG1. */
    uint8_t  ppg2_enable;  /**< 1 = keep readout channel 2 powered. Default 0 (off). */
    uint8_t  int_pin;      /**< Core pad wired to INTB (pad 8). 0 = polled. */
    sense_hr_event_cb_t on_event;  /**< Callback fired by process(). NULL = none. */
    void    *event_ctx;    /**< User context passed to on_event. */
} sense_hr_cfg_t;

/* ---- Lifecycle ---- */

/**
 * @brief  Check if a Sense.HR is present on the bus.
 * @param  hal      Platform HAL handle.
 * @param  instance 0 = default address (0x6B), 1 = alternate (0x6A).
 * @return 1 if the device ACKs and PART_ID reads 0x40, 0 otherwise.
 */
uint8_t tile_sense_hr_find(tiles_pal_t *hal, uint8_t instance);

/**
 * @brief  Initialise a Sense.HR tile.
 *
 * Probes the device, verifies PART_ID, soft-resets, pins the LED4/TRIG
 * pin to LED duty, configures the frame rate and slot 0, leaves INTB
 * open-drain, and sets the tile state to READY. Measurements are NOT
 * running until tile_sense_hr_start().
 *
 * @param  hal      Platform HAL handle.
 * @param  instance 0 = default (0x6B), 1 = alternate (0x6A).
 * @param  tile     Tile handle to initialise.
 * @param  cfg      Configuration (NULL for defaults).
 */
void tile_sense_hr_init(tiles_pal_t *hal, uint8_t instance,
                        tile_t *tile, const sense_hr_cfg_t *cfg);

/**
 * @brief  Enter shutdown (SHDN = 1): oscillator off, LEDs off, ~1 µA
 *         (3 µA max on VDD, 0.5 µA max on VLED).
 *
 * Registers are retained; tile_sense_hr_wake() resumes without
 * reconfiguration.
 *
 * @studio expose category=tile name=sleep section=lifecycle
 */
void tile_sense_hr_sleep(tile_t *tile);

/**
 * @brief  Leave shutdown and resume the configured measurements.
 * @studio expose category=tile name=wake section=lifecycle
 */
void tile_sense_hr_wake(tile_t *tile);

/**
 * @brief  Software-reset the device to power-on defaults.
 *
 * All registers return to defaults and the tile state drops to NONE.
 * Call tile_sense_hr_init() again afterwards.
 *
 * @studio expose category=tile name=reset section=lifecycle
 */
void tile_sense_hr_reset(tile_t *tile);

/**
 * @brief  Start the frame engine on every enabled slot.
 *
 * Flushes the FIFO first so the first frame read is fresh.
 *
 * @studio expose category=tile name=start section=lifecycle
 */
void tile_sense_hr_start(tile_t *tile);

/**
 * @brief  Stop all measurements (LEDs off, FIFO stops filling).
 * @studio expose category=tile name=stop section=lifecycle
 */
void tile_sense_hr_stop(tile_t *tile);

/* ---- Event processing ---- */

/**
 * @brief  Process pending interrupt events. Call from your main loop.
 *
 * In interrupt mode (int_pin set): returns immediately unless INTB has
 * fired since the last call, then reads STATUS1 and fires the callback.
 * In polled mode: reads STATUS1 every call. Reading STATUS1 clears its
 * latched bits and releases INTB.
 *
 * @studio expose category=tile name=process section=events
 */
void tile_sense_hr_process(tile_t *tile);

/**
 * @brief  Register or change the event callback.
 * @studio expose category=tile name=on_event section=events
 * @param  cb    Callback to fire on STATUS1 activity. NULL to clear.
 * @param  ctx   Opaque user pointer passed to the callback.
 */
void tile_sense_hr_on_event(tile_t *tile, sense_hr_event_cb_t cb, void *ctx);

/**
 * @brief  Choose which STATUS1 sources drive INTB (Interrupt Enable 1).
 *
 * Pass a mask of MAX86174A_STATUS1_* bits, e.g. FRAME_RDY | A_FULL.
 * PWR_RDY is non-maskable. INTB stays open-drain, active-low.
 *
 * @studio expose category=tile name=set_interrupts section=events
 * @param  mask  STATUS1 bits that should assert INTB.
 */
void tile_sense_hr_set_interrupts(tile_t *tile, uint8_t mask);

/* ---- Configuration ---- */

/**
 * @brief  Set the frame rate.
 *
 * Programs FR_CLK_DIV = 32768 / fps, clamped to the chip's 16–32766
 * divider range (1–2048 fps). If the enabled slots' exposures do not fit
 * in one frame period the chip flags INVALID_CFG in STATUS2.
 *
 * @studio expose category=tile name=set_frame_rate returns=int section=config
 * @param  fps   Requested frames per second.
 * @return Actual frame rate after divider rounding.
 */
uint16_t tile_sense_hr_set_frame_rate(tile_t *tile, uint16_t fps);

/**
 * @brief  Read the configured frame rate.
 * @studio expose category=tile name=get_frame_rate returns=int section=config
 * @return Frames per second after divider rounding.
 */
uint16_t tile_sense_hr_get_frame_rate(tile_t *tile);

/**
 * @brief  Configure and enable a measurement slot with one emitter.
 *
 * Writes the slot's emitter select and LED current, inherits the
 * tile-wide integration / averaging / ADC range / photodiode routing,
 * and enables the slot in SYS_CFG2. Takes effect on the next start()
 * (or immediately if already running). Slot 0 is the one init() set up.
 *
 * @studio expose category=tile name=set_slot section=config
 * @param  slot    Measurement slot 0–5 (MEAS1..MEAS6).
 * @param  led     Emitter to pulse (sense_hr_led_t).
 * @param  led_ua  LED current in µA, 0–100000 (0 disables the LED; above 100 mA is capped).
 */
void tile_sense_hr_set_slot(tile_t *tile, uint8_t slot, sense_hr_led_t led,
                            uint32_t led_ua);

/**
 * @brief  Configure a slot as a direct ambient-light measurement (no LED).
 *
 * The chip requires an ambient slot to be the LAST enabled slot in the
 * frame. Its sample carries the same tag position as an LED slot.
 *
 * @studio expose category=tile name=set_ambient_slot section=config
 * @param  slot  Measurement slot 0–5.
 */
void tile_sense_hr_set_ambient_slot(tile_t *tile, uint8_t slot);

/**
 * @brief  Enable or disable a slot without changing its settings.
 * @studio expose category=tile name=enable_slot section=config
 * @param  slot    Measurement slot 0–5.
 * @param  enable  1 = included in each frame, 0 = skipped.
 */
void tile_sense_hr_enable_slot(tile_t *tile, uint8_t slot, uint8_t enable);

/**
 * @brief  Set a slot's LED drive current.
 *
 * Picks the smallest LED range (32 / 64 / 96 / 128 mA full scale) that
 * covers the request, then the nearest 8-bit code: 125 / 250 / 375 /
 * 500 µA per step. 0 disables the LED for that slot.
 *
 * @studio expose category=tile name=set_led_current section=config
 * @param  slot    Measurement slot 0–5.
 * @param  led_ua  LED current in µA, 0–100000 (above 100 mA is capped).
 */
void tile_sense_hr_set_led_current(tile_t *tile, uint8_t slot, uint32_t led_ua);

/**
 * @brief  Read back a slot's LED drive current after quantisation.
 * @studio expose category=tile name=get_led_current returns=int section=config
 * @param  slot  Measurement slot 0–5.
 * @return LED current in µA as programmed (0 = LED off).
 */
uint32_t tile_sense_hr_get_led_current(tile_t *tile, uint8_t slot);

/**
 * @brief  Set a slot's ADC integration time.
 *
 * Longer integration collects more photocurrent per pulse (more signal,
 * but easier to overflow on crosstalk or ambient light).
 *
 * @studio expose category=tile name=set_integration section=config
 * @param  slot  Measurement slot 0–5.
 * @param  tint  Integration time (sense_hr_tint_t).
 */
void tile_sense_hr_set_integration(tile_t *tile, uint8_t slot, sense_hr_tint_t tint);

/**
 * @brief  Set how many LED pulses a slot averages into one sample.
 *
 * Averaging improves SNR and ambient rejection at the cost of frame
 * time: each pulse costs one integration period plus settling.
 *
 * @studio expose category=tile name=set_averaging section=config
 * @param  slot  Measurement slot 0–5.
 * @param  aver  Pulses per sample (sense_hr_aver_t).
 */
void tile_sense_hr_set_averaging(tile_t *tile, uint8_t slot, sense_hr_aver_t aver);

/**
 * @brief  Set a slot's ADC full-scale photocurrent range (both channels).
 * @studio expose category=tile name=set_adc_range section=config
 * @param  slot   Measurement slot 0–5.
 * @param  range  Full-scale range (sense_hr_adc_range_t).
 */
void tile_sense_hr_set_adc_range(tile_t *tile, uint8_t slot, sense_hr_adc_range_t range);

/**
 * @brief  Set a slot's offset-DAC cancellation current (both channels).
 *
 * The DAC sources part of the photodiode DC away from the converter so
 * the ADC digitises only what is left. The reported sample is
 * (photocurrent − offset); tile_sense_hr_read_photocurrent_na() adds the
 * offset back. Quantised to 0, 4, 8 … 28 µA.
 *
 * @studio expose category=tile name=set_dac_offset section=config
 * @param  slot       Measurement slot 0–5.
 * @param  offset_ua  Cancellation current in µA, 0–28.
 */
void tile_sense_hr_set_dac_offset(tile_t *tile, uint8_t slot, uint8_t offset_ua);

/**
 * @brief  Route the two photodiodes to the readout channels for a slot.
 *
 * Summing both onto PPG1 (default) gives the best single-wavelength
 * SNR. Splitting them (SENSE_HR_PD_1_2) reads each diode on its own
 * channel; PPG2 must then be powered (cfg.ppg2_enable) to see PD2.
 *
 * @studio expose category=tile name=set_photodiodes section=config
 * @param  slot   Measurement slot 0–5.
 * @param  pdsel  Routing (sense_hr_pdsel_t).
 */
void tile_sense_hr_set_photodiodes(tile_t *tile, uint8_t slot, sense_hr_pdsel_t pdsel);

/**
 * @brief  Power readout channel 2 up or down.
 *
 * With both photodiodes summed onto PPG1, channel 2 still emits a
 * near-zero sample per slot per frame; powering it down halves FIFO
 * traffic. Enable it for per-diode readout.
 *
 * @studio expose category=tile name=set_ppg2_enabled section=config
 * @param  enable  1 = channel 2 powered, 0 = powered down.
 */
void tile_sense_hr_set_ppg2_enabled(tile_t *tile, uint8_t enable);

/**
 * @brief  Find LED current, offset and ADC range for a slot automatically.
 *
 * Blocking, roughly two to three seconds. Runs the slot alone, then:
 *  1. raises LED current toward `max_ua` while the offset DAC can still
 *     keep the DC pedestal (crosstalk + tissue return) inside the ADC,
 *     backing off current when it cannot;
 *  2. measures the total photocurrent and picks the largest offset that
 *     leaves ~0.5 µA of headroom, then the tightest ADC range holding
 *     the residual under ~70 % of full scale;
 *  3. verifies, and unwinds one step at a time if that overflowed.
 *
 * Other slots' enables are restored on return; measurements are left
 * stopped. Call tile_sense_hr_start() afterwards. Re-run whenever the
 * optical situation changes (finger placed or removed).
 *
 * @studio expose category=tile name=auto_gain returns=bool section=runtime
 * @param  slot    Measurement slot 0–5 (must already have an emitter).
 * @param  max_ua  Upper bound for LED current in µA (0 = 10000, a safe default for the 0402 emitters). The search runs in the 32 mA range, so anything above 31875 is capped there.
 * @return 1 if a valid operating point was found, 0 if the slot never produced samples.
 */
uint8_t tile_sense_hr_auto_gain(tile_t *tile, uint8_t slot, uint32_t max_ua);

/* ---- Runtime ---- */

/**
 * @brief  Read STATUS1 (clears its latched bits and releases INTB).
 * @studio expose category=tile name=get_status returns=int section=runtime
 * @return Raw STATUS1 byte (use MAX86174A_STATUS1_* masks).
 */
uint8_t tile_sense_hr_get_status(tile_t *tile);

/**
 * @brief  Read STATUS2: invalid-configuration and LED compliance flags.
 *
 * LEDn_COMPB set means VLED is too low for the programmed current on
 * that pin — raise VLED or lower the current.
 *
 * @studio expose category=tile name=get_status2 returns=int section=runtime
 * @return Raw STATUS2 byte (use MAX86174A_STATUS2_* masks).
 */
uint8_t tile_sense_hr_get_status2(tile_t *tile);

/**
 * @brief  Check whether at least one full frame is waiting in the FIFO.
 *
 * Non-clearing: compares the FIFO count against the frame length, so it
 * is safe to poll without disturbing STATUS1 or INTB.
 *
 * @studio expose category=tile name=frame_ready returns=bool section=runtime
 * @return 1 if a frame can be read, 0 otherwise.
 */
uint8_t tile_sense_hr_frame_ready(tile_t *tile);

/**
 * @brief  Number of unread samples in the FIFO (0–256).
 * @studio expose category=tile name=get_fifo_count returns=int section=fifo
 * @return Samples waiting.
 */
uint16_t tile_sense_hr_get_fifo_count(tile_t *tile);

/**
 * @brief  Samples lost to FIFO overflow since the last FIFO read (0–127).
 * @studio expose category=tile name=get_fifo_overflow returns=int section=fifo
 * @return OVF_COUNTER value; saturates at 127.
 */
uint8_t tile_sense_hr_get_fifo_overflow(tile_t *tile);

/**
 * @brief  Discard everything in the FIFO and reset its pointers.
 * @studio expose category=tile name=flush_fifo section=fifo
 */
void tile_sense_hr_flush_fifo(tile_t *tile);

/**
 * @brief  Set the almost-full watermark used by the A_FULL flag / event.
 * @studio expose category=tile name=set_fifo_watermark section=fifo
 * @param  samples  Number of queued samples that asserts A_FULL, 1–256.
 */
void tile_sense_hr_set_fifo_watermark(tile_t *tile, uint16_t samples);

/**
 * @brief  Drain the FIFO and decode the most recent complete frame.
 *
 * Reads every waiting sample, sign-extends the 20-bit two's-complement
 * data, and places it by tag: `frame[2·slot + channel]` (channel 0 =
 * PPG1, 1 = PPG2). Entries for disabled slots, powered-down channels,
 * and samples replaced by an overflow tag read SENSE_HR_NO_SAMPLE.
 * Overflow-tagged samples are counted (see get_overflow_count()).
 * Each entry holds the newest sample for its tag, so if the FIFO ends
 * part-way through a frame the result can mix two adjacent frames.
 *
 * The decoded frame is also cached for tile_sense_hr_read_ppg().
 *
 * @studio expose category=tile name=read_frame returns=bool section=runtime
 * @studio out_buffer frame type=int32_t length=12
 * @param  frame  Caller buffer of SENSE_HR_FRAME_LEN (12) entries.
 * @return 1 if a complete frame was decoded, 0 if the FIFO held less than one frame (buffer then holds the previous frame).
 */
uint8_t tile_sense_hr_read_frame(tile_t *tile, int32_t *frame);

/**
 * @brief  Latest PPG1 sample for a slot, in ADC counts.
 *
 * Drains the FIFO if a full frame is waiting (same as read_frame()),
 * then returns the cached channel-1 value for the slot. Negative values
 * are real: the offset DAC can pull the residual below zero.
 *
 * @studio expose category=tile name=read_ppg returns=int section=runtime
 * @param  slot  Measurement slot 0–5.
 * @return Signed 20-bit sample, or SENSE_HR_NO_SAMPLE if none yet.
 */
int32_t tile_sense_hr_read_ppg(tile_t *tile, uint8_t slot);

/**
 * @brief  Latest PPG2 sample for a slot, in ADC counts.
 *
 * Only meaningful when channel 2 is powered and a photodiode is routed
 * to it (SENSE_HR_PD_1_2 or SENSE_HR_PD_2_1).
 *
 * @studio expose category=tile name=read_ppg2 returns=int section=runtime
 * @param  slot  Measurement slot 0–5.
 * @return Signed 20-bit sample, or SENSE_HR_NO_SAMPLE if none yet.
 */
int32_t tile_sense_hr_read_ppg2(tile_t *tile, uint8_t slot);

/**
 * @brief  Latest PPG1 photocurrent for a slot, in nanoamps.
 *
 * Converts the cached sample with the slot's ADC range (full scale /
 * 524287 counts) and adds the offset-DAC current back, so the result is
 * the photodiode current itself and is comparable across slots that
 * ended up with different gain settings. Integer maths, no float.
 *
 * @studio expose category=tile name=read_photocurrent_na returns=int section=runtime
 * @param  slot  Measurement slot 0–5.
 * @return Photocurrent in nA, or SENSE_HR_NO_SAMPLE if no sample yet.
 */
int32_t tile_sense_hr_read_photocurrent_na(tile_t *tile, uint8_t slot);

/**
 * @brief  Overflow-tagged samples seen in the last FIFO drain.
 *
 * An overflowing channel produces no data — the chip replaces the
 * sample with an ALC- or exposure-overflow tag. A non-zero count means
 * lower the LED current, raise the offset DAC, or widen the ADC range.
 *
 * @studio expose category=tile name=get_overflow_count returns=int section=runtime
 * @return Number of overflow-tagged samples in the last read.
 */
uint16_t tile_sense_hr_get_overflow_count(tile_t *tile);

/**
 * @brief  Read raw 24-bit FIFO words (tag in bits 23:20, data in 19:0).
 *
 * For hosts that want to decode the stream themselves. Data is NOT
 * sign-extended here; bit 19 is the sign.
 *
 * @studio expose category=tile name=read_fifo_raw returns=int section=fifo
 * @studio out_buffer buf type=uint32_t cap_param=count
 * @param  buf    Caller buffer for raw words.
 * @param  count  Capacity of `buf`; at most this many samples are read.
 * @return Number of words actually read.
 */
uint16_t tile_sense_hr_read_fifo_raw(tile_t *tile, uint32_t *buf, uint16_t count);

/* ---- Proximity / threshold ---- */

/**
 * @brief  Program threshold instance 1 on a slot's PPG1 data.
 *
 * THRESH1_HILO is set (and INTB asserted if enabled) when the sample
 * exceeds `upper` or drops below `lower`. Both limits are in units of
 * 2048 ADC counts (0–255). Upper = 255 disables the upper check; lower
 * = 0 disables the lower check. Negative samples clip to 0 first.
 * (The datasheet's register table says upper 0x00 disables it, its
 * functional description 0xFF; the driver follows the description.)
 *
 * @studio expose category=tile name=set_threshold section=advanced
 * @param  slot   Measurement slot 0–5 to watch.
 * @param  lower  Lower limit ÷ 2048.
 * @param  upper  Upper limit ÷ 2048.
 */
void tile_sense_hr_set_threshold(tile_t *tile, uint8_t slot, uint8_t lower, uint8_t upper);

/**
 * @brief  Enable automatic proximity mode.
 *
 * Reserves slot 5 (MEAS6) as a low-power skin detector: while its PPG1
 * reading stays below `lower` × 2048 counts the chip runs slot 5 alone
 * at 8 fps; once skin is detected it returns to the programmed frame
 * rate with every enabled slot. Configure slot 5's emitter and a small
 * LED current first (tile_sense_hr_set_slot()). THRESH1_HILO fires on
 * entering proximity mode.
 *
 * @studio expose category=tile name=set_proximity_mode section=config
 * @param  enable  1 = automatic proximity mode, 0 = normal.
 * @param  lower   Skin-detect threshold ÷ 2048 ADC counts.
 */
void tile_sense_hr_set_proximity_mode(tile_t *tile, uint8_t enable, uint8_t lower);

/* ---- Advanced ---- */

/**
 * @brief  Read any register directly.
 * @studio expose category=tile name=read_reg returns=int section=advanced
 * @param  reg   Register address 0x00–0xFF.
 * @return Register value.
 */
uint8_t tile_sense_hr_read_reg(tile_t *tile, uint8_t reg);

/**
 * @brief  Write any register directly.
 *
 * Bypasses the driver's cached configuration; prefer the typed setters.
 *
 * @studio expose category=tile name=write_reg section=advanced
 * @param  reg   Register address 0x00–0xFF.
 * @param  val   Value to write.
 */
void tile_sense_hr_write_reg(tile_t *tile, uint8_t reg, uint8_t val);

#endif /* TILE_SENSE_HR_H */
