/**
 * @file   tile_sense_mic.c
 * @brief  Sense.MIC (CMM-2718AT-38164W mic + AD8605 amp + MAX11645 ADC) —
 *         complete driver implementation.
 *
 * Platform-agnostic. All bus access via tile->hal raw I2C function pointers.
 * The MAX11645 is a command-based device (no register addresses); we use
 * i2c_write_raw / i2c_read_raw exclusively.
 *
 * Reference: Maxim MAX11644/MAX11645 datasheet, 19-4544; Rev 3, 9/09
 */

#include "tile_sense_mic.h"
#include <stddef.h>

/* ================================================================
 * Instance → I2C address table
 * ================================================================ */

static const uint8_t id_table[] = {
    MAX11645_I2C_ADDR,   /* 0: 0x36 (fixed address — only instance) */
};

#define NUM_INSTANCES  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    return (instance < NUM_INSTANCES) ? id_table[instance] : 0;
}

/* ================================================================
 * Per-instance driver state
 * ================================================================ */

typedef struct {
    uint8_t  setup_byte;    /* Cached setup byte (Vref, clock, polarity) */
    uint8_t  config_byte;   /* Cached config byte (scan, channel, SGL/DIF) */
    uint8_t  ref_sel;       /* Cached reference selection (sense_mic_ref_t) */
    uint8_t  clock_sel;     /* Cached clock selection (sense_mic_clock_t) */
    uint8_t  polarity_sel;  /* Cached output coding (sense_mic_polarity_t) */
    uint16_t vref_mv;       /* Reference voltage in mV (for conversions) */
    uint16_t dc_offset;     /* Measured DC bias point (auto-calibrated at init) */
} mic_state_t;

static mic_state_t mic_state[NUM_INSTANCES];

static mic_state_t *state_for(tile_t *tile)
{
    for (uint8_t i = 0; i < NUM_INSTANCES; i++)
        if (id_table[i] == tile->id) return &mic_state[i];
    return &mic_state[0];
}

/* ================================================================
 * Private helpers — raw I2C access (no register addresses)
 *
 * The MAX11645 uses a command-based protocol:
 *   Write: [ADDR+W] [command byte]
 *   Read:  [ADDR+R] [data_hi] [data_lo]
 *
 * All writes use i2c_write_raw. All reads use i2c_read_raw.
 * ================================================================ */

static void mic_write_cmd(tile_t *tile, uint8_t cmd)
{
    tile->hal->i2c_write_raw(tile->hal->handle, tile->id, &cmd, 1);
}

/* Internal-reference wake-up (datasheet "Automatic Shutdown"): 10 ms. */
#define MIC_REF_WAKE_MS  10

/* A result that is not a conversion: the bus failed, or the upper nibble
 * was not the four high "empty" bits every result starts with. */
#define MIC_SAMPLE_INVALID  0xFFFF

static uint16_t decode(const uint8_t *b)
{
    /* Each result is 2 bytes: four high empty bits (SDA left high), then
     * D11:D8, then D7:D0 (datasheet "Reading a Conversion"). */
    if ((b[0] & 0xF0) != 0xF0) return MIC_SAMPLE_INVALID;
    return (uint16_t)(((b[0] & 0x0F) << 8) | b[1]);
}

/** One conversion, or MIC_SAMPLE_INVALID. */
static uint16_t mic_read_checked(tile_t *tile)
{
    uint8_t buf[2] = {0, 0};
    if (tile->hal->i2c_read_raw(tile->hal->handle, tile->id, buf, 2) != 0)
        return MIC_SAMPLE_INVALID;
    return decode(buf);
}

/** One conversion; 0 on a failed read, so the public API stays in range. */
static uint16_t mic_read_sample(tile_t *tile)
{
    uint16_t v = mic_read_checked(tile);
    return (v == MIC_SAMPLE_INVALID) ? 0 : v;
}

static uint8_t ref_is_internal(uint8_t ref_sel)
{
    return (ref_sel & 0x04) != 0;   /* SEL2 = 1: the internal reference */
}

static void memzero(void *p, uint8_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) *b++ = 0;
}

/* Build a setup byte from ref + clock + polarity enums */
static uint8_t build_setup(uint8_t ref_sel, uint8_t clock_sel, uint8_t polarity_sel)
{
    uint8_t clk_bits = (clock_sel == SENSE_MIC_CLOCK_EXTERNAL)
                     ? MAX11645_CLK_EXTERNAL : MAX11645_CLK_INTERNAL;
    uint8_t pol_bits = (polarity_sel == SENSE_MIC_POLARITY_BIPOLAR)
                     ? MAX11645_BIP : MAX11645_UNI;
    return MAX11645_SETUP_REG
         | ((uint8_t)ref_sel << 4)
         | clk_bits
         | pol_bits
         | MAX11645_RST_NORESET;
}

/* Build a config byte from scan mode + channel */
static uint8_t build_config(uint8_t scan, uint8_t channel)
{
    return MAX11645_CONFIG_REG
         | ((uint8_t)scan << 5)
         | ((uint8_t)channel << 1)
         | MAX11645_SINGLE_ENDED;
}

/* Determine Vref in mV from ref enum, with user override */
static uint16_t resolve_vref(uint8_t ref_sel, uint16_t user_vref_mv)
{
    if (user_vref_mv) return user_vref_mv;
    switch (ref_sel) {
        case SENSE_MIC_REF_INTERNAL:
        case SENSE_MIC_REF_INTERNAL_BUF:
            return 2048;
        default:
            return 3300;  /* VDD or external — assume 3.3V */
    }
}

/* ================================================================
 * Lifecycle
 * ================================================================ */

/** @brief Check if a Sense.MIC is present on the bus. */
uint8_t tile_sense_mic_find(tiles_pal_t *hal, uint8_t instance)
{
    uint8_t addr = resolve_id(instance);
    if (!addr) return 0;
    return hal->i2c_is_ready(hal->handle, addr) == 0;
}

/** @brief Initialize the MAX11645 ADC. */
void tile_sense_mic_init(tiles_pal_t *hal, uint8_t instance,
                         tile_t *tile, const sense_mic_cfg_t *cfg)
{
    memzero(tile, sizeof(tile_t));
    tile->hal = hal;
    tile->id  = resolve_id(instance);

    if (!tile->id) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_mic: invalid instance");
        return;
    }

    /* Probe bus */
    if (hal->i2c_is_ready(hal->handle, tile->id) != 0) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_mic: device not found at 0x36");
        return;
    }

    /* Initialize per-instance state */
    mic_state_t *s = state_for(tile);
    memzero(s, sizeof(mic_state_t));

    /* Apply config or defaults */
    uint8_t ref_sel  = (cfg && cfg->ref) ? cfg->ref : SENSE_MIC_REF_VDD;
    uint8_t channel  = (cfg && cfg->channel) ? cfg->channel : SENSE_MIC_CH_AIN0;
    /* 0 = default = one conversion of the selected channel. (SCAN_UP is 0
     * in the chip's encoding; pick it with set_scan_mode.) */
    uint8_t scan     = (cfg && cfg->scan) ? cfg->scan : SENSE_MIC_SCAN_SINGLE;
    uint8_t clock    = (cfg) ? cfg->clock : SENSE_MIC_CLOCK_INTERNAL;
    uint8_t polarity = (cfg) ? cfg->polarity : SENSE_MIC_POLARITY_UNIPOLAR;

    s->ref_sel      = ref_sel;
    s->clock_sel    = clock;
    s->polarity_sel = polarity;
    s->setup_byte   = build_setup(ref_sel, clock, polarity);
    s->config_byte  = build_config(scan, channel);
    s->vref_mv      = resolve_vref(ref_sel, cfg ? cfg->vref_mv : 0);

    /* Send setup byte — configures Vref, clock, polarity */
    mic_write_cmd(tile, s->setup_byte);
    hal->delay_ms(ref_is_internal(ref_sel) ? MIC_REF_WAKE_MS : 1);

    /* Send configuration byte — scan mode, channel, single-ended */
    mic_write_cmd(tile, s->config_byte);

    /* Verify the device responds by reading a sample. The MAX11645 has no
     * WHO_AM_I; a real result starts with four high bits. */
    uint16_t test = mic_read_checked(tile);
    if (test == MIC_SAMPLE_INVALID) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_mic: test read failed");
        return;
    }

    /* Auto-calibrate DC offset: average 64 samples of the resting level.
     * About 3.5 ms at 400 kHz I2C. On this board the level is near 0 in
     * the VDD / INTERNAL modes (the amp's bias comes from the undriven REF
     * pin; see the header's signal-chain note). */
    {
        uint32_t sum = 0;
        for (uint8_t i = 0; i < 64; i++)
            sum += mic_read_sample(tile);
        s->dc_offset = (uint16_t)(sum >> 6);
    }

    tile->state = TILE_STATE_READY;
}

/** @brief Enter low-power mode. */
void tile_sense_mic_sleep(tile_t *tile)
{
    /* The ADC already shuts itself down between conversions (0.5 µA) —
     * except an "always on" internal reference, about 330 µA. So sleep
     * selects the VDD reference, which turns the internal one off, and
     * wake() restores the real setup. The mic (175 µA) and the AD8605
     * (1 mA) are powered from V+ and keep running: the tile has no
     * switch for them. */
    mic_state_t *s = state_for(tile);
    mic_write_cmd(tile, build_setup(SENSE_MIC_REF_VDD, s->clock_sel, s->polarity_sel));
    tile->state = TILE_STATE_SLEEPING;
}

/** @brief Wake from sleep, restore previous configuration. */
void tile_sense_mic_wake(tile_t *tile)
{
    mic_state_t *s = state_for(tile);

    /* Re-send setup byte (with RST=1 to preserve config) */
    mic_write_cmd(tile, s->setup_byte);
    tile->hal->delay_ms(ref_is_internal(s->ref_sel) ? MIC_REF_WAKE_MS : 1);

    /* Re-send configuration byte */
    mic_write_cmd(tile, s->config_byte);

    tile->state = TILE_STATE_READY;
}

/** @brief Reset config register to power-on defaults. Must call init() again. */
void tile_sense_mic_reset(tile_t *tile)
{
    /* Setup byte with RST=0 resets the configuration register */
    uint8_t reset_setup = MAX11645_SETUP_REG
                        | MAX11645_SEL_VDD
                        | MAX11645_CLK_INTERNAL
                        | MAX11645_UNI
                        | MAX11645_RST_RESET;
    mic_write_cmd(tile, reset_setup);
    tile->state = TILE_STATE_NONE;
}

/* ================================================================
 * Configuration
 * ================================================================ */

/** @brief Change the reference voltage source. */
void tile_sense_mic_set_reference(tile_t *tile, sense_mic_ref_t ref)
{
    mic_state_t *s = state_for(tile);

    /* Rebuild setup byte with new reference, preserving clock + polarity */
    s->ref_sel    = (uint8_t)ref;
    s->setup_byte = build_setup((uint8_t)ref, s->clock_sel, s->polarity_sel);
    s->vref_mv    = resolve_vref((uint8_t)ref, 0);

    mic_write_cmd(tile, s->setup_byte);
    /* The internal reference takes 10 ms to wake; VDD needs nothing. */
    tile->hal->delay_ms(ref_is_internal(s->ref_sel) ? MIC_REF_WAKE_MS : 1);
}

/** @brief Switch the conversion-clock source. */
void tile_sense_mic_set_clock_mode(tile_t *tile, sense_mic_clock_t clk)
{
    mic_state_t *s = state_for(tile);

    s->clock_sel  = (uint8_t)clk;
    s->setup_byte = build_setup(s->ref_sel, s->clock_sel, s->polarity_sel);

    mic_write_cmd(tile, s->setup_byte);
}

/** @brief Switch the output coding (unipolar vs bipolar). */
void tile_sense_mic_set_polarity(tile_t *tile, sense_mic_polarity_t pol)
{
    mic_state_t *s = state_for(tile);

    s->polarity_sel = (uint8_t)pol;
    s->setup_byte   = build_setup(s->ref_sel, s->clock_sel, s->polarity_sel);

    mic_write_cmd(tile, s->setup_byte);
}

/** @brief Change the active ADC channel. */
void tile_sense_mic_set_channel(tile_t *tile, sense_mic_channel_t ch)
{
    mic_state_t *s = state_for(tile);

    /* Rebuild config byte preserving scan mode */
    uint8_t scan = (s->config_byte >> 5) & 0x03;
    s->config_byte = build_config(scan, (uint8_t)ch);

    mic_write_cmd(tile, s->config_byte);
}

/** @brief Change the scan mode. */
void tile_sense_mic_set_scan_mode(tile_t *tile, sense_mic_scan_t scan)
{
    mic_state_t *s = state_for(tile);

    /* Rebuild config byte preserving channel */
    uint8_t channel = (s->config_byte >> 1) & 0x01;
    s->config_byte = build_config((uint8_t)scan, channel);

    mic_write_cmd(tile, s->config_byte);
}

/** @brief Get the currently configured reference voltage in millivolts. */
uint16_t tile_sense_mic_get_vref_mv(tile_t *tile)
{
    mic_state_t *s = state_for(tile);
    return s->vref_mv;
}

/* ================================================================
 * Data reads
 * ================================================================ */

/** @brief Read a single 12-bit ADC sample (0–4095). */
uint16_t tile_sense_mic_get_raw(tile_t *tile)
{
    return mic_read_sample(tile);
}

/** @brief Read a single sample and convert to millivolts. */
uint16_t tile_sense_mic_get_raw_mv(tile_t *tile)
{
    mic_state_t *s = state_for(tile);
    uint16_t raw = mic_read_sample(tile);
    /* mv = raw * vref_mv / 4096 — use 32-bit intermediate to avoid overflow */
    return (uint16_t)(((uint32_t)raw * s->vref_mv) >> 12);
}

/** @brief Read a single AC-coupled audio sample (signed, relative to DC offset). */
int16_t tile_sense_mic_get_audio_sample(tile_t *tile)
{
    mic_state_t *s = state_for(tile);
    uint16_t raw = mic_read_sample(tile);
    return (int16_t)raw - (int16_t)s->dc_offset;
}

/** @brief Get the auto-calibrated DC offset. */
uint16_t tile_sense_mic_get_dc_offset(tile_t *tile)
{
    mic_state_t *s = state_for(tile);
    return s->dc_offset;
}

/** @brief Re-calibrate the DC offset by averaging 64 samples. */
void tile_sense_mic_calibrate(tile_t *tile)
{
    mic_state_t *s = state_for(tile);
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 64; i++)
        sum += mic_read_sample(tile);
    s->dc_offset = (uint16_t)(sum >> 6);
}

/** @brief Burst-read N samples into a buffer. */
void tile_sense_mic_get_samples(tile_t *tile, uint16_t *buf, uint16_t count)
{
    mic_state_t *s = state_for(tile);
    uint16_t i = 0;

    /* In 8x scan mode one read transaction returns eight conversions,
     * saving seven address phases per eight samples. */
    if (((s->config_byte >> 5) & 0x03) == SENSE_MIC_SCAN_8X) {
        uint8_t b[16];
        while (count - i >= 8) {
            if (tile->hal->i2c_read_raw(tile->hal->handle, tile->id, b, 16) != 0) {
                for (uint8_t k = 0; k < 8; k++) buf[i + k] = 0;
            } else {
                for (uint8_t k = 0; k < 8; k++) {
                    uint16_t v = decode(&b[2 * k]);
                    buf[i + k] = (v == MIC_SAMPLE_INVALID) ? 0 : v;
                }
            }
            i += 8;
        }
    }
    for (; i < count; i++) {
        buf[i] = mic_read_sample(tile);
    }
}

/* ================================================================
 * Audio analysis utilities — pure computation, no I2C
 * ================================================================ */

/** @brief Compute the DC level (mean) of a sample buffer. */
uint16_t tile_sense_mic_dc_level(tile_t *tile, const uint16_t *samples, uint16_t count)
{
    (void)tile;
    if (count == 0) return 0;
    uint32_t sum = 0;
    for (uint16_t i = 0; i < count; i++) {
        sum += samples[i];
    }
    return (uint16_t)(sum / count);
}

/** @brief Compute peak-to-peak amplitude of a sample buffer. */
uint16_t tile_sense_mic_peak_to_peak(tile_t *tile, const uint16_t *samples, uint16_t count)
{
    (void)tile;
    if (count == 0) return 0;
    uint16_t min_val = samples[0];
    uint16_t max_val = samples[0];
    for (uint16_t i = 1; i < count; i++) {
        if (samples[i] < min_val) min_val = samples[i];
        if (samples[i] > max_val) max_val = samples[i];
    }
    return max_val - min_val;
}

/** @brief Compute RMS amplitude relative to a DC offset. */
uint16_t tile_sense_mic_rms(tile_t *tile, const uint16_t *samples, uint16_t count,
                            uint16_t dc_offset)
{
    (void)tile;
    if (count == 0) return 0;
    uint32_t sum_sq = 0;
    for (uint16_t i = 0; i < count; i++) {
        int32_t ac = (int32_t)samples[i] - (int32_t)dc_offset;
        sum_sq += (uint32_t)(ac * ac);
    }
    /* Integer square root — Newton's method */
    uint32_t mean_sq = sum_sq / count;
    if (mean_sq == 0) return 0;
    uint32_t x = mean_sq;
    uint32_t y = (x + 1) >> 1;
    while (y < x) {
        x = y;
        y = (x + mean_sq / x) >> 1;
    }
    return (uint16_t)x;
}

/** @brief Convert peak-to-peak raw amplitude to millivolts. */
uint16_t tile_sense_mic_amplitude_mv(tile_t *tile, uint16_t pp_raw)
{
    mic_state_t *s = state_for(tile);
    return (uint16_t)(((uint32_t)pp_raw * s->vref_mv) >> 12);
}

/* ================================================================
 * Tier-2 — SPL conversion + event-detection helpers
 *
 * The signal chain as built (see the header):
 *   - CMM-2718AT-38164W: −38 dBV/Pa, i.e. 12.59 mV RMS per pascal (94 dB SPL).
 *   - C2/C3 (100 nF / 100 nF) halve it into the amp; the AD8605 gains 48x:
 *     24x from mic to ADC.
 *   - In the VDD / INTERNAL reference modes the amp rests at 0 V, so only
 *     the positive half-cycles reach the ADC. For any symmetric signal that
 *     halves the mean square about the resting level: the RMS reads 1/√2
 *     (−3.01 dB) of the full signal's. When the measured resting level is
 *     near 0 (dc_offset below MIC_HALFWAVE_REST_MAX), that is added back.
 *
 *   dB SPL = 94 + 20·log10(adc_mV / 24 / 12.59) (+3.01 if half-wave)
 *          = 20·log10(adc_mV) + 94 − 22.00 − 27.60 (+3.01)
 *          = 20·log10(adc_mV) + 44.40 (+3.01)
 *
 * The RMS is carried in 0.1 mV units, so one ADC count (0.8 mV at a 3.3 V
 * reference) still resolves: 20·log10(dmV) − 20 dB = 20·log10(mV). The
 * log term is tabulated for 1..32; larger values are halved into range
 * with 6.02 dB added per halving.
 *
 * Range on this board: the floor is about 45 dB SPL (one count of RMS at
 * the VDD reference; the mic's own noise is ~30 dBA), and half-wave
 * clipping at full scale caps it near 112 dB (VDD) / 108 dB (internal
 * 2.048 V), below the mic's 128 dB overload point. Carried in 0.1 dB.
 * ================================================================ */

/* Sample window for tier-2 ops. ~5 ms at 12.5 ksps (400 kHz I2C). */
#define MIC_TIER2_BUF_LEN  64

/* SPL polling interval used by wait_for_sound / detect_clap. */
#define MIC_TIER2_POLL_MS  5

/* 94 dB − 20·log10(12.59 mV/Pa) − 20·log10(24), in 0.1 dB: the mV→SPL
 * offset for this chain. */
#define MIC_SPL_OFFSET_DX_DB   444
/* 20·log10(√2) in 0.1 dB: the half-wave correction. */
#define MIC_HALFWAVE_DX_DB     30
/* Resting level (counts) below which the signal is taken as half-wave. The
 * amp's offset (≤65 µV × 48) is ~4 counts; a centred signal rests near
 * half scale. */
#define MIC_HALFWAVE_REST_MAX  256

/* 20*log10(n) in 0.1 dB units, indexed by integer n.
 * Index 0 is unused (log10(0) is −∞ — handled by the caller).
 * Indexes 1..32 covered; mv_rms_to_spl_dx10() scales larger values down. */
static const int16_t k_log10_x20_table[33] = {
       0,    0,   60,   95,  120,  140,  156,  169,  /* idx 0..7 */
     181,  191,  200,  208,  216,  223,  229,  235,  /* idx 8..15 */
     241,  246,  251,  256,  260,  264,  268,  272,  /* idx 16..23 */
     276,  280,  283,  286,  289,  292,  295,  298,  /* idx 24..31 */
     301,                                            /* idx 32 */
};

/* Convert an RMS in 0.1 mV units to SPL in 0.1 dB units. Integer-only. */
static int16_t dmv_rms_to_spl_dx10(uint16_t dmv_rms, uint8_t half_wave)
{
    /* No measurable signal: report the documented floor value. */
    if (dmv_rms == 0) {
        return 300;  /* 30.0 dB SPL — below the noise floor */
    }
    /* Above 32, halve k times into the table's 16..32 range and add
     * k * 20*log10(2) (6.02 dB each) back. Rounding error <= ~0.3 dB. */
    uint8_t k = 0;
    while ((dmv_rms >> k) > 32) k++;
    uint16_t idx = (k == 0) ? dmv_rms
                            : (uint16_t)((dmv_rms + (1u << (k - 1))) >> k);
    if (idx > 32) idx = 32;
    int16_t log_term = (int16_t)(k_log10_x20_table[idx]  /* 20*log10(dmV) in 0.1 dB */
                                 + ((int16_t)k * 602) / 10);
    /* 20·log10(mV) = 20·log10(dmV) − 20 dB. */
    int16_t spl = (int16_t)(log_term - 200 + MIC_SPL_OFFSET_DX_DB
                            + (half_wave ? MIC_HALFWAVE_DX_DB : 0));
    return (spl < 300) ? 300 : spl;
}

/* Capture a sample buffer and return its SPL. Used by all SPL helpers. */
static int16_t mic_capture_spl(tile_t *tile)
{
    uint16_t buf[MIC_TIER2_BUF_LEN];
    mic_state_t *s = state_for(tile);
    tile_sense_mic_get_samples(tile, buf, MIC_TIER2_BUF_LEN);
    uint16_t rms_raw = tile_sense_mic_rms(tile, buf, MIC_TIER2_BUF_LEN, s->dc_offset);
    /* 0.1 mV = raw * vref_mv * 10 / 4096 */
    uint16_t dmv = (uint16_t)(((uint32_t)rms_raw * s->vref_mv * 10u) >> 12);
    return dmv_rms_to_spl_dx10(dmv, s->dc_offset < MIC_HALFWAVE_REST_MAX);
}

/** @brief Quick "is the room loud right now?" check. */
uint8_t tile_sense_mic_is_loud(tile_t *tile, int16_t threshold_db)
{
    int16_t spl = tile_sense_mic_read_spl_db(tile);
    return (spl > threshold_db) ? 1 : 0;
}

/** @brief Read instantaneous SPL in 0.1 dB units.
 *
 * TODO HW: not yet checked against a reference SPL meter. The constants
 * come from the datasheets and the schematic (±1 dB mic sensitivity,
 * 1 % resistors, the half-wave correction assumes a symmetric signal),
 * so expect a few dB absolute, with no A-weighting. The resting level is
 * captured at init(); call tile_sense_mic_calibrate() to re-zero. */
int16_t tile_sense_mic_read_spl_db(tile_t *tile)
{
    return mic_capture_spl(tile);
}

/** @brief Block until ambient SPL crosses threshold (or timeout). */
uint8_t tile_sense_mic_wait_for_sound(tile_t *tile, int16_t threshold_db,
                                      uint32_t timeout_ms)
{
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        int16_t spl = tile_sense_mic_read_spl_db(tile);
        if (spl > threshold_db) return 1;
        tile->hal->delay_ms(MIC_TIER2_POLL_MS);
        elapsed += MIC_TIER2_POLL_MS;
    }
    return 0;
}

/** @brief Detect a clap pattern (two peaks bracketed by quiet).
 *
 * TODO HW: coarse two-peaks-in-window pattern detector. Door slams,
 * drawer slams, hand-against-thigh, double-knocks, and percussive
 * impacts will all trigger this. The signature of a real human clap
 * is broadband with a fast attack — distinguishing one from a slam
 * needs a trained classifier, which won't fit the integer-only
 * constraint. Tune `peak_thr`/`quiet_thr`/gap window at the bench
 * for the target environment. */
uint8_t tile_sense_mic_detect_clap(tile_t *tile, uint32_t timeout_ms)
{
    /* Pattern thresholds (0.1 dB units):
     *   peak  >= 700 (70.0 dB SPL)
     *   quiet <  500 (50.0 dB SPL)
     * Time windows (ms):
     *   pre-quiet:   >= 100  (need a calm baseline)
     *   gap:         50..500 (between peaks)
     *   post-quiet:  >= 100
     *   peak width:  any single sample crossing
     */
    const int16_t peak_thr  = 700;
    const int16_t quiet_thr = 500;
    const uint32_t pre_quiet_ms  = 100;
    const uint32_t post_quiet_ms = 100;
    const uint32_t gap_min_ms = 50;
    const uint32_t gap_max_ms = 500;

    /* State machine:
     *   0: waiting for pre-quiet bracket
     *   1: pre-quiet seen, waiting for first peak
     *   2: first peak seen, waiting for quiet gap
     *   3: in gap, waiting for second peak (bounded by gap_max_ms)
     *   4: second peak seen, waiting for post-quiet bracket
     */
    uint8_t  st = 0;
    uint32_t state_ms = 0;        /* time spent in current state */
    uint32_t elapsed = 0;

    while (elapsed < timeout_ms) {
        int16_t spl = tile_sense_mic_read_spl_db(tile);
        switch (st) {
            case 0:
                if (spl < quiet_thr) {
                    state_ms += MIC_TIER2_POLL_MS;
                    if (state_ms >= pre_quiet_ms) { st = 1; state_ms = 0; }
                } else {
                    state_ms = 0;  /* not quiet — restart */
                }
                break;
            case 1:
                if (spl > peak_thr) { st = 2; state_ms = 0; }
                break;
            case 2:
                if (spl < quiet_thr) { st = 3; state_ms = MIC_TIER2_POLL_MS; }
                else { state_ms += MIC_TIER2_POLL_MS;
                       if (state_ms > 100) { st = 1; state_ms = 0; } /* peak too long */ }
                break;
            case 3:
                state_ms += MIC_TIER2_POLL_MS;
                if (spl > peak_thr) {
                    if (state_ms >= gap_min_ms && state_ms <= gap_max_ms) {
                        st = 4; state_ms = 0;
                    } else {
                        /* peak in wrong window — treat as a fresh first peak */
                        st = 2; state_ms = 0;
                    }
                } else if (state_ms > gap_max_ms) {
                    /* gap too long, abandon — restart from pre-quiet. */
                    st = 0; state_ms = 0;
                }
                break;
            case 4:
                if (spl < quiet_thr) {
                    state_ms += MIC_TIER2_POLL_MS;
                    if (state_ms >= post_quiet_ms) return 1;  /* clap! */
                } else {
                    /* second peak echoed by more sound — not a clean clap. */
                    st = 0; state_ms = 0;
                }
                break;
            default:
                st = 0; state_ms = 0;
                break;
        }
        tile->hal->delay_ms(MIC_TIER2_POLL_MS);
        elapsed += MIC_TIER2_POLL_MS;
    }
    return 0;
}
