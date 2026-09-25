/**
 * @file   tile_sense_m_3g.c
 * @brief  Sense.M.3G (BMM350) -- platform-agnostic driver implementation.
 *
 * Platform-agnostic. All bus access via tile->hal function pointers.
 *
 * Two sources, and it matters which is which:
 *
 *   [DS]  Bosch Sensortec BMM350 datasheet, BST-BMM350-DS001-27.
 *         Register map, bit semantics, timings, power-mode rules.
 *
 *   [API] Bosch Sensortec BMM350_SensorAPI, BSD-3-Clause,
 *         Copyright (c) 2025 Bosch Sensortec GmbH.
 *         https://github.com/boschsensortec/BMM350_SensorAPI
 *         The OTP trim-word map, the raw-count scaling constants and the
 *         compensation chain. NONE of this is in the datasheet — chapter 7
 *         is a link to the API, and the data registers are uncompensated.
 *
 * The API reference is floating-point; house style here is integer-only, so
 * the chain is reimplemented in Q16.16 fixed point on int64 intermediates.
 * Outputs are nanotesla and milli-degrees Celsius. Fixed-point rounding
 * costs about 1e-4 % on the scaling constants, four orders of magnitude
 * below the part's own +/-1 % gain spec.
 *
 * Every I2C read returns two dummy bytes first [DS §9.2.3] — all reads go
 * through read_regs(), which strips them.
 */

#include "tile_sense_m_3g.h"
#include <stddef.h>

/* ================================================================
 * Instance -> I2C address table
 * ================================================================ */

static const uint8_t id_table[] = {
    BMM350_I2C_ADDR_LOW,    /* 0: 0x14, ADSEL low  — Sense.M.3G */
    BMM350_I2C_ADDR_HIGH,   /* 1: 0x15, ADSEL high */
};

#define NUM_INSTANCES  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    return (instance < NUM_INSTANCES) ? id_table[instance] : 0;
}

/* ================================================================
 * Timings, in microseconds unless noted [DS / API]
 * ================================================================ */

#define BMM350_START_UP_MS            3    /* Power-on to first command */
#define BMM350_SOFT_RESET_MS          24
#define BMM350_GOTO_SUSPEND_MS        6
#define BMM350_SUSPEND_TO_NORMAL_MS   38
#define BMM350_UPD_OAE_MS             1    /* ODR/averaging update command */
#define BMM350_BR_MS                  14   /* Bit reset */
#define BMM350_FGR_MS                 18   /* Flip-gain reset */
#define BMM350_OTP_POLL_US            300

/* Suspend -> forced settling, indexed by averaging (0..3) [API] */
static const uint8_t sus_to_forced_ms[4]      = { 15, 17, 20, 28 };
static const uint8_t sus_to_forced_fast_ms[4] = {  4,  5,  9, 16 };

/* ---- OTP access [DS §8.29-8.32] ---- */

#define BMM350_OTP_CMD_DIR_READ       0x20
#define BMM350_OTP_CMD_PWR_OFF_OTP    0x80
#define BMM350_OTP_WORD_ADDR_MSK      0x1F
#define BMM350_OTP_STATUS_ERROR_MSK   0xE0
#define BMM350_OTP_STATUS_CMD_DONE    0x01
#define BMM350_OTP_WORDS              32

/* ---- OTP word indices holding trim data [API] ---- */

#define OTP_TEMP_OFF_SENS   0x0D
#define OTP_MAG_OFFSET_X    0x0E
#define OTP_MAG_OFFSET_Y    0x0F
#define OTP_MAG_OFFSET_Z    0x10
#define OTP_MAG_SENS_X      0x10
#define OTP_MAG_SENS_Y      0x11
#define OTP_MAG_SENS_Z      0x11
#define OTP_MAG_TCO_X       0x12
#define OTP_MAG_TCO_Y       0x13
#define OTP_MAG_TCO_Z       0x14
#define OTP_MAG_TCS_X       0x12
#define OTP_MAG_TCS_Y       0x13
#define OTP_MAG_TCS_Z       0x14
#define OTP_CROSS_X_Y       0x15
#define OTP_CROSS_Y_X       0x15
#define OTP_CROSS_Z_X       0x16
#define OTP_CROSS_Z_Y       0x16
#define OTP_MAG_DUT_T_0     0x18

/* ================================================================
 * Fixed-point scaling
 * ================================================================ */

#define Q16  16
#define Q16_ONE  65536
#define Q24  24
#define Q24_ONE  16777216

/*
 * Raw counts -> nanotesla, and raw temperature counts -> milli-degC.
 *
 * [API] update_default_coefiecents() computes these from the analog
 * chain, in float:
 *
 *   power   = 1e6 / 1048576            adc_gain = 1 / 1.5
 *   bxy_sens = 14.55   bz_sens = 9.0   lut_gain = 0.714607238769531
 *   ina_xy_gain = 19.46  ina_z_gain = 31.0   temp_sens = 0.00204
 *
 *   uT/LSB (x,y) = power / (bxy_sens * ina_xy_gain * adc_gain * lut_gain)
 *   uT/LSB (z)   = power / (bz_sens  * ina_z_gain  * adc_gain * lut_gain)
 *   degC/LSB     = 1 / (temp_sens * adc_gain * lut_gain * 1048576)
 *
 * Evaluated exactly and scaled to nT/LSB and m°C/LSB in Q16:
 *
 *   xy:   7.069978699505961 nT/LSB  -> 463338 (error -2.7e-5 %)
 *   z:    7.174964082129807 nT/LSB  -> 470218 (error -9.5e-5 %)
 *   temp: 0.981281852408930 m°C/LSB ->  64309 (error -4.5e-4 %)
 */
#define LSB_TO_NT_XY_Q16   463338
#define LSB_TO_NT_Z_Q16    470218
#define LSB_TO_MC_TEMP_Q16 64309

/* [API] Fixed offset subtracted from the scaled temperature, in m°C. */
#define TEMP_RAW_OFFSET_MC 25490

/* ================================================================
 * Per-instance driver state
 * ================================================================ */

typedef struct {
    /* Compensation coefficients, derived from OTP at init [API] */
    int32_t offset_nt[3];    /* Per-axis zero offset, nT */
    int32_t sens_q16[3];     /* Per-axis sensitivity correction, Q16 */
    int32_t tco_q16[3];      /* Temperature coefficient of offset, Q16 nT/K */
    int32_t tcs_q16[3];      /* Temperature coefficient of sensitivity, Q16 1/K */
    int32_t cross_x_y_q24;   /* Cross-axis terms, Q24 (see derive_coefficients) */
    int32_t cross_y_x_q24;
    int32_t cross_z_x_q24;
    int32_t cross_z_y_q24;
    int32_t t_offs_mc;       /* Temperature offset, m°C */
    int32_t t_sens_q16;      /* Temperature sensitivity correction, Q16 */
    int32_t dut_t0_mc;       /* Reference temperature, m°C */

    uint16_t otp[BMM350_OTP_WORDS];

    /* Last sample */
    int32_t raw[4];          /* Uncompensated counts: x, y, z, temp */
    int32_t mag_nt[3];
    int32_t temp_mc;

    sense_m_3g_data_cb_t on_data;
    void *data_ctx;

    volatile uint8_t int_flag;   /* Set by the EXTI ISR */
    uint8_t int_pin;             /* Core pad, 0 = polled */
    uint8_t axis_en;             /* Cached axis-enable mask */
    uint8_t avg;                 /* Cached averaging, for settling delays */
    uint8_t odr;                 /* Cached ODR, to pick FM vs FM_FAST */
    uint8_t fm_primed;           /* A forced conversion has run since bring-up */
} bmm350_state_t;

static bmm350_state_t state[NUM_INSTANCES];

static bmm350_state_t *state_for(tile_t *tile)
{
    for (uint8_t i = 0; i < NUM_INSTANCES; i++)
        if (id_table[i] == tile->id) return &state[i];
    return &state[0];
}

/* ================================================================
 * Bus helpers — dummy bytes handled here, once
 * ================================================================ */

static void memzero(void *p, uint16_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) *b++ = 0;
}

/**
 * Read `len` bytes from `reg`, discarding the two dummy bytes the device
 * sends first [DS §9.2.3]. Caps at a buffer that covers the longest burst
 * this driver performs (the 12-byte data block).
 */
static uint8_t read_regs(tile_t *tile, uint8_t reg, uint8_t *out, uint8_t len)
{
    uint8_t buf[BMM350_MAG_TEMP_DATA_LEN + BMM350_DUMMY_BYTES];

    if (len > BMM350_MAG_TEMP_DATA_LEN)
        return 0;

    memzero(buf, sizeof(buf));
    if (tile->hal->i2c_read(tile->hal->handle, tile->id, reg, buf,
                            (uint16_t)(len + BMM350_DUMMY_BYTES)) != 0)
        return 0;

    for (uint8_t i = 0; i < len; i++)
        out[i] = buf[i + BMM350_DUMMY_BYTES];
    return 1;
}

static uint8_t read_reg(tile_t *tile, uint8_t reg)
{
    uint8_t v = 0;
    (void)read_regs(tile, reg, &v, 1);
    return v;
}

static void write_reg(tile_t *tile, uint8_t reg, uint8_t value)
{
    tile->hal->i2c_write(tile->hal->handle, tile->id, reg, &value, 1);
}

/* ================================================================
 * Sign extension [API fix_sign]
 * ================================================================ */

static int32_t fix_sign(uint32_t val, uint8_t bits)
{
    uint32_t half = 1UL << (bits - 1);
    return (val >= half) ? ((int32_t)val - (int32_t)(half << 1)) : (int32_t)val;
}

/* ================================================================
 * OTP readout and coefficient derivation [API]
 * ================================================================ */

static uint8_t read_otp_word(tile_t *tile, uint8_t addr, uint16_t *out)
{
    write_reg(tile, BMM350_REG_OTP_CMD,
              (uint8_t)(BMM350_OTP_CMD_DIR_READ | (addr & BMM350_OTP_WORD_ADDR_MSK)));

    /* Poll until the command completes or reports an error. 300 us per
     * attempt [API]; the HAL only offers millisecond delays, so this
     * polls a little more slowly and gives up after ~20 ms. */
    for (uint8_t attempt = 0; attempt < 20; attempt++) {
        tile->hal->delay_ms(1);
        uint8_t status = read_reg(tile, BMM350_REG_OTP_STATUS);

        if (status & BMM350_OTP_STATUS_ERROR_MSK)
            return 0;
        if (status & BMM350_OTP_STATUS_CMD_DONE) {
            uint8_t msb = read_reg(tile, BMM350_REG_OTP_DATA_MSB);
            uint8_t lsb = read_reg(tile, BMM350_REG_OTP_DATA_LSB);
            *out = (uint16_t)(((uint16_t)msb << 8) | lsb);
            return 1;
        }
    }
    return 0;
}

/**
 * Turn the raw OTP words into compensation coefficients.
 *
 * [API] update_mag_off_sens(). The bit packing is not documented anywhere
 * else — offsets are 12-bit signed split across word boundaries, and the
 * divisors below (5, 32, 256, 512, 800, 16384) are the API's, converted
 * here into integer nT / m°C / Q16 forms.
 */
static void derive_coefficients(bmm350_state_t *s)
{
    const uint16_t *otp = s->otp;

    /* Offsets: 12-bit signed, packed across three words, in uT. */
    uint16_t off_x = otp[OTP_MAG_OFFSET_X] & 0x0FFF;
    uint16_t off_y = (uint16_t)(((otp[OTP_MAG_OFFSET_X] & 0xF000) >> 4) +
                                (otp[OTP_MAG_OFFSET_Y] & 0x00FF));
    uint16_t off_z = (uint16_t)((otp[OTP_MAG_OFFSET_Y] & 0x0F00) +
                                (otp[OTP_MAG_OFFSET_Z] & 0x00FF));
    uint16_t t_off = otp[OTP_TEMP_OFF_SENS] & 0x00FF;

    s->offset_nt[0] = fix_sign(off_x, 12) * 1000;   /* uT -> nT */
    s->offset_nt[1] = fix_sign(off_y, 12) * 1000;
    s->offset_nt[2] = fix_sign(off_z, 12) * 1000;
    s->t_offs_mc    = fix_sign(t_off, 8) * 200;     /* /5 degC -> m°C */

    /* Sensitivity: 8-bit signed, /256 (temperature /512). */
    uint8_t sens_x = (uint8_t)((otp[OTP_MAG_SENS_X] & 0xFF00) >> 8);
    uint8_t sens_y = (uint8_t)(otp[OTP_MAG_SENS_Y] & 0x00FF);
    uint8_t sens_z = (uint8_t)((otp[OTP_MAG_SENS_Z] & 0xFF00) >> 8);
    uint8_t t_sens = (uint8_t)((otp[OTP_TEMP_OFF_SENS] & 0xFF00) >> 8);

    s->sens_q16[0] = fix_sign(sens_x, 8) * (Q16_ONE / 256);
    s->sens_q16[1] = fix_sign(sens_y, 8) * (Q16_ONE / 256);
    s->sens_q16[2] = fix_sign(sens_z, 8) * (Q16_ONE / 256);
    s->t_sens_q16  = fix_sign(t_sens, 8) * (Q16_ONE / 512);

    /* Temperature coefficient of offset: 8-bit signed, /32 uT/K.
     * In Q16 nT/K that is raw * 1000 * 65536 / 32 = raw * 2048000. */
    s->tco_q16[0] = fix_sign((uint8_t)(otp[OTP_MAG_TCO_X] & 0x00FF), 8) * 2048000;
    s->tco_q16[1] = fix_sign((uint8_t)(otp[OTP_MAG_TCO_Y] & 0x00FF), 8) * 2048000;
    s->tco_q16[2] = fix_sign((uint8_t)(otp[OTP_MAG_TCO_Z] & 0x00FF), 8) * 2048000;

    /* Temperature coefficient of sensitivity: 8-bit signed, /16384 per K. */
    s->tcs_q16[0] = fix_sign((uint8_t)((otp[OTP_MAG_TCS_X] & 0xFF00) >> 8), 8) * (Q16_ONE / 16384);
    s->tcs_q16[1] = fix_sign((uint8_t)((otp[OTP_MAG_TCS_Y] & 0xFF00) >> 8), 8) * (Q16_ONE / 16384);
    s->tcs_q16[2] = fix_sign((uint8_t)((otp[OTP_MAG_TCS_Z] & 0xFF00) >> 8), 8) * (Q16_ONE / 16384);

    /* Reference temperature: 16-bit signed, /512 degC, biased by +23 degC. */
    s->dut_t0_mc = (int32_t)((fix_sign(otp[OTP_MAG_DUT_T_0], 16) * 1000) / 512) + 23000;

    /* Cross-axis: 8-bit signed, /800. Not a power of two, so compute with
     * a 64-bit intermediate rather than a shift — and carry these in Q24
     * rather than Q16. They multiply the full field value, so a Q16
     * quantum here (1.5e-5) lands as ~30 nT of error at full scale; Q24
     * pushes that below a tenth of a nT. */
    s->cross_x_y_q24 = (int32_t)(((int64_t)fix_sign((uint8_t)(otp[OTP_CROSS_X_Y] & 0x00FF), 8) * Q24_ONE) / 800);
    s->cross_y_x_q24 = (int32_t)(((int64_t)fix_sign((uint8_t)((otp[OTP_CROSS_Y_X] & 0xFF00) >> 8), 8) * Q24_ONE) / 800);
    s->cross_z_x_q24 = (int32_t)(((int64_t)fix_sign((uint8_t)(otp[OTP_CROSS_Z_X] & 0x00FF), 8) * Q24_ONE) / 800);
    s->cross_z_y_q24 = (int32_t)(((int64_t)fix_sign((uint8_t)((otp[OTP_CROSS_Z_Y] & 0xFF00) >> 8), 8) * Q24_ONE) / 800);
}

static uint8_t otp_dump(tile_t *tile, bmm350_state_t *s)
{
    for (uint8_t i = 0; i < BMM350_OTP_WORDS; i++) {
        if (!read_otp_word(tile, i, &s->otp[i]))
            return 0;
    }
    derive_coefficients(s);

    /* Power the OTP back down — it is only needed at init [API]. */
    write_reg(tile, BMM350_REG_OTP_CMD, BMM350_OTP_CMD_PWR_OFF_OTP);
    return 1;
}

/* ================================================================
 * Compensation chain [API bmm350_get_compensated_mag_xyz_temp_data]
 * ================================================================ */

/** Round a Q16 value to the nearest integer, symmetric about zero. */
static int32_t q16_round(int64_t v)
{
    return (int32_t)((v >= 0) ? ((v + (Q16_ONE / 2)) >> Q16)
                              : -(((-v) + (Q16_ONE / 2)) >> Q16));
}

static void compensate(bmm350_state_t *s)
{
    /*
     * The whole chain runs in Q16 — magnetic values as Q16 nanotesla,
     * temperature as Q16 milli-degrees — and rounds once at the end.
     * Truncating to whole nT at each step instead costs about 100 nT
     * worst case, which is the same order as the part's own 190 nT rms
     * noise floor; carrying the fraction brings it under 2 nT.
     *
     * Headroom: |raw| < 2^23, so a Q16 nT value stays under 2^23 * 7.2 *
     * 2^16 ~ 4e12, and the largest intermediate (v * Q16_ONE) reaches
     * ~2.5e17 — comfortably inside int64's 9.2e18.
     */
    int64_t v[3];
    v[0] = (int64_t)s->raw[0] * LSB_TO_NT_XY_Q16;
    v[1] = (int64_t)s->raw[1] * LSB_TO_NT_XY_Q16;
    v[2] = (int64_t)s->raw[2] * LSB_TO_NT_Z_Q16;

    int64_t t = (int64_t)s->raw[3] * LSB_TO_MC_TEMP_Q16
                - ((int64_t)TEMP_RAW_OFFSET_MC * Q16_ONE);

    /* Temperature first — the magnetic correction depends on it. */
    t = (((int64_t)(Q16_ONE + s->t_sens_q16) * t) >> Q16)
        + ((int64_t)s->t_offs_mc * Q16_ONE);
    s->temp_mc = q16_round(t);

    /* Deviation from the trim reference temperature, Q16 m°C. */
    int64_t dt = t - ((int64_t)s->dut_t0_mc * Q16_ONE);

    for (uint8_t i = 0; i < 3; i++) {
        v[i] = ((int64_t)(Q16_ONE + s->sens_q16[i]) * v[i]) >> Q16;
        v[i] += (int64_t)s->offset_nt[i] * Q16_ONE;

        /* tco is Q16 nT/K, dt is Q16 m°C: the product is Q32 nT·m°C, so
         * scale m°C to K and shift back to Q16. Divide before shifting —
         * doing it the other way rounds a Q16 quantity by 1000 and leaves
         * ~1.5e-3 % of gain error behind, which is most of the residual. */
        v[i] += ((int64_t)s->tco_q16[i] * dt / 1000) >> Q16;

        int64_t denom = Q16_ONE + (((int64_t)s->tcs_q16[i] * dt / 1000) >> Q16);
        if (denom != 0)
            v[i] = (v[i] * Q16_ONE) / denom;
    }

    /* Cross-axis decoupling. Coefficients are Q24; the shared denominator
     * stays Q16 so that scaling a Q16 nT value by it cannot overflow. */
    int64_t den = Q16_ONE - (((int64_t)s->cross_y_x_q24 * s->cross_x_y_q24) >> (Q24 + Q24 - Q16));
    if (den == 0)
        den = Q16_ONE;

    int64_t x = ((v[0] - (((int64_t)s->cross_x_y_q24 * v[1]) >> Q24)) * Q16_ONE) / den;
    int64_t y = ((v[1] - (((int64_t)s->cross_y_x_q24 * v[0]) >> Q24)) * Q16_ONE) / den;

    int64_t term_x = (((int64_t)s->cross_y_x_q24 * s->cross_z_y_q24) >> Q24) - s->cross_z_x_q24;
    int64_t term_y = (int64_t)s->cross_z_y_q24 -
                     (((int64_t)s->cross_x_y_q24 * s->cross_z_x_q24) >> Q24);
    int64_t z = v[2] + (((v[0] * term_x - v[1] * term_y) >> Q24) * Q16_ONE) / den;

    /* A disabled axis reports zero rather than a stale or garbage value. */
    s->mag_nt[0] = (s->axis_en & BMM350_EN_X) ? q16_round(x) : 0;
    s->mag_nt[1] = (s->axis_en & BMM350_EN_Y) ? q16_round(y) : 0;
    s->mag_nt[2] = (s->axis_en & BMM350_EN_Z) ? q16_round(z) : 0;
}

/* ================================================================
 * Power modes
 * ================================================================ */

/* Issue a PMU command and wait out its settling time. */
static void pmu_command(tile_t *tile, uint8_t cmd, uint16_t settle_ms)
{
    write_reg(tile, BMM350_REG_PMU_CMD, cmd);
    if (settle_ms)
        tile->hal->delay_ms(settle_ms);
}

static void set_mode_direct(tile_t *tile, uint8_t mode)
{
    bmm350_state_t *s = state_for(tile);
    uint16_t settle = 0;

    switch (mode) {
    case BMM350_PMU_CMD_NM:      settle = BMM350_SUSPEND_TO_NORMAL_MS; break;
    case BMM350_PMU_CMD_FM:      settle = sus_to_forced_ms[s->avg & 3]; break;
    case BMM350_PMU_CMD_FM_FAST: settle = sus_to_forced_fast_ms[s->avg & 3]; break;
    default: break;
    }
    pmu_command(tile, mode, settle);
}

void tile_sense_m_3g_set_mode(tile_t *tile, sense_m_3g_mode_t mode)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_SLEEPING)
        return;

    /* Forced mode is only reachable from suspend, and a normal-to-forced
     * request is silently ignored by the device [DS §5.1.4]. Park in
     * suspend first whenever we are leaving normal mode. PMU_CMD[3:0] is
     * write-only [DS §8.6], so ask PMU_CMD_STATUS_0.pwr_mode_is_normal
     * (bit 3) [DS §8.7] where we are. */
    if ((read_reg(tile, BMM350_REG_PMU_CMD_STATUS_0) >> 3) & 1U)
        pmu_command(tile, BMM350_PMU_CMD_SUS, BMM350_GOTO_SUSPEND_MS);

    set_mode_direct(tile, mode);

    tile->state = (mode == BMM350_PMU_CMD_SUS) ? TILE_STATE_SLEEPING
                                               : TILE_STATE_READY;
}

void tile_sense_m_3g_sleep(tile_t *tile)
{
    tile_sense_m_3g_set_mode(tile, SENSE_M_3G_MODE_SUSPEND);
}

void tile_sense_m_3g_wake(tile_t *tile)
{
    tile_sense_m_3g_set_mode(tile, SENSE_M_3G_MODE_NORMAL);
}

uint8_t tile_sense_m_3g_trigger_measurement(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_SLEEPING)
        return 0;

    bmm350_state_t *s = state_for(tile);

    /* FM_FAST is only valid at 25 Hz and above; below that the slow
     * forced mode is mandatory [DS §5.1.4]. Lower ODR enum = faster rate,
     * so "25 Hz or faster" is odr <= ODR_25HZ. And the first forced
     * trigger after init must be FM when more than 0.1 s has passed —
     * which a program cannot know — so the first one always is. */
    uint8_t cmd = (s->fm_primed && s->odr <= SENSE_M_3G_ODR_25HZ)
                ? BMM350_PMU_CMD_FM_FAST : BMM350_PMU_CMD_FM;

    /* INT_STATUS clears on read [DS §8.13]: drop a data-ready left over
     * from earlier, so the poll below waits for THIS conversion. */
    (void)read_reg(tile, BMM350_REG_INT_STATUS);

    tile_sense_m_3g_set_mode(tile, (sense_m_3g_mode_t)cmd);
    s->fm_primed = 1;

    /* The conversion runs to completion on its own; data-ready marks it. */
    for (uint8_t i = 0; i < 50; i++) {
        if (read_reg(tile, BMM350_REG_INT_STATUS) & BMM350_INT_STATUS_DRDY)
            return 1;
        tile->hal->delay_ms(1);
    }
    return 0;
}

uint8_t tile_sense_m_3g_magnetic_reset(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_SLEEPING)
        return 0;

    /* Magnetic reset only runs from suspend; restore normal mode after if
     * that is where we came from [API bmm350_magnetic_reset_and_wait]. */
    uint8_t status = read_reg(tile, BMM350_REG_PMU_CMD_STATUS_0);
    uint8_t was_normal = (status >> 3) & 1U;   /* pwr_mode_is_normal [DS §8.7] */

    if (was_normal)
        pmu_command(tile, BMM350_PMU_CMD_SUS, BMM350_GOTO_SUSPEND_MS);

    uint8_t ok = 1;

    /* Acknowledge check. PMU_CMD_STATUS_0[3:0] are status FLAGS (busy,
     * ODR/AVG overwritten, normal mode) [DS §8.7], never the command, so
     * comparing them with BR (7) / FGR (5) always failed. [API] reads the
     * last command's value from bits [7:5] (reserved in the datasheet). */
    pmu_command(tile, BMM350_PMU_CMD_BR, BMM350_BR_MS);
    if ((read_reg(tile, BMM350_REG_PMU_CMD_STATUS_0) >> 5) != BMM350_PMU_CMD_BR)
        ok = 0;

    pmu_command(tile, BMM350_PMU_CMD_FGR, BMM350_FGR_MS);
    if ((read_reg(tile, BMM350_REG_PMU_CMD_STATUS_0) >> 5) != BMM350_PMU_CMD_FGR)
        ok = 0;

    if (was_normal)
        set_mode_direct(tile, BMM350_PMU_CMD_NM);

    if (!ok)
        TILE_ON_ERROR(tile, "sense_m_3g: magnetic reset not acknowledged");
    return ok;
}

/* ================================================================
 * Lifecycle
 * ================================================================ */

/* Shared by init() and reset(): everything from the soft reset onward. */
static uint8_t bring_up(tile_t *tile, bmm350_state_t *s)
{
    tile->hal->delay_ms(BMM350_START_UP_MS);

    write_reg(tile, BMM350_REG_CMD, BMM350_CMD_SOFTRESET);
    tile->hal->delay_ms(BMM350_SOFT_RESET_MS);

    if (read_reg(tile, BMM350_REG_CHIP_ID) != BMM350_CHIP_ID) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_m_3g: wrong chip id");
        return 0;
    }

    if (!otp_dump(tile, s)) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_m_3g: OTP download failed — data would be uncompensated");
        return 0;
    }

    s->fm_primed = 0;

    /* Boot-time magnetic reset, so the transducer starts from a known
     * state and the CRST capacitor is charged [DS §5.1.5]. */
    tile->state = TILE_STATE_READY;
    (void)tile_sense_m_3g_magnetic_reset(tile);

    tile->state = TILE_STATE_SLEEPING;   /* Device rests in suspend */
    return 1;
}

static void isr_instance_0(void *ctx) { state[0].int_flag = 1; (void)ctx; }
static void isr_instance_1(void *ctx) { state[1].int_flag = 1; (void)ctx; }

uint8_t tile_sense_m_3g_find(tiles_pal_t *hal, uint8_t instance)
{
    uint8_t addr = resolve_id(instance);
    if (!addr) return 0;
    if (hal->i2c_is_ready(hal->handle, addr) != 0) return 0;

    /* An ACK alone is weak evidence on a shared bus — confirm the chip id,
     * remembering the two dummy bytes. */
    uint8_t buf[1 + BMM350_DUMMY_BYTES] = {0};
    if (hal->i2c_read(hal->handle, addr, BMM350_REG_CHIP_ID, buf, sizeof(buf)) != 0)
        return 0;
    return (buf[BMM350_DUMMY_BYTES] == BMM350_CHIP_ID) ? 1 : 0;
}

void tile_sense_m_3g_init(tiles_pal_t *hal, uint8_t instance,
                          tile_t *tile, const sense_m_3g_cfg_t *cfg)
{
    memzero(tile, sizeof(tile_t));
    tile->hal = hal;
    tile->id  = resolve_id(instance);

    if (!tile->id) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_m_3g: invalid instance");
        return;
    }

    bmm350_state_t *s = state_for(tile);
    memzero(s, sizeof(bmm350_state_t));
    s->axis_en = BMM350_EN_XYZ;
    s->odr     = SENSE_M_3G_ODR_25HZ;
    s->avg     = SENSE_M_3G_AVG_4;

    if (hal->i2c_is_ready(hal->handle, tile->id) != 0) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_m_3g: device not found");
        return;
    }
    tile->state = TILE_STATE_FOUND;

    if (!bring_up(tile, s))
        return;

    /* Apply configuration. */
    uint8_t odr = SENSE_M_3G_ODR_25HZ;
    uint8_t avg = SENSE_M_3G_AVG_4;
    if (cfg) {
        s->on_data  = cfg->on_data;
        s->data_ctx = cfg->data_ctx;
        s->int_pin  = cfg->int_pin;
        if (cfg->odr) odr = (uint8_t)cfg->odr;
        avg = (uint8_t)cfg->averaging;      /* AVG_NONE is 0 and is a real choice */
        if (cfg->axes) s->axis_en = cfg->axes;
    }

    tile_sense_m_3g_set_odr_averaging(tile, (sense_m_3g_odr_t)odr, (sense_m_3g_avg_t)avg);
    tile_sense_m_3g_set_axes(tile, s->axis_en);

    /* Data-ready mapping into INT_STATUS is what polling reads, so enable
     * it either way; only drive the pin when one is wired. */
    tile_sense_m_3g_configure_interrupt(tile, s->int_pin ? 1 : 0,
                                        0 /* active low */, 1 /* push-pull */,
                                        0 /* pulsed */);

    if (s->int_pin && hal->gpio_irq_enable) {
        void (*isr)(void *) = (instance == 0) ? isr_instance_0 : isr_instance_1;
        hal->gpio_irq_enable(hal->handle, s->int_pin,
                             TILES_GPIO_EDGE_FALLING, isr, NULL);
    }

    tile->state = TILE_STATE_SLEEPING;   /* In suspend until set_mode() */
}

void tile_sense_m_3g_reset(tile_t *tile)
{
    if (tile->state == TILE_STATE_NONE) return;
    bmm350_state_t *s = state_for(tile);

    memzero(s->raw, sizeof(s->raw));
    memzero(s->mag_nt, sizeof(s->mag_nt));
    s->temp_mc = 0;

    (void)bring_up(tile, s);
}

/* ================================================================
 * Data
 * ================================================================ */

uint8_t tile_sense_m_3g_read(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_SLEEPING)
        return 0;

    bmm350_state_t *s = state_for(tile);
    uint8_t d[BMM350_MAG_TEMP_DATA_LEN];

    /* One burst. The device freezes the data registers for its duration;
     * separate reads would tear across an update [DS §5.2]. */
    if (!read_regs(tile, BMM350_REG_MAG_X_XLSB, d, sizeof(d)))
        return 0;

    /* Three bytes per channel, little-endian. The datasheet describes the
     * payload as 21-bit signed inside a 24-bit register; the API sign-
     * extends from bit 23, which is what the parts actually produce, so
     * that is what we do. */
    for (uint8_t i = 0; i < 4; i++) {
        uint32_t v = (uint32_t)d[i * 3] |
                     ((uint32_t)d[i * 3 + 1] << 8) |
                     ((uint32_t)d[i * 3 + 2] << 16);
        s->raw[i] = fix_sign(v, 24);
    }

    compensate(s);
    return 1;
}

void tile_sense_m_3g_process(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY) return;

    bmm350_state_t *s = state_for(tile);

    if (s->int_pin) {
        if (!s->int_flag) return;
        s->int_flag = 0;
    } else if (!tile_sense_m_3g_data_ready(tile)) {
        return;
    }

    if (tile_sense_m_3g_read(tile) && s->on_data)
        s->on_data(tile, s->data_ctx);
}

void tile_sense_m_3g_on_data(tile_t *tile, sense_m_3g_data_cb_t cb, void *ctx)
{
    bmm350_state_t *s = state_for(tile);
    s->on_data  = cb;
    s->data_ctx = ctx;
}

int32_t tile_sense_m_3g_get_x_nt(tile_t *tile) { return state_for(tile)->mag_nt[0]; }
int32_t tile_sense_m_3g_get_y_nt(tile_t *tile) { return state_for(tile)->mag_nt[1]; }
int32_t tile_sense_m_3g_get_z_nt(tile_t *tile) { return state_for(tile)->mag_nt[2]; }

int32_t tile_sense_m_3g_get_temperature_mc(tile_t *tile)
{
    return state_for(tile)->temp_mc;
}

/** Integer square root, Newton's method — no libm, no float. */
static uint32_t isqrt64(uint64_t n)
{
    if (n == 0) return 0;

    uint64_t x = n, y = (x + 1) / 2;
    while (y < x) {
        x = y;
        y = (x + n / x) / 2;
    }
    return (uint32_t)x;
}

uint32_t tile_sense_m_3g_get_magnitude_nt(tile_t *tile)
{
    bmm350_state_t *s = state_for(tile);
    int64_t x = s->mag_nt[0], y = s->mag_nt[1], z = s->mag_nt[2];
    return isqrt64((uint64_t)(x * x + y * y + z * z));
}

uint8_t tile_sense_m_3g_data_ready(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_SLEEPING)
        return 0;
    return (read_reg(tile, BMM350_REG_INT_STATUS) & BMM350_INT_STATUS_DRDY) ? 1 : 0;
}

uint32_t tile_sense_m_3g_get_sensortime(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_SLEEPING)
        return 0;

    uint8_t d[3] = {0};
    if (!read_regs(tile, BMM350_REG_SENSORTIME_XLSB, d, sizeof(d)))
        return 0;
    return (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16);
}

/* ================================================================
 * Configuration
 * ================================================================ */

void tile_sense_m_3g_set_odr_averaging(tile_t *tile, sense_m_3g_odr_t odr,
                                       sense_m_3g_avg_t averaging)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_SLEEPING)
        return;

    /* High rates cannot sustain heavy averaging [DS Table 5]. Clamp rather
     * than let the device quietly reject the pair. */
    if (odr == SENSE_M_3G_ODR_400HZ && averaging > SENSE_M_3G_AVG_NONE)
        averaging = SENSE_M_3G_AVG_NONE;
    else if (odr == SENSE_M_3G_ODR_200HZ && averaging > SENSE_M_3G_AVG_2)
        averaging = SENSE_M_3G_AVG_2;
    else if (odr == SENSE_M_3G_ODR_100HZ && averaging > SENSE_M_3G_AVG_4)
        averaging = SENSE_M_3G_AVG_4;

    bmm350_state_t *s = state_for(tile);
    s->odr = odr;
    s->avg = averaging;

    write_reg(tile, BMM350_REG_PMU_CMD_AGGR_SET,
              (uint8_t)((odr & 0x0F) | ((averaging & 0x03) << 4)));

    /* A new ODR/averaging pair only takes effect on an update command. */
    uint8_t normal = (read_reg(tile, BMM350_REG_PMU_CMD_STATUS_0) >> 3) & 1U;
    if (normal) (void)read_reg(tile, BMM350_REG_INT_STATUS);   /* clear old drdy */
    pmu_command(tile, BMM350_PMU_CMD_UPD_OAE, BMM350_UPD_OAE_MS);

    /* Before another update may be issued, the busy flag must clear AND,
     * in normal mode, the first sample at the new setting must arrive:
     * busy alone can drop too early (PMU_CMD, UPD_OAE [DS §8.6]). Wait
     * here so back-to-back calls are safe; up to one period at the new
     * rate (640 ms at 1.5625 Hz) plus margin. */
    for (uint8_t i = 0; i < 20 && (read_reg(tile, BMM350_REG_PMU_CMD_STATUS_0) & 1U); i++)
        tile->hal->delay_ms(1);
    if (normal) {
        /* Period = 2.5 ms x 2^(code - 2): 400 Hz is 0x2, 1.5625 Hz is 0xA. */
        uint8_t code = (odr < SENSE_M_3G_ODR_400HZ) ? SENSE_M_3G_ODR_400HZ
                     : (odr > SENSE_M_3G_ODR_1_5625HZ) ? SENSE_M_3G_ODR_1_5625HZ : (uint8_t)odr;
        uint32_t period_ms = ((5u << (code - SENSE_M_3G_ODR_400HZ)) + 1u) / 2u;
        for (uint32_t t = 0; t < period_ms + 20; t++) {
            if (read_reg(tile, BMM350_REG_INT_STATUS) & BMM350_INT_STATUS_DRDY) break;
            tile->hal->delay_ms(1);
        }
    }
}

void tile_sense_m_3g_set_axes(tile_t *tile, uint8_t mask)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_SLEEPING)
        return;

    bmm350_state_t *s = state_for(tile);
    s->axis_en = mask & BMM350_EN_XYZ;
    write_reg(tile, BMM350_REG_PMU_CMD_AXIS_EN, s->axis_en);
}

void tile_sense_m_3g_configure_interrupt(tile_t *tile, uint8_t enable,
                                         uint8_t active_high, uint8_t push_pull,
                                         uint8_t latched)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_SLEEPING)
        return;

    uint8_t v = BMM350_INT_DRDY_EN;   /* Always map drdy into INT_STATUS */
    if (enable)      v |= BMM350_INT_OUTPUT_EN;
    if (active_high) v |= BMM350_INT_POL_HIGH;
    if (push_pull)   v |= BMM350_INT_OD_PUSHPULL;
    if (latched)     v |= BMM350_INT_MODE_LATCHED;

    write_reg(tile, BMM350_REG_INT_CTRL, v);
}

void tile_sense_m_3g_set_pad_drive(tile_t *tile, uint8_t drive)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_SLEEPING)
        return;
    write_reg(tile, BMM350_REG_PAD_CTRL, (uint8_t)(drive & 0x07));
}

void tile_sense_m_3g_set_i2c_watchdog(tile_t *tile, uint8_t enable, uint8_t long_wdt)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_SLEEPING)
        return;

    /* [DS §8.10] bit 0 enables, bit 1 selects the long timeout. */
    uint8_t v = 0;
    if (enable)   v |= (1U << 0);
    if (long_wdt) v |= (1U << 1);
    write_reg(tile, BMM350_REG_I2C_WDT_SET, v);
}

void tile_sense_m_3g_set_sensortime_always_on(tile_t *tile, uint8_t enable)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_SLEEPING)
        return;
    write_reg(tile, BMM350_REG_CTRL_USER, enable ? 1U : 0U);
}

/* ================================================================
 * Diagnostics / advanced
 * ================================================================ */

uint8_t tile_sense_m_3g_get_error(tile_t *tile)
{
    return read_reg(tile, BMM350_REG_ERR_REG);
}

uint8_t tile_sense_m_3g_get_pmu_status(tile_t *tile)
{
    return read_reg(tile, BMM350_REG_PMU_CMD_STATUS_0);
}

uint16_t tile_sense_m_3g_get_otp_word(tile_t *tile, uint8_t word)
{
    if (word >= BMM350_OTP_WORDS) return 0;
    return state_for(tile)->otp[word];
}

int32_t tile_sense_m_3g_get_raw(tile_t *tile, uint8_t axis)
{
    if (axis > 3) return 0;
    return state_for(tile)->raw[axis];
}

uint8_t tile_sense_m_3g_read_reg(tile_t *tile, uint8_t reg)
{
    if (tile->state == TILE_STATE_NONE || tile->state == TILE_STATE_ERROR)
        return 0;
    return read_reg(tile, reg);
}

void tile_sense_m_3g_write_reg(tile_t *tile, uint8_t reg, uint8_t value)
{
    if (tile->state == TILE_STATE_NONE || tile->state == TILE_STATE_ERROR)
        return;
    write_reg(tile, reg, value);
}
