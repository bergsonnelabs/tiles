/**
 * @file  tile_sense_bp.c
 * @brief Sense.BP tile driver implementation — ILPS22QS barometric pressure.
 */

#include "tile_sense_bp.h"

/* ---- Instance → I2C address table ---- */

static const uint8_t id_table[] = {
    ILPS22QS_I2C_ADDR_DEFAULT,  /* 0: 0x5D (AD0 float/high, default) */
    ILPS22QS_I2C_ADDR_ALT,      /* 1: 0x5C (AD0 to GND) */
};

#define NUM_INSTANCES  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    return (instance < NUM_INSTANCES) ? id_table[instance] : 0;
}

/* ---- Per-instance state ---- */

typedef struct {
    uint8_t ctrl_reg1;   /* Cached CTRL_REG1 (ODR + AVG) */
    uint8_t ctrl_reg2;   /* Cached CTRL_REG2 (FS, LPF, BDU) */
    uint8_t fs_mode;     /* 0 = 1260 hPa (4096 LSB/hPa), 1 = 4060 hPa (2048) */
} bp_state_t;

static bp_state_t bp_state[NUM_INSTANCES];

static bp_state_t *state_for(tile_t *tile)
{
    for (uint8_t i = 0; i < NUM_INSTANCES; i++)
        if (id_table[i] == tile->id) return &bp_state[i];
    return &bp_state[0];
}

/* ---- Portable memzero ---- */

static void memzero(void *p, uint8_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) *b++ = 0;
}

/* ---- Bus helpers ---- */

static void bp_write_reg(tile_t *tile, uint8_t reg, uint8_t val)
{
    tile->hal->i2c_write(tile->hal->handle, tile->id, reg, &val, 1);
}

static uint8_t bp_read_reg(tile_t *tile, uint8_t reg)
{
    uint8_t val = 0;
    tile->hal->i2c_read(tile->hal->handle, tile->id, reg, &val, 1);
    return val;
}

static void bp_read_regs(tile_t *tile, uint8_t reg, uint8_t *buf, uint16_t len)
{
    tile->hal->i2c_read(tile->hal->handle, tile->id, reg, buf, len);
}

/* ---- Internal helpers ---- */

static uint8_t build_ctrl_reg1(uint8_t odr, uint8_t avg)
{
    return (uint8_t)(((odr & 0x0F) << 3) | (avg & 0x07));
}

static uint8_t build_ctrl_reg2(uint8_t fs, uint8_t en_lpfp, uint8_t lfpf_cfg,
                               uint8_t bdu)
{
    uint8_t val = 0;
    if (fs)       val |= ILPS22QS_CTRL2_FS_MODE;
    if (lfpf_cfg) val |= ILPS22QS_CTRL2_LFPF_CFG;
    if (en_lpfp)  val |= ILPS22QS_CTRL2_EN_LPFP;
    if (bdu)      val |= ILPS22QS_CTRL2_BDU;
    return val;
}

static int32_t sign_extend_24(uint32_t raw)
{
    if (raw & 0x800000)
        return (int32_t)(raw | 0xFF000000);
    return (int32_t)raw;
}

/* ---- Lifecycle ---- */

uint8_t tile_sense_bp_find(tiles_pal_t *hal, uint8_t instance)
{
    uint8_t addr = resolve_id(instance);
    if (!addr) return 0;

    if (hal->i2c_is_ready(hal->handle, addr) != 0)
        return 0;

    uint8_t who = 0;
    hal->i2c_read(hal->handle, addr, ILPS22QS_REG_WHO_AM_I, &who, 1);
    return (who == ILPS22QS_WHO_AM_I_VALUE) ? 1 : 0;
}

void tile_sense_bp_init(tiles_pal_t *hal, uint8_t instance,
                        tile_t *tile, const sense_bp_cfg_t *cfg)
{
    memzero(tile, sizeof(tile_t));
    tile->hal = hal;
    tile->id  = resolve_id(instance);

    if (!tile->id) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_bp: invalid instance");
        return;
    }

    if (hal->i2c_is_ready(hal->handle, tile->id) != 0) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_bp: device not responding");
        return;
    }

    uint8_t who = bp_read_reg(tile, ILPS22QS_REG_WHO_AM_I);
    if (who != ILPS22QS_WHO_AM_I_VALUE) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_bp: WHO_AM_I mismatch");
        return;
    }

    bp_state_t *s = state_for(tile);
    memzero(s, sizeof(bp_state_t));

    /* Resolve config with defaults */
    uint8_t odr     = (cfg && cfg->odr) ? cfg->odr : SENSE_BP_ODR_25HZ;
    uint8_t avg     = (cfg) ? cfg->avg : SENSE_BP_AVG_4;
    uint8_t fs      = (cfg) ? cfg->fs  : SENSE_BP_FS_1260HPA;
    uint8_t en_lpfp = (cfg) ? cfg->lpf : 1;
    uint8_t lpf_bw  = (cfg) ? cfg->lpf_bw : SENSE_BP_LPF_ODR_4;
    uint8_t bdu     = (cfg) ? cfg->bdu : 1;

    /* If cfg is NULL, use defaults */
    if (!cfg) {
        en_lpfp = 1;
        bdu = 1;
    }

    s->fs_mode   = fs;
    s->ctrl_reg1 = build_ctrl_reg1(odr, avg);
    s->ctrl_reg2 = build_ctrl_reg2(fs, en_lpfp, lpf_bw, bdu);

    /* Disable AH/Qvar for lower power consumption (write 0x00 to CTRL_REG3,
       but keep IF_ADD_INC=1 for multi-byte reads) */
    bp_write_reg(tile, ILPS22QS_REG_CTRL_REG3, 0x01);

    /* Configure ODR and averaging */
    bp_write_reg(tile, ILPS22QS_REG_CTRL_REG1, s->ctrl_reg1);

    /* Configure FS, LPF, BDU */
    bp_write_reg(tile, ILPS22QS_REG_CTRL_REG2, s->ctrl_reg2);

    /* Wait for first measurement (max 1/ODR + margin) */
    hal->delay_ms(5);

    tile->state = TILE_STATE_READY;
}

void tile_sense_bp_sleep(tile_t *tile)
{
    bp_state_t *s = state_for(tile);
    /* Set ODR to power-down (0) while preserving AVG */
    uint8_t avg = s->ctrl_reg1 & 0x07;
    bp_write_reg(tile, ILPS22QS_REG_CTRL_REG1, build_ctrl_reg1(0, avg));
    tile->state = TILE_STATE_SLEEPING;
}

void tile_sense_bp_wake(tile_t *tile)
{
    bp_state_t *s = state_for(tile);
    /* Restore cached CTRL_REG1 (resumes ODR) */
    bp_write_reg(tile, ILPS22QS_REG_CTRL_REG1, s->ctrl_reg1);
    tile->hal->delay_ms(5);
    tile->state = TILE_STATE_READY;
}

void tile_sense_bp_reset(tile_t *tile)
{
    bp_write_reg(tile, ILPS22QS_REG_CTRL_REG2, ILPS22QS_CTRL2_SWRESET);
    tile->hal->delay_ms(5);
    tile->state = TILE_STATE_NONE;
}

/* ---- Configuration ---- */

void tile_sense_bp_set_odr(tile_t *tile, sense_bp_odr_t odr)
{
    bp_state_t *s = state_for(tile);
    uint8_t avg = s->ctrl_reg1 & 0x07;
    s->ctrl_reg1 = build_ctrl_reg1((uint8_t)odr, avg);
    bp_write_reg(tile, ILPS22QS_REG_CTRL_REG1, s->ctrl_reg1);
}

void tile_sense_bp_set_avg(tile_t *tile, sense_bp_avg_t avg)
{
    bp_state_t *s = state_for(tile);
    uint8_t odr = (s->ctrl_reg1 >> 3) & 0x0F;
    s->ctrl_reg1 = build_ctrl_reg1(odr, (uint8_t)avg);
    bp_write_reg(tile, ILPS22QS_REG_CTRL_REG1, s->ctrl_reg1);
}

void tile_sense_bp_set_fullscale(tile_t *tile, sense_bp_fs_t fs)
{
    bp_state_t *s = state_for(tile);
    s->fs_mode = (uint8_t)fs;
    if (fs)
        s->ctrl_reg2 |= ILPS22QS_CTRL2_FS_MODE;
    else
        s->ctrl_reg2 &= (uint8_t)~ILPS22QS_CTRL2_FS_MODE;
    bp_write_reg(tile, ILPS22QS_REG_CTRL_REG2, s->ctrl_reg2);
}

void tile_sense_bp_set_lpf(tile_t *tile, uint8_t enable, sense_bp_lpf_bw_t bw)
{
    bp_state_t *s = state_for(tile);
    if (enable)
        s->ctrl_reg2 |= ILPS22QS_CTRL2_EN_LPFP;
    else
        s->ctrl_reg2 &= (uint8_t)~ILPS22QS_CTRL2_EN_LPFP;

    if (bw)
        s->ctrl_reg2 |= ILPS22QS_CTRL2_LFPF_CFG;
    else
        s->ctrl_reg2 &= (uint8_t)~ILPS22QS_CTRL2_LFPF_CFG;

    bp_write_reg(tile, ILPS22QS_REG_CTRL_REG2, s->ctrl_reg2);
}

/* ---- Pressure data ---- */

int32_t tile_sense_bp_get_pressure_raw(tile_t *tile)
{
    uint8_t buf[3] = {0, 0, 0};
    bp_read_regs(tile, ILPS22QS_REG_PRESS_OUT_XL, buf, 3);
    uint32_t raw = ((uint32_t)buf[2] << 16) |
                   ((uint32_t)buf[1] << 8)  |
                   ((uint32_t)buf[0]);
    return sign_extend_24(raw);
}

int32_t tile_sense_bp_get_pressure_mhpa(tile_t *tile)
{
    bp_state_t *s = state_for(tile);
    int32_t raw = tile_sense_bp_get_pressure_raw(tile);

    /* Pressure (hPa) = raw / sensitivity
     * Mode 1: sensitivity = 4096 LSB/hPa → mhPa = raw * 1000 / 4096
     * Mode 2: sensitivity = 2048 LSB/hPa → mhPa = raw * 1000 / 2048
     *
     * To avoid overflow with 24-bit values (* 1000):
     * Mode 1: (raw * 125) / 512  (equivalent to raw * 1000 / 4096)
     * Mode 2: (raw * 125) / 256  (equivalent to raw * 1000 / 2048)
     */
    if (s->fs_mode == SENSE_BP_FS_4060HPA) {
        return (raw * 125) / 256;
    } else {
        return (raw * 125) / 512;
    }
}

/* ---- Temperature data ---- */

int16_t tile_sense_bp_get_temp_raw(tile_t *tile)
{
    uint8_t buf[2] = {0, 0};
    bp_read_regs(tile, ILPS22QS_REG_TEMP_OUT_L, buf, 2);
    return (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
}

int32_t tile_sense_bp_get_temp_cdeg(tile_t *tile)
{
    /* Sensitivity = 100 LSB/°C, so raw value IS centi-degrees */
    return (int32_t)tile_sense_bp_get_temp_raw(tile);
}

/* ---- One-shot mode ---- */

void tile_sense_bp_oneshot(tile_t *tile)
{
    bp_state_t *s = state_for(tile);
    bp_write_reg(tile, ILPS22QS_REG_CTRL_REG2,
                 s->ctrl_reg2 | ILPS22QS_CTRL2_ONESHOT);
}

/* ---- Status ---- */

uint8_t tile_sense_bp_get_status(tile_t *tile)
{
    return bp_read_reg(tile, ILPS22QS_REG_STATUS);
}

uint8_t tile_sense_bp_pressure_ready(tile_t *tile)
{
    return (bp_read_reg(tile, ILPS22QS_REG_STATUS) & ILPS22QS_STATUS_P_DA)
           ? 1 : 0;
}

uint8_t tile_sense_bp_temp_ready(tile_t *tile)
{
    return (bp_read_reg(tile, ILPS22QS_REG_STATUS) & ILPS22QS_STATUS_T_DA)
           ? 1 : 0;
}

/* ---- FIFO ---- */

void tile_sense_bp_set_fifo_mode(tile_t *tile, sense_bp_fifo_mode_t mode)
{
    uint8_t val;
    switch (mode) {
    case SENSE_BP_FIFO_BYPASS:     val = 0x00; break;
    case SENSE_BP_FIFO_FIFO:       val = 0x01; break;
    case SENSE_BP_FIFO_CONTINUOUS: val = 0x02; break;
    case SENSE_BP_FIFO_BYP2FIFO:   val = 0x05; break;
    case SENSE_BP_FIFO_BYP2CONT:   val = 0x06; break;
    case SENSE_BP_FIFO_CONT2FIFO:  val = 0x07; break;
    default:                       val = 0x00; break;
    }
    bp_write_reg(tile, ILPS22QS_REG_FIFO_CTRL, val);
}

void tile_sense_bp_set_fifo_watermark(tile_t *tile, uint8_t watermark)
{
    bp_write_reg(tile, ILPS22QS_REG_FIFO_WTM, watermark & 0x7F);
}

uint8_t tile_sense_bp_get_fifo_level(tile_t *tile)
{
    return bp_read_reg(tile, ILPS22QS_REG_FIFO_STATUS1);
}

uint8_t tile_sense_bp_get_fifo_status(tile_t *tile)
{
    return bp_read_reg(tile, ILPS22QS_REG_FIFO_STATUS2);
}

int32_t tile_sense_bp_read_fifo_raw(tile_t *tile)
{
    uint8_t buf[3] = {0, 0, 0};
    bp_read_regs(tile, ILPS22QS_REG_FIFO_DATA_OUT_PRESS_XL, buf, 3);
    uint32_t raw = ((uint32_t)buf[2] << 16) |
                   ((uint32_t)buf[1] << 8)  |
                   ((uint32_t)buf[0]);
    return sign_extend_24(raw);
}

uint8_t tile_sense_bp_read_fifo_batch(tile_t *tile, int32_t *buf,
                                      uint8_t count)
{
    uint8_t level = tile_sense_bp_get_fifo_level(tile);
    if (count > level) count = level;
    for (uint8_t i = 0; i < count; i++)
        buf[i] = tile_sense_bp_read_fifo_raw(tile);
    return count;
}

/* ---- Interrupt / threshold ---- */

void tile_sense_bp_set_threshold_hpa(tile_t *tile, uint16_t ths_hpa)
{
    bp_state_t *s = state_for(tile);
    /* THS_P = threshold(hPa) * 16 for mode 1, * 8 for mode 2; a 15-bit
     * field, so saturate rather than wrap. */
    uint32_t ths;
    if (s->fs_mode == SENSE_BP_FS_4060HPA)
        ths = (uint32_t)ths_hpa * 8u;
    else
        ths = (uint32_t)ths_hpa * 16u;
    if (ths > 0x7FFFu) ths = 0x7FFFu;

    bp_write_reg(tile, ILPS22QS_REG_THS_P_L, (uint8_t)(ths & 0xFF));
    bp_write_reg(tile, ILPS22QS_REG_THS_P_H, (uint8_t)((ths >> 8) & 0x7F));
}

void tile_sense_bp_set_interrupt_cfg(tile_t *tile, uint8_t cfg)
{
    bp_write_reg(tile, ILPS22QS_REG_INTERRUPT_CFG, cfg);
}

uint8_t tile_sense_bp_get_int_source(tile_t *tile)
{
    return bp_read_reg(tile, ILPS22QS_REG_INT_SOURCE);
}

uint8_t tile_sense_bp_is_boot_complete(tile_t *tile)
{
    /* BOOT_ON (bit 7 of INT_SOURCE) reads 1 while the NVM-trim
     * reload is in flight, 0 once the chip is ready. The BOOT_ON
     * bit isn't cleared on read (only IA / PH / PL are), so this
     * call is safe to spin on. */
    uint8_t src = bp_read_reg(tile, ILPS22QS_REG_INT_SOURCE);
    return (src & ILPS22QS_INT_SRC_BOOT_ON) ? 0 : 1;
}

/* ---- Reference / offset calibration ---- */

void tile_sense_bp_set_autozero(tile_t *tile)
{
    uint8_t val = bp_read_reg(tile, ILPS22QS_REG_INTERRUPT_CFG);
    val |= ILPS22QS_INTCFG_AUTOZERO;
    bp_write_reg(tile, ILPS22QS_REG_INTERRUPT_CFG, val);
}

void tile_sense_bp_reset_autozero(tile_t *tile)
{
    uint8_t val = bp_read_reg(tile, ILPS22QS_REG_INTERRUPT_CFG);
    val |= ILPS22QS_INTCFG_RESET_AZ;
    bp_write_reg(tile, ILPS22QS_REG_INTERRUPT_CFG, val);
}

void tile_sense_bp_set_autorefp(tile_t *tile)
{
    uint8_t val = bp_read_reg(tile, ILPS22QS_REG_INTERRUPT_CFG);
    val |= ILPS22QS_INTCFG_AUTOREFP;
    bp_write_reg(tile, ILPS22QS_REG_INTERRUPT_CFG, val);
}

void tile_sense_bp_reset_autorefp(tile_t *tile)
{
    uint8_t val = bp_read_reg(tile, ILPS22QS_REG_INTERRUPT_CFG);
    val |= ILPS22QS_INTCFG_RESET_ARP;
    bp_write_reg(tile, ILPS22QS_REG_INTERRUPT_CFG, val);
}

void tile_sense_bp_set_pressure_offset(tile_t *tile, int16_t offset)
{
    uint16_t uoff = (uint16_t)offset;
    bp_write_reg(tile, ILPS22QS_REG_RPDS_L, (uint8_t)(uoff & 0xFF));
    bp_write_reg(tile, ILPS22QS_REG_RPDS_H, (uint8_t)((uoff >> 8) & 0xFF));
}

int16_t tile_sense_bp_get_ref_pressure(tile_t *tile)
{
    uint8_t buf[2] = {0, 0};
    bp_read_regs(tile, ILPS22QS_REG_REF_P_L, buf, 2);
    return (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
}

/* ---- Tier-2 helpers ---- */

/* Convert cached ODR field to a poll period in milliseconds.
 * Returns 0 for power-down (caller should fall back to a default). */
static uint16_t odr_period_ms(uint8_t odr_field)
{
    switch (odr_field) {
    case SENSE_BP_ODR_1HZ:   return 1000;
    case SENSE_BP_ODR_4HZ:   return 250;
    case SENSE_BP_ODR_10HZ:  return 100;
    case SENSE_BP_ODR_25HZ:  return 40;
    case SENSE_BP_ODR_50HZ:  return 20;
    case SENSE_BP_ODR_75HZ:  return 14;  /* ~13.3, round up */
    case SENSE_BP_ODR_100HZ: return 10;
    case SENSE_BP_ODR_200HZ: return 5;
    default:                 return 0;
    }
}

int32_t tile_sense_bp_read_altitude_mm(tile_t *tile, uint32_t sea_level_pa)
{
    /* Live pressure in mhPa → convert to pascals (1 hPa = 100 Pa,
     * so Pa = mhPa / 10). Use signed 32-bit math throughout. */
    int32_t mhpa = tile_sense_bp_get_pressure_mhpa(tile);
    int32_t pressure_pa = mhpa / 10;

    /* Linear approximation around the reference: 8.43 mm per Pa of
     * pressure decrease. h_mm = 8430 * (P0 - P) / 100. Bounded easily
     * by int32 for any sane ILPS22QS reading. */
    int32_t delta_pa = (int32_t)sea_level_pa - pressure_pa;
    return (8430 * delta_pa) / 100;
}

uint8_t tile_sense_bp_wait_for_pressure_change(tile_t *tile,
                                               uint16_t threshold_hpa,
                                               uint32_t timeout_ms)
{
    bp_state_t *s = state_for(tile);

    /* Capture the baseline at call time. */
    int32_t baseline_mhpa = tile_sense_bp_get_pressure_mhpa(tile);

    /* Threshold in mhPa (16-bit hPa * 1000 fits comfortably in int32). */
    int32_t threshold_mhpa = (int32_t)threshold_hpa * 1000;

    /* Derive a polling cadence from the cached ODR. Fall back to 10 ms
     * if the device is in power-down or the field is unrecognised. */
    uint8_t odr_field = (s->ctrl_reg1 >> 3) & 0x0F;
    uint16_t period_ms = odr_period_ms(odr_field);
    if (period_ms == 0) period_ms = 10;

    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        int32_t live = tile_sense_bp_get_pressure_mhpa(tile);
        int32_t diff = live - baseline_mhpa;
        if (diff < 0) diff = -diff;
        if (diff >= threshold_mhpa) return 1;

        tile->hal->delay_ms(period_ms);
        elapsed += period_ms;
    }

    return 0;
}
