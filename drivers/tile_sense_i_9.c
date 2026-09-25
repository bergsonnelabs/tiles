/**
 * @file   tile_sense_i_9.c
 * @brief  9-DOF IMU driver implementation (ICM-20948 + AK09916).
 *
 * References:
 *   - ICM-20948 datasheet DS-000189 rev 1.3 (TDK InvenSense)
 *   - AK09916 datasheet 015007392-E-02 (Asahi Kasei)
 *   - TDK AN-000150: ICM-20948 self-test procedure
 *
 * AK09916 access:
 *   The driver enables INT_PIN_CFG.BYPASS_EN at init so the AK09916
 *   appears directly on the host I2C bus at 0x0C. This is simpler
 *   than the I2C-master path used in some reference drivers and is
 *   why the "sensor hub for external aux sensors" gap remains open.
 */

#include "tile_sense_i_9.h"
#include "tile_sense_i_9_dmp3.h"
#include <stddef.h>
#include <string.h>

/* -------------------------------------------------------------- */
/* Instance mapping                                                */
/* -------------------------------------------------------------- */

static const uint8_t id_table[] = {
    ICM20948_I2C_ADDR_DEFAULT,   /* instance 0 — pad 2 floating (0x69) */
    ICM20948_I2C_ADDR_ALT,       /* instance 1 — pad 2 to GND  (0x68) */
};

#define ID_TABLE_LEN  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    if (instance >= ID_TABLE_LEN) return 0x00;
    return id_table[instance];
}

/* -------------------------------------------------------------- */
/* Per-instance driver state                                       */
/*                                                                  */
/* Today: tracks DMP firmware-load status + last DMP-RAM bank for  */
/* the bank-select-skip optimization. Will grow as Phase 2/3/4     */
/* land more DMP-side state (active features, output rate, etc.). */
/* -------------------------------------------------------------- */

typedef struct {
    uint8_t dmp_loaded;        /**< 1 once dmp_load() has succeeded since reset. */
    uint8_t dmp_active;        /**< 1 between dmp_start_quat9() and dmp_stop().
                                    When set, AK09916 is on the chip's internal
                                    I2C master — bypass-mode mag helpers must
                                    NOT be used. */
    uint8_t last_dmp_bank;     /**< Last MEM_BANK_SEL value written; skip the
                                    bus write when the next access stays in the
                                    same bank. 0xFF = "no bank selected yet". */
} icm_state_t;

static icm_state_t icm_state[ID_TABLE_LEN];

static icm_state_t *state_for(tile_t *tile)
{
    for (uint8_t i = 0; i < ID_TABLE_LEN; i++) {
        if (id_table[i] == tile->id) return &icm_state[i];
    }
    return &icm_state[0];
}

/* -------------------------------------------------------------- */
/* Private helpers                                                 */
/* -------------------------------------------------------------- */

static void icm_write(tile_t* tile, uint8_t reg, uint8_t value)
{
    tile->hal->i2c_write(tile->hal->handle, tile->id, reg, &value, 1);
}

static uint8_t icm_read1(tile_t* tile, uint8_t reg)
{
    uint8_t v = 0;
    tile->hal->i2c_read(tile->hal->handle, tile->id, reg, &v, 1);
    return v;
}

static void icm_read(tile_t* tile, uint8_t reg, uint8_t* data, uint16_t len)
{
    tile->hal->i2c_read(tile->hal->handle, tile->id, reg, data, len);
}

static void icm_modify(tile_t* tile, uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t r = icm_read1(tile, reg);
    r = (uint8_t)((r & ~mask) | (value & mask));
    icm_write(tile, reg, r);
}

static void ak_write(tile_t* tile, uint8_t reg, uint8_t value)
{
    tile->hal->i2c_write(tile->hal->handle, AK09916_I2C_ADDR, reg, &value, 1);
}

static void ak_read(tile_t* tile, uint8_t reg, uint8_t* data, uint16_t len)
{
    tile->hal->i2c_read(tile->hal->handle, AK09916_I2C_ADDR, reg, data, len);
}

static uint8_t ak_read1(tile_t* tile, uint8_t reg)
{
    uint8_t v = 0;
    tile->hal->i2c_read(tile->hal->handle, AK09916_I2C_ADDR, reg, &v, 1);
    return v;
}

static void set_bank(tile_t* tile, uint8_t bank)
{
    icm_write(tile, ICM20948_REG_BANK_SEL, (uint8_t)(bank << 4));
}

/** @brief  Swap bytes in a buffer of int16_t values (big-endian → native). */
static void swap16_buf(int16_t* buf, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        buf[i] = (int16_t)__builtin_bswap16((uint16_t)buf[i]);
    }
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

/** @brief  AK09916 little-endian 16-bit pair (HXL then HXH). */
static int16_t le16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[1] << 8) | p[0]);
}

/* -------------------------------------------------------------- */
/* Public API — lifecycle                                          */
/* -------------------------------------------------------------- */

uint8_t tile_sense_i_9_find(tiles_pal_t* hal, uint8_t instance)
{
    uint8_t id = resolve_id(instance);
    if (id == 0x00) return 0;
    return (hal->i2c_is_ready(hal->handle, id) == 0) ? 1 : 0;
}

void tile_sense_i_9_init(tiles_pal_t* hal, uint8_t instance, tile_t* tile,
                         const sense_i_9_cfg_t *cfg)
{
    (void)cfg;  /* Reserved for future use */
    tile->hal      = NULL;
    tile->id       = 0;
    tile->state    = TILE_STATE_NONE;
    tile->flags    = 0;
    tile->callback = NULL;
    tile->cb_ctx   = NULL;

    uint8_t id = resolve_id(instance);
    if (id == 0x00) {
        TILE_ON_ERROR(tile, "init: invalid instance");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    tile->hal = hal;
    tile->id  = id;

    /* Per-instance state — cleared on every init so DMP load status
     * never carries over from a prior session even if the chip wasn't
     * power-cycled (the software-reset below clears DMP RAM, so the
     * driver-side flag has to follow). */
    icm_state_t *s = state_for(tile);
    s->dmp_loaded     = 0;
    s->dmp_active     = 0;
    s->last_dmp_bank  = 0xFF;

    /* Verify device is on bus */
    if (hal->i2c_is_ready(hal->handle, id) != 0) {
        TILE_ON_ERROR(tile, "init: device not found on bus");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    /* Verify chip identity */
    set_bank(tile, ICM20948_BANK_0);
    if (icm_read1(tile, ICM20948_REG_WHOAMI) != ICM20948_WHOAMI_DEFAULT) {
        TILE_ON_ERROR(tile, "init: unexpected chip ID");
        tile->state = TILE_STATE_ERROR;
        return;
    }

    /* Software reset */
    icm_write(tile, ICM20948_REG_PWR_MGMT_1, 1 << 7);
    hal->delay_ms(50);

    /* Wake: auto clock select */
    icm_write(tile, ICM20948_REG_PWR_MGMT_1, 0x01);

    /* Enable all accel + gyro axes */
    icm_write(tile, ICM20948_REG_PWR_MGMT_2, 0x00);

    /* Enable I2C bypass so we can talk to the AK09916 directly */
    icm_write(tile, ICM20948_REG_INT_PIN_CFG, 0x02);

    /* Return to bank 0 */
    set_bank(tile, ICM20948_BANK_0);

    /* Initialize magnetometer: continuous 100 Hz */
    ak_write(tile, AK09916_REG_CNTL3, 0x01);  /* soft reset */
    hal->delay_ms(1);
    ak_write(tile, AK09916_REG_CNTL2, SENSE_I_9_MAG_CONTINUOUS_100HZ);

    tile->state = TILE_STATE_READY;
}

uint8_t tile_sense_i_9_data_ready(tile_t* tile)
{
    uint8_t status = icm_read1(tile, ICM20948_REG_INT_STATUS_1);
    return (status & 0x01) ? 1 : 0;  /* RAW_DATA_0_RDY_INT */
}

void tile_sense_i_9_set_accel_range(tile_t* tile, sense_i_9_accel_range_t range)
{
    /* ACCEL_CONFIG: [5:3] DLPFCFG, [2:1] FS_SEL, [0] FCHOICE (DS-000189
     * §10.15). Touch only FS_SEL: a whole-register write cleared FCHOICE,
     * bypassing the DLPF and the sample-rate divider (4.5 kHz, unfiltered). */
    set_bank(tile, ICM20948_BANK_2);
    icm_modify(tile, ICM20948_REG_ACCEL_CONFIG, 0x06, (uint8_t)range);
    set_bank(tile, ICM20948_BANK_0);
}

void tile_sense_i_9_set_gyro_range(tile_t* tile, sense_i_9_gyro_range_t range)
{
    /* GYRO_CONFIG_1: [5:3] DLPFCFG, [2:1] FS_SEL, [0] FCHOICE (§10.2).
     * Same read-modify-write as the accel setter. */
    set_bank(tile, ICM20948_BANK_2);
    icm_modify(tile, ICM20948_REG_GYRO_CONFIG, 0x06, (uint8_t)range);
    set_bank(tile, ICM20948_BANK_0);
}

void tile_sense_i_9_set_mag_mode(tile_t* tile, sense_i_9_mag_mode_t mode)
{
    /* Datasheet §9.3: must transit through power-down for 100 µs (Twait)
     * before entering a new mode. ak_write of 0x00 then the new mode. */
    ak_write(tile, AK09916_REG_CNTL2, SENSE_I_9_MAG_POWER_DOWN);
    tile->hal->delay_ms(1);
    ak_write(tile, AK09916_REG_CNTL2, (uint8_t)mode);
}

void tile_sense_i_9_set_accel_odr(tile_t* tile, uint16_t divider)
{
    uint8_t hi = (uint8_t)((divider >> 8) & 0x0F);
    uint8_t lo = (uint8_t)(divider & 0xFF);

    set_bank(tile, ICM20948_BANK_2);
    icm_write(tile, ICM20948_REG_ACCEL_SMPLRT_H, hi);
    icm_write(tile, ICM20948_REG_ACCEL_SMPLRT_L, lo);
    set_bank(tile, ICM20948_BANK_0);
}

void tile_sense_i_9_set_gyro_odr(tile_t* tile, uint8_t divider)
{
    set_bank(tile, ICM20948_BANK_2);
    icm_write(tile, ICM20948_REG_GYRO_SMPLRT, divider);
    set_bank(tile, ICM20948_BANK_0);
}

void tile_sense_i_9_get_raw_accels(tile_t* tile, int16_t* buffer)
{
    icm_read(tile, ICM20948_REG_ACCEL_X_H, (uint8_t*)buffer, 6);
    swap16_buf(buffer, 3);
}

void tile_sense_i_9_get_raw_gyros(tile_t* tile, int16_t* buffer)
{
    icm_read(tile, ICM20948_REG_GYRO_X_H, (uint8_t*)buffer, 6);
    swap16_buf(buffer, 3);
}

void tile_sense_i_9_get_raw_6dof(tile_t* tile, int16_t* buffer)
{
    icm_read(tile, ICM20948_REG_ACCEL_X_H, (uint8_t*)buffer, 12);
    swap16_buf(buffer, 6);
}

void tile_sense_i_9_get_raw_mags(tile_t* tile, int16_t* buffer)
{
    uint8_t st2;

    /* AK09916 data registers are little-endian — no swap needed */
    ak_read(tile, AK09916_REG_HXL, (uint8_t*)buffer, 6);

    /* Reading ST2 releases the data lock for the next measurement */
    ak_read(tile, AK09916_REG_ST2, &st2, 1);
}

uint8_t tile_sense_i_9_mag_overflowed(tile_t* tile)
{
    /* Reading ST2 also releases the AK09916 data lock — same end-effect
     * as get_raw_mags() w.r.t. the chip. */
    return (ak_read1(tile, AK09916_REG_ST2) & AK09916_ST2_HOFL) ? 1 : 0;
}

int16_t tile_sense_i_9_get_temperature(tile_t* tile)
{
    int16_t raw;
    icm_read(tile, ICM20948_REG_TEMP_H, (uint8_t*)&raw, 2);
    raw = (int16_t)__builtin_bswap16((uint16_t)raw);
    return raw;
}

void tile_sense_i_9_sleep(tile_t* tile)
{
    set_bank(tile, ICM20948_BANK_0);
    icm_write(tile, ICM20948_REG_PWR_MGMT_1, 0x41);  /* SLEEP + auto clock */
    tile->state = TILE_STATE_SLEEPING;
}

void tile_sense_i_9_wake(tile_t* tile)
{
    set_bank(tile, ICM20948_BANK_0);
    icm_write(tile, ICM20948_REG_PWR_MGMT_1, 0x01);  /* clear SLEEP, auto clock */
    tile->state = TILE_STATE_READY;
}

void tile_sense_i_9_reset(tile_t* tile)
{
    set_bank(tile, ICM20948_BANK_0);
    icm_write(tile, ICM20948_REG_PWR_MGMT_1, 1 << 7);
    tile->hal->delay_ms(50);
    /* Software reset clears DMP RAM as a side-effect — drop the
     * driver-side flag so dmp_is_loaded() reflects the new reality.
     * The caller has to re-issue dmp_load() before reading DMP outputs. */
    icm_state_t *s = state_for(tile);
    s->dmp_loaded    = 0;
    s->dmp_active    = 0;
    s->last_dmp_bank = 0xFF;
    tile->state = TILE_STATE_NONE;
}

/* -------------------------------------------------------------- */
/* Interrupt source configuration                                  */
/* -------------------------------------------------------------- */

void tile_sense_i_9_int_config(tile_t* tile, uint8_t flags)
{
    /* Mask off the BYPASS_EN bit (bit 1) and INT_ANYRD_2CLEAR if not
     * provided in flags — preserve the bypass bit so the AK09916 remains
     * reachable. Flags supply bits 7,6,5,4 directly. */
    set_bank(tile, ICM20948_BANK_0);
    uint8_t cur = icm_read1(tile, ICM20948_REG_INT_PIN_CFG);
    uint8_t keep = cur & 0x02;  /* BYPASS_EN preserved */
    icm_write(tile, ICM20948_REG_INT_PIN_CFG,
              (uint8_t)((flags & 0xF0) | keep));
}

void tile_sense_i_9_int_data_ready(tile_t* tile, uint8_t enabled)
{
    set_bank(tile, ICM20948_BANK_0);
    /* INT_ENABLE_1 bit 0 = RAW_DATA_0_RDY_EN */
    icm_modify(tile, ICM20948_REG_INT_ENABLE_1, 0x01, enabled ? 0x01 : 0x00);
}

void tile_sense_i_9_int_wom(tile_t* tile, uint8_t enabled)
{
    set_bank(tile, ICM20948_BANK_0);
    /* INT_ENABLE bit 3 = WOM_INT_EN */
    icm_modify(tile, ICM20948_REG_INT_ENABLE,
               ICM20948_INTE_WOM_INT_EN,
               enabled ? ICM20948_INTE_WOM_INT_EN : 0x00);
}

void tile_sense_i_9_int_fifo_overflow(tile_t* tile, uint8_t enabled)
{
    set_bank(tile, ICM20948_BANK_0);
    /* INT_ENABLE_2 bits [4:0] = FIFO_OVERFLOW_EN per sensor.
     * For a single FIFO config any of the bits triggers; enable all 5. */
    icm_modify(tile, ICM20948_REG_INT_ENABLE_2, 0x1F, enabled ? 0x1F : 0x00);
}

void tile_sense_i_9_int_fifo_watermark(tile_t* tile, uint8_t enabled)
{
    set_bank(tile, ICM20948_BANK_0);
    icm_modify(tile, ICM20948_REG_INT_ENABLE_3, 0x1F, enabled ? 0x1F : 0x00);
}

uint8_t tile_sense_i_9_get_int_status(tile_t* tile)
{
    set_bank(tile, ICM20948_BANK_0);
    return icm_read1(tile, ICM20948_REG_INT_STATUS);
}

uint8_t tile_sense_i_9_get_int_status_fifo_overflow(tile_t* tile)
{
    set_bank(tile, ICM20948_BANK_0);
    return icm_read1(tile, ICM20948_REG_INT_STATUS_2) & 0x1F;
}

uint8_t tile_sense_i_9_get_int_status_fifo_watermark(tile_t* tile)
{
    set_bank(tile, ICM20948_BANK_0);
    return icm_read1(tile, ICM20948_REG_INT_STATUS_3) & 0x1F;
}

/* -------------------------------------------------------------- */
/* Wake-on-Motion                                                  */
/* -------------------------------------------------------------- */

void tile_sense_i_9_wom_config(tile_t* tile, uint16_t thr_mg,
                               sense_i_9_wom_mode_t mode)
{
    /* Threshold register is 8-bit, 4 mg/LSB. Clamp at 0xFF (1020 mg). */
    uint16_t lsb = (thr_mg + 2) / 4;     /* round-to-nearest */
    if (lsb > 0xFF) lsb = 0xFF;

    set_bank(tile, ICM20948_BANK_2);
    icm_write(tile, ICM20948_REG_ACCEL_WOM_THR, (uint8_t)lsb);
    /* ACCEL_INTEL_CTRL bit 0 selects compare mode (0=initial, 1=previous).
     * Don't enable here — that's wom_enable's job — preserve INTEL_EN bit. */
    icm_modify(tile, ICM20948_REG_ACCEL_INTEL_CTRL, 0x01,
               (mode == SENSE_I_9_WOM_VS_PREVIOUS) ? 0x01 : 0x00);
    set_bank(tile, ICM20948_BANK_0);
}

void tile_sense_i_9_wom_enable(tile_t* tile)
{
    set_bank(tile, ICM20948_BANK_2);
    /* ACCEL_INTEL_CTRL bit 1 = ACCEL_INTEL_EN */
    icm_modify(tile, ICM20948_REG_ACCEL_INTEL_CTRL, 0x02, 0x02);
    set_bank(tile, ICM20948_BANK_0);
}

void tile_sense_i_9_wom_disable(tile_t* tile)
{
    set_bank(tile, ICM20948_BANK_2);
    icm_modify(tile, ICM20948_REG_ACCEL_INTEL_CTRL, 0x02, 0x00);
    set_bank(tile, ICM20948_BANK_0);
}

/* -------------------------------------------------------------- */
/* FIFO                                                            */
/* -------------------------------------------------------------- */

void tile_sense_i_9_fifo_config(tile_t* tile, sense_i_9_fifo_mode_t mode,
                                uint8_t accel, uint8_t gyro, uint8_t temp)
{
    set_bank(tile, ICM20948_BANK_0);

    /* FIFO_MODE: snapshot vs stream (any non-zero in [4:0] = snapshot). */
    icm_write(tile, ICM20948_REG_FIFO_MODE,
              (mode == SENSE_I_9_FIFO_SNAPSHOT) ? 0x1F : 0x00);

    /* Wire up sensor sources. FIFO_EN_2 [4:0] selects accel + gyro X/Y/Z + temp. */
    uint8_t en2 = 0;
    if (accel) en2 |= (1 << 4);
    if (gyro)  en2 |= (1 << 3) | (1 << 2) | (1 << 1);   /* GZ, GY, GX */
    if (temp)  en2 |= (1 << 0);
    icm_write(tile, ICM20948_REG_FIFO_EN_2, en2);
    icm_write(tile, ICM20948_REG_FIFO_EN_1, 0x00);  /* no aux slaves */

    /* Toggle FIFO_EN in USER_CTRL */
    uint8_t any = (accel | gyro | temp) ? 1 : 0;
    icm_modify(tile, ICM20948_USER_CTRL, ICM20948_UC_FIFO_EN,
               any ? ICM20948_UC_FIFO_EN : 0x00);

    /* Reset FIFO so config takes effect cleanly: assert and de-assert. */
    icm_write(tile, ICM20948_REG_FIFO_RST, 0x1F);
    icm_write(tile, ICM20948_REG_FIFO_RST, 0x00);
}

void tile_sense_i_9_fifo_flush(tile_t* tile)
{
    set_bank(tile, ICM20948_BANK_0);
    icm_write(tile, ICM20948_REG_FIFO_RST, 0x1F);
    icm_write(tile, ICM20948_REG_FIFO_RST, 0x00);
}

uint16_t tile_sense_i_9_fifo_count(tile_t* tile)
{
    uint8_t buf[2];
    set_bank(tile, ICM20948_BANK_0);
    /* Datasheet §8.60: reading COUNTH latches both bytes. We perform a
     * burst read starting at COUNTH which reads COUNTH then COUNTL. */
    icm_read(tile, ICM20948_REG_FIFO_COUNTH, buf, 2);
    return (uint16_t)(((uint16_t)(buf[0] & 0x1F) << 8) | buf[1]);
}

uint8_t tile_sense_i_9_fifo_read_packet(tile_t* tile,
                                        sense_i_9_fifo_packet_t* pkt)
{
    if (tile_sense_i_9_fifo_count(tile) < 12) return 0;

    uint8_t raw[12];
    icm_read(tile, ICM20948_REG_FIFO_R_W, raw, 12);

    /* Big-endian, accel first, gyro after — matches FIFO_EN_2 enable order. */
    pkt->accel[0] = be16(&raw[0]);
    pkt->accel[1] = be16(&raw[2]);
    pkt->accel[2] = be16(&raw[4]);
    pkt->gyro[0]  = be16(&raw[6]);
    pkt->gyro[1]  = be16(&raw[8]);
    pkt->gyro[2]  = be16(&raw[10]);
    return 1;
}

void tile_sense_i_9_fifo_read_packet_flat(tile_t* tile, int32_t* out)
{
    sense_i_9_fifo_packet_t pkt = {0};
    if (!tile_sense_i_9_fifo_read_packet(tile, &pkt)) return;
    if (!out) return;
    out[0] = (int32_t)pkt.accel[0];
    out[1] = (int32_t)pkt.accel[1];
    out[2] = (int32_t)pkt.accel[2];
    out[3] = (int32_t)pkt.gyro[0];
    out[4] = (int32_t)pkt.gyro[1];
    out[5] = (int32_t)pkt.gyro[2];
}

/* -------------------------------------------------------------- */
/* Self-test                                                       */
/* -------------------------------------------------------------- */

/* TDK app-note AN-000150 self-test pass band: 50%–150% of factory ref. */
static uint8_t st_in_band(int32_t response, int32_t reference)
{
    /* If the factory reference is zero (rare) the test is inconclusive
     * — accept any non-zero response of the same sign as a pass. */
    if (reference == 0) return (response != 0) ? 1 : 0;
    int32_t lo = reference / 2;
    int32_t hi = reference + (reference / 2);
    if (lo > hi) { int32_t t = lo; lo = hi; hi = t; }
    return (response >= lo && response <= hi) ? 1 : 0;
}

uint8_t tile_sense_i_9_self_test(tile_t* tile,
                                 uint8_t* accel_pass, uint8_t* gyro_pass)
{
    if (accel_pass) *accel_pass = 0;
    if (gyro_pass)  *gyro_pass  = 0;
    if (tile->state == TILE_STATE_ERROR || tile->hal == NULL) return 0;

    tiles_pal_t* hal = tile->hal;

    /* --- Capture factory reference values from Bank 1 --- */
    set_bank(tile, ICM20948_BANK_1);
    uint8_t ref[6];
    ref[0] = icm_read1(tile, ICM20948_B1_SELF_TEST_X_GYRO);
    ref[1] = icm_read1(tile, ICM20948_B1_SELF_TEST_Y_GYRO);
    ref[2] = icm_read1(tile, ICM20948_B1_SELF_TEST_Z_GYRO);
    ref[3] = icm_read1(tile, ICM20948_B1_SELF_TEST_X_ACCEL);
    ref[4] = icm_read1(tile, ICM20948_B1_SELF_TEST_Y_ACCEL);
    ref[5] = icm_read1(tile, ICM20948_B1_SELF_TEST_Z_ACCEL);

    /* --- Set up known-good config: ±2 g, ±250 dps, 1 kHz, DLPF on --- */
    set_bank(tile, ICM20948_BANK_2);
    icm_write(tile, ICM20948_REG_GYRO_CONFIG,  0x01);  /* DLPF 1, ±250 dps */
    icm_write(tile, ICM20948_REG_ACCEL_CONFIG, 0x01);  /* DLPF 1, ±2 g */
    icm_write(tile, ICM20948_REG_GYRO_SMPLRT,  0x00);
    icm_write(tile, ICM20948_REG_ACCEL_SMPLRT_H, 0x00);
    icm_write(tile, ICM20948_REG_ACCEL_SMPLRT_L, 0x00);
    set_bank(tile, ICM20948_BANK_0);
    hal->delay_ms(50);

    /* --- Sample with self-test OFF, average 16 samples --- */
    int32_t base_acc[3] = {0,0,0}, base_gyro[3] = {0,0,0};
    int16_t s[6];
    for (uint8_t i = 0; i < 16; i++) {
        hal->delay_ms(2);
        tile_sense_i_9_get_raw_6dof(tile, s);
        base_acc[0] += s[0]; base_acc[1] += s[1]; base_acc[2] += s[2];
        base_gyro[0] += s[3]; base_gyro[1] += s[4]; base_gyro[2] += s[5];
    }
    for (uint8_t k = 0; k < 3; k++) { base_acc[k] /= 16; base_gyro[k] /= 16; }

    /* --- Engage self-test on all axes --- */
    set_bank(tile, ICM20948_BANK_2);
    /* ACCEL_CONFIG_2: bits [4:2] = AX/AY/AZ_ST_EN, [1:0] DEC3 = 0 */
    icm_write(tile, ICM20948_REG_ACCEL_CONFIG_2, 0x1C);
    /* Gyro self-test enables live in GYRO_CONFIG_2 [5:3] = X/Y/Z GYRO_CTEN
     * (DS-000189 §10.3). GYRO_CONFIG_1 bits [7:6] are reserved. */
    icm_write(tile, ICM20948_REG_GYRO_CONFIG_2, 0x38);
    set_bank(tile, ICM20948_BANK_0);
    hal->delay_ms(50);  /* Datasheet: ≥ 20 ms for the response to settle */

    /* --- Sample with self-test ON --- */
    int32_t st_acc[3] = {0,0,0}, st_gyro[3] = {0,0,0};
    for (uint8_t i = 0; i < 16; i++) {
        hal->delay_ms(2);
        tile_sense_i_9_get_raw_6dof(tile, s);
        st_acc[0] += s[0]; st_acc[1] += s[1]; st_acc[2] += s[2];
        st_gyro[0] += s[3]; st_gyro[1] += s[4]; st_gyro[2] += s[5];
    }
    for (uint8_t k = 0; k < 3; k++) { st_acc[k] /= 16; st_gyro[k] /= 16; }

    /* --- Disengage self-test --- */
    set_bank(tile, ICM20948_BANK_2);
    icm_write(tile, ICM20948_REG_ACCEL_CONFIG_2, 0x00);
    icm_write(tile, ICM20948_REG_GYRO_CONFIG_2, 0x00);
    set_bank(tile, ICM20948_BANK_0);

    /* --- Compute responses (LSBs at ±2 g / ±250 dps) and compare --- */
    /* Factory reference values are stored in raw "self-test" units that
     * scale identically to the response at the test config above; the
     * 50%–150% band per AN-000150 captures normal device variation. */
    uint8_t apass = 0, gpass = 0;
    for (uint8_t k = 0; k < 3; k++) {
        int32_t a_resp = st_acc[k]  - base_acc[k];
        int32_t g_resp = st_gyro[k] - base_gyro[k];
        if (st_in_band(a_resp, (int32_t)(int8_t)ref[3 + k] << 7)) apass |= (1 << k);
        if (st_in_band(g_resp, (int32_t)(int8_t)ref[k]     << 7)) gpass |= (1 << k);
    }

    if (accel_pass) *accel_pass = apass;
    if (gyro_pass)  *gyro_pass  = gpass;

    return ((apass == 0x07) && (gpass == 0x07)) ? 1 : 0;
}

/* -------------------------------------------------------------- */
/* AK09916 self-test                                               */
/* -------------------------------------------------------------- */

uint8_t tile_sense_i_9_mag_self_test(tile_t* tile)
{
    tiles_pal_t* hal = tile->hal;

    /* §9.4.4.1: power-down → self-test → poll DRDY → read data */
    ak_write(tile, AK09916_REG_CNTL2, SENSE_I_9_MAG_POWER_DOWN);
    hal->delay_ms(1);

    ak_write(tile, AK09916_REG_CNTL2, AK09916_MODE_SELF_TEST);

    /* Poll DRDY for up to 20 ms (single-meas time TSM ≤ 8.2 ms) */
    uint8_t st1 = 0;
    for (uint8_t i = 0; i < 40; i++) {
        st1 = ak_read1(tile, AK09916_REG_ST1);
        if (st1 & 0x01) break;
        hal->delay_ms(1);
    }
    if ((st1 & 0x01) == 0) return 0;

    uint8_t raw[6];
    ak_read(tile, AK09916_REG_HXL, raw, 6);

    /* Read ST2 to release the data lock */
    uint8_t st2 = ak_read1(tile, AK09916_REG_ST2);
    (void)st2;

    int16_t hx = le16(&raw[0]);
    int16_t hy = le16(&raw[2]);
    int16_t hz = le16(&raw[4]);

    /* Datasheet §9.4.4.2 pass criteria. */
    uint8_t pass_x = (hx >= -200) && (hx <= 200);
    uint8_t pass_y = (hy >= -200) && (hy <= 200);
    uint8_t pass_z = (hz >= -1000) && (hz <= -200);

    return (pass_x && pass_y && pass_z) ? 1 : 0;
}

/* -------------------------------------------------------------- */
/* Tier-2 idiomatic helpers                                        */
/* -------------------------------------------------------------- */

/* At ±2 g (init default), 1 g = 16384 LSB. Helper constants use
 * that scaling; if the user has switched ranges they can read raw
 * data with get_raw_accels and convert manually. */
#define SENSE_I_9_LSB_PER_G_2G   16384

/* Face-up / face-down decision band: |Z| > ~0.85 g and |X|,|Y| < ~0.5 g.
 * 0.85 g ≈ 13926 LSB at ±2 g; 0.5 g = 8192 LSB. */
#define SENSE_I_9_FACE_Z_MIN     13926
#define SENSE_I_9_FACE_XY_MAX     8192

/**
 * @brief  Integer atan2 approximation, output in 0.01° units.
 *
 * Returns angle in centi-degrees in the range [-18000, +18000].
 * Uses the well-known min/max polynomial approximation
 *   atan(t) ≈ t * (45 - (|t| - 1) * (14 + 3.83 * |t|))   for |t| ≤ 1
 * scaled to the centi-degree output and extended to all four
 * quadrants. Worst-case error is ~0.3°, well within the inherent
 * noise of accel and mag readings.
 */
static int32_t atan2_centi(int32_t y, int32_t x)
{
    if (x == 0 && y == 0) return 0;

    int32_t ay = y < 0 ? -y : y;
    int32_t ax = x < 0 ? -x : x;

    int32_t angle_centi;  /* in 0.01° */

    /* Compute |angle| in [0, 9000] (centi-degrees) for ratio in [0,1] */
    if (ax >= ay) {
        /* |y/x| <= 1 */
        if (ax == 0) {
            angle_centi = 0;
        } else {
            /* atan(t) ≈ t * (4500 - (|t|*1000 - 1000) * (14 + 4 * |t|*1000 / 1000) / 1000)
             * Done in fixed-point with t scaled by 1000. */
            int32_t t = (int32_t)(((int64_t)ay * 1000) / ax);          /* t * 1000, 0..1000 */
            int32_t corr = ((t - 1000) * (14000 + 4 * t)) / 1000;       /* (t-1)(14+4t), x1000 */
            /* centideg = 100·t·(45 − (t−1)(14+4t)) with t scaled by 1000:
             * the correction term is t·corr/10 (was /1000, which dropped
             * it 100× and left a ~4° linear-atan error). */
            angle_centi = (t * 4500 - t * corr / 10) / 1000;
            if (angle_centi < 0)    angle_centi = 0;
            if (angle_centi > 4500) angle_centi = 4500;
        }
    } else {
        /* |y/x| > 1: angle = 90 - atan(|x/y|) */
        int32_t t = (int32_t)(((int64_t)ax * 1000) / ay);
        int32_t corr = ((t - 1000) * (14000 + 4 * t)) / 1000;
        int32_t inner = (t * 4500 - t * corr / 10) / 1000;
        if (inner < 0)    inner = 0;
        if (inner > 4500) inner = 4500;
        angle_centi = 9000 - inner;
    }

    /* Quadrant fixup */
    if (x < 0) angle_centi = 18000 - angle_centi;
    if (y < 0) angle_centi = -angle_centi;

    return angle_centi;
}

uint8_t tile_sense_i_9_is_face_up(tile_t* tile)
{
    int16_t a[3];
    tile_sense_i_9_get_raw_accels(tile, a);

    int32_t ax = a[0] < 0 ? -(int32_t)a[0] : (int32_t)a[0];
    int32_t ay = a[1] < 0 ? -(int32_t)a[1] : (int32_t)a[1];

    if (ax > SENSE_I_9_FACE_XY_MAX) return 0;
    if (ay > SENSE_I_9_FACE_XY_MAX) return 0;
    return (a[2] > SENSE_I_9_FACE_Z_MIN) ? 1 : 0;
}

uint8_t tile_sense_i_9_is_face_down(tile_t* tile)
{
    int16_t a[3];
    tile_sense_i_9_get_raw_accels(tile, a);

    int32_t ax = a[0] < 0 ? -(int32_t)a[0] : (int32_t)a[0];
    int32_t ay = a[1] < 0 ? -(int32_t)a[1] : (int32_t)a[1];

    if (ax > SENSE_I_9_FACE_XY_MAX) return 0;
    if (ay > SENSE_I_9_FACE_XY_MAX) return 0;
    return (a[2] < -SENSE_I_9_FACE_Z_MIN) ? 1 : 0;
}

uint8_t tile_sense_i_9_is_moving(tile_t* tile, uint16_t threshold_mg)
{
    int16_t a[3];
    tile_sense_i_9_get_raw_accels(tile, a);

    /* magnitude² in LSB² (at ±2 g, 1 g² = 16384² ≈ 2.68e8, fits easily in
     * int64). Compare |mag² − 1g²| against (threshold)² mapped to LSB². */
    int64_t mag2 = (int64_t)a[0] * a[0]
                 + (int64_t)a[1] * a[1]
                 + (int64_t)a[2] * a[2];

    const int64_t one_g    = (int64_t)SENSE_I_9_LSB_PER_G_2G;
    const int64_t one_g_sq = one_g * one_g;

    /* threshold in LSB: (threshold_mg * 16384) / 1000 */
    int64_t thr_lsb = ((int64_t)threshold_mg * one_g) / 1000;

    /* (|a| − 1g) > thr  ⇔  |a|² − 1g²| > something — but keep both signs.
     * Use the magnitude inequality on (|a| − 1g)*(|a| + 1g) = (|a|² − 1g²),
     * with |a| + 1g ≈ 2g for small motion: (|a| − 1g) ≈ (mag2 − 1g²)/(2g). */
    int64_t delta2 = mag2 - one_g_sq;
    if (delta2 < 0) delta2 = -delta2;

    /* moving if delta2 / (2 * one_g) > thr_lsb */
    return (delta2 > thr_lsb * 2 * one_g) ? 1 : 0;
}

void tile_sense_i_9_read_tilt_centi_degrees(tile_t* tile, uint8_t axis,
                                            int16_t* out_centi_deg)
{
    if (out_centi_deg == NULL) return;

    int16_t a[3];
    tile_sense_i_9_get_raw_accels(tile, a);

    int32_t target = (axis < 3) ? a[axis] : 0;
    int32_t other_a = (axis == 0) ? a[1] : a[0];
    int32_t other_b = (axis == 2) ? a[1] : a[2];

    /* magnitude of the components perpendicular to `target`. Use
     * a sum-of-squares + integer sqrt via Newton's iteration. */
    int64_t s = (int64_t)other_a * other_a + (int64_t)other_b * other_b;
    int32_t perp = 0;
    if (s > 0) {
        /* Integer sqrt by Newton's method, iterated to convergence. The
         * previous fixed 12 iterations from r = 1 had not converged for
         * s above ~2000² (1 g at ±2 g is 16384²), overstating `perp` up
         * to ~4× and skewing the angle by tens of degrees. Starting at
         * r = s (or a bound above sqrt) the sequence falls monotonically
         * to floor(sqrt(s)). */
        int64_t r = s;
        int64_t x = (s + 1) / 2;
        while (x < r) {
            r = x;
            x = (r + s / r) / 2;
        }
        perp = (int32_t)r;
    }

    int32_t centi = atan2_centi(perp, target);
    if (centi >  18000) centi =  18000;
    if (centi < -18000) centi = -18000;
    *out_centi_deg = (int16_t)centi;
}

void tile_sense_i_9_read_heading_centi_degrees(tile_t* tile,
                                               uint16_t* out_centi_deg)
{
    if (out_centi_deg == NULL) return;

    int16_t m[3];
    tile_sense_i_9_get_raw_mags(tile, m);

    /* TODO HW: per ICM-20948 datasheet §5.3 the AK09916 magnetometer axes
     * don't align with the gyro/accel frame on the same die — the mag
     * needs an axis remap before this atan2 will produce a sane heading.
     * Without the remap, expect the reading to be ~90° off (and possibly
     * sign-inverted). Calibrate at the bench by rotating the tile through
     * a known heading and adjusting the (sign, axis) pair below.
     *
     * atan2(-my, mx) gives heading where +X = 0°, increasing clockwise
     * when viewed from above (NED-style heading). */
    int32_t centi = atan2_centi(-(int32_t)m[1], (int32_t)m[0]);

    /* Normalise to [0, 36000) */
    if (centi < 0)        centi += 36000;
    if (centi >= 36000)   centi -= 36000;

    *out_centi_deg = (uint16_t)centi;
}

void tile_sense_i_9_read_tilt_centi_degrees_flat(tile_t* tile, uint8_t axis,
                                                 int32_t* out_centi_deg)
{
    int16_t v = 0;
    tile_sense_i_9_read_tilt_centi_degrees(tile, axis, &v);
    if (out_centi_deg) *out_centi_deg = (int32_t)v;
}

void tile_sense_i_9_read_heading_centi_degrees_flat(tile_t* tile,
                                                    int32_t* out_centi_deg)
{
    uint16_t v = 0;
    tile_sense_i_9_read_heading_centi_degrees(tile, &v);
    if (out_centi_deg) *out_centi_deg = (int32_t)v;
}

uint8_t tile_sense_i_9_wait_for_motion(tile_t* tile, uint32_t timeout_ms)
{
    /* Poll ~every 5 ms — fine enough that the worst-case latency
     * after motion is <5 ms, sparse enough that the I2C bus stays
     * mostly idle. Caller-supplied timeout_ms is rounded up to the
     * nearest multiple of the poll interval. */
    const uint32_t poll_ms = 5;
    uint32_t waited = 0;

    while (waited <= timeout_ms) {
        uint8_t status = tile_sense_i_9_get_int_status(tile);
        if (status & ICM20948_INT_WOM) {
            return 1;
        }
        if (waited == timeout_ms) break;
        uint32_t step = (timeout_ms - waited < poll_ms) ? (timeout_ms - waited) : poll_ms;
        tile->hal->delay_ms((uint16_t)step);
        waited += step;
    }
    return 0;
}

/* ================================================================ */
/* DMP3 — firmware load                                              */
/*                                                                    */
/* Reference: TDK eMD-SmartMotion-ICM20948-1.1.0-MP. Load procedure   */
/* mirrored from SparkFun's port (inv_icm20948_write_mems +           */
/* inv_icm20948_firmware_load in their ICM_20948_C.c).                */
/* ================================================================ */

/** Maximum bytes per I²C burst into MEM_R_W. The ICM-20948's
 *  serial interface buffer accepts at least 16 bytes per transaction
 *  per the eMD reference; keep at the proven value. */
#define DMP_MEM_CHUNK   16U

/** Page size of the DMP RAM. Bank crossings (a write that would
 *  span the 0x100-byte boundary) need an extra MEM_BANK_SEL update. */
#define DMP_PAGE_SIZE   0x100U

/** Write `len` bytes from `data` to DMP RAM starting at `addr` (a
 *  16-bit address: high byte = bank, low byte = offset within page).
 *  Handles bank crossings and chunks the bus access at DMP_MEM_CHUNK.
 *  Caller is responsible for being on register bank 0. */
static void dmp_write_mems(tile_t *tile, uint16_t addr, uint16_t len,
                           const uint8_t *data)
{
    icm_state_t *s = state_for(tile);
    uint16_t written = 0;

    while (written < len) {
        uint8_t bank      = (uint8_t)(addr >> 8);   /* 8-bit DMP-RAM bank (was &0x0F: wrapped >4KB) */
        uint8_t page_off  = (uint8_t)(addr & 0xFF);
        uint16_t remain   = len - written;
        uint16_t to_page  = DMP_PAGE_SIZE - page_off;
        uint16_t this_len = remain < DMP_MEM_CHUNK ? remain : DMP_MEM_CHUNK;
        if (this_len > to_page) this_len = to_page;

        if (bank != s->last_dmp_bank) {
            icm_write(tile, ICM20948_REG_MEM_BANK_SEL, bank);
            s->last_dmp_bank = bank;
        }
        icm_write(tile, ICM20948_REG_MEM_START_ADDR, page_off);
        tile->hal->i2c_write(tile->hal->handle, tile->id,
                             ICM20948_REG_MEM_R_W,
                             (uint8_t *)&data[written], (uint16_t)this_len);

        written += this_len;
        addr    += this_len;
    }
}

/** Read `len` bytes from DMP RAM at `addr` into `data`. Same chunking
 *  + bank-crossing rules as dmp_write_mems. Caller is on bank 0. */
static void dmp_read_mems(tile_t *tile, uint16_t addr, uint16_t len,
                          uint8_t *data)
{
    icm_state_t *s = state_for(tile);
    uint16_t read = 0;

    while (read < len) {
        uint8_t bank      = (uint8_t)(addr >> 8);   /* 8-bit DMP-RAM bank (was &0x0F: wrapped >4KB) */
        uint8_t page_off  = (uint8_t)(addr & 0xFF);
        uint16_t remain   = len - read;
        uint16_t to_page  = DMP_PAGE_SIZE - page_off;
        uint16_t this_len = remain < DMP_MEM_CHUNK ? remain : DMP_MEM_CHUNK;
        if (this_len > to_page) this_len = to_page;

        if (bank != s->last_dmp_bank) {
            icm_write(tile, ICM20948_REG_MEM_BANK_SEL, bank);
            s->last_dmp_bank = bank;
        }
        icm_write(tile, ICM20948_REG_MEM_START_ADDR, page_off);
        tile->hal->i2c_read(tile->hal->handle, tile->id,
                            ICM20948_REG_MEM_R_W,
                            &data[read], (uint16_t)this_len);

        read += this_len;
        addr += this_len;
    }
}

uint8_t tile_sense_i_9_dmp_load(tile_t* tile)
{
    if (tile->state != TILE_STATE_READY) return 0;

    icm_state_t *s = state_for(tile);
    if (s->dmp_loaded) return 1;  /* idempotent — re-load is unnecessary */

    /* DMP RAM access lives in bank 0; ensure we're not stuck on bank 1/2/3
     * from some prior register access. Force bank-cache invalidation on
     * the DMP-side so the first write sets MEM_BANK_SEL explicitly. */
    set_bank(tile, ICM20948_BANK_0);
    s->last_dmp_bank = 0xFF;

    /* The chip must be awake AND not in low-power for the DMP RAM port
     * to function. init() leaves it in run mode, but be defensive in
     * case the caller put it to sleep before invoking dmp_load. */
    icm_modify(tile, ICM20948_REG_PWR_MGMT_1, 0x40, 0x00);  /* clear SLEEP */
    icm_modify(tile, ICM20948_REG_PWR_MGMT_1, 0x20, 0x00);  /* clear LP_EN */
    tile->hal->delay_ms(5);

    /* Load: 14301-byte blob to DMP RAM starting at DMP_LOAD_START. */
    dmp_write_mems(tile, ICM20948_DMP_LOAD_START,
                   ICM20948_DMP_FIRMWARE_SIZE,
                   icm20948_dmp3_firmware);

    /* Verify: read back in DMP_MEM_CHUNK-sized windows and compare.
     * A mismatch usually means a flaky bus or a chip in the wrong
     * power state; surface as a load failure so the caller can
     * retry / reset. */
    {
        uint8_t scratch[DMP_MEM_CHUNK];
        uint16_t off = 0;
        s->last_dmp_bank = 0xFF;  /* reset cache for the read pass */
        while (off < ICM20948_DMP_FIRMWARE_SIZE) {
            uint16_t chunk = ICM20948_DMP_FIRMWARE_SIZE - off;
            if (chunk > DMP_MEM_CHUNK) chunk = DMP_MEM_CHUNK;
            dmp_read_mems(tile,
                          (uint16_t)(ICM20948_DMP_LOAD_START + off),
                          chunk, scratch);
            if (memcmp(scratch, &icm20948_dmp3_firmware[off], chunk) != 0) {
                TILE_ON_ERROR(tile, "dmp_load: verify mismatch");
                return 0;
            }
            off += chunk;
        }
    }

    s->dmp_loaded = 1;
    return 1;
}

uint8_t tile_sense_i_9_dmp_is_loaded(tile_t* tile)
{
    return state_for(tile)->dmp_loaded ? 1 : 0;
}

/* ================================================================ */
/* DMP3 — Phase 2: 9-axis quaternion (ROTATION_VECTOR)                */
/*                                                                    */
/* Reference: TDK eMD-SmartMotion-ICM20948-1.1.0-MP. Mirrored from    */
/* SparkFun's ICM_20948.cpp (initializeDMP) +                         */
/* ICM_20948_C.c (inv_icm20948_set_dmp_sensor_period,                 */
/*                 inv_icm20948_enable_dmp_sensor,                    */
/*                 inv_icm20948_read_dmp_data,                        */
/*                 inv_icm20948_set_gyro_sf,                          */
/*                 ICM_20948_i2c_master_enable,                       */
/*                 ICM_20948_i2c_controller_configure_peripheral).    */
/* ================================================================ */

/* DMP RAM offsets (eMD; values lifted unchanged from ICM_20948_DMP.h). */
#define DMP_RAM_DATA_OUT_CTL1      (4U  * 16U + 0U)   /* 0x40, 16-bit BE  */
#define DMP_RAM_DATA_OUT_CTL2      (4U  * 16U + 2U)   /* 0x42, 16-bit BE  */
#define DMP_RAM_DATA_INTR_CTL      (4U  * 16U + 12U)  /* 0x4C, 16-bit BE  */
#define DMP_RAM_MOTION_EVENT_CTL   (4U  * 16U + 14U)  /* 0x4E, 16-bit BE  */
#define DMP_RAM_DATA_RDY_STATUS    (8U  * 16U + 10U)  /* 0x8A, 16-bit BE  */
#define DMP_RAM_ODR_QUAT9          (10U * 16U + 8U)   /* 0xA8, 16-bit BE  */
#define DMP_RAM_ODR_CNTR_QUAT9     (8U  * 16U + 8U)   /* 0x88, 16-bit BE  */
#define DMP_RAM_ACC_SCALE          (30U * 16U + 0U)   /* 0x1E0, 32-bit BE */
#define DMP_RAM_ACC_SCALE2         (79U * 16U + 4U)   /* 0x4F4, 32-bit BE */
#define DMP_RAM_CPASS_MTX_00       (23U * 16U + 0U)
#define DMP_RAM_CPASS_MTX_01       (23U * 16U + 4U)
#define DMP_RAM_CPASS_MTX_02       (23U * 16U + 8U)
#define DMP_RAM_CPASS_MTX_10       (23U * 16U + 12U)
#define DMP_RAM_CPASS_MTX_11       (24U * 16U + 0U)
#define DMP_RAM_CPASS_MTX_12       (24U * 16U + 4U)
#define DMP_RAM_CPASS_MTX_20       (24U * 16U + 8U)
#define DMP_RAM_CPASS_MTX_21       (24U * 16U + 12U)
#define DMP_RAM_CPASS_MTX_22       (25U * 16U + 0U)
#define DMP_RAM_B2S_MTX_00         (208U * 16U + 0U)
#define DMP_RAM_B2S_MTX_11         (209U * 16U + 0U)
#define DMP_RAM_B2S_MTX_22         (210U * 16U + 0U)
#define DMP_RAM_GYRO_SF            (19U * 16U + 0U)   /* 32-bit, signed BE  */
#define DMP_RAM_GYRO_FULLSCALE     (72U * 16U + 12U)  /* 32-bit BE          */
#define DMP_RAM_ACCEL_ONLY_GAIN    (16U * 16U + 12U)  /* 32-bit BE          */
#define DMP_RAM_ACCEL_ALPHA_VAR    (91U * 16U + 0U)   /* 32-bit BE          */
#define DMP_RAM_ACCEL_A_VAR        (92U * 16U + 0U)   /* 32-bit BE          */
#define DMP_RAM_ACCEL_CAL_RATE     (94U * 16U + 4U)   /* 16-bit BE          */
#define DMP_RAM_CPASS_TIME_BUFFER  (112U * 16U + 14U) /* 16-bit BE          */

/* Header bitmap (DATA_OUT_CTL1 + Quat9 packet header). */
#define DMP_HDR_QUAT9              0x0400U
#define DMP_HDR_HEADER2            0x0008U

/* Motion event control bits relevant to Quat9. */
#define DMP_MEC_9AXIS              0x0040U
#define DMP_MEC_ACCEL_CALIBR       0x0200U
#define DMP_MEC_GYRO_CALIBR        0x0100U
#define DMP_MEC_COMPASS_CALIBR     0x0080U

/* DATA_RDY_STATUS bits (which sensors feed the DMP). */
#define DMP_DRS_GYRO               0x0001U
#define DMP_DRS_ACCEL              0x0002U
#define DMP_DRS_SECONDARY_COMPASS  0x0008U

/* DMP firmware program-start address (eMD; stored to PRGM_START_ADDRH/L). */
#define DMP_PRGM_START_ADDRESS     0x1000U

/* Quat9 wire packet sizes. */
#define DMP_QUAT9_HEADER_BYTES     2U
#define DMP_QUAT9_PAYLOAD_BYTES    14U
#define DMP_QUAT9_TOTAL_BYTES      (DMP_QUAT9_HEADER_BYTES + DMP_QUAT9_PAYLOAD_BYTES)  /* 16 */

/* Helpers — write 2 / 4 BE bytes to a DMP RAM offset. */
static void dmp_write_u16_be(tile_t *tile, uint16_t addr, uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)(v & 0xFF) };
    dmp_write_mems(tile, addr, 2U, b);
}

static void dmp_write_u32_be(tile_t *tile, uint16_t addr, uint32_t v)
{
    uint8_t b[4] = {
        (uint8_t)((v >> 24) & 0xFF),
        (uint8_t)((v >> 16) & 0xFF),
        (uint8_t)((v >>  8) & 0xFF),
        (uint8_t)((v      ) & 0xFF),
    };
    dmp_write_mems(tile, addr, 4U, b);
}

/* DMP gyro scaling factor, per inv_icm20948_set_gyro_sf(). The chip
 * stores PLL trim in TIMEBASE_CORRECTION_PLL (bank 1, 0x28); the
 * "magic constant" expression below is straight from InvenSense's
 * eMD reference and is what makes the DMP's internal gyro integrator
 * agree with real-world degrees-per-second. */
static int32_t dmp_calc_gyro_sf(tile_t *tile, uint8_t gyro_div, int gyro_level)
{
    set_bank(tile, ICM20948_BANK_1);
    int8_t pll = (int8_t)icm_read1(tile, 0x28);
    set_bank(tile, ICM20948_BANK_0);

    /* Per the eMD reference, gyro_level is forced to 4 regardless of FSR
     * (gyro full-scale is set separately via DMP_RAM_GYRO_FULLSCALE). */
    (void)gyro_level;
    int level = 4;

    const unsigned long long magic       = 264446880937391ULL;
    const unsigned long long magic_scale = 100000ULL;
    unsigned long long result_ll;
    if ((uint8_t)pll & 0x80U) {
        result_ll = (magic * (1ULL << level) * (1U + gyro_div))
                  / (1270U - ((uint8_t)pll & 0x7FU)) / magic_scale;
    } else {
        result_ll = (magic * (1ULL << level) * (1U + gyro_div))
                  / (1270U + (uint32_t)(uint8_t)pll) / magic_scale;
    }
    if (result_ll > 0x7FFFFFFFULL) result_ll = 0x7FFFFFFFULL;
    return (int32_t)result_ll;
}

/* Configure I2C-master peripheral slot. Mirrors
 * inv_icm20948_i2c_controller_configure_peripheral(). Caller is on
 * any bank; we set + restore bank 0. */
static void dmp_cfg_periph(tile_t *tile, uint8_t slot, uint8_t addr,
                           uint8_t reg, uint8_t len, uint8_t rnw,
                           uint8_t enable, uint8_t grp, uint8_t byte_sw,
                           uint8_t data_out)
{
    /* Slot register-block offsets (in bank 3): every slot is 4 regs apart
     * starting at PERIPH0_ADDR (0x03). */
    uint8_t base = (uint8_t)(ICM20948_REG_I2C_PERIPH0_ADDR + (slot * 4U));

    set_bank(tile, ICM20948_BANK_3);

    /* PERIPHx_ADDR: bit 7 = R/!W, bits [6:0] = 7-bit slave address. */
    icm_write(tile, base + 0U,
              (uint8_t)((rnw ? 0x80U : 0x00U) | (addr & 0x7FU)));
    /* If we're configuring a write, push the byte to PERIPHx_DO first
     * — the master grabs it on the next cycle. */
    if (!rnw) {
        icm_write(tile, base + 3U, data_out);
    }
    icm_write(tile, base + 1U, reg);

    /* PERIPHx_CTRL: bit7 = EN, bit5 = GRP (start pairing at byte 1+2),
     * bit4 = BYTE_SW, bits [3:0] = LEN. */
    uint8_t ctrl = (uint8_t)((enable  ? 0x80U : 0x00U)
                           | (grp     ? 0x10U : 0x00U)
                           | (byte_sw ? 0x40U : 0x00U)
                           | (len & 0x0FU));
    icm_write(tile, base + 2U, ctrl);

    set_bank(tile, ICM20948_BANK_0);
}

/* Switch the AK09916 path from BYPASS into the chip's I2C master so
 * the DMP can pull mag samples internally. Configures slot 0 to do a
 * 10-byte burst read starting at AK09916 register 0x03 (RSV2 — see the
 * SparkFun port's narrative for the empirical reverse-engineering note;
 * that band of the AK09916 contains the data the DMP expects, in BE,
 * with the DRDY bit at the start), and slot 1 to write the
 * single-measurement command back to CNTL2 every cycle. */
static void dmp_setup_mag_master(tile_t *tile)
{
    /* 1. Force AK09916 to a known state via bypass (last chance before
     *    we steal the bus). Soft-reset clears any half-set CNTL2 mode. */
    set_bank(tile, ICM20948_BANK_0);
    icm_modify(tile, ICM20948_REG_INT_PIN_CFG,
               ICM20948_INT_PIN_BYPASS_EN, ICM20948_INT_PIN_BYPASS_EN);
    ak_write(tile, AK09916_REG_CNTL3, 0x01);  /* SRST */
    tile->hal->delay_ms(2);
    ak_write(tile, AK09916_REG_CNTL2, SENSE_I_9_MAG_POWER_DOWN);
    tile->hal->delay_ms(1);

    /* 2. Drop bypass; the AK09916 is now reachable only through the
     *    chip's internal master. */
    icm_modify(tile, ICM20948_REG_INT_PIN_CFG,
               ICM20948_INT_PIN_BYPASS_EN, 0);

    /* 3. Configure I2C master clock + NSR. 0x07 → ~345 kHz which is
     *    inside AK09916's 400 kHz spec; bit 4 (P_NSR=1) inserts a
     *    repeated start. */
    set_bank(tile, ICM20948_BANK_3);
    icm_write(tile, ICM20948_REG_I2C_MST_CTRL, 0x17);
    /* I2C_MST_ODR_CONFIG: 1100 / 2^4 = 68.75 Hz mag read rate. */
    icm_write(tile, ICM20948_REG_I2C_MST_ODR_CFG, 0x04);
    set_bank(tile, ICM20948_BANK_0);

    /* 4. Slot 0: AK09916 → bulk read of 10 bytes starting at 0x03.
     *    GRP=1, BYTE_SW=1 to land mag data in the FIFO byte order the
     *    DMP expects (matches the SparkFun port's args). */
    dmp_cfg_periph(tile, 0,
                   AK09916_I2C_ADDR, AK09916_REG_RSV2, 10U,
                   /*rnw=*/1, /*en=*/1, /*grp=*/1, /*bswap=*/1, /*do=*/0);

    /* 5. Slot 1: AK09916 → write CNTL2 = single-measurement each cycle.
     *    The "len=1" + EN=1 with rnw=0 trips the master into write mode
     *    and pushes PERIPH1_DO. */
    dmp_cfg_periph(tile, 1,
                   AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1U,
                   /*rnw=*/0, /*en=*/1, /*grp=*/0, /*bswap=*/0,
                   /*do=*/SENSE_I_9_MAG_SINGLE);

    /* 6. Engage the master. */
    icm_modify(tile, ICM20948_USER_CTRL,
               ICM20948_USER_CTRL_I2C_MST_EN, ICM20948_USER_CTRL_I2C_MST_EN);
}

/* Reverse dmp_setup_mag_master(): disable slots, drop I2C master,
 * re-enable bypass so the host can talk to the AK09916 again. */
static void dmp_teardown_mag_master(tile_t *tile)
{
    /* Disable slots first to stop the master pinging the AK09916. */
    set_bank(tile, ICM20948_BANK_3);
    icm_write(tile, ICM20948_REG_I2C_PERIPH0_CTRL, 0x00);
    icm_write(tile, ICM20948_REG_I2C_PERIPH1_CTRL, 0x00);
    set_bank(tile, ICM20948_BANK_0);

    /* Drop master, restore bypass. */
    icm_modify(tile, ICM20948_USER_CTRL, ICM20948_USER_CTRL_I2C_MST_EN, 0);
    icm_modify(tile, ICM20948_REG_INT_PIN_CFG,
               ICM20948_INT_PIN_BYPASS_EN, ICM20948_INT_PIN_BYPASS_EN);
}

/* Initialize all of the chip-side + DMP-side knobs needed for any DMP
 * sensor (per InvenSense AN "Programming Sequence for DMP Hardware
 * Functions"; mirrored from SparkFun's initializeDMP()). Caller has
 * already loaded the firmware and confirmed the chip is awake. */
static void dmp_chip_setup(tile_t *tile)
{
    tiles_pal_t *hal = tile->hal;

    set_bank(tile, ICM20948_BANK_0);

    /* Auto clock select. */
    icm_write(tile, ICM20948_REG_PWR_MGMT_1, 0x01);

    /* Enable accel+gyro, disable temp (bit 6 reserved-but-set per InvenSense Nucleo). */
    icm_write(tile, ICM20948_REG_PWR_MGMT_2, 0x40);

    /* Cycle only the I2C master; accel+gyro stay continuous. LP_CONFIG bit 6
     * = I2C master cycled, leave bits 5 + 4 (accel/gyro cycled) clear. */
    icm_write(tile, ICM20948_REG_LP_CONFIG, 0x40);

    /* Disable FIFO + DMP while we configure. */
    icm_modify(tile, ICM20948_USER_CTRL,
               (uint8_t)(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN), 0);

    /* Accel: ±4 g, DLPF on (per eMD recommendation for DMP). ACCEL_FS_SEL is bits [2:1],
     * so ±4 g = 01 → 0x02; with DLPF_EN (bit 0) + DLPFCFG=1 (bits [5:3]) that's 0x0B.
     * NB: 0x09 (used previously) is FS_SEL=00 = ±2 g — a 2x mismatch with the ±4 g
     * DMP-RAM ACC_SCALE constants below (gravity read as 2 g). */
    set_bank(tile, ICM20948_BANK_2);
    icm_write(tile, ICM20948_REG_ACCEL_CONFIG, 0x0B);  /* FS=±4g, DLPFCFG=1, DLPF_EN */
    /* Gyro: ±2000 dps, DLPF on. */
    icm_write(tile, ICM20948_REG_GYRO_CONFIG, 0x07);   /* DLPF=0, FS=2000dps, DLPF_EN */

    /* Sample-rate dividers: 19 → 1100/(1+19) = 55 Hz gyro, 1125/(1+19)
     * ≈ 56.25 Hz accel — the eMD reference rate for most DMP sensors. */
    icm_write(tile, ICM20948_REG_GYRO_SMPLRT, 19);
    icm_write(tile, ICM20948_REG_ACCEL_SMPLRT_H, 0);
    icm_write(tile, ICM20948_REG_ACCEL_SMPLRT_L, 19);
    set_bank(tile, ICM20948_BANK_0);

    /* Disable raw-data-ready and FIFO-stream registers — DMP runs the FIFO. */
    icm_write(tile, ICM20948_REG_FIFO_EN_1, 0x00);
    icm_write(tile, ICM20948_REG_FIFO_EN_2, 0x00);
    icm_write(tile, ICM20948_REG_INT_ENABLE_1, 0x00);

    /* Reset FIFO. */
    icm_write(tile, ICM20948_REG_FIFO_RST, 0x1F);
    icm_write(tile, ICM20948_REG_FIFO_RST, 0x00);

    /* DMP firmware entry point (bank 2, 0x50/0x51, big-endian 0x1000). */
    set_bank(tile, ICM20948_BANK_2);
    icm_write(tile, ICM20948_REG_PRGM_START_ADDRH,
              (uint8_t)((DMP_PRGM_START_ADDRESS >> 8) & 0xFF));
    icm_write(tile, ICM20948_REG_PRGM_START_ADDRL,
              (uint8_t)(DMP_PRGM_START_ADDRESS & 0xFF));
    set_bank(tile, ICM20948_BANK_0);

    /* InvenSense-magic registers — values copied verbatim from the eMD
     * reference. Their datasheet doesn't document these; trust the port. */
    icm_write(tile, ICM20948_REG_HW_FIX_DISABLE, 0x48);
    icm_write(tile, ICM20948_REG_SINGLE_FIFO_PRIO, 0xE4);

    /* DMP-RAM constants for the chosen FSR + ODR. */
    /* ACC_SCALE for ±4 g: 1 g aligns to 2^25 → write 0x04000000. */
    dmp_write_u32_be(tile, DMP_RAM_ACC_SCALE,  0x04000000U);
    /* ACC_SCALE2 for ±4 g: hardware-unit output → write 0x00040000. */
    dmp_write_u32_be(tile, DMP_RAM_ACC_SCALE2, 0x00040000U);

    /* Compass mount matrix: identity-ish with sign flips per eMD nucleo
     * reference. ±1 → ±0x09999999 (≈ 1 µT × 2^30 / 6.667 LSB-per-µT). */
    static const uint8_t cpass_zero[4]  = {0x00, 0x00, 0x00, 0x00};
    static const uint8_t cpass_plus[4]  = {0x09, 0x99, 0x99, 0x99};
    static const uint8_t cpass_minus[4] = {0xF6, 0x66, 0x66, 0x67};
    dmp_write_mems(tile, DMP_RAM_CPASS_MTX_00, 4, cpass_plus);
    dmp_write_mems(tile, DMP_RAM_CPASS_MTX_01, 4, cpass_zero);
    dmp_write_mems(tile, DMP_RAM_CPASS_MTX_02, 4, cpass_zero);
    dmp_write_mems(tile, DMP_RAM_CPASS_MTX_10, 4, cpass_zero);
    dmp_write_mems(tile, DMP_RAM_CPASS_MTX_11, 4, cpass_minus);
    dmp_write_mems(tile, DMP_RAM_CPASS_MTX_12, 4, cpass_zero);
    dmp_write_mems(tile, DMP_RAM_CPASS_MTX_20, 4, cpass_zero);
    dmp_write_mems(tile, DMP_RAM_CPASS_MTX_21, 4, cpass_zero);
    dmp_write_mems(tile, DMP_RAM_CPASS_MTX_22, 4, cpass_minus);

    /* B2S (body-to-sensor) mount matrix: identity. 1.0 = 0x40000000. */
    static const uint8_t b2s_plus[4] = {0x40, 0x00, 0x00, 0x00};
    static const uint8_t b2s_zero[4] = {0x00, 0x00, 0x00, 0x00};
    dmp_write_mems(tile, DMP_RAM_B2S_MTX_00 + 0,  4, b2s_plus);
    dmp_write_mems(tile, DMP_RAM_B2S_MTX_00 + 4,  4, b2s_zero);
    dmp_write_mems(tile, DMP_RAM_B2S_MTX_00 + 8,  4, b2s_zero);
    dmp_write_mems(tile, DMP_RAM_B2S_MTX_00 + 12, 4, b2s_zero);
    dmp_write_mems(tile, DMP_RAM_B2S_MTX_11 + 0,  4, b2s_plus);
    dmp_write_mems(tile, DMP_RAM_B2S_MTX_11 + 4,  4, b2s_zero);
    dmp_write_mems(tile, DMP_RAM_B2S_MTX_11 + 8,  4, b2s_zero);
    dmp_write_mems(tile, DMP_RAM_B2S_MTX_11 + 12, 4, b2s_zero);
    dmp_write_mems(tile, DMP_RAM_B2S_MTX_22,      4, b2s_plus);

    /* Gyro SF — eMD's PLL-trimmed scale; without this the integrator drifts. */
    int32_t gsf = dmp_calc_gyro_sf(tile, /*div=*/19, /*level=*/3);
    dmp_write_u32_be(tile, DMP_RAM_GYRO_SF, (uint32_t)gsf);

    /* Gyro full-scale: 2000 dps → 2^28. */
    dmp_write_u32_be(tile, DMP_RAM_GYRO_FULLSCALE, 0x10000000U);

    /* Accel-only-gain + alpha + a-var for 56 Hz ODR — values straight
     * from the InvenSense Nucleo reference. */
    static const uint8_t accel_only_gain[4] = {0x03, 0xA4, 0x92, 0x49}; /* 56 Hz */
    static const uint8_t accel_alpha_var[4] = {0x34, 0x92, 0x49, 0x25};
    static const uint8_t accel_a_var[4]     = {0x0B, 0x6D, 0xB6, 0xDB};
    dmp_write_mems(tile, DMP_RAM_ACCEL_ONLY_GAIN, 4, accel_only_gain);
    dmp_write_mems(tile, DMP_RAM_ACCEL_ALPHA_VAR, 4, accel_alpha_var);
    dmp_write_mems(tile, DMP_RAM_ACCEL_A_VAR,     4, accel_a_var);

    /* Calibration-rate counters. */
    dmp_write_u16_be(tile, DMP_RAM_ACCEL_CAL_RATE,    0x0000);
    /* Mag is read at 68.75 Hz (I2C_MST_ODR_CFG=4); CPASS_TIME_BUFFER
     * is the number of mag samples per second the DMP expects. */
    dmp_write_u16_be(tile, DMP_RAM_CPASS_TIME_BUFFER, 0x0045);  /* 69 */

    (void)hal;
}

/* Public — start 9-axis quaternion output. */
uint8_t tile_sense_i_9_dmp_start_quat9(tile_t* tile, uint16_t output_period_ms)
{
    if (tile == NULL || tile->hal == NULL) return 0;
    icm_state_t *s = state_for(tile);
    if (!s->dmp_loaded) return 0;

    /* Clamp period to the practical 10..1000 ms band. The DMP base is
     * ~225 Hz (gyro div=19 → 55 Hz; the DMP runs Quat9 at 225 Hz against
     * its internal 1.1 kHz / 5 == 220 Hz reference), so divider =
     * (225 / target_hz) - 1 = (225 * period_ms / 1000) - 1. */
    if (output_period_ms < 10)   output_period_ms = 10;
    if (output_period_ms > 1000) output_period_ms = 1000;
    uint32_t div32 = ((uint32_t)225U * output_period_ms) / 1000U;
    if (div32 == 0) div32 = 1;
    uint16_t odr_div = (uint16_t)(div32 - 1U);

    /* Phase 0: chip + DMP-RAM setup (per InvenSense AN). */
    dmp_chip_setup(tile);

    /* Phase 1: route AK09916 through the internal I2C master so the DMP
     * can pull mag samples. This breaks bypass-mode helpers — see the
     * mag-access-mode rule in the header. */
    dmp_setup_mag_master(tile);
    s->dmp_active = 1;

    /* Phase 2: enable Quat9 output (ROTATION_VECTOR). DATA_OUT_CTL1 bit
     * 0x0400 = Quat9. We deliberately leave DATA_OUT_CTL2 = 0 so the
     * DMP doesn't append a header2 byte — Quat9's payload already has
     * a 2-byte heading-accuracy field at the end, which is what callers
     * actually want. */
    set_bank(tile, ICM20948_BANK_0);
    dmp_write_u16_be(tile, DMP_RAM_DATA_OUT_CTL1, DMP_HDR_QUAT9);
    dmp_write_u16_be(tile, DMP_RAM_DATA_OUT_CTL2, 0x0000);
    dmp_write_u16_be(tile, DMP_RAM_DATA_INTR_CTL, DMP_HDR_QUAT9);

    /* Tell the DMP which raw streams it can rely on. Quat9 needs
     * accel + gyro + secondary compass. */
    dmp_write_u16_be(tile, DMP_RAM_DATA_RDY_STATUS,
                     DMP_DRS_GYRO | DMP_DRS_ACCEL | DMP_DRS_SECONDARY_COMPASS);

    /* Motion event control — turn on the 9-axis pipe + the calibration
     * stages it consumes. */
    dmp_write_u16_be(tile, DMP_RAM_MOTION_EVENT_CTL,
                     DMP_MEC_9AXIS | DMP_MEC_ACCEL_CALIBR
                   | DMP_MEC_GYRO_CALIBR | DMP_MEC_COMPASS_CALIBR);

    /* Output rate. The ODR_CNTR register has to be cleared whenever ODR
     * changes (eMD note in inv_icm20948_set_dmp_sensor_period). */
    dmp_write_u16_be(tile, DMP_RAM_ODR_QUAT9,      odr_div);
    dmp_write_u16_be(tile, DMP_RAM_ODR_CNTR_QUAT9, 0x0000);

    /* Phase 3: light up the FIFO + DMP. Reset FIFO once more so we
     * don't get half-baked packets from the configure phase. */
    icm_write(tile, ICM20948_REG_FIFO_RST, 0x1F);
    icm_write(tile, ICM20948_REG_FIFO_RST, 0x00);

    icm_modify(tile, ICM20948_USER_CTRL,
               (uint8_t)(ICM20948_USER_CTRL_DMP_EN | ICM20948_USER_CTRL_FIFO_EN
                       | ICM20948_USER_CTRL_DMP_RST),
               (uint8_t)(ICM20948_USER_CTRL_DMP_EN | ICM20948_USER_CTRL_FIFO_EN));

    return 1;
}

void tile_sense_i_9_dmp_stop(tile_t* tile)
{
    if (tile == NULL || tile->hal == NULL) return;
    icm_state_t *s = state_for(tile);

    set_bank(tile, ICM20948_BANK_0);

    /* Drop DMP_EN + FIFO_EN. */
    icm_modify(tile, ICM20948_USER_CTRL,
               (uint8_t)(ICM20948_USER_CTRL_DMP_EN | ICM20948_USER_CTRL_FIFO_EN), 0);

    /* Clear the feature masks so a future dmp_start_*() starts clean. */
    if (s->dmp_loaded) {
        dmp_write_u16_be(tile, DMP_RAM_DATA_OUT_CTL1,    0x0000);
        dmp_write_u16_be(tile, DMP_RAM_DATA_OUT_CTL2,    0x0000);
        dmp_write_u16_be(tile, DMP_RAM_DATA_INTR_CTL,    0x0000);
        dmp_write_u16_be(tile, DMP_RAM_MOTION_EVENT_CTL, 0x0000);
        dmp_write_u16_be(tile, DMP_RAM_DATA_RDY_STATUS,  0x0000);
    }

    /* Reset FIFO so leftover packets don't fool a future fifo_count() check. */
    icm_write(tile, ICM20948_REG_FIFO_RST, 0x1F);
    icm_write(tile, ICM20948_REG_FIFO_RST, 0x00);

    /* Revert AK09916 access to bypass — direct mag helpers work again
     * after this returns. */
    if (s->dmp_active) {
        dmp_teardown_mag_master(tile);
        s->dmp_active = 0;
    }
}

uint8_t tile_sense_i_9_dmp_data_ready(tile_t* tile)
{
    if (tile == NULL) return 0;
    return (tile_sense_i_9_fifo_count(tile) >= DMP_QUAT9_TOTAL_BYTES) ? 1 : 0;
}

uint8_t tile_sense_i_9_dmp_read_quat9(tile_t* tile, int32_t* out_q,
                                      uint16_t* out_accuracy)
{
    if (tile == NULL || out_q == NULL) return 0;

    /* Need at least header + payload before we touch FIFO_R_W. */
    set_bank(tile, ICM20948_BANK_0);
    if (tile_sense_i_9_fifo_count(tile) < DMP_QUAT9_TOTAL_BYTES) return 0;

    /* Read header. */
    uint8_t hdr_buf[DMP_QUAT9_HEADER_BYTES];
    icm_read(tile, ICM20948_REG_FIFO_R_W, hdr_buf, DMP_QUAT9_HEADER_BYTES);
    uint16_t header = (uint16_t)((uint16_t)hdr_buf[0] << 8 | hdr_buf[1]);

    /* Quat9-only stream: header should be exactly 0x0400. If something
     * else slipped in (e.g. a stale packet from before reset), nuke
     * the FIFO and bail — the next call gets a clean window. */
    if ((header & DMP_HDR_QUAT9) == 0) {
        icm_write(tile, ICM20948_REG_FIFO_RST, 0x1F);
        icm_write(tile, ICM20948_REG_FIFO_RST, 0x00);
        return 0;
    }
    if (header & DMP_HDR_HEADER2) {
        /* Defensive — Phase 2 shouldn't enable header2, but if some
         * future path does, drain the header2 + bail rather than
         * mis-parse the payload. */
        uint8_t h2[2];
        icm_read(tile, ICM20948_REG_FIFO_R_W, h2, 2);
        (void)h2;
    }

    /* Payload: 12 bytes BE int32 (q1, q2, q3) + 2 bytes BE uint16 accuracy. */
    uint8_t pl[DMP_QUAT9_PAYLOAD_BYTES];
    icm_read(tile, ICM20948_REG_FIFO_R_W, pl, DMP_QUAT9_PAYLOAD_BYTES);

    int32_t q1 = (int32_t)(((uint32_t)pl[0]  << 24)
                         | ((uint32_t)pl[1]  << 16)
                         | ((uint32_t)pl[2]  <<  8)
                         | ((uint32_t)pl[3]       ));
    int32_t q2 = (int32_t)(((uint32_t)pl[4]  << 24)
                         | ((uint32_t)pl[5]  << 16)
                         | ((uint32_t)pl[6]  <<  8)
                         | ((uint32_t)pl[7]       ));
    int32_t q3 = (int32_t)(((uint32_t)pl[8]  << 24)
                         | ((uint32_t)pl[9]  << 16)
                         | ((uint32_t)pl[10] <<  8)
                         | ((uint32_t)pl[11]      ));
    uint16_t accuracy = (uint16_t)(((uint16_t)pl[12] << 8) | pl[13]);

    /* Recover q0 = sqrt(1 - q1² - q2² - q3²). All values are Q30, so
     * 1.0 corresponds to (1<<30). Square-and-sum in Q60 (int64). If the
     * three components are jointly above the unit sphere (rounding
     * noise can put them slightly over) clamp the radicand to 0 so
     * sqrt doesn't barf. Newton's iteration on int64. */
    const int64_t one_q30  = (int64_t)1 << 30;
    const int64_t one_q60  = one_q30 * one_q30;
    int64_t q1q1 = (int64_t)q1 * (int64_t)q1;
    int64_t q2q2 = (int64_t)q2 * (int64_t)q2;
    int64_t q3q3 = (int64_t)q3 * (int64_t)q3;
    int64_t rad  = one_q60 - q1q1 - q2q2 - q3q3;
    if (rad < 0) rad = 0;

    /* Integer sqrt of `rad` (a non-negative int64). Result fits in
     * int32 because rad <= 2^60 → sqrt(rad) <= 2^30. */
    int32_t q0 = 0;
    if (rad > 0) {
        /* Initial guess: the high half of `rad` shifted right gives
         * a sqrt-order approximation; ensures Newton converges fast. */
        uint64_t r = (uint64_t)rad;
        uint64_t x = r;
        /* Bit-tricks initial estimate — start at `r >> 1` clamped above 1. */
        if (x > 1) x >>= 1;
        for (uint8_t i = 0; i < 24; i++) {
            uint64_t y = (x + r / x) >> 1;
            if (y >= x) break;
            x = y;
        }
        if (x > (uint64_t)0x7FFFFFFFU) x = 0x7FFFFFFFU;
        q0 = (int32_t)x;
    }

    out_q[0] = q0;
    out_q[1] = q1;
    out_q[2] = q2;
    out_q[3] = q3;
    if (out_accuracy) *out_accuracy = accuracy;
    return 1;
}
