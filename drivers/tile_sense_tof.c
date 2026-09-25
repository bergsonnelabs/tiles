/**
 * @file  tile_sense_tof.c
 * @brief Sense.TOF tile driver implementation — TMF8806 time-of-flight.
 */

#include "tile_sense_tof.h"

/* ---- Instance -> I2C address table ---- */

static const uint8_t id_table[] = {
    TMF8806_I2C_ADDR,  /* 0: 0x41 (fixed address) */
};

#define NUM_INSTANCES  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    return (instance < NUM_INSTANCES) ? id_table[instance] : 0;
}

/* SERIAL_NUMBER_0..3, valid after command 0x47 (datasheet §7.6). */
#define TOF_REG_SERIAL_0                0x28
/* Factory calibration: 40.96 M iterations, ~2.2 s in 5 m mode. */
#define TOF_FACTORY_CAL_MIN_TIMEOUT_MS  3000

/* ---- Per-instance state ---- */

typedef struct {
    sense_tof_cfg_t cfg;                        /* Cached measurement config */
    uint8_t calib_data[TMF8806_CALIB_DATA_LEN]; /* Factory calibration data */
    uint8_t calib_valid;                        /* 1 if calib_data is loaded */
    uint8_t state_data[TMF8806_STATE_DATA_LEN]; /* Restored algorithm state */
    uint8_t state_valid;                        /* 1 if alg state has been restored */
    uint8_t measuring;                          /* 1 if measurement is active */
    uint8_t calib_mode;                         /* distance mode calib_data is for */
} tof_state_t;

static tof_state_t tof_state[NUM_INSTANCES];

static tof_state_t *state_for(tile_t *tile)
{
    for (uint8_t i = 0; i < NUM_INSTANCES; i++)
        if (id_table[i] == tile->id) return &tof_state[i];
    return &tof_state[0];
}

/* ---- Portable memzero ---- */

static void memzero(void *p, uint8_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) *b++ = 0;
}

/* ---- Bus helpers ---- */

static void tof_write_reg(tile_t *tile, uint8_t reg, uint8_t val)
{
    tile->hal->i2c_write(tile->hal->handle, tile->id, reg, &val, 1);
}

static uint8_t tof_read_reg(tile_t *tile, uint8_t reg)
{
    uint8_t val = 0;
    tile->hal->i2c_read(tile->hal->handle, tile->id, reg, &val, 1);
    return val;
}

static void tof_read_regs(tile_t *tile, uint8_t reg, uint8_t *buf, uint16_t len)
{
    tile->hal->i2c_read(tile->hal->handle, tile->id, reg, buf, len);
}

static void tof_write_regs(tile_t *tile, uint8_t reg, const uint8_t *buf,
                            uint16_t len)
{
    tile->hal->i2c_write(tile->hal->handle, tile->id, reg, buf, len);
}

/* ---- Internal helpers ---- */

/**
 * @brief  Build the algorithm selection byte (cmd_data6) from distance mode.
 *
 * Bit layout:
 *   bit 1: distanceEnabled
 *   bit 2: vcselClkDiv2
 *   bit 3: distanceMode
 *   bit 6: distanceMode10m (unused, keep 0)
 *   bit 7: algKeepReady (unused, keep 0)
 *
 * SHORT_RANGE:  distanceEnabled=0 -> 0x00
 * RANGE_2500MM: distanceEnabled=1, vcselClkDiv2=0, distanceMode=0 -> 0x02
 * RANGE_5000MM: distanceEnabled=1, vcselClkDiv2=1, distanceMode=1 -> 0x0E
 */
static uint8_t build_algo_byte(uint8_t mode)
{
    switch (mode) {
    case SENSE_TOF_SHORT_RANGE:
        return 0x00;
    case SENSE_TOF_RANGE_5000MM:
        return 0x0E;  /* bits 1+2+3 set */
    case SENSE_TOF_RANGE_2500MM:
    default:
        return 0x02;  /* bit 1 set */
    }
}

/**
 * @brief  Write the 11-byte measurement command payload and trigger.
 *
 * Writes cmd_data9..cmd_data0 (registers 0x06-0x0F) followed by
 * the COMMAND register (0x10) in a single burst.
 */
/* cmd_data7[5:3] spadDeadTime: "if unsure use default=4" (datasheet Table 23).
 * The same value is used for calibration and ranging: a calibration only holds
 * for the dead time it was taken with (§6.9). */
#define TOF_SPAD_DEAD_TIME_BITS  (4u << 3)

static void tof_write_cmd_payload(tile_t *tile, uint8_t command)
{
    tof_state_t *s = state_for(tile);
    uint8_t buf[11];

    buf[0]  = 0x00;                              /* cmd_data9: SS SpadChargePump off */
    buf[1]  = 0x00;                              /* cmd_data8: SS VcselChargePump off */
    /* cmd_data7: bit0 factoryCal, bit1 algState ("if set, also set
     * factoryCal=1", Table 23), bits 5:3 spadDeadTime. Factory calibration
     * itself runs without either data bit. */
    buf[2]  = TOF_SPAD_DEAD_TIME_BITS;
    if (command == TMF8806_CMD_MEASURE && s->calib_valid) {
        buf[2] |= 0x01;
        if (s->state_valid) buf[2] |= 0x02;
    }
    buf[3]  = build_algo_byte(s->cfg.mode);       /* cmd_data6: algorithm */
    buf[4]  = 0x00;                              /* cmd_data5: GPIO disabled */
    buf[5]  = 0x00;                              /* cmd_data4: delay disabled */
    buf[6]  = s->cfg.threshold & 0x3F;           /* cmd_data3: threshold[5:0] */
    buf[7]  = s->cfg.period_ms;                  /* cmd_data2: repetition period */
    buf[8]  = (uint8_t)(s->cfg.kilo_iters & 0xFF);  /* cmd_data1: kIters LSB */
    buf[9]  = (uint8_t)(s->cfg.kilo_iters >> 8);    /* cmd_data0: kIters MSB */
    buf[10] = command;                           /* COMMAND register */

    tof_write_regs(tile, TMF8806_REG_CMD_DATA9, buf, 11);
}

/**
 * @brief  Poll a register until it matches an expected value.
 *
 * @param  tile        Tile handle with HAL.
 * @param  reg         Register address to poll.
 * @param  expected    Expected value.
 * @param  mask        Mask applied before comparison (0xFF for exact match).
 * @param  timeout_ms  Maximum time to poll.
 * @return 1 if matched within timeout, 0 if timed out.
 */
static uint8_t tof_poll_reg(tile_t *tile, uint8_t reg, uint8_t expected,
                            uint8_t mask, uint32_t timeout_ms)
{
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        uint8_t val = tof_read_reg(tile, reg);
        if ((val & mask) == expected)
            return 1;
        tile->hal->delay_ms(TMF8806_POLL_INTERVAL_MS);
        elapsed += TMF8806_POLL_INTERVAL_MS;
    }
    return 0;
}

/**
 * @brief  Wait until the chip has finished `cmd`: COMMAND 0x00, PREVIOUS == cmd.
 * @return 1 when done, 0 on timeout.
 */
static uint8_t tof_wait_cmd_done(tile_t *tile, uint8_t cmd, uint32_t timeout_ms)
{
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        uint8_t r[2] = { 0xFF, 0x00 };
        tof_read_regs(tile, TMF8806_REG_COMMAND, r, 2);  /* COMMAND, PREVIOUS */
        if (r[0] == 0x00 && r[1] == cmd) return 1;
        tile->hal->delay_ms(TMF8806_POLL_INTERVAL_MS);
        elapsed += TMF8806_POLL_INTERVAL_MS;
    }
    return 0;
}

/* A command sent while ranging runs is refused ("InvalCmd, no stop command
 * was sent before"). Stop first; return whether ranging was running so the
 * caller can restart it. */
static uint8_t tof_pause(tile_t *tile)
{
    tof_state_t *s = state_for(tile);
    uint8_t was = s->measuring;
    if (was) tile_sense_tof_stop(tile);
    return was;
}

/**
 * @brief  Execute the TMF8806 boot sequence (wake bootloader, start App0).
 *
 * @param  tile  Tile handle with valid HAL and id.
 * @return 1 on success, 0 on timeout.
 */
static uint8_t tof_boot_sequence(tile_t *tile)
{
    /* Step 1: Ensure the CPU is powered and ready (ENABLE == 0x41).
     *
     * This used to first wait for ENABLE == 0x00 ("bootloader sleeping") and
     * bail out if it never arrived. The part does not power up that way: a
     * cold TMF8806 comes up with ENABLE == 0x41 (PON already set, CPU ready)
     * and APPID == 0x80 (bootloader). So that poll ALWAYS timed out, init
     * always returned failure, step 4 was never reached, App0 was never
     * requested — and every result register read back 0x00. It only ever
     * appeared to work when App0 happened to already be running from an
     * earlier session (a warm MCU reset does not power-cycle the tile).
     *
     * Set PON only if it isn't set, then wait for CPU ready. Already-ready is
     * success, not an error. */
    if (tof_read_reg(tile, TMF8806_REG_ENABLE) != TMF8806_ENABLE_CPU_READY) {
        tof_write_reg(tile, TMF8806_REG_ENABLE, TMF8806_ENABLE_PON);
    }
    if (!tof_poll_reg(tile, TMF8806_REG_ENABLE, TMF8806_ENABLE_CPU_READY,
                      0xFF, TMF8806_BOOT_TIMEOUT_MS)) {
        TILE_ON_ERROR(tile, "sense_tof: CPU not ready");
        return 0;
    }

    /* Step 2: Already running the measurement app? Nothing more to do — this
     * is the warm-reset case, where the tile kept running across an MCU reset. */
    if (tof_read_reg(tile, TMF8806_REG_APPID) == TMF8806_APPID_APP0) {
        return 1;
    }

    /* Step 3: Request App0 measurement application */
    tof_write_reg(tile, TMF8806_REG_APPREQID, TMF8806_APPID_APP0);

    /* Step 4: Wait for App0 to start (APPID == 0xC0) */
    if (!tof_poll_reg(tile, TMF8806_REG_APPID, TMF8806_APPID_APP0,
                      0xFF, TMF8806_BOOT_TIMEOUT_MS)) {
        TILE_ON_ERROR(tile, "sense_tof: App0 did not start");
        return 0;
    }

    return 1;
}

/**
 * @brief  Apply default configuration values.
 */
static void tof_apply_defaults(sense_tof_cfg_t *cfg)
{
    cfg->mode       = SENSE_TOF_RANGE_2500MM;
    cfg->period_ms  = 0x1E;   /* 30 ms */
    cfg->kilo_iters = 900;
    cfg->threshold  = 6;
}

/* ---- Lifecycle ---- */

uint8_t tile_sense_tof_find(tiles_pal_t *hal, uint8_t instance)
{
    uint8_t addr = resolve_id(instance);
    if (!addr) return 0;

    if (hal->i2c_is_ready(hal->handle, addr) != 0)
        return 0;

    uint8_t id = 0;
    hal->i2c_read(hal->handle, addr, TMF8806_REG_ID, &id, 1);
    return ((id & TMF8806_ID_MASK) == TMF8806_DEVICE_ID) ? 1 : 0;
}

void tile_sense_tof_init(tiles_pal_t *hal, uint8_t instance,
                         tile_t *tile, const sense_tof_cfg_t *cfg)
{
    memzero(tile, sizeof(tile_t));
    tile->hal = hal;
    tile->id  = resolve_id(instance);

    if (!tile->id) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_tof: invalid instance");
        return;
    }

    if (hal->i2c_is_ready(hal->handle, tile->id) != 0) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_tof: device not responding");
        return;
    }

    /* Verify device ID */
    uint8_t id = tof_read_reg(tile, TMF8806_REG_ID);
    if ((id & TMF8806_ID_MASK) != TMF8806_DEVICE_ID) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_tof: ID mismatch");
        return;
    }

    /* Initialise per-instance state */
    tof_state_t *s = state_for(tile);
    memzero(s, sizeof(tof_state_t));

    /* Apply configuration */
    if (cfg) {
        s->cfg.mode       = cfg->mode;
        s->cfg.period_ms  = cfg->period_ms ? cfg->period_ms : 0x1E;
        s->cfg.kilo_iters = cfg->kilo_iters ? cfg->kilo_iters : 900;
        s->cfg.threshold  = cfg->threshold ? cfg->threshold : 6;
    } else {
        tof_apply_defaults(&s->cfg);
    }

    /* Execute boot sequence */
    if (!tof_boot_sequence(tile)) {
        tile->state = TILE_STATE_ERROR;
        return;
    }

    /* Enable result interrupt */
    tof_write_reg(tile, TMF8806_REG_INT_ENAB, TMF8806_INT_RESULT);

    tile->state = TILE_STATE_READY;
}

void tile_sense_tof_sleep(tile_t *tile)
{
    /* Stop any active measurement first */
    tile_sense_tof_stop(tile);

    /* Power down: clear PON */
    tof_write_reg(tile, TMF8806_REG_ENABLE, 0x00);
    tile->state = TILE_STATE_SLEEPING;
}

void tile_sense_tof_wake(tile_t *tile)
{
    /* Re-run boot sequence to bring App0 back up */
    if (!tof_boot_sequence(tile)) {
        tile->state = TILE_STATE_ERROR;
        return;
    }

    /* Re-enable result interrupt */
    tof_write_reg(tile, TMF8806_REG_INT_ENAB, TMF8806_INT_RESULT);

    tile->state = TILE_STATE_READY;
}

void tile_sense_tof_reset(tile_t *tile)
{
    /* Assert CPU reset: the chip restarts in its bootloader. */
    tof_write_reg(tile, TMF8806_REG_ENABLE, TMF8806_ENABLE_CPU_RESET);
    tile->hal->delay_ms(5);

    /* Calibration and algorithm state belong to the old session; the
     * measurement configuration is kept (a zeroed one would be invalid:
     * kIters must be >= 10). */
    tof_state_t *s = state_for(tile);
    s->calib_valid = 0;
    s->state_valid = 0;
    s->measuring   = 0;

    /* Bring App0 back so the tile is usable again, as after init(). */
    if (!tof_boot_sequence(tile)) {
        tile->state = TILE_STATE_ERROR;
        return;
    }
    tof_write_reg(tile, TMF8806_REG_INT_ENAB, TMF8806_INT_RESULT);
    tile->state = TILE_STATE_READY;
}

/* ---- Measurement control ---- */

void tile_sense_tof_start(tile_t *tile)
{
    tof_state_t *s = state_for(tile);

    /* Calibration (14 B at 0x20), then algorithm state (11 B at 0x2E) when a
     * saved state was restored: one block, host-driver note §9.1
     * ("W 20 <CALIB><STATE>"). The state is only meaningful with the
     * calibration and only for the next start, so it is consumed here. */
    if (s->calib_valid) {
        tof_write_regs(tile, TMF8806_REG_FACTORY_CALIB,
                       s->calib_data, TMF8806_CALIB_DATA_LEN);
        if (s->state_valid)
            tof_write_regs(tile, TMF8806_REG_STATE_DATA_WR,
                           s->state_data, TMF8806_STATE_DATA_LEN);
    }

    /* Clear any pending interrupt */
    tof_write_reg(tile, TMF8806_REG_INT_STATUS, TMF8806_INT_RESULT);

    /* Issue measurement command with configuration */
    tof_write_cmd_payload(tile, TMF8806_CMD_MEASURE);
    s->state_valid = 0;
    s->measuring = (s->cfg.period_ms != 0x00);   /* 0 = one shot, ends by itself */
}

void tile_sense_tof_stop(tile_t *tile)
{
    /* Send stop command */
    tof_write_reg(tile, TMF8806_REG_COMMAND, TMF8806_CMD_STOP);

    /* Done when COMMAND reads back 0x00 and PREVIOUS 0xFF (datasheet §6.2.1:
     * wait for [0x10],2 == [0x00,0xFF]); up to 4.5 ms (host-driver note
     * §8.10). COMMAND still reading 0xFF only means it hasn't been taken
     * yet, so it must not end the wait. */
    tof_wait_cmd_done(tile, TMF8806_CMD_STOP, TMF8806_CMD_TIMEOUT_MS);

    /* Clear any pending interrupt */
    tof_write_reg(tile, TMF8806_REG_INT_STATUS,
                  TMF8806_INT_RESULT | TMF8806_INT_HISTOGRAM);

    tof_state_t *s = state_for(tile);
    s->measuring = 0;
}

uint8_t tile_sense_tof_measure_single(tile_t *tile, sense_tof_result_t *result,
                                      uint32_t timeout_ms)
{
    tof_state_t *s = state_for(tile);
    uint8_t was_measuring = tof_pause(tile);

    /* Save and override period to single-shot */
    uint8_t saved_period = s->cfg.period_ms;
    s->cfg.period_ms = 0x00;

    /* Start the measurement */
    tile_sense_tof_start(tile);

    /* Restore period setting */
    s->cfg.period_ms = saved_period;
    uint8_t ok = 0;

    /* Poll for result interrupt */
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        uint8_t int_status = tof_read_reg(tile, TMF8806_REG_INT_STATUS);
        if (int_status & TMF8806_INT_RESULT) {
            sense_tof_result_t r = {0};
            tile_sense_tof_get_result(tile, &r);
            if (result) *result = r;
            /* A result block (contents 0x55) with no error status. */
            ok = (r.status < 0x10) ? 1 : 0;
            break;
        }
        tile->hal->delay_ms(TMF8806_POLL_INTERVAL_MS);
        elapsed += TMF8806_POLL_INTERVAL_MS;
    }
    if (elapsed >= timeout_ms) tile_sense_tof_stop(tile);   /* timed out */

    /* Put continuous ranging back if it was running. */
    if (was_measuring) tile_sense_tof_start(tile);
    return ok;
}

/* ---- Result reading ---- */

uint16_t tile_sense_tof_max_range_mm(tile_t *tile)
{
    tof_state_t *s = state_for(tile);
    switch (s->cfg.mode) {
    case SENSE_TOF_SHORT_RANGE:  return 200;
    case SENSE_TOF_RANGE_5000MM: return 5000;
    case SENSE_TOF_RANGE_2500MM:
    default:                     return 2500;
    }
}

uint16_t tile_sense_tof_get_distance_mm(tile_t *tile)
{
    /* Same 7-byte window as get_result() (0x1D..0x23), so both take the
     * reading exactly the same way; distance is the last two bytes
     * (0x22 LSB, 0x23 MSB) and reliability comes from RESULT_INFO (0x21).
     *
     * (An earlier revision read only the two distance bytes at 0x22. That was
     * changed to this burst on the theory that the TMF8806's "always start
     * reading from 0x1D" rule made the short read return zeros — it does not;
     * the all-zeros bug was init never starting App0. The burst is kept
     * because it matches get_result() and yields reliability for free.) */
    uint8_t buf[7] = {0};
    tof_read_regs(tile, TMF8806_REG_STATUS, buf, 7);

    /* Clear result interrupt */
    tof_write_reg(tile, TMF8806_REG_INT_STATUS, TMF8806_INT_RESULT);

    const uint16_t mm = (uint16_t)(((uint16_t)buf[6] << 8) | (uint16_t)buf[5]);

    /* Out of range SATURATES to the configured maximum instead of returning 0.
     *
     * The sensor reports "no object" as distance 0, which sits at the CLOSE end
     * of the number line — so the natural way to write proximity logic,
     * `if (distance < threshold)`, fires when there is nothing there at all.
     * Mapping it to max range makes "out of range" behave like "very far away",
     * which is what it physically means.
     *
     * This is deliberately the ONLY transformation: 0 becomes max, every other
     * reading passes through untouched. An earlier version also discarded
     * readings below SENSE_TOF_PRESENCE_RELIABILITY_MIN, which threw away valid
     * close-range measurements — that threshold is for the presence question
     * (is_object_within), where demanding confidence is right, not for reporting
     * a distance. Confidence belongs to the caller: tile_sense_tof_get_result()
     * reports distance, status and reliability, unchanged. */
    if (mm == 0) {
        return tile_sense_tof_max_range_mm(tile);
    }
    return mm;
}

void tile_sense_tof_get_result(tile_t *tile, sense_tof_result_t *result)
{
    /* Read 7 bytes: STATUS(0x1D), REG_CONTENTS(0x1E), TID(0x1F),
       RESULT_NUMBER(0x20), RESULT_INFO(0x21), DIST_LSB(0x22), DIST_MSB(0x23) */
    uint8_t buf[7] = {0};
    tof_read_regs(tile, TMF8806_REG_STATUS, buf, 7);

    result->status        = buf[0];
    result->result_number = buf[3];
    result->reliability   = buf[4] & 0x3F;
    result->distance_mm   = (uint16_t)((uint16_t)buf[6] << 8) | (uint16_t)buf[5];

    /* Read temperature separately (register 0x33, outside the burst range) */
    result->temperature = (int8_t)tof_read_reg(tile, TMF8806_REG_TEMPERATURE);

    /* Clear result interrupt */
    tof_write_reg(tile, TMF8806_REG_INT_STATUS, TMF8806_INT_RESULT);
}

void tile_sense_tof_get_result_flat(tile_t *tile, int32_t *out)
{
    sense_tof_result_t r = {0};
    tile_sense_tof_get_result(tile, &r);
    if (!out) return;
    out[0] = (int32_t)r.distance_mm;
    out[1] = (int32_t)r.status;
    out[2] = (int32_t)r.reliability;
    out[3] = (int32_t)r.temperature;     /* int8_t → int32_t sign-extends */
    out[4] = (int32_t)r.result_number;
}

uint8_t tile_sense_tof_measure_single_flat(tile_t *tile,
                                           int32_t *mm,
                                           int32_t *status,
                                           int32_t *reliability,
                                           int32_t *temp_c,
                                           int32_t *seq,
                                           uint32_t timeout_ms)
{
    sense_tof_result_t r = {0};
    uint8_t ok = tile_sense_tof_measure_single(tile, &r, timeout_ms);
    if (mm)          *mm          = (int32_t)r.distance_mm;
    if (status)      *status      = (int32_t)r.status;
    if (reliability) *reliability = (int32_t)r.reliability;
    if (temp_c)      *temp_c      = (int32_t)r.temperature;
    if (seq)         *seq         = (int32_t)r.result_number;
    return ok;
}

uint8_t tile_sense_tof_result_ready(tile_t *tile)
{
    uint8_t int_status = tof_read_reg(tile, TMF8806_REG_INT_STATUS);
    return (int_status & TMF8806_INT_RESULT) ? 1 : 0;
}

/* ---- Calibration ---- */

uint8_t tile_sense_tof_factory_calibrate(tile_t *tile, uint32_t timeout_ms)
{
    tof_state_t *s = state_for(tile);
    uint8_t was_measuring = tof_pause(tile);

    /* 40.96 M iterations take ~1.1 s in 2.5 m mode and ~2.2 s in 5 m mode;
     * a shorter wait can't succeed. */
    if (timeout_ms < TOF_FACTORY_CAL_MIN_TIMEOUT_MS)
        timeout_ms = TOF_FACTORY_CAL_MIN_TIMEOUT_MS;

    /* Factory calibration needs ~40M iterations (40960 kIters) per the TMF8806
     * datasheet §6.9 / host-driver §8.4 — far more than the ranging default —
     * to produce a low-noise crosstalk reference. Raise kIters for the
     * calibration command only, then restore the ranging configuration. */
    uint16_t saved_kiters = s->cfg.kilo_iters;
    s->cfg.kilo_iters = TMF8806_FACTORY_CAL_KITERS;

    /* Clear any pending interrupt */
    tof_write_reg(tile, TMF8806_REG_INT_STATUS, TMF8806_INT_RESULT);

    /* Issue factory calibration command with the calibration iteration count */
    tof_write_cmd_payload(tile, TMF8806_CMD_FACTORY_CAL);

    s->cfg.kilo_iters = saved_kiters;

    /* Wait for calibration result */
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        uint8_t int_status = tof_read_reg(tile, TMF8806_REG_INT_STATUS);
        if (int_status & TMF8806_INT_RESULT) {
            /* Check that register contents indicate calibration data */
            uint8_t contents = tof_read_reg(tile, TMF8806_REG_REG_CONTENTS);
            if (contents == TMF8806_CONTENTS_CALIB) {
                /* Read and store calibration data */
                tof_read_regs(tile, TMF8806_REG_FACTORY_CALIB,
                              s->calib_data, TMF8806_CALIB_DATA_LEN);
                s->calib_valid = 1;
                s->calib_mode  = s->cfg.mode;
                s->state_valid = 0;

                /* Clear interrupt */
                tof_write_reg(tile, TMF8806_REG_INT_STATUS, TMF8806_INT_RESULT);
                if (was_measuring) tile_sense_tof_start(tile);
                return 1;
            }
            /* Not calibration data — clear and keep waiting */
            tof_write_reg(tile, TMF8806_REG_INT_STATUS, TMF8806_INT_RESULT);
        }
        tile->hal->delay_ms(TMF8806_POLL_INTERVAL_MS);
        elapsed += TMF8806_POLL_INTERVAL_MS;
    }

    if (was_measuring) tile_sense_tof_start(tile);
    return 0;
}

void tile_sense_tof_set_calibration(tile_t *tile, const uint8_t *data)
{
    tof_state_t *s = state_for(tile);
    for (uint8_t i = 0; i < TMF8806_CALIB_DATA_LEN; i++)
        s->calib_data[i] = data[i];
    s->calib_valid = 1;
    s->calib_mode  = s->cfg.mode;   /* taken to be for the current mode */
}

void tile_sense_tof_get_calibration(tile_t *tile, uint8_t *data)
{
    /* The driver's copy: registers 0x20.. hold result bytes once a
     * measurement has run, so reading them back live is not the calibration. */
    tof_state_t *s = state_for(tile);
    for (uint8_t i = 0; i < TMF8806_CALIB_DATA_LEN; i++)
        data[i] = s->calib_valid ? s->calib_data[i] : 0;
}

/* ---- Info ---- */

void tile_sense_tof_get_app_version(tile_t *tile, sense_tof_version_t *version)
{
    version->major = tof_read_reg(tile, TMF8806_REG_APPREV_MAJOR);
    version->minor = tof_read_reg(tile, TMF8806_REG_APPREV_MINOR);
    version->patch = tof_read_reg(tile, TMF8806_REG_APPREV_PATCH);
}

void tile_sense_tof_get_app_version_flat(tile_t *tile,
                                         int32_t *major,
                                         int32_t *minor,
                                         int32_t *patch)
{
    sense_tof_version_t v = {0};
    tile_sense_tof_get_app_version(tile, &v);
    if (major) *major = (int32_t)v.major;
    if (minor) *minor = (int32_t)v.minor;
    if (patch) *patch = (int32_t)v.patch;
}

/* ---- Runtime configuration ---- */

void tile_sense_tof_set_distance_mode(tile_t *tile, sense_tof_distance_mode_t mode)
{
    tof_state_t *s = state_for(tile);
    uint8_t was_measuring = s->measuring;

    if (was_measuring)
        tile_sense_tof_stop(tile);

    s->cfg.mode = (uint8_t)mode;
    /* Short range and 2.5 m share a calibration; 5 m "needs separate
     * calibration data" (datasheet §6.4). Don't send one across that line. */
    if (s->calib_valid &&
        ((s->calib_mode == SENSE_TOF_RANGE_5000MM) != (mode == SENSE_TOF_RANGE_5000MM))) {
        s->calib_valid = 0;
        s->state_valid = 0;
    }

    if (was_measuring)
        tile_sense_tof_start(tile);
}

void tile_sense_tof_set_period(tile_t *tile, sense_tof_period_t period)
{
    tof_state_t *s = state_for(tile);
    uint8_t was_measuring = s->measuring;

    if (was_measuring)
        tile_sense_tof_stop(tile);

    s->cfg.period_ms = (uint8_t)period;

    if (was_measuring)
        tile_sense_tof_start(tile);
}

void tile_sense_tof_set_kilo_iters(tile_t *tile, uint16_t kilo_iters)
{
    tof_state_t *s = state_for(tile);
    uint8_t was_measuring = s->measuring;

    if (was_measuring)
        tile_sense_tof_stop(tile);

    s->cfg.kilo_iters = kilo_iters;

    if (was_measuring)
        tile_sense_tof_start(tile);
}

void tile_sense_tof_set_threshold(tile_t *tile, uint8_t threshold)
{
    tof_state_t *s = state_for(tile);
    uint8_t was_measuring = s->measuring;

    if (was_measuring)
        tile_sense_tof_stop(tile);

    s->cfg.threshold = threshold & 0x3F;

    if (was_measuring)
        tile_sense_tof_start(tile);
}

/* ---- Algorithm state (ultra-low-power) ---- */

void tile_sense_tof_save_state(tile_t *tile, uint8_t *data)
{
    tof_read_regs(tile, TMF8806_REG_STATE_DATA, data, TMF8806_STATE_DATA_LEN);
}

void tile_sense_tof_restore_state(tile_t *tile, const uint8_t *data)
{
    /* Kept and written with the calibration at the next start (the state
     * block follows the calibration at 0x2E, and algState needs factoryCal). */
    tof_state_t *s = state_for(tile);
    for (uint8_t i = 0; i < TMF8806_STATE_DATA_LEN; i++)
        s->state_data[i] = data[i];
    s->state_valid = 1;
}

/* ---- Signal-quality diagnostics ---- */

void tile_sense_tof_get_signal_quality(tile_t *tile, sense_tof_signal_t *sig)
{
    /* reference_hits (0x34-0x37), object_hits (0x38-0x3B), xtalk (0x3C-0x3D)
     * are contiguous — one 10-byte burst, little-endian. */
    uint8_t buf[10];
    tof_read_regs(tile, TMF8806_REG_REF_HITS_0, buf, sizeof(buf));
    sig->reference_hits = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
                        | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    sig->object_hits    = (uint32_t)buf[4] | ((uint32_t)buf[5] << 8)
                        | ((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 24);
    sig->crosstalk      = (uint16_t)((uint16_t)buf[8] | ((uint16_t)buf[9] << 8));
}

void tile_sense_tof_get_signal_quality_flat(tile_t *tile,
                                            int32_t *reference_hits,
                                            int32_t *object_hits,
                                            int32_t *crosstalk)
{
    sense_tof_signal_t sig = {0};
    tile_sense_tof_get_signal_quality(tile, &sig);
    if (reference_hits) *reference_hits = (int32_t)sig.reference_hits;
    if (object_hits)    *object_hits    = (int32_t)sig.object_hits;
    if (crosstalk)      *crosstalk      = (int32_t)sig.crosstalk;
}

/* ---- Info (continued) ---- */

uint8_t tile_sense_tof_get_serial_number(tile_t *tile, uint8_t *serial)
{
    uint8_t was_measuring = tof_pause(tile);

    /* Issue serial number command */
    tof_write_reg(tile, TMF8806_REG_COMMAND, TMF8806_CMD_SERIAL);

    /* Wait for response */
    uint32_t elapsed = 0;
    while (elapsed < TMF8806_CMD_TIMEOUT_MS) {
        uint8_t int_status = tof_read_reg(tile, TMF8806_REG_INT_STATUS);
        if (int_status & TMF8806_INT_RESULT) {
            /* Check that register contents indicate serial data */
            uint8_t contents = tof_read_reg(tile, TMF8806_REG_REG_CONTENTS);
            if (contents == TMF8806_CONTENTS_SERIAL) {
                /* SERIAL_NUMBER_0..3 are 0x28-0x2B (datasheet §7.6) */
                tof_read_regs(tile, TOF_REG_SERIAL_0, serial, 4);

                /* Clear interrupt */
                tof_write_reg(tile, TMF8806_REG_INT_STATUS, TMF8806_INT_RESULT);
                if (was_measuring) tile_sense_tof_start(tile);
                return 1;
            }
            /* Not serial data — clear and keep waiting */
            tof_write_reg(tile, TMF8806_REG_INT_STATUS, TMF8806_INT_RESULT);
        }
        tile->hal->delay_ms(TMF8806_POLL_INTERVAL_MS);
        elapsed += TMF8806_POLL_INTERVAL_MS;
    }

    if (was_measuring) tile_sense_tof_start(tile);
    return 0;
}

void tile_sense_tof_get_serial_number_flat(tile_t *tile, int32_t *out)
{
    if (!out) return;
    uint8_t serial[4] = {0};
    tile_sense_tof_get_serial_number(tile, serial);
    out[0] = (int32_t)serial[0];
    out[1] = (int32_t)serial[1];
    out[2] = (int32_t)serial[2];
    out[3] = (int32_t)serial[3];
}

/* ---------------------------------------------------------------- */
/* Threshold-based interrupts (App0 cmd 0x08 / 0x09)               */
/* ---------------------------------------------------------------- */

#define TMF8806_CMD_WR_ADD_CONFIG   0x08  /**< Write persistence + thresholds */
#define TMF8806_CMD_RD_ADD_CONFIG   0x09  /**< Read back persistence + thresholds */

uint8_t tile_sense_tof_set_threshold_interrupt(tile_t *tile,
                                               uint8_t persistence,
                                               uint16_t low_mm,
                                               uint16_t high_mm)
{
    /* Per HostDriverCommunication §8.12.1 example:
     *   S 41 W 0b <pers> <low_lo> <low_hi> <high_lo> <high_hi> 08 P
     * 6 bytes starting at 0x0B (cmd_data4), wrapping into 0x10 (CMD). */
    uint8_t buf[6] = {
        persistence,
        (uint8_t)(low_mm  & 0xFF),
        (uint8_t)(low_mm  >> 8),
        (uint8_t)(high_mm & 0xFF),
        (uint8_t)(high_mm >> 8),
        TMF8806_CMD_WR_ADD_CONFIG,
    };
    tof_write_regs(tile, TMF8806_REG_CMD_DATA4, buf, sizeof(buf));

    /* Wait for PREVIOUS register to echo the command — confirms
     * App0 has consumed it. */
    return tof_poll_reg(tile, TMF8806_REG_PREVIOUS,
                        TMF8806_CMD_WR_ADD_CONFIG, 0xFF, 50);
}

uint8_t tile_sense_tof_get_threshold_interrupt(tile_t *tile,
                                               uint8_t *persistence,
                                               uint16_t *low_mm,
                                               uint16_t *high_mm)
{
    /* Command 0x09: "persistence, low_threshold and high_threshold are
     * stored in registers 0x20 to 0x24" (datasheet Table 25; the cmd_data
     * registers are write-only). Done = COMMAND 0x00 with PREVIOUS 0x09, so a
     * PREVIOUS left over from an earlier call can't pass for this one. */
    tof_write_reg(tile, TMF8806_REG_COMMAND, TMF8806_CMD_RD_ADD_CONFIG);
    if (!tof_wait_cmd_done(tile, TMF8806_CMD_RD_ADD_CONFIG, 50)) {
        return 0;
    }
    uint8_t buf[5];
    tof_read_regs(tile, TMF8806_REG_FACTORY_CALIB /* 0x20 */, buf, sizeof(buf));

    if (persistence) *persistence = buf[0];
    if (low_mm)      *low_mm  = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
    if (high_mm)     *high_mm = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
    return 1;
}

/* ---------------------------------------------------------------- */
/* Oscillator drift correction — read SYS_CLOCK ticks               */
/* ---------------------------------------------------------------- */

uint32_t tile_sense_tof_get_sys_clock_ticks(tile_t *tile)
{
    /* Datasheet §7.3.5: "an I²C blockread starting from address 0x1D until
     * 0x27 shall be done", and sys_clock "is only valid if its LSB bit is
     * one". SYS_CLOCK_0..3 are the last four bytes of that burst. */
    uint8_t buf[11] = { 0 };
    tof_read_regs(tile, TMF8806_REG_STATUS /* 0x1D */, buf, sizeof(buf));
    uint32_t ticks = (uint32_t)buf[7]
                   | ((uint32_t)buf[8] << 8)
                   | ((uint32_t)buf[9] << 16)
                   | ((uint32_t)buf[10] << 24);
    return (ticks & 1u) ? ticks : 0;
}

/* ---------------------------------------------------------------- */
/* Raw histogram readout (App0 cmd 0x30 + cmd 0x80)                */
/* ---------------------------------------------------------------- */

#define TMF8806_CMD_HIST_CAPTURE    0x30  /**< Configure histogram capture */
#define TMF8806_CMD_HIST_READ_BLOCK 0x80  /**< Start histogram block read */
#define TMF8806_REG_HIST_DATA       0x20  /**< HISTOGRAM_START (datasheet §7.5.1) */

uint8_t tile_sense_tof_read_histogram(tile_t *tile, uint8_t hist_type,
                                      uint8_t *buf128, uint32_t timeout_ms)
{
    if (!buf128) return 0;

    /* 1. Stop any running measurement (best-effort). */
    tof_write_reg(tile, TMF8806_REG_COMMAND, TMF8806_CMD_STOP);
    tof_poll_reg(tile, TMF8806_REG_PREVIOUS, TMF8806_CMD_STOP, 0xFF, 50);

    /* 2. Clear any pending result/histogram interrupts. */
    tof_write_reg(tile, TMF8806_REG_INT_STATUS,
                  TMF8806_INT_RESULT | TMF8806_INT_HISTOGRAM);

    /* 3. Configure histogram capture: payload bytes per
     * HostDriverCommunication §8.11 example
     *   S 41 W 0C <type> 00 00 00 30 P
     * 5 bytes starting at 0x0C (cmd_data3), wrapping into COMMAND. */
    uint8_t cfg[5] = {
        hist_type,                  /* cmd_data3: histogram type */
        0x00,                       /* cmd_data2 */
        0x00,                       /* cmd_data1 */
        0x00,                       /* cmd_data0 */
        TMF8806_CMD_HIST_CAPTURE,   /* COMMAND = 0x30 */
    };
    tof_write_regs(tile, TMF8806_REG_CMD_DATA3, cfg, sizeof(cfg));
    if (!tof_poll_reg(tile, TMF8806_REG_PREVIOUS,
                      TMF8806_CMD_HIST_CAPTURE, 0xFF, 50)) {
        return 0;
    }

    /* 4. Issue a measurement so the chip will produce a histogram.
     * Reuse the existing payload writer; the user's existing cfg is
     * applied. */
    tof_write_cmd_payload(tile, TMF8806_CMD_MEASURE);

    /* 5. Wait for INT_STATUS bit 1 (HISTOGRAM ready). */
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        uint8_t st = tof_read_reg(tile, TMF8806_REG_INT_STATUS);
        if (st & TMF8806_INT_HISTOGRAM) break;
        tile->hal->delay_ms(TMF8806_POLL_INTERVAL_MS);
        elapsed += TMF8806_POLL_INTERVAL_MS;
    }
    if (elapsed >= timeout_ms) return 0;

    /* 6. Save the current TID so we can detect the block read's tick. */
    uint8_t tid_before = tof_read_reg(tile, TMF8806_REG_TID);

    /* 7. Issue the histogram-block-read command (write 0x80 to
     * COMMAND register). */
    tof_write_reg(tile, TMF8806_REG_COMMAND, TMF8806_CMD_HIST_READ_BLOCK);

    /* 8. Wait for TID to change. */
    elapsed = 0;
    while (elapsed < timeout_ms) {
        uint8_t tid_now = tof_read_reg(tile, TMF8806_REG_TID);
        if (tid_now != tid_before) break;
        tile->hal->delay_ms(TMF8806_POLL_INTERVAL_MS);
        elapsed += TMF8806_POLL_INTERVAL_MS;
    }
    if (elapsed >= timeout_ms) return 0;

    /* 9. Read the first 128-byte block in one burst from HISTOGRAM_START:
     * TDC0 quarter 0, bins 0..63 as little-endian 16-bit (host-driver note
     * §8.11). The rest of the histogram (more blocks, continued with
     * command 0x32) is not read here. */
    tof_read_regs(tile, TMF8806_REG_HIST_DATA, buf128, 128);

    /* Clear the interrupt and release the chip, which is otherwise left
     * waiting for the host to continue the readout. */
    tof_write_reg(tile, TMF8806_REG_INT_STATUS, TMF8806_INT_HISTOGRAM);
    tile_sense_tof_stop(tile);
    return 1;
}

void tile_sense_tof_read_histogram_flat(tile_t *tile,
                                        uint8_t hist_type,
                                        uint32_t timeout_ms,
                                        int32_t *out)
{
    if (!out) return;
    uint8_t buf[128] = {0};
    tile_sense_tof_read_histogram(tile, hist_type, buf, timeout_ms);
    for (int i = 0; i < 128; i++) {
        out[i] = (int32_t)buf[i];
    }
}

/* ---------------------------------------------------------------- */
/* Tier-2 idiomatic helpers                                          */
/* ---------------------------------------------------------------- */

/* Single-shot upper bound: ranging at the 4000 k-iteration maximum in 5 m
 * mode takes ~213 ms, plus margin. */
#define TOF_PRESENCE_SINGLE_TIMEOUT_MS  300

/* Reliability (datasheet Table 42): 0 = no object; 1 / 10 = a result from the
 * short-range algorithm (uncalibrated / calibrated), which is what short-range
 * mode and anything within ~200 mm return; any other value = long-range
 * algorithm, 2..63 with 63 best. So 1 and 10 are detections, not "low
 * confidence" long-range results. */
static uint8_t tof_is_detection(uint8_t reliability)
{
    return (reliability == 1 || reliability == 10 ||
            reliability >= SENSE_TOF_PRESENCE_RELIABILITY_MIN) ? 1 : 0;
}

uint8_t tile_sense_tof_is_object_within(tile_t *tile, uint16_t mm)
{
    sense_tof_result_t res;
    if (!tile_sense_tof_measure_single(tile, &res,
                                       TOF_PRESENCE_SINGLE_TIMEOUT_MS)) {
        return 0;
    }
    if (!tof_is_detection(res.reliability)) return 0;
    if (res.distance_mm == 0) return 0;
    return (res.distance_mm <= mm) ? 1 : 0;
}

uint8_t tile_sense_tof_wait_for_object(tile_t *tile, uint16_t mm,
                                       uint32_t timeout_ms)
{
    /* Poll-based v1 — fires single-shot measurements back-to-back,
     * each gated by `is_object_within`. Keeps the helper
     * self-contained: no INT pin wiring, no INT_STATUS bookkeeping
     * across calls. Trade-off is bus traffic and ~30 ms latency
     * per measurement. */
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        if (tile_sense_tof_is_object_within(tile, mm)) return 1;
        tile->hal->delay_ms(SENSE_TOF_WAIT_POLL_INTERVAL_MS);
        elapsed += SENSE_TOF_WAIT_POLL_INTERVAL_MS
                 + TOF_PRESENCE_SINGLE_TIMEOUT_MS / 8;  /* rough per-shot cost */
        /* The /8 above is intentionally a coarse approximation —
         * `measure_single` returns as soon as a result is ready
         * (typically ~30 ms at the 30 ms repetition default), not
         * after the full 200 ms timeout. We add a small fudge so
         * the loop honours `timeout_ms` even if measurements stall. */
    }
    return 0;
}

uint8_t tile_sense_tof_read_distance_with_confidence(tile_t *tile,
                                                     uint16_t *mm,
                                                     uint8_t *confidence_pct)
{
    sense_tof_result_t res;
    if (!tile_sense_tof_measure_single(tile, &res,
                                       TOF_PRESENCE_SINGLE_TIMEOUT_MS)) {
        return 0;
    }
    if (mm) *mm = res.distance_mm;
    if (confidence_pct) {
        /* Long-range reliability 0..63 -> 0..100. The short-range codes are
         * not a scale: 10 (calibrated) reads as 100 %, 1 (uncalibrated) as
         * 50 %. Integer math only. */
        uint16_t rel = (uint16_t)(res.reliability & 0x3F);
        if (rel == 10)     *confidence_pct = 100;
        else if (rel == 1) *confidence_pct = 50;
        else               *confidence_pct = (uint8_t)((rel * 100u) / 63u);
    }
    return 1;
}
