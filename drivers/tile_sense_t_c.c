/**
 * @file   tile_sense_t_c.c
 * @brief  Sense.T.C (IQS323) -- platform-agnostic driver implementation.
 *
 * Platform-agnostic. All bus access via tile->hal function pointers.
 * Reference: Azoteq IQS323 Datasheet v1.11, August 2025.
 */

#include "tile_sense_t_c.h"
#include <stddef.h>

/* ================================================================
 * Instance -> I2C address table
 * ================================================================ */

static const uint8_t id_table[] = {
    IQS323_I2C_ADDR_001,   /* 0: 0x44 (order code 001) */
    IQS323_I2C_ADDR_002,   /* 1: 0x58 (order code 002) */
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
    sense_t_c_event_cb_t on_event;
    void *event_ctx;
    uint16_t last_status;          /* Previous status for edge detection */
    volatile uint8_t rdy_flag;     /* Set by EXTI ISR */
    uint8_t rdy_pin;               /* Core pad number, 0 = polled */
    uint8_t irq_armed;             /* RDY falling-edge ISR registered */
    uint8_t channels;              /* Enabled channel mask (bit n = CHn) */
    uint8_t prox_threshold;
    uint8_t touch_threshold;
    uint8_t power_mode;            /* sense_t_c_power_mode_t chosen by the host */
    uint8_t comm_mode;             /* sense_t_c_comm_mode_t chosen by the host */
    uint8_t ati_retries;           /* Consecutive re-ATIs after an ATI error */
} iqs323_state_t;

static iqs323_state_t state[NUM_INSTANCES];

static iqs323_state_t *state_for(tile_t *tile)
{
    for (uint8_t i = 0; i < NUM_INSTANCES; i++)
        if (id_table[i] == tile->id) return &state[i];
    return &state[0];
}

/* ================================================================
 * Private helpers
 * ================================================================ */

static void memzero(void *p, uint16_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) *b++ = 0;
}

/* Force-communication wait (datasheet §8.13): the window opens 0.1-45 ms
 * after the request. With the RDY interrupt armed we wait for the edge
 * (up to 45 ms); without it a fixed 15 ms, and the read retry covers the
 * slow cases. */
#define IQS323_TWAIT_MAX_MS     45
#define IQS323_TWAIT_POLLED_MS  15

/* Re-ATI budget. I2C is disabled while ATI runs (§8.4); every poll costs a
 * forced window as well as the delay, so the budget counts both. */
#define IQS323_ATI_TIMEOUT_MS   2000

/* Consecutive automatic re-ATIs after an ATI error before giving up. */
#define IQS323_ATI_RETRY_MAX    3

/**
 * Force a communication window open by sending 0xFF as a raw data byte
 * (datasheet §8.13). Returns the milliseconds spent waiting for it.
 * Only works when hal->i2c_write_raw is available.
 */
static uint32_t iqs_force_comms(tile_t *tile)
{
    if (!tile->hal->i2c_write_raw) return 0;
    iqs323_state_t *s = state_for(tile);
    uint8_t cmd = 0xFF;
    if (s->irq_armed) {
        s->rdy_flag = 0;
        tile->hal->i2c_write_raw(tile->hal->handle, tile->id, &cmd, 1);
        uint32_t waited = 0;
        while (!s->rdy_flag && waited < IQS323_TWAIT_MAX_MS) {
            tile->hal->delay_ms(1);
            waited++;
        }
        return waited;
    }
    tile->hal->i2c_write_raw(tile->hal->handle, tile->id, &cmd, 1);
    tile->hal->delay_ms(IQS323_TWAIT_POLLED_MS);
    return IQS323_TWAIT_POLLED_MS;
}

/** Read a 16-bit little-endian register. 0xEEEE outside a window or on a
 *  bus error. */
static uint16_t iqs_read(tile_t *tile, uint8_t reg)
{
    uint8_t buf[2] = {0xEE, 0xEE};
    if (tile->hal->i2c_read(tile->hal->handle, tile->id, reg, buf, 2) != 0)
        return IQS323_INVALID_RESPONSE;
    return ((uint16_t)buf[1] << 8) | buf[0];
}

/** Write a 16-bit little-endian register. */
static void iqs_write(tile_t *tile, uint8_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(value & 0xFF);
    buf[1] = (uint8_t)((value >> 8) & 0xFF);
    tile->hal->i2c_write(tile->hal->handle, tile->id, reg, buf, 2);
}

/** Read with force-comms + retry until non-0xEEEE or timeout. */
static uint16_t iqs_read_window(tile_t *tile, uint8_t reg)
{
    for (int attempt = 0; attempt < 10; attempt++) {
        iqs_force_comms(tile);
        uint16_t val = iqs_read(tile, reg);
        if (val != IQS323_INVALID_RESPONSE)
            return val;
        tile->hal->delay_ms(5);
    }
    return IQS323_INVALID_RESPONSE;
}

/** Write with force-comms + verify readback.
 *  Each I2C STOP closes the window, so we need force_comms before
 *  both the write AND the readback. Not for System Control: its trigger
 *  bits clear themselves, so the readback never matches. */
static void iqs_write_verified(tile_t *tile, uint8_t reg, uint16_t value)
{
    for (int attempt = 0; attempt < 10; attempt++) {
        iqs_force_comms(tile);
        iqs_write(tile, reg, value);
        iqs_force_comms(tile);
        uint16_t readback = iqs_read(tile, reg);
        if (readback == value)
            return;
    }
}

/** Read-modify-write with force-comms. Leaves the register alone if it
 *  can't be read (rather than writing a guess over it). */
static void iqs_modify(tile_t *tile, uint8_t reg, uint16_t mask, uint16_t val)
{
    uint16_t r = iqs_read_window(tile, reg);
    if (r == IQS323_INVALID_RESPONSE) return;
    r = (r & ~mask) | (val & mask);
    iqs_force_comms(tile);
    iqs_write(tile, reg, r);
}

/** Poll System Status until ATI Active clears, within IQS323_ATI_TIMEOUT_MS
 *  of real time. Returns the final status, or 0xEEEE on timeout. */
static uint16_t iqs_wait_ati(tile_t *tile)
{
    uint32_t elapsed = 0;
    while (elapsed < IQS323_ATI_TIMEOUT_MS) {
        tile->hal->delay_ms(10);
        elapsed += 10;
        elapsed += iqs_force_comms(tile);
        uint16_t st = iqs_read(tile, IQS323_REG_SYSTEM_STATUS);
        if (st != IQS323_INVALID_RESPONSE && !(st & IQS323_STATUS_ATI_ACTIVE))
            return st;
    }
    return IQS323_INVALID_RESPONSE;
}

/** System Control value for the host's power mode and interface choice. */
static uint16_t iqs_ctrl_word(const iqs323_state_t *s)
{
    return (uint16_t)(((uint16_t)s->power_mode << IQS323_CTRL_POWER_SHIFT)
                      & IQS323_CTRL_POWER_MASK)
         | (s->comm_mode == SENSE_T_C_COMM_EVENT ? IQS323_CTRL_INTERFACE_SEL : 0);
}

/* Tile wiring (schematic): the top surface is on CRx1/CTx1 through 470 Ω,
 * pad 8 on CRx0/CTx0, CRx2 is not routed. Self-capacitance needs the same
 * CRx and CTx selected (datasheet §5.12.1); after reset every channel selects
 * CRx0/CTx0 (Sensor Setup 0x0101, Prox Input 0x01CF), so CH1 must be moved
 * to its own pin. Prox Control's reset value is already self-capacitance. */
#define IQS323_SENSOR_SETUP_EN(ch)  ((uint16_t)((1U << (8 + (ch))) | 0x0001))
#define IQS323_PROX_INPUT_RX(ch)    ((uint16_t)((1U << (8 + (ch))) | 0x00CF))

/* Settings after a reset (§9 gives 0x0000 for all of these): Azoteq's EV-kit
 * values (IQS323_3PROJ_BUTTONS_EV_KIT.h), which the datasheet's current table
 * (§3.4) is measured at. */
#define IQS323_DEF_PROX_THRESHOLD   20      /* delta counts, 4/4 debounce */
#define IQS323_DEF_TOUCH_THRESHOLD  40      /* x/256 of the LTA */
#define IQS323_DEF_COUNTS_FILTER    0x0202  /* LP 2, NP 2 */
#define IQS323_DEF_LTA_FILTER       0x0505  /* LP 5, NP 5 */
#define IQS323_DEF_LTA_FAST_FILTER  0x0303  /* LP 3, NP 3 */
#define IQS323_DEF_FAST_FILTER_BAND 30
#define IQS323_DEF_NP_RATE_MS       16
#define IQS323_DEF_LP_RATE_MS       60
#define IQS323_DEF_ULP_RATE_MS      160
#define IQS323_DEF_HALT_RATE_MS     3000
#define IQS323_DEF_POWER_TIMEOUT_MS 2000

static uint16_t prox_settings(uint8_t threshold)
{
    /* Prox Settings (A.16): [15:12] debounce-exit, [11:8] debounce-enter,
     * [7:0] threshold. Azoteq's reference 4/4 debounce. */
    return (uint16_t)((4U << 12) | (4U << 8) | threshold);
}

/**
 * Bring the chip from its reset state to the configured one: acknowledge the
 * reset, write every setting, run ATI, then select the interface. Runs from
 * init() and again from process() whenever the chip reports a reset event
 * (datasheet §6.6.1: after a reset its settings revert and must be re-written).
 */
static void iqs_configure(tile_t *tile)
{
    iqs323_state_t *s = state_for(tile);

    /* ACK the reset first: until then the chip opens windows continuously
     * and can't enter event mode (§6.6.1, §8.12). */
    iqs_force_comms(tile);
    iqs_write(tile, IQS323_REG_SYSTEM_CONTROL, IQS323_CTRL_ACK_RESET);
    tile->hal->delay_ms(10);

    for (uint8_t ch = 0; ch < SENSE_T_C_NUM_CHANNELS; ch++) {
        if (s->channels & (1U << ch)) {
            iqs_write_verified(tile, IQS323_REG_SENSOR_SETUP(ch), IQS323_SENSOR_SETUP_EN(ch));
            iqs_write_verified(tile, IQS323_REG_PROX_INPUT(ch), IQS323_PROX_INPUT_RX(ch));
            iqs_write_verified(tile, IQS323_REG_PROX_SETTINGS(ch), prox_settings(s->prox_threshold));
            iqs_write_verified(tile, IQS323_REG_TOUCH_SETTINGS(ch), s->touch_threshold);
        } else {
            /* Disabled: no Tx, channel off. */
            iqs_write_verified(tile, IQS323_REG_SENSOR_SETUP(ch), 0x0000);
        }
    }

    iqs_write_verified(tile, IQS323_REG_COUNTS_FILTER, IQS323_DEF_COUNTS_FILTER);
    iqs_write_verified(tile, IQS323_REG_LTA_FILTER, IQS323_DEF_LTA_FILTER);
    iqs_write_verified(tile, IQS323_REG_LTA_FAST_FILTER, IQS323_DEF_LTA_FAST_FILTER);
    iqs_write_verified(tile, IQS323_REG_FAST_FILTER_BAND, IQS323_DEF_FAST_FILTER_BAND);
    iqs_write_verified(tile, IQS323_REG_NP_REPORT_RATE, IQS323_DEF_NP_RATE_MS);
    iqs_write_verified(tile, IQS323_REG_LP_REPORT_RATE, IQS323_DEF_LP_RATE_MS);
    iqs_write_verified(tile, IQS323_REG_ULP_REPORT_RATE, IQS323_DEF_ULP_RATE_MS);
    iqs_write_verified(tile, IQS323_REG_HALT_REPORT_RATE, IQS323_DEF_HALT_RATE_MS);
    iqs_write_verified(tile, IQS323_REG_POWER_TIMEOUT, IQS323_DEF_POWER_TIMEOUT_MS);

    /* Touch + prox events drive RDY in event mode. */
    iqs_write_verified(tile, IQS323_REG_EVENTS_ENABLE,
                       IQS323_STATUS_TOUCH_EVENT | IQS323_STATUS_PROX_EVENT);

    /* Re-ATI in streaming mode, then switch to the chosen interface
     * (Azoteq's program flow, §8.15: event mode goes on last). */
    iqs_force_comms(tile);
    iqs_write(tile, IQS323_REG_SYSTEM_CONTROL,
              (uint16_t)(((uint16_t)s->power_mode << IQS323_CTRL_POWER_SHIFT)
                         & IQS323_CTRL_POWER_MASK) | IQS323_CTRL_RE_ATI);
    iqs_wait_ati(tile);

    iqs_force_comms(tile);
    iqs_write(tile, IQS323_REG_SYSTEM_CONTROL, iqs_ctrl_word(s));

    s->ati_retries = 0;
    s->last_status = 0;
}

/* ---- RDY pin EXTI callback ---- */

/* RDY pin EXTI callbacks — used when rdy_pin is configured.
 * Referenced by pointer in init(), so the compiler won't warn. */
static void _rdy_isr_0(void *ctx) { state[0].rdy_flag = 1; (void)ctx; }
static void _rdy_isr_1(void *ctx) { state[1].rdy_flag = 1; (void)ctx; }

/* ================================================================
 * Lifecycle
 * ================================================================ */

uint8_t tile_sense_t_c_find(tiles_pal_t *hal, uint8_t instance)
{
    uint8_t addr = resolve_id(instance);
    if (!addr) return 0;
    return hal->i2c_is_ready(hal->handle, addr) == 0;
}

void tile_sense_t_c_init(tiles_pal_t *hal, uint8_t instance,
                         tile_t *tile, const sense_t_c_cfg_t *cfg)
{
    memzero(tile, sizeof(tile_t));
    tile->hal = hal;
    tile->id  = resolve_id(instance);

    if (!tile->id) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_t_c: invalid instance");
        return;
    }

    /* Initialize per-instance state. A re-init drops the old RDY handler
     * first, so the forced windows below use the fixed wait. */
    iqs323_state_t *s = state_for(tile);
    if (s->irq_armed && s->rdy_pin && hal->gpio_irq_disable)
        hal->gpio_irq_disable(hal->handle, s->rdy_pin);
    memzero(s, sizeof(iqs323_state_t));
    if (cfg) {
        s->on_event = cfg->on_event;
        s->event_ctx = cfg->event_ctx;
        s->rdy_pin = cfg->rdy_pin;
        s->prox_threshold = cfg->prox_threshold;
        s->touch_threshold = cfg->touch_threshold;
        s->channels = cfg->channels & SENSE_T_C_CHANNELS_ROUTED;
    }
    if (!s->channels) s->channels = SENSE_T_C_CHANNELS_ROUTED;
    if (!s->prox_threshold) s->prox_threshold = IQS323_DEF_PROX_THRESHOLD;
    if (!s->touch_threshold) s->touch_threshold = IQS323_DEF_TOUCH_THRESHOLD;
    s->power_mode = SENSE_T_C_POWER_AUTO;
    s->comm_mode = SENSE_T_C_COMM_EVENT;

    /* Probe bus */
    if (hal->i2c_is_ready(hal->handle, tile->id) != 0) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_t_c: device not found");
        return;
    }

    /* Read Hardware ID */
    uint16_t hw_id = iqs_read_window(tile, IQS323_REG_HARDWARE_ID);
    tile->flags = (uint8_t)(hw_id & 0xFF);
    if (hw_id == IQS323_INVALID_RESPONSE) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_t_c: no communication window");
        return;
    }
    if (hw_id != IQS323_HW_ID_3DD && hw_id != IQS323_HW_ID_3ED) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_t_c: unexpected hardware ID");
        return;
    }

    /* Soft reset, so the chip starts from known settings whatever ran
     * before, then configure it (ACK, settings, ATI, interface). */
    iqs_force_comms(tile);
    iqs_write(tile, IQS323_REG_SYSTEM_CONTROL, IQS323_CTRL_SOFT_RESET);
    hal->delay_ms(20);
    iqs_configure(tile);

    /* Set up RDY pin interrupt if configured */
    s->rdy_flag = 0;
    if (s->rdy_pin && hal->gpio_irq_enable) {
        void (*isr)(void *) = (instance == 0) ? _rdy_isr_0 : _rdy_isr_1;
        s->irq_armed = hal->gpio_irq_enable(hal->handle, s->rdy_pin,
                                            TILES_GPIO_EDGE_FALLING, isr, NULL) == 0;
    }

    tile->state = TILE_STATE_READY;
}

/* ================================================================
 * Event processing
 * ================================================================ */

void tile_sense_t_c_process(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY) return;

    iqs323_state_t *s = state_for(tile);

    /* In RDY mode, skip if no interrupt fired */
    if (s->irq_armed && !s->rdy_flag)
        return;
    s->rdy_flag = 0;

    /* Read status. In RDY mode the window is already open (RDY went LOW),
     * so skip force_comms. In polled mode, force a window first. */
    if (!s->irq_armed)
        iqs_force_comms(tile);
    uint16_t status = iqs_read(tile, IQS323_REG_SYSTEM_STATUS);
    if (status == IQS323_INVALID_RESPONSE)
        return;

    /* The chip reset (brown-out, watchdog, MCLR): its settings are back at
     * their reset values, so configure it again before trusting any data. */
    if (status & IQS323_STATUS_RESET_EVENT) {
        iqs_configure(tile);
        return;
    }

    /* ATI couldn't reach its target: retry a few times, then leave the
     * flag for the application to see in the status word. */
    if (status & IQS323_STATUS_ATI_ERROR) {
        if (s->ati_retries < IQS323_ATI_RETRY_MAX) {
            s->ati_retries++;
            iqs_modify(tile, IQS323_REG_SYSTEM_CONTROL,
                       IQS323_CTRL_RE_ATI, IQS323_CTRL_RE_ATI);
        }
    } else {
        s->ati_retries = 0;
    }

    /* Fire callback on every valid read so LED tracks state in real time */
    if (s->on_event) {
        s->on_event(tile, status, s->event_ctx);
    }
    s->last_status = status;
}

void tile_sense_t_c_on_event(tile_t *tile, sense_t_c_event_cb_t cb, void *ctx)
{
    iqs323_state_t *s = state_for(tile);
    s->on_event = cb;
    s->event_ctx = ctx;
}

/* ================================================================
 * Lifecycle
 * ================================================================ */

void tile_sense_t_c_sleep(tile_t *tile)
{
    iqs_modify(tile, IQS323_REG_SYSTEM_CONTROL,
               IQS323_CTRL_POWER_MASK,
               SENSE_T_C_POWER_HALT << IQS323_CTRL_POWER_SHIFT);
    tile->state = TILE_STATE_SLEEPING;
}

void tile_sense_t_c_wake(tile_t *tile)
{
    /* Back to the mode the host last chose (init: AUTO), not a fixed one. */
    iqs323_state_t *s = state_for(tile);
    iqs_modify(tile, IQS323_REG_SYSTEM_CONTROL,
               IQS323_CTRL_POWER_MASK,
               (uint16_t)s->power_mode << IQS323_CTRL_POWER_SHIFT);
    tile->state = TILE_STATE_READY;
}

void tile_sense_t_c_reset(tile_t *tile)
{
    iqs_force_comms(tile);
    iqs_write(tile, IQS323_REG_SYSTEM_CONTROL, IQS323_CTRL_SOFT_RESET);
    tile->hal->delay_ms(20);
    tile->state = TILE_STATE_NONE;
}

/* ================================================================
 * Status (cached from last process() call)
 * ================================================================ */

uint16_t tile_sense_t_c_get_status(tile_t *tile)
{
    iqs323_state_t *s = state_for(tile);
    return s->last_status;
}

uint16_t tile_sense_t_c_get_gestures(tile_t *tile)
{
    /* A live read (not cached by process()). */
    return iqs_read_window(tile, IQS323_REG_GESTURE_STATUS);
}

uint8_t tile_sense_t_c_is_touched(tile_t *tile, uint8_t channel)
{
    if (channel >= SENSE_T_C_NUM_CHANNELS) return 0;
    iqs323_state_t *s = state_for(tile);
    return (s->last_status & IQS323_STATUS_CH_TOUCH(channel)) ? 1 : 0;
}

uint8_t tile_sense_t_c_is_prox(tile_t *tile, uint8_t channel)
{
    if (channel >= SENSE_T_C_NUM_CHANNELS) return 0;
    iqs323_state_t *s = state_for(tile);
    return (s->last_status & IQS323_STATUS_CH_PROX(channel)) ? 1 : 0;
}

/* ================================================================
 * Data
 * ================================================================ */

uint16_t tile_sense_t_c_get_counts(tile_t *tile, uint8_t channel)
{
    if (channel >= SENSE_T_C_NUM_CHANNELS) return 0;
    return iqs_read_window(tile, IQS323_REG_CH0_COUNTS + channel * 2);
}

uint16_t tile_sense_t_c_get_lta(tile_t *tile, uint8_t channel)
{
    if (channel >= SENSE_T_C_NUM_CHANNELS) return 0;
    return iqs_read_window(tile, IQS323_REG_CH0_LTA + channel * 2);
}

int16_t tile_sense_t_c_get_delta(tile_t *tile, uint8_t channel)
{
    uint16_t counts = tile_sense_t_c_get_counts(tile, channel);
    uint16_t lta    = tile_sense_t_c_get_lta(tile, channel);
    if (counts == IQS323_INVALID_RESPONSE || lta == IQS323_INVALID_RESPONSE)
        return 0;
    return (int16_t)(counts - lta);
}

uint16_t tile_sense_t_c_get_slider(tile_t *tile)
{
    return iqs_read_window(tile, IQS323_REG_SLIDER_POSITION);
}

/* ================================================================
 * Configuration
 * ================================================================ */

void tile_sense_t_c_set_thresholds(tile_t *tile, uint8_t channel,
                                   uint8_t prox_thresh, uint8_t touch_thresh)
{
    if (channel >= SENSE_T_C_NUM_CHANNELS) return;
    if (prox_thresh)
        iqs_write_verified(tile, IQS323_REG_PROX_SETTINGS(channel),
                           prox_settings(prox_thresh));
    if (touch_thresh) {
        /* Touch Settings (A.17): [15:12] hysteresis, [11:8] reserved,
         * [7:0] threshold. Hysteresis 0; the old (2<<8) hit reserved bits. */
        iqs_write_verified(tile, IQS323_REG_TOUCH_SETTINGS(channel), touch_thresh);
    }
}

void tile_sense_t_c_set_touch_threshold(tile_t *tile, uint8_t threshold)
{
    if (!threshold) return;
    iqs323_state_t *s = state_for(tile);
    s->touch_threshold = threshold;  /* survives a chip reset */
    for (uint8_t ch = 0; ch < SENSE_T_C_NUM_CHANNELS; ch++)
        if (s->channels & (1U << ch))
            iqs_write_verified(tile, IQS323_REG_TOUCH_SETTINGS(ch), threshold);
}

void tile_sense_t_c_set_prox_threshold(tile_t *tile, uint8_t threshold)
{
    if (!threshold) return;
    iqs323_state_t *s = state_for(tile);
    s->prox_threshold = threshold;  /* survives a chip reset */
    for (uint8_t ch = 0; ch < SENSE_T_C_NUM_CHANNELS; ch++)
        if (s->channels & (1U << ch))
            iqs_write_verified(tile, IQS323_REG_PROX_SETTINGS(ch), prox_settings(threshold));
}

void tile_sense_t_c_set_power_mode(tile_t *tile, sense_t_c_power_mode_t mode)
{
    if ((unsigned)mode > SENSE_T_C_POWER_AUTO_NO_ULP) return;  /* 6, 7 reserved */
    state_for(tile)->power_mode = (uint8_t)mode;
    iqs_modify(tile, IQS323_REG_SYSTEM_CONTROL,
               IQS323_CTRL_POWER_MASK,
               (uint16_t)mode << IQS323_CTRL_POWER_SHIFT);
}

void tile_sense_t_c_enable_events(tile_t *tile, uint16_t mask)
{
    /* Events Enable (A.33): bits 0-4 and 6 are defined; the rest reserved. */
    iqs_write_verified(tile, IQS323_REG_EVENTS_ENABLE, mask & IQS323_EVENTS_DEFINED);
}

void tile_sense_t_c_ati(tile_t *tile)
{
    iqs_modify(tile, IQS323_REG_SYSTEM_CONTROL,
               IQS323_CTRL_RE_ATI, IQS323_CTRL_RE_ATI);
    iqs_wait_ati(tile);
}

/* ================================================================
 * ATI fine-tuning + filter / conversion-frequency tuning
 * ================================================================ */

void tile_sense_t_c_set_ati_setup(tile_t *tile, uint8_t channel, uint16_t value)
{
    if (channel >= SENSE_T_C_NUM_CHANNELS) return;
    iqs_write_verified(tile, IQS323_REG_ATI_SETUP(channel), value);
}

void tile_sense_t_c_set_counts_filter(tile_t *tile, uint16_t beta)
{
    iqs_write_verified(tile, IQS323_REG_COUNTS_FILTER, beta);
}

void tile_sense_t_c_set_conversion_freq(tile_t *tile, uint8_t channel,
                                        uint16_t value)
{
    if (channel >= SENSE_T_C_NUM_CHANNELS) return;
    iqs_write_verified(tile, IQS323_REG_CONV_FREQ(channel), value);
}

/* ================================================================
 * Reference UI — channel mode + reference sensor ID
 * ================================================================ */

void tile_sense_t_c_set_channel_mode(tile_t *tile, uint8_t channel,
                                     sense_t_c_channel_mode_t mode,
                                     uint8_t reference_id)
{
    if (channel >= SENSE_T_C_NUM_CHANNELS) return;

    /* Channel Setup register: bits [3:0] = Channel Mode,
     * bits [7:4] = Reference Sensor ID. Preserve the upper byte
     * (Follower Event Mask). */
    uint16_t low = ((uint16_t)reference_id & 0x0F) << 4
                 | ((uint16_t)mode & 0x0F);
    iqs_modify(tile, IQS323_REG_CH_SETUP(channel), 0x00FF, low);
}

/* ================================================================
 * I²C communication mode — event vs streaming
 * ================================================================ */

void tile_sense_t_c_set_comm_mode(tile_t *tile, sense_t_c_comm_mode_t mode)
{
    /* System Control bit 7 = Interface Selection
     *   0 = I²C streaming, 1 = I²C event. Preserve other bits. */
    state_for(tile)->comm_mode = (mode == SENSE_T_C_COMM_EVENT)
                               ? SENSE_T_C_COMM_EVENT : SENSE_T_C_COMM_STREAM;
    iqs_modify(tile, IQS323_REG_SYSTEM_CONTROL, IQS323_CTRL_INTERFACE_SEL,
               (mode == SENSE_T_C_COMM_EVENT) ? IQS323_CTRL_INTERFACE_SEL : 0);
}

/* ================================================================
 * Tier-2 idiomatic helpers
 * ================================================================ */

/* Bitmask of all per-channel touch bits (CH0/CH1/CH2). */
#define IQS323_STATUS_ANY_TOUCH \
    (IQS323_STATUS_CH_TOUCH(0) | IQS323_STATUS_CH_TOUCH(1) | IQS323_STATUS_CH_TOUCH(2))

/* Bitmask covering every gesture event bit defined in §A.2. */
#define IQS323_GESTURE_ANY \
    (IQS323_GESTURE_TAP | IQS323_GESTURE_SWIPE_POS | IQS323_GESTURE_SWIPE_NEG | \
     IQS323_GESTURE_FLICK_POS | IQS323_GESTURE_FLICK_NEG | IQS323_GESTURE_HOLD)

/* Poll cadence for wait_for_* helpers (ms). 5ms keeps latency low while
 * leaving plenty of headroom for the host's other work; in RDY mode the
 * underlying process() returns immediately when no event is pending. */
#define SENSE_T_C_POLL_INTERVAL_MS  5

uint8_t tile_sense_t_c_is_touched_any(tile_t *tile)
{
    iqs323_state_t *s = state_for(tile);
    return (s->last_status & IQS323_STATUS_ANY_TOUCH) ? 1 : 0;
}

uint8_t tile_sense_t_c_wait_for_touch(tile_t *tile, uint32_t timeout_ms)
{
    if (tile->state != TILE_STATE_READY) return 0;

    /* In polled mode each process() also waits for a forced window;
     * count that too, so the timeout is real time. */
    const uint32_t per_poll = SENSE_T_C_POLL_INTERVAL_MS
                            + (state_for(tile)->irq_armed ? 0 : IQS323_TWAIT_POLLED_MS);
    uint32_t elapsed = 0;
    /* First sample: process() may already have a fresh status. */
    tile_sense_t_c_process(tile);
    if (tile_sense_t_c_is_touched_any(tile))
        return 1;

    while (elapsed < timeout_ms) {
        tile->hal->delay_ms(SENSE_T_C_POLL_INTERVAL_MS);
        elapsed += per_poll;
        tile_sense_t_c_process(tile);
        if (tile_sense_t_c_is_touched_any(tile))
            return 1;
    }
    return 0;
}

uint16_t tile_sense_t_c_wait_for_gesture(tile_t *tile, uint32_t timeout_ms)
{
    if (tile->state != TILE_STATE_READY) return 0;

    /* Gesture status is read directly (not cached by process()) so we
     * always pull a fresh value here; one forced window per poll, its
     * wait counted against the timeout. */
    uint32_t elapsed = 0;
    for (;;) {
        elapsed += iqs_force_comms(tile);
        uint16_t g = iqs_read(tile, IQS323_REG_GESTURE_STATUS);
        if (g != IQS323_INVALID_RESPONSE && (g & IQS323_GESTURE_ANY))
            return g;
        if (elapsed >= timeout_ms)
            return 0;
        tile->hal->delay_ms(SENSE_T_C_POLL_INTERVAL_MS);
        elapsed += SENSE_T_C_POLL_INTERVAL_MS;
    }
}

uint8_t tile_sense_t_c_read_slider_pct(tile_t *tile, uint8_t *out_pct)
{
    if (!out_pct) return 0;

    /* The position runs 0..Slider Resolution (register 0x93, datasheet
     * §7.4 "Slider"), not a fixed 10-bit range. Resolution 0 (the reset
     * default) means no slider has been configured. */
    uint16_t res = iqs_read_window(tile, IQS323_REG_SLIDER_RESOLUTION);
    if (res == IQS323_INVALID_RESPONSE || res == 0)
        return 0;
    uint16_t raw = iqs_read_window(tile, IQS323_REG_SLIDER_POSITION);
    if (raw == IQS323_INVALID_RESPONSE)
        return 0;

    if (raw > res) raw = res;
    *out_pct = (uint8_t)(((uint32_t)raw * 100u) / res);
    return 1;
}

/* ================================================================
 * Low-level register access
 * ================================================================ */

uint16_t tile_sense_t_c_read_reg(tile_t *tile, uint8_t reg)
{
    return iqs_read_window(tile, reg);
}

void tile_sense_t_c_write_reg(tile_t *tile, uint8_t reg, uint16_t value)
{
    /* System Control's trigger bits clear themselves, so it can't be
     * verified by reading back; everything else is. */
    if (reg == IQS323_REG_SYSTEM_CONTROL) {
        iqs_force_comms(tile);
        iqs_write(tile, reg, value);
        return;
    }
    iqs_write_verified(tile, reg, value);
}

/* ================================================================
 * v1.3 capability additions
 * ================================================================ */

void tile_sense_t_c_reseed(tile_t *tile)
{
    /* RESEED is a self-clearing trigger bit in SYSTEM_CONTROL. */
    iqs_modify(tile, IQS323_REG_SYSTEM_CONTROL,
               IQS323_CTRL_RESEED, IQS323_CTRL_RESEED);
}

void tile_sense_t_c_set_report_rate(tile_t *tile, sense_t_c_power_mode_t mode,
                                    uint16_t ms)
{
    uint8_t reg;
    switch (mode) {
    case SENSE_T_C_POWER_NORMAL:    reg = IQS323_REG_NP_REPORT_RATE;   break;
    case SENSE_T_C_POWER_LOW:       reg = IQS323_REG_LP_REPORT_RATE;   break;
    case SENSE_T_C_POWER_ULTRA_LOW: reg = IQS323_REG_ULP_REPORT_RATE;  break;
    case SENSE_T_C_POWER_HALT:      reg = IQS323_REG_HALT_REPORT_RATE; break;
    default: return;  /* AUTO / AUTO_NO_ULP have no rate register */
    }
    if (ms > 3000) ms = 3000;
    iqs_write_verified(tile, reg, ms);
}

void tile_sense_t_c_set_power_timeout(tile_t *tile, uint16_t ms)
{
    if (ms > 65000) ms = 65000;  /* §9: range 0-65000 */
    iqs_write_verified(tile, IQS323_REG_POWER_TIMEOUT, ms);
}

uint16_t tile_sense_t_c_get_compensation(tile_t *tile, uint8_t channel)
{
    if (channel >= SENSE_T_C_NUM_CHANNELS) return 0;
    uint16_t r = iqs_read_window(tile, IQS323_REG_COMPENSATION(channel));
    return (r == IQS323_INVALID_RESPONSE) ? 0 : r;
}

void tile_sense_t_c_set_compensation(tile_t *tile, uint8_t channel,
                                     uint16_t value, uint8_t divider)
{
    if (channel >= SENSE_T_C_NUM_CHANNELS) return;
    /* Compensation (A.14): [15:11] divider, [9:0] value, bit 10 reserved. */
    uint16_t reg = (uint16_t)(((uint16_t)(divider & 0x1F) << 11)
                            | (value & 0x03FF));
    iqs_write_verified(tile, IQS323_REG_COMPENSATION(channel), reg);
}
