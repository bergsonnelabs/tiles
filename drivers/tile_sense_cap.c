/**
 * @file   tile_sense_cap.c
 * @brief  Sense.CAP (IQS7211A) -- platform-agnostic driver implementation.
 *
 * Platform-agnostic. All bus access via tile->hal function pointers.
 * Reference: Azoteq IQS7211A Datasheet v1.3, April 2025.
 *
 * Comms model: the device only serves registers inside a communication
 * window. In streaming mode it clock-stretches an early master and plain
 * reads work; in event or low-power modes, or with Comms Request enabled,
 * a read instead returns 0xEEEE and the master must ask for a window
 * (write 0x00 to 0xFF, wait, retry) — one window, one transaction. Every
 * access here retries behind that request, and nothing derived from an
 * 0xEEEE read is ever written back: 0xEEEE carries Comms Request, Event
 * Mode, Manual Control and WDT, so writing it strands the part in
 * request-only mode until it is power-cycled. Observed the hard way on a
 * Core.ST.L4.1 at 100 kHz, 2026-08-04.
 *
 * Register data is 16-bit little-endian, two bytes per 8-bit register
 * address, and reads auto-increment through consecutive addresses
 * (§11.5) — which is what lets process() pull the whole trackpad block
 * in one transaction.
 */

#include "tile_sense_cap.h"
#include <stddef.h>

/* ================================================================
 * Instance -> I2C address table
 * ================================================================ */

/* The IQS7211A address is fixed in silicon, so there is exactly one
 * instance per bus. The table keeps the shape of the other drivers. */
static const uint8_t id_table[] = {
    IQS7211A_I2C_ADDR,   /* 0: 0x56 */
};

#define NUM_INSTANCES  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    return (instance < NUM_INSTANCES) ? id_table[instance] : 0;
}

/* ================================================================
 * Per-instance driver state
 * ================================================================ */

/* Offsets into the cached data block, in registers from 0x10. */
#define BLK_INFO_FLAGS   0
#define BLK_GESTURES     1
#define BLK_RELATIVE_X   2
#define BLK_RELATIVE_Y   3
#define BLK_FINGER_BASE  4   /* finger n: base + n*4 = X, Y, strength, area */
#define BLK_WORDS        12  /* 0x10..0x1B */

/* Touch-recognizer timing/geometry constants (integer ms / pixels). */
#define TOUCH_TAP_MS      300   /* max down-time for a tap */
#define TOUCH_DOUBLE_MS   350   /* max gap between taps for a double */
#define TOUCH_HOLD_MS     600   /* stationary time for a long press */
#define TOUCH_TAP_DIST    48    /* movement budget before a tap becomes a drag */
#define TOUCH_MOVE_DEAD   4     /* movement below this is jitter, not MOVED */
#define TOUCH_PINCH_DEAD  3     /* pinch-distance change below this is jitter */
#define TOUCH_EVQ_LEN     8     /* ring-buffer depth for next_touch_event() */

#define XY_INVALID        0xFFFF  /* chip's "no valid coordinate" sentinel */

typedef struct {
    sense_cap_event_cb_t on_event;
    void *event_ctx;
    uint16_t block[BLK_WORDS];     /* Cached 0x10..0x1B from last process() */
    uint16_t version_major;
    uint16_t version_minor;
    uint16_t config_shadow;        /* Last known-good Config Settings (0x51) */
    uint8_t  config_valid;         /* 1 once config_shadow holds a real read */
    volatile uint8_t rdy_flag;     /* Set by EXTI ISR */
    uint8_t rdy_pin;               /* Core pad number, 0 = polled */
    uint8_t num_channels;          /* rx*tx once a surface is configured */

    /* ---- surface geometry cache (for zone math) ---- */
    uint8_t  srf_rx, srf_tx;       /* electrode counts */
    uint8_t  srf_switch_xy;        /* 1 = X along Txs */
    uint16_t x_res, y_res;         /* output resolution */

    /* ---- clock ---- */
    uint32_t (*millis)(void *);    /* user clock, or NULL */
    void *millis_ctx;
    uint32_t tick_ms;              /* fallback clock, advanced per process() */
    uint16_t est_period_ms;        /* fallback increment (report-rate guess) */

    /* ---- touch state machine ---- */
    uint16_t px[2], py[2];         /* last position per finger slot */
    uint8_t  presence_cnt[2];      /* consecutive cycles in a flipped state */
    uint16_t down_x[2], down_y[2]; /* position at DOWN */
    uint32_t down_t[2];            /* time of DOWN */
    uint32_t move_t[2];            /* time of last MOVED */
    uint8_t  was_down;             /* bit per slot */
    uint8_t  moved_far;            /* bit per slot: beyond tap radius */
    uint8_t  hold_fired;           /* bit per slot */
    int16_t  vx, vy;               /* latest tracked velocity */
    uint32_t last_tap_t;           /* for double-tap pairing */
    uint32_t last_up_t;            /* for gating chip gestures on touch */
    uint16_t pinch_dist;
    uint8_t  pinch_valid;
    uint16_t prev_gestures;        /* edge detector for chip gesture bits */
    uint16_t ev_latch;             /* SENSE_CAP_EV_* accumulator */

    /* ---- event queue ---- */
    sense_cap_touch_t evq[TOUCH_EVQ_LEN];
    uint8_t evq_rd, evq_count;

    /* ---- recognizer callbacks ---- */
    sense_cap_touch_cb_t on_touch;  void *touch_ctx;
    sense_cap_tap_cb_t   on_tap;    void *tap_ctx;
    sense_cap_hold_cb_t  on_hold;   void *hold_ctx;
    sense_cap_swipe_cb_t on_swipe;  void *swipe_ctx;
    sense_cap_drag_cb_t  on_drag;   void *drag_ctx;
    sense_cap_pinch_cb_t on_pinch;  void *pinch_ctx;
} iqs7211a_state_t;

static iqs7211a_state_t state[NUM_INSTANCES];

static iqs7211a_state_t *state_for(tile_t *tile)
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

/** How many comms-request retries before giving a read up as failed. */
#define IQS_WINDOW_RETRIES  8
/** Settling time after a comms request, in ms (t_max is well under this). */
#define IQS_WINDOW_WAIT_MS  15

/**
 * Ask the device to open a communication window (§11.9.2): write 0x00 to
 * register 0xFF, then let it schedule the window. Also the terminate-comms
 * command in the other direction (§11.7) — same bytes, and harmless when a
 * window is already open.
 */
static void iqs_request_window(tile_t *tile)
{
    uint8_t zero = 0;
    tile->hal->i2c_write(tile->hal->handle, tile->id,
                         IQS7211A_REG_END_COMMS, &zero, 1);
    tile->hal->delay_ms(IQS_WINDOW_WAIT_MS);
}

/** Single raw read of a 16-bit little-endian register. */
static uint16_t iqs_read_once(tile_t *tile, uint8_t reg)
{
    uint8_t buf[2] = {0, 0};
    tile->hal->i2c_read(tile->hal->handle, tile->id, reg, buf, 2);
    return (uint16_t)(((uint16_t)buf[1] << 8) | buf[0]);
}

/**
 * Read a register, falling back to an explicit comms request.
 *
 * Returns IQS7211A_INVALID_RESPONSE if no window could be obtained —
 * callers must check before using the value, and must never write it
 * back.
 */
static uint16_t iqs_read(tile_t *tile, uint8_t reg)
{
    uint16_t v = iqs_read_once(tile, reg);
    for (uint8_t i = 0; v == IQS7211A_INVALID_RESPONSE && i < IQS_WINDOW_RETRIES; i++) {
        iqs_request_window(tile);
        v = iqs_read_once(tile, reg);
    }
    return v;
}

/**
 * Read a 16-bit register from the extended memory map (16-bit address).
 * The PAL sends two address bytes MSB-first when reg > 0xFF, which is
 * exactly the extended-map access format (§11.5). Same window-retry
 * shape as iqs_read().
 */
static uint16_t iqs_read_ext(tile_t *tile, uint16_t reg)
{
    uint8_t buf[2] = {0, 0};
    tile->hal->i2c_read(tile->hal->handle, tile->id, reg, buf, 2);
    uint16_t v = (uint16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    for (uint8_t i = 0; v == IQS7211A_INVALID_RESPONSE && i < IQS_WINDOW_RETRIES; i++) {
        iqs_request_window(tile);
        tile->hal->i2c_read(tile->hal->handle, tile->id, reg, buf, 2);
        v = (uint16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    }
    return v;
}

/**
 * Read `n` consecutive 16-bit extended-map words in ONE transaction (the
 * address auto-increments, §11.5). One window serves the whole block, so
 * a 6-channel read costs one window wait instead of six. Same retry shape.
 *
 * @return 1 on success, 0 if the block never came back valid.
 */
static uint8_t iqs_read_ext_block(tile_t *tile, uint16_t reg, uint16_t *out, uint8_t n)
{
    uint8_t buf[64];
    if (n == 0 || n > 32) return 0;
    for (uint8_t attempt = 0; attempt <= IQS_WINDOW_RETRIES; attempt++) {
        if (attempt) iqs_request_window(tile);
        tile->hal->i2c_read(tile->hal->handle, tile->id, reg, buf, (uint16_t)(n * 2));
        uint16_t first = (uint16_t)(((uint16_t)buf[1] << 8) | buf[0]);
        if (first == IQS7211A_INVALID_RESPONSE) continue;
        for (uint8_t i = 0; i < n; i++)
            out[i] = (uint16_t)(((uint16_t)buf[2 * i + 1] << 8) | buf[2 * i]);
        return 1;
    }
    return 0;
}

/** Write a 16-bit little-endian register. */
static void iqs_write(tile_t *tile, uint8_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(value & 0xFF);
    buf[1] = (uint8_t)((value >> 8) & 0xFF);
    tile->hal->i2c_write(tile->hal->handle, tile->id, reg, buf, 2);
}

/**
 * Write a register and confirm it landed, requesting a window between
 * attempts. A write outside a window is silently dropped, so a blind
 * write is not enough.
 *
 * @return 1 if the readback matched, 0 if every attempt failed.
 */
static uint8_t iqs_write_verified(tile_t *tile, uint8_t reg, uint16_t value)
{
    for (uint8_t i = 0; i < IQS_WINDOW_RETRIES; i++) {
        /* Ask first, every time. One window serves one transaction, and
         * the STOP that ends this write closes it again — so the readback
         * below needs its own window, which iqs_read() arranges. */
        iqs_request_window(tile);
        iqs_write(tile, reg, value);

        if (iqs_read(tile, reg) == value)
            return 1;
    }
    return 0;
}

/**
 * Read-modify-write a 16-bit register.
 *
 * Bails out without writing when the read is invalid — writing a value
 * derived from 0xEEEE is what strands the device in request-only mode.
 */
static void iqs_modify(tile_t *tile, uint8_t reg, uint16_t clear, uint16_t set)
{
    uint16_t v = iqs_read(tile, reg);
    if (v == IQS7211A_INVALID_RESPONSE) {
        TILE_ON_ERROR(tile, "sense_cap: no comms window; register left alone");
        return;
    }
    v = (uint16_t)((v & (uint16_t)~clear) | set);
    iqs_write_verified(tile, reg, v);
}

/**
 * Pulse a System Control command bit.
 *
 * Every bit in System Control is a one-shot the device consumes, so only
 * the mode-select field has to survive the write. Deliberately does NOT
 * OR onto the whole read: carrying unknown high bits back in could set
 * Tx-test or a software reset, and an invalid read would set both.
 */
static void iqs_command(tile_t *tile, uint16_t bit)
{
    uint16_t ctrl = iqs_read(tile, IQS7211A_REG_SYSTEM_CONTROL);
    uint16_t mode = (ctrl == IQS7211A_INVALID_RESPONSE)
                        ? 0u
                        : (uint16_t)(ctrl & IQS7211A_CTRL_MODE_MASK);

    /* The read above ended in a STOP, which closed whatever window served
     * it, so this write needs a fresh one — unconditionally. Skipping it
     * is why an unverified command silently does nothing. Command bits
     * self-clear, so the caller must confirm the effect, not the value. */
    iqs_request_window(tile);
    iqs_write(tile, IQS7211A_REG_SYSTEM_CONTROL, (uint16_t)(mode | bit));
}

/**
 * Acknowledge the reset indication, and confirm it actually cleared.
 *
 * A command write outside a communication window is dropped without
 * complaint, so a single blind pulse is not enough — Show Reset simply
 * stays set and every later boot looks like an unexpected reset.
 *
 * @return 1 once Show Reset reads clear, 0 if it never did.
 */
static uint8_t iqs_ack_reset(tile_t *tile)
{
    for (uint8_t i = 0; i < IQS_WINDOW_RETRIES; i++) {
        iqs_command(tile, IQS7211A_CTRL_ACK_RESET);

        uint16_t info = iqs_read(tile, IQS7211A_REG_INFO_FLAGS);
        if (info != IQS7211A_INVALID_RESPONSE && !(info & IQS7211A_INFO_SHOW_RESET))
            return 1;

        iqs_request_window(tile);
    }
    return 0;
}

/** Cached info flags, or 0 when nothing has been read yet. */
static uint16_t cached_info(tile_t *tile)
{
    return state_for(tile)->block[BLK_INFO_FLAGS];
}

/* ---- RDY pin EXTI callback ---- */

/* Used only when cfg.rdy_pin is set; referenced by pointer in init(). */
static void _rdy_isr_0(void *ctx) { state[0].rdy_flag = 1; (void)ctx; }

/* ================================================================
 * Lifecycle
 * ================================================================ */

uint8_t tile_sense_cap_find(tiles_pal_t *hal, uint8_t instance)
{
    uint8_t addr = resolve_id(instance);
    if (!addr) return 0;
    return hal->i2c_is_ready(hal->handle, addr) == 0;
}

void tile_sense_cap_init(tiles_pal_t *hal, uint8_t instance,
                         tile_t *tile, const sense_cap_cfg_t *cfg)
{
    memzero(tile, sizeof(tile_t));
    tile->hal = hal;
    tile->id  = resolve_id(instance);

    if (!tile->id) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_cap: invalid instance");
        return;
    }

    iqs7211a_state_t *s = state_for(tile);
    memzero(s, sizeof(iqs7211a_state_t));
    if (cfg) {
        s->on_event  = cfg->on_event;
        s->event_ctx = cfg->event_ctx;
        s->rdy_pin   = cfg->rdy_pin;
        s->millis     = cfg->millis;
        s->millis_ctx = cfg->millis_ctx;
        if (cfg->active_rate_ms)
            s->est_period_ms = cfg->active_rate_ms;
    }
    if (s->est_period_ms == 0)
        s->est_period_ms = 10;   /* fallback-clock step per process() */

    /* Probe the bus */
    if (hal->i2c_is_ready(hal->handle, tile->id) != 0) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_cap: device not found");
        return;
    }
    tile->state = TILE_STATE_FOUND;

    /* Identify: the product number is the only fixed identity the part
     * offers — firmware versions vary between production batches. */
    uint16_t product = iqs_read(tile, IQS7211A_REG_PRODUCT_NUM);
    if (product == IQS7211A_INVALID_RESPONSE) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_cap: device ACKs but grants no comms window");
        return;
    }
    if (product != IQS7211A_PRODUCT_NUMBER) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_cap: wrong product number");
        return;
    }
    s->version_major = iqs_read(tile, IQS7211A_REG_MAJOR_VER);
    s->version_minor = iqs_read(tile, IQS7211A_REG_MINOR_VER);

    /* Snapshot Config Settings while the device is demonstrably talking.
     * sleep()/wake() restore from this instead of a live read-modify-write:
     * parking the part in manual-control LP2 makes windows scarce, and a
     * wake() that cannot read is a wake() that never returns the device. */
    uint16_t config = iqs_read(tile, IQS7211A_REG_CONFIG_SETTINGS);
    if (config != IQS7211A_INVALID_RESPONSE) {
        s->config_shadow = config;
        s->config_valid  = 1;
    }

    /* Clear the power-on reset indication so a later Show Reset means
     * something went wrong rather than "we just booted". Not fatal if it
     * fails — the device is otherwise fine, the flag is just stale. */
    if (!iqs_ack_reset(tile))
        TILE_ON_ERROR(tile, "sense_cap: reset indication would not clear");

    /* Apply only what the caller asked for. The device's trackpad
     * geometry and ATI settings are deliberately left alone — see the
     * `@studio unsupported` notes in the header. */
    if (cfg) {
        if (cfg->gestures)
            iqs_write_verified(tile, IQS7211A_REG_GESTURE_ENABLE, cfg->gestures);

        if (cfg->active_rate_ms)
            iqs_write_verified(tile, IQS7211A_REG_RATE_ACTIVE, cfg->active_rate_ms);

        if (cfg->max_touches) {
            /* 0x61 high byte = max multi-touches, low byte = total Txs.
             * Preserve the Tx count, which belongs to the geometry. */
            uint16_t v = iqs_read(tile, IQS7211A_REG_TP_TOUCHES);
            if (v != IQS7211A_INVALID_RESPONSE) {
                v = (uint16_t)((v & 0x00FF) | ((uint16_t)cfg->max_touches << 8));
                iqs_write_verified(tile, IQS7211A_REG_TP_TOUCHES, v);
            }
        }

        if (cfg->event_mode) {
            /* Event mode with no event source enabled would go silent,
             * so turn on the trackpad and gesture sources alongside it. */
            iqs_modify(tile, IQS7211A_REG_CONFIG_SETTINGS, 0,
                       (uint16_t)(IQS7211A_CFG_EVENT_MODE |
                                  IQS7211A_CFG_TP_EVENT |
                                  IQS7211A_CFG_GESTURE_EVENT));
        }
    }

    /* Set up the RDY interrupt if a pad map routes it. */
    s->rdy_flag = 0;
    if (s->rdy_pin && hal->gpio_irq_enable) {
        hal->gpio_irq_enable(hal->handle, s->rdy_pin,
                             TILES_GPIO_EDGE_FALLING, _rdy_isr_0, NULL);
    }

    tile->state = TILE_STATE_READY;
}

void tile_sense_cap_reset(tile_t *tile)
{
    if (tile->state == TILE_STATE_NONE) return;

    iqs_command(tile, IQS7211A_CTRL_SW_RESET);
    /* The reset lands once the comms window closes (§9.3.2); the part
     * then cold-boots. Give it time before touching the bus again. */
    tile->hal->delay_ms(100);

    iqs7211a_state_t *s = state_for(tile);
    memzero(s->block, sizeof(s->block));

    iqs_ack_reset(tile);
    tile->state = TILE_STATE_READY;
}

void tile_sense_cap_ack_reset(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_SLEEPING)
        return;
    iqs_ack_reset(tile);
}

void tile_sense_cap_sleep(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY) return;

    iqs7211a_state_t *s = state_for(tile);

    /* Without a trustworthy config snapshot there is no safe way back out
     * of manual control, so refuse rather than strand the device. */
    if (!s->config_valid) {
        TILE_ON_ERROR(tile, "sense_cap: no config snapshot; refusing to sleep");
        return;
    }

    /* Manual control hands mode switching to the host, then select LP2. */
    if (!iqs_write_verified(tile, IQS7211A_REG_CONFIG_SETTINGS,
                            (uint16_t)(s->config_shadow | IQS7211A_CFG_MANUAL_CONTROL))) {
        TILE_ON_ERROR(tile, "sense_cap: sleep aborted, config write failed");
        return;
    }
    iqs_write(tile, IQS7211A_REG_SYSTEM_CONTROL, (uint16_t)SENSE_CAP_MODE_LP2);
    tile->state = TILE_STATE_SLEEPING;
}

void tile_sense_cap_wake(tile_t *tile)
{
    if (tile->state != TILE_STATE_SLEEPING && tile->state != TILE_STATE_READY)
        return;

    iqs7211a_state_t *s = state_for(tile);

    /* Mode select first, and blind: in LP2 a read may well fail, and this
     * write is a plain assignment — every other bit in System Control is a
     * self-clearing one-shot, so there is nothing to preserve. */
    iqs_request_window(tile);
    iqs_write(tile, IQS7211A_REG_SYSTEM_CONTROL, (uint16_t)SENSE_CAP_MODE_ACTIVE);

    /* Hand mode switching back to the device's own timers, from the
     * snapshot rather than a read that may not land. */
    if (s->config_valid) {
        iqs_write_verified(tile, IQS7211A_REG_CONFIG_SETTINGS,
                           (uint16_t)(s->config_shadow & (uint16_t)~IQS7211A_CFG_MANUAL_CONTROL));
    }
    tile->state = TILE_STATE_READY;
}

/* ================================================================
 * Surface configuration
 * ================================================================ */

const sense_cap_surface_t sense_cap_surface_2x3 = {
    .total_rx   = 2,
    .total_tx   = 3,
    .rx_pins    = { 3, 6 },        /* tile pad 2 = RX3 (top strip), pad 9 = RX6 */
    .tx_pins    = { 11, 9, 8 },    /* tile pads 6, 7, 8, columns left to right
                                      (the WLCSP18 ball on pad 7 bonds TX9+TX10;
                                      either number selects it — 9 used here) */
    .switch_xy  = 1,               /* X along the three Tx columns */
    .flip_x     = 0,
    .flip_y     = 0,
    .x_res      = 512,             /* 256 points between electrodes: (3-1)*256 */
    .y_res      = 256,             /* (2-1)*256 */
    .ati_target = 900,             /* validated on hardware 2026-08-06: the
                                      div31 base + 900-count working point is
                                      where this surface's touch signal lives
                                      (~100-190 counts); chip-default base at
                                      target 300 tunes but is nearly deaf. */
    .alp_enable = 1,               /* whole surface as one mutual wake channel */
    .alp_ati_target = 200,
    .ati_base = 0x023F,            /* fine 1, mult 1, coarse div 31 */
    .touch_set_mult = 8,           /* ~56-count set threshold at target */
    .touch_clear_mult = 5,
};

/** Prox-block of a chip Rx pin: RX0-3 sense in block A, RX4-7 in block B. */
static uint8_t rx_block_b(uint8_t rx_pin)
{
    return (rx_pin >= 4) ? 1 : 0;
}

/**
 * Surface re-ATI. Queues the trackpad re-ATI — plus the ALP's when the
 * surface configured one — but waits only for the trackpad completion:
 * a queued ALP re-ATI executes when the ALP is next sensed, which is
 * LP1/LP2 (§5.6.3), so it cannot complete while we sit in Active mode.
 * It stays queued on-chip and runs at the first low-power entry.
 */
static uint8_t surface_re_ati(tile_t *tile, uint8_t with_alp)
{
    uint16_t cmd = IQS7211A_CTRL_TP_RE_ATI;
    if (with_alp)
        cmd |= IQS7211A_CTRL_ALP_RE_ATI;

    iqs_read(tile, IQS7211A_REG_INFO_FLAGS);
    iqs_command(tile, cmd);

    /* The routine only executes once the communication window closes
     * (§5.6.3), so give it genuine quiet time first — polling at 10 ms
     * keeps forcing windows and starves it. Then check at a gentle
     * cadence. */
    tile->hal->delay_ms(300);

    /* Wait for completion, but never trust the flags alone: ATI-error
     * raises transiently while the chip's own retry passes converge
     * (seen on hardware), and the completion flag can be consumed
     * between polls. The flags only end the wait early — the verdict
     * below comes from the outcome. */
    for (uint8_t i = 0; i < 40; i++) {
        uint16_t info = iqs_read(tile, IQS7211A_REG_INFO_FLAGS);

        if (info != IQS7211A_INVALID_RESPONSE &&
            (info & IQS7211A_INFO_RE_ATI_OCCURRED))
            break;
        tile->hal->delay_ms(50);
    }

    /* Judge by result: every configured channel within 25% of the
     * target counts as tuned. Convergence after a cold geometry write
     * can take several passes (seen on hardware), so re-queue and
     * re-check a few times before giving up. */
    iqs7211a_state_t *s = state_for(tile);
    uint16_t target = iqs_read(tile, IQS7211A_REG_TP_ATI_TARGET);
    if (target == IQS7211A_INVALID_RESPONSE || target == 0 ||
        s->num_channels == 0)
        return 0;

    for (uint8_t round = 0; round < 4; round++) {
        uint8_t in_range = 1;
        for (uint8_t ch = 0; ch < s->num_channels; ch++) {
            uint16_t c = tile_sense_cap_get_channel_count(tile, ch);
            if (c < target - target / 4 || c > target + target / 4) {
                in_range = 0;
                break;
            }
        }
        if (in_range) return 1;
        if (round < 3) {
            iqs_command(tile, IQS7211A_CTRL_TP_RE_ATI);
            tile->hal->delay_ms(600);
        }
    }
    return 0;
}

uint8_t tile_sense_cap_configure_surface(tile_t *tile,
                                         const sense_cap_surface_t *surf)
{
    if (tile->state != TILE_STATE_READY || surf == NULL) return 0;
    if (surf->total_rx == 0 || surf->total_rx > SENSE_CAP_MAX_RX)  return 0;
    if (surf->total_tx == 0 || surf->total_tx > SENSE_CAP_MAX_TX)  return 0;
    if ((uint8_t)(surf->total_rx + surf->total_tx) > SENSE_CAP_MAX_MAP) return 0;
    if ((uint16_t)surf->total_rx * surf->total_tx > 32) return 0;

    uint8_t ok = 1;

    /* Sizing + orientation. 0x60 high byte is Total Rxs, low byte the
     * settings bits; MAV + dynamic IIR are the datasheet-recommended
     * filter setup. 0x61 low byte is Total Txs — the high byte (max
     * multi-touches) is preserved for set_max_touches() to own. */
    uint16_t settings = IQS7211A_TP_MAV_FILTER | IQS7211A_TP_IIR_FILTER;
    if (surf->switch_xy) settings |= IQS7211A_TP_SWITCH_XY;
    if (surf->flip_x)    settings |= IQS7211A_TP_FLIP_X;
    if (surf->flip_y)    settings |= IQS7211A_TP_FLIP_Y;
    ok &= iqs_write_verified(tile, IQS7211A_REG_TP_SETTINGS,
                             (uint16_t)((uint16_t)surf->total_rx << 8) | settings);
    iqs_modify(tile, IQS7211A_REG_TP_TOUCHES, 0x00FF, surf->total_tx);
    ok &= iqs_write_verified(tile, IQS7211A_REG_X_RESOLUTION, surf->x_res);
    ok &= iqs_write_verified(tile, IQS7211A_REG_Y_RESOLUTION, surf->y_res);

    /* Rx/Tx pin mapping (§7.1.5): Rx pins first, Tx pins immediately
     * after, two entries per register from 0x90. Unused slots get 0xFF. */
    uint8_t map[SENSE_CAP_MAX_MAP];
    uint8_t n = 0;
    for (uint8_t i = 0; i < surf->total_rx; i++) map[n++] = surf->rx_pins[i];
    for (uint8_t i = 0; i < surf->total_tx; i++) map[n++] = surf->tx_pins[i];
    while (n < SENSE_CAP_MAX_MAP) map[n++] = 0xFF;
    for (uint8_t w = 0; w < SENSE_CAP_MAX_MAP / 2; w++) {
        uint16_t v = (uint16_t)map[2 * w] | ((uint16_t)map[2 * w + 1] << 8);
        ok &= iqs_write_verified(tile, (uint8_t)(IQS7211A_REG_RXTX_MAP_BASE + w), v);
    }

    /* Sensing cycles (§7.1.2). Channel number = tx_index * total_rx +
     * rx_index (§5.1.1: along the Rxs first). Each cycle senses one
     * block-A channel and one block-B channel of the same Tx, so pair
     * them up per Tx; an unpairable channel gets a cycle to itself. */
    uint8_t cyc[18][2];
    uint8_t n_cyc = 0;
    for (uint8_t c = 0; c < 18; c++) { cyc[c][0] = IQS7211A_CHANNEL_NONE;
                                       cyc[c][1] = IQS7211A_CHANNEL_NONE; }
    for (uint8_t t = 0; t < surf->total_tx && n_cyc <= 18; t++) {
        uint8_t a[SENSE_CAP_MAX_RX], b[SENSE_CAP_MAX_RX];
        uint8_t na = 0, nb = 0;
        for (uint8_t r = 0; r < surf->total_rx; r++) {
            uint8_t ch = (uint8_t)(t * surf->total_rx + r);
            if (rx_block_b(surf->rx_pins[r])) b[nb++] = ch;
            else                              a[na++] = ch;
        }
        for (uint8_t i = 0; i < na || i < nb; i++) {
            if (n_cyc >= 18) { ok = 0; break; }   /* > 18 cycles: no fit */
            cyc[n_cyc][0] = (i < na) ? a[i] : IQS7211A_CHANNEL_NONE;
            cyc[n_cyc][1] = (i < nb) ? b[i] : IQS7211A_CHANNEL_NONE;
            n_cyc++;
        }
    }

    /* Pack as 3-byte records [0x05, ch_a, ch_b] into the two register
     * windows: cycles 0-9 at 0xA0-0xAE, cycles 10-17 at 0xB0-0xBB. */
    for (uint8_t half = 0; half < 2; half++) {
        uint8_t first = (uint8_t)(half * 10);
        uint8_t count = half ? 8 : 10;
        uint8_t base  = half ? IQS7211A_REG_CYCLE_BASE_10 : IQS7211A_REG_CYCLE_BASE_0;
        uint8_t bytes[30];
        for (uint8_t c = 0; c < count; c++) {
            bytes[3 * c + 0] = IQS7211A_CYCLE_PROX_BYTE;
            bytes[3 * c + 1] = cyc[first + c][0];
            bytes[3 * c + 2] = cyc[first + c][1];
        }
        for (uint8_t w = 0; w < (uint8_t)(count * 3 / 2); w++) {
            uint16_t v = (uint16_t)bytes[2 * w] | ((uint16_t)bytes[2 * w + 1] << 8);
            ok &= iqs_write_verified(tile, (uint8_t)(base + w), v);
        }
    }

    /* ALP wake channel over the same electrodes: mutual sensing with
     * the count filter, every surface Rx and Tx enabled. Thresholds,
     * debounce and betas stay at chip defaults (see header note). */
    if (surf->alp_enable) {
        uint16_t rx_mask = 0, tx_mask = 0;
        for (uint8_t i = 0; i < surf->total_rx; i++)
            rx_mask |= (uint16_t)(1U << surf->rx_pins[i]);
        for (uint8_t i = 0; i < surf->total_tx; i++)
            tx_mask |= (uint16_t)(1U << surf->tx_pins[i]);
        ok &= iqs_write_verified(tile, IQS7211A_REG_ALP_SETUP,
                                 (uint16_t)(IQS7211A_ALP_SETUP_MUTUAL |
                                            IQS7211A_ALP_SETUP_FILTER | rx_mask));
        ok &= iqs_write_verified(tile, IQS7211A_REG_ALP_TX_ENABLE, tx_mask);
        ok &= iqs_write_verified(tile, IQS7211A_REG_ALP_ATI_TARGET,
                                 surf->alp_ati_target);
    }

    /* ATI base (0 = leave the chip default) and target, then re-tune
     * against the new geometry. Other dividers and drift limits stay at
     * chip defaults (see the ATI fine-tuning note in the header). */
    if (surf->ati_base)
        ok &= iqs_write_verified(tile, IQS7211A_REG_TP_ATI_MULTDIV,
                                 surf->ati_base);
    ok &= iqs_write_verified(tile, IQS7211A_REG_TP_ATI_TARGET, surf->ati_target);

    if (surf->touch_set_mult || surf->touch_clear_mult)
        tile_sense_cap_set_touch_multipliers(tile, surf->touch_set_mult,
                                             surf->touch_clear_mult);

    /* Cache the geometry for the zone / percentage helpers. */
    iqs7211a_state_t *s = state_for(tile);
    s->num_channels  = (uint8_t)(surf->total_rx * surf->total_tx);
    s->srf_rx        = surf->total_rx;
    s->srf_tx        = surf->total_tx;
    s->srf_switch_xy = surf->switch_xy;
    s->x_res         = surf->x_res;
    s->y_res         = surf->y_res;

    ok &= surface_re_ati(tile, surf->alp_enable);
    return ok;
}

uint8_t tile_sense_cap_setup_2x3(tile_t *tile)
{
    return tile_sense_cap_configure_surface(tile, &sense_cap_surface_2x3);
}

/* ================================================================
 * Touch events and zones (high level)
 * ================================================================ */

void tile_sense_cap_on_touch(tile_t *tile, sense_cap_touch_cb_t cb, void *ctx)
{
    iqs7211a_state_t *s = state_for(tile);
    s->on_touch = cb;
    s->touch_ctx = ctx;
}

uint8_t tile_sense_cap_next_touch_event(tile_t *tile, sense_cap_touch_t *ev)
{
    iqs7211a_state_t *s = state_for(tile);
    if (s->evq_count == 0) return 0;
    if (ev) *ev = s->evq[s->evq_rd];
    s->evq_rd = (uint8_t)((s->evq_rd + 1) % TOUCH_EVQ_LEN);
    s->evq_count--;
    return 1;
}

void tile_sense_cap_on_tap(tile_t *tile, sense_cap_tap_cb_t cb, void *ctx)
{
    iqs7211a_state_t *s = state_for(tile);
    s->on_tap = cb;
    s->tap_ctx = ctx;
}

void tile_sense_cap_on_long_press(tile_t *tile, sense_cap_hold_cb_t cb, void *ctx)
{
    iqs7211a_state_t *s = state_for(tile);
    s->on_hold = cb;
    s->hold_ctx = ctx;
}

void tile_sense_cap_on_swipe(tile_t *tile, sense_cap_swipe_cb_t cb, void *ctx)
{
    iqs7211a_state_t *s = state_for(tile);
    s->on_swipe = cb;
    s->swipe_ctx = ctx;
}

void tile_sense_cap_on_drag(tile_t *tile, sense_cap_drag_cb_t cb, void *ctx)
{
    iqs7211a_state_t *s = state_for(tile);
    s->on_drag = cb;
    s->drag_ctx = ctx;
}

void tile_sense_cap_on_pinch(tile_t *tile, sense_cap_pinch_cb_t cb, void *ctx)
{
    iqs7211a_state_t *s = state_for(tile);
    s->on_pinch = cb;
    s->pinch_ctx = ctx;
}

uint16_t tile_sense_cap_get_touch_events(tile_t *tile)
{
    iqs7211a_state_t *s = state_for(tile);
    uint16_t ev = s->ev_latch;
    s->ev_latch = 0;
    return ev;
}

uint8_t tile_sense_cap_was_tapped(tile_t *tile)
{
    iqs7211a_state_t *s = state_for(tile);
    uint16_t taps = (uint16_t)(s->ev_latch
                    & (SENSE_CAP_EV_TAP | SENSE_CAP_EV_DOUBLE_TAP));
    s->ev_latch &= (uint16_t)~taps;
    return taps ? 1 : 0;
}

int8_t tile_sense_cap_zone_at(tile_t *tile, uint16_t x, uint16_t y)
{
    iqs7211a_state_t *s = state_for(tile);
    if (s->srf_rx == 0 || s->x_res == 0 || s->y_res == 0) return -1;

    /* Channel number = tx_index * total_rx + rx_index (§5.1.1). With the
     * XY axes switched (X along Txs), X selects the Tx and Y the Rx;
     * unswitched, the roles swap. */
    uint8_t a = (uint8_t)(((uint32_t)x * (s->srf_switch_xy ? s->srf_tx
                                                           : s->srf_rx)) / s->x_res);
    uint8_t b = (uint8_t)(((uint32_t)y * (s->srf_switch_xy ? s->srf_rx
                                                           : s->srf_tx)) / s->y_res);
    uint8_t tx_i = s->srf_switch_xy ? a : b;
    uint8_t rx_i = s->srf_switch_xy ? b : a;
    if (tx_i >= s->srf_tx) tx_i = (uint8_t)(s->srf_tx - 1);
    if (rx_i >= s->srf_rx) rx_i = (uint8_t)(s->srf_rx - 1);

    return (int8_t)(tx_i * s->srf_rx + rx_i);
}

int8_t tile_sense_cap_get_zone(tile_t *tile)
{
    iqs7211a_state_t *s = state_for(tile);

    uint8_t fingers = (uint8_t)((s->block[BLK_INFO_FLAGS]
                                 >> IQS7211A_INFO_NUM_FINGERS_SHIFT) & 3);
    uint16_t x = s->block[BLK_FINGER_BASE + 0];
    uint16_t y = s->block[BLK_FINGER_BASE + 1];
    if (fingers == 0 || x == XY_INVALID || y == XY_INVALID) return -1;

    return tile_sense_cap_zone_at(tile, x, y);
}

uint8_t tile_sense_cap_is_zone_touched(tile_t *tile, uint8_t zone)
{
    return tile_sense_cap_is_touched(tile, zone);
}

void tile_sense_cap_get_position_pct(tile_t *tile,
                                     int32_t *x_pct, int32_t *y_pct)
{
    iqs7211a_state_t *s = state_for(tile);
    uint8_t fingers = (uint8_t)((s->block[BLK_INFO_FLAGS]
                                 >> IQS7211A_INFO_NUM_FINGERS_SHIFT) & 3);
    uint16_t x = s->block[BLK_FINGER_BASE + 0];
    uint16_t y = s->block[BLK_FINGER_BASE + 1];

    if (fingers == 0 || x == XY_INVALID || y == XY_INVALID ||
        s->x_res < 2 || s->y_res < 2) {
        if (x_pct) *x_pct = -1;
        if (y_pct) *y_pct = -1;
        return;
    }
    if (x_pct) *x_pct = (int32_t)((uint32_t)x * 100 / (s->x_res - 1));
    if (y_pct) *y_pct = (int32_t)((uint32_t)y * 100 / (s->y_res - 1));
}

uint8_t tile_sense_cap_wait_for_touch(tile_t *tile, uint32_t timeout_ms)
{
    uint32_t elapsed = 0;
    while (elapsed <= timeout_ms) {
        tile_sense_cap_process(tile);
        if (tile_sense_cap_get_num_fingers(tile) > 0)
            return 1;
        tile->hal->delay_ms(5);
        elapsed += 5;
    }
    return 0;
}

void tile_sense_cap_set_sensitivity(tile_t *tile, uint8_t level)
{
    /* set/clear multiplier pairs, firmest (1) to lightest (5) —
     * threshold = reference * mult / 128 (§5.5.1). */
    static const uint8_t pairs[5][2] = {
        { 48, 28 }, { 28, 16 }, { 16, 10 }, { 10, 6 }, { 5, 3 },
    };
    if (level < 1) level = 1;
    if (level > 5) level = 5;
    tile_sense_cap_set_touch_multipliers(tile, pairs[level - 1][0],
                                         pairs[level - 1][1]);
}

/* ================================================================
 * Touch state machine
 * ================================================================ */

/** Driver clock: user-supplied ms counter, else a report-rate estimate. */
static uint32_t touch_now(iqs7211a_state_t *s)
{
    if (s->millis)
        return s->millis(s->millis_ctx);
    return s->tick_ms;
}

/** Push an event into the ring; oldest is dropped on overflow. */
static void touch_push(iqs7211a_state_t *s, const sense_cap_touch_t *ev)
{
    if (s->evq_count == TOUCH_EVQ_LEN) {
        s->evq_rd = (uint8_t)((s->evq_rd + 1) % TOUCH_EVQ_LEN);
        s->evq_count--;
    }
    s->evq[(uint8_t)((s->evq_rd + s->evq_count) % TOUCH_EVQ_LEN)] = *ev;
    s->evq_count++;
}

/** Emit one touch event: queue it and fire the stream callback. */
static void touch_emit(tile_t *tile, iqs7211a_state_t *s, uint8_t phase,
                       uint8_t finger, uint16_t x, uint16_t y,
                       int16_t dx, int16_t dy, uint32_t now)
{
    sense_cap_touch_t ev;
    ev.phase    = phase;
    ev.finger   = finger;
    ev.x        = x;
    ev.y        = y;
    ev.dx       = dx;
    ev.dy       = dy;
    ev.vx       = s->vx;
    ev.vy       = s->vy;
    ev.strength = s->block[BLK_FINGER_BASE + finger * 4 + 2];
    ev.t_ms     = now;

    touch_push(s, &ev);
    if (s->on_touch)
        s->on_touch(tile, &ev, s->touch_ctx);
}

static int16_t iabs16(int16_t v) { return (int16_t)(v < 0 ? -v : v); }

/**
 * Advance the touch state machine from the freshly cached data block.
 * Runs inside process(), main-loop context; fires every recognizer.
 */
static void touch_update(tile_t *tile, iqs7211a_state_t *s)
{
    uint32_t now = touch_now(s);
    s->tick_ms += s->est_period_ms;

    uint8_t fingers = (uint8_t)((s->block[BLK_INFO_FLAGS]
                                 >> IQS7211A_INFO_NUM_FINGERS_SHIFT) & 3);

    for (uint8_t n = 0; n < 2; n++) {
        uint16_t x = s->block[BLK_FINGER_BASE + n * 4 + 0];
        uint16_t y = s->block[BLK_FINGER_BASE + n * 4 + 1];
        uint8_t present = (n < fingers) && x != XY_INVALID && y != XY_INVALID;
        uint8_t was = (uint8_t)(s->was_down >> n) & 1;

        /* Presence debounce: a marginal touch can flicker the chip's
         * finger flag every report cycle, which would cascade into
         * DOWN/UP/TAP storms. Require two consecutive cycles in the new
         * state before honoring a transition. */
        if (present != was) {
            if (++s->presence_cnt[n] < 2) {
                present = was;   /* not yet — hold the previous state */
            } else {
                s->presence_cnt[n] = 0;
            }
        } else {
            s->presence_cnt[n] = 0;
        }

        if (present && !was) {
            s->px[n] = s->down_x[n] = x;
            s->py[n] = s->down_y[n] = y;
            s->down_t[n] = s->move_t[n] = now;
            s->was_down   |= (uint8_t)(1U << n);
            s->moved_far  &= (uint8_t)~(1U << n);
            s->hold_fired &= (uint8_t)~(1U << n);
            s->ev_latch |= SENSE_CAP_EV_TOUCH_DOWN;
            touch_emit(tile, s, SENSE_CAP_TOUCH_DOWN, n, x, y, 0, 0, now);
        } else if (present && was) {
            /* A held finger can report a transient invalid coordinate
             * (the same marginal frames the presence debounce absorbs).
             * Keep the last good position and skip movement processing —
             * one bad frame must not poison deltas, velocity or the
             * moved-far flag. */
            if (x == XY_INVALID || y == XY_INVALID) {
                x = s->px[n];
                y = s->py[n];
            }
            int16_t dx = (int16_t)(x - s->px[n]);
            int16_t dy = (int16_t)(y - s->py[n]);

            if ((int16_t)(iabs16(dx) + iabs16(dy)) >= TOUCH_MOVE_DEAD) {
                uint32_t dt = now - s->move_t[n];
                if (dt == 0) dt = 1;
                s->vx = (int16_t)((int32_t)dx * 1000 / (int32_t)dt);
                s->vy = (int16_t)((int32_t)dy * 1000 / (int32_t)dt);

                int16_t total = (int16_t)(iabs16((int16_t)(x - s->down_x[n]))
                                        + iabs16((int16_t)(y - s->down_y[n])));
                if (total > TOUCH_TAP_DIST) {
                    s->moved_far |= (uint8_t)(1U << n);
                    s->ev_latch |= SENSE_CAP_EV_DRAG;
                    if (s->on_drag)
                        s->on_drag(tile, dx, dy, x, y, s->drag_ctx);
                }

                touch_emit(tile, s, SENSE_CAP_TOUCH_MOVED, n, x, y, dx, dy, now);
                s->px[n] = x;
                s->py[n] = y;
                s->move_t[n] = now;
            }

            if (!((s->hold_fired >> n) & 1) && !((s->moved_far >> n) & 1) &&
                now - s->down_t[n] >= TOUCH_HOLD_MS) {
                s->hold_fired |= (uint8_t)(1U << n);
                s->ev_latch |= SENSE_CAP_EV_LONG_PRESS;
                if (s->on_hold)
                    s->on_hold(tile, x, y, s->hold_ctx);
            }
        } else if (!present && was) {
            s->was_down &= (uint8_t)~(1U << n);
            s->last_up_t = now;
            s->ev_latch |= SENSE_CAP_EV_TOUCH_UP;
            touch_emit(tile, s, SENSE_CAP_TOUCH_UP, n,
                       s->px[n], s->py[n], 0, 0, now);

            /* Fling: a drag released at speed is a swipe, whether or not
             * the chip's own engine noticed. Dominant axis wins. */
            if (((s->moved_far >> n) & 1) &&
                (iabs16(s->vx) > 400 || iabs16(s->vy) > 400)) {
                uint8_t dir;
                uint16_t evb;
                if (iabs16(s->vx) >= iabs16(s->vy)) {
                    dir = s->vx > 0 ? SENSE_CAP_DIR_RIGHT : SENSE_CAP_DIR_LEFT;
                    evb = s->vx > 0 ? SENSE_CAP_EV_SWIPE_RIGHT
                                    : SENSE_CAP_EV_SWIPE_LEFT;
                } else {
                    dir = s->vy > 0 ? SENSE_CAP_DIR_DOWN : SENSE_CAP_DIR_UP;
                    evb = s->vy > 0 ? SENSE_CAP_EV_SWIPE_DOWN
                                    : SENSE_CAP_EV_SWIPE_UP;
                }
                s->ev_latch |= evb;
                if (s->on_swipe)
                    s->on_swipe(tile, dir, s->vx, s->vy, s->swipe_ctx);
            }

            if (!((s->moved_far >> n) & 1) && !((s->hold_fired >> n) & 1) &&
                now - s->down_t[n] <= TOUCH_TAP_MS) {
                uint8_t taps = 1;
                if (s->last_tap_t != 0 && now - s->last_tap_t <= TOUCH_DOUBLE_MS) {
                    taps = 2;
                    s->last_tap_t = 0;
                    s->ev_latch |= SENSE_CAP_EV_DOUBLE_TAP;
                } else {
                    s->last_tap_t = now;
                    s->ev_latch |= SENSE_CAP_EV_TAP;
                }
                if (s->on_tap)
                    s->on_tap(tile, taps, s->px[n], s->py[n], s->tap_ctx);
            }
        }
    }

    /* Chip swipe engine: act on newly set bits only — and only around a
     * touch our own tracker saw. The chip's engine can emit gesture bits
     * from sub-threshold noise wander; a swipe with no finger is noise. */
    uint16_t gest = s->block[BLK_GESTURES];
    uint16_t fresh = (uint16_t)(gest & (uint16_t)~s->prev_gestures);
    s->prev_gestures = gest;
    if (!s->was_down && (s->last_up_t == 0 || now - s->last_up_t > 400))
        fresh = 0;

    static const struct { uint16_t bit; uint16_t ev; uint8_t dir; } swipes[] = {
        { SENSE_CAP_GESTURE_SWIPE_X_NEG, SENSE_CAP_EV_SWIPE_LEFT,  SENSE_CAP_DIR_LEFT  },
        { SENSE_CAP_GESTURE_SWIPE_X_POS, SENSE_CAP_EV_SWIPE_RIGHT, SENSE_CAP_DIR_RIGHT },
        { SENSE_CAP_GESTURE_SWIPE_Y_NEG, SENSE_CAP_EV_SWIPE_UP,    SENSE_CAP_DIR_UP    },
        { SENSE_CAP_GESTURE_SWIPE_Y_POS, SENSE_CAP_EV_SWIPE_DOWN,  SENSE_CAP_DIR_DOWN  },
    };
    for (uint8_t i = 0; i < 4; i++) {
        if (fresh & swipes[i].bit) {
            s->ev_latch |= swipes[i].ev;
            if (s->on_swipe)
                s->on_swipe(tile, swipes[i].dir, s->vx, s->vy, s->swipe_ctx);
        }
    }

    /* Two-finger pinch from the spread distance (Manhattan — no sqrt). */
    uint16_t x0 = s->block[BLK_FINGER_BASE + 0];
    uint16_t y0 = s->block[BLK_FINGER_BASE + 1];
    uint16_t x1 = s->block[BLK_FINGER_BASE + 4];
    uint16_t y1 = s->block[BLK_FINGER_BASE + 5];
    if (fingers == 2 && x0 != XY_INVALID && x1 != XY_INVALID) {
        uint16_t dist = (uint16_t)(iabs16((int16_t)(x0 - x1))
                                 + iabs16((int16_t)(y0 - y1)));
        if (s->pinch_valid) {
            int16_t delta = (int16_t)(dist - s->pinch_dist);
            if (iabs16(delta) >= TOUCH_PINCH_DEAD) {
                s->ev_latch |= SENSE_CAP_EV_PINCH;
                if (s->on_pinch)
                    s->on_pinch(tile, delta, s->pinch_ctx);
                s->pinch_dist = dist;
            }
        } else {
            s->pinch_valid = 1;
            s->pinch_dist = dist;
        }
    } else {
        s->pinch_valid = 0;
    }
}

/* ================================================================
 * Event processing
 * ================================================================ */

void tile_sense_cap_process(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY) return;

    iqs7211a_state_t *s = state_for(tile);

    /* In RDY mode, do nothing until the device says it has data. */
    if (s->rdy_pin && !s->rdy_flag)
        return;
    s->rdy_flag = 0;

    /* One burst across 0x10..0x1B — reads auto-increment (§11.5). */
    uint8_t buf[BLK_WORDS * 2];
    uint16_t info = IQS7211A_INVALID_RESPONSE;

    for (uint8_t attempt = 0; attempt < IQS_WINDOW_RETRIES; attempt++) {
        memzero(buf, sizeof(buf));
        tile->hal->i2c_read(tile->hal->handle, tile->id,
                            IQS7211A_REG_INFO_FLAGS, buf, sizeof(buf));
        info = (uint16_t)(((uint16_t)buf[1] << 8) | buf[0]);
        if (info != IQS7211A_INVALID_RESPONSE)
            break;
        iqs_request_window(tile);
    }

    /* No window: keep the previous snapshot rather than caching garbage,
     * and stay quiet — the next cycle usually lands. */
    if (info == IQS7211A_INVALID_RESPONSE)
        return;

    for (uint8_t i = 0; i < BLK_WORDS; i++)
        s->block[i] = (uint16_t)(((uint16_t)buf[i * 2 + 1] << 8) | buf[i * 2]);

    touch_update(tile, s);

    if (s->on_event)
        s->on_event(tile, s->block[BLK_INFO_FLAGS], s->event_ctx);
}

void tile_sense_cap_on_event(tile_t *tile, sense_cap_event_cb_t cb, void *ctx)
{
    iqs7211a_state_t *s = state_for(tile);
    s->on_event  = cb;
    s->event_ctx = ctx;
}

/* ================================================================
 * Identification
 * ================================================================ */

uint16_t tile_sense_cap_get_version_major(tile_t *tile)
{
    return state_for(tile)->version_major;
}

uint16_t tile_sense_cap_get_version_minor(tile_t *tile)
{
    return state_for(tile)->version_minor;
}

uint16_t tile_sense_cap_get_settings_version(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY) return 0;
    uint16_t v = iqs_read(tile, IQS7211A_REG_SETTINGS_VER);
    return (v == IQS7211A_INVALID_RESPONSE) ? 0 : v;
}

/* ================================================================
 * Runtime data
 * ================================================================ */

uint16_t tile_sense_cap_get_info_flags(tile_t *tile)
{
    return cached_info(tile);
}

uint16_t tile_sense_cap_get_gestures(tile_t *tile)
{
    return state_for(tile)->block[BLK_GESTURES];
}

uint8_t tile_sense_cap_get_num_fingers(tile_t *tile)
{
    uint16_t info = cached_info(tile);
    return (uint8_t)((info & IQS7211A_INFO_NUM_FINGERS_MASK)
                     >> IQS7211A_INFO_NUM_FINGERS_SHIFT);
}

/** Fetch one word of a finger's 4-word slot. */
static uint16_t finger_word(tile_t *tile, uint8_t finger, uint8_t word)
{
    if (finger >= SENSE_CAP_NUM_FINGERS) return 0;
    return state_for(tile)->block[BLK_FINGER_BASE + finger * 4 + word];
}

uint16_t tile_sense_cap_get_finger_x(tile_t *tile, uint8_t finger)
{
    return finger_word(tile, finger, 0);
}

uint16_t tile_sense_cap_get_finger_y(tile_t *tile, uint8_t finger)
{
    return finger_word(tile, finger, 1);
}

uint16_t tile_sense_cap_get_finger_strength(tile_t *tile, uint8_t finger)
{
    return finger_word(tile, finger, 2);
}

uint16_t tile_sense_cap_get_finger_area(tile_t *tile, uint8_t finger)
{
    return finger_word(tile, finger, 3);
}

int16_t tile_sense_cap_get_relative_x(tile_t *tile)
{
    return (int16_t)state_for(tile)->block[BLK_RELATIVE_X];
}

int16_t tile_sense_cap_get_relative_y(tile_t *tile)
{
    return (int16_t)state_for(tile)->block[BLK_RELATIVE_Y];
}

uint32_t tile_sense_cap_get_touch_status(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY) return 0;

    uint8_t buf[4] = {0, 0, 0, 0};
    uint32_t low = IQS7211A_INVALID_RESPONSE;

    for (uint8_t attempt = 0; attempt < IQS_WINDOW_RETRIES; attempt++) {
        tile->hal->i2c_read(tile->hal->handle, tile->id,
                            IQS7211A_REG_TOUCH_STATUS_0, buf, sizeof(buf));
        low = (uint32_t)(((uint16_t)buf[1] << 8) | buf[0]);
        if (low != IQS7211A_INVALID_RESPONSE)
            break;
        iqs_request_window(tile);
    }
    if (low == IQS7211A_INVALID_RESPONSE)
        return 0;

    uint32_t high = (uint32_t)(((uint16_t)buf[3] << 8) | buf[2]);
    return (high << 16) | low;
}

uint8_t tile_sense_cap_is_touched(tile_t *tile, uint8_t channel)
{
    if (channel > 31) return 0;
    return (tile_sense_cap_get_touch_status(tile) >> channel) & 1U;
}

uint8_t tile_sense_cap_get_num_channels(tile_t *tile)
{
    return state_for(tile)->num_channels;
}

uint16_t tile_sense_cap_get_channel_count(tile_t *tile, uint8_t channel)
{
    if (tile->state != TILE_STATE_READY || channel >= 32) return 0;
    uint16_t v = iqs_read_ext(tile, (uint16_t)(IQS7211A_REG_EXT_COUNTS + channel));
    return (v == IQS7211A_INVALID_RESPONSE) ? 0 : v;
}

uint16_t tile_sense_cap_get_channel_delta(tile_t *tile, uint8_t channel)
{
    if (tile->state != TILE_STATE_READY || channel >= 32) return 0;
    uint16_t v = iqs_read_ext(tile, (uint16_t)(IQS7211A_REG_EXT_DELTAS + channel));
    return (v == IQS7211A_INVALID_RESPONSE) ? 0 : v;
}

uint8_t tile_sense_cap_read_channels(tile_t *tile, uint16_t *counts,
                                     uint16_t *deltas, uint8_t n)
{
    if (tile->state != TILE_STATE_READY || n == 0 || n > 32) return 0;
    uint8_t ok = 1;
    if (counts && !iqs_read_ext_block(tile, IQS7211A_REG_EXT_COUNTS, counts, n)) ok = 0;
    if (deltas && !iqs_read_ext_block(tile, IQS7211A_REG_EXT_DELTAS, deltas, n)) ok = 0;
    return ok;
}

uint8_t tile_sense_cap_is_alp_active(tile_t *tile)
{
    return (cached_info(tile) & IQS7211A_INFO_ALP_OUTPUT) ? 1 : 0;
}

uint16_t tile_sense_cap_get_alp_count(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY) return 0;
    uint16_t v = iqs_read(tile, IQS7211A_REG_ALP_COUNT);
    return (v == IQS7211A_INVALID_RESPONSE) ? 0 : v;
}

uint16_t tile_sense_cap_get_alp_lta(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY) return 0;
    uint16_t v = iqs_read(tile, IQS7211A_REG_ALP_LTA);
    return (v == IQS7211A_INVALID_RESPONSE) ? 0 : v;
}

uint8_t tile_sense_cap_get_mode(tile_t *tile)
{
    return (uint8_t)((cached_info(tile) & IQS7211A_INFO_MODE_MASK)
                     >> IQS7211A_INFO_MODE_SHIFT);
}

uint8_t tile_sense_cap_has_ati_error(tile_t *tile)
{
    uint16_t info = cached_info(tile);
    return (info & (IQS7211A_INFO_ATI_ERROR | IQS7211A_INFO_ALP_ATI_ERROR)) ? 1 : 0;
}

/* ================================================================
 * Configuration
 * ================================================================ */

void tile_sense_cap_enable_gestures(tile_t *tile, uint16_t mask)
{
    if (tile->state != TILE_STATE_READY) return;
    iqs_write(tile, IQS7211A_REG_GESTURE_ENABLE, mask);
}

void tile_sense_cap_set_tap_timing(tile_t *tile, uint16_t tap_ms, uint16_t hold_ms)
{
    if (tile->state != TILE_STATE_READY) return;
    iqs_write(tile, IQS7211A_REG_TAP_TIME,  tap_ms);
    iqs_write(tile, IQS7211A_REG_HOLD_TIME, hold_ms);
}

void tile_sense_cap_set_swipe_timing(tile_t *tile, uint16_t swipe_ms,
                                     uint16_t x_dist, uint16_t y_dist)
{
    if (tile->state != TILE_STATE_READY) return;
    iqs_write(tile, IQS7211A_REG_SWIPE_TIME,   swipe_ms);
    iqs_write(tile, IQS7211A_REG_SWIPE_X_DIST, x_dist);
    iqs_write(tile, IQS7211A_REG_SWIPE_Y_DIST, y_dist);
}

/** Map a mode onto its report-rate register, or 0 if there isn't one. */
static uint8_t rate_reg_for_mode(uint8_t mode)
{
    switch (mode) {
    case SENSE_CAP_MODE_ACTIVE:     return IQS7211A_REG_RATE_ACTIVE;
    case SENSE_CAP_MODE_IDLE_TOUCH: return IQS7211A_REG_RATE_IDLE_TOUCH;
    case SENSE_CAP_MODE_IDLE:       return IQS7211A_REG_RATE_IDLE;
    case SENSE_CAP_MODE_LP1:        return IQS7211A_REG_RATE_LP1;
    case SENSE_CAP_MODE_LP2:        return IQS7211A_REG_RATE_LP2;
    default:                        return 0;
    }
}

void tile_sense_cap_set_report_rate(tile_t *tile, uint8_t mode, uint16_t ms)
{
    if (tile->state != TILE_STATE_READY) return;
    uint8_t reg = rate_reg_for_mode(mode);
    if (!reg) return;
    /* Verified: a write that misses its window is silently dropped, and a
     * report rate that did not land shows up later as a slow poll. */
    iqs_write_verified(tile, reg, ms);
}

uint8_t tile_sense_cap_set_poll_latency(tile_t *tile, uint16_t ms)
{
    if (tile->state != TILE_STATE_READY) return 0;
    if (ms < 5) ms = 5;

    /* One period for every charging mode, LP2 included: whichever mode the
     * part has drifted into, the next window is at most `ms` away. */
    static const uint8_t regs[5] = {
        IQS7211A_REG_RATE_ACTIVE, IQS7211A_REG_RATE_IDLE_TOUCH,
        IQS7211A_REG_RATE_IDLE,   IQS7211A_REG_RATE_LP1,
        IQS7211A_REG_RATE_LP2,
    };
    uint8_t ok = 1;
    for (uint8_t i = 0; i < 5; i++)
        if (!iqs_write_verified(tile, regs[i], ms)) ok = 0;
    return ok;
}

/** Map a mode onto its timeout register. LP2 has no timeout — it is the floor. */
static uint8_t timeout_reg_for_mode(uint8_t mode)
{
    switch (mode) {
    case SENSE_CAP_MODE_ACTIVE:     return IQS7211A_REG_TIMEOUT_ACTIVE;
    case SENSE_CAP_MODE_IDLE_TOUCH: return IQS7211A_REG_TIMEOUT_IDLE_TOUCH;
    case SENSE_CAP_MODE_IDLE:       return IQS7211A_REG_TIMEOUT_IDLE;
    case SENSE_CAP_MODE_LP1:        return IQS7211A_REG_TIMEOUT_LP1;
    default:                        return 0;
    }
}

uint8_t tile_sense_cap_set_mode_timeout(tile_t *tile, uint8_t mode, uint16_t seconds)
{
    if (tile->state != TILE_STATE_READY) return 0;
    uint8_t reg = timeout_reg_for_mode(mode);
    if (!reg) return 0;
    return iqs_write_verified(tile, reg, seconds);
}

void tile_sense_cap_set_max_touches(tile_t *tile, uint8_t fingers)
{
    if (tile->state != TILE_STATE_READY) return;
    if (fingers < 1 || fingers > SENSE_CAP_NUM_FINGERS) return;

    /* High byte only — the low byte is the total Tx count (geometry). */
    uint16_t v = iqs_read(tile, IQS7211A_REG_TP_TOUCHES);
    v = (uint16_t)((v & 0x00FF) | ((uint16_t)fingers << 8));
    iqs_write(tile, IQS7211A_REG_TP_TOUCHES, v);
}

void tile_sense_cap_set_resolution(tile_t *tile, uint16_t x_res, uint16_t y_res)
{
    if (tile->state != TILE_STATE_READY) return;
    iqs_write(tile, IQS7211A_REG_X_RESOLUTION, x_res);
    iqs_write(tile, IQS7211A_REG_Y_RESOLUTION, y_res);

    iqs7211a_state_t *s = state_for(tile);
    s->x_res = x_res;
    s->y_res = y_res;
}

void tile_sense_cap_set_touch_multipliers(tile_t *tile, uint8_t set_mult,
                                          uint8_t clear_mult)
{
    if (tile->state != TILE_STATE_READY) return;
    iqs_write_verified(tile, IQS7211A_REG_TOUCH_MULT,
                       (uint16_t)(((uint16_t)clear_mult << 8) | set_mult));
}

void tile_sense_cap_set_alp_threshold(tile_t *tile, uint16_t threshold)
{
    if (tile->state != TILE_STATE_READY) return;
    iqs_write_verified(tile, IQS7211A_REG_ALP_THRESHOLD, threshold);
}

void tile_sense_cap_set_event_mode(tile_t *tile, uint8_t enable, uint16_t events)
{
    if (tile->state != TILE_STATE_READY) return;

    uint16_t clear = IQS7211A_CFG_EVENT_MODE;
    uint16_t set   = enable ? IQS7211A_CFG_EVENT_MODE : 0;

    if (events) {
        clear |= IQS7211A_CFG_EVENT_MASK;
        set   |= (uint16_t)(events & IQS7211A_CFG_EVENT_MASK);
    }
    iqs_modify(tile, IQS7211A_REG_CONFIG_SETTINGS, clear, set);
}

void tile_sense_cap_set_watchdog(tile_t *tile, uint8_t enable)
{
    if (tile->state != TILE_STATE_READY) return;
    iqs_modify(tile, IQS7211A_REG_CONFIG_SETTINGS,
               IQS7211A_CFG_WDT_EN, enable ? IQS7211A_CFG_WDT_EN : 0);
}

uint8_t tile_sense_cap_re_ati(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY) return 0;

    /* Clear any stale completion flags, then queue both channels. */
    iqs_read(tile, IQS7211A_REG_INFO_FLAGS);
    iqs_command(tile, (uint16_t)(IQS7211A_CTRL_TP_RE_ATI | IQS7211A_CTRL_ALP_RE_ATI));

    /* ATI runs when the channels are next sensed, so wait on the
     * completion flags rather than a fixed delay. */
    for (uint8_t i = 0; i < 50; i++) {
        tile->hal->delay_ms(10);
        uint16_t info = iqs_read(tile, IQS7211A_REG_INFO_FLAGS);

        if (info & (IQS7211A_INFO_ATI_ERROR | IQS7211A_INFO_ALP_ATI_ERROR))
            return 0;
        if (info & IQS7211A_INFO_RE_ATI_OCCURRED)
            return 1;
    }
    return 0;   /* timed out */
}

void tile_sense_cap_reseed(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY) return;
    iqs_command(tile, (uint16_t)(IQS7211A_CTRL_TP_RESEED | IQS7211A_CTRL_ALP_RESEED));
}

/* ================================================================
 * Advanced / escape hatch
 * ================================================================ */

uint16_t tile_sense_cap_read_reg(tile_t *tile, uint8_t reg)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_FOUND)
        return 0;
    /* Returns IQS7211A_INVALID_RESPONSE verbatim when no window opened —
     * callers at this level want to see that, not a laundered zero. */
    return iqs_read(tile, reg);
}

void tile_sense_cap_write_reg(tile_t *tile, uint8_t reg, uint16_t value)
{
    if (tile->state != TILE_STATE_READY && tile->state != TILE_STATE_FOUND)
        return;
    iqs_write_verified(tile, reg, value);
}
