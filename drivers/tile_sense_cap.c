/**
 * @file   tile_sense_cap.c
 * @brief  Sense.CAP (IQS7211A) -- platform-agnostic driver implementation.
 *
 * All bus access via tile->hal function pointers.
 * Reference: Azoteq IQS7211A Datasheet v1.3, April 2025.
 *
 * Comms model (see the header for the policy). Every access goes through
 * three primitives:
 *   - rd_words(): one read attempt; on 0xEEEE (no window) ask for one
 *     (write 0x00 to 0xFF, §11.9.2) and retry every millisecond until
 *     the window opens, within a real-time budget of one report period
 *     of the chip's current mode plus a margin.
 *   - wr_word(): a write outside a window is dropped by the chip, and
 *     the host cannot tell. After a comms request the same (idempotent)
 *     write is repeated every millisecond for one budget: the first
 *     attempt inside the window lands and its STOP closes the window
 *     (§11.7), the rest fall outside and are dropped. Callers verify by
 *     reading back.
 *   - a comms request is only ever sent right after a transaction proved
 *     the window closed (a 0xEEEE read, or the STOP of a completed
 *     transaction): a request written into an open window would be
 *     taken as a write to 0xFF and end that window instead.
 * Before the baseline sets Comms Request EN, the chip clock-stretches
 * instead (Table A.7 bit 4 = 0); the primitives then do single
 * transactions. Nothing derived from a failed or 0xEEEE read is ever
 * written back.
 *
 * The apply job and the ATI machine (surface_poll(), process(),
 * ati_poll()) never block for a whole budget. Each call makes at most one
 * register transaction that may wait for a window, and that wait is
 * capped at step_cap() (one Active report period + 2 ms, at most 20 ms):
 *   - rd_words() in step mode sends at most one request and gives up at
 *     the cap; the request stays pending, and the chip holds an unserviced
 *     window open for its I2C timeout, so the next call's read lands.
 *     miss_expired() turns misses into a failure only after the old
 *     blocking budget (three window budgets) of real time.
 *   - wr_slice() repeats one write across calls until the whole interval
 *     in which the requested window can open has been covered; only then
 *     is the write done (and its window consumed). A write therefore never
 *     misses a window that opens after the call returned.
 */

#include "tile_sense_cap.h"
#include <stddef.h>

/* ================================================================
 * Instance -> I2C address table
 * ================================================================ */

static const uint8_t id_table[] = {
    IQS7211A_I2C_ADDR,   /* 0: 0x56 */
};

#define NUM_INSTANCES  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    return (instance < NUM_INSTANCES) ? id_table[instance] : 0;
}

/* ================================================================
 * Constants
 * ================================================================ */

/* Comms */
#define IQS_POLL_MS            1     /* spacing of read/write attempts while waiting for a window */
#define IQS_WIN_MARGIN_MS      8     /* added to the report period for a window budget */
#define IQS_BUDGET_UNKNOWN_MS  250   /* budget before the chip's report rates are known */
#define IQS_BUDGET_MAX_MS      1000
#define IQS_PROCESS_RETRY_MS   3     /* process(): min spacing of read attempts while a window is pending */
#define IQS_ERR_REPORT_RUN     50    /* consecutive failed process() reads before ERR_COMMS */
#define IQS_FIND_TIMEOUT_MS    400   /* find() and init()'s identify: covers one LP2 window */
#define IQS_STEP_MARGIN_MS     2     /* step_cap(): added to the Active report period */
#define IQS_STEP_WAIT_MAX_MS   20    /* step_cap() ceiling, whatever the rates */

/* Reset. The datasheet gives no boot time; 100 ms is what v0.x used, the
 * ring waits 150 ms, so wait 100 ms and then poll for the product number. */
#define IQS_BOOT_MIN_MS        100
#define IQS_BOOT_TIMEOUT_MS    600
#define IQS_MCLR_LOW_MS        2     /* >= 250 ns needed (Table 4.2); the tile RC is ~0.5 ms */
#define IQS_RESET_ACK_RETRY_MS 1000

/* ATI (§5.6.3: runs once the window is terminated; comms resume after it) */
#define ATI_QUIET_MS           100   /* no bus traffic after queuing */
#define ATI_POLL_MS            50    /* gap between completion checks */
#define ATI_ROUND_TIMEOUT_MS   4000
#define ATI_TP_ROUNDS          3     /* a cold part often needs a second round (ring, 2026-09) */
#define ATI_ALP_ROUNDS         2

/* Jobs */
#define JOB_OPS_PER_STEP       1     /* register transactions per job step (one window wait) */
#define JOB_RETRIES            3
#define JOB_RESTARTS_MAX       3
#define CONFIGURE_TIMEOUT_MS   20000

/* Baseline values (the power-on values Studio documents) */
#define BASE_I2C_TIMEOUT_MS    50
#define BASE_TAP_MS            300
#define BASE_HOLD_MS           300   /* long press at tap + hold = 600 ms, as v0.x */
#define BASE_SWIPE_MS          500
#define BASE_SWIPE_ANGLE_DEG   45
#define BASE_MAX_TOUCHES       2

/* Recognizer constants, in pixels per 256 px of electrode pitch (the
 * hardware-proven values at the 2x3 grid's 256 px pitch). */
#define PITCH_PX               256
#define TOUCH_TAP_DIST         48
#define TOUCH_MOVE_DEAD        4
#define TOUCH_PINCH_DEAD       3
#define TOUCH_FLING_PXS        400   /* px/s */
#define TOUCH_DOUBLE_MS        350
#define TOUCH_CHIP_GATE_MS     400   /* chip gesture bits count only this close to a touch */
#define TOUCH_EVQ_LEN          8

#define XY_INVALID             0xFFFF  /* empty finger slot, observed on hardware */

/* Cached data block, in registers from 0x10 */
#define BLK_INFO_FLAGS   0
#define BLK_GESTURES     1
#define BLK_RELATIVE_X   2
#define BLK_RELATIVE_Y   3
#define BLK_FINGER_BASE  4   /* finger n: base + n*4 = X, Y, strength, area */
#define BLK_WORDS        12  /* 0x10..0x1B */

/* xf_read results */
#define XF_OK    0
#define XF_BUSY  1   /* 0xEEEE: no window */
#define XF_BUS   2   /* HAL error (NACK, host timeout) */

/* Power profiles: report periods Active, Idle-touch, Idle, LP1, LP2 (ms,
 * §6.1) and timeouts Active, Idle-touch, Idle, LP1 (s, §6.2). */
typedef struct {
    uint16_t rate[5];
    uint16_t tmo[4];
} profile_t;

static const profile_t PROFILES[4] = {
    { { 10, 10, 10, 10, 10 },     { 0, 0, 0, 0 } },      /* ALWAYS_ACTIVE */
    { { 10, 20, 20, 50, 100 },    { 10, 30, 20, 60 } },  /* RESPONSIVE */
    { { 10, 50, 50, 100, 200 },   { 5, 30, 10, 30 } },   /* BALANCED (Table 3.4) */
    { { 20, 100, 100, 160, 320 }, { 2, 20, 3, 10 } },    /* LOW_POWER */
};

/* 64 * tan(deg), deg 0..75 (register 0x87, §8.3). */
static const uint8_t TAN64[76] = {
      0,   1,   2,   3,   4,   6,   7,   8,   9,  10,  11,  12,  14,  15,  16,  17,
     18,  20,  21,  22,  23,  25,  26,  27,  28,  30,  31,  33,  34,  35,  37,  38,
     40,  42,  43,  45,  46,  48,  50,  52,  54,  56,  58,  60,  62,  64,  66,  69,
     71,  74,  76,  79,  82,  85,  88,  91,  95,  99, 102, 107, 111, 115, 120, 126,
    131, 137, 144, 151, 158, 167, 176, 186, 197, 209, 223, 239,
};

/* Register blocks the driver owns, in apply order. The rates block (index
 * 0) is synced first of all, so the rest of the job runs at the baseline's
 * cadence (in clock-stretch mode every access waits one report period). */
static const uint8_t MAIN_BLK_REG[] = { 0x40, 0x52, 0x30, 0x60, 0x70, 0x80, 0x90 };
static const uint8_t MAIN_BLK_N[]   = { 11,   10,   10,   10,   4,    8,    6    };
#define MAIN_BLKS  (sizeof(MAIN_BLK_REG))
static const uint8_t CYC_BLK_REG[]  = { IQS7211A_REG_CYCLE_BASE_0, IQS7211A_REG_CYCLE_BASE_10 };
static const uint8_t CYC_BLK_N[]    = { 15, 12 };
#define CYC_BLKS   2

/* Job phases. J_PROBE (learn the comms mode and rates) runs first after
 * init or a chip reset; J_RSTCHK asks "did the chip reset?" after a
 * failure, one read per call. */
enum {
    J_IDLE = 0, J_PROBE, J_GEO, J_RATES, J_CLEAR, J_BLOCKS, J_CYCLES, J_CONFIG, J_ACK, J_ATI,
    J_RSTCHK, J_DONE, J_FAILED,
};

/* ATI phases */
enum {
    A_IDLE = 0,
    A_TP_CMD, A_TP_QUIET, A_TP_POLL, A_TP_VERDICT, A_TP_COUNTS,
    A_ALP_ENTER, A_ALP_CMD, A_ALP_QUIET, A_ALP_POLL, A_ALP_VERDICT, A_ALP_EXIT,
    A_DONE,
};

/* vw_step() targets: verified writes the job and the ATI make */
enum { VW_CONFIG = 0, VW_MODE = 1 };

/* ================================================================
 * Per-instance driver state
 * ================================================================ */

typedef struct {
    /* ---- callbacks, clock, reset pin ---- */
    sense_cap_event_cb_t on_event;  void *event_ctx;
    uint32_t (*millis)(void *);     void *millis_ctx;
    void (*mclr)(void *, uint8_t);  void *mclr_ctx;
    uint32_t tick;                  /* fallback clock: driver delays + one period per read */

    /* ---- comms ---- */
    uint8_t  cr_mode;               /* chip believed to have Comms Request EN set */
    uint8_t  req_pending;           /* a comms request is outstanding */
    uint32_t req_t;
    uint32_t last_try_t;
    uint16_t err_run;
    uint8_t  err_reported;
    uint8_t  boost;                 /* use the slowest mode's budget (after a miss) */
    uint8_t  rates_known;
    uint16_t chip_rate[5];          /* report periods the chip actually holds */
    uint16_t chip_i2c_to;           /* the chip's I2C timeout, 0 = unknown */
    uint8_t  step;                  /* in a non-blocking step: window waits capped at step_cap() */
    uint8_t  wr_on;                 /* a sliced write is in progress (same write next call) */
    uint8_t  miss_on;               /* a sliced read has found no window yet ... */
    uint32_t miss_t;                /* ... since this time */
    uint8_t  probed;                /* comms mode + rates learned since init / the last reset */
    uint8_t  learn_cfg;             /* J_PROBE records the chip's Config Settings as s->config */

    /* ---- identity ---- */
    uint16_t ver_major, ver_minor;

    /* ---- chip shadows ---- */
    uint16_t config;                /* last Config Settings verified on the chip */
    uint8_t  mode_sel;              /* System Control mode select (manual control) */
    uint8_t  ati_manual;            /* manual control held for an ALP ATI */
    uint8_t  need_ack;
    uint8_t  ack_verified;          /* an Ack Reset was seen to clear Show Reset */
    uint32_t reset_t;

    /* ---- desired settings ---- */
    uint8_t  skip_baseline;
    uint8_t  profile, power_mode, alp_wake, wdt, sens, gest_src, max_touches_user;
    uint8_t  mult_set, mult_clear;
    uint16_t rate[5], tmo[4];
    uint16_t i2c_timeout;
    uint16_t ref_update;            uint8_t ref_update_set;
    uint16_t alp_threshold;         uint8_t alp_thr_set;
    uint8_t  user_flip_x, user_flip_y, user_switch;
    uint16_t user_xres, user_yres;
    uint16_t tap_ms, hold_ms, swipe_ms, swipe_dist_user;
    uint8_t  swipe_angle_deg;
    uint16_t gest_mask;

    /* ---- surface ---- */
    sense_cap_surface_t srf;
    uint8_t  layout;
    uint8_t  has_srf;               /* the surface senses at least one channel */
    uint8_t  srf_applied, srf_tuned;
    uint8_t  base_ok;               /* the last job verified the register image (baseline) */
    uint8_t  n_ch;
    uint8_t  ch_alloc;              /* bit n = channel n allocated to a cycle */
    uint8_t  cyc[18][2];
    uint8_t  alp_rx;                /* effective ALP Rx enables (0x72) */
    uint16_t alp_tx;                /* effective ALP Tx enables (0x73) */

    /* ---- apply job ---- */
    struct {
        uint8_t phase, next, blk, sub, idx, vrounds, tries, restarts, dirty, ati_done, clear;
        sense_cap_result_t result;
        sense_cap_result_t pend_err;    /* J_RSTCHK: the failure being examined */
        uint16_t cur[16];
    } job;

    /* ---- verified write in progress (vw_step) ---- */
    struct {
        uint8_t stage, tries;
    } vw;

    /* ---- ATI machine ---- */
    struct {
        uint8_t phase, sub, targets, round, req_sent, standalone;
        uint16_t target;                /* trackpad ATI target for the verdict */
        uint32_t t_cmd, t_req, t_next;
        sense_cap_result_t result;
    } ati;

    /* ---- data cache ---- */
    uint16_t block[BLK_WORDS];

    /* ---- touch state machine ---- */
    uint16_t px[2], py[2];
    uint8_t  presence_cnt[2];
    uint16_t down_x[2], down_y[2];
    uint32_t down_t[2];
    uint32_t move_t[2];
    uint8_t  was_down, moved_far, hold_fired;
    int16_t  vx, vy;
    uint32_t last_tap_t;
    uint8_t  tap_armed;             /* a single tap at last_tap_t may pair into a double */
    uint32_t last_up_t;
    uint8_t  up_seen;               /* last_up_t holds a real finger lift */
    uint16_t pinch_dist;
    uint8_t  pinch_valid;
    uint16_t prev_gestures;
    uint16_t ev_latch;
    uint16_t ev_call;               /* events raised during this process() call */

    sense_cap_touch_t evq[TOUCH_EVQ_LEN];
    uint8_t  evq_rd, evq_count;

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
 * Small helpers
 * ================================================================ */

static void memzero(void *p, uint16_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) *b++ = 0;
}

static uint16_t u16min(uint16_t a, uint16_t b) { return a < b ? a : b; }
static int16_t iabs16(int16_t v) { return (int16_t)(v < 0 ? -v : v); }

static uint32_t now_ms(iqs7211a_state_t *s)
{
    return s->millis ? s->millis(s->millis_ctx) : s->tick;
}

static void sleep_ms(tile_t *tile, iqs7211a_state_t *s, uint32_t ms)
{
    tile->hal->delay_ms(ms);
    s->tick += ms;
}

static uint8_t ready(tile_t *tile)
{
    return tile && tile->hal && tile->state == TILE_STATE_READY;
}

/* ================================================================
 * Comms primitives
 * ================================================================ */

static uint8_t xf_read(tile_t *tile, uint16_t reg, uint8_t *buf, uint16_t len)
{
    buf[0] = 0xEE;
    buf[1] = 0xEE;
    if (tile->hal->i2c_read(tile->hal->handle, tile->id, reg, buf, len) != 0)
        return XF_BUS;
    if (buf[0] == 0xEE && buf[1] == 0xEE)
        return XF_BUSY;
    return XF_OK;
}

static int xf_write16(tile_t *tile, uint8_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(value & 0xFF);
    buf[1] = (uint8_t)(value >> 8);
    return tile->hal->i2c_write(tile->hal->handle, tile->id, reg, buf, 2);
}

/** Ask for the next communication window (§11.9.2): 0x00 to 0xFF. */
static void comms_request(tile_t *tile, iqs7211a_state_t *s)
{
    uint8_t zero = 0;
    (void)tile->hal->i2c_write(tile->hal->handle, tile->id,
                               IQS7211A_REG_END_COMMS, &zero, 1);
    s->req_pending = 1;
    s->req_t = now_ms(s);
}

/** Mode the chip is (believed to be) in, for window budgets. */
static uint8_t current_mode(iqs7211a_state_t *s)
{
    if (s->ati_manual || s->power_mode != SENSE_CAP_POWER_AUTO)
        return (uint8_t)(s->mode_sel & 7U);
    return (uint8_t)(s->block[BLK_INFO_FLAGS] & IQS7211A_INFO_MODE_MASK);
}

/** How long a requested window can take to open: one report period of
 *  the current mode plus a margin (the slowest mode after a miss). */
static uint32_t win_budget(iqs7211a_state_t *s)
{
    if (!s->rates_known) return IQS_BUDGET_UNKNOWN_MS;
    uint16_t r = 0;
    if (s->boost) {
        for (uint8_t i = 0; i < 5; i++)
            if (s->chip_rate[i] > r) r = s->chip_rate[i];
    } else {
        /* The chip leaves Active for Idle the moment a finger lifts
         * (Figure 6.1), so an "awake" cached mode covers all three. */
        uint8_t m = current_mode(s);
        if (m <= SENSE_CAP_MODE_IDLE) {
            for (uint8_t i = 0; i <= SENSE_CAP_MODE_IDLE; i++)
                if (s->chip_rate[i] > r) r = s->chip_rate[i];
        } else {
            r = s->chip_rate[m < 5 ? m : 4];
        }
    }
    uint32_t b = (uint32_t)r + IQS_WIN_MARGIN_MS;
    return b > IQS_BUDGET_MAX_MS ? IQS_BUDGET_MAX_MS : b;
}

/** Longest one non-blocking step may wait for a window: one Active report
 *  period plus a margin, at most IQS_STEP_WAIT_MAX_MS and never more than
 *  the window budget. A window that opens later is caught next call. */
static uint32_t step_cap(iqs7211a_state_t *s)
{
    uint32_t c = (uint32_t)(s->rates_known ? s->chip_rate[SENSE_CAP_MODE_ACTIVE] : 10U) +
                 IQS_STEP_MARGIN_MS;
    uint32_t b = win_budget(s);
    if (c > IQS_STEP_WAIT_MAX_MS) c = IQS_STEP_WAIT_MAX_MS;
    return c < b ? c : b;
}

/** A sliced read found no window this call. Returns 1 once the old
 *  blocking budget (three window budgets) of real time has gone by since
 *  the first miss, i.e. when the miss counts as a failed try. */
static uint8_t miss_expired(iqs7211a_state_t *s)
{
    uint32_t now = now_ms(s);
    if (!s->miss_on) {
        s->miss_on = 1;
        s->miss_t = now;
        return 0;
    }
    if (now - s->miss_t <= win_budget(s) * 3U + 20U) return 0;
    s->miss_on = 0;
    return 1;
}

/** A pending request is stale once its window must have opened and timed out. */
static uint8_t req_stale(iqs7211a_state_t *s)
{
    if (!s->req_pending) return 1;
    uint32_t b = win_budget(s);
    uint32_t to = s->chip_i2c_to ? s->chip_i2c_to : b;
    return (now_ms(s) - s->req_t) > (b + to);
}

/** Record the report rates / I2C timeout from a read of 0x40.. */
static void learn_rates(iqs7211a_state_t *s, uint8_t first, const uint16_t *w, uint8_t n)
{
    if (first != IQS7211A_REG_RATE_ACTIVE || n < 5) return;
    for (uint8_t i = 0; i < 5; i++)
        s->chip_rate[i] = w[i] ? w[i] : 1;
    s->rates_known = 1;
    if (n >= 11)
        s->chip_i2c_to = w[10];
}

/**
 * Read n consecutive 16-bit words (8-bit or extended 16-bit address),
 * waiting for a window within the budget (three window budgets, up to
 * three requests). In step mode the wait is step_cap() with at most one
 * request; the caller retries next call. Returns 1 on a valid read;
 * `out` is untouched on failure.
 */
static uint8_t rd_words(tile_t *tile, iqs7211a_state_t *s, uint16_t reg,
                        uint16_t *out, uint8_t n)
{
    uint8_t buf[64];
    if (n == 0 || n > 32) return 0;

    uint32_t limit = s->step ? step_cap(s) : win_budget(s) * 3U + 20U;
    uint8_t max_req = s->step ? 1 : 3;
    uint32_t t0 = now_ms(s);
    uint32_t max_iters = limit / IQS_POLL_MS + 20U;
    uint32_t iters = 0;
    uint8_t requests = 0;

    for (;;) {
        uint8_t r = xf_read(tile, reg, buf, (uint16_t)(n * 2U));
        if (r == XF_OK) {
            s->req_pending = 0;
            s->miss_on = 0;
            for (uint8_t i = 0; i < n; i++)
                out[i] = (uint16_t)(((uint16_t)buf[2 * i + 1] << 8) | buf[2 * i]);
            if (reg <= 0xFF) learn_rates(s, (uint8_t)reg, out, n);
            return 1;
        }
        if (r == XF_BUSY) s->cr_mode = 1;   /* only Comms Request mode answers 0xEEEE */

        if ((r == XF_BUSY || s->cr_mode) && req_stale(s)) {
            if (requests >= max_req) return 0;
            comms_request(tile, s);
            requests++;
        }
        if (++iters > max_iters || now_ms(s) - t0 >= limit)
            return 0;
        sleep_ms(tile, s, IQS_POLL_MS);
    }
}

/**
 * Write one register inside a window. In Comms Request mode the write
 * is repeated for one budget after the request (idempotent; see the file
 * header). Unverified: the caller reads back. `stop_on_nack` ends the
 * attempts once the chip stops answering (a software reset took hold).
 */
static void wr_word(tile_t *tile, iqs7211a_state_t *s, uint8_t reg, uint16_t value,
                    uint8_t stop_on_nack)
{
    if (!s->cr_mode) {
        /* Clock-stretch mode: the write waits for the window itself. */
        (void)xf_write16(tile, reg, value);
        s->req_pending = 0;
        return;
    }
    if (req_stale(s))
        comms_request(tile, s);

    uint32_t budget = win_budget(s);
    uint32_t max_iters = budget / IQS_POLL_MS + 5U;
    uint32_t iters = 0;
    uint8_t acked = 0;
    do {
        sleep_ms(tile, s, IQS_POLL_MS);
        if (xf_write16(tile, reg, value) == 0)
            acked = 1;
        else if (stop_on_nack && acked)
            break;
        iters++;
    } while (iters < max_iters && (iters < 3U || now_ms(s) - s->req_t < budget));
    s->req_pending = 0;   /* its window was used or has expired */
}

/**
 * Non-blocking wr_word(): one slice of at most step_cap() per call.
 * Returns 1 when the write is done, 0 to call again next time with the
 * SAME register and value. In Comms Request mode the write is repeated
 * until the whole interval in which the requested window can open (one
 * window budget from the request) has been covered, across as many calls
 * as that takes; the chip holds an unserviced window open for its I2C
 * timeout, so a window that opens between calls is still caught. Like
 * wr_word(), unverified: the caller reads back.
 */
static uint8_t wr_slice(tile_t *tile, iqs7211a_state_t *s, uint8_t reg, uint16_t value)
{
    if (!s->cr_mode) {
        /* Clock-stretch mode: the write waits for the window itself. */
        (void)xf_write16(tile, reg, value);
        s->req_pending = 0;
        s->wr_on = 0;
        return 1;
    }
    if (!s->wr_on) {
        s->wr_on = 1;
        if (req_stale(s))
            comms_request(tile, s);
    }

    uint32_t t0 = now_ms(s);
    uint32_t budget = win_budget(s);
    uint32_t cap = step_cap(s);
    uint32_t iters = 0;
    for (;;) {
        sleep_ms(tile, s, IQS_POLL_MS);
        (void)xf_write16(tile, reg, value);
        iters++;
        uint32_t now = now_ms(s);
        if (iters >= 3U && now - s->req_t >= budget) {
            s->wr_on = 0;
            s->req_pending = 0;   /* its window was used or has expired */
            return 1;
        }
        if (now - t0 >= cap || iters > cap / IQS_POLL_MS + 5U)
            return 0;
    }
}

/** Write and read back, up to three times. */
static uint8_t wr_verified(tile_t *tile, iqs7211a_state_t *s, uint8_t reg, uint16_t value)
{
    for (uint8_t tries = 0; tries < 3; tries++) {
        uint16_t rb;
        wr_word(tile, s, reg, value, 0);
        if (rd_words(tile, s, reg, &rb, 1) && rb == value) {
            s->boost = 0;
            return 1;
        }
        s->boost = 1;
    }
    return 0;
}

/**
 * Send a System Control command. Every bit but the mode select is a
 * one-shot the chip consumes, so the word is built from the mode shadow,
 * never from a read (an 0xEEEE read would carry SW Reset and Tx test).
 * The Tx short test is never set (RX0/TX0 is grounded on this tile).
 */
static void send_command(tile_t *tile, iqs7211a_state_t *s, uint16_t bits)
{
    uint16_t v = (uint16_t)((s->mode_sel & IQS7211A_CTRL_MODE_MASK) | bits);
    v &= (uint16_t)~IQS7211A_CTRL_TX_TEST;
    wr_word(tile, s, IQS7211A_REG_SYSTEM_CONTROL, v,
            (bits & IQS7211A_CTRL_SW_RESET) ? 1 : 0);
}

/** send_command() as a wr_slice(): 1 when done, 0 to call again. Never
 *  used for SW Reset (reset() is the blocking path). */
static uint8_t cmd_slice(tile_t *tile, iqs7211a_state_t *s, uint16_t bits)
{
    uint16_t v = (uint16_t)((s->mode_sel & IQS7211A_CTRL_MODE_MASK) | bits);
    v &= (uint16_t)~(IQS7211A_CTRL_TX_TEST | IQS7211A_CTRL_SW_RESET);
    return wr_slice(tile, s, IQS7211A_REG_SYSTEM_CONTROL, v);
}

/**
 * Select a charging mode under manual control and confirm it: the Mode
 * Select field is a setting, not a one-shot, so it reads back (Table
 * A.6). Budgets use the slowest mode, since the chip may still be
 * cycling at the old mode's rate.
 */
static uint8_t select_mode(tile_t *tile, iqs7211a_state_t *s, uint8_t mode)
{
    s->mode_sel = (uint8_t)(mode & IQS7211A_CTRL_MODE_MASK);
    for (uint8_t tries = 0; tries < 3; tries++) {
        uint16_t rb;
        s->boost = 1;
        send_command(tile, s, 0);
        if (rd_words(tile, s, IQS7211A_REG_SYSTEM_CONTROL, &rb, 1) &&
            (rb & IQS7211A_CTRL_MODE_MASK) == s->mode_sel) {
            s->boost = 0;
            return 1;
        }
    }
    s->boost = 0;
    return 0;
}

/* ================================================================
 * Surface geometry and the desired register image
 * ================================================================ */

static uint8_t eff_switch(const iqs7211a_state_t *s)
{
    return (uint8_t)((s->srf.geometry.switch_xy ^ s->user_switch) & 1U);
}
static uint8_t eff_flip_x(const iqs7211a_state_t *s)
{
    return (uint8_t)((s->srf.geometry.flip_x ^ s->user_flip_x) & 1U);
}
static uint8_t eff_flip_y(const iqs7211a_state_t *s)
{
    return (uint8_t)((s->srf.geometry.flip_y ^ s->user_flip_y) & 1U);
}

/* Electrodes along the output X / Y axes (§7.7: Rxs are columns (X)
 * unless Switch XY puts the Txs there). */
static uint8_t n_x(const iqs7211a_state_t *s)
{
    return eff_switch(s) ? s->srf.geometry.n_tx : s->srf.geometry.n_rx;
}
static uint8_t n_y(const iqs7211a_state_t *s)
{
    return eff_switch(s) ? s->srf.geometry.n_rx : s->srf.geometry.n_tx;
}

/** 256 points between electrodes (§7.4). A one-electrode axis has no range. */
static uint16_t auto_res(uint8_t n)
{
    return (uint16_t)(n > 1 ? (n - 1U) * PITCH_PX : 0U);
}

static uint16_t eff_xres(const iqs7211a_state_t *s)
{
    if (s->user_xres) return s->user_xres;
    uint16_t r = s->user_switch ? s->srf.geometry.y_res : s->srf.geometry.x_res;
    return r ? r : auto_res(n_x(s));
}
static uint16_t eff_yres(const iqs7211a_state_t *s)
{
    if (s->user_yres) return s->user_yres;
    uint16_t r = s->user_switch ? s->srf.geometry.x_res : s->srf.geometry.y_res;
    return r ? r : auto_res(n_y(s));
}

/** Output pixels per electrode pitch (256 on the 2x3 grid). */
static uint16_t pixels_per_pitch(const iqs7211a_state_t *s)
{
    uint32_t best = 0;
    uint8_t nx = n_x(s), ny = n_y(s);
    if (nx > 1) best = eff_xres(s) / (nx - 1U);
    if (ny > 1) {
        uint32_t v = eff_yres(s) / (ny - 1U);
        if (v > best) best = v;
    }
    if (best == 0) return PITCH_PX;
    return (uint16_t)(best > 0xFFFFU ? 0xFFFFU : best);
}

/** Scale a per-256-px-pitch constant to the configured resolution. */
static uint16_t scale_px(const iqs7211a_state_t *s, uint32_t c)
{
    uint32_t v = c * pixels_per_pitch(s) / PITCH_PX;
    if (v == 0) v = 1;
    return (uint16_t)(v > 0xFFFFU ? 0xFFFFU : v);
}

static uint16_t eff_swipe_dist(const iqs7211a_state_t *s)
{
    return s->swipe_dist_user ? s->swipe_dist_user : scale_px(s, TOUCH_TAP_DIST);
}

static uint8_t alp_in_use(const iqs7211a_state_t *s)
{
    return (uint8_t)(s->alp_wake && s->srf.alp.enable);
}

/** Idle only times out into LP1 while the ALP can wake the chip. */
static uint16_t eff_timeout(const iqs7211a_state_t *s, uint8_t i)
{
    if (i == SENSE_CAP_MODE_IDLE && !alp_in_use(s)) return 0;
    return s->tmo[i];
}

static uint8_t eff_max_touches(const iqs7211a_state_t *s)
{
    if (s->max_touches_user) return s->max_touches_user;
    if (s->srf.tuning.max_touches) return s->srf.tuning.max_touches;
    return BASE_MAX_TOUCHES;
}

static uint16_t eff_alp_threshold(const iqs7211a_state_t *s)
{
    return s->alp_thr_set ? s->alp_threshold : s->srf.alp.threshold;
}

static uint16_t base_word(uint8_t fine, uint8_t mult, uint8_t div)
{
    return (uint16_t)(((uint16_t)(fine & 0x1FU) << 9) |
                      ((uint16_t)(mult & 0x0FU) << 5) | (div & 0x1FU));
}

/** Config Settings (0x51) under the driver's comms policy (Table A.7). */
static uint16_t config_word(const iqs7211a_state_t *s)
{
    uint16_t v = (uint16_t)(IQS7211A_CFG_TP_RE_ATI_EN | IQS7211A_CFG_COMMS_REQUEST_EN |
                            IQS7211A_CFG_EVENT_MODE);
    if (alp_in_use(s))  v |= IQS7211A_CFG_ALP_RE_ATI_EN;
    if (s->wdt)         v |= IQS7211A_CFG_WDT_EN;
    if (s->ati_manual || s->power_mode != SENSE_CAP_POWER_AUTO)
        v |= IQS7211A_CFG_MANUAL_CONTROL;
    if (s->gest_src == SENSE_CAP_GESTURES_CHIP)
        v |= IQS7211A_CFG_GESTURE_EVENT;
    return v;
}

/** Cycle allocation (§7.1.2): each cycle senses one block-A (RX0-3) and
 *  one block-B (RX4-7) channel on the same Tx. The first slot carries
 *  block A (hardware-verified on the 2x3 grid; the datasheet is silent). */
static void compute_cycles(iqs7211a_state_t *s)
{
    const sense_cap_geometry_t *g = &s->srf.geometry;
    uint8_t n = 0;
    for (uint8_t c = 0; c < 18; c++) {
        s->cyc[c][0] = IQS7211A_CHANNEL_NONE;
        s->cyc[c][1] = IQS7211A_CHANNEL_NONE;
    }
    s->ch_alloc = 0;
    for (uint8_t t = 0; t < g->n_tx; t++) {
        uint8_t a[SENSE_CAP_MAX_RX], b[SENSE_CAP_MAX_RX], na = 0, nb = 0;
        for (uint8_t r = 0; r < g->n_rx; r++) {
            uint8_t ch = (uint8_t)(t * g->n_rx + r);
            if (g->disabled_channels & (1U << ch)) continue;
            if (g->rx[r] >= 4) b[nb++] = ch;
            else               a[na++] = ch;
            s->ch_alloc |= (uint8_t)(1U << ch);
        }
        for (uint8_t i = 0; (i < na || i < nb) && n < 18; i++) {
            s->cyc[n][0] = (i < na) ? a[i] : IQS7211A_CHANNEL_NONE;
            s->cyc[n][1] = (i < nb) ? b[i] : IQS7211A_CHANNEL_NONE;
            n++;
        }
    }
    s->n_ch = (uint8_t)(g->n_rx * g->n_tx);
    s->has_srf = s->ch_alloc ? 1 : 0;
}

/** One byte of the packed cycle table [0x05, 1st, 2nd] per cycle. */
static uint8_t cycle_byte(const iqs7211a_state_t *s, uint8_t idx, uint8_t clear)
{
    uint8_t c = (uint8_t)(idx / 3U), pos = (uint8_t)(idx % 3U);
    if (pos == 0) return IQS7211A_CYCLE_PROX_BYTE;
    if (clear || c >= 18) return IQS7211A_CHANNEL_NONE;
    return s->cyc[c][pos - 1U];
}

/** Rx/Tx map entry (§7.1.5): Rxs first, then Txs, 0xFF unused. */
static uint8_t map_entry(const iqs7211a_state_t *s, uint8_t i)
{
    const sense_cap_geometry_t *g = &s->srf.geometry;
    if (i < g->n_rx) return g->rx[i];
    if (i < g->n_rx + g->n_tx) return g->tx[i - g->n_rx];
    return 0xFF;
}

static uint16_t keep_bytes(uint16_t cur, uint8_t hi, uint8_t lo)
{
    uint16_t v = cur;
    if (hi) v = (uint16_t)((v & 0x00FFU) | ((uint16_t)hi << 8));
    if (lo) v = (uint16_t)((v & 0xFF00U) | lo);
    return v;
}

static uint8_t autoprox_code(uint8_t e, uint8_t cur_code)
{
    switch (e) {
    case SENSE_CAP_AUTOPROX_4:   return 0;
    case SENSE_CAP_AUTOPROX_8:   return 1;
    case SENSE_CAP_AUTOPROX_16:  return 2;
    case SENSE_CAP_AUTOPROX_32:  return 3;
    case SENSE_CAP_AUTOPROX_OFF: return 4;
    default:                     return cur_code;
    }
}

/**
 * The register image. Returns 1 if the driver owns `reg` and sets *out to
 * the wanted value given its current contents `cur` (so partly-owned
 * registers keep the bits the driver does not manage); 0 = leave alone.
 */
static uint8_t want_reg(const iqs7211a_state_t *s, uint8_t reg, uint16_t cur,
                        uint8_t clear_cycles, uint16_t *out)
{
    const sense_cap_geometry_t *g = &s->srf.geometry;
    const sense_cap_tuning_t *t = &s->srf.tuning;
    const sense_cap_alp_t *a = &s->srf.alp;
    uint16_t v;

    if (reg >= IQS7211A_REG_CYCLE_BASE_0 && reg <= 0xAE) {
        uint8_t i = (uint8_t)((reg - IQS7211A_REG_CYCLE_BASE_0) * 2U);
        *out = (uint16_t)(cycle_byte(s, i, clear_cycles) |
                          ((uint16_t)cycle_byte(s, (uint8_t)(i + 1U), clear_cycles) << 8));
        return 1;
    }
    if (reg >= IQS7211A_REG_CYCLE_BASE_10 && reg <= 0xBB) {
        uint8_t i = (uint8_t)(30U + (reg - IQS7211A_REG_CYCLE_BASE_10) * 2U);
        *out = (uint16_t)(cycle_byte(s, i, clear_cycles) |
                          ((uint16_t)cycle_byte(s, (uint8_t)(i + 1U), clear_cycles) << 8));
        return 1;
    }
    if (reg >= IQS7211A_REG_RXTX_MAP_BASE && reg <= 0x95) {
        uint8_t i = (uint8_t)((reg - IQS7211A_REG_RXTX_MAP_BASE) * 2U);
        *out = (uint16_t)(map_entry(s, i) | ((uint16_t)map_entry(s, (uint8_t)(i + 1U)) << 8));
        return 1;
    }
    if (reg >= IQS7211A_REG_RATE_ACTIVE && reg <= IQS7211A_REG_RATE_LP2) {
        *out = s->rate[reg - IQS7211A_REG_RATE_ACTIVE];
        return 1;
    }
    if (reg >= IQS7211A_REG_TIMEOUT_ACTIVE && reg <= IQS7211A_REG_TIMEOUT_LP1) {
        *out = eff_timeout(s, (uint8_t)(reg - IQS7211A_REG_TIMEOUT_ACTIVE));
        return 1;
    }

    switch (reg) {
    case IQS7211A_REG_TP_ATI_MULTDIV:
        if (!(t->ati_fine_div | t->ati_coarse_mult | t->ati_coarse_div)) return 0;
        *out = (uint16_t)((cur & 0xC000U) |
                          base_word(t->ati_fine_div, t->ati_coarse_mult, t->ati_coarse_div));
        return 1;
    case IQS7211A_REG_TP_ATI_COMP_DIV:
        if (!t->ati_comp_div) return 0;
        *out = t->ati_comp_div; return 1;
    case IQS7211A_REG_TP_ATI_TARGET:
        if (!t->ati_target) return 0;
        *out = t->ati_target; return 1;
    case IQS7211A_REG_TP_REF_DRIFT:
        if (!t->ref_drift_limit) return 0;
        *out = t->ref_drift_limit; return 1;
    case IQS7211A_REG_TP_MIN_COUNT:
        if (!t->min_count_reati) return 0;
        *out = t->min_count_reati; return 1;
    case IQS7211A_REG_REATI_RETRY:
        if (!t->reati_retry_s) return 0;
        *out = t->reati_retry_s; return 1;
    case IQS7211A_REG_ALP_ATI_MULTDIV:
        if (a->ati_fine_div | a->ati_coarse_mult | a->ati_coarse_div)
            v = base_word(a->ati_fine_div, a->ati_coarse_mult, a->ati_coarse_div);
        else if (t->ati_fine_div | t->ati_coarse_mult | t->ati_coarse_div)
            v = base_word(t->ati_fine_div, t->ati_coarse_mult, t->ati_coarse_div);
        else
            return 0;
        *out = (uint16_t)((cur & 0xC000U) | v);
        return 1;
    case IQS7211A_REG_ALP_ATI_COMP_DIV:
        if (!a->ati_comp_div) return 0;
        *out = a->ati_comp_div; return 1;
    case IQS7211A_REG_ALP_ATI_TARGET:
        if (!a->ati_target) return 0;
        *out = a->ati_target; return 1;
    case IQS7211A_REG_ALP_LTA_DRIFT:
        if (!a->lta_drift_limit) return 0;
        *out = a->lta_drift_limit; return 1;

    case IQS7211A_REG_REF_UPDATE_TIME:
        if (s->ref_update_set) { *out = s->ref_update; return 1; }
        if (!t->ref_update_s) return 0;
        *out = t->ref_update_s; return 1;
    case IQS7211A_REG_I2C_TIMEOUT:
        *out = s->i2c_timeout; return 1;

    case IQS7211A_REG_OTHER_SETTINGS:
        /* 14 MHz main oscillator; everything else as the chip has it. */
        *out = (uint16_t)(cur & (uint16_t)~IQS7211A_OTHER_18MHZ); return 1;
    case IQS7211A_REG_TOUCH_MULT:
        *out = (uint16_t)(((uint16_t)s->mult_clear << 8) | s->mult_set); return 1;
    case IQS7211A_REG_ALP_THRESHOLD:
        v = eff_alp_threshold(s);
        if (!v) return 0;
        *out = v; return 1;
    case IQS7211A_REG_ALP_DEBOUNCE:
        if (!a->set_debounce && !a->clear_debounce) return 0;
        *out = keep_bytes(cur, a->clear_debounce, a->set_debounce); return 1;
    case IQS7211A_REG_TP_HW_SETTINGS:
        *out = (uint16_t)(cur | IQS7211A_HW_RF_FILTER); return 1;
    case IQS7211A_REG_ALP_HW_SETTINGS: {
        uint8_t lp2 = (uint8_t)((cur >> IQS7211A_HW_LP2_AUTOPROX_SHIFT) & 7U);
        uint8_t lp1 = (uint8_t)((cur >> IQS7211A_HW_LP1_AUTOPROX_SHIFT) & 7U);
        lp2 = autoprox_code(a->lp2_auto_prox, lp2);
        lp1 = autoprox_code(a->lp1_auto_prox, lp1);
        v = (uint16_t)(cur & (uint16_t)~(0x3FU << 2));
        v |= (uint16_t)((uint16_t)lp2 << IQS7211A_HW_LP2_AUTOPROX_SHIFT);
        v |= (uint16_t)((uint16_t)lp1 << IQS7211A_HW_LP1_AUTOPROX_SHIFT);
        *out = (uint16_t)(v | IQS7211A_HW_RF_FILTER);
        return 1;
    }

    case IQS7211A_REG_TP_SETTINGS:
        v = (uint16_t)(((uint16_t)g->n_rx << 8) | (cur & 0x00C0U) | (t->filters & 0x38U));
        if (eff_switch(s)) v |= IQS7211A_TP_SWITCH_XY;
        if (eff_flip_x(s)) v |= IQS7211A_TP_FLIP_X;
        if (eff_flip_y(s)) v |= IQS7211A_TP_FLIP_Y;
        *out = v; return 1;
    case IQS7211A_REG_TP_TOUCHES:
        *out = (uint16_t)(((uint16_t)eff_max_touches(s) << 8) | g->n_tx); return 1;
    case IQS7211A_REG_X_RESOLUTION:
        *out = eff_xres(s); return 1;
    case IQS7211A_REG_Y_RESOLUTION:
        *out = eff_yres(s); return 1;
    case IQS7211A_REG_DYN_BOTTOM_SPEED:
        if (!t->dyn_bottom_speed) return 0;
        *out = t->dyn_bottom_speed; return 1;
    case IQS7211A_REG_DYN_TOP_SPEED:
        if (!t->dyn_top_speed) return 0;
        *out = t->dyn_top_speed; return 1;
    case IQS7211A_REG_FILTER_BETAS:
        if (!t->static_beta && !t->dyn_bottom_beta) return 0;
        *out = keep_bytes(cur, t->static_beta, t->dyn_bottom_beta); return 1;
    case IQS7211A_REG_SPLIT_STATIONARY:
        if (!t->finger_split && !t->stationary_thr) return 0;
        *out = keep_bytes(cur, t->finger_split, t->stationary_thr); return 1;
    case IQS7211A_REG_X_TRIM:
        if (!g->x_trim) return 0;
        *out = g->x_trim; return 1;
    case IQS7211A_REG_Y_TRIM:
        if (!g->y_trim) return 0;
        *out = g->y_trim; return 1;

    case IQS7211A_REG_ALP_COUNT_BETA:
        if (!a->count_beta) return 0;
        *out = a->count_beta; return 1;
    case IQS7211A_REG_ALP_LTA_BETAS:
        if (!a->lp1_lta_beta && !a->lp2_lta_beta) return 0;
        *out = keep_bytes(cur, a->lp2_lta_beta, a->lp1_lta_beta); return 1;
    case IQS7211A_REG_ALP_SETUP:
        /* Count filter always on (rev a: no series resistors). */
        v = (uint16_t)((cur & 0xFC00U) | IQS7211A_ALP_SETUP_FILTER | s->alp_rx);
        if (a->mutual) v |= IQS7211A_ALP_SETUP_MUTUAL;
        *out = v; return 1;
    case IQS7211A_REG_ALP_TX_ENABLE:
        *out = (uint16_t)((cur & 0xF000U) | (s->alp_tx & 0x0FFFU)); return 1;

    case IQS7211A_REG_GESTURE_ENABLE:
        v = (s->gest_src == SENSE_CAP_GESTURES_CHIP) ? (uint16_t)(s->gest_mask & 0x3FU) : 0U;
        *out = (uint16_t)((cur & 0xFFC0U) | v); return 1;
    case IQS7211A_REG_TAP_TIME:
        *out = s->tap_ms; return 1;
    case IQS7211A_REG_TAP_DISTANCE:
        *out = scale_px(s, TOUCH_TAP_DIST); return 1;
    case IQS7211A_REG_HOLD_TIME:
        *out = s->hold_ms; return 1;
    case IQS7211A_REG_SWIPE_TIME:
        *out = s->swipe_ms; return 1;
    case IQS7211A_REG_SWIPE_X_DIST:
    case IQS7211A_REG_SWIPE_Y_DIST:
        *out = eff_swipe_dist(s); return 1;
    case IQS7211A_REG_SWIPE_ANGLE:
        *out = (uint16_t)((cur & 0xFF00U) | TAN64[s->swipe_angle_deg <= 75 ? s->swipe_angle_deg : 45]);
        return 1;
    default:
        return 0;
    }
}

/* ================================================================
 * Blocking register-range sync (setters)
 * ================================================================ */

/** Bring registers [first, first+n) to the image: read, write only what
 *  differs, read back; up to three rounds. */
static uint8_t apply_range(tile_t *tile, iqs7211a_state_t *s, uint8_t first, uint8_t n)
{
    uint16_t cur[16];
    if (n == 0 || n > 16) return 0;
    for (uint8_t round = 0; round < 4; round++) {
        if (!rd_words(tile, s, first, cur, n)) {
            s->boost = 1;
            continue;
        }
        uint8_t diff = 0;
        for (uint8_t i = 0; i < n; i++) {
            uint16_t w;
            if (want_reg(s, (uint8_t)(first + i), cur[i], 0, &w) && w != cur[i]) {
                diff = 1;
                if (round < 3) wr_word(tile, s, (uint8_t)(first + i), w, 0);
            }
        }
        if (!diff) {
            /* Keep the window budgets in step with what the chip holds. */
            for (uint8_t i = 0; i < n; i++) {
                uint8_t r = (uint8_t)(first + i);
                if (r >= IQS7211A_REG_RATE_ACTIVE && r <= IQS7211A_REG_RATE_LP2)
                    s->chip_rate[r - IQS7211A_REG_RATE_ACTIVE] = cur[i] ? cur[i] : 1;
                if (r == IQS7211A_REG_I2C_TIMEOUT)
                    s->chip_i2c_to = cur[i];
            }
            s->boost = 0;
            return 1;
        }
        if (round) s->boost = 1;
    }
    return 0;
}

/** Write Config Settings from the driver policy and confirm it. */
static uint8_t config_apply(tile_t *tile, iqs7211a_state_t *s)
{
    uint16_t v = config_word(s);
    for (uint8_t tries = 0; tries < 3; tries++) {
        uint16_t rb;
        wr_word(tile, s, IQS7211A_REG_CONFIG_SETTINGS, v, 0);
        /* The comms method may just have changed: believe the write
         * first, then let the read-back tell the truth. */
        s->cr_mode = (v & IQS7211A_CFG_COMMS_REQUEST_EN) ? 1 : 0;
        if (rd_words(tile, s, IQS7211A_REG_CONFIG_SETTINGS, &rb, 1)) {
            s->cr_mode = (rb & IQS7211A_CFG_COMMS_REQUEST_EN) ? 1 : 0;
            if (rb == v) {
                s->config = v;
                s->boost = 0;
                return 1;
            }
        }
        s->boost = 1;
    }
    return 0;
}

/**
 * config_apply() / select_mode() for the job and the ATI machine, one
 * transaction per call: the write (sliced), then the read-back. Returns
 * SENSE_CAP_BUSY until verified (SENSE_CAP_OK), or SENSE_CAP_ERR_VERIFY
 * after three tries. VW_MODE writes s->mode_sel and gets the slowest
 * mode's budgets, as select_mode() does.
 */
static sense_cap_result_t vw_step(tile_t *tile, iqs7211a_state_t *s, uint8_t what)
{
    uint8_t reg = (what == VW_CONFIG) ? IQS7211A_REG_CONFIG_SETTINGS : IQS7211A_REG_SYSTEM_CONTROL;
    uint16_t v = (what == VW_CONFIG) ? config_word(s)
                                     : (uint16_t)(s->mode_sel & IQS7211A_CTRL_MODE_MASK);
    uint16_t rb;

    if (what == VW_MODE) s->boost = 1;
    if (s->vw.stage == 0) {
        if (!wr_slice(tile, s, reg, v)) return SENSE_CAP_BUSY;
        /* The comms method may just have changed: believe the write
         * first, then let the read-back tell the truth. */
        if (what == VW_CONFIG) s->cr_mode = (v & IQS7211A_CFG_COMMS_REQUEST_EN) ? 1 : 0;
        s->vw.stage = 1;
        return SENSE_CAP_BUSY;
    }
    if (rd_words(tile, s, reg, &rb, 1)) {
        uint8_t good;
        if (what == VW_CONFIG) {
            s->cr_mode = (rb & IQS7211A_CFG_COMMS_REQUEST_EN) ? 1 : 0;
            good = (uint8_t)(rb == v);
            if (good) s->config = v;
        } else {
            good = (uint8_t)((rb & IQS7211A_CTRL_MODE_MASK) == v);
        }
        if (good) {
            s->boost = 0;
            s->vw.stage = 0;
            s->vw.tries = 0;
            return SENSE_CAP_OK;
        }
    } else if (!miss_expired(s)) {
        return SENSE_CAP_BUSY;              /* no window yet: read again next call */
    }
    s->boost = 1;
    s->vw.stage = 0;
    if (++s->vw.tries >= 3) {
        s->vw.tries = 0;
        if (what == VW_MODE) s->boost = 0;
        return SENSE_CAP_ERR_VERIFY;
    }
    return SENSE_CAP_BUSY;
}

/* ================================================================
 * Touch recognizers
 * ================================================================ */

static void ev_raise(iqs7211a_state_t *s, uint16_t bits)
{
    s->ev_latch |= bits;
    s->ev_call  |= bits;
}

static void touch_push(iqs7211a_state_t *s, const sense_cap_touch_t *ev)
{
    if (s->evq_count == TOUCH_EVQ_LEN) {
        s->evq_rd = (uint8_t)((s->evq_rd + 1U) % TOUCH_EVQ_LEN);
        s->evq_count--;
    }
    s->evq[(uint8_t)((s->evq_rd + s->evq_count) % TOUCH_EVQ_LEN)] = *ev;
    s->evq_count++;
}

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
    ev.strength = s->block[BLK_FINGER_BASE + finger * 4U + 2U];
    ev.t_ms     = now;
    touch_push(s, &ev);
    if (s->on_touch)
        s->on_touch(tile, &ev, s->touch_ctx);
}

/** Lift every tracked finger (after a chip reset), emitting UP events. */
static void touch_reset_all(tile_t *tile, iqs7211a_state_t *s, uint8_t emit)
{
    uint32_t now = now_ms(s);
    for (uint8_t n = 0; n < SENSE_CAP_NUM_FINGERS; n++) {
        if (emit && ((s->was_down >> n) & 1U)) {
            ev_raise(s, SENSE_CAP_EV_TOUCH_UP);
            touch_emit(tile, s, SENSE_CAP_TOUCH_UP, n, s->px[n], s->py[n], 0, 0, now);
        }
        s->presence_cnt[n] = 0;
    }
    s->was_down = 0;
    s->moved_far = 0;
    s->hold_fired = 0;
    s->pinch_valid = 0;
    s->prev_gestures = 0;
    memzero(s->block, sizeof(s->block));
}

static void fire_tap(tile_t *tile, iqs7211a_state_t *s, uint16_t x, uint16_t y, uint32_t now)
{
    uint8_t taps = 1;
    if (s->tap_armed && now - s->last_tap_t <= TOUCH_DOUBLE_MS) {
        taps = 2;
        s->tap_armed = 0;
        ev_raise(s, SENSE_CAP_EV_DOUBLE_TAP);
    } else {
        s->last_tap_t = now;
        s->tap_armed = 1;
        ev_raise(s, SENSE_CAP_EV_TAP);
    }
    if (s->on_tap)
        s->on_tap(tile, taps, x, y, s->tap_ctx);
}

static void fire_swipe(tile_t *tile, iqs7211a_state_t *s, uint8_t dir)
{
    static const uint16_t evb[4] = {
        SENSE_CAP_EV_SWIPE_LEFT, SENSE_CAP_EV_SWIPE_RIGHT,
        SENSE_CAP_EV_SWIPE_UP,   SENSE_CAP_EV_SWIPE_DOWN,
    };
    ev_raise(s, (uint16_t)(SENSE_CAP_EV_SWIPE | evb[dir & 3U]));
    if (s->on_swipe)
        s->on_swipe(tile, dir, s->vx, s->vy, s->swipe_ctx);
}

/** Gesture-enable bit that gates a swipe direction. */
static uint16_t dir_gesture_bit(uint8_t dir)
{
    switch (dir) {
    case SENSE_CAP_DIR_LEFT:  return SENSE_CAP_GESTURE_SWIPE_X_NEG;
    case SENSE_CAP_DIR_RIGHT: return SENSE_CAP_GESTURE_SWIPE_X_POS;
    case SENSE_CAP_DIR_UP:    return SENSE_CAP_GESTURE_SWIPE_Y_NEG;
    default:                  return SENSE_CAP_GESTURE_SWIPE_Y_POS;
    }
}

/**
 * Advance the touch state machine from the freshly cached block. The
 * driver recognizers keep the v0.x behaviour at the 2x3 grid's 256 px
 * pitch; their pixel constants scale with the resolution.
 */
static void touch_update(tile_t *tile, iqs7211a_state_t *s)
{
    uint32_t now = now_ms(s);
    uint8_t fingers = (uint8_t)((s->block[BLK_INFO_FLAGS] >> IQS7211A_INFO_NUM_FINGERS_SHIFT) & 3U);
    uint8_t drv = (s->gest_src == SENSE_CAP_GESTURES_DRIVER) ? 1 : 0;
    uint16_t tap_dist   = scale_px(s, TOUCH_TAP_DIST);
    uint16_t move_dead  = scale_px(s, TOUCH_MOVE_DEAD);
    uint16_t pinch_dead = scale_px(s, TOUCH_PINCH_DEAD);
    uint16_t fling      = scale_px(s, TOUCH_FLING_PXS);
    uint16_t swipe_dist = eff_swipe_dist(s);
    uint32_t hold_total = (uint32_t)s->tap_ms + s->hold_ms;   /* §8.2 */
    uint8_t  angle      = TAN64[s->swipe_angle_deg <= 75 ? s->swipe_angle_deg : 45];

    for (uint8_t n = 0; n < SENSE_CAP_NUM_FINGERS; n++) {
        uint16_t x = s->block[BLK_FINGER_BASE + n * 4U + 0U];
        uint16_t y = s->block[BLK_FINGER_BASE + n * 4U + 1U];
        uint8_t present = (uint8_t)((n < fingers) && x != XY_INVALID && y != XY_INVALID);
        uint8_t was = (uint8_t)((s->was_down >> n) & 1U);

        /* Presence debounce: a marginal touch can flicker the finger flag
         * every cycle; require two cycles in the new state. */
        if (present != was) {
            if (++s->presence_cnt[n] < 2) present = was;
            else                          s->presence_cnt[n] = 0;
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
            ev_raise(s, SENSE_CAP_EV_TOUCH_DOWN);
            touch_emit(tile, s, SENSE_CAP_TOUCH_DOWN, n, x, y, 0, 0, now);
        } else if (present && was) {
            /* A held finger can report one invalid frame: keep the last
             * good position rather than poisoning deltas and velocity. */
            if (x == XY_INVALID || y == XY_INVALID) {
                x = s->px[n];
                y = s->py[n];
            }
            int16_t dx = (int16_t)(x - s->px[n]);
            int16_t dy = (int16_t)(y - s->py[n]);

            if ((uint16_t)(iabs16(dx) + iabs16(dy)) >= move_dead) {
                uint32_t dt = now - s->move_t[n];
                if (dt == 0) dt = 1;
                s->vx = (int16_t)((int32_t)dx * 1000 / (int32_t)dt);
                s->vy = (int16_t)((int32_t)dy * 1000 / (int32_t)dt);

                uint16_t total = (uint16_t)(iabs16((int16_t)(x - s->down_x[n])) +
                                            iabs16((int16_t)(y - s->down_y[n])));
                if (total > tap_dist) {
                    s->moved_far |= (uint8_t)(1U << n);
                    ev_raise(s, SENSE_CAP_EV_DRAG);
                    if (s->on_drag)
                        s->on_drag(tile, dx, dy, x, y, s->drag_ctx);
                }
                touch_emit(tile, s, SENSE_CAP_TOUCH_MOVED, n, x, y, dx, dy, now);
                s->px[n] = x;
                s->py[n] = y;
                s->move_t[n] = now;
            }

            if (drv && (s->gest_mask & SENSE_CAP_GESTURE_PRESS_HOLD) &&
                !((s->hold_fired >> n) & 1U) && !((s->moved_far >> n) & 1U) &&
                now - s->down_t[n] >= hold_total) {
                s->hold_fired |= (uint8_t)(1U << n);
                ev_raise(s, SENSE_CAP_EV_LONG_PRESS);
                if (s->on_hold)
                    s->on_hold(tile, x, y, s->hold_ctx);
            }
        } else if (!present && was) {
            s->was_down &= (uint8_t)~(1U << n);
            s->last_up_t = now;
            s->up_seen = 1;
            ev_raise(s, SENSE_CAP_EV_TOUCH_UP);
            touch_emit(tile, s, SENSE_CAP_TOUCH_UP, n, s->px[n], s->py[n], 0, 0, now);

            if (!drv) continue;

            /* Fling: a drag released at speed is a swipe, dominant axis. */
            uint16_t travel = (uint16_t)(iabs16((int16_t)(s->px[n] - s->down_x[n])) +
                                         iabs16((int16_t)(s->py[n] - s->down_y[n])));
            int16_t avx = iabs16(s->vx), avy = iabs16(s->vy);
            if (((s->moved_far >> n) & 1U) && travel >= swipe_dist &&
                ((uint16_t)avx > fling || (uint16_t)avy > fling)) {
                uint8_t dir;
                int16_t major = avx >= avy ? avx : avy;
                int16_t minor = avx >= avy ? avy : avx;
                if (avx >= avy) dir = s->vx > 0 ? SENSE_CAP_DIR_RIGHT : SENSE_CAP_DIR_LEFT;
                else            dir = s->vy > 0 ? SENSE_CAP_DIR_DOWN : SENSE_CAP_DIR_UP;
                if ((int32_t)minor * 64 <= (int32_t)angle * major &&
                    (s->gest_mask & dir_gesture_bit(dir)))
                    fire_swipe(tile, s, dir);
            }

            if ((s->gest_mask & SENSE_CAP_GESTURE_SINGLE_TAP) &&
                !((s->moved_far >> n) & 1U) && !((s->hold_fired >> n) & 1U) &&
                now - s->down_t[n] <= s->tap_ms)
                fire_tap(tile, s, s->px[n], s->py[n], now);
        }
    }

    /* Chip gesture engine (CHIP mode only; switched off on the chip in
     * DRIVER mode, so nothing fires twice). Newly set bits only, and only
     * around a touch our tracker saw: noise wander can raise them. */
    if (!drv) {
        uint16_t gest = (uint16_t)(s->block[BLK_GESTURES] & s->gest_mask);
        uint16_t fresh = (uint16_t)(gest & (uint16_t)~s->prev_gestures);
        s->prev_gestures = gest;
        if (!s->was_down && (!s->up_seen || now - s->last_up_t > TOUCH_CHIP_GATE_MS))
            fresh = 0;
        /* §8.1: a tap reports after release, located in finger 1's XY. */
        uint16_t gx = s->block[BLK_FINGER_BASE + 0], gy = s->block[BLK_FINGER_BASE + 1];
        if (fresh & SENSE_CAP_GESTURE_SINGLE_TAP)
            fire_tap(tile, s, gx, gy, now);
        if (fresh & SENSE_CAP_GESTURE_PRESS_HOLD) {
            ev_raise(s, SENSE_CAP_EV_LONG_PRESS);
            if (s->on_hold)
                s->on_hold(tile, gx, gy, s->hold_ctx);
        }
        if (fresh & SENSE_CAP_GESTURE_SWIPE_X_NEG) fire_swipe(tile, s, SENSE_CAP_DIR_LEFT);
        if (fresh & SENSE_CAP_GESTURE_SWIPE_X_POS) fire_swipe(tile, s, SENSE_CAP_DIR_RIGHT);
        if (fresh & SENSE_CAP_GESTURE_SWIPE_Y_NEG) fire_swipe(tile, s, SENSE_CAP_DIR_UP);
        if (fresh & SENSE_CAP_GESTURE_SWIPE_Y_POS) fire_swipe(tile, s, SENSE_CAP_DIR_DOWN);
    }

    /* Two-finger pinch from the spread distance (Manhattan, no sqrt). */
    uint16_t x0 = s->block[BLK_FINGER_BASE + 0], y0 = s->block[BLK_FINGER_BASE + 1];
    uint16_t x1 = s->block[BLK_FINGER_BASE + 4], y1 = s->block[BLK_FINGER_BASE + 5];
    if (fingers == 2 && x0 != XY_INVALID && x1 != XY_INVALID) {
        uint16_t dist = (uint16_t)(iabs16((int16_t)(x0 - x1)) + iabs16((int16_t)(y0 - y1)));
        if (s->pinch_valid) {
            int16_t delta = (int16_t)(dist - s->pinch_dist);
            if ((uint16_t)iabs16(delta) >= pinch_dead) {
                ev_raise(s, SENSE_CAP_EV_PINCH);
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
 * ATI machine (non-blocking)
 * ================================================================ */

static uint8_t ati_active(const iqs7211a_state_t *s)
{
    return (uint8_t)(s->ati.phase != A_IDLE && s->ati.phase != A_DONE);
}

static void ati_begin(iqs7211a_state_t *s, uint8_t targets, uint8_t standalone)
{
    s->ati.targets = targets;
    s->ati.round = 0;
    s->ati.sub = 0;
    s->ati.req_sent = 0;
    s->ati.standalone = standalone;
    s->ati.result = SENSE_CAP_BUSY;
    s->ati.phase = (targets & SENSE_CAP_ATI_TRACKPAD) ? A_TP_CMD : A_ALP_ENTER;
    s->wr_on = 0;
    s->miss_on = 0;
    s->vw.stage = 0;
    s->vw.tries = 0;
}

/**
 * One non-blocking completion check: at most one request or one read.
 * Returns 1 when System Control was read and `bit` has cleared (the chip
 * clears the re-ATI bits when the algorithm completes, §5.6.3).
 */
static uint8_t ati_check_done(tile_t *tile, iqs7211a_state_t *s, uint16_t bit)
{
    uint32_t now = now_ms(s);
    uint8_t buf[2];

    if (!s->ati.req_sent) {
        if ((int32_t)(now - s->ati.t_next) < 0) return 0;
        s->ati.req_sent = 1;
        s->ati.t_req = now;
        if (s->cr_mode) {
            comms_request(tile, s);
            return 0;
        }
    }
    uint8_t r = xf_read(tile, IQS7211A_REG_SYSTEM_CONTROL, buf, 2);
    if (r != XF_OK) {
        if (r == XF_BUSY) s->cr_mode = 1;
        uint32_t to = s->chip_i2c_to ? s->chip_i2c_to : win_budget(s);
        if (now - s->ati.t_req > win_budget(s) + to) {
            s->ati.req_sent = 0;            /* window missed: ask again */
            s->req_pending = 0;
            s->ati.t_next = now;
        }
        return 0;
    }
    s->req_pending = 0;
    s->ati.req_sent = 0;
    s->ati.t_next = now + ATI_POLL_MS;
    uint16_t ctrl = (uint16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    return (ctrl & bit) ? 0 : 1;
}

/** Every sensed channel within 25% of the ATI target (the v0.x verdict,
 *  hardware-proven: the error flag rises transiently between rounds). */
static uint8_t tp_counts_in_range(const iqs7211a_state_t *s, const uint16_t *counts)
{
    uint16_t target = s->ati.target;
    if (!target || s->n_ch == 0 || s->n_ch > SENSE_CAP_MAX_CHANNELS) return 0;
    for (uint8_t ch = 0; ch < s->n_ch; ch++) {
        if (!(s->ch_alloc & (1U << ch))) continue;
        if (counts[ch] < target - target / 4U || counts[ch] > target + target / 4U)
            return 0;
    }
    return 1;
}

static sense_cap_result_t ati_finish(iqs7211a_state_t *s, sense_cap_result_t r)
{
    s->ati.result = r;
    s->ati.phase = A_DONE;
    return r;
}

/** A trackpad verdict round failed: queue another, or give up. */
static sense_cap_result_t ati_tp_retry(iqs7211a_state_t *s)
{
    if (++s->ati.round >= ATI_TP_ROUNDS) return ati_finish(s, SENSE_CAP_ERR_ATI);
    s->ati.phase = A_TP_CMD;
    return SENSE_CAP_BUSY;
}

/** One ATI step: at most one register transaction (see the file header). */
static sense_cap_result_t ati_step(tile_t *tile, iqs7211a_state_t *s)
{
    uint32_t now = now_ms(s);
    uint16_t info;
    sense_cap_result_t v;

    switch (s->ati.phase) {
    case A_IDLE:
        return SENSE_CAP_OK;
    case A_DONE:
        return s->ati.result;

    /* ---- trackpad ---- */
    case A_TP_CMD:
        if (!cmd_slice(tile, s, IQS7211A_CTRL_TP_RE_ATI)) return SENSE_CAP_BUSY;
        s->ati.t_cmd = now_ms(s);
        s->ati.phase = A_TP_QUIET;
        return SENSE_CAP_BUSY;
    case A_TP_QUIET:
    case A_ALP_QUIET:
        if (now - s->ati.t_cmd < ATI_QUIET_MS) return SENSE_CAP_BUSY;
        s->ati.req_sent = 0;
        s->ati.t_next = now;
        s->ati.phase = (s->ati.phase == A_TP_QUIET) ? A_TP_POLL : A_ALP_POLL;
        return SENSE_CAP_BUSY;
    case A_TP_POLL:
        if (ati_check_done(tile, s, IQS7211A_CTRL_TP_RE_ATI)) {
            s->ati.phase = A_TP_VERDICT;
        } else if (now - s->ati.t_cmd > ATI_ROUND_TIMEOUT_MS) {
            if (++s->ati.round >= ATI_TP_ROUNDS) return ati_finish(s, SENSE_CAP_ERR_TIMEOUT);
            s->ati.phase = A_TP_CMD;
        }
        return SENSE_CAP_BUSY;
    case A_TP_VERDICT:
        if (!rd_words(tile, s, IQS7211A_REG_INFO_FLAGS, &info, 1)) {
            if (!miss_expired(s)) return SENSE_CAP_BUSY;
            return ati_finish(s, SENSE_CAP_ERR_COMMS);
        }
        s->block[BLK_INFO_FLAGS] = info;
        /* A stale flag (ack never took) must not abort the ATI. */
        if ((info & IQS7211A_INFO_SHOW_RESET) && s->ack_verified)
            return ati_finish(s, SENSE_CAP_ERR_RESET);
        s->ati.target = s->srf.tuning.ati_target;
        s->ati.phase = A_TP_COUNTS;
        return SENSE_CAP_BUSY;
    case A_TP_COUNTS: {
        /* The target first if the surface left it to the chip, then every
         * sensed channel's count: one read per call. */
        uint16_t counts[SENSE_CAP_MAX_CHANNELS];
        uint8_t got;
        if (!s->ati.target) {
            got = rd_words(tile, s, IQS7211A_REG_TP_ATI_TARGET, &s->ati.target, 1);
            if (got && s->ati.target) return SENSE_CAP_BUSY;
        } else if (s->n_ch == 0 || s->n_ch > SENSE_CAP_MAX_CHANNELS) {
            got = 1;
        } else {
            got = rd_words(tile, s, IQS7211A_REG_EXT_COUNTS, counts, s->n_ch);
            if (got && tp_counts_in_range(s, counts)) {
                s->ati.round = 0;
                if (!(s->ati.targets & SENSE_CAP_ATI_ALP)) return ati_finish(s, SENSE_CAP_OK);
                s->ati.phase = A_ALP_ENTER;
                s->ati.sub = 0;
                return SENSE_CAP_BUSY;
            }
        }
        if (!got && !miss_expired(s)) return SENSE_CAP_BUSY;
        return ati_tp_retry(s);                 /* out of range or unreadable */
    }

    /* ---- ALP: only sensed in LP1/LP2 (§5.6.3), so hold LP1 under manual
     *      control for the duration (§6.3) ---- */
    case A_ALP_ENTER:
        s->ati_manual = 1;
        s->mode_sel = SENSE_CAP_MODE_LP1;
        v = vw_step(tile, s, s->ati.sub == 0 ? VW_CONFIG : VW_MODE);
        if (v == SENSE_CAP_BUSY) return SENSE_CAP_BUSY;
        if (v != SENSE_CAP_OK) {
            s->ati_manual = 0;
            s->ati.phase = A_ALP_EXIT;
            s->ati.sub = 0;
            s->ati.result = SENSE_CAP_ERR_VERIFY;
            return SENSE_CAP_BUSY;
        }
        if (s->ati.sub++ == 0) return SENSE_CAP_BUSY;   /* config done: mode next */
        s->ati.phase = A_ALP_CMD;
        return SENSE_CAP_BUSY;
    case A_ALP_CMD:
        /* Unverifiable (the bit clears when done), so give the write the
         * slowest mode's budget; the chip is cycling at the LP1 rate. */
        s->boost = 1;
        if (!cmd_slice(tile, s, IQS7211A_CTRL_ALP_RE_ATI))   /* mode LP1 + ALP re-ATI */
            return SENSE_CAP_BUSY;
        s->boost = 0;
        s->ati.t_cmd = now_ms(s);
        s->ati.phase = A_ALP_QUIET;
        return SENSE_CAP_BUSY;
    case A_ALP_POLL:
        if (ati_check_done(tile, s, IQS7211A_CTRL_ALP_RE_ATI)) {
            s->ati.phase = A_ALP_VERDICT;
        } else if (now - s->ati.t_cmd > ATI_ROUND_TIMEOUT_MS) {
            s->ati.result = SENSE_CAP_ERR_TIMEOUT;
            s->ati.phase = A_ALP_EXIT;
        }
        return SENSE_CAP_BUSY;
    case A_ALP_VERDICT:
        if (!rd_words(tile, s, IQS7211A_REG_INFO_FLAGS, &info, 1)) {
            if (!miss_expired(s)) return SENSE_CAP_BUSY;
            s->ati.result = SENSE_CAP_ERR_COMMS;
        } else if ((info & IQS7211A_INFO_SHOW_RESET) && s->ack_verified) {
            return ati_finish(s, SENSE_CAP_ERR_RESET);
        } else if (info & IQS7211A_INFO_ALP_ATI_ERROR) {
            if (++s->ati.round < ATI_ALP_ROUNDS) {
                s->ati.phase = A_ALP_CMD;
                return SENSE_CAP_BUSY;
            }
            s->ati.result = SENSE_CAP_ERR_ATI;
        } else {
            s->ati.result = SENSE_CAP_OK;
        }
        s->ati.phase = A_ALP_EXIT;
        s->ati.sub = 0;
        return SENSE_CAP_BUSY;
    case A_ALP_EXIT:
        /* Mode select back first (it only applies under manual control),
         * then hand mode switching back unless the host forces a mode. */
        if (s->ati.sub == 0) {
            switch (s->power_mode) {
            case SENSE_CAP_POWER_FORCE_LP1: s->mode_sel = SENSE_CAP_MODE_LP1; break;
            case SENSE_CAP_POWER_FORCE_LP2: s->mode_sel = SENSE_CAP_MODE_LP2; break;
            default:                        s->mode_sel = SENSE_CAP_MODE_ACTIVE; break;
            }
            v = vw_step(tile, s, VW_MODE);
            if (v == SENSE_CAP_BUSY) return SENSE_CAP_BUSY;
            if (v != SENSE_CAP_OK && s->ati.result == SENSE_CAP_OK)
                s->ati.result = SENSE_CAP_ERR_VERIFY;
            s->ati_manual = 0;
            s->ati.sub = 1;
            return SENSE_CAP_BUSY;
        }
        v = vw_step(tile, s, VW_CONFIG);
        if (v == SENSE_CAP_BUSY) return SENSE_CAP_BUSY;
        if (v != SENSE_CAP_OK && s->ati.result == SENSE_CAP_OK)
            s->ati.result = SENSE_CAP_ERR_VERIFY;
        if (s->power_mode == SENSE_CAP_POWER_AUTO) s->mode_sel = 0;
        return ati_finish(s, s->ati.result);
    default:
        return ati_finish(s, SENSE_CAP_ERR_STATE);
    }
}

/* ================================================================
 * Apply job (baseline + surface), bounded steps
 * ================================================================ */

static uint8_t job_active(const iqs7211a_state_t *s)
{
    return (uint8_t)(s->job.phase != J_IDLE && s->job.phase != J_DONE &&
                     s->job.phase != J_FAILED);
}

/** Where a baseline / re-apply job starts: the whole register image, or
 *  only the reset ack for a GUI-programmed part without a surface. */
static uint8_t first_phase(const iqs7211a_state_t *s)
{
    return (s->skip_baseline && !s->has_srf) ? J_ACK : J_GEO;
}

/** Start a job at `phase`. Until the comms mode and report rates are known
 *  (after init or a chip reset) J_PROBE runs first, then `phase`. */
static void job_start(iqs7211a_state_t *s, uint8_t phase)
{
    s->job.next = phase;
    s->job.phase = s->probed ? phase : J_PROBE;
    s->job.blk = 0;
    s->job.sub = 0;
    s->job.idx = 0;
    s->job.vrounds = 0;
    s->job.tries = 0;
    s->job.restarts = 0;
    s->job.dirty = 0;
    s->job.ati_done = 0;
    s->job.result = SENSE_CAP_BUSY;
    s->srf_tuned = 0;
    s->base_ok = 0;
    s->wr_on = 0;
    s->miss_on = 0;
    s->vw.stage = 0;
    s->vw.tries = 0;
    if (s->ati.phase != A_IDLE && !s->ati.standalone) s->ati.phase = A_IDLE;
}

/** The job cannot continue: stop with `err`. */
static sense_cap_result_t job_abort(tile_t *tile, iqs7211a_state_t *s, sense_cap_result_t err)
{
    s->job.phase = J_FAILED;
    s->job.result = err;
    s->wr_on = 0;
    s->miss_on = 0;
    TILE_ON_ERROR(tile, "sense_cap: surface/baseline apply failed");
    return err;
}

/** A job step failed. Unless it was the ATI, first ask whether the chip
 *  reset under us (J_RSTCHK, one read per call) and start over if so. */
static sense_cap_result_t job_fail(tile_t *tile, iqs7211a_state_t *s, sense_cap_result_t err)
{
    if (err == SENSE_CAP_ERR_ATI || s->job.restarts >= JOB_RESTARTS_MAX)
        return job_abort(tile, s, err);
    s->job.pend_err = err;
    s->job.phase = J_RSTCHK;
    s->job.sub = 0;
    s->wr_on = 0;
    s->miss_on = 0;
    return SENSE_CAP_BUSY;
}

/** A job read found no window. Returns 1 when the miss has become a
 *  failed try (and sets *err once the tries are used up). */
static uint8_t job_read_missed(iqs7211a_state_t *s, sense_cap_result_t *err)
{
    if (!miss_expired(s)) return 0;       /* no window yet: read again next call */
    s->boost = 1;
    if (++s->job.tries >= JOB_RETRIES) *err = SENSE_CAP_ERR_COMMS;
    return 1;
}

/**
 * Sync the current block of the current phase: read it, write the
 * registers that differ one at a time (one transaction per call), read
 * back, repeat. Returns 1 when every block of the phase is done, 0 to
 * continue, and sets *err on failure.
 */
static uint8_t job_block(tile_t *tile, iqs7211a_state_t *s, const uint8_t *regs,
                         const uint8_t *ns, uint8_t count, uint8_t clear,
                         uint8_t *ops, sense_cap_result_t *err)
{
    uint8_t reg = regs[s->job.blk], n = ns[s->job.blk];

    if (s->job.sub == 0) {
        (*ops)++;
        if (!rd_words(tile, s, reg, s->job.cur, n)) {
            (void)job_read_missed(s, err);
            return 0;
        }
        s->job.tries = 0;
        uint8_t diff = 0;
        for (uint8_t i = 0; i < n; i++) {
            uint16_t w;
            if (want_reg(s, (uint8_t)(reg + i), s->job.cur[i], clear, &w) && w != s->job.cur[i])
                diff = 1;
        }
        if (!diff) {
            s->boost = 0;
            s->job.vrounds = 0;
            if (++s->job.blk >= count) return 1;
            return 0;
        }
        if (s->job.vrounds >= JOB_RETRIES) {
            *err = SENSE_CAP_ERR_VERIFY;
            return 0;
        }
        if (s->job.vrounds) s->boost = 1;
        s->job.vrounds++;
        s->job.sub = 1;
        s->job.idx = 0;
        return 0;
    }

    while (s->job.idx < n && *ops < JOB_OPS_PER_STEP) {
        uint16_t w;
        uint8_t r = (uint8_t)(reg + s->job.idx);
        if (want_reg(s, r, s->job.cur[s->job.idx], clear, &w) && w != s->job.cur[s->job.idx]) {
            (*ops)++;
            if (!wr_slice(tile, s, r, w)) return 0;   /* the same write next call */
        }
        s->job.idx++;
    }
    if (s->job.idx >= n) s->job.sub = 0;   /* read back next */
    return 0;
}

static void job_next_phase(iqs7211a_state_t *s, uint8_t phase)
{
    s->job.phase = phase;
    s->job.blk = 0;
    s->job.sub = 0;
    s->job.vrounds = 0;
    s->job.tries = 0;
}

/** One job step: at most JOB_OPS_PER_STEP (one) register transaction. */
static sense_cap_result_t job_step_body(tile_t *tile, iqs7211a_state_t *s)
{
    uint8_t ops = 0;

    while (ops < JOB_OPS_PER_STEP) {
        sense_cap_result_t err = SENSE_CAP_OK;
        sense_cap_result_t v;

        switch (s->job.phase) {
        case J_IDLE:
            return SENSE_CAP_OK;
        case J_DONE:
        case J_FAILED:
            return s->job.result;

        case J_PROBE: {
            /* Before any write, learn how the chip talks (Config Settings)
             * and how fast it cycles (0x40-0x4A): one read per call. */
            uint16_t w[11];
            ops++;
            if (!rd_words(tile, s, s->job.sub ? IQS7211A_REG_RATE_ACTIVE : IQS7211A_REG_CONFIG_SETTINGS,
                          w, s->job.sub ? 11U : 1U)) {
                (void)job_read_missed(s, &err);
                break;
            }
            s->job.tries = 0;
            if (s->job.sub == 0) {
                s->cr_mode = (w[0] & IQS7211A_CFG_COMMS_REQUEST_EN) ? 1 : 0;
                if (s->learn_cfg) {
                    s->config = w[0];
                    if (s->skip_baseline) s->wdt = (w[0] & IQS7211A_CFG_WDT_EN) ? 1 : 0;
                    s->learn_cfg = 0;
                }
                s->job.sub = 1;
                break;
            }
            s->boost = 0;
            s->probed = 1;                  /* rd_words() learned the rates */
            job_next_phase(s, s->job.next);
            break;
        }

        case J_GEO: {
            /* Geometry unchanged? Then the cycle table can be synced in
             * place; otherwise empty it first, so no cycle ever names a
             * channel outside the rectangle while sizes and map change.
             * Sizes (0x60-0x61) into cur[0..1], then the map into cur[2..7]. */
            uint16_t *sz = &s->job.cur[0], *map = &s->job.cur[2], w = 0;
            uint8_t same = 1;
            ops++;
            if (!rd_words(tile, s, s->job.sub ? IQS7211A_REG_RXTX_MAP_BASE : IQS7211A_REG_TP_SETTINGS,
                          s->job.sub ? map : sz, s->job.sub ? 6U : 2U)) {
                (void)job_read_missed(s, &err);
                break;
            }
            s->job.tries = 0;
            if (s->job.sub == 0) {
                s->job.sub = 1;
                break;
            }
            want_reg(s, IQS7211A_REG_TP_SETTINGS, sz[0], 0, &w);
            if ((w & 0xFF00U) != (sz[0] & 0xFF00U)) same = 0;
            want_reg(s, IQS7211A_REG_TP_TOUCHES, sz[1], 0, &w);
            if ((w & 0x00FFU) != (sz[1] & 0x00FFU)) same = 0;
            for (uint8_t i = 0; i < 6; i++) {
                want_reg(s, (uint8_t)(IQS7211A_REG_RXTX_MAP_BASE + i), map[i], 0, &w);
                if (w != map[i]) same = 0;
            }
            s->job.clear = same ? 0 : 1;
            job_next_phase(s, J_RATES);
            break;
        }
        case J_RATES:
            if (job_block(tile, s, MAIN_BLK_REG, MAIN_BLK_N, 1, 0, &ops, &err))
                job_next_phase(s, s->job.clear ? J_CLEAR : J_BLOCKS);
            break;
        case J_CLEAR:
            if (job_block(tile, s, CYC_BLK_REG, CYC_BLK_N, CYC_BLKS, 1, &ops, &err))
                job_next_phase(s, J_BLOCKS);
            break;
        case J_BLOCKS:
            if (s->job.blk == 0) s->job.blk = 1;   /* rates already synced */
            if (job_block(tile, s, MAIN_BLK_REG, MAIN_BLK_N, (uint8_t)MAIN_BLKS, 0, &ops, &err))
                job_next_phase(s, J_CYCLES);
            break;
        case J_CYCLES:
            if (job_block(tile, s, CYC_BLK_REG, CYC_BLK_N, CYC_BLKS, 0, &ops, &err))
                job_next_phase(s, J_CONFIG);
            break;

        case J_CONFIG:
            /* Last: switching Comms Request on changes how every later
             * access is made. Mode select follows under manual control. */
            ops++;
            v = vw_step(tile, s, s->job.sub == 0 ? VW_CONFIG : VW_MODE);
            if (v == SENSE_CAP_BUSY) break;
            if (v != SENSE_CAP_OK) {
                s->job.sub = 0;
                if (++s->job.tries >= JOB_RETRIES) return job_fail(tile, s, SENSE_CAP_ERR_VERIFY);
                break;
            }
            if (s->job.sub == 0 && (s->config & IQS7211A_CFG_MANUAL_CONTROL)) {
                s->job.sub = 1;
                break;
            }
            job_next_phase(s, J_ACK);
            break;

        case J_ACK:
            if (s->need_ack) {
                uint16_t info;
                ops++;
                if (s->job.sub == 0) {                 /* the ack, then the read-back */
                    if (cmd_slice(tile, s, IQS7211A_CTRL_ACK_RESET)) s->job.sub = 1;
                    break;
                }
                if (rd_words(tile, s, IQS7211A_REG_INFO_FLAGS, &info, 1)) {
                    s->block[BLK_INFO_FLAGS] = (uint16_t)(info & (uint16_t)~IQS7211A_INFO_SHOW_RESET);
                    if (!(info & IQS7211A_INFO_SHOW_RESET)) {
                        s->ack_verified = 1;
                        s->need_ack = 0;
                    }
                } else if (!miss_expired(s)) {
                    break;
                }
                s->job.sub = 0;
                if (s->need_ack && ++s->job.tries >= JOB_RETRIES) {
                    /* Not fatal: reset detection falls back to checking
                     * whether our Config Settings survived. */
                    s->need_ack = 0;
                    TILE_ON_ERROR(tile, "sense_cap: Show Reset would not clear");
                }
                if (s->need_ack) break;
            }
            s->job.tries = 0;
            s->job.sub = 0;
            if (s->job.dirty) {             /* a setter changed the image meanwhile */
                s->job.dirty = 0;
                job_next_phase(s, J_RATES);
                s->job.clear = 0;
                break;
            }
            s->base_ok = 1;
            s->srf_applied = s->has_srf;
            if (s->has_srf && !s->job.ati_done) {
                ati_begin(s, (uint8_t)(SENSE_CAP_ATI_TRACKPAD |
                                       (alp_in_use(s) ? SENSE_CAP_ATI_ALP : 0U)), 0);
                s->job.phase = J_ATI;
                break;
            }
            s->job.phase = J_DONE;
            s->job.result = (s->has_srf && !s->srf_tuned) ? SENSE_CAP_ERR_ATI : SENSE_CAP_OK;
            return s->job.result;

        case J_ATI: {
            sense_cap_result_t r = ati_step(tile, s);
            if (r == SENSE_CAP_BUSY) return SENSE_CAP_BUSY;
            s->ati.phase = A_IDLE;
            if (r == SENSE_CAP_ERR_RESET) return job_fail(tile, s, SENSE_CAP_ERR_COMMS);
            s->srf_tuned = (r == SENSE_CAP_OK) ? 1 : 0;
            s->job.ati_done = 1;
            if (s->job.dirty) {
                s->job.dirty = 0;
                job_next_phase(s, J_RATES);
                s->job.clear = 0;
                return SENSE_CAP_BUSY;
            }
            s->job.result = r;
            s->job.phase = (r == SENSE_CAP_OK || r == SENSE_CAP_ERR_ATI) ? J_DONE : J_FAILED;
            if (r != SENSE_CAP_OK) TILE_ON_ERROR(tile, "sense_cap: ATI did not converge");
            return r;
        }

        case J_RSTCHK: {
            /* After a failure: did the chip reset under us? Show Reset
             * first, then whether our Config Settings survived (this also
             * re-learns the comms mode). */
            uint16_t w;
            ops++;
            if (!rd_words(tile, s, s->job.sub ? IQS7211A_REG_CONFIG_SETTINGS : IQS7211A_REG_INFO_FLAGS,
                          &w, 1)) {
                if (!miss_expired(s)) break;
                return job_abort(tile, s, s->job.pend_err);
            }
            if (s->job.sub == 0) {
                if (!(w & IQS7211A_INFO_SHOW_RESET)) return job_abort(tile, s, s->job.pend_err);
                s->job.sub = 1;
                break;
            }
            s->cr_mode = (w & IQS7211A_CFG_COMMS_REQUEST_EN) ? 1 : 0;
            if (!s->ack_verified && w == s->config) return job_abort(tile, s, s->job.pend_err);
            uint8_t restarts = (uint8_t)(s->job.restarts + 1U);
            s->need_ack = 1;
            s->ack_verified = 0;
            s->rates_known = 0;
            s->probed = 0;
            job_start(s, J_GEO);
            s->job.restarts = restarts;
            return SENSE_CAP_BUSY;
        }
        default:
            return job_abort(tile, s, SENSE_CAP_ERR_STATE);
        }

        if (err != SENSE_CAP_OK)
            return job_fail(tile, s, err);
    }
    return SENSE_CAP_BUSY;
}

/** job_step_body() with every window wait capped (see the file header). */
static sense_cap_result_t job_step(tile_t *tile, iqs7211a_state_t *s)
{
    s->step = 1;
    sense_cap_result_t r = job_step_body(tile, s);
    s->step = 0;
    return r;
}

/** Run a job to completion (blocking wrapper), bounded by real time. */
static sense_cap_result_t job_run(tile_t *tile, iqs7211a_state_t *s, uint32_t timeout_ms)
{
    uint32_t t0 = now_ms(s);
    sense_cap_result_t r;
    while ((r = job_step(tile, s)) == SENSE_CAP_BUSY) {
        if (now_ms(s) - t0 > timeout_ms) {
            s->job.phase = J_FAILED;
            s->job.result = SENSE_CAP_ERR_TIMEOUT;
            s->ati.phase = A_IDLE;
            s->wr_on = 0;
            return SENSE_CAP_ERR_TIMEOUT;
        }
        sleep_ms(tile, s, 2);
    }
    return r;
}

/** Start re-applying after a chip reset (the chip is back at defaults, so
 *  its comms mode and rates are learned again first). */
static void start_reapply(iqs7211a_state_t *s)
{
    s->need_ack = 1;
    s->ack_verified = 0;
    s->srf_applied = 0;
    s->srf_tuned = 0;
    s->mode_sel = 0;
    s->ati_manual = 0;
    s->ati.phase = A_IDLE;
    s->probed = 0;
    s->learn_cfg = 0;
    job_start(s, first_phase(s));
}

/* ================================================================
 * Surface validation and presets
 * ================================================================ */

static uint8_t base_ok(uint8_t fine, uint8_t mult, uint8_t div)
{
    if (!(fine | mult | div)) return 1;   /* keep */
    return (uint8_t)(fine >= 1 && fine <= 31 && mult >= 1 && mult <= 15 &&
                     div >= 1 && div <= 31);
}

/** Tile rules (the PCB) and datasheet ranges. Produces the ALP masks. */
static sense_cap_result_t validate_surface(const sense_cap_surface_t *f,
                                           uint8_t *alp_rx, uint16_t *alp_tx)
{
    const sense_cap_geometry_t *g = &f->geometry;
    const sense_cap_tuning_t *t = &f->tuning;
    const sense_cap_alp_t *a = &f->alp;
    uint16_t used = 0, rxm = 0, txm = 0;

    if (g->n_rx < 1 || g->n_rx > SENSE_CAP_MAX_RX) return SENSE_CAP_ERR_GEOMETRY;
    if (g->n_tx < 1 || g->n_tx > SENSE_CAP_MAX_TX) return SENSE_CAP_ERR_GEOMETRY;
    for (uint8_t i = 0; i < g->n_rx + g->n_tx; i++) {
        uint8_t is_rx = (uint8_t)(i < g->n_rx);
        uint8_t p = is_rx ? g->rx[i] : g->tx[i - g->n_rx];
        if (p == 0 || p == 10) return SENSE_CAP_ERR_FORBIDDEN;   /* RX0/TX0 grounded; TX10 merged */
        if (p > 11) return SENSE_CAP_ERR_PIN;
        uint16_t bit = (uint16_t)(1U << p);
        if (!(bit & (is_rx ? SENSE_CAP_ROUTED_RX_MASK : SENSE_CAP_ROUTED_TX_MASK)))
            return SENSE_CAP_ERR_PIN;
        if (used & bit) return SENSE_CAP_ERR_GEOMETRY;          /* one role per pin */
        used |= bit;
        if (is_rx) rxm |= bit;
        else       txm |= bit;
    }
    if (g->n_rx * g->n_tx > SENSE_CAP_MAX_CHANNELS) return SENSE_CAP_ERR_GEOMETRY;
    if (g->disabled_channels >> (g->n_rx * g->n_tx)) return SENSE_CAP_ERR_ARG;
    if (g->switch_xy > 1 || g->flip_x > 1 || g->flip_y > 1) return SENSE_CAP_ERR_ARG;

    if (!base_ok(t->ati_fine_div, t->ati_coarse_mult, t->ati_coarse_div)) return SENSE_CAP_ERR_ARG;
    if (t->ati_target > 16383) return SENSE_CAP_ERR_ARG;          /* Max Count ceiling, Table A.10 */
    if (t->touch_set_mult < 1 || t->touch_clear_mult > t->touch_set_mult) return SENSE_CAP_ERR_ARG;
    if (t->filters & (uint8_t)~0x38U) return SENSE_CAP_ERR_ARG;
    if (t->max_touches > SENSE_CAP_NUM_FINGERS) return SENSE_CAP_ERR_ARG;
    if (t->reati_retry_s > 60 || t->ref_update_s > 60) return SENSE_CAP_ERR_ARG;   /* §5.7.3, §5.4.1 */

    if (!base_ok(a->ati_fine_div, a->ati_coarse_mult, a->ati_coarse_div)) return SENSE_CAP_ERR_ARG;
    if (a->ati_target > 16383) return SENSE_CAP_ERR_ARG;
    if (a->lp1_auto_prox > SENSE_CAP_AUTOPROX_OFF || a->lp2_auto_prox > SENSE_CAP_AUTOPROX_OFF)
        return SENSE_CAP_ERR_ARG;

    uint16_t arx = a->rx_mask ? a->rx_mask : rxm;
    uint16_t atx = a->mutual ? (a->tx_mask ? a->tx_mask : txm) : 0U;
    if ((arx & 1U) || (atx & ((1U << 0) | (1U << 10)))) return SENSE_CAP_ERR_FORBIDDEN;
    if ((arx & (uint16_t)~SENSE_CAP_ROUTED_RX_MASK) || (atx & (uint16_t)~SENSE_CAP_ROUTED_TX_MASK))
        return SENSE_CAP_ERR_PIN;
    if ((arx & atx) || arx == 0) return SENSE_CAP_ERR_GEOMETRY;

    *alp_rx = (uint8_t)arx;
    *alp_tx = atx;
    return SENSE_CAP_OK;
}

sense_cap_result_t tile_sense_cap_layout_preset(sense_cap_layout_t layout,
                                                sense_cap_surface_t *out)
{
    static const sense_cap_surface_t grid = SENSE_CAP_SURFACE_DEFAULTS;
    if (!out) return SENSE_CAP_ERR_ARG;
    *out = grid;
    switch (layout) {
    case SENSE_CAP_LAYOUT_NONE:
        /* Routed geometry, nothing allocated: no conversions at all. */
        out->geometry.disabled_channels = 0x3F;
        out->tuning.ati_target = 0;
        out->tuning.ati_fine_div = out->tuning.ati_coarse_mult = out->tuning.ati_coarse_div = 0;
        out->alp.enable = 0;
        out->alp.ati_target = 0;
        return SENSE_CAP_OK;
    case SENSE_CAP_LAYOUT_GRID_2X3:
    case SENSE_CAP_LAYOUT_BUTTONS_2X3:
        return SENSE_CAP_OK;
    case SENSE_CAP_LAYOUT_SLIDER_1X3:
        out->geometry.n_rx = 1;
        out->geometry.rx[1] = 0;
        out->tuning.max_touches = 1;
        return SENSE_CAP_OK;
    case SENSE_CAP_LAYOUT_SLIDER_1X4:
        out->geometry.n_rx = 1;
        out->geometry.rx[1] = 0;
        out->geometry.n_tx = 4;
        out->geometry.tx[3] = SENSE_CAP_PIN_RX6;   /* RX6/TX6 pin driven as a Tx */
        out->tuning.max_touches = 1;
        return SENSE_CAP_OK;
    default:
        return SENSE_CAP_ERR_ARG;
    }
}

/** Scale the surface's multipliers by the sensitivity level, keeping the
 *  set/clear ratio (rounded), set >= 1, clear <= set. */
static void apply_sensitivity(iqs7211a_state_t *s)
{
    static const uint8_t num[5] = { 2, 3, 1, 3, 1 };
    static const uint8_t den[5] = { 1, 2, 1, 4, 2 };
    uint8_t l = s->sens <= SENSE_CAP_SENS_HIGHEST ? s->sens : SENSE_CAP_SENS_NORMAL;
    uint32_t set = ((uint32_t)s->srf.tuning.touch_set_mult * num[l] + den[l] / 2U) / den[l];
    uint32_t clr = ((uint32_t)s->srf.tuning.touch_clear_mult * num[l] + den[l] / 2U) / den[l];
    if (set < 1) set = 1;
    if (set > 255) set = 255;
    if (clr > set) clr = set;
    if (clr == set && set > 1 && s->srf.tuning.touch_clear_mult < s->srf.tuning.touch_set_mult)
        clr = set - 1;
    s->mult_set = (uint8_t)set;
    s->mult_clear = (uint8_t)clr;
}

/* ================================================================
 * Lifecycle
 * ================================================================ */

uint8_t tile_sense_cap_find(tiles_pal_t *hal, uint8_t instance)
{
    uint8_t addr = resolve_id(instance);
    uint8_t buf[2], seen_marker = 0, zero = 0;
    uint32_t waited = 0;
    if (!addr || !hal || !hal->i2c_read) return 0;

    /* A bare address probe goes unanswered outside a window (ring bench,
     * 2026-09), so read the product number, asking for a window whenever
     * the chip answers 0xEEEE or does not answer. */
    while (waited < IQS_FIND_TIMEOUT_MS) {
        buf[0] = buf[1] = 0xEE;
        if (hal->i2c_read(hal->handle, addr, IQS7211A_REG_PRODUCT_NUM, buf, 2) == 0) {
            uint16_t v = (uint16_t)(((uint16_t)buf[1] << 8) | buf[0]);
            if (v == IQS7211A_PRODUCT_NUMBER) return 1;
            if (v == IQS7211A_INVALID_RESPONSE) seen_marker = 1;
        }
        if (waited % 20U == 0U)
            (void)hal->i2c_write(hal->handle, addr, IQS7211A_REG_END_COMMS, &zero, 1);
        hal->delay_ms(2);
        waited += 2;
    }
    return seen_marker;
}

/**
 * Read the product number and firmware version (0x00-0x02) within
 * `timeout_ms` of real time, each attempt capped at step_cap(). Returns 1
 * if it is an IQS7211A (versions cached), 0 on silence or another part.
 */
static uint8_t identify(tile_t *tile, iqs7211a_state_t *s, uint32_t timeout_ms)
{
    uint16_t w[3];
    uint8_t found = 0;
    uint32_t t0 = now_ms(s);
    s->step = 1;
    for (;;) {
        if (rd_words(tile, s, IQS7211A_REG_PRODUCT_NUM, w, 3)) {
            found = (uint8_t)(w[0] == IQS7211A_PRODUCT_NUMBER);
            if (found) {
                s->ver_major = w[1];
                s->ver_minor = w[2];
            }
            break;
        }
        if (now_ms(s) - t0 >= timeout_ms) break;
    }
    s->step = 0;
    return found;
}

/** Wait for the chip after a reset (it NACKs while booting) and identify
 *  it. Its comms mode and rates are learned by the job's J_PROBE. */
static uint8_t wait_boot(tile_t *tile, iqs7211a_state_t *s)
{
    s->cr_mode = 0;             /* reset default: clock stretching */
    s->req_pending = 0;
    s->rates_known = 0;
    s->chip_i2c_to = 0;
    s->boost = 0;
    s->probed = 0;
    sleep_ms(tile, s, IQS_BOOT_MIN_MS);
    return identify(tile, s, IQS_BOOT_TIMEOUT_MS);
}

static void mclr_pulse(tile_t *tile, iqs7211a_state_t *s)
{
    s->mclr(s->mclr_ctx, 0);
    sleep_ms(tile, s, IQS_MCLR_LOW_MS);
    s->mclr(s->mclr_ctx, 1);
}

void tile_sense_cap_init(tiles_pal_t *hal, uint8_t instance,
                         tile_t *tile, const sense_cap_cfg_t *cfg)
{
    if (!tile) return;
    memzero(tile, sizeof(tile_t));
    tile->hal = hal;
    tile->id  = resolve_id(instance);
    if (!hal || !tile->id) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_cap: invalid instance");
        return;
    }

    iqs7211a_state_t *s = state_for(tile);
    uint8_t reset_first = 0;
    memzero(s, sizeof(iqs7211a_state_t));
    if (cfg) {
        s->on_event      = cfg->on_event;
        s->event_ctx     = cfg->event_ctx;
        s->millis        = cfg->millis;
        s->millis_ctx    = cfg->millis_ctx;
        s->mclr          = cfg->mclr;
        s->mclr_ctx      = cfg->mclr_ctx;
        s->skip_baseline = cfg->skip_baseline ? 1 : 0;
        reset_first      = (uint8_t)(cfg->reset_on_init && cfg->mclr);
    }

    /* Baseline settings: the power-on values documented for Studio. */
    s->profile = SENSE_CAP_POWER_RESPONSIVE;
    for (uint8_t i = 0; i < 5; i++) s->rate[i] = PROFILES[s->profile].rate[i];
    for (uint8_t i = 0; i < 4; i++) s->tmo[i]  = PROFILES[s->profile].tmo[i];
    s->power_mode      = SENSE_CAP_POWER_AUTO;
    s->wdt             = 1;
    s->sens            = SENSE_CAP_SENS_NORMAL;
    s->gest_src        = SENSE_CAP_GESTURES_DRIVER;
    s->gest_mask       = SENSE_CAP_GESTURE_ALL;
    s->i2c_timeout     = BASE_I2C_TIMEOUT_MS;
    s->tap_ms          = BASE_TAP_MS;
    s->hold_ms         = BASE_HOLD_MS;
    s->swipe_ms        = BASE_SWIPE_MS;
    s->swipe_angle_deg = BASE_SWIPE_ANGLE_DEG;
    s->layout          = SENSE_CAP_LAYOUT_NONE;
    (void)tile_sense_cap_layout_preset(SENSE_CAP_LAYOUT_NONE, &s->srf);
    (void)validate_surface(&s->srf, &s->alp_rx, &s->alp_tx);
    compute_cycles(s);
    apply_sensitivity(s);

    /* Identify. With cfg.reset_on_init, pulse MCLR first (a known chip
     * state at boot). Otherwise a chip that does not answer at all may be
     * wedged: one MCLR pulse if the host wired it. */
    uint8_t found;
    if (reset_first) {
        mclr_pulse(tile, s);
        found = wait_boot(tile, s);
    } else {
        found = identify(tile, s, IQS_FIND_TIMEOUT_MS);
        if (!found && s->mclr) {
            mclr_pulse(tile, s);
            found = wait_boot(tile, s);
        }
    }
    if (!found) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_cap: no IQS7211A answering at 0x56");
        return;
    }

    /* Ready for calls; the baseline itself is a job that surface_poll() /
     * process() advance, one register transaction per call: first the
     * comms mode and rates (J_PROBE), then the register image, the reset
     * ack and, with a layout, the ATI. */
    s->need_ack = 1;
    s->learn_cfg = 1;
    tile->state = TILE_STATE_READY;
    job_start(s, first_phase(s));

    if (cfg && cfg->layout != SENSE_CAP_LAYOUT_NONE)
        (void)tile_sense_cap_set_layout(tile, cfg->layout);
}

sense_cap_result_t tile_sense_cap_surface_begin(tile_t *tile,
                                                const sense_cap_surface_t *surf)
{
    uint8_t rx;
    uint16_t tx;
    if (!ready(tile)) return SENSE_CAP_ERR_STATE;
    if (!surf) return SENSE_CAP_ERR_ARG;
    iqs7211a_state_t *s = state_for(tile);
    if (ati_active(s) && s->ati.standalone) return SENSE_CAP_ERR_STATE;

    sense_cap_result_t r = validate_surface(surf, &rx, &tx);
    if (r != SENSE_CAP_OK) return r;

    s->srf = *surf;
    s->alp_rx = rx;
    s->alp_tx = tx;
    s->layout = 0xFF;   /* custom until set_layout says otherwise */
    compute_cycles(s);
    apply_sensitivity(s);
    s->srf_applied = 0;
    job_start(s, J_GEO);
    return SENSE_CAP_OK;
}

sense_cap_result_t tile_sense_cap_configure_surface(tile_t *tile,
                                                    const sense_cap_surface_t *surf)
{
    sense_cap_result_t r = tile_sense_cap_surface_begin(tile, surf);
    if (r != SENSE_CAP_OK) return r;
    return job_run(tile, state_for(tile), CONFIGURE_TIMEOUT_MS);
}

sense_cap_result_t tile_sense_cap_set_layout(tile_t *tile, sense_cap_layout_t layout)
{
    sense_cap_surface_t surf;
    sense_cap_result_t r = tile_sense_cap_layout_preset(layout, &surf);
    if (r != SENSE_CAP_OK) return r;
    r = tile_sense_cap_surface_begin(tile, &surf);
    if (r == SENSE_CAP_OK) state_for(tile)->layout = (uint8_t)layout;
    return r;
}

sense_cap_result_t tile_sense_cap_surface_poll(tile_t *tile)
{
    if (!ready(tile)) return SENSE_CAP_ERR_STATE;
    return job_step(tile, state_for(tile));
}

uint8_t tile_sense_cap_is_surface_ready(tile_t *tile)
{
    if (!tile) return 0;
    iqs7211a_state_t *s = state_for(tile);
    if (!ready(tile) || job_active(s) || ati_active(s) || !s->base_ok) return 0;
    return (uint8_t)(!s->has_srf || (s->srf_applied && s->srf_tuned));
}

sense_cap_result_t tile_sense_cap_ati_start(tile_t *tile, sense_cap_ati_target_t target)
{
    if (!ready(tile)) return SENSE_CAP_ERR_STATE;
    iqs7211a_state_t *s = state_for(tile);
    uint8_t t = (uint8_t)target;
    if (t < 1 || t > 3) return SENSE_CAP_ERR_ARG;
    if (job_active(s) || ati_active(s)) return SENSE_CAP_ERR_STATE;
    if (!s->srf.alp.enable) t &= (uint8_t)~SENSE_CAP_ATI_ALP;
    if ((t & SENSE_CAP_ATI_TRACKPAD) && !s->srf_applied) t &= (uint8_t)~SENSE_CAP_ATI_TRACKPAD;
    if (!t) return SENSE_CAP_ERR_STATE;
    ati_begin(s, t, 1);
    return SENSE_CAP_OK;
}

sense_cap_result_t tile_sense_cap_ati_poll(tile_t *tile)
{
    if (!ready(tile)) return SENSE_CAP_ERR_STATE;
    iqs7211a_state_t *s = state_for(tile);
    if (!s->ati.standalone || s->ati.phase == A_IDLE) return SENSE_CAP_OK;
    s->step = 1;
    sense_cap_result_t r = ati_step(tile, s);
    s->step = 0;
    if (r == SENSE_CAP_BUSY) return r;
    if (r == SENSE_CAP_ERR_RESET) {
        s->ati.phase = A_IDLE;
        touch_reset_all(tile, s, 1);
        start_reapply(s);
        return r;
    }
    if (s->ati.targets & SENSE_CAP_ATI_TRACKPAD)
        s->srf_tuned = (r == SENSE_CAP_OK) ? 1 : 0;
    return r;
}

/** Common tail of hw_reset() / reset(): wait for boot, confirm the reset
 *  took, drop cached state, start re-applying baseline + surface. */
static sense_cap_result_t after_reset(tile_t *tile, iqs7211a_state_t *s)
{
    uint16_t info = 0;
    s->job.phase = J_IDLE;
    s->ati.phase = A_IDLE;
    s->wr_on = 0;
    if (!wait_boot(tile, s)) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_cap: chip did not come back after reset");
        return SENSE_CAP_ERR_NOT_FOUND;
    }
    tile->state = TILE_STATE_READY;
    (void)rd_words(tile, s, IQS7211A_REG_INFO_FLAGS, &info, 1);
    s->ev_call = 0;
    touch_reset_all(tile, s, 1);
    ev_raise(s, SENSE_CAP_EV_RESET);
    if (s->on_event)
        s->on_event(tile, (uint8_t)(s->ev_call & 0xFFU), s->event_ctx);
    start_reapply(s);
    return (info & IQS7211A_INFO_SHOW_RESET) ? SENSE_CAP_OK : SENSE_CAP_ERR_VERIFY;
}

sense_cap_result_t tile_sense_cap_hw_reset(tile_t *tile)
{
    if (!tile || !tile->hal || !tile->id || tile->state == TILE_STATE_NONE)
        return SENSE_CAP_ERR_STATE;
    iqs7211a_state_t *s = state_for(tile);
    if (!s->mclr) return SENSE_CAP_ERR_NO_CALLBACK;
    mclr_pulse(tile, s);
    return after_reset(tile, s);
}

sense_cap_result_t tile_sense_cap_reset(tile_t *tile)
{
    if (!tile || !tile->hal || (tile->state != TILE_STATE_READY &&
                                tile->state != TILE_STATE_FOUND))
        return SENSE_CAP_ERR_STATE;
    iqs7211a_state_t *s = state_for(tile);
    s->job.phase = J_IDLE;
    s->ati.phase = A_IDLE;
    /* Takes effect when the window closes (§9.3.2); the attempts stop at
     * the first NACK once the chip is rebooting. */
    send_command(tile, s, IQS7211A_CTRL_SW_RESET);
    return after_reset(tile, s);
}

uint16_t tile_sense_cap_get_version_major(tile_t *tile)
{
    return tile ? state_for(tile)->ver_major : 0;
}

uint16_t tile_sense_cap_get_version_minor(tile_t *tile)
{
    return tile ? state_for(tile)->ver_minor : 0;
}

uint16_t tile_sense_cap_get_settings_version(tile_t *tile)
{
    uint16_t v;
    if (!ready(tile)) return 0;
    return rd_words(tile, state_for(tile), IQS7211A_REG_SETTINGS_VER, &v, 1) ? v : 0;
}

/* ================================================================
 * Event processing
 * ================================================================ */

/**
 * Show Reset in a data read. If an Ack Reset has ever been seen to clear
 * the flag, it is a real reset. Otherwise (the ack may not take on this
 * silicon, ring NOTES 2026-09-22) check whether our Config Settings
 * survived: a reset returns them to the chip defaults.
 */
static uint8_t reset_is_real(tile_t *tile, iqs7211a_state_t *s)
{
    uint16_t cfg;
    if (s->ack_verified) return 1;
    /* Unreadable (process() caps the wait): assume stale for now
     * (re-acked, checked again later) rather than risk a re-apply and
     * RESET event on every read. */
    if (!rd_words(tile, s, IQS7211A_REG_CONFIG_SETTINGS, &cfg, 1)) return 0;
    s->cr_mode = (cfg & IQS7211A_CFG_COMMS_REQUEST_EN) ? 1 : 0;
    return (uint8_t)(cfg != s->config);
}

sense_cap_result_t tile_sense_cap_process(tile_t *tile)
{
    uint8_t buf[BLK_WORDS * 2];
    if (!ready(tile)) return SENSE_CAP_ERR_STATE;
    iqs7211a_state_t *s = state_for(tile);

    if (job_active(s)) return job_step(tile, s);
    if (ati_active(s)) return tile_sense_cap_ati_poll(tile);

    uint32_t now = now_ms(s);
    if (s->req_pending && now - s->last_try_t < IQS_PROCESS_RETRY_MS)
        return SENSE_CAP_BUSY;
    s->last_try_t = now;

    /* One burst across 0x10..0x1B (reads auto-increment, §11.5). */
    uint8_t r = xf_read(tile, IQS7211A_REG_INFO_FLAGS, buf, sizeof(buf));
    if (r != XF_OK) {
        if (r == XF_BUSY) s->cr_mode = 1;
        if ((r == XF_BUSY || s->cr_mode) && req_stale(s))
            comms_request(tile, s);
        if (s->err_run < 0xFFFFU) s->err_run++;
        if (s->err_run >= IQS_ERR_REPORT_RUN) {
            if (!s->err_reported) {
                s->err_reported = 1;
                TILE_ON_ERROR(tile, "sense_cap: no data window granted");
            }
            return SENSE_CAP_ERR_COMMS;
        }
        return SENSE_CAP_BUSY;
    }
    s->req_pending = 0;
    s->err_run = 0;
    s->err_reported = 0;

    uint16_t w[BLK_WORDS];
    for (uint8_t i = 0; i < BLK_WORDS; i++)
        w[i] = (uint16_t)(((uint16_t)buf[i * 2 + 1] << 8) | buf[i * 2]);

    s->ev_call = 0;
    if (w[BLK_INFO_FLAGS] & IQS7211A_INFO_SHOW_RESET) {
        /* At most one more transaction this call, its window wait capped
         * at step_cap(): a due re-ack of a stale indication, or else the
         * Config Settings check. */
        uint8_t real = s->ack_verified;
        s->step = 1;
        if (!real && now - s->reset_t >= IQS_RESET_ACK_RETRY_MS) {
            /* Stale (the ack may not take on this silicon, ring NOTES
             * 2026-09-22): re-ack now and then, and use the data. One
             * slice; an ack that misses its window is retried next time. */
            s->reset_t = now;
            s->wr_on = 0;
            (void)cmd_slice(tile, s, IQS7211A_CTRL_ACK_RESET);
            s->wr_on = 0;
        } else if (!real) {
            real = reset_is_real(tile, s);
        }
        s->step = 0;
        if (real) {
            touch_reset_all(tile, s, 1);
            ev_raise(s, SENSE_CAP_EV_RESET);
            if (s->on_event)
                s->on_event(tile, (uint8_t)(s->ev_call & 0xFFU), s->event_ctx);
            s->rates_known = 0;
            s->config = 0;
            start_reapply(s);
            return SENSE_CAP_ERR_RESET;
        }
    }

    for (uint8_t i = 0; i < BLK_WORDS; i++) s->block[i] = w[i];
    if (!s->millis) s->tick += s->rate[0];   /* one report period per read */

    touch_update(tile, s);
    if (s->on_event && (s->ev_call & 0xFFU))
        s->on_event(tile, (uint8_t)(s->ev_call & 0xFFU), s->event_ctx);

    /* Ask for the next window now, so the next call usually lands. Not in
     * LP1/LP2: holding the chip awake for a window costs the power those
     * modes save. */
    uint8_t mode = (uint8_t)(s->block[BLK_INFO_FLAGS] & IQS7211A_INFO_MODE_MASK);
    if (s->cr_mode && mode <= SENSE_CAP_MODE_IDLE && !s->req_pending)
        comms_request(tile, s);
    return SENSE_CAP_OK;
}

void tile_sense_cap_on_event(tile_t *tile, sense_cap_event_cb_t cb, void *ctx)
{
    if (!tile) return;
    iqs7211a_state_t *s = state_for(tile);
    s->on_event  = cb;
    s->event_ctx = ctx;
}

void tile_sense_cap_on_touch(tile_t *tile, sense_cap_touch_cb_t cb, void *ctx)
{
    if (!tile) return;
    iqs7211a_state_t *s = state_for(tile);
    s->on_touch = cb;
    s->touch_ctx = ctx;
}

uint8_t tile_sense_cap_next_touch_event(tile_t *tile, sense_cap_touch_t *ev)
{
    if (!tile) return 0;
    iqs7211a_state_t *s = state_for(tile);
    if (s->evq_count == 0) return 0;
    if (ev) *ev = s->evq[s->evq_rd];
    s->evq_rd = (uint8_t)((s->evq_rd + 1U) % TOUCH_EVQ_LEN);
    s->evq_count--;
    return 1;
}

void tile_sense_cap_on_tap(tile_t *tile, sense_cap_tap_cb_t cb, void *ctx)
{
    if (!tile) return;
    iqs7211a_state_t *s = state_for(tile);
    s->on_tap = cb;
    s->tap_ctx = ctx;
}

void tile_sense_cap_on_long_press(tile_t *tile, sense_cap_hold_cb_t cb, void *ctx)
{
    if (!tile) return;
    iqs7211a_state_t *s = state_for(tile);
    s->on_hold = cb;
    s->hold_ctx = ctx;
}

void tile_sense_cap_on_swipe(tile_t *tile, sense_cap_swipe_cb_t cb, void *ctx)
{
    if (!tile) return;
    iqs7211a_state_t *s = state_for(tile);
    s->on_swipe = cb;
    s->swipe_ctx = ctx;
}

void tile_sense_cap_on_drag(tile_t *tile, sense_cap_drag_cb_t cb, void *ctx)
{
    if (!tile) return;
    iqs7211a_state_t *s = state_for(tile);
    s->on_drag = cb;
    s->drag_ctx = ctx;
}

void tile_sense_cap_on_pinch(tile_t *tile, sense_cap_pinch_cb_t cb, void *ctx)
{
    if (!tile) return;
    iqs7211a_state_t *s = state_for(tile);
    s->on_pinch = cb;
    s->pinch_ctx = ctx;
}

/* ================================================================
 * Runtime data
 * ================================================================ */

uint16_t tile_sense_cap_get_touch_events(tile_t *tile)
{
    if (!tile) return 0;
    iqs7211a_state_t *s = state_for(tile);
    uint16_t ev = s->ev_latch;
    s->ev_latch = 0;
    return ev;
}

uint8_t tile_sense_cap_was_tapped(tile_t *tile)
{
    if (!tile) return 0;
    iqs7211a_state_t *s = state_for(tile);
    uint16_t taps = (uint16_t)(s->ev_latch & (SENSE_CAP_EV_TAP | SENSE_CAP_EV_DOUBLE_TAP));
    s->ev_latch &= (uint16_t)~taps;
    return taps ? 1 : 0;
}

/** Nearest electrode index along an axis of n electrodes over range res. */
static uint8_t nearest_electrode(uint16_t pos, uint16_t res, uint8_t n)
{
    if (n <= 1 || res == 0) return 0;
    uint32_t k = ((uint32_t)pos * (n - 1U) + res / 2U) / res;
    return (uint8_t)(k > (uint32_t)(n - 1U) ? n - 1U : k);
}

int8_t tile_sense_cap_zone_at(tile_t *tile, uint16_t x, uint16_t y)
{
    if (!tile) return -1;
    iqs7211a_state_t *s = state_for(tile);
    if (!s->srf_applied) return -1;

    uint8_t nx = n_x(s), ny = n_y(s);
    uint8_t ix = nearest_electrode(x, eff_xres(s), nx);
    uint8_t iy = nearest_electrode(y, eff_yres(s), ny);
    /* Undo the output flips to get back to map order (§7.7). */
    if (eff_flip_x(s)) ix = (uint8_t)(nx - 1U - ix);
    if (eff_flip_y(s)) iy = (uint8_t)(ny - 1U - iy);
    uint8_t tx_i = eff_switch(s) ? ix : iy;
    uint8_t rx_i = eff_switch(s) ? iy : ix;
    return (int8_t)(tx_i * s->srf.geometry.n_rx + rx_i);
}

static uint8_t first_finger_xy(iqs7211a_state_t *s, uint16_t *x, uint16_t *y)
{
    uint8_t fingers = (uint8_t)((s->block[BLK_INFO_FLAGS] >> IQS7211A_INFO_NUM_FINGERS_SHIFT) & 3U);
    *x = s->block[BLK_FINGER_BASE + 0];
    *y = s->block[BLK_FINGER_BASE + 1];
    return (uint8_t)(fingers > 0 && *x != XY_INVALID && *y != XY_INVALID);
}

int8_t tile_sense_cap_get_zone(tile_t *tile)
{
    uint16_t x, y;
    if (!tile || !first_finger_xy(state_for(tile), &x, &y)) return -1;
    return tile_sense_cap_zone_at(tile, x, y);
}

int16_t tile_sense_cap_get_x_pct(tile_t *tile)
{
    uint16_t x, y;
    if (!tile) return -1;
    iqs7211a_state_t *s = state_for(tile);
    uint16_t res = eff_xres(s);
    if (!first_finger_xy(s, &x, &y) || res == 0) return -1;
    uint32_t p = (uint32_t)x * 100U / res;
    return (int16_t)(p > 100 ? 100 : p);
}

int16_t tile_sense_cap_get_y_pct(tile_t *tile)
{
    uint16_t x, y;
    if (!tile) return -1;
    iqs7211a_state_t *s = state_for(tile);
    uint16_t res = eff_yres(s);
    if (!first_finger_xy(s, &x, &y) || res == 0) return -1;
    uint32_t p = (uint32_t)y * 100U / res;
    return (int16_t)(p > 100 ? 100 : p);
}

uint8_t tile_sense_cap_wait_for_touch(tile_t *tile, uint32_t timeout_ms)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    uint32_t t0 = now_ms(s);
    for (;;) {
        (void)tile_sense_cap_process(tile);
        if (tile_sense_cap_get_num_fingers(tile) > 0) return 1;
        if (now_ms(s) - t0 >= timeout_ms) return 0;
        sleep_ms(tile, s, 5);
    }
}

uint16_t tile_sense_cap_get_info_flags(tile_t *tile)
{
    return tile ? state_for(tile)->block[BLK_INFO_FLAGS] : 0;
}

uint16_t tile_sense_cap_get_gestures(tile_t *tile)
{
    return tile ? state_for(tile)->block[BLK_GESTURES] : 0;
}

uint8_t tile_sense_cap_get_num_fingers(tile_t *tile)
{
    if (!tile) return 0;
    return (uint8_t)((state_for(tile)->block[BLK_INFO_FLAGS] & IQS7211A_INFO_NUM_FINGERS_MASK)
                     >> IQS7211A_INFO_NUM_FINGERS_SHIFT);
}

static uint16_t finger_word(tile_t *tile, uint8_t finger, uint8_t word)
{
    if (!tile || finger >= SENSE_CAP_NUM_FINGERS) return 0;
    return state_for(tile)->block[BLK_FINGER_BASE + finger * 4U + word];
}

uint16_t tile_sense_cap_get_finger_x(tile_t *tile, uint8_t finger)        { return finger_word(tile, finger, 0); }
uint16_t tile_sense_cap_get_finger_y(tile_t *tile, uint8_t finger)        { return finger_word(tile, finger, 1); }
uint16_t tile_sense_cap_get_finger_strength(tile_t *tile, uint8_t finger) { return finger_word(tile, finger, 2); }
uint16_t tile_sense_cap_get_finger_area(tile_t *tile, uint8_t finger)     { return finger_word(tile, finger, 3); }

int16_t tile_sense_cap_get_relative_x(tile_t *tile)
{
    return tile ? (int16_t)state_for(tile)->block[BLK_RELATIVE_X] : 0;
}

int16_t tile_sense_cap_get_relative_y(tile_t *tile)
{
    return tile ? (int16_t)state_for(tile)->block[BLK_RELATIVE_Y] : 0;
}

uint32_t tile_sense_cap_get_touch_status(tile_t *tile)
{
    uint16_t w[2];
    if (!ready(tile)) return 0;
    if (!rd_words(tile, state_for(tile), IQS7211A_REG_TOUCH_STATUS_0, w, 2)) return 0;
    return ((uint32_t)w[1] << 16) | w[0];
}

uint8_t tile_sense_cap_is_touched(tile_t *tile, uint8_t channel)
{
    if (channel > 31) return 0;
    return (uint8_t)((tile_sense_cap_get_touch_status(tile) >> channel) & 1U);
}

uint8_t tile_sense_cap_get_num_channels(tile_t *tile)
{
    if (!tile) return 0;
    iqs7211a_state_t *s = state_for(tile);
    return s->srf_applied ? s->n_ch : 0;
}

uint16_t tile_sense_cap_get_channel_count(tile_t *tile, uint8_t channel)
{
    uint16_t v;
    if (!ready(tile) || channel >= 32) return 0;
    return rd_words(tile, state_for(tile), (uint16_t)(IQS7211A_REG_EXT_COUNTS + channel), &v, 1) ? v : 0;
}

int16_t tile_sense_cap_get_channel_delta(tile_t *tile, uint8_t channel)
{
    uint16_t v;
    if (!ready(tile) || channel >= 32) return 0;
    return rd_words(tile, state_for(tile), (uint16_t)(IQS7211A_REG_EXT_DELTAS + channel), &v, 1)
               ? (int16_t)v : 0;
}

uint8_t tile_sense_cap_read_channel_counts(tile_t *tile, uint16_t *counts, uint8_t n)
{
    if (!ready(tile) || !counts || n == 0 || n > 32) return 0;
    return rd_words(tile, state_for(tile), IQS7211A_REG_EXT_COUNTS, counts, n) ? n : 0;
}

uint8_t tile_sense_cap_read_channel_deltas(tile_t *tile, int16_t *deltas, uint8_t n)
{
    uint16_t raw[32];
    if (!ready(tile) || !deltas || n == 0 || n > 32) return 0;
    if (!rd_words(tile, state_for(tile), IQS7211A_REG_EXT_DELTAS, raw, n)) return 0;
    for (uint8_t i = 0; i < n; i++) deltas[i] = (int16_t)raw[i];
    return n;
}

uint8_t tile_sense_cap_read_channels(tile_t *tile, uint16_t *counts,
                                     int16_t *deltas, uint8_t n)
{
    uint8_t ok = 1;
    if (!ready(tile) || n == 0 || n > 32) return 0;
    if (counts && !tile_sense_cap_read_channel_counts(tile, counts, n)) ok = 0;
    if (deltas && !tile_sense_cap_read_channel_deltas(tile, deltas, n)) ok = 0;
    return ok;
}

uint8_t tile_sense_cap_is_alp_active(tile_t *tile)
{
    return (tile && (state_for(tile)->block[BLK_INFO_FLAGS] & IQS7211A_INFO_ALP_OUTPUT)) ? 1 : 0;
}

uint16_t tile_sense_cap_get_alp_count(tile_t *tile)
{
    uint16_t v;
    if (!ready(tile)) return 0;
    return rd_words(tile, state_for(tile), IQS7211A_REG_ALP_COUNT, &v, 1) ? v : 0;
}

uint16_t tile_sense_cap_get_alp_lta(tile_t *tile)
{
    uint16_t v;
    if (!ready(tile)) return 0;
    return rd_words(tile, state_for(tile), IQS7211A_REG_ALP_LTA, &v, 1) ? v : 0;
}

uint8_t tile_sense_cap_get_mode(tile_t *tile)
{
    return tile ? (uint8_t)(state_for(tile)->block[BLK_INFO_FLAGS] & IQS7211A_INFO_MODE_MASK) : 0;
}

uint8_t tile_sense_cap_has_ati_error(tile_t *tile)
{
    if (!tile) return 0;
    iqs7211a_state_t *s = state_for(tile);
    uint16_t info = s->block[BLK_INFO_FLAGS];
    if (info & IQS7211A_INFO_ATI_ERROR) return 1;
    return (uint8_t)(alp_in_use(s) && (info & IQS7211A_INFO_ALP_ATI_ERROR));
}

/* ================================================================
 * Configuration
 * ================================================================ */

/* Setters change the desired state, then sync the affected registers. A
 * running surface job picks the change up instead (it re-syncs before
 * finishing), so the setter reports success for the queued change. */
static uint8_t commit(tile_t *tile, iqs7211a_state_t *s, uint8_t first, uint8_t n)
{
    if (job_active(s)) { s->job.dirty = 1; return 1; }
    if (ati_active(s)) return 0;
    return apply_range(tile, s, first, n);
}

static uint8_t commit_config(tile_t *tile, iqs7211a_state_t *s)
{
    if (job_active(s)) { s->job.dirty = 1; return 1; }
    if (ati_active(s)) return 0;
    return config_apply(tile, s);
}

uint8_t tile_sense_cap_set_sensitivity(tile_t *tile, sense_cap_sensitivity_t level)
{
    if (!ready(tile) || (unsigned)level > (unsigned)SENSE_CAP_SENS_HIGHEST) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->sens = (uint8_t)level;
    apply_sensitivity(s);
    return commit(tile, s, IQS7211A_REG_TOUCH_MULT, 1);
}

uint8_t tile_sense_cap_set_touch_multipliers(tile_t *tile, uint8_t set_mult,
                                             uint8_t clear_mult)
{
    if (!ready(tile) || set_mult < 1) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->mult_set = set_mult;
    s->mult_clear = clear_mult > set_mult ? set_mult : clear_mult;
    return commit(tile, s, IQS7211A_REG_TOUCH_MULT, 1);
}

uint8_t tile_sense_cap_set_power_profile(tile_t *tile, sense_cap_power_profile_t profile)
{
    if (!ready(tile) || (unsigned)profile > (unsigned)SENSE_CAP_POWER_LOW_POWER) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->profile = (uint8_t)profile;
    for (uint8_t i = 0; i < 5; i++) s->rate[i] = PROFILES[profile].rate[i];
    for (uint8_t i = 0; i < 4; i++) s->tmo[i]  = PROFILES[profile].tmo[i];
    return commit(tile, s, IQS7211A_REG_RATE_ACTIVE, 9);
}

uint8_t tile_sense_cap_set_power_mode(tile_t *tile, sense_cap_power_mode_t mode)
{
    if (!ready(tile) || (unsigned)mode > (unsigned)SENSE_CAP_POWER_FORCE_LP2) return 0;
    iqs7211a_state_t *s = state_for(tile);
    uint8_t sel;
    switch (mode) {
    case SENSE_CAP_POWER_FORCE_ACTIVE: sel = SENSE_CAP_MODE_ACTIVE; break;
    case SENSE_CAP_POWER_FORCE_LP1:    sel = SENSE_CAP_MODE_LP1;    break;
    case SENSE_CAP_POWER_FORCE_LP2:    sel = SENSE_CAP_MODE_LP2;    break;
    default:                           sel = SENSE_CAP_MODE_ACTIVE; break;
    }
    if (job_active(s)) {
        s->power_mode = (uint8_t)mode;
        s->mode_sel = (mode == SENSE_CAP_POWER_AUTO) ? 0 : sel;
        s->job.dirty = 1;
        return 1;
    }
    if (ati_active(s)) return 0;

    if (mode == SENSE_CAP_POWER_AUTO) {
        /* Mode select back to Active while manual control still holds
         * (it only applies there, Table A.6), then release control. */
        if (s->config & IQS7211A_CFG_MANUAL_CONTROL)
            (void)select_mode(tile, s, SENSE_CAP_MODE_ACTIVE);
        s->power_mode = SENSE_CAP_POWER_AUTO;
        s->mode_sel = 0;
        return config_apply(tile, s);
    }
    s->power_mode = (uint8_t)mode;
    s->mode_sel = sel;
    if (!config_apply(tile, s)) return 0;
    return select_mode(tile, s, sel);
}

uint8_t tile_sense_cap_set_active_rate(tile_t *tile, uint16_t ms)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    if (ms < 10) ms = 10;
    if (ms > 1000) ms = 1000;
    s->rate[0] = ms;
    return commit(tile, s, IQS7211A_REG_RATE_ACTIVE, 1);
}

uint8_t tile_sense_cap_set_idle_timeout(tile_t *tile, uint16_t seconds)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->tmo[SENSE_CAP_MODE_IDLE] = u16min(seconds, 3600);
    return commit(tile, s, IQS7211A_REG_TIMEOUT_IDLE, 1);
}

uint8_t tile_sense_cap_set_report_rate(tile_t *tile, sense_cap_mode_t mode, uint16_t ms)
{
    if (!ready(tile) || (unsigned)mode > (unsigned)SENSE_CAP_MODE_LP2 || ms == 0) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->rate[mode] = ms;
    return commit(tile, s, (uint8_t)(IQS7211A_REG_RATE_ACTIVE + mode), 1);
}

uint8_t tile_sense_cap_set_mode_timeout(tile_t *tile, sense_cap_mode_t mode, uint16_t seconds)
{
    if (!ready(tile) || (unsigned)mode > (unsigned)SENSE_CAP_MODE_LP1) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->tmo[mode] = seconds;
    return commit(tile, s, (uint8_t)(IQS7211A_REG_TIMEOUT_ACTIVE + mode), 1);
}

uint8_t tile_sense_cap_set_alp_wake(tile_t *tile, uint8_t enable)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    if (enable && !s->srf.alp.enable) return 0;
    s->alp_wake = enable ? 1 : 0;
    if (job_active(s)) { s->job.dirty = 1; return 1; }
    if (ati_active(s)) return 0;
    return (uint8_t)(apply_range(tile, s, IQS7211A_REG_TIMEOUT_IDLE, 1) && config_apply(tile, s));
}

uint8_t tile_sense_cap_set_alp_threshold(tile_t *tile, uint16_t threshold)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    if (threshold < 1) threshold = 1;
    if (threshold > 1000) threshold = 1000;
    s->alp_threshold = threshold;
    s->alp_thr_set = 1;
    return commit(tile, s, IQS7211A_REG_ALP_THRESHOLD, 1);
}

/* Orientation / resolution changes move the gesture pixel distances too. */
static uint8_t commit_xy(tile_t *tile, iqs7211a_state_t *s)
{
    if (job_active(s)) { s->job.dirty = 1; return 1; }
    if (ati_active(s)) return 0;
    return (uint8_t)(apply_range(tile, s, IQS7211A_REG_TP_SETTINGS, 4) &&
                     apply_range(tile, s, IQS7211A_REG_GESTURE_ENABLE, 8));
}

uint8_t tile_sense_cap_set_flip_x(tile_t *tile, uint8_t enable)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->user_flip_x = enable ? 1 : 0;
    return commit_xy(tile, s);
}

uint8_t tile_sense_cap_set_flip_y(tile_t *tile, uint8_t enable)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->user_flip_y = enable ? 1 : 0;
    return commit_xy(tile, s);
}

uint8_t tile_sense_cap_set_switch_xy(tile_t *tile, uint8_t enable)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->user_switch = enable ? 1 : 0;
    return commit_xy(tile, s);
}

uint8_t tile_sense_cap_set_resolution(tile_t *tile, uint16_t x_res, uint16_t y_res)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->user_xres = x_res;
    s->user_yres = y_res;
    return commit_xy(tile, s);
}

uint8_t tile_sense_cap_set_tap_time(tile_t *tile, uint16_t ms)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    if (ms < 50) ms = 50;
    s->tap_ms = u16min(ms, 2000);
    return commit(tile, s, IQS7211A_REG_TAP_TIME, 1);
}

uint8_t tile_sense_cap_set_hold_time(tile_t *tile, uint16_t ms)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->hold_ms = u16min(ms, 5000);
    return commit(tile, s, IQS7211A_REG_HOLD_TIME, 1);
}

uint8_t tile_sense_cap_set_swipe_time(tile_t *tile, uint16_t ms)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    if (ms < 50) ms = 50;
    s->swipe_ms = u16min(ms, 2000);
    return commit(tile, s, IQS7211A_REG_SWIPE_TIME, 1);
}

uint8_t tile_sense_cap_set_swipe_distance(tile_t *tile, uint16_t px)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->swipe_dist_user = px;
    return commit(tile, s, IQS7211A_REG_SWIPE_X_DIST, 2);
}

uint8_t tile_sense_cap_set_swipe_angle(tile_t *tile, uint8_t degrees)
{
    if (!ready(tile) || degrees < 1 || degrees > 75) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->swipe_angle_deg = degrees;
    return commit(tile, s, IQS7211A_REG_SWIPE_ANGLE, 1);
}

uint8_t tile_sense_cap_set_gesture_source(tile_t *tile, sense_cap_gesture_source_t source)
{
    if (!ready(tile) || (unsigned)source > (unsigned)SENSE_CAP_GESTURES_CHIP) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->gest_src = (uint8_t)source;
    s->prev_gestures = 0;
    if (job_active(s)) { s->job.dirty = 1; return 1; }
    if (ati_active(s)) return 0;
    return (uint8_t)(apply_range(tile, s, IQS7211A_REG_GESTURE_ENABLE, 1) && config_apply(tile, s));
}

uint8_t tile_sense_cap_enable_gestures(tile_t *tile, uint16_t mask)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->gest_mask = (uint16_t)(mask & SENSE_CAP_GESTURE_ALL);
    return commit(tile, s, IQS7211A_REG_GESTURE_ENABLE, 1);
}

uint8_t tile_sense_cap_set_max_touches(tile_t *tile, uint8_t fingers)
{
    if (!ready(tile) || fingers < 1 || fingers > SENSE_CAP_NUM_FINGERS) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->max_touches_user = fingers;
    return commit(tile, s, IQS7211A_REG_TP_TOUCHES, 1);
}

uint8_t tile_sense_cap_set_reference_update(tile_t *tile, uint16_t seconds)
{
    if (!ready(tile) || seconds > 60) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->ref_update = seconds;
    s->ref_update_set = 1;
    return commit(tile, s, IQS7211A_REG_REF_UPDATE_TIME, 1);
}

uint8_t tile_sense_cap_set_i2c_timeout(tile_t *tile, uint16_t ms)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    if (ms < 2) ms = 2;
    s->i2c_timeout = u16min(ms, 1000);
    if (!commit(tile, s, IQS7211A_REG_I2C_TIMEOUT, 1)) return 0;
    if (!job_active(s)) s->chip_i2c_to = s->i2c_timeout;
    return 1;
}

uint8_t tile_sense_cap_set_watchdog(tile_t *tile, uint8_t enable)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    s->wdt = enable ? 1 : 0;
    return commit_config(tile, s);
}

uint8_t tile_sense_cap_reseed(tile_t *tile)
{
    if (!ready(tile)) return 0;
    iqs7211a_state_t *s = state_for(tile);
    if (job_active(s) || ati_active(s)) return 0;
    send_command(tile, s, (uint16_t)(IQS7211A_CTRL_TP_RESEED | IQS7211A_CTRL_ALP_RESEED));
    return 1;
}

/* ================================================================
 * Advanced / escape hatch
 * ================================================================ */

uint16_t tile_sense_cap_read_reg(tile_t *tile, uint8_t reg)
{
    uint16_t v;
    if (!tile || !tile->hal || (tile->state != TILE_STATE_READY &&
                                tile->state != TILE_STATE_FOUND))
        return IQS7211A_INVALID_RESPONSE;
    return rd_words(tile, state_for(tile), reg, &v, 1) ? v : IQS7211A_INVALID_RESPONSE;
}

/** Map bytes naming RX0/TX0 (grounded) or TX10 (bonded to TX9). */
static uint8_t map_byte_forbidden(uint8_t b)
{
    return (uint8_t)(b == 0 || b == 10);
}

sense_cap_result_t tile_sense_cap_write_reg(tile_t *tile, uint8_t reg, uint16_t value)
{
    if (!tile || !tile->hal || (tile->state != TILE_STATE_READY &&
                                tile->state != TILE_STATE_FOUND))
        return SENSE_CAP_ERR_STATE;
    iqs7211a_state_t *s = state_for(tile);

    /* Tile guards (HW-gated: RX0/TX0 grounded, TX9/TX10 one ball). */
    if (value == IQS7211A_INVALID_RESPONSE) return SENSE_CAP_ERR_FORBIDDEN;
    if (reg == IQS7211A_REG_SYSTEM_CONTROL && (value & IQS7211A_CTRL_TX_TEST))
        return SENSE_CAP_ERR_FORBIDDEN;
    if (reg == IQS7211A_REG_ALP_SETUP && (value & 1U))
        return SENSE_CAP_ERR_FORBIDDEN;
    if (reg == IQS7211A_REG_ALP_TX_ENABLE && (value & ((1U << 0) | (1U << 10))))
        return SENSE_CAP_ERR_FORBIDDEN;
    if (reg >= IQS7211A_REG_RXTX_MAP_BASE && reg <= 0x95 &&
        (map_byte_forbidden((uint8_t)(value & 0xFFU)) || map_byte_forbidden((uint8_t)(value >> 8))))
        return SENSE_CAP_ERR_FORBIDDEN;

    if (job_active(s) || ati_active(s)) return SENSE_CAP_ERR_STATE;

    if (reg == IQS7211A_REG_SYSTEM_CONTROL) {
        /* Commands self-clear, so there is nothing to read back. */
        uint8_t prev = s->mode_sel;
        s->mode_sel = (uint8_t)(value & IQS7211A_CTRL_MODE_MASK);
        send_command(tile, s, (uint16_t)(value & (uint16_t)~IQS7211A_CTRL_MODE_MASK));
        if (!(s->config & IQS7211A_CFG_MANUAL_CONTROL)) s->mode_sel = prev;
        return SENSE_CAP_OK;
    }
    if (!wr_verified(tile, s, reg, value)) return SENSE_CAP_ERR_VERIFY;
    if (reg == IQS7211A_REG_CONFIG_SETTINGS) {
        s->config = value;
        s->cr_mode = (value & IQS7211A_CFG_COMMS_REQUEST_EN) ? 1 : 0;
    }
    if (reg >= IQS7211A_REG_RATE_ACTIVE && reg <= IQS7211A_REG_RATE_LP2)
        s->chip_rate[reg - IQS7211A_REG_RATE_ACTIVE] = value ? value : 1;
    if (reg == IQS7211A_REG_I2C_TIMEOUT)
        s->chip_i2c_to = value;
    return SENSE_CAP_OK;
}
