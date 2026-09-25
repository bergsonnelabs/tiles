/**
 * @file  tile_sense_hr.c
 * @brief Sense.HR tile driver implementation — MAX86174A PPG front end.
 */

#include "tile_sense_hr.h"

/* ---- Instance → I2C address table ---- */

static const uint8_t id_table[] = {
    MAX86174A_I2C_ADDR_DEFAULT,  /* 0: 0x6B (ADDR high, pad 2 open) */
    MAX86174A_I2C_ADDR_ALT,      /* 1: 0x6A (ADDR low, pad 2 to GND) */
};

#define NUM_INSTANCES  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    return (instance < NUM_INSTANCES) ? id_table[instance] : 0;
}

/* ---- Per-instance state ---- */

typedef struct {
    uint8_t led;        /* sense_hr_led_t (MEASx_DRVA)            */
    uint8_t ambient;    /* 1 = direct ambient slot (MEASx_AMB)    */
    uint8_t led_rge;    /* MEASx_LED_RGE, 0..3                    */
    uint8_t pa;         /* MEASx_DRVA_PA, 0 = LED off             */
    uint8_t dacoff;     /* MEASx_PPGy_DACOFF code, 0..7           */
    uint8_t adc_rge;    /* MEASx_PPGy_ADC_RGE, 0..3               */
    uint8_t tint;       /* MEASx_TINT, 0..3                       */
    uint8_t aver;       /* MEASx_AVER, 0..7                       */
    uint8_t pdsel;      /* MEASx_PDSEL, 0..3                      */
} hr_slot_t;

typedef struct {
    uint16_t fr_div;         /* FR_CLK_DIV as programmed              */
    uint8_t  enable_mask;    /* SYS_CFG2 slot enables                 */
    uint8_t  running;        /* 1 between start() and stop()          */
    uint8_t  sys_cfg1;       /* cached SYS_CFG1 (PPG2_PWRDN, SHDN)    */
    uint8_t  sys_cfg3;       /* cached SYS_CFG3 (PROX_AUTO, ...)      */
    uint8_t  int_pin;        /* Core pad wired to INTB, 0 = polled    */
    volatile uint8_t int_flag;
    sense_hr_event_cb_t on_event;
    void    *event_ctx;
    /* tile-wide defaults inherited by newly configured slots */
    uint8_t  def_tint, def_aver, def_adc_rge, def_pdsel;
    hr_slot_t slot[SENSE_HR_SLOTS];
    int32_t  frame[SENSE_HR_FRAME_LEN];
    uint16_t ovf_count;
} hr_state_t;

static hr_state_t hr_state[NUM_INSTANCES];

static hr_state_t *state_for(tile_t *tile)
{
    for (uint8_t i = 0; i < NUM_INSTANCES; i++)
        if (id_table[i] == tile->id) return &hr_state[i];
    return &hr_state[0];
}

/* ISR trampolines — one per instance, keep them trivial. */
static void _intb_isr_0(void *ctx) { hr_state[0].int_flag = 1; (void)ctx; }
static void _intb_isr_1(void *ctx) { hr_state[1].int_flag = 1; (void)ctx; }

/* ---- Portable memzero ---- */

static void memzero(void *p, uint16_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) *b++ = 0;
}

/* ---- Bus helpers ---- */

static void hr_write(tile_t *tile, uint8_t reg, uint8_t val)
{
    tile->hal->i2c_write(tile->hal->handle, tile->id, reg, &val, 1);
}

static uint8_t hr_read(tile_t *tile, uint8_t reg)
{
    uint8_t val = 0;
    tile->hal->i2c_read(tile->hal->handle, tile->id, reg, &val, 1);
    return val;
}

static uint8_t meas_reg(uint8_t slot, uint8_t off)
{
    return (uint8_t)(MAX86174A_REG_MEAS1_BASE + 8u * slot + off);
}

/* ---- Conversion tables (datasheet register decodes) ---- */

static const uint16_t led_lsb_ua[4]  = { 125, 250, 375, 500 };  /* per LED_RGE */
static const uint8_t  adc_fs_ua[4]   = { 4, 8, 16, 32 };        /* per ADC_RGE */
static const uint8_t  dacoff_ua[8]   = { 0, 4, 8, 12, 16, 20, 24, 28 };

/* Pick the smallest LED range covering `ua`, then the nearest code. */
static void led_quantise(uint32_t ua, uint8_t *rge, uint8_t *pa)
{
    if (ua == 0) { *pa = 0; return; }
    if (ua > SENSE_HR_LED_MAX_UA) ua = SENSE_HR_LED_MAX_UA;
    for (uint8_t r = 0; r < 4; r++) {
        uint32_t lsb = led_lsb_ua[r];
        if (ua <= 255u * lsb) {
            uint32_t code = (ua + lsb / 2) / lsb;
            if (code == 0) code = 1;
            if (code > 255) code = 255;
            *rge = r;
            *pa  = (uint8_t)code;
            return;
        }
    }
    *rge = 3;
    *pa  = 255;
}

/* ---- Slot register writers ---- */

static void slot_write_sel(tile_t *tile, uint8_t slot)
{
    hr_slot_t *s = &state_for(tile)->slot[slot];
    uint8_t sel = s->ambient ? (uint8_t)(1u << 6) : (uint8_t)(s->led & 0x03);
    hr_write(tile, meas_reg(slot, MAX86174A_MEAS_SEL), sel);
}

static void slot_write_cfg1(tile_t *tile, uint8_t slot)
{
    hr_slot_t *s = &state_for(tile)->slot[slot];
    hr_write(tile, meas_reg(slot, MAX86174A_MEAS_CFG1),
             (uint8_t)(((s->pdsel & 0x03) << 6) | ((s->tint & 0x03) << 3) |
                       (s->aver & 0x07)));
}

static void slot_write_cfg2(tile_t *tile, uint8_t slot)
{
    hr_slot_t *s = &state_for(tile)->slot[slot];
    hr_write(tile, meas_reg(slot, MAX86174A_MEAS_CFG2),
             (uint8_t)(((s->led_rge & 0x03) << 4) | ((s->adc_rge & 0x03) << 2) |
                       (s->adc_rge & 0x03)));
}

static void slot_write_cfg4(tile_t *tile, uint8_t slot)
{
    hr_slot_t *s = &state_for(tile)->slot[slot];
    hr_write(tile, meas_reg(slot, MAX86174A_MEAS_CFG4),
             (uint8_t)(((s->dacoff & 0x07) << 4) | (s->dacoff & 0x07)));
}

static void slot_write_current(tile_t *tile, uint8_t slot)
{
    hr_slot_t *s = &state_for(tile)->slot[slot];
    hr_write(tile, meas_reg(slot, MAX86174A_MEAS_DRVA_PA), s->pa);
    hr_write(tile, meas_reg(slot, MAX86174A_MEAS_DRVB_PA), 0x00);
}

static void slot_write_all(tile_t *tile, uint8_t slot)
{
    slot_write_sel(tile, slot);
    slot_write_cfg1(tile, slot);
    slot_write_cfg2(tile, slot);
    slot_write_cfg4(tile, slot);
    slot_write_current(tile, slot);
}

/* Push the enable mask to the chip if the frame engine is running. */
static void apply_enables(tile_t *tile)
{
    hr_state_t *st = state_for(tile);
    if (st->running)
        hr_write(tile, MAX86174A_REG_SYS_CFG2, st->enable_mask & 0x3F);
}

static uint8_t channels_active(const hr_state_t *st)
{
    return (st->sys_cfg1 & MAX86174A_CFG1_PPG2_PWRDN) ? 1 : 2;
}

static uint8_t slots_enabled(const hr_state_t *st)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < SENSE_HR_SLOTS; i++)
        if (st->enable_mask & (1u << i)) n++;
    /* PROX_AUTO enables MEAS6 even when its bit is clear. */
    if ((st->sys_cfg3 & MAX86174A_CFG3_PROX_AUTO) && !(st->enable_mask & 0x20)) n++;
    return n;
}

/* Samples the chip pushes per frame with the current configuration. */
static uint16_t frame_samples(const hr_state_t *st)
{
    uint16_t n = (uint16_t)slots_enabled(st) * channels_active(st);
    return n ? n : 1;
}

static uint16_t fifo_count(tile_t *tile)
{
    /* One 2-byte burst (the address auto-increments), so DATA_COUNT[8] and
     * [7:0] come from the same instant; two reads could straddle 255->256
     * and return 0. A non-zero OVF_COUNTER means the FIFO is full (the
     * datasheet's read pseudo-code). */
    uint8_t c[2] = {0, 0};
    if (tile->hal->i2c_read(tile->hal->handle, tile->id,
                            MAX86174A_REG_FIFO_CNT1, c, 2) != 0)
        return 0;
    if (c[0] & 0x7F) return SENSE_HR_FIFO_DEPTH;
    return (uint16_t)(((c[0] & 0x80) ? 0x100 : 0) | c[1]);
}

/* The FIFO's two non-sample words are whole 24-bit values: 0xFFFFFF from
 * an empty FIFO, 0xFFFFFE the marker. Tag 0xF never carries a sample, so a
 * real sample of -1 or -2 (tag 0..B) is kept. */
#define FIFO_WORD_EMPTY   0xFFFFFFul
#define FIFO_WORD_MARKER  0xFFFFFEul
static uint8_t fifo_word_is_filler(uint32_t w)
{
    return w == FIFO_WORD_EMPTY || w == FIFO_WORD_MARKER;
}

/* Read up to `count` raw 24-bit words; returns the number read. */
static uint16_t fifo_read_words(tile_t *tile, uint32_t *buf, uint16_t count)
{
    uint16_t avail = fifo_count(tile);
    if (count > avail) count = avail;
    if (count > SENSE_HR_FIFO_DEPTH) count = SENSE_HR_FIFO_DEPTH;

    uint16_t done = 0;
    uint8_t raw[64 * 3];
    while (done < count) {
        uint16_t n = (uint16_t)(count - done);
        if (n > 64) n = 64;
        if (tile->hal->i2c_read(tile->hal->handle, tile->id,
                                MAX86174A_REG_FIFO_DATA, raw, (uint16_t)(n * 3)) != 0)
            break;
        for (uint16_t i = 0; i < n; i++)
            buf[done + i] = ((uint32_t)raw[i * 3] << 16) |
                            ((uint32_t)raw[i * 3 + 1] << 8) |
                             (uint32_t)raw[i * 3 + 2];
        done = (uint16_t)(done + n);
    }
    return done;
}

static int32_t sign_extend_20(uint32_t raw)
{
    int32_t v = (int32_t)(raw & MAX86174A_FIFO_DATA_MASK);
    if (v & 0x80000) v -= 0x100000;
    return v;
}

/* The tags one frame produces, in FIFO order: each enabled measurement
 * from MEAS1 up, PPG1 then PPG2 [DS "FIFO Description"]. Returns count. */
static uint8_t frame_tags(const hr_state_t *st, uint8_t *tags)
{
    uint8_t mask = st->enable_mask & 0x3F;
    if (st->sys_cfg3 & MAX86174A_CFG3_PROX_AUTO) mask |= 0x20;
    uint8_t ch = channels_active(st);
    uint8_t n = 0;
    for (uint8_t m = 0; m < SENSE_HR_SLOTS; m++) {
        if (!(mask & (1u << m))) continue;
        tags[n++] = m;                                  /* PPG1 */
        if (ch > 1) tags[n++] = (uint8_t)(m + SENSE_HR_SLOTS);  /* PPG2 */
    }
    return n;
}

/* Drain the FIFO into the cached frame: keep the last sample per tag.
 * Every entry starts as SENSE_HR_NO_SAMPLE, so a slot that is disabled,
 * powered down or over range this drain reads that rather than a stale
 * value. An overflow tag stands in the place of the sample it replaces
 * [DS Table 6], so its position in the frame says which one it was.
 * Returns the number of data words decoded (overflow tags excluded). */
static uint16_t drain_into_frame(tile_t *tile)
{
    hr_state_t *st = state_for(tile);
    uint32_t words[64];
    uint16_t decoded = 0;
    uint8_t  seq[SENSE_HR_FRAME_LEN];
    uint8_t  seq_len = frame_tags(st, seq);
    uint8_t  pos = 0;                 /* expected position in the frame */
    st->ovf_count = 0;
    for (uint8_t i = 0; i < SENSE_HR_FRAME_LEN; i++)
        st->frame[i] = SENSE_HR_NO_SAMPLE;

    for (;;) {
        uint16_t n = fifo_read_words(tile, words, 64);
        if (n == 0) break;
        for (uint16_t i = 0; i < n; i++) {
            if (fifo_word_is_filler(words[i])) continue;
            uint32_t data = words[i] & MAX86174A_FIFO_DATA_MASK;
            uint8_t  tag  = (uint8_t)((words[i] >> MAX86174A_FIFO_TAG_SHIFT) & 0x0F);
            if (tag == MAX86174A_FIFO_TAG_EXP_OVF || tag == MAX86174A_FIFO_TAG_ALC_OVF) {
                st->ovf_count++;
                if (seq_len) {
                    uint8_t t = seq[pos];
                    st->frame[(t % SENSE_HR_SLOTS) * SENSE_HR_CHANNELS + t / SENSE_HR_SLOTS] =
                        SENSE_HR_NO_SAMPLE;
                    pos = (uint8_t)((pos + 1) % seq_len);
                }
                continue;
            }
            if (tag >= SENSE_HR_FRAME_LEN) continue;   /* DARK / reserved */
            /* tag 0..5 = MEAS1..6 PPG1, 6..11 = MEAS1..6 PPG2 */
            uint8_t slot = (uint8_t)(tag % SENSE_HR_SLOTS);
            uint8_t ch   = (uint8_t)(tag / SENSE_HR_SLOTS);
            st->frame[slot * SENSE_HR_CHANNELS + ch] = sign_extend_20(data);
            decoded++;
            /* Re-sync the position on every real sample. */
            for (uint8_t k = 0; k < seq_len; k++)
                if (seq[k] == tag) { pos = (uint8_t)((k + 1) % seq_len); break; }
        }
        if (n < 64) break;
    }
    return decoded;
}

/* ---- Lifecycle ---- */

uint8_t tile_sense_hr_find(tiles_pal_t *hal, uint8_t instance)
{
    uint8_t addr = resolve_id(instance);
    if (!addr) return 0;

    if (hal->i2c_is_ready(hal->handle, addr) != 0)
        return 0;

    uint8_t id = 0;
    hal->i2c_read(hal->handle, addr, MAX86174A_REG_PART_ID, &id, 1);
    return (id == MAX86174A_PART_ID_VALUE) ? 1 : 0;
}

void tile_sense_hr_init(tiles_pal_t *hal, uint8_t instance,
                        tile_t *tile, const sense_hr_cfg_t *cfg)
{
    memzero(tile, sizeof(tile_t));
    tile->hal = hal;
    tile->id  = resolve_id(instance);

    if (!tile->id) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_hr: invalid instance");
        return;
    }

    if (hal->i2c_is_ready(hal->handle, tile->id) != 0) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_hr: device not responding");
        return;
    }

    if (hr_read(tile, MAX86174A_REG_PART_ID) != MAX86174A_PART_ID_VALUE) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "sense_hr: PART_ID mismatch");
        return;
    }

    hr_state_t *st = state_for(tile);
    memzero(st, sizeof(hr_state_t));
    for (uint8_t i = 0; i < SENSE_HR_FRAME_LEN; i++)
        st->frame[i] = SENSE_HR_NO_SAMPLE;

    /* Resolve config with defaults */
    uint16_t fps      = (cfg && cfg->fps)       ? cfg->fps       : 128;
    /* led 0 is "default" (green) here, like every other zero field; pick
     * the LED1 emitter with set_slot() after init. */
    uint8_t  led      = (cfg && cfg->led)       ? cfg->led       : SENSE_HR_LED_GREEN;
    uint32_t led_ua   = (cfg && cfg->led_ua)    ? cfg->led_ua    : 1000;
    st->def_tint      = (cfg && cfg->tint)      ? cfg->tint      : SENSE_HR_TINT_117US;
    st->def_aver      = (cfg && cfg->aver)      ? cfg->aver      : SENSE_HR_AVER_4;
    st->def_adc_rge   = (cfg && cfg->adc_range) ? cfg->adc_range : SENSE_HR_ADC_32UA;
    st->def_pdsel     = (cfg)                   ? cfg->pdsel     : SENSE_HR_PD_BOTH_PPG1;
    uint8_t ppg2      = (cfg)                   ? cfg->ppg2_enable : 0;
    st->int_pin       = (cfg)                   ? cfg->int_pin   : 0;
    st->on_event      = (cfg)                   ? cfg->on_event  : 0;
    st->event_ctx     = (cfg)                   ? cfg->event_ctx : 0;

    /* Known state: soft reset (self-clearing), then wait for the 200 µs
     * internal power-up plus margin. No user NVM on this part. */
    hr_write(tile, MAX86174A_REG_SYS_CFG1, MAX86174A_CFG1_RESET);
    hal->delay_ms(2);

    /* LED4_DRV/TRIG pin is the second green emitter on this tile; the
     * internal frame oscillator (SYNC_MODE 0) also requires this bit.
     * INTB_OCFG = 0: open-drain active-low, so the host's pull-up sets the
     * level (push-pull drives only to 1.8 V, too low for many 3.3 V MCUs). */
    hr_write(tile, MAX86174A_REG_PIN_OUT, 0x01);
    /* INTB enabled, cleared on status/FIFO read; TRIG edge irrelevant. */
    hr_write(tile, MAX86174A_REG_PIN_FUNC, (uint8_t)(0x1 << 1));

    st->sys_cfg1 = ppg2 ? 0x00 : MAX86174A_CFG1_PPG2_PWRDN;
    hr_write(tile, MAX86174A_REG_SYS_CFG1, st->sys_cfg1);

    st->sys_cfg3 = 0x00;   /* analog ALC on, no proximity, computed samples */
    hr_write(tile, MAX86174A_REG_SYS_CFG3, st->sys_cfg3);

    /* FIFO: roll over when full (count stays live), status cleared by
     * FIFO reads as well as STATUS1 reads. Watermark stays at POR (0x7F). */
    hr_write(tile, MAX86174A_REG_FIFO_CFG2,
             MAX86174A_FIFO_STAT_CLR | MAX86174A_FIFO_RO);

    /* Nothing drives INTB until the user picks sources. */
    hr_write(tile, MAX86174A_REG_INT_EN1, 0x00);
    hr_write(tile, MAX86174A_REG_INT_EN2, 0x00);

    tile_sense_hr_set_frame_rate(tile, fps);

    /* Slot 0: the single-wavelength default. Every slot starts from the
     * tile-wide defaults so later set_slot() calls inherit them. */
    for (uint8_t s = 0; s < SENSE_HR_SLOTS; s++) {
        st->slot[s].tint    = st->def_tint;
        st->slot[s].aver    = st->def_aver;
        st->slot[s].adc_rge = st->def_adc_rge;
        st->slot[s].pdsel   = st->def_pdsel;
    }
    tile_sense_hr_set_slot(tile, 0, (sense_hr_led_t)led, led_ua);

    /* Frame engine stays off until start(). */
    hr_write(tile, MAX86174A_REG_SYS_CFG2, 0x00);

    /* Optional INTB wiring: open-drain active-low → falling edge. */
    if (st->int_pin && hal->gpio_irq_enable) {
        void (*isr)(void *) = (instance == 0) ? _intb_isr_0 : _intb_isr_1;
        hal->gpio_irq_enable(hal->handle, st->int_pin,
                             TILES_GPIO_EDGE_FALLING, isr, 0);
    }

    /* Clear the PWR_RDY latch from power-up so the first event is real. */
    (void)hr_read(tile, MAX86174A_REG_STATUS1);

    tile->state = TILE_STATE_READY;
}

void tile_sense_hr_sleep(tile_t *tile)
{
    hr_state_t *st = state_for(tile);
    st->sys_cfg1 |= MAX86174A_CFG1_SHDN;
    hr_write(tile, MAX86174A_REG_SYS_CFG1, st->sys_cfg1);
    tile->state = TILE_STATE_SLEEPING;
}

void tile_sense_hr_wake(tile_t *tile)
{
    hr_state_t *st = state_for(tile);
    st->sys_cfg1 &= (uint8_t)~MAX86174A_CFG1_SHDN;
    hr_write(tile, MAX86174A_REG_SYS_CFG1, st->sys_cfg1);
    tile->hal->delay_ms(1);
    tile->state = TILE_STATE_READY;
}

void tile_sense_hr_reset(tile_t *tile)
{
    hr_state_t *st = state_for(tile);
    if (st->int_pin && tile->hal->gpio_irq_disable)
        tile->hal->gpio_irq_disable(tile->hal->handle, st->int_pin);
    hr_write(tile, MAX86174A_REG_SYS_CFG1, MAX86174A_CFG1_RESET);
    tile->hal->delay_ms(2);
    st->running = 0;
    tile->state = TILE_STATE_NONE;
}

void tile_sense_hr_start(tile_t *tile)
{
    hr_state_t *st = state_for(tile);
    tile_sense_hr_flush_fifo(tile);
    st->running = 1;
    hr_write(tile, MAX86174A_REG_SYS_CFG2, st->enable_mask & 0x3F);
}

void tile_sense_hr_stop(tile_t *tile)
{
    hr_state_t *st = state_for(tile);
    st->running = 0;
    hr_write(tile, MAX86174A_REG_SYS_CFG2, 0x00);
}

/* ---- Event processing ---- */

void tile_sense_hr_process(tile_t *tile)
{
    if (tile->state != TILE_STATE_READY) return;

    hr_state_t *st = state_for(tile);
    if (st->int_pin && !st->int_flag)
        return;
    st->int_flag = 0;

    uint8_t status = hr_read(tile, MAX86174A_REG_STATUS1);
    if (status && st->on_event)
        st->on_event(tile, status, st->event_ctx);
}

void tile_sense_hr_on_event(tile_t *tile, sense_hr_event_cb_t cb, void *ctx)
{
    hr_state_t *st = state_for(tile);
    st->on_event  = cb;
    st->event_ctx = ctx;
}

void tile_sense_hr_set_interrupts(tile_t *tile, uint8_t mask)
{
    /* PWR_RDY (bit 0) is non-maskable and its enable bit is reserved. */
    hr_write(tile, MAX86174A_REG_INT_EN1, (uint8_t)(mask & 0xFE));
}

/* ---- Configuration ---- */

uint16_t tile_sense_hr_set_frame_rate(tile_t *tile, uint16_t fps)
{
    hr_state_t *st = state_for(tile);
    if (fps == 0) fps = 1;
    uint32_t div = (SENSE_HR_FR_CLK_HZ + fps / 2u) / fps;
    if (div < SENSE_HR_FR_DIV_MIN) div = SENSE_HR_FR_DIV_MIN;
    if (div > SENSE_HR_FR_DIV_MAX) div = SENSE_HR_FR_DIV_MAX;
    st->fr_div = (uint16_t)div;
    hr_write(tile, MAX86174A_REG_FR_CLK_DIV_H, (uint8_t)((div >> 8) & 0x7F));
    hr_write(tile, MAX86174A_REG_FR_CLK_DIV_L, (uint8_t)(div & 0xFF));
    return tile_sense_hr_get_frame_rate(tile);
}

uint16_t tile_sense_hr_get_frame_rate(tile_t *tile)
{
    hr_state_t *st = state_for(tile);
    if (st->fr_div == 0) return 0;
    return (uint16_t)((SENSE_HR_FR_CLK_HZ + st->fr_div / 2u) / st->fr_div);
}

void tile_sense_hr_set_slot(tile_t *tile, uint8_t slot, sense_hr_led_t led,
                            uint32_t led_ua)
{
    if (slot >= SENSE_HR_SLOTS) return;
    hr_state_t *st = state_for(tile);
    hr_slot_t  *s  = &st->slot[slot];
    s->led     = (uint8_t)led & 0x03;
    s->ambient = 0;
    led_quantise(led_ua, &s->led_rge, &s->pa);
    slot_write_all(tile, slot);
    st->enable_mask |= (uint8_t)(1u << slot);
    apply_enables(tile);
}

void tile_sense_hr_set_ambient_slot(tile_t *tile, uint8_t slot)
{
    if (slot >= SENSE_HR_SLOTS) return;
    hr_state_t *st = state_for(tile);
    hr_slot_t  *s  = &st->slot[slot];
    s->ambient = 1;
    s->pa      = 0;
    slot_write_all(tile, slot);
    st->enable_mask |= (uint8_t)(1u << slot);
    apply_enables(tile);
}

void tile_sense_hr_enable_slot(tile_t *tile, uint8_t slot, uint8_t enable)
{
    if (slot >= SENSE_HR_SLOTS) return;
    hr_state_t *st = state_for(tile);
    if (enable) st->enable_mask |= (uint8_t)(1u << slot);
    else        st->enable_mask &= (uint8_t)~(1u << slot);
    apply_enables(tile);
}

void tile_sense_hr_set_led_current(tile_t *tile, uint8_t slot, uint32_t led_ua)
{
    if (slot >= SENSE_HR_SLOTS) return;
    hr_slot_t *s = &state_for(tile)->slot[slot];
    led_quantise(led_ua, &s->led_rge, &s->pa);
    slot_write_cfg2(tile, slot);
    slot_write_current(tile, slot);
}

uint32_t tile_sense_hr_get_led_current(tile_t *tile, uint8_t slot)
{
    if (slot >= SENSE_HR_SLOTS) return 0;
    hr_slot_t *s = &state_for(tile)->slot[slot];
    return (uint32_t)s->pa * led_lsb_ua[s->led_rge & 0x03];
}

void tile_sense_hr_set_integration(tile_t *tile, uint8_t slot, sense_hr_tint_t tint)
{
    if (slot >= SENSE_HR_SLOTS) return;
    state_for(tile)->slot[slot].tint = (uint8_t)tint & 0x03;
    slot_write_cfg1(tile, slot);
}

void tile_sense_hr_set_averaging(tile_t *tile, uint8_t slot, sense_hr_aver_t aver)
{
    if (slot >= SENSE_HR_SLOTS) return;
    state_for(tile)->slot[slot].aver = (uint8_t)aver & 0x07;
    slot_write_cfg1(tile, slot);
}

void tile_sense_hr_set_adc_range(tile_t *tile, uint8_t slot, sense_hr_adc_range_t range)
{
    if (slot >= SENSE_HR_SLOTS) return;
    state_for(tile)->slot[slot].adc_rge = (uint8_t)range & 0x03;
    slot_write_cfg2(tile, slot);
}

void tile_sense_hr_set_dac_offset(tile_t *tile, uint8_t slot, uint8_t offset_ua)
{
    if (slot >= SENSE_HR_SLOTS) return;
    if (offset_ua > SENSE_HR_DACOFF_MAX_UA) offset_ua = SENSE_HR_DACOFF_MAX_UA;
    state_for(tile)->slot[slot].dacoff = (uint8_t)((offset_ua + 2) / 4);
    slot_write_cfg4(tile, slot);
}

void tile_sense_hr_set_photodiodes(tile_t *tile, uint8_t slot, sense_hr_pdsel_t pdsel)
{
    if (slot >= SENSE_HR_SLOTS) return;
    state_for(tile)->slot[slot].pdsel = (uint8_t)pdsel & 0x03;
    slot_write_cfg1(tile, slot);
}

void tile_sense_hr_set_ppg2_enabled(tile_t *tile, uint8_t enable)
{
    hr_state_t *st = state_for(tile);
    if (enable) st->sys_cfg1 &= (uint8_t)~MAX86174A_CFG1_PPG2_PWRDN;
    else        st->sys_cfg1 |= MAX86174A_CFG1_PPG2_PWRDN;
    hr_write(tile, MAX86174A_REG_SYS_CFG1, st->sys_cfg1);
}

/* Collect samples for ~ms on one slot running alone. Returns the number of
 * good PPG1 samples; mean DC and overflow-tag count via out params. */
static uint16_t probe_slot(tile_t *tile, uint8_t slot, uint16_t ms,
                           int32_t *dc_out, uint16_t *ovf_out)
{
    int64_t  sum = 0;
    uint32_t n   = 0;
    uint16_t ovf = 0;
    uint32_t words[64];

    for (uint16_t t = 0; t < ms / 25u; t++) {
        tile->hal->delay_ms(25);
        uint16_t got = fifo_read_words(tile, words, 64);
        for (uint16_t i = 0; i < got; i++) {
            if (fifo_word_is_filler(words[i])) continue;
            uint32_t data = words[i] & MAX86174A_FIFO_DATA_MASK;
            uint8_t  tag  = (uint8_t)((words[i] >> MAX86174A_FIFO_TAG_SHIFT) & 0x0F);
            if (tag == MAX86174A_FIFO_TAG_EXP_OVF || tag == MAX86174A_FIFO_TAG_ALC_OVF) {
                ovf++;
                continue;
            }
            if (tag != slot) continue;   /* PPG1 tag of this slot */
            sum += sign_extend_20(data);
            n++;
        }
    }
    *ovf_out = ovf;
    *dc_out  = n ? (int32_t)(sum / (int64_t)n) : 0;
    return (uint16_t)n;
}

static void run_alone(tile_t *tile, uint8_t slot)
{
    slot_write_all(tile, slot);
    hr_write(tile, MAX86174A_REG_SYS_CFG2, 0x00);
    tile_sense_hr_flush_fifo(tile);
    hr_write(tile, MAX86174A_REG_SYS_CFG2, (uint8_t)(1u << slot));
}

uint8_t tile_sense_hr_auto_gain(tile_t *tile, uint8_t slot, uint32_t max_ua)
{
    if (slot >= SENSE_HR_SLOTS) return 0;
    hr_state_t *st = state_for(tile);
    hr_slot_t  *s  = &st->slot[slot];
    if (s->ambient) return 0;
    if (max_ua == 0) max_ua = 10000;

    /* Work in the 32 mA range so one code is 125 µA throughout. */
    uint8_t pa_cap = (uint8_t)((max_ua > 255u * 125u) ? 255 : max_ua / 125u);
    if (pa_cap == 0) pa_cap = 1;

    uint8_t saved_mask = st->enable_mask;
    uint8_t was_running = st->running;
    st->running = 0;

    s->led_rge = 0;
    s->pa      = (uint8_t)((pa_cap < 8) ? pa_cap : 8);
    s->dacoff  = 0;
    s->adc_rge = 3;

    int32_t  dc  = 0;
    uint16_t ovf = 0;
    uint16_t n   = 0;

    /* Phase 1: push LED current toward the cap while the offset DAC can
     * still keep the DC pedestal inside the converter. */
    for (uint8_t iter = 0; iter < 14; iter++) {
        run_alone(tile, slot);
        n = probe_slot(tile, slot, 150, &dc, &ovf);

        if (ovf > 4 || (n && dc > (SENSE_HR_ADC_FULL_SCALE * 3) / 4)) {
            if (s->dacoff < 7)     s->dacoff++;
            else if (s->pa > 4)    s->pa = (uint8_t)((s->pa * 3) / 4);
            else break;
        } else if (n && s->pa < pa_cap) {
            uint16_t next = (uint16_t)(s->pa + (s->pa / 2) + 1);
            s->pa = (uint8_t)(next > pa_cap ? pa_cap : next);
        } else {
            break;
        }
    }

    /* Phase 2: cancel the pedestal, then tighten the range. Computed in
     * photocurrent (hundredths of a µA), which is what the DAC subtracts
     * and the range selects. */
    run_alone(tile, slot);
    n = probe_slot(tile, slot, 200, &dc, &ovf);
    uint8_t found = (n != 0);
    /* A sample is photocurrent minus the offset DAC [DS MEASx Config 4], so
     * the photocurrent is the residual plus whatever phase 1 already
     * cancelled. Choosing from the residual alone threw that away. */
    int64_t total_x100 = n ? ((int64_t)dc * adc_fs_ua[s->adc_rge] * 100) / SENSE_HR_ADC_FULL_SCALE
                             + (int64_t)dacoff_ua[s->dacoff] * 100
                           : 0;
    if (n && total_x100 > 0) {
        uint32_t i_x100 = (uint32_t)total_x100;
        uint8_t best = 0;
        for (uint8_t d = 0; d < 8; d++)
            if ((uint32_t)dacoff_ua[d] * 100u + 50u <= i_x100) best = d;
        s->dacoff = best;

        uint32_t resid_x100 = i_x100 - (uint32_t)dacoff_ua[best] * 100u;
        for (uint8_t r = 0; r < 4; r++) {
            if (resid_x100 * 10u < (uint32_t)adc_fs_ua[r] * 700u) {
                s->adc_rge = r;
                break;
            }
        }

        /* Verify; unwind one step at a time if that was too aggressive. */
        for (uint8_t guard = 0; guard < 4; guard++) {
            run_alone(tile, slot);
            n = probe_slot(tile, slot, 150, &dc, &ovf);
            if (n && ovf <= 4 && dc > 0 && dc < (SENSE_HR_ADC_FULL_SCALE * 3) / 4) break;
            if (s->adc_rge < 3)     s->adc_rge++;
            else if (s->dacoff > 0) s->dacoff--;
            else break;
        }
    }

    /* Leave the chip stopped with the final settings written. */
    hr_write(tile, MAX86174A_REG_SYS_CFG2, 0x00);
    slot_write_all(tile, slot);
    st->enable_mask = saved_mask;
    if (was_running) tile_sense_hr_start(tile);
    return found;
}

/* ---- Runtime ---- */

uint8_t tile_sense_hr_get_status(tile_t *tile)
{
    return hr_read(tile, MAX86174A_REG_STATUS1);
}

uint8_t tile_sense_hr_get_status2(tile_t *tile)
{
    return hr_read(tile, MAX86174A_REG_STATUS2);
}

uint8_t tile_sense_hr_frame_ready(tile_t *tile)
{
    hr_state_t *st = state_for(tile);
    return (fifo_count(tile) >= frame_samples(st)) ? 1 : 0;
}

uint16_t tile_sense_hr_get_fifo_count(tile_t *tile)
{
    return fifo_count(tile);
}

uint8_t tile_sense_hr_get_fifo_overflow(tile_t *tile)
{
    return (uint8_t)(hr_read(tile, MAX86174A_REG_FIFO_CNT1) & 0x7F);
}

void tile_sense_hr_flush_fifo(tile_t *tile)
{
    hr_write(tile, MAX86174A_REG_FIFO_CFG2,
             MAX86174A_FIFO_FLUSH | MAX86174A_FIFO_STAT_CLR | MAX86174A_FIFO_RO);
}

void tile_sense_hr_set_fifo_watermark(tile_t *tile, uint16_t samples)
{
    if (samples == 0) samples = 1;
    if (samples > SENSE_HR_FIFO_DEPTH) samples = SENSE_HR_FIFO_DEPTH;
    /* Register holds the number of FREE slots at which A_FULL asserts. */
    hr_write(tile, MAX86174A_REG_FIFO_CFG1,
             (uint8_t)(SENSE_HR_FIFO_DEPTH - samples));
}

uint8_t tile_sense_hr_read_frame(tile_t *tile, int32_t *frame)
{
    hr_state_t *st = state_for(tile);
    uint8_t ok = 0;
    if (fifo_count(tile) >= frame_samples(st)) {
        drain_into_frame(tile);
        ok = 1;
    }
    for (uint8_t i = 0; i < SENSE_HR_FRAME_LEN; i++)
        frame[i] = st->frame[i];
    return ok;
}

static int32_t cached_sample(tile_t *tile, uint8_t slot, uint8_t ch)
{
    if (slot >= SENSE_HR_SLOTS) return SENSE_HR_NO_SAMPLE;
    hr_state_t *st = state_for(tile);
    if (fifo_count(tile) >= frame_samples(st))
        drain_into_frame(tile);
    return st->frame[slot * SENSE_HR_CHANNELS + ch];
}

int32_t tile_sense_hr_read_ppg(tile_t *tile, uint8_t slot)
{
    return cached_sample(tile, slot, 0);
}

int32_t tile_sense_hr_read_ppg2(tile_t *tile, uint8_t slot)
{
    return cached_sample(tile, slot, 1);
}

int32_t tile_sense_hr_read_photocurrent_na(tile_t *tile, uint8_t slot)
{
    int32_t counts = cached_sample(tile, slot, 0);
    if (counts == SENSE_HR_NO_SAMPLE) return SENSE_HR_NO_SAMPLE;
    hr_slot_t *s = &state_for(tile)->slot[slot];
    /* full scale (µA) × 1000 nA/µA / 524287 counts, plus the DAC offset */
    int64_t na = ((int64_t)counts * adc_fs_ua[s->adc_rge & 0x03] * 1000)
                 / SENSE_HR_ADC_FULL_SCALE;
    na += (int64_t)dacoff_ua[s->dacoff & 0x07] * 1000;
    return (int32_t)na;
}

uint16_t tile_sense_hr_get_overflow_count(tile_t *tile)
{
    return state_for(tile)->ovf_count;
}

uint16_t tile_sense_hr_read_fifo_raw(tile_t *tile, uint32_t *buf, uint16_t count)
{
    return fifo_read_words(tile, buf, count);
}

/* ---- Proximity / threshold ---- */

void tile_sense_hr_set_threshold(tile_t *tile, uint8_t slot, uint8_t lower, uint8_t upper)
{
    if (slot >= SENSE_HR_SLOTS) return;
    uint8_t sel = hr_read(tile, MAX86174A_REG_THRESH_SEL);
    sel = (uint8_t)((sel & 0x70) | ((slot + 1) & 0x07));
    hr_write(tile, MAX86174A_REG_THRESH_SEL, sel);
    uint8_t hyst = hr_read(tile, MAX86174A_REG_THRESH_HYST);
    hr_write(tile, MAX86174A_REG_THRESH_HYST, (uint8_t)(hyst & ~(1u << 6)));  /* PPG1 */
    hr_write(tile, MAX86174A_REG_THRESH1_HI, upper);
    hr_write(tile, MAX86174A_REG_THRESH1_LO, lower);
}

void tile_sense_hr_set_proximity_mode(tile_t *tile, uint8_t enable, uint8_t lower)
{
    hr_state_t *st = state_for(tile);
    if (enable) {
        hr_write(tile, MAX86174A_REG_THRESH1_LO, lower);
        st->sys_cfg3 |= MAX86174A_CFG3_PROX_AUTO | MAX86174A_CFG3_PROX_DATA_EN;
    } else {
        st->sys_cfg3 &= (uint8_t)~(MAX86174A_CFG3_PROX_AUTO | MAX86174A_CFG3_PROX_DATA_EN);
    }
    hr_write(tile, MAX86174A_REG_SYS_CFG3, st->sys_cfg3);
}

/* ---- Advanced ---- */

uint8_t tile_sense_hr_read_reg(tile_t *tile, uint8_t reg)
{
    return hr_read(tile, reg);
}

void tile_sense_hr_write_reg(tile_t *tile, uint8_t reg, uint8_t val)
{
    hr_write(tile, reg, val);
}
