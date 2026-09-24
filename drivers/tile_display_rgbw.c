/**
 * @file   tile_display_rgbw.c
 * @brief  Disp.RGBW (LP5811) — platform-agnostic driver.
 */

#include "tile_display_rgbw.h"
#include <stddef.h>

/* ---- Instance table ---- */

static const uint8_t id_table[] = { LP5811_I2C_ADDR_DEFAULT };
#define NUM_INSTANCES  (sizeof(id_table) / sizeof(id_table[0]))

static uint8_t resolve_id(uint8_t instance)
{
    return (instance < NUM_INSTANCES) ? id_table[instance] : 0;
}

/* ---- Private helpers ---- */

static void lp_write(tile_t *tile, uint8_t reg, uint8_t val)
{
    tile->hal->i2c_write(tile->hal->handle, tile->id, reg, &val, 1);
}

static uint8_t lp_read(tile_t *tile, uint8_t reg)
{
    uint8_t val = 0;
    tile->hal->i2c_read(tile->hal->handle, tile->id, reg, &val, 1);
    return val;
}

/* The LP5811's register address is 10 bits wide. The lower 8 bits
 * sit in the I2C "register byte"; the upper 2 bits ("page") are
 * encoded into bits[1:0] of the 7-bit chip address. Status registers
 * we care about live on page 3 (addresses 0x300+), so we read them
 * via tile->id | 0x03. See LP5811 datasheet §7.5 — Programming. */
static uint8_t lp_read_page(tile_t *tile, uint8_t page, uint8_t reg)
{
    uint8_t val = 0;
    uint8_t paged_id = (tile->id & ~0x03) | (page & 0x03);
    tile->hal->i2c_read(tile->hal->handle, paged_id, reg, &val, 1);
    return val;
}

/** Latch any Dev_Config_* writes — required by the chip per
 *  datasheet §2.4.1 (CMD_Update). Writing 0x55 to 0x10 commits
 *  registers 0x001..0x00B. */
static void lp_commit(tile_t *tile)
{
    lp_write(tile, LP5811_REG_CMD_UPDATE, 0x55);
}

/* ---- Public API ---- */

uint8_t tile_display_rgbw_find(tiles_pal_t *hal, uint8_t instance)
{
    uint8_t addr = resolve_id(instance);
    if (!addr) return 0;
    return hal->i2c_is_ready(hal->handle, addr) == 0;
}

void tile_display_rgbw_init(tiles_pal_t *hal, uint8_t instance, tile_t *tile,
                         const disp_rgbw_cfg_t *cfg)
{
    (void)cfg;  /* Reserved for future use */
    for (uint8_t i = 0; i < sizeof(tile_t); i++)
        ((uint8_t *)tile)[i] = 0;

    tile->hal = hal;
    tile->id  = resolve_id(instance);

    if (!tile->id) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "disp_rgbw: invalid instance");
        return;
    }

    if (hal->i2c_is_ready(hal->handle, tile->id) != 0) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "disp_rgbw: device not found");
        return;
    }

    /* Enable chip */
    lp_write(tile, LP5811_REG_CHIP_EN, 0x01);
    hal->delay_ms(2);

    /* Verify chip is alive */
    uint8_t cfg2 = lp_read(tile, LP5811_REG_CONFIG_2);
    if (cfg2 != LP5811_CONFIG_2_DEFAULT) {
        tile->state = TILE_STATE_ERROR;
        TILE_ON_ERROR(tile, "disp_rgbw: CONFIG_2 verify failed");
        return;
    }

    /* Dev_Config_12: clamp default (vmid_sel=0, clamp_sel=0, clamp_dis=0),
     * lod_action=1 (open shuts down sink), lsd_action=0 (short reports
     * only — driver-level choice; firmware can opt-in via set_short_shutdown),
     * lsd_threshold=3 (0.65 × VOUT, most permissive). Latched by the
     * first commit of the boost ramp below. */
    lp_write(tile, LP5811_REG_CONFIG_12, 0x0B);

    /* Boost to 4.5 V, max current 51 mA — RAMPED, not slammed.
     *
     * Dev_Config_0 layout (datasheet §Dev_Config_0): bits[5:1] =
     * boost_vout, 3.0 V + 0.1 V × code; bit[0] = max_current
     * (1 = 51 mA full-scale). Reset default is code 0 → the boost
     * idles at 3.0 V once the chip is enabled.
     *
     * Committing the 4.5 V target in one step (pre-v2.3.0 behavior)
     * makes the boost slew 3.0 → 4.5 V at once; the inrush can
     * collapse a marginal supply (long leads, probe fixtures, loaded
     * USB rails) and brown-out-reset the host MCU — 100% reproducible
     * on the 2026-07 Display.RGBW panel bring-up rig, where stepping
     * 0.1 V per commit was 100% reliable. Ramp cost at the proven
     * 100 ms/step is ~1.6 s of init time (see the header note on
     * LP5811_BOOST_RAMP_STEP_MS for tuning). */
    for (uint8_t code = 0; code <= LP5811_BOOST_VOUT_CODE_4V5; code++) {
        lp_write(tile, LP5811_REG_CONFIG_0,
                 (uint8_t)((code << 1) | LP5811_CONFIG_0_MC_51MA));
        lp_commit(tile);
        hal->delay_ms(LP5811_BOOST_RAMP_STEP_MS);
    }

    /* Enable all 4 LED channels */
    lp_write(tile, LP5811_REG_LED_EN, 0x0F);

    /* Default current limits: conservative, NOT half scale.
     *
     * ~1 mA per channel at full PWM. The old 0x80 (~25.6 mA) was both
     * painful to look at on this tile's bare LED and about 2x above the
     * level at which an abrupt switch-on browns out a bench supply.
     * See LP5811_DC_DEFAULT for the measurements; set_current() raises
     * it when the application actually wants the light. */
    lp_write(tile, LP5811_REG_DC_0, LP5811_DC_DEFAULT);
    lp_write(tile, LP5811_REG_DC_1, LP5811_DC_DEFAULT);
    lp_write(tile, LP5811_REG_DC_2, LP5811_DC_DEFAULT);
    lp_write(tile, LP5811_REG_DC_3, LP5811_DC_DEFAULT);

    /* Autonomous mode drives from Auto_DC_x (0x50-0x53), not Manual_DC,
     * and those reset to 0 (register map SNVU925 §2.10): without this an
     * AEU animation (breathe_auto etc.) runs at zero current. Mirror the
     * same safe default. */
    lp_write(tile, LP5811_REG_AUTO_DC_0, LP5811_DC_DEFAULT);
    lp_write(tile, LP5811_REG_AUTO_DC_1, LP5811_DC_DEFAULT);
    lp_write(tile, LP5811_REG_AUTO_DC_2, LP5811_DC_DEFAULT);
    lp_write(tile, LP5811_REG_AUTO_DC_3, LP5811_DC_DEFAULT);

    tile->state = TILE_STATE_READY;
}

void tile_display_rgbw_set(tile_t *tile, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    /* Channel mapping: LED0=R, LED1=W, LED2=G, LED3=B (tile schematic) */
    lp_write(tile, LP5811_REG_PWM_0, r);
    lp_write(tile, LP5811_REG_PWM_1, w);
    lp_write(tile, LP5811_REG_PWM_2, g);
    lp_write(tile, LP5811_REG_PWM_3, b);
}

void tile_display_rgbw_off(tile_t *tile)
{
    lp_write(tile, LP5811_REG_PWM_0, 0);
    lp_write(tile, LP5811_REG_PWM_1, 0);
    lp_write(tile, LP5811_REG_PWM_2, 0);
    lp_write(tile, LP5811_REG_PWM_3, 0);
}

void tile_display_rgbw_set_current(tile_t *tile, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    lp_write(tile, LP5811_REG_DC_0, r);
    lp_write(tile, LP5811_REG_DC_1, w);
    lp_write(tile, LP5811_REG_DC_2, g);
    lp_write(tile, LP5811_REG_DC_3, b);
    /* Autonomous-mode current follows the manual setting (see init). */
    lp_write(tile, LP5811_REG_AUTO_DC_0, r);
    lp_write(tile, LP5811_REG_AUTO_DC_1, w);
    lp_write(tile, LP5811_REG_AUTO_DC_2, g);
    lp_write(tile, LP5811_REG_AUTO_DC_3, b);
}

void tile_display_rgbw_set_max_current(tile_t *tile, disp_rgbw_max_current_t mode)
{
    /* Read-modify-write Dev_Config_0 — preserves boost_vout. Only bit 0
     * is the MC selector. */
    uint8_t cfg0 = lp_read(tile, LP5811_REG_CONFIG_0);
    cfg0 = (cfg0 & ~0x01u) | (mode ? 0x01u : 0x00u);
    lp_write(tile, LP5811_REG_CONFIG_0, cfg0);
    lp_commit(tile);
}

void tile_display_rgbw_read_faults(tile_t *tile, disp_rgbw_faults_t *out)
{
    if (!out) return;
    /* Zero on entry so partial bus failures don't leak garbage. */
    out->open_mask        = 0;
    out->short_mask       = 0;
    out->thermal_shutdown = 0;
    out->config_error     = 0;

    uint8_t tsd = lp_read_page(tile, 3, LP5811_REG_TSD_STATUS);
    uint8_t lod = lp_read_page(tile, 3, LP5811_REG_LOD_STATUS_0);
    uint8_t lsd = lp_read_page(tile, 3, LP5811_REG_LSD_STATUS_0);

    out->config_error     = (tsd & 0x01) ? 1 : 0;
    out->thermal_shutdown = (tsd & 0x02) ? 1 : 0;
    out->open_mask  = lod & 0x0F;
    out->short_mask = lsd & 0x0F;
}

void tile_display_rgbw_clear_faults(tile_t *tile)
{
    /* Fault_Clear (0x22) is W1C: bit2=tsd, bit1=lsd, bit0=lod. */
    lp_write(tile, LP5811_REG_FAULT_CLEAR, 0x07);
}

void tile_display_rgbw_set_short_threshold(tile_t *tile,
                                           disp_rgbw_lsd_threshold_t threshold)
{
    uint8_t cfg12 = lp_read(tile, LP5811_REG_CONFIG_12);
    cfg12 = (cfg12 & ~0x03u) | ((uint8_t)threshold & 0x03u);
    lp_write(tile, LP5811_REG_CONFIG_12, cfg12);
    lp_commit(tile);
}

void tile_display_rgbw_set_short_shutdown(tile_t *tile, uint8_t enabled)
{
    /* Dev_Config_12 bit 2 = lsd_action. */
    uint8_t cfg12 = lp_read(tile, LP5811_REG_CONFIG_12);
    cfg12 = (cfg12 & ~0x04u) | (enabled ? 0x04u : 0x00u);
    lp_write(tile, LP5811_REG_CONFIG_12, cfg12);
    lp_commit(tile);
}

void tile_display_rgbw_set_open_shutdown(tile_t *tile, uint8_t enabled)
{
    /* Dev_Config_12 bit 3 = lod_action. */
    uint8_t cfg12 = lp_read(tile, LP5811_REG_CONFIG_12);
    cfg12 = (cfg12 & ~0x08u) | (enabled ? 0x08u : 0x00u);
    lp_write(tile, LP5811_REG_CONFIG_12, cfg12);
    lp_commit(tile);
}

void tile_display_rgbw_sleep(tile_t *tile)
{
    lp_write(tile, LP5811_REG_CHIP_EN, 0x00);
    tile->state = TILE_STATE_SLEEPING;
}

void tile_display_rgbw_wake(tile_t *tile)
{
    lp_write(tile, LP5811_REG_CHIP_EN, 0x01);
    tile->hal->delay_ms(2);
    tile->state = TILE_STATE_READY;
}

void tile_display_rgbw_reset(tile_t *tile)
{
    lp_write(tile, LP5811_REG_RESET, 0x66);  /* per datasheet §2.7.1: write 0x66 */
    tile->hal->delay_ms(2);
    tile->state = TILE_STATE_NONE;
}

/* ---- Tier-2 idiomatic helpers ---- */

void tile_display_rgbw_set_color(tile_t *tile, uint8_t r, uint8_t g, uint8_t b)
{
    tile_display_rgbw_set(tile, r, g, b, 0);
}

void tile_display_rgbw_pulse(tile_t *tile, uint8_t r, uint8_t g, uint8_t b,
                             uint16_t ms)
{
    tile_display_rgbw_set(tile, r, g, b, 0);
    tile->hal->delay_ms(ms);
    tile_display_rgbw_off(tile);
}

void tile_display_rgbw_breathe(tile_t *tile, uint8_t r, uint8_t g, uint8_t b,
                               uint16_t period_ms)
{
    /* Software ramp — 32 steps up, 32 steps down (64 total).
     * breathe_auto() is the on-chip (AEU) equivalent. */
    const uint8_t STEPS = 32;
    uint16_t step_ms = period_ms / (uint16_t)(STEPS * 2u);
    if (step_ms == 0) step_ms = 1;  /* clamp — too-short period falls back to choppy */

    /* Ramp up: 0 → peak. */
    for (uint8_t i = 1; i <= STEPS; i++) {
        uint8_t scale_r = (uint8_t)(((uint16_t)r * i) / STEPS);
        uint8_t scale_g = (uint8_t)(((uint16_t)g * i) / STEPS);
        uint8_t scale_b = (uint8_t)(((uint16_t)b * i) / STEPS);
        tile_display_rgbw_set(tile, scale_r, scale_g, scale_b, 0);
        tile->hal->delay_ms(step_ms);
    }

    /* Ramp down: peak → 0. */
    for (uint8_t i = STEPS; i > 0; i--) {
        uint8_t j = (uint8_t)(i - 1u);
        uint8_t scale_r = (uint8_t)(((uint16_t)r * j) / STEPS);
        uint8_t scale_g = (uint8_t)(((uint16_t)g * j) / STEPS);
        uint8_t scale_b = (uint8_t)(((uint16_t)b * j) / STEPS);
        tile_display_rgbw_set(tile, scale_r, scale_g, scale_b, 0);
        tile->hal->delay_ms(step_ms);
    }

    /* Guarantee fully off at end (rounding could leave a sliver). */
    tile_display_rgbw_off(tile);
}

void tile_display_rgbw_flash(tile_t *tile, uint8_t r, uint8_t g, uint8_t b,
                             uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        tile_display_rgbw_set(tile, r, g, b, 0);
        tile->hal->delay_ms(100);
        tile_display_rgbw_off(tile);
        tile->hal->delay_ms(100);
    }
}

uint8_t tile_display_rgbw_is_faulted(tile_t *tile)
{
    disp_rgbw_faults_t f;
    tile_display_rgbw_read_faults(tile, &f);
    if (f.open_mask || f.short_mask || f.thermal_shutdown || f.config_error) {
        return 1;
    }
    return 0;
}

/* ============================================================== */
/* Autonomous animation engine (AEU)                               */
/* ============================================================== */

/* Shared 16-entry time table (ms) for AEU slope times T1..T4 and the
 * Auto_Pause start/end fields. Code 0 = no time (instant); the rest are
 * the datasheet's geometric-ish ladder up to 8.05 s. */
static const uint16_t lp_time_ms[16] = {
    0, 90, 180, 360, 540, 800, 1070, 1520,
    2060, 2500, 3040, 4020, 5010, 5990, 7060, 8050,
};

uint8_t tile_display_rgbw_ms_to_slope(uint16_t ms)
{
    /* Nearest-code lookup. Returns 0 for ms==0, else the closest entry. */
    uint8_t best = 0;
    uint16_t best_err = 0xFFFF;
    for (uint8_t i = 0; i < 16; i++) {
        uint16_t err = (ms > lp_time_ms[i]) ? (ms - lp_time_ms[i])
                                            : (uint16_t)(lp_time_ms[i] - ms);
        if (err < best_err) {
            best_err = err;
            best = i;
        }
    }
    return best;
}

void tile_display_rgbw_set_autonomous(tile_t *tile, uint8_t channel, uint8_t enabled)
{
    if (channel > 3) return;
    uint8_t v = lp_read(tile, LP5811_REG_CONFIG_3);
    if (enabled) v |= (uint8_t)(1u << channel);
    else         v &= (uint8_t)~(1u << channel);
    lp_write(tile, LP5811_REG_CONFIG_3, v);
}

void tile_display_rgbw_set_aeu(tile_t *tile, uint8_t channel, uint8_t aeu,
                               const display_rgbw_aeu_t *prog)
{
    if (channel > 3 || aeu < 1 || aeu > 3 || prog == NULL) return;

    uint8_t base = (uint8_t)(LP5811_LED_ANIM_BASE(channel) + LP5811_AEU_OFFSET(aeu));

    /* PWM1..PWM5 keyframe levels. */
    for (uint8_t i = 0; i < 5; i++) {
        lp_write(tile, (uint8_t)(base + i), prog->pwm[i]);
    }
    /* T12: t2 in [7:4], t1 in [3:0].  T34: t4 in [7:4], t3 in [3:0]. */
    lp_write(tile, (uint8_t)(base + 5),
             (uint8_t)(((prog->t[1] & 0x0F) << 4) | (prog->t[0] & 0x0F)));
    lp_write(tile, (uint8_t)(base + 6),
             (uint8_t)(((prog->t[3] & 0x0F) << 4) | (prog->t[2] & 0x0F)));
    /* Playback: pt in [1:0], 3 = infinite. */
    lp_write(tile, (uint8_t)(base + 7), (uint8_t)(prog->repeats & 0x03));
}

void tile_display_rgbw_set_animation(tile_t *tile, uint8_t channel, uint8_t num_aeu,
                                     uint8_t pause_start, uint8_t pause_end, uint8_t repeats)
{
    if (channel > 3) return;
    if (num_aeu < 1) num_aeu = 1;
    if (num_aeu > 3) num_aeu = 3;

    uint8_t base = LP5811_LED_ANIM_BASE(channel);

    /* Auto_Pause: tp_ts in [7:4] (start), tp_te in [3:0] (end). */
    lp_write(tile, base,
             (uint8_t)(((pause_start & 0x0F) << 4) | (pause_end & 0x0F)));

    /* Auto_Playback: aeu_num in [5:4] (0=AEU1, 1=AEU1+2, 2=AEU1+2+3),
     * pt in [3:0] (0-14 repeats, Fh = infinite). */
    uint8_t aeu_num = (uint8_t)(num_aeu - 1);  /* 1→0, 2→1, 3→2 */
    lp_write(tile, (uint8_t)(base + 1),
             (uint8_t)(((aeu_num & 0x03) << 4) | (repeats & 0x0F)));
}

void tile_display_rgbw_set_exp_dimming(tile_t *tile, uint8_t channel, uint8_t enabled)
{
    if (channel > 3) return;
    uint8_t v = lp_read(tile, LP5811_REG_CONFIG_5);
    if (enabled) v |= (uint8_t)(1u << channel);
    else         v &= (uint8_t)~(1u << channel);
    lp_write(tile, LP5811_REG_CONFIG_5, v);
}

void tile_display_rgbw_set_phase_align(tile_t *tile, uint8_t channel, uint8_t mode)
{
    if (channel > 3) return;
    /* Two bits per channel in Dev_Config_7. */
    uint8_t shift = (uint8_t)(channel * 2);
    uint8_t v = lp_read(tile, LP5811_REG_CONFIG_7);
    v &= (uint8_t)~(0x03 << shift);
    v |= (uint8_t)((mode & 0x03) << shift);
    lp_write(tile, LP5811_REG_CONFIG_7, v);
}

void tile_display_rgbw_update(tile_t *tile)
{
    lp_commit(tile);  /* CMD_Update = 0x55 */
}

void tile_display_rgbw_animate_start(tile_t *tile)
{
    lp_write(tile, LP5811_REG_CMD_START, 0xFF);
}

void tile_display_rgbw_animate_stop(tile_t *tile)
{
    lp_write(tile, LP5811_REG_CMD_STOP, 0xAA);
}

void tile_display_rgbw_animate_pause(tile_t *tile)
{
    lp_write(tile, LP5811_REG_CMD_PAUSE, 0x33);
}

void tile_display_rgbw_animate_continue(tile_t *tile)
{
    lp_write(tile, LP5811_REG_CMD_CONTINUE, 0xCC);
}

void tile_display_rgbw_breathe_auto(tile_t *tile, uint8_t channel, uint8_t peak,
                                    uint16_t period_ms, uint8_t repeats)
{
    if (channel > 3) return;

    /* Symmetric ramp 0 → peak → 0 across one AEU. The five keyframes are
     * 0, peak/2, peak, peak/2, 0; each of the four legs takes a quarter of
     * the period. */
    uint8_t leg = tile_display_rgbw_ms_to_slope((uint16_t)(period_ms / 4u));
    display_rgbw_aeu_t prog = {
        .pwm = { 0, (uint8_t)(peak / 2), peak, (uint8_t)(peak / 2), 0 },
        .t   = { leg, leg, leg, leg },
        .repeats = 3,  /* AEU loops infinitely; whole-pattern repeat gates it */
    };

    tile_display_rgbw_set_aeu(tile, channel, 1, &prog);
    tile_display_rgbw_set_animation(tile, channel, 1, 0, 0, repeats);
    tile_display_rgbw_set_exp_dimming(tile, channel, 1);  /* eye-friendly */
    tile_display_rgbw_set_autonomous(tile, channel, 1);
    tile_display_rgbw_update(tile);
    tile_display_rgbw_animate_start(tile);
}
