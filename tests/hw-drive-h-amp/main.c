/**
 * hw-drive-h-amp — Drive.H output-amplitude characterization.
 *
 * Why: driver v4.1 defaults programmed RATED_VOLTAGE=0x1A /
 * OD_CLAMP=0x25 (a 0.7 Vrms actuator) and ran open-loop, where
 * OD_CLAMP alone sets the full-scale output (DRV2605 datasheet
 * §7.5.2 / §7.5.8.1.1) — with 5 V on V_MOTOR the actuator still only
 * ever saw ~0.7 V. Driver v4.2 defaults to closed-loop 1.8 Vrms;
 * this staircase remains useful for characterizing an unknown LRA.
 *
 * This test re-inits the driver in closed-loop LRA mode (auto-
 * resonance tracking) and steps the drive level up through
 * 0.7 → 1.0 → 1.8 → 2.0 Vrms, auto-calibrating at each step and
 * playing a feel sequence (2 clicks + 600 ms buzz), so an unknown
 * coin LRA (Vybronix class, typically 2.0 Vrms rated) can be
 * characterized by feel without overdriving it. It then settles at
 * the strongest level that calibrated cleanly and heartbeat-clicks
 * every 3 s.
 *
 * Core.ST.L4, clock=max, ROM-DFU bootloader
 *   I2C1 @ 400 kHz — Disp.RGBW (status LED, optional)
 *   I2C3 @ 400 kHz — Drive.H   (falls back to I2C1 if not found)
 *
 * LED codes:
 *   Yellow  = stepping through levels
 *   Green   = settled on final config
 *   Red     = DRV2605 not found / init failed
 */

#include "core.h"
#include "core_usb.h"
#include "core_tiles.h"
#include "tile_display_rgbw.h"
#include "tile_drive_h.h"

typedef struct {
    const char *label;
    uint8_t rated_v;   /* RATED_VOLTAGE (0x16), closed-loop reference */
    uint8_t od_clamp;  /* OD_CLAMP (0x17), peak ceiling */
} amp_level_t;

/* 0.5-1.8 Vrms rows from the tile_drive_h.h config table; 2.0 Vrms
 * extrapolated from the same scale for Vybronix-class coin LRAs. */
static const amp_level_t levels[] = {
    { "0.7 Vrms (driver default)", 0x1A, 0x25 },
    { "1.0 Vrms",                  0x26, 0x36 },
    { "1.8 Vrms",                  0x56, 0x8C },
    { "2.0 Vrms",                  0x60, 0x9C },
};
#define N_LEVELS  (sizeof(levels) / sizeof(levels[0]))

static tile_t led;
static tile_t haptic;
static uint8_t led_ok = 0;

static void led_yellow(void) { if (led_ok) tile_display_rgbw_set(&led, 24, 12, 0, 0); }
static void led_green(void)  { if (led_ok) tile_display_rgbw_set(&led, 0, 24, 0, 0); }
static void led_red(void)    { if (led_ok) tile_display_rgbw_set(&led, 24, 0, 0, 0); }

static uint8_t drv_reg_read(uint8_t reg)
{
    uint8_t val = 0;
    haptic.hal->i2c_read(haptic.hal->handle, haptic.id, reg, &val, 1);
    return val;
}

static void drv_reg_write(uint8_t reg, uint8_t val)
{
    haptic.hal->i2c_write(haptic.hal->handle, haptic.id, reg, &val, 1);
}

static void dump_amp_regs(void)
{
    core_usb_printf("    RATED_VOLTAGE=0x%02X  OD_CLAMP=0x%02X  "
                    "A_CAL_COMP=0x%02X  A_CAL_BEMF=0x%02X  CTRL1=0x%02X\r\n",
                    drv_reg_read(0x16), drv_reg_read(0x17),
                    drv_reg_read(0x18), drv_reg_read(0x19),
                    drv_reg_read(0x1B));
}

/* Buzz at full scale for `ms`, sampling VBAT and resonance mid-buzz. */
static void feel_buzz(uint16_t ms, uint16_t *vbat, uint16_t *res)
{
    tile_drive_h_rtp_start(&haptic);
    tile_drive_h_rtp_write(&haptic, 0x7F);
    core_delay_ms(ms / 2);
    *vbat = tile_drive_h_get_vbat_mv(&haptic);
    *res  = tile_drive_h_get_resonance_hz(&haptic);
    core_delay_ms(ms / 2);
    tile_drive_h_rtp_write(&haptic, 0x00);
    tile_drive_h_rtp_stop(&haptic);
}

/* Retune DRIVE_TIME to the measured LRA resonance via the driver. */
static void retune_drive_time(uint16_t res_hz)
{
    if (res_hz < 100 || res_hz > 400) return;

    tile_drive_h_set_resonance_hz(&haptic, res_hz);
    core_usb_printf("  DRIVE_TIME retuned for %u Hz (CTRL1=0x%02X)\r\n",
                    res_hz, drv_reg_read(0x1B));
}

int main(void)
{
    core_init();
    core_usb_init();
    core_delay_ms(1500);

    core_usb_printf("\r\n==============================================\r\n");
    core_usb_printf("  hw-drive-h-amp — Drive.H amplitude staircase\r\n");
    core_usb_printf("  driver v%u.%u.%u\r\n",
                    TILE_DRIVE_H_VERSION_MAJOR,
                    TILE_DRIVE_H_VERSION_MINOR,
                    TILE_DRIVE_H_VERSION_PATCH);
    core_usb_printf("==============================================\r\n");

    tile_display_rgbw_init(core_tiles_pal(&core_i2c1), 0, &led, NULL);
    led_ok = tile_is_ready(&led);
    if (!led_ok) {
        core_usb_printf("  [INFO] Disp.RGBW not found — continuing without LED\r\n");
    }
    led_yellow();

    /* Locate the DRV2605: I2C3 (standard harness), then I2C1. */
    tiles_pal_t *pal = core_tiles_pal(&core_i2c3);
    if (!tile_drive_h_find(pal, 0)) {
        core_usb_printf("  [INFO] not on I2C3, probing I2C1...\r\n");
        pal = core_tiles_pal(&core_i2c1);
        if (!tile_drive_h_find(pal, 0)) {
            core_usb_printf("  [FAIL] DRV2605 not found on I2C3 or I2C1 @ 0x5A\r\n");
            led_red();
            while (1) { core_delay_ms(1000); }
        }
    }

    /* NULL cfg exercises the v4.2 defaults: closed-loop LRA,
     * 1.8 Vrms drive levels (the staircase overwrites them anyway).
     * Retry a couple of times in case the bus needs settling. */
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        tile_drive_h_init(pal, 0, &haptic, NULL);
        if (tile_is_ready(&haptic)) break;
        core_usb_printf("  [INFO] init attempt %u failed (STATUS=0x%02X), "
                        "retrying...\r\n", attempt + 1,
                        drv_reg_read(0x00));
        core_delay_ms(100);
    }
    if (!tile_is_ready(&haptic)) {
        core_usb_printf("  [FAIL] init failed after retries\r\n");
        led_red();
        while (1) { core_delay_ms(1000); }
    }
    core_usb_printf("  [PASS] init closed-loop, STATUS=0x%02X\r\n",
                    drv_reg_read(0x00));

    int8_t best = -1;
    uint16_t vbat = 0, res = 0;

    for (uint8_t i = 0; i < N_LEVELS; i++) {
        core_usb_printf("\r\n--- Level %u: %s (RV=0x%02X OC=0x%02X) ---\r\n",
                        i, levels[i].label, levels[i].rated_v,
                        levels[i].od_clamp);
        led_yellow();

        tile_drive_h_set_actuator_params(&haptic, levels[i].rated_v,
                                         levels[i].od_clamp, 0xFF, 0xFF);

        uint8_t cal_ok = tile_drive_h_calibrate(&haptic);
        core_usb_printf("  calibration: %s\r\n",
                        cal_ok ? "CONVERGED" : "did not converge");
        dump_amp_regs();
        if (cal_ok) best = (int8_t)i;

        core_usb_printf("  FEEL NOW: 2 clicks + 600 ms buzz\r\n");
        tile_drive_h_play(&haptic, 1, 2);   /* strong click x2 */
        core_delay_ms(600);
        feel_buzz(600, &vbat, &res);
        core_usb_printf("  during buzz: VBAT=%u mV, resonance=%u Hz\r\n",
                        vbat, res);

        /* First level doubles as a resonance probe: retune DRIVE_TIME
         * (init assumed 260 Hz) before the higher-level calibrations. */
        if (i == 0) {
            retune_drive_time(res);
        }

        core_delay_ms(2000);
    }

    /* Settle on the strongest level that calibrated cleanly. */
    uint8_t cal_any = (best >= 0);
    if (best < 0) {
        best = 2;  /* 1.8 Vrms — safe for 2.0 Vrms-class parts */
        core_usb_printf("\r\n  [WARN] no level calibrated cleanly; "
                        "settling at %s anyway\r\n", levels[best].label);
    }
    tile_drive_h_set_actuator_params(&haptic, levels[best].rated_v,
                                     levels[best].od_clamp, 0xFF, 0xFF);
    tile_drive_h_calibrate(&haptic);

    core_usb_printf("\r\n==============================================\r\n");
    core_usb_printf("  SETTLED: %s (RV=0x%02X OC=0x%02X, closed-loop)\r\n",
                    levels[best].label, levels[best].rated_v,
                    levels[best].od_clamp);
    dump_amp_regs();
    core_usb_printf("  heartbeat click every 3 s — compare by feel\r\n");
    core_usb_printf("==============================================\r\n");
    led_green();

    while (1) {
        tile_drive_h_play(&haptic, 1, 1);
        /* Reprint the verdict each beat so a late-attached serial
         * console still sees which level won. */
        core_usb_printf("  SETTLED %s (RV=0x%02X OC=0x%02X) res=%u Hz "
                        "cal=%s\r\n",
                        levels[best].label, levels[best].rated_v,
                        levels[best].od_clamp, res,
                        cal_any ? "ok" : "fallback");
        core_delay_ms(3000);
    }
}
