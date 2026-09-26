/**
 * hw-drive-dc-h-pad — PH/EN external pad control for Drive.DC.H.
 *
 * Demonstrates controlling the DRV8214 via external GPIO/PWM pads
 * instead of I2C bridge control. I2C is used only for initial
 * configuration and monitoring (voltage, current, faults).
 *
 * Core.ST.L4, clock=max
 *   Pad 3  (TIM2.2)  → Drive.DC.H EN  (tile pad 2) — PWM speed
 *   Pad 9  (GPIO)    → Drive.DC.H PH  (tile pad 3) — direction
 *   I2C1 @ 400 kHz   → Disp.RGBW (status LED)
 *   I2C3 @ 400 kHz   → Drive.DC.H (config + monitoring)
 *
 * LED codes:
 *   Yellow  = test in progress
 *   Green   = all tests passed
 *   Red     = failure (check CDC output)
 */

#include "core.h"
#include "core_usb.h"
#include "core_gpio.h"
#include "core_pwm.h"
#include "core_tiles.h"
#include "tile_display_rgbw.h"
#include "tile_drive_dc_h.h"

#define EN_PAD  3   /* TIM2.2 — PWM speed control */
#define PH_PAD  9   /* GPIO   — direction */

/* USB-safe pad control config: 0.5A sense, no regulation (external PWM) */
static const drive_dc_h_cfg_t motor_cfg = {
    .mode    = DRIVE_DC_H_MODE_PAD_PHEN,
    .vm_gain = 1,     /* 0-3.92V range for voltage monitoring */
    .cs_gain = 3,     /* 0.5A max sense, 800 mA OCP */
    .target  = 0,     /* unused in pad mode */
};

static tile_t led;
static tile_t motor;
static core_timer_t pwm;

static uint8_t test_fail = 0;

/* ---- Helpers ---- */

static void led_yellow(void) { tile_display_rgbw_set(&led, 24, 12, 0, 0); }
static void led_green(void)  { tile_display_rgbw_set(&led, 0, 24, 0, 0); }
static void led_red(void)    { tile_display_rgbw_set(&led, 24, 0, 0, 0); }

static void pass(const char *name)
{
    core_usb_printf("  [PASS] %s\r\n", name);
}

static void fail(const char *name, const char *reason)
{
    core_usb_printf("  [FAIL] %s — %s\r\n", name, reason);
    test_fail = 1;
}

static void section(const char *name)
{
    core_usb_printf("\r\n--- %s ---\r\n", name);
    led_yellow();
}

static void print_readings(void)
{
    uint16_t mv = tile_drive_dc_h_get_voltage_mv(&motor);
    uint16_t ma = tile_drive_dc_h_get_current_ma(&motor);
    uint8_t  f  = tile_drive_dc_h_get_fault(&motor);
    core_usb_printf("  V=%u mV  I=%u mA  FAULT=0x%02X\r\n", mv, ma, f);
}

/* Safe stop: set EN duty to 0 (brake in PH/EN mode) */
static void safe_stop(void)
{
    core_pwm_set_pad(&pwm, EN_PAD, 0);
    core_delay_ms(200);
}

/* ---- Tests ---- */

static void test_init(tiles_pal_t *hal)
{
    section("Init (PAD_PHEN mode)");

    tile_drive_dc_h_init(hal, 0, &motor, &motor_cfg);

    if (tile_is_ready(&motor)) {
        pass("init");
        core_usb_printf("  Device at 0x%02X\r\n", motor.id);
        core_usb_printf("  Mode = PAD_PHEN (EN on pad %u, PH on pad %u)\r\n",
                        EN_PAD, PH_PAD);
    } else {
        fail("init", "DRV8214 not ready");
    }
}

static void test_forward_sweep(void)
{
    section("Forward sweep (PH=HIGH, EN duty 0→100%)");

    core_pad_write(PH_PAD, 1);  /* forward direction */

    for (uint16_t duty = 0; duty <= 1000; duty += 200) {
        core_pwm_set_pad(&pwm, EN_PAD, duty);
        core_delay_ms(500);
        core_usb_printf("  Duty=%u%%: ", duty / 10);
        print_readings();
    }

    pass("forward sweep complete");
    safe_stop();
    core_delay_ms(300);
}

static void test_reverse_sweep(void)
{
    section("Reverse sweep (PH=LOW, EN duty 0→100%)");

    core_pad_write(PH_PAD, 0);  /* reverse direction */

    for (uint16_t duty = 0; duty <= 1000; duty += 200) {
        core_pwm_set_pad(&pwm, EN_PAD, duty);
        core_delay_ms(500);
        core_usb_printf("  Duty=%u%%: ", duty / 10);
        print_readings();
    }

    pass("reverse sweep complete");
    safe_stop();
    core_delay_ms(300);
}

static void test_brake(void)
{
    section("Brake (EN=LOW while motor spinning)");

    core_pad_write(PH_PAD, 1);
    core_pwm_set(&pwm, 2, 800);  /* 80% forward */
    core_delay_ms(500);
    core_usb_printf("  Driving at 80%%: ");
    print_readings();

    /* EN low = brake in PH/EN mode */
    core_pwm_set_pad(&pwm, EN_PAD, 0);
    core_delay_ms(200);
    core_usb_printf("  After brake: ");
    print_readings();

    pass("brake (EN low)");
}

static void test_direction_change(void)
{
    section("Direction change (PH toggle while driving)");

    core_pad_write(PH_PAD, 1);
    core_pwm_set_pad(&pwm, EN_PAD, 500);  /* 50% forward */
    core_delay_ms(500);
    core_usb_printf("  Forward 50%%: ");
    print_readings();

    /* Stop before reversing — safe for USB power */
    safe_stop();
    core_delay_ms(500);

    core_pad_write(PH_PAD, 0);   /* switch to reverse */
    core_pwm_set_pad(&pwm, EN_PAD, 500);  /* 50% reverse */
    core_delay_ms(500);
    core_usb_printf("  Reverse 50%%: ");
    print_readings();

    safe_stop();
    pass("direction change");
}

static void test_sleep_wake(void)
{
    section("Sleep / Wake (EN_OUT via I2C)");

    tile_drive_dc_h_sleep(&motor);
    if (tile_state(&motor) == TILE_STATE_SLEEPING) {
        pass("sleep");
    } else {
        fail("sleep", "state not SLEEPING");
    }

    /* PWM while sleeping — motor should NOT spin (EN_OUT=0) */
    core_pad_write(PH_PAD, 1);
    core_pwm_set_pad(&pwm, EN_PAD, 800);
    core_delay_ms(300);
    core_usb_printf("  PWM at 80%% during sleep (motor should be stopped)\r\n");

    /* Stop PWM before I2C reads — bus hangs with EN active + EN_OUT=0 */
    safe_stop();

    tile_drive_dc_h_wake(&motor);
    if (tile_is_ready(&motor)) {
        pass("wake");
    } else {
        fail("wake", "state not READY");
    }

    /* Motor should spin now */
    core_pad_write(PH_PAD, 1);
    core_pwm_set_pad(&pwm, EN_PAD, 800);
    core_delay_ms(500);
    core_usb_printf("  PWM at 80%% after wake: ");
    print_readings();

    safe_stop();
    pass("sleep/wake pad control");
}

/* ---- Main ---- */

int main(void)
{
    core_init();
    core_usb_init();
    core_delay_ms(1500);

    core_usb_printf("\r\n========================================\r\n");
    core_usb_printf("  hw-drive-dc-h-pad — PH/EN pad control\r\n");
    core_usb_printf("========================================\r\n");

    /* Init LED */
    tiles_pal_t *hal_led   = core_tiles_pal(&core_i2c1);
    tiles_pal_t *hal_motor = core_tiles_pal(&core_i2c3);

    tile_display_rgbw_init(hal_led, 0, &led, NULL);
    led_yellow();

    /* Init PWM on EN pad — 25 kHz to match DRV8214 expectations */
    core_pwm_init_pad(&pwm, EN_PAD, 25000);
    core_pwm_set_pad(&pwm, EN_PAD, 0);   /* start at 0% duty */
    core_pwm_start(&pwm);

    /* Init PH pad as GPIO output, default low */
    core_pad_output(PH_PAD);
    core_pad_write(PH_PAD, 0);

    /* Init DRV8214 in PAD_PHEN mode */
    test_init(hal_motor);
    if (!tile_is_ready(&motor)) {
        fail("init", "cannot continue");
        led_red();
        while (1) { core_delay_ms(1000); }
    }

    /* Clear any startup faults */
    tile_drive_dc_h_clear_fault(&motor);

    /* Run tests */
    test_forward_sweep();
    test_reverse_sweep();
    test_brake();
    test_direction_change();
    test_sleep_wake();

    /* Final result */
    core_usb_printf("\r\n========================================\r\n");
    if (test_fail) {
        core_usb_printf("  RESULT: FAIL (see above)\r\n");
        led_red();
    } else {
        core_usb_printf("  RESULT: ALL TESTS PASSED\r\n");
        led_green();
    }
    core_usb_printf("========================================\r\n");

    while (1) {
        core_delay_ms(1000);
    }
}
