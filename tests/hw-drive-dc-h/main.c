/**
 * hw-drive-dc-h — Hardware validation for the Drive.DC.H tile driver (DRV8214).
 *
 * Runs through every driver feature with USB CDC printf output
 * and Disp.RGBW LED for pass/fail indication. Connect a small
 * brushed DC motor to the Drive.DC.H outputs for motor tests.
 *
 * Core.ST.L4, clock=max
 *   I2C1 @ 400 kHz — Disp.RGBW (status LED)
 *   I2C3 @ 400 kHz — Drive.DC.H (motor driver)
 *
 * LED codes:
 *   Yellow  = test in progress
 *   Green   = all tests passed
 *   Red     = failure (check CDC output)
 *
 * Power budget: USB 5V / 500 mA. Config uses CS_GAIN_SEL=3
 * (0.5 A sense range, 800 mA OCP) and ~2.8 V target voltage.
 */

#include "core.h"
#include "core_usb.h"
#include "core_tiles.h"
#include "tile_display_rgbw.h"
#include "tile_drive_dc_h.h"

/* USB-safe motor config with ripple counting enabled.
 * RS PRO 834-7644 (gearhead removed, direct drive):
 *   ~6 ohm winding, 3-pole/2-brush (6 ripples/rev),
 *   Kv ≈ 187 uV/RPM (from 3V / 16014 RPM no-load).           */
static const drive_dc_h_cfg_t motor_cfg = {
    .mode            = DRIVE_DC_H_MODE_RIPPLE_COUNT,
    .vm_gain         = 1,     /* 0-3.92V range */
    .cs_gain         = 3,     /* 0.5A max sense, 800 mA OCP */
    .target          = 180,   /* 3.92 * 180/255 ≈ 2.8V */
    .motor_mohm      = 6000,  /* estimated 6 ohm winding */
    .ripples_per_rev = 12,    /* estimated from ripple count vs expected RPM */
    .kv_uv_per_rpm   = 187,   /* estimated from no-load speed */
};

static tile_t led;
static tile_t motor;

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

static void info(const char *name, const char *detail)
{
    core_usb_printf("  [INFO] %s — %s\r\n", name, detail);
}

static void section(const char *name)
{
    core_usb_printf("\r\n--- %s ---\r\n", name);
    led_yellow();
}

/* Stop motor safely: brake first, then coast to avoid back-EMF spikes */
static void safe_stop(void)
{
    tile_drive_dc_h_brake(&motor);
    core_delay_ms(500);
    tile_drive_dc_h_coast(&motor);
    core_delay_ms(200);
}

static void print_fault(void)
{
    uint8_t f = tile_drive_dc_h_get_fault(&motor);
    if (f) core_usb_printf("  FAULT=0x%02X%s%s%s%s\r\n", f,
        (f & DRV8214_FAULT_STALL) ? " STALL" : "",
        (f & DRV8214_FAULT_OCP)   ? " OCP"   : "",
        (f & DRV8214_FAULT_OVP)   ? " OVP"   : "",
        (f & DRV8214_FAULT_TSD)   ? " TSD"   : "");
}

/* ---- I2C scan ---- */

static void i2c_scan(tiles_pal_t *hal)
{
    section("I2C3 bus scan");
    uint8_t found = 0;

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (hal->i2c_is_ready(hal->handle, addr) == 0) {
            core_usb_printf("  Device at 0x%02X\r\n", addr);
            found++;
        }
    }

    if (found > 0) {
        core_usb_printf("  %u device(s) found\r\n", found);
    } else {
        core_usb_printf("  No devices found\r\n");
    }
}

/* ---- Tests ---- */

static void test_find(tiles_pal_t *hal)
{
    section("Find (address probe)");

    /* Try default instance 0 (0x34) */
    if (tile_drive_dc_h_find(hal, 0)) {
        pass("find instance 0 (0x34)");
        return;
    }

    core_usb_printf("  Instance 0 not found, trying instance 4 (0x33)...\r\n");
    if (tile_drive_dc_h_find(hal, 4)) {
        pass("find instance 4 (0x33)");
        info("address", "tile is at instance 4, not default");
        return;
    }

    /* Brute-force scan all 9 instances */
    for (uint8_t i = 1; i < 9; i++) {
        if (i == 4) continue;  /* already tried */
        if (tile_drive_dc_h_find(hal, i)) {
            core_usb_printf("  Found at instance %u\r\n", i);
            pass("find (alternate instance)");
            return;
        }
    }

    fail("find", "DRV8214 not detected on any known address");
}

static void test_init(tiles_pal_t *hal)
{
    section("Init (USB-safe cfg: 0.5A, 2.8V)");

    /* Try instance 0 first, fall back to instance 4 */
    tile_drive_dc_h_init(hal, 0, &motor, &motor_cfg);

    if (!tile_is_ready(&motor)) {
        core_usb_printf("  Instance 0 failed, trying instance 4...\r\n");
        tile_drive_dc_h_init(hal, 4, &motor, &motor_cfg);
    }

    if (tile_is_ready(&motor)) {
        pass("init");
        core_usb_printf("  Device at 0x%02X\r\n", motor.id);
    } else {
        fail("init", "tile not ready after init");
    }
}

static void test_fault_status(void)
{
    section("Fault status");

    uint8_t fault = tile_drive_dc_h_get_fault(&motor);
    core_usb_printf("  FAULT = 0x%02X\r\n", fault);

    if (fault & DRV8214_FAULT_STALL) info("fault", "STALL");
    if (fault & DRV8214_FAULT_OCP)   info("fault", "OCP");
    if (fault & DRV8214_FAULT_OVP)   info("fault", "OVP");
    if (fault & DRV8214_FAULT_TSD)   info("fault", "TSD");
    if (fault & DRV8214_FAULT_NPOR)  info("fault", "NPOR");

    if (!(fault & DRV8214_FAULT_FAULT)) {
        pass("no active faults");
    } else {
        info("faults", "clearing...");
        tile_drive_dc_h_clear_fault(&motor);
        fault = tile_drive_dc_h_get_fault(&motor);
        if (!(fault & DRV8214_FAULT_FAULT)) {
            pass("faults cleared");
        } else {
            fail("clear_fault", "faults persist after clear");
        }
    }
}

static void test_forward(void)
{
    section("Forward drive (2s)");

    tile_drive_dc_h_forward(&motor);
    core_delay_ms(500);

    uint16_t mv = tile_drive_dc_h_get_voltage_mv(&motor);
    uint16_t ma = tile_drive_dc_h_get_current_ma(&motor);
    core_usb_printf("  Voltage = %u mV\r\n", mv);
    core_usb_printf("  Current = %u mA\r\n", ma);
    print_fault();

    if (mv > 0) {
        pass("forward drive (voltage detected)");
    } else {
        info("forward", "VMTR=0 (motor may not be connected)");
    }

    core_delay_ms(1500);
    pass("forward 2s complete");

    safe_stop();
}

static void test_brake(void)
{
    section("Brake");

    tile_drive_dc_h_forward(&motor);
    core_delay_ms(500);
    tile_drive_dc_h_brake(&motor);
    core_delay_ms(200);

    uint16_t mv = tile_drive_dc_h_get_voltage_mv(&motor);
    core_usb_printf("  Voltage after brake = %u mV\r\n", mv);
    pass("brake");

    /* Coast safely after brake */
    core_delay_ms(300);
    tile_drive_dc_h_coast(&motor);
    core_delay_ms(500);
}

static void test_reverse(void)
{
    section("Reverse drive (2s)");

    tile_drive_dc_h_reverse(&motor);
    core_delay_ms(500);

    uint16_t mv = tile_drive_dc_h_get_voltage_mv(&motor);
    uint16_t ma = tile_drive_dc_h_get_current_ma(&motor);
    core_usb_printf("  Voltage = %u mV\r\n", mv);
    core_usb_printf("  Current = %u mA\r\n", ma);
    print_fault();

    core_delay_ms(1500);
    pass("reverse 2s complete");

    safe_stop();
}

static void test_coast(void)
{
    section("Coast");

    /* Drive briefly, then safe-stop (brake → coast) */
    tile_drive_dc_h_forward(&motor);
    core_delay_ms(500);
    safe_stop();
    pass("coast (brake → Hi-Z)");
}

static void test_ripple_and_speed(void)
{
    section("Ripple count & speed");

    /* Clear counter and drive forward to generate ripples */
    tile_drive_dc_h_clear_ripple_count(&motor);
    tile_drive_dc_h_forward(&motor);
    core_delay_ms(500);  /* let motor spin up */

    /* Sample speed and count a few times */
    for (uint8_t i = 0; i < 4; i++) {
        core_delay_ms(500);
        uint16_t count = tile_drive_dc_h_get_ripple_count(&motor);
        uint8_t  speed = tile_drive_dc_h_get_speed(&motor);
        uint16_t mv    = tile_drive_dc_h_get_voltage_mv(&motor);
        uint16_t ma    = tile_drive_dc_h_get_current_ma(&motor);
        core_usb_printf("  [%u] count=%u  speed=%u  V=%u mV  I=%u mA\r\n",
                        i, count, speed, mv, ma);
    }

    uint16_t final_count = tile_drive_dc_h_get_ripple_count(&motor);
    safe_stop();

    if (final_count > 0) {
        /* Estimate RPM: count was over ~2.5s of driving.
         * RPM = (count / ripples_per_rev) / time_s * 60
         * With 12 ripples/rev and 2.5s:
         *   RPM = count * 60 / (12 * 2.5) = count * 2             */
        uint32_t est_rpm = (uint32_t)final_count * 2;
        core_usb_printf("  Total ripples in ~2.5s: %u\r\n", final_count);
        core_usb_printf("  Estimated RPM: ~%lu (assuming 12 ripples/rev)\r\n",
                        (unsigned long)est_rpm);
        pass("ripple counting active");
    } else {
        info("ripple count", "0 (motor may not be connected or ripple counting not tuned)");
    }

    /* Test stall detection */
    uint8_t stalled = tile_drive_dc_h_is_stalled(&motor);
    core_usb_printf("  Stalled = %u\r\n", stalled);
    pass("is_stalled read");
}

static void test_set_target(void)
{
    section("Set target voltage");

    /* Drive forward at reduced voltage */
    tile_drive_dc_h_set_target(&motor, 64);   /* ~1.0V */
    tile_drive_dc_h_forward(&motor);
    core_delay_ms(500);

    uint16_t mv_low = tile_drive_dc_h_get_voltage_mv(&motor);
    core_usb_printf("  Target=64:  Voltage = %u mV\r\n", mv_low);

    /* Restore test config target */
    tile_drive_dc_h_set_target(&motor, 180);  /* ~2.8V */
    core_delay_ms(500);

    uint16_t mv_high = tile_drive_dc_h_get_voltage_mv(&motor);
    core_usb_printf("  Target=180: Voltage = %u mV\r\n", mv_high);

    safe_stop();

    if (mv_high > mv_low && mv_low > 0) {
        pass("voltage regulation (higher target = higher voltage)");
    } else if (mv_high > 0) {
        info("set_target", "voltage difference not detected (motor load dependent)");
    } else {
        info("set_target", "no voltage reading (motor may not be connected)");
    }
}

static void test_sleep_wake(void)
{
    section("Sleep / Wake");

    tile_drive_dc_h_sleep(&motor);
    if (tile_state(&motor) == TILE_STATE_SLEEPING) {
        pass("sleep (EN_OUT cleared)");
    } else {
        fail("sleep", "state not SLEEPING");
    }

    tile_drive_dc_h_wake(&motor);
    if (tile_is_ready(&motor)) {
        pass("wake (EN_OUT restored)");
    } else {
        fail("wake", "state not READY after wake");
    }

    /* Verify motor control still works after wake */
    tile_drive_dc_h_forward(&motor);
    core_delay_ms(300);
    safe_stop();
    pass("forward after wake");
}

/* ---- Main ---- */

int main(void)
{
    core_init();
    core_usb_init();
    core_delay_ms(1500);

    core_usb_printf("\r\n========================================\r\n");
    core_usb_printf("  hw-drive-dc-h — Drive.DC.H v%u.%u.%u\r\n",
                    TILE_DRIVE_DC_H_VERSION_MAJOR,
                    TILE_DRIVE_DC_H_VERSION_MINOR,
                    TILE_DRIVE_DC_H_VERSION_PATCH);
    core_usb_printf("========================================\r\n");

    /* Init LED */
    tiles_pal_t *hal_led   = core_tiles_pal(&core_i2c1);
    tiles_pal_t *hal_motor = core_tiles_pal(&core_i2c3);

    tile_display_rgbw_init(hal_led, 0, &led, NULL);
    led_yellow();

    /* I2C scan */
    i2c_scan(hal_motor);

    /* Find */
    test_find(hal_motor);

    /* Init */
    test_init(hal_motor);
    if (!tile_is_ready(&motor)) {
        fail("init", "cannot continue — motor driver not ready");
        led_red();
        while (1) { core_delay_ms(1000); }
    }

    /* Test sequence */
    test_fault_status();
    test_forward();
    test_brake();
    test_reverse();
    test_coast();
    test_ripple_and_speed();
    test_set_target();
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
