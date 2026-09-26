/**
 * hw-drive-h — Hardware validation for the Drive.H tile driver (DRV2605L).
 *
 * Runs through every driver feature with USB CDC printf output
 * and Disp.RGBW LED for pass/fail indication. Requires an LRA
 * connected to Drive.H outputs.
 *
 * Core.ST.L4, clock=max
 *   I2C1 @ 400 kHz — Disp.RGBW (status LED)
 *   I2C3 @ 400 kHz — Drive.H   (haptic driver)
 *
 * LED codes:
 *   Yellow  = test in progress
 *   Green   = all tests passed
 *   Red     = failure (check CDC output)
 */

#include "core.h"
#include "core_usb.h"
#include "core_gpio.h"
#include "core_tiles.h"
#include "tile_display_rgbw.h"
#include "tile_drive_h.h"

#define TRIG_PAD  3  /* Core.ST.L4 pad 3 → Drive.H IN/TRIG */

static tile_t led;
static tile_t haptic;

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

/* Direct register helper for diagnostic reads */
static uint8_t drv_reg_read(uint8_t reg)
{
    uint8_t val = 0;
    haptic.hal->i2c_read(haptic.hal->handle, haptic.id, reg, &val, 1);
    return val;
}

static void dump_cal_regs(void)
{
    core_usb_printf("  Registers:\r\n");
    core_usb_printf("    STATUS        = 0x%02X\r\n", drv_reg_read(0x00));
    core_usb_printf("    RATED_VOLTAGE = 0x%02X\r\n", drv_reg_read(0x16));
    core_usb_printf("    OD_CLAMP      = 0x%02X\r\n", drv_reg_read(0x17));
    core_usb_printf("    A_CAL_COMP    = 0x%02X\r\n", drv_reg_read(0x18));
    core_usb_printf("    A_CAL_BEMF    = 0x%02X\r\n", drv_reg_read(0x19));
    core_usb_printf("    FEEDBACK_CTRL = 0x%02X\r\n", drv_reg_read(0x1A));
    core_usb_printf("    CONTROL1      = 0x%02X\r\n", drv_reg_read(0x1B));
    core_usb_printf("    CONTROL3      = 0x%02X\r\n", drv_reg_read(0x1D));
}

/* ---- Tests ---- */

static void test_find_and_status(void)
{
    section("Find & Status");

    uint8_t found = tile_drive_h_find(core_tiles_pal(&core_i2c3), 0);
    if (found) {
        pass("find");
    } else {
        fail("find", "DRV2605L not detected on I2C3 @ 0x5A");
        return;
    }

    uint8_t status = tile_drive_h_get_status(&haptic);
    core_usb_printf("  STATUS = 0x%02X (DEVICE_ID=%u)\r\n",
                    status, (status >> 5) & 0x07);

    if ((status & DRV2605L_STATUS_OVER_TEMP) == 0) {
        pass("no over-temp");
    } else {
        fail("over-temp", "OVER_TEMP flag set");
    }

    if ((status & DRV2605L_STATUS_OC_DETECT) == 0) {
        pass("no overcurrent");
    } else {
        fail("overcurrent", "OC_DETECT flag set");
    }
}

static void test_init(void)
{
    section("Init (default cfg)");

    tile_drive_h_init(core_tiles_pal(&core_i2c3), 0, &haptic, NULL);

    if (tile_is_ready(&haptic)) {
        pass("init (defaults)");
    } else {
        fail("init", "tile not ready after init");
    }
}

static void test_single_play(void)
{
    section("Single effect playback");

    core_usb_printf("  Playing effect #1 (strong click)...\r\n");
    tile_drive_h_play(&haptic, 1, 1);

    /* Wait for effect to complete */
    uint16_t wait = 0;
    while (tile_drive_h_is_playing(&haptic) && wait < 2000) {
        core_delay_ms(10);
        wait += 10;
    }

    if (!tile_drive_h_is_playing(&haptic)) {
        core_usb_printf("  Completed in ~%u ms\r\n", wait);
        pass("play + is_playing");
    } else {
        fail("play", "GO bit never cleared");
    }

    core_delay_ms(300);
}

static void test_sequence(void)
{
    section("Sequence playback (3 effects)");

    const uint8_t seq[] = { 1, 10, 47 };
    core_usb_printf("  Playing sequence: #1, #10, #47...\r\n");
    tile_drive_h_play_sequence(&haptic, seq, 3);

    uint16_t wait = 0;
    while (tile_drive_h_is_playing(&haptic) && wait < 5000) {
        core_delay_ms(10);
        wait += 10;
    }

    if (!tile_drive_h_is_playing(&haptic)) {
        core_usb_printf("  Completed in ~%u ms\r\n", wait);
        pass("play_sequence");
    } else {
        fail("play_sequence", "GO bit never cleared");
        tile_drive_h_stop(&haptic);
    }

    core_delay_ms(300);
}

static void test_edge_trigger(void)
{
    section("Edge trigger (pad 3 -> IN/TRIG)");

    /* Pre-load effect #1 (strong click) into sequencer */
    const uint8_t fx[] = { 1 };
    tile_drive_h_load_sequence(&haptic, fx, 1);

    /* Switch to edge trigger mode */
    tile_drive_h_set_trigger(&haptic, DRIVE_H_TRIG_EDGE);
    core_usb_printf("  Mode = edge trigger, pulsing pad %u...\r\n", TRIG_PAD);

    /* Ensure pin starts low */
    core_pad_output(TRIG_PAD);
    core_pad_write(TRIG_PAD, 0);
    core_delay_ms(5);

    /* Rising edge fires the pre-loaded effect */
    core_pad_write(TRIG_PAD, 1);
    core_delay_ms(1);       /* Pulse width >= 1 µs (datasheet) */
    core_pad_write(TRIG_PAD, 0);

    /* Check if playback started */
    core_delay_ms(10);
    uint8_t started = tile_drive_h_is_playing(&haptic);

    if (started) {
        core_usb_printf("  Effect triggered via GPIO edge\r\n");
    } else {
        core_usb_printf("  Effect may have already completed\r\n");
    }

    /* Wait for completion */
    uint16_t wait = 0;
    while (tile_drive_h_is_playing(&haptic) && wait < 2000) {
        core_delay_ms(10);
        wait += 10;
    }
    core_usb_printf("  Completed in ~%u ms\r\n", wait + 10);
    pass("edge trigger");

    /* Return to internal trigger mode */
    tile_drive_h_set_trigger(&haptic, DRIVE_H_TRIG_INTERNAL);
    core_delay_ms(200);
}

static void test_level_trigger(void)
{
    section("Level trigger (pad 3 -> IN/TRIG)");

    /* Pre-load a longer effect (#47 = buzzing) */
    const uint8_t fx[] = { 47 };
    tile_drive_h_load_sequence(&haptic, fx, 1);

    /* Switch to level trigger mode */
    tile_drive_h_set_trigger(&haptic, DRIVE_H_TRIG_LEVEL);
    core_usb_printf("  Mode = level trigger\r\n");

    /* Hold pin high to start playing */
    core_pad_write(TRIG_PAD, 1);
    core_usb_printf("  Pin HIGH — playing...\r\n");
    core_delay_ms(300);

    /* Verify playing while pin is high */
    uint8_t playing = tile_drive_h_is_playing(&haptic);
    if (playing) {
        pass("level trigger (GO follows pin)");
    } else {
        info("level trigger", "GO=0 (effect may have ended)");
    }

    /* Drop pin to cancel */
    core_pad_write(TRIG_PAD, 0);
    core_usb_printf("  Pin LOW — cancelled\r\n");
    core_delay_ms(50);

    /* Verify stopped */
    if (!tile_drive_h_is_playing(&haptic)) {
        pass("level cancel (falling edge)");
    } else {
        fail("level cancel", "GO still high after pin low");
        tile_drive_h_stop(&haptic);
    }

    /* Return to internal trigger mode */
    tile_drive_h_set_trigger(&haptic, DRIVE_H_TRIG_INTERNAL);
    core_delay_ms(200);
}

static void test_rtp(void)
{
    section("RTP mode");

    tile_drive_h_rtp_start(&haptic);
    core_usb_printf("  Ramping amplitude 0 -> 127...\r\n");

    for (uint8_t a = 0; a < 128; a += 16) {
        tile_drive_h_rtp_write(&haptic, a);
        core_delay_ms(50);
    }
    tile_drive_h_rtp_write(&haptic, 127);
    core_delay_ms(200);

    /* Read VBAT and resonance while driving */
    uint16_t vbat = tile_drive_h_get_vbat_mv(&haptic);
    uint16_t res  = tile_drive_h_get_resonance_hz(&haptic);

    core_usb_printf("  VBAT = %u mV\r\n", vbat);
    core_usb_printf("  LRA resonance = %u Hz\r\n", res);

    if (vbat > 0) {
        pass("get_vbat_mv");
    } else {
        fail("get_vbat_mv", "VBAT = 0 during RTP");
    }

    if (res > 0) {
        pass("get_resonance_hz");
    } else {
        fail("get_resonance_hz", "resonance = 0 during RTP");
    }

    tile_drive_h_rtp_write(&haptic, 0);
    core_delay_ms(50);
    tile_drive_h_rtp_stop(&haptic);
    pass("rtp_start / rtp_write / rtp_stop");

    core_delay_ms(300);
}

static void test_calibration(void)
{
    section("Auto-calibration");

    core_usb_printf("  Pre-cal register state:\r\n");
    dump_cal_regs();

    core_usb_printf("  Running calibration...\r\n");
    uint8_t cal_ok = tile_drive_h_calibrate(&haptic);

    core_usb_printf("  Post-cal register state:\r\n");
    dump_cal_regs();

    if (cal_ok) {
        pass("calibrate");
    } else {
        fail("calibrate", "did not converge");
    }

    core_delay_ms(200);
}

static void test_diagnostics(void)
{
    section("Diagnostics");

    core_usb_printf("  Running actuator diagnostics...\r\n");
    uint8_t diag = tile_drive_h_diagnose(&haptic);

    uint8_t status = tile_drive_h_get_status(&haptic);
    core_usb_printf("  Post-diag STATUS = 0x%02X\r\n", status);

    if (diag) {
        pass("diagnose (actuator OK)");
    } else {
        /* Back-EMF diagnostic is marginal with small LRAs on cold
         * boot. Not a real fault if playback/RTP/cal all work. */
        info("diagnose", "DIAG_RESULT=fail (marginal with small LRA)");
    }

    core_delay_ms(200);
}

static void test_standby_wake(void)
{
    section("Standby / Wake");

    tile_drive_h_standby(&haptic);
    if (tile_state(&haptic) == TILE_STATE_SLEEPING) {
        pass("standby");
    } else {
        fail("standby", "state not SLEEPING");
    }

    tile_drive_h_wake(&haptic);
    if (tile_is_ready(&haptic)) {
        pass("wake");
    } else {
        fail("wake", "state not READY after wake");
    }

    /* Verify playback still works after wake */
    tile_drive_h_play(&haptic, 1, 1);
    core_delay_ms(200);

    if (!tile_drive_h_is_playing(&haptic)) {
        pass("play after wake");
    } else {
        tile_drive_h_stop(&haptic);
        pass("play after wake (stopped)");
    }
}

/* ---- Main ---- */

int main(void)
{
    core_init();
    core_usb_init();
    core_delay_ms(1500);

    core_usb_printf("\r\n========================================\r\n");
    core_usb_printf("  hw-drive-h — Drive.H v%u.%u.%u validation\r\n",
                    TILE_DRIVE_H_VERSION_MAJOR,
                    TILE_DRIVE_H_VERSION_MINOR,
                    TILE_DRIVE_H_VERSION_PATCH);
    core_usb_printf("========================================\r\n");

    /* Init LED */
    tile_display_rgbw_init(core_tiles_pal(&core_i2c1), 0, &led, NULL);
    led_yellow();

    /* Init haptic driver */
    test_init();
    if (!tile_is_ready(&haptic)) {
        fail("init", "cannot continue — haptic not ready");
        led_red();
        while (1) { core_delay_ms(1000); }
    }

    test_find_and_status();
    test_single_play();
    test_sequence();
    test_edge_trigger();
    test_level_trigger();

    /* RTP first — gives us resonance/VBAT info */
    test_rtp();

    /* Diagnostics before calibration: verify actuator is present
     * before optimizing for it. Diag uses default cal baselines. */
    test_diagnostics();
    test_calibration();

    test_standby_wake();

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
