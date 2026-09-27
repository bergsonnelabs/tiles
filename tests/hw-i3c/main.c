/**
 * hw-i3c — I3C on Core.ST.H5 with a Sense.I.6P6 (ICM-42686-P), over USB CDC.
 *
 * config.json asks for I3C on I2C1 ("i3c": true). The tile's definition
 * offers I3C on its bus pads, so coregen runs pads 4/5 (PB8 SCL, PB7 SDA) as
 * I3C1 at 12.4 MHz, resets stale dynamic addresses (RSTDAA) and gives the
 * tile its dynamic address (SETDASA 0x69 -> 0x09) inside core_init(). The
 * tile driver then runs unchanged through core_tiles_pal(&core_i3c1).
 *
 * The report prints when a terminal opens the port, and again on "R":
 *   TILE   driver find/init over the I3C bridge; WHO_AM_I two ways
 *   DAA    RSTDAA, ENTDAA (PID/BCR/DCR), RSTDAA again, then SETDASA and
 *          GETPID/GETBCR/GETDCR/GETSTATUS at the dynamic address
 *   DATA   accel / gyro / temperature through tile_sense_i_6p6 (|a| ~ 1 g)
 *   TPUT   burst-read throughput: I3C at 12.4 MHz vs I2C1 on the same pads
 *          and device (14-byte sensor frame, 64-byte register burst). The
 *          I2C part runs the driver itself over plain I2C at 100 kHz,
 *          400 kHz and 1 MHz, open-drain on the MCU's pull-ups
 *   IBI    data-ready in-band interrupts per second at 100 Hz and 1 kHz ODR,
 *          and samples read on IBI
 *   ERR    a NACKed address, SETDASA to an absent static address, a tile that
 *          lost its dynamic address (RSTDAA) and the bridge re-assigning it,
 *          and a bus with no device (I3C2 on pads 2/3, unconnected here)
 * One command per line: R report, T/D/S/P/I/E one section, V hello,
 * Q drop off the bus (tests the 15 s no-USB safety net), X reset.
 *
 * Bench notes (2026-09-27, breadboard, tile on pads 1/4/5/10 only):
 *   - I2C here has only the MCU's ~40 kOhm pull-ups. The ICM boots with its
 *     I3C slave on, and then has no glitch filter: open-drain I2C failed
 *     until the driver applied the datasheet's I2C setting (1.4.0).
 *   - The ICM's reset SDA slew (< 2 ns) corrupts every I3C read on this
 *     wiring; the driver sets 4-12 ns on I3C (tile_sense_i_6p6.c).
 *
 *   make && make flash-serial
 *
 * BOOTLOADER is deliberately NOT set: "bootloader": "rom" in config.json gives
 * the ROM-DFU layout (app @ 0x08000000). The custom-bootloader layout is
 * parked, and flashing it onto a ROM-DFU board corrupts the resident app.
 */

#include "core.h"
#include "core_usb.h"
#include "core_watchdog.h"
#include "tile_handles.h"
#include "hal_i2c.h"
#include "core_i2c.h"
#include "ll_i2c.h"
#include "ll_usb_drd.h"
#include "hal_dfu.h"
#include <string.h>

#define DEMCR     REG32(0xE000EDFCUL)   /* TRCENA = bit 24 */
#define DWT_CTRL  REG32(0xE0001000UL)   /* CYCCNTENA = bit 0 */
#define DWT_CYC   REG32(0xE0001004UL)

#define ICM_STATIC    0x69U
#define ICM_WHO_AM_I  0x75U
#define ICM_TEMP1     0x1DU             /* TEMP_DATA1 .. GYRO_DATA_Z0: 14 bytes */
#define ICM_REGS_40   0x40U             /* 0x40-0x7F: config registers, no read side effects */
#define ICM_BANK_SEL  0x76U
#define ICM_INT_SRC8  0x4FU             /* bank 4: UI_DRDY_IBI_EN = bit 3 (DS-000639 §17.13) */

static tile_t *const imu = &tile_sense_i_6p6_i2c1_0;
static uint32_t g_pass, g_fail;

static void feed(void) { core_watchdog_feed(); }

static void verdict(int ok, const char *what)
{
    if (ok) g_pass++; else g_fail++;
    core_usb_printf("%s %s\n", ok ? "PASS" : "FAIL", what);
}

static uint32_t us_since(uint32_t c0) { return (DWT_CYC - c0) / (SYSCLK_HZ / 1000000UL); }

static uint32_t isqrt(uint32_t v)
{
    uint32_t r = 0, b = 1UL << 30;
    while (b > v) b >>= 2;
    while (b) { if (v >= r + b) { v -= r + b; r = (r >> 1) + b; } else r >>= 1; b >>= 2; }
    return r;
}

static uint8_t icm_da(void) { return core_i3c_attach(&core_i3c1, ICM_STATIC); }

/* ---- TILE ---- */

static void check_tile(void)
{
    {   /* what core_init() left: the table entry and a GETSTATUS at it */
        hal_i3c_target_t *t0 = &core_i3c1.targets[0];
        uint16_t st = 0;
        hal_status_t g = t0->dyn_addr ? core_i3c_get_status(&core_i3c1, t0->dyn_addr, &st) : HAL_ERROR;
        core_usb_printf("TILE after core_init: target static 0x%02x DA 0x%02x; GETSTATUS -> %d (SER 0x%03lx)\n",
                        t0->static_addr, t0->dyn_addr, (int)g, (unsigned long)core_i3c1.last_ser);
    }
    tiles_pal_t *pal = core_tiles_pal(&core_i3c1);
    uint32_t c0 = DWT_CYC;
    uint8_t found = tile_sense_i_6p6_find(pal, 0);
    tile_sense_i_6p6_init(pal, 0, imu, NULL);
    uint32_t init_us = us_since(c0);
    uint8_t who = 0;
    hal_status_t s = core_i3c_read_reg(&core_i3c1, icm_da(), ICM_WHO_AM_I, &who, 1);
    core_usb_printf("TILE find=%u init state=%d in %lu us, driver WHO_AM_I=0x%02x, "
                    "core_i3c_read_reg WHO_AM_I=0x%02x (%d) at DA 0x%02x, SCL %lu Hz, OD low %lu ns\n",
                    found, (int)imu->state, (unsigned long)init_us, imu->flags, who, (int)s, icm_da(),
                    (unsigned long)core_i3c1.scl_hz, (unsigned long)core_i3c1.od_low_ns);
    verdict(found && imu->state == TILE_STATE_READY && imu->flags == 0x44 && who == 0x44,
            "TILE driver init + WHO_AM_I 0x44 over I3C");
}

/* ---- DAA ---- */

static void check_daa(void)
{
    uint8_t n = 0;
    hal_status_t r1 = core_i3c_rstdaa(&core_i3c1);
    hal_status_t e = core_i3c_entdaa(&core_i3c1, &n);
    hal_i3c_target_t d = core_i3c1.targets[0];
    core_usb_printf("DAA RSTDAA=%d ENTDAA=%d found=%u: DA 0x%02x PID %02x%02x%02x%02x%02x%02x "
                    "(MIPI mfr 0x%03x) BCR 0x%02x DCR 0x%02x\n", (int)r1, (int)e, n, d.dyn_addr,
                    d.pid[0], d.pid[1], d.pid[2], d.pid[3], d.pid[4], d.pid[5],
                    (unsigned)(((d.pid[0] << 8) | d.pid[1]) >> 1), d.bcr, d.dcr);
    verdict(e == I3C_OK && n == 1, "DAA ENTDAA assigns the tile (1 target)");

    hal_status_t r2 = core_i3c_rstdaa(&core_i3c1);
    hal_status_t sd = core_i3c_setdasa(&core_i3c1, ICM_STATIC, 0);
    uint8_t da = icm_da();
    uint8_t pid[6] = {0}, bcr = 0, dcr = 0;
    uint16_t st = 0xFFFF;
    hal_status_t g1 = core_i3c_get_pid(&core_i3c1, da, pid);
    hal_status_t g2 = core_i3c_get_bcr(&core_i3c1, da, &bcr);
    hal_status_t g3 = core_i3c_get_dcr(&core_i3c1, da, &dcr);
    hal_status_t g4 = core_i3c_get_status(&core_i3c1, da, &st);
    core_usb_printf("DAA RSTDAA=%d SETDASA 0x%02x->0x%02x =%d GETPID=%d %02x%02x%02x%02x%02x%02x "
                    "GETBCR=%d 0x%02x GETDCR=%d 0x%02x GETSTATUS=%d 0x%04x\n", (int)r2, ICM_STATIC, da,
                    (int)sd, (int)g1, pid[0], pid[1], pid[2], pid[3], pid[4], pid[5], (int)g2, bcr,
                    (int)g3, dcr, (int)g4, st);
    verdict(sd == I3C_OK && da && da != ICM_STATIC && g1 == I3C_OK && !memcmp(pid, d.pid, 6) &&
            bcr == d.bcr && dcr == d.dcr && g4 == I3C_OK,
            "DAA SETDASA + GETPID/BCR/DCR match ENTDAA");
}

/* ---- DATA ---- */

static void check_data(void)
{
    int16_t a[3], g[3];
    uint32_t mag_min = 0xFFFFFFFFUL, mag_max = 0;
    core_delay_ms(30);
    for (int k = 0; k < 5; k++) {
        tile_sense_i_6p6_get_raw_accels(imu, a);
        tile_sense_i_6p6_get_raw_gyros(imu, g);
        int16_t t = tile_sense_i_6p6_get_temperature(imu);
        /* ±8 g range: 4096 LSB/g; ±1000 dps: 32.8 LSB/dps (DS-000639 Tables 1/2). */
        uint32_t mag = isqrt((uint32_t)((int32_t)a[0] * a[0] + (int32_t)a[1] * a[1] + (int32_t)a[2] * a[2]))
                       * 1000UL / 4096UL;
        if (mag < mag_min) mag_min = mag;
        if (mag > mag_max) mag_max = mag;
        core_usb_printf("DATA accel %6d %6d %6d (|a| %lu mg) gyro %6d %6d %6d temp %ld.%02ld C\n",
                        a[0], a[1], a[2], (unsigned long)mag, g[0], g[1], g[2],
                        (long)(2500 + (int32_t)t * 10000 / 13248) / 100,
                        (long)((2500 + (int32_t)t * 10000 / 13248) % 100));
        core_delay_ms(20);
        feed();
    }
    verdict(mag_min > 800 && mag_max < 1200, "DATA |accel| within 800-1200 mg (gravity) via the driver");
}

/* ---- TPUT ---- */

static void pins_i3c(void)
{
    ll_gpio_config_af(GPIOB, 8, 3, LL_GPIO_OTYPE_PP, LL_GPIO_SPEED_HIGH, LL_GPIO_PULL_UP);
    ll_gpio_config_af(GPIOB, 7, 3, LL_GPIO_OTYPE_PP, LL_GPIO_SPEED_HIGH, LL_GPIO_PULL_UP);
}

/* Reads of `len` bytes from `reg` for 1 s; returns bytes/s, counts failures. */
static uint32_t i3c_rate(uint8_t reg, uint16_t len, uint32_t *fails, uint32_t *per_us)
{
    static uint8_t buf[64];
    uint8_t da = icm_da();
    uint32_t n = 0, f = 0, t0 = core_millis(), c0 = DWT_CYC;
    while ((uint32_t)(core_millis() - t0) < 1000u) {
        if (core_i3c_read_reg(&core_i3c1, da, reg, buf, len) == I3C_OK) n++; else f++;
        if ((n & 255u) == 0) feed();
    }
    uint32_t us = us_since(c0);
    *fails = f;
    *per_us = n ? us / n : 0;
    return (uint32_t)((uint64_t)n * len * 1000000ULL / us);
}

static hal_i2c_t g_i2c;
static tile_t    g_imu_i2c;

/* I2C1 on pads 4/5 the way coregen sets up an I2C bus: AF4, open-drain, very
 * high speed, the MCU's pull-ups (the only ones on this bench). */
static void i2c_up(uint32_t hz)
{
    ll_rcc_apb1_clk_enable(LL_APB1_I2C1);
    ll_gpio_config_af(GPIOB, 8, 4, LL_GPIO_OTYPE_OD, LL_GPIO_SPEED_VHIGH, LL_GPIO_PULL_UP);
    ll_gpio_config_af(GPIOB, 7, 4, LL_GPIO_OTYPE_OD, LL_GPIO_SPEED_VHIGH, LL_GPIO_PULL_UP);
    core_i2c_init(&g_i2c, I2C1, hz);
}

/* 200 WHO_AM_I reads and 1 s of 14-byte sensor frames through the driver,
 * each frame checked against gravity. */
static void i2c_measure(const char *name, tile_t *t, uint32_t *bps_out)
{
    uint32_t ok = 0;
    for (int k = 0; k < 200; k++) {
        if (tile_sense_i_6p6_read_reg(t, 0, ICM_WHO_AM_I) == 0x44) ok++;
        if ((k & 15) == 0) feed();
    }
    int16_t r[7];
    uint32_t n = 0, off = 0, t0 = core_millis(), c0 = DWT_CYC;
    while ((uint32_t)(core_millis() - t0) < 1000u) {
        tile_sense_i_6p6_get_raw_all(t, r);          /* temp, accel xyz, gyro xyz: 14 bytes */
        uint32_t mg = isqrt((uint32_t)((int32_t)r[1] * r[1] + (int32_t)r[2] * r[2] + (int32_t)r[3] * r[3]))
                      * 1000UL / 4096UL;
        if (mg < 800 || mg > 1200) off++;
        n++;
        if ((n & 63u) == 0) feed();
    }
    uint32_t us = us_since(c0);
    *bps_out = (uint32_t)((uint64_t)n * 14 * 1000000ULL / us);
    core_usb_printf("INFO I2C %-7s open-drain: WHO_AM_I %lu/200; 14 B frames %lu B/s (%lu us each), "
                    "%lu of %lu off gravity\n", name, (unsigned long)ok, (unsigned long)*bps_out,
                    (unsigned long)(n ? us / n : 0), (unsigned long)off, (unsigned long)n);
    feed();
}

/* GETMRL: the target's maximum read length, 2 bytes MSB first; it ends
 * longer private reads early. (MIPI allows a third byte, the IBI payload
 * size; the ICM-42686-P sends two, and asking for three is a CE0 error.) */
static uint16_t get_mrl(uint8_t da, hal_status_t *st)
{
    uint8_t b[2] = {0};
    *st = hal_i3c_ccc_read(&core_i3c1, LL_I3C_CCC_GETMRL, da, b, 2, 0);
    return (uint16_t)((b[0] << 8) | b[1]);
}

static void check_tput(void)
{
    uint32_t f14, f64, us14, us64;
    uint32_t b14 = i3c_rate(ICM_TEMP1, 14, &f14, &us14);
    /* A 64-byte read is longer than the ICM's default maximum read length:
     * raise it with SETMRL (direct CCC 0x8A, 2 bytes MSB first). */
    uint8_t da0 = icm_da();
    hal_status_t sg1, sg2;
    uint16_t mrl0 = get_mrl(da0, &sg1);
    uint8_t mrl[2] = { 0, 64 };
    hal_status_t ss = hal_i3c_ccc_write(&core_i3c1, LL_I3C_CCC_SETMRL, da0, mrl, 2);
    uint16_t mrl1 = get_mrl(da0, &sg2);
    core_usb_printf("TPUT GETMRL=%d %u B, SETMRL 64 -> %d, GETMRL=%d %u B\n", (int)sg1, mrl0, (int)ss,
                    (int)sg2, mrl1);
    uint32_t b64 = i3c_rate(ICM_REGS_40, 64, &f64, &us64);
    core_usb_printf("TPUT I3C %lu Hz: 14 B burst %lu B/s (%lu us each, %lu failed); "
                    "64 B burst (0x40-0x7F) %lu B/s (%lu us each, %lu failed)\n",
                    (unsigned long)core_i3c1.scl_hz, (unsigned long)b14, (unsigned long)us14,
                    (unsigned long)f14, (unsigned long)b64, (unsigned long)us64, (unsigned long)f64);

    /* I2C on the same pads and device: drop the dynamic address so the ICM
     * answers its static one, move the pads to I2C1 (AF4), and let the driver
     * run over a plain I2C bridge. */
    core_i3c_rstdaa(&core_i3c1);
    core_i3c_deinit(&core_i3c1);        /* the controller must not see the I2C traffic */
    /* The driver's init over plain I2C at 100 kHz applies the datasheet's I2C
     * setting (tile_sense_i_6p6.c), which it has to get through while the
     * chip is still in I3C mode: on this bench (no pull-ups but the MCU's)
     * that is hit and miss. When it lands, the same tile is then read at
     * 100 kHz, 400 kHz and 1 MHz (no re-init: its soft reset would turn the
     * chip's I3C slave back on). INFO lines, not verdicts: they measure the
     * bench's pull-ups as much as the SDK. */
    uint32_t b100 = 0, b400 = 0, b1m = 0;
    i2c_up(I2C_100K);
    tile_sense_i_6p6_init(core_tiles_pal(&g_i2c), 0, &g_imu_i2c, NULL);
    uint8_t intf6 = tile_sense_i_6p6_read_reg(&g_imu_i2c, 1, 0x7C);
    core_usb_printf("INFO I2C driver init at 100 kHz, open-drain, MCU pull-ups: state %d, INTF_CONFIG6 0x%02x\n",
                    (int)g_imu_i2c.state, intf6);
    if (g_imu_i2c.state == TILE_STATE_READY) {
        core_delay_ms(50);
        i2c_measure("100 kHz", &g_imu_i2c, &b100);
        i2c_up(I2C_400K);
        i2c_measure("400 kHz", &g_imu_i2c, &b400);
        i2c_up(I2C_1M);
        i2c_measure("1 MHz", &g_imu_i2c, &b1m);
        /* Its I3C slave is off now: soft reset over I2C turns it back on. */
        tile_sense_i_6p6_reset(&g_imu_i2c);
        core_delay_ms(10);
    }
    uint32_t best = b1m;
    I2C1->CR1 &= ~LL_I2C_CR1_PE;
    /* Back to I3C, whatever the I2C part left: ENTDAA finds the tile without
     * knowing its address state, a soft reset over I3C clears any stray
     * write, then the driver's init over I3C, as check_tile() did. */
    pins_i3c();
    core_i3c_init(&core_i3c1, I3C1, I3C1_SCL_HZ);
    core_i3c_rstdaa(&core_i3c1);
    uint8_t nfound = 0;
    if (core_i3c_entdaa(&core_i3c1, &nfound) != I3C_OK || !nfound) {
        /* Not on I3C: its I3C slave is off (the I2C setting stuck although
         * init reported otherwise). A soft reset over I2C turns it back on. */
        core_i3c_deinit(&core_i3c1);
        i2c_up(I2C_100K);
        tile_sense_i_6p6_reset(&g_imu_i2c);
        core_delay_ms(10);
        I2C1->CR1 &= ~LL_I2C_CR1_PE;
        pins_i3c();
        core_i3c_init(&core_i3c1, I3C1, I3C1_SCL_HZ);
        core_i3c_rstdaa(&core_i3c1);
        nfound = 0;
        core_i3c_entdaa(&core_i3c1, &nfound);
    }
    if (nfound) {
        uint8_t zero = 0, one = 1, tda = core_i3c1.targets[0].dyn_addr;
        core_i3c_write_reg(&core_i3c1, tda, ICM_BANK_SEL, &zero, 1);
        core_i3c_write_reg(&core_i3c1, tda, 0x11, &one, 1);   /* DEVICE_CONFIG soft reset */
        core_delay_ms(10);
    }
    core_i3c_rstdaa(&core_i3c1);
    tile_sense_i_6p6_init(core_tiles_pal(&core_i3c1), 0, imu, NULL);
    uint8_t da = icm_da();
    core_usb_printf("TPUT tile reset over I2C, driver init over I3C again: state %d\n", (int)imu->state);
    (void)b100; (void)b400;
    core_usb_printf("TPUT back on I3C: DA 0x%02x; I3C / I2C at 1 MHz = %lu.%01lux\n", da,
                    (unsigned long)(best ? b14 / best : 0), (unsigned long)(best ? (b14 * 10 / best) % 10 : 0));
    verdict(f14 == 0 && f64 == 0 && b14 > 0 && sg1 == I3C_OK && ss == I3C_OK && mrl1 == 64 &&
            da && imu->state == TILE_STATE_READY,
            "TPUT I3C bursts error-free, bus and tile restored");
}

/* ---- IBI ---- */

static volatile uint32_t g_ibi, g_ibi_len, g_ibi_mdb;
static volatile uint8_t  g_ibi_flag;
static void on_ibi(void *ctx, uint8_t da, const uint8_t *p, uint8_t n)
{
    (void)ctx; (void)da;
    g_ibi++;
    g_ibi_len = n;
    g_ibi_mdb = n ? p[0] : 0;
    g_ibi_flag = 1;
}

static uint32_t count_ibi(uint32_t ms, uint8_t read_on_ibi, uint32_t *samples)
{
    int16_t a[3];
    *samples = 0;
    g_ibi = 0;
    uint32_t t0 = core_millis();
    while ((uint32_t)(core_millis() - t0) < ms) {
        feed();
        if (read_on_ibi && g_ibi_flag) {
            g_ibi_flag = 0;
            tile_sense_i_6p6_get_raw_accels(imu, a);
            (*samples)++;
        }
    }
    return g_ibi;
}

static void check_ibi(void)
{
    uint8_t da = icm_da();
    tile_sense_i_6p6_set_accel_odr(imu, SENSE_I_6P6_ODR_100HZ);
    tile_sense_i_6p6_set_gyro_odr(imu, SENSE_I_6P6_ODR_100HZ);
    tile_sense_i_6p6_write_reg(imu, 4, ICM_INT_SRC8, 0x08);        /* UI data-ready -> IBI */
    hal_status_t s1 = core_i3c_ibi_enable(&core_i3c1, da, on_ibi, 0);
    hal_status_t s2 = core_i3c_enec(&core_i3c1, da, LL_I3C_EVT_INT);
    uint32_t smp;
    uint32_t n100 = count_ibi(2000, 0, &smp);
    tile_sense_i_6p6_set_accel_odr(imu, SENSE_I_6P6_ODR_1KHZ);
    tile_sense_i_6p6_set_gyro_odr(imu, SENSE_I_6P6_ODR_1KHZ);
    uint32_t n1k = count_ibi(2000, 0, &smp);
    uint32_t n1kr = count_ibi(2000, 1, &smp);
    core_usb_printf("IBI enable=%d ENEC=%d: 100 Hz ODR %lu IBI/s; 1 kHz ODR %lu IBI/s; 1 kHz with an "
                    "accel read per IBI %lu IBI/s, %lu reads/s; last MDB 0x%02lx (%lu B payload)\n",
                    (int)s1, (int)s2, (unsigned long)(n100 / 2), (unsigned long)(n1k / 2),
                    (unsigned long)(n1kr / 2), (unsigned long)(smp / 2), (unsigned long)g_ibi_mdb,
                    (unsigned long)g_ibi_len);
    core_i3c_disec(&core_i3c1, da, LL_I3C_EVT_INT);
    core_i3c_ibi_disable(&core_i3c1, da);
    tile_sense_i_6p6_write_reg(imu, 4, ICM_INT_SRC8, 0x00);
    tile_sense_i_6p6_set_accel_odr(imu, SENSE_I_6P6_ODR_100HZ);
    tile_sense_i_6p6_set_gyro_odr(imu, SENSE_I_6P6_ODR_100HZ);
    verdict(s1 == I3C_OK && s2 == I3C_OK && n100 >= 190 && n100 <= 210 && n1k >= 1900 && n1k <= 2100,
            "IBI data-ready rate matches ODR (100 Hz and 1 kHz, +-5 %)");
}

/* ---- ERR ---- */

static void check_err(void)
{
    uint8_t x = 0;
    uint32_t c0 = DWT_CYC;
    hal_status_t a = core_i3c_read(&core_i3c1, 0x33, &x, 1);
    uint32_t ta = us_since(c0);
    uint32_t sa = core_i3c1.last_ser;
    c0 = DWT_CYC;
    hal_status_t b = core_i3c_setdasa(&core_i3c1, 0x55, 0);
    uint32_t tb = us_since(c0);
    core_usb_printf("ERR private read at unassigned DA 0x33 -> %d (SER 0x%03lx) in %lu us; "
                    "SETDASA to absent static 0x55 -> %d in %lu us\n", (int)a, (unsigned long)sa,
                    (unsigned long)ta, (int)b, (unsigned long)tb);
    verdict(a == I3C_NACK && (sa & LL_I3C_SER_ANACK) && b == I3C_NACK, "ERR NACKed addresses reported as I3C_NACK");

    /* The tile loses its dynamic address (RSTDAA); a raw read at it NACKs,
     * and the driver's next call succeeds because the bridge re-assigns it. */
    uint8_t da = icm_da();
    core_i3c_rstdaa(&core_i3c1);
    hal_status_t c = core_i3c_read_reg(&core_i3c1, da, ICM_WHO_AM_I, &x, 1);
    uint8_t who = tile_sense_i_6p6_read_reg(imu, 0, ICM_WHO_AM_I);
    core_usb_printf("ERR after RSTDAA: read at old DA 0x%02x -> %d; driver read_reg WHO_AM_I -> 0x%02x "
                    "(bridge re-assigned DA 0x%02x)\n", da, (int)c, who, icm_da());
    verdict(c == I3C_NACK && who == 0x44, "ERR lost dynamic address recovered by the bridge");

    /* A bus with nothing on it: I3C2 on pads 2 (PA8, AF9) and 3 (PB4, AF10),
     * DS14540 Table 15. Nothing is wired to them on this bench. */
    static core_i3c_t empty;
    uint32_t ma = GPIOA->MODER, pa = GPIOA->PUPDR, ra = GPIOA->AFR[1], sa2 = GPIOA->OSPEEDR;
    uint32_t mb = GPIOB->MODER, pb = GPIOB->PUPDR, rb = GPIOB->AFR[0], ob = GPIOB->OTYPER, sb = GPIOB->OSPEEDR;
    ll_rcc_gpio_clk_enable(GPIOA);
    ll_gpio_config_af(GPIOA, 8, 9, LL_GPIO_OTYPE_PP, LL_GPIO_SPEED_HIGH, LL_GPIO_PULL_UP);
    ll_gpio_config_af(GPIOB, 4, 10, LL_GPIO_OTYPE_PP, LL_GPIO_SPEED_HIGH, LL_GPIO_PULL_UP);
    hal_status_t i = core_i3c_init(&empty, I3C2, I3C_12M5);
    uint8_t n = 0xEE;
    c0 = DWT_CYC;
    hal_status_t r = core_i3c_rstdaa(&empty);
    hal_status_t e = core_i3c_entdaa(&empty, &n);
    hal_status_t p = core_i3c_read(&empty, 0x69, &x, 1);
    uint32_t te = us_since(c0);
    core_usb_printf("ERR empty bus I3C2 (init %d): RSTDAA -> %d, ENTDAA -> %d found %u, private read -> %d; "
                    "all three in %lu us\n", (int)i, (int)r, (int)e, n, (int)p, (unsigned long)te);
    core_i3c_deinit(&empty);
    GPIOA->MODER = ma; GPIOA->PUPDR = pa; GPIOA->AFR[1] = ra; GPIOA->OSPEEDR = sa2;
    GPIOB->AFR[0] = rb; GPIOB->PUPDR = pb; GPIOB->OTYPER = ob; GPIOB->OSPEEDR = sb; GPIOB->MODER = mb;
    verdict(i == I3C_OK && r == I3C_NACK && e == I3C_NACK && n == 0 && p == I3C_NACK && te < 10000,
            "ERR empty bus: NACK, no hang");
}

static void report(void)
{
    g_pass = g_fail = 0;
    core_usb_printf("hw-i3c on %s, SYSCLK %lu Hz, I3C1 kernel %lu Hz\n", TILE_FULL_NAME,
                    (unsigned long)SYSCLK_HZ, (unsigned long)core_i3c1.kernel_hz);
    check_tile();  feed();
    check_daa();   feed();
    check_data();  feed();
    check_tput();  feed();
    check_ibi();   feed();
    check_err();   feed();
    core_usb_printf("DONE %lu pass, %lu fail\n", (unsigned long)g_pass, (unsigned long)g_fail);
}

int main(void)
{
    core_init();     /* max clock, USB CDC, 5 s watchdog, I3C1 up + tile SETDASA */
    DEMCR |= (1UL << 24);
    DWT_CYC = 0;
    DWT_CTRL |= 1UL;

    char line[32];
    uint32_t len = 0;
    int was = 0;
    for (;;) {
        feed();
        /* Bench safety net: a board strapped BOOT0-high has no other way
         * back. No USB address from the host 15 s after boot means USB never
         * came up: reset (into the ROM bootloader with BOOT0 high). */
        if ((USB_DADDR & 0x7FUL) == 0 && core_millis() > 15000u) {
            SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
            for (;;) { }
        }
        int now = core_usb_connected();
        if (now && !was) {
            core_delay_ms(200);  /* let the open-time settle pass */
            report();
        }
        was = now;
        uint8_t b;
        while (core_usb_try_read(&b)) {
            if (b == '\r') continue;
            if (b != '\n') {
                if (len < sizeof line - 1) line[len++] = (char)b;
                continue;
            }
            line[len] = '\0';
            len = 0;
            switch (line[0]) {
            case 'R': report(); break;
            case 'T': check_tile(); break;
            case 'D': check_daa(); break;
            case 'S': check_data(); break;
            case 'P': check_tput(); break;
            case 'I': check_ibi(); break;
            case 'E': check_err(); break;
            case 'V': core_usb_printf("HELLO hw-i3c\n"); break;
            case 'Q':           /* test the safety net: drop off the bus, lose the address */
                USB_BCDR &= ~USB_BCDR_DPPU_DPD;
                USB_DADDR = 0;
                break;
            case 'X':
                SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
                for (;;) { }
            default: break;
            }
        }
    }
}
