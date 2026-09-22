/**
 * hw-adc-mux-host — I2C host for the l0-adc-mux bring-up.
 *
 * A Core.ST.L4.1 on I2C1 (pads 4/5) talks to a Core.ST.L0.1 running
 * projects/l0-adc-mux at 0x28 and reports over USB-CDC. This is the first
 * time the hub is exercised over a real bus: until now it was driven by
 * writing its RAM over SWD, which bypassed the target ISR's byte path and
 * stalled its CPU on every read, so the publish rate could not be measured.
 *
 * Open the serial port (any baud) and the sequence runs once:
 *   1. bus scan
 *   2. identity and a full register dump
 *   3. data snapshots
 *   4. publish rate and error count at 100 kHz, then at 400 kHz
 *   5. rate/window changes written over I2C, including a rejected one
 * then prints one live line per second until the port closes. Reopen it
 * to run the sequence again.
 *
 * The publish rate is timed on this Core's clock, so it carries both
 * boards' oscillator error: a result within a percent or two of the
 * requested rate is a pass.
 */

#include "core.h"
#include "core_i2c.h"
#include "core_usb.h"

#define HUB             0x28
#define P               core_usb_printf

/* Hub registers (projects/l0-adc-mux/main.c, v1.2) */
#define R_WHO_AM_I      0x00
#define R_VERSION       0x01
#define R_STATUS        0x02
#define R_VDDA          0x10
#define R_ADDR_CUR      0x12
#define R_RATE          0x15
#define R_APPLY         0x18
#define R_COUNT         0x19

#define APPLY_REQUEST   0x5A

static unsigned pass_n, fail_n;

static void check(int ok, const char *what)
{
    if (ok) { pass_n++; P("  [PASS] %s\r\n", what); }
    else    { fail_n++; P("  [FAIL] %s\r\n", what); }
}

static hal_status_t rd(uint8_t reg, uint8_t *buf, uint32_t len)
{
    return core_i2c_read_reg(&core_i2c1, HUB, reg, buf, len);
}

static void print_data(const uint8_t *d)   /* 14 bytes from 0x02 */
{
    P("  status=0x%02X seq=%3u  mV:", d[0], d[1]);
    for (int i = 0; i < 6; i++) {
        P(" %4u", (unsigned)(d[2 + 2 * i] | (d[3 + 2 * i] << 8)));
    }
    P("\r\n");
}

typedef struct {
    uint32_t reads, errors, published, repeats, gaps, ms;
} rate_t;

/**
 * Poll STATUS+SEQ every ~3 ms and count how far SEQ advances. Polling
 * several times faster than the hub publishes means each publish is seen
 * once; a jump of more than one ("gap") means the host itself missed one.
 */
static rate_t measure(uint32_t duration_ms)
{
    rate_t r = {0};
    uint8_t b[2], prev = 0;
    int have = 0;
    uint32_t t0 = core_millis();

    while ((r.ms = core_millis() - t0) < duration_ms) {
        r.reads++;
        if (rd(R_STATUS, b, 2) != HAL_OK) {
            r.errors++;
        } else {
            if (have) {
                uint8_t d = (uint8_t)(b[1] - prev);
                if (d == 0) r.repeats++;
                else { r.published += d; if (d > 1) r.gaps++; }
            }
            prev = b[1];
            have = 1;
        }
        core_delay_ms(3);
    }
    return r;
}

/** Publish rate in hundredths of a hertz. */
static uint32_t centi_hz(const rate_t *r)
{
    return r->ms ? (uint32_t)((uint64_t)r->published * 100000u / r->ms) : 0;
}

static void report_rate(const char *label, uint32_t want_hz, uint32_t ms)
{
    rate_t r = measure(ms);
    uint32_t c = centi_hz(&r);
    P("  %s: %lu.%02lu Hz over %lu ms  (%lu reads, %lu errors, %lu gaps)\r\n",
      label, c / 100, c % 100, r.ms, r.reads, r.errors, r.gaps);

    /* within 2% of the request, counting both boards' clocks */
    uint32_t lo = want_hz * 98, hi = want_hz * 102;
    check(r.errors == 0, "no bus errors");
    check(c >= lo && c <= hi, "publish rate within 2% of request");
}

/** Write rate/oversamp/window + APPLY in one transaction; return APPLY's answer. */
static uint8_t apply(uint8_t rate, uint8_t oversamp, uint8_t window)
{
    uint8_t w[4] = { rate, oversamp, window, APPLY_REQUEST };
    if (core_i2c_write_reg(&core_i2c1, HUB, R_RATE, w, 4) != HAL_OK) return 0xFF;
    core_delay_ms(50);                 /* applied within one output period */
    uint8_t a = 0xFF;
    rd(R_APPLY, &a, 1);
    return a;
}

static void run_sequence(void)
{
    uint8_t reg[R_COUNT], d[14];
    pass_n = fail_n = 0;

    P("\r\n=== hw-adc-mux-host: L4.1 reading l0-adc-mux at 0x%02X ===\r\n", HUB);

    /* 1. Scan */
    P("\r\n--- bus scan (100 kHz) ---\r\n");
    uint8_t found[16], n = 0;
    core_i2c_scan(&core_i2c1, found, &n, sizeof(found));
    P("  found %u:", n);
    int hub_seen = 0;
    for (uint8_t i = 0; i < n && i < sizeof(found); i++) {
        P(" 0x%02X", found[i]);
        if (found[i] == HUB) hub_seen = 1;
    }
    P("\r\n");
    check(hub_seen, "hub answers at 0x28");
    if (!hub_seen) { P("  stopping: nothing to talk to\r\n"); return; }

    /* 2. Identity and full dump: one burst over every register, which
     * also exercises auto-increment across the whole map. */
    P("\r\n--- identity ---\r\n");
    hal_status_t s = rd(0x00, reg, R_COUNT);
    check(s == HAL_OK, "25-byte burst read");
    P("  dump:");
    for (int i = 0; i < R_COUNT; i++) P("%s%02X", (i % 8) ? " " : "\r\n    ", reg[i]);
    P("\r\n");
    check(reg[R_WHO_AM_I] == 0x6D, "WHO_AM_I = 0x6D");
    check(reg[R_VERSION]  == 0x12, "VERSION = 0x12 (v1.2)");
    check(reg[R_ADDR_CUR] == HUB,  "ADDR_CUR = 0x28");
    unsigned vdda = reg[R_VDDA] | (reg[R_VDDA + 1] << 8);
    P("  VDDA %u mV, settings %u/%u/%u\r\n", vdda, reg[0x15], reg[0x16], reg[0x17]);
    check(vdda > 1700 && vdda < 3700, "VDDA plausible");

    /* 3. Data */
    P("\r\n--- data (pointer 0x02, 14-byte burst) ---\r\n");
    for (int i = 0; i < 3; i++) {
        if (rd(R_STATUS, d, 14) == HAL_OK) print_data(d);
        core_delay_ms(20);
    }
    check((d[0] & 0x07) == 0x07, "STATUS valid + DMA + VDDA");

    /* 4. Rate and errors at both bus speeds */
    P("\r\n--- publish rate, 100 kHz bus ---\r\n");
    report_rate("100/8/1", 100, 5000);

    P("\r\n--- publish rate, 400 kHz bus ---\r\n");
    core_i2c_init(&core_i2c1, I2C1, I2C_400K);
    report_rate("100/8/1", 100, 5000);

    /* 5. Settings over I2C */
    P("\r\n--- settings written over I2C ---\r\n");
    check(apply(100, 8, 2) == 0x01, "APPLY 100/8/2 (50 Hz reject)");
    core_delay_ms(100);
    report_rate("100/8/2", 100, 3000);

    check(apply(120, 8, 2) == 0x01, "APPLY 120/8/2 (60 Hz reject)");
    core_delay_ms(100);
    report_rate("120/8/2", 120, 3000);

    check(apply(100, 32, 2) == 0xE1, "APPLY 100/32/2 rejected (64 scans)");
    rd(R_RATE, reg, 3);
    check(reg[0] == 120 && reg[1] == 8 && reg[2] == 2,
          "rejected APPLY restores 120/8/2");

    check(apply(100, 8, 1) == 0x01, "APPLY 100/8/1 (back to default)");

    core_i2c_init(&core_i2c1, I2C1, I2C_100K);
    P("\r\n=== %u passed, %u failed ===\r\n", pass_n, fail_n);
    P("live readings follow, once a second; close the port to stop\r\n\r\n");
}

int main(void)
{
    core_init();   /* clock, pads, I2C1 @ 100 kHz, USB-CDC */

    for (;;) {
        while (!core_usb_connected()) core_delay_ms(20);
        core_delay_ms(300);            /* let the host finish opening */
        run_sequence();

        uint8_t d[14];
        while (core_usb_connected()) {
            if (rd(R_STATUS, d, 14) == HAL_OK) print_data(d);
            else P("  read failed\r\n");
            core_delay_ms(1000);
        }
    }
}
