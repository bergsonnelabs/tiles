/**
 * hw-usb-robust — USB CDC robustness and USB-reachable sleep on Core.ST.L4.
 * Driven from the host by host_test.py (python3 + pyserial); see README.md.
 *
 * The Core prints "EARLY ..." right after core_init(), before any delay: with
 * the TX FIFO holding text written before the port opens, that line must be
 * the first thing a terminal sees. Then it answers one command per line:
 *
 *   I          identity: "ID serial=<24 hex UID> ..."; the host compares it
 *              with the USB serial number the OS reports
 *   S <s>      stall: print as fast as possible for <s> seconds while the host
 *              doesn't read; times every write (DWT cycle counter)
 *   R          report the stall run: "SRES max_us=... calls=... short=..."
 *   B <n>      receive <n> bytes, reading slowly (16 B per ms) so the 256 B
 *              ring fills and the endpoint has to NAK; "BRES n=... fnv=..."
 *   Z <n>      one write of exactly <n> bytes (n-1 'z' and '\n'), for the
 *              ZLP check: it must arrive without a following write
 *   D <k>      wait for DTR to drop, write <k> lines while it is low (timed),
 *              wait for DTR to come back; "DRES ..." after the queued lines
 *   W <n>      n x core_stop_for(1) with Stop forced as if the bus were
 *              suspended (test hook); run with the port closed. A Stop that
 *              ends early was ended by the USB wakeup (EXTI line 17)
 *   X          reset (the host then checks the EARLY line again)
 *   P <s>      sleep mode, until reset: loop core_stop_for(<s>) and print a
 *              counter; an RX callback answers "PONG" to each line from
 *              the interrupt while the main loop is asleep ("X" resets)
 */

#include "core.h"
#include "core_usb.h"
#include "core_power.h"
#include "core_watchdog.h"
#include "core_uid.h"
#include <string.h>
#include <stdlib.h>

#define DEMCR     REG32(0xE000EDFCUL)   /* TRCENA = bit 24 */
#define DWT_CTRL  REG32(0xE0001000UL)   /* CYCCNTENA = bit 0 */
#define DWT_CYC   REG32(0xE0001004UL)
#define CYC_PER_US  (SYSCLK_HZ / 1000000UL)

static struct { uint32_t max_cyc, calls, short_writes; } g_stall;

static void cyc_init(void)
{
    DEMCR |= (1UL << 24);
    DWT_CYC = 0;
    DWT_CTRL |= 1UL;
}

/* One timed core_usb_write(); returns the cycles it took. */
static uint32_t timed_write(const char *s, int n, int *short_write)
{
    uint32_t c0 = DWT_CYC;
    int r = core_usb_write((const uint8_t *)s, (uint16_t)n);
    uint32_t dc = DWT_CYC - c0;
    *short_write = (r < n);
    return dc;
}

static void cmd_identity(void)
{
    char uid[25];
    core_uid_hex(uid, sizeof uid);
    core_usb_printf("ID serial=%s tx_timeout_ms=%d\n", uid, HAL_USB_CDC_TX_TIMEOUT_MS);
}

static void cmd_stall(uint32_t secs)
{
    char msg[72];
    memset(&g_stall, 0, sizeof g_stall);
    uint32_t t0 = core_millis();
    while ((uint32_t)(core_millis() - t0) < secs * 1000UL) {
        int n = snprintf(msg, sizeof msg,
                         "S %08lu the host is not reading this line............\n",
                         (unsigned long)g_stall.calls);
        int sh;
        uint32_t dc = timed_write(msg, n, &sh);
        if (dc > g_stall.max_cyc) g_stall.max_cyc = dc;
        g_stall.calls++;
        g_stall.short_writes += (uint32_t)sh;
        core_watchdog_feed();
    }
}

static uint32_t fnv1a(uint32_t h, const uint8_t *p, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619UL; }
    return h;
}

static void cmd_burst(uint32_t want)
{
    uint32_t got = 0, h = 2166136261UL, t0 = core_millis();
    uint8_t buf[16];
    core_usb_printf("BREADY %lu\n", (unsigned long)want);
    while (got < want && (uint32_t)(core_millis() - t0) < 20000UL) {
        uint16_t take = (want - got) < sizeof buf ? (uint16_t)(want - got) : sizeof buf;
        uint16_t n = core_usb_read(buf, take);
        h = fnv1a(h, buf, n);
        got += n;
        core_delay_ms(1);                   /* slow reader: the ring fills */
        core_watchdog_feed();
    }
    core_usb_printf("BRES n=%lu fnv=%08lx ms=%lu\n", (unsigned long)got,
                    (unsigned long)h, (unsigned long)(core_millis() - t0));
}

static void cmd_zlp(uint32_t n)
{
    static char buf[256];
    if (n < 1 || n > sizeof buf) return;
    memset(buf, 'z', n - 1);
    buf[n - 1] = '\n';
    core_usb_write((const uint8_t *)buf, (uint16_t)n);
}

static void cmd_dtr(uint32_t k)
{
    uint32_t t0 = core_millis();
    while (core_usb_connected() && (uint32_t)(core_millis() - t0) < 5000UL)
        core_watchdog_feed();
    int dropped = !core_usb_connected();
    uint32_t max_cyc = 0, shorts = 0;
    char msg[32];
    for (uint32_t i = 0; i < k; i++) {
        int n = snprintf(msg, sizeof msg, "Q %lu while closed\n", (unsigned long)i);
        int sh;
        uint32_t dc = timed_write(msg, n, &sh);
        if (dc > max_cyc) max_cyc = dc;
        shorts += (uint32_t)sh;
    }
    t0 = core_millis();
    while (!core_usb_connected() && (uint32_t)(core_millis() - t0) < 10000UL)
        core_watchdog_feed();
    core_usb_printf("DRES dtr_dropped=%d max_us=%lu short=%lu back=%d\n", dropped,
                    (unsigned long)(max_cyc / CYC_PER_US), (unsigned long)shorts,
                    core_usb_connected());
}

/* Sleep mode: the RX callback runs in the USB interrupt while main() sits in
 * core_stop_for(). Its write never waits (interrupt context). */
static volatile uint32_t g_pings;
static void pong_cb(const uint8_t *data, uint16_t len, void *ctx)
{
    (void)ctx;
    for (uint16_t i = 0; i < len; i++) {
        if (data[i] == 'X') {               /* way out of sleep mode */
            SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
            for (;;) { }
        }
        if (data[i] == '\n') {
            g_pings++;
            static const char pong[] = "PONG\n";
            core_usb_write((const uint8_t *)pong, sizeof pong - 1);
        }
    }
}

static void cmd_sleep(uint32_t secs) __attribute__((noreturn));
static void cmd_sleep(uint32_t secs)
{
    core_usb_on_receive(pong_cb, 0);
    for (uint32_t k = 0;; k++) {
        core_usb_printf("SLEEP k=%lu for %lu s (suspended=%d)\n", (unsigned long)k,
                        (unsigned long)secs, hal_usb_cdc_suspended());
        uint32_t c0 = DWT_CYC, t0 = core_millis();
        core_stop_for(secs);
        uint32_t dc = DWT_CYC - c0, dt = core_millis() - t0;
        /* DWT counts in Sleep (HCLK runs) but not in Stop: cycles per ms near
         * SYSCLK/1000 means the wait was in Sleep mode. */
        uint32_t stops, wakes;
        hal_usb_cdc_test_stats(&stops, &wakes);   /* HAL_USB_CDC_TEST_HOOKS */
        core_usb_printf("WOKE k=%lu slept_ms=%lu cyc_per_ms=%lu (full %lu) pings=%lu "
                        "usb_stops=%lu usb_wakes=%lu\n",
                        (unsigned long)k, (unsigned long)dt,
                        (unsigned long)(dt ? dc / dt : 0), (unsigned long)(SYSCLK_HZ / 1000UL),
                        (unsigned long)g_pings, (unsigned long)stops, (unsigned long)wakes);
        const uint32_t *lg;
        uint32_t nl = hal_usb_cdc_test_log(&lg);
        for (uint32_t i = 0; i < nl; i++)
            core_usb_printf("  usb event %c%s at %lu ms\n", (char)(lg[i] & 0x7Fu),
                            (lg[i] & 0x80u) ? "+W" : "", (unsigned long)(lg[i] >> 8));
        core_watchdog_feed();
    }
}

/* DTR history, for the early-print check: when did a terminal appear? */
static uint32_t g_dtr_log[8], g_dtr_n;
static int g_dtr_last;
static void dtr_track(void)
{
    int c = core_usb_connected();
    if (c != g_dtr_last) {
        g_dtr_last = c;
        if (g_dtr_n < 8) g_dtr_log[g_dtr_n++] = (core_millis() << 1) | (uint32_t)c;
    }
}

/* W <n>: n x core_stop_for(1), each with its Stop taken as if the bus were
 * suspended (test hook). A Stop that ends long before the 1 s RTC wake was
 * ended by the asynchronous USB wakeup (EXTI line 17): nothing else runs in
 * Stop. Run with the port CLOSED: with it open, the host's IN tokens go
 * unanswered during the Stop and macOS resets the port. Output is queued and
 * read when the port reopens. */
static void cmd_usb_wake(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        uint32_t s0, w0, s1, w1;
        hal_usb_cdc_test_stats(&s0, &w0);
        hal_usb_cdc_test_force_suspend();
        uint32_t c0 = DWT_CYC, t0 = core_millis();
        core_stop_for(1);
        uint32_t dt = core_millis() - t0, cpm = dt ? (DWT_CYC - c0) / dt : 0;
        hal_usb_cdc_test_stats(&s1, &w1);
        core_usb_printf("W %lu stops=%lu wakes=%lu ms=%lu cyc_per_ms=%lu\n", (unsigned long)i,
                        (unsigned long)(s1 - s0), (unsigned long)(w1 - w0),
                        (unsigned long)dt, (unsigned long)cpm);
        core_watchdog_feed();
    }
}

static void dispatch(char *line)
{
    char c = line[0];
    uint32_t arg = (uint32_t)strtoul(line + 1, 0, 10);
    switch (c) {
    case 'I': cmd_identity(); break;
    case 'S': cmd_stall(arg ? arg : 30u); break;
    case 'R':
        core_usb_printf("SRES max_us=%lu calls=%lu short=%lu\n",
                        (unsigned long)(g_stall.max_cyc / CYC_PER_US),
                        (unsigned long)g_stall.calls, (unsigned long)g_stall.short_writes);
        break;
    case 'B': cmd_burst(arg ? arg : 4096u); break;
    case 'Z': cmd_zlp(arg); break;
    case 'D': cmd_dtr(arg ? arg : 5u); break;
    case 'X':
        SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
        for (;;) { }
    case 'P': cmd_sleep(arg ? arg : 60u);
    case 'V': core_usb_printf("HELLO hw-usb-robust\n"); break;
    case 'W': cmd_usb_wake(arg ? arg : 6u); break;
    case 'L':
        for (uint32_t i = 0; i < g_dtr_n; i++)
            core_usb_printf("DTR %s at %lu ms\n", (g_dtr_log[i] & 1u) ? "up" : "down",
                            (unsigned long)(g_dtr_log[i] >> 1));
        core_usb_printf("LEND\n");
        break;
    default:  break;
    }
}

int main(void)
{
    core_init();                 /* arms the 5 s watchdog (config.json) */
    /* No startup delay: this must reach the host once the port opens. */
    core_usb_printf("EARLY boot, printed at t=%lu ms, before enumeration\n",
                    (unsigned long)core_millis());
    core_usb_init();             /* as many programs do: must not reset USB or drop EARLY */
    cyc_init();

    char line[48];
    uint32_t len = 0;
    for (;;) {
        core_watchdog_feed();
        dtr_track();
        uint8_t b;
        while (core_usb_try_read(&b)) {
            if (b == '\r') continue;
            if (b == '\n') {
                line[len] = '\0';
                if (len) dispatch(line);
                len = 0;
            } else if (len < sizeof line - 1) {
                line[len++] = (char)b;
            }
        }
    }
}
