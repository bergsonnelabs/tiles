/**
 * hw-thread -- two-node Thread bench test for Core.ST.W5 (one image, both nodes)
 *
 * Flash this same image onto two W5 Cores. Each one joins (or forms) the
 * development network from config.json, then once a second, while attached,
 * broadcasts a UDP datagram to ff03::1 port 1234 carrying a counter and its
 * extended address. Each node counts the datagrams it receives from the other
 * (its own are ignored by sender address). The first node up forms the network
 * and becomes leader; the second attaches as a child and, within ~2 minutes
 * (OpenThread's router upgrade jitter), usually becomes a router.
 *
 * LED (PB12): heartbeats of 2 ms blips (dim, and a blip shows the loop is alive):
 *   detached / disabled   fast blink: a 2 ms flash every 125 ms
 *   child                 one blip a second
 *   router                two blips 150 ms apart, once a second
 *   leader                three blips 150 ms apart, once a second
 *   + one extra blip for each datagram received from the other node, at
 *     receive time. One that lands on (or within 50 ms of) the heartbeat
 *     group waits until 50 ms after it, so it never merges into the rhythm.
 *
 * Status in RAM for SWD (CoreProbe), 19 words at g_thread_status:
 *   make
 *   arm-none-eabi-nm build/hw-thread.elf | grep g_thread_status    # its address
 *   probe-rs read --chip STM32WBA55CGUx b32 <address> 19
 * Words (offsets in bytes):
 *    0 magic 0x54485244 ("THRD")     4 boots (warm resets since power-on)
 *    8 role (0 off, 1 detached, 2 child, 3 router, 4 leader)
 *   12 rloc16 (0xFFFE detached)      16 partition id
 *   20 tx count (broadcasts queued)  24 rx count (from the other node)
 *   28 last rx counter (the other node's tx count in its last datagram)
 *   32 last error (start: CORE_THREAD_ERR_*, else core_thread_last_error)
 *   36 tx failures                   40 neighbour count
 *   44 parent RSSI dBm (127: not a child)
 *   48 uptime ms                     52 settings snapshots written to flash
 *   56 fatal: reason << 16 | arg low 16 bits, from the run before (0: none)
 *   60 last rx: sender's rloc16      64/68 own extended address (bytes 0-3, 4-7)
 *   72 role changes
 * Healthy pair after a minute: roles 4 and 3 (or 4 and 2), the same partition
 * id on both, tx and rx climbing by ~1 a second, last rx counter tracking the
 * other node's tx count, last error 0.
 */

#include <string.h>
#include "core.h"
#include "core_thread.h"
#include "core_watchdog.h"
#include "thread_app.h"          /* thread_fatal_info, settings diagnostics */

#define UDP_PORT      1234u
#define STATUS_MAGIC  0x54485244UL   /* "THRD" */

typedef struct {
    uint32_t magic;
    uint32_t boots;
    uint32_t role;
    uint32_t rloc16;
    uint32_t partition_id;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t last_rx_counter;
    int32_t  last_error;
    uint32_t tx_fail;
    uint32_t neighbors;
    int32_t  parent_rssi;
    uint32_t uptime_ms;
    uint32_t settings_saves;
    uint32_t fatal;
    uint32_t last_rx_rloc16;
    uint32_t ext_lo;
    uint32_t ext_hi;
    uint32_t role_changes;
} thread_status_t;

/* .noinit: `boots` survives a warm reset (a watchdog reset shows up there);
 * everything else is set at boot. */
volatile thread_status_t g_thread_status __attribute__((section(".noinit")));

/* Datagram: "TH", version 1, 0, tx counter (LE), sender rloc16 (LE), 2 spare,
 * sender extended address. */
typedef struct __attribute__((packed)) {
    uint8_t  tag[2];
    uint8_t  version;
    uint8_t  reserved;
    uint32_t counter;
    uint16_t rloc16;
    uint16_t spare;
    uint8_t  ext[8];
} beacon_t;

static uint8_t s_ext[8];
static volatile uint8_t s_rx_blips;   /* receive blips waiting to be shown */
static uint32_t s_rx_blip_until;       /* end of the receive blip being shown */

static void on_rx(const core_thread_udp_packet_t *pkt, void *ctx)
{
    const uint8_t *data = pkt->data;
    uint16_t len = pkt->len;
    (void)ctx;
    beacon_t b;
    if (len < sizeof b)
        return;
    memcpy(&b, data, sizeof b);
    if (b.tag[0] != 'T' || b.tag[1] != 'H' || b.version != 1)
        return;
    if (memcmp(b.ext, s_ext, sizeof s_ext) == 0)
        return;                                /* our own broadcast */
    g_thread_status.rx_count++;
    g_thread_status.last_rx_counter = b.counter;
    g_thread_status.last_rx_rloc16 = b.rloc16;
    if (s_rx_blips < 4u)
        s_rx_blips++;
}

static void on_role(core_thread_role_t role, void *ctx)
{
    (void)ctx;
    g_thread_status.role = (uint32_t)role;
    g_thread_status.role_changes++;
}

#define BLIP_MS      2u      /* LED on time per blip */
#define BLIP_GAP_MS  150u    /* between the blips of a heartbeat group */
#define RX_GUARD_MS  50u     /* receive blips keep this far from the group */

/* Heartbeat blips per second: 1 child, 2 router, 3 leader (0: detached). */
static uint32_t heartbeat_blips(core_thread_role_t role)
{
    if (role == CORE_THREAD_LEADER) return 3u;
    if (role == CORE_THREAD_ROUTER) return 2u;
    if (role == CORE_THREAD_CHILD)  return 1u;
    return 0u;
}

/* Non-blocking: called every pass of the main loop (<= 1 ms apart). */
static void led_update(uint32_t now, core_thread_role_t role)
{
    uint32_t n = heartbeat_blips(role);
    int on;

    if (n == 0u) {
        on = (now % 125u) < BLIP_MS;
        s_rx_blips = 0;
    } else {
        uint32_t phase = now % 1000u;
        uint32_t group_end = (n - 1u) * BLIP_GAP_MS + BLIP_MS;     /* the group is [0, group_end) */
        on = phase < group_end && (phase % BLIP_GAP_MS) < BLIP_MS;

        /* A receive blip, one at a time, kept clear of the heartbeat group
         * (and of the next one, which starts at phase 1000). */
        if ((int32_t)(now - s_rx_blip_until) > 1000)
            s_rx_blip_until = now - 1000u;                    /* stay in signed range */
        int busy = (int32_t)(now - s_rx_blip_until) < (int32_t)RX_GUARD_MS;
        int near_group = phase < group_end + RX_GUARD_MS || phase >= 1000u - RX_GUARD_MS;
        if (s_rx_blips != 0u && !busy && !near_group) {
            s_rx_blips--;
            s_rx_blip_until = now + BLIP_MS;
        }
        if ((int32_t)(now - s_rx_blip_until) < 0)
            on = 1;
    }
    if (on)
        LED_ON();
    else
        LED_OFF();
}

static void send_beacon(void)
{
    beacon_t b;
    memset(&b, 0, sizeof b);
    b.tag[0] = 'T';
    b.tag[1] = 'H';
    b.version = 1;
    b.counter = g_thread_status.tx_count + 1u;
    b.rloc16 = core_thread_rloc16();
    memcpy(b.ext, s_ext, sizeof s_ext);

    int rc = core_thread_udp_broadcast(UDP_PORT, &b, sizeof b);
    if (rc == 0) {
        g_thread_status.tx_count++;
    } else {
        g_thread_status.tx_fail++;
        g_thread_status.last_error = (rc == CORE_THREAD_ERR_FAILED) ? core_thread_last_error() : rc;
    }
}

int main(void)
{
    core_init();          /* clock + 5 s watchdog, from config.json */
    core_led_init();

    /* Status block: keep the boot count across warm resets. */
    uint32_t boots = (g_thread_status.magic == STATUS_MAGIC) ? g_thread_status.boots + 1u : 1u;
    memset((void *)&g_thread_status, 0, sizeof g_thread_status);
    g_thread_status.boots = boots;
    g_thread_status.rloc16 = 0xFFFEu;
    g_thread_status.parent_rssi = 127;
    if (thread_fatal_info.magic == THREAD_FATAL_MAGIC) {
        g_thread_status.fatal = (thread_fatal_info.reason << 16) | (thread_fatal_info.arg & 0xFFFFu);
        thread_fatal_info.magic = 0;
    }
    g_thread_status.magic = STATUS_MAGIC;

    core_thread_on_role_change(on_role, NULL);
    int rc = core_thread_start();
    if (rc != 0)
        g_thread_status.last_error = rc;
    rc = core_thread_udp_open(UDP_PORT, on_rx, NULL);
    if (rc != 0 && g_thread_status.last_error == 0)
        g_thread_status.last_error = rc;

    core_thread_ext_addr(s_ext);
    g_thread_status.ext_lo = (uint32_t)s_ext[0] | ((uint32_t)s_ext[1] << 8) |
                             ((uint32_t)s_ext[2] << 16) | ((uint32_t)s_ext[3] << 24);
    g_thread_status.ext_hi = (uint32_t)s_ext[4] | ((uint32_t)s_ext[5] << 8) |
                             ((uint32_t)s_ext[6] << 16) | ((uint32_t)s_ext[7] << 24);

    uint32_t next_tx = core_millis() + 1000u;
    while (1) {
        core_watchdog_feed();
        core_thread_process();           /* returns after the next interrupt (<= 1 ms) */

        uint32_t now = core_millis();
        core_thread_role_t role = core_thread_role();
        led_update(now, role);

        if ((int32_t)(now - next_tx) >= 0) {
            next_tx += 1000u;
            if (role >= CORE_THREAD_CHILD)
                send_beacon();
            g_thread_status.role = (uint32_t)role;
            g_thread_status.rloc16 = core_thread_rloc16();
            g_thread_status.partition_id = core_thread_partition_id();
            g_thread_status.neighbors = core_thread_neighbor_count();
            g_thread_status.parent_rssi = core_thread_parent_rssi();
            g_thread_status.settings_saves = thread_settings_save_count();
            g_thread_status.uptime_ms = now;
        }
    }
}
