/**
 * hw-ble-thread -- BLE + Thread bench test for Core.ST.W5 (one image, both radios)
 *
 * The Thread side is tests/hw-thread unchanged: flash this image onto two W5
 * Cores (or one, next to a hw-thread node: the datagrams are the same). Each
 * joins (or forms) the development network from config.json, then once a
 * second, while attached, broadcasts a UDP datagram to ff03::1 port 1234
 * carrying a counter and its extended address, and counts the datagrams it
 * receives from other nodes (its own are ignored by sender address).
 *
 * The BLE side runs at the same time: the Core advertises as
 * "Core.ST.W5 Thread" with one service whose characteristic a phone can read,
 * and subscribe to (notified once a second while subscribed):
 *   service         00007400-8E22-4541-9D4C-21EDAE82ED19
 *   characteristic  00007401-8E22-4541-9D4C-21EDAE82ED19  read + notify, 8 bytes:
 *     [0] Thread role (0 off, 1 detached, 2 child, 3 router, 4 leader)
 *     [1] neighbour count
 *     [2..3] rloc16, little-endian (0xFFFE detached)
 *     [4..7] rx count (datagrams from other nodes), little-endian
 * (nRF Connect: connect, open the 7400 service, read or enable notifications
 * on 7401.) The Studio Link service is there too, as in every BLE build. A
 * phone that sees the name, reads a role of 2-4 and watches the rx count climb
 * has seen both radios work in one image.
 *
 * LED (PB12): heartbeats of 2 ms blips (dim, and a blip shows the loop is alive):
 *   detached / disabled   fast blink: a 2 ms flash every 125 ms
 *   child                 one blip a second
 *   router                two blips 150 ms apart, once a second
 *   leader                three blips 150 ms apart, once a second
 *   + one extra blip for each datagram received from another node, at
 *     receive time. One that lands on (or within 50 ms of) the heartbeat
 *     group waits until 50 ms after it, so it never merges into the rhythm.
 *
 * Status in RAM for SWD (CoreProbe), 25 words at g_ble_thread_status:
 *   make
 *   arm-none-eabi-nm build/hw-ble-thread.elf | grep g_ble_thread_status    # its address
 *   probe-rs read --chip STM32WBA55CGUx b32 <address> 25
 * Words 0-18 are hw-thread's g_thread_status (offsets in bytes), with its own
 * magic:
 *    0 magic 0x42544844 ("BTHD")     4 boots (warm resets since power-on)
 *    8 role (0 off, 1 detached, 2 child, 3 router, 4 leader)
 *   12 rloc16 (0xFFFE detached)      16 partition id
 *   20 tx count (broadcasts queued)  24 rx count (from other nodes)
 *   28 last rx counter (the other node's tx count in its last datagram)
 *   32 last error (start: CORE_THREAD_ERR_*, else core_thread_last_error)
 *   36 tx failures                   40 neighbour count
 *   44 parent RSSI dBm (127: not a child)
 *   48 uptime ms                     52 settings snapshots written to flash
 *   56 fatal: reason << 16 | arg low 16 bits, from the run before (0: none)
 *   60 last rx: sender's rloc16      64/68 own extended address (bytes 0-3, 4-7)
 *   72 role changes
 * then BLE:
 *   76 BLE connected (0 / 1)         80 BLE connections since boot
 *   84 notifications sent            88 advertise result (0 ok, -1 failed)
 *   92 status-characteristic value refreshes (once a second)
 * and the stack:
 *   96 stack peak, bytes: the deepest the stack has reached since boot (RAM
 *      above the heap is painted at boot and checked every 10 s). The BLE +
 *      Thread linker script reserves 12 KB; free RAM beyond it absorbs more,
 *      but a peak near or past 12288 means the reserve is too small.
 * Healthy after a minute: the hw-thread numbers (roles 4 and 3 or 4 and 2 on
 * a pair, the same partition id, tx / rx climbing ~1 a second, last error 0),
 * advertise result 0, and with a phone connected and subscribed: connected 1,
 * notifications climbing ~1 a second; stack peak well under 12288.
 */

#include <string.h>
#include "core.h"
#include "core_ble.h"
#include "core_thread.h"
#include "core_watchdog.h"
#include "thread_app.h"          /* thread_fatal_info, settings diagnostics */

#define UDP_PORT      1234u
#define STATUS_MAGIC  0x42544844UL   /* "BTHD" */
#define BLE_NAME      "Core.ST.W5 Thread"
#define BLE_SVC_ID    0x7400u
#define BLE_CHAR_ID   0x7401u

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
    /* BLE (not in hw-thread) */
    uint32_t ble_connected;
    uint32_t ble_connects;
    uint32_t ble_notifies;
    int32_t  ble_adv_result;
    uint32_t ble_updates;
    uint32_t stack_peak;
} thread_status_t;

/* .noinit: `boots` survives a warm reset (a watchdog reset shows up there);
 * everything else is set at boot. */
volatile thread_status_t g_ble_thread_status __attribute__((section(".noinit")));

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

static void on_rx(const uint8_t *data, uint16_t len, const uint8_t src[16], uint16_t port, void *ctx)
{
    (void)src;
    (void)port;
    (void)ctx;
    beacon_t b;
    if (len < sizeof b)
        return;
    memcpy(&b, data, sizeof b);
    if (b.tag[0] != 'T' || b.tag[1] != 'H' || b.version != 1)
        return;
    if (memcmp(b.ext, s_ext, sizeof s_ext) == 0)
        return;                                /* our own broadcast */
    g_ble_thread_status.rx_count++;
    g_ble_thread_status.last_rx_counter = b.counter;
    g_ble_thread_status.last_rx_rloc16 = b.rloc16;
    if (s_rx_blips < 4u)
        s_rx_blips++;
}

static void on_role(core_thread_role_t role, void *ctx)
{
    (void)ctx;
    g_ble_thread_status.role = (uint32_t)role;
    g_ble_thread_status.role_changes++;
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
    b.counter = g_ble_thread_status.tx_count + 1u;
    b.rloc16 = core_thread_rloc16();
    memcpy(b.ext, s_ext, sizeof s_ext);

    int rc = core_thread_udp_broadcast(UDP_PORT, &b, sizeof b);
    if (rc == 0) {
        g_ble_thread_status.tx_count++;
    } else {
        g_ble_thread_status.tx_fail++;
        g_ble_thread_status.last_error = (rc == CORE_THREAD_ERR_FAILED) ? core_thread_last_error() : rc;
    }
}

/* ============================================================
 * Stack high-water mark
 * ============================================================ */

extern uint32_t end;          /* linker scripts: the C heap starts here (1 KB) */
extern uint32_t _estack;      /* top of SRAM */
#define STACK_PAINT   0x5AA5C33CUL
#define HEAP_RESERVE  0x400u

/* Fill the RAM between the heap reserve and (just below) the current stack
 * pointer: everything the stack has not reached yet. */
static void stack_paint(void)
{
    uint32_t sp;
    __asm volatile ("mov %0, sp" : "=r"(sp));
    volatile uint32_t *p = (volatile uint32_t *)((uintptr_t)&end + HEAP_RESERVE);
    while ((uintptr_t)p < sp - 64u)
        *p++ = STACK_PAINT;
}

/* Bytes from the top of SRAM down to the deepest word the stack overwrote. */
static uint32_t stack_peak(void)
{
    const volatile uint32_t *p = (const volatile uint32_t *)((uintptr_t)&end + HEAP_RESERVE);
    const uintptr_t top = (uintptr_t)&_estack;
    while ((uintptr_t)p < top && *p == STACK_PAINT)
        p++;
    return (uint32_t)(top - (uintptr_t)p);
}

/* ============================================================
 * BLE: the Thread status as a read / notify characteristic
 * ============================================================ */

static core_ble_char_t s_status_char;

static void ble_services(void)
{
    core_ble_svc_t svc = core_ble_add_service_id("Thread status", BLE_SVC_ID);
    s_status_char = core_ble_add_char_id(svc, "status", BLE_CHAR_ID, CORE_BLE_READ | CORE_BLE_NOTIFY,
                                         CORE_BLE_BYTES(8), NULL, NULL);
}

static void on_connect(void *ctx)
{
    (void)ctx;
    g_ble_thread_status.ble_connected = 1u;
    g_ble_thread_status.ble_connects++;
}

static void on_disconnect(void *ctx)
{
    (void)ctx;
    g_ble_thread_status.ble_connected = 0u;
}

/* Once a second: refresh the characteristic, notify a subscriber. */
static void ble_publish(core_thread_role_t role)
{
    uint8_t v[8];
    uint16_t rloc = core_thread_rloc16();
    uint32_t rx = g_ble_thread_status.rx_count;
    v[0] = (uint8_t)role;
    v[1] = core_thread_neighbor_count();
    v[2] = (uint8_t)rloc;
    v[3] = (uint8_t)(rloc >> 8);
    v[4] = (uint8_t)rx;
    v[5] = (uint8_t)(rx >> 8);
    v[6] = (uint8_t)(rx >> 16);
    v[7] = (uint8_t)(rx >> 24);
    if (core_ble_set_value(s_status_char, v, sizeof v) == 0)
        g_ble_thread_status.ble_updates++;
    if (core_ble_subscribed(s_status_char) && core_ble_notify(s_status_char) == 0)
        g_ble_thread_status.ble_notifies++;
}

int main(void)
{
    stack_paint();        /* first: before anything else uses the stack */
    core_init();          /* clock + 5 s watchdog, from config.json */
    core_led_init();

    /* Status block: keep the boot count across warm resets. */
    uint32_t boots = (g_ble_thread_status.magic == STATUS_MAGIC) ? g_ble_thread_status.boots + 1u : 1u;
    memset((void *)&g_ble_thread_status, 0, sizeof g_ble_thread_status);
    g_ble_thread_status.boots = boots;
    g_ble_thread_status.rloc16 = 0xFFFEu;
    g_ble_thread_status.parent_rssi = 127;
    if (thread_fatal_info.magic == THREAD_FATAL_MAGIC) {
        g_ble_thread_status.fatal = (thread_fatal_info.reason << 16) | (thread_fatal_info.arg & 0xFFFFu);
        thread_fatal_info.magic = 0;
    }
    g_ble_thread_status.magic = STATUS_MAGIC;

    /* BLE first, then Thread (core_thread_start() needs the radio BLE brought
     * up, as ST's BLE_HeartRate_Thread orders it). */
    core_ble_set_services(ble_services);
    core_ble_on_connect(on_connect, NULL);
    core_ble_on_disconnect(on_disconnect, NULL);
    core_ble_init();
    core_ble_process();
    g_ble_thread_status.ble_adv_result = core_ble_advertise(BLE_NAME);

    core_thread_on_role_change(on_role, NULL);
    int rc = core_thread_start();
    if (rc != 0)
        g_ble_thread_status.last_error = rc;
    rc = core_thread_udp_open(UDP_PORT, on_rx, NULL);
    if (rc != 0 && g_ble_thread_status.last_error == 0)
        g_ble_thread_status.last_error = rc;

    core_thread_ext_addr(s_ext);
    g_ble_thread_status.ext_lo = (uint32_t)s_ext[0] | ((uint32_t)s_ext[1] << 8) |
                             ((uint32_t)s_ext[2] << 16) | ((uint32_t)s_ext[3] << 24);
    g_ble_thread_status.ext_hi = (uint32_t)s_ext[4] | ((uint32_t)s_ext[5] << 8) |
                             ((uint32_t)s_ext[6] << 16) | ((uint32_t)s_ext[7] << 24);

    uint32_t next_tx = core_millis() + 1000u;
    while (1) {
        core_watchdog_feed();
        core_ble_process();              /* both run the one sequencer: each */
        core_thread_process();           /* returns after the next interrupt (<= 1 ms) */

        uint32_t now = core_millis();
        core_thread_role_t role = core_thread_role();
        led_update(now, role);

        if ((int32_t)(now - next_tx) >= 0) {
            next_tx += 1000u;
            if (role >= CORE_THREAD_CHILD)
                send_beacon();
            g_ble_thread_status.role = (uint32_t)role;
            g_ble_thread_status.rloc16 = core_thread_rloc16();
            g_ble_thread_status.partition_id = core_thread_partition_id();
            g_ble_thread_status.neighbors = core_thread_neighbor_count();
            g_ble_thread_status.parent_rssi = core_thread_parent_rssi();
            g_ble_thread_status.settings_saves = thread_settings_save_count();
            g_ble_thread_status.uptime_ms = now;
            ble_publish(role);
            if (now / 1000u % 10u == 0u)
                g_ble_thread_status.stack_peak = stack_peak();
        }
    }
}
