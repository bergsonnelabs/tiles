/* hw-thread-ble-gw: Thread -> BLE -> computer, the bare path (bench check
 * for BLE + Thread in one image; first run 2026-10-12, no loss over 18 s).
 *
 * Senders: any Core.ST.W5 running tests/hw-thread. Computer: run
 * tools/thread_gw_listen.py in a terminal with Bluetooth permission.
 *
 * Joins the default tiles-dev Thread network, listens for hw-thread beacons
 * (UDP port 1234, 20 bytes: "TH", ver, 0, counter u32, rloc16 u16, spare,
 * ext[8]) and forwards each as a 16-byte BLE notification:
 *   0 gw rx count u32 | 4 sender counter u32 | 8 sender rloc16 u16 |
 *   10 gw role u8 | 11 gw neighbors u8 | 12 gw uptime s u32   (all LE)
 * Service "thread_gw", characteristic "beacon" (read + notify). */
#include "core.h"
#include "core_led.h"
#include "core_watchdog.h"
#include "core_ble.h"
#include "core_thread.h"
#include <string.h>

typedef struct __attribute__((packed)) {
    uint8_t tag[2]; uint8_t version; uint8_t reserved;
    uint32_t counter; uint16_t rloc16; uint16_t spare; uint8_t ext[8];
} beacon_t;

typedef struct __attribute__((packed)) {
    uint32_t gw_rx; uint32_t counter; uint16_t rloc16;
    uint8_t role; uint8_t neighbors; uint32_t uptime_s;
} notif_t;

static core_ble_char_t s_ch;
volatile uint32_t g_gwb_rx, g_gwb_notified, g_gwb_loops;
volatile int      g_gwb_role;
static volatile uint8_t s_pending;
static notif_t s_last;

static void services(void)
{
    core_ble_svc_t svc = core_ble_add_service("thread_gw");
    s_ch = core_ble_add_char(svc, "beacon", CORE_BLE_READ | CORE_BLE_NOTIFY,
                             CORE_BLE_BYTES(sizeof(notif_t)), NULL, NULL);
}

static void on_beacon(const core_thread_udp_packet_t *pkt, void *ctx)
{
    const uint8_t *data = pkt->data;
    uint16_t len = pkt->len;
    (void)ctx;
    if (len < sizeof(beacon_t)) return;
    beacon_t b; memcpy(&b, data, sizeof b);
    if (b.tag[0] != 'T' || b.tag[1] != 'H') return;
    g_gwb_rx++;
    s_last.gw_rx = g_gwb_rx;
    s_last.counter = b.counter;
    s_last.rloc16 = b.rloc16;
    s_pending = 1;                       /* notify from the loop, not the callback */
}

int main(void)
{
    core_init();
    core_led_init();
    core_led_heartbeat(1000, 20);
    core_ble_set_services(services);
    core_ble_init();                     /* also starts Thread (combined build) */
    core_ble_advertise("W5 Thread GW");
    core_thread_udp_open(1234, on_beacon, NULL);

    while (1) {
        core_watchdog_feed();
        core_ble_process();
        g_gwb_role = (int)core_thread_role();
        if (s_pending) {
            s_pending = 0;
            s_last.role = (uint8_t)g_gwb_role;
            s_last.neighbors = (uint8_t)core_thread_neighbor_count();
            s_last.uptime_s = core_millis() / 1000u;
            core_ble_set_value(s_ch, &s_last, sizeof s_last);
            g_gwb_notified++;
        }
        g_gwb_loops++;
        core_delay_ms(5);
    }
}
