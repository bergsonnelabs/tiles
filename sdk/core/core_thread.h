/**
 * core_thread.h — Thread (OpenThread) networking for Core.ST.W5
 *
 * A minimal API over OpenThread: join a fixed network, see your role, and
 * exchange UDP datagrams with the other nodes. The device is a full Thread
 * device (FTD): it attaches as a child and becomes a router (or the leader)
 * when the network needs one. OpenThread, the 802.15.4 radio and the
 * sequencer are managed internally.
 *
 * Joins on boot: with config.json `"thread": {"enabled": true}` the node
 * joins (or forms) the network without any code. core_init() calls
 * core_thread_start() (core_ble_init() does, with BLE in the image too), and
 * core_delay_ms() and core_watchdog_feed() run core_thread_process() while
 * they wait or feed, so even a program that only blinks stays on the network.
 *
 * Usage (all of it optional except what you use):
 *   core_init();                                    // starts Thread
 *   core_thread_udp_open(1234, on_rx, NULL);
 *   while (1) {
 *       core_thread_process();                      // or core_delay_ms() / core_watchdog_feed()
 *       if (core_thread_role() >= CORE_THREAD_CHILD)
 *           core_thread_udp_broadcast(1234, "hi", 2);
 *   }
 *
 * Requires: Core.ST.W5 (STM32WBA55), config.json `"thread": {"enabled": true}`
 * (or `make THREAD_ENABLED=1`, which links Thread but leaves starting it to
 * the application), clock "medium" or higher.
 *
 * With BLE: config.json may enable "ble" as well, and one image then runs
 * both (ST's concurrent link layer time-shares the radio). core_ble_init()
 * starts Thread right after BLE (core_thread_start() returns
 * CORE_THREAD_ERR_STATE until BLE is up). Call core_ble_process() from the
 * main loop: it also does BLE's own housekeeping (re-advertising, updates),
 * so it must not be left out. The two stacks share one scheduler, so neither
 * starves the other, and BLE callbacks may call this API.
 * OpenThread is a trimmed build in that case (see the notes at the end).
 *
 * Network: DEVELOPMENT ONLY. The network credentials come from config.json
 * ("thread": channel, panid, network_key, network_name, ext_panid,
 * mesh_local_prefix; coregen writes them into coregen/thread_dataset.h) and
 * every node built from the same config joins the same network: no
 * commissioning. Anyone with the config can join and read the traffic. Real
 * commissioning (joiner / commissioner) is deferred.
 *
 * Provisioning: with config.json `"thread": {"provisioning": true}` the
 * network can also be set at run time (core_thread_set_dataset()). A network
 * set that way is stored with OpenThread's settings and wins over config.json
 * at every boot, across reboots and BLE firmware updates; config.json's
 * network is the factory default, used while nothing is stored, and
 * core_thread_factory_reset() returns to it. `"ble_provisioning": true` (BLE +
 * Thread builds) adds a GATT service that lets a phone do the same:
 * docs/thread-provisioning-protocol.md. DEVELOPMENT ONLY: that service has no
 * pairing, so ANY phone in range that connects can read the status and move
 * the node to another network (it cannot read the network key back).
 * core_thread_prov_allow_write() is the hook to gate it.
 *
 * Deferred (not in this API yet): commissioning, MTD / sleepy end devices,
 * CoAP, SRP / DNS, border routing, TCP, Studio DSL blocks. The full OpenThread C API (sdk/thread/include/openthread) is
 * available to C code through core_thread_instance().
 *
 * @studio category thread label=Core.Thread icon=⌬
 *
 * @studio coverage
 *   id:    thread
 *   name:  Thread — 802.15.4 mesh networking (OpenThread)
 *   page:  /docs/sdk/thread
 *   blurb: Core.ST.W5-only API for a Thread mesh: join a fixed development
 *          network from config.json as a full Thread device, read the role
 *          and link diagnostics, and send / receive UDP (unicast and
 *          realm-local multicast). OpenThread 1.3 from STM32CubeWBA 1.6.1.
 *          Tier 1 (C) only: no DSL bindings yet.
 */

#ifndef CORE_THREAD_H
#define CORE_THREAD_H

#if !defined(STM32WBA55xx)
#error "core_thread.h: Thread is only available on Core.ST.W5 (STM32WBA55). This tile has no 802.15.4 radio."
#endif

#include <stdint.h>

/* ============================================================
 * Types
 * ============================================================ */

/** Device role (the values of OpenThread's otDeviceRole). */
typedef enum {
    CORE_THREAD_DISABLED = 0,   /**< Thread is not running */
    CORE_THREAD_DETACHED = 1,   /**< Running, not attached to a network (searching) */
    CORE_THREAD_CHILD    = 2,   /**< Attached as a child of a router */
    CORE_THREAD_ROUTER   = 3,   /**< A router */
    CORE_THREAD_LEADER   = 4,   /**< The leader (a router that also manages the partition) */
} core_thread_role_t;

/* Error codes: functions return 0 on success or one of these (negative). */
#define CORE_THREAD_ERR_STATE   (-1)   /**< Not started, or not attached */
#define CORE_THREAD_ERR_ARG     (-2)   /**< Bad argument */
#define CORE_THREAD_ERR_NOBUFS  (-3)   /**< Out of message buffers */
#define CORE_THREAD_ERR_FULL    (-4)   /**< No free UDP socket */
#define CORE_THREAD_ERR_FAILED  (-5)   /**< OpenThread refused; see core_thread_last_error() */
#define CORE_THREAD_ERR_UNSUPPORTED (-6) /**< Not in this build: needs config.json "thread": {"provisioning": true} */

#define CORE_THREAD_UDP_SOCKETS      4     /**< Sockets core_thread_udp_open() can hold */
#define CORE_THREAD_UDP_MAX_PAYLOAD  256   /**< Received datagrams are cut to this many bytes */

/** Role-change callback: the new role. Runs from core_thread_process(). */
typedef void (*core_thread_role_cb)(core_thread_role_t role, void *ctx);

/**
 * UDP receive callback. Runs from core_thread_process().
 * src_addr is the sender's IPv6 address (16 bytes, network order); reply with
 * core_thread_udp_send(src_addr, src_port, ...).
 */
typedef void (*core_thread_udp_rx_cb)(const uint8_t *data, uint16_t len,
                                      const uint8_t src_addr[16], uint16_t src_port, void *ctx);

/**
 * A Thread network: the operational dataset fields config.json sets.
 * Byte strings are in the order they are written down (an ext PAN ID of
 * dead00beef00cafe is { 0xde, 0xad, 0x00, 0xbe, 0xef, 0x00, 0xca, 0xfe }).
 */
typedef struct {
    uint8_t  channel;               /**< 802.15.4 channel, 11-26 */
    uint16_t panid;                 /**< PAN ID, 0x0000-0xFFFE */
    uint8_t  ext_panid[8];          /**< Extended PAN ID */
    uint8_t  mesh_local_prefix[8];  /**< Mesh-local prefix: the first 8 bytes of the /64 (fdde:ad00:beef:0:: is fd de ad 00 be ef 00 00) */
    uint8_t  network_key[16];       /**< Network key (never returned by core_thread_get_network()) */
    char     network_name[17];      /**< Network name, 1-16 bytes, NUL-terminated */
    uint32_t active_timestamp;      /**< Active Timestamp, seconds; 0 means 1, the value config.json networks use */
} core_thread_dataset_t;

/* ============================================================
 * Lifecycle
 * ============================================================ */

/**
 * @brief  Start Thread: bring up the radio and OpenThread, then join (or form) the network from config.json.
 * @return 0 on success (also when Thread is already running), or a negative CORE_THREAD_ERR_* code.
 *
 * core_init() calls it for you when config.json enables "thread" (core_ble_init()
 * does, in a build with BLE too); calling it again is harmless and returns 0.
 * Returns at once; attaching takes a few seconds (watch core_thread_role()).
 * Settings OpenThread keeps across resets (frame counters, role) live in two
 * reserved flash pages. In a build with BLE too, it returns
 * CORE_THREAD_ERR_STATE before core_ble_init().
 */
int core_thread_start(void);

/**
 * @brief  Run Thread: radio events, OpenThread timers and tasks, received datagrams.
 *
 * Optional: core_delay_ms() and core_watchdog_feed() call it for you. Call it
 * from the main loop for the lowest latency. It waits (WFI) for the next
 * interrupt when there is nothing to do, so a loop around it sleeps between
 * events and wakes at least every millisecond (SysTick). It does nothing
 * before core_thread_start(), inside a Thread or BLE callback, or in an
 * interrupt handler.
 */
void core_thread_process(void);

/**
 * @brief  The device's current Thread role.
 * @return CORE_THREAD_DISABLED before core_thread_start(), else the role.
 */
core_thread_role_t core_thread_role(void);

/**
 * @brief  Register a callback for role changes (detached, child, router, leader).
 * @param  cb   Called with the new role from core_thread_process(); NULL to remove.
 * @param  ctx  Passed back to cb; may be NULL.
 */
void core_thread_on_role_change(core_thread_role_cb cb, void *ctx);

/* ============================================================
 * UDP
 * ============================================================ */

/**
 * @brief  Open a UDP socket bound to a port and receive datagrams sent to it.
 * @param  port   Local UDP port (1-65535).
 * @param  rx_cb  Called for each datagram received on the port; may be NULL to only send.
 * @param  ctx    Passed back to rx_cb; may be NULL.
 * @return 0 on success, CORE_THREAD_ERR_FULL when all sockets are taken, or another negative code.
 *
 * The first socket opened is also the source of core_thread_udp_send() and
 * core_thread_udp_broadcast(). Call after core_thread_start().
 */
int core_thread_udp_open(uint16_t port, core_thread_udp_rx_cb rx_cb, void *ctx);

/**
 * @brief  Send a UDP datagram to an IPv6 address on the Thread network.
 * @param  addr6  Destination IPv6 address, 16 bytes in network order.
 * @param  port   Destination UDP port.
 * @param  buf    Payload.
 * @param  len    Payload length in bytes (up to ~1200).
 * @return 0 when queued, CORE_THREAD_ERR_STATE when not attached, or another negative code.
 *
 * Sent from the first opened socket's port (an ephemeral port if none is open).
 */
int core_thread_udp_send(const uint8_t addr6[16], uint16_t port, const void *buf, uint16_t len);

/**
 * @brief  Send a UDP datagram to every node on the Thread network (ff03::1, realm-local all-nodes).
 * @param  port  Destination UDP port.
 * @param  buf   Payload.
 * @param  len   Payload length in bytes.
 * @return 0 when queued, CORE_THREAD_ERR_STATE when not attached, or another negative code.
 */
int core_thread_udp_broadcast(uint16_t port, const void *buf, uint16_t len);

/* ============================================================
 * Diagnostics
 * ============================================================ */

/**
 * @brief  The device's RLOC16 (its short address in the mesh; changes with the role).
 * @return The RLOC16, or 0xFFFE when not attached.
 */
uint16_t core_thread_rloc16(void);

/**
 * @brief  Copy the device's IEEE 802.15.4 extended address (8 bytes).
 * @param  out  Receives 8 bytes; zeros before core_thread_start().
 */
void core_thread_ext_addr(uint8_t out[8]);

/**
 * @brief  The ID of the Thread partition the device belongs to.
 * @return The partition ID, or 0 when not attached.
 */
uint32_t core_thread_partition_id(void);

/**
 * @brief  Number of neighbours in the device's neighbour table (routers and children it hears).
 * @return The neighbour count.
 */
uint8_t core_thread_neighbor_count(void);

/**
 * @brief  Average RSSI of frames from the parent, when the device is a child.
 * @return RSSI in dBm, or 127 when there is no parent (not a child).
 */
int8_t core_thread_parent_rssi(void);

/**
 * @brief  The last error OpenThread reported to this API (an otError value; 0 = none).
 * @return The otError code (see openthread/error.h).
 */
int core_thread_last_error(void);

/**
 * @brief  The OpenThread instance, for C code that calls the OpenThread API directly.
 * @return An otInstance pointer (as void*), or NULL before core_thread_start().
 */
void *core_thread_instance(void);

/* ============================================================
 * Network: run-time provisioning
 * ============================================================ */

/**
 * @brief  Read the network the device is set to (its active dataset), without the network key.
 * @param  out  Receives channel, PAN ID, ext PAN ID, mesh-local prefix, name and timestamp; network_key is all zeros.
 * @return 0 on success, CORE_THREAD_ERR_STATE before core_thread_start(), or another negative code.
 */
int core_thread_get_network(core_thread_dataset_t *out);

/**
 * @brief  Stop or restart Thread: leave the network (detach, radio idle for Thread) or attach again.
 * @param  on  0 to stop (the role becomes CORE_THREAD_DISABLED), 1 to start again.
 * @return 0 on success, CORE_THREAD_ERR_STATE before core_thread_start(), or another negative code.
 *
 * The network settings are kept; core_thread_start() at the next boot attaches as usual.
 */
int core_thread_set_enabled(int on);

/**
 * @brief  Move the device to another Thread network and store it, so it wins over config.json from now on.
 * @param  ds  The new network: channel 11-26, PAN ID 0x0000-0xFFFE, a 1-16 byte name, key, ext PAN ID, mesh-local prefix.
 * @return 0 on success, CORE_THREAD_ERR_ARG for a bad dataset, CORE_THREAD_ERR_STATE before core_thread_start(),
 *         CORE_THREAD_ERR_UNSUPPORTED without "provisioning" in config.json, or another negative code.
 *
 * Stops Thread, makes the dataset active, marks it provisioned, and starts
 * Thread again, which attaches to the new network within a few seconds
 * (watch core_thread_role()). Both are written to the settings flash pages
 * 250 ms later, in one snapshot. If OpenThread refuses a step, the previous
 * network is restored and attached again. Setting the network the device is
 * already on writes nothing (beyond the provisioned mark) and does not detach.
 * Needs config.json `"thread": {"provisioning": true}` (or "ble_provisioning").
 */
int core_thread_set_dataset(const core_thread_dataset_t *ds);

/**
 * @brief  Forget the stored (provisioned) network and return to config.json's, then attach again.
 * @return 0 on success, CORE_THREAD_ERR_STATE before core_thread_start(),
 *         CORE_THREAD_ERR_UNSUPPORTED without "provisioning" in config.json, or another negative code.
 *
 * Restarts Thread on config.json's network (also after core_thread_set_enabled(0)).
 */
int core_thread_factory_reset(void);

/**
 * @brief  Is the current network a stored (provisioned) one rather than config.json's factory default?
 * @return 1 when a provisioned network is in use, 0 when it is config.json's (always 0 before
 *         core_thread_start() and in builds without "provisioning").
 */
int core_thread_is_provisioned(void);

/**
 * @brief  Hook: may a BLE client change the network now? Override to gate the provisioning service.
 * @return Nonzero to accept writes to the BLE provisioning service, 0 to refuse them
 *         (THREAD_PROV_ERR_DENIED). The SDK's default (a weak function) returns 1: open writes.
 *
 * DEVELOPMENT ONLY default: any connected phone may set the network. Define
 * this function in the application to require, say, a button press or a
 * bonded link. Called from core_thread_process(), never from an interrupt.
 * BLE + Thread builds with "ble_provisioning" only.
 */
int core_thread_prov_allow_write(void);

// @studio unsupported tier=1 value=H title="BLE provisioning is open (development only)"
//   With "ble_provisioning" any phone that connects can move the node to
//   another Thread network: there is no pairing or authorization yet. The
//   network key cannot be read back. core_thread_prov_allow_write() is the
//   hook for a gate; pairing-protected writes are planned.

// @studio unsupported tier=2 value=M title="No DSL blocks for Thread"
//   Thread is C-only (Tier 1) for now, like BLE: no palette blocks, no twin.
//   The role and UDP callbacks need the host-call ABI to grow first.

// @studio unsupported tier=1 value=H title="No commissioning (fixed development network)"
//   Every node built from the same config.json joins the same network with the
//   same network key, unless it is moved at run time (core_thread_set_dataset(),
//   or the open BLE provisioning service); there is no joiner / commissioner
//   yet. Development only.

// @studio unsupported tier=1 value=L title="BLE + Thread builds use a trimmed OpenThread"
//   With BLE in the same image, OpenThread is rebuilt without the commissioner,
//   joiner, CoAP secure (DTLS), SRP and DNS clients, TCP, ping, mesh and
//   link-metrics diagnostics, with 64 message buffers instead of 128, so both
//   stacks fit in RAM with room for an application. A full router (FTD) all
//   the same; Thread-only builds keep ST's complete library.

// @studio unsupported tier=1 value=M title="FTD only: no sleepy end devices (MTD / SED)"
//   The device is always a full Thread device with its receiver on; battery
//   end devices that sleep between polls are deferred.

// @studio unsupported tier=1 value=L title="No ECC (PKA): ECJPAKE, ECDSA, DTLS unavailable"
//   The prebuilt mbedTLS calls the ST HAL for the PKA; the SDK answers those
//   calls with an error, so commissioning, SRP client keys and CoAP secure
//   fail. AES (MAC and MLE security) runs on the AES peripheral.

// @studio unsupported tier=1 value=L title="No CoAP / SRP / DNS / border routing wrappers"
//   Present in the OpenThread library and callable through
//   core_thread_instance() and the OpenThread headers, but not wrapped.

#endif /* CORE_THREAD_H */
