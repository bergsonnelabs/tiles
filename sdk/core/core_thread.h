/**
 * core_thread.h — Thread (OpenThread) networking for Core.ST.W5
 *
 * A minimal API over OpenThread: join a fixed network, see your role, and
 * exchange UDP datagrams with the other nodes. The device is a full Thread
 * device (FTD): it attaches as a child and becomes a router (or the leader)
 * when the network needs one. OpenThread, the 802.15.4 radio and the
 * sequencer are managed internally.
 *
 * Usage:
 *   core_init();
 *   core_thread_start();                            // join / form the network
 *   core_thread_udp_open(1234, on_rx, NULL);
 *   while (1) {
 *       core_thread_process();
 *       if (core_thread_role() >= CORE_THREAD_CHILD)
 *           core_thread_udp_broadcast(1234, "hi", 2);
 *   }
 *
 * Requires: Core.ST.W5 (STM32WBA55), config.json `"thread": {"enabled": true}`
 * (or `make THREAD_ENABLED=1`), clock "medium" or higher. Not together with
 * BLE yet (BLE + Thread comes in a later milestone).
 *
 * Network: DEVELOPMENT ONLY. The network credentials come from config.json
 * ("thread": channel, panid, network_key, network_name, ext_panid,
 * mesh_local_prefix; coregen writes them into coregen/thread_dataset.h) and
 * every node built from the same config joins the same network: no
 * commissioning. Anyone with the config can join and read the traffic. Real
 * commissioning (joiner / commissioner) is deferred.
 *
 * Deferred (not in this API yet): commissioning, MTD / sleepy end devices,
 * CoAP, SRP / DNS, border routing, TCP, BLE + Thread together, Studio DSL
 * blocks. The full OpenThread C API (sdk/thread/include/openthread) is
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

/* ============================================================
 * Lifecycle
 * ============================================================ */

/**
 * @brief  Start Thread: bring up the radio and OpenThread, then join (or form) the network from config.json.
 * @return 0 on success, or a negative CORE_THREAD_ERR_* code.
 *
 * Call once after core_init(). Returns at once; attaching takes a few seconds
 * (watch core_thread_role()). Settings OpenThread keeps across resets (frame
 * counters, role) live in two reserved flash pages.
 */
int core_thread_start(void);

/**
 * @brief  Run Thread: radio events, OpenThread timers and tasks, received datagrams.
 *
 * Call continuously from the main loop. It waits (WFI) for the next interrupt
 * when there is nothing to do, so a loop around it sleeps between events and
 * wakes at least every millisecond (SysTick).
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

// @studio unsupported tier=2 value=M title="No DSL blocks for Thread"
//   Thread is C-only (Tier 1) for now, like BLE: no palette blocks, no twin.
//   The role and UDP callbacks need the host-call ABI to grow first.

// @studio unsupported tier=1 value=H title="No commissioning (fixed development network)"
//   Every node built from the same config.json joins the same network with the
//   same network key; there is no joiner / commissioner yet. Development only.

// @studio unsupported tier=1 value=M title="BLE and Thread not in the same build"
//   A build enables one radio protocol: BLE + Thread (ST's concurrent link
//   layer) comes in a later milestone. The build stops with an error if both
//   are enabled.

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
