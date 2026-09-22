/**
 * core_ble.h — BLE for Core.ST.W5
 *
 * Simple BLE API: advertise, connect, define GATT services.
 * The BLE stack, sequencer, and radio are managed internally.
 *
 * Usage:
 *   core_ble_set_services(my_services);     // register service builder
 *   core_ble_init();
 *   core_ble_advertise("MY-DEVICE");
 *   while (1) { core_ble_process(); }
 *
 * Requires: Core.ST.W5 (STM32WBA55), BLE_ENABLED=1, clock >= "default" (HSE).
 *
 * @studio category ble label=Core.BLE icon=ᛒ
 *
 * @studio coverage
 *   id:    ble
 *   name:  BLE — Bluetooth Low Energy
 *   page:  /docs/sdk/ble
 *   blurb: Core.ST.W5-only API for BLE peripheral mode: advertise, define
 *          GATT services + characteristics, read/write/notify, and
 *          connect/disconnect callbacks. Currently Tier 1 only — there
 *          are no DSL bindings yet, and the API is C-callback heavy
 *          which doesn't translate cleanly to the current Studio
 *          host-call ABI.
 */

#ifndef CORE_BLE_H
#define CORE_BLE_H

#if !defined(STM32WBA55xx)
#error "core_ble.h: BLE is only available on Core.ST.W5 (STM32WBA55). This tile does not have a BLE radio."
#endif

#include <stdint.h>

/* ============================================================
 * Value types for core_ble_add_char
 * ============================================================ */

#define CORE_BLE_BOOL      1
#define CORE_BLE_UINT8     1
#define CORE_BLE_INT8      1
#define CORE_BLE_UINT16    2
#define CORE_BLE_INT16     2
#define CORE_BLE_UINT32    4
#define CORE_BLE_INT32     4
#define CORE_BLE_BYTES(n)  (n)

/* ============================================================
 * Access modes for core_ble_add_char
 * ============================================================ */

#define CORE_BLE_READ    0x02
#define CORE_BLE_WRITE   0x08
#define CORE_BLE_NOTIFY  0x10
#define CORE_BLE_RW      (CORE_BLE_READ | CORE_BLE_WRITE)

/* ============================================================
 * Handles
 * ============================================================ */

typedef uint16_t core_ble_svc_t;
typedef uint16_t core_ble_char_t;

/* Write callback type (called when central writes to a characteristic) */
typedef void (*core_ble_write_cb)(const uint8_t *data, uint16_t len, void *ctx);

/* ============================================================
 * Lifecycle
 * ============================================================ */

/**
 * Register a service builder function.
 * Called before core_ble_init(). The builder is invoked during init
 * after the GATT server is ready.
 */
void core_ble_set_services(void (*builder)(void));

/**
 * Register an ADDITIONAL service builder, run after the application's own.
 * For SDK modules that bring a GATT service of their own (core_scope's
 * Studio Link) without taking over core_ble_set_services(), which the
 * application — or coregen's generated contract — owns. Call before
 * core_ble_init(). Registering the same builder twice is harmless.
 * Returns 0, or -1 if the slots are full or the stack has already started.
 */
int core_ble_add_services(void (*builder)(void));

/** Initialize the BLE stack. Call once after core_init(). */
void core_ble_init(void);

/** Start advertising. Call after init + at least one core_ble_process(). */
int core_ble_advertise(const char *name);

/** Stop advertising. */
int core_ble_stop_advertise(void);

/** Process BLE events. Call continuously from main loop. */
void core_ble_process(void);

/* ============================================================
 * Service builder — call from your services function
 * ============================================================ */

/**
 * Add a GATT service. Returns a handle for adding characteristics.
 *
 * The UUID is auto-assigned SEQUENTIALLY by registration order
 * (0000B000-…, 0000B100-…, …). Convenient, but the UUIDs shift if you
 * reorder or insert services — so for a contract shared with client apps,
 * prefer core_ble_add_service_id() to pin a stable, order-independent UUID.
 *
 * @param name  Human-readable service name (for documentation/debugging).
 */
core_ble_svc_t core_ble_add_service(const char *name);

/**
 * Add a GATT service with an EXPLICIT 16-bit ID. The full UUID is
 * 0000<id>-8E22-4541-9D4C-21EDAE82ED19. Pinning the ID makes the GATT
 * contract independent of registration order — the recommended path when the
 * service map is a source of truth shared with phone / desktop client apps.
 *
 * @param name  Human-readable service name (for documentation/debugging).
 * @param id    16-bit identifier placed in the shared base UUID.
 */
core_ble_svc_t core_ble_add_service_id(const char *name, uint16_t id);

/**
 * Add a characteristic to a service.
 *
 * @param svc       Service handle from core_ble_add_service.
 * @param name      Human-readable name (for documentation/debugging).
 * @param access    Access mode: CORE_BLE_READ, CORE_BLE_WRITE, CORE_BLE_NOTIFY,
 *                  or combinations (CORE_BLE_RW, CORE_BLE_READ | CORE_BLE_NOTIFY).
 * @param type      Value type: CORE_BLE_BOOL, CORE_BLE_UINT8, CORE_BLE_UINT16,
 *                  CORE_BLE_UINT32, or CORE_BLE_BYTES(n) for raw buffers.
 * @param on_write  Callback when central writes. NULL if read-only.
 * @param ctx       User context passed to on_write callback; may be NULL.
 * @return          Characteristic handle for set_value/notify.
 */
core_ble_char_t core_ble_add_char(core_ble_svc_t svc,
                                   const char *name,
                                   uint8_t access,
                                   uint8_t type,
                                   core_ble_write_cb on_write,
                                   void *ctx);

/**
 * Add a characteristic with an EXPLICIT 16-bit ID (0000<id>-8E22-…), the
 * order-independent counterpart to core_ble_add_char(). Use for the stable
 * contract shared with client apps.
 *
 * @param svc       Service handle.
 * @param name      Human-readable name (for documentation/debugging).
 * @param id        16-bit identifier placed in the shared base UUID.
 * @param access    CORE_BLE_READ / _WRITE / _NOTIFY (or combinations).
 * @param type      CORE_BLE_BOOL / _UINT8/16/32 / CORE_BLE_BYTES(n).
 * @param on_write  Write callback (NULL if not writable).
 * @param ctx       User context for on_write; may be NULL.
 */
core_ble_char_t core_ble_add_char_id(core_ble_svc_t svc,
                                     const char *name,
                                     uint16_t id,
                                     uint8_t access,
                                     uint8_t type,
                                     core_ble_write_cb on_write,
                                     void *ctx);

/**
 * Add a SIG-adopted service by its 16-bit Bluetooth UUID (e.g. 0x180F Battery
 * Service, 0x180A Device Information). Use adopted services for standardised
 * profiles so generic clients and the host OS recognise them; use the _id
 * variants for custom application-specific data.
 *
 * @param name    Human-readable service name (for documentation/debugging).
 * @param uuid16  The assigned 16-bit Bluetooth SIG service UUID.
 */
core_ble_svc_t core_ble_add_service_sig(const char *name, uint16_t uuid16);

/**
 * Add a SIG-adopted characteristic by its 16-bit Bluetooth UUID (e.g. 0x2A19
 * Battery Level, 0x2A29 Manufacturer Name) to a service.
 *
 * @param svc       Service handle from core_ble_add_service_sig.
 * @param name      Human-readable name (for documentation/debugging).
 * @param uuid16    The assigned 16-bit Bluetooth SIG characteristic UUID.
 * @param access    CORE_BLE_READ / _WRITE / _NOTIFY (or combinations).
 * @param type      CORE_BLE_BOOL / _UINT8/16/32 / CORE_BLE_BYTES(n).
 * @param on_write  Write callback (NULL if not writable).
 * @param ctx       User context for on_write; may be NULL.
 */
core_ble_char_t core_ble_add_char_sig(core_ble_svc_t svc,
                                      const char *name,
                                      uint16_t uuid16,
                                      uint8_t access,
                                      uint8_t type,
                                      core_ble_write_cb on_write,
                                      void *ctx);

/* ============================================================
 * Runtime — read/write/notify
 * ============================================================ */

/**
 * Update a characteristic's value. For readable characteristics,
 * this is what the central will read. For notify characteristics,
 * call core_ble_notify() after to push the update.
 */
int core_ble_set_value(core_ble_char_t ch, const void *data, uint16_t len);

/**
 * Send a notification to the connected central.
 * The central must have enabled notifications (CCCD) for this to work.
 * Sends the current value set by core_ble_set_value().
 */
int core_ble_notify(core_ble_char_t ch);

/**
 * @brief  Has a client subscribed to notifications on this characteristic?
 * @param  ch  handle from core_ble_add_char*()
 * @return 1 when connected and the client has enabled notify/indicate, else 0.
 *
 * The CCCD state, which is the only signal there is that anyone is listening.
 * Gate periodic publishing on it: a notify characteristic that free-runs while
 * unsubscribed burns the radio for nobody. Cleared on disconnect, so it never
 * reports a previous client's subscription.
 *
 * Always 0 for a characteristic without CORE_BLE_NOTIFY, which has no CCCD.
 */
int core_ble_subscribed(core_ble_char_t ch);

/* ============================================================
 * Connection
 * ============================================================ */

/** Returns 1 if a central is connected. */
int core_ble_connected(void);

/** Set callback for connection events.
 *  @param cb   Called when a central connects; may be NULL
 *  @param ctx  User context passed to callback; may be NULL
 */
void core_ble_on_connect(void (*cb)(void *ctx), void *ctx);

/** Set callback for disconnection events.
 *  @param cb   Called when a central disconnects; may be NULL
 *  @param ctx  User context passed to callback; may be NULL
 */
void core_ble_on_disconnect(void (*cb)(void *ctx), void *ctx);

/* ============================================================
 * Configuration (call before core_ble_init)
 * ============================================================ */

/** TX power: 0=low(-20dBm), 1=medium(0dBm), 2=high(+10dBm). Default: 1. */
void core_ble_set_tx_power(uint8_t level);

/** Advertising interval in ms (20-10240). Default: 100/150. */
void core_ble_set_adv_interval(uint16_t min_ms, uint16_t max_ms);

/**
 * Request a preferred connection parameter set from the central.
 *
 * The central ultimately owns the connection timing, but a peripheral can ask
 * for parameters that suit its traffic. After each connection the SDK sends an
 * L2CAP update request (retrying until the stack accepts it, since the link is
 * busy with pairing/discovery right after connecting). May also be called while
 * connected to re-request — e.g. tighten the interval while streaming, relax it
 * when idle.
 *
 * For Apple hosts, keep within their guidelines: interval_min >= 15 ms,
 * interval_max >= interval_min + 15 ms, interval_max * (latency + 1) <= 2 s, and
 * timeout in 2000..6000 ms. A short interval lowers latency for high-rate
 * notifications at the cost of power; latency > 0 saves power when the
 * peripheral often has nothing to send.
 *
 * @param min_ms      Minimum connection interval, ms (7.5-4000).
 * @param max_ms      Maximum connection interval, ms (>= min_ms).
 * @param latency     Peripheral latency — connection events it may skip (0-499).
 * @param timeout_ms  Supervision (link-loss) timeout, ms (100-32000).
 */
void core_ble_set_conn_params(uint16_t min_ms, uint16_t max_ms,
                              uint16_t latency, uint16_t timeout_ms);

/**
 * Enable OS-level pairing + bonding (Just Works).
 *
 * When enabled, every characteristic is registered behind an encrypted link,
 * so on first access the host runs its pairing flow (a "Pair?" prompt on most
 * OSes; silent for Just Works on macOS). Keys are persisted in flash NVM, so
 * the bond survives resets — the host doesn't re-pair on the next connection
 * and the device stays listed in OS Bluetooth settings. (Reconnect itself is
 * the central's choice; the peripheral simply re-advertises after a drop.)
 *
 * Call before core_ble_init(). Default: disabled (characteristics open, no
 * pairing). Note that once enabled, *all* clients must pair to read/write.
 */
void core_ble_enable_pairing(void);

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=M title="No Twin coverage for BLE"
//   DSL coverage landed: a project declares a contract, gets palette
//   blocks to publish and receive, and can bind a characteristic to a
//   variable so coregen generates the publisher. What is still missing
//   is the simulator side, so a DSL program that uses BLE cannot be
//   exercised without flashing a board.
//
// @studio unsupported tier=2 value=M title="Connect / disconnect are not DSL events"
//   core_ble_on_connect() and core_ble_on_disconnect() take a callback
//   pointer rather than overriding a weak symbol, so coregen has no
//   seam to emit a dispatcher against. Every other event in the DSL
//   arrives through that weak-symbol pipeline. Until this is wired the
//   only way to notice a connection from the DSL is to poll.
//
// @studio unsupported tier=2 value=L title="Byte-array characteristics have no DSL form"
//   A `bytes` characteristic can be declared and published from C, but
//   the DSL has no type that carries a buffer and a length, so it gets
//   neither a publish block nor a write handler, and it cannot be bound
//   to a variable. Scalars and strings both work.
//
// @studio unsupported tier=1 value=M title="Central / scanner mode"
//   Peripheral-only today. No scanning, no central-role connections,
//   no GATT-client reads/writes. Tracked as its own initiative — most
//   Bergsonne use-cases are peripheral-role (sensor advertising to a
//   phone). Apps that need central-role drop into the WBA BLE stack.
//
// @studio unsupported tier=1 value=L title="Pairing refinements (LE Secure Connections, directed reconnect)"
//   core_ble_enable_pairing() does Just Works *legacy* pairing with
//   bonding: characteristics require an encrypted link and the keys
//   persist in flash NVM, so a bonded host reconnects without re-prompting.
//   Not yet exposed: LE Secure Connections (numeric-comparison / passkey
//   MITM protection) and directed advertising for fast reconnect to a
//   known bonded central.
//
// @studio unsupported tier=1 value=L title="Arbitrary 128-bit UUIDs"
//   Narrower than it used to be. A contract pins its own 16-bit ids,
//   custom (placed in the Bergsonne base UUID) or SIG-adopted, and
//   core_ble_add_service_id() / _sig() take them verbatim, so ids never
//   shift and deployed clients keep working. What is still missing is a
//   fully arbitrary 128-bit UUID, which is what interop with an existing
//   app that expects some other vendor's base requires.
//
// @studio unsupported tier=1 value=L title="Advanced features (LE Audio, extended adv, multi-link)"
//   The WBA radio supports LE Audio (LC3), extended advertising / 2M
//   PHY / coded PHY, and multi-link (multiple simultaneous connections).
//   None of that is exposed.

#endif /* CORE_BLE_H */
