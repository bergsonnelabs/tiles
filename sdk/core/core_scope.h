/**
 * core_scope.h — Scope: stream live variables to a host for plotting
 *
 * Register variables once, then call one function where the data is fresh.
 * The module snapshots the variables, frames them, and streams them over
 * whichever link the Core has (USB CDC, or Bluetooth LE on a radio Core).
 * Studio's Scope panel plots the stream; so does anything else that speaks
 * the protocol in docs/scope-protocol.md — the stream describes its own
 * channels, so a host needs no project, no ELF and no configuration.
 *
 * Typical usage:
 *
 *   static int32_t distance_mm;
 *   static float   temp_c;
 *   static int16_t accel[3];
 *
 *   core_scope_watch(distance_mm);          // type and name come from the variable
 *   core_scope_watch(temp_c);
 *   core_scope_watch_array(accel, 3);       // channels accel[0], accel[1], accel[2]
 *
 *   while (1) {
 *       ... update the variables ...
 *       core_scope_update();                // snapshot + send; never blocks
 *   }
 *
 * Sampling from an interrupt (jitter-free, the plot follows the data's own
 * clock instead of the main loop's):
 *
 *   core_scope_declare_rate_hz(1000);       // promise: sample() runs at exactly 1 kHz
 *   void my_timer_isr(void) { ...; core_scope_sample(); }   // ISR-safe: copies, nothing else
 *   while (1) { ...; core_scope_pump(); }                   // frames + sends from the main loop
 *
 * Cost. With no host listening (no DTR on USB, no subscriber on BLE) every
 * call is one load and one branch. While streaming, sample() is a gather of
 * the registered variables into a ring; pump() does the framing and the
 * link I/O. Nothing here blocks: when the host or the radio cannot keep up,
 * whole samples are dropped and the host is told how many.
 *
 * Production builds. Define CORE_SCOPE_ENABLED=0 (config.json
 * `"scope": {"enabled": false}`) and every call below compiles to nothing:
 * no flash, no RAM, and no Studio Link service in the GATT table.
 *
 * BLE ordering. On a radio Core the module adds its own GATT service, which
 * has to happen before the stack starts: make the first core_scope_* call
 * (or core_scope_init()) BEFORE core_ble_init().
 *
 * @studio category scope label=Core.Scope icon=📈
 *
 * @studio coverage
 *   id:    scope
 *   name:  Scope — live variable streaming
 *   blurb: Self-describing binary stream of registered variables over USB
 *          CDC or BLE notifications, shared with print text on the same
 *          port. ISR-safe sampling, non-blocking sends, drop-and-count
 *          flow control, compiles out for production. The DSL surface is
 *          the `scope(...)` statement.
 */
#ifndef CORE_SCOPE_H
#define CORE_SCOPE_H

#include <stdbool.h>
#include <stdint.h>

/** Compile-time master switch. 0 turns every call into nothing. */
#ifndef CORE_SCOPE_ENABLED
#define CORE_SCOPE_ENABLED 1
#endif

/* Buffer sizes. The defaults cost about 1.2 KB of RAM once the module is
 * used (nothing if it is not). The Core.ST.L0 has 2 KB in total and no link
 * of its own, so there the defaults shrink to what a UART link could use. */
#if defined(STM32L011xx)
#define CORE_SCOPE_DEFAULT_CHANNELS_ 8
#define CORE_SCOPE_DEFAULT_RING_     128
#define CORE_SCOPE_DEFAULT_TX_       128
#elif defined(BLE_ENABLED) && BLE_ENABLED
/* A radio delivers in bursts: nothing moves between connection events (15-50
 * ms apart), and a missed event or a retransmission doubles the wait. The
 * ring is what rides that out, so it is sized for ~250 ms of a 6-channel
 * int16 stream at 1 kHz rather than USB's few milliseconds. */
#define CORE_SCOPE_DEFAULT_CHANNELS_ 32
#define CORE_SCOPE_DEFAULT_RING_     4096
#define CORE_SCOPE_DEFAULT_TX_       320
#else
#define CORE_SCOPE_DEFAULT_CHANNELS_ 32
#define CORE_SCOPE_DEFAULT_RING_     512
#define CORE_SCOPE_DEFAULT_TX_       320
#endif

/** Most channels a program can register (an array counts per element).
 * The wire format's ceiling is 32. 12 bytes of RAM each. */
#ifndef CORE_SCOPE_MAX_CHANNELS
#define CORE_SCOPE_MAX_CHANNELS CORE_SCOPE_DEFAULT_CHANNELS_
#endif

/** Sample ring, bytes. Holds snapshots between sample() and pump(); one
 * snapshot costs 4 bytes plus the sum of the channel widths. */
#ifndef CORE_SCOPE_RING_BYTES
#define CORE_SCOPE_RING_BYTES CORE_SCOPE_DEFAULT_RING_
#endif

/** Frame staging buffer, bytes. Must hold the schema frame (6 + 2 + the
 * channel descriptors); a channel that would not fit is refused. */
#ifndef CORE_SCOPE_TX_BYTES
#define CORE_SCOPE_TX_BYTES CORE_SCOPE_DEFAULT_TX_
#endif

/** Channel value types — the wire type codes of docs/scope-protocol.md. */
typedef enum {
    CORE_SCOPE_I32  = 0, /**< int32_t, 4 bytes on the wire */
    CORE_SCOPE_F32  = 1, /**< float, 4 bytes */
    CORE_SCOPE_BOOL = 2, /**< bool in memory, travels as int32 0/1 */
    CORE_SCOPE_U32  = 3, /**< uint32_t, 4 bytes */
    CORE_SCOPE_I16  = 4, /**< int16_t, 2 bytes */
    CORE_SCOPE_U16  = 5, /**< uint16_t, 2 bytes */
    CORE_SCOPE_I8   = 6, /**< int8_t, 1 byte */
    CORE_SCOPE_U8   = 7, /**< uint8_t, 1 byte */
} core_scope_type_t;

/**
 * A link carries the framed byte stream to the host. USB and BLE ship with
 * the SDK; anything with a write function (a UART, a radio module) can be a
 * link too — see core_scope_set_link().
 */
typedef struct {
    /** Optional one-time setup (BLE registers its GATT service here). */
    void (*init)(void);
    /** Non-zero while a host is listening. Streaming stops while it is 0. */
    int (*ready)(void);
    /** Offer `len` bytes. Returns how many were taken (0 = busy, try later).
     * MUST NOT block. */
    int (*write)(const uint8_t *buf, uint16_t len);
    /** Largest single write, bytes (a USB packet, a BLE notification). */
    uint16_t packet_bytes;
    /** Hold a partly filled packet this long to batch frames into it.
     * 0 = send at once (lowest latency; right for USB). */
    uint16_t flush_ms;
} core_scope_link_t;

#if CORE_SCOPE_ENABLED

/** USB CDC link. Shares the port with print text. USB-capable Cores only. */
extern const core_scope_link_t core_scope_link_usb;
/** Bluetooth LE link: notifications on the Studio Link service
 * (0x5C00 / 0x5C01). Radio Cores built with BLE enabled only. */
extern const core_scope_link_t core_scope_link_ble;

/**
 * @brief  Start the module on the Core's default link.
 *
 * Optional — the first core_scope_watch()/add() does it. Call it explicitly
 * on a radio Core to get the GATT service registered before core_ble_init().
 * The default link is USB where the Core has it, else BLE when the build has
 * the radio on, else none (calls do nothing until core_scope_set_link()).
 */
void core_scope_init(void);

/**
 * @brief  Stream over a different link than the default.
 * @param  link  Link to use; must outlive the program. NULL = no link.
 */
void core_scope_set_link(const core_scope_link_t *link);

/**
 * @brief  Register one variable as a channel.
 *
 * Prefer core_scope_watch(), which fills in the name and type. Register
 * everything before sampling starts: a later add restarts the stream with a
 * new channel list.
 *
 * @param  name  Channel name shown on the plot. Not copied — must outlive
 *               the program (a string literal).
 * @param  ptr   The variable. Read on every sample, so it must stay valid
 *               (static or global, not a local).
 * @param  type  The variable's type.
 * @return 0 on success, -1 if the table or the schema frame is full.
 */
int core_scope_add(const char *name, const volatile void *ptr, core_scope_type_t type);

/**
 * @brief  Register a fixed array as channels `name[0]` .. `name[count-1]`.
 * @param  name   Base name. Not copied.
 * @param  ptr    First element.
 * @param  type   Element type.
 * @param  count  Number of elements.
 * @return 0 on success, -1 if they do not all fit (none are added).
 */
int core_scope_add_array(const char *name, const volatile void *ptr, core_scope_type_t type,
                         uint8_t count);

/**
 * @brief  Limit core_scope_update() to one sample per `ms` milliseconds.
 *
 * A gate, not a timer: a loop that runs slower than this streams at its own
 * pace. 0 (the default) samples on every call.
 *
 * @param  ms  [0..60000] Minimum time between samples.
 */
void core_scope_set_interval_ms(uint32_t ms);

/**
 * @brief  Promise that samples are taken at exactly this rate.
 *
 * For sampling driven by a timer or a sensor's data-ready interrupt. The
 * stream then carries a sample counter instead of a per-sample millisecond
 * timestamp: smaller, exact above 1 kHz, and a dropped sample shows up as a
 * gap of exactly the right width. 0 returns to timestamped samples.
 *
 * @param  hz  [0..4000000] Samples per second.
 */
void core_scope_declare_rate_hz(uint32_t hz);

/**
 * @brief  As core_scope_declare_rate_hz(), in millihertz — for rates that
 *         are not a whole number of hertz (an IMU at 416.667 Hz = 416667).
 * @param  millihertz  Samples per 1000 seconds.
 */
void core_scope_declare_rate_millihertz(uint32_t millihertz);

/**
 * @brief  Snapshot the registered variables. ISR-safe.
 *
 * Copies the current values into the sample ring and returns; the cost is
 * the copy. When the ring is full the sample is dropped and counted. Call
 * from ONE context only (one ISR, or the main loop) — and not alongside
 * core_scope_update(), which samples too.
 */
void core_scope_sample(void);

/**
 * @brief  Frame the pending samples and hand them to the link. Main loop.
 *
 * Never blocks: what the link will not take now is offered again on the
 * next call. Also announces the channel list (once a second, so a host that
 * connects mid-run catches up) and notices the host coming and going.
 */
void core_scope_pump(void);

/**
 * @brief  core_scope_sample() + core_scope_pump(), honoring the interval
 *         gate. The one call a simple main loop needs.
 */
void core_scope_update(void);

/**
 * @brief  Pause or resume at run time (a debug button, a BLE command).
 * @param  on  0 = paused: calls cost one branch. Non-zero = running.
 */
void core_scope_enable(int on);

/**
 * @brief  Is a host listening right now?
 * @return Non-zero while samples are being streamed.
 */
int core_scope_active(void);

/**
 * @brief  Samples dropped since boot because the ring was full.
 * @return Running count; wraps at 2^32.
 */
uint32_t core_scope_dropped(void);

/* `long` is int32 on the Cores (ILP32) and absent from the list elsewhere,
 * so watching a 64-bit long is a compile error rather than a wrong plot. */
#if defined(__SIZEOF_LONG__) && __SIZEOF_LONG__ == 4
#define CORE_SCOPE_LONG_TYPES_ long : CORE_SCOPE_I32, unsigned long : CORE_SCOPE_U32,
#else
#define CORE_SCOPE_LONG_TYPES_
#endif

/** The wire type of a variable. No `double` / 64-bit entry on purpose:
 * watching one fails to compile. */
#define CORE_SCOPE_TYPE_OF(var)               \
    _Generic((var),                           \
        CORE_SCOPE_LONG_TYPES_                \
        int : CORE_SCOPE_I32,                 \
        unsigned int : CORE_SCOPE_U32,        \
        short : CORE_SCOPE_I16,               \
        unsigned short : CORE_SCOPE_U16,      \
        signed char : CORE_SCOPE_I8,          \
        unsigned char : CORE_SCOPE_U8,        \
        char : CORE_SCOPE_U8,                 \
        float : CORE_SCOPE_F32,               \
        _Bool : CORE_SCOPE_BOOL)

/** Register `var` as a channel named after itself. */
#define core_scope_watch(var) core_scope_add(#var, &(var), CORE_SCOPE_TYPE_OF(var))

/** Register `var` under a display name of your choosing. */
#define core_scope_watch_as(name, var) core_scope_add((name), &(var), CORE_SCOPE_TYPE_OF(var))

/** Register the first `count` elements of array `arr` as `arr[0]`, `arr[1]`, … */
#define core_scope_watch_array(arr, count) \
    core_scope_add_array(#arr, &(arr)[0], CORE_SCOPE_TYPE_OF((arr)[0]), (uint8_t)(count))

#else /* !CORE_SCOPE_ENABLED — every call compiles to nothing. Taking the
       * address keeps a watched variable "used" (no load is generated, even
       * for a volatile), so -Werror builds stay quiet. */

/* A call, not a literal 0, so `core_scope_watch(x);` as a bare statement does
 * not trip -Wunused-value. */
static inline int core_scope_off_(void) { return 0; }

#define core_scope_init()                        ((void)0)
#define core_scope_set_link(link)                ((void)(link))
#define core_scope_add(name, ptr, type)          ((void)(ptr), core_scope_off_())
#define core_scope_add_array(name, ptr, type, n) ((void)(ptr), core_scope_off_())
#define core_scope_set_interval_ms(ms)           ((void)0)
#define core_scope_declare_rate_hz(hz)           ((void)0)
#define core_scope_declare_rate_millihertz(mhz)  ((void)0)
#define core_scope_sample()                      ((void)0)
#define core_scope_pump()                        ((void)0)
#define core_scope_update()                      ((void)0)
#define core_scope_enable(on)                    ((void)0)
#define core_scope_active()                      (0)
#define core_scope_dropped()                     (0u)
#define core_scope_watch(var)                    ((void)&(var), core_scope_off_())
#define core_scope_watch_as(name, var)           ((void)&(var), core_scope_off_())
#define core_scope_watch_array(arr, count)       ((void)&(arr), core_scope_off_())

#endif /* CORE_SCOPE_ENABLED */

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=M title="No host-to-device control channel"
//   The stream is one-way. A host cannot start, stop or re-rate it, or pick
//   channels; core_scope_enable() is firmware-side only. USB has an RX path
//   and BLE would need a write characteristic beside Studio Scope.

// @studio unsupported tier=2 value=M title="No units or scaling in the stream"
//   The SDK is integer-only by convention (milli-units), so a channel in
//   mdeg plots in mdeg. A per-channel unit + scale frame is planned
//   (docs/scope-protocol.md, reserved frame types).

// @studio unsupported tier=3 value=L title="BLE packet size is assumed, not negotiated"
//   core_ble does not report the ATT MTU, so the BLE link sends 180-byte
//   notifications. A central that stayed at the 23-byte default would see
//   nothing but CRC failures. Driver-deferred: needs core_ble_mtu().

// @studio unsupported tier=3 value=L title="No 64-bit or double channels"
//   The wire has no 8-byte type. Watching one is a compile error by design.

#endif /* CORE_SCOPE_H */
