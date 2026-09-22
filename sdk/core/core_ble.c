/**
 * core_ble.c — BLE API for Core.ST.W5
 *
 * Wraps the low-level ble_app.c and ble_svc.c with a clean core_ API.
 */

#include "core_ble.h"
#include <stdint.h>

/* Low-level BLE functions (sdk/ble/) */
extern void ble_app_init(void);
extern int  ble_app_advertise(const char *name);
extern int  ble_app_stop_advertise(void);
extern void ble_app_set_services_cb(void (*cb)(void));

extern void     ble_svc_init(void);
extern uint16_t ble_svc_add_service(const char *name, uint8_t num_chars);
extern uint16_t ble_svc_add_char(uint16_t svc_handle, const char *name,
                                  uint8_t access, uint8_t value_len,
                                  void (*on_write)(const uint8_t *data, uint16_t len, void *ctx),
                                  void *ctx);
extern uint16_t ble_svc_add_service_id(const char *name, uint8_t num_chars, uint16_t uuid16);
extern uint16_t ble_svc_add_char_id(uint16_t svc_handle, const char *name, uint16_t uuid16,
                                    uint8_t access, uint8_t value_len,
                                    void (*on_write)(const uint8_t *data, uint16_t len, void *ctx),
                                    void *ctx);
extern uint16_t ble_svc_add_service_sig(const char *name, uint8_t num_chars, uint16_t uuid16);
extern uint16_t ble_svc_add_char_sig(uint16_t svc_handle, const char *name, uint16_t uuid16,
                                     uint8_t access, uint8_t value_len,
                                     void (*on_write)(const uint8_t *data, uint16_t len, void *ctx),
                                     void *ctx);
extern int      ble_svc_set_value(uint16_t char_handle, const void *data, uint16_t len);
extern int      ble_svc_notify(uint16_t char_handle);
extern int      ble_svc_subscribed(uint16_t char_handle);

/* Emitted by coregen when the project's BLE contract binds a characteristic to
 * a value; the weak no-op below stands in when it does not. */
void ble_contract_publish(void);
extern void     ble_svc_set_secure(uint8_t on);

/* Configurable parameters in ble_app.c */
extern uint8_t  ble_app_tx_power_code;
extern uint16_t ble_app_adv_interval_min;
extern uint16_t ble_app_adv_interval_max;
extern uint8_t  ble_app_pairing_enabled;

/* Sequencer */
extern void UTIL_SEQ_Run(uint32_t mask);

/* Connection state (ble_app_glue.c) */
extern volatile uint8_t ble_connected;
extern volatile uint16_t ble_conn_handle;
extern volatile uint8_t ble_conn_param_req_pending;
extern void (*ble_on_connect_cb)(void *ctx);
extern void *ble_on_connect_ctx;
extern void (*ble_on_disconnect_cb)(void *ctx);
extern void *ble_on_disconnect_ctx;

/* L2CAP connection-parameter update request (ble_wrap.c). Declared here to keep
 * core_ble.c free of the stack's internal headers. */
extern uint8_t aci_l2cap_connection_parameter_update_req(
    uint16_t conn_handle, uint16_t interval_min, uint16_t interval_max,
    uint16_t latency, uint16_t timeout);

/* Preferred connection parameters, in stack units (interval = 1.25 ms,
 * timeout = 10 ms), set via core_ble_set_conn_params(). */
static uint8_t  _conn_params_set;
static uint16_t _conn_interval_min;
static uint16_t _conn_interval_max;
static uint16_t _conn_latency;
static uint16_t _conn_timeout;

/* ---- State ---- */

static uint8_t _ble_initialized;
static uint8_t _ble_seq_warmup_done;
static uint32_t _ble_seq_count;

/* User's service builder function */
static void (*_services_builder)(void);

/* Extra builders from SDK modules (core_ble_add_services) */
#define CORE_BLE_EXTRA_BUILDERS 2
static void (*_extra_builders[CORE_BLE_EXTRA_BUILDERS])(void);

/* TX power codes: 0=low, 1=medium, 2=high */
static const uint8_t _tx_power_codes[] = { 0x00, 0x19, 0x1F };

/* Service builder callback — called from ble_app_init after SVCCTL_Init */
static void _register_services(void)
{
    ble_svc_init();
    /* If the app requested pairing, register characteristics behind an
     * encrypted link so the host actually bonds (see ble_svc_set_secure). */
    ble_svc_set_secure(ble_app_pairing_enabled);
    if (_services_builder) _services_builder();
    for (int i = 0; i < CORE_BLE_EXTRA_BUILDERS; i++) {
        if (_extra_builders[i]) _extra_builders[i]();
    }
}

/* ============================================================
 * Lifecycle
 * ============================================================ */

void core_ble_set_services(void (*builder)(void))
{
    _services_builder = builder;
}

int core_ble_add_services(void (*builder)(void))
{
    if (_ble_initialized || !builder) return -1;
    for (int i = 0; i < CORE_BLE_EXTRA_BUILDERS; i++) {
        if (_extra_builders[i] == builder) return 0;
        if (!_extra_builders[i]) {
            _extra_builders[i] = builder;
            return 0;
        }
    }
    return -1;
}

void core_ble_init(void)
{
    ble_app_set_services_cb(_register_services);
    ble_app_init();
    _ble_initialized = 1;
    _ble_seq_warmup_done = 0;
    _ble_seq_count = 0;
}

/* Saved name for re-advertising after disconnect */
static const char *_adv_name;

int core_ble_advertise(const char *name)
{
    if (!_ble_initialized) return -1;
    _adv_name = name;
    while (!_ble_seq_warmup_done) {
        UTIL_SEQ_Run(~0UL);
        _ble_seq_count++;
        if (_ble_seq_count > 100) _ble_seq_warmup_done = 1;
    }
    return ble_app_advertise(name);
}

int core_ble_stop_advertise(void)
{
    if (!_ble_initialized) return -1;
    return ble_app_stop_advertise();
}

/* Re-advertise flag (ble_app_glue.c) */
extern volatile uint8_t ble_need_readvertise;

void core_ble_process(void)
{
    /* Bound characteristics go out from here, so a contract that declares a
     * source needs nothing added to the application loop. It runs first: a
     * value produced this pass should reach a subscriber in this pass. */
    ble_contract_publish();

    UTIL_SEQ_Run(~0UL);
    if (!_ble_seq_warmup_done) {
        _ble_seq_count++;
        if (_ble_seq_count > 100) _ble_seq_warmup_done = 1;
    }

    /* Auto re-advertise after disconnect */
    if (ble_need_readvertise && _adv_name) {
        ble_need_readvertise = 0;
        ble_app_advertise(_adv_name);
    }

    /* Send the preferred connection parameters once connected, retrying until
     * the stack accepts the request — the link is busy with pairing/discovery
     * for the first moments after connecting, so an early attempt can bounce. */
    if (_conn_params_set && ble_conn_param_req_pending && ble_connected) {
        if (aci_l2cap_connection_parameter_update_req(
                ble_conn_handle, _conn_interval_min, _conn_interval_max,
                _conn_latency, _conn_timeout) == 0) {
            ble_conn_param_req_pending = 0;
        }
    }
}

/* ============================================================
 * Service builder
 * ============================================================ */

core_ble_svc_t core_ble_add_service(const char *name)
{
    /* We pass 8 as max chars per service — generous default.
     * The BLE stack allocates attribute records from the pool
     * configured in BleStack_Init. */
    return ble_svc_add_service(name, 8);
}

core_ble_char_t core_ble_add_char(core_ble_svc_t svc,
                                   const char *name,
                                   uint8_t access,
                                   uint8_t type,
                                   core_ble_write_cb on_write,
                                   void *ctx)
{
    return ble_svc_add_char(svc, name, access, type, on_write, ctx);
}

core_ble_svc_t core_ble_add_service_id(const char *name, uint16_t uuid16)
{
    return ble_svc_add_service_id(name, 8, uuid16);
}

core_ble_char_t core_ble_add_char_id(core_ble_svc_t svc,
                                     const char *name,
                                     uint16_t uuid16,
                                     uint8_t access,
                                     uint8_t type,
                                     core_ble_write_cb on_write,
                                     void *ctx)
{
    return ble_svc_add_char_id(svc, name, uuid16, access, type, on_write, ctx);
}

core_ble_svc_t core_ble_add_service_sig(const char *name, uint16_t uuid16)
{
    return ble_svc_add_service_sig(name, 8, uuid16);
}

core_ble_char_t core_ble_add_char_sig(core_ble_svc_t svc,
                                      const char *name,
                                      uint16_t uuid16,
                                      uint8_t access,
                                      uint8_t type,
                                      core_ble_write_cb on_write,
                                      void *ctx)
{
    return ble_svc_add_char_sig(svc, name, uuid16, access, type, on_write, ctx);
}

/* ============================================================
 * Runtime
 * ============================================================ */

int core_ble_set_value(core_ble_char_t ch, const void *data, uint16_t len)
{
    return ble_svc_set_value(ch, data, len);
}

int core_ble_notify(core_ble_char_t ch)
{
    return ble_svc_notify(ch);
}

int core_ble_subscribed(core_ble_char_t ch)
{
    return ble_connected && ble_svc_subscribed(ch);
}

/* Generated by coregen from the project's BLE contract when a characteristic
 * declares a `source`. Weak and empty here so core_ble_process() below links in
 * a project that has no contract, or one where nothing is bound. This is the
 * same rendezvous-by-name the tile event dispatchers use. */
__attribute__((weak)) void ble_contract_publish(void)
{
}

/* ============================================================
 * Connection
 * ============================================================ */

int core_ble_connected(void)      { return ble_connected; }
void core_ble_on_connect(void (*cb)(void *ctx), void *ctx)    { ble_on_connect_cb = cb; ble_on_connect_ctx = ctx; }
void core_ble_on_disconnect(void (*cb)(void *ctx), void *ctx) { ble_on_disconnect_cb = cb; ble_on_disconnect_ctx = ctx; }

/* ============================================================
 * Configuration
 * ============================================================ */

void core_ble_set_tx_power(uint8_t level)
{
    if (level > 2) level = 2;
    ble_app_tx_power_code = _tx_power_codes[level];
}

void core_ble_enable_pairing(void)
{
    ble_app_pairing_enabled = 1;
}

void core_ble_set_adv_interval(uint16_t min_ms, uint16_t max_ms)
{
    if (min_ms < 20) min_ms = 20;
    if (max_ms < min_ms) max_ms = min_ms;
    if (max_ms > 10240) max_ms = 10240;
    ble_app_adv_interval_min = (uint16_t)((uint32_t)min_ms * 8 / 5);
    ble_app_adv_interval_max = (uint16_t)((uint32_t)max_ms * 8 / 5);
}

void core_ble_set_conn_params(uint16_t min_ms, uint16_t max_ms,
                              uint16_t latency, uint16_t timeout_ms)
{
    /* Convert to stack units: connection interval = 1.25 ms, timeout = 10 ms,
     * clamped to the spec ranges (7.5-4000 ms interval, 100 ms-32 s timeout). */
    if (max_ms < min_ms) max_ms = min_ms;
    uint16_t imin = (uint16_t)((uint32_t)min_ms * 4 / 5);
    uint16_t imax = (uint16_t)((uint32_t)max_ms * 4 / 5);
    if (imin < 6)    imin = 6;
    if (imax < imin) imax = imin;
    if (imax > 3200) imax = 3200;
    uint16_t tmo = (uint16_t)(timeout_ms / 10);
    if (tmo < 10)   tmo = 10;
    if (tmo > 3200) tmo = 3200;
    if (latency > 499) latency = 499;

    _conn_interval_min = imin;
    _conn_interval_max = imax;
    _conn_latency      = latency;
    _conn_timeout      = tmo;
    _conn_params_set   = 1;

    /* If already connected, re-request on the next process tick. */
    if (ble_connected) ble_conn_param_req_pending = 1;
}
