/**
 * thread_prov_ble.c — Thread provisioning over BLE (Core.ST.W5, BLE + Thread)
 *
 * A GATT service that lets a phone read the Thread status and move the node
 * to another Thread network: config.json `"thread": {"ble_provisioning":
 * true}` in a build with "ble" enabled too (coregen defines
 * THREAD_CFG_BLE_PROVISIONING in coregen/thread_dataset.h). Spec, byte
 * layouts and a worked example: docs/thread-provisioning-protocol.md.
 *
 *   0x5D00  service "Thread Provisioning"
 *   0x5D01  status   read + notify   39 bytes (thread_prov_encode_status)
 *   0x5D02  dataset  write           58 bytes (thread_prov_decode_dataset), staged
 *   0x5D03  control  write 1 byte (command); read + notify 4 bytes (result)
 *
 * DEVELOPMENT ONLY: no pairing. Any phone that connects may stage a dataset
 * and apply it; core_thread_prov_allow_write() (weak, returns 1 here) is the
 * gate. The network key is never readable: the dataset characteristic has no
 * read property, and its stored value is overwritten with zeros once copied.
 *
 * Contexts: the write callbacks run in the BLE host task (ble_svc.c) and only
 * copy bytes; core_thread_process() calls thread_prov_ble_process(), which
 * decodes, applies (core_thread_set_dataset() and friends) and publishes the
 * result and the status from the main loop.
 *
 * Registered by core_ble's service builder (sdk/core/core_ble.c, BLE +
 * Thread builds) after Studio Link, so it shifts no other handle.
 */

#if defined(BLE_ENABLED) && defined(THREAD_ENABLED)

#include <stdint.h>
#include <string.h>
#include "core_thread.h"
#include "core_timing.h"
#include "thread_prov.h"

#if __has_include("thread_dataset.h")
#include "thread_dataset.h"
#endif

#if defined(THREAD_CFG_BLE_PROVISIONING)

/* ble_svc.c */
extern uint16_t ble_svc_add_service_id(const char *name, uint8_t num_chars, uint16_t uuid16);
extern uint16_t ble_svc_add_char_id(uint16_t svc_handle, const char *name, uint16_t uuid16,
                                    uint8_t access, uint8_t value_len,
                                    void (*on_write)(const uint8_t *data, uint16_t len, void *ctx),
                                    void *ctx);
extern int      ble_svc_set_value(uint16_t char_handle, const void *data, uint16_t len);

#define ACC_READ    0x02u    /* CORE_BLE_READ */
#define ACC_WRITE   0x08u    /* CORE_BLE_WRITE (ble_svc adds write without response) */
#define ACC_NOTIFY  0x10u    /* CORE_BLE_NOTIFY */

#define STATUS_PERIOD_MS  1000u

static uint16_t s_ch_status, s_ch_dataset, s_ch_control;

/* Written by the BLE host task, taken by thread_prov_ble_process(). One byte
 * more than a dataset, so an over-long write is seen as one. */
static uint8_t  s_rx_ds[THREAD_PROV_DATASET_LEN + 1u];
static uint16_t s_rx_ds_len;
static uint8_t  s_rx_ds_pending;
static uint8_t  s_rx_cmd;
static uint16_t s_rx_cmd_len;
static uint8_t  s_rx_cmd_pending;

static core_thread_dataset_t s_staged;     /* holds a network key while staged */
static uint8_t  s_staged_ok;

static uint8_t  s_result[THREAD_PROV_RESULT_LEN];
static uint8_t  s_result_seq;
static uint8_t  s_result_dirty;

static uint8_t  s_status[THREAD_PROV_STATUS_LEN];
static uint8_t  s_status_dirty = 1;
static uint8_t  s_status_set;              /* s_status has reached the stack once */
static uint32_t s_status_at;

/* ============================================================
 * The gate (DEVELOPMENT ONLY default)
 * ============================================================ */

__attribute__((weak)) int core_thread_prov_allow_write(void)
{
    return 1;   /* open: any connected phone may set the network */
}

/* ============================================================
 * BLE host task: copy only
 * ============================================================ */

static void on_dataset(const uint8_t *data, uint16_t len, void *ctx)
{
    (void)ctx;
    uint16_t n = (len < sizeof s_rx_ds) ? len : (uint16_t)sizeof s_rx_ds;
    memcpy(s_rx_ds, data, n);
    s_rx_ds_len = len;
    s_rx_ds_pending = 1;
}

static void on_control(const uint8_t *data, uint16_t len, void *ctx)
{
    (void)ctx;
    s_rx_cmd = data[0];       /* ble_svc calls only for len > 0 */
    s_rx_cmd_len = len;
    s_rx_cmd_pending = 1;
}

void thread_prov_ble_register(void)
{
    /* Four records' worth (ble_svc sizes 3 attributes per characteristic; with
     * the CCCDs and user descriptions these three need 11 + the service). */
    uint16_t svc = ble_svc_add_service_id("Thread Provisioning", 4, THREAD_PROV_SVC_ID);
    s_ch_status = ble_svc_add_char_id(svc, "Thread status", THREAD_PROV_STATUS_ID,
                                      ACC_READ | ACC_NOTIFY, THREAD_PROV_STATUS_LEN, NULL, NULL);
    s_ch_dataset = ble_svc_add_char_id(svc, "Thread dataset", THREAD_PROV_DATASET_ID,
                                       ACC_WRITE, THREAD_PROV_DATASET_LEN, on_dataset, NULL);
    s_ch_control = ble_svc_add_char_id(svc, "Thread control", THREAD_PROV_CONTROL_ID,
                                       ACC_READ | ACC_WRITE | ACC_NOTIFY, THREAD_PROV_RESULT_LEN,
                                       on_control, NULL);
}

void thread_prov_ble_state_changed(void)
{
    s_status_dirty = 1;
}

/* ============================================================
 * Main loop (core_thread_process)
 * ============================================================ */

static void scrub(void *p, size_t n)
{
    volatile uint8_t *b = (volatile uint8_t *)p;
    while (n--)
        *b++ = 0;
}

static void drop_staged(void)
{
    scrub(&s_staged, sizeof s_staged);
    s_staged_ok = 0;
}

static void result(uint8_t op, uint8_t code, uint8_t detail)
{
    s_result[0] = op;
    s_result[1] = code;
    s_result[2] = ++s_result_seq;
    s_result[3] = detail;
    s_result_dirty = 1;
    s_status_dirty = 1;                       /* the flags may have changed too */
}

/* A core_thread_* return code as a result code (+ the otError for FAILED). */
static void result_rc(uint8_t op, int rc)
{
    if (rc == 0)
        result(op, THREAD_PROV_OK, 0);
    else if (rc == CORE_THREAD_ERR_STATE)
        result(op, THREAD_PROV_ERR_STATE, 0);
    else
        result(op, THREAD_PROV_ERR_FAILED, (uint8_t)core_thread_last_error());
}

static void handle_dataset(void)
{
    static const uint8_t zeros[THREAD_PROV_DATASET_LEN];
    uint16_t len = s_rx_ds_len;

    drop_staged();                            /* a new write replaces (or voids) the old one */
    if (!core_thread_prov_allow_write()) {
        result(THREAD_PROV_OP_DATASET, THREAD_PROV_ERR_DENIED, 0);
    } else {
        int rc = thread_prov_decode_dataset(s_rx_ds, len, &s_staged);
        s_staged_ok = (rc == THREAD_PROV_OK);
        result(THREAD_PROV_OP_DATASET, (uint8_t)rc, 0);
    }
    /* The key in the receive buffer and in the attribute's stored value. */
    scrub(s_rx_ds, sizeof s_rx_ds);
    (void)ble_svc_set_value(s_ch_dataset, zeros, sizeof zeros);
}

static void handle_command(uint8_t cmd, uint16_t len)
{
    if (!core_thread_prov_allow_write()) {
        result(cmd, THREAD_PROV_ERR_DENIED, 0);
        return;
    }
    if (len != 1u) {
        result(cmd, THREAD_PROV_ERR_LENGTH, 0);
        return;
    }
    switch (cmd) {
    case THREAD_PROV_CMD_APPLY:
        if (!s_staged_ok) {
            result(cmd, THREAD_PROV_ERR_NOTHING_STAGED, 0);
            return;
        }
        result_rc(cmd, core_thread_set_dataset(&s_staged));
        drop_staged();                        /* applied or refused: the key goes either way */
        return;
    case THREAD_PROV_CMD_FACTORY_RESET:
        result_rc(cmd, core_thread_factory_reset());
        return;
    case THREAD_PROV_CMD_LEAVE:
        result_rc(cmd, core_thread_set_enabled(0));
        return;
    case THREAD_PROV_CMD_ATTACH:
        result_rc(cmd, core_thread_set_enabled(1));
        return;
    case THREAD_PROV_CMD_CLEAR:
        drop_staged();
        result(cmd, THREAD_PROV_OK, 0);
        return;
    default:
        result(cmd, THREAD_PROV_ERR_COMMAND, 0);
        return;
    }
}

static void build_status(uint8_t out[THREAD_PROV_STATUS_LEN])
{
    thread_prov_status_t st;
    memset(&st, 0, sizeof st);
    core_thread_role_t role = core_thread_role();
    st.role = (uint8_t)role;
    st.rloc16 = core_thread_rloc16();
    st.partition_id = core_thread_partition_id();
    st.neighbors = core_thread_neighbor_count();
    st.parent_rssi = core_thread_parent_rssi();
    (void)core_thread_get_network(&st.net);   /* zeros before core_thread_start() */
    if (core_thread_is_provisioned())
        st.flags |= THREAD_PROV_FLAG_PROVISIONED;
    if (s_staged_ok)
        st.flags |= THREAD_PROV_FLAG_STAGED;
    if (role != CORE_THREAD_DISABLED)
        st.flags |= THREAD_PROV_FLAG_ENABLED;
    if (core_thread_prov_allow_write())
        st.flags |= THREAD_PROV_FLAG_WRITABLE;
    (void)thread_prov_encode_status(&st, out);
}

void thread_prov_ble_process(void)
{
    if (s_rx_ds_pending) {
        s_rx_ds_pending = 0;
        handle_dataset();
    }
    if (s_rx_cmd_pending) {
        s_rx_cmd_pending = 0;
        handle_command(s_rx_cmd, s_rx_cmd_len);
    }

    /* The result: set (and notified) once per change; retried while the
     * stack's TX queue is full. */
    if (s_result_dirty && ble_svc_set_value(s_ch_control, s_result, sizeof s_result) == 0)
        s_result_dirty = 0;

    /* The status: on a change OpenThread reports, after a write, and once a
     * second for the values it does not report (neighbors, parent RSSI).
     * Sent only when the bytes differ, so a quiet network is not notified. */
    uint32_t now = core_millis();
    if (!s_status_dirty && (uint32_t)(now - s_status_at) < STATUS_PERIOD_MS)
        return;
    s_status_at = now;
    uint8_t st[THREAD_PROV_STATUS_LEN];
    build_status(st);
    if (!s_status_set || memcmp(st, s_status, sizeof st) != 0) {
        if (ble_svc_set_value(s_ch_status, st, sizeof st) != 0)
            return;                           /* TX queue full: s_status_dirty stays, retry next pass */
        memcpy(s_status, st, sizeof st);
        s_status_set = 1;
    }
    s_status_dirty = 0;
}

#else /* !THREAD_CFG_BLE_PROVISIONING */

/* core_ble.c calls this in every BLE + Thread build. */
void thread_prov_ble_register(void)
{
}

#endif /* THREAD_CFG_BLE_PROVISIONING */
#endif /* BLE_ENABLED && THREAD_ENABLED */
