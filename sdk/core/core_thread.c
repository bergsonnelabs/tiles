/**
 * core_thread.c — core_thread.h on OpenThread (Core.ST.W5, THREAD_ENABLED builds)
 *
 * Compiled only when the build enables Thread (Makefile THREAD_ENABLED), alone
 * or together with BLE (then core_ble_init() must run first). Thread joins on
 * boot: the generated core_init() (or core_ble_init()) calls
 * core_thread_start(), and core_delay_ms() / core_watchdog_feed() run
 * core_thread_process() (core_thread_service_tick()). The
 * network credentials come from coregen/thread_dataset.h, which coregen writes
 * from config.json's "thread" block; a build forced with `make THREAD_ENABLED=1`
 * on a config without one uses the defaults below (the same as coregen's).
 */

#include <stdint.h>
#include <string.h>

#include <openthread/dataset.h>
#include <openthread/dataset_ftd.h>
#include <openthread/instance.h>
#include <openthread/ip6.h>
#include <openthread/link.h>
#include <openthread/message.h>
#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#include <openthread/udp.h>
#include <openthread/platform/radio.h>
#include <openthread/platform/settings.h>

#include "core_thread.h"
#include "hal_common.h"                  /* hal_tick() */
#include "thread_app.h"
#include "thread_prov.h"

#if __has_include("thread_dataset.h")
#include "thread_dataset.h"
#endif

/* Provisioning (config.json "thread": "provisioning" / "ble_provisioning";
 * coregen defines these in thread_dataset.h). Everything it adds here is
 * inside #if THREAD_CFG_PROVISIONING: builds without it compile none of it. */
#if defined(THREAD_CFG_BLE_PROVISIONING) && !defined(THREAD_CFG_PROVISIONING)
#define THREAD_CFG_PROVISIONING 1
#endif

/* Defaults: DEVELOPMENT ONLY (coregen uses the same, tools/coregen/coregen.py). */
#ifndef THREAD_CFG_CHANNEL
#define THREAD_CFG_CHANNEL            15
#define THREAD_CFG_PANID              0x1234
#define THREAD_CFG_NETWORK_NAME       "tiles-dev"
#define THREAD_CFG_NETWORK_KEY        { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, \
                                        0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff }
#define THREAD_CFG_EXT_PANID          { 0xde, 0xad, 0x00, 0xbe, 0xef, 0x00, 0xca, 0xfe }
#define THREAD_CFG_MESH_LOCAL_PREFIX  { 0xfd, 0xde, 0xad, 0x00, 0xbe, 0xef, 0x00, 0x00 }
#endif

#define CCA_THRESHOLD_DBM  (-70)    /* as ST's Thread applications */

typedef struct {
    otUdpSocket           sock;
    core_thread_udp_rx_cb cb;
    void                 *ctx;
    uint8_t               open;
} udp_slot_t;

static otInstance *s_ot;
static core_thread_role_cb s_role_cb;
static void *s_role_ctx;
static udp_slot_t s_udp[CORE_THREAD_UDP_SOCKETS];
static otUdpSocket s_tx_sock;            /* used when no socket is open */
static uint8_t s_tx_sock_open;
static int s_last_error;
static uint32_t s_service_at;            /* hal_tick() of the last core_thread_process() pass */

static int fail(otError e)
{
    s_last_error = (int)e;
    if (e == OT_ERROR_NO_BUFS)
        return CORE_THREAD_ERR_NOBUFS;
    if (e == OT_ERROR_INVALID_ARGS)
        return CORE_THREAD_ERR_ARG;
    if (e == OT_ERROR_INVALID_STATE || e == OT_ERROR_DETACHED)
        return CORE_THREAD_ERR_STATE;
    return CORE_THREAD_ERR_FAILED;
}

/* ============================================================
 * Dataset from config.json
 * ============================================================ */

static void config_dataset(otOperationalDataset *ds)
{
    static const uint8_t key[16] = THREAD_CFG_NETWORK_KEY;
    static const uint8_t xpan[8] = THREAD_CFG_EXT_PANID;
    static const uint8_t mlp[8]  = THREAD_CFG_MESH_LOCAL_PREFIX;

    ds->mActiveTimestamp.mSeconds = 1;
    ds->mActiveTimestamp.mTicks = 0;
    ds->mActiveTimestamp.mAuthoritative = false;
    ds->mComponents.mIsActiveTimestampPresent = true;

    memcpy(ds->mNetworkKey.m8, key, sizeof key);
    ds->mComponents.mIsNetworkKeyPresent = true;

    memset(&ds->mNetworkName, 0, sizeof ds->mNetworkName);
    strncpy(ds->mNetworkName.m8, THREAD_CFG_NETWORK_NAME, OT_NETWORK_NAME_MAX_SIZE);
    ds->mComponents.mIsNetworkNamePresent = true;

    memcpy(ds->mExtendedPanId.m8, xpan, sizeof xpan);
    ds->mComponents.mIsExtendedPanIdPresent = true;

    memcpy(ds->mMeshLocalPrefix.m8, mlp, sizeof mlp);
    ds->mComponents.mIsMeshLocalPrefixPresent = true;

    ds->mPanId = (otPanId)THREAD_CFG_PANID;
    ds->mComponents.mIsPanIdPresent = true;

    ds->mChannel = (uint16_t)THREAD_CFG_CHANNEL;
    ds->mComponents.mIsChannelPresent = true;

    /* No PSKc: it only serves commissioning, which this milestone does not do,
     * and leaving it out keeps every node's dataset identical. */
    ds->mComponents.mIsPskcPresent = false;
    ds->mComponents.mIsPendingTimestampPresent = false;
    ds->mComponents.mIsDelayPresent = false;
}

static bool dataset_matches(const otOperationalDataset *a, const otOperationalDataset *b)
{
    return a->mComponents.mIsActiveTimestampPresent &&
           a->mActiveTimestamp.mSeconds == b->mActiveTimestamp.mSeconds &&
           a->mComponents.mIsNetworkKeyPresent &&
           memcmp(a->mNetworkKey.m8, b->mNetworkKey.m8, OT_NETWORK_KEY_SIZE) == 0 &&
           a->mComponents.mIsNetworkNamePresent &&
           strncmp(a->mNetworkName.m8, b->mNetworkName.m8, OT_NETWORK_NAME_MAX_SIZE) == 0 &&
           a->mComponents.mIsExtendedPanIdPresent &&
           memcmp(a->mExtendedPanId.m8, b->mExtendedPanId.m8, OT_EXT_PAN_ID_SIZE) == 0 &&
           a->mComponents.mIsMeshLocalPrefixPresent &&
           memcmp(a->mMeshLocalPrefix.m8, b->mMeshLocalPrefix.m8, OT_MESH_LOCAL_PREFIX_SIZE) == 0 &&
           a->mComponents.mIsPanIdPresent && a->mPanId == b->mPanId &&
           a->mComponents.mIsChannelPresent && a->mChannel == b->mChannel;
}

#if defined(THREAD_CFG_PROVISIONING)
/* ============================================================
 * Provisioning: the stored network and its mark
 * ============================================================ */

/* The mark: a vendor key in OpenThread's settings (otPlatSettings, so it is in
 * the same flash snapshot as the dataset it describes; keys 0x8000-0xFFFF are
 * the vendor's, openthread/platform/settings.h). Present = the active dataset
 * was provisioned at run time and wins over config.json at boot. */
#define PROV_MARK_KEY      0x8B01u
#define PROV_MARK_VERSION  1u

static const uint8_t s_mark[4] = { 'T', 'P', PROV_MARK_VERSION, 0 };

static bool mark_present(otInstance *ot)
{
    uint8_t v[sizeof s_mark];
    uint16_t n = sizeof v;
    return otPlatSettingsGet(ot, PROV_MARK_KEY, 0, v, &n) == OT_ERROR_NONE &&
           n == sizeof v && memcmp(v, s_mark, 3) == 0;
}

static otError mark_set(otInstance *ot)
{
    return otPlatSettingsSet(ot, PROV_MARK_KEY, s_mark, sizeof s_mark);
}

static void mark_clear(otInstance *ot)
{
    (void)otPlatSettingsDelete(ot, PROV_MARK_KEY, -1);   /* NOT_FOUND: nothing to clear */
}

/* Every field config_dataset() sets is there (a dataset core_thread can use). */
static bool dataset_complete(const otOperationalDataset *a)
{
    return a->mComponents.mIsActiveTimestampPresent && a->mComponents.mIsNetworkKeyPresent &&
           a->mComponents.mIsNetworkNamePresent && a->mComponents.mIsExtendedPanIdPresent &&
           a->mComponents.mIsMeshLocalPrefixPresent && a->mComponents.mIsPanIdPresent &&
           a->mComponents.mIsChannelPresent;
}

/* A provisioned network as a dataset (fields as config_dataset() sets them). */
static void prov_dataset(otOperationalDataset *ds, const core_thread_dataset_t *src)
{
    ds->mActiveTimestamp.mSeconds = thread_prov_timestamp(src);
    ds->mActiveTimestamp.mTicks = 0;
    ds->mActiveTimestamp.mAuthoritative = false;
    ds->mComponents.mIsActiveTimestampPresent = true;

    memcpy(ds->mNetworkKey.m8, src->network_key, OT_NETWORK_KEY_SIZE);
    ds->mComponents.mIsNetworkKeyPresent = true;

    memset(&ds->mNetworkName, 0, sizeof ds->mNetworkName);
    strncpy(ds->mNetworkName.m8, src->network_name, OT_NETWORK_NAME_MAX_SIZE);
    ds->mComponents.mIsNetworkNamePresent = true;

    memcpy(ds->mExtendedPanId.m8, src->ext_panid, OT_EXT_PAN_ID_SIZE);
    ds->mComponents.mIsExtendedPanIdPresent = true;

    memcpy(ds->mMeshLocalPrefix.m8, src->mesh_local_prefix, OT_MESH_LOCAL_PREFIX_SIZE);
    ds->mComponents.mIsMeshLocalPrefixPresent = true;

    ds->mPanId = (otPanId)src->panid;
    ds->mComponents.mIsPanIdPresent = true;

    ds->mChannel = src->channel;
    ds->mComponents.mIsChannelPresent = true;

    ds->mComponents.mIsPskcPresent = false;      /* as config_dataset() */
    ds->mComponents.mIsPendingTimestampPresent = false;
    ds->mComponents.mIsDelayPresent = false;
}

/* Datasets hold the network key: clear copies on the stack before returning. */
static void scrub(void *p, size_t n)
{
    volatile uint8_t *b = (volatile uint8_t *)p;
    while (n--)
        *b++ = 0;
}

/* Boot: which dataset to run (thread_prov_boot_action has the rule). */
static otError boot_select_dataset(otInstance *ot)
{
    otOperationalDataset want, have;
    otError e;

    memset(&want, 0, sizeof want);
    e = otDatasetCreateNewNetwork(ot, &want);
    if (e != OT_ERROR_NONE)
        return e;
    config_dataset(&want);

    memset(&have, 0, sizeof have);
    bool have_ok = otDatasetGetActive(ot, &have) == OT_ERROR_NONE && dataset_complete(&have);
    int action = thread_prov_boot_action(mark_present(ot), have_ok, have_ok && dataset_matches(&have, &want));

    if (action == THREAD_PROV_BOOT_WRITE_CONFIG_CLEAR_MARK)
        mark_clear(ot);
    if (action != THREAD_PROV_BOOT_KEEP)
        e = otDatasetSetActive(ot, &want);
    scrub(&want, sizeof want);
    scrub(&have, sizeof have);
    return e;
}

/* Switch the active dataset to `want` with Thread stopped, set or clear the
 * mark, start Thread again. On a refusal: the previous dataset and mark back,
 * Thread restarted (when it was running), and the error returned. */
static otError switch_network(otInstance *ot, const otOperationalDataset *want, bool mark)
{
    otOperationalDataset have, check;
    otError e;
    bool was_running = otThreadGetDeviceRole(ot) != OT_DEVICE_ROLE_DISABLED;
    bool was_marked = mark_present(ot);

    memset(&have, 0, sizeof have);
    bool have_ok = otDatasetGetActive(ot, &have) == OT_ERROR_NONE;
    bool same = have_ok && dataset_complete(&have) && dataset_matches(&have, want);

    if (same && was_marked == mark && was_running) {
        scrub(&have, sizeof have);
        return OT_ERROR_NONE;                     /* already there: nothing to write */
    }

    if (!same && was_running)
        (void)otThreadSetEnabled(ot, false);      /* Stop() cannot fail */

    e = OT_ERROR_NONE;
    if (!same) {
        (void)otDatasetSetActive(ot, want);       /* returns NONE in this OpenThread; check below */
        memset(&check, 0, sizeof check);
        if (otDatasetGetActive(ot, &check) != OT_ERROR_NONE || !dataset_matches(&check, want))
            e = OT_ERROR_NO_BUFS;                 /* the settings did not take it */
        scrub(&check, sizeof check);
    }
    if (e == OT_ERROR_NONE) {
        if (mark)
            e = mark_set(ot);
        else
            mark_clear(ot);
    }
    if (e == OT_ERROR_NONE && otThreadGetDeviceRole(ot) == OT_DEVICE_ROLE_DISABLED)
        e = otThreadSetEnabled(ot, true);

    if (e != OT_ERROR_NONE) {
        (void)otThreadSetEnabled(ot, false);
        if (have_ok)
            (void)otDatasetSetActive(ot, &have);
        if (was_marked)
            (void)mark_set(ot);
        else
            mark_clear(ot);
        if (was_running)
            (void)otThreadSetEnabled(ot, true);
    }
    scrub(&have, sizeof have);
    return e;
}
#endif /* THREAD_CFG_PROVISIONING */

/* ============================================================
 * Lifecycle
 * ============================================================ */

static void state_changed(otChangedFlags flags, void *ctx)
{
    (void)ctx;
    if ((flags & OT_CHANGED_THREAD_ROLE) && s_role_cb != NULL)
        s_role_cb((core_thread_role_t)otThreadGetDeviceRole(s_ot), s_role_ctx);
#if defined(THREAD_CFG_BLE_PROVISIONING)
    if (flags & (OT_CHANGED_THREAD_ROLE | OT_CHANGED_THREAD_PARTITION_ID | OT_CHANGED_THREAD_CHANNEL |
                 OT_CHANGED_THREAD_PANID | OT_CHANGED_THREAD_NETWORK_NAME | OT_CHANGED_THREAD_EXT_PANID |
                 OT_CHANGED_ACTIVE_DATASET | OT_CHANGED_THREAD_RLOC_ADDED))
        thread_prov_ble_state_changed();
#endif
}

int core_thread_start(void)
{
    otError e;

    if (s_ot != NULL)
        return 0;
#if defined(BLE_ENABLED)
    /* BLE + Thread: the radio comes up in ST's order, BLE first (thread_app.c). */
    extern int ble_app_ready(void);
    if (!ble_app_ready())
        return fail(OT_ERROR_INVALID_STATE);
#endif
    otInstance *ot = thread_app_init();
    if (ot == NULL)
        return fail(OT_ERROR_FAILED);

    e = otSetStateChangedCallback(ot, state_changed, NULL);
    if (e != OT_ERROR_NONE)
        return fail(e);
    (void)otPlatRadioSetCcaEnergyDetectThreshold(ot, CCA_THRESHOLD_DBM);

#if defined(THREAD_CFG_PROVISIONING)
    /* Precedence: a provisioned network (the mark beside the dataset in the
     * settings) wins over config.json's, which is the factory default. Without
     * the mark, as below: config.json's dataset, written only when it differs. */
    e = boot_select_dataset(ot);
    if (e != OT_ERROR_NONE)
        return fail(e);
#else
    /* Set the active dataset only when the one OpenThread restored from flash
     * differs: every node built from the same config gets the same dataset,
     * and an unchanged boot writes nothing to flash. Start from
     * otDatasetCreateNewNetwork() so the fields config.json does not set
     * (channel mask, security policy) get OpenThread's defaults. */
    {
        otOperationalDataset want, have;
        memset(&want, 0, sizeof want);
        e = otDatasetCreateNewNetwork(ot, &want);
        if (e != OT_ERROR_NONE)
            return fail(e);
        config_dataset(&want);

        memset(&have, 0, sizeof have);
        if (otDatasetGetActive(ot, &have) != OT_ERROR_NONE || !dataset_matches(&have, &want)) {
            e = otDatasetSetActive(ot, &want);
            if (e != OT_ERROR_NONE)
                return fail(e);
        }
    }
#endif

    otPlatRadioEnableSrcMatch(ot, true);

    e = otIp6SetEnabled(ot, true);
    if (e != OT_ERROR_NONE)
        return fail(e);
    e = otThreadSetEnabled(ot, true);
    if (e != OT_ERROR_NONE)
        return fail(e);

    s_ot = ot;
    return 0;
}

/* Called from the application's loop and, in every Thread build, from
 * core_delay_ms() (each pass of its wait) and core_watchdog_feed() (through
 * core_thread_service_tick()), so a program that never mentions core_thread
 * stays on the network. It does nothing before core_thread_start(), inside a
 * sequencer pass (an OpenThread / BLE callback that delays or feeds must not
 * re-enter the stacks), in an interrupt handler, or with interrupts masked. */
void core_thread_process(void)
{
    uint32_t ipsr, primask;
    __asm volatile ("mrs %0, ipsr" : "=r"(ipsr));
    __asm volatile ("mrs %0, primask" : "=r"(primask));
    if (s_ot == NULL || thread_app_seq_depth != 0u || (ipsr & 0x1FFu) != 0u || (primask & 1u) != 0u)
        return;
    s_service_at = hal_tick();
#if defined(THREAD_CFG_BLE_PROVISIONING)
    thread_prov_ble_process();
#endif
    thread_app_process();
}

/* core_watchdog_feed()'s hook (Thread builds): a loop that feeds but never
 * delays still gets Thread serviced, at most every SERVICE_TICK_MS, so a busy
 * loop loses at most one sequencer pass (<= 1 ms, its WFI) in that period. */
#define SERVICE_TICK_MS  5u

void core_thread_service_tick(void)
{
    if ((uint32_t)(hal_tick() - s_service_at) >= SERVICE_TICK_MS)
        core_thread_process();
}

core_thread_role_t core_thread_role(void)
{
    if (s_ot == NULL)
        return CORE_THREAD_DISABLED;
    return (core_thread_role_t)otThreadGetDeviceRole(s_ot);
}

void core_thread_on_role_change(core_thread_role_cb cb, void *ctx)
{
    s_role_ctx = ctx;
    s_role_cb = cb;
}

/* ============================================================
 * UDP
 * ============================================================ */

static void udp_receive(void *ctx, otMessage *msg, const otMessageInfo *info)
{
    udp_slot_t *slot = (udp_slot_t *)ctx;
    static uint8_t buf[CORE_THREAD_UDP_MAX_PAYLOAD];

    if (slot == NULL || slot->cb == NULL)
        return;
    uint16_t n = otMessageRead(msg, otMessageGetOffset(msg), buf, sizeof buf);
    core_thread_udp_packet_t pkt;
    pkt.data = buf;
    pkt.len = n;
    memcpy(pkt.src, info->mPeerAddr.mFields.m8, sizeof pkt.src);
    pkt.src_port = info->mPeerPort;
    slot->cb(&pkt, slot->ctx);
}

int core_thread_udp_open(uint16_t port, core_thread_udp_rx_cb rx_cb, void *ctx)
{
    if (s_ot == NULL)
        return CORE_THREAD_ERR_STATE;
    if (port == 0)
        return CORE_THREAD_ERR_ARG;

    udp_slot_t *slot = NULL;
    for (unsigned i = 0; i < CORE_THREAD_UDP_SOCKETS; i++) {
        if (!s_udp[i].open) {
            slot = &s_udp[i];
            break;
        }
    }
    if (slot == NULL)
        return CORE_THREAD_ERR_FULL;

    memset(&slot->sock, 0, sizeof slot->sock);
    slot->cb = rx_cb;
    slot->ctx = ctx;
    otError e = otUdpOpen(s_ot, &slot->sock, udp_receive, slot);
    if (e != OT_ERROR_NONE)
        return fail(e);

    otSockAddr sa;
    memset(&sa, 0, sizeof sa);          /* :: = any address */
    sa.mPort = port;
    e = otUdpBind(s_ot, &slot->sock, &sa, OT_NETIF_THREAD);
    if (e != OT_ERROR_NONE) {
        (void)otUdpClose(s_ot, &slot->sock);
        return fail(e);
    }
    slot->open = 1;
    return 0;
}

static otUdpSocket *tx_socket(void)
{
    for (unsigned i = 0; i < CORE_THREAD_UDP_SOCKETS; i++)
        if (s_udp[i].open)
            return &s_udp[i].sock;
    if (!s_tx_sock_open) {
        memset(&s_tx_sock, 0, sizeof s_tx_sock);
        if (otUdpOpen(s_ot, &s_tx_sock, NULL, NULL) != OT_ERROR_NONE)
            return NULL;
        s_tx_sock_open = 1;
    }
    return &s_tx_sock;
}

static int udp_send_to(const otIp6Address *dst, uint16_t port, const void *buf, uint16_t len)
{
    if (s_ot == NULL)
        return CORE_THREAD_ERR_STATE;
    if ((buf == NULL && len) || port == 0)
        return CORE_THREAD_ERR_ARG;
    otDeviceRole role = otThreadGetDeviceRole(s_ot);
    if (role == OT_DEVICE_ROLE_DISABLED || role == OT_DEVICE_ROLE_DETACHED)
        return CORE_THREAD_ERR_STATE;

    otUdpSocket *sock = tx_socket();
    if (sock == NULL)
        return fail(OT_ERROR_FAILED);

    otMessage *msg = otUdpNewMessage(s_ot, NULL);
    if (msg == NULL)
        return fail(OT_ERROR_NO_BUFS);
    otError e = otMessageAppend(msg, buf, len);
    if (e == OT_ERROR_NONE) {
        otMessageInfo info;
        memset(&info, 0, sizeof info);
        info.mPeerAddr = *dst;
        info.mPeerPort = port;
        e = otUdpSend(s_ot, sock, msg, &info);   /* takes the message on success */
    }
    if (e != OT_ERROR_NONE) {
        otMessageFree(msg);
        return fail(e);
    }
    return 0;
}

int core_thread_udp_send(const uint8_t addr6[16], uint16_t port, const void *buf, uint16_t len)
{
    if (addr6 == NULL)
        return CORE_THREAD_ERR_ARG;
    otIp6Address dst;
    memcpy(dst.mFields.m8, addr6, sizeof dst.mFields.m8);
    return udp_send_to(&dst, port, buf, len);
}

int core_thread_udp_broadcast(uint16_t port, const void *buf, uint16_t len)
{
    /* ff03::1, realm-local all-nodes: every Thread node in the mesh, forwarded
     * by the routers (ff02::1 would reach radio neighbours only). */
    otIp6Address dst;
    memset(&dst, 0, sizeof dst);
    dst.mFields.m8[0] = 0xff;
    dst.mFields.m8[1] = 0x03;
    dst.mFields.m8[15] = 0x01;
    return udp_send_to(&dst, port, buf, len);
}

/* ============================================================
 * Diagnostics
 * ============================================================ */

uint16_t core_thread_rloc16(void)
{
    if (s_ot == NULL || core_thread_role() <= CORE_THREAD_DETACHED)
        return 0xFFFE;
    return otThreadGetRloc16(s_ot);
}

void core_thread_ext_addr(uint8_t out[8])
{
    if (out == NULL)
        return;
    otInstance *ot = thread_app_instance();
    if (ot == NULL) {
        memset(out, 0, 8);
        return;
    }
    memcpy(out, otLinkGetExtendedAddress(ot)->m8, 8);
}

uint32_t core_thread_partition_id(void)
{
    if (s_ot == NULL || core_thread_role() <= CORE_THREAD_DETACHED)
        return 0;
    return otThreadGetPartitionId(s_ot);
}

uint8_t core_thread_neighbor_count(void)
{
    if (s_ot == NULL)
        return 0;
    otNeighborInfoIterator it = OT_NEIGHBOR_INFO_ITERATOR_INIT;
    otNeighborInfo info;
    uint8_t n = 0;
    while (otThreadGetNextNeighborInfo(s_ot, &it, &info) == OT_ERROR_NONE && n < 255u)
        n++;
    return n;
}

int8_t core_thread_parent_rssi(void)
{
    int8_t rssi;
    if (s_ot == NULL || core_thread_role() != CORE_THREAD_CHILD)
        return 127;
    if (otThreadGetParentAverageRssi(s_ot, &rssi) != OT_ERROR_NONE)
        return 127;
    return rssi;
}

int core_thread_last_error(void)
{
    return s_last_error;
}

void *core_thread_instance(void)
{
    return thread_app_instance();
}

/* ============================================================
 * Network: run-time provisioning
 * ============================================================ */

int core_thread_get_network(core_thread_dataset_t *out)
{
    if (out == NULL)
        return CORE_THREAD_ERR_ARG;
    memset(out, 0, sizeof *out);
    if (s_ot == NULL)
        return CORE_THREAD_ERR_STATE;

    otOperationalDataset ds;
    memset(&ds, 0, sizeof ds);
    otError e = otDatasetGetActive(s_ot, &ds);
    if (e != OT_ERROR_NONE)
        return fail(e);
    out->channel = (uint8_t)ds.mChannel;
    out->panid = ds.mPanId;
    memcpy(out->ext_panid, ds.mExtendedPanId.m8, sizeof out->ext_panid);
    memcpy(out->mesh_local_prefix, ds.mMeshLocalPrefix.m8, sizeof out->mesh_local_prefix);
    memcpy(out->network_name, ds.mNetworkName.m8, OT_NETWORK_NAME_MAX_SIZE);
    out->network_name[OT_NETWORK_NAME_MAX_SIZE] = '\0';
    out->active_timestamp = (uint32_t)ds.mActiveTimestamp.mSeconds;
    /* network_key stays zero: the key is never handed out. */
    volatile uint8_t *k = ds.mNetworkKey.m8;
    for (unsigned i = 0; i < OT_NETWORK_KEY_SIZE; i++)
        k[i] = 0;
    return 0;
}

int core_thread_set_enabled(int on)
{
    if (s_ot == NULL)
        return CORE_THREAD_ERR_STATE;
    otError e = otThreadSetEnabled(s_ot, on != 0);
    return (e == OT_ERROR_NONE) ? 0 : fail(e);
}

#if defined(THREAD_CFG_PROVISIONING)

int core_thread_set_dataset(const core_thread_dataset_t *ds)
{
    if (ds == NULL || thread_prov_check(ds) != THREAD_PROV_OK)
        return CORE_THREAD_ERR_ARG;
    if (s_ot == NULL)
        return CORE_THREAD_ERR_STATE;

    otOperationalDataset want;
    memset(&want, 0, sizeof want);
    otError e = otDatasetCreateNewNetwork(s_ot, &want);   /* channel mask, security policy */
    if (e == OT_ERROR_NONE) {
        prov_dataset(&want, ds);
        e = switch_network(s_ot, &want, true);
    }
    scrub(&want, sizeof want);
    return (e == OT_ERROR_NONE) ? 0 : fail(e);
}

int core_thread_factory_reset(void)
{
    if (s_ot == NULL)
        return CORE_THREAD_ERR_STATE;

    otOperationalDataset want;
    memset(&want, 0, sizeof want);
    otError e = otDatasetCreateNewNetwork(s_ot, &want);
    if (e == OT_ERROR_NONE) {
        config_dataset(&want);
        e = switch_network(s_ot, &want, false);
    }
    scrub(&want, sizeof want);
    return (e == OT_ERROR_NONE) ? 0 : fail(e);
}

int core_thread_is_provisioned(void)
{
    return (s_ot != NULL && mark_present(s_ot)) ? 1 : 0;
}

#else /* !THREAD_CFG_PROVISIONING: the API is there, the feature is not */

int core_thread_set_dataset(const core_thread_dataset_t *ds)
{
    (void)ds;
    return CORE_THREAD_ERR_UNSUPPORTED;
}

int core_thread_factory_reset(void)
{
    return CORE_THREAD_ERR_UNSUPPORTED;
}

int core_thread_is_provisioned(void)
{
    return 0;
}

#endif /* THREAD_CFG_PROVISIONING */
