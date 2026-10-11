/**
 * core_thread.c — core_thread.h on OpenThread (Core.ST.W5, THREAD_ENABLED builds)
 *
 * Compiled only when the build enables Thread (Makefile THREAD_ENABLED). The
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

#include "core_thread.h"
#include "thread_app.h"

#if __has_include("thread_dataset.h")
#include "thread_dataset.h"
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

/* ============================================================
 * Lifecycle
 * ============================================================ */

static void state_changed(otChangedFlags flags, void *ctx)
{
    (void)ctx;
    if ((flags & OT_CHANGED_THREAD_ROLE) && s_role_cb != NULL)
        s_role_cb((core_thread_role_t)otThreadGetDeviceRole(s_ot), s_role_ctx);
}

int core_thread_start(void)
{
    otError e;

    if (s_ot != NULL)
        return 0;
    otInstance *ot = thread_app_init();
    if (ot == NULL)
        return fail(OT_ERROR_FAILED);

    e = otSetStateChangedCallback(ot, state_changed, NULL);
    if (e != OT_ERROR_NONE)
        return fail(e);
    (void)otPlatRadioSetCcaEnergyDetectThreshold(ot, CCA_THRESHOLD_DBM);

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

void core_thread_process(void)
{
    thread_app_process();
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
    slot->cb(buf, n, info->mPeerAddr.mFields.m8, info->mPeerPort, slot->ctx);
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
