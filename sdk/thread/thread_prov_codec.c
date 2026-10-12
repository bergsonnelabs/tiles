/**
 * thread_prov_codec.c — Thread provisioning wire format and boot precedence
 *
 * Plain C, no OpenThread / BLE / hardware: the firmware links it (BLE + Thread
 * and Thread builds; unused functions are dropped by --gc-sections) and
 * tests/host-thread-prov compiles it with the host cc. Layouts:
 * thread_prov.h and docs/thread-provisioning-protocol.md. Multi-byte integers
 * are little-endian; the ext PAN ID, mesh-local prefix and network key are
 * byte strings in the order they are written down (dead00beef00cafe is
 * de ad 00 be ef 00 ca fe).
 */

#include <stdint.h>
#include <string.h>
#include "thread_prov.h"

/* Dataset blob, version 1 (58 bytes). */
#define DS_VERSION    0
#define DS_FLAGS      1
#define DS_CHANNEL    2
#define DS_NAME_LEN   3
#define DS_PANID      4     /* u16 LE */
#define DS_EXT_PANID  6     /* 8 */
#define DS_MLP        14    /* 8 */
#define DS_KEY        22    /* 16 */
#define DS_TIMESTAMP  38    /* u32 LE */
#define DS_NAME       42    /* 16, zero padded */

/* Status, version 1 (39 bytes). */
#define ST_VERSION    0
#define ST_ROLE       1
#define ST_FLAGS      2
#define ST_CHANNEL    3
#define ST_PANID      4     /* u16 LE */
#define ST_RLOC16     6     /* u16 LE */
#define ST_PARTITION  8     /* u32 LE */
#define ST_NEIGHBORS  12
#define ST_RSSI       13    /* int8 */
#define ST_EXT_PANID  14    /* 8 */
#define ST_NAME_LEN   22
#define ST_NAME       23    /* 16, zero padded */

#define NAME_MAX      16u

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Length of the name (0-16), or 17 when no NUL ends it within 17 bytes. */
static uint32_t name_len(const char *name)
{
    uint32_t n = 0;
    while (n <= NAME_MAX && name[n] != '\0')
        n++;
    return n;
}

int thread_prov_check(const core_thread_dataset_t *ds)
{
    if (ds->channel < 11u || ds->channel > 26u)
        return THREAD_PROV_ERR_CHANNEL;
    if (ds->panid == 0xFFFFu)
        return THREAD_PROV_ERR_PANID;
    uint32_t n = name_len(ds->network_name);
    if (n < 1u || n > NAME_MAX)
        return THREAD_PROV_ERR_NAME;
    return THREAD_PROV_OK;
}

int thread_prov_decode_dataset(const uint8_t *buf, uint16_t len, core_thread_dataset_t *out)
{
    int rc = THREAD_PROV_OK;

    memset(out, 0, sizeof *out);
    if (buf == NULL || len != THREAD_PROV_DATASET_LEN)
        return THREAD_PROV_ERR_LENGTH;
    if (buf[DS_VERSION] != THREAD_PROV_VERSION)
        return THREAD_PROV_ERR_VERSION;
    if (buf[DS_FLAGS] != 0u)
        return THREAD_PROV_ERR_RESERVED;

    uint8_t n = buf[DS_NAME_LEN];
    if (n < 1u || n > NAME_MAX || memchr(&buf[DS_NAME], 0, n) != NULL)
        rc = THREAD_PROV_ERR_NAME;
    else
        memcpy(out->network_name, &buf[DS_NAME], n);    /* NUL from the memset */

    out->channel = buf[DS_CHANNEL];
    out->panid = get16(&buf[DS_PANID]);
    memcpy(out->ext_panid, &buf[DS_EXT_PANID], sizeof out->ext_panid);
    memcpy(out->mesh_local_prefix, &buf[DS_MLP], sizeof out->mesh_local_prefix);
    memcpy(out->network_key, &buf[DS_KEY], sizeof out->network_key);
    out->active_timestamp = get32(&buf[DS_TIMESTAMP]);

    if (rc == THREAD_PROV_OK)
        rc = thread_prov_check(out);
    if (rc != THREAD_PROV_OK)
        memset(out, 0, sizeof *out);            /* never hand back half a dataset (or its key) */
    return rc;
}

uint16_t thread_prov_encode_dataset(const core_thread_dataset_t *ds, uint8_t out[THREAD_PROV_DATASET_LEN])
{
    uint32_t n = name_len(ds->network_name);
    if (n > NAME_MAX)
        n = NAME_MAX;

    memset(out, 0, THREAD_PROV_DATASET_LEN);
    out[DS_VERSION] = THREAD_PROV_VERSION;
    out[DS_FLAGS] = 0u;
    out[DS_CHANNEL] = ds->channel;
    out[DS_NAME_LEN] = (uint8_t)n;
    put16(&out[DS_PANID], ds->panid);
    memcpy(&out[DS_EXT_PANID], ds->ext_panid, 8);
    memcpy(&out[DS_MLP], ds->mesh_local_prefix, 8);
    memcpy(&out[DS_KEY], ds->network_key, 16);
    put32(&out[DS_TIMESTAMP], ds->active_timestamp);
    memcpy(&out[DS_NAME], ds->network_name, n);
    return THREAD_PROV_DATASET_LEN;
}

uint16_t thread_prov_encode_status(const thread_prov_status_t *st, uint8_t out[THREAD_PROV_STATUS_LEN])
{
    uint32_t n = name_len(st->net.network_name);
    if (n > NAME_MAX)
        n = NAME_MAX;

    memset(out, 0, THREAD_PROV_STATUS_LEN);
    out[ST_VERSION] = THREAD_PROV_VERSION;
    out[ST_ROLE] = st->role;
    out[ST_FLAGS] = st->flags;
    out[ST_CHANNEL] = st->net.channel;
    put16(&out[ST_PANID], st->net.panid);
    put16(&out[ST_RLOC16], st->rloc16);
    put32(&out[ST_PARTITION], st->partition_id);
    out[ST_NEIGHBORS] = st->neighbors;
    out[ST_RSSI] = (uint8_t)st->parent_rssi;
    memcpy(&out[ST_EXT_PANID], st->net.ext_panid, 8);
    out[ST_NAME_LEN] = (uint8_t)n;
    memcpy(&out[ST_NAME], st->net.network_name, n);
    return THREAD_PROV_STATUS_LEN;
}

uint32_t thread_prov_timestamp(const core_thread_dataset_t *ds)
{
    return ds->active_timestamp ? ds->active_timestamp : 1u;
}

int thread_prov_boot_action(int marked, int stored_ok, int stored_is_config)
{
    if (marked)
        return stored_ok ? THREAD_PROV_BOOT_KEEP : THREAD_PROV_BOOT_WRITE_CONFIG_CLEAR_MARK;
    if (stored_ok && stored_is_config)
        return THREAD_PROV_BOOT_KEEP;              /* unchanged boot: nothing written */
    return THREAD_PROV_BOOT_WRITE_CONFIG;
}
