/**
 * ble_svc.c — Generic GATT service builder for Cores SDK
 *
 * Lets users define services and characteristics with simple calls:
 *   svc = core_ble_add_service("My Service");
 *   ch  = core_ble_add_char(svc, "Value", CORE_BLE_RW, CORE_BLE_UINT8, on_write);
 *
 * UUIDs are auto-generated from a Tiletown base UUID with incrementing IDs.
 * Base: 0000xx00-8E22-4541-9D4C-21EDAE82ED19
 *   Services: xx = 0xB0, 0xB1, 0xB2, ...
 *   Chars:    xx = 0xB0 + char_index_within_service
 */

#include <stdint.h>
#include <string.h>
#include "svc_ctl.h"
#include "ble_defs.h"
#include "ble_std.h"
#include "auto/ble_gatt_aci.h"
#include "auto/ble_vs_codes.h"
#include "auto/ble_types.h"

/* ---- Limits ---- */
#define MAX_SERVICES  4
/* Characteristic records. A registration past this returns handle 0 and every
 * later set_value on it fails silently — the Ring hit exactly that at 18 chars
 * (DIS 5 + Battery 1 + System 3 + Motion 2 + Haptics 2 + Audio 2 + Info 1 +
 * Touch 2) on 2026-09-22. Keep it above what the largest app registers; the
 * attribute budget in ble_app.c (CFG_BLE_NUM_GATT_ATTRIBUTES) is the real cap. */
#define MAX_CHARS     24

/* ---- Characteristic record ---- */
typedef struct {
    uint16_t svc_handle;    /* parent service handle */
    uint16_t char_handle;   /* characteristic handle from aci_gatt_add_char */
    uint8_t  access;        /* CHAR_PROP_* flags */
    uint8_t  value_len;     /* max value length */
    void (*on_write)(const uint8_t *data, uint16_t len, void *ctx);
    void *on_write_ctx;
    uint8_t  subscribed;    /* client has written the CCCD to enable notify */
} ble_char_record_t;

/* ---- State ---- */
static ble_char_record_t chars[MAX_CHARS];
static uint8_t char_count;
static uint8_t svc_count;

/* When set, characteristics are registered with encryption-required
 * permissions so the central must pair/bond before access. Driven by
 * core_ble_enable_pairing() (see ble_svc_set_secure). */
static uint8_t secure_mode;

/* ---- UUID generation ---- */
/* Base: 0000xx00-8E22-4541-9D4C-21EDAE82ED19 */
static void make_uuid(uint8_t *uuid, uint8_t id_hi, uint8_t id_lo)
{
    const uint8_t base[] = {0x19,0xED,0x82,0xAE,0xED,0x21,0x4C,0x9D,
                            0x41,0x45,0x22,0x8E,0x00,0x00,0x00,0x00};
    memcpy(uuid, base, 16);
    uuid[12] = id_lo;
    uuid[13] = id_hi;
}

/* Defined below; ble_svc_init() clears subscriptions before anything is added. */
void ble_svc_clear_subscriptions(void);

/* ---- Event handler ---- */

static SVCCTL_EvtAckStatus_t BLE_SVC_EventHandler(void *p_Event)
{
    hci_event_pckt *p_evt = (hci_event_pckt *)((hci_uart_pckt *)p_Event)->data;

    if (p_evt->evt != HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE)
        return SVCCTL_EvtNotAck;

    evt_blecore_aci *p_aci = (evt_blecore_aci *)p_evt->data;

    if (p_aci->ecode == ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE)
    {
        aci_gatt_attribute_modified_event_rp0 *p_mod =
            (aci_gatt_attribute_modified_event_rp0 *)p_aci->data;

        /* Find which characteristic was written */
        for (uint8_t i = 0; i < char_count; i++)
        {
            /* Attribute handle for the value is char_handle + 1 */
            if (p_mod->Attr_Handle == chars[i].char_handle + 1)
            {
                if (chars[i].on_write && p_mod->Attr_Data_Length > 0)
                {
                    chars[i].on_write(p_mod->Attr_Data, p_mod->Attr_Data_Length, chars[i].on_write_ctx);
                }
                return SVCCTL_EvtAckFlowEnable;
            }

            /* The CCCD sits at char_handle + 2 when the characteristic carries
             * NOTIFY: the stack allocates it with the characteristic, ahead of
             * any descriptor we add afterwards. A client writes it to subscribe
             * and unsubscribe, and that is the only signal there is that anyone
             * is listening. Bit 0 is notify, bit 1 indicate; either counts. */
            if (p_mod->Attr_Handle == chars[i].char_handle + 2)
            {
                if (p_mod->Attr_Data_Length > 0)
                    chars[i].subscribed = (p_mod->Attr_Data[0] & 0x03u) ? 1 : 0;
                return SVCCTL_EvtAckFlowEnable;
            }
        }
    }

    return SVCCTL_EvtNotAck;
}

/* ---- Public API ---- */

void ble_svc_init(void)
{
    ble_svc_clear_subscriptions();
    char_count = 0;
    svc_count = 0;
    secure_mode = 0;
    SVCCTL_RegisterSvcHandler(BLE_SVC_EventHandler);
}

/* Require encryption (pairing/bonding) on subsequently-added characteristics.
 * Called by core_ble after ble_svc_init() and before the service builder when
 * core_ble_enable_pairing() was requested. */
void ble_svc_set_secure(uint8_t on)
{
    secure_mode = on ? 1 : 0;
}

/* Core service add with an explicit 16-bit ID, placed in the shared base
 * UUID → 0000<uuid16>-8E22-4541-9D4C-21EDAE82ED19. */
static uint16_t svc_add_uuid16(uint8_t num_chars, uint16_t uuid16)
{
    uint8_t uuid[16];
    uint16_t svc_handle = 0;

    make_uuid(uuid, (uint8_t)(uuid16 >> 8), (uint8_t)(uuid16 & 0xFFu));

    /* max_attr_record = 1 (service) + 3 per char (value + CCCD/descriptors) */
    uint8_t max_attrs = 1 + num_chars * 3;

    aci_gatt_add_service(UUID_TYPE_128,
                         (Service_UUID_t *)uuid,
                         PRIMARY_SERVICE,
                         max_attrs,
                         &svc_handle);
    return svc_handle;
}

/* Cap on the Characteristic User Description (0x2901) we publish per
 * characteristic. Long enough for a readable label, short enough that a device
 * with many characteristics does not spend its attribute budget on prose. */
#define CHAR_DESC_MAX_LEN  20

/* NOTE: `name` is deliberately unused for SERVICES. GATT provides no
 * service-name mechanism — a Characteristic User Description (0x2901) attaches
 * to a characteristic, and there is no service equivalent. A scanner resolves a
 * service name only by recognising a SIG-assigned UUID in its own table, so a
 * custom service always displays as a raw UUID. The argument is kept for
 * readable call sites and for the generated contract/client, which do carry it.
 */
uint16_t ble_svc_add_service(const char *name, uint8_t num_chars)
{
    (void)name;
    /* Auto ID, sequential by registration order: 0xB000, 0xB100, … */
    uint16_t h = svc_add_uuid16(num_chars, (uint16_t)((0xB0 + svc_count) << 8));
    svc_count++;
    return h;
}

uint16_t ble_svc_add_service_id(const char *name, uint8_t num_chars, uint16_t uuid16)
{
    (void)name;  /* explicit ID → order-independent (the stable-contract path) */
    return svc_add_uuid16(num_chars, uuid16);
}

uint16_t ble_svc_add_service_sig(const char *name, uint8_t num_chars, uint16_t uuid16)
{
    (void)name;  /* SIG-adopted 16-bit service UUID (e.g. 0x180F Battery) */
    uint16_t svc_handle = 0;
    uint8_t max_attrs = 1 + num_chars * 3;
    aci_gatt_add_service(UUID_TYPE_16,
                         (Service_UUID_t *)&uuid16,
                         PRIMARY_SERVICE,
                         max_attrs,
                         &svc_handle);
    return svc_handle;
}

/* Register a characteristic from a prepared UUID (type + pointer). Shared by
 * the custom-128 and SIG-16 paths — props mapping + bookkeeping in one place. */
static uint16_t char_register(uint16_t svc_handle, uint8_t uuid_type, const void *uuid_ptr,
                              const char *name,
                              uint8_t access, uint8_t value_len,
                              void (*on_write)(const uint8_t *data, uint16_t len, void *ctx),
                              void *ctx)
{
    if (char_count >= MAX_CHARS) return 0;

    uint16_t char_handle = 0;

    /* Map access flags */
    uint8_t props = 0;
    if (access & 0x02) props |= CHAR_PROP_READ;
    if (access & 0x08) props |= CHAR_PROP_WRITE;
    if (access & 0x04) props |= CHAR_PROP_WRITE_WITHOUT_RESP;
    if (access & 0x10) props |= CHAR_PROP_NOTIFY;

    /* If writable, also allow write-without-response for convenience */
    if (props & CHAR_PROP_WRITE)
        props |= CHAR_PROP_WRITE_WITHOUT_RESP;

    uint8_t evt_mask = on_write ? GATT_NOTIFY_ATTRIBUTE_WRITE : 0;

    /* In secure mode, require an encrypted (paired/bonded) link to access the
     * characteristic. Reads/notifications gate on ENCRY_READ; writes on
     * ENCRY_WRITE. This is what triggers the host's pairing flow — without it,
     * a "bonding" configuration never prompts and the device never bonds. */
    uint8_t permissions = ATTR_PERMISSION_NONE;
    if (secure_mode) {
        if (props & (CHAR_PROP_READ | CHAR_PROP_NOTIFY | CHAR_PROP_INDICATE))
            permissions |= ATTR_PERMISSION_ENCRY_READ;
        if (props & (CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP))
            permissions |= ATTR_PERMISSION_ENCRY_WRITE;
    }

    aci_gatt_add_char(svc_handle,
                      uuid_type,
                      (Char_UUID_t *)uuid_ptr,
                      value_len,
                      props,
                      permissions,
                      evt_mask,
                      10,     /* encryption key size */
                      0,      /* not fixed length */
                      &char_handle);

    /* Publish the human-readable name as a Characteristic User Description
     * (0x2901) so generic scanners can show it. Without this the name argument
     * is invisible on the wire and every custom characteristic reads as a bare
     * 128-bit UUID in LightBlue / nRF Connect — which is exactly what it did
     * before, because this function used to drop `name` on the floor.
     *
     * Characteristics only. GATT has no service-name mechanism: a service is
     * resolved solely by a scanner recognising a SIG-assigned UUID, so a custom
     * service shows as a raw UUID no matter what we do here.
     *
     * Best-effort — a device that has run out of attribute space still gets a
     * working characteristic, just an anonymous one. */
    if (char_handle && name && name[0]) {
        uint16_t desc_handle = 0;
        uint16_t desc_uuid   = CHAR_USER_DESC_UUID;
        uint8_t  len         = 0;
        while (name[len] && len < CHAR_DESC_MAX_LEN) len++;
        aci_gatt_add_char_desc(svc_handle, char_handle,
                               UUID_TYPE_16, (Char_Desc_Uuid_t *)&desc_uuid,
                               len, len, (const uint8_t *)name,
                               ATTR_PERMISSION_NONE, ATTR_ACCESS_READ_ONLY,
                               0,      /* no event mask - we never see writes  */
                               10,     /* encryption key size (matches chars)  */
                               0,      /* fixed length                          */
                               &desc_handle);
    }

    /* Store record */
    chars[char_count].svc_handle  = svc_handle;
    chars[char_count].char_handle = char_handle;
    chars[char_count].access      = access;
    chars[char_count].value_len   = value_len;
    chars[char_count].on_write     = on_write;
    chars[char_count].on_write_ctx = ctx;
    char_count++;

    return char_handle;
}

/* Characteristic with an explicit 16-bit ID in the shared custom base UUID. */
static uint16_t char_add_uuid16(uint16_t svc_handle, uint16_t uuid16, const char *name,
                                uint8_t access, uint8_t value_len,
                                void (*on_write)(const uint8_t *data, uint16_t len, void *ctx),
                                void *ctx)
{
    uint8_t uuid[16];
    make_uuid(uuid, (uint8_t)(uuid16 >> 8), (uint8_t)(uuid16 & 0xFFu));
    return char_register(svc_handle, UUID_TYPE_128, uuid, name, access, value_len, on_write, ctx);
}

/* Characteristic with a 16-bit SIG-adopted UUID (e.g. 0x2A19 Battery Level). */
static uint16_t char_add_sig(uint16_t svc_handle, uint16_t uuid16, const char *name,
                             uint8_t access, uint8_t value_len,
                             void (*on_write)(const uint8_t *data, uint16_t len, void *ctx),
                             void *ctx)
{
    return char_register(svc_handle, UUID_TYPE_16, &uuid16, name, access, value_len, on_write, ctx);
}

uint16_t ble_svc_add_char(uint16_t svc_handle, const char *name,
                           uint8_t access, uint8_t value_len,
                           void (*on_write)(const uint8_t *data, uint16_t len, void *ctx),
                           void *ctx)
{
    /* Auto ID: last-added service hi byte + per-service characteristic index. */
    uint8_t char_idx = 0;
    for (uint8_t i = 0; i < char_count; i++)
        if (chars[i].svc_handle == svc_handle) char_idx++;
    uint16_t uuid16 = (uint16_t)(((0xB0 + svc_count - 1) << 8) | (char_idx + 1));
    return char_add_uuid16(svc_handle, uuid16, name, access, value_len, on_write, ctx);
}

uint16_t ble_svc_add_char_id(uint16_t svc_handle, const char *name, uint16_t uuid16,
                              uint8_t access, uint8_t value_len,
                              void (*on_write)(const uint8_t *data, uint16_t len, void *ctx),
                              void *ctx)
{
    return char_add_uuid16(svc_handle, uuid16, name, access, value_len, on_write, ctx);
}

uint16_t ble_svc_add_char_sig(uint16_t svc_handle, const char *name, uint16_t uuid16,
                               uint8_t access, uint8_t value_len,
                               void (*on_write)(const uint8_t *data, uint16_t len, void *ctx),
                               void *ctx)
{
    return char_add_sig(svc_handle, uuid16, name, access, value_len, on_write, ctx);
}

int ble_svc_set_value(uint16_t char_handle, const void *data, uint16_t len)
{
    /* Find the service handle for this characteristic */
    uint16_t svc_handle = 0;
    for (uint8_t i = 0; i < char_count; i++) {
        if (chars[i].char_handle == char_handle) {
            svc_handle = chars[i].svc_handle;
            break;
        }
    }

    tBleStatus ret = aci_gatt_update_char_value(svc_handle, char_handle, 0, len, (const uint8_t *)data);
    return (ret == 0) ? 0 : -1;
}

/* Has a client enabled notifications on this characteristic?
 *
 * There is no way to ask the stack, so this is the CCCD state as last written
 * by a client. It answers the only question a publisher needs: is anyone
 * listening. A characteristic without NOTIFY never has a CCCD and so always
 * reads 0, which is why core_ble_subscribed() is only consulted for notify
 * characteristics. */
int ble_svc_subscribed(uint16_t char_handle)
{
    for (uint8_t i = 0; i < char_count; i++)
        if (chars[i].char_handle == char_handle)
            return chars[i].subscribed;
    return 0;
}

/* Forget every subscription. Called on disconnect: CCCD state belongs to a
 * connection, and without this the next client would inherit the last one's
 * subscriptions and be sent notifications it never asked for. */
void ble_svc_clear_subscriptions(void)
{
    for (uint8_t i = 0; i < MAX_CHARS; i++)
        chars[i].subscribed = 0;
}

int ble_svc_notify(uint16_t char_handle)
{
    /* No-op: notifications are sent automatically by ble_svc_set_value()
     * when the client has subscribed (CCCD enabled). Calling set_value
     * both updates the stored value and triggers the notification. */
    (void)char_handle;
    return 0;
}
