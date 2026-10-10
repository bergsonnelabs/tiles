/**
 * ble_app_glue.c — Application-level callbacks for BLE stack
 *
 * Handles HCI events (connection, disconnection, PHY updates),
 * AMM callbacks, and debug signal stubs.
 */

#include <stdint.h>
#include <stddef.h>
#include "stm32_seq.h"
#include "stm32_mm.h"
#include "advanced_memory_manager.h"
#include "app_conf.h"
#include "svc_ctl.h"
#include "ble_std.h"
#include "auto/ble_types.h"
#include "auto/ble_gap_aci.h"
#include "core_config.h"

/* ---- SystemCoreClock (CMSIS convention) ---- */

uint32_t SystemCoreClock = SYSCLK_HZ;

/* ---- Connection state (readable from core_ble.c) ---- */

/* Defined in ble_svc.c, which owns the per-characteristic CCCD state. */
extern void ble_svc_clear_subscriptions(void);

volatile uint8_t  ble_connected;
volatile uint16_t ble_conn_handle;
/* Bumped on every connect and disconnect: ble_update.c drops a link's
 * authentication and partial frames when it moves. */
volatile uint32_t ble_link_gen;

/* Debug event log for pairing investigation */
#define EVT_LOG_SIZE 32
volatile uint32_t _evt_log[EVT_LOG_SIZE] __attribute__((used));
volatile uint32_t _evt_log_idx;

/* User callbacks (set via core_ble API) */
void (*ble_on_connect_cb)(void *ctx);
void *ble_on_connect_ctx;
void (*ble_on_disconnect_cb)(void *ctx);
void *ble_on_disconnect_ctx;

/* Re-advertise flag — set on disconnect, polled by core_ble_process */
volatile uint8_t ble_need_readvertise;

/* Connection-parameter request flag — set on connect, polled by
 * core_ble_process to send the preferred params once the link is up */
volatile uint8_t ble_conn_param_req_pending;

/* ---- GAP command response release ---- */

static void gap_cmd_resp_release(void)
{
    UTIL_SEQ_SetEvt(1U << CFG_IDLEEVT_PROC_GAP_COMPLETE);
}

/* ---- SVCCTL_App_Notification ---- */

static void elog(uint32_t v) {
    uint32_t i = _evt_log_idx;
    if (i < EVT_LOG_SIZE) { _evt_log[i] = v; _evt_log_idx = i + 1; }
}

SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification(void *p_Pckt)
{
    hci_event_pckt *p_event = (hci_event_pckt *)((hci_uart_pckt *)p_Pckt)->data;
    elog(0xE0000000 | p_event->evt);

    switch (p_event->evt)
    {
    case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
    {
        ble_connected = 0;
        ble_conn_handle = 0xFFFF;
        ble_link_gen++;
        ble_need_readvertise = 1;
        ble_conn_param_req_pending = 0;
        /* CCCD state belongs to the connection that set it. */
        ble_svc_clear_subscriptions();
        gap_cmd_resp_release();
        if (ble_on_disconnect_cb) ble_on_disconnect_cb(ble_on_disconnect_ctx);
        break;
    }

    case HCI_LE_META_EVT_CODE:
    {
        evt_le_meta_event *p_meta = (evt_le_meta_event *)p_event->data;

        switch (p_meta->subevent)
        {
        case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
        {
            hci_le_connection_complete_event_rp0 *p_conn =
                (hci_le_connection_complete_event_rp0 *)p_meta->data;
            if (p_conn->Status == 0) {
                ble_link_gen++;
                ble_connected = 1;
                ble_conn_handle = p_conn->Connection_Handle;
                ble_conn_param_req_pending = 1;
                if (ble_on_connect_cb) ble_on_connect_cb(ble_on_connect_ctx);
            }
            break;
        }

        case HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE:
        {
            hci_le_enhanced_connection_complete_event_rp0 *p_conn =
                (hci_le_enhanced_connection_complete_event_rp0 *)p_meta->data;
            if (p_conn->Status == 0) {
                ble_link_gen++;
                ble_connected = 1;
                ble_conn_handle = p_conn->Connection_Handle;
                ble_conn_param_req_pending = 1;
                if (ble_on_connect_cb) ble_on_connect_cb(ble_on_connect_ctx);
            }
            break;
        }

        case HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE:
            gap_cmd_resp_release();
            break;

        case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
            break;

        default:
            break;
        }
        break;
    }

    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
    {
        evt_blecore_aci *p_aci = (evt_blecore_aci *)p_event->data;
        elog(0xAC000000 | p_aci->ecode);
        switch (p_aci->ecode)
        {
        case 0x0401:  /* ACI_GAP_PAIRING_COMPLETE */
        {
            /* data[0-1] = conn handle, data[2] = status, data[3] = reason */
            uint8_t status = p_aci->data[2];
            uint8_t reason = p_aci->data[3];
            elog(0xBD000000 | (status << 8) | reason);
            break;
        }
        case 0x0402:  /* ACI_GAP_PASS_KEY_REQ */
            /* Pass key requested — respond with fixed pin 0 (Just Works) */
            aci_gap_pass_key_resp(ble_conn_handle, 0);
            break;
        case 0x0405:  /* ACI_GAP_BOND_LOST */
            aci_gap_allow_rebond(ble_conn_handle);
            break;
        case 0x0409:  /* ACI_GAP_NUMERIC_COMPARISON_VALUE */
            /* Auto-confirm numeric comparison (Just Works behavior) */
            aci_gap_numeric_comparison_value_confirm_yesno(ble_conn_handle, 1);
            break;
        default:
            break;
        }
        break;
    }

    default:
        break;
    }

    return SVCCTL_UserEvtFlowEnable;
}

/* ---- AMM callbacks ---- */

static uint32_t amm_buffer[CFG_AMM_POOL_SIZE];

void AMM_RegisterBasicMemoryManager(AMM_BasicMemoryManagerFunctions_t *const p_fns)
{
    p_fns->Init(amm_buffer, sizeof(amm_buffer));
}

void AMM_ProcessRequest(void)
{
    UTIL_SEQ_SetTask(1U << CFG_TASK_AMM, CFG_SEQ_PRIO_0);
}

/* ---- Link layer debug signal stubs ---- */

void LINKLAYER_DEBUG_SIGNAL_SET(uint8_t signal)  { (void)signal; }
void LINKLAYER_DEBUG_SIGNAL_RESET(uint8_t signal) { (void)signal; }
void LINKLAYER_DEBUG_SIGNAL_TOGGLE(uint8_t signal) { (void)signal; }
