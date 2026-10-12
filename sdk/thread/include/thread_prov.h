/**
 * thread_prov.h — Thread provisioning: wire format and boot precedence
 *
 * Internal to the SDK (sdk/core/core_thread.c, sdk/thread/thread_prov_ble.c,
 * tests/host-thread-prov). Applications use core_thread.h.
 *
 * The codec (thread_prov_codec.c) is plain C with no OpenThread or BLE
 * dependency, so the host tests compile it with the system cc. The byte
 * layouts here are the protocol: docs/thread-provisioning-protocol.md
 * documents them, and the host test pins the worked example printed there.
 *
 * DEVELOPMENT ONLY: the BLE service that speaks this format lets any
 * connected phone set the network (no pairing); see core_thread.h.
 */

#ifndef THREAD_PROV_H
#define THREAD_PROV_H

#include <stdint.h>
#include "core_thread.h"

/* ---- GATT: 0000xxxx-8E22-4541-9D4C-21EDAE82ED19 (the SDK's base UUID) ---- */
#define THREAD_PROV_SVC_ID        0x5D00u   /* service "Thread Provisioning" */
#define THREAD_PROV_STATUS_ID     0x5D01u   /* read + notify: status (39 bytes) */
#define THREAD_PROV_DATASET_ID    0x5D02u   /* write only: dataset blob (58 bytes) */
#define THREAD_PROV_CONTROL_ID    0x5D03u   /* write: command (1 byte); read + notify: result (4 bytes) */

#define THREAD_PROV_VERSION       1u        /* byte 0 of the dataset and the status */
#define THREAD_PROV_DATASET_LEN   58u
#define THREAD_PROV_STATUS_LEN    39u
#define THREAD_PROV_RESULT_LEN    4u

/* Commands (write one byte to THREAD_PROV_CONTROL_ID). */
#define THREAD_PROV_CMD_APPLY          0x01u   /* make the staged dataset the network, persist, re-attach */
#define THREAD_PROV_CMD_FACTORY_RESET  0x02u   /* forget the stored network: back to config.json's */
#define THREAD_PROV_CMD_LEAVE          0x03u   /* stop Thread (detach); the stored network is kept */
#define THREAD_PROV_CMD_ATTACH         0x04u   /* start Thread again after LEAVE */
#define THREAD_PROV_CMD_CLEAR          0x05u   /* drop the staged dataset */

/* Result byte 0 ("op"): what the result answers. */
#define THREAD_PROV_OP_NONE       0x00u     /* nothing has happened yet */
#define THREAD_PROV_OP_DATASET    0x10u     /* a write to THREAD_PROV_DATASET_ID */
                                            /* else: the command byte */

/* Result codes (result byte 1). */
#define THREAD_PROV_OK                 0x00u
#define THREAD_PROV_ERR_LENGTH         0x01u   /* wrong number of bytes */
#define THREAD_PROV_ERR_VERSION        0x02u   /* dataset byte 0 is not 1 */
#define THREAD_PROV_ERR_RESERVED       0x03u   /* dataset flags byte is not 0 */
#define THREAD_PROV_ERR_CHANNEL        0x04u   /* channel outside 11-26 */
#define THREAD_PROV_ERR_PANID          0x05u   /* PAN ID 0xFFFF (broadcast) */
#define THREAD_PROV_ERR_NAME           0x06u   /* name length outside 1-16, or a NUL byte in it */
#define THREAD_PROV_ERR_NOTHING_STAGED 0x07u   /* APPLY without a valid staged dataset */
#define THREAD_PROV_ERR_COMMAND        0x08u   /* unknown command */
#define THREAD_PROV_ERR_DENIED         0x09u   /* core_thread_prov_allow_write() said no */
#define THREAD_PROV_ERR_STATE          0x0Au   /* Thread not started (core_thread_start) */
#define THREAD_PROV_ERR_FAILED         0x0Bu   /* OpenThread refused: result byte 3 = otError */

/* Status flags (status byte 2). */
#define THREAD_PROV_FLAG_PROVISIONED   0x01u   /* the network is a stored (provisioned) one, not config.json's */
#define THREAD_PROV_FLAG_STAGED        0x02u   /* a valid dataset is staged, waiting for APPLY */
#define THREAD_PROV_FLAG_ENABLED       0x04u   /* Thread is running (0: not started, or after LEAVE) */
#define THREAD_PROV_FLAG_WRITABLE      0x08u   /* core_thread_prov_allow_write() returns true now */

/** What the status characteristic reports (thread_prov_encode_status). */
typedef struct {
    uint8_t  role;                 /* core_thread_role_t */
    uint8_t  flags;                /* THREAD_PROV_FLAG_* */
    uint8_t  neighbors;
    int8_t   parent_rssi;          /* dBm, 127 = no parent */
    uint16_t rloc16;               /* 0xFFFE = not attached */
    uint32_t partition_id;         /* 0 = not attached */
    core_thread_dataset_t net;     /* channel, PAN ID, ext PAN ID, name (key ignored) */
} thread_prov_status_t;

/**
 * Check a dataset: channel 11-26, PAN ID not 0xFFFF, name 1-16 bytes.
 * @return THREAD_PROV_OK or a THREAD_PROV_ERR_* code.
 */
int thread_prov_check(const core_thread_dataset_t *ds);

/**
 * Decode and check a dataset blob (THREAD_PROV_DATASET_LEN bytes, version 1).
 * @return THREAD_PROV_OK and *out filled, or a THREAD_PROV_ERR_* code and
 *         *out zeroed.
 */
int thread_prov_decode_dataset(const uint8_t *buf, uint16_t len, core_thread_dataset_t *out);

/** Encode a dataset as a version-1 blob. @return THREAD_PROV_DATASET_LEN. */
uint16_t thread_prov_encode_dataset(const core_thread_dataset_t *ds, uint8_t out[THREAD_PROV_DATASET_LEN]);

/** Encode the status (version 1). @return THREAD_PROV_STATUS_LEN. */
uint16_t thread_prov_encode_status(const thread_prov_status_t *st, uint8_t out[THREAD_PROV_STATUS_LEN]);

/** The Active Timestamp a dataset stands for: its own, or 1 when it says 0. */
uint32_t thread_prov_timestamp(const core_thread_dataset_t *ds);

/* Boot precedence (thread_prov_boot_action). */
#define THREAD_PROV_BOOT_KEEP                     0   /* leave the stored dataset alone */
#define THREAD_PROV_BOOT_WRITE_CONFIG             1   /* make config.json's dataset active */
#define THREAD_PROV_BOOT_WRITE_CONFIG_CLEAR_MARK  2   /* the same, and forget the provisioned mark */

/**
 * What core_thread_start() does with the dataset OpenThread restored from flash.
 * @param marked            the provisioned mark is in the settings
 * @param stored_ok         a complete active dataset was restored
 * @param stored_is_config  ... and it equals config.json's
 * @return THREAD_PROV_BOOT_*: a provisioned network wins; without the mark,
 *         config.json's network wins (written only when it differs).
 */
int thread_prov_boot_action(int marked, int stored_ok, int stored_is_config);

/* ---- BLE service (thread_prov_ble.c, BLE + Thread builds) ---- */

/** Register the GATT service (from core_ble's service builder). No-op unless enabled in config.json. */
void thread_prov_ble_register(void);

/** Handle writes, refresh the status (from core_thread_process()). */
void thread_prov_ble_process(void);

/** OpenThread reported a state change: refresh the status soon. */
void thread_prov_ble_state_changed(void);

#endif /* THREAD_PROV_H */
