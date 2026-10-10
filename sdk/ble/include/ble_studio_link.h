/**
 * ble_studio_link.h — the Studio Link GATT service (SDK-internal)
 *
 * Every Core.ST.W5 build with BLE on registers Studio Link (0x5C00) after the
 * application's own services, with the Studio Update characteristic (0x5C02,
 * the BLE firmware update: sdk/ble/ble_update.c, docs/ble-update-protocol.md).
 * SDK modules add their own characteristics to it through
 * ble_studio_link_attach(): core_scope's Studio Scope (0x5C01) does.
 *
 * Not an application API (that is core_ble.h).
 */

#ifndef BLE_STUDIO_LINK_H
#define BLE_STUDIO_LINK_H

#include <stdint.h>

#define STUDIO_LINK_SERVICE_ID   0x5C00u
#define STUDIO_LINK_SCOPE_ID     0x5C01u   /* core_scope.c */
#define STUDIO_LINK_UPDATE_ID    0x5C02u   /* ble_update.c */
/* Notification / write payload. core_ble does not report the negotiated ATT
 * MTU; every desktop and phone central negotiates at least 185. */
#define STUDIO_LINK_PACKET       180u

/**
 * Add a characteristic builder to Studio Link: `add(svc)` runs while the GATT
 * table is built, after the application's services, and before the update
 * characteristic. Call before core_ble_init(). 0, or -1 (slot taken / too late).
 */
int ble_studio_link_attach(void (*add)(uint16_t svc));

/** core_ble: register the service (from its service builder). */
void ble_studio_link_register(void);

/** core_ble: run the update receiver from the main loop (core_ble_process). */
void ble_update_process(void);

#endif /* BLE_STUDIO_LINK_H */
