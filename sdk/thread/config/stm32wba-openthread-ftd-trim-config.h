/**
 * stm32wba-openthread-ftd-trim-config.h — OpenThread configuration of the
 * trimmed FTD library that BLE + Thread builds link
 *
 * sdk/thread/lib/stm32wba_ot_ftd_trim_lib.a is OpenThread rebuilt from the
 * source ST ships in STM32CubeWBA V1.6.1 (Middlewares/ST/STM32_WPAN/thread/
 * openthread/stack, OpenThread 4a0d6b7, the same code as ST's prebuilt
 * stm32wba_ot_ftd_lib.a) by tools/build_openthread_trim.py, with ST's FTD
 * configuration (stm32wba-openthread-ftd-config.h, included below) minus what
 * the SDK does not use, so that BLE and Thread fit in the WBA55's 128 KB of
 * SRAM with room for an application. Thread-only builds keep ST's library.
 *
 * Kept: everything a full Thread device (router / leader capable) runs: MLE
 * router, network data leader, address resolution, MPL, DHCPv6 client,
 * SLAAC, Thread 1.2 DUA / MLR, CoAP (also used internally by TMF), the
 * network diagnostic client and server, MAC filter, channel mask, child
 * supervision, the microsecond timer.
 *
 * Removed (none of it is reachable through core_thread.h; all of it is
 * listed in core_thread.h's @studio unsupported notes or needs the PKA,
 * which the SDK does not drive):
 *   commissioner, joiner and CoAP secure (the DTLS transport: no
 *     SecureTransport, no ECJPAKE, and no DTLS-sized mbedTLS heap),
 *   SRP client (and its buffers / auto start / domain name API), DNS client
 *     (and NAT64, service discovery), ECDSA, TCP,
 *   ping sender, mesh diagnostics, link metrics (initiator and subject),
 *   CSL receiver (an FTD keeps its receiver on), reference-device test hooks.
 *
 * Sized down:
 *   message buffers 128 -> 64 (ST's own value for its static-memory builds;
 *     OpenThread's default is 44). 128 bytes each: -8 KB.
 *   OpenThread's internal heap (mbedTLS allocations) 12544 B -> 4096 B. With
 *     no DTLS and no ECDSA, mbedTLS allocates only message-digest and cipher
 *     contexts: HMAC-SHA256 (MLE / MAC key derivation, KEK) takes ST's
 *     hardware SHA-256 context (608 B with ST's sha256_alt.h, measured) +
 *     128 B of HMAC pads, AES-CMAC (PSKc) an AES context (144 B) + its CMAC
 *     state. 4096 B holds five HMAC contexts at once; OpenThread's own default
 *     for this case is 384 B, sized for mbedTLS's smaller software context.
 *
 * Any of it can come back by editing this file and re-running
 * tools/build_openthread_trim.py; the core_thread.c glue and every BLE +
 * Thread build compile against this same file (Makefile OPENTHREAD_CONFIG_FILE).
 */

#ifndef STM32WBA_OPENTHREAD_FTD_TRIM_CONFIG_H
#define STM32WBA_OPENTHREAD_FTD_TRIM_CONFIG_H

#include "stm32wba-openthread-ftd-config.h"

/* ---- Features off ---- */
#undef OPENTHREAD_CONFIG_COMMISSIONER_ENABLE
#define OPENTHREAD_CONFIG_COMMISSIONER_ENABLE 0
#undef OPENTHREAD_CONFIG_JOINER_ENABLE
#define OPENTHREAD_CONFIG_JOINER_ENABLE 0
#undef OPENTHREAD_CONFIG_COAP_SECURE_API_ENABLE
#define OPENTHREAD_CONFIG_COAP_SECURE_API_ENABLE 0
#undef OPENTHREAD_CONFIG_SRP_CLIENT_ENABLE
#define OPENTHREAD_CONFIG_SRP_CLIENT_ENABLE 0
#undef OPENTHREAD_CONFIG_SRP_CLIENT_BUFFERS_ENABLE
#define OPENTHREAD_CONFIG_SRP_CLIENT_BUFFERS_ENABLE 0
#undef OPENTHREAD_CONFIG_SRP_CLIENT_AUTO_START_API_ENABLE
#define OPENTHREAD_CONFIG_SRP_CLIENT_AUTO_START_API_ENABLE 0
#undef OPENTHREAD_CONFIG_SRP_CLIENT_DOMAIN_NAME_API_ENABLE
#define OPENTHREAD_CONFIG_SRP_CLIENT_DOMAIN_NAME_API_ENABLE 0
#undef OPENTHREAD_CONFIG_DNS_CLIENT_ENABLE
#define OPENTHREAD_CONFIG_DNS_CLIENT_ENABLE 0
#undef OPENTHREAD_CONFIG_DNS_CLIENT_NAT64_ENABLE
#define OPENTHREAD_CONFIG_DNS_CLIENT_NAT64_ENABLE 0
#undef OPENTHREAD_CONFIG_DNS_CLIENT_DEFAULT_NAT64_ALLOWED
#define OPENTHREAD_CONFIG_DNS_CLIENT_DEFAULT_NAT64_ALLOWED 0
#undef OPENTHREAD_CONFIG_DNS_CLIENT_SERVICE_DISCOVERY_ENABLE
#define OPENTHREAD_CONFIG_DNS_CLIENT_SERVICE_DISCOVERY_ENABLE 0
#undef OPENTHREAD_CONFIG_ECDSA_ENABLE
#define OPENTHREAD_CONFIG_ECDSA_ENABLE 0
#undef OPENTHREAD_CONFIG_TCP_ENABLE
#define OPENTHREAD_CONFIG_TCP_ENABLE 0
#undef OPENTHREAD_CONFIG_PING_SENDER_ENABLE
#define OPENTHREAD_CONFIG_PING_SENDER_ENABLE 0
#undef OPENTHREAD_CONFIG_MESH_DIAG_ENABLE
#define OPENTHREAD_CONFIG_MESH_DIAG_ENABLE 0
#undef OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE
#define OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE 0
#undef OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
#define OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE 0
#undef OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
#define OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE 0
#undef OPENTHREAD_CONFIG_REFERENCE_DEVICE_ENABLE
#define OPENTHREAD_CONFIG_REFERENCE_DEVICE_ENABLE 0

/* ---- Sizes ---- */
#undef OPENTHREAD_CONFIG_NUM_MESSAGE_BUFFERS
#define OPENTHREAD_CONFIG_NUM_MESSAGE_BUFFERS 64
#define OPENTHREAD_CONFIG_HEAP_INTERNAL_SIZE_NO_DTLS 4096

#undef OPENTHREAD_CONFIG_PLATFORM_INFO
#define OPENTHREAD_CONFIG_PLATFORM_INFO "STM32WBA V1.3.0 Thread_FTD tiles-trim"

#endif /* STM32WBA_OPENTHREAD_FTD_TRIM_CONFIG_H */
