/**
 * thread_platform.c — OpenThread platform glue on ST's 802.15.4 link layer
 *
 * The radio itself (otPlatRadio*) lives in ST's link-layer library
 * (LinkLayer_Thread_lib.a, ot_inc headers); this file does what ST's
 * platform/system.c and misc.c do around it: bring the radio up, route its
 * completion callbacks into OpenThread, and supply the small otPlat*
 * functions OpenThread needs from the platform (reset, assert, entropy,
 * logging). Bare metal: no RTOS, everything runs from sequencer tasks.
 *
 * Adapted from STM32CubeWBA V1.6.1
 * Middlewares/ST/STM32_WPAN/thread/openthread/platform/{system,misc,entropy_renamed}.c,
 * which carry these notices:
 *
 *  Copyright (c) 2017, The OpenThread Authors. (system.c)
 *  Copyright (c) 2021 STMicroelectronics. (misc.c, entropy_renamed.c)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * tiles: bare metal (no RTOS), entropy straight from the hardware RNG,
 * a real system reset, our fatal-error path.
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Real OpenThread headers first: ST's link-layer copies (ot_inc) carry the
 * same include guards, so they are skipped where both exist, as in ST's
 * system.c. */
#include <openthread/instance.h>
#include <openthread/platform/radio.h>
#include <openthread/platform/misc.h>
#include <openthread/platform/entropy.h>
#include <openthread/platform/logging.h>

#include "platform.h"        /* LL: radio_init, radio_call_back_funcs_init, CSMA setters;
                                via common_types.h, LL_LOCK / LL_UNLOCK (g_ll_lock) */
#include "thread_app.h"

extern void HW_RNG_Get(uint8_t n, uint32_t *val);   /* sdk/wpan/hw_rng.c */

/* CSMA-CA parameters (ST's values, IEEE 802.15.4 defaults: macMinBE 3,
 * macMaxBE 5 is the standard; ST runs 6). */
#define CSMA_MAX_BE      6
#define CSMA_MIN_BE      3
#define CSMA_MAX_BACKOFF 4

/* ============================================================
 * Radio bring-up and callback routing
 * ============================================================ */

static void system_rx_done(otInstance *aInstance, otRadioFrame *aFrame, otError aError)
{
    /* ST: OpenThread does not handle an aborted receive (the LL reports one
     * when it interrupts continuous RX to transmit); drop it here. */
    if (aError != OT_ERROR_ABORT)
        otPlatRadioReceiveDone(aInstance, aFrame, aError);
}

static struct mac_cbk_dispatch_tbl s_ot_cbk_dispatch_tbl = {
    .mac_ed_scan_done = otPlatRadioEnergyScanDone,
    .mac_tx_done      = otPlatRadioTxDone,
    .mac_rx_done      = system_rx_done,
    .mac_tx_strtd     = otPlatRadioTxStarted,
    .mac_frm_updtd    = NULL,
};

void otSysInit(int argc, char *argv[])
{
    static uint8_t radio_up;
    (void)argc;
    (void)argv;

    LL_LOCK();
    if (!radio_up) {
        radio_init();
        set_max_csma_be(CSMA_MAX_BE);
        set_min_csma_be(CSMA_MIN_BE);
        set_max_csma_backoff(CSMA_MAX_BACKOFF);
        otPlatRadioSetPromiscuous(NULL, false);
        radio_up = 1;
    }
    thread_alarm_init();
    LL_UNLOCK();
}

/* Declared in the LL's platform.h; ST's applications call it after
 * otInstanceInitSingle(). */
void otDispatch_tbl_init(otInstance *sInstance)
{
    (void)sInstance;
    radio_call_back_funcs_init(&s_ot_cbk_dispatch_tbl);
}

/* ============================================================
 * Reset / assert
 * ============================================================ */

#define SCB_AIRCR  (*(volatile uint32_t *)0xE000ED0CUL)

void otPlatReset(otInstance *aInstance)
{
    (void)aInstance;
    /* A real system reset (ST: NVIC reset as its "pseudo reset"). The
     * settings that OpenThread wants back are in flash (thread_settings.c). */
    __asm volatile ("dsb 0xF" ::: "memory");
    SCB_AIRCR = (0x5FAUL << 16) | (1UL << 2);   /* VECTKEY | SYSRESETREQ */
    __asm volatile ("dsb 0xF" ::: "memory");
    for (;;)
        ;
}

otPlatResetReason otPlatGetResetReason(otInstance *aInstance)
{
    (void)aInstance;
    return OT_PLAT_RESET_REASON_POWER_ON;   /* as ST's misc.c */
}

void otPlatWakeHost(void)
{
}

void otPlatAssertFail(const char *aFilename, int aLineNumber)
{
    thread_app_fatal(THREAD_FATAL_OT_ASSERT, (uint32_t)aLineNumber, aFilename);
}

/* ============================================================
 * Entropy: the hardware RNG (sdk/wpan/hw_rng.c)
 *
 * mbedTLS seeds its CTR-DRBG from this (otPlatCryptoRandomInit, the OT
 * library's default), and OpenThread uses it for its own randomness.
 * HW_RNG_Get reports a timed-out read as 0xDEADBEEF: fail rather than hand
 * that out as entropy.
 * ============================================================ */

otError otPlatEntropyGet(uint8_t *aOutput, uint16_t aOutputLength)
{
    if (aOutput == NULL)
        return OT_ERROR_INVALID_ARGS;

    while (aOutputLength > 0u) {
        uint32_t w = 0;
        HW_RNG_Get(1, &w);
        if (w == 0xDEADBEEFUL)
            return OT_ERROR_FAILED;
        for (uint8_t i = 0; i < 4u && aOutputLength > 0u; i++, aOutputLength--) {
            *aOutput++ = (uint8_t)w;
            w >>= 8;
        }
    }
    return OT_ERROR_NONE;
}

/* ============================================================
 * Logging: off (OPENTHREAD_CONFIG_LOG_OUTPUT_APP). Defined here so the
 * library's NCP object, which also defines it, is not pulled in.
 * ============================================================ */

void otPlatLog(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, ...)
{
    (void)aLogLevel;
    (void)aLogRegion;
    (void)aFormat;
}

/* ============================================================
 * Link-layer debug GPIO hooks: unused (BLE builds have the same stubs in
 * sdk/ble/ble_app_glue.c)
 * ============================================================ */

void LINKLAYER_DEBUG_SIGNAL_SET(uint8_t signal);
void LINKLAYER_DEBUG_SIGNAL_RESET(uint8_t signal);
void LINKLAYER_DEBUG_SIGNAL_TOGGLE(uint8_t signal);
void LINKLAYER_DEBUG_SIGNAL_SET(uint8_t signal)    { (void)signal; }
void LINKLAYER_DEBUG_SIGNAL_RESET(uint8_t signal)  { (void)signal; }
void LINKLAYER_DEBUG_SIGNAL_TOGGLE(uint8_t signal) { (void)signal; }

/* ============================================================
 * Runtime symbols the prebuilt (Arm Compiler) libraries reference
 * ============================================================ */

/* ST: "__2snprintf symbol missing in aeabi OT libs, add it on platform". */
int __2snprintf(char *s, size_t n, const char *format, ...);
int __2snprintf(char *s, size_t n, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int r = vsnprintf(s, n, format, args);
    va_end(args);
    return r;
}

/* assert() inside the OpenThread library (mac_frame) */
void __aeabi_assert(const char *expr, const char *file, int line);
void __aeabi_assert(const char *expr, const char *file, int line)
{
    (void)expr;
    thread_app_fatal(THREAD_FATAL_LIB_ASSERT, (uint32_t)line, file);
}
