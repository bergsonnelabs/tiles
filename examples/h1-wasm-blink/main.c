/*
 * Generated firmware harness for a Compiled-mode Studio flash.
 * main.c assembled by apps/web/src/lib/wasmHarness.ts; do not
 * edit by hand — the web app regenerates on every flash.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "core.h"
#include "core_led.h"
#include "core_usb.h"
#include "hal_usb_cdc.h"

#include "wasm_export.h"
#include "studio_natives.h"
#include "studio_natives_project.h"  /* coregen: pad / pwm / adc / dac */

/* Precompiled Studio DSL program as a Wasm binary. */
static const uint8_t g_wasm_blob_flash[] = {
    0x00, 0x61, 0x73, 0x6d, 0x01, 0x00, 0x00, 0x00, 0x01, 0x09, 0x02, 0x60,
    0x02, 0x7f, 0x7f, 0x00, 0x60, 0x00, 0x00, 0x02, 0x1a, 0x01, 0x03, 0x65,
    0x6e, 0x76, 0x12, 0x63, 0x6f, 0x72, 0x65, 0x5f, 0x6c, 0x65, 0x64, 0x5f,
    0x68, 0x65, 0x61, 0x72, 0x74, 0x62, 0x65, 0x61, 0x74, 0x00, 0x00, 0x03,
    0x03, 0x02, 0x01, 0x01, 0x07, 0x1e, 0x02, 0x0c, 0x73, 0x74, 0x75, 0x64,
    0x69, 0x6f, 0x5f, 0x73, 0x74, 0x61, 0x72, 0x74, 0x00, 0x01, 0x0b, 0x73,
    0x74, 0x75, 0x64, 0x69, 0x6f, 0x5f, 0x6c, 0x6f, 0x6f, 0x70, 0x00, 0x02,
    0x0a, 0x0e, 0x02, 0x09, 0x00, 0x41, 0xe8, 0x07, 0x41, 0x02, 0x10, 0x00,
    0x0b, 0x02, 0x00, 0x0b,
};
static const size_t g_wasm_blob_flash_len = sizeof(g_wasm_blob_flash);

/* WAMR's loader rewrites each import/export name one byte backward
 * over its leb128 length prefix to null-terminate it in place. That
 * requires the module buffer to be writable — flash is not. Also,
 * the CDC hot-swap path needs to drop new .wasm blobs in here, so
 * the buffer is sized for the MAX module we'll accept, not just the
 * initial embedded one. */
#define MAX_MODULE_BYTES (32768)
static uint8_t g_wasm_blob[MAX_MODULE_BYTES] __attribute__((aligned(4)));
static size_t g_wasm_blob_len = 0;

/* WAMR's pool allocator. Sidesteps newlib's malloc (nosys _sbrk
 * fails silently and hangs wasm_runtime_full_init otherwise). */
#define WAMR_HEAP_BYTES (32768)
__attribute__((aligned(8)))
static uint8_t g_wamr_heap[WAMR_HEAP_BYTES];

/* wasm_c_api dead-stub — wasm_runtime_common.c compiles in a
 * dispatch path that references wasm_trap_delete even with
 * WASM_ENABLE_WASM_C_API=0. Provide a no-op to keep the linker
 * happy. Never called in the interpreter path. */
typedef struct wasm_trap_t wasm_trap_t;
void wasm_trap_delete(wasm_trap_t *trap) { (void)trap; }

/* Current Wasm instance. Set by load_module, cleared by unload.
 * Accessed from main() and the hot-swap path. No locking — single
 * flow of control; CDC RX just feeds bytes into a frame parser. */
static wasm_module_t      g_module;
static wasm_module_inst_t g_inst;
static wasm_exec_env_t    g_exec;
static wasm_function_inst_t g_fn_start;
static wasm_function_inst_t g_fn_loop;

static void usb_warmup(void)
{
    core_delay_ms(1500);
}

static void fatal(const char *tag, const char *detail)
{
    core_usb_printf("studio: %s - %s\r\n", tag, detail);
    while (1) {
        core_led_blink(3, 100, 100);
        core_delay_ms(500);
    }
}

/* Load + instantiate whatever is currently in g_wasm_blob[0 .. g_wasm_blob_len].
 * On success: g_module/g_inst/g_exec/g_fn_start/g_fn_loop are live and
 * studio_start has run. On failure: nothing survives and the caller
 * should fatal() — we try to clean up partial state along the way. */
static bool load_and_start(char *err_buf, size_t err_buf_size)
{
    g_module = wasm_runtime_load(g_wasm_blob, g_wasm_blob_len,
                                 err_buf, err_buf_size);
    if (!g_module) return false;

    g_inst = wasm_runtime_instantiate(
        g_module, 4096, 4096,
        err_buf, err_buf_size);
    if (!g_inst) {
        wasm_runtime_unload(g_module);
        g_module = NULL;
        return false;
    }

    g_exec = wasm_runtime_create_exec_env(g_inst, 4096);
    if (!g_exec) {
        wasm_runtime_deinstantiate(g_inst);
        wasm_runtime_unload(g_module);
        g_inst = NULL; g_module = NULL;
        snprintf(err_buf, err_buf_size, "create_exec_env returned NULL");
        return false;
    }

    g_fn_start = wasm_runtime_lookup_function(g_inst, "studio_start");
    g_fn_loop  = wasm_runtime_lookup_function(g_inst, "studio_loop");
    if (!g_fn_start || !g_fn_loop) {
        wasm_runtime_destroy_exec_env(g_exec);
        wasm_runtime_deinstantiate(g_inst);
        wasm_runtime_unload(g_module);
        g_exec = NULL; g_inst = NULL; g_module = NULL;
        snprintf(err_buf, err_buf_size, "studio_start / studio_loop missing");
        return false;
    }

    if (!wasm_runtime_call_wasm(g_exec, g_fn_start, 0, NULL)) {
        const char *ex = wasm_runtime_get_exception(g_inst);
        wasm_runtime_destroy_exec_env(g_exec);
        wasm_runtime_deinstantiate(g_inst);
        wasm_runtime_unload(g_module);
        g_exec = NULL; g_inst = NULL; g_module = NULL;
        snprintf(err_buf, err_buf_size, "studio_start: %s", ex ? ex : "(no message)");
        return false;
    }
    return true;
}

static void unload_current(void)
{
    if (g_exec)   wasm_runtime_destroy_exec_env(g_exec);
    if (g_inst)   wasm_runtime_deinstantiate(g_inst);
    if (g_module) wasm_runtime_unload(g_module);
    g_exec = NULL; g_inst = NULL; g_module = NULL;
    g_fn_start = NULL; g_fn_loop = NULL;
}

/* ---------------------------------------------------------------------
 * CDC bytecode hot-swap protocol
 *
 *   magic    4 bytes   'T' 'S' 'W' 'M'
 *   version  1 byte    0x01
 *   flags    1 byte    reserved (bit 0 = persist-to-flash; future)
 *   reserved 2 bytes   0x00 0x00
 *   length   4 bytes   u32 little-endian (payload size)
 *   payload  N bytes   .wasm binary
 *
 * The parser is resync-able: any byte that isn't the next expected
 * piece resets the state machine, so an in-progress core_usb_printf
 * TX (or stray console bytes) can't corrupt a later frame.
 * ------------------------------------------------------------------ */

#define FRAME_HEADER_BYTES 12
static const uint8_t FRAME_MAGIC[4] = { 'T', 'S', 'W', 'M' };

static size_t   g_rx_pos = 0;        /* bytes consumed into the current frame */
static uint32_t g_rx_expected_len = 0;

static void frame_reset(void)
{
    g_rx_pos = 0;
    g_rx_expected_len = 0;
}

/* Feed one byte into the state machine. Returns true iff a complete
 * frame has just landed (payload is g_wasm_blob[0..g_rx_expected_len]). */
static bool frame_feed_byte(uint8_t b)
{
    if (g_rx_pos < 4) {
        if (b == FRAME_MAGIC[g_rx_pos]) { g_rx_pos++; return false; }
        /* Misaligned: the byte might itself be the start of magic. */
        g_rx_pos = (b == FRAME_MAGIC[0]) ? 1 : 0;
        return false;
    }
    if (g_rx_pos == 4) {  /* version */
        if (b != 0x01) { frame_reset(); return false; }
        g_rx_pos++; return false;
    }
    if (g_rx_pos < 8) {   /* flags + 2 reserved */
        g_rx_pos++; return false;
    }
    if (g_rx_pos < 12) {  /* length, little-endian */
        size_t shift = (g_rx_pos - 8) * 8;
        g_rx_expected_len |= (uint32_t)b << shift;
        g_rx_pos++;
        if (g_rx_pos == 12) {
            if (g_rx_expected_len == 0
                || g_rx_expected_len > MAX_MODULE_BYTES) {
                core_usb_printf(
                    "studio: rejecting frame (%lu bytes; max %u)\r\n",
                    (unsigned long)g_rx_expected_len,
                    (unsigned)MAX_MODULE_BYTES);
                frame_reset();
            }
        }
        return false;
    }
    /* Payload */
    size_t payload_idx = g_rx_pos - FRAME_HEADER_BYTES;
    g_wasm_blob[payload_idx] = b;
    g_rx_pos++;
    if (payload_idx + 1 == g_rx_expected_len) {
        g_wasm_blob_len = g_rx_expected_len;
        return true;
    }
    return false;
}

/* Drain bytes pending on CDC RX. Returns true if a complete frame
 * has landed (caller should swap the running module). */
static bool frame_poll(void)
{
    uint8_t b;
    while (hal_usb_cdc_rx_try(&b)) {
        if (frame_feed_byte(b)) return true;
    }
    return false;
}

int main(void)
{
    core_init();
    core_usb_init();
    core_led_init();
    usb_warmup();

    core_usb_printf("studio: booting (WAMR-2.4.4, %u-byte boot module)\r\n",
                    (unsigned)g_wasm_blob_flash_len);

    /* Seed the RAM buffer with the flash-resident boot module. */
    memcpy(g_wasm_blob, g_wasm_blob_flash, g_wasm_blob_flash_len);
    g_wasm_blob_len = g_wasm_blob_flash_len;

    RuntimeInitArgs init_args = { 0 };
    init_args.mem_alloc_type = Alloc_With_Pool;
    init_args.mem_alloc_option.pool.heap_buf = g_wamr_heap;
    init_args.mem_alloc_option.pool.heap_size = sizeof(g_wamr_heap);
    if (!wasm_runtime_full_init(&init_args)) {
        fatal("wasm_runtime_full_init", "returned false");
    }

    if (!wasm_runtime_register_natives(
            "env", g_studio_natives, g_studio_natives_count)) {
        fatal("wasm_runtime_register_natives (static)", "returned false");
    }
    /* Per-project table: pad / pwm / adc / dac adapters that need
     * coregen-emitted state in scope. Generated alongside core_init.c. */
    if (!wasm_runtime_register_natives(
            "env", g_studio_natives_project, g_studio_natives_project_count)) {
        fatal("wasm_runtime_register_natives (project)", "returned false");
    }

    char err_buf[96];
    if (!load_and_start(err_buf, sizeof err_buf)) {
        fatal("boot module", err_buf);
    }

    core_usb_printf("studio: running (push TSWM frames over CDC to hot-swap)\r\n");

    uint32_t tick = 0;
    while (1) {
        if (!wasm_runtime_call_wasm(g_exec, g_fn_loop, 0, NULL)) {
            const char *ex = wasm_runtime_get_exception(g_inst);
            core_usb_printf("studio: studio_loop exception: %s\r\n",
                            ex ? ex : "(none)");
            /* Tear down the failed instance so a push-new-DSL
             * recovers without a reboot. If no new DSL arrives
             * we idle here until CDC delivers one. */
            unload_current();
        }

        if ((++tick % 2) == 0 && g_inst) {
            core_usb_printf("  tick=%-6lu\r\n", (unsigned long)tick);
        }

        /* Drain CDC RX. If a complete hot-swap frame landed,
         * teardown and reload the new module. */
        if (frame_poll()) {
            core_usb_printf("studio: hot-swap frame (%lu bytes)\r\n",
                            (unsigned long)g_wasm_blob_len);
            unload_current();
            if (!load_and_start(err_buf, sizeof err_buf)) {
                core_usb_printf("studio: hot-swap failed: %s\r\n",
                                err_buf);
                /* Keep spinning — next push might be valid. */
            } else {
                core_usb_printf("studio: running new module\r\n");
                tick = 0;
            }
            frame_reset();
        }

        /* When no instance is live (load failed, exception above,
         * etc.), back off so the CPU isn't in a tight spin. */
        if (!g_inst) {
            core_delay_ms(100);
        }
    }
}
