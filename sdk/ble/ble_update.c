/**
 * ble_update.c — Studio Link and the BLE firmware update (Core.ST.W5)
 *
 * Receives a new image over the Studio Update characteristic (0x5C02) into
 * free flash above the running app, checks it (CRC-32, per-board AES-CMAC),
 * writes the install marker and resets; the page-0 copier
 * (ble_update_boot.c) then installs it. Automatic in every BLE build: nothing
 * to call. Spec: docs/ble-update-protocol.md.
 *
 * The protocol state machine is the USB serial update's (su_core.c) built
 * with SU_STAGING; the install marker / key / copy logic is su_ota.c (linked
 * into SRAM by ble_update_boot.c); the CMAC is su_cmac.c. All three are
 * hardware-free and host-tested (make -C sdk/serial_update test).
 *
 * Contexts: the characteristic's write callback runs in the BLE host task
 * (ble_svc.c) — it only copies bytes into `rx`. Frames are handled from
 * ble_update_process(), called by core_ble_process() in the main loop, where
 * the flash work may block on radio windows (ble_flash.c).
 */

#include <stdint.h>
#include <stddef.h>

/* ---- The protocol core, staging build ---- */
#define SU_STAGING           1
#define SU_PROTOCOL_VERSION  3u
#define SU_MAX_PAYLOAD       2048u     /* chunk size */
#define SU_PROG_UNIT         16u       /* WBA quad-word, RM0493 §7.3.7 */
#include "../serial_update/su_ota.h"
#define SU_CRC32             su_ota_crc32   /* table-driven, ~3x su_crc32's speed */
#include "../serial_update/su_core.c"
#include "../serial_update/su_cmac.c"

#include "ble_flash.h"
#include "ble_studio_link.h"
#include "core_timing.h"
#include "ll_rcc.h"
#include "ll_iwdg.h"

/* ble_svc.c */
extern uint16_t ble_svc_add_service_id(const char *name, uint8_t num_chars, uint16_t uuid16);
extern uint16_t ble_svc_add_char_id(uint16_t svc_handle, const char *name, uint16_t uuid16,
                                    uint8_t access, uint8_t value_len,
                                    void (*on_write)(const uint8_t *data, uint16_t len, void *ctx),
                                    void *ctx);
extern int      ble_svc_set_value(uint16_t char_handle, const void *data, uint16_t len);
extern int      ble_svc_subscribed(uint16_t char_handle);
/* ble_app_glue.c */
extern volatile uint8_t  ble_connected;
extern volatile uint16_t ble_conn_handle;
extern volatile uint32_t ble_link_gen;
/* hw_rng.c */
extern void HW_RNG_Get(uint8_t n, uint32_t *val);
/* core_nvm.c: set by the NMI on an uncorrectable ECC error in page 124 */
extern volatile uint8_t _core_flash_ecc_ota;
/* stack */
extern uint8_t aci_gap_terminate(uint16_t conn_handle, uint8_t reason);
/* linker script */
extern const uint8_t __flash_image_end[];
/* Read through this .data word rather than as a literal in init_ops: the image
 * end moves with every program edit, and the project's code is linked last so
 * that such an edit changes only the last flash pages (Makefile, OBJECTS). The
 * word's initial value lives in the .data image, which is at the end too. */
static const uint8_t *volatile s_flash_image_end = __flash_image_end;

#define ACC_WRITE            0x08u     /* CORE_BLE_WRITE (+ write without response) */
#define ACC_NOTIFY           0x10u     /* CORE_BLE_NOTIFY */

#define FLASH_BASE_ADDR      0x08000000UL
#define DEV_ID_WBA5          0x492u    /* RM0493 §43.12.7 DBGMCU_IDCODER DEV_ID[11:0]: STM32WBA5xxx */
#define SRAM_END             0x20020000UL   /* 128 KB */
#define REBOOT_DISCONNECT_MS 300u
#define REBOOT_RESET_MS      500u

/* ============================================================
 * Flash (the app side): radio-safe windows through ble_flash
 * ============================================================ */

/* The ICACHE (on in BLE builds) also caches data reads of flash: invalidate
 * after an erase / program so a read-back sees the array (RM0493 §8.4.8;
 * the same as core_nvm.c). */
#define ICACHE_CR   (*(volatile uint32_t *)0x40030400UL)
#define ICACHE_SR   (*(volatile uint32_t *)0x40030404UL)
#define ICACHE_FCR  (*(volatile uint32_t *)0x4003040CUL)
static void icache_invalidate(void)
{
    if (!(ICACHE_CR & 1UL)) return;                 /* EN */
    ICACHE_FCR = (1UL << 1);                        /* CBSYENDF */
    ICACHE_CR |= (1UL << 1);                        /* CACHEINV */
    while (!(ICACHE_SR & (1UL << 1)))               /* BSYENDF */
        ;
}

static uint32_t s_app_end;          /* this image's end, offset from 0x08000000 */
static uint32_t s_stage_lo;         /* nothing below this is ever erased / programmed */

/* Only staging pages (above the running app, below page 124) and page 124
 * itself: defence in depth under su_core's own checks. */
static int writable(uint32_t off, uint32_t len)
{
    if (off >= SU_OTA_RES_OFF && off + len <= SU_OTA_RES_OFF + SU_OTA_PAGE) return 1;
    return off >= s_stage_lo && off + len <= SU_OTA_APP_LIMIT && off + len > off;
}

static int fl_erase(uint32_t page)
{
    if (!ble_flash_ready() || !writable(page * SU_OTA_PAGE, SU_OTA_PAGE)) return -1;
    ll_iwdg_refresh();
    int rc = ble_flash_erase_page(page);
    icache_invalidate();
    return rc;
}

static int fl_program(uint32_t off, const uint8_t *src, uint32_t len)
{
    if (!ble_flash_ready() || (off & 15u) || (len & 15u) || !writable(off, len)) return -1;
    uint32_t buf[64];                                /* 256 B, word aligned for FM_Write */
    int rc = 0;
    while (len && rc == 0) {
        uint32_t n = len < sizeof(buf) ? len : (uint32_t)sizeof(buf);
        uint8_t *b = (uint8_t *)buf;
        for (uint32_t i = 0; i < n; i++) b[i] = src[i];
        rc = ble_flash_program(FLASH_BASE_ADDR + off, buf, n / 4u);
        off += n;
        src += n;
        len -= n;
    }
    icache_invalidate();
    return rc;
}

/* su_ota_io_t for the app: page 124 and staging reads, ECC-checked. */
static int io_read(void *ctx, uint32_t off, void *dst, uint32_t len)
{
    (void)ctx;
    const volatile uint8_t *s = (const volatile uint8_t *)(FLASH_BASE_ADDR + off);
    uint8_t *d = (uint8_t *)dst;
    _core_flash_ecc_ota = 0;
    uint32_t i = 0;
    if (((off | (uint32_t)(uintptr_t)dst) & 3u) == 0u)       /* words where aligned */
        for (; i + 4u <= len; i += 4u)
            *(uint32_t *)(void *)(d + i) = *(const volatile uint32_t *)(const volatile void *)(s + i);
    for (; i < len; i++) d[i] = s[i];
    __asm volatile ("dsb 0xF\n\tisb 0xF" ::: "memory");
    return _core_flash_ecc_ota ? -1 : 0;
}
static int io_erase(void *ctx, uint32_t page) { (void)ctx; return fl_erase(page); }
static int io_program(void *ctx, uint32_t off, const uint32_t *src, uint32_t len)
{
    (void)ctx;
    return fl_program(off, (const uint8_t *)src, len);
}
static void io_feed(void *ctx) { (void)ctx; ll_iwdg_refresh(); }

static const su_ota_io_t s_io = { io_read, io_erase, io_program, io_feed, 0 };

/* ============================================================
 * Key and CMAC
 * ============================================================ */

static su_cmac_t s_img_mac;          /* the running image tag (holds the key schedule) */
static int       s_img_mac_on;

static int key_get(su_ota_key_t *k)
{
    return su_ota_key_read(&s_io, k);
}

static void le32_put(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

/* ============================================================
 * su_core operations
 * ============================================================ */

static uint16_t s_char;              /* Studio Update characteristic */

/* Replies: one line per notification, queued (the stack's notification queue
 * can be full; the line is offered again from the main loop). */
#define REPLY_SLOTS 4u
static char     s_reply[REPLY_SLOTS][64];
static uint8_t  s_reply_len[REPLY_SLOTS];
static uint32_t s_reply_head, s_reply_tail;

static void op_send(const char *line, uint32_t len)
{
    if (len > 63u) len = 63u;
    if (s_reply_head - s_reply_tail >= REPLY_SLOTS) s_reply_tail++;   /* drop the oldest */
    uint32_t i = s_reply_head % REPLY_SLOTS;
    for (uint32_t k = 0; k < len; k++) s_reply[i][k] = line[k];
    s_reply_len[i] = (uint8_t)len;
    s_reply_head++;
}

static uint32_t s_reboot_at;         /* core_millis() of SU DONE, 0 = none */
static uint8_t  s_rebooting, s_disconnected;

static void op_reboot(void)
{
    s_rebooting = 1;
    s_reboot_at = core_millis();
}

static int op_erase(uint32_t page) { return fl_erase(page); }
static int op_program(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    return fl_program(addr - FLASH_BASE_ADDR, buf, len);
}

static int op_stage(uint32_t size, uint32_t *at)
{
    return su_ota_fit(s_app_end, size, at);
}

static int op_key_info(uint32_t *key_id)
{
    su_ota_key_t k;
    if (!key_get(&k)) return 0;
    *key_id = k.key_id;
    su_wipe(&k, sizeof(k));
    return 1;
}

/* CRC-32 of [0x08000000, __flash_image_end): the .bin this image was built
 * as. Once per boot (~50 ms for 140 KB at 32 MHz), on the first QUERY. */
static uint32_t op_image_crc(void)
{
    static uint32_t crc;
    static uint8_t  have;
    if (!have) {
        crc = su_ota_crc32(0, (const uint8_t *)FLASH_BASE_ADDR, s_app_end);
        have = 1;
    }
    return crc;
}

static int op_challenge(uint8_t out[16])
{
    su_ota_key_t k;
    if (!key_get(&k)) return (int)SU_ERR_NOKEY;
    su_wipe(&k, sizeof(k));

    /* The link layer draws from the same RNG (hw_rng.c, possibly from its
     * interrupt): keep it out while we take each word (a few µs).
     * HW_RNG_Get reports a timeout as 0xDEADBEEF. */
    uint32_t w[4];
    for (int i = 0; i < 4; i++) {
        uint32_t primask;
        __asm volatile ("mrs %0, primask\n\tcpsid i" : "=r"(primask) :: "memory");
        HW_RNG_Get(1, &w[i]);
        __asm volatile ("msr primask, %0" :: "r"(primask) : "memory");
    }
    for (int i = 0; i < 4; i++)
        if (w[i] == 0xDEADBEEFUL) return (int)SU_ERR_RNG;
    if (w[0] == w[1] && w[1] == w[2] && w[2] == w[3]) return (int)SU_ERR_RNG;
    for (int i = 0; i < 4; i++) le32_put(out + 4 * i, w[i]);
    su_wipe(w, sizeof(w));
    return 0;
}

static int op_auth_check(const uint8_t n16[16], const uint8_t tag[16])
{
    su_ota_key_t k;
    uint8_t want[16];
    if (!key_get(&k)) return 0;
    su_tag_auth(k.key, n16, want);
    int ok = su_ct_equal(want, tag, 16);
    su_wipe(&k, sizeof(k));
    su_wipe(want, sizeof(want));
    return ok;
}

static void op_mac_start(uint32_t size, uint32_t crc)
{
    su_ota_key_t k;
    s_img_mac_on = key_get(&k);
    if (!s_img_mac_on) return;
    su_tag_image_start(&s_img_mac, k.key, size, crc);
    su_wipe(&k, sizeof(k));
}

static void op_mac_update(const uint8_t *p, uint32_t len)
{
    if (s_img_mac_on) su_cmac_update(&s_img_mac, p, len);
}

static int op_mac_check(const uint8_t n16[16], const uint8_t tag[16])
{
    if (!s_img_mac_on) return 0;
    s_img_mac_on = 0;
    uint8_t image_tag[16], want[16];
    su_cmac_final(&s_img_mac, image_tag);            /* also wipes s_img_mac */
    su_ota_key_t k;
    int ok = 0;
    if (key_get(&k)) {
        su_tag_end(k.key, n16, image_tag, want);
        ok = su_ct_equal(want, tag, 16);
        su_wipe(&k, sizeof(k));
    }
    su_wipe(want, sizeof(want));
    su_wipe(image_tag, sizeof(image_tag));
    return ok;
}

/* COPY-B records this image's size and CRC (the copier's reflash guard, docs
 * §8.2): the CRC READY reports, computed from flash once per boot (this
 * image can't change while it runs: writable() keeps every write above it).
 * KEEP records the pages the host sent as whole-page S frames. */
static int op_commit(uint32_t at, uint32_t size, uint32_t crc, const uint32_t keep[2])
{
    su_ota_commit_t c;
    c.stage    = at;
    c.size     = size;
    c.crc      = crc;
    c.old_size = s_app_end;
    c.old_crc  = op_image_crc();
    c.keep[0]  = keep[0];
    c.keep[1]  = keep[1];
    return su_ota_commit(&s_io, &c);
}

static void op_feed(void) { ll_iwdg_refresh(); }

static su_ops_t s_ops;

/* ============================================================
 * Receive buffer (BLE host task → main loop)
 * ============================================================ */

/* Two whole frames (2 x (16 + 2048)) plus slack, in this ring alone: the
 * spec's window of two unacknowledged DATA frames fits even if the main loop
 * hasn't run since. su_core keeps its own one-frame buffer (2128 B) on top.
 * Positions are free-running byte counts modulo RX_SIZE (2^32 bytes never
 * arrive in one boot). */
#define RX_SIZE (2u * (SU_HDR_LEN + SU_MAX_PAYLOAD) + 96u)   /* 4224 */
static uint8_t           s_rx[RX_SIZE];
static volatile uint32_t s_rx_head, s_rx_tail;
static volatile uint32_t s_rx_dropped;
static uint32_t          s_seen_gen;

static void on_write(const uint8_t *data, uint16_t len, void *ctx)
{
    (void)ctx;
    uint32_t h = s_rx_head;
    if (len > RX_SIZE - (h - s_rx_tail)) {           /* host ran ahead: drop the packet */
        s_rx_dropped++;
        return;
    }
    for (uint32_t i = 0; i < len; i++) s_rx[(h + i) % RX_SIZE] = data[i];
    __asm volatile ("dmb 0xF" ::: "memory");
    s_rx_head = h + len;
}

/* ============================================================
 * Studio Link service
 * ============================================================ */

static void (*s_attach)(uint16_t svc);
static uint8_t s_registered;

int ble_studio_link_attach(void (*add)(uint16_t svc))
{
    if (s_registered || !add) return -1;
    if (s_attach && s_attach != add) return -1;
    s_attach = add;
    return 0;
}

static void init_ops(void)
{
    s_app_end  = (uint32_t)(uintptr_t)s_flash_image_end - FLASH_BASE_ADDR;
    s_stage_lo = (s_app_end + SU_OTA_PAGE - 1u) & ~(SU_OTA_PAGE - 1u);

    s_ops.flash_base  = FLASH_BASE_ADDR;
    s_ops.flash_size  = 1024u * 1024u;
    s_ops.image_limit = su_ota_limit(s_app_end);
    s_ops.page_size   = SU_MAX_PAYLOAD;
    s_ops.dev_id      = DEV_ID_WBA5;
    s_ops.sram_start  = 0x20000000UL;
    s_ops.sram_end    = SRAM_END;
    s_ops.flash_read  = (const uint8_t *)FLASH_BASE_ADDR;
    s_ops.erase_page  = op_erase;
    s_ops.program     = op_program;
    s_ops.send        = op_send;
    s_ops.reboot      = op_reboot;
    s_ops.erase_size  = SU_OTA_PAGE;
    s_ops.stage       = op_stage;
    s_ops.key_info    = op_key_info;
    s_ops.image_crc   = op_image_crc;
    s_ops.challenge   = op_challenge;
    s_ops.auth_check  = op_auth_check;
    s_ops.mac_start   = op_mac_start;
    s_ops.mac_update  = op_mac_update;
    s_ops.mac_check   = op_mac_check;
    s_ops.commit      = op_commit;
    s_ops.run_size    = s_app_end;      /* S frames copy from [0, s_app_end) */
    s_ops.feed        = op_feed;
    su_core_init(&s_ops);
}

void ble_studio_link_register(void)
{
    /* 3 characteristic slots: 1 + 3*3 = 10 attribute records, enough for
     * scope + update (declaration, value, CCCD, user description each). */
    uint16_t svc = ble_svc_add_service_id("Studio Link", 3, STUDIO_LINK_SERVICE_ID);
    if (s_attach) s_attach(svc);
    s_char = ble_svc_add_char_id(svc, "Studio Update", STUDIO_LINK_UPDATE_ID,
                                 ACC_WRITE | ACC_NOTIFY, STUDIO_LINK_PACKET, on_write, 0);
    s_registered = 1;
    init_ops();
    s_seen_gen = ble_link_gen;
}

/* ============================================================
 * Main loop
 * ============================================================ */

static uint8_t s_busy;

void ble_update_process(void)
{
    if (!s_registered || s_busy) return;
    s_busy = 1;

    /* A new or lost link: authentication and partial frames go, the
     * transfer stays (docs §6). */
    if (ble_link_gen != s_seen_gen) {
        s_seen_gen = ble_link_gen;
        s_rx_tail = s_rx_head;
        s_reply_tail = s_reply_head;
        su_core_link_reset();
    }

    /* Feed whole received bytes to the protocol core while it has room. A
     * complete frame is handled inside su_core_rx (flash work included). */
    while (s_rx_tail != s_rx_head && !s_rebooting) {
        uint8_t  chunk[64];
        uint32_t t = s_rx_tail;
        uint32_t n = s_rx_head - t;
        uint32_t room = su_core_rx_space();
        if (n > sizeof(chunk)) n = sizeof(chunk);
        if (n > room) n = room;
        if (n == 0) break;
        for (uint32_t i = 0; i < n; i++) chunk[i] = s_rx[(t + i) % RX_SIZE];
        s_rx_tail = t + n;
        (void)su_core_rx(chunk, n);
    }
    if (s_rebooting) s_rx_tail = s_rx_head;

    /* Replies, oldest first, while a host listens. */
    while (s_reply_tail != s_reply_head) {
        if (!ble_connected || !ble_svc_subscribed(s_char)) { s_reply_tail = s_reply_head; break; }
        uint32_t i = s_reply_tail % REPLY_SLOTS;
        if (ble_svc_set_value(s_char, s_reply[i], s_reply_len[i]) != 0) break;   /* queue full */
        s_reply_tail++;
    }

    /* After SU DONE: let the reply go out, drop the link (the host sees it
     * at once rather than at its supervision timeout), reset into the copier. */
    if (s_rebooting) {
        uint32_t dt = core_millis() - s_reboot_at;
        if (dt >= REBOOT_DISCONNECT_MS && !s_disconnected && ble_connected) {
            s_disconnected = 1;
            (void)aci_gap_terminate(ble_conn_handle, 0x13);   /* remote user terminated */
        }
        if (dt >= REBOOT_RESET_MS) {
            __asm volatile ("dsb 0xF" ::: "memory");
            *(volatile uint32_t *)0xE000ED0CUL = 0x05FA0004UL;  /* AIRCR: SYSRESETREQ */
            for (;;)
                ;
        }
    }
    s_busy = 0;
}
