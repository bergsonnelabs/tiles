/**
 * ota_sim.c — the BLE update (Core.ST.W5) on a simulated flash, no hardware
 *
 * Builds the receiver exactly as sdk/ble/ble_update.c does (su_core.c with
 * SU_STAGING, su_cmac.c) plus the install logic sdk/ble/ble_update_boot.c runs
 * (su_ota.c), against a 1 MB WBA55 flash model: 8 KB pages, 16-byte quad-words
 * that program once between erases, and power cuts that tear the operation in
 * progress (the torn quad-word / page then reads with an ECC error, as the
 * copier's NMI path sees it). The "host" side builds frames, splits them into
 * 180-byte BLE writes and computes its tags with its own one-shot CMAC.
 *
 * Covers: CMAC vectors (FIPS 197, RFC 4493, the spec's), a full transfer and
 * install, link drop + resume, partial frames, bad / replayed tags, wrong key,
 * no key, too-big images, bad vectors / device, abort, power cuts at every
 * flash operation of the copy (page 0 changed and unchanged), the commit and
 * (sampled) the transfer, a Studio SWD flash that clears the marker and one
 * that doesn't, the reflash guard (START, resumed copies, v1 markers), delta
 * S frames (runs, refusals, window, wrong base), and log compaction. What it doesn't cover: the BLE glue in ble_update.c (receive
 * buffer, reply queue, link generation) and the register-level flash /
 * NMI code in ble_update_boot.c — those need the bench.
 *
 *   ota_sim            run everything; exit 0 = all passed
 *   ota_sim --vectors  print the spec's test vectors (docs/ble-update-protocol.md §5)
 */

#define SU_STAGING           1
#define SU_PROTOCOL_VERSION  3u
#define SU_MAX_PAYLOAD       2048u
#define SU_PROG_UNIT         16u
#include "../su_core.c"
#include "../su_cmac.c"
#include "../su_ota.c"

#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ============================================================
 * Flash model
 * ============================================================ */

#define FL_SIZE   (1024u * 1024u)
#define FL_QWS    (FL_SIZE / 16u)
#define FL_BASE   0x08000000u

static uint8_t  fl[FL_SIZE];
static uint8_t  qw_prog[FL_QWS];       /* programmed since its page was erased */
static uint8_t  qw_torn[FL_QWS];       /* reads with an uncorrectable ECC error */
static long     ops;                   /* erase + quad-word programs so far */
static long     page0_erases;          /* erases of page 0 so far */
static long     page_erases[128];      /* erases per page so far */
static long     cut_at = -1;           /* power fails during this op */
static jmp_buf  cut_env;
static uint32_t rnd_state = 12345;

static uint32_t rnd(void)
{
    rnd_state ^= rnd_state << 13;
    rnd_state ^= rnd_state >> 17;
    rnd_state ^= rnd_state << 5;
    return rnd_state;
}

static void power_cut_maybe(void)
{
    if (cut_at >= 0 && ops == cut_at) {
        cut_at = -1;
        longjmp(cut_env, 1);
    }
}

static int fl_erase(uint32_t page)
{
    if (page >= FL_SIZE / SU_OTA_PAGE) return -1;
    ops++;
    if (page == 0) page0_erases++;
    page_erases[page]++;
    if (cut_at >= 0 && ops == cut_at) {
        /* Half an erase: garbage that reads with ECC errors. */
        for (uint32_t i = 0; i < SU_OTA_PAGE; i++) fl[page * SU_OTA_PAGE + i] = (uint8_t)rnd();
        for (uint32_t q = 0; q < SU_OTA_PAGE / 16u; q++) {
            qw_torn[page * (SU_OTA_PAGE / 16u) + q] = 1;
            qw_prog[page * (SU_OTA_PAGE / 16u) + q] = 1;
        }
        power_cut_maybe();
    }
    memset(fl + page * SU_OTA_PAGE, 0xFF, SU_OTA_PAGE);
    memset(qw_prog + page * (SU_OTA_PAGE / 16u), 0, SU_OTA_PAGE / 16u);
    memset(qw_torn + page * (SU_OTA_PAGE / 16u), 0, SU_OTA_PAGE / 16u);
    return 0;
}

static int fl_program(uint32_t off, const uint8_t *src, uint32_t len)
{
    if ((off & 15u) || (len & 15u) || off + len > FL_SIZE) return -1;
    for (uint32_t i = 0; i < len; i += 16u) {
        uint32_t q = (off + i) / 16u;
        if (qw_prog[q] || qw_torn[q]) return -1;      /* PROGERR: not erased */
        ops++;
        if (cut_at >= 0 && ops == cut_at) {
            for (uint32_t k = 0; k < 16u; k++) fl[off + i + k] = (uint8_t)(src[i + k] | (uint8_t)rnd());
            qw_torn[q] = qw_prog[q] = 1;
            power_cut_maybe();
        }
        memcpy(fl + off + i, src + i, 16);
        qw_prog[q] = 1;
    }
    return 0;
}

static int fl_read(uint32_t off, void *dst, uint32_t len)
{
    if (off + len > FL_SIZE) return -1;
    memcpy(dst, fl + off, len);
    for (uint32_t q = off / 16u; q * 16u < off + len; q++)
        if (qw_torn[q]) return -1;
    return 0;
}

/* su_ota_io_t on the model (what both the app and the copier see). */
static int  io_read(void *c, uint32_t off, void *dst, uint32_t len) { (void)c; return fl_read(off, dst, len); }
static int  io_erase(void *c, uint32_t page) { (void)c; return fl_erase(page); }
static int  io_program(void *c, uint32_t off, const uint32_t *src, uint32_t len) { (void)c; return fl_program(off, (const uint8_t *)src, len); }
static void io_feed(void *c) { (void)c; }
static const su_ota_io_t sim_io = { io_read, io_erase, io_program, io_feed, 0 };

/* ============================================================
 * Test bookkeeping
 * ============================================================ */

static int failures;

static void check(int cond, const char *fmt, ...)
{
    if (cond) return;
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "FAIL  ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
    failures++;
}

static int failures_seen;
static void ok(const char *what)
{
    printf("  %s  %s\n", failures > failures_seen ? "FAIL" : "ok  ", what);
    failures_seen = failures;
}

/* ============================================================
 * Images, key, Studio's SWD flash
 * ============================================================ */

static const uint8_t KEY[16]   = { 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f };
static const uint8_t WRONG[16] = { 0xff,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f };
#define KEY_ID 0x5a17c0deu

static uint8_t *make_image(uint32_t size, uint32_t seed)
{
    uint8_t *p = malloc(size);
    uint32_t s = seed;
    for (uint32_t i = 0; i < size; i++) { s = s * 1103515245u + 12345u; p[i] = (uint8_t)(s >> 16); }
    /* A W5 vector table: SP at the top of SRAM, Thumb reset vector inside. */
    uint32_t sp = 0x20020000u, rv = FL_BASE + 0x161u;
    memcpy(p, &sp, 4);
    memcpy(p + 4, &rv, 4);
    return p;
}

/* The key record, docs §7.2. */
static void key_record(uint32_t key_id, const uint8_t key[16], uint32_t rec[8])
{
    rec[0] = SU_OTA_KEY_MAGIC;
    rec[1] = key_id;
    memcpy(&rec[4], key, 16);
    uint32_t c = su_ota_crc32(0, (const uint8_t *)&rec[0], 8);
    rec[2] = su_ota_crc32(c, (const uint8_t *)&rec[4], 16);
    rec[3] = 0xFFFFFFFFu;
}

/* What Studio's SWD flasher does (docs §7.2): keep or make the key record,
 * erase page 124, write the record back, write the image below page 124. */
static void studio_swd_flash(const uint8_t *img, uint32_t size, int provision, int clear_marker)
{
    uint32_t rec[8];
    int have = 0;
    if (!provision) {
        su_ota_key_t k;
        have = su_ota_key_read(&sim_io, &k);
        if (have) key_record(k.key_id, k.key, rec);
    }
    if (!have) key_record(KEY_ID, KEY, rec);
    if (clear_marker || provision) {
        fl_erase(SU_OTA_RES_PAGE);
        fl_program(SU_OTA_RES_OFF, (const uint8_t *)rec, 32);
    }
    uint32_t pages = (size + SU_OTA_PAGE - 1u) / SU_OTA_PAGE;
    for (uint32_t p = 0; p < pages; p++) fl_erase(p);
    uint8_t qw[16];
    for (uint32_t o = 0; o < size; o += 16u) {
        memset(qw, 0xFF, 16);
        memcpy(qw, img + o, size - o < 16u ? size - o : 16u);
        fl_program(o, qw, 16);
    }
}

/* A fresh board: everything erased, then the first SWD flash. Pages 125-127
 * get a pattern standing in for core_nvm and the bonds. */
static void fresh_board(const uint8_t *app, uint32_t size)
{
    memset(fl, 0xFF, sizeof(fl));
    memset(qw_prog, 0, sizeof(qw_prog));
    memset(qw_torn, 0, sizeof(qw_torn));
    for (uint32_t i = 125u * SU_OTA_PAGE; i < FL_SIZE; i++) fl[i] = (uint8_t)(i * 7u);
    studio_swd_flash(app, size, 1, 1);
}

static uint32_t nvm_sum(void)
{
    return su_ota_crc32(0, fl + 125u * SU_OTA_PAGE, 3u * SU_OTA_PAGE);
}

static int flash_is(const uint8_t *img, uint32_t size)
{
    return memcmp(fl, img, size) == 0;
}

/* ============================================================
 * The device: su_core staging + the ops ble_update.c gives it
 * ============================================================ */

static uint32_t dev_app_end;           /* running app's end */
static int      dev_rebooting;
static char     replies[64][64];
static int      n_replies;
static uint8_t  next_nonce[16];
static int      rng_broken;
static su_cmac_t dev_mac;
static int      dev_mac_on;
static su_ops_t dev_ops;

static int writable(uint32_t off, uint32_t len)
{
    uint32_t lo = (dev_app_end + SU_OTA_PAGE - 1u) & ~(SU_OTA_PAGE - 1u);
    if (off >= SU_OTA_RES_OFF && off + len <= SU_OTA_RES_OFF + SU_OTA_PAGE) return 1;
    return off >= lo && off + len <= SU_OTA_APP_LIMIT;
}
static int d_erase(uint32_t page) { return writable(page * SU_OTA_PAGE, SU_OTA_PAGE) ? fl_erase(page) : -1; }
static int d_program(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint32_t off = addr - FL_BASE;
    return writable(off, len) ? fl_program(off, buf, len) : -1;
}
static void d_send(const char *line, uint32_t len)
{
    char *r = replies[n_replies % 64];
    uint32_t n = len < 63u ? len : 63u;
    check(len <= 63u && len > 0 && line[len - 1] == '\n', "reply not one line under 64 bytes");
    memcpy(r, line, n);
    r[n] = 0;
    if (n && r[n - 1] == '\n') r[n - 1] = 0;
    n_replies++;
}
static void d_reboot(void) { dev_rebooting = 1; }
static int  d_stage(uint32_t size, uint32_t *at) { return su_ota_fit(dev_app_end, size, at); }
static int  d_key_info(uint32_t *id)
{
    su_ota_key_t k;
    if (!su_ota_key_read(&sim_io, &k)) return 0;
    *id = k.key_id;
    return 1;
}
static uint32_t d_image_crc(void) { return su_crc32(0, fl, dev_app_end); }
static int d_challenge(uint8_t out[16])
{
    su_ota_key_t k;
    if (!su_ota_key_read(&sim_io, &k)) return (int)SU_ERR_NOKEY;
    if (rng_broken) return (int)SU_ERR_RNG;
    memcpy(out, next_nonce, 16);
    for (int i = 0; i < 16; i++) next_nonce[i] = (uint8_t)rnd();
    return 0;
}
static int d_auth(const uint8_t n16[16], const uint8_t tag[16])
{
    su_ota_key_t k;
    uint8_t want[16];
    if (!su_ota_key_read(&sim_io, &k)) return 0;
    su_tag_auth(k.key, n16, want);
    return su_ct_equal(want, tag, 16);
}
static void d_mac_start(uint32_t size, uint32_t crc)
{
    su_ota_key_t k;
    dev_mac_on = su_ota_key_read(&sim_io, &k);
    if (dev_mac_on) su_tag_image_start(&dev_mac, k.key, size, crc);
}
static void d_mac_update(const uint8_t *p, uint32_t len) { if (dev_mac_on) su_cmac_update(&dev_mac, p, len); }
static int d_mac_check(const uint8_t n16[16], const uint8_t tag[16])
{
    if (!dev_mac_on) return 0;
    dev_mac_on = 0;
    uint8_t it[16], want[16];
    su_cmac_final(&dev_mac, it);
    su_ota_key_t k;
    if (!su_ota_key_read(&sim_io, &k)) return 0;
    su_tag_end(k.key, n16, it, want);
    return su_ct_equal(want, tag, 16);
}
static uint32_t d_image_crc(void);
static int d_commit(uint32_t at, uint32_t size, uint32_t crc, const uint32_t keep[2])
{
    su_ota_commit_t c;
    c.stage = at; c.size = size; c.crc = crc;
    c.old_size = dev_app_end; c.old_crc = d_image_crc();
    c.keep[0] = keep[0]; c.keep[1] = keep[1];
    return su_ota_commit(&sim_io, &c);
}
static long dev_feeds;
static void d_feed(void) { dev_feeds++; }

/* Power on with the app currently in flash (ending at app_end). */
static void device_boot_app(uint32_t app_end)
{
    dev_app_end = app_end;
    dev_rebooting = 0;
    n_replies = 0;
    dev_mac_on = 0;
    memset(&dev_ops, 0, sizeof(dev_ops));
    dev_ops.flash_base  = FL_BASE;
    dev_ops.flash_size  = FL_SIZE;
    dev_ops.image_limit = su_ota_limit(app_end);
    dev_ops.page_size   = SU_MAX_PAYLOAD;
    dev_ops.dev_id      = 0x492u;
    dev_ops.sram_start  = 0x20000000u;
    dev_ops.sram_end    = 0x20020000u;
    dev_ops.flash_read  = fl;
    dev_ops.erase_page  = d_erase;
    dev_ops.program     = d_program;
    dev_ops.send        = d_send;
    dev_ops.reboot      = d_reboot;
    dev_ops.erase_size  = SU_OTA_PAGE;
    dev_ops.stage       = d_stage;
    dev_ops.key_info    = d_key_info;
    dev_ops.image_crc   = d_image_crc;
    dev_ops.challenge   = d_challenge;
    dev_ops.auth_check  = d_auth;
    dev_ops.mac_start   = d_mac_start;
    dev_ops.mac_update  = d_mac_update;
    dev_ops.mac_check   = d_mac_check;
    dev_ops.commit      = d_commit;
    dev_ops.run_size    = app_end;
    dev_ops.feed        = d_feed;
    su_core_init(&dev_ops);
}

/* ============================================================
 * The host (Studio's side)
 * ============================================================ */

static void cmac_oneshot(const uint8_t key[16], const uint8_t *m, uint32_t n, uint8_t out[16])
{
    su_cmac_t c;
    su_cmac_init(&c, key);
    su_cmac_update(&c, m, n);
    su_cmac_final(&c, out);
}

static void h_auth_tag(const uint8_t key[16], const uint8_t nonce[16], uint8_t out[16])
{
    uint8_t m[20];
    memcpy(m, "SUA1", 4);
    memcpy(m + 4, nonce, 16);
    cmac_oneshot(key, m, sizeof(m), out);
}

static void h_image_tag(const uint8_t key[16], const uint8_t *img, uint32_t size, uint32_t crc, uint8_t out[16])
{
    uint8_t *m = malloc(12u + size);
    memcpy(m, "SUI1", 4);
    for (int i = 0; i < 4; i++) { m[4 + i] = (uint8_t)(size >> (8 * i)); m[8 + i] = (uint8_t)(crc >> (8 * i)); }
    memcpy(m + 12, img, size);
    cmac_oneshot(key, m, 12u + size, out);
    free(m);
}

static void h_end_tag(const uint8_t key[16], const uint8_t nonce[16], const uint8_t itag[16], uint8_t out[16])
{
    uint8_t m[36];
    memcpy(m, "SUE1", 4);
    memcpy(m + 4, nonce, 16);
    memcpy(m + 20, itag, 16);
    cmac_oneshot(key, m, sizeof(m), out);
}

/* Send one frame as BLE writes of at most 180 bytes; return the last reply
 * line it produced (or "" for none). `stop_after` < frame length sends only
 * that many bytes (a link that drops mid-frame). */
static const char *send_frame_part(char type, uint32_t a0, uint32_t a1, const uint8_t *pl, uint32_t len, uint32_t stop_after)
{
    static uint8_t f[16 + SU_MAX_PAYLOAD + 16];
    memset(f, 0, 16);
    f[0] = 'S'; f[1] = 'U'; f[2] = (uint8_t)type;
    f[4] = (uint8_t)len; f[5] = (uint8_t)(len >> 8);
    for (int i = 0; i < 4; i++) { f[8 + i] = (uint8_t)(a0 >> (8 * i)); f[12 + i] = (uint8_t)(a1 >> (8 * i)); }
    if (len) memcpy(f + 16, pl, len);
    uint32_t total = 16u + len, sent = 0;
    if (stop_after < total) total = stop_after;
    int before = n_replies;
    while (sent < total) {
        uint32_t n = total - sent < 180u ? total - sent : 180u;
        su_core_rx(f + sent, n);
        sent += n;
    }
    return n_replies > before ? replies[(n_replies - 1) % 64] : "";
}

static const char *send_frame(char type, uint32_t a0, uint32_t a1, const uint8_t *pl, uint32_t len)
{
    return send_frame_part(type, a0, a1, pl, len, 0xFFFFFFFFu);
}

static uint8_t h_nonce[16];

/* Q, C, A with `key`. Returns the A reply. */
static const char *h_connect(const uint8_t key[16])
{
    send_frame('Q', 0, 0, 0, 0);
    const char *r = send_frame('C', 0, 0, 0, 0);
    if (strncmp(r, "SU NONCE ", 9) != 0) return r;
    for (int i = 0; i < 16; i++) {
        unsigned v;
        sscanf(r + 9 + 2 * i, "%2x", &v);
        h_nonce[i] = (uint8_t)v;
    }
    uint8_t tag[16];
    h_auth_tag(key, h_nonce, tag);
    return send_frame('A', 0, 0, tag, 16);
}

static const char *h_begin(uint32_t size, uint32_t crc)
{
    uint8_t dev[4] = { 0x92, 0x04, 0, 0 };
    return send_frame('B', size, crc, dev, 4);
}

/* D frames from `from` until `until` (exclusive, chunk-aligned or the end). */
static const char *h_data(const uint8_t *img, uint32_t size, uint32_t from, uint32_t until)
{
    const char *r = "";
    for (uint32_t off = from; off < size && off < until; off += SU_MAX_PAYLOAD) {
        uint32_t n = size - off < SU_MAX_PAYLOAD ? size - off : SU_MAX_PAYLOAD;
        r = send_frame('D', off, su_crc32(0, img + off, n), img + off, n);
        char want[32];
        snprintf(want, sizeof(want), "SU OK D %u", off + n);
        if (strcmp(r, want) != 0) return r;
    }
    return r;
}

static const char *h_end(const uint8_t key[16], const uint8_t nonce[16], const uint8_t *img, uint32_t size, uint32_t crc)
{
    uint8_t it[16], et[16];
    h_image_tag(key, img, size, crc, it);
    h_end_tag(key, nonce, it, et);
    return send_frame('E', 0, 0, et, 16);
}

/* A whole update from a connected, idle device. Returns the E reply. */
static const char *h_update(const uint8_t *img, uint32_t size)
{
    uint32_t crc = su_crc32(0, img, size);
    const char *r = h_connect(KEY);
    if (strcmp(r, "SU OK A") != 0) return r;
    r = h_begin(size, crc);
    if (strncmp(r, "SU OK B", 7) != 0) return r;
    r = h_data(img, size, 0, size);
    if (strncmp(r, "SU OK D", 7) != 0) return r;
    return h_end(KEY, h_nonce, img, size, crc);
}

/* Boot the Core until it settles: run the copier; INSTALLED resets and boots
 * again. Returns the first result. */
static int boot_until_settled(int *second)
{
    int r = su_ota_boot(&sim_io);
    int r2 = SU_OTA_BOOT_NONE;
    if (r == SU_OTA_BOOT_INSTALLED) r2 = su_ota_boot(&sim_io);
    if (second) *second = r2;
    return r;
}

/* ============================================================
 * Tests
 * ============================================================ */

static void hex(const uint8_t *p, int n, char *out)
{
    for (int i = 0; i < n; i++) sprintf(out + 2 * i, "%02x", p[i]);
}

static void test_crypto(void)
{
    /* FIPS 197 Appendix C.1 */
    static const uint8_t k[16]  = { 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f };
    static const uint8_t pt[16] = { 0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff };
    su_aes128_t a;
    uint8_t ct[16];
    char h[64];
    su_aes128_init(&a, k);
    su_aes128_encrypt(&a, pt, ct);
    hex(ct, 16, h);
    check(strcmp(h, "69c4e0d86a7b0430d8cdb78070b4c55a") == 0, "AES-128 FIPS 197 C.1: %s", h);

    /* RFC 4493 §4 */
    static const uint8_t rk[16] = { 0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c };
    static const uint8_t m[64] = {
        0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
        0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
        0x30,0xc8,0x1c,0x46,0xa3,0x5c,0xe4,0x11,0xe5,0xfb,0xc1,0x19,0x1a,0x0a,0x52,0xef,
        0xf6,0x9f,0x24,0x45,0xdf,0x4f,0x9b,0x17,0xad,0x2b,0x41,0x7b,0xe6,0x6c,0x37,0x10 };
    static const char *want[4] = { "bb1d6929e95937287fa37d129b756746", "070a16b46b4d4144f79bdd9dd04a287c",
                                   "dfa66747de9ae63030ca32611497c827", "51f0bebf7e3b9d92fc49741779363cfe" };
    static const uint32_t lens[4] = { 0, 16, 40, 64 };
    for (int i = 0; i < 4; i++) {
        uint8_t t[16];
        /* Fed in odd pieces, to exercise the streaming. */
        su_cmac_t c;
        su_cmac_init(&c, rk);
        for (uint32_t o = 0; o < lens[i]; o += 7u) su_cmac_update(&c, m + o, lens[i] - o < 7u ? lens[i] - o : 7u);
        su_cmac_final(&c, t);
        hex(t, 16, h);
        check(strcmp(h, want[i]) == 0, "CMAC RFC 4493 len %u: %s", lens[i], h);
    }
    ok("AES-128 (FIPS 197) and AES-CMAC (RFC 4493, all four, streamed)");
}

/* The spec's vectors (docs §5), cross-checked against OpenSSL 3 by hand:
 *   printf 'SUA1' | cat - nonce.bin | openssl mac -cipher AES-128-CBC -macopt hexkey:<K> CMAC */
static const uint8_t VEC_NONCE[16] = { 0xf0,0xe1,0xd2,0xc3,0xb4,0xa5,0x96,0x87,0x78,0x69,0x5a,0x4b,0x3c,0x2d,0x1e,0x0f };
#define VEC_AUTH  "9edcf74eaf564b287d7cfec175987d53"
#define VEC_CRC   0x0da62e3cu
#define VEC_ITAG  "559931c85ad5df723ad703750da306c5"
#define VEC_END   "f18a6f9daccd1c7c5de8e5b106df0802"

static void vectors(char auth[33], uint32_t *crc, char itag[33], char end[33])
{
    uint8_t img[40], t[16], it[16];
    for (int i = 0; i < 40; i++) img[i] = (uint8_t)i;
    *crc = su_crc32(0, img, 40);
    su_tag_auth(KEY, VEC_NONCE, t);
    hex(t, 16, auth);
    su_cmac_t c;
    su_tag_image_start(&c, KEY, 40, *crc);
    su_cmac_update(&c, img, 40);
    su_cmac_final(&c, it);
    hex(it, 16, itag);
    su_tag_end(KEY, VEC_NONCE, it, t);
    hex(t, 16, end);
    /* the host's one-shot path must agree */
    uint8_t t2[16];
    h_auth_tag(KEY, VEC_NONCE, t2);
    char a2[33];
    hex(t2, 16, a2);
    check(strcmp(a2, auth) == 0, "auth tag: streamed %s vs one-shot %s", auth, a2);
    h_image_tag(KEY, img, 40, *crc, t2);
    hex(t2, 16, a2);
    check(strcmp(a2, itag) == 0, "image tag: streamed %s vs one-shot %s", itag, a2);
}

static void test_vectors(void)
{
    char auth[33], itag[33], end[33];
    uint32_t crc;
    vectors(auth, &crc, itag, end);
    check(strcmp(auth, VEC_AUTH) == 0, "spec auth_tag %s", auth);
    check(crc == VEC_CRC, "spec crc %08x", crc);
    check(strcmp(itag, VEC_ITAG) == 0, "spec image_tag %s", itag);
    check(strcmp(end, VEC_END) == 0, "spec end_tag %s", end);
    ok("spec test vectors (docs §5) reproduce; streamed and one-shot tags agree");
}

#define OLD_SIZE  (140u * 1024u + 300u)
#define BIG_SIZE  (150u * 1024u + 4u)        /* bigger than the running app */
#define SMALL_SIZE (60u * 1024u + 12u)

static void test_full(uint32_t new_size, uint32_t seed, const char *what)
{
    uint8_t *old = make_image(OLD_SIZE, 1);
    uint8_t *img = make_image(new_size, seed);
    fresh_board(old, OLD_SIZE);
    uint32_t nvm = nvm_sum();
    device_boot_app(OLD_SIZE);

    const char *r = send_frame('Q', 0, 0, 0, 0);
    char want[96];
    snprintf(want, sizeof(want), "SU READY 3 492 1024 2048 0 %u 1 %x %x delta 1", su_ota_limit(OLD_SIZE), KEY_ID,
             su_crc32(0, old, OLD_SIZE));
    check(strcmp(r, want) == 0, "READY: '%s' want '%s'", r, want);

    r = h_update(img, new_size);
    check(strcmp(r, "SU DONE") == 0, "%s: E -> '%s'", what, r);
    check(dev_rebooting, "%s: reboot requested", what);
    check(flash_is(old, OLD_SIZE), "%s: running app untouched by the transfer", what);

    int second;
    int b = boot_until_settled(&second);
    check(b == SU_OTA_BOOT_INSTALLED && second == SU_OTA_BOOT_NONE, "%s: boot %d then %d", what, b, second);
    /* The new app reports its own CRC: what Studio compares to confirm. */
    device_boot_app(new_size);
    snprintf(want, sizeof(want), "SU READY 3 492 1024 2048 0 %u 1 %x %x delta 1", su_ota_limit(new_size), KEY_ID,
             su_crc32(0, img, new_size));
    r = send_frame('Q', 0, 0, 0, 0);
    check(strcmp(r, want) == 0, "READY after: '%s' want '%s'", r, want);
    check(flash_is(img, new_size), "%s: new image in place", what);
    check(nvm_sum() == nvm, "%s: pages 125-127 untouched", what);
    su_ota_key_t k;
    check(su_ota_key_read(&sim_io, &k) && k.key_id == KEY_ID, "%s: key kept", what);
    ok(what);
    free(old);
    free(img);
}

static void test_resume(void)
{
    uint8_t *old = make_image(OLD_SIZE, 1);
    uint8_t *img = make_image(BIG_SIZE, 7);
    uint32_t crc = su_crc32(0, img, BIG_SIZE);
    fresh_board(old, OLD_SIZE);
    device_boot_app(OLD_SIZE);

    check(strcmp(h_connect(KEY), "SU OK A") == 0, "resume: auth");
    check(strcmp(h_begin(BIG_SIZE, crc), "SU OK B 0") == 0, "resume: begin");
    h_data(img, BIG_SIZE, 0, 40u * 2048u);
    /* Half of the next frame, then the link drops. */
    send_frame_part('D', 40u * 2048u, su_crc32(0, img + 40u * 2048u, 2048), img + 40u * 2048u, 2048, 700);
    su_core_link_reset();

    /* New link: nothing but Q / C / A without auth. */
    uint8_t dev[4] = { 0x92, 0x04, 0, 0 };
    check(strcmp(send_frame('B', BIG_SIZE, crc, dev, 4), "SU ERR 12 noauth") == 0, "resume: B before A");
    check(strcmp(h_connect(KEY), "SU OK A") == 0, "resume: auth again");
    const char *r = h_begin(BIG_SIZE, crc);
    check(strcmp(r, "SU OK B 81920") == 0, "resume: B -> '%s'", r);
    /* The lost reply case: the last chunk again is acknowledged unchanged. */
    r = send_frame('D', 39u * 2048u, su_crc32(0, img + 39u * 2048u, 2048), img + 39u * 2048u, 2048);
    check(strcmp(r, "SU OK D 81920") == 0, "resume: duplicate chunk -> '%s'", r);
    r = send_frame('D', 50u * 2048u, su_crc32(0, img + 50u * 2048u, 2048), img + 50u * 2048u, 2048);
    check(strcmp(r, "SU ERR 6 order") == 0, "resume: out of order -> '%s'", r);
    r = h_data(img, BIG_SIZE, 40u * 2048u, BIG_SIZE);
    check(strncmp(r, "SU OK D", 7) == 0, "resume: rest -> '%s'", r);
    /* E with the nonce of THIS link. */
    r = h_end(KEY, h_nonce, img, BIG_SIZE, crc);
    check(strcmp(r, "SU DONE") == 0, "resume: E -> '%s'", r);
    int second;
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED && flash_is(img, BIG_SIZE), "resume: installed");
    ok("link drop mid-frame + resume by offset (re-auth, duplicate and out-of-order chunks)");
    free(old);
    free(img);
}

static void test_bad_auth(void)
{
    uint8_t *old = make_image(OLD_SIZE, 1);
    uint8_t *img = make_image(SMALL_SIZE, 9);
    uint32_t crc = su_crc32(0, img, SMALL_SIZE);
    fresh_board(old, OLD_SIZE);
    device_boot_app(OLD_SIZE);

    check(strcmp(h_connect(WRONG), "SU ERR 11 auth") == 0, "wrong key refused");
    uint8_t t[16] = { 0 };
    check(strcmp(send_frame('A', 0, 0, t, 16), "SU ERR 11 auth") == 0, "nonce spent after a wrong A");
    check(strcmp(send_frame('A', 0, 0, t, 3), "SU ERR 1 length") == 0, "short A");
    uint8_t dev[4] = { 0x92, 0x04, 0, 0 };
    check(strcmp(send_frame('B', SMALL_SIZE, crc, dev, 4), "SU ERR 12 noauth") == 0, "B without auth");
    check(strcmp(send_frame('X', 0, 0, 0, 0), "SU ERR 12 noauth") == 0, "X without auth");

    /* Right key, wrong image tag (key mismatch on the image, i.e. tampering). */
    check(strcmp(h_connect(KEY), "SU OK A") == 0, "auth");
    h_begin(SMALL_SIZE, crc);
    h_data(img, SMALL_SIZE, 0, SMALL_SIZE);
    const char *r = h_end(WRONG, h_nonce, img, SMALL_SIZE, crc);
    check(strcmp(r, "SU ERR 13 mac") == 0, "bad end tag -> '%s'", r);
    su_ota_state_t s;
    su_ota_scan(&sim_io, &s);
    check(!s.pending, "no marker after a bad tag");
    check(strcmp(h_begin(SMALL_SIZE, crc), "SU OK B 0") == 0, "transfer forgotten after a bad tag");

    /* A tampered chunk: CRC and tag sent for the real image, data altered in
     * flight with a matching chunk CRC → image CRC catches it... and a chunk
     * altered together with the whole-image CRC → the tag catches it. */
    uint8_t *evil = malloc(SMALL_SIZE);
    memcpy(evil, img, SMALL_SIZE);
    evil[5000] ^= 0x55;
    uint32_t ecrc = su_crc32(0, evil, SMALL_SIZE);
    check(strcmp(h_begin(SMALL_SIZE, ecrc), "SU OK B 0") == 0, "evil begin");
    h_data(evil, SMALL_SIZE, 0, SMALL_SIZE);
    uint8_t it[16], et[16];
    h_image_tag(KEY, img, SMALL_SIZE, crc, it);          /* the tag of the genuine image */
    h_end_tag(KEY, h_nonce, it, et);
    check(strcmp(send_frame('E', 0, 0, et, 16), "SU ERR 13 mac") == 0, "altered image + CRC refused by the tag");

    /* Replay: a good E from an earlier link doesn't verify under a new nonce. */
    check(strcmp(h_begin(SMALL_SIZE, crc), "SU OK B 0") == 0, "replay begin");
    h_data(img, SMALL_SIZE, 0, SMALL_SIZE);
    uint8_t old_nonce[16];
    memcpy(old_nonce, h_nonce, 16);
    su_core_link_reset();
    check(strcmp(h_connect(KEY), "SU OK A") == 0, "replay: new link");
    check(strcmp(h_begin(SMALL_SIZE, crc), "SU OK B 61452") == 0, "replay: resumed at the end");
    check(strcmp(h_end(KEY, old_nonce, img, SMALL_SIZE, crc), "SU ERR 13 mac") == 0, "replayed end tag refused");
    su_ota_scan(&sim_io, &s);
    check(!s.pending && flash_is(old, OLD_SIZE), "nothing installed");
    check(su_ota_boot(&sim_io) == SU_OTA_BOOT_NONE, "boot: nothing pending");
    ok("wrong key, spent nonce, no auth, bad / tampered / replayed end tag: refused, nothing installed");
    free(evil);
    free(old);
    free(img);
}

static void test_refusals(void)
{
    uint8_t *old = make_image(OLD_SIZE, 1);
    fresh_board(old, OLD_SIZE);
    device_boot_app(OLD_SIZE);
    check(strcmp(h_connect(KEY), "SU OK A") == 0, "auth");

    uint32_t lim = su_ota_limit(OLD_SIZE);
    check(lim == SU_OTA_APP_LIMIT / 2u, "limit beside a 140 KB app: %u", lim);
    uint8_t dev[4] = { 0x92, 0x04, 0, 0 };
    check(strcmp(send_frame('B', lim + 16u, 1, dev, 4), "SU ERR 3 size") == 0, "too big refused");
    check(strncmp(send_frame('B', lim, 1, dev, 4), "SU OK B", 7) == 0, "exactly the limit fits");
    check(strcmp(send_frame('B', 4, 1, dev, 4), "SU ERR 3 size") == 0, "too small refused");
    uint8_t dev2[4] = { 0x64, 0x04, 0, 0 };
    check(strcmp(send_frame('B', 4096, 1, dev2, 4), "SU ERR 2 device") == 0, "L4 DEV_ID refused");

    /* A big running app shrinks the limit: 700 KB app → 1015808 - 704 KB. */
    check(su_ota_limit(700u * 1024u) == SU_OTA_APP_LIMIT - 704u * 1024u, "limit beside a 700 KB app");
    uint32_t st;
    check(su_ota_fit(700u * 1024u, SU_OTA_APP_LIMIT - 704u * 1024u, &st) == 0 && st == 704u * 1024u, "fit at the limit");
    check(su_ota_fit(700u * 1024u, SU_OTA_APP_LIMIT - 704u * 1024u + 1u, &st) != 0, "fit past the limit");
    check(su_ota_fit(100u, 400u * 1024u, &st) == 0 && st == 400u * 1024u, "a bigger image stages above itself");
    check(su_ota_fit(100u, 400u * 1024u + 1u, &st) == 0 && st == 408u * 1024u, "... page-aligned");
    check(su_ota_fit(100u, 500u * 1024u, &st) != 0, "two 500 KB copies don't fit in 992 KB");

    /* Not a vector table: refused at chunk 0, staging untouched. */
    uint8_t *bad = make_image(8192, 3);
    uint32_t sp = 0x30000000u;
    memcpy(bad, &sp, 4);
    uint32_t crc = su_crc32(0, bad, 8192);
    send_frame('B', 8192, crc, dev, 4);
    long before = ops;
    const char *r = send_frame('D', 0, su_crc32(0, bad, 2048), bad, 2048);
    check(strcmp(r, "SU ERR 4 vectors") == 0 && ops == before, "bad vectors -> '%s', %ld flash ops", r, ops - before);

    /* Abort forgets; chunk CRC wrong is resendable. */
    uint8_t *img = make_image(8192, 4);
    crc = su_crc32(0, img, 8192);
    send_frame('B', 8192, crc, dev, 4);
    r = send_frame('D', 0, crc ^ 1u, img, 2048);
    check(strcmp(r, "SU ERR 7 crc") == 0, "chunk crc -> '%s'", r);
    check(strcmp(h_data(img, 8192, 0, 4096), "SU OK D 4096") == 0, "resend ok");
    check(strcmp(send_frame('X', 0, 0, 0, 0), "SU OK X") == 0, "abort");
    check(strcmp(send_frame('D', 4096, su_crc32(0, img + 4096, 2048), img + 4096, 2048), "SU ERR 9 state") == 0, "D after abort");
    check(strcmp(send_frame('E', 0, 0, img, 16), "SU ERR 9 state") == 0, "E without a transfer");
    check(!dev_rebooting && flash_is(old, OLD_SIZE), "abort: app untouched, no reset");

    /* Garbage between frames is skipped silently. */
    int n = n_replies;
    uint8_t junk[40] = "SUZ...hello SU";
    su_core_rx(junk, sizeof(junk));
    check(n_replies == n, "junk answered");
    check(strncmp(send_frame('Q', 0, 0, 0, 0), "SU READY 3", 10) == 0, "Q after junk");

    /* No key: READY says so, C refuses, nothing else is possible. */
    fl_erase(SU_OTA_RES_PAGE);
    device_boot_app(OLD_SIZE);
    r = send_frame('Q', 0, 0, 0, 0);
    char want[80];
    snprintf(want, sizeof(want), "SU READY 3 492 1024 2048 0 %u 0 0 %x delta 1", lim, su_crc32(0, old, OLD_SIZE));
    check(strcmp(r, want) == 0, "READY without a key: '%s'", r);
    check(strcmp(send_frame('C', 0, 0, 0, 0), "SU ERR 10 nokey") == 0, "C without a key");
    /* A torn key record is no key either. */
    uint32_t rec[8];
    key_record(KEY_ID, KEY, rec);
    rec[5] ^= 1u;
    fl_program(SU_OTA_RES_OFF, (const uint8_t *)rec, 32);
    check(strcmp(send_frame('C', 0, 0, 0, 0), "SU ERR 10 nokey") == 0, "C with a corrupt key record");
    /* RNG fault */
    fresh_board(old, OLD_SIZE);
    device_boot_app(OLD_SIZE);
    rng_broken = 1;
    check(strcmp(send_frame('C', 0, 0, 0, 0), "SU ERR 14 rng") == 0, "RNG fault");
    rng_broken = 0;
    ok("too big / too small / wrong chip / bad vectors / chunk CRC / abort / junk / no key / RNG fault");
    free(bad);
    free(img);
    free(old);
}

/* Commit an update of `img` (size) beside `old` and stop before the reset. */
static void staged(const uint8_t *old, uint32_t old_size, const uint8_t *img, uint32_t size)
{
    fresh_board(old, old_size);
    device_boot_app(old_size);
    const char *r = h_update(img, size);
    check(strcmp(r, "SU DONE") == 0, "staged: '%s'", r);
}

/* Power cut at every flash operation of the copy (or every `step`-th), then
 * boot again: the install must complete, except for cuts inside the page-0
 * window (counted: that is the documented SWD-recovery case). */
static void sweep_copy(uint32_t old_size, uint32_t size, long step, int same_p0, const char *what)
{
    uint8_t *old = make_image(old_size, 11);
    uint8_t *img = make_image(size, 12);
    /* Page 0 identical (the usual delta case): the copier must leave it alone. */
    if (same_p0) memcpy(img, old, SU_OTA_PAGE);
    staged(old, old_size, img, size);
    static uint8_t snap[FL_SIZE], snap_prog[FL_QWS], snap_torn[FL_QWS];
    memcpy(snap, fl, FL_SIZE);
    memcpy(snap_prog, qw_prog, FL_QWS);
    memcpy(snap_torn, qw_torn, FL_QWS);

    /* Count the copier's operations once. */
    long start = ops, e0 = page0_erases;
    int second;
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED, "%s: clean install", what);
    long total = ops - start;
    check(same_p0 ? page0_erases == e0 : page0_erases == e0 + 1, "%s: page 0 erased %ld times", what, page0_erases - e0);

    int bricked = 0, completed = 0;
    for (long k = 1; k <= total; k += step) {
        memcpy(fl, snap, FL_SIZE);
        memcpy(qw_prog, snap_prog, FL_QWS);
        memcpy(qw_torn, snap_torn, FL_QWS);
        long base = ops;
        cut_at = base + k;
        if (setjmp(cut_env) == 0) {
            (void)boot_until_settled(&second);
            cut_at = -1;
        }
        /* Powered again. Bootable only if the first quad-word (SP + reset
         * vector) is a clean, programmed one. */
        int bootable = !qw_torn[0] && qw_prog[0];
        if (!bootable) {
            bricked++;
            continue;
        }
        int b = boot_until_settled(&second);
        int fine = flash_is(img, size) &&
                   (b == SU_OTA_BOOT_INSTALLED || b == SU_OTA_BOOT_MARKED || b == SU_OTA_BOOT_NONE);
        check(fine, "%s: cut at op %ld of %ld: boot %d, image %s", what, k, total, b, flash_is(img, size) ? "ok" : "WRONG");
        su_ota_state_t s;
        su_ota_scan(&sim_io, &s);
        check(!s.pending, "%s: cut at op %ld: still pending after recovery", what, k);
        if (fine) completed++;
    }
    char line[200];
    snprintf(line, sizeof(line), "%s: power cut at %s%ld copy ops -> %d installed on the next boot, %d in the page-0 window (SWD)",
             what, step > 1 ? "every " : "each of ", step > 1 ? step : total, completed, bricked);
    if (step > 1) snprintf(line, sizeof(line), "%s: power cut at every %ld-th of %ld copy ops -> %d installed on the next boot, %d in the page-0 window (SWD)",
                           what, step, total, completed, bricked);
    /* The window: page 0's erase plus its quad-words; nothing else may brick,
     * and nothing at all when page 0 is unchanged. */
    if (same_p0) check(bricked == 0, "%s: %d bricked cuts with page 0 unchanged", what, bricked);
    else check(bricked <= (int)(SU_OTA_PAGE / 16u + 1u) / (int)step + 1, "%s: %d bricked cuts, more than the page-0 window", what, bricked);
    ok(line);
    free(old);
    free(img);
}

static void sweep_commit(void)
{
    uint8_t *old = make_image(40u * 1024u, 21);
    uint8_t *img = make_image(30u * 1024u, 22);
    uint32_t crc = su_crc32(0, img, 30u * 1024u);
    int installed = 0, old_kept = 0;
    for (long k = 1; k <= 2; k++) {        /* COPY-A, COPY-B: one program each */
        fresh_board(old, 40u * 1024u);
        device_boot_app(40u * 1024u);
        h_connect(KEY);
        h_begin(30u * 1024u, crc);
        h_data(img, 30u * 1024u, 0, 30u * 1024u);
        cut_at = ops + k;
        if (setjmp(cut_env) == 0) {
            h_end(KEY, h_nonce, img, 30u * 1024u, crc);
            cut_at = -1;
        }
        int second;
        int b = boot_until_settled(&second);
        if (flash_is(img, 30u * 1024u)) installed++;
        else if (flash_is(old, 40u * 1024u) && b == SU_OTA_BOOT_NONE) old_kept++;
        else check(0, "commit cut %ld: neither old nor new (boot %d)", k, b);
        /* and the next update still works */
        uint32_t end = flash_is(img, 30u * 1024u) ? 30u * 1024u : 40u * 1024u;
        device_boot_app(end);
        uint8_t *img2 = make_image(20u * 1024u, 23);
        check(strcmp(h_update(img2, 20u * 1024u), "SU DONE") == 0, "update after a commit cut");
        check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED && flash_is(img2, 20u * 1024u), "installed after a commit cut");
        free(img2);
    }
    char line[160];
    snprintf(line, sizeof(line), "power cut while writing the marker: old app %d, new app %d, never a mix; next update fine", old_kept, installed);
    ok(line);
    free(old);
    free(img);
}

static void sweep_transfer(void)
{
    uint8_t *old = make_image(OLD_SIZE, 31);
    uint8_t *img = make_image(BIG_SIZE, 32);
    uint32_t crc = su_crc32(0, img, BIG_SIZE);
    int runs = 0;
    for (long k = 1; k < 1200; k += 37) {
        fresh_board(old, OLD_SIZE);
        device_boot_app(OLD_SIZE);
        h_connect(KEY);
        h_begin(BIG_SIZE, crc);
        cut_at = ops + k;
        if (setjmp(cut_env) == 0) {
            h_data(img, BIG_SIZE, 0, BIG_SIZE);
            cut_at = -1;
        }
        /* Power back: the old app runs, nothing pending; a new transfer starts at 0. */
        check(flash_is(old, OLD_SIZE) && su_ota_boot(&sim_io) == SU_OTA_BOOT_NONE, "transfer cut %ld: old app", k);
        device_boot_app(OLD_SIZE);
        const char *r = h_update(img, BIG_SIZE);
        check(strcmp(r, "SU DONE") == 0, "transfer cut %ld: retry -> '%s'", k, r);
        int second;
        check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED && flash_is(img, BIG_SIZE), "transfer cut %ld: installed", k);
        runs++;
    }
    char line[120];
    snprintf(line, sizeof(line), "power cut during the transfer (%d points): old app boots, the retry installs", runs);
    ok(line);
    free(old);
    free(img);
}

static void test_swd(void)
{
    uint8_t *old = make_image(OLD_SIZE, 41);
    uint8_t *img = make_image(SMALL_SIZE, 42);
    uint8_t *swd = make_image(90u * 1024u, 43);

    /* Studio's SWD flash clears the marker: the copier leaves its image alone. */
    staged(old, OLD_SIZE, img, SMALL_SIZE);
    studio_swd_flash(swd, 90u * 1024u, 0, 1);
    check(su_ota_boot(&sim_io) == SU_OTA_BOOT_NONE && flash_is(swd, 90u * 1024u), "Studio SWD flash: marker cleared, image kept");
    su_ota_key_t k;
    check(su_ota_key_read(&sim_io, &k) && k.key_id == KEY_ID && memcmp(k.key, KEY, 16) == 0, "Studio SWD flash: key kept");

    /* Another tool that doesn't clear it: the running image's CRC no longer
     * matches COPY-B, so the copier abandons the install. */
    staged(old, OLD_SIZE, img, SMALL_SIZE);
    studio_swd_flash(swd, 90u * 1024u, 0, 0);
    check(su_ota_boot(&sim_io) == SU_OTA_BOOT_ABANDONED && flash_is(swd, 90u * 1024u), "foreign SWD flash: abandoned, image kept");
    check(su_ota_boot(&sim_io) == SU_OTA_BOOT_NONE, "foreign SWD flash: settled");

    /* Interrupted copy, then Studio reflashes over SWD: its image stays. */
    staged(old, OLD_SIZE, img, SMALL_SIZE);
    cut_at = ops + 300;
    if (setjmp(cut_env) == 0) { su_ota_boot(&sim_io); cut_at = -1; }
    studio_swd_flash(swd, 90u * 1024u, 0, 1);
    check(su_ota_boot(&sim_io) == SU_OTA_BOOT_NONE && flash_is(swd, 90u * 1024u), "SWD recovery after a cut copy");
    ok("SWD flash: Studio's (clears the marker, keeps the key) and a foreign one (copier abandons)");
    free(old);
    free(img);
    free(swd);
}

/* The reflash guard (docs §8.2): COPY-B holds the running image's size and
 * CRC; the boot that would start the copy checks them. */
static void test_guard(void)
{
    uint8_t *old = make_image(OLD_SIZE, 81);
    uint8_t *img = make_image(BIG_SIZE, 82);
    memcpy(img, old, SU_OTA_PAGE);                /* page 0 unchanged, as usual now */
    su_ota_state_t s;

    /* COPY-B records what was running. */
    staged(old, OLD_SIZE, img, BIG_SIZE);
    su_ota_scan(&sim_io, &s);
    check(s.pending && !s.started && s.old_size == OLD_SIZE && s.old_crc == su_crc32(0, old, OLD_SIZE),
          "COPY-B: old size %u crc %08x", s.old_size, s.old_crc);

    /* A foreign tool reflashes a program with the same page 0 (the case the
     * old page-0 compare missed) and leaves page 124 alone: abandoned. */
    uint8_t *swd = make_image(OLD_SIZE, 83);
    memcpy(swd, old, SU_OTA_PAGE);
    studio_swd_flash(swd, OLD_SIZE, 0, 0);
    long e0 = ops;
    check(su_ota_boot(&sim_io) == SU_OTA_BOOT_ABANDONED && flash_is(swd, OLD_SIZE), "same-page-0 reflash: abandoned, its image kept");
    check(ops - e0 == 1, "same-page-0 reflash: only the DONE written (%ld ops)", ops - e0);
    check(su_ota_boot(&sim_io) == SU_OTA_BOOT_NONE, "same-page-0 reflash: settled");
    /* One byte changed at the very end of the old image is caught too. */
    staged(old, OLD_SIZE, img, BIG_SIZE);
    uint8_t qw[16];
    memcpy(qw, fl + ((OLD_SIZE - 1u) & ~15u), 16);
    fl_erase((OLD_SIZE - 1u) / SU_OTA_PAGE);
    for (uint32_t o = ((OLD_SIZE - 1u) / SU_OTA_PAGE) * SU_OTA_PAGE; o < OLD_SIZE; o += 16u) {
        uint8_t q2[16];
        memset(q2, 0xFF, 16);
        memcpy(q2, old + o, OLD_SIZE - o < 16u ? OLD_SIZE - o : 16u);
        if (o == ((OLD_SIZE - 1u) & ~15u)) q2[(OLD_SIZE - 1u) & 15u] ^= 0x01;
        fl_program(o, q2, 16);
    }
    (void)qw;
    check(su_ota_boot(&sim_io) == SU_OTA_BOOT_ABANDONED, "last byte of the old image changed: abandoned");

    /* A resumed copy is not re-checked: [0, old size) is half replaced. */
    staged(old, OLD_SIZE, img, BIG_SIZE);
    cut_at = ops + 400;                           /* START written, a few pages copied */
    if (setjmp(cut_env) == 0) { su_ota_boot(&sim_io); cut_at = -1; }
    su_ota_scan(&sim_io, &s);
    int mixed = !flash_is(old, OLD_SIZE) && !flash_is(img, BIG_SIZE);
    check(s.pending && s.started && mixed, "cut mid-copy: started %d, flash mixed %d", s.started, mixed);
    int second;
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED && flash_is(img, BIG_SIZE), "resumed copy: installed without the guard");

    /* A torn START (cut while programming it): the next boot runs the guard
     * again (flash untouched) and installs. */
    staged(old, OLD_SIZE, img, BIG_SIZE);
    long st0 = ops;
    int ok1 = 1;
    (void)ok1;
    /* The copier's first flash op is the START program. */
    cut_at = st0 + 1;
    if (setjmp(cut_env) == 0) { su_ota_boot(&sim_io); cut_at = -1; }
    su_ota_scan(&sim_io, &s);
    check(s.pending && !s.started && flash_is(old, OLD_SIZE), "torn START: not started, old image intact");
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED && flash_is(img, BIG_SIZE), "torn START: installed on the next boot");

    /* The same image again: marked installed, nothing copied. */
    staged(old, OLD_SIZE, old, OLD_SIZE);
    e0 = ops;
    check(su_ota_boot(&sim_io) == SU_OTA_BOOT_MARKED && ops - e0 == 1 && flash_is(old, OLD_SIZE), "identical image: DONE only");

    /* An install pending in the v1 format ("COPB", page-0 CRCs, written by
     * firmware before this change) reads as not pending: nothing is copied. */
    fresh_board(old, OLD_SIZE);
    uint32_t rec[8];
    uint32_t st;
    su_ota_fit(OLD_SIZE, BIG_SIZE, &st);
    rec[0] = SU_OTA_COPA; rec[1] = FL_BASE + st; rec[2] = BIG_SIZE; rec[3] = su_crc32(0, img, BIG_SIZE);
    rec[4] = SU_OTA_COPB_V1; rec[5] = su_crc32(0, old, SU_OTA_PAGE); rec[6] = su_crc32(0, img, SU_OTA_PAGE);
    rec[7] = su_ota_crc32(su_ota_crc32(0, (const uint8_t *)&rec[0], 16), (const uint8_t *)&rec[4], 12);
    fl_program(SU_OTA_RES_OFF + SU_OTA_LOG_OFF, (const uint8_t *)rec, 32);
    /* and a v2 pair ("COP2": old size + CRC, no KEEP) after it */
    rec[4] = SU_OTA_COPB_V2; rec[5] = OLD_SIZE; rec[6] = su_crc32(0, old, OLD_SIZE);
    rec[7] = su_ota_crc32(su_ota_crc32(0, (const uint8_t *)&rec[0], 16), (const uint8_t *)&rec[4], 12);
    fl_program(SU_OTA_RES_OFF + SU_OTA_LOG_OFF + 32u, (const uint8_t *)rec, 32);
    su_ota_scan(&sim_io, &s);
    e0 = ops;
    check(!s.pending && su_ota_boot(&sim_io) == SU_OTA_BOOT_NONE && ops == e0 && flash_is(old, OLD_SIZE), "v1 / v2 markers: ignored");
    /* ... and the next update appends after it and installs. */
    device_boot_app(OLD_SIZE);
    check(strcmp(h_update(img, BIG_SIZE), "SU DONE") == 0, "update after a v1 marker");
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED && flash_is(img, BIG_SIZE), "installed after a v1 marker");

    ok("reflash guard: same-page-0 reflash and a 1-byte change abandoned; resumed copy and torn START install; identical image and v1 / v2 markers");
    free(swd);
    free(old);
    free(img);
}

/* ============================================================
 * Delta (S frames)
 * ============================================================ */

static int n_s, n_d;
static int plan_pages = 1;             /* 1: S runs stop at 8 KB page boundaries (Studio's plan) */

/* The host's delta plan: chunks equal to `base` (the image it believes is
 * running, base_size bytes) go as S runs of up to 8 KB, the rest as D.
 * Returns the last reply; stops at the first unexpected one. */
static const char *h_delta(const uint8_t *base, uint32_t base_size, const uint8_t *img, uint32_t size,
                           uint32_t from, uint32_t until)
{
    const char *r = "";
    char want[32];
    uint32_t run_off = 0, run_len = 0;
    if (until < size) size = until;              /* stop early (the image is still `size` long) */
    n_s = n_d = 0;
    for (uint32_t off = from; off < size; off += SU_MAX_PAYLOAD) {
        uint32_t n = size - off < SU_MAX_PAYLOAD ? size - off : SU_MAX_PAYLOAD;
        int same = off + n <= base_size && memcmp(img + off, base + off, n) == 0;
        if (same) {
            if (run_len == 0) run_off = off;
            run_len += n;
            int at_page = plan_pages && ((off + n) % SU_OTA_PAGE) == 0;
            if (run_len < SU_SAME_MAX && off + n < size && !at_page) continue;
        }
        if (run_len) {
            r = send_frame('S', run_off, run_len, 0, 0);
            n_s++;
            snprintf(want, sizeof(want), "SU OK S %u", run_off + run_len);
            if (strcmp(r, want) != 0) return r;
            run_len = 0;
        }
        if (!same) {
            r = send_frame('D', off, su_crc32(0, img + off, n), img + off, n);
            n_d++;
            snprintf(want, sizeof(want), "SU OK D %u", off + n);
            if (strcmp(r, want) != 0) return r;
        }
    }
    return r;
}

static const char *h_delta_update(const uint8_t *base, uint32_t base_size, const uint8_t *img, uint32_t size)
{
    uint32_t crc = su_crc32(0, img, size);
    const char *r = h_connect(KEY);
    if (strcmp(r, "SU OK A") != 0) return r;
    r = h_begin(size, crc);
    if (strncmp(r, "SU OK B", 7) != 0) return r;
    r = h_delta(base, base_size, img, size, 0, size);
    if (strncmp(r, "SU OK", 5) != 0) return r;
    return h_end(KEY, h_nonce, img, size, crc);
}

/* A new image from the old one: a few bytes changed at `at`, size `size`
 * (grown with fresh bytes, or cut short). */
static uint8_t *edit_of(const uint8_t *old, uint32_t old_size, uint32_t size, uint32_t at)
{
    uint8_t *img = make_image(size, 91);
    memcpy(img, old, size < old_size ? size : old_size);
    if (at + 3u < size) { img[at] ^= 0x5A; img[at + 3u] ^= 0x11; }
    return img;
}

static void test_delta(void)
{
    uint8_t *old = make_image(OLD_SIZE, 1);
    uint32_t lim = su_ota_limit(OLD_SIZE);
    int second;
    char want[96];
    (void)lim;

    /* 1. A one-chunk edit, image grows past the old end: aligned S runs, one
     * D, S again, D for the tail the old image doesn't have. */
    uint8_t *img = edit_of(old, OLD_SIZE, BIG_SIZE, 100u * 1024u + 77u);
    fresh_board(old, OLD_SIZE);
    device_boot_app(OLD_SIZE);
    long f0 = dev_feeds;
    const char *r = h_delta_update(old, OLD_SIZE, img, BIG_SIZE);
    check(strcmp(r, "SU DONE") == 0, "delta grow: E -> '%s'", r);
    check(dev_feeds > f0, "delta: watchdog fed during S runs");
    int s1 = n_s, d1 = n_d;
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED && flash_is(img, BIG_SIZE), "delta grow: installed");
    device_boot_app(BIG_SIZE);
    snprintf(want, sizeof(want), "SU READY 3 492 1024 2048 0 %u 1 %x %x delta 1", su_ota_limit(BIG_SIZE), KEY_ID,
             su_crc32(0, img, BIG_SIZE));
    check(strcmp(send_frame('Q', 0, 0, 0, 0), want) == 0, "delta grow: READY names the new image");
    free(img);

    /* 2. Shrinks, ending inside an S run: the final run is short (not a
     * multiple of 2048) and ends at the image size. */
    uint32_t shrunk = OLD_SIZE - 5000u;            /* 138 KB + ..., not chunk-aligned */
    img = edit_of(old, OLD_SIZE, shrunk, 20u * 1024u);
    fresh_board(old, OLD_SIZE);
    device_boot_app(OLD_SIZE);
    r = h_delta_update(old, OLD_SIZE, img, shrunk);
    check(strcmp(r, "SU DONE") == 0, "delta shrink: E -> '%s'", r);
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED && flash_is(img, shrunk), "delta shrink: installed");
    free(img);

    /* 3. Frame rules, on a fresh transfer. */
    img = edit_of(old, OLD_SIZE, BIG_SIZE, 50u * 1024u);
    uint32_t crc = su_crc32(0, img, BIG_SIZE);
    fresh_board(old, OLD_SIZE);
    device_boot_app(OLD_SIZE);
    check(strcmp(send_frame('S', 0, 8192, 0, 0), "SU ERR 12 noauth") == 0, "S before A");
    check(strcmp(h_connect(KEY), "SU OK A") == 0, "auth");
    check(strcmp(send_frame('S', 0, 8192, 0, 0), "SU ERR 9 state") == 0, "S before B");
    check(strcmp(h_begin(BIG_SIZE, crc), "SU OK B 0") == 0, "begin");
    long o0 = ops;
    check(strcmp(send_frame('S', 0, 0, 0, 0), "SU ERR 1 length") == 0, "S length 0");
    check(strcmp(send_frame('S', 0, 8192u + 2048u, 0, 0), "SU ERR 1 length") == 0, "S longer than 8 KB");
    check(strcmp(send_frame('S', 0, 3000, 0, 0), "SU ERR 1 length") == 0, "S not a chunk multiple, not at the end");
    uint8_t junk[4] = { 1, 2, 3, 4 };
    check(strcmp(send_frame('S', 0, 2048, junk, 4), "SU ERR 1 length") == 0, "S with a payload");
    check(strcmp(send_frame('S', 2048, 2048, 0, 0), "SU ERR 6 order") == 0, "S not at the next offset");
    check(strcmp(send_frame('S', 1024, 2048, 0, 0), "SU ERR 6 order") == 0, "S at an unaligned offset");
    check(ops == o0, "refused S frames touch no flash");
    /* Window of two, S and D counted together: S, D, S in one burst through
     * the receive path, replies in order. */
    {
        static uint8_t burst[3 * 16 + SU_MAX_PAYLOAD];
        uint32_t n = 0;
        uint8_t h[16];
        memset(h, 0, 16); h[0] = 'S'; h[1] = 'U'; h[2] = 'S';
        uint32_t a0 = 0, a1 = 8192;
        memcpy(h + 8, &a0, 4); memcpy(h + 12, &a1, 4);
        memcpy(burst + n, h, 16); n += 16;
        memset(h, 0, 16); h[0] = 'S'; h[1] = 'U'; h[2] = 'D'; h[5] = 0x08;
        a0 = 8192; a1 = su_crc32(0, img + 8192, 2048);
        memcpy(h + 8, &a0, 4); memcpy(h + 12, &a1, 4);
        memcpy(burst + n, h, 16); n += 16;
        memcpy(burst + n, img + 8192, 2048); n += 2048;
        memset(h, 0, 16); h[0] = 'S'; h[1] = 'U'; h[2] = 'S';
        a0 = 10240; a1 = 6144;
        memcpy(h + 8, &a0, 4); memcpy(h + 12, &a1, 4);
        memcpy(burst + n, h, 16); n += 16;
        int before = n_replies;
        uint32_t fed = 0;
        while (fed < n) {
            uint32_t k = n - fed < 180u ? n - fed : 180u;
            uint32_t room = su_core_rx_space();
            if (k > room) k = room;
            check(k > 0, "window: receive buffer stuck");
            if (!k) break;
            su_core_rx(burst + fed, k);
            fed += k;
        }
        check(n_replies - before == 3 &&
              strcmp(replies[(before) % 64], "SU OK S 8192") == 0 &&
              strcmp(replies[(before + 1) % 64], "SU OK D 10240") == 0 &&
              strcmp(replies[(before + 2) % 64], "SU OK S 16384") == 0,
              "window S,D,S: %d replies '%s' '%s' '%s'", n_replies - before, replies[before % 64],
              replies[(before + 1) % 64], replies[(before + 2) % 64]);
    }
    /* A lost OK: the run just written again is acknowledged unchanged. */
    o0 = ops;
    check(strcmp(send_frame('S', 10240, 6144, 0, 0), "SU OK S 16384") == 0 && ops == o0, "duplicate S acknowledged, no flash op");
    /* Past the new image's end; past the running image's end. */
    check(strcmp(send_frame('S', 16384, BIG_SIZE, 0, 0), "SU ERR 1 length") == 0, "S run longer than 8 KB (to the end)");
    check(strcmp(send_frame('S', 16384, 8192, 0, 0), "SU OK S 24576") == 0, "S 16-24 KB");
    r = h_delta(old, OLD_SIZE, img, BIG_SIZE, 24576, 136u * 1024u);   /* to 136 KB */
    check(strcmp(r, "SU OK S 139264") == 0, "S / D up to 136 KB -> '%s'", r);
    check(strcmp(h_begin(BIG_SIZE, crc), "SU OK B 139264") == 0, "re-B resumes after S frames");
    r = send_frame('S', 139264, 8192, 0, 0);       /* 136 KB + 8 KB > the 140.3 KB old image */
    check(strcmp(r, "SU ERR 1 length") == 0, "S beyond the running image -> '%s'", r);
    r = send_frame('S', 139264, 4096, 0, 0);       /* ends at 140 KB: inside */
    check(strcmp(r, "SU OK S 143360") == 0, "S just inside the running image -> '%s'", r);
    r = send_frame('S', 143360, 2048, 0, 0);       /* 140 KB..142 KB: the old image ends at 140.3 KB */
    check(strcmp(r, "SU ERR 1 length") == 0, "S across the running image's end -> '%s'", r);
    r = h_data(img, BIG_SIZE, 143360, BIG_SIZE);
    check(strncmp(r, "SU OK D", 7) == 0, "D after an S past the running image -> '%s'", r);
    r = h_end(KEY, h_nonce, img, BIG_SIZE, crc);
    check(strcmp(r, "SU DONE") == 0, "mixed S / D: E -> '%s'", r);
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED && flash_is(img, BIG_SIZE), "mixed S / D: installed");
    free(img);

    /* S at 0 runs chunk 0's vector check on the copied bytes: the running
     * image's reset vector (0x160) lies outside a 256-byte new image. An S
     * past the new image's end is "length". */
    fresh_board(old, OLD_SIZE);
    device_boot_app(OLD_SIZE);
    h_connect(KEY);
    check(strcmp(h_begin(256, su_crc32(0, old, 256)), "SU OK B 0") == 0, "begin 256");
    check(strcmp(send_frame('S', 0, 256, 0, 0), "SU ERR 4 vectors") == 0, "S at 0: vector check on the copied bytes");
    check(strcmp(h_begin(4196, su_crc32(0, old, 4196)), "SU OK B 0") == 0, "begin 4196");
    check(strcmp(send_frame('S', 0, 8192, 0, 0), "SU ERR 1 length") == 0, "S past the new image's end");
    check(strcmp(send_frame('S', 0, 4196, 0, 0), "SU OK S 4196") == 0, "S at 0 to the end (short final run)");

    /* 4. The running image isn't what the host diffed against: E says "mac",
     * nothing is installed; a full transfer then works. */
    uint8_t *other = make_image(OLD_SIZE, 2);       /* what the host thinks is running */
    img = edit_of(other, OLD_SIZE, BIG_SIZE, 30u * 1024u);
    for (uint32_t i = 0; i < 4096u; i++) img[60u * 1024u + i] = old[60u * 1024u + i];  /* some chunks match the real one */
    fresh_board(old, OLD_SIZE);
    device_boot_app(OLD_SIZE);
    r = h_delta_update(other, OLD_SIZE, img, BIG_SIZE);
    check(strcmp(r, "SU ERR 13 mac") == 0, "wrong base: E -> '%s'", r);
    su_ota_state_t st;
    su_ota_scan(&sim_io, &st);
    check(!st.pending && su_ota_boot(&sim_io) == SU_OTA_BOOT_NONE && flash_is(old, OLD_SIZE), "wrong base: nothing installed");
    check(strcmp(h_begin(BIG_SIZE, su_crc32(0, img, BIG_SIZE)), "SU OK B 0") == 0, "wrong base: transfer forgotten");
    device_boot_app(OLD_SIZE);
    r = h_update(img, BIG_SIZE);
    check(strcmp(r, "SU DONE") == 0, "wrong base: full transfer -> '%s'", r);
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED && flash_is(img, BIG_SIZE), "wrong base: full transfer installed");
    free(img);
    free(other);

    char line[200];
    snprintf(line, sizeof(line), "delta (S): 1-chunk edit = %d S + %d D; short final run; refusals; window S,D,S; dup; S past either end -> ERR 1; wrong base -> ERR 13, full retry", s1, d1);
    ok(line);
    free(old);
}

/* ============================================================
 * Kept pages (whole-page S frames, docs §3.1 / §8.2)
 * ============================================================ */

static long erases_snapshot[128];
static void erases_mark(void) { memcpy(erases_snapshot, page_erases, sizeof(page_erases)); }
static long erases_since(uint32_t page) { return page_erases[page] - erases_snapshot[page]; }

/* A kept-mostly delta update of `img` beside `old`, stopped before the reset.
 * Returns the E reply; *keep gets the recorded bitmap. */
static const char *keep_staged(const uint8_t *old, uint32_t old_size, const uint8_t *img, uint32_t size, uint32_t keep[2])
{
    fresh_board(old, old_size);
    device_boot_app(old_size);
    const char *r = h_delta_update(old, old_size, img, size);
    su_ota_state_t s;
    su_ota_scan(&sim_io, &s);
    keep[0] = s.keep[0];
    keep[1] = s.keep[1];
    return r;
}

static int bit(const uint32_t keep[2], uint32_t p) { return (int)((keep[p >> 5] >> (p & 31u)) & 1u); }

static void test_keep(void)
{
    uint8_t *old = make_image(OLD_SIZE, 1);          /* 140.3 KB: 18 pages */
    uint32_t keep[2];
    int second;
    char want[96];

    /* 1. One small edit in page 10, mid-page: that page mixes S and D and is
     * staged whole; the other 17 are kept: never staged, never erased. */
    uint8_t *img = edit_of(old, OLD_SIZE, OLD_SIZE, 10u * SU_OTA_PAGE + 3000u);
    fresh_board(old, OLD_SIZE);
    device_boot_app(OLD_SIZE);
    erases_mark();
    const char *r = h_delta_update(old, OLD_SIZE, img, OLD_SIZE);
    check(strcmp(r, "SU DONE") == 0, "keep: E -> '%s'", r);
    int s1 = n_s, d1 = n_d;
    su_ota_state_t s;
    su_ota_scan(&sim_io, &s);
    uint32_t pages = (OLD_SIZE + SU_OTA_PAGE - 1u) / SU_OTA_PAGE;
    uint32_t want_keep = ((1u << pages) - 1u) & ~(1u << 10);   /* the last, partial page too */
    check(s.keep[0] == want_keep && s.keep[1] == 0, "keep: bitmap %08x %08x, want %08x", s.keep[0], s.keep[1], want_keep);
    long staged_erases = 0;
    for (uint32_t p = 0; p < 128; p++) staged_erases += erases_since(p);
    check(staged_erases == 1, "keep: %ld staging erases during the transfer, want 1", staged_erases);
    erases_mark();
    long o0 = ops;
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED && flash_is(img, OLD_SIZE), "keep: installed");
    long install_ops = ops - o0;
    for (uint32_t p = 0; p < pages; p++)
        check(erases_since(p) == (p == 10u ? 1 : 0), "keep: page %u erased %ld times by the install", p, erases_since(p));
    device_boot_app(OLD_SIZE);
    snprintf(want, sizeof(want), "SU READY 3 492 1024 2048 0 %u 1 %x %x delta 1", su_ota_limit(OLD_SIZE), KEY_ID,
             su_crc32(0, img, OLD_SIZE));
    check(strcmp(send_frame('Q', 0, 0, 0, 0), want) == 0, "keep: READY names the new image");
    free(img);

    /* 2. Shrinks, the last page partial and unchanged: kept (it ends exactly
     * at the new size, inside the old image); the old bytes past the new
     * end stay in that page, outside the image. */
    uint32_t shrunk = 13u * SU_OTA_PAGE + 1000u;
    img = edit_of(old, OLD_SIZE, shrunk, 3u * SU_OTA_PAGE + 100u);
    r = keep_staged(old, OLD_SIZE, img, shrunk, keep);
    check(strcmp(r, "SU DONE") == 0 && bit(keep, 13) && !bit(keep, 3) && bit(keep, 0), "partial last page: '%s' keep %08x", r, keep[0]);
    erases_mark();
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED && flash_is(img, shrunk), "partial last page: installed");
    check(erases_since(13) == 0 && memcmp(fl + shrunk, old + shrunk, 500) == 0, "partial last page: never erased");
    free(img);

    /* 3. Page 0 changes (a new interrupt handler, say), the rest kept: page 0
     * is staged and written last as before. */
    img = edit_of(old, OLD_SIZE, OLD_SIZE, 0x200u);
    r = keep_staged(old, OLD_SIZE, img, OLD_SIZE, keep);
    check(strcmp(r, "SU DONE") == 0 && !bit(keep, 0) && bit(keep, 1), "page 0 changed: '%s' keep %08x", r, keep[0]);
    erases_mark();
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED && flash_is(img, OLD_SIZE) && erases_since(0) == 1 && erases_since(1) == 0,
          "page 0 changed: installed, only page 0 erased");
    free(img);

    /* 4. The old plan (8 KB runs from 0, across page boundaries after the
     * first D) still works: its runs before the edit happen to be whole
     * pages (kept), the rest is copied into staging. */
    img = edit_of(old, OLD_SIZE, BIG_SIZE, 10u * SU_OTA_PAGE + 3000u);
    plan_pages = 0;
    r = keep_staged(old, OLD_SIZE, img, BIG_SIZE, keep);
    plan_pages = 1;
    check(strcmp(r, "SU DONE") == 0 && keep[0] == 0x3FFu && keep[1] == 0, "unaligned plan: '%s', keep %08x", r, keep[0]);
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED && flash_is(img, BIG_SIZE), "unaligned plan: installed");
    free(img);

    /* 5. A staged page goes bad between E and the boot: the copier's image
     * CRC (staged + kept) catches it before anything is erased. */
    img = edit_of(old, OLD_SIZE, OLD_SIZE, 10u * SU_OTA_PAGE + 3000u);
    r = keep_staged(old, OLD_SIZE, img, OLD_SIZE, keep);
    su_ota_scan(&sim_io, &s);
    fl[s.stage - FL_BASE + 10u * SU_OTA_PAGE + 5u] ^= 0x40;
    erases_mark();
    check(su_ota_boot(&sim_io) == SU_OTA_BOOT_ABANDONED && flash_is(old, OLD_SIZE) && erases_since(0) == 0, "bad staging: abandoned, old app intact");

    /* 6. The guard covers kept pages: a reflash that changes only a kept
     * page, or only the staged one, is abandoned. */
    uint8_t *swd = malloc(OLD_SIZE);
    for (int which = 0; which < 2; which++) {
        keep_staged(old, OLD_SIZE, img, OLD_SIZE, keep);
        memcpy(swd, old, OLD_SIZE);
        swd[(which ? 10u : 4u) * SU_OTA_PAGE + 77u] ^= 1u;
        studio_swd_flash(swd, OLD_SIZE, 0, 0);
        check(su_ota_boot(&sim_io) == SU_OTA_BOOT_ABANDONED && flash_is(swd, OLD_SIZE),
              "guard: reflash changing a %s page abandoned", which ? "staged" : "kept");
    }
    free(swd);

    /* 7. Wrong base with kept pages: ERR 13, nothing kept or installed. */
    uint8_t *other = make_image(OLD_SIZE, 2);
    memcpy(other + 5u * SU_OTA_PAGE, old + 5u * SU_OTA_PAGE, 4u * SU_OTA_PAGE);   /* some pages match */
    free(img);
    img = edit_of(other, OLD_SIZE, OLD_SIZE, 10u * SU_OTA_PAGE + 3000u);
    fresh_board(old, OLD_SIZE);
    device_boot_app(OLD_SIZE);
    r = h_delta_update(other, OLD_SIZE, img, OLD_SIZE);
    check(strcmp(r, "SU ERR 13 mac") == 0, "keep, wrong base -> '%s'", r);
    check(su_ota_boot(&sim_io) == SU_OTA_BOOT_NONE && flash_is(old, OLD_SIZE), "keep, wrong base: nothing installed");
    free(other);
    free(img);

    char line[200];
    snprintf(line, sizeof(line), "kept pages: 1 edit = %d S + %d D, 1 page staged + rewritten (%ld install ops); partial last page; page 0 changed; old plan; bad staging; guard; wrong base",
             s1, d1, install_ops);
    ok(line);
    free(old);
}

/* Power cut at every (or every step-th) flash op of a kept-mostly install:
 * the next boot must finish it, a kept page is never erased, and nothing is
 * unbootable unless page 0 itself is rewritten. */
static void sweep_keep(uint32_t old_size, uint32_t size, uint32_t edit_at, long step, const char *what)
{
    uint8_t *old = make_image(old_size, 11);
    uint8_t *img = edit_of(old, old_size, size, edit_at);
    uint32_t keep[2];
    const char *r = keep_staged(old, old_size, img, size, keep);
    check(strcmp(r, "SU DONE") == 0, "%s: staged '%s'", what, r);
    static uint8_t snap[FL_SIZE], snap_prog[FL_QWS], snap_torn[FL_QWS];
    memcpy(snap, fl, FL_SIZE);
    memcpy(snap_prog, qw_prog, FL_QWS);
    memcpy(snap_torn, qw_torn, FL_QWS);
    long start = ops;
    int second;
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED, "%s: clean install", what);
    long total = ops - start;

    int bricked = 0, completed = 0;
    long kept_erased = 0;
    for (long k = 1; k <= total; k += step) {
        memcpy(fl, snap, FL_SIZE);
        memcpy(qw_prog, snap_prog, FL_QWS);
        memcpy(qw_torn, snap_torn, FL_QWS);
        erases_mark();
        cut_at = ops + k;
        if (setjmp(cut_env) == 0) {
            (void)boot_until_settled(&second);
            cut_at = -1;
        }
        if (qw_torn[0] || !qw_prog[0]) {
            bricked++;
            continue;
        }
        int b = boot_until_settled(&second);
        int fine = flash_is(img, size) && (b == SU_OTA_BOOT_INSTALLED || b == SU_OTA_BOOT_MARKED || b == SU_OTA_BOOT_NONE);
        check(fine, "%s: cut at op %ld of %ld: boot %d", what, k, total, b);
        for (uint32_t p = 0; p < 64; p++)
            if (bit(keep, p)) kept_erased += erases_since(p);
        su_ota_state_t s;
        su_ota_scan(&sim_io, &s);
        check(!s.pending, "%s: cut at op %ld: still pending", what, k);
        if (fine) completed++;
    }
    check(kept_erased == 0, "%s: kept pages erased %ld times", what, kept_erased);
    if (bit(keep, 0)) check(bricked == 0, "%s: %d unbootable cuts with page 0 kept", what, bricked);
    char line[200];
    snprintf(line, sizeof(line), "%s: cut at %s%ld of %ld install ops -> %d installed, %d unrecoverable, kept pages never erased",
             what, step > 1 ? "every " : "each ", step > 1 ? step : total, total, completed, bricked);
    ok(line);
    free(old);
    free(img);
}

/* Studio's recovery rules (docs §2): after ERR 7 (and the ERR 6 a pipelined
 * frame then gets) it re-sends B and resumes at the offset B returns; after a
 * link drop around E it reads READY's image CRC, and may send X, which never
 * touches flash or the marker. */
static void test_studio_rules(void)
{
    uint8_t *old = make_image(OLD_SIZE, 71);
    uint8_t *img = make_image(SMALL_SIZE, 72);
    uint32_t crc = su_crc32(0, img, SMALL_SIZE);
    fresh_board(old, OLD_SIZE);
    device_boot_app(OLD_SIZE);
    check(strcmp(h_connect(KEY), "SU OK A") == 0, "auth");
    check(strcmp(h_begin(SMALL_SIZE, crc), "SU OK B 0") == 0, "begin");
    h_data(img, SMALL_SIZE, 0, 6u * 2048u);
    /* Two frames in flight: the first arrives corrupted, the second out of order. */
    const char *r1 = send_frame('D', 6u * 2048u, su_crc32(0, img + 6u * 2048u, 2048) ^ 1u, img + 6u * 2048u, 2048);
    const char *r1c = strcmp(r1, "SU ERR 7 crc") == 0 ? "ok" : r1;
    check(strcmp(r1c, "ok") == 0, "corrupt chunk -> '%s'", r1);
    const char *r2 = send_frame('D', 7u * 2048u, su_crc32(0, img + 7u * 2048u, 2048), img + 7u * 2048u, 2048);
    check(strcmp(r2, "SU ERR 6 order") == 0, "pipelined next chunk -> '%s'", r2);
    const char *r3 = h_begin(SMALL_SIZE, crc);
    check(strcmp(r3, "SU OK B 12288") == 0, "re-B after ERR 7 -> '%s'", r3);
    check(strncmp(h_data(img, SMALL_SIZE, 6u * 2048u, SMALL_SIZE), "SU OK D", 7) == 0, "resume after re-B");

    /* E processed, its DONE lost, link gone: frames until the reset are
     * dropped, then the copier installs and the new app answers. */
    int before = n_replies;
    check(strcmp(h_end(KEY, h_nonce, img, SMALL_SIZE, crc), "SU DONE") == 0, "E");
    before = n_replies;
    su_core_link_reset();
    send_frame('Q', 0, 0, 0, 0);
    send_frame('X', 0, 0, 0, 0);
    check(n_replies == before, "frames after DONE (before the reset) are ignored");
    su_ota_state_t s;
    su_ota_scan(&sim_io, &s);
    check(s.pending, "marker still pending after X before the reset");
    int second;
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED, "installed");
    device_boot_app(SMALL_SIZE);
    char want[96];
    snprintf(want, sizeof(want), "SU READY 3 492 1024 2048 0 %u 1 %x %x delta 1", su_ota_limit(SMALL_SIZE), KEY_ID, crc);
    check(strcmp(send_frame('Q', 0, 0, 0, 0), want) == 0, "READY names the new image");
    /* B on the new app starts at 0 (nothing in progress); X there is harmless. */
    check(strcmp(h_connect(KEY), "SU OK A") == 0, "auth on the new app");
    check(strcmp(h_begin(SMALL_SIZE, crc), "SU OK B 0") == 0, "B on the new app -> 0");
    long ops0 = ops;
    check(strcmp(send_frame('X', 0, 0, 0, 0), "SU OK X") == 0 && ops == ops0, "X: no flash operation");
    check(su_ota_boot(&sim_io) == SU_OTA_BOOT_NONE && flash_is(img, SMALL_SIZE), "X after an install changes nothing");
    /* A flash error mid-page forgets the transfer: the retry starts at 0. */
    fresh_board(old, OLD_SIZE);
    device_boot_app(OLD_SIZE);
    h_connect(KEY);
    h_begin(SMALL_SIZE, crc);
    h_data(img, SMALL_SIZE, 0, 5u * 2048u);
    uint32_t st;
    su_ota_fit(OLD_SIZE, SMALL_SIZE, &st);
    qw_prog[(st + 5u * 2048u + 32u) / 16u] = 1;           /* this quad-word won't program */
    const char *rf = send_frame('D', 5u * 2048u, su_crc32(0, img + 5u * 2048u, 2048), img + 5u * 2048u, 2048);
    check(strcmp(rf, "SU ERR 5 flash") == 0, "flash error -> '%s'", rf);
    check(strcmp(h_begin(SMALL_SIZE, crc), "SU OK B 0") == 0, "after a flash error B restarts at 0");
    check(strncmp(h_data(img, SMALL_SIZE, 0, SMALL_SIZE), "SU OK D", 7) == 0, "retry re-erases and completes");
    check(strcmp(h_end(KEY, h_nonce, img, SMALL_SIZE, crc), "SU DONE") == 0, "retry: E");
    check(boot_until_settled(&second) == SU_OTA_BOOT_INSTALLED && flash_is(img, SMALL_SIZE), "retry: installed");
    ok("Studio's rules: ERR 7 + pipelined ERR 6 -> re-B resumes; ERR 5 -> restart at 0; after E: frames ignored, READY's CRC confirms, X harmless");
    free(old);
    free(img);
}

static void test_compaction(void)
{
    uint8_t *a = make_image(12u * 1024u, 51);
    fresh_board(a, 12u * 1024u);
    uint32_t nvm = nvm_sum();
    uint32_t cur = 12u * 1024u;
    int compactions = 0;
    for (int i = 0; i < 200; i++) {
        uint32_t size = (i & 1) ? 10u * 1024u + 16u * (uint32_t)i : 12u * 1024u;
        uint8_t *img = make_image(size, 100u + (uint32_t)i);
        su_ota_state_t before;
        su_ota_scan(&sim_io, &before);
        device_boot_app(cur);
        const char *r = h_update(img, size);
        su_ota_state_t after;
        su_ota_scan(&sim_io, &after);
        if (after.log_end < before.log_end) compactions++;
        int second;
        int b = boot_until_settled(&second);
        check(strcmp(r, "SU DONE") == 0 && b == SU_OTA_BOOT_INSTALLED && flash_is(img, size), "update %d: %s / boot %d", i, r, b);
        cur = size;
        free(img);
        if (failures) break;
    }
    su_ota_key_t k;
    check(su_ota_key_read(&sim_io, &k) && k.key_id == KEY_ID, "key survives compaction");
    check(nvm_sum() == nvm, "pages 125-127 untouched by 200 updates");
    char line[100];
    snprintf(line, sizeof(line), "200 updates in a row: marker log compacted %d times, key kept", compactions);
    check(compactions >= 1, "compaction never happened");
    ok(line);
    free(a);
}

static void test_torn_marker(void)
{
    uint8_t *old = make_image(OLD_SIZE, 61);
    uint8_t *img = make_image(SMALL_SIZE, 62);
    staged(old, OLD_SIZE, img, SMALL_SIZE);
    /* Install, but tear the DONE record: the next boot sees it installed and
     * writes DONE again. */
    long start = ops;
    int second;
    staged(old, OLD_SIZE, img, SMALL_SIZE);
    long n0 = ops;
    (void)start;
    boot_until_settled(&second);
    long copy_ops = ops - n0;
    staged(old, OLD_SIZE, img, SMALL_SIZE);
    cut_at = ops + copy_ops;             /* the last op: the DONE record */
    if (setjmp(cut_env) == 0) { boot_until_settled(&second); cut_at = -1; }
    int b = su_ota_boot(&sim_io);
    check(b == SU_OTA_BOOT_MARKED && flash_is(img, SMALL_SIZE), "torn DONE: boot %d", b);
    check(su_ota_boot(&sim_io) == SU_OTA_BOOT_NONE, "torn DONE: settled");
    ok("torn DONE record (ECC error on read): next boot re-marks, no re-copy");
    free(old);
    free(img);
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "--vectors") == 0) {
        char auth[33], itag[33], end[33];
        uint32_t crc;
        vectors(auth, &crc, itag, end);
        printf("auth_tag  %s\ncrc32     %08x\nimage_tag %s\nend_tag   %s\n", auth, crc, itag, end);
        return 0;
    }
    for (int i = 0; i < 16; i++) next_nonce[i] = (uint8_t)(0xA0 + i);

    test_crypto();
    test_vectors();
    test_full(BIG_SIZE, 2, "full transfer + install, new image bigger than the running app");
    test_full(SMALL_SIZE, 3, "full transfer + install, new image smaller than the running app");
    test_resume();
    test_bad_auth();
    test_refusals();
    sweep_commit();
    sweep_copy(20u * 1024u, 24u * 1024u + 100u, 1, 0, "small images, page 0 differs");
    sweep_copy(20u * 1024u, 24u * 1024u + 100u, 1, 1, "small images, page 0 identical");
    sweep_copy(OLD_SIZE, BIG_SIZE, 41, 0, "140 KB -> 150 KB, page 0 differs");
    sweep_copy(OLD_SIZE, BIG_SIZE, 41, 1, "140 KB -> 150 KB, page 0 identical");
    sweep_copy(OLD_SIZE, SMALL_SIZE, 1, 1, "140 KB -> 60 KB, page 0 identical, every op");
    sweep_transfer();
    test_torn_marker();
    test_studio_rules();
    test_swd();
    test_guard();
    test_delta();
    test_keep();
    sweep_keep(20u * 1024u, 20u * 1024u, 9000u, 1, "keep-mostly, 3 pages, edit in page 1");
    sweep_keep(20u * 1024u, 20u * 1024u, 100u, 1, "keep-mostly, 3 pages, edit in page 0");
    sweep_keep(OLD_SIZE, OLD_SIZE, 10u * SU_OTA_PAGE + 3000u, 1, "keep-mostly, 140 KB, edit in page 10");
    sweep_keep(OLD_SIZE, BIG_SIZE, 10u * SU_OTA_PAGE + 3000u, 7, "keep-mostly, 140 -> 150 KB");
    test_compaction();

    if (failures) {
        printf("%d FAILED\n", failures);
        return 1;
    }
    printf("all passed\n");
    return 0;
}
