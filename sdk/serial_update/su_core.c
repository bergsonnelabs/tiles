/**
 * su_core.c — Serial update protocol state machine (hardware-free)
 *
 * No libc: this runs from SRAM inside the flasher blob, which links with
 * -nostdlib, so every copy / compare / format is a plain loop. See su_core.h
 * for the safety order and docs/serial-update-protocol.md for the wire format.
 *
 * Built with SU_STAGING (sdk/ble/ble_update.c, Core.ST.W5) the same state
 * machine receives an image over BLE into a staging area while the app keeps
 * running: no page-0 buffer, nothing erased under the app, authentication
 * (C / A) before BEGIN, an image tag at END, and an install marker instead of
 * a reset into the image. docs/ble-update-protocol.md. Everything staging adds
 * is under #if SU_STAGING, so the flasher blobs build byte-identical.
 */

#include "su_core.h"

static const su_ops_t *O;

static uint8_t  rx[SU_HDR_LEN + SU_MAX_PAYLOAD + 64];
static uint32_t rx_fill;

#if !SU_STAGING
static uint8_t  page0[SU_MAX_PAYLOAD];   /* held until the image verifies */
static uint32_t page0_len;
static int      page0_have;
#endif
static int      page0_erased;

static int      begun;
static uint32_t img_size, img_crc, next_off;
static uint32_t idle_ms;
static int      rebooting;

#if SU_STAGING
static uint32_t stage_off;               /* image offset 0 is written here (from flash_base) */
static uint32_t run_crc;                 /* CRC-32 of the image so far (staged + kept bytes) */
/* The running image CRC's function: the BLE build passes its table-driven one. */
#ifndef SU_CRC32
#define SU_CRC32 su_crc32
#endif
static uint32_t keep_map[2];             /* kept pages: bit p = image page p (<= 64 pages) */
static uint8_t  link_nonce[SU_KEY_BYTES]; /* this link's challenge */
static int      nonce_fresh;             /* not yet spent on an AUTH */
static int      authed;                  /* this link passed AUTH */
#endif

/* ============================================================
 * Small helpers (no libc)
 * ============================================================ */

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint32_t su_crc32(uint32_t crc, const uint8_t *buf, uint32_t len)
{
    uint32_t c = ~crc;
    for (uint32_t i = 0; i < len; i++) {
        c ^= buf[i];
        for (int k = 0; k < 8; k++)
            c = (c >> 1) ^ (0xEDB88320UL & (0UL - (c & 1UL)));
    }
    return ~c;
}

typedef struct {
    char     s[64];
    uint32_t n;
} line_t;

static void put_str(line_t *l, const char *s)
{
    while (*s && l->n < sizeof(l->s) - 2) l->s[l->n++] = *s++;
}

static void put_dec(line_t *l, uint32_t v)
{
    char tmp[10];
    int  k = 0;
    do { tmp[k++] = (char)('0' + v % 10u); v /= 10u; } while (v && k < 10);
    while (k && l->n < sizeof(l->s) - 2) l->s[l->n++] = tmp[--k];
}

static void put_hex(line_t *l, uint32_t v)
{
    static const char hx[] = "0123456789abcdef";
    char tmp[8];
    int  k = 0;
    do { tmp[k++] = hx[v & 0xFu]; v >>= 4; } while (v && k < 8);
    while (k && l->n < sizeof(l->s) - 2) l->s[l->n++] = tmp[--k];
}

#if SU_STAGING
static void put_hex2(line_t *l, uint8_t v)
{
    static const char hx[] = "0123456789abcdef";
    if (l->n < sizeof(l->s) - 3) {
        l->s[l->n++] = hx[v >> 4];
        l->s[l->n++] = hx[v & 0xFu];
    }
}
#endif

static void send_line(line_t *l)
{
    l->s[l->n++] = '\n';
    O->send(l->s, l->n);
}

static void reply_ok(char type, int with_next)
{
    line_t l;
    l.n = 0;
    char t[2] = { type, 0 };
    put_str(&l, "SU OK ");
    put_str(&l, t);
    if (with_next) {
        put_str(&l, " ");
        put_dec(&l, next_off);
    }
    send_line(&l);
}

static void reply_err(uint32_t code, const char *word)
{
    line_t l;
    l.n = 0;
    put_str(&l, "SU ERR ");
    put_dec(&l, code);
    put_str(&l, " ");
    put_str(&l, word);
    send_line(&l);
}

static void reboot(void)
{
    rebooting = 1;
    O->reboot();
}

/* Program `len` bytes at flash offset `off`, padding the tail with 0xFF to the
 * programming unit (SU_PROG_UNIT: 8 bytes L4, 16 H5), then read it back.
 * `skip_head` leaves the first unit for a separate, final write (page 0's SP
 * and reset vector are in it). */
static int write_verify(uint32_t off, const uint8_t *buf, uint32_t len, int skip_head)
{
    uint32_t start   = skip_head ? SU_PROG_UNIT : 0u;
    uint32_t aligned = len & ~(SU_PROG_UNIT - 1u);

    if (aligned > start && O->program(O->flash_base + off + start, buf + start, aligned - start) != 0)
        return -1;
    if (len > aligned) {
        uint8_t tail[SU_PROG_UNIT];
        for (uint32_t i = 0; i < SU_PROG_UNIT; i++) tail[i] = (aligned + i < len) ? buf[aligned + i] : 0xFF;
        if (O->program(O->flash_base + off + aligned, tail, SU_PROG_UNIT) != 0) return -1;
    }
    for (uint32_t i = start; i < len; i++)
        if (O->flash_read[off + i] != buf[i]) return -1;
    return 0;
}

/* The image's first two words must look like a vector table for this chip:
 * an initial SP inside SRAM and a Thumb reset vector inside the image. */
static int vectors_ok(const uint8_t *p, uint32_t len)
{
    if (len < 8) return 0;
    uint32_t sp = le32(p);
    uint32_t rv = le32(p + 4);
    if (sp <= O->sram_start || sp > O->sram_end || (sp & 3u)) return 0;
    if (!(rv & 1u)) return 0;
    rv &= ~1u;
    return rv >= O->flash_base + 8u && rv < O->flash_base + img_size;
}

/* ============================================================
 * Frame handlers
 * ============================================================ */

static void on_query(void)
{
    line_t l;
    l.n = 0;
    put_str(&l, "SU READY ");
    put_dec(&l, SU_PROTOCOL_VERSION);
    put_str(&l, " ");
    put_hex(&l, O->dev_id);
    put_str(&l, " ");
    put_dec(&l, O->flash_size / 1024u);
    put_str(&l, " ");
    put_dec(&l, O->page_size);
    put_str(&l, page0_erased ? " 1" : " 0");
#if SU_PROTOCOL_VERSION >= 2
    put_str(&l, " ");
    put_dec(&l, O->image_limit);
#endif
#if SU_STAGING
    uint32_t key_id = 0;
    int have_key = O->key_info(&key_id);
    put_str(&l, have_key ? " 1 " : " 0 ");
    put_hex(&l, have_key ? key_id : 0u);
    put_str(&l, " ");
    put_hex(&l, O->image_crc());
    put_str(&l, " delta 1");          /* this firmware takes S frames */
#endif
    send_line(&l);
}

#if SU_STAGING
static void forget_nonce(void)
{
    for (uint32_t i = 0; i < SU_KEY_BYTES; i++) link_nonce[i] = 0;
    nonce_fresh = 0;
}

static void on_challenge(void)
{
    authed = 0;
    forget_nonce();
    int rc = O->challenge(link_nonce);
    if (rc == (int)SU_ERR_NOKEY) { reply_err(SU_ERR_NOKEY, "nokey"); return; }
    if (rc != 0)                 { forget_nonce(); reply_err(SU_ERR_RNG, "rng"); return; }
    nonce_fresh = 1;
    line_t l;
    l.n = 0;
    put_str(&l, "SU NONCE ");
    for (uint32_t i = 0; i < SU_KEY_BYTES; i++) put_hex2(&l, link_nonce[i]);
    send_line(&l);
}

static void on_auth(const uint8_t *p, uint32_t len)
{
    if (len != SU_KEY_BYTES) { reply_err(SU_ERR_LENGTH, "length"); return; }
    /* One AUTH per nonce, right or wrong. The nonce itself stays: END's tag
     * is bound to it. */
    int ok = nonce_fresh && O->auth_check(link_nonce, p);
    nonce_fresh = 0;
    if (!ok) {
        authed = 0;
        forget_nonce();
        reply_err(SU_ERR_AUTH, "auth");
        return;
    }
    authed = 1;
    reply_ok(SU_T_AUTH, 0);
}

static int need_auth(void)
{
    if (authed) return 0;
    reply_err(SU_ERR_NOAUTH, "noauth");
    return 1;
}

static void on_begin(uint32_t size, uint32_t crc, const uint8_t *p, uint32_t len)
{
    if (need_auth()) return;
    if (len != 4 || (le32(p) & 0xFFFu) != O->dev_id) { reply_err(SU_ERR_DEVICE, "device"); return; }
    uint32_t so;
    if (size < 8u || size > O->image_limit || size > 64u * O->erase_size || O->stage(size, &so) != 0) {
        reply_err(SU_ERR_SIZE, "size");
        return;
    }
    /* The same image again (a new link, or a host recovering): carry on. */
    if (begun && size == img_size && crc == img_crc && so == stage_off) {
        reply_ok(SU_T_BEGIN, 1);
        return;
    }
    begun     = 1;
    img_size  = size;
    img_crc   = crc;
    next_off  = 0;
    stage_off = so;
    run_crc   = 0;
    keep_map[0] = keep_map[1] = 0;
    O->mac_start(size, crc);
    reply_ok(SU_T_BEGIN, 1);
}

/* One chunk (at most page_size bytes, at a chunk-aligned image offset) into
 * staging, and into the image tag. Staging only: the running app is never
 * touched. Each flash page is erased as its first chunk arrives (chunks divide
 * pages). 0 = written and read back. */
static int stage_chunk(uint32_t off, const uint8_t *p, uint32_t len)
{
    uint32_t at = stage_off + off;
    if (((at % O->erase_size) == 0u && O->erase_page(at / O->erase_size) != 0) ||
        write_verify(at, p, len, 0) != 0)
        return -1;
    /* What was just read back from staging equals p: the CRC of the staged
     * image accumulates here, so E needn't read staging again. */
    run_crc = SU_CRC32(run_crc, p, len);
    O->mac_update(p, len);
    return 0;
}

static void on_data(uint32_t off, uint32_t crc, const uint8_t *p, uint32_t len)
{
    if (need_auth()) return;
    if (!begun)                                  { reply_err(SU_ERR_STATE, "state"); return; }
    if (len == 0 || len > O->page_size)          { reply_err(SU_ERR_LENGTH, "length"); return; }
    if (off < next_off && off + len == next_off) { reply_ok(SU_T_DATA, 1); return; }
    if (off != next_off)                         { reply_err(SU_ERR_ORDER, "order"); return; }
    if (off + len > img_size)                    { reply_err(SU_ERR_SIZE, "size"); return; }
    if (len != O->page_size && off + len != img_size) { reply_err(SU_ERR_LENGTH, "length"); return; }
    if (su_crc32(0, p, len) != crc)              { reply_err(SU_ERR_CRC, "crc"); return; }
    if (off == 0 && !vectors_ok(p, len))         { reply_err(SU_ERR_VECTORS, "vectors"); return; }

    if (stage_chunk(off, p, len) != 0) {
        /* The page may be half-programmed: forget the transfer, so the
         * retry starts at 0 and erases every staging page again. */
        begun = 0;
        reply_err(SU_ERR_FLASH, "flash");
        return;
    }
    next_off = off + len;
    reply_ok(SU_T_DATA, 1);
}

/* SAME (delta): image bytes [off, off + len) equal the running image's at the
 * same offsets. They are copied from the running image (flash_read) and go
 * down exactly the D path, chunk by chunk: staging erase / program / read
 * back, the image tag. So E's checks cover them like any received byte, and
 * a running image that isn't the one the host diffed against fails E. */
static void on_same(uint32_t off, uint32_t len, uint32_t payload_len)
{
    if (need_auth()) return;
    if (!begun)                                  { reply_err(SU_ERR_STATE, "state"); return; }
    if (payload_len != 0 || len == 0 || len > SU_SAME_MAX) { reply_err(SU_ERR_LENGTH, "length"); return; }
    if (off < next_off && off + len == next_off) { reply_ok(SU_T_SAME, 1); return; }
    if (off != next_off)                         { reply_err(SU_ERR_ORDER, "order"); return; }
    /* Every malformed run is "length" (a host bug: stop), whichever image's
     * end it passes; an offset other than the next one is "order", as for D. */
    if (off + len > img_size || off + len > O->run_size ||
        ((len % O->page_size) != 0u && off + len != img_size)) { reply_err(SU_ERR_LENGTH, "length"); return; }
    /* The source must lie below staging (stage() puts staging above the
     * running image): never copy staging into staging. */
    if (O->run_size > stage_off)                 { begun = 0; reply_err(SU_ERR_FLASH, "flash"); return; }
    const uint8_t *src = O->flash_read;
    if (off == 0 && !vectors_ok(src, len < O->page_size ? len : O->page_size)) {
        reply_err(SU_ERR_VECTORS, "vectors");
        return;
    }
    /* Exactly one flash page (or the image's partial last page, starting on
     * a page): keep it. Read only, for the CRC and the tag; the copier
     * leaves the page as it is. Anything else is copied into staging. */
    int keep = (off % O->erase_size) == 0u && (len == O->erase_size || off + len == img_size);
    for (uint32_t o = off; o < off + len; o += O->page_size) {
        uint32_t n = off + len - o < O->page_size ? off + len - o : O->page_size;
        if (keep) {
            run_crc = SU_CRC32(run_crc, src + o, n);
            O->mac_update(src + o, n);
        } else if (stage_chunk(o, src + o, n) != 0) {
            begun = 0;
            reply_err(SU_ERR_FLASH, "flash");
            return;
        }
        if (O->feed) O->feed();
    }
    if (keep) {
        uint32_t pg = off / O->erase_size;
        keep_map[pg >> 5] |= 1UL << (pg & 31u);
    }
    next_off = off + len;
    reply_ok(SU_T_SAME, 1);
}

static void on_end(const uint8_t *p, uint32_t len)
{
    if (need_auth()) return;
    if (len != SU_KEY_BYTES)                 { reply_err(SU_ERR_LENGTH, "length"); return; }
    if (!begun || next_off != img_size)      { reply_err(SU_ERR_STATE, "state"); return; }

    /* What is in flash, not what was received. A failure here forgets the
     * transfer: the host starts over. */
    begun = 0;
    /* The tag first: it covers what was received, copied and kept, so a
     * running image other than the one the host diffed against shows as
     * "mac" (docs §3.1). Then the CRC, accumulated as each chunk was read
     * back from staging (or read from a kept page): nothing is re-read here;
     * the copier checks staging + kept pages again before it starts. */
    if (!O->mac_check(link_nonce, p))        { reply_err(SU_ERR_MAC, "mac"); return; }
    if (run_crc != img_crc)                  { reply_err(SU_ERR_IMAGE_CRC, "image-crc"); return; }
    if (O->commit(stage_off, img_size, img_crc, keep_map) != 0) { reply_err(SU_ERR_FLASH, "flash"); return; }

    line_t l;
    l.n = 0;
    put_str(&l, "SU DONE");
    send_line(&l);
    reboot();
}

void su_core_link_reset(void)
{
    authed = 0;
    forget_nonce();
    rx_fill = 0;
}
#else /* !SU_STAGING */

static void on_begin(uint32_t size, uint32_t crc, const uint8_t *p, uint32_t len)
{
    if (len != 4 || (le32(p) & 0xFFFu) != O->dev_id) { reply_err(SU_ERR_DEVICE, "device"); return; }
    /* image_limit, not flash_size: the pages above it hold core_nvm's data,
     * which an update must leave alone (an image is erased page by page, so
     * one that ends below them never touches them). */
    if (size < 8u || size > O->image_limit)           { reply_err(SU_ERR_SIZE, "size"); return; }
    begun      = 1;
    img_size   = size;
    img_crc    = crc;
    next_off   = 0;
    page0_have = 0;
    reply_ok(SU_T_BEGIN, 0);
}

static void on_data(uint32_t off, uint32_t crc, const uint8_t *p, uint32_t len)
{
    if (!begun)                                  { reply_err(SU_ERR_STATE, "state"); return; }
    if (len == 0 || len > O->page_size)          { reply_err(SU_ERR_LENGTH, "length"); return; }
    /* A resend of the chunk just written (its OK was lost): acknowledge again. */
    if (off < next_off && off + len == next_off) { reply_ok(SU_T_DATA, 1); return; }
    if (off != next_off)                         { reply_err(SU_ERR_ORDER, "order"); return; }
    if (off + len > img_size)                    { reply_err(SU_ERR_SIZE, "size"); return; }
    if (len != O->page_size && off + len != img_size) { reply_err(SU_ERR_LENGTH, "length"); return; }
    if (su_crc32(0, p, len) != crc)              { reply_err(SU_ERR_CRC, "crc"); return; }

    if (off == 0) {
        /* Checked before anything is erased: a wrong or corrupt image is
         * refused with the old app still intact. */
        if (!vectors_ok(p, len)) { reply_err(SU_ERR_VECTORS, "vectors"); return; }
        if (!page0_erased) {
            if (O->erase_page(0) != 0) { reply_err(SU_ERR_FLASH, "flash"); return; }
            page0_erased = 1;       /* from here on the chip boots the ROM bootloader */
        }
        for (uint32_t i = 0; i < len; i++) page0[i] = p[i];
        page0_len  = len;
        page0_have = 1;
    } else {
        if (!page0_have) { reply_err(SU_ERR_STATE, "state"); return; }
        if (O->erase_page(off / O->page_size) != 0 || write_verify(off, p, len, 0) != 0) {
            reply_err(SU_ERR_FLASH, "flash");
            return;
        }
    }
    next_off = off + len;
    reply_ok(SU_T_DATA, 1);
}

static void on_end(void)
{
    if (!begun || !page0_have || next_off != img_size) { reply_err(SU_ERR_STATE, "state"); return; }

    uint32_t c = su_crc32(0, page0, page0_len);
    c = su_crc32(c, O->flash_read + page0_len, img_size - page0_len);
    if (c != img_crc) { reply_err(SU_ERR_IMAGE_CRC, "image-crc"); return; }

    /* Page 0 body first, then its first programming unit (SP + reset vector)
     * last: a failure anywhere before that final write leaves 0x08000000
     * erased. */
    if (write_verify(0, page0, page0_len, 1) != 0 ||
        O->program(O->flash_base, page0, SU_PROG_UNIT) != 0) {
        reply_err(SU_ERR_FLASH, "flash");
        return;
    }
    for (uint32_t i = 0; i < SU_PROG_UNIT; i++)
        if (O->flash_read[i] != page0[i]) { reply_err(SU_ERR_FLASH, "flash"); return; }

    page0_erased = 0;
    line_t l;
    l.n = 0;
    put_str(&l, "SU DONE");
    send_line(&l);
    reboot();
}
#endif /* SU_STAGING */

static void handle(uint8_t type, uint32_t a0, uint32_t a1, const uint8_t *p, uint32_t len)
{
    idle_ms = 0;
    switch (type) {
    case SU_T_QUERY: on_query();               break;
    case SU_T_BEGIN: on_begin(a0, a1, p, len); break;
    case SU_T_DATA:  on_data(a0, a1, p, len);  break;
#if SU_STAGING
    case SU_T_CHALLENGE: on_challenge();       break;
    case SU_T_AUTH:  on_auth(p, len);          break;
    case SU_T_SAME:  on_same(a0, a1, len);     break;
    case SU_T_END:   on_end(p, len);           break;
    /* The running app stays: abort only forgets the transfer. */
    case SU_T_ABORT: if (!need_auth()) { begun = 0; reply_ok(SU_T_ABORT, 0); } break;
#else
    case SU_T_END:   on_end();                 break;
    case SU_T_ABORT: reply_ok(SU_T_ABORT, 0); reboot(); break;
#endif
    }
}

/* ============================================================
 * Receive path
 * ============================================================ */

static void consume(uint32_t n)
{
    if (n > rx_fill) n = rx_fill;
    for (uint32_t i = n; i < rx_fill; i++) rx[i - n] = rx[i];
    rx_fill -= n;
}

static void parse(void)
{
    while (!rebooting) {
        /* Resync on "SU": anything else before a frame is dropped. */
        uint32_t i = 0;
        while (i < rx_fill && !(rx[i] == SU_SYNC0 && (i + 1 == rx_fill || rx[i + 1] == SU_SYNC1))) i++;
        consume(i);
        if (rx_fill < SU_HDR_LEN) return;

        /* Only a plausible header is a frame. App output that happens to
         * contain "SU" is skipped silently rather than answered. */
        uint32_t len = (uint32_t)rx[4] | ((uint32_t)rx[5] << 8);
        uint8_t  t   = rx[2];
        if ((t != SU_T_QUERY && t != SU_T_BEGIN && t != SU_T_DATA && t != SU_T_END && t != SU_T_ABORT
#if SU_STAGING
             && t != SU_T_CHALLENGE && t != SU_T_AUTH && t != SU_T_SAME
#endif
            ) ||
            rx[3] != 0 || rx[6] != 0 || rx[7] != 0 || len > SU_MAX_PAYLOAD) {
            consume(1);
            continue;
        }
        if (rx_fill < SU_HDR_LEN + len) return;
        handle(rx[2], le32(rx + 8), le32(rx + 12), rx + SU_HDR_LEN, len);
        consume(SU_HDR_LEN + len);
    }
}

void su_core_init(const su_ops_t *ops)
{
    O = ops;
    rx_fill = 0;
#if !SU_STAGING
    page0_len = 0;
    page0_have = 0;
#else
    stage_off = 0;
    run_crc = 0;
    keep_map[0] = keep_map[1] = 0;
    authed = 0;
    forget_nonce();
#endif
    page0_erased = 0;
    begun = 0;
    img_size = img_crc = next_off = 0;
    idle_ms = 0;
    rebooting = 0;
}

uint32_t su_core_rx_space(void)
{
    return (uint32_t)sizeof(rx) - rx_fill;
}

uint32_t su_core_rx(const uint8_t *buf, uint32_t len)
{
    if (rebooting) return len;
    uint32_t n = su_core_rx_space();
    if (len < n) n = len;
    for (uint32_t i = 0; i < n; i++) rx[rx_fill++] = buf[i];
    parse();
    return n;
}

void su_core_tick(uint32_t elapsed_ms)
{
#if SU_STAGING
    /* The app is running and stays running: no idle reset. */
    (void)elapsed_ms;
    return;
#endif
    if (rebooting) return;
    idle_ms += elapsed_ms;
    if (!page0_erased && idle_ms >= SU_IDLE_TIMEOUT_MS) reboot();
}

int su_core_page0_erased(void)
{
    return page0_erased;
}
