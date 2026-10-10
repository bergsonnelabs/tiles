/**
 * su_ota.c — BLE update on Core.ST.W5: reserved page, install marker, copier
 *
 * See su_ota.h and docs/ble-update-protocol.md §7-8. Runs from SRAM on the
 * target (the boot hook copies `.ota_ram` there at every boot): no libc, no
 * calls out of this file except through su_ota_io_t, no struct copies or
 * aggregate initialisers (GCC turns them into memcpy / memset calls: seen at
 * -Os), no division. tools/check_ota_boot.py enforces it on every BLE build.
 */

#include "su_ota.h"

/* ============================================================
 * Helpers
 * ============================================================ */

/* Byte-at-a-time table (1 KB, in .ota_ram with the copier): about 2.5x the
 * nibble version's speed, which the boot-time checks of a whole image need. */
static const uint32_t crc_tab[256] SU_OTA_RAM_DATA = {
    0x00000000UL, 0x77073096UL, 0xEE0E612CUL, 0x990951BAUL, 0x076DC419UL, 0x706AF48FUL,
    0xE963A535UL, 0x9E6495A3UL, 0x0EDB8832UL, 0x79DCB8A4UL, 0xE0D5E91EUL, 0x97D2D988UL,
    0x09B64C2BUL, 0x7EB17CBDUL, 0xE7B82D07UL, 0x90BF1D91UL, 0x1DB71064UL, 0x6AB020F2UL,
    0xF3B97148UL, 0x84BE41DEUL, 0x1ADAD47DUL, 0x6DDDE4EBUL, 0xF4D4B551UL, 0x83D385C7UL,
    0x136C9856UL, 0x646BA8C0UL, 0xFD62F97AUL, 0x8A65C9ECUL, 0x14015C4FUL, 0x63066CD9UL,
    0xFA0F3D63UL, 0x8D080DF5UL, 0x3B6E20C8UL, 0x4C69105EUL, 0xD56041E4UL, 0xA2677172UL,
    0x3C03E4D1UL, 0x4B04D447UL, 0xD20D85FDUL, 0xA50AB56BUL, 0x35B5A8FAUL, 0x42B2986CUL,
    0xDBBBC9D6UL, 0xACBCF940UL, 0x32D86CE3UL, 0x45DF5C75UL, 0xDCD60DCFUL, 0xABD13D59UL,
    0x26D930ACUL, 0x51DE003AUL, 0xC8D75180UL, 0xBFD06116UL, 0x21B4F4B5UL, 0x56B3C423UL,
    0xCFBA9599UL, 0xB8BDA50FUL, 0x2802B89EUL, 0x5F058808UL, 0xC60CD9B2UL, 0xB10BE924UL,
    0x2F6F7C87UL, 0x58684C11UL, 0xC1611DABUL, 0xB6662D3DUL, 0x76DC4190UL, 0x01DB7106UL,
    0x98D220BCUL, 0xEFD5102AUL, 0x71B18589UL, 0x06B6B51FUL, 0x9FBFE4A5UL, 0xE8B8D433UL,
    0x7807C9A2UL, 0x0F00F934UL, 0x9609A88EUL, 0xE10E9818UL, 0x7F6A0DBBUL, 0x086D3D2DUL,
    0x91646C97UL, 0xE6635C01UL, 0x6B6B51F4UL, 0x1C6C6162UL, 0x856530D8UL, 0xF262004EUL,
    0x6C0695EDUL, 0x1B01A57BUL, 0x8208F4C1UL, 0xF50FC457UL, 0x65B0D9C6UL, 0x12B7E950UL,
    0x8BBEB8EAUL, 0xFCB9887CUL, 0x62DD1DDFUL, 0x15DA2D49UL, 0x8CD37CF3UL, 0xFBD44C65UL,
    0x4DB26158UL, 0x3AB551CEUL, 0xA3BC0074UL, 0xD4BB30E2UL, 0x4ADFA541UL, 0x3DD895D7UL,
    0xA4D1C46DUL, 0xD3D6F4FBUL, 0x4369E96AUL, 0x346ED9FCUL, 0xAD678846UL, 0xDA60B8D0UL,
    0x44042D73UL, 0x33031DE5UL, 0xAA0A4C5FUL, 0xDD0D7CC9UL, 0x5005713CUL, 0x270241AAUL,
    0xBE0B1010UL, 0xC90C2086UL, 0x5768B525UL, 0x206F85B3UL, 0xB966D409UL, 0xCE61E49FUL,
    0x5EDEF90EUL, 0x29D9C998UL, 0xB0D09822UL, 0xC7D7A8B4UL, 0x59B33D17UL, 0x2EB40D81UL,
    0xB7BD5C3BUL, 0xC0BA6CADUL, 0xEDB88320UL, 0x9ABFB3B6UL, 0x03B6E20CUL, 0x74B1D29AUL,
    0xEAD54739UL, 0x9DD277AFUL, 0x04DB2615UL, 0x73DC1683UL, 0xE3630B12UL, 0x94643B84UL,
    0x0D6D6A3EUL, 0x7A6A5AA8UL, 0xE40ECF0BUL, 0x9309FF9DUL, 0x0A00AE27UL, 0x7D079EB1UL,
    0xF00F9344UL, 0x8708A3D2UL, 0x1E01F268UL, 0x6906C2FEUL, 0xF762575DUL, 0x806567CBUL,
    0x196C3671UL, 0x6E6B06E7UL, 0xFED41B76UL, 0x89D32BE0UL, 0x10DA7A5AUL, 0x67DD4ACCUL,
    0xF9B9DF6FUL, 0x8EBEEFF9UL, 0x17B7BE43UL, 0x60B08ED5UL, 0xD6D6A3E8UL, 0xA1D1937EUL,
    0x38D8C2C4UL, 0x4FDFF252UL, 0xD1BB67F1UL, 0xA6BC5767UL, 0x3FB506DDUL, 0x48B2364BUL,
    0xD80D2BDAUL, 0xAF0A1B4CUL, 0x36034AF6UL, 0x41047A60UL, 0xDF60EFC3UL, 0xA867DF55UL,
    0x316E8EEFUL, 0x4669BE79UL, 0xCB61B38CUL, 0xBC66831AUL, 0x256FD2A0UL, 0x5268E236UL,
    0xCC0C7795UL, 0xBB0B4703UL, 0x220216B9UL, 0x5505262FUL, 0xC5BA3BBEUL, 0xB2BD0B28UL,
    0x2BB45A92UL, 0x5CB36A04UL, 0xC2D7FFA7UL, 0xB5D0CF31UL, 0x2CD99E8BUL, 0x5BDEAE1DUL,
    0x9B64C2B0UL, 0xEC63F226UL, 0x756AA39CUL, 0x026D930AUL, 0x9C0906A9UL, 0xEB0E363FUL,
    0x72076785UL, 0x05005713UL, 0x95BF4A82UL, 0xE2B87A14UL, 0x7BB12BAEUL, 0x0CB61B38UL,
    0x92D28E9BUL, 0xE5D5BE0DUL, 0x7CDCEFB7UL, 0x0BDBDF21UL, 0x86D3D2D4UL, 0xF1D4E242UL,
    0x68DDB3F8UL, 0x1FDA836EUL, 0x81BE16CDUL, 0xF6B9265BUL, 0x6FB077E1UL, 0x18B74777UL,
    0x88085AE6UL, 0xFF0F6A70UL, 0x66063BCAUL, 0x11010B5CUL, 0x8F659EFFUL, 0xF862AE69UL,
    0x616BFFD3UL, 0x166CCF45UL, 0xA00AE278UL, 0xD70DD2EEUL, 0x4E048354UL, 0x3903B3C2UL,
    0xA7672661UL, 0xD06016F7UL, 0x4969474DUL, 0x3E6E77DBUL, 0xAED16A4AUL, 0xD9D65ADCUL,
    0x40DF0B66UL, 0x37D83BF0UL, 0xA9BCAE53UL, 0xDEBB9EC5UL, 0x47B2CF7FUL, 0x30B5FFE9UL,
    0xBDBDF21CUL, 0xCABAC28AUL, 0x53B39330UL, 0x24B4A3A6UL, 0xBAD03605UL, 0xCDD70693UL,
    0x54DE5729UL, 0x23D967BFUL, 0xB3667A2EUL, 0xC4614AB8UL, 0x5D681B02UL, 0x2A6F2B94UL,
    0xB40BBE37UL, 0xC30C8EA1UL, 0x5A05DF1BUL, 0x2D02EF8DUL,
};

SU_OTA_RAM uint32_t su_ota_crc32(uint32_t crc, const uint8_t *p, uint32_t n)
{
    uint32_t c = ~crc;
    while (n--) c = (c >> 8) ^ crc_tab[(c ^ *p++) & 0xFFu];
    return ~c;
}

static SU_OTA_RAM void feed(const su_ota_io_t *io)
{
    if (io->feed) io->feed(io->ctx);
}

static SU_OTA_RAM int all_ff(const uint32_t *w, uint32_t words)
{
    uint32_t acc = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < words; i++) acc &= w[i];
    return acc == 0xFFFFFFFFUL;
}

static SU_OTA_RAM int words_equal(const uint32_t *a, const uint32_t *b, uint32_t words)
{
    for (uint32_t i = 0; i < words; i++)
        if (a[i] != b[i]) return 0;
    return 1;
}

/* CRC of `len` bytes of flash at `off`, then `pad` bytes of 0xFF. -1 in *ok on
 * an ECC error. */
static SU_OTA_RAM uint32_t crc_flash(const su_ota_io_t *io, uint32_t off, uint32_t len, uint32_t pad, int *ok)
{
    uint32_t buf[16];                      /* 64 bytes */
    uint32_t c = 0;
    while (len) {
        uint32_t n = len < sizeof(buf) ? len : (uint32_t)sizeof(buf);
        if (io->read(io->ctx, off, buf, n) != 0) *ok = 0;
        c = su_ota_crc32(c, (const uint8_t *)buf, n);
        off += n;
        len -= n;
        if ((off & (SU_OTA_PAGE - 1u)) == 0) feed(io);
    }
    for (uint32_t i = 0; i < 16; i++) buf[i] = 0xFFFFFFFFUL;
    while (pad) {
        uint32_t n = pad < sizeof(buf) ? pad : (uint32_t)sizeof(buf);
        c = su_ota_crc32(c, (const uint8_t *)buf, n);
        pad -= n;
    }
    return c;
}

/* ============================================================
 * Key record (docs §7.2)
 * ============================================================ */

SU_OTA_RAM int su_ota_key_read(const su_ota_io_t *io, su_ota_key_t *k)
{
    uint32_t r[8];
    if (io->read(io->ctx, SU_OTA_RES_OFF + SU_OTA_KEY_OFF, r, sizeof(r)) != 0) return 0;
    if (r[0] != SU_OTA_KEY_MAGIC || r[1] == 0u || r[1] == 0xFFFFFFFFUL) return 0;
    uint32_t c = su_ota_crc32(0, (const uint8_t *)&r[0], 8);
    c = su_ota_crc32(c, (const uint8_t *)&r[4], 16);
    if (c != r[2]) return 0;
    k->key_id = r[1];
    const uint8_t *kb = (const uint8_t *)&r[4];
    for (int i = 0; i < 16; i++) k->key[i] = kb[i];
    for (int i = 0; i < 8; i++) r[i] = 0;
    return 1;
}

/* ============================================================
 * Marker log (docs §7.1)
 * ============================================================ */

/* COPY-B[3]: CRC-32 of COPY-A ‖ KEEP ‖ COPY-B[0..11]. */
static SU_OTA_RAM uint32_t copy_check(const uint32_t a[4], const uint32_t k[4], const uint32_t b[4])
{
    uint32_t c = su_ota_crc32(0, (const uint8_t *)a, 16);
    c = su_ota_crc32(c, (const uint8_t *)k, 16);
    return su_ota_crc32(c, (const uint8_t *)b, 12);
}

SU_OTA_RAM void su_ota_scan(const su_ota_io_t *io, su_ota_state_t *s)
{
    /* The two records before this one, and whether each read back clean. */
    uint32_t p1[4], p2[4];
    int      p1_ok = 0, p2_ok = 0;
    for (int i = 0; i < 4; i++) p1[i] = p2[i] = 0;

    s->pending = s->started = 0;
    s->stage = s->size = s->crc = s->old_size = s->old_crc = s->check = 0;
    s->keep[0] = s->keep[1] = 0;
    s->log_end = SU_OTA_LOG_OFF;

    for (uint32_t off = SU_OTA_LOG_OFF; off < SU_OTA_LOG_END; off += SU_OTA_QW) {
        uint32_t r[4];
        int ok = io->read(io->ctx, SU_OTA_RES_OFF + off, r, sizeof(r)) == 0;
        if (ok && all_ff(r, 4)) ok = 0;    /* free: breaks a group like a torn record */
        else s->log_end = off + SU_OTA_QW; /* used (valid, torn or unknown) */

        /* COPY-A, KEEP, COPY-B in consecutive slots, COPY-B's check over all
         * three. A v1 "COPB" or v2 "COP2" COPY-B is not one here: an install
         * it describes reads as not pending (docs §7.1). */
        if (ok && r[0] == SU_OTA_COPB && p2_ok && p2[0] == SU_OTA_COPA && p1_ok && p1[0] == SU_OTA_KEEP &&
            copy_check(p2, p1, r) == r[3]) {
            s->pending  = 1;
            s->started  = 0;
            s->stage    = p2[1];
            s->size     = p2[2];
            s->crc      = p2[3];
            s->keep[0]  = p1[1];
            s->keep[1]  = p1[2];
            s->old_size = r[1];
            s->old_crc  = r[2];
            s->check    = r[3];
        } else if (ok && r[0] == SU_OTA_STRT && r[3] == ~r[1] && s->pending && r[1] == s->check) {
            s->started = 1;
        } else if (ok && r[0] == SU_OTA_DONE && r[3] == ~r[1] && s->pending && r[1] == s->check) {
            s->pending = s->started = 0;
        }
        for (int i = 0; i < 4; i++) { p2[i] = p1[i]; p1[i] = r[i]; }
        p2_ok = p1_ok;
        p1_ok = ok;
    }
    s->free_slots = (SU_OTA_LOG_END - s->log_end) / SU_OTA_QW;
}

/* Erase page 124 and put the key record back (if there was a valid one).
 * Not power-safe for the key: a cut in the ~2 ms between the erase and the
 * rewrite loses it (BLE updates then answer "nokey" until the next SWD
 * flash). Happens about once every 120 installs. */
static SU_OTA_RAM int compact(const su_ota_io_t *io)
{
    uint32_t r[8];
    int keep = io->read(io->ctx, SU_OTA_RES_OFF + SU_OTA_KEY_OFF, r, sizeof(r)) == 0 && r[0] == SU_OTA_KEY_MAGIC;
    if (keep) {
        su_ota_key_t k;
        keep = su_ota_key_read(io, &k);
        for (int i = 0; i < 16; i++) k.key[i] = 0;
    }
    if (io->erase(io->ctx, SU_OTA_RES_PAGE) != 0) return -1;
    if (keep) {
        uint32_t back[8];
        if (io->program(io->ctx, SU_OTA_RES_OFF + SU_OTA_KEY_OFF, r, sizeof(r)) != 0) return -1;
        if (io->read(io->ctx, SU_OTA_RES_OFF + SU_OTA_KEY_OFF, back, sizeof(back)) != 0 ||
            !words_equal(r, back, 8)) return -1;
        for (int i = 0; i < 8; i++) r[i] = back[i] = 0;
    }
    return 0;
}

/* Append `n` records (n x 16 bytes, all in one go: COPY-A and COPY-B must be
 * adjacent) after the last used slot, skipping a slot that won't program. */
static SU_OTA_RAM int append(const su_ota_io_t *io, const uint32_t *rec, uint32_t n)
{
    su_ota_state_t s;
    su_ota_scan(io, &s);
    uint32_t off = s.log_end;
    for (int tries = 0; tries < 4; tries++) {
        if (off + n * SU_OTA_QW > SU_OTA_LOG_END) return -1;
        int ok = 1;
        for (uint32_t i = 0; i < n && ok; i++) {
            uint32_t back[4];
            uint32_t a = SU_OTA_RES_OFF + off + i * SU_OTA_QW;
            if (io->program(io->ctx, a, rec + 4u * i, SU_OTA_QW) != 0 ||
                io->read(io->ctx, a, back, sizeof(back)) != 0 || !words_equal(back, rec + 4u * i, 4))
                ok = 0;
        }
        if (ok) return 0;
        /* Something didn't take (a slot torn by an earlier cut that reads
         * blank, say): rescan and go after everything written so far. */
        su_ota_scan(io, &s);
        off = s.log_end;
    }
    return -1;
}

/* START or DONE: [1] the COPY-B check it refers to, [3] its complement. */
static SU_OTA_RAM int append_mark(const su_ota_io_t *io, uint32_t magic, uint32_t check, uint32_t result)
{
    uint32_t rec[4];
    rec[0] = magic;
    rec[1] = check;
    rec[2] = result;
    rec[3] = ~check;
    return append(io, rec, 1);
}

static SU_OTA_RAM int append_done(const su_ota_io_t *io, uint32_t check, uint32_t result)
{
    if (append_mark(io, SU_OTA_DONE, check, result) == 0) return 0;
    /* Out of room: compacting drops the COPY too, which is as good as DONE. */
    return compact(io);
}

/* ============================================================
 * Staging
 * ============================================================ */

static SU_OTA_RAM uint32_t page_ceil(uint32_t v)
{
    return (v + (SU_OTA_PAGE - 1u)) & ~(SU_OTA_PAGE - 1u);
}

/* Image page p is kept (bit p of keep[p / 32]). */
static SU_OTA_RAM int kept(const uint32_t keep[2], uint32_t p)
{
    if (p >= SU_OTA_MAX_PAGES) return 0;
    return (int)((keep[p >> 5] >> (p & 31u)) & 1u);
}

/* Where image page p ends: the next page, or the image's end. */
static SU_OTA_RAM uint32_t page_end(uint32_t p, uint32_t size)
{
    uint32_t e = (p + 1u) * SU_OTA_PAGE;
    return e < size ? e : size;
}

SU_OTA_RAM int su_ota_fit(uint32_t app_end, uint32_t size, uint32_t *stage)
{
    if (size == 0u || size > SU_OTA_APP_LIMIT || app_end > SU_OTA_APP_LIMIT) return -1;
    /* Above the running app (live code) and above the copy's destination
     * [0, size), so a resumed copy re-reads intact data. */
    uint32_t s = page_ceil(app_end > size ? app_end : size);
    if (s > SU_OTA_APP_LIMIT || size > SU_OTA_APP_LIMIT - s) return -1;
    *stage = s;
    return 0;
}

SU_OTA_RAM uint32_t su_ota_limit(uint32_t app_end)
{
    uint32_t a = page_ceil(app_end);
    if (a >= SU_OTA_APP_LIMIT) return 0;
    return a >= SU_OTA_APP_LIMIT / 2u ? SU_OTA_APP_LIMIT - a : SU_OTA_APP_LIMIT / 2u;
}

SU_OTA_RAM int su_ota_commit(const su_ota_io_t *io, const su_ota_commit_t *c)
{
    /* Staging lies above the running image (su_ota_fit), which holds the
     * kept pages; a kept page lies inside both images. */
    uint32_t pages = page_ceil(c->size) >> 13;
    if (c->size == 0u || pages > SU_OTA_MAX_PAGES || c->old_size == 0u || c->old_size > c->stage) return -1;
    for (uint32_t p = 0; p < SU_OTA_MAX_PAGES; p++)
        if (kept(c->keep, p) && (p >= pages || page_end(p, c->size) > c->old_size)) return -1;

    su_ota_state_t s;
    su_ota_scan(io, &s);
    if (s.free_slots < SU_OTA_MIN_FREE && compact(io) != 0) return -1;

    uint32_t rec[12];
    rec[0]  = SU_OTA_COPA;
    rec[1]  = SU_OTA_FLASH_BASE + c->stage;
    rec[2]  = c->size;
    rec[3]  = c->crc;
    rec[4]  = SU_OTA_KEEP;
    rec[5]  = c->keep[0];
    rec[6]  = c->keep[1];
    rec[7]  = ~(c->keep[0] ^ c->keep[1]);
    rec[8]  = SU_OTA_COPB;
    rec[9]  = c->old_size;
    rec[10] = c->old_crc;
    rec[11] = copy_check(&rec[0], &rec[4], &rec[8]);
    if (append(io, rec, 3) != 0) return -1;

    /* The page now reads "pending" for exactly this image, or it isn't. */
    su_ota_scan(io, &s);
    return (s.pending && s.check == rec[11] && s.stage == SU_OTA_FLASH_BASE + c->stage && s.size == c->size &&
            s.keep[0] == c->keep[0] && s.keep[1] == c->keep[1]) ? 0 : -1;
}

/* ============================================================
 * Boot: decide, copy (docs §8.2)
 * ============================================================ */

/* Copy one 8 KB page of the image from staging to its place, then read it
 * back. Page 0's first quad-word (SP + reset vector) is left for last when
 * `hold_qw0`. All-0xFF quad-words are left erased. */
static SU_OTA_RAM int copy_page(const su_ota_io_t *io, uint32_t stage, uint32_t size, uint32_t page, int hold_qw0)
{
    uint32_t base = page * SU_OTA_PAGE;
    feed(io);
    if (io->erase(io->ctx, page) != 0) return -1;
    for (uint32_t q = (hold_qw0 ? SU_OTA_QW : 0u); q < SU_OTA_PAGE && base + q < size; q += SU_OTA_QW) {
        uint32_t w[4], back[4];
        uint32_t n = size - (base + q);
        if (n > SU_OTA_QW) n = SU_OTA_QW;
        w[0] = w[1] = w[2] = w[3] = 0xFFFFFFFFUL;
        if (io->read(io->ctx, stage + base + q, w, n) != 0) return -1;
        if (all_ff(w, 4)) continue;
        if (io->program(io->ctx, base + q, w, SU_OTA_QW) != 0) return -1;
        if (io->read(io->ctx, base + q, back, sizeof(back)) != 0 || !words_equal(w, back, 4)) return -1;
        if ((q & 0x7FFu) == 0) feed(io);
    }
    return 0;
}

/* One pass over flash for the boot checks (docs §8.2): `*guard` = CRC-32 of
 * [0, old_size) as flash holds it now (only when `old_size` != 0), `*image`
 * = CRC-32 of the image as the copier will assemble it: kept pages from
 * their place, the others from staging. Each flash byte is read once. 0, or
 * -1 on an ECC error. */
static SU_OTA_RAM int boot_crcs(const su_ota_io_t *io, const su_ota_state_t *s, uint32_t stage,
                                uint32_t old_size, uint32_t *guard, uint32_t *image)
{
    uint32_t buf[16], sbuf[16];            /* 64 bytes: divides a page */
    uint32_t g = 0, v = 0;
    uint32_t end = old_size > s->size ? old_size : s->size;
    int bad = 0;
    for (uint32_t off = 0; off < end; off += sizeof(buf)) {
        uint32_t n = end - off < sizeof(buf) ? end - off : (uint32_t)sizeof(buf);
        int k = off < s->size && kept(s->keep, off >> 13);
        if (off < old_size || k) {
            if (io->read(io->ctx, off, buf, n) != 0) bad = 1;
            if (off < old_size) g = su_ota_crc32(g, (const uint8_t *)buf, old_size - off < n ? old_size - off : n);
        }
        if (off < s->size) {
            uint32_t m = s->size - off < n ? s->size - off : n;
            if (k) {
                v = su_ota_crc32(v, (const uint8_t *)buf, m);
            } else {
                if (io->read(io->ctx, stage + off, sbuf, m) != 0) bad = 1;
                v = su_ota_crc32(v, (const uint8_t *)sbuf, m);
            }
        }
        if (((off + n) & (SU_OTA_PAGE - 1u)) == 0) feed(io);
    }
    *guard = g;
    *image = v;
    return bad ? -1 : 0;
}

/* 1 if page 0 already holds exactly what copying would put there: the
 * image's first 8 KB, 0xFF past its end. An ECC error reads as "differs". */
static SU_OTA_RAM int page0_same(const su_ota_io_t *io, uint32_t stage, uint32_t size)
{
    for (uint32_t q = 0; q < SU_OTA_PAGE; q += SU_OTA_QW) {
        uint32_t w[4], cur[4];
        w[0] = w[1] = w[2] = w[3] = 0xFFFFFFFFUL;
        if (q < size) {
            uint32_t n = size - q;
            if (n > SU_OTA_QW) n = SU_OTA_QW;
            if (io->read(io->ctx, stage + q, w, n) != 0) return 0;
        }
        if (io->read(io->ctx, q, cur, sizeof(cur)) != 0 || !words_equal(w, cur, 4)) return 0;
    }
    return 1;
}

/* Kept pages are never erased or written: they already hold the image. */
static SU_OTA_RAM int copy_image(const su_ota_io_t *io, uint32_t stage, uint32_t size, const uint32_t keep[2])
{
    uint32_t pages = page_ceil(size) >> 13;
    for (uint32_t p = 1; p < pages; p++)
        if (!kept(keep, p) && copy_page(io, stage, size, p, 0) != 0) return -1;
    /* Page 0 kept, or staged but identical anyway (a full transfer): leave it
     * alone, so there is no moment without a vector table (docs §8.2). */
    if (kept(keep, 0) || page0_same(io, stage, size)) return 0;
    /* Page 0 last, its first quad-word last of all. */
    if (copy_page(io, stage, size, 0, 1) != 0) return -1;
    uint32_t w[4], back[4];
    uint32_t n = size < SU_OTA_QW ? size : SU_OTA_QW;
    w[0] = w[1] = w[2] = w[3] = 0xFFFFFFFFUL;
    if (io->read(io->ctx, stage, w, n) != 0) return -1;
    if (io->program(io->ctx, 0, w, SU_OTA_QW) != 0) return -1;
    if (io->read(io->ctx, 0, back, sizeof(back)) != 0 || !words_equal(w, back, 4)) return -1;
    return 0;
}

SU_OTA_RAM int su_ota_boot(const su_ota_io_t *io)
{
    su_ota_state_t s;
    su_ota_scan(io, &s);
    if (!s.pending) return SU_OTA_BOOT_NONE;

    uint32_t stage = s.stage - SU_OTA_FLASH_BASE;
    uint32_t pages = page_ceil(s.size) >> 13;
    int sane = s.size != 0u && stage <= SU_OTA_APP_LIMIT && s.size <= SU_OTA_APP_LIMIT - stage &&
               stage >= s.size && (stage & (SU_OTA_PAGE - 1u)) == 0u &&
               s.old_size != 0u && s.old_size <= stage && pages <= SU_OTA_MAX_PAGES;
    for (uint32_t p = 0; sane && p < SU_OTA_MAX_PAGES; p++)
        if (kept(s.keep, p) && (p >= pages || page_end(p, s.size) > s.old_size)) sane = 0;
    if (!sane) {
        (void)append_done(io, s.check, 1u);
        return SU_OTA_BOOT_ABANDONED;
    }

    uint32_t guard, image;
    if (!s.started) {
        /* 1. The reflash guard: flash must still hold the image that
         * committed (kept pages included: they are part of it). If another
         * tool flashed the Core since, leave its image alone. Nothing has been
         * erased yet, so the check is exact. The same pass checks the image
         * the copy will assemble: staged pages + kept pages. */
        int rc = boot_crcs(io, &s, stage, s.old_size, &guard, &image);
        if (rc != 0 || guard != s.old_crc) {
            (void)append_done(io, s.check, 1u);
            return SU_OTA_BOOT_ABANDONED;
        }
        /* The same image again: nothing to copy. */
        if (s.old_size == s.size && s.old_crc == s.crc) {
            (void)append_done(io, s.check, 0u);
            return SU_OTA_BOOT_MARKED;
        }
        /* 2. Staging + kept pages make the image, then START: from here on a
         * cut resumes the copy rather than re-running the guard (flash is
         * half replaced). */
        if (image != s.crc || append_mark(io, SU_OTA_STRT, s.check, 0u) != 0) {
            (void)append_done(io, s.check, 1u);
            return SU_OTA_BOOT_ABANDONED;
        }
    } else {
        /* A resumed copy. Already installed (cut after the last write,
         * before DONE)? */
        int ok = 1;
        if (crc_flash(io, 0, s.size, 0, &ok) == s.crc && ok) {
            (void)append_done(io, s.check, 0u);
            return SU_OTA_BOOT_MARKED;
        }
        /* Staging (+ the kept pages, never erased) is the only good copy
         * left; if it doesn't check out, flash is a mix of two images: stop
         * rather than run it (SWD recovers). */
        if (boot_crcs(io, &s, stage, 0u, &guard, &image) != 0 || image != s.crc) return SU_OTA_BOOT_FAILED;
    }

    /* 3. Copy the pages that aren't kept (idempotent: staging is above the
     * destination), verify, DONE. */
    for (int attempt = 0; attempt < 3; attempt++) {
        if (copy_image(io, stage, s.size, s.keep) != 0) continue;
        int ok4 = 1;
        if (crc_flash(io, 0, s.size, 0, &ok4) != s.crc || !ok4) continue;
        (void)append_done(io, s.check, 0u);
        return SU_OTA_BOOT_INSTALLED;
    }
    return SU_OTA_BOOT_FAILED;
}
