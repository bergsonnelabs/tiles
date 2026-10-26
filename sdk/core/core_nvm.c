/**
 * core_nvm.c — core_nvm on every Core (API and behaviour: core_nvm.h).
 *
 * Compiled into every build, like core_watchdog.c; --gc-sections drops it
 * when a project never calls core_nvm. The one exception is the NMI handler on
 * the L4 and W5 (the port's, port_core_nvm_hw.h), which the vector table
 * always pulls in.
 *
 *   Core.ST.L0  data EEPROM, written byte by byte (the header-only code that
 *               was here before, plus FLASH_SR checks and a read-back).
 *   Core.ST.L4  flash emulation (this file), page erase/program via ll_flash.
 *   Core.ST.W5  flash emulation (this file); with BLE running, every erase and
 *               program goes through ST's flash manager (sdk/wpan/wpan_flash.c);
 *               so does a Thread build's.
 *   Core.ST.H5  flash emulation (this file) in the high-cycle data area
 *               (EDATA), written 16 bits at a time; plus the first-boot switch
 *               that turns the area on (_core_nvm_h5_boot, the port's
 *               port_core_nvm_h5_boot.h, included at the end).
 *
 * ---- Flash emulation: layout ----
 *
 * Two pages, A and B, at CORE_NVM_FLASH_ADDR. A unit (UNIT) is the program
 * granule: 8 B on the L4, 16 B on the W5. Everything is written in whole
 * units, in address order, and never rewritten until the page is erased.
 * On the H5 a unit is 8 B too, but the data area programs 16-bit words, so a
 * unit is four programs; a power cut can tear it anywhere, which is the same
 * torn-record case as below.
 *
 *   unit 0      page header: 'NVM1' magic, generation (u32)
 *   unit 1      commit: ~generation, CRC-32 of (magic, generation)
 *   unit 2...   records, back to back, each padded to a whole unit:
 *                 u16 offset, u16 length, u32 CRC-32 of (offset, length, data)
 *                 data[length]
 *
 * The valid page (header and commit both correct) with the highest
 * generation is active. Its records, replayed in order over an image of
 * 0xFF, give the stored bytes. The first record that is not valid ends the log.
 *
 * ---- Writes and compaction ----
 *
 * A write appends one record, then reads it back. When the record does not fit
 * (or the tail of the page is unusable, below), the write compacts instead:
 * erase the other page, write its header, copy the current contents into it as
 * 128-byte records (all-0xFF chunks skipped) with the new data merged in,
 * verify, and write the commit unit last. The old page is left as it is until
 * the next compaction erases it.
 *
 * ---- Power loss ----
 *
 *   - During an append: the torn record fails its CRC (or reads with an ECC
 *     error), so the log ends before it and the old bytes stand. The tail is
 *     marked unusable and the next write compacts.
 *   - During a compaction: until the commit unit is complete, the new page
 *     has no valid commit and the old page stays active (old value). Once it
 *     is, the new page has the higher generation (new value). The erase at the
 *     start only ever touches the page that is not active.
 *   - A torn program can leave a unit whose ECC can't be corrected: reading it
 *     raises an NMI (RM0394 §3.3.2, RM0493 §7.3.2). The port's NMI handler
 *     recognises a double-ECC error inside the NVM pages, clears it and flags
 *     the read, so the scan treats that unit as invalid instead of hanging in
 *     Default_Handler at every boot. Any other NMI still goes to the same
 *     endless loop it did before.
 *
 * ST's EEPROM emulation (AN4894) uses the same two-page idea with per-page
 * state words; this keeps it to a generation number and a commit unit, and
 * makes a multi-byte write one CRC-checked record so it is atomic.
 */

#include "core_nvm.h"
/* The chip's part is the port's, in three files included where that code
 * always sat (port_core_nvm_hw.h lists them). This one: the L0 EEPROM steps,
 * the flash reads and their ECC handling, the H5 layout check. */
#include "port_core_nvm_hw.h"

/* ============================================================
 * Core.ST.L0 — data EEPROM
 * ============================================================ */

#if CORE_NVM_EEPROM

int core_nvm_read(uint32_t offset, void *buf, uint32_t len)
{
    /* Written this way, not `offset + len > SIZE`, because that sum wraps:
     * a large offset with a small len passes the naive check and then
     * reads or writes outside the region. */
    if (len > (uint32_t)CORE_NVM_SIZE ||
        offset > (uint32_t)CORE_NVM_SIZE - len)
        return CORE_NVM_ERR_RANGE;

    memcpy(buf, (const void *)(CORE_NVM_BASE + offset), len);
    return CORE_NVM_OK;
}

int core_nvm_write(uint32_t offset, const void *data, uint32_t len)
{
    if (len > (uint32_t)CORE_NVM_SIZE ||
        offset > (uint32_t)CORE_NVM_SIZE - len)
        return CORE_NVM_ERR_RANGE;

    /* True EEPROM: the hardware erases before it writes. */
    uint32_t vos_was = port_core_nvm_eeprom_begin();
    const uint8_t *src = (const uint8_t *)data;
    volatile uint8_t *dst = (volatile uint8_t *)(CORE_NVM_BASE + offset);
    int rc = CORE_NVM_OK;
    for (uint32_t i = 0; i < len; i++) {
        dst[i] = src[i];
        if (port_core_nvm_eeprom_wait() != 0 || dst[i] != src[i]) {
            rc = CORE_NVM_ERR_FLASH;
            break;
        }
    }
    port_core_nvm_eeprom_end(vos_was);
    return rc;
}

int core_nvm_erase_all(void)
{
    /* Word writes of 0 (an EEPROM erase: RM0377 §3.3.4). Words already 0 are
     * skipped, so a cleared EEPROM costs nothing. */
    uint32_t vos_was = port_core_nvm_eeprom_begin();
    volatile uint32_t *w = (volatile uint32_t *)CORE_NVM_BASE;
    int rc = CORE_NVM_OK;
    for (uint32_t i = 0; i < (uint32_t)CORE_NVM_SIZE / 4u; i++) {
        if (w[i] == 0u) continue;
        w[i] = 0u;
        if (port_core_nvm_eeprom_wait() != 0 || w[i] != 0u) {
            rc = CORE_NVM_ERR_FLASH;
            break;
        }
    }
    port_core_nvm_eeprom_end(vos_was);
    return rc;
}

/* ============================================================
 * Core.ST.L4 / Core.ST.W5 — flash emulation
 * ============================================================ */

#elif CORE_NVM_FLASH_EMU

#define UNIT       CORE_NVM_PROG_UNIT
#define PAGE       CORE_NVM_PAGE_SIZE
#define HDR_BYTES  (2u * UNIT)              /* header unit + commit unit */
#define REC_HDR    8u                       /* offset, length, CRC */
#define CHUNK      128u                     /* compaction record size */
#define MAGIC      0x314D564EUL             /* "NVM1" */
#define ROUND_U(n) (((n) + (UNIT - 1u)) & ~(UNIT - 1u))

/* ---- The linker script must reserve the pages (sdk/device/ scripts) ---- */
extern const uint8_t __core_nvm_start[];
/* Core.ST.H5: the port's NVM_LAYOUT_OK() checks the option bytes too
 * (port_core_nvm_hw.h). */
#ifndef NVM_LAYOUT_OK
#define NVM_LAYOUT_OK()  ((uint32_t)(uintptr_t)__core_nvm_start == CORE_NVM_FLASH_ADDR)
#endif

/* ---- CRC-32 (IEEE, as zlib), nibble table ---- */
static uint32_t crc32_upd(uint32_t c, const uint8_t *p, uint32_t n)
{
    static const uint32_t t[16] = {
        0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
        0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
        0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
        0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL,
    };
    while (n--) {
        c ^= *p++;
        c = (c >> 4) ^ t[c & 0xFu];
        c = (c >> 4) ^ t[c & 0xFu];
    }
    return c;
}

/* The chip's erase and program (port_core_nvm_erase(), _program()). */
#include "port_core_nvm_hw_prog.h"

/* ============================================================
 * State
 * ============================================================ */

static struct {
    uint8_t  mounted;
    int8_t   active;        /* 0 / 1, or -1: no valid page (all bytes 0xFF) */
    uint8_t  dirty;         /* tail of the active page unusable: compact next */
    uint32_t gen;           /* highest valid generation seen */
    uint32_t end;           /* first free byte in the active page */
} s;

static volatile uint8_t s_busy;

/* Staging buffer for records: whole units, so the flash-manager path can hand
 * it over in one call. */
#define STAGE 256u
static uint32_t s_stage[STAGE / 4u];

static inline uint32_t page_addr(int p)
{
    return CORE_NVM_FLASH_ADDR + (uint32_t)p * PAGE;
}

/* Returns 1 and the generation if page p has a valid header and commit. */
static int read_header(int p, uint32_t *gen)
{
    uint32_t h[2], c[2];
    if (port_core_nvm_flash_read(page_addr(p), h, 8) != 0) return 0;
    if (port_core_nvm_flash_read(page_addr(p) + UNIT, c, 8) != 0) return 0;
    if (h[0] != MAGIC || h[1] == 0u || h[1] == 0xFFFFFFFFUL) return 0;
    if (c[0] != ~h[1] || c[1] != ~crc32_upd(0xFFFFFFFFUL, (const uint8_t *)h, 8)) return 0;
    *gen = h[1];
    return 1;
}

/* Record header at addr: 1 = valid record (off/len out), 0 = erased (end of
 * a clean log), -1 = invalid (torn or garbage). Checks the CRC. */
static int read_record(uint32_t addr, uint32_t limit, uint32_t *off, uint32_t *len)
{
    uint32_t h[2];
    if (port_core_nvm_flash_read(addr, h, 8) != 0) return -1;
    if (h[0] == 0xFFFFFFFFUL && h[1] == 0xFFFFFFFFUL)
        return port_core_nvm_flash_is_blank(addr, UNIT) ? 0 : -1;    /* a torn unit can be FF up front */
    uint32_t o = h[0] & 0xFFFFu, n = h[0] >> 16;
    if (n == 0u || n > (uint32_t)CORE_NVM_SIZE || o > (uint32_t)CORE_NVM_SIZE - n) return -1;
    if (ROUND_U(REC_HDR + n) > limit - addr) return -1;

    uint32_t c = crc32_upd(0xFFFFFFFFUL, (const uint8_t *)h, 4);
    uint8_t buf[64];
    for (uint32_t i = 0; i < n; i += sizeof(buf)) {
        uint32_t k = (n - i) < sizeof(buf) ? (n - i) : sizeof(buf);
        if (port_core_nvm_flash_read(addr + REC_HDR + i, buf, k) != 0) return -1;
        c = crc32_upd(c, buf, k);
    }
    if (~c != h[1]) return -1;
    *off = o;
    *len = n;
    return 1;
}

/* Walk the active page's log: sets s.end (first free byte) and s.dirty. */
static void scan(void)
{
    uint32_t base = page_addr(s.active), limit = base + PAGE;
    uint32_t pos = base + HDR_BYTES;
    s.dirty = 0;
    while (pos < limit) {
        uint32_t o, n;
        int r = read_record(pos, limit, &o, &n);
        if (r == 0) break;
        if (r < 0) { s.dirty = 1; break; }
        pos += ROUND_U(REC_HDR + n);
    }
    s.end = pos - base;
}

static int mount(void)
{
    if (s.mounted) return CORE_NVM_OK;
    if (!NVM_LAYOUT_OK())
        return CORE_NVM_ERR_LAYOUT;

    port_core_nvm_clear_ecc();

    uint32_t g0 = 0, g1 = 0;
    int v0 = read_header(0, &g0), v1 = read_header(1, &g1);
    if (v0 && v1)  s.active = (g1 > g0) ? 1 : 0;
    else if (v0)   s.active = 0;
    else if (v1)   s.active = 1;
    else           s.active = -1;
    s.gen = (g0 > g1) ? g0 : g1;
    if (s.active >= 0) scan();
    s.mounted = 1;
    return CORE_NVM_OK;
}

/* The stored bytes [off, off + len) from the active page (0xFF if none). */
static int replay(uint32_t off, uint8_t *dst, uint32_t len)
{
    memset(dst, 0xFF, len);
    if (s.active < 0) return CORE_NVM_OK;
    uint32_t base = page_addr(s.active);
    uint32_t pos = base + HDR_BYTES, end = base + s.end;
    while (pos < end) {
        uint32_t h;
        if (port_core_nvm_flash_read(pos, &h, 4) != 0) return CORE_NVM_ERR_FLASH;
        uint32_t o = h & 0xFFFFu, n = h >> 16;
        uint32_t lo = (o > off) ? o : off;
        uint32_t hi = (o + n < off + len) ? o + n : off + len;
        if (lo < hi &&
            port_core_nvm_flash_read(pos + REC_HDR + (lo - o), dst + (lo - off), hi - lo) != 0)
            return CORE_NVM_ERR_FLASH;
        pos += ROUND_U(REC_HDR + n);
    }
    return CORE_NVM_OK;
}

/* Program one record at addr: header, data, 0xFF padding to a whole unit. */
static int program_record(uint32_t addr, uint32_t off, const uint8_t *data, uint32_t len)
{
    uint32_t h[2];
    h[0] = (off & 0xFFFFu) | (len << 16);
    h[1] = ~crc32_upd(crc32_upd(0xFFFFFFFFUL, (const uint8_t *)h, 4), data, len);

    uint8_t *st = (uint8_t *)s_stage;
    uint32_t total = ROUND_U(REC_HDR + len), done = 0;
    while (done < total) {
        uint32_t n = (total - done) < STAGE ? (total - done) : STAGE;
        for (uint32_t i = 0; i < n; i++) {
            uint32_t k = done + i;
            st[i] = (k < REC_HDR) ? ((const uint8_t *)h)[k]
                  : (k < REC_HDR + len) ? data[k - REC_HDR] : 0xFFu;
        }
        if (port_core_nvm_program(addr + done, st, n) != 0) return CORE_NVM_ERR_FLASH;
        done += n;
    }

    /* Read back: a program that reported no error can still be short. */
    uint32_t o, n;
    if (read_record(addr, addr + total, &o, &n) != 1 || o != off || n != len)
        return CORE_NVM_ERR_FLASH;
    uint8_t buf[64];
    for (uint32_t i = 0; i < len; i += sizeof(buf)) {
        uint32_t k = (len - i) < sizeof(buf) ? (len - i) : sizeof(buf);
        if (port_core_nvm_flash_read(addr + REC_HDR + i, buf, k) != 0 || memcmp(buf, data + i, k) != 0)
            return CORE_NVM_ERR_FLASH;
    }
    return CORE_NVM_OK;
}

/* Copy the current contents (with [off, off+len) replaced by data, or
 * nothing at all if wipe) into the other page and make it active. */
static int compact(uint32_t off, const uint8_t *data, uint32_t len, int wipe)
{
    int tgt = (s.active < 0) ? 0 : 1 - s.active;
    uint32_t base = page_addr(tgt);
    uint32_t gen = s.gen + 1u;
    if (gen == 0xFFFFFFFFUL) gen = 1u;          /* 4e9 compactions: never */

    /* Always erase, even a page that reads blank: an erase cut short by a
     * reset can leave cells that read 1 without being fully erased. */
    if (port_core_nvm_erase(CORE_NVM_FIRST_PAGE + (uint32_t)tgt) != 0 || !port_core_nvm_flash_is_blank(base, PAGE))
        return CORE_NVM_ERR_FLASH;

    uint32_t u[UNIT / 4u];
    memset(u, 0xFF, sizeof(u));
    u[0] = MAGIC;
    u[1] = gen;
    if (port_core_nvm_program(base, (const uint8_t *)u, UNIT) != 0) return CORE_NVM_ERR_FLASH;
    uint32_t hcrc = ~crc32_upd(0xFFFFFFFFUL, (const uint8_t *)u, 8);

    uint32_t pos = HDR_BYTES;
    for (uint32_t c = 0; !wipe && c < (uint32_t)CORE_NVM_SIZE; c += CHUNK) {
        uint8_t buf[CHUNK];
        if (replay(c, buf, CHUNK) != 0) return CORE_NVM_ERR_FLASH;
        uint32_t lo = (off > c) ? off : c;
        uint32_t hi = (off + len < c + CHUNK) ? off + len : c + CHUNK;
        if (lo < hi) memcpy(buf + (lo - c), data + (lo - off), hi - lo);

        int blank = 1;
        for (uint32_t i = 0; i < CHUNK; i++) if (buf[i] != 0xFFu) { blank = 0; break; }
        if (blank) continue;

        if (program_record(base + pos, c, buf, CHUNK) != 0) return CORE_NVM_ERR_FLASH;
        pos += ROUND_U(REC_HDR + CHUNK);
    }

    /* Commit: from here the new page wins. */
    memset(u, 0xFF, sizeof(u));
    u[0] = ~gen;
    u[1] = hcrc;
    if (port_core_nvm_program(base + UNIT, (const uint8_t *)u, UNIT) != 0) return CORE_NVM_ERR_FLASH;
    uint32_t g;
    if (!read_header(tgt, &g) || g != gen) return CORE_NVM_ERR_FLASH;

    s.active = (int8_t)tgt;
    s.gen = gen;
    s.end = pos;
    s.dirty = 0;
    return CORE_NVM_OK;
}

/* 1 if [off, off+len) already holds data. */
static int unchanged(uint32_t off, const uint8_t *data, uint32_t len)
{
    uint8_t buf[64];
    for (uint32_t i = 0; i < len; i += sizeof(buf)) {
        uint32_t k = (len - i) < sizeof(buf) ? (len - i) : sizeof(buf);
        if (replay(off + i, buf, k) != 0 || memcmp(buf, data + i, k) != 0) return 0;
    }
    return 1;
}

static int do_write(uint32_t off, const uint8_t *data, uint32_t len)
{
    if (unchanged(off, data, len)) return CORE_NVM_OK;
    if (s.active < 0) return compact(off, data, len, 0);

    uint32_t need = ROUND_U(REC_HDR + len);
    uint32_t at = page_addr(s.active) + s.end;
    if (!s.dirty && s.end + need <= PAGE && port_core_nvm_flash_is_blank(at, need)) {
        if (program_record(at, off, data, len) == 0) {
            s.end += need;
            return CORE_NVM_OK;
        }
    }
    /* Full, or the tail is unusable (a torn or failed record): compact. */
    s.dirty = 1;
    return compact(off, data, len, 0);
}

/* ============================================================
 * API
 * ============================================================ */

/* Always inlined: on the L4 / W5 this is exactly the `s_busy = 0` it was. */
static inline __attribute__((always_inline)) void leave(void)
{
    port_core_nvm_hw_end();
    s_busy = 0;
}

static int enter(void)
{
    if (s_busy) return CORE_NVM_ERR_BUSY;
    s_busy = 1;
    port_core_nvm_hw_begin();
    int rc = mount();
    if (rc != CORE_NVM_OK) leave();
    return rc;
}

int core_nvm_read(uint32_t offset, void *buf, uint32_t len)
{
    /* Written this way, not `offset + len > SIZE`, because that sum wraps. */
    if (len > (uint32_t)CORE_NVM_SIZE ||
        offset > (uint32_t)CORE_NVM_SIZE - len)
        return CORE_NVM_ERR_RANGE;
    int rc = enter();
    if (rc != CORE_NVM_OK) return rc;
    rc = replay(offset, (uint8_t *)buf, len);
    leave();
    return rc;
}

int core_nvm_write(uint32_t offset, const void *data, uint32_t len)
{
    if (len > (uint32_t)CORE_NVM_SIZE ||
        offset > (uint32_t)CORE_NVM_SIZE - len)
        return CORE_NVM_ERR_RANGE;
    if (len == 0u) return CORE_NVM_OK;
    int rc = enter();
    if (rc != CORE_NVM_OK) return rc;
    rc = do_write(offset, (const uint8_t *)data, len);
    leave();
    return rc;
}

/* Not API: bytes left in the active page before the next compaction (0 when
 * the next write compacts anyway). tests/hw-nvm-flash uses it to aim its
 * power cuts at appends and at compactions. */
uint32_t _core_nvm_log_free(void)
{
    if (enter() != CORE_NVM_OK) return 0;
    uint32_t f = (s.active < 0 || s.dirty) ? 0u : PAGE - s.end;
    leave();
    return f;
}

int core_nvm_erase_all(void)
{
    int rc = enter();
    if (rc != CORE_NVM_OK) return rc;
    if (s.active >= 0 && (s.dirty || s.end > HDR_BYTES)) {
        /* Records may all be 0xFF already; only erase if something isn't. */
        int empty = 1;
        uint8_t buf[CHUNK];
        for (uint32_t c = 0; empty && c < (uint32_t)CORE_NVM_SIZE; c += CHUNK) {
            if (replay(c, buf, CHUNK) != 0) { empty = 0; break; }
            for (uint32_t i = 0; i < CHUNK; i++) if (buf[i] != 0xFFu) { empty = 0; break; }
        }
        if (!empty || s.dirty) rc = compact(0, 0, 0, 1);
    }
    leave();
    return rc;
}

/* Core.ST.H5: the first-boot switch of the data area (_core_nvm_h5_boot()). */
#include "port_core_nvm_h5_boot.h"

/* ============================================================
 * A Core without NVM
 * ============================================================ */

#else

/* CORE_NVM_SIZE is 0: only a zero-length read at offset 0 is in range, and
 * it copies nothing (as the header-only version did). */
int core_nvm_read(uint32_t offset, void *buf, uint32_t len)
{
    (void)buf;
    if (len > (uint32_t)CORE_NVM_SIZE ||
        offset > (uint32_t)CORE_NVM_SIZE - len)
        return CORE_NVM_ERR_RANGE;
    return CORE_NVM_OK;
}

int core_nvm_write(uint32_t offset, const void *data, uint32_t len)
{
    (void)offset; (void)data; (void)len;
    return CORE_NVM_ERR_RANGE;
}

int core_nvm_erase_all(void)
{
    return CORE_NVM_ERR_RANGE;
}

#endif
