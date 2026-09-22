/**
 * core_scope.c — Scope: stream live variables to a host for plotting
 *
 * Wire format: docs/scope-protocol.md. Three stages, so that the part that
 * runs where the data is made stays tiny:
 *
 *   sample()  gather the watched variables into a fixed-size record in the
 *             sample ring. ISR-safe; single producer.
 *   pump()    turn ring records into frames in the staging buffer, then
 *             offer staged bytes to the link. Main loop; single consumer.
 *   link      USB CDC queue / BLE notification / anything with a write().
 *
 * There is exactly one place a sample can be lost: the ring, when pump()
 * or the link falls behind. Staged frames are never discarded while the
 * host is listening — they wait, the ring backs up, and the producer counts
 * what it had to drop.
 */
#include "core_scope.h"

#if CORE_SCOPE_ENABLED

#include <string.h>

#ifdef CORE_SCOPE_HOST_TEST
uint32_t core_millis(void); /* supplied by the test harness */
#else
#include "core_timing.h"
#endif

#define SCOPE_MAGIC0        0xA5u
#define SCOPE_MAGIC1        0x53u
#define SCOPE_FRAME_SCHEMA  0x01u /* channel list                          */
#define SCOPE_FRAME_DATA    0x02u /* one sample, millisecond timestamp      */
#define SCOPE_FRAME_ROWS    0x03u /* N samples at a declared rate           */
#define SCOPE_OVERHEAD      6u    /* magic(2) type seq len ... crc          */
#define SCOPE_MAX_PAYLOAD   255u
#define SCOPE_SCHEMA_MS     1000u /* re-announce the channel list this often */
#define SCOPE_NAME_MAX      63u   /* bytes of a channel name sent            */
#define SCOPE_ROWS_HEADER   9u    /* schema_id, n0 u32, rate u32            */
#define SCOPE_DATA_HEADER   5u    /* schema_id, millis u32                  */
#define SCOPE_SCALAR        0xFFu /* chan.index: not an array element       */

_Static_assert(CORE_SCOPE_MAX_CHANNELS >= 1 && CORE_SCOPE_MAX_CHANNELS <= 32,
               "the wire format carries at most 32 channels");
_Static_assert(CORE_SCOPE_TX_BYTES >= 64, "CORE_SCOPE_TX_BYTES is too small to be useful");
_Static_assert(CORE_SCOPE_RING_BYTES >= 16 && CORE_SCOPE_RING_BYTES <= 0xFFFF,
               "CORE_SCOPE_RING_BYTES out of range");

typedef struct {
    const char          *name;
    const volatile void *ptr;   /* the element itself, for an array channel */
    uint8_t              type;  /* core_scope_type_t */
    uint8_t              index; /* array index for the name, or SCOPE_SCALAR */
} scope_chan_t;

/* Bytes per value on the wire, by type code. A bool is 1 byte in memory and
 * 4 on the wire (as the generated Studio scope always sent it). */
static const uint8_t k_wire_bytes[8] = { 4, 4, 4, 4, 2, 2, 1, 1 };

static scope_chan_t s_chan[CORE_SCOPE_MAX_CHANNELS];
static uint8_t      s_count;
static uint16_t     s_schema_len = 2; /* schema payload: id, count, descriptors */
static uint8_t      s_schema_id;
static uint8_t      s_schema_dirty;

/* Sample ring: s_cap fixed-size records of [stamp u32][row]. One slot is
 * kept empty, so head == tail means empty. */
static uint8_t           s_ring[CORE_SCOPE_RING_BYTES];
static uint8_t           s_row_bytes;
static uint16_t          s_rec_bytes = 4;
static uint16_t          s_cap;
static volatile uint16_t s_head; /* producer: sample() */
static volatile uint16_t s_tail; /* consumer: pump()   */
static volatile uint32_t s_sample_n;
static volatile uint32_t s_drop_total;
static uint32_t          s_drop_seen;

static volatile uint8_t s_active;
static uint8_t          s_enabled = 1;
static uint8_t          s_inited;
static uint32_t         s_rate_mhz;    /* 0 = timestamped samples */
static uint32_t         s_interval_ms; /* update() gate */
static uint32_t         s_last_ms;

static const core_scope_link_t *s_link;

/* Staging: whole frames, back to back. s_tx_cont counts the bytes at the
 * front that are the tail of a frame already partly sent. */
static uint8_t  s_tx[CORE_SCOPE_TX_BYTES];
static uint16_t s_tx_len;
static uint16_t s_tx_cont;
static uint32_t s_tx_ms;
static uint8_t  s_seq;
static uint8_t  s_frame_crc;
static uint8_t  s_schema_sent;
static uint32_t s_schema_ms;

/* ---- CRC-8, poly 0x07, init 0 ---- */

static const uint8_t k_crc8[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3,
};

static uint8_t crc8(uint8_t crc, const uint8_t *p, uint16_t n)
{
    while (n--) crc = k_crc8[crc ^ *p++];
    return crc;
}

/* ---- Frames into staging ---- */

static void put_u32(uint8_t *p, uint32_t u)
{
    p[0] = (uint8_t)u;
    p[1] = (uint8_t)(u >> 8);
    p[2] = (uint8_t)(u >> 16);
    p[3] = (uint8_t)(u >> 24);
}

static void frame_put(const uint8_t *p, uint16_t n)
{
    memcpy(&s_tx[s_tx_len], p, n);
    s_tx_len = (uint16_t)(s_tx_len + n);
    s_frame_crc = crc8(s_frame_crc, p, n);
}

/* Open a frame, or refuse (-1) when staging cannot hold all of it. */
static int frame_begin(uint8_t type, uint8_t len, uint32_t now)
{
    if ((uint32_t)s_tx_len + SCOPE_OVERHEAD + len > CORE_SCOPE_TX_BYTES) return -1;
    if (s_tx_len == 0) s_tx_ms = now;
    s_tx[s_tx_len++] = SCOPE_MAGIC0;
    s_tx[s_tx_len++] = SCOPE_MAGIC1;
    const uint8_t hdr[3] = { type, s_seq++, len };
    s_frame_crc = 0;
    frame_put(hdr, 3);
    return 0;
}

static void frame_end(void)
{
    s_tx[s_tx_len++] = s_frame_crc;
}

/* ---- Schema ---- */

static uint8_t name_len(const scope_chan_t *ch)
{
    size_t n = strlen(ch->name);
    if (n > SCOPE_NAME_MAX - 4) n = SCOPE_NAME_MAX - 4; /* room for "[31]" */
    if (ch->index != SCOPE_SCALAR) n += (ch->index >= 10) ? 4u : 3u;
    return (uint8_t)n;
}

/* Walk the descriptor (count, then type / name_len / name per channel),
 * feeding it to `put` — the CRC for the schema id, or the open frame. */
static void schema_walk(void (*put)(const uint8_t *, uint16_t))
{
    put(&s_count, 1);
    for (uint8_t i = 0; i < s_count; i++) {
        const scope_chan_t *ch = &s_chan[i];
        uint8_t total = name_len(ch);
        uint8_t suffix = (ch->index == SCOPE_SCALAR) ? 0u : (ch->index >= 10 ? 4u : 3u);
        const uint8_t head[2] = { ch->type, total };
        put(head, 2);
        put((const uint8_t *)ch->name, (uint16_t)(total - suffix));
        if (suffix) {
            uint8_t s[4], k = 0;
            s[k++] = '[';
            if (ch->index >= 10) s[k++] = (uint8_t)('0' + ch->index / 10);
            s[k++] = (uint8_t)('0' + ch->index % 10);
            s[k++] = ']';
            put(s, k);
        }
    }
}

static uint8_t s_id_crc;
static void id_put(const uint8_t *p, uint16_t n) { s_id_crc = crc8(s_id_crc, p, n); }

static int emit_schema(uint32_t now)
{
    if (frame_begin(SCOPE_FRAME_SCHEMA, (uint8_t)s_schema_len, now) != 0) return -1;
    frame_put(&s_schema_id, 1);
    schema_walk(frame_put);
    frame_end();
    return 0;
}

/* ---- Link ---- */

static void scope_stop(void)
{
    s_active = 0;
    s_tx_len = 0;
    s_tx_cont = 0;
    s_schema_sent = 0;
}

/* How much of staging to offer in one write: the rest of a split frame,
 * then as many WHOLE frames as fit a packet. A frame bigger than a packet
 * (a long schema on BLE) is the only thing ever split. */
static uint16_t next_chunk(uint16_t pkt)
{
    uint16_t n = 0;
    if (s_tx_cont) {
        n = s_tx_cont < pkt ? s_tx_cont : pkt;
        if (n < s_tx_cont) return n;
    }
    while ((uint32_t)n + SCOPE_OVERHEAD <= s_tx_len) {
        uint16_t flen = (uint16_t)(SCOPE_OVERHEAD + s_tx[n + 4]);
        if ((uint32_t)n + flen > pkt) break;
        n = (uint16_t)(n + flen);
    }
    if (n == 0) n = s_tx_len < pkt ? s_tx_len : pkt;
    return n;
}

/* `taken` bytes left staging: work out where that leaves the frame
 * boundary, then close the gap. */
static void consume(uint16_t taken)
{
    uint16_t pos = s_tx_cont < taken ? s_tx_cont : taken;
    s_tx_cont = (uint16_t)(s_tx_cont - pos);
    while (pos < taken) {
        uint16_t flen = (uint16_t)(SCOPE_OVERHEAD + s_tx[pos + 4]);
        if ((uint32_t)pos + flen > taken) {
            s_tx_cont = (uint16_t)(pos + flen - taken);
            break;
        }
        pos = (uint16_t)(pos + flen);
    }
    s_tx_len = (uint16_t)(s_tx_len - taken);
    if (s_tx_len) memmove(s_tx, &s_tx[taken], s_tx_len);
}

static void flush(uint32_t now)
{
    uint16_t pkt = s_link->packet_bytes ? s_link->packet_bytes : CORE_SCOPE_TX_BYTES;
    while (s_tx_len) {
        /* Batch into a packet until it is full or has waited long enough. */
        if (s_tx_len < pkt && (now - s_tx_ms) < s_link->flush_ms) break;
        uint16_t offer = next_chunk(pkt);
        int taken = s_link->write(s_tx, offer);
        if (taken <= 0) break;
        consume((uint16_t)taken);
        s_tx_ms = now;
    }
}

/* ---- Channels ---- */

static void relayout(void)
{
    uint16_t row = 0;
    for (uint8_t i = 0; i < s_count; i++) row = (uint16_t)(row + k_wire_bytes[s_chan[i].type]);
    s_row_bytes = (uint8_t)row;
    s_rec_bytes = (uint16_t)(4u + row);
    s_cap = (uint16_t)(CORE_SCOPE_RING_BYTES / s_rec_bytes);
    s_head = 0;
    s_tail = 0;
    s_schema_dirty = 1;
}

static int add_one(const char *name, const volatile void *ptr, uint8_t type, uint8_t index)
{
    if (!name || !ptr || type > CORE_SCOPE_U8) return -1;
    if (s_count >= CORE_SCOPE_MAX_CHANNELS) return -1;
    scope_chan_t ch = { name, ptr, type, index };
    uint16_t len = (uint16_t)(s_schema_len + 2u + name_len(&ch));
    if (len > SCOPE_MAX_PAYLOAD || len + SCOPE_OVERHEAD > CORE_SCOPE_TX_BYTES) return -1;
    /* A ring that cannot hold two records cannot hold one. */
    if (CORE_SCOPE_RING_BYTES / (s_rec_bytes + k_wire_bytes[type]) < 2) return -1;
    s_chan[s_count++] = ch;
    s_schema_len = len;
    return 0;
}

void core_scope_init(void)
{
    if (s_inited) return;
    s_inited = 1;
#if defined(STM32L422xx) || defined(STM32H523xx)
    s_link = &core_scope_link_usb;
#elif defined(BLE_ENABLED) && BLE_ENABLED
    s_link = &core_scope_link_ble;
#endif
    if (s_link && s_link->init) s_link->init();
}

void core_scope_set_link(const core_scope_link_t *link)
{
    s_inited = 1;
    scope_stop();
    s_link = link;
    if (s_link && s_link->init) s_link->init();
}

int core_scope_add(const char *name, const volatile void *ptr, core_scope_type_t type)
{
    core_scope_init();
    scope_stop(); /* sampling resumes, with the new channel list, on the next pump() */
    int rc = add_one(name, ptr, (uint8_t)type, SCOPE_SCALAR);
    relayout();
    return rc;
}

int core_scope_add_array(const char *name, const volatile void *ptr, core_scope_type_t type,
                         uint8_t count)
{
    core_scope_init();
    if ((uint8_t)type > CORE_SCOPE_U8 || count == 0 || count > 32) return -1;
    scope_stop();
    uint8_t  count0 = s_count;
    uint16_t len0 = s_schema_len;
    uint8_t  mem = (type == CORE_SCOPE_BOOL) ? (uint8_t)sizeof(bool) : k_wire_bytes[type];
    int      rc = 0;
    for (uint8_t i = 0; i < count && rc == 0; i++) {
        rc = add_one(name, (const volatile uint8_t *)ptr + (uint16_t)(i * mem), (uint8_t)type, i);
        if (rc == 0) s_rec_bytes = (uint16_t)(s_rec_bytes + k_wire_bytes[type]);
    }
    if (rc != 0) { /* all or none */
        s_count = count0;
        s_schema_len = len0;
    }
    relayout();
    return rc;
}

void core_scope_set_interval_ms(uint32_t ms) { s_interval_ms = ms; }

void core_scope_declare_rate_millihertz(uint32_t millihertz)
{
    scope_stop(); /* records already in the ring carry the other kind of stamp */
    s_rate_mhz = millihertz;
}

void core_scope_declare_rate_hz(uint32_t hz)
{
    core_scope_declare_rate_millihertz(hz > 4000000u ? 4000000000u : hz * 1000u);
}

void core_scope_enable(int on)
{
    s_enabled = on ? 1 : 0;
    if (!on) scope_stop();
}

int core_scope_active(void) { return s_active; }

uint32_t core_scope_dropped(void) { return s_drop_total; }

/* ---- Sample / pump ---- */

void core_scope_sample(void)
{
    if (!s_active) return;

    uint32_t stamp = s_rate_mhz ? s_sample_n : core_millis();
    s_sample_n++; /* counts dropped samples too: the gap is how the host sees them */

    uint16_t head = s_head;
    uint16_t next = (uint16_t)(head + 1u);
    if (next == s_cap) next = 0;
    if (next == s_tail) {
        s_drop_total++;
        return;
    }

    uint8_t *d = &s_ring[(uint32_t)head * s_rec_bytes];
    memcpy(d, &stamp, 4);
    d += 4;
    for (uint8_t i = 0; i < s_count; i++) {
        const scope_chan_t *ch = &s_chan[i];
        /* One typed load per variable, so a value an ISR updates is never
         * seen half-written. The Cores are little-endian, as is the wire. */
        switch (k_wire_bytes[ch->type]) {
        case 4: {
            uint32_t v = (ch->type == CORE_SCOPE_BOOL)
                             ? (uint32_t)(*(const volatile bool *)ch->ptr ? 1u : 0u)
                             : *(const volatile uint32_t *)ch->ptr;
            memcpy(d, &v, 4);
            d += 4;
            break;
        }
        case 2: {
            uint16_t v = *(const volatile uint16_t *)ch->ptr;
            memcpy(d, &v, 2);
            d += 2;
            break;
        }
        default:
            *d++ = *(const volatile uint8_t *)ch->ptr;
            break;
        }
    }
    s_head = next;
}

/* Frame one timestamped record. */
static int emit_data(uint16_t tail, uint32_t now)
{
    const uint8_t *rec = &s_ring[(uint32_t)tail * s_rec_bytes];
    if (frame_begin(SCOPE_FRAME_DATA, (uint8_t)(SCOPE_DATA_HEADER + s_row_bytes), now) != 0) return 0;
    frame_put(&s_schema_id, 1);
    frame_put(rec, s_rec_bytes); /* [millis u32][row] is already the payload layout */
    frame_end();
    return 1;
}

/* Frame a run of records whose sample numbers are consecutive. */
static int emit_rows(uint16_t tail, uint16_t avail, uint32_t now)
{
    uint16_t room = (uint16_t)(CORE_SCOPE_TX_BYTES - s_tx_len);
    uint16_t pkt = s_link->packet_bytes ? s_link->packet_bytes : CORE_SCOPE_TX_BYTES;
    uint16_t budget = SCOPE_MAX_PAYLOAD + SCOPE_OVERHEAD;
    if (budget > pkt) budget = pkt; /* keep a frame inside one packet when it can be */
    if (budget > room) budget = room;
    if (budget < SCOPE_OVERHEAD + SCOPE_ROWS_HEADER + s_row_bytes) {
        if (room < SCOPE_OVERHEAD + SCOPE_ROWS_HEADER + s_row_bytes) return 0;
        budget = (uint16_t)(SCOPE_OVERHEAD + SCOPE_ROWS_HEADER + s_row_bytes); /* tiny packet: one row, split */
    }
    uint16_t max_rows = (uint16_t)((budget - SCOPE_OVERHEAD - SCOPE_ROWS_HEADER) / s_row_bytes);
    if (max_rows > avail) max_rows = avail;

    uint32_t n0;
    memcpy(&n0, &s_ring[(uint32_t)tail * s_rec_bytes], 4);
    uint16_t rows = 1, t = tail;
    while (rows < max_rows) {
        t = (uint16_t)(t + 1u == s_cap ? 0 : t + 1u);
        uint32_t n;
        memcpy(&n, &s_ring[(uint32_t)t * s_rec_bytes], 4);
        if (n != n0 + rows) break; /* a drop: the next frame restarts the count */
        rows++;
    }

    uint8_t head[SCOPE_ROWS_HEADER];
    head[0] = s_schema_id;
    put_u32(&head[1], n0);
    put_u32(&head[5], s_rate_mhz);
    (void)frame_begin(SCOPE_FRAME_ROWS, (uint8_t)(SCOPE_ROWS_HEADER + rows * s_row_bytes), now);
    frame_put(head, SCOPE_ROWS_HEADER);
    t = tail;
    for (uint16_t i = 0; i < rows; i++) {
        frame_put(&s_ring[(uint32_t)t * s_rec_bytes + 4u], s_row_bytes);
        t = (uint16_t)(t + 1u == s_cap ? 0 : t + 1u);
    }
    frame_end();
    return (int)rows;
}

void core_scope_pump(void)
{
    if (!s_enabled || !s_link || s_count == 0) return;
    if (!s_link->ready()) {
        if (s_active) scope_stop();
        return;
    }

    uint32_t now = core_millis();
    if (!s_active) {
        /* A host just arrived. Start clean: it gets the channel list first
         * and nothing sampled before it was listening. */
        s_tail = s_head;
        s_drop_seen = s_drop_total;
        if (s_schema_dirty) {
            s_id_crc = 0;
            schema_walk(id_put);
            s_schema_id = s_id_crc;
            s_schema_dirty = 0;
        }
        s_active = 1;
    }

    if (!s_schema_sent || (now - s_schema_ms) >= SCOPE_SCHEMA_MS) {
        if (emit_schema(now) == 0) {
            s_schema_sent = 1;
            s_schema_ms = now;
        }
    }

    if (s_schema_sent) {
        if (!s_rate_mhz) {
            /* Timestamped frames carry no sample number, so ring drops are
             * reported the way the host already counts loss: a seq gap. */
            uint32_t lost = s_drop_total - s_drop_seen;
            s_drop_seen += lost;
            s_seq = (uint8_t)(s_seq + (lost > 255u ? 255u : lost));
        }
        uint16_t tail = s_tail;
        for (;;) {
            uint16_t head = s_head;
            if (tail == head) break;
            uint16_t avail = (uint16_t)(head >= tail ? head - tail : s_cap - tail + head);
            int      done = s_rate_mhz ? emit_rows(tail, avail, now) : emit_data(tail, now);
            if (done == 0) break; /* staging is full: the rest waits in the ring */
            tail = (uint16_t)((tail + (uint16_t)done) % s_cap);
            s_tail = tail;
        }
    }

    flush(now);
}

void core_scope_update(void)
{
    if (!s_enabled) return;
    if (s_active) {
        if (s_interval_ms) {
            uint32_t now = core_millis();
            if ((now - s_last_ms) >= s_interval_ms) {
                s_last_ms = now;
                core_scope_sample();
            }
        } else {
            core_scope_sample();
        }
    }
    core_scope_pump();
}

/* ---- USB CDC link ---- */

#if defined(STM32L422xx) || defined(STM32H523xx)
#include "hal_usb_cdc.h"

static int usb_ready(void) { return hal_usb_cdc_connected(); }

static int usb_write(const uint8_t *buf, uint16_t len)
{
    int n = hal_usb_cdc_try_write(buf, len);
    return n > 0 ? n : 0;
}

/* The HAL queue takes a buffer whole or not at all and sends it ahead of any
 * later print, so a frame is never split and never interleaved with text. */
const core_scope_link_t core_scope_link_usb = {
    .init = 0,
    .ready = usb_ready,
    .write = usb_write,
    .packet_bytes = HAL_USB_CDC_TX_QUEUE_SIZE,
    .flush_ms = 0,
};
#endif

/* ---- Bluetooth LE link ---- */

#if defined(BLE_ENABLED) && BLE_ENABLED
#include "core_ble.h"

#define SCOPE_BLE_SERVICE_ID 0x5C00u /* Studio Link  */
#define SCOPE_BLE_CHAR_ID    0x5C01u /* Studio Scope */
/* core_ble does not report the negotiated ATT MTU. Every desktop and phone
 * central negotiates at least 185, so 180 fits without asking. */
#define SCOPE_BLE_PACKET     180u
/* A connection event comes every 15-50 ms and carries a few packets, so
 * frames are batched; they carry device time, so a batch plots correctly. */
#define SCOPE_BLE_FLUSH_MS   30u

static core_ble_char_t s_ble_ch;

static void ble_build(void)
{
    core_ble_svc_t svc = core_ble_add_service_id("Studio Link", SCOPE_BLE_SERVICE_ID);
    s_ble_ch = core_ble_add_char_id(svc, "Studio Scope", SCOPE_BLE_CHAR_ID, CORE_BLE_NOTIFY,
                                    CORE_BLE_BYTES(SCOPE_BLE_PACKET), 0, 0);
}

static void ble_link_init(void) { (void)core_ble_add_services(ble_build); }

static int ble_ready(void)
{
    return s_ble_ch && core_ble_connected() && core_ble_subscribed(s_ble_ch);
}

static int ble_write(const uint8_t *buf, uint16_t len)
{
    /* -1 = the stack's queue is full. The packet stays staged and is
     * offered again on a later pump(). */
    return core_ble_set_value(s_ble_ch, buf, len) == 0 ? (int)len : 0;
}

const core_scope_link_t core_scope_link_ble = {
    .init = ble_link_init,
    .ready = ble_ready,
    .write = ble_write,
    .packet_bytes = SCOPE_BLE_PACKET,
    .flush_ms = SCOPE_BLE_FLUSH_MS,
};
#endif

#endif /* CORE_SCOPE_ENABLED */
