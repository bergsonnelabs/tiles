/**
 * thread_settings.c — OpenThread settings (otPlatSettings*) kept in flash
 *
 * OpenThread stores what it must not forget across a reset here: the active
 * dataset, the network info (role, RLOC16, key sequence and the MAC / MLE
 * frame counters), the parent and the child table. Losing the frame counters
 * is what breaks a rebooted node (its neighbours reject its frames as
 * replays), so the settings live in flash, not only in RAM.
 *
 * In RAM: one buffer of {key, length, value} blocks, ST's format (their
 * platform/flash.c keeps only this and leaves saving to the application).
 * In flash: two reserved 8 KB pages, 0x080F4000-0x080F7FFF (pages 122-123,
 * __thread_settings_start/_end in sdk/device/stm32wba55hg_thread.ld), below
 * the BLE-update page, core_nvm and the bond store. Every change schedules a
 * snapshot of the whole buffer 250 ms later (one write for a burst of
 * changes): a 16-byte header {magic, sequence, length, CRC-32} and the data,
 * appended in the current page. When it is full the other page is erased and
 * the snapshot goes there. At boot the valid snapshot with the highest
 * sequence number wins; a torn one (power cut mid-write) fails its CRC, or
 * raises an ECC error that core_nvm's NMI handler absorbs, and the one before
 * it is used.
 *
 * Erase / program go through ST's flash manager (wpan_flash.c), which on a
 * Thread build does them directly (no radio time windows on this link layer):
 * a page erase stalls the CPU ~1.5 ms. On a BLE + Thread build it waits for a
 * radio-free window, as for BLE (wpan_flash.h). Snapshots are rare (attach,
 * role change, every 1000 frames for the frame counters). In BLE + Thread
 * builds the pages are outside the BLE update's staging area (su_ota.h), so
 * the settings survive an update.
 *
 * The RAM functions are adapted from STM32CubeWBA V1.6.1
 * Middlewares/ST/STM32_WPAN/thread/openthread/platform/flash.c
 * Copyright (c) 2023 STMicroelectronics. All rights reserved.
 * Licensed under the terms in sdk/thread/LICENSE-ST.txt.
 * tiles: buffer in .bss (not .noinit), flash persistence added.
 */

#include <stdint.h>
#include <string.h>
#include <openthread/instance.h>
#include <openthread/platform/settings.h>
#include "stm32_timer.h"
#include "stm32_seq.h"
#include "app_conf.h"
#include "wpan_flash.h"
#include "thread_app.h"

/* ---- Layout ---- */

#define SETTINGS_BUF_SIZE   2048u                   /* RAM buffer (ST: 1024) */
#define FLASH_BASE_ADDR     0x08000000UL
#define FLASH_PAGE          8192u
#define REC_MAGIC           0x3153544FUL            /* "OTS1" */
#define REC_HDR             16u
#define SAVE_DELAY_MS       250u

extern const uint8_t __thread_settings_start[];   /* linker script */
extern const uint8_t __thread_settings_end[];
extern volatile uint8_t _core_flash_ecc_thread;   /* core_nvm.c NMI handler */

#define AREA_START  ((uint32_t)(uintptr_t)__thread_settings_start)
#define AREA_END    ((uint32_t)(uintptr_t)__thread_settings_end)

typedef struct {
    uint32_t magic;
    uint32_t seq;
    uint32_t len;
    uint32_t crc;
} rec_hdr_t;

/* ---- RAM store (ST format) ---- */

typedef struct __attribute__((packed)) {
    uint16_t key;
    uint16_t length;
} block_t;

static uint8_t  s_buf[SETTINGS_BUF_SIZE] __attribute__((aligned(4)));
static uint32_t s_used;                 /* bytes of s_buf in use */

/* ---- Flash state ---- */

static uint32_t s_seq;                  /* sequence of the newest snapshot */
static uint32_t s_page_addr;            /* page holding it (0: none yet) */
static uint32_t s_next_off;             /* where the next snapshot goes in it */
static uint8_t  s_dirty;
static uint8_t  s_loaded;
static uint32_t s_saves;
static int      s_last_error;
static uint8_t  s_failures;             /* consecutive failed snapshots */
static UTIL_TIMER_Object_t s_save_timer;

/* Snapshot staging: the flash manager runs other tasks while it waits, and
 * one of them may change s_buf meanwhile. */
static uint32_t s_img[(REC_HDR + SETTINGS_BUF_SIZE) / 4u];

/* ============================================================
 * Helpers
 * ============================================================ */

static uint32_t crc32(const uint8_t *p, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFUL;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++)
            c = (c >> 1) ^ (0xEDB88320UL & (0u - (c & 1u)));
    }
    return ~c;
}

static uint32_t round16(uint32_t n)
{
    return (n + 15u) & ~15u;
}

/* The ICACHE (on in radio builds) also caches data reads of flash: invalidate
 * after an erase / program so a read-back sees the array (RM0493 §8.4.8; the
 * same as core_nvm.c and the BLE update). */
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

/* Copy from flash, noting an ECC error (a torn quad-word) on the way. */
static int flash_read(uint32_t addr, void *dst, uint32_t n)
{
    _core_flash_ecc_thread = 0;
    memcpy(dst, (const void *)(uintptr_t)addr, n);
    __asm volatile ("dsb 0xF" ::: "memory");
    return _core_flash_ecc_thread ? -1 : 0;
}

static int range_blank(uint32_t addr, uint32_t n)
{
    for (uint32_t off = 0; off < n; off += 4u) {
        uint32_t w;
        if (flash_read(addr + off, &w, 4u) != 0 || w != 0xFFFFFFFFUL)
            return 0;
    }
    return 1;
}

/* Walk one page; return the newest valid record's seq (0: none), with its
 * offset and length, and where the records end. */
static uint32_t scan_page(uint32_t page, uint32_t *rec_off, uint32_t *rec_len, uint32_t *end_off)
{
    uint32_t best = 0, off = 0;

    while (off + REC_HDR <= FLASH_PAGE) {
        rec_hdr_t h;
        if (flash_read(page + off, &h, sizeof h) != 0)
            break;                                  /* torn header: stop here */
        if (h.magic != REC_MAGIC || h.len > SETTINGS_BUF_SIZE ||
            off + REC_HDR + round16(h.len) > FLASH_PAGE)
            break;                                  /* blank (or garbage): end */
        uint32_t next = off + REC_HDR + round16(h.len);
        if (flash_read(page + off + REC_HDR, s_img, h.len) == 0 &&
            crc32((const uint8_t *)s_img, h.len) == h.crc && h.seq > best) {
            best = h.seq;
            *rec_off = off;
            *rec_len = h.len;
        }
        off = next;
    }
    *end_off = off;
    return best;
}

/* ============================================================
 * Boot: load the newest snapshot
 * ============================================================ */

void thread_settings_load(void)
{
    s_used = 0;
    s_seq = 0;
    s_page_addr = 0;
    s_next_off = 0;

    for (uint32_t page = AREA_START; page + FLASH_PAGE <= AREA_END; page += FLASH_PAGE) {
        uint32_t off = 0, len = 0, end = 0;
        uint32_t seq = scan_page(page, &off, &len, &end);
        if (seq > s_seq) {
            if (flash_read(page + off + REC_HDR, s_buf, len) == 0) {
                s_seq = seq;
                s_used = len;
                s_page_addr = page;
                s_next_off = end;
            }
        }
    }
    s_loaded = 1;
}

/* ============================================================
 * Save: snapshot the buffer to flash
 * ============================================================ */

static void save_timer_cb(void *arg)
{
    (void)arg;
    UTIL_SEQ_SetTask(1U << CFG_TASK_OT_SETTINGS, TASK_PRIO_SETTINGS);
}

static void mark_dirty(void)
{
    s_dirty = 1;
    s_failures = 0;
    if (!s_loaded)
        return;                                   /* before init: saved at the next change */
    static uint8_t timer_made;
    if (!timer_made) {
        UTIL_TIMER_Create(&s_save_timer, SAVE_DELAY_MS, UTIL_TIMER_ONESHOT, save_timer_cb, NULL);
        timer_made = 1;
    }
    UTIL_TIMER_Stop(&s_save_timer);
    UTIL_TIMER_Start(&s_save_timer);
}

static int write_snapshot(void)
{
    uint32_t len = s_used;
    uint32_t total = REC_HDR + round16(len);

    /* Stage header + data (padding 0xFF, like erased flash). */
    memset(s_img, 0xFF, total);
    memcpy((uint8_t *)s_img + REC_HDR, s_buf, len);
    rec_hdr_t h = { REC_MAGIC, s_seq + 1u, len, crc32(s_buf, len) };
    memcpy(s_img, &h, sizeof h);

    /* Same page if the snapshot fits behind the last one and that space is
     * still blank; otherwise erase the other page and start it. */
    uint32_t page = s_page_addr, off = s_next_off;
    if (page == 0 || off + total > FLASH_PAGE || !range_blank(page + off, total)) {
        page = (page == AREA_START) ? AREA_START + FLASH_PAGE : AREA_START;
        off = 0;
        if (wpan_flash_erase_page((page - FLASH_BASE_ADDR) / FLASH_PAGE) != 0)
            return -1;
        icache_invalidate();
    }
    int rc = wpan_flash_program(page + off, s_img, total / 4u);
    icache_invalidate();
    if (rc != 0)
        return -2;

    s_seq = h.seq;
    s_page_addr = page;
    s_next_off = off + total;
    s_saves++;
    return 0;
}

void thread_settings_process(void)
{
    if (!s_dirty || !wpan_flash_ready())
        return;
    s_dirty = 0;
    int rc = write_snapshot();
    if (rc == 0) {
        s_failures = 0;
        return;
    }
    s_last_error = rc;
    s_next_off = FLASH_PAGE;      /* retry in the other page; keep this one's snapshots */
    s_dirty = 1;
    /* Retry twice; after that wait for the next change rather than erase
     * pages every 250 ms against a flash that keeps failing. */
    if (++s_failures < 3u) {
        UTIL_TIMER_Stop(&s_save_timer);
        UTIL_TIMER_Start(&s_save_timer);
    }
}

uint32_t thread_settings_save_count(void) { return s_saves; }
int thread_settings_last_error(void) { return s_last_error; }

/* ============================================================
 * otPlatSettings* (RAM store; ST's logic)
 * ============================================================ */

void otPlatSettingsInit(otInstance *aInstance, const uint16_t *aSensitiveKeys, uint16_t aSensitiveKeysLength)
{
    (void)aInstance;
    (void)aSensitiveKeys;
    (void)aSensitiveKeysLength;
    if (!s_loaded)
        thread_settings_load();
}

void otPlatSettingsDeinit(otInstance *aInstance)
{
    (void)aInstance;
}

otError otPlatSettingsGet(otInstance *aInstance, uint16_t aKey, int aIndex, uint8_t *aValue, uint16_t *aValueLength)
{
    (void)aInstance;
    uint32_t pos = 0;
    int index = 0;
    uint16_t value_len = 0;
    otError error = OT_ERROR_NOT_FOUND;

    while (pos + sizeof(block_t) <= s_used) {
        block_t b;
        memcpy(&b, &s_buf[pos], sizeof b);
        if (b.key == aKey) {
            if (index == aIndex) {
                uint16_t n = b.length;
                if (aValue != NULL && aValueLength != NULL) {
                    if (n > *aValueLength)
                        n = *aValueLength;
                    memcpy(aValue, &s_buf[pos + sizeof b], n);
                }
                value_len = b.length;
                error = OT_ERROR_NONE;
                break;
            }
            index++;
        }
        pos += sizeof b + b.length;
    }
    if (aValueLength != NULL)
        *aValueLength = value_len;
    return error;
}

otError otPlatSettingsAdd(otInstance *aInstance, uint16_t aKey, const uint8_t *aValue, uint16_t aValueLength)
{
    (void)aInstance;
    uint32_t n = sizeof(block_t) + aValueLength;
    if (s_used + n > SETTINGS_BUF_SIZE)
        return OT_ERROR_NO_BUFS;

    block_t b = { aKey, aValueLength };
    memcpy(&s_buf[s_used], &b, sizeof b);
    if (aValueLength)
        memcpy(&s_buf[s_used + sizeof b], aValue, aValueLength);
    s_used += n;
    mark_dirty();
    return OT_ERROR_NONE;
}

otError otPlatSettingsDelete(otInstance *aInstance, uint16_t aKey, int aIndex)
{
    (void)aInstance;
    uint32_t pos = 0;
    int index = 0;

    while (pos + sizeof(block_t) <= s_used) {
        block_t b;
        memcpy(&b, &s_buf[pos], sizeof b);
        uint32_t n = sizeof b + b.length;
        if (b.key == aKey) {
            if (index == aIndex || aIndex == -1) {
                memmove(&s_buf[pos], &s_buf[pos + n], s_used - (pos + n));
                s_used -= n;
                mark_dirty();
                return OT_ERROR_NONE;
            }
            index++;
        }
        pos += n;
    }
    return OT_ERROR_NOT_FOUND;
}

otError otPlatSettingsSet(otInstance *aInstance, uint16_t aKey, const uint8_t *aValue, uint16_t aValueLength)
{
    uint32_t pos = 0;

    /* Remove every instance of the key, then add the new value. */
    while (pos + sizeof(block_t) <= s_used) {
        block_t b;
        memcpy(&b, &s_buf[pos], sizeof b);
        uint32_t n = sizeof b + b.length;
        if (b.key == aKey) {
            memmove(&s_buf[pos], &s_buf[pos + n], s_used - (pos + n));
            s_used -= n;
        } else {
            pos += n;
        }
    }
    return otPlatSettingsAdd(aInstance, aKey, aValue, aValueLength);
}

void otPlatSettingsWipe(otInstance *aInstance)
{
    (void)aInstance;
    s_used = 0;
    mark_dirty();
}
