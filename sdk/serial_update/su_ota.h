/**
 * su_ota.h — BLE update on Core.ST.W5: reserved page, install marker, copier
 *
 * Hardware-free. The running app (sdk/ble/ble_update.c) uses it to read the
 * board key and to write the install marker; the boot hook's copier
 * (sdk/ble/ble_update_boot.c) uses it to finish an install; the host tests
 * (sdk/serial_update/test/ota_sim.c) drive both on a simulated flash with
 * power cuts. Layout and rules: docs/ble-update-protocol.md §7-8.
 *
 * Every function here may run from the copier's SRAM copy while flash is
 * being rewritten, so the target build places this whole file in `.ota_ram`
 * (SU_OTA_RAM) and nothing here may call out of it: no libc, no division
 * helpers, no tables outside the section. All flash access goes through
 * su_ota_io_t.
 */

#ifndef SU_OTA_H
#define SU_OTA_H

#include <stdint.h>

#ifndef SU_OTA_RAM
#define SU_OTA_RAM                      /* host build: ordinary functions */
#endif
#ifndef SU_OTA_RAM_DATA
#define SU_OTA_RAM_DATA
#endif

/* ---- Geometry (STM32WBA55, RM0493 §7.3.1: 128 pages of 8 KB) ---- */
#define SU_OTA_FLASH_BASE   0x08000000UL
#define SU_OTA_PAGE         8192u
#define SU_OTA_QW           16u                       /* programming unit, §7.3.7 */
#define SU_OTA_RES_PAGE     124u                      /* key + install marker */
#define SU_OTA_RES_OFF      (SU_OTA_RES_PAGE * SU_OTA_PAGE)   /* 0xF8000 */
#define SU_OTA_APP_LIMIT    SU_OTA_RES_OFF            /* 1015808: images end below page 124 */

/* ---- Page 124 ---- */
#define SU_OTA_KEY_OFF      0x000u                    /* 32-byte key record */
#define SU_OTA_LOG_OFF      0x100u                    /* marker log: 16-byte records */
#define SU_OTA_LOG_END      SU_OTA_PAGE
#define SU_OTA_LOG_SLOTS    ((SU_OTA_LOG_END - SU_OTA_LOG_OFF) / SU_OTA_QW)   /* 496 */
#define SU_OTA_MIN_FREE     8u                        /* compact below this many free slots */

#define SU_OTA_KEY_MAGIC    0x314B5553UL              /* "SUK1" */
#define SU_OTA_COPA         0x41504F43UL              /* "COPA" */
#define SU_OTA_KEEP         0x5045454BUL              /* "KEEP": kept-page bitmap, between COPY-A and COPY-B */
#define SU_OTA_COPB         0x33504F43UL              /* "COP3": COPY-B, v3 (old size + CRC; check covers KEEP) */
#define SU_OTA_COPB_V2      0x32504F43UL              /* "COP2": v2 (no KEEP), no longer read */
#define SU_OTA_COPB_V1      0x42504F43UL              /* "COPB": v1 (page-0 CRCs), no longer read */
#define SU_OTA_MAX_PAGES    64u                       /* the keep bitmap's reach; images are <= 62 pages */
#define SU_OTA_STRT         0x54525453UL              /* "STRT": the copier began erasing */
#define SU_OTA_DONE         0x454E4F44UL              /* "DONE" */

/* Flash access, offsets from 0x08000000. */
typedef struct {
    /** Copy len bytes. 0, or -1 if any of it read with an uncorrectable ECC error. */
    int  (*read)(void *ctx, uint32_t off, void *dst, uint32_t len);
    /** Erase one 8 KB page. 0 = ok. */
    int  (*erase)(void *ctx, uint32_t page);
    /** Program len bytes (a multiple of 16, 16-aligned) from word-aligned src. 0 = ok. */
    int  (*program)(void *ctx, uint32_t off, const uint32_t *src, uint32_t len);
    /** Feed the watchdog (may be 0). */
    void (*feed)(void *ctx);
    void *ctx;
} su_ota_io_t;

typedef struct {
    uint32_t key_id;
    uint8_t  key[16];
} su_ota_key_t;

typedef struct {
    int      pending;             /* a COPY with no DONE after it */
    int      started;             /* ... and a START after it: the copy began */
    uint32_t stage;               /* staging address (0x08000000 + offset), as recorded */
    uint32_t size, crc;           /* image */
    uint32_t old_size, old_crc;   /* the image that was running at the commit */
    uint32_t keep[2];             /* kept pages: bit p of keep[p / 32] = image page p */
    uint32_t check;               /* COPY-B[3]: what a START / DONE must quote */
    uint32_t log_end;             /* first slot after the last used one (page offset) */
    uint32_t free_slots;
} su_ota_state_t;

/* Results of su_ota_boot(). */
#define SU_OTA_BOOT_NONE       0  /* nothing pending: boot on */
#define SU_OTA_BOOT_MARKED     1  /* it was already installed; DONE written: boot on */
#define SU_OTA_BOOT_ABANDONED  2  /* pending install not possible; DONE(abandoned): boot on */
#define SU_OTA_BOOT_INSTALLED  3  /* copied and verified, DONE written: reset now */
#define SU_OTA_BOOT_FAILED     4  /* flash failed mid-copy: nothing bootable, stop */

/** CRC-32 (IEEE, zlib), continued by passing the previous value. */
uint32_t su_ota_crc32(uint32_t crc, const uint8_t *p, uint32_t n) SU_OTA_RAM;

/** Read the key record. 1 = valid (k filled), 0 = none / torn. */
int su_ota_key_read(const su_ota_io_t *io, su_ota_key_t *k) SU_OTA_RAM;

/** Scan the marker log. */
void su_ota_scan(const su_ota_io_t *io, su_ota_state_t *s) SU_OTA_RAM;

/** Staging offset for an image of `size` bytes beside a running app ending at
 *  `app_end` (offsets from 0x08000000). 0 = fits (*stage set), -1 = doesn't. */
int su_ota_fit(uint32_t app_end, uint32_t size, uint32_t *stage) SU_OTA_RAM;

/** The largest image su_ota_fit() accepts beside an app ending at app_end. */
uint32_t su_ota_limit(uint32_t app_end) SU_OTA_RAM;

/* What su_ota_commit() records. */
typedef struct {
    uint32_t stage;               /* staging offset (from 0x08000000) */
    uint32_t size, crc;           /* the new image */
    uint32_t old_size, old_crc;   /* the running image: its end and CRC-32 (the reflash guard) */
    uint32_t keep[2];             /* kept pages (never staged, never rewritten) */
} su_ota_commit_t;

/** Record a verified staged image as pending (COPY-A, KEEP, COPY-B; compacts
 *  the log first if it is nearly full). 0 = written and read back, -1 = flash
 *  error or an implausible request. */
int su_ota_commit(const su_ota_io_t *io, const su_ota_commit_t *c) SU_OTA_RAM;

/** Boot-time decision + copy (docs §8.2). Interrupts off, no radio. */
int su_ota_boot(const su_ota_io_t *io) SU_OTA_RAM;

#endif /* SU_OTA_H */
