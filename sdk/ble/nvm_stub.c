/**
 * nvm_stub.c — Flash-backed NVM for BLE bonding data
 *
 * Uses the last flash page (page 127, 8KB) on WBA55 to persist
 * BLE security database records across power cycles.
 *
 * Layout: simple append-only log of records. Each record has a
 * 4-byte header (magic + type + size) followed by data, aligned
 * to 16 bytes: the WBA55 programs flash in 128-bit quad-words
 * (RM0493 §7.3.7), and the controller completes a program only after
 * all four words are written. The 8-byte double-word writes this used
 * to issue never completed a line, so no bond ever reached flash.
 *
 * When the page fills up, it's erased and records are compacted.
 * For simplicity, the current implementation uses RAM as a write
 * buffer and flushes to flash on NVM_Add.
 *
 * Page 127 only: core_nvm's store is pages 125-126 just below
 * (sdk/core/core_nvm.h), and the linker scripts end the application at
 * 0x080FA000 to leave all three alone.
 *
 * Once BLE is up (ble_flash_ready()), the flush goes through ST's flash
 * manager like core_nvm's writes, so the erase and the programming run in
 * time windows between radio events. It is asynchronous: NVM_Add is called
 * from inside the BLE stack, which must not block on the radio it runs. A
 * flush requested while one is in flight runs again when it completes, so
 * the page always ends up matching the RAM copy. (A power cut between the
 * erase and the write still loses the bonds, as it always did.)
 */

#include <stdint.h>
#include <string.h>
#include "nvm.h"
#include "ll_common.h"
#include "ll_flash.h"
#include "ble_flash.h"
#include "flash_manager.h"
#include "flash_driver.h"

/* NVM flash page — last page of 1MB flash */
#define NVM_PAGE         127
#define NVM_FLASH_ADDR   (FLASH_START + NVM_PAGE * FLASH_PAGE_SIZE)
#define NVM_FLASH_SIZE   FLASH_PAGE_SIZE  /* 8KB */

/* Record header */
#define NVM_RECORD_MAGIC 0xBE
typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  type;
    uint16_t size;
} nvm_record_header_t;

/* Record alignment = flash program unit (quad-word). */
#define NVM_ALIGN        16u
#define NVM_ALIGN_UP(n)  (((n) + (NVM_ALIGN - 1u)) & ~(NVM_ALIGN - 1u))

/* RAM mirror for read operations (word-aligned: it is read as uint32_t) */
#define NVM_RAM_SIZE     4096
static uint8_t nvm_ram[NVM_RAM_SIZE] __attribute__((aligned(16)));
static uint8_t nvm_initialized;
static uint16_t nvm_write_pos;

/* Read cursor */
static uint16_t nvm_read_cursor;

/* ---- Flash helpers ---- */

#if defined(STM32WBA55xx)

static void nvm_flash_write(uint32_t addr, const uint8_t *data, uint16_t len)
{
    ll_flash_unlock();
    ll_flash_clear_errors();

    /* Program in quad-words (16 bytes, 16-byte aligned); pad the last one
     * with 0xFF (erased). addr is the page start, so it is aligned. */
    for (uint16_t i = 0; i < len; i += NVM_ALIGN) {
        uint32_t w[4];
        for (uint16_t k = 0; k < 4u; k++) {
            uint16_t at = (uint16_t)(i + 4u * k);
            if (at + 4u <= len) {
                memcpy(&w[k], &data[at], 4);
            } else {
                uint8_t b[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
                for (uint16_t j = 0; at + j < len && j < 4u; j++) b[j] = data[at + j];
                memcpy(&w[k], b, 4);
            }
        }
        ll_flash_program_qword(addr + i, w);
    }

    ll_flash_lock();
}

static void nvm_flash_erase(void)
{
    ll_flash_unlock();
    ll_flash_erase_page(NVM_PAGE);
    ll_flash_lock();
}

static void nvm_load_from_flash(void)
{
    /* Copy flash contents to RAM mirror */
    const uint8_t *flash = (const uint8_t *)NVM_FLASH_ADDR;

    nvm_write_pos = 0;
    memset(nvm_ram, 0xFF, NVM_RAM_SIZE);

    /* Scan for valid records */
    uint16_t pos = 0;
    while (pos + sizeof(nvm_record_header_t) < NVM_FLASH_SIZE) {
        const nvm_record_header_t *hdr = (const nvm_record_header_t *)&flash[pos];
        if (hdr->magic != NVM_RECORD_MAGIC) break;

        uint16_t total = sizeof(nvm_record_header_t) + hdr->size;
        uint16_t aligned = NVM_ALIGN_UP(total);

        if (pos + aligned > NVM_FLASH_SIZE) break;
        if (nvm_write_pos + aligned > NVM_RAM_SIZE) break;

        memcpy(&nvm_ram[nvm_write_pos], &flash[pos], total);
        nvm_write_pos += aligned;
        pos += aligned;
    }
}

/* ---- Asynchronous flush through the flash manager ---- */

static volatile uint8_t  fm_flush_busy;     /* an erase + write is in flight */
static volatile uint8_t  fm_flush_again;    /* RAM changed meanwhile: flush again */
static uint16_t          fm_flush_len;      /* bytes the write in flight covers */

static void fm_flush_kick(void);
static void fm_erase_cb(FM_FlashOp_Status_t st);
static void fm_write_cb(FM_FlashOp_Status_t st);
static FM_CallbackNode_t fm_erase_node = { .Callback = fm_erase_cb };
static FM_CallbackNode_t fm_write_node = { .Callback = fm_write_cb };

static void fm_flush_done(void)
{
    (void)FD_TakeHwError();     /* a failed flush is retried by the next one */
    fm_flush_busy = 0;
    if (fm_flush_again) fm_flush_kick();
}

static void fm_start_write(void)
{
    FM_Cmd_Status_t st = FM_Write((uint32_t *)(void *)nvm_ram, (uint32_t *)NVM_FLASH_ADDR,
                                  (int32_t)(fm_flush_len / 4u), &fm_write_node);
    if (st == FM_ERROR) fm_flush_done();
    /* FM_BUSY: fm_write_cb(AVAILABLE) asks again */
}

static void fm_start_erase(void)
{
    FM_Cmd_Status_t st = FM_Erase(NVM_PAGE, 1u, &fm_erase_node);
    if (st == FM_ERROR) fm_flush_done();
    else if (st == FM_OK) (void)FD_TakeHwError();   /* stale, from an earlier op */
}

static void fm_erase_cb(FM_FlashOp_Status_t st)
{
    if (st == FM_OPERATION_AVAILABLE) { fm_start_erase(); return; }
    if (FD_TakeHwError() || fm_flush_len == 0u) { fm_flush_done(); return; }
    fm_start_write();
}

static void fm_write_cb(FM_FlashOp_Status_t st)
{
    if (st == FM_OPERATION_AVAILABLE) { fm_start_write(); return; }
    fm_flush_done();
}

static void fm_flush_kick(void)
{
    if (fm_flush_busy) { fm_flush_again = 1; return; }
    fm_flush_busy = 1;
    fm_flush_again = 0;
    fm_flush_len = nvm_write_pos;   /* 16-byte multiple (NVM_ALIGN) */
    fm_start_erase();
}

static void nvm_flush_to_flash(void)
{
    if (ble_flash_ready()) {
        fm_flush_kick();
        return;
    }
    /* Before BLE is up nothing else touches flash: write it directly.
     * Always erase, so NVM_Discard (write_pos 0) clears the stored bonds too;
     * it used to return early and leave them in flash. */
    nvm_flash_erase();
    if (nvm_write_pos)
        nvm_flash_write(NVM_FLASH_ADDR, nvm_ram, nvm_write_pos);
}

#else

/* Non-WBA55: RAM-only (no flash persistence) */
static void nvm_load_from_flash(void) { memset(nvm_ram, 0xFF, NVM_RAM_SIZE); nvm_write_pos = 0; }
static void nvm_flush_to_flash(void) { }

#endif

/* ---- Public API ---- */

void NVM_Init(void *buffer, uint32_t offset, uint32_t size)
{
    (void)buffer; (void)offset; (void)size;
    nvm_load_from_flash();
    nvm_initialized = 1;
    nvm_read_cursor = 0;
}

int NVM_Add(uint8_t type, const uint8_t *data, uint16_t size,
            const uint8_t *extra_data, uint16_t extra_size)
{
    if (!nvm_initialized) return -1;

    uint16_t total_data = size + extra_size;
    uint16_t record_size = sizeof(nvm_record_header_t) + total_data;
    uint16_t aligned = NVM_ALIGN_UP(record_size);

    if (nvm_write_pos + aligned > NVM_RAM_SIZE) {
        /* NVM full — erase and start fresh */
        memset(nvm_ram, 0xFF, NVM_RAM_SIZE);
        nvm_write_pos = 0;
    }

    /* Write header */
    nvm_record_header_t *hdr = (nvm_record_header_t *)&nvm_ram[nvm_write_pos];
    hdr->magic = NVM_RECORD_MAGIC;
    hdr->type = type;
    hdr->size = total_data;

    /* Write data */
    memcpy(&nvm_ram[nvm_write_pos + sizeof(nvm_record_header_t)], data, size);
    if (extra_data && extra_size > 0) {
        memcpy(&nvm_ram[nvm_write_pos + sizeof(nvm_record_header_t) + size],
               extra_data, extra_size);
    }

    nvm_write_pos += aligned;

    /* Flush to flash for persistence */
    nvm_flush_to_flash();

    return 0;
}

int NVM_Get(int mode, uint8_t type, uint16_t offset, uint8_t *data, uint16_t size)
{
    if (!nvm_initialized) return -3;  /* EOF */

    if (mode == 0) {  /* NVM_FIRST */
        nvm_read_cursor = 0;
    }

    /* Search from cursor */
    while (nvm_read_cursor < nvm_write_pos) {
        nvm_record_header_t *hdr = (nvm_record_header_t *)&nvm_ram[nvm_read_cursor];
        if (hdr->magic != NVM_RECORD_MAGIC) return -3;

        uint16_t record_start = nvm_read_cursor + sizeof(nvm_record_header_t);
        uint16_t aligned = NVM_ALIGN_UP(sizeof(nvm_record_header_t) + hdr->size);
        nvm_read_cursor += aligned;

        if (hdr->type == type || type == 0xFF) {
            uint16_t avail = (offset < hdr->size) ? (hdr->size - offset) : 0;
            uint16_t copy = (size < avail) ? size : avail;
            if (data && copy > 0) {
                memcpy(data, &nvm_ram[record_start + offset], copy);
            }
            return (int)copy;
        }
    }

    return -3;  /* EOF */
}

int NVM_Compare(uint16_t offset, const uint8_t *data, uint16_t size)
{
    if (!nvm_initialized) return 1;
    if (offset + size > nvm_write_pos) return 1;
    return memcmp(&nvm_ram[offset], data, size);
}

void NVM_Discard(int mode)
{
    (void)mode;
    if (nvm_initialized) {
        memset(nvm_ram, 0xFF, NVM_RAM_SIZE);
        nvm_write_pos = 0;
        nvm_read_cursor = 0;
        nvm_flush_to_flash();
    }
}
