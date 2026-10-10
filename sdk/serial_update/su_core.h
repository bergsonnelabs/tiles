/**
 * su_core.h — Serial update protocol state machine (hardware-free)
 *
 * The flashers (flasher_l4.c, flasher_h5.c) and the native test harness
 * (test/su_sim.c) all
 * drive this: feed it received bytes, tick it, and give it the handful of
 * operations below. Everything that decides whether the Core can be left
 * unbootable lives here, so the host tests exercise the real logic.
 *
 * Safety order: page 0 (the vector table) is checked before anything is
 * erased, erased before any other page is touched, held in RAM, and written
 * only after the whole image has been verified. Until then the chip is
 * "empty", which the L4 boots into the ROM bootloader (the H5 needs BOOT0
 * for that: docs/serial-update-protocol.md §5).
 */

#ifndef SU_CORE_H
#define SU_CORE_H

#include <stdint.h>
#include "su_protocol.h"

typedef struct {
    uint32_t flash_base;          /* where the image starts (0x08000000) */
    uint32_t flash_size;          /* bytes */
    uint32_t image_limit;         /* largest image, bytes: flash_size less the
                                     core_nvm pages at the top, never erased */
    uint32_t page_size;           /* bytes, <= SU_MAX_PAYLOAD */
    uint32_t dev_id;              /* DEV_ID[11:0] */
    uint32_t sram_start, sram_end;/* plausible initial SP range */
    const uint8_t *flash_read;    /* memory-mapped view of flash_base */

    int  (*erase_page)(uint32_t page);                       /* 0 = ok */
    int  (*program)(uint32_t addr, const uint8_t *buf, uint32_t len); /* 0 = ok; len % SU_PROG_UNIT == 0 */
    void (*send)(const char *line, uint32_t len);            /* one reply line */
    void (*reboot)(void);         /* leave the flasher: L4 resets (the chip re-checks page 0
                                     as it boots), H5 starts the image in place */
#if SU_STAGING
    /* Staging mode (BLE update, docs/ble-update-protocol.md). flash_base is
     * where the image is LINKED (the vector check); it is written at
     * flash_base + the staging offset stage() picks. page_size is the chunk
     * size, erase_size the flash page; image_limit is what READY reports. */
    uint32_t erase_size;
    /** BEGIN: does an image of `size` fit? 0 and *stage_off (bytes from
     *  flash_base, a multiple of erase_size), or -1. */
    int  (*stage)(uint32_t size, uint32_t *stage_off);
    /** QUERY: 1 and *key_id if a board key is provisioned, else 0. */
    int  (*key_info)(uint32_t *key_id);
    /** QUERY: CRC-32 of the running image (what was flashed: its .bin). */
    uint32_t (*image_crc)(void);
    /** CHALLENGE: a fresh nonce. 0, or SU_ERR_NOKEY / SU_ERR_RNG. */
    int  (*challenge)(uint8_t nonce[16]);
    /** AUTH: 1 if tag == CMAC(K, "SUA1" ‖ nonce). */
    int  (*auth_check)(const uint8_t nonce[16], const uint8_t tag[16]);
    /** The running image tag: start at BEGIN, fed each new chunk in order,
     *  checked once at END against tag = CMAC(K, "SUE1" ‖ nonce ‖ image_tag). */
    void (*mac_start)(uint32_t size, uint32_t crc);
    void (*mac_update)(const uint8_t *p, uint32_t len);
    int  (*mac_check)(const uint8_t nonce[16], const uint8_t tag[16]);
    /** END, all checks passed: write the install marker. `keep` is the kept-
     *  page bitmap (bit p of keep[p / 32]: image page p, of erase_size bytes,
     *  stays as the running image has it and was never staged). 0 = ok. */
    int  (*commit)(uint32_t stage_off, uint32_t size, uint32_t crc, const uint32_t keep[2]);
    /** SAME (delta): the running image's size in bytes from flash_base (its
     *  end, __flash_image_end). An S run must end at or below it; its bytes
     *  are read through flash_read. Staging always lies above it. An S that
     *  covers exactly one erase_size page (or the image's partial last page)
     *  keeps that page: read for the CRC and the tag, never staged. */
    uint32_t run_size;
    /** Feed the watchdog between the pieces of a long S run (may be 0). */
    void (*feed)(void);
#endif
} su_ops_t;

/** Initialise with the platform operations. */
void su_core_init(const su_ops_t *ops);

/** Append received bytes; frames are handled as they complete.
 *  Returns how many bytes were accepted (less than len if the buffer is full). */
uint32_t su_core_rx(const uint8_t *buf, uint32_t len);

/** Free space in the receive buffer (the flasher stops reading USB below one packet). */
uint32_t su_core_rx_space(void);

/** Advance the idle timer. Reboots into the old app after SU_IDLE_TIMEOUT_MS
 *  with no BEGIN, but only while flash is untouched. */
void su_core_tick(uint32_t elapsed_ms);

/** True once page 0 has been erased (the Core is not bootable until END). */
int su_core_page0_erased(void);

#if SU_STAGING
/** The link went away: forget the authentication and any partial frame, keep
 *  the transfer (size, CRC, next offset, image tag) so a new link can resume. */
void su_core_link_reset(void);
#endif

/** CRC-32 (IEEE, as zlib / JS). Continue a running value by passing it back in. */
uint32_t su_crc32(uint32_t crc, const uint8_t *buf, uint32_t len);

#endif /* SU_CORE_H */
