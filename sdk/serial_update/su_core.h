/**
 * su_core.h — Serial update protocol state machine (hardware-free)
 *
 * The flasher (flasher_l4.c) and the native test harness (test/su_sim.c) both
 * drive this: feed it received bytes, tick it, and give it the handful of
 * operations below. Everything that decides whether the Core can be left
 * unbootable lives here, so the host tests exercise the real logic.
 *
 * Safety order: page 0 (the vector table) is checked before anything is
 * erased, erased before any other page is touched, held in RAM, and written
 * only after the whole image has been verified. Until then the chip is
 * "empty", which the L4 boots into the ROM bootloader.
 */

#ifndef SU_CORE_H
#define SU_CORE_H

#include <stdint.h>
#include "su_protocol.h"

typedef struct {
    uint32_t flash_base;          /* where the image starts (0x08000000) */
    uint32_t flash_size;          /* bytes */
    uint32_t page_size;           /* bytes, <= SU_MAX_PAYLOAD */
    uint32_t dev_id;              /* DEV_ID[11:0] */
    uint32_t sram_start, sram_end;/* plausible initial SP range */
    const uint8_t *flash_read;    /* memory-mapped view of flash_base */

    int  (*erase_page)(uint32_t page);                       /* 0 = ok */
    int  (*program)(uint32_t addr, const uint8_t *buf, uint32_t len); /* 0 = ok; len % 8 == 0 */
    void (*send)(const char *line, uint32_t len);            /* one reply line */
    void (*reboot)(void);         /* reset; the chip re-checks page 0 as it boots */
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

/** CRC-32 (IEEE, as zlib / JS). Continue a running value by passing it back in. */
uint32_t su_crc32(uint32_t crc, const uint8_t *buf, uint32_t len);

#endif /* SU_CORE_H */
