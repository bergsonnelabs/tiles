/**
 * su_protocol.h — Serial update: wire protocol and app→flasher handoff
 *
 * Serial update reprograms a running Core over its USB-CDC port, without the
 * ST ROM bootloader. The host opens the port at SU_TRIGGER_BAUD; the running
 * app (hal_usb_cdc.c) sees that line coding, copies the flasher blob into
 * SRAM and jumps to it. The flasher keeps the same USB connection (same
 * endpoints, same data toggles, no re-enumeration), receives the image and
 * rewrites flash. Spec: tiles docs/serial-update-protocol.md.
 *
 * Shared by the flasher blobs (sdk/serial_update/flasher_l4.c, flasher_h5.c)
 * and the app hook (sdk/hal/hal_usb_cdc.c). Keep it free of anything either
 * side can't compile. The values a flasher build may override (-D) are the
 * ones under #ifndef; the L4 build uses the defaults.
 */

#ifndef SU_PROTOCOL_H
#define SU_PROTOCOL_H

#include <stdint.h>

/* Opening the CDC port at this baud rate hands the Core to the flasher.
 * A standard rate on every host OS (no custom-baud paths), and one nobody
 * uses to talk to a USB CDC device, whose baud rate is otherwise meaningless. */
#define SU_TRIGGER_BAUD         2400u

/* 1: READY has 7 tokens. 2 (Core.ST.H5): READY appends the largest image the
 * flasher accepts, in bytes, so the host needs no per-chip table. */
#ifndef SU_PROTOCOL_VERSION
#define SU_PROTOCOL_VERSION     1u
#endif

/* ---- Host → Core frames ----
 *
 * 16-byte little-endian header, then `len` payload bytes:
 *   [0] 'S'  [1] 'U'  [2] type  [3] flags (0)
 *   [4..5] len   [6..7] reserved (0)
 *   [8..11] arg0 [12..15] arg1
 */
#define SU_SYNC0                'S'
#define SU_SYNC1                'U'
#define SU_HDR_LEN              16u
#ifndef SU_MAX_PAYLOAD
#define SU_MAX_PAYLOAD          2048u   /* one flash page: 2 KB L4, 8 KB H5 */
#endif

/* Flash programming unit, bytes: the L4 programs double-words, the H5
 * 128-bit flash words with ECC (RM0481 §7.3.5), which must each be written
 * exactly once. */
#ifndef SU_PROG_UNIT
#define SU_PROG_UNIT            8u
#endif

#define SU_T_QUERY              'Q'   /* → "SU READY ..." */
#define SU_T_BEGIN              'B'   /* arg0 = image size, arg1 = CRC-32; payload = u32 DEV_ID */
#define SU_T_DATA               'D'   /* arg0 = offset, arg1 = CRC-32 of payload */
#define SU_T_END                'E'   /* verify, write page 0, reset into the new image */
#define SU_T_ABORT              'X'   /* reset now (old app if page 0 intact) */

/* ---- Core → host replies ----
 *
 * One ASCII line per frame, each shorter than one 64-byte USB packet:
 *   SU READY <ver> <dev_id hex> <flash KB> <page bytes> <page0 erased 0|1> [<image limit bytes>, v2]
 *   SU OK <type> [next offset]
 *   SU ERR <code> <word>
 *   SU DONE
 */
#define SU_ERR_LENGTH           1u    /* DATA chunk empty, too long, or short mid-image */
#define SU_ERR_DEVICE           2u    /* DEV_ID in BEGIN is not this chip */
#define SU_ERR_SIZE             3u    /* image empty, or reaches the core_nvm pages */
#define SU_ERR_VECTORS          4u    /* image's SP / reset vector implausible */
#define SU_ERR_FLASH            5u    /* erase / program / verify failed */
#define SU_ERR_ORDER            6u    /* not the next expected offset */
#define SU_ERR_CRC              7u    /* chunk CRC mismatch (resend it) */
#define SU_ERR_IMAGE_CRC        8u    /* whole-image CRC mismatch at END */
#define SU_ERR_STATE            9u    /* frame not valid in this state */

/* The top of flash kept for core_nvm (its last two 2 KB pages on the L4,
 * sdk/core/core_nvm.h; two 8 KB sectors on the H5, where core_nvm is still to
 * come). The flasher refuses an image that would reach it (SU_ERR_SIZE) and so
 * never erases it. */
#define SU_NVM_RESERVED         4096u
#define SU_NVM_RESERVED_H5      16384u

/* Without a BEGIN for this long, and with flash untouched, the flasher resets
 * back into the old app. Once page 0 is erased it never gives up on its own. */
#define SU_IDLE_TIMEOUT_MS      10000u

/* ---- App → flasher handoff (STM32L422, STM32H523) ----
 *
 * SRAM map while the flasher runs (the same on both; the H5 has 272 KB but
 * the flasher stays in the L4's first 40 KB):
 *   0x20000000  flasher image (vector table first) + .bss   (< 0x7000)
 *   0x20007000  su_handoff_t, written by the app just before the jump
 *   0x20007100  flasher stack, growing down from SU_STACK_TOP
 *   top - 16    DFU magic + brick-recovery words — never touched
 *               (0x20009FF0 L4, 0x20043FF0 H5)
 */
#define SU_FLASHER_BASE         0x20000000UL
#define SU_HANDOFF_ADDR         0x20007000UL
#define SU_STACK_TOP            0x20009FE0UL

#define SU_HANDOFF_MAGIC        0x32485553UL   /* "SUH2": serial[] added */

/* Everything the flasher needs to keep answering the host as the same USB
 * device. Pointers reference the app's const data in flash, which the flasher
 * copies into its own RAM before it erases anything, or (the serial number,
 * built from the chip UID at run time) a string inside this block. */
typedef struct {
    uint32_t       magic;
    const uint8_t *dev_desc;
    const uint8_t *cfg_desc;
    const uint8_t *hid_report_desc;
    const char    *strings[3];        /* manufacturer, product, serial */
    uint16_t       dev_desc_len;
    uint16_t       cfg_desc_len;
    uint16_t       cfg_hid_offset;    /* HID class descriptor within cfg_desc */
    uint16_t       hid_report_desc_len;
    uint8_t        line_coding[7];    /* current CDC line coding (the trigger) */
    uint8_t        configured;        /* SET_CONFIGURATION already done */
    char           serial[26];        /* NUL-terminated serial; strings[2] may point here */
} su_handoff_t;

#endif /* SU_PROTOCOL_H */
