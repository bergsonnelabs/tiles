/**
 * core_otp.h — One-time-programmable (OTP) identity storage
 *
 * Immutable, write-once bytes that survive a full chip mass-erase. The OTP
 * region lives in the system-flash information block, OUTSIDE the main flash
 * array that page/mass erase can reach — so a downstream ST-Link / J-Link
 * "chip erase" (or any normal firmware download) cannot clear it. Intended
 * for permanent per-unit hardware identity — board variant, capability
 * flags, serial — burned once at production and read at every boot.
 *
 *   - Core.ST.W5 (STM32WBA55): 512 B OTP @ 0x0BF90000, programmed in 128-bit
 *     (16 B) quad-words, write-once per quad-word.
 *   - Core.ST.L0 / L4 / H5: no OTP surface exposed here yet
 *     (CORE_OTP_AVAILABLE == 0; the API still compiles — reads/programs
 *     return an error).
 *
 * Reads are a plain memory-mapped load — no unlock. Programming a slot is
 * IRREVERSIBLE: OTP bits only transition 1 -> 0 and a programmed quad-word
 * can never be re-written. core_otp_program_slot() refuses a non-blank slot
 * and verifies the read-back, but there is no undo. To amend a mis-burn, use
 * an append / last-valid-record scheme in the application layer (32 slots on
 * WBA55) rather than trying to rewrite a slot.
 *
 * The record format (magic, CRC, field layout) is a PRODUCT concern and lives
 * in the application, not here — this module is the generic OTP mechanism only.
 *
 * @studio category otp label=Core.OTP icon=▦
 *
 * @studio coverage
 *   id:    otp
 *   name:  OTP — one-time-programmable identity
 *   blurb: Read / program the WBA55 512 B OTP region (16 B quad-word slots,
 *          write-once, survives chip-erase). Reads are memory-mapped; the
 *          program path unlocks flash, writes one 128-bit quad-word, re-locks,
 *          and verifies the read-back. The record/CRC format is the
 *          application's to define.
 */

#ifndef CORE_OTP_H
#define CORE_OTP_H

#include <stdint.h>
#include <string.h>
#include "ll_common.h"
#include "ll_flash.h"

/* ============================================================
 * Platform geometry
 * ============================================================ */

#if defined(STM32WBA55xx)
  /* FLASH_OTP_BASE / FLASH_OTP_SIZE per the CubeWBA CMSIS header. */
  #define CORE_OTP_BASE       0x0BF90000UL
  #define CORE_OTP_SIZE       512u
  #define CORE_OTP_SLOT_SIZE  16u            /* 128-bit quad-word program unit */
  #define CORE_OTP_AVAILABLE  1
#else
  /* No OTP surface exposed on this Core yet. */
  #define CORE_OTP_BASE       0UL
  #define CORE_OTP_SIZE       0u
  #define CORE_OTP_SLOT_SIZE  16u
  #define CORE_OTP_AVAILABLE  0
#endif

#define CORE_OTP_SLOT_COUNT   (CORE_OTP_SIZE / CORE_OTP_SLOT_SIZE)

/* ============================================================
 * API
 * ============================================================ */

/**
 * Total OTP size in bytes for this Core (0 where unavailable).
 *
 * @studio expose category=otp name=size returns=int
 */
static inline uint32_t core_otp_size(void)
{
    return CORE_OTP_SIZE;
}

/**
 * Number of write-once quad-word slots (CORE_OTP_SIZE / 16).
 *
 * @studio expose category=otp name=slot_count returns=int
 */
static inline uint32_t core_otp_slot_count(void)
{
    return CORE_OTP_SLOT_COUNT;
}

/** Absolute address of a slot (no bounds check). */
static inline uint32_t core_otp_slot_addr(uint32_t slot)
{
    return CORE_OTP_BASE + slot * CORE_OTP_SLOT_SIZE;
}

/**
 * Read bytes from OTP into `buf`. Memory-mapped load; no unlock needed.
 * Returns 0 on success, -1 if the range is out of bounds or OTP is
 * unavailable on this Core.
 */
static inline int core_otp_read(uint32_t offset, void *buf, uint32_t len)
{
    if (!CORE_OTP_AVAILABLE || offset + len > CORE_OTP_SIZE)
        return -1;
    memcpy(buf, (const void *)(CORE_OTP_BASE + offset), len);
    return 0;
}

/**
 * Return 1 if `slot` is entirely erased (all 0xFF), 0 if any bit is
 * programmed, or -1 if the slot index is out of range / OTP unavailable.
 * A blank slot is programmable; a non-blank slot is not (write-once).
 *
 * @studio expose category=otp name=slot_is_blank returns=int
 * @param slot Slot index, 0 to core_otp_slot_count() - 1.
 */
static inline int core_otp_slot_is_blank(uint32_t slot)
{
    if (!CORE_OTP_AVAILABLE || slot >= CORE_OTP_SLOT_COUNT)
        return -1;
    const volatile uint8_t *p = (const volatile uint8_t *)core_otp_slot_addr(slot);
    for (uint32_t i = 0; i < CORE_OTP_SLOT_SIZE; i++)
        if (p[i] != 0xFFu)
            return 0;
    return 1;
}

/**
 * Program one 16-byte quad-word slot (write-once) and verify the read-back.
 *
 * IRREVERSIBLE. Refuses a slot that is not blank (returns -2) so a second
 * write can't corrupt an existing record. On WBA55 this unlocks flash, issues
 * a single 128-bit quad-word program, re-locks, and confirms the stored bytes
 * match `rec`. There is no retry and no undo — callers should treat a failure
 * as "advance to the next slot", not "try again here".
 *
 *   slot: 0 .. CORE_OTP_SLOT_COUNT-1
 *   rec:  pointer to exactly CORE_OTP_SLOT_SIZE (16) bytes.
 *
 * Returns 0 on success, -1 on range/program/verify error, -2 if the slot is
 * not blank, -3 if OTP programming is unavailable on this Core.
 *
 * Deliberately NOT `@studio expose`d: a destructive, irreversible burn does
 * not belong in the DSL palette. It is escape-to-C only (see @studio
 * unsupported below).
 */
static inline int core_otp_program_slot(uint32_t slot, const void *rec)
{
#if CORE_OTP_AVAILABLE
    if (slot >= CORE_OTP_SLOT_COUNT)
        return -1;

    int blank = core_otp_slot_is_blank(slot);
    if (blank < 0)  return -1;
    if (blank == 0) return -2;      /* write-once: never overwrite a record */

    /* Copy through a local to honor the quad-word write's alignment/ordering
     * regardless of the caller's buffer alignment. */
    uint32_t w[4];
    memcpy(w, rec, sizeof(w));

    uint32_t addr = core_otp_slot_addr(slot);
    ll_flash_unlock();
    int rc = ll_flash_program_qword(addr, w);
    ll_flash_lock();
    if (rc != 0)
        return -1;

    /* OTP has no retry — confirm the read-back before trusting the record. */
    if (memcmp((const void *)addr, rec, CORE_OTP_SLOT_SIZE) != 0)
        return -1;

    return 0;
#else
    (void)slot; (void)rec;
    return -3;
#endif
}

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=1 value=H title="OTP only on Core.ST.W5"
//   CORE_OTP_* is backed only on Core.ST.W5 (STM32WBA55, 512 B @ 0x0BF90000).
//   L0 / L4 / H5 report size 0 and every read/program returns an error until
//   their OTP geometry is added. WBA55 is the only Core that currently ships a
//   product using OTP identity.
//
// @studio unsupported tier=2 value=M title="program_slot is escape-to-C (destructive)"
//   Reads (size, slot_count, slot_is_blank, read) are safe to surface, but the
//   burn (core_otp_program_slot) is irreversible and write-once, so it is
//   intentionally NOT exposed to the DSL — a stray palette call could
//   permanently consume a slot. Provisioning burns happen from C / the
//   production programmer, not from a Studio program.

#endif /* CORE_OTP_H */
