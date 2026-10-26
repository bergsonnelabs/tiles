/**
 * port_core_otp.h — STM32 port: the bodies of core_otp.h
 *
 * Included at the end of core_otp.h, after its prototypes; not for direct
 * use. Reads are memory-mapped loads; the program path goes through ll_flash
 * (unlock, one 128-bit quad-word program, lock) and verifies the read-back.
 * The geometry: the WBA55 (Core.ST.W5) OTP area; size 0 elsewhere.
 */

#ifndef PORT_CORE_OTP_H
#define PORT_CORE_OTP_H

#include <stdint.h>
#include <string.h>
#include "ll_common.h"
#include "ll_flash.h"
#include "core_caps.h"

#if CORE_HAS_OTP
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

static inline uint32_t core_otp_size(void)
{
    return CORE_OTP_SIZE;
}

static inline uint32_t core_otp_slot_count(void)
{
    return CORE_OTP_SLOT_COUNT;
}

static inline uint32_t core_otp_slot_addr(uint32_t slot)
{
    return CORE_OTP_BASE + slot * CORE_OTP_SLOT_SIZE;
}

static inline int core_otp_read(uint32_t offset, void *buf, uint32_t len)
{
    if (!CORE_OTP_AVAILABLE || offset + len > CORE_OTP_SIZE)
        return -1;
    memcpy(buf, (const void *)(CORE_OTP_BASE + offset), len);
    return 0;
}

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

#endif /* PORT_CORE_OTP_H */
