/**
 * port_core_nvm_types.h — STM32 port: where core_nvm keeps its bytes
 *
 * Included near the top of core_nvm.h, before CORE_NVM_ERASED; not for
 * direct use. No types: the per-Core layout, which the linker scripts and
 * (on the H5) the option bytes match, so the addresses here must not move.
 *
 *   CORE_NVM_SIZE        the store's size in bytes
 *   CORE_NVM_EEPROM      1: a byte-writable data EEPROM at CORE_NVM_BASE
 *   CORE_NVM_FLASH_EMU   1: core_nvm.c's two-page log at CORE_NVM_FLASH_ADDR,
 *                        CORE_NVM_PAGE_SIZE bytes each (erase pages
 *                        CORE_NVM_FIRST_PAGE and the one after), written in
 *                        CORE_NVM_PROG_UNIT-byte units
 *   CORE_NVM_EDATA       1: the H5's flash high-cycle data area (EDATA),
 *                        whose sectors' code address is CORE_NVM_CODE_ADDR
 *
 * ll_common.h (REG32, the peripheral bases) comes in here, right after
 * stdint.h and string.h, where core_nvm.h always included it.
 */

#ifndef PORT_CORE_NVM_TYPES_H
#define PORT_CORE_NVM_TYPES_H

#include "ll_common.h"

#if defined(STM32L011xx)
  /* True EEPROM: 512 bytes, byte-writable, no erase needed */
  #define CORE_NVM_BASE        0x08080000UL
  #define CORE_NVM_SIZE        512
  #define CORE_NVM_EEPROM      1
  #define CORE_NVM_FLASH_EMU   0

#elif defined(STM32L422xx)
  /* Flash emulation in the last two 2 KB pages (62, 63) of the 128 KB bank.
   * The linker scripts end the application at 0x0801F000 to match. */
  #define CORE_NVM_SIZE        1024
  #define CORE_NVM_EEPROM      0
  #define CORE_NVM_FLASH_EMU   1
  #define CORE_NVM_FLASH_ADDR  0x0801F000UL
  #define CORE_NVM_PAGE_SIZE   2048u
  #define CORE_NVM_FIRST_PAGE  62u
  #define CORE_NVM_PROG_UNIT   8u      /* RM0394 §3.3.7: 64-bit double-words */

#elif defined(STM32WBA55xx)
  /* Flash emulation in pages 125-126 (8 KB each). Page 127, the last one, is
   * the BLE bond store (sdk/ble/nvm_stub.c), which was there first. The
   * linker scripts end the application at 0x080FA000 to leave all three. */
  #define CORE_NVM_SIZE        1024
  #define CORE_NVM_EEPROM      0
  #define CORE_NVM_FLASH_EMU   1
  #define CORE_NVM_FLASH_ADDR  0x080FA000UL
  #define CORE_NVM_PAGE_SIZE   8192u
  #define CORE_NVM_FIRST_PAGE  125u
  #define CORE_NVM_PROG_UNIT   16u     /* RM0493 §7.3.7: 128-bit quad-words */

#elif defined(STM32H523xx)
  /* Flash emulation in the high-cycle data area (EDATA, RM0481 §7.3.10):
   * bank 2 sectors 30 and 31, 6 KB each as data, at 0x09015000 (bank 2's
   * sector 24 is data address 0x0900C000, RM0481 Figure 27). The linker
   * script ends every image at 0x0807C000, the same sectors' code address.
   * The array is written 16 bits at a time; records still align to 8 B. */
  #define CORE_NVM_SIZE        1024
  #define CORE_NVM_EEPROM      0
  #define CORE_NVM_FLASH_EMU   1
  #define CORE_NVM_EDATA       1
  #define CORE_NVM_FLASH_ADDR  0x09015000UL
  #define CORE_NVM_PAGE_SIZE   6144u
  #define CORE_NVM_FIRST_PAGE  62u     /* flat 8 KB sector: bank 2, sector 30 */
  #define CORE_NVM_PROG_UNIT   8u      /* record alignment (the hardware writes 16 bits) */
  #define CORE_NVM_CODE_ADDR   0x0807C000UL   /* the two sectors' code address */
#endif

#ifndef CORE_NVM_EDATA
  #define CORE_NVM_EDATA       0
#endif

#endif /* PORT_CORE_NVM_TYPES_H */
