/**
 * ble_flash.h — Flash erase / program on Core.ST.W5 while BLE runs
 *
 * The WBA55 has one flash bank: an erase (~1.5 ms) or a quad-word program
 * stalls every fetch from flash, the radio interrupt handlers included. ST's
 * flash manager (flash_manager.c, rf_timing_synchro.c, flash_driver.c from
 * STM32Cube_FW_WBA Projects/Common/WPAN/Modules/Flash) asks the link layer for
 * a time window between radio events and runs the operation in it, so
 * advertising and connections carry on. This module wires it into the SDK and
 * gives core_nvm (and the bond store) a blocking call on top.
 *
 * ble_app_init() calls ble_flash_init() before the stack starts and
 * ble_flash_ll_started() after; until then ble_flash_ready() is 0 and callers
 * write flash directly (no radio is running).
 */

#ifndef BLE_FLASH_H
#define BLE_FLASH_H

#include <stdint.h>

/** Register the flash manager task and allow direct access (radio not up). */
void ble_flash_init(void);

/** The link layer runs: from now on flash access waits for time windows. */
void ble_flash_ll_started(void);

/** 1 once the flash manager is in charge of flash access. */
int ble_flash_ready(void);

/**
 * Erase one 8 KB page in a radio-safe window. Blocks until done, running the
 * sequencer (BLE tasks included) meanwhile; call from thread context only.
 * @return 0 on success, -1 on a flash error or a refused request.
 */
int ble_flash_erase_page(uint32_t page);

/**
 * Program n_words 32-bit words (a whole number of 16 B quad-words) from src
 * (word-aligned) to dest (16 B aligned). Blocks like ble_flash_erase_page().
 * @return 0 on success, -1 on a flash error or a refused request.
 */
int ble_flash_program(uint32_t dest, const uint32_t *src, uint32_t n_words);

#endif /* BLE_FLASH_H */
