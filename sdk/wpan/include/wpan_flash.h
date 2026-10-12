/**
 * wpan_flash.h — Flash erase / program on Core.ST.W5 while the radio runs
 *
 * The WBA55 has one flash bank: an erase (~1.5 ms) or a quad-word program
 * stalls every fetch from flash, the radio interrupt handlers included. ST's
 * flash manager (flash_manager.c, rf_timing_synchro.c, flash_driver.c from
 * STM32Cube_FW_WBA Projects/Common/WPAN/Modules/Flash) asks the link layer for
 * a time window between radio events and runs the operation in it, so
 * advertising and connections carry on. This module wires it into the SDK and
 * gives core_nvm, the BLE bond store and the Thread settings a blocking call
 * on top.
 *
 * BLE: ble_app_init() calls wpan_flash_init() before the stack starts and
 * wpan_flash_ll_started() after; until then wpan_flash_ready() is 0 and callers
 * write flash directly (no radio is running).
 *
 * Thread: thread_app_init() calls wpan_flash_init() only. ST's Thread link
 * layer has no external-event scheduler (DISABLE_RFTS_EXT_EVNT_HNDLR=1, as in
 * ST's Thread projects), so no time window would ever be granted: Thread
 * builds keep the RFTS bypass and an erase stalls the CPU (~1.5 ms) like any
 * flash erase. The flash manager still serializes the operations.
 *
 * BLE + Thread: as BLE. ble_app_init() calls both; thread_app_init() then
 * calls neither, and the concurrent link layer grants the windows between
 * BLE and 802.15.4 events alike (ST's BLE_HeartRate_Thread does the same:
 * RFTS bypass off after APP_BLE_Init, before APP_THREAD_Init), so the Thread
 * settings are written in radio-free windows too.
 */

#ifndef WPAN_FLASH_H
#define WPAN_FLASH_H

#include <stdint.h>

/** Register the flash manager task and allow direct access (radio not up). */
void wpan_flash_init(void);

/** The link layer runs: from now on flash access waits for time windows. */
void wpan_flash_ll_started(void);

/** 1 once the flash manager is in charge of flash access. */
int wpan_flash_ready(void);

/**
 * Erase one 8 KB page in a radio-safe window. Blocks until done, running the
 * sequencer (radio tasks included) meanwhile; call from thread context only.
 * @return 0 on success, -1 on a flash error or a refused request.
 */
int wpan_flash_erase_page(uint32_t page);

/**
 * Program n_words 32-bit words (a whole number of 16 B quad-words) from src
 * (word-aligned) to dest (16 B aligned). Blocks like wpan_flash_erase_page().
 * @return 0 on success, -1 on a flash error or a refused request.
 */
int wpan_flash_program(uint32_t dest, const uint32_t *src, uint32_t n_words);

#endif /* WPAN_FLASH_H */
