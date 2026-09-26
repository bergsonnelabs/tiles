/**
 * ble_flash.c — ST's flash manager wired into the SDK (see ble_flash.h)
 *
 * The same setup ST's CubeWBA applications do in app_entry.c
 * (APPE_FLASH_MANAGER_Init, FM_ProcessRequest, the RFTS bypass switch), plus
 * a blocking erase / program for core_nvm: it waits on a sequencer event with
 * UTIL_SEQ_WaitEvt, which keeps running every other sequencer task (the link
 * layer, the flash manager itself) until the operation's callback fires.
 */

#include <stdint.h>
#include "ble_flash.h"
#include "flash_manager.h"
#include "flash_driver.h"
#include "stm32_seq.h"
#include "app_conf.h"

static volatile uint8_t s_ready;
static volatile uint8_t s_done, s_avail;

static void on_op(FM_FlashOp_Status_t status)
{
    if (status == FM_OPERATION_COMPLETE) s_done = 1;
    else                                 s_avail = 1;
    UTIL_SEQ_SetEvt(1U << CFG_IDLEEVT_FLASH_OP);
}

static FM_CallbackNode_t s_node = { .Callback = on_op };

static void wait_for(volatile uint8_t *flag)
{
    while (!*flag)
        UTIL_SEQ_WaitEvt(1U << CFG_IDLEEVT_FLASH_OP);
}

/* The flash manager calls this whenever it has work: run it as a task. */
void FM_ProcessRequest(void)
{
    UTIL_SEQ_SetTask(1U << CFG_TASK_FLASH_MANAGER, CFG_SEQ_PRIO_0);
}

void ble_flash_init(void)
{
    UTIL_SEQ_RegTask(1U << CFG_TASK_FLASH_MANAGER, UTIL_SEQ_RFU, FM_BackgroundProcess);

    /* No window yet; direct access allowed while the link layer isn't up. */
    FD_SetStatus(FD_FLASHACCESS_RFTS, LL_FLASH_DISABLE);
    FD_SetStatus(FD_FLASHACCESS_RFTS_BYPASS, LL_FLASH_ENABLE);
    FD_SetStatus(FD_FLASHACCESS_SYSTEM, LL_FLASH_ENABLE);
    s_ready = 1;
}

void ble_flash_ll_started(void)
{
    /* From here on every erase / program waits for a radio time window. */
    FD_SetStatus(FD_FLASHACCESS_RFTS_BYPASS, LL_FLASH_DISABLE);
}

int ble_flash_ready(void)
{
    return s_ready;
}

/* start() returns the FM_* command status. FM_BUSY queues our callback node:
 * wait for FM_OPERATION_AVAILABLE and ask again. */
static int run(FM_Cmd_Status_t (*start)(uint32_t, uint32_t, uint32_t), uint32_t a, uint32_t b, uint32_t c)
{
    for (;;) {
        s_done = 0;
        s_avail = 0;
        FM_Cmd_Status_t st = start(a, b, c);
        if (st == FM_ERROR) return -1;
        if (st == FM_OK) {
            /* The operation runs in the flash manager task, not yet: clear a
             * hardware error some earlier operation left behind. */
            (void)FD_TakeHwError();
            wait_for(&s_done);
            return FD_TakeHwError() ? -1 : 0;
        }
        wait_for(&s_avail);
    }
}

static FM_Cmd_Status_t start_erase(uint32_t page, uint32_t unused, uint32_t unused2)
{
    (void)unused; (void)unused2;
    return FM_Erase(page, 1u, &s_node);
}

static FM_Cmd_Status_t start_write(uint32_t dest, uint32_t src, uint32_t n_words)
{
    return FM_Write((uint32_t *)src, (uint32_t *)dest, (int32_t)n_words, &s_node);
}

int ble_flash_erase_page(uint32_t page)
{
    return run(start_erase, page, 0u, 0u);
}

int ble_flash_program(uint32_t dest, const uint32_t *src, uint32_t n_words)
{
    if ((dest & 0xFu) || (n_words & 3u) || n_words == 0u) return -1;
    return run(start_write, dest, (uint32_t)(uintptr_t)src, n_words);
}
