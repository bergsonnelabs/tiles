/**
 * ble_update_boot.c — BLE update: the page-0 boot hook and the SRAM copier
 *
 * Every BLE build of Core.ST.W5 carries this. Reset_Handler (which the BLE
 * linker script puts in flash page 0, with this file's .ota_boot section)
 * calls Reset_EarlyHook() first thing, before .data / .bss exist. The hook
 * copies the copier (.ota_ram: this file's RAM code, sdk/serial_update/su_ota.c
 * and a 16-entry vector table) from page 0 into SRAM and runs it there. When
 * the install marker in page 124 says an install is pending, the copier copies
 * the staged image to 0x08000000 (page 0 last), marks it done and resets;
 * otherwise it returns and the app starts as usual.
 *
 * Why page 0 and SRAM: during an install, every page but 0 is being replaced,
 * and page 0 is replaced last. Code here may therefore not call anything
 * outside page 0 / .ota_ram: no libc, no ll_* helpers (they're inline, but
 * -Og keeps some out of line), no flash-resident tables, no struct copies.
 * tools/check_ota_boot.py (the Makefile runs it after every BLE link) disassembles the
 * two sections and fails on any branch or literal that leaves them.
 *
 * Registers: RM0493 (Rev 6) §7.9 FLASH (base 0x40022000: NSKEYR 0x08, NSSR
 * 0x20, NSCR1 0x28, ECCR 0x30), §34.7 IWDG (base 0x40003000, KR 0x00), and
 * the Armv8-M SCB (VTOR 0xE000ED08, AIRCR 0xE000ED0C). Spec:
 * docs/ble-update-protocol.md §8.
 */

/* Loops must stay loops: a memcpy / memset call would land in flash pages
 * that are being rewritten. */
#pragma GCC optimize ("no-tree-loop-distribute-patterns")

#include <stdint.h>

#define SU_OTA_RAM       __attribute__((section(".ota_ram")))
#define SU_OTA_RAM_DATA  __attribute__((section(".ota_ram.rodata")))
#include "../serial_update/su_ota.c"

#define OTA_BOOT         __attribute__((section(".ota_boot"), used))

#define REG(a)           (*(volatile uint32_t *)(a))

/* ---- FLASH, RM0493 §7.9 ---- */
#define F_BASE           0x40022000UL
#define F_NSKEYR         REG(F_BASE + 0x08UL)
#define F_NSSR           REG(F_BASE + 0x20UL)
#define F_NSCR1          REG(F_BASE + 0x28UL)
#define F_ECCR           REG(F_BASE + 0x30UL)
#define F_KEY1           0x45670123UL
#define F_KEY2           0xCDEF89ABUL
#define SR_EOP           (1UL << 0)
#define SR_ERRS          ((1UL << 1) | (1UL << 3) | (1UL << 4) | (1UL << 5) | \
                          (1UL << 6) | (1UL << 7) | (1UL << 13))   /* OPERR PROGERR WRPERR PGAERR SIZERR PGSERR OPTWERR */
#define SR_BSY           (1UL << 16)
#define SR_WDW           (1UL << 17)
#define CR_PG            (1UL << 0)
#define CR_PER           (1UL << 1)
#define CR_PNB_SHIFT     3u                                     /* PNB[6:0] = bits 9:3 */
#define CR_PNB_MASK      (0x7FUL << CR_PNB_SHIFT)
#define CR_STRT          (1UL << 16)
#define CR_LOCK          (1UL << 31)
#define ECCR_ECCIE       (1UL << 24)
#define ECCR_ECCC        (1UL << 30)
#define ECCR_ECCD        (1UL << 31)

/* ---- IWDG, RM0493 §34.7.1 ---- */
#define IWDG_KR          REG(0x40003000UL)

/* ---- Cortex-M33 SCB ---- */
#define SCB_VTOR         REG(0xE000ED08UL)
#define SCB_AIRCR        REG(0xE000ED0CUL)

/* ============================================================
 * SRAM-resident: exception table, flash access, the entry point
 * ============================================================ */

/* In .bss, which doesn't exist yet when the copier runs: fine, it is only
 * ever written (b_read, the NMI) before it is read. Keeping writable data out
 * of .ota_ram keeps that section read-only + executable. */
static volatile uint32_t ecc_hit;

/* A double ECC error raises an NMI; the load completes with bad data (§7.3.2).
 * During the copier that's a torn marker or staging quad-word: note it, clear
 * ECCD (rc_w1), return. */
static SU_OTA_RAM void ota_nmi(void)
{
    uint32_t e = F_ECCR;
    F_ECCR = (e & (ECCR_ECCIE | ECCR_ECCC)) | ECCR_ECCD;
    ecc_hit = 1;
}

/* Anything else (HardFault, ...) during the copier: stop. A running IWDG then
 * resets the chip and the copier starts over; without one it waits for a
 * power cycle. Either way nothing half-copied runs. */
static SU_OTA_RAM void ota_fault(void)
{
    for (;;)
        ;
}

extern uint32_t __ota_ram_start[];

/* Only the system exceptions: the copier runs with interrupts masked, and
 * VTOR goes back to the app's table before the app starts. Entry 0 (SP) is
 * never read through VTOR. */
static void (*const ota_vtab[16])(void) __attribute__((section(".ota_ram.vtab"), used, aligned(128))) = {
    0,          ota_fault, ota_nmi,   ota_fault,   /* SP, Reset, NMI, HardFault */
    ota_fault,  ota_fault, ota_fault, ota_fault,   /* MemManage, BusFault, UsageFault, SecureFault */
    ota_fault,  ota_fault, ota_fault, ota_fault,   /* reserved, SVCall */
    ota_fault,  ota_fault, ota_fault, ota_fault,   /* DebugMon, reserved, PendSV, SysTick */
};

static SU_OTA_RAM void f_wait(void)
{
    while (F_NSSR & (SR_BSY | SR_WDW))
        ;
}

static SU_OTA_RAM int f_unlock(void)
{
    if (F_NSCR1 & CR_LOCK) {
        F_NSKEYR = F_KEY1;
        F_NSKEYR = F_KEY2;
    }
    return (F_NSCR1 & CR_LOCK) ? -1 : 0;
}

static SU_OTA_RAM int b_read(void *ctx, uint32_t off, void *dst, uint32_t len)
{
    (void)ctx;
    const volatile uint8_t *s = (const volatile uint8_t *)(SU_OTA_FLASH_BASE + off);
    uint8_t *d = (uint8_t *)dst;
    ecc_hit = 0;
    uint32_t i = 0;
    if (((off | (uint32_t)(uintptr_t)dst) & 3u) == 0u)       /* words where aligned */
        for (; i + 4u <= len; i += 4u)
            *(uint32_t *)(void *)(d + i) = *(const volatile uint32_t *)(const volatile void *)(s + i);
    for (; i < len; i++) d[i] = s[i];
    __asm volatile ("dsb 0xF\n\tisb 0xF" ::: "memory");   /* let a pending NMI land */
    return ecc_hit ? -1 : 0;
}

/* RM0493 §7.3.6 page erase: BSY clear, errors cleared, PER + PNB, STRT, BSY. */
static SU_OTA_RAM int b_erase(void *ctx, uint32_t page)
{
    (void)ctx;
    if (page >= 128u || f_unlock() != 0) return -1;
    f_wait();
    F_NSSR = SR_ERRS | SR_EOP;
    F_NSCR1 = (F_NSCR1 & ~(CR_PG | CR_PNB_MASK)) | CR_PER | (page << CR_PNB_SHIFT);
    F_NSCR1 |= CR_STRT;
    f_wait();
    F_NSCR1 &= ~(CR_PER | CR_PNB_MASK);
    return (F_NSSR & SR_ERRS) ? -1 : 0;
}

/* RM0493 §7.3.7 quad-word program: BSY and WDW clear, errors cleared, PG,
 * four words, then WDW and BSY clear. */
static SU_OTA_RAM int b_program(void *ctx, uint32_t off, const uint32_t *src, uint32_t len)
{
    (void)ctx;
    if ((off & 15u) || (len & 15u) || f_unlock() != 0) return -1;
    for (uint32_t i = 0; i < len; i += 16u) {
        volatile uint32_t *d = (volatile uint32_t *)(SU_OTA_FLASH_BASE + off + i);
        f_wait();
        F_NSSR = SR_ERRS | SR_EOP;
        F_NSCR1 |= CR_PG;
        d[0] = src[i / 4u + 0];
        d[1] = src[i / 4u + 1];
        d[2] = src[i / 4u + 2];
        d[3] = src[i / 4u + 3];
        f_wait();
        F_NSCR1 &= ~CR_PG;
        if (F_NSSR & SR_ERRS) return -1;
    }
    return 0;
}

static SU_OTA_RAM void b_feed(void *ctx)
{
    (void)ctx;
    IWDG_KR = 0xAAAAUL;          /* reload; does nothing if the IWDG isn't running */
}

/* The copier's entry, in SRAM. Returns to the hook to boot on, or resets. */
SU_OTA_RAM void ble_update_boot_ram(void)
{
    su_ota_io_t io;
    io.read = b_read;
    io.erase = b_erase;
    io.program = b_program;
    io.feed = b_feed;
    io.ctx = 0;

    uint32_t vtor = SCB_VTOR;
    __asm volatile ("cpsid i" ::: "memory");
    SCB_VTOR = (uint32_t)(uintptr_t)__ota_ram_start;       /* = ota_vtab */
    __asm volatile ("dsb 0xF\n\tisb 0xF" ::: "memory");

    int r = su_ota_boot(&io);

    F_NSCR1 |= CR_LOCK;
    if (r == SU_OTA_BOOT_INSTALLED) {
        __asm volatile ("dsb 0xF" ::: "memory");
        SCB_AIRCR = 0x05FA0004UL;                            /* VECTKEY | SYSRESETREQ */
        for (;;)
            ;
    }
    if (r == SU_OTA_BOOT_FAILED)
        ota_fault();             /* flash broke mid-copy: nothing bootable */

    SCB_VTOR = vtor;
    __asm volatile ("dsb 0xF\n\tisb 0xF" ::: "memory");
    __asm volatile ("cpsie i" ::: "memory");
}

/* ============================================================
 * Flash page 0: the hook
 * ============================================================ */

extern uint32_t __ota_ram_end[];
extern uint32_t __ota_ram_load[];

/* Strong override of the startup's weak no-op. Called with only a stack. */
OTA_BOOT void Reset_EarlyHook(void)
{
    volatile uint32_t *d = __ota_ram_start;
    const volatile uint32_t *s = __ota_ram_load;
    while (d < (volatile uint32_t *)__ota_ram_end)
        *d++ = *s++;
    __asm volatile ("dsb 0xF\n\tisb 0xF" ::: "memory");
    /* Through a register: a direct BL from flash to SRAM is out of range and
     * the linker's veneer could land outside page 0. */
    void (*volatile entry)(void) = ble_update_boot_ram;
    entry();
}
