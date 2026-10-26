/**
 * port_core_nvm_h5_boot.h — STM32 port: the hardware behind core_nvm.c (3 of 3)
 *
 * Included by core_nvm.c only, at its end (see port_core_nvm_hw.h); not for
 * direct use. Core.ST.H5's first-boot switch of the flash high-cycle data
 * area: _core_nvm_h5_boot(), which coregen's core_init() calls, and its
 * record, _core_nvm_h5_prov() (declared in port_core_nvm.h, with the rules).
 * Nothing on the other Cores, nor in a host build.
 */

#ifndef PORT_CORE_NVM_H5_BOOT_H
#define PORT_CORE_NVM_H5_BOOT_H

#include "port_core_nvm_hw.h"

#if CORE_NVM_EDATA && defined(__arm__)
/* ============================================================
 * Core.ST.H5 — switching the data area on at first boot
 * (port_core_nvm.h has the rules; RM0481 §7.4.3 has the sequence)
 * ============================================================ */

#include "hal_dfu.h"        /* hal_dfu_started_by_rom, SCB_AIRCR */

#define PROV_MAGIC  0xEDA7A2B0UL

/* Kept across the reset below: startup neither copies nor zeroes .noinit
 * (sdk/device/stm32h523he.ld), and a system reset keeps SRAM on this board,
 * as hal_dfu.h's recovery words also rely on. */
static core_nvm_h5_prov_t s_prov __attribute__((section(".noinit")));

/* The option registers with a _CUR / _PRG pair (RM0481 §7.11.19-53), the
 * order of core_nvm_h5_prov_t.ob_before / ob_after. How the check before a
 * change treats each _PRG:
 *   PAIR_EQ     must equal its _CUR, or the change is refused;
 *   PAIR_EPOCH  an epoch counter: OPTSTRT changes it only to a value above
 *               _CUR (§7.4.7, Table 52), so a _PRG at or below _CUR (the
 *               factory state reads _PRG 0, _CUR 1) changes nothing; above
 *               it would, and is refused;
 *   PAIR_SEC    secure-only, RAZ/WI to this non-secure code (§7.11.27-28,
 *               .33-34, .46-47): nothing here can read or change it. */
enum { PAIR_EQ, PAIR_EPOCH, PAIR_SEC };
static const uint16_t k_ob_pairs[CORE_NVM_H5_OB_N][3] = {
    { 0x050, 0x054, PAIR_EQ },  { 0x060, 0x064, PAIR_EPOCH }, { 0x068, 0x06C, PAIR_EPOCH },
    { 0x070, 0x074, PAIR_EQ },  { 0x080, 0x084, PAIR_EQ },    { 0x088, 0x08C, PAIR_SEC },
    { 0x090, 0x094, PAIR_EQ },  { 0x0E0, 0x0E4, PAIR_SEC },   { 0x0E8, 0x0EC, PAIR_EQ },
    { 0x0F0, 0x0F4, PAIR_EQ },  { 0x0F8, 0x0FC, PAIR_EQ },    { 0x1E0, 0x1E4, PAIR_SEC },
    { 0x1E8, 0x1EC, PAIR_EQ },  { 0x1F0, 0x1F4, PAIR_EQ },    { 0x1F8, 0x1FC, PAIR_EQ },
};
/* OPTSR, NSEPOCHR, SECEPOCHR, OPTSR2, NSBOOTR, SECBOOTR, OTPBLR, SECWM1R,
 * WRP1R, EDATA1R, HDP1R, SECWM2R, WRP2R, EDATA2R, HDP2R */

static void port_core_nvm_h5_ob_snapshot(uint32_t *v)
{
    for (uint32_t i = 0; i < CORE_NVM_H5_OB_N; i++) v[i] = REG32(FLASH_BASE + k_ob_pairs[i][0]);
}

static int port_core_nvm_h5_edata2_on(uint32_t e)
{
    return (e & FLASH_EDATA_EN) && (e & FLASH_EDATA_STRT) >= 1u;
}

const core_nvm_h5_prov_t *_core_nvm_h5_prov(void)
{
    return s_prov.magic == PROV_MAGIC ? &s_prov : 0;
}

void _core_nvm_h5_boot(void)
{
    if (s_prov.magic != PROV_MAGIC) {           /* power-on: a new record */
        memset(&s_prov, 0, sizeof s_prov);
        s_prov.magic = PROV_MAGIC;
    }
    s_prov.boots++;
    s_prov.opsr   = FLASH_OPSR;
    s_prov.optsr  = FLASH_OPTSR_CUR;
    s_prov.edata1 = FLASH_EDATA1R_CUR;
    uint32_t cur  = FLASH_EDATA2R_CUR;
    if (port_core_nvm_h5_edata2_on(cur)) {                       /* the usual case: nothing to do */
        if (!s_prov.writes) s_prov.edata2_before = s_prov.edata2_after = cur;
        s_prov.state = CORE_NVM_H5_ENABLED;
        return;
    }
    if (s_prov.writes) {                        /* one change per power-on, whatever */
        s_prov.state = CORE_NVM_H5_RETRY;       /* happened to the last one: no loop */
        return;
    }
    if (hal_dfu_started_by_rom()) { s_prov.state = CORE_NVM_H5_ROM; return; }
    if (s_prov.optsr & FLASH_OPTSR_SWAP_BANK) { s_prov.state = CORE_NVM_H5_SWAP; return; }
    for (uint32_t i = 0; i < CORE_NVM_H5_OB_N; i++) {
        uint32_t c = REG32(FLASH_BASE + k_ob_pairs[i][0]), p = REG32(FLASH_BASE + k_ob_pairs[i][1]);
        int bad = (k_ob_pairs[i][2] == PAIR_EQ) ? (p != c)
                : (k_ob_pairs[i][2] == PAIR_EPOCH) ? (p > c) : 0;
        if (bad) {
            s_prov.mismatch = k_ob_pairs[i][1];
            s_prov.state = CORE_NVM_H5_PRG;
            return;
        }
    }

    /* RM0481 §7.4.3 "Option bytes modification sequence". */
    ll_iwdg_refresh();                          /* a watchdog from the last run survives resets */
    ll_flash_wait_bsy();                        /* 1, 2: BSY, WBNE, DBNE clear */
    FLASH_CCR = FLASH_SR_ERR_MASK | FLASH_SR_EOP | FLASH_SR_OPTCHANGEERR;    /* 3 */
    if (FLASH_OPTCR & FLASH_OPTCR_OPTLOCK) {    /* 4 (never twice: that locks it) */
        FLASH_OPTKEYR = FLASH_OPTKEY1;
        FLASH_OPTKEYR = FLASH_OPTKEY2;
    }
    s_prov.edata2_before = cur;
    port_core_nvm_h5_ob_snapshot(s_prov.ob_before);
    s_prov.writes++;
    FLASH_EDATA2R_PRG = (FLASH_EDATA2R_PRG & ~(FLASH_EDATA_EN | FLASH_EDATA_STRT))
                      | FLASH_EDATA_EN | 1u;    /* 5: the two last sectors */
    SET_BITS(FLASH_OPTCR, FLASH_OPTCR_OPTSTRT); /* 6 */
    uint32_t guard = 200000000UL;               /* 7, 8: ~2 s at 248 MHz; OPTSTRT */
    while (((FLASH_SR & FLASH_SR_BSY) || (FLASH_OPTCR & FLASH_OPTCR_OPTSTRT)) && --guard) { }
    s_prov.nssr = FLASH_SR;
    SET_BITS(FLASH_OPTCR, FLASH_OPTCR_OPTLOCK);
    s_prov.edata2_after = FLASH_EDATA2R_CUR;
    port_core_nvm_h5_ob_snapshot(s_prov.ob_after);
    if (!guard || (s_prov.nssr & (FLASH_SR_ERR_MASK | FLASH_SR_OPTCHANGEERR))
        || !port_core_nvm_h5_edata2_on(s_prov.edata2_after)) {
        s_prov.state = CORE_NVM_H5_FAILED;      /* no reset: this boot runs without NVM */
        return;
    }
    s_prov.state = CORE_NVM_H5_CHANGED;
    /* 9: reset, if this record survives it (FLASH_OPTSR2.SRAM13_RST = 1: SRAM1
     * kept on a system reset, §7.11.23), so the next boot can't loop back
     * here. Otherwise carry on: _CUR already has the new value (§7.4.3). */
    if (REG32(FLASH_BASE + 0x070UL) & (1UL << 2)) {
        __asm volatile ("dsb 0xF" ::: "memory");
        SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
        for (;;) { }
    }
}
#endif /* CORE_NVM_EDATA && __arm__ */

#endif /* PORT_CORE_NVM_H5_BOOT_H */
