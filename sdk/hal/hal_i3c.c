/**
 * hal_i3c.c — I3C controller HAL (STM32H523 / Core.ST.H5)
 *
 * Every transfer is one "frame": a list of messages (control words) pushed
 * through the C-FIFO, with their data fed to the TX-FIFO and drained from the
 * RX-FIFO by polling I3C_EVR, until the frame completes (FCF) or fails
 * (ERRF). RM0481 §49.7.2 (controller sequence), §49.10 (FIFOs, controller),
 * §49.12.1 (controller errors). The controller stalls SCL for up to ~100 µs
 * (TIMINGR1.AVAL = 1 µs, §49.16.21) while it waits for software, so short
 * interrupts during a polled frame do not break it; a longer gap ends the
 * frame with a DOVR/COVR error, which is reported, never hung on.
 *
 * Settings (RM0481 §49.16.3): C-FIFO and TX-FIFO not preloaded (TMODE 0),
 * S-FIFO off (SMODE 0, I3C_SR read after the frame), byte thresholds, no DMA,
 * arbitrable 0x7E header on (NOARBH 0) unless the config opts out: with IBIs
 * enabled the header avoids an address-arbitration collision (§49.9.13 note).
 */

#include "hal_i3c.h"

#if LL_I3C_AVAILABLE

#include <string.h>

#define _OWN_DA        0x08U   /* controller's own dynamic address (I3C_DEVR0) */
#define _DA_POOL_FIRST 0x09U

/* ---- IRQ dispatch: one registered handle per instance ----
 * The handlers call through a pointer that hal_i3c_ibi_enable() sets, so an
 * image that never enables IBIs links none of the IBI code. */
static hal_i3c_t *_i3c_h[2];
static void (*_i3c_isr)(hal_i3c_t *h);

static int _inst_idx(I3C_TypeDef *i)
{
    if (i == I3C1) return 0;
    if (i == I3C2) return 1;
    return -1;
}

void I3C1_EV_IRQHandler(void) { if (_i3c_h[0] && _i3c_isr) _i3c_isr(_i3c_h[0]); }
void I3C1_ER_IRQHandler(void) { if (_i3c_h[0] && _i3c_isr) _i3c_isr(_i3c_h[0]); }
void I3C2_EV_IRQHandler(void) { if (_i3c_h[1] && _i3c_isr) _i3c_isr(_i3c_h[1]); }
void I3C2_ER_IRQHandler(void) { if (_i3c_h[1] && _i3c_isr) _i3c_isr(_i3c_h[1]); }

/* ============================================================
 * Timing (RM0481 §49.6.2, §49.16.20-21)
 * ============================================================ */

static uint32_t _cycles(uint32_t fk, uint32_t ns)       /* ceil(ns * fk / 1e9) */
{
    return (uint32_t)(((uint64_t)ns * fk + 999999999ULL) / 1000000000ULL);
}

static uint32_t _clamp(uint32_t v, uint32_t lo, uint32_t hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Push-pull: SCL period = (SCLL_PP + 1) + (SCLH_I3C + 1) kernel cycles,
 * split 50/50 with the high phase at least 40 ns (tDIG_H min 32 ns, pure I3C
 * bus: coregen only puts a bus on I3C when every tile on it speaks I3C) and
 * the low phase never shorter: DS14540 Table 108 measured tSU_PP at 12 ns
 * against the 3 ns MIPI minimum, and its mitigation is a longer SCL low.
 * Open drain: SCLL_OD covers the address, ACK and T-bit phases (tDIG_OD_L >=
 * 200 ns). It is an 8-bit count, so at a 248 MHz kernel it tops out at
 * 1.03 µs; the requested time is clamped and the achieved one reported in
 * h->od_low_ns. The kernel must run at >= 2x SCL (§49.6.2). */
static hal_status_t _timing(hal_i3c_t *h, uint32_t scl_hz, uint32_t od_ns)
{
    uint32_t fk = h->kernel_hz;
    if (scl_hz == 0) scl_hz = 12500000UL;
    if (fk < 2UL * 100000UL) return HAL_ERROR;
    uint32_t n  = (fk + scl_hz - 1) / scl_hz;
    uint32_t hi = _cycles(fk, 40);
    if (hi < n / 2) hi = n / 2;                               /* 50 % duty below the maximum */
    hi = _clamp(hi, 1, 256);
    uint32_t lo = n > hi ? n - hi : 1;
    if (lo < hi) lo = hi;
    lo = _clamp(lo, 1, 256);
    if (od_ns == 0) od_ns = 2000;
    uint32_t od = _clamp(_cycles(fk, od_ns), _cycles(fk, 200), 256);
    uint32_t i2ch = _clamp(_cycles(fk, 600), 1, 256);         /* legacy Fm, unused here */
    /* FREE: tCBP = (FREE+1)/fk and tCAS = ((FREE+1)*2 - (0.5+SDA_HD))/fk;
     * 250 ns covers tCBP >= 19.2 ns, tCAS >= 38.4 ns with room. */
    uint32_t fr   = _clamp(_cycles(fk, 250), 1, 128) - 1;
    uint32_t sdhd = fk > 100000000UL ? 1U : 0U;               /* hold >= ~6 ns */
    uint32_t aval = _clamp(fk / 1000000UL, 2, 256) - 1;        /* 1 µs time unit */

    h->instance->TIMINGR0 = LL_I3C_TIMINGR0(i2ch - 1, od - 1, hi - 1, lo - 1);
    h->instance->TIMINGR1 = LL_I3C_TIMINGR1(sdhd, fr, aval);
    h->instance->TIMINGR2 = 0;
    h->scl_hz    = fk / (hi + lo);
    h->od_low_ns = (uint32_t)(((uint64_t)od * 1000000000ULL) / fk);
    return HAL_OK;
}

/* ============================================================
 * Lifecycle
 * ============================================================ */

hal_status_t hal_i3c_init(hal_i3c_t *h, I3C_TypeDef *instance, const hal_i3c_config_t *cfg)
{
    if (!h || !cfg || _inst_idx(instance) < 0) return HAL_ERROR;
    memset(h, 0, sizeof(*h));
    h->instance   = instance;
    h->timeout_ms = cfg->timeout_ms ? cfg->timeout_ms : 10;
    for (int i = 0; i < HAL_I3C_MAX_TARGETS; i++) { h->targets[i].bcr = 0xFF; h->targets[i].dcr = 0xFF; }

    ll_i3c_clk_enable_reset(instance);
    if (cfg->kernel == HAL_I3C_KERNEL_PCLK) {
        if (!cfg->pclk_hz) return HAL_ERROR;
        ll_i3c_set_kernel_clk(instance, LL_I3C_KER_PCLK);
        h->kernel_hz = cfg->pclk_hz;
    } else {
        ll_i3c_set_kernel_clk(instance, LL_I3C_KER_HSI);
        h->kernel_hz = ll_i3c_hsi_hz();
    }
    uint32_t scl = cfg->scl_hz ? cfg->scl_hz : 12500000UL;
    if (h->kernel_hz < 2UL * scl) scl = h->kernel_hz / 2;          /* §49.6.2 */
    if (_timing(h, scl, cfg->od_low_ns) != HAL_OK) return HAL_ERROR;

    I3C_TypeDef *i = instance;
    i->DEVR0 = (uint32_t)_OWN_DA << LL_I3C_DEVR_DA_SHIFT;
    for (int k = 0; k < 4; k++) i->DEVR[k] = 0;
    i->IER  = 0;
    i->CEVR = LL_I3C_CEVR_ALL_CTRL;
    /* CRINIT and EN in one write (§49.16.3: CRINIT/HKSDAEN change only with EN 0,
     * or together with setting it). */
    i->CFGR = LL_I3C_CFGR_CRINIT | LL_I3C_CFGR_EN |
              (cfg->no_arb_header ? LL_I3C_CFGR_NOARBH : 0);
    return HAL_OK;
}

void hal_i3c_deinit(hal_i3c_t *h)
{
    if (!h || !h->instance) return;
    int x = _inst_idx(h->instance);
    if (x >= 0) {
        hal_nvic_disable_irq(x ? LL_I3C2_EV_IRQn : LL_I3C1_EV_IRQn);
        hal_nvic_disable_irq(x ? LL_I3C2_ER_IRQn : LL_I3C1_ER_IRQn);
        if (_i3c_h[x] == h) _i3c_h[x] = 0;
    }
    h->instance->IER  = 0;
    h->instance->CFGR = 0;
    ll_i3c_clk_disable(h->instance);
    h->instance = 0;
}

/* ============================================================
 * Frame engine
 * ============================================================ */

typedef struct {
    uint32_t       cr;       /* control word (I3C_CR) */
    uint8_t        pre[2];   /* bytes sent before `tx` (register address) */
    uint8_t        npre;
    const uint8_t *tx;       /* write data */
    uint16_t       ntx;
    uint8_t       *rx;       /* read data */
    uint16_t       nrx;
} _msg_t;

/* Partial reset after a timeout: flush every FIFO and cycle EN, which resets
 * the kernel and SCL domains and keeps the registers (§49.16.3 EN). */
static void _recover(hal_i3c_t *h)
{
    I3C_TypeDef *i = h->instance;
    uint32_t cfgr = i->CFGR & ~(LL_I3C_CFGR_CFLUSH | LL_I3C_CFGR_SFLUSH |
                                LL_I3C_CFGR_TXFLUSH | LL_I3C_CFGR_RXFLUSH | LL_I3C_CFGR_TSFSET);
    i->CFGR = cfgr | LL_I3C_CFGR_CFLUSH | LL_I3C_CFGR_TXFLUSH | LL_I3C_CFGR_RXFLUSH;
    i->CFGR = cfgr & ~LL_I3C_CFGR_EN;
    i->CFGR = cfgr;
    i->CEVR = LL_I3C_EVR_FCF | LL_I3C_EVR_ERRF | LL_I3C_EVR_RXTGTENDF;
}

static hal_status_t _ser_status(uint32_t ser)
{
    if (ser & (LL_I3C_SER_ANACK | LL_I3C_SER_DNACK)) return HAL_NACK;
    /* CE2: nobody acknowledged the 0x7E broadcast address (§49.12.1). */
    if ((ser & LL_I3C_SER_PERR) && (ser & LL_I3C_SER_CODERR_MASK) == 2U) return HAL_NACK;
    return HAL_ERROR;
}

/* Byte `k` of message m's outgoing data (register-address prefix, then data). */
static uint8_t _txbyte(const _msg_t *m, uint32_t k)
{
    return k < m->npre ? m->pre[k] : m->tx[k - m->npre];
}

static hal_status_t _frame(hal_i3c_t *h, const _msg_t *m, uint32_t n)
{
    I3C_TypeDef *i = h->instance;
    uint32_t mi = 0, tm = 0, ti = 0, rm = 0, ri = 0;
    uint32_t t0 = hal_tick();
    uint32_t guard = h->timeout_ms * 400000UL + 400000UL;   /* also bounded if SysTick is masked */

    /* Previous frame's leftovers: nothing may be queued. */
    if (!(i->EVR & LL_I3C_EVR_CFEF)) _recover(h);
    i->CFGR |= LL_I3C_CFGR_RXFLUSH | LL_I3C_CFGR_TXFLUSH;
    i->CEVR = LL_I3C_EVR_FCF | LL_I3C_EVR_ERRF | LL_I3C_EVR_RXTGTENDF;
    h->last_abt = 0;

    i->CR = m[mi++].cr;                         /* no preload: this starts the frame */
    for (;;) {
        uint32_t evr = i->EVR;
        if (evr & LL_I3C_EVR_RXFNEF) {
            while (rm < n && ri >= m[rm].nrx) { rm++; ri = 0; }
            uint8_t b = (uint8_t)i->RDR;
            if (rm < n) m[rm].rx[ri++] = b;
            continue;
        }
        if (evr & LL_I3C_EVR_ERRF) {
            uint32_t ser = i->SER;
            h->last_ser = ser;
            h->last_xdcnt = (uint16_t)(i->SR & LL_I3C_SR_XDCNT_MASK);
            i->CEVR = LL_I3C_EVR_ERRF;
            i->CFGR |= LL_I3C_CFGR_RXFLUSH | LL_I3C_CFGR_TXFLUSH | LL_I3C_CFGR_CFLUSH;
            return _ser_status(ser);
        }
        if (evr & LL_I3C_EVR_TXFNFF) {
            while (tm < n && ti >= (uint32_t)m[tm].npre + m[tm].ntx) { tm++; ti = 0; }
            if (tm < n) { i->TDR = _txbyte(&m[tm], ti++); continue; }
        }
        if ((evr & LL_I3C_EVR_CFNFF) && mi < n) {
            i->CR = m[mi++].cr;
            continue;
        }
        if (evr & LL_I3C_EVR_RXTGTENDF) {
            /* The target ended a private read early: SR.XDCNT is what arrived.
             * Its remaining bytes belong to no one; move to the next reader. */
            h->last_abt = 1;
            h->last_xdcnt = (uint16_t)(i->SR & LL_I3C_SR_XDCNT_MASK);
            while (rm < n && m[rm].nrx == 0) rm++;
            if (rm < n) { rm++; ri = 0; }
            i->CEVR = LL_I3C_EVR_RXTGTENDF;
            continue;
        }
        if (evr & LL_I3C_EVR_FCF) {
            while (i->EVR & LL_I3C_EVR_RXFNEF) {
                while (rm < n && ri >= m[rm].nrx) { rm++; ri = 0; }
                uint8_t b = (uint8_t)i->RDR;
                if (rm < n) m[rm].rx[ri++] = b;
            }
            if (!h->last_abt) h->last_xdcnt = (uint16_t)(i->SR & LL_I3C_SR_XDCNT_MASK);
            i->CEVR = LL_I3C_EVR_FCF;
            return HAL_OK;
        }
        if (hal_timeout_expired(t0, h->timeout_ms) || --guard == 0) {
            h->last_ser = i->SER;
            _recover(h);
            return HAL_TIMEOUT;
        }
    }
}

static uint32_t _cr_ccc(uint8_t ccc, uint16_t dcnt, int last)
{
    return (last ? LL_I3C_CR_MEND : 0) | LL_I3C_CR_MTYPE_CCC |
           ((uint32_t)ccc << LL_I3C_CR_CCC_SHIFT) | dcnt;
}

static uint32_t _cr_msg(uint32_t mtype, uint8_t addr, int rnw, uint16_t dcnt, int last)
{
    return (last ? LL_I3C_CR_MEND : 0) | mtype |
           ((uint32_t)(addr & 0x7FU) << LL_I3C_CR_ADD_SHIFT) |
           (rnw ? LL_I3C_CR_RNW : 0) | dcnt;
}

/* ============================================================
 * Address table
 * ============================================================ */

/* 0x00-0x07 are reserved and 0x7E is the broadcast address; 0x3E, 0x5E,
 * 0x6E, 0x76, 0x7A, 0x7C and 0x7F are one bit away from 0x7E and are not to
 * be assigned (MIPI I3C v1.1 §5.1.2.2.1, Table 8). */
static int _da_legal(uint8_t a)
{
    if (a < 0x08U || a > 0x7DU) return 0;
    switch (a) {
    case 0x3E: case 0x5E: case 0x6E: case 0x76: case 0x7A: case 0x7C: return 0;
    default: return a != _OWN_DA;
    }
}

hal_i3c_target_t *hal_i3c_target_by_dyn(hal_i3c_t *h, uint8_t dyn_addr)
{
    for (int k = 0; k < HAL_I3C_MAX_TARGETS; k++)
        if (h->targets[k].dyn_addr && h->targets[k].dyn_addr == dyn_addr) return &h->targets[k];
    return 0;
}

hal_i3c_target_t *hal_i3c_target_by_static(hal_i3c_t *h, uint8_t static_addr)
{
    if (!static_addr) return 0;
    for (int k = 0; k < HAL_I3C_MAX_TARGETS; k++)
        if (h->targets[k].dyn_addr && h->targets[k].static_addr == static_addr) return &h->targets[k];
    return 0;
}

static hal_i3c_target_t *_free_entry(hal_i3c_t *h)
{
    for (int k = 0; k < HAL_I3C_MAX_TARGETS; k++)
        if (!h->targets[k].dyn_addr) return &h->targets[k];
    return 0;
}

/* A free dynamic address: `want` when given, legal and free, else the lowest
 * pool address that is neither assigned nor a known static address. The pool
 * never hands out a static address: a target that loses its dynamic address
 * (its own reset clears it) answers its static address again as a legacy I2C
 * device, so a dynamic address equal to it would be ACKed by the wrong
 * protocol and the loss could not be detected (benched 2026-09-27: the
 * ICM-42686-P after a soft reset, with DA 0x69 = its static address). */
static uint8_t _pick_da(hal_i3c_t *h, uint8_t want)
{
    if (want && _da_legal(want) && !hal_i3c_target_by_dyn(h, want)) return want;
    for (uint8_t a = _DA_POOL_FIRST; a <= 0x7DU; a++) {
        if (!_da_legal(a) || hal_i3c_target_by_dyn(h, a)) continue;
        int is_static = 0;
        for (int k = 0; k < HAL_I3C_MAX_TARGETS; k++)
            if (h->targets[k].static_addr == a) is_static = 1;
        if (!is_static) return a;
    }
    return 0;
}

/* ============================================================
 * Dynamic addressing
 * ============================================================ */

hal_status_t hal_i3c_rstdaa(hal_i3c_t *h)
{
    _msg_t m = { .cr = _cr_ccc(LL_I3C_CCC_RSTDAA, 0, 1) };
    hal_status_t s = _frame(h, &m, 1);
    for (int k = 0; k < 4; k++) {
        h->instance->DEVR[k] &= ~(LL_I3C_DEVRX_IBIACK | LL_I3C_DEVRX_CRACK);
    }
    for (int k = 0; k < HAL_I3C_MAX_TARGETS; k++) {
        memset(&h->targets[k], 0, sizeof(h->targets[k]));
        h->targets[k].bcr = 0xFF; h->targets[k].dcr = 0xFF;
    }
    return s;
}

hal_status_t hal_i3c_setdasa(hal_i3c_t *h, uint8_t static_addr, uint8_t dyn_addr)
{
    hal_i3c_target_t *t = hal_i3c_target_by_static(h, static_addr);
    if (!t) t = _free_entry(h);
    if (!t) return HAL_ERROR;
    if (!t->dyn_addr) t->static_addr = static_addr;     /* reserve it before picking */
    uint8_t da = t->dyn_addr ? t->dyn_addr : _pick_da(h, dyn_addr);
    if (!da || da == static_addr) { if (!t->dyn_addr) t->static_addr = 0; return HAL_ERROR; }
    /* SETDASA: direct write CCC to the static address, one data byte = the
     * dynamic address in bits [7:1] (MIPI I3C v1.1 §5.1.9.3.10). */
    uint8_t b = (uint8_t)(da << 1);
    _msg_t m[2] = {
        { .cr = _cr_ccc(LL_I3C_CCC_SETDASA, 0, 0) },
        { .cr = _cr_msg(LL_I3C_CR_MTYPE_DIRECT, static_addr, 0, 1, 1), .tx = &b, .ntx = 1 },
    };
    hal_status_t s = _frame(h, m, 2);
    if (s != HAL_OK) {
        if (!t->dyn_addr) t->static_addr = 0;
        return s;
    }
    t->static_addr = static_addr;
    t->dyn_addr = da;
    return HAL_OK;
}

hal_status_t hal_i3c_entdaa(hal_i3c_t *h, uint8_t *found)
{
    I3C_TypeDef *i = h->instance;
    uint8_t got = 0, buf[8], k = 0;
    hal_i3c_target_t *pending = 0;
    uint32_t t0 = hal_tick();
    uint32_t guard = h->timeout_ms * 400000UL * 4UL + 400000UL;

    if (found) *found = 0;
    if (!(i->EVR & LL_I3C_EVR_CFEF)) _recover(h);
    i->CFGR |= LL_I3C_CFGR_RXFLUSH | LL_I3C_CFGR_TXFLUSH;
    i->CEVR = LL_I3C_EVR_FCF | LL_I3C_EVR_ERRF | LL_I3C_EVR_RXTGTENDF;
    /* Figure 664: one broadcast CCC 0x07, DCNT 0; per target the controller
     * reads 8 bytes (48-bit PID MSB first, BCR, DCR) into the RX-FIFO, then
     * asks for the 7-bit address to assign through the TX-FIFO. The frame
     * ends when no target acknowledges 0x7E/R; SR.XDCNT = targets assigned. */
    i->CR = _cr_ccc(LL_I3C_CCC_ENTDAA, 0, 1);
    for (;;) {
        uint32_t evr = i->EVR;
        if (evr & LL_I3C_EVR_RXFNEF) {
            uint8_t b = (uint8_t)i->RDR;
            if (k < 8) buf[k++] = b;
            continue;
        }
        if (evr & LL_I3C_EVR_ERRF) {
            uint32_t ser = i->SER;
            h->last_ser = ser;
            i->CEVR = LL_I3C_EVR_ERRF;
            i->CFGR |= LL_I3C_CFGR_RXFLUSH | LL_I3C_CFGR_TXFLUSH | LL_I3C_CFGR_CFLUSH;
            if (pending) pending->dyn_addr = 0;
            if (found) *found = got;
            return got ? HAL_OK : _ser_status(ser);
        }
        if ((evr & LL_I3C_EVR_TXFNFF) && k == 8) {
            /* Commit the previous target (its address went out). */
            if (pending) got++;
            pending = _free_entry(h);
            uint8_t da = pending ? _pick_da(h, 0) : 0;
            if (!pending || !da) { _recover(h); if (found) *found = got; return HAL_ERROR; }
            memset(pending, 0, sizeof(*pending));
            memcpy(pending->pid, buf, 6);
            pending->bcr = buf[6];
            pending->dcr = buf[7];
            pending->dyn_addr = da;
            i->TDR = da;          /* 7-bit address in TDB0[6:0]; the hardware adds the parity */
            k = 0;
            continue;
        }
        if (evr & LL_I3C_EVR_FCF) {
            uint16_t n = (uint16_t)(i->SR & LL_I3C_SR_XDCNT_MASK);
            i->CEVR = LL_I3C_EVR_FCF;
            if (pending) got++;
            h->last_xdcnt = n;
            if (found) *found = got;
            return got ? HAL_OK : HAL_NACK;
        }
        if (hal_timeout_expired(t0, h->timeout_ms * 4U) || --guard == 0) {
            if (pending) pending->dyn_addr = 0;
            _recover(h);
            if (found) *found = got;
            return HAL_TIMEOUT;
        }
    }
}

/* ============================================================
 * CCCs
 * ============================================================ */

hal_status_t hal_i3c_ccc_broadcast(hal_i3c_t *h, uint8_t ccc, const uint8_t *data, uint16_t len)
{
    if (ccc & 0x80U) return HAL_ERROR;
    _msg_t m = { .cr = _cr_ccc(ccc, len, 1), .tx = data, .ntx = len };
    return _frame(h, &m, 1);
}

hal_status_t hal_i3c_ccc_write(hal_i3c_t *h, uint8_t ccc, uint8_t dyn_addr,
                               const uint8_t *data, uint16_t len)
{
    if (!(ccc & 0x80U)) return HAL_ERROR;
    _msg_t m[2] = {
        { .cr = _cr_ccc(ccc, 0, 0) },
        { .cr = _cr_msg(LL_I3C_CR_MTYPE_DIRECT, dyn_addr, 0, len, 1), .tx = data, .ntx = len },
    };
    return _frame(h, m, 2);
}

hal_status_t hal_i3c_ccc_read(hal_i3c_t *h, uint8_t ccc, uint8_t dyn_addr,
                              uint8_t *buf, uint16_t len, uint16_t *got)
{
    if (!(ccc & 0x80U) || !len) return HAL_ERROR;
    _msg_t m[2] = {
        { .cr = _cr_ccc(ccc, 0, 0) },
        { .cr = _cr_msg(LL_I3C_CR_MTYPE_DIRECT, dyn_addr, 1, len, 1), .rx = buf, .nrx = len },
    };
    hal_status_t s = _frame(h, m, 2);
    if (got) *got = s == HAL_OK ? h->last_xdcnt : 0;
    return s;
}

hal_status_t hal_i3c_get_pid(hal_i3c_t *h, uint8_t dyn_addr, uint8_t pid[6])
{
    hal_status_t s = hal_i3c_ccc_read(h, LL_I3C_CCC_GETPID, dyn_addr, pid, 6, 0);
    hal_i3c_target_t *t = s == HAL_OK ? hal_i3c_target_by_dyn(h, dyn_addr) : 0;
    if (t) memcpy(t->pid, pid, 6);
    return s;
}

hal_status_t hal_i3c_get_bcr(hal_i3c_t *h, uint8_t dyn_addr, uint8_t *bcr)
{
    hal_status_t s = hal_i3c_ccc_read(h, LL_I3C_CCC_GETBCR, dyn_addr, bcr, 1, 0);
    hal_i3c_target_t *t = s == HAL_OK ? hal_i3c_target_by_dyn(h, dyn_addr) : 0;
    if (t) t->bcr = *bcr;
    return s;
}

hal_status_t hal_i3c_get_dcr(hal_i3c_t *h, uint8_t dyn_addr, uint8_t *dcr)
{
    hal_status_t s = hal_i3c_ccc_read(h, LL_I3C_CCC_GETDCR, dyn_addr, dcr, 1, 0);
    hal_i3c_target_t *t = s == HAL_OK ? hal_i3c_target_by_dyn(h, dyn_addr) : 0;
    if (t) t->dcr = *dcr;
    return s;
}

hal_status_t hal_i3c_get_status(hal_i3c_t *h, uint8_t dyn_addr, uint16_t *status)
{
    uint8_t b[2] = { 0, 0 };
    hal_status_t s = hal_i3c_ccc_read(h, LL_I3C_CCC_GETSTATUS, dyn_addr, b, 2, 0);
    if (s == HAL_OK && status) *status = (uint16_t)((b[0] << 8) | b[1]);
    return s;
}

hal_status_t hal_i3c_enec(hal_i3c_t *h, uint8_t dyn_addr, uint8_t events)
{
    if (dyn_addr == 0x7EU) return hal_i3c_ccc_broadcast(h, LL_I3C_CCC_ENEC_BC, &events, 1);
    return hal_i3c_ccc_write(h, LL_I3C_CCC_ENEC, dyn_addr, &events, 1);
}

hal_status_t hal_i3c_disec(hal_i3c_t *h, uint8_t dyn_addr, uint8_t events)
{
    if (dyn_addr == 0x7EU) return hal_i3c_ccc_broadcast(h, LL_I3C_CCC_DISEC_BC, &events, 1);
    return hal_i3c_ccc_write(h, LL_I3C_CCC_DISEC, dyn_addr, &events, 1);
}

/* ============================================================
 * Private transfers
 * ============================================================ */

hal_status_t hal_i3c_write(hal_i3c_t *h, uint8_t dyn_addr, const uint8_t *data, uint16_t len)
{
    if (!len) return HAL_ERROR;                  /* DCNT must be non-null (§49.16.1) */
    _msg_t m = { .cr = _cr_msg(LL_I3C_CR_MTYPE_PRIVATE, dyn_addr, 0, len, 1), .tx = data, .ntx = len };
    return _frame(h, &m, 1);
}

hal_status_t hal_i3c_read(hal_i3c_t *h, uint8_t dyn_addr, uint8_t *buf, uint16_t len, uint16_t *got)
{
    if (!len) return HAL_ERROR;
    _msg_t m = { .cr = _cr_msg(LL_I3C_CR_MTYPE_PRIVATE, dyn_addr, 1, len, 1), .rx = buf, .nrx = len };
    hal_status_t s = _frame(h, &m, 1);
    if (got) *got = s == HAL_OK ? h->last_xdcnt : 0;
    return s;
}

static uint8_t _reg_prefix(uint16_t reg, uint8_t pre[2])
{
    if (reg > 0xFFU) { pre[0] = (uint8_t)(reg >> 8); pre[1] = (uint8_t)reg; return 2; }
    pre[0] = (uint8_t)reg;
    return 1;
}

hal_status_t hal_i3c_write_reg(hal_i3c_t *h, uint8_t dyn_addr, uint16_t reg,
                               const uint8_t *data, uint16_t len)
{
    _msg_t m = { .tx = data, .ntx = len };
    m.npre = _reg_prefix(reg, m.pre);
    m.cr = _cr_msg(LL_I3C_CR_MTYPE_PRIVATE, dyn_addr, 0, (uint16_t)(m.npre + len), 1);
    return _frame(h, &m, 1);
}

hal_status_t hal_i3c_read_reg(hal_i3c_t *h, uint8_t dyn_addr, uint16_t reg,
                              uint8_t *buf, uint16_t len)
{
    if (!len) return HAL_ERROR;
    _msg_t m[2] = { { 0 }, { .rx = buf, .nrx = len } };
    m[0].npre = _reg_prefix(reg, m[0].pre);
    m[0].cr = _cr_msg(LL_I3C_CR_MTYPE_PRIVATE, dyn_addr, 0, m[0].npre, 0);
    m[1].cr = _cr_msg(LL_I3C_CR_MTYPE_PRIVATE, dyn_addr, 1, len, 1);
    hal_status_t s = _frame(h, m, 2);
    if (s == HAL_OK && h->last_abt) return HAL_ERROR;   /* fewer bytes than asked */
    return s;
}

/* ============================================================
 * In-band interrupts (RM0481 §49.9.13, §49.16.17)
 * ============================================================ */

static void _isr(hal_i3c_t *h)
{
    I3C_TypeDef *i = h->instance;
    uint32_t evr = i->EVR;
    if (evr & LL_I3C_EVR_IBIF) {
        uint32_t rmr = i->RMR, d = i->IBIDR;
        /* Clearing IBIF re-opens the bus to IBIs: until then every further
         * IBI is NACKed (§49.16.17 IBIACK). */
        i->CEVR = LL_I3C_EVR_IBIF;
        uint8_t p[4] = { (uint8_t)d, (uint8_t)(d >> 8), (uint8_t)(d >> 16), (uint8_t)(d >> 24) };
        uint8_t n = (uint8_t)(rmr & LL_I3C_RMR_IBIRDCNT_MASK);
        uint8_t a = (uint8_t)((rmr >> LL_I3C_RMR_RADD_SHIFT) & 0x7FU);
        h->ibi_count++;
        h->ibi_last_addr = a;
        if (h->ibi_cb) h->ibi_cb(h->ibi_ctx, a, p, n > 4 ? 4 : n);
    }
    if (evr & (LL_I3C_EVR_HJF | LL_I3C_EVR_CRF))
        i->CEVR = evr & (LL_I3C_EVR_HJF | LL_I3C_EVR_CRF);
}

void hal_i3c_irq(hal_i3c_t *h)
{
    if (h && h->instance) _isr(h);
}

hal_status_t hal_i3c_ibi_enable(hal_i3c_t *h, uint8_t dyn_addr, hal_i3c_ibi_cb_t cb, void *ctx)
{
    hal_i3c_target_t *t = hal_i3c_target_by_dyn(h, dyn_addr);
    if (!t) return HAL_ERROR;
    if (t->bcr == 0xFF) {
        uint8_t bcr;
        hal_status_t s = hal_i3c_get_bcr(h, dyn_addr, &bcr);
        if (s != HAL_OK) return s;
    }
    uint8_t slot = t->ibi_slot;
    if (!slot) {
        uint8_t used = 0;
        for (int k = 0; k < HAL_I3C_MAX_TARGETS; k++)
            if (h->targets[k].dyn_addr && h->targets[k].ibi_slot) used |= (uint8_t)(1U << h->targets[k].ibi_slot);
        for (uint8_t s = 1; s <= HAL_I3C_IBI_SLOTS; s++)
            if (!(used & (1U << s))) { slot = s; break; }
        if (!slot) return HAL_ERROR;
    }
    I3C_TypeDef *i = h->instance;
    volatile uint32_t *devr = &i->DEVR[slot - 1];
    /* DA and IBIDEN are locked while DIS is set, i.e. while IBIACK/CRACK is
     * set: drop the acknowledge, wait for DIS to clear, then rewrite. */
    *devr &= ~(LL_I3C_DEVRX_IBIACK | LL_I3C_DEVRX_CRACK);
    for (uint32_t w = 100000; (*devr & LL_I3C_DEVRX_DIS) && w; w--) { }
    *devr = ((uint32_t)dyn_addr << LL_I3C_DEVR_DA_SHIFT) |
            ((t->bcr & 0x04U) ? LL_I3C_DEVRX_IBIDEN : 0);
    *devr |= LL_I3C_DEVRX_IBIACK;
    t->ibi_slot = slot;

    int x = _inst_idx(i);
    h->ibi_cb = cb;
    h->ibi_ctx = ctx;
    _i3c_isr = _isr;
    _i3c_h[x] = h;
    i->CEVR = LL_I3C_EVR_IBIF;
    i->IER |= LL_I3C_IER_IBIIE;
    uint32_t irq = x ? LL_I3C2_EV_IRQn : LL_I3C1_EV_IRQn;
    hal_nvic_set_priority(irq, 2);
    hal_nvic_enable_irq(irq);
    return HAL_OK;
}

hal_status_t hal_i3c_ibi_disable(hal_i3c_t *h, uint8_t dyn_addr)
{
    hal_i3c_target_t *t = hal_i3c_target_by_dyn(h, dyn_addr);
    if (!t || !t->ibi_slot) return HAL_ERROR;
    h->instance->DEVR[t->ibi_slot - 1] &= ~LL_I3C_DEVRX_IBIACK;
    t->ibi_slot = 0;
    return HAL_OK;
}

#endif /* LL_I3C_AVAILABLE */
