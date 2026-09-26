/**
 * l0-adc-mux — six-channel ADC sensor hub with an I2C target interface.
 *
 * A Core.ST.L0 that answers as an I2C device at 0x28. It samples six
 * analog inputs into a DMA ring at a TIM2-paced scan rate, averages each
 * channel over a configurable window, and publishes calibrated millivolts
 * into a register file the host reads. 100 Hz output by default; the
 * rate and averaging window can be changed over I2C.
 *
 * Register map. Multi-byte values are LITTLE-endian, which is the
 * Core's native order, so a host on a little-endian machine can memcpy
 * the block straight into a uint16 array.
 *
 *   0x00  WHO_AM_I   0x6D, constant
 *   0x01  VERSION    0x13 = v1.3
 *   0x02  STATUS     bit0 sample valid
 *                    bit1 DMA running
 *                    bit2 VDDA measured against VREFINT (clear: the
 *                         3.3 V nominal fallback is in use)
 *   0x03  SEQ        increments once per published set, wraps at 255
 *   0x04  CH0 mV     pad 2  (ADC3)
 *   0x06  CH1 mV     pad 3  (ADC0)
 *   0x08  CH2 mV     pad 6  (ADC1)
 *   0x0A  CH3 mV     pad 7  (ADC2)
 *   0x0C  CH4 mV     pad 8  (ADC5)
 *   0x0E  CH5 mV     pad 9  (ADC8)
 *   0x10  VDDA mV    supply rail, measured once at power-up
 *   0x12  ADDR_CUR   the address this hub is answering on right now
 *   0x13  ADDR_SET   writable, staged address for the next commit
 *   0x14  COMMIT     writable, see "Saving"
 *   0x15  RATE_HZ    writable, output rate in Hz          (1..250)
 *   0x16  OVERSAMP   writable, scans averaged per period  (1..32)
 *   0x17  WINDOW     writable, periods averaged           (1 or 2)
 *   0x18  APPLY      writable, see "Acquisition control"
 *
 * Only 0x13..0x18 accept host writes. Everything else is read-only:
 * writes to it set the register pointer and are otherwise absorbed. The
 * data block 0x02..0x0F is unchanged from v1.0, so existing hosts that
 * burst-read it keep working.
 *
 * Acquisition control. TIM2 triggers one scan of all six channels at
 * RATE_HZ x OVERSAMP scans per second, and every 1/RATE_HZ seconds the
 * hub publishes the average of the most recent WINDOW x OVERSAMP scans.
 *
 *   WINDOW 1  boxcar over one output period: nulls at RATE_HZ and every
 *             multiple of it.
 *   WINDOW 2  moving boxcar over two periods, still published every
 *             period: nulls at RATE_HZ / 2 and every multiple. This is
 *             the mains-rejection setting.
 *
 *   rejecting 50 Hz at 100 Hz out:  RATE 100, WINDOW 2   (20 ms window)
 *   rejecting 60 Hz at 120 Hz out:  RATE 120, WINDOW 2   (16.7 ms window)
 *   rejecting both, slowly:         RATE 10,  WINDOW 1   (100 ms window)
 *
 * The three settings only make sense together, so they are staged and
 * then applied as a set:
 *
 *   1. write RATE_HZ, OVERSAMP, WINDOW (one burst from 0x15 is fine)
 *   2. write 0x5A to APPLY
 *   3. read APPLY back:
 *        0x00 idle          0x01 applied
 *        0xE1 rejected: the staged registers are restored to the live
 *             values, so reading 0x15..0x17 always shows what is running
 *             once APPLY has been answered
 *
 * Limits, and why: OVERSAMP x WINDOW is capped at 32 scans because the
 * DMA ring costs 12 bytes per scan on a 2 KB part; RATE_HZ x OVERSAMP is
 * capped at 8000 scans/s so a scan (about 65 us at the current sampling
 * time) always finishes well before the next trigger.
 *
 * A change applies within one output period. While the ring refills
 * with samples taken under the new settings, STATUS.valid is clear and
 * SEQ does not advance, so a host never averages across a change.
 *
 * Saving. COMMIT stores the staged address AND the live acquisition
 * settings to the L011's 512 bytes of true EEPROM:
 *
 *   1. write the wanted address (0x08..0x77) to ADDR_SET
 *   2. write 0xA5 to COMMIT
 *   3. read COMMIT back for the result:
 *        0x00 idle          0x01 stored
 *        0xE1 address out of range (nothing stored)
 *        0xE2 EEPROM write failed or did not read back
 *   4. reset the hub if the address changed
 *
 * Acquisition settings are live the moment they are applied and COMMIT
 * only makes them survive a reset. The address is different: it applies
 * at the NEXT RESET, not immediately. Changing it mid-transaction would
 * break the very transfer that requested it, and a board that moves
 * address underneath a running host is far worse to debug than one that
 * needs a reset. The two-step with a magic byte is deliberate: a single
 * writable address register would let one stray write silently move a
 * deployed board off the bus, and that looks identical to a dead board.
 *
 * Commit on a bench or a jig, not on a live array. The L011 is a
 * single-bank part, so per RM0377 any NVM read during a write stalls
 * the master until it finishes. Code runs from flash, so the CPU (and
 * with it the I2C ISR) stalls for about 3.2 ms per byte written. Only
 * bytes that changed are written, so a commit costs between 0 and about
 * 29 ms. Clock stretching keeps the bus correct rather than corrupt, but
 * the host needs a timeout longer than that. Other hubs are unaffected.
 *
 * A blank or corrupt EEPROM falls back to HUB_I2C_ADDR_DEFAULT and the
 * default acquisition settings, so an unprogrammed board still answers.
 * Override the default address per build with:
 *
 *   make EXTRA_CFLAGS=-DHUB_I2C_ADDR_DEFAULT=0x2A
 *
 * but prefer committing to EEPROM: one firmware image for every board
 * in the array beats tracking which image went where.
 *
 * Typical host transaction: write the pointer 0x02, then burst-read 14
 * bytes. That returns STATUS, SEQ and all six channels from ONE coherent
 * sample set, because the driver latches the published snapshot into the
 * register file at the start of a read and nothing rewrites it mid
 * transfer. A SEQ that has not changed between two reads means the host
 * is polling faster than RATE_HZ, not that the hub is stuck.
 *
 * Six is the hard ceiling here, not a round number: pads 2, 3, 6, 7, 8, 9
 * are every ADC-capable pad the tile has once I2C1 takes 4 and 5.
 */

#include "core.h"
#include "core_adc.h"
#include "core_i2c.h"
#include "core_timer.h"
#include "core_nvm.h"
#include "ll_dma.h"
#include <string.h>

/* ---- Bus identity ---- */
#ifndef HUB_I2C_ADDR_DEFAULT
#define HUB_I2C_ADDR_DEFAULT  0x28
#endif
#define HUB_WHO_AM_I    0x6D
#define HUB_VERSION     0x13   /* v1.3 — explicit sampling time, rounded averages */

/* Smallest and largest addresses I2C leaves to devices. Below 0x08 and
 * above 0x77 are reserved by the spec, and a board that took one would
 * be unreachable with no way to put it back. */
#define ADDR_MIN        0x08
#define ADDR_MAX        0x77

/* ---- EEPROM layout ----
 * Two independent records, each with its own magic and checksum, so a
 * board committed by v1.1 firmware (address only) keeps its address and
 * simply picks up the default acquisition settings. */
#define NVM_ADDR_OFF    0      /* 'M' 'X' addr xor */
#define NVM_MAGIC0      0x4D   /* 'M' */
#define NVM_MAGIC1      0x58   /* 'X' */
#define NVM_ADDR_LEN    4
#define NVM_CFG_OFF     4      /* 'C' rate oversamp window xor */
#define NVM_CFG_MAGIC   0x43   /* 'C' */
#define NVM_CFG_LEN     5

/* ---- COMMIT protocol ---- */
#define COMMIT_REQUEST  0xA5
#define COMMIT_IDLE     0x00
#define COMMIT_OK       0x01
#define COMMIT_ERR_ADDR 0xE1
#define COMMIT_ERR_NVM  0xE2

/* ---- APPLY protocol ---- */
#define APPLY_REQUEST   0x5A
#define APPLY_IDLE      0x00
#define APPLY_OK        0x01
#define APPLY_ERR       0xE1

/* ---- Register offsets ---- */
#define REG_WHO_AM_I    0x00
#define REG_VERSION     0x01
#define REG_STATUS      0x02
#define REG_SEQ         0x03
#define REG_CH0         0x04
#define REG_VDDA        0x10
#define REG_ADDR_CUR    0x12
#define REG_ADDR_SET    0x13
#define REG_COMMIT      0x14
#define REG_RATE        0x15
#define REG_OVERSAMP    0x16
#define REG_WINDOW      0x17
#define REG_APPLY       0x18
#define REG_COUNT       0x19

/* The target driver takes one contiguous writable range. */
#define REG_WRITABLE_FIRST  REG_ADDR_SET
#define REG_WRITABLE_COUNT  (REG_APPLY - REG_ADDR_SET + 1)

/* ---- STATUS bits ---- */
#define ST_VALID        (1U << 0)
#define ST_DMA          (1U << 1)
#define ST_VDDA         (1U << 2)

/* ---- Acquisition ----
 *
 * Scans are paced by TIM2 rather than free-running, so the scans in the
 * ring are spread evenly across the averaging window instead of landing
 * in the 60 us it takes the ADC to fill the ring flat out. Averaging a
 * burst reduces noise inside the burst and does nothing about aliasing;
 * averaging scans spread across the window is a boxcar filter with
 * nulls at 1/window and every multiple. It also stops the ADC
 * converting 133,000 scans/s and discarding almost all of them.
 *
 * Publishing is driven by the DMA's position in the ring, not by a
 * millisecond timer: each time OVERSAMP more scans have landed, one
 * output period has passed. That keeps the output rate locked to TIM2,
 * with no drift between two clocks, and lets RATE_HZ be any value
 * rather than only divisors of 1000, which is what 60 Hz rejection
 * needs. */
#define N_CH            6

#define RATE_DEFAULT        100
#define OVERSAMP_DEFAULT    8
#define WINDOW_DEFAULT      1

#define RATE_MAX            250
#define OVERSAMP_MAX        32
#define WINDOW_MAX          2
#define RING_SCANS_MAX      32      /* 12 B per scan -> 384 B of RAM */
#define SCAN_HZ_MAX         8000UL  /* a scan takes ~65 us at 160.5 cycles */

typedef struct {
    uint8_t rate_hz;    /* output rate */
    uint8_t oversamp;   /* scans per output period */
    uint8_t window;     /* output periods averaged */
} acq_cfg_t;

/* Channels in register order. Ascending pad order, so CH0..CH5 read left
 * to right on the board rather than in ADC channel-number order. */
static const uint8_t CH_PAD[N_CH] = { 2, 3, 6, 7, 8, 9 };

/* DMA ring, sized for the largest allowed window. Only the first
 * N_CH * oversamp * window entries are used under any given setting. */
static uint16_t dma_buf[N_CH * RING_SCANS_MAX];

/* Slot each channel occupies within one scan. Cached at startup because
 * the ADC converts in channel-number order, not the order pads were
 * registered, so this is NOT simply 0..5. */
static int ch_slot[N_CH];

/* Live acquisition state. Owned by the main loop. */
static acq_cfg_t acq;
static uint16_t  ring_len;      /* samples in use: N_CH * scans */
static uint16_t  ring_scans;    /* scans averaged: oversamp * window */
static uint16_t  period_len;    /* samples per output period */

/* Published snapshot. Written by the main loop under ll_irq_save, read by
 * the I2C ISR at the start of a host read. */
static volatile uint16_t pub_mv[N_CH];
static volatile uint8_t  pub_seq;
static volatile uint8_t  pub_status;

/* The register file the host actually reads. */
static uint8_t regs[REG_COUNT];

/* Set by the ISR, acted on by the main loop. An EEPROM byte takes about
 * 3.2 ms to program and an ADC reconfiguration has to stop DMA, both far
 * too slow and too disruptive to do inside an I2C interrupt. */
static volatile uint8_t commit_pending;
static volatile uint8_t apply_pending;

static core_i2c_target_t hub;
static core_timer_t      pacer;
static core_adc_t *const adc = &core_adc1;

/**
 * Called from the I2C ISR. At READ_START the peripheral is still
 * stretching SCL and no byte has gone out yet, which is the one moment a
 * whole consistent set can be dropped into the register file.
 */
static void on_i2c_event(void *ctx, hal_i2c_target_event_t evt,
                         uint16_t reg, uint8_t value)
{
    (void)ctx;

    if (evt == HAL_I2C_TARGET_WRITE) {
        /* Hand the slow parts to the main loop and return immediately. */
        if (reg == REG_COMMIT && value == COMMIT_REQUEST) commit_pending = 1;
        if (reg == REG_APPLY  && value == APPLY_REQUEST)  apply_pending  = 1;
        return;
    }

    if (evt != HAL_I2C_TARGET_READ_START) return;

    regs[REG_STATUS] = pub_status;
    regs[REG_SEQ]    = pub_seq;
    for (int i = 0; i < N_CH; i++) {
        uint16_t mv = pub_mv[i];
        regs[REG_CH0 + 2 * i]     = (uint8_t)(mv & 0xFF);
        regs[REG_CH0 + 2 * i + 1] = (uint8_t)(mv >> 8);
    }
}

/* ============================================================
 * EEPROM
 * ============================================================ */

/**
 * Write only the bytes that differ. Every byte programmed stalls the CPU
 * and the I2C ISR for about 3.2 ms, so re-committing an unchanged
 * setting should cost nothing, and it saves EEPROM wear too.
 */
static int nvm_update(uint32_t off, const uint8_t *b, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        uint8_t cur;
        if (core_nvm_read(off + i, &cur, 1) != 0) return -1;
        if (cur == b[i]) continue;
        if (core_nvm_write(off + i, &b[i], 1) != 0) return -1;
    }
    return 0;
}

/**
 * Read the stored address, or 0 if nothing valid is stored.
 *
 * Guarded by magic bytes plus a checksum, so a blank EEPROM (all 0x00
 * from the factory) and a half-written one both read as absent and fall
 * back to the compile-time default. A board that fails to program is
 * then still reachable, which is the whole point.
 */
static uint8_t addr_load(void)
{
    uint8_t b[NVM_ADDR_LEN];
    if (core_nvm_read(NVM_ADDR_OFF, b, NVM_ADDR_LEN) != 0) return 0;
    if (b[0] != NVM_MAGIC0 || b[1] != NVM_MAGIC1) return 0;
    if (b[3] != (uint8_t)(b[0] ^ b[1] ^ b[2])) return 0;
    if (b[2] < ADDR_MIN || b[2] > ADDR_MAX) return 0;
    return b[2];
}

/** Store an address, then read it back to confirm it actually took. */
static int addr_store(uint8_t addr)
{
    uint8_t b[NVM_ADDR_LEN];
    b[0] = NVM_MAGIC0;
    b[1] = NVM_MAGIC1;
    b[2] = addr;
    b[3] = (uint8_t)(b[0] ^ b[1] ^ b[2]);

    if (nvm_update(NVM_ADDR_OFF, b, NVM_ADDR_LEN) != 0) return -1;
    /* core_nvm_write does not check FLASH_SR, so verify rather than
     * trust it. Silently failing to store an address would strand the
     * board at the default and collide with its neighbour. */
    return (addr_load() == addr) ? 0 : -1;
}

/* ============================================================
 * Acquisition settings
 * ============================================================ */

static int acq_valid(const acq_cfg_t *c)
{
    if (c->rate_hz  < 1 || c->rate_hz  > RATE_MAX)     return 0;
    if (c->oversamp < 1 || c->oversamp > OVERSAMP_MAX) return 0;
    if (c->window   < 1 || c->window   > WINDOW_MAX)   return 0;
    if ((uint32_t)c->oversamp * c->window > RING_SCANS_MAX) return 0;
    if ((uint32_t)c->rate_hz * c->oversamp > SCAN_HZ_MAX)   return 0;
    return 1;
}

/** Stored settings, or 0 if absent, corrupt, or no longer within limits. */
static int acq_load(acq_cfg_t *out)
{
    uint8_t b[NVM_CFG_LEN];
    if (core_nvm_read(NVM_CFG_OFF, b, NVM_CFG_LEN) != 0) return 0;
    if (b[0] != NVM_CFG_MAGIC) return 0;
    if (b[4] != (uint8_t)(b[0] ^ b[1] ^ b[2] ^ b[3])) return 0;
    acq_cfg_t c = { b[1], b[2], b[3] };
    if (!acq_valid(&c)) return 0;
    *out = c;
    return 1;
}

static int acq_store(const acq_cfg_t *c)
{
    uint8_t b[NVM_CFG_LEN];
    b[0] = NVM_CFG_MAGIC;
    b[1] = c->rate_hz;
    b[2] = c->oversamp;
    b[3] = c->window;
    b[4] = (uint8_t)(b[0] ^ b[1] ^ b[2] ^ b[3]);
    if (nvm_update(NVM_CFG_OFF, b, NVM_CFG_LEN) != 0) return -1;

    acq_cfg_t back;
    if (!acq_load(&back)) return -1;
    return (back.rate_hz == c->rate_hz && back.oversamp == c->oversamp &&
            back.window == c->window) ? 0 : -1;
}

/** Mirror the live settings into the staging registers. */
static void acq_show(void)
{
    uint32_t s = ll_irq_save();
    regs[REG_RATE]     = acq.rate_hz;
    regs[REG_OVERSAMP] = acq.oversamp;
    regs[REG_WINDOW]   = acq.window;
    ll_irq_restore(s);
}

/* Ring position tracking, owned by the main loop. */
static uint16_t last_pos;
static uint8_t  periods_since_start;

/**
 * Stop, retune and restart acquisition under new settings.
 *
 * The order matters. TIM2's update event is the ADC trigger, and
 * retuning the timer generates one (set_freq writes EGR.UG), so the ADC
 * has to be stopped before the timer is touched or it converts a stray
 * scan into the new ring. The timer starts last, once the ADC is armed.
 *
 * Reconfiguring the ADC under a running DMA is exactly how the first
 * silicon run went wrong (a VREFINT conversion hijacked the channel
 * selection), so this only ever runs from the main loop, with DMA
 * stopped.
 */
static int acq_start(const acq_cfg_t *c)
{
    if (!acq_valid(c)) return -1;

    core_timer_stop(&pacer);
    core_adc_stop_dma(adc);

    acq        = *c;
    ring_scans = (uint16_t)(c->oversamp * c->window);
    ring_len   = (uint16_t)(N_CH * ring_scans);
    period_len = (uint16_t)(N_CH * c->oversamp);

    /* Not valid again until the ring holds only new samples. */
    uint32_t s = ll_irq_save();
    pub_status &= (uint8_t)~(ST_VALID | ST_DMA);
    ll_irq_restore(s);

    core_timer_set_freq(&pacer, (uint32_t)c->rate_hz * c->oversamp);

    int ok = core_adc_start_dma(adc, dma_buf, ring_len, NULL, NULL) == HAL_OK;
    if (ok) {
        s = ll_irq_save();
        pub_status |= ST_DMA;
        ll_irq_restore(s);
    }

    last_pos = 0;
    periods_since_start = 0;
    core_timer_start(&pacer);
    acq_show();
    return ok ? 0 : -1;
}

/* ============================================================
 * Publishing
 * ============================================================ */

/**
 * Average the ring down to one value per channel and publish it.
 *
 * The DMA keeps writing while we read, so the oldest scan in the ring
 * may already be partly replaced by the newest. Harmless: the ring still
 * holds a full window of evenly spaced scans, just shifted by one.
 */
static void publish(void)
{
    uint16_t snap[N_CH];

    for (int i = 0; i < N_CH; i++) {
        int slot = ch_slot[i];
        if (slot < 0) { snap[i] = 0; continue; }

        uint32_t acc = 0;
        for (uint16_t k = 0; k < ring_scans; k++) {
            acc += dma_buf[slot + k * N_CH];
        }
        /* Round to nearest: truncating biased every reading half a
         * count (about 0.4 mV at 3.3 V) low. */
        uint16_t raw = (uint16_t)((acc + ring_scans / 2) / ring_scans);
        snap[i] = (uint16_t)core_adc_raw_to_mv(adc, raw);
    }

    /* Publish as one unit. Short enough that blocking the I2C ISR here
     * costs at most a stretched clock, not a dropped transfer. */
    uint32_t s = ll_irq_save();
    for (int i = 0; i < N_CH; i++) pub_mv[i] = snap[i];
    pub_seq++;
    pub_status |= ST_VALID;
    ll_irq_restore(s);
}

/**
 * Has another output period of scans landed since the last call?
 *
 * The DMA counts down from ring_len and reloads at the end of the ring,
 * so the write position is ring_len - CNDTR. A period boundary is either
 * the position moving into the next period of the ring (window 2) or
 * wrapping back to the start (every window). The main loop polls this
 * far faster than the shortest period (4 ms), so no boundary is missed
 * outside an EEPROM commit, which stalls everything anyway.
 */
static int period_elapsed(void)
{
    uint16_t pos = (uint16_t)(ring_len - ll_dma_remaining(DMA1_CH1));
    if (pos >= ring_len) pos = 0;   /* CNDTR reads ring_len at reload */

    int crossed = (pos < last_pos) ||
                  (pos / period_len != last_pos / period_len);
    last_pos = pos;
    return crossed;
}

int main(void)
{
    core_init();   /* clock, pads, ADC handle + pads, I2C1 (master) */

    /* VDDA has to be measured BEFORE DMA claims the regular sequence:
     * VREFINT needs a conversion of its own, and doing it under a running
     * DMA hijacks the channel selection. hal_adc_read_vdda_mv caches the
     * result, so every later millivolt conversion reuses it. */
    uint32_t vdda_mv = core_adc_vdd(adc);
    if (adc->vdda_calibrated) pub_status |= ST_VDDA;

    /* The L0 has one sampling time for every channel (RM0377 ADC_SMPR), so
     * the last channel added sets it. coregen adds the pads at MED (7.5
     * cycles, under 0.5 us), and the VDDA measurement above left it at
     * SLOW only as a side effect. Set it on purpose: 160.5 cycles at the
     * 16 MHz HSI16 ADC clock is about 10 us, which is what lets the inputs
     * take 30-50 kOhm sources, and what the ~65 us scan time assumes. */
    for (int i = 0; i < N_CH; i++) {
        core_adc_add(adc, CH_PAD[i], HAL_ADC_SAMP_SLOW);
    }

    for (int i = 0; i < N_CH; i++) {
        ch_slot[i] = core_adc_dma_slot(adc, CH_PAD[i]);
    }

    /* TIM2 emits TRGO on every update. No pin and no channel are
     * consumed, which matters on a tile where all 14 pads are committed.
     * The frequency is set by acq_start. */
    core_timer_init_freq(&pacer, TIM2, 1000);
    core_timer_set_trgo(&pacer, LL_TIM_MMS_UPDATE);

    /* Trigger has to be set before start_dma, which applies it and
     * clears CONT. One scan of all six channels per rising edge. */
    core_adc_set_trigger(adc, LL_ADC_L0_TRG_TIM2_TRGO, ADC_TRIG_RISING);

    /* Stored settings if there are any, otherwise the defaults. */
    acq_cfg_t boot = { RATE_DEFAULT, OVERSAMP_DEFAULT, WINDOW_DEFAULT };
    (void)acq_load(&boot);
    acq_start(&boot);

    /* EEPROM wins if it holds a valid address, otherwise the build-time
     * default. One firmware image serves every board in the array. */
    uint8_t stored = addr_load();
    uint8_t addr   = stored ? stored : (uint8_t)HUB_I2C_ADDR_DEFAULT;

    regs[REG_WHO_AM_I]  = HUB_WHO_AM_I;
    regs[REG_VERSION]   = HUB_VERSION;
    regs[REG_STATUS]    = pub_status;
    regs[REG_VDDA]      = (uint8_t)(vdda_mv & 0xFF);
    regs[REG_VDDA + 1]  = (uint8_t)((vdda_mv >> 8) & 0xFF);
    regs[REG_ADDR_CUR]  = addr;
    regs[REG_ADDR_SET]  = addr;
    regs[REG_COMMIT]    = COMMIT_IDLE;
    regs[REG_APPLY]     = APPLY_IDLE;

    /* Takes I2C1 over from the master handle coregen built. One role per
     * peripheral: core_i2c1 goes unused from here on. */
    core_i2c_target_init_rw(&hub, I2C1, addr, I2C_400K,
                            regs, sizeof(regs),
                            REG_WRITABLE_FIRST, REG_WRITABLE_COUNT,
                            on_i2c_event, NULL);

    for (;;) {
        if (period_elapsed()) {
            /* Publish only once the ring holds nothing but samples taken
             * under the current settings: one full window after a start. */
            if (periods_since_start < acq.window) periods_since_start++;
            if (periods_since_start >= acq.window) publish();
        }

        if (apply_pending) {
            apply_pending = 0;

            uint32_t s = ll_irq_save();
            acq_cfg_t want = { regs[REG_RATE], regs[REG_OVERSAMP],
                               regs[REG_WINDOW] };
            ll_irq_restore(s);

            if (acq_valid(&want) && acq_start(&want) == 0) {
                regs[REG_APPLY] = APPLY_OK;
            } else {
                /* Put the staging registers back to what is running, so
                 * the host can read what it actually got. */
                acq_show();
                regs[REG_APPLY] = APPLY_ERR;
            }
        }

        if (commit_pending) {
            commit_pending = 0;
            uint8_t want = regs[REG_ADDR_SET];
            if (want < ADDR_MIN || want > ADDR_MAX) {
                regs[REG_COMMIT] = COMMIT_ERR_ADDR;
            } else if (addr_store(want) != 0 || acq_store(&acq) != 0) {
                regs[REG_COMMIT] = COMMIT_ERR_NVM;
            } else {
                regs[REG_COMMIT] = COMMIT_OK;
            }
            /* Programming may have stalled the loop across several
             * periods. Re-sync to the ring rather than guessing how many
             * were missed; the next boundary publishes as normal. */
            last_pos = (uint16_t)(ring_len - ll_dma_remaining(DMA1_CH1));
            if (last_pos >= ring_len) last_pos = 0;
        }
    }
}
