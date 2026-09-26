/**
 * core_i2c.h — I2C bus communication
 *
 * Provides a simplified I2C API with:
 *   - Speed constants: I2C_100K, I2C_400K, I2C_1M
 *   - Auto-resolved timing from the compile-time kernel clock
 *   - Friendly status aliases: I2C_OK, I2C_NACK, I2C_TIMEOUT
 *
 * Typical usage (coregen pre-creates core_i2c1, core_i2c3 handles):
 *
 *   uint8_t data[] = { 0x42 };
 *   core_i2c_write(&core_i2c1, 0x68, data, 1);
 *
 *   uint8_t val;
 *   core_i2c_read_byte(&core_i2c1, 0x68, 0x75, &val);  // WHO_AM_I
 *
 * For manual init (no coregen):
 *
 *   core_i2c_t bus;
 *   core_i2c_init(&bus, I2C1, I2C_400K);
 *
 * @studio category i2c label=Core.I2C icon=⇄
 *
 * @studio coverage
 *   id:    i2c
 *   name:  I2C — bus communication
 *   page:  /docs/sdk/i2c
 *   blurb: Master-mode I2C: polled read/write, register helpers, probing,
 *          bus scan. Tier 2 exposes byte-level register I/O + probing
 *          against a bus id (1, 2, 3 …) — coregen resolves the handle
 *          via core_i2c_handle_for_bus(). Tier 1 keeps the explicit
 *          handle-based forms for finer control. Implementation wraps
 *          hal_i2c.
 */

#ifndef CORE_I2C_H
#define CORE_I2C_H

#include "hal_i2c.h"
#include "core_config.h"

/** Core-level I2C handle. Alias for hal_i2c_t — use this in application code. */
typedef hal_i2c_t core_i2c_t;

/* ---- Speed constants ---- */

#define I2C_100K    100000UL
#define I2C_400K    400000UL
#define I2C_1M     1000000UL

/* ---- Status aliases ---- */

#define I2C_OK       HAL_OK
#define I2C_NACK     HAL_NACK
#define I2C_TIMEOUT  HAL_TIMEOUT
#define I2C_ERROR    HAL_ERROR

/* ---- Auto-resolving init ---- */

/**
 * Resolve the TIMINGR value for a given speed using the compile-time
 * I2C kernel clock (I2C_KERNEL_CLK_MHZ from core_config.h).
 * Returns 0 if no pre-computed value exists.
 */
static inline uint32_t _core_i2c_timing(uint32_t speed_hz)
{
    switch (speed_hz) {
        case I2C_100K: return ll_i2c_timing_100k(I2C_KERNEL_CLK_MHZ);
        case I2C_400K: return ll_i2c_timing_400k(I2C_KERNEL_CLK_MHZ);
        case I2C_1M:   return ll_i2c_timing_1m(I2C_KERNEL_CLK_MHZ);
        default:       return 0;
    }
}

/**
 * Initialize an I2C bus with automatic timing resolution.
 *
 *   core_i2c_t bus;
 *   core_i2c_init(&bus, I2C1, I2C_400K);
 *
 * Resolves the TIMINGR value from the compile-time kernel clock.
 * Enables FMP mode automatically when speed is I2C_1M.
 */
static inline hal_status_t core_i2c_init(core_i2c_t *h,
                                          I2C_TypeDef *instance,
                                          uint32_t speed_hz)
{
    uint32_t timing = _core_i2c_timing(speed_hz);
    if (!timing) return HAL_ERROR;
    hal_i2c_config_t cfg = {
        .timing = timing,
        .timeout_ms = 100,
        .fmp = (speed_hz >= I2C_1M) ? 1 : 0,
    };
    return hal_i2c_init(h, instance, &cfg);
}

/**
 * Initialize I2C with explicit config struct (advanced).
 * For most use cases, prefer core_i2c_init(h, instance, speed_hz).
 */
static inline hal_status_t core_i2c_init_cfg(core_i2c_t *h,
                                              I2C_TypeDef *instance,
                                              const hal_i2c_config_t *cfg)
{
    return hal_i2c_init(h, instance, cfg);
}

/** @deprecated Use core_i2c_init(). */
#define core_i2c_setup core_i2c_init

/* ---- Master operations ---- */

/** Write data to a 7-bit address device. */
static inline hal_status_t core_i2c_write(core_i2c_t *h, uint8_t addr,
                                           const uint8_t *data, uint32_t len)
{
    return hal_i2c_write(h, addr, data, len);
}

/** Read data from a 7-bit address device. */
static inline hal_status_t core_i2c_read(core_i2c_t *h, uint8_t addr,
                                          uint8_t *buf, uint32_t len)
{
    return hal_i2c_read(h, addr, buf, len);
}

/* ---- Register helpers ---- */

/** Write to a device register (8 or 16-bit register address). */
static inline hal_status_t core_i2c_write_reg(core_i2c_t *h, uint8_t addr,
                                               uint16_t reg,
                                               const uint8_t *data,
                                               uint32_t len)
{
    return hal_i2c_write_reg(h, addr, reg, data, len);
}

/** Read from a device register (repeated START). */
static inline hal_status_t core_i2c_read_reg(core_i2c_t *h, uint8_t addr,
                                              uint16_t reg, uint8_t *buf,
                                              uint32_t len)
{
    return hal_i2c_read_reg(h, addr, reg, buf, len);
}

/** Write a single byte to a register. */
static inline hal_status_t core_i2c_write_byte(core_i2c_t *h, uint8_t addr,
                                                uint16_t reg, uint8_t value)
{
    return hal_i2c_write_byte(h, addr, reg, value);
}

/** Read a single byte from a register. */
static inline hal_status_t core_i2c_read_byte(core_i2c_t *h, uint8_t addr,
                                               uint16_t reg, uint8_t *value)
{
    return hal_i2c_read_byte(h, addr, reg, value);
}

/* ---- Bus utilities ---- */

/** Check if a device responds at addr. Returns I2C_OK or I2C_NACK. */
static inline hal_status_t core_i2c_probe(core_i2c_t *h, uint8_t addr)
{
    return hal_i2c_probe(h, addr);
}

/** Scan the I2C bus (0x08–0x77). Fills found[] with responding addresses. */
static inline void core_i2c_scan(core_i2c_t *h, uint8_t *found,
                                  uint8_t *count, uint8_t max_count)
{
    hal_i2c_scan(h, found, count, max_count);
}

/* ---- Tier 2 — default-instance bus helpers ---------------------------- */

/* These wrappers take a bus id (the I2C peripheral number — 1, 2, 3 …
 * matching how config.json declares it) instead of a handle. The
 * dispatcher `core_i2c_handle_for_bus` is emitted by coregen alongside
 * the per-bus extern handles (core_i2c1, core_i2c3 …) so the wrapper
 * never needs to see the hal_i2c_t shape itself. DSL programs reach
 * for these; escape-to-C drops back to the Tier 1 handle-based forms
 * above when finer control is needed. */

/**
 * The I2C handle for a bus id declared in config.json, or NULL if the
 * project doesn't declare that bus. Emitted per project by coregen into
 * core_init.c (when the project declares any I2C bus); the *_bus helpers below
 * resolve their handle through it.
 *
 * Forward-declared here rather than `#include "core_init.h"` so this header
 * compiles without a project (val tests, examples without config.json). The
 * natives-side caller is gated on CORE_HAS_I2C_BUSES, so the linker never asks for
 * the symbol unless the dispatcher exists.
 *
 * @param bus Bus id (1 = the first instance, ...).
 * @return The coregen-initialized handle, or NULL.
 */
hal_i2c_t *core_i2c_handle_for_bus(uint8_t bus);

/**
 * Write a single byte to a register on a device on `bus`.
 * Returns I2C_OK on success, I2C_NACK / I2C_ERROR on bus failure,
 * or I2C_ERROR if `bus` isn't declared in config.json.
 *
 * @studio expose category=i2c name=write_byte returns=int
 * @studio twin full
 * @param bus I2C bus id as declared in config.json (1 = I2C1, ...).
 * @param addr 7-bit device address.
 * @param reg Register address (two bytes, MSB first, when above 0xFF).
 * @param value Byte to write.
 */
static inline hal_status_t core_i2c_write_byte_bus(uint8_t bus, uint8_t addr,
                                                    uint16_t reg, uint8_t value)
{
    hal_i2c_t *h = core_i2c_handle_for_bus(bus);
    if (!h) return HAL_ERROR;
    return hal_i2c_write_byte(h, addr, reg, value);
}

/**
 * Read a single byte from a register on a device on `bus`.
 * Returns the byte value (0..255) on success, or -1 on any error
 * (bus undeclared, NACK, timeout). The signed return lets DSL
 * programs branch on `< 0` without an out-pointer.
 *
 * @studio expose category=i2c name=read_byte returns=int
 * @studio twin full
 * @param bus I2C bus id as declared in config.json (1 = I2C1, ...).
 * @param addr 7-bit device address.
 * @param reg Register address (two bytes, MSB first, when above 0xFF).
 */
static inline int core_i2c_read_byte_bus(uint8_t bus, uint8_t addr, uint16_t reg)
{
    hal_i2c_t *h = core_i2c_handle_for_bus(bus);
    if (!h) return -1;
    uint8_t v = 0;
    if (hal_i2c_read_byte(h, addr, reg, &v) != HAL_OK) return -1;
    return (int)v;
}

/**
 * Check if a device responds at `addr` on `bus`. Returns 1 if the
 * device ACKs, 0 on NACK / timeout / undeclared bus.
 *
 * @studio expose category=i2c name=probe returns=bool
 * @studio twin full
 * @param bus I2C bus id as declared in config.json (1 = I2C1, ...).
 * @param addr 7-bit device address to probe.
 */
static inline int core_i2c_probe_bus(uint8_t bus, uint8_t addr)
{
    hal_i2c_t *h = core_i2c_handle_for_bus(bus);
    if (!h) return 0;
    return hal_i2c_probe(h, addr) == HAL_OK ? 1 : 0;
}

/* ============================================================
 * Target (device) mode
 * ============================================================
 *
 * Answer as a device on someone else's bus, instead of driving your own.
 * The register-file model matches how a host expects to talk to a
 * sensor: write a register pointer, then read or write from there with
 * auto-increment.
 *
 *   static uint8_t regs[16];
 *   static core_i2c_target_t tgt;
 *
 *   static void on_evt(void *ctx, hal_i2c_target_event_t e,
 *                      uint16_t reg, uint8_t val)
 *   {
 *       if (e == HAL_I2C_TARGET_READ_START) latch_snapshot();
 *   }
 *
 *   core_i2c_target_init(&tgt, I2C1, 0x28, I2C_400K, regs, sizeof(regs),
 *                        on_evt, NULL);
 *
 * The pads still come from config.json (I2C1.CLK / I2C1.DAT); coregen
 * sets them up as AF open-drain and this re-initializes the peripheral
 * in target mode, so the coregen-emitted master handle for the same bus
 * goes unused. One role per peripheral: a Core cannot be controller and
 * target on the same I2C at once.
 *
 * C-level only. The callback and register buffer have no DSL
 * representation, so there is no Tier 2 / Studio surface for this yet.
 */

/** Core-level I2C target handle. Alias for hal_i2c_target_t. */
typedef hal_i2c_target_t core_i2c_target_t;

/** Event codes and callback signature come straight from the HAL. */
#define I2C_TARGET_READ_START  HAL_I2C_TARGET_READ_START
#define I2C_TARGET_WRITE       HAL_I2C_TARGET_WRITE
#define I2C_TARGET_STOP        HAL_I2C_TARGET_STOP

/**
 * Bring up an I2C peripheral as a read-only target at `addr`.
 *
 * @param addr   7-bit own address, unshifted
 * @param speed  I2C_100K / I2C_400K / I2C_1M — match the controller
 * @param regs   register file the host reads from
 * @param n_regs size of regs[] in bytes
 * @param cb     optional; NULL for none
 */
static inline hal_status_t core_i2c_target_init(core_i2c_target_t *t,
                                                I2C_TypeDef *instance,
                                                uint8_t addr, uint32_t speed,
                                                uint8_t *regs, uint16_t n_regs,
                                                hal_i2c_target_cb_t cb,
                                                void *ctx)
{
    uint32_t timing = _core_i2c_timing(speed);
    if (!timing) return HAL_ERROR;
    hal_i2c_target_config_t cfg = {
        .addr           = addr,
        .timing         = timing,
        .regs           = regs,
        .n_regs         = n_regs,
        .writable_first = 0,
        .writable_count = 0,
        .cb             = cb,
        .ctx            = ctx,
    };
    return hal_i2c_target_init(t, instance, &cfg);
}

/**
 * As core_i2c_target_init, but accepts host writes to the register
 * range [first, first + count). Everything outside stays read-only.
 */
static inline hal_status_t core_i2c_target_init_rw(core_i2c_target_t *t,
                                                   I2C_TypeDef *instance,
                                                   uint8_t addr, uint32_t speed,
                                                   uint8_t *regs, uint16_t n_regs,
                                                   uint16_t first, uint16_t count,
                                                   hal_i2c_target_cb_t cb,
                                                   void *ctx)
{
    uint32_t timing = _core_i2c_timing(speed);
    if (!timing) return HAL_ERROR;
    hal_i2c_target_config_t cfg = {
        .addr           = addr,
        .timing         = timing,
        .regs           = regs,
        .n_regs         = n_regs,
        .writable_first = first,
        .writable_count = count,
        .cb             = cb,
        .ctx            = ctx,
    };
    return hal_i2c_target_init(t, instance, &cfg);
}

/** Stop answering on the bus. */
static inline void core_i2c_target_deinit(core_i2c_target_t *t)
{
    hal_i2c_target_deinit(t);
}

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=M title="Tier 2 is byte-level only — no bulk / scan"
//   The current Tier 2 surface is write_byte_bus / read_byte_bus /
//   probe_bus. DSL programs that need to push a multi-byte payload
//   (display init sequences, IMU FIFO drains) or discover what's on a
//   bus (`scan -> int[]`) drop back to Tier 1 with a hal_i2c_t handle.
//   Adds need the array-IN / array-OUT host-call ABI prototyped on the
//   tile-driver side — track with the DSL Capability Coverage close.
//
// @studio unsupported tier=1 value=H title="Interrupt-driven / non-blocking I/O"
//   All current operations are polled. A non-blocking variant (with
//   completion callback or status poll) would let DSL programs interleave
//   bus traffic without stalling the main loop. Tracked on the SDK
//   roadmap as 'I2C interrupt-driven'.
//
// @studio unsupported tier=1 value=H title="DMA transfers"
//   No DMA path for bulk reads/writes. Long FIFO drains (e.g., IMU
//   water-level batch reads) currently block on polled byte loops.
//
// @studio unsupported tier=2 value=M title="Target mode is C-only"
//   Target (device) mode landed at Tier 1: core_i2c_target_init() lets a
//   Core answer as an I2C device on a host bus, using a register-file
//   model with an optional ISR callback. There is no Tier 2 / DSL
//   surface, because a register buffer and a callback have no
//   representation in the DSL host-call ABI. A Studio-level version
//   would need a declarative "expose these variables as registers"
//   binding, close to what the BLE characteristic binding does.
//
// @studio unsupported tier=1 value=L title="Target mode on H5 I2C2 / I2C3"
//   startup_stm32h523xx.s carries vectors for I2C1_EV, I2C1_ER and
//   I2C2_EV only. There is no I2C2_ER or I2C3 entry, and slot 58 is
//   labelled as I2C2 Error shared with USART1, so target mode is
//   restricted to I2C1 on Core.ST.H5. I2C1 + I2C3 both work on
//   Core.ST.L4 and Core.ST.W5; Core.ST.L0 has only I2C1.
//
// @studio unsupported tier=1 value=M title="10-bit addressing"
//   API takes uint8_t addr — 7-bit only. No path for 10-bit-addressed
//   peripherals (rare in practice but in spec).
//
// @studio unsupported tier=1 value=L title="SMBus / PMBus extensions"
//   No PEC, no Alert Response, no block read/write protocol framing.
//   Not requested by any current tile.

#endif /* CORE_I2C_H */
