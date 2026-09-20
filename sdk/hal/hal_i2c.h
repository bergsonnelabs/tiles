/**
 * hal_i2c.h — I2C HAL driver
 *
 * Master-mode I2C with polling, register-level helpers,
 * and bus scanning. Wraps ll_i2c with ergonomic API.
 */

#ifndef HAL_I2C_H
#define HAL_I2C_H

#include "hal_common.h"
#include "ll_i2c.h"

/* ============================================================
 * Types
 * ============================================================ */

typedef struct hal_i2c {
    I2C_TypeDef *instance;
    uint32_t timeout_ms;    /* Default: 100 */
    uint32_t timing;        /* Saved for bus recovery re-init */
    uint8_t  fmp;           /* Saved for bus recovery re-init */
} hal_i2c_t;

typedef struct {
    uint32_t timing;        /* TIMINGR value (use LL_I2C_TIMING_* defines) */
    uint32_t timeout_ms;    /* 0 = use default (100ms) */
    uint8_t  fmp;           /* Non-zero to enable Fast-mode Plus (CR1.FMP, 20 mA drive) */
} hal_i2c_config_t;

/* ============================================================
 * API declarations (implemented in hal_i2c.c)
 * ============================================================ */

/**
 * Initialize I2C in master mode.
 * The peripheral clock is auto-enabled. SDA/SCL pins must be
 * configured as AF open-drain with pull-up (via tile_init or manually).
 */
hal_status_t hal_i2c_init(hal_i2c_t *h, I2C_TypeDef *instance,
                          const hal_i2c_config_t *cfg);

void hal_i2c_deinit(hal_i2c_t *h);

/**
 * Attempt bus recovery when I2C is stuck (SDA held low).
 * Disables the peripheral, bit-bangs SCL as GPIO to clock out the
 * stuck slave, generates a STOP condition, then re-initializes I2C.
 * Returns HAL_OK if SDA was released, HAL_ERROR if still stuck.
 */
hal_status_t hal_i2c_recover(hal_i2c_t *h);

/* ---- Raw master operations ---- */

/** Write data to a 7-bit address device */
hal_status_t hal_i2c_write(hal_i2c_t *h, uint8_t addr,
                           const uint8_t *data, uint32_t len);

/** Read data from a 7-bit address device */
hal_status_t hal_i2c_read(hal_i2c_t *h, uint8_t addr,
                          uint8_t *buf, uint32_t len);

/* ---- Register-level helpers (most common I2C pattern) ---- */

/**
 * Write to a register on an I2C device.
 * reg is 8-bit or 16-bit: if > 0xFF, two address bytes are sent (MSB first).
 */
hal_status_t hal_i2c_write_reg(hal_i2c_t *h, uint8_t addr,
                               uint16_t reg, const uint8_t *data, uint32_t len);

/**
 * Read from a register on an I2C device.
 * reg is 8-bit or 16-bit: if > 0xFF, two address bytes are sent (MSB first).
 */
hal_status_t hal_i2c_read_reg(hal_i2c_t *h, uint8_t addr,
                              uint16_t reg, uint8_t *buf, uint32_t len);

/* ---- Single-byte convenience ---- */

/** Write a single byte to a register (8-bit or 16-bit reg) */
hal_status_t hal_i2c_write_byte(hal_i2c_t *h, uint8_t addr,
                                uint16_t reg, uint8_t value);

/** Read a single byte from a register (8-bit or 16-bit reg) */
hal_status_t hal_i2c_read_byte(hal_i2c_t *h, uint8_t addr,
                               uint16_t reg, uint8_t *value);

/* ---- Bus utilities ---- */

/** Check if a device responds at addr. Returns HAL_OK or HAL_NACK. */
hal_status_t hal_i2c_probe(hal_i2c_t *h, uint8_t addr);

/**
 * Scan the I2C bus (0x08–0x77). Fills found[] with responding
 * addresses. Returns actual count via *count.
 */
void hal_i2c_scan(hal_i2c_t *h, uint8_t *found, uint8_t *count,
                  uint8_t max_count);

/* ============================================================
 * Target (device) mode — register-file model
 * ============================================================
 *
 * Lets a Core answer as an I2C device on a host's bus, in the shape
 * almost every real sensor uses: the controller writes a register
 * pointer, then reads or writes bytes from that offset with
 * auto-increment.
 *
 * The driver owns the bus protocol; the application owns the bytes.
 * You hand it a flat `regs` buffer and it services the whole transfer
 * from the ISR, so the main loop only ever updates the buffer.
 *
 *   static uint8_t regs[16];
 *   static hal_i2c_target_t tgt;
 *
 *   hal_i2c_target_config_t cfg = {
 *       .addr   = 0x28,
 *       .timing = LL_I2C_TIMING_400K_32MHZ,
 *       .regs   = regs,
 *       .n_regs = sizeof(regs),
 *       .cb     = on_event,
 *   };
 *   hal_i2c_target_init(&tgt, I2C1, &cfg);
 *
 * Coherency is the caller's job, and the READ_START event is how you do
 * it: it fires with SCL still stretched, before the first byte goes out,
 * which is the one safe moment to latch a snapshot into `regs`.
 * ============================================================ */

typedef enum {
    /** Controller addressed us for a read. Fires before the first byte
     *  is transmitted, with SCL stretched. Latch your snapshot here. */
    HAL_I2C_TARGET_READ_START = 0,
    /** Controller wrote a byte to a register inside the writable window.
     *  `reg` is the offset, `value` the byte, already stored in regs[]. */
    HAL_I2C_TARGET_WRITE      = 1,
    /** Transfer ended (STOP on the bus). */
    HAL_I2C_TARGET_STOP       = 2,
} hal_i2c_target_event_t;

/** Called from ISR context. Keep it short and touch only volatile state. */
typedef void (*hal_i2c_target_cb_t)(void *ctx, hal_i2c_target_event_t evt,
                                    uint16_t reg, uint8_t value);

typedef struct hal_i2c_target {
    I2C_TypeDef *instance;
    uint8_t     *regs;
    uint16_t     n_regs;
    uint16_t     writable_first;
    uint16_t     writable_count;
    hal_i2c_target_cb_t cb;
    void        *ctx;
    uint32_t     timing;
    uint8_t      addr;

    volatile uint16_t ptr;        /* current register pointer */
    volatile uint8_t  have_ptr;   /* first byte of a write sets the pointer */

    /* Diagnostics — useful when a host says "it went quiet". */
    volatile uint32_t n_reads;    /* read transactions served */
    volatile uint32_t n_writes;   /* register bytes accepted */
    volatile uint32_t n_errors;   /* BERR / ARLO / OVR seen */
} hal_i2c_target_t;

typedef struct {
    uint8_t   addr;            /* 7-bit own address, unshifted */
    uint32_t  timing;          /* TIMINGR — match the controller's speed */
    uint8_t  *regs;            /* backing register file */
    uint16_t  n_regs;          /* size of regs[] in bytes */
    /* Host writes are accepted only for offsets in
     * [writable_first, writable_first + writable_count). Leave both zero
     * for a read-only map: reads outside any window still work, and
     * writes are absorbed without touching regs[]. */
    uint16_t  writable_first;
    uint16_t  writable_count;
    hal_i2c_target_cb_t cb;    /* optional, may be NULL */
    void     *ctx;             /* passed back to cb */
} hal_i2c_target_config_t;

/**
 * Bring up I2C as a target and unmask its interrupt.
 * The peripheral clock is auto-enabled; SDA/SCL must already be AF
 * open-drain (coregen does this from the I2C pads in config.json).
 *
 * Returns HAL_ERROR if regs/n_regs are unset, the writable window falls
 * outside the register file, or the instance is not one this build
 * installs a vector for.
 */
hal_status_t hal_i2c_target_init(hal_i2c_target_t *t, I2C_TypeDef *instance,
                                 const hal_i2c_target_config_t *cfg);

/** Stop answering, mask the IRQ, and unregister the instance. */
void hal_i2c_target_deinit(hal_i2c_target_t *t);

/**
 * Service one I2C interrupt for this target.
 * The SDK already installs the vectors for every I2C instance it knows,
 * so calling this yourself is only needed with a custom vector table.
 */
void hal_i2c_target_irq(hal_i2c_target_t *t);

#endif /* HAL_I2C_H */
