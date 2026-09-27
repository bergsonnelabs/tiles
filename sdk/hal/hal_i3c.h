/**
 * hal_i3c.h — I3C controller HAL (STM32H523 / Core.ST.H5)
 *
 * SDR controller over the H5 I3C peripheral (RM0481 chapter 49):
 *   - init at a chosen push-pull SCL, with the open-drain phases timed for
 *     the bus's pull-up (TIMINGR0/1 computed from the kernel clock)
 *   - dynamic addressing: RSTDAA, SETDASA (static -> dynamic), ENTDAA
 *   - CCCs: broadcast/direct write, direct read (GETPID/BCR/DCR/STATUS …),
 *     ENEC/DISEC
 *   - private read/write, and register-style write/read for drivers
 *   - in-band interrupts (IBI) with a callback, up to 4 targets
 * Every transfer is polled with a timeout and returns a hal_status_t; the IBI
 * path is interrupt-driven. The pins must already be in their I3C alternate
 * function, push-pull: the controller drives SCL and the push-pull SDA
 * phases, and as controller it enables its own SDA pull-up for the
 * open-drain phases (RM0481 §49.16.3 CRINIT; benched with the GPIO pull-up
 * off too, 2026-09-27).
 *
 * On parts without I3C this header declares nothing (HAL_I3C_AVAILABLE 0).
 */

#ifndef HAL_I3C_H
#define HAL_I3C_H

#include "hal_common.h"
#include "ll_i3c.h"

#define HAL_I3C_AVAILABLE LL_I3C_AVAILABLE

#if LL_I3C_AVAILABLE

/** Targets the controller keeps track of (address map + IBI slots). */
#define HAL_I3C_MAX_TARGETS  8
/** I3C_DEVR1..4: the targets whose IBIs the hardware can acknowledge. */
#define HAL_I3C_IBI_SLOTS    4

/** Kernel clock source (RCC_CCIPR4.I3CxSEL). */
typedef enum {
    HAL_I3C_KERNEL_HSI   = 0,   /**< hsi_ker_ck: 64 MHz HSI / HSIDIV (default) */
    HAL_I3C_KERNEL_PCLK  = 1,   /**< rcc_pclk1 (I3C1) / rcc_pclk3 (I3C2) */
} hal_i3c_kernel_t;

/** Called from the I3C event interrupt for each acknowledged IBI.
 *  `payload` holds the mandatory data byte (MDB) first; `len` is 0 when the
 *  target's BCR[2] says it sends none. */
typedef void (*hal_i3c_ibi_cb_t)(void *ctx, uint8_t dyn_addr,
                                 const uint8_t *payload, uint8_t len);

/** What the controller knows about one target. */
typedef struct {
    uint8_t  static_addr;  /**< I2C static address, 0 if found by ENTDAA */
    uint8_t  dyn_addr;     /**< assigned dynamic address, 0 = free slot */
    uint8_t  bcr;          /**< bus characteristics (ENTDAA / GETBCR), 0xFF = unknown */
    uint8_t  dcr;          /**< device characteristics, 0xFF = unknown */
    uint8_t  pid[6];       /**< provisioned ID, MSB first (ENTDAA / GETPID) */
    uint8_t  ibi_slot;     /**< 1..4 = I3C_DEVRx acknowledging its IBIs, 0 = none */
} hal_i3c_target_t;

typedef struct hal_i3c {
    I3C_TypeDef *instance;
    uint32_t kernel_hz;         /**< I3C kernel clock actually in use */
    uint32_t scl_hz;            /**< achieved push-pull SCL */
    uint32_t od_low_ns;         /**< achieved open-drain SCL low time */
    uint32_t timeout_ms;        /**< per transfer; default 10 */
    hal_i3c_target_t targets[HAL_I3C_MAX_TARGETS];
    hal_i3c_ibi_cb_t ibi_cb;
    void    *ibi_ctx;
    volatile uint32_t ibi_count;   /**< IBIs received */
    volatile uint8_t  ibi_last_addr;
    uint32_t last_ser;          /**< I3C_SER of the last failed transfer (diagnostics) */
    uint16_t last_xdcnt;        /**< I3C_SR.XDCNT of the last transfer */
    uint8_t  last_abt;          /**< last private read ended early by the target */
} hal_i3c_t;

typedef struct {
    uint32_t scl_hz;       /**< push-pull SCL; 0 = 12.5 MHz. Clamped to the kernel clock / 2 */
    uint32_t od_low_ns;    /**< open-drain SCL low time; 0 = 2000 ns. 200 ns is the MIPI
                                minimum; clamped to what SCLL_OD can count (256 kernel
                                cycles, 1.03 µs at 248 MHz) */
    uint8_t  kernel;       /**< hal_i3c_kernel_t */
    uint32_t pclk_hz;      /**< APB clock in Hz, needed when kernel = HAL_I3C_KERNEL_PCLK */
    uint32_t timeout_ms;   /**< per transfer; 0 = 10 ms */
    uint8_t  no_arb_header;/**< 1 = skip the 0x7E header before private transfers (faster,
                                only safe while no target can IBI) */
} hal_i3c_config_t;

/* ---- Lifecycle ---- */

/**
 * Bring an I3C instance up as the bus controller. Enables and resets its
 * clock, selects the kernel clock, programs TIMINGR0/1 and enables it.
 * Returns HAL_ERROR if the kernel clock can't give at least 2x the SCL.
 */
hal_status_t hal_i3c_init(hal_i3c_t *h, I3C_TypeDef *instance, const hal_i3c_config_t *cfg);

/** Disable the peripheral, its interrupts and its clock. */
void hal_i3c_deinit(hal_i3c_t *h);

/* ---- Dynamic addressing ---- */

/** Broadcast RSTDAA: every target forgets its dynamic address. Clears the table. */
hal_status_t hal_i3c_rstdaa(hal_i3c_t *h);

/**
 * Direct SETDASA: give the target at I2C static address `static_addr` the
 * dynamic address `dyn_addr` (0 = pick a free one; never the static address,
 * which a target that loses its dynamic address answers again as an I2C
 * device). A target already in the table keeps its dynamic address (this is
 * how a lost address is re-assigned). Records it in the table.
 * HAL_NACK: no target with that static address answered. HAL_ERROR: no free
 * address, or dyn_addr == static_addr.
 */
hal_status_t hal_i3c_setdasa(hal_i3c_t *h, uint8_t static_addr, uint8_t dyn_addr);

/**
 * Broadcast ENTDAA: assign dynamic addresses to every target that has none,
 * recording each one's PID, BCR and DCR. `found` (may be NULL) gets the number
 * assigned by this call. HAL_NACK: no target answered.
 */
hal_status_t hal_i3c_entdaa(hal_i3c_t *h, uint8_t *found);

/** Table entry of the target at a dynamic address, or NULL. */
hal_i3c_target_t *hal_i3c_target_by_dyn(hal_i3c_t *h, uint8_t dyn_addr);

/** Table entry of the target assigned from a static address (SETDASA), or NULL. */
hal_i3c_target_t *hal_i3c_target_by_static(hal_i3c_t *h, uint8_t static_addr);

/* ---- CCCs ---- */

/** Broadcast CCC (code < 0x80) with `len` data/defining bytes. */
hal_status_t hal_i3c_ccc_broadcast(hal_i3c_t *h, uint8_t ccc, const uint8_t *data, uint16_t len);

/** Direct write CCC (code >= 0x80) to one target, `len` data bytes. */
hal_status_t hal_i3c_ccc_write(hal_i3c_t *h, uint8_t ccc, uint8_t dyn_addr,
                               const uint8_t *data, uint16_t len);

/**
 * Direct read CCC (code >= 0x80) from one target: up to `len` bytes into buf.
 * `got` (may be NULL) gets the count received (a target may send fewer).
 */
hal_status_t hal_i3c_ccc_read(hal_i3c_t *h, uint8_t ccc, uint8_t dyn_addr,
                              uint8_t *buf, uint16_t len, uint16_t *got);

/** GETPID (6 bytes, MSB first). */
hal_status_t hal_i3c_get_pid(hal_i3c_t *h, uint8_t dyn_addr, uint8_t pid[6]);
/** GETBCR. */
hal_status_t hal_i3c_get_bcr(hal_i3c_t *h, uint8_t dyn_addr, uint8_t *bcr);
/** GETDCR. */
hal_status_t hal_i3c_get_dcr(hal_i3c_t *h, uint8_t dyn_addr, uint8_t *dcr);
/** GETSTATUS (format 1, 16-bit status, MSB first on the wire). */
hal_status_t hal_i3c_get_status(hal_i3c_t *h, uint8_t dyn_addr, uint16_t *status);

/** ENEC to one target (dyn_addr), or to all (dyn_addr = 0x7E). events: LL_I3C_EVT_*. */
hal_status_t hal_i3c_enec(hal_i3c_t *h, uint8_t dyn_addr, uint8_t events);
/** DISEC to one target, or to all (0x7E). */
hal_status_t hal_i3c_disec(hal_i3c_t *h, uint8_t dyn_addr, uint8_t events);

/* ---- Private transfers ---- */

/** Private write of `len` bytes (len >= 1). */
hal_status_t hal_i3c_write(hal_i3c_t *h, uint8_t dyn_addr, const uint8_t *data, uint16_t len);

/** Private read of up to `len` bytes (len >= 1). `got` (may be NULL) = bytes received. */
hal_status_t hal_i3c_read(hal_i3c_t *h, uint8_t dyn_addr, uint8_t *buf, uint16_t len, uint16_t *got);

/**
 * Register write: one private write of the register address (1 byte, or 2
 * MSB-first when reg > 0xFF) followed by `len` data bytes.
 */
hal_status_t hal_i3c_write_reg(hal_i3c_t *h, uint8_t dyn_addr, uint16_t reg,
                               const uint8_t *data, uint16_t len);

/**
 * Register read: a private write of the register address, a repeated start,
 * then a private read of `len` bytes, in one frame.
 */
hal_status_t hal_i3c_read_reg(hal_i3c_t *h, uint8_t dyn_addr, uint16_t reg,
                              uint8_t *buf, uint16_t len);

/* ---- In-band interrupts ---- */

/**
 * Acknowledge IBIs from `dyn_addr` and deliver them to `cb` (ISR context).
 * Uses one of the 4 I3C_DEVRx slots; the payload flag comes from the target's
 * BCR[2] (GETBCR if not known yet). Also enables the EV interrupt. Does NOT
 * send ENEC: call hal_i3c_enec() once the target is ready to interrupt.
 */
hal_status_t hal_i3c_ibi_enable(hal_i3c_t *h, uint8_t dyn_addr, hal_i3c_ibi_cb_t cb, void *ctx);

/** Stop acknowledging IBIs from `dyn_addr` (they are NACKed). */
hal_status_t hal_i3c_ibi_disable(hal_i3c_t *h, uint8_t dyn_addr);

/** Service the I3C interrupt (the SDK installs the vectors; only for custom tables). */
void hal_i3c_irq(hal_i3c_t *h);

#endif /* LL_I3C_AVAILABLE */

#endif /* HAL_I3C_H */
