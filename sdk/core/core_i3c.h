/**
 * core_i3c.h — I3C bus (Core.ST.H5 only)
 *
 * The STM32H523 is the only Core with an I3C peripheral (RM0481 chapter 49):
 * I3C1 on pads 4 (SCL, PB8) and 5 (SDA, PB7), I3C2 on pads 2/3. This is the
 * controller side, SDR only:
 *   - dynamic addressing: RSTDAA, SETDASA (static -> dynamic), ENTDAA
 *   - CCCs: GETPID / GETBCR / GETDCR / GETSTATUS, ENEC / DISEC, raw ones
 *   - private read / write, plus register-style helpers
 *   - in-band interrupts (IBI) with a callback, up to 4 targets
 * Transfers are polled, bounded by a timeout, and return I3C_OK / I3C_NACK /
 * I3C_TIMEOUT / I3C_ERROR.
 *
 * coregen brings a bus up when config.json asks for it on an I2C bus whose
 * tiles all speak I3C ("interfaces": {"I2C1": {"i3c": true}}): it emits
 * core_i3c1, resets any dynamic addresses left from a previous image
 * (RSTDAA), and gives each known tile its dynamic address (SETDASA). Tile
 * drivers then run unchanged through core_tiles_pal(&core_i3c1).
 *
 * Manual use:
 *
 *   core_i3c_t bus;
 *   core_i3c_init(&bus, I3C1, I3C_12M5);      // pins already AF3, push-pull
 *   core_i3c_rstdaa(&bus);
 *   uint8_t da = core_i3c_attach(&bus, 0x69); // SETDASA; da = 0x09, the first free one
 *   uint8_t who;
 *   core_i3c_read_reg(&bus, da, 0x75, &who, 1);
 *
 * @studio category i3c label=Core.I3C icon=⇶
 *
 * @studio coverage
 *   id:    i3c
 *   name:  I3C — SDR controller (Core.ST.H5)
 *   blurb: Tier 1 only. Controller-role I3C SDR on the H5's I3C1/I3C2:
 *          dynamic addressing (RSTDAA, SETDASA, ENTDAA), direct/broadcast
 *          CCCs, private read/write and register helpers, IBIs with a
 *          callback. Polled transfers with timeouts; IBIs by interrupt.
 *          Tile drivers use it through core_tiles_pal(&core_i3c1), which
 *          maps each driver's static I2C address to its dynamic address.
 */

#ifndef CORE_I3C_H
#define CORE_I3C_H

#include "hal_i3c.h"
#include "core_config.h"

#if !HAL_I3C_AVAILABLE
#error "core_i3c.h: I3C exists only on Core.ST.H5 (STM32H523)"
#endif

/** Core-level I3C handle. Alias for hal_i3c_t — use this in application code. */
typedef hal_i3c_t core_i3c_t;

/* ---- SCL constants (push-pull; the kernel clock sets what is reachable) ---- */

#define I3C_12M5    12500000UL   /**< SDR maximum (12.4 MHz at a 248 MHz kernel) */
#define I3C_10M     10000000UL
#define I3C_5M       5000000UL
#define I3C_1M       1000000UL

/* ---- Status aliases ---- */

#define I3C_OK       HAL_OK
#define I3C_NACK     HAL_NACK
#define I3C_TIMEOUT  HAL_TIMEOUT
#define I3C_ERROR    HAL_ERROR

/**
 * Bring an I3C instance up as bus controller at `scl_hz` (push-pull SCL).
 *
 * The kernel clock is the APB clock (rcc_pclk1 for I3C1, rcc_pclk3 for I3C2;
 * both run at SYSCLK on the Cores), so the fastest SCL depends on the clock
 * level: 12.4 MHz at "max" (248 MHz), 10.7 MHz at "medium" (64 MHz), 8 MHz at
 * "low" (16 MHz). The open-drain phases get the longest low time the kernel
 * can count, up to 2 µs. The pads must already be in their I3C alternate
 * function, push-pull (coregen does this).
 *
 * @param h         Handle to fill.
 * @param instance  I3C1 or I3C2.
 * @param scl_hz    Push-pull SCL in Hz, e.g. I3C_12M5.
 * @return I3C_OK, or I3C_ERROR for a bad instance.
 */
static inline hal_status_t core_i3c_init(core_i3c_t *h, I3C_TypeDef *instance, uint32_t scl_hz)
{
    hal_i3c_config_t cfg = {
        .scl_hz     = scl_hz,
        .od_low_ns  = 2000,
        .kernel     = HAL_I3C_KERNEL_PCLK,
        .pclk_hz    = (instance == I3C2) ? HCLK_HZ : PCLK1_HZ,
        .timeout_ms = 10,
    };
    return hal_i3c_init(h, instance, &cfg);
}

/** Disable an I3C instance and stop its clock. */
static inline void core_i3c_deinit(core_i3c_t *h)
{
    hal_i3c_deinit(h);
}

/* ---- Dynamic addressing ---- */

/**
 * Broadcast RSTDAA: every target on the bus forgets its dynamic address.
 * Tiles keep their state across a reflash (they stay powered, and
 * flash-serial on the H5 starts the new image without a reset), so run this
 * before assigning addresses.
 */
static inline hal_status_t core_i3c_rstdaa(core_i3c_t *h)
{
    return hal_i3c_rstdaa(h);
}

/**
 * SETDASA: give the target at I2C static address `static_addr` the dynamic
 * address `dyn_addr` (0 = the first free one, 0x09 up). Never its static
 * address: a target that loses its dynamic address answers the static one
 * again as an I2C device, and the controller could not tell.
 *
 * @return I3C_OK, I3C_NACK when no target answers at that static address,
 *         I3C_ERROR when dyn_addr is taken or equals static_addr.
 */
static inline hal_status_t core_i3c_setdasa(core_i3c_t *h, uint8_t static_addr, uint8_t dyn_addr)
{
    return hal_i3c_setdasa(h, static_addr, dyn_addr);
}

/**
 * ENTDAA: assign dynamic addresses to every target that has none, recording
 * each one's PID, BCR and DCR in h->targets[].
 *
 * @param found  Gets the number of targets assigned (may be NULL).
 * @return I3C_OK, or I3C_NACK when no target took part.
 */
static inline hal_status_t core_i3c_entdaa(core_i3c_t *h, uint8_t *found)
{
    return hal_i3c_entdaa(h, found);
}

/**
 * Dynamic address of the target known by its static address, giving it one
 * with SETDASA first if it has none yet.
 *
 * @return The dynamic address, or 0 if the target does not answer.
 */
static inline uint8_t core_i3c_attach(core_i3c_t *h, uint8_t static_addr)
{
    hal_i3c_target_t *t = hal_i3c_target_by_static(h, static_addr);
    if (!t && hal_i3c_setdasa(h, static_addr, 0) == HAL_OK)
        t = hal_i3c_target_by_static(h, static_addr);
    return t ? t->dyn_addr : 0;
}

/* ---- CCCs ---- */

/**
 * GETPID: the target's 48-bit provisioned ID, MSB first
 * (MIPI manufacturer ID in the top 15 bits).
 */
static inline hal_status_t core_i3c_get_pid(core_i3c_t *h, uint8_t dyn_addr, uint8_t pid[6])
{
    return hal_i3c_get_pid(h, dyn_addr, pid);
}

/** GETBCR: bus characteristics (bit 1 IBI capable, bit 2 IBI payload). */
static inline hal_status_t core_i3c_get_bcr(core_i3c_t *h, uint8_t dyn_addr, uint8_t *bcr)
{
    return hal_i3c_get_bcr(h, dyn_addr, bcr);
}

/** GETDCR: device characteristics (MIPI device-type code). */
static inline hal_status_t core_i3c_get_dcr(core_i3c_t *h, uint8_t dyn_addr, uint8_t *dcr)
{
    return hal_i3c_get_dcr(h, dyn_addr, dcr);
}

/** GETSTATUS (format 1): the target's 16-bit status word. */
static inline hal_status_t core_i3c_get_status(core_i3c_t *h, uint8_t dyn_addr, uint16_t *status)
{
    return hal_i3c_get_status(h, dyn_addr, status);
}

/**
 * ENEC: let one target (or all, dyn_addr 0x7E) raise the given events
 * (LL_I3C_EVT_INT for IBIs).
 */
static inline hal_status_t core_i3c_enec(core_i3c_t *h, uint8_t dyn_addr, uint8_t events)
{
    return hal_i3c_enec(h, dyn_addr, events);
}

/** DISEC: stop one target (or all, 0x7E) raising the given events. */
static inline hal_status_t core_i3c_disec(core_i3c_t *h, uint8_t dyn_addr, uint8_t events)
{
    return hal_i3c_disec(h, dyn_addr, events);
}

/* ---- Private transfers ---- */

/** Private write of `len` bytes (len >= 1) to a target. */
static inline hal_status_t core_i3c_write(core_i3c_t *h, uint8_t dyn_addr,
                                          const uint8_t *data, uint16_t len)
{
    return hal_i3c_write(h, dyn_addr, data, len);
}

/** Private read of up to `len` bytes (len >= 1) from a target. */
static inline hal_status_t core_i3c_read(core_i3c_t *h, uint8_t dyn_addr,
                                         uint8_t *buf, uint16_t len)
{
    return hal_i3c_read(h, dyn_addr, buf, len, 0);
}

/**
 * Write `len` bytes starting at register `reg` (8-bit, or 16-bit MSB first
 * when > 0xFF), in one private write.
 */
static inline hal_status_t core_i3c_write_reg(core_i3c_t *h, uint8_t dyn_addr, uint16_t reg,
                                              const uint8_t *data, uint16_t len)
{
    return hal_i3c_write_reg(h, dyn_addr, reg, data, len);
}

/**
 * Read `len` bytes starting at register `reg`: a private write of the
 * register address, a repeated start, a private read, in one frame.
 */
static inline hal_status_t core_i3c_read_reg(core_i3c_t *h, uint8_t dyn_addr, uint16_t reg,
                                             uint8_t *buf, uint16_t len)
{
    return hal_i3c_read_reg(h, dyn_addr, reg, buf, len);
}

/* ---- In-band interrupts ---- */

/**
 * Acknowledge IBIs from a target and call `cb` for each (interrupt context:
 * keep it short). Then enable them on the target with core_i3c_enec(). The
 * target also has to route an event to IBI in its own registers.
 */
static inline hal_status_t core_i3c_ibi_enable(core_i3c_t *h, uint8_t dyn_addr,
                                               hal_i3c_ibi_cb_t cb, void *ctx)
{
    return hal_i3c_ibi_enable(h, dyn_addr, cb, ctx);
}

/** Stop acknowledging IBIs from a target. */
static inline hal_status_t core_i3c_ibi_disable(core_i3c_t *h, uint8_t dyn_addr)
{
    return hal_i3c_ibi_disable(h, dyn_addr);
}

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=1 value=M title="No HDR modes"
//   The H5 I3C peripheral is SDR-only (RM0481 §49.2): no HDR-DDR,
//   HDR-TSL/TSP or HDR-BT, although targets such as the ICM-42686-P offer DDR.
//
// @studio unsupported tier=1 value=M title="No legacy I2C targets on an I3C bus"
//   The peripheral can send legacy I2C messages (MTYPE 0100), but this SDK
//   does not expose them: coregen keeps a bus on I2C unless every tile on it
//   speaks I3C, so a mixed bus never happens.
//
// @studio unsupported tier=1 value=L title="No DMA, hot-join or secondary controller"
//   FIFOs are serviced by polling (the controller stalls SCL for up to
//   ~100 µs while it waits, §49.16.21); hot-join requests are NACKed
//   (HJACK 0) and controller-role requests are not acknowledged.
//
// @studio unsupported tier=2 value=M title="Not in the DSL / Studio yet"
//   Nothing here carries `@studio expose`, and Studio cannot request I3C for
//   a bus yet; a hand-written config.json can ("i3c": true).

#endif /* CORE_I3C_H */
