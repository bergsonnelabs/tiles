/**
 * hal_i2c.c — I2C HAL driver implementation
 */

#include "hal_i2c.h"
#include "ll_rcc.h"
#include "ll_gpio.h"
#include <string.h>

/* ---- Peripheral clock enable ---- */

static void _i2c_clk_enable(I2C_TypeDef *instance)
{
    if (instance == I2C1) ll_rcc_apb1_clk_enable(LL_APB1_I2C1);
#if defined(STM32L422xx)
    if (instance == I2C3) ll_rcc_apb1_clk_enable(LL_APB1_I2C3);
#elif defined(STM32WBA55xx)
    if (instance == I2C3) ll_rcc_apb7_clk_enable(LL_APB7_I2C3);
#elif defined(STM32H523xx)
    if (instance == I2C2) ll_rcc_apb1_clk_enable(LL_APB1_I2C2);
    if (instance == I2C3) ll_rcc_apb3_clk_enable(LL_APB3_I2C3);
#endif
    (void)REG32(RCC_BASE);
}

/* ============================================================
 * Init / Deinit
 * ============================================================ */

hal_status_t hal_i2c_init(hal_i2c_t *h, I2C_TypeDef *instance,
                          const hal_i2c_config_t *cfg)
{
    memset(h, 0, sizeof(*h));
    h->instance = instance;
    h->timeout_ms = cfg->timeout_ms ? cfg->timeout_ms : 100;
    h->timing = cfg->timing;
    h->fmp = cfg->fmp;

    _i2c_clk_enable(instance);
    if (cfg->fmp) {
        ll_i2c_init_fmp(instance, cfg->timing);
    } else {
        ll_i2c_init(instance, cfg->timing);
    }
    return HAL_OK;
}

void hal_i2c_deinit(hal_i2c_t *h)
{
    if (h && h->instance) {
        ll_i2c_disable(h->instance);
        h->instance = NULL;
    }
}

/* ============================================================
 * Status conversion
 * ============================================================ */

static hal_status_t _convert_status(int ll_result)
{
    switch (ll_result) {
    case LL_I2C_OK:      return HAL_OK;
    case LL_I2C_NACK:    return HAL_NACK;
    case LL_I2C_TIMEOUT: return HAL_TIMEOUT;
    default:             return HAL_ERROR;
    }
}

/* ============================================================
 * Auto-recovery wrapper — retry once after bus recovery
 * ============================================================ */

static hal_status_t _with_recovery(hal_i2c_t *h, int ll_result)
{
    hal_status_t s = _convert_status(ll_result);
    if (s == HAL_TIMEOUT || s == HAL_ERROR) {
        hal_i2c_recover(h);
    }
    return s;
}

/* ============================================================
 * Raw master operations
 * ============================================================ */

hal_status_t hal_i2c_write(hal_i2c_t *h, uint8_t addr,
                           const uint8_t *data, uint32_t len)
{
    return _with_recovery(h, ll_i2c_write(h->instance, addr, data, len));
}

hal_status_t hal_i2c_read(hal_i2c_t *h, uint8_t addr,
                          uint8_t *buf, uint32_t len)
{
    return _with_recovery(h, ll_i2c_read(h->instance, addr, buf, len));
}

/* ============================================================
 * Register-level helpers
 * ============================================================ */

hal_status_t hal_i2c_write_reg(hal_i2c_t *h, uint8_t addr,
                               uint16_t reg, const uint8_t *data, uint32_t len)
{
    return _with_recovery(h, ll_i2c_write_reg(h->instance, addr, reg, data, len));
}

hal_status_t hal_i2c_read_reg(hal_i2c_t *h, uint8_t addr,
                              uint16_t reg, uint8_t *buf, uint32_t len)
{
    return _with_recovery(h, ll_i2c_read_reg(h->instance, addr, reg, buf, len));
}

hal_status_t hal_i2c_write_byte(hal_i2c_t *h, uint8_t addr,
                                uint16_t reg, uint8_t value)
{
    return hal_i2c_write_reg(h, addr, reg, &value, 1);
}

hal_status_t hal_i2c_read_byte(hal_i2c_t *h, uint8_t addr,
                               uint16_t reg, uint8_t *value)
{
    return hal_i2c_read_reg(h, addr, reg, value, 1);
}

/* ============================================================
 * Bus recovery — clock out a stuck slave
 * ============================================================ */

/* Map I2C instance → SCL/SDA GPIO port+pin.
 * These are fixed per tile family (set by coregen core_pads_init). */
typedef struct {
    GPIO_TypeDef *scl_port; uint8_t scl_pin; uint8_t scl_af;
    GPIO_TypeDef *sda_port; uint8_t sda_pin; uint8_t sda_af;
} _i2c_pins_t;

static _i2c_pins_t _i2c_pins(I2C_TypeDef *instance)
{
    _i2c_pins_t p = {0};
#if defined(STM32L422xx)
    if (instance == I2C1) {
        p = (const _i2c_pins_t){ GPIOB, 6, 4, GPIOB, 7, 4 };
    } else if (instance == I2C3) {
        p = (const _i2c_pins_t){ GPIOA, 7, 4, GPIOB, 4, 4 };
    }
#elif defined(STM32WBA55xx)
    if (instance == I2C1) {
        p = (const _i2c_pins_t){ GPIOB, 2, 4, GPIOB, 1, 4 };
    } else if (instance == I2C3) {
        p = (const _i2c_pins_t){ GPIOA, 6, 4, GPIOA, 7, 4 };
    }
#elif defined(STM32H523xx)
    if (instance == I2C1) {
        p = (const _i2c_pins_t){ GPIOB, 6, 4, GPIOB, 7, 4 };
    } else if (instance == I2C2) {
        p = (const _i2c_pins_t){ GPIOB, 10, 4, GPIOB, 11, 4 };
    }
#elif defined(STM32L011xx)
    if (instance == I2C1) {
        p = (const _i2c_pins_t){ GPIOA, 9, 1, GPIOA, 10, 1 };
    }
#endif
    return p;
}

static void _i2c_delay(void)
{
    for (volatile int i = 0; i < 50; i++) ;
}

hal_status_t hal_i2c_recover(hal_i2c_t *h)
{
    _i2c_pins_t pins = _i2c_pins(h->instance);
    if (!pins.scl_port) return HAL_ERROR;  /* unknown instance */

    /* 1. Disable I2C peripheral */
    h->instance->CR1 &= ~LL_I2C_CR1_PE;

    /* 2. Switch SCL to push-pull output (high), SDA to input */
    ll_gpio_config_output(pins.scl_port, pins.scl_pin);
    pins.scl_port->BSRR = (1UL << pins.scl_pin);  /* SCL high */

    ll_gpio_config_input(pins.sda_port, pins.sda_pin, LL_GPIO_PULL_UP);

    _i2c_delay();

    /* 3. Clock SCL up to 16 times until SDA goes high */
    uint8_t recovered = 0;
    for (int i = 0; i < 16; i++) {
        if (pins.sda_port->IDR & (1UL << pins.sda_pin)) {
            recovered = 1;
            break;
        }
        pins.scl_port->BSRR = (1UL << (pins.scl_pin + 16));  /* SCL low */
        _i2c_delay();
        pins.scl_port->BSRR = (1UL << pins.scl_pin);          /* SCL high */
        _i2c_delay();
    }

    /* 4. Generate STOP: SDA low, then SDA high while SCL is high */
    /* Switch SDA to output temporarily */
    ll_gpio_config_output(pins.sda_port, pins.sda_pin);
    ll_gpio_set_output_type(pins.sda_port, pins.sda_pin, LL_GPIO_OTYPE_OD);

    pins.sda_port->BSRR = (1UL << (pins.sda_pin + 16));  /* SDA low */
    _i2c_delay();
    pins.sda_port->BSRR = (1UL << pins.sda_pin);          /* SDA high */
    _i2c_delay();

    /* 5. Restore pins to AF open-drain and re-init I2C */
    ll_gpio_config_af(pins.scl_port, pins.scl_pin, pins.scl_af,
                      LL_GPIO_OTYPE_OD, LL_GPIO_SPEED_HIGH, LL_GPIO_PULL_UP);
    ll_gpio_config_af(pins.sda_port, pins.sda_pin, pins.sda_af,
                      LL_GPIO_OTYPE_OD, LL_GPIO_SPEED_HIGH, LL_GPIO_PULL_UP);

    if (h->fmp)
        ll_i2c_init_fmp(h->instance, h->timing);
    else
        ll_i2c_init(h->instance, h->timing);

    return recovered ? HAL_OK : HAL_ERROR;
}

/* ============================================================
 * Bus utilities
 * ============================================================ */

hal_status_t hal_i2c_probe(hal_i2c_t *h, uint8_t addr)
{
    return _convert_status(ll_i2c_probe(h->instance, addr));
}

void hal_i2c_scan(hal_i2c_t *h, uint8_t *found, uint8_t *count,
                  uint8_t max_count)
{
    *count = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (hal_i2c_probe(h, addr) == HAL_OK) {
            if (*count < max_count) {
                found[*count] = addr;
            }
            (*count)++;
        }
    }
}

/* ============================================================
 * Target (device) mode
 * ============================================================
 *
 * One registered target per I2C peripheral. The vectors below are
 * strong definitions that override the weak ones in the startup file,
 * matching how hal_exti.c and hal_adc.c install theirs. They cost a
 * null check when no target is registered.
 * ============================================================ */

/* Slot 0 = I2C1 everywhere. Slot 1 = I2C3 on L4/WBA.
 * H5 I2C2/I2C3 are deliberately absent: startup_stm32h523xx.s has no
 * I2C2_ER or I2C3 vectors at all, and its slot 58 comment marks I2C2
 * Error as shared with USART1. Registering a target there would route
 * bus errors into the wrong handler. */
#define _HAL_I2C_TGT_SLOTS 2

static hal_i2c_target_t *_i2c_tgt[_HAL_I2C_TGT_SLOTS];

static int _tgt_slot(I2C_TypeDef *instance)
{
    if (instance == I2C1) return 0;
#if defined(STM32L422xx) || defined(STM32WBA55xx)
    if (instance == I2C3) return 1;
#endif
    return -1;
}

static void _tgt_nvic(int slot, int enable)
{
    switch (slot) {
    case 0:
#if defined(STM32L011xx)
        /* L0 folds event and error onto one vector. */
        if (enable) {
            hal_nvic_set_priority(HAL_IRQ_I2C1_EV, 2);
            hal_nvic_enable_irq(HAL_IRQ_I2C1_EV);
        } else {
            hal_nvic_disable_irq(HAL_IRQ_I2C1_EV);
        }
#else
        if (enable) {
            hal_nvic_set_priority(HAL_IRQ_I2C1_EV, 2);
            hal_nvic_set_priority(HAL_IRQ_I2C1_ER, 2);
            hal_nvic_enable_irq(HAL_IRQ_I2C1_EV);
            hal_nvic_enable_irq(HAL_IRQ_I2C1_ER);
        } else {
            hal_nvic_disable_irq(HAL_IRQ_I2C1_EV);
            hal_nvic_disable_irq(HAL_IRQ_I2C1_ER);
        }
#endif
        break;
#if defined(STM32L422xx) || defined(STM32WBA55xx)
    case 1:
        if (enable) {
            hal_nvic_set_priority(HAL_IRQ_I2C3_EV, 2);
            hal_nvic_set_priority(HAL_IRQ_I2C3_ER, 2);
            hal_nvic_enable_irq(HAL_IRQ_I2C3_EV);
            hal_nvic_enable_irq(HAL_IRQ_I2C3_ER);
        } else {
            hal_nvic_disable_irq(HAL_IRQ_I2C3_EV);
            hal_nvic_disable_irq(HAL_IRQ_I2C3_ER);
        }
        break;
#endif
    default:
        break;
    }
}

hal_status_t hal_i2c_target_init(hal_i2c_target_t *t, I2C_TypeDef *instance,
                                 const hal_i2c_target_config_t *cfg)
{
    if (!t || !cfg || !cfg->regs || cfg->n_regs == 0) return HAL_ERROR;

    int slot = _tgt_slot(instance);
    if (slot < 0) return HAL_ERROR;

    /* A writable window has to sit inside the register file, or a host
     * write would run off the end of the buffer. */
    if (cfg->writable_count > cfg->n_regs ||
        cfg->writable_first > (uint16_t)(cfg->n_regs - cfg->writable_count)) {
        return HAL_ERROR;
    }

    memset(t, 0, sizeof(*t));
    t->instance       = instance;
    t->regs           = cfg->regs;
    t->n_regs         = cfg->n_regs;
    t->writable_first = cfg->writable_first;
    t->writable_count = cfg->writable_count;
    t->cb             = cfg->cb;
    t->ctx            = cfg->ctx;
    t->timing         = cfg->timing;
    t->addr           = cfg->addr;

    _i2c_clk_enable(instance);
    ll_i2c_target_init(instance, cfg->timing, cfg->addr);

    _i2c_tgt[slot] = t;
    _tgt_nvic(slot, 1);
    return HAL_OK;
}

void hal_i2c_target_deinit(hal_i2c_target_t *t)
{
    if (!t || !t->instance) return;

    int slot = _tgt_slot(t->instance);
    if (slot >= 0) {
        _tgt_nvic(slot, 0);
        _i2c_tgt[slot] = NULL;
    }
    ll_i2c_target_disable(t->instance);
    t->instance = NULL;
}

void hal_i2c_target_irq(hal_i2c_target_t *t)
{
    I2C_TypeDef *i2c = t->instance;
    uint32_t isr = i2c->ISR;

    if (isr & LL_I2C_ISR_ADDR) {
        if (isr & LL_I2C_ISR_DIR) {
            /* Controller will read from us. Drop any byte a previous,
             * abandoned read left loaded, otherwise it becomes the first
             * byte of this one and shifts the whole transfer. */
            ll_i2c_target_flush_tx(i2c);
            t->n_reads++;
            /* SCL is still stretched here: the one safe moment for the
             * application to latch a coherent snapshot into regs[]. */
            if (t->cb) t->cb(t->ctx, HAL_I2C_TARGET_READ_START, t->ptr, 0);
        } else {
            /* Controller will write. First byte is the register pointer. */
            t->have_ptr = 0;
        }
        ll_i2c_target_clear_addr(i2c);   /* releases SCL */
    }

    if (isr & LL_I2C_ISR_RXNE) {
        uint8_t b = (uint8_t)i2c->RXDR;
        if (!t->have_ptr) {
            t->ptr = b;
            t->have_ptr = 1;
        } else {
            uint16_t p = t->ptr;
            uint16_t wlast = (uint16_t)(t->writable_first + t->writable_count);
            if (t->writable_count && p >= t->writable_first && p < wlast) {
                t->regs[p] = b;
                t->n_writes++;
                if (t->cb) t->cb(t->ctx, HAL_I2C_TARGET_WRITE, p, b);
            }
            /* Absorb writes outside the window: ACKing and discarding
             * keeps a mis-addressed host from wedging the bus. */
            if ((uint16_t)(p + 1) < t->n_regs) t->ptr = (uint16_t)(p + 1);
        }
    }

    if (isr & LL_I2C_ISR_TXIS) {
        uint16_t p = t->ptr;
        /* Reads past the end return 0xFF rather than stalling, so a host
         * that asks for too many bytes gets an obvious answer. */
        i2c->TXDR = (p < t->n_regs) ? t->regs[p] : 0xFFU;
        if ((uint16_t)(p + 1) < t->n_regs) t->ptr = (uint16_t)(p + 1);
    }

    if (isr & LL_I2C_ISR_NACKF) {
        /* Expected: a controller NACKs the last byte it reads. */
        i2c->ICR = LL_I2C_ICR_NACKCF;
    }

    if (isr & LL_I2C_ISR_STOPF) {
        i2c->ICR = LL_I2C_ICR_STOPCF;
        ll_i2c_target_flush_tx(i2c);
        t->have_ptr = 0;
        if (t->cb) t->cb(t->ctx, HAL_I2C_TARGET_STOP, t->ptr, 0);
    }

    if (isr & (LL_I2C_ISR_BERR | LL_I2C_ISR_ARLO | LL_I2C_ISR_OVR)) {
        i2c->ICR = LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF;
        t->n_errors++;
    }
}

/* ---- Vectors ---- */

static void _tgt_dispatch(int slot)
{
    hal_i2c_target_t *t = _i2c_tgt[slot];
    if (t && t->instance) hal_i2c_target_irq(t);
}

#if defined(STM32L011xx)
void I2C1_IRQHandler(void)    { _tgt_dispatch(0); }
#else
void I2C1_EV_IRQHandler(void) { _tgt_dispatch(0); }
void I2C1_ER_IRQHandler(void) { _tgt_dispatch(0); }
#endif

#if defined(STM32L422xx) || defined(STM32WBA55xx)
void I2C3_EV_IRQHandler(void) { _tgt_dispatch(1); }
void I2C3_ER_IRQHandler(void) { _tgt_dispatch(1); }
#endif
