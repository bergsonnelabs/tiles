/**
 * hal_spi.c — SPI HAL driver implementation
 *
 * Polled transfers are ll_spi_xfer_poll (bounded, FIFO-pipelined). DMA runs
 * on one channel pair per Core, shared by every handle:
 *   L4:  DMA1 CH2 = SPI1_RX, CH3 = SPI1_TX (request 1, RM0394 Table 45)
 *   WBA: GPDMA1 CH6 = RX, CH7 = TX (REQSEL spi1_rx 1 / spi1_tx 2 /
 *        spi3_rx 3 / spi3_tx 4, RM0493 Table 125)
 *   H5:  no DMA yet (HAL_ERROR)
 * The ADC uses DMA1 CH1 (L4) / GPDMA1 CH0 (WBA); nothing else in the SDK
 * claims these channels.
 */

#include "hal_spi.h"
#include "ll_rcc.h"
#include "ll_dma.h"
#include <string.h>

#if defined(STM32L422xx) || defined(STM32WBA55xx)
#define _SPI_HAS_DMA 1
#endif

/* The handle whose DMA transfer is running (one at a time: shared channels). */
static hal_spi_t *volatile _spi_dma_owner;

/* ---- Small critical section (PRIMASK) ---- */

static inline uint32_t _spi_irq_save(void)
{
    uint32_t pm;
    __asm volatile ("mrs %0, primask" : "=r"(pm));
    __asm volatile ("cpsid i" ::: "memory");
    return pm;
}

static inline void _spi_irq_restore(uint32_t pm)
{
    __asm volatile ("msr primask, %0" :: "r"(pm) : "memory");
}

/* ---- Peripheral clock enable ---- */

static void _spi_clk_enable(SPI_TypeDef *instance)
{
    if (instance == SPI1) ll_rcc_apb2_clk_enable(LL_APB2_SPI1);
#if defined(STM32WBA55xx)
    if (instance == SPI3) ll_rcc_apb7_clk_enable(LL_APB7_SPI3);
#elif defined(STM32H523xx)
    if (instance == SPI2) ll_rcc_apb1_clk_enable((1UL << 14));
    if (instance == SPI3) SET_BITS(REG32(RCC_BASE + 0xA8UL), LL_APB3_SPI3);
#endif
    (void)REG32(RCC_BASE);
}

/* ---- Peripheral reset: FIFOs, flags and state machine back to reset ----
 * On the L4, SPE=0 does not flush the FIFOs (RM0394 §41.4.9 names the RCC
 * reset as the way to start clean), so recovery after a timeout pulses it. */

static void _spi_reset(SPI_TypeDef *instance)
{
#if defined(STM32L422xx)
    if (instance == SPI1) {                       /* RCC_APB2RSTR.SPI1RST, RM0394 §6.4.14 */
        SET_BITS(REG32(RCC_BASE + 0x40UL), 1UL << 12);
        CLR_BITS(REG32(RCC_BASE + 0x40UL), 1UL << 12);
    }
#elif defined(STM32WBA55xx)
    if (instance == SPI1) {                       /* RCC_APB2RSTR.SPI1RST, RM0493 §12.8.18 */
        SET_BITS(REG32(RCC_BASE + 0x7CUL), 1UL << 12);
        CLR_BITS(REG32(RCC_BASE + 0x7CUL), 1UL << 12);
    }
    if (instance == SPI3) {                       /* RCC_APB7RSTR.SPI3RST, RM0493 §12.8.19 */
        SET_BITS(REG32(RCC_BASE + 0x80UL), 1UL << 5);
        CLR_BITS(REG32(RCC_BASE + 0x80UL), 1UL << 5);
    }
#else
    /* H5: left to the H5 SPI work (its SPI has no kernel clock yet). */
    ll_spi_disable(instance);
#endif
}

static void _spi_apply(hal_spi_t *h)
{
    ll_spi_config(h->instance, h->cfg.prescaler, h->cfg.cpol, h->cfg.cpha,
                  h->cfg.lsb_first);
}

static void _spi_recover(hal_spi_t *h)
{
    _spi_reset(h->instance);
    _spi_apply(h);
}

static hal_status_t _spi_status(int rc)
{
    return rc == LL_SPI_OK ? HAL_OK : rc == LL_SPI_TIMEOUT ? HAL_TIMEOUT : HAL_ERROR;
}

static hal_status_t _spi_result(hal_spi_t *h, int rc)
{
    if (rc != LL_SPI_OK) _spi_recover(h);
    return _spi_status(rc);
}

/* ============================================================
 * Init / Configure / Deinit
 * ============================================================ */

hal_status_t hal_spi_init(hal_spi_t *h, SPI_TypeDef *instance,
                          const hal_spi_config_t *cfg)
{
    if (!h || !instance || !cfg) return HAL_ERROR;
    if (_spi_dma_owner == h) _spi_dma_owner = NULL;
    memset(h, 0, sizeof(*h));
    h->instance = instance;
    h->cs_active_low = 1;  /* Default: CS active low */
    h->fill = 0xFF;
    h->cfg = *cfg;

    _spi_clk_enable(instance);
    _spi_reset(instance);
    _spi_apply(h);
    return HAL_OK;
}

hal_status_t hal_spi_configure(hal_spi_t *h, const hal_spi_config_t *cfg)
{
    if (!h || !h->instance || !cfg) return HAL_ERROR;
    if (h->busy) return HAL_BUSY;
    h->cfg = *cfg;
    ll_spi_disable(h->instance);   /* bounded wait for the bus to go idle */
    _spi_apply(h);
    return HAL_OK;
}

void hal_spi_deinit(hal_spi_t *h)
{
    if (h && h->instance) {
        if (h->busy) (void)hal_spi_dma_wait(h);
        ll_spi_disable(h->instance);
        h->instance = NULL;
    }
}

void hal_spi_set_cs(hal_spi_t *h, GPIO_TypeDef *port, uint32_t pin)
{
    h->cs_port = port;
    h->cs_pin = pin;

    /* Configure CS as push-pull output and deassert */
    ll_rcc_gpio_clk_enable(port);
    if (h->cs_active_low) ll_gpio_set(port, 1UL << pin);
    else                  ll_gpio_clear(port, 1UL << pin);
    ll_gpio_config_output(port, pin);
}

void hal_spi_set_cs_map(hal_spi_t *h, const hal_spi_cs_t *map, uint8_t n)
{
    if (!h) return;
    h->cs_map = n ? map : NULL;
    h->cs_map_len = map ? n : 0;
    for (uint8_t i = 0; i < h->cs_map_len; i++) {
        GPIO_TypeDef *port = map[i].port;
        uint32_t pin = map[i].pin;
        /* Level first, then mode: the pin never drives the active level. */
        ll_rcc_gpio_clk_enable(port);
        if (h->cs_active_low) ll_gpio_set(port, 1UL << pin);
        else                  ll_gpio_clear(port, 1UL << pin);
        ll_gpio_config_output(port, pin);
    }
}

uint32_t hal_spi_sck_hz(const hal_spi_t *h)
{
    if (!h || !h->instance) return 0;
    return (_ll_spi_cycles_per_ms() * 1000UL) >> ((h->cfg.prescaler & 7UL) + 1UL);
}

/* ============================================================
 * CS control
 * ============================================================ */

void hal_spi_select(hal_spi_t *h)
{
    if (h->cs_port) {
        if (h->cs_active_low)
            ll_gpio_clear(h->cs_port, 1UL << h->cs_pin);
        else
            ll_gpio_set(h->cs_port, 1UL << h->cs_pin);
    }
}

void hal_spi_deselect(hal_spi_t *h)
{
    if (h->cs_port) {
        if (h->cs_active_low)
            ll_gpio_set(h->cs_port, 1UL << h->cs_pin);
        else
            ll_gpio_clear(h->cs_port, 1UL << h->cs_pin);
    }
}

/* Drive one pin active (1) or inactive (0) for the handle's polarity. */
static void _spi_cs_drive(const hal_spi_t *h, GPIO_TypeDef *port, uint32_t pin, int active)
{
    if (active == (h->cs_active_low != 0)) ll_gpio_clear(port, 1UL << pin);
    else                                   ll_gpio_set(port, 1UL << pin);
}

static const hal_spi_cs_t *_spi_cs_find(const hal_spi_t *h, uint8_t id)
{
    for (uint8_t i = 0; i < h->cs_map_len; i++)
        if (h->cs_map[i].id == id)
            return &h->cs_map[i];
    return NULL;
}

int hal_spi_select_id(hal_spi_t *h, uint8_t id)
{
    if (!h) return -1;
    if (!h->cs_map_len) {           /* one device on the bus: its own CS */
        hal_spi_select(h);
        return 0;
    }
    const hal_spi_cs_t *cs = _spi_cs_find(h, id);
    if (!cs) return -1;             /* unknown device: select nothing */
    _spi_cs_drive(h, cs->port, cs->pin, 1);
    return 0;
}

void hal_spi_deselect_id(hal_spi_t *h, uint8_t id)
{
    if (!h) return;
    if (!h->cs_map_len) {
        hal_spi_deselect(h);
        return;
    }
    const hal_spi_cs_t *cs = _spi_cs_find(h, id);
    if (cs) _spi_cs_drive(h, cs->port, cs->pin, 0);
}

/* ============================================================
 * Polled transfers
 * ============================================================ */

hal_status_t hal_spi_exchange(hal_spi_t *h, const uint8_t *tx, uint8_t *rx,
                              uint32_t len)
{
    if (!h || !h->instance) return HAL_ERROR;
    if (h->busy) return HAL_BUSY;
    if (len == 0) return HAL_OK;
    int rc = ll_spi_xfer_poll(h->instance, tx, rx, len, h->fill,
                              ll_spi_stall_cycles(h->instance));
    return _spi_result(h, rc);
}

uint8_t hal_spi_transfer(hal_spi_t *h, uint8_t tx)
{
    uint8_t rx = 0xFF;
    if (hal_spi_exchange(h, &tx, &rx, 1) != HAL_OK) return 0xFF;
    return rx;
}

void hal_spi_transfer_buf(hal_spi_t *h, uint8_t *buf, uint32_t len)
{
    (void)hal_spi_exchange(h, buf, buf, len);
}

hal_status_t hal_spi_write(hal_spi_t *h, const uint8_t *data, uint32_t len)
{
    return hal_spi_exchange(h, data, NULL, len);
}

hal_status_t hal_spi_read(hal_spi_t *h, uint8_t *buf, uint32_t len)
{
    return hal_spi_exchange(h, NULL, buf, len);
}

hal_status_t hal_spi_xfer(hal_spi_t *h, const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    if (!h || !h->instance) return HAL_ERROR;
    hal_spi_select(h);
    hal_status_t s = hal_spi_exchange(h, tx, rx, len);
    hal_spi_deselect(h);
    return s;
}

hal_status_t hal_spi_write_read(hal_spi_t *h, const uint8_t *tx, uint32_t tx_len,
                                uint8_t *rx, uint32_t rx_len)
{
    if (!h || !h->instance) return HAL_ERROR;
    hal_spi_select(h);
    hal_status_t s = hal_spi_exchange(h, tx, NULL, tx_len);
    if (s == HAL_OK) s = hal_spi_exchange(h, NULL, rx, rx_len);
    hal_spi_deselect(h);
    return s;
}

/* ============================================================
 * DMA transfers
 * ============================================================ */

#if defined(_SPI_HAS_DMA)
/* Finish an async transfer: status, release the channels, callback. */
static void _spi_dma_done(hal_spi_t *h, int rc)
{
    if (rc != LL_SPI_OK) _spi_recover(h);
    h->dma_status = (int8_t)_spi_status(rc);
    _spi_dma_owner = NULL;
    h->busy = 0;
    if (h->complete_cb)
        h->complete_cb(h->complete_ctx);
}

/* Claim the shared channel pair. */
static int _spi_dma_claim(hal_spi_t *h)
{
    uint32_t pm = _spi_irq_save();
    int ok = (_spi_dma_owner == NULL);
    if (ok) _spi_dma_owner = h;
    _spi_irq_restore(pm);
    return ok;
}
#endif

#if defined(STM32L422xx)
/* ---------------- L4: classic DMA1 ---------------- */

#define _SPI_DMA_MAX_FRAMES  0xFFFFUL      /* CNDTR is 16-bit */

static uint8_t _spi_dma_sink;               /* RX destination when rx == NULL */

/* Arm both channels and start (RM0394 §41.4.9 "Communication using DMA":
 * RXDMAEN, then the channels, then TXDMAEN; SPE is already on). PSIZE = 8
 * bits keeps the DR access byte-wide, so no data packing. */
static void _l4_dma_arm(hal_spi_t *h, const uint8_t *tx, uint8_t *rx,
                        uint32_t n, int irq)
{
    SPI_TypeDef *s = h->instance;

    ll_rcc_dma1_clk_enable();
    ll_dma_disable(DMA1_CH2);
    ll_dma_disable(DMA1_CH3);
    ll_dma_clear_flags(DMA1_CH2);
    ll_dma_clear_flags(DMA1_CH3);
    ll_dma_set_request(DMA1_CH2, 1);  /* SPI1_RX */
    ll_dma_set_request(DMA1_CH3, 1);  /* SPI1_TX */

    if (!(s->CR1 & LL_SPI_CR1_SPE)) SET_BITS(s->CR1, LL_SPI_CR1_SPE);
    for (uint32_t i = 0; i < 4 && (s->SR & LL_SPI_SR_RXNE); i++)
        (void)*(volatile uint8_t *)&s->DR;          /* stale bytes */

    uint32_t rx_ccr = LL_DMA_CCR_PSIZE_8 | LL_DMA_CCR_MSIZE_8 | LL_DMA_CCR_PL_VHIGH
                    | (rx ? LL_DMA_CCR_MINC : 0)
                    | (irq ? (LL_DMA_CCR_TCIE | LL_DMA_CCR_TEIE) : 0);
    ll_dma_config(DMA1_CH2, &s->DR, rx ? rx : &_spi_dma_sink, n, rx_ccr);

    uint32_t tx_ccr = LL_DMA_CCR_DIR  /* Memory-to-peripheral */
                    | LL_DMA_CCR_PSIZE_8 | LL_DMA_CCR_MSIZE_8 | LL_DMA_CCR_PL_HIGH
                    | (tx ? LL_DMA_CCR_MINC : 0);
    ll_dma_config(DMA1_CH3, &s->DR, tx ? (void *)(uintptr_t)tx : (void *)&h->fill, n, tx_ccr);

    ll_spi_enable_dma_rx(s);
    ll_dma_enable(DMA1_CH2);
    ll_dma_enable(DMA1_CH3);
    ll_spi_enable_dma_tx(s);          /* TXE is set: the first request starts SCK */
}

/* Close (RM0394 §41.4.9): channels off, the disable wait (FTLVL=0, BSY=0),
 * then the SPI DMA enables off. SPE stays on for the next transfer. After a
 * failure there is no bus wait: the caller resets the peripheral anyway. */
static int _l4_dma_close(hal_spi_t *h, int rc)
{
    SPI_TypeDef *s = h->instance;
    ll_dma_disable(DMA1_CH2);
    ll_dma_disable(DMA1_CH3);

    if (rc == LL_SPI_OK) {
        ll_spi_deadline_t dl;
        _ll_spi_deadline_start(&dl, ll_spi_stall_cycles(s));
        while (s->SR & (LL_SPI_SR_FTLVL_MASK | LL_SPI_SR_BSY)) {
            if (_ll_spi_deadline_expired(&dl)) { rc = LL_SPI_TIMEOUT; break; }
        }
    }
    ll_spi_disable_dma_rx(s);
    ll_spi_disable_dma_tx(s);
    ll_dma_clear_flags(DMA1_CH2);
    ll_dma_clear_flags(DMA1_CH3);
    if (rc == LL_SPI_OK && (s->SR & (LL_SPI_SR_OVR | LL_SPI_SR_MODF))) rc = LL_SPI_ERR;
    return rc;
}

/* Remaining RX frames: the progress measure for the stall deadline. */
static inline uint32_t _dma_rx_left(void) { return DMA1_CH2->CNDTR & 0xFFFFUL; }

static int _l4_dma_wait(hal_spi_t *h)
{
    ll_spi_deadline_t dl;
    uint32_t stall = ll_spi_stall_cycles(h->instance);
    uint32_t left = _dma_rx_left();
    _ll_spi_deadline_start(&dl, stall);
    for (;;) {
        if (ll_dma_transfer_error(DMA1_CH2) || ll_dma_transfer_error(DMA1_CH3))
            return LL_SPI_ERR;
        if (ll_dma_transfer_complete(DMA1_CH2))   /* last byte received = bus done */
            return LL_SPI_OK;
        uint32_t now = _dma_rx_left();
        if (now != left) { left = now; _ll_spi_deadline_start(&dl, stall); }
        else if (_ll_spi_deadline_expired(&dl)) return LL_SPI_TIMEOUT;
    }
}

#define _dma_arm    _l4_dma_arm
#define _dma_wait   _l4_dma_wait
#define _dma_close  _l4_dma_close

void DMA1_Channel2_IRQHandler(void)
{
    /* RX transfer complete (or error) for the async path */
    int err = ll_dma_transfer_error(DMA1_CH2);
    int tc  = ll_dma_transfer_complete(DMA1_CH2);
    ll_dma_clear_flags(DMA1_CH2);

    hal_spi_t *h = _spi_dma_owner;
    if (!h || !h->busy) { ll_dma_disable(DMA1_CH2); return; }
    if (!err && !tc) return;
    int rc = _l4_dma_close(h, err ? LL_SPI_ERR : LL_SPI_OK);
    _spi_dma_done(h, rc);
}

static void _dma_irq_enable(hal_spi_t *h)
{
    (void)h;
    hal_nvic_set_priority(HAL_IRQ_DMA1_CH2, 6);
    hal_nvic_enable_irq(HAL_IRQ_DMA1_CH2);
}

static uint32_t _dma_async_max(hal_spi_t *h) { (void)h; return _SPI_DMA_MAX_FRAMES; }

#elif defined(STM32WBA55xx)
/* ---------------- WBA: GPDMA1 + SPI v2 ---------------- */

#define _SPI_RX_CH   GPDMA1_CH6
#define _SPI_TX_CH   GPDMA1_CH7

static uint8_t _spi_dma_sink;               /* RX destination when rx == NULL */

/* Stop a channel with a bounded wait: suspend, wait for idle, reset
 * (GPDMA_CxCR.RESET needs a suspended or disabled channel). */
static void _gpdma_abort(GPDMA_Channel_TypeDef *ch)
{
    if (ch->CCR & LL_GPDMA_CCR_EN) {
        SET_BITS(ch->CCR, LL_GPDMA_CCR_SUSP);
        uint32_t guard = 100000;
        while (!(ch->CSR & LL_GPDMA_CSR_IDLEF) && --guard) { }
    }
    ch->CCR  = LL_GPDMA_CCR_RESET;
    ch->CFCR = LL_GPDMA_CFCR_ALL;
}

/* Arm and start (RM0493 §41.4.14: RXDMAEN, channels, TXDMAEN, then SPE and
 * CSTART). TSIZE = n closes the session with EOT. */
static void _wba_dma_arm(hal_spi_t *h, const uint8_t *tx, uint8_t *rx,
                         uint32_t n, int irq)
{
    SPI_TypeDef *s = h->instance;
    uint32_t rq_rx = (s == SPI1) ? 1UL : 3UL;
    uint32_t rq_tx = (s == SPI1) ? 2UL : 4UL;

    ll_rcc_dma1_clk_enable();
    _gpdma_abort(_SPI_RX_CH);
    _gpdma_abort(_SPI_TX_CH);

    CLR_BITS(s->CR1, LL_SPI_CR1_SPE);            /* TSIZE writable only when off */
    CLR_BITS(s->CFG1, LL_SPI_CFG1_TXDMAEN | LL_SPI_CFG1_RXDMAEN);
    s->CR2  = n;
    s->IFCR = LL_SPI_IFCR_ALL;
    s->IER  = 0;

    /* RX: RXDR → memory; DREQ=0, so the SPI (the source) paces it. */
    ll_gpdma_config(_SPI_RX_CH, (volatile void *)&s->RXDR, rx ? rx : &_spi_dma_sink, n,
                    LL_GPDMA_CTR1_SDW_BYTE | LL_GPDMA_CTR1_DDW_BYTE
                    | (rx ? LL_GPDMA_CTR1_DINC : 0),
                    (rq_rx << LL_GPDMA_CTR2_REQSEL_SHIFT) | LL_GPDMA_CTR2_TCEM_BLOCK);
    /* TX: memory → TXDR; DREQ=1, the SPI (the destination) paces it. */
    ll_gpdma_config(_SPI_TX_CH,
                    (volatile void *)(uintptr_t)(tx ? tx : &h->fill),
                    (void *)(uintptr_t)&s->TXDR, n,
                    LL_GPDMA_CTR1_SDW_BYTE | LL_GPDMA_CTR1_DDW_BYTE
                    | (tx ? LL_GPDMA_CTR1_SINC : 0),
                    (rq_tx << LL_GPDMA_CTR2_REQSEL_SHIFT) | LL_GPDMA_CTR2_DREQ
                    | LL_GPDMA_CTR2_TCEM_BLOCK);
    _SPI_RX_CH->CCR = LL_GPDMA_CCR_PRIO_HIGH;     /* RX first: never let RxFIFO fill */
    _SPI_TX_CH->CCR = LL_GPDMA_CCR_PRIO_LOW_HIGH;

    SET_BITS(s->CFG1, LL_SPI_CFG1_RXDMAEN);
    ll_gpdma_enable(_SPI_RX_CH);
    ll_gpdma_enable(_SPI_TX_CH);
    SET_BITS(s->CFG1, LL_SPI_CFG1_TXDMAEN);
    if (irq) s->IER = LL_SPI_IER_EOTIE;
    SET_BITS(s->CR1, LL_SPI_CR1_SPE);
    SET_BITS(s->CR1, LL_SPI_CR1_CSTART);
}

/* Close (RM0493 §41.4.14): channels off, SPI disable procedure (after EOT
 * that is just SPE=0), DMA enables off. Leaves SPE=0, as every v2 transfer
 * does. After a failure the caller resets the peripheral. */
static int _wba_dma_close(hal_spi_t *h, int rc)
{
    SPI_TypeDef *s = h->instance;
    _gpdma_abort(_SPI_RX_CH);
    _gpdma_abort(_SPI_TX_CH);
    s->IER = 0;
    if (rc == LL_SPI_OK && (s->SR & (LL_SPI_SR_OVR | LL_SPI_SR_MODF))) rc = LL_SPI_ERR;
    CLR_BITS(s->CR1, LL_SPI_CR1_SPE);   /* EOT seen, or a failure the caller resets */
    CLR_BITS(s->CFG1, LL_SPI_CFG1_TXDMAEN | LL_SPI_CFG1_RXDMAEN);
    s->IFCR = LL_SPI_IFCR_ALL;
    return rc;
}

static inline uint32_t _dma_rx_left(void) { return _SPI_RX_CH->CBR1 & 0xFFFFUL; }

#define _GPDMA_ERRS  (LL_GPDMA_CSR_DTEF | LL_GPDMA_CSR_ULEF | LL_GPDMA_CSR_USEF)

/* Done = EOT (last frame on the bus) and the RX channel's TC (last frame in
 * memory: after EOT the final packet is still drained by DMA, §41.4.14). */
static int _wba_dma_wait(hal_spi_t *h)
{
    SPI_TypeDef *s = h->instance;
    ll_spi_deadline_t dl;
    uint32_t stall = ll_spi_stall_cycles(s);
    uint32_t left = _dma_rx_left();
    _ll_spi_deadline_start(&dl, stall);
    for (;;) {
        if ((_SPI_RX_CH->CSR | _SPI_TX_CH->CSR) & _GPDMA_ERRS) return LL_SPI_ERR;
        uint32_t sr = s->SR;
        if (sr & (LL_SPI_SR_OVR | LL_SPI_SR_MODF)) return LL_SPI_ERR;
        if ((sr & LL_SPI_SR_EOT) && (_SPI_RX_CH->CSR & LL_GPDMA_CSR_TCF)) return LL_SPI_OK;
        uint32_t now = _dma_rx_left();
        if (now != left) { left = now; _ll_spi_deadline_start(&dl, stall); }
        else if (_ll_spi_deadline_expired(&dl)) return LL_SPI_TIMEOUT;
    }
}

#define _dma_arm    _wba_dma_arm
#define _dma_wait   _wba_dma_wait
#define _dma_close  _wba_dma_close

/* Async completion: the SPI's own EOT interrupt (bus-accurate, unlike the TX
 * channel's TC, which fires when the last byte enters the FIFO). */
static void _wba_spi_irq(SPI_TypeDef *s)
{
    hal_spi_t *h = _spi_dma_owner;
    uint32_t sr = s->SR;
    if (!h || h->instance != s || !h->busy) {
        s->IER = 0;
        s->IFCR = LL_SPI_IFCR_ALL;
        return;
    }
    if (!(sr & (LL_SPI_SR_EOT | LL_SPI_SR_OVR | LL_SPI_SR_MODF))) return;
    int rc = (sr & (LL_SPI_SR_OVR | LL_SPI_SR_MODF)) ? LL_SPI_ERR : LL_SPI_OK;
    if (rc == LL_SPI_OK) {
        uint32_t guard = 20000;           /* the last frame or two, RxFIFO → memory */
        while (!(_SPI_RX_CH->CSR & (LL_GPDMA_CSR_TCF | _GPDMA_ERRS)) && --guard) { }
        if (!guard || (_SPI_RX_CH->CSR & _GPDMA_ERRS)) rc = LL_SPI_ERR;
    }
    rc = _wba_dma_close(h, rc);
    _spi_dma_done(h, rc);
}

void SPI1_IRQHandler(void) { _wba_spi_irq(SPI1); }
void SPI3_IRQHandler(void) { _wba_spi_irq(SPI3); }

static void _dma_irq_enable(hal_spi_t *h)
{
    uint32_t irqn = (h->instance == SPI1) ? HAL_IRQ_SPI1 : HAL_IRQ_SPI3;
    hal_nvic_set_priority(irqn, 6);
    hal_nvic_enable_irq(irqn);
}

static uint32_t _dma_async_max(hal_spi_t *h) { return _ll_spi_max_frames(h->instance); }

#endif /* DMA backends */

hal_status_t hal_spi_exchange_dma(hal_spi_t *h, const uint8_t *tx, uint8_t *rx,
                                  uint32_t len)
{
    if (!h || !h->instance) return HAL_ERROR;
    if (h->busy) return HAL_BUSY;
    if (len == 0) return HAL_OK;
#if defined(_SPI_HAS_DMA)
    if (!_spi_dma_claim(h)) return HAL_BUSY;
#if defined(STM32L422xx)
    const uint32_t max = _SPI_DMA_MAX_FRAMES;
#else
    const uint32_t max = _ll_spi_max_frames(h->instance);
#endif
    int rc = LL_SPI_OK;
    while (len && rc == LL_SPI_OK) {
        uint32_t n = len > max ? max : len;
        _dma_arm(h, tx, rx, n, 0);
        rc = _dma_close(h, _dma_wait(h));
        if (tx) tx += n;
        if (rx) rx += n;
        len -= n;
    }
    _spi_dma_owner = NULL;
    return _spi_result(h, rc);
#else
    (void)tx; (void)rx;
    return HAL_ERROR;
#endif
}

hal_status_t hal_spi_xfer_dma(hal_spi_t *h, const uint8_t *tx, uint8_t *rx,
                               uint32_t len, hal_callback_t cb, void *ctx)
{
    if (!h || !h->instance) return HAL_ERROR;
    if (h->busy) return HAL_BUSY;
#if defined(_SPI_HAS_DMA)
    if (len == 0 || len > _dma_async_max(h)) return HAL_ERROR;
    if (!_spi_dma_claim(h)) return HAL_BUSY;

    h->busy          = 1;
    h->dma_status    = (int8_t)HAL_BUSY;
    h->complete_cb   = cb;
    h->complete_ctx  = ctx;
    h->dma_rx_buf    = rx;
    h->dma_len       = len;

    _dma_irq_enable(h);
    _dma_arm(h, tx, rx, len, 1);
    return HAL_OK;
#else
    (void)tx; (void)rx; (void)len; (void)cb; (void)ctx;
    return HAL_ERROR;
#endif
}

hal_status_t hal_spi_dma_wait(hal_spi_t *h)
{
    if (!h) return HAL_ERROR;
    if (!h->busy || !h->instance) return (hal_status_t)h->dma_status;
#if defined(_SPI_HAS_DMA)
    ll_spi_deadline_t dl;
    uint32_t stall = ll_spi_stall_cycles(h->instance);
    uint32_t left = _dma_rx_left();
    _ll_spi_deadline_start(&dl, stall);
    while (h->busy) {
        uint32_t now = _dma_rx_left();
        if (now != left) { left = now; _ll_spi_deadline_start(&dl, stall); continue; }
        if (!_ll_spi_deadline_expired(&dl)) continue;
        uint32_t pm = _spi_irq_save();       /* keep the ISR out while aborting */
        if (h->busy) _spi_dma_done(h, _dma_close(h, LL_SPI_TIMEOUT));
        _spi_irq_restore(pm);
    }
#endif
    return (hal_status_t)h->dma_status;
}

int hal_spi_busy(hal_spi_t *h)
{
    return h->busy;
}
