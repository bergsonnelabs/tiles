/**
 * hal_uart.c — UART HAL driver implementation
 *
 * Polling TX, interrupt-driven RX with ring buffer, printf support.
 * ISR handlers override weak stubs in startup assembly.
 */

#include "hal_uart.h"
#include "ll_uart.h"
#include "ll_rcc.h"
#include <stdio.h>
#include <string.h>

/* ---- Peripheral clock enable ---- */

static void _uart_clk_enable(USART_TypeDef *instance)
{
#if defined(STM32L011xx)
    if (instance == USART2)  ll_rcc_apb1_clk_enable(LL_APB1_USART2);
    if (instance == LPUART1) ll_rcc_apb1_clk_enable(LL_APB1_LPUART1);  /* RM0377 §7.3.14 bit 18 */
#elif defined(STM32L422xx)
    if (instance == USART1)  ll_rcc_apb2_clk_enable(LL_APB2_USART1);
    if (instance == USART2)  ll_rcc_apb1_clk_enable(LL_APB1_USART2);
    if (instance == LPUART1) SET_BITS(REG32(RCC_BASE + 0x5CUL), (1UL << 0));
#elif defined(STM32WBA55xx)
    if (instance == USART1)  ll_rcc_apb2_clk_enable(LL_APB2_USART1);
    if (instance == USART2)  ll_rcc_apb1_clk_enable(LL_APB1_USART2);
    /* RCC_APB7ENR is at 0x0A8 (RM0493 §12.8.27); 0x0AC is RCC_AHB1SMENR. */
    if (instance == LPUART1) ll_rcc_apb7_clk_enable(LL_APB7_LPUART1);
#elif defined(STM32H523xx)
    if (instance == USART1)  ll_rcc_apb2_clk_enable(LL_APB2_USART1);
    if (instance == USART2)  ll_rcc_apb1_clk_enable(LL_APB1_USART2);
    if (instance == USART3)  ll_rcc_apb1_clk_enable(LL_APB1_USART3);
    if (instance == LPUART1) SET_BITS(REG32(RCC_BASE + 0xA8UL), LL_APB3_LPUART1);
#endif
    (void)REG32(RCC_BASE);
}

/* ============================================================
 * Handle registry (maps USART instance → HAL handle)
 * ============================================================ */

#if defined(STM32L011xx)
  #define HAL_UART_MAX_INSTANCES  2
  static hal_uart_t *_handles[HAL_UART_MAX_INSTANCES]; /* USART2, LPUART1 */
#elif defined(STM32L422xx)
  #define HAL_UART_MAX_INSTANCES  3
  static hal_uart_t *_handles[HAL_UART_MAX_INSTANCES]; /* USART1, USART2, LPUART1 */
#elif defined(STM32WBA55xx)
  #define HAL_UART_MAX_INSTANCES  3
  static hal_uart_t *_handles[HAL_UART_MAX_INSTANCES];
#elif defined(STM32H523xx)
  #define HAL_UART_MAX_INSTANCES  4
  static hal_uart_t *_handles[HAL_UART_MAX_INSTANCES]; /* USART1, USART2, USART3, LPUART1 */
#endif

static int _instance_index(USART_TypeDef *instance)
{
#if defined(STM32L011xx)
    if (instance == USART2)  return 0;
    if (instance == LPUART1) return 1;
#elif defined(STM32L422xx)
    if (instance == USART1)  return 0;
    if (instance == USART2)  return 1;
    if (instance == LPUART1) return 2;
#elif defined(STM32WBA55xx)
    if (instance == USART1)  return 0;
    if (instance == USART2)  return 1;
    if (instance == LPUART1) return 2;
#elif defined(STM32H523xx)
    if (instance == USART1)  return 0;
    if (instance == USART2)  return 1;
    if (instance == USART3)  return 2;
    if (instance == LPUART1) return 3;
#endif
    return -1;
}

static uint32_t _irq_number(USART_TypeDef *instance)
{
#if defined(STM32L011xx)
    if (instance == USART2)  return HAL_IRQ_USART2;
    if (instance == LPUART1) return HAL_IRQ_LPUART1;
#elif defined(STM32L422xx)
    if (instance == USART1)  return HAL_IRQ_USART1;
    if (instance == USART2)  return HAL_IRQ_USART2;
    if (instance == LPUART1) return HAL_IRQ_LPUART1;
#elif defined(STM32WBA55xx)
    if (instance == USART1)  return HAL_IRQ_USART1;
    if (instance == USART2)  return HAL_IRQ_USART2;
    if (instance == LPUART1) return HAL_IRQ_LPUART1;
#elif defined(STM32H523xx)
    if (instance == USART1)  return HAL_IRQ_USART1;
    if (instance == USART2)  return HAL_IRQ_USART2;
    if (instance == USART3)  return HAL_IRQ_USART3;
    if (instance == LPUART1) return HAL_IRQ_LPUART1;
#endif
    return 0;
}

/* ============================================================
 * ISR handler (shared across all instances)
 * ============================================================ */

static void _uart_isr(hal_uart_t *h)
{
    if (!h) return;

    uint32_t isr = h->instance->ISR;

    /* RX: data available */
    if (isr & LL_USART_ISR_RXNE) {
        uint8_t byte = (uint8_t)h->instance->RDR;
        if (!hal_ringbuf_put(&h->rx_ring, byte)) {
            h->rx_overrun++;
        }
    }

    /* Error flags */
    if (isr & LL_USART_ISR_ORE) {
        h->rx_overrun++;
        h->instance->ICR = LL_USART_ICR_ORECF;
    }
    if (isr & LL_USART_ISR_FE) {
        h->rx_framing_err++;
        h->instance->ICR = LL_USART_ICR_FECF;
    }
    if (isr & (LL_USART_ISR_PE | LL_USART_ISR_NE)) {
        h->instance->ICR = LL_USART_ICR_PECF | LL_USART_ICR_NECF;
    }
}

/* ============================================================
 * ISR handlers — strong symbols overriding startup's weak stubs
 * ============================================================ */

#if defined(STM32L422xx) || defined(STM32WBA55xx) || defined(STM32H523xx)
void USART1_IRQHandler(void)
{
    _uart_isr(_handles[0]);
}
#endif

void USART2_IRQHandler(void)
{
#if defined(STM32L011xx)
    _uart_isr(_handles[0]);
#else
    _uart_isr(_handles[1]);
#endif
}

#if defined(STM32H523xx)
void USART3_IRQHandler(void)
{
    _uart_isr(_handles[2]);
}
#endif

void LPUART1_IRQHandler(void)
{
#if defined(STM32L011xx)
    _uart_isr(_handles[1]);
#elif defined(STM32L422xx) || defined(STM32WBA55xx)
    _uart_isr(_handles[2]);
#elif defined(STM32H523xx)
    _uart_isr(_handles[3]);
#endif
}

/* ============================================================
 * Init / Deinit
 * ============================================================ */

hal_status_t hal_uart_init(hal_uart_t *h, USART_TypeDef *instance,
                           uint32_t pclk_hz, const hal_uart_config_t *cfg)
{
    int idx = _instance_index(instance);
    if (idx < 0) return HAL_ERROR;

    memset(h, 0, sizeof(*h));
    h->instance = instance;
    h->pclk_hz = pclk_hz;

    /* Enable peripheral clock */
    _uart_clk_enable(instance);

    /* Initialize LL UART */
    if (instance == LPUART1) {
        ll_lpuart_init(instance, pclk_hz, cfg->baud);
    } else {
        ll_uart_init(instance, pclk_hz, cfg->baud);
    }

    /* Set up interrupt RX if requested */
    if (cfg->rx_interrupt) {
        hal_ringbuf_init(&h->rx_ring, h->rx_buf, HAL_UART_RX_BUF_SIZE);
        h->rx_irq_enabled = 1;

        /* Enable RXNE interrupt */
        SET_BITS(instance->CR1, LL_USART_CR1_RXNEIE);

        /* Enable error interrupts for overrun/framing detection */
        SET_BITS(instance->CR3, (1UL << 0));  /* CR3: EIE — error interrupt enable */

        /* Register handle and enable NVIC */
        _handles[idx] = h;
        hal_nvic_set_priority(_irq_number(instance), 4);  /* level 4, mid (the helper shifts it) */
        hal_nvic_enable_irq(_irq_number(instance));
    }

    return HAL_OK;
}

void hal_uart_deinit(hal_uart_t *h)
{
    if (!h || !h->instance) return;

    int idx = _instance_index(h->instance);
    if (idx >= 0) {
        hal_nvic_disable_irq(_irq_number(h->instance));
        _handles[idx] = NULL;
    }

    ll_uart_disable(h->instance);
    h->instance = NULL;
}

/* ============================================================
 * Blocking TX
 * ============================================================ */

void hal_uart_putc(hal_uart_t *h, uint8_t byte)
{
    ll_uart_tx(h->instance, byte);
}

void hal_uart_tx(hal_uart_t *h, const uint8_t *data, uint32_t len)
{
    ll_uart_tx_buf(h->instance, data, len);
}

void hal_uart_tx_str(hal_uart_t *h, const char *str)
{
    ll_uart_tx_str(h->instance, str);
}

int hal_uart_printf(hal_uart_t *h, const char *fmt, ...)
{
    char buf[HAL_UART_PRINTF_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0) {
        if (len > (int)sizeof(buf) - 1)
            len = (int)sizeof(buf) - 1;
        hal_uart_tx(h, (const uint8_t *)buf, (uint32_t)len);
    }
    return len;
}

/* ============================================================
 * RX
 * ============================================================ */

int hal_uart_rx_ready(hal_uart_t *h)
{
    if (h->rx_irq_enabled) {
        return !hal_ringbuf_empty(&h->rx_ring);
    }
    return ll_uart_rx_ready(h->instance);
}

uint8_t hal_uart_rx(hal_uart_t *h)
{
    if (h->rx_irq_enabled) {
        uint8_t byte;
        while (!hal_ringbuf_get(&h->rx_ring, &byte))
            ;
        return byte;
    }
    return ll_uart_rx(h->instance);
}

int hal_uart_rx_try(hal_uart_t *h, uint8_t *byte)
{
    if (h->rx_irq_enabled) {
        return hal_ringbuf_get(&h->rx_ring, byte);
    }
    return ll_uart_rx_try(h->instance, byte);
}

uint16_t hal_uart_read(hal_uart_t *h, uint8_t *buf, uint16_t max_len)
{
    if (h->rx_irq_enabled) {
        return hal_ringbuf_read(&h->rx_ring, buf, max_len);
    }
    /* Polling fallback: read whatever's available */
    uint16_t count = 0;
    while (count < max_len && ll_uart_rx_ready(h->instance)) {
        buf[count++] = (uint8_t)h->instance->RDR;
    }
    return count;
}

uint16_t hal_uart_available(hal_uart_t *h)
{
    if (h->rx_irq_enabled) {
        return hal_ringbuf_count(&h->rx_ring);
    }
    return ll_uart_rx_ready(h->instance) ? 1 : 0;
}

/* ============================================================
 * DMA TX (stub — enabled when DMA channel defines exist)
 * ============================================================ */

hal_status_t hal_uart_tx_dma(hal_uart_t *h, const uint8_t *data, uint32_t len,
                              hal_callback_t cb, void *ctx)
{
    (void)h; (void)data; (void)len; (void)cb; (void)ctx;
    /* DMA TX not yet implemented — requires DMA channel assignment
       from coregen (tile_dma.h) and per-family DMA setup. */
    return HAL_ERROR;
}

int hal_uart_tx_busy(hal_uart_t *h)
{
    return h->tx_busy;
}
