/**
 * hal_sai.h — SAI PDM microphone capture (DMA)
 *
 * Configures SAI1 block A as a PDM master receiver and streams the raw 1-bit
 * PDM bitstream into a circular RAM buffer via GPDMA, calling back at the
 * half- and full-buffer marks. Decimation to PCM is done separately by
 * core_pdm (the SAI has no hardware decimator on WBA55).
 *
 * Only SAI1 block A supports the PDM controller (HAL constraint). The SAI
 * kernel clock (PLLSAI1) must already be configured and routed — pass its
 * frequency in; this module does not own the clock tree.
 *
 * Buffer / decimation: the STM32 SAI PDM divider chain fixes the PDM bit
 * clock at Fsai/(2·MCKDIV) and the audio frame at Fsai/(MCKDIV·256), so the
 * decimation factor is always 128 (= PDM_clk / Fs). Pick a kernel clock that
 * makes MCKDIV an integer (e.g. 12.288 MHz → 16 kHz, MCKDIV = 3).
 */
#ifndef HAL_SAI_H
#define HAL_SAI_H

#include <stdint.h>
#include <stdbool.h>
#include "hal_common.h"
#include "ll_sai.h"
#include "ll_rcc.h"   /* defines RCC_BASE — must precede ll_dma.h */
#include "ll_dma.h"

/** Fixed PDM decimation of the STM32 SAI PDM chain (PDM_clk / Fs). */
#define HAL_SAI_PDM_DECIMATION 128

typedef struct {
    uint32_t kernel_clk_hz;  /**< SAI1 kernel clock source, already routed (e.g. HSI16). */
    uint32_t pcm_rate_hz;    /**< Target audio rate (16000 typical). PDM_CK = rate*128. */
    uint8_t  data_line;      /**< SAI_Dn the mic sits on: 1 (pair 1) or 2 (pair 2).
                                  Sets MICNBR = data_line-1 (RM0493 Table 408): a mic on
                                  D2 needs MICNBR=1 so de-interleaver pair 2 is active. */
    uint8_t  clock_line;     /**< PDM clock output pin: 1 = CK1, 2 = CK2.
                                  The Ring MIC tile is on CK2/D2 → data_line=2, clock_line=2. */
} hal_sai_pdm_config_t;

/* Handle + capture API: W5 only (matches hal_sai.c). The L011, L422 and H523
 * have no SAI peripheral. */
#if defined(STM32WBA55xx)

/**
 * SAI capture handle. Allocate in SRAM (the embedded LLI node must be a
 * 32-bit-aligned descriptor the GPDMA engine reads) — a file-scope/global
 * instance is the simplest way to satisfy that.
 */
typedef struct {
    SAI_Block_TypeDef     *block;        /* SAI1_Block_A */
    GPDMA_Channel_TypeDef *dma_ch;       /* capture channel */
    uint8_t               *buf;          /* raw PDM byte buffer */
    uint32_t               len;          /* total bytes (must be even) */
    hal_callback_t         callback;     /* invoked at HT and TC */
    void                  *callback_ctx;
    volatile uint8_t       ready_half;   /* 0 = lower half ready, 1 = upper */
    bool                   active;
    ll_gpdma_node_t        node;         /* self-linked circular descriptor */
} hal_sai_t;

/**
 * @brief  Configure SAI1 block A as a PDM master receiver (block left disabled).
 * @return HAL_OK, or HAL_ERROR if block != SAI1_Block_A or cfg is invalid.
 */
hal_status_t hal_sai_pdm_init(hal_sai_t *s, SAI_Block_TypeDef *block,
                              const hal_sai_pdm_config_t *cfg);

/**
 * @brief  Start continuous DMA capture into `buf` (circular).
 *
 * Arms a self-linking GPDMA LLI (no classic circular bit on GPDMA), enables
 * SAI DMA requests, then SAIEN — which starts the PDM clock. `callback` fires
 * at the half-buffer (process buf[0 .. len/2)) and full-buffer (process
 * buf[len/2 .. len)) marks; read `s->ready_half` to know which.
 *
 * The caller must enable the channel's GPDMA NVIC IRQ and route its handler to
 * hal_sai_irq_handler(s).
 *
 * @param  buf  raw PDM byte buffer in SRAM; len even. 4-byte aligned preferred
 *              (DMA does 32-bit dest writes).
 */
hal_status_t hal_sai_capture_start(hal_sai_t *s, GPDMA_Channel_TypeDef *ch,
                                   uint8_t *buf, uint32_t len,
                                   hal_callback_t callback, void *ctx);

/** @brief  Stop capture (SAIEN off, DMA channel disabled). */
void hal_sai_capture_stop(hal_sai_t *s);

/** @brief  GPDMA channel ISR body — call from the channel's IRQ handler. */
void hal_sai_irq_handler(hal_sai_t *s);

#endif /* STM32WBA55xx */

#endif /* HAL_SAI_H */
