/**
 * hal_sai.c — SAI1 PDM master receiver + GPDMA circular capture (WBA55).
 *
 * Register sequence distilled from ST's HAL_SAI_Init / HAL_SAI_Receive_DMA for
 * the PDM path; data packing (1 bit/clock, MSB-first into 32-bit words) per
 * RM0493 §SAI. Register defs in ll_sai.h; GPDMA LLI in ll_dma.h.
 *
 * NOTE: not yet silicon-validated. The PDM clock math, CKEN line, and the
 * GPDMA self-linked circular descriptor want an on-hardware check (see the
 * MIC_PDM harness step).
 */
#include "hal_sai.h"

/* SAI capture: W5 only — the only Core with an SAI (the L011, L422 and H523
 * have none; see ll_sai.h). Empty translation unit elsewhere. */
#if defined(STM32WBA55xx)

/* SAI1 GPDMA hardware request line (RM0493 Table; SAI1_A). */
#define HAL_SAI_GPDMA_REQ_SAI1_A 17u

/* SAI1 peripheral clock enable: RCC APB2ENR (offset 0x0A4) bit 21. */
static inline void sai1_clk_enable(void)
{
    SET_BITS(REG32(RCC_BASE + 0x0A4UL), (1UL << 21));
    (void)REG32(RCC_BASE + 0x0A4UL);
}

hal_status_t hal_sai_pdm_init(hal_sai_t *s, SAI_Block_TypeDef *block,
                              const hal_sai_pdm_config_t *cfg)
{
    /* PDM is SAI1 block A only; mic on data line 1 or 2; clock on CK1 or CK2. */
    if (block != SAI1_Block_A || cfg == NULL) return HAL_ERROR;
    if (cfg->data_line != 1 && cfg->data_line != 2) return HAL_ERROR;
    if (cfg->clock_line != 1 && cfg->clock_line != 2) return HAL_ERROR;
    if (cfg->kernel_clk_hz == 0 || cfg->pcm_rate_hz == 0) return HAL_ERROR;

    s->block        = block;
    s->dma_ch       = NULL;
    s->active       = false;
    s->ready_half   = 0;

    sai1_clk_enable();

    /* A mic on SAI_D[m] is de-interleaver pair m, which needs MICNBR = m-1
     * (RM0493 Table 408): D1 → MICNBR=0 (2 mics), D2 → MICNBR=1 (4 mics, pair 2
     * = our mic, pair 1 unconnected). One slot per frame carries one byte from
     * each mic: 16-bit slot for MICNBR=0, 32-bit for MICNBR=1 (Fig 479). */
    uint32_t micnbr   = (uint32_t)cfg->data_line - 1u;
    uint32_t ds_field = (micnbr == 1u) ? SAI_CR1_DS_32 : SAI_CR1_DS_16;
    uint32_t frl      = (16u * (micnbr + 1u)) - 1u;     /* RM0493 Table 408 */

    /* PDM bitstream clock (RM0493 Table 407 "Adjusting the bitstream clock"):
     *   F_PDM_CK = pcm_rate * 128 (fixed decimation),
     *   F_SCK_A  = F_PDM_CK * (MICNBR+1) * 2.
     * With NODIV=1 the bit clock is generated directly as F_ker / MCKDIV, so
     *   MCKDIV = round(F_ker / F_SCK_A), clamped 1..63. */
    uint32_t pdm_ck = cfg->pcm_rate_hz * HAL_SAI_PDM_DECIMATION;
    uint32_t f_sck  = pdm_ck * (micnbr + 1u) * 2u;
    uint32_t mckdiv = (cfg->kernel_clk_hz + f_sck / 2u) / f_sck;
    if (mckdiv < 1) mckdiv = 1;
    if (mckdiv > 63) mckdiv = 63;

    /* Block must be off before (re)configuring; GCR with both blocks idle. */
    ll_sai_block_disable(block);
    SAI1->GCR = 0;

    /* CR1 per RM0493 Table 407: master RX, free protocol (TDM), MSB-first,
     * CKSTR=0 (data stable on the falling edge), MONO=0 (always stereo for PDM),
     * NODIV=1 (no MCLK — drive the PDM bitstream clock on CKx, not MCLK_A; this
     * is the bit that was wrong, leaving PA6/AF3 acting as MCLK_A not CK2). */
    block->CR1 = SAI_CR1_MODE_MASTER_RX
               | SAI_CR1_PRTCFG_FREE
               | ds_field
               | SAI_CR1_NODIV
               | ((mckdiv & 0x3FUL) << SAI_CR1_MCKDIV_Pos);

    /* CR2: FIFO threshold half-full for DMA. */
    block->CR2 = SAI_CR2_FTH_HF;

    /* FRCR per Table 407: FRL = 16*(MICNBR+1)-1; FS active high, start-of-frame,
     * one-bit pulse, no offset (FSALL=FSDEF=FSOFF=0, FSPOL=1). */
    block->FRCR = (frl & SAI_FRCR_FRL) | SAI_FRCR_FSPOL;

    /* SLOTR: one slot per frame (NBSLOT=0), SLOTSZ=0 (slot size = data size),
     * enable slot 0. The single slot holds one de-interleaved byte per mic. */
    block->SLOTR = (0u << SAI_SLOTR_NBSLOT_Pos)
                 | (0x1u << SAI_SLOTR_SLOTEN_Pos);

    /* PDMCR (RM0493 enable sequence): MICNBR + bitstream-clock pin, PDMEN last,
     * and PDMEN must be set before SAIEN (done by the caller / capture start). */
    uint32_t cken = (cfg->clock_line == 2) ? SAI_PDMCR_CKEN2 : SAI_PDMCR_CKEN1;
    SAI1->PDMCR &= ~SAI_PDMCR_PDMEN;
    SAI1->PDMCR  = cken | (micnbr << SAI_PDMCR_MICNBR_Pos);
    SAI1->PDMDLY = 0;
    SAI1->PDMCR |= SAI_PDMCR_PDMEN;

    return HAL_OK;
}

hal_status_t hal_sai_capture_start(hal_sai_t *s, GPDMA_Channel_TypeDef *ch,
                                   uint8_t *buf, uint32_t len,
                                   hal_callback_t callback, void *ctx)
{
    if (s->active) return HAL_BUSY;
    if (s->block == NULL || ch == NULL || buf == NULL || len == 0 || (len & 1u))
        return HAL_ERROR;

    s->dma_ch       = ch;
    s->buf          = buf;
    s->len          = len;
    s->callback     = callback;
    s->callback_ctx = ctx;
    s->ready_half   = 0;

    ll_rcc_dma1_clk_enable();

    /* Periph→mem: source = SAI DR (no increment, 32-bit), dest = buffer
     * (increment, 32-bit). HW request = SAI1_A; TCEM at block end so TCF marks
     * full-buffer and HTF marks half-buffer. */
    uint32_t ctr1 = LL_GPDMA_CTR1_SDW_WORD | LL_GPDMA_CTR1_DDW_WORD | LL_GPDMA_CTR1_DINC;
    uint32_t ctr2 = (HAL_SAI_GPDMA_REQ_SAI1_A << LL_GPDMA_CTR2_REQSEL_SHIFT)
                  | LL_GPDMA_CTR2_TCEM_BLOCK;   /* DREQ=0 → source is the HW request */

    ll_gpdma_node_init(&s->node, (volatile void *)&s->block->DR, buf, len, ctr1, ctr2);
    ll_gpdma_node_link(&s->node, &s->node);     /* self-link → circular */

    s->active = true;
    ll_gpdma_list_start(ch, &s->node,
                        LL_GPDMA_CCR_HTIE | LL_GPDMA_CCR_TCIE | LL_GPDMA_CCR_DTEIE |
                        LL_GPDMA_CCR_PRIO_HIGH);

    /* DMA requests on, then enable the block (SAIEN last → starts the clock). */
    ll_sai_block_enable_dma(s->block);
    ll_sai_block_enable(s->block);
    return HAL_OK;
}

void hal_sai_capture_stop(hal_sai_t *s)
{
    if (!s->active) return;
    ll_sai_block_disable(s->block);
    if (s->dma_ch) ll_gpdma_disable(s->dma_ch);
    s->active = false;
}

void hal_sai_irq_handler(hal_sai_t *s)
{
    GPDMA_Channel_TypeDef *ch = s->dma_ch;
    if (ch == NULL) return;
    uint32_t csr = ch->CSR;

    if (csr & LL_GPDMA_CSR_HTF) {        /* lower half filled */
        ch->CFCR = LL_GPDMA_CFCR_HTF;
        s->ready_half = 0;
        if (s->callback) s->callback(s->callback_ctx);
    }
    if (csr & LL_GPDMA_CSR_TCF) {        /* upper half filled */
        ch->CFCR = LL_GPDMA_CFCR_TCF;
        s->ready_half = 1;
        if (s->callback) s->callback(s->callback_ctx);
    }
    if (csr & LL_GPDMA_CSR_DTEF) {       /* transfer error — clear + flag overrun */
        ch->CFCR = LL_GPDMA_CFCR_DTEF;
        ll_sai_clear_overrun(s->block);
    }
}

#endif /* SAI-capable core */
