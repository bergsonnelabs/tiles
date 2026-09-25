/**
 * core_audio.h — PDM microphone capture → PCM
 *
 * One-call capture path for a PDM digital mic (e.g. the SPG01 on the Ring's
 * MIC tile): configures SAI1 in PDM mode, streams the raw bitstream into a
 * circular buffer over GPDMA (hal_sai), and decimates each half-buffer to
 * int16 PCM (core_pdm), delivering finished PCM to your callback. Header-only
 * glue over the two SDK functions it leverages.
 *
 * Usage:
 * @code
 *   static core_audio_t mic;            // SRAM (holds the GPDMA descriptor)
 *   static uint8_t  pdm[2048];          // raw PDM (4-byte aligned)
 *   static int16_t  pcm[2048/32];       // per-half PCM: pdm_len/32 samples
 *
 *   core_audio_init(&mic, 12288000, 16000, 2, 2, 4, 0);   // 16 kHz, mic on D2/CK2
 *   core_audio_start(&mic, GPDMA1_CH0, pdm, sizeof pdm, pcm, on_pcm, NULL);
 *   // route GPDMA1_Channel0_IRQHandler → core_audio_irq(&mic)
 * @endcode
 *
 * The SAI kernel clock (PLLSAI1) must be configured/routed beforehand (clock
 * tree is coregen/core_init's job) — pass its frequency in.
 *
 * @studio coverage
 *   id:    audio
 *   name:  Audio — PDM microphone capture
 *   blurb: Tier 1. SAI PDM-mode DMA capture + integer decimation to PCM for a
 *          digital MEMS mic. No DSL surface — audio is escape-to-C; the MIC
 *          tile wraps level/RMS helpers on top.
 */
#ifndef CORE_AUDIO_H
#define CORE_AUDIO_H

#include "hal_sai.h"
#include "core_pdm.h"

#if !defined(STM32WBA55xx)
#error "core_audio.h: SAI PDM capture needs an SAI, and only Core.ST.W5 (STM32WBA55) has one"
#endif

/** Delivered finished PCM: `n` samples at the configured rate. */
typedef void (*core_audio_pcm_cb)(const int16_t *pcm, uint32_t n, void *ctx);

typedef struct {
    hal_sai_t          sai;      /* SAI PDM capture (embeds the GPDMA node) */
    core_pdm_cic_t     pdm;      /* decimator */
    uint8_t           *pdm_buf;  /* raw PDM ring (len bytes) */
    uint32_t           pdm_len;
    int16_t           *pcm_buf;  /* per-half PCM scratch (>= pdm_len/32) */
    core_audio_pcm_cb  on_pcm;
    void              *user_ctx;
} core_audio_t;

/* Internal: decimate whichever half just filled, hand PCM to the user. */
static inline void core_audio__on_half(void *ctx)
{
    core_audio_t *a = (core_audio_t *)ctx;
    uint32_t half = a->pdm_len / 2u;
    const uint8_t *p = a->pdm_buf + (a->sai.ready_half ? half : 0u);
    uint32_t n = core_pdm_process(&a->pdm, p, half, a->pcm_buf);
    if (a->on_pcm) a->on_pcm(a->pcm_buf, n, a->user_ctx);
}

/**
 * @brief  Configure the mic: SAI PDM block + decimator (capture not started).
 * @param  kernel_clk_hz  SAI1 kernel clock (PLLSAI1), already routed.
 * @param  pcm_rate_hz    output audio rate (e.g. 16000).
 * @param  data_line      SAI_Dn the mic sits on: 1 or 2 (sets MICNBR).
 * @param  clock_line     PDM clock pin: 2 = CK2/D2 (the Ring MIC tile).
 * @param  cic_order      decimator order (4 = good voice default).
 * @param  gain_shift     extra digital gain (left shift on PCM; 0 = unity).
 */
static inline hal_status_t core_audio_init(core_audio_t *a, uint32_t kernel_clk_hz,
                                           uint32_t pcm_rate_hz, uint8_t data_line,
                                           uint8_t clock_line, uint8_t cic_order,
                                           uint8_t gain_shift)
{
    hal_sai_pdm_config_t cfg = {
        .kernel_clk_hz = kernel_clk_hz, .pcm_rate_hz = pcm_rate_hz,
        .data_line = data_line, .clock_line = clock_line,
    };
    hal_status_t st = hal_sai_pdm_init(&a->sai, SAI1_Block_A, &cfg);
    if (st != HAL_OK) return st;
    /* SAI PDM fixes decimation at 128; bits arrive MSB-first (lsb_first=0). */
    core_pdm_init(&a->pdm, cic_order ? cic_order : 4u, HAL_SAI_PDM_DECIMATION,
                  gain_shift, 0);
    return HAL_OK;
}

/**
 * @brief  Start continuous capture; `on_pcm` fires per half-buffer with PCM.
 * @param  pdm_buf  raw PDM byte ring (even length, 4-byte aligned).
 * @param  pcm_buf  PCM scratch holding at least pdm_len/32 samples.
 */
static inline hal_status_t core_audio_start(core_audio_t *a, GPDMA_Channel_TypeDef *ch,
                                            uint8_t *pdm_buf, uint32_t pdm_len,
                                            int16_t *pcm_buf, core_audio_pcm_cb on_pcm,
                                            void *user_ctx)
{
    a->pdm_buf = pdm_buf; a->pdm_len = pdm_len; a->pcm_buf = pcm_buf;
    a->on_pcm = on_pcm; a->user_ctx = user_ctx;
    return hal_sai_capture_start(&a->sai, ch, pdm_buf, pdm_len, core_audio__on_half, a);
}

/** @brief  Stop capture. */
static inline void core_audio_stop(core_audio_t *a) { hal_sai_capture_stop(&a->sai); }

/** @brief  Route the capture channel's GPDMA IRQ here. */
static inline void core_audio_irq(core_audio_t *a) { hal_sai_irq_handler(&a->sai); }

#endif /* CORE_AUDIO_H */
