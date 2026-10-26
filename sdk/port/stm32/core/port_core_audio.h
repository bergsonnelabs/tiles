/**
 * port_core_audio.h — STM32 port: the bodies of core_audio.h
 *
 * Included at the end of core_audio.h, after its prototypes; not for direct
 * use. SAI1 block A in PDM mode, its GPDMA circular capture (hal_sai) and
 * the per-half-buffer decimation into the user's callback.
 */

#ifndef PORT_CORE_AUDIO_H
#define PORT_CORE_AUDIO_H

#include "hal_sai.h"

/* Internal: decimate whichever half just filled, hand PCM to the user. */
static inline void core_audio__on_half(void *ctx)
{
    core_audio_t *a = (core_audio_t *)ctx;
    uint32_t half = a->pdm_len / 2u;
    const uint8_t *p = a->pdm_buf + (a->sai.ready_half ? half : 0u);
    uint32_t n = core_pdm_process(&a->pdm, p, half, a->pcm_buf);
    if (a->on_pcm) a->on_pcm(a->pcm_buf, n, a->user_ctx);
}

static inline core_status_t core_audio_init(core_audio_t *a, uint32_t kernel_clk_hz,
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

static inline core_status_t core_audio_start(core_audio_t *a, core_dma_channel_t *ch,
                                             uint8_t *pdm_buf, uint32_t pdm_len,
                                             int16_t *pcm_buf, core_audio_pcm_cb on_pcm,
                                             void *user_ctx)
{
    a->pdm_buf = pdm_buf; a->pdm_len = pdm_len; a->pcm_buf = pcm_buf;
    a->on_pcm = on_pcm; a->user_ctx = user_ctx;
    return hal_sai_capture_start(&a->sai, ch, pdm_buf, pdm_len, core_audio__on_half, a);
}

static inline void core_audio_stop(core_audio_t *a) { hal_sai_capture_stop(&a->sai); }

static inline void core_audio_irq(core_audio_t *a) { hal_sai_irq_handler(&a->sai); }

#endif /* PORT_CORE_AUDIO_H */
