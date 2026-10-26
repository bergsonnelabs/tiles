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
 * Core.ST.W5 only: it is the only Core with an SAI.
 *
 * @studio category audio label=Core.Audio icon=🎤
 *
 * @studio coverage
 *   id:    audio
 *   name:  Audio — PDM microphone capture
 *   page:  /docs/sdk/audio
 *   blurb: Tier 1. SAI PDM-mode DMA capture + integer decimation to PCM for a
 *          digital MEMS mic. No DSL surface — audio is escape-to-C; the MIC
 *          tile wraps level/RMS helpers on top.
 */
#ifndef CORE_AUDIO_H
#define CORE_AUDIO_H

/* The capture driver comes with the port (port_core_audio_types.h). */
#include "core_pdm.h"
#include "core_caps.h"

#if !CORE_HAS_AUDIO
#error "core_audio.h: SAI PDM capture needs an SAI, and only Core.ST.W5 (STM32WBA55) has one"
#endif

#include "core_types.h"

/** Delivered finished PCM: `n` samples at the configured rate. */
typedef void (*core_audio_pcm_cb)(const int16_t *pcm, uint32_t n, void *ctx);

/* core_dma_channel_t and core_audio_t (the capture state, the decimator,
 * the buffers and the callback) are the port's (port_core_audio_types.h). */
#include "port_core_audio_types.h"

/**
 * @brief  Configure the mic: SAI PDM block + decimator (capture not started).
 * @param  kernel_clk_hz  SAI1 kernel clock (PLLSAI1), already routed.
 * @param  pcm_rate_hz    output audio rate (e.g. 16000).
 * @param  data_line      SAI_Dn the mic sits on: 1 or 2 (sets MICNBR).
 * @param  clock_line     PDM clock pin: 2 = CK2/D2 (the Ring MIC tile).
 * @param  cic_order      decimator order (4 = good voice default).
 * @param  gain_shift     extra digital gain (left shift on PCM; 0 = unity).
 */
CORE_INLINE core_status_t core_audio_init(core_audio_t *a, uint32_t kernel_clk_hz,
                                          uint32_t pcm_rate_hz, uint8_t data_line,
                                          uint8_t clock_line, uint8_t cic_order,
                                          uint8_t gain_shift);

/**
 * @brief  Start continuous capture; `on_pcm` fires per half-buffer with PCM.
 * @param  pdm_buf  raw PDM byte ring (even length, 4-byte aligned).
 * @param  pcm_buf  PCM scratch holding at least pdm_len/32 samples.
 */
CORE_INLINE core_status_t core_audio_start(core_audio_t *a, core_dma_channel_t *ch,
                                           uint8_t *pdm_buf, uint32_t pdm_len,
                                           int16_t *pcm_buf, core_audio_pcm_cb on_pcm,
                                           void *user_ctx);

/** @brief  Stop capture. */
CORE_INLINE void core_audio_stop(core_audio_t *a);

/** @brief  Route the capture channel's GPDMA IRQ here. */
CORE_INLINE void core_audio_irq(core_audio_t *a);

#include "port_core_audio.h"

#endif /* CORE_AUDIO_H */
