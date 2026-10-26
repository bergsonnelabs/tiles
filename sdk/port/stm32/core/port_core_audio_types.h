/**
 * port_core_audio_types.h — STM32 port: audio capture types for core_audio.h
 *
 * Included by core_audio.h before its prototypes; not for direct use. The
 * bodies of its functions are in port_core_audio.h.
 */

#ifndef PORT_CORE_AUDIO_TYPES_H
#define PORT_CORE_AUDIO_TYPES_H

#include "hal_sai.h"   /* ll_dma.h: GPDMA_Channel_TypeDef */

/** The DMA channel that moves captured samples: a GPDMA channel on the W5. */
typedef GPDMA_Channel_TypeDef core_dma_channel_t;

typedef struct {
    hal_sai_t          sai;      /* SAI PDM capture (embeds the GPDMA node) */
    core_pdm_cic_t     pdm;      /* decimator */
    uint8_t           *pdm_buf;  /* raw PDM ring (len bytes) */
    uint32_t           pdm_len;
    int16_t           *pcm_buf;  /* per-half PCM scratch (>= pdm_len/32) */
    core_audio_pcm_cb  on_pcm;
    void              *user_ctx;
} core_audio_t;

#endif /* PORT_CORE_AUDIO_TYPES_H */
