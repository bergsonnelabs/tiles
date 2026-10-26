/**
 * port_core_audio.h — STM32 port: audio capture types for core_audio.h
 *
 * Included by core_audio.h; not for direct use.
 */

#ifndef PORT_CORE_AUDIO_H
#define PORT_CORE_AUDIO_H

#include "hal_sai.h"   /* ll_dma.h: GPDMA_Channel_TypeDef */

/** The DMA channel that moves captured samples: a GPDMA channel on the W5. */
typedef GPDMA_Channel_TypeDef core_dma_channel_t;

#endif /* PORT_CORE_AUDIO_H */
