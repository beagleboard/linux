/*
 * Copyright 2021
 */

#ifndef _LIGHT_PCM_H
#define _LIGHT_PCM_H

#include <linux/scatterlist.h>
#include <linux/device.h>
#include <linux/dmaengine.h>

struct thead_pcm_fiq_params {
	int irq;
	void __iomem *base;
	struct snd_dmaengine_dai_dma_data *dma_params_rx;
	struct snd_dmaengine_dai_dma_data *dma_params_tx;
};

int light_pcm_dma_init(struct platform_device *pdev, size_t size);

#endif /* _LIGHT_PCM_H */
