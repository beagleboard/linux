/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * K3 DTHE V2 crypto accelerator driver
 *
 * Copyright (C) Texas Instruments 2025 - https://www.ti.com
 * Author: T Pratham <t-pratham@ti.com>
 */

#ifndef __TI_DTHEV2_H__
#define __TI_DTHE2V_H__

#include <crypto/aead.h>
#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <crypto/hash.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/skcipher.h>

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/scatterlist.h>

#define DTHE_REG_SIZE		4
#define DTHE_DMA_TIMEOUT_MS	2000

enum dthe_aes_mode {
	DTHE_AES_ECB = 0,
	DTHE_AES_CBC,
};

/* Driver specific struct definitions */

struct dthe_tfm_ctx;

/**
 * struct dthe_data - DTHE_V2 driver instance data
 * @dev: Device pointer
 * @regs: Base address of the register space
 * @list: list node for dev
 * @dma_aes_rx: AES Rx DMA Channel
 * @dma_aes_tx: AES Tx DMA Channel
 * @dma_sha_tx: SHA Tx DMA Channel
 * @aes_mutex: Mutex protecting access to AES engine
 * @ctx: Transform context struct
 */
struct dthe_data {
	struct device *dev;
	void __iomem *regs;
	struct list_head list;

	struct dma_chan *dma_aes_rx;
	struct dma_chan *dma_aes_tx;

	struct dma_chan *dma_sha_tx;

	struct mutex aes_mutex;

	struct dthe_tfm_ctx *ctx;
};

/**
 * struct dthe_list - device data list head
 * @dev_list: linked list head
 * @lock: Spinlock protecting accesses to the list
 */
struct dthe_list {
	struct list_head dev_list;
	spinlock_t lock;
};

/**
 * struct dthe_aes_ctx - AES engine ctx struct
 * @mode: AES mode
 * @keylen: AES key length
 * @key: AES key
 * @enc: flag indicating encryption or decryption operation
 * @aes_compl: Completion variable for use in manual completion in case of DMA callback failure
 */
struct dthe_aes_ctx {
	enum dthe_aes_mode mode;
	unsigned int keylen;
	u32 key[AES_KEYSIZE_256 / sizeof(u32)];
	int enc;
	struct completion aes_compl;
};

/**
 * struct dthe_tfm_ctx - Transform ctx struct containing ctx for all sub-components of DTHE V2
 * @dev_data: Device data struct pointer
 * @ctx_info: Union of ctx structs of various sub-components of DTHE_V2
 */
struct dthe_tfm_ctx {
	struct dthe_data *dev_data;
	union {
		struct dthe_aes_ctx *aes_ctx;
	} ctx_info;
};

/* Struct definitions end */

struct dthe_data *dthe_get_dev(struct dthe_tfm_ctx *ctx);

int dthe_register_aes_algs(void);
void dthe_unregister_aes_algs(void);

#endif
