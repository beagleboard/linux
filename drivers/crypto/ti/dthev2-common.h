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
#include <crypto/md5.h>
#include <crypto/sha2.h>

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/scatterlist.h>

#define DTHE_REG_SIZE		4
#define DTHE_DMA_TIMEOUT_MS	2000

enum dthe_hash_alg_sel {
	DTHE_HASH_MD5		= 0,
	DTHE_HASH_SHA1		= BIT(1),
	DTHE_HASH_SHA224	= BIT(2),
	DTHE_HASH_SHA256	= BIT(1) | BIT(2),
	DTHE_HASH_SHA384	= BIT(0),
	DTHE_HASH_SHA512	= BIT(0) | BIT(1)
};

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
 * @hash_mutex: Mutex protecting access to HASH engine
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
	struct mutex hash_mutex;

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
 * struct dthe_hash_ctx - Hashing engine ctx struct
 * @mode: Hashing Engine mode
 * @block_size: block size of hash algorithm selected
 * @digest_size: digest size of hash algorithm selected
 * @phash_available: flag indicating if a partial hash from a previous operation is available
 * @phash: buffer to store a partial hash from a previous operation
 * @phash_size: partial hash size of the hash algorithm selected
 * @digestcnt: stores the digest count from a previous operation
 * @data_buf: buffer to store part of input data to be carried over to next operation
 * @buflen: length of input data stored in data_buf
 * @flags: flags for internal use
 * @hash_compl: Completion variable for use in manual completion in case of DMA callback failure
 */
struct dthe_hash_ctx {
	enum dthe_hash_alg_sel mode;
	u16 block_size;
	u8 digest_size;
	u8 phash_available;
	u32 phash[SHA512_DIGEST_SIZE / sizeof(u32)];
	u32 phash_size;
	u32 digestcnt;
	u8 data_buf[SHA512_BLOCK_SIZE];
	u8 buflen;
	u8 flags;
	struct completion hash_compl;
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
		struct dthe_hash_ctx *hash_ctx;
	} ctx_info;
};

/* Struct definitions end */

struct dthe_data *dthe_get_dev(struct dthe_tfm_ctx *ctx);

int dthe_register_aes_algs(void);
void dthe_unregister_aes_algs(void);

int dthe_register_hash_algs(void);
void dthe_unregister_hash_algs(void);

#endif
