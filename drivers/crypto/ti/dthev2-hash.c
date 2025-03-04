// SPDX-License-Identifier: GPL-2.0-only
/*
 * K3 DTHE V2 crypto accelerator driver
 *
 * Copyright (C) Texas Instruments 2025 - https://www.ti.com
 * Author: T Pratham <t-pratham@ti.com>
 */

#include <crypto/algapi.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <crypto/md5.h>
#include <crypto/sha2.h>

#include "dthev2-common.h"

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/scatterlist.h>

/* Registers */

// Hashing Engine
#define DTHE_P_HASH_BASE		0x5000
#define DTHE_P_HASH512_IDIGEST_A	0x0240
#define DTHE_P_HASH512_DIGEST_COUNT	0x0280
#define DTHE_P_HASH512_MODE		0x0284
#define DTHE_P_HASH512_LENGTH		0x0288
#define DTHE_P_HASH512_DATA_IN_START	0x0080
#define DTHE_P_HASH512_DATA_IN_END	0x00FC

#define DTHE_P_HASH_SYSCONFIG		0x0110
#define DTHE_P_HASH_IRQSTATUS		0x0118
#define DTHE_P_HASH_IRQENABLE		0x011C

/* Register write values and macros */
#define DTHE_HASH_SYSCONFIG_INT_EN		BIT(2)
#define DTHE_HASH_SYSCONFIG_DMA_EN		BIT(3)
#define DTHE_HASH_IRQENABLE_EN_ALL		GENMASK(3, 0)
#define DTHE_HASH_IRQSTATUS_OP_READY		BIT(0)
#define DTHE_HASH_IRQSTATUS_IP_READY		BIT(1)
#define DTHE_HASH_IRQSTATUS_PH_READY		BIT(2)
#define DTHE_HASH_IRQSTATUS_CTX_READY		BIT(3)

#define DTHE_HASH_MODE_USE_ALG_CONST		BIT(3)
#define DTHE_HASH_MODE_CLOSE_HASH		BIT(4)

/* Misc */
#define MD5_BLOCK_SIZE				(MD5_BLOCK_WORDS * 4)

enum dthe_hash_dma_callback_src {
	DMA_CALLBACK_FROM_UPDATE = 0,
	DMA_CALLBACK_FROM_FINAL,
	DMA_CALLBACK_FROM_FINUP,
};

static int cnt;

static struct scatterlist *dthe_set_src_sg(struct scatterlist *src, struct scatterlist *sg,
					   int nents, int buflen)
{
	struct scatterlist *from_sg, *to_sg;
	int sglen;

	sg_init_table(src, nents);

	for (to_sg = src, from_sg = sg; buflen && from_sg; buflen -= sglen) {
		sglen = from_sg->length;
		if (sglen > buflen)
			sglen = buflen;
		sg_set_buf(to_sg, sg_virt(from_sg), sglen);
		from_sg = sg_next(from_sg);
		to_sg = sg_next(to_sg);
	}

	return to_sg;
}

static void dthe_hash_write_zero_message(enum dthe_hash_alg_sel mode, void *dst)
{
	switch (mode) {
	case DTHE_HASH_SHA512:
		memcpy(dst, sha512_zero_message_hash, SHA512_DIGEST_SIZE);
		break;
	case DTHE_HASH_SHA384:
		memcpy(dst, sha384_zero_message_hash, SHA384_DIGEST_SIZE);
		break;
	case DTHE_HASH_SHA256:
		memcpy(dst, sha256_zero_message_hash, SHA256_DIGEST_SIZE);
		break;
	case DTHE_HASH_SHA224:
		memcpy(dst, sha224_zero_message_hash, SHA224_DIGEST_SIZE);
		break;
	case DTHE_HASH_MD5:
		memcpy(dst, md5_zero_message_hash, MD5_DIGEST_SIZE);
		break;
	default:
		break;
	}
}

static int dthe_sha512_cra_init(struct crypto_tfm *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_tfm_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);

	if (!dev_data)
		return -ENODEV;

	ctx->ctx_info.hash_ctx = kzalloc(sizeof(*ctx->ctx_info.hash_ctx), GFP_KERNEL);
	if (!ctx->ctx_info.hash_ctx)
		return -ENOMEM;

	ctx->ctx_info.hash_ctx->mode = DTHE_HASH_SHA512;
	ctx->ctx_info.hash_ctx->block_size = SHA512_BLOCK_SIZE;
	ctx->ctx_info.hash_ctx->digest_size = SHA512_DIGEST_SIZE;
	ctx->ctx_info.hash_ctx->phash_size = SHA512_DIGEST_SIZE;
	return 0;
}

static int dthe_sha384_cra_init(struct crypto_tfm *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_tfm_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);

	if (!dev_data)
		return -ENODEV;

	ctx->ctx_info.hash_ctx = kzalloc(sizeof(*ctx->ctx_info.hash_ctx), GFP_KERNEL);
	if (!ctx->ctx_info.hash_ctx)
		return -ENOMEM;

	ctx->ctx_info.hash_ctx->mode = DTHE_HASH_SHA384;
	ctx->ctx_info.hash_ctx->block_size = SHA384_BLOCK_SIZE;
	ctx->ctx_info.hash_ctx->digest_size = SHA384_DIGEST_SIZE;
	ctx->ctx_info.hash_ctx->phash_size = SHA512_DIGEST_SIZE;
	return 0;
}

static int dthe_sha256_cra_init(struct crypto_tfm *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_tfm_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);

	if (!dev_data)
		return -ENODEV;

	ctx->ctx_info.hash_ctx = kzalloc(sizeof(*ctx->ctx_info.hash_ctx), GFP_KERNEL);
	if (!ctx->ctx_info.hash_ctx)
		return -ENOMEM;

	ctx->ctx_info.hash_ctx->mode = DTHE_HASH_SHA256;
	ctx->ctx_info.hash_ctx->block_size = SHA256_BLOCK_SIZE;
	ctx->ctx_info.hash_ctx->digest_size = SHA256_DIGEST_SIZE;
	ctx->ctx_info.hash_ctx->phash_size = SHA256_DIGEST_SIZE;
	return 0;
}

static int dthe_sha224_cra_init(struct crypto_tfm *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_tfm_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);

	if (!dev_data)
		return -ENODEV;

	ctx->ctx_info.hash_ctx = kzalloc(sizeof(*ctx->ctx_info.hash_ctx), GFP_KERNEL);
	if (!ctx->ctx_info.hash_ctx)
		return -ENOMEM;

	ctx->ctx_info.hash_ctx->mode = DTHE_HASH_SHA224;
	ctx->ctx_info.hash_ctx->block_size = SHA224_BLOCK_SIZE;
	ctx->ctx_info.hash_ctx->digest_size = SHA224_DIGEST_SIZE;
	ctx->ctx_info.hash_ctx->phash_size = SHA256_DIGEST_SIZE;
	return 0;
}

static int dthe_md5_cra_init(struct crypto_tfm *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_tfm_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);

	if (!dev_data)
		return -ENODEV;

	ctx->ctx_info.hash_ctx = kzalloc(sizeof(*ctx->ctx_info.hash_ctx), GFP_KERNEL);
	if (!ctx->ctx_info.hash_ctx)
		return -ENOMEM;

	ctx->ctx_info.hash_ctx->mode = DTHE_HASH_MD5;
	ctx->ctx_info.hash_ctx->block_size = MD5_BLOCK_SIZE;
	ctx->ctx_info.hash_ctx->digest_size = MD5_DIGEST_SIZE;
	ctx->ctx_info.hash_ctx->phash_size = MD5_DIGEST_SIZE;
	return 0;
}

static void dthe_hash_cra_exit(struct crypto_tfm *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_tfm_ctx(tfm);

	kfree(ctx->ctx_info.hash_ctx);
}

static void dthe_hash_dma_in_callback(void *data)
{
	struct ahash_request *req = (struct ahash_request *)data;

	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	struct dthe_hash_ctx *sctx = ctx->ctx_info.hash_ctx;
	u32 *data_out;
	u32 out_len;

	void __iomem *sha_base_reg = dev_data->regs + DTHE_P_HASH_BASE;

	if (sctx->flags == DMA_CALLBACK_FROM_UPDATE) {
		// If coming from update, we need to read the phash and store it for future
		data_out = sctx->phash;
		out_len = sctx->phash_size / sizeof(u32);
	} else {
		// If coming from finup or final, we need to read the final digest
		data_out = (u32 *)req->result;
		out_len = sctx->digest_size / sizeof(u32);
	}

	for (int i = 0; i < out_len; ++i)
		data_out[i] = readl_relaxed(sha_base_reg +
					    DTHE_P_HASH512_IDIGEST_A +
					    (DTHE_REG_SIZE * i));

	sctx->digestcnt = readl_relaxed(sha_base_reg + DTHE_P_HASH512_DIGEST_COUNT);
	sctx->phash_available = 1;

	complete(&sctx->hash_compl);
}

static int dthe_hash_dma_start(struct ahash_request *req, struct scatterlist *src, size_t len)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	struct dthe_hash_ctx *sctx = ctx->ctx_info.hash_ctx;
	struct dma_slave_config cfg;
	struct device *tx_dev;
	struct dma_async_tx_descriptor *desc_out;
	int src_nents;
	int mapped_nents;
	enum dma_data_direction src_dir = DMA_TO_DEVICE;
	int ret = 0;
	void __iomem *sha_base_reg = dev_data->regs + DTHE_P_HASH_BASE;

	// Config SHA DMA channel as per SHA mode
	memzero_explicit(&cfg, sizeof(cfg));

	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_maxburst = sctx->block_size / 4;

	// HACK: Delay to workaround DMA driver issue
	cnt++;
	if (cnt % 50 == 0) {
		unsigned long delay = jiffies + usecs_to_jiffies(100);

		while (time_before(jiffies, delay))
			cond_resched();
		cnt = 0;
	}

	ret = dmaengine_slave_config(dev_data->dma_sha_tx, &cfg);
	if (ret) {
		dev_err(dev_data->dev, "Can't configure OUT2 dmaengine slave: %d\n", ret);
		goto hash_err;
	}

	tx_dev = dmaengine_get_dma_device(dev_data->dma_sha_tx);
	if (!tx_dev) {
		ret = -ENODEV;
		goto hash_err;
	}

	src_nents = sg_nents_for_len(src, len);
	mapped_nents = dma_map_sg(tx_dev, src, src_nents, src_dir);
	if (mapped_nents == 0) {
		ret = -EINVAL;
		goto hash_err;
	}

	desc_out = dmaengine_prep_slave_sg(dev_data->dma_sha_tx, src, mapped_nents,
					   DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc_out) {
		dev_err(dev_data->dev, "OUT prep_slave_sg() failed\n");
		ret = -EINVAL;
		goto hash_prep_err;
	}

	desc_out->callback = dthe_hash_dma_in_callback;
	desc_out->callback_param = req;

	init_completion(&sctx->hash_compl);

	dmaengine_submit(desc_out);

	dma_async_issue_pending(dev_data->dma_sha_tx);

	ret = wait_for_completion_timeout(&sctx->hash_compl,
					  msecs_to_jiffies(DTHE_DMA_TIMEOUT_MS));
	if (!ret) {
		u32 *data_out;
		u32 out_len;

		ret = -ETIMEDOUT;

		if (sctx->flags == DMA_CALLBACK_FROM_UPDATE) {
			data_out = sctx->phash;
			out_len = sctx->phash_size / sizeof(u32);
		} else {
			data_out = (u32 *)req->result;
			out_len = sctx->digest_size / sizeof(u32);
		}

		for (int i = 0; i < out_len; ++i)
			data_out[i] = readl_relaxed(sha_base_reg +
						    DTHE_P_HASH512_IDIGEST_A +
						    (DTHE_REG_SIZE * i));

		sctx->digestcnt = readl_relaxed(sha_base_reg + DTHE_P_HASH512_DIGEST_COUNT);
		sctx->phash_available = 1;
	} else {
		ret = 0;
	}

hash_prep_err:
	dma_unmap_sg(tx_dev, src, src_nents, src_dir);
hash_err:
	mutex_unlock(&dev_data->hash_mutex);
	ahash_request_complete(req, ret);

	if (sctx->flags == DMA_CALLBACK_FROM_FINUP)
		if ((req->nbytes + sctx->buflen) % sctx->block_size)
			kfree(sg_virt(&src[src_nents - 1]));
	kfree(src);
	return ret;
}

static int dthe_hash_init(struct ahash_request *req)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_data *dev_data = dthe_get_dev(ctx);

	void __iomem *sha_base_reg = dev_data->regs + DTHE_P_HASH_BASE;
	u32 sha_sysconfig_val = DTHE_HASH_SYSCONFIG_INT_EN | DTHE_HASH_SYSCONFIG_DMA_EN;

	ctx->ctx_info.hash_ctx->phash_available = 0;
	ctx->ctx_info.hash_ctx->buflen = 0;
	ctx->ctx_info.hash_ctx->digestcnt = 0;

	writel_relaxed(sha_sysconfig_val, sha_base_reg + DTHE_P_HASH_SYSCONFIG);
	writel_relaxed(DTHE_HASH_IRQENABLE_EN_ALL, sha_base_reg + DTHE_P_HASH_IRQENABLE);

	return 0;
}

static int dthe_hash_update(struct ahash_request *req)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	struct dthe_hash_ctx *sctx = ctx->ctx_info.hash_ctx;

	struct scatterlist *src;
	struct scatterlist *tmp;
	int src_nents = 0;
	int in_nents = sg_nents_for_len(req->src, req->nbytes);
	unsigned int tot_len, cur_len;
	unsigned int len_to_send, len_to_push;
	u32 hash_mode_val;
	int ret;

	void __iomem *sha_base_reg = dev_data->regs + DTHE_P_HASH_BASE;

	if (req->nbytes == 0) {
		if (!sctx->phash_available && !sctx->buflen)
			dthe_hash_write_zero_message(sctx->mode, sctx->phash);

		return 0;
	}

	tot_len = sctx->buflen + req->nbytes;
	len_to_send = tot_len - (tot_len % sctx->block_size);
	len_to_push = ((len_to_send == 0) ? req->nbytes : (tot_len % sctx->block_size));
	cur_len = 0;

	if (tot_len % sctx->block_size == 0) {
		len_to_send -= sctx->block_size;
		if (tot_len == sctx->block_size)
			len_to_push = req->nbytes;
		else
			len_to_push = sctx->block_size;
	}

	if (len_to_send == 0) {
		sg_copy_to_buffer(req->src, in_nents, sctx->data_buf + sctx->buflen, len_to_push);
		sctx->buflen += len_to_push;
		return 0;
	}

	if (len_to_push < req->nbytes)
		src_nents = sg_nents_for_len(req->src, req->nbytes - len_to_push);
	if (sctx->buflen > 0)
		src_nents++;

	src = kcalloc(src_nents, sizeof(struct scatterlist), GFP_KERNEL);
	if (!src)
		return -ENOMEM;

	tmp = src;

	if (sctx->buflen > 0) {
		sg_set_buf(tmp, sctx->data_buf, sctx->buflen);
		tmp = sg_next(tmp);
		cur_len += sctx->buflen;
		src_nents--;
	}
	if (src_nents > 0)
		dthe_set_src_sg(tmp, req->src, src_nents, len_to_send - cur_len);

	mutex_lock(&dev_data->hash_mutex);

	hash_mode_val = sctx->mode;
	if (sctx->phash_available) {
		for (int i = 0; i < sctx->phash_size / sizeof(u32); ++i)
			writel_relaxed(sctx->phash[i],
				       sha_base_reg +
				       DTHE_P_HASH512_IDIGEST_A +
				       (DTHE_REG_SIZE * i));

		writel_relaxed(sctx->digestcnt, sha_base_reg + DTHE_P_HASH512_DIGEST_COUNT);
	} else {
		hash_mode_val |= DTHE_HASH_MODE_USE_ALG_CONST;
	}

	writel_relaxed(hash_mode_val, sha_base_reg + DTHE_P_HASH512_MODE);
	writel_relaxed(len_to_send, sha_base_reg + DTHE_P_HASH512_LENGTH);

	sctx->flags = DMA_CALLBACK_FROM_UPDATE;
	ret = dthe_hash_dma_start(req, src, len_to_send);

	sg_pcopy_to_buffer(req->src, in_nents, sctx->data_buf,
			   len_to_push, req->nbytes - len_to_push);
	sctx->buflen = len_to_push;

	return ret;
}

static int dthe_hash_final(struct ahash_request *req)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	struct dthe_hash_ctx *sctx = ctx->ctx_info.hash_ctx;
	struct scatterlist *src;

	void __iomem *sha_base_reg = dev_data->regs + DTHE_P_HASH_BASE;
	u32 sha_mode_val = sctx->mode | DTHE_HASH_MODE_CLOSE_HASH;

	if (sctx->buflen > 0) {
		src = kzalloc(sizeof(struct scatterlist), GFP_KERNEL);
		if (!src)
			return -ENOMEM;

		/* Certain DMA restrictions forced us to send data in multiples of BLOCK_SIZE
		 * bytes. So, add a padding 0s at the end of src scatterlist if data is not a
		 * multiple of block_size bytes. The extra data is ignored by the DTHE hardware.
		 */
		for (int i = sctx->buflen; i < sctx->block_size; ++i)
			sctx->data_buf[i] = 0;

		sg_set_buf(src, sctx->data_buf, sctx->block_size);

		mutex_lock(&dev_data->hash_mutex);
		if (sctx->phash_available) {
			for (int i = 0; i < sctx->phash_size / sizeof(u32); ++i)
				writel_relaxed(sctx->phash[i],
					       sha_base_reg +
					       DTHE_P_HASH512_IDIGEST_A +
					       (DTHE_REG_SIZE * i));

			writel_relaxed(sctx->digestcnt,
				       sha_base_reg + DTHE_P_HASH512_DIGEST_COUNT);
		} else {
			sha_mode_val |= DTHE_HASH_MODE_USE_ALG_CONST;
		}

		writel_relaxed(sha_mode_val, sha_base_reg + DTHE_P_HASH512_MODE);
		writel_relaxed(sctx->buflen, sha_base_reg + DTHE_P_HASH512_LENGTH);

		sctx->flags = DMA_CALLBACK_FROM_FINAL;
		return dthe_hash_dma_start(req, src, sctx->block_size);
	} else if (!sctx->phash_available) {
		dthe_hash_write_zero_message(sctx->mode, req->result);
	}

	memcpy(req->result, sctx->phash, sctx->digest_size);

	ahash_request_complete(req, 0);
	return 0;
}

static int dthe_hash_finup(struct ahash_request *req)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	struct dthe_hash_ctx *sctx = ctx->ctx_info.hash_ctx;

	unsigned int tot_len = sctx->buflen + req->nbytes;
	unsigned int cur_len = 0;
	unsigned int pad_len = 0;
	struct scatterlist *src;
	struct scatterlist *tmp_sg;
	int src_nents = 0;
	u32 hash_mode_val;
	u8 *pad_buf;

	void __iomem *sha_base_reg = dev_data->regs + DTHE_P_HASH_BASE;

	if (tot_len == 0) {
		if (sctx->phash_available)
			memcpy(req->result, sctx->phash, sctx->digest_size);
		else
			dthe_hash_write_zero_message(sctx->mode, req->result);

		return 0;
	}

	if (tot_len % sctx->block_size)
		pad_len = sctx->block_size - (tot_len % sctx->block_size);

	if (req->nbytes > 0)
		src_nents = sg_nents_for_len(req->src, req->nbytes);
	if (sctx->buflen > 0)
		src_nents++;
	if (pad_len > 0)
		src_nents++;

	src = kcalloc(src_nents, sizeof(struct scatterlist), GFP_KERNEL);
	if (!src)
		return -ENOMEM;

	tmp_sg = src;

	if (sctx->buflen > 0) {
		sg_set_buf(tmp_sg, sctx->data_buf, sctx->buflen);
		tmp_sg = sg_next(tmp_sg);
		cur_len += sctx->buflen;
		src_nents--;
	}
	if (tot_len - cur_len > 0)
		tmp_sg = dthe_set_src_sg(tmp_sg, req->src, src_nents, tot_len - cur_len);

	/* Padding 0s in an additional nent at the end. See comment in dthe_hash_final function */
	if (pad_len > 0) {
		pad_buf = kcalloc(pad_len, sizeof(u8), GFP_KERNEL);
		if (!pad_buf) {
			kfree(src);
			return -ENOMEM;
		}
		sg_set_buf(tmp_sg, pad_buf, pad_len);
	}

	mutex_lock(&dev_data->hash_mutex);

	hash_mode_val = sctx->mode | DTHE_HASH_MODE_CLOSE_HASH;
	if (!sctx->phash_available) {
		hash_mode_val |= DTHE_HASH_MODE_USE_ALG_CONST;
	} else {
		for (int i = 0; i < sctx->phash_size / sizeof(u32); ++i)
			writel_relaxed(sctx->phash[i],
				       sha_base_reg +
				       DTHE_P_HASH512_IDIGEST_A +
				       (DTHE_REG_SIZE * i));

		writel_relaxed(sctx->digestcnt,
			       sha_base_reg + DTHE_P_HASH512_DIGEST_COUNT);
	}

	writel_relaxed(hash_mode_val, sha_base_reg + DTHE_P_HASH512_MODE);
	writel_relaxed(tot_len, sha_base_reg + DTHE_P_HASH512_LENGTH);

	sctx->flags = DMA_CALLBACK_FROM_FINUP;
	return dthe_hash_dma_start(req, src, tot_len + pad_len);
}

static int dthe_hash_digest(struct ahash_request *req)
{
	dthe_hash_init(req);
	return dthe_hash_finup(req);
}

static int dthe_hash_export(struct ahash_request *req, void *out)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));

	memcpy(out, ctx->ctx_info.hash_ctx, sizeof(struct dthe_hash_ctx));
	return 0;
}

static int dthe_hash_import(struct ahash_request *req, const void *in)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));

	memcpy(ctx->ctx_info.hash_ctx, in, sizeof(struct dthe_hash_ctx));
	return 0;
}

static struct ahash_alg hash_algs[] = {
	{
		.init	= dthe_hash_init,
		.update	= dthe_hash_update,
		.final	= dthe_hash_final,
		.finup	= dthe_hash_finup,
		.digest	= dthe_hash_digest,
		.export = dthe_hash_export,
		.import = dthe_hash_import,
		.halg	= {
			.digestsize = SHA512_DIGEST_SIZE,
			.statesize = sizeof(struct dthe_hash_ctx),
			.base = {
				.cra_name	 = "sha512",
				.cra_driver_name = "sha512-dthev2",
				.cra_priority	 = 400,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
						   CRYPTO_ALG_ASYNC |
						   CRYPTO_ALG_OPTIONAL_KEY |
						   CRYPTO_ALG_KERN_DRIVER_ONLY |
						   CRYPTO_ALG_ALLOCATES_MEMORY,
				.cra_blocksize	 = SHA512_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct dthe_tfm_ctx),
				.cra_module	 = THIS_MODULE,
				.cra_init	 = dthe_sha512_cra_init,
				.cra_exit	 = dthe_hash_cra_exit,
			}
		}
	},
	{
		.init	= dthe_hash_init,
		.update	= dthe_hash_update,
		.final	= dthe_hash_final,
		.finup	= dthe_hash_finup,
		.digest	= dthe_hash_digest,
		.export = dthe_hash_export,
		.import = dthe_hash_import,
		.halg	= {
			.digestsize = SHA384_DIGEST_SIZE,
			.statesize = sizeof(struct dthe_hash_ctx),
			.base = {
				.cra_name	 = "sha384",
				.cra_driver_name = "sha384-dthev2",
				.cra_priority	 = 400,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
						   CRYPTO_ALG_ASYNC |
						   CRYPTO_ALG_OPTIONAL_KEY |
						   CRYPTO_ALG_KERN_DRIVER_ONLY |
						   CRYPTO_ALG_ALLOCATES_MEMORY,
				.cra_blocksize	 = SHA384_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct dthe_tfm_ctx),
				.cra_module	 = THIS_MODULE,
				.cra_init	 = dthe_sha384_cra_init,
				.cra_exit	 = dthe_hash_cra_exit,
			}
		}
	},
	{
		.init	= dthe_hash_init,
		.update	= dthe_hash_update,
		.final	= dthe_hash_final,
		.finup	= dthe_hash_finup,
		.digest	= dthe_hash_digest,
		.export = dthe_hash_export,
		.import = dthe_hash_import,
		.halg	= {
			.digestsize = SHA256_DIGEST_SIZE,
			.statesize = sizeof(struct dthe_hash_ctx),
			.base = {
				.cra_name	 = "sha256",
				.cra_driver_name = "sha256-dthev2",
				.cra_priority	 = 400,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
						   CRYPTO_ALG_ASYNC |
						   CRYPTO_ALG_OPTIONAL_KEY |
						   CRYPTO_ALG_KERN_DRIVER_ONLY |
						   CRYPTO_ALG_ALLOCATES_MEMORY,
				.cra_blocksize	 = SHA256_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct dthe_tfm_ctx),
				.cra_module	 = THIS_MODULE,
				.cra_init	 = dthe_sha256_cra_init,
				.cra_exit	 = dthe_hash_cra_exit,
			}
		}
	},
	{
		.init	= dthe_hash_init,
		.update	= dthe_hash_update,
		.final	= dthe_hash_final,
		.finup	= dthe_hash_finup,
		.digest	= dthe_hash_digest,
		.export = dthe_hash_export,
		.import = dthe_hash_import,
		.halg	= {
			.digestsize = SHA224_DIGEST_SIZE,
			.statesize = sizeof(struct dthe_hash_ctx),
			.base = {
				.cra_name	 = "sha224",
				.cra_driver_name = "sha224-dthev2",
				.cra_priority	 = 400,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
						   CRYPTO_ALG_ASYNC |
						   CRYPTO_ALG_OPTIONAL_KEY |
						   CRYPTO_ALG_KERN_DRIVER_ONLY |
						   CRYPTO_ALG_ALLOCATES_MEMORY,
				.cra_blocksize	 = SHA224_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct dthe_tfm_ctx),
				.cra_module	 = THIS_MODULE,
				.cra_init	 = dthe_sha224_cra_init,
				.cra_exit	 = dthe_hash_cra_exit,
			}
		}
	},
	{
		.init	= dthe_hash_init,
		.update	= dthe_hash_update,
		.final	= dthe_hash_final,
		.finup	= dthe_hash_finup,
		.digest	= dthe_hash_digest,
		.export = dthe_hash_export,
		.import = dthe_hash_import,
		.halg	= {
			.digestsize = MD5_DIGEST_SIZE,
			.statesize = sizeof(struct dthe_hash_ctx),
			.base = {
				.cra_name	 = "md5",
				.cra_driver_name = "md5-dthev2",
				.cra_priority	 = 400,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
						   CRYPTO_ALG_ASYNC |
						   CRYPTO_ALG_OPTIONAL_KEY |
						   CRYPTO_ALG_KERN_DRIVER_ONLY |
						   CRYPTO_ALG_ALLOCATES_MEMORY,
				.cra_blocksize	 = MD5_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct dthe_tfm_ctx),
				.cra_module	 = THIS_MODULE,
				.cra_init	 = dthe_md5_cra_init,
				.cra_exit	 = dthe_hash_cra_exit,
			}
		}
	},
};

int dthe_register_hash_algs(void)
{
	return crypto_register_ahashes(hash_algs, ARRAY_SIZE(hash_algs));
}

void dthe_unregister_hash_algs(void)
{
	crypto_unregister_ahashes(hash_algs, ARRAY_SIZE(hash_algs));
}
