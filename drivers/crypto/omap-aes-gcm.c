/*
 * Cryptographic API.
 *
 * Support for OMAP AES GCM HW acceleration.
 *
 * Copyright (c) 2015 Texas Instruments Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/omap-dma.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/crypto.h>
#include <linux/interrupt.h>
#include <crypto/scatterwalk.h>
#include <crypto/aes.h>
#include "omap-aes.h"

static int omap_aes_gcm_handle_queue(struct omap_aes_dev *dd,
				     struct aead_request *req);

static void omap_aes_gcm_finish_req(struct omap_aes_dev *dd, int ret)
{
	struct aead_request *req = dd->aead_req;

	dd->flags &= ~FLAGS_BUSY;
	dd->in_sg = NULL;
	dd->out_sg = NULL;

	req->base.complete(&req->base, ret);
}

static void omap_aes_gcm_done_task(struct omap_aes_dev *dd)
{
	u8 *tag;
	int alen, clen, i, ret = 0, nsg;

	alen = ALIGN(dd->assoc_len, AES_BLOCK_SIZE);
	clen = ALIGN(dd->total, AES_BLOCK_SIZE);

	nsg = 1 + !!(dd->assoc_len && dd->total);

	if (!dd->pio_only) {
		dma_sync_sg_for_device(dd->dev, dd->out_sg, dd->out_sg_len,
				       DMA_FROM_DEVICE);
		dma_unmap_sg(dd->dev, dd->in_sg, dd->in_sg_len, DMA_TO_DEVICE);
		dma_unmap_sg(dd->dev, dd->out_sg, dd->out_sg_len,
			     DMA_FROM_DEVICE);
		omap_aes_crypt_dma_stop(dd);
	}

	if (dd->flags & FLAGS_ENCRYPT)
		scatterwalk_map_and_copy(dd->ctx->auth_tag, dd->aead_req->dst,
					 dd->total, dd->authsize, 1);

	if (!(dd->flags & FLAGS_ENCRYPT)) {
		tag = (u8 *)dd->ctx->auth_tag;
		for (i = 0; i < dd->authsize; i++) {
			if (tag[i]) {
				dev_err(dd->dev, "GCM decryption: Tag Message is wrong\n");
				ret = -EBADMSG;
			}
		}
	}

	omap_aes_gcm_finish_req(dd, ret);
	omap_aes_gcm_handle_queue(dd, NULL);
}

static int omap_aes_gcm_copy_buffers(struct omap_aes_dev *dd,
				     struct aead_request *req)
{
	void *buf_in;
	int alen, clen;
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	unsigned int authlen = crypto_aead_authsize(aead);
	u32 dec = !(dd->flags & FLAGS_ENCRYPT);

	alen = req->assoclen;
	clen = req->cryptlen - (dec * authlen);

	dd->sgs_copied = 0;

	sg_init_table(dd->in_sgl, 2);
	buf_in = sg_virt(req->assoc);
	sg_set_buf(dd->in_sgl, buf_in, alen);

	buf_in = sg_virt(req->src);
	sg_set_buf(&dd->in_sgl[1], buf_in, clen);

	dd->in_sg = dd->in_sgl;
	dd->total = clen;
	dd->assoc_len = req->assoclen;
	dd->authsize = authlen;
	dd->out_sg = req->dst;

	dd->in_sg_len = scatterwalk_bytes_sglen(dd->in_sg, alen + clen);
	dd->out_sg_len = scatterwalk_bytes_sglen(dd->out_sg, clen);

	return 0;
}

static void tcrypt_complete(struct crypto_async_request *req, int err)
{
	struct tcrypt_result *res = req->data;

	if (err == -EINPROGRESS)
		return;

	res->err = err;
	complete(&res->completion);
}

static int do_encrypt_iv(struct aead_request *req, u32 *tag)
{
	struct scatterlist iv_sg;
	struct ablkcipher_request *ablk_req;
	struct crypto_ablkcipher *tfm;
	struct tcrypt_result result;
	struct omap_aes_ctx *ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));
	int ret = 0;

	tfm = crypto_alloc_ablkcipher("ctr(aes)", 0, 0);
	ablk_req = ablkcipher_request_alloc(tfm, GFP_KERNEL);
	if (!ablk_req) {
		pr_err("skcipher: Failed to allocate request\n");
		return -1;
	}

	init_completion(&result.completion);

	sg_init_one(&iv_sg, tag, AES_BLOCK_SIZE);
	ablkcipher_request_set_callback(ablk_req, CRYPTO_TFM_REQ_MAY_BACKLOG,
					tcrypt_complete, &result);
	ret = crypto_ablkcipher_setkey(tfm, (u8 *)ctx->key, ctx->keylen);
	ablkcipher_request_set_crypt(ablk_req, &iv_sg, &iv_sg, AES_BLOCK_SIZE,
				     req->iv);
	ret = crypto_ablkcipher_encrypt(ablk_req);
	switch (ret) {
	case 0:
		break;
	case -EINPROGRESS:
	case -EBUSY:
		ret = wait_for_completion_interruptible(&result.completion);
		if (!ret) {
			ret = result.err;
			if (!ret) {
				reinit_completion(&result.completion);
				break;
			}
		}
		/* fall through */
	default:
		pr_err("Encryptio of IV failed for GCM mode");
		break;
	}

	ablkcipher_request_free(ablk_req);
	crypto_free_ablkcipher(tfm);
	return ret;
}

void omap_aes_gcm_dma_out_callback(void *data)
{
	struct omap_aes_dev *dd = data;
	int i, val;
	u32 *auth_tag, tag[4];

	if (!(dd->flags & FLAGS_ENCRYPT))
		scatterwalk_map_and_copy(tag, dd->aead_req->src, dd->total,
					 dd->authsize, 0);

	auth_tag = dd->ctx->auth_tag;
	for (i = 0; i < 4; i++) {
		val = omap_aes_read(dd, AES_REG_TAG_N(dd, i));
		auth_tag[i] = val ^ auth_tag[i];
		if (!(dd->flags & FLAGS_ENCRYPT))
			auth_tag[i] = auth_tag[i] ^ tag[i];
	}

	/* dma_lch_out - completed */
	omap_aes_gcm_done_task(dd);
}

static int omap_aes_gcm_handle_queue(struct omap_aes_dev *dd,
				     struct aead_request *req)
{
	struct omap_aes_ctx *ctx;
	struct crypto_async_request *async_req, *backlog;
	struct omap_aes_reqctx *rctx;
	unsigned long flags;
	int err, ret = 0;

	spin_lock_irqsave(&dd->lock, flags);
	if (req)
		ret = crypto_enqueue_request(&dd->aead_queue, &req->base);
	if (dd->flags & FLAGS_BUSY) {
		spin_unlock_irqrestore(&dd->lock, flags);
		return ret;
	}
	backlog = crypto_get_backlog(&dd->aead_queue);
	async_req = crypto_dequeue_request(&dd->aead_queue);
	if (async_req)
		dd->flags |= FLAGS_BUSY;
	spin_unlock_irqrestore(&dd->lock, flags);

	if (!async_req)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	req = aead_request_cast(async_req);

	ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));
	rctx = aead_request_ctx(req);

	dd->ctx = ctx;
	ctx->dd = dd;
	dd->aead_req = req;

	rctx->mode &= FLAGS_MODE_MASK;
	dd->flags = (dd->flags & ~FLAGS_MODE_MASK) | rctx->mode;

	err = omap_aes_gcm_copy_buffers(dd, req);
	if (err)
		return err;

	err = omap_aes_write_ctrl(dd);
	if (!err)
		err = omap_aes_crypt_dma_start(dd);

	if (err) {
		omap_aes_gcm_finish_req(dd, err);
		omap_aes_gcm_handle_queue(dd, NULL);
	}

	return ret;
}

static int omap_aes_gcm_crypt(struct aead_request *req, unsigned long mode)
{
	struct omap_aes_ctx *ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));
	struct omap_aes_reqctx *rctx = aead_request_ctx(req);
	struct omap_aes_dev *dd;
	__be32 counter = cpu_to_be32(1);
	int err;

	memset(ctx->auth_tag, 0, sizeof(ctx->auth_tag));
	memcpy(req->iv + 12, &counter, 4);

	/* Create E(K, IV) */
	err = do_encrypt_iv(req, ctx->auth_tag);
	if (err)
		return err;

	dd = omap_aes_find_dev(ctx);
	if (!dd)
		return -ENODEV;
	rctx->mode = mode;

	return omap_aes_gcm_handle_queue(dd, req);
}

int omap_aes_gcm_encrypt(struct aead_request *req)
{
	return omap_aes_gcm_crypt(req, FLAGS_ENCRYPT | FLAGS_GCM);
}

int omap_aes_gcm_decrypt(struct aead_request *req)
{
	return omap_aes_gcm_crypt(req, FLAGS_GCM);
}

int omap_aes_gcm_setkey(struct crypto_aead *tfm, const u8 *key,
			unsigned int keylen)
{
	struct omap_aes_ctx *ctx = crypto_aead_ctx(tfm);

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 &&
	    keylen != AES_KEYSIZE_256)
		return -EINVAL;

	memcpy(ctx->key, key, keylen);
	ctx->keylen = keylen;

	return 0;
}
