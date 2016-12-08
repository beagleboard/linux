/*
 * Cryptographic API.
 *
 * Support for OMAP AES GCM HW acceleration.
 *
 * Copyright (c) 2016 Texas Instruments Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 */

#include <linux/errno.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/omap-dma.h>
#include <linux/interrupt.h>
#include <crypto/aes.h>
#include <crypto/scatterwalk.h>
#include <crypto/skcipher.h>
#include <crypto/internal/aead.h>
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
	void *buf;
	u8 *tag;
	int pages, alen, clen, i, ret = 0, nsg;
	struct omap_aes_reqctx *rctx;

	alen = ALIGN(dd->assoc_len, AES_BLOCK_SIZE);
	clen = ALIGN(dd->total, AES_BLOCK_SIZE);
	rctx = aead_request_ctx(dd->aead_req);

	nsg = !!(dd->assoc_len && dd->total);

	dma_sync_sg_for_device(dd->dev, dd->out_sg, dd->out_sg_len,
			       DMA_FROM_DEVICE);
	dma_unmap_sg(dd->dev, dd->in_sg, dd->in_sg_len, DMA_TO_DEVICE);
	dma_unmap_sg(dd->dev, dd->out_sg, dd->out_sg_len, DMA_FROM_DEVICE);
	omap_aes_crypt_dma_stop(dd);

	if (dd->sgs_copied & AES_OUT_DATA_COPIED) {
		buf = sg_virt(&dd->out_sgl);
		scatterwalk_map_and_copy(buf, dd->orig_out,
					 dd->aead_req->assoclen, dd->total, 1);

		pages = get_order(clen);
		free_pages((unsigned long)buf, pages);
	}

	if (dd->flags & FLAGS_ENCRYPT)
		scatterwalk_map_and_copy(rctx->auth_tag,
					 dd->aead_req->dst,
					 dd->total + dd->aead_req->assoclen,
					 dd->authsize, 1);

	if (dd->sgs_copied & AES_ASSOC_DATA_COPIED) {
		buf = sg_virt(&dd->in_sgl[0]);
		pages = get_order(alen);
		free_pages((unsigned long)buf, pages);
	}
	if (dd->sgs_copied & AES_IN_DATA_COPIED) {
		buf = sg_virt(&dd->in_sgl[nsg]);
		pages = get_order(clen);
		free_pages((unsigned long)buf, pages);
	}

	if (!(dd->flags & FLAGS_ENCRYPT)) {
		tag = (u8 *)rctx->auth_tag;
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
	int pages, alen, clen, cryptlen, nsg, assoclen;
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	unsigned int authlen = crypto_aead_authsize(aead);
	u32 dec = !(dd->flags & FLAGS_ENCRYPT);
	struct scatterlist *input, *assoc, tmp[2];

	if (dd->flags & FLAGS_RFC4106_GCM)
		assoclen = req->assoclen - 8;
	else
		assoclen = req->assoclen;
	alen = ALIGN(assoclen, AES_BLOCK_SIZE);
	cryptlen = req->cryptlen - (dec * authlen);
	clen = ALIGN(cryptlen, AES_BLOCK_SIZE);

	dd->sgs_copied = 0;

	nsg = !!(assoclen && req->cryptlen);

	assoc = &req->src[0];
	sg_init_table(dd->in_sgl, nsg + 1);
	if (assoclen) {
		if (omap_aes_copy_needed(assoc, assoclen)) {
			dd->sgs_copied |= AES_ASSOC_DATA_COPIED;
			pages = get_order(alen);
			buf_in = (void *)__get_free_pages(GFP_ATOMIC, pages);
			if (!buf_in) {
				pr_err("Couldn't allocate for unaligncases.\n");
				return -1;
			}

			scatterwalk_map_and_copy(buf_in, assoc, 0,
						 assoclen, 0);
			memset(buf_in + assoclen, 0, alen - assoclen);
		} else {
			buf_in = sg_virt(assoc);
		}
		sg_set_buf(dd->in_sgl, buf_in, alen);
	}

	if (req->cryptlen) {
		input = scatterwalk_ffwd(tmp, req->src, req->assoclen);

		if (omap_aes_copy_needed(input, req->cryptlen)) {
			dd->sgs_copied |= AES_IN_DATA_COPIED;
			pages = get_order(clen);
			buf_in = (void *)__get_free_pages(GFP_ATOMIC, pages);
			if (!buf_in) {
				pr_err("Couldn't allocate for unaligncases.\n");
				return -1;
			}

			scatterwalk_map_and_copy(buf_in, input, 0, cryptlen, 0);
			memset(buf_in + cryptlen, 0, clen - cryptlen);
		} else {
			buf_in = sg_virt(input);
		}
		sg_set_buf(&dd->in_sgl[nsg], buf_in, clen);
	}

	dd->in_sg = dd->in_sgl;
	dd->total = cryptlen;
	dd->assoc_len = assoclen;
	dd->authsize = authlen;

	if (omap_aes_copy_needed(req->dst, cryptlen + assoclen)) {
		pages = get_order(clen);

		buf_in = (void *)__get_free_pages(GFP_ATOMIC, pages);

		if (!buf_in) {
			pr_err("Couldn't allocate for unaligned cases.\n");
			return -1;
		}

		sg_init_one(&dd->out_sgl, buf_in, clen);
		dd->out_sg = &dd->out_sgl;
		dd->orig_out = req->dst;
		dd->sgs_copied |= AES_OUT_DATA_COPIED;
	} else {
		dd->out_sg = scatterwalk_ffwd(tmp, req->dst, req->assoclen);
	}

	dd->in_sg_len = scatterwalk_bytes_sglen(dd->in_sg, alen + clen);
	dd->out_sg_len = scatterwalk_bytes_sglen(dd->out_sg, clen);

	return 0;
}

static void omap_aes_gcm_complete(struct crypto_async_request *req, int err)
{
	struct omap_aes_gcm_result *res = req->data;

	if (err == -EINPROGRESS)
		return;

	res->err = err;
	complete(&res->completion);
}

static int do_encrypt_iv(struct aead_request *req, u32 *tag, u32 *iv)
{
	struct scatterlist iv_sg, tag_sg;
	struct skcipher_request *sk_req;
	struct omap_aes_gcm_result result;
	struct omap_aes_ctx *ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));
	int ret = 0;

	sk_req = skcipher_request_alloc(ctx->ctr, GFP_KERNEL);
	if (!sk_req) {
		pr_err("skcipher: Failed to allocate request\n");
		return -1;
	}

	init_completion(&result.completion);

	sg_init_one(&iv_sg, iv, AES_BLOCK_SIZE);
	sg_init_one(&tag_sg, tag, AES_BLOCK_SIZE);
	skcipher_request_set_callback(sk_req, CRYPTO_TFM_REQ_MAY_BACKLOG,
				      omap_aes_gcm_complete, &result);
	ret = crypto_skcipher_setkey(ctx->ctr, (u8 *)ctx->key, ctx->keylen);
	skcipher_request_set_crypt(sk_req, &iv_sg, &tag_sg, AES_BLOCK_SIZE,
				   NULL);
	ret = crypto_skcipher_encrypt(sk_req);
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

	skcipher_request_free(sk_req);
	return ret;
}

void omap_aes_gcm_dma_out_callback(void *data)
{
	struct omap_aes_dev *dd = data;
	struct omap_aes_reqctx *rctx;
	int i, val;
	u32 *auth_tag, tag[4];

	if (!(dd->flags & FLAGS_ENCRYPT))
		scatterwalk_map_and_copy(tag, dd->aead_req->src,
					 dd->total + dd->aead_req->assoclen,
					 dd->authsize, 0);

	rctx = aead_request_ctx(dd->aead_req);
	auth_tag = (u32 *)rctx->auth_tag;
	for (i = 0; i < 4; i++) {
		val = omap_aes_read(dd, AES_REG_TAG_N(dd, i));
		auth_tag[i] = val ^ auth_tag[i];
		if (!(dd->flags & FLAGS_ENCRYPT))
			auth_tag[i] = auth_tag[i] ^ tag[i];
	}

	omap_aes_gcm_done_task(dd);
}

static int omap_aes_gcm_handle_queue(struct omap_aes_dev *dd,
				     struct aead_request *req)
{
	struct omap_aes_ctx *ctx;
	struct aead_request *backlog;
	struct omap_aes_reqctx *rctx;
	unsigned long flags;
	int err, ret = 0;

	spin_lock_irqsave(&dd->lock, flags);
	if (req)
		ret = aead_enqueue_request(&dd->aead_queue, req);
	if (dd->flags & FLAGS_BUSY) {
		spin_unlock_irqrestore(&dd->lock, flags);
		return ret;
	}

	backlog = aead_get_backlog(&dd->aead_queue);
	req = aead_dequeue_request(&dd->aead_queue);
	if (req)
		dd->flags |= FLAGS_BUSY;
	spin_unlock_irqrestore(&dd->lock, flags);

	if (!req)
		return ret;

	if (backlog)
		backlog->base.complete(&backlog->base, -EINPROGRESS);

	ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));
	rctx = aead_request_ctx(req);

	dd->ctx = ctx;
	rctx->dd = dd;
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
	struct omap_aes_reqctx *rctx = aead_request_ctx(req);
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	unsigned int authlen = crypto_aead_authsize(aead);
	struct omap_aes_dev *dd;
	__be32 counter = cpu_to_be32(1);
	int err, assoclen;

	memset(rctx->auth_tag, 0, sizeof(rctx->auth_tag));
	memcpy(rctx->iv + 12, &counter, 4);

	err = do_encrypt_iv(req, (u32 *)rctx->auth_tag, (u32 *)rctx->iv);
	if (err)
		return err;

	if (mode & FLAGS_RFC4106_GCM)
		assoclen = req->assoclen - 8;
	else
		assoclen = req->assoclen;
	if (assoclen + req->cryptlen == 0) {
		scatterwalk_map_and_copy(rctx->auth_tag, req->dst, 0, authlen,
					 1);
		return 0;
	}

	dd = omap_aes_find_dev(rctx);
	if (!dd)
		return -ENODEV;
	rctx->mode = mode;

	return omap_aes_gcm_handle_queue(dd, req);
}

int omap_aes_gcm_encrypt(struct aead_request *req)
{
	struct omap_aes_reqctx *rctx = aead_request_ctx(req);

	memcpy(rctx->iv, req->iv, 12);
	return omap_aes_gcm_crypt(req, FLAGS_ENCRYPT | FLAGS_GCM);
}

int omap_aes_gcm_decrypt(struct aead_request *req)
{
	struct omap_aes_reqctx *rctx = aead_request_ctx(req);

	memcpy(rctx->iv, req->iv, 12);
	return omap_aes_gcm_crypt(req, FLAGS_GCM);
}

int omap_aes_4106gcm_encrypt(struct aead_request *req)
{
	struct omap_aes_ctx *ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));
	struct omap_aes_reqctx *rctx = aead_request_ctx(req);

	memcpy(rctx->iv, ctx->nonce, 4);
	memcpy(rctx->iv + 4, req->iv, 8);
	return omap_aes_gcm_crypt(req, FLAGS_ENCRYPT | FLAGS_GCM |
				  FLAGS_RFC4106_GCM);
}

int omap_aes_4106gcm_decrypt(struct aead_request *req)
{
	struct omap_aes_ctx *ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));
	struct omap_aes_reqctx *rctx = aead_request_ctx(req);

	memcpy(rctx->iv, ctx->nonce, 4);
	memcpy(rctx->iv + 4, req->iv, 8);
	return omap_aes_gcm_crypt(req, FLAGS_GCM | FLAGS_RFC4106_GCM);
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

int omap_aes_4106gcm_setkey(struct crypto_aead *tfm, const u8 *key,
			    unsigned int keylen)
{
	struct omap_aes_ctx *ctx = crypto_aead_ctx(tfm);

	if (keylen < 4)
		return -EINVAL;

	keylen -= 4;
	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 &&
	    keylen != AES_KEYSIZE_256)
		return -EINVAL;

	memcpy(ctx->key, key, keylen);
	memcpy(ctx->nonce, key + keylen, 4);
	ctx->keylen = keylen;

	return 0;
}
