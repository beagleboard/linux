/*
 * Cryptographic API.
 *
 * Support for OMAP AES HW acceleration.
 *
 * Copyright (c) 2010 Nokia Corporation
 * Author: Dmitry Kasatkin <dmitry.kasatkin@nokia.com>
 * Copyright (c) 2011 Texas Instruments Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) "%20s: " fmt, __func__
#define prn(num) pr_debug(#num "=%d\n", num)
#define prx(num) pr_debug(#num "=%x\n", num)

#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/crypto.h>
#include <linux/interrupt.h>
#include <crypto/scatterwalk.h>
#include <crypto/aes.h>
#include <crypto/skcipher.h>
#include <crypto/internal/aead.h>
#include "omap-aes.h"

#define DEFAULT_AUTOSUSPEND_DELAY	1000

/* keep registered devices data here */
static LIST_HEAD(dev_list);
static DEFINE_SPINLOCK(list_lock);

static int aes_fallback_sz = 200;

#ifdef DEBUG
#define omap_aes_read(dd, offset)				\
({								\
	int _read_ret;						\
	_read_ret = __raw_readl(dd->io_base + offset);		\
	pr_debug("omap_aes_read(" #offset "=%#x)= %#x\n",	\
		 offset, _read_ret);				\
	_read_ret;						\
})
#else
inline u32 omap_aes_read(struct omap_aes_dev *dd, u32 offset)
{
	return __raw_readl(dd->io_base + offset);
}
#endif

#ifdef DEBUG
#define omap_aes_write(dd, offset, value)				\
	do {								\
		pr_debug("omap_aes_write(" #offset "=%#x) value=%#x\n",	\
			 offset, value);				\
		__raw_writel(value, dd->io_base + offset);		\
	} while (0)
#else
inline void omap_aes_write(struct omap_aes_dev *dd, u32 offset,
				  u32 value)
{
	__raw_writel(value, dd->io_base + offset);
}
#endif

static inline void omap_aes_write_mask(struct omap_aes_dev *dd, u32 offset,
					u32 value, u32 mask)
{
	u32 val;

	val = omap_aes_read(dd, offset);
	val &= ~mask;
	val |= value;
	omap_aes_write(dd, offset, val);
}

static void omap_aes_write_n(struct omap_aes_dev *dd, u32 offset,
					u32 *value, int count)
{
	for (; count--; value++, offset += 4)
		omap_aes_write(dd, offset, *value);
}

static int omap_aes_hw_init(struct omap_aes_dev *dd)
{
	int err;

	if (!(dd->flags & FLAGS_INIT)) {
		dd->flags |= FLAGS_INIT;
		dd->err = 0;
	}

	err = pm_runtime_get_sync(dd->dev);
	if (err < 0) {
		dev_err(dd->dev, "failed to get sync: %d\n", err);
		return err;
	}

	return 0;
}

int omap_aes_write_ctrl(struct omap_aes_dev *dd)
{
	struct omap_aes_reqctx *rctx;
	unsigned int key32;
	int i, err;
	u32 val;

	err = omap_aes_hw_init(dd);
	if (err)
		return err;

	key32 = dd->ctx->keylen / sizeof(u32);

	/* RESET the key as previous HASH keys should not get affected*/
	if (dd->flags & FLAGS_GCM)
		for (i = 0; i < 0x40; i = i + 4)
			omap_aes_write(dd, i, 0x0);

	for (i = 0; i < key32; i++) {
		omap_aes_write(dd, AES_REG_KEY(dd, i),
			__le32_to_cpu(dd->ctx->key[i]));
	}

	if ((dd->flags & (FLAGS_CBC | FLAGS_CTR)) && dd->req->info)
		omap_aes_write_n(dd, AES_REG_IV(dd, 0), dd->req->info, 4);

	if ((dd->flags & (FLAGS_GCM)) && dd->aead_req->iv) {
		rctx = aead_request_ctx(dd->aead_req);
		omap_aes_write_n(dd, AES_REG_IV(dd, 0), (u32 *)rctx->iv, 4);
	}

	val = FLD_VAL(((dd->ctx->keylen >> 3) - 1), 4, 3);
	if (dd->flags & FLAGS_CBC)
		val |= AES_REG_CTRL_CBC;

	if (dd->flags & (FLAGS_CTR | FLAGS_GCM))
		val |= AES_REG_CTRL_CTR | AES_REG_CTRL_CTR_WIDTH_128;

	if (dd->flags & FLAGS_GCM)
		val |= AES_REG_CTRL_GCM;

	if (dd->flags & FLAGS_ENCRYPT)
		val |= AES_REG_CTRL_DIRECTION;

	omap_aes_write_mask(dd, AES_REG_CTRL(dd), val, AES_REG_CTRL_MASK);

	return 0;
}

static void omap_aes_dma_trigger_omap2(struct omap_aes_dev *dd, int length)
{
	u32 mask, val;

	val = dd->pdata->dma_start;

	if (dd->dma_lch_out != NULL)
		val |= dd->pdata->dma_enable_out;
	if (dd->dma_lch_in != NULL)
		val |= dd->pdata->dma_enable_in;

	mask = dd->pdata->dma_enable_out | dd->pdata->dma_enable_in |
	       dd->pdata->dma_start;

	omap_aes_write_mask(dd, AES_REG_MASK(dd), val, mask);

}

static void omap_aes_dma_trigger_omap4(struct omap_aes_dev *dd, int length)
{
	omap_aes_write(dd, AES_REG_LENGTH_N(0), length);
	omap_aes_write(dd, AES_REG_LENGTH_N(1), 0);
	if (dd->flags & FLAGS_GCM)
		omap_aes_write(dd, AES_REG_A_LEN, dd->assoc_len);

	omap_aes_dma_trigger_omap2(dd, length);
}

static void omap_aes_dma_stop(struct omap_aes_dev *dd)
{
	u32 mask;

	mask = dd->pdata->dma_enable_out | dd->pdata->dma_enable_in |
	       dd->pdata->dma_start;

	omap_aes_write_mask(dd, AES_REG_MASK(dd), 0, mask);
}

struct omap_aes_dev *omap_aes_find_dev(struct omap_aes_reqctx *rctx)
{
	struct omap_aes_dev *dd;

	spin_lock_bh(&list_lock);
	dd = list_first_entry(&dev_list, struct omap_aes_dev, list);
	list_move_tail(&dd->list, &dev_list);
	rctx->dd = dd;
	spin_unlock_bh(&list_lock);

	return dd;
}

static void omap_aes_dma_out_callback(void *data)
{
	struct omap_aes_dev *dd = data;

	/* dma_lch_out - completed */
	tasklet_schedule(&dd->done_task);
}

static int omap_aes_dma_init(struct omap_aes_dev *dd)
{
	int err;

	dd->dma_lch_out = NULL;
	dd->dma_lch_in = NULL;

	dd->dma_lch_in = dma_request_chan(dd->dev, "rx");
	if (IS_ERR(dd->dma_lch_in)) {
		dev_err(dd->dev, "Unable to request in DMA channel\n");
		return PTR_ERR(dd->dma_lch_in);
	}

	dd->dma_lch_out = dma_request_chan(dd->dev, "tx");
	if (IS_ERR(dd->dma_lch_out)) {
		dev_err(dd->dev, "Unable to request out DMA channel\n");
		err = PTR_ERR(dd->dma_lch_out);
		goto err_dma_out;
	}

	return 0;

err_dma_out:
	dma_release_channel(dd->dma_lch_in);

	return err;
}

static void omap_aes_dma_cleanup(struct omap_aes_dev *dd)
{
	if (dd->pio_only)
		return;

	dma_release_channel(dd->dma_lch_out);
	dma_release_channel(dd->dma_lch_in);
}

static void sg_copy_buf(void *buf, struct scatterlist *sg,
			      unsigned int start, unsigned int nbytes, int out)
{
	struct scatter_walk walk;

	if (!nbytes)
		return;

	scatterwalk_start(&walk, sg);
	scatterwalk_advance(&walk, start);
	scatterwalk_copychunks(buf, &walk, nbytes, out);
	scatterwalk_done(&walk, out, 0);
}

static int omap_aes_crypt_dma(struct omap_aes_dev *dd,
			      struct scatterlist *in_sg,
			      struct scatterlist *out_sg,
			      int in_sg_len, int out_sg_len)
{
	struct dma_async_tx_descriptor *tx_in, *tx_out;
	struct dma_slave_config cfg;
	int ret;

	if (dd->pio_only) {
		scatterwalk_start(&dd->in_walk, dd->in_sg);
		scatterwalk_start(&dd->out_walk, dd->out_sg);

		/* Enable DATAIN interrupt and let it take
		   care of the rest */
		omap_aes_write(dd, AES_REG_IRQ_ENABLE(dd), 0x2);
		return 0;
	}

	dma_sync_sg_for_device(dd->dev, dd->in_sg, in_sg_len, DMA_TO_DEVICE);

	memset(&cfg, 0, sizeof(cfg));

	cfg.src_addr = dd->phys_base + AES_REG_DATA_N(dd, 0);
	cfg.dst_addr = dd->phys_base + AES_REG_DATA_N(dd, 0);
	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.src_maxburst = DST_MAXBURST;
	cfg.dst_maxburst = DST_MAXBURST;

	/* IN */
	ret = dmaengine_slave_config(dd->dma_lch_in, &cfg);
	if (ret) {
		dev_err(dd->dev, "can't configure IN dmaengine slave: %d\n",
			ret);
		return ret;
	}

	tx_in = dmaengine_prep_slave_sg(dd->dma_lch_in, in_sg, in_sg_len,
					DMA_MEM_TO_DEV,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx_in) {
		dev_err(dd->dev, "IN prep_slave_sg() failed\n");
		return -EINVAL;
	}

	/* No callback necessary */
	tx_in->callback_param = dd;

	/* OUT */
	ret = dmaengine_slave_config(dd->dma_lch_out, &cfg);
	if (ret) {
		dev_err(dd->dev, "can't configure OUT dmaengine slave: %d\n",
			ret);
		return ret;
	}

	tx_out = dmaengine_prep_slave_sg(dd->dma_lch_out, out_sg, out_sg_len,
					DMA_DEV_TO_MEM,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx_out) {
		dev_err(dd->dev, "OUT prep_slave_sg() failed\n");
		return -EINVAL;
	}

	if (dd->flags & FLAGS_GCM)
		tx_out->callback = omap_aes_gcm_dma_out_callback;
	else
		tx_out->callback = omap_aes_dma_out_callback;
	tx_out->callback_param = dd;

	dmaengine_submit(tx_in);
	dmaengine_submit(tx_out);

	dma_async_issue_pending(dd->dma_lch_in);
	dma_async_issue_pending(dd->dma_lch_out);

	/* start DMA */
	dd->pdata->trigger(dd, dd->total);

	return 0;
}

int omap_aes_crypt_dma_start(struct omap_aes_dev *dd)
{
	int err;

	pr_debug("total: %d\n", dd->total);

	if (!dd->pio_only) {
		err = dma_map_sg(dd->dev, dd->in_sg, dd->in_sg_len,
				 DMA_TO_DEVICE);
		if (!err) {
			dev_err(dd->dev, "dma_map_sg() error\n");
			return -EINVAL;
		}

		err = dma_map_sg(dd->dev, dd->out_sg, dd->out_sg_len,
				 DMA_FROM_DEVICE);
		if (!err) {
			dev_err(dd->dev, "dma_map_sg() error\n");
			return -EINVAL;
		}
	}

	err = omap_aes_crypt_dma(dd, dd->in_sg, dd->out_sg, dd->in_sg_len,
				 dd->out_sg_len);
	if (err && !dd->pio_only) {
		dma_unmap_sg(dd->dev, dd->in_sg, dd->in_sg_len, DMA_TO_DEVICE);
		dma_unmap_sg(dd->dev, dd->out_sg, dd->out_sg_len,
			     DMA_FROM_DEVICE);
	}

	return err;
}

static void omap_aes_finish_req(struct omap_aes_dev *dd, int err)
{
	struct ablkcipher_request *req = dd->req;

	pr_debug("err: %d\n", err);

	dd->flags &= ~FLAGS_BUSY;

	req->base.complete(&req->base, err);

	pm_runtime_mark_last_busy(dd->dev);
	pm_runtime_put_autosuspend(dd->dev);
}

int omap_aes_crypt_dma_stop(struct omap_aes_dev *dd)
{
	pr_debug("total: %d\n", dd->total);

	omap_aes_dma_stop(dd);


	return 0;
}

bool omap_aes_copy_needed(struct scatterlist *sg, int total)
{
	int len = 0;

	if (!IS_ALIGNED(total, AES_BLOCK_SIZE))
		return true;

	while (sg) {
		if (!IS_ALIGNED(sg->offset, 4))
			return true;
		if (!IS_ALIGNED(sg->length, AES_BLOCK_SIZE))
			return true;
#ifdef CONFIG_ZONE_DMA
		if (page_zonenum(sg_page(sg)) != ZONE_DMA)
			return true;
#endif

		len += sg->length;
		sg = sg_next(sg);

		if (len >= total)
			break;
	}

	if (len != total)
		return true;

	return false;
}

static int omap_aes_copy_sgs(struct omap_aes_dev *dd)
{
	void *buf_in, *buf_out;
	int pages, total;

	total = ALIGN(dd->total, AES_BLOCK_SIZE);
	pages = get_order(total);

	buf_in = (void *)__get_free_pages(GFP_ATOMIC, pages);
	buf_out = (void *)__get_free_pages(GFP_ATOMIC, pages);

	if (!buf_in || !buf_out) {
		pr_err("Couldn't allocated pages for unaligned cases.\n");
		return -1;
	}

	dd->orig_out = dd->out_sg;

	sg_copy_buf(buf_in, dd->in_sg, 0, dd->total, 0);

	sg_init_table(dd->in_sgl, 1);
	sg_set_buf(dd->in_sgl, buf_in, total);
	dd->in_sg = dd->in_sgl;

	sg_init_table(&dd->out_sgl, 1);
	sg_set_buf(&dd->out_sgl, buf_out, total);
	dd->out_sg = &dd->out_sgl;

	return 0;
}

static int omap_aes_handle_queue(struct omap_aes_dev *dd,
			       struct ablkcipher_request *req)
{
	struct crypto_async_request *async_req, *backlog;
	struct omap_aes_ctx *ctx;
	struct omap_aes_reqctx *rctx;
	unsigned long flags;
	int err, ret = 0, len;

	spin_lock_irqsave(&dd->lock, flags);
	if (req)
		ret = ablkcipher_enqueue_request(&dd->queue, req);
	if (dd->flags & FLAGS_BUSY) {
		spin_unlock_irqrestore(&dd->lock, flags);
		return ret;
	}
	backlog = crypto_get_backlog(&dd->queue);
	async_req = crypto_dequeue_request(&dd->queue);
	if (async_req)
		dd->flags |= FLAGS_BUSY;
	spin_unlock_irqrestore(&dd->lock, flags);

	if (!async_req)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	req = ablkcipher_request_cast(async_req);

	/* assign new request to device */
	dd->req = req;
	dd->total = req->nbytes;
	dd->total_save = req->nbytes;
	dd->in_sg = req->src;
	dd->out_sg = req->dst;

	if (omap_aes_copy_needed(dd->in_sg, dd->total) ||
	    omap_aes_copy_needed(dd->out_sg, dd->total)) {
		if (omap_aes_copy_sgs(dd))
			pr_err("Failed to copy SGs for unaligned cases\n");
		dd->sgs_copied = 1;
	} else {
		dd->sgs_copied = 0;
	}

	len = ALIGN(dd->total, AES_BLOCK_SIZE);
	dd->in_sg_len = scatterwalk_bytes_sglen(dd->in_sg, len);
	dd->out_sg_len = scatterwalk_bytes_sglen(dd->out_sg, len);
	BUG_ON(dd->in_sg_len < 0 || dd->out_sg_len < 0);

	rctx = ablkcipher_request_ctx(req);
	ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	rctx->mode &= FLAGS_MODE_MASK;
	dd->flags = (dd->flags & ~FLAGS_MODE_MASK) | rctx->mode;

	dd->ctx = ctx;
	rctx->dd = dd;

	err = omap_aes_write_ctrl(dd);

	if (!err)
		err = omap_aes_crypt_dma_start(dd);
	if (err) {
		/* aes_task will not finish it, so do it here */
		omap_aes_finish_req(dd, err);
		tasklet_schedule(&dd->queue_task);
	}

	return ret; /* return ret, which is enqueue return value */
}

static void omap_aes_done_task(unsigned long data)
{
	struct omap_aes_dev *dd = (struct omap_aes_dev *)data;
	void *buf_in, *buf_out;
	int pages, len;

	pr_debug("enter done_task\n");

	if (!dd->pio_only) {
		dma_sync_sg_for_device(dd->dev, dd->out_sg, dd->out_sg_len,
				       DMA_FROM_DEVICE);
		dma_unmap_sg(dd->dev, dd->in_sg, dd->in_sg_len, DMA_TO_DEVICE);
		dma_unmap_sg(dd->dev, dd->out_sg, dd->out_sg_len,
			     DMA_FROM_DEVICE);
		omap_aes_crypt_dma_stop(dd);
	}

	if (dd->sgs_copied) {
		buf_in = sg_virt(dd->in_sgl);
		buf_out = sg_virt(&dd->out_sgl);

		sg_copy_buf(buf_out, dd->orig_out, 0, dd->total_save, 1);

		len = ALIGN(dd->total_save, AES_BLOCK_SIZE);
		pages = get_order(len);
		free_pages((unsigned long)buf_in, pages);
		free_pages((unsigned long)buf_out, pages);
	}

	omap_aes_finish_req(dd, 0);
	omap_aes_handle_queue(dd, NULL);

	pr_debug("exit\n");
}

static void omap_aes_queue_task(unsigned long data)
{
	struct omap_aes_dev *dd = (struct omap_aes_dev *)data;

	omap_aes_handle_queue(dd, NULL);
}

static int omap_aes_crypt(struct ablkcipher_request *req, unsigned long mode)
{
	struct crypto_tfm *tfm =
		crypto_ablkcipher_tfm(crypto_ablkcipher_reqtfm(req));
	struct omap_aes_ctx *ctx = crypto_ablkcipher_ctx(
			crypto_ablkcipher_reqtfm(req));
	struct omap_aes_reqctx *rctx = ablkcipher_request_ctx(req);
	struct omap_aes_dev *dd;
	int ret;

	pr_debug("nbytes: %d, enc: %d, cbc: %d\n", req->nbytes,
		  !!(mode & FLAGS_ENCRYPT),
		  !!(mode & FLAGS_CBC));

	if (req->nbytes < aes_fallback_sz) {
		ablkcipher_request_set_tfm(req, ctx->fallback);

		if (mode & FLAGS_ENCRYPT)
			ret = crypto_ablkcipher_encrypt(req);
		else
			ret = crypto_ablkcipher_decrypt(req);
		ablkcipher_request_set_tfm(req, __crypto_ablkcipher_cast(tfm));
		return ret;
	}
	dd = omap_aes_find_dev(rctx);
	if (!dd)
		return -ENODEV;

	rctx->mode = mode;

	return omap_aes_handle_queue(dd, req);
}

/* ********************** ALG API ************************************ */

static int omap_aes_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
			   unsigned int keylen)
{
	struct omap_aes_ctx *ctx = crypto_ablkcipher_ctx(tfm);
	int ret;

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 &&
		   keylen != AES_KEYSIZE_256)
		return -EINVAL;

	pr_debug("enter, keylen: %d\n", keylen);

	memcpy(ctx->key, key, keylen);
	ctx->keylen = keylen;

	ctx->fallback->base.crt_flags &= ~CRYPTO_TFM_REQ_MASK;
	ctx->fallback->base.crt_flags |=
		tfm->base.crt_flags & CRYPTO_TFM_REQ_MASK;

	ret = crypto_ablkcipher_setkey(ctx->fallback, key, keylen);
	if (!ret)
		return 0;

	return 0;
}

static int omap_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	return omap_aes_crypt(req, FLAGS_ENCRYPT);
}

static int omap_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	return omap_aes_crypt(req, 0);
}

static int omap_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	return omap_aes_crypt(req, FLAGS_ENCRYPT | FLAGS_CBC);
}

static int omap_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	return omap_aes_crypt(req, FLAGS_CBC);
}

static int omap_aes_ctr_encrypt(struct ablkcipher_request *req)
{
	return omap_aes_crypt(req, FLAGS_ENCRYPT | FLAGS_CTR);
}

static int omap_aes_ctr_decrypt(struct ablkcipher_request *req)
{
	return omap_aes_crypt(req, FLAGS_CTR);
}

static int omap_aes_cra_init(struct crypto_tfm *tfm)
{
	const char *name = crypto_tfm_alg_name(tfm);
	const u32 flags = CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK;
	struct omap_aes_ctx *ctx = crypto_tfm_ctx(tfm);
	struct crypto_ablkcipher *blk;

	blk = crypto_alloc_ablkcipher(name, 0, flags);
	if (IS_ERR(blk))
		return PTR_ERR(blk);

	ctx->fallback = blk;

	tfm->crt_ablkcipher.reqsize = sizeof(struct omap_aes_reqctx);

	return 0;
}

static int omap_aes_gcm_cra_init(struct crypto_aead *tfm)
{
	struct omap_aes_ctx *ctx = crypto_aead_ctx(tfm);

	tfm->reqsize = sizeof(struct omap_aes_reqctx);
	ctx->ctr = crypto_alloc_skcipher("ecb(aes)", 0, 0);
	if (IS_ERR(ctx->ctr)) {
		pr_warn("could not load aes driver for encrypting IV\n");
		return PTR_ERR(ctx->ctr);
	}

	return 0;
}

static void omap_aes_cra_exit(struct crypto_tfm *tfm)
{
	struct omap_aes_ctx *ctx = crypto_tfm_ctx(tfm);

	if (ctx->fallback)
		crypto_free_ablkcipher(ctx->fallback);

	ctx->fallback = NULL;
}

static void omap_aes_gcm_cra_exit(struct crypto_aead *tfm)
{
	struct omap_aes_ctx *ctx = crypto_aead_ctx(tfm);

	omap_aes_cra_exit(crypto_aead_tfm(tfm));

	if (ctx->ctr)
		crypto_free_skcipher(ctx->ctr);
}

/* ********************** ALGS ************************************ */

static struct crypto_alg algs_ecb_cbc[] = {
{
	.cra_name		= "ecb(aes)",
	.cra_driver_name	= "ecb-aes-omap",
	.cra_priority		= 300,
	.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
				  CRYPTO_ALG_KERN_DRIVER_ONLY |
				  CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
	.cra_blocksize		= AES_BLOCK_SIZE,
	.cra_ctxsize		= sizeof(struct omap_aes_ctx),
	.cra_alignmask		= 0,
	.cra_type		= &crypto_ablkcipher_type,
	.cra_module		= THIS_MODULE,
	.cra_init		= omap_aes_cra_init,
	.cra_exit		= omap_aes_cra_exit,
	.cra_u.ablkcipher = {
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.setkey		= omap_aes_setkey,
		.encrypt	= omap_aes_ecb_encrypt,
		.decrypt	= omap_aes_ecb_decrypt,
	}
},
{
	.cra_name		= "cbc(aes)",
	.cra_driver_name	= "cbc-aes-omap",
	.cra_priority		= 300,
	.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
				  CRYPTO_ALG_KERN_DRIVER_ONLY |
				  CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
	.cra_blocksize		= AES_BLOCK_SIZE,
	.cra_ctxsize		= sizeof(struct omap_aes_ctx),
	.cra_alignmask		= 0,
	.cra_type		= &crypto_ablkcipher_type,
	.cra_module		= THIS_MODULE,
	.cra_init		= omap_aes_cra_init,
	.cra_exit		= omap_aes_cra_exit,
	.cra_u.ablkcipher = {
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.ivsize		= AES_BLOCK_SIZE,
		.setkey		= omap_aes_setkey,
		.encrypt	= omap_aes_cbc_encrypt,
		.decrypt	= omap_aes_cbc_decrypt,
	}
}
};

static struct crypto_alg algs_ctr[] = {
{
	.cra_name		= "ctr(aes)",
	.cra_driver_name	= "ctr-aes-omap",
	.cra_priority		= 300,
	.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
				  CRYPTO_ALG_KERN_DRIVER_ONLY |
				  CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
	.cra_blocksize		= AES_BLOCK_SIZE,
	.cra_ctxsize		= sizeof(struct omap_aes_ctx),
	.cra_alignmask		= 0,
	.cra_type		= &crypto_ablkcipher_type,
	.cra_module		= THIS_MODULE,
	.cra_init		= omap_aes_cra_init,
	.cra_exit		= omap_aes_cra_exit,
	.cra_u.ablkcipher = {
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.geniv		= "eseqiv",
		.ivsize		= AES_BLOCK_SIZE,
		.setkey		= omap_aes_setkey,
		.encrypt	= omap_aes_ctr_encrypt,
		.decrypt	= omap_aes_ctr_decrypt,
	}
} ,
};

static struct omap_aes_algs_info omap_aes_algs_info_ecb_cbc[] = {
	{
		.algs_list	= algs_ecb_cbc,
		.size		= ARRAY_SIZE(algs_ecb_cbc),
	},
};

static struct aead_alg algs_aead_gcm[] = {
{
	.base = {
		.cra_name		= "gcm(aes)",
		.cra_driver_name	= "gcm-aes-omap",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_ASYNC |
					  CRYPTO_ALG_KERN_DRIVER_ONLY,
		.cra_blocksize		= 1,
		.cra_ctxsize		= sizeof(struct omap_aes_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
	},
	.init		= omap_aes_gcm_cra_init,
	.exit		= omap_aes_gcm_cra_exit,
	.ivsize		= 12,
	.maxauthsize	= AES_BLOCK_SIZE,
	.setkey		= omap_aes_gcm_setkey,
	.encrypt	= omap_aes_gcm_encrypt,
	.decrypt	= omap_aes_gcm_decrypt,
},
{
	.base = {
		.cra_name		= "rfc4106(gcm(aes))",
		.cra_driver_name	= "rfc4106-gcm-aes-omap",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_ASYNC |
					  CRYPTO_ALG_KERN_DRIVER_ONLY,
		.cra_blocksize		= 1,
		.cra_ctxsize		= sizeof(struct omap_aes_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
	},
	.init		= omap_aes_gcm_cra_init,
	.exit		= omap_aes_gcm_cra_exit,
	.maxauthsize	= AES_BLOCK_SIZE,
	.ivsize		= 8,
	.setkey		= omap_aes_4106gcm_setkey,
	.encrypt	= omap_aes_4106gcm_encrypt,
	.decrypt	= omap_aes_4106gcm_decrypt,
},
};

static struct omap_aes_aead_algs omap_aes_aead_info = {
	.algs_list	=	algs_aead_gcm,
	.size		=	ARRAY_SIZE(algs_aead_gcm),
};

static const struct omap_aes_pdata omap_aes_pdata_omap2 = {
	.algs_info	= omap_aes_algs_info_ecb_cbc,
	.algs_info_size	= ARRAY_SIZE(omap_aes_algs_info_ecb_cbc),
	.trigger	= omap_aes_dma_trigger_omap2,
	.key_ofs	= 0x1c,
	.iv_ofs		= 0x20,
	.ctrl_ofs	= 0x30,
	.data_ofs	= 0x34,
	.rev_ofs	= 0x44,
	.mask_ofs	= 0x48,
	.dma_enable_in	= BIT(2),
	.dma_enable_out	= BIT(3),
	.dma_start	= BIT(5),
	.major_mask	= 0xf0,
	.major_shift	= 4,
	.minor_mask	= 0x0f,
	.minor_shift	= 0,
};

#ifdef CONFIG_OF
static struct omap_aes_algs_info omap_aes_algs_info_ecb_cbc_ctr[] = {
	{
		.algs_list	= algs_ecb_cbc,
		.size		= ARRAY_SIZE(algs_ecb_cbc),
	},
	{
		.algs_list	= algs_ctr,
		.size		= ARRAY_SIZE(algs_ctr),
	},
};

static const struct omap_aes_pdata omap_aes_pdata_omap3 = {
	.algs_info	= omap_aes_algs_info_ecb_cbc_ctr,
	.algs_info_size	= ARRAY_SIZE(omap_aes_algs_info_ecb_cbc_ctr),
	.trigger	= omap_aes_dma_trigger_omap2,
	.key_ofs	= 0x1c,
	.iv_ofs		= 0x20,
	.ctrl_ofs	= 0x30,
	.data_ofs	= 0x34,
	.rev_ofs	= 0x44,
	.mask_ofs	= 0x48,
	.dma_enable_in	= BIT(2),
	.dma_enable_out	= BIT(3),
	.dma_start	= BIT(5),
	.major_mask	= 0xf0,
	.major_shift	= 4,
	.minor_mask	= 0x0f,
	.minor_shift	= 0,
};

static const struct omap_aes_pdata omap_aes_pdata_omap4 = {
	.algs_info	= omap_aes_algs_info_ecb_cbc_ctr,
	.algs_info_size	= ARRAY_SIZE(omap_aes_algs_info_ecb_cbc_ctr),
	.aead_algs_info	= &omap_aes_aead_info,
	.trigger	= omap_aes_dma_trigger_omap4,
	.key_ofs	= 0x3c,
	.iv_ofs		= 0x40,
	.ctrl_ofs	= 0x50,
	.data_ofs	= 0x60,
	.rev_ofs	= 0x80,
	.mask_ofs	= 0x84,
	.irq_status_ofs = 0x8c,
	.irq_enable_ofs = 0x90,
	.dma_enable_in	= BIT(5),
	.dma_enable_out	= BIT(6),
	.major_mask	= 0x0700,
	.major_shift	= 8,
	.minor_mask	= 0x003f,
	.minor_shift	= 0,
};

static irqreturn_t omap_aes_irq(int irq, void *dev_id)
{
	struct omap_aes_dev *dd = dev_id;
	u32 status, i;
	u32 *src, *dst;

	status = omap_aes_read(dd, AES_REG_IRQ_STATUS(dd));
	if (status & AES_REG_IRQ_DATA_IN) {
		omap_aes_write(dd, AES_REG_IRQ_ENABLE(dd), 0x0);

		BUG_ON(!dd->in_sg);

		BUG_ON(_calc_walked(in) > dd->in_sg->length);

		src = sg_virt(dd->in_sg) + _calc_walked(in);

		for (i = 0; i < AES_BLOCK_WORDS; i++) {
			omap_aes_write(dd, AES_REG_DATA_N(dd, i), *src);

			scatterwalk_advance(&dd->in_walk, 4);
			if (dd->in_sg->length == _calc_walked(in)) {
				dd->in_sg = sg_next(dd->in_sg);
				if (dd->in_sg) {
					scatterwalk_start(&dd->in_walk,
							  dd->in_sg);
					src = sg_virt(dd->in_sg) +
					      _calc_walked(in);
				}
			} else {
				src++;
			}
		}

		/* Clear IRQ status */
		status &= ~AES_REG_IRQ_DATA_IN;
		omap_aes_write(dd, AES_REG_IRQ_STATUS(dd), status);

		/* Enable DATA_OUT interrupt */
		omap_aes_write(dd, AES_REG_IRQ_ENABLE(dd), 0x4);

	} else if (status & AES_REG_IRQ_DATA_OUT) {
		omap_aes_write(dd, AES_REG_IRQ_ENABLE(dd), 0x0);

		BUG_ON(!dd->out_sg);

		BUG_ON(_calc_walked(out) > dd->out_sg->length);

		dst = sg_virt(dd->out_sg) + _calc_walked(out);

		for (i = 0; i < AES_BLOCK_WORDS; i++) {
			*dst = omap_aes_read(dd, AES_REG_DATA_N(dd, i));
			scatterwalk_advance(&dd->out_walk, 4);
			if (dd->out_sg->length == _calc_walked(out)) {
				dd->out_sg = sg_next(dd->out_sg);
				if (dd->out_sg) {
					scatterwalk_start(&dd->out_walk,
							  dd->out_sg);
					dst = sg_virt(dd->out_sg) +
					      _calc_walked(out);
				}
			} else {
				dst++;
			}
		}

		dd->total -= min_t(size_t, AES_BLOCK_SIZE, dd->total);

		/* Clear IRQ status */
		status &= ~AES_REG_IRQ_DATA_OUT;
		omap_aes_write(dd, AES_REG_IRQ_STATUS(dd), status);

		if (!dd->total)
			/* All bytes read! */
			tasklet_schedule(&dd->done_task);
		else
			/* Enable DATA_IN interrupt for next block */
			omap_aes_write(dd, AES_REG_IRQ_ENABLE(dd), 0x2);
	}

	return IRQ_HANDLED;
}

static const struct of_device_id omap_aes_of_match[] = {
	{
		.compatible	= "ti,omap2-aes",
		.data		= &omap_aes_pdata_omap2,
	},
	{
		.compatible	= "ti,omap3-aes",
		.data		= &omap_aes_pdata_omap3,
	},
	{
		.compatible	= "ti,omap4-aes",
		.data		= &omap_aes_pdata_omap4,
	},
	{},
};
MODULE_DEVICE_TABLE(of, omap_aes_of_match);

static int omap_aes_get_res_of(struct omap_aes_dev *dd,
		struct device *dev, struct resource *res)
{
	struct device_node *node = dev->of_node;
	const struct of_device_id *match;
	int err = 0;

	match = of_match_device(of_match_ptr(omap_aes_of_match), dev);
	if (!match) {
		dev_err(dev, "no compatible OF match\n");
		err = -EINVAL;
		goto err;
	}

	err = of_address_to_resource(node, 0, res);
	if (err < 0) {
		dev_err(dev, "can't translate OF node address\n");
		err = -EINVAL;
		goto err;
	}

	dd->pdata = match->data;

err:
	return err;
}
#else
static const struct of_device_id omap_aes_of_match[] = {
	{},
};

static int omap_aes_get_res_of(struct omap_aes_dev *dd,
		struct device *dev, struct resource *res)
{
	return -EINVAL;
}
#endif

static int omap_aes_get_res_pdev(struct omap_aes_dev *dd,
		struct platform_device *pdev, struct resource *res)
{
	struct device *dev = &pdev->dev;
	struct resource *r;
	int err = 0;

	/* Get the base address */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(dev, "no MEM resource info\n");
		err = -ENODEV;
		goto err;
	}
	memcpy(res, r, sizeof(*res));

	/* Only OMAP2/3 can be non-DT */
	dd->pdata = &omap_aes_pdata_omap2;

err:
	return err;
}

static ssize_t fallback_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%d\n", aes_fallback_sz);
}

static ssize_t fallback_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t size)
{
	ssize_t status;
	long value;

	status = kstrtol(buf, 0, &value);
	if (status)
		return status;

	/* HW accelerator only works with buffers > 9 */
	if (value < 9) {
		dev_err(dev, "minimum fallback size 9\n");
		return -EINVAL;
	}

	aes_fallback_sz = value;

	return size;
}

static ssize_t queue_len_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct omap_aes_dev *dd = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", dd->queue.max_qlen);
}

static ssize_t queue_len_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t size)
{
	struct omap_aes_dev *dd;
	ssize_t status;
	long value;
	unsigned long flags;

	status = kstrtol(buf, 0, &value);
	if (status)
		return status;

	if (value < 0)
		return -EINVAL;

	/*
	 * Changing the queue size in fly is safe, if size becomes smaller
	 * than current size, it will just not accept new entries until
	 * it has shrank enough.
	 */
	spin_lock_bh(&list_lock);
	list_for_each_entry(dd, &dev_list, list) {
		spin_lock_irqsave(&dd->lock, flags);
		dd->queue.max_qlen = value;
		dd->aead_queue.base.max_qlen = value;
		spin_unlock_irqrestore(&dd->lock, flags);
	}
	spin_unlock_bh(&list_lock);

	return size;
}

static DEVICE_ATTR_RW(queue_len);
static DEVICE_ATTR_RW(fallback);

static struct attribute *omap_aes_attrs[] = {
	&dev_attr_queue_len.attr,
	&dev_attr_fallback.attr,
	NULL,
};

static struct attribute_group omap_aes_attr_group = {
	.attrs = omap_aes_attrs,
};

static DEFINE_MUTEX(probe_lock);

static int omap_aes_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct omap_aes_dev *dd;
	struct crypto_alg *algp;
	struct aead_alg *aalg;
	struct resource res;
	int err = -ENOMEM, i, j, irq = -1;
	u32 reg;

	dd = devm_kzalloc(dev, sizeof(struct omap_aes_dev), GFP_KERNEL);
	if (dd == NULL) {
		dev_err(dev, "unable to alloc data struct.\n");
		goto err_data;
	}
	dd->dev = dev;
	platform_set_drvdata(pdev, dd);

	spin_lock_init(&dd->lock);
	crypto_init_queue(&dd->queue, OMAP_AES_QUEUE_LENGTH);
	aead_init_queue(&dd->aead_queue, OMAP_AES_QUEUE_LENGTH);

	err = (dev->of_node) ? omap_aes_get_res_of(dd, dev, &res) :
			       omap_aes_get_res_pdev(dd, pdev, &res);
	if (err)
		goto err_res;

	dd->io_base = devm_ioremap_resource(dev, &res);
	if (IS_ERR(dd->io_base)) {
		err = PTR_ERR(dd->io_base);
		goto err_res;
	}
	dd->phys_base = res.start;

	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, DEFAULT_AUTOSUSPEND_DELAY);

	pm_runtime_enable(dev);
	err = pm_runtime_get_sync(dev);
	if (err < 0) {
		dev_err(dev, "%s: failed to get_sync(%d)\n",
			__func__, err);
		goto err_res;
	}

	omap_aes_dma_stop(dd);

	reg = omap_aes_read(dd, AES_REG_REV(dd));

	pm_runtime_put_sync(dev);

	dev_info(dev, "OMAP AES hw accel rev: %u.%u\n",
		 (reg & dd->pdata->major_mask) >> dd->pdata->major_shift,
		 (reg & dd->pdata->minor_mask) >> dd->pdata->minor_shift);

	tasklet_init(&dd->done_task, omap_aes_done_task, (unsigned long)dd);
	tasklet_init(&dd->queue_task, omap_aes_queue_task, (unsigned long)dd);

	err = omap_aes_dma_init(dd);
	if (err == -EPROBE_DEFER) {
		goto err_irq;
	} else if (err && AES_REG_IRQ_STATUS(dd) && AES_REG_IRQ_ENABLE(dd)) {
		dd->pio_only = 1;

		irq = platform_get_irq(pdev, 0);
		if (irq < 0) {
			dev_err(dev, "can't get IRQ resource\n");
			goto err_irq;
		}

		err = devm_request_irq(dev, irq, omap_aes_irq, 0,
				dev_name(dev), dd);
		if (err) {
			dev_err(dev, "Unable to grab omap-aes IRQ\n");
			goto err_irq;
		}
	}


	INIT_LIST_HEAD(&dd->list);
	spin_lock(&list_lock);
	list_add_tail(&dd->list, &dev_list);
	spin_unlock(&list_lock);

	mutex_lock(&probe_lock);

	for (i = 0; i < dd->pdata->algs_info_size; i++) {
		if (!dd->pdata->algs_info[i].registered) {
			for (j = 0; j < dd->pdata->algs_info[i].size; j++) {
				algp = &dd->pdata->algs_info[i].algs_list[j];

				pr_debug("reg alg: %s\n", algp->cra_name);
				INIT_LIST_HEAD(&algp->cra_list);

				err = crypto_register_alg(algp);
				if (err) {
					mutex_unlock(&probe_lock);
					goto err_algs;
				}

				dd->pdata->algs_info[i].registered++;
			}
		}
	}

	if (!dd->pdata->aead_algs_info->registered) {
		for (i = 0; i < dd->pdata->aead_algs_info->size; i++) {
			aalg = &dd->pdata->aead_algs_info->algs_list[i];
			algp = &aalg->base;

			pr_debug("reg alg: %s\n", algp->cra_name);
			INIT_LIST_HEAD(&algp->cra_list);

			err = crypto_register_aead(aalg);
			if (err) {
				mutex_unlock(&probe_lock);
				goto err_aead_algs;
			}

			dd->pdata->aead_algs_info->registered++;
		}
	}

	mutex_unlock(&probe_lock);

	err = sysfs_create_group(&dev->kobj, &omap_aes_attr_group);
	if (err) {
		dev_err(dev, "could not create sysfs device attrs\n");
		goto err_aead_algs;
	}

	return 0;
err_aead_algs:
	for (i = dd->pdata->aead_algs_info->registered - 1; i >= 0; i--) {
		aalg = &dd->pdata->aead_algs_info->algs_list[i];
		crypto_unregister_aead(aalg);
	}
err_algs:
	for (i = dd->pdata->algs_info_size - 1; i >= 0; i--)
		for (j = dd->pdata->algs_info[i].registered - 1; j >= 0; j--)
			crypto_unregister_alg(
					&dd->pdata->algs_info[i].algs_list[j]);

	omap_aes_dma_cleanup(dd);
err_irq:
	tasklet_kill(&dd->done_task);
	tasklet_kill(&dd->queue_task);
	pm_runtime_disable(dev);
err_res:
	dd = NULL;
err_data:
	dev_err(dev, "initialization failed.\n");
	return err;
}

static int omap_aes_remove(struct platform_device *pdev)
{
	struct omap_aes_dev *dd = platform_get_drvdata(pdev);
	struct aead_alg *aalg;
	int i, j;

	if (!dd)
		return -ENODEV;

	spin_lock(&list_lock);
	list_del(&dd->list);
	spin_unlock(&list_lock);

	for (i = dd->pdata->algs_info_size - 1; i >= 0; i--)
		for (j = dd->pdata->algs_info[i].registered - 1; j >= 0; j--)
			crypto_unregister_alg(
					&dd->pdata->algs_info[i].algs_list[j]);

	for (i = dd->pdata->aead_algs_info->size - 1; i >= 0; i--) {
		aalg = &dd->pdata->aead_algs_info->algs_list[i];
		crypto_unregister_aead(aalg);
	}

	tasklet_kill(&dd->done_task);
	tasklet_kill(&dd->queue_task);
	omap_aes_dma_cleanup(dd);
	pm_runtime_disable(dd->dev);
	dd = NULL;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int omap_aes_suspend(struct device *dev)
{
	pm_runtime_put_sync(dev);
	return 0;
}

static int omap_aes_resume(struct device *dev)
{
	pm_runtime_get_sync(dev);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(omap_aes_pm_ops, omap_aes_suspend, omap_aes_resume);

static struct platform_driver omap_aes_driver = {
	.probe	= omap_aes_probe,
	.remove	= omap_aes_remove,
	.driver	= {
		.name	= "omap-aes",
		.pm	= &omap_aes_pm_ops,
		.of_match_table	= omap_aes_of_match,
	},
};

module_platform_driver(omap_aes_driver);

MODULE_DESCRIPTION("OMAP AES hw acceleration support.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Dmitry Kasatkin");

