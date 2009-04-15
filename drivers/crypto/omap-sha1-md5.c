/*
 * Cryptographic API.
 *
 * Support for OMAP SHA1/MD5 HW acceleration.
 *
 * Copyright (c) 2007 Instituto Nokia de Tecnologia - INdT
 * Author: David Cohen <david.cohen@indt.org.br>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This driver is based on padlock-sha.c driver.
 */

#include <asm/arch-omap/irqs.h>
#include <crypto/algapi.h>
#include <crypto/sha.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/cryptohash.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>

#define SHA_REG_DIGEST(x)		(0x00 + ((x) * 0x04))
#define SHA_REG_DIN(x)			(0x1C + ((x) * 0x04))

#define SHA1_MD5_BLOCK_SIZE		SHA1_BLOCK_SIZE
#define MD5_DIGEST_SIZE			16

#define SHA_REG_DIGCNT			0x14

#define SHA_REG_CTRL			0x18
#define SHA_REG_CTRL_LENGTH		(0xFFFFFFFF << 5)
#define SHA_REG_CTRL_CLOSE_HASH		(1 << 4)
#define SHA_REG_CTRL_ALGO_CONST		(1 << 3)
#define SHA_REG_CTRL_ALGO		(1 << 2)
#define SHA_REG_CTRL_INPUT_READY	(1 << 1)
#define SHA_REG_CTRL_OUTPUT_READY	(1 << 0)

#define SHA_REG_REV			0x5C
#define SHA_REG_REV_MAJOR		0xF0
#define SHA_REG_REV_MINOR		0x0F

#define SHA_REG_MASK			0x60
#define SHA_REG_MASK_DMA_EN		(1 << 3)
#define SHA_REG_MASK_IT_EN		(1 << 2)
#define SHA_REG_MASK_SOFTRESET		(1 << 1)
#define SHA_REG_AUTOIDLE		(1 << 0)

#define SHA_REG_SYSSTATUS		0x64
#define SHA_REG_SYSSTATUS_RESETDONE	(1 << 0)

#define DRIVER_NAME			"OMAP SHA1/MD5"

struct omap_sha1_md5_ctx {
	unsigned int		type_algo;
	unsigned int		bufcnt;
	unsigned int		digcnt;
	int			algo_const;
	int			bypass;
	int			digsize;
	u8			hash[SHA1_DIGEST_SIZE];
	u8			buffer[SHA1_BLOCK_SIZE];
	struct			hash_desc fallback;
};

struct omap_sha1_md5_dev {
	unsigned long		base_address;
	int			irq;
	int			digready;
	struct clk		*sha1_ick;
	struct omap_sha1_md5_ctx
				*hw_ctx;
	struct device		*dev;
	wait_queue_head_t	wq;
};

static struct omap_sha1_md5_dev *sha1_md5_data;

#define SHA_REG_IOADDR(d, x) (void *)IO_ADDRESS((d)->base_address + (x))

static u32 omap_sha1_md5_read(struct omap_sha1_md5_dev *data, u32 offset)
{
	return __raw_readl(SHA_REG_IOADDR(data, offset));
}

static void omap_sha1_md5_write(struct omap_sha1_md5_dev *data,
					u32 value, u32 offset)
{
	__raw_writel(value, SHA_REG_IOADDR(data, offset));
}

static void omap_sha1_md5_write_mask(struct omap_sha1_md5_dev *data,
					u32 value, u32 mask, u32 address)
{
	u32 val;

	val = omap_sha1_md5_read(data, address);
	val &= ~mask;
	val |= value;
	omap_sha1_md5_write(data, val, address);
}

static inline void omap_sha1_md5_enable_clk(struct crypto_tfm *tfm)
{
	struct omap_sha1_md5_dev *data = sha1_md5_data;

	clk_enable(data->sha1_ick);
}

static inline void omap_sha1_md5_disable_clk(struct crypto_tfm *tfm)
{
	struct omap_sha1_md5_dev *data = sha1_md5_data;

	clk_disable(data->sha1_ick);
}

static void omap_sha1_md5_copy_hash(struct crypto_tfm *tfm)
{
	struct omap_sha1_md5_ctx *ctx = crypto_tfm_ctx(tfm);
	struct omap_sha1_md5_dev *data = sha1_md5_data;

	u32 *hash = (u32 *)ctx->hash;

	if (ctx->type_algo) {
		/* SHA1 results are in big endian */
		hash[0] = be32_to_cpu(
				omap_sha1_md5_read(data, SHA_REG_DIGEST(0)));
		hash[1] = be32_to_cpu(
				omap_sha1_md5_read(data, SHA_REG_DIGEST(1)));
		hash[2] = be32_to_cpu(
				omap_sha1_md5_read(data, SHA_REG_DIGEST(2)));
		hash[3] = be32_to_cpu(
				omap_sha1_md5_read(data, SHA_REG_DIGEST(3)));
		hash[4] = be32_to_cpu(
				omap_sha1_md5_read(data, SHA_REG_DIGEST(4)));
	} else {
		/* MD5 results are in little endian */
		hash[0] = le32_to_cpu(
				omap_sha1_md5_read(data, SHA_REG_DIGEST(0)));
		hash[1] = le32_to_cpu(
				omap_sha1_md5_read(data, SHA_REG_DIGEST(1)));
		hash[2] = le32_to_cpu(
				omap_sha1_md5_read(data, SHA_REG_DIGEST(2)));
		hash[3] = le32_to_cpu(
				omap_sha1_md5_read(data, SHA_REG_DIGEST(3)));
	}
}

static void omap_sha1_md5_bypass(struct crypto_tfm *tfm,
				u8 *data, unsigned int length)
{
	struct omap_sha1_md5_ctx *ctx = crypto_tfm_ctx(tfm);

	if (unlikely(!ctx->bypass))
		return;

	if (ctx->bypass == 1) {
		crypto_hash_init(&ctx->fallback);
		ctx->bypass++;
	}

	if (length) {
		struct scatterlist sg;

		sg_set_buf(&sg, data, length);
		crypto_hash_update(&ctx->fallback, &sg, sg.length);
	}
}

static void omap_sha1_md5_digest_buffer(struct crypto_tfm *tfm,
				u8 *buf, unsigned int len, int close_hash)
{
	struct omap_sha1_md5_ctx *ctx = crypto_tfm_ctx(tfm);
	struct omap_sha1_md5_dev *data = sha1_md5_data;
	unsigned int algo_const = 0;
	int c;
	u32 *buffer = (u32 *)buf;

	if (unlikely(ctx->bypass)) {
		omap_sha1_md5_bypass(tfm, buf, len);
		return;
	}

	if (unlikely(ctx->algo_const)) {
		algo_const = SHA_REG_CTRL_ALGO_CONST;
		ctx->algo_const = 0;
	} else
		omap_sha1_md5_write(data, ctx->digcnt, SHA_REG_DIGCNT);

	if (unlikely(close_hash))
		close_hash = SHA_REG_CTRL_CLOSE_HASH;

	/* Setting ALGO_CONST only for the first iteration
	 * and CLOSE_HASH only for the last one. */
	omap_sha1_md5_write_mask(data,
			ctx->type_algo | algo_const | close_hash | (len << 5),
			SHA_REG_CTRL_ALGO_CONST | SHA_REG_CTRL_CLOSE_HASH |
			SHA_REG_CTRL_ALGO | SHA_REG_CTRL_LENGTH,
			SHA_REG_CTRL);

	ctx->digcnt += len;
	while (!(omap_sha1_md5_read(data, SHA_REG_CTRL)
		& SHA_REG_CTRL_INPUT_READY));

	if (len % 4)
		len = (len/4) + 1;
	else
		len /= 4;
	for (c = 0; c < len; c++)
		omap_sha1_md5_write(data, buffer[c], SHA_REG_DIN(c));
}

static void omap_sha1_md5_append_buffer(struct crypto_tfm *tfm,
				const uint8_t *data, unsigned int length)
{
	struct omap_sha1_md5_ctx *ctx = crypto_tfm_ctx(tfm);

	BUG_ON((ctx->bufcnt + length) > SHA1_MD5_BLOCK_SIZE);

	memcpy(&ctx->buffer[ctx->bufcnt], data, length);
	ctx->bufcnt += length;
}

static void omap_sha1_md5_dia_update(struct crypto_tfm *tfm,
				const uint8_t *data, unsigned int length)
{
	struct omap_sha1_md5_ctx *ctx = crypto_tfm_ctx(tfm);

	/* We need to save the last buffer <= 64 to digest it with
	 * CLOSE_HASH = 1 */
	if (ctx->bufcnt && ((ctx->bufcnt + length) > SHA1_MD5_BLOCK_SIZE)) {
		unsigned int c = SHA1_MD5_BLOCK_SIZE - ctx->bufcnt;

		omap_sha1_md5_append_buffer(tfm, data, c);
		data += c;
		length -= c;
		if (length) {
			ctx->bufcnt = 0;
			omap_sha1_md5_digest_buffer(tfm, ctx->buffer,
					SHA1_MD5_BLOCK_SIZE, 0);
		}
	}

	while (length > SHA1_MD5_BLOCK_SIZE) {
		/* Revisit: use DMA here */
		omap_sha1_md5_digest_buffer(tfm, (u8 *)data,
				SHA1_MD5_BLOCK_SIZE, 0);
		length -= SHA1_MD5_BLOCK_SIZE;
		data += SHA1_MD5_BLOCK_SIZE;
	}

	if (length)
		omap_sha1_md5_append_buffer(tfm, data, length);
}

static void omap_sha1_md5_start_reset(struct crypto_tfm *tfm)
{
	struct omap_sha1_md5_dev *data = sha1_md5_data;

	omap_sha1_md5_write_mask(data, SHA_REG_MASK_SOFTRESET,
			SHA_REG_MASK_SOFTRESET, SHA_REG_MASK);
}

static void omap_sha1_md5_wait_reset(struct crypto_tfm *tfm)
{
	struct omap_sha1_md5_dev *data = sha1_md5_data;

	while (!(omap_sha1_md5_read(data, SHA_REG_SYSSTATUS)
			& SHA_REG_SYSSTATUS_RESETDONE));
}

static void omap_sha1_md5_dia_init(struct crypto_tfm *tfm)
{
	struct omap_sha1_md5_dev *data = sha1_md5_data;
	struct omap_sha1_md5_ctx *ctx = crypto_tfm_ctx(tfm);

	if (unlikely(data->hw_ctx))
		ctx->bypass = 1;
	else {
		data->hw_ctx = ctx;
		ctx->bypass = 0;
		omap_sha1_md5_enable_clk(tfm);
		omap_sha1_md5_start_reset(tfm);
		data->digready = 0;
	}

	if (ctx->bypass) {
		omap_sha1_md5_bypass(tfm, NULL, 0);
		return;
	}

	ctx->algo_const = 1;
	ctx->bufcnt = 0;
	ctx->digcnt = 0;

	omap_sha1_md5_wait_reset(tfm);
	omap_sha1_md5_write_mask(data, SHA_REG_MASK_IT_EN,
		SHA_REG_MASK_DMA_EN | SHA_REG_MASK_IT_EN, SHA_REG_MASK);
}

static void omap_sha1_md5_dia_final(struct crypto_tfm *tfm, uint8_t *out)
{
	struct omap_sha1_md5_ctx *ctx = crypto_tfm_ctx(tfm);
	struct omap_sha1_md5_dev *data = sha1_md5_data;
	int digsize = ctx->digsize;

	/* The buffer should be >= 9 */
	if (((ctx->digcnt + ctx->bufcnt) < 9) && !ctx->bypass)
		ctx->bypass = 1;

	omap_sha1_md5_digest_buffer(tfm, ctx->buffer, ctx->bufcnt, 1);

	if (unlikely(ctx->bypass)) {
		crypto_hash_final(&ctx->fallback, out);
		ctx->bypass = 0;
		goto bypass;
	} else
		data->digready = 1;

	wait_event_interruptible(data->wq, (data->digready == 2));
	omap_sha1_md5_copy_hash(tfm);

	memcpy(out, ctx->hash, digsize);

bypass:
	if (data->hw_ctx == ctx) {
		omap_sha1_md5_disable_clk(tfm);
		data->hw_ctx = NULL;
	}
}

static irqreturn_t omap_sha1_md5_irq(int irq, void *dev_id)
{
	struct omap_sha1_md5_dev *data = dev_id;

	omap_sha1_md5_write_mask(data, SHA_REG_CTRL_OUTPUT_READY,
			SHA_REG_CTRL_OUTPUT_READY, SHA_REG_CTRL);

	if (likely(!data->digready))
		return IRQ_HANDLED;

	if (data->hw_ctx == NULL) {
		dev_err(data->dev, "unknown interrupt.\n");
		return IRQ_HANDLED;
	}

	data->digready = 2;
	wake_up_interruptible(&data->wq);

	return IRQ_HANDLED;
}

static int omap_sha1_md5_cra_init(struct crypto_tfm *tfm)
{
	struct omap_sha1_md5_ctx *ctx = crypto_tfm_ctx(tfm);
	struct omap_sha1_md5_dev *data = sha1_md5_data;
	const char *fallback_driver_name = tfm->__crt_alg->cra_name;
	struct crypto_hash *fallback_tfm;

	/* Allocate a fallback and abort if it failed. */
	fallback_tfm = crypto_alloc_hash(fallback_driver_name, 0,
					 CRYPTO_ALG_ASYNC |
					 CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(fallback_tfm)) {
		dev_err(data->dev, "fallback driver '%s' could not be"
				"loaded.\n", fallback_driver_name);
		return PTR_ERR(fallback_tfm);
	}

	ctx->fallback.tfm = fallback_tfm;

	return 0;
}

static int omap_sha1_cra_init(struct crypto_tfm *tfm)
{
	struct omap_sha1_md5_ctx *ctx = crypto_tfm_ctx(tfm);

	ctx->type_algo = SHA_REG_CTRL_ALGO;
	ctx->digsize = SHA1_DIGEST_SIZE;

	return omap_sha1_md5_cra_init(tfm);
}

static int omap_md5_cra_init(struct crypto_tfm *tfm)
{
	struct omap_sha1_md5_ctx *ctx = crypto_tfm_ctx(tfm);

	ctx->type_algo = 0;
	ctx->digsize = MD5_DIGEST_SIZE;

	return omap_sha1_md5_cra_init(tfm);
}

static void omap_sha1_md5_cra_exit(struct crypto_tfm *tfm)
{
	struct omap_sha1_md5_ctx *ctx = crypto_tfm_ctx(tfm);

	crypto_free_hash(ctx->fallback.tfm);
	ctx->fallback.tfm = NULL;
}

static struct crypto_alg omap_sha1_alg = {
	.cra_name		=	"sha1",
	.cra_driver_name	=	"omap-sha1",
	.cra_flags		=	CRYPTO_ALG_TYPE_DIGEST |
					CRYPTO_ALG_NEED_FALLBACK,
	.cra_blocksize		=	SHA1_MD5_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct omap_sha1_md5_ctx),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(omap_sha1_alg.cra_list),
	.cra_init		=	omap_sha1_cra_init,
	.cra_exit		=	omap_sha1_md5_cra_exit,
	.cra_u			=	{
		.digest = {
			.dia_digestsize	=	SHA1_DIGEST_SIZE,
			.dia_init	=	omap_sha1_md5_dia_init,
			.dia_update	=	omap_sha1_md5_dia_update,
			.dia_final	=	omap_sha1_md5_dia_final,
		}
	}
};

static struct crypto_alg omap_md5_alg = {
	.cra_name		=	"md5",
	.cra_driver_name	=	"omap-md5",
	.cra_flags		=	CRYPTO_ALG_TYPE_DIGEST |
					CRYPTO_ALG_NEED_FALLBACK,
	.cra_blocksize		=	SHA1_MD5_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct omap_sha1_md5_ctx),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(omap_md5_alg.cra_list),
	.cra_init		=	omap_md5_cra_init,
	.cra_exit		=	omap_sha1_md5_cra_exit,
	.cra_u			=	{
		.digest = {
			.dia_digestsize	=	MD5_DIGEST_SIZE,
			.dia_init	=	omap_sha1_md5_dia_init,
			.dia_update	=	omap_sha1_md5_dia_update,
			.dia_final	=	omap_sha1_md5_dia_final,
		}
	}
};

static int omap_sha1_md5_probe(struct platform_device *pdev)
{
	struct omap_sha1_md5_dev *data;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int rc;

	rc = crypto_register_alg(&omap_sha1_alg);
	if (rc)
		goto sha1_err;
	rc = crypto_register_alg(&omap_md5_alg);
	if (rc)
		goto md5_err;

	data = kzalloc(sizeof(struct omap_sha1_md5_dev), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "unable to alloc data struct.\n");
		goto data_err;
	}
	platform_set_drvdata(pdev, data);
	data->dev = dev;

	/* Get the base address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "invalid resource type\n");
		rc = -ENODEV;
		goto res_err;
	}
	data->base_address = res->start;

	/* Set the private data */
	sha1_md5_data = data;

	/* Get the IRQ */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "invalid resource type\n");
		rc = -ENODEV;
		goto res_err;
	}
	data->irq = res->start;

	rc = request_irq(res->start, omap_sha1_md5_irq,
			IRQF_TRIGGER_LOW, DRIVER_NAME, data);
	if (rc) {
		dev_err(dev, "unable to request irq.\n");
		goto res_err;
	}

	/* Initializing the clock */
	data->sha1_ick = clk_get(0, "sha_ick");
	if (!data->sha1_ick) {
		dev_err(dev, "clock intialization failed.\n");
		rc = -ENODEV;
		goto clk_err;
	}

	init_waitqueue_head(&data->wq);

	dev_info(dev, "hw accel on OMAP rev %u.%u\n",
		(omap_sha1_md5_read(data, SHA_REG_REV) & SHA_REG_REV_MAJOR)>>4,
		omap_sha1_md5_read(data, SHA_REG_REV) & SHA_REG_REV_MINOR);

	return 0;

clk_err:
	free_irq(data->irq, data);
res_err:
	kfree(data);
data_err:
	crypto_unregister_alg(&omap_md5_alg);
md5_err:
	crypto_unregister_alg(&omap_sha1_alg);
sha1_err:
	dev_err(dev, "initialization failed.\n");
	return rc;
}

static int omap_sha1_md5_remove(struct platform_device *pdev)
{
	struct omap_sha1_md5_dev *data = platform_get_drvdata(pdev);

	free_irq(data->irq, data);
	kfree(data);
	crypto_unregister_alg(&omap_sha1_alg);
	crypto_unregister_alg(&omap_md5_alg);

	return 0;
}

static struct platform_driver omap_sha1_md5_driver = {
	.probe	= omap_sha1_md5_probe,
	.remove	= omap_sha1_md5_remove,
	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init omap_sha1_md5_init(void)
{
	int ret;

	ret = platform_driver_register(&omap_sha1_md5_driver);
	if (ret)
		return ret;

	return 0;
}

static void __exit omap_sha1_md5_exit(void)
{
	platform_driver_unregister(&omap_sha1_md5_driver);
}

module_init(omap_sha1_md5_init);
module_exit(omap_sha1_md5_exit);

MODULE_DESCRIPTION("OMAP SHA1/MD5 hw acceleration support.");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Cohen");
