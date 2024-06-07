// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) Texas Instruments 2023 - http://www.ti.com
 * Author: Kamlesh Gurudasani <kamlesh@ti.com>
 */

#include <linux/crc64.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <crypto/internal/hash.h>

#include <asm/unaligned.h>

#define DRIVER_NAME		"mcrc64"
#define CHKSUM_DIGEST_SIZE	8
#define CHKSUM_BLOCK_SIZE	1

/* Registers */
#define CRC_CTRL0 0x0000 /* CRC Global Control Register 0 */
#define CH_PSA_SWRE(ch) BIT(((ch) - 1) << 3) /* PSA Software Reset  */

#define CRC_CTRL1 0x0008 /* CRC Global Control Register 1 */
#define PWDN BIT(0) /* Power Down  */

#define CRC_CTRL2 0x0010 /* CRC Global Control Register 2 */
#define CH_MODE(ch, m) ((m) << (((ch) - 1) << 3))

#define PSA_SIGREGL(ch) ((0x6 + (4 * ((ch) - 1))) << 4) /* Signature register */

#define MCRC64_ALG_MASK 0x8000000000000000
#define MCRC64_CRC64_POLY 0x000000000000001b

#define MCRC64_AUTOSUSPEND_DELAY 50

enum mcrc64_mode {
	MCRC64_MODE_DATA_CAPTURE = 0,
	MCRC64_MODE_AUTO,
	MCRC64_MODE_SEMI_CPU,
	MCRC64_MODE_FULL_CPU,
	MCRC64_MODE_INVALID,
};

enum mcrc64_channel {
	MCRC64_CHANNEL_1 = 1,
	MCRC64_CHANNEL_2,
	MCRC64_CHANNEL_3,
	MCRC64_CHANNEL_4,
	MCRC64_CHANNEL_INVALID,
};

struct mcrc64_data {
	struct list_head list;
	struct device *dev;
	void __iomem *regs;
};

struct mcrc64_list {
	struct list_head dev_list;
	spinlock_t lock; /* protect dev_list */
};

static struct mcrc64_list mcrc64_dev_list = {
	.dev_list = LIST_HEAD_INIT(mcrc64_dev_list.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(mcrc64_dev_list.lock),
};

struct mcrc64_tfm_ctx {
	struct mcrc64_data *dev_data;
	u64 key;
};

struct mcrc64_desc_ctx {
	u64 signature;
};

static struct mcrc64_data *mcrc64_get_dev(struct mcrc64_tfm_ctx *tctx)
{
	struct mcrc64_data *dev_data;

	if (tctx->dev_data)
		return tctx->dev_data;

	spin_lock_bh(&mcrc64_dev_list.lock);
	dev_data = list_first_entry(&mcrc64_dev_list.dev_list, struct mcrc64_data, list);
	if (dev_data)
		list_move_tail(&dev_data->list, &mcrc64_dev_list.dev_list);
	spin_unlock_bh(&mcrc64_dev_list.lock);

	return dev_data;
}

static int mcrc64_set_mode(void __iomem *regs, u32 channel, u32 mode)
{
	u32 mode_set_val;
	u32 crc_ctrl2_reg = 0;

	if (mode < 0 || mode >= MCRC64_MODE_INVALID)
		return -EINVAL;

	if (channel <= 0 || channel >= MCRC64_CHANNEL_INVALID)
		return -EINVAL;

	mode_set_val = crc_ctrl2_reg | CH_MODE(channel, mode);

	/* Write CRC_CTRL2, set mode */
	writel_relaxed(mode_set_val, regs + CRC_CTRL2);

	return 0;
}

static int mcrc64_reset_signature(void __iomem *regs, u32 channel)
{
	u32 crc_ctrl0_reg, reset_val, reset_undo_val;

	if (channel <= 0 || channel >= MCRC64_CHANNEL_INVALID)
		return -EINVAL;

	/* reset PSA */
	crc_ctrl0_reg = readl_relaxed(regs + CRC_CTRL0);

	reset_val = crc_ctrl0_reg | CH_PSA_SWRE(channel);
	reset_undo_val = crc_ctrl0_reg & ~CH_PSA_SWRE(channel);

	/* Write CRC_CTRL0 register, reset PSA register */
	writel_relaxed(reset_val, regs + CRC_CTRL0);
	writel_relaxed(reset_undo_val, regs + CRC_CTRL0);

	return 0;
}

/* This helper implements crc64 calculation using CPU */
static u64 mcrc64_calculate_sw_crc(u64 crc, u8 byte)
{
	u64 bit = 0;
	u8 j;

	for (j = 0; j < 8; j++) {
		bit = crc & MCRC64_ALG_MASK;
		crc <<= 1;
		if (byte & (0x80 >> j))
			bit ^= MCRC64_ALG_MASK;
		if (bit)
			crc ^= MCRC64_CRC64_POLY;
	}

	return crc;
}

static int mcrc64_calculate_crc(void __iomem *regs, u32 channel,
				const u8 *d8, size_t length, u64 *crc64)
{
	void __iomem *psa_reg;
	u64 signature = 0;

	if (channel <= 0 || channel >= MCRC64_CHANNEL_INVALID)
		return -EINVAL;

	psa_reg = regs + PSA_SIGREGL(channel);

	for (; length >= sizeof(u64); d8 += sizeof(u64), length -= sizeof(u64))
		writeq_relaxed(cpu_to_be64p((u64 *)d8), psa_reg);

	signature = readq_relaxed(psa_reg);

	if (length) {
		while (length--)
			signature = mcrc64_calculate_sw_crc(signature, *d8++);

		/* set capture mode */
		int ret = mcrc64_set_mode(regs, MCRC64_CHANNEL_1,
					MCRC64_MODE_DATA_CAPTURE);
		if (ret)
			return ret;

		ret = mcrc64_reset_signature(regs, MCRC64_CHANNEL_1);
		if (ret)
			return ret;

		writeq_relaxed(signature, psa_reg);

		ret = mcrc64_set_mode(regs, MCRC64_CHANNEL_1,
				      MCRC64_MODE_FULL_CPU);
		if (ret)
			return ret;
	}

	*crc64 = signature;

	return 0;
}

static int mcrc64_cra_init(struct crypto_tfm *tfm)
{
	struct mcrc64_tfm_ctx *tctx = crypto_tfm_ctx(tfm);
	struct mcrc64_data *dev_data;

	dev_data = mcrc64_get_dev(tctx);
	if (!dev_data)
		return -ENODEV;

	pm_runtime_get_sync(dev_data->dev);

	tctx->key = 0;

	return 0;
}

static void mcrc64_cra_exit(struct crypto_tfm *tfm)
{
	struct mcrc64_tfm_ctx *tctx = crypto_tfm_ctx(tfm);
	struct mcrc64_data *dev_data;

	dev_data = mcrc64_get_dev(tctx);

	pm_runtime_mark_last_busy(dev_data->dev);
	pm_runtime_put_autosuspend(dev_data->dev);
}

static int mcrc64_setkey(struct crypto_shash *tfm, const u8 *key,
			 unsigned int keylen)
{
	struct mcrc64_tfm_ctx *tctx = crypto_shash_ctx(tfm);

	if (keylen != sizeof(u64))
		return -EINVAL;

	tctx->key = get_unaligned_le64(key);

	return 0;
}

static int mcrc64_init(struct shash_desc *desc)
{
	struct mcrc64_desc_ctx *ctx = shash_desc_ctx(desc);
	struct mcrc64_tfm_ctx *tctx = crypto_shash_ctx(desc->tfm);
	struct mcrc64_data *dev_data;
	void __iomem *psa_reg;

	dev_data = mcrc64_get_dev(tctx);
	if (!dev_data)
		return -ENODEV;

	pm_runtime_get_sync(dev_data->dev);

	/* set capture mode */
	int ret = mcrc64_set_mode(dev_data->regs, MCRC64_CHANNEL_1,
				  MCRC64_MODE_DATA_CAPTURE);
	if (ret)
		return ret;

	/* reset PSA */
	psa_reg = dev_data->regs + PSA_SIGREGL(MCRC64_CHANNEL_1);
	ret =  mcrc64_reset_signature(dev_data->regs, MCRC64_CHANNEL_1);
	if (ret)
		return ret;

	/* write key  */
	writeq_relaxed(tctx->key, psa_reg);

	/* set full cpu mode */
	ret = mcrc64_set_mode(dev_data->regs, MCRC64_CHANNEL_1,
			      MCRC64_MODE_FULL_CPU);
	if (ret)
		return ret;

	ctx->signature = readq_relaxed(psa_reg);

	return 0;
}

static int mcrc64_update(struct shash_desc *desc, const u8 *d8,
			 unsigned int length)
{
	struct mcrc64_desc_ctx *ctx = shash_desc_ctx(desc);
	struct mcrc64_tfm_ctx *tctx = crypto_shash_ctx(desc->tfm);
	struct mcrc64_data *dev_data;

	dev_data = mcrc64_get_dev(tctx);
	if (!dev_data)
		return -ENODEV;

	return mcrc64_calculate_crc(dev_data->regs, MCRC64_CHANNEL_1,
				    d8, length, &ctx->signature);
}

static int mcrc64_final(struct shash_desc *desc, u8 *out)
{
	struct mcrc64_desc_ctx *ctx = shash_desc_ctx(desc);

	/* Send computed CRC */
	put_unaligned_le64(ctx->signature, out);
	return 0;
}

static int mcrc64_finup(struct shash_desc *desc, const u8 *data,
			unsigned int length, u8 *out)
{
	return mcrc64_update(desc, data, length) ?:
		mcrc64_final(desc, out);
}

static int mcrc64_digest(struct shash_desc *desc, const u8 *data,
			 unsigned int length, u8 *out)
{
	return mcrc64_init(desc) ?: mcrc64_finup(desc, data, length, out);
}

static unsigned int refcnt;
static DEFINE_MUTEX(refcnt_lock);
static struct shash_alg algs[] = {
	/* CRC-64 */
	{
		.setkey		= mcrc64_setkey,
		.init		= mcrc64_init,
		.update		= mcrc64_update,
		.final		= mcrc64_final,
		.finup		= mcrc64_finup,
		.digest		= mcrc64_digest,
		.descsize	= sizeof(struct mcrc64_desc_ctx),
		.digestsize	= CHKSUM_DIGEST_SIZE,
		.base		= {
			.cra_name		= CRC64_ISO3309_STRING,
			.cra_driver_name	= "mcrc64",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_OPTIONAL_KEY,
			.cra_blocksize		= CHKSUM_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct mcrc64_tfm_ctx),
			.cra_module		= THIS_MODULE,
			.cra_init		= mcrc64_cra_init,
			.cra_exit		= mcrc64_cra_exit,
		}
	}
};

static int mcrc64_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mcrc64_data *dev_data;
	int ret;

	dev_data = devm_kzalloc(dev, sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data)
		return -ENOMEM;

	dev_data->dev = dev;
	dev_data->regs = devm_platform_ioremap_resource(pdev, 0);

	platform_set_drvdata(pdev, dev_data);

	spin_lock(&mcrc64_dev_list.lock);
	list_add(&dev_data->list, &mcrc64_dev_list.dev_list);
	spin_unlock(&mcrc64_dev_list.lock);

	mutex_lock(&refcnt_lock);
	if (!refcnt) {
		ret = crypto_register_shashes(algs, ARRAY_SIZE(algs));
		if (ret) {
			mutex_unlock(&refcnt_lock);
			dev_err(dev, "Failed to register\n");
			return ret;
		}
	}
	refcnt++;
	mutex_unlock(&refcnt_lock);

	pm_runtime_set_autosuspend_delay(dev, MCRC64_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(dev);

	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	pm_runtime_put_sync(dev);

	return 0;
}

static int mcrc64_remove(struct platform_device *pdev)
{
	struct mcrc64_data *dev_data = platform_get_drvdata(pdev);
	int ret;

	ret = pm_runtime_resume_and_get(dev_data->dev);
	if (ret < 0)
		return ret;

	spin_lock(&mcrc64_dev_list.lock);
	list_del(&dev_data->list);
	spin_unlock(&mcrc64_dev_list.lock);

	mutex_lock(&refcnt_lock);
	if (!--refcnt)
		crypto_unregister_shashes(algs, ARRAY_SIZE(algs));
	mutex_unlock(&refcnt_lock);

	pm_runtime_disable(dev_data->dev);
	pm_runtime_put_noidle(dev_data->dev);

	return 0;
}

static int __maybe_unused mcrc64_suspend(struct device *dev)
{
	return	pm_runtime_force_suspend(dev);
}

static int __maybe_unused mcrc64_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}

static const struct dev_pm_ops mcrc64_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mcrc64_suspend,
				mcrc64_resume)
};

static const struct of_device_id of_match[] = {
	{ .compatible = "ti,am62-mcrc64", },
	{},
};
MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver mcrc64_driver = {
	.probe	= mcrc64_probe,
	.remove = mcrc64_remove,
	.driver = {
		.name		= DRIVER_NAME,
		.pm		= &mcrc64_pm_ops,
		.of_match_table = of_match,
	},
};

module_platform_driver(mcrc64_driver);

MODULE_AUTHOR("Kamlesh Gurudasani <kamlesh@ti.com>");
MODULE_DESCRIPTION("Texas Instruments MCRC64 driver");
MODULE_LICENSE("GPL");
