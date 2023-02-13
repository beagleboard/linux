// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) Texas Instruments 2023 - http://www.ti.com
 * Author: Kamlesh Gurudasani <kamlesh@ti.com>
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <crypto/internal/hash.h>

#include <asm/unaligned.h>

#define DRIVER_NAME		"ti-mcrc"
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

#define MCRC_AUTOSUSPEND_DELAY	50

static struct device *mcrc_k3_dev;

static unsigned int burst_size;

module_param(burst_size, uint, 0644);
MODULE_PARM_DESC(burst_size, "Select burst byte size (0 unlimited)");

enum mcrc_mode {
	MCRC_MODE_DATA_CAPTURE = 0,
	MCRC_MODE_AUTO,
	MCRC_MODE_SEMI_CPU,
	MCRC_MODE_FULL_CPU,
	MCRC_MODE_INVALID,
};

enum mcrc_pattern {
	MCRC_PATTERN_8BIT = 0,
	MCRC_PATTERN_16BIT,
	MCRC_PATTERN_32BIT,
	MCRC_PATTERN_64BIT,
	MCRC_PATTERN_INVALID,
};

enum mcrc_channel {
	MCRC_CHANNEL_1 = 1,
	MCRC_CHANNEL_2,
	MCRC_CHANNEL_3,
	MCRC_CHANNEL_4,
	MCRC_CHANNEL_INVALID,
};

struct mcrc_data {
	struct device	 *dev;
	void __iomem	 *regs;
};

struct mcrc_ctx {
	u32 key;
};

struct mcrc_desc_ctx {
	u64    signature;
};

static int mcrc_set_mode(void __iomem *regs, u32 channel, u32 mode)
{
	u32 crc_ctrl2_reg, mode_set_val;

	if (mode < 0 || mode >= MCRC_MODE_INVALID)
		return -EINVAL;

	if (channel <= 0 || channel >= MCRC_CHANNEL_INVALID)
		return -EINVAL;

	crc_ctrl2_reg = readl_relaxed(regs + CRC_CTRL2);

	mode_set_val = crc_ctrl2_reg | CH_MODE(channel, mode);

	/* Write CRC_CTRL2, set mode */
	writel_relaxed(mode_set_val, regs + CRC_CTRL2);

	return 0;
}

static int mcrc_reset_signature(void __iomem *regs, u32 channel)
{
	u32 crc_ctrl0_reg, reset_val, reset_undo_val;

	if (channel <= 0 || channel >= MCRC_CHANNEL_INVALID)
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

static int mcrc_calculate_crc(void __iomem *regs, u32 channel,
			      u32 pattern, const u8 *d8, size_t length)
{
	void __iomem *psa_reg;
	u64 signature;

	if (channel <= 0 || channel >= MCRC_CHANNEL_INVALID)
		return -EINVAL;

	psa_reg = regs + PSA_SIGREGL(channel);

	for (; length >= sizeof(u64); d8 += sizeof(u64), length -= sizeof(u64)) {
		writeq_relaxed(*((u64 *)d8), psa_reg);
		signature = readq_relaxed(psa_reg);
	}

	if (length) {
		u64 leftover = 0;

		while (length--)
			leftover =  (*d8++) << 8  | leftover;
		writeq_relaxed(leftover, psa_reg);
		signature = readq_relaxed(psa_reg);
	}

	return 0;
}

static int mcrc_cra_init(struct crypto_tfm *tfm)
{
	struct mcrc_ctx *mctx = crypto_tfm_ctx(tfm);

	struct mcrc_data *dev_data = dev_get_drvdata(mcrc_k3_dev);

	pm_runtime_get_sync(dev_data->dev);

	mctx->key = 0;

	return 0;
}

static void mcrc_cra_exit(struct crypto_tfm *tfm)
{
	struct mcrc_data *dev_data = dev_get_drvdata(mcrc_k3_dev);

	pm_runtime_mark_last_busy(dev_data->dev);
	pm_runtime_put_autosuspend(dev_data->dev);
}

static int mcrc_setkey(struct crypto_shash *tfm, const u8 *key,
		       unsigned int keylen)
{
	struct mcrc_ctx *mctx = crypto_shash_ctx(tfm);

	if (keylen != sizeof(u32))
		return -EINVAL;

	mctx->key = get_unaligned_le32(key);

	return 0;
}

static int mcrc_init(struct shash_desc *desc)
{
	struct mcrc_data *dev_data = dev_get_drvdata(mcrc_k3_dev);

	/* set full cpu mode */
	int ret = mcrc_set_mode(dev_data->regs, MCRC_CHANNEL_1,
				MCRC_MODE_FULL_CPU);
	if (ret)
		return ret;

	/* reset PSA */
	return mcrc_reset_signature(dev_data->regs, MCRC_CHANNEL_1);
}

static int burst_update(struct shash_desc *desc, const u8 *d8,
			size_t length)
{
	struct mcrc_desc_ctx *ctx = shash_desc_ctx(desc);
	struct mcrc_data *dev_data = dev_get_drvdata(mcrc_k3_dev);

	int ret = mcrc_calculate_crc(dev_data->regs, MCRC_CHANNEL_1,
				     MCRC_PATTERN_64BIT, d8, length);
	if (ret)
		return ret;

	/* Store signature result */
	ctx->signature = readq_relaxed(dev_data->regs +
				       PSA_SIGREGL(MCRC_CHANNEL_1));

	return 0;
}

static int mcrc_update(struct shash_desc *desc, const u8 *d8,
		       unsigned int length)
{
	const unsigned int burst_sz = burst_size;
	unsigned int rem_sz;
	const u8 *cur;
	size_t size;
	int ret;

	if (!burst_sz)
		return burst_update(desc, d8, length);

	/* Digest first bytes not 64bit aligned at first pass in the loop */
	size = min_t(size_t, length, burst_sz + (size_t)d8 -
		ALIGN_DOWN((size_t)d8, sizeof(u64)));
	for (rem_sz = length, cur = d8; rem_sz;
	     rem_sz -= size, cur += size, size = min(rem_sz, burst_sz)) {
		ret = burst_update(desc, cur, size);
		if (ret)
			return ret;
	}

	return 0;
}

static int mcrc_final(struct shash_desc *desc, u8 *out)
{
	struct mcrc_desc_ctx *ctx = shash_desc_ctx(desc);

	/* Send computed CRC */
	put_unaligned_le64(ctx->signature, out);
	return 0;
}

static int mcrc_finup(struct shash_desc *desc, const u8 *data,
		      unsigned int length, u8 *out)
{
	return mcrc_update(desc, data, length) ?:
		mcrc_final(desc, out);
}

static int mcrc_digest(struct shash_desc *desc, const u8 *data,
		       unsigned int length, u8 *out)
{
	return mcrc_init(desc) ?: mcrc_finup(desc, data, length, out);
}

static struct shash_alg algs[] = {
	/* CRC-64 */
	{
		.setkey		= mcrc_setkey,
		.init		= mcrc_init,
		.update		= mcrc_update,
		.final		= mcrc_final,
		.finup		= mcrc_finup,
		.digest		= mcrc_digest,
		.descsize	= sizeof(struct mcrc_desc_ctx),
		.digestsize	= CHKSUM_DIGEST_SIZE,
		.base		= {
			.cra_name		= "crc64",
			.cra_driver_name	= "mcrc",
			.cra_priority		= 200,
			.cra_flags		= CRYPTO_ALG_OPTIONAL_KEY,
			.cra_blocksize		= CHKSUM_BLOCK_SIZE,
			.cra_alignmask		= 7,
			.cra_ctxsize		= sizeof(struct mcrc_ctx),
			.cra_module		= THIS_MODULE,
			.cra_init		= mcrc_cra_init,
			.cra_exit		= mcrc_cra_exit,
		}
	}
};

static int mcrc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mcrc_data *dev_data;

	dev_data = devm_kzalloc(dev, sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data)
		return -ENOMEM;

	mcrc_k3_dev = dev;
	dev_data->dev = dev;
	dev_data->regs = devm_platform_ioremap_resource(pdev, 0);

	platform_set_drvdata(pdev, dev_data);
	dev_set_drvdata(mcrc_k3_dev, dev_data);

	crypto_register_shashes(algs, ARRAY_SIZE(algs));

	pm_runtime_set_autosuspend_delay(dev, MCRC_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(dev);

	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	pm_runtime_put_sync(dev);

	return 0;
}

static int mcrc_remove(struct platform_device *pdev)
{
	struct mcrc_data *dev_data = platform_get_drvdata(pdev);

	int ret = pm_runtime_get_sync(dev_data->dev);

	if (ret < 0) {
		pm_runtime_put_noidle(dev_data->dev);
		return ret;
	}

	crypto_unregister_shashes(algs, ARRAY_SIZE(algs));

	pm_runtime_disable(dev_data->dev);
	pm_runtime_put_noidle(dev_data->dev);

	return 0;
}

static int __maybe_unused mcrc_suspend(struct device *dev)
{
	return	pm_runtime_force_suspend(dev);
}

static int __maybe_unused mcrc_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}

static const struct dev_pm_ops mcrc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mcrc_suspend,
				mcrc_resume)
};

static const struct of_device_id of_match[] = {
	{ .compatible = "ti,mcrc", },
	{},
};
MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver mcrc_driver = {
	.probe	= mcrc_probe,
	.remove = mcrc_remove,
	.driver = {
		.name		= DRIVER_NAME,
		.pm		= &mcrc_pm_ops,
		.of_match_table = of_match,
	},
};

module_platform_driver(mcrc_driver);

MODULE_AUTHOR("Kamlesh Gurudasani <kamlesh@ti.com>");
MODULE_DESCRIPTION("Texas Instruments MCRC hardware driver");
MODULE_LICENSE("GPL v2");
