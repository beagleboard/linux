// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017, Impinj, Inc.
 *
 * i.MX7 System Reset Controller (SRC) driver
 *
 * Author: Andrey Smirnov <andrew.smirnov@gmail.com>
 */

#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/regmap.h>
#include <dt-bindings/reset/light-reset.h>

struct light_src_signal {
	unsigned int offset, bit;
};

struct light_src_variant {
	const struct light_src_signal *signals;
	unsigned int signals_num;
	struct reset_control_ops ops;
};

struct light_src {
	struct reset_controller_dev rcdev;
	struct regmap *regmap;
	const struct light_src_signal *signals;
};

enum light_src_registers {
	SRC_WDT0		= 0x0034,
	SRC_WDT1		= 0x0038,
};

static int light_reset_update(struct light_src *lightsrc,
			     unsigned long id, unsigned int value)
{
	const struct light_src_signal *signal = &lightsrc->signals[id];

	return regmap_update_bits(lightsrc->regmap,
				  signal->offset, signal->bit, value);
}

static const struct light_src_signal light_src_signals[] = {
	[LIGHT_RESET_WDT0] = { SRC_WDT0, BIT(0) },
	[LIGHT_RESET_WDT1] = { SRC_WDT1, BIT(0) },
};

static struct light_src *to_light_src(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct light_src, rcdev);
}

static int light_reset_set(struct reset_controller_dev *rcdev,
			  unsigned long id, bool assert)
{
	struct light_src *lightsrc = to_light_src(rcdev);
	const unsigned int bit = lightsrc->signals[id].bit;
	unsigned int value = assert ? bit : 0;

	switch (id) {
		/*add special dispatch*/
		default:
		break;
	}

	return light_reset_update(lightsrc, id, value);
}

static int light_reset_assert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	pr_info("%s id:%u\n",__func__,id);
	return light_reset_set(rcdev, id, false);
}

static int light_reset_deassert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	pr_info("%s id:%u\n",__func__,id);
	return light_reset_set(rcdev, id, true);
}

static const struct light_src_variant variant_light = {
	.signals = light_src_signals,
	.signals_num = ARRAY_SIZE(light_src_signals),
	.ops = {
		.assert   = light_reset_assert,
		.deassert = light_reset_deassert,
	},
};

static int light_reset_probe(struct platform_device *pdev)
{
	struct light_src *lightsrc;
	struct device *dev = &pdev->dev;
	struct regmap_config config = { .name = "src" };
	const struct light_src_variant *variant = of_device_get_match_data(dev);

	lightsrc = devm_kzalloc(dev, sizeof(*lightsrc), GFP_KERNEL);
	if (!lightsrc)
		return -ENOMEM;

	lightsrc->signals = variant->signals;
	lightsrc->regmap = syscon_node_to_regmap(dev->of_node);
	if (IS_ERR(lightsrc->regmap)) {
		dev_err(dev, "Unable to get light-src regmap");
		return PTR_ERR(lightsrc->regmap);
	}
	regmap_attach_dev(dev, lightsrc->regmap, &config);

	lightsrc->rcdev.owner     = THIS_MODULE;
	lightsrc->rcdev.nr_resets = variant->signals_num;
	lightsrc->rcdev.ops       = &variant->ops;
	lightsrc->rcdev.of_node   = dev->of_node;

	return devm_reset_controller_register(dev, &lightsrc->rcdev);
}

static const struct of_device_id light_reset_dt_ids[] = {
	{ .compatible = "thead,light-reset-src", .data = &variant_light },
};
MODULE_DEVICE_TABLE(of, light_reset_dt_ids);

static struct platform_driver light_reset_driver = {
	.probe	= light_reset_probe,
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= light_reset_dt_ids,
	},
};
module_platform_driver(light_reset_driver);

MODULE_AUTHOR("zenglinghui.zlh <zenglinghui.zlh@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead light reset driver");
MODULE_LICENSE("GPL v2");
