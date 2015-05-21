/*
 * Allwinner sunXi SoCs Security ID support.
 *
 * Copyright (c) 2013 Oliver Schinagl <oliver@schinagl.nl>
 * Copyright (C) 2014 Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/of.h>
#include "nvmem-mmio.h"

static bool sunxi_sid_writeable_reg(struct device *dev, unsigned int reg)
{
	return false;
}

static struct nvmem_config econfig = {
	.name = "sunix-sid",
	.owner = THIS_MODULE,
};

static struct regmap_config sunxi_sid_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.writeable_reg = sunxi_sid_writeable_reg,
};

static struct nvmem_mmio_data sunxi_data = {
	.nvmem_config = &econfig,
	.regmap_config = &sunxi_sid_regmap_config,
};

static const struct of_device_id sunxi_sid_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-sid", .data = &sunxi_data},
	{ .compatible = "allwinner,sun7i-a20-sid", .data = &sunxi_data},
	{/* sentinel */},
};
MODULE_DEVICE_TABLE(of, sunxi_sid_of_match);

static struct platform_driver sunxi_sid_driver = {
	.probe = nvmem_mmio_probe,
	.remove = nvmem_mmio_remove,
	.driver = {
		.name = "eeprom-sunxi-sid",
		.of_match_table = sunxi_sid_of_match,
	},
};
module_platform_driver(sunxi_sid_driver);

MODULE_AUTHOR("Oliver Schinagl <oliver@schinagl.nl>");
MODULE_DESCRIPTION("Allwinner sunxi security id driver");
MODULE_LICENSE("GPL");
