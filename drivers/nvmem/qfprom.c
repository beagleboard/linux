/*
 * Copyright (C) 2015 Srinivas Kandagatla <srinivas.kandagatla@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/of.h>
#include "nvmem-mmio.h"

static struct regmap_config qfprom_regmap_config = {
	.reg_bits = 32,
	.val_bits = 8,
	.reg_stride = 1,
};

static struct nvmem_config econfig = {
	.name = "qfprom",
	.owner = THIS_MODULE,
};

static struct nvmem_mmio_data qfprom_data = {
	.nvmem_config = &econfig,
	.regmap_config = &qfprom_regmap_config,
};

static const struct of_device_id qfprom_of_match[] = {
	{ .compatible = "qcom,qfprom", .data = &qfprom_data},
	{/* sentinel */},
};
MODULE_DEVICE_TABLE(of, qfprom_of_match);

static struct platform_driver qfprom_driver = {
	.probe = nvmem_mmio_probe,
	.remove = nvmem_mmio_remove,
	.driver = {
		.name = "qcom,qfprom",
		.of_match_table = qfprom_of_match,
	},
};
module_platform_driver(qfprom_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org>");
MODULE_DESCRIPTION("Qualcomm QFPROM driver");
MODULE_LICENSE("GPL v2");
