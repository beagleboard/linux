/*
 * PRCM reset driver for TI SoC's
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/reset.h>
#include <linux/reset-controller.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#define DRIVER_NAME "ti_reset"

struct ti_reset_reg_data {
	u32	rstctrl_offs;
	u32	rstst_offs;
	u8	rstctrl_bit;
	u8	rstst_bit;
};

struct ti_reset_data {
	struct	ti_reset_reg_data *reg_data;
	u8	nr_resets;
};

static void __iomem *reg_base;
static const struct ti_reset_data *reset_data;

static struct ti_reset_reg_data am335x_reset_reg_data[] = {
	{
		.rstctrl_offs	= 0x1104,
		.rstst_offs	= 0x1114,
		.rstctrl_bit	= 0,
		.rstst_bit	= 0,
	},
	{
		.rstctrl_offs	= 0x0D00,
		.rstst_offs	= 0x0D0C,
		.rstctrl_bit	= 3,
		.rstst_bit	= 5,
	},
};

static struct ti_reset_data am335x_reset_data = {
	.reg_data	= am335x_reset_reg_data,
	.nr_resets	= ARRAY_SIZE(am335x_reset_reg_data),
};

static struct ti_reset_reg_data am43x_reset_reg_data[] = {
	{
		.rstctrl_offs	= 0x410,
		.rstst_offs	= 0x414,
		.rstctrl_bit	= 0,
		.rstst_bit	= 0,
	},
	{
		.rstctrl_offs	= 0x2010,
		.rstst_offs	= 0x2014,
		.rstctrl_bit	= 3,
		.rstst_bit	= 5,
	},
};

static struct ti_reset_data am43x_reset_data = {
	.reg_data	= am43x_reset_reg_data,
	.nr_resets	= ARRAY_SIZE(am43x_reset_reg_data),
};

static struct ti_reset_reg_data dra7_reset_reg_data[] = {
	{
		.rstctrl_offs	= 0x1310,
		.rstst_offs	= 0x1314,
		.rstctrl_bit	= 0,
		.rstst_bit	= 0,
	},
};

static struct ti_reset_data dra7_reset_data = {
	.reg_data	= dra7_reset_reg_data,
	.nr_resets	= ARRAY_SIZE(dra7_reset_reg_data),
};

static int ti_reset_clear_reset(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	void __iomem *reg = reset_data->reg_data[id].rstst_offs + reg_base;
	u8 bit = reset_data->reg_data[id].rstst_bit;
	u32 val = readl(reg);

	val &= ~(1 << bit);
	val |= 1 << bit;
	writel(val, reg);
	return 0;
}

static int ti_reset_is_reset(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	void __iomem *reg = reset_data->reg_data[id].rstst_offs + reg_base;
	u8 bit = reset_data->reg_data[id].rstst_bit;
	u32 val = readl(reg);

	val &= (1 << bit);
	return !!val;
}

static int ti_reset_deassert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	void __iomem *reg = reset_data->reg_data[id].rstctrl_offs +
				reg_base;
	u8 bit = reset_data->reg_data[id].rstctrl_bit;
	u32 val = readl(reg);

	val &= ~(1 << bit);
	writel(val, reg);
	return 0;
}

static struct reset_control_ops ti_reset_ops = {
	.deassert = ti_reset_deassert,
	.is_reset = ti_reset_is_reset,
	.clear_reset = ti_reset_clear_reset,
};

static struct reset_controller_dev ti_reset_controller = {
	.ops = &ti_reset_ops,
};

static const struct of_device_id ti_reset_of_match[] = {
	{ .compatible = "ti,am3352-prcm", .data = &am335x_reset_data,},
	{ .compatible = "ti,am4372-prcm", .data = &am43x_reset_data,},
	{ .compatible = "ti,dra7-prcm", .data = &dra7_reset_data,},
	{},
};

static int ti_reset_probe(struct platform_device *pdev)
{
	struct resource *res;
	const struct of_device_id *id;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(reg_base))
		return PTR_ERR(reg_base);

	ti_reset_controller.of_node = pdev->dev.of_node;
	id = of_match_device(ti_reset_of_match, &pdev->dev);
	reset_data = id->data;
	ti_reset_controller.nr_resets = reset_data->nr_resets;

	reset_controller_register(&ti_reset_controller);

	return 0;
}

static int ti_reset_remove(struct platform_device *pdev)
{
	reset_controller_unregister(&ti_reset_controller);

	return 0;
}

static struct platform_driver ti_reset_driver = {
	.probe	= ti_reset_probe,
	.remove	= ti_reset_remove,
	.driver	= {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(ti_reset_of_match),
	},
};
module_platform_driver(ti_reset_driver);

MODULE_DESCRIPTION("PRCM reset driver for TI SoC's");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
