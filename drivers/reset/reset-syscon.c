/*
 * SYSCON regmap reset driver
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 *	Suman Anna <afd@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/idr.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

/**
 * struct syscon_reset_control - reset control structure
 * @offset: reset control register offset from syscon base
 * @reset_bit: reset bit in the reset control register
 * @assert_high: flag to indicate if setting the bit high asserts the reset
 * @status_offset: reset status register offset from syscon base
 * @status_reset_bit: reset status bit in the reset status register
 * @status_assert_high: flag to indicate if a set bit represents asserted state
 */
struct syscon_reset_control {
	unsigned int offset;
	unsigned int reset_bit;
	bool assert_high;
	unsigned int status_offset;
	unsigned int status_reset_bit;
	bool status_assert_high;
};

/**
 * struct syscon_reset_data - reset controller information structure
 * @rcdev: reset controller entity
 * @dev: reset controller device pointer
 * @memory: regmap handle containing the memory-mapped reset registers
 * @idr: idr structure for mapping ids to reset control structures
 */
struct syscon_reset_data {
	struct reset_controller_dev rcdev;
	struct device *dev;
	struct regmap *memory;
	struct idr idr;
};

#define to_syscon_reset_data(rcdev)	\
	container_of(rcdev, struct syscon_reset_data, rcdev)

/**
 * syscon_reset_set() - program a device's reset
 * @rcdev: reset controller entity
 * @id: ID of the reset to toggle
 * @assert: boolean flag to indicate assert or deassert
 *
 * This is a common internal function used to assert or deassert a device's
 * reset using the regmap API. The device's reset is asserted if the @assert
 * argument is true, or deasserted if the @assert argument is false.
 *
 * Return: 0 for successful request, else a corresponding error value
 */
static int syscon_reset_set(struct reset_controller_dev *rcdev,
			    unsigned long id, bool assert)
{
	struct syscon_reset_data *data = to_syscon_reset_data(rcdev);
	struct syscon_reset_control *control;
	unsigned int mask, value;

	control = idr_find(&data->idr, id);
	if (!control)
		return -EINVAL;

	mask = BIT(control->reset_bit);
	value = (assert == control->assert_high) ? mask : 0x0;

	return regmap_update_bits(data->memory, control->offset, mask, value);
}

/**
 * syscon_reset_assert() - assert device reset
 * @rcdev: reset controller entity
 * @id: ID of the reset to be asserted
 *
 * This function implements the reset driver op to assert a device's reset.
 * This invokes the function syscon_reset_set() with the corresponding
 * parameters as passed in, but with the @assert argument set to true for
 * asserting the reset.
 *
 * Return: 0 for successful request, else a corresponding error value
 */
static int syscon_reset_assert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	return syscon_reset_set(rcdev, id, true);
}

/**
 * syscon_reset_deassert() - deassert device reset
 * @rcdev: reset controller entity
 * @id: ID of reset to be deasserted
 *
 * This function implements the reset driver op to deassert a device's reset.
 * This invokes the function syscon_reset_set() with the corresponding
 * parameters as passed in, but with the @assert argument set to false for
 * deasserting the reset.
 *
 * Return: 0 for successful request, else a corresponding error value
 */
static int syscon_reset_deassert(struct reset_controller_dev *rcdev,
				 unsigned long id)
{
	return syscon_reset_set(rcdev, id, false);
}

/**
 * syscon_reset_status() - check device reset status
 * @rcdev: reset controller entity
 * @id: ID of the reset for which the status is being requested
 *
 * This function implements the reset driver op to return the status of a
 * device's reset.
 *
 * Return: 0 if reset is deasserted, or a non-zero value if reset is asserted
 */
static int syscon_reset_status(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	struct syscon_reset_data *data = to_syscon_reset_data(rcdev);
	struct syscon_reset_control *control;
	unsigned int reset_state;
	int ret;

	control = idr_find(&data->idr, id);
	if (!control)
		return -EINVAL;

	ret = regmap_read(data->memory, control->status_offset, &reset_state);
	if (ret)
		return ret;

	return (reset_state & BIT(control->status_reset_bit)) ==
			control->status_assert_high;
}

static struct reset_control_ops syscon_reset_ops = {
	.assert		= syscon_reset_assert,
	.deassert	= syscon_reset_deassert,
	.status		= syscon_reset_status,
};

/**
 * syscon_reset_of_xlate() - translate a set of OF arguments to a reset ID
 * @rcdev: reset controller entity
 * @reset_spec: OF reset argument specifier
 *
 * This function performs the translation of the reset argument specifier
 * values defined in a reset consumer device node. The function allocates a
 * reset control structure for that device reset, that will be used by the
 * driver for performing any reset functions on that reset. An idr structure
 * is allocated and used to map to the reset control structure. This idr is
 * used by the driver to do reset lookups.
 *
 * Return: 0 for successful request, else a corresponding error value
 */
static int syscon_reset_of_xlate(struct reset_controller_dev *rcdev,
				 const struct of_phandle_args *reset_spec)
{
	struct syscon_reset_data *data = to_syscon_reset_data(rcdev);
	struct syscon_reset_control *control;

	if (WARN_ON(reset_spec->args_count != rcdev->of_reset_n_cells))
		return -EINVAL;

	control = devm_kzalloc(data->dev, sizeof(*control), GFP_KERNEL);
	if (!control)
		return -ENOMEM;

	control->offset = reset_spec->args[0];
	control->reset_bit = reset_spec->args[1];
	control->assert_high = reset_spec->args[2] == 1;
	control->status_offset = reset_spec->args[3];
	control->status_reset_bit = reset_spec->args[4];
	control->status_assert_high = reset_spec->args[5] == 1;

	return idr_alloc(&data->idr, control, 0, 0, GFP_KERNEL);
}

static const struct of_device_id syscon_reset_of_match[] = {
	{ .compatible = "syscon-reset", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, syscon_reset_of_match);

static int syscon_reset_probe(struct platform_device *pdev)
{
	struct syscon_reset_data *data;
	struct device_node *np = pdev->dev.of_node;
	struct regmap *memory;

	if (!np)
		return -ENODEV;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	memory = syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(memory))
		return PTR_ERR(memory);

	data->rcdev.ops = &syscon_reset_ops;
	data->rcdev.owner = THIS_MODULE;
	data->rcdev.of_node = np;
	data->rcdev.of_reset_n_cells = 6;
	data->rcdev.of_xlate = syscon_reset_of_xlate;
	data->dev = &pdev->dev;
	data->memory = memory;
	idr_init(&data->idr);

	platform_set_drvdata(pdev, data);

	return reset_controller_register(&data->rcdev);
}

static int syscon_reset_remove(struct platform_device *pdev)
{
	struct syscon_reset_data *data = platform_get_drvdata(pdev);

	reset_controller_unregister(&data->rcdev);

	idr_destroy(&data->idr);

	return 0;
}

static struct platform_driver syscon_reset_driver = {
	.probe = syscon_reset_probe,
	.remove = syscon_reset_remove,
	.driver = {
		.name = "syscon-reset",
		.of_match_table = syscon_reset_of_match,
	},
};
module_platform_driver(syscon_reset_driver);

MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_DESCRIPTION("SYSCON Regmap Reset Driver");
MODULE_LICENSE("GPL v2");
