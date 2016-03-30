/*
 * SYSCON regmap reset driver
 *
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
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

#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
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
 * @toggle: flag to indicate this reset has no readable status register
 */
struct syscon_reset_control {
	unsigned int offset;
	unsigned int reset_bit;
	bool assert_high;
	unsigned int status_offset;
	unsigned int status_reset_bit;
	bool status_assert_high;
	bool toggle;
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
};

#define to_syscon_reset_data(rcdev)	\
	container_of(rcdev, struct syscon_reset_data, rcdev)

/**
 * syscon_reset_get_control() - get reset control info from DT node
 * @data: reset controller data
 * @id: ID address of control data node
 * @control: control data struct to fill
 *
 * Search DT for a child node at the specified address that contains
 * reset control info and use that to fill in control data.
 *
 * XXX: Evaluate if parsing needs to be done in probe to improve
 *      lookup at runtime.
 *
 * Return: 0 for successful request, else a corresponding error value
 */
static int syscon_reset_get_control(struct syscon_reset_data *data,
				    unsigned long id,
				    struct syscon_reset_control *control)
{
	struct device_node *child;
	const __be32 *address;
	const __be32 *list;
	int size;

	for_each_child_of_node(data->rcdev.of_node, child) {
		address = of_get_address(child, 0, NULL, NULL);
		if (!address || be32_to_cpup(address) != id)
			continue;

		list = of_get_property(child, "reset-control", &size);
		if (!list || size != (3 * sizeof(*list)))
			return -EINVAL;
		control->offset = be32_to_cpup(list++);
		control->reset_bit = be32_to_cpup(list++);
		control->assert_high = be32_to_cpup(list) == 1;

		if (of_find_property(child, "reset-toggle", NULL)) {
			control->toggle = true;
			return 0;
		}
		control->toggle = false;

		list = of_get_property(child, "reset-status", &size);
		if (!list) {
			/* use control register values */
			control->status_offset = control->offset;
			control->status_reset_bit = control->reset_bit;
			control->status_assert_high = control->assert_high;
			return 0;
		}

		if (size != (3 * sizeof(*list)))
			return -EINVAL;
		control->status_offset = be32_to_cpup(list++);
		control->status_reset_bit = be32_to_cpup(list++);
		control->status_assert_high = be32_to_cpup(list) == 1;
		return 0;
	}

	return -ENOENT;
}

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
	struct syscon_reset_control control;
	unsigned int mask, value;
	int ret;

	ret = syscon_reset_get_control(data, id, &control);
	if (ret)
		return ret;

	mask = BIT(control.reset_bit);
	value = (assert == control.assert_high) ? mask : 0x0;

	return regmap_update_bits(data->memory, control.offset, mask, value);
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
 * Return: 0 if reset is deasserted, true if reset is asserted, else a
 * corresponding error value
 */
static int syscon_reset_status(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	struct syscon_reset_data *data = to_syscon_reset_data(rcdev);
	struct syscon_reset_control control;
	unsigned int reset_state;
	int ret;

	ret = syscon_reset_get_control(data, id, &control);
	if (ret)
		return ret;

	if (control.toggle)
		return -ENOSYS; /* status not supported for this reset */

	ret = regmap_read(data->memory, control.status_offset, &reset_state);
	if (ret)
		return ret;

	return (reset_state & BIT(control.status_reset_bit)) ==
			control.status_assert_high;
}

static struct reset_control_ops syscon_reset_ops = {
	.assert		= syscon_reset_assert,
	.deassert	= syscon_reset_deassert,
	.status		= syscon_reset_status,
};

static int syscon_reset_probe(struct platform_device *pdev)
{
	struct syscon_reset_data *data;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct regmap *memory;

	if (!np)
		return -ENODEV;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	memory = syscon_node_to_regmap(np->parent);
	if (IS_ERR(memory))
		return PTR_ERR(memory);

	data->rcdev.ops = &syscon_reset_ops;
	data->rcdev.owner = THIS_MODULE;
	data->rcdev.of_node = np;
	data->rcdev.nr_resets = of_get_child_count(np);
	data->dev = dev;
	data->memory = memory;
	platform_set_drvdata(pdev, data);

	return reset_controller_register(&data->rcdev);
}

static int syscon_reset_remove(struct platform_device *pdev)
{
	struct syscon_reset_data *data = platform_get_drvdata(pdev);

	reset_controller_unregister(&data->rcdev);

	return 0;
}

static const struct of_device_id syscon_reset_of_match[] = {
	{ .compatible = "syscon-reset", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, syscon_reset_of_match);

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
