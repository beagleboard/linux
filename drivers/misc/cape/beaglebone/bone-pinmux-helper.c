/*
 * Pinmux helper driver
 *
 * Copyright (C) 2013 Pantelis Antoniou <panto@antoniou-consulting.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>

static const struct of_device_id bone_pinmux_helper_of_match[] = {
	{
		.compatible = "bone-pinmux-helper",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, bone_pinmux_helper_of_match);

static int bone_pinmux_helper_probe(struct platform_device *pdev)
{
	struct pinctrl *pinctrl;

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);

	/* don't bother with anything */
	return PTR_RET(pinctrl);
}

static int bone_pinmux_helper_remove(struct platform_device *pdev)
{
	/* nothing more is needed */
	return 0;
}

struct platform_driver bone_pinmux_helper_driver = {
	.probe		= bone_pinmux_helper_probe,
	.remove		= bone_pinmux_helper_remove,
	.driver = {
		.name		= "bone-pinmux-helper",
		.owner		= THIS_MODULE,
		.of_match_table	= bone_pinmux_helper_of_match,
	},
};

module_platform_driver(bone_pinmux_helper_driver);

MODULE_AUTHOR("Pantelis Antoniou");
MODULE_DESCRIPTION("Beaglebone pinmux helper driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bone-pinmux-helper");
