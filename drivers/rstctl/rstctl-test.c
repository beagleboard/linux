/*
 * Test driver for rstctl
 *
 * Author: Pantelis Antoniou <panto@antoniou-consulting.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rstctl.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/err.h>

struct test_rctrl_info {
	struct rstctl_dev *rdev;
};

int test_rctrl_request(struct rstctl_dev *rdev,
		const struct rstctl_line *line)
{
	dev_info(rdev->dev, "%s %s\n", __func__, line->name);
	return 0;
}

int test_rctrl_release(struct rstctl_dev *rdev,
		const struct rstctl_line *line)
{
	dev_info(rdev->dev, "%s %s\n", __func__, line->name);
	return 0;
}

int test_rctrl_assert(struct rstctl_dev *rdev,
		const struct rstctl_line *line)
{
	dev_info(rdev->dev, "%s %s\n", __func__, line->name);
	return 0;
}

int test_rctrl_deassert(struct rstctl_dev *rdev,
		const struct rstctl_line *line)
{
	dev_info(rdev->dev, "%s %s\n", __func__, line->name);
	return 0;
}

int test_rctrl_pulse(struct rstctl_dev *rdev,
		const struct rstctl_line *line,
		unsigned long hold_ns)
{
	dev_info(rdev->dev, "%s %s\n", __func__, line->name);
	return 0;
}

static const struct rstctl_ops test_rctrl_ops = {
	.request	= test_rctrl_request,
	.release	= test_rctrl_release,
	.assert		= test_rctrl_assert,
	.deassert	= test_rctrl_deassert,
	.pulse		= test_rctrl_pulse,
};

static const struct rstctl_line test_rctrl_lines[] = {
	{ .name = "RESET1", },
	{ .name = "RESET2", },
};

static const struct rstctl_desc test_rctrl_desc = {
	.name	= "test",
	.ops	= &test_rctrl_ops,
	.nlines	= ARRAY_SIZE(test_rctrl_lines),
	.lines	= test_rctrl_lines,
};

static int test_rctrl_probe(struct platform_device *pdev)
{
	struct test_rctrl_info *info;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->rdev = rstctl_register(&pdev->dev, &test_rctrl_desc);
	if (IS_ERR(info->rdev)) {
		dev_err(&pdev->dev, "failed to register\n");
		return PTR_ERR(info->rdev);
	}
	platform_set_drvdata(pdev, info);

	dev_info(&pdev->dev, "loaded OK\n");

	return 0;
}

static int test_rctrl_remove(struct platform_device *pdev)
{
	struct test_rctrl_info *info = platform_get_drvdata(pdev);
	int err;

	err = rstctl_unregister(info->rdev);
	if (err == 0)
		dev_info(&pdev->dev, "removed OK\n");

	return err;
}

#ifdef CONFIG_OF
static struct of_device_id test_rctrl_of_match[] = {
	{ .compatible = "test-rctrl" },
	{ },
};
MODULE_DEVICE_TABLE(of, test_rctrl_of_match);
#endif

static struct platform_driver test_rctrl_driver = {
	.driver = {
		.name = "test-rctrl",
		.of_match_table = of_match_ptr(test_rctrl_of_match),
	},
	.probe = test_rctrl_probe,
	.remove = test_rctrl_remove,
};
module_platform_driver(test_rctrl_driver);

MODULE_AUTHOR("Pantelis Antoniou <panto@antoniou-consulting.com>");
MODULE_DESCRIPTION("rstctl test driver");
MODULE_LICENSE("GPL");
