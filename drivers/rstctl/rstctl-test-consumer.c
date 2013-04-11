/*
 * Test consumer driver for rstctl
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

static int test_consumer_rctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rstctl *rctrl;

	dev_info(dev, "Trying to get NULL (OF case only)\n");
	rctrl = rstctl_get(dev, NULL);
	if (IS_ERR(rctrl)) {
		dev_info(dev, "Failed to get it\n");
		return PTR_ERR(rctrl);
	}

	dev_info(dev, "Got it (%s:#%d name %s) label:%s\n",
			rctrl->rdev->rdesc->name,
			rctrl->line - rctrl->rdev->rdesc->lines,
			rctrl->line->name, rctrl->label);

	/* for now always assert */
	rstctl_assert(rctrl);
	platform_set_drvdata(pdev, rctrl);

	dev_info(&pdev->dev, "loaded OK\n");
	return 0;
}

static int test_consumer_rctrl_remove(struct platform_device *pdev)
{
	struct rstctl *rctrl = platform_get_drvdata(pdev);

	rstctl_deassert(rctrl);
	rstctl_put(rctrl);

	dev_info(&pdev->dev, "unloaded OK\n");
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id test_consumer_rctrl_of_match[] = {
	{ .compatible = "test-consumer-rctrl" },
	{ },
};
MODULE_DEVICE_TABLE(of, test_consumer_rctrl_of_match);
#endif

static struct platform_driver test_consumer_rctrl_driver = {
	.driver = {
		.name = "test-consumer-rctrl",
		.of_match_table = of_match_ptr(test_consumer_rctrl_of_match),
	},
	.probe = test_consumer_rctrl_probe,
	.remove = test_consumer_rctrl_remove,
};
module_platform_driver(test_consumer_rctrl_driver);

MODULE_AUTHOR("Pantelis Antoniou <panto@antoniou-consulting.com>");
MODULE_DESCRIPTION("rstctl test consumer driver");
MODULE_LICENSE("GPL");
