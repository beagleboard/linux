/*
 * LCD panel driver for Samsung LTE430WQ-F0C
 *
 * Author: Steve Sakoman <steve@sakoman.com>
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
#include <linux/delay.h>

#include <plat/display.h>

static struct omap_video_timings samsung_lte_timings = {
	.x_res = 480,
	.y_res = 272,

	.pixel_clock	= 9200,

	.hsw		= 41,
	.hfp		= 8,
	.hbp		= 45-41,

	.vsw		= 10,
	.vfp		= 4,
	.vbp		= 12-10,
};

static int samsung_lte_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS;
	dssdev->panel.timings = samsung_lte_timings;

	return 0;
}

static void samsung_lte_panel_remove(struct omap_dss_device *dssdev)
{
}

static int samsung_lte_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	/* wait couple of vsyncs until enabling the LCD */
	msleep(50);

	if (dssdev->platform_enable)
		r = dssdev->platform_enable(dssdev);

	return r;
}

static void samsung_lte_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */

	msleep(100);
}

static int samsung_lte_panel_suspend(struct omap_dss_device *dssdev)
{
	samsung_lte_panel_disable(dssdev);
	return 0;
}

static int samsung_lte_panel_resume(struct omap_dss_device *dssdev)
{
	return samsung_lte_panel_enable(dssdev);
}

static struct omap_dss_driver samsung_lte_driver = {
	.probe		= samsung_lte_panel_probe,
	.remove		= samsung_lte_panel_remove,

	.enable		= samsung_lte_panel_enable,
	.disable	= samsung_lte_panel_disable,
	.suspend	= samsung_lte_panel_suspend,
	.resume		= samsung_lte_panel_resume,

	.driver         = {
		.name   = "samsung_lte_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init samsung_lte_panel_drv_init(void)
{
	return omap_dss_register_driver(&samsung_lte_driver);
}

static void __exit samsung_lte_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&samsung_lte_driver);
}

module_init(samsung_lte_panel_drv_init);
module_exit(samsung_lte_panel_drv_exit);
MODULE_LICENSE("GPL");
