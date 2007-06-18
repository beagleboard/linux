/*
 * LCD panel support for the TI OMAP H2 board
 *
 * Copyright (C) 2004 Nokia Corporation
 * Author: Imre Deak <imre.deak@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/tsc2101.h>

#include <asm/arch/mux.h>
#include <asm/arch/omapfb.h>

static struct {
	struct platform_device	*lcd_dev;
	struct spi_device	*tsc2101_dev;
} h2_panel_dev;

static int h2_panel_init(struct lcd_panel *panel, struct omapfb_device *fbdev)
{
	return 0;
}

static void h2_panel_cleanup(struct lcd_panel *panel)
{
}

static int h2_panel_enable(struct lcd_panel *panel)
{
	int r;

	/*
	 * Assert LCD_EN, BKLIGHT_EN pins on LCD panel
	 * page2, GPIO config reg, GPIO(0,1) to out and asserted
	 */
	r = tsc2101_write_sync(h2_panel_dev.tsc2101_dev, 2, 0x23, 0xcc00);
	if (r < 0)
		dev_err(&h2_panel_dev.lcd_dev->dev,
			"failed to enable LCD panel\n");

	return r;
}

static void h2_panel_disable(struct lcd_panel *panel)
{
	/*
	 * Deassert LCD_EN and BKLIGHT_EN pins on LCD panel
	 * page2, GPIO config reg, GPIO(0,1) to out and deasserted
	 */
	if (tsc2101_write_sync(h2_panel_dev.tsc2101_dev, 2, 0x23, 0x8800))
		dev_err(&h2_panel_dev.lcd_dev->dev,
			"failed to disable LCD panel\n");
}

static unsigned long h2_panel_get_caps(struct lcd_panel *panel)
{
	return 0;
}

struct lcd_panel h2_panel = {
	.name		= "h2",
	.config		= OMAP_LCDC_PANEL_TFT,

	.bpp		= 16,
	.data_lines	= 16,
	.x_res		= 240,
	.y_res		= 320,
	.pixel_clock	= 5000,
	.hsw		= 12,
	.hfp		= 12,
	.hbp		= 46,
	.vsw		= 1,
	.vfp		= 1,
	.vbp		= 0,

	.init		= h2_panel_init,
	.cleanup	= h2_panel_cleanup,
	.enable		= h2_panel_enable,
	.disable	= h2_panel_disable,
	.get_caps	= h2_panel_get_caps,
};

static int h2_panel_probe(struct platform_device *pdev)
{
	struct spi_device *tsc2101;

	tsc2101 = pdev->dev.platform_data;
	if (tsc2101 == NULL) {
		dev_err(&pdev->dev, "no platform data\n");
		return -ENODEV;
	}
	if (strncmp(tsc2101->modalias, "tsc2101", 8) != 0) {
		dev_err(&pdev->dev, "tsc2101 not found\n");
		return -EINVAL;
	}
	h2_panel_dev.lcd_dev = pdev;
	h2_panel_dev.tsc2101_dev = tsc2101;
	omapfb_register_panel(&h2_panel);
	return 0;
}

static int h2_panel_remove(struct platform_device *pdev)
{
	return 0;
}

static int h2_panel_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int h2_panel_resume(struct platform_device *pdev)
{
	return 0;
}

struct platform_driver h2_panel_driver = {
	.probe		= h2_panel_probe,
	.remove		= h2_panel_remove,
	.suspend	= h2_panel_suspend,
	.resume		= h2_panel_resume,
	.driver		= {
		.name	= "lcd_h2",
		.owner	= THIS_MODULE,
	},
};

static int h2_panel_drv_init(void)
{
	return platform_driver_register(&h2_panel_driver);
}

static void h2_panel_drv_cleanup(void)
{
	platform_driver_unregister(&h2_panel_driver);
}

module_init(h2_panel_drv_init);
module_exit(h2_panel_drv_cleanup);

