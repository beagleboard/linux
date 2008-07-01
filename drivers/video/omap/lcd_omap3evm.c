/*
 * LCD panel support for the TI OMAP3 EVM board
 *
 * Author: Steve Sakoman <steve@sakoman.com>
 *
 * Derived from drivers/video/omap/lcd-apollon.c
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
#include <linux/i2c/twl4030.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/omapfb.h>
#include <asm/mach-types.h>

#define LCD_PANEL_ENABLE_GPIO       153
#define LCD_PANEL_LR                2
#define LCD_PANEL_UD                3
#define LCD_PANEL_INI               152
#define LCD_PANEL_QVGA              154
#define LCD_PANEL_RESB              155

#define LCD_XRES	 	480
#define LCD_YRES 		640
#define LCD_PIXCLOCK		26000 /* in kHz  */

#define ENABLE_VDAC_DEDICATED	0x03
#define ENABLE_VDAC_DEV_GRP	0x20
#define ENABLE_VPLL2_DEDICATED	0x05
#define ENABLE_VPLL2_DEV_GRP	0xE0

#define TWL_LED_LEDEN		0x00
#define TWL_PWMA_PWMAON		0x00
#define TWL_PWMA_PWMAOFF	0x01

static unsigned int bklight_level;

static int omap3evm_panel_init(struct lcd_panel *panel,
				struct omapfb_device *fbdev)
{
	omap_request_gpio(LCD_PANEL_LR);
	omap_request_gpio(LCD_PANEL_UD);
	omap_request_gpio(LCD_PANEL_INI);
	omap_request_gpio(LCD_PANEL_RESB);
	omap_request_gpio(LCD_PANEL_QVGA);

	omap_set_gpio_direction(LCD_PANEL_LR, 0);
	omap_set_gpio_direction(LCD_PANEL_UD, 0);
	omap_set_gpio_direction(LCD_PANEL_INI, 0);
	omap_set_gpio_direction(LCD_PANEL_RESB, 0);
	omap_set_gpio_direction(LCD_PANEL_QVGA, 0);

	twl4030_i2c_write_u8(TWL4030_MODULE_LED, 0x11, TWL_LED_LEDEN);
	twl4030_i2c_write_u8(TWL4030_MODULE_PWMA, 0x01, TWL_PWMA_PWMAON);
	twl4030_i2c_write_u8(TWL4030_MODULE_PWMA, 0x02, TWL_PWMA_PWMAOFF);
	bklight_level = 100;

	omap_set_gpio_dataout(LCD_PANEL_RESB, 1);
	omap_set_gpio_dataout(LCD_PANEL_INI, 1);
	omap_set_gpio_dataout(LCD_PANEL_QVGA, 0);
	omap_set_gpio_dataout(LCD_PANEL_LR, 1);
	omap_set_gpio_dataout(LCD_PANEL_UD, 1);

	return 0;
}

static void omap3evm_panel_cleanup(struct lcd_panel *panel)
{
}

static int omap3evm_panel_enable(struct lcd_panel *panel)
{
	omap_set_gpio_dataout(LCD_PANEL_ENABLE_GPIO, 0);
	return 0;
}

static void omap3evm_panel_disable(struct lcd_panel *panel)
{
	omap_set_gpio_dataout(LCD_PANEL_ENABLE_GPIO, 1);
}

static unsigned long omap3evm_panel_get_caps(struct lcd_panel *panel)
{
	return 0;
}

static int omap3evm_bklight_setlevel(struct lcd_panel *panel,
						unsigned int level)
{
	u8 c;
	if ((level >= 0) && (level <= 100)) {
		c = (125 * (100 - level)) / 100 + 2;
		twl4030_i2c_write_u8(TWL4030_MODULE_PWMA, c, TWL_PWMA_PWMAOFF);
		bklight_level = level;
	}
	return 0;
}

static unsigned int omap3evm_bklight_getlevel(struct lcd_panel *panel)
{
	return bklight_level;
}

static unsigned int omap3evm_bklight_getmaxlevel(struct lcd_panel *panel)
{
	return 100;
}

struct lcd_panel omap3evm_panel = {
	.name		= "omap3evm",
	.config		= OMAP_LCDC_PANEL_TFT | OMAP_LCDC_INV_VSYNC |
			  OMAP_LCDC_INV_HSYNC,

	.bpp		= 16,
	.data_lines	= 18,
	.x_res		= LCD_XRES,
	.y_res		= LCD_YRES,
	.hsw		= 3,		/* hsync_len (4) - 1 */
	.hfp		= 3,		/* right_margin (4) - 1 */
	.hbp		= 39,		/* left_margin (40) - 1 */
	.vsw		= 1,		/* vsync_len (2) - 1 */
	.vfp		= 2,		/* lower_margin */
	.vbp		= 7,		/* upper_margin (8) - 1 */

	.pixel_clock	= LCD_PIXCLOCK,

	.init		= omap3evm_panel_init,
	.cleanup	= omap3evm_panel_cleanup,
	.enable		= omap3evm_panel_enable,
	.disable	= omap3evm_panel_disable,
	.get_caps	= omap3evm_panel_get_caps,
	.set_bklight_level      = omap3evm_bklight_setlevel,
	.get_bklight_level      = omap3evm_bklight_getlevel,
	.get_bklight_max        = omap3evm_bklight_getmaxlevel,
};

static int omap3evm_panel_probe(struct platform_device *pdev)
{
	omapfb_register_panel(&omap3evm_panel);
	return 0;
}

static int omap3evm_panel_remove(struct platform_device *pdev)
{
	return 0;
}

static int omap3evm_panel_suspend(struct platform_device *pdev,
				   pm_message_t mesg)
{
	return 0;
}

static int omap3evm_panel_resume(struct platform_device *pdev)
{
	return 0;
}

struct platform_driver omap3evm_panel_driver = {
	.probe		= omap3evm_panel_probe,
	.remove		= omap3evm_panel_remove,
	.suspend	= omap3evm_panel_suspend,
	.resume		= omap3evm_panel_resume,
	.driver		= {
		.name	= "omap3evm_lcd",
		.owner	= THIS_MODULE,
	},
};

static int __init omap3evm_panel_drv_init(void)
{
	return platform_driver_register(&omap3evm_panel_driver);
}

static void __exit omap3evm_panel_drv_exit(void)
{
	platform_driver_unregister(&omap3evm_panel_driver);
}

module_init(omap3evm_panel_drv_init);
module_exit(omap3evm_panel_drv_exit);
