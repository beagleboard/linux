/*
 * File: drivers/video/omap/lcd-p2.c
 *
 * LCD panel support for the TI OMAP P2 board
 *
 * Authors:
 *   jekyll <jekyll@mail.jekyll.idv.tw>
 *   B Jp <lastjp_fr@yahoo.fr>
 *   Brian Swetland <swetland@android.com>
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
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>
#include <asm/arch/omapfb.h>

/*
 * File: epson-md-tft.h
 *
 * This file contains definitions for Epsons MD-TF LCD Module
 *
 * Copyright (C) 2004 MPC-Data Limited  (http://www.mpc-data.co.uk)
 * Author: Dave Peverley <dpeverley at mpc-data.co.uk>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS  PROVIDED  ``AS  IS''  AND   ANY  EXPRESS  OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Please report all bugs and problems to the author.
 *
 */

/* LCD uWire commands & params
 * All values from Epson
 */
#define LCD_DISON 0xAF
#define LCD_DISOFF 0xAE
#define LCD_DISNOR 0xA6
#define LCD_DISINV 0xA7
#define LCD_DISCTL 0xCA
#define LCD_GCP64 0xCB
#define LCD_GCP16 0xCC
#define LCD_GSSET 0xCD
#define LCD_SLPIN 0x95
#define LCD_SLPOUT 0x94
#define LCD_SD_PSET 0x75
#define LCD_MD_PSET 0x76
#define LCD_SD_CSET 0x15
#define LCD_MD_CSET 0x16
#define LCD_DATCTL 0xBC
#define LCD_RAMWR 0x5C
#define LCD_RAMRD 0x5D
#define LCD_PTLIN 0xA8
#define LCD_PTLOUT 0xA9
#define LCD_ASCSET 0xAA
#define LCD_SCSTART 0xAB
#define LCD_VOLCTL 0xC6
#define LCD_NOP 0x25
#define LCD_OSCISEL 0x7
#define LCD_3500KSET 0xD1
#define LCD_3500KEND 0xD2
#define LCD_14MSET 0xD3
#define LCD_14MEND 0xD4

#define INIT_3500KSET 0x45
#define INIT_14MSET 0x4B
#define INIT_DATCTL 0x08 /* 6.6.6 bits for D-Sample */

#define INIT_OSCISEL 0x05

#define INIT_VOLCTL 0x77 /* Nominel "volume" */

#define INIT_VOLCTL_Ton 0x98 /* Activate power-IC timer */
#define INIT_GSSET 0x00

const unsigned short INIT_DISCTL[11] =
{
	0xDE, 0x01, 0x64, 0x00, 0x1B, 0xF4, 0x00, 0xDC, 0x00, 0x02, 0x00
};

const unsigned short INIT_GCP64[126] =
{
	0x3B,0x00,0x42,0x00,0x4A,0x00,0x51,0x00,
	0x58,0x00,0x5F,0x00,0x66,0x00,0x6E,0x00,
	0x75,0x00,0x7C,0x00,0x83,0x00,0x8A,0x00,
	0x92,0x00,0x99,0x00,0xA0,0x00,0xA7,0x00,
	0xAE,0x00,0xB6,0x00,0xBD,0x00,0xC4,0x00,
	0xCB,0x00,0xD2,0x00,0xDA,0x00,0xE1,0x00,
	0xE8,0x00,0xEF,0x00,0xF6,0x00,0xFE,0x00,
	0x05,0x01,0x0C,0x01,0x13,0x01,0x1A,0x01,
	0x22,0x01,0x29,0x01,0x30,0x01,0x37,0x01,
	0x3E,0x01,0x46,0x01,0x4D,0x01,0x54,0x01,
	0x5B,0x01,0x62,0x01,0x6A,0x01,0x71,0x01,
	0x78,0x01,0x7F,0x01,0x86,0x01,0x8E,0x01,
	0x95,0x01,0x9C,0x01,0xA3,0x01,0xAA,0x01,
	0xB2,0x01,0xB9,0x01,0xC0,0x01,0xC7,0x01,
	0xCE,0x01,0xD6,0x01,0xDD,0x01,0xE4,0x01,
	0xEB,0x01,0xF2,0x01,0xFA,0x01
};

const unsigned short INIT_GCP16[15] =
{
	0x1A,0x31,0x48,0x54,0x5F,0x67,0x70,0x76,0x7C,0x80,0x83,0x84,0x85,0x87,0x96
};

const unsigned short INIT_MD_PSET[4] = { 0, 0, 219, 0 };
const unsigned short INIT_MD_CSET[4] = { 2, 0, 177, 0 };

const unsigned short INIT_SD_PSET[4] = { 0x00, 0x01, 0x00, 0x01 };
const unsigned short INIT_SD_CSET[4] = { 0x00, 0x02, 0x00, 0x02 };

const unsigned short INIT_ASCSET[7] = { 0x00, 0x00, 0xDB, 0x00, 0xDC, 0x00, 0x01 };
const unsigned short INIT_SCSTART[2] = { 0x00, 0x00 };

/* ----- end of epson_md_tft.h ----- */


#include "../drivers/ssi/omap-uwire.h"

#define LCD_UWIRE_CS 0

static int p2_panel_init(struct lcd_panel *panel, struct omapfb_device *fbdev)
{
	return 0;
}

static void p2_panel_cleanup(struct lcd_panel *panel)
{
}

static int p2_panel_enable(struct lcd_panel *panel)
{
	int i;
	unsigned long value;

		/* thwack the reset line */
	omap_set_gpio_direction(19, 0);
	omap_set_gpio_dataout(19, 0);
	mdelay(2);
	omap_set_gpio_dataout(19, 1);

		/* bits 31:28 -> 0  LCD_PXL_15 .. 12 */
	value = omap_readl(OMAP730_IO_CONF_3) & 0x0FFFFFFF;
	omap_writel(value, OMAP730_IO_CONF_3);

		/* bits 19:0 -> 0  LCD_VSYNC, AC, PXL_0, PCLK, HSYNC,
		**                 PXL_9..1, PXL_10, PXL_11
		*/
	value = omap_readl(OMAP730_IO_CONF_4) & 0xFFF00000;
	omap_writel(value, OMAP730_IO_CONF_4);

	omap_uwire_configure_mode(0,16);

	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_DISOFF, 9, 0,NULL,1);
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_SLPIN, 9, 0,NULL,1);
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_DISNOR, 9, 0,NULL,1);
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_GSSET, 9, 0,NULL,1);
	omap_uwire_data_transfer(LCD_UWIRE_CS, (INIT_GSSET | 0x100), 9, 0,NULL,1);

	/* DISCTL */
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_DISCTL, 9, 0,NULL,1);
	for (i = 0; i < (sizeof(INIT_DISCTL)/sizeof(unsigned short)); i++)
		omap_uwire_data_transfer(LCD_UWIRE_CS, (INIT_DISCTL[i] | 0x100), 9, 0,NULL,1);

	/* GCP64 */
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_GCP64, 9, 0,NULL,1);
	for (i = 0; i < (sizeof(INIT_GCP64)/sizeof(unsigned short)); i++)
		omap_uwire_data_transfer(LCD_UWIRE_CS, (INIT_GCP64[i] | 0x100), 9, 0,NULL,1);

	/* GCP16 */
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_GCP16, 9, 0,NULL,1);
	for (i = 0; i < (sizeof(INIT_GCP16)/sizeof(unsigned short)); i++)
		omap_uwire_data_transfer(LCD_UWIRE_CS, (INIT_GCP16[i] | 0x100), 9, 0,NULL,1);

	/* MD_CSET */
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_MD_CSET, 9, 0,NULL,1);
	for (i = 0; i < (sizeof(INIT_MD_CSET)/sizeof(unsigned short)); i++)
		omap_uwire_data_transfer(LCD_UWIRE_CS, (INIT_MD_CSET[i] | 0x100), 9, 0,NULL,1);

	/* MD_PSET */
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_MD_PSET, 9, 0,NULL,1);
	for (i = 0; i < (sizeof(INIT_MD_PSET)/sizeof(unsigned short)); i++)
		omap_uwire_data_transfer(LCD_UWIRE_CS, (INIT_MD_PSET[i] | 0x100), 9, 0,NULL,1);

	/* SD_CSET */
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_SD_CSET, 9, 0,NULL,1);
	for (i = 0; i < (sizeof(INIT_SD_CSET)/sizeof(unsigned short)); i++)
		omap_uwire_data_transfer(LCD_UWIRE_CS, (INIT_SD_CSET[i] | 0x100), 9, 0,NULL,1);

	/* SD_PSET */
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_SD_PSET, 9, 0,NULL,1);
	for (i = 0; i < (sizeof(INIT_SD_PSET)/sizeof(unsigned short)); i++)
		omap_uwire_data_transfer(LCD_UWIRE_CS, (INIT_SD_PSET[i] | 0x100), 9, 0,NULL,1);

	/* DATCTL */
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_DATCTL, 9, 0,NULL,1);
	omap_uwire_data_transfer(LCD_UWIRE_CS, (INIT_DATCTL | 0x100), 9, 0,NULL,1);

	/* OSSISEL = d'5 */
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_OSCISEL, 9, 0,NULL,1);
	omap_uwire_data_transfer(LCD_UWIRE_CS, (INIT_OSCISEL | 0x100), 9, 0,NULL,1);

	/* 14MSET = d'74 */
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_14MSET, 9, 0,NULL,1);
	omap_uwire_data_transfer(LCD_UWIRE_CS, (INIT_14MSET | 0x100), 9, 0,NULL,1);

	/* 14MEND */
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_14MEND, 9, 0,NULL,1);

	/* 3500KSET = d'69 */
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_3500KSET, 9, 0,NULL,1);
	omap_uwire_data_transfer(LCD_UWIRE_CS, (INIT_3500KSET | 0x100), 9, 0,NULL,1);

	/* 3500KEND */
	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_3500KEND, 9, 0,NULL,1);

	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_SLPOUT, 9, 0,NULL,1);

	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_VOLCTL, 9, 0,NULL,1);
	omap_uwire_data_transfer(LCD_UWIRE_CS, (INIT_VOLCTL_Ton | 0x100), 9, 0,NULL,1);

	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_VOLCTL, 9, 0,NULL,1);

	omap_uwire_data_transfer(LCD_UWIRE_CS, (INIT_VOLCTL | 0x100), 9, 0,NULL,1);

	omap_uwire_data_transfer(LCD_UWIRE_CS, LCD_DISON, 9, 0,NULL,1);

	/* enable backlight */
	omap_set_gpio_direction(134, 0);
	omap_set_gpio_dataout(134, 1);

	return 0;
}

static void p2_panel_disable(struct lcd_panel *panel)
{
}

static unsigned long p2_panel_get_caps(struct lcd_panel *panel)
{
	return 0;
}

struct lcd_panel p2_panel = {
	.name		= "p2",
	.config		= OMAP_LCDC_PANEL_TFT | OMAP_LCDC_INV_PIX_CLOCK,

	.bpp		= 16,
	.data_lines	= 16,
	.x_res		= 176,
	.y_res		= 220,
	.pixel_clock	= 12500,
	.hsw		= 5,
	.hfp		= 1,
	.hbp		= 1,
	.vsw		= 2,
	.vfp		= 12,
	.vbp		= 1,

	.init		= p2_panel_init,
	.cleanup	= p2_panel_cleanup,
	.enable		= p2_panel_enable,
	.disable	= p2_panel_disable,
	.get_caps	= p2_panel_get_caps,
};

static int p2_panel_probe(struct platform_device *pdev)
{
	omapfb_register_panel(&p2_panel);
	return 0;
}

static int p2_panel_remove(struct platform_device *pdev)
{
	return 0;
}

static int p2_panel_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int p2_panel_resume(struct platform_device *pdev)
{
	return 0;
}

struct platform_driver p2_panel_driver = {
	.probe		= p2_panel_probe,
	.remove		= p2_panel_remove,
	.suspend	= p2_panel_suspend,
	.resume		= p2_panel_resume,
	.driver		= {
		.name	= "lcd_p2",
		.owner	= THIS_MODULE,
	},
};

static int p2_panel_drv_init(void)
{
	return platform_driver_register(&p2_panel_driver);
}

static void p2_panel_drv_cleanup(void)
{
	platform_driver_unregister(&p2_panel_driver);
}

module_init(p2_panel_drv_init);
module_exit(p2_panel_drv_cleanup);

