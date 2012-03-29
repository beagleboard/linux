/*
 * linux/drivers/video/st7735fb.c -- FB driver for ST7735 LCD controller
 * Layout is based on skeletonfb.c by James Simmons and Geert Uytterhoeven.
 *
 * Copyright (C) 2011, Matt Porter
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

#include <video/st7735fb.h>

static struct st7735_function st7735_cfg_script[] = {
	{ ST7735_START, ST7735_START},
	{ ST7735_CMD, ST7735_SWRESET},
	{ ST7735_DELAY, 150},
	{ ST7735_CMD, ST7735_SLPOUT},
	{ ST7735_DELAY, 500},
	{ ST7735_CMD, ST7735_FRMCTR1},
	{ ST7735_DATA, 0x01},
	{ ST7735_DATA, 0x2c},
	{ ST7735_DATA, 0x2d},
	{ ST7735_CMD, ST7735_FRMCTR2},
	{ ST7735_DATA, 0x01},
	{ ST7735_DATA, 0x2c},
	{ ST7735_DATA, 0x2d},
	{ ST7735_CMD, ST7735_FRMCTR3},
	{ ST7735_DATA, 0x01},
	{ ST7735_DATA, 0x2c},
	{ ST7735_DATA, 0x2d},
	{ ST7735_DATA, 0x01},
	{ ST7735_DATA, 0x2c},
	{ ST7735_DATA, 0x2d},
	{ ST7735_CMD, ST7735_INVCTR},
	{ ST7735_DATA, 0x07},
	{ ST7735_CMD, ST7735_PWCTR1},
	{ ST7735_DATA, 0xa2},
	{ ST7735_DATA, 0x02},
	{ ST7735_DATA, 0x84},
	{ ST7735_CMD, ST7735_PWCTR2},
	{ ST7735_DATA, 0xc5},
	{ ST7735_CMD, ST7735_PWCTR3},
	{ ST7735_DATA, 0x0a},
	{ ST7735_DATA, 0x00},
	{ ST7735_CMD, ST7735_PWCTR4},
	{ ST7735_DATA, 0x8a},
	{ ST7735_DATA, 0x2a},
	{ ST7735_CMD, ST7735_PWCTR5},
	{ ST7735_DATA, 0x8a},
	{ ST7735_DATA, 0xee},
	{ ST7735_CMD, ST7735_VMCTR1},
	{ ST7735_DATA, 0x0e},
	{ ST7735_CMD, ST7735_INVOFF},
	{ ST7735_CMD, ST7735_MADCTL},
	{ ST7735_DATA, 0xc8},
	{ ST7735_CMD, ST7735_COLMOD},
	{ ST7735_DATA, 0x05},
	{ ST7735_CMD, ST7735_CASET},
	{ ST7735_DATA, 0x00},
	{ ST7735_DATA, 0x00},
	{ ST7735_DATA, 0x00},
	{ ST7735_DATA, 0x00},
	{ ST7735_DATA, 0x7f},
	{ ST7735_CMD, ST7735_RASET},
	{ ST7735_DATA, 0x00},
	{ ST7735_DATA, 0x00},
	{ ST7735_DATA, 0x00},
	{ ST7735_DATA, 0x00},
	{ ST7735_DATA, 0x9f},
	{ ST7735_CMD, ST7735_GMCTRP1},
	{ ST7735_DATA, 0x02},
	{ ST7735_DATA, 0x1c},
	{ ST7735_DATA, 0x07},
	{ ST7735_DATA, 0x12},
	{ ST7735_DATA, 0x37},
	{ ST7735_DATA, 0x32},
	{ ST7735_DATA, 0x29},
	{ ST7735_DATA, 0x2d},
	{ ST7735_DATA, 0x29},
	{ ST7735_DATA, 0x25},
	{ ST7735_DATA, 0x2b},
	{ ST7735_DATA, 0x39},
	{ ST7735_DATA, 0x00},
	{ ST7735_DATA, 0x01},
	{ ST7735_DATA, 0x03},
	{ ST7735_DATA, 0x10},
	{ ST7735_CMD, ST7735_GMCTRN1},
	{ ST7735_DATA, 0x03},
	{ ST7735_DATA, 0x1d},
	{ ST7735_DATA, 0x07},
	{ ST7735_DATA, 0x06},
	{ ST7735_DATA, 0x2e},
	{ ST7735_DATA, 0x2c},
	{ ST7735_DATA, 0x29},
	{ ST7735_DATA, 0x2d},
	{ ST7735_DATA, 0x2e},
	{ ST7735_DATA, 0x2e},
	{ ST7735_DATA, 0x37},
	{ ST7735_DATA, 0x3f},
	{ ST7735_DATA, 0x00},
	{ ST7735_DATA, 0x00},
	{ ST7735_DATA, 0x02},
	{ ST7735_DATA, 0x10},
	{ ST7735_CMD, ST7735_DISPON},
	{ ST7735_DELAY, 100},
	{ ST7735_CMD, ST7735_NORON},
	{ ST7735_DELAY, 10},
	{ ST7735_END, ST7735_END},
};

static struct fb_fix_screeninfo st7735fb_fix __devinitdata = {
	.id =		"ST7735", 
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_PSEUDOCOLOR,
	.xpanstep =	0,
	.ypanstep =	0,
	.ywrapstep =	0, 
	.line_length =	WIDTH*BPP/8,
	.accel =	FB_ACCEL_NONE,
};

static struct fb_var_screeninfo st7735fb_var __devinitdata = {
	.xres =			WIDTH,
	.yres =			HEIGHT,
	.xres_virtual =		WIDTH,
	.yres_virtual =		HEIGHT,
	.bits_per_pixel =	BPP,
	.nonstd	=		1,
};

static int st7735_write(struct st7735fb_par *par, u8 data)
{
	par->buf[0] = data;

	return spi_write(par->spi, par->buf, 1);
}

static void st7735_write_data(struct st7735fb_par *par, u8 data)
{
	int ret = 0;

	/* Set data mode */
	gpio_set_value(par->dc, 1);

	ret = st7735_write(par, data);
	if (ret < 0)
		pr_err("%s: write data %02x failed with status %d\n",
			par->info->fix.id, data, ret);
}

static int st7735_write_data_buf(struct st7735fb_par *par,
					u8 *txbuf, int size)
{
	/* Set data mode */
	gpio_set_value(par->dc, 1);

	/* Write entire buffer */
	return spi_write(par->spi, txbuf, size);
}

static void st7735_write_cmd(struct st7735fb_par *par, u8 data)
{
	int ret = 0;

	/* Set command mode */
	gpio_set_value(par->dc, 0);

	ret = st7735_write(par, data);
	if (ret < 0)
		pr_err("%s: write command %02x failed with status %d\n",
			par->info->fix.id, data, ret);
}

static void st7735_run_cfg_script(struct st7735fb_par *par)
{
	int i = 0;
	int end_script = 0;

	do {
		switch (st7735_cfg_script[i].cmd)
		{
		case ST7735_START:
			break;
		case ST7735_CMD:
			st7735_write_cmd(par,
				st7735_cfg_script[i].data & 0xff);
			break;
		case ST7735_DATA:
			st7735_write_data(par,
				st7735_cfg_script[i].data & 0xff);
			break;
		case ST7735_DELAY:
			mdelay(st7735_cfg_script[i].data);
			break;
		case ST7735_END:
			end_script = 1;
		}
		i++;
	} while (!end_script);
}

static void st7735_set_addr_win(struct st7735fb_par *par,
				int xs, int ys, int xe, int ye)
{
	st7735_write_cmd(par, ST7735_CASET);
	st7735_write_data(par, 0x00);
	st7735_write_data(par, xs+2);
	st7735_write_data(par, 0x00);
	st7735_write_data(par, xe+2);
	st7735_write_cmd(par, ST7735_RASET);
	st7735_write_data(par, 0x00);
	st7735_write_data(par, ys+1);
	st7735_write_data(par, 0x00);
	st7735_write_data(par, ye+1);
}

static void st7735_reset(struct st7735fb_par *par)
{
	/* Reset controller */
	gpio_set_value(par->rst, 0);
	udelay(10);
	gpio_set_value(par->rst, 1);
	mdelay(120);
}

static void st7735fb_update_display(struct st7735fb_par *par)
{
	int ret = 0;
	u16 *vmem;
#ifdef __LITTLE_ENDIAN
	int i;
	u16 *vmem16 = (u16 *)par->info->screen_base;
	vmem = par->ssbuf;

	for (i=0; i<WIDTH*HEIGHT*BPP/8/2; i++)
		vmem[i] = swab16(vmem16[i]);
#else
	vmem = (u16 *)par->info->screen_base;
#endif

	/* Set row/column data window */
	st7735_set_addr_win(par, 0, 0, WIDTH-1, HEIGHT-1);

	/* Internal RAM write command */
	st7735_write_cmd(par, ST7735_RAMWR);

	/* Blast framebuffer to ST7735 internal display RAM */
	ret = st7735_write_data_buf(par, (u8 *)vmem, WIDTH*HEIGHT*BPP/8);
	if (ret < 0)
		pr_err("%s: spi_write failed to update display buffer\n",
			par->info->fix.id);
}

static void st7735fb_deferred_io(struct fb_info *info,
				struct list_head *pagelist)
{
	st7735fb_update_display(info->par);
}

static int st7735fb_init_display(struct st7735fb_par *par)
{
	/* TODO: Need some error checking on gpios */

        /* Request GPIOs and initialize to default values */
        gpio_request_one(par->rst, GPIOF_OUT_INIT_HIGH,
			"ST7735 Reset Pin");
        gpio_request_one(par->dc, GPIOF_OUT_INIT_LOW,
			"ST7735 Data/Command Pin");

	st7735_reset(par);

	st7735_run_cfg_script(par);

	return 0;
}

void st7735fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	struct st7735fb_par *par = info->par;

	sys_fillrect(info, rect);

	st7735fb_update_display(par);
}

void st7735fb_copyarea(struct fb_info *info, const struct fb_copyarea *area) 
{
	struct st7735fb_par *par = info->par;

	sys_copyarea(info, area);

	st7735fb_update_display(par);
}

void st7735fb_imageblit(struct fb_info *info, const struct fb_image *image) 
{
	struct st7735fb_par *par = info->par;

	sys_imageblit(info, image);

	st7735fb_update_display(par);
}

static ssize_t st7735fb_write(struct fb_info *info, const char __user *buf,
		size_t count, loff_t *ppos)
{
	struct st7735fb_par *par = info->par;
	unsigned long p = *ppos;
	void *dst;
	int err = 0;
	unsigned long total_size;

	if (info->state != FBINFO_STATE_RUNNING)
		return -EPERM;

	total_size = info->fix.smem_len;

	if (p > total_size)
		return -EFBIG;

	if (count > total_size) {
		err = -EFBIG;
		count = total_size;
	}

	if (count + p > total_size) {
		if (!err)
			err = -ENOSPC;

		count = total_size - p;
	}

	dst = (void __force *) (info->screen_base + p);

	if (copy_from_user(dst, buf, count))
		err = -EFAULT;

	if  (!err)
		*ppos += count;

	st7735fb_update_display(par);

	return (err) ? err : count;
}

static struct fb_ops st7735fb_ops = {
	.owner		= THIS_MODULE,
	.fb_read	= fb_sys_read,
	.fb_write	= st7735fb_write,
	.fb_fillrect	= st7735fb_fillrect,
	.fb_copyarea	= st7735fb_copyarea,
	.fb_imageblit	= st7735fb_imageblit,
};

static struct fb_deferred_io st7735fb_defio = {
	.delay		= HZ/20,
	.deferred_io	= st7735fb_deferred_io,
};

static int __devinit st7735fb_probe (struct spi_device *spi)
{
	int chip = spi_get_device_id(spi)->driver_data;
	struct st7735fb_platform_data *pdata = spi->dev.platform_data;
	int vmem_size = WIDTH*HEIGHT*BPP/8;
	u8 *vmem;
	struct fb_info *info;
	struct st7735fb_par *par;
	int retval = -ENOMEM;

	if (chip != ST7735_DISPLAY_AF_TFT18) {
		pr_err("%s: only the %s device is supported\n", DRVNAME,
			to_spi_driver(spi->dev.driver)->id_table->name);
		return -EINVAL;
	}

	if (!pdata) {
		pr_err("%s: platform data required for rst and dc info\n",
			DRVNAME);
		return -EINVAL;
	}

#ifdef __LITTLE_ENDIAN
	vmem = (u8 *)vmalloc(vmem_size);
#else
	vmem = (u8 *)kmalloc(vmem_size, GFP_KERNEL);
#endif
	if (!vmem)
		return retval;

	info = framebuffer_alloc(sizeof(struct st7735fb_par), &spi->dev);
	if (!info)
		goto fballoc_fail;

	info->screen_base = (u8 __force __iomem *)vmem;
	info->fbops = &st7735fb_ops;
	info->fix = st7735fb_fix;
	info->fix.smem_len = vmem_size;
	info->var = st7735fb_var;
	/* Choose any packed pixel format as long as it's RGB565 */
	info->var.red.offset = 11;
	info->var.red.length = 5;
	info->var.green.offset = 5;
	info->var.green.length = 6;
	info->var.blue.offset = 0;
	info->var.blue.length = 5;
	info->var.transp.offset = 0;
	info->var.transp.length = 0;
	info->flags = FBINFO_FLAG_DEFAULT |
#ifdef __LITTLE_ENDIAN
			FBINFO_FOREIGN_ENDIAN |
#endif
			FBINFO_VIRTFB;

	info->fbdefio = &st7735fb_defio;
	fb_deferred_io_init(info);

	par = info->par;
	par->info = info;
	par->spi = spi;
	par->rst = pdata->rst_gpio;
	par->dc = pdata->dc_gpio;
	par->buf = kmalloc(1, GFP_KERNEL);

#ifdef __LITTLE_ENDIAN
	/* Allocated swapped shadow buffer */
	par->ssbuf = kmalloc(vmem_size, GFP_KERNEL);
	if (!par->ssbuf)
		return retval;
#endif

	retval = register_framebuffer(info);
	if (retval < 0)
		goto fbreg_fail;

	spi_set_drvdata(spi, info);

	retval = st7735fb_init_display(par);
	if (retval < 0)
		goto init_fail;

	printk(KERN_INFO
		"fb%d: %s frame buffer device,\n\tusing %d KiB of video memory\n",
		info->node, info->fix.id, vmem_size);

	return 0;


	/* TODO: release gpios on fail */
init_fail:
	spi_set_drvdata(spi, NULL);

fbreg_fail:
	framebuffer_release(info);

fballoc_fail:
#ifdef __LITTLE_ENDIAN
	vfree(vmem);
#else
	kfree(vmem);
#endif

	return retval;
}

static int __devexit st7735fb_remove(struct spi_device *spi)
{
	struct fb_info *info = spi_get_drvdata(spi);

	spi_set_drvdata(spi, NULL);

	if (info) {
		unregister_framebuffer(info);
		vfree(info->screen_base);	
		framebuffer_release(info);
	}

	/* TODO: release gpios */

	return 0;
}

static const struct spi_device_id st7735fb_ids[] = {
	{ "adafruit_tft18", ST7735_DISPLAY_AF_TFT18 },
	{ },
};

MODULE_DEVICE_TABLE(spi, st7735fb_ids);

static struct spi_driver st7735fb_driver = {
	.driver = {
		.name   = "st7735fb",
		.owner  = THIS_MODULE,
	},
	.id_table = st7735fb_ids,
	.probe  = st7735fb_probe,
	.remove = __devexit_p(st7735fb_remove),
};

static int __init st7735fb_init(void)
{
	return spi_register_driver(&st7735fb_driver);
}

static void __exit st7735fb_exit(void)
{
	spi_unregister_driver(&st7735fb_driver);
}

/* ------------------------------------------------------------------------- */

module_init(st7735fb_init);
module_exit(st7735fb_exit);

MODULE_DESCRIPTION("FB driver for ST7735 display controller");
MODULE_AUTHOR("Matt Porter");
MODULE_LICENSE("GPL");
