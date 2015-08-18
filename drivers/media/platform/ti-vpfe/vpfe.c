/*
 * TI VPFE capture Driver
 *
 * Copyright (C) 2013 - 2014 Texas Instruments, Inc.
 * Benoit Parrot <bparrot@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <media/v4l2-common.h>
#include <media/v4l2-of.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <linux/io.h>

#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>
#include <linux/gfp.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/clk-private.h>
#include <linux/pm_runtime.h>
#include "vpfe.h"

#define VPFE_MODULE_NAME "vpfe"
#define VPFE_VERSION "0.1.0"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level 0-8");

MODULE_DESCRIPTION("VPFE Video for Linux Capture Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
MODULE_VERSION(VPFE_VERSION);

#define MAX_WIDTH	4096
#define MAX_HEIGHT	4096

#define vpfe_dbg(level, dev, fmt, arg...)	\
		v4l2_dbg(level, debug, &dev->v4l2_dev, fmt, ##arg)
#define vpfe_info(dev, fmt, arg...)	\
		v4l2_info(&dev->v4l2_dev, fmt, ##arg)
#define vpfe_err(dev, fmt, arg...)	\
		v4l2_err(&dev->v4l2_dev, fmt, ##arg)

/* standard information */
struct vpfe_standard {
	v4l2_std_id std_id;
	unsigned int width;
	unsigned int height;
	struct v4l2_fract pixelaspect;
	/* 0 - progressive, 1 - interlaced */
	int frame_format;
};

/* MULTI: Global default OK for multi-instance use */
/* data structures */
const struct vpfe_standard vpfe_standards[] = {
	{V4L2_STD_525_60, 720, 480, {11, 10}, 1},
	{V4L2_STD_625_50, 720, 576, {54, 59}, 1},
};

/*
 * struct vpfe_fmt - VPFE media bus format information
 * @name: V4L2 format description
 * @code: V4L2 media bus format code
 * @shifted: V4L2 media bus format code for the same pixel layout but
 *	shifted to be 8 bits per pixel. =0 if format is not shiftable.
 * @pixelformat: V4L2 pixel format FCC identifier
 * @width: Bits per pixel (when transferred over a bus)
 * @bpp: Bytes per pixel (when stored in memory)
 */
struct bus_format {
	unsigned int width;
	unsigned int bpp;
};

struct vpfe_fmt {
	const char *name;
	u32 fourcc;
	enum v4l2_mbus_pixelcode code;
	struct bus_format l;
	struct bus_format s;
};

static struct vpfe_fmt formats[] = {
	{ /* 0 */
		.name		= "4:2:2, packed, YUYV",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.code		= V4L2_MBUS_FMT_YUYV8_2X8,
		.l.width	= 10,
		.l.bpp		= 4,
		.s.width	= 8,
		.s.bpp		= 2,
	},
	{
		.name		= "4:2:2, packed, UYVY",
		.fourcc		= V4L2_PIX_FMT_UYVY,
		.code		= V4L2_MBUS_FMT_UYVY8_2X8,
		.l.width	= 10,
		.l.bpp		= 4,
		.s.width	= 8,
		.s.bpp		= 2,
	},
	{
		.name		= "4:2:2, packed, YVYU",
		.fourcc		= V4L2_PIX_FMT_YVYU,
		.code		= V4L2_MBUS_FMT_YVYU8_2X8,
		.l.width	= 10,
		.l.bpp		= 4,
		.s.width	= 8,
		.s.bpp		= 2,
	},
	{
		.name		= "4:2:2, packed, VYUY",
		.fourcc		= V4L2_PIX_FMT_VYUY,
		.code		= V4L2_MBUS_FMT_VYUY8_2X8,
		.l.width	= 10,
		.l.bpp		= 4,
		.s.width	= 8,
		.s.bpp		= 2,
	},
	{
		.name		= "4:2:0 Y/CbCr co-planar",
		.fourcc		= V4L2_PIX_FMT_NV12,
		.code		= V4L2_MBUS_FMT_YDYUYDYV8_1X16,
		.l.width	= 10,
		.l.bpp		= 2,
		.s.width	= 8,
		.s.bpp		= 1,
	},
	{
		.name		= "4:2:0 YUV co-planar",
		.fourcc		= V4L2_PIX_FMT_YUV420,
		.code		= V4L2_MBUS_FMT_YDYUYDYV8_1X16,
		.l.width	= 10,
		.l.bpp		= 2,
		.s.width	= 8,
		.s.bpp		= 1,
	},

	{
		.name		= "RAW8 BGGR",
		.fourcc		= V4L2_PIX_FMT_SBGGR8,
		.code		= V4L2_MBUS_FMT_SBGGR8_1X8,
		.l.width	= 10,
		.l.bpp		= 2,
		.s.width	= 8,
		.s.bpp		= 1,
	},
	{
		.name		= "RAW8 GBRG",
		.fourcc		= V4L2_PIX_FMT_SGBRG8,
		.code		= V4L2_MBUS_FMT_SGBRG8_1X8,
		.l.width	= 10,
		.l.bpp		= 2,
		.s.width	= 8,
		.s.bpp		= 1,
	},
	{
		.name		= "RAW8 GRBG",
		.fourcc		= V4L2_PIX_FMT_SGRBG8,
		.code		= V4L2_MBUS_FMT_SGRBG8_1X8,
		.l.width	= 10,
		.l.bpp		= 2,
		.s.width	= 8,
		.s.bpp		= 1,
	},
	{
		.name		= "RAW8 RGGB",
		.fourcc		= V4L2_PIX_FMT_SRGGB8,
		.code		= V4L2_MBUS_FMT_SRGGB8_1X8,
		.l.width	= 10,
		.l.bpp		= 2,
		.s.width	= 8,
		.s.bpp		= 1,
	},

	{
		.name		= "RGB565 (LE)",
		.fourcc		= V4L2_PIX_FMT_RGB565,
		.code		= V4L2_MBUS_FMT_RGB565_2X8_LE,
		.l.width	= 10,
		.l.bpp		= 4,
		.s.width	= 8,
		.s.bpp		= 2,
	},
	{
		.name		= "RGB565 (BE)",
		.fourcc		= V4L2_PIX_FMT_RGB565X,
		.code		= V4L2_MBUS_FMT_RGB565_2X8_BE,
		.l.width	= 10,
		.l.bpp		= 4,
		.s.width	= 8,
		.s.bpp		= 2,
	},
};

/*
 * Find our format description corresponding to the passed v4l2_format
 */
static struct vpfe_fmt *find_format_by_code(unsigned int code)
{
	struct vpfe_fmt *fmt;
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(formats); k++) {
		fmt = &formats[k];
		if (fmt->code == code)
			return fmt;
	}

	return NULL;
}

static struct vpfe_fmt *find_format_by_pix(unsigned int pixelformat)
{
	struct vpfe_fmt *fmt;
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(formats); k++) {
		fmt = &formats[k];
		if (fmt->fourcc == pixelformat)
			return fmt;
	}

	return NULL;
}

/* map mbus_fmt to pixelformat */
static void mbus_to_pix(struct vpfe_device *dev,
		const struct v4l2_mbus_framefmt *mbus,
		struct v4l2_pix_format *pix, unsigned int *bpp)
{
	unsigned int bus_width =
			dev->current_subdev->isif_if_params.bus_width;
	struct vpfe_fmt *fmt;

	memset(pix, 0, sizeof(*pix));
	pix->width = mbus->width;
	pix->height = mbus->height;

	fmt = find_format_by_code(mbus->code);
	if (WARN_ON(fmt == NULL)) {
		pr_err("Invalid mbus code set\n");
		*bpp = 1;
		return;
	}

	pix->colorspace = mbus->colorspace;
	pix->field = mbus->field;
	pix->pixelformat = fmt->fourcc;
	*bpp = (bus_width == 10) ?  fmt->l.bpp : fmt->s.bpp;

	pix->bytesperline = pix->width * *bpp;
	/* pitch should be 32 bytes aligned */
	pix->bytesperline = ALIGN(pix->bytesperline, 32);
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_YUV420:
		pix->sizeimage = pix->bytesperline * pix->height +
				((pix->bytesperline * pix->height) >> 1);
		break;
	default:
		pix->sizeimage = pix->bytesperline * pix->height;
		break;
	}
}

/* map mbus_fmt to pixelformat */
static int pix_to_mbus(struct vpfe_device *dev,
		struct v4l2_pix_format *pix,
		struct v4l2_mbus_framefmt *mbus)
{
	struct vpfe_fmt *fmt;

	memset(mbus, 0, sizeof(*mbus));
	mbus->width = pix->width;
	mbus->height = pix->height;

	fmt = find_format_by_pix(pix->pixelformat);
	if (fmt == NULL) {
		/* default to first entry */
		vpfe_dbg(3, dev, "Invalid pixel code: %x, default used instead\n",
			pix->pixelformat);
		fmt = &formats[0];
	}

	mbus->code = fmt->code;
	mbus->colorspace = pix->colorspace;
	mbus->field = pix->field;

	return 0;
}

/*  Print Four-character-code (FOURCC) */
static char *print_fourcc(u32 fmt)
{
	static char code[5];
	code[0] = (unsigned char)(fmt & 0xff);
	code[1] = (unsigned char)((fmt>>8) & 0xff);
	code[2] = (unsigned char)((fmt>>16) & 0xff);
	code[3] = (unsigned char)((fmt>>24) & 0xff);
	code[4] = '\0';

	return code;
}

static int cmp_v4l2_format(const struct v4l2_format *lhs,
	const struct v4l2_format *rhs)
{
	return lhs->type == rhs->type &&
		lhs->fmt.pix.width == rhs->fmt.pix.width &&
		lhs->fmt.pix.height == rhs->fmt.pix.height &&
		lhs->fmt.pix.pixelformat == rhs->fmt.pix.pixelformat &&
		lhs->fmt.pix.field == rhs->fmt.pix.field &&
		lhs->fmt.pix.colorspace == rhs->fmt.pix.colorspace;
}

static inline u32 isif_read(struct vpfe_isif_device *isif,
	u32 offset)
{
	return ioread32(isif->isif_cfg.base_addr + offset);

}

static inline void isif_write(struct vpfe_isif_device *isif,
	u32 val, u32 offset)
{
	iowrite32(val, isif->isif_cfg.base_addr + offset);
}

static void isif_enable(struct vpfe_isif_device *isif, int flag)
{
	unsigned int pcr = 0;
	struct vpfe_device *dev = container_of(isif,
					struct vpfe_device, vpfe_isif);

	pcr = isif_read(isif, ISIF_PCR);

	if (flag) { /* Enable */
		pcr = 1;
	} else { /* Disable */
		pcr = 0;
	}

	vpfe_dbg(2, dev, "isif_enable(%d) pcr:%x\n", flag, pcr);
	isif_write(isif, pcr, ISIF_PCR);
}

static void isif_module_enable(struct vpfe_isif_device *isif, int flag)
{
	unsigned int cfg = 0;
	struct vpfe_device *dev = container_of(isif,
					struct vpfe_device, vpfe_isif);

	cfg = isif_read(isif, ISIF_VPFE_CONFIG);

	if (flag) { /* Enable */
		cfg = (ISIF_VPFE_CONFIG_VPFE_EN_ENABLE<<
				ISIF_VPFE_CONFIG_VPFE_EN_SHIFT);
	} else { /* Disable */
		cfg &= ~(ISIF_VPFE_CONFIG_VPFE_EN_ENABLE<<
				ISIF_VPFE_CONFIG_VPFE_EN_SHIFT);
	}

	vpfe_dbg(2, dev, "isif_module_enable(%d) cfg:%x\n", flag, cfg);
	isif_write(isif, cfg, ISIF_VPFE_CONFIG);
}

/*
 * isif_setwin()
 * This function will configure the window size
 * to be capture in ISIF reg
 */
static void isif_setwin(struct vpfe_isif_device *isif,
		struct v4l2_rect *image_win,
		enum isif_frmfmt frm_fmt,
		int bpp)
{
	int horz_start, horz_nr_pixels;
	int vert_start, vert_nr_lines;
	int val = 0, mid_img = 0;
	struct vpfe_device *dev = container_of(isif,
					struct vpfe_device, vpfe_isif);

	vpfe_dbg(2, dev, "isif_setwin: %dx%d (WxH)\n",
		image_win->width, image_win->height);
	vpfe_dbg(2, dev, "isif_setwin: top %d left %d\n",
		image_win->top, image_win->left);
	/*
	 * ppc - per pixel count. indicates how many pixels per cell
	 * output to SDRAM. example, for ycbcr, it is one y and one c, so 2.
	 * raw capture this is 1
	 */
	horz_start = image_win->left * bpp;
	horz_nr_pixels = (image_win->width * bpp) - 1;
	isif_write(isif,
		   (horz_start << ISIF_HORZ_INFO_SPH_SHIFT) | horz_nr_pixels,
		   ISIF_HORZ_INFO);

	vert_start = image_win->top;

	if (frm_fmt == ISIF_FRMFMT_INTERLACED) {
		vert_nr_lines = (image_win->height >> 1) - 1;
		vert_start >>= 1;
		/* Since first line doesn't have any data */
		vert_start += 1;
		/* configure VDINT0 */
		val = (vert_start << ISIF_VDINT_VDINT0_SHIFT);
		isif_write(isif, val, ISIF_VDINT);

	} else {
		/* Since first line doesn't have any data */
		vert_start += 1;
		vert_nr_lines = image_win->height - 1;
		/*
		 * configure VDINT0 and VDINT1. VDINT1 will be at half
		 * of image height
		 */
		mid_img = vert_start + (image_win->height / 2);
		val = (vert_start << ISIF_VDINT_VDINT0_SHIFT) |
		    (mid_img & ISIF_VDINT_VDINT1_MASK);
		isif_write(isif, val, ISIF_VDINT);
	}
	isif_write(isif,
		   (vert_start << ISIF_VERT_START_SLV0_SHIFT) | vert_start,
		   ISIF_VERT_START);
	isif_write(isif, vert_nr_lines, ISIF_VERT_LINES);
}

static void isif_readregs(struct vpfe_isif_device *isif)
{
	struct vpfe_device *dev = container_of(isif,
					struct vpfe_device, vpfe_isif);

	vpfe_dbg(3, dev, "ALAW: 0x%x\n",
		isif_read(isif, ISIF_ALAW));
	vpfe_dbg(3, dev, "CLAMP: 0x%x\n",
		isif_read(isif, ISIF_CLAMP));
	vpfe_dbg(3, dev, "DCSUB: 0x%x\n",
		isif_read(isif, ISIF_DCSUB));
	vpfe_dbg(3, dev, "BLKCMP: 0x%x\n",
		isif_read(isif, ISIF_BLKCMP));
	vpfe_dbg(3, dev, "COLPTN: 0x%x\n",
		isif_read(isif, ISIF_COLPTN));
	vpfe_dbg(3, dev, "HSIZE_OFF: 0x%x\n",
		isif_read(isif, ISIF_HSIZE_OFF));
	vpfe_dbg(3, dev, "SDOFST: 0x%x\n",
		isif_read(isif, ISIF_SDOFST));
	vpfe_dbg(3, dev, "SYN_MODE: 0x%x\n",
		isif_read(isif, ISIF_SYN_MODE));
	vpfe_dbg(3, dev, "HORZ_INFO: 0x%x\n",
		isif_read(isif, ISIF_HORZ_INFO));
	vpfe_dbg(3, dev, "VERT_START: 0x%x\n",
		isif_read(isif, ISIF_VERT_START));
	vpfe_dbg(3, dev, "VERT_LINES: 0x%x\n",
		isif_read(isif, ISIF_VERT_LINES));
}

static int validate_isif_param(struct vpfe_isif_device *isif,
	struct isif_config_params_raw *ccdcparam)
{
	struct vpfe_device *dev = container_of(isif,
					struct vpfe_device, vpfe_isif);
	if (ccdcparam->alaw.enable) {
		u8 max_gamma =
			isif_gamma_width_max_bit(ccdcparam->alaw.gamma_wd);
		u8 max_data =
			isif_data_size_max_bit(ccdcparam->data_sz);

		if ((ccdcparam->alaw.gamma_wd > ISIF_GAMMA_BITS_09_0) ||
		    (ccdcparam->alaw.gamma_wd < ISIF_GAMMA_BITS_15_6) ||
		    (max_gamma > max_data)) {
			vpfe_dbg(1, dev, "\nInvalid data line select");
			return -1;
		}
	}
	return 0;
}

static int isif_update_raw_params(struct vpfe_isif_device *isif,
	struct isif_config_params_raw *raw_params)
{
	struct isif_config_params_raw *config_params =
				&isif->isif_cfg.bayer.config_params;

	memcpy(config_params, raw_params, sizeof(*raw_params));

	return 0;
}

/*
 * isif_restore_defaults()
 * This function will write defaults to all ISIF registers
 */
static void isif_restore_defaults(struct vpfe_isif_device *isif)
{
	int i;

	/* disable ISIF */
	isif_enable(isif, 0);
	/* set all registers to default value */
	for (i = 4; i <= 0x94; i += 4)
		isif_write(isif, 0,  i);
	isif_write(isif, ISIF_NO_CULLING, ISIF_CULLING);
	isif_write(isif, ISIF_GAMMA_BITS_11_2, ISIF_ALAW);
}

static int isif_intr_status(struct vpfe_isif_device *isif);
static void isif_clear_intr(struct vpfe_isif_device *isif, int vdint);

static int isif_close(struct vpfe_isif_device *isif, struct device *dev)
{
	struct vpfe_device *vpfe_dev = container_of(isif,
					struct vpfe_device, vpfe_isif);
	int dma_cntl, i, pcr;

	vpfe_dbg(2, vpfe_dev, "isif_close\n");

	/* If the ISIF module is still busy wait for it to be done */
	i = 0;
	while (i++ < 10) {
		usleep_range(5000, 6000);
		pcr = isif_read(isif, ISIF_PCR);
		if (pcr) {
			/* make sure it it is disabled */
			isif_enable(isif, 0);
		} else {
			break;
		}
	}

	/* Disable ISIF by resetting all register to default POR values */
	isif_restore_defaults(isif);

	/* if DMA_CNTL overflow bit is set. Clear it
	 *  It appears to take a while for this to become quiescent ~20ms
	 */
	i = 0;
	while (i++ < 10) {
		dma_cntl = isif_read(isif, ISIF_DMA_CNTL);
		if (dma_cntl & ISIF_DMA_CNTL_OVERFLOW) {
			/* Clear the overflow bit */
			isif_write(isif, dma_cntl, ISIF_DMA_CNTL);
			usleep_range(5000, 6000);
		} else {
			break;
		}
	}

	/* Disabled the module at the CONFIG level */
	isif_module_enable(isif, 0);

	pm_runtime_put_sync(dev);
	return 0;
}

static int isif_open(struct vpfe_isif_device *isif, struct device *dev)
{
	struct vpfe_device *vpfe_dev = container_of(isif,
					struct vpfe_device, vpfe_isif);
	vpfe_dbg(2, vpfe_dev, "isif_open\n");

	pm_runtime_get_sync(dev);
	isif_module_enable(isif, 1);
	isif_restore_defaults(isif);

	return 0;
}

/* Parameter operations */
static int isif_set_params(struct vpfe_isif_device *isif, void __user *params)
{
	struct isif_config_params_raw isif_raw_params;
	struct vpfe_device *dev = container_of(isif,
					struct vpfe_device, vpfe_isif);
	int x;

	if (isif->isif_cfg.if_type != VPFE_RAW_BAYER)
		return -EINVAL;

	x = copy_from_user(&isif_raw_params, params, sizeof(isif_raw_params));
	if (x) {
		vpfe_dbg(1, dev,
			"isif_set_params: error in copying ccdc params, %d\n",
			x);
		return -EFAULT;
	}

	if (!validate_isif_param(isif, &isif_raw_params)) {
		if (!isif_update_raw_params(isif, &isif_raw_params))
			return 0;
	}
	return -EINVAL;
}

/*
 * isif_config_ycbcr()
 * This function will configure ISIF for YCbCr video capture
 */
static void isif_config_ycbcr(struct vpfe_isif_device *isif)
{
	struct isif_params_ycbcr *params = &isif->isif_cfg.ycbcr;
	struct vpfe_device *dev = container_of(isif,
					struct vpfe_device, vpfe_isif);
	u32 syn_mode;

	vpfe_dbg(3, dev, "isif_config_ycbcr:\n");
	/*
	 * first restore the ISIF registers to default values
	 * This is important since we assume default values to be set in
	 * a lot of registers that we didn't touch
	 */
	isif_restore_defaults(isif);

	/*
	 * configure pixel format, frame format, configure video frame
	 * format, enable output to SDRAM, enable internal timing generator
	 * and 8bit pack mode
	 */
	syn_mode = (((params->pix_fmt & ISIF_SYN_MODE_INPMOD_MASK) <<
		    ISIF_SYN_MODE_INPMOD_SHIFT) |
		    ((params->frm_fmt & ISIF_SYN_FLDMODE_MASK) <<
		    ISIF_SYN_FLDMODE_SHIFT) | ISIF_VDHDEN_ENABLE |
		    ISIF_WEN_ENABLE | ISIF_DATA_PACK_ENABLE);

	/* setup BT.656 sync mode */
	if (params->bt656_enable) {
		isif_write(isif, ISIF_REC656IF_BT656_EN, ISIF_REC656IF);

		/*
		 * configure the FID, VD, HD pin polarity,
		 * fld,hd pol positive, vd negative, 8-bit data
		 */
		syn_mode |= ISIF_SYN_MODE_VD_POL_NEGATIVE;
		if (isif->isif_cfg.if_type == VPFE_BT656_10BIT)
			syn_mode |= ISIF_SYN_MODE_10BITS;
		else
			syn_mode |= ISIF_SYN_MODE_8BITS;
	} else {
		/* y/c external sync mode */
		syn_mode |= (((params->fid_pol & ISIF_FID_POL_MASK) <<
			     ISIF_FID_POL_SHIFT) |
			     ((params->hd_pol & ISIF_HD_POL_MASK) <<
			     ISIF_HD_POL_SHIFT) |
			     ((params->vd_pol & ISIF_VD_POL_MASK) <<
			     ISIF_VD_POL_SHIFT));
	}
	isif_write(isif, syn_mode, ISIF_SYN_MODE);

	/* configure video window */
	isif_setwin(isif, &params->win, params->frm_fmt, params->bytesperpixel);

	/*
	 * configure the order of y cb cr in SDRAM, and disable latch
	 * internal register on vsync
	 */
	if (isif->isif_cfg.if_type == VPFE_BT656_10BIT)
		isif_write(isif,
			   (params->pix_order << ISIF_CCDCFG_Y8POS_SHIFT) |
			   ISIF_LATCH_ON_VSYNC_DISABLE |
			   ISIF_CCDCFG_BW656_10BIT,
			   ISIF_CCDCFG);
	else
		isif_write(isif,
			   (params->pix_order << ISIF_CCDCFG_Y8POS_SHIFT) |
			   ISIF_LATCH_ON_VSYNC_DISABLE, ISIF_CCDCFG);

	/*
	 * configure the horizontal line offset. This should be a
	 * on 32 byte boundary. So clear LSB 5 bits
	 */
	isif_write(isif, params->bytesperline, ISIF_HSIZE_OFF);

	/* configure the memory line offset */
	if (params->buf_type == ISIF_BUFTYPE_FLD_INTERLEAVED)
		/* two fields are interleaved in memory */
		isif_write(isif, ISIF_SDOFST_FIELD_INTERLEAVED, ISIF_SDOFST);

	vpfe_dbg(3, dev, "\nEnd of isif_config_ycbcr...\n");
}

static void isif_config_black_clamp(struct vpfe_isif_device *isif,
	struct isif_black_clamp *bclamp)
{
	u32 val;

	if (!bclamp->enable) {
		/* configure DCSub */
		val = (bclamp->dc_sub) & ISIF_BLK_DC_SUB_MASK;
		isif_write(isif, val, ISIF_DCSUB);
		isif_write(isif, ISIF_CLAMP_DEFAULT_VAL, ISIF_CLAMP);
		return;
	}
	/*
	 * Configure gain,  Start pixel, No of line to be avg,
	 * No of pixel/line to be avg, & Enable the Black clamping
	 */
	val = ((bclamp->sgain & ISIF_BLK_SGAIN_MASK) |
	       ((bclamp->start_pixel & ISIF_BLK_ST_PXL_MASK) <<
		ISIF_BLK_ST_PXL_SHIFT) |
	       ((bclamp->sample_ln & ISIF_BLK_SAMPLE_LINE_MASK) <<
		ISIF_BLK_SAMPLE_LINE_SHIFT) |
	       ((bclamp->sample_pixel & ISIF_BLK_SAMPLE_LN_MASK) <<
		ISIF_BLK_SAMPLE_LN_SHIFT) | ISIF_BLK_CLAMP_ENABLE);
	isif_write(isif, val, ISIF_CLAMP);
	/* If Black clamping is enable then make dcsub 0 */
	isif_write(isif, ISIF_DCSUB_DEFAULT_VAL, ISIF_DCSUB);
}

static void isif_config_black_compense(struct vpfe_isif_device *isif,
	struct isif_black_compensation *bcomp)
{
	u32 val;

	val = ((bcomp->b & ISIF_BLK_COMP_MASK) |
	      ((bcomp->gb & ISIF_BLK_COMP_MASK) <<
	       ISIF_BLK_COMP_GB_COMP_SHIFT) |
	      ((bcomp->gr & ISIF_BLK_COMP_MASK) <<
	       ISIF_BLK_COMP_GR_COMP_SHIFT) |
	      ((bcomp->r & ISIF_BLK_COMP_MASK) <<
	       ISIF_BLK_COMP_R_COMP_SHIFT));
	isif_write(isif, val, ISIF_BLKCMP);
}

/*
 * isif_config_raw()
 * This function will configure ISIF for Raw capture mode
 */
static void isif_config_raw(struct vpfe_isif_device *isif)
{
	struct isif_params_raw *params = &isif->isif_cfg.bayer;
	struct isif_config_params_raw *config_params =
				&isif->isif_cfg.bayer.config_params;
	struct vpfe_device *dev = container_of(isif,
					struct vpfe_device, vpfe_isif);
	unsigned int syn_mode = 0;
	unsigned int val;

	vpfe_dbg(3, dev, "isif_config_raw:\n");

	/* Reset ISIF */
	isif_restore_defaults(isif);

	/* Disable latching function registers on VSYNC  */
	isif_write(isif, ISIF_LATCH_ON_VSYNC_DISABLE, ISIF_CCDCFG);

	/*
	 * Configure the vertical sync polarity(SYN_MODE.VDPOL),
	 * horizontal sync polarity (SYN_MODE.HDPOL), frame id polarity
	 * (SYN_MODE.FLDPOL), frame format(progressive or interlace),
	 * data size(SYNMODE.DATSIZ), &pixel format (Input mode), output
	 * SDRAM, enable internal timing generator
	 */
	syn_mode =
		(((params->vd_pol & ISIF_VD_POL_MASK) << ISIF_VD_POL_SHIFT) |
		((params->hd_pol & ISIF_HD_POL_MASK) << ISIF_HD_POL_SHIFT) |
		((params->fid_pol & ISIF_FID_POL_MASK) << ISIF_FID_POL_SHIFT) |
		((params->frm_fmt & ISIF_FRM_FMT_MASK) << ISIF_FRM_FMT_SHIFT) |
		((config_params->data_sz & ISIF_DATA_SZ_MASK) <<
		ISIF_DATA_SZ_SHIFT) |
		((params->pix_fmt & ISIF_PIX_FMT_MASK) << ISIF_PIX_FMT_SHIFT) |
		ISIF_WEN_ENABLE | ISIF_VDHDEN_ENABLE);

	/* Enable and configure aLaw register if needed */
	if (config_params->alaw.enable) {
		val = ((config_params->alaw.gamma_wd &
		      ISIF_ALAW_GAMMA_WD_MASK) | ISIF_ALAW_ENABLE);
		isif_write(isif, val, ISIF_ALAW);
		vpfe_dbg(3, dev, "\nWriting 0x%x to ALAW...\n", val);
	}

	/* Configure video window */
	isif_setwin(isif, &params->win, params->frm_fmt, params->bytesperpixel);

	/* Configure Black Clamp */
	isif_config_black_clamp(isif, &config_params->blk_clamp);

	/* Configure Black level compensation */
	isif_config_black_compense(isif, &config_params->blk_comp);

	/* If data size is 8 bit then pack the data */
	if ((config_params->data_sz == ISIF_DATA_8BITS) ||
	    config_params->alaw.enable)
		syn_mode |= ISIF_DATA_PACK_ENABLE;

	/* Configure the color pattern according to mt9t001 sensor */
/*	isif_write(isif, ISIF_COLPTN_VAL, ISIF_COLPTN);
	dev_dbg(isif->isif_cfg.dev, "\nWriting 0xBB11BB11 to COLPTN...\n"); */

	/*
	 * Configure Horizontal offset register. If pack 8 is enabled then
	 * 1 pixel will take 1 byte
	 */

	isif_write(isif, params->bytesperline, ISIF_HSIZE_OFF);
	vpfe_dbg(3, dev, "Writing %d (%x) to HSIZE_OFF\n",
		params->bytesperline, params->bytesperline);

	/* Set value for SDOFST */
	if (params->frm_fmt == ISIF_FRMFMT_INTERLACED) {
		if (params->image_invert_enable) {
			/* For intelace inverse mode */
			isif_write(isif, ISIF_INTERLACED_IMAGE_INVERT,
				   ISIF_SDOFST);
		}

		else {
			/* For intelace non inverse mode */
			isif_write(isif, ISIF_INTERLACED_NO_IMAGE_INVERT,
				   ISIF_SDOFST);
		}
	} else if (params->frm_fmt == ISIF_FRMFMT_PROGRESSIVE) {
		isif_write(isif, ISIF_PROGRESSIVE_NO_IMAGE_INVERT,
			   ISIF_SDOFST);
	}

	isif_write(isif, syn_mode, ISIF_SYN_MODE);

	vpfe_dbg(3, dev, "end of isif_config_raw...\n");
	isif_readregs(isif);
}

static int isif_configure(struct vpfe_isif_device *isif)
{
	if (isif->isif_cfg.if_type == VPFE_RAW_BAYER)
		isif_config_raw(isif);
	else
		isif_config_ycbcr(isif);
	return 0;
}

static int isif_set_buftype(struct vpfe_isif_device *isif,
	enum isif_buftype buf_type)
{
	if (isif->isif_cfg.if_type == VPFE_RAW_BAYER)
		isif->isif_cfg.bayer.buf_type = buf_type;
	else
		isif->isif_cfg.ycbcr.buf_type = buf_type;
	return 0;
}

static enum isif_buftype isif_get_buftype(struct vpfe_isif_device *isif)
{
	if (isif->isif_cfg.if_type == VPFE_RAW_BAYER)
		return isif->isif_cfg.bayer.buf_type;
	return isif->isif_cfg.ycbcr.buf_type;
}

static int isif_set_pixel_format(struct vpfe_isif_device *isif, u32 pixfmt)
{
	struct vpfe_device *dev = container_of(isif,
					struct vpfe_device, vpfe_isif);

	vpfe_dbg(1, dev, "isif_set_pixel_format: if_type: %d, pixfmt:%s\n",
		isif->isif_cfg.if_type, print_fourcc(pixfmt));

	if (isif->isif_cfg.if_type == VPFE_RAW_BAYER) {
		isif->isif_cfg.bayer.pix_fmt = ISIF_PIXFMT_RAW;
		/*
		 * Need to clear it in case it was left on
		 * after the last capture.
		 */
		isif->isif_cfg.bayer.config_params.alaw.enable = 0;

		switch (pixfmt) {
		case V4L2_PIX_FMT_SBGGR8:
			isif->isif_cfg.bayer.config_params.alaw.enable = 1;
			break;
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_YUV420:
		case V4L2_PIX_FMT_NV12:
		case V4L2_PIX_FMT_RGB565X:
			break; /* nothing for now */
		case V4L2_PIX_FMT_SBGGR16:
		default:
			return -EINVAL;
		}
	} else {
		switch (pixfmt) {
		case V4L2_PIX_FMT_YUYV:
			isif->isif_cfg.ycbcr.pix_order = ISIF_PIXORDER_YCBYCR;
			break;
		case V4L2_PIX_FMT_UYVY:
			isif->isif_cfg.ycbcr.pix_order = ISIF_PIXORDER_CBYCRY;
			break;
		default:
			return -EINVAL;
		}
	}
	return 0;
}

static u32 isif_get_pixel_format(struct vpfe_isif_device *isif)
{
	u32 pixfmt;

	if (isif->isif_cfg.if_type == VPFE_RAW_BAYER) {
		pixfmt = V4L2_PIX_FMT_YUYV;
	} else {
		if (isif->isif_cfg.ycbcr.pix_order == ISIF_PIXORDER_YCBYCR)
			pixfmt = V4L2_PIX_FMT_YUYV;
		else
			pixfmt = V4L2_PIX_FMT_UYVY;
	}
	return pixfmt;
}

static int isif_set_image_window(struct vpfe_isif_device *isif,
	struct v4l2_rect *win, unsigned int bpp)
{
	if (isif->isif_cfg.if_type == VPFE_RAW_BAYER) {
		isif->isif_cfg.bayer.win = *win;
		isif->isif_cfg.bayer.bytesperpixel = bpp;
		isif->isif_cfg.bayer.bytesperline = ALIGN(win->width * bpp, 32);
	} else {
		isif->isif_cfg.ycbcr.win = *win;
		isif->isif_cfg.ycbcr.bytesperpixel = bpp;
		isif->isif_cfg.ycbcr.bytesperline = ALIGN(win->width * bpp, 32);
	}
	return 0;
}

static void isif_get_image_window(struct vpfe_isif_device *isif,
	struct v4l2_rect *win)
{
	if (isif->isif_cfg.if_type == VPFE_RAW_BAYER)
		*win = isif->isif_cfg.bayer.win;
	else
		*win = isif->isif_cfg.ycbcr.win;
}

static unsigned int isif_get_line_length(struct vpfe_isif_device *isif)
{
	if (isif->isif_cfg.if_type == VPFE_RAW_BAYER)
		return isif->isif_cfg.bayer.bytesperline;
	else
		return isif->isif_cfg.ycbcr.bytesperline;
}

static int isif_set_frame_format(struct vpfe_isif_device *isif,
	enum isif_frmfmt frm_fmt)
{
	if (isif->isif_cfg.if_type == VPFE_RAW_BAYER)
		isif->isif_cfg.bayer.frm_fmt = frm_fmt;
	else
		isif->isif_cfg.ycbcr.frm_fmt = frm_fmt;
	return 0;
}

static enum isif_frmfmt isif_get_frame_format(struct vpfe_isif_device *isif)
{
	if (isif->isif_cfg.if_type == VPFE_RAW_BAYER)
		return isif->isif_cfg.bayer.frm_fmt;
	else
		return isif->isif_cfg.ycbcr.frm_fmt;
}

static inline int isif_getfid(struct vpfe_isif_device *isif)
{
	return (isif_read(isif, ISIF_SYN_MODE) >> 15) & 1;
}

/* misc operations */
static inline void isif_setfbaddr(struct vpfe_isif_device *isif,
	unsigned long addr)
{
	isif_write(isif, addr & 0xffffffe0, ISIF_SDR_ADDR);
}

static int isif_set_hw_if_params(struct vpfe_isif_device *isif,
	struct vpfe_hw_if_param *params)
{
	struct vpfe_device *dev = container_of(isif,
					struct vpfe_device, vpfe_isif);

	isif->isif_cfg.if_type = params->if_type;

	switch (params->if_type) {
	case VPFE_BT656:
	case VPFE_YCBCR_SYNC_16:
	case VPFE_YCBCR_SYNC_8:
	case VPFE_BT656_10BIT:
		isif->isif_cfg.ycbcr.vd_pol = params->vdpol;
		isif->isif_cfg.ycbcr.hd_pol = params->hdpol;
		break;
	case VPFE_RAW_BAYER:
		isif->isif_cfg.bayer.vd_pol = params->vdpol;
		isif->isif_cfg.bayer.hd_pol = params->hdpol;
		if (params->bus_width == 10)
			isif->isif_cfg.bayer.config_params.data_sz =
				ISIF_DATA_10BITS;
		else
			isif->isif_cfg.bayer.config_params.data_sz =
				ISIF_DATA_8BITS;
		vpfe_dbg(1, dev, "params.bus_width: %d\n",
			params->bus_width);
		vpfe_dbg(1, dev, "config_params.data_sz: %d\n",
			isif->isif_cfg.bayer.config_params.data_sz);
		break;

	default:
		/* TODO add support for raw bayer here */
		return -EINVAL;
	}
	return 0;
}

static void isif_save_context(struct vpfe_isif_device *isif)
{
	isif->isif_ctx[ISIF_PCR >> 2] = isif_read(isif, ISIF_PCR);
	isif->isif_ctx[ISIF_SYN_MODE >> 2] = isif_read(isif, ISIF_SYN_MODE);
	isif->isif_ctx[ISIF_HD_VD_WID >> 2] = isif_read(isif, ISIF_HD_VD_WID);
	isif->isif_ctx[ISIF_PIX_LINES >> 2] = isif_read(isif, ISIF_PIX_LINES);
	isif->isif_ctx[ISIF_HORZ_INFO >> 2] = isif_read(isif, ISIF_HORZ_INFO);
	isif->isif_ctx[ISIF_VERT_START >> 2] = isif_read(isif, ISIF_VERT_START);
	isif->isif_ctx[ISIF_VERT_LINES >> 2] = isif_read(isif, ISIF_VERT_LINES);
	isif->isif_ctx[ISIF_CULLING >> 2] = isif_read(isif, ISIF_CULLING);
	isif->isif_ctx[ISIF_HSIZE_OFF >> 2] = isif_read(isif, ISIF_HSIZE_OFF);
	isif->isif_ctx[ISIF_SDOFST >> 2] = isif_read(isif, ISIF_SDOFST);
	isif->isif_ctx[ISIF_SDR_ADDR >> 2] = isif_read(isif, ISIF_SDR_ADDR);
	isif->isif_ctx[ISIF_CLAMP >> 2] = isif_read(isif, ISIF_CLAMP);
	isif->isif_ctx[ISIF_DCSUB >> 2] = isif_read(isif, ISIF_DCSUB);
	isif->isif_ctx[ISIF_COLPTN >> 2] = isif_read(isif, ISIF_COLPTN);
	isif->isif_ctx[ISIF_BLKCMP >> 2] = isif_read(isif, ISIF_BLKCMP);
	isif->isif_ctx[ISIF_VDINT >> 2] = isif_read(isif, ISIF_VDINT);
	isif->isif_ctx[ISIF_ALAW >> 2] = isif_read(isif, ISIF_ALAW);
	isif->isif_ctx[ISIF_REC656IF >> 2] = isif_read(isif, ISIF_REC656IF);
	isif->isif_ctx[ISIF_CCDCFG >> 2] = isif_read(isif, ISIF_CCDCFG);
}

static void isif_restore_context(struct vpfe_isif_device *isif)
{
	isif_write(isif, isif->isif_ctx[ISIF_SYN_MODE >> 2], ISIF_SYN_MODE);
	isif_write(isif, isif->isif_ctx[ISIF_HD_VD_WID >> 2], ISIF_HD_VD_WID);
	isif_write(isif, isif->isif_ctx[ISIF_PIX_LINES >> 2], ISIF_PIX_LINES);
	isif_write(isif, isif->isif_ctx[ISIF_HORZ_INFO >> 2], ISIF_HORZ_INFO);
	isif_write(isif, isif->isif_ctx[ISIF_VERT_START >> 2], ISIF_VERT_START);
	isif_write(isif, isif->isif_ctx[ISIF_VERT_LINES >> 2], ISIF_VERT_LINES);
	isif_write(isif, isif->isif_ctx[ISIF_CULLING >> 2], ISIF_CULLING);
	isif_write(isif, isif->isif_ctx[ISIF_HSIZE_OFF >> 2], ISIF_HSIZE_OFF);
	isif_write(isif, isif->isif_ctx[ISIF_SDOFST >> 2], ISIF_SDOFST);
	isif_write(isif, isif->isif_ctx[ISIF_SDR_ADDR >> 2], ISIF_SDR_ADDR);
	isif_write(isif, isif->isif_ctx[ISIF_CLAMP >> 2], ISIF_CLAMP);
	isif_write(isif, isif->isif_ctx[ISIF_DCSUB >> 2], ISIF_DCSUB);
	isif_write(isif, isif->isif_ctx[ISIF_COLPTN >> 2], ISIF_COLPTN);
	isif_write(isif, isif->isif_ctx[ISIF_BLKCMP >> 2], ISIF_BLKCMP);
	isif_write(isif, isif->isif_ctx[ISIF_VDINT >> 2], ISIF_VDINT);
	isif_write(isif, isif->isif_ctx[ISIF_ALAW >> 2], ISIF_ALAW);
	isif_write(isif, isif->isif_ctx[ISIF_REC656IF >> 2], ISIF_REC656IF);
	isif_write(isif, isif->isif_ctx[ISIF_CCDCFG >> 2], ISIF_CCDCFG);
	isif_write(isif, isif->isif_ctx[ISIF_PCR >> 2], ISIF_PCR);
}

static void isif_clear_intr(struct vpfe_isif_device *isif, int vdint)
{
	unsigned int vpfe_int_status;

	vpfe_int_status = isif_read(isif, ISIF_VPFE_IRQ_STATUS);

	switch (vdint) {
	/* VD0 interrrupt */
	case ISIF_VPFE_VDINT0:
		vpfe_int_status &= ~ISIF_VPFE_VDINT0;
		vpfe_int_status |= ISIF_VPFE_VDINT0;
		break;
	/* VD1 interrrupt */
	case ISIF_VPFE_VDINT1:
		vpfe_int_status &= ~ISIF_VPFE_VDINT1;
		vpfe_int_status |= ISIF_VPFE_VDINT1;
		break;
	/* VD2 interrrupt */
	case ISIF_VPFE_VDINT2:
		vpfe_int_status &= ~ISIF_VPFE_VDINT2;
		vpfe_int_status |= ISIF_VPFE_VDINT2;
		break;
	/* Clear all interrrupts */
	default:
		vpfe_int_status &= ~(ISIF_VPFE_VDINT0 |
				ISIF_VPFE_VDINT1 |
				ISIF_VPFE_VDINT2);
		vpfe_int_status |= (ISIF_VPFE_VDINT0 |
				ISIF_VPFE_VDINT1 |
				ISIF_VPFE_VDINT2);
		break;
	}
	/* Clear specific VDINT from the status register */
	isif_write(isif, vpfe_int_status, ISIF_VPFE_IRQ_STATUS);

	vpfe_int_status = isif_read(isif, ISIF_VPFE_IRQ_STATUS);

	/* Acknowledge that we are done with all interrupts */
	isif_write(isif, 1, ISIF_VPFE_IRQ_EOI);
}

static inline int isif_intr_status(struct vpfe_isif_device *isif)
{
	return isif_read(isif, ISIF_VPFE_IRQ_STATUS);
}

static int isif_intr_enable(struct vpfe_isif_device *isif, int vdint)
{
	unsigned int intr;
	struct vpfe_device *dev = container_of(isif,
					struct vpfe_device, vpfe_isif);
	vpfe_dbg(3, dev, "isif_intr_enable(%d) ", vdint);
	isif_write(isif, vdint, ISIF_VPFE_IRQ_ENABLE_SET);
	intr = isif_read(isif, ISIF_VPFE_IRQ_ENABLE_SET);
	vpfe_dbg(3, dev, "irq_enable_set: %x\n", intr);
	return 0;
}

static int isif_intr_disable(struct vpfe_isif_device *isif, int vdint)
{
	unsigned int intr;
	struct vpfe_device *dev = container_of(isif,
					struct vpfe_device, vpfe_isif);
	vpfe_dbg(3, dev, "isif_intr_disable(%d) ", vdint);
	isif_write(isif, vdint, ISIF_VPFE_IRQ_ENABLE_CLR);
	intr = isif_read(isif, ISIF_VPFE_IRQ_ENABLE_CLR);
	vpfe_dbg(3, dev, "irq_enable_CLR: %x\n", intr);
	return 0;
}

static int isif_remove(struct vpfe_isif_device *isif,
		       struct platform_device *pdev)
{
	struct resource	*res;
	struct vpfe_device *dev = container_of(isif,
					struct vpfe_device, vpfe_isif);

	vpfe_dbg(2, dev, "isif_remove\n");

	pm_runtime_disable(&pdev->dev);
	iounmap(isif->isif_cfg.base_addr);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		release_mem_region(res->start, resource_size(res));
	return 0;
}

static int isif_suspend(struct vpfe_isif_device *isif, struct device *dev)
{
	struct vpfe_device *vpfe_dev = container_of(isif,
					struct vpfe_device, vpfe_isif);
	vpfe_dbg(2, vpfe_dev, "isif_suspend\n");

	pm_runtime_get_sync(dev);
	isif_module_enable(isif, 1);

	/* Save ISIF context */
	isif_save_context(isif);

	/* Disable ISIF */
	isif_enable(isif, 0);
	isif_module_enable(isif, 0);
	/* Disable both master and slave clock */
	pm_runtime_put_sync(dev);

	return 0;
}

static int isif_resume(struct vpfe_isif_device *isif, struct device *dev)
{
	struct vpfe_device *vpfe_dev = container_of(isif,
					struct vpfe_device, vpfe_isif);
	vpfe_dbg(2, vpfe_dev, "isif_resume\n");

	/* Enable both master and slave clock */
	pm_runtime_get_sync(dev);
	isif_module_enable(isif, 1);
	/* Restore ISIF context */
	isif_restore_context(isif);

	isif_module_enable(isif, 0);
	pm_runtime_put_sync(dev);

	return 0;
}

static void isif_config_defaults(struct vpfe_isif_device *isif)
{
	isif->isif_cfg.if_type = VPFE_RAW_BAYER;

	isif->isif_cfg.ycbcr.pix_fmt = ISIF_PIXFMT_YCBCR_8BIT;
	isif->isif_cfg.ycbcr.frm_fmt = ISIF_FRMFMT_INTERLACED;
	isif->isif_cfg.ycbcr.fid_pol = VPFE_PINPOL_POSITIVE;
	isif->isif_cfg.ycbcr.vd_pol = VPFE_PINPOL_POSITIVE;
	isif->isif_cfg.ycbcr.hd_pol = VPFE_PINPOL_POSITIVE;
	isif->isif_cfg.ycbcr.pix_order = ISIF_PIXORDER_CBYCRY;
	isif->isif_cfg.ycbcr.buf_type = ISIF_BUFTYPE_FLD_INTERLEAVED;
	/* ISIF_WIN_PAL */
	isif->isif_cfg.ycbcr.win.left = 0;
	isif->isif_cfg.ycbcr.win.top = 0;
	isif->isif_cfg.ycbcr.win.width = 720;
	isif->isif_cfg.ycbcr.win.height = 576;
	isif->isif_cfg.ycbcr.bt656_enable = 1;

	isif->isif_cfg.bayer.pix_fmt = ISIF_PIXFMT_RAW;
	isif->isif_cfg.bayer.frm_fmt = ISIF_FRMFMT_PROGRESSIVE;
	isif->isif_cfg.bayer.fid_pol = VPFE_PINPOL_POSITIVE;
	isif->isif_cfg.bayer.vd_pol = VPFE_PINPOL_POSITIVE;
	isif->isif_cfg.bayer.hd_pol = VPFE_PINPOL_POSITIVE;
	/* ISIF_WIN_SVGA */
	isif->isif_cfg.bayer.win.left = 0;
	isif->isif_cfg.bayer.win.top = 0;
	isif->isif_cfg.bayer.win.width = 800;
	isif->isif_cfg.bayer.win.height = 600;
	isif->isif_cfg.bayer.config_params.data_sz = ISIF_DATA_8BITS;
	isif->isif_cfg.bayer.config_params.alaw.gamma_wd = ISIF_GAMMA_BITS_09_0;
}

/*
 * vpfe_isif_init() - Initialize V4L2 subdev and media entity
 * @isif: VPFE isif module
 * @pdev: Pointer to platform device structure.
 * Return 0 on success and a negative error code on failure.
 */
static int vpfe_isif_init(struct vpfe_isif_device *isif,
			  struct platform_device *pdev)
{
	struct resource	*res;
	int status = 0;
	struct vpfe_device *dev = container_of(isif,
					struct vpfe_device, vpfe_isif);

	vpfe_dbg(2, dev, "vpfe_isif_init\n");

	if (!isif) {
		vpfe_err(dev, "Failed to init isif device: isif is null\n");
		return -EFAULT;
	}

	strncpy(isif->name, "AM437x ISIF", sizeof(isif->name));
	isif->owner = THIS_MODULE;

	vpfe_dbg(1, dev, "vpfe_isif_init: %s\n", isif->name);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		status = -ENODEV;
		goto fail_nores;
	}

	res = request_mem_region(res->start, resource_size(res), res->name);
	if (!res) {
		status = -EBUSY;
		goto fail_nores;
	}

	isif->isif_cfg.base_addr =
			ioremap_nocache(res->start, resource_size(res));
	if (!isif->isif_cfg.base_addr) {
		status = -ENOMEM;
		goto fail_nomem;
	}

	/* not sure we still need this but keeping it for
	 * 1st pass multi-instance mods
	 */
	isif->isif_cfg.dev = &pdev->dev;

	/* Enabling module functional clock */
	pm_runtime_enable(&pdev->dev);

	/* for now just enable it here instead of waiting for the open */
	pm_runtime_get_sync(&pdev->dev);

	isif_config_defaults(isif);

	vpfe_dbg(1, dev, "%s is registered with vpfe.\n", isif->name);

	pm_runtime_put_sync(&pdev->dev);

	return 0;

fail_nomem:
	release_mem_region(res->start, resource_size(res));
fail_nores:
	isif_remove(isif, pdev);
	return status;
}

/*
 * vpfe_get_isif_image_format - Get image parameters based on ISIF settings
 */
static int vpfe_get_isif_image_format(struct vpfe_device *dev,
				 struct v4l2_format *f)
{
	struct v4l2_rect image_win;
	enum isif_buftype buf_type;
	enum isif_frmfmt frm_fmt;

	memset(f, 0, sizeof(*f));
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	isif_get_image_window(&dev->vpfe_isif, &image_win);
	f->fmt.pix.width = image_win.width;
	f->fmt.pix.height = image_win.height;
	f->fmt.pix.bytesperline = isif_get_line_length(&dev->vpfe_isif);
	f->fmt.pix.sizeimage = f->fmt.pix.bytesperline *
				f->fmt.pix.height;
	buf_type = isif_get_buftype(&dev->vpfe_isif);
	f->fmt.pix.pixelformat = isif_get_pixel_format(&dev->vpfe_isif);
	frm_fmt = isif_get_frame_format(&dev->vpfe_isif);
	if (frm_fmt == ISIF_FRMFMT_PROGRESSIVE) {
		f->fmt.pix.field = V4L2_FIELD_NONE;
	} else if (frm_fmt == ISIF_FRMFMT_INTERLACED) {
		if (buf_type == ISIF_BUFTYPE_FLD_INTERLEAVED) {
			f->fmt.pix.field = V4L2_FIELD_INTERLACED;
		 } else if (buf_type == ISIF_BUFTYPE_FLD_SEPARATED) {
			f->fmt.pix.field = V4L2_FIELD_SEQ_TB;
		} else {
			vpfe_err(dev, "Invalid buf_type\n");
			return -EINVAL;
		}
	} else {
		vpfe_err(dev, "Invalid frm_fmt\n");
		return -EINVAL;
	}
	return 0;
}

/*
 * vpfe_config_isif_image_format()
 * For a pix format, configure isif to setup the capture
 */
static int vpfe_config_isif_image_format(struct vpfe_device *dev)
{
	enum isif_frmfmt frm_fmt = ISIF_FRMFMT_INTERLACED;
	int ret = 0;

	vpfe_dbg(2, dev, "vpfe_config_isif_image_format\n");

	vpfe_dbg(1, dev, "pixelformat: %s\n",
		print_fourcc(dev->fmt.fmt.pix.pixelformat));

	if (isif_set_pixel_format(
			&dev->vpfe_isif,
			dev->fmt.fmt.pix.pixelformat) < 0) {
		vpfe_err(dev,
			"couldn't set pix format in isif\n");
		return -EINVAL;
	}
	/* configure the image window */
	isif_set_image_window(&dev->vpfe_isif,	&dev->crop, dev->bpp);

	switch (dev->fmt.fmt.pix.field) {
	case V4L2_FIELD_INTERLACED:
		/* do nothing, since it is default */
		ret = isif_set_buftype(
				&dev->vpfe_isif,
				ISIF_BUFTYPE_FLD_INTERLEAVED);
		break;
	case V4L2_FIELD_NONE:
		frm_fmt = ISIF_FRMFMT_PROGRESSIVE;
		/* buffer type only applicable for interlaced scan */
		break;
	case V4L2_FIELD_SEQ_TB:
		ret = isif_set_buftype(
				&dev->vpfe_isif,
				ISIF_BUFTYPE_FLD_SEPARATED);
		break;
	default:
		return -EINVAL;
	}

	/* set the frame format */
	if (!ret)
		ret = isif_set_frame_format(&dev->vpfe_isif, frm_fmt);
	return ret;
}
/*
 * vpfe_config_image_format()
 * For a given standard, this functions sets up the default
 * pix format & crop values in the vpfe device and isif.  It first
 * starts with defaults based values from the standard table.
 * It then checks if sub device support g_mbus_fmt and then override the
 * values based on that.Sets crop values to match with scan resolution
 * starting at 0,0. It calls vpfe_config_isif_image_format() set the
 * values in isif
 */
static int vpfe_config_image_format(struct vpfe_device *dev,
				    v4l2_std_id std_id)
{
	struct vpfe_subdev_info *sdinfo = dev->current_subdev;
	struct v4l2_mbus_framefmt mbus_fmt;
	struct v4l2_pix_format *pix = &dev->fmt.fmt.pix;
	struct v4l2_pix_format pix_test;
	struct v4l2_subdev_format sd_fmt;
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(vpfe_standards); i++) {
		if (vpfe_standards[i].std_id & std_id) {
			dev->std_info.active_pixels =
					vpfe_standards[i].width;
			dev->std_info.active_lines =
					vpfe_standards[i].height;
			dev->std_info.frame_format =
					vpfe_standards[i].frame_format;
			dev->std_index = i;
			break;
		}
	}

	if (i ==  ARRAY_SIZE(vpfe_standards)) {
		vpfe_err(dev, "standard not supported\n");
		return -EINVAL;
	}

	dev->crop.top = 0;
	dev->crop.left = 0;
	dev->crop.width = dev->std_info.active_pixels;
	dev->crop.height = dev->std_info.active_lines;
	pix->width = dev->crop.width;
	pix->height = dev->crop.height;

	/* first field and frame format based on standard frame format */
	if (dev->std_info.frame_format) {
		pix->field = V4L2_FIELD_INTERLACED;
		/* assume V4L2_PIX_FMT_UYVY as default */
		pix->pixelformat = V4L2_PIX_FMT_YUYV;
		v4l2_fill_mbus_format(&mbus_fmt, pix,
				      V4L2_MBUS_FMT_YUYV10_2X10);
	} else {
		pix->field = V4L2_FIELD_NONE;
		/* assume V4L2_PIX_FMT_SBGGR8 */
		pix->pixelformat = V4L2_PIX_FMT_YUYV;
		v4l2_fill_mbus_format(&mbus_fmt, pix,
				      V4L2_MBUS_FMT_YUYV10_2X10);
	}

	/* copy pix format for testing */
	pix_test = *pix;

	if (!v4l2_device_has_op(&dev->v4l2_dev, video, g_mbus_fmt)) {
		vpfe_dbg(4, dev,
			"v4l2_device_has_op: sub device '%s' does not support g_mbus_fmt\n",
			sdinfo->name);

		sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;

		ret = v4l2_device_call_until_err(&dev->v4l2_dev,
				sdinfo->grp_id, pad, get_fmt, NULL, &sd_fmt);

		if (ret && ret != -ENOIOCTLCMD) {
			vpfe_err(dev,
				"error in getting g_mbus_fmt from sub device\n");
			return ret;
		}
	} else {
		/* if sub device supports g_mbus_fmt, override the defaults */
		ret = v4l2_device_call_until_err(&dev->v4l2_dev,
				sdinfo->grp_id, video, g_mbus_fmt, &mbus_fmt);

		if (ret && ret != -ENOIOCTLCMD) {
			vpfe_err(dev,
				"error in getting g_mbus_fmt from sub device\n");
			return ret;
		} else if (ret == -ENOIOCTLCMD) {
			vpfe_dbg(3, dev,
				"sub device '%s' does not support g_mbus_fmt\n",
				sdinfo->name);
		} else {
			vpfe_dbg(3, dev,
				"v4l2_subdev_call g_mbus_fmt: %d\n", ret);
		}
	}

	/* convert mbus_format to v4l2_format */
	v4l2_fill_pix_format(pix, &sd_fmt.format);
	mbus_to_pix(dev, &sd_fmt.format, pix, &dev->bpp);

	vpfe_dbg(1, dev, "vpfe_config_image_format pix 1: size %dx%d pixelformat: %s bytesperline = %d, sizeimage = %d\n",
		pix->width, pix->height,
		print_fourcc(pix->pixelformat),
		pix->bytesperline, pix->sizeimage);

	/* Update the crop window based on found values */
	dev->crop.width = pix->width;
	dev->crop.height = pix->height;

	/* Sets the values in ISIF */
	ret = vpfe_config_isif_image_format(dev);

	return ret;
}

static int vpfe_initialize_device(struct vpfe_device *dev)
{
	int ret = 0;

	/* set first input of current subdevice as the current input */
	dev->current_input = 0;

	/* set default standard */
	dev->std_index = 0;

	/* Configure the default format information */
	ret = vpfe_config_image_format(dev,
				vpfe_standards[dev->std_index].std_id);
	if (ret)
		return ret;

	/* now open the isif device to initialize it */
	ret = isif_open(&dev->vpfe_isif, dev->pdev);

	/* Clear all VPFE/ISIF interrupts */
	isif_clear_intr(&dev->vpfe_isif, -1);

	return ret;
}

/*
 * vpfe_open : This function is based on the v4l2_fh_open helper function.
 * It has been augmented to handle module power management,
 * by disabling/enabling h/w module fcntl clock when necessary.
 */

static int vpfe_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct vpfe_device *dev = video_drvdata(file);
	struct v4l2_fh *fh = kzalloc(sizeof(*fh), GFP_KERNEL);

	vpfe_dbg(2, dev, "vpfe_open\n");

	file->private_data = fh;
	if (fh == NULL)
		return -ENOMEM;
	v4l2_fh_init(fh, vdev);
	v4l2_fh_add(fh);

	/*
	 * If this is the first open file.
	 * Then initialize hw module.
	 */
	if (v4l2_fh_is_singular_file(file)) {
		mutex_lock(&dev->lock);
		if (vpfe_initialize_device(dev)) {
			mutex_unlock(&dev->lock);
			return -ENODEV;
		}
		mutex_unlock(&dev->lock);
	}

	return 0;
}

/**
 * vpfe_schedule_next_buffer: set next buffer address for capture
 * @dev : ptr to device
 *
 * This function will get next buffer from the dma queue and
 * set the buffer address in the vpfe register for capture.
 * the buffer is marked active
 *
 * Assumes caller is holding dev->dma_queue_lock already
 */
static void vpfe_schedule_next_buffer(struct vpfe_device *dev)
{
	unsigned long addr;

	dev->next_frm = list_entry(dev->dma_queue.next,
					struct vpfe_cap_buffer, list);
	list_del(&dev->next_frm->list);

	dev->next_frm->vb.state = VB2_BUF_STATE_ACTIVE;
	addr = vb2_dma_contig_plane_dma_addr(&dev->next_frm->vb, 0);

	isif_setfbaddr(&dev->vpfe_isif, addr);
}

static void vpfe_schedule_bottom_field(struct vpfe_device *dev)
{
	unsigned long addr;

	addr = vb2_dma_contig_plane_dma_addr(&dev->next_frm->vb, 0);
	addr += dev->field_off;
	isif_setfbaddr(&dev->vpfe_isif, addr);
}

/*
 * vpfe_process_buffer_complete: process a completed buffer
 * @dev : ptr to device
 *
 * This function time stamp the buffer and mark it as DONE. It also
 * wake up any process waiting on the QUEUE and set the next buffer
 * as current
 */
static void vpfe_process_buffer_complete(struct vpfe_device *dev)
{
	v4l2_get_timestamp(&dev->cur_frm->vb.v4l2_buf.timestamp);

	vb2_buffer_done(&dev->cur_frm->vb, VB2_BUF_STATE_DONE);
	dev->cur_frm = dev->next_frm;
}

/*
 * vpfe_isr : ISR handler for vpfe capture (VINT0)
 * @irq: irq number
 * @dev_id: dev_id ptr
 *
 * It changes status of the captured buffer, takes next buffer from the queue
 * and sets its address in VPFE registers
 */
static irqreturn_t vpfe_isr(int irq, void *dev_id)
{
	struct vpfe_device *dev = dev_id;
	struct vb2_queue *q = &dev->buffer_queue;
	int vpfe_intr_status;
	enum v4l2_field field;
	int fid;

	/* Which interrupt did we get */
	vpfe_intr_status = isif_intr_status(&dev->vpfe_isif);

	/* if streaming not started, don't do anything */
	if (!vb2_is_streaming(q))
		goto clear_intr;

	if (vpfe_intr_status & ISIF_VPFE_VDINT0) {
		field = dev->fmt.fmt.pix.field;

		if (field == V4L2_FIELD_NONE) {
			/* handle progressive frame capture */
			if (dev->cur_frm != dev->next_frm)
				vpfe_process_buffer_complete(dev);
			goto next_intr;
		}

		/* interlaced or TB capture check which field
		   we are in hardware */
		fid = isif_getfid(&dev->vpfe_isif);

		/* switch the software maintained field id */
		dev->field_id ^= 1;
		if (fid == dev->field_id) {
			/* we are in-sync here,continue */
			if (fid == 0) {
				/*
				 * One frame is just being captured. If the
				 * next frame is available, release the
				 * current frame and move on
				 */
				if (dev->cur_frm != dev->next_frm)
					vpfe_process_buffer_complete(dev);
				/*
				 * based on whether the two fields are stored
				 * interleavely or separately in memory,
				 * reconfigure the ISIF memory address
				 */
				if (field == V4L2_FIELD_SEQ_TB)
					vpfe_schedule_bottom_field(dev);

				goto next_intr;
			}
			/*
			 * if one field is just being captured configure
			 * the next frame get the next frame from the empty
			 * queue if no frame is available hold on to the
			 * current buffer
			 */
			spin_lock(&dev->dma_queue_lock);
			if (!list_empty(&dev->dma_queue) &&
			    dev->cur_frm == dev->next_frm)
				vpfe_schedule_next_buffer(dev);
			spin_unlock(&dev->dma_queue_lock);
		} else if (fid == 0) {
			/*
			 * out of sync. Recover from any hardware out-of-sync.
			 * May loose one frame
			 */
			dev->field_id = fid;
		}
	}

next_intr:
	if (vpfe_intr_status & ISIF_VPFE_VDINT1) {
		spin_lock(&dev->dma_queue_lock);
		if ((dev->fmt.fmt.pix.field == V4L2_FIELD_NONE) &&
		    !list_empty(&dev->dma_queue) &&
		    dev->cur_frm == dev->next_frm)
			vpfe_schedule_next_buffer(dev);
		spin_unlock(&dev->dma_queue_lock);
	}

clear_intr:
	isif_clear_intr(&dev->vpfe_isif, vpfe_intr_status);

	return IRQ_HANDLED;
}

static void vpfe_detach_irq(struct vpfe_device *dev)
{
	enum isif_frmfmt frame_format;
	unsigned int intr = ISIF_VPFE_VDINT0;

	frame_format = isif_get_frame_format(&dev->vpfe_isif);
	if (frame_format == ISIF_FRMFMT_PROGRESSIVE)
		intr |= ISIF_VPFE_VDINT1;

	isif_intr_disable(&dev->vpfe_isif, intr);
}

static int vpfe_attach_irq(struct vpfe_device *dev)
{
	enum isif_frmfmt frame_format;
	unsigned int intr = ISIF_VPFE_VDINT0;

	frame_format = isif_get_frame_format(&dev->vpfe_isif);
	if (frame_format == ISIF_FRMFMT_PROGRESSIVE)
		intr |= ISIF_VPFE_VDINT1;

	isif_intr_enable(&dev->vpfe_isif, intr);
	return 0;
}

/* vpfe_stop_isif_capture: stop streaming in ccdc/isif */
static void vpfe_stop_isif_capture(struct vpfe_device *dev)
{
	isif_enable(&dev->vpfe_isif, 0);
}

/*
 * vpfe_release : This function is based on the vb2_fop_release
 * helper function.
 * It has been augmented to handle module power management,
 * by disabling/enabling h/w module fcntl clock when necessary.
 */
static int vpfe_release(struct file *file)
{
	struct vpfe_device *dev = video_drvdata(file);
	bool fh_singular = v4l2_fh_is_singular_file(file);
	int ret;

	vpfe_dbg(2, dev, "vpfe_release\n");

	/* the release helper will cleanup any on-going streaming */
	ret = vb2_fop_release(file);

	/*
	 * If this was the last open file.
	 * Then de-initialize hw module.
	 */
	if (fh_singular) {
		mutex_lock(&dev->lock);
		isif_close(&dev->vpfe_isif, dev->pdev);
		mutex_unlock(&dev->lock);
	}

	return ret;
}

static int vpfe_querycap(struct file *file, void  *priv,
			       struct v4l2_capability *cap)
{
	struct vpfe_device *dev = video_drvdata(file);

	vpfe_dbg(2, dev, "vpfe_querycap\n");

	snprintf(cap->bus_info, sizeof(cap->bus_info),
			"platform:%s", dev->v4l2_dev.name);
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_READWRITE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	strlcpy(cap->driver, VPFE_MODULE_NAME, sizeof(cap->driver));
	strlcpy(cap->card, dev->cfg->card_name, sizeof(cap->card));
	return 0;
}

/* get the subdev which is connected to the output video node */
static struct v4l2_subdev *
vpfe_remote_subdev(struct vpfe_device *dev, u32 *pad)
{
	struct media_pad *remote = media_entity_remote_pad(&dev->pad);

	vpfe_dbg(2, dev, "vpfe_remote_subdev: remote:%x\n",
		(unsigned int)remote);

	if (remote == NULL) {
		return NULL;
	} else if ((remote->entity->type & MEDIA_ENT_T_V4L2_SUBDEV) == 0) {
		vpfe_dbg(1, dev,
			"vpfe_remote_subdev: remote->entity->type:%d was expecting %d\n",
			remote->entity->type, MEDIA_ENT_T_V4L2_SUBDEV);
		return NULL;
	}

	if (pad)
		*pad = remote->index;

	return media_entity_to_v4l2_subdev(remote->entity);
}

/* get the format set at output pad of the adjacent subdev */
static int __vpfe_get_format(struct vpfe_device *dev,
			struct v4l2_format *format, unsigned int *bpp)
{
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev *subdev;
	struct media_pad *remote;
	u32 pad;
	int ret;

	vpfe_dbg(2, dev, "__vpfe_get_format\n");

	subdev = vpfe_remote_subdev(dev, &pad);
	if (subdev == NULL)
		return -EINVAL;

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	remote = media_entity_remote_pad(&dev->pad);
	fmt.pad = remote->index;

	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt);
	if (ret == -ENOIOCTLCMD)
		return -EINVAL;

	format->type = dev->fmt.type;

	/* convert mbus_format to v4l2_format */
	v4l2_fill_pix_format(&format->fmt.pix, &fmt.format);
	mbus_to_pix(dev, &fmt.format, &format->fmt.pix, bpp);
	vpfe_dbg(1, dev, "__vpfe_get_format size %dx%d (%s) bytesperline = %d, sizeimage = %d, bpp = %d\n",
		format->fmt.pix.width, format->fmt.pix.height,
		print_fourcc(format->fmt.pix.pixelformat),
		format->fmt.pix.bytesperline, format->fmt.pix.sizeimage, *bpp);

	return 0;
}

/* set the format at output pad of the adjacent subdev */
static int __vpfe_set_format(struct vpfe_device *dev,
			struct v4l2_format *format, unsigned int *bpp)
{
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev *subdev;
	struct media_pad *remote;
	u32 pad;
	int ret;

	vpfe_dbg(2, dev, "__vpfe_set_format\n");

	subdev = vpfe_remote_subdev(dev, &pad);
	if (subdev == NULL)
		return -EINVAL;

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	remote = media_entity_remote_pad(&dev->pad);
	fmt.pad = remote->index;

	ret = pix_to_mbus(dev, &format->fmt.pix, &fmt.format);
	if (ret)
		return ret;

	ret = v4l2_subdev_call(subdev, pad, set_fmt, NULL, &fmt);
	if (ret == -ENOIOCTLCMD)
		return -EINVAL;

	format->type = dev->fmt.type;

	/* convert mbus_format to v4l2_format */
	v4l2_fill_pix_format(&format->fmt.pix, &fmt.format);
	mbus_to_pix(dev, &fmt.format, &format->fmt.pix, bpp);
	vpfe_dbg(1, dev, "__vpfe_set_format size %dx%d (%s) bytesperline = %d, sizeimage = %d, bpp = %d\n",
		format->fmt.pix.width, format->fmt.pix.height,
		print_fourcc(format->fmt.pix.pixelformat),
		format->fmt.pix.bytesperline, format->fmt.pix.sizeimage, *bpp);

	return 0;
}

static int vpfe_g_fmt(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct vpfe_device *dev = video_drvdata(file);
	struct v4l2_format format;
	unsigned int bpp;
	int ret = 0;

	vpfe_dbg(2, dev, "vpfe_g_fmt\n");

	ret = __vpfe_get_format(dev, &format, &bpp);
	if (ret)
		return ret;

	/* Fill in the information about format */
	*fmt = dev->fmt;

	return ret;
}

static int vpfe_enum_fmt(struct file *file, void  *priv,
			 struct v4l2_fmtdesc *f)
{
	struct vpfe_device *dev = video_drvdata(file);
	struct v4l2_subdev_mbus_code_enum mbus_code;
	struct v4l2_subdev *subdev;
	struct media_pad *remote;
	struct vpfe_fmt *fmt;
	u32 pad;
	int ret;

	vpfe_dbg(2, dev, "vpfe_enum_format index:%d\n",
		f->index);

	subdev = vpfe_remote_subdev(dev, &pad);
	if (subdev == NULL)
		return -EINVAL;

	remote = media_entity_remote_pad(&dev->pad);
	mbus_code.index = f->index;
	ret = v4l2_subdev_call(subdev, pad, enum_mbus_code, NULL, &mbus_code);
	if (ret)
		return -EINVAL;

	/* convert mbus_format to v4l2_format */
	fmt = find_format_by_code(mbus_code.code);
	if (fmt == NULL) {
		vpfe_err(dev, "mbus code 0x%08x not found\n",
			mbus_code.code);
		return -EINVAL;
	}

	strncpy(f->description, fmt->name, sizeof(f->description) - 1);
	f->pixelformat = fmt->fourcc;
	f->type = dev->fmt.type;

	vpfe_dbg(1, dev, "vpfe_enum_format: mbus index: %d code: %x pixelformat: %s [%s]\n",
		f->index, fmt->code, print_fourcc(fmt->fourcc), fmt->name);

	return 0;
}

static int vpfe_s_fmt(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct vpfe_device *dev = video_drvdata(file);
	struct vb2_queue *q = &dev->buffer_queue;
	struct v4l2_format format;
	unsigned int bpp;
	int ret = 0;

	vpfe_dbg(2, dev, "vpfe_s_fmt\n");

	/* Check for valid frame format */
	ret = __vpfe_get_format(dev, &format, &bpp);
	if (ret)
		return ret;

	/* If streaming is started, return error */
	if (vb2_is_busy(q)) {
		vpfe_err(dev, "%s device busy\n", __func__);
		return -EBUSY;
	}

	if (!cmp_v4l2_format(fmt, &format)) {
		/* Sensor format is different from the requested format
		 * so we need to change it
		 */
		ret = __vpfe_set_format(dev, fmt, &bpp);
		if (ret)
			return ret;
	} else /* Just make sure all of the fields are consistent */
		*fmt = format;

	fmt->fmt.pix.priv = 0;

	/* First detach any IRQ if currently attached */
	vpfe_detach_irq(dev);
	dev->fmt = *fmt;
	dev->bpp = bpp;

	/* Update the crop window based on found values */
	dev->crop.width = fmt->fmt.pix.width;
	dev->crop.height = fmt->fmt.pix.height;

	/* set image capture parameters in the isif */
	ret = vpfe_config_isif_image_format(dev);
	return ret;
}

static int vpfe_try_fmt(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct vpfe_device *dev = video_drvdata(file);
	struct v4l2_format format;
	unsigned int bpp;
	int ret = 0;

	vpfe_dbg(2, dev, "vpfe_try_fmt\n");

	memset(&format, 0x00, sizeof(format));
	ret = __vpfe_get_format(dev, &format, &bpp);
	if (ret)
		return ret;

	*fmt = format;
	fmt->fmt.pix.priv = 0;
	return 0;
}

static int vpfe_enum_size(struct file *file, void  *priv,
				   struct v4l2_frmsizeenum *fsize)
{
	struct vpfe_device *dev = video_drvdata(file);
	struct v4l2_mbus_framefmt mbus;
	struct v4l2_subdev_frame_size_enum fse;
	struct v4l2_subdev *subdev;
	struct media_pad *remote;
	struct v4l2_pix_format pix;
	struct vpfe_fmt *fmt;
	u32 pad;
	int ret;

	vpfe_dbg(2, dev, "vpfe_enum_size\n");

	/* check for valid format */
	fmt = find_format_by_pix(fsize->pixel_format);
	if (fmt == NULL) {
		vpfe_dbg(3, dev, "Invalid pixel code: %x, default used instead\n",
			fsize->pixel_format);
		return -EINVAL;
	}

	memset(fsize->reserved, 0x00, sizeof(fsize->reserved));
	subdev = vpfe_remote_subdev(dev, &pad);
	if (subdev == NULL)
		return -EINVAL;

	/* Construct pix from parameter and use defualt for the rest */
	pix.pixelformat = fsize->pixel_format;
	pix.width = 640;
	pix.height = 480;
	pix.colorspace = V4L2_COLORSPACE_JPEG;
	pix.field = V4L2_FIELD_NONE;
	ret = pix_to_mbus(dev, &pix, &mbus);
	if (ret)
		return ret;

	remote = media_entity_remote_pad(&dev->pad);
	fse.index = fsize->index;
	fse.pad = remote->index;
	fse.code = mbus.code;
	ret = v4l2_subdev_call(subdev, pad, enum_frame_size, NULL, &fse);
	if (ret)
		return -EINVAL;

	vpfe_dbg(1, dev, "vpfe_enum_size: index: %d code: %x W:[%d,%d] H:[%d,%d]\n",
		fse.index, fse.code, fse.min_width, fse.max_width,
		fse.min_height, fse.max_height);

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = fse.max_width;
	fsize->discrete.height = fse.max_height;

	vpfe_dbg(1, dev, "vpfe_enum_size: index: %d pixformat: %s size: %dx%d\n",
		fsize->index, print_fourcc(fsize->pixel_format),
		fsize->discrete.width, fsize->discrete.height);

	return 0;
}

/*
 * vpfe_get_subdev_input_index - Get subdev index and subdev input index for a
 * given app input index
 */
static int vpfe_get_subdev_input_index(struct vpfe_device *dev,
					int *subdev_index,
					int *subdev_input_index,
					int app_input_index)
{
	struct vpfe_config *cfg = dev->cfg;
	struct vpfe_subdev_info *sdinfo;
	int i, j = 0;

	for (i = 0; i < cfg->num_subdevs; i++) {
		sdinfo = &cfg->sub_devs[i];
		if (app_input_index < (j + sdinfo->num_inputs)) {
			*subdev_index = i;
			*subdev_input_index = app_input_index - j;
			return 0;
		}
		j += sdinfo->num_inputs;
	}
	return -EINVAL;
}

/*
 * vpfe_get_app_input - Get app input index for a given subdev input index
 * driver stores the input index of the current sub device and translate it
 * when application request the current input
 */
static int vpfe_get_app_input_index(struct vpfe_device *dev,
				    int *app_input_index)
{
	struct vpfe_config *cfg = dev->cfg;
	struct vpfe_subdev_info *sdinfo;
	int i, j = 0;

	for (i = 0; i < cfg->num_subdevs; i++) {
		sdinfo = &cfg->sub_devs[i];
		if (!strcmp(sdinfo->name, dev->current_subdev->name)) {
			if (dev->current_input >= sdinfo->num_inputs)
				return -1;
			*app_input_index = j + dev->current_input;
			return 0;
		}
		j += sdinfo->num_inputs;
	}
	return -EINVAL;
}

static int vpfe_enum_input(struct file *file, void *priv,
				 struct v4l2_input *inp)
{
	struct vpfe_device *dev = video_drvdata(file);
	struct vpfe_subdev_info *sdinfo;
	int subdev, index;

	vpfe_dbg(2, dev, "vpfe_enum_input\n");

	if (vpfe_get_subdev_input_index(dev,
					&subdev,
					&index,
					inp->index) < 0) {
		vpfe_dbg(1, dev,
			"input information not found for the subdev\n");
		return -EINVAL;
	}
	sdinfo = &dev->cfg->sub_devs[subdev];
	memcpy(inp, &sdinfo->inputs[index], sizeof(struct v4l2_input));
	return 0;
}

static int vpfe_g_input(struct file *file, void *priv, unsigned int *index)
{
	struct vpfe_device *dev = video_drvdata(file);

	vpfe_dbg(2, dev, "vpfe_g_input\n");

	return vpfe_get_app_input_index(dev, index);
}

/* Assumes caller is holding vpfe_dev->lock */
static int vpfe_set_input(struct vpfe_device *dev, unsigned int index)
{
	struct v4l2_subdev *sd;
	struct vpfe_subdev_info *sdinfo;
	struct vb2_queue *q = &dev->buffer_queue;
	int subdev_index = 0, inp_index = 0;
	struct vpfe_route *route;
	u32 input = 0, output = 0;
	int ret = -EINVAL;

	vpfe_dbg(2, dev, "vpfe_set_input: index: %d\n", index);

	/* If streaming is started, return error */
	if (vb2_is_busy(q)) {
		vpfe_err(dev, "%s device busy\n", __func__);
		ret = -EBUSY;
		goto get_out;
	}
	ret = vpfe_get_subdev_input_index(dev,
					  &subdev_index,
					  &inp_index,
					  index);
	vpfe_dbg(2, dev,
		"vpfe_set_input: after vpfe_get_subdev_input_index: ret %d\n",
		ret);

	if (ret < 0) {
		vpfe_err(dev, "invalid input index\n");
		goto get_out;
	}

	sdinfo = &dev->cfg->sub_devs[subdev_index];
	sd = dev->sd[subdev_index];
	route = &sdinfo->routes[inp_index];
	if (route && sdinfo->can_route) {
		input = route->input;
		output = route->output;
		if (sd) {
			ret = v4l2_subdev_call(sd, video,
					s_routing, input, output, 0);
			vpfe_dbg(3, dev,
				"vpfe_set_input: after s_routing: ret %d\n",
				ret);
		}

		if (ret) {
			vpfe_err(dev,
				"vpfe_doioctl:error in setting input in sub device\n");
			ret = -EINVAL;
			goto get_out;
		}
	}

	dev->current_subdev = sdinfo;
	if (sd)
		dev->v4l2_dev.ctrl_handler = sd->ctrl_handler;
	dev->current_input = index;
	dev->std_index = 0;

	/* set the bus/interface parameter for the sub device in isif */
	ret = isif_set_hw_if_params(
			&dev->vpfe_isif, &sdinfo->isif_if_params);
	vpfe_dbg(3, dev,
		"vpfe_set_input: after set_hw_if_params: ret %d\n", ret);
	if (ret)
		goto get_out;

	/* set the default image parameters in the device */
	ret = vpfe_config_image_format(dev,
				vpfe_standards[dev->std_index].std_id);
	vpfe_dbg(3, dev,
		"vpfe_set_input: after vpfe_config_image_format: ret %d\n",
		ret);

get_out:
	return ret;
}

static int vpfe_s_input(struct file *file, void *priv, unsigned int index)
{
	struct vpfe_device *dev = video_drvdata(file);
	int ret = -EINVAL;

	vpfe_dbg(2, dev,
		"vpfe_s_input: index: %d\n", index);

	ret = vpfe_set_input(dev, index);

	return ret;
}

static int vpfe_querystd(struct file *file, void *priv, v4l2_std_id *std_id)
{
	struct vpfe_device *dev = video_drvdata(file);
	struct vpfe_subdev_info *sdinfo;
	int ret = 0;

	vpfe_dbg(2, dev, "vpfe_querystd\n");

	sdinfo = dev->current_subdev;
	if (ret)
		return ret;
	/* Call querystd function of decoder device */
	ret = v4l2_device_call_until_err(&dev->v4l2_dev, sdinfo->grp_id,
					 video, querystd, std_id);
	return ret;
}

static int vpfe_s_std(struct file *file, void *priv, v4l2_std_id std_id)
{
	struct vpfe_device *dev = video_drvdata(file);
	struct vpfe_subdev_info *sdinfo;
	struct vb2_queue *q = &dev->buffer_queue;
	int ret = 0;

	vpfe_dbg(2, dev, "vpfe_s_std\n");

	sdinfo = dev->current_subdev;
	/* If streaming is started, return error */
	if (vb2_is_busy(q)) {
		vpfe_err(dev, "%s device busy\n", __func__);
		ret = -EBUSY;
		return ret;
	}

	ret = v4l2_device_call_until_err(&dev->v4l2_dev, sdinfo->grp_id,
					 core, s_std, std_id);
	if (ret < 0) {
		vpfe_err(dev, "Failed to set standard\n");
		return ret;
	}
	ret = vpfe_config_image_format(dev, std_id);

	return ret;
}

static int vpfe_g_std(struct file *file, void *priv, v4l2_std_id *std_id)
{
	struct vpfe_device *dev = video_drvdata(file);

	vpfe_dbg(2, dev, "vpfe_g_std\n");

	*std_id = vpfe_standards[dev->std_index].std_id;
	return 0;
}
/*
 *  Videobuf operations
 */

/*
 * queue_setup : Callback function for buffer setup.
 * @vq: vb2_queue ptr
 * @fmt: v4l2 format
 * @nbuffers: ptr to number of buffers requested by application
 * @nplanes:: contains number of distinct video planes needed to hold a frame
 * @sizes[]: contains the size (in bytes) of each plane.
 * @alloc_ctxs: ptr to allocation context
 *
 * This callback function is called when reqbuf() is called to adjust
 * the buffer count and buffer size
 */

static int queue_setup(struct vb2_queue *vq,
			const struct v4l2_format *fmt,
			unsigned int *nbuffers, unsigned int *nplanes,
			unsigned int sizes[], void *alloc_ctxs[])
{
	struct vpfe_device *dev = vb2_get_drv_priv(vq);
	unsigned long size;

	vpfe_dbg(2, dev, "queue_setup\n");
	size = dev->fmt.fmt.pix.sizeimage;

	*nplanes = 1;
	sizes[0] = size;
	alloc_ctxs[0] = dev->alloc_ctx;

	vpfe_dbg(1, dev,
		"nbuffers=%d, size=%ld\n", *nbuffers, size);
	return 0;
}

/*
 * buffer_prepare :  callback function for buffer prepare
 * @vb: ptr to vb2_buffer
 *
 * This is the callback function for buffer prepare when vb2_qbuf()
 * function is called. The buffer is prepared and user space virtual address
 * or user address is converted into  physical address
 */
static int buffer_prepare(struct vb2_buffer *vb)
{
	struct vpfe_device *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct vpfe_cap_buffer *buf = container_of(vb,
					struct vpfe_cap_buffer, vb);
	unsigned long size;

	size = dev->fmt.fmt.pix.sizeimage;
	if (vb2_plane_size(vb, 0) < size) {
		vpfe_err(dev,
			"data will not fit into plane (%lu < %lu)\n",
			vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(&buf->vb, 0, size);

	return 0;
}

/*
 * buffer_queue : Callback function to add buffer to DMA queue
 * @vb: ptr to vb2_buffer
 */
static void buffer_queue(struct vb2_buffer *vb)
{
	/* Get the file handle object and device object */
	struct vpfe_device *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct vpfe_cap_buffer *buf = container_of(vb,
					struct vpfe_cap_buffer, vb);
	unsigned long flags = 0;

	/* add the buffer to the DMA queue */
	spin_lock_irqsave(&dev->dma_queue_lock, flags);
	list_add_tail(&buf->list, &dev->dma_queue);
	spin_unlock_irqrestore(&dev->dma_queue_lock, flags);
}

static void vpfe_unlock(struct vb2_queue *vq)
{
	struct vpfe_device *dev = vb2_get_drv_priv(vq);
	mutex_unlock(&dev->lock);
}

static void vpfe_lock(struct vb2_queue *vq)
{
	struct vpfe_device *dev = vb2_get_drv_priv(vq);
	mutex_lock(&dev->lock);
}

static void vpfe_calculate_offsets(struct vpfe_device *dev);
static void vpfe_start_isif_capture(struct vpfe_device *dev);

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct vpfe_device *dev = vb2_get_drv_priv(vq);
	struct v4l2_subdev *sd = dev->sd[0];
	unsigned long addr = 0;
	unsigned long flags;
	int ret = 0;

	/* If buffer queue is empty, return error */
	spin_lock_irqsave(&dev->dma_queue_lock, flags);

	/*
	 * If buffer queue is empty, return error.
	 * This check might be removed once
	 * min_buffers_needed is available.
	 */
	if (list_empty(&dev->dma_queue)) {
		spin_unlock_irqrestore(&dev->dma_queue_lock, flags);
		vpfe_dbg(1, dev, "buffer queue is empty\n");
		return -EIO;
	}

	/* Get the next frame from the buffer queue */
	dev->next_frm = list_entry(dev->dma_queue.next,
				   struct vpfe_cap_buffer, list);
	dev->cur_frm = dev->next_frm;
	/* Remove buffer from the buffer queue */
	list_del(&dev->cur_frm->list);
	spin_unlock_irqrestore(&dev->dma_queue_lock, flags);

	/* Initialize field_id and started member */
	dev->field_id = 0;
	addr = vb2_dma_contig_plane_dma_addr(&dev->cur_frm->vb, 0);

	v4l2_get_timestamp(&dev->cur_frm->vb.v4l2_buf.timestamp);

	if (sd) {
		ret = v4l2_subdev_call(sd, video, s_stream, 1);
		if (ret) {
			vpfe_err(dev, "stream on failed in subdev\n");
			return -EINVAL;
		}
	}

	/* Calculate field offset */
	vpfe_calculate_offsets(dev);

	if (vpfe_attach_irq(dev) < 0) {
		vpfe_err(dev,
			"Error in attaching interrupt handle\n");
		return -EFAULT;
	}
	if (isif_configure(&dev->vpfe_isif) < 0) {
		vpfe_err(dev,
			"Error in configuring isif\n");
		return -EINVAL;
	}

	vpfe_dbg(1, dev,
		"start_streaming: initial buffer %lx\n", addr);
	isif_setfbaddr(&dev->vpfe_isif, (unsigned long)(addr));
	vpfe_start_isif_capture(dev);
	return ret;
}

/* abort streaming and wait for last buffer */
static int stop_streaming(struct vb2_queue *vq)
{
	struct vpfe_device *dev = vb2_get_drv_priv(vq);
	struct v4l2_subdev *sd = dev->sd[0];
	unsigned long flags;
	int ret = 0;

	vpfe_dbg(1, dev, "stop_streaming\n");
	vpfe_stop_isif_capture(dev);
	vpfe_detach_irq(dev);

	if (sd) {
		ret = v4l2_subdev_call(sd, video, s_stream, 0);
		if (ret) {
			vpfe_err(dev, "stream off failed in subdev\n");
			return -EINVAL;
		}
	}

	/* release all active buffers */
	spin_lock_irqsave(&dev->dma_queue_lock, flags);
	while (!list_empty(&dev->dma_queue)) {
		dev->next_frm = list_entry(dev->dma_queue.next,
						struct vpfe_cap_buffer, list);
		list_del(&dev->next_frm->list);
		vb2_buffer_done(&dev->next_frm->vb, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&dev->dma_queue_lock, flags);

	return 0;
}

/*
 * vpfe_calculate_offsets : This function calculates buffers offset
 * for top and bottom field
 */
static void vpfe_calculate_offsets(struct vpfe_device *dev)
{
	struct v4l2_rect image_win;

	vpfe_dbg(2, dev, "vpfe_calculate_offsets\n");

	isif_get_image_window(&dev->vpfe_isif, &image_win);
	dev->field_off = image_win.height * image_win.width;
}

/* vpfe_start_isif_capture: start streaming in ccdc/isif */
static void vpfe_start_isif_capture(struct vpfe_device *dev)
{
	isif_enable(&dev->vpfe_isif, 1);
}

static int vpfe_cropcap(struct file *file, void *priv,
			      struct v4l2_cropcap *crop)
{
	struct vpfe_device *dev = video_drvdata(file);

	vpfe_dbg(2, dev, "vpfe_cropcap\n");

	if (dev->std_index >= ARRAY_SIZE(vpfe_standards))
		return -EINVAL;

	memset(crop, 0, sizeof(struct v4l2_cropcap));
	crop->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	crop->defrect.width = vpfe_standards[dev->std_index].width;
	crop->bounds.width = crop->defrect.width;
	crop->defrect.height = vpfe_standards[dev->std_index].height;
	crop->bounds.height = crop->defrect.height;
	crop->pixelaspect = vpfe_standards[dev->std_index].pixelaspect;
	return 0;
}

static int vpfe_g_selection(struct file *file, void *fh,
			   struct v4l2_selection *s)
{
	struct vpfe_device *dev = video_drvdata(file);

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = dev->crop.width;
		s->r.height = dev->crop.height;
		return 0;

	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_CROP:
		s->r = dev->crop;
		return 0;
	}

	return -EINVAL;
}

static int enclosed_rectangle(struct v4l2_rect *a, struct v4l2_rect *b)
{
	if (a->left < b->left || a->top < b->top)
		return 0;
	if (a->left + a->width > b->left + b->width)
		return 0;
	if (a->top + a->height > b->top + b->height)
		return 0;

	return 1;
}

static int vpfe_s_selection(struct file *file, void *fh,
			   struct v4l2_selection *s)
{
	struct vpfe_device *dev = video_drvdata(file);
	struct v4l2_rect r = s->r;
	struct v4l2_rect cr = dev->crop;

	v4l_bound_align_image(&r.width, 0, cr.width, 0,
			      &r.height, 0, cr.height, 0, 0);

	r.left = clamp_t(unsigned int, r.left, 0, cr.width - r.width);
	r.top  = clamp_t(unsigned int, r.top, 0, cr.height - r.height);

	if (s->flags & V4L2_SEL_FLAG_LE && !enclosed_rectangle(&r, &s->r))
		return -ERANGE;

	if (s->flags & V4L2_SEL_FLAG_GE && !enclosed_rectangle(&s->r, &r))
		return -ERANGE;

	s->r = dev->crop = r;

	isif_set_image_window(&dev->vpfe_isif, &r, dev->bpp);
	dev->fmt.fmt.pix.width = r.width;
	dev->fmt.fmt.pix.height = r.height;
	dev->fmt.fmt.pix.bytesperline = isif_get_line_length(&dev->vpfe_isif);
	dev->fmt.fmt.pix.sizeimage =
		dev->fmt.fmt.pix.bytesperline *
		dev->fmt.fmt.pix.height;

	vpfe_dbg(1, dev, "cropped (%d,%d)/%dx%d of %dx%d\n",
		r.left, r.top, r.width, r.height,
		cr.width, cr.height);

	return 0;
}

static long vpfe_ioctl_default(struct file *file, void *priv,
		bool valid_prio, unsigned int cmd, void *param)
{
	struct vpfe_device *dev = video_drvdata(file);
	struct vb2_queue *q = &dev->buffer_queue;
	int ret = 0;

	vpfe_dbg(2, dev, "vpfe_ioctl_default\n");

	if (!valid_prio) {
		vpfe_err(dev, "%s device busy\n", __func__);
		return -EBUSY;
	}

	/* If streaming is started, return error */
	if (vb2_is_busy(q)) {
		vpfe_err(dev, "%s device busy\n", __func__);
		return -EBUSY;
	}

	switch (cmd) {
	case VPFE_CMD_S_ISIF_RAW_PARAMS:
		vpfe_err(dev,
			 "VPFE_CMD_S_ISIF_RAW_PARAMS: experimental ioctl\n");
		ret = isif_set_params(&dev->vpfe_isif, param);
		if (ret) {
			vpfe_dbg(2, dev,
				"Error setting parameters in ISIF\n");
			return ret;
		}
		ret = vpfe_get_isif_image_format(dev,
						 &dev->fmt);
		if (ret < 0) {
			vpfe_dbg(2, dev,
				"Invalid image format at ISIF\n");
			return ret;
		}
		break;
	default:
		ret = -ENOTTY;
	}

	return ret;
}

static struct vb2_ops vpfe_video_qops = {
	.queue_setup		= queue_setup,
	.buf_prepare		= buffer_prepare,
	.buf_queue		= buffer_queue,
	.start_streaming	= start_streaming,
	.stop_streaming		= stop_streaming,
	.wait_prepare		= vpfe_unlock,
	.wait_finish		= vpfe_lock,
};

/* vpfe capture driver file operations */
static const struct v4l2_file_operations vpfe_fops = {
	.owner			= THIS_MODULE,
	.open			= vpfe_open,
	.release		= vpfe_release,
	.read			= vb2_fop_read,
	.poll			= vb2_fop_poll,
	.unlocked_ioctl		= video_ioctl2, /* V4L2 ioctl handler */
	.mmap			= vb2_fop_mmap,

};

/* vpfe capture ioctl operations */
static const struct v4l2_ioctl_ops vpfe_ioctl_ops = {
	.vidioc_querycap	= vpfe_querycap,
	.vidioc_g_fmt_vid_cap   = vpfe_g_fmt,
	.vidioc_enum_fmt_vid_cap = vpfe_enum_fmt,
	.vidioc_s_fmt_vid_cap   = vpfe_s_fmt,
	.vidioc_try_fmt_vid_cap = vpfe_try_fmt,
	.vidioc_enum_framesizes = vpfe_enum_size,
	.vidioc_enum_input	= vpfe_enum_input,
	.vidioc_g_input		= vpfe_g_input,
	.vidioc_s_input		= vpfe_s_input,
	.vidioc_querystd	= vpfe_querystd,
	.vidioc_s_std		= vpfe_s_std,
	.vidioc_g_std		= vpfe_g_std,

	.vidioc_reqbufs		= vb2_ioctl_reqbufs,
	.vidioc_create_bufs	= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf	= vb2_ioctl_prepare_buf,
	.vidioc_querybuf	= vb2_ioctl_querybuf,
	.vidioc_qbuf		= vb2_ioctl_qbuf,
	.vidioc_dqbuf		= vb2_ioctl_dqbuf,
	.vidioc_expbuf		= vb2_ioctl_expbuf,
	.vidioc_streamon	= vb2_ioctl_streamon,
	.vidioc_streamoff	= vb2_ioctl_streamoff,
	.vidioc_log_status	= v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,

	.vidioc_cropcap		= vpfe_cropcap,
	.vidioc_g_selection	= vpfe_g_selection,
	.vidioc_s_selection	= vpfe_s_selection,
	.vidioc_default		= vpfe_ioctl_default,
};

static struct video_device vpfe_videodev = {
	.name		= VPFE_MODULE_NAME,
	.fops		= &vpfe_fops,
	.ioctl_ops	= &vpfe_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,
	.tvnorms	= 0,
};

static int vpfe_async_bound(struct v4l2_async_notifier *notifier,
			    struct v4l2_subdev *subdev,
			    struct v4l2_async_subdev *asd)
{
	int i, j;
	struct vpfe_subdev_info *sdinfo;
	struct vpfe_device *dev = container_of(notifier->v4l2_dev,
					struct vpfe_device, v4l2_dev);

	vpfe_dbg(1, dev, "vpfe_async_bound\n");

	for (i = 0; i < dev->cfg->num_subdevs; i++) {
		sdinfo = &dev->cfg->sub_devs[i];

		if (!strcmp(sdinfo->name, subdev->name)) {
			dev->sd[i] = subdev;
			vpfe_info(dev,
				 "v4l2 sub device %s registered\n",
				 subdev->name);
			dev->sd[i]->grp_id =
					sdinfo->grp_id;
			/* update tvnorms from the sub devices */
			for (j = 0;
				 j < sdinfo->num_inputs;
				 j++) {
				dev->video_dev->tvnorms |=
					sdinfo->inputs[j].std;
			}
			return 0;
		}
	}

	vpfe_info(dev, "vpfe_async_bound sub device (%s) not matched\n",
		 subdev->name);
	return -EINVAL;
}

static int vpfe_probe_complete(struct vpfe_device *dev)
{
	int err;
	unsigned int flags;
	struct vb2_queue *q;
	struct video_device *vfd;

	spin_lock_init(&dev->dma_queue_lock);
	mutex_init(&dev->lock);

	dev->pad.flags = MEDIA_PAD_FL_SINK;
	err = media_entity_init(&dev->video_dev->entity,
				1, &dev->pad, 0);
	if (err < 0)
		return err;

	/* register video device */
	vpfe_dbg(1, dev, "trying to register vpfe device.\n");
	vpfe_dbg(1, dev,
		"video_dev=%x\n", (int)&dev->video_dev);
	dev->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	vpfe_dbg(1, dev, "%s capture driver initialized\n",
		 dev->cfg->card_name);

	/* set first sub device as current one */
	dev->current_subdev = &dev->cfg->sub_devs[0];
	dev->v4l2_dev.ctrl_handler = dev->sd[0]->ctrl_handler;

	/* Initialize videobuf2 queue as per the buffer type */
	dev->alloc_ctx = vb2_dma_contig_init_ctx(dev->pdev);
	if (IS_ERR(dev->alloc_ctx)) {
		vpfe_err(dev, "Failed to get the context\n");
		err = PTR_ERR(dev->alloc_ctx);
		goto probe_out;
	}
	q = &dev->buffer_queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	/*
	 * VB2_USERPTR support is not really possible
	 * when using videobuf2-dma-contig unless the buffer
	 * being passed is coming from another driver which
	 * allocated it using videobuf2-dma-contig as well
	 *
	 * q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_READ;
	 */
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	q->drv_priv = dev;
	q->ops = &vpfe_video_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct vpfe_cap_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	/* Feature not back-ported yet. Enable when available */
	/* q->min_buffers_needed = 3; */

	err = vb2_queue_init(q);
	if (err) {
		vpfe_err(dev, "vb2_queue_init() failed\n");
		vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
		goto probe_out;
	}

	INIT_LIST_HEAD(&dev->dma_queue);

	vfd = dev->video_dev;
	/* Pass on debug flag if it is high enough */
	vfd->debug = (debug >= 4)?1:0;
	vfd->queue = q;
	set_bit(V4L2_FL_USE_FH_PRIO, &vfd->flags);

	/*
	 * Provide a mutex to v4l2 core. It will be used to protect
	 * all fops and v4l2 ioctls.
	 */
	vfd->lock = &dev->lock;
	/* set video driver private data */
	video_set_drvdata(vfd, dev);

	/* select input 0 */
	err = vpfe_set_input(dev, 0);
	if (err)
		goto probe_out;

	/* connect subdev to video node */
	vpfe_dbg(1, dev, "sd[0]->entity is %x\n",
		(unsigned int)&dev->sd[0]->entity);
	vpfe_dbg(1, dev, "video_dev->entity is %x\n",
		(unsigned int)&dev->video_dev->entity);

	vpfe_dbg(1, dev, "source_pad:%d source->num_pads:%d\n",
		0, dev->sd[0]->entity.num_pads);
	vpfe_dbg(1, dev, "sink_pad:%d sink->num_pads:%d\n",
		0, dev->video_dev->entity.num_pads);

	flags = MEDIA_LNK_FL_ENABLED;
	err = media_entity_create_link(&dev->sd[0]->entity, 0,
				       &dev->video_dev->entity,
				       0, flags);
	if (err < 0)
		goto probe_out;

	err = v4l2_device_register_subdev_nodes(&dev->v4l2_dev);
	if (err < 0)
		goto probe_out;

	err = video_register_device(dev->video_dev,
				    VFL_TYPE_GRABBER, -1);
	if (err) {
		vpfe_err(dev,
			"Unable to register video device.\n");
		goto probe_out;
	}
	vpfe_info(dev, "%s video device registered as /dev/%s\n",
		 dev->cfg->card_name,
		 video_device_node_name(dev->video_dev));

	return 0;

probe_out:
	v4l2_device_unregister(&dev->v4l2_dev);
	return err;
}

static int vpfe_async_complete(struct v4l2_async_notifier *notifier)
{
	struct vpfe_device *dev = container_of(notifier->v4l2_dev,
					struct vpfe_device, v4l2_dev);
	return vpfe_probe_complete(dev);
}

static struct vpfe_config *
vpfe_get_pdata(struct platform_device *pdev)
{
	struct vpfe_config *pdata;
	struct device_node *endpoint, *rem;
	struct vpfe_subdev_info *sdinfo;
	const char *instance_name;

	dev_dbg(&pdev->dev, "vpfe_get_pdata\n");

	if (!IS_ENABLED(CONFIG_OF) || !pdev->dev.of_node)
		return pdev->dev.platform_data;

	dev_dbg(&pdev->dev, "vpfe_get_pdata: DT Node found\n");

	dev_dbg(&pdev->dev, "dev.of_node->name: %s\n",
		pdev->dev.of_node->name);
	dev_dbg(&pdev->dev, "dev.id: %d\n",
		pdev->dev.id);
	/* the hwmods property basically provides us with the instance
	 * name and number. So use instead of creating some other means
	 */
	of_property_read_string(pdev->dev.of_node, "ti,hwmods", &instance_name);
	dev_dbg(&pdev->dev, "hwmods: %s\n", instance_name);

	endpoint = of_graph_get_next_endpoint(pdev->dev.of_node, NULL);
	if (!endpoint)
		return NULL;

	dev_dbg(&pdev->dev, "vpfe_get_pdata: endpoint found\n");

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		goto done;

	/* Populate vpfe_config with default if needed */
	pdata->card_name = (char *)instance_name;
	pdata->isif = "AM437x ISIF";

	/* We only support one sub_devive per port */
	/* Will need cleanup */
	pdata->num_subdevs = 1;
	sdinfo = &pdata->sub_devs[0];
	sdinfo->grp_id = 0;

	/* There is only one input on the camera */
	sdinfo->num_inputs = 1;
	sdinfo->inputs[0].index = 0;
	strcpy(sdinfo->inputs[0].name, "camera");
	sdinfo->inputs[0].type = V4L2_INPUT_TYPE_CAMERA;
	sdinfo->inputs[0].std = V4L2_STD_ALL;
	sdinfo->inputs[0].capabilities = V4L2_IN_CAP_STD;

	sdinfo->can_route = 0;
	sdinfo->routes = NULL;

	of_property_read_u32(endpoint, "if_type",
			     &sdinfo->isif_if_params.if_type);
	if (sdinfo->isif_if_params.if_type < 0)
		sdinfo->isif_if_params.if_type = VPFE_RAW_BAYER;
	of_property_read_u32(endpoint, "bus_width",
			     &sdinfo->isif_if_params.bus_width);
	if (sdinfo->isif_if_params.bus_width != 10)
		sdinfo->isif_if_params.bus_width = 8;
	of_property_read_u32(endpoint, "hdpol",
			     &sdinfo->isif_if_params.hdpol);
	if (sdinfo->isif_if_params.hdpol != 0)
		sdinfo->isif_if_params.hdpol = 1;
	of_property_read_u32(endpoint, "vdpol",
			     &sdinfo->isif_if_params.vdpol);
	if (sdinfo->isif_if_params.vdpol != 0)
		sdinfo->isif_if_params.vdpol = 1;

	rem = of_graph_get_remote_port_parent(endpoint);
	if (rem == NULL) {
		dev_err(&pdev->dev, "Remote device at %s not found\n",
			endpoint->full_name);
		goto done;
	} else {
		dev_dbg(&pdev->dev, "endpoint->full_name: %s\n",
			endpoint->full_name);
		dev_dbg(&pdev->dev, "remote->full_name: %s\n",
			rem->full_name);
		dev_dbg(&pdev->dev, "remote->name: %s\n",
			rem->name);
		strncpy(sdinfo->name, rem->name, sizeof(sdinfo->name));
	}

	sdinfo->asd.match_type = V4L2_ASYNC_MATCH_OF;
	sdinfo->asd.match.of.node = rem;
	pdata->asd[0] = &sdinfo->asd;
	pdata->asd_sizes = 1;
	of_node_put(rem);

done:
	of_node_put(endpoint);
	return pdata;
}

/*
 * vpfe_probe : This function creates device entries by register
 * itself to the V4L2 driver and initializes fields of each
 * device objects
 */
static int vpfe_probe(struct platform_device *pdev)
{
	struct vpfe_subdev_info *sdinfo;
	struct vpfe_config *vpfe_cfg = vpfe_get_pdata(pdev);
	struct vpfe_device *dev;
	struct i2c_adapter *i2c_adap;
	struct video_device *vfd;
	int ret = -ENOMEM, i, j;
	int num_subdevs = 0;

	dev_dbg(&pdev->dev, "info vpfe_probe asynch\n");

	/* Allocate memory for device objects */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);

	if (!dev) {
		dev_err(&pdev->dev,
			"Failed to allocate memory for vpfe_dev\n");
		return ret;
	}

	dev->pdev = &pdev->dev;

	if (vpfe_cfg == NULL) {
		dev_err(&pdev->dev, "Unable to get vpfe config\n");
		ret = -ENODEV;
		goto probe_free_dev_mem;
	}

	dev->cfg = vpfe_cfg;
	if (NULL == vpfe_cfg->isif ||
	    NULL == vpfe_cfg->card_name ||
	    NULL == vpfe_cfg->sub_devs) {
		dev_err(&pdev->dev, "null ptr in vpfe_cfg\n");
		ret = -ENOENT;
		goto probe_free_dev_mem;
	}

	dev_dbg(&pdev->dev, "vpfe_cfg->isif: %s\n", vpfe_cfg->isif);

	/* Get VINT0 irq resource */
	dev->isif_irq0 = platform_get_irq(pdev, 0);
	if (dev->isif_irq0 < 0) {
		dev_err(&pdev->dev,
			"Unable to get interrupt for VINT0\n");
		ret = -ENODEV;
		goto probe_free_dev_mem;
	}

	dev_dbg(&pdev->dev, "Found ISIF IRQ: %d\n",
		dev->isif_irq0);

	ret = request_irq(dev->isif_irq0, vpfe_isr, IRQF_DISABLED,
			  "vpfe_capture0", dev);
	if (0 != ret) {
		dev_err(&pdev->dev, "Unable to request interrupt\n");
		goto probe_free_dev_mem;
	}

	/* Allocate memory for video device */
	vfd = video_device_alloc();
	if (NULL == vfd) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "Unable to alloc video device\n");
		goto probe_out_release_irq;
	}

	/* Initialize field of video device */
	*vfd = vpfe_videodev;

	/* Set video_dev to the video device */
	dev->video_dev = vfd;

	dev->media_dev.dev = dev->pdev;
	strcpy((char *)&dev->media_dev.model, "ti-vpfe-media");

	ret = media_device_register(&dev->media_dev);
	if (ret) {
		dev_err(&pdev->dev,
			"Unable to register media device.\n");
		goto probe_out_video_release;
	}

	dev->v4l2_dev.mdev = &dev->media_dev;

	snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name),
		"%s", dev->cfg->card_name);

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		vpfe_err(dev,
			"Unable to register v4l2 device.\n");
		goto probe_out_media_unregister;
	}

	vpfe_dbg(1, dev, "v4l2 device registered\n");

	/* set the driver data in platform device */
	platform_set_drvdata(pdev, dev);

	vfd->v4l2_dev = &dev->v4l2_dev;

	/* We have at least one sub device to work with */
	ret = vpfe_isif_init(&dev->vpfe_isif, pdev);
	if (ret) {
		vpfe_err(dev, "Error initializing isif\n");
		ret = -EINVAL;
		goto probe_out_v4l2_unregister;
	}

	num_subdevs = vpfe_cfg->num_subdevs;
	dev->sd = kmalloc(sizeof(struct v4l2_subdev *) * num_subdevs,
				GFP_KERNEL);
	if (NULL == dev->sd) {
		vpfe_err(dev,
			"unable to allocate memory for subdevice pointers\n");
		ret = -ENOMEM;
		goto probe_out_v4l2_unregister;
	}

	vpfe_dbg(2, dev, "dev->cfg->asd_sizes = %x\n",
		dev->cfg->asd_sizes);

	if (dev->cfg->asd_sizes == 0) {
		vpfe_dbg(1, dev, "Synchronous subdevice registration\n");

		i2c_adap = i2c_get_adapter(vpfe_cfg->i2c_adapter_id);

		for (i = 0; i < num_subdevs; i++) {
			struct v4l2_input *inps;

			sdinfo = &vpfe_cfg->sub_devs[i];

			/* Load up the subdevice */
			dev->sd[i] =
				v4l2_i2c_new_subdev_board(&dev->v4l2_dev,
							  i2c_adap,
							  &sdinfo->board_info,
							  NULL);
			if (dev->sd[i]) {
				vpfe_info(dev,
					 "v4l2 sub device %s registered\n",
					 sdinfo->name);
				dev->sd[i]->grp_id = sdinfo->grp_id;
				/* update tvnorms from the sub devices */
				for (j = 0; j < sdinfo->num_inputs; j++) {
					inps = &sdinfo->inputs[j];
					vfd->tvnorms |= inps->std;
				}
			} else {
				vpfe_err(dev,
					"v4l2 sub device %s register fails\n",
					sdinfo->name);
				goto probe_sd_out;
			}
		}
		vpfe_probe_complete(dev);

	} else {
		vpfe_dbg(1, dev, "Asynchronous subdevice registration\n");

		dev->notifier.subdevs = dev->cfg->asd;
		dev->notifier.num_subdevs = dev->cfg->asd_sizes;
		dev->notifier.bound = vpfe_async_bound;
		dev->notifier.complete = vpfe_async_complete;
		ret = v4l2_async_notifier_register(&dev->v4l2_dev,
						   &dev->notifier);
		if (ret) {
			vpfe_err(dev, "Error registering async notifier\n");
			ret = -EINVAL;
			goto probe_sd_out;
		}
	}

	return 0;

probe_sd_out:
	kfree(dev->sd);
probe_out_v4l2_unregister:
	v4l2_device_unregister(&dev->v4l2_dev);
probe_out_media_unregister:
	media_device_unregister(&dev->media_dev);
probe_out_video_release:
	if (!video_is_registered(dev->video_dev))
		video_device_release(dev->video_dev);
probe_out_release_irq:
	free_irq(dev->isif_irq0, dev);
probe_free_dev_mem:
	kfree(dev);
	return ret;
}

/*
 * vpfe_remove : It un-register device from V4L2 driver
 */
static int vpfe_remove(struct platform_device *pdev)
{
	struct vpfe_device *dev = platform_get_drvdata(pdev);

	vpfe_dbg(2, dev, "vpfe_remove\n");

	isif_remove(&dev->vpfe_isif, pdev);

	free_irq(dev->isif_irq0, dev);
	v4l2_async_notifier_unregister(&dev->notifier);
	kfree(dev->sd);
	v4l2_device_unregister(&dev->v4l2_dev);
	media_device_unregister(&dev->media_dev);
	video_unregister_device(dev->video_dev);
	kfree(dev);
	return 0;
}

static int vpfe_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct vpfe_device *vpfe_dev = platform_get_drvdata(pdev);
	isif_suspend(&vpfe_dev->vpfe_isif, dev);

	/* Select sleep pin state */
	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int vpfe_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct vpfe_device *vpfe_dev = platform_get_drvdata(pdev);
	isif_resume(&vpfe_dev->vpfe_isif, dev);

	/* Select default pin state */
	pinctrl_pm_select_default_state(dev);

	return 0;
}

static const struct dev_pm_ops vpfe_dev_pm_ops = {
	.suspend = vpfe_suspend,
	.resume = vpfe_resume,
};

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id vpfe_of_match[] = {
	{ .compatible = "ti,am437x-vpfe", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, vpfe_of_match);
#endif

static struct platform_driver vpfe_driver = {
	.driver = {
		.name = VPFE_MODULE_NAME,
		.owner = THIS_MODULE,
		.pm = &vpfe_dev_pm_ops,
		.of_match_table = of_match_ptr(vpfe_of_match),
	},
	.probe = vpfe_probe,
	.remove = vpfe_remove,
};

module_platform_driver(vpfe_driver);
