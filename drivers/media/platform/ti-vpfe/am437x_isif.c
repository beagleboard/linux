/*
 * TI AM437x Image Sensor Interface
 *
 * Copyright (C) 2013 - 2014 Texas Instruments, Inc.
 *
 * Benoit Parrot <bparrot@ti.com>
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
#include "vpfe_capture.h"

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

/* Raw Bayer formats */
static u32 isif_raw_bayer_pix_formats[] = {
	V4L2_PIX_FMT_SBGGR8,
	V4L2_PIX_FMT_SBGGR16,
	V4L2_PIX_FMT_UYVY,
	V4L2_PIX_FMT_YUYV
};

/* Raw YUV formats */
static u32 isif_raw_yuv_pix_formats[] = {
	V4L2_PIX_FMT_UYVY,
	V4L2_PIX_FMT_YUYV
};

static inline u32 isif_read(struct vpfe_isif_device *isif,
	u32 offset)
{
	return readl(isif->isif_cfg.base_addr + offset);
}

static inline void isif_write(struct vpfe_isif_device *isif,
	u32 val, u32 offset)
{
	writel(val, isif->isif_cfg.base_addr + offset);
}

static void isif_enable(struct vpfe_isif_device *isif, int flag)
{
	unsigned int pcr = 0;

	pcr = isif_read(isif, ISIF_PCR);

	if (flag) { /* Enable */
		pcr = 1;
	} else { /* Disable */
		pcr = 0;
	}

	dev_dbg(isif->isif_cfg.dev,
		"isif_enable(%d) pcr:%x\n", flag, pcr);
	isif_write(isif, pcr, ISIF_PCR);
}
static void isif_module_enable(struct vpfe_isif_device *isif, int flag)
{
	unsigned int cfg = 0;

	cfg = isif_read(isif, ISIF_VPFE_CONFIG);

	if (flag) { /* Enable */
		cfg = (ISIF_VPFE_CONFIG_VPFE_EN_ENABLE<<
				ISIF_VPFE_CONFIG_VPFE_EN_SHIFT);
	} else { /* Disable */
		cfg &= ~(ISIF_VPFE_CONFIG_VPFE_EN_ENABLE<<
				ISIF_VPFE_CONFIG_VPFE_EN_SHIFT);
	}

	dev_dbg(isif->isif_cfg.dev,
		"isif_module_enable(%d) cfg:%x\n", flag, cfg);
	isif_write(isif, cfg, ISIF_VPFE_CONFIG);
}

/*
 * isif_setwin()
 * This function will configure the window size
 * to be capture in ISIF reg
 */
void isif_setwin(struct vpfe_isif_device *isif, struct v4l2_rect *image_win,
		enum isif_frmfmt frm_fmt,
		int bpp)
{
	int horz_start, horz_nr_pixels;
	int vert_start, vert_nr_lines;
	int val = 0, mid_img = 0;

	dev_dbg(isif->isif_cfg.dev, "isif_setwin: %dx%d (WxH)\n",
		image_win->width, image_win->height);
	dev_dbg(isif->isif_cfg.dev, "isif_setwin: top %d left %d\n",
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
	dev_dbg(isif->isif_cfg.dev, "End of isif_setwin...\n");
}

static void isif_readregs(struct vpfe_isif_device *isif)
{
	dev_dbg(isif->isif_cfg.dev, "ALAW: 0x%x\n",
		isif_read(isif, ISIF_ALAW));
	dev_dbg(isif->isif_cfg.dev, "CLAMP: 0x%x\n",
		isif_read(isif, ISIF_CLAMP));
	dev_dbg(isif->isif_cfg.dev, "DCSUB: 0x%x\n",
		isif_read(isif, ISIF_DCSUB));
	dev_dbg(isif->isif_cfg.dev, "BLKCMP: 0x%x\n",
		isif_read(isif, ISIF_BLKCMP));
	dev_dbg(isif->isif_cfg.dev, "COLPTN: 0x%x\n",
		isif_read(isif, ISIF_COLPTN));
	dev_dbg(isif->isif_cfg.dev, "HSIZE_OFF: 0x%x\n",
		isif_read(isif, ISIF_HSIZE_OFF));
	dev_dbg(isif->isif_cfg.dev, "SDOFST: 0x%x\n",
		isif_read(isif, ISIF_SDOFST));
	dev_dbg(isif->isif_cfg.dev, "SYN_MODE: 0x%x\n",
		isif_read(isif, ISIF_SYN_MODE));
	dev_dbg(isif->isif_cfg.dev, "HORZ_INFO: 0x%x\n",
		isif_read(isif, ISIF_HORZ_INFO));
	dev_dbg(isif->isif_cfg.dev, "VERT_START: 0x%x\n",
		isif_read(isif, ISIF_VERT_START));
	dev_dbg(isif->isif_cfg.dev, "VERT_LINES: 0x%x\n",
		isif_read(isif, ISIF_VERT_LINES));
}

static int validate_isif_param(struct vpfe_isif_device *isif,
	struct isif_config_params_raw *ccdcparam)
{
	if (ccdcparam->alaw.enable) {
		u8 max_gamma =
			isif_gamma_width_max_bit(ccdcparam->alaw.gamma_wd);
		u8 max_data =
			isif_data_size_max_bit(ccdcparam->data_sz);

		if ((ccdcparam->alaw.gamma_wd > ISIF_GAMMA_BITS_09_0) ||
		    (ccdcparam->alaw.gamma_wd < ISIF_GAMMA_BITS_15_6) ||
		    (max_gamma > max_data)) {
			dev_dbg(isif->isif_cfg.dev, "\nInvalid data line select");
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
	int dma_cntl, i, pcr;

	dev_dbg(isif->isif_cfg.dev, "isif_close\n");

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
	dev_dbg(isif->isif_cfg.dev, "isif_open\n");

	pm_runtime_get_sync(dev);
	isif_module_enable(isif, 1);
	isif_restore_defaults(isif);

	return 0;
}

/* Parameter operations */
static int isif_set_params(struct vpfe_isif_device *isif, void __user *params)
{
	struct isif_config_params_raw isif_raw_params;
	int x;

	if (isif->isif_cfg.if_type != VPFE_RAW_BAYER)
		return -EINVAL;

	x = copy_from_user(&isif_raw_params, params, sizeof(isif_raw_params));
	if (x) {
		dev_dbg(isif->isif_cfg.dev, "isif_set_params: error in copying ccdc params, %d\n",
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
void isif_config_ycbcr(struct vpfe_isif_device *isif)
{
	struct isif_params_ycbcr *params = &isif->isif_cfg.ycbcr;
	u32 syn_mode;

	dev_dbg(isif->isif_cfg.dev, "isif_config_ycbcr:\n");
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

	dev_dbg(isif->isif_cfg.dev, "\nEnd of isif_config_ycbcr...\n");
}

static void isif_config_black_clamp(struct vpfe_isif_device *isif,
	struct isif_black_clamp *bclamp)
{
	u32 val;

	if (!bclamp->enable) {
		/* configure DCSub */
		val = (bclamp->dc_sub) & ISIF_BLK_DC_SUB_MASK;
		isif_write(isif, val, ISIF_DCSUB);
		dev_dbg(isif->isif_cfg.dev,
			"\nWriting 0x%x to DCSUB...\n", val);
		isif_write(isif, ISIF_CLAMP_DEFAULT_VAL, ISIF_CLAMP);
		dev_dbg(isif->isif_cfg.dev,
			"\nWriting 0x0000 to CLAMP...\n");
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
	dev_dbg(isif->isif_cfg.dev, "\nWriting 0x%x to CLAMP...\n", val);
	/* If Black clamping is enable then make dcsub 0 */
	isif_write(isif, ISIF_DCSUB_DEFAULT_VAL, ISIF_DCSUB);
	dev_dbg(isif->isif_cfg.dev, "\nWriting 0x00000000 to DCSUB...\n");
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
void isif_config_raw(struct vpfe_isif_device *isif)
{
	struct isif_params_raw *params = &isif->isif_cfg.bayer;
	struct isif_config_params_raw *config_params =
				&isif->isif_cfg.bayer.config_params;
	unsigned int syn_mode = 0;
	unsigned int val;

	dev_dbg(isif->isif_cfg.dev, "isif_config_raw:\n");

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
		dev_dbg(isif->isif_cfg.dev, "\nWriting 0x%x to ALAW...\n", val);
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
	dev_dbg(isif->isif_cfg.dev, "Writing %d (%x) to HSIZE_OFF\n",
		params->bytesperline, params->bytesperline);

	/* Set value for SDOFST */
	if (params->frm_fmt == ISIF_FRMFMT_INTERLACED) {
		if (params->image_invert_enable) {
			/* For intelace inverse mode */
			isif_write(isif, ISIF_INTERLACED_IMAGE_INVERT,
				   ISIF_SDOFST);
			dev_dbg(isif->isif_cfg.dev, "\nWriting 0x4B6D to SDOFST..\n");
		}

		else {
			/* For intelace non inverse mode */
			isif_write(isif, ISIF_INTERLACED_NO_IMAGE_INVERT,
				   ISIF_SDOFST);
			dev_dbg(isif->isif_cfg.dev, "\nWriting 0x0249 to SDOFST..\n");
		}
	} else if (params->frm_fmt == ISIF_FRMFMT_PROGRESSIVE) {
		isif_write(isif, ISIF_PROGRESSIVE_NO_IMAGE_INVERT,
			   ISIF_SDOFST);
		dev_dbg(isif->isif_cfg.dev, "\nWriting 0x0000 to SDOFST...\n");
	}

	isif_write(isif, syn_mode, ISIF_SYN_MODE);
	dev_dbg(isif->isif_cfg.dev,
		"\nWriting 0x%x to SYN_MODE...\n", syn_mode);

	dev_dbg(isif->isif_cfg.dev, "end of isif_config_raw...\n");
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

static int isif_enum_pix(struct vpfe_isif_device *isif, u32 *pix, int i)
{
	int ret = -EINVAL;
	if (isif->isif_cfg.if_type == VPFE_RAW_BAYER) {
		if (i < ARRAY_SIZE(isif_raw_bayer_pix_formats)) {
			*pix = isif_raw_bayer_pix_formats[i];
			ret = 0;
		}
	} else {
		if (i < ARRAY_SIZE(isif_raw_yuv_pix_formats)) {
			*pix = isif_raw_yuv_pix_formats[i];
			ret = 0;
		}
	}
	return ret;
}

static int isif_set_pixel_format(struct vpfe_isif_device *isif, u32 pixfmt)
{
	dev_dbg(isif->isif_cfg.dev, "isif_set_pixel_format: if_type: %d, pixfmt:%s\n",
		isif->isif_cfg.if_type, print_fourcc(pixfmt));

	if (isif->isif_cfg.if_type == VPFE_RAW_BAYER) {
		isif->isif_cfg.bayer.pix_fmt = ISIF_PIXFMT_RAW;
		/* Need to clear it in case it was left on
		   after the last capture. */
		isif->isif_cfg.bayer.config_params.alaw.enable = 0;
		if (pixfmt == V4L2_PIX_FMT_SBGGR8)
			isif->isif_cfg.bayer.config_params.alaw.enable = 1;
		else if (pixfmt == V4L2_PIX_FMT_YUYV)
			; /*nothing for now */
		else if (pixfmt == V4L2_PIX_FMT_UYVY)
			; /*nothing for now */
		else if (pixfmt == V4L2_PIX_FMT_YUV420)
			; /*nothing for now */
		else if (pixfmt == V4L2_PIX_FMT_NV12)
			; /*nothing for now */
		else if (pixfmt == V4L2_PIX_FMT_RGB565X)
			; /*nothing for now */
		else if (pixfmt != V4L2_PIX_FMT_SBGGR16)
			return -EINVAL;
	} else {
		if (pixfmt == V4L2_PIX_FMT_YUYV)
			isif->isif_cfg.ycbcr.pix_order = ISIF_PIXORDER_YCBYCR;
		else if (pixfmt == V4L2_PIX_FMT_UYVY)
			isif->isif_cfg.ycbcr.pix_order = ISIF_PIXORDER_CBYCRY;
		else
			return -EINVAL;
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

static int isif_getfid(struct vpfe_isif_device *isif)
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
		dev_dbg(isif->isif_cfg.dev, "params.bus_width: %d\n",
			params->bus_width);
		dev_dbg(isif->isif_cfg.dev, "config_params.data_sz: %d\n",
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

static int isif_intr_status(struct vpfe_isif_device *isif)
{
	return isif_read(isif, ISIF_VPFE_IRQ_STATUS);
}

static int isif_intr_enable(struct vpfe_isif_device *isif, int vdint)
{
	unsigned int intr;
	dev_dbg(isif->isif_cfg.dev, "isif_intr_enable(%d) ", vdint);
	isif_write(isif, vdint, ISIF_VPFE_IRQ_ENABLE_SET);
	intr = isif_read(isif, ISIF_VPFE_IRQ_ENABLE_SET);
	dev_dbg(isif->isif_cfg.dev, "irq_enable_set: %x\n", intr);
	return 0;
}

static int isif_intr_disable(struct vpfe_isif_device *isif, int vdint)
{
	unsigned int intr;
	dev_dbg(isif->isif_cfg.dev, "isif_intr_disable(%d) ", vdint);
	isif_write(isif, vdint, ISIF_VPFE_IRQ_ENABLE_CLR);
	intr = isif_read(isif, ISIF_VPFE_IRQ_ENABLE_CLR);
	dev_dbg(isif->isif_cfg.dev, "irq_enable_CLR: %x\n", intr);
	return 0;
}

int isif_remove(struct vpfe_isif_device *isif, struct platform_device *pdev)
{
	struct resource	*res;

	dev_dbg(isif->isif_cfg.dev, "isif_remove\n");

	pm_runtime_disable(&pdev->dev);
	iounmap(isif->isif_cfg.base_addr);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		release_mem_region(res->start, resource_size(res));
	return 0;
}

int isif_suspend(struct vpfe_isif_device *isif, struct device *dev)
{
	dev_dbg(isif->isif_cfg.dev, "isif_suspend\n");

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

int isif_resume(struct vpfe_isif_device *isif, struct device *dev)
{
	dev_dbg(isif->isif_cfg.dev, "isif_resume\n");

	/* Enable both master and slave clock */
	pm_runtime_get_sync(dev);
	isif_module_enable(isif, 1);
	/* Restore ISIF context */
	isif_restore_context(isif);

	isif_module_enable(isif, 0);
	pm_runtime_put_sync(dev);

	return 0;
}

/*
 * vpfe_isif_init() - Initialize V4L2 subdev and media entity
 * @isif: VPFE isif module
 * @pdev: Pointer to platform device structure.
 * Return 0 on success and a negative error code on failure.
 */

static const struct isif_hw_ops isif_ops = {
	.open = isif_open,
	.close = isif_close,
	.enable = isif_enable,
	.set_hw_if_params = isif_set_hw_if_params,
	.set_params = isif_set_params,
	.configure = isif_configure,
	.set_buftype = isif_set_buftype,
	.get_buftype = isif_get_buftype,
	.enum_pix = isif_enum_pix,
	.set_pixel_format = isif_set_pixel_format,
	.get_pixel_format = isif_get_pixel_format,
	.set_frame_format = isif_set_frame_format,
	.get_frame_format = isif_get_frame_format,
	.set_image_window = isif_set_image_window,
	.get_image_window = isif_get_image_window,
	.get_line_length = isif_get_line_length,
	.setfbaddr = isif_setfbaddr,
	.getfid = isif_getfid,
	.clear_intr = isif_clear_intr,
	.intr_status = isif_intr_status,
	.intr_enable = isif_intr_enable,
	.intr_disable = isif_intr_disable,
};

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

int vpfe_isif_init(struct vpfe_isif_device *isif, struct platform_device *pdev)
{
	struct resource	*res;
	int status = 0;

	dev_dbg(&pdev->dev, "vpfe_isif_init\n");

	if (!isif) {
		dev_err(&pdev->dev, "Failed to init isif device: isif is null\n");
		return -EFAULT;
	}

	strncpy(isif->name, "AM437x ISIF", sizeof(isif->name));
	isif->owner = THIS_MODULE;

	dev_dbg(&pdev->dev, "vpfe_isif_init: %s\n", isif->name);

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

	/* Populate the function pointers table */
	isif->hw_ops = isif_ops;

	isif_config_defaults(isif);

	dev_info(isif->isif_cfg.dev,
		 "%s is registered with vpfe.\n", isif->name);

	pm_runtime_put_sync(&pdev->dev);

	return 0;

fail_nomem:
	release_mem_region(res->start, resource_size(res));
fail_nores:
	isif_remove(isif, pdev);
	return status;
}

/*
 * vpfe_isif_cleanup - isif module cleanup
 * @isif: pointer to isif subdevice
 * @dev: pointer to platform device structure
 */
void
vpfe_isif_cleanup(struct vpfe_isif_device *isif, struct platform_device *pdev)
{
	isif_remove(isif, pdev);
}
