/*
 * TI AM437x Image Sensor Interface device API
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

#ifndef _AM437X_ISIF_H
#define _AM437X_ISIF_H

#ifdef __KERNEL__
#include <linux/videodev2.h>
#include <linux/device.h>
#include <linux/io.h>
#include "am437x_isif_regs.h"
#include "vpfe_capture.h"

enum isif_pixfmt {
	ISIF_PIXFMT_RAW,
	ISIF_PIXFMT_YCBCR_16BIT,
	ISIF_PIXFMT_YCBCR_8BIT
};

enum isif_frmfmt {
	ISIF_FRMFMT_PROGRESSIVE,
	ISIF_FRMFMT_INTERLACED
};

/* PIXEL ORDER IN MEMORY from LSB to MSB */
/* only applicable for 8-bit input mode  */
enum isif_pixorder {
	ISIF_PIXORDER_YCBYCR,
	ISIF_PIXORDER_CBYCRY,
};

enum isif_buftype {
	ISIF_BUFTYPE_FLD_INTERLEAVED,
	ISIF_BUFTYPE_FLD_SEPARATED
};


/* enum for No of pixel per line to be avg. in Black Clamping*/
enum isif_sample_length {
	ISIF_SAMPLE_1PIXELS,
	ISIF_SAMPLE_2PIXELS,
	ISIF_SAMPLE_4PIXELS,
	ISIF_SAMPLE_8PIXELS,
	ISIF_SAMPLE_16PIXELS
};

/* enum for No of lines in Black Clamping */
enum isif_sample_line {
	ISIF_SAMPLE_1LINES,
	ISIF_SAMPLE_2LINES,
	ISIF_SAMPLE_4LINES,
	ISIF_SAMPLE_8LINES,
	ISIF_SAMPLE_16LINES
};

/* enum for Alaw gamma width */
enum isif_gamma_width {
	ISIF_GAMMA_BITS_15_6,	/* use bits 15-6 for gamma */
	ISIF_GAMMA_BITS_14_5,
	ISIF_GAMMA_BITS_13_4,
	ISIF_GAMMA_BITS_12_3,
	ISIF_GAMMA_BITS_11_2,
	ISIF_GAMMA_BITS_10_1,
	ISIF_GAMMA_BITS_09_0	/* use bits 9-0 for gamma */
};

/* returns the highest bit used for the gamma */
static inline u8 isif_gamma_width_max_bit(enum isif_gamma_width width)
{
	return 15 - width;
}

enum isif_data_size {
	ISIF_DATA_16BITS,
	ISIF_DATA_15BITS,
	ISIF_DATA_14BITS,
	ISIF_DATA_13BITS,
	ISIF_DATA_12BITS,
	ISIF_DATA_11BITS,
	ISIF_DATA_10BITS,
	ISIF_DATA_8BITS
};

/* returns the highest bit used for this data size */
static inline u8 isif_data_size_max_bit(enum isif_data_size sz)
{
	return sz == ISIF_DATA_8BITS ? 7 : 15 - sz;
}

/* structure for ALaw */
struct isif_a_law {
	/* Enable/disable A-Law */
	unsigned char enable;
	/* Gamma Width Input */
	enum isif_gamma_width gamma_wd;
};

/* structure for Black Clamping */
struct isif_black_clamp {
	unsigned char enable;
	/* only if bClampEnable is TRUE */
	enum isif_sample_length sample_pixel;
	/* only if bClampEnable is TRUE */
	enum isif_sample_line sample_ln;
	/* only if bClampEnable is TRUE */
	unsigned short start_pixel;
	/* only if bClampEnable is TRUE */
	unsigned short sgain;
	/* only if bClampEnable is FALSE */
	unsigned short dc_sub;
};

/* structure for Black Level Compensation */
struct isif_black_compensation {
	/* Constant value to subtract from Red component */
	char r;
	/* Constant value to subtract from Gr component */
	char gr;
	/* Constant value to subtract from Blue component */
	char b;
	/* Constant value to subtract from Gb component */
	char gb;
};

/* Structure for ISIF configuration parameters for raw capture mode passed
 * by application
 */
struct isif_config_params_raw {
	/* data size value from 8 to 16 bits */
	enum isif_data_size data_sz;
	/* Structure for Optional A-Law */
	struct isif_a_law alaw;
	/* Structure for Optical Black Clamp */
	struct isif_black_clamp blk_clamp;
	/* Structure for Black Compensation */
	struct isif_black_compensation blk_comp;
};


/* Define for extra pixel/line and extra lines/frame */
#define NUM_EXTRAPIXELS		8
#define NUM_EXTRALINES		8

/* settings for commonly used video formats */
#define ISIF_WIN_PAL     {0, 0, 720, 576}
/* ntsc square pixel */
#define ISIF_WIN_VGA	{0, 0, (640 + NUM_EXTRAPIXELS), (480 + NUM_EXTRALINES)}
#define ISIF_WIN_SVGA	{0, 0, (800), (600)}

/* Structure for ISIF configuration parameters for raw capture mode */
struct isif_params_raw {
	/* pixel format */
	enum isif_pixfmt pix_fmt;
	/* progressive or interlaced frame */
	enum isif_frmfmt frm_fmt;
	/* v4l2 format for reference */
	struct v4l2_format fmt;
	/* video window */
	struct v4l2_rect win;
	/* Current Format Bytes Per Pixels */
	unsigned int bytesperpixel;
	/* Current Format Bytes per Lines
	 * (Aligned to 32 bytes) used for HORZ_INFO
	 */
	unsigned int bytesperline;
	/* field id polarity */
	enum vpfe_pin_pol fid_pol;
	/* vertical sync polarity */
	enum vpfe_pin_pol vd_pol;
	/* horizontal sync polarity */
	enum vpfe_pin_pol hd_pol;
	/* interleaved or separated fields */
	enum isif_buftype buf_type;
	/*
	 * enable to store the image in inverse
	 * order in memory(bottom to top)
	 */
	unsigned char image_invert_enable;
	/* configurable paramaters */
	struct isif_config_params_raw config_params;
};

struct isif_params_ycbcr {
	/* pixel format */
	enum isif_pixfmt pix_fmt;
	/* progressive or interlaced frame */
	enum isif_frmfmt frm_fmt;
	/* v4l2 format for reference */
	struct v4l2_format fmt;
	/* video window */
	struct v4l2_rect win;
	/* Current Format Bytes Per Pixels */
	unsigned int bytesperpixel;
	/* Current Format Bytes per Lines
	 * (Aligned to 32 bytes) used for HORZ_INFO
	 */
	unsigned int bytesperline;
	/* field id polarity */
	enum vpfe_pin_pol fid_pol;
	/* vertical sync polarity */
	enum vpfe_pin_pol vd_pol;
	/* horizontal sync polarity */
	enum vpfe_pin_pol hd_pol;
	/* enable BT.656 embedded sync mode */
	int bt656_enable;
	/* cb:y:cr:y or y:cb:y:cr in memory */
	enum isif_pixorder pix_order;
	/* interleaved or separated fields  */
	enum isif_buftype buf_type;
};

/*
 * isif operational configuration
 */
struct isif_oper_config {
	struct device *dev;
	/* ISIF interface type */
	enum vpfe_hw_if_type if_type;
	/* Raw Bayer configuration */
	struct isif_params_raw bayer;
	/* YCbCr configuration */
	struct isif_params_ycbcr ycbcr;
	/* ccdc base address */
	void __iomem *base_addr;
};


/*
 * isif hw operations
 */
struct vpfe_isif_device;

struct isif_hw_ops {
	/* Pointer to initialize function to initialize isif device */
	int (*open)(struct vpfe_isif_device *isif, struct device *dev);
	/* Pointer to deinitialize function */
	int (*close)(struct vpfe_isif_device *isif,
		struct device *dev);
	/* set isif base address */
	void (*set_isif_base)(struct vpfe_isif_device *isif,
		void *base, int size);
	/* Pointer to function to enable or disable isif */
	void (*enable)(struct vpfe_isif_device *isif, int en);
	/* reset sbl. only for 6446 */
	void (*reset)(struct vpfe_isif_device *isif);
	/* enable output to sdram */
	void (*enable_out_to_sdram)(struct vpfe_isif_device *isif,
		int en);
	/* Pointer to function to set hw parameters */
	int (*set_hw_if_params)(struct vpfe_isif_device *isif,
		struct vpfe_hw_if_param *param);
	/* get interface parameters */
	int (*get_hw_if_params)(struct vpfe_isif_device *isif,
		struct vpfe_hw_if_param *param);
	/*
	 * Pointer to function to set parameters. Used
	 * for implementing VPFE_S_ISIF_PARAMS
	 */
	int (*set_params)(struct vpfe_isif_device *isif,
		void *params);
	/*
	 * Pointer to function to get parameter. Used
	 * for implementing VPFE_G_ISIF_PARAMS
	 */
	int (*get_params)(struct vpfe_isif_device *isif,
		void *params);
	/* Pointer to function to configure isif */
	int (*configure)(struct vpfe_isif_device *isif);

	/* Pointer to function to set buffer type */
	int (*set_buftype)(struct vpfe_isif_device *isif,
		enum isif_buftype buf_type);
	/* Pointer to function to get buffer type */
	enum isif_buftype (*get_buftype)(struct vpfe_isif_device *isif);
	/* Pointer to function to set frame format */
	int (*set_frame_format)(struct vpfe_isif_device *isif,
		enum isif_frmfmt frm_fmt);
	/* Pointer to function to get frame format */
	enum isif_frmfmt (*get_frame_format)(struct vpfe_isif_device *isif);
	/* enumerate hw pix formats */
	int (*enum_pix)(struct vpfe_isif_device *isif, u32 *hw_pix, int i);
	/* Pointer to function to set buffer type */
	u32 (*get_pixel_format)(struct vpfe_isif_device *isif);
	/* Pointer to function to get pixel format. */
	int (*set_pixel_format)(struct vpfe_isif_device *isif, u32 pixfmt);
	/* Pointer to function to set image window */
	int (*set_image_window)(struct vpfe_isif_device *isif,
		struct v4l2_rect *win, unsigned int bpp);
	/* Pointer to function to set image window */
	void (*get_image_window)(struct vpfe_isif_device *isif,
		struct v4l2_rect *win);
	/* Pointer to function to get line length */
	unsigned int (*get_line_length)(struct vpfe_isif_device *isif);

	/* Query ISIF control IDs */
	int (*queryctrl)(struct vpfe_isif_device *isif,
		struct v4l2_queryctrl *qctrl);
	/* Set ISIF control */
	int (*set_control)(struct vpfe_isif_device *isif,
		struct v4l2_control *ctrl);
	/* Get ISIF control */
	int (*get_control)(struct vpfe_isif_device *isif,
		struct v4l2_control *ctrl);
	/* Clear Interrupt */
	void (*clear_intr)(struct vpfe_isif_device *isif, int vdint);
	/* Read Interrupt Status */
	int (*intr_status)(struct vpfe_isif_device *isif);
	/* Enable Interrupt */
	int (*intr_enable)(struct vpfe_isif_device *isif, int vdint);
	/* Disable Interrupt */
	int (*intr_disable)(struct vpfe_isif_device *isif, int vdint);

	/* Pointer to function to set frame buffer address */
	void (*setfbaddr)(struct vpfe_isif_device *isif, unsigned long addr);
	/* Pointer to function to get field id */
	int (*getfid)(struct vpfe_isif_device *isif);
};

struct vpfe_isif_device {
	/* isif device name */
	char name[32];
	/* module owner */
	struct module *owner;
	struct isif_oper_config isif_cfg;
	/* hw ops */
	struct isif_hw_ops hw_ops;
	/* ISIF Save/Restore context */
	u32 isif_ctx[ISIF_REG_END / sizeof(u32)];

};

int vpfe_isif_init(struct vpfe_isif_device *isif,
	struct platform_device *pdev);
void vpfe_isif_cleanup(struct vpfe_isif_device *isif,
	struct platform_device *pdev);
int isif_remove(struct vpfe_isif_device *isif, struct platform_device *pdev);
int isif_suspend(struct vpfe_isif_device *isif, struct device *dev);
int isif_resume(struct vpfe_isif_device *isif, struct device *dev);

#endif
#endif
