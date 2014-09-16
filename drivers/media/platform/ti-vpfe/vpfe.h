/*
 * TI VPFE capture Driver
 *
 * Copyright (C) 2013 - 2014 Texas Instruments, Inc.
 * Benoit Parrot <bparrot@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * TI VPFE Multi instance Capture Driver
 *
 */

#ifndef _VPFE_H
#define _VPFE_H

#ifdef __KERNEL__

/* Header files */
#include <media/v4l2-dev.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/io.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-dma-contig.h>

enum vpfe_pin_pol {
	VPFE_PINPOL_POSITIVE,
	VPFE_PINPOL_NEGATIVE
};

enum vpfe_hw_if_type {
	/* BT656 - 8 bit */
	VPFE_BT656,
	/* BT1120 - 16 bit */
	VPFE_BT1120,
	/* Raw Bayer */
	VPFE_RAW_BAYER,
	/* YCbCr - 8 bit with external sync */
	VPFE_YCBCR_SYNC_8,
	/* YCbCr - 16 bit with external sync */
	VPFE_YCBCR_SYNC_16,
	/* BT656 - 10 bit */
	VPFE_BT656_10BIT
};

/* interface description */
struct vpfe_hw_if_param {
	enum vpfe_hw_if_type if_type;
	enum vpfe_pin_pol hdpol;
	enum vpfe_pin_pol vdpol;
	unsigned int bus_width;
};

/* Macros */
#define VPFE_MAX_SUBDEV 2
#define VPFE_MAX_INPUTS 2

/**************************************************************************\
* Register OFFSET Definitions
\**************************************************************************/
#define ISIF_PID				0x0
#define ISIF_PCR				0x4
#define ISIF_SYN_MODE				0x8
#define ISIF_HD_VD_WID				0xc
#define ISIF_PIX_LINES				0x10
#define ISIF_HORZ_INFO				0x14
#define ISIF_VERT_START				0x18
#define ISIF_VERT_LINES				0x1c
#define ISIF_CULLING				0x20
#define ISIF_HSIZE_OFF				0x24
#define ISIF_SDOFST				0x28
#define ISIF_SDR_ADDR				0x2c
#define ISIF_CLAMP				0x30
#define ISIF_DCSUB				0x34
#define ISIF_COLPTN				0x38
#define ISIF_BLKCMP				0x3c
#define ISIF_VDINT				0x48
#define ISIF_ALAW				0x4c
#define ISIF_REC656IF				0x50
#define ISIF_CCDCFG				0x54
#define ISIF_DMA_CNTL				0x98
#define ISIF_VPFE_SYSCONFIG			0x104
#define ISIF_VPFE_CONFIG			0x108
#define ISIF_VPFE_IRQ_EOI			0x110
#define ISIF_VPFE_IRQ_STATUS_RAW		0x114
#define ISIF_VPFE_IRQ_STATUS			0x118
#define ISIF_VPFE_IRQ_ENABLE_SET		0x11c
#define ISIF_VPFE_IRQ_ENABLE_CLR		0x120
#define ISIF_REG_END				0x124

/***************************************************************
*	Define for various register bit mask and shifts for ISIF
****************************************************************/
#define ISIF_FID_POL_MASK			1
#define ISIF_FID_POL_SHIFT			4
#define ISIF_HD_POL_MASK			1
#define ISIF_HD_POL_SHIFT			3
#define ISIF_VD_POL_MASK			1
#define ISIF_VD_POL_SHIFT			2
#define ISIF_HSIZE_OFF_MASK			0xffffffe0
#define ISIF_32BYTE_ALIGN_VAL			31
#define ISIF_FRM_FMT_MASK			0x1
#define ISIF_FRM_FMT_SHIFT			7
#define ISIF_DATA_SZ_MASK			7
#define ISIF_DATA_SZ_SHIFT			8
#define ISIF_PIX_FMT_MASK			3
#define ISIF_PIX_FMT_SHIFT			12
#define ISIF_VP2SDR_DISABLE			0xFFFBFFFF
#define ISIF_WEN_ENABLE				(1 << 17)
#define ISIF_SDR2RSZ_DISABLE			0xFFF7FFFF
#define ISIF_VDHDEN_ENABLE			(1 << 16)
#define ISIF_LPF_ENABLE				(1 << 14)
#define ISIF_ALAW_ENABLE			(1 << 3)
#define ISIF_ALAW_GAMMA_WD_MASK			7
#define ISIF_BLK_CLAMP_ENABLE			(1 << 31)
#define ISIF_BLK_SGAIN_MASK			0x1F
#define ISIF_BLK_ST_PXL_MASK			0x7FFF
#define ISIF_BLK_ST_PXL_SHIFT			10
#define ISIF_BLK_SAMPLE_LN_MASK			7
#define ISIF_BLK_SAMPLE_LN_SHIFT		28
#define ISIF_BLK_SAMPLE_LINE_MASK		7
#define ISIF_BLK_SAMPLE_LINE_SHIFT		25
#define ISIF_BLK_DC_SUB_MASK			0x03FFF
#define ISIF_BLK_COMP_MASK			0xFF
#define ISIF_BLK_COMP_GB_COMP_SHIFT		8
#define ISIF_BLK_COMP_GR_COMP_SHIFT		16
#define ISIF_BLK_COMP_R_COMP_SHIFT		24
#define ISIF_LATCH_ON_VSYNC_DISABLE		(1 << 15)
#define ISIF_DATA_PACK_ENABLE			(1 << 11)
#define ISIF_HORZ_INFO_SPH_SHIFT		16
#define ISIF_VERT_START_SLV0_SHIFT		16
#define ISIF_VDINT_VDINT0_SHIFT			16
#define ISIF_VDINT_VDINT1_MASK			0xFFFF
#define ISIF_PPC_RAW				1
#define ISIF_DCSUB_DEFAULT_VAL			0
#define ISIF_CLAMP_DEFAULT_VAL			0
#define ISIF_COLPTN_VAL				0xBB11BB11
#define ISIF_TWO_BYTES_PER_PIXEL		2
#define ISIF_INTERLACED_IMAGE_INVERT		0x4B6D
#define ISIF_INTERLACED_NO_IMAGE_INVERT		0x0249
#define ISIF_PROGRESSIVE_IMAGE_INVERT		0x4000
#define ISIF_PROGRESSIVE_NO_IMAGE_INVERT	0
#define ISIF_INTERLACED_HEIGHT_SHIFT		1
#define ISIF_SYN_MODE_INPMOD_SHIFT		12
#define ISIF_SYN_MODE_INPMOD_MASK		3
#define ISIF_SYN_MODE_8BITS			(7 << 8)
#define ISIF_SYN_MODE_10BITS			(6 << 8)
#define ISIF_SYN_MODE_11BITS			(5 << 8)
#define ISIF_SYN_MODE_12BITS			(4 << 8)
#define ISIF_SYN_MODE_13BITS			(3 << 8)
#define ISIF_SYN_MODE_14BITS			(2 << 8)
#define ISIF_SYN_MODE_15BITS			(1 << 8)
#define ISIF_SYN_MODE_16BITS			(0 << 8)
#define ISIF_SYN_FLDMODE_MASK			1
#define ISIF_SYN_FLDMODE_SHIFT			7
#define ISIF_REC656IF_BT656_EN			3
#define ISIF_SYN_MODE_VD_POL_NEGATIVE		(1 << 2)
#define ISIF_CCDCFG_Y8POS_SHIFT			11
#define ISIF_CCDCFG_BW656_10BIT			(1 << 5)
#define ISIF_SDOFST_FIELD_INTERLEAVED		0x249
#define ISIF_NO_CULLING				0xffff00ff
#define ISIF_VPFE_VDINT0			(1 << 0)
#define ISIF_VPFE_VDINT1			(1 << 1)
#define ISIF_VPFE_VDINT2			(1 << 2)
#define ISIF_DMA_CNTL_OVERFLOW			(1 << 31)

#define ISIF_VPFE_CONFIG_PCLK_INV_SHIFT		0
#define ISIF_VPFE_CONFIG_PCLK_INV_MASK		1
#define ISIF_VPFE_CONFIG_PCLK_INV_NOT_INV	0
#define ISIF_VPFE_CONFIG_PCLK_INV_INV		1
#define ISIF_VPFE_CONFIG_VPFE_EN_SHIFT		1
#define ISIF_VPFE_CONFIG_VPFE_EN_MASK		2
#define ISIF_VPFE_CONFIG_VPFE_EN_DISABLE	0
#define ISIF_VPFE_CONFIG_VPFE_EN_ENABLE		1
#define ISIF_VPFE_CONFIG_VPFE_ST_SHIFT		2
#define ISIF_VPFE_CONFIG_VPFE_ST_MASK		4
#define ISIF_VPFE_CONFIG_VPFE_ST_OCP_ACTIVE	0
#define ISIF_VPFE_CONFIG_VPFE_ST_OCP_STANDBY	1

struct vpfe_pixel_format {
	struct v4l2_fmtdesc fmtdesc;
	/* bytes per pixel */
	int bpp;
};

struct vpfe_std_info {
	int active_pixels;
	int active_lines;
	/* current frame format */
	int frame_format;
};

struct vpfe_route {
	u32 input;
	u32 output;
};

struct vpfe_subdev_info {
	/* Sub device name */
	char name[32];
	/* Sub device group id */
	int grp_id;
	/* Number of inputs supported */
	int num_inputs;
	/* inputs available at the sub device */
	struct v4l2_input inputs[VPFE_MAX_INPUTS];
	/* Sub dev routing information for each input */
	struct vpfe_route *routes;
	/* check if sub dev supports routing */
	int can_route;
	/* isif bus/interface configuration */
	struct vpfe_hw_if_param isif_if_params;
	/* i2c subdevice board info */
	struct i2c_board_info board_info;
	struct v4l2_async_subdev asd;
};

struct vpfe_config {
	/* Number of sub devices connected to vpfe */
	int num_subdevs;
	/* i2c bus adapter no */
	int i2c_adapter_id;
	/* information about each subdev */
	struct vpfe_subdev_info sub_devs[VPFE_MAX_SUBDEV];
	/* evm card info */
	char *card_name;
	/* isif name */
	char *isif;
	/* Flat array, arranged in groups */
	struct v4l2_async_subdev *asd[VPFE_MAX_SUBDEV];
	int asd_sizes;
};

struct vpfe_cap_buffer {
	struct vb2_buffer vb;
	struct list_head list;
};

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

struct vpfe_isif_device;

struct vpfe_isif_device {
	/* isif device name */
	char name[32];
	/* module owner */
	struct module *owner;
	struct isif_oper_config isif_cfg;
	/* ISIF Save/Restore context */
	u32 isif_ctx[ISIF_REG_END / sizeof(u32)];

};

struct vpfe_device {
	/* V4l2 specific parameters */
	/* Identifies video device for this channel */
	struct video_device *video_dev;
	/* media pad of video entity */
	struct media_pad pad;
	/* sub devices */
	struct v4l2_subdev **sd;
	/* vpfe cfg */
	struct vpfe_config *cfg;
	/* V4l2 device */
	struct v4l2_device v4l2_dev;
	/* parent device */
	struct device *pdev;
	/* Subdevive Async Notifier */
	struct v4l2_async_notifier notifier;
	/* Indicates id of the field which is being displayed */
	u32 field_id;
	/* current interface type */
	struct vpfe_hw_if_param vpfe_if_params;
	/* ptr to currently selected sub device */
	struct vpfe_subdev_info *current_subdev;
	/* current input at the sub device */
	int current_input;
	/* Keeps track of the information about the standard */
	struct vpfe_std_info std_info;
	/* std index into std table */
	int std_index;
	/* IRQs used when CCDC/ISIF output to SDRAM */
	unsigned int isif_irq0;
	unsigned int isif_irq1;
	/* number of buffers in fbuffers */
	u32 numbuffers;
	/* List of buffer pointers for storing frames */
	u8 *fbuffers[VIDEO_MAX_FRAME];
	/* Pointer pointing to current v4l2_buffer */
	struct vpfe_cap_buffer *cur_frm;
	/* Pointer pointing to next v4l2_buffer */
	struct vpfe_cap_buffer *next_frm;
	/* Used to store pixel format */
	struct v4l2_format fmt;
	/* Used to store current bytes per pixel based on current format */
	unsigned int bpp;
	/*
	 * used when IMP is chained to store the crop window which
	 * is different from the image window
	 */
	struct v4l2_rect crop;
	/* Buffer queue used in video-buf */
	struct vb2_queue buffer_queue;
	/* Allocator-specific contexts for each plane */
	struct vb2_alloc_ctx *alloc_ctx;
	/* Queue of filled frames */
	struct list_head dma_queue;
	/* IRQ lock for DMA queue */
	spinlock_t dma_queue_lock;
	/* lock used to access this structure */
	struct mutex lock;
	/*
	 * offset where second field starts from the starting of the
	 * buffer for field separated YCbCr formats
	 */
	u32 field_off;
	/* media device */
	struct media_device media_dev;
	/* isif sub device */
	struct vpfe_isif_device vpfe_isif;
};

struct vpfe_config_params {
	u8 min_numbuffers;
	u8 numbuffers;
	u32 min_bufsize;
	u32 device_bufsize;
};

struct vpfe_v4l2_subdev {
	struct list_head list;  /* internal use only */
	struct i2c_client *client;
	void *priv;
	struct device_node *node;
};

#endif				/* End of __KERNEL__ */

/**
 * VPFE_CMD_S_ISIF_RAW_PARAMS - EXPERIMENTAL IOCTL to set raw capture params
 * This can be used to configure modules such as defect pixel correction,
 * color space conversion, culling etc. This is an experimental ioctl that
 * will change in future kernels. So use this ioctl with care !
 * TODO: This is to be split into multiple ioctls and also explore the
 * possibility of extending the v4l2 api to include this
 **/
#define VPFE_CMD_S_ISIF_RAW_PARAMS _IOW('V', BASE_VIDIOC_PRIVATE + 1, \
					void *)

#endif	/* _VPFE_H */
