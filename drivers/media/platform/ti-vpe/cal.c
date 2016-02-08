/*
 * TI CAL camera interface driver
 *
 * Copyright (c) 2015 Texas Instruments Inc.
 * Benoit Parrot, <bparrot@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>

#include <media/v4l2-of.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>
#include <media/v4l2-common.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include "cal_regs.h"

#define CAL_MODULE_NAME "cal"

#define MAX_WIDTH 1920
#define MAX_HEIGHT 1200

#define CAL_VERSION "0.1.0"

MODULE_DESCRIPTION("TI CAL driver");
MODULE_AUTHOR("Benoit Parrot, <bparrot@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(CAL_VERSION);

static unsigned video_nr = -1;
module_param(video_nr, uint, 0644);
MODULE_PARM_DESC(video_nr, "videoX start number, -1 is autodetect");

static unsigned debug;
module_param(debug, uint, 0644);
MODULE_PARM_DESC(debug, "activates debug info");

/* timeperframe: min/max and default */
static const struct v4l2_fract
	tpf_default = {.numerator = 1001,	.denominator = 30000};

#define cal_dbg(level, caldev, fmt, arg...)	\
		v4l2_dbg(level, debug, &caldev->v4l2_dev, fmt, ##arg)
#define cal_info(caldev, fmt, arg...)	\
		v4l2_info(&caldev->v4l2_dev, fmt, ##arg)
#define cal_err(caldev, fmt, arg...)	\
		v4l2_err(&caldev->v4l2_dev, fmt, ##arg)

#define ctx_dbg(level, ctx, fmt, arg...)	\
		v4l2_dbg(level, debug, &ctx->v4l2_dev, fmt, ##arg)
#define ctx_info(ctx, fmt, arg...)	\
		v4l2_info(&ctx->v4l2_dev, fmt, ##arg)
#define ctx_err(ctx, fmt, arg...)	\
		v4l2_err(&ctx->v4l2_dev, fmt, ##arg)

#define CAL_NUM_INPUT 1
#define CAL_NUM_CONTEXT 2

/* ------------------------------------------------------------------
	Basic structures
   ------------------------------------------------------------------*/

struct cal_fmt {
	const char *name;
	u32	fourcc;
	u32	code;
	u32	colorspace;
	u8	depth;
	bool	supported;
	u32	index;
};

static const struct cal_fmt formats[] = {
	{
		.name		= "YUV 4:2:2 packed, YCbYCr",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.code		= MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
		.depth		= 16,
		.supported	= false,
	}, {
		.name		= "YUV 4:2:2 packed, CbYCrY",
		.fourcc		= V4L2_PIX_FMT_UYVY,
		.code		= MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
		.depth		= 16,
		.supported	= false,
	}, {
		.name		= "YUV 4:2:2 packed, YCrYCb",
		.fourcc		= V4L2_PIX_FMT_YVYU,
		.code		= MEDIA_BUS_FMT_YVYU8_2X8,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
		.depth		= 16,
		.supported	= false,
	}, {
		.name		= "4YUV 4:2:2 packed, CrYCbY",
		.fourcc		= V4L2_PIX_FMT_VYUY,
		.code		= MEDIA_BUS_FMT_VYUY8_2X8,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
		.depth		= 16,
		.supported	= false,
	}, {
		.name		= "RGB565 (LE)",
		.fourcc		= V4L2_PIX_FMT_RGB565, /* gggbbbbb rrrrrggg */
		.code		= MEDIA_BUS_FMT_RGB565_2X8_LE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 16,
		.supported	= false,
	}, {
		.name		= "RGB565 (BE)",
		.fourcc		= V4L2_PIX_FMT_RGB565X, /* rrrrrggg gggbbbbb */
		.code		= MEDIA_BUS_FMT_RGB565_2X8_BE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 16,
		.supported	= false,
	}, {
		.name		= "RGB555 (LE)",
		.fourcc		= V4L2_PIX_FMT_RGB555, /* gggbbbbb arrrrrgg */
		.code		= MEDIA_BUS_FMT_RGB555_2X8_PADHI_LE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 16,
		.supported	= false,
	}, {
		.name		= "RGB555 (BE)",
		.fourcc		= V4L2_PIX_FMT_RGB555X, /* arrrrrgg gggbbbbb */
		.code		= MEDIA_BUS_FMT_RGB555_2X8_PADHI_BE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 16,
		.supported	= false,
	}, {
		.name		= "RGB24 (LE)",
		.fourcc		= V4L2_PIX_FMT_RGB24, /* rgb */
		.code		= MEDIA_BUS_FMT_RGB888_2X12_LE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 24,
		.supported	= false,
	}, {
		.name		= "RGB24 (BE)",
		.fourcc		= V4L2_PIX_FMT_BGR24, /* bgr */
		.code		= MEDIA_BUS_FMT_RGB888_2X12_BE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 24,
		.supported	= false,
	}, {
		.name		= "RGB32",
		.fourcc		= V4L2_PIX_FMT_RGB32, /* argb */
		.code		= MEDIA_BUS_FMT_ARGB8888_1X32,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 32,
		.supported	= false,
	}, {
		.name		= "RAW8 BGGR",
		.fourcc		= V4L2_PIX_FMT_SBGGR8,
		.code		= MEDIA_BUS_FMT_SBGGR8_1X8,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 8,
		.supported	= false,
	}, {
		.name		= "RAW8 GBRG",
		.fourcc		= V4L2_PIX_FMT_SGBRG8,
		.code		= MEDIA_BUS_FMT_SGBRG8_1X8,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 8,
		.supported	= false,
	}, {
		.name		= "RAW8 GRBG",
		.fourcc		= V4L2_PIX_FMT_SGRBG8,
		.code		= MEDIA_BUS_FMT_SGRBG8_1X8,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 8,
		.supported	= false,
	}, {
		.name		= "RAW8 RGGB",
		.fourcc		= V4L2_PIX_FMT_SRGGB8,
		.code		= MEDIA_BUS_FMT_SRGGB8_1X8,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 8,
		.supported	= false,
	}, {
		.name		= "RAW10 BGGR",
		.fourcc		= V4L2_PIX_FMT_SBGGR10,
		.code		= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 16,
		.supported	= false,
	}, {
		.name		= "RAW10 GBRG",
		.fourcc		= V4L2_PIX_FMT_SGBRG10,
		.code		= MEDIA_BUS_FMT_SGBRG10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 16,
		.supported	= false,
	}, {
		.name		= "RAW10 GRBG",
		.fourcc		= V4L2_PIX_FMT_SGRBG10,
		.code		= MEDIA_BUS_FMT_SGRBG10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 16,
		.supported	= false,
	}, {
		.name		= "RAW10 RGGB",
		.fourcc		= V4L2_PIX_FMT_SRGGB10,
		.code		= MEDIA_BUS_FMT_SRGGB10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 16,
		.supported	= false,
	}, {
		.name		= "RAW12 BGGR",
		.fourcc		= V4L2_PIX_FMT_SBGGR12,
		.code		= MEDIA_BUS_FMT_SBGGR12_1X12,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 16,
		.supported	= false,
	}, {
		.name		= "RAW12 GBRG",
		.fourcc		= V4L2_PIX_FMT_SGBRG12,
		.code		= MEDIA_BUS_FMT_SGBRG12_1X12,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 16,
		.supported	= false,
	}, {
		.name		= "RAW12 GRBG",
		.fourcc		= V4L2_PIX_FMT_SGRBG12,
		.code		= MEDIA_BUS_FMT_SGRBG12_1X12,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 16,
		.supported	= false,
	}, {
		.name		= "RAW12 RGGB",
		.fourcc		= V4L2_PIX_FMT_SRGGB12,
		.code		= MEDIA_BUS_FMT_SRGGB12_1X12,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.depth		= 16,
		.supported	= false,
	},
};

static const struct cal_fmt *find_format_by_pix(u32 pixelformat)
{
	const struct cal_fmt *fmt;
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(formats); k++) {
		fmt = &formats[k];
		if (fmt->fourcc == pixelformat)
			return fmt;
	}

	return NULL;
}

static const struct cal_fmt *find_format_by_code(u32 code)
{
	const struct cal_fmt *fmt;
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(formats); k++) {
		fmt = &formats[k];
		if (fmt->code == code)
			return fmt;
	}

	return NULL;
}

/* buffer for one video frame */
struct cal_buffer {
	/* common v4l buffer stuff -- must be first */
	struct vb2_buffer	vb;
	struct list_head	list;
	const struct cal_fmt	*fmt;
};

struct cal_dmaqueue {
	struct list_head	active;

	/* Counters to control fps rate */
	int			frame;
	int			ini_jiffies;
};

struct cm_data {
	void __iomem		*base;
	struct resource		*res;

	unsigned int		camerrx_control;

	struct platform_device *pdev;
};

struct cc_data {
	void __iomem		*base;
	struct resource		*res;

	struct platform_device *pdev;
};

/*
 * there is one cal_dev structure in the driver, it is shared by
 * all instances.
 */
struct cal_dev {
	int			irq;
	void __iomem		*base;
	struct resource		*res;
	struct platform_device	*pdev;
	struct v4l2_device	v4l2_dev;

	/* Controller flags for special cases */
	unsigned int		flags;

	/* Control Module handle */
	struct cm_data		*cm;
	/* Camera Core Module handle */
	struct cc_data		*cc[CAL_NUM_CSI2_PORTS];

	struct cal_ctx		*ctx[CAL_NUM_CONTEXT];
};

/*
 * There is one cal_ctx structure for each camera core context.
 */
struct cal_ctx {
	struct v4l2_device	v4l2_dev;
	struct v4l2_ctrl_handler ctrl_handler;
	struct video_device	vdev;
	struct v4l2_async_notifier notifier;
	struct v4l2_subdev	*sensor;
	struct v4l2_of_endpoint	endpoint;

	struct v4l2_async_subdev asd;
	struct v4l2_async_subdev *asd_list[1];

	struct v4l2_fh		fh;
	struct cal_dev		*dev;
	struct cc_data		*cc;

	/* v4l2_ioctl mutex */
	struct mutex		mutex;
	/* v4l2 buffers lock */
	spinlock_t		slock;

	/* Several counters */
	unsigned long		jiffies;

	struct vb2_alloc_ctx	*alloc_ctx;
	struct cal_dmaqueue	vidq;

	/* Input Number */
	int			input;

	/* video capture */
	const struct cal_fmt	*fmt;
	struct v4l2_fract	timeperframe;
	unsigned int		width, height;
	unsigned int		field;
	unsigned int		sequence;
	unsigned int		pixelsize;
	unsigned int		external_rate;
	struct vb2_queue	vb_vidq;
	unsigned int		seq_count;
	unsigned int		csi2_port;
	unsigned int		virtual_channel;

	/* Pointer pointing to current v4l2_buffer */
	struct cal_buffer	*cur_frm;
	/* Pointer pointing to next v4l2_buffer */
	struct cal_buffer	*next_frm;
};

struct cal_of_data {
	unsigned int flags;
};

static inline struct cal_ctx *notifier_to_ctx(struct v4l2_async_notifier *n)
{
	return container_of(n, struct cal_ctx, notifier);
}

/* register field read/write helpers */
static inline int get_field(u32 value, u32 mask, int shift)
{
	return (value & (mask << shift)) >> shift;
}

static inline void write_field(u32 *valp, u32 field, u32 mask, int shift)
{
	u32 val = *valp;

	val &= ~(mask << shift);
	val |= (field & mask) << shift;
	*valp = val;
}

static inline u32 cal_read(struct cal_dev *dev, int offset)
{
	return ioread32(dev->base + offset);
}

static inline void cal_write(struct cal_dev *dev, int offset, u32 value)
{
	iowrite32(value, dev->base + offset);
}

static inline int
cal_read_field(struct cal_dev *dev, int offset, u32 mask, int shift)
{
	return get_field(cal_read(dev, offset), mask, shift);
}

static inline void cal_write_field(struct cal_dev *dev, int offset, u32 field,
				   u32 mask, int shift)
{
	u32 val = cal_read(dev, offset);

	write_field(&val, field, mask, shift);

	cal_write(dev, offset, val);
}

/*
 * Control Module block access
 */
static struct cm_data *cm_create(struct cal_dev *dev)
{
	struct platform_device *pdev = dev->pdev;
	struct cm_data *cm;

	cal_dbg(3, dev, "cm_create\n");

	cm = devm_kzalloc(&pdev->dev, sizeof(*cm), GFP_KERNEL);
	if (!cm)
		return ERR_PTR(-ENOMEM);

	cm->res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"camerrx_control");
	if (!cm->res)
		return ERR_PTR(-ENODEV);

	cal_dbg(1, dev, "ioresource %s at  %pa - %pa\n",
		cm->res->name, &cm->res->start, &cm->res->end);

	cm->base = devm_ioremap_resource(&pdev->dev, cm->res);
	if (!cm->base) {
		cal_err(dev, "failed to ioremap\n");
		return ERR_PTR(-ENOMEM);
	}

	return cm;
}

static inline u32 cm_read(struct cm_data *dev, int offset)
{
	return ioread32(dev->base + offset);
}

static inline void cm_write(struct cm_data *dev, int offset, u32 value)
{
	iowrite32(value, dev->base + offset);
}

static inline void cm_write_field(struct cm_data *dev, int offset, u32 field,
				  u32 mask, int shift)
{
	u32 val = cm_read(dev, offset);

	write_field(&val, field, mask, shift);

	cm_write(dev, offset, val);
}

static void camerarx_phy_enable(struct cal_ctx *ctx)
{
	u32 val;

	ctx_dbg(3, ctx, "%s\n", __func__);

	if (!ctx->dev->cm->base) {
		ctx_err(ctx, "cm not mapped\n");
		return;
	}

	val = cm_read(ctx->dev->cm, CM_CTRL_CORE_CAMERRX_CONTROL);
	if (ctx->csi2_port == 1) {
		write_field(&val, 1, CM_CAMERRX_CTRL_CSI0_CTRLCLKEN_MASK,
			    CM_CAMERRX_CTRL_CSI0_CTRLCLKEN_SHIFT);
		write_field(&val, 0, CM_CAMERRX_CTRL_CSI0_CAMMODE_MASK,
			    CM_CAMERRX_CTRL_CSI0_CAMMODE_SHIFT);
		/* enable all lanes by default */
		write_field(&val, 0xf, CM_CAMERRX_CTRL_CSI0_LANEENABLE_MASK,
			    CM_CAMERRX_CTRL_CSI0_LANEENABLE_SHIFT);
		write_field(&val, 1, CM_CAMERRX_CTRL_CSI0_MODE_MASK,
			    CM_CAMERRX_CTRL_CSI0_MODE_SHIFT);
	} else if (ctx->csi2_port == 2) {
		write_field(&val, 1, CM_CAMERRX_CTRL_CSI1_CTRLCLKEN_MASK,
			    CM_CAMERRX_CTRL_CSI0_CTRLCLKEN_SHIFT);
		write_field(&val, 0, CM_CAMERRX_CTRL_CSI1_CAMMODE_MASK,
			    CM_CAMERRX_CTRL_CSI0_CAMMODE_SHIFT);
		/* enable all lanes by default */
		write_field(&val, 0x3, CM_CAMERRX_CTRL_CSI1_LANEENABLE_MASK,
			    CM_CAMERRX_CTRL_CSI0_LANEENABLE_SHIFT);
		write_field(&val, 1, CM_CAMERRX_CTRL_CSI1_MODE_MASK,
			    CM_CAMERRX_CTRL_CSI0_MODE_SHIFT);
	}
	cm_write(ctx->dev->cm, CM_CTRL_CORE_CAMERRX_CONTROL, val);
}

static void camerarx_phy_disable(struct cal_ctx *ctx)
{
	ctx_dbg(3, ctx, "%s\n", __func__);

	if (!ctx->dev->cm->base) {
		ctx_err(ctx, "cm not mapped\n");
		return;
	}

	if (ctx->csi2_port == 1)
		cm_write_field(ctx->dev->cm,
			       CM_CTRL_CORE_CAMERRX_CONTROL,
			       0x0,
			       CM_CAMERRX_CTRL_CSI0_CTRLCLKEN_MASK,
			       CM_CAMERRX_CTRL_CSI0_CTRLCLKEN_SHIFT);
	else if (ctx->csi2_port == 2)
		cm_write_field(ctx->dev->cm,
			       CM_CTRL_CORE_CAMERRX_CONTROL,
			       0x0,
			       CM_CAMERRX_CTRL_CSI1_CTRLCLKEN_MASK,
			       CM_CAMERRX_CTRL_CSI1_CTRLCLKEN_SHIFT);
}

/*
 * Camera Instance access block
 */
static struct cc_data *cc_create(struct cal_dev *dev, unsigned int core)
{
	struct platform_device *pdev = dev->pdev;
	struct cc_data *cc;

	cal_dbg(3, dev, "cc_create\n");

	cc = devm_kzalloc(&pdev->dev, sizeof(*cc), GFP_KERNEL);
	if (!cc)
		return ERR_PTR(-ENOMEM);

	cc->res = platform_get_resource_byname(pdev,
					       IORESOURCE_MEM,
					       (core == 0) ?
						"cal_rx_core0" :
						"cal_rx_core1");
	if (!cc->res) {
		cal_err(dev, "missing platform resources data\n");
		return ERR_PTR(-ENODEV);
	}

	cal_dbg(1, dev, "ioresource %s at  %pa - %pa\n",
		cc->res->name, &cc->res->start, &cc->res->end);

	cc->base = devm_ioremap_resource(&pdev->dev, cc->res);
	if (!cc->base) {
		cal_err(dev, "failed to ioremap\n");
		return ERR_PTR(-ENOMEM);
	}

	return cc;
}

static inline u32 cc_read(struct cc_data *dev, int offset)
{
	return ioread32(dev->base + offset);
}

static inline void cc_write(struct cc_data *dev, int offset, u32 value)
{
	iowrite32(value, dev->base + offset);
}

/*
 * Get Revision and HW info
 */
static void cal_get_hwinfo(struct cal_dev *dev)
{
	u32 revision = 0;
	u32 hwinfo = 0;

	revision = cal_read(dev, CAL_HL_REVISION);
	cal_dbg(3, dev, "CAL_HL_REVISION = 0x%08x (expecting 0x40000200)\n",
		revision);

	hwinfo = cal_read(dev, CAL_HL_HWINFO);
	cal_dbg(3, dev, "CAL_HL_HWINFO = 0x%08x (expecting 0xA3C90469)\n",
		hwinfo);
}

/*
 *   Errata i913: CSI2 LDO Needs to be disabled when module is powered on
 *
 *   Enabling CSI2 LDO shorts it to core supply. It is crucial the 2 CSI2
 *   LDOs on the device are disabled if CSI-2 module is powered on
 *   (0x4845 B304 | 0x4845 B384 [28:27] = 0x1) or in ULPS (0x4845 B304
 *   | 0x4845 B384 [28:27] = 0x2) mode. Common concerns include: high
 *   current draw on the module supply in active mode.
 *
 *   Errata does not apply when CSI-2 module is powered off
 *   (0x4845 B304 | 0x4845 B384 [28:27] = 0x0).
 *
 * SW Workaround:
 *	Set the following register bits to disable the LDO,
 *	which is essentially CSI2 REG10 bit 6:
 *
 *		Core 0:  0x4845 B828 = 0x0000 0040
 *		Core 1:  0x4845 B928 = 0x0000 0040
 */
static void i913_errata(struct cal_dev *dev, unsigned int port)
{
	u32 reg10 = cc_read(dev->cc[port], CAL_CSI2_PHY_REG10);

	write_field(&reg10, CAL_CSI2_PHY_REG0_HSCLOCKCONFIG_DISABLE,
		    CAL_CSI2_PHY_REG10_I933_LDO_DISABLE_MASK,
		    CAL_CSI2_PHY_REG10_I933_LDO_DISABLE_SHIFT);

	cal_dbg(1, dev, "CSI2_%d_REG10 = 0x%08x\n", port, reg10);
	cc_write(dev->cc[port], CAL_CSI2_PHY_REG10, reg10);
}

static inline int cal_runtime_get(struct cal_dev *dev)
{
	int r;

	cal_dbg(3, dev, "cal_runtime_get\n");

	r = pm_runtime_get_sync(&dev->pdev->dev);
	WARN_ON(r < 0);

	if (dev->flags & DRA72_CAL_PRE_ES2_LDO_DISABLE) {
		/*
		 * Apply errata on both paort eveytime we (re-)enable
		 * the clock
		 */
		i913_errata(dev, 0);
		i913_errata(dev, 1);
	}
	return r < 0 ? r : 0;
}

static inline void cal_runtime_put(struct cal_dev *dev)
{
	int r;

	cal_dbg(3, dev, "cal_runtime_put\n");

	r = pm_runtime_put_sync(&dev->pdev->dev);
	WARN_ON(r < 0 && r != -ENOSYS);
}

/*
 * Soft-Reset the Main Cal module. Not sure if this is needed.
 */
/*
static void cal_top_reset(struct cal_dev *dev)
{
	cal_write_field(dev,
			CAL_HL_SYSCONFIG,
			CAL_HL_SYSCONFIG_SOFTRESET_RESET,
			CAL_HL_SYSCONFIG_SOFTRESET_MASK,
			CAL_HL_SYSCONFIG_SOFTRESET_SHIFT);

	while(cal_read_field(dev,
			     CAL_HL_SYSCONFIG,
			     CAL_HL_SYSCONFIG_SOFTRESET_MASK,
			     CAL_HL_SYSCONFIG_SOFTRESET_SHIFT) !=
	      CAL_HL_SYSCONFIG_SOFTRESET_DONE);
}
*/

static void cal_quickdump_regs(struct cal_dev *dev)
{
	cal_info(dev, "CAL Registers @ %pa:\n", &dev->res->start);
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 16, 4,
		       dev->base, (dev->res->end - dev->res->start + 1), false);

	if (dev->ctx[0]) {
		cal_info(dev, "CSI2 Core 0 Registers @ %pa:\n",
			 &dev->ctx[0]->cc->res->start);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 16, 4,
			       dev->ctx[0]->cc->base,
			       (dev->ctx[0]->cc->res->end -
				dev->ctx[0]->cc->res->start + 1),
			       false);
	}

	if (dev->ctx[1]) {
		cal_info(dev, "CSI2 Core 1 Registers @ %pa:\n",
			 &dev->ctx[1]->cc->res->start);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 16, 4,
			       dev->ctx[1]->cc->base,
			       (dev->ctx[1]->cc->res->end -
				dev->ctx[1]->cc->res->start + 1),
			       false);
	}

	cal_info(dev, "CAMERRX_Control Registers @ %pa:\n",
		 &dev->cm->res->start);
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 16, 4,
		       dev->cm->base,
		       (dev->cm->res->end - dev->cm->res->start + 1), false);
}

/*
 * Enable the expected IRQ sources
 */
static void enable_irqs(struct cal_ctx *ctx)
{
	/* Enable IRQ_WDMA_END 0/1 */
	cal_write_field(ctx->dev,
			CAL_HL_IRQENABLE_SET(2),
			CAL_HL_IRQ_ENABLE,
			CAL_HL_IRQ_MASK(ctx->csi2_port),
			CAL_HL_IRQ_SHIFT(ctx->csi2_port));
	/* Enable IRQ_WDMA_START 0/1 */
	cal_write_field(ctx->dev,
			CAL_HL_IRQENABLE_SET(3),
			CAL_HL_IRQ_ENABLE,
			CAL_HL_IRQ_MASK(ctx->csi2_port),
			CAL_HL_IRQ_SHIFT(ctx->csi2_port));
	/* Todo: Add VC_IRQ and CSI2_COMPLEXIO_IRQ handling */
	cal_write(ctx->dev, CAL_CSI2_VC_IRQENABLE(1), 0xFF000000);
}

static void disable_irqs(struct cal_ctx *ctx)
{
	/* Disable IRQ_WDMA_END 0/1 */
	cal_write_field(ctx->dev,
			CAL_HL_IRQENABLE_CLR(2),
			CAL_HL_IRQ_CLEAR,
			CAL_HL_IRQ_MASK(ctx->csi2_port),
			CAL_HL_IRQ_SHIFT(ctx->csi2_port));
	/* Disable IRQ_WDMA_START 0/1 */
	cal_write_field(ctx->dev,
			CAL_HL_IRQENABLE_CLR(3),
			CAL_HL_IRQ_ENABLE,
			CAL_HL_IRQ_MASK(ctx->csi2_port),
			CAL_HL_IRQ_SHIFT(ctx->csi2_port));
	/* Todo: Add VC_IRQ and CSI2_COMPLEXIO_IRQ handling */
	cal_write(ctx->dev, CAL_CSI2_VC_IRQENABLE(1), 0);
}

static void csi2_init(struct cal_ctx *ctx)
{
	u32 val;

	ctx_dbg(3, ctx, "%s\n", __func__);

	val = cal_read(ctx->dev, CAL_CSI2_TIMING(ctx->csi2_port));
	write_field(&val, CAL_GEN_ENABLE,
		    CAL_CSI2_TIMING_FORCE_RX_MODE_IO1_MASK,
		    CAL_CSI2_TIMING_FORCE_RX_MODE_IO1_SHIFT);
	write_field(&val, CAL_GEN_ENABLE,
		    CAL_CSI2_TIMING_STOP_STATE_X16_IO1_MASK,
		    CAL_CSI2_TIMING_STOP_STATE_X16_IO1_SHIFT);
	write_field(&val, CAL_GEN_DISABLE,
		    CAL_CSI2_TIMING_STOP_STATE_X4_IO1_MASK,
		    CAL_CSI2_TIMING_STOP_STATE_X4_IO1_SHIFT);
	write_field(&val, 407, CAL_CSI2_TIMING_STOP_STATE_COUNTER_IO1_MASK,
		    CAL_CSI2_TIMING_STOP_STATE_COUNTER_IO1_SHIFT);
	cal_write(ctx->dev, CAL_CSI2_TIMING(ctx->csi2_port), val);
	ctx_dbg(3, ctx, "CAL_CSI2_TIMING(%d) = 0x%08x\n", ctx->csi2_port,
		cal_read(ctx->dev, CAL_CSI2_TIMING(ctx->csi2_port)));

	val = cal_read(ctx->dev, CAL_CSI2_COMPLEXIO_CFG(ctx->csi2_port));
	write_field(&val, CAL_CSI2_COMPLEXIO_CFG_RESET_CTRL_OPERATIONAL,
		    CAL_CSI2_COMPLEXIO_CFG_RESET_CTRL_MASK,
		    CAL_CSI2_COMPLEXIO_CFG_RESET_CTRL_SHIFT);
	write_field(&val, CAL_CSI2_COMPLEXIO_CFG_PWR_CMD_STATE_ON,
		    CAL_CSI2_COMPLEXIO_CFG_PWR_CMD_MASK,
		    CAL_CSI2_COMPLEXIO_CFG_PWR_CMD_SHIFT);
	cal_write(ctx->dev, CAL_CSI2_COMPLEXIO_CFG(ctx->csi2_port), val);
	while (cal_read_field(ctx->dev,
			      CAL_CSI2_COMPLEXIO_CFG(ctx->csi2_port),
			      CAL_CSI2_COMPLEXIO_CFG_PWR_STATUS_MASK,
			      CAL_CSI2_COMPLEXIO_CFG_PWR_STATUS_SHIFT) !=
	       CAL_CSI2_COMPLEXIO_CFG_PWR_STATUS_STATE_ON)
		;
	ctx_dbg(3, ctx, "CAL_CSI2_COMPLEXIO_CFG(%d) = 0x%08x\n", ctx->csi2_port,
		cal_read(ctx->dev, CAL_CSI2_COMPLEXIO_CFG(ctx->csi2_port)));

	val = cal_read(ctx->dev, CAL_CTRL);
	write_field(&val, CAL_CTRL_BURSTSIZE_BURST128,
		    CAL_CTRL_BURSTSIZE_MASK, CAL_CTRL_BURSTSIZE_SHIFT);
	write_field(&val, 0xF,
		    CAL_CTRL_TAGCNT_MASK, CAL_CTRL_TAGCNT_SHIFT);
	write_field(&val, CAL_CTRL_POSTED_WRITES_NONPOSTED,
		    CAL_CTRL_POSTED_WRITES_MASK, CAL_CTRL_POSTED_WRITES_SHIFT);
	write_field(&val, 0xFF,
		    CAL_CTRL_MFLAGL_MASK, CAL_CTRL_MFLAGL_SHIFT);
	write_field(&val, 0xFF,
		    CAL_CTRL_MFLAGH_MASK, CAL_CTRL_MFLAGH_SHIFT);
	cal_write(ctx->dev, CAL_CTRL, val);
	ctx_dbg(3, ctx, "CAL_CTRL = 0x%08x\n", cal_read(ctx->dev, CAL_CTRL));
}

static void csi2_lane_config(struct cal_ctx *ctx)
{
	u32 val = cal_read(ctx->dev, CAL_CSI2_COMPLEXIO_CFG(ctx->csi2_port));
	u32 lane_shift = CAL_CSI2_COMPLEXIO_CFG_CLOCK_POSITION_SHIFT;
	u32 lane_mask = CAL_CSI2_COMPLEXIO_CFG_CLOCK_POSITION_MASK;
	u32 polarity_mask = CAL_CSI2_COMPLEXIO_CFG_CLOCK_POL_MASK;
	struct v4l2_of_bus_mipi_csi2 *mipi_csi2 = &ctx->endpoint.bus.mipi_csi2;
	int lane;

	ctx_dbg(3, ctx, "%s\n", __func__);

	write_field(&val, mipi_csi2->clock_lane + 1,
		    lane_mask, lane_shift);
	write_field(&val, mipi_csi2->lane_polarities[0],
		    polarity_mask, lane_shift + 3);
	for (lane = 0; lane < mipi_csi2->num_data_lanes; lane++) {
		/*
		 * Every lane are one nibble apart starting with the
		 * clock followed by the data lanes so shift incements by 4.
		 */
		lane_shift += 4;
		write_field(&val, mipi_csi2->data_lanes[lane] + 1,
			    lane_mask, lane_shift);
		write_field(&val, mipi_csi2->lane_polarities[lane + 1],
			    polarity_mask, lane_shift + 3);
	}
	cal_write(ctx->dev, CAL_CSI2_COMPLEXIO_CFG(ctx->csi2_port), val);
	ctx_dbg(3, ctx, "CAL_CSI2_COMPLEXIO_CFG(%d) = 0x%08x\n",
		ctx->csi2_port, val);
}

static void csi2_ppi_enable(struct cal_ctx *ctx)
{
	ctx_dbg(3, ctx, "%s\n", __func__);

	cal_write_field(ctx->dev,
			CAL_CSI2_PPI_CTRL(ctx->csi2_port),
			CAL_GEN_ENABLE,
			CAL_CSI2_PPI_CTRL_IF_EN_MASK,
			CAL_CSI2_PPI_CTRL_IF_EN_SHIFT);
}

static void csi2_ppi_disable(struct cal_ctx *ctx)
{
	ctx_dbg(3, ctx, "%s\n", __func__);

	cal_write_field(ctx->dev,
			CAL_CSI2_PPI_CTRL(ctx->csi2_port),
			CAL_GEN_DISABLE,
			CAL_CSI2_PPI_CTRL_IF_EN_MASK,
			CAL_CSI2_PPI_CTRL_IF_EN_SHIFT);
}

static void csi2_ctx_config(struct cal_ctx *ctx)
{
	u32 val;

	ctx_dbg(3, ctx, "%s\n", __func__);

	val = cal_read(ctx->dev, CAL_CSI2_CTX0(ctx->csi2_port));
	write_field(&val, ctx->csi2_port, CAL_CSI2_CTX_CPORT_MASK,
		    CAL_CSI2_CTX_CPORT_SHIFT);
	/* DT type: MIPI CSI-2 Specs
	      1: All DT filter is disabled
	   0x24: RGB888 1 pixel  = 3 bytes
	   0x2B: RAW10  4 pixels = 5 bytes
	   0x2A: RAW8   1 pixel  = 1 byte
	   0x1E: YUV422 2 pixels = 4 bytes
	 */
	write_field(&val, 0x1, CAL_CSI2_CTX_DT_MASK,
		    CAL_CSI2_CTX_DT_SHIFT);
	/* Virtual Channel from the CSI2 sensor usually 0! */
	write_field(&val, ctx->virtual_channel, CAL_CSI2_CTX_VC_MASK,
		    CAL_CSI2_CTX_VC_SHIFT);
	/* NUM_LINES_PER_FRAME => 0 means we don't know */
	write_field(&val, 0, CAL_CSI2_CTX_LINES_MASK,
		    CAL_CSI2_CTX_LINES_SHIFT);
	write_field(&val, CAL_CSI2_CTX_ATT_PIX, CAL_CSI2_CTX_ATT_MASK,
		    CAL_CSI2_CTX_ATT_SHIFT);
	cal_write_field(ctx->dev,
			CAL_CSI2_CTX0(ctx->csi2_port),
			CAL_CSI2_CTX_PACK_MODE_LINE,
			CAL_CSI2_CTX_PACK_MODE_MASK,
			CAL_CSI2_CTX_PACK_MODE_SHIFT);
	write_field(&val, CAL_CSI2_CTX_PACK_MODE_LINE,
		    CAL_CSI2_CTX_PACK_MODE_MASK, CAL_CSI2_CTX_PACK_MODE_SHIFT);
	cal_write(ctx->dev, CAL_CSI2_CTX0(ctx->csi2_port), val);
	ctx_dbg(3, ctx, "CAL_CSI2_CTX0(%d) = 0x%08x\n", ctx->csi2_port,
		cal_read(ctx->dev, CAL_CSI2_CTX0(ctx->csi2_port)));
}

static void pix_proc_config(struct cal_ctx *ctx)
{
	u32 val;

	ctx_dbg(3, ctx, "%s\n", __func__);

	val = cal_read(ctx->dev, CAL_PIX_PROC(ctx->csi2_port));
	write_field(&val, CAL_PIX_PROC_EXTRACT_B8, CAL_PIX_PROC_EXTRACT_MASK,
		    CAL_PIX_PROC_EXTRACT_SHIFT);
	write_field(&val, CAL_PIX_PROC_DPCMD_BYPASS, CAL_PIX_PROC_DPCMD_MASK,
		    CAL_PIX_PROC_DPCMD_SHIFT);
	write_field(&val, CAL_PIX_PROC_DPCME_BYPASS, CAL_PIX_PROC_DPCME_MASK,
		    CAL_PIX_PROC_DPCME_SHIFT);
	write_field(&val, CAL_PIX_PROC_PACK_B8, CAL_PIX_PROC_PACK_MASK,
		    CAL_PIX_PROC_PACK_SHIFT);
	write_field(&val, ctx->csi2_port, CAL_PIX_PROC_CPORT_MASK,
		    CAL_PIX_PROC_CPORT_SHIFT);
	cal_write_field(ctx->dev,
			CAL_PIX_PROC(ctx->csi2_port),
			CAL_GEN_ENABLE,
			CAL_PIX_PROC_EN_MASK,
			CAL_PIX_PROC_EN_SHIFT);
	write_field(&val, CAL_GEN_ENABLE, CAL_PIX_PROC_EN_MASK,
		    CAL_PIX_PROC_EN_SHIFT);
	cal_write(ctx->dev, CAL_PIX_PROC(ctx->csi2_port), val);
	ctx_dbg(3, ctx, "CAL_PIX_PROC(%d) = 0x%08x\n", ctx->csi2_port,
		cal_read(ctx->dev, CAL_PIX_PROC(ctx->csi2_port)));
}

#define bytes_per_line(pixel, bpp) (ALIGN(pixel * bpp, 16))

static void cal_wr_dma_config(struct cal_ctx *ctx,
			      unsigned int width)
{
	u32 val;

	ctx_dbg(3, ctx, "%s\n", __func__);

	val = cal_read(ctx->dev, CAL_WR_DMA_CTRL(ctx->csi2_port));
	write_field(&val, ctx->csi2_port, CAL_WR_DMA_CTRL_CPORT_MASK,
		    CAL_WR_DMA_CTRL_CPORT_SHIFT);
	write_field(&val, CAL_WR_DMA_CTRL_DTAG_PIX_DAT,
		    CAL_WR_DMA_CTRL_DTAG_MASK, CAL_WR_DMA_CTRL_DTAG_SHIFT);
	write_field(&val, CAL_WR_DMA_CTRL_MODE_CONST,
		    CAL_WR_DMA_CTRL_MODE_MASK, CAL_WR_DMA_CTRL_MODE_SHIFT);
	write_field(&val, CAL_WR_DMA_CTRL_PATTERN_LINEAR,
		    CAL_WR_DMA_CTRL_PATTERN_MASK,
		    CAL_WR_DMA_CTRL_PATTERN_SHIFT);
	write_field(&val, CAL_GEN_ENABLE,
		    CAL_WR_DMA_CTRL_STALL_RD_MASK,
		    CAL_WR_DMA_CTRL_STALL_RD_SHIFT);
	cal_write(ctx->dev, CAL_WR_DMA_CTRL(ctx->csi2_port), val);
	ctx_dbg(3, ctx, "CAL_WR_DMA_CTRL(%d) = 0x%08x\n", ctx->csi2_port,
		cal_read(ctx->dev, CAL_WR_DMA_CTRL(ctx->csi2_port)));

	/*
	 * width/16 not sure but giving it a whirl.
	 * zero does not work right
	 */
	cal_write_field(ctx->dev,
			CAL_WR_DMA_OFST(ctx->csi2_port),
			(width / 16),
			CAL_WR_DMA_OFST_MASK,
			CAL_WR_DMA_OFST_SHIFT);
	ctx_dbg(3, ctx, "CAL_WR_DMA_OFST(%d) = 0x%08x\n", ctx->csi2_port,
		cal_read(ctx->dev, CAL_WR_DMA_OFST(ctx->csi2_port)));

	val = cal_read(ctx->dev, CAL_WR_DMA_XSIZE(ctx->csi2_port));
	/* 64 bit word means no skipping */
	write_field(&val, 0, CAL_WR_DMA_XSIZE_XSKIP_MASK,
		    CAL_WR_DMA_XSIZE_XSKIP_SHIFT);
	/*
	 * (width*8)/64 this should be size of an entire line
	 * in 64bit word but 0 means all data until the end
	 * is detected automagically
	 */
	write_field(&val, (width / 8), CAL_WR_DMA_XSIZE_MASK,
		    CAL_WR_DMA_XSIZE_SHIFT);
	cal_write(ctx->dev, CAL_WR_DMA_XSIZE(ctx->csi2_port), val);
	ctx_dbg(3, ctx, "CAL_WR_DMA_XSIZE(%d) = 0x%08x\n", ctx->csi2_port,
		cal_read(ctx->dev, CAL_WR_DMA_XSIZE(ctx->csi2_port)));
}

static void cal_wr_dma_addr(struct cal_ctx *ctx, unsigned int dmaaddr)
{
	cal_write(ctx->dev, CAL_WR_DMA_ADDR(ctx->csi2_port), dmaaddr);
/*	ctx_dbg(3, ctx, "CAL_WR_DMA_ADDR(%d) = 0x%08x\n", ctx->csi2_port,
		cal_read(ctx->dev,CAL_WR_DMA_ADDR(ctx->csi2_port))); */
}

/*
 * TCLK values are OK at their reset values
 */
#define TCLK_TERM	0
#define TCLK_MISS	1
#define TCLK_SETTLE	14
#define THS_SETTLE	15

static void csi2_phy_config(struct cal_ctx *ctx)
{
	unsigned int reg0, reg1;
	unsigned int ths_term, ths_settle;

	ctx_dbg(3, ctx, "%s\n", __func__);

#ifdef LEGACY_CSI2PHY_FORMULA
	{
	int csi2_ddrclk_khz;

	csi2_ddrclk_khz = ctx->external_rate / 1000
		/ (2 * ctx->endpoint.bus.mipi_csi2.num_data_lanes)
		* ctx->fmt->depth;

	/*
	 * THS_TERM: Programmed value = ceil(12.5 ns/DDRClk period) - 1.
	 * THS_SETTLE: Programmed value = ceil(90 ns/DDRClk period) + 3.
	 */
	ths_term = DIV_ROUND_UP(25 * csi2_ddrclk_khz, 2000000) - 1;
	ths_settle = DIV_ROUND_UP(90 * csi2_ddrclk_khz, 1000000) + 3;
	}
#else
	{
	unsigned int ddrclkperiod_us;

	/*
	 * THS_TERM: Programmed value = floor(20 ns/DDRClk period) - 2.
	 */
	ddrclkperiod_us = ctx->external_rate / 2000000;
	ddrclkperiod_us = 1000000 / ddrclkperiod_us;
	ctx_dbg(1, ctx, "ddrclkperiod_us: %d\n", ddrclkperiod_us);

	ths_term = 20000 / ddrclkperiod_us;
	ths_term = (ths_term >= 2) ? ths_term - 2 : ths_term;
	ctx_dbg(1, ctx, "ths_term: %d (0x%02x)\n", ths_term, ths_term);

	/*
	 * THS_SETTLE: Programmed value = floor(176.3 ns/CtrlClk period) - 1.
	 *	Since CtrlClk is fixed at 96Mhz then we get
	 *	ths_settle = floor(176.3 / 10.416) - 1 = 15
	 * If we ever switch to a dynamic clock then this code might be useful
	 *
	 * unsigned int ctrlclkperiod_us;
	 * ctrlclkperiod_us = 96000000 / 1000000;
	 * ctrlclkperiod_us = 1000000 / ctrlclkperiod_us;
	 * ctx_dbg(1, ctx, "ctrlclkperiod_us: %d\n", ctrlclkperiod_us);

	 * ths_settle = 176300  / ctrlclkperiod_us;
	 * ths_settle = (ths_settle > 1) ? ths_settle - 1 : ths_settle;
	 */

	ths_settle = THS_SETTLE;
	ctx_dbg(1, ctx, "ths_settle: %d (0x%02x)\n", ths_settle, ths_settle);
	}
#endif
	reg0 = cc_read(ctx->cc, CAL_CSI2_PHY_REG0);
	write_field(&reg0, CAL_CSI2_PHY_REG0_HSCLOCKCONFIG_DISABLE,
		    CAL_CSI2_PHY_REG0_HSCLOCKCONFIG_MASK,
		    CAL_CSI2_PHY_REG0_HSCLOCKCONFIG_SHIFT);
	write_field(&reg0, ths_term,
		    CAL_CSI2_PHY_REG0_THS_TERM_MASK,
		    CAL_CSI2_PHY_REG0_THS_TERM_SHIFT);
	write_field(&reg0, ths_settle,
		    CAL_CSI2_PHY_REG0_THS_SETTLE_MASK,
		    CAL_CSI2_PHY_REG0_THS_SETTLE_SHIFT);

	ctx_dbg(1, ctx, "CSI2_%d_REG0 = 0x%08x\n", (ctx->csi2_port - 1), reg0);
	cc_write(ctx->cc, CAL_CSI2_PHY_REG0, reg0);

	reg1 = cc_read(ctx->cc, CAL_CSI2_PHY_REG1);
	write_field(&reg1, TCLK_TERM,
		    CAL_CSI2_PHY_REG1_TCLK_TERM_MASK,
		    CAL_CSI2_PHY_REG1_TCLK_TERM_SHIFT);
	write_field(&reg1, 0xb8,
		    CAL_CSI2_PHY_REG1_DPHY_HS_SYNC_PATTERN_MASK,
		    CAL_CSI2_PHY_REG1_DPHY_HS_SYNC_PATTERN_SHIFT);
	write_field(&reg1, TCLK_MISS,
		    CAL_CSI2_PHY_REG1_CTRLCLK_DIV_FACTOR_MASK,
		    CAL_CSI2_PHY_REG1_CTRLCLK_DIV_FACTOR_SHIFT);
	write_field(&reg1, TCLK_SETTLE,
		    CAL_CSI2_PHY_REG1_TCLK_SETTLE_MASK,
		    CAL_CSI2_PHY_REG1_TCLK_SETTLE_SHIFT);

	ctx_dbg(1, ctx, "CSI2_%d_REG1 = 0x%08x\n", (ctx->csi2_port - 1), reg1);
	cc_write(ctx->cc, CAL_CSI2_PHY_REG1, reg1);
}

static int cal_get_external_info(struct cal_ctx *ctx)
{
	struct v4l2_ext_control ctrl_ext;
	struct v4l2_ext_controls ctrls_ext;
	int ret;

	ctx_dbg(3, ctx, "%s\n", __func__);

	memset(&ctrls_ext, 0, sizeof(ctrls_ext));
	memset(&ctrl_ext, 0, sizeof(ctrl_ext));

	ctrl_ext.id = V4L2_CID_PIXEL_RATE;

	ctrls_ext.count = 1;
	ctrls_ext.controls = &ctrl_ext;

	ret = v4l2_g_ext_ctrls(&ctx->ctrl_handler, &ctrls_ext);
	if (ret < 0) {
		ctx_err(ctx, "no pixel rate control in subdev: %s\n",
			ctx->sensor->name);
		return -EPIPE;
	}

	ctx->external_rate = ctrl_ext.value64;
	ctx_dbg(3, ctx, "sensor Pixel Rate: %d\n", ctx->external_rate);

	return 0;
}

static inline void cal_schedule_next_buffer(struct cal_ctx *ctx)
{
	struct cal_dmaqueue *dma_q = &ctx->vidq;
	struct cal_buffer *buf;
	unsigned long addr;

	buf = list_entry(dma_q->active.next, struct cal_buffer, list);
	ctx->next_frm = buf;
	list_del(&buf->list);

	addr = vb2_dma_contig_plane_dma_addr(&buf->vb, 0);
	cal_wr_dma_addr(ctx, addr);
}

static inline void cal_process_buffer_complete(struct cal_ctx *ctx)
{
	v4l2_get_timestamp(&ctx->cur_frm->vb.v4l2_buf.timestamp);
	ctx->cur_frm->vb.v4l2_buf.field = ctx->field;
	ctx->cur_frm->vb.v4l2_buf.sequence = ctx->sequence++;

	vb2_buffer_done(&ctx->cur_frm->vb, VB2_BUF_STATE_DONE);
	ctx->cur_frm = ctx->next_frm;
}

#define isvcirqset(irq, vc, ff) (irq & \
	(CAL_CSI2_VC_IRQENABLE_ ##ff ##_IRQ_##vc ##_MASK << \
	 CAL_CSI2_VC_IRQENABLE_ ##ff ##_IRQ_##vc ##_SHIFT))

#define isportirqset(irq, port) (irq & \
	(CAL_HL_IRQ_MASK(port) << CAL_HL_IRQ_SHIFT(port)))

static irqreturn_t cal_irq(int irq_cal, void *data)
{
	struct cal_dev *dev = (struct cal_dev *)data;
	struct cal_ctx *ctx;
	struct cal_dmaqueue *dma_q;
	u32 irqst2, irqst3;

	/* Check which DMA just finished */
	irqst2 = cal_read(dev, CAL_HL_IRQSTATUS(2));
	if (irqst2) {
		/* Clear Interrupt status */
		cal_write(dev, CAL_HL_IRQSTATUS(2), irqst2);

		/* Need to check both port */
		if (isportirqset(irqst2, 1)) {
			ctx = dev->ctx[0];

			if (ctx->cur_frm != ctx->next_frm)
				cal_process_buffer_complete(ctx);
		}

		if (isportirqset(irqst2, 2)) {
			ctx = dev->ctx[1];

			if (ctx->cur_frm != ctx->next_frm)
				cal_process_buffer_complete(ctx);
		}
	}

	/* Check which DMA just started */
	irqst3 = cal_read(dev, CAL_HL_IRQSTATUS(3));
	if (irqst3) {
		/* Clear Interrupt status */
		cal_write(dev, CAL_HL_IRQSTATUS(3), irqst3);

		/* Need to check both port */
		if (isportirqset(irqst3, 1)) {
			ctx = dev->ctx[0];
			dma_q = &ctx->vidq;

			spin_lock(&ctx->slock);
			if (!list_empty(&dma_q->active) &&
			    ctx->cur_frm == ctx->next_frm)
				cal_schedule_next_buffer(ctx);
			spin_unlock(&ctx->slock);
		}

		if (isportirqset(irqst3, 2)) {
			ctx = dev->ctx[1];
			dma_q = &ctx->vidq;

			spin_lock(&ctx->slock);
			if (!list_empty(&dma_q->active) &&
			    ctx->cur_frm == ctx->next_frm)
				cal_schedule_next_buffer(ctx);
			spin_unlock(&ctx->slock);
		}
	}

	return IRQ_HANDLED;
}

/*
 * video ioctls
 */
static int cal_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	struct cal_ctx *ctx = video_drvdata(file);

	strcpy(cap->driver, CAL_MODULE_NAME);
	strcpy(cap->card, CAL_MODULE_NAME);
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "platform:%s", ctx->v4l2_dev.name);
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_READWRITE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int cal_enum_fmt_vid_cap(struct file *file, void  *priv,
				struct v4l2_fmtdesc *f)
{
	const struct cal_fmt *fmt = NULL;
	u32 k;

	if (f->index >= ARRAY_SIZE(formats))
		return -EINVAL;

	for (k = 0; k < ARRAY_SIZE(formats); k++) {
		if ((formats[k].index == f->index) &&
		    (formats[k].supported)) {
			fmt = &formats[k];
			break;
		}
	}
	if (!fmt)
		return -EINVAL;

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	return 0;
}

static int __subdev_get_format(struct cal_ctx *ctx,
			       struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_subdev_format sd_fmt;
	struct v4l2_mbus_framefmt *mbus_fmt = &sd_fmt.format;
	int ret;

	ctx_dbg(2, ctx, "%s\n", __func__);

	if (!ctx->sensor)
		return -EINVAL;

	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sd_fmt.pad = 0;

	ret = v4l2_subdev_call(ctx->sensor, pad, get_fmt, NULL, &sd_fmt);
	if (ret)
		return ret;

	*fmt = *mbus_fmt;

	ctx_dbg(1, ctx, "%s %dx%d code:%04X\n", __func__,
		fmt->width, fmt->height, fmt->code);

	return 0;
}

static int __subdev_set_format(struct cal_ctx *ctx,
			       struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_subdev_format sd_fmt;
	struct v4l2_mbus_framefmt *mbus_fmt = &sd_fmt.format;
	int ret;

	ctx_dbg(2, ctx, "%s\n", __func__);

	if (!ctx->sensor)
		return -EINVAL;

	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sd_fmt.pad = 0;
	*mbus_fmt = *fmt;

	ret = v4l2_subdev_call(ctx->sensor, pad, set_fmt, NULL, &sd_fmt);
	if (ret)
		return ret;

	ctx_dbg(1, ctx, "%s %dx%d code:%04X\n", __func__,
		fmt->width, fmt->height, fmt->code);

	return 0;
}

static int cal_g_fmt_vid_cap(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct cal_ctx *ctx = video_drvdata(file);
	const struct cal_fmt *fmt;
	struct v4l2_mbus_framefmt mbus_fmt;
	int ret;

	ret = __subdev_get_format(ctx, &mbus_fmt);
	if (ret)
		return ret;

	fmt = find_format_by_code(mbus_fmt.code);
	if (!fmt) {
		ctx_dbg(3, ctx, "mbus code format (0x%08x) not found.\n",
			mbus_fmt.code);
		/* code not found, use a working default */
		fmt = find_format_by_code(MEDIA_BUS_FMT_YUYV8_2X8);
		mbus_fmt.code = fmt->code;
		mbus_fmt.colorspace = fmt->colorspace;
		mbus_fmt.width = 1920;
		mbus_fmt.height = 1080;
		mbus_fmt.field = V4L2_FIELD_NONE;
	}

	if (ctx->fmt != fmt) {
		/* looks like current format has changed, update local */
		ctx->fmt = fmt;
		ctx->width = mbus_fmt.width;
		ctx->height = mbus_fmt.height;
		ctx->field = mbus_fmt.field;
		ctx->pixelsize = ctx->fmt->depth >> 3;
	}

	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	f->fmt.pix.width        = ctx->width;
	f->fmt.pix.height       = ctx->height;
	f->fmt.pix.field        = ctx->field;
	f->fmt.pix.pixelformat  = ctx->fmt->fourcc;
	f->fmt.pix.colorspace   = ctx->fmt->colorspace;
	f->fmt.pix.bytesperline =
		(f->fmt.pix.width * ctx->fmt->depth) >> 3;
	f->fmt.pix.sizeimage =
		f->fmt.pix.height * f->fmt.pix.bytesperline;
	return 0;
}

static int cal_try_fmt_vid_cap(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct cal_ctx *ctx = video_drvdata(file);
	const struct cal_fmt *fmt;
	struct v4l2_fmtdesc fmt_desc;

	ctx_dbg(2, ctx, "%s\n", __func__);

	fmt = find_format_by_pix(f->fmt.pix.pixelformat);
	if (!fmt) {
		ctx_dbg(3, ctx, "Fourcc format (0x%08x) not found.\n",
			f->fmt.pix.pixelformat);

		/* Just get the first one enumerated */
		fmt_desc.index = 0;
		if (cal_enum_fmt_vid_cap(file, priv, &fmt_desc)) {
			ctx_dbg(3, ctx,
				"no default fmt found , this should not happen.\n");
			fmt = find_format_by_code(MEDIA_BUS_FMT_YUYV8_2X8);
		} else {
			fmt = find_format_by_pix(fmt_desc.pixelformat);
		}
		f->fmt.pix.pixelformat = fmt->fourcc;
	}

	f->fmt.pix.field = ctx->field;
	v4l_bound_align_image(&f->fmt.pix.width, 48, MAX_WIDTH, 2,
			      &f->fmt.pix.height, 32, MAX_HEIGHT, 0, 0);
	f->fmt.pix.bytesperline =
		(f->fmt.pix.width * fmt->depth) >> 3;
	f->fmt.pix.sizeimage =
		f->fmt.pix.height * f->fmt.pix.bytesperline;
	f->fmt.pix.colorspace = fmt->colorspace;
	f->fmt.pix.priv = 0;
	return 0;
}

static int cal_s_fmt_vid_cap(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct cal_ctx *ctx = video_drvdata(file);
	struct vb2_queue *q = &ctx->vb_vidq;
	const struct cal_fmt *fmt;
	struct v4l2_mbus_framefmt mbus_fmt;
	int ret;

	ctx_dbg(2, ctx, "%s\n", __func__);

	if (vb2_is_busy(q)) {
		ctx_dbg(3, ctx, "%s device busy\n", __func__);
		return -EBUSY;
	}

	ret = cal_try_fmt_vid_cap(file, priv, f);
	if (ret < 0)
		return ret;

	fmt = find_format_by_pix(f->fmt.pix.pixelformat);

	v4l2_fill_mbus_format(&mbus_fmt, &f->fmt.pix, fmt->code);

	ret = __subdev_set_format(ctx, &mbus_fmt);
	if (ret)
		return ret;

	/* Just double check nothing has gone wrong */
	if (mbus_fmt.code != fmt->code) {
		ctx_dbg(3, ctx,
			"%s subdev changed format on us, this should not happen\n",
			__func__);
		return -EINVAL;
	}

	ctx->fmt = fmt;
	ctx->pixelsize = ctx->fmt->depth >> 3;
	ctx->field = f->fmt.pix.field;
	ctx->width = f->fmt.pix.width;
	ctx->height = f->fmt.pix.height;

	return 0;
}

static int cal_enum_framesizes(struct file *file, void *fh,
			       struct v4l2_frmsizeenum *fsize)
{
	struct cal_ctx *ctx = video_drvdata(file);
	const struct cal_fmt *fmt;
	struct v4l2_subdev_frame_size_enum fse;
	int ret;

	ctx_dbg(2, ctx, "%s\n", __func__);

	/* check for valid format */
	fmt = find_format_by_pix(fsize->pixel_format);
	if (!fmt) {
		ctx_dbg(3, ctx, "Invalid pixel code: %x\n",
			fsize->pixel_format);
		return -EINVAL;
	}

	fse.index = fsize->index;
	fse.pad = 0;
	fse.code = fmt->code;

	ret = v4l2_subdev_call(ctx->sensor, pad, enum_frame_size, NULL, &fse);
	if (ret)
		return -EINVAL;

	ctx_dbg(1, ctx, "%s: index: %d code: %x W:[%d,%d] H:[%d,%d]\n",
		__func__, fse.index, fse.code, fse.min_width, fse.max_width,
		fse.min_height, fse.max_height);

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = fse.max_width;
	fsize->discrete.height = fse.max_height;

	return 0;
}

static int cal_enum_input(struct file *file, void *priv,
			  struct v4l2_input *inp)
{
	if (inp->index >= CAL_NUM_INPUT)
		return -EINVAL;

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	sprintf(inp->name, "Camera %u", inp->index);
	return 0;
}

static int cal_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct cal_ctx *ctx = video_drvdata(file);

	*i = ctx->input;
	return 0;
}

static int cal_s_input(struct file *file, void *priv, unsigned int i)
{
	struct cal_ctx *ctx = video_drvdata(file);

	if (i >= CAL_NUM_INPUT)
		return -EINVAL;

	if (i == ctx->input)
		return 0;

	ctx->input = i;
	return 0;
}

/* timeperframe is arbitrary and continuous */
static int cal_enum_frameintervals(struct file *file, void *priv,
				   struct v4l2_frmivalenum *fival)
{
	struct cal_ctx *ctx = video_drvdata(file);
	const struct cal_fmt *fmt;
	struct v4l2_subdev_frame_size_enum fse;
	int ret;

	if (fival->index)
		return -EINVAL;

	fmt = find_format_by_pix(fival->pixel_format);
	if (!fmt)
		return -EINVAL;

	/* check for valid width/height */
	ret = 0;
	fse.pad = 0;
	fse.code = fmt->code;
	fse.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	for (fse.index = 0; ; fse.index++) {
		ret = v4l2_subdev_call(ctx->sensor, pad, enum_frame_size,
				       NULL, &fse);
		if (ret)
			return -EINVAL;

		if ((fival->width == fse.max_width) &&
		    (fival->height == fse.max_height))
			break;
		else if ((fival->width >= fse.min_width) &&
			 (fival->width <= fse.max_width) &&
			 (fival->height >= fse.min_height) &&
			 (fival->height <= fse.max_height))
			break;

		return -EINVAL;
	}

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = 30;

	return 0;
}

/*
 * Videobuf operations
 */
static int cal_queue_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
			   unsigned int *nbuffers, unsigned int *nplanes,
			   unsigned int sizes[], void *alloc_ctxs[])
{
	struct cal_ctx *ctx = vb2_get_drv_priv(vq);
	unsigned long size;

	size = ctx->width * ctx->height * ctx->pixelsize;
	if (fmt) {
		if (fmt->fmt.pix.sizeimage < size)
			return -EINVAL;
		size = fmt->fmt.pix.sizeimage;
		/* check against insane over 8K resolution buffers */
		if (size > 7680 * 4320 * ctx->pixelsize)
			return -EINVAL;
	}

	*nplanes = 1;
	sizes[0] = size;
	alloc_ctxs[0] = ctx->alloc_ctx;

	ctx_dbg(3, ctx, "nbuffers=%d, size=%ld\n", *nbuffers, size);

	return 0;
}

static int cal_buffer_prepare(struct vb2_buffer *vb)
{
	struct cal_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct cal_buffer *buf = container_of(vb, struct cal_buffer, vb);
	unsigned long size;

	BUG_ON(NULL == ctx->fmt);

	size = ctx->width * ctx->height * ctx->pixelsize;
	if (vb2_plane_size(vb, 0) < size) {
		ctx_err(ctx,
			"data will not fit into plane (%lu < %lu)\n",
			vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(&buf->vb, 0, size);
	return 0;
}

static void cal_buffer_queue(struct vb2_buffer *vb)
{
	struct cal_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct cal_buffer *buf = container_of(vb, struct cal_buffer, vb);
	struct cal_dmaqueue *vidq = &ctx->vidq;
	unsigned long flags = 0;

	/* recheck locking */
	spin_lock_irqsave(&ctx->slock, flags);
	list_add_tail(&buf->list, &vidq->active);
	spin_unlock_irqrestore(&ctx->slock, flags);
}

static int cal_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct cal_ctx *ctx = vb2_get_drv_priv(vq);
	struct cal_dmaqueue *dma_q = &ctx->vidq;
	struct cal_buffer *buf;
	unsigned long addr = 0;
	unsigned long flags;
	int ret;

	ctx_dbg(3, ctx, "%s\n", __func__);

	spin_lock_irqsave(&ctx->slock, flags);
	if (list_empty(&dma_q->active)) {
		spin_unlock_irqrestore(&ctx->slock, flags);
		ctx_dbg(3, ctx, "buffer queue is empty\n");
		return -EIO;
	}

	buf = list_entry(dma_q->active.next, struct cal_buffer, list);
	ctx->cur_frm = buf;
	ctx->next_frm = buf;
	list_del(&buf->list);
	spin_unlock_irqrestore(&ctx->slock, flags);

	v4l2_get_timestamp(&buf->vb.v4l2_buf.timestamp);

	addr = vb2_dma_contig_plane_dma_addr(&ctx->cur_frm->vb, 0);
	ctx->sequence = 0;

	ctx_dbg(3, ctx, "enable_irqs\n");

	ret = cal_get_external_info(ctx);
	if (ret < 0)
		return ret;

	cal_runtime_get(ctx->dev);

	enable_irqs(ctx);
	camerarx_phy_enable(ctx);
	csi2_init(ctx);
	csi2_phy_config(ctx);
	csi2_lane_config(ctx);
	csi2_ctx_config(ctx);
	pix_proc_config(ctx);
	cal_wr_dma_config(ctx, ALIGN((ctx->width * ctx->pixelsize), 16));
	cal_wr_dma_addr(ctx, addr);
	csi2_ppi_enable(ctx);

	if (ctx->sensor) {
		if (v4l2_subdev_call(ctx->sensor, video, s_stream, 1)) {
			ctx_err(ctx, "stream on failed in subdev\n");
			cal_runtime_put(ctx->dev);
			return -EINVAL;
		}
	}

	if (debug >= 4)
		cal_quickdump_regs(ctx->dev);

	ctx_dbg(3, ctx, "returning from %s\n", __func__);
	return 0;
}

static void cal_stop_streaming(struct vb2_queue *vq)
{
	struct cal_ctx *ctx = vb2_get_drv_priv(vq);
	struct cal_dmaqueue *dma_q = &ctx->vidq;
	unsigned long flags;

	ctx_dbg(3, ctx, "%s\n", __func__);

	if (ctx->sensor) {
		if (v4l2_subdev_call(ctx->sensor, video, s_stream, 0))
			ctx_err(ctx, "stream off failed in subdev\n");
	}

	ctx_dbg(3, ctx, "csi2_ppi_disable\n");
	csi2_ppi_disable(ctx);

	ctx_dbg(3, ctx, "disable_irqs\n");
	disable_irqs(ctx);

	/* Release all active buffers */
	spin_lock_irqsave(&ctx->slock, flags);
	while (!list_empty(&dma_q->active)) {
		struct cal_buffer *buf;

		buf = list_entry(dma_q->active.next, struct cal_buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
		ctx_dbg(3, ctx, "[%p/%d] done\n", buf, buf->vb.v4l2_buf.index);
	}
	spin_unlock_irqrestore(&ctx->slock, flags);

	if (ctx->cur_frm == ctx->next_frm) {
		vb2_buffer_done(&ctx->cur_frm->vb, VB2_BUF_STATE_ERROR);
		ctx_dbg(3, ctx, "[%p/%d] done cur_frm\n", ctx->cur_frm,
			ctx->cur_frm->vb.v4l2_buf.index);
	} else {
		vb2_buffer_done(&ctx->cur_frm->vb, VB2_BUF_STATE_ERROR);
		ctx_dbg(3, ctx, "[%p/%d] done cur_frm\n", ctx->cur_frm,
			ctx->cur_frm->vb.v4l2_buf.index);
		vb2_buffer_done(&ctx->next_frm->vb, VB2_BUF_STATE_ERROR);
		ctx_dbg(3, ctx, "[%p/%d] done next_frm\n", ctx->next_frm,
			ctx->next_frm->vb.v4l2_buf.index);
	}
	ctx->cur_frm = NULL;
	ctx->next_frm = NULL;

	cal_runtime_put(ctx->dev);

	ctx_dbg(3, ctx, "returning from %s\n", __func__);
}

static struct vb2_ops cal_video_qops = {
	.queue_setup		= cal_queue_setup,
	.buf_prepare		= cal_buffer_prepare,
	.buf_queue		= cal_buffer_queue,
	.start_streaming	= cal_start_streaming,
	.stop_streaming		= cal_stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static const struct v4l2_file_operations cal_fops = {
	.owner		= THIS_MODULE,
	.open           = v4l2_fh_open,
	.release        = vb2_fop_release,
	.read           = vb2_fop_read,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2, /* V4L2 ioctl handler */
	.mmap           = vb2_fop_mmap,
};

static const struct v4l2_ioctl_ops cal_ioctl_ops = {
	.vidioc_querycap      = cal_querycap,
	.vidioc_enum_fmt_vid_cap  = cal_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap     = cal_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap   = cal_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap     = cal_s_fmt_vid_cap,
	.vidioc_enum_framesizes   = cal_enum_framesizes,
	.vidioc_reqbufs       = vb2_ioctl_reqbufs,
	.vidioc_create_bufs   = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf   = vb2_ioctl_prepare_buf,
	.vidioc_querybuf      = vb2_ioctl_querybuf,
	.vidioc_qbuf          = vb2_ioctl_qbuf,
	.vidioc_dqbuf         = vb2_ioctl_dqbuf,
	.vidioc_expbuf        = vb2_ioctl_expbuf,
	.vidioc_enum_input    = cal_enum_input,
	.vidioc_g_input       = cal_g_input,
	.vidioc_s_input       = cal_s_input,
	.vidioc_enum_frameintervals = cal_enum_frameintervals,
	.vidioc_streamon      = vb2_ioctl_streamon,
	.vidioc_streamoff     = vb2_ioctl_streamoff,
	.vidioc_log_status    = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static struct video_device cal_videodev = {
	.name		= CAL_MODULE_NAME,
	.fops		= &cal_fops,
	.ioctl_ops	= &cal_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,
};

/* -----------------------------------------------------------------
	Initialization and module stuff
   ------------------------------------------------------------------*/
static int cal_release(struct cal_dev *dev)
{
	struct cal_ctx *ctx;
	int i;

	for (i = 0; i < CAL_NUM_CONTEXT; i++) {
		ctx = dev->ctx[i];
		if (ctx) {
			v4l2_info(&ctx->v4l2_dev, "unregistering %s\n",
				  video_device_node_name(&ctx->vdev));
			video_unregister_device(&ctx->vdev);
			v4l2_device_unregister(&ctx->v4l2_dev);
			vb2_dma_contig_cleanup_ctx(ctx->alloc_ctx);
			v4l2_ctrl_handler_free(&ctx->ctrl_handler);
			kfree(ctx->cc);
			kfree(ctx);
		}
	}

	return 0;
}

static int cal_complete_ctx(struct cal_ctx *ctx);

static int cal_async_bound(struct v4l2_async_notifier *notifier,
			   struct v4l2_subdev *subdev,
			   struct v4l2_async_subdev *asd)
{
	struct cal_ctx *ctx = notifier_to_ctx(notifier);
	struct v4l2_subdev_mbus_code_enum mbus_code;
	int i, j;

	ctx_dbg(1, ctx, "cal_async_bound\n");

	if (ctx->sensor) {
		ctx_info(ctx, "Rejecting subdev %s (Already set!!)",
			 subdev->name);
		return 0;
	}

	ctx->sensor = subdev;
	ctx_info(ctx, "Using sensor %s for capture\n",
		 subdev->name);

	/* setup the supported formats & indexes */
	for (j = 0, i = 0; ; ++j) {
		struct cal_fmt *fmt;
		int ret;

		memset(&mbus_code, 0, sizeof(mbus_code));
		mbus_code.index = j;
		ret = v4l2_subdev_call(subdev, pad, enum_mbus_code,
				       NULL, &mbus_code);
		if (ret)
			break;

		fmt = (struct cal_fmt *)find_format_by_code(mbus_code.code);
		if (!fmt)
			continue;

		fmt->supported = true;
		fmt->index = i++;
	}

	cal_complete_ctx(ctx);

	return 0;
}

static int cal_async_complete(struct v4l2_async_notifier *notifier)
{
	struct cal_ctx *ctx = notifier_to_ctx(notifier);

	ctx_dbg(1, ctx, "cal_async_complete\n");
	return 0;
}

static int cal_complete_ctx(struct cal_ctx *ctx)
{
	struct video_device *vfd;
	struct vb2_queue *q;
	int ret;

	ctx->timeperframe = tpf_default;

	ctx->width = 1920;
	ctx->height = 1080;
	ctx->field = V4L2_FIELD_NONE;
	ctx->fmt = find_format_by_code(MEDIA_BUS_FMT_SGRBG8_1X8);
	ctx->pixelsize = ctx->fmt->depth >> 3;
	ctx->external_rate = 192000000;

	/* initialize locks */
	spin_lock_init(&ctx->slock);
	mutex_init(&ctx->mutex);

	/* initialize queue */
	q = &ctx->vb_vidq;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	q->drv_priv = ctx;
	q->buf_struct_size = sizeof(struct cal_buffer);
	q->ops = &cal_video_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &ctx->mutex;
	q->min_buffers_needed = 3;

	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	/* init video dma queues */
	INIT_LIST_HEAD(&ctx->vidq.active);

	vfd = &ctx->vdev;
	*vfd = cal_videodev;
	vfd->v4l2_dev = &ctx->v4l2_dev;
	vfd->queue = q;

	/*
	 * Provide a mutex to v4l2 core. It will be used to protect
	 * all fops and v4l2 ioctls.
	 */
	vfd->lock = &ctx->mutex;
	video_set_drvdata(vfd, ctx);

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, video_nr);
	if (ret < 0)
		return ret;

	v4l2_info(&ctx->v4l2_dev, "V4L2 device registered as %s\n",
		  video_device_node_name(vfd));

	ctx->alloc_ctx = vb2_dma_contig_init_ctx(vfd->v4l2_dev->dev);
	if (IS_ERR(ctx->alloc_ctx)) {
		ctx_err(ctx, "Failed to alloc vb2 context\n");
		ret = PTR_ERR(ctx->alloc_ctx);
		goto vdev_unreg;
	}

	return 0;

vdev_unreg:
	video_unregister_device(vfd);
	return ret;
}

static struct device_node *
of_get_next_port(const struct device_node *parent,
		 struct device_node *prev)
{
	struct device_node *port = NULL;

	if (!parent)
		return NULL;

	if (!prev) {
		struct device_node *ports;
		/*
		 * It's the first call, we have to find a port subnode
		 * within this node or within an optional 'ports' node.
		 */
		ports = of_get_child_by_name(parent, "ports");
		if (ports)
			parent = ports;

		port = of_get_child_by_name(parent, "port");

		/* release the 'ports' node */
		of_node_put(ports);
	} else {
		struct device_node *ports;

		ports = of_get_parent(prev);
		if (!ports)
			return NULL;

		do {
			port = of_get_next_child(ports, prev);
			if (!port) {
				of_node_put(ports);
				return NULL;
			}
			prev = port;
		} while (of_node_cmp(port->name, "port") != 0);
	}

	return port;
}

static struct device_node *
of_get_next_endpoint(const struct device_node *parent,
		     struct device_node *prev)
{
	struct device_node *ep = NULL;

	if (!parent)
		return NULL;

	do {
		ep = of_get_next_child(parent, prev);
		if (!ep)
			return NULL;
		prev = ep;
	} while (of_node_cmp(ep->name, "endpoint") != 0);

	return ep;
}

static int of_cal_create_instance(struct cal_ctx *ctx, int inst)
{
	struct platform_device *pdev = ctx->dev->pdev;
	struct device_node *ep_node, *port, *remote_ep,
			*sensor_node, *parent;
	struct v4l2_of_endpoint *endpoint;
	struct v4l2_async_subdev *asd;
	u32 regval = 0;
	int ret, index, found_port = 0, lane;

	parent = pdev->dev.of_node;

	asd = &ctx->asd;
	endpoint = &ctx->endpoint;

	ep_node = NULL;
	port = NULL;
	remote_ep = NULL;
	sensor_node = NULL;
	ret = -EINVAL;

	ctx_dbg(3, ctx, "Scanning Port node for csi2 port: %d\n", inst);
	for (index = 0; index < CAL_NUM_CSI2_PORTS; index++) {
		port = of_get_next_port(parent, port);
		if (!port) {
			ctx_dbg(1, ctx, "No port node found for csi2 port:%d\n",
				index);
			goto cleanup_exit;
		}

		/* Match the slice number with <REG> */
		of_property_read_u32(port, "reg", &regval);
		ctx_dbg(3, ctx, "port:%d inst:%d <reg>:%d\n",
			index, inst, regval);
		if ((regval == inst) && (index == inst)) {
			found_port = 1;
			break;
		}
	}

	if (!found_port) {
		ctx_dbg(1, ctx, "No port node matches csi2 port:%d\n",
			inst);
		goto cleanup_exit;
	}

	ctx_dbg(3, ctx, "Scanning sub-device for csi2 port: %d\n",
		inst);

	ep_node = of_get_next_endpoint(port, ep_node);
	if (!ep_node) {
		ctx_dbg(3, ctx, "can't get next endpoint\n");
		goto cleanup_exit;
	}

	sensor_node = of_graph_get_remote_port_parent(ep_node);
	if (!sensor_node) {
		ctx_dbg(3, ctx, "can't get remote parent\n");
		goto cleanup_exit;
	}
	asd->match_type = V4L2_ASYNC_MATCH_OF;
	asd->match.of.node = sensor_node;

	remote_ep = of_parse_phandle(ep_node, "remote-endpoint", 0);
	if (!remote_ep) {
		ctx_dbg(3, ctx, "can't get remote-endpoint\n");
		goto cleanup_exit;
	}
	v4l2_of_parse_endpoint(remote_ep, endpoint);

	if (endpoint->bus_type != V4L2_MBUS_CSI2) {
		ctx_err(ctx, "Port:%d sub-device %s is not a CSI2 device\n",
			inst, sensor_node->name);
		goto cleanup_exit;
	}

	/* Store Virtual Channel number */
	ctx->virtual_channel = endpoint->base.id;

	ctx_dbg(3, ctx, "Port:%d v4l2-endpoint: CSI2\n", inst);
	ctx_dbg(3, ctx, "Virtual Channel=%d\n", ctx->virtual_channel);
	ctx_dbg(3, ctx, "flags=0x%08x\n", endpoint->bus.mipi_csi2.flags);
	ctx_dbg(3, ctx, "clock_lane=%d\n", endpoint->bus.mipi_csi2.clock_lane);
	ctx_dbg(3, ctx, "num_data_lanes=%d\n",
		endpoint->bus.mipi_csi2.num_data_lanes);
	ctx_dbg(3, ctx, "data_lanes= <\n");
	for (lane = 0; lane < endpoint->bus.mipi_csi2.num_data_lanes; lane++)
		ctx_dbg(3, ctx, "\t%d\n",
			endpoint->bus.mipi_csi2.data_lanes[lane]);
	ctx_dbg(3, ctx, "\t>\n");

	ctx_dbg(1, ctx, "Port: %d found sub-device %s\n",
		inst, sensor_node->name);

	ctx_dbg(1, ctx, "Asynchronous subdevice registration\n");
	ctx->asd_list[0] = asd;
	ctx->notifier.subdevs = ctx->asd_list;
	ctx->notifier.num_subdevs = 1;
	ctx->notifier.bound = cal_async_bound;
	ctx->notifier.complete = cal_async_complete;
	ret = v4l2_async_notifier_register(&ctx->v4l2_dev,
					   &ctx->notifier);
	if (ret) {
		ctx_err(ctx, "Error registering async notifier\n");
		ret = -EINVAL;
	}

cleanup_exit:
	if (!remote_ep)
		of_node_put(remote_ep);
	if (!sensor_node)
		of_node_put(sensor_node);
	if (!ep_node)
		of_node_put(ep_node);
	if (!port)
		of_node_put(port);

	return ret;
}

static struct cal_ctx *cal_create_instance(struct cal_dev *dev, int inst)
{
	struct cal_ctx *ctx;
	struct v4l2_ctrl_handler *hdl;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return 0;
	/* save the cal_dev * for future ref */
	ctx->dev = dev;

	snprintf(ctx->v4l2_dev.name, sizeof(ctx->v4l2_dev.name),
		 "%s-%03d", CAL_MODULE_NAME, inst);
	ret = v4l2_device_register(&dev->pdev->dev, &ctx->v4l2_dev);
	if (ret)
		goto free_ctx;

	hdl = &ctx->ctrl_handler;
	ret = v4l2_ctrl_handler_init(hdl, 11);
	if (ret) {
		ctx_err(ctx, "Failed to init ctrl handler\n");
		goto unreg_dev;
	}
	ctx->v4l2_dev.ctrl_handler = hdl;

	/* Make sure Camera Core H/W register area is available */
	ctx->cc = dev->cc[inst];

	/* Store the instance id */
	ctx->csi2_port = inst + 1;

	ret = of_cal_create_instance(ctx, inst);
	if (ret) {
		ctx_dbg(1, ctx, "Error scanning cal instance: %d\n", inst);
		ret = -EINVAL;
		goto free_hdl;
	}
	return ctx;

free_hdl:
	v4l2_ctrl_handler_free(hdl);
unreg_dev:
	v4l2_device_unregister(&ctx->v4l2_dev);
free_ctx:
	kfree(ctx);
	return 0;
}

static const struct of_device_id cal_of_match[];

static int cal_probe(struct platform_device *pdev)
{
	struct cal_dev *dev;
	const struct of_device_id *match;
	const struct cal_of_data *data;
	int ret;
	int irq;

	dev_info(&pdev->dev, "Probing %s\n",
		 CAL_MODULE_NAME);

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	match = of_match_device(of_match_ptr(cal_of_match), &pdev->dev);
	if (match) {
		if (match->data) {
			data = match->data;
			dev->flags = data->flags;
		}
	}

	/* set pseudo v4l2 device name so we can use v4l2_printk */
	strcpy(dev->v4l2_dev.name, CAL_MODULE_NAME);

	/* save pdev pointer */
	dev->pdev = pdev;

	dev->res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			"cal_top");
	cal_dbg(1, dev, "ioresource %s at  %pa - %pa\n",
		dev->res->name, &dev->res->start, &dev->res->end);

	dev->base = devm_ioremap(&pdev->dev, dev->res->start, SZ_32K);
	if (!dev->base) {
		ret = -ENOMEM;
		goto just_exit;
	}

	irq = platform_get_irq(pdev, 0);
	cal_dbg(1, dev, "got irq# %d\n", irq);
	ret = devm_request_irq(&pdev->dev, irq, cal_irq, 0, CAL_MODULE_NAME,
			       dev);
	if (ret)
		goto just_exit;

	platform_set_drvdata(pdev, dev);

	dev->cm = cm_create(dev);
	if (IS_ERR(dev->cm)) {
		ret = PTR_ERR(dev->cm);
		goto just_exit;
	}

	dev->cc[0] = cc_create(dev, 0);
	if (IS_ERR(dev->cc[0])) {
		ret = PTR_ERR(dev->cc[0]);
		goto free_cm;
	}

	dev->cc[1] = cc_create(dev, 1);
	if (IS_ERR(dev->cc[1])) {
		ret = PTR_ERR(dev->cc[1]);
		goto free_cc0;
	}

	dev->ctx[0] = NULL;
	dev->ctx[1] = NULL;

	dev->ctx[0] = cal_create_instance(dev, 0);
	dev->ctx[1] = cal_create_instance(dev, 1);
	if (!dev->ctx[0] && !dev->ctx[1]) {
		ret = -ENODEV;
		cal_err(dev, "Neither port is configured, no point in staying up\n");
		goto free_ctx;
	}

	pm_runtime_enable(&pdev->dev);

	ret = cal_runtime_get(dev);
	if (ret)
		goto runtime_put;

	/* Just check we can actually access the module */
	cal_get_hwinfo(dev);

	cal_runtime_put(dev);

	return 0;

runtime_put:
	pm_runtime_disable(&pdev->dev);
free_ctx:
	kfree(dev->ctx[0]);
	kfree(dev->ctx[1]);
free_cc0:
	kfree(dev->cc[0]);
free_cm:
	kfree(dev->cm);
just_exit:
	return ret;
}

static int cal_remove(struct platform_device *pdev)
{
	struct cal_dev *dev =
		(struct cal_dev *)platform_get_drvdata(pdev);

	cal_info(dev, "Removing %s\n", CAL_MODULE_NAME);

	cal_runtime_get(dev);

	cal_release(dev);

	/* disable csi2 phy */
	if (dev->ctx[0])
		camerarx_phy_disable(dev->ctx[0]);
	if (dev->ctx[1])
		camerarx_phy_disable(dev->ctx[1]);
	kfree(dev->cm);

	cal_runtime_put(dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

#if defined(CONFIG_OF)
static const struct cal_of_data dra72_pre_es2_cal_of_data = {
	/*
	 * See DRA72x Errata: i913: CSI2 LDO Needs to be disabled
	 * when module is powered on.
	 */
	.flags = DRA72_CAL_PRE_ES2_LDO_DISABLE,
};

static const struct of_device_id cal_of_match[] = {
	{
		.compatible = "ti,dra72-cal",
	},
	{
		.compatible = "ti,dra72-pre-es2-cal",
		.data = &dra72_pre_es2_cal_of_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, cal_of_match);
#endif

static struct platform_driver cal_pdrv = {
	.probe		= cal_probe,
	.remove		= cal_remove,
	.driver		= {
		.name	= CAL_MODULE_NAME,
		.of_match_table = of_match_ptr(cal_of_match),
	},
};

module_platform_driver(cal_pdrv);
