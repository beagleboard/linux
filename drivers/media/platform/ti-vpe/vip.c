/*
 * TI VIP capture driver
 *
 * Copyright (C) 2013 - 2014 Texas Instruments, Inc.
 * David Griego, <dagriego@biglakesoftware.com>
 * Dale Farnsworth, <dale@farnsworth.org>
 * Nikhil Devshatwar, <nikhil.nd@ti.com>
 * Benoit Parrot, <bparrot@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include <linux/pinctrl/consumer.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>

#include <media/v4l2-of.h>
#include <media/v4l2-async.h>

#include "vip.h"

#define VIP_MODULE_NAME "vip"
#define VIP_INPUT_NAME "Vin0"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-8)");

/*
 * Minimum and maximum frame sizes
 */
#define MIN_W		128
#define MIN_H		128
#define MAX_W		2048
#define MAX_H		1536

/*
 * Required alignments
 */
#define S_ALIGN		0 /* multiple of 1 */
#define H_ALIGN		1 /* multiple of 2 */
#define W_ALIGN		1 /* multiple of 2 */
#define L_ALIGN		7 /* multiple of 128, line stride, 16 bytes */

/*
 * Need a descriptor entry for each of up to 15 outputs,
 * and up to 2 control transfers.
 */
#define VIP_DESC_LIST_SIZE	(17 * sizeof(struct vpdma_dtd))

#define vip_dbg(level, dev, fmt, arg...)	\
		v4l2_dbg(level, debug, &dev->v4l2_dev, fmt, ##arg)
#define vip_err(dev, fmt, arg...)	\
		v4l2_err(&dev->v4l2_dev, fmt, ##arg)
#define vip_info(dev, fmt, arg...)	\
		v4l2_info(&dev->v4l2_dev, fmt, ##arg)

#define CTRL_CORE_SMA_SW_1      0x534
/*
 * The srce_info structure contains per-srce data.
 */
struct vip_srce_info {
	u8	base_channel;	/* the VPDMA channel nummber */
	u8	vb_index;	/* input frame f, f-1, f-2 index */
	u8	vb_part;	/* identifies section of co-planar formats */
};

#define VIP_VPDMA_FIFO_SIZE	2
#define VIP_DROPQ_SIZE		3

/*
 * Define indices into the srce_info tables
 */

#define VIP_SRCE_MULT_PORT		0
#define VIP_SRCE_MULT_ANC		1
#define VIP_SRCE_LUMA		2
#define VIP_SRCE_CHROMA		3
#define VIP_SRCE_RGB		4

static struct vip_srce_info srce_info[5] = {
	[VIP_SRCE_MULT_PORT] = {
		.base_channel	= VIP1_CHAN_NUM_MULT_PORT_A_SRC0,
		.vb_index	= 0,
		.vb_part	= VIP_CHROMA,
	},
	[VIP_SRCE_MULT_ANC] = {
		.base_channel	= VIP1_CHAN_NUM_MULT_ANC_A_SRC0,
		.vb_index	= 0,
		.vb_part	= VIP_LUMA,
	},
	[VIP_SRCE_LUMA] = {
		.base_channel	= VIP1_CHAN_NUM_PORT_B_LUMA,
		.vb_index	= 1,
		.vb_part	= VIP_LUMA,
	},
	[VIP_SRCE_CHROMA] = {
		.base_channel	= VIP1_CHAN_NUM_PORT_B_CHROMA,
		.vb_index	= 1,
		.vb_part	= VIP_CHROMA,
	},
	[VIP_SRCE_RGB] = {
		.base_channel	= VIP1_CHAN_NUM_PORT_B_RGB,
		.vb_part	= VIP_LUMA,
	},
};

static struct vip_fmt vip_formats[] = {
	{
		.name		= "NV12 YUV 420 co-planar",
		.fourcc		= V4L2_PIX_FMT_NV12,
		.code		= MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
		.coplanar	= 1,
		.vpdma_fmt	= { &vpdma_yuv_fmts[VPDMA_DATA_FMT_Y420],
				    &vpdma_yuv_fmts[VPDMA_DATA_FMT_C420],
				  },
	},
	{
		.name		= "UYVY 422 packed",
		.fourcc		= V4L2_PIX_FMT_UYVY,
		.code		= MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
		.coplanar	= 0,
		.vpdma_fmt	= { &vpdma_yuv_fmts[VPDMA_DATA_FMT_CBY422],
				  },
	},
	{
		.name		= "YUYV 422 packed",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.code		= MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
		.coplanar	= 0,
		.vpdma_fmt	= { &vpdma_yuv_fmts[VPDMA_DATA_FMT_YCB422],
				  },
	},
	{
		.name		= "VYUY 422 packed",
		.fourcc		= V4L2_PIX_FMT_VYUY,
		.code		= MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
		.coplanar	= 0,
		.vpdma_fmt	= { &vpdma_yuv_fmts[VPDMA_DATA_FMT_CRY422],
				  },
	},
	{
		.name		= "YVYU 422 packed",
		.fourcc		= V4L2_PIX_FMT_YVYU,
		.code		= MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
		.coplanar	= 0,
		.vpdma_fmt	= { &vpdma_yuv_fmts[VPDMA_DATA_FMT_YCR422],
				  },
	},
	{
		.name		= "RGB888 packed",
		.fourcc		= V4L2_PIX_FMT_RGB24,
		.code		= MEDIA_BUS_FMT_RGB888_1X24,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.coplanar	= 0,
		.vpdma_fmt	= { &vpdma_rgb_fmts[VPDMA_DATA_FMT_RGB24],
				  },
	},
	{
		.name		= "ARGB888 packed",
		.fourcc		= V4L2_PIX_FMT_RGB32,
		.code		= MEDIA_BUS_FMT_ARGB8888_1X32,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.coplanar	= 0,
		.vpdma_fmt	= { &vpdma_rgb_fmts[VPDMA_DATA_FMT_ARGB32],
				  },
	},
};

/*  Print Four-character-code (FOURCC) */
static char *fourcc_to_str(u32 fmt)
{
	static char code[5];

	code[0] = (unsigned char)(fmt & 0xff);
	code[1] = (unsigned char)((fmt >> 8) & 0xff);
	code[2] = (unsigned char)((fmt >> 16) & 0xff);
	code[3] = (unsigned char)((fmt >> 24) & 0xff);
	code[4] = '\0';

	return code;
}

/*
 * Find our format description corresponding to the passed v4l2_format
 */

static struct vip_fmt *find_port_format_by_pix(struct vip_port *port,
					       u32 pixelformat)
{
	struct vip_fmt *fmt;
	unsigned int k;

	for (k = 0; k < port->num_active_fmt; k++) {
		fmt = port->active_fmt[k];
		if (fmt->fourcc == pixelformat)
			return fmt;
	}

	return NULL;
}

static struct vip_fmt *find_port_format_by_code(struct vip_port *port,
						u32 code)
{
	struct vip_fmt *fmt;
	unsigned int k;

	for (k = 0; k < port->num_active_fmt; k++) {
		fmt = port->active_fmt[k];
		if (fmt->code == code)
			return fmt;
	}

	return NULL;
}

inline struct vip_port *notifier_to_vip_port(struct v4l2_async_notifier *n)
{
	return container_of(n, struct vip_port, notifier);
}

/*
 * port flag bits
 */
#define FLAG_FRAME_1D		BIT(0)
#define FLAG_EVEN_LINE_SKIP	BIT(1)
#define FLAG_ODD_LINE_SKIP	BIT(2)
#define FLAG_MODE_TILED		BIT(3)
#define FLAG_INTERLACED		BIT(4)
#define FLAG_MULTIPLEXED	BIT(5)
#define FLAG_MULT_PORT		BIT(6)
#define FLAG_MULT_ANC		BIT(7)

/*
 * Function prototype declarations
 */
static int alloc_port(struct vip_dev *, int);
static void free_port(struct vip_port *);
static int vip_setup_parser(struct vip_port *port);
static void stop_dma(struct vip_stream *stream);

static inline u32 read_sreg(struct vip_shared *shared, int offset)
{
	return ioread32(shared->base + offset);
}

static inline void write_sreg(struct vip_shared *shared, int offset, u32 value)
{
	iowrite32(value, shared->base + offset);
}

static inline u32 read_vreg(struct vip_dev *dev, int offset)
{
	return ioread32(dev->base + offset);
}

static inline void write_vreg(struct vip_dev *dev, int offset, u32 value)
{
	iowrite32(value, dev->base + offset);
}

/*
 * Insert a masked field into a 32-bit field
 */
static void insert_field(u32 *valp, u32 field, u32 mask, int shift)
{
	u32 val = *valp;

	val &= ~(mask << shift);
	val |= (field & mask) << shift;
	*valp = val;
}

/*
 * Enable or disable the VIP clocks
 */
static void vip_set_clock_enable(struct vip_dev *dev, bool on)
{
	u32 val = 0;

	val = read_vreg(dev, VIP_CLK_ENABLE);
	if (on) {
		val |= VIP_VPDMA_CLK_ENABLE;
		if (dev->slice_id == VIP_SLICE1)
			val |= VIP_VIP1_DATA_PATH_CLK_ENABLE;
		else
			val |= VIP_VIP2_DATA_PATH_CLK_ENABLE;
	} else {
		if (dev->slice_id == VIP_SLICE1)
			val &= ~VIP_VIP1_DATA_PATH_CLK_ENABLE;
		else
			val &= ~VIP_VIP2_DATA_PATH_CLK_ENABLE;

		/* Both VIP are disabled then shutdown VPDMA also */
		if (!(val & (VIP_VIP1_DATA_PATH_CLK_ENABLE |
			     VIP_VIP2_DATA_PATH_CLK_ENABLE)))
			val = 0;
	}

	write_vreg(dev, VIP_CLK_ENABLE, val);
}

/* This helper function is used to enable the clock early on to
 * enable vpdma firmware loading before the slice device are created
 */
static void vip_shared_set_clock_enable(struct vip_shared *shared, bool on)
{
	u32 val = 0;

	if (on)
		val = VIP_VIP1_DATA_PATH_CLK_ENABLE | VIP_VPDMA_CLK_ENABLE;

	write_sreg(shared, VIP_CLK_ENABLE, val);
}

static void vip_top_reset(struct vip_dev *dev)
{
	u32 val = 0;

	val = read_vreg(dev, VIP_CLK_RESET);

	if (dev->slice_id == VIP_SLICE1)
		insert_field(&val, 1, VIP_DATA_PATH_CLK_RESET_MASK,
			     VIP_VIP1_DATA_PATH_RESET_SHIFT);
	else
		insert_field(&val, 1, VIP_DATA_PATH_CLK_RESET_MASK,
			     VIP_VIP2_DATA_PATH_RESET_SHIFT);

	write_vreg(dev, VIP_CLK_RESET, val);

	usleep_range(200, 250);

	val = read_vreg(dev, VIP_CLK_RESET);

	if (dev->slice_id == VIP_SLICE1)
		insert_field(&val, 0, VIP_DATA_PATH_CLK_RESET_MASK,
			     VIP_VIP1_DATA_PATH_RESET_SHIFT);
	else
		insert_field(&val, 0, VIP_DATA_PATH_CLK_RESET_MASK,
			     VIP_VIP2_DATA_PATH_RESET_SHIFT);
	write_vreg(dev, VIP_CLK_RESET, val);
}

static void vip_top_vpdma_reset(struct vip_shared *shared)
{
	u32 val;

	val = read_sreg(shared, VIP_CLK_RESET);
	insert_field(&val, 1, VIP_VPDMA_CLK_RESET_MASK,
		     VIP_VPDMA_CLK_RESET_SHIFT);
	write_sreg(shared, VIP_CLK_RESET, val);

	usleep_range(200, 250);

	val = read_sreg(shared, VIP_CLK_RESET);
	insert_field(&val, 0, VIP_VPDMA_CLK_RESET_MASK,
		     VIP_VPDMA_CLK_RESET_SHIFT);
	write_sreg(shared, VIP_CLK_RESET, val);
}

static void vip_set_pclk_invert(struct vip_port *port)
{
	u32 offset;
	/*
	 * When the VIP parser is configured to so that the pixel clock
	 * is to be sampled at falling edge, the pixel clock needs to be
	 * inverted before it is given to the VIP module. This is done
	 * by setting a bit in the CTRL_CORE_SMA_SW1 register.
	 */

	if (port->dev->instance_id == VIP_INSTANCE1)
		offset = 0 + 2 * port->port_id + port->dev->slice_id;
	else if (port->dev->instance_id == VIP_INSTANCE2)
		offset = 4 + 2 * port->port_id + port->dev->slice_id;
	else if (port->dev->instance_id == VIP_INSTANCE3)
		offset = 10 - port->dev->slice_id;
	else
		BUG();

	regmap_update_bits(port->dev->syscon, CTRL_CORE_SMA_SW_1,
				1 << offset, 1 << offset);
}

static inline uint32_t vip_parser_field(struct vip_port *port, int offset)
{
	uint32_t reg = offset;

	if (port->dev->slice_id == VIP_SLICE1)
		reg += VIP1_PARSER_REG_OFFSET;
	else
		reg += VIP2_PARSER_REG_OFFSET;

	if (offset == 0) {
		if (port->port_id == VIP_PORTA)
			reg += VIP_PARSER_PORTA_0;
		else
			reg += VIP_PARSER_PORTB_0;
	} else if (offset == 1) {
		if (port->port_id == VIP_PORTA)
			reg += VIP_PARSER_PORTA_1;
		else
			reg += VIP_PARSER_PORTB_1;
	}

	return reg;
}

static void vip_set_data_interface(struct vip_port *port,
				   enum data_interface_modes mode)
{
	u32 val = 0;

	if (port->dev->slice_id == VIP_SLICE1) {
		insert_field(&val, mode, VIP_DATA_INTERFACE_MODE_MASK,
			     VIP_DATA_INTERFACE_MODE_SHFT);

		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET, val);
	} else if (port->dev->slice_id == VIP_SLICE2) {
		insert_field(&val, mode, VIP_DATA_INTERFACE_MODE_MASK,
			     VIP_DATA_INTERFACE_MODE_SHFT);

		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET, val);
	}
}

static void vip_set_slice_path(struct vip_dev *dev,
			       enum data_path_select data_path)
{
	u32 val = 0;

	switch (data_path) {
	case VIP_MULTI_CHANNEL_DATA_SELECT:
		if (dev->slice_id == VIP_SLICE1) {
			val |= VIP_MULTI_CHANNEL_SELECT;
			insert_field(&val, data_path, VIP_DATAPATH_SELECT_MASK,
				     VIP_DATAPATH_SELECT_SHFT);

			write_vreg(dev, VIP_VIP1_DATA_PATH_SELECT, val);
		} else if (dev->slice_id == VIP_SLICE2) {
			val |= VIP_MULTI_CHANNEL_SELECT;
			insert_field(&val, data_path, VIP_DATAPATH_SELECT_MASK,
				     VIP_DATAPATH_SELECT_SHFT);

			write_vreg(dev, VIP_VIP2_DATA_PATH_SELECT, val);
		}
		break;
	case VIP_RGB_OUT_LO_DATA_SELECT:
		if (dev->slice_id == VIP_SLICE1) {
			val |= VIP_RGB_OUT_LO_SRC_SELECT;
			insert_field(&val, data_path, VIP_DATAPATH_SELECT_MASK,
				     VIP_DATAPATH_SELECT_SHFT);

			write_vreg(dev, VIP_VIP1_DATA_PATH_SELECT, val);
		} else if (dev->slice_id == VIP_SLICE2) {
			val |= VIP_RGB_OUT_LO_SRC_SELECT;
			insert_field(&val, data_path, VIP_DATAPATH_SELECT_MASK,
				     VIP_DATAPATH_SELECT_SHFT);

			write_vreg(dev, VIP_VIP2_DATA_PATH_SELECT, val);
		}
		break;
	default:
		BUG();
	}
}

/*
 * Return the vip_stream structure for a given struct file
 */
static inline struct vip_stream *file2stream(struct file *file)
{
	return video_drvdata(file);
}

/*
 * Append a destination descriptor to the current descriptor list,
 * setting up dma to the given srce.
 */
static int add_out_dtd(struct vip_stream *stream, int srce_type)
{
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct vip_srce_info *sinfo = &srce_info[srce_type];
	struct v4l2_rect *c_rect = &port->c_rect;
	struct vip_fmt *fmt = port->fmt;
	int channel, plane = 0;
	int max_width, max_height;
	dma_addr_t dma_addr;
	u32 flags;

	channel = sinfo->base_channel;

	switch (srce_type) {
	case VIP_SRCE_MULT_PORT:
	case VIP_SRCE_MULT_ANC:
		if (port->port_id == VIP_PORTB)
			channel += VIP_CHAN_MULT_PORTB_OFFSET;
		channel += stream->stream_id;
		flags = 0;
		break;
	case VIP_SRCE_CHROMA:
		plane = 1;
	case VIP_SRCE_LUMA:
		if (port->port_id == VIP_PORTB)
			channel += VIP_CHAN_YUV_PORTB_OFFSET;
		flags = port->flags;
		break;
	case VIP_SRCE_RGB:
		if (port->port_id == VIP_PORTB)
			channel += VIP_CHAN_RGB_PORTB_OFFSET;
		flags = port->flags;
		break;
	default:
		BUG();
	}

	if (dev->slice_id == VIP_SLICE2)
		channel += VIP_CHAN_VIP2_OFFSET;

	/* This is just for initialization purposes.
	 * The actual dma_addr will be configured in vpdma_update_dma_addr
	 */
	dma_addr = (dma_addr_t)NULL;

	/*
	 * Use VPDMA_MAX_SIZE1 or VPDMA_MAX_SIZE2 register for slice0/1
	 */

	if (dev->slice_id == VIP_SLICE1) {
		vpdma_set_max_size(dev->shared->vpdma, VPDMA_MAX_SIZE1,
				   stream->width, stream->height);

		max_width = MAX_OUT_WIDTH_REG1;
		max_height = MAX_OUT_HEIGHT_REG1;
	} else {
		vpdma_set_max_size(dev->shared->vpdma, VPDMA_MAX_SIZE2,
				   stream->width, stream->height);

		max_width = MAX_OUT_WIDTH_REG2;
		max_height = MAX_OUT_HEIGHT_REG2;
	}

	/* Mark this channel to be cleared while cleaning up resources
	 * This will make sure that an abort descriptor for this channel
	 * would be submitted to VPDMA causing any ongoing  transaction to be
	 * aborted and cleanup the VPDMA FSM for this channel */
	stream->vpdma_channels[channel] = 1;

	vpdma_rawchan_add_out_dtd(&stream->desc_list, c_rect->width, c_rect,
				  fmt->vpdma_fmt[plane], dma_addr,
				  max_width, max_height, channel, flags);

	return 0;
}

/*
 * add_stream_dtds - prepares and starts DMA for pending transfers
 */
static void add_stream_dtds(struct vip_stream *stream)
{
	struct vip_port *port = stream->port;
	int srce_type;

	if (port->flags & FLAG_MULT_PORT)
		srce_type = VIP_SRCE_MULT_PORT;
	else if (port->flags & FLAG_MULT_ANC)
		srce_type = VIP_SRCE_MULT_ANC;
	else if (port->fmt->colorspace == V4L2_COLORSPACE_SRGB)
		srce_type = VIP_SRCE_RGB;
	else
		srce_type = VIP_SRCE_LUMA;

	add_out_dtd(stream, srce_type);

	if (srce_type == VIP_SRCE_LUMA && port->fmt->coplanar)
		add_out_dtd(stream, VIP_SRCE_CHROMA);
}

static void enable_irqs(struct vip_dev *dev, int irq_num, int list_num)
{
	u32 reg_addr = VIP_INT0_ENABLE0_SET +
			VIP_INTC_INTX_OFFSET * irq_num;

	write_sreg(dev->shared, reg_addr, 1 << (list_num * 2));

	vpdma_enable_list_complete_irq(dev->shared->vpdma,
				       irq_num, list_num, true);
}

static void disable_irqs(struct vip_dev *dev, int irq_num, int list_num)
{
	u32 reg_addr = VIP_INT0_ENABLE0_CLR +
			VIP_INTC_INTX_OFFSET * irq_num;

	write_sreg(dev->shared, reg_addr, 1 << (list_num * 2));

	vpdma_enable_list_complete_irq(dev->shared->vpdma,
				       irq_num, list_num, false);
}

static void clear_irqs(struct vip_dev *dev, int irq_num, int list_num)
{
	u32 reg_addr = VIP_INT0_STATUS0_CLR +
			VIP_INTC_INTX_OFFSET * irq_num;

	write_sreg(dev->shared, reg_addr, 1 << (list_num * 2));

	vpdma_clear_list_stat(dev->shared->vpdma, irq_num, dev->slice_id);
}

static void populate_desc_list(struct vip_stream *stream)
{
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	unsigned int list_length;

	stream->desc_next = stream->desc_list.buf.addr;
	add_stream_dtds(stream);

	list_length = stream->desc_next - stream->desc_list.buf.addr;
	vpdma_map_desc_buf(dev->shared->vpdma, &stream->desc_list.buf);
}

/*
 * start_dma - adds descriptors to the dma list and submits them.
 * Should be called after a new vb is queued and on a vpdma list
 * completion interrupt.
 */
static void start_dma(struct vip_stream *stream, struct vip_buffer *buf)
{
	struct vip_dev *dev = stream->port->dev;
	struct vpdma_data *vpdma = dev->shared->vpdma;
	int list_num = stream->list_num;
	dma_addr_t dma_addr;
	int drop_data;

	if (vpdma_list_busy(vpdma, list_num)) {
		vip_err(dev, "vpdma list busy, cannot post");
		return;				/* nothing to do */
	}

	if (buf) {
		dma_addr = vb2_dma_contig_plane_dma_addr(&buf->vb, 0);
		drop_data = 0;
		vip_dbg(4, dev, "start_dma: buf:0x%08x, vb:0x%08x, dma_addr:0x%08x\n",
			(unsigned int)buf, (unsigned int)&buf->vb, dma_addr);
	} else {
		dma_addr = (dma_addr_t)NULL;
		drop_data = 1;
		vip_dbg(4, dev, "start_dma: dropped\n");
	}

	vpdma_update_dma_addr(dev->shared->vpdma, &stream->desc_list,
			      dma_addr, stream->write_desc, drop_data, 0);

	if (stream->port->fmt->coplanar) {
		dma_addr += stream->width * stream->height;
		vpdma_update_dma_addr(dev->shared->vpdma, &stream->desc_list,
				      dma_addr, stream->write_desc + 1,
				      drop_data, 1);
	}

	vpdma_submit_descs(dev->shared->vpdma,
			   &stream->desc_list, stream->list_num);
}

static void vip_schedule_next_buffer(struct vip_stream *stream)
{
	struct vip_dev *dev = stream->port->dev;
	struct vip_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&dev->slock, flags);
	if (list_empty(&stream->vidq)) {
		vip_dbg(4, dev, "Dropping frame\n");
		if (list_empty(&stream->dropq)) {
			vip_err(dev, "No dropq buffer left!");
			spin_unlock_irqrestore(&dev->slock, flags);
			return;
		}
		buf = list_entry(stream->dropq.next,
				 struct vip_buffer, list);

		buf->drop = true;
		list_move_tail(&buf->list, &stream->post_bufs);
		buf = NULL;
	} else if (vb2_is_streaming(&stream->vb_vidq)) {
		buf = list_entry(stream->vidq.next,
				 struct vip_buffer, list);
		buf->drop = false;
		list_move_tail(&buf->list, &stream->post_bufs);
		vip_dbg(4, dev, "added next buffer\n");
	} else {
		vip_err(dev, "IRQ occurred when not streaming\n");
		if (list_empty(&stream->dropq)) {
			vip_err(dev, "No dropq buffer left!");
			spin_unlock_irqrestore(&dev->slock, flags);
			return;
		}
		buf = list_entry(stream->dropq.next,
				 struct vip_buffer, list);
		buf->drop = true;
		list_move_tail(&buf->list, &stream->post_bufs);
		buf = NULL;
	}

	spin_unlock_irqrestore(&dev->slock, flags);
	start_dma(stream, buf);
}

static void vip_process_buffer_complete(struct vip_stream *stream)
{
	struct vip_dev *dev = stream->port->dev;
	struct vip_buffer *buf;
	struct vb2_buffer *vb = NULL;
	unsigned long flags, fld;

	buf = list_first_entry(&stream->post_bufs, struct vip_buffer, list);

	if (stream->port->flags & FLAG_INTERLACED) {
		vpdma_unmap_desc_buf(dev->shared->vpdma,
				     &stream->desc_list.buf);

		fld = dtd_get_field(stream->write_desc);
		stream->field = fld ? V4L2_FIELD_BOTTOM : V4L2_FIELD_TOP;

		vpdma_map_desc_buf(dev->shared->vpdma, &stream->desc_list.buf);
	}

	if (buf) {
		vip_dbg(4, dev, "vip buffer complete 0x%x, 0x%x\n",
			(unsigned int)buf, buf->drop);

		vb = &buf->vb;
		vb->v4l2_buf.field = stream->field;
		vb->v4l2_buf.sequence = stream->sequence;
		v4l2_get_timestamp(&vb->v4l2_buf.timestamp);

		if (buf->drop) {
			spin_lock_irqsave(&dev->slock, flags);
			list_move_tail(&buf->list, &stream->dropq);
			spin_unlock_irqrestore(&dev->slock, flags);
		} else {
			spin_lock_irqsave(&dev->slock, flags);
			list_del(&buf->list);
			spin_unlock_irqrestore(&dev->slock, flags);
			vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		}
	} else {
		BUG();
	}

	stream->sequence++;
}

static irqreturn_t vip_irq(int irq_vip, void *data)
{
	struct vip_dev *dev = (struct vip_dev *)data;
	struct vpdma_data *vpdma = dev->shared->vpdma;
	struct vip_stream *stream;
	int list_num;
	int irq_num = dev->slice_id;
	u32 irqst, reg_addr;

	if (!dev->shared)
		return IRQ_HANDLED;

	reg_addr = VIP_INT0_STATUS0 +
			VIP_INTC_INTX_OFFSET * irq_num;
	irqst = read_sreg(dev->shared, reg_addr);

	vip_dbg(8, dev, "IRQ %d VIP_INT%d_STATUS0 0x%x\n",
		irq_vip, irq_num, irqst);
	if (irqst) {
		reg_addr = VIP_INT0_STATUS0_CLR +
			VIP_INTC_INTX_OFFSET * irq_num;
		write_sreg(dev->shared, reg_addr, irqst);

		for (list_num = 0; list_num < 8;  list_num++) {
			/* Check for LIST_COMPLETE IRQ */
			if (!(irqst & (1 << list_num * 2)))
				continue;

			vip_dbg(8, dev, "IRQ %d: handling LIST%d_COMPLETE\n",
				irq_num, list_num);

			stream = vpdma_hwlist_get_priv(vpdma, list_num);
			if (!stream || stream->list_num != list_num) {
				vip_err(dev, "IRQ occurred for unused list");
				continue;
			}

			vpdma_clear_list_stat(vpdma, irq_num, list_num);

			if (dev->num_skip_irq)
				dev->num_skip_irq--;
			else
				vip_process_buffer_complete(stream);

			vip_schedule_next_buffer(stream);
			irqst &= ~((1 << list_num * 2));
		}
	}

	return IRQ_HANDLED;
}

/*
 * video ioctls
 */
static int vip_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	strncpy(cap->driver, VIP_MODULE_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, VIP_MODULE_NAME, sizeof(cap->card) - 1);
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 VIP_MODULE_NAME);
	cap->device_caps  = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE |
			    V4L2_CAP_READWRITE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int vip_enuminput(struct file *file, void *priv,
			 struct v4l2_input *inp)
{
	struct vip_stream *stream = file2stream(file);

	if (inp->index)
		return -EINVAL;

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->std = stream->vfd->tvnorms;
	sprintf(inp->name, "camera %u", stream->vfd->num);

	return 0;
}

static int vip_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int vip_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i != 0)
		return -EINVAL;
	return 0;
}

static int vip_querystd(struct file *file, void *fh, v4l2_std_id *std)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;

	*std = stream->vfd->tvnorms;
	v4l2_subdev_call(port->subdev, video, querystd, std);
	vip_dbg(1, dev, "querystd: 0x%lx\n", (unsigned long)*std);
	return 0;
}

static int vip_g_std(struct file *file, void *fh, v4l2_std_id *std)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;

	*std = stream->vfd->tvnorms;
	v4l2_subdev_call(port->subdev, video, g_std_output, std);
	vip_dbg(1, dev, "g_std: 0x%lx\n", (unsigned long)*std);

	return 0;
}

static int vip_s_std(struct file *file, void *fh, v4l2_std_id std)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;

	vip_dbg(1, dev, "s_std: 0x%lx\n", (unsigned long)std);

	if (!(std & stream->vfd->tvnorms)) {
		vip_dbg(1, dev, "s_std after check: 0x%lx\n",
			(unsigned long)std);
		return -EINVAL;
	}

	v4l2_subdev_call(port->subdev, video, s_std_output, std);
	return 0;
}

static int vip_enum_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct vip_fmt *fmt;

	vip_dbg(3, dev, "enum_fmt index:%d\n", f->index);
	if (f->index >= port->num_active_fmt)
		return -EINVAL;

	fmt = port->active_fmt[f->index];

	strncpy(f->description, fmt->name, sizeof(f->description) - 1);
	f->pixelformat = fmt->fourcc;

	vip_dbg(3, dev, "enum_fmt fourcc:%s description:%s\n",
		fourcc_to_str(f->pixelformat), f->description);

	return 0;
}

static int vip_enum_framesizes(struct file *file, void *priv,
			       struct v4l2_frmsizeenum *f)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct vip_fmt *fmt;
	struct v4l2_subdev_frame_size_enum fse;
	int ret;

	fmt = find_port_format_by_pix(port, f->pixel_format);
	if (!fmt)
		return -EINVAL;

	fse.index = f->index;
	fse.pad = 0;
	fse.code = fmt->code;

	ret = v4l2_subdev_call(port->subdev, pad, enum_frame_size, NULL, &fse);
	if (ret)
		return -EINVAL;

	vip_dbg(1, dev, "%s: index: %d code: %x W:[%d,%d] H:[%d,%d]\n",
		__func__, fse.index, fse.code, fse.min_width, fse.max_width,
		fse.min_height, fse.max_height);

	f->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	f->discrete.width = fse.max_width;
	f->discrete.height = fse.max_height;

	return 0;
}

static int vip_enum_frameintervals(struct file *file, void *priv,
				   struct v4l2_frmivalenum *f)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_port *port = stream->port;
	struct v4l2_subdev_frame_size_enum fse;
	struct vip_fmt *fmt;
	int ret;

	if (f->index)
		return -EINVAL;

	fmt = find_port_format_by_pix(port, f->pixel_format);
	if (!fmt)
		return -EINVAL;

	/* check for valid width/height */
	ret = 0;
	fse.pad = 0;
	fse.code = fmt->code;
	fse.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	for (fse.index = 0; ; fse.index++) {
		ret = v4l2_subdev_call(port->subdev, pad, enum_frame_size,
				       NULL, &fse);
		if (ret)
			return -EINVAL;

		if ((f->width == fse.max_width) &&
		    (f->height == fse.max_height))
			break;
		else if ((f->width >= fse.min_width) &&
			 (f->width <= fse.max_width) &&
			 (f->height >= fse.min_height) &&
			 (f->height <= fse.max_height))
			break;

		return -EINVAL;
	}

	f->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	f->discrete.numerator = 1;
	f->discrete.denominator = 30;

	return 0;
}

static int vip_g_parm(struct file *file, void *priv,
		      struct v4l2_streamparm *parm)
{
	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	parm->parm.capture.capability   = V4L2_CAP_TIMEPERFRAME;
	parm->parm.capture.timeperframe.numerator = 1;
	parm->parm.capture.timeperframe.denominator = 30;
	parm->parm.capture.readbuffers  = 4;
	return 0;
}

static int vip_s_parm(struct file *file, void *priv,
		      struct v4l2_streamparm *parm)
{
	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	parm->parm.capture.timeperframe.numerator = 1;
	parm->parm.capture.timeperframe.denominator = 30;
	parm->parm.capture.readbuffers  = 4;

	return 0;
}

static int vip_calc_format_size(struct vip_port *port,
				struct vip_fmt *fmt,
				struct v4l2_format *f)
{
	struct vip_dev *dev = port->dev;
	enum v4l2_field *field;

	if (!fmt) {
		vip_dbg(2, dev,
			"no vip_fmt format provided!\n");
		return -EINVAL;
	}

	field = &f->fmt.pix.field;
	if (*field == V4L2_FIELD_ANY)
		*field = V4L2_FIELD_NONE;
	else if (V4L2_FIELD_NONE != *field && V4L2_FIELD_ALTERNATE != *field)
		return -EINVAL;

	v4l_bound_align_image(&f->fmt.pix.width, MIN_W, MAX_W, W_ALIGN,
			      &f->fmt.pix.height, MIN_H, MAX_H, H_ALIGN,
			      S_ALIGN);

	f->fmt.pix.bytesperline = f->fmt.pix.width *
				  (fmt->vpdma_fmt[0]->depth >> 3);
	f->fmt.pix.bytesperline = ALIGN(f->fmt.pix.bytesperline,
					VPDMA_STRIDE_ALIGN);
	f->fmt.pix.sizeimage = f->fmt.pix.height * f->fmt.pix.width *
		(fmt->vpdma_fmt[0]->depth +
		 (fmt->coplanar ? fmt->vpdma_fmt[1]->depth : 0)) >> 3;
	f->fmt.pix.colorspace = fmt->colorspace;
	f->fmt.pix.priv = 0;

	vip_dbg(3, dev, "calc_format_size: fourcc:%s size: %dx%d bpl:%d img_size:%d\n",
		fourcc_to_str(f->fmt.pix.pixelformat),
		f->fmt.pix.width, f->fmt.pix.height,
		f->fmt.pix.bytesperline, f->fmt.pix.sizeimage);

	return 0;
}

static inline bool vip_is_size_dma_aligned(uint32_t bpp, uint32_t width)
{
	return ((width * bpp) == ALIGN(width * bpp, VPDMA_STRIDE_ALIGN));
}

static int vip_try_fmt_vid_cap(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct v4l2_subdev_frame_size_enum fse;
	struct vip_fmt *fmt;
	uint32_t best_width, best_height;
	int ret, found;

	vip_dbg(3, dev, "try_fmt fourcc:%s size: %dx%d\n",
		fourcc_to_str(f->fmt.pix.pixelformat),
		f->fmt.pix.width, f->fmt.pix.height);

	fmt = find_port_format_by_pix(port, f->fmt.pix.pixelformat);
	if (!fmt) {
		vip_dbg(2, dev,
			"Fourcc format (0x%08x) not found.\n",
			f->fmt.pix.pixelformat);

		/* Just get the first one enumerated */
		fmt = port->active_fmt[0];
		f->fmt.pix.pixelformat = fmt->fourcc;
	}

	/* check for/find a valid width/height */
	ret = 0;
	found = false;
	best_width = 0;
	best_height = 0;
	fse.pad = 0;
	fse.code = fmt->code;
	fse.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	for (fse.index = 0; ; fse.index++) {
		ret = v4l2_subdev_call(port->subdev, pad, enum_frame_size,
				       NULL, &fse);
		if (ret)
			break;

		if (vip_is_size_dma_aligned(fmt->vpdma_fmt[0]->depth >> 3,
					    fse.max_width)) {
			if (abs(best_width - f->fmt.pix.width) >
			    abs(fse.max_width - f->fmt.pix.width)) {
				best_width = fse.max_width;
				best_height = fse.max_height;
			}
			if ((f->fmt.pix.width == fse.max_width) &&
			    (f->fmt.pix.height == fse.max_height)) {
				found = true;
				break;
			} else if ((f->fmt.pix.width >= fse.min_width) &&
				 (f->fmt.pix.width <= fse.max_width) &&
				 (f->fmt.pix.height >= fse.min_height) &&
				 (f->fmt.pix.height <= fse.max_height)) {
				found = true;
				break;
			}
		}
	}

	if (!found) {
		if (best_width) {
			f->fmt.pix.width = best_width;
			f->fmt.pix.height =  best_height;
		} else {
			/* use existing values as default */
			f->fmt.pix.width = port->mbus_framefmt.width;
			f->fmt.pix.height =  port->mbus_framefmt.height;
		}
	}

	/* That we have a fmt calculate imagesize and bytesperline */
	return vip_calc_format_size(port, fmt, f);
}

static int vip_g_fmt_vid_cap(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = stream->port->dev;
	struct vip_fmt *fmt = port->fmt;

	/* Use last known values or defaults */
	f->fmt.pix.width	= stream->width;
	f->fmt.pix.height	= stream->height;
	f->fmt.pix.pixelformat	= port->fmt->fourcc;
	f->fmt.pix.field	= stream->sup_field;
	f->fmt.pix.colorspace	= port->fmt->colorspace;
	f->fmt.pix.bytesperline	= stream->bytesperline;
	f->fmt.pix.sizeimage	= stream->sizeimage;

	vip_dbg(3, dev,
		"g_fmt fourcc:%s code: %04x size: %dx%d bpl:%d img_size:%d\n",
		fourcc_to_str(f->fmt.pix.pixelformat),
		fmt->code,
		f->fmt.pix.width, f->fmt.pix.height,
		f->fmt.pix.bytesperline, f->fmt.pix.sizeimage);
	vip_dbg(3, dev, "g_fmt vpdma data type: 0x%02X\n",
		port->fmt->vpdma_fmt[0]->data_type);

	return 0;
}

static int vip_s_fmt_vid_cap(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct v4l2_subdev_format sfmt;
	struct v4l2_mbus_framefmt *mf;
	int ret;

	vip_dbg(3, dev, "s_fmt input fourcc:%s size: %dx%d\n",
		fourcc_to_str(f->fmt.pix.pixelformat),
		f->fmt.pix.width, f->fmt.pix.height);

	ret = vip_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	vip_dbg(3, dev, "s_fmt try_fmt fourcc:%s size: %dx%d\n",
		fourcc_to_str(f->fmt.pix.pixelformat),
		f->fmt.pix.width, f->fmt.pix.height);

	if (vb2_is_busy(&stream->vb_vidq)) {
		vip_err(dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	port->fmt = find_port_format_by_pix(port,
					    f->fmt.pix.pixelformat);
	stream->width		= f->fmt.pix.width;
	stream->height		= f->fmt.pix.height;
	stream->bytesperline	= f->fmt.pix.bytesperline;
	stream->sizeimage	= f->fmt.pix.sizeimage;
	stream->sup_field	= f->fmt.pix.field;

	port->c_rect.left	= 0;
	port->c_rect.top	= 0;
	port->c_rect.width	= stream->width;
	port->c_rect.height	= stream->height;

	if (stream->sup_field == V4L2_FIELD_ALTERNATE)
		port->flags |= FLAG_INTERLACED;
	else
		port->flags &= ~FLAG_INTERLACED;

	vip_dbg(3, dev, "s_fmt fourcc:%s size: %dx%d bpl:%d img_size:%d\n",
		fourcc_to_str(f->fmt.pix.pixelformat),
		f->fmt.pix.width, f->fmt.pix.height,
		f->fmt.pix.bytesperline, f->fmt.pix.sizeimage);

	mf = &sfmt.format;
	v4l2_fill_mbus_format(mf, &f->fmt.pix, port->fmt->code);

	vip_dbg(3, dev, "s_fmt pix_to_mbus mbus_code: %04X size: %dx%d\n",
		mf->code,
		mf->width, mf->height);

	sfmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sfmt.pad = 0;
	ret = v4l2_subdev_call(port->subdev, pad, set_fmt, NULL, &sfmt);
	if (ret) {
		vip_dbg(1, dev, "set_fmt failed in subdev\n");
		return ret;
	}

	/* Save it */
	port->mbus_framefmt = *mf;

	vip_dbg(3, dev, "s_fmt subdev s_fmt mbus_code: %04X size: %dx%d\n",
		mf->code,
		mf->width, mf->height);
	vip_dbg(3, dev, "s_fmt vpdma data type: 0x%02X\n",
		port->fmt->vpdma_fmt[0]->data_type);

	return 0;
}

/*
 * Set the registers that are modified when the video format changes.
 */
static void set_fmt_params(struct vip_stream *stream)
{
	struct vip_dev *dev = stream->port->dev;
	int data_path_reg;

	stream->sequence = 0;
	stream->field = V4L2_FIELD_TOP;

	if (stream->port->fmt->colorspace == V4L2_COLORSPACE_SRGB) {
		vip_set_slice_path(dev, VIP_RGB_OUT_LO_DATA_SELECT);
		/* Set alpha component in background color */
		vpdma_set_bg_color(dev->shared->vpdma,
				   (struct vpdma_data_format *)
				   stream->port->fmt->vpdma_fmt[0],
				   0xff);
	}

	data_path_reg = VIP_VIP1_DATA_PATH_SELECT + 4 * dev->slice_id;
	if (stream->port->fmt->coplanar) {
		stream->port->flags &= ~FLAG_MULT_PORT;
		write_vreg(dev, data_path_reg, 0x600);
	} else {
		stream->port->flags |= FLAG_MULT_PORT;
		write_vreg(dev, data_path_reg, 0x8000);
	}
}

static int vip_g_selection(struct file *file, void *fh,
			   struct v4l2_selection *s)
{
	struct vip_stream *stream = file2stream(file);

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = stream->width;
		s->r.height = stream->height;
		return 0;

	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_CROP:
		s->r = stream->port->c_rect;
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

static int vip_s_selection(struct file *file, void *fh,
			   struct v4l2_selection *s)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_port *port = stream->port;
	struct v4l2_rect r = s->r;

	v4l_bound_align_image(&r.width, 0, stream->width, 0,
			      &r.height, 0, stream->height, 0, 0);

	r.left = clamp_t(unsigned int, r.left, 0, stream->width - r.width);
	r.top  = clamp_t(unsigned int, r.top, 0, stream->height - r.height);

	if (s->flags & V4L2_SEL_FLAG_LE && !enclosed_rectangle(&r, &s->r))
		return -ERANGE;

	if (s->flags & V4L2_SEL_FLAG_GE && !enclosed_rectangle(&s->r, &r))
		return -ERANGE;

	s->r = r;
	stream->port->c_rect = r;

	set_fmt_params(stream);

	vip_dbg(1, port->dev, "cropped (%d,%d)/%dx%d of %dx%d\n",
		r.left, r.top, r.width, r.height,
		stream->width, stream->height);

	return 0;
}

static long vip_ioctl_default(struct file *file, void *fh, bool valid_prio,
			      unsigned int cmd, void *arg)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_port *port = stream->port;

	if (!valid_prio) {
		vip_err(port->dev, "%s device busy\n", __func__);
		return -EBUSY;
	}

	switch (cmd) {
	default:
		return -ENOTTY;
	}
}

static const struct v4l2_ioctl_ops vip_ioctl_ops = {
	.vidioc_querycap	= vip_querycap,
	.vidioc_enum_input	= vip_enuminput,
	.vidioc_g_input		= vip_g_input,
	.vidioc_s_input		= vip_s_input,

	.vidioc_querystd	= vip_querystd,
	.vidioc_g_std		= vip_g_std,
	.vidioc_s_std		= vip_s_std,

	.vidioc_enum_fmt_vid_cap = vip_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	= vip_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap	= vip_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap	= vip_s_fmt_vid_cap,

	.vidioc_enum_frameintervals	= vip_enum_frameintervals,
	.vidioc_enum_framesizes		= vip_enum_framesizes,
	.vidioc_s_parm			= vip_s_parm,
	.vidioc_g_parm			= vip_g_parm,
	.vidioc_g_selection	= vip_g_selection,
	.vidioc_s_selection	= vip_s_selection,
	.vidioc_reqbufs		= vb2_ioctl_reqbufs,
	.vidioc_create_bufs	= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf	= vb2_ioctl_prepare_buf,
	.vidioc_querybuf	= vb2_ioctl_querybuf,
	.vidioc_qbuf		= vb2_ioctl_qbuf,
	.vidioc_dqbuf		= vb2_ioctl_dqbuf,

	.vidioc_streamon	= vb2_ioctl_streamon,
	.vidioc_streamoff	= vb2_ioctl_streamoff,
	.vidioc_log_status	= v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_default		= vip_ioctl_default,
};

/*
 * Videobuf operations
 */
static int vip_queue_setup(struct vb2_queue *vq,
			   const struct v4l2_format *fmt,
			   unsigned int *nbuffers, unsigned int *nplanes,
			   unsigned int sizes[], void *alloc_ctxs[])
{
	struct vip_stream *stream = vb2_get_drv_priv(vq);
	struct vip_dev *dev = stream->port->dev;

	*nplanes = 1;
	sizes[0] = stream->sizeimage;
	alloc_ctxs[0] = dev->alloc_ctx;
	vip_dbg(1, dev, "get %d buffer(s) of size %d each.\n",
		*nbuffers, sizes[0]);

	return 0;
}

static int vip_buf_prepare(struct vb2_buffer *vb)
{
	struct vip_stream *stream = vb2_get_drv_priv(vb->vb2_queue);
	struct vip_dev *dev = stream->port->dev;

	if (vb2_plane_size(vb, 0) < stream->sizeimage) {
		vip_dbg(1, dev,
			"%s data will not fit into plane (%lu < %lu)\n",
			__func__, vb2_plane_size(vb, 0),
			(long)stream->sizeimage);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, stream->sizeimage);

	return 0;
}

static void vip_buf_queue(struct vb2_buffer *vb)
{
	struct vip_stream *stream = vb2_get_drv_priv(vb->vb2_queue);
	struct vip_dev *dev = stream->port->dev;
	struct vip_buffer *buf = container_of(vb, struct vip_buffer, vb);
	unsigned long flags;

	spin_lock_irqsave(&dev->slock, flags);
	list_add_tail(&buf->list, &stream->vidq);
	spin_unlock_irqrestore(&dev->slock, flags);
}

static int vip_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct vip_stream *stream = vb2_get_drv_priv(vq);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct vip_buffer *buf;
	unsigned long flags;
	int ret;

	set_fmt_params(stream);
	vip_setup_parser(port);

	buf = list_entry(stream->vidq.next,
			 struct vip_buffer, list);

	vip_dbg(2, dev, "start_streaming: buf 0x%x %d\n",
		(unsigned int)buf, count);
	buf->drop = false;
	stream->sequence = 0;
	stream->field = V4L2_FIELD_TOP;

	if (port->subdev) {
		ret = v4l2_subdev_call(port->subdev, video, s_stream, 1);
		if (ret) {
			vip_dbg(1, dev, "stream on failed in subdev\n");
			return ret;
		}
	}

	populate_desc_list(stream);

	/* The first few VPDMA ListComplete interrupts fire pretty quiclky
	 * until the internal VPDMA descriptor fifo is full.
	 * The subsequent ListComplete interrupts will fire at the actual
	 * capture frame rate. The first few interrupts are therefore used
	 * only to queue up descriptors, and then they will also be used
	 * as End of Frame (EOF) event
	 */
	dev->num_skip_irq = VIP_VPDMA_FIFO_SIZE;

	spin_lock_irqsave(&dev->slock, flags);
	if (vpdma_list_busy(dev->shared->vpdma, stream->list_num)) {
		spin_unlock_irqrestore(&dev->slock, flags);
		vpdma_unmap_desc_buf(dev->shared->vpdma,
				     &stream->desc_list.buf);
		vpdma_reset_desc_list(&stream->desc_list);
		return -EBUSY;
	}

	list_move_tail(&buf->list, &stream->post_bufs);
	spin_unlock_irqrestore(&dev->slock, flags);

	vip_dbg(2, dev, "start_streaming: start_dma buf 0x%x\n",
		(unsigned int)buf);
	start_dma(stream, buf);

	/* We enable the irq after posting the vpdma descriptor
	 * to prevent sprurious interrupt coming in before the
	 * vb2 layer is completely ready to handle them
	 * otherwise the vb2_streaming test would fail early on
	  */
	enable_irqs(dev, dev->slice_id, stream->list_num);

	return 0;
}

/*
 * Abort streaming and wait for last buffer
 */
static void vip_stop_streaming(struct vb2_queue *vq)
{
	struct vip_stream *stream = vb2_get_drv_priv(vq);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct vip_buffer *buf;
	int ret;

	if (port->subdev) {
		ret = v4l2_subdev_call(port->subdev, video, s_stream, 0);
		if (ret)
			vip_dbg(1, dev, "stream on failed in subdev\n");
	}

	disable_irqs(dev, dev->slice_id, stream->list_num);
	clear_irqs(dev, dev->slice_id, stream->list_num);
	stop_dma(stream);

	/* release all active buffers */
	while (!list_empty(&stream->post_bufs)) {
		buf = list_entry(stream->post_bufs.next,
				 struct vip_buffer, list);
		list_del(&buf->list);
		if (buf->drop == 1)
			list_add_tail(&buf->list, &stream->dropq);
		else
			vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}
	while (!list_empty(&stream->vidq)) {
		buf = list_entry(stream->vidq.next, struct vip_buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}

	if (!vb2_is_streaming(vq))
		return;

	vpdma_unmap_desc_buf(dev->shared->vpdma, &stream->desc_list.buf);
	vpdma_reset_desc_list(&stream->desc_list);
}

static struct vb2_ops vip_video_qops = {
	.queue_setup		= vip_queue_setup,
	.buf_prepare		= vip_buf_prepare,
	.buf_queue		= vip_buf_queue,
	.start_streaming	= vip_start_streaming,
	.stop_streaming		= vip_stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

/*
 * File operations
 */

static int vip_init_dev(struct vip_dev *dev)
{
	if (dev->num_ports != 0)
		goto done;

	vip_set_clock_enable(dev, 1);
done:
	dev->num_ports++;

	return 0;
}

static int vip_init_port(struct vip_port *port)
{
	int ret;
	struct vip_dev *dev = port->dev;
	struct vip_fmt *fmt;
	struct v4l2_subdev_format sd_fmt;
	struct v4l2_mbus_framefmt *mbus_fmt = &sd_fmt.format;

	if (port->num_streams != 0)
		goto done;

	ret = vip_init_dev(port->dev);
	if (ret)
		goto done;

	/* Get subdevice current frame format */
	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sd_fmt.pad = 0;
	ret = v4l2_subdev_call(port->subdev, pad, get_fmt, NULL, &sd_fmt);
	if (ret)
		vip_dbg(1, dev, "init_port get_fmt failed in subdev\n");

	/* try to find one that matches */
	fmt = find_port_format_by_code(port, mbus_fmt->code);
	if (!fmt) {
		vip_dbg(1, dev, "subdev default mbus_fmt %04x is not matched.\n",
			mbus_fmt->code);
		/* if all else fails just pick the first one */
		fmt = port->active_fmt[0];

		mbus_fmt->code = fmt->code;
		sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		sd_fmt.pad = 0;
		ret = v4l2_subdev_call(port->subdev, pad, set_fmt,
				       NULL, &sd_fmt);
		if (ret)
			vip_dbg(1, dev, "init_port set_fmt failed in subdev\n");
	}

	/* Assign current format */
	port->fmt = fmt;
	port->mbus_framefmt = *mbus_fmt;

	vip_dbg(3, dev, "vip_init_port: g_mbus_fmt subdev mbus_code: %04X fourcc:%s size: %dx%d\n",
		fmt->code,
		fourcc_to_str(fmt->fourcc),
		mbus_fmt->width, mbus_fmt->height);

	if (mbus_fmt->field == V4L2_FIELD_ALTERNATE)
		port->flags |= FLAG_INTERLACED;
	else
		port->flags &= ~FLAG_INTERLACED;

	port->c_rect.left	= 0;
	port->c_rect.top	= 0;
	port->c_rect.width	= mbus_fmt->width;
	port->c_rect.height	= mbus_fmt->height;

done:
	port->num_streams++;
	return 0;
}

static int vip_init_stream(struct vip_stream *stream)
{
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct vip_fmt *fmt;
	struct v4l2_mbus_framefmt *mbus_fmt;
	struct v4l2_format f;
	int ret;

	ret = vip_init_port(port);
	if (ret != 0)
		return ret;

	fmt = port->fmt;
	mbus_fmt = &port->mbus_framefmt;

	/* Properly calculate the sizeimage and bytesperline values. */
	v4l2_fill_pix_format(&f.fmt.pix, mbus_fmt);
	f.fmt.pix.pixelformat = fmt->fourcc;
	ret = vip_calc_format_size(port, fmt, &f);
	if (ret)
		return ret;

	stream->width = f.fmt.pix.width;
	stream->height = f.fmt.pix.height;
	stream->sup_field = f.fmt.pix.field;
	stream->bytesperline = f.fmt.pix.bytesperline;
	stream->sizeimage = f.fmt.pix.sizeimage;

	vip_dbg(3, dev, "init_stream fourcc:%s size: %dx%d bpl:%d img_size:%d\n",
		fourcc_to_str(f.fmt.pix.pixelformat),
		f.fmt.pix.width, f.fmt.pix.height,
		f.fmt.pix.bytesperline, f.fmt.pix.sizeimage);
	vip_dbg(3, dev, "init_stream vpdma data type: 0x%02X\n",
		port->fmt->vpdma_fmt[0]->data_type);

	ret = vpdma_create_desc_list(&stream->desc_list, VIP_DESC_LIST_SIZE,
				     VPDMA_LIST_TYPE_NORMAL);

	if (ret != 0)
		return ret;

	stream->write_desc = (struct vpdma_dtd *)stream->desc_list.buf.addr
				+ 15;
	return 0;
}

static void vip_release_dev(struct vip_dev *dev)
{
	/*
	 * On last close, disable clocks to conserve power
	 */

	if (--dev->num_ports == 0)
		vip_set_clock_enable(dev, 0);
}

static int vip_setup_parser(struct vip_port *port)
{
	struct vip_dev *dev = port->dev;
	struct v4l2_of_endpoint *endpoint = port->endpoint;
	int iface, sync_type;
	uint32_t flags = 0, config0;

	/* Reset the port */
	write_vreg(port->dev, vip_parser_field(port, 0), VIP_SW_RESET);
	usleep_range(200, 250);
	write_vreg(port->dev, vip_parser_field(port, 0), 0x00000000);

	write_vreg(port->dev, vip_parser_field(port, 0), VIP_PORT_ENABLE);
	config0 = read_vreg(port->dev, vip_parser_field(port, 0));

	if (endpoint->bus_type == V4L2_MBUS_BT656) {
		flags = endpoint->bus.parallel.flags;
		iface = DUAL_8B_INTERFACE;

		/* Ideally, this should come from subdev
		   port->fmt can be anything once CSC is enabled */
		if (port->fmt->colorspace == V4L2_COLORSPACE_SRGB) {
			sync_type = EMBEDDED_SYNC_SINGLE_RGB_OR_YUV444;
		} else {
			switch (endpoint->bus.parallel.num_channels) {
			case 4:
				sync_type = EMBEDDED_SYNC_4X_MULTIPLEXED_YUV422;
				break;
			case 2:
				sync_type = EMBEDDED_SYNC_2X_MULTIPLEXED_YUV422;
				break;
			case 1:
				sync_type = EMBEDDED_SYNC_SINGLE_YUV422;
				break;
			default:
				sync_type =
				EMBEDDED_SYNC_LINE_MULTIPLEXED_YUV422;
			}
			if (endpoint->bus.parallel.pixmux == 0)
				sync_type =
				EMBEDDED_SYNC_LINE_MULTIPLEXED_YUV422;
		}

	} else if (endpoint->bus_type == V4L2_MBUS_PARALLEL) {
		flags = endpoint->bus.parallel.flags;

		switch (endpoint->bus.parallel.bus_width) {
		case 24:
			iface = SINGLE_24B_INTERFACE;
		break;
		case 16:
			iface = SINGLE_16B_INTERFACE;
		break;
		case 8:
		default:
			iface = DUAL_8B_INTERFACE;
		}

		if (port->fmt->colorspace == V4L2_COLORSPACE_SRGB)
			sync_type = DISCRETE_SYNC_SINGLE_RGB_24B;
		else
			sync_type = DISCRETE_SYNC_SINGLE_YUV422;

		if (flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH)
			config0 |= VIP_HSYNC_POLARITY;
		else if (flags & V4L2_MBUS_HSYNC_ACTIVE_LOW)
			config0 &= ~VIP_HSYNC_POLARITY;

		if (flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH)
			config0 |= VIP_VSYNC_POLARITY;
		else if (flags & V4L2_MBUS_VSYNC_ACTIVE_LOW)
			config0 &= ~VIP_VSYNC_POLARITY;

		config0 &= ~VIP_USE_ACTVID_HSYNC_ONLY;
		config0 |= VIP_ACTVID_POLARITY;
		config0 |= VIP_DISCRETE_BASIC_MODE;

	} else {
		vip_err(dev, "Device doesn't support CSI2");
		return -EINVAL;
	}

	if (flags & V4L2_MBUS_PCLK_SAMPLE_FALLING) {
		vip_set_pclk_invert(port);
		config0 |= VIP_PIXCLK_EDGE_POLARITY;
	} else {
		config0 &= ~VIP_PIXCLK_EDGE_POLARITY;
	}

	config0 |= ((sync_type & VIP_SYNC_TYPE_MASK) << VIP_SYNC_TYPE_SHFT);
	write_vreg(port->dev, vip_parser_field(port, 0), config0);

	vip_set_data_interface(port, iface);

	return 0;
}

static void vip_release_stream(struct vip_stream *stream)
{
	struct vip_dev *dev = stream->port->dev;

	vpdma_unmap_desc_buf(dev->shared->vpdma, &stream->desc_list.buf);
	vpdma_free_desc_buf(&stream->desc_list.buf);
	vpdma_free_desc_list(&stream->desc_list);
}

static void stop_dma(struct vip_stream *stream)
{
	struct vip_dev *dev = stream->port->dev;
	int ch, size = 0;

	/* Create a list of channels to be cleared */
	for (ch = 0; ch < VPDMA_MAX_CHANNELS; ch++) {
		if (stream->vpdma_channels[ch] == 1) {
			stream->vpdma_channels[size++] = ch;
			vip_dbg(2, dev, "Clear channel no: %d\n", ch);
		}
	}

	/* Clear all the used channels for the list */
	vpdma_list_cleanup(dev->shared->vpdma, stream->list_num,
			   stream->vpdma_channels, size);

	for (ch = 0; ch < VPDMA_MAX_CHANNELS; ch++)
		stream->vpdma_channels[ch] = 0;
}

static int vip_open(struct file *file)
{
	struct vip_stream *stream = video_drvdata(file);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct v4l2_fh *fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	int ret = 0;

	vip_dbg(2, dev, "vip_open\n");

	file->private_data = fh;
	if (!fh)
		return -ENOMEM;

	mutex_lock(&dev->mutex);

	v4l2_fh_init(fh, video_devdata(file));
	v4l2_fh_add(fh);

	/*
	 * If this is the first open file.
	 * Then initialize hw module.
	 */
	if (v4l2_fh_is_singular_file(file)) {
		if (vip_init_stream(stream)) {
			ret = -ENODEV;
			goto free_fh;
		}
		vip_dbg(1, dev, "Created stream instance %p\n", stream);
	}

	mutex_unlock(&dev->mutex);
	return 0;

free_fh:
	mutex_unlock(&dev->mutex);
	if (fh) {
		v4l2_fh_del(fh);
		v4l2_fh_exit(fh);
		kfree(fh);
	}
	return ret;
}

static int vip_release(struct file *file)
{
	struct vip_stream *stream = video_drvdata(file);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct vb2_queue *q = &stream->vb_vidq;

	vip_dbg(2, dev, "vip_release\n");

	/*
	 * If this is the last open file.
	 * Then de-initialize hw module.
	 */
	if (v4l2_fh_is_singular_file(file)) {
		mutex_lock(&dev->mutex);

		vip_stop_streaming(q);
		vip_release_stream(stream);

		if (--port->num_streams == 0)
			vip_release_dev(port->dev);

		mutex_unlock(&dev->mutex);
		vip_dbg(1, dev, "Releasing stream instance %p\n", stream);
	}

	return vb2_fop_release(file);
}

static const struct v4l2_file_operations vip_fops = {
	.owner		= THIS_MODULE,
	.open		= vip_open,
	.release	= vip_release,
	.read		= vb2_fop_read,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
};

static struct video_device vip_videodev = {
	.name		= VIP_MODULE_NAME,
	.fops		= &vip_fops,
	.ioctl_ops	= &vip_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,
	.tvnorms	= V4L2_STD_NTSC | V4L2_STD_PAL | V4L2_STD_SECAM,
};

static int alloc_stream(struct vip_port *port, int stream_id, int vfl_type)
{
	struct vip_stream *stream;
	struct vip_dev *dev = port->dev;
	struct vb2_queue *q;
	struct video_device *vfd;
	struct vip_buffer *buf;
	int ret, i;

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	stream->port = port;
	stream->stream_id = stream_id;
	stream->vfl_type = vfl_type;

	stream->list_num = vpdma_hwlist_alloc(dev->shared->vpdma, stream);
	if (stream->list_num < 0) {
		vip_err(dev, "Could not get VPDMA hwlist");
		ret = -ENODEV;
		goto do_free_stream;
	}

	INIT_LIST_HEAD(&stream->post_bufs);

	if (vfl_type == VFL_TYPE_GRABBER)
		port->cap_streams[stream_id] = stream;
	else
		port->vbi_streams[stream_id] = stream;

	/*
	 * Initialize queue
	 */
	q = &stream->vb_vidq;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	q->drv_priv = stream;
	q->buf_struct_size = sizeof(struct vip_buffer);
	q->ops = &vip_video_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &dev->mutex;
	q->min_buffers_needed = 3;

	ret = vb2_queue_init(q);
	if (ret)
		goto do_free_stream;

	INIT_LIST_HEAD(&stream->vidq);

	/* Allocate/populate Drop queue entries */
	INIT_LIST_HEAD(&stream->dropq);
	for (i = 0; i < VIP_DROPQ_SIZE; i++) {
		buf = kzalloc(sizeof(*buf), GFP_ATOMIC);
		if (!buf) {
			ret = -ENOMEM;
			goto do_free_stream;
		}
		buf->drop = true;
		list_add(&buf->list, &stream->dropq);
	}

	vfd = video_device_alloc();
	if (!vfd)
		goto do_free_stream;
	*vfd = vip_videodev;
	vfd->v4l2_dev = &dev->v4l2_dev;
	vfd->queue = q;

	vfd->lock = &dev->mutex;
	video_set_drvdata(vfd, stream);

	ret = video_register_device(vfd, vfl_type, -1);
	if (ret) {
		vip_err(dev, "Failed to register video device\n");
		goto do_free_stream;
	}

	stream->vfd = vfd;

	vip_info(dev, "device registered as %s\n",
		 video_device_node_name(vfd));
	return 0;

do_free_stream:
	kfree(stream);
	return ret;
}

static void free_stream(struct vip_stream *stream)
{
	struct vip_dev *dev;
	struct vip_buffer *buf;
	struct list_head *pos, *q;

	if (!stream)
		return;

	dev = stream->port->dev;
	/* Free up the Drop queue */
	list_for_each_safe(pos, q, &stream->dropq) {
		buf = list_entry(pos,
				 struct vip_buffer, list);
		vip_dbg(1, dev, "dropq buffer\n");
		list_del(pos);
		kfree(buf);
	}

	video_unregister_device(stream->vfd);
	vpdma_hwlist_release(dev->shared->vpdma, stream->list_num);
	stream->port->cap_streams[stream->stream_id] = NULL;
	kfree(stream);
}

static int get_subdev_active_format(struct vip_port *port,
				    struct v4l2_subdev *subdev)
{
	struct vip_dev *dev = port->dev;
	struct vip_fmt *fmt;
	struct v4l2_subdev_mbus_code_enum mbus_code;
	int ret = 0;
	unsigned int k, i, j;

	/* Enumerate sub device formats and enable all matching local formats */
	port->num_active_fmt = 0;
	for (k = 0, i = 0;
	     (ret != -EINVAL);
	     k++) {
		memset(&mbus_code, 0, sizeof(mbus_code));
		mbus_code.index = k;
		ret = v4l2_subdev_call(subdev, pad, enum_mbus_code,
				       NULL, &mbus_code);
		if (ret == 0) {
			vip_dbg(2, dev,
				"subdev %s: code: %04x idx: %d\n",
				subdev->name, mbus_code.code, k);

			for (j = 0; j < ARRAY_SIZE(vip_formats); j++) {
				fmt = &vip_formats[j];
				if (mbus_code.code == fmt->code) {
					port->active_fmt[i] = fmt;
					vip_dbg(2, dev,
						"matched fourcc: %s: code: %04x idx: %d\n",
						fourcc_to_str(fmt->fourcc),
						fmt->code, i);
					port->num_active_fmt = ++i;
				}
			}
		}
	}

	if (i == 0) {
		vip_err(dev, "No suitable format reported by subdev %s\n",
			subdev->name);
		return -EINVAL;
	}
	return 0;
}

static int alloc_port(struct vip_dev *dev, int id)
{
	struct vip_port *port;

	port = kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	dev->ports[id] = port;
	port->dev = dev;
	port->port_id = id;
	port->num_streams = 0;
	return 0;
}

static void free_port(struct vip_port *port)
{
	if (!port)
		return;

	v4l2_async_notifier_unregister(&port->notifier);
	free_stream(port->cap_streams[0]);

	kfree(port);
}

static int get_field(u32 value, u32 mask, int shift)
{
	return (value & (mask << shift)) >> shift;
}

static int vip_of_probe(struct platform_device *pdev);
static void vip_vpdma_fw_cb(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "VPDMA firmware loaded\n");

	if (pdev->dev.of_node) {
		vip_of_probe(pdev);
	}
}

static int vip_create_streams(struct vip_port *port,
			      struct v4l2_subdev *subdev)
{
	struct v4l2_of_bus_parallel *bus;
	int i;

	for (i = 0; i < VIP_CAP_STREAMS_PER_PORT; i++)
		free_stream(port->cap_streams[i]);

	if (get_subdev_active_format(port, subdev))
		return -ENODEV;

	if (port->endpoint->bus_type == V4L2_MBUS_PARALLEL) {
		port->flags |= FLAG_MULT_PORT;
		alloc_stream(port, 0, VFL_TYPE_GRABBER);
	} else if (port->endpoint->bus_type == V4L2_MBUS_BT656) {
		port->flags |= FLAG_MULT_PORT;
		bus = &port->endpoint->bus.parallel;
		for (i = 0; i < bus->num_channels; i++) {
			if (bus->channels[i] >= 16)
				continue;
			alloc_stream(port, bus->channels[i], VFL_TYPE_GRABBER);
		}
	}
	return 0;
}

static int vip_async_bound(struct v4l2_async_notifier *notifier,
			   struct v4l2_subdev *subdev,
			   struct v4l2_async_subdev *asd)
{
	struct vip_port *port = notifier_to_vip_port(notifier);
	struct vip_async_config *config = &port->config;
	struct vip_dev *dev = port->dev;
	unsigned int idx = asd - &config->asd[0];
	int ret;

	vip_dbg(1, dev, "vip_async_bound\n");
	if (idx > config->asd_sizes)
		return -EINVAL;

	if (port->subdev) {
		if (asd < port->subdev->asd)
			/* Notified of a subdev earlier in the array */
			vip_info(dev, "Switching to subdev %s (High priority)",
				 subdev->name);

		else {
			vip_info(dev, "Rejecting subdev %s (Low priority)",
				 subdev->name);
			return 0;
		}
	}

	port->endpoint = &config->endpoints[idx];
	vip_info(dev, "Port %c: Using subdev %s for capture\n",
		 port->port_id == VIP_PORTA ? 'A' : 'B', subdev->name);

	ret = vip_create_streams(port, subdev);
	if (ret)
		return ret;

	port->subdev = subdev;

	return 0;
}

static int vip_async_complete(struct v4l2_async_notifier *notifier)
{
	struct vip_port *port = notifier_to_vip_port(notifier);
	struct vip_dev *dev = port->dev;

	vip_dbg(1, dev, "vip_async_complete\n");
	return 0;
}

static struct device_node *
of_get_next_available_port(const struct device_node *parent,
			   struct device_node *prev)
{
	struct device_node *port = NULL;

	if (!parent)
		return NULL;

	do {
		port = of_get_next_available_child(parent, prev);
		if (!port)
			return NULL;

		prev = port;
	} while (of_node_cmp(port->name, "port") != 0);

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

static int vip_register_subdev_notif(struct vip_port *port,
				struct device_node *port_node)
{
	struct vip_async_config *config = &port->config;
	struct v4l2_async_notifier *notifier = &port->notifier;
	struct v4l2_async_subdev *asd;
	struct vip_dev *dev = port->dev;
	struct device_node *ep_node = NULL, *subdev_node, *subdev_ep;
	int i = 0, ret;

	while (i < VIP_MAX_SUBDEV) {

		ep_node = of_get_next_endpoint(port_node, ep_node);
		if (!ep_node) {
			vip_dbg(3, dev, "can't get next endpoint: loop: %d\n",
				i);
			break;
		}

		subdev_node = of_graph_get_remote_port_parent(ep_node);
		if (!subdev_node) {
			vip_dbg(3, dev, "can't get remote parent: loop: %d\n",
				i);
			goto of_node_cleanup;
		}

		subdev_ep = of_parse_phandle(ep_node, "remote-endpoint", 0);
		if (!subdev_ep) {
			vip_dbg(3, dev, "can't get remote-endpoint: loop: %d\n",
				i);
			goto of_node_cleanup;
		}

		ret = v4l2_of_parse_endpoint(subdev_ep, &config->endpoints[i]);
		if (ret) {
			vip_dbg(3, dev, "Failed to parse endpoint: loop: %d\n",
				i);
			goto of_node_cleanup;
		}

		asd = &config->asd[i];
		asd->match_type = V4L2_ASYNC_MATCH_OF;
		asd->match.of.node = subdev_node;
		config->asd_list[i] = asd;
		i++;

of_node_cleanup:
		if (!subdev_ep)
			of_node_put(subdev_ep);
		if (!subdev_node)
			of_node_put(subdev_node);
	}

	if (i == 0) {
		vip_err(dev, "Port %c enabled but no endpoints found\n",
			port->port_id == VIP_PORTA ? 'A' : 'B');
		ret = -EINVAL;
		goto skip_async;
	}

	config->asd_sizes = i;
	notifier->bound = vip_async_bound;
	notifier->complete = vip_async_complete;
	notifier->subdevs = config->asd_list;
	notifier->num_subdevs = config->asd_sizes;

	vip_dbg(1, dev, "register async notifier for %d subdevs\n", i);
	ret = v4l2_async_notifier_register(&dev->v4l2_dev, notifier);
	if (ret) {
		vip_dbg(1, dev, "Error registering async notifier\n");
		ret = -EINVAL;
	}

skip_async:
	return ret;
}

static int vip_of_probe(struct platform_device *pdev)
{
	struct vip_shared *shared = platform_get_drvdata(pdev);
	struct regmap *syscon;
	struct vip_port *port;
	struct vip_dev *dev;

	struct device_node *parent = pdev->dev.of_node;
	struct device_node *port_node = NULL, *syscon_np;
	int ret, slice_id, port_id;
	u32 regval = 0;

	syscon_np = of_parse_phandle(pdev->dev.of_node, "syscon", 0);
	syscon = syscon_node_to_regmap(syscon_np);
	of_node_put(syscon_np);

	while (1) {
		port_node = of_get_next_available_port(parent, port_node);
		if (!port_node)
			break;

		/* Find the port from <REG> */
		of_property_read_u32(port_node, "reg", &regval);
		switch (regval) {
		case 0:
			slice_id = VIP_SLICE1;	port_id = VIP_PORTA;
			break;
		case 1:
			slice_id = VIP_SLICE2;	port_id = VIP_PORTA;
			break;
		case 2:
			slice_id = VIP_SLICE1;	port_id = VIP_PORTB;
			break;
		case 3:
			slice_id = VIP_SLICE2;	port_id = VIP_PORTB;
			break;
		default:
			dev_err(&pdev->dev, "Unknown port reg=<%d>\n", regval);
			continue;
		}

		dev = shared->devs[slice_id];
		dev->syscon = syscon;
		alloc_port(dev, port_id);
		port = dev->ports[port_id];

		ret = vip_register_subdev_notif(port, port_node);
		of_node_put(port_node);
	}
	return 0;
}

static const struct of_device_id vip_of_match[];
static int vip_probe(struct platform_device *pdev)
{
	struct vip_dev *dev;
	struct vip_shared *shared;
	const struct of_device_id *of_dev_id;
	struct pinctrl *pinctrl;
	int ret, slice = VIP_SLICE1;
	u32 tmp, pid;
	struct v4l2_ctrl_handler *hdl;

	pm_runtime_enable(&pdev->dev);

	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret)
		goto err_runtime_get;

	of_dev_id = of_match_device(vip_of_match, &pdev->dev);
	if (!of_dev_id) {
		dev_err(&pdev->dev, "%s: Unable to match device\n", __func__);
		return -ENODEV;
	}

	shared = kzalloc(sizeof(*shared), GFP_KERNEL);
	if (!shared)
		return -ENOMEM;

	shared->res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "vip");
	shared->base = devm_ioremap_resource(&pdev->dev, shared->res);
	if (IS_ERR(shared->base)) {
		dev_err(&pdev->dev, "failed to ioremap\n");
		ret = PTR_ERR(shared->base);
		goto free_shared;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);

	/* Make sure H/W module has the right functionality */
	pid = read_sreg(shared, VIP_PID);
	tmp = get_field(pid, VIP_PID_FUNC_MASK, VIP_PID_FUNC_SHIFT);

	if (tmp != VIP_PID_FUNC) {
		dev_info(&pdev->dev, "vip: unexpected PID function: 0x%x\n",
			 tmp);
		ret = -ENODEV;
		goto free_shared;
	}

	/* enable clocks, so the firmware will load properly */
	vip_shared_set_clock_enable(shared, 1);
	vip_top_vpdma_reset(shared);

	platform_set_drvdata(pdev, shared);

	for (slice = VIP_SLICE1; slice < VIP_NUM_SLICES; slice++) {
		dev = kzalloc(sizeof(*dev), GFP_KERNEL);
		if (!dev)
			return -ENOMEM;

		dev->instance_id = (int)of_dev_id->data;
		snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name),
			 "%s%d-s%d", VIP_MODULE_NAME, dev->instance_id, slice);

		dev->irq = platform_get_irq(pdev, slice);
		if (!dev->irq) {
			dev_err(&pdev->dev, "Could not get IRQ");
			goto err_runtime_get;
		}

		if (devm_request_irq(&pdev->dev, dev->irq, vip_irq,
				     0, dev->v4l2_dev.name, dev) < 0) {
			ret = -ENOMEM;
			goto dev_unreg;
		}

		spin_lock_init(&dev->slock);
		spin_lock_init(&dev->lock);

		ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
		if (ret)
			goto err_runtime_get;

		mutex_init(&dev->mutex);

		hdl = &dev->ctrl_handler;
		v4l2_ctrl_handler_init(hdl, 11);
		dev->v4l2_dev.ctrl_handler = hdl;

		dev->slice_id = slice;
		dev->pdev = pdev;
		dev->res = shared->res;
		dev->base = shared->base;

		dev->shared = shared;
		shared->devs[slice] = dev;

		dev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
		if (IS_ERR(dev->alloc_ctx)) {
			vip_err(dev, "Failed to alloc vb2 context\n");
			ret = PTR_ERR(dev->alloc_ctx);
			goto dev_unreg;
		}

		vip_top_reset(dev);
		vip_set_slice_path(dev, VIP_MULTI_CHANNEL_DATA_SELECT);
	}

	shared->vpdma = &shared->vpdma_data;
	ret = vpdma_create(pdev, shared->vpdma, vip_vpdma_fw_cb);
	if (ret) {
		dev_err(&pdev->dev, "Creating VPDMA failed");
		goto dev_unreg;
	}

	return 0;

dev_unreg:
	v4l2_device_unregister(&dev->v4l2_dev);
free_shared:
	kfree(shared);
err_runtime_get:
	if (slice == VIP_SLICE1) {
		pm_runtime_disable(&pdev->dev);
		return ret;
	} else {
		return 0;
	}
}

static int vip_remove(struct platform_device *pdev)
{
	struct vip_shared *shared = platform_get_drvdata(pdev);
	struct vip_dev *dev;
	int slice;

	for (slice = 0; slice < VIP_NUM_SLICES; slice++) {
		dev = shared->devs[slice];
		if (!dev)
			continue;

		free_port(dev->ports[VIP_PORTA]);
		free_port(dev->ports[VIP_PORTB]);
		vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
		kfree(dev);
	}
	kfree(shared);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id vip_of_match[] = {
	{
		.compatible = "ti,vip1", .data = (void *)VIP_INSTANCE1,
	},

	{
		.compatible = "ti,vip2", .data = (void *)VIP_INSTANCE2,
	},

	{
		.compatible = "ti,vip3", .data = (void *)VIP_INSTANCE3,
	},
	{},
};
MODULE_DEVICE_TABLE(of, vip_of_match);
#endif

static struct platform_driver vip_pdrv = {
	.probe		= vip_probe,
	.remove		= vip_remove,
	.driver		= {
		.name	= VIP_MODULE_NAME,
		.of_match_table = of_match_ptr(vip_of_match),
	},
};

module_platform_driver(vip_pdrv);

MODULE_DESCRIPTION("TI VIP driver");
MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL v2");
