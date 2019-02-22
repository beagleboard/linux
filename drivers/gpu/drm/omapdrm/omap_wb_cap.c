// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016-2018 Texas Instruments Incorporated -  http://www.ti.com/
 * Author: Benoit Parrot <bparrot@ti.com>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#include "omap_wb.h"

static int omap_channel_to_wb_channel(int oc)
{
	switch (oc) {
	case OMAP_DSS_CHANNEL_LCD:
		return DSS_WB_LCD1_MGR;
	case OMAP_DSS_CHANNEL_DIGIT:
		return DSS_WB_TV_MGR;
	case OMAP_DSS_CHANNEL_LCD2:
		return DSS_WB_LCD2_MGR;
	case OMAP_DSS_CHANNEL_LCD3:
		return DSS_WB_LCD3_MGR;
	default:
		return DSS_WB_LCD1_MGR;
	}
}

static char *omap_channel_to_name(int oc)
{
	switch (oc) {
	case OMAP_DSS_CHANNEL_LCD:
		return "LCD1";
	case OMAP_DSS_CHANNEL_DIGIT:
		return "DIGIT/TV";
	case OMAP_DSS_CHANNEL_LCD2:
		return "LCD2";
	case OMAP_DSS_CHANNEL_LCD3:
		return "LCD3";
	default:
		return "LCD1";
	}
}

/* driver info for each of the supported input overlay/mgr */
struct wb_input {
	char name[64];
	u32 wb_channel;
	u32 omap_channel;
	u32 crtc_index;
};

static struct wb_input wb_inputs[8];
static int num_wb_input;

static bool is_input_active(struct wbcap_dev *wbcap)
{
	struct omap_drm_private *priv = wbcap->dev->drm_dev->dev_private;
	u32 oc = wb_inputs[wbcap->input].omap_channel;

	return priv->dispc_ops->mgr_is_enabled(priv->dispc, oc);
}

static bool is_input_enabled(struct wbcap_dev *wbcap)
{
	struct omap_drm_private *priv = wbcap->dev->drm_dev->dev_private;
	struct drm_crtc *crtc;
	struct wb_input *input;

	input = &wb_inputs[wbcap->input];
	crtc = priv->pipes[input->crtc_index].crtc;

	return crtc->enabled;
}

static void build_input_table(struct wbcap_dev *wbcap)
{
	struct omap_drm_private *priv = wbcap->dev->drm_dev->dev_private;
	struct drm_crtc *crtc;
	struct wb_input *input;
	int i;

	for (i = 0; i < priv->num_pipes; i++) {
		crtc = priv->pipes[i].crtc;
		input = &wb_inputs[i];

		input->crtc_index = i;
		input->omap_channel = omap_crtc_channel(crtc);
		input->wb_channel =
			omap_channel_to_wb_channel(input->omap_channel);
		snprintf(input->name, sizeof(input->name), "CRTC#%d - %s",
			 i, omap_channel_to_name(input->omap_channel));

		log_dbg(wbcap, "Input# %d, name:'%s' omap_channel:%d wb_channel:%d\n",
			i, input->name, input->omap_channel, input->wb_channel);
	}
	num_wb_input = i;
}

static struct wb_q_data *get_q_data(struct wbcap_dev *dev,
				    enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return &dev->q_data[Q_DATA_DST];
	default:
		return NULL;
	}
	return NULL;
}

static bool wb_cap_setup(struct wbcap_dev *dev,
			 enum dss_writeback_channel wb_channel,
			 const struct omap_dss_writeback_info *wb_info)
{
	struct omap_drm_private *priv = dev->dev->drm_dev->dev_private;
	struct drm_crtc *crtc;
	struct videomode *ct;
	int r;

	crtc = priv->pipes[wb_inputs[dev->input].crtc_index].crtc;
	ct = omap_crtc_timings(crtc);

	/* configure wb */
	r = priv->dispc_ops->wb_setup(priv->dispc, wb_info, false, ct, wb_channel);
	if (r)
		return false;

	if (is_input_active(dev)) {
		priv->dispc_ops->ovl_enable(priv->dispc, OMAP_DSS_WB, true);
		priv->dispc_ops->wb_go(priv->dispc);
	} else {
		log_err(dev, "CHANNEL %u not enabled, skip WB GO\n",
			wb_inputs[dev->input].omap_channel);
	}

	return true;
}

static bool is_input_irq_vsync_set(struct wbcap_dev *dev, u32 irqstatus)
{
	struct omap_drm_private *priv = dev->dev->drm_dev->dev_private;
	u32 oc = wb_inputs[dev->input].omap_channel;

	if (irqstatus & priv->dispc_ops->mgr_get_vsync_irq(priv->dispc, oc))
		return true;
	return false;
}

static int wbcap_schedule_next_buffer(struct wbcap_dev *dev)
{
	struct wb_buffer *buf;
	unsigned long addr_y = 0;
	unsigned long addr_uv = 0;
	struct wb_q_data *q_data;
	int num_planes;
	bool ok;
	struct omap_dss_writeback_info wb_info = { 0 };
	struct v4l2_pix_format_mplane *pix;
	unsigned long flags;

	if (!is_input_active(dev)) {
		dev->next_frm = NULL;
		return 0;
	}

	spin_lock_irqsave(&dev->qlock, flags);
	if (list_empty(&dev->buf_list)) {
		dev->next_frm = NULL;
		spin_unlock_irqrestore(&dev->qlock, flags);
		return 0;
	}

	buf = list_entry(dev->buf_list.next, struct wb_buffer, list);
	dev->next_frm = buf;
	list_del(&buf->list);
	spin_unlock_irqrestore(&dev->qlock, flags);

	q_data = get_q_data(dev, buf->vb.vb2_buf.type);
	if (!q_data)
		return -EINVAL;

	pix = &q_data->format.fmt.pix_mp;
	num_planes = pix->num_planes;

	addr_y = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
	if (num_planes == 2)
		addr_uv = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 1);
	else if (pix->pixelformat == V4L2_PIX_FMT_NV12)
		addr_uv = addr_y + (pix->plane_fmt[0].bytesperline *
				    pix->height);

	/* fill WB DSS info */
	wb_info.paddr = (u32)addr_y;
	wb_info.p_uv_addr = (u32)addr_uv;
	wb_info.buf_width = pix->plane_fmt[0].bytesperline /
			    (q_data->fmt->depth[LUMA_PLANE] / 8);

	wb_info.width = pix->width;
	wb_info.height = pix->height;
	wb_info.fourcc = omap_wb_fourcc_v4l2_to_drm(pix->pixelformat);
	wb_info.pre_mult_alpha = 1;

	wb_info.rotation = DRM_MODE_ROTATE_0;
	wb_info.rotation_type = OMAP_DSS_ROT_NONE;

	ok = wb_cap_setup(dev,
			  wb_inputs[dev->input].wb_channel,
			  &wb_info);
	if (!ok)
		return -EINVAL;

	return 0;
}

static void wbcap_process_buffer_complete(struct wbcap_dev *dev)
{
	dev->cur_frm->vb.vb2_buf.timestamp = ktime_get_ns();
	dev->cur_frm->vb.field = dev->field;
	dev->cur_frm->vb.sequence = dev->sequence++;

	vb2_buffer_done(&dev->cur_frm->vb.vb2_buf, VB2_BUF_STATE_DONE);
	dev->cur_frm = dev->next_frm;
}

static enum hrtimer_restart wbcap_wbgo_timer(struct hrtimer *timer)
{
	struct wbcap_dev *dev = container_of(timer,
					     struct wbcap_dev, wbgo_timer);
	struct omap_drm_private *priv = dev->dev->drm_dev->dev_private;

	if (priv->dispc_ops->wb_go_busy(priv->dispc))
		log_err(dev, "WARNING, WB BUSY at hrtimer, state %u\n",
			dev->state);

	switch (dev->state) {
	case WB_STATE_NONE:
		break;

	case WB_STATE_FIRST_FRAME:
		dev->cur_frm = dev->next_frm;
		wbcap_schedule_next_buffer(dev);
		dev->state = WB_STATE_CAPTURING;
		break;

	case WB_STATE_CAPTURING:
		if (dev->cur_frm && dev->next_frm) {
			/*
			 * We have cur_frm that was just captured, and next_frm
			 * to which the HW will start capturing.
			 * This means cur_frm is now released from DSS HW.
			 */
			wbcap_process_buffer_complete(dev);
			dev->next_frm = NULL;
		} else {
			/*
			 * We have cur_frm which has a captured frame,
			 * but we don't have next_frm.
			 * This means cur_frm is will still be used by
			 * DSS for capture
			 */
		}

		if (dev->stopping) {
			/* XXX should we set WB GO? */
			priv->dispc_ops->ovl_enable(priv->dispc, OMAP_DSS_WB,
						    false);
			dev->state = WB_STATE_STOPPING;
		} else {
			wbcap_schedule_next_buffer(dev);
		}
		break;

	case WB_STATE_STOPPING:
		if (dev->cur_frm)
			wbcap_process_buffer_complete(dev);

		dev->state = WB_STATE_STOPPED;
		atomic_dec(&dev->dev->irq_enabled);
		dev->stopping = false;
		wake_up(&dev->event);
		break;

	case WB_STATE_STOPPED:
		log_err(dev, "ERROR: timer triggered in the stopped state. This shouldn't happen\n");
		break;
	}

	return HRTIMER_NORESTART;
}

static void wbcap_handle_vsync(struct wbcap_dev *dev)
{
	/*
	 * In writeback capture mode, the GO bit doesn't get reset
	 * at the manager's VSYNC interrupt. It takes an extra
	 * 'WBDELAYCOUNTER' time after VSYNC when the writeback
	 * FIFOs are flushed and the shadow registers are taken in.
	 * There isn't any DSS interrupt to notify this point in time.
	 * The correct solution is to set a timer far enough that it
	 * should cover the period defined by WBDELAYCOUNTER.
	 * The max value allowed in WBDELAYCOUNTER is 255 which
	 * correspond to 255 lines. So waiting anywhere from 1/4 to
	 * 1/2 a frame (i.e. 2ms at 60  to 120 fps) should be safe
	 * enough.
	 */

	hrtimer_start_range_ns(&dev->wbgo_timer, ms_to_ktime(3), 1000000,
			       HRTIMER_MODE_REL);
}

void wbcap_irq(struct wbcap_dev *dev, u32 irqstatus)
{
	if (irqstatus & DISPC_IRQ_FRAMEDONEWB)
		log_dbg(dev, "WB: FRAMEDONE\n");

	if (irqstatus & DISPC_IRQ_WBBUFFEROVERFLOW)
		log_err(dev, "WB: UNDERFLOW\n");

	if (irqstatus & DISPC_IRQ_WBUNCOMPLETEERROR)
		log_err(dev, "WB: WBUNCOMPLETEERROR\n");

	if (is_input_irq_vsync_set(dev, irqstatus)) {
		if (dev->field != V4L2_FIELD_NONE) {
			if (irqstatus & DISPC_IRQ_EVSYNC_EVEN)
				dev->field = V4L2_FIELD_BOTTOM;
			else if (irqstatus & DISPC_IRQ_EVSYNC_ODD)
				dev->field = V4L2_FIELD_TOP;
		}
		wbcap_handle_vsync(dev);
	}
}

/*
 * Setup the constraints of the queue: besides setting the number of planes
 * per buffer and the size and allocation context of each plane, it also
 * checks if sufficient buffers have been allocated. Usually 3 is a good
 * minimum number: many DMA engines need a minimum of 2 buffers in the
 * queue and you need to have another available for userspace processing.
 */
static int queue_setup(struct vb2_queue *vq,
		       unsigned int *nbuffers, unsigned int *nplanes,
		       unsigned int sizes[], struct device *alloc_devs[])
{
	int i;
	struct wbcap_dev *wbcap = vb2_get_drv_priv(vq);
	struct wb_q_data *q_data;

	q_data = get_q_data(wbcap, vq->type);

	if (!q_data)
		return -EINVAL;

	if (vq->num_buffers + *nbuffers < 2)
		*nbuffers = 2 - vq->num_buffers;

	*nplanes = q_data->format.fmt.pix_mp.num_planes;

	for (i = 0; i < *nplanes; i++)
		sizes[i] = q_data->format.fmt.pix_mp.plane_fmt[i].sizeimage;

	log_dbg(wbcap, "get %d buffer(s) of size %d\n", *nbuffers,
		sizes[LUMA_PLANE]);
	if (*nplanes == 2)
		log_dbg(wbcap, " and %d\n", sizes[CHROMA_PLANE]);

	return 0;
}

/*
 * Prepare the buffer for queueing to the DMA engine: check and set the
 * payload size.
 */
static int buffer_prepare(struct vb2_buffer *vb)
{
	struct wbcap_dev *wbcap = vb2_get_drv_priv(vb->vb2_queue);
	struct wb_q_data *q_data;
	struct v4l2_pix_format_mplane *mp;
	int i, num_planes;

	q_data = get_q_data(wbcap, vb->vb2_queue->type);
	if (!q_data)
		return -EINVAL;
	num_planes = q_data->format.fmt.pix_mp.num_planes;

	for (i = 0; i < num_planes; i++) {
		mp = &q_data->format.fmt.pix_mp;
		if (vb2_plane_size(vb, i) < mp->plane_fmt[i].sizeimage) {
			log_err(wbcap,
				"data will not fit into plane (%lu < %lu)\n",
				vb2_plane_size(vb, i),
				(long)mp->plane_fmt[i].sizeimage);
			return -EINVAL;
		}
		vb2_set_plane_payload(vb, i, mp->plane_fmt[i].sizeimage);
	}

	return 0;
}

/*
 * Queue this buffer to the DMA engine.
 */
static void buffer_queue(struct vb2_buffer *vb)
{
	struct wbcap_dev *wbcap = vb2_get_drv_priv(vb->vb2_queue);
	struct wb_buffer *buf = to_wb_buffer(vb);
	unsigned long flags;

	spin_lock_irqsave(&wbcap->qlock, flags);
	list_add_tail(&buf->list, &wbcap->buf_list);

	spin_unlock_irqrestore(&wbcap->qlock, flags);
}

static void return_all_buffers(struct wbcap_dev *wbcap,
			       enum vb2_buffer_state state)
{
	struct wb_buffer *buf, *node;
	unsigned long flags;

	spin_lock_irqsave(&wbcap->qlock, flags);
	list_for_each_entry_safe(buf, node, &wbcap->buf_list, list) {
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		list_del(&buf->list);
	}

	if (wbcap->cur_frm) {
		vb2_buffer_done(&wbcap->cur_frm->vb.vb2_buf, state);
		wbcap->cur_frm = NULL;
	}

	if (wbcap->next_frm) {
		vb2_buffer_done(&wbcap->next_frm->vb.vb2_buf, state);
		wbcap->next_frm = NULL;
	}

	spin_unlock_irqrestore(&wbcap->qlock, flags);
}

/*
 * Start streaming. First check if the minimum number of buffers have been
 * queued. If not, then return -ENOBUFS and the vb2 framework will call
 * this function again the next time a buffer has been queued until enough
 * buffers are available to actually start the DMA engine.
 */
static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct wbcap_dev *wbcap = vb2_get_drv_priv(vq);
	struct omap_drm_private *priv = wbcap->dev->drm_dev->dev_private;
	struct drm_crtc *crtc;
	int ret;
	struct wb_q_data *q_data;

	priv->dispc_ops->runtime_get(priv->dispc);

	wbcap->sequence = 0;
	q_data = get_q_data(wbcap, wbcap->queue.type);
	if (!q_data) {
		log_err(wbcap, "ERROR: getting q_data failed\n");
		return_all_buffers(wbcap, VB2_BUF_STATE_QUEUED);
		priv->dispc_ops->runtime_put(priv->dispc);
		return -EINVAL;
	}

	if (q_data->format.fmt.pix_mp.field == V4L2_FIELD_ALTERNATE)
		wbcap->field = V4L2_FIELD_TOP;
	else
		wbcap->field = V4L2_FIELD_NONE;

	log_dbg(wbcap, "Input (%s) is %s : %s\n",
		wb_inputs[wbcap->input].name,
		is_input_enabled(wbcap) ? "enabled" : "disabled",
		is_input_active(wbcap) ? "active" : "inactive");

	if (!is_input_active(wbcap)) {
		log_err(wbcap, "ERROR: Selected input (%s) is not active, bailing out\n",
			wb_inputs[wbcap->input].name);
		return_all_buffers(wbcap, VB2_BUF_STATE_QUEUED);
		priv->dispc_ops->runtime_put(priv->dispc);
		return -EINVAL;
	}

	/* Enable vsync irq on the input crtc */
	crtc = priv->pipes[wb_inputs[wbcap->input].crtc_index].crtc;
	ret = drm_crtc_vblank_get(crtc);
	WARN_ON(ret != 0);

	if (wbcap_schedule_next_buffer(wbcap)) {
		return_all_buffers(wbcap, VB2_BUF_STATE_QUEUED);
		priv->dispc_ops->runtime_put(priv->dispc);
		return -EINVAL;
	}

	wbcap->state = WB_STATE_FIRST_FRAME;
	atomic_inc(&wbcap->dev->irq_enabled);
	return 0;
}

/*
 * Stop the DMA engine. Any remaining buffers in the DMA queue are dequeued
 * and passed on to the vb2 framework marked as STATE_ERROR.
 */
static void stop_streaming(struct vb2_queue *vq)
{
	struct wbcap_dev *wbcap = vb2_get_drv_priv(vq);
	struct omap_drm_private *priv = wbcap->dev->drm_dev->dev_private;
	struct drm_crtc *crtc;
	int ret;

	log_dbg(wbcap, "Stopping WB\n");
	log_dbg(wbcap, "current state: %d\n", wbcap->state);

	wbcap->stopping = true;
	ret = wait_event_timeout(wbcap->event,
				 !wbcap->stopping,
				 msecs_to_jiffies(250));

	log_dbg(wbcap, "Returning VB2 buffers\n");

	if (priv->dispc_ops->wb_go_busy(priv->dispc))
		log_err(wbcap, "WARNING, WB BUSY when stopping\n");

	/* Release all active buffers */
	return_all_buffers(wbcap, VB2_BUF_STATE_ERROR);

	/* Disable vsync irq on the input crtc */
	crtc = priv->pipes[wb_inputs[wbcap->input].crtc_index].crtc;
	drm_crtc_vblank_put(crtc);

	priv->dispc_ops->runtime_put(priv->dispc);
}

/*
 * The vb2 queue ops. Note that since q->lock is set we can use the standard
 * vb2_ops_wait_prepare/finish helper functions. If q->lock would be NULL,
 * then this driver would have to provide these ops.
 */
static struct vb2_ops wbcap_qops = {
	.queue_setup		= queue_setup,
	.buf_prepare		= buffer_prepare,
	.buf_queue		= buffer_queue,
	.start_streaming	= start_streaming,
	.stop_streaming		= stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

/*
 * Required ioctl querycap. Note that the version field is prefilled with
 * the version of the kernel.
 */
static int wbcap_querycap(struct file *file, void *priv,
			  struct v4l2_capability *cap)
{
	struct wbcap_dev *wbcap = video_drvdata(file);

	strlcpy(cap->driver, WBCAP_MODULE_NAME, sizeof(cap->driver));
	strlcpy(cap->card, WBCAP_MODULE_NAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 wbcap->v4l2_dev.name);
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_READWRITE |
			   V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

/*
 * Helper function to check and correct struct v4l2_pix_format. It's used
 * not only in VIDIOC_TRY/S_FMT, but also elsewhere if changes to the SDTV
 * standard, HDTV timings or the video input would require updating the
 * current format.
 */
static int wbcap_fill_pix_format(struct wbcap_dev *wbcap,
				 struct v4l2_format *f)
{
	struct wb_fmt *fmt = find_format(f);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *plane_fmt;
	unsigned int w_align;
	int i, depth, depth_bytes;

	if (!fmt) {
		log_dbg(wbcap, "Fourcc format (0x%08x) invalid.\n",
			pix->pixelformat);
		fmt = &wb_formats[1];
	}

	/* we only allow V4L2_FIELD_NONE or V4L2_FIELD_ALTERNATE */
	if (pix->field != V4L2_FIELD_NONE &&
	    pix->field != V4L2_FIELD_ALTERNATE)
		pix->field = V4L2_FIELD_NONE;

	depth = fmt->depth[LUMA_PLANE];

	/*
	 * The line stride needs to be even is even.
	 * Special case is with YUV422 interleaved format an even number
	 * of pixels is required also.
	 */
	depth_bytes = depth >> 3;

	w_align = 0;
	if ((depth_bytes == 3) || (depth_bytes == 1))
		w_align = 1;
	else if ((depth_bytes == 2) &&
		 (fmt->fourcc == V4L2_PIX_FMT_YUYV ||
		  fmt->fourcc == V4L2_PIX_FMT_UYVY))
		w_align = 1;

	v4l_bound_align_image(&pix->width, MIN_W, MAX_W, w_align,
			      &pix->height, MIN_H, MAX_H, H_ALIGN,
			      S_ALIGN);
	pix->num_planes = fmt->coplanar ? 2 : 1;
	pix->pixelformat = fmt->fourcc;

	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	memset(pix->reserved, 0, sizeof(pix->reserved));
	for (i = 0; i < pix->num_planes; i++) {
		plane_fmt = &pix->plane_fmt[i];
		depth = fmt->depth[i];

		if (i == LUMA_PLANE)
			plane_fmt->bytesperline = pix->width * depth / 8;
		else
			plane_fmt->bytesperline = pix->width;

		plane_fmt->sizeimage = (pix->height * pix->width *
					depth) / 8;

		if (fmt->fourcc == V4L2_PIX_FMT_NV12) {
			/*
			 * Since we are using a single plane buffer
			 * we need to adjust the reported sizeimage
			 * to include the colocated UV part.
			 */
			plane_fmt->sizeimage += (pix->height / 2 *
				plane_fmt->bytesperline);
		}

		memset(plane_fmt->reserved, 0, sizeof(plane_fmt->reserved));
	}

	return 0;
}

static int wbcap_try_fmt_vid_cap(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct wbcap_dev *wbcap = video_drvdata(file);
	struct omap_drm_private *drmpriv = wbcap->dev->drm_dev->dev_private;
	struct drm_crtc *crtc;
	struct videomode *ct;

	log_dbg(wbcap, "requested fourcc:%4.4s size: %dx%d\n",
		(char *)&f->fmt.pix_mp.pixelformat,
		f->fmt.pix_mp.width, f->fmt.pix_mp.height);

	/*
	 * Scaling currently does not work properly for Capture mode.
	 * So we are temporarily forcing the frame size to be the
	 * same as the source crtc for now.
	 */
	crtc = drmpriv->pipes[wb_inputs[wbcap->input].crtc_index].crtc;
	ct = omap_crtc_timings(crtc);

	f->fmt.pix.width = ct->hactive;
	f->fmt.pix.height = ct->vactive;

	if (ct->flags & DISPLAY_FLAGS_INTERLACED) {
		f->fmt.pix.height /= 2;
		f->fmt.pix_mp.field = V4L2_FIELD_ALTERNATE;
	}

	log_dbg(wbcap, "replied fourcc:%4.4s size: %dx%d\n",
		(char *)&f->fmt.pix_mp.pixelformat,
		f->fmt.pix_mp.width, f->fmt.pix_mp.height);

	return wbcap_fill_pix_format(wbcap, f);
}

static int wbcap_s_fmt_vid_cap(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct wbcap_dev *wbcap = video_drvdata(file);
	int ret;
	struct wb_q_data *q_data;

	log_dbg(wbcap, "type:%d\n", f->type);

	ret = wbcap_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	q_data = get_q_data(wbcap, f->type);
	if (!q_data)
		return -EINVAL;

	/*
	 * It is not allowed to change the format while buffers for use with
	 * streaming have already been allocated.
	 */
	if (vb2_is_busy(&wbcap->queue))
		return -EBUSY;

	q_data->format = *f;
	q_data->fmt = find_format(f);

	log_dbg(wbcap, "Setting format for type %d, %dx%d, fmt: %4.4s bpl_y %d",
		f->type, f->fmt.pix_mp.width, f->fmt.pix_mp.height,
		(char *)&f->fmt.pix_mp.pixelformat,
		f->fmt.pix_mp.plane_fmt[LUMA_PLANE].bytesperline);
	if (f->fmt.pix_mp.num_planes == 2)
		log_dbg(wbcap, " bpl_uv %d\n",
			f->fmt.pix_mp.plane_fmt[CHROMA_PLANE].bytesperline);

	return 0;
}

static int wbcap_g_fmt_vid_cap(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct wbcap_dev *wbcap = video_drvdata(file);
	struct wb_q_data *q_data;

	log_dbg(wbcap, "type:%d\n", f->type);

	q_data = get_q_data(wbcap, f->type);
	if (!q_data)
		return -EINVAL;

	*f = q_data->format;
	return 0;
}

static int wbcap_enum_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_fmtdesc *f)
{
	if (f->index >= num_wb_formats)
		return -EINVAL;

	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	f->pixelformat = wb_formats[f->index].fourcc;
	return 0;
}

static int wbcap_enum_input(struct file *file, void *priv,
			    struct v4l2_input *i)
{
	if (i->index >= num_wb_input)
		return -EINVAL;

	i->type = V4L2_INPUT_TYPE_CAMERA;
	strlcpy(i->name, wb_inputs[i->index].name, sizeof(i->name));
	return 0;
}

static int wbcap_s_input(struct file *file, void *priv, unsigned int i)
{
	struct wbcap_dev *wbcap = video_drvdata(file);
	struct wb_q_data *q_data;

	log_dbg(wbcap, "%d\n", i);

	q_data = get_q_data(wbcap, wbcap->queue.type);
	if (!q_data)
		return -EINVAL;

	if (i >= num_wb_input)
		return -EINVAL;

	/*
	 * Changing the input implies a format change, which is not allowed
	 * while buffers for use with streaming have already been allocated.
	 */
	if (vb2_is_busy(&wbcap->queue))
		return -EBUSY;

	wbcap->input = i;

	/* Update the internal format to match the selected input */
	wbcap_try_fmt_vid_cap(file, priv, &q_data->format);
	return 0;
}

static int wbcap_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct wbcap_dev *wbcap = video_drvdata(file);

	log_dbg(wbcap, "%d\n", wbcap->input);

	*i = wbcap->input;
	return 0;
}

/*
 * File operations
 */
static int wbcap_open(struct file *file)
{
	struct wbcap_dev *dev = video_drvdata(file);
	int ret;

	log_dbg(dev, "enter\n");

	if (mutex_lock_interruptible(&dev->dev->lock)) {
		ret = -ERESTARTSYS;
		goto unlock;
	}

	if ((dev->dev->mode != OMAP_WB_NOT_CONFIGURED) &&
	    (dev->dev->mode != OMAP_WB_CAPTURE_MGR)) {
		/* WB is already open for other modes */
		ret = -EBUSY;
		goto unlock;
	}

	ret = v4l2_fh_open(file);
	if (ret) {
		log_err(dev, "v4l2_fh_open failed\n");
		goto unlock;
	}

	if (v4l2_fh_is_singular_file(file))
		dev->dev->mode = OMAP_WB_CAPTURE_MGR;

unlock:
	mutex_unlock(&dev->dev->lock);
	return ret;
}

static int wbcap_release(struct file *file)
{
	struct wbcap_dev *dev = video_drvdata(file);
	bool fh_singular;
	int ret;

	log_dbg(dev, "releasing\n");

	mutex_lock(&dev->dev->lock);

	/* Save the singular status before we call the clean-up helper */
	fh_singular = v4l2_fh_is_singular_file(file);

	/* the release helper will cleanup any on-going streaming */
	ret = _vb2_fop_release(file, NULL);

	if (fh_singular)
		dev->dev->mode = OMAP_WB_NOT_CONFIGURED;

	mutex_unlock(&dev->dev->lock);

	return ret;
}

/*
 * The set of all supported ioctls. Note that all the streaming ioctls
 * use the vb2 helper functions that take care of all the locking and
 * that also do ownership tracking (i.e. only the filehandle that requested
 * the buffers can call the streaming ioctls, all other filehandles will
 * receive -EBUSY if they attempt to call the same streaming ioctls).
 *
 * The last three ioctls also use standard helper functions: these implement
 * standard behavior for drivers with controls.
 */
static const struct v4l2_ioctl_ops wbcap_ioctl_ops = {
	.vidioc_querycap = wbcap_querycap,
	.vidioc_try_fmt_vid_cap_mplane = wbcap_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap_mplane = wbcap_s_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap_mplane = wbcap_g_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap_mplane = wbcap_enum_fmt_vid_cap,

	.vidioc_enum_input = wbcap_enum_input,
	.vidioc_g_input = wbcap_g_input,
	.vidioc_s_input = wbcap_s_input,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,

	.vidioc_log_status = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/*
 * The set of file operations. Note that all these ops are standard core
 * helper functions.
 */
static const struct v4l2_file_operations wbcap_fops = {
	.owner = THIS_MODULE,
	.open = wbcap_open,
	.release = wbcap_release,
	.unlocked_ioctl = video_ioctl2,
	.read = vb2_fop_read,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

/*
 * The initial setup of this device instance. Note that the initial state of
 * the driver should be complete. So the initial format, standard, timings
 * and video input should all be initialized to some reasonable value.
 */
int wbcap_init(struct wb_dev *dev)
{
	struct wbcap_dev *wbcap;
	struct video_device *vdev;
	struct vb2_queue *q;
	struct wb_q_data *q_data;
	int ret;

	if (!dev)
		return -ENOMEM;

	/* Allocate a new instance */
	wbcap = devm_kzalloc(dev->drm_dev->dev, sizeof(*wbcap), GFP_KERNEL);
	if (!wbcap)
		return -ENOMEM;

	dev->cap = wbcap;
	wbcap->dev = dev;

	/* Fill in the initial format-related settings */
	q_data = &wbcap->q_data[Q_DATA_DST];
	q_data->fmt = &wb_formats[1];
	q_data->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q_data->format.fmt.pix_mp.pixelformat = q_data->fmt->fourcc;
	q_data->format.fmt.pix_mp.width = 1920;
	q_data->format.fmt.pix_mp.height = 1080;
	wbcap_fill_pix_format(wbcap, &q_data->format);

	/* Initialize the top-level structure */
	strlcpy(wbcap->v4l2_dev.name, WBCAP_MODULE_NAME,
		sizeof(wbcap->v4l2_dev.name));
	ret = v4l2_device_register(dev->drm_dev->dev, &wbcap->v4l2_dev);
	if (ret)
		return ret;

	/*
	 * This lock is now created by the main level.
	 * We might need one per sub structure in the future
	 *
	 *  mutex_init(&dev->lock);
	 */

	/* Initialize the vb2 queue */
	q = &wbcap->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	q->drv_priv = wbcap;
	q->buf_struct_size = sizeof(struct wb_buffer);
	q->ops = &wbcap_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->dev = wbcap->v4l2_dev.dev;

	/*
	 * Assume that this DMA engine needs to have at least two buffers
	 * available before it can be started. The start_streaming() op
	 * won't be called until at least this many buffers are queued up.
	 */
	q->min_buffers_needed = 2;
	/*
	 * The serialization lock for the streaming ioctls. This is the same
	 * as the main serialization lock, but if some of the non-streaming
	 * ioctls could take a long time to execute, then you might want to
	 * have a different lock here to prevent VIDIOC_DQBUF from being
	 * blocked while waiting for another action to finish. This is
	 * generally not needed for PCI devices, but USB devices usually do
	 * want a separate lock here.
	 */
	q->lock = &dev->lock;
	/*
	 * Since this driver can only do 32-bit DMA we must make sure that
	 * the vb2 core will allocate the buffers in 32-bit DMA memory.
	 */
	q->gfp_flags = GFP_DMA32;
	ret = vb2_queue_init(q);
	if (ret)
		goto free_hdl;

	INIT_LIST_HEAD(&wbcap->buf_list);
	spin_lock_init(&wbcap->qlock);

	/* Initialize the video_device structure */
	vdev = &wbcap->vdev;
	strlcpy(vdev->name, WBCAP_MODULE_NAME, sizeof(vdev->name));
	/*
	 * There is nothing to clean up, so release is set to an empty release
	 * function. The release callback must be non-NULL.
	 */
	vdev->release = video_device_release_empty;
	vdev->fops = &wbcap_fops,
	vdev->ioctl_ops = &wbcap_ioctl_ops,
	/*
	 * The main serialization lock. All ioctls are serialized by this
	 * lock. Exception: if q->lock is set, then the streaming ioctls
	 * are serialized by that separate lock.
	 */
	vdev->lock = &dev->lock;
	vdev->queue = q;
	vdev->v4l2_dev = &wbcap->v4l2_dev;
	video_set_drvdata(vdev, wbcap);

	hrtimer_init(&wbcap->wbgo_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	wbcap->wbgo_timer.function = wbcap_wbgo_timer;

	init_waitqueue_head(&wbcap->event);
	wbcap->stopping = false;

	build_input_table(wbcap);

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, 11);
	if (ret)
		goto free_hdl;

	log_dbg(wbcap, "Device registered as %s\n",
		video_device_node_name(vdev));
	return 0;

free_hdl:
	v4l2_device_unregister(&wbcap->v4l2_dev);
	return ret;
}

void wbcap_cleanup(struct wb_dev *dev)
{
	log_dbg(dev, "Cleanup WB Capture\n");

	video_unregister_device(&dev->cap->vdev);
	v4l2_device_unregister(&dev->cap->v4l2_dev);
}
