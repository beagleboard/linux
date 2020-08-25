/*
 *  Virtual AVB ALSA sound device
 *
 *  Implementation:
 *  Copyright (c) by Niklas Wantrupp <niklaswantrupp@web.de>
 *
 *  Based on generic AVB Driver:
 *  Copyright (c) Indumathi Duraipandian <indu9086@gmail.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/uio.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/info.h>
#include <sound/initval.h>
#include <sound/hwdep.h>

#include <linux/netdevice.h>
#include <linux/ip.h>
#include <linux/in.h>
#include <linux/delay.h>
#include <linux/un.h>
#include <linux/unistd.h>
#include <linux/ctype.h>
#include <linux/hrtimer.h>

#include <asm/unistd.h>
#include <asm/div64.h>

#include <net/sock.h>
#include <net/tcp.h>
#include <net/inet_connection_sock.h>
#include <net/request_sock.h>

#include "avb.h"

MODULE_AUTHOR("Indumathi Duraipandian <indu9086@gmail.com>");
MODULE_DESCRIPTION("AVB soundcard");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,AVB soundcard}}");

#define AVB_DEBUG

static int index[SND_AVB_NUM_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SND_AVB_NUM_CARDS] = SNDRV_DEFAULT_STR;
static bool enable[SND_AVB_NUM_CARDS] = { 1,
					  [1 ...(SND_AVB_NUM_CARDS - 1)] = 0 };
static int pcm_substreams[SND_AVB_NUM_CARDS] = { 1 };
static int pcm_notify[SND_AVB_NUM_CARDS];

module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for avb soundcard.");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for avb soundcard.");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable this avb soundcard.");
module_param_array(pcm_substreams, int, NULL, 0444);
MODULE_PARM_DESC(pcm_substreams, "PCM substreams # (1-8) for avb driver.");
module_param_array(pcm_notify, int, NULL, 0444);
MODULE_PARM_DESC(pcm_notify,
		 "Break capture when PCM format/rate/channels changes.");

static struct avb_device avb_device;
static int numcards = 0;
static struct platform_device *avbdevices[SND_AVB_NUM_CARDS];

static int avb_playback_open(struct snd_pcm_substream *substream);
static int avb_playback_close(struct snd_pcm_substream *substream);
static int avb_playback_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params);
static int avb_playback_hw_free(struct snd_pcm_substream *substream);
static int avb_playback_prepare(struct snd_pcm_substream *substream);
static int avb_playback_trigger(struct snd_pcm_substream *substream, int cmd);
static snd_pcm_uframes_t
avb_playback_pointer(struct snd_pcm_substream *substream);
static int avb_playback_copy(struct snd_pcm_substream *substream, int channel,
			     unsigned long pos, void __user *dst,
			     unsigned long count);
#ifdef AVB_USE_HIGH_RES_TIMER
enum hrtimer_restart avb_avtp_timer(struct hrtimer *t);
#else
static void avb_avtp_timer(unsigned long arg);
#endif

static int avb_capture_open(struct snd_pcm_substream *substream);
static int avb_capture_close(struct snd_pcm_substream *substream);
static int avb_capture_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params);
static int avb_capture_hw_free(struct snd_pcm_substream *substream);
static int avb_capture_prepare(struct snd_pcm_substream *substream);
static int avb_capture_trigger(struct snd_pcm_substream *substream, int cmd);
static snd_pcm_uframes_t
avb_capture_pointer(struct snd_pcm_substream *substream);
static int avb_capture_copy(struct snd_pcm_substream *substream, int channel,
			    unsigned long pos, void __user *dst,
			    unsigned long count);
static int avb_avtp_listen(struct avb_card *card);

static void avb_wq_fn(struct work_struct *work);

static int avb_pcm_new(struct avb_card *avbc, int device, int substreams);

#ifdef CONFIG_PM_SLEEP

static int avb_suspend(struct device *pdev);
static int avb_resume(struct device *pdev);

#endif

static int avb_probe(struct platform_device *devptr);
static int avb_remove(struct platform_device *devptr);
static void avb_remove_all(void);
static int __init alsa_avb_init(void);
static void __exit alsa_avb_exit(void);

static SIMPLE_DEV_PM_OPS(avb_pm, avb_suspend, avb_resume);

static struct platform_driver avb_driver = {
	.probe		= avb_probe,
	.remove		= avb_remove,
	.driver		= {
		.name	= SND_AVB_DRIVER,
		.pm		= AVB_PM_OPS,
	},
};

static struct snd_pcm_ops avb_playback_ops = {
	.open			= avb_playback_open,
	.close			= avb_playback_close,
	.ioctl			= snd_pcm_lib_ioctl,
	.hw_params		= avb_playback_hw_params,
	.hw_free		= avb_playback_hw_free,
	.prepare		= avb_playback_prepare,
	.trigger		= avb_playback_trigger,
	.pointer		= avb_playback_pointer,
	.copy_user		= avb_playback_copy
};

static struct snd_pcm_ops avb_capture_ops = {
	.open 			= avb_capture_open,
	.close 			= avb_capture_close,
	.ioctl 			= snd_pcm_lib_ioctl,
	.hw_params		= avb_capture_hw_params,
	.hw_free 		= avb_capture_hw_free,
	.prepare 		= avb_capture_prepare,
	.trigger 		= avb_capture_trigger,
	.pointer 		= avb_capture_pointer,
	.copy_user 		= avb_capture_copy
};

static struct snd_pcm_hardware avb_playback_hw = {
	.info			= (SNDRV_PCM_INFO_INTERLEAVED |
				   SNDRV_PCM_INFO_BLOCK_TRANSFER),
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= SNDRV_PCM_RATE_8000_192000,
	.rate_min		= 8000,
	.rate_max		= 192000,
	.channels_min		= 1,
	.channels_max		= 8,
	.buffer_bytes_max 	= 131072,
	.period_bytes_min	= 16384,
	.period_bytes_max	= 131072,
	.periods_min		= 2,
	.periods_max 		= 4,
};

static struct snd_pcm_hardware avb_capture_hw = {
	.info			= (SNDRV_PCM_INFO_INTERLEAVED |
				   SNDRV_PCM_INFO_BLOCK_TRANSFER),
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= SNDRV_PCM_RATE_8000_192000,
	.rate_min		= 8000,
	.rate_max		= 192000,
	.channels_min		= 1,
	.channels_max		= 8,
	.buffer_bytes_max	= 131072,
	.period_bytes_min	= 16384,
	.period_bytes_max	= 131072,
	.periods_min		= 2,
	.periods_max		= 4,
};

static struct work_data *avb_init_and_queue_work(int work_id, void *wdata,
						 int delay)
{
	struct work_data *wd;

	wd = (struct work_data *)kmalloc(sizeof(struct work_data), GFP_KERNEL);

	if (wd == NULL) {
		avb_log(AVB_KERN_ERR,
			KERN_ERR
			"avb_init_and_queue_work work_data allocation failed for work %d",
			work_id);
		return NULL;
	}

	wd->dw.data = wdata;
	wd->delayed_work_id = work_id;
	INIT_DELAYED_WORK((struct delayed_work *)wd, avb_wq_fn);

	queue_delayed_work(avb_device.wq, (struct delayed_work *)wd, delay);

	return wd;
}

static int avb_playback_open(struct snd_pcm_substream *substream)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_playback_open");

	substream->runtime->hw = avb_playback_hw;

	return 0;
}

static int avb_playback_close(struct snd_pcm_substream *substream)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_playback_close");

	return 0;
}

static int avb_playback_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *hw_params)
{
	struct avb_card *avb_card = snd_pcm_substream_chip(substream);

	avb_log(AVB_KERN_NOT,
		KERN_NOTICE "avb_playback_hw_params numbytes:%d sr:%d",
		params_buffer_bytes(hw_params), params_rate(hw_params));

	avb_card->tx.substream = substream;
	avb_card->tx.st = 0;
	avb_card->tx.sr = params_rate(hw_params);
	avb_card->tx.hw_idx = 0;
	avb_card->tx.seq_no = 0;
	avb_card->tx.hwnw_idx = 0;
	avb_card->tx.fill_size = 0;
	avb_card->tx.last_timer_ts = jiffies;
	avb_card->tx.socket_count = 0;
	avb_card->tx.pending_tx_frames = 0;
	avb_card->tx.num_bytes_consumed = 0;
	avb_card->tx.period_size = params_period_size(hw_params);
	avb_card->tx.buffer_size = params_buffer_bytes(hw_params);
	avb_card->tx.frame_count = params_buffer_size(hw_params);
	avb_card->tx.frame_size =
		params_buffer_bytes(hw_params) / params_buffer_size(hw_params);

	avb_avtp_aaf_header_init(&avb_card->sd.tx_buf[0], substream, hw_params);

#ifdef AVB_USE_HIGH_RES_TIMER
	hrtimer_init((struct hrtimer *)&avb_device.txTimer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	avb_device.txTimer.timer.function = &avb_avtp_timer;
	avb_device.txTimer.card = avb_card;
#else
	init_timer(&avb_device.txTimer);
	avb_device.txTimer.data = (unsigned long)avb_card;
	avb_device.txTimer.function = avb_avtp_timer;
	avb_device.txTimer.expires = jiffies + 1;
	add_timer(&avb_device.txTimer);
#endif

	memset(&avb_device.tx_ts[0], 0, (sizeof(int) * AVB_MAX_TS_SLOTS));
	avb_device.tx_idx = 0;

	avb_card->tx.tmp_buf = kmalloc(avb_card->tx.buffer_size, GFP_KERNEL);

	return 0;
}

static int avb_playback_hw_free(struct snd_pcm_substream *substream)
{
	struct avb_card *avb_card = snd_pcm_substream_chip(substream);

	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_playback_hw_free");

#ifdef AVB_USE_HIGH_RES_TIMER
	hrtimer_try_to_cancel((struct hrtimer *)&avb_device.txTimer);
#else
	del_timer(&avb_device.txTimer);
#endif
	kfree(avb_card->tx.tmp_buf);

	return 0;
}

static int avb_playback_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static int avb_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	ktime_t kt;
	int ret = 0;
	int avtp_max_frames_per_packet = 0;
	struct avb_card *avb_card = snd_pcm_substream_chip(substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		avb_card->tx.st = 1;
		avb_log(AVB_KERN_NOT,
			KERN_NOTICE "avb_playback_trigger: Start @ %lu",
			jiffies);
#ifdef AVB_USE_HIGH_RES_TIMER
		avtp_max_frames_per_packet =
			((ETH_DATA_LEN - sizeof(struct avt_pdu_aaf_pcm_hdr)) /
			 avb_card->tx.frame_size);
		avb_card->tx.timer_val =
			((avtp_max_frames_per_packet * 1000000u) /
			 (avb_card->tx.sr / 1000));
		kt = ktime_set(0, avb_card->tx.timer_val);
		hrtimer_start((struct hrtimer *)&avb_device.txTimer, kt,
			      HRTIMER_MODE_REL);
#else
		avb_card->tx.start_ts = jiffies;
		if ((avb_card->tx.pending_tx_frames > 0) &&
		    (!timer_pending(&avb_device.txTimer))) {
			avb_device.txTimer.expires = jiffies + 1;
			add_timer(&avb_device.txTimer);
		}
#endif
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		avb_log(AVB_KERN_NOT,
			KERN_NOTICE "avb_playback_trigger: Stop @ %lu",
			jiffies);
		avb_card->tx.st = 0;
#ifdef AVB_USE_HIGH_RES_TIMER
		hrtimer_try_to_cancel((struct hrtimer *)&avb_device.txTimer);
#endif
		break;
	default:
		avb_log(AVB_KERN_WARN,
			KERN_WARNING "avb_playback_trigger: Unknown");
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t
avb_playback_pointer(struct snd_pcm_substream *substream)
{
	struct avb_card *avb_card = snd_pcm_substream_chip(substream);

	avb_log(AVB_KERN_INFO,
		KERN_INFO
		"avb_playback_pointer hw_idx:%lu numBytes:%lu, time: %u us",
		avb_card->tx.hw_idx, avb_card->tx.num_bytes_consumed,
		jiffies_to_usecs(jiffies - avb_card->tx.start_ts));

	return avb_card->tx.hw_idx;
}

static int avb_playback_copy(struct snd_pcm_substream *substream, int channel,
			     unsigned long pos, void __user *dst,
			     unsigned long count)
{
	int err = 0;
	struct avb_card *avb_card = snd_pcm_substream_chip(substream);
	count = bytes_to_frames(substream->runtime, count);
	pos = bytes_to_frames(substream->runtime, pos);

	avb_log(AVB_KERN_INFO,
		KERN_INFO "avb_playback_copy: ch:%d, pos: %ld, count: %lu",
		channel, pos, count);

	if ((err = copy_from_user(
		     &avb_card->tx.tmp_buf[(pos * avb_card->tx.frame_size)],
		     dst, (count * avb_card->tx.frame_size))) != 0) {
		avb_log(AVB_KERN_WARN,
			KERN_WARNING
			"avb_playback_copy copy from user fails: %d \n",
			err);
		return -1;
	}

	avb_card->tx.pending_tx_frames += count;
	avb_card->tx.num_bytes_consumed +=
		(count * (substream->runtime->frame_bits / 8));

#ifndef AVB_USE_HIGH_RES_TIMER
	if (avb_card->tx.st == 1) {
		avb_avtp_timer((unsigned long)avb_card);
	}
	if ((avb_card->tx.pending_tx_frames > 0) &&
	    (!timer_pending(&avb_device.txTimer) && (avb_card->tx.st == 1))) {
		avb_device.txTimer.expires = jiffies + 1;
		add_timer(&avb_device.txTimer);
	}
#endif

	return 0;
}

#ifdef AVB_USE_HIGH_RES_TIMER
enum hrtimer_restart avb_avtp_timer(struct hrtimer *t)
#else
static void avb_avtp_timer(unsigned long arg)
#endif
{
	ktime_t kt;
	int i = 0;
	int err = 0;
	int tx_size = 0;
	snd_pcm_uframes_t bytes_avai = 0;
	snd_pcm_uframes_t bytes_to_copy = 0;
	snd_pcm_uframes_t frames_to_copy = 0;
	snd_pcm_uframes_t avtp_frames_per_packet = 0;
	snd_pcm_uframes_t avtp_max_frames_per_packet = 0;
	enum hrtimer_restart hr_res = HRTIMER_NORESTART;
#ifdef AVB_USE_HIGH_RES_TIMER
	struct avb_card *avb_card = ((struct avb_hrtimer *)t)->card;
#else
	struct avb_card *avb_card = (struct avb_card *)arg;
#endif
	struct avt_pdu_aaf_pcm_hdr *hdr =
		(struct avt_pdu_aaf_pcm_hdr *)&avb_card->sd
			.tx_buf[sizeof(struct ethhdr)];
#ifndef AVB_USE_HIGH_RES_TIMER
	unsigned long int numJiffies =
		((jiffies > avb_card->tx.last_timer_ts) ?
			 (jiffies - avb_card->tx.last_timer_ts) :
			 (1));
	snd_pcm_uframes_t frameCount = ((avb_card->tx.sr * numJiffies) / HZ);
#endif

	avtp_max_frames_per_packet =
		((ETH_DATA_LEN - sizeof(struct avt_pdu_aaf_pcm_hdr)) /
		 avb_card->tx.frame_size);

#ifdef AVB_USE_HIGH_RES_TIMER
	avb_card->tx.accum_frame_count += avtp_max_frames_per_packet;
	avtp_frames_per_packet = avtp_max_frames_per_packet;
	kt = ktime_set(0, avb_card->tx.timer_val);
	avb_log(AVB_KERN_INFO,
		KERN_INFO
		"avb_avtp_timer mfppk: %lu, fppk: %lu, frSz: %lu, sr: %lu, time: %lu",
		avtp_max_frames_per_packet, avtp_frames_per_packet,
		avb_card->tx.frame_size, avb_card->tx.sr,
		avb_card->tx.timer_val);
#else
	avb_card->tx.accum_frame_count += frameCount;
	avtp_frames_per_packet = (avb_card->tx.sr / HZ);
	avtp_frames_per_packet =
		((avtp_frames_per_packet > avtp_max_frames_per_packet) ?
			 (avtp_max_frames_per_packet) :
			 (avtp_frames_per_packet));
	avb_log(AVB_KERN_INFO,
		KERN_INFO
		"avb_avtp_timer ct: %lu, mfppk: %lu, fppk: %lu, frSz: %lu, sr: %lu, HZ: %lu",
		frameCount, avtp_max_frames_per_packet, avtp_frames_per_packet,
		avb_card->tx.frame_size, avb_card->tx.sr, HZ);
#endif

	while (((avb_card->tx.accum_frame_count >= avtp_frames_per_packet) ||
		(avb_card->tx.pending_tx_frames <= avtp_frames_per_packet)) &&
#ifdef AVB_USE_HIGH_RES_TIMER
	       ((avb_card->tx.pending_tx_frames > 0) && (i < 1))) {
#else
	       ((avb_card->tx.pending_tx_frames > 0) && (frameCount > 0) &&
		(i < 32))) {
#endif
		i++; /* Just as a failsafe to quit loop */
		avb_card->tx.seq_no++;
		hdr->h.f.seq_no = avb_card->tx.seq_no;

		tx_size = sizeof(struct ethhdr) +
			  sizeof(struct avt_pdu_aaf_pcm_hdr);
		frames_to_copy = ((avb_card->tx.pending_tx_frames >
				   avtp_frames_per_packet) ?
					  (avtp_frames_per_packet) :
					  (avb_card->tx.pending_tx_frames));
		bytes_to_copy = (frames_to_copy * avb_card->tx.frame_size);

		bytes_avai = ((avb_card->tx.frame_count - avb_card->tx.hw_idx) *
			      avb_card->tx.frame_size);
		bytes_avai = ((bytes_avai >= bytes_to_copy) ? (bytes_to_copy) :
							      (bytes_avai));

		memcpy(&avb_card->sd.tx_buf[tx_size],
		       &avb_card->tx.tmp_buf[(avb_card->tx.hw_idx *
					      avb_card->tx.frame_size)],
		       bytes_avai);

		if (bytes_avai < bytes_to_copy) {
			memcpy(&avb_card->sd.tx_buf[tx_size + bytes_avai],
			       &avb_card->tx.tmp_buf[0],
			       (bytes_to_copy - bytes_avai));
		}

		hdr->h.f.avtp_ts = avb_device.tx_ts[(
			(avb_card->tx.hwnw_idx / avb_card->tx.period_size) %
			AVB_MAX_TS_SLOTS)];
		hdr->h.f.stream_data_len = bytes_to_copy;
		tx_size += bytes_to_copy;

		avb_card->sd.tx_iov.iov_base = avb_card->sd.tx_buf;
		avb_card->sd.tx_iov.iov_len = tx_size;
		iov_iter_init(&avb_card->sd.tx_msg_hdr.msg_iter, WRITE,
			      &avb_card->sd.tx_iov, 1, tx_size);

		if ((err = sock_sendmsg(avb_card->sd.sock,
					&avb_card->sd.tx_msg_hdr)) <= 0) {
			avb_log(AVB_KERN_WARN,
				KERN_WARNING
				"avb_avtp_timer Socket transmission fails %d \n",
				err);
			goto end;
		}

		avb_card->tx.accum_frame_count =
			((avb_card->tx.accum_frame_count > frames_to_copy) ?
				 (avb_card->tx.accum_frame_count -
				  frames_to_copy) :
				 (0));

		avb_card->tx.hw_idx += frames_to_copy;
		avb_card->tx.hwnw_idx += frames_to_copy;
		avb_card->tx.fill_size += frames_to_copy;
		avb_card->tx.hw_idx =
			((avb_card->tx.hw_idx < avb_card->tx.frame_count) ?
				 (avb_card->tx.hw_idx) :
				 (avb_card->tx.hw_idx %
				  avb_card->tx.frame_count));
		avb_card->tx.pending_tx_frames -= frames_to_copy;

		avb_log(AVB_KERN_INFO,
			KERN_INFO
			"avb_avtp_timer seq_no:%d, hw_idx: %lu, afrCt: %lu, penFrs:%lu, filSz:%lu",
			hdr->h.f.seq_no, avb_card->tx.hw_idx,
			avb_card->tx.accum_frame_count,
			avb_card->tx.pending_tx_frames, avb_card->tx.fill_size);

		if (avb_card->tx.fill_size >= avb_card->tx.period_size) {
			avb_card->tx.fill_size %= avb_card->tx.period_size;
			snd_pcm_period_elapsed(avb_card->tx.substream);
		}
	}

end:
#ifdef AVB_USE_HIGH_RES_TIMER
	if (avb_card->tx.st == 1) {
		hrtimer_forward_now(t, kt);
		hr_res = HRTIMER_RESTART;
	}

	return hr_res;
#else
	if ((avb_card->tx.pending_tx_frames > 0) &&
	    (!timer_pending(&avb_device.txTimer))) {
		avb_device.txTimer.expires = jiffies + 1;
		add_timer(&avb_device.txTimer);
	}
	avb_card->tx.last_timer_ts = jiffies;
#endif
}

static int avb_capture_open(struct snd_pcm_substream *substream)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_capture_open");

	substream->runtime->hw = avb_capture_hw;

	return 0;
}

static int avb_capture_close(struct snd_pcm_substream *substream)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_capture_close");

	return 0;
}

static int avb_capture_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *hw_params)
{
	struct avb_card *avb_card = snd_pcm_substream_chip(substream);

	avb_card->rx.substream = substream;
	avb_card->rx.hw_idx = 0;
	avb_card->rx.seq_no = 0;
	avb_card->rx.hwnw_idx = 0;
	avb_card->rx.fill_size = 0;
	avb_card->rx.prev_hw_idx = 0;
	avb_card->rx.socket_count = 0;
	avb_card->rx.num_bytes_consumed = 0;
	avb_card->rx.period_size = params_period_size(hw_params);
	avb_card->rx.buffer_size = params_buffer_bytes(hw_params);
	avb_card->rx.frame_count = params_buffer_size(hw_params);
	avb_card->rx.frame_size =
		params_buffer_bytes(hw_params) / params_buffer_size(hw_params);

	avb_log(AVB_KERN_NOT,
		KERN_NOTICE
		"avb_capture_hw_params buffer_size:%lu frame_size:%lu",
		avb_card->rx.buffer_size, avb_card->rx.frame_size);

	memset(&avb_device.rx_ts[0], 0, (sizeof(int) * AVB_MAX_TS_SLOTS));
	avb_device.rx_idx = 0;

	avb_device.avtpwd = avb_init_and_queue_work(AVB_DELAY_WORK_AVTP,
						    (void *)avb_card, 1);

	avb_card->rx.tmp_buf = kmalloc(avb_card->rx.buffer_size, GFP_KERNEL);

	return 0;
}

static int avb_capture_hw_free(struct snd_pcm_substream *substream)
{
	struct avb_card *avb_card = snd_pcm_substream_chip(substream);

	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_capture_hw_free");

	if (avb_device.avtpwd != NULL) {
		cancel_delayed_work((struct delayed_work *)avb_device.avtpwd);
		kfree(avb_device.avtpwd);
		avb_device.avtpwd = NULL;
	}

	kfree(avb_card->rx.tmp_buf);

	return 0;
}

static int avb_capture_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static int avb_capture_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;
	struct avb_card *avb_card = snd_pcm_substream_chip(substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		avb_log(AVB_KERN_NOT,
			KERN_NOTICE "avb_capture_trigger: Start @ %lu",
			jiffies);
		avb_card->rx.start_ts = jiffies;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_capture_trigger: Stop");
		break;
	default:
		avb_log(AVB_KERN_WARN,
			KERN_WARNING "avb_capture_trigger: Unknown");
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t
avb_capture_pointer(struct snd_pcm_substream *substream)
{
	struct avb_card *avb_card = snd_pcm_substream_chip(substream);

	avb_log(AVB_KERN_INFO,
		KERN_INFO "avb_capture_pointer hw_idx:%lu numBytes:%lu",
		avb_card->rx.hw_idx, avb_card->rx.num_bytes_consumed);

	return avb_card->rx.hw_idx;
}

static int avb_capture_copy(struct snd_pcm_substream *substream, int channel,
			    unsigned long pos, void __user *dst,
			    unsigned long count)
{
	char *src_buf;
	int copyres = 0;
	struct avb_card *avb_card = snd_pcm_substream_chip(substream);

	pos = bytes_to_frames(substream->runtime, pos);
	count = bytes_to_frames(substream->runtime, count);

	src_buf = (char *)&avb_card->rx.tmp_buf[pos * avb_card->rx.frame_size];

	copyres = copy_to_user(dst, src_buf, (count * avb_card->rx.frame_size));

	avb_log(AVB_KERN_INFO,
		KERN_INFO "avb_capture_copy: ch:%d, pos: %ld, ct: %ld, res: %d",
		channel, pos, count, copyres);

	// TODO: Check if frames to bytes is needed
	avb_card->rx.num_bytes_consumed += frames_to_bytes(
		substream->runtime, count * avb_card->rx.frame_size);

	return count;
}

static int avb_avtp_listen(struct avb_card *avb_card)
{
	int err = 0;
	char *src_buf;
	char *dest_buf;
	int rx_off = 0;
	int rx_size = 0;
	int nrx_size = 0;
	int avai_size = 0;
	int rx_frames = 0;
	int nrx_frames = 0;
	int next_seq_no = 0;
	int skipped_packets = 0;
	mm_segment_t oldfs;
	snd_pcm_uframes_t hw_idx = 0;
	struct kvec vec;
	struct avt_pdu_aaf_pcm_hdr *hdr =
		(struct avt_pdu_aaf_pcm_hdr *)&avb_card->sd
			.rx_buf[sizeof(struct ethhdr)];

	memset(avb_card->sd.rx_buf, 0, AVB_MAX_ETH_FRAME_SIZE);
	avb_card->sd.rx_iov.iov_base = avb_card->sd.rx_buf;
	avb_card->sd.rx_iov.iov_len = AVB_MAX_ETH_FRAME_SIZE;
	iov_iter_init(&avb_card->sd.rx_msg_hdr.msg_iter, READ,
		      &avb_card->sd.rx_iov, 1, AVB_MAX_ETH_FRAME_SIZE);

	vec.iov_len = AVB_MAX_ETH_FRAME_SIZE;
	vec.iov_base = avb_card->sd.rx_buf;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	err = kernel_recvmsg(avb_card->sd.sock, &avb_card->sd.rx_msg_hdr, &vec,
			     1, AVB_MAX_ETH_FRAME_SIZE, MSG_DONTWAIT);
	set_fs(oldfs);

	if (err > 0) {
		avb_card->rx.socket_count++;

		rx_off = sizeof(struct ethhdr) +
			 sizeof(struct avt_pdu_aaf_pcm_hdr);
		src_buf = (char *)&avb_card->sd.rx_buf[rx_off];

		rx_size = hdr->h.f.stream_data_len;
		rx_frames = (rx_size / avb_card->rx.frame_size);

		avb_device.rx_ts[(
			(avb_card->rx.hwnw_idx / avb_card->rx.period_size) %
			AVB_MAX_TS_SLOTS)] = hdr->h.f.avtp_ts;

		next_seq_no = (avb_card->rx.seq_no + 1) % 256;

		if (next_seq_no != hdr->h.f.seq_no) {
			avb_log(AVB_KERN_INFO,
				KERN_ERR
				"avb_listen missing frames from %d to %d \n",
				avb_card->rx.seq_no, hdr->h.f.seq_no);

			skipped_packets =
				((hdr->h.f.seq_no >= avb_card->rx.seq_no) ?
					 (hdr->h.f.seq_no -
					  avb_card->rx.seq_no) :
					 ((hdr->h.f.seq_no + 255) -
					  avb_card->rx.seq_no));
			nrx_frames = (skipped_packets * rx_frames);
			nrx_size = nrx_frames * avb_card->rx.frame_size;

			avb_log(AVB_KERN_INFO,
				KERN_INFO
				"avb_listen idx: %ld nrsz: %d, nrf: %d \n",
				avb_card->rx.hw_idx, nrx_size, nrx_frames);

			avai_size = ((avb_card->rx.frame_count -
				      avb_card->rx.hw_idx) *
				     avb_card->rx.frame_size);
			avai_size = ((avai_size < nrx_size) ? (avai_size) :
							      (nrx_size));
			dest_buf = (char *)&avb_card->rx
					   .tmp_buf[avb_card->rx.hw_idx *
						    avb_card->rx.frame_size];
			memset(dest_buf, 0, avai_size);

			if (avai_size < nrx_size) {
				dest_buf = (char *)&avb_card->rx.tmp_buf[0];
				memset(dest_buf, 0, (nrx_size - avai_size));
			}

			hw_idx = ((avb_card->rx.hw_idx + nrx_frames) %
				  (avb_card->rx.frame_count));
			rx_frames = rx_frames + nrx_frames;
		} else {
			hw_idx = avb_card->rx.hw_idx;
		}

		avb_log(AVB_KERN_INFO,
			KERN_INFO
			"avb_listen (%d) seq: %d, idx: %ld, sz: %d, ts: %u, rf: %d \n",
			avb_card->rx.socket_count, hdr->h.f.seq_no, hw_idx,
			rx_size, hdr->h.f.avtp_ts, rx_frames);

		avb_card->rx.seq_no = hdr->h.f.seq_no;

		avai_size = ((avb_card->rx.frame_count - hw_idx) *
			     avb_card->rx.frame_size);
		avai_size = ((avai_size < rx_size) ? (avai_size) : (rx_size));
		dest_buf = (char *)&avb_card->rx
				   .tmp_buf[hw_idx * avb_card->rx.frame_size];
		memcpy(dest_buf, src_buf, avai_size);

		if (avai_size < rx_size) {
			dest_buf = (char *)&avb_card->rx.tmp_buf[0];
			memcpy(dest_buf, &src_buf[avai_size],
			       (rx_size - avai_size));
		}
	} else {
		if (err != -11)
			avb_log(AVB_KERN_WARN,
				KERN_WARNING
				"avb_avtp_listen Socket reception fails %d \n",
				err);
		return 0;
	}

	return rx_frames;
}

static void avb_wq_fn(struct work_struct *work)
{
	int err = 0;
	int rx_count = 0;
	int fill_size = 0;
	int rx_frames = -1;
	int rx_loop_count = 0;
	struct work_data *wd = (struct work_data *)work;

	if (wd->delayed_work_id == AVB_DELAY_WORK_MSRP) {
		if (wd->dw.msrp->initialized == false)
			wd->dw.msrp->initialized = avb_msrp_init(wd->dw.msrp);

		if (wd->dw.msrp->initialized == false) {
			queue_delayed_work(
				avb_device.wq,
				(struct delayed_work *)avb_device.msrpwd,
				10000);
		} else {
			if (wd->dw.msrp->started == true) {
				do {
					err = avb_msrp_listen(wd->dw.msrp);
					rx_count += ((err > 0) ? (1) : (0));
				} while (err > 0);

				if (rx_count == 0) {
					if ((wd->dw.msrp->tx_state ==
					     MSRP_DECLARATION_STATE_NONE) ||
					    (wd->dw.msrp->tx_state ==
					     MSRP_DECLARATION_STATE_READY)) {
						avb_msrp_talkerdeclarations(
							wd->dw.msrp, true,
							wd->dw.msrp->tx_state);
					}
				} else {
					if ((wd->dw.msrp->tx_state ==
					     MSRP_DECLARATION_STATE_NONE) ||
					    (wd->dw.msrp->tx_state ==
					     MSRP_DECLARATION_STATE_UNKNOWN)) {
						avb_msrp_talkerdeclarations(
							wd->dw.msrp, true,
							wd->dw.msrp->tx_state);
					}

					if ((wd->dw.msrp->rx_state ==
					     MSRP_DECLARATION_STATE_NONE) ||
					    (wd->dw.msrp->rx_state ==
					     MSRP_DECLARATION_STATE_READY)) {
						avb_msrp_listenerdeclarations(
							wd->dw.msrp, true,
							wd->dw.msrp->rx_state);
					}

					wd->dw.msrp->tx_state =
						((wd->dw.msrp->tx_state ==
						  MSRP_DECLARATION_STATE_UNKNOWN) ?
							 (MSRP_DECLARATION_STATE_NONE) :
							 (wd->dw.msrp
								  ->tx_state));
					wd->dw.msrp->rx_state =
						((wd->dw.msrp->rx_state ==
						  MSRP_DECLARATION_STATE_UNKNOWN) ?
							 (MSRP_DECLARATION_STATE_NONE) :
							 (wd->dw.msrp
								  ->rx_state));
				}
			}

			avb_msrp_domaindeclarations(wd->dw.msrp);

			queue_delayed_work(
				avb_device.wq,
				(struct delayed_work *)avb_device.msrpwd,
				msecs_to_jiffies(2000));
		}
	} else if (wd->delayed_work_id == AVB_DELAY_WORK_AVDECC) {
		if (wd->dw.avdecc->initialized == false)
			wd->dw.avdecc->initialized =
				avb_avdecc_init(wd->dw.avdecc);

		if (wd->dw.avdecc->initialized == false) {
			queue_delayed_work(
				avb_device.wq,
				(struct delayed_work *)avb_device.avdeccwd,
				10000);
		} else {
			if (wd->dw.avdecc->last_ADP_adv_jiffy == 0)
				avb_adp_discover(wd->dw.avdecc);
			if (((jiffies - wd->dw.avdecc->last_ADP_adv_jiffy) >=
			     2000) ||
			    (wd->dw.avdecc->last_ADP_adv_jiffy == 0)) {
				avb_adp_advertise(wd->dw.avdecc);
				avb_maap_announce(wd->dw.avdecc);
				wd->dw.avdecc->last_ADP_adv_jiffy = jiffies;
			}
			avb_avdecc_listen_and_respond(wd->dw.avdecc,
						      &avb_device.msrp);
			queue_delayed_work(
				avb_device.wq,
				(struct delayed_work *)avb_device.avdeccwd, 1);
		}
	} else if (wd->delayed_work_id == AVB_DELAY_WORK_AVTP) {
		memcpy(&avb_device.card, wd->dw.card, sizeof(struct avb_card));

		do {
			rx_frames = avb_avtp_listen(&avb_device.card);

			if (rx_frames > 0) {
				avb_device.card.rx.hw_idx += rx_frames;
				avb_device.card.rx.hwnw_idx += rx_frames;
				avb_device.card.rx.hw_idx %=
					avb_device.card.rx.frame_count;

				if (avb_device.card.rx.hw_idx <
				    avb_device.card.rx.prev_hw_idx)
					fill_size =
						avb_device.card.rx.frame_count +
						avb_device.card.rx.prev_hw_idx -
						avb_device.card.rx.hw_idx;
				else
					fill_size =
						avb_device.card.rx.hw_idx -
						avb_device.card.rx.prev_hw_idx;

				avb_device.card.rx.prev_hw_idx =
					avb_device.card.rx.hw_idx;
				avb_device.card.rx.fill_size += fill_size;

				avb_log(AVB_KERN_INFO,
					KERN_INFO
					"avb_wq_fn: AVTP-%lu @ %lu rxFrms:%d hw_idx:%lu filSz: %lu",
					rx_loop_count++, jiffies, rx_frames,
					avb_device.card.rx.hw_idx,
					avb_device.card.rx.fill_size);

				if (avb_device.card.rx.fill_size >=
				    avb_device.card.rx.period_size) {
					avb_device.card.rx.fill_size %=
						avb_device.card.rx.period_size;
					snd_pcm_period_elapsed(
						avb_device.card.rx.substream);
				}
			} else {
				break;
			}

			memcpy(wd->dw.card, &avb_device.card,
			       sizeof(struct avb_card));

		} while (rx_frames > 0);

		if (avb_device.avtpwd != NULL) {
			queue_delayed_work(
				avb_device.wq,
				(struct delayed_work *)avb_device.avtpwd, 1);
		}
	} else {
		avb_log(AVB_KERN_INFO, KERN_INFO "avb_wq_fn: Unknown: %d",
			wd->delayed_work_id);
	}
}

static int avb_pcm_new(struct avb_card *avbc, int device, int substreams)
{
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(avbc->card, "AVB PCM", device, substreams, substreams,
			  &pcm);
	if (err < 0)
		return err;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &avb_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &avb_capture_ops);

	pcm->private_data = avbc;
	pcm->info_flags = 0;

	strcpy(pcm->name, "AVB PCM");

	avbc->pcm[device] = pcm;

	return 0;
}

static int avb_hwdep_open(struct snd_hwdep *hw, struct file *file)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_hwdep_open");

	return 0;
}

static int avb_hwdep_ioctl(struct snd_hwdep *hw, struct file *file,
			   unsigned int cmd, unsigned long arg)
{
	int res = 0;

	if (cmd == 0) {
		avb_log(AVB_KERN_INFO,
			KERN_INFO "avb_hwdep_ioctl set ts: %ld @ idx: %d", arg,
			avb_device.tx_idx);
		avb_device.tx_ts[avb_device.tx_idx] = arg;
		avb_device.tx_idx++;
		avb_device.tx_idx %= AVB_MAX_TS_SLOTS;
	} else {
		res = copy_to_user((void *)arg,
				   &avb_device.rx_ts[avb_device.rx_idx],
				   sizeof(unsigned long));
		avb_log(AVB_KERN_INFO,
			KERN_INFO "avb_hwdep_ioctl get ts: %d @ %d, res: %d",
			avb_device.rx_ts[avb_device.rx_idx], avb_device.rx_idx,
			res);
		avb_device.rx_idx++;
		avb_device.rx_idx %= AVB_MAX_TS_SLOTS;
	}

	return 0;
}

static int avb_hwdep_release(struct snd_hwdep *hw, struct file *file)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_hwdep_release");

	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int avb_suspend(struct device *pdev)
{
	avb_log(AVB_KERN_INFO, KERN_INFO "avb_suspend");
	return 0;
}

static int avb_resume(struct device *pdev)
{
	avb_log(AVB_KERN_INFO, KERN_INFO "avb_resume");
	return 0;
}

#endif

static int avb_probe(struct platform_device *devptr)
{
	struct snd_card *card;
	struct avb_card *avb_card;
	int dev = devptr->id;
	int err;

	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_probe");

	err = snd_card_new(&devptr->dev, index[dev], id[dev], THIS_MODULE,
			   sizeof(struct avb_card), &card);

	if (err < 0) {
		avb_log(AVB_KERN_ERR, KERN_ERR "avb_probe card new err: %d",
			err);
		return err;
	}

	avb_card = card->private_data;
	avb_card->card = card;

	err = avb_pcm_new(avb_card, 0, pcm_substreams[dev]);
	if (err < 0) {
		avb_log(AVB_KERN_ERR, KERN_ERR "avb_probe card pcm new err: %d",
			err);
		goto __nodev;
	}

	err = snd_hwdep_new(card, "avbhw", 0, &avb_device.hwdep);
	if (err < 0) {
		avb_log(AVB_KERN_ERR,
			KERN_ERR "avb_probe card hwdep new err: %d", err);
		goto __nodev;
	}

	avb_device.hwdep->ops.open = avb_hwdep_open;
	avb_device.hwdep->ops.ioctl = avb_hwdep_ioctl;
	avb_device.hwdep->ops.release = avb_hwdep_release;

	strcpy(card->driver, "avb");
	strcpy(card->shortname, "avb");
	sprintf(card->longname, "avb %i", dev + 1);
	err = snd_card_register(card);
	if (!err) {
		platform_set_drvdata(devptr, card);

		avb_card->sd.type = ETH_P_TSN;
		avb_card->sd.destmac[0] = 0x01;
		avb_card->sd.destmac[1] = 0x80;
		avb_card->sd.destmac[2] = 0xC2;
		avb_card->sd.destmac[3] = 0x00;
		avb_card->sd.destmac[4] = 0x00;
		avb_card->sd.destmac[5] = 0x0E;

		if (!avb_socket_init(&avb_card->sd, 100)) {
			avb_log(AVB_KERN_ERR,
				KERN_ERR "avb_probe socket init failed");
			err = -1;
			goto __nodev;
		}

		return 0;
	}

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_probe card reg err: %d", err);

__nodev:
	snd_card_free(card);

	return err;
}

static int avb_remove(struct platform_device *devptr)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_remove");
	snd_card_free(platform_get_drvdata(devptr));
	return 0;
}

static void avb_remove_all(void)
{
	int i = 0;

	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_remove_all");

	for (i = 0; i < numcards; i++)
		platform_device_unregister(avbdevices[i]);
}

static int __init alsa_avb_init(void)
{
	int i, err;
	struct platform_device *dev;
	avb_log(AVB_KERN_NOT, KERN_NOTICE "alsa_avb_init");

	err = platform_driver_register(&avb_driver);
	if (err < 0) {
		avb_log(AVB_KERN_ERR, KERN_ERR "alsa_avb_init reg err %d", err);
		return err;
	}

	for (i = 0; i < SND_AVB_NUM_CARDS; i++) {
		if (!enable[i])
			continue;

		dev = platform_device_register_simple(SND_AVB_DRIVER, i, NULL,
						      0);

		if (IS_ERR(dev)) {
			avb_log(AVB_KERN_ERR,
				KERN_ERR "alsa_avb_init regsimple err");
			continue;
		}

		if (!platform_get_drvdata(dev)) {
			avb_log(AVB_KERN_ERR,
				KERN_ERR "alsa_avb_init getdrvdata err");
			platform_device_unregister(dev);
			continue;
		}

		avbdevices[i] = dev;
		numcards++;
	}

	if (!numcards) {
		avb_remove_all();
	} else {
		memset(&avb_device, 0, sizeof(struct avb_device));

		avb_device.wq = create_workqueue(AVB_WQ);
		if (avb_device.wq == NULL) {
			avb_log(AVB_KERN_ERR, KERN_ERR
				"alsa_avb_init workqueue creation failed");
			return -1;
		}

		avb_device.msrpwd = avb_init_and_queue_work(
			AVB_DELAY_WORK_MSRP, (void *)&avb_device.msrp, 100);
		avb_device.avdeccwd = avb_init_and_queue_work(
			AVB_DELAY_WORK_AVDECC, (void *)&avb_device.avdecc, 100);

		avb_log(AVB_KERN_NOT,
			KERN_NOTICE "alsa_avb_init done err: %d, numcards: %d",
			err, numcards);
	}

	return 0;
}

static void __exit alsa_avb_exit(void)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "alsa_avb_exit");

	if (avb_device.msrpwd != NULL) {
		cancel_delayed_work((struct delayed_work *)avb_device.msrpwd);
		kfree(avb_device.msrpwd);
		avb_device.msrpwd = NULL;
	}

	if (avb_device.avdeccwd != NULL) {
		cancel_delayed_work((struct delayed_work *)avb_device.avdeccwd);
		kfree(avb_device.avdeccwd);
		avb_device.avdeccwd = NULL;
	}

	if (avb_device.wq != NULL) {
		flush_workqueue(avb_device.wq);
		destroy_workqueue(avb_device.wq);
	}

	avb_remove_all();

	platform_driver_unregister(&avb_driver);

	avb_log(AVB_KERN_NOT, KERN_NOTICE "alsa_avb_exit done");
}

module_init(alsa_avb_init)
module_exit(alsa_avb_exit)