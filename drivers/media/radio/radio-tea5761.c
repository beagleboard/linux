/*
 * drivers/media/radio/radio-tea5761.c
 *
 * Copyright (C) 2005 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-common.h>
#include <media/tuner.h>

#include "dvb_frontend.h"
#include "tea5761.h"

#define DRIVER_NAME "tea5761"

#define TEA5761_VERSION		KERNEL_VERSION(0, 0, 1)

#define TEA5761_FREQ_LOW	87500
#define TEA5761_FREQ_HIGH	108000

struct tea5761_device {
	struct video_device	*video_dev;
	struct device		*dev;
	struct dvb_frontend	fe;
	/* To control number of users access (.users field) */
	struct mutex		mutex;
	int			users;
	unsigned int		freq;
	u16			audmode;
	u8			mute;
	u8			power;
};

static struct tea5761_device tea5761;

static struct i2c_driver	tea5761_driver;
static int radio_nr = -1;

static void tea5761_power_up(struct tea5761_device *tea)
{
	struct dvb_frontend *fe = &tea->fe;
	struct dvb_tuner_ops *fe_tuner_ops = &fe->ops.tuner_ops;

	if (fe_tuner_ops->init)
		fe_tuner_ops->init(fe);
	tea->power = 1;
}

static void tea5761_power_down(struct tea5761_device *tea)
{
	struct dvb_frontend *fe = &tea->fe;
	struct dvb_tuner_ops *fe_tuner_ops = &fe->ops.tuner_ops;

	if (fe_tuner_ops->sleep)
		fe_tuner_ops->sleep(fe);
	tea->power = 0;
}

static void tea5761_set_freq(struct tea5761_device *tea, unsigned int freq)
{
	struct dvb_frontend *fe = &tea->fe;
	struct dvb_tuner_ops *fe_tuner_ops = &fe->ops.tuner_ops;
	struct analog_parameters params = {
		.mode		= V4L2_TUNER_RADIO,
		.audmode	= tea->audmode,
		.frequency	= freq,
	};

	if (NULL == fe_tuner_ops->set_analog_params) {
		dev_warn(tea->dev,
			"Tuner frontend module has no way to set frequency\n");
		return;
	}
	if (!fe_tuner_ops->set_analog_params(fe, &params))
		tea->freq = freq;
}

static int tea5761_get_freq(struct tea5761_device *tea)
{
	struct dvb_frontend *fe = &tea->fe;
	struct dvb_tuner_ops *fe_tuner_ops = &fe->ops.tuner_ops;
	u32 freq;

	if (fe_tuner_ops->get_frequency) {
		fe_tuner_ops->get_frequency(fe, &freq);
		return freq * 2 / 125;
	}

	return -ENODEV;
}

static void tea5761_set_audout_mode(struct tea5761_device *tea, int audmode)
{
	struct dvb_frontend *fe = &tea->fe;
	struct dvb_tuner_ops *fe_tuner_ops = &fe->ops.tuner_ops;
	struct analog_parameters params = {
		.mode		= V4L2_TUNER_RADIO,
		.frequency	= tea->freq,
		.audmode	= audmode,
	};

	if (NULL == fe_tuner_ops->set_analog_params) {
		dev_warn(tea->dev,
			"Tuner frontend module has no way to set frequency\n");
		return;
	}
	if (!fe_tuner_ops->set_analog_params(fe, &params))
		tea->audmode = audmode;
}

static int tea5761_get_audout_mode(struct tea5761_device *tea)
{
	return tea->audmode;
}

static void tea5761_mute(struct tea5761_device *tea, int on)
{
	struct dvb_frontend *fe = &tea->fe;
	struct dvb_tuner_ops *fe_tuner_ops = &fe->ops.tuner_ops;
	struct analog_parameters params = {
		.mode		= on ? T_STANDBY : V4L2_TUNER_RADIO,
		.frequency	= tea->freq,
		.audmode	= tea->audmode,
	};

	if (NULL == fe_tuner_ops->set_analog_params) {
		dev_warn(tea->dev,
			"Tuner frontend module has no way to set frequency\n");
		return;
	}
	if (!fe_tuner_ops->set_analog_params(fe, &params))
		tea->mute = on;
}

static int tea5761_is_muted(struct tea5761_device *tea)
{
	return tea->mute;
}

static int tea5761_vidioc_querycap(struct file *file, void *priv,
					struct v4l2_capability *c)
{
	struct tea5761_device *tea = file->private_data;
	struct video_device *dev = tea->video_dev;

	memset(c, 0, sizeof(*c));
	strlcpy(c->driver, dev->dev->driver->name, sizeof(*c->driver));
	strlcpy(c->card, dev->name, sizeof(c->card));
	snprintf(c->bus_info, sizeof(c->bus_info), "I2C:%s",
		dev->dev->bus_id);
	c->version = TEA5761_VERSION;
	c->capabilities = V4L2_CAP_TUNER | V4L2_CAP_RADIO;

	return 0;
}

static int tea5761_vidioc_g_tuner(struct file *file, void *priv,
					struct v4l2_tuner *t)
{
	struct tea5761_device *tea = file->private_data;
	struct dvb_frontend *fe = &tea->fe;
	u16 strength = 0;

	/* Only one tuner chip */
	if (t->index != 0)
		return -EINVAL;

	memset(t, 0, sizeof(*t));
	t->type = V4L2_TUNER_RADIO;
	strlcpy(t->name, "FM", sizeof(t->name));
	/* Frequency in 62.5Hz units */
	t->rangelow = TEA5761_FREQ_LOW * 16;
	t->rangehigh = TEA5761_FREQ_HIGH * 16;
	t->capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO;

	t->audmode = tea5761_get_audout_mode(tea);
	if (t->audmode == V4L2_TUNER_MODE_STEREO)
		t->rxsubchans = V4L2_TUNER_SUB_STEREO;

	if (fe->ops.tuner_ops.get_rf_strength)
		fe->ops.tuner_ops.get_rf_strength(fe, &strength);
	t->signal = strength;

	return 0;
}

static int tea5761_vidioc_s_tuner(struct file *file, void *priv,
					struct v4l2_tuner *t)
{
	struct tea5761_device *tea = file->private_data;

	/* Only tuner number 0 can be selected. */
	if (t->index != 0)
		return -EINVAL;
	tea5761_set_audout_mode(tea, t->audmode);

	return 0;
}

static int tea5761_vidioc_g_frequency(struct file *file, void *priv,
					struct v4l2_frequency *f)
{
	struct tea5761_device *tea = file->private_data;

	memset(f, 0, sizeof(*f));
	f->type = V4L2_TUNER_RADIO;
	if (tea->power)
		f->frequency = (tea5761_get_freq(tea) * 2) / 125;
	else
		f->frequency = 0;

	return 0;
}

static int tea5761_vidioc_s_frequency(struct file *file, void *priv,
					struct v4l2_frequency *f)
{
	struct tea5761_device *tea = file->private_data;

	if (f->tuner != 0)
		return -EINVAL;
	if (f->frequency == 0) {
		/* We special case this as a power down
		 * control. */
		tea5761_power_down(tea);
		return 0;
	}
	if (f->frequency < 16 * TEA5761_FREQ_LOW)
		return -EINVAL;
	if (f->frequency > 16 * TEA5761_FREQ_HIGH)
		return -EINVAL;

	tea5761_power_up(tea);
	tea5761_set_freq(tea, f->frequency);

	return 0;
}

static int tea5761_vidioc_queryctrl(struct file *file, void *priv,
					struct v4l2_queryctrl *qc)
{
	if (qc->id != V4L2_CID_AUDIO_MUTE)
		return -EINVAL;
	strlcpy(qc->name, "Mute", sizeof(qc->name));
	qc->minimum = 0;
	qc->maximum = 1;
	qc->step = 1;
	qc->default_value = 0;
	qc->type = V4L2_CTRL_TYPE_BOOLEAN;

	return 0;
}

static int tea5761_vidioc_g_ctrl(struct file *file, void *priv,
					struct v4l2_control *ct)
{
	struct tea5761_device *tea = file->private_data;

	if (ct->id != V4L2_CID_AUDIO_MUTE)
		return -EINVAL;
	if (tea->power)
		ct->value = tea5761_is_muted(tea) ? 1 : 0;
	else
		ct->value = 0;

	return 0;
}

static int tea5761_vidioc_s_ctrl(struct file *file, void *priv,
				struct v4l2_control *ct)
{
	struct tea5761_device *tea = file->private_data;

	if (ct->id != V4L2_CID_AUDIO_MUTE)
		return -EINVAL;
	tea5761_mute(tea, ct->value);

	return 0;
}

static int tea5761_vidioc_g_audio(struct file *file, void *priv,
					struct v4l2_audio *audio)
{
	struct tea5761_device *tea = file->private_data;

	strlcpy(audio->name, "FM Radio", ARRAY_SIZE(audio->name));
	audio->mode = tea->audmode;

	return 0;
}

static int tea5761_vidioc_s_audio(struct file *file, void *priv,
					struct v4l2_audio *audio)
{
	struct tea5761_device *tea = file->private_data;

	tea5761_set_audout_mode(tea,  audio->mode);

	return 0;
}

static int tea5761_vidioc_g_input(struct file *filp, void *priv,
		unsigned int *i)
{
	*i = 0;

	return 0;
}

static int tea5761_vidioc_s_input(struct file *filp, void *priv, unsigned int i)
{
	if (i)
		return -EINVAL;

	return 0;
}


static int tea5761_open(struct inode *inode, struct file *file)
{
	int minor = iminor(file->f_dentry->d_inode);
	/* Currently we support only one device */
	struct tea5761_device *tea = &tea5761;

	if (tea->video_dev->minor != minor)
		return -ENODEV;

	mutex_lock(&tea->mutex);
	/* Only exclusive access */
	if (tea->users) {
		mutex_unlock(&tea->mutex);
		return -EBUSY;
	}
	tea->users++;
	mutex_unlock(&tea->mutex);

	file->private_data = tea;
	return 0;
}

static int tea5761_release(struct inode *inode, struct file *file)
{
	struct tea5761_device *tea = file->private_data;

	mutex_lock(&tea->mutex);
	tea->users--;
	mutex_unlock(&tea->mutex);

	return 0;
}

static struct file_operations tea5761_fops = {
	.owner		= THIS_MODULE,
	.open           = tea5761_open,
	.release	= tea5761_release,
	.llseek         = no_llseek,
	.ioctl		= video_ioctl2,
	.compat_ioctl	= v4l_compat_ioctl32,
};

/*
 * tea5761_viddev_tamples - video device interface
 */
static struct video_device tea5761_video_device = {
	.owner			= THIS_MODULE,
	.name			= "TEA5761 FM-Radio",
	.type			= VID_TYPE_TUNER,
	.release		= video_device_release,
	.fops			= &tea5761_fops,
	.vidioc_querycap	= tea5761_vidioc_querycap,
	.vidioc_g_tuner		= tea5761_vidioc_g_tuner,
	.vidioc_s_tuner		= tea5761_vidioc_s_tuner,
	.vidioc_g_frequency	= tea5761_vidioc_g_frequency,
	.vidioc_s_frequency	= tea5761_vidioc_s_frequency,
	.vidioc_queryctrl	= tea5761_vidioc_queryctrl,
	.vidioc_g_ctrl		= tea5761_vidioc_g_ctrl,
	.vidioc_s_ctrl		= tea5761_vidioc_s_ctrl,
	.vidioc_g_audio		= tea5761_vidioc_g_audio,
	.vidioc_s_audio		= tea5761_vidioc_s_audio,
	.vidioc_g_input		= tea5761_vidioc_g_input,
	.vidioc_s_input		= tea5761_vidioc_s_input,
};

static int tea5761_i2c_driver_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct video_device *video_dev;
	int err = 0;
	struct tea5761_device *tea = &tea5761;

	mutex_init(&tea->mutex);

	/* Tuner attach */
	if (!dvb_attach(tea5761_attach, &tea->fe, client->adapter,
			client->addr)) {
		dev_err(&client->dev, "Could not attach tuner\n");
		err = -ENODEV;
		goto exit;
	}

	/* initialize and power off the chip */
	tea5761_power_up(tea);
	tea5761_set_audout_mode(tea, V4L2_TUNER_MODE_STEREO);
	tea5761_mute(tea, 0);
	tea5761_power_down(tea);

	/* V4L initialization */
	video_dev = video_device_alloc();
	if (video_dev == NULL) {
		dev_err(&client->dev, "Could not allocate memory\n");
		err = -ENOMEM;
		goto exit;
	}
	tea->video_dev = video_dev;

	*video_dev = tea5761_video_device;
	video_dev->dev = &client->dev;
	i2c_set_clientdata(client, video_dev);
	tea->video_dev = video_dev;
	tea->dev = &client->dev;

	err = video_register_device(video_dev, VFL_TYPE_RADIO, radio_nr);
	if (err) {
		dev_err(&client->dev, "Could not register video device\n");
		goto err_video_alloc;
	}

	return 0;

err_video_alloc:
	video_device_release(video_dev);
exit:
	kfree(client);
	return err;
}

static int tea5761_i2c_driver_remove(struct i2c_client *client)
{
	struct video_device *vd = i2c_get_clientdata(client);

	video_unregister_device(vd);

	return 0;
}

static const struct i2c_device_id tea5761_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, tea5761_id);

static struct i2c_driver tea5761_driver = {
	.driver = {
		.name	= DRIVER_NAME,
	},
	.probe	= tea5761_i2c_driver_probe,
	.remove = __devexit_p(tea5761_i2c_driver_remove),
	.id_table = tea5761_id,
};

static int __init tea5761_init(void)
{
	return i2c_add_driver(&tea5761_driver);
}

static void __exit tea5761_exit(void)
{
	i2c_del_driver(&tea5761_driver);
}

MODULE_AUTHOR("Timo Teräs");
MODULE_DESCRIPTION("I2C interface for TEA5761.");
MODULE_LICENSE("GPL");

module_param(radio_nr, int, 0);
MODULE_PARM_DESC(nr_radio, "video4linux device number to use");

module_init(tea5761_init)
module_exit(tea5761_exit)
