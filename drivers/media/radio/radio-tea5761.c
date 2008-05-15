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

#define DRIVER_NAME "tea5761"

#define TEA5761_VERSION		KERNEL_VERSION(0, 0, 1)

#define TEA5761_I2C_ADDR	0x10

#define TEA5761_MANID		0x002b
#define TEA5761_CHIPID		0x5761

#define TEA5761_INTREG_BLMSK	0x0001
#define TEA5761_INTREG_FRRMSK	0x0002
#define TEA5761_INTREG_LEVMSK	0x0008
#define TEA5761_INTREG_IFMSK	0x0010
#define TEA5761_INTREG_BLMFLAG	0x0100
#define TEA5761_INTREG_FRRFLAG	0x0200
#define TEA5761_INTREG_LEVFLAG	0x0800
#define TEA5761_INTREG_IFFLAG	0x1000

#define TEA5761_FRQSET_SUD	0x8000
#define TEA5761_FRQSET_SM	0x4000

#define TEA5761_TNCTRL_PUPD0	0x4000
#define TEA5761_TNCTRL_BLIM	0x2000
#define TEA5761_TNCTRL_SWPM	0x1000
#define TEA5761_TNCTRL_IFCTC	0x0800
#define TEA5761_TNCTRL_AFM	0x0400
#define TEA5761_TNCTRL_SMUTE	0x0200
#define TEA5761_TNCTRL_SNC	0x0100
#define TEA5761_TNCTRL_MU	0x0080
#define TEA5761_TNCTRL_SSL1	0x0040
#define TEA5761_TNCTRL_SSL0	0x0020
#define TEA5761_TNCTRL_HLSI	0x0010
#define TEA5761_TNCTRL_MST	0x0008
#define TEA5761_TNCTRL_SWP	0x0004
#define TEA5761_TNCTRL_DTC	0x0002
#define TEA5761_TNCTRL_AHLSI	0x0001

#define TEA5761_TUNCHK_LEVEL(x)	(((x) & 0x00F0) >> 4)
#define TEA5761_TUNCHK_IFCNT(x) (((x) & 0xFE00) >> 9)
#define TEA5761_TUNCHK_TUNTO	0x0100
#define TEA5761_TUNCHK_LD	0x0008
#define TEA5761_TUNCHK_STEREO	0x0004

#define TEA5761_TESTREG_TRIGFR	0x0800

#define TEA5761_FREQ_LOW	87500
#define TEA5761_FREQ_HIGH	108000

struct tea5761_regs {
	u16 intreg;
	u16 frqset;
	u16 tnctrl;
	u16 frqchk;
	u16 tunchk;
	u16 testreg;
	u16 manid;
	u16 chipid;
} __attribute__ ((packed));

struct tea5761_write_regs {
	u8 intreg;
	u16 frqset;
	u16 tnctrl;
	u16 testreg;
} __attribute__ ((packed));

struct tea5761_device {
	struct video_device	*video_dev;
	struct i2c_client	*i2c_dev;
	struct tea5761_regs	regs;
	struct mutex		mutex;
	int			users;
};

static struct tea5761_device tea5761;

static struct i2c_driver	tea5761_driver;
static int radio_nr = -1;

static int tea5761_read_regs(struct tea5761_device *tea)
{
	int rc, i;
	u16 *p = (u16 *) &tea->regs;
	struct i2c_client *client = tea->i2c_dev;

	rc = i2c_master_recv(client, (void*) &tea->regs, sizeof(tea->regs));
	for (i = 0; i < 8; i++) {
		p[i] = __be16_to_cpu(p[i]);
	}

	dev_dbg(&client->dev,
		"chip state: %04x %04x %04x %04x %04x %04x %04x %04x\n",
		p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);

	if (rc < 0)
		dev_err(&client->dev, "read\n");

	return rc;
}

static void tea5761_write_regs(struct tea5761_device *tea)
{
	struct tea5761_write_regs wr;
	struct tea5761_regs *r = &tea->regs;
	struct i2c_client *client = tea->i2c_dev;
	u8 *p = (u8 *) r;

	wr.intreg = r->intreg & 0xff;
	wr.frqset = __cpu_to_be16(r->frqset);
	wr.tnctrl = __cpu_to_be16(r->tnctrl);
	wr.testreg = __cpu_to_be16(r->testreg);

	dev_dbg(&client->dev,
		"writing state: %02x %02x %02x %02x %02x %02x %02x\n",
		p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
	if (i2c_master_send(client, (void *) &wr, sizeof(wr)) < 0)
		dev_err(&client->dev, "write\n");
}

static void tea5761_power_up(struct tea5761_device *tea)
{
	struct tea5761_regs *r = &tea->regs;

	if (!(r->tnctrl & TEA5761_TNCTRL_PUPD0)) {
		r->tnctrl &= ~(TEA5761_TNCTRL_AFM | TEA5761_TNCTRL_MU |
			       TEA5761_TNCTRL_HLSI);
		r->testreg |= TEA5761_TESTREG_TRIGFR;
		r->tnctrl |= TEA5761_TNCTRL_PUPD0;
		return tea5761_write_regs(tea);
	}
}

static void tea5761_power_down(struct tea5761_device *tea)
{
	struct tea5761_regs *r = &tea->regs;

	if (r->tnctrl & TEA5761_TNCTRL_PUPD0) {
		r->tnctrl &= ~TEA5761_TNCTRL_PUPD0;
		return tea5761_write_regs(tea);
	}
}

static void tea5761_set_freq(struct tea5761_device *tea, int freq)
{
	struct tea5761_regs *r = &tea->regs;

	if (r->tnctrl & TEA5761_TNCTRL_HLSI)
		r->frqset = (freq + 225000) / 8192;
	else
		r->frqset = (freq - 225000) / 8192;
}

static int tea5761_get_freq(struct tea5761_device *tea)
{
	struct tea5761_regs *r = &tea->regs;

	if (r->tnctrl & TEA5761_TNCTRL_HLSI)
		return (r->frqchk * 8192) - 225000;
	else
		return (r->frqchk * 8192) + 225000;
}

static void tea5761_tune(struct tea5761_device *tea, int freq)
{
	tea5761_set_freq(tea, freq);
	tea5761_write_regs(tea);
}

static void tea5761_set_audout_mode(struct tea5761_device *tea, int audmode)
{
	struct tea5761_regs *r = &tea->regs;
	int tnctrl = r->tnctrl;

	if (audmode == V4L2_TUNER_MODE_MONO)
		r->tnctrl |= TEA5761_TNCTRL_MST;
	else
		r->tnctrl &= ~TEA5761_TNCTRL_MST;
	if (tnctrl != r->tnctrl)
		tea5761_write_regs(tea);
}

static int tea5761_get_audout_mode(struct tea5761_device *tea)
{
	struct tea5761_regs *r = &tea->regs;

	if (r->tnctrl & TEA5761_TNCTRL_MST)
		return V4L2_TUNER_MODE_MONO;
	else
		return V4L2_TUNER_MODE_STEREO;
}

static void tea5761_mute(struct tea5761_device *tea, int on)
{
	struct tea5761_regs *r = &tea->regs;
	int tnctrl = r->tnctrl;

	if (on)
		r->tnctrl |= TEA5761_TNCTRL_MU;
	else
		r->tnctrl &= ~TEA5761_TNCTRL_MU;
	if (tnctrl != r->tnctrl)
		tea5761_write_regs(tea);
}

static int tea5761_is_muted(struct tea5761_device *tea)
{
	return tea->regs.tnctrl & TEA5761_TNCTRL_MU;
}

static int tea5761_do_ioctl(struct inode *inode, struct file *file,
			    unsigned int cmd, void *arg)
{
	struct tea5761_device *tea = file->private_data;
	struct video_device *dev = tea->video_dev;
	struct i2c_client *client = tea->i2c_dev;
	struct tea5761_regs *r = &tea->regs;

	union {
		struct v4l2_capability c;
		struct v4l2_tuner t;
		struct v4l2_frequency f;
		struct v4l2_queryctrl qc;
		struct v4l2_control ct;
	} *u = arg;

	tea5761_read_regs(tea);

	switch (cmd) {
	case VIDIOC_QUERYCAP:
		dev_dbg(&client->dev, "VIDIOC_QUERYCAP\n");
		memset(&u->c, 0, sizeof(u->c));
		strlcpy(u->c.driver, dev->dev->driver->name,
			sizeof(u->c.driver));
		strlcpy(u->c.card, dev->name, sizeof(u->c.card));
		snprintf(u->c.bus_info, sizeof(u->c.bus_info), "I2C:%s",
			 dev->dev->bus_id);
		u->c.version = TEA5761_VERSION;
		u->c.capabilities = V4L2_CAP_TUNER | V4L2_CAP_RADIO;
		break;

	case VIDIOC_G_TUNER:
		/* Only one tuner chip */
		dev_dbg(&client->dev, "VIDIOC_G_TUNER\n");
		if (u->t.index != 0)
			return -EINVAL;

		memset(&u->t, 0, sizeof(u->t));
		u->t.type = V4L2_TUNER_RADIO;
		strlcpy(u->t.name, "FM", sizeof(u->t.name));
		/* Freq in 62.5Hz units */
		u->t.rangelow = TEA5761_FREQ_LOW * 16;
		u->t.rangehigh = TEA5761_FREQ_HIGH * 16;
		u->t.capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO;
		if (r->tunchk & TEA5761_TUNCHK_STEREO)
			u->t.rxsubchans = V4L2_TUNER_SUB_STEREO;
		u->t.audmode = tea5761_get_audout_mode(tea);
		u->t.signal = TEA5761_TUNCHK_LEVEL(r->tunchk) * 0xffff / 0xf;
		u->t.afc = TEA5761_TUNCHK_IFCNT(r->tunchk);
		break;

	case VIDIOC_S_TUNER:
		/* Only tuner nro 0 can be selected. */
		dev_dbg(&client->dev, "VIDIOC_S_TUNER\n");
		if (u->t.index != 0)
			return -EINVAL;
		tea5761_set_audout_mode(tea, u->t.audmode);
		break;

	case VIDIOC_G_FREQUENCY:
		dev_dbg(&client->dev, "VIDIOC_G_FREQUENCY\n");
		memset(&u->f, 0, sizeof(u->f));
		u->f.type = V4L2_TUNER_RADIO;
		if (r->tnctrl & TEA5761_TNCTRL_PUPD0)
			u->f.frequency = (tea5761_get_freq(tea) * 2) / 125;
		else
			u->f.frequency = 0;
		break;

	case VIDIOC_S_FREQUENCY:
		dev_dbg(&client->dev, "VIDIOC_S_FREQUENCY %u\n",
			u->f.frequency);
		if (u->f.tuner != 0)
			return -EINVAL;
		if (u->f.frequency == 0) {
			/* We special case this as a power down
			 * control. */
			tea5761_power_down(tea);
			break;
		}
		if (u->f.frequency < 16 * TEA5761_FREQ_LOW)
			return -EINVAL;
		if (u->f.frequency > 16 * TEA5761_FREQ_HIGH)
			return -EINVAL;

		tea5761_power_up(tea);
		tea5761_tune(tea, (u->f.frequency * 125) / 2);
		break;

	case VIDIOC_QUERYCTRL:
		dev_dbg(&client->dev, "VIDIOC_QUERYCTRL %d\n", u->qc.id);
		if (u->qc.id != V4L2_CID_AUDIO_MUTE)
			return -EINVAL;
		strlcpy(u->qc.name, "Mute", sizeof(u->qc.name));
		u->qc.minimum = 0;
		u->qc.maximum = 1;
		u->qc.step = 1;
		u->qc.default_value = 0;
		u->qc.type = V4L2_CTRL_TYPE_BOOLEAN;
		break;

	case VIDIOC_G_CTRL:
		dev_dbg(&client->dev, "VIDIOC_G_CTRL %d\n", u->ct.id);
		if (u->ct.id != V4L2_CID_AUDIO_MUTE)
			return -EINVAL;
		if (r->tnctrl & TEA5761_TNCTRL_PUPD0)
			u->ct.value = tea5761_is_muted(tea) ? 1 : 0;
		else
			u->ct.value = 0;
		break;

	case VIDIOC_S_CTRL:
		dev_dbg(&client->dev, "VIDIOC_S_CTRL %d\n", u->ct.id);
		if (u->ct.id != V4L2_CID_AUDIO_MUTE)
			return -EINVAL;
		tea5761_mute(tea, u->ct.value);
		break;

	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}

static int tea5761_ioctl(struct inode *inode, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	return video_usercopy(inode, file, cmd, arg, tea5761_do_ioctl);
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
	.ioctl		= tea5761_ioctl,
	.llseek         = no_llseek,
};

static struct video_device tea5761_video_device = {
	.owner         = THIS_MODULE,
	.name          = "TEA5761 FM-Radio",
	.type          = VID_TYPE_TUNER,
	.fops          = &tea5761_fops,
	.release       = video_device_release
};

static int tea5761_i2c_driver_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct video_device *video_dev;
	int err = 0;
	struct tea5761_device *tea = &tea5761;

	mutex_init(&tea->mutex);

	tea->i2c_dev = client;

	/* V4L initialization */
	video_dev = video_device_alloc();
	if (video_dev == NULL) {
		dev_err(&client->dev, "couldn't allocate memory\n");
		err = -ENOMEM;
		goto exit;
	}
	tea->video_dev = video_dev;

	*video_dev = tea5761_video_device;
	video_dev->dev = &client->dev;
	i2c_set_clientdata(client, video_dev);

	/* initialize and power off the chip */
	tea5761_read_regs(tea);
	tea5761_set_audout_mode(tea, V4L2_TUNER_MODE_STEREO);
	tea5761_mute(tea, 0);
	tea5761_power_down(tea);

	tea5761.video_dev = video_dev;
	tea5761.i2c_dev = client;

	err = video_register_device(video_dev, VFL_TYPE_RADIO, radio_nr);
	if (err) {
		dev_err(&client->dev, "couldn't register video device\n");
		goto err_video_alloc;
	}

	dev_info(&client->dev, "tea5761 (version %d) detected\n",
		(tea->regs.manid >> 12) & 0xf);

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
	int res;

	if ((res = i2c_add_driver(&tea5761_driver))) {
		printk(KERN_ERR DRIVER_NAME ": driver registration failed\n");
		return res;
	}

	return 0;
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
