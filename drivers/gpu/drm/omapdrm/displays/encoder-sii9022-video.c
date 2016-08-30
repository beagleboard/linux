/*
 * Silicon Image SiI9022 Encoder Driver
 *
 * Copyright (C) 2014 Texas Instruments
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/of_irq.h>
#include <linux/hdmi.h>

#include <drm/drm_edid.h>

#include "../dss/omapdss.h"
#include "encoder-sii9022.h"

static const struct regmap_config sii9022_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int sii9022_set_power_state(struct panel_drv_data *ddata,
	enum sii9022_power_state state)
{
	unsigned pwr;
	unsigned cold;
	int r;

	switch (state) {
	case SII9022_POWER_STATE_D0:
		pwr = 0;
		cold = 0;
		break;
	case SII9022_POWER_STATE_D2:
		pwr = 2;
		cold = 0;
		break;
	case SII9022_POWER_STATE_D3_HOT:
		pwr = 3;
		cold = 0;
		break;
	case SII9022_POWER_STATE_D3_COLD:
		pwr = 3;
		cold = 1;
		break;
	default:
		return -EINVAL;
	}

	r = regmap_update_bits(ddata->regmap, SII9022_POWER_STATE_CTRL_REG,
		1 << 2, cold << 2);
	if (r) {
		dev_err(&ddata->i2c_client->dev, "failed to set hot/cold bit\n");
		return r;
	}

	r = regmap_update_bits(ddata->regmap, SII9022_POWER_STATE_CTRL_REG,
		0x3, pwr);
	if (r) {
		dev_err(&ddata->i2c_client->dev,
			"failed to set power state to %d\n", pwr);
		return r;
	}

	return 0;
}

static int sii9022_ddc_read(struct i2c_adapter *adapter,
		unsigned char *buf, u16 count, u8 offset)
{
	int r, retries;

	for (retries = 3; retries > 0; retries--) {
		struct i2c_msg msgs[] = {
			{
				.addr   = HDMI_I2C_MONITOR_ADDRESS,
				.flags  = 0,
				.len    = 1,
				.buf    = &offset,
			}, {
				.addr   = HDMI_I2C_MONITOR_ADDRESS,
				.flags  = I2C_M_RD,
				.len    = count,
				.buf    = buf,
			}
		};

		r = i2c_transfer(adapter, msgs, 2);
		if (r == 2)
			return 0;

		if (r != -EAGAIN)
			break;
	}

	return r < 0 ? r : -EIO;
}

static int sii9022_request_ddc_access(struct panel_drv_data *ddata,
	unsigned *ctrl_reg)
{
	struct device *dev = &ddata->i2c_client->dev;
	unsigned int val;
	int r;
	unsigned retries;

	*ctrl_reg = 0;

	/* Read TPI system control register*/
	r = regmap_read(ddata->regmap, SII9022_SYS_CTRL_DATA_REG, &val);
	if (r) {
		dev_err(dev, "error reading DDC BUS REQUEST\n");
		return r;
	}

	/* set SII9022_SYS_CTRL_DDC_BUS_REQUEST to request the DDC bus */
	val |= SII9022_SYS_CTRL_DDC_BUS_REQUEST;

	r = regmap_write(ddata->regmap, SII9022_SYS_CTRL_DATA_REG, val);
	if (r) {
		dev_err(dev, "error writing DDC BUS REQUEST\n");
		return r;
	}

	/*  Poll for bus DDC Bus control to be granted */
	retries = 0;
	do {
		r = regmap_read(ddata->regmap, SII9022_SYS_CTRL_DATA_REG, &val);
		if (retries++ > 100)
			return r;

	} while ((val & SII9022_SYS_CTRL_DDC_BUS_GRANTED) == 0);

	/*  Close the switch to the DDC */
	val |= SII9022_SYS_CTRL_DDC_BUS_REQUEST |
		SII9022_SYS_CTRL_DDC_BUS_GRANTED;
	r = regmap_write(ddata->regmap, SII9022_SYS_CTRL_DATA_REG, val);
	if (r) {
		dev_err(dev, "error closing switch to DDC BUS REQUEST\n");
		return r;
	}

	*ctrl_reg = val;

	return 0;
}

static int sii9022_release_ddc_access(struct panel_drv_data *ddata,
	unsigned ctrl_reg)
{
	struct device *dev = &ddata->i2c_client->dev;
	unsigned int val;
	int r;
	unsigned retries;

	val = ctrl_reg;
	val &= ~(SII9022_SYS_CTRL_DDC_BUS_REQUEST |
		SII9022_SYS_CTRL_DDC_BUS_GRANTED);

	/* retry write until we can read the register, and the bits are 0 */
	for (retries = 5; retries > 0; --retries) {
		unsigned v;

		/* ignore error, as the chip won't ACK this. */
		regmap_write(ddata->regmap, SII9022_SYS_CTRL_DATA_REG, val);

		r = regmap_read(ddata->regmap, SII9022_SYS_CTRL_DATA_REG, &v);
		if (r)
			continue;

		if (v == val)
			break;
	}

	if (retries == 0) {
		dev_err(dev, "error releasing DDC Bus Access\n");
		return r;
	}

	return 0;
}

static int sii9022_write_avi_infoframe(struct panel_drv_data *ddata)
{
	struct regmap *regmap = ddata->regmap;
	u8 data[HDMI_INFOFRAME_SIZE(AVI)];
	int r;

	r = hdmi_avi_infoframe_pack(&ddata->frame, data, sizeof(data));
	if (r < 0)
		return r;

	print_hex_dump_debug("AVI: ", DUMP_PREFIX_NONE, 16, 1, data,
		HDMI_INFOFRAME_SIZE(AVI), false);

	/* SiI9022 wants the checksum + the avi infoframe */
	r = regmap_bulk_write(regmap, SII9022_AVI_INFOFRAME_BASE_REG,
		&data[3], 1 + HDMI_AVI_INFOFRAME_SIZE);

	return r;
}

static int sii9022_clear_avi_infoframe(struct panel_drv_data *ddata)
{
	struct regmap *regmap = ddata->regmap;
	u8 data[1 + HDMI_AVI_INFOFRAME_SIZE] = { 0 };
	int r;

	r = regmap_bulk_write(regmap, SII9022_AVI_INFOFRAME_BASE_REG,
		data, 1 + HDMI_AVI_INFOFRAME_SIZE);

	return r;
}

static int sii9022_probe_chip_version(struct panel_drv_data *ddata)
{
	struct device *dev = &ddata->i2c_client->dev;
	int r = 0;
	unsigned id, rev, tpi_id;

	r = regmap_read(ddata->regmap, SII9022_DEVICE_ID_REG, &id);
	if (r) {
		dev_err(dev, "failed to read device ID\n");
		return r;
	}

	if (id != SII9022_ID_902xA) {
		dev_err(dev, "unsupported device ID: 0x%x\n", id);
		return -ENODEV;
	}

	r = regmap_read(ddata->regmap, SII9022_DEVICE_REV_ID_REG, &rev);
	if (r) {
		dev_err(dev, "failed to read device revision\n");
		return r;
	}

	r = regmap_read(ddata->regmap, SII9022_DEVICE_TPI_ID_REG, &tpi_id);
	if (r) {
		dev_err(dev, "failed to read TPI ID\n");
		return r;
	}

	dev_info(dev, "SiI902xA HDMI device %x, rev %x, tpi %x\n",
		id, rev, tpi_id);

	return r;
}

static int sii9022_enable_tpi(struct panel_drv_data *ddata)
{
	struct device *dev = &ddata->i2c_client->dev;
	int r;

	r = regmap_write(ddata->regmap, SII9022_TPI_RQB_REG, 0);
	if (r) {
		dev_err(dev, "failed to enable TPI commands\n");
		return r;
	}

	return 0;
}

static int sii9022_enable_tmds(struct panel_drv_data *ddata, bool enable)
{
	struct regmap *regmap = ddata->regmap;
	struct device *dev = &ddata->i2c_client->dev;
	int r;

	r = regmap_update_bits(regmap, SII9022_SYS_CTRL_DATA_REG,
		1 << 4, (enable ? 0 : 1) << 4);
	if (r) {
		dev_err(dev, "failed to %s TMDS output\n",
			enable ? "enable" : "disable");
		return r;
	}

	return 0;
}

static int sii9022_setup_video(struct panel_drv_data *ddata,
	struct omap_video_timings *timings)
{
	struct regmap *regmap = ddata->regmap;
	struct device *dev = &ddata->i2c_client->dev;
	int r;
	unsigned pck = timings->pixelclock / 10000;
	unsigned xres = timings->x_res;
	unsigned yres = timings->y_res;
	unsigned vfreq = 60;

	u8 vals[] = {
		pck & 0xff,
		(pck & 0xff00) >> 8,
		vfreq & 0xff,
		(vfreq & 0xff00) >> 8,
		(xres & 0xff),
		(xres & 0xff00) >> 8,
		(yres & 0xff),
		(yres & 0xff00) >> 8,
	};

	r = regmap_bulk_write(regmap, SII9022_VIDEO_DATA_BASE_REG,
		&vals, ARRAY_SIZE(vals));
	if (r) {
		dev_err(dev, "failed to write video mode config\n");
		return r;
	}

	return 0;
}

static int sii9022_hw_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct regmap *regmap = ddata->regmap;
	struct device *dev = &ddata->i2c_client->dev;
	int r;

	/* make sure we're in D2 */
	r = sii9022_set_power_state(ddata, SII9022_POWER_STATE_D2);
	if (r)
		return r;

	r = sii9022_setup_video(ddata, &ddata->timings);
	if (r)
		return r;

	/* configure input video format */
	r = regmap_write(regmap, SII9022_AVI_IN_FORMAT_REG, 0);
	if (r) {
		dev_err(dev, "failed to set input format\n");
		return r;
	}

	/* configure output video format */
	r = regmap_write(regmap, SII9022_AVI_OUT_FORMAT_REG,
		(1 << 4)); /* CONV_BT709 */
	if (r) {
		dev_err(dev, "failed to set output format\n");
		return r;
	}

	if (ddata->hdmi_mode)
		r = sii9022_write_avi_infoframe(ddata);
	else
		r = sii9022_clear_avi_infoframe(ddata);

	if (r) {
		dev_err(dev, "failed to write AVI infoframe\n");
		return r;
	}

	/* select DVI / HDMI */
	/* note: must be done before D0 */
	r = regmap_update_bits(regmap, SII9022_SYS_CTRL_DATA_REG,
		1 << 0, ddata->hdmi_mode ? 1 : 0); /* 0 = DVI, 1 = HDMI */
	if (r) {
		dev_err(dev, "failed to set DVI/HDMI mode\n");
		return r;
	}

	/* power up transmitter */
	r = sii9022_set_power_state(ddata, SII9022_POWER_STATE_D0);
	if (r)
		return r;

	/* enable TMDS */
	r = sii9022_enable_tmds(ddata, true);
	if (r)
		return r;

	/* configure input bus and pixel repetition */
	/* Note: must be done after enabling TMDS */
	r = regmap_write(regmap, SII9022_PIXEL_REPETITION_REG,
		(0 << 4) |	/* Edge select = latch on falling edge */
		(1 << 5) |	/* 24BIT */
		(1 << 6)	/* CLK_RATIO_1X */
		);
	if (r) {
		dev_err(dev, "failed to write pixel repetition reg\n");
		return r;
	}

	return 0;
}

static int sii9022_hw_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	int r;

	sii9022_enable_tmds(ddata, false);

	r = sii9022_set_power_state(ddata, SII9022_POWER_STATE_D2);
	if (r)
		return r;

	return 0;
}

static void sii9022_handle_hpd(struct panel_drv_data *ddata)
{
	struct device *dev = &ddata->i2c_client->dev;
	unsigned int stat;
	int r;
	bool htplg, rxsense;
	bool htplg_ev, rxsense_ev;

	htplg_ev = rxsense_ev = false;

	r = regmap_read(ddata->regmap, SII9022_IRQ_STATUS_REG, &stat);

	if (stat & (SII9022_IRQ_HPE | SII9022_IRQ_RXSENSE)) {
		if (stat & SII9022_IRQ_HPE)
			htplg_ev = true;
		if (stat & SII9022_IRQ_RXSENSE)
			rxsense_ev = true;

		regmap_write(ddata->regmap, SII9022_IRQ_STATUS_REG,
			     SII9022_IRQ_HPE | SII9022_IRQ_RXSENSE);
	}

	htplg = stat & SII9022_IRQ_HP_STATE;
	rxsense = stat & SII9022_IRQ_RXSENSE_STATE;

	if (ddata->htplg_state != htplg || htplg_ev) {
		dev_dbg(dev, "hotplug %sconnect\n", htplg ? "" : "dis");
		ddata->htplg_state = htplg;
	}

	if (ddata->rxsense_state != rxsense || rxsense_ev) {
		dev_dbg(dev, "rxsense %sconnect\n", rxsense ? "" : "dis");
		ddata->rxsense_state = rxsense;
	}
}

static irqreturn_t sii9022_irq_handler(int irq, void *arg)
{
	struct panel_drv_data *ddata = arg;

	mutex_lock(&ddata->lock);

	sii9022_handle_hpd(ddata);

	mutex_unlock(&ddata->lock);

	return IRQ_HANDLED;
}

static void sii9022_poll(struct work_struct *work)
{
	struct panel_drv_data *ddata;

	ddata = container_of(work, struct panel_drv_data, work.work);

	mutex_lock(&ddata->lock);

	sii9022_handle_hpd(ddata);

	mutex_unlock(&ddata->lock);

	schedule_delayed_work(&ddata->work, msecs_to_jiffies(250));
}

static int sii9022_connect(struct omap_dss_device *dssdev,
		struct omap_dss_device *dst)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct device *dev = &ddata->i2c_client->dev;
	struct omap_dss_device *in = ddata->in;
	int r;

	if (omapdss_device_is_connected(dssdev))
		return -EBUSY;

	r = in->ops.dpi->connect(in, dssdev);
	if (r)
		return r;

	mutex_lock(&ddata->lock);

	r = sii9022_set_power_state(ddata, SII9022_POWER_STATE_D2);
	if (r)
		goto err_pwr;

	ddata->htplg_state = ddata->rxsense_state = false;

	sii9022_handle_hpd(ddata);

	regmap_write(ddata->regmap, SII9022_IRQ_ENABLE_REG,
		     SII9022_IRQ_HPE | SII9022_IRQ_RXSENSE);

	if (ddata->use_polling) {
		INIT_DELAYED_WORK(&ddata->work, sii9022_poll);
		schedule_delayed_work(&ddata->work, msecs_to_jiffies(250));
	} else {
		r = devm_request_threaded_irq(dev, ddata->irq,
			NULL, sii9022_irq_handler,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"sii9022 int", ddata);
		if (r) {
			dev_err(dev, "failed to request irq\n");
			goto err_irq;
		}
	}

	dst->src = dssdev;
	dssdev->dst = dst;

	mutex_unlock(&ddata->lock);

	return 0;

err_irq:
err_pwr:
	mutex_unlock(&ddata->lock);
	in->ops.dpi->disconnect(in, dssdev);
	return r;
}

static void sii9022_disconnect(struct omap_dss_device *dssdev,
		struct omap_dss_device *dst)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct device *dev = &ddata->i2c_client->dev;
	struct omap_dss_device *in = ddata->in;

	WARN_ON(!omapdss_device_is_connected(dssdev));
	if (!omapdss_device_is_connected(dssdev))
		return;

	WARN_ON(dst != dssdev->dst);
	if (dst != dssdev->dst)
		return;

	if (ddata->use_polling)
		cancel_delayed_work_sync(&ddata->work);
	else
		devm_free_irq(dev, ddata->irq, ddata);

	dst->src = NULL;
	dssdev->dst = NULL;

	in->ops.dpi->disconnect(in, dssdev);
}

static int sii9022_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (!omapdss_device_is_connected(dssdev))
		return -ENODEV;

	if (omapdss_device_is_enabled(dssdev))
		return 0;

	in->ops.dpi->set_timings(in, &ddata->timings);

	r = in->ops.dpi->enable(in);
	if (r)
		return r;

	if (ddata->reset_gpio)
		gpiod_set_value_cansleep(ddata->reset_gpio, 0);

	mutex_lock(&ddata->lock);

	r = sii9022_hw_enable(dssdev);
	if (r)
		goto err_hw_enable;

	mutex_unlock(&ddata->lock);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;

err_hw_enable:
	mutex_unlock(&ddata->lock);

	if (ddata->reset_gpio)
		gpiod_set_value_cansleep(ddata->reset_gpio, 1);

	in->ops.dpi->disable(in);

	return r;
}

static void sii9022_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_enabled(dssdev))
		return;

	mutex_lock(&ddata->lock);

	sii9022_hw_disable(dssdev);

	mutex_unlock(&ddata->lock);

	if (ddata->reset_gpio)
		gpiod_set_value_cansleep(ddata->reset_gpio, 1);

	in->ops.dpi->disable(in);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static void sii9022_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	struct omap_video_timings t = *timings;

	/* update DPI specific timing info */
	t.data_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE;
	t.de_level = OMAPDSS_SIG_ACTIVE_HIGH;
	t.sync_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE;

	ddata->timings = t;
	dssdev->panel.timings = t;

	in->ops.dpi->set_timings(in, &t);
}

static void sii9022_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	*timings = ddata->timings;
}

static int sii9022_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	/* update DPI specific timing info */
	timings->data_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE;
	timings->de_level = OMAPDSS_SIG_ACTIVE_HIGH;
	timings->sync_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE;

	return in->ops.dpi->check_timings(in, timings);
}

static int _sii9022_read_edid(struct panel_drv_data *ddata,
		u8 *edid, int len)
{
	struct i2c_client *client = ddata->i2c_client;
	unsigned ctrl_reg;
	int r, l, bytes_read;

	r = sii9022_request_ddc_access(ddata, &ctrl_reg);
	if (r)
		return r;

	l = min(len, EDID_LENGTH);

	r = sii9022_ddc_read(client->adapter, edid, l, 0);
	if (r)
		goto err_ddc_read;

	bytes_read = l;

	/* if there are extensions, read second block */
	if (len > EDID_LENGTH && edid[0x7e] > 0) {
		/*
		 * XXX the magical sleep that makes things work. Without this
		 * we easily get a failed read. A few us seems to be too short,
		 * 100 us seems to be ok, but to be sure, lets sleep for a bit
		 * longer as this is a slow code path anyway.
		 */
		msleep(10);

		l = min(EDID_LENGTH, len - EDID_LENGTH);

		r = sii9022_ddc_read(client->adapter, edid + EDID_LENGTH,
				l, EDID_LENGTH);
		if (r)
			goto err_ddc_read;

		bytes_read += l;
	}

	r = sii9022_release_ddc_access(ddata, ctrl_reg);
	if (r)
		goto err_ddc_read;

	return bytes_read;

err_ddc_read:
	sii9022_release_ddc_access(ddata, ctrl_reg);

	return r;
}

static int sii9022_read_edid(struct omap_dss_device *dssdev,
		u8 *edid, int len)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	int r;

	mutex_lock(&ddata->lock);

	if (ddata->use_polling)
		sii9022_handle_hpd(ddata);

	if (ddata->htplg_state == false) {
		r = -ENODEV;
		goto err_hpd;
	}

	/*
	 * Sometimes we get -EREMOTEIO. The reason is unclear, but doing an i2c
	 * read to SiI9022 after requesting the DDC access seems to cause
	 * -EREMOTEIO both from the first i2c read and from the subsequent EDID
	 * read. We don't do that, but it could mean that SiI9022 has some
	 * issues around the DDC access.
	 *
	 * Retrying the EDID read solves the problem.
	 */

	r = _sii9022_read_edid(ddata, edid, len);
	if (r == -EREMOTEIO)
		r = _sii9022_read_edid(ddata, edid, len);
	if (r < 0)
		goto err_ddc_read;

	print_hex_dump_debug("EDID: ", DUMP_PREFIX_NONE, 16, 1, edid,
		r, false);

	mutex_unlock(&ddata->lock);

	return r;

err_ddc_read:
err_hpd:
	mutex_unlock(&ddata->lock);

	return r;
}

static bool sii9022_detect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	bool hpd;

	mutex_lock(&ddata->lock);

	if (ddata->use_polling)
		sii9022_handle_hpd(ddata);

	hpd = ddata->htplg_state;

	mutex_unlock(&ddata->lock);

	return hpd;
}

static int sii9022_set_infoframe(struct omap_dss_device *dssdev,
	const struct hdmi_avi_infoframe *infoframe)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	ddata->frame = *infoframe;

	return 0;
}


static int sii9022_set_hdmi_mode(struct omap_dss_device *dssdev, bool hdmi_mode)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	ddata->hdmi_mode = hdmi_mode;

	return 0;
}

static const struct omapdss_hdmi_ops sii9022_hdmi_ops = {
	.connect		= sii9022_connect,
	.disconnect		= sii9022_disconnect,

	.enable			= sii9022_enable,
	.disable		= sii9022_disable,

	.check_timings		= sii9022_check_timings,
	.set_timings		= sii9022_set_timings,
	.get_timings		= sii9022_get_timings,

	.read_edid		= sii9022_read_edid,
	.detect			= sii9022_detect,
	.set_hdmi_mode		= sii9022_set_hdmi_mode,
	.set_infoframe		= sii9022_set_infoframe,
};

static int sii9022_probe_of(struct i2c_client *client)
{
	struct panel_drv_data *ddata = dev_get_drvdata(&client->dev);
	struct device_node *node = client->dev.of_node;
	struct omap_dss_device *in;
	struct gpio_desc *gpio;

	gpio = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);

	ddata->reset_gpio = gpio;

	ddata->irq = irq_of_parse_and_map(node, 0);
	if (ddata->irq > 0)
		ddata->use_polling = false;
	else
		ddata->use_polling = true;

	in = omapdss_of_find_source_for_first_ep(node);
	if (IS_ERR(in)) {
		dev_err(&client->dev, "failed to find video source\n");
		return PTR_ERR(in);
	}

	ddata->in = in;

	return 0;
}

static int sii9022_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct panel_drv_data *ddata;
	struct omap_dss_device *dssdev;
	struct regmap *regmap;
	int r = 0;

	regmap = devm_regmap_init_i2c(client, &sii9022_regmap_config);
	if (IS_ERR(regmap)) {
		r = PTR_ERR(regmap);
		dev_err(&client->dev, "Failed to init regmap: %d\n", r);
		return r;
	}

	ddata = devm_kzalloc(&client->dev, sizeof(*ddata), GFP_KERNEL);
	if (ddata == NULL)
		return -ENOMEM;

	dev_set_drvdata(&client->dev, ddata);

	mutex_init(&ddata->lock);

	if (client->dev.of_node) {
		r = sii9022_probe_of(client);
		if (r)
			return r;
	} else {
		return -ENODEV;
	}

	ddata->regmap = regmap;
	ddata->i2c_client = client;

	dssdev = &ddata->dssdev;
	dssdev->dev = &client->dev;
	dssdev->ops.hdmi = &sii9022_hdmi_ops;
	dssdev->type = OMAP_DISPLAY_TYPE_DPI;
	dssdev->output_type = OMAP_DISPLAY_TYPE_HDMI;
	dssdev->owner = THIS_MODULE;
	dssdev->port_num = 1;

	r = sii9022_enable_tpi(ddata);
	if (r)
		goto err_tpi;

	r = sii9022_probe_chip_version(ddata);
	if (r)
		goto err_i2c;

	r = omapdss_register_output(dssdev);
	if (r) {
		dev_err(&client->dev, "Failed to register output\n");
		goto err_reg;
	}

#ifdef CONFIG_DISPLAY_ENCODER_SII9022_AUDIO_CODEC
	r = sii9022_hdmi_codec_register(&client->dev);
	if (r)
		dev_err(&client->dev,
			"Failed to register audio codec, no audio support!\n");
#endif

	return 0;
err_reg:
err_i2c:
err_tpi:
	omap_dss_put_device(ddata->in);
	return r;
}

static int sii9022_remove(struct i2c_client *client)
{
	struct panel_drv_data *ddata = dev_get_drvdata(&client->dev);
	struct omap_dss_device *dssdev = &ddata->dssdev;

#ifdef CONFIG_DISPLAY_ENCODER_SII9022_AUDIO_CODEC
	sii9022_hdmi_codec_unregister(&client->dev);
#endif

	omapdss_unregister_output(dssdev);

	WARN_ON(omapdss_device_is_enabled(dssdev));
	if (omapdss_device_is_enabled(dssdev))
		sii9022_disable(dssdev);

	WARN_ON(omapdss_device_is_connected(dssdev));
	if (omapdss_device_is_connected(dssdev))
		sii9022_disconnect(dssdev, dssdev->dst);

	omap_dss_put_device(ddata->in);

	return 0;
}

static const struct i2c_device_id sii9022_id[] = {
	{ "sii9022", 0 },
	{ },
};

static const struct of_device_id sii9022_of_match[] = {
	{ .compatible = "omapdss,sil,sii9022", },
	{},
};

MODULE_DEVICE_TABLE(i2c, sii9022_id);

static struct i2c_driver sii9022_driver = {
	.driver = {
		.name  = "sii9022",
		.owner = THIS_MODULE,
		.of_match_table = sii9022_of_match,
		},
	.probe		= sii9022_probe,
	.remove		= sii9022_remove,
	.id_table	= sii9022_id,
};

module_i2c_driver(sii9022_driver);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ti.com>");
MODULE_DESCRIPTION("SiI9022 HDMI Encoder Driver");
MODULE_LICENSE("GPL");
