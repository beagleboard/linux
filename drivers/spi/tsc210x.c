/*
 * tsc210x.c - TSC2101/2102/... driver core
 *
 * Copyright (c) 2005-2007 Andrzej Zaborowski  <balrog@zabor.org>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/autoconf.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc210x.h>


/* NOTE:  It should be straightforward to make this driver framework handle
 * tsc2100 and tsc2111 chips, and maybe others too.  The main differences
 * are in the audio codec capabilities, but there are also some differences
 * in how the various sensors (including touchscreen) are handled.
 */

/* Bit field definitions for chip registers */

/* Scan X, Y, Z1, Z2, chip controlled, 12-bit, 16 samples, 500 usec */
#define TSC210X_ADC_TS_CONTROL		0x8bf4
/* Scan BAT1, BAT2, AUX1, AUX2, 12-bit, 16 samples, 500 usec */
#define TSC210X_ADC_SCAN_CONTROL	0x2ff4
/* Scan TEMP1, 12-bit, 16 samples, 500 usec */
#define TSC210X_ADC_T1_CONTROL		0x2bf4
/* Scan TEMP2, 12-bit, 16 samples, 500 usec */
#define TSC210X_ADC_T2_CONTROL		0x33f4
/* PINT/DAV acts as DAV */
#define TSC210X_ADC_DAV			0x4000
/* Internal reference, 100 usec delay, 1.25 V reference */
#define TSC210X_ADC_INT_REF		0x0016
/* External reference, 100 usec delay, 1.25 V reference */
#define TSC210X_ADC_EXT_REF		0x0002
/* 84 usec precharge time, 32 usec sense time */
#define TSC210X_CONFIG_TIMES		0x0008
/* The reset sequence */
#define TSC210X_RESET			0xbb00
/* Pen Status bit */
#define TSC210X_ADC_PSTCM		(1 << 15)
/* A/D Status bit */
#define TSC210X_ADC_ADST		(1 << 14)
/* (At least) One of X, Y, Z1, Z2 contains data */
#define TSC210X_TS_DAV			0x0780
/* (At least) One of BAT1, BAT2, AUX1, AUX2 contains data */
#define TSC210X_PS_DAV			0x0078
/* TEMP1 contains data */
#define TSC210X_T1_DAV			0x0004
/* TEMP2 contains data */
#define TSC210X_T2_DAV			0x0002
#define TSC2101_DAC_ON			0x0000
#define TSC2101_DAC_OFF			0xe7fc
#define TSC2102_DAC_ON			0x3ba0
#define TSC2102_DAC_OFF			0xafa0
#define TSC210X_FS44K			(1 << 13)
#define TSC210X_PLL1_OFF		0x0000
#define TSC210X_PLL1_44K		0x811c
#define TSC210X_PLL1_48K		0x8120
#define TSC210X_PLL2_44K		(5462 << 2)
#define TSC210X_PLL2_48K		(1920 << 2)
#define TSC210X_SLVMS			(1 << 11)
#define TSC210X_DEEMPF			(1 << 0)
#define TSC2102_BASSBC			(1 << 1)
#define TSC210X_KEYCLICK_OFF		0x0000

#define CS_CHANGE(val)			0

struct tsc210x_spi_req {
	struct spi_device *dev;
	u16 command;
	u16 data;
	struct spi_message message;
};

struct tsc210x_dev {
	enum tsc_type {
		tsc2101,
		tsc2102,
	} kind;
	struct tsc210x_config *pdata;
	struct clk *bclk_ck;

	struct workqueue_struct *queue;
	struct delayed_work ts_worker;		/* Poll-wait for PEN UP */
	struct delayed_work sensor_worker;	/* Scan the ADC inputs */
	struct mutex queue_lock;
	struct completion data_avail;

	tsc210x_touch_t touch_cb;
	void *touch_cb_ctx;

	tsc210x_coords_t coords_cb;
	void *coords_cb_ctx;

	tsc210x_ports_t ports_cb;
	void *ports_cb_ctx;

	tsc210x_temp_t temp1_cb;
	void *temp2_cb_ctx;

	tsc210x_temp_t temp2_cb;
	void *temp1_cb_ctx;

	struct spi_device *spi;
	struct spi_transfer *transfers;
	struct tsc210x_spi_req req_adc;
	struct tsc210x_spi_req req_status;
	struct tsc210x_spi_req req_mode;
	struct tsc210x_spi_req req_stop;

	int pendown;
	int flushing;			/* Queue flush in progress */
	u16 status;
	u16 adc_data[4];
	int bat[2], aux[2], temp[2];
};

static struct {
	unsigned int ts_msecs;		/* Interval for .ts_timer */
	unsigned int mode_msecs;	/* Interval for .mode_timer */
} settings;

module_param_named(touch_check_msecs, settings.ts_msecs, uint, 0);
MODULE_PARM_DESC(touch_check_msecs, "Pen-up polling interval in msecs");

module_param_named(sensor_scan_msecs, settings.mode_msecs, uint, 0);
MODULE_PARM_DESC(sensor_scan_msecs, "Temperature & battery scan interval");

int tsc210x_write_sync(struct tsc210x_dev *dev,
		int page, u8 address, u16 data)
{
	static struct tsc210x_spi_req req;
	static struct spi_transfer transfer[2];
	int ret;

	spi_message_init(&req.message);

	/* Address */
	req.command = (page << 11) | (address << 5);
	transfer[0].tx_buf = &req.command;
	transfer[0].rx_buf = NULL;
	transfer[0].len = 2;
	spi_message_add_tail(&transfer[0], &req.message);

	/* Data */
	transfer[1].tx_buf = &data;
	transfer[1].rx_buf = NULL;
	transfer[1].len = 2;
	transfer[1].cs_change = CS_CHANGE(1);
	spi_message_add_tail(&transfer[1], &req.message);

	ret = spi_sync(dev->spi, &req.message);
	if (!ret && req.message.status)
		ret = req.message.status;
	if (ret)
		dev_dbg(&dev->spi->dev, "write_sync --> %d\n", ret);

	return ret;
}
EXPORT_SYMBOL(tsc210x_write_sync);

int tsc210x_reads_sync(struct tsc210x_dev *dev,
		int page, u8 startaddress, u16 *data, int numregs)
{
	static struct tsc210x_spi_req req;
	static struct spi_transfer transfer[6];
	int ret, i, j;

	if (numregs + 1 > ARRAY_SIZE(transfer))
		return -EINVAL;

	spi_message_init(&req.message);
	i = 0;
	j = 0;

	/* Address */
	req.command = 0x8000 | (page << 11) | (startaddress << 5);
	transfer[i].tx_buf = &req.command;
	transfer[i].rx_buf = NULL;
	transfer[i].len = 2;
	spi_message_add_tail(&transfer[i ++], &req.message);

	/* Data */
	while (j < numregs) {
		transfer[i].tx_buf = NULL;
		transfer[i].rx_buf = &data[j ++];
		transfer[i].len = 2;
		transfer[i].cs_change = CS_CHANGE(j == numregs);
		spi_message_add_tail(&transfer[i ++], &req.message);
	}

	ret = spi_sync(dev->spi, &req.message);
	if (!ret && req.message.status)
		ret = req.message.status;
	if (ret)
		dev_dbg(&dev->spi->dev, "reads_sync --> %d\n", ret);

	return ret;
}
EXPORT_SYMBOL(tsc210x_reads_sync);

int tsc210x_read_sync(struct tsc210x_dev *dev, int page, u8 address)
{
	u16 ret;
	int status;

	status = tsc210x_reads_sync(dev, page, address, &ret, 1);
	return status ? : ret;
}
EXPORT_SYMBOL(tsc210x_read_sync);


static void tsc210x_submit_async(struct tsc210x_spi_req *spi)
{
	int ret;

	ret = spi_async(spi->dev, &spi->message);
	if (ret)
		dev_dbg(&spi->dev->dev, "%s: error %i in SPI request\n",
				__FUNCTION__, ret);
}

static void tsc210x_request_alloc(struct tsc210x_dev *dev,
		struct tsc210x_spi_req *spi, int direction,
		int page, u8 startaddress, int numregs, u16 *data,
		void (*complete)(struct tsc210x_dev *context),
		struct spi_transfer **transfer)
{
	spi->dev = dev->spi;

	if (direction == 1)	/* Write */
		numregs = 2;
	else			/* Read */
		numregs += 1;

	spi_message_init(&spi->message);
	spi->message.complete = (void (*)(void *)) complete;
	spi->message.context = dev;

	/* Address */
	spi->command = (page << 11) | (startaddress << 5);
	if (direction != 1)
		spi->command |= 1 << 15;

	(*transfer)->tx_buf = &spi->command;
	(*transfer)->rx_buf = NULL;
	(*transfer)->len = 2;
	spi_message_add_tail((*transfer) ++, &spi->message);

	/* Data */
	while (-- numregs) {
		if (direction == 1)
			(*transfer)->tx_buf = &spi->data;
		else
			(*transfer)->rx_buf = data++;
		(*transfer)->len = 2;
		(*transfer)->cs_change = CS_CHANGE(numregs != 1);
		spi_message_add_tail((*transfer) ++, &spi->message);
	}
}

#define tsc210x_cb_register_func(cb, cb_t)	\
int tsc210x_ ## cb(struct device *dev, cb_t handler, void *context)	\
{	\
	struct tsc210x_dev *tsc = dev_get_drvdata(dev);	\
	\
	/* Lock the module */	\
	if (handler && !tsc->cb)	\
		if (!try_module_get(THIS_MODULE)) {	\
			dev_err(dev, "Failed to get TSC module\n");	\
		}	\
	if (!handler && tsc->cb)	\
		module_put(THIS_MODULE);	\
	\
	tsc->cb = handler;	\
	tsc->cb ## _ctx = context;	\
	return 0;	\
} \
EXPORT_SYMBOL(tsc210x_ ## cb);

tsc210x_cb_register_func(touch_cb, tsc210x_touch_t)
tsc210x_cb_register_func(coords_cb, tsc210x_coords_t)
tsc210x_cb_register_func(ports_cb, tsc210x_ports_t)
tsc210x_cb_register_func(temp1_cb, tsc210x_temp_t)
tsc210x_cb_register_func(temp2_cb, tsc210x_temp_t)

#ifdef DEBUG
static void tsc210x_print_dav(struct tsc210x_dev *dev)
{
	int status = tsc210x_read_sync(dev, TSC210X_TS_STATUS_CTRL);

	if (status < 0) {
		dev_dbg(&dev->spi->dev, "status %d\n", status);
		return;
	}

	if (!(status & 0x0fff))
		return;

	dev_dbg(&dev->spi->dev, "data in %s%s%s%s%s%s%s%s%s%s%s\n",
		(status & 0x0400) ? " X" : "",
		(status & 0x0200) ? " Y" : "",
		(status & 0x0100) ? " Z1" : "",
		(status & 0x0080) ? " Z2" : "",
		(status & 0x0040) ? " BAT1" : "",
		(status & 0x0020) ? " BAT2" : "",
		(status & 0x0010) ? " AUX1" : "",
		(status & 0x0008) ? " AUX2" : "",
		(status & 0x0004) ? " TEMP1" : "",
		(status & 0x0002) ? " TEMP2" : "",
		(status & 0x0001) ? " KP" : "");
}
#endif

static void tsc210x_complete_dummy(struct tsc210x_dev *dev)
{
}

static inline void tsc210x_touchscreen_mode(struct tsc210x_dev *dev)
{
	/* Scan X, Y, Z1, Z2, chip controlled, 12-bit, 16 samples, 500 usec */
	dev->req_mode.data = TSC210X_ADC_TS_CONTROL;
	tsc210x_submit_async(&dev->req_mode);
}

static inline void tsc210x_portscan_mode(struct tsc210x_dev *dev)
{
	/* Scan BAT1, BAT2, AUX1, AUX2, 12-bit, 16 samples, 500 usec */
	dev->req_mode.data = TSC210X_ADC_SCAN_CONTROL;
	tsc210x_submit_async(&dev->req_mode);
}

static inline void tsc210x_temp1_mode(struct tsc210x_dev *dev)
{
	/* Scan TEMP1, 12-bit, 16 samples, 500 usec */
	dev->req_mode.data = TSC210X_ADC_T1_CONTROL;
	tsc210x_submit_async(&dev->req_mode);
}

static inline void tsc210x_temp2_mode(struct tsc210x_dev *dev)
{
	/* Scan TEMP2, 12-bit, 16 samples, 500 usec */
	dev->req_mode.data = TSC210X_ADC_T2_CONTROL;
	tsc210x_submit_async(&dev->req_mode);
}

/* Abort current conversion if any */
static void tsc210x_new_mode(struct tsc210x_dev *dev)
{
	dev->req_stop.data = TSC210X_ADC_ADST;
	tsc210x_submit_async(&dev->req_stop);
}

static void tsc210x_queue_scan(struct tsc210x_dev *dev)
{
	if (dev->pdata->monitor)
		if (!queue_delayed_work(dev->queue,
				&dev->sensor_worker,
				msecs_to_jiffies(settings.mode_msecs)))
			dev_err(&dev->spi->dev,
					"%s: can't queue measurements\n",
					__FUNCTION__);
}

static void tsc210x_queue_penup(struct tsc210x_dev *dev)
{
	if (!queue_delayed_work(dev->queue,
			&dev->ts_worker,
			msecs_to_jiffies(settings.ts_msecs)))
		dev_err(&dev->spi->dev,
				"%s: can't queue pen-up poll\n",
				__FUNCTION__);
}

static void tsc210x_status_report(struct tsc210x_dev *dev)
{
	/*
	 * Read all converted data from corresponding registers
	 * so that the ADC can move on to a new conversion.
	 */
	if (dev->status & TSC210X_TS_DAV) {
		if (!dev->pendown && !dev->flushing) {
			dev->pendown = 1;
			if (dev->touch_cb)
				dev->touch_cb(dev->touch_cb_ctx, 1);

			tsc210x_queue_penup(dev);
		}

		tsc210x_submit_async(&dev->req_adc);
	}

	if (dev->status & (TSC210X_PS_DAV | TSC210X_T1_DAV | TSC210X_T2_DAV))
		complete(&dev->data_avail);
}

static void tsc210x_data_report(struct tsc210x_dev *dev)
{
	u16 adc_data[4];

	if (dev->status & TSC210X_PS_DAV) {
		tsc210x_reads_sync(dev, TSC210X_TS_BAT1, adc_data, 4);
		/* NOTE: reads_sync() could fail */

		dev->bat[0] = adc_data[0];
		dev->bat[1] = adc_data[1];
		dev->aux[0] = adc_data[2];
		dev->aux[1] = adc_data[3];
		if (dev->ports_cb)
			dev->ports_cb(dev->ports_cb_ctx, dev->bat, dev->aux);
	}

	if (dev->status & TSC210X_T1_DAV) {
		dev->temp[0] = tsc210x_read_sync(dev, TSC210X_TS_TEMP1);

		if (dev->temp[0] >= 0 && dev->temp1_cb)
			dev->temp1_cb(dev->temp1_cb_ctx, dev->temp[0]);
	}

	if (dev->status & TSC210X_T2_DAV) {
		dev->temp[1] = tsc210x_read_sync(dev, TSC210X_TS_TEMP2);

		if (dev->temp[1] >= 0 && dev->temp2_cb)
			dev->temp2_cb(dev->temp2_cb_ctx, dev->temp[1]);
	}
}

static void tsc210x_coords_report(struct tsc210x_dev *dev)
{
	if (dev->coords_cb)
		dev->coords_cb(dev->coords_cb_ctx,
				dev->adc_data[0], dev->adc_data[1],
				dev->adc_data[2], dev->adc_data[3]);
}

/*
 * There are at least three ways to check for pen-up:
 *	- the PINT/DAV pin state,
 *	- reading PSTCM bit in ADC Control register (D15, offset 0x00),
 *	- reading ADST bit in ADC Control register (D14, offset 0x00),
 *		ADC idle would indicate no screen touch.
 * Unfortunately none of them seems to be 100% accurate and you will
 * find they are totally inconsistent, i.e. you get to see any arbitrary
 * combination of values in these three bits.  So we will busy-wait
 * for a moment when the latter two indicate a pen-up, using a queue,
 * before we report a pen-up.
 */
static void tsc210x_pressure(struct work_struct *work)
{
	struct tsc210x_dev *dev =
		container_of(work, struct tsc210x_dev, ts_worker.work);
	int adc_status;

	WARN_ON(!dev->pendown);

	adc_status = tsc210x_read_sync(dev, TSC210X_TS_ADC_CTRL);
	if (adc_status < 0) {
		dev_dbg(&dev->spi->dev, "pressure, err %d\n", adc_status);
		return;
	}

	if ((adc_status & TSC210X_ADC_PSTCM) != 0
			|| !(adc_status & TSC210X_ADC_ADST))
		tsc210x_queue_penup(dev);
	else {
		dev->pendown = 0;
		if (dev->touch_cb)
			dev->touch_cb(dev->touch_cb_ctx, 0);
	}
}

static void tsc210x_wait_data(struct tsc210x_dev *dev)
{
	wait_for_completion(&dev->data_avail);

	tsc210x_data_report(dev);
}

static void tsc210x_input_scan(struct work_struct *work)
{
	struct tsc210x_dev *dev = (struct tsc210x_dev *)
		container_of(work, struct tsc210x_dev, sensor_worker.work);

	tsc210x_new_mode(dev);

	if (dev->pdata->monitor &
			(TSC_BAT1 | TSC_BAT2 | TSC_AUX1 | TSC_AUX2)) {
		tsc210x_portscan_mode(dev);
		tsc210x_wait_data(dev);
	}

	if (dev->pdata->monitor & TSC_TEMP) {
		tsc210x_temp1_mode(dev);
		tsc210x_wait_data(dev);

		tsc210x_temp2_mode(dev);
		tsc210x_wait_data(dev);
	}

	tsc210x_touchscreen_mode(dev);

	mutex_lock(&dev->queue_lock);
	if (!dev->flushing)
		tsc210x_queue_scan(dev);
	mutex_unlock(&dev->queue_lock);
}

/* ADC has finished a new conversion for us.  */
static irqreturn_t tsc210x_handler(int irq, void *dev_id)
{
	struct tsc210x_dev *dev = (struct tsc210x_dev *) dev_id;

	/* See what data became available.  */
	tsc210x_submit_async(&dev->req_status);

	return IRQ_HANDLED;
}

#if defined(CONFIG_SOUND) || defined(CONFIG_SOUND_MODULE)

/*
 * FIXME the audio support shouldn't be included in upstream patches
 * until it's ready.  They might be better as utility functions linked
 * with a chip-specific tsc21xx audio module ... e.g. chips with input
 * channels need more, as will ones with multiple output channels and
 * so on.  Each of these functions should probably return a fault code,
 * and will need to be exported so the sound drier can be modular.
 */

/*
 * Volume level values should be in the range [0, 127].
 * Higher values mean lower volume.
 */
void tsc210x_set_dac_volume(struct device *dev, u8 left_ch, u8 right_ch)
{
	struct tsc210x_dev *tsc = dev_get_drvdata(dev);
	int val;

	if (tsc->kind == tsc2102) {
		/* All 0's or all 1's */
		if (left_ch == 0x00 || left_ch == 0x7f)
			left_ch ^= 0x7f;
		if (right_ch == 0x00 || right_ch == 0x7f)
			right_ch ^= 0x7f;
	}

	val = tsc210x_read_sync(tsc, TSC210X_DAC_GAIN_CTRL);
	if (val < 0) {
		dev_dbg(dev, "%s, err %d\n", __FUNCTION__, val);
		return;
	}

	val &= 0x8080;	/* Preserve mute-bits */
	val |= (left_ch << 8) | right_ch;

	tsc210x_write_sync(tsc, TSC210X_DAC_GAIN_CTRL, val);
	/* NOTE: write_sync() could fail */
}

void tsc210x_set_dac_mute(struct device *dev, int left_ch, int right_ch)
{
	struct tsc210x_dev *tsc = dev_get_drvdata(dev);
	int val;

	val = tsc210x_read_sync(tsc, TSC210X_DAC_GAIN_CTRL);
	if (val < 0) {
		dev_dbg(dev, "%s, err %d\n", __FUNCTION__, val);
		return;
	}

	val &= 0x7f7f;	/* Preserve volume settings */
	val |= (left_ch << 15) | (right_ch << 7);

	tsc210x_write_sync(tsc, TSC210X_DAC_GAIN_CTRL, val);
	/* NOTE: write_sync() could fail */
}

void tsc210x_get_dac_mute(struct device *dev, int *left_ch, int *right_ch)
{
	struct tsc210x_dev *tsc = dev_get_drvdata(dev);
	int val;

	val = tsc210x_read_sync(tsc, TSC210X_DAC_GAIN_CTRL);
	if (val < 0) {
		dev_dbg(dev, "%s, err %d\n", __FUNCTION__, val);
		return;
	}

	*left_ch = !!(val & (1 << 15));
	*right_ch = !!(val & (1 << 7));
}

void tsc210x_set_deemphasis(struct device *dev, int enable)
{
	struct tsc210x_dev *tsc = dev_get_drvdata(dev);
	int val;

	val = tsc210x_read_sync(tsc, TSC210X_POWER_CTRL);
	if (val < 0) {
		dev_dbg(dev, "%s, err %d\n", __FUNCTION__, val);
		return;
	}

	if (enable)
		val &= ~TSC210X_DEEMPF;
	else
		val |= TSC210X_DEEMPF;

	tsc210x_write_sync(tsc, TSC210X_POWER_CTRL, val);
	/* NOTE: write_sync() could fail */
}

void tsc2102_set_bassboost(struct device *dev, int enable)
{
	struct tsc210x_dev *tsc = dev_get_drvdata(dev);
	int val;

	val = tsc210x_read_sync(tsc, TSC210X_POWER_CTRL);
	if (val < 0) {
		dev_dbg(dev, "%s, err %d\n", __FUNCTION__, val);
		return;
	}

	if (enable)
		val &= ~TSC2102_BASSBC;
	else
		val |= TSC2102_BASSBC;

	tsc210x_write_sync(tsc, TSC210X_POWER_CTRL, val);
	/* NOTE: write_sync() could fail */
}

/*	{rate, dsor, fsref}	*/
static const struct tsc210x_rate_info_s tsc2101_rates[] = {
	/* Fsref / 6.0 */
	{7350,	7,	1},
	{8000,	7,	0},
	/* Fsref / 5.5 */
	{8018,	6,	1},
	{8727,	6,	0},
	/* Fsref / 5.0 */
	{8820,	5,	1},
	{9600,	5,	0},
	/* Fsref / 4.0 */
	{11025,	4,	1},
	{12000,	4,	0},
	/* Fsref / 3.0 */
	{14700,	3,	1},
	{16000,	3,	0},
	/* Fsref / 2.0 */
	{22050,	2,	1},
	{24000,	2,	0},
	/* Fsref / 1.5 */
	{29400,	1,	1},
	{32000,	1,	0},
	/* Fsref */
	{44100,	0,	1},
	{48000,	0,	0},

	{0,	0,	0},
};

/*	{rate, dsor, fsref}	*/
static const struct tsc210x_rate_info_s tsc2102_rates[] = {
	/* Fsref / 6.0 */
	{7350,	63,	1},
	{8000,	63,	0},
	/* Fsref / 6.0 */
	{7350,	54,	1},
	{8000,	54,	0},
	/* Fsref / 5.0 */
	{8820,	45,	1},
	{9600,	45,	0},
	/* Fsref / 4.0 */
	{11025,	36,	1},
	{12000,	36,	0},
	/* Fsref / 3.0 */
	{14700,	27,	1},
	{16000,	27,	0},
	/* Fsref / 2.0 */
	{22050,	18,	1},
	{24000,	18,	0},
	/* Fsref / 1.5 */
	{29400,	9,	1},
	{32000,	9,	0},
	/* Fsref */
	{44100,	0,	1},
	{48000,	0,	0},

	{0,	0,	0},
};

int tsc210x_set_rate(struct device *dev, int rate)
{
	struct tsc210x_dev *tsc = dev_get_drvdata(dev);
	int i;
	int val;
	const struct tsc210x_rate_info_s *rates;

	if (tsc->kind == tsc2101)
		rates = tsc2101_rates;
	else
		rates = tsc2102_rates;

	for (i = 0; rates[i].sample_rate; i ++)
		if (rates[i].sample_rate == rate)
			break;
	if (rates[i].sample_rate == 0) {
		dev_err(dev, "Unknown sampling rate %i.0 Hz\n", rate);
		return -EINVAL;
	}

	if (tsc->kind == tsc2101) {
		val = tsc210x_read_sync(tsc, TSC210X_AUDIO1_CTRL);
		if (val < 0) {
			dev_dbg(dev, "%s, err %d\n", __FUNCTION__, val);
			return val;
		}
		val &= ~((7 << 3) | (7 << 0));
		val |= rates[i].divisor << 3;
		val |= rates[i].divisor << 0;
	} else
		val = rates[i].divisor;

	tsc210x_write_sync(tsc, TSC210X_AUDIO1_CTRL, val);
	/* NOTE: write_sync() could fail */

	val = tsc210x_read_sync(tsc, TSC210X_AUDIO3_CTRL);
	if (val < 0) {
		dev_dbg(dev, "%s, err %d\n", __FUNCTION__, val);
		return val;
	}

	if (tsc2102_rates[i].fs_44k) {
		tsc210x_write_sync(tsc,
				TSC210X_AUDIO3_CTRL, val | TSC210X_FS44K);
		/* Enable Phase-locked-loop, set up clock dividers */
		tsc210x_write_sync(tsc, TSC210X_PLL1_CTRL, TSC210X_PLL1_44K);
		tsc210x_write_sync(tsc, TSC210X_PLL2_CTRL, TSC210X_PLL2_44K);
	} else {
		tsc210x_write_sync(tsc,
				TSC210X_AUDIO3_CTRL, val & ~TSC210X_FS44K);
		/* Enable Phase-locked-loop, set up clock dividers */
		tsc210x_write_sync(tsc, TSC210X_PLL1_CTRL, TSC210X_PLL1_48K);
		tsc210x_write_sync(tsc, TSC210X_PLL2_CTRL, TSC210X_PLL2_48K);
	}

	return 0;
}

/*
 * Perform basic set-up with default values and power the DAC/ADC on.
 */
void tsc210x_dac_power(struct device *dev, int on)
{
	struct tsc210x_dev *tsc = dev_get_drvdata(dev);

	/* NOTE: write_sync() could fail */
	if (on) {
		/* 16-bit words, DSP mode, sample at Fsref */
		tsc210x_write_sync(tsc,
				TSC210X_AUDIO1_CTRL, 0x0100);
		/* Keyclicks off, soft-stepping at normal rate */
		tsc210x_write_sync(tsc,
				TSC210X_AUDIO2_CTRL, TSC210X_KEYCLICK_OFF);
		/* 44.1 kHz Fsref, continuous transfer mode, master DAC */
		tsc210x_write_sync(tsc,
				TSC210X_AUDIO3_CTRL, 0x2000);
		/* Soft-stepping enabled, 1 dB MIX AGC hysteresis */
		tsc210x_write_sync(tsc,
				TSC210X_AUDIO4_CTRL, 0x0000);

		/* PLL generates 44.1 kHz */
		tsc210x_write_sync(tsc,
				TSC210X_PLL1_CTRL, TSC210X_PLL1_44K);
		tsc210x_write_sync(tsc,
				TSC210X_PLL2_CTRL, TSC210X_PLL2_44K);

		/* Codec & DAC power up, virtual ground disabled */
		tsc210x_write_sync(tsc,
				TSC210X_POWER_CTRL, (tsc->kind == tsc2101) ?
				TSC2101_DAC_ON : TSC2102_DAC_ON);
	} else {
		/* All off */
		tsc210x_write_sync(tsc,
				TSC210X_AUDIO2_CTRL, TSC210X_KEYCLICK_OFF);
		tsc210x_write_sync(tsc,
				TSC210X_PLL1_CTRL, TSC210X_PLL1_OFF);
#if 0
		tsc210x_write_sync(tsc,
				TSC210X_POWER_CTRL, (tsc->kind == tsc2101) ?
				TSC2102_DAC_OFF : TSC2102_DAC_OFF);
#endif
	}
}

void tsc210x_set_i2s_master(struct device *dev, int state)
{
	struct tsc210x_dev *tsc = dev_get_drvdata(dev);
	int val;

	val = tsc210x_read_sync(tsc, TSC210X_AUDIO3_CTRL);
	if (val < 0) {
		dev_dbg(dev, "%s, err %d\n", __FUNCTION__, val);
		return;
	}

	/* NOTE: write_sync() could fail */
	if (state)
		tsc210x_write_sync(tsc, TSC210X_AUDIO3_CTRL,
				val | TSC210X_SLVMS);
	else
		tsc210x_write_sync(tsc, TSC210X_AUDIO3_CTRL,
				val & ~TSC210X_SLVMS);
}
#endif	/* CONFIG_SOUND */

static int tsc210x_configure(struct tsc210x_dev *dev)
{
	/* NOTE: write_sync() could fail */

	/* Reset the chip */
	tsc210x_write_sync(dev, TSC210X_TS_RESET_CTRL, TSC210X_RESET);

	/* Reference mode */
	if (dev->pdata->use_internal)
		tsc210x_write_sync(dev,
				TSC210X_TS_REF_CTRL, TSC210X_ADC_INT_REF);
	else
		tsc210x_write_sync(dev,
				TSC210X_TS_REF_CTRL, TSC210X_ADC_EXT_REF);

	/* Precharge and sense delays, pen touch detection on */
	tsc210x_write_sync(dev, TSC210X_TS_CONFIG_CTRL, TSC210X_CONFIG_TIMES);

	/* PINT/DAV acts as DAV */
	tsc210x_write_sync(dev, TSC210X_TS_STATUS_CTRL, TSC210X_ADC_DAV);

	tsc210x_queue_scan(dev);
	return 0;
}

void tsc210x_keyclick(struct tsc210x_dev *dev,
		int amplitude, int freq, int length)
{
	int val;

	val = tsc210x_read_sync(dev, TSC210X_AUDIO2_CTRL);
	if (val < 0) {
		dev_dbg(&dev->spi->dev, "%s, err %d\n",
				__FUNCTION__, val);
		return;
	}
	val &= 0x800f;

	/* Set amplitude */
	switch (amplitude) {
	case 1:
		val |= 4 << 12;
		break;
	case 2:
		val |= 7 << 12;
		break;
	default:
		break;
	}

	/* Frequency */
	val |= (freq & 0x7) << 8;

	/* Round to nearest supported length */
	if (dev->kind == tsc2101)
		val = (min(length - 1, 31) >> 1) << 4;
	else {
		if (length > 20)
			val |= 4 << 4;
		else if (length > 6)
			val |= 3 << 4;
		else if (length > 4)
			val |= 2 << 4;
		else if (length > 2)
			val |= 1 << 4;
	}

	/* Enable keyclick */
	val |= 0x8000;

	/* NOTE: write_sync() could fail */
	tsc210x_write_sync(dev, TSC210X_AUDIO2_CTRL, val);
}
EXPORT_SYMBOL(tsc210x_keyclick);

#ifdef CONFIG_PM
/*
 * Suspend the chip.
 */
static int
tsc210x_suspend(struct spi_device *spi, pm_message_t state)
{
	struct tsc210x_dev *dev = dev_get_drvdata(&spi->dev);

	if (!dev)
		return -ENODEV;

	/* Stop the inputs scan loop */
	mutex_lock(&dev->queue_lock);
	dev->flushing = 1;
	cancel_delayed_work(&dev->sensor_worker);
	mutex_unlock(&dev->queue_lock);
	flush_workqueue(dev->queue);

	/* Wait until pen-up happens */
	while (dev->pendown)
		flush_workqueue(dev->queue);

	/* Abort current conversion and power down the ADC */
	tsc210x_write_sync(dev, TSC210X_TS_ADC_CTRL, TSC210X_ADC_ADST);
	/* NOTE: write_sync() could fail */

	return 0;
}

/*
 * Resume chip operation.
 */
static int tsc210x_resume(struct spi_device *spi)
{
	struct tsc210x_dev *dev = dev_get_drvdata(&spi->dev);
	int err;

	if (!dev)
		return 0;

	mutex_lock(&dev->queue_lock);
	err = tsc210x_configure(dev);

	dev->flushing = 0;
	mutex_unlock(&dev->queue_lock);

	return err;
}
#else
#define tsc210x_suspend	NULL
#define tsc210x_resume	NULL
#endif

/* REVISIT don't make these static */
static struct platform_device tsc210x_ts_device = {
	.name		= "tsc210x-ts",
	.id		= -1,
};

static struct platform_device tsc210x_hwmon_device = {
	.name		= "tsc210x-hwmon",
	.id		= -1,
};

static struct platform_device tsc210x_alsa_device = {
	.name		= "tsc210x-alsa",
	.id		= -1,
};

static int tsc210x_probe(struct spi_device *spi, enum tsc_type type)
{
	struct tsc210x_config *pdata = spi->dev.platform_data;
	struct spi_transfer *spi_buffer;
	struct tsc210x_dev *dev;
	int reg;
	int err = 0;

	if (!pdata) {
		dev_dbg(&spi->dev, "Platform data not supplied\n");
		return -ENOENT;
	}

	if (!spi->irq) {
		dev_dbg(&spi->dev, "Invalid irq value\n");
		return -EINVAL;
	}

	dev = (struct tsc210x_dev *)
		kzalloc(sizeof(struct tsc210x_dev), GFP_KERNEL);
	if (!dev) {
		dev_dbg(&spi->dev, "No memory\n");
		return -ENOMEM;
	}

	dev->pdata = pdata;
	dev->pendown = 0;
	dev->spi = spi;
	dev->kind = type;
	dev->queue = create_singlethread_workqueue(spi->dev.driver->name);
	if (!dev->queue) {
		dev_dbg(&spi->dev, "Can't make a workqueue\n");
		err = -ENOMEM;
		goto err_queue;
	}

	mutex_init(&dev->queue_lock);
	init_completion(&dev->data_avail);

	/* Allocate enough struct spi_transfer's for all requests */
	spi_buffer = kzalloc(sizeof(struct spi_transfer) * 16, GFP_KERNEL);
	if (!spi_buffer) {
		dev_dbg(&spi->dev, "No memory for SPI buffers\n");
		err = -ENOMEM;
		goto err_buffers;
	}

	dev->transfers = spi_buffer;
	tsc210x_request_alloc(dev, &dev->req_adc, 0,
			TSC210X_TS_X, 4, dev->adc_data,
			tsc210x_coords_report, &spi_buffer);
	tsc210x_request_alloc(dev, &dev->req_status, 0,
			TSC210X_TS_STATUS_CTRL, 1, &dev->status,
			tsc210x_status_report, &spi_buffer);
	tsc210x_request_alloc(dev, &dev->req_mode, 1,
			TSC210X_TS_ADC_CTRL, 1, NULL,
			tsc210x_complete_dummy, &spi_buffer);
	tsc210x_request_alloc(dev, &dev->req_stop, 1,
			TSC210X_TS_ADC_CTRL, 1, NULL,
			tsc210x_complete_dummy, &spi_buffer);

	if (pdata->bclk) {
		/* Get the BCLK */
		dev->bclk_ck = clk_get(&spi->dev, pdata->bclk);
		if (IS_ERR(dev->bclk_ck)) {
			err = PTR_ERR(dev->bclk_ck);
			dev_dbg(&spi->dev, "Unable to get '%s': %i\n",
					pdata->bclk, err);
			goto err_clk;
		}

		clk_enable(dev->bclk_ck);
	}

	INIT_DELAYED_WORK(&dev->ts_worker, tsc210x_pressure);
	INIT_DELAYED_WORK(&dev->sensor_worker, tsc210x_input_scan);

	/* Setup the communication bus */
	dev_set_drvdata(&spi->dev, dev);
	spi->mode = SPI_MODE_1;
	spi->bits_per_word = 16;
	err = spi_setup(spi);
	if (err)
		goto err_spi;

	/* Now try to detect the chip, make first contact.  These chips
	 * don't self-identify, but we can expect that the status register
	 * reports the ADC is idle and use that as a sanity check.  (It'd
	 * be even better if we did a soft reset first...)
	 */
	reg = tsc210x_read_sync(dev, TSC210X_TS_ADC_CTRL);
	if (reg < 0) {
		err = reg;
		dev_dbg(&dev->spi->dev, "adc_ctrl, err %d\n", err);
		goto err_spi;
	}
	if (!(reg & (1 << 14))) {
		err = -EIO;
		dev_dbg(&dev->spi->dev, "adc_ctrl, busy? - %04x\n", reg);
		goto err_spi;
	}

	reg = tsc210x_read_sync(dev, TSC210X_AUDIO3_CTRL);
	if (reg < 0) {
		err = reg;
		dev_dbg(&dev->spi->dev, "revision, err %d\n", err);
		goto err_spi;
	}
	if (reg == 0xffff) {
		err = -ENODEV;
		dev_dbg(&dev->spi->dev, "no device, err %d\n", err);
		goto err_spi;
	}
	dev_info(&spi->dev, "rev %d, irq %d\n", reg & 0x0007, spi->irq);

	err = tsc210x_configure(dev);
	if (err)
		goto err_spi;

	/* We want no interrupts before configuration succeeds.  */
	mutex_lock(&dev->queue_lock);
	dev->flushing = 1;

	if (request_irq(spi->irq, tsc210x_handler, IRQF_SAMPLE_RANDOM |
				IRQF_TRIGGER_FALLING, spi->dev.driver->name,
				dev)) {
		dev_dbg(&spi->dev, "Could not allocate touchscreen IRQ!\n");
		err = -EINVAL;
		goto err_irq;
	}

	/* Register subdevices controlled by the TSC 2101/2102 */
	tsc210x_ts_device.dev.platform_data = dev;
	tsc210x_ts_device.dev.parent = &spi->dev;
	err = platform_device_register(&tsc210x_ts_device);
	if (err)
		goto err_irq;

	tsc210x_hwmon_device.dev.platform_data = pdata;
	tsc210x_hwmon_device.dev.parent = &spi->dev;
	err = platform_device_register(&tsc210x_hwmon_device);
	if (err)
		goto err_hwmon;

	tsc210x_alsa_device.dev.platform_data = pdata->alsa_config;
	tsc210x_alsa_device.dev.parent = &spi->dev;
	err = platform_device_register(&tsc210x_alsa_device);
	if (err)
		goto err_alsa;

	dev->flushing = 0;
	mutex_unlock(&dev->queue_lock);
	return 0;

err_alsa:
	platform_device_unregister(&tsc210x_hwmon_device);
err_hwmon:
	platform_device_unregister(&tsc210x_ts_device);
err_irq:
	mutex_unlock(&dev->queue_lock);
err_spi:
	dev_set_drvdata(&spi->dev, NULL);
	clk_disable(dev->bclk_ck);
	clk_put(dev->bclk_ck);
err_clk:
	kfree(dev->transfers);
err_buffers:
	destroy_workqueue(dev->queue);
err_queue:
	kfree(dev);
	return err;
}

static int tsc2101_probe(struct spi_device *spi)
{
	return tsc210x_probe(spi, tsc2101);
}

static int tsc2102_probe(struct spi_device *spi)
{
	return tsc210x_probe(spi, tsc2102);
}

static int tsc210x_remove(struct spi_device *spi)
{
	struct tsc210x_dev *dev = dev_get_drvdata(&spi->dev);

	/* Stop the inputs scan loop */
	mutex_lock(&dev->queue_lock);
	dev->flushing = 1;
	cancel_delayed_work(&dev->sensor_worker);
	mutex_unlock(&dev->queue_lock);
	flush_workqueue(dev->queue);

	/* Wait for pen-up */
	while (dev->pendown)
		flush_workqueue(dev->queue);

	/* Abort current conversion and power down the ADC */
	tsc210x_write_sync(dev, TSC210X_TS_ADC_CTRL, TSC210X_ADC_ADST);
	/* NOTE: write_sync() could fail */

	destroy_workqueue(dev->queue);

	platform_device_unregister(&tsc210x_ts_device);
	platform_device_unregister(&tsc210x_hwmon_device);
	platform_device_unregister(&tsc210x_alsa_device);

	dev_set_drvdata(&spi->dev, NULL);

	/* Release the BCLK */
	clk_disable(dev->bclk_ck);
	clk_put(dev->bclk_ck);

	kfree(dev->transfers);
	kfree(dev);

	return 0;
}

static struct spi_driver tsc2101_driver = {
	.probe		= tsc2101_probe,
	.remove		= tsc210x_remove,
	.suspend	= tsc210x_suspend,
	.resume		= tsc210x_resume,
	.driver		= {
		.name	= "tsc2101",
		.owner	= THIS_MODULE,
		.bus	= &spi_bus_type,
	},
};

static struct spi_driver tsc2102_driver = {
	.probe		= tsc2102_probe,
	.remove		= tsc210x_remove,
	.suspend	= tsc210x_suspend,
	.resume		= tsc210x_resume,
	.driver		= {
		.name	= "tsc2102",
		.owner	= THIS_MODULE,
		.bus	= &spi_bus_type,
	},
};

static char __initdata banner[] = KERN_INFO "TI TSC210x driver initializing\n";

static int __init tsc210x_init(void)
{
	int err;
	printk(banner);

	settings.ts_msecs = 20;
	settings.mode_msecs = 1000;

	err = spi_register_driver(&tsc2101_driver);
	if (err != 0)
		return err;

	err = spi_register_driver(&tsc2102_driver);
	if (err != 0)
		spi_unregister_driver(&tsc2101_driver);

	return err;
}
module_init(tsc210x_init);

static void __exit tsc210x_exit(void)
{
	spi_unregister_driver(&tsc2101_driver);
	spi_unregister_driver(&tsc2102_driver);
}
module_exit(tsc210x_exit);

MODULE_AUTHOR("Andrzej Zaborowski");
MODULE_DESCRIPTION("Interface driver for TI TSC210x chips.");
MODULE_LICENSE("GPL");
