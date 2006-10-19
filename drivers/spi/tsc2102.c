/*
 * drivers/spi/tsc2102.c
 *
 * TSC2102 interface driver.
 *
 * Copyright (c) 2005 Andrzej Zaborowski  <balrog@zabor.org>
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
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc2102.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>

#ifdef CONFIG_APM
#include <asm/apm.h>
#endif

/* Bit field definitions for chip registers */
#define TSC2102_ADC_TS_CONTROL		0x8bf4
#define TSC2102_ADC_SCAN_CONTROL	0x2ff4
#define TSC2102_ADC_T1_CONTROL		0x2bf4
#define TSC2102_ADC_T2_CONTROL		0x33f4
#define TSC2102_ADC_DAV			0x4000
#define TSC2102_ADC_INT_REF		0x0016
#define TSC2102_ADC_EXT_REF		0x0002
#define TSC2102_CONFIG_TIMES		0x0008
#define TSC2102_RESET			0xbb00
#define TSC2102_ADC_PSTCM		(1 << 15)
#define TSC2102_ADC_ADST		(1 << 14)
#define TSC2102_TS_DAV			0x0780
#define TSC2102_PS_DAV			0x0078
#define TSC2102_T1_DAV			0x0004
#define TSC2102_T2_DAV			0x0002
#define TSC2102_DAC_ON			0x3ba0
#define TSC2102_DAC_OFF			0xafa0
#define TSC2102_FS44K			(1 << 13)
#define TSC2102_PLL1_OFF		0x0000
#define TSC2102_PLL1_44K		0x811c
#define TSC2102_PLL1_48K		0x8120
#define TSC2102_PLL2_44K		(5462 << 2)
#define TSC2102_PLL2_48K		(1920 << 2)
#define TSC2102_SLVMS			(1 << 11)
#define TSC2102_DEEMPF			(1 << 0)
#define TSC2102_BASSBC			(1 << 1)
#define TSC2102_KEYCLICK_OFF		0x0000

#define CS_CHANGE(val)			0

struct tsc2102_spi_req {
	struct spi_device *dev;
	uint16_t command;
	uint16_t data;
	struct spi_transfer *transfer;
	struct spi_message message;
};

struct tsc2102_dev {
	struct tsc2102_config *pdata;
	spinlock_t lock, lock_sync;
	struct clk *bclk_ck;

	int state;			/* 0: TS, 1: Portscan, 2-3: Temps */
	struct timer_list ts_timer;	/* Busy-wait for PEN UP */
	struct timer_list mode_timer;	/* Change .state every some time */
	int pendown;
	int data_pending;
	uint16_t status, adc_status, adc_data[4];
	tsc2102_touch_t touch_cb;
	tsc2102_coords_t coords_cb;
	tsc2102_ports_t ports_cb;
	tsc2102_temp_t temp1_cb;
	tsc2102_temp_t temp2_cb;
	unsigned int ts_msecs;		/* Interval for .ts_timer */
	unsigned int mode_msecs;	/* Interval for .mode_timer */

	struct spi_device *spi;
	struct spi_transfer *transfers;
	struct tsc2102_spi_req req_adc;
	struct tsc2102_spi_req req_status;
	struct tsc2102_spi_req req_pressure;
	struct tsc2102_spi_req req_stopadc;
	struct tsc2102_spi_req req_mode;

	int bat[2], aux[1], temp[2];
	struct class_device *hwmondev;
};

static struct tsc2102_dev tsc;

module_param_named(touch_check_msecs, tsc.ts_msecs, uint, 0);
MODULE_PARM_DESC(touch_check_msecs, "Pen-up polling interval in msecs");

module_param_named(sensor_scan_msecs, tsc.mode_msecs, uint, 0);
MODULE_PARM_DESC(sensor_scan_msecs, "Temperature & battery scan interval");

void tsc2102_write_sync(int page, u8 address, u16 data)
{
	static struct tsc2102_spi_req req;
	static struct spi_transfer transfer[2];
	int ret;

	spi_message_init(&req.message);
	req.transfer = transfer;

	/* Address */
	req.command = (page << 11) | (address << 5);
	req.transfer[0].tx_buf = &req.command;
	req.transfer[0].rx_buf = 0;
	req.transfer[0].len = 2;
	spi_message_add_tail(&req.transfer[0], &req.message);

	/* Data */
	req.transfer[1].tx_buf = &data;
	req.transfer[1].rx_buf = 0;
	req.transfer[1].len = 2;
	req.transfer[1].cs_change = CS_CHANGE(1);
	spi_message_add_tail(&req.transfer[1], &req.message);

	ret = spi_sync(tsc.spi, &req.message);
	if (!ret && req.message.status)
		ret = req.message.status;

	if (ret)
		printk(KERN_ERR "%s: error %i in SPI request\n",
				__FUNCTION__, ret);
}

void tsc2102_reads_sync(int page, u8 startaddress, u16 *data, int numregs)
{
	static struct tsc2102_spi_req req;
	static struct spi_transfer transfer[6];
	int ret, i, j;

	BUG_ON(numregs + 1 > ARRAY_SIZE(transfer));

	spi_message_init(&req.message);
	req.transfer = transfer;
	i = 0;
	j = 0;

	/* Address */
	req.command = 0x8000 | (page << 11) | (startaddress << 5);
	req.transfer[i].tx_buf = &req.command;
	req.transfer[i].rx_buf = 0;
	req.transfer[i].len = 2;
	spi_message_add_tail(&req.transfer[i ++], &req.message);

	/* Data */
	while (j < numregs) {
		req.transfer[i].tx_buf = 0;
		req.transfer[i].rx_buf = &data[j ++];
		req.transfer[i].len = 2;
		req.transfer[i].cs_change = CS_CHANGE(j == numregs);
		spi_message_add_tail(&req.transfer[i ++], &req.message);
	}

	ret = spi_sync(tsc.spi, &req.message);
	if (!ret && req.message.status)
		ret = req.message.status;

	if (ret)
		printk(KERN_ERR "%s: error %i in SPI request\n",
				__FUNCTION__, ret);
}

u16 tsc2102_read_sync(int page, u8 address)
{
	u16 ret;
	tsc2102_reads_sync(page, address, &ret, 1);
	return ret;
}

static void tsc2102_write_async(
		struct tsc2102_spi_req *spi, int page, u8 address, u16 data,
		void (*complete)(struct tsc2102_dev *context))
{
	int ret;

	spi_message_init(&spi->message);
	spi->message.complete = (void (*)(void *)) complete;
	spi->message.context = &tsc;

	/* Address */
	spi->command = (page << 11) | (address << 5);
	spi->transfer[0].tx_buf = &spi->command;
	spi->transfer[0].rx_buf = 0;
	spi->transfer[0].len = 2;
	spi_message_add_tail(&spi->transfer[0], &spi->message);

	/* Data */
	spi->data = data;
	spi->transfer[1].tx_buf = &spi->data;
	spi->transfer[1].rx_buf = 0;
	spi->transfer[1].len = 2;
	spi->transfer[1].cs_change = CS_CHANGE(1);
	spi_message_add_tail(&spi->transfer[1], &spi->message);

	ret = spi_async(spi->dev, &spi->message);
	if (ret)
		printk(KERN_ERR "%s: error %i in SPI request\n",
				__FUNCTION__, ret);
}

static void tsc2102_reads_async(struct tsc2102_spi_req *spi,
		int page, u8 startaddress, u16 *data, int numregs,
		void (*complete)(struct tsc2102_dev *context))
{
	int ret, i, j;

	spi_message_init(&spi->message);
	spi->message.complete = (void (*)(void *)) complete;
	spi->message.context = &tsc;
	i = 0;
	j = 0;

	/* Address */
	spi->command = 0x8000 | (page << 11) | (startaddress << 5);
	spi->transfer[i].tx_buf = &spi->command;
	spi->transfer[i].rx_buf = 0;
	spi->transfer[i].len = 2;
	spi_message_add_tail(&spi->transfer[i ++], &spi->message);

	/* Data */
	while (j < numregs) {
		spi->transfer[i].tx_buf = 0;
		spi->transfer[i].rx_buf = &data[j ++];
		spi->transfer[i].len = 2;
		spi->transfer[i].cs_change = CS_CHANGE(j == numregs);
		spi_message_add_tail(&spi->transfer[i ++], &spi->message);
	}

	ret = spi_async(spi->dev, &spi->message);
	if (ret)
		printk(KERN_ERR "%s: error %i in SPI request\n",
				__FUNCTION__, ret);
}

static void tsc2102_read_async(struct tsc2102_spi_req *spi,
		int page, u8 address, u16 *ret,
		void (*complete)(struct tsc2102_dev *context))
{
	tsc2102_reads_async(spi, page, address, ret, 1, complete);
}

static void tsc2102_request_alloc(struct tsc2102_dev *dev,
		struct tsc2102_spi_req *spi, int direction, int numregs,
		struct spi_transfer **buffer)
{
	spi->dev = dev->spi;

	if (direction == 1)	/* Write */
		numregs = 2;
	else			/* Read */
		numregs += 1;

	spi->transfer = *buffer;
	*buffer += numregs;
}

#define tsc2102_cb_register_func(cb, cb_t)	\
int tsc2102_ ## cb(cb_t handler)	\
{	\
	spin_lock(&tsc.lock);	\
	\
	/* Lock the module */	\
	if (handler && !tsc.cb)	\
		if (!try_module_get(THIS_MODULE)) {	\
			printk(KERN_INFO "Failed to get TSC module\n");	\
		}	\
	if (!handler && tsc.cb)	\
		module_put(THIS_MODULE);	\
	\
	tsc.cb = handler;	\
	\
	spin_unlock(&tsc.lock);	\
	return 0;	\
}

tsc2102_cb_register_func(touch_cb, tsc2102_touch_t)
tsc2102_cb_register_func(coords_cb, tsc2102_coords_t)
tsc2102_cb_register_func(ports_cb, tsc2102_ports_t)
tsc2102_cb_register_func(temp1_cb, tsc2102_temp_t)
tsc2102_cb_register_func(temp2_cb, tsc2102_temp_t)

#ifdef DEBUG
static void tsc2102_print_dav(void)
{
	u16 status = tsc2102_read_sync(TSC2102_TS_STATUS_CTRL);
	if (status & 0x0fff)
		printk("TSC2102: data in");
	if (status & 0x0400)
		printk(" X");
	if (status & 0x0200)
		printk(" Y");
	if (status & 0x0100)
		printk(" Z1");
	if (status & 0x0080)
		printk(" Z2");
	if (status & 0x0040)
		printk(" BAT1");
	if (status & 0x0020)
		printk(" BAT2");
	if (status & 0x0010)
		printk(" AUX1");
	if (status & 0x0008)
		printk(" AUX2");
	if (status & 0x0004)
		printk(" TEMP1");
	if (status & 0x0002)
		printk(" TEMP2");
	if (status & 0x0001)
		printk(" KP");
	if (status & 0x0fff)
		printk(".\n");
}
#endif

static void tsc2102_complete_dummy(struct tsc2102_dev *dev)
{
}

static inline void tsc2102_touchscreen_mode(struct tsc2102_dev *dev)
{
	/* Scan X, Y, Z1, Z2, chip controlled, 12-bit, 16 samples, 500 usec */
	tsc2102_write_async(&dev->req_mode,
			TSC2102_TS_ADC_CTRL, TSC2102_ADC_TS_CONTROL,
			tsc2102_complete_dummy);
}

static inline void tsc2102_portscan_mode(struct tsc2102_dev *dev)
{
	/* Scan BAT1, BAT2, AUX, 12-bit, 16 samples, 500 usec */
	tsc2102_write_async(&dev->req_mode,
			TSC2102_TS_ADC_CTRL, TSC2102_ADC_SCAN_CONTROL,
			tsc2102_complete_dummy);
}

static inline void tsc2102_temp1_mode(struct tsc2102_dev *dev)
{
	/* Scan TEMP1, 12-bit, 16 samples, 500 usec */
	tsc2102_write_async(&dev->req_mode,
			TSC2102_TS_ADC_CTRL, TSC2102_ADC_T1_CONTROL,
			tsc2102_complete_dummy);
}

static inline void tsc2102_temp2_mode(struct tsc2102_dev *dev)
{
	/* Scan TEMP2, 12-bit, 16 samples, 500 usec */
	tsc2102_write_async(&dev->req_mode,
			TSC2102_TS_ADC_CTRL, TSC2102_ADC_T2_CONTROL,
			tsc2102_complete_dummy);
}

static void tsc2102_mode(struct tsc2102_dev *dev)
{
	switch (dev->state) {
	case 0:
		tsc2102_touchscreen_mode(dev);
		break;
	case 1:
		tsc2102_portscan_mode(dev);
		break;
	case 2:
		tsc2102_temp1_mode(dev);
		break;
	case 3:
		tsc2102_temp2_mode(dev);
		break;
	default:
		dev->state = 0;
		tsc2102_touchscreen_mode(dev);
		break;
	}
}

/* Lock is held when this is called.  */
static void tsc2102_new_mode(struct tsc2102_dev *dev)
{
	/* Abort current conversion if any */
	tsc2102_write_async(&dev->req_stopadc,
			TSC2102_TS_ADC_CTRL, TSC2102_ADC_ADST,
			tsc2102_complete_dummy);

	dev->state ++;
	tsc2102_mode(dev);
}

static void tsc2102_check_status(struct tsc2102_dev *dev);

/* TSC has new data for us availiable.  */
static irqreturn_t
tsc2102_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	struct tsc2102_dev *dev = (struct tsc2102_dev *) dev_id;
	spin_lock_irq(&dev->lock);

	if (!dev->data_pending)
		tsc2102_check_status(dev);

	dev->data_pending ++;

	spin_unlock_irq(&dev->lock);
	return IRQ_HANDLED;
}

static void tsc2102_data_report(struct tsc2102_dev *dev)
{
	if (dev->status & TSC2102_TS_DAV) {
		if (dev->coords_cb)
			dev->coords_cb(
					dev->adc_data[0], dev->adc_data[1],
					dev->adc_data[2], dev->adc_data[3]);
	}

	if (dev->status & TSC2102_PS_DAV) {
		if (dev->ports_cb)
			dev->ports_cb(dev->adc_data[0],
					dev->adc_data[1], dev->adc_data[2]);
		dev->bat[0] = dev->adc_data[0];
		dev->bat[1] = dev->adc_data[1];
		dev->aux[0] = dev->adc_data[2];
	}

	if (dev->status & TSC2102_T1_DAV) {
		if (dev->temp1_cb)
			dev->temp1_cb(*dev->adc_data);
		dev->temp[0] = *dev->adc_data;
	}

	if (dev->status & TSC2102_T2_DAV) {
		if (dev->temp2_cb)
			dev->temp2_cb(*dev->adc_data);
		dev->temp[1] = *dev->adc_data;
	}

	spin_lock_irq(&dev->lock);

	dev->data_pending --;

	/*
	 * This may happen if the registers were successfully read and a
	 * new conversion was started and completed by the TSC before the
	 * completion for SPI read was called.
	 */
	if (dev->data_pending)
		tsc2102_check_status(dev);

	if (dev->status & (TSC2102_PS_DAV | TSC2102_T1_DAV | TSC2102_T2_DAV))
		tsc2102_new_mode(dev);

	spin_unlock_irq(&dev->lock);
}

static void tsc2102_status_report(struct tsc2102_dev *dev)
{
	/*
	 * Read all converted data from corresponding registers
	 * so that the ADC can move on to a new conversion.
	 */
	if (dev->status & TSC2102_TS_DAV) {
		tsc2102_reads_async(&dev->req_adc, TSC2102_TS_X,
				dev->adc_data, 4, tsc2102_data_report);
		if (!dev->pendown) {
			dev->pendown = 1;
			if (dev->touch_cb)
				dev->touch_cb(1);

			mod_timer(&dev->ts_timer, jiffies +
				msecs_to_jiffies(dev->ts_msecs));
		}
	}

	if (dev->status & TSC2102_PS_DAV) {
		tsc2102_reads_async(&dev->req_adc, TSC2102_TS_BAT1,
				dev->adc_data, 3, tsc2102_data_report);
	}

	if (dev->status & TSC2102_T1_DAV) {
		tsc2102_read_async(&dev->req_adc, TSC2102_TS_TEMP1,
				dev->adc_data, tsc2102_data_report);
	}

	if (dev->status & TSC2102_T2_DAV) {
		tsc2102_read_async(&dev->req_adc, TSC2102_TS_TEMP2,
				dev->adc_data, tsc2102_data_report);
	}

	if (!(dev->status & (TSC2102_TS_DAV | TSC2102_PS_DAV |
					TSC2102_T1_DAV | TSC2102_T2_DAV))) {
		spin_lock_irq(&dev->lock);
		dev->data_pending --;
		spin_unlock_irq(&dev->lock);

		WARN_ON(!dev->state);
	}
}

static void tsc2102_check_status(struct tsc2102_dev *dev)
{
	tsc2102_read_async(&dev->req_status, TSC2102_TS_STATUS_CTRL,
			&dev->status, tsc2102_status_report);
}

static void tsc2102_mode_timer(unsigned long data)
{
	struct tsc2102_dev *dev = (struct tsc2102_dev *) data;
	spin_lock_irq(&dev->lock);

	BUG_ON(dev->state);

	tsc2102_new_mode(dev);

	mod_timer(&dev->mode_timer, jiffies +
			msecs_to_jiffies(dev->mode_msecs));
	spin_unlock_irq(&dev->lock);
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
 * for a moment when all three indicate a pen-up, using a timer, before
 * we report a pen-up.
 */
static void tsc2102_pressure_report(struct tsc2102_dev *dev)
{
	if (!dev->pendown)
		return;

	if (dev->state ||
			(dev->adc_status & TSC2102_ADC_PSTCM) ||
			!(dev->adc_status & TSC2102_ADC_ADST)) {
		mod_timer(&dev->ts_timer, jiffies +
				msecs_to_jiffies(dev->ts_msecs));
	} else {
		dev->pendown = 0;
		if (dev->touch_cb)
			dev->touch_cb(0);
	}
}

static void tsc2102_pressure(unsigned long data)
{
	struct tsc2102_dev *dev = (struct tsc2102_dev *) data;

	BUG_ON(!dev->pendown);

	tsc2102_read_async(&dev->req_pressure, TSC2102_TS_ADC_CTRL,
			&dev->adc_status, tsc2102_pressure_report);
}

#ifdef CONFIG_SOUND
/*
 * Volume level values should be in the range [0, 127].
 * Higher values mean lower volume.
 */
void tsc2102_set_volume(uint8_t left_ch, uint8_t right_ch)
{
	u16 val;
	if (left_ch == 0x00 || left_ch == 0x7f)	/* All 0's or all 1's */
		left_ch ^= 0x7f;
	if (right_ch == 0x00 || right_ch == 0x7f)
		right_ch ^= 0x7f;

	spin_lock(&tsc.lock_sync);

	val = tsc2102_read_sync(TSC2102_DAC_GAIN_CTRL);

	val &= 0x8080;	/* Preserve mute-bits */
	val |= (left_ch << 8) | right_ch;

	tsc2102_write_sync(TSC2102_DAC_GAIN_CTRL, val);

	spin_unlock(&tsc.lock_sync);
}

void tsc2102_set_mute(int left_ch, int right_ch)
{
	u16 val;
	spin_lock(&tsc.lock_sync);

	val = tsc2102_read_sync(TSC2102_DAC_GAIN_CTRL);

	val &= 0x7f7f;	/* Preserve volume settings */
	val |= (left_ch << 15) | (right_ch << 7);

	tsc2102_write_sync(TSC2102_DAC_GAIN_CTRL, val);

	spin_unlock(&tsc.lock_sync);
}

void tsc2102_get_mute(int *left_ch, int *right_ch)
{
	u16 val;
	spin_lock(&tsc.lock_sync);

	val = tsc2102_read_sync(TSC2102_DAC_GAIN_CTRL);

	spin_unlock(&tsc.lock_sync);

	*left_ch = !!(val & (1 << 15));
	*right_ch = !!(val & (1 << 7));
}

void tsc2102_set_deemphasis(int enable)
{
	u16 val;
	spin_lock(&tsc.lock_sync);
	val = tsc2102_read_sync(TSC2102_DAC_POWER_CTRL);

	if (enable)
		val &= ~TSC2102_DEEMPF;
	else
		val |= TSC2102_DEEMPF;

	tsc2102_write_sync(TSC2102_DAC_POWER_CTRL, val);
	spin_unlock(&tsc.lock_sync);
}

void tsc2102_set_bassboost(int enable)
{
	u16 val;
	spin_lock(&tsc.lock_sync);
	val = tsc2102_read_sync(TSC2102_DAC_POWER_CTRL);

	if (enable)
		val &= ~TSC2102_BASSBC;
	else
		val |= TSC2102_BASSBC;

	tsc2102_write_sync(TSC2102_DAC_POWER_CTRL, val);
	spin_unlock(&tsc.lock_sync);
}

/*	{rate, dsor, fsref}	*/
static const struct tsc2102_rate_info_s tsc2102_rates[] = {
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

	{0,	0, 	0},
};

int tsc2102_set_rate(int rate)
{
	int i;
	uint16_t val;

	for (i = 0; tsc2102_rates[i].sample_rate; i ++)
		if (tsc2102_rates[i].sample_rate == rate)
			break;
	if (tsc2102_rates[i].sample_rate == 0) {
		printk(KERN_ERR "Unknown sampling rate %i.0 Hz\n", rate);
		return -EINVAL;
	}

	spin_lock(&tsc.lock_sync);

	tsc2102_write_sync(TSC2102_AUDIO1_CTRL, tsc2102_rates[i].divisor);

	val = tsc2102_read_sync(TSC2102_AUDIO3_CTRL);

	if (tsc2102_rates[i].fs_44k) {
		tsc2102_write_sync(TSC2102_AUDIO3_CTRL, val | TSC2102_FS44K);
		/* Enable Phase-locked-loop, set up clock dividers */
		tsc2102_write_sync(TSC2102_PLL1_CTRL, TSC2102_PLL1_44K);
		tsc2102_write_sync(TSC2102_PLL2_CTRL, TSC2102_PLL2_44K);
	} else {
		tsc2102_write_sync(TSC2102_AUDIO3_CTRL, val & ~TSC2102_FS44K);
		/* Enable Phase-locked-loop, set up clock dividers */
		tsc2102_write_sync(TSC2102_PLL1_CTRL, TSC2102_PLL1_48K);
		tsc2102_write_sync(TSC2102_PLL2_CTRL, TSC2102_PLL2_48K);
	}

	spin_unlock(&tsc.lock_sync);
	return 0;
}

/*
 * Perform basic set-up with default values and power the DAC on.
 */
void tsc2102_dac_power(int state)
{
	spin_lock(&tsc.lock_sync);

	if (state) {
		/* 16-bit words, DSP mode, sample at Fsref */
		tsc2102_write_sync(TSC2102_AUDIO1_CTRL, 0x0100);
		/* Keyclicks off, soft-stepping at normal rate */
		tsc2102_write_sync(TSC2102_AUDIO2_CTRL, TSC2102_KEYCLICK_OFF);
		/* 44.1 kHz Fsref, continuous transfer mode, master DAC */
		tsc2102_write_sync(TSC2102_AUDIO3_CTRL, 0x2800);
		/* Soft-stepping enabled */
		tsc2102_write_sync(TSC2102_AUDIO4_CTRL, 0x0000);

		/* PLL generates 44.1 kHz */
		tsc2102_write_sync(TSC2102_PLL1_CTRL, TSC2102_PLL1_44K);
		tsc2102_write_sync(TSC2102_PLL2_CTRL, TSC2102_PLL2_44K);

		/* Codec & DAC power up, virtual ground disabled */
		tsc2102_write_sync(TSC2102_DAC_POWER_CTRL, TSC2102_DAC_ON);
	} else {
		/* All off */
		tsc2102_write_sync(TSC2102_AUDIO4_CTRL, TSC2102_KEYCLICK_OFF);
		tsc2102_write_sync(TSC2102_PLL1_CTRL, TSC2102_PLL1_OFF);
	}

	spin_unlock(&tsc.lock_sync);
}

void tsc2102_set_i2s_master(int state)
{
	uint16_t val;
	spin_lock(&tsc.lock_sync);

	val = tsc2102_read_sync(TSC2102_AUDIO3_CTRL);

	if (state)
		tsc2102_write_sync(TSC2102_AUDIO3_CTRL, val | TSC2102_SLVMS);
	else
		tsc2102_write_sync(TSC2102_AUDIO3_CTRL, val & ~TSC2102_SLVMS);

	spin_unlock(&tsc.lock_sync);
}
#endif	/* CONFIG_SOUND */

static int tsc2102_configure(struct tsc2102_dev *dev)
{
	/* Reset the chip */
	tsc2102_write_sync(TSC2102_TS_RESET_CTRL, TSC2102_RESET);

	/* Reference mode, 100 usec delay, 1.25 V reference */
	if (dev->pdata->use_internal)
		tsc2102_write_sync(TSC2102_TS_REF_CTRL, TSC2102_ADC_INT_REF);
	else
		tsc2102_write_sync(TSC2102_TS_REF_CTRL, TSC2102_ADC_EXT_REF);

	/* 84 usec precharge time, 32 usec sense time */
	tsc2102_write_sync(TSC2102_TS_CONFIG_CTRL, TSC2102_CONFIG_TIMES);

	/* PINT/DAV acts as DAV */
	tsc2102_write_sync(TSC2102_TS_STATUS_CTRL, TSC2102_ADC_DAV);

	tsc2102_mode(dev);
	mod_timer(&dev->mode_timer, jiffies +
			msecs_to_jiffies(dev->mode_msecs));
	return 0;
}

/*
 * Retrieves chip revision.  Should be always 1.
 */
int tsc2102_get_revision(void)
{
	return tsc2102_read_sync(TSC2102_AUDIO3_CTRL) & 7;
}

/*
 * Emit a short keyclick typically in order to give feedback to
 * user on specific events.
 *
 * amplitude must be between 0 (lowest) and 2 (highest).
 * freq must be between 0 (corresponds to 62.5 Hz) and 7 (8 kHz).
 * length should be between 2 and 32 periods.
 *
 * This function sleeps but doesn't sleep until the sound has
 * finished.
 */
void tsc2102_keyclick(int amplitude, int freq, int length)
{
	u16 val;
	spin_lock(&tsc.lock_sync);
	val = tsc2102_read_sync(TSC2102_AUDIO2_CTRL);
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
	if (length > 20)
		val |= 4 << 4;
	else if (length > 6)
		val |= 3 << 4;
	else if (length > 4)
		val |= 2 << 4;
	else if (length > 2)
		val |= 1 << 4;

	/* Enable keyclick */
	val |= 0x8000;

	tsc2102_write_sync(TSC2102_AUDIO2_CTRL, val);
	spin_unlock(&tsc.lock_sync);
}

#ifdef CONFIG_HWMON
#define TSC2102_INPUT(devname, field)	\
static ssize_t show_ ## devname(struct device *dev,	\
		struct device_attribute *devattr, char *buf)	\
{	\
	struct tsc2102_dev *devhwmon = dev_get_drvdata(dev);	\
	int value = devhwmon->field;	\
	return sprintf(buf, "%i\n", value);	\
}	\
static DEVICE_ATTR(devname ## _input, S_IRUGO, show_ ## devname, NULL);

TSC2102_INPUT(in0, bat[0])
TSC2102_INPUT(in1, bat[1])
TSC2102_INPUT(in2, aux[0])
TSC2102_INPUT(in3, temp[0])
TSC2102_INPUT(in4, temp[1])

static ssize_t show_temp1(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct tsc2102_dev *devhwmon = dev_get_drvdata(dev);
	int t1, t2;
	int value, diff;

	t1 = devhwmon->temp[0];
	t2 = devhwmon->temp[1];

	/*
	 * Use method #2 (differential) to calculate current temperature.
	 * The difference between TEMP2 and TEMP1 input values is
	 * multiplied by a constant to obtain current temperature.
	 * To find this constant we use the values measured at 25 C as
	 * thermometer calibration data.
	 *
	 * 298150 is 25 degrees Celcius represented in Kelvins and
	 * multiplied by 1000 for fixed point precision (273.15 + 25).
	 * 273150 is zero degrees Celcius.
	 */
	diff = devhwmon->pdata->temp_at25c[1] - devhwmon->pdata->temp_at25c[0];
	BUG_ON(diff == 0);
	value = (t2 - t1) * 298150 / diff;	/* This is in Kelvins now */

	t1 = value - 273150;			/* Celcius millidegree */
	return sprintf(buf, "%i\n", t1);
}
static DEVICE_ATTR(temp1_input, S_IRUGO, show_temp1, NULL);
#endif	/* CONFIG_HWMON */

#ifdef CONFIG_APM
static void tsc2102_get_power_status(struct apm_power_info *info)
{
	tsc.pdata->apm_report(info, tsc.bat);
}
#endif

#ifdef CONFIG_PM
/*
 * Suspend the chip.
 */
static int
tsc2102_suspend(struct spi_device *spi, pm_message_t state)
{
	struct tsc2102_dev *dev = dev_get_drvdata(&spi->dev);

	if (!dev)
		return 0;

	spin_lock(&dev->lock_sync);

	del_timer(&dev->mode_timer);
	del_timer(&dev->ts_timer);

	if (dev->pendown && dev->touch_cb)
		dev->touch_cb(0);

	/* Abort current conversion and power down the ADC */
	tsc2102_write_sync(TSC2102_TS_ADC_CTRL, TSC2102_ADC_ADST);

	dev->spi->dev.power.power_state = state;

	spin_unlock(&dev->lock_sync);
	return 0;
}

/*
 * Resume chip operation.
 */
static int tsc2102_resume(struct spi_device *spi)
{
	struct tsc2102_dev *dev = dev_get_drvdata(&spi->dev);
	int err;

	if (!dev)
		return 0;

	spin_lock(&dev->lock_sync);

	dev->spi->dev.power.power_state = PMSG_ON;

	dev->state = 0;
	dev->pendown = 0;

	err = tsc2102_configure(dev);

	spin_unlock(&dev->lock_sync);
	return err;
}
#else
#define tsc2102_suspend	NULL
#define tsc2102_resume	NULL
#endif

static struct platform_device tsc2102_ts_device = {
	.name 		= "tsc2102-ts",
	.id 		= -1,
};

static struct platform_device tsc2102_alsa_device = {
	.name 		= "tsc2102-alsa",
	.id 		= -1,
};

static int tsc2102_probe(struct spi_device *spi)
{
	struct tsc2102_config *pdata = spi->dev.platform_data;
	struct spi_transfer *spi_buffer;
	int err = 0;

	if (!pdata) {
		printk(KERN_ERR "TSC2102: Platform data not supplied\n");
		return -ENOENT;
	}

	if (!spi->irq) {
		printk(KERN_ERR "TSC2102: Invalid irq value\n");
		return -ENOENT;
	}

	tsc.pdata = pdata;
	tsc.state = 0;
	tsc.pendown = 0;
	tsc.data_pending = 0;
	tsc.ts_msecs = 20;
	tsc.mode_msecs = 1000;
	tsc.spi = spi;

	/* Allocate enough struct spi_transfer's for all requests */
	spi_buffer = kzalloc(sizeof(struct spi_transfer) * 16, GFP_KERNEL);
	if (!spi_buffer) {
		printk(KERN_ERR "TSC2102: No memory for SPI buffers\n");
		return -ENOMEM;
	}

	tsc.transfers = spi_buffer;
	tsc2102_request_alloc(&tsc, &tsc.req_adc, 0, 4, &spi_buffer);
	tsc2102_request_alloc(&tsc, &tsc.req_status, 0, 1, &spi_buffer);
	tsc2102_request_alloc(&tsc, &tsc.req_pressure, 0, 1, &spi_buffer);
	tsc2102_request_alloc(&tsc, &tsc.req_stopadc, 1, 1, &spi_buffer);
	tsc2102_request_alloc(&tsc, &tsc.req_mode, 1, 1, &spi_buffer);

	spin_lock_init(&tsc.lock);
	spin_lock(&tsc.lock_sync);

	/* Get the BCLK - assuming the rate is at 12000000 */
	tsc.bclk_ck = clk_get(0, "bclk");
	if (!tsc.bclk_ck) {
		printk(KERN_ERR "Unable to get the clock BCLK\n");
		err = -EPERM;
		goto done;
	}

	clk_enable(tsc.bclk_ck);

	if (request_irq(spi->irq, tsc2102_handler, SA_SAMPLE_RANDOM |
				SA_TRIGGER_FALLING, "tsc2102", &tsc)) {
		printk(KERN_ERR "Could not allocate touchscreen IRQ!\n");
		err = -EINVAL;
		goto err_clk;
	}

	setup_timer(&tsc.ts_timer,
			tsc2102_pressure, (unsigned long) &tsc);
	setup_timer(&tsc.mode_timer,
			tsc2102_mode_timer, (unsigned long) &tsc);

	/* Set up the communication bus */
	dev_set_drvdata(&spi->dev, &tsc);
	spi->dev.power.power_state = PMSG_ON;
	spi->mode = SPI_MODE_1;
	spi->bits_per_word = 16;
	err = spi_setup(spi);
	if (err)
		goto err_timer;

	/* Now try to detect the chip, make first contact */
	if (tsc2102_get_revision() != 0x1) {
		printk(KERN_ERR "No TI TSC2102 chip found!\n");
		goto err_timer;
	}

	err = tsc2102_configure(&tsc);
	if (err)
		goto err_timer;

	/* Register devices controlled by TSC 2102 */
	tsc2102_ts_device.dev.platform_data = pdata;
	tsc2102_ts_device.dev.parent = &spi->dev;
	err = platform_device_register(&tsc2102_ts_device);
	if (err)
		goto err_timer;

	tsc2102_alsa_device.dev.platform_data = pdata->alsa_config;
	tsc2102_alsa_device.dev.parent = &spi->dev;
	err = platform_device_register(&tsc2102_alsa_device);
	if (err)
		goto err_ts;

#ifdef CONFIG_HWMON
	tsc.hwmondev = hwmon_device_register(&spi->dev);
	if (IS_ERR(tsc.hwmondev)) {
		printk(KERN_ERR "tsc2102_hwmon: Device registration failed\n");
		err = PTR_ERR(tsc.hwmondev);
		goto err_alsa;
	}

	if (pdata->monitor & TSC_BAT1)
		err |= device_create_file(&spi->dev, &dev_attr_in0_input);
	if (pdata->monitor & TSC_BAT2)
		err |= device_create_file(&spi->dev, &dev_attr_in1_input);
	if (pdata->monitor & TSC_AUX)
		err |= device_create_file(&spi->dev, &dev_attr_in2_input);
	if (pdata->monitor & TSC_TEMP) {
		err |= device_create_file(&spi->dev, &dev_attr_temp1_input);
		err |= device_create_file(&spi->dev, &dev_attr_in3_input);
		err |= device_create_file(&spi->dev, &dev_attr_in4_input);
	}

	if (err)
		printk(KERN_ERR "tsc2102_hwmon: Creating one or more "
				"attribute files failed\n");
	err = 0;	/* Not fatal */
#endif

#ifdef CONFIG_APM
	if (pdata->apm_report)
		apm_get_power_status = tsc2102_get_power_status;
#endif

	if (!err)
		goto done;

err_alsa:
	platform_device_unregister(&tsc2102_alsa_device);
err_ts:
	platform_device_unregister(&tsc2102_ts_device);
err_timer:
	del_timer(&tsc.ts_timer);
	del_timer(&tsc.mode_timer);
	dev_set_drvdata(&spi->dev, NULL);
err_clk:
	clk_disable(tsc.bclk_ck);
	clk_put(tsc.bclk_ck);
done:
	spin_unlock(&tsc.lock_sync);
	return err;
}

static int tsc2102_remove(struct spi_device *spi)
{
	struct tsc2102_dev *dev = dev_get_drvdata(&spi->dev);

	spin_lock(&dev->lock_sync);

	platform_device_unregister(&tsc2102_ts_device);
	platform_device_unregister(&tsc2102_alsa_device);

	dev_set_drvdata(&spi->dev, NULL);

	/* Release the BCLK */
	clk_disable(dev->bclk_ck);
	clk_put(dev->bclk_ck);

	del_timer(&tsc.mode_timer);
	del_timer(&tsc.ts_timer);

	kfree(tsc.transfers);

#ifdef CONFIG_HWMON
	hwmon_device_unregister(dev->hwmondev);
#endif

#ifdef CONFIG_APM
	apm_get_power_status = 0;
#endif

	spin_unlock(&dev->lock_sync);

	return 0;
}

static struct spi_driver tsc2102_driver = {
	.probe		= tsc2102_probe,
	.remove		= tsc2102_remove,
	.suspend	= tsc2102_suspend,
	.resume		= tsc2102_resume,
	.driver		= {
		.name	= "tsc2102",
		.owner	= THIS_MODULE,
		.bus	= &spi_bus_type,
	},
};

static char __initdata banner[] = KERN_INFO "TI TSC2102 driver initializing\n";

static int __init tsc2102_init(void)
{
	printk(banner);
	return spi_register_driver(&tsc2102_driver);
}

static void __exit tsc2102_exit(void)
{
	spi_unregister_driver(&tsc2102_driver);
}

module_init(tsc2102_init);
module_exit(tsc2102_exit);

EXPORT_SYMBOL(tsc2102_read_sync);
EXPORT_SYMBOL(tsc2102_reads_sync);
EXPORT_SYMBOL(tsc2102_write_sync);
EXPORT_SYMBOL(tsc2102_keyclick);

MODULE_AUTHOR("Andrzej Zaborowski");
MODULE_DESCRIPTION("Interface driver for TI TSC2102 chips.");
MODULE_LICENSE("GPL");
