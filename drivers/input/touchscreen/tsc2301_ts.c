/*
 * TSC2301 touchscreen driver
 *
 * Copyright (C) 2005-2008 Nokia Corporation
 *
 * Written by Jarkko Oikarinen, Imre Deak and Juha Yrjola
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>

#include <linux/spi/tsc2301.h>

/**
 * The touchscreen interface operates as follows:
 *
 * Initialize:
 *    Request access to GPIO103 (DAV)
 *    tsc2301_ts_irq_handler will trigger when DAV line goes down
 *
 *  1) Pen is pressed against touchscreeen
 *  2) TSC2301 performs AD conversion
 *  3) After the conversion is done TSC2301 drives DAV line down
 *  4) GPIO IRQ is received and tsc2301_ts_irq_handler is called
 *  5) tsc2301_ts_irq_handler queues up an spi transfer to fetch
 *     the x, y, z1, z2 values
 *  6) SPI framework calls tsc2301_ts_rx after the coordinates are read
 *  7) When the penup_timer expires, there have not been DAV interrupts
 *     during the last 20ms which means the pen has been lifted.
 */


#define TSC2301_TOUCHSCREEN_PRODUCT_ID      		0x0052
#define TSC2301_TOUCHSCREEN_PRODUCT_VERSION 		0x0001

#define TSC2301_TS_PENUP_TIME		     		20

#define TSC2301_ADCREG_CONVERSION_CTRL_BY_TSC2301	0x8000
#define TSC2301_ADCREG_CONVERSION_CTRL_BY_HOST		0x0000

#define TSC2301_ADCREG_FUNCTION_NONE			0x0000
#define TSC2301_ADCREG_FUNCTION_XY			0x0400
#define TSC2301_ADCREG_FUNCTION_XYZ			0x0800
#define TSC2301_ADCREG_FUNCTION_X			0x0C00
#define TSC2301_ADCREG_FUNCTION_Y			0x1000
#define TSC2301_ADCREG_FUNCTION_Z			0x1400
#define TSC2301_ADCREG_FUNCTION_DAT1			0x1800
#define TSC2301_ADCREG_FUNCTION_DAT2			0x1C00
#define TSC2301_ADCREG_FUNCTION_AUX1			0x2000
#define TSC2301_ADCREG_FUNCTION_AUX2			0x2400
#define TSC2301_ADCREG_FUNCTION_TEMP			0x2800

#define TSC2301_ADCREG_RESOLUTION_8BIT			0x0100
#define TSC2301_ADCREG_RESOLUTION_10BIT			0x0200
#define TSC2301_ADCREG_RESOLUTION_12BIT			0x0300

#define TSC2301_ADCREG_AVERAGING_NONE			0x0000
#define TSC2301_ADCREG_AVERAGING_4AVG			0x0040
#define TSC2301_ADCREG_AVERAGING_8AVG			0x0080
#define TSC2301_ADCREG_AVERAGING_16AVG			0x00C0

#define TSC2301_ADCREG_CLOCK_8MHZ			0x0000
#define TSC2301_ADCREG_CLOCK_4MHZ			0x0010
#define TSC2301_ADCREG_CLOCK_2MHZ			0x0020
#define TSC2301_ADCREG_CLOCK_1MHZ			0x0030

#define TSC2301_ADCREG_VOLTAGE_STAB_0US			0x0000
#define TSC2301_ADCREG_VOLTAGE_STAB_100US		0x0002
#define TSC2301_ADCREG_VOLTAGE_STAB_500US		0x0004
#define TSC2301_ADCREG_VOLTAGE_STAB_1MS			0x0006
#define TSC2301_ADCREG_VOLTAGE_STAB_5MS			0x0008
#define TSC2301_ADCREG_VOLTAGE_STAB_10MS		0x000A
#define TSC2301_ADCREG_VOLTAGE_STAB_50MS		0x000C
#define TSC2301_ADCREG_VOLTAGE_STAB_100MS		0x000E

#define TSC2301_ADCREG_STOP_CONVERSION			0x4000

#define MAX_12BIT					((1 << 12) - 1)

#define TS_RECT_SIZE					8
#define TSF_MIN_Z1					100
#define TSF_MAX_Z2					4000

#define TSF_SAMPLES					4

struct ts_filter {
	int			sample_cnt;

	int 			avg_x;
	int 			avg_y;
	int 			avg_z1;
	int 			avg_z2;
};

struct ts_coords {
	u16 			x;
	u16 			y;
	u16 			z1;
	u16 			z2;
};

struct tsc2301_ts {
	struct input_dev	*idev;
	char			phys[32];
	struct timer_list	penup_timer;
	struct mutex		mutex;

	struct spi_transfer	read_xfer[2];
	struct spi_message	read_msg;
	struct ts_coords	*coords;

	struct ts_filter	filter;

	int			hw_avg_max;
	u16			x;
	u16			y;
	u16			p;

	u16			x_plate_ohm;
	int			stab_time;
	int			max_pressure;
	int			touch_pressure;

	u8			event_sent;
	u8			pen_down;
	u8			disabled;
	u8			disable_depth;

	int			hw_flags;
	int			irq;
};


static const u16 tsc2301_ts_read_data = 0x8000 | TSC2301_REG_X;

static int tsc2301_ts_check_config(struct tsc2301_ts *ts, int *hw_flags)
{
	int flags;

	flags = 0;
	switch (ts->hw_avg_max) {
	case 0:
		flags |= TSC2301_ADCREG_AVERAGING_NONE;
		break;
	case 4:
		flags |= TSC2301_ADCREG_AVERAGING_4AVG;
		break;
	case 8:
		flags |= TSC2301_ADCREG_AVERAGING_8AVG;
		break;
	case 16:
		flags |= TSC2301_ADCREG_AVERAGING_16AVG;
		break;
	default:
		return -EINVAL;
	}

	switch (ts->stab_time) {
	case 0:
		flags |= TSC2301_ADCREG_VOLTAGE_STAB_0US;
		break;
	case 100:
		flags |= TSC2301_ADCREG_VOLTAGE_STAB_100US;
		break;
	case 500:
		flags |= TSC2301_ADCREG_VOLTAGE_STAB_500US;
		break;
	case 1000:
		flags |= TSC2301_ADCREG_VOLTAGE_STAB_1MS;
		break;
	case 5000:
		flags |= TSC2301_ADCREG_VOLTAGE_STAB_5MS;
		break;
	case 10000:
		flags |= TSC2301_ADCREG_VOLTAGE_STAB_10MS;
		break;
	case 50000:
		flags |= TSC2301_ADCREG_VOLTAGE_STAB_50MS;
		break;
	case 100000:
		flags |= TSC2301_ADCREG_VOLTAGE_STAB_100MS;
		break;
	default:
		return -EINVAL;
	}

	*hw_flags = flags;
	return 0;
}

/*
 * This odd three-time initialization is to work around a bug in TSC2301.
 * See TSC2301 errata for details.
 */
static int tsc2301_ts_configure(struct tsc2301 *tsc, int flags)
{
	struct spi_transfer xfer[5];
	struct spi_transfer *x;
	struct spi_message m;
	int i;
	u16 val1, val2, val3;
	u16 data[10];

	val1 = TSC2301_ADCREG_CONVERSION_CTRL_BY_HOST |
		TSC2301_ADCREG_STOP_CONVERSION |
		TSC2301_ADCREG_FUNCTION_NONE |
		TSC2301_ADCREG_RESOLUTION_12BIT |
		TSC2301_ADCREG_AVERAGING_NONE |
		TSC2301_ADCREG_CLOCK_2MHZ |
		TSC2301_ADCREG_VOLTAGE_STAB_100MS;

	val2 = TSC2301_ADCREG_CONVERSION_CTRL_BY_HOST |
		TSC2301_ADCREG_FUNCTION_XYZ |
		TSC2301_ADCREG_RESOLUTION_12BIT |
		TSC2301_ADCREG_AVERAGING_16AVG |
		TSC2301_ADCREG_CLOCK_1MHZ |
		TSC2301_ADCREG_VOLTAGE_STAB_100MS;

	/* Averaging and voltage stabilization settings in flags */
	val3 = TSC2301_ADCREG_CONVERSION_CTRL_BY_TSC2301 |
		TSC2301_ADCREG_FUNCTION_XYZ |
		TSC2301_ADCREG_RESOLUTION_12BIT |
		TSC2301_ADCREG_CLOCK_2MHZ |
		flags;

	/* Now we prepare the command for transferring */
	data[0] = TSC2301_REG_ADC;
	data[1] = val1;
	data[2] = TSC2301_REG_ADC;
	data[3] = val2;
	data[4] = TSC2301_REG_ADC;
	data[5] = val3;
	data[6] = TSC2301_REG_REF;
	data[7] = 1 << 4 | 1 << 2 | 1; /* intref, 100uS settl, 2.5V ref */
	data[8] = TSC2301_REG_CONFIG;
	data[9] = 3 << 3 | 2 << 0; /* 340uS pre-chrg, 544us delay */

	spi_message_init(&m);
	m.spi = tsc->spi;

	memset(xfer, 0, sizeof(xfer));
	x = &xfer[0];

	for (i = 0; i < 10; i += 2) {
		x->tx_buf = &data[i];
		x->len = 4;
		if (i != 8)
			x->cs_change = 1;
		spi_message_add_tail(x, &m);
		x++;
	}
	spi_sync(m.spi, &m);

	return 0;
}

static void tsc2301_ts_start_scan(struct tsc2301 *tsc)
{
	tsc2301_ts_configure(tsc, tsc->ts->hw_flags);
	tsc2301_kp_restart(tsc);
}

static void tsc2301_ts_stop_scan(struct tsc2301 *tsc)
{
	tsc2301_write_reg(tsc, TSC2301_REG_ADC, TSC2301_ADCREG_STOP_CONVERSION);
	tsc2301_kp_restart(tsc);
}

static void update_pen_state(struct tsc2301_ts *ts, int x, int y, int pressure)
{
	if (pressure) {
		input_report_abs(ts->idev, ABS_X, x);
		input_report_abs(ts->idev, ABS_Y, y);
		input_report_abs(ts->idev, ABS_PRESSURE, pressure);
		if (!ts->pen_down)
			input_report_key(ts->idev, BTN_TOUCH, 1);
		ts->pen_down = 1;
	} else {
		input_report_abs(ts->idev, ABS_PRESSURE, 0);
		if (ts->pen_down)
			input_report_key(ts->idev, BTN_TOUCH, 0);
		ts->pen_down = 0;
	}

	input_sync(ts->idev);

#ifdef VERBOSE
	dev_dbg(&tsc->spi->dev, "x %4d y %4d p %4d\n", x, y, pressure);
#endif
}

static int filter(struct tsc2301_ts *ts, int x, int y, int z1, int z2)
{
	int inside_rect, pressure_limit, Rt;
	struct ts_filter *tsf = &ts->filter;

	/* validate pressure and position */
	if (x > MAX_12BIT || y > MAX_12BIT)
		return 0;

	/* skip coords if the pressure-components are out of range */
	if (z1 < TSF_MIN_Z1 || z2 > TSF_MAX_Z2)
		return 0;

	/* Use the x,y,z1,z2 directly on the first "pen down" event */
	if (ts->event_sent) {
		tsf->avg_x  += x;
		tsf->avg_y  += y;
		tsf->avg_z1 += z1;
		tsf->avg_z2 += z2;

		if (++tsf->sample_cnt < TSF_SAMPLES)
			return 0;
		x = tsf->avg_x / TSF_SAMPLES;
		y = tsf->avg_y / TSF_SAMPLES;
		z1 = tsf->avg_z1 / TSF_SAMPLES;
		z2 = tsf->avg_z2 / TSF_SAMPLES;
	}
	tsf->sample_cnt = 0;
	tsf->avg_x  = 0;
	tsf->avg_y  = 0;
	tsf->avg_z1 = 0;
	tsf->avg_z2 = 0;

	pressure_limit = ts->event_sent? ts->max_pressure: ts->touch_pressure;

	/* z1 is always at least 100: */
	Rt = x * (z2 - z1) / z1;
	Rt = Rt * ts->x_plate_ohm / 4096;
	if (Rt > pressure_limit)
		return 0;

	/* discard the event if it still is within the previous rect - unless
	 * if the pressure is harder, but then use previous x,y position */
	inside_rect = (
	    x > (int)ts->x - TS_RECT_SIZE && x < (int)ts->x + TS_RECT_SIZE &&
	    y > (int)ts->y - TS_RECT_SIZE && y < (int)ts->y + TS_RECT_SIZE);

	if (!ts->event_sent || !inside_rect) {
		ts->x = x;
		ts->y = y;
		ts->p = Rt;
		return 1;
	} else if (Rt < ts->p) {
		ts->p = Rt;
		return 1;
	}
	return 0;
}

/*
 * This procedure is called by the SPI framework after the coordinates
 * have been read from TSC2301
 */
static void tsc2301_ts_rx(void *arg)
{
	struct tsc2301 *tsc = arg;
	struct tsc2301_ts *ts = tsc->ts;
	int send_event;
	int x, y, z1, z2;

	x  = ts->coords->x;
	y  = ts->coords->y;
	z1 = ts->coords->z1;
	z2 = ts->coords->z2;

	send_event = filter(ts, x, y, z1, z2);
	if (send_event) {
		update_pen_state(ts, ts->x, ts->y, ts->p);
		ts->event_sent = 1;
	}

	mod_timer(&ts->penup_timer,
		  jiffies + msecs_to_jiffies(TSC2301_TS_PENUP_TIME));
}

/*
 * Timer is called TSC2301_TS_PENUP_TIME after pen is up
 */
static void tsc2301_ts_timer_handler(unsigned long data)
{
	struct tsc2301 *tsc = (struct tsc2301 *)data;
	struct tsc2301_ts *ts = tsc->ts;

	if (ts->event_sent) {
		ts->event_sent = 0;
		update_pen_state(ts, 0, 0, 0);
	}
}

/*
 * This interrupt is called when pen is down and coordinates are
 * available. That is indicated by a falling edge on DEV line.
 */
static irqreturn_t tsc2301_ts_irq_handler(int irq, void *dev_id)
{
	struct tsc2301 *tsc = dev_id;
	struct tsc2301_ts *ts = tsc->ts;
	int r;

	r = spi_async(tsc->spi, &ts->read_msg);
	if (r)
		dev_err(&tsc->spi->dev, "ts: spi_async() failed");

	mod_timer(&ts->penup_timer,
		  jiffies + msecs_to_jiffies(TSC2301_TS_PENUP_TIME));

	return IRQ_HANDLED;
}

static void tsc2301_ts_disable(struct tsc2301 *tsc)
{
	struct tsc2301_ts *ts = tsc->ts;

	if (ts->disable_depth++ != 0)
		return;

	disable_irq(ts->irq);

	/* wait until penup timer expire normally */
	do {
		msleep(1);
	} while (ts->event_sent);

	tsc2301_ts_stop_scan(tsc);
}

static void tsc2301_ts_enable(struct tsc2301 *tsc)
{
	struct tsc2301_ts *ts = tsc->ts;

	if (--ts->disable_depth != 0)
		return;

	enable_irq(ts->irq);

	tsc2301_ts_start_scan(tsc);
}

#ifdef CONFIG_PM
int tsc2301_ts_suspend(struct tsc2301 *tsc)
{
	struct tsc2301_ts *ts = tsc->ts;

	mutex_lock(&ts->mutex);
	tsc2301_ts_disable(tsc);
	mutex_unlock(&ts->mutex);

	return 0;
}

void tsc2301_ts_resume(struct tsc2301 *tsc)
{
	struct tsc2301_ts *ts = tsc->ts;

	mutex_lock(&ts->mutex);
	tsc2301_ts_enable(tsc);
	mutex_unlock(&ts->mutex);
}
#endif

static void tsc2301_ts_setup_spi_xfer(struct tsc2301 *tsc)
{
	struct tsc2301_ts *ts = tsc->ts;
	struct spi_message *m = &ts->read_msg;
	struct spi_transfer *x = &ts->read_xfer[0];

	spi_message_init(m);

	x->tx_buf = &tsc2301_ts_read_data;
	x->len = 2;
	spi_message_add_tail(x, m);

	x++;
	x->rx_buf = ts->coords;
	x->len = 8;
	spi_message_add_tail(x, m);

	m->complete = tsc2301_ts_rx;
	m->context = tsc;
}

static ssize_t tsc2301_ts_pen_down_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct tsc2301 *tsc = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", tsc->ts->pen_down);
}

static DEVICE_ATTR(pen_down, S_IRUGO, tsc2301_ts_pen_down_show, NULL);

static ssize_t tsc2301_ts_disable_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct tsc2301		*tsc = dev_get_drvdata(dev);
	struct tsc2301_ts	*ts = tsc->ts;

	return sprintf(buf, "%u\n", ts->disabled);
}

static ssize_t tsc2301_ts_disable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct tsc2301		*tsc = dev_get_drvdata(dev);
	struct tsc2301_ts	*ts = tsc->ts;
	char *endp;
	int i;

	i = simple_strtoul(buf, &endp, 10);
	i = i ? 1 : 0;
	mutex_lock(&ts->mutex);
	if (i == ts->disabled) goto out;
	ts->disabled = i;

	if (i)
		tsc2301_ts_disable(tsc);
	else
		tsc2301_ts_enable(tsc);
out:
	mutex_unlock(&ts->mutex);
	return count;
}

static DEVICE_ATTR(disable_ts, 0664, tsc2301_ts_disable_show,
		   tsc2301_ts_disable_store);

int __devinit tsc2301_ts_init(struct tsc2301 *tsc,
			      struct tsc2301_platform_data *pdata)
{
	struct tsc2301_ts *ts;
	struct input_dev *idev;
	int r;
	int x_max, y_max;
	int x_fudge, y_fudge, p_fudge;

	if (pdata->dav_int <= 0) {
		dev_err(&tsc->spi->dev, "need DAV IRQ");
		return -EINVAL;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL)
		return -ENOMEM;
	tsc->ts = ts;

	ts->coords = kzalloc(sizeof(*ts->coords), GFP_KERNEL);
	if (ts->coords == NULL) {
		kfree(ts);
		return -ENOMEM;
	}

	ts->irq = pdata->dav_int;

	init_timer(&ts->penup_timer);
	setup_timer(&ts->penup_timer, tsc2301_ts_timer_handler,
			(unsigned long)tsc);

	mutex_init(&ts->mutex);

	ts->x_plate_ohm	= pdata->ts_x_plate_ohm ? : 280;
	ts->hw_avg_max	= pdata->ts_hw_avg;
	ts->max_pressure = pdata->ts_max_pressure ? : MAX_12BIT;
	ts->touch_pressure = pdata->ts_touch_pressure ? : ts->max_pressure;
	ts->stab_time	= pdata->ts_stab_time;

	x_max		= pdata->ts_x_max ? : 4096;
	y_max		= pdata->ts_y_max ? : 4096;
	x_fudge		= pdata->ts_x_fudge ? : 4;
	y_fudge		= pdata->ts_y_fudge ? : 8;
	p_fudge		= pdata->ts_pressure_fudge ? : 2;

	if ((r = tsc2301_ts_check_config(ts, &ts->hw_flags))) {
		dev_err(&tsc->spi->dev, "invalid configuration\n");
		goto err2;
	}

	idev = input_allocate_device();
	if (idev == NULL) {
		r = -ENOMEM;
		goto err2;
	}
	idev->name = "TSC2301 touchscreen";
	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input-ts", tsc->spi->dev.bus_id);
	idev->phys = ts->phys;
	idev->dev.parent = &tsc->spi->dev;

	idev->evbit[0] = BIT(EV_ABS) | BIT(EV_KEY);
	idev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
	ts->idev = idev;

	tsc2301_ts_setup_spi_xfer(tsc);

	/* These parameters should perhaps be configurable? */
	input_set_abs_params(idev, ABS_X, 0, x_max, x_fudge, 0);
	input_set_abs_params(idev, ABS_Y, 0, y_max, y_fudge, 0);
	input_set_abs_params(idev, ABS_PRESSURE, 0, ts->max_pressure,
			     p_fudge, 0);

	tsc2301_ts_start_scan(tsc);

	r = request_irq(ts->irq, tsc2301_ts_irq_handler,
			IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_FALLING,
			"tsc2301-ts", tsc);
	if (r < 0) {
		dev_err(&tsc->spi->dev, "unable to get DAV IRQ");
		goto err3;
	}
	set_irq_wake(ts->irq, 1);

	if (device_create_file(&tsc->spi->dev, &dev_attr_pen_down) < 0)
		goto err4;
	if (device_create_file(&tsc->spi->dev, &dev_attr_disable_ts) < 0)
		goto err5;

	r = input_register_device(idev);
	if (r < 0) {
		dev_err(&tsc->spi->dev, "can't register touchscreen device\n");
		goto err6;
	}

	return 0;
err6:
	device_remove_file(&tsc->spi->dev, &dev_attr_disable_ts);
err5:
	device_remove_file(&tsc->spi->dev, &dev_attr_pen_down);
err4:
	free_irq(ts->irq, tsc);
err3:
	tsc2301_ts_stop_scan(tsc);
	input_free_device(idev);
err2:
	kfree(ts->coords);
	kfree(ts);
	return r;
}

void __devexit tsc2301_ts_exit(struct tsc2301 *tsc)
{
	struct tsc2301_ts *ts = tsc->ts;

	tsc2301_ts_disable(tsc);

	device_remove_file(&tsc->spi->dev, &dev_attr_disable_ts);
	device_remove_file(&tsc->spi->dev, &dev_attr_pen_down);

	free_irq(ts->irq, tsc);
	input_unregister_device(ts->idev);

	kfree(ts->coords);
	kfree(ts);
}
MODULE_AUTHOR("Jarkko Oikarinen <jarkko.oikarinen@nokia.com>");
MODULE_LICENSE("GPL");
