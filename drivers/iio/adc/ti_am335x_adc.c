/*
 * TI ADC MFD driver
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/mfd/ti_am335x_tscadc.h>
#include <linux/platform_data/ti_am335x_adc.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/wait.h>
#include <linux/sched.h>

struct tiadc_device {
	struct ti_tscadc_dev *mfd_tscadc;
	int channels;
	struct iio_map *map;
	u8 channel_line[8];
	u8 channel_step[8];
	struct work_struct poll_work;
	wait_queue_head_t wq_data_avail;
	bool data_avail;
	u32 *inputbuffer;
	int sample_count;
	int irq;
};

static unsigned int tiadc_readl(struct tiadc_device *adc, unsigned int reg)
{
	return readl(adc->mfd_tscadc->tscadc_base + reg);
}

static void tiadc_writel(struct tiadc_device *adc, unsigned int reg,
					unsigned int val)
{
	writel(val, adc->mfd_tscadc->tscadc_base + reg);
}

static u32 get_adc_step_mask(struct tiadc_device *adc_dev)
{
	u32 step_en;

	step_en = ((1 << adc_dev->channels) - 1);
	step_en <<= TOTAL_STEPS - adc_dev->channels + 1;
	return step_en;
}

static void tiadc_step_config(struct iio_dev *indio_dev)
{
	struct tiadc_device *adc_dev = iio_priv(indio_dev);
	unsigned int stepconfig;
	int i, steps, chan;

	/*
	 * There are 16 configurable steps and 8 analog input
	 * lines available which are shared between Touchscreen and ADC.
	 * Steps backwards i.e. from 16 towards 0 are used by ADC
	 * depending on number of input lines needed.
	 * Channel would represent which analog input
	 * needs to be given to ADC to digitalize data.
	 */
	steps = TOTAL_STEPS - adc_dev->channels;
	if (iio_buffer_enabled(indio_dev))
		stepconfig = STEPCONFIG_AVG_16 | STEPCONFIG_FIFO1
					| STEPCONFIG_MODE_SWCNT;
	else
		stepconfig = STEPCONFIG_AVG_16 | STEPCONFIG_FIFO1;

	for (i = 0; i < adc_dev->channels; i++) {
		chan = adc_dev->channel_line[i];
		tiadc_writel(adc_dev, REG_STEPCONFIG(steps),
				stepconfig | STEPCONFIG_INP(chan));
		tiadc_writel(adc_dev, REG_STEPDELAY(steps),
				STEPCONFIG_OPENDLY);
		adc_dev->channel_step[i] = steps;
		steps++;
	}
}

static irqreturn_t tiadc_irq(int irq, void *private)
{
	struct iio_dev *idev = private;
	struct tiadc_device *adc_dev = iio_priv(idev);
	unsigned int status, config;
	status = tiadc_readl(adc_dev, REG_IRQSTATUS);

	/* FIFO Overrun. Clear flag. Disable/Enable ADC to recover */
	if (status & IRQENB_FIFO1OVRRUN) {
		config = tiadc_readl(adc_dev, REG_CTRL);
		config &= ~(CNTRLREG_TSCSSENB);
		tiadc_writel(adc_dev, REG_CTRL, config);
		tiadc_writel(adc_dev, REG_IRQSTATUS, IRQENB_FIFO1OVRRUN |
				IRQENB_FIFO1UNDRFLW | IRQENB_FIFO1THRES);
		tiadc_writel(adc_dev, REG_CTRL, (config | CNTRLREG_TSCSSENB));
		return IRQ_HANDLED;
	} else if (status & IRQENB_FIFO1THRES) {
		/* Wake adc_work that pushes FIFO data to iio buffer */
		tiadc_writel(adc_dev, REG_IRQCLR, IRQENB_FIFO1THRES);
		adc_dev->data_avail = 1;
		wake_up_interruptible(&adc_dev->wq_data_avail);
		return IRQ_HANDLED;
	} else
		return IRQ_NONE;
}

static irqreturn_t tiadc_trigger_h(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct tiadc_device *adc_dev = iio_priv(indio_dev);
	unsigned int config;

	schedule_work(&adc_dev->poll_work);
	config = tiadc_readl(adc_dev, REG_CTRL);
	tiadc_writel(adc_dev, REG_CTRL,	config & ~CNTRLREG_TSCSSENB);
	tiadc_writel(adc_dev, REG_CTRL,	config |  CNTRLREG_TSCSSENB);

	tiadc_writel(adc_dev,  REG_IRQSTATUS, IRQENB_FIFO1THRES |
			 IRQENB_FIFO1OVRRUN | IRQENB_FIFO1UNDRFLW);
	tiadc_writel(adc_dev,  REG_IRQENABLE, IRQENB_FIFO1THRES
				| IRQENB_FIFO1OVRRUN);

	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int tiadc_buffer_preenable(struct iio_dev *indio_dev)
{
	return iio_sw_buffer_preenable(indio_dev);
}

static int tiadc_buffer_postenable(struct iio_dev *indio_dev)
{
	struct tiadc_device *adc_dev = iio_priv(indio_dev);
	struct iio_buffer *buffer = indio_dev->buffer;
	unsigned int enb, stepnum;
	u8 bit;

	tiadc_step_config(indio_dev);
	tiadc_writel(adc_dev, REG_SE, 0x00);
	for_each_set_bit(bit, buffer->scan_mask,
			adc_dev->channels) {
		struct iio_chan_spec const *chan = indio_dev->channels + bit;
		/*
		 * There are a total of 16 steps available
		 * that are shared between ADC and touchscreen.
		 * We start configuring from step 16 to 0 incase of
		 * ADC. Hence the relation between input channel
		 * and step for ADC would be as below.
		 */
		stepnum = chan->channel + 9;
		enb = tiadc_readl(adc_dev, REG_SE);
		enb |= (1 << stepnum);
		tiadc_writel(adc_dev, REG_SE, enb);
	}

	return iio_triggered_buffer_postenable(indio_dev);
}

static int tiadc_buffer_predisable(struct iio_dev *indio_dev)
{
	struct tiadc_device *adc_dev = iio_priv(indio_dev);
	int fifo1count, i, read, config;

	config = tiadc_readl(adc_dev, REG_CTRL);
	config &= ~(CNTRLREG_TSCSSENB);
	tiadc_writel(adc_dev, REG_CTRL, config);
	tiadc_writel(adc_dev, REG_IRQCLR, (IRQENB_FIFO1THRES |
				IRQENB_FIFO1OVRRUN | IRQENB_FIFO1UNDRFLW));
	tiadc_writel(adc_dev, REG_SE, STPENB_STEPENB_TC);

	/* Flush FIFO of any leftover data */
	fifo1count = tiadc_readl(adc_dev, REG_FIFO1CNT);
	for (i = 0; i < fifo1count; i++)
		read = tiadc_readl(adc_dev, REG_FIFO1);

	return iio_triggered_buffer_predisable(indio_dev);
}

static int tiadc_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct tiadc_device *adc_dev = iio_priv(indio_dev);
	int config;

	tiadc_step_config(indio_dev);
	config = tiadc_readl(adc_dev, REG_CTRL);
	tiadc_writel(adc_dev, REG_CTRL, (config | CNTRLREG_TSCSSENB));

	return 0;
}

static const struct iio_buffer_setup_ops tiadc_buffer_setup_ops = {
	.preenable = &tiadc_buffer_preenable,
	.postenable = &tiadc_buffer_postenable,
	.predisable = &tiadc_buffer_predisable,
	.postdisable = &tiadc_buffer_postdisable,
};

static void tiadc_adc_work(struct work_struct *work_s)
{
	struct tiadc_device *adc_dev =
		container_of(work_s, struct tiadc_device, poll_work);
	struct iio_dev *indio_dev = iio_priv_to_dev(adc_dev);
	struct iio_buffer *buffer = indio_dev->buffer;
	int i, j, k, fifo1count, read;
	unsigned int config;
	int size_to_acquire = buffer->access->get_length(buffer);
	int sample_count = 0;
	u32 *data;

	adc_dev->data_avail = 0;
	data = kmalloc(indio_dev->scan_bytes, GFP_KERNEL);
	if (data == NULL)
		goto out;

	while (sample_count < size_to_acquire) {
		tiadc_writel(adc_dev, REG_IRQSTATUS, IRQENB_FIFO1THRES);
		tiadc_writel(adc_dev, REG_IRQENABLE, IRQENB_FIFO1THRES);

		wait_event_interruptible(adc_dev->wq_data_avail,
					(adc_dev->data_avail == 1));
		adc_dev->data_avail = 0;

		fifo1count = tiadc_readl(adc_dev, REG_FIFO1CNT);
		if (fifo1count * sizeof(u32) <
				buffer->access->get_bytes_per_datum(buffer))
			continue;

		sample_count = sample_count + fifo1count;
		for (k = 0; k < fifo1count; k = k + i) {
			for (i = 0, j = 0; i < (indio_dev->scan_bytes)/4; i++) {
				read = tiadc_readl(adc_dev, REG_FIFO1);
				data[i] = read & FIFOREAD_DATA_MASK;
			}
			iio_push_to_buffers(indio_dev, (u8 *) data);
		}
	}
out:
	tiadc_writel(adc_dev, REG_IRQCLR, (IRQENB_FIFO1THRES |
				IRQENB_FIFO1OVRRUN | IRQENB_FIFO1UNDRFLW));
	config = tiadc_readl(adc_dev, REG_CTRL);
	tiadc_writel(adc_dev, REG_CTRL,	config & ~CNTRLREG_TSCSSENB);
}

irqreturn_t tiadc_iio_pollfunc(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct tiadc_device *adc_dev = iio_priv(indio_dev);
	int i, fifo1count, read;

	tiadc_writel(adc_dev, REG_IRQCLR, (IRQENB_FIFO1THRES |
				IRQENB_FIFO1OVRRUN |
				IRQENB_FIFO1UNDRFLW));

	/* Flush FIFO before trigger */
	fifo1count = tiadc_readl(adc_dev, REG_FIFO1CNT);
	for (i = 0; i < fifo1count; i++)
		read = tiadc_readl(adc_dev, REG_FIFO1);

	return IRQ_WAKE_THREAD;
}

static const char * const chan_name_ain[] = {
	"AIN0",
	"AIN1",
	"AIN2",
	"AIN3",
	"AIN4",
	"AIN5",
	"AIN6",
	"AIN7",
};

static int tiadc_channel_init(struct iio_dev *indio_dev, int channels)
{
	struct tiadc_device *adc_dev = iio_priv(indio_dev);
	struct iio_chan_spec *chan_array;
	struct iio_chan_spec *chan;
	struct iio_map *map;
	int i, ret;

	indio_dev->num_channels = channels;
	chan_array = kcalloc(channels,
			sizeof(struct iio_chan_spec), GFP_KERNEL);
	if (chan_array == NULL) {
		ret = -ENOMEM;
		goto err_no_chan_array;
	}

	chan = chan_array;
	for (i = 0; i < channels; i++, chan++) {

		chan->type = IIO_VOLTAGE;
		chan->indexed = 1;
		chan->channel = adc_dev->channel_line[i];
		chan->info_mask = IIO_CHAN_INFO_RAW_SEPARATE_BIT;
		chan->datasheet_name = chan_name_ain[chan->channel];
		chan->scan_index = i;
		chan->scan_type.sign = 'u';
		chan->scan_type.realbits = 12;
		chan->scan_type.storagebits = 32;
	}

	indio_dev->channels = chan_array;

	map = kcalloc(channels + 1, sizeof(struct iio_map), GFP_KERNEL);
	if (map == NULL) {
		ret = -ENOMEM;
		goto err_no_iio_map;
	}
	adc_dev->map = map;

	for (i = 0, chan = chan_array; i < channels; i++, chan++, map++) {
		map->adc_channel_label = chan->datasheet_name;
		map->consumer_dev_name = "any";
		map->consumer_channel = chan->datasheet_name;
	}
	map->adc_channel_label = NULL;
	map->consumer_dev_name = NULL;
	map->consumer_channel = NULL;

	ret = iio_map_array_register(indio_dev, adc_dev->map);
	if (ret != 0)
		goto err_iio_map_register_fail;

	return 0;

err_iio_map_register_fail:
	kfree(adc_dev->map);
	adc_dev->map = NULL;
err_no_iio_map:
	kfree(chan_array);
	indio_dev->channels = NULL;
err_no_chan_array:
	return ret;
}

static void tiadc_channels_remove(struct iio_dev *indio_dev)
{
	struct tiadc_device *adc_dev = iio_priv(indio_dev);
	kfree(indio_dev->channels);
	kfree(adc_dev->map);
}

static int tiadc_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int *val, int *val2, long mask)
{
	struct tiadc_device *adc_dev = iio_priv(indio_dev);
	int i, map_val;
	unsigned int fifo1count, read, stepid, step_en;

	if (iio_buffer_enabled(indio_dev))
		return -EBUSY;
	else {
		unsigned long timeout = jiffies + usecs_to_jiffies
					(IDLE_TIMEOUT * adc_dev->channels);
		step_en = get_adc_step_mask(adc_dev);
		am335x_tsc_se_set(adc_dev->mfd_tscadc, step_en);

		/* Wait for ADC sequencer to complete sampling */
		while (tiadc_readl(adc_dev, REG_ADCFSM) & SEQ_STATUS) {
			if (time_after(jiffies, timeout))
				return -EAGAIN;
			}
		map_val = chan->channel + TOTAL_CHANNELS;

		/*
		 * When the sub-system is first enabled,
		 * the sequencer will always start with the
		 * lowest step (1) and continue until step (16).
		 * For ex: If we have enabled 4 ADC channels and
		 * currently use only 1 out of them, the
		 * sequencer still configures all the 4 steps,
		 * leading to 3 unwanted data.
		 * Hence we need to flush out this data.
		 */

		fifo1count = tiadc_readl(adc_dev, REG_FIFO1CNT);
		for (i = 0; i < fifo1count; i++) {
			read = tiadc_readl(adc_dev, REG_FIFO1);
			stepid = read & FIFOREAD_CHNLID_MASK;
			stepid = stepid >> 0x10;

			if (stepid == map_val) {
				read = read & FIFOREAD_DATA_MASK;
				*val = read;
				return IIO_VAL_INT;
			}
		}
		return -EAGAIN;
	}
}

static const struct iio_info tiadc_info = {
	.read_raw = &tiadc_read_raw,
	.driver_module = THIS_MODULE,
};

static int tiadc_probe(struct platform_device *pdev)
{
	struct iio_dev		*indio_dev;
	struct tiadc_device	*adc_dev;
	struct ti_tscadc_dev	*tscadc_dev = ti_tscadc_dev_get(pdev);
	struct mfd_tscadc_board	*pdata = tscadc_dev->dev->platform_data;
	struct device_node	*node = tscadc_dev->dev->of_node;
	struct property		*prop;
	const __be32		*cur;
	int			err;
	u32			val;
	int			channels = 0;

	if (!pdata && !node) {
		dev_err(&pdev->dev, "Could not find platform data\n");
		return -EINVAL;
	}

	indio_dev = iio_device_alloc(sizeof(struct tiadc_device));
	if (indio_dev == NULL) {
		dev_err(&pdev->dev, "failed to allocate iio device\n");
		err = -ENOMEM;
		goto err_ret;
	}
	adc_dev = iio_priv(indio_dev);

	adc_dev->mfd_tscadc = ti_tscadc_dev_get(pdev);

	if (pdata)
		adc_dev->channels = pdata->adc_init->adc_channels;
	else {
		node = of_get_child_by_name(node, "adc");
		of_property_for_each_u32(node, "ti,adc-channels", prop, cur, val) {
				adc_dev->channel_line[channels] = val;
			channels++;
		}
		adc_dev->channels = channels;
	}
	adc_dev->channels = channels;
	adc_dev->irq = adc_dev->mfd_tscadc->irq;

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &tiadc_info;

	tiadc_step_config(indio_dev);
	tiadc_writel(adc_dev, REG_FIFO1THR, FIFO1_THRESHOLD);

	err = tiadc_channel_init(indio_dev, adc_dev->channels);
	if (err < 0) {
		dev_err(&pdev->dev, "tiadc_channel_init() failed\n");
		goto err_free_device;
	}

	INIT_WORK(&adc_dev->poll_work, &tiadc_adc_work);
	init_waitqueue_head(&adc_dev->wq_data_avail);

	err = request_irq(adc_dev->irq, tiadc_irq, IRQF_SHARED,
		indio_dev->name, indio_dev);
	if (err)
		goto err_free_irq;

	err = iio_triggered_buffer_setup(indio_dev, &tiadc_iio_pollfunc,
			&tiadc_trigger_h, &tiadc_buffer_setup_ops);
	if (err)
		goto err_unregister;

	err = iio_device_register(indio_dev);
	if (err) {
		dev_err(&pdev->dev, "iio_device_register() failed\n");
		goto err_free_channels;
	}

	platform_set_drvdata(pdev, indio_dev);

	return 0;

err_unregister:
	iio_buffer_unregister(indio_dev);
err_free_irq:
	free_irq(adc_dev->irq, indio_dev);
err_free_channels:
	tiadc_channels_remove(indio_dev);
err_free_device:
	iio_device_free(indio_dev);
err_ret:
	return err;
}

static int tiadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct tiadc_device *adc_dev = iio_priv(indio_dev);
	u32 step_en;

	free_irq(adc_dev->irq, indio_dev);
	iio_device_unregister(indio_dev);
	iio_buffer_unregister(indio_dev);
	tiadc_channels_remove(indio_dev);

	step_en = get_adc_step_mask(adc_dev);
	am335x_tsc_se_clr(adc_dev->mfd_tscadc, step_en);

	iio_device_free(indio_dev);

	return 0;
}

#ifdef CONFIG_PM
static int tiadc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tiadc_device *adc_dev = iio_priv(indio_dev);
	struct ti_tscadc_dev *tscadc_dev;
	unsigned int idle;

	tscadc_dev = ti_tscadc_dev_get(to_platform_device(dev));
	if (!device_may_wakeup(tscadc_dev->dev)) {
		idle = tiadc_readl(adc_dev, REG_CTRL);
		idle &= ~(CNTRLREG_TSCSSENB);
		tiadc_writel(adc_dev, REG_CTRL, (idle |
				CNTRLREG_POWERDOWN));
	}

	return 0;
}

static int tiadc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tiadc_device *adc_dev = iio_priv(indio_dev);
	unsigned int restore;

	/* Make sure ADC is powered up */
	restore = tiadc_readl(adc_dev, REG_CTRL);
	restore &= ~(CNTRLREG_TSCSSENB);
	tiadc_writel(adc_dev, REG_CTRL, restore);

	tiadc_writel(adc_dev, REG_FIFO1THR, FIFO1_THRESHOLD);
	tiadc_step_config(indio_dev);

	/* Make sure ADC is powered up */
	restore &= ~(CNTRLREG_POWERDOWN);
	restore |= CNTRLREG_TSCSSENB;
	tiadc_writel(adc_dev, REG_CTRL, restore);

	return 0;
}

static const struct dev_pm_ops tiadc_pm_ops = {
	.suspend = tiadc_suspend,
	.resume = tiadc_resume,
};
#define TIADC_PM_OPS (&tiadc_pm_ops)
#else
#define TIADC_PM_OPS NULL
#endif

static const struct of_device_id ti_adc_dt_ids[] = {
	{ .compatible = "ti,ti-tscadc", },
	{ }
};
MODULE_DEVICE_TABLE(of, ti_adc_dt_ids);

static struct platform_driver tiadc_driver = {
	.driver = {
		.name   = "tiadc",
		.owner	= THIS_MODULE,
		.pm	= TIADC_PM_OPS,
		.of_match_table = of_match_ptr(ti_adc_dt_ids),
	},
	.probe	= tiadc_probe,
	.remove	= tiadc_remove,
};

module_platform_driver(tiadc_driver);

MODULE_DESCRIPTION("TI ADC controller driver");
MODULE_AUTHOR("Rachna Patil <rachna@ti.com>");
MODULE_LICENSE("GPL");
