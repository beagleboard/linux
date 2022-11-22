/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <linux/of_platform.h>
#include <linux/err.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>

/* This will be the driver name the kernel reports */
#define DRIVER_NAME "light-adc"

/* ADC registers */
#define LIGHT_ADC_PHY_CFG			0x00
#define LIGHT_ADC_PHY_CTRL			0x04
#define LIGHT_ADC_PHY_TEST			0x08
#define LIGHT_ADC_OP_CTRL			0x0C
#define LIGHT_ADC_OP_SINGLE_START		0x10
#define LIGHT_ADC_FCLK_CTRL			0x14
#define LIGHT_ADC_START_TIME			0x18
#define LIGHT_ADC_SAMPLE_TIME			0x1C
#define LIGHT_ADC_SAMPLE_DATA			0x20
#define LIGHT_ADC_INT_CTRL1			0x50
#define LIGHT_ADC_INT_CTRL2			0x54
#define LIGHT_ADC_INT_STATUS			0x58
#define LIGHT_ADC_INT_ACTUAL_VALUE_CH0		0x60
#define LIGHT_ADC_INT_ACTUAL_VALUE_CH1		0x64
#define LIGHT_ADC_INT_DELTA_VALUE_CH0		0x90
#define LIGHT_ADC_INT_DELTA_VALUE_CH1		0x94

/* Configuration register field define */
#define LIGHT_ADC_PHY_CFG_SELRES_6BIT                    	(0x0)
#define LIGHT_ADC_PHY_CFG_SELRES_8BIT                		(0x1)
#define LIGHT_ADC_PHY_CFG_SELRES_10BIT               		(0x2)
#define LIGHT_ADC_PHY_CFG_SELRES_12BIT               		(0x3)
#define LIGHT_ADC_PHY_CFG_SELDIFF_SINGLE_ENDED_INPUTS       	(0x0 << 4)
#define LIGHT_ADC_PHY_CFG_SELDIFF_DIFFERENTIAL_INPUTS       	(0x1 << 4)
#define LIGHT_ADC_PHY_CFG_SELBG_INTERNAL               		(0x1 << 8)
#define LIGHT_ADC_PHY_CFG_SELBG_EXTERNAL               		(0x0 << 8)
#define LIGHT_ADC_PHY_CFG_SELREF_INTERNAL              		(0x1 << 12)
#define LIGHT_ADC_PHY_CFG_SELREF_EXT              		(0x0 << 12)

/* PHY CTRL register field define */
#define LIGHT_ADC_PHY_CTRL_ENOFFSET_EN                      	(0x1 << 12)
#define LIGHT_ADC_PHY_CTRL_ENOFFMEAS_EN                   	(0x1 << 8)
#define LIGHT_ADC_PHY_CTRL_RST_EN				(0x1 << 4)
#define LIGHT_ADC_PHY_CTRL_ENADC_EN				(0x1 << 0)

/* ADC OP ctrl field define  */
#define LIGHT_ADC_OP_CTRL_CH_EN_ALL                        	GENMASK(19, 12)
#define LIGHT_ADC_OP_CTRL_CH_EN_0                        	(12)
#define LIGHT_ADC_OP_CTRL_MODE_SINGLE                    	(0x1 << 0)
#define LIGHT_ADC_OP_CTRL_MODE_CONTINOUS                 	(0x0 << 0)

/* ADC OP single start */
#define LIGHT_ADC_OP_SINGLE_START_EN				BIT(0)

/* ADC fclk ctrl */
#define LIGHT_ADC_FCLK_CTRL_FCLLK_DIV				GENMASK(6, 0)
#define LIGHT_ADC_FCLK_CTRL_TYP_1M                        	(0x10004)
#define LIGHT_ADC_START_TIME_TYP_1M                        	(0x160)
#define LIGHT_ADC_SAMPLE_TIME_TYP_1M                       	(0x10)
#define LIGHT_ADC_SAMPLE_TIME_TYP_6BIT				(8)
#define LIGHT_ADC_SAMPLE_TIME_TYP_8BIT				(10)
#define LIGHT_ADC_SAMPLE_TIME_TYP_10BIT				(12)
#define LIGHT_ADC_SAMPLE_TIME_TYP_12BIT				(14)

/* ADC sample data */
#define LIGHT_ADC_SAMPLE_DATA_CH1                      		GENMASK(27, 16)
#define LIGHT_ADC_SAMPLE_DATA_CH1_OFF                      	(16)
#define LIGHT_ADC_SAMPLE_DATA_CH1_VLD				BIT(31)
#define LIGHT_ADC_SAMPLE_DATA_CH1_NUMBER			GENMASK(30, 28)
#define LIGHT_ADC_SAMPLE_DATA_CH1_NUMBER_OFF			(28)
#define LIGHT_ADC_SAMPLE_DATA_CH0                      		GENMASK(11, 0)
#define LIGHT_ADC_SAMPLE_DATA_CH0_VLD				BIT(15)
#define LIGHT_ADC_SAMPLE_DATA_CH0_OFF                      	(0)
#define LIGHT_ADC_SAMPLE_DATA_CH0_NUMBER			GENMASK(14, 12)
#define LIGHT_ADC_SAMPLE_DATA_CH0_NUMBER_OFF			(12)

/* ADC INT Ctrl */
#define LIGHT_ADC_INT_CTRL1_CH1_INT_MODE			BIT(1)
#define LIGHT_ADC_INT_CTRL1_CH0_INT_MODE			BIT(0)
#define LIGHT_ADC_INT_CTRL2_CH1_INT_MASK			BIT(1)
#define LIGHT_ADC_INT_CTRL2_CH0_INT_MASK			BIT(0)
#define LIGHT_ADC_INT_STS_CH1_INT_STS				BIT(1)
#define LIGHT_ADC_INT_STS_CH0_INT_STS				BIT(0)

#define LIGHT_ADC_ACTUAL_VALUE_CH0_HVAL				GENMASK(27, 16)
#define LIGHT_ADC_ACTUAL_VALUE_CH0_HVAL_OFF			(16)
#define LIGHT_ADC_ACTUAL_VALUE_CH0_LVAL				GENMASK(11, 0)
#define LIGHT_ADC_ACTUAL_VALUE_CH0_LVAL_OFF			(0)
#define LIGHT_ADC_ACTUAL_VALUE_CH1_HVAL				GENMASK(27, 16)
#define LIGHT_ADC_ACTUAL_VALUE_CH1_HVAL_OFF			(16)
#define LIGHT_ADC_ACTUAL_VALUE_CH1_LVAL				GENMASK(11, 0)
#define LIGHT_ADC_ACTUAL_VALUE_CH1_LVAL_OFF			(0)

#define LIGHT_ADC_DLT_VALUE_CH0_HVAL				GENMASK(27, 16)
#define LIGHT_ADC_DLT_VALUE_CH0_HVAL_OFF			(16)
#define LIGHT_ADC_DLT_VALUE_CH0_LVAL				GENMASK(11, 0)
#define LIGHT_ADC_DLT_VALUE_CH0_LVAL_OFF			(0)
#define LIGHT_ADC_DLT_VALUE_CH1_HVAL				GENMASK(27, 16)
#define LIGHT_ADC_DLT_VALUE_CH1_HVAL_OFF			(16)
#define LIGHT_ADC_DLT_VALUE_CH1_LVAL				GENMASK(11, 0)
#define LIGHT_ADC_DLT_VALUE_CH1_LVAL_OFF			(0)

#define LIGHT_ADC_FIFO_DATA_SIZE 32
#define LIGHT_ADC_PHY_ENCTR	0x8e0
#define LIGHT_ADC_TIMEOUT	500000

enum vol_ref {
	LIGHT_ADC_VOL_VREF,
	LIGHT_ADC_VOL_INTE,
};

enum input_mode_sel {
	LIGHT_ADC_SINGLE_ENDED_INPUTS,
	LIGHT_ADC_DIFFERENTIAL_INPUTS,
};

enum selres_sel {
	LIGHT_ADC_SELRES_6BIT = 6,
	LIGHT_ADC_SELRES_8BIT = 8,
	LIGHT_ADC_SELRES_10BIT = 10,
	LIGHT_ADC_SELRES_12BIT = 12,
};

enum offset_mode_sel {
	LIGHT_ADC_OFFSET_DIS = 0,
	LIGHT_ADC_OFFSET_EN,
};

enum conversion_mode_sel {
	LIGHT_ADC_MODE_SINGLE,
	LIGHT_ADC_MODE_CONTINOUS,
};

enum clk_sel {
	LIGHT_ADC_FCLK_TYP_1M,
};

enum int_actual_mask {
	LIGHT_ADC_ACTUAL_CH0,
	LIGHT_ADC_ACTUAL_CH1,
	LIGHT_ADC_ACTUAL_ALL,

};
enum int_delta_mask {
	LIGHT_ADC_DETAL_CH0,
	LIGHT_ADC_DETAL_CH1,
	LIGHT_ADC_DETAL_ALL,
};

struct light_adc_feature {
	enum selres_sel			selres_sel;
	enum input_mode_sel		input_mode;
	enum vol_ref			vol_ref;
	enum offset_mode_sel		offset_mode;
	enum conversion_mode_sel 	conv_mode;
	enum clk_sel			clk_sel;
	enum int_actual_mask		int_actual;
	enum int_delta_mask		int_detal;
};

struct light_adc {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;

	u32 vref_uv;
	u32 value;
	struct regulator *vref;
	struct light_adc_feature adc_feature;
	u32 current_clk;
	u32 ch0_offmeas;
	u32 ch1_offmeas;

	struct completion completion;
};


static inline void light_adc_cfg_init(struct light_adc *info)
{
	struct light_adc_feature *adc_feature = &info->adc_feature;

	/* set default Configuration for ADC controller */
	adc_feature->selres_sel = LIGHT_ADC_SELRES_12BIT;
	adc_feature->input_mode = LIGHT_ADC_SINGLE_ENDED_INPUTS;
	adc_feature->vol_ref = LIGHT_ADC_VOL_VREF;
	adc_feature->offset_mode = LIGHT_ADC_OFFSET_DIS;
	adc_feature->conv_mode = LIGHT_ADC_MODE_SINGLE;
	adc_feature->clk_sel = LIGHT_ADC_FCLK_TYP_1M;

	adc_feature->int_actual = LIGHT_ADC_ACTUAL_ALL;
	adc_feature->int_detal = LIGHT_ADC_DETAL_ALL;

	info->ch0_offmeas = 0;
	info->ch1_offmeas = 0;
}

static void light_adc_reg_set(struct light_adc *info)
{
	struct light_adc_feature *adc_feature = &info->adc_feature;
	uint32_t phy_cfg = 0;
	uint32_t op_ctrl = 0;

	/* phy_cfg */
	switch (adc_feature->selres_sel) {
	case LIGHT_ADC_SELRES_6BIT:
		phy_cfg |= LIGHT_ADC_PHY_CFG_SELRES_6BIT;
		break;
	case LIGHT_ADC_SELRES_8BIT:
		phy_cfg |= LIGHT_ADC_PHY_CFG_SELRES_8BIT;
		break;
	case LIGHT_ADC_SELRES_10BIT:
		phy_cfg |= LIGHT_ADC_PHY_CFG_SELRES_10BIT;
		break;
	case LIGHT_ADC_SELRES_12BIT:
		phy_cfg |= LIGHT_ADC_PHY_CFG_SELRES_12BIT;
		break;
	default:
		break;
	}

	switch (adc_feature->input_mode) {
	case LIGHT_ADC_SINGLE_ENDED_INPUTS:
		phy_cfg |= LIGHT_ADC_PHY_CFG_SELDIFF_SINGLE_ENDED_INPUTS;
		break;
	case LIGHT_ADC_DIFFERENTIAL_INPUTS:
		phy_cfg |= LIGHT_ADC_PHY_CFG_SELDIFF_DIFFERENTIAL_INPUTS;
		break;
	default:
		break;
	}

	switch (adc_feature->vol_ref) {
	case LIGHT_ADC_VOL_VREF:
		phy_cfg |= LIGHT_ADC_PHY_CFG_SELBG_EXTERNAL | LIGHT_ADC_PHY_CFG_SELREF_EXT;
		break;
	case LIGHT_ADC_VOL_INTE:
		phy_cfg |= LIGHT_ADC_PHY_CFG_SELBG_INTERNAL | LIGHT_ADC_PHY_CFG_SELREF_INTERNAL;
		break;
	default:
		break;
	}

	/* op_ctrl */
	switch (adc_feature->conv_mode) {
	case LIGHT_ADC_MODE_SINGLE:
		op_ctrl |= LIGHT_ADC_OP_CTRL_MODE_SINGLE;
		break;
	case LIGHT_ADC_MODE_CONTINOUS:
		op_ctrl |= LIGHT_ADC_OP_CTRL_MODE_CONTINOUS;
		break;
	default:
		break;
	}

	writel(phy_cfg, info->regs + LIGHT_ADC_PHY_CFG);
	writel(op_ctrl, info->regs + LIGHT_ADC_OP_CTRL);
	writel(LIGHT_ADC_PHY_ENCTR, info->regs + LIGHT_ADC_PHY_TEST);
	/* disable the irq */
	writel(0xff, info->regs + LIGHT_ADC_INT_CTRL1);
	writel(0xff, info->regs + LIGHT_ADC_INT_CTRL2);
	if (adc_feature->conv_mode == LIGHT_ADC_MODE_CONTINOUS)
		writel(LIGHT_ADC_PHY_CTRL_ENADC_EN, info->regs + LIGHT_ADC_PHY_CTRL);
}

static void light_adc_fclk_set(struct light_adc *info)
{
	struct light_adc_feature *adc_feature = &(info->adc_feature);
	int fclk_ctrl = 0;
	int start_time = 0;
	int sample_time = 0;

	switch (adc_feature->clk_sel) {
	case LIGHT_ADC_FCLK_TYP_1M:
		fclk_ctrl = LIGHT_ADC_FCLK_CTRL_TYP_1M;
		start_time = LIGHT_ADC_START_TIME_TYP_1M;
		if (adc_feature->selres_sel == LIGHT_ADC_SELRES_6BIT)
			sample_time = LIGHT_ADC_SAMPLE_TIME_TYP_6BIT;
		else if (adc_feature->selres_sel == LIGHT_ADC_SELRES_8BIT)
			sample_time = LIGHT_ADC_SAMPLE_TIME_TYP_8BIT;
		else if (adc_feature->selres_sel == LIGHT_ADC_SELRES_10BIT)
			sample_time = LIGHT_ADC_SAMPLE_TIME_TYP_10BIT;
		else if (adc_feature->selres_sel == LIGHT_ADC_SELRES_12BIT)
			sample_time = LIGHT_ADC_SAMPLE_TIME_TYP_12BIT;
		else {
			pr_err("[%s,%d]invalid selres select \n", __func__, __LINE__);
			return;
		}
		break;
	default:
		break;
	}
	writel(fclk_ctrl, info->regs + LIGHT_ADC_FCLK_CTRL);
	writel(start_time, info->regs + LIGHT_ADC_START_TIME);
	writel(sample_time, info->regs + LIGHT_ADC_SAMPLE_TIME);
}

static void light_adc_hw_init(struct light_adc *info)
{
	light_adc_reg_set(info);

	light_adc_fclk_set(info);
}

#define LIGHT_ADC_CHAN(_idx, _chan_type) {			\
	.type = (_chan_type),					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
}

static const struct iio_chan_spec light_adc_iio_channels[] = {
	LIGHT_ADC_CHAN(0, IIO_VOLTAGE),
	LIGHT_ADC_CHAN(1, IIO_VOLTAGE),
	LIGHT_ADC_CHAN(2, IIO_VOLTAGE),
	LIGHT_ADC_CHAN(3, IIO_VOLTAGE),
	LIGHT_ADC_CHAN(4, IIO_VOLTAGE),
	LIGHT_ADC_CHAN(5, IIO_VOLTAGE),
	LIGHT_ADC_CHAN(6, IIO_VOLTAGE),
	LIGHT_ADC_CHAN(7, IIO_VOLTAGE),
	/* sentinel */
};

static irqreturn_t light_adc_isr(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct light_adc *info = iio_priv(indio_dev);
	/* TBD */
	complete(&info->completion);
	return IRQ_HANDLED;
}

static int light_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val,
			int *val2,
			long mask)
{
	struct light_adc *info = iio_priv(indio_dev);
	int tmp;
	long ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		if (info->adc_feature.conv_mode == LIGHT_ADC_MODE_SINGLE) {
			uint single_retry = LIGHT_ADC_FIFO_DATA_SIZE;
			uint ievent;
			uint vld_flag;
			uint op_ctrl = 0;
			uint real_chan;
			uint phy_ctrl;

			op_ctrl = readl(info->regs + LIGHT_ADC_OP_CTRL);
			op_ctrl &= ~LIGHT_ADC_OP_CTRL_CH_EN_ALL;
			op_ctrl |= (BIT(chan->channel + LIGHT_ADC_OP_CTRL_CH_EN_0) & LIGHT_ADC_OP_CTRL_CH_EN_ALL);
			writel(op_ctrl, info->regs + LIGHT_ADC_OP_CTRL);

			writel(LIGHT_ADC_PHY_CTRL_ENADC_EN, info->regs + LIGHT_ADC_PHY_CTRL);

			vld_flag = LIGHT_ADC_SAMPLE_DATA_CH0_VLD;

			while (single_retry--) {
				writel(LIGHT_ADC_OP_SINGLE_START_EN, info->regs + LIGHT_ADC_OP_SINGLE_START);
				/* wait the sampling result */
				ret  = readl_poll_timeout(info->regs + LIGHT_ADC_SAMPLE_DATA,
							  ievent,
							  ievent & vld_flag, 100, LIGHT_ADC_TIMEOUT);
				if (ret)
					pr_err("wait the sampling timeout\n");

				real_chan = (ievent & LIGHT_ADC_SAMPLE_DATA_CH0_NUMBER) >> LIGHT_ADC_SAMPLE_DATA_CH0_NUMBER_OFF;
				if (real_chan == chan->channel)
					break;
			}

			phy_ctrl = readl(info->regs + LIGHT_ADC_PHY_CTRL);
			phy_ctrl &= ~ LIGHT_ADC_PHY_CTRL_ENADC_EN;
			writel(phy_ctrl, info->regs + LIGHT_ADC_PHY_CTRL);

			/* read the sampling data */
			*val = (ievent & LIGHT_ADC_SAMPLE_DATA_CH0) >> LIGHT_ADC_SAMPLE_DATA_CH0_OFF;
		} else {
			uint ievent;
			uint vld_flag;
			uint op_single;
			uint op_ctrl = 0;

			op_ctrl = readl(info->regs + LIGHT_ADC_OP_CTRL);
			op_ctrl &= ~LIGHT_ADC_OP_CTRL_CH_EN_ALL;
			op_ctrl |= (BIT(chan->channel + LIGHT_ADC_OP_CTRL_CH_EN_0) & LIGHT_ADC_OP_CTRL_CH_EN_ALL);
			writel(op_ctrl, info->regs + LIGHT_ADC_OP_CTRL);

			op_single = readl(info->regs + LIGHT_ADC_OP_SINGLE_START);
			op_single &= ~LIGHT_ADC_OP_SINGLE_START_EN;
			writel(op_single, info->regs + LIGHT_ADC_OP_SINGLE_START);

			vld_flag = LIGHT_ADC_SAMPLE_DATA_CH0_VLD | LIGHT_ADC_SAMPLE_DATA_CH1_VLD;

			/* wait the sampling result */
			ret  = readl_poll_timeout(info->regs + LIGHT_ADC_SAMPLE_DATA,
						  ievent,
						  ievent & vld_flag, 10, LIGHT_ADC_TIMEOUT);
			if (ret)
				pr_err("wait the sampling timeout\n");

			/* read the sampling data */
			tmp = readl(info->regs + LIGHT_ADC_SAMPLE_DATA);
			if (tmp & LIGHT_ADC_SAMPLE_DATA_CH0_VLD)
				*val = (tmp & LIGHT_ADC_SAMPLE_DATA_CH0) >> LIGHT_ADC_SAMPLE_DATA_CH0_OFF;
			else {
				*val = (tmp & LIGHT_ADC_SAMPLE_DATA_CH1) >> LIGHT_ADC_SAMPLE_DATA_CH1_OFF;
			}

		}

		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = info->vref_uv / 1000;
		*val2 = info->adc_feature.selres_sel;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = info->current_clk;
		*val2 = 0;
		return IIO_VAL_INT;

	default:
		break;
	}

	return -EINVAL;
}

static void light_adc_reset(struct light_adc *info)
{
	u32 tmp;

	tmp = readl(info->regs + LIGHT_ADC_PHY_CTRL);
	tmp |= LIGHT_ADC_PHY_CTRL_RST_EN;
	writel(tmp, info->regs + LIGHT_ADC_PHY_CTRL);
	udelay(10);
	tmp &= ~LIGHT_ADC_PHY_CTRL_RST_EN;
	writel(tmp, info->regs + LIGHT_ADC_PHY_CTRL);
}

static void light_adc_set_clk(struct light_adc *info, int val)
{
	int fclk_ctrl;
	u32 count;
	u32 apb_clk = clk_get_rate(info->clk);

	fclk_ctrl = readl(info->regs + LIGHT_ADC_FCLK_CTRL);

	count = DIV_ROUND_UP(apb_clk, val);
	info->current_clk = apb_clk / count;
	fclk_ctrl &= ~LIGHT_ADC_FCLK_CTRL_FCLLK_DIV;
	fclk_ctrl |= count;

	writel(fclk_ctrl, info->regs + LIGHT_ADC_FCLK_CTRL);
}

static int light_write_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int val,
			int val2,
			long mask)
{
	struct light_adc *info = iio_priv(indio_dev);

	if (mask != IIO_CHAN_INFO_SAMP_FREQ)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);
	light_adc_set_clk(info, val);
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

static const struct iio_info light_adc_iio_info = {
	.read_raw = &light_read_raw,
	.write_raw = &light_write_raw,
};

static const struct of_device_id light_adc_match[] = {
	{ .compatible = "thead,light-adc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, light_adc_match);


static ssize_t light_adc_res_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct light_adc *info = iio_priv(indio_dev);
	size_t bufpos = 0, count = 5;

	snprintf(buf + bufpos, count - bufpos, "%.*x: ", 4, info->adc_feature.selres_sel);
	bufpos += 4;
	buf[bufpos++] = '\n';

	return bufpos;
}

static ssize_t light_adc_res_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct light_adc *info = iio_priv(indio_dev);
	char *start = (char *)buf;
	unsigned long res;

	if (kstrtoul(start, 0, &res))
		return -EINVAL;

	switch (res) {
	case LIGHT_ADC_SELRES_6BIT:
	case LIGHT_ADC_SELRES_8BIT:
	case LIGHT_ADC_SELRES_10BIT:
	case LIGHT_ADC_SELRES_12BIT:
		info->adc_feature.selres_sel = res;
		light_adc_reset(info);
		light_adc_hw_init(info);
		break;
	default:
		dev_err(dev, "not support res\n");
		return -EINVAL;
	}

	return size;
}

static DEVICE_ATTR(light_adc_res, 0644, light_adc_res_show, light_adc_res_store);

static int light_adc_probe(struct platform_device *pdev)
{
	struct light_adc *info;
	struct iio_dev *indio_dev;
	struct resource *mem;
	int irq;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(struct light_adc));
	if (!indio_dev) {
		dev_err(&pdev->dev, "Failed allocating iio device\n");
		return -ENOMEM;
	}

	info = iio_priv(indio_dev);
	info->dev = &pdev->dev;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(info->dev, irq,
				light_adc_isr, 0,
				dev_name(&pdev->dev), indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed requesting irq, irq = %d\n", irq);
		return ret;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	info->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(info->regs))
		return PTR_ERR(info->regs);

	info->clk = devm_clk_get(&pdev->dev, "adc");
	if (IS_ERR(info->clk)) {
		dev_err(&pdev->dev, "failed getting clock, err = %ld\n",
						PTR_ERR(info->clk));
		return PTR_ERR(info->clk);
	}

	info->vref = devm_regulator_get(&pdev->dev, "vref");
	if (IS_ERR(info->vref))
		return PTR_ERR(info->vref);

	ret = regulator_enable(info->vref);
	if (ret)
		return ret;

	info->vref_uv = regulator_get_voltage(info->vref);

	platform_set_drvdata(pdev, indio_dev);

	init_completion(&info->completion);

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &light_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = light_adc_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(light_adc_iio_channels);

	ret = clk_prepare_enable(info->clk);
	if (ret) {
		dev_err(&pdev->dev,
			"Could not prepare or enable the clock.\n");
		goto error_adc_clk_enable;
	}

	light_adc_cfg_init(info);
	light_adc_reset(info);
	light_adc_hw_init(info);

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't register the device.\n");
		goto error_iio_device_register;
	}

	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_light_adc_res.attr);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create adc debug sysfs.\n");
		goto error_iio_device_register;
	}

	dev_info(&pdev->dev, "Thead light adc registered.\n");
	return 0;

error_iio_device_register:
	clk_disable_unprepare(info->clk);
error_adc_clk_enable:
	regulator_disable(info->vref);

	return ret;
}

static int light_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct light_adc *info = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	regulator_disable(info->vref);
	clk_disable_unprepare(info->clk);

	return 0;
}

static int __maybe_unused light_adc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct light_adc *info = iio_priv(indio_dev);
	int phy_ctrl;

	phy_ctrl = readl(info->regs + LIGHT_ADC_PHY_CTRL);
	phy_ctrl &= ~ LIGHT_ADC_PHY_CTRL_ENADC_EN;
	writel(phy_ctrl, info->regs + LIGHT_ADC_PHY_CTRL);

	clk_disable_unprepare(info->clk);
	regulator_disable(info->vref);

	return 0;
}

static int __maybe_unused light_adc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct light_adc *info = iio_priv(indio_dev);
	int ret;

	ret = regulator_enable(info->vref);
	if (ret)
		return ret;

	ret = clk_prepare_enable(info->clk);
	if (ret)
		goto disable_reg;

	light_adc_reset(info);
	light_adc_set_clk(info, info->current_clk);
	light_adc_hw_init(info);

	return 0;

disable_reg:
	regulator_disable(info->vref);
	return ret;
}

static SIMPLE_DEV_PM_OPS(light_adc_pm_ops, light_adc_suspend, light_adc_resume);

static struct platform_driver light_adc_driver = {
	.probe          = light_adc_probe,
	.remove         = light_adc_remove,
	.driver         = {
		.name   = DRIVER_NAME,
		.of_match_table = light_adc_match,
		.pm     = &light_adc_pm_ops,
	},
};

module_platform_driver(light_adc_driver);

MODULE_AUTHOR("fugang.duan <duanfugang.dfg@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead Light ADC driver");
MODULE_LICENSE("GPL v2");
