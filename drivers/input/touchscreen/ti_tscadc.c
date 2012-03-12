/*
 * TI Touch Screen driver
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
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
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/input/ti_tscadc.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>

#define TSCADC_REG_IRQEOI		0x020
#define TSCADC_REG_RAWIRQSTATUS		0x024
#define TSCADC_REG_IRQSTATUS		0x028
#define TSCADC_REG_IRQENABLE		0x02C
#define TSCADC_REG_IRQCLR		0x030
#define TSCADC_REG_IRQWAKEUP		0x034
#define TSCADC_REG_CTRL			0x040
#define TSCADC_REG_ADCFSM		0x044
#define TSCADC_REG_CLKDIV		0x04C
#define TSCADC_REG_SE			0x054
#define TSCADC_REG_IDLECONFIG		0x058
#define TSCADC_REG_CHARGECONFIG		0x05C
#define TSCADC_REG_CHARGEDELAY		0x060
#define TSCADC_REG_STEPCONFIG(n)	(0x64 + ((n-1) * 8))
#define TSCADC_REG_STEPDELAY(n)		(0x68 + ((n-1) * 8))
#define TSCADC_REG_STEPCONFIG13		0x0C4
#define TSCADC_REG_STEPDELAY13		0x0C8
#define TSCADC_REG_STEPCONFIG14		0x0CC
#define TSCADC_REG_STEPDELAY14		0x0D0
#define TSCADC_REG_FIFO0CNT		0xE4
#define TSCADC_REG_FIFO0THR		0xE8
#define TSCADC_REG_FIFO1CNT		0xF0
#define TSCADC_REG_FIFO1THR		0xF4
#define TSCADC_REG_FIFO0		0x100
#define TSCADC_REG_FIFO1		0x200

/*	Register Bitfields	*/
#define TSCADC_IRQWKUP_ENB		BIT(0)
#define TSCADC_IRQWKUP_DISABLE		0x00
#define TSCADC_STPENB_STEPENB		0x7FFF
#define TSCADC_IRQENB_FIFO0THRES	BIT(2)
#define TSCADC_IRQENB_FIFO1THRES	BIT(5)
#define TSCADC_IRQENB_PENUP		BIT(9)
#define TSCADC_IRQENB_HW_PEN		BIT(0)
#define TSCADC_STEPCONFIG_MODE_HWSYNC	0x2
#define TSCADC_STEPCONFIG_2SAMPLES_AVG	(1 << 4)
#define TSCADC_STEPCONFIG_XPP		BIT(5)
#define TSCADC_STEPCONFIG_XNN		BIT(6)
#define TSCADC_STEPCONFIG_YPP		BIT(7)
#define TSCADC_STEPCONFIG_YNN		BIT(8)
#define TSCADC_STEPCONFIG_XNP		BIT(9)
#define TSCADC_STEPCONFIG_YPN		BIT(10)
#define TSCADC_STEPCONFIG_RFP		(1 << 12)
#define TSCADC_STEPCONFIG_INM		(1 << 18)
#define TSCADC_STEPCONFIG_INP_4		(1 << 19)
#define TSCADC_STEPCONFIG_INP		(1 << 20)
#define TSCADC_STEPCONFIG_INP_5		(1 << 21)
#define TSCADC_STEPCONFIG_FIFO1		(1 << 26)
#define TSCADC_STEPCONFIG_IDLE_INP	(1 << 22)
#define TSCADC_STEPCONFIG_OPENDLY	0x018
#define TSCADC_STEPCONFIG_SAMPLEDLY	0x88
#define TSCADC_STEPCONFIG_Z1		(3 << 19)
#define TSCADC_STEPCHARGE_INM_SWAP	BIT(16)
#define TSCADC_STEPCHARGE_INM		BIT(15)
#define TSCADC_STEPCHARGE_INP_SWAP	BIT(20)
#define TSCADC_STEPCHARGE_INP		BIT(19)
#define TSCADC_STEPCHARGE_RFM		(1 << 23)
#define TSCADC_STEPCHARGE_DELAY		0x1
#define TSCADC_CNTRLREG_TSCSSENB	BIT(0)
#define TSCADC_CNTRLREG_STEPID		BIT(1)
#define TSCADC_CNTRLREG_STEPCONFIGWRT	BIT(2)
#define TSCADC_CNTRLREG_TSCENB		BIT(7)
#define TSCADC_CNTRLREG_4WIRE		(0x1 << 5)
#define TSCADC_CNTRLREG_5WIRE		(0x1 << 6)
#define TSCADC_CNTRLREG_8WIRE		(0x3 << 5)
#define TSCADC_ADCFSM_STEPID		0x10
#define TSCADC_ADCFSM_FSM		BIT(5)

#define ADC_CLK				3000000

#define MAX_12BIT                       ((1 << 12) - 1)

int pen = 1;
unsigned int bckup_x = 0, bckup_y = 0;

struct tscadc {
	struct input_dev	*input;
	int			wires;
	int			analog_input;
	int			x_plate_resistance;
	int			irq;
	void __iomem		*tsc_base;
	unsigned int		ctrl;
};

static unsigned int tscadc_readl(struct tscadc *ts, unsigned int reg)
{
	return readl(ts->tsc_base + reg);
}

static void tscadc_writel(struct tscadc *tsc, unsigned int reg,
					unsigned int val)
{
	writel(val, tsc->tsc_base + reg);
}

static void tsc_step_config(struct tscadc *ts_dev)
{
	unsigned int	stepconfigx = 0, stepconfigy = 0;
	unsigned int	delay, chargeconfig = 0;
	unsigned int	stepconfigz1 = 0, stepconfigz2 = 0;
	int i;

	/* Configure the Step registers */

	delay = TSCADC_STEPCONFIG_SAMPLEDLY | TSCADC_STEPCONFIG_OPENDLY;

	stepconfigx = TSCADC_STEPCONFIG_MODE_HWSYNC |
			TSCADC_STEPCONFIG_2SAMPLES_AVG | TSCADC_STEPCONFIG_XPP;

	switch (ts_dev->wires) {
	case 4:
		if (ts_dev->analog_input == 0)
			stepconfigx |= TSCADC_STEPCONFIG_INP_4 |
				TSCADC_STEPCONFIG_YPN;
		else
			stepconfigx |= TSCADC_STEPCONFIG_INP |
				TSCADC_STEPCONFIG_XNN;
		break;
	case 5:
		stepconfigx |= TSCADC_STEPCONFIG_YNN |
				TSCADC_STEPCONFIG_INP_5;
		if (ts_dev->analog_input == 0)
			stepconfigx |= TSCADC_STEPCONFIG_XNP |
				TSCADC_STEPCONFIG_YPN;
		else
			stepconfigx |= TSCADC_STEPCONFIG_XNN |
				TSCADC_STEPCONFIG_YPP;
		break;
	case 8:
		if (ts_dev->analog_input == 0)
			stepconfigx |= TSCADC_STEPCONFIG_INP_4 |
				TSCADC_STEPCONFIG_YPN;
		else
			stepconfigx |= TSCADC_STEPCONFIG_INP |
				TSCADC_STEPCONFIG_XNN;
		break;
	}

	for (i = 1; i < 7; i++) {
		tscadc_writel(ts_dev, TSCADC_REG_STEPCONFIG(i), stepconfigx);
		tscadc_writel(ts_dev, TSCADC_REG_STEPDELAY(i), delay);
	}

	stepconfigy = TSCADC_STEPCONFIG_MODE_HWSYNC |
			TSCADC_STEPCONFIG_2SAMPLES_AVG | TSCADC_STEPCONFIG_YNN |
			TSCADC_STEPCONFIG_INM | TSCADC_STEPCONFIG_FIFO1;
	switch (ts_dev->wires) {
	case 4:
		if (ts_dev->analog_input == 0)
			stepconfigy |= TSCADC_STEPCONFIG_XNP;
		else
			stepconfigy |= TSCADC_STEPCONFIG_YPP;
		break;
	case 5:
		stepconfigy |= TSCADC_STEPCONFIG_XPP | TSCADC_STEPCONFIG_INP_5;
		if (ts_dev->analog_input == 0)
			stepconfigy |= TSCADC_STEPCONFIG_XNN |
				TSCADC_STEPCONFIG_YPP;
		else
			stepconfigy |= TSCADC_STEPCONFIG_XNP |
				TSCADC_STEPCONFIG_YPN;
		break;
	case 8:
		if (ts_dev->analog_input == 0)
			stepconfigy |= TSCADC_STEPCONFIG_XNP;
		else
			stepconfigy |= TSCADC_STEPCONFIG_YPP;
		break;
	}

	for (i = 7; i < 13; i++) {
		tscadc_writel(ts_dev, TSCADC_REG_STEPCONFIG(i), stepconfigy);
		tscadc_writel(ts_dev, TSCADC_REG_STEPDELAY(i), delay);
	}

	chargeconfig = TSCADC_STEPCONFIG_XPP |
			TSCADC_STEPCONFIG_YNN |
			TSCADC_STEPCONFIG_RFP |
			TSCADC_STEPCHARGE_RFM;
	if (ts_dev->analog_input == 0)
		chargeconfig |= TSCADC_STEPCHARGE_INM_SWAP |
			TSCADC_STEPCHARGE_INP_SWAP;
	else
		chargeconfig |= TSCADC_STEPCHARGE_INM | TSCADC_STEPCHARGE_INP;
	tscadc_writel(ts_dev, TSCADC_REG_CHARGECONFIG, chargeconfig);
	tscadc_writel(ts_dev, TSCADC_REG_CHARGEDELAY, TSCADC_STEPCHARGE_DELAY);

	 /* Configure to calculate pressure */
	stepconfigz1 = TSCADC_STEPCONFIG_MODE_HWSYNC |
				TSCADC_STEPCONFIG_2SAMPLES_AVG |
				TSCADC_STEPCONFIG_XNP |
				TSCADC_STEPCONFIG_YPN | TSCADC_STEPCONFIG_INM;
	stepconfigz2 = stepconfigz1 | TSCADC_STEPCONFIG_Z1 |
				TSCADC_STEPCONFIG_FIFO1;
	tscadc_writel(ts_dev, TSCADC_REG_STEPCONFIG13, stepconfigz1);
	tscadc_writel(ts_dev, TSCADC_REG_STEPDELAY13, delay);
	tscadc_writel(ts_dev, TSCADC_REG_STEPCONFIG14, stepconfigz2);
	tscadc_writel(ts_dev, TSCADC_REG_STEPDELAY14, delay);

	tscadc_writel(ts_dev, TSCADC_REG_SE, TSCADC_STPENB_STEPENB);
}

static void tsc_idle_config(struct tscadc *ts_config)
{
	/* Idle mode touch screen config */
	unsigned int	 idleconfig;

	idleconfig = TSCADC_STEPCONFIG_YNN |
			TSCADC_STEPCONFIG_INM | TSCADC_STEPCONFIG_IDLE_INP;
	if (ts_config->analog_input == 0)
		idleconfig |= TSCADC_STEPCONFIG_XNN;
	else
		idleconfig |= TSCADC_STEPCONFIG_YPN;

	tscadc_writel(ts_config, TSCADC_REG_IDLECONFIG, idleconfig);
}

static irqreturn_t tscadc_interrupt(int irq, void *dev)
{
	struct tscadc		*ts_dev = (struct tscadc *)dev;
	struct input_dev	*input_dev = ts_dev->input;
	unsigned int		status, irqclr = 0;
	int			i;
	int			fsm = 0, fifo0count = 0, fifo1count = 0;
	unsigned int		readx1 = 0, ready1 = 0;
	unsigned int		prev_val_x = ~0, prev_val_y = ~0;
	unsigned int		prev_diff_x = ~0, prev_diff_y = ~0;
	unsigned int		cur_diff_x = 0, cur_diff_y = 0;
	unsigned int		val_x = 0, val_y = 0, diffx = 0, diffy = 0;
	unsigned int		z1 = 0, z2 = 0, z = 0;

	status = tscadc_readl(ts_dev, TSCADC_REG_IRQSTATUS);

	if (status & TSCADC_IRQENB_FIFO1THRES) {
		fifo0count = tscadc_readl(ts_dev, TSCADC_REG_FIFO0CNT);
		fifo1count = tscadc_readl(ts_dev, TSCADC_REG_FIFO1CNT);
		for (i = 0; i < (fifo0count-1); i++) {
			readx1 = tscadc_readl(ts_dev, TSCADC_REG_FIFO0);
			readx1 = readx1 & 0xfff;
			if (readx1 > prev_val_x)
				cur_diff_x = readx1 - prev_val_x;
			else
				cur_diff_x = prev_val_x - readx1;

			if (cur_diff_x < prev_diff_x) {
				prev_diff_x = cur_diff_x;
				val_x = readx1;
			}

			prev_val_x = readx1;
			ready1 = tscadc_readl(ts_dev, TSCADC_REG_FIFO1);
				ready1 &= 0xfff;
			if (ready1 > prev_val_y)
				cur_diff_y = ready1 - prev_val_y;
			else
				cur_diff_y = prev_val_y - ready1;

			if (cur_diff_y < prev_diff_y) {
				prev_diff_y = cur_diff_y;
				val_y = ready1;
			}

			prev_val_y = ready1;
		}

		if (val_x > bckup_x) {
			diffx = val_x - bckup_x;
			diffy = val_y - bckup_y;
		} else {
			diffx = bckup_x - val_x;
			diffy = bckup_y - val_y;
		}
		bckup_x = val_x;
		bckup_y = val_y;

		z1 = ((tscadc_readl(ts_dev, TSCADC_REG_FIFO0)) & 0xfff);
		z2 = ((tscadc_readl(ts_dev, TSCADC_REG_FIFO1)) & 0xfff);

		if ((z1 != 0) && (z2 != 0)) {
			/*
			 * cal pressure using formula
			 * Resistance(touch) = x plate resistance *
			 * x postion/4096 * ((z2 / z1) - 1)
			 */
			z = z2 - z1;
			z *= val_x;
			z *= ts_dev->x_plate_resistance;
			z /= z1;
			z = (z + 2047) >> 12;

			/*
			 * Sample found inconsistent by debouncing
			 * or pressure is beyond the maximum.
			 * Don't report it to user space.
			 */
			if (pen == 0) {
				if ((diffx < 15) && (diffy < 15)
						&& (z <= MAX_12BIT)) {
					input_report_abs(input_dev, ABS_X,
							val_x);
					input_report_abs(input_dev, ABS_Y,
							val_y);
					input_report_abs(input_dev, ABS_PRESSURE,
							z);
					input_report_key(input_dev, BTN_TOUCH,
							1);
					input_sync(input_dev);
				}
			}
		}
		irqclr |= TSCADC_IRQENB_FIFO1THRES;
	}

	udelay(315);

	status = tscadc_readl(ts_dev, TSCADC_REG_RAWIRQSTATUS);
	if (status & TSCADC_IRQENB_PENUP) {
		/* Pen up event */
		fsm = tscadc_readl(ts_dev, TSCADC_REG_ADCFSM);
		if (fsm == 0x10) {
			pen = 1;
			bckup_x = 0;
			bckup_y = 0;
			input_report_key(input_dev, BTN_TOUCH, 0);
			input_report_abs(input_dev, ABS_PRESSURE, 0);
			input_sync(input_dev);
		} else {
			pen = 0;
		}
		irqclr |= TSCADC_IRQENB_PENUP;
	}
	irqclr |= TSCADC_IRQENB_HW_PEN;

	tscadc_writel(ts_dev, TSCADC_REG_IRQSTATUS, irqclr);

	/* check pending interrupts */
	tscadc_writel(ts_dev, TSCADC_REG_IRQEOI, 0x0);

	tscadc_writel(ts_dev, TSCADC_REG_SE, TSCADC_STPENB_STEPENB);
	return IRQ_HANDLED;
}

/*
* The functions for inserting/removing driver as a module.
*/

static	int __devinit tscadc_probe(struct platform_device *pdev)
{
	struct tscadc			*ts_dev;
	struct input_dev		*input_dev;
	int				err;
	int				clk_value;
	int				clock_rate, irqenable, ctrl;
	struct	tsc_data		*pdata = pdev->dev.platform_data;
	struct resource			*res;
	struct clk			*clk;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no memory resource defined.\n");
		return -EINVAL;
	}

	/* Allocate memory for device */
	ts_dev = kzalloc(sizeof(struct tscadc), GFP_KERNEL);
	if (!ts_dev) {
		dev_err(&pdev->dev, "failed to allocate memory.\n");
		return -ENOMEM;
	}

	ts_dev->irq = platform_get_irq(pdev, 0);
	if (ts_dev->irq < 0) {
		dev_err(&pdev->dev, "no irq ID is specified.\n");
		return -ENODEV;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&pdev->dev, "failed to allocate input device.\n");
		err = -ENOMEM;
		goto err_free_mem;
	}
	ts_dev->input = input_dev;

	res =  request_mem_region(res->start, resource_size(res), pdev->name);
	if (!res) {
		dev_err(&pdev->dev, "failed to reserve registers.\n");
		err = -EBUSY;
		goto err_free_mem;
	}

	ts_dev->tsc_base = ioremap(res->start, resource_size(res));
	if (!ts_dev->tsc_base) {
		dev_err(&pdev->dev, "failed to map registers.\n");
		err = -ENOMEM;
		goto err_release_mem;
	}

	err = request_irq(ts_dev->irq, tscadc_interrupt, IRQF_DISABLED,
				pdev->dev.driver->name, ts_dev);
	if (err) {
		dev_err(&pdev->dev, "failed to allocate irq.\n");
		goto err_unmap_regs;
	}

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	clk = clk_get(&pdev->dev, "adc_tsc_fck");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get TSC fck\n");
		err = PTR_ERR(clk);
		goto err_free_irq;
	}
	clock_rate = clk_get_rate(clk);
	clk_value = clock_rate / ADC_CLK;
	if (clk_value < 7) {
		dev_err(&pdev->dev, "clock input less than min clock requirement\n");
		err = -EINVAL;
		goto err_fail;
	}
	/* TSCADC_CLKDIV needs to be configured to the value minus 1 */
	clk_value = clk_value - 1;
	tscadc_writel(ts_dev, TSCADC_REG_CLKDIV, clk_value);

	ts_dev->wires = pdata->wires;
	ts_dev->analog_input = pdata->analog_input;
	ts_dev->x_plate_resistance = pdata->x_plate_resistance;

	/* Set the control register bits */
	ctrl = TSCADC_CNTRLREG_STEPCONFIGWRT |
			TSCADC_CNTRLREG_TSCENB |
			TSCADC_CNTRLREG_STEPID;
	switch (ts_dev->wires) {
	case 4:
		ctrl |= TSCADC_CNTRLREG_4WIRE;
		break;
	case 5:
		ctrl |= TSCADC_CNTRLREG_5WIRE;
		break;
	case 8:
		ctrl |= TSCADC_CNTRLREG_8WIRE;
		break;
	}
	tscadc_writel(ts_dev, TSCADC_REG_CTRL, ctrl);
	ts_dev->ctrl = ctrl;

	/* Set register bits for Idel Config Mode */
	tsc_idle_config(ts_dev);

	/* IRQ Enable */
	irqenable = TSCADC_IRQENB_FIFO1THRES;
	tscadc_writel(ts_dev, TSCADC_REG_IRQENABLE, irqenable);

	tsc_step_config(ts_dev);

	tscadc_writel(ts_dev, TSCADC_REG_FIFO1THR, 6);

	ctrl |= TSCADC_CNTRLREG_TSCSSENB;
	tscadc_writel(ts_dev, TSCADC_REG_CTRL, ctrl);

	input_dev->name = "ti-tsc-adcc";
	input_dev->dev.parent = &pdev->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);

	/* register to the input system */
	err = input_register_device(input_dev);
	if (err)
		goto err_fail;

	device_init_wakeup(&pdev->dev, true);
	platform_set_drvdata(pdev, ts_dev);
	return 0;

err_fail:
	pm_runtime_disable(&pdev->dev);
err_free_irq:
	free_irq(ts_dev->irq, ts_dev);
err_unmap_regs:
	iounmap(ts_dev->tsc_base);
err_release_mem:
	release_mem_region(res->start, resource_size(res));
	input_free_device(ts_dev->input);
err_free_mem:
	kfree(ts_dev);
	return err;
}

static int __devexit tscadc_remove(struct platform_device *pdev)
{
	struct tscadc		*ts_dev = platform_get_drvdata(pdev);
	struct resource		*res;

	free_irq(ts_dev->irq, ts_dev);

	input_unregister_device(ts_dev->input);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(ts_dev->tsc_base);
	release_mem_region(res->start, resource_size(res));

	pm_runtime_disable(&pdev->dev);

	kfree(ts_dev);

	device_init_wakeup(&pdev->dev, 0);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static int tscadc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tscadc *ts_dev = platform_get_drvdata(pdev);
	unsigned int idle;

	if (device_may_wakeup(&pdev->dev)) {
		idle = tscadc_readl(ts_dev, TSCADC_REG_IRQENABLE);
		tscadc_writel(ts_dev, TSCADC_REG_IRQENABLE,
				(idle | TSCADC_IRQENB_HW_PEN));
		tscadc_writel(ts_dev, TSCADC_REG_IRQWAKEUP, TSCADC_IRQWKUP_ENB);
	}

	/* module disable */
	idle = 0;
	idle = tscadc_readl(ts_dev, TSCADC_REG_CTRL);
	idle &= ~(TSCADC_CNTRLREG_TSCSSENB);
	tscadc_writel(ts_dev, TSCADC_REG_CTRL, idle);

	pm_runtime_put_sync(&pdev->dev);

	return 0;

}

static int tscadc_resume(struct platform_device *pdev)
{
	struct tscadc *ts_dev = platform_get_drvdata(pdev);
	unsigned int restore;

	pm_runtime_get_sync(&pdev->dev);

	if (device_may_wakeup(&pdev->dev)) {
		tscadc_writel(ts_dev, TSCADC_REG_IRQWAKEUP,
				TSCADC_IRQWKUP_DISABLE);
		tscadc_writel(ts_dev, TSCADC_REG_IRQCLR, TSCADC_IRQENB_HW_PEN);
	}

	/* context restore */
	tscadc_writel(ts_dev, TSCADC_REG_CTRL, ts_dev->ctrl);
	tsc_idle_config(ts_dev);
	tsc_step_config(ts_dev);
	tscadc_writel(ts_dev, TSCADC_REG_FIFO1THR, 6);
	restore = tscadc_readl(ts_dev, TSCADC_REG_CTRL);
	tscadc_writel(ts_dev, TSCADC_REG_CTRL,
			(restore | TSCADC_CNTRLREG_TSCSSENB));

	return 0;
}

static struct platform_driver ti_tsc_driver = {
	.probe	  = tscadc_probe,
	.remove	 = __devexit_p(tscadc_remove),
	.driver	 = {
		.name   = "tsc",
		.owner  = THIS_MODULE,
	},
	.suspend = tscadc_suspend,
	.resume  = tscadc_resume,
};

static int __init ti_tsc_init(void)
{
	return platform_driver_register(&ti_tsc_driver);
}
module_init(ti_tsc_init);

static void __exit ti_tsc_exit(void)
{
	platform_driver_unregister(&ti_tsc_driver);
}
module_exit(ti_tsc_exit);

MODULE_DESCRIPTION("TI touchscreen controller driver");
MODULE_AUTHOR("Rachna Patil <rachna@ti.com>");
MODULE_LICENSE("GPL");
