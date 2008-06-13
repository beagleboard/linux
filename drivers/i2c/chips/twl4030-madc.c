/*
 * drivers/i2c/chips/twl4030-madc.c
 *
 * TWL4030 MADC module driver
 *
 * Copyright (C) 2008 Nokia Corporation
 * Mikko Ylinen <mikko.k.ylinen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/i2c/twl4030.h>
#include <linux/i2c/twl4030-madc.h>

#include <asm/uaccess.h>

#define TWL4030_MADC_PFX	"twl4030-madc: "

static struct twl4030_madc_data {
	struct mutex		lock;
	struct work_struct	ws;
	struct twl4030_madc_request	requests[TWL4030_MADC_NUM_METHODS];
} twl4030_madc;

static const char irq_pin = 1; /* XXX Read from platfrom data */

static
const struct twl4030_madc_conversion_method twl4030_conversion_methods[] = {
	[TWL4030_MADC_RT] = {
		.sel	= TWL4030_MADC_RTSELECT_LSB,
		.avg	= TWL4030_MADC_RTAVERAGE_LSB,
		.rbase	= TWL4030_MADC_RTCH0_LSB,
	},
	[TWL4030_MADC_SW1] = {
		.sel	= TWL4030_MADC_SW1SELECT_LSB,
		.avg	= TWL4030_MADC_SW1AVERAGE_LSB,
		.rbase	= TWL4030_MADC_GPCH0_LSB,
		.ctrl	= TWL4030_MADC_CTRL_SW1,
	},
	[TWL4030_MADC_SW2] = {
		.sel	= TWL4030_MADC_SW2SELECT_LSB,
		.avg	= TWL4030_MADC_SW2AVERAGE_LSB,
		.rbase	= TWL4030_MADC_GPCH0_LSB,
		.ctrl	= TWL4030_MADC_CTRL_SW2,
	},
};

static void twl4030_madc_read(u8 reg, u8 *val)
{
	int ret = twl4030_i2c_read_u8(TWL4030_MODULE_MADC, val, reg);
	if (ret)
		printk(KERN_ERR TWL4030_MADC_PFX
		       "unable to read register 0x%X\n", reg);
}

static void twl4030_madc_write(u8 reg, u8 val)
{
	int ret = twl4030_i2c_write_u8(TWL4030_MODULE_MADC, val, reg);
	if (ret)
		printk(KERN_ERR TWL4030_MADC_PFX
		       "unable to write register 0x%X\n", reg);
}

static int twl4030_madc_channel_raw_read(u8 reg)
{
	u8 msb, lsb;

	/* For each ADC channel, we have MSB and LSB register pair. MSB address
	 * is always LSB address+1. reg parameter is the addr of LSB register */
	twl4030_madc_read(reg+1, &msb);
	twl4030_madc_read(reg, &lsb);

	return (int)(((msb << 8) | lsb) >> 6);
}

static int twl4030_madc_read_channels(u8 reg_base, u16 channels, int *buf)
{
	int count = 0;
	u8 reg, i;

	if (unlikely(!buf))
		return 0;

	for (i = 0; i < TWL4030_MADC_MAX_CHANNELS; i++) {
		if (channels & (1<<i)) {
			reg = reg_base + 2*i;
			buf[i] = twl4030_madc_channel_raw_read(reg);
			count++;
		}
	}
	return count;
}

static void twl4030_madc_enable_irq(int id)
{
	u8 val;

	static u8 imr = (irq_pin == 1) ? TWL4030_MADC_IMR1 : TWL4030_MADC_IMR2;

	twl4030_madc_read(imr, &val);
	val &= ~(1 << id);
	twl4030_madc_write(imr, val);
}

static void twl4030_madc_disable_irq(int id)
{
	u8 val;

	static u8 imr = (irq_pin == 1) ? TWL4030_MADC_IMR1 : TWL4030_MADC_IMR2;

	twl4030_madc_read(imr, &val);
	val |= (1 << id);
	twl4030_madc_write(imr, val);
}

static irqreturn_t twl4030_madc_irq_handler(int irq, void *madc_dev)
{
	u8 isr_val, imr_val;
	static u8 isr, imr;
	int i;

	imr = (irq_pin == 1) ? TWL4030_MADC_IMR1 : TWL4030_MADC_IMR2;
	isr = (irq_pin == 1) ? TWL4030_MADC_ISR1 : TWL4030_MADC_ISR2;

	/* Use COR to ack interrupts since we have no shared IRQs in ISRx */
	twl4030_madc_read(isr, &isr_val);
	twl4030_madc_read(imr, &imr_val);

	isr_val &= ~imr_val;

	for (i = 0; i < TWL4030_MADC_NUM_METHODS; i++) {

		if (!(isr_val & (1<<i)))
			continue;

		twl4030_madc_disable_irq(i);
		twl4030_madc.requests[i].result_pending = 1;
	}

	schedule_work(&twl4030_madc.ws);

	return IRQ_HANDLED;
}

static void twl4030_madc_work(struct work_struct *ws)
{
	const struct twl4030_madc_conversion_method *method;
	struct twl4030_madc_request *r;
	int len, i;

	mutex_lock(&twl4030_madc.lock);

	for (i = 0; i < TWL4030_MADC_NUM_METHODS; i++) {

		r = &twl4030_madc.requests[i];

		/* No pending results for this method, move to next one */
		if (!r->result_pending)
			continue;

		method = &twl4030_conversion_methods[r->method];

		/* Read results */
		len = twl4030_madc_read_channels(method->rbase,
						 r->channels, r->rbuf);

		/* Return results to caller */
		if (r->func_cb != NULL) {
			r->func_cb(len, r->channels, r->rbuf);
			r->func_cb = NULL;
		}

		/* Free request */
		r->result_pending = 0;
		r->active	  = 0;
	}

	mutex_unlock(&twl4030_madc.lock);
}

static int twl4030_madc_set_irq(struct twl4030_madc_request *req)
{
	struct twl4030_madc_request *p;

	p = &twl4030_madc.requests[req->method];

	memcpy(p, req, sizeof *req);

	twl4030_madc_enable_irq(req->method);

	return 0;
}

static inline void twl4030_madc_start_conversion(int conv_method)
{
	const struct twl4030_madc_conversion_method *method;

	method = &twl4030_conversion_methods[conv_method];

	switch (conv_method) {
	case TWL4030_MADC_SW1:
	case TWL4030_MADC_SW2:
		twl4030_madc_write(method->ctrl, TWL4030_MADC_SW_START);
		break;
	case TWL4030_MADC_RT:
	default:
		break;
	}
}

static void twl4030_madc_wait_conversion_ready_ms(u8 *time, u8 status_reg)
{
	u8 reg = 0;

	do {
		msleep(1);
		(*time)--;
		twl4030_madc_read(status_reg, &reg);
	} while (((reg & TWL4030_MADC_BUSY) && !(reg & TWL4030_MADC_EOC_SW)) &&
		  (*time != 0));
}

int twl4030_madc_conversion(struct twl4030_madc_request *req)
{
	const struct twl4030_madc_conversion_method *method;
	u8 wait_time, ch_msb, ch_lsb;
	int ret;

	if (unlikely(!req))
		return -EINVAL;

	/* Do we have a conversion request ongoing */
	if (twl4030_madc.requests[req->method].active)
		return -EBUSY;

	ch_msb = (req->channels >> 8) & 0xff;
	ch_lsb = req->channels & 0xff;

	method = &twl4030_conversion_methods[req->method];

	mutex_lock(&twl4030_madc.lock);

	/* Select channels to be converted */
	twl4030_madc_write(method->sel + 1, ch_msb);
	twl4030_madc_write(method->sel, ch_lsb);

	/* Select averaging for all channels if do_avg is set */
	if (req->do_avg) {
		twl4030_madc_write(method->avg + 1, ch_msb);
		twl4030_madc_write(method->avg, ch_lsb);
	}

	if ((req->type == TWL4030_MADC_IRQ_ONESHOT) && (req->func_cb != NULL)) {
		twl4030_madc_set_irq(req);
		twl4030_madc_start_conversion(req->method);
		twl4030_madc.requests[req->method].active = 1;
		ret = 0;
		goto out;
	}

	/* With RT method we should not be here anymore */
	if (req->method == TWL4030_MADC_RT) {
		ret = -EINVAL;
		goto out;
	}

	twl4030_madc_start_conversion(req->method);
	twl4030_madc.requests[req->method].active = 1;

	/* Wait until conversion is ready (ctrl register returns EOC) */
	wait_time = 50;
	twl4030_madc_wait_conversion_ready_ms(&wait_time, method->ctrl);
	if (wait_time == 0) {
		printk(KERN_ERR TWL4030_MADC_PFX "conversion timeout!\n");
		ret = -EAGAIN;
		goto out;
	}

	ret = twl4030_madc_read_channels(method->rbase, req->channels,
					 req->rbuf);

	twl4030_madc.requests[req->method].active = 0;

out:
	mutex_unlock(&twl4030_madc.lock);

	return ret;
}

EXPORT_SYMBOL(twl4030_madc_conversion);

static int twl4030_madc_set_current_generator(int chan, int on)
{
	int ret;
	u8 regval;

	/* Current generator is only available for ADCIN0 and ADCIN1. NB:
	 * ADCIN1 current generator only works when AC or VBUS is present */
	if (chan > 1)
		return EINVAL;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE,
				  &regval, TWL4030_BCI_BCICTL1);
	if (on)
		regval |= (chan) ? TWL4030_BCI_ITHEN : TWL4030_BCI_TYPEN;
	else
		regval &= (chan) ? ~TWL4030_BCI_ITHEN : ~TWL4030_BCI_TYPEN;
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_MAIN_CHARGE,
				   regval, TWL4030_BCI_BCICTL1);

	return ret;
}

static int twl4030_madc_set_power(int on)
{
	u8 regval;

	twl4030_madc_read(TWL4030_MADC_CTRL1, &regval);
	if (on)
		regval |= TWL4030_MADC_MADCON;
	else
		regval &= ~TWL4030_MADC_MADCON;
	twl4030_madc_write(TWL4030_MADC_CTRL1, regval);

	return 0;
}

static int twl4030_madc_ioctl(struct inode *inode, struct file *filp,
			      unsigned int cmd, unsigned long arg)
{
	struct twl4030_madc_user_parms par;
	int val, ret;

	ret = copy_from_user(&par, (void __user *) arg, sizeof(par));
	if (ret) {
		printk(KERN_ERR TWL4030_MADC_PFX "copy_from_user: %d\n", ret);
		return -EACCES;
	}

	switch (cmd) {
	case TWL4030_MADC_IOCX_ADC_RAW_READ: {
		struct twl4030_madc_request req;
		if (par.channel >= TWL4030_MADC_MAX_CHANNELS)
			return -EINVAL;

		req.channels = (1<<par.channel);
		req.do_avg	= par.average;
		req.method	= TWL4030_MADC_SW1;

		val = twl4030_madc_conversion(&req);
		if (val <= 0) {
			par.status = -1;
		} else {
			par.status = 0;
			par.result = (u16)req.rbuf[par.channel];
		}
		break;
					     }
	default:
		return -EINVAL;
	}

	ret = copy_to_user((void __user *) arg, &par, sizeof(par));
	if (ret) {
		printk(KERN_ERR TWL4030_MADC_PFX "copy_to_user: %d\n", ret);
		return -EACCES;
	}

	return 0;
}

static struct file_operations twl4030_madc_fileops = {
	.owner = THIS_MODULE,
	.ioctl = twl4030_madc_ioctl
};

static struct miscdevice twl4030_madc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "twl4030-adc",
	.fops = &twl4030_madc_fileops
};

static int __init twl4030_madc_init(void)
{
	int ret;
	u8 regval;

	ret = misc_register(&twl4030_madc_device);
	if (ret == -1) {
		printk(KERN_ERR TWL4030_MADC_PFX "misc_register() failed!\n");
		return ret;
	}
	twl4030_madc_set_power(1);
	twl4030_madc_set_current_generator(0, 1);

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE,
				  &regval, TWL4030_BCI_BCICTL1);

	regval |= TWL4030_BCI_MESBAT;

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_MAIN_CHARGE,
				   regval, TWL4030_BCI_BCICTL1);

	ret = request_irq(TWL4030_MODIRQ_MADC, twl4030_madc_irq_handler,
			  IRQF_DISABLED, "twl4030_madc", &twl4030_madc);
	if (ret)
		printk(KERN_ERR TWL4030_MADC_PFX "request_irq: %d\n", ret);

	mutex_init(&twl4030_madc.lock);

	INIT_WORK(&twl4030_madc.ws, twl4030_madc_work);

	printk(KERN_INFO TWL4030_MADC_PFX "initialised\n");

	return ret;
}

static void __exit twl4030_madc_exit(void)
{
	twl4030_madc_set_power(0);
	twl4030_madc_set_current_generator(0, 0);
	free_irq(TWL4030_MODIRQ_MADC, &twl4030_madc);
	cancel_work_sync(&twl4030_madc.ws);
	misc_deregister(&twl4030_madc_device);
}

module_init(twl4030_madc_init);
module_exit(twl4030_madc_exit);

MODULE_ALIAS("i2c:twl4030-adc");
MODULE_AUTHOR("Nokia Corporation");
MODULE_DESCRIPTION("twl4030 ADC driver");
MODULE_LICENSE("GPL");
