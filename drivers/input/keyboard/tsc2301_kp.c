/*
 * TSC2301 keypad driver
 *
 * Copyright (C) 2005-2006 Nokia Corporation
 *
 * Written by Jarkko Oikarinen
 * Rewritten by Juha Yrjola <juha.yrjola@nokia.com>
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
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>

#include <linux/spi/tsc2301.h>

#define TSC2301_KEYBOARD_PRODUCT_ID      0x0051
#define TSC2301_KEYBOARD_PRODUCT_VERSION 0x0001
#define TSC2301_DEBOUNCE_TIME_2MS        0x0000
#define TSC2301_DEBOUNCE_TIME_10MS       0x0800
#define TSC2301_DEBOUNCE_TIME_20MS       0x1000
#define TSC2301_DEBOUNCE_TIME_50MS       0x1800
#define TSC2301_DEBOUNCE_TIME_60MS       0x2000
#define TSC2301_DEBOUNCE_TIME_80MS       0x2800
#define TSC2301_DEBOUNCE_TIME_100MS      0x3000
#define TSC2301_DEBOUNCE_TIME_120MS      0x3800

#define TSC2301_DEBOUNCE_TIME		TSC2301_DEBOUNCE_TIME_20MS

#define TSC2301_RELEASE_TIMEOUT		50

struct tsc2301_kp {
	struct input_dev	*idev;
	char			phys[32];
	spinlock_t		lock;
	struct mutex		mutex;
	struct timer_list	timer;
	u16			keys_pressed;
	unsigned		pending:1;
	unsigned		user_disabled:1;
	unsigned		disable_depth;

	struct spi_transfer	read_xfer[4];
	struct spi_message	read_msg;

	u16			data;
	u16			mask;

	int			irq;
	s16			keymap[16];
};

static inline int tsc2301_kp_disabled(struct tsc2301 *tsc)
{
	return tsc->kp->disable_depth != 0;
}

static void tsc2301_kp_send_key_events(struct tsc2301 *tsc,
				       u16 prev_state,
				       u16 new_state)
{
	struct tsc2301_kp *kp = tsc->kp;
	u16 common, released, pressed;
	int i;

	common = prev_state & new_state;
	released = common ^ prev_state;
	pressed = common ^ new_state;
	if (!released && !pressed)
		return;
	for (i = 0; i < 16 && (released || pressed); i++) {
		if (released & 1) {
			dev_dbg(&tsc->spi->dev, "key %d released\n", i);
			input_report_key(kp->idev, kp->keymap[i], 0);
		}
		released >>= 1;
		if (pressed & 1) {
			dev_dbg(&tsc->spi->dev, "key %d pressed\n", i);
			input_report_key(kp->idev, kp->keymap[i], 1);
		}
		pressed >>= 1;
	}
	input_sync(kp->idev);
}

static inline void _filter_out(struct tsc2301 *tsc, u16 prev_state,
			       u16 *new_state, int row1, int row2, u8 rect_pat)
{
	u16 mask;

	mask = (rect_pat << (row1 * 4)) | (rect_pat << (row2 * 4));
	mask &= ~prev_state;
	*new_state &= ~mask;
	dev_dbg(&tsc->spi->dev, "filtering ghost keys %02x\n", mask);
}

static void tsc2301_filter_ghost_keys(struct tsc2301 *tsc, u16 prev_state,
				      u16 *new_state)
{
	int row1, row2;
	u16 key_map;
	u16 row1_map;
	static const u8 rect_pat[] = {
		0x3, 0x5, 0x9, 0x6, 0xa, 0xc, 0,
	};

	key_map = *new_state;
	for (row1 = 0; row1 < 4; row1++) {
		row1_map = (key_map >> (row1 * 4)) & 0xf;
		if (!row1_map)
			continue;
		for (row2 = row1 + 1; row2 < 4; row2++) {
			u16 rect_map = (key_map >> (row2 * 4)) & 0xf;
			const u8 *rp;

			rect_map &= row1_map;
			if (!rect_map)
				continue;
			for (rp = rect_pat; *rp; rp++)
				if ((rect_map & *rp) == *rp)
					_filter_out(tsc, prev_state, new_state,
						    row1, row2, *rp);
		}
	}
}

static void tsc2301_kp_timer(unsigned long arg)
{
	struct tsc2301 *tsc = (void *) arg;
	struct tsc2301_kp *kp = tsc->kp;
	unsigned long flags;

	tsc2301_kp_send_key_events(tsc, kp->keys_pressed, 0);
	spin_lock_irqsave(&kp->lock, flags);
	kp->keys_pressed = 0;
	spin_unlock_irqrestore(&kp->lock, flags);
}

static void tsc2301_kp_rx(void *arg)
{
	struct tsc2301 *tsc = arg;
	struct tsc2301_kp *kp = tsc->kp;
	unsigned long flags;
	u16 kp_data;

	kp_data = kp->data;
	dev_dbg(&tsc->spi->dev, "KP data %04x\n", kp_data);

	tsc2301_filter_ghost_keys(tsc, kp->keys_pressed, &kp_data);
	tsc2301_kp_send_key_events(tsc, kp->keys_pressed, kp_data);
	spin_lock_irqsave(&kp->lock, flags);
	kp->keys_pressed = kp_data;
	kp->pending = 0;
	spin_unlock_irqrestore(&kp->lock, flags);
}

static irqreturn_t tsc2301_kp_irq_handler(int irq, void *dev_id)
{
	struct tsc2301 *tsc = dev_id;
	struct tsc2301_kp *kp = tsc->kp;
	unsigned long flags;
	int r;

	spin_lock_irqsave(&kp->lock, flags);
	if (tsc2301_kp_disabled(tsc)) {
		spin_unlock_irqrestore(&kp->lock, flags);
		return IRQ_HANDLED;
	}
	kp->pending = 1;
	spin_unlock_irqrestore(&kp->lock, flags);
	mod_timer(&kp->timer,
		 jiffies + msecs_to_jiffies(TSC2301_RELEASE_TIMEOUT));
	r = spi_async(tsc->spi, &tsc->kp->read_msg);
	if (r)
		dev_err(&tsc->spi->dev, "kp: spi_async() failed");
	return IRQ_HANDLED;
}

static void tsc2301_kp_start_scan(struct tsc2301 *tsc)
{
	tsc2301_write_reg(tsc, TSC2301_REG_KPMASK, tsc->kp->mask);
	tsc2301_write_reg(tsc, TSC2301_REG_KEY, TSC2301_DEBOUNCE_TIME);
}

static void tsc2301_kp_stop_scan(struct tsc2301 *tsc)
{
	tsc2301_write_reg(tsc, TSC2301_REG_KEY, 1 << 14);
}

/* Must be called with the mutex held */
static void tsc2301_kp_enable(struct tsc2301 *tsc)
{
	struct tsc2301_kp *kp = tsc->kp;
	unsigned long flags;

	spin_lock_irqsave(&kp->lock, flags);
	BUG_ON(!tsc2301_kp_disabled(tsc));
	if (--kp->disable_depth != 0) {
		spin_unlock_irqrestore(&kp->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&kp->lock, flags);

	set_irq_type(kp->irq, IRQ_TYPE_EDGE_FALLING);
	tsc2301_kp_start_scan(tsc);
	enable_irq(kp->irq);
}

/* Must be called with the mutex held */
static int tsc2301_kp_disable(struct tsc2301 *tsc, int release_keys)
{
	struct tsc2301_kp *kp = tsc->kp;
	unsigned long flags;

	spin_lock_irqsave(&kp->lock, flags);
	if (kp->disable_depth++ != 0) {
		spin_unlock_irqrestore(&kp->lock, flags);
		goto out;
	}
	disable_irq_nosync(kp->irq);
	set_irq_type(kp->irq, IRQ_TYPE_NONE);
	spin_unlock_irqrestore(&kp->lock, flags);

	while (kp->pending) {
		msleep(1);
	}

	tsc2301_kp_stop_scan(tsc);
out:
	if (!release_keys)
		del_timer(&kp->timer); /* let timeout release keys */

	return 0;
}

/* The following workaround is needed for a HW bug triggered by the
 * following:
 * 1. keep any key pressed
 * 2. disable keypad
 * 3. release all keys
 * 4. reenable keypad
 * 5. disable touch screen controller
 *
 * After this the keypad scanner will get stuck in busy state and won't
 * report any interrupts for further keypresses. One way to recover is to
 * restart the keypad scanner whenever we enable / disable the
 * touchscreen controller.
 */
void tsc2301_kp_restart(struct tsc2301 *tsc)
{
	if (!tsc2301_kp_disabled(tsc)) {
		tsc2301_kp_start_scan(tsc);
	}
}

static ssize_t tsc2301_kp_disable_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct tsc2301		*tsc = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", tsc2301_kp_disabled(tsc) ? 1 : 0);
}

static ssize_t tsc2301_kp_disable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct tsc2301		*tsc = dev_get_drvdata(dev);
	struct tsc2301_kp	*kp = tsc->kp;
	char *endp;
	int i;

	i = simple_strtoul(buf, &endp, 10);
	i = i ? 1 : 0;

	mutex_lock(&kp->mutex);
	if (i == kp->user_disabled) {
		mutex_unlock(&kp->mutex);
		return count;
	}
	kp->user_disabled = i;

	if (i)
		tsc2301_kp_disable(tsc, 1);
	else
		tsc2301_kp_enable(tsc);
	mutex_unlock(&kp->mutex);

	return count;
}

static DEVICE_ATTR(disable_kp, 0664, tsc2301_kp_disable_show,
		   tsc2301_kp_disable_store);

static const u16 tsc2301_kp_read_data = 0x8000 | TSC2301_REG_KPDATA;

static void tsc2301_kp_setup_spi_xfer(struct tsc2301 *tsc)
{
	struct tsc2301_kp *kp = tsc->kp;
	struct spi_message *m = &kp->read_msg;
	struct spi_transfer *x = &kp->read_xfer[0];

	spi_message_init(&kp->read_msg);

	x->tx_buf = &tsc2301_kp_read_data;
	x->len = 2;
	spi_message_add_tail(x, m);
	x++;

	x->rx_buf = &kp->data;
	x->len = 2;
	spi_message_add_tail(x, m);

	m->complete = tsc2301_kp_rx;
	m->context = tsc;
}

#ifdef CONFIG_PM
int tsc2301_kp_suspend(struct tsc2301 *tsc)
{
	struct tsc2301_kp *kp = tsc->kp;

	mutex_lock(&kp->mutex);
	tsc2301_kp_disable(tsc, 1);
	mutex_unlock(&kp->mutex);
	return 0;
}

void tsc2301_kp_resume(struct tsc2301 *tsc)
{
	struct tsc2301_kp *kp = tsc->kp;

	mutex_lock(&kp->mutex);
	tsc2301_kp_enable(tsc);
	mutex_unlock(&kp->mutex);
}
#endif

int __devinit tsc2301_kp_init(struct tsc2301 *tsc,
			      struct tsc2301_platform_data *pdata)
{
	struct input_dev *idev;
	struct tsc2301_kp *kp;
	int r, i;
	u16 mask;

	if (pdata->keyb_int < 0) {
		dev_err(&tsc->spi->dev, "need kbirq");
		return -EINVAL;
	}

	kp = kzalloc(sizeof(*kp), GFP_KERNEL);
	if (kp == NULL)
		return -ENOMEM;
	tsc->kp = kp;

	kp->irq = pdata->keyb_int;
	spin_lock_init(&kp->lock);
	mutex_init(&kp->mutex);

	init_timer(&kp->timer);
	kp->timer.data = (unsigned long) tsc;
	kp->timer.function = tsc2301_kp_timer;

	idev = input_allocate_device();
	if (idev == NULL) {
		r = -ENOMEM;
		goto err1;
	}
	if (pdata->keyb_name)
		idev->name = pdata->keyb_name;
	else
		idev->name = "TSC2301 keypad";
	snprintf(kp->phys, sizeof(kp->phys), "%s/input-kp", tsc->spi->dev.bus_id);
	idev->phys = kp->phys;

	mask = 0;
	idev->evbit[0] = BIT(EV_KEY);
	for (i = 0; i < 16; i++) {
		if (pdata->keymap[i] > 0) {
			set_bit(pdata->keymap[i], idev->keybit);
			kp->keymap[i] = pdata->keymap[i];
		} else {
			kp->keymap[i] = -1;
			mask |= 1 << i;
		}
	}

	if (pdata->kp_rep)
		set_bit(EV_REP, idev->evbit);

	kp->idev = idev;

	tsc2301_kp_setup_spi_xfer(tsc);

	r = device_create_file(&tsc->spi->dev, &dev_attr_disable_kp);
	if (r < 0)
		goto err2;

	tsc2301_kp_start_scan(tsc);

	/* IRQ mode 0 is faulty, it can cause the KBIRQ to get stuck.
	 * Mode 2 deasserts the IRQ at:
	 * - HW or SW reset
	 * - Setting SCS flag in REG_KEY register
	 * - Releasing all keys
	 * - Reading the REG_KPDATA
	 */
	tsc2301_write_kbc(tsc, 2);

	tsc2301_write_reg(tsc, TSC2301_REG_KPMASK, mask);
	kp->mask = mask;

	set_irq_type(kp->irq, IRQ_TYPE_EDGE_FALLING);

	r = request_irq(kp->irq, tsc2301_kp_irq_handler, IRQF_SAMPLE_RANDOM,
			"tsc2301-kp", tsc);
	if (r < 0) {
		dev_err(&tsc->spi->dev, "unable to get kbirq IRQ");
		goto err3;
	}
	set_irq_wake(kp->irq, 1);

	/* We need to read the register once..? */
	tsc2301_read_reg(tsc, TSC2301_REG_KPDATA);

	r = input_register_device(idev);
	if (r < 0) {
		dev_err(&tsc->spi->dev, "can't register keypad device\n");
		goto err4;
	}

	return 0;

err4:
	free_irq(kp->irq, tsc);
err3:
	tsc2301_kp_stop_scan(tsc);
	device_remove_file(&tsc->spi->dev, &dev_attr_disable_kp);
err2:
	input_free_device(kp->idev);
err1:
	kfree(kp);
	return r;
}

void __devexit tsc2301_kp_exit(struct tsc2301 *tsc)
{
	struct tsc2301_kp *kp = tsc->kp;

	tsc2301_kp_disable(tsc, 1);
	input_unregister_device(kp->idev);
	free_irq(kp->irq, tsc);
	device_remove_file(&tsc->spi->dev, &dev_attr_disable_kp);

	kfree(kp);
}
