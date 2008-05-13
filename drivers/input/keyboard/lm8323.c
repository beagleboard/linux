/*
 * drivers/i2c/chips/lm8323.c
 *
 * Copyright (C) 2007 Nokia Corporation
 *
 * Written by Daniel Stone <daniel.stone@nokia.com>
 *            Timo O. Karjalainen <timo.o.karjalainen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation (version 2 of the License only).
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/i2c/lm8323.h>

#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#ifdef VERBOSE
#define debug dev_dbg
#else
#define debug(...)
#endif

/* Commands to send to the chip. */
#define LM8323_CMD_READ_ID		0x80 /* Read chip ID. */
#define LM8323_CMD_WRITE_CFG		0x81 /* Set configuration item. */
#define LM8323_CMD_READ_INT		0x82 /* Get interrupt status. */
#define LM8323_CMD_RESET		0x83 /* Reset, same as external one */
#define LM8323_CMD_WRITE_PORT_SEL	0x85 /* Set GPIO in/out. */
#define LM8323_CMD_WRITE_PORT_STATE	0x86 /* Set GPIO pullup. */
#define LM8323_CMD_READ_PORT_SEL	0x87 /* Get GPIO in/out. */
#define LM8323_CMD_READ_PORT_STATE	0x88 /* Get GPIO pullup. */
#define LM8323_CMD_READ_FIFO		0x89 /* Read byte from FIFO. */
#define LM8323_CMD_RPT_READ_FIFO	0x8a /* Read FIFO (no increment). */
#define LM8323_CMD_SET_ACTIVE		0x8b /* Set active time. */
#define LM8323_CMD_READ_ERR		0x8c /* Get error status. */
#define LM8323_CMD_READ_ROTATOR		0x8e /* Read rotator status. */
#define LM8323_CMD_SET_DEBOUNCE		0x8f /* Set debouncing time. */
#define LM8323_CMD_SET_KEY_SIZE		0x90 /* Set keypad size. */
#define LM8323_CMD_READ_KEY_SIZE	0x91 /* Get keypad size. */
#define LM8323_CMD_READ_CFG		0x92 /* Get configuration item. */
#define LM8323_CMD_WRITE_CLOCK		0x93 /* Set clock config. */
#define LM8323_CMD_READ_CLOCK		0x94 /* Get clock config. */
#define LM8323_CMD_PWM_WRITE		0x95 /* Write PWM script. */
#define LM8323_CMD_START_PWM		0x96 /* Start PWM engine. */
#define LM8323_CMD_STOP_PWM		0x97 /* Stop PWM engine. */

/* Interrupt status. */
#define INT_KEYPAD			0x01 /* Key event. */
#define INT_ROTATOR			0x02 /* Rotator event. */
#define INT_ERROR			0x08 /* Error: use CMD_READ_ERR. */
#define INT_NOINIT			0x10 /* Lost configuration. */
#define INT_PWM1			0x20 /* PWM1 stopped. */
#define INT_PWM2			0x40 /* PWM2 stopped. */
#define INT_PWM3			0x80 /* PWM3 stopped. */

/* Errors (signalled by INT_ERROR, read with CMD_READ_ERR). */
#define ERR_BADPAR			0x01 /* Bad parameter. */
#define ERR_CMDUNK			0x02 /* Unknown command. */
#define ERR_KEYOVR			0x04 /* Too many keys pressed. */
#define ERR_FIFOOVER			0x40 /* FIFO overflow. */

/* Configuration keys (CMD_{WRITE,READ}_CFG). */
#define CFG_MUX1SEL			0x01 /* Select MUX1_OUT input. */
#define CFG_MUX1EN			0x02 /* Enable MUX1_OUT. */
#define CFG_MUX2SEL			0x04 /* Select MUX2_OUT input. */
#define CFG_MUX2EN			0x08 /* Enable MUX2_OUT. */
#define CFG_PSIZE			0x20 /* Package size (must be 0). */
#define CFG_ROTEN			0x40 /* Enable rotator. */

/* Clock settings (CMD_{WRITE,READ}_CLOCK). */
#define CLK_RCPWM_INTERNAL		0x00
#define CLK_RCPWM_EXTERNAL		0x03
#define CLK_SLOWCLKEN			0x08 /* Enable 32.768kHz clock. */
#define CLK_SLOWCLKOUT			0x40 /* Enable slow pulse output. */

/* The possible addresses corresponding to CONFIG1 and CONFIG2 pin wirings. */
#define LM8323_I2C_ADDR00		(0x84 >> 1)	/* 1000 010x */
#define LM8323_I2C_ADDR01		(0x86 >> 1)	/* 1000 011x */
#define LM8323_I2C_ADDR10		(0x88 >> 1)	/* 1000 100x */
#define LM8323_I2C_ADDR11		(0x8A >> 1)	/* 1000 101x */

/* Key event fifo length */
#define LM8323_FIFO_LEN			15

/* Commands for PWM engine; feed in with PWM_WRITE. */
/* Load ramp counter from duty cycle field (range 0 - 0xff). */
#define PWM_SET(v)			(0x4000 | ((v) & 0xff))
/* Go to start of script. */
#define PWM_GOTOSTART			0x0000
/*
 * Stop engine (generates interrupt).  If reset is 1, clear the program
 * counter, else leave it.
 */
#define PWM_END(reset)			(0xc000 | (!!(reset) << 11))
/*
 * Ramp.  If s is 1, divide clock by 512, else divide clock by 16.
 * Take t clock scales (up to 63) per step, for n steps (up to 126).
 * If u is set, ramp up, else ramp down.
 */
#define PWM_RAMP(s, t, n, u)		((!!(s) << 14) | ((t) & 0x3f) << 8 | \
					 ((n) & 0x7f) | ((u) ? 0 : 0x80))
/*
 * Loop (i.e. jump back to pos) for a given number of iterations (up to 63).
 * If cnt is zero, execute until PWM_END is encountered.
 */
#define PWM_LOOP(cnt, pos)		(0xa000 | (((cnt) & 0x3f) << 7) | \
					 ((pos) & 0x3f))
/*
 * Wait for trigger.  Argument is a mask of channels, shifted by the channel
 * number, e.g. 0xa for channels 3 and 1.  Note that channels are numbered
 * from 1, not 0.
 */
#define PWM_WAIT_TRIG(chans)		(0xe000 | (((chans) & 0x7) << 6))
/* Send trigger.  Argument is same as PWM_WAIT_TRIG. */
#define PWM_SEND_TRIG(chans)		(0xe000 | ((chans) & 0x7))

#define DRIVER_NAME  "lm8323"

struct lm8323_pwm {
	int			id;
	int			enabled;
	int			fade_time;
	int			brightness;
	int			desired_brightness;
	struct work_struct	work;
	struct led_classdev	cdev;
};

struct lm8323_chip {
	struct mutex		lock;
	struct i2c_client	*client;
	struct work_struct	work;
	struct input_dev	*idev;
	int			irq;
	unsigned		kp_enabled : 1;
	unsigned		pm_suspend : 1;
	unsigned		keys_down;
	char			phys[32];
	s16			keymap[LM8323_KEYMAP_SIZE];
	int			size_x;
	int			size_y;
	int			debounce_time;
	int			active_time;
	struct lm8323_pwm	pwm1;
	struct lm8323_pwm	pwm2;
	struct lm8323_pwm	pwm3;
};

#define client_to_lm8323(c)	container_of(c, struct lm8323_chip, client)
#define dev_to_lm8323(d)	container_of(d, struct lm8323_chip, client->dev)
#define work_to_lm8323(w)	container_of(w, struct lm8323_chip, work)
#define cdev_to_pwm(c)		container_of(c, struct lm8323_pwm, cdev)
#define work_to_pwm(w)		container_of(w, struct lm8323_pwm, work)

static struct lm8323_chip *pwm_to_lm8323(struct lm8323_pwm *pwm)
{
	switch (pwm->id) {
	case 1:
		return container_of(pwm, struct lm8323_chip, pwm1);
	case 2:
		return container_of(pwm, struct lm8323_chip, pwm2);
	case 3:
		return container_of(pwm, struct lm8323_chip, pwm3);
	default:
		return NULL;
	}
}

static struct lm8323_platform_data *lm8323_pdata;


#define LM8323_MAX_DATA 8

/*
 * To write, we just access the chip's address in write mode, and dump the
 * command and data out on the bus.  The command byte and data are taken as
 * sequential u8s out of varargs, to a maximum of LM8323_MAX_DATA.
 */
static int lm8323_write(struct lm8323_chip *lm, int len, ...)
{
	int ret, i;
	va_list ap;
	u8 data[LM8323_MAX_DATA];

	va_start(ap, len);

	if (unlikely(len > LM8323_MAX_DATA)) {
		dev_err(&lm->client->dev, "tried to send %d bytes\n", len);
		va_end(ap);
		return 0;
	}

	for (i = 0; i < len; i++)
		data[i] = va_arg(ap, int);

	va_end(ap);

	/*
	 * If the host is asleep while we send the data, we can get a NACK
	 * back while it wakes up, so try again, once.
	 */
	ret = i2c_master_send(lm->client, data, len);
	if (unlikely(ret == -EREMOTEIO))
		ret = i2c_master_send(lm->client, data, len);
	if (unlikely(ret != len))
		dev_err(&lm->client->dev, "sent %d bytes of %d total\n",
			len, ret);

	return ret;
}

/*
 * To read, we first send the command byte to the chip and end the transaction,
 * then access the chip in read mode, at which point it will send the data.
 */
static int lm8323_read(struct lm8323_chip *lm, u8 cmd, u8 *buf, int len)
{
	int ret;

	/*
	 * If the host is asleep while we send the byte, we can get a NACK
	 * back while it wakes up, so try again, once.
	 */
	ret = i2c_master_send(lm->client, &cmd, 1);
	if (unlikely(ret == -EREMOTEIO))
		ret = i2c_master_send(lm->client, &cmd, 1);
	if (unlikely(ret != 1)) {
		dev_err(&lm->client->dev, "sending read cmd 0x%02x failed\n",
			cmd);
		return 0;
	}

	ret = i2c_master_recv(lm->client, buf, len);
	if (unlikely(ret != len))
		dev_err(&lm->client->dev, "wanted %d bytes, got %d\n",
			len, ret);

	return ret;
}

/*
 * Set the chip active time (idle time before it enters halt).
 */
static void lm8323_set_active_time(struct lm8323_chip *lm, int time)
{
	lm8323_write(lm, 2, LM8323_CMD_SET_ACTIVE, time >> 2);
}

/*
 * The signals are AT-style: the low 7 bits are the keycode, and the top
 * bit indicates the state (1 for down, 0 for up).
 */
static inline u8 lm8323_whichkey(u8 event)
{
	return event & 0x7f;
}

static inline int lm8323_ispress(u8 event)
{
	return (event & 0x80) ? 1 : 0;
}

static void process_keys(struct lm8323_chip *lm)
{
	u8 event;
	u8 key_fifo[LM8323_FIFO_LEN + 1];
	int old_keys_down = lm->keys_down;
	int ret;
	int i = 0;

	/*
	 * Read all key events from the FIFO at once. Next READ_FIFO clears the
	 * FIFO even if we didn't read all events previously.
	 */
	ret = lm8323_read(lm, LM8323_CMD_READ_FIFO, key_fifo, LM8323_FIFO_LEN);

	if (ret < 0) {
		dev_err(&lm->client->dev, "Failed reading fifo \n");
		return;
	}
	key_fifo[ret] = 0;

	while ((event = key_fifo[i])) {
		u8 key = lm8323_whichkey(event);
		int isdown = lm8323_ispress(event);
		s16 keycode = lm->keymap[key];

		if (likely(keycode > 0)) {
			debug(&lm->client->dev, "key 0x%02x %s\n", key,
			      isdown ? "down" : "up");
			if (likely(lm->kp_enabled)) {
				input_report_key(lm->idev, keycode, isdown);
				input_sync(lm->idev);
			}
			if (isdown)
				lm->keys_down++;
			else
				lm->keys_down--;
		} else {
			dev_err(&lm->client->dev, "keycode 0x%02x not mapped "
				"to any key\n", key);
		}
		i++;
	}

	/*
	 * Errata: We need to ensure that the chip never enters halt mode
	 * during a keypress, so set active time to 0.  When it's released,
	 * we can enter halt again, so set the active time back to normal.
	 */
	if (!old_keys_down && lm->keys_down)
		lm8323_set_active_time(lm, 0);
	if (old_keys_down && !lm->keys_down)
		lm8323_set_active_time(lm, lm->active_time);
}

static void lm8323_process_error(struct lm8323_chip *lm)
{
	u8 error;

	if (lm8323_read(lm, LM8323_CMD_READ_ERR, &error, 1) == 1) {
		if (error & ERR_FIFOOVER)
			debug(&lm->client->dev, "fifo overflow!\n");
		if (error & ERR_KEYOVR)
			debug(&lm->client->dev, "more than two keys pressed\n");
		if (error & ERR_CMDUNK)
			debug(&lm->client->dev, "unknown command submitted\n");
		if (error & ERR_BADPAR)
			debug(&lm->client->dev, "bad command parameter\n");
	}
}

static void lm8323_reset(struct lm8323_chip *lm)
{
	/* The docs say we must pass 0xAA as the data byte. */
	lm8323_write(lm, 2, LM8323_CMD_RESET, 0xAA);
}

static int lm8323_configure(struct lm8323_chip *lm)
{
	int keysize = (lm->size_x << 4) | lm->size_y;
	int clock = (CLK_SLOWCLKEN | CLK_RCPWM_EXTERNAL);
	int debounce = lm->debounce_time >> 2;
	int active = lm->active_time >> 2;

	/*
	 * Active time must be greater than the debounce time: if it's
	 * a close-run thing, give ourselves a 12ms buffer.
	 */
	if (debounce >= active)
		active = debounce + 3;

	lm8323_write(lm, 2, LM8323_CMD_WRITE_CFG, 0);
	lm8323_write(lm, 2, LM8323_CMD_WRITE_CLOCK, clock);
	lm8323_write(lm, 2, LM8323_CMD_SET_KEY_SIZE, keysize);
	lm8323_set_active_time(lm, lm->active_time);
	lm8323_write(lm, 2, LM8323_CMD_SET_DEBOUNCE, debounce);
	lm8323_write(lm, 3, LM8323_CMD_WRITE_PORT_STATE, 0xff, 0xff);
	lm8323_write(lm, 3, LM8323_CMD_WRITE_PORT_SEL, 0, 0);

	/*
	 * Not much we can do about errors at this point, so just hope
	 * for the best.
	 */

	return 0;
}

/*
 * Bottom half: handle the interrupt by posting key events, or dealing with
 * errors appropriately.
 */
static void lm8323_work(struct work_struct *work)
{
	struct lm8323_chip *lm = work_to_lm8323(work);
	u8 ints;

	mutex_lock(&lm->lock);

	while ((lm8323_read(lm, LM8323_CMD_READ_INT, &ints, 1) == 1) && ints) {
		if (likely(ints & INT_KEYPAD))
			process_keys(lm);
		if (ints & INT_ROTATOR) {
			/* We don't currently support the rotator. */
			debug(&lm->client->dev, "rotator fired\n");
		}
		if (ints & INT_ERROR) {
			debug(&lm->client->dev, "error!\n");
			lm8323_process_error(lm);
		}
		if (ints & INT_NOINIT) {
			dev_err(&lm->client->dev, "chip lost config; "
						  "reinitialising\n");
			lm8323_configure(lm);
		}
		if (ints & INT_PWM1)
			debug(&lm->client->dev, "pwm1 engine completed\n");
		if (ints & INT_PWM2)
			debug(&lm->client->dev, "pwm2 engine completed\n");
		if (ints & INT_PWM3)
			debug(&lm->client->dev, "pwm3 engine completed\n");
	}

	mutex_unlock(&lm->lock);
}

/*
 * We cannot use I2C in interrupt context, so we just schedule work.
 */
static irqreturn_t lm8323_irq(int irq, void *data)
{
	struct lm8323_chip *lm = data;

	schedule_work(&lm->work);

	return IRQ_HANDLED;
}

/*
 * Read the chip ID.
 */
static int lm8323_read_id(struct lm8323_chip *lm, u8 *buf)
{
	int bytes;

	bytes = lm8323_read(lm, LM8323_CMD_READ_ID, buf, 2);
	if (unlikely(bytes != 2))
		return -EIO;

	return 0;
}

static void lm8323_write_pwm_one(struct lm8323_pwm *pwm, int pos, u16 cmd)
{
	struct lm8323_chip *lm = pwm_to_lm8323(pwm);

	lm8323_write(lm, 4, LM8323_CMD_PWM_WRITE, (pos << 2) | pwm->id,
		     (cmd & 0xff00) >> 8, cmd & 0x00ff);
}

/*
 * Write a script into a given PWM engine, concluding with PWM_END.
 * If 'keepalive' is specified, the engine will be kept running
 * indefinitely.
 */
static void lm8323_write_pwm(struct lm8323_pwm *pwm, int keepalive,
			     int len, ...)
{
	struct lm8323_chip *lm = pwm_to_lm8323(pwm);
	int i, cmd;
	va_list ap;

	/*
	 * If there are any scripts running at the moment, terminate them
	 * and make sure the duty cycle is as if it finished.
	 */
	lm8323_write(lm, 2, LM8323_CMD_STOP_PWM, pwm->id);

	va_start(ap, len);
	for (i = 0; i < len; i++) {
		cmd = va_arg(ap, int);
		lm8323_write_pwm_one(pwm, i, cmd);
	}
	va_end(ap);

	/* Wait for a trigger from any channel. This keeps the engine alive. */
	if (keepalive)
		lm8323_write_pwm_one(pwm, i++, PWM_WAIT_TRIG(0xe));
	else
		lm8323_write_pwm_one(pwm, i++, PWM_END(1));

	lm8323_write(lm, 2, LM8323_CMD_START_PWM, pwm->id);
}

static void lm8323_pwm_work(struct work_struct *work)
{
	struct lm8323_pwm *pwm = work_to_pwm(work);
	int div, perstep, steps, hz, direction, keepalive;

	/* Do nothing if we're already at the requested level. */
	if (pwm->desired_brightness == pwm->brightness)
		return;

	keepalive = (pwm->desired_brightness > 0);
	direction = (pwm->desired_brightness > pwm->brightness);
	steps = abs(pwm->desired_brightness - pwm->brightness);

	/*
	 * Convert time (in ms) into a divisor (512 or 16 on a refclk of
	 * 32768Hz), and number of ticks per step.
	 */
	if ((pwm->fade_time / steps) > (32768 / 512))
		div = 512;
	else
		div = 16;

	hz = 32768 / div;
	if (pwm->fade_time < ((steps * 1000) / hz))
		perstep = 1;
	else
		perstep = (hz * pwm->fade_time) / (steps * 1000);

	if (perstep == 0)
		perstep = 1;
	else if (perstep > 63)
		perstep = 63;

	if (steps > 252) {
		lm8323_write_pwm(pwm, keepalive, 3,
				 PWM_RAMP((div == 512), perstep, 126,
					  direction),
				 PWM_RAMP((div == 512), perstep, 126,
					  direction),
				 PWM_RAMP((div == 512), perstep, steps - 252,
					  direction));
	} else if (steps > 126) {
		lm8323_write_pwm(pwm, keepalive, 2,
				 PWM_RAMP((div == 512), perstep, 126,
					  direction),
				 PWM_RAMP((div == 512), perstep, steps - 126,
					  direction));
	} else {
		lm8323_write_pwm(pwm, keepalive, 1,
				 PWM_RAMP((div == 512), perstep, steps,
					  direction));
	}

	pwm->brightness = pwm->desired_brightness;
}

static void lm8323_pwm_set_brightness(struct led_classdev *led_cdev,
				      enum led_brightness brightness)
{
	struct lm8323_pwm *pwm = cdev_to_pwm(led_cdev);
	struct lm8323_chip *lm = pwm_to_lm8323(pwm);

	pwm->desired_brightness = brightness;

	if (in_interrupt()) {
		schedule_work(&pwm->work);
	} else {
		/*
		 * Schedule PWM work as usual unless we are going into suspend
		 */
		mutex_lock(&lm->lock);
		if (likely(!lm->pm_suspend))
			schedule_work(&pwm->work);
		else
			lm8323_pwm_work(&pwm->work);
		mutex_unlock(&lm->lock);
	}
}

static ssize_t lm8323_pwm_show_time(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm8323_pwm *pwm = cdev_to_pwm(led_cdev);

	return sprintf(buf, "%d\n", pwm->fade_time);
}

static ssize_t lm8323_pwm_store_time(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm8323_pwm *pwm = cdev_to_pwm(led_cdev);
	int ret;
	int time;

	ret = sscanf(buf, "%d", &time);
	/* Numbers only, please. */
	if (ret)
		return -EINVAL;

	pwm->fade_time = time;

	return strlen(buf);
}
static DEVICE_ATTR(time, 0644, lm8323_pwm_show_time, lm8323_pwm_store_time);

static int init_pwm(struct lm8323_chip *lm, int id, struct device *dev,
		    const char *name)
{
	struct lm8323_pwm *pwm = NULL;

	BUG_ON(id > 3);

	switch (id) {
	case 1:
		pwm = &lm->pwm1;
		break;
	case 2:
		pwm = &lm->pwm2;
		break;
	case 3:
		pwm = &lm->pwm3;
		break;
	}

	pwm->id = id;
	pwm->fade_time = 0;
	pwm->brightness = 0;
	pwm->desired_brightness = 0;
	if (name) {
		pwm->cdev.name = name;
		pwm->cdev.brightness_set = lm8323_pwm_set_brightness;
		if (led_classdev_register(dev, &pwm->cdev) < 0) {
			dev_err(dev, "couldn't register PWM %d\n", id);
			return -1;
		}
		if (device_create_file(pwm->cdev.dev,
					     &dev_attr_time) < 0) {
			dev_err(dev, "couldn't register time attribute\n");
			led_classdev_unregister(&pwm->cdev);
			return -1;
		}
		INIT_WORK(&pwm->work, lm8323_pwm_work);
		pwm->enabled = 1;
	} else {
		pwm->enabled = 0;
	}

	return 0;
}

static struct i2c_driver lm8323_i2c_driver;

static ssize_t lm8323_show_disable(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct lm8323_chip *lm = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", !lm->kp_enabled);
}

static ssize_t lm8323_set_disable(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct lm8323_chip *lm = dev_get_drvdata(dev);
	int ret;
	int i;

	i = sscanf(buf, "%d", &ret);

	mutex_lock(&lm->lock);
	lm->kp_enabled = !i;
	mutex_unlock(&lm->lock);

	return count;
}
static DEVICE_ATTR(disable_kp, 0644, lm8323_show_disable, lm8323_set_disable);

static int lm8323_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct input_dev *idev;
	struct lm8323_chip *lm;
	int i, err = 0;
	unsigned long tmo;
	u8 data[2];

	lm = kzalloc(sizeof *lm, GFP_KERNEL);
	if (!lm)
		return -ENOMEM;

	i2c_set_clientdata(client, lm);
	lm->client = client;
	lm8323_pdata = client->dev.platform_data;
	if (!lm8323_pdata)
		return -EINVAL; /* ? */

	lm->size_x = lm8323_pdata->size_x;
	if (lm->size_x == 0) {
		lm->size_x = 8;
	} else if (lm->size_x > 8) {
		dev_err(&client->dev, "invalid x size %d specified\n",
				lm->size_x);
		lm->size_x = 8;
	}

	lm->size_y = lm8323_pdata->size_y;
	if (lm->size_y == 0) {
		lm->size_y = 12;
	} else if (lm->size_y > 12) {
		dev_err(&client->dev, "invalid y size %d specified\n",
				lm->size_y);
		lm->size_x = 12;
	}

	debug(&c->dev, "Keypad size: %d x %d\n", lm->size_x, lm->size_y);

	lm->debounce_time = lm8323_pdata->debounce_time;
	if (lm->debounce_time == 0) /* Default. */
		lm->debounce_time = 12;
	else if (lm->debounce_time == -1) /* Disable debounce. */
		lm->debounce_time = 0;

	lm->active_time = lm8323_pdata->active_time;
	if (lm->active_time == 0) /* Default. */
		lm->active_time = 500;
	else if (lm->active_time == -1) /* Disable sleep. */
		lm->active_time = 0;

	lm8323_reset(lm);

	/* Nothing's set up to service the IRQ yet, so just spin for max.
	 * 100ms until we can configure. */
	tmo = jiffies + msecs_to_jiffies(100);
	while (lm8323_read(lm, LM8323_CMD_READ_INT, data, 1) == 1) {
		if (data[0] & INT_NOINIT)
			break;

		if (time_after(jiffies, tmo)) {
			dev_err(&client->dev,
					"timeout waiting for initialisation\n");
			break;
		}

		msleep(1);
	}
	lm8323_configure(lm);

	/* If a true probe check the device */
	if (lm8323_read_id(lm, data) != 0) {
		dev_err(&client->dev, "device not found\n");
		err = -ENODEV;
		goto fail2;
	}

	if (init_pwm(lm, 1, &client->dev, lm8323_pdata->pwm1_name) < 0)
		goto fail3;
	if (init_pwm(lm, 2, &client->dev, lm8323_pdata->pwm2_name) < 0)
		goto fail4;
	if (init_pwm(lm, 3, &client->dev, lm8323_pdata->pwm3_name) < 0)
		goto fail5;

	lm->irq = lm8323_pdata->irq_gpio;
	debug(&c->dev, "IRQ: %d\n", lm->irq);

	mutex_init(&lm->lock);
	INIT_WORK(&lm->work, lm8323_work);

	err = request_irq(client->irq, lm8323_irq,
			  IRQF_TRIGGER_FALLING | IRQF_DISABLED |
			  IRQF_SAMPLE_RANDOM, DRIVER_NAME, lm);
	if (err) {
		dev_err(&client->dev, "could not get IRQ %d\n", lm->irq);
		goto fail6;
	}

	set_irq_wake(lm->irq, 1);

	lm->kp_enabled = 1;
	err = device_create_file(&client->dev, &dev_attr_disable_kp);
	if (err < 0)
		goto fail7;

	idev = input_allocate_device();
	if (idev == NULL) {
		err = -ENOMEM;
		goto fail8;
	}

	if (lm8323_pdata->name)
		idev->name = lm8323_pdata->name;
	else
		idev->name = "LM8323 keypad";
	snprintf(lm->phys, sizeof(lm->phys), "%s/input-kp", client->dev.bus_id);
	idev->phys = lm->phys;

	lm->keys_down = 0;
	idev->evbit[0] = BIT(EV_KEY);
	for (i = 0; i < LM8323_KEYMAP_SIZE; i++) {
		if (lm8323_pdata->keymap[i] > 0)
			set_bit(lm8323_pdata->keymap[i], idev->keybit);

		lm->keymap[i] = lm8323_pdata->keymap[i];
	}

	if (lm8323_pdata->repeat)
		set_bit(EV_REP, idev->evbit);

	lm->idev = idev;
	if (input_register_device(idev)) {
		dev_dbg(&client->dev, "error registering input device\n");
		goto fail8;
	}

	return 0;

fail8:
	device_remove_file(&client->dev, &dev_attr_disable_kp);
fail7:
	free_irq(lm->irq, lm);
fail6:
	if (lm->pwm3.enabled)
		led_classdev_unregister(&lm->pwm3.cdev);
fail5:
	if (lm->pwm2.enabled)
		led_classdev_unregister(&lm->pwm2.cdev);
fail4:
	if (lm->pwm1.enabled)
		led_classdev_unregister(&lm->pwm1.cdev);
fail3:
fail2:
	kfree(lm);
	return err;
}

static int lm8323_remove(struct i2c_client *client)
{
	struct lm8323_chip *lm = i2c_get_clientdata(client);

	free_irq(lm->irq, lm);
	device_remove_file(&lm->client->dev, &dev_attr_disable_kp);

	return 0;
}

/*
 * We don't need to explicitly suspend the chip, as it already switches off
 * when there's no activity.
 */
static int lm8323_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lm8323_chip *lm = i2c_get_clientdata(client);

	set_irq_wake(lm->irq, 0);
	disable_irq(lm->irq);

	mutex_lock(&lm->lock);
	lm->pm_suspend = 1;
	mutex_unlock(&lm->lock);

	if (lm->pwm1.enabled)
		led_classdev_suspend(&lm->pwm1.cdev);
	if (lm->pwm2.enabled)
		led_classdev_suspend(&lm->pwm2.cdev);
	if (lm->pwm3.enabled)
		led_classdev_suspend(&lm->pwm3.cdev);

	return 0;
}

static int lm8323_resume(struct i2c_client *client)
{
	struct lm8323_chip *lm = i2c_get_clientdata(client);

	mutex_lock(&lm->lock);
	lm->pm_suspend = 0;
	mutex_unlock(&lm->lock);

	if (lm->pwm1.enabled)
		led_classdev_resume(&lm->pwm1.cdev);
	if (lm->pwm2.enabled)
		led_classdev_resume(&lm->pwm2.cdev);
	if (lm->pwm3.enabled)
		led_classdev_resume(&lm->pwm3.cdev);

	enable_irq(lm->irq);
	set_irq_wake(lm->irq, 1);

	return 0;
}

static const struct i2c_device_id lm8323_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};

static struct i2c_driver lm8323_i2c_driver = {
	.driver = {
		.name	 = DRIVER_NAME,
	},
	.probe		= lm8323_probe,
	.remove		= __devexit_p(lm8323_remove),
	.suspend	= lm8323_suspend,
	.resume		= lm8323_resume,
	.id_table	= lm8323_id,
};
MODULE_DEVICE_TABLE(i2c, lm8323_id);

static int __init lm8323_init(void)
{
	return i2c_add_driver(&lm8323_i2c_driver);
}

static void __exit lm8323_exit(void)
{
	i2c_del_driver(&lm8323_i2c_driver);
}

MODULE_AUTHOR("Daniel Stone");
MODULE_DESCRIPTION("LM8323 keypad driver");
MODULE_LICENSE("GPL");

module_init(lm8323_init);
module_exit(lm8323_exit);
