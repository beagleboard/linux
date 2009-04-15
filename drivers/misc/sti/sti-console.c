/*
 * Console support for OMAP STI/XTI
 *
 * Copyright (C) 2004, 2005, 2006 Nokia Corporation
 * Written by: Paul Mundt <paul.mundt@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/console.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <mach/sti.h>
#include <mach/board.h>

#define DRV_NAME "sticon"

static struct tty_driver *tty_driver;
static DEFINE_SPINLOCK(sti_console_lock);
static unsigned int sti_console_channel = -1;
static int sti_line_done = -1;

/*
 * Write a string to any channel (including terminating NULL)
 * Returns number of characters written.
 */
static int sti_channel_puts(const char *string, unsigned int channel, int len)
{
	int count = 0;

	/*
	 * sti_line_done is needed to determine when we have reached the
	 * end of the line. write() has a tendency to hand us small
	 * strings which otherwise end up creating newlines.. we need to
	 * keep the channel open and in append mode until the line has
	 * been terminated.
	 */
	if (sti_line_done != 0) {
#ifdef __LITTLE_ENDIAN
		sti_channel_writeb(0xc3, channel);
#else
		sti_channel_writeb(0xc0, channel);
#endif
		xchg(&sti_line_done, 0);
	}

	while (*string && count != len) {
		char c = *string++;

		count++;

		if (c == '\n') {
			xchg(&sti_line_done, 1);
			sti_channel_writeb(0, channel);
			break;
		} else
			sti_channel_writeb(c, channel);
	}

	if (sti_line_done)
		sti_channel_flush(channel);

	return count;
}

static int sti_tty_open(struct tty_struct *tty, struct file *filp)
{
	return 0;
}

static int sti_tty_write(struct tty_struct *tty,
			 const unsigned char *buf, int len)
{
	unsigned long flags;
	int bytes;

	spin_lock_irqsave(&sti_console_lock, flags);
	bytes = sti_channel_puts(buf, sti_console_channel, len);
	spin_unlock_irqrestore(&sti_console_lock, flags);

	return bytes;
}

static int sti_tty_write_room(struct tty_struct *tty)
{
	return 0x100000;
}

static int sti_tty_chars_in_buffer(struct tty_struct *tty)
{
	return 0;
}

static struct tty_operations sti_tty_ops = {
	.open			= sti_tty_open,
	.write			= sti_tty_write,
	.write_room		= sti_tty_write_room,
	.chars_in_buffer	= sti_tty_chars_in_buffer,
};

static void sti_console_write(struct console *c, const char *s, unsigned n)
{
	unsigned long flags;

	spin_lock_irqsave(&sti_console_lock, flags);
	sti_channel_puts(s, sti_console_channel, n);
	spin_unlock_irqrestore(&sti_console_lock, flags);
}

static struct tty_driver *sti_console_device(struct console *c, int *index)
{
	*index = c->index;
	return tty_driver;
}

static int sti_console_setup(struct console *c, char *opts)
{
	return 0;
}

static struct console sti_console = {
	.name		= DRV_NAME,
	.write		= sti_console_write,
	.device		= sti_console_device,
	.setup		= sti_console_setup,
	.flags		= CON_PRINTBUFFER | CON_ENABLED,
	.index		= -1,
};

static int __init sti_console_init(void)
{
	const struct omap_sti_console_config *info;

	info = omap_get_config(OMAP_TAG_STI_CONSOLE,
			       struct omap_sti_console_config);
	if (info && info->enable) {
		add_preferred_console(DRV_NAME, 0, NULL);

		sti_console_channel = info->channel;
	}

	if (unlikely(sti_console_channel == -1))
		return -EINVAL;

	register_console(&sti_console);

	return 0;
}
__initcall(sti_console_init);

static int __init sti_tty_init(void)
{
	struct tty_driver *tty;
	int ret;

	tty = alloc_tty_driver(1);
	if (!tty)
		return -ENOMEM;

	tty->name		= DRV_NAME;
	tty->driver_name	= DRV_NAME;
	tty->major		= 0;	/* dynamic major */
	tty->minor_start	= 0;
	tty->type		= TTY_DRIVER_TYPE_SYSTEM;
	tty->subtype		= SYSTEM_TYPE_SYSCONS;
	tty->init_termios	= tty_std_termios;

	tty_set_operations(tty, &sti_tty_ops);

	ret = tty_register_driver(tty);
	if (ret) {
		put_tty_driver(tty);
		return ret;
	}

	tty_driver = tty;
	return 0;
}
late_initcall(sti_tty_init);

module_param(sti_console_channel, uint, 0);
MODULE_PARM_DESC(sti_console_channel, "STI console channel");
MODULE_AUTHOR("Paul Mundt");
MODULE_DESCRIPTION("OMAP STI console support");
MODULE_LICENSE("GPL");
