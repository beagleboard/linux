/*
 * jhd629-i2c.c: Serial driver (over I2C) for the JHD629 series of modules
 *
 * Copyright (C) 2013 Pantelis Antoniou <panto@antoniou-consulting.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/idr.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <asm/bitops.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <linux/err.h>

#define JHD629_MAJOR			204
#define JHD629_MINOR			220

/* maximum 4 instances... (meh; tty layer is archaic) */
#define JHD629_MAX_INSTANCES		4

/* just one byte */
#define JHD629_FIFO_SIZE		1

struct jhd629_port {
	int id;
	struct i2c_client 	*client;

	struct uart_driver	uart;
	struct uart_port	port;

	const char		*name;

	struct workqueue_struct	*tx_wq;
	struct work_struct	tx_work;
	struct workqueue_struct	*rx_wq;
	struct work_struct	rx_work;

	struct mutex		lock;

	int 			keypad_irq;
	char			keypad_gpio_name[32];
	struct timer_list	poll_timer;
	unsigned int		poll_timer_running : 1;

	/* escape in progress */
	int			esc_mode;
#define ESC_NONE	0
#define ESC_ESC		1
#define ESC_LBRACKET	2
	int			esc_next;
	char			esc_buf[64];

	/* configuration data */
	int			rows, cols;
	char			keymap[16 + 1];	/* 4x4 + 1 '\0' */
	int			keypad_gpio;
	int			poll_period;
};

static DEFINE_IDA(jhd629_ida);

static void jhd629_send_byte(struct jhd629_port *s, u8 ch)
{
	i2c_smbus_write_byte(s->client, ch);
}

static int jhd629_recv_byte(struct jhd629_port *s)
{
	return i2c_smbus_read_byte(s->client);
}

static void jhd629_send_block(struct jhd629_port *s, const void *buf, int len)
{
	const u8 *p;

	p = buf;
	while (len-- > 0)
		jhd629_send_byte(s, *p++);
}

static void jhd629_send_str(struct jhd629_port *s, const char *str)
{
	jhd629_send_block(s, str, strlen(str));
}

static void jhd629_handle_escape(struct jhd629_port *s)
{
	char esc_ch;
	char *ep, *epe, *epr, *epnext;
	int num = 0, row = 0, col = 0, qmark = 0, num_exists = 0;
	char xmitbuf[8];

	if (s->esc_mode == ESC_NONE || s->esc_next <= 0)
		goto esc_end;

	/* fast access to escape buffer */
	ep = s->esc_buf;

	/* same to the end of the buffer */
	epe = s->esc_buf + s->esc_next;

	/* single character escapes */
	if (ep[0] != '[') {
		esc_ch = ep[0];

		switch (esc_ch) {
		case 'c':
			dev_dbg(s->port.dev, "esc-RESET\n");
			xmitbuf[0] = 0x1b;
			xmitbuf[1] = 'c';
			jhd629_send_block(s, xmitbuf, 2);
			break;
		case 'a':
			dev_dbg(s->port.dev, "esc-ACK\n");
			/* I2C does not have ACK */
			break;
		default:
			/* nothing */
			break;
		}
	} else {

		esc_ch = epe[-1];	/* last character */

		dev_dbg(s->port.dev, "handle escape 0x1b %c\n", esc_ch);
		num = 1;

		epr = ep + 1;
		qmark = 0;
		if (*epr == '?') {
			epr++;
			qmark = 1;
		}
		/* digit?, read it */
		if (*epr >= '0' && *epr <= '9') {
			num_exists = 1;
			num = simple_strtoul(epr, &epnext, 10);
			if (*epnext == ',' || *epnext == ';') {
				row = num;
				col = simple_strtoul(epnext + 1, NULL, 10);
			}
		} else {
			num_exists = 0;
			num = 1;
			row = 1;
			col = 1;
		}

		if (!qmark) {
			switch (esc_ch) {
			case 'A':
				dev_dbg(s->port.dev, "esc-UP%d\n", num);
				while (num-- > 0)
					jhd629_send_str(s, "\x1b\x20");
				break;
			case 'B':
				dev_dbg(s->port.dev, "esc-DOWN%d\n", num);
				while (num-- > 0)
					jhd629_send_str(s, "\x1b\x21");
				break;
			case 'C':
				dev_dbg(s->port.dev, "esc-RIGHT%d\n", num);
				while (num-- > 0)
					jhd629_send_str(s, "\x1b\x22");
				break;
			case 'D':
				dev_dbg(s->port.dev, "esc-RIGHT%d\n", num);
				while (num-- > 0)
					jhd629_send_str(s, "\x1b\x23");
				break;
			case 'H':
				dev_dbg(s->port.dev, "esc-HOME(%d;%d)\n", row, col);

				if (col != 1 || row != 1) {
					xmitbuf[0] = 0x1b;
					xmitbuf[1] = 0x24;
					xmitbuf[2] = (u8)row;
					xmitbuf[3] = (u8)col;
					jhd629_send_block(s, xmitbuf, 4);
				} else
					jhd629_send_str(s, "\x1b\x25");
				break;

			case 'E':
				dev_dbg(s->port.dev, "esc-DIRECT%d\n", num);
				xmitbuf[0] = 0x1b;
				xmitbuf[1] = 0x01;
				xmitbuf[2] = num;
				jhd629_send_block(s, xmitbuf, 3);
				break;

			case 'L':
				dev_dbg(s->port.dev, "esc-LINES%d\n", num);
				xmitbuf[0] = 0x1b;
				xmitbuf[1] = 0x30;
				xmitbuf[2] = num;
				jhd629_send_block(s, xmitbuf, 3);
				break;

			case 'c':
				dev_dbg(s->port.dev, "esc-COLUMNS%d\n", num);
				xmitbuf[0] = 0x1b;
				xmitbuf[1] = 0x31;
				xmitbuf[2] = num;
				jhd629_send_block(s, xmitbuf, 3);
				break;

			case 'b':
				dev_dbg(s->port.dev, "esc-BAUD%d\n", num);
				/* ignored */
				break;

			case 'd':
				dev_dbg(s->port.dev, "esc-LF/CR%d\n", num);
				/* ignored */
				break;

			case 'x':
				dev_dbg(s->port.dev, "esc-VERTSCROLL%d\n", num);
				xmitbuf[0] = 0x1b;
				xmitbuf[1] = 0x45;
				xmitbuf[2] = num;
				jhd629_send_block(s, xmitbuf, 3);
				break;

			case 'a':
				dev_dbg(s->port.dev, "esc-I2CADDR%d\n", num);
				xmitbuf[0] = 0x1b;
				xmitbuf[1] = 0x42;
				xmitbuf[2] = 0x55;
				xmitbuf[3] = 0x37;
				xmitbuf[4] = num;
				jhd629_send_block(s, xmitbuf, 5);
				break;

			case 'J':
				if (num == 4) {
					dev_dbg(s->port.dev, "esc-RESETEEPROM\n");
					jhd629_send_str(s, "\x1b\x44");
				} else if (num == 2) {
					dev_dbg(s->port.dev, "esc-CLEARSCR\n");
					jhd629_send_str(s, "\x1b\x50");
				}
				break;
			case 'K':
				if (!num_exists) {
					dev_dbg(s->port.dev, "esc-CLRLINERIGHT\n");
					jhd629_send_str(s, "\x1b\x51");
				} else if (num == 1) {
					dev_dbg(s->port.dev, "esc-CLRLINELEFT\n");
					jhd629_send_str(s, "\x1b\x52");
				} else if (num == 2) {
					dev_dbg(s->port.dev, "esc-CLRLINE\n");
					jhd629_send_str(s, "\x1b\x53");
				}
				break;
			}
		} else {
			switch (esc_ch) {
			case 'I':
				if (num == 25) {
					dev_dbg(s->port.dev, "esc-CURSOROFF\n");
					xmitbuf[0] = 0x1b;
					xmitbuf[1] = 0x01;
					xmitbuf[2] = 0x0c;
					jhd629_send_block(s, xmitbuf, 3);
				} else if (num == 26) {
					dev_dbg(s->port.dev, "esc-BACKLIGHTOFF\n");
					xmitbuf[0] = 0x1b;
					xmitbuf[1] = 0x03;
					xmitbuf[2] = 0x00;
					jhd629_send_block(s, xmitbuf, 3);
				}
				break;
			case 'h':
				if (num == 25) {
					dev_dbg(s->port.dev, "esc-CURSORON\n");
					xmitbuf[0] = 0x1b;
					xmitbuf[1] = 0x01;
					xmitbuf[2] = 0x0e;
					jhd629_send_block(s, xmitbuf, 3);
				} else if (num == 26) {
					dev_dbg(s->port.dev, "esc-BACKLIGHTON\n");
					xmitbuf[0] = 0x1b;
					xmitbuf[1] = 0x03;
					xmitbuf[2] = 0x01;
					jhd629_send_block(s, xmitbuf, 3);
				}
				break;
			case 'd':
				break;
			case 'f':
				break;
			case 'S':
				break;
			}
		}
	}

esc_end:
	s->esc_mode = ESC_NONE;
	s->esc_next = 0;
}

static void jh629_cancel_escape(struct jhd629_port *s)
{
	s->esc_mode = ESC_NONE;
	s->esc_next = 0;
}

/* lock should be taken */
static void jhd629_tx_char(struct jhd629_port *s, int ch)
{
	/* normal, non-escaped output */
	if (s->esc_mode == ESC_NONE) {
		switch (ch) {
		case 0x1b:	/* ESC */
			s->esc_mode = ESC_ESC;
			s->esc_next = 0;
			s->esc_buf[s->esc_next] = '\0';
			break;
		case '\n':	/* LF */
			jhd629_send_str(s, "\x1b\x0a");
			break;
		case '\r':	/* CR */
			jhd629_send_str(s, "\x1b\x0d");
			break;
		case '\b':	/* BS */
			/* ignored */
			break;
		default:
			jhd629_send_byte(s, ch);
			break;
		}
		return;
	}

	/* an escaped mode */

	/* append to escape buffer */
	if (s->esc_next < ARRAY_SIZE(s->esc_buf) - 1) {
		s->esc_buf[s->esc_next++] = ch;
		s->esc_buf[s->esc_next] = '\0';
	}

	switch (s->esc_mode) {

	case ESC_NONE:
		/* never happens */
		break;

	case ESC_ESC:
		switch (ch) {
		case '[':	/* left bracket */
			s->esc_mode = ESC_LBRACKET;
			break;
		case 'a':
		case 'c':
			jhd629_handle_escape(s);
			break;
		default:
			/* unknown escape sequence (ignore) */
			jh629_cancel_escape(s);
			break;
		}
		break;

	case ESC_LBRACKET:
		switch (ch) {
		case '0': case '1': case '2': case '3': case '4':
		case '5': case '6': case '7': case '8': case '9':
		case ';': case '?': case ',':
			break;
		default:
			jhd629_handle_escape(s);
			break;
		}
	}
}

static void jhd629_handle_rx(struct jhd629_port *s)
{
	struct tty_struct *tty = tty_port_tty_get(&s->port.state->port);
	int rx_len, rx_cnt, ch;
	int row, col;

	if (!tty)
		return;

	/* find out how many keys in the buffer */
	rx_cnt = 0;
	for (;;) {
		jhd629_send_byte(s, 0x1b);
		jhd629_send_byte(s, 0x10);
		rx_len = jhd629_recv_byte(s);
		if (rx_len <= 0)
			break;

		/* now get key scan code */
		while (rx_len--) {

			jhd629_send_byte(s, 0x1b);
			jhd629_send_byte(s, 0x11);
			ch = jhd629_recv_byte(s);
			if (ch <= 0)
				continue;

			row = ffz(ch & 0xf);
			col = ffz((ch >> 4) & 0xf);

			ch = s->keymap[row * 4 + col];

			rx_cnt++;

			s->port.icount.rx++;
			if (uart_handle_sysrq_char(s->port, ch))
				continue;

			dev_dbg(s->port.dev, "rx 0x%02x\n", (int)ch & 0xff);

			uart_insert_char(&s->port, 0, 0, ch, TTY_NORMAL);
		}
	}

	if (rx_cnt > 0)
		tty_flip_buffer_push(tty);

	tty_kref_put(tty);
}

static void jhd629_handle_tx(struct jhd629_port *s)
{
	struct circ_buf *xmit = &s->port.state->xmit;
	unsigned int to_send;

	if (unlikely(s->port.x_char)) {

		jhd629_tx_char(s, s->port.x_char);

		s->port.icount.tx++;
		s->port.x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&s->port))
		return;

	/* Get length of data pending in circular buffer */
	to_send = uart_circ_chars_pending(xmit);
	if (likely(to_send)) {

		/* Add data to send */
		s->port.icount.tx += to_send;
		while (to_send--) {

			jhd629_tx_char(s, xmit->buf[xmit->tail]);

			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		};
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&s->port);
}

static void jhd629_tx_wq_proc(struct work_struct *ws)
{
	struct jhd629_port *s = container_of(ws, struct jhd629_port, tx_work);

	mutex_lock(&s->lock);
	jhd629_handle_tx(s);
	mutex_unlock(&s->lock);
}

static void jhd629_rx_wq_proc(struct work_struct *ws)
{
	struct jhd629_port *s = container_of(ws, struct jhd629_port, rx_work);

	mutex_lock(&s->lock);
	jhd629_handle_rx(s);
	mutex_unlock(&s->lock);
}

static void jhd629_start_tx(struct uart_port *port)
{
	struct jhd629_port *s = container_of(port, struct jhd629_port, port);

	queue_work(s->tx_wq, &s->tx_work);
}

static void jhd629_stop_tx(struct uart_port *port)
{
	/* Do nothing */
}

static void jhd629_stop_rx(struct uart_port *port)
{
	struct jhd629_port *s = container_of(port, struct jhd629_port, port);

	mutex_lock(&s->lock);

	if (s->poll_timer_running) {
		del_timer_sync(&s->poll_timer);
		s->poll_timer_running = 0;
	}

	mutex_unlock(&s->lock);
}

static unsigned int jhd629_tx_empty(struct uart_port *port)
{
	/* transmitter is always empty */
	return TIOCSER_TEMT;
}

static void jhd629_enable_ms(struct uart_port *port)
{
	/* Modem status not supported */
}

static unsigned int jhd629_get_mctrl(struct uart_port *port)
{
	/* DCD and DSR are not wired and CTS/RTS is handled automatically
	 * so just indicate DSR and CAR asserted
	 */
	return TIOCM_DSR | TIOCM_CAR;
}

static void jhd629_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* DCD and DSR are not wired and CTS/RTS is hadnled automatically
	 * so do nothing
	 */
}

static void jhd629_break_ctl(struct uart_port *port, int break_state)
{
	/* nothing */
}

static void jhd629_set_termios(struct uart_port *port,
				struct ktermios *termios,
				struct ktermios *old)
{
	struct jhd629_port *s = container_of(port, struct jhd629_port, port);

	/* we don't support much... */

	mutex_lock(&s->lock);

	/* Mask termios capabilities we don't support */
	termios->c_cflag &= ~CMSPAR;
	termios->c_iflag &= ~IXANY;

	/* Set read status mask */
	port->read_status_mask = 0;

	/* Set status ignore mask */
	port->ignore_status_mask = 0;

	mutex_unlock(&s->lock);
}

static int jhd629_startup(struct uart_port *port)
{
	struct jhd629_port *s = container_of(port, struct jhd629_port, port);

	mutex_lock(&s->lock);

	if (s->keypad_irq < 0 && !s->poll_timer_running) {
		s->poll_timer.expires = jiffies +
				msecs_to_jiffies(s->poll_period);
		s->poll_timer_running = 1;
		add_timer(&s->poll_timer);
	}

	mutex_unlock(&s->lock);

	return 0;
}

static void jhd629_shutdown(struct uart_port *port)
{
	struct jhd629_port *s = container_of(port, struct jhd629_port, port);

	mutex_lock(&s->lock);

	if (s->poll_timer_running) {
		del_timer_sync(&s->poll_timer);
		s->poll_timer_running = 0;
	}

	mutex_unlock(&s->lock);
}

static const char *jhd629_type(struct uart_port *port)
{
	struct jhd629_port *s = container_of(port, struct jhd629_port, port);

	return (port->type == PORT_JHD629) ? s->name : NULL;
}

static int jhd629_request_port(struct uart_port *port)
{
	/* Do nothing */
	return 0;
}

static void jhd629_release_port(struct uart_port *port)
{
	/* Do nothing */
}

static void jhd629_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_JHD629;
}

static int jhd629_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->type == PORT_UNKNOWN || ser->type == PORT_JHD629)
		return 0;

	return -EINVAL;
}

static struct uart_ops jhd629_ops = {
	.tx_empty	= jhd629_tx_empty,
	.set_mctrl	= jhd629_set_mctrl,
	.get_mctrl	= jhd629_get_mctrl,
	.stop_tx	= jhd629_stop_tx,
	.start_tx	= jhd629_start_tx,
	.stop_rx	= jhd629_stop_rx,
	.enable_ms	= jhd629_enable_ms,
	.break_ctl	= jhd629_break_ctl,
	.startup	= jhd629_startup,
	.shutdown	= jhd629_shutdown,
	.set_termios	= jhd629_set_termios,
	.type		= jhd629_type,
	.request_port	= jhd629_request_port,
	.release_port	= jhd629_release_port,
	.config_port	= jhd629_config_port,
	.verify_port	= jhd629_verify_port,
};

static irqreturn_t jhd629_keypad_irq(int irq, void *dev_id)
{
	struct jhd629_port *s = dev_id;

	/* just queue the check */
	queue_work(s->rx_wq, &s->rx_work);

	return IRQ_HANDLED;
}

static void jhd629_poll_timer(unsigned long ptr)
{
	struct jhd629_port *s = (void *)ptr;

	/* check for input in the rx work queue */
	queue_work(s->rx_wq, &s->rx_work);

	/* schedule next timer poll */
	s->poll_timer.expires = jiffies +
			msecs_to_jiffies(s->poll_period);
	add_timer(&s->poll_timer);
}

/* we only support DT for now */
static int jhd629_setup(struct jhd629_port *s)
{
	struct device_node *node = s->client->dev.of_node;
	const char *keymap;
	static const char *default_keymap =
		"123A"
		"456B"
		"789C"
		"*0#D";
	u32 val;

	/* rows */
	if (of_property_read_u32(node, "rows", &val) == 0)
		s->rows = val;
	else
		s->rows = 4;

	/* columns */
	if (of_property_read_u32(node, "columns", &val) == 0)
		s->cols = val;
	else
		s->cols = 20;

	/* keymap */
	if (of_property_read_string(node, "keymap", &keymap) != 0)
		keymap = default_keymap;
	strncpy(s->keymap, keymap, sizeof(s->keymap));

	/* poll_period */
	if (of_property_read_u32(node, "poll-period", &val) == 0)
		s->poll_period = val;
	else
		s->poll_period = 250;

	s->keypad_gpio = of_get_gpio_flags(node, 0, NULL);
	if (IS_ERR_VALUE(s->keypad_gpio))
		s->keypad_gpio = -1; /* no gpio, switch to polling */

	return 0;
}

static int jhd629_apply_irq_config(struct jhd629_port *s)
{
	int err;

	snprintf(s->keypad_gpio_name, sizeof(s->keypad_gpio_name) - 1,
			"jhd629:%d", s->id);

	err = gpio_request_one(s->keypad_gpio,
			GPIOF_DIR_IN | GPIOF_EXPORT,
			s->keypad_gpio_name);
	if (err != 0) {
		dev_err(s->port.dev, "Failed to request keypad INT\n");
		goto err_no_req;
	}

	s->keypad_irq = gpio_to_irq(s->keypad_gpio);
	if (IS_ERR_VALUE(s->keypad_irq)) {
		dev_err(s->port.dev, "unable to get keypad IRQ\n");
		err = s->keypad_irq;
		s->keypad_irq = -1;
		goto err_no_irq;
	}

	err = request_irq(s->keypad_irq, jhd629_keypad_irq,
			IRQF_TRIGGER_FALLING | IRQF_SHARED,
			s->keypad_gpio_name, s);
	if (err != 0) {
		dev_err(s->port.dev, "unable to request keypad irq\n");
		goto err_no_irq;
	}

	dev_dbg(s->port.dev, "Using keyboard irq #%d\n",
			s->keypad_irq);
	return 0;

err_no_irq:
	s->keypad_irq = -1;
	gpio_free(s->keypad_gpio);
err_no_req:
	s->keypad_gpio = -1;
	return err;
}

static int jhd629_apply_config(struct jhd629_port *s)
{
	int err;

	/* clear screen */
	jhd629_send_block(s, "\x1b\x50", 2);

	/* columns */
	jhd629_send_block(s, "\x1b\x31", 2);
	jhd629_send_byte(s, s->cols);

	/* rows */
	jhd629_send_block(s, "\x1b\x30", 2);
	jhd629_send_byte(s, s->rows);

	/* try to apply the irq config (if possible) */
	if (s->keypad_gpio >= 0) {
		err = jhd629_apply_irq_config(s);
		if (err == 0)
			return 0;
	}

	dev_dbg(s->port.dev, "Using polling period of %dms\n",
			s->poll_period);

	return 0;
}

static int jhd629_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	/* struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent); */
	struct device *dev = &client->dev;
	struct jhd629_port *s = NULL;
	int ret = 0;
	struct pinctrl *pinctrl;

	/* this better be early */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_warn(dev,
			"pins are not configured from the driver\n");

	/* Alloc port structure */
	s = devm_kzalloc(dev, sizeof(struct jhd629_port), GFP_KERNEL);
	if (!s) {
		dev_err(dev, "Error allocating port structure\n");
		return -ENOMEM;
	}
	s->id = -1;

	s->client = client;
	i2c_set_clientdata(client, s);

	mutex_init(&s->lock);

	s->id = ida_simple_get(&jhd629_ida, 0, JHD629_MAX_INSTANCES, GFP_KERNEL);
	if (s->id < 0) {
		dev_err(dev, "Failed to get port ID (too many instances?)\n");
		goto err_out;
	}

	/* Register UART driver */
	s->uart.owner		= THIS_MODULE;
	s->uart.driver_name	= dev_name(dev);
	s->uart.dev_name	= "ttyJHD";
	s->uart.major		= JHD629_MAJOR;
	s->uart.minor		= JHD629_MINOR + s->id;
	s->uart.nr		= 1;
	s->name			= "JHD629";
	ret = uart_register_driver(&s->uart);
	if (ret != 0) {
		dev_err(dev, "Registering UART driver failed\n");
		goto err_out;
	}

	/* Initialize workqueue for start TX */
	s->tx_wq = create_freezable_workqueue(dev_name(dev));
	INIT_WORK(&s->tx_work, jhd629_tx_wq_proc);

	/* Initialize workqueue for RX polling */
	s->rx_wq = create_freezable_workqueue(dev_name(dev));
	INIT_WORK(&s->rx_work, jhd629_rx_wq_proc);

	init_timer(&s->poll_timer);
	s->poll_timer.function = jhd629_poll_timer;
	s->poll_timer.data = (unsigned long)s;

	s->keypad_gpio = -1;
	s->keypad_irq = -1;

	/* Initialize UART port data */
	s->port.line		= 0;
	s->port.dev		= dev;
	s->port.irq		= -1;
	s->port.type		= PORT_JHD629;
	s->port.fifosize	= 1;
	s->port.flags		= UPF_SKIP_TEST | UPF_FIXED_TYPE;
	s->port.iotype		= UPIO_PORT;
	s->port.membase		= (void __iomem *)0xffffffff; /* Bogus value */
	s->port.uartclk		= 9600 * 16;	/* emulate a 9600 baud port */
	s->port.ops		= &jhd629_ops;
	ret = uart_add_one_port(&s->uart, &s->port);
	if (ret != 0) {
		dev_err(s->port.dev, "Adding a port failed\n");
		goto err_port;
	}

	ret = jhd629_setup(s);
	if (ret != 0) {
		dev_err(s->port.dev, "jhd629_setup failed\n");
		goto err_port;
	}

	ret = jhd629_apply_config(s);
	if (ret != 0) {
		dev_err(s->port.dev, "jhd629_apply_config failed\n");
		goto err_port;
	}

	dev_dbg(s->port.dev, "Added port #%d\n", s->id);

	return 0;

err_port:
	uart_remove_one_port(&s->uart, &s->port);

err_out:
	if (s != NULL) {
		if (s->id >= 0) {
			ida_simple_remove(&jhd629_ida, s->id);
			s->id = -1;
		}
	}
	i2c_set_clientdata(client, NULL);

	return ret;
}

static int jhd629_remove(struct i2c_client *client)
{
	struct jhd629_port *s = i2c_get_clientdata(client);

	dev_dbg(s->port.dev, "Removing port #%d\n", s->id);

	if (s->keypad_irq >= 0) {
		free_irq(s->keypad_irq, s);
		gpio_free(s->keypad_gpio);
	}

	if (s->poll_timer_running) {
		del_timer_sync(&s->poll_timer);
		s->poll_timer_running = 0;
	}

	destroy_workqueue(s->rx_wq);
	destroy_workqueue(s->tx_wq);

	uart_remove_one_port(&s->uart, &s->port);
	uart_unregister_driver(&s->uart);

	ida_simple_remove(&jhd629_ida, s->id);

	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id jhd629_id_table[] = {
	{ "jhd629" },
	{ }
};
MODULE_DEVICE_TABLE(spi, jhd629_id_table);

static struct i2c_driver jhd629_driver = {
	.driver = {
		.name	= "jhd629-i2c",
		.owner	= THIS_MODULE,
	},
	.probe		= jhd629_probe,
	.remove		= jhd629_remove,
	.id_table	= jhd629_id_table,
};
module_i2c_driver(jhd629_driver);

MODULE_AUTHOR("Pantelis Antoniou <panto@antoniou-consulting.com>");
MODULE_DESCRIPTION("JHD629-i2c driver");
MODULE_LICENSE("GPL");
