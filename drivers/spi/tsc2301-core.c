/*
 * TSC2301 driver
 *
 * Copyright (C) 2005, 2006 Nokia Corporation
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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc2301.h>

u16 tsc2301_read_reg(struct tsc2301 *tsc, int reg)
{
	struct spi_transfer t[2];
	struct spi_message m;
	u16 data = 0, cmd;

	cmd = reg;
	cmd |= 0x8000;

	memset(t, 0, sizeof(t));
	spi_message_init(&m);
	m.spi = tsc->spi;

	t[0].tx_buf = &cmd;
	t[0].rx_buf = NULL;
	t[0].len = 2;
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = NULL;
	t[1].rx_buf = &data;
	t[1].len = 2;
	spi_message_add_tail(&t[1], &m);

	spi_sync(m.spi, &m);

	return data;
}

void tsc2301_write_reg(struct tsc2301 *tsc, int reg, u16 val)
{
	struct spi_transfer t;
	struct spi_message m;
	u16 data[2];

	/* Now we prepare the command for transferring */
	data[0] = reg;
	data[1] = val;

	spi_message_init(&m);
	m.spi = tsc->spi;

	memset(&t, 0, sizeof(t));
	t.tx_buf = data;
	t.rx_buf = NULL;
	t.len = 4;
	spi_message_add_tail(&t, &m);

	spi_sync(m.spi, &m);
}

void tsc2301_write_kbc(struct tsc2301 *tsc, int val)
{
	u16 w;

	w = tsc->config2_shadow;
	w &= ~(0x03 << 14);
	w |= (val & 0x03) << 14;
	tsc2301_write_reg(tsc, TSC2301_REG_CONFIG2, w);
	tsc->config2_shadow = w;
}

void tsc2301_write_pll(struct tsc2301 *tsc,
		       int pll_n, int pll_a, int pll_pdc, int pct_e, int pll_o)
{
	u16 w;

	w = tsc->config2_shadow;
	w &= ~0x3fff;
	w |= (pll_n & 0x0f) | ((pll_a & 0x0f) << 4) | ((pll_pdc & 0x0f) << 8);
	w |= pct_e ? (1 << 12) : 0;
	w |= pll_o ? (1 << 13) : 0;
	tsc2301_write_reg(tsc, TSC2301_REG_CONFIG2, w);
	tsc->config2_shadow = w;
}

void tsc2301_read_buf(struct tsc2301 *tsc, int reg, u16 *rx_buf, int len)
{
	struct spi_transfer t[2];
	struct spi_message m;
	u16 cmd, i;

	cmd = reg;
	cmd |= 0x8000;

	spi_message_init(&m);
	m.spi = tsc->spi;

	memset(t, 0, sizeof(t));
	t[0].tx_buf = &cmd;
	t[0].rx_buf = NULL;
	t[0].len = 2;
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = NULL;
	t[1].rx_buf = rx_buf;
	t[1].len = 2 * len;
	spi_message_add_tail(&t[1], &m);

	spi_sync(m.spi, &m);

	for (i = 0; i < len; i++)
		printk(KERN_DEBUG "rx_buf[%d]: %04x\n", i, rx_buf[i]);
}

static int __devinit tsc2301_probe(struct spi_device *spi)
{
	struct tsc2301			*tsc;
	struct tsc2301_platform_data	*pdata = spi->dev.platform_data;
	int r;
	u16 w;

	if (!pdata) {
		dev_dbg(&spi->dev, "no platform data?\n");
		return -ENODEV;
	}

	tsc = kzalloc(sizeof(*tsc), GFP_KERNEL);
	if (tsc == NULL)
		return -ENOMEM;

	dev_set_drvdata(&spi->dev, tsc);
	tsc->spi = spi;

	tsc->enable_clock = pdata->enable_clock;
	tsc->disable_clock = pdata->disable_clock;

	if (pdata->reset_gpio >= 0) {
		tsc->reset_gpio = pdata->reset_gpio;
		r = gpio_request(tsc->reset_gpio, "TSC2301 reset");
		if (r < 0)
			goto err1;
		gpio_direction_output(tsc->reset_gpio, 1);
		mdelay(1);
		gpio_set_value(tsc->reset_gpio, 0);
	} else
		tsc->reset_gpio = -1;

	spi->mode = SPI_MODE_1;
	spi->bits_per_word = 16;
	/* The max speed might've been defined by the board-specific
	 * struct */
	if (!spi->max_speed_hz)
		spi->max_speed_hz = TSC2301_HZ;
	spi_setup(spi);

	/* Soft reset */
	tsc2301_write_reg(tsc, TSC2301_REG_RESET, 0xbb00);
	msleep(1);

	w = tsc2301_read_reg(tsc, TSC2301_REG_ADC);
	if (!(w & (1 << 14))) {
		dev_err(&spi->dev, "invalid ADC reg value: %04x\n", w);
		r = -ENODEV;
		goto err1;
	}

	w = tsc2301_read_reg(tsc, TSC2301_REG_DAC);
	if (!(w & (1 << 15))) {
		dev_err(&spi->dev, "invalid DAC reg value: %04x\n", w);
		r = -ENODEV;
		goto err1;
	}

	/* Stop keypad scanning */
	tsc2301_write_reg(tsc, TSC2301_REG_KEY, 0x4000);

	/* We have to cache this for read-modify-write, since we can't
	 * read back BIT15 */
	w = tsc2301_read_reg(tsc, TSC2301_REG_CONFIG2);
	/* By default BIT15 is set */
	w |= 1 << 15;
	tsc->config2_shadow = w;

	r = tsc2301_kp_init(tsc, pdata);
	if (r)
		goto err1;
	r = tsc2301_ts_init(tsc, pdata);
	if (r)
		goto err2;
	return 0;

	tsc2301_ts_exit(tsc);
err2:
	tsc2301_kp_exit(tsc);
err1:
	kfree(tsc);
	return r;
}

static int __devexit tsc2301_remove(struct spi_device *spi)
{
	struct tsc2301 *tsc = dev_get_drvdata(&spi->dev);

	tsc2301_ts_exit(tsc);
	tsc2301_kp_exit(tsc);
	if (tsc->reset_gpio >= 0)
		gpio_free(tsc->reset_gpio);
	kfree(tsc);

	return 0;
}

#ifdef CONFIG_PM
static int tsc2301_suspend(struct spi_device *spi, pm_message_t mesg)
{
	struct tsc2301 *tsc = dev_get_drvdata(&spi->dev);
	int r;

	if ((r = tsc2301_kp_suspend(tsc)) < 0)
		return r;
	if ((r = tsc2301_ts_suspend(tsc)) < 0) {
		tsc2301_kp_resume(tsc);
		return r;
	}

	return 0;
}

static int tsc2301_resume(struct spi_device *spi)
{
	struct tsc2301 *tsc = dev_get_drvdata(&spi->dev);

	tsc2301_ts_resume(tsc);
	tsc2301_kp_resume(tsc);

	return 0;
}
#endif

static struct spi_driver tsc2301_driver = {
	.driver = {
		   .name = "tsc2301",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
	},
#ifdef CONFIG_PM
	.suspend = tsc2301_suspend,
	.resume = tsc2301_resume,
#endif
	.probe = tsc2301_probe,
	.remove = __devexit_p(tsc2301_remove),
};

static int __init tsc2301_init(void)
{
	printk("TSC2301 driver initializing\n");

	return spi_register_driver(&tsc2301_driver);
}
module_init(tsc2301_init);

static void __exit tsc2301_exit(void)
{
	spi_unregister_driver(&tsc2301_driver);
}
module_exit(tsc2301_exit);

MODULE_AUTHOR("Juha Yrjölä <juha.yrjola@nokia.com>");
MODULE_LICENSE("GPL");
