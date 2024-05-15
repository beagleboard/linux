// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#include <linux/crc7.h>
#include <linux/spi/spi.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>

#include "wlcore.h"
#include "io.h"


enum {
	WSPI_CMD_READ				= 0x40000000,
	WSPI_CMD_WRITE				= 0x00000000,
	WSPI_CMD_FIXED				= 0x20000000,
	WSPI_CMD_BYTE_LENGTH		= 0x1FFE0000,
	WSPI_CMD_BYTE_LENGTH_OFFSET	= 17,
	WSPI_CMD_BYTE_ADDR			= 0x0001FFFF
};

enum {
	WSPI_INIT_CMD_CRC_LEN		= 5,
	WSPI_INIT_CMD_START			= 0x00,
	WSPI_INIT_CMD_TX			= 0x40,
	WSPI_INIT_CMD_FIXEDBUSY_LEN	= 0x07,
	WSPI_INIT_CMD_EN_FIXEDBUSY	= 0x80,
	WSPI_INIT_CMD_DIS_FIXEDBUSY	= 0x00,
	WSPI_INIT_CMD_OPS			= 0x08,
	WSPI_INIT_CMD_IOD			= 0x40,
	WSPI_INIT_CMD_IP			= 0x20,
	WSPI_INIT_CMD_CS			= 0x10,
	WSPI_INIT_CMD_WS			= 0x08,
	WSPI_INIT_CMD_WSPI			= 0x01,
	WSPI_INIT_CMD_END			= 0x01,
	WSPI_INIT_CMD_LEN			= 8,
};

#define HW_ACCESS_WSPI_FIXED_BUSY_LEN \
		((CC33XX_BUSY_WORD_LEN - 4) / sizeof(u32))
#define HW_ACCESS_WSPI_INIT_CMD_MASK  	0

/* HW limitation: maximum possible chunk size is 4095 bytes */
/* Actual size will have to be 32 bit aligned */
#define WSPI_MAX_CHUNK_SIZE		4092

static const struct cc33xx_family_data cc33xx_data = {
	.name = "cc33xx",
	.cfg_name = "ti-connectivity/cc33xx-conf.bin",
	.nvs_name = "ti-connectivity/cc33xx-nvs.bin",
};

struct cc33xx_spi_glue {
	struct device *dev;
	struct platform_device *core;
	struct regulator *reg; /* Power regulator */
	pid_t locking_pid;
	int lock_count;
};

/* Must be allocated from DMA-safe memory */
struct spi_transaction_buffers {
	u32 spi_cmd;
	u32 busyword;
};

static void __cc33xx_spi_lock(struct cc33xx_spi_glue *glue)
{
	if (glue->locking_pid != current->pid)
	{
		spi_bus_lock(to_spi_device(glue->dev)->master);
		glue->locking_pid = current->pid;
		glue->lock_count = 1;
	} else {
		glue->lock_count++;
	}
}

static void __cc33xx_spi_unlock(struct cc33xx_spi_glue *glue)
{
	BUG_ON(glue->locking_pid != current->pid);
	BUG_ON(!glue->lock_count);

	glue->lock_count--;
	if (!glue->lock_count){
		glue->locking_pid = 0;
		spi_bus_unlock(to_spi_device(glue->dev)->master);
	}
}

static void cc33xx_spi_reset(struct device *child)
{
	struct cc33xx_spi_glue *glue = dev_get_drvdata(child->parent);
	u8 *cmd;
	struct spi_transfer t;
	struct spi_message m;

	cmd = kzalloc(WSPI_INIT_CMD_LEN, GFP_KERNEL);
	if (!cmd) {
		dev_err(child->parent,
			"could not allocate cmd for spi reset\n");
		return;
	}

	memset(&t, 0, sizeof(t));
	spi_message_init(&m);

	memset(cmd, 0xff, WSPI_INIT_CMD_LEN);

	t.tx_buf = cmd;
	t.len = WSPI_INIT_CMD_LEN;
	spi_message_add_tail(&t, &m);

	spi_sync(to_spi_device(glue->dev), &m);

	kfree(cmd);
}

static void cc33xx_spi_init(struct device *child)
{
	struct cc33xx_spi_glue *glue = dev_get_drvdata(child->parent);
	struct spi_transfer t;
	struct spi_message m;
	struct spi_device *spi = to_spi_device(glue->dev);
	u8 *cmd = kzalloc(WSPI_INIT_CMD_LEN, GFP_KERNEL);

	if (!cmd) {
		dev_err(child->parent,
			"could not allocate cmd for spi init\n");
		return;
	}

	memset(&t, 0, sizeof(t));
	spi_message_init(&m);

	/*
	 * Set WSPI_INIT_COMMAND
	 * the data is being send from the MSB to LSB
	 */
	cmd[0] = 0xff;
	cmd[1] = 0xff;
	cmd[2] = WSPI_INIT_CMD_START | WSPI_INIT_CMD_TX;
	cmd[3] = 0;
	cmd[4] = 0;
	cmd[5] = HW_ACCESS_WSPI_INIT_CMD_MASK << 3;
	cmd[5] |= HW_ACCESS_WSPI_FIXED_BUSY_LEN & WSPI_INIT_CMD_FIXEDBUSY_LEN;
	cmd[5] |= WSPI_INIT_CMD_OPS;

	cmd[6] = WSPI_INIT_CMD_IOD | WSPI_INIT_CMD_IP | WSPI_INIT_CMD_CS
		| WSPI_INIT_CMD_WSPI | WSPI_INIT_CMD_WS;

	if (HW_ACCESS_WSPI_FIXED_BUSY_LEN == 0)
		cmd[6] |= WSPI_INIT_CMD_DIS_FIXEDBUSY;
	else
		cmd[6] |= WSPI_INIT_CMD_EN_FIXEDBUSY;

	cmd[7] = crc7_be(0, cmd+2, WSPI_INIT_CMD_CRC_LEN) | WSPI_INIT_CMD_END;

	/*
	 * The above is the logical order; it must actually be stored
	 * in the buffer byte-swapped.
	 */
	__swab32s((u32 *)cmd);
	__swab32s((u32 *)cmd+1);

	t.tx_buf = cmd;
	t.len = WSPI_INIT_CMD_LEN;
	spi_message_add_tail(&t, &m);

	spi_sync(to_spi_device(glue->dev), &m);

	/* Send extra clocks with inverted CS (high). this is required
	 * by the wilink family in order to successfully enter WSPI mode.
	 */
	spi->mode ^= SPI_CS_HIGH;
	memset(&m, 0, sizeof(m));
	spi_message_init(&m);

	cmd[0] = 0xff;
	cmd[1] = 0xff;
	cmd[2] = 0xff;
	cmd[3] = 0xff;
	__swab32s((u32 *)cmd);

	t.tx_buf = cmd;
	t.len = 4;
	spi_message_add_tail(&t, &m);

	spi_sync(to_spi_device(glue->dev), &m);

	/* Restore chip select configration to normal */
	spi->mode ^= SPI_CS_HIGH;
	kfree(cmd);
}

#define CC33XX_BUSY_READ_TIMEOUT_MSEC 50

static int cc33xx_spi_read_busy(struct device *child, u32 *busy_buf)
{
	struct cc33xx_spi_glue *glue = dev_get_drvdata(child->parent);
	unsigned long end_time = jiffies + msecs_to_jiffies(CC33XX_BUSY_READ_TIMEOUT_MSEC);
	bool read_timeout = false;
	struct spi_transfer t[1];
	struct spi_message m;

	/*
	 * Read further busy words from SPI until a non-busy word is
	 * encountered, then read the data itself into the buffer.
	 */

	while (!read_timeout) {
		read_timeout = time_is_before_eq_jiffies(end_time);
		spi_message_init(&m);
		memset(t, 0, sizeof(t));
		t[0].rx_buf = busy_buf;
		t[0].len = sizeof(u32);
		t[0].cs_change = true;
		spi_message_add_tail(&t[0], &m);
		spi_sync_locked(to_spi_device(glue->dev), &m);

		if (*busy_buf & 0x1)
			return 0;
	}

	/* The SPI bus is unresponsive, the read failed. */
	dev_err(child->parent, "SPI read busy-word timeout!\n");
	return -ETIMEDOUT;
}

static int __must_check cc33xx_spi_raw_read(struct device *child, int addr,
					    void *buf, size_t len, bool fixed)
{
	struct cc33xx_spi_glue *glue = dev_get_drvdata(child->parent);
	struct spi_transaction_buffers* txn_buffers;
	struct spi_transfer t[2];
	struct spi_message m;
	int ret;
	u32 *busy_buf;
	u32 *cmd;

	if (unlikely(len > WSPI_MAX_CHUNK_SIZE)){
		WARN_ON(1);
		return -EFAULT;
	}

	txn_buffers = kzalloc(sizeof (*txn_buffers), GFP_KERNEL);
	if (!txn_buffers)
		return -ENOMEM;

	__cc33xx_spi_lock(glue);

	cmd = &txn_buffers->spi_cmd;
	busy_buf = &txn_buffers->busyword;

	*cmd = 0;
	*cmd |= WSPI_CMD_READ;
	*cmd |= (len << WSPI_CMD_BYTE_LENGTH_OFFSET) &
		WSPI_CMD_BYTE_LENGTH;
	*cmd |= addr & WSPI_CMD_BYTE_ADDR;

	if (fixed)
		*cmd |= WSPI_CMD_FIXED;

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	t[0].tx_buf = cmd;
	t[0].len = 4;
	t[0].cs_change = false;
	spi_message_add_tail(&t[0], &m);

	/* Busy and non busy words read */
	t[1].rx_buf = busy_buf;
	t[1].len = CC33XX_BUSY_WORD_LEN;
	t[1].cs_change = false;
	spi_message_add_tail(&t[1], &m);

	spi_sync_locked(to_spi_device(glue->dev), &m);

	if (unlikely((*busy_buf & 0x1) == 0)){
		if( cc33xx_spi_read_busy(child, busy_buf) != 0){
			memset(buf, 0, len);
			ret = -EIO;
			goto out;
		}
	}

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	t[0].rx_buf = buf;
	t[0].len = len;
	t[0].cs_change = false;
	spi_message_add_tail(&t[0], &m);

	spi_sync_locked(to_spi_device(glue->dev), &m);

	ret=0;

out:
	__cc33xx_spi_unlock(glue);
	kfree(txn_buffers);
	return ret;
}

static int __cc33xx_spi_raw_write(struct device *child, int addr,
				  void *buf, size_t len, bool fixed)
{
	struct cc33xx_spi_glue *glue = dev_get_drvdata(child->parent);
	struct spi_transfer t[2];
	struct spi_transaction_buffers* txn_buffers;
	struct spi_message m;
	u32 *cmd;
	u32 *busy_buf;
	u32 chunk_len;
	int ret;

	txn_buffers = kzalloc(sizeof (*txn_buffers), GFP_KERNEL);
	if (!txn_buffers)
		return -ENOMEM;

	cmd = &txn_buffers->spi_cmd;
	busy_buf = &txn_buffers->busyword;

	__cc33xx_spi_lock(glue);

	while (len > 0) {
		chunk_len = min_t(size_t, WSPI_MAX_CHUNK_SIZE, len);

		*cmd = 0;
		*cmd |= WSPI_CMD_WRITE;
		*cmd |= (chunk_len << WSPI_CMD_BYTE_LENGTH_OFFSET) &
			WSPI_CMD_BYTE_LENGTH;
		*cmd |= addr & WSPI_CMD_BYTE_ADDR;

		if (fixed)
			*cmd |= WSPI_CMD_FIXED;

		spi_message_init(&m);
		memset(t, 0, sizeof(t));

		t[0].tx_buf = cmd;
		t[0].len = 4;
		t[0].cs_change = false;
		spi_message_add_tail(&t[0], &m);

		/* Busy and non busy words read */
		t[1].rx_buf = busy_buf;
		t[1].len = CC33XX_BUSY_WORD_LEN;
		t[1].cs_change = false;
		spi_message_add_tail(&t[1], &m);

		spi_sync_locked(to_spi_device(glue->dev), &m);

		if (unlikely((*busy_buf & 0x1) == 0)){
			if( cc33xx_spi_read_busy(child, busy_buf) != 0){
				memset(buf, 0, chunk_len);
				ret = -EIO;
				goto out;
			}
		}

		spi_message_init(&m);
		memset(t, 0, sizeof(t));

		t[0].tx_buf = buf;
		t[0].len = chunk_len;
		t[0].cs_change = false;
		spi_message_add_tail(&t[0], &m);

		spi_sync_locked(to_spi_device(glue->dev), &m);

		if (!fixed)
			addr += chunk_len;
		buf += chunk_len;
		len -= chunk_len;
	}

	ret=0;

out:
	__cc33xx_spi_unlock(glue);
	kfree(txn_buffers);
	return ret;
}

static inline int __must_check cc33xx_spi_raw_write(struct device *child,
						    int addr, void *buf,
						    size_t len, bool fixed)
{
	return __cc33xx_spi_raw_write(child, addr, buf, len, fixed);
}

/**
 * cc33xx_spi_set_power - power on/off the cc33xx unit
 * @child: cc33xx device handle.
 * @enable: true/false to power on/off the unit.
 *
 * use the WiFi enable regulator to enable/disable the WiFi unit.
 */
static int cc33xx_spi_set_power(struct device *child, bool enable)
{
	int ret = 0;
	struct cc33xx_spi_glue *glue = dev_get_drvdata(child->parent);

	WARN_ON(!glue->reg);

	/* Update regulator state */
	if (enable) {
		ret = regulator_enable(glue->reg);
		if (ret)
			dev_err(child, "Power enable failure\n");
	} else {
		ret =  regulator_disable(glue->reg);
		if (ret)
			dev_err(child, "Power disable failure\n");
	}

	return ret;
}

/**
 * cc33xx_spi_set_block_size
 *
 * This function is not needed for spi mode, but need to be present.
 * Without it defined the wlcore fallback to use the wrong packet
 * allignment on tx.
 */
static void cc33xx_spi_set_block_size(struct device *child,
				      unsigned int blksz)
{
}

static size_t cc33xx_spi_get_max_transfer_len(struct device *child)
{
	return WSPI_MAX_CHUNK_SIZE;
}

static void cc33xx_spi_set_irq_handler(struct device *child, void* irq_handler)
{
	struct cc33xx_spi_glue *glue = dev_get_drvdata(child->parent);
	struct platform_device *pdev = glue->core;
	struct wlcore_platdev_data *pdev_data = dev_get_platdata(&pdev->dev);

	pdev_data->irq_handler = irq_handler;
}

static void cc33xx_spi_enable_irq (struct device *child)
{
	struct cc33xx_spi_glue *glue = dev_get_drvdata(child->parent);
	struct platform_device *pdev = glue->core;
	struct wlcore_platdev_data *pdev_data = dev_get_platdata(&pdev->dev);

	enable_irq(pdev_data->gpio_irq_num);
}

static void cc33xx_spi_disable_irq (struct device *child)
{
	struct cc33xx_spi_glue *glue = dev_get_drvdata(child->parent);
	struct platform_device *pdev = glue->core;
	struct wlcore_platdev_data *pdev_data = dev_get_platdata(&pdev->dev);

	disable_irq(pdev_data->gpio_irq_num);
}

static void cc33xx_spi_interface_claim(struct device *child)
{
	struct cc33xx_spi_glue *glue = dev_get_drvdata(child->parent);

	__cc33xx_spi_lock(glue);
}

static void cc33xx_spi_interface_release(struct device *child)
{
	struct cc33xx_spi_glue *glue = dev_get_drvdata(child->parent);

	__cc33xx_spi_unlock(glue);
}

static irqreturn_t cc33xx_spi_irq_hard_handler(int irq, void *cookie)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t cc33xx_spi_irq_handler(int irq, void *cookie)
{
	struct spi_device *spi = cookie;
	struct cc33xx_spi_glue *glue = spi_get_drvdata(spi);
	struct platform_device *pdev = glue->core;
	struct wlcore_platdev_data *pdev_data = dev_get_platdata(&pdev->dev);

	BUG_ON(!pdev_data->irq_handler);
	pdev_data->irq_handler(pdev);

	return IRQ_HANDLED;
}

static struct cc33xx_if_operations spi_ops = {
	.read				= cc33xx_spi_raw_read,
	.write				= cc33xx_spi_raw_write,
	.reset				= cc33xx_spi_reset,
	.init				= cc33xx_spi_init,
	.power				= cc33xx_spi_set_power,
	.set_block_size 		= cc33xx_spi_set_block_size,
	.get_max_transaction_len	= cc33xx_spi_get_max_transfer_len,
	.set_irq_handler		= cc33xx_spi_set_irq_handler,
	.enable_irq			= cc33xx_spi_enable_irq,
	.disable_irq			= cc33xx_spi_disable_irq,
	.interface_claim		= cc33xx_spi_interface_claim,
	.interface_release		= cc33xx_spi_interface_release,
};

static const struct of_device_id wlcore_spi_of_match_table[] = {
	{ .compatible = "ti,cc33xx", .data = &cc33xx_data},
	{ }
};
MODULE_DEVICE_TABLE(of, wlcore_spi_of_match_table);

/**
 * wlcore_probe_of - DT node parsing.
 * @spi: SPI slave device parameters.
 * @res: resource parameters.
 * @glue: cc33xx SPI bus to slave device glue parameters.
 * @pdev_data: wlcore device parameters
 */
static int wlcore_probe_of(struct spi_device *spi, struct cc33xx_spi_glue *glue,
			   int *irq, struct wlcore_platdev_data *pdev_data)
{
	struct device_node *dt_node = spi->dev.of_node;
	const struct of_device_id *of_id;

	of_id = of_match_node(wlcore_spi_of_match_table, dt_node);
	if (!of_id)
		return -ENODEV;

	pdev_data->family = of_id->data;
	dev_info(&spi->dev, "selected chip family is %s\n",
		 pdev_data->family->name);

	*irq = irq_of_parse_and_map(dt_node, 0);
	if (0 == *irq){
		dev_err(&spi->dev, "Could not parse IRQ property");
		return -ENODEV;
	}

	if (of_find_property(dt_node, "clock-xtal", NULL))
		pdev_data->ref_clock_xtal = true;

	return 0;
}

static int spi_cc33xx_probe(struct spi_device *spi)
{
	struct cc33xx_spi_glue *glue;
	struct wlcore_platdev_data *pdev_data;
	int irq;
	int ret;

	pdev_data = devm_kzalloc(&spi->dev, sizeof(*pdev_data), GFP_KERNEL);
	if (!pdev_data)
		return -ENOMEM;

	pdev_data->if_ops = &spi_ops;

	glue = devm_kzalloc(&spi->dev, sizeof(*glue), GFP_KERNEL);
	if (!glue) {
		dev_err(&spi->dev, "can't allocate glue\n");
		return -ENOMEM;
	}

	glue->dev = &spi->dev;

	spi_set_drvdata(spi, glue);

	/* This is the only SPI value that we need to set here, the rest
	 * comes from the board-peripherals file */
	spi->bits_per_word = 32;

	glue->reg = devm_regulator_get(&spi->dev, "vwlan");
	if (PTR_ERR(glue->reg) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (IS_ERR(glue->reg)) {
		dev_err(glue->dev, "can't get regulator\n");
		return PTR_ERR(glue->reg);
	}

	ret = wlcore_probe_of(spi, glue, &irq, pdev_data);
	if (ret) {
		dev_err(glue->dev,
			"can't get device tree parameters (%d)\n", ret);
		return ret;
	}

	irq_set_status_flags(irq, IRQ_NOAUTOEN);

	ret = request_threaded_irq(
		irq, cc33xx_spi_irq_hard_handler, cc33xx_spi_irq_handler,
		IRQF_TRIGGER_HIGH|IRQF_ONESHOT, pdev_data->family->name, spi);
	if (ret) {
		dev_err(glue->dev, "can't register GPIO IRQ handler\n");
		goto out_dev_put;
	}

	pdev_data->gpio_irq_num = irq;

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(glue->dev, "spi_setup failed\n");
		return ret;
	}

	glue->core = platform_device_alloc(pdev_data->family->name,
					   PLATFORM_DEVID_AUTO);
	if (!glue->core) {
		dev_err(glue->dev, "can't allocate platform_device\n");
		return -ENOMEM;
	}

	glue->core->dev.parent = &spi->dev;

	ret = platform_device_add_data(glue->core, pdev_data,
				       sizeof(*pdev_data));
	if (ret) {
		dev_err(glue->dev, "can't add platform data\n");
		goto out_dev_put;
	}

	ret = platform_device_add(glue->core);
	if (ret) {
		dev_err(glue->dev, "can't register platform device\n");
		goto out_dev_put;
	}

	return 0;

out_dev_put:
	platform_device_put(glue->core);
	return ret;
}

static void cc33xx_remove(struct spi_device *spi)
{
	struct cc33xx_spi_glue *glue = spi_get_drvdata(spi);
	struct platform_device *pdev = glue->core;
	struct wlcore_platdev_data *pdev_data = dev_get_platdata(&pdev->dev);

	platform_device_unregister(glue->core);

	free_irq(pdev_data->gpio_irq_num, spi);

	return;
}

static struct spi_driver cc33xx_spi_driver = {
	.driver = {
		.name		= "cc33xx_spi",
		.of_match_table = of_match_ptr(wlcore_spi_of_match_table),
	},

	.probe		= spi_cc33xx_probe,
	.remove		= cc33xx_remove,
};

module_spi_driver(cc33xx_spi_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Luciano Coelho <coelho@ti.com>");
MODULE_AUTHOR("Juuso Oikarinen <juuso.oikarinen@nokia.com>");
MODULE_ALIAS("spi:cc33xx");
