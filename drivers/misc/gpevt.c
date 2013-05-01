/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>

#define GPEVT_MAGIC 0xdeadbeef

static u32 *dst_fifo;
static	dma_addr_t fifo_addr;

static void gpevt_callback(void *data)
{
	struct device *dev = data;

	dma_unmap_single(dev, fifo_addr, 32, DMA_FROM_DEVICE);

	if (*dst_fifo == GPEVT_MAGIC)
		dev_info(dev, "*** DMA transfer succeeded ***\n");
	else
		dev_info(dev, "*** DMA transfer failed ***\n");
}

static int gpevt_probe (struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct pinctrl *pinctrl;
	struct dma_chan *chan;
	struct dma_slave_config cfg;
	struct dma_async_tx_descriptor *tx;
	int gpio_evt = 0;
	int ret;
	u32 *src_buf;
	struct scatterlist sg;

	src_buf = devm_kzalloc(&pdev->dev, 32, GFP_KERNEL);
	if (!src_buf) {
		dev_err(&pdev->dev, "failed to allocate src buffer\n");
		return -ENOMEM;
	}

	dst_fifo = devm_kzalloc(&pdev->dev, 32, GFP_KERNEL);
	if (!dst_fifo) {
		dev_err(&pdev->dev, "failed to allocate dst fifo\n");
		return -ENOMEM;
	}
		
	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&pdev->dev,
			"pins are not configured from the driver\n");

        gpio_evt = of_get_named_gpio(np, "gpio-evt", 0);
	if (gpio_evt < 0) {
		dev_err(&pdev->dev, "failed to find gpio event signal!\n");
		return -EINVAL;
	}

        ret = devm_gpio_request_one(&pdev->dev, gpio_evt,
				    GPIOF_IN, "GPIO Event Pin");
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to claim gpio-evt pin\n");
		return ret;
	}

	ret = request_irq(gpio_to_irq(gpio_evt), no_action,
			  IRQ_TYPE_EDGE_FALLING, "gpevt", &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request falling edge irq/event\n");
		return ret;
	}

	chan = dma_request_slave_channel(&pdev->dev, "gpioevt");
	if (!chan) {
		dev_err(&pdev->dev, "no gpio channel for gpevt\n");
		return -EAGAIN;
	}

	fifo_addr = dma_map_single(&pdev->dev, dst_fifo, 32, DMA_FROM_DEVICE);
	if (!fifo_addr) {
		dev_err(&pdev->dev, "could not map dst fifo\n");
		return -EIO;
	}
	cfg.dst_addr = fifo_addr;
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_maxburst = 1;

	ret = dmaengine_slave_config(chan, &cfg);
	if (ret)
		return ret;

	*src_buf = GPEVT_MAGIC;
	sg_init_table(&sg, 1);
	sg_dma_address(&sg) = dma_map_single(&pdev->dev, src_buf, 32, DMA_TO_DEVICE);
	sg_dma_len(&sg) = 4;

	tx = dmaengine_prep_slave_sg(chan, &sg, 1, DMA_MEM_TO_DEV,
				     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx) {
		dev_err(&pdev->dev, "prep_slave_sg() failed\n");
		return -EIO;
	}

	tx->callback = gpevt_callback;
	tx->callback_param = &pdev->dev;
	dmaengine_submit(tx);

	dma_async_issue_pending(chan);
		
	dev_info(&pdev->dev, "Amazing GPIO DMA Event Test Driver(tm) engaged\n");

	return 0;
}

static int gpevt_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id gpevt_dt_ids[] = {
	{ .compatible = "gpevt", .data = (void *) NULL, },
};
MODULE_DEVICE_TABLE(of, gpevt_dt_ids);

static struct platform_driver gpevt_driver = {
	.driver = {
		.name   = "gpevt",
		.owner  = THIS_MODULE,
		.of_match_table = gpevt_dt_ids,
	},
	.probe  = gpevt_probe,
	.remove = gpevt_remove,
};

static int __init gpevt_init(void)
{
	return platform_driver_register(&gpevt_driver);
}

static void __exit gpevt_exit(void)
{
	platform_driver_unregister(&gpevt_driver);
}

/* ------------------------------------------------------------------------- */

module_init(gpevt_init);
module_exit(gpevt_exit);

MODULE_DESCRIPTION("Amazing GPIO DMA Event Test Driver(tm)");
MODULE_AUTHOR("Matt Porter <mporter@ti.com>");
MODULE_LICENSE("GPL");
