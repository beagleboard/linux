/*
 * Support functions for OMAP STI/XTI (Serial Tracing Interface)
 *
 * Copyright (C) 2004, 2005, 2006 Nokia Corporation
 * Written by: Paul Mundt <paul.mundt@nokia.com>
 *
 * STI initialization code and channel handling
 * from Juha Yrjölä <juha.yrjola@nokia.com>.
 *
 * XTI initialization
 * from Roman Tereshonkov <roman.tereshonkov@nokia.com>.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <mach/sti.h>
#include <asm/byteorder.h>

static struct clk *sti_ck;
void __iomem *sti_base, *sti_channel_base;
static unsigned long sti_kern_mask = STIEn;
static unsigned long sti_irq_mask = STI_IRQSTATUS_MASK;
static DEFINE_SPINLOCK(sti_lock);

static struct sti_irqdesc {
	irqreturn_t (*func)(unsigned long);
	unsigned long data;
} ____cacheline_aligned sti_irq_desc[STI_NR_IRQS];

void sti_channel_write_trace(int len, int id, void *data, unsigned int channel)
{
	const u8 *tpntr = data;

	sti_channel_writeb(id, channel);

	if (cpu_is_omap16xx())
		/* Check u32 boundary */
		if (!((u32)data & (STI_PERCHANNEL_SIZE - 1)) &&
		     (len >= STI_PERCHANNEL_SIZE)) {
			const u32 *asrc = data;

			do {
				sti_channel_writel(cpu_to_be32(*asrc++),
						   channel);
				len -= STI_PERCHANNEL_SIZE;
			} while (len >= STI_PERCHANNEL_SIZE);

			tpntr = (const u8 *)asrc;
		}

	while (len--)
		sti_channel_writeb(*tpntr++, channel);

	sti_channel_flush(channel);
}
EXPORT_SYMBOL(sti_channel_write_trace);

void sti_enable_irq(unsigned int id)
{
	spin_lock_irq(&sti_lock);
	sti_writel(1 << id, STI_IRQSETEN);
	spin_unlock_irq(&sti_lock);
}
EXPORT_SYMBOL(sti_enable_irq);

void sti_disable_irq(unsigned int id)
{
	spin_lock_irq(&sti_lock);

	if (cpu_is_omap16xx())
		sti_writel(1 << id, STI_IRQCLREN);
	else if (cpu_is_omap24xx())
		sti_writel(sti_readl(STI_IRQSETEN) & ~(1 << id), STI_IRQSETEN);
	else
		BUG();

	spin_unlock_irq(&sti_lock);
}
EXPORT_SYMBOL(sti_disable_irq);

void sti_ack_irq(unsigned int id)
{
	/* Even though the clear state is 0, we have to write 1 to clear */
	sti_writel(1 << id, STI_IRQSTATUS);
}
EXPORT_SYMBOL(sti_ack_irq);

int sti_request_irq(unsigned int irq, void *handler, unsigned long arg)
{
	struct sti_irqdesc *desc;

	if (unlikely(!handler || irq > STI_NR_IRQS))
		return -EINVAL;

	desc = sti_irq_desc + irq;
	if (unlikely(desc->func)) {
		printk(KERN_WARNING "STI: Attempting to request in-use IRQ "
				    "%d, consider fixing your code..\n", irq);
		return -EBUSY;
	}

	desc->func = handler;
	desc->data = arg;

	sti_enable_irq(irq);
	return 0;
}
EXPORT_SYMBOL(sti_request_irq);

void sti_free_irq(unsigned int irq)
{
	struct sti_irqdesc *desc = sti_irq_desc + irq;

	if (unlikely(irq > STI_NR_IRQS))
		return;

	sti_disable_irq(irq);

	desc->func = NULL;
	desc->data = 0;
}
EXPORT_SYMBOL(sti_free_irq);

/*
 * This is a bit heavy, so normally we would defer this to a tasklet.
 * Unfortunately tasklets are too slow for the RX FIFO interrupt (and
 * possibly some others), so we just do the irqdesc walking here.
 */
static irqreturn_t sti_interrupt(int irq, void *dev_id)
{
	int ret = IRQ_NONE;
	u16 status;
	int i;

	status = sti_readl(STI_IRQSTATUS) & sti_irq_mask;

	for (i = 0; status; i++) {
		struct sti_irqdesc *desc = sti_irq_desc + i;
		u16 id = 1 << i;

		if (!(status & id))
			continue;

		if (likely(desc && desc->func))
			ret |= desc->func(desc->data);
		if (unlikely(ret == IRQ_NONE)) {
			printk("STI: spurious interrupt (id %d)\n", id);
			sti_disable_irq(i);
			sti_ack_irq(i);
			ret = IRQ_HANDLED;
		}

		status &= ~id;
	}

	return IRQ_RETVAL(ret);
}

static void omap_sti_reset(void)
{
	int i;

	/* Reset STI module */
	sti_writel(0x02, STI_SYSCONFIG);

	/* Wait a while for the STI module to complete its reset */
	for (i = 0; i < 10000; i++)
		if (sti_readl(STI_SYSSTATUS) & 1)
			break;
}

static int __init sti_init(void)
{
	char buf[64];
	int i;

	if (cpu_is_omap16xx()) {
		/* Release ARM Rhea buses peripherals enable */
		sti_writel(sti_readl(ARM_RSTCT2) | 0x0001, ARM_RSTCT2);

		/* Enable TC1_CK (functional clock) */
		sti_ck = clk_get(NULL, "tc1_ck");
	} else if (cpu_is_omap24xx())
		/* Enable emulation tools clock */
		sti_ck = clk_get(NULL, "emul_ck");

	if (IS_ERR(sti_ck))
		return PTR_ERR(sti_ck);

	clk_enable(sti_ck);

	/* Reset STI module */
	omap_sti_reset();

	/* Enable STI */
	sti_trace_enable(MPUCmdEn);

	/* Change to custom serial protocol */
	sti_writel(0x01, STI_SERIAL_CFG);

	/* Set STI clock control register to normal mode */
	sti_writel(0x00, STI_CLK_CTRL);

	i = sti_readl(STI_REVISION);
	snprintf(buf, sizeof(buf), "OMAP STI support loaded (HW v%u.%u)\n",
	        (i >> 4) & 0x0f, i & 0x0f);
	printk(KERN_INFO "%s", buf);

	sti_channel_write_trace(strlen(buf), 0xc3, buf, 239);

	return 0;
}

static void sti_exit(void)
{
	u32 tmp;

	/*
	 * This should have already been done by reset, but we switch off
	 * STI entirely just for added sanity..
	 */
	tmp = sti_readl(STI_ER);
	tmp &= ~STIEn;
	sti_writel(tmp, STI_ER);

	clk_disable(sti_ck);
	clk_put(sti_ck);
}

static void __sti_trace_enable(int event)
{
	u32 tmp;

	tmp = sti_readl(STI_ER);
	tmp |= sti_kern_mask | event;
	sti_writel(tmp, STI_ER);
}

int sti_trace_enable(int event)
{
	spin_lock_irq(&sti_lock);
	sti_kern_mask |= event;
	__sti_trace_enable(event);
	spin_unlock_irq(&sti_lock);

	return 0;
}
EXPORT_SYMBOL(sti_trace_enable);

static void __sti_trace_disable(int event)
{
	u32 tmp;

	tmp = sti_readl(STI_DR);

	if (cpu_is_omap16xx()) {
		tmp |= event;
		tmp &= ~sti_kern_mask;
	} else if (cpu_is_omap24xx()) {
		tmp &= ~event;
		tmp |= sti_kern_mask;
	} else
		BUG();

	sti_writel(tmp, STI_DR);
}

void sti_trace_disable(int event)
{
	spin_lock_irq(&sti_lock);
	sti_kern_mask &= ~event;
	__sti_trace_disable(event);
	spin_unlock_irq(&sti_lock);
}
EXPORT_SYMBOL(sti_trace_disable);

static ssize_t
sti_trace_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%08lx\n", sti_readl(STI_ER));
}

static ssize_t
sti_trace_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int evt = simple_strtoul(buf, NULL, 0);
	int mask = ~evt;

	spin_lock_irq(&sti_lock);
	__sti_trace_disable(mask);
	__sti_trace_enable(evt);
	spin_unlock_irq(&sti_lock);

	return count;
}
static DEVICE_ATTR(trace, S_IRUGO | S_IWUSR, sti_trace_show, sti_trace_store);

static ssize_t
sti_imask_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%04lx\n", sti_irq_mask);
}

static ssize_t
sti_imask_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	spin_lock_irq(&sti_lock);
	sti_irq_mask = simple_strtoul(buf, NULL, 0);
	spin_unlock_irq(&sti_lock);

	return count;
}
static DEVICE_ATTR(imask, S_IRUGO | S_IWUSR, sti_imask_show, sti_imask_store);

static int __devinit sti_probe(struct platform_device *pdev)
{
	struct resource *res, *cres;
	unsigned int size;
	int ret;

	if (pdev->num_resources != 3) {
		dev_err(&pdev->dev, "invalid number of resources: %d\n",
			pdev->num_resources);
		return -ENODEV;
	}

	/* STI base */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!res)) {
		dev_err(&pdev->dev, "invalid mem resource\n");
		return -ENODEV;
	}

	/* Channel base */
	cres = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (unlikely(!cres)) {
		dev_err(&pdev->dev, "invalid channel mem resource\n");
		return -ENODEV;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_trace);
	if (unlikely(ret != 0))
		return ret;

	ret = device_create_file(&pdev->dev, &dev_attr_imask);
	if (unlikely(ret != 0))
		goto err;

	size = res->end - res->start + 1;
	sti_base = ioremap(res->start, size);
	if (!sti_base) {
		ret = -ENOMEM;
		goto err_badremap;
	}

	size = cres->end - cres->start + 1;
	sti_channel_base = ioremap(cres->start, size);
	if (!sti_channel_base) {
		iounmap(sti_base);
		ret = -ENOMEM;
		goto err_badremap;
	}

	ret = request_irq(platform_get_irq(pdev, 0), sti_interrupt,
			  IRQF_DISABLED, "sti", NULL);
	if (unlikely(ret != 0))
		goto err_badirq;

	return sti_init();

err_badirq:
	iounmap(sti_channel_base);
	iounmap(sti_base);
err_badremap:
	device_remove_file(&pdev->dev, &dev_attr_imask);
err:
	device_remove_file(&pdev->dev, &dev_attr_trace);

	return ret;
}

static int __devexit sti_remove(struct platform_device *pdev)
{
	unsigned int irq = platform_get_irq(pdev, 0);

	iounmap(sti_channel_base);
	iounmap(sti_base);

	device_remove_file(&pdev->dev, &dev_attr_trace);
	device_remove_file(&pdev->dev, &dev_attr_imask);
	free_irq(irq, NULL);
	sti_exit();

	return 0;
}

static struct platform_driver sti_driver = {
	.probe		= sti_probe,
	.remove		= __devexit_p(sti_remove),
	.driver		= {
		.name	= "sti",
		.owner	= THIS_MODULE,
	},
};

static int __init sti_module_init(void)
{
	return platform_driver_register(&sti_driver);
}

static void __exit sti_module_exit(void)
{
	platform_driver_unregister(&sti_driver);
}
subsys_initcall(sti_module_init);
module_exit(sti_module_exit);

MODULE_AUTHOR("Paul Mundt, Juha Yrjölä, Roman Tereshonkov");
MODULE_LICENSE("GPL");
