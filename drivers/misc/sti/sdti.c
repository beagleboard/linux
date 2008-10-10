/*
 * Support functions for OMAP3 SDTI (Serial Debug Tracing Interface)
 *
 * Copyright (C) 2008 Nokia Corporation
 * Written by: Roman Tereshonkov <roman.tereshonkov@nokia.com>
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
#include <asm/io.h>

#define SDTI_REVISION		0x000
#define SDTI_SYSCONFIG		0x010
#define SDTI_SYSSTATUS		0x014
#define SDTI_WINCTRL		0x024
#define SDTI_SCONFIG		0x028
#define SDTI_TESTCTRL		0x02C
#define SDTI_LOCK_ACCESS	0xFB0

#define CPU1_TRACE_EN		0x01
#define CPU2_TRACE_EN		0x02

static struct clk *sdti_ck;
void __iomem *sti_base, *sti_channel_base;
static DEFINE_SPINLOCK(sdti_lock);

void omap_sti_channel_write_trace(int len, int id, void *data,
				unsigned int channel)
{
	const u8 *tpntr = data;

	spin_lock_irq(&sdti_lock);

	sti_channel_writeb(id, channel);
	while (len--)
		sti_channel_writeb(*tpntr++, channel);
	sti_channel_flush(channel);

	spin_unlock_irq(&sdti_lock);
}
EXPORT_SYMBOL(omap_sti_channel_write_trace);

static void omap_sdti_reset(void)
{
	int i;

	sti_writel(0x02, SDTI_SYSCONFIG);

	for (i = 0; i < 10000; i++)
		if (sti_readl(SDTI_SYSSTATUS) & 1)
			break;
	if (i == 10000)
		printk(KERN_WARNING "XTI: no real reset\n");
}

static int __init omap_sdti_init(void)
{
	char buf[64];
	int i;

	sdti_ck = clk_get(NULL, "emu_per_alwon_ck");
	if (IS_ERR(sdti_ck)) {
		printk(KERN_ERR "Cannot get clk emu_per_alwon_ck\n");
		return PTR_ERR(sdti_ck);
	}
	clk_enable(sdti_ck);

	omap_sdti_reset();
	sti_writel(0xC5ACCE55, SDTI_LOCK_ACCESS);

	/* Claim SDTI */
	sti_writel(1 << 30, SDTI_WINCTRL);
	i = sti_readl(SDTI_WINCTRL);
	if (!(i & (1 << 30)))
		printk(KERN_WARNING "SDTI: cannot claim SDTI\n");

	/* 4 bits dual, fclk/3 */
	sti_writel(0x43, SDTI_SCONFIG);

	/* CPU2 trace enable */
	sti_writel(i | CPU2_TRACE_EN, SDTI_WINCTRL);
	i = sti_readl(SDTI_WINCTRL);

	/* Enable SDTI */
	sti_writel((1 << 31) | (i & 0x3FFFFFFF), SDTI_WINCTRL);

	i = sti_readl(SDTI_REVISION);
	snprintf(buf, sizeof(buf), "OMAP SDTI support loaded (HW v%u.%u)\n",
		(i >> 4) & 0x0f, i & 0x0f);
	printk(KERN_INFO "%s", buf);
	omap_sti_channel_write_trace(strlen(buf), 0xc3, buf, 239);

	return 0;
}

static void omap_sdti_exit(void)
{
	sti_writel(0, SDTI_WINCTRL);
	clk_disable(sdti_ck);
	clk_put(sdti_ck);
}

static int __devinit omap_sdti_probe(struct platform_device *pdev)
{
	struct resource *res, *cres;
	unsigned int size;

	if (pdev->num_resources != 2) {
		dev_err(&pdev->dev, "invalid number of resources: %d\n",
			pdev->num_resources);
		return -ENODEV;
	}

	/* SDTI base */
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

	size = res->end - res->start;
	sti_base = ioremap(res->start, size);
	if (unlikely(!sti_base))
		return -ENODEV;

	size = cres->end - cres->start;
	sti_channel_base = ioremap(cres->start, size);
	if (unlikely(!sti_channel_base)) {
		iounmap(sti_base);
		return -ENODEV;
	}

	return omap_sdti_init();
}

static int __devexit omap_sdti_remove(struct platform_device *pdev)
{
	iounmap(sti_channel_base);
	iounmap(sti_base);
	omap_sdti_exit();

	return 0;
}

static struct platform_driver omap_sdti_driver = {
	.probe		= omap_sdti_probe,
	.remove		= __devexit_p(omap_sdti_remove),
	.driver		= {
		.name	= "sti",
		.owner	= THIS_MODULE,
	},
};

static int __init omap_sdti_module_init(void)
{
	return platform_driver_register(&omap_sdti_driver);
}

static void __exit omap_sdti_module_exit(void)
{
	platform_driver_unregister(&omap_sdti_driver);
}
subsys_initcall(omap_sdti_module_init);
module_exit(omap_sdti_module_exit);

MODULE_AUTHOR("Roman Tereshonkov");
MODULE_LICENSE("GPL");
