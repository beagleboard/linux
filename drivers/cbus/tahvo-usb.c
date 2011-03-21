/**
 * drivers/cbus/tahvo-usb.c
 *
 * Tahvo USB transeiver
 *
 * Copyright (C) 2005-2006 Nokia Corporation
 *
 * Parts copied from drivers/i2c/chips/isp1301_omap.c
 * Copyright (C) 2004 Texas Instruments
 * Copyright (C) 2004 David Brownell
 *
 * Written by Juha Yrjölä <juha.yrjola@nokia.com>,
 *	      Tony Lindgren <tony@atomide.com>, and
 *	      Timo Teräs <timo.teras@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/clk.h>
#include <linux/mutex.h>

#include <asm/irq.h>
#include <plat/usb.h>

#include "cbus.h"
#include "tahvo.h"

#define DRIVER_NAME     "tahvo-usb"

#define USBR_SLAVE_CONTROL	(1 << 8)
#define USBR_VPPVIO_SW		(1 << 7)
#define USBR_SPEED		(1 << 6)
#define USBR_REGOUT		(1 << 5)
#define USBR_MASTER_SW2		(1 << 4)
#define USBR_MASTER_SW1		(1 << 3)
#define USBR_SLAVE_SW		(1 << 2)
#define USBR_NSUSPEND		(1 << 1)
#define USBR_SEMODE		(1 << 0)

/* bits in OTG_CTRL */

/* Bits that are controlled by OMAP OTG and are read-only */
#define OTG_CTRL_OMAP_MASK	(OTG_PULLDOWN|OTG_PULLUP|OTG_DRV_VBUS|\
				OTG_PD_VBUS|OTG_PU_VBUS|OTG_PU_ID)
/* Bits that are controlled by transceiver */
#define OTG_CTRL_XCVR_MASK	(OTG_ASESSVLD|OTG_BSESSEND|\
				OTG_BSESSVLD|OTG_VBUSVLD|OTG_ID)
/* Bits that are controlled by system */
#define OTG_CTRL_SYS_MASK	(OTG_A_BUSREQ|OTG_A_SETB_HNPEN|OTG_B_BUSREQ|\
				OTG_B_HNPEN|OTG_BUSDROP)

#if defined(CONFIG_USB_OHCI_HCD) && !defined(CONFIG_USB_OTG)
#error tahvo-otg.c does not work with OCHI yet!
#endif

#define TAHVO_MODE_HOST		0
#define TAHVO_MODE_PERIPHERAL	1

#ifdef CONFIG_USB_OTG
#define TAHVO_MODE(tu)		(tu)->tahvo_mode
#elif defined(CONFIG_USB_GADGET_OMAP)
#define TAHVO_MODE(tu)		TAHVO_MODE_PERIPHERAL
#else
#define TAHVO_MODE(tu)		TAHVO_MODE_HOST
#endif

struct tahvo_usb {
	struct platform_device *pt_dev;
	struct otg_transceiver otg;
	int vbus_state;
	struct work_struct irq_work;
	struct mutex serialize;
#ifdef CONFIG_USB_OTG
	int tahvo_mode;
#endif
	struct clk *ick;
};
static struct tahvo_usb *tahvo_usb_device;

/*
 * ---------------------------------------------------------------------------
 * OTG related functions
 *
 * These shoud be separated into omap-otg.c driver module, as they are used
 * by various transceivers. These functions are needed in the UDC-only case
 * as well. These functions are copied from GPL isp1301_omap.c
 * ---------------------------------------------------------------------------
 */
static struct platform_device *tahvo_otg_dev;

static irqreturn_t omap_otg_irq(int irq, void *arg)
{
	struct tahvo_usb *tu = arg;
	u16 otg_irq;

	otg_irq = omap_readw(OTG_IRQ_SRC);
	if (otg_irq & OPRT_CHG) {
		omap_writew(OPRT_CHG, OTG_IRQ_SRC);
	} else if (otg_irq & B_SRP_TMROUT) {
		omap_writew(B_SRP_TMROUT, OTG_IRQ_SRC);
	} else if (otg_irq & B_HNP_FAIL) {
		omap_writew(B_HNP_FAIL, OTG_IRQ_SRC);
	} else if (otg_irq & A_SRP_DETECT) {
		omap_writew(A_SRP_DETECT, OTG_IRQ_SRC);
	} else if (otg_irq & A_REQ_TMROUT) {
		omap_writew(A_REQ_TMROUT, OTG_IRQ_SRC);
	} else if (otg_irq & A_VBUS_ERR) {
		omap_writew(A_VBUS_ERR, OTG_IRQ_SRC);
	} else if (otg_irq & DRIVER_SWITCH) {
#ifdef CONFIG_USB_OTG
		if ((!(omap_readl(OTG_CTRL) & OTG_DRIVER_SEL)) &&
		   tu->otg.host && tu->otg.state == OTG_STATE_A_HOST) {
			/* role is host */
			usb_bus_start_enum(tu->otg.host,
					   tu->otg.host->otg_port);
		}
#endif
		omap_writew(DRIVER_SWITCH, OTG_IRQ_SRC);
	} else
		return IRQ_NONE;

	return IRQ_HANDLED;

}

static int tahvo_otg_init(void)
{
	u32 l;

#ifdef CONFIG_USB_OTG
	if (!tahvo_otg_dev) {
		printk("tahvo-usb: no tahvo_otg_dev\n");
		return -ENODEV;
	}
#endif

	l = omap_readl(OTG_SYSCON_1);
	l &= ~OTG_IDLE_EN;
	omap_writel(l, OTG_SYSCON_1);
	udelay(100);

	/* some of these values are board-specific... */
	l = omap_readl(OTG_SYSCON_2);
	l |= OTG_EN
		/* for B-device: */
		| SRP_GPDATA		/* 9msec Bdev D+ pulse */
		| SRP_GPDVBUS		/* discharge after VBUS pulse */
		// | (3 << 24)		/* 2msec VBUS pulse */
		/* for A-device: */
		| (0 << 20)		/* 200ms nominal A_WAIT_VRISE timer */
		| SRP_DPW		/* detect 167+ns SRP pulses */
		| SRP_DATA | SRP_VBUS;	/* accept both kinds of SRP pulse */
	omap_writel(l, OTG_SYSCON_2);

	omap_writew(DRIVER_SWITCH | OPRT_CHG
			| B_SRP_TMROUT | B_HNP_FAIL
				  | A_VBUS_ERR | A_SRP_DETECT | A_REQ_TMROUT,
					OTG_IRQ_EN);
	l = omap_readl(OTG_SYSCON_2);
	l |= OTG_EN;
	omap_writel(l, OTG_SYSCON_2);

	return 0;
}

static int __init omap_otg_probe(struct platform_device *pdev)
{
	int ret;

	tahvo_otg_dev = pdev;
	ret = tahvo_otg_init();
	if (ret != 0) {
		printk(KERN_ERR "tahvo-usb: tahvo_otg_init failed\n");
		return ret;
	}

	return request_irq(tahvo_otg_dev->resource[1].start,
			   omap_otg_irq, IRQF_DISABLED, DRIVER_NAME,
			   tahvo_usb_device);
}

static int __exit omap_otg_remove(struct platform_device *pdev)
{
	free_irq(tahvo_otg_dev->resource[1].start, tahvo_usb_device);
	tahvo_otg_dev = NULL;

	return 0;
}

struct platform_driver omap_otg_driver = {
	.driver		= {
		.name	= "omap_otg",
	},
	.remove		= __exit_p(omap_otg_remove),
};

/*
 * ---------------------------------------------------------------------------
 * Tahvo related functions
 * These are Nokia proprietary code, except for the OTG register settings,
 * which are copied from isp1301.c
 * ---------------------------------------------------------------------------
 */
static ssize_t vbus_state_show(struct device *device,
			       struct device_attribute *attr, char *buf)
{
	struct tahvo_usb *tu = dev_get_drvdata(device);
	return sprintf(buf, "%d\n", tu->vbus_state);
}
static DEVICE_ATTR(vbus_state, 0444, vbus_state_show, NULL);

int vbus_active = 0;

#if 0

static int host_suspend(struct tahvo_usb *tu)
{
	struct device	*dev;

	if (!tu->otg.host)
		return -ENODEV;

	/* Currently ASSUMES only the OTG port matters;
	 * other ports could be active...
	 */
	dev = tu->otg.host->controller;
	return dev->driver->suspend(dev, PMSG_SUSPEND);
}

static int host_resume(struct tahvo_usb *tu)
{
	struct device	*dev;

	if (!tu->otg.host)
		return -ENODEV;

	dev = tu->otg.host->controller;
	return dev->driver->resume(dev);
}

#else

static int host_suspend(struct tahvo_usb *tu)
{
	return 0;
}

static int host_resume(struct tahvo_usb *tu)
{
	return 0;
}

#endif

static void check_vbus_state(struct tahvo_usb *tu)
{
	int reg, prev_state;

	reg = tahvo_read_reg(TAHVO_REG_IDSR);
	if (reg & 0x01) {
		u32 l;

		vbus_active = 1;
		switch (tu->otg.state) {
		case OTG_STATE_B_IDLE:
			/* Enable the gadget driver */
			if (tu->otg.gadget)
				usb_gadget_vbus_connect(tu->otg.gadget);
			/* Set B-session valid and not B-sessio ended to indicate
			 * Vbus to be ok. */
			l = omap_readl(OTG_CTRL);
			l &= ~OTG_BSESSEND;
			l |= OTG_BSESSVLD;
			omap_writel(l, OTG_CTRL);

			tu->otg.state = OTG_STATE_B_PERIPHERAL;
			break;
		case OTG_STATE_A_IDLE:
			/* Session is now valid assuming the USB hub is driving Vbus */
			tu->otg.state = OTG_STATE_A_HOST;
			host_resume(tu);
			break;
		default:
			break;
		}
		printk("USB cable connected\n");
	} else {
		switch (tu->otg.state) {
		case OTG_STATE_B_PERIPHERAL:
			if (tu->otg.gadget)
				usb_gadget_vbus_disconnect(tu->otg.gadget);
			tu->otg.state = OTG_STATE_B_IDLE;
			break;
		case OTG_STATE_A_HOST:
			tu->otg.state = OTG_STATE_A_IDLE;
			break;
		default:
			break;
		}
		printk("USB cable disconnected\n");
		vbus_active = 0;
	}

	prev_state = tu->vbus_state;
	tu->vbus_state = reg & 0x01;
	if (prev_state != tu->vbus_state)
		sysfs_notify(&tu->pt_dev->dev.kobj, NULL, "vbus_state");
}

static void tahvo_usb_become_host(struct tahvo_usb *tu)
{
	u32 l;

	/* Clear system and transceiver controlled bits
	 * also mark the A-session is always valid */
	tahvo_otg_init();

	l = omap_readl(OTG_CTRL);
	l &= ~(OTG_CTRL_XCVR_MASK | OTG_CTRL_SYS_MASK);
	l |= OTG_ASESSVLD;
	omap_writel(l, OTG_CTRL);

	/* Power up the transceiver in USB host mode */
	tahvo_write_reg(TAHVO_REG_USBR, USBR_REGOUT | USBR_NSUSPEND |
			USBR_MASTER_SW2 | USBR_MASTER_SW1);
	tu->otg.state = OTG_STATE_A_IDLE;

	check_vbus_state(tu);
}

static void tahvo_usb_stop_host(struct tahvo_usb *tu)
{
	host_suspend(tu);
	tu->otg.state = OTG_STATE_A_IDLE;
}

static void tahvo_usb_become_peripheral(struct tahvo_usb *tu)
{
	u32 l;

	/* Clear system and transceiver controlled bits
	 * and enable ID to mark peripheral mode and
	 * BSESSEND to mark no Vbus */
	tahvo_otg_init();
	l = omap_readl(OTG_CTRL);
	l &= ~(OTG_CTRL_XCVR_MASK | OTG_CTRL_SYS_MASK | OTG_BSESSVLD);
	l |= OTG_ID | OTG_BSESSEND;
	omap_writel(l, OTG_CTRL);

	/* Power up transceiver and set it in USB perhiperal mode */
	tahvo_write_reg(TAHVO_REG_USBR, USBR_SLAVE_CONTROL | USBR_REGOUT | USBR_NSUSPEND | USBR_SLAVE_SW);
	tu->otg.state = OTG_STATE_B_IDLE;

	check_vbus_state(tu);
}

static void tahvo_usb_stop_peripheral(struct tahvo_usb *tu)
{
	u32 l;

	l = omap_readl(OTG_CTRL);
	l &= ~OTG_BSESSVLD;
	l |= OTG_BSESSEND;
	omap_writel(l, OTG_CTRL);

	if (tu->otg.gadget)
		usb_gadget_vbus_disconnect(tu->otg.gadget);
	tu->otg.state = OTG_STATE_B_IDLE;

}

static void tahvo_usb_power_off(struct tahvo_usb *tu)
{
	u32 l;
	int id;

	/* Disable gadget controller if any */
	if (tu->otg.gadget)
		usb_gadget_vbus_disconnect(tu->otg.gadget);

	host_suspend(tu);

	/* Disable OTG and interrupts */
	if (TAHVO_MODE(tu) == TAHVO_MODE_PERIPHERAL)
		id = OTG_ID;
	else
		id = 0;
	l = omap_readl(OTG_CTRL);
	l &= ~(OTG_CTRL_XCVR_MASK | OTG_CTRL_SYS_MASK | OTG_BSESSVLD);
	l |= id | OTG_BSESSEND;
	omap_writel(l, OTG_CTRL);
	omap_writew(0, OTG_IRQ_EN);

	l = omap_readl(OTG_SYSCON_2);
	l &= ~OTG_EN;
	omap_writel(l, OTG_SYSCON_2);

	l = omap_readl(OTG_SYSCON_1);
	l |= OTG_IDLE_EN;
	omap_writel(l, OTG_SYSCON_1);

	/* Power off transceiver */
	tahvo_write_reg(TAHVO_REG_USBR, 0);
	tu->otg.state = OTG_STATE_UNDEFINED;
}


static int tahvo_usb_set_power(struct otg_transceiver *dev, unsigned mA)
{
	struct tahvo_usb *tu = container_of(dev, struct tahvo_usb, otg);

	dev_dbg(&tu->pt_dev->dev, "set_power %d mA\n", mA);

	if (dev->state == OTG_STATE_B_PERIPHERAL) {
		/* REVISIT: Can Tahvo charge battery from VBUS? */
	}
	return 0;
}

static int tahvo_usb_set_suspend(struct otg_transceiver *dev, int suspend)
{
	struct tahvo_usb *tu = container_of(dev, struct tahvo_usb, otg);
	u16 w;

	dev_dbg(&tu->pt_dev->dev, "set_suspend\n");

	w = tahvo_read_reg(TAHVO_REG_USBR);
	if (suspend)
		w &= ~USBR_NSUSPEND;
	else
		w |= USBR_NSUSPEND;
	tahvo_write_reg(TAHVO_REG_USBR, w);

	return 0;
}

static int tahvo_usb_start_srp(struct otg_transceiver *dev)
{
	struct tahvo_usb *tu = container_of(dev, struct tahvo_usb, otg);
	u32 otg_ctrl;

	dev_dbg(&tu->pt_dev->dev, "start_srp\n");

	if (!dev || tu->otg.state != OTG_STATE_B_IDLE)
		return -ENODEV;

	otg_ctrl = omap_readl(OTG_CTRL);
	if (!(otg_ctrl & OTG_BSESSEND))
		return -EINVAL;

	otg_ctrl |= OTG_B_BUSREQ;
	otg_ctrl &= ~OTG_A_BUSREQ & OTG_CTRL_SYS_MASK;
	omap_writel(otg_ctrl, OTG_CTRL);
	tu->otg.state = OTG_STATE_B_SRP_INIT;

	return 0;
}

static int tahvo_usb_start_hnp(struct otg_transceiver *otg)
{
	struct tahvo_usb *tu = container_of(otg, struct tahvo_usb, otg);

	dev_dbg(&tu->pt_dev->dev, "start_hnp\n");
#ifdef CONFIG_USB_OTG
	/* REVISIT: Add this for OTG */
#endif
	return -EINVAL;
}

static int tahvo_usb_set_host(struct otg_transceiver *otg, struct usb_bus *host)
{
	struct tahvo_usb *tu = container_of(otg, struct tahvo_usb, otg);
	u32 l;

	dev_dbg(&tu->pt_dev->dev, "set_host %p\n", host);

	if (otg == NULL)
		return -ENODEV;

#if defined(CONFIG_USB_OTG) || !defined(CONFIG_USB_GADGET_OMAP)

	mutex_lock(&tu->serialize);

	if (host == NULL) {
		if (TAHVO_MODE(tu) == TAHVO_MODE_HOST)
			tahvo_usb_power_off(tu);
		tu->otg.host = NULL;
		mutex_unlock(&tu->serialize);
		return 0;
	}

	l = omap_readl(OTG_SYSCON_1);
	l &= ~(OTG_IDLE_EN | HST_IDLE_EN | DEV_IDLE_EN);
	omap_writel(l, OTG_SYSCON_1);

	if (TAHVO_MODE(tu) == TAHVO_MODE_HOST) {
		tu->otg.host = NULL;
		tahvo_usb_become_host(tu);
	} else
		host_suspend(tu);

	tu->otg.host = host;

	mutex_unlock(&tu->serialize);
#else
	/* No host mode configured, so do not allow host controlled to be set */
	return -EINVAL;
#endif

	return 0;
}

static int tahvo_usb_set_peripheral(struct otg_transceiver *otg, struct usb_gadget *gadget)
{
	struct tahvo_usb *tu = container_of(otg, struct tahvo_usb, otg);

	dev_dbg(&tu->pt_dev->dev, "set_peripheral %p\n", gadget);

	if (!otg)
		return -ENODEV;

#if defined(CONFIG_USB_OTG) || defined(CONFIG_USB_GADGET_OMAP)

	mutex_lock(&tu->serialize);

	if (!gadget) {
		if (TAHVO_MODE(tu) == TAHVO_MODE_PERIPHERAL)
			tahvo_usb_power_off(tu);
		tu->otg.gadget = NULL;
		mutex_unlock(&tu->serialize);
		return 0;
	}

	tu->otg.gadget = gadget;
	if (TAHVO_MODE(tu) == TAHVO_MODE_PERIPHERAL)
		tahvo_usb_become_peripheral(tu);

	mutex_unlock(&tu->serialize);
#else
	/* No gadget mode configured, so do not allow host controlled to be set */
	return -EINVAL;
#endif

	return 0;
}

static void tahvo_usb_irq_work(struct work_struct *work)
{
	struct tahvo_usb *tu = container_of(work, struct tahvo_usb, irq_work);

	mutex_lock(&tu->serialize);
	check_vbus_state(tu);
	mutex_unlock(&tu->serialize);
}

static void tahvo_usb_vbus_interrupt(unsigned long arg)
{
	struct tahvo_usb *tu = (struct tahvo_usb *) arg;

	tahvo_ack_irq(TAHVO_INT_VBUSON);
	/* Seems we need this to acknowledge the interrupt */
	tahvo_read_reg(TAHVO_REG_IDSR);
	schedule_work(&tu->irq_work);
}

#ifdef CONFIG_USB_OTG
static ssize_t otg_mode_show(struct device *device,
			     struct device_attribute *attr, char *buf)
{
	struct tahvo_usb *tu = dev_get_drvdata(device);
	switch (tu->tahvo_mode) {
	case TAHVO_MODE_HOST:
		return sprintf(buf, "host\n");
	case TAHVO_MODE_PERIPHERAL:
		return sprintf(buf, "peripheral\n");
	}
	return sprintf(buf, "unknown\n");
}

static ssize_t otg_mode_store(struct device *device,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct tahvo_usb *tu = dev_get_drvdata(device);
	int r;

	r = strlen(buf);
	mutex_lock(&tu->serialize);
	if (strncmp(buf, "host", 4) == 0) {
		if (tu->tahvo_mode == TAHVO_MODE_PERIPHERAL)
			tahvo_usb_stop_peripheral(tu);
		tu->tahvo_mode = TAHVO_MODE_HOST;
		if (tu->otg.host) {
			printk(KERN_INFO "Selected HOST mode: host controller present.\n");
			tahvo_usb_become_host(tu);
		} else {
			printk(KERN_INFO "Selected HOST mode: no host controller, powering off.\n");
			tahvo_usb_power_off(tu);
		}
	} else if (strncmp(buf, "peripheral", 10) == 0) {
		if (tu->tahvo_mode == TAHVO_MODE_HOST)
			tahvo_usb_stop_host(tu);
		tu->tahvo_mode = TAHVO_MODE_PERIPHERAL;
		if (tu->otg.gadget) {
			printk(KERN_INFO "Selected PERIPHERAL mode: gadget driver present.\n");
			tahvo_usb_become_peripheral(tu);
		} else {
			printk(KERN_INFO "Selected PERIPHERAL mode: no gadget driver, powering off.\n");
			tahvo_usb_power_off(tu);
		}
	} else
		r = -EINVAL;

	mutex_unlock(&tu->serialize);
	return r;
}

static DEVICE_ATTR(otg_mode, 0644, otg_mode_show, otg_mode_store);
#endif

static int __init tahvo_usb_probe(struct platform_device *pdev)
{
	struct tahvo_usb *tu;
	struct device *dev = &pdev->dev;
	int ret;

	ret = tahvo_get_status();
	if (!ret)
		return -ENODEV;

	dev_dbg(dev, "probe\n");

	/* Create driver data */
	tu = kzalloc(sizeof(*tu), GFP_KERNEL);
	if (!tu)
		return -ENOMEM;
	tahvo_usb_device = tu;

	tu->pt_dev = container_of(dev, struct platform_device, dev);
#ifdef CONFIG_USB_OTG
	/* Default mode */
#ifdef CONFIG_CBUS_TAHVO_USB_HOST_BY_DEFAULT
	tu->tahvo_mode = TAHVO_MODE_HOST;
#else
	tu->tahvo_mode = TAHVO_MODE_PERIPHERAL;
#endif
#endif

	INIT_WORK(&tu->irq_work, tahvo_usb_irq_work);
	mutex_init(&tu->serialize);

	tu->ick = clk_get(NULL, "usb_l4_ick");
	if (IS_ERR(tu->ick)) {
		dev_err(dev, "Failed to get usb_l4_ick\n");
		ret = PTR_ERR(tu->ick);
		goto err_free_tu;
	}
	clk_enable(tu->ick);

	/* Set initial state, so that we generate kevents only on
	 * state changes */
	tu->vbus_state = tahvo_read_reg(TAHVO_REG_IDSR) & 0x01;

	/* We cannot enable interrupt until omap_udc is initialized */
	ret = tahvo_request_irq(TAHVO_INT_VBUSON, tahvo_usb_vbus_interrupt,
				(unsigned long) tu, "vbus_interrupt");
	if (ret != 0) {
		printk(KERN_ERR "Could not register Tahvo interrupt for VBUS\n");
		goto err_release_clk;
	}

	/* Attributes */
	ret = device_create_file(dev, &dev_attr_vbus_state);
#ifdef CONFIG_USB_OTG
	ret |= device_create_file(dev, &dev_attr_otg_mode);
#endif
	if (ret)
		printk(KERN_ERR "attribute creation failed: %d\n", ret);

	/* Create OTG interface */
	tahvo_usb_power_off(tu);
	tu->otg.state = OTG_STATE_UNDEFINED;
	tu->otg.label = DRIVER_NAME;
	tu->otg.set_host = tahvo_usb_set_host;
	tu->otg.set_peripheral = tahvo_usb_set_peripheral;
	tu->otg.set_power = tahvo_usb_set_power;
	tu->otg.set_suspend = tahvo_usb_set_suspend;
	tu->otg.start_srp = tahvo_usb_start_srp;
	tu->otg.start_hnp = tahvo_usb_start_hnp;

	ret = otg_set_transceiver(&tu->otg);
	if (ret < 0) {
		printk(KERN_ERR "Cannot register USB transceiver\n");
		goto err_free_irq;
	}

	dev_set_drvdata(dev, tu);

	/* Act upon current vbus state once at startup. A vbus state irq may or
	 * may not be generated in addition to this. */
	schedule_work(&tu->irq_work);
	return 0;

err_free_irq:
	tahvo_free_irq(TAHVO_INT_VBUSON);
err_release_clk:
	clk_disable(tu->ick);
	clk_put(tu->ick);
err_free_tu:
	kfree(tu);
	tahvo_usb_device = NULL;

	return ret;
}

static int __exit tahvo_usb_remove(struct platform_device *pdev)
{
	struct tahvo_usb *tu = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "remove\n");

	tahvo_free_irq(TAHVO_INT_VBUSON);
	flush_scheduled_work();
	otg_set_transceiver(0);
	device_remove_file(&pdev->dev, &dev_attr_vbus_state);
#ifdef CONFIG_USB_OTG
	device_remove_file(&pdev->dev, &dev_attr_otg_mode);
#endif
	clk_disable(tu->ick);
	clk_put(tu->ick);

	kfree(tu);
	tahvo_usb_device = NULL;

	return 0;
}

static struct platform_driver tahvo_usb_driver = {
	.driver		= {
		.name	= "tahvo-usb",
	},
	.remove		= __exit_p(tahvo_usb_remove),
};

static int __init tahvo_usb_init(void)
{
	int ret = 0;

	ret = platform_driver_probe(&tahvo_usb_driver, tahvo_usb_probe);
	if (ret)
		return ret;

	ret = platform_driver_probe(&omap_otg_driver, omap_otg_probe);
	if (ret) {
		platform_driver_unregister(&tahvo_usb_driver);
		return ret;
	}

	return 0;
}

subsys_initcall(tahvo_usb_init);

static void __exit tahvo_usb_exit(void)
{
	platform_driver_unregister(&omap_otg_driver);
	platform_driver_unregister(&tahvo_usb_driver);
}
module_exit(tahvo_usb_exit);

MODULE_DESCRIPTION("Tahvo USB OTG Transceiver Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Juha Yrjölä, Tony Lindgren, and Timo Teräs");
