/* USB OTG (On The Go) defines */
/*
 *
 * These APIs may be used between USB controllers.  USB device drivers
 * (for either host or peripheral roles) don't use these calls; they
 * continue to use just usb_device and usb_gadget.
 */

#ifndef __LINUX_USB_OTG_H
#define __LINUX_USB_OTG_H

#include <linux/phy/phy.h>
#include <linux/device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg-fsm.h>
#include <linux/usb/phy.h>

struct usb_otg {
	u8			default_a;

	struct phy		*phy;
	/* old usb_phy interface */
	struct usb_phy		*usb_phy;
	struct usb_bus		*host;
	struct usb_gadget	*gadget;

	enum usb_otg_state	state;

/*------------- deprecated interface -----------------------------*/
	/* bind/unbind the host controller */
	int	(*set_host)(struct usb_otg *otg, struct usb_bus *host);

	/* bind/unbind the peripheral controller */
	int	(*set_peripheral)(struct usb_otg *otg,
					struct usb_gadget *gadget);

	/* effective for A-peripheral, ignored for B devices */
	int	(*set_vbus)(struct usb_otg *otg, bool enabled);

	/* for B devices only:  start session with A-Host */
	int	(*start_srp)(struct usb_otg *otg);

	/* start or continue HNP role switch */
	int	(*start_hnp)(struct usb_otg *otg);
/*---------------------------------------------------------------*/
};

extern const char *usb_otg_state_string(enum usb_otg_state state);

enum usb_dr_mode {
	USB_DR_MODE_UNKNOWN,
	USB_DR_MODE_HOST,
	USB_DR_MODE_PERIPHERAL,
	USB_DR_MODE_OTG,
};

#if IS_ENABLED(CONFIG_USB_OTG)
struct otg_fsm *usb_otg_register(struct device *parent_dev,
				 struct otg_fsm_ops *fsm_ops, bool drd_only);
int usb_otg_unregister(struct device *parent_dev);
int usb_otg_register_hcd(struct usb_hcd *hcd, unsigned int irqnum,
			 unsigned long irqflags, struct otg_hcd_ops *ops);
int usb_otg_unregister_hcd(struct usb_hcd *hcd);
int usb_otg_register_gadget(struct usb_gadget *gadget,
			    struct otg_gadget_ops *ops);
int usb_otg_unregister_gadget(struct usb_gadget *gadget);
void usb_otg_sync_inputs(struct otg_fsm *fsm);
int usb_otg_kick_fsm(struct device *hcd_gcd_device);
struct device *usb_otg_fsm_to_dev(struct otg_fsm *fsm);

#else /* CONFIG_USB_OTG */

static inline struct otg_fsm *usb_otg_register(struct device *parent_dev,
					       struct otg_fsm_ops *fsm_ops,
					       bool drd_only)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline int usb_otg_unregister(struct device *parent_dev)
{
	return -ENOTSUPP;
}

static inline int usb_otg_register_hcd(struct usb_hcd *hcd, unsigned int irqnum,
				       unsigned long irqflags,
				       struct otg_hcd_ops *ops)
{
	return -ENOTSUPP;
}

static inline int usb_otg_unregister_hcd(struct usb_hcd *hcd)
{
	return -ENOTSUPP;
}

static inline int usb_otg_register_gadget(struct usb_gadget *gadget,
					  struct otg_gadget_ops *ops)
{
	return -ENOTSUPP;
}

static inline int usb_otg_unregister_gadget(struct usb_gadget *gadget)
{
	return -ENOTSUPP;
}

static inline void usb_otg_sync_inputs(struct otg_fsm *fsm)
{
}

static inline int usb_otg_kick_fsm(struct device *hcd_gcd_device)
{
	return -ENOTSUPP;
}

static inline struct device *usb_otg_fsm_to_dev(struct otg_fsm *fsm)
{
	return NULL;
}

#endif /* CONFIG_USB_OTG */

/*------------- deprecated interface -----------------------------*/
/* Context: can sleep */
static inline int
otg_start_hnp(struct usb_otg *otg)
{
	if (otg && otg->start_hnp)
		return otg->start_hnp(otg);

	return -ENOTSUPP;
}

/* Context: can sleep */
static inline int
otg_set_vbus(struct usb_otg *otg, bool enabled)
{
	if (otg && otg->set_vbus)
		return otg->set_vbus(otg, enabled);

	return -ENOTSUPP;
}

/* for HCDs */
static inline int
otg_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	if (otg && otg->set_host)
		return otg->set_host(otg, host);

	return -ENOTSUPP;
}

/* for usb peripheral controller drivers */

/* Context: can sleep */
static inline int
otg_set_peripheral(struct usb_otg *otg, struct usb_gadget *periph)
{
	if (otg && otg->set_peripheral)
		return otg->set_peripheral(otg, periph);

	return -ENOTSUPP;
}

static inline int
otg_start_srp(struct usb_otg *otg)
{
	if (otg && otg->start_srp)
		return otg->start_srp(otg);

	return -ENOTSUPP;
}

/*---------------------------------------------------------------*/

/* for OTG controller drivers (and maybe other stuff) */
extern int usb_bus_start_enum(struct usb_bus *bus, unsigned port_num);

#endif /* __LINUX_USB_OTG_H */
