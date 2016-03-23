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
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg-fsm.h>
#include <linux/usb/phy.h>

/**
 * struct otg_hcd - host controller state and interface
 *
 * @hcd: host controller
 * @irqnum: irq number
 * @irqflags: irq flags
 * @ops: otg to host controller interface
 */
struct otg_hcd {
	struct usb_hcd *hcd;
	unsigned int irqnum;
	unsigned long irqflags;
	struct otg_hcd_ops *ops;
};

struct usb_otg;

/**
 * struct otg_timer - otg timer data
 *
 * @timer: high resolution timer
 * @timeout: timeout value
 * @timetout_bit: pointer to variable that is set on timeout
 * @otgd: usb otg data
 */
struct otg_timer {
	struct hrtimer timer;
	ktime_t timeout;
	/* callback data */
	int *timeout_bit;
	struct usb_otg *otgd;
};

/**
 * struct usb_otg - usb otg controller state
 *
 * @default_a: Indicates we are an A device. i.e. Host.
 * @phy: USB phy interface
 * @usb_phy: old usb_phy interface
 * @host: host controller bus
 * @gadget: gadget device
 * @state: current otg state
 * @dev: otg controller device
 * @caps: otg capabilities revision, hnp, srp, etc
 * @fsm: otg finite state machine
 * @fsm_ops: controller hooks for the state machine
 * ------- internal use only -------
 * @primary_hcd: primary host state and interface
 * @shared_hcd: shared host state and interface
 * @gadget_ops: gadget interface
 * @timers: otg timers for state machine
 * @list: list of otg controllers
 * @work: otg state machine work
 * @wq: otg state machine work queue
 * @fsm_running: state machine running/stopped indicator
 * @flags: to track if host/gadget is running
 * @drd_only: dual-role mode. no otg features.
 */
struct usb_otg {
	u8			default_a;

	struct phy		*phy;
	/* old usb_phy interface */
	struct usb_phy		*usb_phy;

	struct usb_bus		*host;
	struct usb_gadget	*gadget;

	enum usb_otg_state	state;

	struct device *dev;
	struct usb_otg_caps *caps;
	struct otg_fsm fsm;
	struct otg_fsm_ops fsm_ops;

	/* internal use only */
	struct otg_hcd primary_hcd;
	struct otg_hcd shared_hcd;
	struct otg_gadget_ops *gadget_ops;
	struct otg_timer timers[NUM_OTG_FSM_TIMERS];
	struct list_head list;
	struct work_struct work;
	struct workqueue_struct *wq;
	bool fsm_running;
	u32 flags;
#define OTG_FLAG_GADGET_RUNNING (1 << 0)
#define OTG_FLAG_HOST_RUNNING (1 << 1)
	/* use otg->fsm.lock for serializing access */
	bool drd_only;

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

/**
 * struct usb_otg_caps - describes the otg capabilities of the device
 * @otg_rev: The OTG revision number the device is compliant with, it's
 *		in binary-coded decimal (i.e. 2.0 is 0200H).
 * @hnp_support: Indicates if the device supports HNP.
 * @srp_support: Indicates if the device supports SRP.
 * @adp_support: Indicates if the device supports ADP.
 */
struct usb_otg_caps {
	u16 otg_rev;
	bool hnp_support;
	bool srp_support;
	bool adp_support;
};

/**
 * struct usb_otg_config - otg controller configuration
 * @caps: otg capabilities of the controller
 * @ops: otg fsm operations
 * @otg_timeouts: override default otg fsm timeouts
 */
struct usb_otg_config {
	struct usb_otg_caps otg_caps;
	struct otg_fsm_ops *fsm_ops;
	unsigned otg_timeouts[NUM_OTG_FSM_TIMERS];
};

extern const char *usb_otg_state_string(enum usb_otg_state state);

#if IS_ENABLED(CONFIG_USB_OTG)
struct otg_fsm *usb_otg_register(struct device *dev,
				 struct usb_otg_config *config);
int usb_otg_unregister(struct device *dev);
int usb_otg_register_hcd(struct usb_hcd *hcd, unsigned int irqnum,
			 unsigned long irqflags, struct otg_hcd_ops *ops);
int usb_otg_unregister_hcd(struct usb_hcd *hcd);
int usb_otg_register_gadget(struct usb_gadget *gadget,
			    struct otg_gadget_ops *ops);
int usb_otg_unregister_gadget(struct usb_gadget *gadget);
void usb_otg_sync_inputs(struct otg_fsm *fsm);
int usb_otg_kick_fsm(struct device *hcd_gcd_device);
struct device *usb_otg_fsm_to_dev(struct otg_fsm *fsm);
int usb_otg_start_host(struct otg_fsm *fsm, int on);
int usb_otg_start_gadget(struct otg_fsm *fsm, int on);

#else /* CONFIG_USB_OTG */

static inline struct otg_fsm *usb_otg_register(struct device *dev,
					       struct usb_otg_config *config)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline int usb_otg_unregister(struct device *dev)
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

static inline int usb_otg_start_host(struct otg_fsm *fsm, int on)
{
	return -ENOTSUPP;
}

static inline int usb_otg_start_gadget(struct otg_fsm *fsm, int on)
{
	return -ENOTSUPP;
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

enum usb_dr_mode {
	USB_DR_MODE_UNKNOWN,
	USB_DR_MODE_HOST,
	USB_DR_MODE_PERIPHERAL,
	USB_DR_MODE_OTG,
};

/**
 * usb_get_dr_mode - Get dual role mode for given device
 * @dev: Pointer to the given device
 *
 * The function gets phy interface string from property 'dr_mode',
 * and returns the correspondig enum usb_dr_mode
 */
extern enum usb_dr_mode usb_get_dr_mode(struct device *dev);

#endif /* __LINUX_USB_OTG_H */
