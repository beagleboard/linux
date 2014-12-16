#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/usb/hcd.h>

struct usb_drd_setup {
	int	(*ll_start)(void *);
	int	(*ll_stop)(void *);
	void	(*ll_release)(struct device *);
	void	*data;
};

struct usb_drd_host {
	struct usb_hcd		*main_hcd;
	struct usb_hcd		*shared_hcd;
	int			hcd_irq;
	struct usb_drd_setup *host_setup;
};

struct usb_drd_gadget {
	struct usb_gadget_driver *g_driver;
	struct usb_gadget *gadget;
	struct usb_drd_setup *gadget_setup;
};

#define DRD_UNREGISTERED	0x0
#define DRD_DEVICE_REGISTERED	0x1
#define DRD_HOST_REGISTERED	0x2
#define DRD_HOST_ACTIVE		0x4
#define DRD_DEVICE_ACTIVE	0x8

#if IS_ENABLED(CONFIG_DRD_LIB)
int usb_drd_release(struct device *parent);
int usb_drd_add(struct device *parent);
int usb_drd_register_udc(struct device *parent,
			 struct usb_drd_gadget *gadget);
int usb_drd_register_udc_driver(struct device *parent,
				struct usb_gadget_driver *driver);
int usb_drd_unregister_udc(struct device *parent);
int usb_drd_unregister_udc_driver(struct device *parent);
int usb_drd_register_hcd(struct device *parent,
			 struct usb_drd_host *host);
int usb_drd_unregister_hcd(struct device *parent);
int usb_drd_start_hcd(struct device *parent);
int usb_drd_stop_hcd(struct device *parent);
int usb_drd_start_udc(struct device *parent);
int usb_drd_stop_udc(struct device *parent);
int usb_drd_get_state(struct device *parent);
#else
static inline int usb_drd_release(struct device *parent)
{ return 0; }
static inline int usb_drd_add(struct device *parent)
{ return 0; }
static inline int usb_drd_register_udc(struct device *parent,
				       struct usb_drd_gadget *gadget)
{ return 0; }
static inline int usb_drd_register_udc_driver(struct device *parent,
					      struct usb_gadget_driver *driver)
{ return 0; }
static inline int usb_drd_unregister_udc(struct device *parent,
					 struct usb_drd_gadget *gadget)
{ return 0; }
static inline int usb_drd_unregister_udc_driver(struct device *parent)
{ return 0; }
static inline int usb_drd_register_hcd(struct device *parent,
				       struct usb_drd_host *host)
{ return 0; }
static inline int usb_drd_unregister_hcd(struct device *parent)
{ return 0; }
static inline int usb_drd_stop_hcd(struct device *parent)
{ return 0; }
static inline int usb_drd_start_udc(struct device *parent)
{ return 0; }
static inline int usb_drd_stop_udc(struct device *parent)
{ return 0; }
static inline int usb_drd_get_state(struct device *parent)
{ return 0; }
#endif
