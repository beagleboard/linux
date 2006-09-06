/*
 * This is used to for host and peripheral modes of the driver for
 * Inventra (Multidrop) Highspeed Dual-Role Controllers:  (M)HDRC.
 *
 * Board initialization should put one of these into dev->platform_data,
 * probably on some platform_device named "musb_hdrc".  It encapsulates
 * key configuration differences between boards.
 */

/* The USB role is defined by the connector used on the board, so long as
 * standards are being followed.  (Developer boards sometimes won't.)
 */
enum musb_mode {
	MUSB_UNDEFINED = 0,
	MUSB_HOST,		/* A or Mini-A connector */
	MUSB_PERIPHERAL,	/* B or Mini-B connector */
	MUSB_OTG		/* Mini-AB connector */
};

struct musb_hdrc_platform_data {
	/* MUSB_HOST, MUSB_PERIPHERAL, or MUSB_OTG */
	u8		mode;

	/* (HOST or OTG) switch VBUS on/off */
	int		(*set_vbus)(struct device *dev, int is_on);

	/* (HOST or OTG) mA/2 power supplied on (default = 8mA) */
	u8		power;

	/* (HOST or OTG) msec/2 after VBUS on till power good */
	u8		potpgt;

	/* TBD:  chip defaults should probably go someplace else,
	 * e.g. number of tx/rx endpoints, etc
	 */
	unsigned	multipoint:1;

	/* Power the device on or off */
	int		(*set_power)(int state);
};

