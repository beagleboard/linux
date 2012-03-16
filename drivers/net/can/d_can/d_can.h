/*
 * CAN bus driver for Bosch D_CAN controller
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Borrowed from C_CAN driver
 * Copyright (C) 2010 ST Microelectronics
 * - Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * Borrowed heavily from the C_CAN driver originally written by:
 * Copyright (C) 2007
 * - Sascha Hauer, Marc Kleine-Budde, Pengutronix <s.hauer@pengutronix.de>
 * - Simon Kallweit, intefo AG <simon.kallweit@intefo.ch>
 *
 * Bosch D_CAN controller is compliant to CAN protocol version 2.0 part A and B.
 * Bosch D_CAN user manual can be obtained from:
 * http://www.semiconductors.bosch.de/media/en/pdf/ipmodules_1/can/
 * d_can_users_manual_111.pdf
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef D_CAN_H
#define D_CAN_H

#define D_CAN_DRV_NAME	"d_can"
#define D_CAN_VERSION	"1.0"
#define D_CAN_DRV_DESC	"CAN bus driver for Bosch D_CAN controller " \
			D_CAN_VERSION

/* d_can private data structure */
struct d_can_priv {
	struct can_priv can;	/* must be the first member */
	struct napi_struct napi;
	struct net_device *dev;
	struct platform_device *pdev;
	int current_status;
	int last_status;
	unsigned int irqstatus;
	void __iomem *base;
	u32 napi_weight;
	struct clk *fck;
	struct clk *ick;
	unsigned int irq;	/* device IRQ number, for all MO and ES	*/
	unsigned int irq_obj;	/* device IRQ number for only Msg Object */
	unsigned int irq_parity; /* device IRQ number for parity error */
	unsigned long irq_flags; /* for request_irq() */
	unsigned int tx_next;
	unsigned int tx_echo;
	unsigned int rx_next;
	bool opened;
	void *priv;		/* for board-specific data */
	void (*ram_init) (unsigned int, unsigned int);
};

struct net_device *alloc_d_can_dev(int);
void free_d_can_dev(struct net_device *dev);
int d_can_power_up(struct d_can_priv *d_can);
int d_can_power_down(struct d_can_priv *d_can);
int register_d_can_dev(struct net_device *dev);
void unregister_d_can_dev(struct net_device *dev);
void d_can_reset_ram(struct d_can_priv *d_can, unsigned int instance,
					unsigned int enable);

#endif /* D_CAN_H */
