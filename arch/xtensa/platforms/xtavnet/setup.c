/*
 * arch/xtensa/platform-xtavnet/setup.c
 *
 * Setup/initialization for Tensilica Avnet boards (XT-AV60, XT-AV110, XT-AV200,
 * derived from Avnet LX60, LX110, LX200 boards respectively).
 * For details, see "Tensilica Avnet LX### (XT-AV###) Board User's Guide"
 * (where ### is 60, 110, or 200).
 *
 * Authors:	Chris Zankel <chris@zankel.net>
 *		Joe Taylor <joe@tensilica.com>
 *		Pete Delaney <piet@tensilica.com>
 *		Marc Gauthier <marc@tensilica.com> <marc@alumni.uwaterloo.ca>
 * 
 * Copyright 2001-2010 Tensilica Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License, version 2.  See the file "COPYING" in the main directory of this
 * archive for more details.
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/reboot.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/major.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/stringify.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <asm/timex.h>

#include <linux/etherdevice.h>
#include <net/ethoc.h>

#include <asm/processor.h>
#include <asm/platform.h>
#include <asm/bootparam.h>
#include <platform/lcd.h>
#include <platform/hardware.h>
#include <variant/core.h>

/*  For doing extra init. beyond what the ethoc driver does.  */
struct oeth_regs {
	unsigned moder;		/* Mode Register */
	unsigned int_src;	/* Interrupt Source Register */
	unsigned int_mask;	/* Interrupt Mask Register */
	unsigned ipgt;		/* Back to Bak Inter Packet Gap Register */
	unsigned ipgr1;		/* Non Back to Back Inter Packet Gap Register 1 */
	unsigned ipgr2;		/* Non Back to Back Inter Packet Gap Register 2 */
	unsigned packet_len;	/* Packet Length Register (min. and max.) */
	unsigned collconf;	/* Collision and Retry Configuration Register */
	unsigned tx_bd_num;	/* Transmit Buffer Descriptor Number Register */
	unsigned ctrlmoder;	/* Control Module Mode Register */
	unsigned miimoder;	/* MII Mode Register */
	unsigned miicommand;	/* MII Command Register */
	unsigned miiaddress;	/* MII Address Register */
	unsigned miitx_data;	/* MII Transmit Data Register */
	unsigned miirx_data;	/* MII Receive Data Register */
	unsigned miistatus;	/* MII Status Register */
	unsigned mac_addr0;	/* MAC Individual Address Register 0 */
	unsigned mac_addr1;	/* MAC Individual Address Register 1 */
	unsigned hash_addr0;	/* Hash Register 0 */
	unsigned hash_addr1;	/* Hash Register 1 */
};
/* MODER Register */
#define OETH_MODER_RST		0x00000800	/* Reset MAC */
/* MII Mode Register */
#define OETH_MIIMODER_CLKDIV	0x000000FF	/* Clock Divider */



void platform_halt(void)
{
	/* Just display HALT on LCD display, and loop.  */
	lcd_disp_at_pos(" HALT ", 0);
	local_irq_disable();
	while (1);
}

void platform_power_off(void)
{
	/* No software-controlled power-off, just display POWEROFF and loop.  */
        lcd_disp_at_pos ("POWEROFF", 0);
	local_irq_disable();
	while (1);
}

void platform_restart(void)
{
	/* Software-initiated board reset.  */
	*(volatile unsigned *)XTAVNET_SWRST_VADDR = 0xdead;
}

void platform_heartbeat(void)
{
	/* Executes every timer tick. */
}


/* 
 * Called from time_init().  "Calibrating" is a misnomer, here we just read
 * the clock rate from the board specific FPGA registers.
 */
void platform_calibrate_ccount(void)
{
	long clk_freq = *(long *)XTAVNET_CLKFRQ_VADDR;

	ccount_per_jiffy = clk_freq / HZ;
	nsec_per_ccount = 1000000000UL / clk_freq;
}


/*----------------------------------------------------------------------------
 *  Ethernet -- OpenCores Ethernet MAC (ethoc driver)
 */

static struct resource ethoc_res[] = {
	[0] = {	/* register space */
		.start = OETH_REGS_PADDR,
		.end   = OETH_REGS_PADDR + OETH_REGS_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {	/* buffer space */
		.start = OETH_SRAMBUFF_PADDR,
		.end   = OETH_SRAMBUFF_PADDR + OETH_SRAMBUFF_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {	/* IRQ number */
		.start = OETH_IRQ,
		.end   = OETH_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct ethoc_platform_data ethoc_pdata = {
	.hwaddr = { OETH_MACADDR },	/* last byte written in setup below */
	.phy_id = -1,
};

static struct platform_device ethoc_device = {
	.name = "ethoc",
	.id = -1,
	.num_resources = ARRAY_SIZE(ethoc_res),
	.resource = ethoc_res,
	.dev = {
		.platform_data = &ethoc_pdata,
	},
};


/*----------------------------------------------------------------------------
 *  UART
 */

static struct resource serial_resource = {
	.start	= UART_PADDR,
	.end	= UART_PADDR + 0x1f,
	.flags	= IORESOURCE_MEM,
};

static struct plat_serial8250_port serial_platform_data[] = {
	[0] = {
		.mapbase	= UART_PADDR,
		.irq		= UART_INTNUM,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST | UPF_IOREMAP,
		.iotype		= UPIO_MEM32,
		.regshift	= 2,
		.uartclk	= 0,	/* set in xtavnet_init() */
	},
	{ },
};

static struct platform_device xtavnet_uart = {
	.name		= "serial8250",
	.id		= PLAT8250_DEV_PLATFORM,
	.dev		= {
		.platform_data	= serial_platform_data,
	},
	.num_resources	= 1,
	.resource	= &serial_resource,
};


/*----------------------------------------------------------------------------
 */

/* platform devices */
static struct platform_device *platform_devices[] = {
	&ethoc_device,
	&xtavnet_uart,
};



/* very early init */
void __init platform_setup(char **cmdline)
{
	if (cmdline) {
		if (cmdline[0])
			printk("XTAVnet: platform_setup(cmdline[0]:'%s')\n", cmdline[0]);
		else
			printk("XTAVnet: platform_setup(cmdline[0]:<null>)\n");
	}
}

/* early initialization, before secondary cpu's have been brought up */

void platform_init(bp_tag_t *bootparams)
{
	printk("\n");
	if( bootparams )
		printk("XTAVnet: platform_init(bootparams:0x%x)\n", (unsigned)bootparams);
}

static int xtavnet_init(void)
{
	volatile struct oeth_regs *regs = (volatile struct oeth_regs*)OETH_REGS_VADDR;

	/*
	 *  Do some of the initialization missing in the ETHOC driver.
	 *  (Perhaps not all necessary, but was in a previously used driver.)
	 */

	/* Reset the controller. */
	regs->moder = OETH_MODER_RST;	/* Reset ON */
	regs->moder &= ~OETH_MODER_RST;	/* Reset OFF */

	regs->packet_len = (64 << 16) | 1536;
	regs->ipgr1 = 0x0000000c;
	regs->ipgr2 = 0x00000012;
	regs->collconf = 0x000f003f;
	regs->ctrlmoder = 0;
	regs->miimoder = (OETH_MIIMODER_CLKDIV & 0x2);

	/*
	 *  Setup dynamic info in platform device init structures.
	 */

	/* Ethernet MAC address.  */
	ethoc_pdata.hwaddr[5] = *(u32*)DIP_SWITCHES_VADDR;

	/* Clock rate varies among FPGA bitstreams; board specific FPGA register
	 * reports the actual clock rate.  */
	serial_platform_data[0].uartclk = *(long *)XTAVNET_CLKFRQ_VADDR;


	/* register platform devices */
	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));

	/*  ETHOC driver is a bit quiet; at least display Ethernet MAC, so user
	    knows whether they set it correctly on the DIP switches.  */
	printk("XTAVnet: Ethernet MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
		ethoc_pdata.hwaddr[0], ethoc_pdata.hwaddr[1], ethoc_pdata.hwaddr[2],
		ethoc_pdata.hwaddr[3], ethoc_pdata.hwaddr[4], ethoc_pdata.hwaddr[5]);

	return 0;
}


/*
 * Register to be done during do_initcalls().
 */
arch_initcall(xtavnet_init);


