/*
 * drivers/char/innovator_ps2.c
 *
 * Basic PS/2 keyboard/mouse driver for the Juno® USAR HID controller
 * present on the TI Innovator/OMAP1510 Break-out-board.
 *
 *
 * Author: MontaVista Software, Inc.
 *         <gdavis@mvista.com> or <source@mvista.com>
 *
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 *
 * REFERENCES:
 *
 * 1.	Technical Reference Manual
 *	Juno® 01
 *	Multi-function ICs family
 *	UR8HC007-001 HID & Power management controller
 *	Document Number: DOC8-007-001-TR-075
 *	Date: February 2002
 *	Copyright ©1998-2002 Semtech Corporation
 *	http://www.semtech.com/pdf/doc8-007-001-tr.pdf
 *
 * 2.	Juno® 01 UR8HC007-001 Data Sheet
 *	Extremely Low-power Input Device and Power Management IC
 *	Copyright ©1998-2002 Semtech Corporation
 *	DOC8-007-001-DS-112
 *	http://www.semtech.com/pdf/doc8-007-001-ds.pdf
 *
 *
 * HISTORY:
 *
 * 20030626: George G. Davis <gdavis@mvista.com>
 *      Initially based on the following RidgeRun DSPlinux Version 1.6 files:
 *		linux-2.4.15-rmk1-dsplinux/arch/arm/dsplinux/hid/omap1510_hid.c
 *		linux-2.4.15-rmk1-dsplinux/arch/arm/dsplinux/hid/omap1510_hid.h
 *		linux-2.4.15-rmk1-dsplinux/arch/arm/dsplinux/hid/omap1510_ps2.c
 *		linux-2.4.15-rmk1-dsplinux/arch/arm/dsplinux/hid/omap1510_spi.c
 *	All original files above are
 *		Copyright (C) 2001 RidgeRun, Inc.
 *		Author: Alex McMains <aam@ridgerun.com>
 *
 * 20040812: Thiago Radicchi <trr@dcc.ufmg.br>
 *      Cleanup of old code from 2.4 driver and some debug code.
 *      Minor changes in interrupt handling code.
 *
 * NOTES:
 *
 * 1. This driver does not provide support for setting keyboard/mouse
 *    configuration parameters. Both devices are managed directly by
 *    the Juno UR8HC007-001 on behalf of the host. This minimises the
 *    amount of host processing required to manage HID events and state
 *    changes, e.g. both keyboard and mouse devices are hot pluggable
 *    with no host intervention required. However, we cannot customise
 *    keyboard/mouse settings in this case. So we live with the defaults
 *    as setup by the Juno UR8HC007-001 whatever they may be.
 * 2. Keyboard auto repeat does not work. See 1 above. : )
 *
 *
 * TODO:
 *
 * 1. Complete DPM/LDM stubs and test.
 * 2. Add SPI error handling support, i.e. resend, etc.,.
 * 3. Determine why innovator_hid_interrupt() is called for every
 *    invocation of Innovator FPGA IRQ demux. It appears that the
 *    missed Innovator ethernet workaround may be to blame. However,
 *    it does not adversely affect operation of this driver since we
 *    check for assertion of ATN prior to servicing the interrupt. If
 *    ATN is negated, we bug out right away.
 *
 */

#include <linux/version.h>
#include <linux/stddef.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/ptrace.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/arch/fpga.h>

#undef	INNOVATOR_KEYB_DEBUG
#ifdef	INNOVATOR_KEYB_DEBUG
#define	dbg(format, arg...) printk(KERN_DEBUG "%s:%d: " format , \
				   __FUNCTION__ , __LINE__ , ## arg)
#define	entry()	printk(KERN_DEBUG "%s:%d: Entry\n" , __FUNCTION__ , __LINE__)
#define	exit()	printk(KERN_DEBUG "%s:%d: Exit\n" , __FUNCTION__ , __LINE__)
#define dump_packet(p, n)					\
	{							\
		int i;						\
		printk(KERN_DEBUG "%s:%d: %08x:" ,		\
		       __FUNCTION__ , __LINE__ , (int) p);	\
		for (i = 0; i < n; i += 1) {			\
			printk(" %02x", (int) p[i]);		\
		}						\
		printk("\n");					\
	}
#else
#define	dbg(format, arg...) do {} while (0)
#define	entry()	do {} while (0)
#define	exit()	do {} while (0)
#define dump_packet(p, n) do {} while (0)
#endif


#define	PFX	"innovator_ps2"
#define err(format, arg...)	printk(KERN_ERR PFX ": " format , ## arg)
#define info(format, arg...)	printk(KERN_INFO PFX ": " format , ## arg)
#define warn(format, arg...)	printk(KERN_WARNING PFX ": " format , ## arg)


/****************************************************************************/

/*
 * Synchronous communications timing parameters (Reference [1] pg 7-7)
 */

#define tMSA	5000	/* -/5ms	_SS to _ATN (master transfer) */
#define tMAC	100	/* 100us/5ms	_ATN to first clock pulse (master
					transfer) */
#define tMIB	150	/* 150us/5ms	Beginning of byte transfer to beginning
					of next byte transfer */
#define tSIB	150	/* 150us/5ms	Beginning of byte transfer to beginning
					of next byte transfer */
#define tMSP	100	/* -/100us	Last clock pulse of packet to _SS
					de-assertion */
#define tMNSA	100	/* -/100us	_SS de-assertion to _ATN de-assertion */
#define tMNEXT	120	/* 120uS/-	_ATN release to _SS re-assertion
					(master transfer) */
#define	tSAS	5000	/* -/5ms	_ATN to _SS (slave transfer) */
#define tSSC	100	/* 100us/5ms	_SS to first clock pulse (slave
					transfer) */
#define tSNA	100	/* -/100us	Last clock pulse of packet to _ATN
					de-assertion */
#define tSNAS	100	/* -/100us	_ATN release to _SS de-assertion */
#define tSNEXT	120	/* 120us/-	_SS release to _ATN re-assertion
					(slave transfer) */
#define tSCK	4	/* 4us/-	Clock period */
#define tSLOW	2	/* 2us/-	Clock LOW period */
#define tHOLD	200	/* 200ns/-	Master data hold time */
#define tSETUP	100	/* 100ns/-	Master data setup Time */
#define tSSETUP	500	/* -/500ns	Slave data setup time from clock
					falling edge */


/*
 * Protocol Headers (Reference [1], pg. 5-1):
 */


/* Protocols used in commands issued by the host: */
#define SIMPLE			0x80	/* Simple commands
					 * Common for both host and controller
					 * protocol headers.
					 */
#define WRITE_REGISTER_BIT	0x81	/* Write register bit */
#define READ_REGISTER_BIT	0x82	/* Read register bit */
#define WRITE_REGISTER		0x83	/* Write register */
#define READ_REGISTER		0x84	/* Read register */
#define WRITE_BLOCK		0x85	/* Write block */
#define READ_BLOCK		0x86	/* Read block */


/* Protocols used in responses, reports and alerts issued by the controller: */
#define REPORT_REGISTER_BIT	0x81	/* Report register bit & event alerts */
#define REPORT_REGISTER		0x83	/* Report register */
#define REPORT_BLOCK		0x85	/* Report block */
#define POINTING_REPORT		0x87	/* Pointing device data report */
#define KEYBOARD_REPORT		0x88	/* Keyboard device data report */


/* Simple Commands (Reference [1], pg 5-3): */
#define INITIALIZE		0x00	/* Forces the recipient to enter the
					 * known default power-on state.
					 */
#define INITIALIZATION_COMPLETE	0x01	/* Issued as a hand-shake response only
					 * to the "Initialize" command.
					 */
#define RESEND_REQUEST		0x05	/* Issued upon error in the reception
					 * of a package. The recipient resends
					 * the last transmitted packet.
					 */

/* Register offsets (Reference [1], pg 6-1 thru 6-9): */

#define REG_PM_COMM		0
#define REG_PM_STATUS		1
#define REG_PAGENO		255

/* Power management bits ((Reference [1], pg 6-10): */

#define SUS_STATE		0x2	/* in REG_PM_COMM */

/* Miscellaneous constants: */

#define X_MSB_SHIFT	(8-4)
#define X_MSB_MASK	(3<<4)
#define Y_MSB_SHIFT	(8-6)
#define Y_MSB_MASK	(3<<6)


#define JUNO_BLOCK_SIZE     32
#define JUNO_BUFFER_SIZE    256


/*
 * Errors:
 */

#define E_BAD_HEADER	1
#define E_BAD_LRC	2
#define E_ZERO_BYTES	3
#define E_BAD_VALUE	4
#define E_BAD_MODE	5
#define E_REPORT_MODE	6
#define E_BAD_ACK	7
#define E_BAD_DEVICE_ID	8
#define E_PKT_SZ	9


/*
 * Host/Controller Command/Response Formats:
 */

typedef struct _simple_t {
	u8 header;
	u8 cmd_code;
	u8 LRC;
} __attribute__ ((packed)) simple_t;

typedef struct _write_bit_t {
	u8 header;
	u8 offset;
	u8 value_bit;
	u8 LRC;
} __attribute__ ((packed)) write_bit_t;

typedef struct _read_bit_t {
	u8 header;
	u8 offset;
	u8 bit;
	u8 LRC;
} __attribute__ ((packed)) read_bit_t;

typedef struct _write_reg_t {
	u8 header;
	u8 offset;
	u8 value;
	u8 LRC;
} __attribute__ ((packed)) write_reg_t;

typedef struct _read_reg_t {
	u8 header;
	u8 offset;
	u8 LRC;
} __attribute__ ((packed)) read_reg_t;

typedef struct _write_block_t {
	u8 header;
	u8 offset;
	u8 length;
	u8 block[JUNO_BLOCK_SIZE + 1]; /* Hack: LRC is last element of block[] */
} __attribute__ ((packed)) write_block_t;

typedef struct _read_block_t {
	u8 header;
	u8 offset;
	u8 length;
	u8 LRC;
} __attribute__ ((packed)) read_block_t;

typedef struct _report_bit_t {
	u8 header;
	u8 offset;
	u8 value_bit;
	u8 LRC;
} __attribute__ ((packed)) report_bit_t;

typedef struct _report_reg_t {
	u8 header;
	u8 offset;
	u8 value;
	u8 LRC;
} __attribute__ ((packed)) report_reg_t;

typedef struct _report_block_t {
	u8 header;
	u8 offset;
	u8 length;
	u8 block[32];
	u8 LRC;
} __attribute__ ((packed)) report_block_t;

typedef struct _mse_report_t {
	u8 header;
	u8 buttons;
	u8 Xdisplacement;
	u8 Ydisplacement;
	u8 Zdisplacement;
	u8 LRC;
} __attribute__ ((packed)) mse_report_t;

typedef struct _kdb_report_t {
	u8 header;
	u8 keynum;		/* up > 0x80, down < 0x7E, all keys up 0x00 */
	u8 LRC;
} __attribute__ ((packed)) kdb_report_t;


static u8 buffer[JUNO_BUFFER_SIZE];

static void do_hid_tasklet(unsigned long);
DECLARE_TASKLET(hid_tasklet, do_hid_tasklet, 0);
static struct innovator_hid_dev *hid;

struct innovator_hid_dev {
	struct input_dev *mouse, *keyboard;
	int open;
	int irq_enabled;
};

/****************************************************************************/

/*
 * Low-level TI Innovator/OMAP1510 FPGA HID SPI interface helper functions:
 */

static u8
innovator_fpga_hid_rd(void)
{
	u8 val = inb(INNOVATOR_FPGA_HID_SPI);
	return val;
}

static void
innovator_fpga_hid_wr(u8 val)
{
	outb(val, INNOVATOR_FPGA_HID_SPI);
}

static void
innovator_fpga_hid_frob(u8 mask, u8 val)
{
	unsigned long flags;
	local_irq_save(flags);
	innovator_fpga_hid_wr((innovator_fpga_hid_rd() & ~mask) | val);
	local_irq_restore(flags);
}

static void
innovator_fpga_hid_set_bits(u8 x)
{
	innovator_fpga_hid_frob(x, x);
}

static void
SS(int value)
{
	innovator_fpga_hid_frob(OMAP1510_FPGA_HID_nSS, value ? OMAP1510_FPGA_HID_nSS : 0);
}

static void
SCLK(int value)
{
	innovator_fpga_hid_frob(OMAP1510_FPGA_HID_SCLK, value ? OMAP1510_FPGA_HID_SCLK : 0);
}

static void
MOSI(int value)
{
	innovator_fpga_hid_frob(OMAP1510_FPGA_HID_MOSI, value ? OMAP1510_FPGA_HID_MOSI : 0);
}

static u8
MISO(void)
{
	return ((innovator_fpga_hid_rd() & OMAP1510_FPGA_HID_MISO) ? 1 : 0);
}

static u8 
ATN(void)
{
	return ((innovator_fpga_hid_rd() & OMAP1510_FPGA_HID_ATN) ? 1 : 0);
}

static int
wait_for_ATN(int assert, int timeout)
{
	do {
		if (ATN() == assert)
			return 0;
		udelay(1);
	} while (timeout -= 1);
	return -1;
}

static u8
innovator_fpga_hid_xfer_byte(u8 xbyte)
{
	int i;
	u8 rbyte;

	for (rbyte = 0, i = 7; i >= 0; i -= 1) {
		SCLK(0);
		MOSI((xbyte >> i) & 1);
		udelay(tSLOW);
		SCLK(1);
		rbyte = (rbyte << 1) | MISO();
		udelay(tSLOW);
	}

	return rbyte;
}

static void
innovator_fpga_hid_reset(void)
{
	innovator_fpga_hid_wr(OMAP1510_FPGA_HID_SCLK | OMAP1510_FPGA_HID_MOSI);
	mdelay(1);
	innovator_fpga_hid_set_bits(OMAP1510_FPGA_HID_RESETn);
}


/*****************************************************************************

  Refer to Reference [1], Chapter 7 / Low-level communications, Serial
  Peripheral Interface (SPI) implementation Host (master) packet
  transmission timing, pg. 7-3, for timing and implementation details
  for spi_xmt().

 *****************************************************************************/

int
spi_xmt(u8 * p, u8 n)
{
	unsigned long flags;

	dump_packet(p, n);
	local_irq_save(flags);
	disable_irq(OMAP1510_INT_FPGA_ATN);

	if (ATN()) {
		/* Oops, we have a collision. */
		enable_irq(OMAP1510_INT_FPGA_ATN);
		local_irq_restore(flags);
		dbg("Protocol error: ATN is asserted\n");
		return -EAGAIN;
	}

	SS(1);

	if (wait_for_ATN(1, tMSA) < 0) {
		SS(0);
		enable_irq(OMAP1510_INT_FPGA_ATN);
		local_irq_restore(flags);
		dbg("timeout waiting for ATN assertion\n");
		return -EREMOTEIO;
	}

	udelay(tMAC);

	while (n--) {
		innovator_fpga_hid_xfer_byte(*p++);
		if (n) {
			udelay(tMIB - 8 * tSCK);
		}
	}

	MOSI(1);	/* Set MOSI to idle high. */

	/* NOTE: The data sheet does not specify a minimum delay
	 * here. But innovator_fpga_hid_xfer_byte() gives us a half-clock
	 * delay (tSLOW) after the last bit is sent. So I'm happy with
	 * that.
	 */

	SS(0);

	if (wait_for_ATN(0, tMNSA) < 0) {
		enable_irq(OMAP1510_INT_FPGA_ATN);
		local_irq_restore(flags);
		dbg("timeout waiting for ATN negation\n");
		return -EREMOTEIO;
	}

	udelay(tMNEXT);
	enable_irq(OMAP1510_INT_FPGA_ATN);
	local_irq_restore(flags);
	return 0;
}


/*****************************************************************************

  Refer to Reference [1],  Chapter 7 / Low-level communications, Serial
  Peripheral Interface (SPI) implementation, Slave packet transmission
  timing, pg. 7-5, for timing and implementation details for spi_rcv().

 *****************************************************************************/

int
spi_rcv(u8 * p, int len)
{
	unsigned long flags;
	int ret = 0;

	if (len > 256) {
		/* Limit packet size to something reasonable */
		return -1;
	}

	local_irq_save(flags);

	if (wait_for_ATN(1, tMSA) < 0) {
		local_irq_restore(flags);
		dbg("Protocol error: ATN is not asserted\n");
		return -EREMOTEIO;
	}

	SS(1);

	udelay(tSSC);

	while (ATN()) {
		if (ret >= len) {
			err("over run error\n");
			ret = -1;
			break;
		}
		p[ret++] = innovator_fpga_hid_xfer_byte(0xff);
		udelay(tSNA);	/* Wait long enough to detect negation of ATN
				 * after last clock pulse of packet.
				 *
				 * NOTE: Normally, we need a minimum delay of
				 *	 tSIB between the start of one byte
				 *	 and the start of the next. However,
				 *	 we also need to wait long enough
				 *	 for the USAR to negate ATN before
				 *	 starting the next byte. So we use
				 *	 max(tSIB - 8 * tSCK, tSNA) here to
				 *	 satisfy both constraints.
				 */
	}

	SS(0);	/* NOTE: The data sheet does not specify a minimum delay
		 * here. But innovator_fpga_hid_xfer_byte() gives us a
		 * half-clock delay (tSLOW) after the last bit is sent. So
		 * I'm happy with that (rather than no delay at all : ).
		 */


	udelay(tSNEXT);	/* This isn't quite right. Assertion of ATN after
			 * negation of SS is an USAR timing constraint.
			 * What we need here is a spec for the minimum
			 * delay from SS negation to SS assertion. But
			 * for now, just use this brain dead delay.
			 */

	local_irq_restore(flags);

	if (ret > 0) {
		dump_packet(p, ret);
	}

	return ret;
}


/*****************************************************************************
  Calculate Host/Controller Command/Response Longitudinal Redundancy Check (LRC)

  The algorithm implemented in calculate_LRC() below is taken directly from
  the reference [1], Chapter 7 / Low-level communications, LRC (Longitudinal
  Redundancy Check), pg 5-10.

 *****************************************************************************/

static u8
calculate_LRC(u8 * p, int n)
{
	u8 LRC;
	int i;

	/*
	 * Init the LRC using the first two message bytes.
	 */
	LRC = p[0] ^ p[1];

	/*
	 * Update the LRC using the remainder of the p.
	 */
	for (i = 2; i < n; i++)
		LRC ^= p[i];

	/*
	 * If the MSB is set then clear the MSB and change the next
	 * most significant bit
	 */
	if (LRC & 0x80)
		LRC ^= 0xC0;

	return LRC;
}


/*
 * Controller response helper functions:
 */

static inline int
report_mouse(mse_report_t * p, int n)
{
	if (p->header != POINTING_REPORT)
		return -E_BAD_HEADER;

	if (n != sizeof(mse_report_t))
		return -E_PKT_SZ;

	return (p->LRC != calculate_LRC((u8 *) p, sizeof(mse_report_t) - 1)) ?
		-E_BAD_LRC : POINTING_REPORT;
}

static inline int
report_keyboard(kdb_report_t * p, int n)
{
	if (p->header != KEYBOARD_REPORT)
		return -E_BAD_HEADER;

	if (n != sizeof(kdb_report_t))
		return -E_PKT_SZ;

	return (p->LRC != calculate_LRC((u8 *) p, sizeof(kdb_report_t) - 1)) ?
		-E_BAD_LRC : KEYBOARD_REPORT;
}


/*
 * Miscellaneous helper functions:
 */

static inline int
report_type(u8 * type)
{
	/* check the header to find out what kind of report it is */
	if ((*type) == KEYBOARD_REPORT)
		return KEYBOARD_REPORT;
	else if ((*type) == POINTING_REPORT)
		return POINTING_REPORT;
	else
		return -E_BAD_HEADER;
}

static inline int
report_async(void * p, int n)
{
	int ret;

	if ((ret = spi_rcv((u8 *) p, n)) < 0)
		return ret;

	if (report_type((u8 *) p) == POINTING_REPORT)
		ret = report_mouse((mse_report_t *) p, ret);
	else if (report_type((u8 *) p) == KEYBOARD_REPORT)
		ret = report_keyboard((kdb_report_t *) p, ret);

	return ret;
}

/*
 * Host command helper functions:
 */

#if	0
/* REVISIT/TODO: Wrapper for command/response with resend handing. */
static int
spi_xfer(u8 * optr, u8 osz, u8 * iptr, u8 isz)
{
	static u8 buf[256];
	int ret;
	int xretries = 3;

	do {
		if (optr != NULL && osz) {
			do {
				ret = spi_xmt((u8 *) optr, osz);
			} while (ret < 0);
		}

		ret = spi_rcv((u8 *) buf, 256);

		if (ret == -EREMOTEIO) {
			if (iptr == NULL) {
				break;
			}
		}
	} while (xretries--);

	return ret;
}
#endif

/* REVISIT: Enable these when/if additional Juno features are required. */
static inline int
simple(u8 cmd)
{
	static simple_t p;
	int ret;

	p.header = SIMPLE;
	p.cmd_code = cmd;
	p.LRC = calculate_LRC((u8 *) & p, sizeof(p) - 1);

	if ((ret = spi_xmt((u8 *) & p, sizeof(p))) < 0)
		return ret;

	if ((ret = spi_rcv((u8 *) & p, sizeof(p))) < 0)
		return ret;

	if (ret == 0)
		return -E_ZERO_BYTES;

	if (ret != sizeof(p))
		return -E_PKT_SZ;

	if (p.header != SIMPLE)
		return -E_BAD_HEADER;

	if (p.LRC != calculate_LRC((u8 *) & p, sizeof(p) - 1))
		return -E_BAD_LRC;

	/* REVISIT: Need to check or return response code here? */
}

static inline int
write_bit(u8 offset, u8 bit, u8 value)
{
	static write_bit_t p;

	p.header = WRITE_REGISTER_BIT;
	p.offset = offset;
	p.value_bit = (bit << 1) | (value & 1);
	p.LRC = calculate_LRC((u8 *) & p, sizeof(p) - 1);

	return spi_xmt((u8 *) & p, sizeof(p));
}

static inline int
read_bit(u8 offset, u8 bit, u8 * data)
{
	static read_bit_t p;
	static report_bit_t q;
	int ret;

	p.header = READ_REGISTER_BIT;
	p.offset = offset;
	p.bit = bit;
	p.LRC = calculate_LRC((u8 *) & p, sizeof(p) - 1);

	if ((ret = spi_xmt((u8 *) & p, sizeof(p))) < 0)
		return ret;

	if ((ret = spi_rcv((u8 *) & q, sizeof(q))) < 0)
		return ret;

	if (ret == 0)
		return -E_ZERO_BYTES;

	if (ret != sizeof(q))
		return -E_PKT_SZ;

	if (q.header != REPORT_REGISTER_BIT)
		return -E_BAD_HEADER;

	if (q.LRC != calculate_LRC((u8 *) & q, sizeof(q) - 1))
		return -E_BAD_LRC;

	*data = q.value_bit;

	return 0;
}

static inline int
write_reg(u8 offset, u8 value)
{
	static write_reg_t p;

	p.header = WRITE_REGISTER;
	p.offset = offset;
	p.value = value;
	p.LRC = calculate_LRC((u8 *) & p, sizeof(p) - 1);

	return spi_xmt((u8 *) & p, sizeof(p));
}

static inline int
read_reg(u8 offset, u8 * data)
{
	static read_reg_t p;
	static report_reg_t q;
	int ret;

	p.header = READ_REGISTER;
	p.offset = offset;
	p.LRC = calculate_LRC((u8 *) & p, sizeof(p) - 1);

	if ((ret = spi_xmt((u8 *) & p, sizeof(p))) < 0)
		return ret;

	if ((ret = spi_rcv((u8 *) & q, sizeof(q))) < 0)
		return ret;

	if (ret == 0)
		return -E_ZERO_BYTES;

	if (ret != sizeof(q))
		return -E_PKT_SZ;

	if (q.header != REPORT_REGISTER)
		return -E_BAD_HEADER;

	if (q.LRC != calculate_LRC((u8 *) & q, sizeof(q) - 1))
		return -E_BAD_LRC;

	*data = q.value;

	return 0;
}

static inline int
write_block(u8 offset, u8 length, u8 * block)
{
	static write_block_t p;

	p.header = WRITE_BLOCK;
	p.offset = offset;
	p.length = length;
	memcpy(&p.block, block, length);
	p.block[length] = calculate_LRC((u8 *) & p, 3 + length);

	return spi_xmt((u8 *) & p, 4 + length);
}

static inline int
read_block(u8 offset, u8 length, u8 * buf)
{
	static read_block_t p;
	static report_block_t q;
	int ret;

	p.header = READ_BLOCK;
	p.offset = offset;
	p.length = length;
	p.LRC = calculate_LRC((u8 *) & p, sizeof(p) - 1);

	if ((ret = spi_xmt((u8 *) & p, sizeof(p))) < 0)
		return ret;

	if ((ret = spi_rcv((u8 *) & q, sizeof(q))) < 0)
		return ret;

	if (ret == 0)
		return -E_ZERO_BYTES;

	if (ret != sizeof(4 + q.length))
		return -E_PKT_SZ;

	if (q.header != REPORT_BLOCK)
		return -E_BAD_HEADER;

	if (q.block[q.length] != calculate_LRC((u8 *) & q, 3 + q.length))
		return -E_BAD_LRC;

	if (length != q.length)
		return -E_PKT_SZ;

	memcpy(buf, &q.block, length);

	return 0;
}

#ifdef	INNOVATOR_KEYB_DEBUG
static void
ctrl_dump_regs(void)
{
	int i;
	int n;

	for (i = 0; i < 256; i += 8) {
		read_block(i, 16, buffer);
		mdelay(1);
	}
}
#endif

/*****************************************************************************/

static void
process_pointing_report(struct innovator_hid_dev *hid, u8 * buffer)
{
	static int prev_x, prev_y, prev_btn;
	int x, y, btn;
	hid->keyboard = input_allocate_device();
	hid->mouse = input_allocate_device();

	if (buffer[1] & (1 << 3)) {
		/* relative pointing device report */
		x = buffer[2];
		y = buffer[3];

		/* check the sign and convert from 2's complement if negative */
		if (buffer[1] & (1<<4))
			x = ~(-x) - 255;

		/* input driver wants -y */
		if (buffer[1] & (1<<5))
			y = -(~(-y) - 255);
		else
			y = -y;

		input_report_key(hid->mouse,
				 BTN_LEFT, buffer[1] & (1<<0));
		input_report_key(hid->mouse,
				 BTN_RIGHT, buffer[1] & (1<<1));
		input_report_key(hid->mouse,
				 BTN_MIDDLE, buffer[1] & (1<<2));
		input_report_rel(hid->mouse, REL_X, x);
		input_report_rel(hid->mouse, REL_Y, y);
	} else {
		/* REVISIT: Does this work? */
		/* absolute pointing device report */
		x = buffer[2] + ((buffer[1] & X_MSB_MASK) << X_MSB_SHIFT);
		y = buffer[3] + ((buffer[1] & Y_MSB_MASK) << Y_MSB_SHIFT);
		btn = buffer[1] & (1<<0);

		if ((prev_x == x) && (prev_y == y)
		    && (prev_btn == btn))
			return;

		input_report_key(hid->mouse, BTN_LEFT, btn);
		input_report_abs(hid->mouse, ABS_X, x);
		input_report_abs(hid->mouse, ABS_Y, y);
		prev_x = x;
		prev_y = y;
		prev_btn = btn;
	}
	input_sync(hid->mouse);
	dbg("HID X: %d Y: %d Functions: %x\n", x, y, buffer[1]);
}

/*
 * Reference [1], Appendix A, Semtech standard PS/2 key number definitions,
 * pgs. A-1 through A-3. The following table lists standard PS/2 key numbers
 * used by the Juno® 01 keyboard manager.
 *
 * NOTES:
 * 1. The following table indices are E0 codes which require special handling:
 *	53..62, 77..78, 94, 96, 100, 102..104, 108..110
 * 2. The following table indices are E1 codes which require special handling:
 *	101
 */

static unsigned char usar2scancode[128] = {
	0x00, 0x29, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1a, 0x1b, 0x2b, 0x1e, 0x1f, 0x20,
	0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
	0x1c, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32,
	0x33, 0x34, 0x35, 0x39, 0x01, 0x52, 0x53, 0x4b,
	0x47, 0x4f, 0x48, 0x50, 0x49, 0x51, 0x4d, 0x37,
	0x4e, 0x4f, 0x50, 0x51, 0x4b, 0x4c, 0x4d, 0x47,
	0x48, 0x49, 0x52, 0x53, 0x4a, 0x1c, 0x35, 0x3b,
	0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
	0x44, 0x57, 0x58, 0x2a, 0x36, 0x38, 0x38, 0x1d,
	0x1d, 0x3a, 0x45, 0x46, 0x2a, 0x1d, 0x5b, 0x5c,
	0x5d, 0xff, 0x00, 0x00, 0x5e, 0x5f, 0x63, 0x70,
	0x7b, 0x79, 0x7d, 0x73, 0x5b, 0x5c, 0x5d, 0x63,
	0x65, 0x66, 0x68, 0x69, 0x6b, 0x56, 0x54, 0x00
};

/*
 * The following are bit masks used to encode E0 scan codes which
 * require special handling. However, scan codes 100 and 101 are
 * excludable here since they each require unique multi-byte scan
 * code translations and are therefore dealt with individually via
 * handle_print_scr() and handle_pause() respectively below.
 */

static unsigned long int e0_codes1 = 0x030003ff; /* scan codes 53..84 */
static unsigned long int e0_codes2 = 0x038e0a00; /* scan codes 85..116 */

static void
handle_print_scr(int up)
{
	if (up) {
		input_report_key(hid->keyboard, 0xe0, 1);
		input_report_key(hid->keyboard, 0xb7, 1);
		input_report_key(hid->keyboard, 0xe0, 1);
		input_report_key(hid->keyboard, 0xaa, 1);
	} else {
		input_report_key(hid->keyboard, 0xe0, 0);
		input_report_key(hid->keyboard, 0x2a, 0);
		input_report_key(hid->keyboard, 0xe0, 0);
		input_report_key(hid->keyboard, 0x37, 0);
	}
}

static void
handle_pause(void)
{
	input_report_key(hid->keyboard, 0xe1, 0);
	input_report_key(hid->keyboard, 0x1d, 0);
	input_report_key(hid->keyboard, 0x45, 0);
	input_report_key(hid->keyboard, 0xe1, 0);
	input_report_key(hid->keyboard, 0x9d, 0);
	input_report_key(hid->keyboard, 0xc5, 0);
}

static void
process_keyboard_report(struct innovator_hid_dev *hid, u8 * buffer)
{
	unsigned char ch = buffer[1] & 0x7f;
	int up = buffer[1] & 0x80 ? 1 : 0;
	int is_e0 = 0;
	hid->keyboard = input_allocate_device();
	hid->mouse = input_allocate_device();

	if ((ch == 106) || (ch == 107))
		return;		/* no code */

	if (ch == 100) {
		handle_print_scr(up);
		return;
	}

	if (ch == 101) {
		handle_pause();
		return;
	}

	if ((ch >= 53) && (ch <= 84)) {
		/* first block of e0 codes */
		is_e0 = e0_codes1 & (1 << (ch - 53));
	} else if ((ch >= 85) && (ch <= 116)) {
		/* second block of e0 codes */
		is_e0 = e0_codes2 & (1 << (ch - 85));
	}

	if (is_e0) {
		input_report_key(hid->keyboard, 0xe0, !up);
	}
	input_report_key(hid->keyboard, usar2scancode[ch], !up);
	input_sync(hid->keyboard);
}

static irqreturn_t
innovator_hid_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	if (ATN()) {
		disable_irq(OMAP1510_INT_FPGA_ATN);
		tasklet_schedule(&hid_tasklet);
	}
	return IRQ_HANDLED;
}

static void
do_hid_tasklet(unsigned long unused)
{
	int ret;
	if ((ret = report_async(buffer, 256)) == -1) {
		dbg("Error: Bad Juno return value: %d\n", ret);
	} else if (ret == KEYBOARD_REPORT) {
		process_keyboard_report(hid, buffer);
	} else if (ret == POINTING_REPORT) {
		process_pointing_report(hid, buffer);
	} else {
		dbg("ERROR: bad report\n");
	}
	enable_irq(OMAP1510_INT_FPGA_ATN);
}

static int
innovator_hid_open(struct input_dev *dev)
{
	if (hid->open++)
		return 0;

	if (request_irq(OMAP1510_INT_FPGA_ATN, (void *) innovator_hid_interrupt,
			IRQF_DISABLED, PFX, hid) < 0)
		return -EINVAL;

	return 0;
}

static void
innovator_hid_close(struct input_dev *dev)
{
	if (!--hid->open)
		return;

	if (hid == NULL)
		return;

	kfree(hid);
}

static int innovator_ps2_remove(struct device *dev)
{
	return 0;
}

static void innovator_ps2_device_release(struct device *dev)
{
	/* Nothing */
}

static int innovator_ps2_suspend(struct device *dev, pm_message_t state)
{
	u8 pmcomm = 0;

	/*
	 * Set SUS_STATE in REG_PM_COMM (Page 0 R0).  This will cause
	 * PM_MOD bits of REG_PM_STATUS to show suspended state,
	 * but the SUS_STAT bit of REG_PM_STATUS will continue to
	 * reflect the state of the _HSUS pin.
	 */

	if (write_reg(REG_PAGENO, 0) < 0)
		printk("ps2 suspend: write_reg REG_PAGENO error\n");

	if (read_reg(REG_PM_COMM, &pmcomm) < 0)
		printk("ps2 suspend: read_reg REG_PM_COMM error\n");
		
	if (write_reg(REG_PM_COMM, pmcomm | SUS_STATE) < 0)
		printk("ps2 suspend: write_reg REG_PM_COMM error\n");

	return 0;
}

static int innovator_ps2_resume(struct device *dev)
{
	u8 pmcomm = 0;

	/*
	 * Clear SUS_STATE from REG_PM_COMM (Page 0 R0).
	 */

	if (write_reg(REG_PAGENO, 0) < 0)
		printk("ps2 resume: write_reg REG_PAGENO error\n");

	if (read_reg(REG_PM_COMM, &pmcomm) < 0)
		printk("ps2 resume: read_reg REG_PM_COMM error\n");

	if (write_reg(REG_PM_COMM, pmcomm & ~SUS_STATE) < 0)
		printk("ps2 resume: write_reg REG_PM_COMM error\n");

	return 0;
}

static struct device_driver innovator_ps2_driver = {
	.name		= "innovator_ps2",
	.bus		= &platform_bus_type,
	.remove		= innovator_ps2_remove,
	.suspend	= innovator_ps2_suspend,
	.resume		= innovator_ps2_resume,
};

static struct platform_device innovator_ps2_device = {
	.name		= "ps2",
	.id		= -1,
	.dev = {
		.driver		= &innovator_ps2_driver,
		.release	= innovator_ps2_device_release,
	},
};

static int __init
innovator_kbd_init(void)
{
	int i;
	info("Innovator PS/2 keyboard/mouse driver v1.0\n");

	innovator_fpga_hid_reset();

	if ((hid = kmalloc(sizeof(struct innovator_hid_dev),
	     GFP_KERNEL)) == NULL) {
		warn("unable to allocate space for HID device\n");
		return -ENOMEM;
	}

	/* setup the mouse */
	memset(hid, 0, sizeof(struct innovator_hid_dev));
	hid->mouse = input_allocate_device();
	hid->mouse->evbit[0] = BIT(EV_KEY) | BIT(EV_REL);
	hid->mouse->keybit[BIT_WORD(BTN_MOUSE)] =
	    BIT(BTN_LEFT) | BIT(BTN_RIGHT) |
	    BIT(BTN_MIDDLE) | BIT(BTN_TOUCH);
	hid->mouse->relbit[0] = BIT(REL_X) | BIT(REL_Y);
	hid->mouse->private = hid;
	hid->mouse->open = innovator_hid_open;
	hid->mouse->close = innovator_hid_close;
	hid->mouse->name = "innovator_mouse";
	hid->mouse->id.bustype = 0;
	hid->mouse->id.vendor = 0;
	hid->mouse->id.product = 0;
	hid->mouse->id.version = 0;
       hid->keyboard = input_allocate_device();
	hid->keyboard->evbit[0] = BIT(EV_KEY) | BIT(EV_REP);
       hid->keyboard->keycodesize = sizeof(unsigned char);
       hid->keyboard->keycodemax = ARRAY_SIZE(usar2scancode);
	for(i = 0; i < 128; i++)
		set_bit(usar2scancode[i], hid->keyboard->keybit);
	hid->keyboard->private = hid;
	hid->keyboard->open = innovator_hid_open;
	hid->keyboard->close = innovator_hid_close;
	hid->keyboard->name = "innovator_keyboard";
	hid->keyboard->id.bustype = 0;
	hid->keyboard->id.vendor = 0;
	hid->keyboard->id.product = 0;
	hid->keyboard->id.version = 0;
	input_register_device(hid->mouse);
	input_register_device(hid->keyboard);
	innovator_hid_open(hid->mouse);
	innovator_hid_open(hid->keyboard);

	if (driver_register(&innovator_ps2_driver) != 0)
		printk(KERN_ERR "Driver register failed for innovator_ps2\n");

	if (platform_device_register(&innovator_ps2_device) != 0) {
		printk(KERN_ERR "Device register failed for ps2\n");
		driver_unregister(&innovator_ps2_driver);
	}

#ifdef	INNOVATOR_KEYB_DEBUG
	ctrl_dump_regs();
#endif
	return 0;
}

static void __exit
innovator_kbd_exit(void)
{
	input_unregister_device(hid->mouse);
	input_unregister_device(hid->keyboard);
	free_irq(OMAP1510_INT_FPGA_ATN, hid);
	if (hid != NULL)
		kfree(hid);
	driver_unregister(&innovator_ps2_driver);
	platform_device_unregister(&innovator_ps2_device);
	return;
}

module_init(innovator_kbd_init);
module_exit(innovator_kbd_exit);

MODULE_AUTHOR("George G. Davis <gdavis@mvista.com>");
MODULE_DESCRIPTION("Innovator PS/2 Driver");
MODULE_LICENSE("GPL");
