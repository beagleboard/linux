/*
 * STI RX FIFO Support
 *
 * Copyright (C) 2005, 2006 Nokia Corporation
 * Written by:  Paul Mundt <paul.mundt@nokia.com> and
 *		Roman Tereshonkov <roman.tereshonkov@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/module.h>
#include <mach/sti.h>

#define STI_READ_BUFFER_SIZE	1024
#define sti_buf_pos(pos)	((sti_crb->bufpos + (pos)) % \
				 STI_READ_BUFFER_SIZE)

static struct sti_cycle_buffer {
	int bufpos;
	int datalen;
	unsigned char *buf;
} *sti_crb;

/**
 * sti_read_packet - STI read packet (read an entire STI packet)
 * @buf: Buffer to store the packet.
 * @maxsize: Maximum size requested.
 *
 * This reads in a single completed STI packet from the RX FIFOs and
 * places it in @buf for further processing.
 *
 * The return value is < 0 on error, and >= 0 for the number of bytes
 * actually read. As per the STI specification, we require a 0xC1 to
 * indicate the end of the packet, and we don't return the packet until
 * we've read the entire thing in.
 *
 * Due to the size of the FIFOs, it's unrealistic to constantly drain
 * this for 1 or 2 bytes at a time, so we assemble it here and return
 * the whole thing.
 */
int sti_read_packet(unsigned char *buf, int maxsize)
{
	unsigned int pos;

	if (unlikely(!buf))
		return -EINVAL;
	if (!sti_crb->datalen)
		return 0;

	pos = sti_buf_pos(sti_crb->datalen - 1);
	/* End of packet */
	if (sti_crb->buf[pos] == 0xC1) {
		int i;

		for (i = 0; i < sti_crb->datalen && i < maxsize; i++) {
			pos = sti_buf_pos(i);
			buf[i] = sti_crb->buf[pos];
		}

		sti_crb->bufpos = sti_buf_pos(i);
		sti_crb->datalen -= i;

		return i;
	}

	return 0;
}
EXPORT_SYMBOL(sti_read_packet);

static void sti_fifo_irq(unsigned long arg)
{
	/* If there is data read it */
	while (!(sti_readl(STI_RX_STATUS) & STI_RXFIFO_EMPTY)) {
		unsigned int pos = sti_buf_pos(sti_crb->datalen);

		sti_crb->buf[pos] = sti_readl(STI_RX_DR);
		sti_crb->datalen++;
	}

	sti_ack_irq(STI_RX_INT);
}

static int __init sti_fifo_init(void)
{
	unsigned int size;
	int ret;

	size = sizeof(struct sti_cycle_buffer) + STI_READ_BUFFER_SIZE;
	sti_crb = kmalloc(size, GFP_KERNEL);
	if (!sti_crb)
		return -ENOMEM;

	sti_crb->bufpos = sti_crb->datalen = 0;
	sti_crb->buf = (unsigned char *)(sti_crb + sizeof(*sti_crb));

	ret = sti_request_irq(STI_RX_INT, sti_fifo_irq, 0);
	if (ret != 0)
		kfree(sti_crb);

	return ret;
}

static void __exit sti_fifo_exit(void)
{
	sti_free_irq(STI_RX_INT);
	kfree(sti_crb);
}

module_init(sti_fifo_init);
module_exit(sti_fifo_exit);

MODULE_AUTHOR("Paul Mundt, Roman Tereshonkov");
MODULE_LICENSE("GPL");
