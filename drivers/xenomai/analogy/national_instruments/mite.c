/*
 * Hardware driver for NI Mite PCI interface chip
 *
 * Copyright (C) 1999 David A. Schleef <ds@schleef.org>
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * This code is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * The NI Mite driver was originally written by Tomasz Motylewski
 * <...>, and ported to comedi by ds.
 *
 * References for specifications:
 *
 * 321747b.pdf  Register Level Programmer Manual (obsolete)
 * 321747c.pdf  Register Level Programmer Manual (new)
 * DAQ-STC reference manual
 *
 * Other possibly relevant info:
 *
 * 320517c.pdf  User manual (obsolete)
 * 320517f.pdf  User manual (new)
 * 320889a.pdf  delete
 * 320906c.pdf  maximum signal ratings
 * 321066a.pdf  about 16x
 * 321791a.pdf  discontinuation of at-mio-16e-10 rev. c
 * 321808a.pdf  about at-mio-16e-10 rev P
 * 321837a.pdf  discontinuation of at-mio-16de-10 rev d
 * 321838a.pdf  about at-mio-16de-10 rev N
 *
 * ISSUES:
 */

#include <linux/module.h>
#include "mite.h"

#ifdef CONFIG_DEBUG_MITE
#define MDPRINTK(fmt, args...) rtdm_printk(fmt, ##args)
#else /* !CONFIG_DEBUG_MITE */
#define MDPRINTK(fmt, args...)
#endif /* CONFIG_DEBUG_MITE */

static LIST_HEAD(mite_devices);

static struct pci_device_id mite_id[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_NATINST, PCI_ANY_ID), },
	{0, }
};

static int mite_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int i, err = 0;
	struct mite_struct *mite;

	mite = kmalloc(sizeof(struct mite_struct), GFP_KERNEL);
	if(mite == NULL)
		return -ENOMEM;

	memset(mite, 0, sizeof(struct mite_struct));

	rtdm_lock_init(&mite->lock);

	mite->pcidev = dev;
	if (pci_enable_device(dev) < 0) {
		__a4l_err("error enabling mite\n");
		err = -EIO;
		goto out;
	}

	for(i = 0; i < MAX_MITE_DMA_CHANNELS; i++) {
		mite->channels[i].mite = mite;
		mite->channels[i].channel = i;
		mite->channels[i].done = 1;
	}

	list_add(&mite->list, &mite_devices);

out:
	if (err < 0)
		kfree(mite);

	return err;
}

static void mite_remove(struct pci_dev *dev)
{
	struct list_head *this;

	list_for_each(this, &mite_devices) {
		struct mite_struct *mite =
			list_entry(this, struct mite_struct, list);

		if(mite->pcidev == dev) {
			list_del(this);
			kfree(mite);
			break;
		}
	}
}

static struct pci_driver mite_driver = {
	.name = "analogy_mite",
	.id_table = mite_id,
	.probe = mite_probe,
	.remove = mite_remove,
};

int a4l_mite_setup(struct mite_struct *mite, int use_iodwbsr_1)
{
	unsigned long length;
	resource_size_t addr;
	int i;
	u32 csigr_bits;
	unsigned unknown_dma_burst_bits;

	__a4l_dbg(1, drv_dbg, "starting setup...\n");

	pci_set_master(mite->pcidev);

	if (pci_request_regions(mite->pcidev, "mite")) {
		__a4l_err("failed to request mite io regions\n");
		return -EIO;
	};

	/* The PCI BAR0 is the Mite */
	addr = pci_resource_start(mite->pcidev, 0);
	length = pci_resource_len(mite->pcidev, 0);
	mite->mite_phys_addr = addr;
	mite->mite_io_addr = ioremap(addr, length);
	if (!mite->mite_io_addr) {
		__a4l_err("failed to remap mite io memory address\n");
		pci_release_regions(mite->pcidev);
		return -ENOMEM;
	}

	__a4l_dbg(1, drv_dbg, "bar0(mite) 0x%08llx mapped to %p\n",
		  (unsigned long long)mite->mite_phys_addr,
		  mite->mite_io_addr);


	/* The PCI BAR1 is the DAQ */
	addr = pci_resource_start(mite->pcidev, 1);
	length = pci_resource_len(mite->pcidev, 1);
	mite->daq_phys_addr = addr;
	mite->daq_io_addr = ioremap(mite->daq_phys_addr, length);
	if (!mite->daq_io_addr) {
		__a4l_err("failed to remap daq io memory address\n");
		pci_release_regions(mite->pcidev);
		return -ENOMEM;
	}

	__a4l_dbg(1, drv_dbg, "bar0(daq) 0x%08llx mapped to %p\n",
		  (unsigned long long)mite->daq_phys_addr,
		  mite->daq_io_addr);

	if (use_iodwbsr_1) {
		__a4l_dbg(1, drv_dbg, "using I/O Window Base Size register 1\n");
		writel(0, mite->mite_io_addr + MITE_IODWBSR);
		writel(mite->
		       daq_phys_addr | WENAB |
		       MITE_IODWBSR_1_WSIZE_bits(length),
		       mite->mite_io_addr + MITE_IODWBSR_1);
		writel(0, mite->mite_io_addr + MITE_IODWCR_1);
	} else {
		writel(mite->daq_phys_addr | WENAB,
		       mite->mite_io_addr + MITE_IODWBSR);
	}

	/* Make sure dma bursts work.  I got this from running a bus analyzer
	   on a pxi-6281 and a pxi-6713.  6713 powered up with register value
	   of 0x61f and bursts worked.  6281 powered up with register value of
	   0x1f and bursts didn't work.  The NI windows driver reads the register,
	   then does a bitwise-or of 0x600 with it and writes it back.
	*/
	unknown_dma_burst_bits =
		readl(mite->mite_io_addr + MITE_UNKNOWN_DMA_BURST_REG);
	unknown_dma_burst_bits |= UNKNOWN_DMA_BURST_ENABLE_BITS;
	writel(unknown_dma_burst_bits,
	       mite->mite_io_addr + MITE_UNKNOWN_DMA_BURST_REG);

	csigr_bits = readl(mite->mite_io_addr + MITE_CSIGR);
	mite->num_channels = mite_csigr_dmac(csigr_bits);
	if (mite->num_channels > MAX_MITE_DMA_CHANNELS) {
		__a4l_err("MITE: bug? chip claims to have %i dma channels. "
			  "Setting to %i.\n",
			  mite->num_channels, MAX_MITE_DMA_CHANNELS);
		mite->num_channels = MAX_MITE_DMA_CHANNELS;
	}

	__a4l_dbg(1, drv_dbg, " version = %i, type = %i, mite mode = %i, "
		  "interface mode = %i\n",
		  mite_csigr_version(csigr_bits),
		  mite_csigr_type(csigr_bits),
		  mite_csigr_mmode(csigr_bits),
		  mite_csigr_imode(csigr_bits));
	__a4l_dbg(1, drv_dbg, " num channels = %i, write post fifo depth = %i, "
		  "wins = %i, iowins = %i\n",
		  mite_csigr_dmac(csigr_bits),
		  mite_csigr_wpdep(csigr_bits),
		  mite_csigr_wins(csigr_bits),
		  mite_csigr_iowins(csigr_bits));

	for (i = 0; i < mite->num_channels; i++) {
		/* Registers the channel as a free one */
		mite->channel_allocated[i] = 0;
		/* Reset the channel */
		writel(CHOR_DMARESET, mite->mite_io_addr + MITE_CHOR(i));
		/* Disable interrupts */
		writel(CHCR_CLR_DMA_IE | CHCR_CLR_LINKP_IE | CHCR_CLR_SAR_IE |
		       CHCR_CLR_DONE_IE | CHCR_CLR_MRDY_IE | CHCR_CLR_DRDY_IE |
		       CHCR_CLR_LC_IE | CHCR_CLR_CONT_RB_IE,
		       mite->mite_io_addr + MITE_CHCR(i));

		__a4l_dbg(1, drv_dbg, "channel[%d] initialized\n", i);
	}

	mite->used = 1;

	return 0;
}

void a4l_mite_unsetup(struct mite_struct *mite)
{
	if (!mite)
		return;

	if (mite->mite_io_addr) {
		iounmap(mite->mite_io_addr);
		mite->mite_io_addr = NULL;
	}

	if (mite->daq_io_addr) {
		iounmap(mite->daq_io_addr);
		mite->daq_io_addr = NULL;
	}

	if(mite->used)
		pci_release_regions( mite->pcidev );

	mite->used = 0;
}

void a4l_mite_list_devices(void)
{
	struct list_head *this;

	printk("Analogy: MITE: Available NI device IDs:");
	list_for_each(this, &mite_devices) {
		struct mite_struct *mite =
			list_entry(this, struct mite_struct, list);

		printk(" 0x%04x", mite_device_id(mite));
		if(mite->used)
			printk("(used)");
	}

	printk("\n");
}



struct mite_struct * a4l_mite_find_device(int bus, 
					  int slot, unsigned short device_id)
{
	struct list_head *this;

	list_for_each(this, &mite_devices) {
		struct mite_struct *mite =
			list_entry(this, struct mite_struct, list);

		if(mite->pcidev->device != device_id)
			continue;

		if((bus <= 0 && slot <= 0) ||
		   (bus == mite->pcidev->bus->number &&
		    slot == PCI_SLOT(mite->pcidev->devfn)))
			return mite;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(a4l_mite_find_device);

struct mite_channel *
a4l_mite_request_channel_in_range(struct mite_struct *mite,
				  struct mite_dma_descriptor_ring *ring,
				  unsigned min_channel, unsigned max_channel)
{
	int i;
	unsigned long flags;
	struct mite_channel *channel = NULL;

	__a4l_dbg(1, drv_dbg, " min_channel = %u, max_channel = %u\n",
		  min_channel, max_channel);

	/* spin lock so a4l_mite_release_channel can be called safely
	   from interrupts */
	rtdm_lock_get_irqsave(&mite->lock, flags);
	for (i = min_channel; i <= max_channel; ++i) {

	__a4l_dbg(1, drv_dbg, " channel[%d] allocated = %d\n",
		  i, mite->channel_allocated[i]);

		if (mite->channel_allocated[i] == 0) {
			mite->channel_allocated[i] = 1;
			channel = &mite->channels[i];
			channel->ring = ring;
			break;
		}
	}
	rtdm_lock_put_irqrestore(&mite->lock, flags);
	return channel;
}

void a4l_mite_release_channel(struct mite_channel *mite_chan)
{
	struct mite_struct *mite = mite_chan->mite;
	unsigned long flags;

	/* Spin lock to prevent races with mite_request_channel */
	rtdm_lock_get_irqsave(&mite->lock, flags);
	if (mite->channel_allocated[mite_chan->channel]) {
		/* disable all channel's interrupts */
		writel(CHCR_CLR_DMA_IE | CHCR_CLR_LINKP_IE |
		       CHCR_CLR_SAR_IE | CHCR_CLR_DONE_IE |
		       CHCR_CLR_MRDY_IE | CHCR_CLR_DRDY_IE |
		       CHCR_CLR_LC_IE | CHCR_CLR_CONT_RB_IE,
		       mite->mite_io_addr + MITE_CHCR(mite_chan->channel));
		a4l_mite_dma_disarm(mite_chan);
		mite_dma_reset(mite_chan);
		mite->channel_allocated[mite_chan->channel] = 0;
		mite_chan->ring = NULL;
		mmiowb();
	}
	rtdm_lock_put_irqrestore(&mite->lock, flags);
}

void a4l_mite_dma_arm(struct mite_channel *mite_chan)
{
	struct mite_struct *mite = mite_chan->mite;
	int chor;
	unsigned long flags;

	MDPRINTK("a4l_mite_dma_arm ch%i\n", mite_chan->channel);
	/* Memory barrier is intended to insure any twiddling with the buffer
	   is done before writing to the mite to arm dma transfer */
	smp_mb();
	/* arm */
	chor = CHOR_START;
	rtdm_lock_get_irqsave(&mite->lock, flags);
	mite_chan->done = 0;
	writel(chor, mite->mite_io_addr + MITE_CHOR(mite_chan->channel));
	mmiowb();
	rtdm_lock_put_irqrestore(&mite->lock, flags);
}

void a4l_mite_dma_disarm(struct mite_channel *mite_chan)
{
	struct mite_struct *mite = mite_chan->mite;
	unsigned chor;

	/* disarm */
	chor = CHOR_ABORT;
	writel(chor, mite->mite_io_addr + MITE_CHOR(mite_chan->channel));
}

int a4l_mite_buf_change(struct mite_dma_descriptor_ring *ring, struct a4l_subdevice *subd)
{
	struct a4l_buffer *buf = subd->buf;
	unsigned int n_links;
	int i;

	if (ring->descriptors) {
		pci_free_consistent(ring->pcidev,
				    ring->n_links * sizeof(struct mite_dma_descriptor),
				    ring->descriptors, ring->descriptors_dma_addr);
	}
	ring->descriptors = NULL;
	ring->descriptors_dma_addr = 0;
	ring->n_links = 0;

	if (buf->size == 0) {
		return 0;
	}
	n_links = buf->size >> PAGE_SHIFT;

	MDPRINTK("ring->pcidev=%p, n_links=0x%04x\n", ring->pcidev, n_links);

	ring->descriptors =
		pci_alloc_consistent(ring->pcidev,
				     n_links * sizeof(struct mite_dma_descriptor),
				     &ring->descriptors_dma_addr);
	if (!ring->descriptors) {
		printk("MITE: ring buffer allocation failed\n");
		return -ENOMEM;
	}
	ring->n_links = n_links;

	for (i = 0; i < n_links; i++) {
		ring->descriptors[i].count = cpu_to_le32(PAGE_SIZE);
		ring->descriptors[i].addr = cpu_to_le32(buf->pg_list[i]);
		ring->descriptors[i].next =
			cpu_to_le32(ring->descriptors_dma_addr +
				    (i + 1) * sizeof(struct mite_dma_descriptor));
	}

	ring->descriptors[n_links - 1].next =
		cpu_to_le32(ring->descriptors_dma_addr);

	/* Barrier is meant to insure that all the writes to the dma descriptors
	   have completed before the dma controller is commanded to read them */
	smp_wmb();

	return 0;
}

void a4l_mite_prep_dma(struct mite_channel *mite_chan,
		   unsigned int num_device_bits, unsigned int num_memory_bits)
{
	unsigned int chor, chcr, mcr, dcr, lkcr;
	struct mite_struct *mite = mite_chan->mite;

	MDPRINTK("a4l_mite_prep_dma ch%i\n", mite_chan->channel);

	/* reset DMA and FIFO */
	chor = CHOR_DMARESET | CHOR_FRESET;
	writel(chor, mite->mite_io_addr + MITE_CHOR(mite_chan->channel));

	/* short link chaining mode */
	chcr = CHCR_SET_DMA_IE | CHCR_LINKSHORT | CHCR_SET_DONE_IE |
		CHCR_BURSTEN;
	/*
	 * Link Complete Interrupt: interrupt every time a link
	 * in MITE_RING is completed. This can generate a lot of
	 * extra interrupts, but right now we update the values
	 * of buf_int_ptr and buf_int_count at each interrupt.  A
	 * better method is to poll the MITE before each user
	 * "read()" to calculate the number of bytes available.
	 */
	chcr |= CHCR_SET_LC_IE;
	if (num_memory_bits == 32 && num_device_bits == 16) {
		/* Doing a combined 32 and 16 bit byteswap gets the 16
		   bit samples into the fifo in the right order.
		   Tested doing 32 bit memory to 16 bit device
		   transfers to the analog out of a pxi-6281, which
		   has mite version = 1, type = 4.  This also works
		   for dma reads from the counters on e-series boards.
		*/
		chcr |= CHCR_BYTE_SWAP_DEVICE | CHCR_BYTE_SWAP_MEMORY;
	}

	if (mite_chan->dir == A4L_INPUT) {
		chcr |= CHCR_DEV_TO_MEM;
	}
	writel(chcr, mite->mite_io_addr + MITE_CHCR(mite_chan->channel));

	/* to/from memory */
	mcr = CR_RL(64) | CR_ASEQUP;
	switch (num_memory_bits) {
	case 8:
		mcr |= CR_PSIZE8;
		break;
	case 16:
		mcr |= CR_PSIZE16;
		break;
	case 32:
		mcr |= CR_PSIZE32;
		break;
	default:
		__a4l_err("MITE: bug! "
			  "invalid mem bit width for dma transfer\n");
		break;
	}
	writel(mcr, mite->mite_io_addr + MITE_MCR(mite_chan->channel));

	/* from/to device */
	dcr = CR_RL(64) | CR_ASEQUP;
	dcr |= CR_PORTIO | CR_AMDEVICE | CR_REQSDRQ(mite_chan->channel);
	switch (num_device_bits) {
	case 8:
		dcr |= CR_PSIZE8;
		break;
	case 16:
		dcr |= CR_PSIZE16;
		break;
	case 32:
		dcr |= CR_PSIZE32;
		break;
	default:
		__a4l_info("MITE: bug! "
			   "invalid dev bit width for dma transfer\n");
		break;
	}
	writel(dcr, mite->mite_io_addr + MITE_DCR(mite_chan->channel));

	/* reset the DAR */
	writel(0, mite->mite_io_addr + MITE_DAR(mite_chan->channel));

	/* the link is 32bits */
	lkcr = CR_RL(64) | CR_ASEQUP | CR_PSIZE32;
	writel(lkcr, mite->mite_io_addr + MITE_LKCR(mite_chan->channel));

	/* starting address for link chaining */
	writel(mite_chan->ring->descriptors_dma_addr,
	       mite->mite_io_addr + MITE_LKAR(mite_chan->channel));

	MDPRINTK("exit a4l_mite_prep_dma\n");
}

u32 mite_device_bytes_transferred(struct mite_channel *mite_chan)
{
	struct mite_struct *mite = mite_chan->mite;
	return readl(mite->mite_io_addr + MITE_DAR(mite_chan->channel));
}

u32 a4l_mite_bytes_in_transit(struct mite_channel * mite_chan)
{
	struct mite_struct *mite = mite_chan->mite;
	return readl(mite->mite_io_addr +
		     MITE_FCR(mite_chan->channel)) & 0x000000FF;
}

/* Returns lower bound for number of bytes transferred from device to memory */
u32 a4l_mite_bytes_written_to_memory_lb(struct mite_channel * mite_chan)
{
	u32 device_byte_count;

	device_byte_count = mite_device_bytes_transferred(mite_chan);
	return device_byte_count - a4l_mite_bytes_in_transit(mite_chan);
}

/* Returns upper bound for number of bytes transferred from device to memory */
u32 a4l_mite_bytes_written_to_memory_ub(struct mite_channel * mite_chan)
{
	u32 in_transit_count;

	in_transit_count = a4l_mite_bytes_in_transit(mite_chan);
	return mite_device_bytes_transferred(mite_chan) - in_transit_count;
}

/* Returns lower bound for number of bytes read from memory for transfer to device */
u32 a4l_mite_bytes_read_from_memory_lb(struct mite_channel * mite_chan)
{
	u32 device_byte_count;

	device_byte_count = mite_device_bytes_transferred(mite_chan);
	return device_byte_count + a4l_mite_bytes_in_transit(mite_chan);
}

/* Returns upper bound for number of bytes read from memory for transfer to device */
u32 a4l_mite_bytes_read_from_memory_ub(struct mite_channel * mite_chan)
{
	u32 in_transit_count;

	in_transit_count = a4l_mite_bytes_in_transit(mite_chan);
	return mite_device_bytes_transferred(mite_chan) + in_transit_count;
}

int a4l_mite_sync_input_dma(struct mite_channel *mite_chan, struct a4l_subdevice *subd)
{
	unsigned int nbytes_lb, nbytes_ub;

	nbytes_lb = a4l_mite_bytes_written_to_memory_lb(mite_chan);
	nbytes_ub = a4l_mite_bytes_written_to_memory_ub(mite_chan);

	if(a4l_buf_prepare_absput(subd, nbytes_ub) != 0) {
		__a4l_err("MITE: DMA overwrite of free area\n");
		return -EPIPE;
	}

	return a4l_buf_commit_absput(subd, nbytes_lb);
}

int a4l_mite_sync_output_dma(struct mite_channel *mite_chan, struct a4l_subdevice *subd)
{
	struct a4l_buffer *buf = subd->buf;
	unsigned int nbytes_ub, nbytes_lb;
	int err;

	nbytes_lb = a4l_mite_bytes_read_from_memory_lb(mite_chan);
	nbytes_ub = a4l_mite_bytes_read_from_memory_ub(mite_chan);

	err = a4l_buf_prepare_absget(subd, nbytes_ub);
	if(err < 0) {
		__a4l_info("MITE: DMA underrun\n");
		return -EPIPE;
	}

	err = a4l_buf_commit_absget(subd, nbytes_lb);

	/* If the MITE has already transfered more than required, we
	   can disable it */
	if (test_bit(A4L_BUF_EOA_NR, &buf->flags))
		writel(CHOR_STOP,
		       mite_chan->mite->mite_io_addr +
		       MITE_CHOR(mite_chan->channel));

	return err;
}

u32 a4l_mite_get_status(struct mite_channel *mite_chan)
{
	struct mite_struct *mite = mite_chan->mite;
	u32 status;
	unsigned long flags;

	rtdm_lock_get_irqsave(&mite->lock, flags);
	status = readl(mite->mite_io_addr + MITE_CHSR(mite_chan->channel));
	if (status & CHSR_DONE) {
		mite_chan->done = 1;
		writel(CHOR_CLRDONE,
		       mite->mite_io_addr + MITE_CHOR(mite_chan->channel));
	}
	mmiowb();
	rtdm_lock_put_irqrestore(&mite->lock, flags);
	return status;
}

int a4l_mite_done(struct mite_channel *mite_chan)
{
	struct mite_struct *mite = mite_chan->mite;
	unsigned long flags;
	int done;

	a4l_mite_get_status(mite_chan);
	rtdm_lock_get_irqsave(&mite->lock, flags);
	done = mite_chan->done;
	rtdm_lock_put_irqrestore(&mite->lock, flags);
	return done;
}

#ifdef CONFIG_DEBUG_MITE

static void a4l_mite_decode(const char *const bit_str[], unsigned int bits);

/* names of bits in mite registers */

static const char *const mite_CHOR_strings[] = {
	"start", "cont", "stop", "abort",
	"freset", "clrlc", "clrrb", "clrdone",
	"clr_lpause", "set_lpause", "clr_send_tc",
	"set_send_tc", "12", "13", "14",
	"15", "16", "17", "18",
	"19", "20", "21", "22",
	"23", "24", "25", "26",
	"27", "28", "29", "30",
	"dmareset",
};

static const char *const mite_CHCR_strings[] = {
	"continue", "ringbuff", "2", "3",
	"4", "5", "6", "7",
	"8", "9", "10", "11",
	"12", "13", "bursten", "fifodis",
	"clr_cont_rb_ie", "set_cont_rb_ie", "clr_lc_ie", "set_lc_ie",
	"clr_drdy_ie", "set_drdy_ie", "clr_mrdy_ie", "set_mrdy_ie",
	"clr_done_ie", "set_done_ie", "clr_sar_ie", "set_sar_ie",
	"clr_linkp_ie", "set_linkp_ie", "clr_dma_ie", "set_dma_ie",
};

static const char *const mite_MCR_strings[] = {
	"amdevice", "1", "2", "3",
	"4", "5", "portio", "portvxi",
	"psizebyte", "psizehalf (byte & half = word)", "aseqxp1", "11",
	"12", "13", "blocken", "berhand",
	"reqsintlim/reqs0", "reqs1", "reqs2", "rd32",
	"rd512", "rl1", "rl2", "rl8",
	"24", "25", "26", "27",
	"28", "29", "30", "stopen",
};

static const char *const mite_DCR_strings[] = {
	"amdevice", "1", "2", "3",
	"4", "5", "portio", "portvxi",
	"psizebyte", "psizehalf (byte & half = word)", "aseqxp1", "aseqxp2",
	"aseqxp8", "13", "blocken", "berhand",
	"reqsintlim", "reqs1", "reqs2", "rd32",
	"rd512", "rl1", "rl2", "rl8",
	"23", "24", "25", "27",
	"28", "wsdevc", "wsdevs", "rwdevpack",
};

static const char *const mite_LKCR_strings[] = {
	"amdevice", "1", "2", "3",
	"4", "5", "portio", "portvxi",
	"psizebyte", "psizehalf (byte & half = word)", "asequp", "aseqdown",
	"12", "13", "14", "berhand",
	"16", "17", "18", "rd32",
	"rd512", "rl1", "rl2", "rl8",
	"24", "25", "26", "27",
	"28", "29", "30", "chngend",
};

static const char *const mite_CHSR_strings[] = {
	"d.err0", "d.err1", "m.err0", "m.err1",
	"l.err0", "l.err1", "drq0", "drq1",
	"end", "xferr", "operr0", "operr1",
	"stops", "habort", "sabort", "error",
	"16", "conts_rb", "18", "linkc",
	"20", "drdy", "22", "mrdy",
	"24", "done", "26", "sars",
	"28", "lpauses", "30", "int",
};

void a4l_mite_dump_regs(struct mite_channel *mite_chan)
{
	unsigned long mite_io_addr =
		(unsigned long)mite_chan->mite->mite_io_addr;
	unsigned long addr = 0;
	unsigned long temp = 0;

	printk("a4l_mite_dump_regs ch%i\n", mite_chan->channel);
	printk("mite address is  =0x%08lx\n", mite_io_addr);

	addr = mite_io_addr + MITE_CHOR(mite_chan->channel);
	printk("mite status[CHOR]at 0x%08lx =0x%08lx\n", addr, temp =
	       readl((void *)addr));
	a4l_mite_decode(mite_CHOR_strings, temp);
	addr = mite_io_addr + MITE_CHCR(mite_chan->channel);
	printk("mite status[CHCR]at 0x%08lx =0x%08lx\n", addr, temp =
	       readl((void *)addr));
	a4l_mite_decode(mite_CHCR_strings, temp);
	addr = mite_io_addr + MITE_TCR(mite_chan->channel);
	printk("mite status[TCR] at 0x%08lx =0x%08x\n", addr,
	       readl((void *)addr));
	addr = mite_io_addr + MITE_MCR(mite_chan->channel);
	printk("mite status[MCR] at 0x%08lx =0x%08lx\n", addr, temp =
	       readl((void *)addr));
	a4l_mite_decode(mite_MCR_strings, temp);

	addr = mite_io_addr + MITE_MAR(mite_chan->channel);
	printk("mite status[MAR] at 0x%08lx =0x%08x\n", addr,
	       readl((void *)addr));
	addr = mite_io_addr + MITE_DCR(mite_chan->channel);
	printk("mite status[DCR] at 0x%08lx =0x%08lx\n", addr, temp =
	       readl((void *)addr));
	a4l_mite_decode(mite_DCR_strings, temp);
	addr = mite_io_addr + MITE_DAR(mite_chan->channel);
	printk("mite status[DAR] at 0x%08lx =0x%08x\n", addr,
	       readl((void *)addr));
	addr = mite_io_addr + MITE_LKCR(mite_chan->channel);
	printk("mite status[LKCR]at 0x%08lx =0x%08lx\n", addr, temp =
	       readl((void *)addr));
	a4l_mite_decode(mite_LKCR_strings, temp);
	addr = mite_io_addr + MITE_LKAR(mite_chan->channel);
	printk("mite status[LKAR]at 0x%08lx =0x%08x\n", addr,
	       readl((void *)addr));

	addr = mite_io_addr + MITE_CHSR(mite_chan->channel);
	printk("mite status[CHSR]at 0x%08lx =0x%08lx\n", addr, temp =
	       readl((void *)addr));
	a4l_mite_decode(mite_CHSR_strings, temp);
	addr = mite_io_addr + MITE_FCR(mite_chan->channel);
	printk("mite status[FCR] at 0x%08lx =0x%08x\n\n", addr,
	       readl((void *)addr));
}


static void a4l_mite_decode(const char *const bit_str[], unsigned int bits)
{
	int i;

	for (i = 31; i >= 0; i--) {
		if (bits & (1 << i)) {
			printk(" %s", bit_str[i]);
		}
	}
	printk("\n");
}

#endif /* CONFIG_DEBUG_MITE */


static int __init mite_init(void)
{
	int err;

	/* Register the mite's PCI driver */
	err = pci_register_driver(&mite_driver);

	if(err == 0)
		a4l_mite_list_devices();

	return err;
}

static void __exit mite_cleanup(void)
{

	/* Unregister the PCI structure driver */
	pci_unregister_driver(&mite_driver);

	/* Just paranoia... */
	while(&mite_devices != mite_devices.next) {
		struct list_head *this = mite_devices.next;
		struct mite_struct *mite =
			list_entry(this, struct mite_struct, list);

		list_del(this);
		kfree(mite);
	}
}

MODULE_LICENSE("GPL");
module_init(mite_init);
module_exit(mite_cleanup);

EXPORT_SYMBOL_GPL(a4l_mite_dma_arm);
EXPORT_SYMBOL_GPL(a4l_mite_dma_disarm);
EXPORT_SYMBOL_GPL(a4l_mite_sync_input_dma);
EXPORT_SYMBOL_GPL(a4l_mite_sync_output_dma);
EXPORT_SYMBOL_GPL(a4l_mite_setup);
EXPORT_SYMBOL_GPL(a4l_mite_unsetup);
EXPORT_SYMBOL_GPL(a4l_mite_list_devices);
EXPORT_SYMBOL_GPL(a4l_mite_request_channel_in_range);
EXPORT_SYMBOL_GPL(a4l_mite_release_channel);
EXPORT_SYMBOL_GPL(a4l_mite_prep_dma);
EXPORT_SYMBOL_GPL(a4l_mite_buf_change);
EXPORT_SYMBOL_GPL(a4l_mite_bytes_written_to_memory_lb);
EXPORT_SYMBOL_GPL(a4l_mite_bytes_written_to_memory_ub);
EXPORT_SYMBOL_GPL(a4l_mite_bytes_read_from_memory_lb);
EXPORT_SYMBOL_GPL(a4l_mite_bytes_read_from_memory_ub);
EXPORT_SYMBOL_GPL(a4l_mite_bytes_in_transit);
EXPORT_SYMBOL_GPL(a4l_mite_get_status);
EXPORT_SYMBOL_GPL(a4l_mite_done);
#ifdef CONFIG_DEBUG_MITE
EXPORT_SYMBOL_GPL(a4l_mite_decode);
EXPORT_SYMBOL_GPL(a4l_mite_dump_regs);
#endif /* CONFIG_DEBUG_MITE */
