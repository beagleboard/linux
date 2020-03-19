/*
 * Hardware driver for NI Mite PCI interface chip
 * @note Copyright (C) 1999 David A. Schleef <ds@schleef.org>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 */
#ifndef __ANALOGY_NI_MITE_H__
#define __ANALOGY_NI_MITE_H__

#include <linux/pci.h>
#include <linux/slab.h>
#include <rtdm/analogy/device.h>

#define PCI_VENDOR_ID_NATINST 0x1093
#define PCI_MITE_SIZE 4096
#define PCI_DAQ_SIZE 4096
#define PCI_DAQ_SIZE_660X 8192
#define PCIMIO_COMPAT
#define MAX_MITE_DMA_CHANNELS 8

#define TOP_OF_PAGE(x) ((x)|(~(PAGE_MASK)))

struct mite_dma_descriptor {
	u32 count;
	u32 addr;
	u32 next;
	u32 dar;
};

struct mite_dma_descriptor_ring {
	struct pci_dev *pcidev;
	u32 n_links;
	struct mite_dma_descriptor *descriptors;
	dma_addr_t descriptors_dma_addr;
};

struct mite_channel {
	struct mite_struct *mite;
	u32 channel;
	u32 dir;
	u32 done;
	struct mite_dma_descriptor_ring *ring;
};

struct mite_struct {
	struct list_head list;
	rtdm_lock_t lock;
	u32 used;
	u32 num_channels;

	struct mite_channel channels[MAX_MITE_DMA_CHANNELS];
	u32 channel_allocated[MAX_MITE_DMA_CHANNELS];

	struct pci_dev *pcidev;
	resource_size_t mite_phys_addr;
	void *mite_io_addr;
	resource_size_t daq_phys_addr;
	void *daq_io_addr;
};

static inline
struct mite_dma_descriptor_ring *mite_alloc_ring(struct	mite_struct *mite)
{
	struct mite_dma_descriptor_ring *ring =
		kmalloc(sizeof(struct mite_dma_descriptor_ring), GFP_DMA);

	if (ring == NULL)
		return ring;

	memset(ring, 0, sizeof(struct mite_dma_descriptor_ring));

	ring->pcidev = mite->pcidev;
	if (ring->pcidev == NULL) {
		kfree(ring);
		return NULL;
	}

	return ring;
};

static inline void mite_free_ring(struct mite_dma_descriptor_ring *ring)
{
	if (ring) {
		if (ring->descriptors) {
			pci_free_consistent(
				ring->pcidev,
				ring->n_links *
				sizeof(struct mite_dma_descriptor),
				ring->descriptors, ring->descriptors_dma_addr);
		}
		kfree(ring);
	}
};

static inline unsigned int mite_irq(struct mite_struct *mite)
{
	return mite->pcidev->irq;
};
static inline unsigned int mite_device_id(struct mite_struct *mite)
{
	return mite->pcidev->device;
};

int a4l_mite_setup(struct mite_struct *mite, int use_iodwbsr_1);
void a4l_mite_unsetup(struct mite_struct *mite);
void a4l_mite_list_devices(void);
struct mite_struct * a4l_mite_find_device(int bus,
					  int slot, unsigned short device_id);
struct mite_channel *
a4l_mite_request_channel_in_range(struct mite_struct *mite,
				  struct mite_dma_descriptor_ring *ring,
				  unsigned min_channel, unsigned max_channel);
static inline struct mite_channel *mite_request_channel(struct mite_struct
	*mite, struct mite_dma_descriptor_ring *ring)
{
	return a4l_mite_request_channel_in_range(mite, ring, 0,
		mite->num_channels - 1);
}
void a4l_mite_release_channel(struct mite_channel *mite_chan);

void a4l_mite_dma_arm(struct mite_channel *mite_chan);
void a4l_mite_dma_disarm(struct mite_channel *mite_chan);
int a4l_mite_sync_input_dma(struct mite_channel *mite_chan, struct a4l_subdevice *subd);
int a4l_mite_sync_output_dma(struct mite_channel *mite_chan, struct a4l_subdevice *subd);
u32 a4l_mite_bytes_written_to_memory_lb(struct mite_channel *mite_chan);
u32 a4l_mite_bytes_written_to_memory_ub(struct mite_channel *mite_chan);
u32 a4l_mite_bytes_read_from_memory_lb(struct mite_channel *mite_chan);
u32 a4l_mite_bytes_read_from_memory_ub(struct mite_channel *mite_chan);
u32 a4l_mite_bytes_in_transit(struct mite_channel *mite_chan);
u32 a4l_mite_get_status(struct mite_channel *mite_chan);
int a4l_mite_done(struct mite_channel *mite_chan);
void a4l_mite_prep_dma(struct mite_channel *mite_chan,
		   unsigned int num_device_bits, unsigned int num_memory_bits);
int a4l_mite_buf_change(struct mite_dma_descriptor_ring *ring, struct a4l_subdevice *subd);

#ifdef CONFIG_DEBUG_MITE
void mite_print_chsr(unsigned int chsr);
void a4l_mite_dump_regs(struct mite_channel *mite_chan);
#endif

static inline int CHAN_OFFSET(int channel)
{
	return 0x500 + 0x100 * channel;
};

enum mite_registers {
	/* The bits 0x90180700 in MITE_UNKNOWN_DMA_BURST_REG can be
	   written and read back.  The bits 0x1f always read as 1.
	   The rest always read as zero. */
	MITE_UNKNOWN_DMA_BURST_REG = 0x28,
	MITE_IODWBSR = 0xc0,	//IO Device Window Base Size Register
	MITE_IODWBSR_1 = 0xc4,	// IO Device Window Base Size Register 1
	MITE_IODWCR_1 = 0xf4,
	MITE_PCI_CONFIG_OFFSET = 0x300,
	MITE_CSIGR = 0x460	//chip signature
};
static inline int MITE_CHOR(int channel)	// channel operation
{
	return CHAN_OFFSET(channel) + 0x0;
};
static inline int MITE_CHCR(int channel)	// channel control
{
	return CHAN_OFFSET(channel) + 0x4;
};
static inline int MITE_TCR(int channel)	// transfer count
{
	return CHAN_OFFSET(channel) + 0x8;
};
static inline int MITE_MCR(int channel)	// memory configuration
{
	return CHAN_OFFSET(channel) + 0xc;
};
static inline int MITE_MAR(int channel)	// memory address
{
	return CHAN_OFFSET(channel) + 0x10;
};
static inline int MITE_DCR(int channel)	// device configuration
{
	return CHAN_OFFSET(channel) + 0x14;
};
static inline int MITE_DAR(int channel)	// device address
{
	return CHAN_OFFSET(channel) + 0x18;
};
static inline int MITE_LKCR(int channel)	// link configuration
{
	return CHAN_OFFSET(channel) + 0x1c;
};
static inline int MITE_LKAR(int channel)	// link address
{
	return CHAN_OFFSET(channel) + 0x20;
};
static inline int MITE_LLKAR(int channel)	// see mite section of tnt5002 manual
{
	return CHAN_OFFSET(channel) + 0x24;
};
static inline int MITE_BAR(int channel)	// base address
{
	return CHAN_OFFSET(channel) + 0x28;
};
static inline int MITE_BCR(int channel)	// base count
{
	return CHAN_OFFSET(channel) + 0x2c;
};
static inline int MITE_SAR(int channel)	// ? address
{
	return CHAN_OFFSET(channel) + 0x30;
};
static inline int MITE_WSCR(int channel)	// ?
{
	return CHAN_OFFSET(channel) + 0x34;
};
static inline int MITE_WSER(int channel)	// ?
{
	return CHAN_OFFSET(channel) + 0x38;
};
static inline int MITE_CHSR(int channel)	// channel status
{
	return CHAN_OFFSET(channel) + 0x3c;
};
static inline int MITE_FCR(int channel)	// fifo count
{
	return CHAN_OFFSET(channel) + 0x40;
};

enum MITE_IODWBSR_bits {
	WENAB = 0x80,		// window enable
};

static inline unsigned MITE_IODWBSR_1_WSIZE_bits(unsigned size)
{
	unsigned order = 0;
	while (size >>= 1)
		++order;
	BUG_ON(order < 1);
	return (order - 1) & 0x1f;
}

enum MITE_UNKNOWN_DMA_BURST_bits {
	UNKNOWN_DMA_BURST_ENABLE_BITS = 0x600
};

static inline int mite_csigr_version(u32 csigr_bits)
{
	return csigr_bits & 0xf;
};
static inline int mite_csigr_type(u32 csigr_bits)
{				// original mite = 0, minimite = 1
	return (csigr_bits >> 4) & 0xf;
};
static inline int mite_csigr_mmode(u32 csigr_bits)
{				// mite mode, minimite = 1
	return (csigr_bits >> 8) & 0x3;
};
static inline int mite_csigr_imode(u32 csigr_bits)
{				// cpu port interface mode, pci = 0x3
	return (csigr_bits >> 12) & 0x3;
};
static inline int mite_csigr_dmac(u32 csigr_bits)
{				// number of dma channels
	return (csigr_bits >> 16) & 0xf;
};
static inline int mite_csigr_wpdep(u32 csigr_bits)
{				// write post fifo depth
	unsigned int wpdep_bits = (csigr_bits >> 20) & 0x7;
	if (wpdep_bits == 0)
		return 0;
	else
		return 1 << (wpdep_bits - 1);
};
static inline int mite_csigr_wins(u32 csigr_bits)
{
	return (csigr_bits >> 24) & 0x1f;
};
static inline int mite_csigr_iowins(u32 csigr_bits)
{				// number of io windows
	return (csigr_bits >> 29) & 0x7;
};

enum MITE_MCR_bits {
	MCRPON = 0,
};

enum MITE_DCR_bits {
	DCR_NORMAL = (1 << 29),
	DCRPON = 0,
};

enum MITE_CHOR_bits {
	CHOR_DMARESET = (1 << 31),
	CHOR_SET_SEND_TC = (1 << 11),
	CHOR_CLR_SEND_TC = (1 << 10),
	CHOR_SET_LPAUSE = (1 << 9),
	CHOR_CLR_LPAUSE = (1 << 8),
	CHOR_CLRDONE = (1 << 7),
	CHOR_CLRRB = (1 << 6),
	CHOR_CLRLC = (1 << 5),
	CHOR_FRESET = (1 << 4),
	CHOR_ABORT = (1 << 3),	/* stop without emptying fifo */
	CHOR_STOP = (1 << 2),	/* stop after emptying fifo */
	CHOR_CONT = (1 << 1),
	CHOR_START = (1 << 0),
	CHOR_PON = (CHOR_CLR_SEND_TC | CHOR_CLR_LPAUSE),
};

enum MITE_CHCR_bits {
	CHCR_SET_DMA_IE = (1 << 31),
	CHCR_CLR_DMA_IE = (1 << 30),
	CHCR_SET_LINKP_IE = (1 << 29),
	CHCR_CLR_LINKP_IE = (1 << 28),
	CHCR_SET_SAR_IE = (1 << 27),
	CHCR_CLR_SAR_IE = (1 << 26),
	CHCR_SET_DONE_IE = (1 << 25),
	CHCR_CLR_DONE_IE = (1 << 24),
	CHCR_SET_MRDY_IE = (1 << 23),
	CHCR_CLR_MRDY_IE = (1 << 22),
	CHCR_SET_DRDY_IE = (1 << 21),
	CHCR_CLR_DRDY_IE = (1 << 20),
	CHCR_SET_LC_IE = (1 << 19),
	CHCR_CLR_LC_IE = (1 << 18),
	CHCR_SET_CONT_RB_IE = (1 << 17),
	CHCR_CLR_CONT_RB_IE = (1 << 16),
	CHCR_FIFODIS = (1 << 15),
	CHCR_FIFO_ON = 0,
	CHCR_BURSTEN = (1 << 14),
	CHCR_NO_BURSTEN = 0,
	CHCR_BYTE_SWAP_DEVICE = (1 << 6),
	CHCR_BYTE_SWAP_MEMORY = (1 << 4),
	CHCR_DIR = (1 << 3),
	CHCR_DEV_TO_MEM = CHCR_DIR,
	CHCR_MEM_TO_DEV = 0,
	CHCR_NORMAL = (0 << 0),
	CHCR_CONTINUE = (1 << 0),
	CHCR_RINGBUFF = (2 << 0),
	CHCR_LINKSHORT = (4 << 0),
	CHCR_LINKLONG = (5 << 0),
	CHCRPON =
		(CHCR_CLR_DMA_IE | CHCR_CLR_LINKP_IE | CHCR_CLR_SAR_IE |
		CHCR_CLR_DONE_IE | CHCR_CLR_MRDY_IE | CHCR_CLR_DRDY_IE |
		CHCR_CLR_LC_IE | CHCR_CLR_CONT_RB_IE),
};

enum ConfigRegister_bits {
	CR_REQS_MASK = 0x7 << 16,
	CR_ASEQDONT = 0x0 << 10,
	CR_ASEQUP = 0x1 << 10,
	CR_ASEQDOWN = 0x2 << 10,
	CR_ASEQ_MASK = 0x3 << 10,
	CR_PSIZE8 = (1 << 8),
	CR_PSIZE16 = (2 << 8),
	CR_PSIZE32 = (3 << 8),
	CR_PORTCPU = (0 << 6),
	CR_PORTIO = (1 << 6),
	CR_PORTVXI = (2 << 6),
	CR_PORTMXI = (3 << 6),
	CR_AMDEVICE = (1 << 0),
};
static inline int CR_REQS(int source)
{
	return (source & 0x7) << 16;
};
static inline int CR_REQSDRQ(unsigned drq_line)
{
	/* This also works on m-series when
	   using channels (drq_line) 4 or 5. */
	return CR_REQS((drq_line & 0x3) | 0x4);
}
static inline int CR_RL(unsigned int retry_limit)
{
	int value = 0;

	while (retry_limit) {
		retry_limit >>= 1;
		value++;
	}
	if (value > 0x7)
		__a4l_err("bug! retry_limit too large\n");

	return (value & 0x7) << 21;
}

enum CHSR_bits {
	CHSR_INT = (1 << 31),
	CHSR_LPAUSES = (1 << 29),
	CHSR_SARS = (1 << 27),
	CHSR_DONE = (1 << 25),
	CHSR_MRDY = (1 << 23),
	CHSR_DRDY = (1 << 21),
	CHSR_LINKC = (1 << 19),
	CHSR_CONTS_RB = (1 << 17),
	CHSR_ERROR = (1 << 15),
	CHSR_SABORT = (1 << 14),
	CHSR_HABORT = (1 << 13),
	CHSR_STOPS = (1 << 12),
	CHSR_OPERR_mask = (3 << 10),
	CHSR_OPERR_NOERROR = (0 << 10),
	CHSR_OPERR_FIFOERROR = (1 << 10),
	CHSR_OPERR_LINKERROR = (1 << 10),	/* ??? */
	CHSR_XFERR = (1 << 9),
	CHSR_END = (1 << 8),
	CHSR_DRQ1 = (1 << 7),
	CHSR_DRQ0 = (1 << 6),
	CHSR_LxERR_mask = (3 << 4),
	CHSR_LBERR = (1 << 4),
	CHSR_LRERR = (2 << 4),
	CHSR_LOERR = (3 << 4),
	CHSR_MxERR_mask = (3 << 2),
	CHSR_MBERR = (1 << 2),
	CHSR_MRERR = (2 << 2),
	CHSR_MOERR = (3 << 2),
	CHSR_DxERR_mask = (3 << 0),
	CHSR_DBERR = (1 << 0),
	CHSR_DRERR = (2 << 0),
	CHSR_DOERR = (3 << 0),
};

static inline void mite_dma_reset(struct mite_channel *mite_chan)
{
	writel(CHOR_DMARESET | CHOR_FRESET,
		mite_chan->mite->mite_io_addr + MITE_CHOR(mite_chan->channel));
};

#endif /* !__ANALOGY_NI_MITE_H__ */
