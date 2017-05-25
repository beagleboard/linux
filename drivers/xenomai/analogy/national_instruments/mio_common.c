/*
 * Hardware driver for DAQ-STC based boards
 *
 * Copyright (C) 1997-2001 David A. Schleef <ds@schleef.org>
 * Copyright (C) 2002-2006 Frank Mori Hess <fmhess@users.sourceforge.net>
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
 * Description: DAQ-STC systems
 *
 * References:
 * 340747b.pdf  AT-MIO E series Register-Level Programmer Manual
 * 341079b.pdf  PCI E Series Register-Level Programmer Manual
 * 340934b.pdf  DAQ-STC reference manual
 * 322080b.pdf  6711/6713/6715 User Manual
 * 320945c.pdf  PCI E Series User Manual
 * 322138a.pdf  PCI-6052E and DAQPad-6052E User Manual
 * 320517c.pdf  AT E Series User manual (obsolete)
 * 320517f.pdf  AT E Series User manual
 * 320906c.pdf  Maximum signal ratings
 * 321066a.pdf  About 16x
 * 321791a.pdf  Discontinuation of at-mio-16e-10 rev. c
 * 321808a.pdf  About at-mio-16e-10 rev P
 * 321837a.pdf  Discontinuation of at-mio-16de-10 rev d
 * 321838a.pdf  About at-mio-16de-10 rev N
 *
 * ISSUES:
 * - The interrupt routine needs to be cleaned up
 * - S-Series PCI-6143 support has been added but is not fully tested
 *   as yet. Terry Barnaby, BEAM Ltd.
 *
 */
#include <linux/module.h>
#include <linux/slab.h>
#include "../intel/8255.h"
#include "mite.h"
#include "ni_stc.h"
#include "ni_mio.h"

#define NI_TIMEOUT 1000

/* Note: this table must match the ai_gain_* definitions */
static const short ni_gainlkup[][16] = {
	/* ai_gain_16 */
	{0, 1, 2, 3, 4, 5, 6, 7, 0x100, 0x101, 0x102, 0x103, 0x104, 0x105,
	 0x106, 0x107},
	/* ai_gain_8 */
	{1, 2, 4, 7, 0x101, 0x102, 0x104, 0x107},
	/* ai_gain_14 */
	{1, 2, 3, 4, 5, 6, 7, 0x101, 0x102, 0x103, 0x104, 0x105, 0x106,
	 0x107},
	/* ai_gain_4 */
	{0, 1, 4, 7},
	/* ai_gain_611x */
	{0x00a, 0x00b, 0x001, 0x002, 0x003, 0x004, 0x005, 0x006},
	/* ai_gain_622x */
	{0, 1, 4, 5},
	/* ai_gain_628x */
	{1, 2, 3, 4, 5, 6, 7},
	/* ai_gain_6143 */
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};

struct a4l_rngtab rng_ni_E_ai = {16, {
	RANGE_V(-10, 10),
	RANGE_V(-5, 5),
	RANGE_V(-2.5, 2.5),
	RANGE_V(-1, 1),
	RANGE_V(-0.5, 0.5),
	RANGE_V(-0.25, 0.25),
	RANGE_V(-0.1, 0.1),
	RANGE_V(-0.05, 0.05),
	RANGE_V(0, 20),
	RANGE_V(0, 10),
	RANGE_V(0, 5),
	RANGE_V(0, 2),
	RANGE_V(0, 1),
	RANGE_V(0, 0.5),
	RANGE_V(0, 0.2),
	RANGE_V(0, 0.1),
}};
struct a4l_rngdesc a4l_range_ni_E_ai =
	RNG_GLOBAL(rng_ni_E_ai);

struct a4l_rngtab rng_ni_E_ai_limited = {8, {
	RANGE_V(-10, 10),
	RANGE_V(-5, 5),
	RANGE_V(-1, 1),
	RANGE_V(-0.1, 0.1),
	RANGE_V(0, 10),
	RANGE_V(0, 5),
	RANGE_V(0, 1),
	RANGE_V(0, 0.1),
}};
struct a4l_rngdesc a4l_range_ni_E_ai_limited =
	RNG_GLOBAL(rng_ni_E_ai_limited);

struct a4l_rngtab rng_ni_E_ai_limited14 = {14, {
	RANGE_V(-10, 10),
	RANGE_V(-5, 5),
	RANGE_V(-2, 2),
	RANGE_V(-1, 1),
	RANGE_V(-0.5, 0.5),
	RANGE_V(-0.2, 0.2),
	RANGE_V(-0.1, 0.1),
	RANGE_V(0, 10),
	RANGE_V(0, 5),
	RANGE_V(0, 2),
	RANGE_V(0, 1),
	RANGE_V(0, 0.5),
	RANGE_V(0, 0.2),
	RANGE_V(0, 0.1),
}};
struct a4l_rngdesc a4l_range_ni_E_ai_limited14 =
	RNG_GLOBAL(rng_ni_E_ai_limited14);

struct a4l_rngtab rng_ni_E_ai_bipolar4 = {4, {
	RANGE_V(-10,10),
	RANGE_V(-5, 5),
	RANGE_V(-0.5, 0.5),
	RANGE_V(-0.05, 0.05),
}};
struct a4l_rngdesc a4l_range_ni_E_ai_bipolar4 =
	RNG_GLOBAL(rng_ni_E_ai_bipolar4);

struct a4l_rngtab rng_ni_E_ai_611x = {8, {
	RANGE_V(-50, 50),
	RANGE_V(-20, 20),
	RANGE_V(-10, 10),
	RANGE_V(-5, 5),
	RANGE_V(-2, 2),
	RANGE_V(-1, 1),
	RANGE_V(-0.5, 0.5),
	RANGE_V(-0.2, 0.2),
}};
struct a4l_rngdesc a4l_range_ni_E_ai_611x =
	RNG_GLOBAL(rng_ni_E_ai_611x);

struct a4l_rngtab rng_ni_M_ai_622x = {4, {
	RANGE_V(-10, 10),
	RANGE_V(-5, 5),
	RANGE_V(-1, 1),
	RANGE_V(-0.2, 0.2),
}};
struct a4l_rngdesc a4l_range_ni_M_ai_622x =
	RNG_GLOBAL(rng_ni_M_ai_622x);

struct a4l_rngtab rng_ni_M_ai_628x = {7, {
	RANGE_V(-10, 10),
	RANGE_V(-5, 5),
	RANGE_V(-2, 2),
	RANGE_V(-1, 1),
	RANGE_V(-0.5, 0.5),
	RANGE_V(-0.2, 0.2),
	RANGE_V(-0.1, 0.1),
}};
struct a4l_rngdesc a4l_range_ni_M_ai_628x =
	RNG_GLOBAL(rng_ni_M_ai_628x);

struct a4l_rngtab rng_ni_S_ai_6143 = {1, {
	RANGE_V(-5, 5),
}};
struct a4l_rngdesc a4l_range_ni_S_ai_6143 =
	RNG_GLOBAL(rng_ni_S_ai_6143);


struct a4l_rngtab rng_ni_E_ao_ext = {4, {
	RANGE_V(-10, 10),
	RANGE_V(0, 10),
	RANGE_ext(-1, 1),
	RANGE_ext(0, 1),
}};
struct a4l_rngdesc a4l_range_ni_E_ao_ext =
	RNG_GLOBAL(rng_ni_E_ao_ext);

struct a4l_rngdesc *ni_range_lkup[] = {
	&a4l_range_ni_E_ai,
	&a4l_range_ni_E_ai_limited,
	&a4l_range_ni_E_ai_limited14,
	&a4l_range_ni_E_ai_bipolar4,
	&a4l_range_ni_E_ai_611x,
	&a4l_range_ni_M_ai_622x,
	&a4l_range_ni_M_ai_628x,
	&a4l_range_ni_S_ai_6143
};

static const int num_adc_stages_611x = 3;

static void ni_handle_fifo_dregs(struct a4l_subdevice *subd);
static void get_last_sample_611x(struct a4l_subdevice *subd);
static void get_last_sample_6143(struct a4l_subdevice *subd);
static void handle_cdio_interrupt(struct a4l_device *dev);
static void ni_load_channelgain_list(struct a4l_device *dev,
				     unsigned int n_chan, unsigned int *list);

#if (!defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) && \
     !defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))
static void ni_handle_fifo_half_full(struct a4l_subdevice *subd);
static int ni_ao_fifo_half_empty(struct a4l_subdevice *subd);
#endif /* !CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

static inline void ni_set_bitfield(struct a4l_device *dev,
				   int reg,
				   unsigned int bit_mask,
				   unsigned int bit_values)
{
	unsigned long flags;

	rtdm_lock_get_irqsave(&devpriv->soft_reg_copy_lock, flags);
	switch (reg) {
	case Interrupt_A_Enable_Register:
		devpriv->int_a_enable_reg &= ~bit_mask;
		devpriv->int_a_enable_reg |= bit_values & bit_mask;
		devpriv->stc_writew(dev, devpriv->int_a_enable_reg,
				    Interrupt_A_Enable_Register);
		break;
	case Interrupt_B_Enable_Register:
		devpriv->int_b_enable_reg &= ~bit_mask;
		devpriv->int_b_enable_reg |= bit_values & bit_mask;
		devpriv->stc_writew(dev, devpriv->int_b_enable_reg,
				    Interrupt_B_Enable_Register);
		break;
	case IO_Bidirection_Pin_Register:
		devpriv->io_bidirection_pin_reg &= ~bit_mask;
		devpriv->io_bidirection_pin_reg |= bit_values & bit_mask;
		devpriv->stc_writew(dev, devpriv->io_bidirection_pin_reg,
				    IO_Bidirection_Pin_Register);
		break;
	case AI_AO_Select:
		devpriv->ai_ao_select_reg &= ~bit_mask;
		devpriv->ai_ao_select_reg |= bit_values & bit_mask;
		ni_writeb(devpriv->ai_ao_select_reg, AI_AO_Select);
		break;
	case G0_G1_Select:
		devpriv->g0_g1_select_reg &= ~bit_mask;
		devpriv->g0_g1_select_reg |= bit_values & bit_mask;
		ni_writeb(devpriv->g0_g1_select_reg, G0_G1_Select);
		break;
	default:
		a4l_err(dev,
			"Warning %s() called with invalid register\n",
			__FUNCTION__);
		a4l_err(dev,"reg is %d\n", reg);
		break;
	}

	mmiowb();
	rtdm_lock_put_irqrestore(&devpriv->soft_reg_copy_lock, flags);
}

#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))

static int ni_ai_drain_dma(struct a4l_subdevice *subd);

static inline void ni_set_ai_dma_channel(struct a4l_device * dev, int channel)
{
	unsigned bitfield;

	if (channel >= 0) {
		bitfield =
			(ni_stc_dma_channel_select_bitfield(channel) <<
			 AI_DMA_Select_Shift) & AI_DMA_Select_Mask;
	} else {
		bitfield = 0;
	}
	ni_set_bitfield(dev, AI_AO_Select, AI_DMA_Select_Mask, bitfield);
}

static inline void ni_set_ao_dma_channel(struct a4l_device * dev, int channel)
{
	unsigned bitfield;

	if (channel >= 0) {
		bitfield =
			(ni_stc_dma_channel_select_bitfield(channel) <<
			 AO_DMA_Select_Shift) & AO_DMA_Select_Mask;
	} else {
		bitfield = 0;
	}
	ni_set_bitfield(dev, AI_AO_Select, AO_DMA_Select_Mask, bitfield);
}

static inline void ni_set_gpct_dma_channel(struct a4l_device * dev,
					   unsigned gpct_index, int mite_channel)
{
	unsigned bitfield;

	if (mite_channel >= 0) {
		bitfield = GPCT_DMA_Select_Bits(gpct_index, mite_channel);
	} else {
		bitfield = 0;
	}
	ni_set_bitfield(dev, G0_G1_Select, GPCT_DMA_Select_Mask(gpct_index),
			bitfield);
}

static inline void ni_set_cdo_dma_channel(struct a4l_device * dev, int mite_channel)
{
	unsigned long flags;

	rtdm_lock_get_irqsave(&devpriv->soft_reg_copy_lock, flags);
	devpriv->cdio_dma_select_reg &= ~CDO_DMA_Select_Mask;
	if (mite_channel >= 0) {
		/*XXX just guessing
		  ni_stc_dma_channel_select_bitfield() returns the right
		  bits, under the assumption the cdio dma selection
		  works just like ai/ao/gpct. Definitely works for dma
		  channels 0 and 1. */
		devpriv->cdio_dma_select_reg |=
			(ni_stc_dma_channel_select_bitfield(mite_channel) <<
			 CDO_DMA_Select_Shift) & CDO_DMA_Select_Mask;
	}
	ni_writeb(devpriv->cdio_dma_select_reg, M_Offset_CDIO_DMA_Select);
	mmiowb();
	rtdm_lock_put_irqrestore(&devpriv->soft_reg_copy_lock, flags);
}

static int ni_request_ai_mite_channel(struct a4l_device * dev)
{
	unsigned long flags;

	rtdm_lock_get_irqsave(&devpriv->mite_channel_lock, flags);
	BUG_ON(devpriv->ai_mite_chan);
	devpriv->ai_mite_chan =
		mite_request_channel(devpriv->mite, devpriv->ai_mite_ring);
	if (devpriv->ai_mite_chan == NULL) {
		rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock,
				      flags);
		a4l_err(dev,
			"ni_request_ai_mite_channel: "
			"failed to reserve mite dma channel for analog input.");
		return -EBUSY;
	}
	devpriv->ai_mite_chan->dir = A4L_INPUT;
	ni_set_ai_dma_channel(dev, devpriv->ai_mite_chan->channel);
	rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock, flags);
	return 0;
}

static int ni_request_ao_mite_channel(struct a4l_device * dev)
{
	unsigned long flags;

	rtdm_lock_get_irqsave(&devpriv->mite_channel_lock, flags);
	BUG_ON(devpriv->ao_mite_chan);
	devpriv->ao_mite_chan =
		mite_request_channel(devpriv->mite, devpriv->ao_mite_ring);
	if (devpriv->ao_mite_chan == NULL) {
		rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock,
				      flags);
		a4l_err(dev,
			"ni_request_ao_mite_channel: "
			"failed to reserve mite dma channel for analog outut.");
		return -EBUSY;
	}
	devpriv->ao_mite_chan->dir = A4L_OUTPUT;
	ni_set_ao_dma_channel(dev, devpriv->ao_mite_chan->channel);
	rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock, flags);
	return 0;
}

static int ni_request_gpct_mite_channel(struct a4l_device * dev,
					unsigned gpct_index, int direction)
{
	unsigned long flags;
	struct mite_channel *mite_chan;

	BUG_ON(gpct_index >= NUM_GPCT);
	rtdm_lock_get_irqsave(&devpriv->mite_channel_lock, flags);
	BUG_ON(devpriv->counter_dev->counters[gpct_index]->mite_chan);
	mite_chan = mite_request_channel(devpriv->mite,
					 devpriv->gpct_mite_ring[gpct_index]);
	if (mite_chan == NULL) {
		rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock,
				      flags);
		a4l_err(dev,
			"ni_request_gpct_mite_channel: "
			"failed to reserve mite dma channel for counter.");
		return -EBUSY;
	}
	mite_chan->dir = direction;
	a4l_ni_tio_set_mite_channel(devpriv->counter_dev->counters[gpct_index],
				mite_chan);
	ni_set_gpct_dma_channel(dev, gpct_index, mite_chan->channel);
	rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock, flags);
	return 0;
}

static int ni_request_cdo_mite_channel(struct a4l_device *dev)
{
	unsigned long flags;
	int err = 0;

	rtdm_lock_get_irqsave(&devpriv->mite_channel_lock, flags);

	/* No channel should be allocated... */
	BUG_ON(devpriv->cdo_mite_chan);
	/* ...until now */
	devpriv->cdo_mite_chan =
		mite_request_channel(devpriv->mite, devpriv->cdo_mite_ring);

	if (devpriv->cdo_mite_chan) {
		devpriv->cdo_mite_chan->dir = A4L_OUTPUT;
		ni_set_cdo_dma_channel(dev, devpriv->cdo_mite_chan->channel);
	} else {
		err = -EBUSY;
		a4l_err(dev,
			"ni_request_cdo_mite_channel: "
			"failed to reserve mite dma channel "
			"for correlated digital outut.");
	}

	rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock, flags);

	return err;
}

void ni_release_ai_mite_channel(struct a4l_device *dev)
{
	unsigned long flags;

	rtdm_lock_get_irqsave(&devpriv->mite_channel_lock, flags);
	if (devpriv->ai_mite_chan) {
		ni_set_ai_dma_channel(dev, -1);
		a4l_mite_release_channel(devpriv->ai_mite_chan);
		devpriv->ai_mite_chan = NULL;
	}
	rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock, flags);

}

void ni_release_ao_mite_channel(struct a4l_device *dev)
{
	unsigned long flags;

	rtdm_lock_get_irqsave(&devpriv->mite_channel_lock, flags);
	if (devpriv->ao_mite_chan) {
		ni_set_ao_dma_channel(dev, -1);
		a4l_mite_release_channel(devpriv->ao_mite_chan);
		devpriv->ao_mite_chan = NULL;
	}
	rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock, flags);

}

void ni_release_gpct_mite_channel(struct a4l_device *dev, unsigned gpct_index)
{
	unsigned long flags;

	BUG_ON(gpct_index >= NUM_GPCT);
	rtdm_lock_get_irqsave(&devpriv->mite_channel_lock, flags);
	if (devpriv->counter_dev->counters[gpct_index]->mite_chan) {
		struct mite_channel *mite_chan =
			devpriv->counter_dev->counters[gpct_index]->mite_chan;

		ni_set_gpct_dma_channel(dev, gpct_index, -1);
		a4l_ni_tio_set_mite_channel(devpriv->counter_dev->
					counters[gpct_index], NULL);
		a4l_mite_release_channel(mite_chan);
	}
	rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock, flags);

}

void ni_release_cdo_mite_channel(struct a4l_device *dev)
{
	unsigned long flags;

	rtdm_lock_get_irqsave(&devpriv->mite_channel_lock, flags);
	if (devpriv->cdo_mite_chan) {
		ni_set_cdo_dma_channel(dev, -1);
		a4l_mite_release_channel(devpriv->cdo_mite_chan);
		devpriv->cdo_mite_chan = NULL;
	}
	rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock, flags);

}

void ni_sync_ai_dma(struct a4l_subdevice *subd)
{
	struct a4l_device *dev = subd->dev;
	unsigned long flags;

	rtdm_lock_get_irqsave(&devpriv->mite_channel_lock, flags);
	if (devpriv->ai_mite_chan)
		a4l_mite_sync_input_dma(devpriv->ai_mite_chan, subd);
	rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock, flags);
}

void mite_handle_b_linkc(struct a4l_subdevice *subd)
{
	struct a4l_device *dev = subd->dev;
	unsigned long flags;

	rtdm_lock_get_irqsave(&devpriv->mite_channel_lock, flags);
	if (devpriv->ao_mite_chan)
		a4l_mite_sync_output_dma(devpriv->ao_mite_chan, subd);
	rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock, flags);
}

static int ni_ao_wait_for_dma_load(struct a4l_subdevice *subd)
{
	static const int timeout = 10000;

	struct a4l_device *dev = subd->dev;
	struct a4l_buffer *buf = subd->buf;

	int i;

	for (i = 0; i < timeout; i++) {

		int buffer_filled;
		unsigned short b_status;

		b_status = devpriv->stc_readw(dev, AO_Status_1_Register);

		buffer_filled = test_bit(A4L_BUF_EOA_NR, &buf->flags);
		buffer_filled |= (b_status & AO_FIFO_Half_Full_St);

		if (buffer_filled)
			break;

		/* If we poll too often, the pci bus activity seems
		   to slow the dma transfer down */
		a4l_udelay(10);
	}

	if (i == timeout) {
		a4l_err(dev,
			"ni_ao_wait_for_dma_load: "
			"timed out waiting for dma load");
		return -EPIPE;
	}

	return 0;
}


#else /* !CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

static inline int ni_ai_drain_dma(struct a4l_subdevice *subd)
{
	return -ENOTSUPP;
}

static inline int ni_request_ai_mite_channel(struct a4l_device * dev)
{
	return -ENOTSUPP;
}

static inline int ni_request_ao_mite_channel(struct a4l_device * dev)
{
	return -ENOTSUPP;
}

static inline
int ni_request_gpct_mite_channel(struct a4l_device * dev,
				 unsigned gpct_index, int direction)
{
	return -ENOTSUPP;
}

static inline int ni_request_cdo_mite_channel(struct a4l_device *dev)
{
	return -ENOTSUPP;
}

#define ni_release_ai_mite_channel(x) do { } while (0)
#define ni_release_ao_mite_channel(x) do { } while (0)
#define ni_release_gpct_mite_channel(x) do { } while (0)
#define ni_release_cdo_mite_channel(x) do { } while (0)
#define ni_sync_ai_dma(x) do { } while (0)
#define mite_handle_b_linkc(x) do { } while (0)

static inline int ni_ao_wait_for_dma_load(struct a4l_subdevice *subd)
{
	return -ENOTSUPP;
}

#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

/* E-series boards use the second irq signals to generate dma requests
   for their counters */
void ni_e_series_enable_second_irq(struct a4l_device *dev,
				   unsigned gpct_index, short enable)
{
	if (boardtype.reg_type & ni_reg_m_series_mask)
		return;
	switch (gpct_index) {
	case 0:
		if (enable) {
			devpriv->stc_writew(dev, G0_Gate_Second_Irq_Enable,
					    Second_IRQ_A_Enable_Register);
		} else {
			devpriv->stc_writew(dev, 0,
					    Second_IRQ_A_Enable_Register);
		}
		break;
	case 1:
		if (enable) {
			devpriv->stc_writew(dev, G1_Gate_Second_Irq_Enable,
					    Second_IRQ_B_Enable_Register);
		} else {
			devpriv->stc_writew(dev, 0,
					    Second_IRQ_B_Enable_Register);
		}
		break;
	default:
		BUG();
		break;
	}
}

void ni_clear_ai_fifo(struct a4l_device *dev)
{
	if (boardtype.reg_type == ni_reg_6143) {
		/* Flush the 6143 data FIFO */
		ni_writel(0x10, AIFIFO_Control_6143); /* Flush fifo */
		ni_writel(0x00, AIFIFO_Control_6143); /* Flush fifo */
		while (ni_readl(AIFIFO_Status_6143) & 0x10); /* Wait for complete */
	} else {
		devpriv->stc_writew(dev, 1, ADC_FIFO_Clear);
		if (boardtype.reg_type == ni_reg_625x) {
			ni_writeb(0, M_Offset_Static_AI_Control(0));
			ni_writeb(1, M_Offset_Static_AI_Control(0));
		}
	}
}

#define ao_win_out(data, addr) ni_ao_win_outw(dev, data, addr)
static inline void ni_ao_win_outw(struct a4l_device *dev, uint16_t data, int addr)
{
	unsigned long flags;

	rtdm_lock_get_irqsave(&devpriv->window_lock, flags);
	ni_writew(addr, AO_Window_Address_611x);
	ni_writew(data, AO_Window_Data_611x);
	rtdm_lock_put_irqrestore(&devpriv->window_lock, flags);
}

static inline void ni_ao_win_outl(struct a4l_device *dev, uint32_t data, int addr)
{
	unsigned long flags;

	rtdm_lock_get_irqsave(&devpriv->window_lock, flags);
	ni_writew(addr, AO_Window_Address_611x);
	ni_writel(data, AO_Window_Data_611x);
	rtdm_lock_put_irqrestore(&devpriv->window_lock, flags);
}

static inline unsigned short ni_ao_win_inw(struct a4l_device *dev, int addr)
{
	unsigned long flags;
	unsigned short data;

	rtdm_lock_get_irqsave(&devpriv->window_lock, flags);
	ni_writew(addr, AO_Window_Address_611x);
	data = ni_readw(AO_Window_Data_611x);
	rtdm_lock_put_irqrestore(&devpriv->window_lock, flags);
	return data;
}

/*
 * ni_set_bits( ) allows different parts of the ni_mio_common driver
 * to share registers (such as Interrupt_A_Register) without interfering
 * with each other.
 *
 * NOTE: the switch/case statements are optimized out for a constant
 * argument so this is actually quite fast--- If you must wrap another
 * function around this make it inline to avoid a large speed penalty.
 *
 * value should only be 1 or 0.
 */

static inline void ni_set_bits(struct a4l_device *dev,
			       int reg, unsigned bits, unsigned value)
{
	unsigned bit_values;

	if (value)
		bit_values = bits;
	else
		bit_values = 0;

	ni_set_bitfield(dev, reg, bits, bit_values);
}

static void shutdown_ai_command(struct a4l_subdevice *subd)
{
	ni_ai_drain_dma(subd);
	ni_handle_fifo_dregs(subd);
	get_last_sample_611x(subd);
	get_last_sample_6143(subd);

	/* TODO: stop the acquisiton */
}

static void ni_handle_eos(struct a4l_subdevice *subd)
{
	struct a4l_device *dev = subd->dev;

	if (devpriv->aimode == AIMODE_SCAN) {
		static const int timeout = 10;
		int i;

		for (i = 0; i < timeout; i++) {
			ni_sync_ai_dma(subd);
			/* TODO: stop when the transfer is really over */
			a4l_udelay(1);
		}
	}

	/* Handle special case of single scan using AI_End_On_End_Of_Scan */
	if ((devpriv->ai_cmd2 & AI_End_On_End_Of_Scan)) {
		shutdown_ai_command(subd);
	}
}

static void ni_event(struct a4l_subdevice * subd)
{
	/* Temporary hack */
	struct a4l_buffer *buf = subd->buf;

	if(test_bit(A4L_BUF_ERROR_NR, &buf->flags)) {
		if (subd->cancel != NULL)
			subd->cancel(subd);
	}

	a4l_buf_evt(subd, 0);

}

static void handle_gpct_interrupt(struct a4l_device *dev, unsigned short counter_index)
{
#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))
	struct ni_gpct *counter = devpriv->counter_dev->counters[counter_index];
	a4l_ni_tio_handle_interrupt(counter, dev);
#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */
}

#ifdef CONFIG_DEBUG_MIO_COMMON
static const char *const status_a_strings[] = {
	"passthru0", "fifo", "G0_gate", "G0_TC",
	"stop", "start", "sc_tc", "start1",
	"start2", "sc_tc_error", "overflow", "overrun",
	"fifo_empty", "fifo_half_full", "fifo_full", "interrupt_a"
};

static void ni_mio_print_status_a(int status)
{
	int i;

	__a4l_info("A status:");
	for (i = 15; i >= 0; i--) {
		if (status & (1 << i)) {
			__a4l_info(" %s", status_a_strings[i]);
		}
	}
	__a4l_info("\n");
}

static const char *const status_b_strings[] = {
	"passthru1", "fifo", "G1_gate", "G1_TC",
	"UI2_TC", "UPDATE", "UC_TC", "BC_TC",
	"start1", "overrun", "start", "bc_tc_error",
	"fifo_empty", "fifo_half_full", "fifo_full", "interrupt_b"
};

static void ni_mio_print_status_b(int status)
{
	int i;

	__a4l_info("B status:");
	for (i = 15; i >= 0; i--) {
		if (status & (1 << i)) {
			__a4l_info(" %s", status_b_strings[i]);
		}
	}
	__a4l_info("\n");
}

#else /* !CONFIG_DEBUG_MIO_COMMON */

#define ni_mio_print_status_a(x)
#define ni_mio_print_status_b(x)

#endif /* CONFIG_DEBUG_MIO_COMMON */

static void ack_a_interrupt(struct a4l_device *dev, unsigned short a_status)
{
	unsigned short ack = 0;

	if (a_status & AI_SC_TC_St) {
		ack |= AI_SC_TC_Interrupt_Ack;
	}
	if (a_status & AI_START1_St) {
		ack |= AI_START1_Interrupt_Ack;
	}
	if (a_status & AI_START_St) {
		ack |= AI_START_Interrupt_Ack;
	}
	if (a_status & AI_STOP_St) {
		/* not sure why we used to ack the START here also,
		   instead of doing it independently. Frank Hess
		   2007-07-06 */
		ack |= AI_STOP_Interrupt_Ack;
	}
	if (ack)
		devpriv->stc_writew(dev, ack, Interrupt_A_Ack_Register);
}

static void handle_a_interrupt(struct a4l_device *dev,
			       unsigned short status,unsigned int ai_mite_status)
{

	struct a4l_subdevice *subd = a4l_get_subd(dev, NI_AI_SUBDEV);

	/* 67xx boards don't have ai subdevice, but their gpct0
	   might generate an a interrupt. */

	if((subd->flags & A4L_SUBD_TYPES) == A4L_SUBD_UNUSED)
		return;

	a4l_dbg(1, drv_dbg, dev, "ni_mio_common: interrupt: "
		"a_status=%04x ai_mite_status=%08x\n",status, ai_mite_status);
	ni_mio_print_status_a(status);

#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))
	if (ai_mite_status & CHSR_LINKC)
		ni_sync_ai_dma(subd);

	if (ai_mite_status & ~(CHSR_INT | CHSR_LINKC | CHSR_DONE | CHSR_MRDY |
			       CHSR_DRDY | CHSR_DRQ1 | CHSR_DRQ0 | CHSR_ERROR |
			       CHSR_SABORT | CHSR_XFERR | CHSR_LxERR_mask)) {
		a4l_dbg(1, drv_dbg, dev, "ni_mio_common: interrupt: "
			"unknown mite interrupt, ack! (ai_mite_status=%08x)\n",
			ai_mite_status);
		a4l_buf_evt(subd, A4L_BUF_ERROR);
	}
#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

	/* Test for all uncommon interrupt events at the same time */
	if (status & (AI_Overrun_St | AI_Overflow_St | AI_SC_TC_Error_St |
		      AI_SC_TC_St | AI_START1_St)) {
		if (status == 0xffff) {
			a4l_dbg(1, drv_dbg, dev, "ni_mio_common: interrupt: "
				"a_status=0xffff.  Card removed?\n");
			/* TODO: we probably aren't even running a command now,
			   so it's a good idea to be careful.
			   we should check the transfer status */
			a4l_buf_evt(subd, A4L_BUF_ERROR);
			ni_event(subd);
			return;
		}
		if (status & (AI_Overrun_St | AI_Overflow_St |
			      AI_SC_TC_Error_St)) {
			a4l_dbg(1, drv_dbg, dev, "ni_mio_common: interrupt: "
				"ai error a_status=%04x\n", status);
			ni_mio_print_status_a(status);

			shutdown_ai_command(subd);

			a4l_buf_evt(subd, A4L_BUF_ERROR);
			ni_event(subd);

			return;
		}
		if (status & AI_SC_TC_St) {
			a4l_dbg(1, drv_dbg, dev, "ni_mio_common: SC_TC interrupt\n");
			if (!devpriv->ai_continuous) {
				shutdown_ai_command(subd);
			}
		}
	}

#if (!defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) && \
     !defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))

	if (status & AI_FIFO_Half_Full_St) {
		int i;
		static const int timeout = 10;
		/* PCMCIA cards (at least 6036) seem to stop producing
		   interrupts if we fail to get the fifo less than half
		   full, so loop to be sure. */
		for (i = 0; i < timeout; ++i) {
			ni_handle_fifo_half_full(subd);
			if ((devpriv->stc_readw(dev, AI_Status_1_Register) &
			     AI_FIFO_Half_Full_St) == 0)
				break;
		}
	}
#endif /* !CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

	if ((status & AI_STOP_St)) {
		ni_handle_eos(subd);
	}

	ni_event(subd);

	status = devpriv->stc_readw(dev, AI_Status_1_Register);
	if (status & Interrupt_A_St)
		a4l_dbg(1, drv_dbg, dev, "ni_mio_common: interrupt: "
			" didn't clear interrupt? status=0x%x\n", status);
}

static void ack_b_interrupt(struct a4l_device *dev, unsigned short b_status)
{
	unsigned short ack = 0;
	if (b_status & AO_BC_TC_St) {
		ack |= AO_BC_TC_Interrupt_Ack;
	}
	if (b_status & AO_Overrun_St) {
		ack |= AO_Error_Interrupt_Ack;
	}
	if (b_status & AO_START_St) {
		ack |= AO_START_Interrupt_Ack;
	}
	if (b_status & AO_START1_St) {
		ack |= AO_START1_Interrupt_Ack;
	}
	if (b_status & AO_UC_TC_St) {
		ack |= AO_UC_TC_Interrupt_Ack;
	}
	if (b_status & AO_UI2_TC_St) {
		ack |= AO_UI2_TC_Interrupt_Ack;
	}
	if (b_status & AO_UPDATE_St) {
		ack |= AO_UPDATE_Interrupt_Ack;
	}
	if (ack)
		devpriv->stc_writew(dev, ack, Interrupt_B_Ack_Register);
}

static void handle_b_interrupt(struct a4l_device * dev,
			       unsigned short b_status, unsigned int ao_mite_status)
{

	struct a4l_subdevice *subd = a4l_get_subd(dev, NI_AO_SUBDEV);

	a4l_dbg(1, drv_dbg, dev,
		"ni_mio_common: interrupt: b_status=%04x m1_status=%08x\n",
		b_status, ao_mite_status);

	ni_mio_print_status_b(b_status);

	if (b_status == 0xffff)
		return;

	if (b_status & AO_Overrun_St) {
		a4l_err(dev,
			"ni_mio_common: interrupt: "
			"AO FIFO underrun status=0x%04x status2=0x%04x\n",
			b_status,
			devpriv->stc_readw(dev, AO_Status_2_Register));
		a4l_buf_evt(subd, A4L_BUF_ERROR);
	}

	if (b_status & AO_BC_TC_St) {
		a4l_dbg(1, drv_dbg, dev,
			"ni_mio_common: interrupt: "
			"AO BC_TC status=0x%04x status2=0x%04x\n",
			b_status, devpriv->stc_readw(dev, AO_Status_2_Register));
		a4l_buf_evt(subd, A4L_BUF_EOA);
	}

#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))

	if (ao_mite_status & CHSR_STOPS) {
		a4l_dbg(1, drv_dbg, dev,
			"ni_mio_common: interrupt: MITE transfer stopped\n");
	} else if (ao_mite_status & CHSR_LINKC) {
		/* Currently, mite.c requires us to handle LINKC */
		mite_handle_b_linkc(subd);
	}

	if (ao_mite_status &
	    ~(CHSR_INT | CHSR_LINKC | CHSR_DONE | CHSR_MRDY |
	      CHSR_DRDY | CHSR_DRQ1 | CHSR_DRQ0 | CHSR_ERROR |
	      CHSR_SABORT | CHSR_STOPS | CHSR_XFERR | CHSR_LxERR_mask)) {
		a4l_err(dev,
			"unknown mite interrupt, ack! (ao_mite_status=%08x)\n",
			 ao_mite_status);
		a4l_buf_evt(subd, A4L_BUF_ERROR);
	}
#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

#if (!defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) && \
     !defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))

	if (b_status & AO_FIFO_Request_St) {
		int ret;

		ret = ni_ao_fifo_half_empty(subd);
		if (!ret) {
			a4l_err(dev,
				"ni_mio_common: "
				"interrupt: AO buffer underrun\n");
			ni_set_bits(dev, Interrupt_B_Enable_Register,
				    AO_FIFO_Interrupt_Enable |
				    AO_Error_Interrupt_Enable, 0);
			a4l_buf_evt(subd, A4L_BUF_ERROR);
		}
	}
#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

	ni_event(subd);
}

int a4l_ni_E_interrupt(unsigned int irq, void *d)
{
	struct a4l_device *dev = d;
	unsigned short a_status;
	unsigned short b_status;
	unsigned int ai_mite_status = 0;
	unsigned int ao_mite_status = 0;
	unsigned long flags;
	struct mite_struct *mite = devpriv->mite;

	/* Make sure dev->attached is checked before handler does
	   anything else. */
	smp_mb();

	/* lock to avoid race with a4l_poll */
	rtdm_lock_get_irqsave(&dev->lock, flags);
	a_status = devpriv->stc_readw(dev, AI_Status_1_Register);
	b_status = devpriv->stc_readw(dev, AO_Status_1_Register);
	if (mite) {
#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))
		rtdm_lock_get(&devpriv->mite_channel_lock);
		if (devpriv->ai_mite_chan) {
			ai_mite_status = a4l_mite_get_status(devpriv->ai_mite_chan);
			if (ai_mite_status & CHSR_LINKC)
				writel(CHOR_CLRLC,
				       devpriv->mite->mite_io_addr +
				       MITE_CHOR(devpriv->ai_mite_chan->channel));
		}
		if (devpriv->ao_mite_chan) {
			ao_mite_status = a4l_mite_get_status(devpriv->ao_mite_chan);
			if (ao_mite_status & CHSR_LINKC)
				writel(CHOR_CLRLC,
				       mite->mite_io_addr +
				       MITE_CHOR(devpriv->ao_mite_chan->channel));
		}
		rtdm_lock_put(&devpriv->mite_channel_lock);
#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */
	}
	ack_a_interrupt(dev, a_status);
	ack_b_interrupt(dev, b_status);
	if ((a_status & Interrupt_A_St) || (ai_mite_status & CHSR_INT))
		handle_a_interrupt(dev, a_status, ai_mite_status);
	if ((b_status & Interrupt_B_St) || (ao_mite_status & CHSR_INT))
		handle_b_interrupt(dev, b_status, ao_mite_status);
	handle_gpct_interrupt(dev, 0);
	handle_gpct_interrupt(dev, 1);
	handle_cdio_interrupt(dev);

	rtdm_lock_put_irqrestore(&dev->lock, flags);
	return 0;
}

#if (!defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) && \
     !defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))

static void ni_ao_fifo_load(struct a4l_subdevice *subd, int n)
{
	struct a4l_device *dev = subd->dev;
	sampl_t d;
	u32 packed_data;
	int i, err = 1;

	for (i = 0; i < n; i++) {
		err = a4l_buf_get(subd, &d, sizeof(sampl_t));
		if (err != 0)
			break;

		if (boardtype.reg_type & ni_reg_6xxx_mask) {
			packed_data = d & 0xffff;
			/* 6711 only has 16 bit wide ao fifo */
			if (boardtype.reg_type != ni_reg_6711) {
				err = a4l_buf_get(subd, &d, sizeof(sampl_t));
				if (err != 0)
					break;
				i++;
				packed_data |= (d << 16) & 0xffff0000;
			}
			ni_writel(packed_data, DAC_FIFO_Data_611x);
		} else {
			ni_writew(d, DAC_FIFO_Data);
		}
	}
	if (err != 0) {
		a4l_buf_evt(subd, A4L_BUF_ERROR);
	}
}

/*
 *  There's a small problem if the FIFO gets really low and we
 *  don't have the data to fill it.  Basically, if after we fill
 *  the FIFO with all the data available, the FIFO is _still_
 *  less than half full, we never clear the interrupt.  If the
 *  IRQ is in edge mode, we never get another interrupt, because
 *  this one wasn't cleared.  If in level mode, we get flooded
 *  with interrupts that we can't fulfill, because nothing ever
 *  gets put into the buffer.
 *
 *  This kind of situation is recoverable, but it is easier to
 *  just pretend we had a FIFO underrun, since there is a good
 *  chance it will happen anyway.  This is _not_ the case for
 *  RT code, as RT code might purposely be running close to the
 *  metal.  Needs to be fixed eventually.
 */
static int ni_ao_fifo_half_empty(struct a4l_subdevice *subd)
{
	struct a4l_device *dev = subd->dev;
	int n;

	n = a4l_buf_count(subd);
	if (n == 0) {
		a4l_buf_evt(subd, A4L_BUF_ERROR);
		return 0;
	}

	n /= sizeof(sampl_t);
	if (n > boardtype.ao_fifo_depth / 2)
		n = boardtype.ao_fifo_depth / 2;

	ni_ao_fifo_load(subd, n);

	return 1;
}

static int ni_ao_prep_fifo(struct a4l_subdevice *subd)
{
	struct a4l_device *dev = subd->dev;
	int n;

	/* Reset fifo */
	devpriv->stc_writew(dev, 1, DAC_FIFO_Clear);
	if (boardtype.reg_type & ni_reg_6xxx_mask)
		ni_ao_win_outl(dev, 0x6, AO_FIFO_Offset_Load_611x);

	/* Load some data */
	n = a4l_buf_count(subd);
	if (n == 0)
		return 0;

	n /= sizeof(sampl_t);
	if (n > boardtype.ao_fifo_depth)
		n = boardtype.ao_fifo_depth;

	ni_ao_fifo_load(subd, n);

	return n;
}

static void ni_ai_fifo_read(struct a4l_subdevice *subd, int n)
{
	struct a4l_device *dev = subd->dev;
	int i;

	if (boardtype.reg_type == ni_reg_611x) {
		sampl_t data[2];
		u32 dl;

		for (i = 0; i < n / 2; i++) {
			dl = ni_readl(ADC_FIFO_Data_611x);
			/* This may get the hi/lo data in the wrong order */
			data[0] = (dl >> 16) & 0xffff;
			data[1] = dl & 0xffff;
			a4l_buf_put(subd, data, sizeof(sampl_t) * 2);
		}
		/* Check if there's a single sample stuck in the FIFO */
		if (n % 2) {
			dl = ni_readl(ADC_FIFO_Data_611x);
			data[0] = dl & 0xffff;
			a4l_buf_put(subd, &data[0], sizeof(sampl_t));
		}
	} else if (boardtype.reg_type == ni_reg_6143) {
		sampl_t data[2];
		u32 dl;

		/* This just reads the FIFO assuming the data is
		   present, no checks on the FIFO status are performed */
		for (i = 0; i < n / 2; i++) {
			dl = ni_readl(AIFIFO_Data_6143);

			data[0] = (dl >> 16) & 0xffff;
			data[1] = dl & 0xffff;
			a4l_buf_put(subd, data, sizeof(sampl_t) * 2);
		}
		if (n % 2) {
			/* Assume there is a single sample stuck in the FIFO.
			   Get stranded sample into FIFO */
			ni_writel(0x01, AIFIFO_Control_6143);
			dl = ni_readl(AIFIFO_Data_6143);
			data[0] = (dl >> 16) & 0xffff;
			a4l_buf_put(subd, &data[0], sizeof(sampl_t));
		}
	} else {
		if (n > sizeof(devpriv->ai_fifo_buffer) /
		    sizeof(devpriv->ai_fifo_buffer[0])) {
			a4l_err(dev,
				"ni_ai_fifo_read: "
				"bug! ai_fifo_buffer too small");
			a4l_buf_evt(subd, A4L_BUF_ERROR);
			return;
		}
		for (i = 0; i < n; i++) {
			devpriv->ai_fifo_buffer[i] =
				ni_readw(ADC_FIFO_Data_Register);
		}
		a4l_buf_put(subd,
			    devpriv->ai_fifo_buffer,
			    n * sizeof(devpriv->ai_fifo_buffer[0]));
	}
}

static void ni_handle_fifo_half_full(struct a4l_subdevice *subd)
{
	struct a4l_device *dev = subd->dev;
	ni_ai_fifo_read(subd, boardtype.ai_fifo_depth / 2);
}

#endif /* !CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))

static int ni_ai_drain_dma(struct a4l_subdevice *subd)
{
	int i;
	static const int timeout = 10000;
	unsigned long flags;
	int retval = 0;
	struct a4l_device *dev = subd->dev;

	rtdm_lock_get_irqsave(&devpriv->mite_channel_lock, flags);
	if (devpriv->ai_mite_chan) {
		for (i = 0; i < timeout; i++) {
			if ((devpriv->stc_readw(dev,
						AI_Status_1_Register) &
			     AI_FIFO_Empty_St)
			    && a4l_mite_bytes_in_transit(devpriv->
						     ai_mite_chan) == 0)
				break;
			a4l_udelay(5);
		}
		if (i == timeout) {
			a4l_info(dev, "wait for dma drain timed out\n");

			a4l_info(dev, "a4l_mite_bytes_in_transit=%i, "
				 "AI_Status1_Register=0x%x\n",
				 a4l_mite_bytes_in_transit(devpriv->ai_mite_chan),
				 devpriv->stc_readw(dev, AI_Status_1_Register));
			retval = -1;
		}
	}
	rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock, flags);

	ni_sync_ai_dma(subd);

	return retval;
}

#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

/* Empties the AI fifo */
static void ni_handle_fifo_dregs(struct a4l_subdevice *subd)
{
	sampl_t data[2];
	u32 dl;
	short fifo_empty;
	int i;
	struct a4l_device *dev = subd->dev;

	if (boardtype.reg_type == ni_reg_611x) {
		while ((devpriv->stc_readw(dev,
					   AI_Status_1_Register) &
			AI_FIFO_Empty_St) == 0) {
			dl = ni_readl(ADC_FIFO_Data_611x);

			/* This may get the hi/lo data in the wrong order */
			data[0] = (dl >> 16);
			data[1] = (dl & 0xffff);
			a4l_buf_put(subd, data, sizeof(sampl_t) * 2);
		}
	} else if (boardtype.reg_type == ni_reg_6143) {
		i = 0;
		while (ni_readl(AIFIFO_Status_6143) & 0x04) {
			dl = ni_readl(AIFIFO_Data_6143);

			/* This may get the hi/lo data in the wrong order */
			data[0] = (dl >> 16);
			data[1] = (dl & 0xffff);
			a4l_buf_put(subd, data, sizeof(sampl_t) * 2);
			i += 2;
		}
		// Check if stranded sample is present
		if (ni_readl(AIFIFO_Status_6143) & 0x01) {
			ni_writel(0x01, AIFIFO_Control_6143);	// Get stranded sample into FIFO
			dl = ni_readl(AIFIFO_Data_6143);
			data[0] = (dl >> 16) & 0xffff;
			a4l_buf_put(subd, &data[0], sizeof(sampl_t));
		}

	} else {
		fifo_empty =
			devpriv->stc_readw(dev,
					   AI_Status_1_Register) & AI_FIFO_Empty_St;
		while (fifo_empty == 0) {
			for (i = 0;
			     i <
				     sizeof(devpriv->ai_fifo_buffer) /
				     sizeof(devpriv->ai_fifo_buffer[0]); i++) {
				fifo_empty =
					devpriv->stc_readw(dev,
							   AI_Status_1_Register) &
					AI_FIFO_Empty_St;
				if (fifo_empty)
					break;
				devpriv->ai_fifo_buffer[i] =
					ni_readw(ADC_FIFO_Data_Register);
			}
			a4l_buf_put(subd,
				    devpriv->ai_fifo_buffer,
				    i * sizeof(devpriv->ai_fifo_buffer[0]));
		}
	}
}

static void get_last_sample_611x(struct a4l_subdevice *subd)
{
	sampl_t data;
	u32 dl;
	struct a4l_device *dev = subd->dev;

	if (boardtype.reg_type != ni_reg_611x)
		return;

	/* Check if there's a single sample stuck in the FIFO */
	if (ni_readb(XXX_Status) & 0x80) {
		dl = ni_readl(ADC_FIFO_Data_611x);
		data = (dl & 0xffff);
		a4l_buf_put(subd, &data, sizeof(sampl_t));
	}
}

static void get_last_sample_6143(struct a4l_subdevice *subd)
{
	sampl_t data;
	u32 dl;
	struct a4l_device *dev = subd->dev;

	if (boardtype.reg_type != ni_reg_6143)
		return;

	/* Check if there's a single sample stuck in the FIFO */
	if (ni_readl(AIFIFO_Status_6143) & 0x01) {
		/* Get stranded sample into FIFO */
		ni_writel(0x01, AIFIFO_Control_6143);
		dl = ni_readl(AIFIFO_Data_6143);

		/* This may get the hi/lo data in the wrong order */
		data = (dl >> 16) & 0xffff;
		a4l_buf_put(subd, &data, sizeof(sampl_t));
	}
}

static void ni_ai_munge16(struct a4l_subdevice *subd, void *buf, unsigned long size)
{
	struct a4l_device *dev = subd->dev;
	struct a4l_cmd_desc *cmd = a4l_get_cmd(subd);
	int chan_idx = a4l_get_chan(subd);
	unsigned int i;
	sampl_t *array = buf;

	for (i = 0; i < size / sizeof(sampl_t); i++) {
#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))
		array[i] = le16_to_cpu(array[i]);
#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */
		array[i] += devpriv->ai_offset[chan_idx];
		chan_idx++;
		chan_idx %= cmd->nb_chan;
	}
}

static void ni_ai_munge32(struct a4l_subdevice *subd, void *buf, unsigned long size)
{
	struct a4l_device *dev = subd->dev;
	struct a4l_cmd_desc *cmd = a4l_get_cmd(subd);
	int chan_idx = a4l_get_chan(subd);
	unsigned int i;
	lsampl_t *larray = buf;

	for (i = 0; i < size / sizeof(lsampl_t); i++) {
#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))
		larray[i] = le32_to_cpu(larray[i]);
#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */
		larray[i] += devpriv->ai_offset[chan_idx];
		chan_idx++;
		chan_idx %= cmd->nb_chan;
	}
}

#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))

static int ni_ai_setup_MITE_dma(struct a4l_subdevice *subd)
{
	struct a4l_device *dev = subd->dev;
	unsigned long flags;
	int err;

	err = ni_request_ai_mite_channel(dev);
	if (err < 0)
		return err;

	err = a4l_mite_buf_change(devpriv->ai_mite_chan->ring, subd);
	if (err < 0)
		return err;

	rtdm_lock_get_irqsave(&devpriv->mite_channel_lock, flags);

	switch (boardtype.reg_type) {
	case ni_reg_611x:
	case ni_reg_6143:
		a4l_mite_prep_dma(devpriv->ai_mite_chan, 32, 16);
		break;
	case ni_reg_628x:
		a4l_mite_prep_dma(devpriv->ai_mite_chan, 32, 32);
		break;
	default:
		a4l_mite_prep_dma(devpriv->ai_mite_chan, 16, 16);
		break;
	};

	/* start the MITE */
	a4l_mite_dma_arm(devpriv->ai_mite_chan);

	rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock, flags);

	return 0;
}

static int ni_ao_setup_MITE_dma(struct a4l_subdevice *subd)
{
	struct a4l_device *dev = subd->dev;
	unsigned long flags;
	int err;

	err = ni_request_ao_mite_channel(dev);
	if (err < 0)
		return err;

	err = a4l_mite_buf_change(devpriv->ao_mite_chan->ring, subd);
	if (err < 0)
		return err;

	rtdm_lock_get_irqsave(&devpriv->mite_channel_lock, flags);

	if (devpriv->ao_mite_chan) {

		if (boardtype.reg_type & (ni_reg_611x | ni_reg_6713)) {
			a4l_mite_prep_dma(devpriv->ao_mite_chan, 32, 32);
		} else {
			/* Doing 32 instead of 16 bit wide transfers
			   from memory makes the mite do 32 bit pci
			   transfers, doubling pci bandwidth. */
			a4l_mite_prep_dma(devpriv->ao_mite_chan, 16, 32);
		}
		a4l_mite_dma_arm(devpriv->ao_mite_chan);
	} else
		err = -EIO;

	rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock, flags);

	return err;
}

static int ni_cdo_setup_MITE_dma(struct a4l_subdevice *subd)
{
	struct a4l_device *dev = subd->dev;
	unsigned long flags;
	int err;

	err = ni_request_cdo_mite_channel(dev);
	if (err < 0)
		return err;

	/* No need to get a lock to setup the ring buffer */
	err = a4l_mite_buf_change(devpriv->cdo_mite_chan->ring, subd);
	if (err < 0)
		return err;

	rtdm_lock_get_irqsave(&devpriv->mite_channel_lock, flags);

	/* This test should be useless but one never knows */
	if (devpriv->cdo_mite_chan) {
		/* Configure the DMA transfer */
		a4l_mite_prep_dma(devpriv->cdo_mite_chan, 32, 32);
		a4l_mite_dma_arm(devpriv->cdo_mite_chan);
	} else
		err = -EIO;

	rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock, flags);

	return err;
}

#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

static void ni_ai_reset(struct a4l_subdevice *subd)
{
	struct a4l_device *dev = subd->dev;

	ni_release_ai_mite_channel(dev);

	/* ai configuration */
	devpriv->stc_writew(dev, AI_Configuration_Start | AI_Reset,
			    Joint_Reset_Register);

	ni_set_bits(dev, Interrupt_A_Enable_Register,
		    AI_SC_TC_Interrupt_Enable | AI_START1_Interrupt_Enable |
		    AI_START2_Interrupt_Enable | AI_START_Interrupt_Enable |
		    AI_STOP_Interrupt_Enable | AI_Error_Interrupt_Enable |
		    AI_FIFO_Interrupt_Enable, 0);

	ni_clear_ai_fifo(dev);

	if (boardtype.reg_type != ni_reg_6143)
		ni_writeb(0, Misc_Command);

	devpriv->stc_writew(dev, AI_Disarm, AI_Command_1_Register);	/* reset pulses */
	devpriv->stc_writew(dev,
			    AI_Start_Stop | AI_Mode_1_Reserved /*| AI_Trigger_Once */ ,
			    AI_Mode_1_Register);
	devpriv->stc_writew(dev, 0x0000, AI_Mode_2_Register);
	/* generate FIFO interrupts on non-empty */
	devpriv->stc_writew(dev, (0 << 6) | 0x0000, AI_Mode_3_Register);
	if (boardtype.reg_type == ni_reg_611x) {
		devpriv->stc_writew(dev, AI_SHIFTIN_Pulse_Width |
				    AI_SOC_Polarity |
				    AI_LOCALMUX_CLK_Pulse_Width, AI_Personal_Register);
		devpriv->stc_writew(dev, AI_SCAN_IN_PROG_Output_Select(3) |
				    AI_EXTMUX_CLK_Output_Select(0) |
				    AI_LOCALMUX_CLK_Output_Select(2) |
				    AI_SC_TC_Output_Select(3) |
				    AI_CONVERT_Output_Select(AI_CONVERT_Output_Enable_High),
				    AI_Output_Control_Register);
	} else if (boardtype.reg_type == ni_reg_6143) {
		devpriv->stc_writew(dev, AI_SHIFTIN_Pulse_Width |
				    AI_SOC_Polarity |
				    AI_LOCALMUX_CLK_Pulse_Width, AI_Personal_Register);
		devpriv->stc_writew(dev, AI_SCAN_IN_PROG_Output_Select(3) |
				    AI_EXTMUX_CLK_Output_Select(0) |
				    AI_LOCALMUX_CLK_Output_Select(2) |
				    AI_SC_TC_Output_Select(3) |
				    AI_CONVERT_Output_Select(AI_CONVERT_Output_Enable_Low),
				    AI_Output_Control_Register);
	} else {
		unsigned int ai_output_control_bits;
		devpriv->stc_writew(dev, AI_SHIFTIN_Pulse_Width |
				    AI_SOC_Polarity |
				    AI_CONVERT_Pulse_Width |
				    AI_LOCALMUX_CLK_Pulse_Width, AI_Personal_Register);
		ai_output_control_bits = AI_SCAN_IN_PROG_Output_Select(3) |
			AI_EXTMUX_CLK_Output_Select(0) |
			AI_LOCALMUX_CLK_Output_Select(2) |
			AI_SC_TC_Output_Select(3);
		if (boardtype.reg_type == ni_reg_622x)
			ai_output_control_bits |=
				AI_CONVERT_Output_Select
				(AI_CONVERT_Output_Enable_High);
		else
			ai_output_control_bits |=
				AI_CONVERT_Output_Select
				(AI_CONVERT_Output_Enable_Low);
		devpriv->stc_writew(dev, ai_output_control_bits,
				    AI_Output_Control_Register);
	}

	/* the following registers should not be changed, because there
	 * are no backup registers in devpriv.  If you want to change
	 * any of these, add a backup register and other appropriate code:
	 *      AI_Mode_1_Register
	 *      AI_Mode_3_Register
	 *      AI_Personal_Register
	 *      AI_Output_Control_Register
	 */

	/* clear interrupts */
	devpriv->stc_writew(dev, AI_SC_TC_Error_Confirm | AI_START_Interrupt_Ack |
			    AI_START2_Interrupt_Ack | AI_START1_Interrupt_Ack |
			    AI_SC_TC_Interrupt_Ack | AI_Error_Interrupt_Ack |
			    AI_STOP_Interrupt_Ack, Interrupt_A_Ack_Register);

	devpriv->stc_writew(dev, AI_Configuration_End, Joint_Reset_Register);
}

static int ni_ai_insn_read(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	const unsigned int mask = (1 << boardtype.adbits) - 1;
	int i, n;
	unsigned int signbits;
	unsigned short d;
	unsigned long dl;
	uint16_t *data = (uint16_t *)insn->data;

	ni_load_channelgain_list(dev, 1, &insn->chan_desc);

	ni_clear_ai_fifo(dev);

	signbits = devpriv->ai_offset[0];
	if (boardtype.reg_type == ni_reg_611x) {
		for (n = 0; n < num_adc_stages_611x; n++) {
			devpriv->stc_writew(dev, AI_CONVERT_Pulse,
					    AI_Command_1_Register);
			a4l_udelay(1);
		}
		for (n = 0; n < insn->data_size / sizeof(uint16_t); n++) {
			devpriv->stc_writew(dev, AI_CONVERT_Pulse,
					    AI_Command_1_Register);
			/* The 611x has screwy 32-bit FIFOs. */
			d = 0;
			for (i = 0; i < NI_TIMEOUT; i++) {
				if (ni_readb(XXX_Status) & 0x80) {
					d = (ni_readl(ADC_FIFO_Data_611x) >> 16)
						& 0xffff;
					break;
				}
				if (!(devpriv->stc_readw(dev,
							 AI_Status_1_Register) &
				      AI_FIFO_Empty_St)) {
					d = ni_readl(ADC_FIFO_Data_611x) &
						0xffff;
					break;
				}
			}
			if (i == NI_TIMEOUT) {
				a4l_warn(dev,
					 "ni_mio_common: "
					 "timeout in 611x ni_ai_insn_read\n");
				return -ETIME;
			}
			d += signbits;
			data[n] = d;
		}
	} else if (boardtype.reg_type == ni_reg_6143) {
		for (n = 0; n < insn->data_size / sizeof(uint16_t); n++) {
			devpriv->stc_writew(dev, AI_CONVERT_Pulse,
					    AI_Command_1_Register);

			/* The 6143 has 32-bit FIFOs.
			   You need to strobe a bit to move a single
			   16bit stranded sample into the FIFO */
			dl = 0;
			for (i = 0; i < NI_TIMEOUT; i++) {
				if (ni_readl(AIFIFO_Status_6143) & 0x01) {
					ni_writel(0x01, AIFIFO_Control_6143);	// Get stranded sample into FIFO
					dl = ni_readl(AIFIFO_Data_6143);
					break;
				}
			}
			if (i == NI_TIMEOUT) {
				a4l_warn(dev,
					 "ni_mio_common: "
					 "timeout in 6143 ni_ai_insn_read\n");
				return -ETIME;
			}
			data[n] = (((dl >> 16) & 0xFFFF) + signbits) & 0xFFFF;
		}
	} else {
		for (n = 0; n < insn->data_size / sizeof(uint16_t); n++) {
			devpriv->stc_writew(dev, AI_CONVERT_Pulse,
					    AI_Command_1_Register);
			for (i = 0; i < NI_TIMEOUT; i++) {
				if (!(devpriv->stc_readw(dev,
							 AI_Status_1_Register) &
				      AI_FIFO_Empty_St))
					break;
			}
			if (i == NI_TIMEOUT) {
				a4l_warn(dev,
					 "ni_mio_common: "
					 "timeout in ni_ai_insn_read\n");
				return -ETIME;
			}
			if (boardtype.reg_type & ni_reg_m_series_mask) {
				data[n] = ni_readl(M_Offset_AI_FIFO_Data) & mask;
			} else {
				d = ni_readw(ADC_FIFO_Data_Register);
				/* subtle: needs to be short addition */
				d += signbits;
				data[n] = d;
			}
		}
	}
	return 0;
}

void ni_prime_channelgain_list(struct a4l_device *dev)
{
	int i;
	devpriv->stc_writew(dev, AI_CONVERT_Pulse, AI_Command_1_Register);
	for (i = 0; i < NI_TIMEOUT; ++i) {
		if (!(devpriv->stc_readw(dev,
					 AI_Status_1_Register) &
		      AI_FIFO_Empty_St)) {
			devpriv->stc_writew(dev, 1, ADC_FIFO_Clear);
			return;
		}
		a4l_udelay(1);
	}
	a4l_warn(dev, "ni_mio_common: timeout loading channel/gain list\n");
}

static void ni_m_series_load_channelgain_list(struct a4l_device *dev,
					      unsigned int n_chan,
					      unsigned int *list)
{
	unsigned int chan, range, aref;
	unsigned int i;
	unsigned offset;
	unsigned int dither;
	unsigned range_code;

	devpriv->stc_writew(dev, 1, Configuration_Memory_Clear);

	if ((list[0] & CR_ALT_SOURCE)) {
		unsigned bypass_bits;
		chan = CR_CHAN(list[0]);
		range = CR_RNG(list[0]);
		range_code = ni_gainlkup[boardtype.gainlkup][range];
		dither = ((list[0] & CR_ALT_FILTER) != 0);
		bypass_bits = MSeries_AI_Bypass_Config_FIFO_Bit;
		bypass_bits |= chan;
		bypass_bits |=
			(devpriv->
			 ai_calib_source) & (MSeries_AI_Bypass_Cal_Sel_Pos_Mask |
					     MSeries_AI_Bypass_Cal_Sel_Neg_Mask |
					     MSeries_AI_Bypass_Mode_Mux_Mask |
					     MSeries_AO_Bypass_AO_Cal_Sel_Mask);
		bypass_bits |= MSeries_AI_Bypass_Gain_Bits(range_code);
		if (dither)
			bypass_bits |= MSeries_AI_Bypass_Dither_Bit;
		// don't use 2's complement encoding
		bypass_bits |= MSeries_AI_Bypass_Polarity_Bit;
		ni_writel(bypass_bits, M_Offset_AI_Config_FIFO_Bypass);
	} else {
		ni_writel(0, M_Offset_AI_Config_FIFO_Bypass);
	}
	offset = 0;
	for (i = 0; i < n_chan; i++) {
		unsigned config_bits = 0;
		chan = CR_CHAN(list[i]);
		aref = CR_AREF(list[i]);
		range = CR_RNG(list[i]);
		dither = ((list[i] & CR_ALT_FILTER) != 0);

		range_code = ni_gainlkup[boardtype.gainlkup][range];
		devpriv->ai_offset[i] = offset;
		switch (aref) {
		case AREF_DIFF:
			config_bits |=
				MSeries_AI_Config_Channel_Type_Differential_Bits;
			break;
		case AREF_COMMON:
			config_bits |=
				MSeries_AI_Config_Channel_Type_Common_Ref_Bits;
			break;
		case AREF_GROUND:
			config_bits |=
				MSeries_AI_Config_Channel_Type_Ground_Ref_Bits;
			break;
		case AREF_OTHER:
			break;
		}
		config_bits |= MSeries_AI_Config_Channel_Bits(chan);
		config_bits |=
			MSeries_AI_Config_Bank_Bits(boardtype.reg_type, chan);
		config_bits |= MSeries_AI_Config_Gain_Bits(range_code);
		if (i == n_chan - 1)
			config_bits |= MSeries_AI_Config_Last_Channel_Bit;
		if (dither)
			config_bits |= MSeries_AI_Config_Dither_Bit;
		// don't use 2's complement encoding
		config_bits |= MSeries_AI_Config_Polarity_Bit;
		ni_writew(config_bits, M_Offset_AI_Config_FIFO_Data);
	}
	ni_prime_channelgain_list(dev);
}

/*
 * Notes on the 6110 and 6111:
 * These boards a slightly different than the rest of the series, since
 * they have multiple A/D converters.
 * From the driver side, the configuration memory is a
 * little different.
 * Configuration Memory Low:
 *   bits 15-9: same
 *   bit 8: unipolar/bipolar (should be 0 for bipolar)
 *   bits 0-3: gain.  This is 4 bits instead of 3 for the other boards
 *       1001 gain=0.1 (+/- 50)
 *       1010 0.2
 *       1011 0.1
 *       0001 1
 *       0010 2
 *       0011 5
 *       0100 10
 *       0101 20
 *       0110 50
 * Configuration Memory High:
 *   bits 12-14: Channel Type
 *       001 for differential
 *       000 for calibration
 *   bit 11: coupling  (this is not currently handled)
 *       1 AC coupling
 *       0 DC coupling
 *   bits 0-2: channel
 *       valid channels are 0-3
 */
static void ni_load_channelgain_list(struct a4l_device *dev,
				     unsigned int n_chan, unsigned int *list)
{
	unsigned int chan, range, aref;
	unsigned int i;
	unsigned int hi, lo;
	unsigned offset;
	unsigned int dither;

	if (boardtype.reg_type & ni_reg_m_series_mask) {
		ni_m_series_load_channelgain_list(dev, n_chan, list);
		return;
	}
	if (n_chan == 1 && (boardtype.reg_type != ni_reg_611x)
	    && (boardtype.reg_type != ni_reg_6143)) {
		if (devpriv->changain_state
		    && devpriv->changain_spec == list[0]) {
			/* ready to go. */
			return;
		}
		devpriv->changain_state = 1;
		devpriv->changain_spec = list[0];
	} else {
		devpriv->changain_state = 0;
	}

	devpriv->stc_writew(dev, 1, Configuration_Memory_Clear);

	/* Set up Calibration mode if required */
	if (boardtype.reg_type == ni_reg_6143) {
		if ((list[0] & CR_ALT_SOURCE)
		    && !devpriv->ai_calib_source_enabled) {
			/* Strobe Relay enable bit */
			ni_writew(devpriv->
				  ai_calib_source |
				  Calibration_Channel_6143_RelayOn,
				  Calibration_Channel_6143);
			ni_writew(devpriv->ai_calib_source,
				  Calibration_Channel_6143);
			devpriv->ai_calib_source_enabled = 1;
			/* Allow relays to change */
			if(rtdm_in_rt_context())
				rtdm_task_sleep(100*1000000);
			else
				msleep_interruptible(100);
		} else if (!(list[0] & CR_ALT_SOURCE)
			   && devpriv->ai_calib_source_enabled) {
			/* Strobe Relay disable bit */
			ni_writew(devpriv->
				  ai_calib_source |
				  Calibration_Channel_6143_RelayOff,
				  Calibration_Channel_6143);
			ni_writew(devpriv->ai_calib_source,
				  Calibration_Channel_6143);
			devpriv->ai_calib_source_enabled = 0;
			/* Allow relays to change */
			if(rtdm_in_rt_context())
				rtdm_task_sleep(100*1000000);
			else
				msleep_interruptible(100);
		}
	}

	offset = 1 << (boardtype.adbits - 1);
	for (i = 0; i < n_chan; i++) {
		if ((boardtype.reg_type != ni_reg_6143)
		    && (list[i] & CR_ALT_SOURCE)) {
			chan = devpriv->ai_calib_source;
		} else {
			chan = CR_CHAN(list[i]);
		}
		aref = CR_AREF(list[i]);
		range = CR_RNG(list[i]);
		dither = ((list[i] & CR_ALT_FILTER) != 0);

		/* fix the external/internal range differences */
		range = ni_gainlkup[boardtype.gainlkup][range];
		if (boardtype.reg_type == ni_reg_611x)
			devpriv->ai_offset[i] = offset;
		else
			devpriv->ai_offset[i] = (range & 0x100) ? 0 : offset;

		hi = 0;
		if ((list[i] & CR_ALT_SOURCE)) {
			if (boardtype.reg_type == ni_reg_611x)
				ni_writew(CR_CHAN(list[i]) & 0x0003,
					  Calibration_Channel_Select_611x);
		} else {
			if (boardtype.reg_type == ni_reg_611x)
				aref = AREF_DIFF;
			else if (boardtype.reg_type == ni_reg_6143)
				aref = AREF_OTHER;
			switch (aref) {
			case AREF_DIFF:
				hi |= AI_DIFFERENTIAL;
				break;
			case AREF_COMMON:
				hi |= AI_COMMON;
				break;
			case AREF_GROUND:
				hi |= AI_GROUND;
				break;
			case AREF_OTHER:
				break;
			}
		}
		hi |= AI_CONFIG_CHANNEL(chan);

		ni_writew(hi, Configuration_Memory_High);

		if (boardtype.reg_type != ni_reg_6143) {
			lo = range;
			if (i == n_chan - 1)
				lo |= AI_LAST_CHANNEL;
			if (dither)
				lo |= AI_DITHER;

			ni_writew(lo, Configuration_Memory_Low);
		}
	}

	/* prime the channel/gain list */
	if ((boardtype.reg_type != ni_reg_611x)
	    && (boardtype.reg_type != ni_reg_6143)) {
		ni_prime_channelgain_list(dev);
	}
}

static int ni_ns_to_timer(const struct a4l_device *dev,
			  unsigned int nanosec, int round_mode)
{
	int divider;
	switch (round_mode) {
	case TRIG_ROUND_NEAREST:
	default:
		divider = (nanosec + devpriv->clock_ns / 2) / devpriv->clock_ns;
		break;
	case TRIG_ROUND_DOWN:
		divider = (nanosec) / devpriv->clock_ns;
		break;
	case TRIG_ROUND_UP:
		divider = (nanosec + devpriv->clock_ns - 1) / devpriv->clock_ns;
		break;
	}
	return divider - 1;
}

static unsigned int ni_timer_to_ns(const struct a4l_device *dev, int timer)
{
	return devpriv->clock_ns * (timer + 1);
}

static unsigned int ni_min_ai_scan_period_ns(struct a4l_device *dev,
					     unsigned int num_channels)
{
	switch (boardtype.reg_type) {
	case ni_reg_611x:
	case ni_reg_6143:
		/* simultaneously-sampled inputs */
		return boardtype.ai_speed;
		break;
	default:
		/* multiplexed inputs */
		break;
	};
	return boardtype.ai_speed * num_channels;
}

static struct a4l_cmd_desc mio_ai_cmd_mask = {
	.idx_subd = 0,
	.start_src = TRIG_NOW | TRIG_INT | TRIG_EXT,
	.scan_begin_src = TRIG_TIMER | TRIG_EXT,
	.convert_src = TRIG_TIMER | TRIG_EXT | TRIG_NOW,
	.scan_end_src = TRIG_COUNT,
	.stop_src = TRIG_COUNT | TRIG_NONE,
};

int ni_ai_inttrig(struct a4l_subdevice *subd, lsampl_t trignum)
{
	struct a4l_device *dev = subd->dev;

	if (trignum != 0)
		return -EINVAL;

	devpriv->stc_writew(dev, AI_START1_Pulse | devpriv->ai_cmd2,
			    AI_Command_2_Register);

	return 1;
}

#define cfc_check_trigger_arg_is(a,b) __cfc_check_trigger_arg_is(a,b, dev, __LINE__)
static inline int __cfc_check_trigger_arg_is(unsigned int *arg,
	                                     unsigned int val,
					     struct a4l_device *dev,
	                                     unsigned int line)
{
	if (*arg != val) {
		a4l_dbg(1, drv_dbg, dev, "line %d: *arg (%d) != val (%d) \n",
			line, *arg, val);
		*arg = val;
		return -EINVAL;
	}
	return 0;
}

#define cfc_check_trigger_is_unique(a) __cfc_check_trigger_is_unique(a, dev, __LINE__)
static inline int __cfc_check_trigger_is_unique(unsigned int src,
					        struct a4l_device *dev,
	                                        unsigned int line)
{
	/* this test is true if more than one _src bit is set */
	if ((src & (src - 1)) != 0) {
		a4l_dbg(1, drv_dbg, dev, "line %d: src (%d) \n", line, src);
		return -EINVAL;
	}
	return 0;
}

#define cfc_check_trigger_src(a,b) __cfc_check_trigger_src(a,b, dev, __LINE__)
static inline int __cfc_check_trigger_src(unsigned int *src,
	                                  unsigned int flags,
					  struct a4l_device *dev,
	                                  unsigned int line)
{
	unsigned int orig_src = *src;

	*src = orig_src & flags;
	if (*src == 0 || *src != orig_src){
		a4l_dbg(1, drv_dbg, dev, "line %d: *src (%d)  orig_src (%d) flags(%d) \n",
			line, *src, orig_src, flags);
		return -EINVAL;
	}

	return 0;
}

#define cfc_check_trigger_arg_min(a,b) __cfc_check_trigger_arg_min(a,b, dev, __LINE__)
static inline int __cfc_check_trigger_arg_min(unsigned int *arg,
					      unsigned int val,
					      struct a4l_device *dev,
	                                      unsigned int line)
{
	if (*arg < val) {
		a4l_dbg(1, drv_dbg, dev, "line %d: *arg (%d) < val (%d) \n",
			line, *arg, val);
		*arg = val;
		return -EINVAL;
	}
	return 0;
}

#define cfc_check_trigger_arg_max(a,b) __cfc_check_trigger_arg_max(a,b, dev, __LINE__)
static inline int __cfc_check_trigger_arg_max(unsigned int *arg,
					      unsigned int val,
					      struct a4l_device *dev,
	                                      unsigned int line)
{
	if (*arg > val) {
		a4l_dbg(1, drv_dbg, dev, "line %d: *arg (%d) > val (%d) \n",
			line, *arg, val);
		*arg = val;
		return -EINVAL;
	}
	return 0;
}

static int ni_ai_cmdtest(struct a4l_subdevice *subd, struct a4l_cmd_desc *cmd)
{
	struct a4l_device *dev = subd->dev;
	unsigned int sources;
	int tmp, err = 0;

	/* Step 1 : check if triggers are trivially valid */
	err |= cfc_check_trigger_src(&cmd->start_src, TRIG_NOW | TRIG_INT | TRIG_EXT);
	err |= cfc_check_trigger_src(&cmd->scan_begin_src, TRIG_TIMER | TRIG_EXT);

	sources = TRIG_TIMER | TRIG_EXT;
	if (boardtype.reg_type == ni_reg_611x || boardtype.reg_type == ni_reg_6143)
		sources |= TRIG_NOW;

	err |= cfc_check_trigger_src(&cmd->convert_src, sources);
	err |= cfc_check_trigger_src(&cmd->scan_end_src, TRIG_COUNT);
	err |= cfc_check_trigger_src(&cmd->stop_src, TRIG_COUNT | TRIG_NONE);

	if (err) {
		if (cmd->valid_simul_stages & BIT(1))
			return 0;

		a4l_dbg(1, drv_dbg, dev, "ai_cmdtest ERR 1 \n");
		return -EINVAL;
	}

	/* Step 2a : make sure trigger sources are unique */
	err |= cfc_check_trigger_is_unique(cmd->start_src);
	err |= cfc_check_trigger_is_unique(cmd->scan_begin_src);
	err |= cfc_check_trigger_is_unique(cmd->convert_src);
	err |= cfc_check_trigger_is_unique(cmd->stop_src);

	/* Step 2b : and mutually compatible */

	if (err) {
		if (cmd->valid_simul_stages & BIT(2))
			return 0;

		a4l_dbg(1, drv_dbg, dev, "ai_cmdtest ERR 2 \n");
		return -EINVAL;
	}

	/* Step 3: check if arguments are trivially valid */

	if (cmd->start_src == TRIG_EXT) {
		/* external trigger */
		unsigned int tmp = CR_CHAN(cmd->start_arg);
		if (tmp > 16)
			tmp = 16;
		tmp |= (cmd->start_arg & (CR_INVERT | CR_EDGE));
		err |= cfc_check_trigger_arg_is(&cmd->start_arg, tmp);

	} else {
		/* true for both TRIG_NOW and TRIG_INT */
		err |= cfc_check_trigger_arg_is(&cmd->start_arg, 0);
	}

	if (cmd->scan_begin_src == TRIG_TIMER) {
		err |= cfc_check_trigger_arg_min(&cmd->scan_begin_arg,
			ni_min_ai_scan_period_ns(dev, cmd->nb_chan));

		err |= cfc_check_trigger_arg_max(&cmd->scan_begin_arg,
						 devpriv->clock_ns * 0xffffff);
	} else if (cmd->scan_begin_src == TRIG_EXT) {
		/* external trigger */
		unsigned int tmp = CR_CHAN(cmd->scan_begin_arg);

		if (tmp > 16)
			tmp = 16;
		tmp |= (cmd->scan_begin_arg & (CR_INVERT | CR_EDGE));
		err |= cfc_check_trigger_arg_is(&cmd->scan_begin_arg, tmp);

	} else {		/* TRIG_OTHER */
		err |= cfc_check_trigger_arg_is(&cmd->scan_begin_arg, 0);

	}

	if (cmd->convert_src == TRIG_TIMER) {
		if ((boardtype.reg_type == ni_reg_611x)
		    || (boardtype.reg_type == ni_reg_6143)) {
			err |= cfc_check_trigger_arg_is(&cmd->convert_arg, 0);

		} else {
			err |= cfc_check_trigger_arg_min(&cmd->convert_arg,
							 boardtype.ai_speed);
			err |= cfc_check_trigger_arg_max(&cmd->convert_arg,
						devpriv->clock_ns * 0xffff);
		}
	} else if (cmd->convert_src == TRIG_EXT) {
		/* external trigger */
		unsigned int tmp = CR_CHAN(cmd->convert_arg);

		if (tmp > 16)
			tmp = 16;
		tmp |= (cmd->convert_arg & (CR_ALT_FILTER | CR_INVERT));
		err |= cfc_check_trigger_arg_is(&cmd->convert_arg, tmp);
	} else if (cmd->convert_src == TRIG_NOW) {
		err |= cfc_check_trigger_arg_is(&cmd->convert_arg, 0);
	}

	err |= cfc_check_trigger_arg_is(&cmd->scan_end_arg, cmd->nb_chan);

	if (cmd->stop_src == TRIG_COUNT) {
		unsigned int max_count = 0x01000000;

		if (boardtype.reg_type == ni_reg_611x)
			max_count -= num_adc_stages_611x;
		err |= cfc_check_trigger_arg_max(&cmd->stop_arg, max_count);
		err |= cfc_check_trigger_arg_min(&cmd->stop_arg, 1);

	} else {
		/* TRIG_NONE */
		err |= cfc_check_trigger_arg_is(&cmd->stop_arg, 0);
	}

	if (err) {
		if (cmd->valid_simul_stages & BIT(3))
			return 0;

		a4l_dbg(1, drv_dbg, dev, "ai_cmdtest ERR 3 \n");
		return 3;
	}

	/* step 4: fix up any arguments */
	if (cmd->scan_begin_src == TRIG_TIMER) {
		tmp = cmd->scan_begin_arg;
		cmd->scan_begin_arg =
		    ni_timer_to_ns(dev, ni_ns_to_timer(dev,
						       cmd->scan_begin_arg,
						       cmd->flags &
						       TRIG_ROUND_MASK));
		if (tmp != cmd->scan_begin_arg)
			err++;
	}
	if (cmd->convert_src == TRIG_TIMER) {
		if ((boardtype.reg_type != ni_reg_611x)
		    && (boardtype.reg_type != ni_reg_6143)) {
			tmp = cmd->convert_arg;
			cmd->convert_arg =
			    ni_timer_to_ns(dev, ni_ns_to_timer(dev,
							       cmd->convert_arg,
							       cmd->
							       flags &
							       TRIG_ROUND_MASK));
			if (tmp != cmd->convert_arg)
				err++;
			if (cmd->scan_begin_src == TRIG_TIMER &&
			    cmd->scan_begin_arg <
			    cmd->convert_arg * cmd->scan_end_arg) {
				cmd->scan_begin_arg =
				    cmd->convert_arg * cmd->scan_end_arg;
				err++;
			}
		}
	}

	if (err) {
		if (cmd->valid_simul_stages & BIT(4))
			return 0;

		a4l_dbg(1, drv_dbg, dev, "ai_cmdtest ERR 4 \n");
		return -EINVAL;
	}

	return 0;


}

static int ni_ai_cmd(struct a4l_subdevice *subd, struct a4l_cmd_desc *cmd)
{
	struct a4l_device *dev = subd->dev;
	int timer;
	int mode1 = 0;		/* mode1 is needed for both stop and convert */
	int mode2 = 0;
	int start_stop_select = 0;
	unsigned int stop_count;
	int interrupt_a_enable = 0;

	a4l_info(dev, "start\n");

	if (a4l_get_irq(dev) == A4L_IRQ_UNUSED) {
		a4l_err(dev, "ni_ai_cmd: cannot run command without an irq");
		return -EIO;
	}
	ni_clear_ai_fifo(dev);

	ni_load_channelgain_list(dev, cmd->nb_chan, cmd->chan_descs);

	/* start configuration */
	devpriv->stc_writew(dev, AI_Configuration_Start, Joint_Reset_Register);

	/* disable analog triggering for now, since it
	 * interferes with the use of pfi0 */
	devpriv->an_trig_etc_reg &= ~Analog_Trigger_Enable;
	devpriv->stc_writew(dev, devpriv->an_trig_etc_reg,
			    Analog_Trigger_Etc_Register);

	switch (cmd->start_src) {
	case TRIG_INT:
	case TRIG_NOW:
		devpriv->stc_writew(dev, AI_START2_Select(0) |
				    AI_START1_Sync | AI_START1_Edge | AI_START1_Select(0),
				    AI_Trigger_Select_Register);
		break;
	case TRIG_EXT:
	{
		int chan = CR_CHAN(cmd->start_arg);
		unsigned int bits = AI_START2_Select(0) |
			AI_START1_Sync | AI_START1_Select(chan + 1);

		if (cmd->start_arg & CR_INVERT)
			bits |= AI_START1_Polarity;
		if (cmd->start_arg & CR_EDGE)
			bits |= AI_START1_Edge;
		devpriv->stc_writew(dev, bits,
				    AI_Trigger_Select_Register);
		break;
	}
	}

	mode2 &= ~AI_Pre_Trigger;
	mode2 &= ~AI_SC_Initial_Load_Source;
	mode2 &= ~AI_SC_Reload_Mode;
	devpriv->stc_writew(dev, mode2, AI_Mode_2_Register);

	if (cmd->nb_chan == 1 || (boardtype.reg_type == ni_reg_611x)
	    || (boardtype.reg_type == ni_reg_6143)) {
		start_stop_select |= AI_STOP_Polarity;
		start_stop_select |= AI_STOP_Select(31);/* logic low */
		start_stop_select |= AI_STOP_Sync;
	} else {
		start_stop_select |= AI_STOP_Select(19);/* ai configuration memory */
	}
	devpriv->stc_writew(dev, start_stop_select,
			    AI_START_STOP_Select_Register);

	devpriv->ai_cmd2 = 0;
	switch (cmd->stop_src) {
	case TRIG_COUNT:
		stop_count = cmd->stop_arg - 1;

		if (boardtype.reg_type == ni_reg_611x) {
			/* have to take 3 stage adc pipeline into account */
			stop_count += num_adc_stages_611x;
		}
		/* stage number of scans */
		devpriv->stc_writel(dev, stop_count, AI_SC_Load_A_Registers);

		mode1 |= AI_Start_Stop | AI_Mode_1_Reserved | AI_Trigger_Once;
		devpriv->stc_writew(dev, mode1, AI_Mode_1_Register);
		/* load SC (Scan Count) */
		devpriv->stc_writew(dev, AI_SC_Load, AI_Command_1_Register);

		devpriv->ai_continuous = 0;
		if (stop_count == 0) {
			devpriv->ai_cmd2 |= AI_End_On_End_Of_Scan;
			interrupt_a_enable |= AI_STOP_Interrupt_Enable;
			/* this is required to get the last sample
			   for nb_chan > 1, not sure why */
			if (cmd->nb_chan > 1)
				start_stop_select |=
					AI_STOP_Polarity | AI_STOP_Edge;
		}
		break;
	case TRIG_NONE:
		/* stage number of scans */
		devpriv->stc_writel(dev, 0, AI_SC_Load_A_Registers);

		mode1 |= AI_Start_Stop | AI_Mode_1_Reserved | AI_Continuous;
		devpriv->stc_writew(dev, mode1, AI_Mode_1_Register);

		/* load SC (Scan Count) */
		devpriv->stc_writew(dev, AI_SC_Load, AI_Command_1_Register);

		devpriv->ai_continuous = 1;

		break;
	}

	switch (cmd->scan_begin_src) {
	case TRIG_TIMER:
		/*
		  stop bits for non 611x boards
		  AI_SI_Special_Trigger_Delay=0
		  AI_Pre_Trigger=0
		  AI_START_STOP_Select_Register:
		  AI_START_Polarity=0 (?)      rising edge
		  AI_START_Edge=1              edge triggered
		  AI_START_Sync=1 (?)
		  AI_START_Select=0            SI_TC
		  AI_STOP_Polarity=0           rising edge
		  AI_STOP_Edge=0               level
		  AI_STOP_Sync=1
		  AI_STOP_Select=19            external pin (configuration mem)
		*/
		start_stop_select |= AI_START_Edge | AI_START_Sync;
		devpriv->stc_writew(dev, start_stop_select,
				    AI_START_STOP_Select_Register);

		mode2 |= AI_SI_Reload_Mode(0);
		/* AI_SI_Initial_Load_Source=A */
		mode2 &= ~AI_SI_Initial_Load_Source;

		devpriv->stc_writew(dev, mode2, AI_Mode_2_Register);

		/* load SI */
		timer = ni_ns_to_timer(dev, cmd->scan_begin_arg,
				       TRIG_ROUND_NEAREST);
		devpriv->stc_writel(dev, timer, AI_SI_Load_A_Registers);
		devpriv->stc_writew(dev, AI_SI_Load, AI_Command_1_Register);
		break;
	case TRIG_EXT:
		if (cmd->scan_begin_arg & CR_EDGE)
			start_stop_select |= AI_START_Edge;
		/* AI_START_Polarity==1 is falling edge */
		if (cmd->scan_begin_arg & CR_INVERT)
			start_stop_select |= AI_START_Polarity;
		if (cmd->scan_begin_src != cmd->convert_src ||
		    (cmd->scan_begin_arg & ~CR_EDGE) !=
		    (cmd->convert_arg & ~CR_EDGE))
			start_stop_select |= AI_START_Sync;
		start_stop_select |=
			AI_START_Select(1 + CR_CHAN(cmd->scan_begin_arg));
		devpriv->stc_writew(dev, start_stop_select,
				    AI_START_STOP_Select_Register);
		break;
	}

	switch (cmd->convert_src) {
	case TRIG_TIMER:
	case TRIG_NOW:
		if (cmd->convert_arg == 0 || cmd->convert_src == TRIG_NOW)
			timer = 1;
		else
			timer = ni_ns_to_timer(dev, cmd->convert_arg,
					       TRIG_ROUND_NEAREST);
		devpriv->stc_writew(dev, 1, AI_SI2_Load_A_Register);	/* 0,0 does not work. */
		devpriv->stc_writew(dev, timer, AI_SI2_Load_B_Register);

		/* AI_SI2_Reload_Mode = alternate */
		/* AI_SI2_Initial_Load_Source = A */
		mode2 &= ~AI_SI2_Initial_Load_Source;
		mode2 |= AI_SI2_Reload_Mode;
		devpriv->stc_writew(dev, mode2, AI_Mode_2_Register);

		/* AI_SI2_Load */
		devpriv->stc_writew(dev, AI_SI2_Load, AI_Command_1_Register);

		mode2 |= AI_SI2_Reload_Mode; /* alternate */
		mode2 |= AI_SI2_Initial_Load_Source; /* B */

		devpriv->stc_writew(dev, mode2, AI_Mode_2_Register);
		break;
	case TRIG_EXT:
		mode1 |= AI_CONVERT_Source_Select(1 + cmd->convert_arg);
		if ((cmd->convert_arg & CR_INVERT) == 0)
			mode1 |= AI_CONVERT_Source_Polarity;
		devpriv->stc_writew(dev, mode1, AI_Mode_1_Register);

		mode2 |= AI_Start_Stop_Gate_Enable | AI_SC_Gate_Enable;
		devpriv->stc_writew(dev, mode2, AI_Mode_2_Register);

		break;
	}

	if (a4l_get_irq(dev) != A4L_IRQ_UNUSED) {

		/* interrupt on FIFO, errors, SC_TC */
		interrupt_a_enable |= AI_Error_Interrupt_Enable |
			AI_SC_TC_Interrupt_Enable;

#if (!defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) && \
     !defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))
		interrupt_a_enable |= AI_FIFO_Interrupt_Enable;
#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

		if (cmd->flags & TRIG_WAKE_EOS
		    || (devpriv->ai_cmd2 & AI_End_On_End_Of_Scan)) {
			/* wake on end-of-scan */
			devpriv->aimode = AIMODE_SCAN;
		} else {
			devpriv->aimode = AIMODE_HALF_FULL;
		}

		switch (devpriv->aimode) {
		case AIMODE_HALF_FULL:
			/* generate FIFO interrupts and DMA requests on half-full */
#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))
			devpriv->stc_writew(dev, AI_FIFO_Mode_HF_to_E,
					    AI_Mode_3_Register);
#else /* !CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */
			devpriv->stc_writew(dev, AI_FIFO_Mode_HF,
					    AI_Mode_3_Register);
#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */
			break;
		case AIMODE_SAMPLE:
			/* generate FIFO interrupts on non-empty */
			devpriv->stc_writew(dev, AI_FIFO_Mode_NE,
					    AI_Mode_3_Register);
			break;
		case AIMODE_SCAN:
#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))
			devpriv->stc_writew(dev, AI_FIFO_Mode_NE,
					    AI_Mode_3_Register);
#else /* !CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */
			devpriv->stc_writew(dev, AI_FIFO_Mode_HF,
					    AI_Mode_3_Register);
#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */
			interrupt_a_enable |= AI_STOP_Interrupt_Enable;
			break;
		default:
			break;
		}

		/* Clear interrupts */
		devpriv->stc_writew(dev,
				    AI_Error_Interrupt_Ack | AI_STOP_Interrupt_Ack |
				    AI_START_Interrupt_Ack | AI_START2_Interrupt_Ack |
				    AI_START1_Interrupt_Ack | AI_SC_TC_Interrupt_Ack |
				    AI_SC_TC_Error_Confirm, Interrupt_A_Ack_Register);	/* clear interrupts */

		ni_set_bits(dev, Interrupt_A_Enable_Register,
			    interrupt_a_enable, 1);

		a4l_info(dev, "Interrupt_A_Enable_Register = 0x%04x\n",
			 devpriv->int_a_enable_reg);
	} else {
		/* interrupt on nothing */
		ni_set_bits(dev, Interrupt_A_Enable_Register, ~0, 0);

		/* XXX start polling if necessary */
		a4l_warn(dev, "ni_ai_cmd: interrupting on nothing\n");
	}

	/* end configuration */
	devpriv->stc_writew(dev, AI_Configuration_End, Joint_Reset_Register);

	switch (cmd->scan_begin_src) {
	case TRIG_TIMER:
		devpriv->stc_writew(dev,
				    AI_SI2_Arm | AI_SI_Arm | AI_DIV_Arm | AI_SC_Arm,
				    AI_Command_1_Register);
		break;
	case TRIG_EXT:
		/* XXX AI_SI_Arm? */
		devpriv->stc_writew(dev,
				    AI_SI2_Arm | AI_SI_Arm | AI_DIV_Arm | AI_SC_Arm,
				    AI_Command_1_Register);
		break;
	}

#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))
	{
		int retval = ni_ai_setup_MITE_dma(subd);
		if (retval)
			return retval;
	}

#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

	switch (cmd->start_src) {
	case TRIG_NOW:
		/* AI_START1_Pulse */
		devpriv->stc_writew(dev, AI_START1_Pulse | devpriv->ai_cmd2,
				    AI_Command_2_Register);
		break;
	case TRIG_EXT:
		/* TODO: set trigger callback field to NULL */
		break;
	case TRIG_INT:
		/* TODO: set trigger callback field to ni_ai_inttrig */
		break;
	}

	a4l_info(dev, "exit\n");

	return 0;
}

int ni_ai_config_analog_trig(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	unsigned int a, b, modebits;
	int err = 0;
	uint32_t *data = (uint32_t *)insn->data;

	/* data[1] is flags
	 * data[2] is analog line
	 * data[3] is set level
	 * data[4] is reset level */
	if (!boardtype.has_analog_trig)
		return -EINVAL;

	if ((data[1] & 0xffff0000) != A4L_EV_SCAN_BEGIN) {
		data[1] &= (A4L_EV_SCAN_BEGIN | 0xffff);
		err++;
	}
	if (data[2] >= boardtype.n_adchan) {
		data[2] = boardtype.n_adchan - 1;
		err++;
	}
	if (data[3] > 255) {	/* a */
		data[3] = 255;
		err++;
	}
	if (data[4] > 255) {	/* b */
		data[4] = 255;
		err++;
	}
	/*
	 * 00 ignore
	 * 01 set
	 * 10 reset
	 *
	 * modes:
	 *   1 level:                    +b-   +a-
	 *     high mode                00 00 01 10
	 *     low mode                 00 00 10 01
	 *   2 level: (a<b)
	 *     hysteresis low mode      10 00 00 01
	 *     hysteresis high mode     01 00 00 10
	 *     middle mode              10 01 01 10
	 */

	a = data[3];
	b = data[4];
	modebits = data[1] & 0xff;
	if (modebits & 0xf0) {
		/* two level mode */
		if (b < a) {
			/* swap order */
			a = data[4];
			b = data[3];
			modebits = ((data[1] & 0xf) << 4) |
				((data[1] & 0xf0) >> 4);
		}
		devpriv->atrig_low = a;
		devpriv->atrig_high = b;
		switch (modebits) {
		case 0x81:	/* low hysteresis mode */
			devpriv->atrig_mode = 6;
			break;
		case 0x42:	/* high hysteresis mode */
			devpriv->atrig_mode = 3;
			break;
		case 0x96:	/* middle window mode */
			devpriv->atrig_mode = 2;
			break;
		default:
			data[1] &= ~0xff;
			err++;
		}
	} else {
		/* one level mode */
		if (b != 0) {
			data[4] = 0;
			err++;
		}
		switch (modebits) {
		case 0x06:	/* high window mode */
			devpriv->atrig_high = a;
			devpriv->atrig_mode = 0;
			break;
		case 0x09:	/* low window mode */
			devpriv->atrig_low = a;
			devpriv->atrig_mode = 1;
			break;
		default:
			data[1] &= ~0xff;
			err++;
		}
	}

	if (err)
		return -EAGAIN;

	return 0;
}

int ni_ai_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	unsigned int *data = (unsigned int *)insn->data;

	if (insn->data_size < sizeof(unsigned int))
		return -EINVAL;

	switch (data[0]) {
	case A4L_INSN_CONFIG_ANALOG_TRIG:
		return ni_ai_config_analog_trig(subd, insn);
	case A4L_INSN_CONFIG_ALT_SOURCE:
		if (boardtype.reg_type & ni_reg_m_series_mask) {
			if (data[1] & ~(MSeries_AI_Bypass_Cal_Sel_Pos_Mask |
					MSeries_AI_Bypass_Cal_Sel_Neg_Mask |
					MSeries_AI_Bypass_Mode_Mux_Mask |
					MSeries_AO_Bypass_AO_Cal_Sel_Mask)) {
				return -EINVAL;
			}
			devpriv->ai_calib_source = data[1];
		} else if (boardtype.reg_type == ni_reg_6143) {
			unsigned int calib_source;

			calib_source = data[1] & 0xf;

			if (calib_source > 0xF)
				return -EINVAL;

			devpriv->ai_calib_source = calib_source;
			ni_writew(calib_source, Calibration_Channel_6143);
		} else {
			unsigned int calib_source;
			unsigned int calib_source_adjust;

			calib_source = data[1] & 0xf;
			calib_source_adjust = (data[1] >> 4) & 0xff;

			if (calib_source >= 8)
				return -EINVAL;
			devpriv->ai_calib_source = calib_source;
			if (boardtype.reg_type == ni_reg_611x) {
				ni_writeb(calib_source_adjust,
					  Cal_Gain_Select_611x);
			}
		}
		return 0;
	default:
		break;
	}

	return -EINVAL;
}

/* munge data from unsigned to 2's complement for analog output bipolar modes */
static void ni_ao_munge(struct a4l_subdevice *subd, void *buf, unsigned long size)
{
	struct a4l_device *dev = subd->dev;
	struct a4l_cmd_desc *cmd = a4l_get_cmd(subd);
	int chan_idx = a4l_get_chan(subd);
	uint16_t *array = buf;
	unsigned int i, range, offset;

	offset = 1 << (boardtype.aobits - 1);
	for (i = 0; i < size / sizeof(uint16_t); i++) {

		range = CR_RNG(cmd->chan_descs[chan_idx]);
		if (boardtype.ao_unipolar == 0 || (range & 1) == 0)
			array[i] -= offset;

#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))
		array[i] = cpu_to_le16(array[i]);
#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

		chan_idx++;
		chan_idx %= cmd->nb_chan;
	}
}

static int ni_m_series_ao_config_chan_descs(struct a4l_subdevice *subd,
					    unsigned int chanspec[],
					    unsigned int n_chans, int timed)
{
	unsigned int range;
	unsigned int chan;
	unsigned int conf;
	int i, invert = 0;
	struct a4l_device *dev = subd->dev;

	for (i = 0; i < boardtype.n_aochan; ++i) {
		ni_writeb(0xf, M_Offset_AO_Waveform_Order(i));
	}
	for (i = 0; i < n_chans; i++) {
		struct a4l_range *rng;
		int idx;
		chan = CR_CHAN(chanspec[i]);
		range = CR_RNG(chanspec[i]);

		/* TODO: this a huge hack!
		   Something is missing in the kernel API. We must
		   allow access on the proper range descriptor */
		idx =  (subd->rng_desc->mode !=
			A4L_RNG_GLOBAL_RNGDESC) ? chan : 0;
		rng = &(subd->rng_desc->rngtabs[idx]->rngs[range]);

		invert = 0;
		conf = 0;
		switch (rng->max - rng->min) {
		case 20000000:
			conf |= MSeries_AO_DAC_Reference_10V_Internal_Bits;
			ni_writeb(0, M_Offset_AO_Reference_Attenuation(chan));
			break;
		case 10000000:
			conf |= MSeries_AO_DAC_Reference_5V_Internal_Bits;
			ni_writeb(0, M_Offset_AO_Reference_Attenuation(chan));
			break;
		case 4000000:
			conf |= MSeries_AO_DAC_Reference_10V_Internal_Bits;
			ni_writeb(MSeries_Attenuate_x5_Bit,
				  M_Offset_AO_Reference_Attenuation(chan));
			break;
		case 2000000:
			conf |= MSeries_AO_DAC_Reference_5V_Internal_Bits;
			ni_writeb(MSeries_Attenuate_x5_Bit,
				  M_Offset_AO_Reference_Attenuation(chan));
			break;
		default:
			a4l_err(subd->dev,
				"%s: bug! unhandled ao reference voltage\n",
				__FUNCTION__);
			break;
		}
		switch (rng->max + rng->min) {
		case 0:
			conf |= MSeries_AO_DAC_Offset_0V_Bits;
			break;
		case 10000000:
			conf |= MSeries_AO_DAC_Offset_5V_Bits;
			break;
		default:
			a4l_err(subd->dev,
				"%s: bug! unhandled ao offset voltage\n",
				__FUNCTION__);
			break;
		}
		if (timed)
			conf |= MSeries_AO_Update_Timed_Bit;
		ni_writeb(conf, M_Offset_AO_Config_Bank(chan));
		devpriv->ao_conf[chan] = conf;
		ni_writeb(i, M_Offset_AO_Waveform_Order(chan));
	}
	return invert;
}

static int ni_old_ao_config_chan_descs(struct a4l_subdevice *subd,
				       unsigned int chanspec[],
				       unsigned int n_chans)
{
	struct a4l_device *dev = subd->dev;
	unsigned int range;
	unsigned int chan;
	unsigned int conf;
	int i, invert = 0;

	for (i = 0; i < n_chans; i++) {
		chan = CR_CHAN(chanspec[i]);
		range = CR_RNG(chanspec[i]);
		conf = AO_Channel(chan);

		if (boardtype.ao_unipolar) {
			if ((range & 1) == 0) {
				conf |= AO_Bipolar;
				invert = (1 << (boardtype.aobits - 1));
			} else {
				invert = 0;
			}
			if (range & 2)
				conf |= AO_Ext_Ref;
		} else {
			conf |= AO_Bipolar;
			invert = (1 << (boardtype.aobits - 1));
		}

		/* not all boards can deglitch, but this shouldn't hurt */
		if (chanspec[i] & CR_DEGLITCH)
			conf |= AO_Deglitch;

		/* analog reference */
		/* AREF_OTHER connects AO ground to AI ground, i think */
		conf |= (CR_AREF(chanspec[i]) ==
			 AREF_OTHER) ? AO_Ground_Ref : 0;

		ni_writew(conf, AO_Configuration);
		devpriv->ao_conf[chan] = conf;
	}
	return invert;
}

static int ni_ao_config_chan_descs(struct a4l_subdevice *subd,
				   unsigned int chanspec[],
				   unsigned int n_chans, int timed)
{
	struct a4l_device *dev = subd->dev;

	if (boardtype.reg_type & ni_reg_m_series_mask)
		return ni_m_series_ao_config_chan_descs(subd,
							chanspec,
							n_chans, timed);
	else
		return ni_old_ao_config_chan_descs(subd, chanspec, n_chans);
}

int ni_ao_insn_read(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint16_t *data = (uint16_t *)insn->data;

	data[0] = devpriv->ao[CR_CHAN(insn->chan_desc)];

	return 0;
}

int ni_ao_insn_write(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	unsigned int chan = CR_CHAN(insn->chan_desc);
	uint16_t *data = (uint16_t *)insn->data;
	unsigned int invert;

	invert = ni_ao_config_chan_descs(subd,
					 &insn->chan_desc, 1, 0);

	devpriv->ao[chan] = data[0];

	if (boardtype.reg_type & ni_reg_m_series_mask) {
		ni_writew(data[0], M_Offset_DAC_Direct_Data(chan));
	} else
		ni_writew(data[0] ^ invert,
			  (chan) ? DAC1_Direct_Data : DAC0_Direct_Data);

	return 0;
}

int ni_ao_insn_write_671x(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	unsigned int chan = CR_CHAN(insn->chan_desc);
	uint16_t *data = (uint16_t *)insn->data;
	unsigned int invert;

	ao_win_out(1 << chan, AO_Immediate_671x);
	invert = 1 << (boardtype.aobits - 1);

	ni_ao_config_chan_descs(subd, &insn->chan_desc, 1, 0);

	devpriv->ao[chan] = data[0];
	ao_win_out(data[0] ^ invert, DACx_Direct_Data_671x(chan));

	return 0;
}

int ni_ao_inttrig(struct a4l_subdevice *subd, lsampl_t trignum)
{
	struct a4l_device *dev = subd->dev;
	int ret, interrupt_b_bits, i;
	static const int timeout = 1000;

	if (trignum != 0)
		return -EINVAL;

	/* TODO: disable trigger until a command is recorded.
	   Null trig at beginning prevent ao start trigger from executing
	   more than once per command (and doing things like trying to
	   allocate the ao dma channel multiple times) */

	ni_set_bits(dev, Interrupt_B_Enable_Register,
		    AO_FIFO_Interrupt_Enable | AO_Error_Interrupt_Enable, 0);
	interrupt_b_bits = AO_Error_Interrupt_Enable;

#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))
	devpriv->stc_writew(dev, 1, DAC_FIFO_Clear);
	if (boardtype.reg_type & ni_reg_6xxx_mask)
		ni_ao_win_outl(dev, 0x6, AO_FIFO_Offset_Load_611x);
	ret = ni_ao_setup_MITE_dma(subd);
	if (ret)
		return ret;
	ret = ni_ao_wait_for_dma_load(subd);
	if (ret < 0)
		return ret;
#else /* !CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */
	ret = ni_ao_prep_fifo(subd);
	if (ret == 0)
		return -EPIPE;

	interrupt_b_bits |= AO_FIFO_Interrupt_Enable;
#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

	devpriv->stc_writew(dev, devpriv->ao_mode3 | AO_Not_An_UPDATE,
			    AO_Mode_3_Register);
	devpriv->stc_writew(dev, devpriv->ao_mode3, AO_Mode_3_Register);
	/* wait for DACs to be loaded */
	for (i = 0; i < timeout; i++) {
		a4l_udelay(1);
		if ((devpriv->stc_readw(dev,Joint_Status_2_Register) &
		     AO_TMRDACWRs_In_Progress_St) == 0)
			break;
	}
	if (i == timeout) {
		a4l_err(dev,
			"ni_ao_inttrig: timed out "
			"waiting for AO_TMRDACWRs_In_Progress_St to clear");
		return -EIO;
	}
	/* stc manual says we are need to clear error interrupt after
	   AO_TMRDACWRs_In_Progress_St clears */
	devpriv->stc_writew(dev, AO_Error_Interrupt_Ack,
			    Interrupt_B_Ack_Register);

	ni_set_bits(dev, Interrupt_B_Enable_Register, interrupt_b_bits, 1);

	devpriv->stc_writew(dev,
			    devpriv->ao_cmd1 |
			    AO_UI_Arm | AO_UC_Arm |
			    AO_BC_Arm | AO_DAC1_Update_Mode |
			    AO_DAC0_Update_Mode,
			    AO_Command_1_Register);

	devpriv->stc_writew(dev,
			    devpriv->ao_cmd2 | AO_START1_Pulse,
			    AO_Command_2_Register);

	return 0;
}

int ni_ao_cmd(struct a4l_subdevice *subd, struct a4l_cmd_desc *cmd)
{
	struct a4l_device *dev = subd->dev;

	int bits;
	int i;
	unsigned trigvar;

	if (a4l_get_irq(dev) == A4L_IRQ_UNUSED) {
		a4l_err(dev, "ni_ao_cmd: cannot run command without an irq");
		return -EIO;
	}

	devpriv->stc_writew(dev, AO_Configuration_Start, Joint_Reset_Register);

	devpriv->stc_writew(dev, AO_Disarm, AO_Command_1_Register);

	if (boardtype.reg_type & ni_reg_6xxx_mask) {
		ao_win_out(CLEAR_WG, AO_Misc_611x);

		bits = 0;
		for (i = 0; i < cmd->nb_chan; i++) {
			int chan;

			chan = CR_CHAN(cmd->chan_descs[i]);
			bits |= 1 << chan;
			ao_win_out(chan, AO_Waveform_Generation_611x);
		}
		ao_win_out(bits, AO_Timed_611x);
	}

	ni_ao_config_chan_descs(subd, cmd->chan_descs, cmd->nb_chan, 1);

	if (cmd->stop_src == TRIG_NONE) {
		devpriv->ao_mode1 |= AO_Continuous;
		devpriv->ao_mode1 &= ~AO_Trigger_Once;
	} else {
		devpriv->ao_mode1 &= ~AO_Continuous;
		devpriv->ao_mode1 |= AO_Trigger_Once;
	}
	devpriv->stc_writew(dev, devpriv->ao_mode1, AO_Mode_1_Register);
	devpriv->ao_trigger_select &=
		~(AO_START1_Polarity | AO_START1_Select(-1));
	devpriv->ao_trigger_select |= AO_START1_Edge | AO_START1_Sync;
	devpriv->stc_writew(dev, devpriv->ao_trigger_select,
			    AO_Trigger_Select_Register);
	devpriv->ao_mode3 &= ~AO_Trigger_Length;
	devpriv->stc_writew(dev, devpriv->ao_mode3, AO_Mode_3_Register);

	devpriv->stc_writew(dev, devpriv->ao_mode1, AO_Mode_1_Register);
	devpriv->ao_mode2 &= ~AO_BC_Initial_Load_Source;
	devpriv->stc_writew(dev, devpriv->ao_mode2, AO_Mode_2_Register);
	if (cmd->stop_src == TRIG_NONE) {
		devpriv->stc_writel(dev, 0xffffff, AO_BC_Load_A_Register);
	} else {
		devpriv->stc_writel(dev, 0, AO_BC_Load_A_Register);
	}
	devpriv->stc_writew(dev, AO_BC_Load, AO_Command_1_Register);
	devpriv->ao_mode2 &= ~AO_UC_Initial_Load_Source;
	devpriv->stc_writew(dev, devpriv->ao_mode2, AO_Mode_2_Register);
	switch (cmd->stop_src) {
	case TRIG_COUNT:
		devpriv->stc_writel(dev, cmd->stop_arg, AO_UC_Load_A_Register);
		devpriv->stc_writew(dev, AO_UC_Load, AO_Command_1_Register);
		devpriv->stc_writel(dev, cmd->stop_arg - 1,
				    AO_UC_Load_A_Register);
		break;
	case TRIG_NONE:
		devpriv->stc_writel(dev, 0xffffff, AO_UC_Load_A_Register);
		devpriv->stc_writew(dev, AO_UC_Load, AO_Command_1_Register);
		devpriv->stc_writel(dev, 0xffffff, AO_UC_Load_A_Register);
		break;
	default:
		devpriv->stc_writel(dev, 0, AO_UC_Load_A_Register);
		devpriv->stc_writew(dev, AO_UC_Load, AO_Command_1_Register);
		devpriv->stc_writel(dev, cmd->stop_arg, AO_UC_Load_A_Register);
	}

	devpriv->ao_mode1 &=
		~(AO_UI_Source_Select(0x1f) | AO_UI_Source_Polarity |
		  AO_UPDATE_Source_Select(0x1f) | AO_UPDATE_Source_Polarity);
	switch (cmd->scan_begin_src) {
	case TRIG_TIMER:
		devpriv->ao_cmd2 &= ~AO_BC_Gate_Enable;
		trigvar =
			ni_ns_to_timer(dev, cmd->scan_begin_arg,
				       TRIG_ROUND_NEAREST);
		devpriv->stc_writel(dev, 1, AO_UI_Load_A_Register);
		devpriv->stc_writew(dev, AO_UI_Load, AO_Command_1_Register);
		devpriv->stc_writel(dev, trigvar, AO_UI_Load_A_Register);
		break;
	case TRIG_EXT:
		devpriv->ao_mode1 |=
			AO_UPDATE_Source_Select(cmd->scan_begin_arg);
		if (cmd->scan_begin_arg & CR_INVERT)
			devpriv->ao_mode1 |= AO_UPDATE_Source_Polarity;
		devpriv->ao_cmd2 |= AO_BC_Gate_Enable;
		break;
	default:
		BUG();
		break;
	}
	devpriv->stc_writew(dev, devpriv->ao_cmd2, AO_Command_2_Register);
	devpriv->stc_writew(dev, devpriv->ao_mode1, AO_Mode_1_Register);
	devpriv->ao_mode2 &=
		~(AO_UI_Reload_Mode(3) | AO_UI_Initial_Load_Source);
	devpriv->stc_writew(dev, devpriv->ao_mode2, AO_Mode_2_Register);

	if ((boardtype.reg_type & ni_reg_6xxx_mask) == 0) {
		if (cmd->scan_end_arg > 1) {
			devpriv->ao_mode1 |= AO_Multiple_Channels;
			devpriv->stc_writew(dev,
					    AO_Number_Of_Channels(cmd->scan_end_arg - 1) |
					    AO_UPDATE_Output_Select
					    (AO_Update_Output_High_Z),
					    AO_Output_Control_Register);
		} else {
			unsigned int bits;
			devpriv->ao_mode1 &= ~AO_Multiple_Channels;
			bits = AO_UPDATE_Output_Select(AO_Update_Output_High_Z);
			if (boardtype.reg_type & ni_reg_m_series_mask) {
				bits |= AO_Number_Of_Channels(0);
			} else {
				bits |= AO_Number_Of_Channels(CR_CHAN(cmd->
								      chan_descs[0]));
			}
			devpriv->stc_writew(dev, bits,
					    AO_Output_Control_Register);
		}
		devpriv->stc_writew(dev, devpriv->ao_mode1, AO_Mode_1_Register);
	}

	devpriv->stc_writew(dev, AO_DAC0_Update_Mode | AO_DAC1_Update_Mode,
			    AO_Command_1_Register);

	devpriv->ao_mode3 |= AO_Stop_On_Overrun_Error;
	devpriv->stc_writew(dev, devpriv->ao_mode3, AO_Mode_3_Register);

	devpriv->ao_mode2 &= ~AO_FIFO_Mode_Mask;

#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))
	devpriv->ao_mode2 |= AO_FIFO_Mode_HF_to_F;
#else /* !CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */
	devpriv->ao_mode2 |= AO_FIFO_Mode_HF;
#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */
	devpriv->ao_mode2 &= ~AO_FIFO_Retransmit_Enable;
	devpriv->stc_writew(dev, devpriv->ao_mode2, AO_Mode_2_Register);

	bits = AO_BC_Source_Select | AO_UPDATE_Pulse_Width |
		AO_TMRDACWR_Pulse_Width;
	if (boardtype.ao_fifo_depth)
		bits |= AO_FIFO_Enable;
	else
		bits |= AO_DMA_PIO_Control;
#if 0
	/* F Hess: windows driver does not set AO_Number_Of_DAC_Packages bit for 6281,
	   verified with bus analyzer. */
	if (boardtype.reg_type & ni_reg_m_series_mask)
		bits |= AO_Number_Of_DAC_Packages;
#endif
	devpriv->stc_writew(dev, bits, AO_Personal_Register);
	/* enable sending of ao dma requests */
	devpriv->stc_writew(dev, AO_AOFREQ_Enable, AO_Start_Select_Register);

	devpriv->stc_writew(dev, AO_Configuration_End, Joint_Reset_Register);

	if (cmd->stop_src == TRIG_COUNT) {
		devpriv->stc_writew(dev, AO_BC_TC_Interrupt_Ack,
				    Interrupt_B_Ack_Register);
		ni_set_bits(dev, Interrupt_B_Enable_Register,
			    AO_BC_TC_Interrupt_Enable, 1);
	}

	return 0;
}

struct a4l_cmd_desc mio_ao_cmd_mask = {
	.idx_subd = 0,
	.start_src = TRIG_INT,
	.scan_begin_src = TRIG_TIMER | TRIG_EXT,
	.convert_src = TRIG_NOW,
	.scan_end_src = TRIG_COUNT,
	.stop_src = TRIG_COUNT | TRIG_NONE,
};

int ni_ao_cmdtest(struct a4l_subdevice *subd, struct a4l_cmd_desc *cmd)
{
	struct a4l_device *dev = subd->dev;

	/* Make sure trigger sources are unique and mutually compatible */

	if (cmd->stop_src != TRIG_COUNT && cmd->stop_src != TRIG_NONE)
		return -EINVAL;

	/* Make sure arguments are trivially compatible */

	if (cmd->start_arg != 0) {
		cmd->start_arg = 0;
		return -EINVAL;
	}

	if (cmd->scan_begin_src == TRIG_TIMER) {
		if (cmd->scan_begin_arg < boardtype.ao_speed) {
			cmd->scan_begin_arg = boardtype.ao_speed;
			return -EINVAL;
		}
		if (cmd->scan_begin_arg > devpriv->clock_ns * 0xffffff) {
			/* XXX check */
			cmd->scan_begin_arg = devpriv->clock_ns * 0xffffff;
			return -EINVAL;
		}
	}

	if (cmd->convert_arg != 0) {
		cmd->convert_arg = 0;
		return -EINVAL;
	}
	if (cmd->scan_end_arg != cmd->nb_chan) {
		cmd->scan_end_arg = cmd->nb_chan;
		return -EINVAL;
	}
	if (cmd->stop_src == TRIG_COUNT) {
		/* XXX check */
		if (cmd->stop_arg > 0x00ffffff) {
			cmd->stop_arg = 0x00ffffff;
			return -EINVAL;
		}
	} else {
		/* TRIG_NONE */
		if (cmd->stop_arg != 0) {
			cmd->stop_arg = 0;
			return -EINVAL;
		}
	}

	/* step 4: fix up any arguments */
	if (cmd->scan_begin_src == TRIG_TIMER) {

		if(cmd->scan_begin_arg !=
		   ni_timer_to_ns(dev,
				  ni_ns_to_timer(dev,
						 cmd->scan_begin_arg,
						 cmd->flags & TRIG_ROUND_MASK)))
			return -EINVAL;
	}

	return 0;
}

void ni_ao_reset(struct a4l_subdevice *subd)
{
	struct a4l_device *dev = subd->dev;

	ni_release_ao_mite_channel(dev);

	devpriv->stc_writew(dev, AO_Configuration_Start, Joint_Reset_Register);
	devpriv->stc_writew(dev, AO_Disarm, AO_Command_1_Register);
	ni_set_bits(dev, Interrupt_B_Enable_Register, ~0, 0);
	devpriv->stc_writew(dev, AO_BC_Source_Select, AO_Personal_Register);
	devpriv->stc_writew(dev, 0x3f98, Interrupt_B_Ack_Register);
	devpriv->stc_writew(dev, AO_BC_Source_Select | AO_UPDATE_Pulse_Width |
			    AO_TMRDACWR_Pulse_Width, AO_Personal_Register);
	devpriv->stc_writew(dev, 0, AO_Output_Control_Register);
	devpriv->stc_writew(dev, 0, AO_Start_Select_Register);
	devpriv->ao_cmd1 = 0;
	devpriv->stc_writew(dev, devpriv->ao_cmd1, AO_Command_1_Register);
	devpriv->ao_cmd2 = 0;
	devpriv->stc_writew(dev, devpriv->ao_cmd2, AO_Command_2_Register);
	devpriv->ao_mode1 = 0;
	devpriv->stc_writew(dev, devpriv->ao_mode1, AO_Mode_1_Register);
	devpriv->ao_mode2 = 0;
	devpriv->stc_writew(dev, devpriv->ao_mode2, AO_Mode_2_Register);
	if (boardtype.reg_type & ni_reg_m_series_mask)
		devpriv->ao_mode3 = AO_Last_Gate_Disable;
	else
		devpriv->ao_mode3 = 0;
	devpriv->stc_writew(dev, devpriv->ao_mode3, AO_Mode_3_Register);
	devpriv->ao_trigger_select = 0;
	devpriv->stc_writew(dev, devpriv->ao_trigger_select,
			    AO_Trigger_Select_Register);
	if (boardtype.reg_type & ni_reg_6xxx_mask) {
		ao_win_out(0x3, AO_Immediate_671x);
		ao_win_out(CLEAR_WG, AO_Misc_611x);
	}
	devpriv->stc_writew(dev, AO_Configuration_End, Joint_Reset_Register);
}

/* digital io */

int ni_dio_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	unsigned int *data = (unsigned int *)insn->data;

#ifdef CONFIG_DEBUG_DIO
	a4l_info(dev, "chan=%d io=%d\n", CR_CHAN(insn->chan_desc), data[0]);
#endif /* CONFIG_DEBUG_DIO */

	switch (data[0]) {
	case A4L_INSN_CONFIG_DIO_OUTPUT:
		devpriv->io_bits |= 1 << CR_CHAN(insn->chan_desc);
		break;
	case A4L_INSN_CONFIG_DIO_INPUT:
		devpriv->io_bits &= ~(1 << CR_CHAN(insn->chan_desc));
		break;
	case A4L_INSN_CONFIG_DIO_QUERY:
		data[1] = (devpriv->io_bits &
			   (1 << CR_CHAN(insn->chan_desc))) ?
			A4L_OUTPUT : A4L_INPUT;
		return 0;
		break;
	default:
		return -EINVAL;
	}

	devpriv->dio_control &= ~DIO_Pins_Dir_Mask;
	devpriv->dio_control |= DIO_Pins_Dir(devpriv->io_bits);
	devpriv->stc_writew(dev, devpriv->dio_control, DIO_Control_Register);

	return 1;
}

int ni_dio_insn_bits(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint8_t *data = (uint8_t *)insn->data;

#ifdef CONFIG_DEBUG_DIO
	a4l_info(dev, "mask=0x%x bits=0x%x\n", data[0], data[1]);
#endif

	if (insn->data_size != 2 * sizeof(uint8_t))
		return -EINVAL;

	if (data[0]) {
		/* Perform check to make sure we're not using the
		   serial part of the dio */
		if ((data[0] & (DIO_SDIN | DIO_SDOUT))
		    && devpriv->serial_interval_ns)
			return -EBUSY;

		devpriv->dio_state &= ~data[0];
		devpriv->dio_state |= (data[0] & data[1]);
		devpriv->dio_output &= ~DIO_Parallel_Data_Mask;
		devpriv->dio_output |=
			DIO_Parallel_Data_Out(devpriv->dio_state);
		devpriv->stc_writew(dev, devpriv->dio_output,
				    DIO_Output_Register);
	}

	data[1] = (uint8_t)
		devpriv->stc_readw(dev, DIO_Parallel_Input_Register);

	return 0;
}

int ni_m_series_dio_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	unsigned int *data = (unsigned int *)insn->data;

#ifdef CONFIG_DEBUG_DIO
	a4l_info(dev, "chan=%d io=%d\n", CR_CHAN(insn->chan_desc), data[0]);
#endif
	switch (data[0]) {
	case A4L_INSN_CONFIG_DIO_OUTPUT:
		devpriv->io_bits |= 1 << CR_CHAN(insn->chan_desc);
		break;
	case A4L_INSN_CONFIG_DIO_INPUT:
		devpriv->io_bits &= ~(1 << CR_CHAN(insn->chan_desc));
		break;
	case A4L_INSN_CONFIG_DIO_QUERY:
		data[1] = (devpriv->io_bits &
			   (1 << CR_CHAN(insn->chan_desc))) ?
			A4L_OUTPUT : A4L_INPUT;
		return 0;
		break;
	default:
		return -EINVAL;
	}

	ni_writel(devpriv->io_bits, M_Offset_DIO_Direction);

	return 0;
}

int ni_m_series_dio_insn_bits_8(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint8_t *data = (uint8_t *)insn->data;

#ifdef CONFIG_DEBUG_DIO
	a4l_info(dev, "mask=0x%x bits=0x%x\n", data[0], data[1]);
#endif

	if (insn->data_size != 2 * sizeof(uint8_t))
		return -EINVAL;

	if (data[0]) {
		devpriv->dio_state &= ~data[0];
		devpriv->dio_state |= (data[0] & data[1]);
		ni_writel(devpriv->dio_state, M_Offset_Static_Digital_Output);
	}

	data[1] = (uint8_t) ni_readl(M_Offset_Static_Digital_Input);

	return 0;
}

int ni_m_series_dio_insn_bits_32(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint32_t *data = (uint32_t *)insn->data;

#ifdef CONFIG_DEBUG_DIO
	a4l_info(dev, "mask=0x%x bits=0x%x\n", data[0], data[1]);
#endif

	if (insn->data_size != 2 * sizeof(uint32_t))
		return -EINVAL;

	if (data[0]) {
		devpriv->dio_state &= ~data[0];
		devpriv->dio_state |= (data[0] & data[1]);
		ni_writel(devpriv->dio_state, M_Offset_Static_Digital_Output);
	}

	data[1] = ni_readl(M_Offset_Static_Digital_Input);

	return 0;
}

#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))

struct a4l_cmd_desc mio_dio_cmd_mask = {
	.idx_subd = 0,
	.start_src = TRIG_INT,
	.scan_begin_src = TRIG_EXT,
	.convert_src = TRIG_NOW,
	.scan_end_src = TRIG_COUNT,
	.stop_src = TRIG_NONE,
};

int ni_cdio_cmdtest(struct a4l_subdevice *subd, struct a4l_cmd_desc *cmd)
{
	unsigned int i;

	/* Make sure arguments are trivially compatible */

	if (cmd->start_arg != 0) {
		cmd->start_arg = 0;
		return -EINVAL;
	}

	if ((cmd->scan_begin_arg &
	     PACK_FLAGS(CDO_Sample_Source_Select_Mask, 0, 0, CR_INVERT)) !=
	    cmd->scan_begin_arg)
		return -EINVAL;

	if (cmd->convert_arg != 0) {
		cmd->convert_arg = 0;
		return -EINVAL;
	}

	if (cmd->scan_end_arg != cmd->nb_chan) {
		cmd->scan_end_arg = cmd->nb_chan;
		return -EINVAL;
	}

	if (cmd->stop_arg != 0) {
		cmd->stop_arg = 0;
		return -EINVAL;
	}

	/* Check chan_descs */

	for (i = 0; i < cmd->nb_chan; ++i) {
		if (cmd->chan_descs[i] != i)
			return -EINVAL;
	}

	return 0;
}

int ni_cdio_cmd(struct a4l_subdevice *subd, struct a4l_cmd_desc *cmd)
{
	struct a4l_device *dev = subd->dev;
	unsigned cdo_mode_bits = CDO_FIFO_Mode_Bit | CDO_Halt_On_Error_Bit;

	ni_writel(CDO_Reset_Bit, M_Offset_CDIO_Command);
	switch (cmd->scan_begin_src) {
	case TRIG_EXT:
		cdo_mode_bits |=
			CR_CHAN(cmd->scan_begin_arg) &
			CDO_Sample_Source_Select_Mask;
		break;
	default:
		BUG();
		break;
	}
	if (cmd->scan_begin_arg & CR_INVERT)
		cdo_mode_bits |= CDO_Polarity_Bit;
	ni_writel(cdo_mode_bits, M_Offset_CDO_Mode);

	if (devpriv->io_bits) {
		ni_writel(devpriv->dio_state, M_Offset_CDO_FIFO_Data);
		ni_writel(CDO_SW_Update_Bit, M_Offset_CDIO_Command);
		ni_writel(devpriv->io_bits, M_Offset_CDO_Mask_Enable);
	} else {
		a4l_err(dev,
			"ni_cdio_cmd: attempted to run digital "
			"output command with no lines configured as outputs");
		return -EIO;
	}

	return 0;
}

void ni_cdio_cancel(struct a4l_subdevice *subd)
{
	struct a4l_device *dev = subd->dev;
	ni_writel(CDO_Disarm_Bit | CDO_Error_Interrupt_Enable_Clear_Bit |
		  CDO_Empty_FIFO_Interrupt_Enable_Clear_Bit |
		  CDO_FIFO_Request_Interrupt_Enable_Clear_Bit,
		  M_Offset_CDIO_Command);

	ni_writel(0, M_Offset_CDO_Mask_Enable);
	ni_release_cdo_mite_channel(dev);
}

int ni_cdo_inttrig(struct a4l_subdevice *subd, lsampl_t trignum)
{
	struct a4l_device *dev = subd->dev;
	int err;
	unsigned i;
	const unsigned timeout = 1000;

	/* TODO: disable trigger until a command is recorded.
	   Null trig at beginning prevent ao start trigger from executing
	   more than once per command (and doing things like trying to
	   allocate the ao dma channel multiple times) */

	err = ni_cdo_setup_MITE_dma(subd);
	if (err < 0)
		return err;

	/* wait for dma to fill output fifo */
	for (i = 0; i < timeout; ++i) {
		if (ni_readl(M_Offset_CDIO_Status) & CDO_FIFO_Full_Bit)
			break;
		a4l_udelay(10);
	}

	if (i == timeout) {
		a4l_err(dev, "ni_cdo_inttrig: dma failed to fill cdo fifo!");
		ni_cdio_cancel(subd);
		return -EIO;
	}

	ni_writel(CDO_Arm_Bit |
		  CDO_Error_Interrupt_Enable_Set_Bit |
		  CDO_Empty_FIFO_Interrupt_Enable_Set_Bit,
		  M_Offset_CDIO_Command);

	return 0;
}

#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

static void handle_cdio_interrupt(struct a4l_device *dev)
{
#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))
	unsigned cdio_status;
	unsigned long flags;
	struct a4l_subdevice *subd = a4l_get_subd(dev, NI_DIO_SUBDEV);

	if ((boardtype.reg_type & ni_reg_m_series_mask) == 0) {
		return;
	}
	rtdm_lock_get_irqsave(&devpriv->mite_channel_lock, flags);
	if (devpriv->cdo_mite_chan) {
		unsigned cdo_mite_status =
			a4l_mite_get_status(devpriv->cdo_mite_chan);
		if (cdo_mite_status & CHSR_LINKC) {
			writel(CHOR_CLRLC,
			       devpriv->mite->mite_io_addr +
			       MITE_CHOR(devpriv->cdo_mite_chan->channel));
		}
		a4l_mite_sync_output_dma(devpriv->cdo_mite_chan, subd);
	}
	rtdm_lock_put_irqrestore(&devpriv->mite_channel_lock, flags);

	cdio_status = ni_readl(M_Offset_CDIO_Status);
	if (cdio_status & (CDO_Overrun_Bit | CDO_Underflow_Bit)) {
		/* XXX just guessing this is needed and does something useful */
		ni_writel(CDO_Error_Interrupt_Confirm_Bit, M_Offset_CDIO_Command);
		a4l_buf_evt(subd, A4L_BUF_ERROR);
	}
	if (cdio_status & CDO_FIFO_Empty_Bit) {
		ni_writel(CDO_Empty_FIFO_Interrupt_Enable_Clear_Bit,
			  M_Offset_CDIO_Command);
	}
	a4l_buf_evt(subd, 0);
#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */
}

static int ni_serial_hw_readwrite8(struct a4l_device * dev,
				   unsigned char data_out, unsigned char *data_in)
{
	unsigned int status1;
	int err = 0, count = 20;

#ifdef CONFIG_DEBUG_DIO
	a4l_info(dev, "outputting 0x%x\n", data_out);
#endif

	devpriv->dio_output &= ~DIO_Serial_Data_Mask;
	devpriv->dio_output |= DIO_Serial_Data_Out(data_out);
	devpriv->stc_writew(dev, devpriv->dio_output, DIO_Output_Register);

	status1 = devpriv->stc_readw(dev, Joint_Status_1_Register);
	if (status1 & DIO_Serial_IO_In_Progress_St) {
		err = -EBUSY;
		goto Error;
	}

	devpriv->dio_control |= DIO_HW_Serial_Start;
	devpriv->stc_writew(dev, devpriv->dio_control, DIO_Control_Register);
	devpriv->dio_control &= ~DIO_HW_Serial_Start;

	/* Wait until STC says we're done, but don't loop infinitely. */
	while ((status1 =
		devpriv->stc_readw(dev,
				   Joint_Status_1_Register)) &
	       DIO_Serial_IO_In_Progress_St) {
		/* Delay one bit per loop */
		a4l_udelay((devpriv->serial_interval_ns + 999) / 1000);
		if (--count < 0) {
			a4l_err(dev,
				"ni_serial_hw_readwrite8: "
				"SPI serial I/O didn't finish in time!\n");
			err = -ETIME;
			goto Error;
		}
	}

	/* Delay for last bit. This delay is absolutely necessary, because
	   DIO_Serial_IO_In_Progress_St goes high one bit too early. */
	a4l_udelay((devpriv->serial_interval_ns + 999) / 1000);

	if (data_in != NULL) {
		*data_in = devpriv->stc_readw(dev, DIO_Serial_Input_Register);
#ifdef CONFIG_DEBUG_DIO
		a4l_info(dev, "inputted 0x%x\n", *data_in);
#endif
	}

Error:
	devpriv->stc_writew(dev, devpriv->dio_control, DIO_Control_Register);

	return err;
}

static int ni_serial_sw_readwrite8(struct a4l_device * dev,
				   unsigned char data_out, unsigned char *data_in)
{
	unsigned char mask, input = 0;

#ifdef CONFIG_DEBUG_DIO
	a4l_info(dev, "outputting 0x%x\n", data_out);
#endif

	/* Wait for one bit before transfer */
	a4l_udelay((devpriv->serial_interval_ns + 999) / 1000);

	for (mask = 0x80; mask; mask >>= 1) {
		/* Output current bit; note that we cannot touch devpriv->dio_state
		   because it is a per-subdevice field, and serial is
		   a separate subdevice from DIO. */
		devpriv->dio_output &= ~DIO_SDOUT;
		if (data_out & mask) {
			devpriv->dio_output |= DIO_SDOUT;
		}
		devpriv->stc_writew(dev, devpriv->dio_output,
				    DIO_Output_Register);

		/* Assert SDCLK (active low, inverted), wait for half of
		   the delay, deassert SDCLK, and wait for the other half. */
		devpriv->dio_control |= DIO_Software_Serial_Control;
		devpriv->stc_writew(dev, devpriv->dio_control,
				    DIO_Control_Register);

		a4l_udelay((devpriv->serial_interval_ns + 999) / 2000);

		devpriv->dio_control &= ~DIO_Software_Serial_Control;
		devpriv->stc_writew(dev, devpriv->dio_control,
				    DIO_Control_Register);

		a4l_udelay((devpriv->serial_interval_ns + 999) / 2000);

		/* Input current bit */
		if (devpriv->stc_readw(dev,
				       DIO_Parallel_Input_Register) & DIO_SDIN) {
			input |= mask;
		}
	}
#ifdef CONFIG_DEBUG_DIO
	a4l_info(dev, "inputted 0x%x\n", input);
#endif
	if (data_in)
		*data_in = input;

	return 0;
}

int ni_serial_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	int err = 0;
	unsigned char byte_out, byte_in = 0;
	unsigned int *data = (unsigned int *)insn->data;

	if (insn->data_size != 2 * sizeof(unsigned int))
		return -EINVAL;

	switch (data[0]) {
	case A4L_INSN_CONFIG_SERIAL_CLOCK:

#ifdef CONFIG_DEBUG_DIO
		a4l_info(dev, "SPI serial clock Config %d\n", data[1]);
#endif

		devpriv->serial_hw_mode = 1;
		devpriv->dio_control |= DIO_HW_Serial_Enable;

		if (data[1] == SERIAL_DISABLED) {
			devpriv->serial_hw_mode = 0;
			devpriv->dio_control &= ~(DIO_HW_Serial_Enable |
						  DIO_Software_Serial_Control);
			data[1] = SERIAL_DISABLED;
			devpriv->serial_interval_ns = data[1];
		} else if (data[1] <= SERIAL_600NS) {
			/* Warning: this clock speed is too fast to reliably
			   control SCXI. */
			devpriv->dio_control &= ~DIO_HW_Serial_Timebase;
			devpriv->clock_and_fout |= Slow_Internal_Timebase;
			devpriv->clock_and_fout &= ~DIO_Serial_Out_Divide_By_2;
			data[1] = SERIAL_600NS;
			devpriv->serial_interval_ns = data[1];
		} else if (data[1] <= SERIAL_1_2US) {
			devpriv->dio_control &= ~DIO_HW_Serial_Timebase;
			devpriv->clock_and_fout |= Slow_Internal_Timebase |
				DIO_Serial_Out_Divide_By_2;
			data[1] = SERIAL_1_2US;
			devpriv->serial_interval_ns = data[1];
		} else if (data[1] <= SERIAL_10US) {
			devpriv->dio_control |= DIO_HW_Serial_Timebase;
			devpriv->clock_and_fout |= Slow_Internal_Timebase |
				DIO_Serial_Out_Divide_By_2;
			/* Note: DIO_Serial_Out_Divide_By_2 only affects
			   600ns/1.2us. If you turn divide_by_2 off with the
			   slow clock, you will still get 10us, except then
			   all your delays are wrong. */
			data[1] = SERIAL_10US;
			devpriv->serial_interval_ns = data[1];
		} else {
			devpriv->dio_control &= ~(DIO_HW_Serial_Enable |
						  DIO_Software_Serial_Control);
			devpriv->serial_hw_mode = 0;
			data[1] = (data[1] / 1000) * 1000;
			devpriv->serial_interval_ns = data[1];
		}

		devpriv->stc_writew(dev, devpriv->dio_control,
				    DIO_Control_Register);
		devpriv->stc_writew(dev, devpriv->clock_and_fout,
				    Clock_and_FOUT_Register);
		return 0;

		break;

	case A4L_INSN_CONFIG_BIDIRECTIONAL_DATA:

		if (devpriv->serial_interval_ns == 0) {
			return -EINVAL;
		}

		byte_out = data[1] & 0xFF;

		if (devpriv->serial_hw_mode) {
			err = ni_serial_hw_readwrite8(dev, byte_out, &byte_in);
		} else if (devpriv->serial_interval_ns > 0) {
			err = ni_serial_sw_readwrite8(dev, byte_out, &byte_in);
		} else {
			a4l_err(dev,
				"ni_serial_insn_config: serial disabled!\n");
			return -EINVAL;
		}
		if (err < 0)
			return err;
		data[1] = byte_in & 0xFF;
		return 0;

		break;
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

void mio_common_detach(struct a4l_device * dev)
{
	if (dev->priv) {
		if (devpriv->counter_dev) {
			a4l_ni_gpct_device_destroy(devpriv->counter_dev);
		}
	}
}

static void init_ao_67xx(struct a4l_device * dev)
{
	struct a4l_subdevice *subd = a4l_get_subd(dev, NI_AO_SUBDEV);
	int i;

	if (subd == NULL) {
		a4l_err(dev, "%s: unable to find AO subdevice\n", __FUNCTION__);
		return;
	}

	for (i = 0; i < subd->chan_desc->length; i++)
		ni_ao_win_outw(dev, AO_Channel(i) | 0x0,
			       AO_Configuration_2_67xx);
}

static unsigned int ni_gpct_to_stc_register(enum ni_gpct_register reg)
{
	unsigned stc_register;
	switch (reg) {
	case NITIO_G0_Autoincrement_Reg:
		stc_register = G_Autoincrement_Register(0);
		break;
	case NITIO_G1_Autoincrement_Reg:
		stc_register = G_Autoincrement_Register(1);
		break;
	case NITIO_G0_Command_Reg:
		stc_register = G_Command_Register(0);
		break;
	case NITIO_G1_Command_Reg:
		stc_register = G_Command_Register(1);
		break;
	case NITIO_G0_HW_Save_Reg:
		stc_register = G_HW_Save_Register(0);
		break;
	case NITIO_G1_HW_Save_Reg:
		stc_register = G_HW_Save_Register(1);
		break;
	case NITIO_G0_SW_Save_Reg:
		stc_register = G_Save_Register(0);
		break;
	case NITIO_G1_SW_Save_Reg:
		stc_register = G_Save_Register(1);
		break;
	case NITIO_G0_Mode_Reg:
		stc_register = G_Mode_Register(0);
		break;
	case NITIO_G1_Mode_Reg:
		stc_register = G_Mode_Register(1);
		break;
	case NITIO_G0_LoadA_Reg:
		stc_register = G_Load_A_Register(0);
		break;
	case NITIO_G1_LoadA_Reg:
		stc_register = G_Load_A_Register(1);
		break;
	case NITIO_G0_LoadB_Reg:
		stc_register = G_Load_B_Register(0);
		break;
	case NITIO_G1_LoadB_Reg:
		stc_register = G_Load_B_Register(1);
		break;
	case NITIO_G0_Input_Select_Reg:
		stc_register = G_Input_Select_Register(0);
		break;
	case NITIO_G1_Input_Select_Reg:
		stc_register = G_Input_Select_Register(1);
		break;
	case NITIO_G01_Status_Reg:
		stc_register = G_Status_Register;
		break;
	case NITIO_G01_Joint_Reset_Reg:
		stc_register = Joint_Reset_Register;
		break;
	case NITIO_G01_Joint_Status1_Reg:
		stc_register = Joint_Status_1_Register;
		break;
	case NITIO_G01_Joint_Status2_Reg:
		stc_register = Joint_Status_2_Register;
		break;
	case NITIO_G0_Interrupt_Acknowledge_Reg:
		stc_register = Interrupt_A_Ack_Register;
		break;
	case NITIO_G1_Interrupt_Acknowledge_Reg:
		stc_register = Interrupt_B_Ack_Register;
		break;
	case NITIO_G0_Status_Reg:
		stc_register = AI_Status_1_Register;
		break;
	case NITIO_G1_Status_Reg:
		stc_register = AO_Status_1_Register;
		break;
	case NITIO_G0_Interrupt_Enable_Reg:
		stc_register = Interrupt_A_Enable_Register;
		break;
	case NITIO_G1_Interrupt_Enable_Reg:
		stc_register = Interrupt_B_Enable_Register;
		break;
	default:
		__a4l_err("%s: unhandled register 0x%x in switch.\n",
			  __FUNCTION__, reg);
		BUG();
		return 0;
		break;
	}
	return stc_register;
}

static void ni_gpct_write_register(struct ni_gpct *counter,
				   unsigned int bits, enum ni_gpct_register reg)
{
	struct a4l_device *dev = counter->counter_dev->dev;
	unsigned stc_register;
	/* bits in the join reset register which are relevant to counters */
	static const unsigned gpct_joint_reset_mask = G0_Reset | G1_Reset;
	static const unsigned gpct_interrupt_a_enable_mask =
		G0_Gate_Interrupt_Enable | G0_TC_Interrupt_Enable;
	static const unsigned gpct_interrupt_b_enable_mask =
		G1_Gate_Interrupt_Enable | G1_TC_Interrupt_Enable;

	switch (reg) {
		/* m-series-only registers */
	case NITIO_G0_Counting_Mode_Reg:
		ni_writew(bits, M_Offset_G0_Counting_Mode);
		break;
	case NITIO_G1_Counting_Mode_Reg:
		ni_writew(bits, M_Offset_G1_Counting_Mode);
		break;
	case NITIO_G0_Second_Gate_Reg:
		ni_writew(bits, M_Offset_G0_Second_Gate);
		break;
	case NITIO_G1_Second_Gate_Reg:
		ni_writew(bits, M_Offset_G1_Second_Gate);
		break;
	case NITIO_G0_DMA_Config_Reg:
		ni_writew(bits, M_Offset_G0_DMA_Config);
		break;
	case NITIO_G1_DMA_Config_Reg:
		ni_writew(bits, M_Offset_G1_DMA_Config);
		break;
	case NITIO_G0_ABZ_Reg:
		ni_writew(bits, M_Offset_G0_MSeries_ABZ);
		break;
	case NITIO_G1_ABZ_Reg:
		ni_writew(bits, M_Offset_G1_MSeries_ABZ);
		break;

		/* 32 bit registers */
	case NITIO_G0_LoadA_Reg:
	case NITIO_G1_LoadA_Reg:
	case NITIO_G0_LoadB_Reg:
	case NITIO_G1_LoadB_Reg:
		stc_register = ni_gpct_to_stc_register(reg);
		devpriv->stc_writel(dev, bits, stc_register);
		break;

		/* 16 bit registers */
	case NITIO_G0_Interrupt_Enable_Reg:
		BUG_ON(bits & ~gpct_interrupt_a_enable_mask);
		ni_set_bitfield(dev, Interrupt_A_Enable_Register,
				gpct_interrupt_a_enable_mask, bits);
		break;
	case NITIO_G1_Interrupt_Enable_Reg:
		BUG_ON(bits & ~gpct_interrupt_b_enable_mask);
		ni_set_bitfield(dev, Interrupt_B_Enable_Register,
				gpct_interrupt_b_enable_mask, bits);
		break;
	case NITIO_G01_Joint_Reset_Reg:
		BUG_ON(bits & ~gpct_joint_reset_mask);
		/* fall-through */
	default:
		stc_register = ni_gpct_to_stc_register(reg);
		devpriv->stc_writew(dev, bits, stc_register);
	}
}

static unsigned int ni_gpct_read_register(struct ni_gpct *counter,
					  enum ni_gpct_register reg)
{
	struct a4l_device *dev = counter->counter_dev->dev;
	unsigned int stc_register;
	switch (reg) {
		/* m-series only registers */
	case NITIO_G0_DMA_Status_Reg:
		return ni_readw(M_Offset_G0_DMA_Status);
		break;
	case NITIO_G1_DMA_Status_Reg:
		return ni_readw(M_Offset_G1_DMA_Status);
		break;

		/* 32 bit registers */
	case NITIO_G0_HW_Save_Reg:
	case NITIO_G1_HW_Save_Reg:
	case NITIO_G0_SW_Save_Reg:
	case NITIO_G1_SW_Save_Reg:
		stc_register = ni_gpct_to_stc_register(reg);
		return devpriv->stc_readl(dev, stc_register);
		break;

		/* 16 bit registers */
	default:
		stc_register = ni_gpct_to_stc_register(reg);
		return devpriv->stc_readw(dev, stc_register);
		break;
	}
	return 0;
}

int ni_freq_out_insn_read(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint8_t *data = (uint8_t *)insn->data;

	data[0] = FOUT_Divider(devpriv->clock_and_fout);

	return 0;
}

int ni_freq_out_insn_write(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint8_t *data = (uint8_t *)insn->data;

	devpriv->clock_and_fout &= ~FOUT_Enable;
	devpriv->stc_writew(dev, devpriv->clock_and_fout,
			    Clock_and_FOUT_Register);
	devpriv->clock_and_fout &= ~FOUT_Divider_mask;
	devpriv->clock_and_fout |= FOUT_Divider(data[0]);
	devpriv->clock_and_fout |= FOUT_Enable;
	devpriv->stc_writew(dev, devpriv->clock_and_fout,
			    Clock_and_FOUT_Register);

	return 0;
}

static int ni_set_freq_out_clock(struct a4l_device * dev, lsampl_t clock_source)
{
	switch (clock_source) {
	case NI_FREQ_OUT_TIMEBASE_1_DIV_2_CLOCK_SRC:
		devpriv->clock_and_fout &= ~FOUT_Timebase_Select;
		break;
	case NI_FREQ_OUT_TIMEBASE_2_CLOCK_SRC:
		devpriv->clock_and_fout |= FOUT_Timebase_Select;
		break;
	default:
		return -EINVAL;
	}
	devpriv->stc_writew(dev, devpriv->clock_and_fout,
			    Clock_and_FOUT_Register);

	return 0;
}

static void ni_get_freq_out_clock(struct a4l_device * dev,
				  unsigned int * clock_source,
				  unsigned int * clock_period_ns)
{
	if (devpriv->clock_and_fout & FOUT_Timebase_Select) {
		*clock_source = NI_FREQ_OUT_TIMEBASE_2_CLOCK_SRC;
		*clock_period_ns = TIMEBASE_2_NS;
	} else {
		*clock_source = NI_FREQ_OUT_TIMEBASE_1_DIV_2_CLOCK_SRC;
		*clock_period_ns = TIMEBASE_1_NS * 2;
	}
}

int ni_freq_out_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	unsigned int *data = (unsigned int *)insn->data;

	switch (data[0]) {
	case A4L_INSN_CONFIG_SET_CLOCK_SRC:
		return ni_set_freq_out_clock(dev, data[1]);
		break;
	case A4L_INSN_CONFIG_GET_CLOCK_SRC:
		ni_get_freq_out_clock(dev, &data[1], &data[2]);
		return 0;
	default:
		break;
	}

	return -EINVAL;
}

static int ni_8255_callback(int dir, int port, int data, unsigned long arg)
{
	struct a4l_device *dev = (struct a4l_device *) arg;

	if (dir) {
		ni_writeb(data, Port_A + 2 * port);
		return 0;
	} else {
		return ni_readb(Port_A + 2 * port);
	}
}

/*
  reads bytes out of eeprom
*/

static int ni_read_eeprom(struct a4l_device *dev, int addr)
{
	int bit;
	int bitstring;

	bitstring = 0x0300 | ((addr & 0x100) << 3) | (addr & 0xff);
	ni_writeb(0x04, Serial_Command);
	for (bit = 0x8000; bit; bit >>= 1) {
		ni_writeb(0x04 | ((bit & bitstring) ? 0x02 : 0),
			  Serial_Command);
		ni_writeb(0x05 | ((bit & bitstring) ? 0x02 : 0),
			  Serial_Command);
	}
	bitstring = 0;
	for (bit = 0x80; bit; bit >>= 1) {
		ni_writeb(0x04, Serial_Command);
		ni_writeb(0x05, Serial_Command);
		bitstring |= ((ni_readb(XXX_Status) & PROMOUT) ? bit : 0);
	}
	ni_writeb(0x00, Serial_Command);

	return bitstring;
}

/*
  presents the EEPROM as a subdevice
*/

static int ni_eeprom_insn_read(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint8_t *data = (uint8_t *)insn->data;

	data[0] = ni_read_eeprom(dev, CR_CHAN(insn->chan_desc));

	return 0;
}


static int ni_m_series_eeprom_insn_read(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint8_t *data = (uint8_t *)insn->data;

	data[0] = devpriv->eeprom_buffer[CR_CHAN(insn->chan_desc)];

	return 0;
}

static int ni_get_pwm_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	unsigned int *data = (unsigned int*)insn->data;

	data[1] = devpriv->pwm_up_count * devpriv->clock_ns;
	data[2] = devpriv->pwm_down_count * devpriv->clock_ns;

	return 0;
}

static int ni_m_series_pwm_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	unsigned int up_count, down_count;
	unsigned int *data = (unsigned int*)insn->data;

	switch (data[0]) {
	case A4L_INSN_CONFIG_PWM_OUTPUT:
		switch (data[1]) {
		case TRIG_ROUND_NEAREST:
			up_count =
				(data[2] +
				 devpriv->clock_ns / 2) / devpriv->clock_ns;
			break;
		case TRIG_ROUND_DOWN:
			up_count = data[2] / devpriv->clock_ns;
			break;
		case TRIG_ROUND_UP:
			up_count =(data[2] + devpriv->clock_ns - 1) /
				devpriv->clock_ns;
			break;
		default:
			return -EINVAL;
			break;
		}
		switch (data[3]) {
		case TRIG_ROUND_NEAREST:
			down_count = (data[4] + devpriv->clock_ns / 2) /
				devpriv->clock_ns;
			break;
		case TRIG_ROUND_DOWN:
			down_count = data[4] / devpriv->clock_ns;
			break;
		case TRIG_ROUND_UP:
			down_count =
				(data[4] + devpriv->clock_ns - 1) /
				devpriv->clock_ns;
			break;
		default:
			return -EINVAL;
			break;
		}
		if (up_count * devpriv->clock_ns != data[2] ||
		    down_count * devpriv->clock_ns != data[4]) {
			data[2] = up_count * devpriv->clock_ns;
			data[4] = down_count * devpriv->clock_ns;
			return -EAGAIN;
		}
		ni_writel(MSeries_Cal_PWM_High_Time_Bits(up_count) |
			  MSeries_Cal_PWM_Low_Time_Bits(down_count),
			  M_Offset_Cal_PWM);
		devpriv->pwm_up_count = up_count;
		devpriv->pwm_down_count = down_count;
		return 0;
		break;
	case A4L_INSN_CONFIG_GET_PWM_OUTPUT:
		return ni_get_pwm_config(subd, insn);
		break;
	default:
		return -EINVAL;
		break;
	}
	return 0;
}

static int ni_6143_pwm_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	unsigned int *data = (unsigned int*)insn->data;

	unsigned up_count, down_count;
	switch (data[0]) {
	case A4L_INSN_CONFIG_PWM_OUTPUT:
		switch (data[1]) {
		case TRIG_ROUND_NEAREST:
			up_count =
				(data[2] + devpriv->clock_ns / 2) /
				devpriv->clock_ns;
			break;
		case TRIG_ROUND_DOWN:
			up_count = data[2] / devpriv->clock_ns;
			break;
		case TRIG_ROUND_UP:
			up_count = (data[2] + devpriv->clock_ns - 1) /
				devpriv->clock_ns;
			break;
		default:
			return -EINVAL;
			break;
		}
		switch (data[3]) {
		case TRIG_ROUND_NEAREST:
			down_count = (data[4] + devpriv->clock_ns / 2) /
				devpriv->clock_ns;
			break;
		case TRIG_ROUND_DOWN:
			down_count = data[4] / devpriv->clock_ns;
			break;
		case TRIG_ROUND_UP:
			down_count = (data[4] + devpriv->clock_ns - 1) /
				devpriv->clock_ns;
			break;
		default:
			return -EINVAL;
			break;
		}
		if (up_count * devpriv->clock_ns != data[2] ||
		    down_count * devpriv->clock_ns != data[4]) {
			data[2] = up_count * devpriv->clock_ns;
			data[4] = down_count * devpriv->clock_ns;
			return -EAGAIN;
		}
		ni_writel(up_count, Calibration_HighTime_6143);
		devpriv->pwm_up_count = up_count;
		ni_writel(down_count, Calibration_LowTime_6143);
		devpriv->pwm_down_count = down_count;
		return 0;
		break;
	case A4L_INSN_CONFIG_GET_PWM_OUTPUT:
		return ni_get_pwm_config(subd, insn);
	default:
		return -EINVAL;
		break;
	}
	return 0;
}

static int pack_mb88341(int addr, int val, int *bitstring)
{
	/*
	  Fujitsu MB 88341
	  Note that address bits are reversed.  Thanks to
	  Ingo Keen for noticing this.

	  Note also that the 88341 expects address values from
	  1-12, whereas we use channel numbers 0-11.  The NI
	  docs use 1-12, also, so be careful here.
	*/
	addr++;
	*bitstring = ((addr & 0x1) << 11) |
		((addr & 0x2) << 9) |
		((addr & 0x4) << 7) | ((addr & 0x8) << 5) | (val & 0xff);
	return 12;
}

static int pack_dac8800(int addr, int val, int *bitstring)
{
	*bitstring = ((addr & 0x7) << 8) | (val & 0xff);
	return 11;
}

static int pack_dac8043(int addr, int val, int *bitstring)
{
	*bitstring = val & 0xfff;
	return 12;
}

static int pack_ad8522(int addr, int val, int *bitstring)
{
	*bitstring = (val & 0xfff) | (addr ? 0xc000 : 0xa000);
	return 16;
}

static int pack_ad8804(int addr, int val, int *bitstring)
{
	*bitstring = ((addr & 0xf) << 8) | (val & 0xff);
	return 12;
}

static int pack_ad8842(int addr, int val, int *bitstring)
{
	*bitstring = ((addr + 1) << 8) | (val & 0xff);
	return 12;
}

struct caldac_struct {
	int n_chans;
	int n_bits;
	int (*packbits) (int, int, int *);
};

static struct caldac_struct caldacs[] = {
	[mb88341] = {12, 8, pack_mb88341},
	[dac8800] = {8, 8, pack_dac8800},
	[dac8043] = {1, 12, pack_dac8043},
	[ad8522] = {2, 12, pack_ad8522},
	[ad8804] = {12, 8, pack_ad8804},
	[ad8842] = {8, 8, pack_ad8842},
	[ad8804_debug] = {16, 8, pack_ad8804},
};

static void ni_write_caldac(struct a4l_device * dev, int addr, int val)
{
	unsigned int loadbit = 0, bits = 0, bit, bitstring = 0;
	int i;
	int type;

	if (devpriv->caldacs[addr] == val)
		return;
	devpriv->caldacs[addr] = val;

	for (i = 0; i < 3; i++) {
		type = boardtype.caldac[i];
		if (type == caldac_none)
			break;
		if (addr < caldacs[type].n_chans) {
			bits = caldacs[type].packbits(addr, val, &bitstring);
			loadbit = SerDacLd(i);
			break;
		}
		addr -= caldacs[type].n_chans;
	}

	for (bit = 1 << (bits - 1); bit; bit >>= 1) {
		ni_writeb(((bit & bitstring) ? 0x02 : 0), Serial_Command);
		a4l_udelay(1);
		ni_writeb(1 | ((bit & bitstring) ? 0x02 : 0), Serial_Command);
		a4l_udelay(1);
	}
	ni_writeb(loadbit, Serial_Command);
	a4l_udelay(1);
	ni_writeb(0, Serial_Command);
}

static void caldac_setup(struct a4l_device *dev, struct a4l_subdevice *subd)
{
	int i, j;
	int n_dacs;
	int n_chans = 0;
	int n_bits;
	int diffbits = 0;
	int type;
	int chan;

	type = boardtype.caldac[0];
	if (type == caldac_none)
		return;
	n_bits = caldacs[type].n_bits;
	for (i = 0; i < 3; i++) {
		type = boardtype.caldac[i];
		if (type == caldac_none)
			break;
		if (caldacs[type].n_bits != n_bits)
			diffbits = 1;
		n_chans += caldacs[type].n_chans;
	}
	n_dacs = i;

	if (diffbits) {

		if (n_chans > MAX_N_CALDACS) {
			a4l_err(dev, "BUG! MAX_N_CALDACS too small\n");
		}

		subd->chan_desc = kmalloc(sizeof(struct a4l_channels_desc) +
					  n_chans * sizeof(struct a4l_channel), GFP_KERNEL);

		memset(subd->chan_desc,
		       0,
		       sizeof(struct a4l_channels_desc) + n_chans * sizeof(struct a4l_channel));

		subd->chan_desc->length = n_chans;
		subd->chan_desc->mode = A4L_CHAN_PERCHAN_CHANDESC;

		chan = 0;
		for (i = 0; i < n_dacs; i++) {
			type = boardtype.caldac[i];
			for (j = 0; j < caldacs[type].n_chans; j++) {

				subd->chan_desc->chans[chan].nb_bits =
					caldacs[type].n_bits;

				chan++;
			}
		}

		for (chan = 0; chan < n_chans; chan++) {
			unsigned long tmp =
				(1 << subd->chan_desc->chans[chan].nb_bits) / 2;
			ni_write_caldac(dev, chan, tmp);
		}
	} else {
		subd->chan_desc = kmalloc(sizeof(struct a4l_channels_desc) +
					  sizeof(struct a4l_channel), GFP_KERNEL);

		memset(subd->chan_desc,
		       0, sizeof(struct a4l_channels_desc) + sizeof(struct a4l_channel));

		subd->chan_desc->length = n_chans;
		subd->chan_desc->mode = A4L_CHAN_GLOBAL_CHANDESC;

		type = boardtype.caldac[0];

		subd->chan_desc->chans[0].nb_bits = caldacs[type].n_bits;

		for (chan = 0; chan < n_chans; chan++)
			ni_write_caldac(dev,
					chan,
					(1 << subd->chan_desc->chans[0].nb_bits) / 2);
	}
}

static int ni_calib_insn_write(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint16_t *data = (uint16_t *)insn->data;

	ni_write_caldac(dev, CR_CHAN(insn->chan_desc), data[0]);
	return 0;
}

static int ni_calib_insn_read(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint16_t *data = (uint16_t *)insn->data;

	data[0] = devpriv->caldacs[CR_CHAN(insn->chan_desc)];

	return 0;
}

static int ni_gpct_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct ni_gpct *counter = (struct ni_gpct *)subd->priv;
	return a4l_ni_tio_insn_config(counter, insn);
}

static int ni_gpct_insn_read(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct ni_gpct *counter = (struct ni_gpct *)subd->priv;
	return a4l_ni_tio_rinsn(counter, insn);
}

static int ni_gpct_insn_write(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct ni_gpct *counter = (struct ni_gpct *)subd->priv;
	return a4l_ni_tio_winsn(counter, insn);
}

#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))

static int ni_gpct_cmd(struct a4l_subdevice *subd, struct a4l_cmd_desc *cmd)
{
	int retval;
	struct a4l_device *dev = subd->dev;
	struct ni_gpct *counter = (struct ni_gpct *)subd->priv;
	struct mite_dma_descriptor_ring *ring;

	retval = ni_request_gpct_mite_channel(dev,
					      counter->counter_index,
					      A4L_INPUT);
	if (retval) {
		a4l_err(dev,
			"ni_gpct_cmd: "
			"no dma channel available for use by counter\n");
		return retval;
	}

	ring = devpriv->gpct_mite_ring[counter->counter_index];
	retval = a4l_mite_buf_change(ring, subd);
	if (retval) {
		a4l_err(dev,
			"ni_gpct_cmd: "
			"dma ring configuration failed\n");
		return retval;

	}

	a4l_ni_tio_acknowledge_and_confirm(counter, NULL, NULL, NULL, NULL);
	ni_e_series_enable_second_irq(dev, counter->counter_index, 1);
	retval = a4l_ni_tio_cmd(counter, cmd);

	return retval;
}

static int ni_gpct_cmdtest(struct a4l_subdevice *subd, struct a4l_cmd_desc *cmd)
{
	struct ni_gpct *counter = (struct ni_gpct *)subd->priv;
	return a4l_ni_tio_cmdtest(counter, cmd);
}

static void ni_gpct_cancel(struct a4l_subdevice *subd)
{
	struct a4l_device *dev = subd->dev;
	struct ni_gpct *counter = (struct ni_gpct *)subd->priv;

	a4l_ni_tio_cancel(counter);
	ni_e_series_enable_second_irq(dev, counter->counter_index, 0);
	ni_release_gpct_mite_channel(dev, counter->counter_index);
}

#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */


/*
 *
 *  Programmable Function Inputs
 *
 */

static int ni_m_series_set_pfi_routing(struct a4l_device *dev,
				       unsigned int chan, unsigned int source)
{
	unsigned int pfi_reg_index;
	unsigned int array_offset;

	if ((source & 0x1f) != source)
		return -EINVAL;
	pfi_reg_index = 1 + chan / 3;
	array_offset = pfi_reg_index - 1;
	devpriv->pfi_output_select_reg[array_offset] &=
		~MSeries_PFI_Output_Select_Mask(chan);
	devpriv->pfi_output_select_reg[array_offset] |=
		MSeries_PFI_Output_Select_Bits(chan, source);
	ni_writew(devpriv->pfi_output_select_reg[array_offset],
		  M_Offset_PFI_Output_Select(pfi_reg_index));
	return 2;
}

static unsigned int ni_old_get_pfi_routing(struct a4l_device *dev,
					   unsigned int chan)
{
	/* pre-m-series boards have fixed signals on pfi pins */

	switch (chan) {
	case 0:
		return NI_PFI_OUTPUT_AI_START1;
		break;
	case 1:
		return NI_PFI_OUTPUT_AI_START2;
		break;
	case 2:
		return NI_PFI_OUTPUT_AI_CONVERT;
		break;
	case 3:
		return NI_PFI_OUTPUT_G_SRC1;
		break;
	case 4:
		return NI_PFI_OUTPUT_G_GATE1;
		break;
	case 5:
		return NI_PFI_OUTPUT_AO_UPDATE_N;
		break;
	case 6:
		return NI_PFI_OUTPUT_AO_START1;
		break;
	case 7:
		return NI_PFI_OUTPUT_AI_START_PULSE;
		break;
	case 8:
		return NI_PFI_OUTPUT_G_SRC0;
		break;
	case 9:
		return NI_PFI_OUTPUT_G_GATE0;
		break;
	default:
		__a4l_err("%s: bug, unhandled case in switch.\n",
			  __FUNCTION__);
		break;
	}
	return 0;
}

static int ni_old_set_pfi_routing(struct a4l_device *dev,
				  unsigned int chan, unsigned int source)
{
	/* pre-m-series boards have fixed signals on pfi pins */
	if (source != ni_old_get_pfi_routing(dev, chan))
		return -EINVAL;

	return 2;
}

static int ni_set_pfi_routing(struct a4l_device *dev,
			      unsigned int chan, unsigned int source)
{
	if (boardtype.reg_type & ni_reg_m_series_mask)
		return ni_m_series_set_pfi_routing(dev, chan, source);
	else
		return ni_old_set_pfi_routing(dev, chan, source);
}

static unsigned int ni_m_series_get_pfi_routing(struct a4l_device *dev,
						unsigned int chan)
{
	const unsigned int array_offset = chan / 3;
	return MSeries_PFI_Output_Select_Source(chan,
						devpriv->pfi_output_select_reg[array_offset]);
}

static unsigned int ni_get_pfi_routing(struct a4l_device *dev, unsigned int chan)
{
	if (boardtype.reg_type & ni_reg_m_series_mask)
		return ni_m_series_get_pfi_routing(dev, chan);
	else
		return ni_old_get_pfi_routing(dev, chan);
}

static int ni_config_filter(struct a4l_device *dev,
			    unsigned int pfi_channel, int filter)
{
	unsigned int bits;
	if ((boardtype.reg_type & ni_reg_m_series_mask) == 0) {
		return -ENOTSUPP;
	}
	bits = ni_readl(M_Offset_PFI_Filter);
	bits &= ~MSeries_PFI_Filter_Select_Mask(pfi_channel);
	bits |= MSeries_PFI_Filter_Select_Bits(pfi_channel, filter);
	ni_writel(bits, M_Offset_PFI_Filter);
	return 0;
}

static int ni_pfi_insn_bits(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint16_t *data = (uint16_t *)insn->data;

	if (data[0]) {
		devpriv->pfi_state &= ~data[0];
		devpriv->pfi_state |= (data[0] & data[1]);
		ni_writew(devpriv->pfi_state, M_Offset_PFI_DO);
	}

	data[1] = ni_readw(M_Offset_PFI_DI);

	return 0;
}

static int ni_pfi_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	unsigned int chan, *data = (unsigned int *)insn->data;

	if (insn->data_size < sizeof(unsigned int))
		return -EINVAL;

	chan = CR_CHAN(insn->chan_desc);

	switch (data[0]) {
	case A4L_OUTPUT:
		ni_set_bits(dev, IO_Bidirection_Pin_Register, 1 << chan, 1);
		break;
	case A4L_INPUT:
		ni_set_bits(dev, IO_Bidirection_Pin_Register, 1 << chan, 0);
		break;
	case A4L_INSN_CONFIG_DIO_QUERY:
		data[1] = (devpriv->io_bidirection_pin_reg & (1 << chan)) ?
			A4L_OUTPUT :	A4L_INPUT;
		return 0;
		break;
	case A4L_INSN_CONFIG_SET_ROUTING:
		return ni_set_pfi_routing(dev, chan, data[1]);
		break;
	case A4L_INSN_CONFIG_GET_ROUTING:
		data[1] = ni_get_pfi_routing(dev, chan);
		break;
	case A4L_INSN_CONFIG_FILTER:
		return ni_config_filter(dev, chan, data[1]);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*
 *
 *  RTSI Bus Functions
 *
 */

/* Find best multiplier/divider to try and get the PLL running at 80 MHz
 * given an arbitrary frequency input clock */
static int ni_mseries_get_pll_parameters(unsigned int reference_period_ns,
					 unsigned int *freq_divider,
					 unsigned int *freq_multiplier,
					 unsigned int *actual_period_ns)
{
	unsigned div;
	unsigned best_div = 1;
	static const unsigned max_div = 0x10;
	unsigned mult;
	unsigned best_mult = 1;
	static const unsigned max_mult = 0x100;
	static const unsigned pico_per_nano = 1000;

	const unsigned reference_picosec = reference_period_ns * pico_per_nano;
	/* m-series wants the phased-locked loop to output 80MHz, which is divided by 4 to
	 * 20 MHz for most timing clocks */
	static const unsigned target_picosec = 12500;
	static const unsigned fudge_factor_80_to_20Mhz = 4;
	int best_period_picosec = 0;
	for (div = 1; div <= max_div; ++div) {
		for (mult = 1; mult <= max_mult; ++mult) {
			unsigned new_period_ps =
				(reference_picosec * div) / mult;
			if (abs(new_period_ps - target_picosec) <
			    abs(best_period_picosec - target_picosec)) {
				best_period_picosec = new_period_ps;
				best_div = div;
				best_mult = mult;
			}
		}
	}
	if (best_period_picosec == 0) {
		__a4l_err("%s: bug, failed to find pll parameters\n",
			  __FUNCTION__);
		return -EIO;
	}
	*freq_divider = best_div;
	*freq_multiplier = best_mult;
	*actual_period_ns =
		(best_period_picosec * fudge_factor_80_to_20Mhz +
		 (pico_per_nano / 2)) / pico_per_nano;
	return 0;
}

static int ni_mseries_set_pll_master_clock(struct a4l_device * dev,
					   unsigned int source,
					   unsigned int period_ns)
{
	static const unsigned min_period_ns = 50;
	static const unsigned max_period_ns = 1000;
	static const unsigned timeout = 1000;
	unsigned pll_control_bits;
	unsigned freq_divider;
	unsigned freq_multiplier;
	unsigned i;
	int retval;
	if (source == NI_MIO_PLL_PXI10_CLOCK)
		period_ns = 100;
	/* These limits are somewhat arbitrary, but NI advertises 1 to
	   20MHz range so we'll use that */
	if (period_ns < min_period_ns || period_ns > max_period_ns) {
		a4l_err(dev,
			"%s: you must specify an input clock frequency "
			"between %i and %i nanosec "
			"for the phased-lock loop.\n",
			__FUNCTION__, min_period_ns, max_period_ns);
		return -EINVAL;
	}
	devpriv->rtsi_trig_direction_reg &= ~Use_RTSI_Clock_Bit;
	devpriv->stc_writew(dev, devpriv->rtsi_trig_direction_reg,
			    RTSI_Trig_Direction_Register);
	pll_control_bits =
		MSeries_PLL_Enable_Bit | MSeries_PLL_VCO_Mode_75_150MHz_Bits;
	devpriv->clock_and_fout2 |=
		MSeries_Timebase1_Select_Bit | MSeries_Timebase3_Select_Bit;
	devpriv->clock_and_fout2 &= ~MSeries_PLL_In_Source_Select_Mask;
	switch (source) {
	case NI_MIO_PLL_PXI_STAR_TRIGGER_CLOCK:
		devpriv->clock_and_fout2 |=
			MSeries_PLL_In_Source_Select_Star_Trigger_Bits;
		retval = ni_mseries_get_pll_parameters(period_ns, &freq_divider,
						       &freq_multiplier, &devpriv->clock_ns);
		if (retval < 0)
			return retval;
		break;
	case NI_MIO_PLL_PXI10_CLOCK:
		/* pxi clock is 10MHz */
		devpriv->clock_and_fout2 |=
			MSeries_PLL_In_Source_Select_PXI_Clock10;
		retval = ni_mseries_get_pll_parameters(period_ns, &freq_divider,
						       &freq_multiplier, &devpriv->clock_ns);
		if (retval < 0)
			return retval;
		break;
	default:
	{
		unsigned rtsi_channel;
		static const unsigned max_rtsi_channel = 7;
		for (rtsi_channel = 0; rtsi_channel <= max_rtsi_channel;
		     ++rtsi_channel) {
			if (source ==
			    NI_MIO_PLL_RTSI_CLOCK(rtsi_channel)) {
				devpriv->clock_and_fout2 |=
					MSeries_PLL_In_Source_Select_RTSI_Bits
					(rtsi_channel);
				break;
			}
		}
		if (rtsi_channel > max_rtsi_channel)
			return -EINVAL;
		retval = ni_mseries_get_pll_parameters(period_ns,
						       &freq_divider, &freq_multiplier,
						       &devpriv->clock_ns);
		if (retval < 0)
			return retval;
	}
	break;
	}
	ni_writew(devpriv->clock_and_fout2, M_Offset_Clock_and_Fout2);
	pll_control_bits |=
		MSeries_PLL_Divisor_Bits(freq_divider) |
		MSeries_PLL_Multiplier_Bits(freq_multiplier);
	ni_writew(pll_control_bits, M_Offset_PLL_Control);
	devpriv->clock_source = source;
	/* It seems to typically take a few hundred microseconds for PLL to lock */
	for (i = 0; i < timeout; ++i) {
		if (ni_readw(M_Offset_PLL_Status) & MSeries_PLL_Locked_Bit) {
			break;
		}
		udelay(1);
	}
	if (i == timeout) {
		a4l_err(dev,
			"%s: timed out waiting for PLL to lock "
			"to reference clock source %i with period %i ns.\n",
			__FUNCTION__, source, period_ns);
		return -ETIMEDOUT;
	}
	return 3;
}

static int ni_set_master_clock(struct a4l_device *dev,
			       unsigned int source, unsigned int period_ns)
{
	if (source == NI_MIO_INTERNAL_CLOCK) {
		devpriv->rtsi_trig_direction_reg &= ~Use_RTSI_Clock_Bit;
		devpriv->stc_writew(dev, devpriv->rtsi_trig_direction_reg,
				    RTSI_Trig_Direction_Register);
		devpriv->clock_ns = TIMEBASE_1_NS;
		if (boardtype.reg_type & ni_reg_m_series_mask) {
			devpriv->clock_and_fout2 &=
				~(MSeries_Timebase1_Select_Bit |
				  MSeries_Timebase3_Select_Bit);
			ni_writew(devpriv->clock_and_fout2,
				  M_Offset_Clock_and_Fout2);
			ni_writew(0, M_Offset_PLL_Control);
		}
		devpriv->clock_source = source;
	} else {
		if (boardtype.reg_type & ni_reg_m_series_mask) {
			return ni_mseries_set_pll_master_clock(dev, source,
							       period_ns);
		} else {
			if (source == NI_MIO_RTSI_CLOCK) {
				devpriv->rtsi_trig_direction_reg |=
					Use_RTSI_Clock_Bit;
				devpriv->stc_writew(dev,
						    devpriv->rtsi_trig_direction_reg,
						    RTSI_Trig_Direction_Register);
				if (devpriv->clock_ns == 0) {
					a4l_err(dev,
						"%s: we don't handle an "
						"unspecified clock period "
						"correctly yet, returning error.\n",
						__FUNCTION__);
					return -EINVAL;
				} else {
					devpriv->clock_ns = period_ns;
				}
				devpriv->clock_source = source;
			} else
				return -EINVAL;
		}
	}
	return 3;
}

static void ni_rtsi_init(struct a4l_device * dev)
{
	/* Initialise the RTSI bus signal switch to a default state */

	/* Set clock mode to internal */
	devpriv->clock_and_fout2 = MSeries_RTSI_10MHz_Bit;
	if (ni_set_master_clock(dev, NI_MIO_INTERNAL_CLOCK, 0) < 0) {
		a4l_err(dev, "ni_set_master_clock failed, bug?");
	}

	/* Default internal lines routing to RTSI bus lines */
	devpriv->rtsi_trig_a_output_reg =
		RTSI_Trig_Output_Bits(0, NI_RTSI_OUTPUT_ADR_START1) |
		RTSI_Trig_Output_Bits(1, NI_RTSI_OUTPUT_ADR_START2) |
		RTSI_Trig_Output_Bits(2, NI_RTSI_OUTPUT_SCLKG) |
		RTSI_Trig_Output_Bits(3, NI_RTSI_OUTPUT_DACUPDN);
	devpriv->stc_writew(dev, devpriv->rtsi_trig_a_output_reg,
			    RTSI_Trig_A_Output_Register);
	devpriv->rtsi_trig_b_output_reg =
		RTSI_Trig_Output_Bits(4, NI_RTSI_OUTPUT_DA_START1) |
		RTSI_Trig_Output_Bits(5, NI_RTSI_OUTPUT_G_SRC0) |
		RTSI_Trig_Output_Bits(6, NI_RTSI_OUTPUT_G_GATE0);

	if (boardtype.reg_type & ni_reg_m_series_mask)
		devpriv->rtsi_trig_b_output_reg |=
			RTSI_Trig_Output_Bits(7, NI_RTSI_OUTPUT_RTSI_OSC);
	devpriv->stc_writew(dev, devpriv->rtsi_trig_b_output_reg,
			    RTSI_Trig_B_Output_Register);
}

int a4l_ni_E_init(struct a4l_device *dev)
{
	int ret;
	unsigned int j, counter_variant;
	struct a4l_subdevice *subd;

	if (boardtype.n_aochan > MAX_N_AO_CHAN) {
		a4l_err(dev, "bug! boardtype.n_aochan > MAX_N_AO_CHAN\n");
		return -EINVAL;
	}

	/* analog input subdevice */

	a4l_dbg(1, drv_dbg, dev, "mio_common: starting attach procedure...\n");

	subd = a4l_alloc_subd(0, NULL);
	if(subd == NULL)
		return -ENOMEM;

	a4l_dbg(1, drv_dbg, dev, "mio_common: registering AI subdevice...\n");

	if (boardtype.n_adchan) {

		a4l_dbg(1, drv_dbg, dev,
			"mio_common: AI: %d channels\n", boardtype.n_adchan);

		subd->flags = A4L_SUBD_AI | A4L_SUBD_CMD | A4L_SUBD_MMAP;
		subd->rng_desc = ni_range_lkup[boardtype.gainlkup];

		subd->chan_desc = kmalloc(sizeof(struct a4l_channels_desc) +
					  sizeof(struct a4l_channel), GFP_KERNEL);

		subd->chan_desc->mode = A4L_CHAN_GLOBAL_CHANDESC;
		subd->chan_desc->length = boardtype.n_adchan;
		subd->chan_desc->chans[0].flags = A4L_CHAN_AREF_DIFF;
		if (boardtype.reg_type != ni_reg_611x)
			subd->chan_desc->chans[0].flags |= A4L_CHAN_AREF_GROUND |
				A4L_CHAN_AREF_COMMON | A4L_CHAN_AREF_OTHER;
		subd->chan_desc->chans[0].nb_bits = boardtype.adbits;

		subd->insn_read = ni_ai_insn_read;
		subd->insn_config = ni_ai_insn_config;
		subd->do_cmdtest = ni_ai_cmdtest;
		subd->do_cmd = ni_ai_cmd;
		subd->cancel = ni_ai_reset;
		subd->trigger = ni_ai_inttrig;

		subd->munge = (boardtype.adbits > 16) ?
			ni_ai_munge32 : ni_ai_munge16;

		subd->cmd_mask = &mio_ai_cmd_mask;
	} else {
		a4l_dbg(1, drv_dbg, dev,
			"mio_common: AI subdevice not present\n");
		subd->flags = A4L_SUBD_UNUSED;
	}

	ret = a4l_add_subd(dev, subd);
	if(ret != NI_AI_SUBDEV)
		return ret;

	a4l_dbg(1, drv_dbg, dev, "mio_common: AI subdevice registered\n");

	subd = a4l_alloc_subd(0, NULL);
	if(subd == NULL)
		return -ENOMEM;

	a4l_dbg(1, drv_dbg, dev, "mio_common: registering AO subdevice...\n");

	/* analog output subdevice */
	if (boardtype.n_aochan) {

		a4l_dbg(1, drv_dbg, dev,
			"mio_common: AO: %d channels\n", boardtype.n_aochan);

		subd->flags = A4L_SUBD_AO;
		subd->chan_desc = kmalloc(sizeof(struct a4l_channels_desc) +
					  sizeof(struct a4l_channel), GFP_KERNEL);

		subd->chan_desc->mode = A4L_CHAN_GLOBAL_CHANDESC;
		subd->chan_desc->length = boardtype.n_aochan;
		subd->chan_desc->chans[0].flags = A4L_CHAN_AREF_GROUND;
		subd->chan_desc->chans[0].nb_bits = boardtype.aobits;

		subd->rng_desc = boardtype.ao_range_table;

		subd->insn_read = ni_ao_insn_read;
		if (boardtype.reg_type & ni_reg_6xxx_mask)
			subd->insn_write = &ni_ao_insn_write_671x;
		else
			subd->insn_write = &ni_ao_insn_write;


		if (boardtype.ao_fifo_depth) {
			subd->flags |= A4L_SUBD_CMD | A4L_SUBD_MMAP;
			subd->do_cmd = &ni_ao_cmd;
			subd->cmd_mask = &mio_ao_cmd_mask;
			subd->do_cmdtest = &ni_ao_cmdtest;
			subd->trigger = ni_ao_inttrig;
			if ((boardtype.reg_type & ni_reg_m_series_mask) == 0)
				subd->munge = &ni_ao_munge;
		}

		subd->cancel = &ni_ao_reset;

	} else {
		a4l_dbg(1, drv_dbg, dev,
			"mio_common: AO subdevice not present\n");
		subd->flags = A4L_SUBD_UNUSED;
	}

	ret = a4l_add_subd(dev, subd);
	if(ret != NI_AO_SUBDEV)
		return ret;

	a4l_dbg(1, drv_dbg, dev, "mio_common: AO subdevice registered\n");

	if ((boardtype.reg_type & ni_reg_67xx_mask))
		init_ao_67xx(dev);

	/* digital i/o subdevice */

	subd = a4l_alloc_subd(0, NULL);
	if(subd == NULL)
		return -ENOMEM;

	a4l_dbg(1, drv_dbg, dev, "mio_common: registering DIO subdevice...\n");
	a4l_dbg(1, drv_dbg, dev,
		"mio_common: DIO: %d channels\n",
		boardtype.num_p0_dio_channels);

	subd->flags = A4L_SUBD_DIO;

	subd->chan_desc = kmalloc(sizeof(struct a4l_channels_desc) +
				  sizeof(struct a4l_channel), GFP_KERNEL);
	subd->chan_desc->mode = A4L_CHAN_GLOBAL_CHANDESC;
	subd->chan_desc->length = boardtype.num_p0_dio_channels;
	subd->chan_desc->chans[0].flags = A4L_CHAN_AREF_GROUND;
	subd->chan_desc->chans[0].nb_bits = 1;
	devpriv->io_bits = 0; /* all bits input */

	subd->rng_desc = &range_digital;

	if (boardtype.reg_type & ni_reg_m_series_mask) {

		if (subd->chan_desc->length == 8)
			subd->insn_bits = ni_m_series_dio_insn_bits_8;
		else
			subd->insn_bits = ni_m_series_dio_insn_bits_32;

		subd->insn_config = ni_m_series_dio_insn_config;

#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))

		a4l_dbg(1, drv_dbg, dev,
			"mio_common: DIO: command feature available\n");

		subd->flags |= A4L_SUBD_CMD;
		subd->do_cmd = ni_cdio_cmd;
		subd->do_cmdtest = ni_cdio_cmdtest;
		subd->cmd_mask = &mio_dio_cmd_mask;
		subd->cancel = ni_cdio_cancel;
		subd->trigger = ni_cdo_inttrig;

#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

		ni_writel(CDO_Reset_Bit | CDI_Reset_Bit, M_Offset_CDIO_Command);
		ni_writel(devpriv->io_bits, M_Offset_DIO_Direction);
	} else {

		subd->insn_bits = ni_dio_insn_bits;
		subd->insn_config = ni_dio_insn_config;
		devpriv->dio_control = DIO_Pins_Dir(devpriv->io_bits);
		ni_writew(devpriv->dio_control, DIO_Control_Register);
	}

	ret = a4l_add_subd(dev, subd);
	if(ret != NI_DIO_SUBDEV)
		return ret;

	a4l_dbg(1, drv_dbg, dev, "mio_common: DIO subdevice registered\n");

	/* 8255 device */
	subd = a4l_alloc_subd(sizeof(subd_8255_t), NULL);
	if(subd == NULL)
		return -ENOMEM;

	a4l_dbg(1, drv_dbg, dev, "mio_common: registering 8255 subdevice...\n");

	if (boardtype.has_8255) {
		devpriv->subd_8255.cb_arg = (unsigned long)dev;
		devpriv->subd_8255.cb_func = ni_8255_callback;
		a4l_subdev_8255_init(subd);
	} else {
		a4l_dbg(1, drv_dbg, dev,
			"mio_common: 8255 subdevice not present\n");
		subd->flags = A4L_SUBD_UNUSED;
	}

	ret = a4l_add_subd(dev, subd);
	if(ret != NI_8255_DIO_SUBDEV)
		return ret;

	a4l_dbg(1, drv_dbg, dev, "mio_common: 8255 subdevice registered\n");

	/* formerly general purpose counter/timer device, but no longer used */
	subd = a4l_alloc_subd(0, NULL);
	if(subd == NULL)
		return -ENOMEM;

	subd->flags = A4L_SUBD_UNUSED;
	ret = a4l_add_subd(dev, subd);
	if(ret != NI_UNUSED_SUBDEV)
		return ret;

	/* calibration subdevice -- ai and ao */
	subd = a4l_alloc_subd(0, NULL);
	if(subd == NULL)
		return -ENOMEM;

	a4l_dbg(1, drv_dbg, dev, "mio_common: registering calib subdevice...\n");

	subd->flags = A4L_SUBD_CALIB;
	if (boardtype.reg_type & ni_reg_m_series_mask) {
		/* internal PWM analog output
		   used for AI nonlinearity calibration */
		a4l_dbg(1, drv_dbg, dev,
			"mio_common: calib: M series calibration");
		subd->insn_config = ni_m_series_pwm_config;
		ni_writel(0x0, M_Offset_Cal_PWM);
	} else if (boardtype.reg_type == ni_reg_6143) {
		/* internal PWM analog output
		   used for AI nonlinearity calibration */
		a4l_dbg(1, drv_dbg, dev,
			"mio_common: calib: 6143 calibration");
		subd->insn_config = ni_6143_pwm_config;
	} else {
		a4l_dbg(1, drv_dbg, dev,
			"mio_common: calib: common calibration");
		subd->insn_read = ni_calib_insn_read;
		subd->insn_write = ni_calib_insn_write;
		caldac_setup(dev, subd);
	}

	ret = a4l_add_subd(dev, subd);
	if(ret != NI_CALIBRATION_SUBDEV)
		return ret;

	a4l_dbg(1, drv_dbg, dev, "mio_common: calib subdevice registered\n");

	/* EEPROM */
	subd = a4l_alloc_subd(0, NULL);
	if(subd == NULL)
		return -ENOMEM;

	a4l_dbg(1, drv_dbg, dev,
		"mio_common: registering EEPROM subdevice...\n");

	subd->flags = A4L_SUBD_MEMORY;

	subd->chan_desc = kmalloc(sizeof(struct a4l_channels_desc) +
				  sizeof(struct a4l_channel), GFP_KERNEL);
	subd->chan_desc->mode = A4L_CHAN_GLOBAL_CHANDESC;
	subd->chan_desc->chans[0].flags = 0;
	subd->chan_desc->chans[0].nb_bits = 8;

	if (boardtype.reg_type & ni_reg_m_series_mask) {
		subd->chan_desc->length = M_SERIES_EEPROM_SIZE;
		subd->insn_read = ni_m_series_eeprom_insn_read;
	} else {
		subd->chan_desc->length = 512;
		subd->insn_read = ni_eeprom_insn_read;
	}

	a4l_dbg(1, drv_dbg, dev,
		"mio_common: EEPROM: size = %lu\n", subd->chan_desc->length);

	ret = a4l_add_subd(dev, subd);
	if(ret != NI_EEPROM_SUBDEV)
		return ret;

	a4l_dbg(1, drv_dbg, dev, "mio_common: EEPROM subdevice registered\n");

	/* PFI */
	subd = a4l_alloc_subd(0, NULL);
	if(subd == NULL)
		return -ENOMEM;

	a4l_dbg(1, drv_dbg, dev,
		"mio_common: registering PFI(DIO) subdevice...\n");

	subd->flags = A4L_SUBD_DIO;

	subd->chan_desc = kmalloc(sizeof(struct a4l_channels_desc) +
				  sizeof(struct a4l_channel), GFP_KERNEL);
	subd->chan_desc->mode = A4L_CHAN_GLOBAL_CHANDESC;
	subd->chan_desc->chans[0].flags = 0;
	subd->chan_desc->chans[0].nb_bits = 1;

	if (boardtype.reg_type & ni_reg_m_series_mask) {
		unsigned int i;
		subd->chan_desc->length = 16;
		ni_writew(devpriv->dio_state, M_Offset_PFI_DO);
		for (i = 0; i < NUM_PFI_OUTPUT_SELECT_REGS; ++i) {
			ni_writew(devpriv->pfi_output_select_reg[i],
				  M_Offset_PFI_Output_Select(i + 1));
		}
	} else
		subd->chan_desc->length = 10;

	a4l_dbg(1, drv_dbg, dev,
		"mio_common: PFI: %lu bits...\n", subd->chan_desc->length);

	if (boardtype.reg_type & ni_reg_m_series_mask) {
		subd->insn_bits = ni_pfi_insn_bits;
	}

	subd->insn_config = ni_pfi_insn_config;
	ni_set_bits(dev, IO_Bidirection_Pin_Register, ~0, 0);

	ret = a4l_add_subd(dev, subd);
	if(ret != NI_PFI_DIO_SUBDEV)
		return ret;

	a4l_dbg(1, drv_dbg, dev, "mio_common: PFI subdevice registered\n");

	/* cs5529 calibration adc */
	subd = a4l_alloc_subd(0, NULL);
	if(subd == NULL)
		return -ENOMEM;

#if 0 /* TODO: add subdevices callbacks */
	subd->flags = A4L_SUBD_AI;

	if (boardtype.reg_type & ni_reg_67xx_mask) {

		subd->chan_desc = kmalloc(sizeof(struct a4l_channels_desc) +
					  sizeof(struct a4l_channel), GFP_KERNEL);
		subd->chan_desc->mode = A4L_CHAN_GLOBAL_CHANDESC;
		subd->chan_desc->length = boardtype.n_aochan;
		subd->chan_desc->chans[0].flags = 0;
		subd->chan_desc->chans[0].nb_bits = 16;

		/* one channel for each analog output channel */
		subd->rng_desc = &a4l_range_unknown;	/* XXX */
		s->insn_read = cs5529_ai_insn_read;
		init_cs5529(dev);
	} else
#endif /* TODO: add subdevices callbacks */
		subd->flags = A4L_SUBD_UNUSED;

	ret = a4l_add_subd(dev, subd);
	if(ret != NI_CS5529_CALIBRATION_SUBDEV)
		return ret;

	/* Serial */
	subd = a4l_alloc_subd(0, NULL);
	if(subd == NULL)
		return -ENOMEM;

	a4l_dbg(1, drv_dbg, dev,
		"mio_common: registering serial subdevice...\n");

	subd->flags = A4L_SUBD_SERIAL;

	subd->chan_desc = kmalloc(sizeof(struct a4l_channels_desc) +
				  sizeof(struct a4l_channel), GFP_KERNEL);
	subd->chan_desc->mode = A4L_CHAN_GLOBAL_CHANDESC;
	subd->chan_desc->length = 1;
	subd->chan_desc->chans[0].flags = 0;
	subd->chan_desc->chans[0].nb_bits = 8;

	subd->insn_config = ni_serial_insn_config;

	devpriv->serial_interval_ns = 0;
	devpriv->serial_hw_mode = 0;

	ret = a4l_add_subd(dev, subd);
	if(ret != NI_SERIAL_SUBDEV)
		return ret;

	a4l_dbg(1, drv_dbg, dev, "mio_common: serial subdevice registered\n");

	/* RTSI */
	subd = a4l_alloc_subd(0, NULL);
	if(subd == NULL)
		return -ENOMEM;

#if 1 /* TODO: add RTSI subdevice */
	subd->flags = A4L_SUBD_UNUSED;
	ni_rtsi_init(dev);

#else /* TODO: add RTSI subdevice */
	subd->flags = A4L_SUBD_DIO;

	subd->chan_desc = kmalloc(sizeof(struct a4l_channels_desc) +
				  sizeof(struct a4l_channel), GFP_KERNEL);
	subd->chan_desc->mode = A4L_CHAN_GLOBAL_CHANDESC;
	subd->chan_desc->length = 8;
	subd->chan_desc->chans[0].flags = 0;
	subd->chan_desc->chans[0].nb_bits = 1;

	subd->insn_bits = ni_rtsi_insn_bits;
	subd->insn_config = ni_rtsi_insn_config;
	ni_rtsi_init(dev);

#endif /* TODO: add RTSI subdevice */

	ret = a4l_add_subd(dev, subd);
	if(ret != NI_RTSI_SUBDEV)
		return ret;

	if (boardtype.reg_type & ni_reg_m_series_mask) {
		counter_variant = ni_gpct_variant_m_series;
	} else {
		counter_variant = ni_gpct_variant_e_series;
	}
	devpriv->counter_dev =
		a4l_ni_gpct_device_construct(dev,
					     &ni_gpct_write_register,
					     &ni_gpct_read_register,
					     counter_variant, NUM_GPCT);

	/* General purpose counters */
	for (j = 0; j < NUM_GPCT; ++j) {
		struct ni_gpct *counter;

		subd = a4l_alloc_subd(sizeof(struct ni_gpct), NULL);
		if(subd == NULL)
			return -ENOMEM;

		a4l_dbg(1, drv_dbg, dev,
			"mio_common: registering GPCT[%d] subdevice...\n", j);

		subd->flags = A4L_SUBD_COUNTER;

		subd->chan_desc = kmalloc(sizeof(struct a4l_channels_desc) +
					  sizeof(struct a4l_channel), GFP_KERNEL);
		subd->chan_desc->mode = A4L_CHAN_GLOBAL_CHANDESC;
		subd->chan_desc->length = 3;
		subd->chan_desc->chans[0].flags = 0;

		if (boardtype.reg_type & ni_reg_m_series_mask)
			subd->chan_desc->chans[0].nb_bits = 32;
		else
			subd->chan_desc->chans[0].nb_bits = 24;

		a4l_dbg(1, drv_dbg, dev,
			"mio_common: GPCT[%d]: %lu bits\n",
			j, subd->chan_desc->chans[0].nb_bits);

		subd->insn_read = ni_gpct_insn_read;
		subd->insn_write = ni_gpct_insn_write;
		subd->insn_config = ni_gpct_insn_config;

#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))

		a4l_dbg(1, drv_dbg, dev,
			"mio_common: GPCT[%d]: command feature available\n", j);
		subd->flags |= A4L_SUBD_CMD;
		subd->cmd_mask = &a4l_ni_tio_cmd_mask;
		subd->do_cmd = ni_gpct_cmd;
		subd->do_cmdtest = ni_gpct_cmdtest;
		subd->cancel = ni_gpct_cancel;
#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

		counter = (struct ni_gpct *)subd->priv;
		rtdm_lock_init(&counter->lock);
		counter->chip_index = 0;
		counter->counter_index = j;
		counter->counter_dev = devpriv->counter_dev;
		devpriv->counter_dev->counters[j] = counter;

		a4l_ni_tio_init_counter(counter);

		ret = a4l_add_subd(dev, subd);
		if(ret != NI_GPCT_SUBDEV(j))
			return ret;

		a4l_dbg(1, drv_dbg, dev,
			"mio_common: GCPT[%d] subdevice registered\n", j);
	}

	/* Frequency output */
	subd = a4l_alloc_subd(0, NULL);
	if(subd == NULL)
		return -ENOMEM;

	a4l_dbg(1, drv_dbg, dev,
		"mio_common: registering counter subdevice...\n");

	subd->flags = A4L_SUBD_COUNTER;

	subd->chan_desc = kmalloc(sizeof(struct a4l_channels_desc) +
				  sizeof(struct a4l_channel), GFP_KERNEL);
	subd->chan_desc->mode = A4L_CHAN_GLOBAL_CHANDESC;
	subd->chan_desc->length = 1;
	subd->chan_desc->chans[0].flags = 0;
	subd->chan_desc->chans[0].nb_bits = 4;

	subd->insn_read = ni_freq_out_insn_read;
	subd->insn_write = ni_freq_out_insn_write;
	subd->insn_config = ni_freq_out_insn_config;

	ret = a4l_add_subd(dev, subd);
	if(ret != NI_FREQ_OUT_SUBDEV)
		return ret;

	a4l_dbg(1, drv_dbg, dev,
		"mio_common: counter subdevice registered\n");

	a4l_dbg(1, drv_dbg, dev, "mio_common: initializing AI...\n");

	/* ai configuration */
	ni_ai_reset(a4l_get_subd(dev, NI_AI_SUBDEV));
	if ((boardtype.reg_type & ni_reg_6xxx_mask) == 0) {
		// BEAM is this needed for PCI-6143 ??
		devpriv->clock_and_fout =
			Slow_Internal_Time_Divide_By_2 |
			Slow_Internal_Timebase |
			Clock_To_Board_Divide_By_2 |
			Clock_To_Board |
			AI_Output_Divide_By_2 | AO_Output_Divide_By_2;
	} else {
		devpriv->clock_and_fout =
			Slow_Internal_Time_Divide_By_2 |
			Slow_Internal_Timebase |
			Clock_To_Board_Divide_By_2 | Clock_To_Board;
	}
	devpriv->stc_writew(dev, devpriv->clock_and_fout,
			    Clock_and_FOUT_Register);

	a4l_dbg(1, drv_dbg, dev, "mio_common: AI initialization OK\n");

	a4l_dbg(1, drv_dbg, dev, "mio_common: initializing A0...\n");

	/* analog output configuration */
	ni_ao_reset(a4l_get_subd(dev, NI_AO_SUBDEV));

	if (a4l_get_irq(dev) != A4L_IRQ_UNUSED) {
		devpriv->stc_writew(dev,
				    (devpriv->irq_polarity ? Interrupt_Output_Polarity : 0) |
				    (Interrupt_Output_On_3_Pins & 0) | Interrupt_A_Enable |
				    Interrupt_B_Enable |
				    Interrupt_A_Output_Select(devpriv->irq_pin) |
				    Interrupt_B_Output_Select(devpriv->irq_pin),
				    Interrupt_Control_Register);
	}

	a4l_dbg(1, drv_dbg, dev, "mio_common: A0 initialization OK\n");

	/* DMA setup */

	a4l_dbg(1, drv_dbg, dev, "mio_common: DMA setup\n");

	ni_writeb(devpriv->ai_ao_select_reg, AI_AO_Select);
	ni_writeb(devpriv->g0_g1_select_reg, G0_G1_Select);

	if (boardtype.reg_type & ni_reg_6xxx_mask) {
		ni_writeb(0, Magic_611x);
	} else if (boardtype.reg_type & ni_reg_m_series_mask) {
		int channel;
		for (channel = 0; channel < boardtype.n_aochan; ++channel) {
			ni_writeb(0xf, M_Offset_AO_Waveform_Order(channel));
			ni_writeb(0x0,
				  M_Offset_AO_Reference_Attenuation(channel));
		}
		ni_writeb(0x0, M_Offset_AO_Calibration);
	}

	a4l_dbg(1, drv_dbg, dev, "mio_common: attach procedure complete\n");

	return 0;
}

MODULE_DESCRIPTION("Analogy support for NI DAQ-STC based boards");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL_GPL(a4l_range_ni_E_ai);
EXPORT_SYMBOL_GPL(a4l_range_ni_E_ai_limited);
EXPORT_SYMBOL_GPL(a4l_range_ni_E_ai_limited14);
EXPORT_SYMBOL_GPL(a4l_range_ni_E_ai_bipolar4);
EXPORT_SYMBOL_GPL(a4l_range_ni_E_ai_611x);
EXPORT_SYMBOL_GPL(a4l_range_ni_M_ai_622x);
EXPORT_SYMBOL_GPL(a4l_range_ni_M_ai_628x);
EXPORT_SYMBOL_GPL(a4l_range_ni_S_ai_6143);
EXPORT_SYMBOL_GPL(a4l_range_ni_E_ao_ext);
EXPORT_SYMBOL_GPL(a4l_ni_E_interrupt);
EXPORT_SYMBOL_GPL(a4l_ni_E_init);
