/**
 * @file
 * Analogy for Linux, subdevice related features
 *
 * Copyright (C) 1997-2000 David A. Schleef <ds@schleef.org>
 * Copyright (C) 2008 Alexis Berlemont <alexis.berlemont@free.fr>
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _COBALT_RTDM_ANALOGY_SUBDEVICE_H
#define _COBALT_RTDM_ANALOGY_SUBDEVICE_H

#include <linux/list.h>
#include <rtdm/analogy/instruction.h>
#include <rtdm/analogy/command.h>
#include <rtdm/analogy/channel_range.h>

/* --- Subdevice descriptor structure --- */

struct a4l_device;
struct a4l_buffer;

/*!
 * @brief Structure describing the subdevice
 * @see a4l_add_subd()
 */

struct a4l_subdevice {

	struct list_head list;
			   /**< List stuff */

	struct a4l_device *dev;
			       /**< Containing device */

	unsigned int idx;
		      /**< Subdevice index */

	struct a4l_buffer *buf;
			       /**< Linked buffer */

	/* Subdevice's status (busy, linked?) */
	unsigned long status;
			     /**< Subdevice's status */

	/* Descriptors stuff */
	unsigned long flags;
			 /**< Type flags */
	struct a4l_channels_desc *chan_desc;
				/**< Tab of channels descriptors pointers */
	struct a4l_rngdesc *rng_desc;
				/**< Tab of ranges descriptors pointers */
	struct a4l_cmd_desc *cmd_mask;
			    /**< Command capabilities mask */

	/* Functions stuff */
	int (*insn_read) (struct a4l_subdevice *, struct a4l_kernel_instruction *);
							/**< Callback for the instruction "read" */
	int (*insn_write) (struct a4l_subdevice *, struct a4l_kernel_instruction *);
							 /**< Callback for the instruction "write" */
	int (*insn_bits) (struct a4l_subdevice *, struct a4l_kernel_instruction *);
							/**< Callback for the instruction "bits" */
	int (*insn_config) (struct a4l_subdevice *, struct a4l_kernel_instruction *);
							  /**< Callback for the configuration instruction */
	int (*do_cmd) (struct a4l_subdevice *, struct a4l_cmd_desc *);
					/**< Callback for command handling */
	int (*do_cmdtest) (struct a4l_subdevice *, struct a4l_cmd_desc *);
						       /**< Callback for command checking */
	void (*cancel) (struct a4l_subdevice *);
					 /**< Callback for asynchronous transfer cancellation */
	void (*munge) (struct a4l_subdevice *, void *, unsigned long);
								/**< Callback for munge operation */
	int (*trigger) (struct a4l_subdevice *, lsampl_t);
					      /**< Callback for trigger operation */

	char priv[0];
		  /**< Private data */
};

/* --- Subdevice related functions and macros --- */

struct a4l_channel *a4l_get_chfeat(struct a4l_subdevice * sb, int idx);
struct a4l_range *a4l_get_rngfeat(struct a4l_subdevice * sb, int chidx, int rngidx);
int a4l_check_chanlist(struct a4l_subdevice * subd,
		       unsigned char nb_chan, unsigned int *chans);

#define a4l_subd_is_input(x) ((A4L_SUBD_MASK_READ & (x)->flags) != 0)
/* The following macro considers that a DIO subdevice is firstly an
   output subdevice */
#define a4l_subd_is_output(x) \
	((A4L_SUBD_MASK_WRITE & (x)->flags) != 0 || \
	 (A4L_SUBD_DIO & (x)->flags) != 0)

/* --- Upper layer functions --- */

struct a4l_subdevice * a4l_get_subd(struct a4l_device *dev, int idx);
struct a4l_subdevice * a4l_alloc_subd(int sizeof_priv,
			    void (*setup)(struct a4l_subdevice *));
int a4l_add_subd(struct a4l_device *dev, struct a4l_subdevice * subd);
int a4l_ioctl_subdinfo(struct a4l_device_context * cxt, void *arg);
int a4l_ioctl_chaninfo(struct a4l_device_context * cxt, void *arg);
int a4l_ioctl_rnginfo(struct a4l_device_context * cxt, void *arg);
int a4l_ioctl_nbchaninfo(struct a4l_device_context * cxt, void *arg);
int a4l_ioctl_nbrnginfo(struct a4l_device_context * cxt, void *arg);

#endif /* !_COBALT_RTDM_ANALOGY_SUBDEVICE_H */
