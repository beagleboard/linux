/*
 * Analogy for Linux, driver facilities
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
#include <linux/module.h>
#include <linux/fs.h>
#include <rtdm/analogy/device.h>

/**
 * @ingroup cobalt
 * @defgroup analogy Analogy framework
 * A RTDM-based interface for implementing DAQ card drivers
 */

/**
 * @ingroup analogy
 * @defgroup analogy_driver_facilities Driver API
 * Programming interface provided to DAQ card drivers
 */

/* --- Driver section --- */

/**
 * @ingroup analogy_driver_facilities
 * @defgroup analogy_driver Driver management services
 *
 * Analogy driver registration / unregistration
 *
 * In a common Linux char driver, the developer has to register a fops
 * structure filled with callbacks for read / write / mmap / ioctl
 * operations.
 *
 * Analogy drivers do not have to implement read / write / mmap /
 * ioctl functions, these procedures are implemented in the Analogy
 * generic layer. Then, the transfers between user-space and
 * kernel-space are already managed. Analogy drivers work with commands
 * and instructions which are some kind of more dedicated read / write
 * operations. And, instead of registering a fops structure, a Analogy
 * driver must register some a4l_driver structure.
 *
 * @{
 */

/**
 * @brief Register an Analogy driver
 *
 * After initialising a driver structure, the driver must be made
 * available so as to be attached.
 *
 * @param[in] drv Driver descriptor structure
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
int a4l_register_drv(struct a4l_driver * drv);
EXPORT_SYMBOL_GPL(a4l_register_drv);

/**
 * @brief Unregister an Analogy driver
 *
 * This function removes the driver descriptor from the Analogy driver
 * list. The driver cannot be attached anymore.
 *
 * @param[in] drv Driver descriptor structure
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
int a4l_unregister_drv(struct a4l_driver * drv);
EXPORT_SYMBOL_GPL(a4l_unregister_drv);

/** @} */

/* --- Subdevice section --- */

/**
 * @ingroup analogy_driver_facilities
 * @defgroup analogy_subdevice Subdevice management services
 *
 * Subdevice declaration in a driver
 *
 * The subdevice structure is the most complex one in the Analogy
 * driver layer. It contains some description fields to fill and some
 * callbacks to declare.
 *
 * The description fields are:
 * - flags: to define the subdevice type and its capabilities;
 * - chan_desc: to describe the channels which compose the subdevice;
 * - rng_desc: to declare the usable ranges;
 *
 * The functions callbakcs are:
 * - do_cmd() and do_cmdtest(): to performe asynchronous acquisitions
 *   thanks to commands;
 * - cancel(): to abort a working asynchronous acquisition;
 * - munge(): to apply modifications on the data freshly acquired
 *   during an asynchronous transfer. Warning: using this feature with
 *   can significantly reduce the performances (if the munge operation
 *   is complex, it will trigger high CPU charge and if the
 *   acquisition device is DMA capable, many cache-misses and
 *   cache-replaces will occur (the benefits of the DMA controller
 *   will vanish);
 * - trigger(): optionnaly to launch an asynchronous acquisition;
 * - insn_read(), insn_write(), insn_bits(), insn_config(): to perform
 *   synchronous acquisition operations.
 *
 * Once the subdevice is filled, it must be inserted into the driver
 * structure thanks to a4l_add_subd().
 *
 * @{
 */

EXPORT_SYMBOL_GPL(a4l_range_bipolar10);
EXPORT_SYMBOL_GPL(a4l_range_bipolar5);
EXPORT_SYMBOL_GPL(a4l_range_unipolar10);
EXPORT_SYMBOL_GPL(a4l_range_unipolar5);
EXPORT_SYMBOL_GPL(a4l_range_unknown);
EXPORT_SYMBOL_GPL(a4l_range_fake);

/**
 * @brief Allocate a subdevice descriptor
 *
 * This is a helper function so as to get a suitable subdevice
 * descriptor
 *
 * @param[in] sizeof_priv Size of the subdevice's private data
 * @param[in] setup Setup function to be called after the allocation
 *
 * @return the index with which the subdevice has been registered, in
 * case of error a negative error code is returned.
 *
 */
struct a4l_subdevice * a4l_alloc_subd(int sizeof_priv,
				  void (*setup)(struct a4l_subdevice *));
EXPORT_SYMBOL_GPL(a4l_alloc_subd);

/**
 * @brief Add a subdevice to the driver descriptor
 *
 * Once the driver descriptor structure is initialized, the function
 * a4l_add_subd() must be used so to add some subdevices to the
 * driver.
 *
 * @param[in] dev Device descriptor structure
 * @param[in] subd Subdevice descriptor structure
 *
 * @return the index with which the subdevice has been registered, in
 * case of error a negative error code is returned.
 *
 */
int a4l_add_subd(struct a4l_device *dev, struct a4l_subdevice *subd);
EXPORT_SYMBOL_GPL(a4l_add_subd);

/**
 * @brief Get a pointer to the subdevice descriptor referenced by its
 * registration index
 *
 * This function is scarcely useful as all the drivers callbacks get
 * the related subdevice descriptor as first argument.
 * This function is not optimized, it goes through a linked list to
 * get the proper pointer. So it must not be used in real-time context
 * but at initialization / cleanup time (attach / detach).
 *
 * @param[in] dev Device descriptor structure
 * @param[in] idx Subdevice index
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
struct a4l_subdevice *a4l_get_subd(struct a4l_device *dev, int idx);
EXPORT_SYMBOL_GPL(a4l_get_subd);

/** @} */

/* --- Buffer section --- */

/**
 * @ingroup analogy_driver_facilities
 * @defgroup analogy_buffer Buffer management services
 *
 * Buffer management services
 *
 * The buffer is the key component of the Analogy infrastructure. It
 * manages transfers between the user-space and the Analogy drivers
 * thanks to generic functions which are described hereafter. Thanks
 * to the buffer subsystem, the driver developer does not have to care
 * about the way the user program retrieves or sends data.
 *
 * To write a classical char driver, the developer has to fill a fops
 * structure so as to provide transfer operations to the user program
 * (read, write, ioctl and mmap if need be).
 *
 * The Analogy infrastructure manages the whole interface with the
 * userspace; the common read, write, mmap, etc. callbacks are generic
 * Analogy functions. These functions manage (and perform, if need be)
 * tranfers between the user-space and an asynchronous buffer thanks
 * to lockless mechanisms.
 *
 * Consequently, the developer has to use the proper buffer functions
 * in order to write / read acquired data into / from the asynchronous
 * buffer.
 *
 * Here are listed the functions:
 * - a4l_buf_prepare_(abs)put() and a4l_buf_commit_(abs)put()
 * - a4l_buf_prepare_(abs)get() and a4l_buf_commit_(abs)get()
 * - a4l_buf_put()
 * - a4l_buf_get()
 * - a4l_buf_evt().
 *
 * The functions count might seem high; however, the developer needs a
 * few of them to write a driver. Having so many functions enables to
 * manage any transfer cases:
 * - If some DMA controller is available, there is no need to make the
 *   driver copy the acquired data into the asynchronous buffer, the
 *   DMA controller must directly trigger DMA shots into / from the
 *   buffer. In that case, a function a4l_buf_prepare_*() must be used
 *   so as to set up the DMA transfer and a function
 *   a4l_buf_commit_*() has to be called to complete the transfer().
 * - For DMA controllers which need to work with global counter (the
 *   transfered data count since the beginning of the acquisition),
 *   the functions a4l_buf_*_abs_*() have been made available.
 * - If no DMA controller is available, the driver has to perform the
 *   copy between the hardware component and the asynchronous
 *   buffer. In such cases, the functions a4l_buf_get() and
 *   a4l_buf_put() are useful.
 *
 * @{
 */

/**
 * @brief Update the absolute count of data sent from the device to
 * the buffer since the start of the acquisition and after the next
 * DMA shot
 *
 * The functions a4l_buf_prepare_(abs)put(),
 * a4l_buf_commit_(abs)put(), a4l_buf_prepare_(abs)get() and
 * a4l_buf_commit_(absg)et() have been made available for DMA
 * transfers. In such situations, no data copy is needed between the
 * Analogy buffer and the device as some DMA controller is in charge
 * of performing data shots from / to the Analogy buffer. However, some
 * pointers still have to be updated so as to monitor the tranfers.
 *
 * @param[in] subd Subdevice descriptor structure
 * @param[in] count The data count to be transferred during the next
 * DMA shot plus the data count which have been copied since the start
 * of the acquisition
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
int a4l_buf_prepare_absput(struct a4l_subdevice *subd, unsigned long count);
EXPORT_SYMBOL_GPL(a4l_buf_prepare_absput);

/**
 * @brief Set the absolute count of data which was sent from the
 * device to the buffer since the start of the acquisition and until
 * the last DMA shot
 *
 * The functions a4l_buf_prepare_(abs)put(),
 * a4l_buf_commit_(abs)put(), a4l_buf_prepare_(abs)get() and
 * a4l_buf_commit_(abs)get() have been made available for DMA
 * transfers. In such situations, no data copy is needed between the
 * Analogy buffer and the device as some DMA controller is in charge
 * of performing data shots from / to the Analogy buffer. However,
 * some pointers still have to be updated so as to monitor the
 * tranfers.
 *
 * @param[in] subd Subdevice descriptor structure
 * @param[in] count The data count transferred to the buffer during
 * the last DMA shot plus the data count which have been sent /
 * retrieved since the beginning of the acquisition
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
int a4l_buf_commit_absput(struct a4l_subdevice *subd, unsigned long count);
EXPORT_SYMBOL_GPL(a4l_buf_commit_absput);

/**
 * @brief Set the count of data which is to be sent to the buffer at
 * the next DMA shot
 *
 * The functions a4l_buf_prepare_(abs)put(),
 * a4l_buf_commit_(abs)put(), a4l_buf_prepare_(abs)get() and
 * a4l_buf_commit_(abs)get() have been made available for DMA
 * transfers. In such situations, no data copy is needed between the
 * Analogy buffer and the device as some DMA controller is in charge
 * of performing data shots from / to the Analogy buffer. However,
 * some pointers still have to be updated so as to monitor the
 * tranfers.
 *
 * @param[in] subd Subdevice descriptor structure
 * @param[in] count The data count to be transferred
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
int a4l_buf_prepare_put(struct a4l_subdevice *subd, unsigned long count);
EXPORT_SYMBOL_GPL(a4l_buf_prepare_put);

/**
 * @brief Set the count of data sent to the buffer during the last
 * completed DMA shots
 *
 * The functions a4l_buf_prepare_(abs)put(),
 * a4l_buf_commit_(abs)put(), a4l_buf_prepare_(abs)get() and
 * a4l_buf_commit_(abs)get() have been made available for DMA
 * transfers. In such situations, no data copy is needed between the
 * Analogy buffer and the device as some DMA controller is in charge
 * of performing data shots from / to the Analogy buffer. However,
 * some pointers still have to be updated so as to monitor the
 * tranfers.
 *
 * @param[in] subd Subdevice descriptor structure
 * @param[in] count The amount of data transferred
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
int a4l_buf_commit_put(struct a4l_subdevice *subd, unsigned long count);
EXPORT_SYMBOL_GPL(a4l_buf_commit_put);

/**
 * @brief Copy some data from the device driver to the buffer
 *
 * The function a4l_buf_put() must copy data coming from some
 * acquisition device to the Analogy buffer. This ring-buffer is an
 * intermediate area between the device driver and the user-space
 * program, which is supposed to recover the acquired data.
 *
 * @param[in] subd Subdevice descriptor structure
 * @param[in] bufdata The data buffer to copy into the Analogy buffer
 * @param[in] count The amount of data to copy
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
int a4l_buf_put(struct a4l_subdevice *subd, void *bufdata, unsigned long count);
EXPORT_SYMBOL_GPL(a4l_buf_put);

/**
 * @brief Update the absolute count of data sent from the buffer to
 * the device since the start of the acquisition and after the next
 * DMA shot
 *
 * The functions a4l_buf_prepare_(abs)put(),
 * a4l_buf_commit_(abs)put(), a4l_buf_prepare_(abs)get() and
 * a4l_buf_commit_(absg)et() have been made available for DMA
 * transfers. In such situations, no data copy is needed between the
 * Analogy buffer and the device as some DMA controller is in charge
 * of performing data shots from / to the Analogy buffer. However,
 * some pointers still have to be updated so as to monitor the
 * tranfers.
 *
 * @param[in] subd Subdevice descriptor structure
 * @param[in] count The data count to be transferred during the next
 * DMA shot plus the data count which have been copied since the start
 * of the acquisition
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
int a4l_buf_prepare_absget(struct a4l_subdevice *subd, unsigned long count);
EXPORT_SYMBOL_GPL(a4l_buf_prepare_absget);

/**
 * @brief Set the absolute count of data which was sent from the
 * buffer to the device since the start of the acquisition and until
 * the last DMA shot
 *
 * The functions a4l_buf_prepare_(abs)put(),
 * a4l_buf_commit_(abs)put(), a4l_buf_prepare_(abs)get() and
 * a4l_buf_commit_(abs)get() have been made available for DMA
 * transfers. In such situations, no data copy is needed between the
 * Analogy buffer and the device as some DMA controller is in charge
 * of performing data shots from / to the Analogy buffer. However,
 * some pointers still have to be updated so as to monitor the
 * tranfers.
 *
 * @param[in] subd Subdevice descriptor structure
 * @param[in] count The data count transferred to the device during
 * the last DMA shot plus the data count which have been sent since
 * the beginning of the acquisition
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
int a4l_buf_commit_absget(struct a4l_subdevice *subd, unsigned long count);
EXPORT_SYMBOL_GPL(a4l_buf_commit_absget);

/**
 * @brief Set the count of data which is to be sent from the buffer to
 * the device at the next DMA shot
 *
 * The functions a4l_buf_prepare_(abs)put(),
 * a4l_buf_commit_(abs)put(), a4l_buf_prepare_(abs)get() and
 * a4l_buf_commit_(abs)get() have been made available for DMA
 * transfers. In such situations, no data copy is needed between the
 * Analogy buffer and the device as some DMA controller is in charge
 * of performing data shots from / to the Analogy buffer. However,
 * some pointers still have to be updated so as to monitor the
 * tranfers.
 *
 * @param[in] subd Subdevice descriptor structure
 * @param[in] count The data count to be transferred
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
int a4l_buf_prepare_get(struct a4l_subdevice *subd, unsigned long count);
EXPORT_SYMBOL_GPL(a4l_buf_prepare_get);

/**
 * @brief Set the count of data sent from the buffer to the device
 * during the last completed DMA shots
 *
 * The functions a4l_buf_prepare_(abs)put(),
 * a4l_buf_commit_(abs)put(), a4l_buf_prepare_(abs)get() and
 * a4l_buf_commit_(abs)get() have been made available for DMA
 * transfers. In such situations, no data copy is needed between the
 * Analogy buffer and the device as some DMA controller is in charge
 * of performing data shots from / to the Analogy buffer. However,
 * some pointers still have to be updated so as to monitor the
 * tranfers.
 *
 * @param[in] subd Subdevice descriptor structure
 * @param[in] count The amount of data transferred
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
int a4l_buf_commit_get(struct a4l_subdevice *subd, unsigned long count);
EXPORT_SYMBOL_GPL(a4l_buf_commit_get);

/**
 * @brief Copy some data from the buffer to the device driver
 *
 * The function a4l_buf_get() must copy data coming from the Analogy
 * buffer to some acquisition device. This ring-buffer is an
 * intermediate area between the device driver and the user-space
 * program, which is supposed to provide the data to send to the
 * device.
 *
 * @param[in] subd Subdevice descriptor structure
 * @param[in] bufdata The data buffer to copy into the Analogy buffer
 * @param[in] count The amount of data to copy
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
int a4l_buf_get(struct a4l_subdevice *subd, void *bufdata, unsigned long count);
EXPORT_SYMBOL_GPL(a4l_buf_get);

/**
 * @brief Signal some event(s) to a user-space program involved in
 * some read / write operation
 *
 * The function a4l_buf_evt() is useful in many cases:
 * - To wake-up a process waiting for some data to read.
 * - To wake-up a process waiting for some data to write.
 * - To notify the user-process an error has occured during the
 *   acquistion.
 *
 * @param[in] subd Subdevice descriptor structure
 * @param[in] evts Some specific event to notify:
 * - A4L_BUF_ERROR to indicate some error has occured during the
 *   transfer
 * - A4L_BUF_EOA to indicate the acquisition is complete (this
 *   event is automatically set, it should not be used).
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
int a4l_buf_evt(struct a4l_subdevice *subd, unsigned long evts);
EXPORT_SYMBOL_GPL(a4l_buf_evt);

/**
 * @brief Get the data amount available in the Analogy buffer
 *
 * @param[in] subd Subdevice descriptor structure
 *
 * @return the amount of data available in the Analogy buffer.
 *
 */
unsigned long a4l_buf_count(struct a4l_subdevice *subd);
EXPORT_SYMBOL_GPL(a4l_buf_count);

#ifdef DOXYGEN_CPP		/* Only used for doxygen doc generation */

/**
 * @brief Get the current Analogy command descriptor
 *
 * @param[in] subd Subdevice descriptor structure
 *
 * @return the command descriptor.
 *
 */
struct a4l_cmd_desc *a4l_get_cmd(struct a4l_subdevice * subd);

#endif /* DOXYGEN_CPP */

/**
 * @brief Get the channel index according to its type
 *
 * @param[in] subd Subdevice descriptor structure
 *
 * @return the channel index.
 *
 */
int a4l_get_chan(struct a4l_subdevice *subd);
EXPORT_SYMBOL_GPL(a4l_get_chan);

/** @} */

/* --- IRQ handling section --- */

/**
 * @ingroup analogy_driver_facilities
 * @defgroup analogy_irq Interrupt management services
 * @{
 */

/**
 * @brief Get the interrupt number in use for a specific device
 *
 * @param[in] dev Device descriptor structure
 *
 * @return the line number used or A4L_IRQ_UNUSED if no interrupt
 * is registered.
 *
 */
unsigned int a4l_get_irq(struct a4l_device * dev);
EXPORT_SYMBOL_GPL(a4l_get_irq);

/**
 * @brief Register an interrupt handler for a specific device
 *
 * @param[in] dev Device descriptor structure
 * @param[in] irq Line number of the addressed IRQ
 * @param[in] handler Interrupt handler
 * @param[in] flags Registration flags:
 * - RTDM_IRQTYPE_SHARED: enable IRQ-sharing with other drivers
 *   (Warning: real-time drivers and non-real-time drivers cannot
 *   share an interrupt line).
 * - RTDM_IRQTYPE_EDGE: mark IRQ as edge-triggered (Warning: this flag
 *   is meaningless in RTDM-less context).
 * - A4L_IRQ_DISABLED: keep IRQ disabled when calling the action
 *   handler (Warning: this flag is ignored in RTDM-enabled
 *   configuration).
 * @param[in] cookie Pointer to be passed to the interrupt handler on
 * invocation
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
int a4l_request_irq(struct a4l_device * dev,
		       unsigned int irq,
		       a4l_irq_hdlr_t handler,
		       unsigned long flags, void *cookie);
EXPORT_SYMBOL_GPL(a4l_request_irq);

/**
 * @brief Release an interrupt handler for a specific device
 *
 * @param[in] dev Device descriptor structure
 * @param[in] irq Line number of the addressed IRQ
 *
 * @return 0 on success, otherwise negative error code.
 *
 */
int a4l_free_irq(struct a4l_device * dev, unsigned int irq);
EXPORT_SYMBOL_GPL(a4l_free_irq);

/** @} */

/* --- Misc section --- */

/**
 * @ingroup analogy_driver_facilities
 * @defgroup analogy_misc Misc services
 * @{
 */

/**
 * @brief Get the absolute time in nanoseconds
 *
 * @return the absolute time expressed in nanoseconds
 *
 */
unsigned long long a4l_get_time(void);
EXPORT_SYMBOL_GPL(a4l_get_time);

/** @} */
