/*
 *  drivers/media/video/omap/camera_hw_if.h
 *
 * Copyright (C) 2004 Texas Instruments, Inc. 
 * 
 * Camera interface to OMAP camera capture drivers
 * Camera interface hardware driver should implement this interface
 *
 * This package is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. 
 * 
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED 
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 */
 
#ifndef OMAP_CAMERA_HW_IF_H
#define OMAP_CAMERA_HW_IF_H

#define LEN_HW_IF_NAME		31

struct sgdma_state;

struct camera_hardware {
	unsigned int version;  //version of camera driver module
	char name[LEN_HW_IF_NAME + 1];

	void *(*init)(void);
	int (*cleanup)(void *);

	int (*open)(void *);  /* acquire h/w resources (irq,DMA), etc. */
	int (*close)(void *); /* free h/w resources, stop i/f */

	int (*enable)(void *);
	int (*disable)(void *);

	int (*abort)(void *);

	int (*set_xclk)(int, void *);

	int (*init_dma)(void *);
	int (*start_dma)(struct sgdma_state *, void (*)(void *arg1, void *arg2),
			void *, void *, void *);
	int (*finish_dma)(void *);
};
 
#endif /* OMAP_CAMERA_HW_IF_H */
