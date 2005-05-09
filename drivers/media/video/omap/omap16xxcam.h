/*
 *  drivers/media/video/omap/omap16xxcam.h
 *
 * Copyright (C) 2004 Texas Instruments, Inc. 
 * 
 * This package is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. 
 * 
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED 
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 */
 
#ifndef OMAP_16XX_CAM_H
#define OMAP_16XX_CAM_H

#define DMA_ELEM_SIZE   4
#define FIFO_TRIGGER_LVL (32)

/*
 * ---------------------------------------------------------------------------
 *  OMAP1610 Camera Interface
 * ---------------------------------------------------------------------------
 */

#ifdef CONFIG_MACH_OMAP_H3
#define CAMERA_BASE          (0x2007d800)
#else
#define CAMERA_BASE          (IO_PHYS + 0x6800)
#endif

#define CAM_CTRLCLOCK_REG    (CAMERA_BASE + 0x00)
#define CAM_IT_STATUS_REG    (CAMERA_BASE + 0x04)
#define CAM_MODE_REG         (CAMERA_BASE + 0x08)
#define CAM_STATUS_REG       (CAMERA_BASE + 0x0C)
#define CAM_CAMDATA_REG      (CAMERA_BASE + 0x10)
#define CAM_GPIO_REG         (CAMERA_BASE + 0x14)
#define CAM_PEAK_CTR_REG     (CAMERA_BASE + 0x18)
#define CAMERA_IOSIZE        0x1C

/* CTRLCLOCK bit shifts */
#define FOSCMOD_BIT    		0
#define FOSCMOD_MASK   		(0x7 << FOSCMOD_BIT)
#define FOSCMOD_12MHz		0x0
#define	FOSCMOD_6MHz		0x2
#define	FOSCMOD_9_6MHz		0x4
#define	FOSCMOD_24MHz		0x5
#define	FOSCMOD_8MHz		0x6
#define	FOSCMOD_TC2_CK2		0x3
#define	FOSCMOD_TC2_CK3    	0x1
#define	FOSCMOD_TC2_CK4     	0x5
#define	FOSCMOD_TC2_CK8     	0x0
#define	FOSCMOD_TC2_CK10     	0x4
#define	FOSCMOD_TC2_CK12     	0x6
#define	FOSCMOD_TC2_CK16     	0x2
#define	POLCLK         		(1<<3)
#define	CAMEXCLK_EN    		(1<<4)
#define	MCLK_EN        		(1<<5)
#define	DPLL_EN        		(1<<6)
#define	LCLK_EN        		(1<<7)

/* IT_STATUS bit shifts */
#define V_UP           (1<<0)
#define V_DOWN         (1<<1)
#define H_UP           (1<<2)
#define H_DOWN         (1<<3)
#define FIFO_FULL      (1<<4)
#define DATA_XFER      (1<<5)

/* MODE bit shifts */
#define CAMOSC         (1<<0)
#define IMGSIZE_BIT    1
#define IMGSIZE_MASK   (0x3 << IMGSIZE_BIT)
#define	IMGSIZE_CIF      (0x0 << IMGSIZE_BIT)    /* 352x288 */
#define	IMGSIZE_QCIF     (0x1 << IMGSIZE_BIT)    /* 176x144 */
#define	IMGSIZE_VGA      (0x2 << IMGSIZE_BIT)    /* 640x480 */
#define	IMGSIZE_QVGA     (0x3 << IMGSIZE_BIT)    /* 320x240 */
#define ORDERCAMD      (1<<3)
#define EN_V_UP        (1<<4)
#define EN_V_DOWN      (1<<5)
#define EN_H_UP        (1<<6)
#define EN_H_DOWN      (1<<7)
#define EN_DMA         (1<<8)
#define THRESHOLD      (1<<9)
#define THRESHOLD_BIT  9
#define THRESHOLD_MASK (0x7f<<9)
#define EN_NIRQ        (1<<16)
#define EN_FIFO_FULL   (1<<17)
#define RAZ_FIFO       (1<<18)

/* STATUS bit shifts */
#define VSTATUS        (1<<0)
#define HSTATUS        (1<<1)

/* GPIO bit shifts */
#define CAM_RST        (1<<0)


#define XCLK_6MHZ     6000000
#define XCLK_8MHZ     8000000
#define XCLK_9_6MHZ   9000000
#define XCLK_12MHZ   12000000
#define XCLK_24MHZ   24000000

#endif /* OMAP_16XX_CAM_H */
