/*
 * OMAP-1510 camera interface
 *
 * FIXME: This will go to same directory with the camera driver
 */
#define CAMERA_BASE		0xfffb6800
#define CAM_CTRLCLOCK_REG	(CAMERA_BASE + 0x00)
#define CAM_IT_STATUS_REG	(CAMERA_BASE + 0x04)
#define CAM_MODE_REG		(CAMERA_BASE + 0x08)
#define CAM_STATUS_REG		(CAMERA_BASE + 0x0C)
#define CAM_CAMDATA_REG		(CAMERA_BASE + 0x10)
#define CAM_GPIO_REG		(CAMERA_BASE + 0x14)
#define CAM_PEAK_CTR_REG	(CAMERA_BASE + 0x18)

#ifndef __ASSEMBLY__
typedef struct {
	__u32 ctrlclock;	/* 00 */
	__u32 it_status;	/* 04 */
	__u32 mode;		/* 08 */
	__u32 status;		/* 0C */
	__u32 camdata;		/* 10 */
	__u32 gpio;		/* 14 */
	__u32 peak_counter;	/* 18 */
} camera_regs_t;
#endif

/* CTRLCLOCK bit shifts */
#define FOSCMOD_BIT		0
#define FOSCMOD_MASK		(0x7 << FOSCMOD_BIT)
#define	  FOSCMOD_12MHz		0x0
#define	  FOSCMOD_6MHz		0x2
#define	  FOSCMOD_9_6MHz	0x4
#define	  FOSCMOD_24MHz		0x5
#define	  FOSCMOD_8MHz		0x6
#define POLCLK			(1<<3)
#define CAMEXCLK_EN		(1<<4)
#define MCLK_EN			(1<<5)
#define DPLL_EN			(1<<6)
#define LCLK_EN			(1<<7)

/* IT_STATUS bit shifts */
#define V_UP			(1<<0)
#define V_DOWN			(1<<1)
#define H_UP			(1<<2)
#define H_DOWN			(1<<3)
#define FIFO_FULL		(1<<4)
#define DATA_XFER		(1<<5)

/* MODE bit shifts */
#define CAMOSC			(1<<0)
#define IMGSIZE_BIT		1
#define IMGSIZE_MASK		(0x3 << IMGSIZE_BIT)
#define	  IMGSIZE_CIF		(0x0 << IMGSIZE_BIT)	   /* 352x288 */
#define	  IMGSIZE_QCIF		(0x1 << IMGSIZE_BIT)	   /* 176x144 */
#define	  IMGSIZE_VGA		(0x2 << IMGSIZE_BIT)	   /* 640x480 */
#define	  IMGSIZE_QVGA		(0x3 << IMGSIZE_BIT)	   /* 320x240 */
#define ORDERCAMD		(1<<3)
#define EN_V_UP			(1<<4)
#define EN_V_DOWN		(1<<5)
#define EN_H_UP			(1<<6)
#define EN_H_DOWN		(1<<7)
#define EN_DMA			(1<<8)
#define THRESHOLD		(1<<9)
#define THRESHOLD_BIT		9
#define THRESHOLD_MASK		(0x7f<<9)
#define EN_NIRQ			(1<<16)
#define EN_FIFO_FULL		(1<<17)
#define RAZ_FIFO		(1<<18)

/* STATUS bit shifts */
#define VSTATUS			(1<<0)
#define HSTATUS			(1<<1)

/* GPIO bit shifts */
#define CAM_RST			(1<<0)
