#ifndef __DRVR_H__
#define __DRVR_H__

#include <linux/cdev.h>

enum drvr_type{
	prog,
	mem
};

struct drvr_prog{
	struct i2c_client * i2c_io;
};


struct drvr_mem{
	unsigned short * base_addr;
	unsigned short * virt_addr;
	unsigned char * dma_buf;
	int dma_chan;
};

union drvr_data{
	struct drvr_prog prog;
	struct drvr_mem mem;
};

struct drvr_device{
	enum drvr_type type;
	union drvr_data data;
	struct cdev cdev;
	unsigned char opened;
};

#endif
