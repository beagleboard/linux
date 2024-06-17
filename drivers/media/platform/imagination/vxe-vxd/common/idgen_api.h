/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ID generation manager API.
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Amit Makani <amit.makani@ti.com>
 *
 * Re-written for upstreamimg
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 *	Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 */
#ifndef __IDGENAPI_H__
#define __IDGENAPI_H__

#include <linux/types.h>

#include "img_errors.h"

/*
 * This function is used to create Id generation context.
 * NOTE: Should only be called once to setup the context structure.
 * NOTE: The client is responsible for providing thread/process safe locks on
 * the context structure to maintain coherence.
 */
int idgen_createcontext(unsigned int maxid, unsigned int blksize,
			int incid, void **idgenhandle);

/*
 * This function is used to destroy an Id generation context.  This function
 * discards any handle blocks associated with the context.
 * NOTE: The client is responsible for providing thread/process safe locks on
 * the context structure to maintain coherence.
 */
int idgen_destroycontext(void *idgenhandle);

/*
 * This function is used to associate a handle with an Id.
 * NOTE: The client is responsible for providing thread/process safe locks on
 * the context structure to maintain coherency.
 */
int idgen_allocid(void *idgenhandle, void *handle, unsigned int *id);

/*
 * This function is used to free an Id.
 * NOTE: The client is responsible for providing thread/process safe locks on
 * the context structure to maintain coherency.
 */
int idgen_freeid(void *idgenhandle, unsigned int id);

/*
 * This function is used to get the handle associated with an Id.
 * NOTE: The client is responsible for providing thread/process safe locks on
 * the context structure to maintain coherency.
 */
int idgen_gethandle(void *idgenhandle, unsigned int id, void **handle);
#endif /* __IDGENAPI_H__ */
