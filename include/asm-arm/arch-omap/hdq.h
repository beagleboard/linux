/*
 * include/asm-arm/arch-omap/hdq.h
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#ifndef __ASM_OMAP2_HDQ_H__
#define __ASM_OMAP2_HDQ_H__

/* Note the following APIs have to be called in a process context */
int omap_hdq_get(void); /* request the HDQ block */
int omap_hdq_put(void); /* release the HDQ block */
int omap_hdq_break(void); /* reset the slave by sending it a break pulse */

int omap_hdq_reset(void); /* reset the HDQ block */

#endif
