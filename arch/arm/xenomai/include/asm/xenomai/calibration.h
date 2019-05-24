/*
 * Copyright (C) 2001,2002,2003,2004,2005 Philippe Gerum <rpm@xenomai.org>.
 *
 * ARM port
 *   Copyright (C) 2005 Stelian Pop
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
 * along with Xenomai; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */
#ifndef _COBALT_ARM_ASM_CALIBRATION_H
#define _COBALT_ARM_ASM_CALIBRATION_H

unsigned int omap_rev(void);
#define cpu_is_omap44xx() ((omap_rev() & 0xff) == 0x44)

static inline void xnarch_get_latencies(struct xnclock_gravity *p)
{
	unsigned int ulat;
#if CONFIG_XENO_OPT_TIMING_SCHEDLAT != 0
	ulat = CONFIG_XENO_OPT_TIMING_SCHEDLAT;
#elif defined(CONFIG_ARCH_AT91RM9200)
	ulat = 8500;
#elif defined(CONFIG_ARCH_AT91SAM9263)
	ulat = 11000;
#elif defined(CONFIG_SOC_IMX6Q)
	ulat = 6000;
#elif defined(CONFIG_ARCH_MX51)
	ulat = 5000;
#elif defined(CONFIG_ARCH_MX53)
	ulat = 5000;
#elif defined(CONFIG_ARCH_MX6)
	ulat = 2000;
#elif defined(CONFIG_SOC_IMX7)
	ulat = 2000;
#elif defined(CONFIG_SOC_LS1021A)
	ulat = 2800;
#elif defined(CONFIG_ARCH_OMAP)
	ulat = cpu_is_omap44xx() ? 2500 : 5000;
#elif defined(CONFIG_ARCH_STI)
	ulat = 6000;
#elif defined(CONFIG_ARCH_SOCFPGA)
	ulat = 4500;
#else
	ulat = 9500;	/* XXX sane? */
#endif
	p->user = xnclock_ns_to_ticks(&nkclock, ulat);
	p->kernel = xnclock_ns_to_ticks(&nkclock, CONFIG_XENO_OPT_TIMING_KSCHEDLAT);
	p->irq = xnclock_ns_to_ticks(&nkclock, CONFIG_XENO_OPT_TIMING_IRQLAT);
}

#endif /* !_COBALT_ARM_ASM_CALIBRATION_H */
