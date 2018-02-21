/*
 * Copyright (C) 2009 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
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
#ifndef _COBALT_ASM_GENERIC_PCI_IDS_H
#define _COBALT_ASM_GENERIC_PCI_IDS_H

#include <linux/pci_ids.h>

/* SMI */
#ifndef PCI_DEVICE_ID_INTEL_ESB2_0
#define PCI_DEVICE_ID_INTEL_ESB2_0 0x2670
#endif
#ifndef PCI_DEVICE_ID_INTEL_ICH7_0
#define PCI_DEVICE_ID_INTEL_ICH7_0 0x27b8
#endif
#ifndef PCI_DEVICE_ID_INTEL_ICH7_1
#define PCI_DEVICE_ID_INTEL_ICH7_1 0x27b9
#endif
#ifndef PCI_DEVICE_ID_INTEL_ICH8_4
#define PCI_DEVICE_ID_INTEL_ICH8_4 0x2815
#endif
#ifndef PCI_DEVICE_ID_INTEL_ICH9_1
#define PCI_DEVICE_ID_INTEL_ICH9_1 0x2917
#endif
#ifndef PCI_DEVICE_ID_INTEL_ICH9_5
#define PCI_DEVICE_ID_INTEL_ICH9_5 0x2919
#endif
#ifndef PCI_DEVICE_ID_INTEL_ICH10_1
#define PCI_DEVICE_ID_INTEL_ICH10_1 0x3a16
#endif
#ifndef PCI_DEVICE_ID_INTEL_PCH_LPC_MIN
#define PCI_DEVICE_ID_INTEL_PCH_LPC_MIN 0x3b00
#endif

/* RTCAN */
#ifndef PCI_VENDOR_ID_ESDGMBH
#define PCI_VENDOR_ID_ESDGMBH 0x12fe
#endif
#ifndef PCI_DEVICE_ID_PLX_9030
#define PCI_DEVICE_ID_PLX_9030 0x9030
#endif
#ifndef PCI_DEVICE_ID_PLX_9056
#define PCI_DEVICE_ID_PLX_9056 0x9056
#endif

#endif /* _COBALT_ASM_GENERIC_PCI_IDS_H */
