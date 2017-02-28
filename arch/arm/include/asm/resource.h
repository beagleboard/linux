#ifndef _ARM_RESOURCE_H
#define _ARM_RESOURCE_H

/*
 * When FCSE is enabled, reduce the default stack size to 1MB, and maximum
 * to 16MB, the address space is only 32MB.
 */
#ifdef CONFIG_ARM_FCSE
#define _STK_LIM		(1024*1024)

#define _STK_LIM_MAX		(16*1024*1024)
#endif /* CONFIG_ARM_FCSE */

#include <asm-generic/resource.h>

#endif
