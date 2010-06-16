#ifndef _ASM_ARM_LMB_H
#define _ASM_ARM_LMB_H

#ifdef CONFIG_MMU
extern phys_addr_t lowmem_end_addr;
#define LMB_REAL_LIMIT	lowmem_end_addr
#else
#define LMB_REAL_LIMIT	0
#endif

struct meminfo;
struct machine_desc;

extern void arm_lmb_init(struct meminfo *, struct machine_desc *);

#endif
