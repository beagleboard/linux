#ifndef __MACH_OMAP1_MMU_H
#define __MACH_OMAP1_MMU_H

#include <asm/arch/mmu.h>
#include <asm/io.h>

#define MMU_LOCK_BASE_MASK		(0x3f << 10)
#define MMU_LOCK_VICTIM_MASK		(0x3f << 4)

#define OMAP_MMU_BASE			(0xfffed200)
#define OMAP_MMU_PREFETCH		(OMAP_MMU_BASE + 0x00)
#define OMAP_MMU_WALKING_ST		(OMAP_MMU_BASE + 0x04)
#define OMAP_MMU_CNTL			(OMAP_MMU_BASE + 0x08)
#define OMAP_MMU_FAULT_AD_H		(OMAP_MMU_BASE + 0x0c)
#define OMAP_MMU_FAULT_AD_L		(OMAP_MMU_BASE + 0x10)
#define OMAP_MMU_FAULT_ST		(OMAP_MMU_BASE + 0x14)
#define OMAP_MMU_IT_ACK			(OMAP_MMU_BASE + 0x18)
#define OMAP_MMU_TTB_H			(OMAP_MMU_BASE + 0x1c)
#define OMAP_MMU_TTB_L			(OMAP_MMU_BASE + 0x20)
#define OMAP_MMU_LOCK			(OMAP_MMU_BASE + 0x24)
#define OMAP_MMU_LD_TLB			(OMAP_MMU_BASE + 0x28)
#define OMAP_MMU_CAM_H			(OMAP_MMU_BASE + 0x2c)
#define OMAP_MMU_CAM_L			(OMAP_MMU_BASE + 0x30)
#define OMAP_MMU_RAM_H			(OMAP_MMU_BASE + 0x34)
#define OMAP_MMU_RAM_L			(OMAP_MMU_BASE + 0x38)
#define OMAP_MMU_GFLUSH			(OMAP_MMU_BASE + 0x3c)
#define OMAP_MMU_FLUSH_ENTRY		(OMAP_MMU_BASE + 0x40)
#define OMAP_MMU_READ_CAM_H		(OMAP_MMU_BASE + 0x44)
#define OMAP_MMU_READ_CAM_L		(OMAP_MMU_BASE + 0x48)
#define OMAP_MMU_READ_RAM_H		(OMAP_MMU_BASE + 0x4c)
#define OMAP_MMU_READ_RAM_L		(OMAP_MMU_BASE + 0x50)

#define OMAP_MMU_CNTL_BURST_16MNGT_EN	0x0020
#define OMAP_MMU_CNTL_WTL_EN		0x0004
#define OMAP_MMU_CNTL_MMU_EN		0x0002
#define OMAP_MMU_CNTL_RESET_SW		0x0001

#define OMAP_MMU_FAULT_AD_H_DP		0x0100
#define OMAP_MMU_FAULT_AD_H_ADR_MASK	0x00ff

#define OMAP_MMU_FAULT_ST_PREF		0x0008
#define OMAP_MMU_FAULT_ST_PERM		0x0004
#define OMAP_MMU_FAULT_ST_TLB_MISS	0x0002
#define OMAP_MMU_FAULT_ST_TRANS		0x0001

#define OMAP_MMU_IT_ACK_IT_ACK		0x0001

#define OMAP_MMU_CAM_H_VA_TAG_H_MASK		0x0003

#define OMAP_MMU_CAM_L_VA_TAG_L1_MASK		0xc000
#define OMAP_MMU_CAM_L_VA_TAG_L2_MASK_1MB	0x0000
#define OMAP_MMU_CAM_L_VA_TAG_L2_MASK_64KB	0x3c00
#define OMAP_MMU_CAM_L_VA_TAG_L2_MASK_4KB	0x3fc0
#define OMAP_MMU_CAM_L_VA_TAG_L2_MASK_1KB	0x3ff0
#define OMAP_MMU_CAM_L_P			0x0008
#define OMAP_MMU_CAM_L_V			0x0004
#define OMAP_MMU_CAM_L_PAGESIZE_MASK		0x0003
#define OMAP_MMU_CAM_L_PAGESIZE_1MB		0x0000
#define OMAP_MMU_CAM_L_PAGESIZE_64KB		0x0001
#define OMAP_MMU_CAM_L_PAGESIZE_4KB		0x0002
#define OMAP_MMU_CAM_L_PAGESIZE_1KB		0x0003

#define OMAP_MMU_CAM_P			OMAP_MMU_CAM_L_P
#define OMAP_MMU_CAM_V			OMAP_MMU_CAM_L_V
#define OMAP_MMU_CAM_PAGESIZE_MASK	OMAP_MMU_CAM_L_PAGESIZE_MASK
#define OMAP_MMU_CAM_PAGESIZE_1MB	OMAP_MMU_CAM_L_PAGESIZE_1MB
#define OMAP_MMU_CAM_PAGESIZE_64KB	OMAP_MMU_CAM_L_PAGESIZE_64KB
#define OMAP_MMU_CAM_PAGESIZE_4KB	OMAP_MMU_CAM_L_PAGESIZE_4KB
#define OMAP_MMU_CAM_PAGESIZE_1KB	OMAP_MMU_CAM_L_PAGESIZE_1KB
#define OMAP_MMU_CAM_PAGESIZE_16MB	-1 /* unused in omap1 */

#define OMAP_MMU_RAM_L_RAM_LSB_MASK	0xfc00
#define OMAP_MMU_RAM_L_AP_MASK		0x0300
#define OMAP_MMU_RAM_L_AP_NA		0x0000
#define OMAP_MMU_RAM_L_AP_RO		0x0200
#define OMAP_MMU_RAM_L_AP_FA		0x0300

#define OMAP_MMU_LD_TLB_RD		0x0002

#define INIT_TLB_ENTRY(ent,v,p,ps)			\
do {							\
	(ent)->va	= (v);				\
	(ent)->pa	= (p);				\
	(ent)->pgsz	= (ps);				\
	(ent)->prsvd	= 0;				\
	(ent)->ap	= OMAP_MMU_RAM_L_AP_FA;		\
	(ent)->tlb	= 1;				\
} while (0)

#define INIT_TLB_ENTRY_4KB_PRESERVED(ent,v,p)		\
do {							\
	(ent)->va	= (v);				\
	(ent)->pa	= (p);				\
	(ent)->pgsz	= OMAP_MMU_CAM_PAGESIZE_4KB;	\
	(ent)->prsvd	= OMAP_MMU_CAM_P;		\
	(ent)->ap	= OMAP_MMU_RAM_L_AP_FA;		\
} while (0)

extern struct omap_mmu_ops omap1_mmu_ops;

struct omap_mmu_tlb_entry {
	unsigned long va;
	unsigned long pa;
	unsigned int pgsz, prsvd, valid;

	u16 ap;
	unsigned int tlb;
};

static inline unsigned short
omap_mmu_read_reg(struct omap_mmu *mmu, unsigned long reg)
{
	return __raw_readw(mmu->base + reg);
}

static inline void omap_mmu_write_reg(struct omap_mmu *mmu,
			       unsigned short val, unsigned long reg)
{
	__raw_writew(val, mmu->base + reg);
}

int omap_dsp_request_mem(void);
void omap_dsp_release_mem(void);

static inline void omap_mmu_itack(struct omap_mmu *mmu)
{
	omap_mmu_write_reg(mmu, OMAP_MMU_IT_ACK_IT_ACK, OMAP_MMU_IT_ACK);
}

#endif /* __MACH_OMAP1_MMU_H */
