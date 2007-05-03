#ifndef __MACH_OMAP2_MMU_H
#define __MACH_OMAP2_MMU_H

#include <asm/arch/mmu.h>
#include <asm/io.h>

#define MMU_LOCK_BASE_MASK		(0x1f << 10)
#define MMU_LOCK_VICTIM_MASK		(0x1f << 4)

#define OMAP_MMU_IRQ_MULTIHITFAULT	0x00000010
#define OMAP_MMU_IRQ_TABLEWALKFAULT	0x00000008
#define OMAP_MMU_IRQ_EMUMISS		0x00000004
#define OMAP_MMU_IRQ_TRANSLATIONFAULT	0x00000002
#define OMAP_MMU_IRQ_TLBMISS		0x00000001

#define OMAP_MMU_CAM_VATAG_MASK		0xfffff000
#define OMAP_MMU_CAM_P			0x00000008
#define OMAP_MMU_CAM_V			0x00000004
#define OMAP_MMU_CAM_PAGESIZE_MASK	0x00000003
#define OMAP_MMU_CAM_PAGESIZE_1MB	0x00000000
#define OMAP_MMU_CAM_PAGESIZE_64KB	0x00000001
#define OMAP_MMU_CAM_PAGESIZE_4KB	0x00000002
#define OMAP_MMU_CAM_PAGESIZE_16MB	0x00000003

#define OMAP_MMU_RAM_PADDR_MASK		0xfffff000
#define OMAP_MMU_RAM_ENDIANNESS		0x00000200
#define OMAP_MMU_RAM_ENDIANNESS_BIG	0x00000200
#define OMAP_MMU_RAM_ENDIANNESS_LITTLE	0x00000000
#define OMAP_MMU_RAM_ELEMENTSIZE_MASK	0x00000180
#define OMAP_MMU_RAM_ELEMENTSIZE_8	0x00000000
#define OMAP_MMU_RAM_ELEMENTSIZE_16	0x00000080
#define OMAP_MMU_RAM_ELEMENTSIZE_32	0x00000100
#define OMAP_MMU_RAM_ELEMENTSIZE_NONE	0x00000180
#define OMAP_MMU_RAM_MIXED		0x00000040

#define IOMAP_VAL	0x3f

#define omap_dsp_request_mem()	do { } while (0)
#define omap_dsp_release_mem()	do { } while (0)

#define INIT_TLB_ENTRY(ent,v,p,ps)				\
do {								\
	(ent)->va	= (v);					\
	(ent)->pa	= (p);					\
	(ent)->pgsz	= (ps);					\
	(ent)->prsvd	= 0;					\
	(ent)->endian	= OMAP_MMU_RAM_ENDIANNESS_LITTLE;	\
	(ent)->elsz	= OMAP_MMU_RAM_ELEMENTSIZE_16;		\
	(ent)->mixed	= 0;					\
	(ent)->tlb	= 1;					\
} while (0)

#define INIT_TLB_ENTRY_4KB_PRESERVED(ent,v,p) \
do {								\
	(ent)->va	= (v);					\
	(ent)->pa	= (p);					\
	(ent)->pgsz	= OMAP_MMU_CAM_PAGESIZE_4KB;		\
	(ent)->prsvd	= OMAP_MMU_CAM_P;			\
	(ent)->endian	= OMAP_MMU_RAM_ENDIANNESS_LITTLE;	\
	(ent)->elsz	= OMAP_MMU_RAM_ELEMENTSIZE_16;		\
	(ent)->mixed	= 0;					\
} while (0)

#define INIT_TLB_ENTRY_4KB_ES32_PRESERVED(ent,v,p)		\
do {								\
	(ent)->va	= (v);					\
	(ent)->pa	= (p);					\
	(ent)->pgsz	= OMAP_MMU_CAM_PAGESIZE_4KB;		\
	(ent)->prsvd	= OMAP_MMU_CAM_P;			\
	(ent)->endian	= OMAP_MMU_RAM_ENDIANNESS_LITTLE;	\
	(ent)->elsz	= OMAP_MMU_RAM_ELEMENTSIZE_32;		\
	(ent)->mixed	= 0;					\
} while (0)

extern struct omap_mmu_ops omap2_mmu_ops;

struct omap_mmu_tlb_entry {
	unsigned long va;
	unsigned long pa;
	unsigned int pgsz, prsvd, valid;

	u32 endian, elsz, mixed;
	unsigned int tlb;
};

static inline unsigned long
omap_mmu_read_reg(struct omap_mmu *mmu, unsigned long reg)
{
	return __raw_readl(mmu->base + reg);
}

static inline void omap_mmu_write_reg(struct omap_mmu *mmu,
			       unsigned long val, unsigned long reg)
{
	__raw_writel(val, mmu->base + reg);
}
static inline void omap_mmu_itack(struct omap_mmu *mmu)
{
}
#endif /* __MACH_OMAP2_MMU_H */
