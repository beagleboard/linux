#ifndef __ARCH_OMAP_MMU_H
#define __ARCH_OMAP_MMU_H

#include <linux/device.h>
#include <linux/workqueue.h>

#define MMU_REVISION		0x00
#define MMU_SYSCONFIG		0x10
#define MMU_SYSSTATUS		0x14
#define MMU_IRQSTATUS		0x18
#define MMU_IRQENABLE		0x1c
#define MMU_WALKING_ST		0x40
#define MMU_CNTL		0x44
#define MMU_FAULT_AD		0x48
#define MMU_TTB			0x4c
#define MMU_LOCK		0x50
#define MMU_LD_TLB		0x54
#define MMU_CAM			0x58
#define MMU_RAM			0x5c
#define MMU_GFLUSH		0x60
#define MMU_FLUSH_ENTRY		0x64
#define MMU_READ_CAM		0x68
#define MMU_READ_RAM		0x6c
#define MMU_EMU_FAULT_AD	0x70

enum exmap_type {
	EXMAP_TYPE_MEM,
	EXMAP_TYPE_FB
};

enum omap_mmu_type {
	OMAP_MMU_DSP,
	OMAP_MMU_IVA1,
	OMAP_MMU_CAMERA,
};

struct exmap_tbl {
	unsigned int valid:1;
	unsigned int prsvd:1;
	int usecount;		/* reference count by mmap */
	enum exmap_type type;
	void *buf;		/* virtual address of the buffer,
				 * i.e. 0xc0000000 - */
	void *vadr;		/* DSP shadow space,
				 * i.e. 0xe0000000 - 0xe0ffffff */
	unsigned int order;
	struct {
		int prev;
		int next;
	} link;			/* grouping */
};

struct cam_ram_regset {
	union {
		struct {
			u16 cam_l;
			u16 cam_h;
		};

		u32 cam;
	};

	union {
		struct {
			u16 ram_l;
			u16 ram_h;
		};

		u32 ram;
	};
};

struct omap_mmu_tlb_lock {
	int base;
	int victim;
};

struct omap_mmu;
struct omap_mmu_tlb_entry;

struct omap_mmu_ops {
	int (*startup)(struct omap_mmu *mmu);
	void (*shutdown)(struct omap_mmu *mmu);

	/* TLB operations */
	void (*read_tlb)(struct omap_mmu *, struct cam_ram_regset *);
	void (*load_tlb)(struct omap_mmu *, struct cam_ram_regset *);
	ssize_t (*show)(struct omap_mmu *, char *, struct omap_mmu_tlb_lock *);

	/* CAM / RAM operations */
	struct cam_ram_regset *(*cam_ram_alloc)(struct omap_mmu_tlb_entry *);
	int (*cam_ram_valid)(struct cam_ram_regset *);
	unsigned long (*cam_va)(struct cam_ram_regset *);

	/* Memory operations */
	int (*mem_enable)(struct omap_mmu *, void *);
	int (*mem_disable)(struct omap_mmu *, void *);

	void (*interrupt)(struct omap_mmu *);

	/* PTE attribute operations */
	pgprot_t (*pte_get_attr)(struct omap_mmu_tlb_entry *);
};

struct omap_mmu {
	const char *name;
	unsigned long base;
	struct clk *clk;

	unsigned long membase, memsize;
	struct clk *memclk;

	enum omap_mmu_type type;

	struct device dev;

	struct rw_semaphore exmap_sem;
	struct exmap_tbl *exmap_tbl;

	unsigned int nr_tlb_entries;
	unsigned int nr_exmap_preserved;

	struct mm_struct *twl_mm;

	/* Size of virtual address space, in bits */
	unsigned int addrspace;

	/* Interrupt */
	unsigned int irq;
	unsigned long fault_address;
	struct work_struct irq_work;

	struct omap_mmu_ops *ops;
};

#define omap_mmu_internal_memory(mmu, addr)					\
	(likely(mmu->membase) && (((unsigned long)(addr) >= mmu->membase) &&	\
		 ((unsigned long)(addr) < mmu->membase + mmu->memsize)))

#define INIT_EXMAP_TBL_ENTRY(ent,b,v,typ,od)	\
do {						\
	(ent)->buf		= (b);		\
	(ent)->vadr		= (v);		\
	(ent)->valid		= 1;		\
	(ent)->prsvd		= 0;		\
	(ent)->usecount		= 0;		\
	(ent)->type		= (typ);	\
	(ent)->order		= (od);		\
	(ent)->link.next	= -1;		\
	(ent)->link.prev	= -1;		\
} while (0)

#define INIT_EXMAP_TBL_ENTRY_4KB_PRESERVED(ent,b,v)	\
do {							\
	(ent)->buf		= (b);			\
	(ent)->vadr		= (v);			\
	(ent)->valid		= 1;			\
	(ent)->prsvd		= 1;			\
	(ent)->usecount		= 0;			\
	(ent)->type		= EXMAP_TYPE_MEM;	\
	(ent)->order		= 0;			\
	(ent)->link.next	= -1;			\
	(ent)->link.prev	= -1;			\
} while (0)

#define omap_mmu_to_virt(mmu, db)	((void *)((mmu)->membase + (db)))
#define virt_to_omap_mmu(mmu, va) \
	(((unsigned long)(va) - (mmu)->membase))

/* arch/arm/plat-omap/mmu.c */
int omap_mmu_register(struct omap_mmu *mmu);
void omap_mmu_unregister(struct omap_mmu *mmu);

void omap_mmu_enable(struct omap_mmu *mmu, int reset);
void omap_mmu_disable(struct omap_mmu *mmu);

int omap_mmu_mem_enable(struct omap_mmu *mmu, void *addr);
void omap_mmu_mem_disable(struct omap_mmu *mmu, void *addr);

void omap_mmu_read_tlb(struct omap_mmu *mmu, struct omap_mmu_tlb_lock *lock,
		       struct cam_ram_regset *cr);

int omap_mmu_load_tlb_entry(struct omap_mmu *, struct omap_mmu_tlb_entry *);
int omap_mmu_clear_tlb_entry(struct omap_mmu *, unsigned long vadr);

int omap_mmu_load_pte_entry(struct omap_mmu *mmu,
			    struct omap_mmu_tlb_entry *entry);
int omap_mmu_clear_pte_entry(struct omap_mmu *mmu, unsigned long vadr);

int omap_mmu_kmem_reserve(struct omap_mmu *mmu, unsigned long size);
void omap_mmu_kmem_release(void);

unsigned long omap_mmu_virt_to_phys(struct omap_mmu *mmu, void *vadr,
				    size_t *len);

int omap_mmu_exmap(struct omap_mmu *mmu, unsigned long dspadr,
		   unsigned long padr, unsigned long size,
		   enum exmap_type type);
int omap_mmu_exunmap(struct omap_mmu *mmu, unsigned long dspadr);
void omap_mmu_exmap_flush(struct omap_mmu *mmu);
void omap_mmu_exmap_use(struct omap_mmu *mmu, void *vadr, size_t len);
void omap_mmu_exmap_unuse(struct omap_mmu *mmu, void *vadr, size_t len);

int exmap_set_armmmu(unsigned long virt, unsigned long phys, unsigned long size);
void exmap_clear_armmmu(unsigned long virt, unsigned long size);
void exmap_setup_preserved_mem_page(struct omap_mmu *mmu, void *buf,
				    unsigned long dspadr, int index);
void exmap_clear_mem_page(struct omap_mmu *mmu, unsigned long dspadr);
int exmap_valid(struct omap_mmu *mmu, void *vadr, size_t len);

/* To be obsolete for backward compatibility */
ssize_t __omap_mmu_mem_read(struct omap_mmu *mmu, char *buf, loff_t offset, size_t count);
ssize_t __omap_mmu_mem_write(struct omap_mmu *mmu, char *buf, loff_t offset, size_t count);

#endif /* __ARCH_OMAP_MMU_H */
