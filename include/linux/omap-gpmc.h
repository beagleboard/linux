/*
 *  OMAP GPMC (General Purpose Memory Controller) defines
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/platform_data/gpmc-omap.h>

#define GPMC_CONFIG_WP		0x00000005

/* GPMC IRQENABLE/IRQSTATUS BIT defs */
#define GPMC_IRQENABLE_FIFOEVENT	BIT(0)
#define GPMC_IRQENABLE_TERMCOUNT	BIT(1)

enum gpmc_nand_irq {
	GPMC_NAND_IRQ_FIFOEVENT = 0,
	GPMC_NAND_IRQ_TERMCOUNT,
};

/**
 * gpmc_nand_ops - Interface between NAND and GPMC
 * @nand_irq_enable: enable the requested GPMC NAND interrupt event.
 * @nand_irq_disable: disable the requested GPMC NAND interrupt event.
 * @nand_irq_clear: clears the GPMC NAND interrupt event status.
 * @nand_irq_status: get the NAND interrupt event status.
 * @nand_write_buffer_empty: get the NAND write buffer empty status.
 */
struct gpmc_nand_ops {
	int (*nand_irq_enable)(enum gpmc_nand_irq irq);
	int (*nand_irq_disable)(enum gpmc_nand_irq irq);
	void (*nand_irq_clear)(enum gpmc_nand_irq irq);
	u32 (*nand_irq_status)(void);
	bool (*nand_writebuffer_empty)(void);
};

struct gpmc_nand_regs;

#if IS_ENABLED(CONFIG_OMAP_GPMC)
struct gpmc_nand_ops *gpmc_omap_get_nand_ops(struct gpmc_nand_regs *regs,
					     int cs);
#else
static inline gpmc_nand_ops *gpmc_omap_get_nand_ops(struct gpmc_nand_regs *regs,
						    int cs)
{
	return NULL;
}
#endif /* CONFIG_OMAP_GPMC */

/*--------------------------------*/

/* deprecated APIs */
#if IS_ENABLED(CONFIG_OMAP_GPMC)
void gpmc_update_nand_reg(struct gpmc_nand_regs *reg, int cs);
int gpmc_get_irq(void);
#else
static inline void gpmc_update_nand_reg(struct gpmc_nand_regs *reg, int cs)
{
	reg = NULL;
}

static inline int gpmc_get_irq(void)
{
	return -ENOTSUPP;
}
#endif /* CONFIG_OMAP_GPMC */
/*--------------------------------*/

extern int gpmc_calc_timings(struct gpmc_timings *gpmc_t,
			     struct gpmc_settings *gpmc_s,
			     struct gpmc_device_timings *dev_t);

struct device_node;

extern unsigned int gpmc_ticks_to_ns(unsigned int ticks);

extern void gpmc_cs_write_reg(int cs, int idx, u32 val);
extern int gpmc_calc_divider(unsigned int sync_clk);
extern int gpmc_cs_set_timings(int cs, const struct gpmc_timings *t,
			       const struct gpmc_settings *s);
extern int gpmc_cs_program_settings(int cs, struct gpmc_settings *p);
extern int gpmc_cs_request(int cs, unsigned long size, unsigned long *base);
extern void gpmc_cs_free(int cs);
extern int gpmc_configure(int cmd, int wval);
extern void gpmc_read_settings_dt(struct device_node *np,
				  struct gpmc_settings *p);

extern void omap3_gpmc_save_context(void);
extern void omap3_gpmc_restore_context(void);

struct gpmc_timings;
struct omap_nand_platform_data;
struct omap_onenand_platform_data;

#if IS_ENABLED(CONFIG_MTD_NAND_OMAP2)
extern int gpmc_nand_init(struct omap_nand_platform_data *d,
			  struct gpmc_timings *gpmc_t);
#else
static inline int gpmc_nand_init(struct omap_nand_platform_data *d,
				 struct gpmc_timings *gpmc_t)
{
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_MTD_ONENAND_OMAP2)
extern void gpmc_onenand_init(struct omap_onenand_platform_data *d);
#else
#define board_onenand_data	NULL
static inline void gpmc_onenand_init(struct omap_onenand_platform_data *d)
{
}
#endif
