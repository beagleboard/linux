// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 *  Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
 */

int xudma_navss_psil_pair(struct udma_dev *ud, u32 src_thread, u32 dst_thread)
{
	return navss_psil_pair(ud, src_thread, dst_thread);
}
EXPORT_SYMBOL(xudma_navss_psil_pair);

int xudma_navss_psil_unpair(struct udma_dev *ud, u32 src_thread, u32 dst_thread)
{
	return navss_psil_unpair(ud, src_thread, dst_thread);
}
EXPORT_SYMBOL(xudma_navss_psil_unpair);

struct udma_dev *of_xudma_dev_get(struct device_node *np, const char *property)
{
	struct device_node *udma_node = np;
	struct platform_device *pdev;
	struct udma_dev *ud;

	if (property) {
		udma_node = of_parse_phandle(np, property, 0);
		if (!udma_node) {
			pr_err("UDMA node is not found\n");
			return ERR_PTR(-ENODEV);
		}
	}

	pdev = of_find_device_by_node(udma_node);
	if (!pdev) {
		pr_err("UDMA device not found\n");
		return ERR_PTR(-EPROBE_DEFER);
	}

	of_node_put(udma_node);

	ud = platform_get_drvdata(pdev);
	if (!ud) {
		pr_err("UDMA has not been proped\n");
		return ERR_PTR(-EPROBE_DEFER);
	}

	pm_runtime_get_sync(&pdev->dev);

	return ud;
}
EXPORT_SYMBOL(of_xudma_dev_get);

void xudma_dev_put(struct udma_dev *ud)
{
	pm_runtime_put_sync(ud->ddev.dev);
}
EXPORT_SYMBOL(xudma_dev_put);

u32 xudma_dev_get_psil_base(struct udma_dev *ud)
{
	return ud->psil_base;
}
EXPORT_SYMBOL(xudma_dev_get_psil_base);

struct udma_tisci_rm *xudma_dev_get_tisci_rm(struct udma_dev *ud)
{
	return &ud->tisci_rm;
}
EXPORT_SYMBOL(xudma_dev_get_tisci_rm);

int xudma_reserve_rflow_range(struct udma_dev *ud, int from, int cnt)
{
	return __udma_reserve_rflow_range(ud, from, cnt);
}
EXPORT_SYMBOL(xudma_reserve_rflow_range);

int xudma_free_rflow_range(struct udma_dev *ud, int from, int cnt)
{
	return __udma_free_rflow_range(ud, from, cnt);
}
EXPORT_SYMBOL(xudma_free_rflow_range);

#define XUDMA_GET_PUT_RESOURCE(res)					\
struct udma_##res *xudma_##res##_get(struct udma_dev *ud, int id)	\
{									\
	return __udma_reserve_##res(ud, false, id);			\
}									\
EXPORT_SYMBOL(xudma_##res##_get);					\
									\
void xudma_##res##_put(struct udma_dev *ud, struct udma_##res *p)	\
{									\
	clear_bit(p->id, ud->res##_map);				\
}									\
EXPORT_SYMBOL(xudma_##res##_put)
XUDMA_GET_PUT_RESOURCE(tchan);
XUDMA_GET_PUT_RESOURCE(rchan);
XUDMA_GET_PUT_RESOURCE(rflow);

#define XUDMA_GET_RESOURCE_ID(res)					\
int xudma_##res##_get_id(struct udma_##res *p)				\
{									\
	return p->id;							\
}									\
EXPORT_SYMBOL(xudma_##res##_get_id)
XUDMA_GET_RESOURCE_ID(tchan);
XUDMA_GET_RESOURCE_ID(rchan);
XUDMA_GET_RESOURCE_ID(rflow);

/* Exported register access functions */
#define XUDMA_RT_IO_FUNCTIONS(res)					\
u32 xudma_##res##rt_read(struct udma_##res *p, int reg)			\
{									\
	return udma_##res##rt_read(p, reg);				\
}									\
EXPORT_SYMBOL(xudma_##res##rt_read);					\
									\
void xudma_##res##rt_write(struct udma_##res *p, int reg, u32 val)	\
{									\
	udma_##res##rt_write(p, reg, val);				\
}									\
EXPORT_SYMBOL(xudma_##res##rt_write)
XUDMA_RT_IO_FUNCTIONS(tchan);
XUDMA_RT_IO_FUNCTIONS(rchan);
