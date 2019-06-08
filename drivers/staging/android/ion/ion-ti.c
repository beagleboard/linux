// SPDX-License-Identifier: GPL-2.0
/*
 * Texas Instruments ION Driver
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/of.h>
#include <linux/of_address.h>

#include "ion.h"

static int __init ti_ion_init(void)
{
	struct device_node *node, *mem, *child = NULL;
	struct resource res;
	phys_addr_t base;
	size_t size;
	struct ion_heap *heap;
	u32 ion_heap_type, ion_chunk_size;
	int ret;

	if (!of_have_populated_dt())
		return -ENODEV;

	node = of_find_compatible_node(NULL, NULL, "ti,ion");
	if (!node)
		return -ENODEV;

	if (of_get_available_child_count(node) == 0) {
		pr_err("No child block node(s) found under ti,ion node\n");
		return -EINVAL;
	}

	while ((child = of_get_next_available_child(node, child))) {
		mem = of_parse_phandle(child, "memory-region", 0);
		if (!mem) {
			pr_err("No memory-region phandle for node %pOF\n", child);
			return -EINVAL;
		}

		ret = of_address_to_resource(mem, 0, &res);
		if (ret) {
			pr_err("Could not get resource address for node %pOF\n", child);
			return -EINVAL;
		}

		of_node_put(mem);

		base = res.start;
		size = resource_size(&res);

		ret = of_property_read_u32(child, "ion-heap-type", &ion_heap_type);
		if (ret) {
			pr_err("No ion-heap-type property for node %pOF\n", child);
			return ret;
		}

		switch (ion_heap_type) {
		case ION_HEAP_TYPE_CHUNK:
			ret = of_property_read_u32(child, "ion-chunk-size", &ion_chunk_size);
			if (ret) {
				pr_err("No ion-chunk-size property for node %pOF\n", child);
				return ret;
			}
			heap = ion_chunk_heap_create(base, size, ion_chunk_size);
			if (IS_ERR(heap)) {
				pr_err("Could not chunk heap for node %pOF\n", child);
				return PTR_ERR(heap);
			}
			break;
		case ION_HEAP_TYPE_CARVEOUT:
			heap = ion_carveout_heap_create(base, size);
			if (IS_ERR(heap)) {
				pr_err("Could not carveout heap for node %pOF\n", child);
				return PTR_ERR(heap);
			}
			break;
		default:
			pr_err("Unknown heap type for node %pOF\n", child);
			continue;
		}

		heap->name = child->name;

		ion_device_add_heap(heap);
	}

	return 0;
}
device_initcall(ti_ion_init);
