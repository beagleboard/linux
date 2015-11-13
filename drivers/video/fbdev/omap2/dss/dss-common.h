#ifndef __OMAP_DSS_COMMON_H
#define __OMAP_DSS_COMMON_H

void omapdss_set_is_initialized(bool set);
void dispc_set_ops(const struct dispc_ops *o);

struct device_node *dss_of_port_get_parent_device(struct device_node *port);
u32 dss_of_port_get_port_number(struct device_node *port);

#endif
