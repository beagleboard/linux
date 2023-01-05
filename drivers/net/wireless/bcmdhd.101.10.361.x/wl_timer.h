#ifndef _wl_timer_
#define _wl_timer_
s32 wl_timer_attach(struct net_device *net);
void wl_timer_dettach(dhd_pub_t *dhdp);
void wl_timer_register(struct net_device *net, timer_list_compat_t *timer, void *cb_func);
void wl_timer_deregister(struct net_device *net, timer_list_compat_t *timer);
void wl_timer_mod(dhd_pub_t *dhd, timer_list_compat_t *timer, uint32 tmo_ms);
#endif
