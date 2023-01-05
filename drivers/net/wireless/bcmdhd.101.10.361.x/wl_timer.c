#include <wl_android.h>
#ifdef WL_TIMER

#define TIMER_ERROR(name, arg1, args...) \
	do { \
		if (android_msg_level & ANDROID_ERROR_LEVEL) { \
			printf("[%s] TIMER-ERROR) %s : " arg1, name, __func__, ## args); \
		} \
	} while (0)
#define TIMER_TRACE(name, arg1, args...) \
	do { \
		if (android_msg_level & ANDROID_TRACE_LEVEL) { \
			printf("[%s] TIMER-TRACE) %s : " arg1, name, __func__, ## args); \
		} \
	} while (0)

#if defined(STRICT_GCC_WARNINGS) && defined(__GNUC__) && (__GNUC__ > 4 || (__GNUC__ == \
	4 && __GNUC_MINOR__ >= 6))
#define BCM_SET_LIST_FIRST_ENTRY(entry, ptr, type, member) \
_Pragma("GCC diagnostic push") \
_Pragma("GCC diagnostic ignored \"-Wcast-qual\"") \
(entry) = list_first_entry((ptr), type, member); \
_Pragma("GCC diagnostic pop") \

#define BCM_SET_CONTAINER_OF(entry, ptr, type, member) \
_Pragma("GCC diagnostic push") \
_Pragma("GCC diagnostic ignored \"-Wcast-qual\"") \
entry = container_of((ptr), type, member); \
_Pragma("GCC diagnostic pop") \

#else
#define BCM_SET_LIST_FIRST_ENTRY(entry, ptr, type, member) \
(entry) = list_first_entry((ptr), type, member); \

#define BCM_SET_CONTAINER_OF(entry, ptr, type, member) \
entry = container_of((ptr), type, member); \

#endif /* STRICT_GCC_WARNINGS */

typedef void(*FUNC_HANDLER) (void *cb_argu);

struct wl_func_q {
	struct list_head eq_list;
	FUNC_HANDLER cb_func;
	void *cb_argu;
};

typedef void(*TIMER_HANDLER) (void *cb_argu);

typedef struct timer_handler_list {
	struct list_head list;
	struct net_device *net;
	timer_list_compat_t *timer;
	TIMER_HANDLER cb_func;
	void *cb_argu;
	uint tmo_ms;
	ulong tmo_jiffies;
} timer_handler_list_t;

typedef struct wl_timer_params {
	dhd_pub_t *pub;
	struct list_head timer_list;
	struct list_head eq_list;
	timer_list_compat_t timer;
	spinlock_t eq_lock;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
	struct workqueue_struct *timer_workq;
	struct work_struct timer_work;
	struct workqueue_struct *func_workq;
	struct work_struct func_work;
#else
	tsk_ctl_t thr_timer_ctl;
	tsk_ctl_t thr_func_ctl;
#endif
} wl_timer_params_t;

static unsigned long
wl_func_lock_eq(struct wl_timer_params *timer_params)
{
	unsigned long flags;

	spin_lock_irqsave(&timer_params->eq_lock, flags);
	return flags;
}

static void
wl_func_unlock_eq(struct wl_timer_params *timer_params, unsigned long flags)
{
	spin_unlock_irqrestore(&timer_params->eq_lock, flags);
}

static void
wl_func_init_eq_lock(struct wl_timer_params *timer_params)
{
	spin_lock_init(&timer_params->eq_lock);
}

static void
wl_func_init_eq(struct wl_timer_params *timer_params)
{
	wl_func_init_eq_lock(timer_params);
	INIT_LIST_HEAD(&timer_params->eq_list);
}

static void
wl_func_flush_eq(struct wl_timer_params *timer_params)
{
	struct wl_func_q *e;
	unsigned long flags;

	flags = wl_func_lock_eq(timer_params);
	while (!list_empty_careful(&timer_params->eq_list)) {
		BCM_SET_LIST_FIRST_ENTRY(e, &timer_params->eq_list, struct wl_func_q, eq_list);
		list_del(&e->eq_list);
		kfree(e);
	}
	wl_func_unlock_eq(timer_params, flags);
}

static struct wl_func_q *
wl_func_deq(struct wl_timer_params *timer_params)
{
	struct wl_func_q *e = NULL;
	unsigned long flags;

	flags = wl_func_lock_eq(timer_params);
	if (likely(!list_empty(&timer_params->eq_list))) {
		BCM_SET_LIST_FIRST_ENTRY(e, &timer_params->eq_list, struct wl_func_q, eq_list);
		list_del(&e->eq_list);
	}
	wl_func_unlock_eq(timer_params, flags);

	return e;
}

static s32
wl_func_enq(struct wl_timer_params *timer_params,
	void *cb_func, void *cb_argu)
{
	struct wl_func_q *e;
	s32 err = 0;
	uint32 funcq_size;
	unsigned long flags;
	gfp_t aflags;

	funcq_size = sizeof(struct wl_func_q);
	aflags = (in_atomic()) ? GFP_ATOMIC : GFP_KERNEL;
	e = kzalloc(funcq_size, aflags);
	if (unlikely(!e)) {
		TIMER_ERROR("wlan", "funcq_size alloc failed %d\n", funcq_size);
		return -ENOMEM;
	}
	e->cb_func = cb_func;
	e->cb_argu = cb_argu;
	flags = wl_func_lock_eq(timer_params);
	list_add_tail(&e->eq_list, &timer_params->eq_list);
	wl_func_unlock_eq(timer_params, flags);

	return err;
}

static void
wl_func_put(struct wl_func_q *e)
{
	kfree(e);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
static void wl_func_handler(struct work_struct *data);
#define WL_FUNC_HANDLER() static void wl_func_handler(struct work_struct *data)
#else
static int wl_func_handler(void *data);
#define WL_FUNC_HANDLER() static int wl_func_handler(void *data)
#endif

WL_FUNC_HANDLER()
{
	struct wl_timer_params *timer_params = NULL;
	struct wl_func_q *e;
	struct net_device *net = NULL;
	dhd_pub_t *dhd;
	unsigned long flags = 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0))
	tsk_ctl_t *tsk = (tsk_ctl_t *)data;
	timer_params = (struct wl_timer_params *)tsk->parent;
#else
	BCM_SET_CONTAINER_OF(timer_params, data, struct wl_timer_params, func_work);
#endif

	dhd = timer_params->pub;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0))
	while (1) {
	if (down_interruptible(&tsk->sema) == 0) {
		SMP_RD_BARRIER_DEPENDS();
		if (tsk->terminated) {
			break;
		}
#endif
	DHD_EVENT_WAKE_LOCK(dhd);
	while ((e = wl_func_deq(timer_params))) {
		DHD_GENERAL_LOCK(dhd, flags);
		if (DHD_BUS_CHECK_DOWN_OR_DOWN_IN_PROGRESS(dhd)) {
			TIMER_ERROR(net->name, "BUS is DOWN.\n");
			DHD_GENERAL_UNLOCK(dhd, flags);
			goto fail;
		}
		DHD_GENERAL_UNLOCK(dhd, flags);
		e->cb_func(e->cb_argu);
fail:
		wl_func_put(e);
	}
	DHD_EVENT_WAKE_UNLOCK(dhd);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0))
	} else {
		break;
	}
	}
	complete_and_exit(&tsk->completed, 0);
#endif
}

void
wl_func_send(void *params, void *cb_func, void *cb_argu)
{
	struct wl_timer_params *timer_params = params;

	if (timer_params == NULL) {
		TIMER_ERROR("wlan", "Stale ignored\n");
		return;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
	if (timer_params->func_workq == NULL) {
		TIMER_ERROR("wlan", "Event handler is not created\n");
		return;
	}
#endif

	if (likely(!wl_func_enq(timer_params, cb_func, cb_argu))) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
		queue_work(timer_params->func_workq, &timer_params->func_work);
#else
		if (timer_params->thr_func_ctl.thr_pid >= 0) {
			up(&timer_params->thr_func_ctl.sema);
		}
#endif
	}
}

static s32
wl_func_create_handler(struct wl_timer_params *timer_params)
{
	int ret = 0;
	TIMER_TRACE("wlan", "Enter\n");

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
	if (!timer_params->func_workq) {
		timer_params->func_workq = alloc_workqueue("timer_funcd",
			WQ_MEM_RECLAIM | WQ_HIGHPRI | WQ_UNBOUND, 0);
	}
	if (!timer_params->func_workq) {
		TIMER_ERROR("wlan", "func_workq alloc_workqueue failed\n");
		ret = -ENOMEM;
	} else {
		INIT_WORK(&timer_params->func_work, wl_func_handler);
	}
#else
	PROC_START(wl_func_handler, timer_params, &timer_params->thr_func_ctl, 0, "timer_funcd");
	if (timer_params->thr_func_ctl.thr_pid < 0) {
		ret = -ENOMEM;
	}
#endif

	return ret;
}

static void
wl_func_destroy_handler(struct wl_timer_params *timer_params)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
	if (timer_params && timer_params->func_workq) {
		cancel_work_sync(&timer_params->func_work);
		destroy_workqueue(timer_params->func_workq);
		timer_params->func_workq = NULL;
	}
#else
	if (timer_params->thr_func_ctl.thr_pid >= 0) {
		PROC_STOP(&timer_params->thr_func_ctl);
	}
#endif
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
static void wl_timer_handler(struct work_struct *data);
#define WL_TIMER_HANDLER() static void wl_timer_handler(struct work_struct *data)
#else
static int wl_timer_handler(void *data);
#define WL_TIMER_HANDLER() static int wl_timer_handler(void *data)
#endif

WL_TIMER_HANDLER()
{
	struct wl_timer_params *timer_params = NULL;
	struct timer_handler_list *node, *next;
	dhd_pub_t *dhd;
	unsigned long flags = 0;
	unsigned long cur_jiffies, diff_jiffies, min_jiffies = 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0))
	tsk_ctl_t *tsk = (tsk_ctl_t *)data;
	timer_params = (struct wl_timer_params *)tsk->parent;
#else
	BCM_SET_CONTAINER_OF(timer_params, data, struct wl_timer_params, timer_work);
#endif

	dhd = timer_params->pub;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0))
	while (1) {
	if (down_interruptible(&tsk->sema) == 0) {
		SMP_RD_BARRIER_DEPENDS();
		if (tsk->terminated) {
			break;
		}
#endif
	DHD_EVENT_WAKE_LOCK(dhd);
	DHD_GENERAL_LOCK(dhd, flags);
	if (DHD_BUS_CHECK_DOWN_OR_DOWN_IN_PROGRESS(dhd)) {
		TIMER_ERROR("wlan", "BUS is DOWN.\n");
		DHD_GENERAL_UNLOCK(dhd, flags);
		goto exit;
	}
	DHD_GENERAL_UNLOCK(dhd, flags);
	cur_jiffies = jiffies;
	list_for_each_entry_safe(node, next, &timer_params->timer_list, list) {
		if (node->tmo_ms) {
			if (time_after(cur_jiffies, node->tmo_jiffies)) {
				wl_func_send(timer_params, node->cb_func, node->cb_argu);
				node->tmo_ms = 0;
			} else {
				diff_jiffies = node->tmo_jiffies - cur_jiffies;
				if (min_jiffies == 0)
					min_jiffies = diff_jiffies;
				else if (diff_jiffies < min_jiffies)
					min_jiffies = diff_jiffies;
			}
		}
	}
	if (min_jiffies)
		mod_timer(&timer_params->timer, jiffies + min_jiffies);
exit:
	DHD_EVENT_WAKE_UNLOCK(dhd);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0))
	} else {
		break;
	}
	}
	complete_and_exit(&tsk->completed, 0);
#endif
}

void
wl_timer_kick_handler(wl_timer_params_t *timer_params)
{
	if (timer_params == NULL) {
		TIMER_ERROR("wlan", "timer_params not ready\n");
		return;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
	if (timer_params->timer_workq == NULL) {
		TIMER_ERROR("wlan", "timer handler is not created\n");
		return;
	}
	queue_work(timer_params->timer_workq, &timer_params->timer_work);
#else
	if (timer_params->thr_timer_ctl.thr_pid >= 0) {
		up(&timer_params->thr_timer_ctl.sema);
	}
#endif
}

static void
wl_timer_timeout(unsigned long data)
{
	struct wl_timer_params *timer_params = (struct wl_timer_params *)data;

	if (!timer_params) {
		TIMER_ERROR("wlan", "timer_params is not ready\n");
		return;
	}

	TIMER_TRACE("wlan", "timer expired\n");
	wl_timer_kick_handler(timer_params);
}

static s32
wl_timer_create_handler(struct wl_timer_params *timer_params)
{
	int ret = 0;

	TIMER_TRACE("wlan", "Enter\n");
	
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
	if (!timer_params->timer_workq) {
		timer_params->timer_workq = alloc_workqueue("timerd",
			WQ_MEM_RECLAIM | WQ_HIGHPRI | WQ_UNBOUND, 0);
	}
	if (!timer_params->timer_workq) {
		TIMER_ERROR("wlan", "timer_workq alloc_workqueue failed\n");
		ret = -ENOMEM;
	} else {
		INIT_WORK(&timer_params->timer_work, wl_timer_handler);
	}
#else
	PROC_START(wl_timer_handler, timer_params, &timer_params->thr_timer_ctl, 0, "timerd");
	if (timer_params->thr_timer_ctl.thr_pid < 0) {
		ret = -ENOMEM;
	}
#endif

	return ret;
}

static void
wl_timer_destroy_handler(struct wl_timer_params *timer_params)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
	if (timer_params && timer_params->timer_workq) {
		cancel_work_sync(&timer_params->timer_work);
		destroy_workqueue(timer_params->timer_workq);
		timer_params->timer_workq = NULL;
	}
#else
	if (timer_params->thr_timer_ctl.thr_pid >= 0) {
		PROC_STOP(&timer_params->thr_timer_ctl);
	}
#endif
}

static void
wl_timer_free(struct wl_timer_params *timer_params)
{
	timer_handler_list_t *node, *next;

	list_for_each_entry_safe(node, next, &timer_params->timer_list, list) {
		TIMER_TRACE(node->net->name, "Free timer\n");
		list_del(&node->list);
		kfree(node);
	}
}

void
wl_timer_mod(dhd_pub_t *dhd, timer_list_compat_t *timer, uint32 tmo_ms)
{
	wl_timer_params_t *timer_params = dhd->timer_params;
	timer_handler_list_t *node, *next;
	bool found = FALSE;

	list_for_each_entry_safe(node, next, &timer_params->timer_list, list) {
		if (node->timer == timer) {
			node->tmo_ms = tmo_ms;
			if (tmo_ms) {
				TIMER_TRACE(node->net->name, "update timer %dms\n", tmo_ms);
				node->tmo_jiffies = jiffies + msecs_to_jiffies(tmo_ms);
				wl_timer_kick_handler(timer_params);
			}
			found = TRUE;
			break;
		}
	}
	if (!found)
		TIMER_ERROR("wlan", "timer not found\n");
}

void
wl_timer_register(struct net_device *net, timer_list_compat_t *timer, void *cb_func)
{
	dhd_pub_t *dhd = dhd_get_pub(net);
	wl_timer_params_t *timer_params = dhd->timer_params;
	timer_handler_list_t *node, *next, *leaf;

	list_for_each_entry_safe(node, next, &timer_params->timer_list, list) {
		if (node->timer == timer) {
			TIMER_TRACE(net->name, "timer already registered\n");
			return;
		}
	}

	leaf = kmalloc(sizeof(timer_handler_list_t), GFP_KERNEL);
	if (!leaf) {
		TIMER_ERROR(net->name, "Memory alloc failure %d\n",
			(int)sizeof(timer_handler_list_t));
		return;
	}
	memset(leaf, 0, sizeof(timer_handler_list_t));
	leaf->net = net;
	leaf->timer = timer;
	leaf->cb_func = cb_func;
	leaf->cb_argu = net;
	TIMER_ERROR(net->name, "timer registered tmo=%d\n", leaf->tmo_ms);
	list_add_tail(&leaf->list, &timer_params->timer_list);

	return;
}

void
wl_timer_deregister(struct net_device *net, timer_list_compat_t *timer)
{
	dhd_pub_t *dhd = dhd_get_pub(net);
	wl_timer_params_t *timer_params = dhd->timer_params;
	timer_handler_list_t *node, *next;

	list_for_each_entry_safe(node, next, &timer_params->timer_list, list) {
		if (node->timer == timer) {
			TIMER_TRACE(net->name, "timer deregistered\n");
			list_del(&node->list);
			kfree(node);
		}
	}
	return;
}

static s32
wl_timer_init_priv(struct wl_timer_params *timer_params)
{
	s32 err = 0;

	INIT_LIST_HEAD(&timer_params->timer_list);
	if (wl_timer_create_handler(timer_params))
		return -ENOMEM;
	wl_func_init_eq(timer_params);
	if (wl_func_create_handler(timer_params))
		return -ENOMEM;

	return err;
}

static void
wl_timer_deinit_priv(struct wl_timer_params *timer_params)
{
	wl_timer_free(timer_params);
	wl_func_destroy_handler(timer_params);
	wl_func_flush_eq(timer_params);
	wl_timer_destroy_handler(timer_params);
}

void
wl_timer_dettach(dhd_pub_t *dhdp)
{
	struct wl_timer_params *timer_params = dhdp->timer_params;

	if (timer_params) {
		if (timer_pending(&timer_params->timer))
			del_timer_sync(&timer_params->timer);
		wl_timer_deinit_priv(timer_params);
		kfree(timer_params);
		dhdp->timer_params = NULL;
	}
}

s32
wl_timer_attach(struct net_device *net)
{
	struct dhd_pub *dhdp = dhd_get_pub(net);
	struct wl_timer_params *timer_params = NULL;
	s32 err = 0;

	timer_params = kmalloc(sizeof(wl_timer_params_t), GFP_KERNEL);
	if (!timer_params) {
		TIMER_ERROR(net->name, "Failed to allocate memory (%zu)\n",
			sizeof(wl_timer_params_t));
		return -ENOMEM;
	}
	dhdp->timer_params = timer_params;
	memset(timer_params, 0, sizeof(wl_timer_params_t));
	timer_params->pub = dhdp;

	err = wl_timer_init_priv(timer_params);
	if (err) {
		TIMER_ERROR(net->name, "Failed to wl_timer_init_priv (%d)\n", err);
		goto exit;
	}
	init_timer_compat(&timer_params->timer, wl_timer_timeout, timer_params);

exit:
	if (err)
		wl_timer_dettach(dhdp);
	return err;
}
#else
void
wl_timer_mod(dhd_pub_t *dhd, timer_list_compat_t *timer, uint32 tmo_ms)
{
	if (timer_pending(timer))
		del_timer_sync(timer);
	if (tmo_ms)
		mod_timer(timer, jiffies + msecs_to_jiffies(tmo_ms));
}

void
wl_timer_register(struct net_device *net, timer_list_compat_t *timer, void *cb_func)
{
	init_timer_compat(timer, cb_func, net);
}

void
wl_timer_deregister(struct net_device *net, timer_list_compat_t *timer)
{
	if (timer_pending(timer))
		del_timer_sync(timer);
}
#endif
