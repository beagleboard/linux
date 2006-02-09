#ifndef __HWA742_H
#define __HWA742_H

#include <linux/fb.h>

#include <asm/arch/omapfb.h>

#define HWA742_EVENT_READY	1
#define HWA742_EVENT_DISABLED	2

struct hwa742_notifier_block {
	struct notifier_block	nb;
	void			*data;
};

typedef int (*hwa742_notifier_callback_t)(struct hwa742_notifier_block *,
					   unsigned long event,
					   struct omapfb_device *fbdev);

extern void hwa742_read_id(int *rev_code, int *config);
extern int hwa742_register_client(struct hwa742_notifier_block *hwa742_nb,
				   hwa742_notifier_callback_t callback,
				   void *callback_data);
extern int hwa742_unregister_client(struct hwa742_notifier_block *hwa742_nb);
extern int hwa742_update_window_async(struct omapfb_update_window *win,
					void (*complete_callback)(void *arg),
					void *complete_callback_data);

#endif
