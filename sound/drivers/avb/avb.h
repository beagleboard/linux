#ifndef AVB_H
#define AVB_H

#include "msrp.h"
#include "avdecc.h"
#include "avtp.h"

struct work_data {
	struct delayed_work work;
	int delayed_work_id;
	union delayed_work_data {
		void *data;
		struct msrp *msrp;
		struct avb_card *card;
		struct avdecc *avdecc;
	} dw;
};

struct avb_device {
	int tx_ts[AVB_MAX_TS_SLOTS];
	int rx_ts[AVB_MAX_TS_SLOTS];
	int tx_idx;
	int rx_idx;
	struct msrp msrp;
	struct avdecc avdecc;
	struct avb_card card;
	struct snd_hwdep *hwdep;
#ifdef AVB_USE_HIGH_RES_TIMER
	struct avb_hrtimer txTimer;
#else
	struct timer_list txTimer;
#endif
	struct work_data *avdeccwd;
	struct work_data *msrpwd;
	struct work_data *avtpwd;
	struct workqueue_struct *wq;
};

#endif