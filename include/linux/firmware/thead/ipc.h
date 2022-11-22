// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */

#ifndef _SC_IPC_H
#define _SC_IPC_H

#include <linux/device.h>
#include <linux/types.h>

#define AON_RPC_MSG_MAGIC       (0xef)
#define LIGHT_AON_RPC_VERSION	1
#define LIGHT_AON_RPC_MSG_NUM	7

struct light_aon_ipc;

enum light_aon_rpc_svc {
	LIGHT_AON_RPC_SVC_UNKNOWN = 0,
	LIGHT_AON_RPC_SVC_RETURN = 1,
	LIGHT_AON_RPC_SVC_PM = 2,
	LIGHT_AON_RPC_SVC_MISC = 3,
	LIGHT_AON_RPC_SVC_AVFS = 4,
};

enum light_aon_misc_func {
	LIGHT_AON_MISC_FUNC_UNKNOWN = 0,
	LIGHT_AON_MISC_FUNC_SET_CONTROL = 1,
	LIGHT_AON_MISC_FUNC_GET_CONTROL = 2,
	LIGHT_AON_MISC_FUNC_WDG_START = 3,
	LIGHT_AON_MISC_FUNC_WDG_STOP = 4,
	LIGHT_AON_MISC_FUNC_WDG_PING = 5,
	LIGHT_AON_MISC_FUNC_WDG_TIMEOUTSET = 6,
	LIGHT_AON_MISC_FUNC_WDG_RESTART = 7,
	LIGHT_AON_MISC_FUNC_WDG_GET_STATE = 8,
	LIGHT_AON_MISC_FUNC_WDG_POWER_OFF = 9,
	LIGHT_AON_MISC_FUNC_AON_WDT_ON  = 10,
	LIGHT_AON_MISC_FUNC_AON_WDT_OFF = 11,
};

enum light_aon_pm_func {
	LIGHT_AON_PM_FUNC_UNKNOWN = 0,
	LIGHT_AON_PM_FUNC_SET_RESOURCE_REGULATOR = 1,
	LIGHT_AON_PM_FUNC_GET_RESOURCE_REGULATOR = 2,
	LIGHT_AON_PM_FUNC_SET_RESOURCE_POWER_MODE = 3,
	LIGHT_AON_PM_FUNC_PWR_SET  = 4,
	LIGHT_AON_PM_FUNC_PWR_GET  = 5,
};

struct light_aon_rpc_msg_hdr {
	uint8_t ver;                   ///< version of msg hdr
	uint8_t size;                  ///< msg size ,uinit in bytes,the size includes rpc msg header self.
	uint8_t svc;                   ///< rpc main service id
	uint8_t func;                  ///< rpc sub func id of specific service, sent by caller
} __packed __aligned(4);

/*
 * Defines for SC PM Power Mode
 */
#define LIGHT_AON_PM_PW_MODE_OFF	0	/* Power off */
#define LIGHT_AON_PM_PW_MODE_STBY	1	/* Power in standby */
#define LIGHT_AON_PM_PW_MODE_LP		2	/* Power in low-power */
#define LIGHT_AON_PM_PW_MODE_ON		3	/* Power on */

int light_aon_call_rpc(struct light_aon_ipc *ipc, void *msg, bool have_resp);
int light_aon_get_handle(struct light_aon_ipc **ipc);
int light_aon_misc_set_control(struct light_aon_ipc *ipc, u16 resource, u32 ctrl, u32 val);
int light_aon_misc_get_control(struct light_aon_ipc *ipc, u16 resource, u32 ctrl, u32 *val);
#endif /* _SC_IPC_H */
