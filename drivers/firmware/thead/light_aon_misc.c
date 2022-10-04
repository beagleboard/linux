// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */

#include <linux/firmware/thead/ipc.h>

struct light_aon_msg_req_misc_set_ctrl {
	struct light_aon_rpc_msg_hdr hdr;
	u32 ctrl;
	u32 val;
	u16 resource;
	u16 reserved[7];
} __packed __aligned(4);

struct light_aon_msg_req_misc_get_ctrl {
	struct light_aon_rpc_msg_hdr hdr;
	u32 ctrl;
	u16 resource;
	u16 reserved[9];
} __packed __aligned(4);

struct light_aon_msg_resp_misc_get_ctrl {
	struct light_aon_rpc_msg_hdr hdr;
	u32 val;
	u32 reserved[5];
} __packed __aligned(4);

int light_aon_misc_set_control(struct light_aon_ipc *ipc, u16 resource,
			    u32 ctrl, u32 val)
{
	struct light_aon_msg_req_misc_set_ctrl msg;
	struct light_aon_rpc_msg_hdr *hdr = &msg.hdr;

	hdr->ver = LIGHT_AON_RPC_VERSION;
	hdr->svc = (uint8_t)LIGHT_AON_RPC_SVC_MISC;
	hdr->func = (uint8_t)LIGHT_AON_MISC_FUNC_SET_CONTROL;
	hdr->size = LIGHT_AON_RPC_MSG_NUM;

	msg.ctrl = ctrl;
	msg.val = val;
	msg.resource = resource;

	return light_aon_call_rpc(ipc, &msg, true);
}
EXPORT_SYMBOL(light_aon_misc_set_control);

int light_aon_misc_get_control(struct light_aon_ipc *ipc, u16 resource,
			    u32 ctrl, u32 *val)
{
	struct light_aon_msg_req_misc_get_ctrl msg;
	struct light_aon_msg_resp_misc_get_ctrl *resp;
	struct light_aon_rpc_msg_hdr *hdr = &msg.hdr;
	int ret;

	hdr->ver = LIGHT_AON_RPC_VERSION;
	hdr->svc = (uint8_t)LIGHT_AON_RPC_SVC_MISC;
	hdr->func = (uint8_t)LIGHT_AON_MISC_FUNC_GET_CONTROL;
	hdr->size = LIGHT_AON_RPC_MSG_NUM;

	msg.ctrl = ctrl;
	msg.resource = resource;

	ret = light_aon_call_rpc(ipc, &msg, true);
	if (ret)
		return ret;

	resp = (struct light_aon_msg_resp_misc_get_ctrl *)&msg;
	if (val != NULL)
		*val = resp->val;

	return 0;
}
EXPORT_SYMBOL(light_aon_misc_get_control);
