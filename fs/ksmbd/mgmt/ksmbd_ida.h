/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 *   Copyright (C) 2018 Samsung Electronics Co., Ltd.
 */

#ifndef __KSMBD_IDA_MANAGEMENT_H__
#define __KSMBD_IDA_MANAGEMENT_H__

#include <linux/slab.h>
#include <linux/idr.h>

/*
 * 2.2.1.6.7 TID Generation
 *    The value 0xFFFF MUST NOT be used as a valid TID. All other
 *    possible values for TID, including zero (0x0000), are valid.
 *    The value 0xFFFF is used to specify all TIDs or no TID,
 *    depending upon the context in which it is used.
 */
#ifdef CONFIG_SMB_INSECURE_SERVER
int ksmbd_acquire_smb1_tid(struct ida *ida);
#endif
int ksmbd_acquire_smb2_tid(struct ida *ida);

/*
 * 2.2.1.6.8 UID Generation
 *    The value 0xFFFE was declared reserved in the LAN Manager 1.0
 *    documentation, so a value of 0xFFFE SHOULD NOT be used as a
 *    valid UID.<21> All other possible values for a UID, excluding
 *    zero (0x0000), are valid.
 */
#ifdef CONFIG_SMB_INSECURE_SERVER
int ksmbd_acquire_smb1_uid(struct ida *ida);
#endif
int ksmbd_acquire_smb2_uid(struct ida *ida);
int ksmbd_acquire_async_msg_id(struct ida *ida);

int ksmbd_acquire_id(struct ida *ida);

void ksmbd_release_id(struct ida *ida, int id);
#endif /* __KSMBD_IDA_MANAGEMENT_H__ */
