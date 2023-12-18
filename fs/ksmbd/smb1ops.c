// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *   Copyright (C) 2016 Namjae Jeon <linkinjeon@kernel.org>
 *   Copyright (C) 2018 Samsung Electronics Co., Ltd.
 */

#include <linux/slab.h>

#include "glob.h"
#include "connection.h"
#include "smb_common.h"
#include "smb1pdu.h"

static struct smb_version_values smb1_server_values = {
	.version_string = SMB1_VERSION_STRING,
	.protocol_id = SMB10_PROT_ID,
	.capabilities = SMB1_SERVER_CAPS,
	.max_read_size = CIFS_DEFAULT_IOSIZE,
	.max_write_size = MAX_STREAM_PROT_LEN,
	.max_trans_size = CIFS_DEFAULT_IOSIZE,
	.large_lock_type = LOCKING_ANDX_LARGE_FILES,
	.exclusive_lock_type = 0,
	.shared_lock_type = LOCKING_ANDX_SHARED_LOCK,
	.unlock_lock_type = 0,
	.header_size = sizeof(struct smb_hdr),
	.max_header_size = MAX_CIFS_HDR_SIZE,
	.read_rsp_size = sizeof(struct smb_com_read_rsp),
	.lock_cmd = cpu_to_le16(SMB_COM_LOCKING_ANDX),
	.cap_unix = CAP_UNIX,
	.cap_nt_find = CAP_NT_SMBS | CAP_NT_FIND,
	.cap_large_files = CAP_LARGE_FILES,
	.signing_enabled = SECMODE_SIGN_ENABLED,
	.signing_required = SECMODE_SIGN_REQUIRED,
};

static struct smb_version_ops smb1_server_ops = {
	.get_cmd_val = get_smb_cmd_val,
	.init_rsp_hdr = init_smb_rsp_hdr,
	.set_rsp_status = set_smb_rsp_status,
	.allocate_rsp_buf = smb_allocate_rsp_buf,
	.check_user_session = smb_check_user_session,
	.is_sign_req = smb1_is_sign_req,
	.check_sign_req = smb1_check_sign_req,
	.set_sign_rsp = smb1_set_sign_rsp,
	.get_ksmbd_tcon = smb_get_ksmbd_tcon,
};

static struct smb_version_cmds smb1_server_cmds[256] = {
	[SMB_COM_CREATE_DIRECTORY]	= { .proc = smb_mkdir, },
	[SMB_COM_DELETE_DIRECTORY]	= { .proc = smb_rmdir, },
	[SMB_COM_CLOSE]			= { .proc = smb_close, },
	[SMB_COM_FLUSH]			= { .proc = smb_flush, },
	[SMB_COM_DELETE]		= { .proc = smb_unlink, },
	[SMB_COM_RENAME]		= { .proc = smb_rename, },
	[SMB_COM_QUERY_INFORMATION]	= { .proc = smb_query_info, },
	[SMB_COM_SETATTR]		= { .proc = smb_setattr, },
	[SMB_COM_LOCKING_ANDX]		= { .proc = smb_locking_andx, },
	[SMB_COM_TRANSACTION]		= { .proc = smb_trans, },
	[SMB_COM_ECHO]			= { .proc = smb_echo, },
	[SMB_COM_OPEN_ANDX]		= { .proc = smb_open_andx, },
	[SMB_COM_READ_ANDX]		= { .proc = smb_read_andx, },
	[SMB_COM_WRITE_ANDX]		= { .proc = smb_write_andx, },
	[SMB_COM_TRANSACTION2]		= { .proc = smb_trans2, },
	[SMB_COM_FIND_CLOSE2]		= { .proc = smb_closedir, },
	[SMB_COM_TREE_DISCONNECT]	= { .proc = smb_tree_disconnect, },
	[SMB_COM_NEGOTIATE]		= { .proc = smb_negotiate_request, },
	[SMB_COM_SESSION_SETUP_ANDX]	= { .proc = smb_session_setup_andx, },
	[SMB_COM_LOGOFF_ANDX]           = { .proc = smb_session_disconnect, },
	[SMB_COM_TREE_CONNECT_ANDX]	= { .proc = smb_tree_connect_andx, },
	[SMB_COM_QUERY_INFORMATION_DISK] = { .proc = smb_query_information_disk, },
	[SMB_COM_NT_CREATE_ANDX]	= { .proc = smb_nt_create_andx, },
	[SMB_COM_NT_CANCEL]		= { .proc = smb_nt_cancel, },
	[SMB_COM_NT_RENAME]		= { .proc = smb_nt_rename, },
	[SMB_COM_WRITE]			= { .proc = smb_write, },
	[SMB_COM_CHECK_DIRECTORY]	= { .proc = smb_checkdir, },
	[SMB_COM_PROCESS_EXIT]		= { .proc = smb_process_exit, },
};

/**
 * init_smb1_server() - initialize a smb server connection with smb1
 *			command dispatcher
 * @conn:	connection instance
 */
int init_smb1_server(struct ksmbd_conn *conn)
{
	conn->vals = &smb1_server_values;
	conn->ops = &smb1_server_ops;
	conn->cmds = smb1_server_cmds;
	conn->max_cmds = ARRAY_SIZE(smb1_server_cmds);
	return 0;
}
