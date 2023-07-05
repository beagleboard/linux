// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *   Copyright (C) 2016 Namjae Jeon <linkinjeon@kernel.org>
 *   Copyright (C) 2018 Samsung Electronics Co., Ltd.
 */
#include <linux/math64.h>
#include <linux/fs.h>
#include <linux/posix_acl_xattr.h>
#include <linux/namei.h>
#include <linux/statfs.h>
#include <linux/vmalloc.h>

#include "glob.h"
#include "oplock.h"
#include "connection.h"
#include "transport_ipc.h"
#include "vfs.h"
#include "misc.h"

#include "auth.h"
#include "asn1.h"
#include "server.h"
#include "smb_common.h"
#include "smb1pdu.h"
#include "smbstatus.h"
#include "mgmt/user_config.h"
#include "mgmt/share_config.h"
#include "mgmt/tree_connect.h"
#include "mgmt/user_session.h"
#include "ndr.h"
#include "smberr.h"

static int smb1_oplock_enable = false;

/* Default: allocation roundup size = 1048576 */
static unsigned int alloc_roundup_size = 1048576;

struct ksmbd_dirent {
	unsigned long long	ino;
	unsigned long long	offset;
	unsigned int		namelen;
	unsigned int		d_type;
	char			name[];
};

/**
 * smb_NTtimeToUnix() - convert NTFS time to unix style time format
 * @ntutc:	NTFS style time
 *
 * Convert the NT UTC (based 1601-01-01, in hundred nanosecond units)
 * into Unix UTC (based 1970-01-01, in seconds).
 *
 * Return:      timespec containing unix style time
 */
static struct timespec64 smb_NTtimeToUnix(__le64 ntutc)
{
	struct timespec64 ts;

	/* BB what about the timezone? BB */

	/* Subtract the NTFS time offset, then convert to 1s intervals. */
	/* this has been taken from cifs, ntfs code */
	u64 t;

	t = le64_to_cpu(ntutc) - NTFS_TIME_OFFSET;
	ts.tv_nsec = do_div(t, 10000000) * 100;
	ts.tv_sec = t;
	return ts;
}

/**
 * get_smb_cmd_val() - get smb command value from smb header
 * @work:	smb work containing smb header
 *
 * Return:      smb command value
 */
u16 get_smb_cmd_val(struct ksmbd_work *work)
{
	struct smb_hdr *rcv_hdr = (struct smb_hdr *)work->request_buf;

	return (u16)rcv_hdr->Command;
}

/**
 * is_smbreq_unicode() - check if the smb command is request is unicode or not
 * @hdr:	pointer to smb_hdr in the the request part
 *
 * Return: check flags and return true if request is unicode, else false
 */
static inline int is_smbreq_unicode(struct smb_hdr *hdr)
{
	return hdr->Flags2 & SMBFLG2_UNICODE ? 1 : 0;
}

/**
 * set_smb_rsp_status() - set error type in smb response header
 * @work:	smb work containing smb response header
 * @err:	error code to set in response
 */
void set_smb_rsp_status(struct ksmbd_work *work, __le32 err)
{
	struct smb_hdr *rsp_hdr = (struct smb_hdr *) work->response_buf;

	rsp_hdr->Status.CifsError = err;
}

/**
 * init_smb_rsp_hdr() - initialize smb response header
 * @work:	smb work containing smb request
 *
 * Return:      0 on success, otherwise -EINVAL
 */
int init_smb_rsp_hdr(struct ksmbd_work *work)
{
	struct ksmbd_conn *conn = work->conn;
	struct smb_hdr *rsp_hdr;
	struct smb_hdr *rcv_hdr = (struct smb_hdr *)work->request_buf;

	rsp_hdr = (struct smb_hdr *) work->response_buf;
	memset(rsp_hdr, 0, sizeof(struct smb_hdr) + 2);

	/* remove 4 byte direct TCP header, add 1 byte wc and 2 byte bcc */
	rsp_hdr->smb_buf_length =
		cpu_to_be32(conn->vals->header_size - 4 + 2);
	memcpy(rsp_hdr->Protocol, rcv_hdr->Protocol, 4);
	rsp_hdr->Command = rcv_hdr->Command;

	/*
	 * Message is response. Other bits are obsolete.
	 */
	rsp_hdr->Flags = (SMBFLG_RESPONSE);

	/*
	 * Lets assume error code are NTLM. True for CIFS and windows 7
	 */
	rsp_hdr->Flags2 = rcv_hdr->Flags2;
	rsp_hdr->PidHigh = rcv_hdr->PidHigh;
	rsp_hdr->Pid = rcv_hdr->Pid;
	rsp_hdr->Mid = rcv_hdr->Mid;
	rsp_hdr->WordCount = 0;

	/* We can do the above test because we have set maxVCN as 1 */
	rsp_hdr->Uid = rcv_hdr->Uid;
	rsp_hdr->Tid = rcv_hdr->Tid;
	return 0;
}

/**
 * smb_allocate_rsp_buf() - allocate response buffer for a command
 * @work:	smb work containing smb request
 *
 * Return:      0 on success, otherwise -ENOMEM
 */
int smb_allocate_rsp_buf(struct ksmbd_work *work)
{
	struct smb_hdr *hdr = (struct smb_hdr *)work->request_buf;
	unsigned char cmd = hdr->Command;
	size_t large_sz = work->conn->vals->max_read_size + MAX_CIFS_HDR_SIZE;
	size_t sz = MAX_CIFS_SMALL_BUFFER_SIZE;

	if (cmd == SMB_COM_TRANSACTION2) {
		struct smb_com_trans2_qpi_req *req = work->request_buf;
		u16 sub_cmd = le16_to_cpu(req->SubCommand);
		u16 infolevel = le16_to_cpu(req->InformationLevel);

		if ((sub_cmd == TRANS2_FIND_FIRST) ||
				(sub_cmd == TRANS2_FIND_NEXT) ||
				(sub_cmd == TRANS2_QUERY_PATH_INFORMATION &&
				 (infolevel == SMB_QUERY_FILE_UNIX_LINK ||
				  infolevel == SMB_QUERY_POSIX_ACL ||
				  infolevel == SMB_INFO_QUERY_ALL_EAS)))
			sz = large_sz;
	}

	if (cmd == SMB_COM_TRANSACTION)
		sz = large_sz;

	if (cmd == SMB_COM_ECHO) {
		int resp_size;
		struct smb_com_echo_req *req = work->request_buf;

		/*
		 * size of struct smb_com_echo_rsp + Bytecount - Size of Data
		 * in struct smb_com_echo_rsp
		 */
		resp_size = sizeof(struct smb_com_echo_rsp) +
			le16_to_cpu(req->ByteCount) - 1;
		if (resp_size > MAX_CIFS_SMALL_BUFFER_SIZE)
			sz = large_sz;
	}

	work->response_buf = kvmalloc(sz, GFP_KERNEL | __GFP_ZERO);
	work->response_sz = sz;

	if (!work->response_buf) {
		pr_err("Failed to allocate %zu bytes buffer\n", sz);
		return -ENOMEM;
	}

	return 0;
}

/**
 * andx_request_buffer() - return pointer to matching andx command
 * @work:	buffer containing smb request
 * @command:	match next command with this command
 *
 * Return:      pointer to matching command buffer on success, otherwise NULL
 */
static char *andx_request_buffer(char *buf, int command)
{
	struct andx_block *andx_ptr = (struct andx_block *)(buf +
					sizeof(struct smb_hdr) - 1);
	struct andx_block *next;

	while (andx_ptr->AndXCommand != SMB_NO_MORE_ANDX_COMMAND) {
		next = (struct andx_block *)
			(buf + 4 + le16_to_cpu(andx_ptr->AndXOffset));
		if (andx_ptr->AndXCommand == command)
			return (char *)next;
		andx_ptr = next;
	}
	return NULL;
}

/**
 * andx_response_buffer() - return pointer to andx response buffer
 * @buf:	buffer containing smb request
 *
 * Return:      pointer to andx command response on success, otherwise NULL
 */
static char *andx_response_buffer(char *buf)
{
	int pdu_length = get_rfc1002_len(buf);

	return buf + 4 + pdu_length;
}

/**
 * smb_check_user_session() - check for valid session for a user
 * @work:	smb work containing smb request buffer
 *
 * Return:      0 on success, otherwise error
 */
int smb_check_user_session(struct ksmbd_work *work)
{
	struct smb_hdr *req_hdr = (struct smb_hdr *)work->request_buf;
	struct ksmbd_conn *conn = work->conn;
	unsigned int cmd = conn->ops->get_cmd_val(work);

	work->sess = NULL;
	if (cmd == SMB_COM_NEGOTIATE || cmd == SMB_COM_SESSION_SETUP_ANDX ||
		cmd == SMB_COM_ECHO)
		return 0;

	if (!ksmbd_conn_good(conn))
		return -EINVAL;

	if (xa_empty(&conn->sessions)) {
		ksmbd_debug(SMB, "NO sessions registered\n");
		return -EINVAL;
	}

	work->sess = ksmbd_session_lookup(conn, le16_to_cpu(req_hdr->Uid));
	if (work->sess)
		return 1;
	ksmbd_debug(SMB, "Invalid user session, Uid %u\n",
			le16_to_cpu(req_hdr->Uid));
	return -EINVAL;
}

/**
 * smb_get_ksmbd_tcon() - get tree connection information for a tree id
 * @sess:	session containing tree list
 * @tid:	match tree connection with tree id
 *
 * Return:      matching tree connection on success, otherwise error
 */
int smb_get_ksmbd_tcon(struct ksmbd_work *work)
{
	struct smb_hdr *req_hdr = (struct smb_hdr *)work->request_buf;
	u8 cmd = req_hdr->Command;
	int tree_id;

	work->tcon = NULL;
	if (cmd == SMB_COM_TREE_CONNECT_ANDX ||
	    cmd == SMB_COM_NT_CANCEL ||
	    cmd == SMB_COM_LOGOFF_ANDX) {
		ksmbd_debug(SMB, "skip to check tree connect request\n");
		return 0;
	}

	if (xa_empty(&work->sess->tree_conns)) {
		ksmbd_debug(SMB, "NO tree connected\n");
		return -ENOENT;
	}

	tree_id = le16_to_cpu(req_hdr->Tid);
	work->tcon = ksmbd_tree_conn_lookup(work->sess, tree_id);
	if (!work->tcon) {
		pr_err("Invalid tid %d\n", tree_id);
		return -EINVAL;
	}

	return 1;
}

/**
 * smb_session_disconnect() - LOGOFF request handler
 * @work:	smb work containing log off request buffer
 *
 * Return:      0 on success, otherwise error
 */
int smb_session_disconnect(struct ksmbd_work *work)
{
	struct ksmbd_conn *conn = work->conn;
	struct ksmbd_session *sess = work->sess;

	/* setting CifsExiting here may race with start_tcp_sess */
	ksmbd_conn_set_need_reconnect(conn);

	ksmbd_conn_wait_idle(conn, sess->id);

	ksmbd_tree_conn_session_logoff(sess);
	xa_erase(&conn->sessions, sess->id);
	ksmbd_session_destroy(sess);
	work->sess = NULL;

	/* let start_tcp_sess free conn info now */
	ksmbd_conn_set_exiting(conn);
	return 0;
}

/**
 * smb_session_disconnect() - tree disconnect request handler
 * @work:	smb work containing tree disconnect request buffer
 *
 * Return:      0 on success, otherwise error
 */
int smb_tree_disconnect(struct ksmbd_work *work)
{
	struct smb_hdr *req_hdr = (struct smb_hdr *)work->request_buf;
	struct smb_hdr *rsp_hdr = (struct smb_hdr *)work->response_buf;
	struct ksmbd_tree_connect *tcon = work->tcon;
	struct ksmbd_session *sess = work->sess;

	if (!tcon) {
		pr_err("Invalid tid %d\n", req_hdr->Tid);
		rsp_hdr->Status.CifsError = STATUS_NO_SUCH_USER;
		return -EINVAL;
	}

	ksmbd_close_tree_conn_fds(work);
	ksmbd_tree_conn_disconnect(sess, tcon);
	return 0;
}

static void set_service_type(struct ksmbd_conn *conn,
		struct ksmbd_share_config *share,
		struct smb_com_tconx_rsp_ext *rsp)
{
	int length;
	char *buf = rsp->Service;

	if (test_share_config_flag(share, KSMBD_SHARE_FLAG_PIPE)) {
		length = strlen(SERVICE_IPC_SHARE);
		memcpy(buf, SERVICE_IPC_SHARE, length);
		rsp->ByteCount = cpu_to_le16(length + 1);
		buf += length;
		*buf = '\0';
	} else {
		int uni_len = 0;

		length = strlen(SERVICE_DISK_SHARE);
		memcpy(buf, SERVICE_DISK_SHARE, length);
		buf[length] = '\0';
		length += 1;
		uni_len = smbConvertToUTF16((__le16 *)(buf + length),
					     NATIVE_FILE_SYSTEM,
					     PATH_MAX, conn->local_nls, 0);
		uni_len++;
		uni_len *= 2;
		length += uni_len;
		rsp->ByteCount = cpu_to_le16(length);
	}
}

/**
 * smb_tree_connect_andx() - tree connect request handler
 * @work:	smb work containing tree connect request buffer
 *
 * Return:      0 on success, otherwise error
 */
int smb_tree_connect_andx(struct ksmbd_work *work)
{
	struct smb_hdr *req_hdr = (struct smb_hdr *)work->request_buf;
	struct smb_hdr *rsp_hdr = (struct smb_hdr *)work->response_buf;
	struct ksmbd_conn *conn = work->conn;
	struct smb_com_tconx_req *req;
	struct smb_com_tconx_rsp_ext *rsp;
	int extra_byte = 0;
	char *treename = NULL, *name = NULL, *dev_type = NULL;
	struct ksmbd_share_config *share;
	struct ksmbd_session *sess = work->sess;
	int dev_flags = 0;
	struct ksmbd_tree_conn_status status;

	/* Is this an ANDX command ? */
	if (req_hdr->Command != SMB_COM_TREE_CONNECT_ANDX) {
		ksmbd_debug(SMB, "SMB_COM_TREE_CONNECT_ANDX is part of ANDX");
		req = (struct smb_com_tconx_req *)
			andx_request_buffer(work->request_buf,
				SMB_COM_TREE_CONNECT_ANDX);
		rsp = (struct smb_com_tconx_rsp_ext *)
			andx_response_buffer(work->response_buf);
		extra_byte = 3;
		if (!req) {
			status.ret = -EINVAL;
			goto out_err;
		}
	} else {
		req = (struct smb_com_tconx_req *)(&req_hdr->WordCount);
		rsp = (struct smb_com_tconx_rsp_ext *)(&rsp_hdr->WordCount);
	}

	/* check if valid tree name is present in request or not */
	if (!req->PasswordLength) {
		treename = smb_strndup_from_utf16(req->Password + 1,
				256, true, conn->local_nls);
		dev_type = smb_strndup_from_utf16(req->Password + 1 +
			((strlen(treename) + 1) * 2), 256, false,
			conn->local_nls);
	} else {
		treename = smb_strndup_from_utf16(req->Password +
				le16_to_cpu(req->PasswordLength), 256, true,
				conn->local_nls);
		dev_type = smb_strndup_from_utf16(req->Password +
			le16_to_cpu(req->PasswordLength) +
				((strlen(treename) + 1) * 2),
			256, false, conn->local_nls);
	}

	if (IS_ERR(treename) || IS_ERR(dev_type)) {
		pr_err("Unable to strdup() treename or devtype uid %d\n",
		       rsp_hdr->Uid);
		status.ret = KSMBD_TREE_CONN_STATUS_ERROR;
		goto out_err;
	}
	name = ksmbd_extract_sharename(conn->um, treename);
	if (IS_ERR(name)) {
		status.ret = KSMBD_TREE_CONN_STATUS_ERROR;
		goto out_err;
	}

	ksmbd_debug(SMB, "tree connect request for tree %s, dev_type : %s\n",
		name, dev_type);

	if (!strcmp(dev_type, "A:"))
		dev_flags = 1;
	else if (!strncmp(dev_type, "LPT", 3))
		dev_flags = 2;
	else if (!strcmp(dev_type, "IPC"))
		dev_flags = 3;
	else if (!strcmp(dev_type, "COMM"))
		dev_flags = 4;
	else if (!strcmp(dev_type, "?????"))
		dev_flags = 5;

	if (!strcmp(name, "IPC$")) {
		if (dev_flags < 3) {
			status.ret = -ENODEV;
			goto out_err;
		}
	} else if (!dev_flags || (dev_flags > 1 && dev_flags < 5)) {
		status.ret = -ENODEV;
		goto out_err;
	}

	status = ksmbd_tree_conn_connect(conn, sess, name);
	if (status.ret == KSMBD_TREE_CONN_STATUS_OK)
		rsp_hdr->Tid = cpu_to_le16(status.tree_conn->id);
	else
		goto out_err;

	status.ret = 0;
	share = status.tree_conn->share_conf;
	rsp->WordCount = 7;
	rsp->OptionalSupport = 0;

	rsp->OptionalSupport = cpu_to_le16((SMB_SUPPORT_SEARCH_BITS |
				SMB_CSC_NO_CACHING | SMB_UNIQUE_FILE_NAME));

	rsp->MaximalShareAccessRights = cpu_to_le32(FILE_READ_RIGHTS |
					FILE_EXEC_RIGHTS);
	if (test_tree_conn_flag(status.tree_conn,
				KSMBD_TREE_CONN_FLAG_WRITABLE))
		rsp->MaximalShareAccessRights |= cpu_to_le32(FILE_WRITE_RIGHTS);
	rsp->GuestMaximalShareAccessRights = 0;

	set_service_type(conn, share, rsp);

	/* For each extra andx response, we have to add 1 byte,
	 * for wc and 2 bytes for byte count
	 */
	inc_rfc1001_len(rsp_hdr,
		7 * 2 + le16_to_cpu(rsp->ByteCount) + extra_byte);

	/* this is an ANDx command ? */
	rsp->AndXReserved = 0;
	rsp->AndXOffset = cpu_to_le16(get_rfc1002_len(rsp_hdr));
	if (req->AndXCommand != SMB_NO_MORE_ANDX_COMMAND) {
		/* adjust response */
		rsp->AndXCommand = req->AndXCommand;
		/* More processing required */
		status.ret = rsp->AndXCommand;
	} else {
		rsp->AndXCommand = SMB_NO_MORE_ANDX_COMMAND;
	}

	kfree(treename);
	kfree(dev_type);
	kfree(name);

	return status.ret;

out_err:
	if (!IS_ERR(treename))
		kfree(treename);
	if (!IS_ERR(dev_type))
		kfree(dev_type);
	if (!IS_ERR(name))
		kfree(name);

	rsp->WordCount = 7;
	rsp->AndXCommand = SMB_NO_MORE_ANDX_COMMAND;
	rsp->AndXReserved = 0;
	rsp->AndXOffset = cpu_to_le16(get_rfc1002_len(rsp_hdr));
	rsp->OptionalSupport = 0;
	rsp->MaximalShareAccessRights = 0;
	rsp->GuestMaximalShareAccessRights = 0;
	rsp->ByteCount = 0;
	ksmbd_debug(SMB, "error while tree connect\n");
	switch (status.ret) {
	case KSMBD_TREE_CONN_STATUS_NO_SHARE:
		rsp_hdr->Status.CifsError = STATUS_BAD_NETWORK_PATH;
		break;
	case -ENOMEM:
	case KSMBD_TREE_CONN_STATUS_NOMEM:
		rsp_hdr->Status.CifsError = STATUS_NO_MEMORY;
		break;
	case KSMBD_TREE_CONN_STATUS_TOO_MANY_CONNS:
	case KSMBD_TREE_CONN_STATUS_TOO_MANY_SESSIONS:
		rsp_hdr->Status.CifsError = STATUS_ACCESS_DENIED;
		break;
	case -ENODEV:
		rsp_hdr->Status.CifsError = STATUS_BAD_DEVICE_TYPE;
		break;
	case KSMBD_TREE_CONN_STATUS_ERROR:
		rsp_hdr->Status.CifsError = STATUS_BAD_NETWORK_NAME;
		break;
	case -EINVAL:
		rsp_hdr->Status.CifsError = STATUS_INVALID_PARAMETER;
		break;
	default:
		rsp_hdr->Status.CifsError = STATUS_ACCESS_DENIED;
	}

	inc_rfc1001_len(rsp_hdr, (7 * 2 + le16_to_cpu(rsp->ByteCount) +
		extra_byte));
	return -EINVAL;
}

/**
 * smb_get_name() - convert filename on smb packet to char string
 * @src:	source filename, mostly in unicode format
 * @maxlen:	maxlen of src string to be used for parsing
 * @work:	smb work containing smb header flag
 * @converted:	src string already converted to local characterset
 *
 * Return:	pointer to filename string on success, otherwise error ptr
 */
static char *
smb_get_name(struct ksmbd_share_config *share, const char *src,
		const int maxlen, struct ksmbd_work *work, bool converted)
{
	struct smb_hdr *req_hdr = (struct smb_hdr *)work->request_buf;
	bool is_unicode = is_smbreq_unicode(req_hdr);
	char *name, *wild_card_pos;

	if (converted)
		name = (char *)src;
	else {
		name = smb_strndup_from_utf16(src, maxlen, is_unicode,
					      work->conn->local_nls);
		if (IS_ERR(name)) {
			ksmbd_debug(SMB, "failed to get name %ld\n",
				PTR_ERR(name));
			return name;
		}
	}

	ksmbd_conv_path_to_unix(name);
	ksmbd_strip_last_slash(name);

	/*Handling of dir path in FIND_FIRST2 having '*' at end of path*/
	wild_card_pos = strrchr(name, '*');

	if (wild_card_pos != NULL)
		*wild_card_pos = '\0';


	if (ksmbd_validate_filename(name) < 0)
		return ERR_PTR(-ENOENT);

	if (ksmbd_share_veto_filename(share, name)) {
		ksmbd_debug(SMB,
			"file(%s) open is not allowed by setting as veto file\n",
				name);
		if (!converted)
			kfree(name);
		return ERR_PTR(-ENOENT);
	}

	ksmbd_debug(SMB, "file name = %s\n", name);

	return name;
}

/**
 * smb_get_dir_name() - convert directory name on smb packet to char string
 * @src:	source dir name, mostly in unicode format
 * @maxlen:	maxlen of src string to be used for parsing
 * @work:	smb work containing smb header flag
 * @srch_ptr:	update search pointer in dir for searching dir entries
 *
 * Return:	pointer to dir name string on success, otherwise error ptr
 */
static char *smb_get_dir_name(struct ksmbd_share_config *share, const char *src,
		const int maxlen, struct ksmbd_work *work, char **srch_ptr)
{
	struct smb_hdr *req_hdr = (struct smb_hdr *)work->request_buf;
	struct smb_hdr *rsp_hdr = (struct smb_hdr *)work->response_buf;
	bool is_unicode = is_smbreq_unicode(req_hdr);
	char *name, *pattern_pos, *pattern = NULL;
	int pattern_len, rc;

	name = smb_strndup_from_utf16(src, maxlen, is_unicode,
			work->conn->local_nls);
	if (IS_ERR(name)) {
		pr_err("failed to allocate memory\n");
		rsp_hdr->Status.CifsError = STATUS_NO_MEMORY;
		return name;
	}

	ksmbd_conv_path_to_unix(name);
	ksmbd_strip_last_slash(name);

	pattern_pos = strrchr(name, '/');

	if (!pattern_pos)
		pattern_pos = name;
	else
		pattern_pos += 1;

	pattern_len = strlen(pattern_pos);
	if (pattern_len == 0) {
		rc = -EINVAL;
		goto err_name;
	}
	ksmbd_debug(SMB, "pattern searched = %s pattern_len = %d\n",
			pattern_pos, pattern_len);
	pattern = kmalloc(pattern_len + 1, GFP_KERNEL);
	if (!pattern) {
		rc = -ENOMEM;
		goto err_name;
	}
	memcpy(pattern, pattern_pos, pattern_len);
	*(pattern + pattern_len) = '\0';
	*pattern_pos = '\0';
	*srch_ptr = pattern;

	if (ksmbd_validate_filename(name) < 0) {
		rc = -ENOENT;
		goto err_pattern;
	}

	if (ksmbd_share_veto_filename(share, name)) {
		ksmbd_debug(SMB,
			"file(%s) open is not allowed by setting as veto file\n",
				name);
		rc = -ENOENT;
		goto err_pattern;
	}

	ksmbd_debug(SMB, "dir name = %s\n", name);
	return name;

err_pattern:
	kfree(pattern);
err_name:
	kfree(name);

	if (rc == -EINVAL)
		rsp_hdr->Status.CifsError = STATUS_INVALID_PARAMETER;
	else if (rc == -ENOMEM)
		rsp_hdr->Status.CifsError = STATUS_NO_MEMORY;
	else if (rc == -ENOENT)
		rsp_hdr->Status.CifsError = STATUS_OBJECT_NAME_INVALID;

	return ERR_PTR(rc);
}

/**
 * smb_rename() - rename request handler
 * @work:	smb work containing rename request buffer
 *
 * Return:      0 on success, otherwise error
 */
int smb_rename(struct ksmbd_work *work)
{
	struct smb_com_rename_req *req = work->request_buf;
	struct smb_com_rename_rsp *rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	bool is_unicode = is_smbreq_unicode(&req->hdr);
	char *oldname, *newname;
	struct ksmbd_file *fp = NULL;
	int oldname_len;
	struct path path;
	bool file_present = true;
	int rc = 0;

	if (!test_tree_conn_flag(work->tcon, KSMBD_TREE_CONN_FLAG_WRITABLE)) {
		ksmbd_debug(SMB,
			"returning as user does not have permission to write\n");
		rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
		return -EACCES;
	}

	oldname = smb_get_name(share, req->OldFileName, PATH_MAX, work, false);
	if (IS_ERR(oldname)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(oldname);
	}

	if (is_unicode)
		oldname_len = smb1_utf16_name_length((__le16 *)req->OldFileName,
				PATH_MAX);
	else {
		oldname_len = strlen(oldname);
		oldname_len++;
	}

	newname = smb_get_name(share, &req->OldFileName[oldname_len + 2],
			       PATH_MAX, work, false);
	if (IS_ERR(newname)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		rc = PTR_ERR(newname);
		newname = NULL;
		goto out;
	}

	rc = ksmbd_vfs_kern_path(work, newname, LOOKUP_NO_SYMLINKS, &path, 1);
	if (rc)
		file_present = false;
	else
		path_put(&path);

	if (file_present && strncmp(oldname, newname, strlen(oldname))) {
		rc = -EEXIST;
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_COLLISION;
		ksmbd_debug(SMB, "cannot rename already existing file\n");
		goto out;
	}

	ksmbd_debug(SMB, "rename %s -> %s\n", oldname, newname);
	rc = ksmbd_vfs_kern_path(work, oldname, LOOKUP_NO_SYMLINKS, &path, 1);
	if (rc)
		goto out;

	fp = ksmbd_vfs_dentry_open(work, &path, O_RDONLY, 0, false);
	if (IS_ERR(fp)) {
		fp = NULL;
		path_put(&path);
		goto out;
	}

	rc = ksmbd_vfs_fp_rename(work, fp, newname);
	if (rc) {
		rsp->hdr.Status.CifsError = STATUS_NO_MEMORY;
		path_put(&path);
		goto out;
	}
	path_put(&path);
	rsp->hdr.WordCount = 0;
	rsp->ByteCount = 0;
out:
	if (fp)
		ksmbd_close_fd(work, fp->volatile_id);
	kfree(oldname);
	kfree(newname);
	return rc;
}

/**
 * smb_handle_negotiate() - negotiate request handler
 * @work:	smb work containing negotiate request buffer
 *
 * Return:      0 on success, otherwise error
 */
int smb_handle_negotiate(struct ksmbd_work *work)
{
	struct ksmbd_conn *conn = work->conn;
	struct smb_negotiate_rsp *neg_rsp = work->response_buf;
	__u64 time;
	int rc = 0;

	WARN_ON(ksmbd_conn_good(conn));

	if (conn->dialect == BAD_PROT_ID) {
		neg_rsp->hdr.Status.CifsError = STATUS_INVALID_LOGON_TYPE;
		rc = -EINVAL;
		goto err_out;
	}

	conn->connection_type = 0;

	/* wct 17 for NTLM */
	neg_rsp->hdr.WordCount = 17;
	neg_rsp->DialectIndex = cpu_to_le16(conn->dialect);

	neg_rsp->SecurityMode = SMB1_SERVER_SECU;
	if (server_conf.signing == KSMBD_CONFIG_OPT_AUTO ||
		server_conf.signing == KSMBD_CONFIG_OPT_MANDATORY) {
		conn->sign = true;
		neg_rsp->SecurityMode |= SECMODE_SIGN_ENABLED;
		if (server_conf.signing == KSMBD_CONFIG_OPT_MANDATORY)
			neg_rsp->SecurityMode |= SECMODE_SIGN_REQUIRED;
	}
	neg_rsp->MaxMpxCount = cpu_to_le16(SMB1_MAX_MPX_COUNT);
	neg_rsp->MaxNumberVcs = cpu_to_le16(SMB1_MAX_VCS);
	neg_rsp->MaxBufferSize = cpu_to_le32(conn->vals->max_read_size);
	neg_rsp->MaxRawSize = cpu_to_le32(SMB1_MAX_RAW_SIZE);
	neg_rsp->SessionKey = 0;
	neg_rsp->Capabilities = cpu_to_le32(SMB1_SERVER_CAPS);

	time = ksmbd_systime();
	neg_rsp->SystemTimeLow = cpu_to_le32(time & 0x00000000FFFFFFFF);
	neg_rsp->SystemTimeHigh =
		cpu_to_le32((time & 0xFFFFFFFF00000000) >> 32);
	neg_rsp->ServerTimeZone = 0;

	if (((struct smb_hdr *)work->request_buf)->Flags2 & SMBFLG2_EXT_SEC)
		conn->use_spnego = true;

	ksmbd_debug(SMB, "spnego is %s\n", conn->use_spnego ? "on" : "off");

	if (conn->use_spnego == false) {
		neg_rsp->EncryptionKeyLength = CIFS_CRYPTO_KEY_SIZE;
		neg_rsp->Capabilities &= ~cpu_to_le32(CAP_EXTENDED_SECURITY);
		neg_rsp->ByteCount = cpu_to_le16(CIFS_CRYPTO_KEY_SIZE);
		/* initialize random server challenge */
		get_random_bytes(conn->ntlmssp.cryptkey, sizeof(__u64));
		memcpy((neg_rsp->u.EncryptionKey), conn->ntlmssp.cryptkey,
				CIFS_CRYPTO_KEY_SIZE);
		/* Adjust pdu length, 17 words and 8 bytes added */
		inc_rfc1001_len(neg_rsp, (17 * 2 + 8));
	} else {
		neg_rsp->EncryptionKeyLength = 0;
		neg_rsp->ByteCount = cpu_to_le16(SMB1_CLIENT_GUID_SIZE +
			AUTH_GSS_LENGTH);
		get_random_bytes(neg_rsp->u.extended_response.GUID,
			SMB1_CLIENT_GUID_SIZE);
		ksmbd_copy_gss_neg_header(
				neg_rsp->u.extended_response.SecurityBlob);
		inc_rfc1001_len(neg_rsp, (17 * 2 + 16 + AUTH_GSS_LENGTH));
	}

	/* Null terminated domain name in unicode */

	ksmbd_conn_set_need_negotiate(conn);
	/* Domain name and PC name are ignored by clients, so no need to send.
	 * We can try sending them later
	 */
err_out:
	return rc;
}

static int build_sess_rsp_noextsec(struct ksmbd_conn *conn,
		struct ksmbd_session *sess,
		struct smb_com_session_setup_req_no_secext *req,
		struct smb_com_session_setup_old_resp *rsp)
{
	int offset, err = 0, len;
	char *name;
	__le16 str[32];

	/* Build response. We don't use extended security (yet), so wct is 3 */
	rsp->hdr.WordCount = 3;
	rsp->Action = 0;
	/* The names should be unicode */
	rsp->ByteCount = 0;
	/* adjust pdu length. data added 6 bytes */
	inc_rfc1001_len(&rsp->hdr, 6);

	/* check if valid user name is present in request or not */
	offset = le16_to_cpu(req->CaseInsensitivePasswordLength) +
		le16_to_cpu(req->CaseSensitivePasswordLength);

	/* 1 byte for padding */
	name = smb_strndup_from_utf16((req->CaseInsensitivePassword + offset +
				1), 256, true, conn->local_nls);
	if (IS_ERR(name)) {
		pr_err("cannot allocate memory\n");
		err = PTR_ERR(name);
		goto out_err;
	}

	WARN_ON(sess->user);

	ksmbd_debug(SMB, "session setup request for user %s\n", name);
	sess->user = ksmbd_login_user(name);
	kfree(name);
	if (!sess->user) {
		pr_err("user not present in database\n");
		err = -EINVAL;
		goto out_err;
	}

	if (user_guest(sess->user)) {
		rsp->Action = cpu_to_le16(GUEST_LOGIN);
		goto no_password_check;
	}

	if (le16_to_cpu(req->CaseSensitivePasswordLength) ==
			CIFS_AUTH_RESP_SIZE) {
		err = ksmbd_auth_ntlm(sess, req->CaseInsensitivePassword +
			le16_to_cpu(req->CaseInsensitivePasswordLength),
			conn->ntlmssp.cryptkey);
		if (err) {
			pr_err("ntlm authentication failed for user %s\n",
			       user_name(sess->user));
			goto out_err;
		}
	} else {
		char *ntdomain;

		offset = le16_to_cpu(req->CaseInsensitivePasswordLength) +
			le16_to_cpu(req->CaseSensitivePasswordLength) +
			((strlen(user_name(sess->user)) + 1) * 2);

		ntdomain = smb_strndup_from_utf16(
				req->CaseInsensitivePassword +
				offset + 1, 256, true, conn->local_nls);
		if (IS_ERR(ntdomain)) {
			pr_err("cannot allocate memory\n");
			err = PTR_ERR(ntdomain);
			goto out_err;
		}

		err = ksmbd_auth_ntlmv2(conn, sess,
			(struct ntlmv2_resp *) ((char *)
			req->CaseInsensitivePassword +
			le16_to_cpu(req->CaseInsensitivePasswordLength)),
			le16_to_cpu(req->CaseSensitivePasswordLength) -
				CIFS_ENCPWD_SIZE, ntdomain,
				conn->ntlmssp.cryptkey);
		kfree(ntdomain);
		if (err) {
			pr_err("authentication failed for user %s\n",
			       user_name(sess->user));
			goto out_err;
		}
	}

no_password_check:
	/* this is an ANDx command ? */
	rsp->AndXReserved = 0;
	rsp->AndXOffset = cpu_to_le16(get_rfc1002_len(&rsp->hdr));

	/* 1 byte padding for word alignment */
	offset = 1;

	memset(str, 0 , sizeof(str));

	len = smb_strtoUTF16(str, "Unix", 4, conn->local_nls);
	len = UNICODE_LEN(len + 1);
	memcpy(rsp->NativeOS + offset, str, len);
	offset += len;

	len = smb_strtoUTF16(str, "ksmbd", 5, conn->local_nls);
	len = UNICODE_LEN(len + 1);
	memcpy(rsp->NativeOS + offset, str, len);
	offset += len;

	len = smb_strtoUTF16(str, "WORKGROUP", 9, conn->local_nls);
	len = UNICODE_LEN(len + 1);
	memcpy(rsp->NativeOS + offset, str, len);
	offset += len;

	rsp->ByteCount = cpu_to_le16(offset);
	inc_rfc1001_len(&rsp->hdr, offset);

	if (req->AndXCommand != SMB_NO_MORE_ANDX_COMMAND) {
		/* adjust response */
		rsp->AndXCommand = req->AndXCommand;
		return rsp->AndXCommand; /* More processing required */
	}
	rsp->AndXCommand = SMB_NO_MORE_ANDX_COMMAND;

out_err:
	return err;
}

static int build_sess_rsp_extsec(struct ksmbd_conn *conn,
		struct ksmbd_session *sess,
		struct smb_com_session_setup_req *req,
		struct smb_com_session_setup_resp *rsp)
{
	struct negotiate_message *negblob;
	char *neg_blob;
	int err = 0, neg_blob_len;
	unsigned char *spnego_blob;
	u16 spnego_blob_len;
	int sz;

	rsp->hdr.WordCount = 4;
	rsp->Action = 0;

	/* The names should be unicode */
	rsp->ByteCount = 0;
	/* adjust pdu length. data added 6 bytes */
	inc_rfc1001_len(&rsp->hdr, 8);

	negblob = (struct negotiate_message *)req->SecurityBlob;
	sz = le16_to_cpu(req->SecurityBlobLength);

	if (ksmbd_decode_negTokenInit((char *)negblob, sz, conn)) {
		if (ksmbd_decode_negTokenTarg((char *)negblob, sz, conn)) {
			conn->use_spnego = false;
		}
	}

	if (conn->mechToken)
		negblob = (struct negotiate_message *)conn->mechToken;

	if (negblob->MessageType == NtLmNegotiate) {
		struct challenge_message *chgblob;

		ksmbd_debug(SMB, "negotiate phase\n");
		err = ksmbd_decode_ntlmssp_neg_blob(negblob,
				le16_to_cpu(req->SecurityBlobLength),
				conn);
		if (err)
			goto out_err;

		chgblob = (struct challenge_message *)rsp->SecurityBlob;
		memset(chgblob, 0, sizeof(struct challenge_message));

		if (conn->use_spnego) {
			int sz;

			sz = sizeof(struct negotiate_message) +
				(strlen(ksmbd_netbios_name()) * 2 + 1 + 4) * 6;
			neg_blob = kmalloc(sz, GFP_KERNEL);
			if (!neg_blob) {
				err = -ENOMEM;
				goto out_err;
			}
			chgblob = (struct challenge_message *)neg_blob;
			neg_blob_len = ksmbd_build_ntlmssp_challenge_blob(
					chgblob,
					conn);
			if (neg_blob_len < 0) {
				kfree(neg_blob);
				err = -ENOMEM;
				goto out_err;
			}

			if (build_spnego_ntlmssp_neg_blob(&spnego_blob,
						&spnego_blob_len,
						neg_blob, neg_blob_len)) {
				kfree(neg_blob);
				err = -ENOMEM;
				goto out_err;
			}

			memcpy((char *)rsp->SecurityBlob, spnego_blob,
					spnego_blob_len);
			rsp->SecurityBlobLength =
				cpu_to_le16(spnego_blob_len);
			kfree(spnego_blob);
			kfree(neg_blob);
		} else {
			neg_blob_len = ksmbd_build_ntlmssp_challenge_blob(
					chgblob,
					conn);
			if (neg_blob_len < 0) {
				err = -ENOMEM;
				goto out_err;
			}

			rsp->SecurityBlobLength = cpu_to_le16(neg_blob_len);
		}

		rsp->hdr.Status.CifsError = STATUS_MORE_PROCESSING_REQUIRED;
		/*
		 * Note: here total size -1 is done as an adjustment
		 * for 0 size blob.
		 */
		inc_rfc1001_len(rsp, le16_to_cpu(rsp->SecurityBlobLength));
		rsp->ByteCount = rsp->SecurityBlobLength;
	} else if (negblob->MessageType == NtLmAuthenticate) {
		struct authenticate_message *authblob;
		char *username;

		ksmbd_debug(SMB, "authenticate phase\n");
		if (conn->use_spnego && conn->mechToken)
			authblob =
				(struct authenticate_message *)conn->mechToken;
		else
			authblob = (struct authenticate_message *)
						req->SecurityBlob;

		username = smb_strndup_from_utf16((const char *)authblob +
				le32_to_cpu(authblob->UserName.BufferOffset),
				le16_to_cpu(authblob->UserName.Length), true,
				conn->local_nls);

		if (IS_ERR(username)) {
			pr_err("cannot allocate memory\n");
			err = PTR_ERR(username);
			goto out_err;
		}

		ksmbd_debug(SMB, "session setup request for user %s\n",
			username);
		sess->user = ksmbd_login_user(username);
		kfree(username);

		if (!sess->user) {
			ksmbd_debug(SMB, "Unknown user name or an error\n");
			err = -EINVAL;
			goto out_err;
		}

		if (user_guest(sess->user)) {
			rsp->Action = cpu_to_le16(GUEST_LOGIN);
			goto no_password_check;
		}

		err = ksmbd_decode_ntlmssp_auth_blob(authblob,
				le16_to_cpu(req->SecurityBlobLength),
				conn, sess);
		if (err) {
			ksmbd_debug(SMB, "authentication failed\n");
			err = -EINVAL;
			goto out_err;
		}

no_password_check:
		if (conn->use_spnego) {
			if (build_spnego_ntlmssp_auth_blob(&spnego_blob,
						&spnego_blob_len, 0)) {
				err = -ENOMEM;
				goto out_err;
			}

			memcpy((char *)rsp->SecurityBlob, spnego_blob,
					spnego_blob_len);
			rsp->SecurityBlobLength =
				cpu_to_le16(spnego_blob_len);
			kfree(spnego_blob);
			inc_rfc1001_len(rsp, spnego_blob_len);
			rsp->ByteCount = rsp->SecurityBlobLength;
		}
	} else {
		pr_err("Invalid phase\n");
		err = -EINVAL;
	}

	/* this is an ANDx command ? */
	rsp->AndXReserved = 0;
	rsp->AndXOffset = cpu_to_le16(get_rfc1002_len(&rsp->hdr));

	if (req->AndXCommand != SMB_NO_MORE_ANDX_COMMAND) {
		/* adjust response */
		rsp->AndXCommand = req->AndXCommand;
		return rsp->AndXCommand; /* More processing required */
	}
	rsp->AndXCommand = SMB_NO_MORE_ANDX_COMMAND;

out_err:
	if (conn->use_spnego && conn->mechToken) {
		kfree(conn->mechToken);
		conn->mechToken = NULL;
	}

	return err;
}

/**
 * smb_session_setup_andx() - session setup request handler
 * @work:   smb work containing session setup request buffer
 *
 * Return:      0 on success, otherwise error
 */
int smb_session_setup_andx(struct ksmbd_work *work)
{
	struct ksmbd_conn *conn = work->conn;
	struct ksmbd_session *sess = NULL;
	int rc = 0, cap;
	unsigned short uid;

	union smb_com_session_setup_andx *pSMB = work->request_buf;
	union smb_com_session_setup_andx *rsp = work->response_buf;

	if (pSMB->req.hdr.WordCount == 12)
		cap = le32_to_cpu(pSMB->req.Capabilities);
	else if (pSMB->req.hdr.WordCount == 13)
		cap = le32_to_cpu(pSMB->req_no_secext.Capabilities);
	else {
		pr_err("malformed packet\n");
		work->send_no_response = 1;
		return 0;
	}

	uid = le16_to_cpu(pSMB->req.hdr.Uid);
	if (uid != 0) {
		sess = ksmbd_session_lookup(conn, uid);
		if (!sess) {
			rc = -ENOENT;
			goto out_err;
		}
		ksmbd_debug(SMB, "Reuse session ID: %llu, Uid: %u\n",
			    sess->id, uid);
	} else {
		sess = ksmbd_smb1_session_create();
		if (!sess) {
			rc = -ENOMEM;
			goto out_err;
		}

		rc = ksmbd_session_register(conn, sess);
		if (rc)
			goto out_err;
		rsp->resp.hdr.Uid = cpu_to_le16(sess->id);
		ksmbd_debug(SMB, "New session ID: %llu, Uid: %u\n", sess->id,
			uid);
	}

	if (cap & CAP_EXTENDED_SECURITY) {
		ksmbd_debug(SMB, "build response with extend_security\n");
		rc = build_sess_rsp_extsec(conn, sess, &pSMB->req, &rsp->resp);

	} else {
		ksmbd_debug(SMB, "build response without extend_security\n");
		rc = build_sess_rsp_noextsec(conn, sess, &pSMB->req_no_secext,
				&rsp->old_resp);
	}
	if (rc < 0)
		goto out_err;

	work->sess = sess;
	ksmbd_conn_set_good(conn);
	return 0;

out_err:
	rsp->resp.hdr.Status.CifsError = STATUS_LOGON_FAILURE;
	rsp->resp.hdr.WordCount = 0;
	rsp->resp.ByteCount = 0;
	if (rc < 0 && sess) {
		xa_erase(&conn->sessions, sess->id);
		ksmbd_session_destroy(sess);
		work->sess = NULL;
	}
	return rc;
}

/**
 * file_create_dispostion_flags() - convert disposition flags to
 *				file open flags
 * @dispostion:		file disposition contained in open request
 * @file_present:	file already present or not
 *
 * Return:      file open flags after conversion from disposition
 */
static int file_create_dispostion_flags(int dispostion, bool file_present)
{
	int disp_flags = 0;

	switch (dispostion) {
	/*
	 * If the file already exists, it SHOULD be superseded (overwritten).
	 * If it does not already exist, then it SHOULD be created.
	 */
	case FILE_SUPERSEDE:
		if (file_present)
			disp_flags |= O_TRUNC;
		else
			disp_flags |= O_CREAT;
		break;
	/*
	 * If the file already exists, it SHOULD be opened rather than created.
	 * If the file does not already exist, the operation MUST fail.
	 */
	case FILE_OPEN:
		if (!file_present)
			return -ENOENT;
		break;
	/*
	 * If the file already exists, the operation MUST fail.
	 * If the file does not already exist, it SHOULD be created.
	 */
	case FILE_CREATE:
		if (file_present)
			return -EEXIST;
		disp_flags |= O_CREAT;
		break;
	/*
	 * If the file already exists, it SHOULD be opened. If the file
	 * does not already exist, then it SHOULD be created.
	 */
	case FILE_OPEN_IF:
		if (!file_present)
			disp_flags |= O_CREAT;
		break;
	/*
	 * If the file already exists, it SHOULD be opened and truncated.
	 * If the file does not already exist, the operation MUST fail.
	 */
	case FILE_OVERWRITE:
		if (!file_present)
			return -ENOENT;
		disp_flags |= O_TRUNC;
		break;
	/*
	 * If the file already exists, it SHOULD be opened and truncated.
	 * If the file does not already exist, it SHOULD be created.
	 */
	case FILE_OVERWRITE_IF:
		if (file_present)
			disp_flags |= O_TRUNC;
		else
			disp_flags |= O_CREAT;
		break;
	default:
		return -EINVAL;
	}

	return disp_flags;
}

static inline int ksmbd_openflags_to_mayflags(int open_flags)
{
	int mask = open_flags & O_ACCMODE;

	if (mask == O_WRONLY)
		return MAY_OPEN | MAY_WRITE;
	else if (mask == O_RDWR)
		return MAY_OPEN | MAY_READ | MAY_WRITE;
	else
		return MAY_OPEN | MAY_READ;
}

/**
 * convert_generic_access_flags() - convert access flags to
 *				file open flags
 * @access_flag:	file access flags contained in open request
 * @open_flag:		file open flags are updated as per access flags
 * @may_flags:		file may flags are updated with @open_flags
 * @attrib:		attribute flag indicating posix symantics or not
 *
 * Return:		access flags
 */
static int
convert_generic_access_flags(int access_flag, int *open_flags,
			     int *may_flags, int attrib)
{
	int aflags = access_flag;
	int oflags = *open_flags;

	if (aflags & GENERIC_READ) {
		aflags &= ~GENERIC_READ;
		aflags |= GENERIC_READ_FLAGS;
	}

	if (aflags & GENERIC_WRITE) {
		aflags &= ~GENERIC_WRITE;
		aflags |= GENERIC_WRITE_FLAGS;
	}

	if (aflags & GENERIC_EXECUTE) {
		aflags &= ~GENERIC_EXECUTE;
		aflags |= GENERIC_EXECUTE_FLAGS;
	}

	if (aflags & GENERIC_ALL) {
		aflags &= ~GENERIC_ALL;
		aflags |= GENERIC_ALL_FLAGS;
	}

	if (oflags & O_TRUNC)
		aflags |= FILE_WRITE_DATA;

	if (aflags & (FILE_WRITE_DATA | FILE_APPEND_DATA)) {
		if (aflags & (FILE_READ_ATTRIBUTES | FILE_READ_DATA |
					FILE_READ_EA | FILE_EXECUTE)) {
			*open_flags |= O_RDWR;

		} else {
			*open_flags |= O_WRONLY;
		}
	} else {
		*open_flags |= O_RDONLY;
	}

	if ((attrib & ATTR_POSIX_SEMANTICS) && (aflags & FILE_APPEND_DATA))
		*open_flags |= O_APPEND;

	*may_flags = ksmbd_openflags_to_mayflags(*open_flags);

	return aflags;
}

/**
 * smb_get_dos_attr() - convert unix style stat info to dos attr
 * @stat:	stat to be converted to dos attr
 *
 * Return:	dos style attribute
 */
static __u32 smb_get_dos_attr(struct kstat *stat)
{
	__u32 attr = 0;

	/* check whether file has attributes ATTR_READONLY, ATTR_HIDDEN,
	 * ATTR_SYSTEM, ATTR_VOLUME, ATTR_DIRECTORY, ATTR_ARCHIVE,
	 * ATTR_DEVICE, ATTR_NORMAL, ATTR_TEMPORARY, ATTR_SPARSE,
	 * ATTR_REPARSE, ATTR_COMPRESSED, ATTR_OFFLINE
	 */

	if (stat->mode & S_ISVTX)   /* hidden */
		attr |=  (ATTR_HIDDEN | ATTR_SYSTEM);

	if (!(stat->mode & 0222))  /* read-only */
		attr |=  ATTR_READONLY;

	if (S_ISDIR(stat->mode))
		attr |= ATTR_DIRECTORY;

	if (stat->size > (stat->blksize * stat->blocks))
		attr |= ATTR_SPARSE;

	if (!attr)
		attr |= ATTR_NORMAL;

	return attr;
}

static int
lock_oplock_release(struct ksmbd_file *fp, int type, int oplock_level)
{
	struct oplock_info *opinfo;
	int ret;

	ksmbd_debug(SMB, "got oplock brk for level OplockLevel = %d\n",
		      oplock_level);

	opinfo = fp->f_opinfo;
	if (opinfo->op_state == OPLOCK_STATE_NONE) {
		pr_err("unexpected oplock state 0x%x\n", opinfo->op_state);
		return -EINVAL;
	}

	if (oplock_level == OPLOCK_EXCLUSIVE || oplock_level == OPLOCK_BATCH) {
		if (opinfo_write_to_none(opinfo) < 0) {
			opinfo->op_state = OPLOCK_STATE_NONE;
			return -EINVAL;
		}
	} else if (((opinfo->level == OPLOCK_EXCLUSIVE) ||
				(opinfo->level == OPLOCK_BATCH)) &&
			(oplock_level == OPLOCK_READ)) {
		ret = opinfo_write_to_read(opinfo);
		if (ret) {
			opinfo->op_state = OPLOCK_STATE_NONE;
			return -EINVAL;
		}
	} else if ((opinfo->level == OPLOCK_READ) &&
			(oplock_level == OPLOCK_NONE)) {
		ret = opinfo_read_to_none(opinfo);
		if (ret) {
			opinfo->op_state = OPLOCK_STATE_NONE;
			return -EINVAL;
		}
	}

	opinfo->op_state = OPLOCK_STATE_NONE;
	wake_up_interruptible(&opinfo->oplock_q);

	return 0;
}

static struct ksmbd_lock *smb_lock_init(struct file_lock *flock,
		unsigned int cmd, int mode, unsigned long long offset,
		unsigned long long length, struct list_head *lock_list)
{
	struct ksmbd_lock *lock;

	lock = kzalloc(sizeof(struct ksmbd_lock), GFP_KERNEL);
	if (!lock)
		return NULL;

	lock->cmd = cmd;
	lock->fl = flock;
	lock->start = offset;
	lock->end = offset + length;
	lock->flags = mode;
	if (lock->start == lock->end)
		lock->zero_len = 1;
	INIT_LIST_HEAD(&lock->llist);
	INIT_LIST_HEAD(&lock->clist);
	INIT_LIST_HEAD(&lock->flist);
	list_add_tail(&lock->llist, lock_list);

	return lock;
}

/**
 * smb_locking_andx() - received oplock break response from client
 * @work:	smb work containing oplock break command
 *
 * Return:	0 on success, otherwise error
 */
int smb_locking_andx(struct ksmbd_work *work)
{
	struct smb_com_lock_req *req = work->request_buf;
	struct smb_com_lock_rsp *rsp = work->response_buf;
	struct ksmbd_file *fp;
	int err = 0;
	struct locking_andx_range32 *lock_ele32 = NULL, *unlock_ele32 = NULL;
	struct locking_andx_range64 *lock_ele64 = NULL, *unlock_ele64 = NULL;
	struct file *filp = NULL;
	struct ksmbd_lock *smb_lock = NULL, *cmp_lock, *tmp, *tmp2;
	int i, lock_count, unlock_count;
	unsigned long long offset, length;
	struct file_lock *flock = NULL;
	unsigned int cmd = 0;
	LIST_HEAD(lock_list);
	LIST_HEAD(rollback_list);
	int locked, timeout;
	const unsigned long long loff_max = ~0;
	struct ksmbd_conn *conn;

	timeout = le32_to_cpu(req->Timeout);
	ksmbd_debug(SMB, "got oplock brk for fid %d lock type = 0x%x, timeout : %d\n",
		      req->Fid, req->LockType, timeout);

	/* find fid */
	fp = ksmbd_lookup_fd_fast(work, req->Fid);
	if (!fp) {
		pr_err("cannot obtain fid for %d\n", req->Fid);
		return -EINVAL;
	}

	if (req->LockType & LOCKING_ANDX_OPLOCK_RELEASE) {
		pr_err("lock type is oplock release\n");
		err = lock_oplock_release(fp, req->LockType, req->OplockLevel);
	}

	filp = fp->filp;
	lock_count = le16_to_cpu(req->NumberOfLocks);
	unlock_count = le16_to_cpu(req->NumberOfUnlocks);

	ksmbd_debug(SMB, "lock count is %d, unlock_count : %d\n",
		lock_count, unlock_count);

	if (req->LockType & LOCKING_ANDX_LARGE_FILES)
		lock_ele64 = (struct locking_andx_range64 *)req->Locks;
	else
		lock_ele32 = (struct locking_andx_range32 *)req->Locks;

	if (req->LockType & LOCKING_ANDX_CHANGE_LOCKTYPE) {
		pr_err("lock type: LOCKING_ANDX_CHANGE_LOCKTYPE\n");
		rsp->hdr.Status.DosError.ErrorClass = ERRDOS;
		rsp->hdr.Status.DosError.Error = cpu_to_le16(ERRnoatomiclocks);
		rsp->hdr.Flags2 &= ~SMBFLG2_ERR_STATUS;
		goto out;
	}

	if (req->LockType & LOCKING_ANDX_CANCEL_LOCK)
		pr_err("lock type: LOCKING_ANDX_CANCEL_LOCK\n");

	for (i = 0; i < lock_count; i++) {
		flock = smb_flock_init(filp);
		if (!flock)
			goto out;

		if (req->LockType & LOCKING_ANDX_SHARED_LOCK) {
			pr_err("received shared request\n");
			if (!(filp->f_mode & FMODE_READ)) {
				rsp->hdr.Status.CifsError =
					STATUS_ACCESS_DENIED;
				locks_free_lock(flock);
				goto out;
			}
			cmd = F_SETLKW;
			flock->fl_type = F_RDLCK;
		} else {
			pr_err("received exclusive request\n");
			if (!(filp->f_mode & FMODE_WRITE)) {
				rsp->hdr.Status.CifsError =
					STATUS_ACCESS_DENIED;
				locks_free_lock(flock);
				goto out;
			}
			cmd = F_SETLKW;
			flock->fl_type = F_WRLCK;
			flock->fl_flags |= FL_SLEEP;
		}

		if (req->LockType & LOCKING_ANDX_LARGE_FILES) {
			offset = (unsigned long long)le32_to_cpu(
					lock_ele64[i].OffsetLow);
			length = (unsigned long long)le32_to_cpu(
					lock_ele64[i].LengthLow);
			offset |= (unsigned long long)le32_to_cpu(
					lock_ele64[i].OffsetHigh) << 32;
			length |= (unsigned long long)le32_to_cpu(
					lock_ele64[i].LengthHigh) << 32;
		} else {
			offset = (unsigned long long)le32_to_cpu(
				lock_ele32[i].Offset);
			length = (unsigned long long)le32_to_cpu(
				lock_ele32[i].Length);
		}

		if (offset > loff_max) {
			pr_err("Invalid lock range requested\n");
			rsp->hdr.Status.CifsError = STATUS_INVALID_LOCK_RANGE;
			locks_free_lock(flock);
			goto out;
		}

		if (offset > 0 && length > (loff_max - offset) + 1) {
			pr_err("Invalid lock range requested\n");
			rsp->hdr.Status.CifsError = STATUS_INVALID_LOCK_RANGE;
			locks_free_lock(flock);
			goto out;
		}

		ksmbd_debug(SMB, "locking offset : %llx, length : %llu\n",
			offset, length);

		if (offset > OFFSET_MAX)
			flock->fl_start = OFFSET_MAX;
		else
			flock->fl_start = offset;
		if (offset + length > OFFSET_MAX)
			flock->fl_end = OFFSET_MAX;
		else
			flock->fl_end = offset + length;

		smb_lock = smb_lock_init(flock, cmd, req->LockType, offset,
			length, &lock_list);
		if (!smb_lock) {
			locks_free_lock(flock);
			goto out;
		}
	}

	list_for_each_entry_safe(smb_lock, tmp, &lock_list, llist) {
		int same_zero_lock = 0;

		list_del(&smb_lock->llist);
		/* check locks in connections */
		down_read(&conn_list_lock);
		list_for_each_entry(conn, &conn_list, conns_list) {
			spin_lock(&conn->llist_lock);
			list_for_each_entry_safe(cmp_lock, tmp2, &conn->lock_list, clist) {
				if (file_inode(cmp_lock->fl->fl_file) !=
					file_inode(smb_lock->fl->fl_file))
					continue;

				if (smb_lock->zero_len &&
					cmp_lock->start == smb_lock->start &&
					cmp_lock->end == smb_lock->end) {
					same_zero_lock = 1;
					spin_unlock(&conn->llist_lock);
					up_read(&conn_list_lock);
					goto out_check_cl;
				}

				/* check zero byte lock range */
				if (cmp_lock->zero_len && !smb_lock->zero_len &&
						cmp_lock->start > smb_lock->start &&
						cmp_lock->start < smb_lock->end) {
					pr_err("previous lock conflict with zero byte lock range\n");
					err = -EPERM;
				} else if (smb_lock->zero_len && !cmp_lock->zero_len &&
					smb_lock->start > cmp_lock->start &&
					smb_lock->start < cmp_lock->end) {
					pr_err("current lock conflict with zero byte lock range\n");
					err = -EPERM;
				} else if (((cmp_lock->start <= smb_lock->start &&
					cmp_lock->end > smb_lock->start) ||
					(cmp_lock->start < smb_lock->end &&
					 cmp_lock->end >= smb_lock->end)) &&
					!cmp_lock->zero_len && !smb_lock->zero_len) {
					pr_err("Not allow lock operation on exclusive lock range\n");
					err = -EPERM;
				}

				if (err) {
					/* Clean error cache */
					if ((smb_lock->zero_len &&
							fp->cflock_cnt > 1) ||
						(timeout && (fp->llock_fstart ==
								smb_lock->start))) {
						ksmbd_debug(SMB, "clean error cache\n");
						fp->cflock_cnt = 0;
					}

					if (timeout > 0 ||
						(fp->cflock_cnt > 0 &&
						fp->llock_fstart == smb_lock->start) ||
						((smb_lock->start >> 63) == 0 &&
						smb_lock->start >= 0xEF000000)) {
						if (timeout) {
							spin_unlock(&conn->llist_lock);
							up_read(&conn_list_lock);
							ksmbd_debug(SMB, "waiting error response for timeout : %d\n",
								timeout);
							msleep(timeout);
						}
						rsp->hdr.Status.CifsError =
							STATUS_FILE_LOCK_CONFLICT;
					} else
						rsp->hdr.Status.CifsError =
							STATUS_LOCK_NOT_GRANTED;
					fp->cflock_cnt++;
					fp->llock_fstart = smb_lock->start;

					if (timeout <= 0) {
						spin_unlock(&conn->llist_lock);
						up_read(&conn_list_lock);
					}
					goto out;
				}
			}
			spin_unlock(&conn->llist_lock);
		}
		up_read(&conn_list_lock);

out_check_cl:
		if (same_zero_lock)
			continue;
		if (smb_lock->zero_len) {
			err = 0;
			goto skip;
		}

		flock = smb_lock->fl;
retry:
		err = vfs_lock_file(filp, smb_lock->cmd, flock, NULL);
		if (err == FILE_LOCK_DEFERRED) {
			pr_err("would have to wait for getting lock\n");
			spin_lock(&work->conn->llist_lock);
			list_add_tail(&smb_lock->clist,
				      &work->conn->lock_list);
			spin_unlock(&work->conn->llist_lock);
			list_add(&smb_lock->llist, &rollback_list);
wait:
			err = ksmbd_vfs_posix_lock_wait_timeout(flock,
							msecs_to_jiffies(10));
			if (err) {
				list_del(&smb_lock->llist);
				spin_lock(&work->conn->llist_lock);
				list_del(&smb_lock->clist);
				spin_unlock(&work->conn->llist_lock);
				goto retry;
			} else
				goto wait;
		} else if (!err) {
skip:
			spin_lock(&work->conn->llist_lock);
			list_add_tail(&smb_lock->clist,
				      &work->conn->lock_list);
			list_add_tail(&smb_lock->flist,
				      &fp->lock_list);
			spin_unlock(&work->conn->llist_lock);
			list_add(&smb_lock->llist, &rollback_list);
			pr_err("successful in taking lock\n");
		} else if (err < 0) {
			rsp->hdr.Status.CifsError = STATUS_LOCK_NOT_GRANTED;
			goto out;
		}
	}

	if (req->LockType & LOCKING_ANDX_LARGE_FILES)
		unlock_ele64 = (struct locking_andx_range64 *)(req->Locks +
				(sizeof(struct locking_andx_range64) *
				 lock_count));
	else
		unlock_ele32 = (struct locking_andx_range32 *)(req->Locks +
				(sizeof(struct locking_andx_range32) *
				 lock_count));

	for (i = 0; i < unlock_count; i++) {
		flock = smb_flock_init(filp);
		if (!flock)
			goto out;

		flock->fl_type = F_UNLCK;
		cmd = 0;

		if (req->LockType & LOCKING_ANDX_LARGE_FILES) {
			offset = (unsigned long long)le32_to_cpu(
					unlock_ele64[i].OffsetLow);
			length = (unsigned long long)le32_to_cpu(
					unlock_ele64[i].LengthLow);
			offset |= (unsigned long long)le32_to_cpu(
					unlock_ele64[i].OffsetHigh) << 32;
			length |= (unsigned long long)le32_to_cpu(
					unlock_ele64[i].LengthHigh) << 32;
		} else {
			offset = (unsigned long long)le32_to_cpu(
				unlock_ele32[i].Offset);
			length = (unsigned long long)le32_to_cpu(
				unlock_ele32[i].Length);
		}

		ksmbd_debug(SMB, "unlock offset : %llx, length : %llu\n",
			offset, length);

		if (offset > OFFSET_MAX)
			flock->fl_start = OFFSET_MAX;
		else
			flock->fl_start = offset;
		if (offset + length > OFFSET_MAX)
			flock->fl_end = OFFSET_MAX;
		else
			flock->fl_end = offset + length;

		locked = 0;
		up_read(&conn_list_lock);
		list_for_each_entry(conn, &conn_list, conns_list) {
			spin_lock(&conn->llist_lock);
			list_for_each_entry(cmp_lock, &conn->lock_list, clist) {
				if (file_inode(cmp_lock->fl->fl_file) !=
					file_inode(flock->fl_file))
					continue;

				if ((cmp_lock->start == offset &&
					 cmp_lock->end == offset + length)) {
					locked = 1;
					spin_unlock(&conn->llist_lock);
					up_read(&conn_list_lock);
					goto out_check_cl_unlck;
				}
			}
			spin_unlock(&conn->llist_lock);
		}
		up_read(&conn_list_lock);

out_check_cl_unlck:
		if (!locked) {
			locks_free_lock(flock);
			rsp->hdr.Status.CifsError = STATUS_RANGE_NOT_LOCKED;
			goto out;
		}

		err = vfs_lock_file(filp, cmd, flock, NULL);
		if (!err) {
			ksmbd_debug(SMB, "File unlocked\n");
			spin_lock(&conn->llist_lock);
			if (!list_empty(&cmp_lock->flist))
				list_del(&cmp_lock->flist);
			list_del(&cmp_lock->clist);
			spin_unlock(&conn->llist_lock);

			locks_free_lock(cmp_lock->fl);
			kfree(cmp_lock);
			fp->cflock_cnt = 0;
		} else if (err == -ENOENT) {
			rsp->hdr.Status.CifsError = STATUS_RANGE_NOT_LOCKED;
			locks_free_lock(flock);
			goto out;
		}
		locks_free_lock(flock);
	}

	rsp->hdr.WordCount = 2;
	rsp->ByteCount = 0;
	inc_rfc1001_len(&rsp->hdr, (rsp->hdr.WordCount * 2));

	/* this is an ANDx command ? */
	rsp->AndXReserved = 0;
	rsp->AndXOffset = cpu_to_le16(get_rfc1002_len(&rsp->hdr));
	if (req->AndXCommand != SMB_NO_MORE_ANDX_COMMAND) {
		/* adjust response */
		rsp->AndXCommand = req->AndXCommand;
		return rsp->AndXCommand; /* More processing required */
	}
	rsp->AndXCommand = SMB_NO_MORE_ANDX_COMMAND;
	ksmbd_fd_put(work, fp);
	return err;

out:
	list_for_each_entry_safe(smb_lock, tmp, &lock_list, llist) {
		locks_free_lock(smb_lock->fl);
		list_del(&smb_lock->llist);
		kfree(smb_lock);
	}

	list_for_each_entry_safe(smb_lock, tmp, &rollback_list, llist) {
		struct file_lock *rlock = NULL;

		rlock = smb_flock_init(filp);
		rlock->fl_type = F_UNLCK;
		rlock->fl_start = smb_lock->start;
		rlock->fl_end = smb_lock->end;

		err = vfs_lock_file(filp, 0, rlock, NULL);
		if (err)
			pr_err("rollback unlock fail : %d\n", err);

		list_del(&smb_lock->llist);
		spin_lock(&work->conn->llist_lock);
		if (!list_empty(&smb_lock->flist))
			list_del(&smb_lock->flist);
		list_del(&smb_lock->clist);
		spin_unlock(&work->conn->llist_lock);

		locks_free_lock(smb_lock->fl);
		locks_free_lock(rlock);
		kfree(smb_lock);
	}

	ksmbd_fd_put(work, fp);
	pr_err("failed in taking lock\n");
	return err;
}

/**
 * smb_trans() - trans2 command dispatcher
 * @work:	smb work containing trans2 command
 *
 * Return:	0 on success, otherwise error
 */
int smb_trans(struct ksmbd_work *work)
{
	struct ksmbd_conn *conn = work->conn;
	struct smb_com_trans_req *req = work->request_buf;
	struct smb_com_trans_rsp *rsp = work->response_buf;
	struct smb_com_trans_pipe_req *pipe_req = work->request_buf;
	struct ksmbd_rpc_command *rpc_resp;
	__u16 subcommand;
	char *name, *pipe;
	char *pipedata;
	int setup_bytes_count = 0;
	int pipe_name_offset = 0;
	int str_len_uni;
	int ret = 0, nbytes = 0;
	int param_len = 0;
	int id;
	int padding;

	if (req->SetupCount)
		setup_bytes_count = 2 * req->SetupCount;

	subcommand = le16_to_cpu(req->SubCommand);
	name = smb_strndup_from_utf16(req->Data + setup_bytes_count, 256, 1,
			conn->local_nls);

	if (IS_ERR(name)) {
		pr_err("failed to allocate memory\n");
		rsp->hdr.Status.CifsError = STATUS_NO_MEMORY;
		return PTR_ERR(name);
	}

	ksmbd_debug(SMB, "Obtained string name = %s setupcount = %d\n",
			name, setup_bytes_count);

	pipe_name_offset = strlen("\\PIPE");
	if (strncmp("\\PIPE", name, pipe_name_offset) != 0) {
		ksmbd_debug(SMB, "Not Pipe request\n");
		rsp->hdr.Status.CifsError = STATUS_NOT_SUPPORTED;
		kfree(name);
		return 0;
	}

	if (name[pipe_name_offset] == '\\')
		pipe_name_offset++;

	pipe = name + pipe_name_offset;

	if (*pipe != '\0' && strncmp(pipe, "LANMAN", sizeof("LANMAN")) != 0) {
		ksmbd_debug(SMB, "Pipe %s not supported request\n", pipe);
		rsp->hdr.Status.CifsError = STATUS_NOT_SUPPORTED;
		kfree(name);
		return 0;
	}

	/* Incoming pipe name unicode len */
	str_len_uni = 2 * (strlen(name) + 1);

	ksmbd_debug(SMB, "Pipe name unicode len = %d\n", str_len_uni);

	/* Some clients like Windows may have additional padding. */
	padding = le16_to_cpu(req->ParameterOffset) -
		offsetof(struct smb_com_trans_req, Data)
		- str_len_uni;
	pipedata = req->Data + str_len_uni + setup_bytes_count + padding;

	if (!strncmp(pipe, "LANMAN", sizeof("LANMAN"))) {
		rpc_resp = ksmbd_rpc_rap(work->sess, pipedata,
					 le16_to_cpu(req->TotalParameterCount));

		if (rpc_resp) {
			if (rpc_resp->flags == KSMBD_RPC_ENOTIMPLEMENTED) {
				rsp->hdr.Status.CifsError =
					STATUS_NOT_SUPPORTED;
				kvfree(rpc_resp);
				goto out;
			} else if (rpc_resp->flags != KSMBD_RPC_OK) {
				rsp->hdr.Status.CifsError =
					STATUS_INVALID_PARAMETER;
				kvfree(rpc_resp);
				goto out;
			}

			nbytes = rpc_resp->payload_sz;
			memcpy((char *)rsp + sizeof(struct smb_com_trans_rsp),
				rpc_resp->payload, nbytes);

			kvfree(rpc_resp);
			ret = 0;
			goto resp_out;
		} else {
			ret = -EINVAL;
			goto out;
		}
	}

	id = pipe_req->fid;
	switch (subcommand) {
	case TRANSACT_DCERPCCMD:

		ksmbd_debug(SMB, "GOT TRANSACT_DCERPCCMD\n");
		ret = -EINVAL;
		rpc_resp = ksmbd_rpc_ioctl(work->sess, id, pipedata,
					   le16_to_cpu(req->DataCount));
		if (rpc_resp) {
			if (rpc_resp->flags == KSMBD_RPC_ENOTIMPLEMENTED) {
				rsp->hdr.Status.CifsError =
					STATUS_NOT_SUPPORTED;
				kvfree(rpc_resp);
				goto out;
			} else if (rpc_resp->flags != KSMBD_RPC_OK) {
				rsp->hdr.Status.CifsError =
					STATUS_INVALID_PARAMETER;
				kvfree(rpc_resp);
				goto out;
			}

			nbytes = rpc_resp->payload_sz;
			memcpy((char *)rsp + sizeof(struct smb_com_trans_rsp),
				rpc_resp->payload, nbytes);
			kvfree(rpc_resp);
			ret = 0;
		}
		break;

	default:
		ksmbd_debug(SMB, "SMB TRANS subcommand not supported %u\n",
				subcommand);
		ret = -EOPNOTSUPP;
		rsp->hdr.Status.CifsError = STATUS_NOT_SUPPORTED;
		goto out;
	}

resp_out:

	rsp->hdr.WordCount = 10;
	rsp->TotalParameterCount = cpu_to_le16(param_len);
	rsp->TotalDataCount = cpu_to_le16(nbytes);
	rsp->Reserved = 0;
	rsp->ParameterCount = cpu_to_le16(param_len);
	rsp->ParameterOffset = cpu_to_le16(56);
	rsp->ParameterDisplacement = 0;
	rsp->DataCount = cpu_to_le16(nbytes);
	rsp->DataOffset = cpu_to_le16(56 + param_len);
	rsp->DataDisplacement = 0;
	rsp->SetupCount = 0;
	rsp->Reserved1 = 0;
	/* Adding 1 for Pad */
	rsp->ByteCount = cpu_to_le16(nbytes + 1 + param_len);
	rsp->Pad = 0;
	inc_rfc1001_len(&rsp->hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));

out:
	kfree(name);
	return ret;
}

/**
 * create_andx_pipe() - create ipc pipe request handler
 * @work:	smb work containing create command
 *
 * Return:	0 on success, otherwise error
 */
static int create_andx_pipe(struct ksmbd_work *work)
{
	struct smb_com_open_req *req = work->request_buf;
	struct smb_com_open_ext_rsp *rsp = work->response_buf;
	char *name;
	int rc = 0;
	__u16 fid;

	/* one byte pad before unicode file name start */
	if (is_smbreq_unicode(&req->hdr))
		name = smb_strndup_from_utf16(req->fileName + 1, 256, 1,
				work->conn->local_nls);
	else
		name = smb_strndup_from_utf16(req->fileName, 256, 1,
				work->conn->local_nls);

	if (IS_ERR(name)) {
		rc = -ENOMEM;
		goto out;
	}

	rc = ksmbd_session_rpc_open(work->sess, name);
	if (rc < 0)
		goto out;
	fid = rc;

	rsp->hdr.WordCount = 42;
	rsp->AndXCommand = SMB_NO_MORE_ANDX_COMMAND;
	rsp->AndXReserved = 0;
	rsp->OplockLevel = 0;
	rsp->Fid = fid;
	rsp->CreateAction = cpu_to_le32(1);
	rsp->CreationTime = 0;
	rsp->LastAccessTime = 0;
	rsp->LastWriteTime = 0;
	rsp->ChangeTime = 0;
	rsp->FileAttributes = cpu_to_le32(ATTR_NORMAL);
	rsp->AllocationSize = cpu_to_le64(0);
	rsp->EndOfFile = 0;
	rsp->FileType = cpu_to_le16(2);
	rsp->DeviceState = cpu_to_le16(0x05ff);
	rsp->DirectoryFlag = 0;
	rsp->fid = 0;
	rsp->MaxAccess = cpu_to_le32(FILE_GENERIC_ALL);
	rsp->GuestAccess = cpu_to_le32(FILE_GENERIC_READ);
	rsp->ByteCount = 0;
	inc_rfc1001_len(&rsp->hdr, 100);

out:
	switch (rc) {
	case 0:
		rsp->hdr.Status.CifsError = STATUS_SUCCESS;
		break;
	case -EINVAL:
		rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
		break;
	case -ENOSPC:
	case -ENOMEM:
	default:
		rsp->hdr.Status.CifsError = STATUS_NO_MEMORY;
		break;
	}

	kfree(name);
	return rc;
}

/**
 * smb_nt_create_andx() - file open request handler
 * @work:	smb work containing nt open command
 *
 * Return:	0 on success, otherwise error
 */
int smb_nt_create_andx(struct ksmbd_work *work)
{
	struct smb_com_open_req *req = work->request_buf;
	struct smb_com_open_rsp *rsp = work->response_buf;
	struct smb_com_open_ext_rsp *ext_rsp = work->response_buf;
	struct ksmbd_conn *conn = work->conn;
	struct ksmbd_tree_connect *tcon = work->tcon;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct path path;
	struct kstat stat;
	int oplock_flags, file_info, open_flags, may_flags, access_flags;
	char *name;
	char *conv_name;
	bool file_present = true, extended_reply;
	__u64 alloc_size = 0, time;
	umode_t mode = 0;
	int err;
	int create_directory = 0;
	char *src;
	char *root = NULL;
	bool is_unicode;
	bool is_relative_root = false;
	struct ksmbd_file *fp = NULL;
	int oplock_rsp = OPLOCK_NONE;
	int share_ret;

	rsp->hdr.Status.CifsError = STATUS_UNSUCCESSFUL;
	if (test_share_config_flag(work->tcon->share_conf,
				   KSMBD_SHARE_FLAG_PIPE)) {
		ksmbd_debug(SMB, "create pipe on IPC\n");
		return create_andx_pipe(work);
	}

	if (req->CreateOptions & FILE_OPEN_BY_FILE_ID_LE) {
		ksmbd_debug(SMB, "file open with FID is not supported\n");
		rsp->hdr.Status.CifsError = STATUS_NOT_SUPPORTED;
		return -EINVAL;
	}

	if (req->CreateOptions & FILE_DELETE_ON_CLOSE_LE) {
		if (req->DesiredAccess &&
				!(le32_to_cpu(req->DesiredAccess) & DELETE)) {
			rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
			return -EPERM;
		}

		if (le32_to_cpu(req->FileAttributes) & ATTR_READONLY) {
			rsp->hdr.Status.CifsError = STATUS_CANNOT_DELETE;
			return -EPERM;
		}
	}

	if (req->CreateOptions & FILE_DIRECTORY_FILE_LE) {
		ksmbd_debug(SMB, "GOT Create Directory via CREATE ANDX\n");
		create_directory = 1;
	}

	/*
	 * Filename is relative to this root directory FID, instead of
	 * tree connect point. Find root dir name from this FID and
	 * prepend root dir name in filename.
	 */
	if (req->RootDirectoryFid) {
		ksmbd_debug(SMB, "path lookup relative to RootDirectoryFid\n");

		is_relative_root = true;
		fp = ksmbd_lookup_fd_fast(work, req->RootDirectoryFid);
		if (fp)
			root = (char *)fp->filp->f_path.dentry->d_name.name;
		else {
			rsp->hdr.Status.CifsError = STATUS_INVALID_HANDLE;
			memset(&rsp->hdr.WordCount, 0, 3);
			return -EINVAL;
		}
		ksmbd_fd_put(work, fp);
	}

	/* here allocated +2 (UNI '\0') length for both ASCII & UNI
	 * to avoid unnecessary if/else check
	 */
	src = kzalloc(le16_to_cpu(req->NameLength) + 2, GFP_KERNEL);
	if (!src) {
		rsp->hdr.Status.CifsError =
			STATUS_NO_MEMORY;

		return -ENOMEM;
	}

	if (is_smbreq_unicode(&req->hdr)) {
		memcpy(src, req->fileName + 1, le16_to_cpu(req->NameLength));
		is_unicode = true;
	} else {
		memcpy(src, req->fileName, le16_to_cpu(req->NameLength));
		is_unicode = false;
	}

	name = smb_strndup_from_utf16(src, PATH_MAX, is_unicode,
			conn->local_nls);
	kfree(src);

	if (IS_ERR(name)) {
		if (PTR_ERR(name) == -ENOMEM) {
			pr_err("failed to allocate memory\n");
			rsp->hdr.Status.CifsError =
				STATUS_NO_MEMORY;
		} else
			rsp->hdr.Status.CifsError =
				STATUS_OBJECT_NAME_INVALID;

		return PTR_ERR(name);
	}

	if (is_relative_root) {
		char *full_name;

		full_name = kasprintf(GFP_KERNEL, "\\%s\\%s", root, name);
		if (!full_name) {
			kfree(name);
			rsp->hdr.Status.CifsError = STATUS_NO_MEMORY;
			return -ENOMEM;
		}

		kfree(name);
		name = full_name;
	}

	root = strrchr(name, '\\');
	if (root) {
		root++;
		if ((root[0] == '*' || root[0] == '/') && (root[1] == '\0')) {
			rsp->hdr.Status.CifsError =
				STATUS_OBJECT_NAME_INVALID;
			kfree(name);
			return -EINVAL;
		}
	}

	conv_name = smb_get_name(share, name, PATH_MAX, work, true);
	if (IS_ERR(conv_name)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		kfree(name);
		return PTR_ERR(conv_name);
	}

	if (ksmbd_override_fsids(work)) {
		err = -ENOMEM;
		goto out1;
	}

	err = ksmbd_vfs_kern_path(work, conv_name, LOOKUP_NO_SYMLINKS, &path,
			(req->hdr.Flags & SMBFLG_CASELESS) &&
			!create_directory);
	if (err) {
		if (err == -EACCES || err == -EXDEV)
			goto out;
		file_present = false;
		ksmbd_debug(SMB, "can not get linux path for %s, err = %d\n",
				conv_name, err);
	} else {
		if (d_is_symlink(path.dentry)) {
			err = -EACCES;
			goto free_path;
		}

		err = vfs_getattr(&path, &stat, STATX_BASIC_STATS,
				  AT_STATX_SYNC_AS_STAT);
		if (err) {
			pr_err("can not stat %s, err = %d\n",
			       conv_name, err);
			goto free_path;
		}
	}

	if (file_present && (req->CreateOptions & FILE_NON_DIRECTORY_FILE_LE) &&
			S_ISDIR(stat.mode)) {
		ksmbd_debug(SMB, "Can't open dir %s, request is to open file\n",
			       conv_name);
		if (!(((struct smb_hdr *)work->request_buf)->Flags2 &
					SMBFLG2_ERR_STATUS)) {
			rsp->hdr.Status.DosError.ErrorClass = ERRDOS;
			rsp->hdr.Status.DosError.Error =
				cpu_to_le16(ERRfilexists);
		} else
			rsp->hdr.Status.CifsError =
				STATUS_OBJECT_NAME_COLLISION;

		memset(&rsp->hdr.WordCount, 0, 3);

		goto free_path;
	}

	if (file_present && create_directory && !S_ISDIR(stat.mode)) {
		ksmbd_debug(SMB, "Can't open file %s, request is to open dir\n",
				conv_name);
		if (!(((struct smb_hdr *)work->request_buf)->Flags2 &
					SMBFLG2_ERR_STATUS)) {
			ntstatus_to_dos(STATUS_NOT_A_DIRECTORY,
					&rsp->hdr.Status.DosError.ErrorClass,
					&rsp->hdr.Status.DosError.Error);
		} else
			rsp->hdr.Status.CifsError =
				STATUS_NOT_A_DIRECTORY;

		memset(&rsp->hdr.WordCount, 0, 3);

		goto free_path;
	}

	oplock_flags = le32_to_cpu(req->OpenFlags) &
		(REQ_OPLOCK | REQ_BATCHOPLOCK);
	extended_reply = le32_to_cpu(req->OpenFlags) & REQ_EXTENDED_INFO;
	open_flags = file_create_dispostion_flags(
			le32_to_cpu(req->CreateDisposition), file_present);

	if (open_flags < 0) {
		ksmbd_debug(SMB, "create_dispostion returned %d\n", open_flags);
		if (file_present) {
			if (!(((struct smb_hdr *)work->request_buf)->Flags2 &
						SMBFLG2_ERR_STATUS)) {
				rsp->hdr.Status.DosError.ErrorClass = ERRDOS;
				rsp->hdr.Status.DosError.Error =
					cpu_to_le16(ERRfilexists);
			} else if (open_flags == -EINVAL)
				rsp->hdr.Status.CifsError =
					STATUS_INVALID_PARAMETER;
			else
				rsp->hdr.Status.CifsError =
					STATUS_OBJECT_NAME_COLLISION;
			memset(&rsp->hdr.WordCount, 0, 3);
			goto free_path;
		} else {
			err = -ENOENT;
			goto out;
		}
	} else {
		if (file_present) {
			if (S_ISFIFO(stat.mode))
				open_flags |= O_NONBLOCK;
		}

		if (req->CreateOptions & FILE_WRITE_THROUGH_LE)
			open_flags |= O_SYNC;
	}

	access_flags = convert_generic_access_flags(
			le32_to_cpu(req->DesiredAccess),
			&open_flags, &may_flags,
			le32_to_cpu(req->FileAttributes));

	mode |= 0777;
	if (le32_to_cpu(req->FileAttributes) & ATTR_READONLY)
		mode &= ~0222;

	/* TODO:
	 * - check req->ShareAccess for sharing file among different process
	 * - check req->FileAttributes for special/readonly file attrib
	 * - check req->SecurityFlags for client security context tracking
	 * - check req->ImpersonationLevel
	 */

	if (!test_tree_conn_flag(work->tcon, KSMBD_TREE_CONN_FLAG_WRITABLE)) {
		if (open_flags & O_CREAT) {
			ksmbd_debug(SMB,
				"returning as user does not have permission to write\n");
			err = -EACCES;
			goto out;
		}
	}

	ksmbd_debug(SMB, "filename : %s, open_flags = 0x%x\n", conv_name,
		open_flags);
	if (!file_present && (open_flags & O_CREAT)) {

		if (!create_directory) {
			mode |= S_IFREG;
			err = ksmbd_vfs_create(work, conv_name, mode);
			if (err)
				goto out;
		} else {
			err = ksmbd_vfs_mkdir(work, conv_name, mode);
			if (err) {
				pr_err("Can't create directory %s",
				       conv_name);
				goto out;
			}
		}

		err = ksmbd_vfs_kern_path(work, conv_name, 0, &path, 0);
		if (err) {
			pr_err("cannot get linux path, err = %d\n", err);
			goto out;
		}
	} else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
		err = inode_permission(mnt_user_ns(path.mnt),
				       d_inode(path.dentry),
				       may_flags);
#else
		err = inode_permission(d_inode(path.dentry),
				       may_flags);
#endif
		if (err)
			goto free_path;
	}

	err = ksmbd_query_inode_status(d_inode(path.dentry->d_parent));
	if (err == KSMBD_INODE_STATUS_PENDING_DELETE) {
		err = -EBUSY;
		goto free_path;
	}

	err = 0;
	/* open  file and get FID */
	fp = ksmbd_vfs_dentry_open(work,
				   &path,
				   open_flags,
				   req->CreateOptions,
				   file_present);
	if (IS_ERR(fp)) {
		err = PTR_ERR(fp);
		fp = NULL;
		goto free_path;
	}
	fp->daccess = req->DesiredAccess;
	fp->saccess = req->ShareAccess;
	fp->pid = le16_to_cpu(req->hdr.Pid);

	write_lock(&fp->f_ci->m_lock);
	list_add(&fp->node, &fp->f_ci->m_fp_list);
	write_unlock(&fp->f_ci->m_lock);

	share_ret = ksmbd_smb_check_shared_mode(fp->filp, fp);
	if (smb1_oplock_enable &&
	    test_share_config_flag(work->tcon->share_conf,
			KSMBD_SHARE_FLAG_OPLOCKS) &&
		!S_ISDIR(file_inode(fp->filp)->i_mode) && oplock_flags) {
		/* Client cannot request levelII oplock directly */
		err = smb_grant_oplock(work, oplock_flags, fp->volatile_id,
			fp, le16_to_cpu(req->hdr.Tid), NULL, share_ret);
		if (err)
			goto free_path;
	} else {
		if (ksmbd_inode_pending_delete(fp)) {
			err = -EBUSY;
			goto free_path;
		}

		if (share_ret < 0) {
			err = -EPERM;
			goto free_path;
		}
	}

	oplock_rsp = fp->f_opinfo != NULL ? fp->f_opinfo->level : 0;

	if (file_present) {
		if (!(open_flags & O_TRUNC))
			file_info = F_OPENED;
		else
			file_info = F_OVERWRITTEN;
	} else
		file_info = F_CREATED;

	if (le32_to_cpu(req->DesiredAccess) & (DELETE | GENERIC_ALL))
		fp->is_nt_open = 1;
	if ((le32_to_cpu(req->DesiredAccess) & DELETE) &&
			(req->CreateOptions & FILE_DELETE_ON_CLOSE_LE))
		ksmbd_fd_set_delete_on_close(fp, file_info);

	/* open success, send back response */
	err = vfs_getattr(&path, &stat, STATX_BASIC_STATS,
		AT_STATX_SYNC_AS_STAT);
	if (err) {
		pr_err("cannot get stat information\n");
		goto free_path;
	}

	alloc_size = le64_to_cpu(req->AllocationSize);
	if (alloc_size && (file_info == F_CREATED ||
				file_info == F_OVERWRITTEN)) {
		if (alloc_size > stat.size) {
			err = ksmbd_vfs_truncate(work, fp, alloc_size);
			if (err) {
				pr_err("failed to expand file, err = %d\n",
				       err);
				goto free_path;
			}
		}
	}

	/* prepare response buffer */
	rsp->hdr.Status.CifsError = STATUS_SUCCESS;

	rsp->OplockLevel = oplock_rsp;
	rsp->Fid = fp->volatile_id;

	if ((le32_to_cpu(req->CreateDisposition) == FILE_SUPERSEDE) &&
			(file_info == F_OVERWRITTEN))
		rsp->CreateAction = cpu_to_le32(F_SUPERSEDED);
	else
		rsp->CreateAction = cpu_to_le32(file_info);

	if (stat.result_mask & STATX_BTIME)
		fp->create_time = ksmbd_UnixTimeToNT(stat.btime);
	else
		fp->create_time = ksmbd_UnixTimeToNT(stat.ctime);
	if (file_present) {
		if (test_share_config_flag(tcon->share_conf,
					   KSMBD_SHARE_FLAG_STORE_DOS_ATTRS)) {
			struct xattr_dos_attrib da;

			err = ksmbd_vfs_get_dos_attrib_xattr(mnt_user_ns(path.mnt),
							     path.dentry, &da);
			if (err > 0)
				fp->create_time = da.create_time;
			err = 0;
		}
	} else {
		if (test_share_config_flag(tcon->share_conf,
					   KSMBD_SHARE_FLAG_STORE_DOS_ATTRS)) {
			struct xattr_dos_attrib da = {0};

			da.version = 4;
			da.attr = smb_get_dos_attr(&stat);
			da.create_time = fp->create_time;

			err = ksmbd_vfs_set_dos_attrib_xattr(mnt_user_ns(path.mnt),
							     path.dentry, &da);
			if (err)
				ksmbd_debug(SMB, "failed to store creation time in xattr\n");
			err = 0;
		}
	}

	rsp->CreationTime = cpu_to_le64(fp->create_time);
	time = ksmbd_UnixTimeToNT(stat.atime);
	rsp->LastAccessTime = cpu_to_le64(time);
	time = ksmbd_UnixTimeToNT(stat.mtime);
	rsp->LastWriteTime = cpu_to_le64(time);
	time = ksmbd_UnixTimeToNT(stat.ctime);
	rsp->ChangeTime = cpu_to_le64(time);

	rsp->FileAttributes = cpu_to_le32(smb_get_dos_attr(&stat));
	rsp->AllocationSize = cpu_to_le64(stat.blocks << 9);
	rsp->EndOfFile = cpu_to_le64(stat.size);
	/* TODO: is it normal file, named pipe, printer, modem etc*/
	rsp->FileType = 0;
	/* status of named pipe*/
	rsp->DeviceState = 0;
	rsp->DirectoryFlag = S_ISDIR(stat.mode) ? 1 : 0;
	if (extended_reply) {
		struct inode *inode;

		rsp->hdr.WordCount = 50;
		memset(&ext_rsp->VolId, 0, 16);
		if (fp) {
			inode = file_inode(fp->filp);
			ext_rsp->fid = inode->i_ino;
			if (S_ISDIR(inode->i_mode) ||
			    (fp->filp->f_mode & FMODE_WRITE))
				ext_rsp->MaxAccess = FILE_GENERIC_ALL_LE;
			else
				ext_rsp->MaxAccess = FILE_GENERIC_READ_LE |
						     FILE_EXECUTE_LE;
		} else {
			ext_rsp->MaxAccess = FILE_GENERIC_ALL_LE;
			ext_rsp->fid = 0;
		}

		ext_rsp->ByteCount = 0;

	} else {
		rsp->hdr.WordCount = 34;
		rsp->ByteCount = 0;
	}
	inc_rfc1001_len(&rsp->hdr, (rsp->hdr.WordCount * 2 + 0));

free_path:
	path_put(&path);
out:
	ksmbd_revert_fsids(work);
out1:
	switch (err) {
	case 0:
		break;
	case -ENOSPC:
		rsp->hdr.Status.CifsError = STATUS_DISK_FULL;
		break;
	case -EMFILE:
		rsp->hdr.Status.CifsError =
			STATUS_TOO_MANY_OPENED_FILES;
		break;
	case -EINVAL:
		rsp->hdr.Status.CifsError = STATUS_NO_SUCH_USER;
		break;
	case -EACCES:
		rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
		break;
	case -EPERM:
		rsp->hdr.Status.CifsError = STATUS_SHARING_VIOLATION;
		break;
	case -ENOENT:
		rsp->hdr.Status.CifsError = STATUS_OBJECT_NAME_NOT_FOUND;
		break;
	case -EBUSY:
		rsp->hdr.Status.CifsError = STATUS_DELETE_PENDING;
		break;
	default:
		rsp->hdr.Status.CifsError =
			STATUS_UNEXPECTED_IO_ERROR;
	}

	if (err && fp)
		ksmbd_close_fd(work, fp->volatile_id);

	kfree(conv_name);

	if (!rsp->hdr.WordCount)
		return err;

	/* this is an ANDx command ? */
	rsp->AndXReserved = 0;
	rsp->AndXOffset = cpu_to_le16(get_rfc1002_len(&rsp->hdr));
	if (req->AndXCommand != SMB_NO_MORE_ANDX_COMMAND) {
		/* adjust response */
		rsp->AndXCommand = req->AndXCommand;
		return rsp->AndXCommand; /* More processing required */
	}
	rsp->AndXCommand = SMB_NO_MORE_ANDX_COMMAND;

	return err;
}

/**
 * smb_close_pipe() - ipc pipe close request handler
 * @work:	smb work containing close command
 *
 * Return:	0 on success, otherwise error
 */
static int smb_close_pipe(struct ksmbd_work *work)
{
	struct smb_com_close_req *req = work->request_buf;

	ksmbd_session_rpc_close(work->sess, req->FileID);
	return 0;
}

/**
 * smb_close() - ipc pipe close request handler
 * @work:	smb work containing close command
 *
 * Return:	0 on success, otherwise error
 */
int smb_close(struct ksmbd_work *work)
{
	struct smb_com_close_req *req = work->request_buf;
	struct smb_com_close_rsp *rsp = work->response_buf;
	int err = 0;

	ksmbd_debug(SMB, "SMB_COM_CLOSE called for fid %u\n", req->FileID);

	if (test_share_config_flag(work->tcon->share_conf,
				   KSMBD_SHARE_FLAG_PIPE)) {
		err = smb_close_pipe(work);
		if (err < 0)
			goto out;
		goto IPC_out;
	}

	/*
	 * TODO: linux cifs client does not send LastWriteTime,
	 * need to check if windows client use this field
	 */
	if (le32_to_cpu(req->LastWriteTime) > 0 &&
	    le32_to_cpu(req->LastWriteTime) < 0xFFFFFFFF)
		pr_info("need to set last modified time before close\n");

	err = ksmbd_close_fd(work, req->FileID);

IPC_out:
	/* file close success, return response to server */
	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 0;
	rsp->ByteCount = 0;

out:
	if (err)
		rsp->hdr.Status.CifsError = STATUS_INVALID_HANDLE;
	return err;
}

/**
 * smb_read_andx_pipe() - read from ipc pipe request handler
 * @work:	smb work containing read command
 *
 * Return:	0 on success, otherwise error
 */
static int smb_read_andx_pipe(struct ksmbd_work *work)
{
	struct smb_com_read_req *req = work->request_buf;
	struct smb_com_read_rsp *rsp = work->response_buf;
	struct ksmbd_rpc_command *rpc_resp;
	char *data_buf;
	int ret = 0, nbytes = 0;
	unsigned int count;
	unsigned int rsp_buflen = MAX_CIFS_SMALL_BUFFER_SIZE -
		sizeof(struct smb_com_read_rsp);

	rsp_buflen = min((unsigned int)(MAX_CIFS_SMALL_BUFFER_SIZE -
				sizeof(struct smb_com_read_rsp)), rsp_buflen);

	count = min_t(unsigned int, le16_to_cpu(req->MaxCount), rsp_buflen);
	data_buf = (char *) (&rsp->ByteCount) + sizeof(rsp->ByteCount);

	rpc_resp = ksmbd_rpc_read(work->sess, req->Fid);
	if (rpc_resp) {
		if (rpc_resp->flags != KSMBD_RPC_OK ||
				!rpc_resp->payload_sz) {
			rsp->hdr.Status.CifsError =
				STATUS_UNEXPECTED_IO_ERROR;
			kvfree(rpc_resp);
			return -EINVAL;
		}

		nbytes = rpc_resp->payload_sz;
		memcpy(data_buf, rpc_resp->payload, rpc_resp->payload_sz);
		kvfree(rpc_resp);
	} else {
		ret = -EINVAL;
	}

	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 12;
	rsp->Remaining = 0;
	rsp->DataCompactionMode = 0;
	rsp->DataCompactionMode = 0;
	rsp->Reserved = 0;
	rsp->DataLength = cpu_to_le16(nbytes & 0xFFFF);
	rsp->DataOffset = cpu_to_le16(sizeof(struct smb_com_read_rsp) -
			sizeof(rsp->hdr.smb_buf_length));
	rsp->DataLengthHigh = cpu_to_le16(nbytes >> 16);
	rsp->Reserved2 = 0;

	rsp->ByteCount = cpu_to_le16(nbytes);
	inc_rfc1001_len(&rsp->hdr, (rsp->hdr.WordCount * 2 + nbytes));

	/* this is an ANDx command ? */
	rsp->AndXReserved = 0;
	rsp->AndXOffset = cpu_to_le16(get_rfc1002_len(&rsp->hdr));
	if (req->AndXCommand != SMB_NO_MORE_ANDX_COMMAND) {
		/* adjust response */
		rsp->AndXCommand = req->AndXCommand;
		return rsp->AndXCommand; /* More processing required */
	}
	rsp->AndXCommand = SMB_NO_MORE_ANDX_COMMAND;

	return ret;
}

/**
 * smb_read_andx() - read request handler
 * @work:	smb work containing read command
 *
 * Return:	0 on success, otherwise error
 */
int smb_read_andx(struct ksmbd_work *work)
{
	struct ksmbd_conn *conn = work->conn;
	struct smb_com_read_req *req = work->request_buf;
	struct smb_com_read_rsp *rsp = work->response_buf;
	struct ksmbd_file *fp;
	loff_t pos;
	size_t count;
	ssize_t nbytes;
	int err = 0;

	if (test_share_config_flag(work->tcon->share_conf,
				   KSMBD_SHARE_FLAG_PIPE))
		return smb_read_andx_pipe(work);

	fp = ksmbd_lookup_fd_fast(work, req->Fid);
	if (!fp) {
		pr_err("failed to get filp for fid %d\n",
		       req->Fid);
		rsp->hdr.Status.CifsError = STATUS_FILE_CLOSED;
		return -ENOENT;
	}

	pos = le32_to_cpu(req->OffsetLow);
	if (req->hdr.WordCount == 12)
		pos |= ((loff_t)le32_to_cpu(req->OffsetHigh) << 32);

	count = le16_to_cpu(req->MaxCount);
	/*
	 * It probably seems to be set to 0 or 0xFFFF if MaxCountHigh is
	 * not supported. If it is 0xFFFF, it is set to a too large value
	 * and a read fail occurs. If it is 0xFFFF, limit it to not set
	 * the value.
	 *
	 * [MS-SMB] 3.2.4.4.1:
	 * If the CAP_LARGE_READX bit is set in
	 * Client.Connection.ServerCapabilities, then the client is allowed to
	 * issue a read of a size larger than Client.Connection.MaxBufferSize
	 * using an SMB_COM_READ_ANDX request.
	 */
	if (conn->vals->capabilities & CAP_LARGE_READ_X &&
		le32_to_cpu(req->MaxCountHigh) < 0xFFFF)
		count |= le32_to_cpu(req->MaxCountHigh) << 16;
	else if (count > CIFS_DEFAULT_IOSIZE) {
		ksmbd_debug(SMB, "read size(%zu) exceeds max size(%u)\n",
				count, CIFS_DEFAULT_IOSIZE);
		ksmbd_debug(SMB, "limiting read size to max size(%u)\n",
				CIFS_DEFAULT_IOSIZE);
		count = CIFS_DEFAULT_IOSIZE;
	}

	ksmbd_debug(SMB, "filename %pd, offset %lld, count %zu\n",
		    fp->filp->f_path.dentry, pos, count);

	work->aux_payload_buf = kvmalloc(count, GFP_KERNEL | __GFP_ZERO);
	if (!work->aux_payload_buf) {
		err = -ENOMEM;
		goto out;
	}

	nbytes = ksmbd_vfs_read(work, fp, count, &pos);
	if (nbytes < 0) {
		err = nbytes;
		goto out;
	}

	/* read success, prepare response */
	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 12;
	rsp->Remaining = 0;
	rsp->DataCompactionMode = 0;
	rsp->DataCompactionMode = 0;
	rsp->Reserved = 0;
	rsp->DataLength = cpu_to_le16(nbytes & 0xFFFF);
	rsp->DataOffset = cpu_to_le16(sizeof(struct smb_com_read_rsp) -
			sizeof(rsp->hdr.smb_buf_length));
	rsp->DataLengthHigh = cpu_to_le16(nbytes >> 16);
	rsp->Reserved2 = 0;

	rsp->ByteCount = cpu_to_le16(nbytes);
	inc_rfc1001_len(&rsp->hdr, (rsp->hdr.WordCount * 2));
	work->resp_hdr_sz = get_rfc1002_len(rsp) + 4;
	work->aux_payload_sz = nbytes;
	inc_rfc1001_len(&rsp->hdr, nbytes);

	/* this is an ANDx command ? */
	rsp->AndXReserved = 0;
	rsp->AndXOffset = cpu_to_le16(get_rfc1002_len(&rsp->hdr));
	if (req->AndXCommand != SMB_NO_MORE_ANDX_COMMAND) {
		/* adjust response */
		rsp->AndXCommand = req->AndXCommand;
		ksmbd_fd_put(work, fp);
		return rsp->AndXCommand; /* More processing required */
	}
	rsp->AndXCommand = SMB_NO_MORE_ANDX_COMMAND;

out:
	ksmbd_fd_put(work, fp);
	if (err)
		rsp->hdr.Status.CifsError = STATUS_INVALID_HANDLE;
	return err;
}

/**
 * smb_write() - write request handler
 * @work:	smb work containing write command
 *
 * Return:	0 on success, otherwise error
 */
int smb_write(struct ksmbd_work *work)
{
	struct smb_com_write_req_32bit *req = work->request_buf;
	struct smb_com_write_rsp_32bit *rsp = work->response_buf;
	struct ksmbd_file *fp = NULL;
	loff_t pos;
	size_t count;
	char *data_buf;
	ssize_t nbytes = 0;
	int err = 0;

	if (!test_tree_conn_flag(work->tcon, KSMBD_TREE_CONN_FLAG_WRITABLE)) {
		ksmbd_debug(SMB,
			"returning as user does not have permission to write\n");
		rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
		return -EACCES;
	}

	fp = ksmbd_lookup_fd_fast(work, req->Fid);
	if (!fp) {
		pr_err("failed to get filp for fid %u\n", req->Fid);
		rsp->hdr.Status.CifsError = STATUS_FILE_CLOSED;
		return -ENOENT;
	}

	pos = le32_to_cpu(req->Offset);
	count = le16_to_cpu(req->Length);
	data_buf = req->Data;

	ksmbd_debug(SMB, "filename %pd, offset %lld, count %zu\n",
		    fp->filp->f_path.dentry, pos, count);
	if (!count) {
		err = ksmbd_vfs_truncate(work, fp, pos);
		nbytes = 0;
	} else
		err = ksmbd_vfs_write(work, fp, data_buf,
				      count, &pos, 0, &nbytes);

	rsp->hdr.WordCount = 1;
	rsp->Written = cpu_to_le16(nbytes & 0xFFFF);
	rsp->ByteCount = 0;
	inc_rfc1001_len(&rsp->hdr, (rsp->hdr.WordCount * 2));

	ksmbd_fd_put(work, fp);
	if (!err) {
		rsp->hdr.Status.CifsError = STATUS_SUCCESS;
		return 0;
	}

	if (err == -ENOSPC || err == -EFBIG)
		rsp->hdr.Status.CifsError = STATUS_DISK_FULL;
	else
		rsp->hdr.Status.CifsError = STATUS_INVALID_HANDLE;
	return err;
}

/**
 * smb_write_andx_pipe() - write on pipe request handler
 * @work:	smb work containing write command
 *
 * Return:	0 on success, otherwise error
 */
static int smb_write_andx_pipe(struct ksmbd_work *work)
{
	struct smb_com_write_req *req = work->request_buf;
	struct smb_com_write_rsp *rsp = work->response_buf;
	struct ksmbd_rpc_command *rpc_resp;
	int ret = 0;
	size_t count = 0;

	count = le16_to_cpu(req->DataLengthLow);
	if (work->conn->vals->capabilities & CAP_LARGE_WRITE_X)
		count |= (le16_to_cpu(req->DataLengthHigh) << 16);

	rpc_resp = ksmbd_rpc_write(work->sess, req->Fid, req->Data, count);
	if (rpc_resp) {
		if (rpc_resp->flags == KSMBD_RPC_ENOTIMPLEMENTED) {
			rsp->hdr.Status.CifsError = STATUS_NOT_SUPPORTED;
			kvfree(rpc_resp);
			return -EOPNOTSUPP;
		}
		if (rpc_resp->flags != KSMBD_RPC_OK) {
			rsp->hdr.Status.CifsError = STATUS_INVALID_HANDLE;
			kvfree(rpc_resp);
			return -EINVAL;
		}
		count = rpc_resp->payload_sz;
		kvfree(rpc_resp);
	} else {
		ret = -EINVAL;
	}

	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 6;
	rsp->Count = cpu_to_le16(count & 0xFFFF);
	rsp->Remaining = 0;
	rsp->CountHigh = cpu_to_le16(count >> 16);
	rsp->Reserved = 0;
	rsp->ByteCount = 0;
	inc_rfc1001_len(&rsp->hdr, (rsp->hdr.WordCount * 2));

	/* this is an ANDx command ? */
	rsp->AndXReserved = 0;
	rsp->AndXOffset = cpu_to_le16(get_rfc1002_len(&rsp->hdr));
	if (req->AndXCommand != SMB_NO_MORE_ANDX_COMMAND) {
		/* adjust response */
		rsp->AndXCommand = req->AndXCommand;
		return rsp->AndXCommand; /* More processing required */
	}
	rsp->AndXCommand = SMB_NO_MORE_ANDX_COMMAND;

	return ret;
}

/**
 * smb_write_andx() - andx write request handler
 * @work:	smb work containing write command
 *
 * Return:	0 on success, otherwise error
 */
int smb_write_andx(struct ksmbd_work *work)
{
	struct ksmbd_conn *conn = work->conn;
	struct smb_com_write_req *req = work->request_buf;
	struct smb_com_write_rsp *rsp = work->response_buf;
	struct ksmbd_file *fp;
	bool writethrough = false;
	loff_t pos;
	size_t count;
	ssize_t nbytes = 0;
	char *data_buf;
	int err = 0;

	if (!test_tree_conn_flag(work->tcon, KSMBD_TREE_CONN_FLAG_WRITABLE)) {
		ksmbd_debug(SMB,
			"returning as user does not have permission to write\n");
		rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
		return -EACCES;
	}

	if (test_share_config_flag(work->tcon->share_conf,
				   KSMBD_SHARE_FLAG_PIPE)) {
		ksmbd_debug(SMB, "Write ANDX called for IPC$");
		return smb_write_andx_pipe(work);
	}

	fp = ksmbd_lookup_fd_fast(work, req->Fid);
	if (!fp) {
		pr_err("failed to get filp for fid %u\n", req->Fid);
		rsp->hdr.Status.CifsError = STATUS_FILE_CLOSED;
		return -ENOENT;
	}

	pos = le32_to_cpu(req->OffsetLow);
	if (req->hdr.WordCount == 14)
		pos |= ((loff_t)le32_to_cpu(req->OffsetHigh) << 32);

	writethrough = (le16_to_cpu(req->WriteMode) == 1);

	/*
	 * [MS-SMB] 3.3.5.8:
	 * If CAP_LARGE_WRITEX is set in Server.Connection.ClientCapabilities,
	 * then it is possible that the count of bytes to be written is larger
	 * than the server's MaxBufferSize
	 */
	count = le16_to_cpu(req->DataLengthLow);
	if (conn->vals->capabilities & CAP_LARGE_WRITE_X)
		count |= (le16_to_cpu(req->DataLengthHigh) << 16);
	else if (count > CIFS_DEFAULT_IOSIZE) {
		ksmbd_debug(SMB, "write size(%zu) exceeds max size(%u)\n",
				count, CIFS_DEFAULT_IOSIZE);
		ksmbd_debug(SMB, "limiting write size to max size(%u)\n",
				CIFS_DEFAULT_IOSIZE);
		count = CIFS_DEFAULT_IOSIZE;
	}

	if (le16_to_cpu(req->DataOffset) ==
			(offsetof(struct smb_com_write_req, Data) - 4)) {
		data_buf = (char *)&req->Data[0];
	} else {
		if ((le16_to_cpu(req->DataOffset) > get_rfc1002_len(req)) ||
				(le16_to_cpu(req->DataOffset) +
				 count > get_rfc1002_len(req))) {
			pr_err("invalid write data offset %u, smb_len %u\n",
			       le16_to_cpu(req->DataOffset),
			       get_rfc1002_len(req));
			err = -EINVAL;
			goto out;
		}

		data_buf = (char *)(((char *)&req->hdr.Protocol) +
				le16_to_cpu(req->DataOffset));
	}

	ksmbd_debug(SMB, "filname %pd, offset %lld, count %zu\n",
		    fp->filp->f_path.dentry, pos, count);
	err = ksmbd_vfs_write(work, fp, data_buf, count, &pos,
			      writethrough, &nbytes);
	if (err < 0)
		goto out;

	/* write success, prepare response */
	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 6;
	rsp->Count = cpu_to_le16(nbytes & 0xFFFF);
	rsp->Remaining = 0;
	rsp->CountHigh = cpu_to_le16(nbytes >> 16);
	rsp->Reserved = 0;
	rsp->ByteCount = 0;
	inc_rfc1001_len(&rsp->hdr, (rsp->hdr.WordCount * 2));

	ksmbd_fd_put(work, fp);
	/* this is an ANDx command ? */
	rsp->AndXReserved = 0;
	rsp->AndXOffset = cpu_to_le16(get_rfc1002_len(&rsp->hdr));
	if (req->AndXCommand != SMB_NO_MORE_ANDX_COMMAND) {
		/* adjust response */
		rsp->AndXCommand = req->AndXCommand;
		return rsp->AndXCommand; /* More processing required */
	}
	rsp->AndXCommand = SMB_NO_MORE_ANDX_COMMAND;

	return 0;

out:
	ksmbd_fd_put(work, fp);
	if (err == -ENOSPC || err == -EFBIG)
		rsp->hdr.Status.CifsError = STATUS_DISK_FULL;
	else
		rsp->hdr.Status.CifsError = STATUS_INVALID_HANDLE;
	return err;
}

/**
 * smb_echo() - echo(ping) request handler
 * @work:	smb work containing echo command
 *
 * Return:	0 on success, otherwise error
 */
int smb_echo(struct ksmbd_work *work)
{
	struct smb_com_echo_req *req = work->request_buf;
	struct smb_com_echo_rsp *rsp = work->response_buf;
	__u16 data_count;
	int i;

	ksmbd_debug(SMB, "SMB_COM_ECHO called with echo count %u\n",
			le16_to_cpu(req->EchoCount));

	if (le16_to_cpu(req->EchoCount) > 1)
		work->multiRsp = 1;

	data_count = le16_to_cpu(req->ByteCount);
	/* send echo response to server */
	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 1;
	rsp->ByteCount = cpu_to_le16(data_count);

	memcpy(rsp->Data, req->Data, data_count);
	inc_rfc1001_len(&rsp->hdr, (rsp->hdr.WordCount * 2) + data_count);

	/* Send req->EchoCount - 1 number of ECHO response now &
	 * if SMB CANCEL for Echo comes don't send response
	 */
	for (i = 1; i < le16_to_cpu(req->EchoCount) &&
	     !work->send_no_response; i++) {
		rsp->SequenceNumber = cpu_to_le16(i);
		ksmbd_conn_write(work);
	}

	/* Last echo response */
	rsp->SequenceNumber = cpu_to_le16(i);
	work->multiRsp = 0;

	return 0;
}

/**
 * smb_flush() - file sync - flush request handler
 * @work:	smb work containing flush command
 *
 * Return:	0 on success, otherwise error
 */
int smb_flush(struct ksmbd_work *work)
{
	struct smb_com_flush_req *req = work->request_buf;
	struct smb_com_flush_rsp *rsp = work->response_buf;
	int err = 0;

	ksmbd_debug(SMB, "SMB_COM_FLUSH called for fid %u\n", req->FileID);

	if (req->FileID == 0xFFFF) {
		err = ksmbd_file_table_flush(work);
		if (err)
			goto out;
	} else {
		err = ksmbd_vfs_fsync(work, req->FileID, KSMBD_NO_FID);
		if (err)
			goto out;
	}

	/* file fsync success, return response to server */
	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 0;
	rsp->ByteCount = 0;
	return err;

out:
	if (err)
		rsp->hdr.Status.CifsError = STATUS_INVALID_HANDLE;

	return err;
}

/*****************************************************************************
 * TRANS2 command implementation functions
 *****************************************************************************/

/**
 * get_filetype() - convert file mode to smb file type
 * @mode:	file mode to be convertd
 *
 * Return:	converted file type
 */
static __u32 get_filetype(mode_t mode)
{
	if (S_ISREG(mode))
		return UNIX_FILE;
	else if (S_ISDIR(mode))
		return UNIX_DIR;
	else if (S_ISLNK(mode))
		return UNIX_SYMLINK;
	else if (S_ISCHR(mode))
		return UNIX_CHARDEV;
	else if (S_ISBLK(mode))
		return UNIX_BLOCKDEV;
	else if (S_ISFIFO(mode))
		return UNIX_FIFO;
	else if (S_ISSOCK(mode))
		return UNIX_SOCKET;

	return UNIX_UNKNOWN;
}

/**
 * init_unix_info() - convert file stat information to smb file info format
 * @unix_info:	smb file information format
 * @stat:	unix file/dir stat information
 */
static void init_unix_info(struct file_unix_basic_info *unix_info,
			   struct user_namespace *user_ns, struct kstat *stat)
{
	u64 time;

	unix_info->EndOfFile = cpu_to_le64(stat->size);
	unix_info->NumOfBytes = cpu_to_le64(512 * stat->blocks);
	time = ksmbd_UnixTimeToNT(stat->ctime);
	unix_info->LastStatusChange = cpu_to_le64(time);
	time = ksmbd_UnixTimeToNT(stat->atime);
	unix_info->LastAccessTime = cpu_to_le64(time);
	time = ksmbd_UnixTimeToNT(stat->mtime);
	unix_info->LastModificationTime = cpu_to_le64(time);
	unix_info->Uid = cpu_to_le64(from_kuid(user_ns, stat->uid));
	unix_info->Gid = cpu_to_le64(from_kgid(user_ns, stat->gid));
	unix_info->Type = cpu_to_le32(get_filetype(stat->mode));
	unix_info->DevMajor = cpu_to_le64(MAJOR(stat->rdev));
	unix_info->DevMinor = cpu_to_le64(MINOR(stat->rdev));
	unix_info->UniqueId = cpu_to_le64(stat->ino);
	unix_info->Permissions = cpu_to_le64(stat->mode);
	unix_info->Nlinks = cpu_to_le64(stat->nlink);
}

/**
 * unix_info_to_attr() - convert smb file info format to unix attr format
 * @unix_info:	smb file information format
 * @attrs:	unix file/dir stat information
 *
 * Return:	0
 */
static int unix_info_to_attr(struct file_unix_basic_info *unix_info,
			     struct user_namespace *user_ns,
			     struct iattr *attrs)
{
	struct timespec64 ts;

	if (le64_to_cpu(unix_info->EndOfFile) != NO_CHANGE_64) {
		attrs->ia_size = le64_to_cpu(unix_info->EndOfFile);
		attrs->ia_valid |= ATTR_SIZE;
	}

	if (le64_to_cpu(unix_info->LastStatusChange) != NO_CHANGE_64) {
		ts = smb_NTtimeToUnix(unix_info->LastStatusChange);
		attrs->ia_ctime = ts;
		attrs->ia_valid |= ATTR_CTIME;
	}

	if (le64_to_cpu(unix_info->LastAccessTime) != NO_CHANGE_64) {
		ts = smb_NTtimeToUnix(unix_info->LastAccessTime);
		attrs->ia_atime = ts;
		attrs->ia_valid |= ATTR_ATIME;
	}

	if (le64_to_cpu(unix_info->LastModificationTime) != NO_CHANGE_64) {
		ts = smb_NTtimeToUnix(unix_info->LastModificationTime);
		attrs->ia_mtime = ts;
		attrs->ia_valid |= ATTR_MTIME;
	}

	if (le64_to_cpu(unix_info->Uid) != NO_CHANGE_64) {
		attrs->ia_uid = make_kuid(user_ns,
				le64_to_cpu(unix_info->Uid));
		attrs->ia_valid |= ATTR_UID;
	}

	if (le64_to_cpu(unix_info->Gid) != NO_CHANGE_64) {
		attrs->ia_gid = make_kgid(user_ns,
					  le64_to_cpu(unix_info->Gid));
		attrs->ia_valid |= ATTR_GID;
	}

	if (le64_to_cpu(unix_info->Permissions) != NO_CHANGE_64) {
		attrs->ia_mode = le64_to_cpu(unix_info->Permissions);
		attrs->ia_valid |= ATTR_MODE;
	}

	switch (le32_to_cpu(unix_info->Type)) {
	case UNIX_FILE:
		attrs->ia_mode |= S_IFREG;
		break;
	case UNIX_DIR:
		attrs->ia_mode |= S_IFDIR;
		break;
	case UNIX_SYMLINK:
		attrs->ia_mode |= S_IFLNK;
		break;
	case UNIX_CHARDEV:
		attrs->ia_mode |= S_IFCHR;
		break;
	case UNIX_BLOCKDEV:
		attrs->ia_mode |= S_IFBLK;
		break;
	case UNIX_FIFO:
		attrs->ia_mode |= S_IFIFO;
		break;
	case UNIX_SOCKET:
		attrs->ia_mode |= S_IFSOCK;
		break;
	default:
		pr_err("unknown file type 0x%x\n",
		       le32_to_cpu(unix_info->Type));
	}

	return 0;
}

/**
 * unix_to_dos_time() - convert unix time to dos format
 * @ts:		unix style time
 * @time:	store dos style time
 * @date:	store dos style date
 */
static void unix_to_dos_time(struct timespec64 ts, __le16 *time, __le16 *date)
{
	struct tm t;
	__u16 val;

	time64_to_tm(ts.tv_sec, (-sys_tz.tz_minuteswest) * 60, &t);
	val = (((unsigned int)(t.tm_mon + 1)) >> 3) | ((t.tm_year - 80) << 1);
	val = ((val & 0xFF) << 8) | (t.tm_mday |
			(((t.tm_mon + 1) & 0x7) << 5));
	*date = cpu_to_le16(val);

	val = ((((unsigned int)t.tm_min >> 3) & 0x7) |
			(((unsigned int)t.tm_hour) << 3));
	val = ((val & 0xFF) << 8) | ((t.tm_sec/2) | ((t.tm_min & 0x7) << 5));
	*time = cpu_to_le16(val);
}

/**
 * cifs_convert_ace() - helper function for convert an Access Control Entry
 *		from cifs wire format to local POSIX xattr format
 * @ace:	local - unix style Access Control Entry format
 * @cifs_ace:	cifs wire Access Control Entry format
 */
static void cifs_convert_ace(struct posix_acl_xattr_entry *ace,
		struct cifs_posix_ace *cifs_ace)
{
	/* u8 cifs fields do not need le conversion */
	ace->e_perm = cpu_to_le16(cifs_ace->cifs_e_perm);
	ace->e_tag  = cpu_to_le16(cifs_ace->cifs_e_tag);
	ace->e_id   = cpu_to_le32(le64_to_cpu(cifs_ace->cifs_uid));
}

/**
 * cifs_copy_posix_acl() - Convert ACL from CIFS POSIX wire format to local
 *		Linux POSIX ACL xattr
 * @trgt:	target buffer for storing in local ace format
 * @src:	source buffer in cifs ace format
 * @buflen:	target buffer length
 * @acl_type:	ace type
 * @size_of_data_area:	max buffer size to store ace xattr
 *
 * Return:	size of convert ace xattr on success, otherwise error
 */
static int cifs_copy_posix_acl(char *trgt, char *src, const int buflen,
		const int acl_type, const int size_of_data_area)
{
	int size =  0;
	int i;
	__u16 count;
	struct cifs_posix_ace *pACE;
	struct cifs_posix_acl *cifs_acl = (struct cifs_posix_acl *)src;
	struct posix_acl_xattr_entry *ace;
	struct posix_acl_xattr_header *local_acl = (void *)trgt;

	if (le16_to_cpu(cifs_acl->version) != CIFS_ACL_VERSION)
		return -EOPNOTSUPP;

	if (acl_type & ACL_TYPE_ACCESS) {
		count = le16_to_cpu(cifs_acl->access_entry_count);
		pACE = &cifs_acl->ace_array[0];
		size = sizeof(struct cifs_posix_acl);
		size += sizeof(struct cifs_posix_ace) * count;
		/* check if we would go beyond end of SMB */
		if (size_of_data_area < size) {
			ksmbd_debug(SMB, "bad CIFS POSIX ACL size %d vs. %d\n",
				 size_of_data_area, size);
			return -EINVAL;
		}
	} else if (acl_type & ACL_TYPE_DEFAULT) {
		count = le16_to_cpu(cifs_acl->default_entry_count);
		pACE = &cifs_acl->ace_array[0];
		size = sizeof(struct cifs_posix_acl);
		size += sizeof(struct cifs_posix_ace) * count;
		/* check if we would go beyond end of SMB */
		if (size_of_data_area < size)
			return -EINVAL;
	} else {
		/* illegal type */
		return -EINVAL;
	}

	size = posix_acl_xattr_size(count);
	if ((buflen != 0) && local_acl && size > buflen)
		return -ERANGE;

	/* buffer big enough */
	ace = (void *)(local_acl + 1);
	local_acl->a_version = cpu_to_le32(POSIX_ACL_XATTR_VERSION);
	for (i = 0; i < count; i++) {
		cifs_convert_ace(&ace[i], pACE);
		pACE++;
	}

	return size;
}

/**
 * convert_ace_to_cifs_ace() - helper function to convert ACL from local
 * Linux POSIX ACL xattr to CIFS POSIX wire format to local
 * @cifs_ace:	target buffer for storing in cifs ace format
 * @local_ace:	source buffer in Linux POSIX ACL xattr format
 *
 * Return:	0
 */
static __u16 convert_ace_to_cifs_ace(struct cifs_posix_ace *cifs_ace,
		const struct posix_acl_xattr_entry *local_ace)
{
	__u16 rc = 0; /* 0 = ACL converted ok */

	cifs_ace->cifs_e_perm = le16_to_cpu(local_ace->e_perm);
	cifs_ace->cifs_e_tag =  le16_to_cpu(local_ace->e_tag);
	/* BB is there a better way to handle the large uid? */
	if (local_ace->e_id == cpu_to_le32(-1)) {
		/* Probably no need to le convert -1 on any
		 * arch but can not hurt
		 */
		cifs_ace->cifs_uid = cpu_to_le64(-1);
	} else
		cifs_ace->cifs_uid = cpu_to_le64(le32_to_cpu(local_ace->e_id));
	return rc;
}

/**
 * ACL_to_cifs_posix() - ACL from local Linux POSIX xattr to CIFS POSIX ACL
 *		wire format
 * @parm_data:	target buffer for storing in cifs ace format
 * @pACL:	source buffer in cifs ace format
 * @buflen:	target buffer length
 * @acl_type:	ace type
 *
 * Return:	0 on success, otherwise error
 */
static __u16 ACL_to_cifs_posix(char *parm_data, const char *pACL,
		const int buflen, const int acl_type)
{
	__u16 rc = 0;
	struct cifs_posix_acl *cifs_acl = (struct cifs_posix_acl *)parm_data;
	struct posix_acl_xattr_header *local_acl = (void *)pACL;
	struct posix_acl_xattr_entry *ace = (void *)(local_acl + 1);
	int count;
	int i, j = 0;

	if ((buflen == 0) || !pACL || !cifs_acl)
		return 0;

	count = posix_acl_xattr_count((size_t)buflen);
	ksmbd_debug(SMB, "setting acl with %d entries from buf of length %d and version of %d\n",
		 count, buflen, le32_to_cpu(local_acl->a_version));
	if (le32_to_cpu(local_acl->a_version) != 2) {
		ksmbd_debug(SMB, "unknown POSIX ACL version %d\n",
			 le32_to_cpu(local_acl->a_version));
		return 0;
	}
	if (acl_type == ACL_TYPE_ACCESS) {
		cifs_acl->access_entry_count = cpu_to_le16(count);
		j = 0;
	} else if (acl_type == ACL_TYPE_DEFAULT) {
		cifs_acl->default_entry_count = cpu_to_le16(count);
		if (cifs_acl->access_entry_count)
			j = le16_to_cpu(cifs_acl->access_entry_count);
	} else {
		ksmbd_debug(SMB, "unknown ACL type %d\n", acl_type);
		return 0;
	}
	for (i = 0; i < count; i++, j++) {
		rc = convert_ace_to_cifs_ace(&cifs_acl->ace_array[i], &ace[i]);
		if (rc != 0) {
			/* ACE not converted */
			break;
		}
	}
	if (rc == 0) {
		rc = (__u16)(count * sizeof(struct cifs_posix_ace));
		/* BB add check to make sure ACL does not overflow SMB */
	}
	return rc;
}

/**
 * smb_get_acl() - handler for query posix acl information
 * @work:	smb work containing posix acl query command
 * @path:	path of file/dir to query acl
 *
 * Return:	0 on success, otherwise error
 */
static int smb_get_acl(struct ksmbd_work *work, struct path *path)
{
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	char *buf = NULL;
	int rc = 0, value_len;
	struct cifs_posix_acl *aclbuf;
	__u16 rsp_data_cnt = 0;

	aclbuf = (struct cifs_posix_acl *)(work->response_buf +
			sizeof(struct smb_com_trans2_rsp) + 4);

	aclbuf->version = cpu_to_le16(CIFS_ACL_VERSION);
	aclbuf->default_entry_count = 0;
	aclbuf->access_entry_count = 0;

	/* check if POSIX_ACL_XATTR_ACCESS exists */
	value_len = ksmbd_vfs_getxattr(mnt_user_ns(path->mnt), path->dentry,
				       XATTR_NAME_POSIX_ACL_ACCESS,
				       &buf);
	if (value_len > 0) {
		rsp_data_cnt += ACL_to_cifs_posix((char *)aclbuf, buf,
				value_len, ACL_TYPE_ACCESS);
		kfree(buf);
		buf = NULL;
	}

	/* check if POSIX_ACL_XATTR_DEFAULT exists */
	value_len = ksmbd_vfs_getxattr(mnt_user_ns(path->mnt), path->dentry,
				       XATTR_NAME_POSIX_ACL_DEFAULT,
				       &buf);
	if (value_len > 0) {
		rsp_data_cnt += ACL_to_cifs_posix((char *)aclbuf, buf,
				value_len, ACL_TYPE_DEFAULT);
		kfree(buf);
		buf = NULL;
	}

	if (rsp_data_cnt)
		rsp_data_cnt += sizeof(struct cifs_posix_acl);

	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = cpu_to_le16(rsp_data_cnt);
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = cpu_to_le16(2);
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = rsp->t2.TotalDataCount;
	rsp->t2.DataOffset = cpu_to_le16(60);
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;
	rsp->ByteCount = cpu_to_le16(rsp_data_cnt + 5);
	inc_rfc1001_len(&rsp->hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));

	if (buf)
		kfree(buf);
	return rc;
}

/**
 * smb_set_acl() - handler for setting posix acl information
 * @work:	smb work containing posix acl set command
 *
 * Return:	0 on success, otherwise error
 */
static int smb_set_acl(struct ksmbd_work *work)
{
	struct smb_com_trans2_spi_req *req = work->request_buf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct cifs_posix_acl *wire_acl_data;
	char *fname, *buf = NULL;
	int rc = 0, acl_type = 0, value_len;

	fname = smb_get_name(share, req->FileName, PATH_MAX, work, false);
	if (IS_ERR(fname)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(fname);
	}

	buf = vmalloc(XATTR_SIZE_MAX);
	if (!buf) {
		rsp->hdr.Status.CifsError = STATUS_NO_MEMORY;
		rc = -ENOMEM;
		goto out;
	}

	wire_acl_data = (struct cifs_posix_acl *)(((char *) &req->hdr.Protocol)
			+ le16_to_cpu(req->DataOffset));
	if (le16_to_cpu(wire_acl_data->access_entry_count) > 0 &&
		le16_to_cpu(wire_acl_data->access_entry_count) < 0xFFFF) {
		acl_type = ACL_TYPE_ACCESS;

	} else if (le16_to_cpu(wire_acl_data->default_entry_count) > 0 &&
		le16_to_cpu(wire_acl_data->default_entry_count) < 0xFFFF) {
		acl_type = ACL_TYPE_DEFAULT;
	} else {
		rc = -EINVAL;
		goto out;
	}

	rc = cifs_copy_posix_acl(buf,
			(char *)wire_acl_data,
			XATTR_SIZE_MAX, acl_type, XATTR_SIZE_MAX);
	if (rc < 0) {
		rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
		goto out;
	}

	value_len = rc;
	if (acl_type == ACL_TYPE_ACCESS) {
		rc = ksmbd_vfs_fsetxattr(work,
					 fname,
					 XATTR_NAME_POSIX_ACL_ACCESS,
					 buf, value_len, 0);
	} else if (acl_type == ACL_TYPE_DEFAULT) {
		rc = ksmbd_vfs_fsetxattr(work,
					 fname,
					 XATTR_NAME_POSIX_ACL_DEFAULT,
					 buf, value_len, 0);
	}

	if (rc < 0) {
		rsp->hdr.Status.CifsError = STATUS_UNEXPECTED_IO_ERROR;
		goto out;
	}

	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = cpu_to_le16(0);
	rsp->t2.ParameterCount = rsp->t2.TotalParameterCount;
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = cpu_to_le16(2);
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = rsp->t2.TotalDataCount;
	rsp->t2.DataOffset = cpu_to_le16(0);
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;

	/* 2 for parameter count + 1 pad1*/
	rsp->ByteCount = cpu_to_le16(3);
	rsp->Pad = 0;
	inc_rfc1001_len(&rsp->hdr, rsp->hdr.WordCount * 2 + 3);

out:
	if (buf)
		vfree(buf);
	kfree(fname);
	return rc;
}

static void *ksmbd_realloc_response(void *ptr, size_t old_sz, size_t new_sz)
{
	size_t sz = min(old_sz, new_sz);
	void *nptr;

	nptr = kvmalloc(new_sz, GFP_KERNEL | __GFP_ZERO);
	if (!nptr)
		return ptr;
	memcpy(nptr, ptr, sz);
	kvfree(ptr);
	return nptr;
}

/**
 * smb_readlink() - handler for reading symlink source path
 * @work:	smb work containing query link information
 *
 * Return:	0 on success, otherwise error
 */
static int smb_readlink(struct ksmbd_work *work, struct path *path)
{
	struct smb_com_trans2_qpi_req *req = work->request_buf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	int err, name_len;
	char *buf, *ptr;

	buf = kzalloc((CIFS_MF_SYMLINK_LINK_MAXLEN), GFP_KERNEL);
	if (!buf) {
		rsp->hdr.Status.CifsError = STATUS_NO_MEMORY;
		return -ENOMEM;
	}

	err = ksmbd_vfs_readlink(path, buf, CIFS_MF_SYMLINK_LINK_MAXLEN);
	if (err < 0) {
		rsp->hdr.Status.CifsError = STATUS_INVALID_HANDLE;
		goto out;
	}

	/*
	 * check if this namelen(unicode) and smb header can fit in small rsp
	 * buf. If not, switch to large rsp buffer.
	 */
	err++;
	err *= 2;
	if (err + MAX_HEADER_SIZE(work->conn) > work->response_sz) {
		void *nptr;
		size_t nsz = err + MAX_HEADER_SIZE(work->conn);

		nptr = ksmbd_realloc_response(work->response_buf,
					      work->response_sz,
					      nsz);
		if (nptr == work->response_buf) {
			rsp->hdr.Status.CifsError = STATUS_NO_MEMORY;
			err = -ENOMEM;
			goto out;
		}

		work->response_buf = nptr;
		rsp = (struct smb_com_trans2_rsp *)work->response_buf;
	}
	err = 0;

	ptr = (char *)&rsp->Buffer[0];
	memset(ptr, 0, 4);
	ptr += 4;

	if (is_smbreq_unicode(&req->hdr)) {
		name_len = smb_strtoUTF16((__le16 *)ptr,
					  buf,
					  CIFS_MF_SYMLINK_LINK_MAXLEN,
					  work->conn->local_nls);
		name_len++;     /* trailing null */
		name_len *= 2;
	} else { /* BB add path length overrun check */
		name_len = strscpy(ptr, buf, CIFS_MF_SYMLINK_LINK_MAXLEN - 1);
		name_len++;     /* trailing null */
	}

	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = cpu_to_le16(name_len);
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = cpu_to_le16(2);
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = rsp->t2.TotalDataCount;
	rsp->t2.DataOffset = cpu_to_le16(60);
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;
	rsp->ByteCount = cpu_to_le16(name_len + 5);
	inc_rfc1001_len(&rsp->hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));

out:
	kfree(buf);
	return err;
}

/**
 * smb_get_ea() - handler for extended attribute query
 * @work:	smb work containing query xattr command
 * @path:	path of file/dir to query xattr command
 *
 * Return:	0 on success, otherwise error
 */
static int smb_get_ea(struct ksmbd_work *work, struct path *path)
{
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	char *name, *ptr, *xattr_list = NULL, *buf;
	int rc, name_len, value_len, xattr_list_len;
	struct fealist *eabuf = (struct fealist *)(work->response_buf +
			sizeof(struct smb_com_trans2_rsp) + 4);
	struct fea *temp_fea;
	ssize_t buf_free_len;
	__u16 rsp_data_cnt = 4;

	eabuf->list_len = cpu_to_le32(rsp_data_cnt);
	buf_free_len = work->response_sz - (get_rfc1002_len(rsp) + 4) -
		sizeof(struct smb_com_trans2_rsp);
	rc = ksmbd_vfs_listxattr(path->dentry, &xattr_list);
	if (rc < 0) {
		rsp->hdr.Status.CifsError = STATUS_INVALID_HANDLE;
		goto out;
	} else if (!rc) { /* there is no EA in the file */
		eabuf->list_len = cpu_to_le32(rsp_data_cnt);
		goto done;
	}

	xattr_list_len = rc;
	rc = 0;

	ptr = (char *)eabuf->list;
	temp_fea = (struct fea *)ptr;
	for (name = xattr_list; name - xattr_list < xattr_list_len;
			name += strlen(name) + 1) {
		ksmbd_debug(SMB, "%s, len %zd\n", name, strlen(name));
		/*
		 * CIFS does not support EA other name user.* namespace,
		 * still keep the framework generic, to list other attrs
		 * in future.
		 */
		if (strncmp(name, XATTR_USER_PREFIX, XATTR_USER_PREFIX_LEN))
			continue;

		name_len = strlen(name);
		if (!strncmp(name, XATTR_USER_PREFIX, XATTR_USER_PREFIX_LEN))
			name_len -= XATTR_USER_PREFIX_LEN;

		ptr = (char *)(&temp_fea->name + name_len + 1);
		buf_free_len -= (offsetof(struct fea, name) + name_len + 1);

		value_len = ksmbd_vfs_getxattr(mnt_user_ns(path->mnt),
					       path->dentry, name, &buf);
		if (value_len <= 0) {
			rc = -ENOENT;
			rsp->hdr.Status.CifsError = STATUS_INVALID_HANDLE;
			goto out;
		}

		memcpy(ptr, buf, value_len);
		kfree(buf);

		temp_fea->EA_flags = 0;
		temp_fea->name_len = name_len;
		if (!strncmp(name, XATTR_USER_PREFIX, XATTR_USER_PREFIX_LEN))
			memcpy(temp_fea->name, &name[XATTR_USER_PREFIX_LEN],
					name_len);
		else
			memcpy(temp_fea->name, name, name_len);

		temp_fea->value_len = cpu_to_le16(value_len);
		buf_free_len -= value_len;
		rsp_data_cnt += offsetof(struct fea, name) + name_len + 1 +
			value_len;
		eabuf->list_len += cpu_to_le32(offsetof(struct fea, name) +
				name_len + 1 + value_len);
		ptr += value_len;
		temp_fea = (struct fea *)ptr;
	}

done:
	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = cpu_to_le16(rsp_data_cnt);
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = cpu_to_le16(2);
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = rsp->t2.TotalDataCount;
	rsp->t2.DataOffset = cpu_to_le16(60);
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;
	rsp->ByteCount = cpu_to_le16(rsp_data_cnt + 5);
	inc_rfc1001_len(&rsp->hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));
out:
	kvfree(xattr_list);
	return rc;
}

/**
 * query_path_info() - handler for query path info
 * @work:	smb work containing query path info command
 *
 * Return:	0 on success, otherwise error
 */
static int query_path_info(struct ksmbd_work *work)
{
	struct smb_hdr *rsp_hdr = work->response_buf;
	struct smb_com_trans2_req *req = work->request_buf;
	struct ksmbd_conn *conn = work->conn;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	struct trans2_qpi_req_params *req_params;
	char *name = NULL;
	struct path path;
	struct kstat st;
	int rc;
	char *ptr;
	__u64 create_time = 0, time;

	if (test_share_config_flag(work->tcon->share_conf,
				   KSMBD_SHARE_FLAG_PIPE)) {
		rsp_hdr->Status.CifsError = STATUS_UNEXPECTED_IO_ERROR;
		return 0;
	}

	req_params = (struct trans2_qpi_req_params *)(work->request_buf +
		     le16_to_cpu(req->ParameterOffset) + 4);
	name = smb_get_name(share, req_params->FileName, PATH_MAX, work,
		false);
	if (IS_ERR(name)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(name);
	}

	if (ksmbd_override_fsids(work)) {
		kfree(name);
		rsp->hdr.Status.CifsError = STATUS_NO_MEMORY;
		return -ENOMEM;
	}

	rc = ksmbd_vfs_kern_path(work, name, LOOKUP_NO_SYMLINKS, &path, 0);
	if (rc) {
		if (rc == -EACCES || rc == -EXDEV)
			rsp_hdr->Status.CifsError = STATUS_ACCESS_DENIED;
		else
			rsp_hdr->Status.CifsError =
					STATUS_OBJECT_NAME_NOT_FOUND;
		ksmbd_debug(SMB, "cannot get linux path for %s, err %d\n",
				name, rc);
		goto out;
	}

	if (d_is_symlink(path.dentry)) {
		rsp_hdr->Status.CifsError = STATUS_ACCESS_DENIED;
		goto err_out;
	}

	rc = vfs_getattr(&path, &st, STATX_BASIC_STATS, AT_STATX_SYNC_AS_STAT);
	if (rc) {
		pr_err("cannot get stat information\n");
		goto err_out;
	}

	if (test_share_config_flag(work->tcon->share_conf,
				   KSMBD_SHARE_FLAG_STORE_DOS_ATTRS)) {
		struct xattr_dos_attrib da;

		rc = ksmbd_vfs_get_dos_attrib_xattr(mnt_user_ns(path.mnt),
						    path.dentry, &da);
		if (rc > 0)
			create_time = da.create_time;
		rc = 0;
	}

	switch (le16_to_cpu(req_params->InformationLevel)) {
	case SMB_INFO_STANDARD:
	{
		struct file_info_standard *infos;

		ksmbd_debug(SMB, "SMB_INFO_STANDARD\n");
		rc = ksmbd_query_inode_status(d_inode(path.dentry));
		if (rc == KSMBD_INODE_STATUS_PENDING_DELETE) {
			rc = -EBUSY;
			goto err_out;
		}

		rc = 0;
		ptr = (char *)&rsp->Pad + 1;
		memset(ptr, 0, 4);
		infos = (struct file_info_standard *)(ptr + 4);
		unix_to_dos_time(ksmbd_NTtimeToUnix(cpu_to_le64(create_time)),
			&infos->CreationDate, &infos->CreationTime);
		unix_to_dos_time(st.atime,
				&infos->LastAccessDate,
				&infos->LastAccessTime);
		unix_to_dos_time(st.mtime,
				&infos->LastWriteDate,
				&infos->LastWriteTime);
		infos->DataSize = cpu_to_le32(st.size);
		infos->AllocationSize = cpu_to_le32(st.blocks << 9);
		infos->Attributes = cpu_to_le16(S_ISDIR(st.mode) ?
					ATTR_DIRECTORY : ATTR_ARCHIVE);
		infos->EASize = 0;

		rsp_hdr->WordCount = 10;
		rsp->t2.TotalParameterCount = cpu_to_le16(2);
		rsp->t2.TotalDataCount = cpu_to_le16(22);
		rsp->t2.Reserved = 0;
		rsp->t2.ParameterCount = cpu_to_le16(2);
		rsp->t2.ParameterOffset = cpu_to_le16(56);
		rsp->t2.ParameterDisplacement = 0;
		rsp->t2.DataCount = cpu_to_le16(22);
		rsp->t2.DataOffset = cpu_to_le16(60);
		rsp->t2.DataDisplacement = 0;
		rsp->t2.SetupCount = 0;
		rsp->t2.Reserved1 = 0;
		rsp->ByteCount = cpu_to_le16(27);
		rsp->Pad = 0;
		inc_rfc1001_len(rsp_hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));
		break;
	}
	case SMB_QUERY_FILE_STANDARD_INFO:
	{
		struct file_standard_info *standard_info;
		unsigned int del_pending;

		ksmbd_debug(SMB, "SMB_QUERY_FILE_STANDARD_INFO\n");
		del_pending = ksmbd_query_inode_status(d_inode(path.dentry));
		if (del_pending == KSMBD_INODE_STATUS_PENDING_DELETE)
			del_pending = 1;
		else
			del_pending = 0;

		rsp_hdr->WordCount = 10;
		rsp->t2.TotalParameterCount = cpu_to_le16(2);
		rsp->t2.TotalDataCount =
			cpu_to_le16(sizeof(struct file_standard_info));
		rsp->t2.Reserved = 0;
		rsp->t2.ParameterCount = cpu_to_le16(2);
		rsp->t2.ParameterOffset = cpu_to_le16(56);
		rsp->t2.ParameterDisplacement = 0;
		rsp->t2.DataCount =
			cpu_to_le16(sizeof(struct file_standard_info));
		rsp->t2.DataOffset = cpu_to_le16(60);
		rsp->t2.DataDisplacement = 0;
		rsp->t2.SetupCount = 0;
		rsp->t2.Reserved1 = 0;
		/*2 for parameter count & 3 pad (1pad1 + 2 pad2)*/
		rsp->ByteCount =
			cpu_to_le16(2 + sizeof(struct file_standard_info) + 3);
		rsp->Pad = 0;
		/* lets set EA info */
		ptr = (char *)&rsp->Pad + 1;
		memset(ptr, 0, 4);
		standard_info = (struct file_standard_info *)(ptr + 4);
		standard_info->AllocationSize = cpu_to_le64(st.blocks << 9);
		standard_info->EndOfFile = cpu_to_le64(st.size);
		standard_info->NumberOfLinks = cpu_to_le32(get_nlink(&st) -
			del_pending);
		standard_info->DeletePending = del_pending;
		standard_info->Directory = S_ISDIR(st.mode) ? 1 : 0;
		inc_rfc1001_len(rsp_hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));
		break;
	}
	case SMB_QUERY_FILE_BASIC_INFO:
	{
		struct file_basic_info *basic_info;

		ksmbd_debug(SMB, "SMB_QUERY_FILE_BASIC_INFO\n");
		rsp_hdr->WordCount = 10;
		rsp->t2.TotalParameterCount = cpu_to_le16(2);
		rsp->t2.TotalDataCount =
			cpu_to_le16(sizeof(struct file_basic_info));
		rsp->t2.Reserved = 0;
		rsp->t2.ParameterCount = cpu_to_le16(2);
		rsp->t2.ParameterOffset = cpu_to_le16(56);
		rsp->t2.ParameterDisplacement = 0;
		rsp->t2.DataCount = cpu_to_le16(sizeof(struct file_basic_info));
		rsp->t2.DataOffset = cpu_to_le16(60);
		rsp->t2.DataDisplacement = 0;
		rsp->t2.SetupCount = 0;
		rsp->t2.Reserved1 = 0;
		/*2 for parameter count & 3 pad (1pad1 + 2 pad2)*/
		rsp->ByteCount =
			cpu_to_le16(2 + sizeof(struct file_basic_info) + 3);
		rsp->Pad = 0;
		/* lets set EA info */
		ptr = (char *)&rsp->Pad + 1;
		memset(ptr, 0, 4);
		basic_info = (struct file_basic_info *)(ptr + 4);
		basic_info->CreationTime = cpu_to_le64(create_time);
		time = ksmbd_UnixTimeToNT(st.atime);
		basic_info->LastAccessTime = cpu_to_le64(time);
		time = ksmbd_UnixTimeToNT(st.mtime);
		basic_info->LastWriteTime = cpu_to_le64(time);
		time = ksmbd_UnixTimeToNT(st.ctime);
		basic_info->ChangeTime = cpu_to_le64(time);
		basic_info->Attributes = S_ISDIR(st.mode) ?
					 ATTR_DIRECTORY_LE : ATTR_ARCHIVE_LE;
		basic_info->Pad = 0;
		inc_rfc1001_len(rsp_hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));
		break;
	}
	case SMB_QUERY_FILE_EA_INFO:
	{
		struct file_ea_info *ea_info;

		ksmbd_debug(SMB, "SMB_QUERY_FILE_EA_INFO\n");
		rsp_hdr->WordCount = 10;
		rsp->t2.TotalParameterCount = cpu_to_le16(2);
		rsp->t2.TotalDataCount =
			cpu_to_le16(sizeof(struct file_ea_info));
		rsp->t2.Reserved = 0;
		rsp->t2.ParameterCount = cpu_to_le16(2);
		rsp->t2.ParameterOffset = cpu_to_le16(56);
		rsp->t2.ParameterDisplacement = 0;
		rsp->t2.DataCount = cpu_to_le16(sizeof(struct file_ea_info));
		rsp->t2.DataOffset = cpu_to_le16(60);
		rsp->t2.DataDisplacement = 0;
		rsp->t2.SetupCount = 0;
		rsp->t2.Reserved1 = 0;
		/*2 for parameter count & 3 pad (1pad1 + 2 pad2)*/
		rsp->ByteCount =
			cpu_to_le16(2 + sizeof(struct file_ea_info) + 3);
		rsp->Pad = 0;
		/* lets set EA info */
		ptr = (char *)&rsp->Pad + 1;
		memset(ptr, 0, 4);
		ea_info = (struct file_ea_info *)(ptr + 4);
		ea_info->EaSize = 0;
		inc_rfc1001_len(rsp_hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));
		break;
	}
	case SMB_QUERY_FILE_NAME_INFO:
	{
		struct file_name_info *name_info;
		int uni_filename_len;
		char *filename;

		ksmbd_debug(SMB, "SMB_QUERY_FILE_NAME_INFO\n");
		ptr = (char *)&rsp->Pad + 1;
		memset(ptr, 0, 4);
		name_info = (struct file_name_info *)(ptr + 4);

		filename = convert_to_nt_pathname(work->tcon->share_conf, &path);
		if (!filename) {
			rc = -ENOMEM;
			goto err_out;
		}
		uni_filename_len = smbConvertToUTF16(
				(__le16 *)name_info->FileName,
				filename, PATH_MAX,
				conn->local_nls, 0);
		kfree(filename);
		uni_filename_len *= 2;
		name_info->FileNameLength = cpu_to_le32(uni_filename_len);

		rsp_hdr->WordCount = 10;
		rsp->t2.TotalParameterCount = cpu_to_le16(2);
		rsp->t2.TotalDataCount = cpu_to_le16(uni_filename_len + 4);
		rsp->t2.Reserved = 0;
		rsp->t2.ParameterCount = cpu_to_le16(2);
		rsp->t2.ParameterOffset = cpu_to_le16(56);
		rsp->t2.ParameterDisplacement = 0;
		rsp->t2.DataCount = cpu_to_le16(uni_filename_len + 4);
		rsp->t2.DataOffset = cpu_to_le16(60);
		rsp->t2.DataDisplacement = 0;
		rsp->t2.SetupCount = 0;
		rsp->t2.Reserved1 = 0;
		/*2 for parameter count & 3 pad (1pad1 + 2 pad2)*/
		rsp->ByteCount = cpu_to_le16(2 + uni_filename_len + 4 + 3);
		rsp->Pad = 0;
		inc_rfc1001_len(rsp_hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));
		break;
	}
	case SMB_QUERY_FILE_ALL_INFO:
	{
		struct file_all_info *ainfo;
		unsigned int del_pending;
		char *filename;
		int uni_filename_len, total_count = 72;

		ksmbd_debug(SMB, "SMB_QUERY_FILE_ALL_INFO\n");

		del_pending = ksmbd_query_inode_status(d_inode(path.dentry));
		if (del_pending == KSMBD_INODE_STATUS_PENDING_DELETE)
			del_pending = 1;
		else
			del_pending = 0;

		filename = convert_to_nt_pathname(work->tcon->share_conf, &path);
		if (!filename) {
			rc = -ENOMEM;
			goto err_out;
		}

		/*
		 * Observation: sizeof smb_hdr is 33 bytes(including word count)
		 * After that: trans2 response 22 bytes when stepcount 0 and
		 * including ByteCount storage.
		 */
		/* lets set EA info */
		ptr = (char *)&rsp->Pad + 1;
		memset(ptr, 0, 4);
		ainfo = (struct file_all_info *) (ptr + 4);

		ainfo->CreationTime = cpu_to_le64(create_time);
		time = ksmbd_UnixTimeToNT(st.atime);
		ainfo->LastAccessTime = cpu_to_le64(time);
		time = ksmbd_UnixTimeToNT(st.mtime);
		ainfo->LastWriteTime = cpu_to_le64(time);
		time = ksmbd_UnixTimeToNT(st.ctime);
		ainfo->ChangeTime = cpu_to_le64(time);
		ainfo->Attributes = S_ISDIR(st.mode) ?
					ATTR_DIRECTORY_LE : ATTR_ARCHIVE_LE;
		ainfo->Pad1 = 0;
		ainfo->AllocationSize = cpu_to_le64(st.blocks << 9);
		ainfo->EndOfFile = cpu_to_le64(st.size);
		ainfo->NumberOfLinks = cpu_to_le32(get_nlink(&st) -
			del_pending);
		ainfo->DeletePending = del_pending;
		ainfo->Directory = S_ISDIR(st.mode) ? 1 : 0;
		ainfo->Pad2 = 0;
		ainfo->EASize = 0;
		uni_filename_len = smbConvertToUTF16(
				(__le16 *)ainfo->FileName,
				filename, PATH_MAX,
				conn->local_nls, 0);
		kfree(filename);
		uni_filename_len *= 2;
		ainfo->FileNameLength = cpu_to_le32(uni_filename_len);
		total_count += uni_filename_len;

		rsp_hdr->WordCount = 10;
		rsp->t2.TotalParameterCount = cpu_to_le16(2);
		/* add unicode name length of name */
		rsp->t2.TotalDataCount = cpu_to_le16(total_count);
		rsp->t2.Reserved = 0;
		rsp->t2.ParameterCount = cpu_to_le16(2);
		rsp->t2.ParameterOffset = cpu_to_le16(56);
		rsp->t2.ParameterDisplacement = 0;
		rsp->t2.DataCount = cpu_to_le16(total_count);
		rsp->t2.DataOffset = cpu_to_le16(60);
		rsp->t2.DataDisplacement = 0;
		rsp->t2.SetupCount = 0;
		rsp->t2.Reserved1 = 0;
		/* 2 for parameter count + 72 data count +
		 * filename length + 3 pad (1pad1 + 2 pad2)
		 */
		rsp->ByteCount = cpu_to_le16(5 + total_count);
		rsp->Pad = 0;
		inc_rfc1001_len(rsp_hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));
		break;
	}
	case SMB_QUERY_ALT_NAME_INFO:
	{
		struct alt_name_info *alt_name_info;
		char *base;
		int filename_len;

		ksmbd_debug(SMB, "SMB_QUERY_ALT_NAME_INFO\n");
		rsp_hdr->WordCount = 10;
		rsp->t2.TotalParameterCount = cpu_to_le16(2);
		rsp->t2.Reserved = 0;
		rsp->t2.ParameterCount = cpu_to_le16(2);
		rsp->t2.ParameterOffset = cpu_to_le16(56);
		rsp->t2.ParameterDisplacement = 0;
		rsp->t2.DataOffset = cpu_to_le16(60);
		rsp->t2.DataDisplacement = 0;
		rsp->t2.SetupCount = 0;
		rsp->t2.Reserved1 = 0;
		/*2 for parameter count & 3 pad (1pad1 + 2 pad2)*/
		rsp->ByteCount = cpu_to_le16(25);
		rsp->Pad = 0;
		/* lets set EA info */
		ptr = (char *)&rsp->Pad + 1;
		memset(ptr, 0, 4);
		alt_name_info = (struct alt_name_info *)(ptr + 4);

		base = strrchr(name, '/');
		if (!base)
			base = name;
		else
			base += 1;

		filename_len = ksmbd_extract_shortname(conn, base,
					alt_name_info->FileName);
		alt_name_info->FileNameLength = cpu_to_le32(filename_len);
		rsp->t2.TotalDataCount = cpu_to_le16(4 + filename_len);
		rsp->t2.DataCount = cpu_to_le16(4 + filename_len);

		inc_rfc1001_len(rsp_hdr, (4 + filename_len + 25));
		break;
	}
	case SMB_QUERY_FILE_UNIX_BASIC:
	{
		struct file_unix_basic_info *unix_info;

		ksmbd_debug(SMB, "SMB_QUERY_FILE_UNIX_BASIC\n");
		rsp_hdr->WordCount = 10;
		rsp->t2.TotalParameterCount = 0;
		rsp->t2.TotalDataCount = cpu_to_le16(100);
		rsp->t2.Reserved = 0;
		rsp->t2.ParameterCount = 0;
		rsp->t2.ParameterOffset = cpu_to_le16(56);
		rsp->t2.ParameterDisplacement = 0;
		rsp->t2.DataCount = cpu_to_le16(100);
		rsp->t2.DataOffset = cpu_to_le16(56);
		rsp->t2.DataDisplacement = 0;
		rsp->t2.SetupCount = 0;
		rsp->t2.Reserved1 = 0;
		rsp->ByteCount = cpu_to_le16(101); /* 100 data count + 1pad */
		rsp->Pad = 0;
		unix_info = (struct file_unix_basic_info *)(&rsp->Pad + 1);
		init_unix_info(unix_info, mnt_user_ns(path.mnt), &st);
		inc_rfc1001_len(rsp_hdr, (10 * 2 + 101));
		break;
	}
	case SMB_QUERY_FILE_INTERNAL_INFO:
	{
		struct file_internal_info *iinfo;

		ksmbd_debug(SMB, "SMB_QUERY_FILE_INTERNAL_INFO\n");
		rsp_hdr->WordCount = 10;
		rsp->t2.TotalParameterCount = cpu_to_le16(2);
		rsp->t2.TotalDataCount = cpu_to_le16(8);
		rsp->t2.Reserved = 0;
		rsp->t2.ParameterCount = cpu_to_le16(2);
		rsp->t2.ParameterOffset = cpu_to_le16(56);
		rsp->t2.ParameterDisplacement = 0;
		rsp->t2.DataCount = cpu_to_le16(8);
		rsp->t2.DataOffset = cpu_to_le16(60);
		rsp->t2.DataDisplacement = 0;
		rsp->t2.SetupCount = 0;
		rsp->t2.Reserved1 = 0;
		rsp->ByteCount = cpu_to_le16(13);
		rsp->Pad = 0;
		ptr = (char *)&rsp->Pad + 1;
		memset(ptr, 0, 4);
		iinfo = (struct file_internal_info *) (ptr + 4);
		iinfo->UniqueId = cpu_to_le64(st.ino);
		inc_rfc1001_len(rsp_hdr, (10 * 2 + 13));
		break;
	}
	case SMB_QUERY_FILE_UNIX_LINK:
		ksmbd_debug(SMB, "SMB_QUERY_FILE_UNIX_LINK\n");
		rc = smb_readlink(work, &path);
		if (rc < 0)
			goto err_out;
		break;
	case SMB_INFO_QUERY_ALL_EAS:
		ksmbd_debug(SMB, "SMB_INFO_QUERY_ALL_EAS\n");
		rc = smb_get_ea(work, &path);
		if (rc < 0)
			goto err_out;
		break;
	case SMB_QUERY_POSIX_ACL:
		ksmbd_debug(SMB, "SMB_QUERY_POSIX_ACL\n");
		rc = smb_get_acl(work, &path);
		if (rc < 0)
			goto err_out;
		break;
	default:
		pr_err("query path info not implemnted for %x\n",
		       le16_to_cpu(req_params->InformationLevel));
		rc = -EINVAL;
		goto err_out;
	}

err_out:
	path_put(&path);
out:
	ksmbd_revert_fsids(work);
	kfree(name);
	return rc;
}

/**
 * create_trans2_reply() - create response for trans2 request
 * @work:	smb work containing smb response buffer
 * @count:	trans2 response buffer size
 */
static void create_trans2_reply(struct ksmbd_work *work, __u16 count)
{
	struct smb_hdr *rsp_hdr = work->response_buf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;

	rsp_hdr->WordCount = 0x0A;
	rsp->t2.TotalParameterCount = 0;
	rsp->t2.TotalDataCount = cpu_to_le16(count);
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = 0;
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = cpu_to_le16(count);
	rsp->t2.DataOffset = cpu_to_le16(56);
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;

	rsp->ByteCount = cpu_to_le16(count + 1);
	rsp->Pad = 0;
	inc_rfc1001_len(rsp_hdr, 10 * 2 + (count + 1));
}

/**
 * set_fs_info() - handler for set fs info commands
 * @work:	smb work containing set fs info command buffer
 *
 * Return:	0 on success, otherwise error
 */
static int set_fs_info(struct ksmbd_work *work)
{
	struct smb_com_trans2_setfsi_req *req = work->request_buf;
	struct smb_com_trans2_setfsi_rsp *rsp = work->response_buf;
	int info_level = le16_to_cpu(req->InformationLevel);

	switch (info_level) {
	u64 client_cap;

	case SMB_SET_CIFS_UNIX_INFO:
		ksmbd_debug(SMB, "SMB_SET_CIFS_UNIX_INFO\n");
		if (le16_to_cpu(req->ClientUnixMajor) !=
			CIFS_UNIX_MAJOR_VERSION) {
			pr_err("Non compatible unix major info\n");
			return -EINVAL;
		}

		if (le16_to_cpu(req->ClientUnixMinor) !=
			CIFS_UNIX_MINOR_VERSION) {
			pr_err("Non compatible unix minor info\n");
			return -EINVAL;
		}

		client_cap = le64_to_cpu(req->ClientUnixCap);
		ksmbd_debug(SMB, "clients unix cap = %llx\n", client_cap);
		/* TODO: process caps */
		rsp->t2.TotalDataCount = 0;
		break;
	default:
		ksmbd_debug(SMB, "info level %x  not supported\n", info_level);
		return -EINVAL;
	}

	create_trans2_reply(work, le16_to_cpu(rsp->t2.TotalDataCount));
	return 0;
}

/**
 * query_fs_info() - handler for query fs info commands
 * @work:	smb work containing query fs info command buffer
 *
 * Return:	0 on success, otherwise error
 */
static int query_fs_info(struct ksmbd_work *work)
{
	struct smb_hdr *req_hdr = work->request_buf;
	struct smb_com_trans2_req *req = work->request_buf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	struct smb_com_trans2_qfsi_req_params *req_params;
	struct ksmbd_conn *conn = work->conn;
	struct kstatfs stfs;
	struct ksmbd_share_config *share;
	int rc;
	struct path path;
	bool incomplete = false;
	int info_level, len = 0;
	struct ksmbd_tree_connect *tree_conn;

	req_params = (struct smb_com_trans2_qfsi_req_params *)
		(work->request_buf + le16_to_cpu(req->ParameterOffset) + 4);
	/* check if more data is coming */
	if (le16_to_cpu(req->TotalParameterCount) !=
		le16_to_cpu(req->ParameterCount)) {
		ksmbd_debug(SMB, "total param = %d, received = %d\n",
			le16_to_cpu(req->TotalParameterCount),
			le16_to_cpu(req->ParameterCount));
		incomplete = true;
	}

	if (le16_to_cpu(req->TotalDataCount) != le16_to_cpu(req->DataCount)) {
		ksmbd_debug(SMB, "total data = %d, received = %d\n",
			le16_to_cpu(req->TotalDataCount),
			le16_to_cpu(req->DataCount));
		incomplete = true;
	}

	if (incomplete) {
		/* create 1 trans_state structure
		 * and add to connection list
		 */
	}

	info_level = le16_to_cpu(req_params->InformationLevel);

	tree_conn = ksmbd_tree_conn_lookup(work->sess,
					   le16_to_cpu(req_hdr->Tid));
	if (!tree_conn)
		return -ENOENT;
	share = tree_conn->share_conf;

	if (test_share_config_flag(share, KSMBD_SHARE_FLAG_PIPE))
		return -ENOENT;

	if (ksmbd_override_fsids(work))
		return -ENOMEM;

	rc = kern_path(share->path, LOOKUP_NO_SYMLINKS, &path);
	if (rc) {
		ksmbd_revert_fsids(work);
		pr_err("cannot create vfs path\n");
		return rc;
	}

	rc = vfs_statfs(&path, &stfs);
	if (rc) {
		pr_err("cannot do stat of path %s\n", share->path);
		goto err_out;
	}

	switch (info_level) {
	case SMB_INFO_ALLOCATION:
	{
		struct filesystem_alloc_info *ainfo;

		ksmbd_debug(SMB, "GOT SMB_INFO_ALLOCATION\n");
		rsp->t2.TotalDataCount = cpu_to_le16(18);
		ainfo = (struct filesystem_alloc_info *)(&rsp->Pad + 1);
		ainfo->fsid = 0;
		ainfo->BytesPerSector = cpu_to_le16(512);
		ainfo->SectorsPerAllocationUnit =
		cpu_to_le32(stfs.f_bsize/le16_to_cpu(ainfo->BytesPerSector));
		ainfo->TotalAllocationUnits = cpu_to_le32(stfs.f_blocks);
		ainfo->FreeAllocationUnits = cpu_to_le32(stfs.f_bfree);
		break;
	}
	case SMB_QUERY_FS_VOLUME_INFO:
	{
		struct filesystem_vol_info *vinfo;

		ksmbd_debug(SMB, "GOT SMB_QUERY_FS_VOLUME_INFO\n");
		vinfo = (struct filesystem_vol_info *)(&rsp->Pad + 1);
		vinfo->VolumeCreationTime = 0;
		/* Taking dummy value of serial number*/
		vinfo->SerialNumber = cpu_to_le32(0xbc3ac512);
		len = smbConvertToUTF16((__le16 *)vinfo->VolumeLabel,
			share->name, PATH_MAX, conn->local_nls, 0);
		vinfo->VolumeLabelSize = cpu_to_le32(len);
		vinfo->Reserved = 0;
		rsp->t2.TotalDataCount =
			cpu_to_le16(sizeof(struct filesystem_vol_info) +
				    len - 2);
		break;
	}
	case SMB_QUERY_FS_SIZE_INFO:
	{
		struct filesystem_info *sinfo;

		ksmbd_debug(SMB, "GOT SMB_QUERY_FS_SIZE_INFO\n");
		rsp->t2.TotalDataCount = cpu_to_le16(24);
		sinfo = (struct filesystem_info *)(&rsp->Pad + 1);
		sinfo->BytesPerSector = cpu_to_le32(512);
		sinfo->SectorsPerAllocationUnit =
			cpu_to_le32(stfs.f_bsize/sinfo->BytesPerSector);
		sinfo->TotalAllocationUnits = cpu_to_le64(stfs.f_blocks);
		sinfo->FreeAllocationUnits = cpu_to_le64(stfs.f_bfree);
		break;
	}
	case SMB_QUERY_FS_DEVICE_INFO:
	{
		struct filesystem_device_info *fdi;

		/* query fs info device info response is 0 word and 8 bytes */
		ksmbd_debug(SMB, "GOT SMB_QUERY_FS_DEVICE_INFO\n");
		if (le16_to_cpu(req->MaxDataCount) < 8) {
			pr_err("Insufficient bytes, cannot response()\n");
			rc = -EINVAL;
			goto err_out;
		}

		rsp->t2.TotalDataCount = cpu_to_le16(18);
		fdi = (struct filesystem_device_info *)(&rsp->Pad + 1);
		fdi->DeviceType = cpu_to_le32(FILE_DEVICE_DISK);
		fdi->DeviceCharacteristics = cpu_to_le32(0x20);
		break;
	}
	case SMB_QUERY_FS_ATTRIBUTE_INFO:
	{
		struct filesystem_attribute_info *info;

		ksmbd_debug(SMB, "GOT SMB_QUERY_FS_ATTRIBUTE_INFO\n");
		/* constant 12 bytes + variable filesystem name */
		info = (struct filesystem_attribute_info *)(&rsp->Pad + 1);

		if (le16_to_cpu(req->MaxDataCount) < 12) {
			pr_err("Insufficient bytes, cannot response()\n");
			rc = -EINVAL;
			goto err_out;
		}

		info->Attributes = cpu_to_le32(FILE_CASE_PRESERVED_NAMES |
				   FILE_CASE_SENSITIVE_SEARCH |
				   FILE_VOLUME_QUOTAS);
		info->MaxPathNameComponentLength = cpu_to_le32(stfs.f_namelen);
		info->FileSystemNameLen = 0;
		rsp->t2.TotalDataCount = cpu_to_le16(12);
		break;
	}
	case SMB_QUERY_CIFS_UNIX_INFO:
	{
		struct filesystem_unix_info *uinfo;

		ksmbd_debug(SMB, "GOT SMB_QUERY_CIFS_UNIX_INFO\n");
		/* constant 12 bytes + variable filesystem name */
		uinfo = (struct filesystem_unix_info *)(&rsp->Pad + 1);

		if (le16_to_cpu(req->MaxDataCount) < 12) {
			pr_err("Insufficient bytes, cannot response()\n");
			rc = -EINVAL;
			goto err_out;
		}
		uinfo->MajorVersionNumber =
			cpu_to_le16(CIFS_UNIX_MAJOR_VERSION);
		uinfo->MinorVersionNumber =
			cpu_to_le16(CIFS_UNIX_MINOR_VERSION);
		uinfo->Capability = cpu_to_le64(SMB_UNIX_CAPS);
		rsp->t2.TotalDataCount = cpu_to_le16(12);
		break;
	}
	case SMB_QUERY_POSIX_FS_INFO:
	{
		struct filesystem_posix_info *pinfo;

		ksmbd_debug(SMB, "GOT SMB_QUERY_POSIX_FS_INFO\n");
		rsp->t2.TotalDataCount = cpu_to_le16(56);
		pinfo = (struct filesystem_posix_info *)(&rsp->Pad + 1);
		pinfo->BlockSize = cpu_to_le32(stfs.f_bsize);
		pinfo->OptimalTransferSize = cpu_to_le32(stfs.f_blocks);
		pinfo->TotalBlocks = cpu_to_le64(stfs.f_blocks);
		pinfo->BlocksAvail = cpu_to_le64(stfs.f_bfree);
		pinfo->UserBlocksAvail = cpu_to_le64(stfs.f_bavail);
		pinfo->TotalFileNodes = cpu_to_le64(stfs.f_files);
		pinfo->FreeFileNodes = cpu_to_le64(stfs.f_ffree);
		pinfo->FileSysIdentifier = 0;
		break;
	}
	default:
		ksmbd_debug(SMB, "info level %x not implemented\n", info_level);
		rc = -EINVAL;
		goto err_out;
	}

	create_trans2_reply(work, le16_to_cpu(rsp->t2.TotalDataCount));

err_out:
	path_put(&path);
	ksmbd_revert_fsids(work);
	return rc;
}

/**
 * smb_posix_convert_flags() - convert smb posix access flags to open flags
 * @flags:	smb posix access flags
 *
 * Return:	file open flags
 */
static __u32 smb_posix_convert_flags(__u32 flags, int *may_flags)
{
	__u32 posix_flags = 0;

	if ((flags & SMB_ACCMODE) == SMB_O_RDONLY)
		posix_flags = O_RDONLY;
	else if ((flags & SMB_ACCMODE) == SMB_O_WRONLY)
		posix_flags = O_WRONLY;
	else if ((flags & SMB_ACCMODE) == SMB_O_RDWR)
		posix_flags = O_RDWR;

	if (flags & SMB_O_SYNC)
		posix_flags |= O_DSYNC;
	if (flags & SMB_O_DIRECTORY)
		posix_flags |= O_DIRECTORY;
	if (flags & SMB_O_NOFOLLOW)
		posix_flags |= O_NOFOLLOW;
	if (flags & SMB_O_APPEND)
		posix_flags |= O_APPEND;

	*may_flags = ksmbd_openflags_to_mayflags(posix_flags);

	return posix_flags;
}

/**
 * smb_get_disposition() - convert smb disposition flags to open flags
 * @flags:		smb file disposition flags
 * @file_present:	file already present or not
 * @stat:		file stat information
 * @open_flags:		open flags should be stored here
 *
 * Return:		file disposition flags
 */
static int smb_get_disposition(unsigned int flags, bool file_present,
		struct kstat *stat, unsigned int *open_flags)
{
	int dispostion, disp_flags;

	if ((flags & (SMB_O_CREAT | SMB_O_EXCL)) == (SMB_O_CREAT | SMB_O_EXCL))
		dispostion = FILE_CREATE;
	else if ((flags & (SMB_O_CREAT | SMB_O_TRUNC)) ==
			(SMB_O_CREAT | SMB_O_TRUNC))
		dispostion = FILE_OVERWRITE_IF;
	else if ((flags & SMB_O_CREAT) == SMB_O_CREAT)
		dispostion = FILE_OPEN_IF;
	else if ((flags & SMB_O_TRUNC) == SMB_O_TRUNC)
		dispostion = FILE_OVERWRITE;
	else if ((flags & (SMB_O_CREAT | SMB_O_EXCL | SMB_O_TRUNC)) == 0)
		dispostion = FILE_OPEN;
	else
		dispostion = FILE_SUPERSEDE;

	disp_flags = file_create_dispostion_flags(dispostion, file_present);
	if (disp_flags < 0)
		return disp_flags;

	*open_flags |= disp_flags;
	return disp_flags;
}

/**
 * smb_posix_open() - handler for smb posix open
 * @work:	smb work containing posix open command
 *
 * Return:	0 on success, otherwise error
 */
static int smb_posix_open(struct ksmbd_work *work)
{
	struct smb_com_trans2_spi_req *pSMB_req = work->request_buf;
	struct smb_com_trans2_spi_rsp *pSMB_rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct open_psx_req *psx_req;
	struct open_psx_rsp *psx_rsp;
	struct file_unix_basic_info *unix_info;
	struct path path;
	struct kstat stat;
	__u16 data_offset, rsp_info_level, file_info = 0;
	__u32 oplock_flags, posix_open_flags, may_flags;
	umode_t mode;
	char *name;
	bool file_present = true;
	int err;
	struct ksmbd_file *fp = NULL;
	int oplock_rsp = OPLOCK_NONE;

	name = smb_get_name(share, pSMB_req->FileName, PATH_MAX, work, false);
	if (IS_ERR(name)) {
		pSMB_rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(name);
	}

	if (ksmbd_override_fsids(work)) {
		pSMB_rsp->hdr.Status.CifsError = STATUS_NO_MEMORY;
		kfree(name);
		return -ENOMEM;
	}

	err = ksmbd_vfs_kern_path(work, name, LOOKUP_NO_SYMLINKS, &path, 0);
	if (err) {
		file_present = false;
		ksmbd_debug(SMB, "cannot get linux path for %s, err = %d\n",
				name, err);
		if (err == -EACCES || err == -EXDEV)
			goto out;
	} else {
		if (d_is_symlink(path.dentry)) {
			err = -EACCES;
			goto free_path;
		}
		err = vfs_getattr(&path, &stat, STATX_BASIC_STATS,
			AT_STATX_SYNC_AS_STAT);
		if (err) {
			pr_err("can not stat %s, err = %d\n", name, err);
			goto free_path;
		}
	}

	data_offset = le16_to_cpu(pSMB_req->DataOffset);
	psx_req = (struct open_psx_req *)(((char *)&pSMB_req->hdr.Protocol) +
			data_offset);
	oplock_flags = le32_to_cpu(psx_req->OpenFlags);

	posix_open_flags = smb_posix_convert_flags(
			le32_to_cpu(psx_req->PosixOpenFlags),
			&may_flags);
	err = smb_get_disposition(le32_to_cpu(psx_req->PosixOpenFlags),
			file_present, &stat,
			&posix_open_flags);
	if (err < 0) {
		ksmbd_debug(SMB, "create_dispostion returned %d\n", err);
		if (file_present)
			goto free_path;
		else
			goto out;
	}

	ksmbd_debug(SMB, "filename : %s, posix_open_flags : %x\n", name,
		posix_open_flags);
	mode = (umode_t) le64_to_cpu(psx_req->Permissions);
	rsp_info_level = le16_to_cpu(psx_req->Level);

	if (!test_tree_conn_flag(work->tcon, KSMBD_TREE_CONN_FLAG_WRITABLE)) {
		if (posix_open_flags & O_CREAT) {
			err = -EACCES;
			ksmbd_debug(SMB,
				"returning as user does not have permission to write\n");
			if (file_present)
				goto free_path;
			else
				goto out;
		}
	}

	/* posix mkdir command */
	if (posix_open_flags == (O_DIRECTORY | O_CREAT)) {
		if (file_present) {
			err = -EEXIST;
			goto free_path;
		}

		err = ksmbd_vfs_mkdir(work, name, mode);
		if (err)
			goto out;

		err = ksmbd_vfs_kern_path(work, name, 0, &path, 0);
		if (err) {
			pr_err("cannot get linux path, err = %d\n", err);
			goto out;
		}
		ksmbd_debug(SMB, "mkdir done for %s, inode %lu\n",
				name, d_inode(path.dentry)->i_ino);
		goto prepare_rsp;
	}

	if (!file_present && (posix_open_flags & O_CREAT)) {
		err = ksmbd_vfs_create(work, name, mode);
		if (err)
			goto out;

		err = ksmbd_vfs_kern_path(work, name, 0, &path, 0);
		if (err) {
			pr_err("cannot get linux path, err = %d\n", err);
			goto out;
		}
	} else if (file_present) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
		err = inode_permission(mnt_user_ns(path.mnt),
				       d_inode(path.dentry),
				       may_flags);
#else
		err = inode_permission(d_inode(path.dentry),
				       may_flags);
#endif
		if (err)
			goto free_path;
	}

	fp = ksmbd_vfs_dentry_open(work, &path, posix_open_flags,
			0, file_present);
	if (IS_ERR(fp)) {
		err = PTR_ERR(fp);
		fp = NULL;
		goto free_path;
	}
	fp->pid = le16_to_cpu(pSMB_req->hdr.Pid);

	write_lock(&fp->f_ci->m_lock);
	list_add(&fp->node, &fp->f_ci->m_fp_list);
	write_unlock(&fp->f_ci->m_lock);

	if (smb1_oplock_enable &&
	    test_share_config_flag(work->tcon->share_conf,
			KSMBD_SHARE_FLAG_OPLOCKS) &&
		!S_ISDIR(file_inode(fp->filp)->i_mode)) {
		/* Client cannot request levelII oplock directly */
		err = smb_grant_oplock(work, oplock_flags &
			(REQ_OPLOCK | REQ_BATCHOPLOCK), fp->volatile_id, fp,
			le16_to_cpu(pSMB_req->hdr.Tid), NULL, 0);
		if (err)
			goto free_path;
	}

	oplock_rsp = fp->f_opinfo != NULL ? fp->f_opinfo->level : 0;

prepare_rsp:
	/* open/mkdir success, send back response */
	data_offset = sizeof(struct smb_com_trans2_spi_rsp) -
		sizeof(pSMB_rsp->hdr.smb_buf_length) +
		3 /*alignment*/;
	psx_rsp = (struct open_psx_rsp *)(((char *)&pSMB_rsp->hdr.Protocol) +
			data_offset);
	if (data_offset + sizeof(struct open_psx_rsp) > work->response_sz) {
		err = -EIO;
		goto free_path;
	}

	psx_rsp->OplockFlags = cpu_to_le16(oplock_rsp);
	psx_rsp->Fid = fp != NULL ? fp->volatile_id : 0;

	if (file_present) {
		if (!(posix_open_flags & O_TRUNC))
			file_info = F_OPENED;
		else
			file_info = F_OVERWRITTEN;
	} else
		file_info = F_CREATED;
	psx_rsp->CreateAction = cpu_to_le32(file_info);

	if (rsp_info_level != SMB_QUERY_FILE_UNIX_BASIC) {
		ksmbd_debug(SMB, "returning null information level response");
		rsp_info_level = SMB_NO_INFO_LEVEL_RESPONSE;
	}
	psx_rsp->ReturnedLevel = cpu_to_le16(rsp_info_level);

	err = vfs_getattr(&path, &stat, STATX_BASIC_STATS,
		AT_STATX_SYNC_AS_STAT);
	if (err) {
		pr_err("cannot get stat information\n");
		goto free_path;
	}

	pSMB_rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	unix_info = (struct file_unix_basic_info *)((char *)psx_rsp +
			sizeof(struct open_psx_rsp));
	init_unix_info(unix_info, mnt_user_ns(path.mnt), &stat);

	pSMB_rsp->hdr.WordCount = 10;
	pSMB_rsp->t2.TotalParameterCount = cpu_to_le16(2);
	pSMB_rsp->t2.TotalDataCount = cpu_to_le16(sizeof(struct open_psx_rsp) +
			sizeof(struct file_unix_basic_info));
	pSMB_rsp->t2.ParameterCount = pSMB_rsp->t2.TotalParameterCount;
	pSMB_rsp->t2.Reserved = 0;
	pSMB_rsp->t2.ParameterCount = cpu_to_le16(2);
	pSMB_rsp->t2.ParameterOffset = cpu_to_le16(56);
	pSMB_rsp->t2.ParameterDisplacement = 0;
	pSMB_rsp->t2.DataCount = pSMB_rsp->t2.TotalDataCount;
	pSMB_rsp->t2.DataOffset = cpu_to_le16(data_offset);
	pSMB_rsp->t2.DataDisplacement = 0;
	pSMB_rsp->t2.SetupCount = 0;
	pSMB_rsp->t2.Reserved1 = 0;

	/* 2 for parameter count + 112 data count + 3 pad (1 pad1 + 2 pad2)*/
	pSMB_rsp->ByteCount = cpu_to_le16(117);
	pSMB_rsp->Reserved2 = 0;
	inc_rfc1001_len(&pSMB_rsp->hdr,
			(pSMB_rsp->hdr.WordCount * 2 + 117));

free_path:
	path_put(&path);
out:
	switch (err) {
	case 0:
		break;
	case -ENOSPC:
		pSMB_rsp->hdr.Status.CifsError = STATUS_DISK_FULL;
		break;
	case -EINVAL:
		pSMB_rsp->hdr.Status.CifsError = STATUS_NO_SUCH_USER;
		break;
	case -EACCES:
		pSMB_rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
		break;
	case -ENOENT:
		pSMB_rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_NOT_FOUND;
		break;
	case -EBUSY:
		pSMB_rsp->hdr.Status.CifsError = STATUS_DELETE_PENDING;
		break;
	default:
		pSMB_rsp->hdr.Status.CifsError =
			STATUS_UNEXPECTED_IO_ERROR;
	}

	if (err && fp)
		ksmbd_close_fd(work, fp->volatile_id);
	kfree(name);
	ksmbd_revert_fsids(work);
	return err;
}

/**
 * smb_posix_unlink() - handler for posix file delete
 * @work:	smb work containing trans2 posix delete command
 *
 * Return:	0 on success, otherwise error
 */
static int smb_posix_unlink(struct ksmbd_work *work)
{
	struct smb_com_trans2_spi_req *req = work->request_buf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	struct unlink_psx_rsp *psx_rsp = NULL;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	char *name;
	int rc = 0;

	if (!test_tree_conn_flag(work->tcon, KSMBD_TREE_CONN_FLAG_WRITABLE)) {
		ksmbd_debug(SMB,
			"returning as user does not have permission to write\n");
		rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
		return -EACCES;
	}

	name = smb_get_name(share, req->FileName, PATH_MAX, work, false);
	if (IS_ERR(name)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(name);
	}

	rc = ksmbd_vfs_remove_file(work, name);
	if (rc < 0)
		goto out;

	psx_rsp = (struct unlink_psx_rsp *)((char *)rsp +
			sizeof(struct smb_com_trans2_rsp));
	psx_rsp->EAErrorOffset = cpu_to_le16(0);
	rsp->hdr.Status.CifsError = STATUS_SUCCESS;

	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = cpu_to_le16(0);
	rsp->t2.ParameterCount = rsp->t2.TotalParameterCount;
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = cpu_to_le16(2);
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = rsp->t2.TotalDataCount;
	rsp->t2.DataOffset = cpu_to_le16(0);
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;

	/* 2 for parameter count + 1 pad1*/
	rsp->ByteCount = cpu_to_le16(3);
	rsp->Pad = 0;
	inc_rfc1001_len(&rsp->hdr, rsp->hdr.WordCount * 2 + 3);

out:
	if (rc)
		rsp->hdr.Status.CifsError = STATUS_UNEXPECTED_IO_ERROR;

	kfree(name);
	return rc;
}

/**
 * smb_set_time_pathinfo() - handler for setting time using set path info
 * @work:	smb work containing set path info command
 *
 * Return:	0 on success, otherwise error
 */
static int smb_set_time_pathinfo(struct ksmbd_work *work)
{
	struct smb_com_trans2_spi_req *req = work->request_buf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	struct file_basic_info *info;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct iattr attrs;
	char *name;
	int err = 0;

	name = smb_get_name(share, req->FileName, PATH_MAX, work, false);
	if (IS_ERR(name)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(name);
	}

	info = (struct file_basic_info *)(((char *) &req->hdr.Protocol) +
			le16_to_cpu(req->DataOffset));

	attrs.ia_valid = 0;
	if (le64_to_cpu(info->LastAccessTime)) {
		attrs.ia_atime = smb_NTtimeToUnix(info->LastAccessTime);
		attrs.ia_valid |= (ATTR_ATIME | ATTR_ATIME_SET);
	}

	if (le64_to_cpu(info->ChangeTime)) {
		attrs.ia_ctime = smb_NTtimeToUnix(info->ChangeTime);
		attrs.ia_valid |= ATTR_CTIME;
	}

	if (le64_to_cpu(info->LastWriteTime)) {
		attrs.ia_mtime = smb_NTtimeToUnix(info->LastWriteTime);
		attrs.ia_valid |= (ATTR_MTIME | ATTR_MTIME_SET);
	}
	/* TODO: check dos mode and acl bits if req->Attributes nonzero */

	if (!attrs.ia_valid)
		goto done;

	err = ksmbd_vfs_setattr(work, name, 0, &attrs);
	if (err) {
		rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
		return err;
	}

done:
	ksmbd_debug(SMB, "%s setattr done\n", name);
	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = 0;
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = rsp->t2.TotalParameterCount;
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = rsp->t2.TotalDataCount;
	rsp->t2.DataOffset = 0;
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;

	/* 3 pad (1 pad1 + 2 pad2)*/
	rsp->ByteCount = cpu_to_le16(3);
	inc_rfc1001_len(&rsp->hdr, rsp->hdr.WordCount * 2 + 3);

	kfree(name);
	return 0;
}

/**
 * smb_set_unix_pathinfo() - handler for setting unix path info(setattr)
 * @work:	smb work containing set path info command
 *
 * Return:	0 on success, otherwise error
 */
static int smb_set_unix_pathinfo(struct ksmbd_work *work)
{
	struct smb_com_trans2_spi_req *req = work->request_buf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	struct file_unix_basic_info *unix_info;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct path path;
	struct iattr attrs;
	char *name;
	int err = 0;

	name = smb_get_name(share, req->FileName, PATH_MAX, work, false);
	if (IS_ERR(name)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(name);
	}

	if (ksmbd_override_fsids(work))
		return -ENOMEM;
	err = kern_path(name, 0, &path);
	if (err) {
		ksmbd_revert_fsids(work);
		kfree(name);
		return -ENOENT;
	}

	unix_info =  (struct file_unix_basic_info *)
		(((char *) &req->hdr.Protocol) + le16_to_cpu(req->DataOffset));
	attrs.ia_valid = 0;
	attrs.ia_mode = 0;
	err = unix_info_to_attr(unix_info, mnt_user_ns(path.mnt), &attrs);
	path_put(&path);
	ksmbd_revert_fsids(work);
	if (err)
		goto out;

	err = ksmbd_vfs_setattr(work, name, 0, &attrs);
	if (err)
		goto out;
	/* setattr success, prepare response */
	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = 0;
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = rsp->t2.TotalParameterCount;
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = rsp->t2.TotalDataCount;
	rsp->t2.DataOffset = 0;
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;

	/* 3 pad (1 pad1 + 2 pad2)*/
	rsp->ByteCount = cpu_to_le16(3);
	inc_rfc1001_len(&rsp->hdr, rsp->hdr.WordCount * 2 + 3);

out:
	kfree(name);
	if (err) {
		rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
		return err;
	}
	return 0;
}

/**
 * smb_set_ea() - handler for setting extended attributes using set path
 *		info command
 * @work:	smb work containing set path info command
 *
 * Return:	0 on success, otherwise error
 */
static int smb_set_ea(struct ksmbd_work *work)
{
	struct smb_com_trans2_spi_req *req = work->request_buf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct fealist *eabuf;
	struct fea *ea;
	char *fname, *attr_name = NULL, *value;
	int rc = 0, list_len, i, next = 0;

	fname = smb_get_name(share, req->FileName, PATH_MAX, work, false);
	if (IS_ERR(fname)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(fname);
	}

	eabuf = (struct fealist *)(((char *) &req->hdr.Protocol)
			+ le16_to_cpu(req->DataOffset));

	list_len = le32_to_cpu(eabuf->list_len) - 4;
	ea = (struct fea *)eabuf->list;

	for (i = 0; list_len >= 0 && ea->name_len != 0; i++, list_len -= next) {
		if (ea->name_len > (XATTR_NAME_MAX - XATTR_USER_PREFIX_LEN)) {
			rc = -EINVAL;
			goto out;
		}

		next = ea->name_len + le16_to_cpu(ea->value_len) + 4;

		attr_name = kmalloc(XATTR_NAME_MAX + 1, GFP_KERNEL);
		if (!attr_name) {
			rc = -ENOMEM;
			goto out;
		}

		memcpy(attr_name, XATTR_USER_PREFIX, XATTR_USER_PREFIX_LEN);
		memcpy(&attr_name[XATTR_USER_PREFIX_LEN], ea->name,
				ea->name_len);
		attr_name[XATTR_USER_PREFIX_LEN + ea->name_len] = '\0';
		value = (char *)&ea->name + ea->name_len + 1;
		ksmbd_debug(SMB, "name: <%s>, name_len %u, value_len %u\n",
			ea->name, ea->name_len, le16_to_cpu(ea->value_len));

		rc = ksmbd_vfs_fsetxattr(work, fname, attr_name, value,
					le16_to_cpu(ea->value_len),
					0);
		if (rc < 0) {
			kfree(attr_name);
			rsp->hdr.Status.CifsError =
				STATUS_UNEXPECTED_IO_ERROR;
			goto out;
		}
		kfree(attr_name);
		ea += next;
	}

	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = cpu_to_le16(0);
	rsp->t2.ParameterCount = rsp->t2.TotalParameterCount;
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = cpu_to_le16(2);
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = rsp->t2.TotalDataCount;
	rsp->t2.DataOffset = cpu_to_le16(0);
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;

	/* 2 for parameter count + 1 pad1*/
	rsp->ByteCount = cpu_to_le16(3);
	rsp->Pad = 0;
	inc_rfc1001_len(&rsp->hdr, rsp->hdr.WordCount * 2 + 3);

out:
	kfree(fname);
	return rc;
}

/**
 * smb_set_file_size_pinfo() - handler for setting eof or truncate using
 *		trans2 set path info command
 * @work:	smb work containing set path info command
 *
 * Return:	0 on success, otherwise error
 */
static int smb_set_file_size_pinfo(struct ksmbd_work *work)
{
	struct smb_com_trans2_spi_req *req = work->request_buf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct file_end_of_file_info *eofinfo;
	struct iattr attr;
	char *name = NULL;
	loff_t newsize;
	int rc = 0;

	name = smb_get_name(share, req->FileName, PATH_MAX, work, false);
	if (IS_ERR(name)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(name);
	}

	eofinfo =  (struct file_end_of_file_info *)
		(((char *) &req->hdr.Protocol) + le16_to_cpu(req->DataOffset));
	newsize = le64_to_cpu(eofinfo->FileSize);
	attr.ia_valid = ATTR_SIZE;
	attr.ia_size = newsize;
	rc = ksmbd_vfs_setattr(work, name, 0, &attr);
	if (rc) {
		rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
		goto out;
	}
	ksmbd_debug(SMB, "%s truncated to newsize %lld\n",
			name, newsize);
	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = 0;
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = rsp->t2.TotalParameterCount;
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = rsp->t2.TotalDataCount;
	rsp->t2.DataOffset = 0;
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;

	/* 2 for parameter count + 1 pad1*/
	rsp->ByteCount = cpu_to_le16(3);
	inc_rfc1001_len(&rsp->hdr, rsp->hdr.WordCount * 2 + 3);

out:
	kfree(name);
	return rc;
}

/**
 * smb_creat_hardlink() - handler for creating hardlink
 * @work:	smb work containing set path info command buffer
 *
 * Return:	0 on success, otherwise error
 */
static int smb_creat_hardlink(struct ksmbd_work *work)
{
	struct smb_com_trans2_spi_req *req = work->request_buf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	char *oldname, *newname, *oldname_offset;
	int err;

	newname = smb_get_name(share, req->FileName, PATH_MAX, work, false);
	if (IS_ERR(newname)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(newname);
	}

	oldname_offset = ((char *)&req->hdr.Protocol) +
				le16_to_cpu(req->DataOffset);
	oldname = smb_get_name(share, oldname_offset, PATH_MAX, work, false);
	if (IS_ERR(oldname)) {
		err = PTR_ERR(oldname);
		oldname = NULL;
		goto out;
	}
	ksmbd_debug(SMB, "oldname %s, newname %s\n", oldname, newname);

	err = ksmbd_vfs_link(work, oldname, newname);
	if (err < 0) {
		if (err == -EACCES)
			rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
		else
			rsp->hdr.Status.CifsError = STATUS_NOT_SAME_DEVICE;
		goto out;
	}

	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = 0;
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = rsp->t2.TotalParameterCount;
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = 0;
	rsp->t2.DataOffset = 0;
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;
	rsp->ByteCount = cpu_to_le16(3);
	inc_rfc1001_len(&rsp->hdr, rsp->hdr.WordCount * 2 + 3);
out:
	kfree(newname);
	kfree(oldname);
	return err;
}

/**
 * smb_creat_symlink() - handler for creating symlink
 * @work:	smb work containing set path info command buffer
 *
 * Return:	0 on success, otherwise error
 */
static int smb_creat_symlink(struct ksmbd_work *work)
{
	struct smb_com_trans2_spi_req *req = work->request_buf;
	struct smb_com_trans2_spi_rsp *rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	char *name, *symname, *name_offset;
	bool is_unicode = is_smbreq_unicode(&req->hdr);
	int err;

	symname = smb_get_name(share, req->FileName, PATH_MAX, work, false);
	if (IS_ERR(symname)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(symname);
	}

	name_offset = ((char *)&req->hdr.Protocol) +
		le16_to_cpu(req->DataOffset);
	name = smb_strndup_from_utf16(name_offset, PATH_MAX, is_unicode,
			work->conn->local_nls);
	if (IS_ERR(name)) {
		kfree(symname);
		rsp->hdr.Status.CifsError = STATUS_NO_MEMORY;
		return PTR_ERR(name);
	}
	ksmbd_debug(SMB, "name %s, symname %s\n", name, symname);

	err = ksmbd_vfs_symlink(work, name, symname);
	if (err < 0) {
		if (err == -ENOSPC)
			rsp->hdr.Status.CifsError = STATUS_DISK_FULL;
		else if (err == -EEXIST)
			rsp->hdr.Status.CifsError =
				STATUS_OBJECT_NAME_COLLISION;
		else
			rsp->hdr.Status.CifsError = STATUS_NOT_SAME_DEVICE;
	} else
		rsp->hdr.Status.CifsError = STATUS_SUCCESS;

	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = 0;
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = rsp->t2.TotalParameterCount;
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = 0;
	rsp->t2.DataOffset = 0;
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;
	rsp->ByteCount = cpu_to_le16(3);
	inc_rfc1001_len(&rsp->hdr, rsp->hdr.WordCount * 2 + 3);
	kfree(name);
	kfree(symname);
	return err;
}

/**
 * set_path_info() - handler for trans2 set path info sub commands
 * @work:	smb work containing set path info command
 *
 * Return:	0 on success, otherwise error
 */
static int set_path_info(struct ksmbd_work *work)
{
	struct smb_com_trans2_spi_req *pSMB_req = work->request_buf;
	struct smb_com_trans2_spi_rsp  *pSMB_rsp = work->response_buf;
	__u16 info_level, total_param;
	int err = 0;

	info_level = le16_to_cpu(pSMB_req->InformationLevel);
	total_param = le16_to_cpu(pSMB_req->TotalParameterCount);
	if (total_param < 7) {
		pSMB_rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
		pr_err("invalid total parameter for info_level 0x%x\n",
		       total_param);
		return -EINVAL;
	}

	switch (info_level) {
	case SMB_POSIX_OPEN:
		err = smb_posix_open(work);
		break;
	case SMB_POSIX_UNLINK:
		err = smb_posix_unlink(work);
		break;
	case SMB_SET_FILE_UNIX_HLINK:
		err = smb_creat_hardlink(work);
		break;
	case SMB_SET_FILE_UNIX_LINK:
		err = smb_creat_symlink(work);
		break;
	case SMB_SET_FILE_BASIC_INFO:
		/* fall through */
	case SMB_SET_FILE_BASIC_INFO2:
		err = smb_set_time_pathinfo(work);
		break;
	case SMB_SET_FILE_UNIX_BASIC:
		err = smb_set_unix_pathinfo(work);
		break;
	case SMB_SET_FILE_EA:
		err = smb_set_ea(work);
		break;
	case SMB_SET_POSIX_ACL:
		err = smb_set_acl(work);
		break;
	case SMB_SET_FILE_END_OF_FILE_INFO2:
		/* fall through */
	case SMB_SET_FILE_END_OF_FILE_INFO:
		err = smb_set_file_size_pinfo(work);
		break;
	default:
		ksmbd_debug(SMB, "info level = %x not implemented yet\n",
				info_level);
		pSMB_rsp->hdr.Status.CifsError = STATUS_NOT_IMPLEMENTED;
		return -EOPNOTSUPP;
	}

	if (err < 0)
		ksmbd_debug(SMB, "info_level 0x%x failed, err %d\n",
				info_level, err);
	return err;
}
static int readdir_info_level_struct_sz(int info_level)
{
	switch (info_level) {
	case SMB_FIND_FILE_INFO_STANDARD:
		return sizeof(struct find_info_standard);
	case SMB_FIND_FILE_QUERY_EA_SIZE:
		return sizeof(struct find_info_query_ea_size);
	case SMB_FIND_FILE_DIRECTORY_INFO:
		return sizeof(struct file_directory_info);
	case SMB_FIND_FILE_FULL_DIRECTORY_INFO:
		return sizeof(struct file_full_directory_info);
	case SMB_FIND_FILE_NAMES_INFO:
		return sizeof(struct file_names_info);
	case SMB_FIND_FILE_BOTH_DIRECTORY_INFO:
		return sizeof(struct file_both_directory_info);
	case SMB_FIND_FILE_ID_FULL_DIR_INFO:
		return sizeof(struct file_id_full_dir_info);
	case SMB_FIND_FILE_ID_BOTH_DIR_INFO:
		return sizeof(struct file_id_both_directory_info);
	case SMB_FIND_FILE_UNIX:
		return sizeof(struct file_unix_info);
	default:
		return -EOPNOTSUPP;
	}
}

/**
 * smb_populate_readdir_entry() - encode directory entry in smb response buffer
 * @conn:	connection instance
 * @info_level:	smb information level
 * @d_info: structure included variables for query dir
 * @ksmbd_kstat: ksmbd wrapper of dirent stat information
 *
 * if directory has many entries, find first can't read it fully.
 * find next might be called multiple times to read remaining dir entries
 *
 * Return:	0 on success, otherwise error
 */
static int smb_populate_readdir_entry(struct ksmbd_conn *conn, int info_level,
		struct ksmbd_dir_info *d_info, struct ksmbd_kstat *ksmbd_kstat)
{
	int next_entry_offset;
	char *conv_name;
	int conv_len;
	int struct_sz;

	struct_sz = readdir_info_level_struct_sz(info_level);
	if (struct_sz == -EOPNOTSUPP)
		return -EOPNOTSUPP;

	conv_name = ksmbd_convert_dir_info_name(d_info,
						conn->local_nls,
						&conv_len);
	if (!conv_name)
		return -ENOMEM;

	next_entry_offset = ALIGN(struct_sz - 1 + conv_len,
				  KSMBD_DIR_INFO_ALIGNMENT);

	if (next_entry_offset > d_info->out_buf_len) {
		kfree(conv_name);
		d_info->out_buf_len = -1;
		return -ENOSPC;
	}

	switch (info_level) {
	case SMB_FIND_FILE_INFO_STANDARD:
	{
		struct find_info_standard *fsinfo;

		fsinfo = (struct find_info_standard *)(d_info->wptr);
		unix_to_dos_time(
			ksmbd_NTtimeToUnix(
				cpu_to_le64(ksmbd_kstat->create_time)),
			&fsinfo->CreationTime,
			&fsinfo->CreationDate);
		unix_to_dos_time(ksmbd_kstat->kstat->atime,
			&fsinfo->LastAccessTime,
			&fsinfo->LastAccessDate);
		unix_to_dos_time(ksmbd_kstat->kstat->mtime,
			&fsinfo->LastWriteTime,
			&fsinfo->LastWriteDate);
		fsinfo->DataSize = cpu_to_le32(ksmbd_kstat->kstat->size);
		fsinfo->AllocationSize =
			cpu_to_le32(ksmbd_kstat->kstat->blocks << 9);
		fsinfo->Attributes =
			cpu_to_le16(S_ISDIR(ksmbd_kstat->kstat->mode) ?
				ATTR_DIRECTORY : ATTR_ARCHIVE);
		fsinfo->FileNameLength = cpu_to_le16(conv_len);
		memcpy(fsinfo->FileName, conv_name, conv_len);

		break;
	}
	case SMB_FIND_FILE_QUERY_EA_SIZE:
	{
		struct find_info_query_ea_size *fesize;

		fesize = (struct find_info_query_ea_size *)(d_info->wptr);
		unix_to_dos_time(
			ksmbd_NTtimeToUnix(
				cpu_to_le64(ksmbd_kstat->create_time)),
			&fesize->CreationTime,
			&fesize->CreationDate);
		unix_to_dos_time(ksmbd_kstat->kstat->atime,
			&fesize->LastAccessTime,
			&fesize->LastAccessDate);
		unix_to_dos_time(ksmbd_kstat->kstat->mtime,
			&fesize->LastWriteTime,
			&fesize->LastWriteDate);

		fesize->DataSize =
			cpu_to_le32(ksmbd_kstat->kstat->size);
		fesize->AllocationSize =
			cpu_to_le32(ksmbd_kstat->kstat->blocks << 9);
		fesize->Attributes =
			cpu_to_le16(S_ISDIR(ksmbd_kstat->kstat->mode) ?
				ATTR_DIRECTORY : ATTR_ARCHIVE);
		fesize->EASize = 0;
		fesize->FileNameLength = (__u8)(conv_len);
		memcpy(fesize->FileName, conv_name, conv_len);

		break;
	}
	case SMB_FIND_FILE_DIRECTORY_INFO:
	{
		struct file_directory_info *fdinfo = NULL;

		fdinfo = (struct file_directory_info *)
			ksmbd_vfs_init_kstat(&d_info->wptr, ksmbd_kstat);
		fdinfo->FileNameLength = cpu_to_le32(conv_len);
		memcpy(fdinfo->FileName, conv_name, conv_len);
		fdinfo->NextEntryOffset = cpu_to_le32(next_entry_offset);
		memset((char *)fdinfo + struct_sz - 1 + conv_len,
			'\0',
			next_entry_offset - struct_sz - 1 + conv_len);
		break;
	}
	case SMB_FIND_FILE_FULL_DIRECTORY_INFO:
	{
		struct file_full_directory_info *ffdinfo = NULL;

		ffdinfo = (struct file_full_directory_info *)
			ksmbd_vfs_init_kstat(&d_info->wptr, ksmbd_kstat);
		ffdinfo->FileNameLength = cpu_to_le32(conv_len);
		ffdinfo->EaSize = 0;
		memcpy(ffdinfo->FileName, conv_name, conv_len);
		ffdinfo->NextEntryOffset = cpu_to_le32(next_entry_offset);
		memset((char *)ffdinfo + struct_sz - 1 + conv_len,
			'\0',
			next_entry_offset - struct_sz - 1 + conv_len);
		break;
	}
	case SMB_FIND_FILE_NAMES_INFO:
	{
		struct file_names_info *fninfo = NULL;

		fninfo = (struct file_names_info *)(d_info->wptr);
		fninfo->FileNameLength = cpu_to_le32(conv_len);
		memcpy(fninfo->FileName, conv_name, conv_len);
		fninfo->NextEntryOffset = cpu_to_le32(next_entry_offset);
		memset((char *)fninfo + struct_sz - 1 + conv_len,
			'\0',
			next_entry_offset - struct_sz - 1 + conv_len);

		break;
	}
	case SMB_FIND_FILE_BOTH_DIRECTORY_INFO:
	{
		struct file_both_directory_info *fbdinfo = NULL;

		fbdinfo = (struct file_both_directory_info *)
			ksmbd_vfs_init_kstat(&d_info->wptr, ksmbd_kstat);
		fbdinfo->FileNameLength = cpu_to_le32(conv_len);
		fbdinfo->EaSize = 0;
		fbdinfo->ShortNameLength = 0;
		fbdinfo->Reserved = 0;
		memcpy(fbdinfo->FileName, conv_name, conv_len);
		fbdinfo->NextEntryOffset = cpu_to_le32(next_entry_offset);
		memset((char *)fbdinfo + struct_sz - 1 + conv_len,
			'\0',
			next_entry_offset - struct_sz - 1 + conv_len);
		break;
	}
	case SMB_FIND_FILE_ID_FULL_DIR_INFO:
	{
		struct file_id_full_dir_info *dinfo = NULL;

		dinfo = (struct file_id_full_dir_info *)
			ksmbd_vfs_init_kstat(&d_info->wptr, ksmbd_kstat);
		dinfo->FileNameLength = cpu_to_le32(conv_len);
		dinfo->EaSize = 0;
		dinfo->Reserved = 0;
		dinfo->UniqueId = cpu_to_le64(ksmbd_kstat->kstat->ino);
		memcpy(dinfo->FileName, conv_name, conv_len);
		dinfo->NextEntryOffset = cpu_to_le32(next_entry_offset);
		memset((char *)dinfo + struct_sz - 1 + conv_len,
			'\0',
			next_entry_offset - struct_sz - 1 + conv_len);
		break;
	}
	case SMB_FIND_FILE_ID_BOTH_DIR_INFO:
	{
		struct file_id_both_directory_info *fibdinfo = NULL;

		fibdinfo = (struct file_id_both_directory_info *)
			ksmbd_vfs_init_kstat(&d_info->wptr, ksmbd_kstat);
		fibdinfo->FileNameLength = cpu_to_le32(conv_len);
		fibdinfo->EaSize = 0;
		fibdinfo->ShortNameLength = 0;
		fibdinfo->Reserved = 0;
		fibdinfo->Reserved2 = 0;
		fibdinfo->UniqueId = cpu_to_le64(ksmbd_kstat->kstat->ino);
		memcpy(fibdinfo->FileName, conv_name, conv_len);
		fibdinfo->NextEntryOffset = cpu_to_le32(next_entry_offset);
		memset((char *)fibdinfo + struct_sz - 1 + conv_len,
			'\0',
			next_entry_offset - struct_sz - 1 + conv_len);

		break;
	}
	case SMB_FIND_FILE_UNIX:
	{
		struct file_unix_info *finfo = NULL;
		struct file_unix_basic_info *unix_info;

		finfo = (struct file_unix_info *)(d_info->wptr);
		finfo->ResumeKey = 0;
		unix_info = (struct file_unix_basic_info *)((char *)finfo + 8);
		init_unix_info(unix_info, &init_user_ns, ksmbd_kstat->kstat);
		/* include null terminator */
		memcpy(finfo->FileName, conv_name, conv_len + 2);
		next_entry_offset += 2;
		finfo->NextEntryOffset = cpu_to_le32(next_entry_offset);
		memset((char *)finfo + struct_sz - 1 + conv_len,
			'\0',
			next_entry_offset - struct_sz - 1 + conv_len);
		break;
	}
	}

	d_info->num_entry++;
	d_info->last_entry_offset = d_info->data_count;
	d_info->data_count += next_entry_offset;
	d_info->out_buf_len -= next_entry_offset;
	d_info->wptr = (char *)(d_info->wptr) + next_entry_offset;
	kfree(conv_name);

	ksmbd_debug(SMB, "info_level : %d, buf_len :%d, next_offset : %d, data_count : %d\n",
			info_level, d_info->out_buf_len,
			next_entry_offset, d_info->data_count);
	return 0;
}

/**
 * ksmbd_fill_dirent() - populates a dirent details in readdir
 * @ctx:	dir_context information
 * @name:	dirent name
 * @namelen:	dirent name length
 * @offset:	dirent offset in directory
 * @ino:	dirent inode number
 * @d_type:	dirent type
 *
 * Return:	0 on success, otherwise -EINVAL
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static bool ksmbd_fill_dirent(struct dir_context *ctx, const char *name, int namlen,
#else
static int ksmbd_fill_dirent(struct dir_context *ctx, const char *name, int namlen,
#endif
		loff_t offset, u64 ino, unsigned int d_type)
{
	struct ksmbd_readdir_data *buf =
		container_of(ctx, struct ksmbd_readdir_data, ctx);
	struct ksmbd_dirent *de = (void *)(buf->dirent + buf->used);
	unsigned int reclen;

	reclen = ALIGN(sizeof(struct ksmbd_dirent) + namlen, sizeof(u64));
	if (buf->used + reclen > PAGE_SIZE)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
		return false;
#else
		return -EINVAL;
#endif

	de->namelen = namlen;
	de->offset = offset;
	de->ino = ino;
	de->d_type = d_type;
	memcpy(de->name, name, namlen);
	buf->used += reclen;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
	return true;
#else
	return 0;
#endif
}

/**
 * find_first() - smb readdir command
 * @work:	smb work containing find first request params
 *
 * Return:	0 on success, otherwise error
 */
static int find_first(struct ksmbd_work *work)
{
	struct smb_hdr *rsp_hdr = work->response_buf;
	struct ksmbd_conn *conn = work->conn;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct smb_com_trans2_req *req = work->request_buf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	struct smb_com_trans2_ffirst_req_params *req_params;
	struct smb_com_trans2_ffirst_rsp_parms *params = NULL;
	struct path path;
	struct ksmbd_dirent *de;
	struct ksmbd_file *dir_fp = NULL;
	struct kstat kstat;
	struct ksmbd_kstat ksmbd_kstat;
	struct ksmbd_dir_info d_info;
	int params_count = sizeof(struct smb_com_trans2_ffirst_rsp_parms);
	int data_alignment_offset = 0;
	int rc = 0, reclen = 0;
	int srch_cnt = 0;
	char *dirpath = NULL;
	char *srch_ptr = NULL;
	int header_size;
	int struct_sz;

	memset(&d_info, 0, sizeof(struct ksmbd_dir_info));

	if (ksmbd_override_fsids(work)) {
		rsp->hdr.Status.CifsError = STATUS_NO_MEMORY;
		return -ENOMEM;
	}

	req_params = (struct smb_com_trans2_ffirst_req_params *)
		(work->request_buf + le16_to_cpu(req->ParameterOffset) + 4);
	dirpath = smb_get_dir_name(share, req_params->FileName, PATH_MAX,
			work, &srch_ptr);
	if (IS_ERR(dirpath)) {
		rsp->hdr.Status.CifsError = STATUS_NO_MEMORY;
		rc = PTR_ERR(dirpath);
		goto err_out;
	}

	ksmbd_debug(SMB, "complete dir path = %s\n",  dirpath);
	rc = ksmbd_vfs_kern_path(work, dirpath, LOOKUP_NO_SYMLINKS | LOOKUP_DIRECTORY,
				 &path, 0);
	if (rc < 0) {
		ksmbd_debug(SMB, "cannot create vfs root path <%s> %d\n",
				dirpath, rc);
		goto err_free_dirpath;
	} else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
		if (inode_permission(mnt_user_ns(path.mnt),
				     d_inode(path.dentry),
				     MAY_READ | MAY_EXEC)) {
#else
		if (inode_permission(d_inode(path.dentry),
					MAY_READ | MAY_EXEC)) {
#endif
			rc = -EACCES;
			path_put(&path);
			goto err_free_dirpath;
		}
	}

	if (d_is_symlink(path.dentry)) {
		rc = -EACCES;
		path_put(&path);
		goto err_free_dirpath;
	}

	dir_fp = ksmbd_vfs_dentry_open(work, &path, O_RDONLY, 0, 1);
	if (IS_ERR(dir_fp)) {
		ksmbd_debug(SMB, "dir dentry open failed with rc=%d\n", rc);
		path_put(&path);
		rc = -EINVAL;
		dir_fp = NULL;
		goto err_free_dirpath;
	}

	write_lock(&dir_fp->f_ci->m_lock);
	list_add(&dir_fp->node, &dir_fp->f_ci->m_fp_list);
	write_unlock(&dir_fp->f_ci->m_lock);

	set_ctx_actor(&dir_fp->readdir_data.ctx, ksmbd_fill_dirent);
	dir_fp->readdir_data.dirent = (void *)__get_free_page(GFP_KERNEL);
	if (!dir_fp->readdir_data.dirent) {
		rc = -ENOMEM;
		goto err_free_dirpath;
	}

	dir_fp->filename = dirpath;
	dir_fp->readdir_data.used = 0;
	dir_fp->dirent_offset = 0;
	dir_fp->readdir_data.file_attr =
		le16_to_cpu(req_params->SearchAttributes);

	if (params_count % 4)
		data_alignment_offset = 4 - params_count % 4;

	d_info.smb1_name = kmalloc(NAME_MAX + 1, GFP_KERNEL);
	if (!d_info.smb1_name) {
		rc = -ENOMEM;
		goto err_out;
	}
	d_info.wptr = (char *)((char *)rsp + sizeof(struct smb_com_trans2_rsp) +
			params_count + data_alignment_offset);

	header_size = sizeof(struct smb_com_trans2_rsp) + params_count +
		data_alignment_offset;


	struct_sz = readdir_info_level_struct_sz(le16_to_cpu(req_params->InformationLevel));

	if (struct_sz < 0) {
		rc = -EFAULT;
		goto err_out;
	}

	/* When search count is zero, respond only 1 entry. */
	srch_cnt = le16_to_cpu(req_params->SearchCount);
	if (!srch_cnt)
		d_info.out_buf_len = struct_sz + header_size;
	else
		d_info.out_buf_len = min_t(int, srch_cnt * struct_sz + header_size,
				MAX_CIFS_LOOKUP_BUFFER_SIZE - header_size);


	/* reserve dot and dotdot entries in head of buffer in first response */
	if (!*srch_ptr || is_asterisk(srch_ptr)) {
		rc = ksmbd_populate_dot_dotdot_entries(work,
				le16_to_cpu(req_params->InformationLevel),
				dir_fp,
				&d_info,
				srch_ptr,
				smb_populate_readdir_entry);
		if (rc)
			goto err_out;
	}

	do {
		if (dir_fp->dirent_offset >= dir_fp->readdir_data.used) {
			dir_fp->dirent_offset = 0;
			dir_fp->readdir_data.used = 0;
			rc = iterate_dir(dir_fp->filp, &dir_fp->readdir_data.ctx);
			if (rc < 0) {
				ksmbd_debug(SMB, "err : %d\n", rc);
				goto err_out;
			}

			if (!dir_fp->readdir_data.used) {
				free_page((unsigned long)
						(dir_fp->readdir_data.dirent));
				dir_fp->readdir_data.dirent = NULL;
				break;
			}

			de = (struct ksmbd_dirent *)
				((char *)dir_fp->readdir_data.dirent);
		} else {
			de = (struct ksmbd_dirent *)
				((char *)dir_fp->readdir_data.dirent +
				 dir_fp->dirent_offset);
		}

		reclen = ALIGN(sizeof(struct ksmbd_dirent) + de->namelen,
				sizeof(__le64));
		dir_fp->dirent_offset += reclen;

		if (dir_fp->readdir_data.file_attr &
			SMB_SEARCH_ATTRIBUTE_DIRECTORY && de->d_type != DT_DIR)
			continue;

		ksmbd_kstat.kstat = &kstat;

		if (de->namelen > NAME_MAX) {
			pr_err("filename length exceeds 255 bytes.\n");
			continue;
		}

		if (!strncmp(de->name, ".", de->namelen) ||
		    !strncmp(de->name, "..", de->namelen))
			continue;

		memcpy(d_info.smb1_name, de->name, de->namelen);
		d_info.smb1_name[de->namelen] = '\0';
		d_info.name = (const char *)d_info.smb1_name;
		d_info.name_len = de->namelen;
		rc = ksmbd_vfs_readdir_name(work,
					    file_mnt_user_ns(dir_fp->filp),
					    &ksmbd_kstat,
					    de->name,
					    de->namelen,
					    dirpath);
		if (rc) {
			ksmbd_debug(SMB, "Cannot read dirent: %d\n", rc);
			continue;
		}

		if (ksmbd_share_veto_filename(share, d_info.name)) {
			ksmbd_debug(SMB, "Veto filename %s\n", d_info.name);
			continue;
		}

		if (match_pattern(d_info.name, d_info.name_len, srch_ptr)) {
			rc = smb_populate_readdir_entry(conn,
				le16_to_cpu(req_params->InformationLevel),
				&d_info,
				&ksmbd_kstat);
			if (rc == -ENOSPC)
				break;
			else if (rc)
				goto err_out;
		}
	} while (d_info.out_buf_len >= 0);

	if (!d_info.data_count && *srch_ptr) {
		ksmbd_debug(SMB, "There is no entry matched with the search pattern\n");
		rc = -ENOENT;
		goto err_out;
	}

	if (d_info.out_buf_len < 0)
		dir_fp->dirent_offset -= reclen;

	params = (struct smb_com_trans2_ffirst_rsp_parms *)((char *)rsp +
			sizeof(struct smb_com_trans2_rsp));
	params->SearchHandle = dir_fp->volatile_id;
	params->SearchCount = cpu_to_le16(d_info.num_entry);
	params->LastNameOffset = cpu_to_le16(d_info.last_entry_offset);

	if (d_info.out_buf_len < 0) {
		ksmbd_debug(SMB, "continue search\n");
		params->EndofSearch = cpu_to_le16(0);
	} else {
		ksmbd_debug(SMB, "end of search\n");
		params->EndofSearch = cpu_to_le16(1);
		path_put(&(dir_fp->filp->f_path));
		if (le16_to_cpu(req_params->SearchFlags) &
				CIFS_SEARCH_CLOSE_AT_END)
			ksmbd_close_fd(work, dir_fp->volatile_id);
	}
	params->EAErrorOffset = cpu_to_le16(0);

	rsp_hdr->WordCount = 0x0A;
	rsp->t2.TotalParameterCount = cpu_to_le16(params_count);
	rsp->t2.TotalDataCount = cpu_to_le16(d_info.data_count);
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = cpu_to_le16(params_count);
	rsp->t2.ParameterOffset =
		cpu_to_le16(sizeof(struct smb_com_trans2_rsp) - 4);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = cpu_to_le16(d_info.data_count);
	rsp->t2.DataOffset = cpu_to_le16(sizeof(struct smb_com_trans2_rsp) +
		params_count + data_alignment_offset - 4);
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;
	rsp->Pad = 0;
	rsp->ByteCount = cpu_to_le16(d_info.data_count +
		params_count + 1 /*pad*/ + data_alignment_offset);
	memset((char *)rsp + sizeof(struct smb_com_trans2_rsp) + params_count,
			'\0', 2);
	inc_rfc1001_len(rsp_hdr, (10 * 2 + d_info.data_count +
				params_count + 1 + data_alignment_offset));
	kfree(srch_ptr);
	kfree(d_info.smb1_name);
	ksmbd_revert_fsids(work);
	return 0;

err_free_dirpath:
	kfree(dirpath);
err_out:
	if (rc == -EINVAL)
		rsp_hdr->Status.CifsError = STATUS_INVALID_PARAMETER;
	else if (rc == -EACCES || rc == -EXDEV)
		rsp_hdr->Status.CifsError = STATUS_ACCESS_DENIED;
	else if (rc == -ENOENT)
		rsp_hdr->Status.CifsError = STATUS_NO_SUCH_FILE;
	else if (rc == -EBADF)
		rsp_hdr->Status.CifsError = STATUS_FILE_CLOSED;
	else if (rc == -ENOMEM)
		rsp_hdr->Status.CifsError = STATUS_NO_MEMORY;
	else if (rc == -EFAULT)
		rsp_hdr->Status.CifsError = STATUS_INVALID_LEVEL;
	if (!rsp->hdr.Status.CifsError)
		rsp->hdr.Status.CifsError = STATUS_UNEXPECTED_IO_ERROR;

	if (dir_fp) {
		if (dir_fp->readdir_data.dirent)  {
			free_page((unsigned long)(dir_fp->readdir_data.dirent));
			dir_fp->readdir_data.dirent = NULL;
		}
		path_put(&(dir_fp->filp->f_path));
		ksmbd_close_fd(work, dir_fp->volatile_id);
	}

	kfree(srch_ptr);
	kfree(d_info.smb1_name);
	ksmbd_revert_fsids(work);
	return 0;
}

/**
 * find_next() - smb next readdir command
 * @work:	smb work containing find next request params
 *
 * if directory has many entries, find first can't read it fully.
 * find next might be called multiple times to read remaining dir entries
 *
 * Return:	0 on success, otherwise error
 */
static int find_next(struct ksmbd_work *work)
{
	struct smb_hdr *rsp_hdr = work->response_buf;
	struct ksmbd_conn *conn = work->conn;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct smb_com_trans2_req *req = work->request_buf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	struct smb_com_trans2_fnext_req_params *req_params;
	struct smb_com_trans2_fnext_rsp_params *params = NULL;
	struct ksmbd_dirent *de;
	struct ksmbd_file *dir_fp;
	struct kstat kstat;
	struct ksmbd_kstat ksmbd_kstat;
	struct ksmbd_dir_info d_info;
	int params_count = sizeof(struct smb_com_trans2_fnext_rsp_params);
	int data_alignment_offset = 0;
	int rc = 0, reclen = 0;
	__u16 sid;
	int header_size, srch_cnt, struct_sz;

	memset(&d_info, 0, sizeof(struct ksmbd_dir_info));

	req_params = (struct smb_com_trans2_fnext_req_params *)
		(work->request_buf + le16_to_cpu(req->ParameterOffset) + 4);
	sid = req_params->SearchHandle;

	dir_fp = ksmbd_lookup_fd_fast(work, sid);
	if (!dir_fp) {
		ksmbd_debug(SMB, "error invalid sid\n");
		rc = -EINVAL;
		goto err_out;
	}

	set_ctx_actor(&dir_fp->readdir_data.ctx, ksmbd_fill_dirent);

	if (params_count % 4)
		data_alignment_offset = 4 - params_count % 4;

	d_info.smb1_name = kmalloc(NAME_MAX + 1, GFP_KERNEL);
	if (!d_info.smb1_name) {
		rc = -ENOMEM;
		goto err_out;
	}
	d_info.wptr = (char *)((char *)rsp + sizeof(struct smb_com_trans2_rsp) +
			params_count + data_alignment_offset);

	header_size = sizeof(struct smb_com_trans2_rsp) + params_count +
		data_alignment_offset;

	srch_cnt = le16_to_cpu(req_params->SearchCount);
	struct_sz = readdir_info_level_struct_sz(le16_to_cpu(req_params->InformationLevel));

	if (struct_sz < 0) {
		rc = -EFAULT;
		goto err_out;
	}

	d_info.out_buf_len = min_t(int, srch_cnt * struct_sz + header_size,
				 MAX_CIFS_LOOKUP_BUFFER_SIZE - header_size);
	do {
		if (dir_fp->dirent_offset >= dir_fp->readdir_data.used) {
			dir_fp->dirent_offset = 0;
			dir_fp->readdir_data.used = 0;
			rc = iterate_dir(dir_fp->filp, &dir_fp->readdir_data.ctx);
			if (rc < 0) {
				ksmbd_debug(SMB, "err : %d\n", rc);
				goto err_out;
			}

			if (!dir_fp->readdir_data.used) {
				free_page((unsigned long)
						(dir_fp->readdir_data.dirent));
				dir_fp->readdir_data.dirent = NULL;
				break;
			}

			de = (struct ksmbd_dirent *)
				((char *)dir_fp->readdir_data.dirent);
		} else {
			de = (struct ksmbd_dirent *)
				((char *)dir_fp->readdir_data.dirent +
				 dir_fp->dirent_offset);
		}

		reclen = ALIGN(sizeof(struct ksmbd_dirent) + de->namelen,
				sizeof(__le64));
		dir_fp->dirent_offset += reclen;

		if (dir_fp->readdir_data.file_attr &
			SMB_SEARCH_ATTRIBUTE_DIRECTORY && de->d_type != DT_DIR)
			continue;

		if (dir_fp->readdir_data.file_attr &
			SMB_SEARCH_ATTRIBUTE_ARCHIVE && (de->d_type == DT_DIR ||
			(!strcmp(de->name, ".") || !strcmp(de->name, ".."))))
			continue;

		ksmbd_kstat.kstat = &kstat;

		if (de->namelen > NAME_MAX) {
			pr_err("filename length exceeds 255 bytes.\n");
			continue;
		}
		memcpy(d_info.smb1_name, de->name, de->namelen);
		d_info.smb1_name[de->namelen] = '\0';
		d_info.name = (const char *)d_info.smb1_name;
		d_info.name_len = de->namelen;
		rc = ksmbd_vfs_readdir_name(work,
					    file_mnt_user_ns(dir_fp->filp),
					    &ksmbd_kstat,
					    de->name,
					    de->namelen,
					    dir_fp->filename);
		if (rc) {
			ksmbd_debug(SMB, "Err while dirent read rc = %d\n", rc);
			rc = 0;
			continue;
		}

		if (ksmbd_share_veto_filename(share, d_info.name)) {
			ksmbd_debug(SMB, "file(%s) is invisible by setting as veto file\n",
				d_info.name);
			continue;
		}

		ksmbd_debug(SMB, "filename string = %.*s\n",
				d_info.name_len, d_info.name);
		rc = smb_populate_readdir_entry(conn,
			le16_to_cpu(req_params->InformationLevel), &d_info,
			&ksmbd_kstat);
		if (rc == -ENOSPC)
			break;
		else if (rc)
			goto err_out;

	} while (d_info.out_buf_len >= 0);

	if (d_info.out_buf_len < 0)
		dir_fp->dirent_offset -= reclen;

	params = (struct smb_com_trans2_fnext_rsp_params *)
		((char *)rsp + sizeof(struct smb_com_trans_rsp));
	params->SearchCount = cpu_to_le16(d_info.num_entry);

	if (d_info.out_buf_len < 0) {
		ksmbd_debug(SMB, "continue search\n");
		params->EndofSearch = cpu_to_le16(0);
		params->LastNameOffset = cpu_to_le16(d_info.last_entry_offset);
	} else {
		ksmbd_debug(SMB, "end of search\n");
		params->EndofSearch = cpu_to_le16(1);
		params->LastNameOffset = cpu_to_le16(0);
		path_put(&(dir_fp->filp->f_path));
		if (le16_to_cpu(req_params->SearchFlags) &
				CIFS_SEARCH_CLOSE_AT_END)
			ksmbd_close_fd(work, sid);
	}
	params->EAErrorOffset = cpu_to_le16(0);

	rsp_hdr->WordCount = 0x0A;
	rsp->t2.TotalParameterCount = cpu_to_le16(params_count);
	rsp->t2.TotalDataCount = cpu_to_le16(d_info.data_count);
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = cpu_to_le16(params_count);
	rsp->t2.ParameterOffset =
		cpu_to_le16(sizeof(struct smb_com_trans_rsp) - 4);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = cpu_to_le16(d_info.data_count);
	rsp->t2.DataOffset = cpu_to_le16(sizeof(struct smb_com_trans_rsp) +
		params_count + data_alignment_offset - 4);
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;
	rsp->Pad = 0;
	rsp->ByteCount = cpu_to_le16(d_info.data_count + params_count + 1 +
		data_alignment_offset);
	memset((char *)rsp + sizeof(struct smb_com_trans_rsp) +
		params_count, '\0', data_alignment_offset);
	inc_rfc1001_len(rsp_hdr, (10 * 2 + d_info.data_count +
		params_count + 1 + data_alignment_offset));
	kfree(d_info.smb1_name);
	ksmbd_fd_put(work, dir_fp);
	return 0;

err_out:
	if (rc == -EINVAL)
		rsp_hdr->Status.CifsError = STATUS_INVALID_PARAMETER;
	else if (rc == -EACCES || rc == -EXDEV)
		rsp_hdr->Status.CifsError = STATUS_ACCESS_DENIED;
	else if (rc == -ENOENT)
		rsp_hdr->Status.CifsError = STATUS_NO_SUCH_FILE;
	else if (rc == -EBADF)
		rsp_hdr->Status.CifsError = STATUS_FILE_CLOSED;
	else if (rc == -ENOMEM)
		rsp_hdr->Status.CifsError = STATUS_NO_MEMORY;
	else if (rc == -EFAULT)
		rsp_hdr->Status.CifsError = STATUS_INVALID_LEVEL;
	if (!rsp->hdr.Status.CifsError)
		rsp->hdr.Status.CifsError = STATUS_UNEXPECTED_IO_ERROR;

	if (dir_fp) {
		if (dir_fp->readdir_data.dirent)  {
			free_page((unsigned long)(dir_fp->readdir_data.dirent));
			dir_fp->readdir_data.dirent = NULL;
		}
		path_put(&(dir_fp->filp->f_path));
		ksmbd_close_fd(work, sid);
	}

	kfree(d_info.smb1_name);
	return 0;
}

/**
 * smb_set_alloc_size() - set file truncate method using trans2
 *		set file info command - file allocation info level
 * @work:	smb work containing set file info command buffer
 *
 * Return:	0 on success, otherwise error
 */
static int smb_set_alloc_size(struct ksmbd_work *work)
{
	struct smb_com_trans2_sfi_req *req;
	struct smb_com_trans2_sfi_rsp *rsp;
	struct file_allocation_info *allocinfo;
	struct kstat stat;
	struct ksmbd_file *fp = NULL;
	loff_t newsize;
	int err = 0;

	req = (struct smb_com_trans2_sfi_req *)work->request_buf;
	rsp = (struct smb_com_trans2_sfi_rsp *)work->response_buf;

	allocinfo =  (struct file_allocation_info *)
		(((char *) &req->hdr.Protocol) + le16_to_cpu(req->DataOffset));
	newsize = le64_to_cpu(allocinfo->AllocationSize);

	fp = ksmbd_lookup_fd_fast(work, req->Fid);
	if (!fp) {
		pr_err("failed to get filp for fid %u\n", req->Fid);
		rsp->hdr.Status.CifsError = STATUS_FILE_CLOSED;
		return -ENOENT;
	}

	err = ksmbd_vfs_getattr(&fp->filp->f_path, &stat);
	if (err) {
		rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
		ksmbd_fd_put(work, fp);
		return err;
	}

	if (newsize == stat.size) /* nothing to do */
		goto out;

	/* Round up size */
	if (alloc_roundup_size) {
		newsize = div64_u64(newsize + alloc_roundup_size - 1,
				alloc_roundup_size);
		newsize *= alloc_roundup_size;
	}

	err = ksmbd_vfs_truncate(work, fp, newsize);
	if (err) {
		rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
		ksmbd_fd_put(work, fp);
		return err;
	}

out:
	ksmbd_debug(SMB, "fid %u, truncated to newsize %llu\n",
			req->Fid, newsize);

	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = 0;
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = rsp->t2.TotalParameterCount;
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = rsp->t2.TotalDataCount;
	rsp->t2.DataOffset = 0;
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;

	/* 3 pad (1 pad1 + 2 pad2)*/
	rsp->ByteCount = cpu_to_le16(3);
	rsp->Reserved2 = 0;
	inc_rfc1001_len(&rsp->hdr, rsp->hdr.WordCount * 2 + 3);
	ksmbd_fd_put(work, fp);

	return 0;
}

/**
 * smb_set_file_size_finfo() - set file truncate method using trans2
 *		set file info command
 * @work:	smb work containing set file info command buffer
 *
 * Return:	0 on success, otherwise error
 */
static int smb_set_file_size_finfo(struct ksmbd_work *work)
{
	struct smb_com_trans2_sfi_req *req;
	struct smb_com_trans2_sfi_rsp *rsp;
	struct file_end_of_file_info *eofinfo;
	struct ksmbd_file *fp;
	loff_t newsize;
	int err = 0;

	req = (struct smb_com_trans2_sfi_req *)work->request_buf;
	rsp = (struct smb_com_trans2_sfi_rsp *)work->response_buf;

	eofinfo =  (struct file_end_of_file_info *)
		(((char *) &req->hdr.Protocol) + le16_to_cpu(req->DataOffset));

	fp = ksmbd_lookup_fd_fast(work, req->Fid);
	if (!fp) {
		pr_err("failed to get filp for fid %u\n", req->Fid);
		rsp->hdr.Status.CifsError = STATUS_FILE_CLOSED;
		return -ENOENT;
	}

	newsize = le64_to_cpu(eofinfo->FileSize);
	err = ksmbd_vfs_truncate(work, fp, newsize);
	if (err) {
		rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
		ksmbd_fd_put(work, fp);
		return err;
	}

	ksmbd_debug(SMB, "fid %u, truncated to newsize %lld\n", req->Fid,
		newsize);
	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = 0;
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = rsp->t2.TotalParameterCount;
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = rsp->t2.TotalDataCount;
	rsp->t2.DataOffset = 0;
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;

	/* 3 pad (1 pad1 + 2 pad2)*/
	rsp->ByteCount = cpu_to_le16(3);
	rsp->Reserved2 = 0;
	inc_rfc1001_len(&rsp->hdr, rsp->hdr.WordCount * 2 + 3);
	ksmbd_fd_put(work, fp);

	return 0;
}

/**
 * query_file_info_pipe() - query file info of IPC pipe
 *		using query file info command
 * @work:	smb work containing query file info command buffer
 *
 * Return:	0 on success, otherwise error
 */
static int query_file_info_pipe(struct ksmbd_work *work)
{
	struct smb_hdr *rsp_hdr = work->response_buf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	struct smb_com_trans2_req *req = work->request_buf;
	struct smb_trans2_qfi_req_params *req_params;
	struct file_standard_info *standard_info;
	char *ptr;

	req_params = (struct smb_trans2_qfi_req_params *)(work->request_buf +
			le16_to_cpu(req->ParameterOffset) + 4);

	if (le16_to_cpu(req_params->InformationLevel) !=
	    SMB_QUERY_FILE_STANDARD_INFO) {
		ksmbd_debug(SMB, "query file info for info %u not supported\n",
				le16_to_cpu(req_params->InformationLevel));
		rsp_hdr->Status.CifsError = STATUS_NOT_SUPPORTED;
		return -EOPNOTSUPP;
	}

	ksmbd_debug(SMB, "SMB_QUERY_FILE_STANDARD_INFO\n");
	rsp_hdr->WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = cpu_to_le16(sizeof(struct file_standard_info));
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = cpu_to_le16(2);
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = cpu_to_le16(sizeof(struct file_standard_info));
	rsp->t2.DataOffset = cpu_to_le16(60);
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;
	/*2 for parameter count & 3 pad (1pad1 + 2 pad2)*/
	rsp->ByteCount = cpu_to_le16(2 + sizeof(struct file_standard_info) + 3);
	rsp->Pad = 0;
	/* lets set EA info */
	ptr = (char *)&rsp->Pad + 1;
	memset(ptr, 0, 4);
	standard_info = (struct file_standard_info *)(ptr + 4);
	standard_info->AllocationSize = cpu_to_le64(4096);
	standard_info->EndOfFile = 0;
	standard_info->NumberOfLinks = cpu_to_le32(1);
	standard_info->DeletePending = 0;
	standard_info->Directory = 0;
	standard_info->DeletePending = 1;
	inc_rfc1001_len(rsp_hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));

	return 0;
}

/**
 * query_file_info() - query file info of file/dir
 *		using query file info command
 * @work:	smb work containing query file info command buffer
 *
 * Return:	0 on success, otherwise error
 */
static int query_file_info(struct ksmbd_work *work)
{
	struct ksmbd_conn *conn = work->conn;
	struct smb_hdr *rsp_hdr = work->response_buf;
	struct smb_com_trans2_req *req = work->request_buf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	struct smb_trans2_qfi_req_params *req_params;
	struct ksmbd_file *fp;
	struct kstat st;
	char *ptr;
	int rc = 0;
	u64 time;

	req_params = (struct smb_trans2_qfi_req_params *)(work->request_buf +
			le16_to_cpu(req->ParameterOffset) + 4);

	if (test_share_config_flag(work->tcon->share_conf,
				   KSMBD_SHARE_FLAG_PIPE)) {
		ksmbd_debug(SMB, "query file info for IPC srvsvc\n");
		return query_file_info_pipe(work);
	}

	fp = ksmbd_lookup_fd_fast(work, req_params->Fid);
	if (!fp) {
		pr_err("failed to get filp for fid %u\n", req_params->Fid);
		rsp_hdr->Status.CifsError = STATUS_UNEXPECTED_IO_ERROR;
		rc = -EIO;
		goto err_out;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
	generic_fillattr(file_mnt_user_ns(fp->filp), file_inode(fp->filp), &st);
#else
	generic_fillattr(file_inode(fp->filp), &st);
#endif

	switch (le16_to_cpu(req_params->InformationLevel)) {

	case SMB_QUERY_FILE_STANDARD_INFO:
	{
		struct file_standard_info *standard_info;
		unsigned int delete_pending;

		ksmbd_debug(SMB, "SMB_QUERY_FILE_STANDARD_INFO\n");
		delete_pending = ksmbd_inode_pending_delete(fp);
		rsp_hdr->WordCount = 10;
		rsp->t2.TotalParameterCount = cpu_to_le16(2);
		rsp->t2.TotalDataCount =
			cpu_to_le16(sizeof(struct file_standard_info));
		rsp->t2.Reserved = 0;
		rsp->t2.ParameterCount = cpu_to_le16(2);
		rsp->t2.ParameterOffset = cpu_to_le16(56);
		rsp->t2.ParameterDisplacement = 0;
		rsp->t2.DataCount =
			cpu_to_le16(sizeof(struct file_standard_info));
		rsp->t2.DataOffset = cpu_to_le16(60);
		rsp->t2.DataDisplacement = 0;
		rsp->t2.SetupCount = 0;
		rsp->t2.Reserved1 = 0;
		/*2 for parameter count & 3 pad (1pad1 + 2 pad2)*/
		rsp->ByteCount =
			cpu_to_le16(2 + sizeof(struct file_standard_info) + 3);
		rsp->Pad = 0;
		/* lets set EA info */
		ptr = (char *)&rsp->Pad + 1;
		memset(ptr, 0, 4);
		standard_info = (struct file_standard_info *)(ptr + 4);
		standard_info->AllocationSize = cpu_to_le64(st.blocks << 9);
		standard_info->EndOfFile = cpu_to_le64(st.size);
		standard_info->NumberOfLinks = cpu_to_le32(get_nlink(&st) -
			delete_pending);
		standard_info->DeletePending = delete_pending;
		standard_info->Directory = S_ISDIR(st.mode) ? 1 : 0;
		inc_rfc1001_len(rsp_hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));
		break;
	}
	case SMB_QUERY_FILE_BASIC_INFO:
	{
		struct file_basic_info *basic_info;

		ksmbd_debug(SMB, "SMB_QUERY_FILE_BASIC_INFO\n");
		rsp_hdr->WordCount = 10;
		rsp->t2.TotalParameterCount = cpu_to_le16(2);
		rsp->t2.TotalDataCount =
			cpu_to_le16(sizeof(struct file_basic_info));
		rsp->t2.Reserved = 0;
		rsp->t2.ParameterCount = cpu_to_le16(2);
		rsp->t2.ParameterOffset = cpu_to_le16(56);
		rsp->t2.ParameterDisplacement = 0;
		rsp->t2.DataCount = cpu_to_le16(sizeof(struct file_basic_info));
		rsp->t2.DataOffset = cpu_to_le16(60);
		rsp->t2.DataDisplacement = 0;
		rsp->t2.SetupCount = 0;
		rsp->t2.Reserved1 = 0;
		/*2 for parameter count & 3 pad (1pad1 + 2 pad2)*/
		rsp->ByteCount =
			cpu_to_le16(2 + sizeof(struct file_basic_info) + 3);
		rsp->Pad = 0;
		/* lets set EA info */
		ptr = (char *)&rsp->Pad + 1;
		memset(ptr, 0, 4);
		basic_info = (struct file_basic_info *)(ptr + 4);
		basic_info->CreationTime =
			cpu_to_le64(fp->create_time);
		time = ksmbd_UnixTimeToNT(st.atime);
		basic_info->LastAccessTime = cpu_to_le64(time);
		time = ksmbd_UnixTimeToNT(st.mtime);
		basic_info->LastWriteTime = cpu_to_le64(time);
		time = ksmbd_UnixTimeToNT(st.ctime);
		basic_info->ChangeTime = cpu_to_le64(time);
		basic_info->Attributes = S_ISDIR(st.mode) ?
			ATTR_DIRECTORY_LE : ATTR_ARCHIVE_LE;
		basic_info->Pad = 0;
		inc_rfc1001_len(rsp_hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));
		break;
	}
	case SMB_QUERY_FILE_EA_INFO:
	{
		struct file_ea_info *ea_info;

		ksmbd_debug(SMB, "SMB_QUERY_FILE_EA_INFO\n");
		rsp_hdr->WordCount = 10;
		rsp->t2.TotalParameterCount = cpu_to_le16(2);
		rsp->t2.TotalDataCount =
			cpu_to_le16(sizeof(struct file_ea_info));
		rsp->t2.Reserved = 0;
		rsp->t2.ParameterCount = cpu_to_le16(2);
		rsp->t2.ParameterOffset = cpu_to_le16(56);
		rsp->t2.ParameterDisplacement = 0;
		rsp->t2.DataCount = cpu_to_le16(sizeof(struct file_ea_info));
		rsp->t2.DataOffset = cpu_to_le16(60);
		rsp->t2.DataDisplacement = 0;
		rsp->t2.SetupCount = 0;
		rsp->t2.Reserved1 = 0;
		/*2 for parameter count & 3 pad (1pad1 + 2 pad2)*/
		rsp->ByteCount =
			cpu_to_le16(2 + sizeof(struct file_ea_info) + 3);
		rsp->Pad = 0;
		/* lets set EA info */
		ptr = (char *)&rsp->Pad + 1;
		memset(ptr, 0, 4);
		ea_info = (struct file_ea_info *)(ptr + 4);
		ea_info->EaSize = 0;
		inc_rfc1001_len(rsp_hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));
		break;
	}
	case SMB_QUERY_FILE_UNIX_BASIC:
	{
		struct file_unix_basic_info *uinfo;

		ksmbd_debug(SMB, "SMB_QUERY_FILE_UNIX_BASIC\n");
		rsp_hdr->WordCount = 10;
		rsp->t2.TotalParameterCount = cpu_to_le16(2);
		rsp->t2.TotalDataCount =
			cpu_to_le16(sizeof(struct file_unix_basic_info));
		rsp->t2.Reserved = 0;
		rsp->t2.ParameterCount = cpu_to_le16(2);
		rsp->t2.ParameterOffset = cpu_to_le16(56);
		rsp->t2.ParameterDisplacement = 0;
		rsp->t2.DataCount =
			cpu_to_le16(sizeof(struct file_unix_basic_info));
		rsp->t2.DataOffset = cpu_to_le16(60);
		rsp->t2.DataDisplacement = 0;
		rsp->t2.SetupCount = 0;
		rsp->t2.Reserved1 = 0;
		/*2 for parameter count & 3 pad (1pad1 + 2 pad2)*/
		rsp->ByteCount =
			cpu_to_le16(2 + sizeof(struct file_unix_basic_info)
				+ 3);
		rsp->Pad = 0;
		/* lets set unix info info */
		ptr = (char *)&rsp->Pad + 1;
		memset(ptr, 0, 4);
		uinfo = (struct file_unix_basic_info *)(ptr + 4);
		init_unix_info(uinfo, file_mnt_user_ns(fp->filp), &st);
		inc_rfc1001_len(rsp_hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));
		break;
	}
	case SMB_QUERY_FILE_NAME_INFO:
	{
		struct file_name_info *name_info;
		int uni_filename_len;
		char *filename;

		ksmbd_debug(SMB, "SMB_QUERY_FILE_NAME_INFO\n");
		ptr = (char *)&rsp->Pad + 1;
		memset(ptr, 0, 4);
		name_info = (struct file_name_info *)(ptr + 4);

		filename = convert_to_nt_pathname(work->tcon->share_conf,
						  &fp->filp->f_path);
		if (!filename) {
			rc = -ENOMEM;
			goto err_out;
		}
		uni_filename_len = smbConvertToUTF16(
				(__le16 *)name_info->FileName,
				filename, PATH_MAX,
				conn->local_nls, 0);
		kfree(filename);
		uni_filename_len *= 2;
		name_info->FileNameLength = cpu_to_le32(uni_filename_len);

		rsp_hdr->WordCount = 10;
		rsp->t2.TotalParameterCount = cpu_to_le16(2);
		rsp->t2.TotalDataCount = cpu_to_le16(uni_filename_len + 4);
		rsp->t2.Reserved = 0;
		rsp->t2.ParameterCount = cpu_to_le16(2);
		rsp->t2.ParameterOffset = cpu_to_le16(56);
		rsp->t2.ParameterDisplacement = 0;
		rsp->t2.DataCount = cpu_to_le16(uni_filename_len + 4);
		rsp->t2.DataOffset = cpu_to_le16(60);
		rsp->t2.DataDisplacement = 0;
		rsp->t2.SetupCount = 0;
		rsp->t2.Reserved1 = 0;
		/*2 for parameter count & 3 pad (1pad1 + 2 pad2)*/
		rsp->ByteCount = cpu_to_le16(2 + uni_filename_len + 4 + 3);
		rsp->Pad = 0;
		inc_rfc1001_len(rsp_hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));
		break;
	}
	case SMB_QUERY_FILE_ALL_INFO:
	{
		struct file_all_info *ainfo;
		unsigned int delete_pending;

		ksmbd_debug(SMB, "SMB_QUERY_FILE_UNIX_BASIC\n");
		delete_pending = ksmbd_inode_pending_delete(fp);
		rsp_hdr->WordCount = 10;
		rsp->t2.TotalParameterCount = cpu_to_le16(2);
		rsp->t2.TotalDataCount =
			cpu_to_le16(sizeof(struct file_all_info));
		rsp->t2.Reserved = 0;
		rsp->t2.ParameterCount = cpu_to_le16(2);
		rsp->t2.ParameterOffset = cpu_to_le16(56);
		rsp->t2.ParameterDisplacement = 0;
		rsp->t2.DataCount = cpu_to_le16(sizeof(struct file_all_info));
		rsp->t2.DataOffset = cpu_to_le16(60);
		rsp->t2.DataDisplacement = 0;
		rsp->t2.SetupCount = 0;
		rsp->t2.Reserved1 = 0;
		/*2 for parameter count & 3 pad (1pad1 + 2 pad2)*/
		rsp->ByteCount =
			cpu_to_le16(2 + sizeof(struct file_all_info) + 3);
		rsp->Pad = 0;
		/* lets set all info info */
		ptr = (char *)&rsp->Pad + 1;
		memset(ptr, 0, 4);
		ainfo = (struct file_all_info *)(ptr + 4);
		ainfo->CreationTime = cpu_to_le64(fp->create_time);
		time = ksmbd_UnixTimeToNT(st.atime);
		ainfo->LastAccessTime = cpu_to_le64(time);
		time = ksmbd_UnixTimeToNT(st.mtime);
		ainfo->LastWriteTime = cpu_to_le64(time);
		time = ksmbd_UnixTimeToNT(st.ctime);
		ainfo->ChangeTime = cpu_to_le64(time);
		ainfo->Attributes = cpu_to_le32(S_ISDIR(st.mode) ?
				ATTR_DIRECTORY : ATTR_ARCHIVE);
		ainfo->Pad1 = 0;
		ainfo->AllocationSize = cpu_to_le64(st.blocks << 9);
		ainfo->EndOfFile = cpu_to_le64(st.size);
		ainfo->NumberOfLinks = cpu_to_le32(get_nlink(&st) -
			delete_pending);
		ainfo->DeletePending = delete_pending;
		ainfo->Directory = S_ISDIR(st.mode) ? 1 : 0;
		ainfo->Pad2 = 0;
		ainfo->EASize = 0;
		ainfo->FileNameLength = 0;
		inc_rfc1001_len(rsp_hdr, 10 * 2 + le16_to_cpu(rsp->ByteCount));
		break;
	}
	default:
		pr_err("query path info not implemnted for %x\n",
		       le16_to_cpu(req_params->InformationLevel));
		rsp_hdr->Status.CifsError = STATUS_NOT_SUPPORTED;
		rc = -EINVAL;
		goto err_out;

	}

err_out:
	ksmbd_fd_put(work, fp);
	return rc;
}

/**
 * smb_set_unix_fileinfo() - set smb unix file info(setattr)
 * @work:	smb work containing unix basic info buffer
 *
 * Return:	0 on success, otherwise error
 */
static int smb_set_unix_fileinfo(struct ksmbd_work *work)
{
	struct smb_com_trans2_sfi_req *req = work->request_buf;
	struct smb_com_trans2_sfi_rsp *rsp = work->response_buf;
	struct file_unix_basic_info *unix_info;
	struct ksmbd_file *fp;
	struct iattr attrs;
	int err = 0;

	if (ksmbd_override_fsids(work))
		return -ENOMEM;
	fp = ksmbd_lookup_fd_fast(work, req->Fid);
	if (!fp) {
		ksmbd_revert_fsids(work);
		return -ENOENT;
	}

	unix_info =  (struct file_unix_basic_info *)
		(((char *) &req->hdr.Protocol) + le16_to_cpu(req->DataOffset));

	attrs.ia_valid = 0;
	attrs.ia_mode = 0;
	err = unix_info_to_attr(unix_info,
				file_mnt_user_ns(fp->filp), &attrs);
	ksmbd_fd_put(work, fp);
	ksmbd_revert_fsids(work);
	if (err)
		goto out;

	err = ksmbd_vfs_setattr(work, NULL, (u64)req->Fid, &attrs);
	if (err)
		goto out;

	/* setattr success, prepare response */
	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = 0;
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = rsp->t2.TotalParameterCount;
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = rsp->t2.TotalDataCount;
	rsp->t2.DataOffset = 0;
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;

	/* 3 pad (1 pad1 + 2 pad2)*/
	rsp->ByteCount = cpu_to_le16(3);
	rsp->Reserved2 = 0;
	inc_rfc1001_len(&rsp->hdr,
			rsp->hdr.WordCount * 2 + le16_to_cpu(rsp->ByteCount));

out:
	if (err) {
		rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
		return err;
	}
	return 0;
}

/**
 * smb_set_dispostion() - set file dispostion method using trans2
 *		using set file info command
 * @work:	smb work containing set file info command buffer
 *
 * Return:	0 on success, otherwise error
 */
static int smb_set_dispostion(struct ksmbd_work *work)
{
	struct smb_com_trans2_sfi_req *req = work->request_buf;
	struct smb_com_trans2_sfi_rsp *rsp = work->response_buf;
	char *disp_info;
	struct ksmbd_file *fp;
	int ret = 0;

	disp_info =  (char *) (((char *) &req->hdr.Protocol)
			+ le16_to_cpu(req->DataOffset));

	fp = ksmbd_lookup_fd_fast(work, req->Fid);
	if (!fp) {
		ksmbd_debug(SMB, "Invalid id for close: %d\n", req->Fid);
		rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
		return -EINVAL;
	}

	if (*disp_info) {
		if (!fp->is_nt_open) {
			rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
			ret = -EPERM;
			goto err_out;
		}

		if (!(file_inode(fp->filp)->i_mode & 0222)) {
			rsp->hdr.Status.CifsError = STATUS_CANNOT_DELETE;
			ret = -EPERM;
			goto err_out;
		}

		if (S_ISDIR(file_inode(fp->filp)->i_mode) &&
				ksmbd_vfs_empty_dir(fp) == -ENOTEMPTY) {
			rsp->hdr.Status.CifsError = STATUS_DIRECTORY_NOT_EMPTY;
			ret = -ENOTEMPTY;
			goto err_out;
		}

		ksmbd_set_inode_pending_delete(fp);
	} else {
		ksmbd_clear_inode_pending_delete(fp);
	}

	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = 0;
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = rsp->t2.TotalParameterCount;
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = rsp->t2.TotalDataCount;
	rsp->t2.DataOffset = 0;
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;

	/* 3 pad (1 pad1 + 2 pad2)*/
	rsp->ByteCount = cpu_to_le16(3);
	rsp->Reserved2 = 0;
	inc_rfc1001_len(&rsp->hdr,
			rsp->hdr.WordCount * 2 + 3);

err_out:
	ksmbd_fd_put(work, fp);
	return ret;
}

/**
 * smb_set_time_fileinfo() - set file time method using trans2
 *		using set file info command
 * @work:	smb work containing set file info command buffer
 *
 * Return:	0 on success, otherwise error
 */
static int smb_set_time_fileinfo(struct ksmbd_work *work)
{
	struct smb_com_trans2_sfi_req *req;
	struct smb_com_trans2_sfi_rsp *rsp;
	struct file_basic_info *info;
	struct iattr attrs;
	int err = 0;

	req = (struct smb_com_trans2_sfi_req *)work->request_buf;
	rsp = (struct smb_com_trans2_sfi_rsp *)work->response_buf;

	info = (struct file_basic_info *)(((char *) &req->hdr.Protocol) +
			le16_to_cpu(req->DataOffset));

	attrs.ia_valid = 0;
	if (le64_to_cpu(info->LastAccessTime)) {
		attrs.ia_atime = smb_NTtimeToUnix(info->LastAccessTime);
		attrs.ia_valid |= (ATTR_ATIME | ATTR_ATIME_SET);
	}

	if (le64_to_cpu(info->ChangeTime)) {
		attrs.ia_ctime = smb_NTtimeToUnix(info->ChangeTime);
		attrs.ia_valid |= ATTR_CTIME;
	}

	if (le64_to_cpu(info->LastWriteTime)) {
		attrs.ia_mtime = smb_NTtimeToUnix(info->LastWriteTime);
		attrs.ia_valid |= (ATTR_MTIME | ATTR_MTIME_SET);
	}
	/* TODO: check dos mode and acl bits if req->Attributes nonzero */

	if (!attrs.ia_valid)
		goto done;

	err = ksmbd_vfs_setattr(work, NULL, (u64)req->Fid, &attrs);
	if (err) {
		rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
		return err;
	}

done:
	ksmbd_debug(SMB, "fid %u, setattr done\n", req->Fid);
	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = 0;
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = rsp->t2.TotalParameterCount;
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = rsp->t2.TotalDataCount;
	rsp->t2.DataOffset = 0;
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;

	/* 3 pad (1 pad1 + 2 pad2)*/
	rsp->ByteCount = cpu_to_le16(3);
	rsp->Reserved2 = 0;
	inc_rfc1001_len(&rsp->hdr,
			rsp->hdr.WordCount * 2 + 3);

	return 0;
}

/**
 * smb_fileinfo_rename() - rename method using trans2 set file info command
 * @work:	smb work containing set file info command buffer
 *
 * Return:	0 on success, otherwise error
 */
static int smb_fileinfo_rename(struct ksmbd_work *work)
{
	struct smb_com_trans2_sfi_req *req;
	struct smb_com_trans2_sfi_rsp *rsp;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct set_file_rename *info;
	struct ksmbd_file *fp;
	char *newname;
	int rc = 0;

	req = (struct smb_com_trans2_sfi_req *)work->request_buf;
	rsp = (struct smb_com_trans2_sfi_rsp *)work->response_buf;
	info =  (struct set_file_rename *)
		(((char *) &req->hdr.Protocol) + le16_to_cpu(req->DataOffset));

	if (!test_tree_conn_flag(work->tcon, KSMBD_TREE_CONN_FLAG_WRITABLE)) {
		ksmbd_debug(SMB,
			"returning as user does not have permission to write\n");
		rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
		return -EACCES;
	}

	fp = ksmbd_lookup_fd_fast(work, req->Fid);
	if (!fp) {
		pr_err("failed to get filp for fid %u\n", req->Fid);
		rsp->hdr.Status.CifsError = STATUS_FILE_CLOSED;
		return -ENOENT;
	}

	if (info->overwrite) {
		rc = ksmbd_vfs_truncate(work, fp, 0);
		if (rc) {
			rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
			ksmbd_fd_put(work, fp);
			return rc;
		}
	}

	newname = smb_get_name(share, info->target_name, PATH_MAX, work, 0);
	if (IS_ERR(newname)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		ksmbd_fd_put(work, fp);
		return PTR_ERR(newname);
	}

	ksmbd_debug(SMB, "new name(%s)\n", newname);
	rc = ksmbd_vfs_fp_rename(work, fp, newname);
	if (rc) {
		rsp->hdr.Status.CifsError = STATUS_UNEXPECTED_IO_ERROR;
		goto out;
	}

	rsp->hdr.WordCount = 10;
	rsp->t2.TotalParameterCount = cpu_to_le16(2);
	rsp->t2.TotalDataCount = 0;
	rsp->t2.Reserved = 0;
	rsp->t2.ParameterCount = rsp->t2.TotalParameterCount;
	rsp->t2.ParameterOffset = cpu_to_le16(56);
	rsp->t2.ParameterDisplacement = 0;
	rsp->t2.DataCount = rsp->t2.TotalDataCount;
	rsp->t2.DataOffset = 0;
	rsp->t2.DataDisplacement = 0;
	rsp->t2.SetupCount = 0;
	rsp->t2.Reserved1 = 0;

	/* 3 pad (1 pad1 + 2 pad2)*/
	rsp->ByteCount = cpu_to_le16(3);
	rsp->Reserved2 = 0;
	inc_rfc1001_len(&rsp->hdr, rsp->hdr.WordCount * 2 + 3);

out:
	ksmbd_fd_put(work, fp);
	kfree(newname);
	return rc;
}

/**
 * set_file_info() - trans2 set file info command dispatcher
 * @work:	smb work containing set file info command buffer
 *
 * Return:	0 on success, otherwise error
 */
static int set_file_info(struct ksmbd_work *work)
{
	struct smb_com_trans2_sfi_req *req;
	struct smb_com_trans2_sfi_rsp *rsp;
	__u16 info_level, total_param;
	int err = 0;

	req = (struct smb_com_trans2_sfi_req *)work->request_buf;
	rsp = (struct smb_com_trans2_sfi_rsp *)work->response_buf;
	info_level = le16_to_cpu(req->InformationLevel);
	total_param = le16_to_cpu(req->TotalParameterCount);
	if (total_param < 4) {
		rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
		pr_err("invalid total parameter for info_level 0x%x\n",
		       total_param);
		return -EINVAL;
	}

	switch (info_level) {
	case SMB_SET_FILE_EA:
		err = smb_set_ea(work);
		break;
	case SMB_SET_FILE_ALLOCATION_INFO2:
		/* fall through */
	case SMB_SET_FILE_ALLOCATION_INFO:
		err = smb_set_alloc_size(work);
		break;
	case SMB_SET_FILE_END_OF_FILE_INFO2:
		/* fall through */
	case SMB_SET_FILE_END_OF_FILE_INFO:
		err = smb_set_file_size_finfo(work);
		break;
	case SMB_SET_FILE_UNIX_BASIC:
		err = smb_set_unix_fileinfo(work);
		break;
	case SMB_SET_FILE_DISPOSITION_INFO:
	case SMB_SET_FILE_DISPOSITION_INFORMATION:
		err = smb_set_dispostion(work);
		break;
	case SMB_SET_FILE_BASIC_INFO2:
		/* fall through */
	case SMB_SET_FILE_BASIC_INFO:
		err = smb_set_time_fileinfo(work);
		break;
	case SMB_SET_FILE_RENAME_INFORMATION:
		err = smb_fileinfo_rename(work);
		break;
	default:
		ksmbd_debug(SMB, "info level = %x not implemented yet\n",
				info_level);
		rsp->hdr.Status.CifsError = STATUS_NOT_IMPLEMENTED;
		return -EOPNOTSUPP;
	}

	if (err < 0)
		ksmbd_debug(SMB, "info_level 0x%x failed, err %d\n",
				info_level, err);
	return err;
}

/**
 * create_dir() - trans2 create directory dispatcher
 * @work:   smb work containing set file info command buffer
 *
 * Return:      0 on success, otherwise error
 */
static int create_dir(struct ksmbd_work *work)
{
	struct smb_com_trans2_req *req = work->request_buf;
	struct smb_com_trans2_rsp *rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	mode_t mode = S_IALLUGO;
	char *name;
	int err;

	name = smb_get_name(share, work->request_buf +
			le16_to_cpu(req->ParameterOffset) + 4,
			PATH_MAX, work, false);
	if (IS_ERR(name)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(name);
	}

	if (ksmbd_override_fsids(work)) {
		kfree(name);
		rsp->hdr.Status.CifsError = STATUS_NO_MEMORY;
		return -ENOMEM;
	}

	err = ksmbd_vfs_mkdir(work, name, mode);
	if (err) {
		if (err == -EEXIST) {
			if (!(((struct smb_hdr *)work->request_buf)->Flags2 &
						SMBFLG2_ERR_STATUS)) {
				ntstatus_to_dos(STATUS_OBJECT_NAME_COLLISION,
					&rsp->hdr.Status.DosError.ErrorClass,
					&rsp->hdr.Status.DosError.Error);
			} else
				rsp->hdr.Status.CifsError =
					STATUS_OBJECT_NAME_COLLISION;
		} else
			rsp->hdr.Status.CifsError = STATUS_DATA_ERROR;
		goto out;
	} else
		rsp->hdr.Status.CifsError = STATUS_SUCCESS;

	if (test_share_config_flag(work->tcon->share_conf,
				   KSMBD_SHARE_FLAG_STORE_DOS_ATTRS)) {
		__u64 ctime;
		struct path path;
		struct xattr_dos_attrib da = {0};

		err = ksmbd_vfs_kern_path(work, name, 0, &path, 1);
		if (!err) {
			ctime = ksmbd_UnixTimeToNT(current_time(d_inode(path.dentry)));

			da.version = 4;
			da.attr = ATTR_DIRECTORY;
			da.itime = da.create_time = ctime;
			da.flags = XATTR_DOSINFO_ATTRIB | XATTR_DOSINFO_CREATE_TIME |
				XATTR_DOSINFO_ITIME;

			err = ksmbd_vfs_set_dos_attrib_xattr(mnt_user_ns(path.mnt),
							     path.dentry, &da);
			if (err)
				ksmbd_debug(SMB, "failed to store creation time in EA\n");
			path_put(&path);
		}
		err = 0;
	}

out:
	memset(&rsp->hdr.WordCount, 0, 3);
	ksmbd_revert_fsids(work);
	kfree(name);
	return err;
}

/**
 * get_dfs_referral() - handler for smb dfs referral command
 * @work:	smb work containing get dfs referral command buffer
 *
 * Return:	0 on success, otherwise error
 */
static int get_dfs_referral(struct ksmbd_work *work)
{
	struct smb_hdr *rsp_hdr = (struct smb_hdr *)work->response_buf;

	rsp_hdr->Status.CifsError = STATUS_NOT_SUPPORTED;
	return 0;
}

/**
 * smb_trans2() - handler for trans2 commands
 * @work:	smb work containing trans2 command buffer
 *
 * Return:	0 on success, otherwise error
 */
int smb_trans2(struct ksmbd_work *work)
{
	struct smb_com_trans2_req *req = work->request_buf;
	struct smb_hdr *rsp_hdr = work->response_buf;
	int err = 0;
	u16 sub_command = le16_to_cpu(req->SubCommand);

	/* at least one setup word for TRANS2 command
	 *		MS-CIFS, SMB COM TRANSACTION
	 */
	if (req->SetupCount < 1) {
		pr_err("Wrong setup count in SMB_TRANS2 - indicates wrong request\n");
		rsp_hdr->Status.CifsError = STATUS_UNSUCCESSFUL;
		return -EINVAL;
	}

	switch (sub_command) {
	case TRANS2_FIND_FIRST:
		err = find_first(work);
		break;
	case TRANS2_FIND_NEXT:
		err = find_next(work);
		break;
	case TRANS2_QUERY_FS_INFORMATION:
		err = query_fs_info(work);
		break;
	case TRANS2_QUERY_PATH_INFORMATION:
		err = query_path_info(work);
		break;
	case TRANS2_SET_PATH_INFORMATION:
		err = set_path_info(work);
		break;
	case TRANS2_SET_FS_INFORMATION:
		err = set_fs_info(work);
		break;
	case TRANS2_QUERY_FILE_INFORMATION:
		err = query_file_info(work);
		break;
	case TRANS2_SET_FILE_INFORMATION:
		err = set_file_info(work);
		break;
	case TRANS2_CREATE_DIRECTORY:
		err = create_dir(work);
		break;
	case TRANS2_GET_DFS_REFERRAL:
		err = get_dfs_referral(work);
		break;
	default:
		ksmbd_debug(SMB, "sub command 0x%x not implemented yet\n",
				sub_command);
		rsp_hdr->Status.CifsError = STATUS_NOT_SUPPORTED;
		return -EINVAL;
	}

	if (err) {
		ksmbd_debug(SMB, "%s failed with error %d\n", __func__, err);
		if (err == -EBUSY)
			rsp_hdr->Status.CifsError = STATUS_DELETE_PENDING;
		return err;
	}

	return 0;
}

/**
 * smb_mkdir() - handler for smb mkdir
 * @work:	smb work containing creat directory command buffer
 *
 * Return:	0 on success, otherwise error
 */
int smb_mkdir(struct ksmbd_work *work)
{
	struct smb_com_create_directory_req *req = work->request_buf;
	struct smb_com_create_directory_rsp *rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	mode_t mode = S_IALLUGO;
	char *name;
	int err;

	if (!test_tree_conn_flag(work->tcon, KSMBD_TREE_CONN_FLAG_WRITABLE)) {
		ksmbd_debug(SMB,
			"returning as user does not have permission to write\n");
		rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
		return -EACCES;
	}

	name = smb_get_name(share, req->DirName, PATH_MAX, work, false);
	if (IS_ERR(name)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(name);
	}

	if (ksmbd_override_fsids(work)) {
		kfree(name);
		rsp->hdr.Status.CifsError = STATUS_NO_MEMORY;
		return -ENOMEM;
	}

	err = ksmbd_vfs_mkdir(work, name, mode);
	if (err) {
		if (err == -EEXIST) {
			if (!(((struct smb_hdr *)work->request_buf)->Flags2 &
						SMBFLG2_ERR_STATUS)) {
				rsp->hdr.Status.DosError.ErrorClass = ERRDOS;
				rsp->hdr.Status.DosError.Error =
					cpu_to_le16(ERRnoaccess);
			} else
				rsp->hdr.Status.CifsError =
					STATUS_OBJECT_NAME_COLLISION;
		} else
			rsp->hdr.Status.CifsError = STATUS_DATA_ERROR;
		goto out;
	} else {
		/* mkdir success, return response to server */
		rsp->hdr.Status.CifsError = STATUS_SUCCESS;
		rsp->hdr.WordCount = 0;
		rsp->ByteCount = 0;
	}

	if (test_share_config_flag(work->tcon->share_conf,
				   KSMBD_SHARE_FLAG_STORE_DOS_ATTRS)) {
		__u64 ctime;
		struct path path;
		struct xattr_dos_attrib da = {0};

		err = ksmbd_vfs_kern_path(work, name, 0, &path, 1);
		if (!err) {
			ctime = ksmbd_UnixTimeToNT(current_time(d_inode(path.dentry)));

			da.version = 4;
			da.attr = ATTR_DIRECTORY;
			da.itime = da.create_time = ctime;
			da.flags = XATTR_DOSINFO_ATTRIB | XATTR_DOSINFO_CREATE_TIME |
				XATTR_DOSINFO_ITIME;

			err = ksmbd_vfs_set_dos_attrib_xattr(mnt_user_ns(path.mnt),
							     path.dentry, &da);
			if (err)
				ksmbd_debug(SMB, "failed to store creation time in xattr\n");
			path_put(&path);
		}
		err = 0;
	}

out:
	ksmbd_revert_fsids(work);
	kfree(name);
	return err;
}

/**
 * smb_checkdir() - handler to verify whether a specified
 * path resolves to a valid directory or not
 *
 * @work:   smb work containing creat directory command buffer
 *
 * Return:      0 on success, otherwise error
 */
int smb_checkdir(struct ksmbd_work *work)
{
	struct smb_com_check_directory_req *req = work->request_buf;
	struct smb_com_check_directory_rsp *rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct path path;
	struct kstat stat;
	char *name, *last;
	int err;
	bool caseless_lookup = req->hdr.Flags & SMBFLG_CASELESS;

	name = smb_get_name(share, req->DirName, PATH_MAX, work, false);
	if (IS_ERR(name)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(name);
	}

	err = ksmbd_vfs_kern_path(work, name, LOOKUP_NO_SYMLINKS, &path,
				  caseless_lookup);
	if (err) {
		if (err == -ENOENT) {
			/*
			 * If the parent directory is valid but not the
			 * last component - then returns
			 * STATUS_OBJECT_NAME_NOT_FOUND
			 * for that case and STATUS_OBJECT_PATH_NOT_FOUND
			 * if the path is invalid.
			 */
			last = strrchr(name, '/');
			if (last && last[1] != '\0') {
				*last = '\0';
				last++;

				err = ksmbd_vfs_kern_path(work, name, LOOKUP_FOLLOW |
						LOOKUP_DIRECTORY, &path,
						caseless_lookup);
			} else {
				ksmbd_debug(SMB, "can't lookup parent %s\n",
					name);
				err = -ENOENT;
			}
		}
		if (err) {
			ksmbd_debug(SMB, "look up failed err %d\n", err);
			switch (err) {
			case -ENOENT:
				rsp->hdr.Status.CifsError =
				STATUS_OBJECT_NAME_NOT_FOUND;
				break;
			case -ENOMEM:
				rsp->hdr.Status.CifsError =
				STATUS_INSUFFICIENT_RESOURCES;
				break;
			case -EACCES:
				rsp->hdr.Status.CifsError =
				STATUS_ACCESS_DENIED;
				break;
			case -EIO:
				rsp->hdr.Status.CifsError =
				STATUS_DATA_ERROR;
				break;
			default:
				rsp->hdr.Status.CifsError =
				STATUS_OBJECT_PATH_SYNTAX_BAD;
				break;
			}
			kfree(name);
			return err;
		}
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
	generic_fillattr(mnt_user_ns(path.mnt), d_inode(path.dentry), &stat);
#else
	generic_fillattr(d_inode(path.dentry), &stat);
#endif

	if (!S_ISDIR(stat.mode)) {
		rsp->hdr.Status.CifsError = STATUS_NOT_A_DIRECTORY;
	} else {
		/* checkdir success, return response to server */
		rsp->hdr.Status.CifsError = STATUS_SUCCESS;
		rsp->hdr.WordCount = 0;
		rsp->ByteCount = 0;
	}

	path_put(&path);
	kfree(name);
	return err;
}

/**
 * smb_process_exit() - handler for smb process exit
 * @work:	smb work containing process exit command buffer
 *
 * Return:	0 on success always
 * This command is obsolete now. Starting with the LAN Manager 1.0 dialect,
 * FIDs are no longer associated with PIDs.CIFS clients SHOULD NOT send
 * SMB_COM_PROCESS_EXIT requests. Instead, CIFS clients SHOULD perform all
 * process cleanup operations, sending individual file close operations
 * as needed.Here it is implemented very minimally for sake
 * of passing smbtorture testcases.
 */
int smb_process_exit(struct ksmbd_work *work)
{
	struct smb_com_process_exit_rsp *rsp = work->response_buf;

	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 0;
	rsp->ByteCount = 0;
	return 0;
}

/**
 * smb_rmdir() - handler for smb rmdir
 * @work:	smb work containing delete directory command buffer
 *
 * Return:	0 on success, otherwise error
 */
int smb_rmdir(struct ksmbd_work *work)
{
	struct smb_com_delete_directory_req *req = work->request_buf;
	struct smb_com_delete_directory_rsp *rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	char *name;
	int err;

	if (!test_tree_conn_flag(work->tcon, KSMBD_TREE_CONN_FLAG_WRITABLE)) {
		ksmbd_debug(SMB,
			"returning as user does not have permission to write\n");
		rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
		return -EACCES;
	}

	name = smb_get_name(share, req->DirName, PATH_MAX, work, false);
	if (IS_ERR(name)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(name);
	}

	err = ksmbd_vfs_remove_file(work, name);
	if (err) {
		if (err == -ENOTEMPTY)
			rsp->hdr.Status.CifsError =
				STATUS_DIRECTORY_NOT_EMPTY;
		else if (err == -ENOENT)
			rsp->hdr.Status.CifsError =
				STATUS_OBJECT_NAME_NOT_FOUND;
		else
			rsp->hdr.Status.CifsError = STATUS_DATA_ERROR;
	} else {
		/* rmdir success, return response to server */
		rsp->hdr.Status.CifsError = STATUS_SUCCESS;
		rsp->hdr.WordCount = 0;
		rsp->ByteCount = 0;
	}

	kfree(name);
	return err;
}

/**
 * smb_unlink() - handler for smb delete file
 * @work:	smb work containing delete file command buffer
 *
 * Return:	0 on success, otherwise error
 */
int smb_unlink(struct ksmbd_work *work)
{
	struct smb_com_delete_file_req *req = work->request_buf;
	struct smb_com_delete_file_rsp *rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	char *name;
	int err;
	struct ksmbd_file *fp;

	if (!test_tree_conn_flag(work->tcon, KSMBD_TREE_CONN_FLAG_WRITABLE)) {
		ksmbd_debug(SMB,
			"returning as user does not have permission to write\n");
		rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
		return -EACCES;
	}

	name = smb_get_name(share, req->fileName, PATH_MAX, work, false);
	if (IS_ERR(name)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(name);
	}

	fp = ksmbd_lookup_fd_filename(work, name);
	if (fp)
		err = -ESHARE;
	else
		err = ksmbd_vfs_remove_file(work, name);

	if (err) {
		if (err == -EISDIR)
			rsp->hdr.Status.CifsError =
				STATUS_FILE_IS_A_DIRECTORY;
		else if (err == -ESHARE)
			rsp->hdr.Status.CifsError = STATUS_SHARING_VIOLATION;
		else if (err == -EACCES || err == -EXDEV)
			rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
		else
			rsp->hdr.Status.CifsError =
				STATUS_OBJECT_NAME_NOT_FOUND;
	} else {
		rsp->hdr.Status.CifsError = STATUS_SUCCESS;
		rsp->hdr.WordCount = 0;
		rsp->ByteCount = 0;
	}

	ksmbd_fd_put(work, fp);
	kfree(name);
	return err;
}

/**
 * smb_nt_cancel() - handler for smb cancel command
 * @work:	smb work containing cancel command buffer
 *
 * Return:	0
 */
int smb_nt_cancel(struct ksmbd_work *work)
{
	struct ksmbd_conn *conn = work->conn;
	struct smb_hdr *hdr = (struct smb_hdr *)work->request_buf;
	struct smb_hdr *work_hdr;
	struct ksmbd_work *new_work;

	ksmbd_debug(SMB, "smb cancel called on mid %u\n", hdr->Mid);

	spin_lock(&conn->request_lock);
	list_for_each_entry(new_work, &conn->requests, request_entry) {
		work_hdr = (struct smb_hdr *)new_work->request_buf;
		if (work_hdr->Mid == hdr->Mid) {
			ksmbd_debug(SMB, "smb with mid %u cancelled command = 0x%x\n",
			       hdr->Mid, work_hdr->Command);
			new_work->send_no_response = 1;
			list_del_init(&new_work->request_entry);
			new_work->sess->sequence_number--;
			break;
		}
	}
	spin_unlock(&conn->request_lock);

	/* For SMB_COM_NT_CANCEL command itself send no response */
	work->send_no_response = 1;
	return 0;
}

/**
 * smb_nt_rename() - handler for smb rename command
 * @work:	smb work containing nt rename command buffer
 *
 * Return:	0 on success, otherwise error
 */
int smb_nt_rename(struct ksmbd_work *work)
{
	struct smb_com_nt_rename_req *req = work->request_buf;
	struct smb_com_rename_rsp *rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	char *oldname, *newname;
	int oldname_len, err;

	if (!test_tree_conn_flag(work->tcon, KSMBD_TREE_CONN_FLAG_WRITABLE)) {
		ksmbd_debug(SMB,
			"returning as user does not have permission to write\n");
		rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
		return -EACCES;
	}

	if (le16_to_cpu(req->Flags) != CREATE_HARD_LINK) {
		rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
		return -EINVAL;
	}

	oldname = smb_get_name(share, req->OldFileName, PATH_MAX, work, false);
	if (IS_ERR(oldname)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(oldname);
	}

	if (is_smbreq_unicode(&req->hdr))
		oldname_len = smb1_utf16_name_length((__le16 *)req->OldFileName,
				PATH_MAX);
	else {
		oldname_len = strlen(oldname);
		oldname_len++;
	}

	newname = smb_get_name(share, &req->OldFileName[oldname_len + 2],
			PATH_MAX, work, false);
	if (IS_ERR(newname)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		kfree(oldname);
		return PTR_ERR(newname);
	}
	ksmbd_debug(SMB, "oldname %s, newname %s, oldname_len %d, unicode %d\n",
			oldname, newname, oldname_len,
			is_smbreq_unicode(&req->hdr));

	err = ksmbd_vfs_link(work, oldname, newname);
	if (err == -EACCES)
		rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
	else if (err < 0)
		rsp->hdr.Status.CifsError = STATUS_NOT_SAME_DEVICE;

	kfree(newname);
	kfree(oldname);
	return err;
}

static __le32 smb_query_info_pipe(struct ksmbd_share_config *share,
		struct kstat *st)
{
	st->mode = S_IFDIR;
	return 0;
}

static __le32 smb_query_info_path(struct ksmbd_work *work, struct kstat *st)
{
	struct smb_com_query_information_req *req = work->request_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct path path;
	char *name;
	__le32 err = 0;
	int ret;

	name = smb_get_name(share, req->FileName, PATH_MAX, work, false);
	if (IS_ERR(name))
		return STATUS_OBJECT_NAME_INVALID;

	if (ksmbd_override_fsids(work)) {
		kfree(name);
		return STATUS_NO_MEMORY;
	}

	ret = ksmbd_vfs_kern_path(work, name, LOOKUP_NO_SYMLINKS, &path, 0);
	if (ret) {
		pr_err("look up failed err %d\n", ret);

		if (d_is_symlink(path.dentry)) {
			err = STATUS_ACCESS_DENIED;
			goto out;
		}
		err = STATUS_OBJECT_NAME_NOT_FOUND;
		goto out;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
	generic_fillattr(mnt_user_ns(path.mnt), d_inode(path.dentry), st);
#else
	generic_fillattr(d_inode(path.dentry), st);
#endif
	path_put(&path);
out:
	ksmbd_revert_fsids(work);
	kfree(name);
	return err;
}

/**
 * smb_query_info() - handler for query information command
 * @work:	smb work containing query info command buffer
 *
 * Return:	0 on success, otherwise error
 */
int smb_query_info(struct ksmbd_work *work)
{
	struct smb_com_query_information_rsp *rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct kstat st = {0,};
	__u16 attr = 0;
	int i;
	__le32 err;

	if (!test_share_config_flag(work->tcon->share_conf,
				    KSMBD_SHARE_FLAG_PIPE))
		err = smb_query_info_path(work, &st);
	else
		err = smb_query_info_pipe(share, &st);

	if (le32_to_cpu(err) != 0) {
		rsp->hdr.Status.CifsError = err;
		return -EINVAL;
	}

	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 10;

	if (st.mode & S_ISVTX)
		attr |=  (ATTR_HIDDEN | ATTR_SYSTEM);
	if (!(st.mode & 0222))
		attr |=  ATTR_READONLY;
	if (S_ISDIR(st.mode))
		attr |= ATTR_DIRECTORY;

	rsp->attr = cpu_to_le16(attr);
	rsp->last_write_time = cpu_to_le32(st.mtime.tv_sec);
	rsp->size = cpu_to_le32((u32)st.size);
	for (i = 0; i < 5; i++)
		rsp->reserved[i] = 0;

	rsp->ByteCount = 0;
	inc_rfc1001_len(&rsp->hdr, rsp->hdr.WordCount * 2);
	return 0;
}

/**
 * smb_closedir() - handler closing dir handle, opened for readdir
 * @work:	smb work containing find close command buffer
 *
 * Return:	0 on success, otherwise error
 */
int smb_closedir(struct ksmbd_work *work)
{
	struct smb_com_findclose_req *req = work->request_buf;
	struct smb_com_close_rsp *rsp = work->response_buf;
	int err;

	ksmbd_debug(SMB, "SMB_COM_FIND_CLOSE2 called for fid %u\n",
		req->FileID);

	rsp->hdr.WordCount = 0;
	rsp->ByteCount = 0;

	err = ksmbd_close_fd(work, req->FileID);
	if (!err)
		rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	else
		rsp->hdr.Status.CifsError = STATUS_INVALID_HANDLE;
	return err;
}

/**
 * convert_open_flags() - convert smb open flags to file open flags
 * @file_present:	is file already present
 * @mode:		smp file open mode
 * @disposition:	smp file disposition information
 *
 * Return:	converted file open flags
 */
static int convert_open_flags(bool file_present,
			      __u16 mode, __u16 dispostion,
			      int *may_flags)
{
	int oflags = 0;

	switch (mode & 0x0007) {
	case SMBOPEN_READ:
		oflags |= O_RDONLY;
		break;
	case SMBOPEN_WRITE:
		oflags |= O_WRONLY;
		break;
	case SMBOPEN_READWRITE:
		oflags |= O_RDWR;
		break;
	default:
		oflags |= O_RDONLY;
		break;
	}

	if (mode & SMBOPEN_WRITE_THROUGH)
		oflags |= O_SYNC;

	if (file_present) {
		switch (dispostion & 0x0003) {
		case SMBOPEN_DISPOSITION_NONE:
			return -EEXIST;
		case SMBOPEN_OAPPEND:
			oflags |= O_APPEND;
			break;
		case SMBOPEN_OTRUNC:
			oflags |= O_TRUNC;
			break;
		default:
			break;
		}
	} else {
		switch (dispostion & 0x0010) {
		case SMBOPEN_DISPOSITION_NONE:
			return -EINVAL;
		case SMBOPEN_OCREATE:
			oflags |= O_CREAT;
			break;
		default:
			break;
		}
	}

	*may_flags = ksmbd_openflags_to_mayflags(oflags);

	return oflags;
}

/**
 * smb_open_andx() - smb andx open method handler
 * @work:	smb work containing buffer for andx open command buffer
 *
 * Return:	error if there is error while processing current command,
 *		otherwise pointer to next andx command in the chain
 */
int smb_open_andx(struct ksmbd_work *work)
{
	struct smb_com_openx_req *req = work->request_buf;
	struct smb_com_openx_rsp *rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct path path;
	struct kstat stat;
	int oplock_flags, file_info, open_flags, may_flags;
	char *name;
	bool file_present = true;
	umode_t mode = 0;
	int err;
	struct ksmbd_file *fp = NULL;
	int oplock_rsp = OPLOCK_NONE, share_ret;

	rsp->hdr.Status.CifsError = STATUS_UNSUCCESSFUL;

	/* check for sharing mode flag */
	if ((le16_to_cpu(req->Mode) & SMBOPEN_SHARING_MODE) >
			SMBOPEN_DENY_NONE) {
		rsp->hdr.Status.DosError.ErrorClass = ERRDOS;
		rsp->hdr.Status.DosError.Error = cpu_to_le16(ERRbadaccess);
		rsp->hdr.Flags2 &= ~SMBFLG2_ERR_STATUS;

		memset(&rsp->hdr.WordCount, 0, 3);
		return -EINVAL;
	}

	if (is_smbreq_unicode(&req->hdr))
		name = smb_get_name(share, req->fileName + 1, PATH_MAX,
				work, false);
	else
		name = smb_get_name(share, req->fileName, PATH_MAX,
				work, false);

	if (IS_ERR(name)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(name);
	}

	if (ksmbd_override_fsids(work)) {
		kfree(name);
		rsp->hdr.Status.CifsError = STATUS_NO_MEMORY;
		return -ENOMEM;
	}

	err = ksmbd_vfs_kern_path(work, name, LOOKUP_NO_SYMLINKS, &path,
				  req->hdr.Flags & SMBFLG_CASELESS);
	if (err) {
		if (err == -EACCES || err == -EXDEV)
			goto out;
		file_present = false;
	} else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
		generic_fillattr(mnt_user_ns(path.mnt), d_inode(path.dentry), &stat);
#else
		generic_fillattr(d_inode(path.dentry), &stat);
#endif

	oplock_flags = le16_to_cpu(req->OpenFlags) &
		(REQ_OPLOCK | REQ_BATCHOPLOCK);

	open_flags = convert_open_flags(file_present,
					le16_to_cpu(req->Mode),
					le16_to_cpu(req->OpenFunction),
					&may_flags);
	if (open_flags < 0) {
		ksmbd_debug(SMB, "create_dispostion returned %d\n", open_flags);
		if (file_present)
			goto free_path;
		else {
			err = -ENOENT;
			goto out;
		}
	}

	if (file_present && !(stat.mode & 0222)) {
		if ((open_flags & O_ACCMODE) == O_WRONLY ||
				(open_flags & O_ACCMODE) == O_RDWR) {
			ksmbd_debug(SMB, "readonly file(%s)\n", name);
			rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
			memset(&rsp->hdr.WordCount, 0, 3);
			goto free_path;
		}
	}

	if (!file_present && (open_flags & O_CREAT)) {
		mode |= 0777;
		if (le16_to_cpu(req->FileAttributes) & ATTR_READONLY)
			mode &= ~0222;

		mode |= S_IFREG;
		err = ksmbd_vfs_create(work, name, mode);
		if (err)
			goto out;

		err = ksmbd_vfs_kern_path(work, name, 0, &path, 0);
		if (err) {
			pr_err("cannot get linux path, err = %d\n", err);
			goto out;
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
		generic_fillattr(mnt_user_ns(path.mnt), d_inode(path.dentry), &stat);
#else
		generic_fillattr(d_inode(path.dentry), &stat);
#endif
	} else if (file_present) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
		err = inode_permission(mnt_user_ns(path.mnt),
				       d_inode(path.dentry),
				       may_flags);
#else
		err = inode_permission(d_inode(path.dentry),
				       may_flags);
#endif
		if (err)
			goto free_path;
	}

	err = ksmbd_query_inode_status(d_inode(path.dentry->d_parent));
	if (err == KSMBD_INODE_STATUS_PENDING_DELETE) {
		err = -EBUSY;
		goto free_path;
	}

	err = 0;
	ksmbd_debug(SMB, "(%s) open_flags = 0x%x, oplock_flags 0x%x\n",
			name, open_flags, oplock_flags);
	/* open  file and get FID */
	fp = ksmbd_vfs_dentry_open(work, &path, open_flags,
			0, file_present);
	if (IS_ERR(fp)) {
		err = PTR_ERR(fp);
		fp = NULL;
		goto free_path;
	}
	fp->pid = le16_to_cpu(req->hdr.Pid);

	write_lock(&fp->f_ci->m_lock);
	list_add(&fp->node, &fp->f_ci->m_fp_list);
	write_unlock(&fp->f_ci->m_lock);

	share_ret = ksmbd_smb_check_shared_mode(fp->filp, fp);
	if (smb1_oplock_enable &&
	    test_share_config_flag(work->tcon->share_conf,
			KSMBD_SHARE_FLAG_OPLOCKS) &&
		!S_ISDIR(file_inode(fp->filp)->i_mode) &&
		oplock_flags) {
		/* Client cannot request levelII oplock directly */
		err = smb_grant_oplock(work, oplock_flags, fp->volatile_id,
			fp, le16_to_cpu(req->hdr.Tid), NULL, 0);
		if (err)
			goto free_path;
	} else {
		if (ksmbd_inode_pending_delete(fp)) {
			err = -EBUSY;
			goto free_path;
		}

		if (share_ret < 0) {
			err = -EPERM;
			goto free_path;
		}
	}

	oplock_rsp = fp->f_opinfo != NULL ? fp->f_opinfo->level : 0;

	/* open success, send back response */
	if (file_present) {
		if (!(open_flags & O_TRUNC))
			file_info = F_OPENED;
		else
			file_info = F_OVERWRITTEN;
	} else
		file_info = F_CREATED;

	if (oplock_rsp)
		file_info |= SMBOPEN_LOCK_GRANTED;

	if (stat.result_mask & STATX_BTIME)
		fp->create_time = ksmbd_UnixTimeToNT(stat.btime);
	else
		fp->create_time = ksmbd_UnixTimeToNT(stat.ctime);
	if (file_present) {
		if (test_share_config_flag(work->tcon->share_conf,
					   KSMBD_SHARE_FLAG_STORE_DOS_ATTRS)) {
			struct xattr_dos_attrib da;

			err = ksmbd_vfs_get_dos_attrib_xattr(mnt_user_ns(path.mnt),
							     path.dentry, &da);
			if (err > 0) {
				fp->create_time = da.create_time;
				fp->itime = da.itime;
			}
			err = 0;
		}
	} else {
		if (test_share_config_flag(work->tcon->share_conf,
					KSMBD_SHARE_FLAG_STORE_DOS_ATTRS)) {
			struct xattr_dos_attrib da = {0};

			da.version = 4;
			da.attr = ATTR_NORMAL;
			da.itime = da.create_time = fp->create_time;
			da.flags = XATTR_DOSINFO_ATTRIB | XATTR_DOSINFO_CREATE_TIME |
				XATTR_DOSINFO_ITIME;

			err = ksmbd_vfs_set_dos_attrib_xattr(mnt_user_ns(path.mnt),
							     path.dentry, &da);
			if (err)
				ksmbd_debug(SMB, "failed to store creation time in xattr\n");
			err = 0;
		}
	}

	/* prepare response buffer */
	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 0x0F;
	rsp->Fid = fp->volatile_id;
	rsp->FileAttributes = cpu_to_le16(ATTR_NORMAL);
	rsp->LastWriteTime = cpu_to_le32(stat.mtime.tv_sec);
	rsp->EndOfFile = cpu_to_le32(stat.size);
	switch (open_flags & O_ACCMODE) {
	case O_RDONLY:
		rsp->Access = cpu_to_le16(SMB_DA_ACCESS_READ);
		break;
	case O_WRONLY:
		rsp->Access = cpu_to_le16(SMB_DA_ACCESS_WRITE);
		break;
	case O_RDWR:
		rsp->Access = cpu_to_le16(SMB_DA_ACCESS_READ_WRITE);
		break;
	default:
		rsp->Access = cpu_to_le16(SMB_DA_ACCESS_READ);
		break;
	}

	rsp->FileType = 0;
	rsp->IPCState = 0;
	rsp->Action = cpu_to_le16(file_info);
	rsp->Reserved = 0;
	rsp->ByteCount = 0;
	inc_rfc1001_len(&rsp->hdr, rsp->hdr.WordCount * 2);

free_path:
	path_put(&path);
out:
	ksmbd_revert_fsids(work);
	if (err) {
		if (err == -ENOSPC)
			rsp->hdr.Status.CifsError = STATUS_DISK_FULL;
		else if (err == -EMFILE)
			rsp->hdr.Status.CifsError =
				STATUS_TOO_MANY_OPENED_FILES;
		else if (err == -EBUSY)
			rsp->hdr.Status.CifsError = STATUS_DELETE_PENDING;
		else if (err == -ENOENT)
			rsp->hdr.Status.CifsError =
				STATUS_OBJECT_NAME_NOT_FOUND;
		else if (err == -EACCES || err == -EXDEV)
			rsp->hdr.Status.CifsError = STATUS_ACCESS_DENIED;
		else
			rsp->hdr.Status.CifsError =
				STATUS_UNEXPECTED_IO_ERROR;
		if (fp)
			ksmbd_close_fd(work, fp->volatile_id);
	}

	kfree(name);
	if (!rsp->hdr.WordCount)
		return err;

	/* this is an ANDx command ? */
	rsp->AndXReserved = 0;
	rsp->AndXOffset = cpu_to_le16(get_rfc1002_len(&rsp->hdr));
	if (req->AndXCommand != SMB_NO_MORE_ANDX_COMMAND) {
		/* adjust response */
		rsp->AndXCommand = req->AndXCommand;
		return rsp->AndXCommand; /* More processing required */
	}
	rsp->AndXCommand = SMB_NO_MORE_ANDX_COMMAND;

	return err;
}

/**
 * smb_setattr() - set file attributes
 * @work:	smb work containing setattr command
 *
 * Return:	0 on success, otherwise error
 */
int smb_setattr(struct ksmbd_work *work)
{
	struct smb_com_setattr_req *req = work->request_buf;
	struct smb_com_setattr_rsp *rsp = work->response_buf;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct path path;
	struct kstat stat;
	struct iattr attrs;
	int err = 0;
	char *name;
	__u16 dos_attr;

	name = smb_get_name(share, req->fileName, PATH_MAX, work, false);
	if (IS_ERR(name)) {
		rsp->hdr.Status.CifsError =
			STATUS_OBJECT_NAME_INVALID;
		return PTR_ERR(name);
	}

	err = ksmbd_vfs_kern_path(work, name, LOOKUP_NO_SYMLINKS, &path,
				  req->hdr.Flags & SMBFLG_CASELESS);
	if (err) {
		ksmbd_debug(SMB, "look up failed err %d\n", err);
		rsp->hdr.Status.CifsError = STATUS_OBJECT_NAME_NOT_FOUND;
		err = 0;
		goto out;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
	generic_fillattr(mnt_user_ns(path.mnt), d_inode(path.dentry), &stat);
#else
	generic_fillattr(d_inode(path.dentry), &stat);
#endif
	path_put(&path);
	attrs.ia_valid = 0;
	attrs.ia_mode = 0;

	dos_attr = le16_to_cpu(req->attr);
	if (!dos_attr)
		attrs.ia_mode = stat.mode | 0200;

	if (dos_attr & ATTR_READONLY)
		attrs.ia_mode = stat.mode & ~0222;

	if (attrs.ia_mode)
		attrs.ia_valid |= ATTR_MODE;

	attrs.ia_mtime.tv_sec = le32_to_cpu(req->LastWriteTime);
	attrs.ia_valid |= (ATTR_MTIME | ATTR_MTIME_SET);

	err = ksmbd_vfs_setattr(work, name, 0, &attrs);
	if (err)
		goto out;

	rsp->hdr.Status.CifsError = STATUS_SUCCESS;
	rsp->hdr.WordCount = 0;
	rsp->ByteCount = 0;

out:
	kfree(name);
	if (err) {
		rsp->hdr.Status.CifsError = STATUS_INVALID_PARAMETER;
		return err;
	}

	return 0;
}

/**
 * smb1_is_sign_req() - handler for checking packet signing status
 * @work:	smb work containing notify command buffer
 *
 * Return:	true if packed is signed, false otherwise
 */
bool smb1_is_sign_req(struct ksmbd_work *work, unsigned int command)
{
#if 0
	struct smb_hdr *rcv_hdr1 = (struct smb_hdr *)work->request_buf;

	/*
	 * FIXME: signed tree connect failed by signing error
	 * with windows XP client. For now, Force to turn off
	 * signing feature in SMB1.
	 */
	if ((rcv_hdr1->Flags2 & SMBFLG2_SECURITY_SIGNATURE) &&
			command != SMB_COM_SESSION_SETUP_ANDX)
		return true;
	return false;
#else
	return false;
#endif
}

/**
 * smb1_check_sign_req() - handler for req packet sign processing
 * @work:	smb work containing notify command buffer
 *
 * Return:	1 on success, 0 otherwise
 */
int smb1_check_sign_req(struct ksmbd_work *work)
{
	struct smb_hdr *rcv_hdr1 = (struct smb_hdr *)work->request_buf;
	char signature_req[CIFS_SMB1_SIGNATURE_SIZE];
	char signature[20];
	struct kvec iov[1];

	memcpy(signature_req, rcv_hdr1->Signature.SecuritySignature,
			CIFS_SMB1_SIGNATURE_SIZE);
	rcv_hdr1->Signature.Sequence.SequenceNumber =
		cpu_to_le32(++work->sess->sequence_number);
	rcv_hdr1->Signature.Sequence.Reserved = 0;

	iov[0].iov_base = rcv_hdr1->Protocol;
	iov[0].iov_len = be32_to_cpu(rcv_hdr1->smb_buf_length);

	if (ksmbd_sign_smb1_pdu(work->sess, iov, 1, signature))
		return 0;

	if (memcmp(signature, signature_req, CIFS_SMB1_SIGNATURE_SIZE)) {
		ksmbd_debug(SMB, "bad smb1 sign\n");
		return 0;
	}

	return 1;
}

/**
 * smb1_set_sign_rsp() - handler for rsp packet sign processing
 * @work:	smb work containing notify command buffer
 *
 */
void smb1_set_sign_rsp(struct ksmbd_work *work)
{
	struct smb_hdr *rsp_hdr = (struct smb_hdr *)work->response_buf;
	char signature[20];
	struct kvec iov[2];
	int n_vec = 1;

	rsp_hdr->Flags2 |= SMBFLG2_SECURITY_SIGNATURE;
	rsp_hdr->Signature.Sequence.SequenceNumber =
		cpu_to_le32(++work->sess->sequence_number);
	rsp_hdr->Signature.Sequence.Reserved = 0;

	iov[0].iov_base = rsp_hdr->Protocol;
	iov[0].iov_len = be32_to_cpu(rsp_hdr->smb_buf_length);

	if (work->aux_payload_sz) {
		iov[0].iov_len -= work->aux_payload_sz;

		iov[1].iov_base = work->aux_payload_buf;
		iov[1].iov_len = work->aux_payload_sz;
		n_vec++;
	}

	if (ksmbd_sign_smb1_pdu(work->sess, iov, n_vec, signature))
		memset(rsp_hdr->Signature.SecuritySignature,
				0, CIFS_SMB1_SIGNATURE_SIZE);
	else
		memcpy(rsp_hdr->Signature.SecuritySignature,
				signature, CIFS_SMB1_SIGNATURE_SIZE);
}
