// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *   Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *   Copyright (C) 2018 Namjae Jeon <linkinjeon@kernel.org>
 */

#include <linux/user_namespace.h>

#include "smb_common.h"
#include "smb1pdu.h"
#include "server.h"
#include "misc.h"
#include "smbstatus.h"
#include "connection.h"
#include "ksmbd_work.h"
#include "mgmt/user_session.h"
#include "mgmt/user_config.h"
#include "mgmt/tree_connect.h"
#include "mgmt/share_config.h"

/*for shortname implementation */
static const char basechars[43] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ_-!@#$%";
#define MANGLE_BASE (sizeof(basechars) / sizeof(char) - 1)
#define MAGIC_CHAR '~'
#define PERIOD '.'
#define mangle(V) ((char)(basechars[(V) % MANGLE_BASE]))

#ifdef CONFIG_SMB_INSECURE_SERVER
#define KSMBD_MIN_SUPPORTED_HEADER_SIZE	(sizeof(struct smb_hdr))
#else
#define KSMBD_MIN_SUPPORTED_HEADER_SIZE	(sizeof(struct smb2_hdr))
#endif

struct smb_protocol {
	int		index;
	char		*name;
	char		*prot;
	__u16		prot_id;
};

static struct smb_protocol smb1_protos[] = {
#ifdef CONFIG_SMB_INSECURE_SERVER
	{
		SMB1_PROT,
		"\2NT LM 0.12",
		"NT1",
		SMB10_PROT_ID
	},
	{
		SMB2_PROT,
		"\2SMB 2.002",
		"SMB2_02",
		SMB20_PROT_ID
	},
#endif
	{
		SMB2X_PROT,
		"\2SMB 2.???",
		"SMB2_22",
		SMB2X_PROT_ID
	},
};

static struct smb_protocol smb2_protos[] = {
	{
		SMB2_PROT,
		"\2SMB 2.002",
		"SMB2_02",
		SMB20_PROT_ID
	},
	{
		SMB21_PROT,
		"\2SMB 2.1",
		"SMB2_10",
		SMB21_PROT_ID
	},
	{
		SMB30_PROT,
		"\2SMB 3.0",
		"SMB3_00",
		SMB30_PROT_ID
	},
	{
		SMB302_PROT,
		"\2SMB 3.02",
		"SMB3_02",
		SMB302_PROT_ID
	},
	{
		SMB311_PROT,
		"\2SMB 3.1.1",
		"SMB3_11",
		SMB311_PROT_ID
	},
};

unsigned int ksmbd_server_side_copy_max_chunk_count(void)
{
	return 256;
}

unsigned int ksmbd_server_side_copy_max_chunk_size(void)
{
	return (2U << 30) - 1;
}

unsigned int ksmbd_server_side_copy_max_total_size(void)
{
	return (2U << 30) - 1;
}

inline int ksmbd_min_protocol(void)
{
#ifdef CONFIG_SMB_INSECURE_SERVER
	return SMB1_PROT;
#else
	return SMB2_PROT;
#endif
}

inline int ksmbd_max_protocol(void)
{
	return SMB311_PROT;
}

int ksmbd_lookup_protocol_idx(char *str)
{
	int offt = ARRAY_SIZE(smb1_protos) - 1;
	int len = strlen(str);

	while (offt >= 0) {
		if (!strncmp(str, smb1_protos[offt].prot, len)) {
			ksmbd_debug(SMB, "selected %s dialect idx = %d\n",
				    smb1_protos[offt].prot, offt);
			return smb1_protos[offt].index;
		}
		offt--;
	}

	offt = ARRAY_SIZE(smb2_protos) - 1;
	while (offt >= 0) {
		if (!strncmp(str, smb2_protos[offt].prot, len)) {
			ksmbd_debug(SMB, "selected %s dialect idx = %d\n",
				    smb2_protos[offt].prot, offt);
			return smb2_protos[offt].index;
		}
		offt--;
	}
	return -1;
}

/**
 * ksmbd_verify_smb_message() - check for valid smb2 request header
 * @work:	smb work
 *
 * check for valid smb signature and packet direction(request/response)
 *
 * Return:      0 on success, otherwise error
 */
int ksmbd_verify_smb_message(struct ksmbd_work *work)
{
	struct smb2_hdr *smb2_hdr = ksmbd_req_buf_next(work);

#ifdef CONFIG_SMB_INSECURE_SERVER
	if (smb2_hdr->ProtocolId == SMB2_PROTO_NUMBER) {
		ksmbd_debug(SMB, "got SMB2 command\n");
		return ksmbd_smb2_check_message(work);
	}

	work->conn->outstanding_credits++;
	return ksmbd_smb1_check_message(work);
#else
	struct smb_hdr *hdr;

	if (smb2_hdr->ProtocolId == SMB2_PROTO_NUMBER)
		return ksmbd_smb2_check_message(work);

	hdr = work->request_buf;
	if (*(__le32 *)hdr->Protocol == SMB1_PROTO_NUMBER &&
	    hdr->Command == SMB_COM_NEGOTIATE) {
		work->conn->outstanding_credits++;
		return 0;
	}

	return -EINVAL;
#endif
}

/**
 * ksmbd_smb_request() - check for valid smb request type
 * @conn:	connection instance
 *
 * Return:      true on success, otherwise false
 */
bool ksmbd_smb_request(struct ksmbd_conn *conn)
{
	return conn->request_buf[0] == 0;
}

static bool supported_protocol(int idx)
{
	if (idx == SMB2X_PROT &&
	    (server_conf.min_protocol >= SMB21_PROT ||
	     server_conf.max_protocol <= SMB311_PROT))
		return true;

	return (server_conf.min_protocol <= idx &&
		idx <= server_conf.max_protocol);
}

static char *next_dialect(char *dialect, int *next_off, int bcount)
{
	dialect = dialect + *next_off;
	*next_off = strnlen(dialect, bcount);
	if (dialect[*next_off] != '\0')
		return NULL;
	return dialect;
}

static int ksmbd_lookup_dialect_by_name(char *cli_dialects, __le16 byte_count)
{
	int i, seq_num, bcount, next;
	char *dialect;

	for (i = ARRAY_SIZE(smb1_protos) - 1; i >= 0; i--) {
		seq_num = 0;
		next = 0;
		dialect = cli_dialects;
		bcount = le16_to_cpu(byte_count);
		do {
			dialect = next_dialect(dialect, &next, bcount);
			if (!dialect)
				break;
			ksmbd_debug(SMB, "client requested dialect %s\n",
				    dialect);
			if (!strcmp(dialect, smb1_protos[i].name)) {
				if (supported_protocol(smb1_protos[i].index)) {
					ksmbd_debug(SMB,
						    "selected %s dialect\n",
						    smb1_protos[i].name);
					if (smb1_protos[i].index == SMB1_PROT)
						return seq_num;
					return smb1_protos[i].prot_id;
				}
			}
			seq_num++;
			bcount -= (++next);
		} while (bcount > 0);
	}

	return BAD_PROT_ID;
}

int ksmbd_lookup_dialect_by_id(__le16 *cli_dialects, __le16 dialects_count)
{
	int i;
	int count;

	for (i = ARRAY_SIZE(smb2_protos) - 1; i >= 0; i--) {
		count = le16_to_cpu(dialects_count);
		while (--count >= 0) {
			ksmbd_debug(SMB, "client requested dialect 0x%x\n",
				    le16_to_cpu(cli_dialects[count]));
			if (le16_to_cpu(cli_dialects[count]) !=
					smb2_protos[i].prot_id)
				continue;

			if (supported_protocol(smb2_protos[i].index)) {
				ksmbd_debug(SMB, "selected %s dialect\n",
					    smb2_protos[i].name);
				return smb2_protos[i].prot_id;
			}
		}
	}

	return BAD_PROT_ID;
}

static int ksmbd_negotiate_smb_dialect(void *buf)
{
	int smb_buf_length = get_rfc1002_len(buf);
	__le32 proto = ((struct smb2_hdr *)smb2_get_msg(buf))->ProtocolId;

	if (proto == SMB2_PROTO_NUMBER) {
		struct smb2_negotiate_req *req;
		int smb2_neg_size =
			offsetof(struct smb2_negotiate_req, Dialects);

		req = (struct smb2_negotiate_req *)smb2_get_msg(buf);
		if (smb2_neg_size > smb_buf_length)
			goto err_out;

		if (smb2_neg_size + le16_to_cpu(req->DialectCount) * sizeof(__le16) >
		    smb_buf_length)
			goto err_out;

		return ksmbd_lookup_dialect_by_id(req->Dialects,
						  req->DialectCount);
	}

	proto = *(__le32 *)((struct smb_hdr *)buf)->Protocol;
	if (proto == SMB1_PROTO_NUMBER) {
		struct smb_negotiate_req *req;

		req = (struct smb_negotiate_req *)buf;
		if (le16_to_cpu(req->ByteCount) < 2)
			goto err_out;

		if (offsetof(struct smb_negotiate_req, DialectsArray) - 4 +
			le16_to_cpu(req->ByteCount) > smb_buf_length) {
			goto err_out;
		}

		return ksmbd_lookup_dialect_by_name(req->DialectsArray,
						    req->ByteCount);
	}

err_out:
	return BAD_PROT_ID;
}

#ifndef CONFIG_SMB_INSECURE_SERVER
#define SMB_COM_NEGOTIATE_EX	0x0

/**
 * get_smb1_cmd_val() - get smb command value from smb header
 * @work:	smb work containing smb header
 *
 * Return:      smb command value
 */
static u16 get_smb1_cmd_val(struct ksmbd_work *work)
{
	return SMB_COM_NEGOTIATE_EX;
}

/**
 * init_smb1_rsp_hdr() - initialize smb negotiate response header
 * @work:	smb work containing smb request
 *
 * Return:      0 on success, otherwise -EINVAL
 */
static int init_smb1_rsp_hdr(struct ksmbd_work *work)
{
	struct smb_hdr *rsp_hdr = (struct smb_hdr *)work->response_buf;
	struct smb_hdr *rcv_hdr = (struct smb_hdr *)work->request_buf;

	/*
	 * Remove 4 byte direct TCP header.
	 */
	*(__be32 *)work->response_buf =
		cpu_to_be32(sizeof(struct smb_hdr) - 4);

	rsp_hdr->Command = SMB_COM_NEGOTIATE;
	*(__le32 *)rsp_hdr->Protocol = SMB1_PROTO_NUMBER;
	rsp_hdr->Flags = SMBFLG_RESPONSE;
	rsp_hdr->Flags2 = SMBFLG2_UNICODE | SMBFLG2_ERR_STATUS |
		SMBFLG2_EXT_SEC | SMBFLG2_IS_LONG_NAME;
	rsp_hdr->Pid = rcv_hdr->Pid;
	rsp_hdr->Mid = rcv_hdr->Mid;
	return 0;
}

/**
 * smb1_check_user_session() - check for valid session for a user
 * @work:	smb work containing smb request buffer
 *
 * Return:      0 on success, otherwise error
 */
static int smb1_check_user_session(struct ksmbd_work *work)
{
	unsigned int cmd = work->conn->ops->get_cmd_val(work);

	if (cmd == SMB_COM_NEGOTIATE_EX)
		return 0;

	return -EINVAL;
}

/**
 * smb1_allocate_rsp_buf() - allocate response buffer for a command
 * @work:	smb work containing smb request
 *
 * Return:      0 on success, otherwise -ENOMEM
 */
static int smb1_allocate_rsp_buf(struct ksmbd_work *work)
{
	work->response_buf = kzalloc(MAX_CIFS_SMALL_BUFFER_SIZE,
			GFP_KERNEL);
	work->response_sz = MAX_CIFS_SMALL_BUFFER_SIZE;

	if (!work->response_buf) {
		pr_err("Failed to allocate %u bytes buffer\n",
				MAX_CIFS_SMALL_BUFFER_SIZE);
		return -ENOMEM;
	}

	return 0;
}

static struct smb_version_ops smb1_server_ops = {
	.get_cmd_val = get_smb1_cmd_val,
	.init_rsp_hdr = init_smb1_rsp_hdr,
	.allocate_rsp_buf = smb1_allocate_rsp_buf,
	.check_user_session = smb1_check_user_session,
};

static int smb1_negotiate(struct ksmbd_work *work)
{
	return ksmbd_smb_negotiate_common(work, SMB_COM_NEGOTIATE);
}

static struct smb_version_cmds smb1_server_cmds[1] = {
	[SMB_COM_NEGOTIATE_EX]	= { .proc = smb1_negotiate, },
};

static void init_smb1_server(struct ksmbd_conn *conn)
{
	conn->ops = &smb1_server_ops;
	conn->cmds = smb1_server_cmds;
	conn->max_cmds = ARRAY_SIZE(smb1_server_cmds);
}
#endif

void ksmbd_init_smb_server(struct ksmbd_work *work)
{
	struct ksmbd_conn *conn = work->conn;
	__le32 proto;

	if (conn->need_neg == false)
		return;

	proto = *(__le32 *)((struct smb_hdr *)work->request_buf)->Protocol;
	if (proto == SMB1_PROTO_NUMBER)
		init_smb1_server(conn);
	else
		init_smb3_11_server(conn);
}

int ksmbd_populate_dot_dotdot_entries(struct ksmbd_work *work, int info_level,
				      struct ksmbd_file *dir,
				      struct ksmbd_dir_info *d_info,
				      char *search_pattern,
				      int (*fn)(struct ksmbd_conn *, int,
						struct ksmbd_dir_info *,
						struct ksmbd_kstat *))
{
	int i, rc = 0;
	struct ksmbd_conn *conn = work->conn;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	struct mnt_idmap *idmap = file_mnt_idmap(dir->filp);
#else
	struct user_namespace *user_ns = file_mnt_user_ns(dir->filp);
#endif

	for (i = 0; i < 2; i++) {
		struct kstat kstat;
		struct ksmbd_kstat ksmbd_kstat;
		struct dentry *dentry;

		if (!dir->dot_dotdot[i]) { /* fill dot entry info */
			if (i == 0) {
				d_info->name = ".";
				d_info->name_len = 1;
				dentry = dir->filp->f_path.dentry;
			} else {
				d_info->name = "..";
				d_info->name_len = 2;
				dentry = dir->filp->f_path.dentry->d_parent;
			}

			if (!match_pattern(d_info->name, d_info->name_len,
					   search_pattern)) {
				dir->dot_dotdot[i] = 1;
				continue;
			}

			ksmbd_kstat.kstat = &kstat;
			ksmbd_vfs_fill_dentry_attrs(work,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
						    idmap,
#else
						    user_ns,
#endif
						    dentry,
						    &ksmbd_kstat);
			rc = fn(conn, info_level, d_info, &ksmbd_kstat);
			if (rc)
				break;
			if (d_info->out_buf_len <= 0)
				break;

			dir->dot_dotdot[i] = 1;
			if (d_info->flags & SMB2_RETURN_SINGLE_ENTRY) {
				d_info->out_buf_len = 0;
				break;
			}
		}
	}

	return rc;
}

/**
 * ksmbd_extract_shortname() - get shortname from long filename
 * @conn:	connection instance
 * @longname:	source long filename
 * @shortname:	destination short filename
 *
 * Return:	shortname length or 0 when source long name is '.' or '..'
 * TODO: Though this function comforms the restriction of 8.3 Filename spec,
 * but the result is different with Windows 7's one. need to check.
 */
int ksmbd_extract_shortname(struct ksmbd_conn *conn, const char *longname,
			    char *shortname)
{
	const char *p;
	char base[9], extension[4];
	char out[13] = {0};
	int baselen = 0;
	int extlen = 0, len = 0;
	unsigned int csum = 0;
	const unsigned char *ptr;
	bool dot_present = true;

	p = longname;
	if ((*p == '.') || (!(strcmp(p, "..")))) {
		/*no mangling required */
		return 0;
	}

	p = strrchr(longname, '.');
	if (p == longname) { /*name starts with a dot*/
		strscpy(extension, "___", strlen("___"));
	} else {
		if (p) {
			p++;
			while (*p && extlen < 3) {
				if (*p != '.')
					extension[extlen++] = toupper(*p);
				p++;
			}
			extension[extlen] = '\0';
		} else {
			dot_present = false;
		}
	}

	p = longname;
	if (*p == '.') {
		p++;
		longname++;
	}
	while (*p && (baselen < 5)) {
		if (*p != '.')
			base[baselen++] = toupper(*p);
		p++;
	}

	base[baselen] = MAGIC_CHAR;
	memcpy(out, base, baselen + 1);

	ptr = longname;
	len = strlen(longname);
	for (; len > 0; len--, ptr++)
		csum += *ptr;

	csum = csum % (MANGLE_BASE * MANGLE_BASE);
	out[baselen + 1] = mangle(csum / MANGLE_BASE);
	out[baselen + 2] = mangle(csum);
	out[baselen + 3] = PERIOD;

	if (dot_present)
		memcpy(&out[baselen + 4], extension, 4);
	else
		out[baselen + 4] = '\0';
	smbConvertToUTF16((__le16 *)shortname, out, PATH_MAX,
			  conn->local_nls, 0);
	len = strlen(out) * 2;
	return len;
}

static int __smb2_negotiate(struct ksmbd_conn *conn)
{
	return (conn->dialect >= SMB20_PROT_ID &&
		conn->dialect <= SMB311_PROT_ID);
}

#ifndef CONFIG_SMB_INSECURE_SERVER
static int smb_handle_negotiate(struct ksmbd_work *work)
{
	struct smb_negotiate_rsp *neg_rsp = work->response_buf;

	ksmbd_debug(SMB, "Unsupported SMB1 protocol\n");

	/* Add 2 byte bcc and 2 byte DialectIndex. */
	inc_rfc1001_len(work->response_buf, 4);
	neg_rsp->hdr.Status.CifsError = STATUS_SUCCESS;

	neg_rsp->hdr.WordCount = 1;
	neg_rsp->DialectIndex = cpu_to_le16(work->conn->dialect);
	neg_rsp->ByteCount = 0;
	return 0;
}
#endif

int ksmbd_smb_negotiate_common(struct ksmbd_work *work, unsigned int command)
{
	struct ksmbd_conn *conn = work->conn;
	int ret;

	conn->dialect =
		ksmbd_negotiate_smb_dialect(work->request_buf);
	ksmbd_debug(SMB, "conn->dialect 0x%x\n", conn->dialect);

	if (command == SMB2_NEGOTIATE_HE) {
		ret = smb2_handle_negotiate(work);
		return ret;
	}

	if (command == SMB_COM_NEGOTIATE) {
		if (__smb2_negotiate(conn)) {
			init_smb3_11_server(conn);
			init_smb2_neg_rsp(work);
			ksmbd_debug(SMB, "Upgrade to SMB2 negotiation\n");
			return 0;
		}
		return smb_handle_negotiate(work);
	}

	pr_err("Unknown SMB negotiation command: %u\n", command);
	return -EINVAL;
}

enum SHARED_MODE_ERRORS {
	SHARE_DELETE_ERROR,
	SHARE_READ_ERROR,
	SHARE_WRITE_ERROR,
	FILE_READ_ERROR,
	FILE_WRITE_ERROR,
	FILE_DELETE_ERROR,
};

static const char * const shared_mode_errors[] = {
	"Current access mode does not permit SHARE_DELETE",
	"Current access mode does not permit SHARE_READ",
	"Current access mode does not permit SHARE_WRITE",
	"Desired access mode does not permit FILE_READ",
	"Desired access mode does not permit FILE_WRITE",
	"Desired access mode does not permit FILE_DELETE",
};

static void smb_shared_mode_error(int error, struct ksmbd_file *prev_fp,
				  struct ksmbd_file *curr_fp)
{
	ksmbd_debug(SMB, "%s\n", shared_mode_errors[error]);
	ksmbd_debug(SMB, "Current mode: 0x%x Desired mode: 0x%x\n",
		    prev_fp->saccess, curr_fp->daccess);
}

int ksmbd_smb_check_shared_mode(struct file *filp, struct ksmbd_file *curr_fp)
{
	int rc = 0;
	struct ksmbd_file *prev_fp;

	/*
	 * Lookup fp in master fp list, and check desired access and
	 * shared mode between previous open and current open.
	 */
	read_lock(&curr_fp->f_ci->m_lock);
	list_for_each_entry(prev_fp, &curr_fp->f_ci->m_fp_list, node) {
		if (file_inode(filp) != file_inode(prev_fp->filp))
			continue;

		if (filp == prev_fp->filp)
			continue;

		if (ksmbd_stream_fd(prev_fp) && ksmbd_stream_fd(curr_fp))
			if (strcmp(prev_fp->stream.name, curr_fp->stream.name))
				continue;

		if (prev_fp->attrib_only != curr_fp->attrib_only)
			continue;

		if (!(prev_fp->saccess & FILE_SHARE_DELETE_LE) &&
		    curr_fp->daccess & FILE_DELETE_LE) {
			smb_shared_mode_error(SHARE_DELETE_ERROR,
					      prev_fp,
					      curr_fp);
			rc = -EPERM;
			break;
		}

		/*
		 * Only check FILE_SHARE_DELETE if stream opened and
		 * normal file opened.
		 */
		if (ksmbd_stream_fd(prev_fp) && !ksmbd_stream_fd(curr_fp))
			continue;

		if (!(prev_fp->saccess & FILE_SHARE_READ_LE) &&
		    curr_fp->daccess & (FILE_EXECUTE_LE | FILE_READ_DATA_LE)) {
			smb_shared_mode_error(SHARE_READ_ERROR,
					      prev_fp,
					      curr_fp);
			rc = -EPERM;
			break;
		}

		if (!(prev_fp->saccess & FILE_SHARE_WRITE_LE) &&
		    curr_fp->daccess & (FILE_WRITE_DATA_LE | FILE_APPEND_DATA_LE)) {
			smb_shared_mode_error(SHARE_WRITE_ERROR,
					      prev_fp,
					      curr_fp);
			rc = -EPERM;
			break;
		}

		if (prev_fp->daccess & (FILE_EXECUTE_LE | FILE_READ_DATA_LE) &&
		    !(curr_fp->saccess & FILE_SHARE_READ_LE)) {
			smb_shared_mode_error(FILE_READ_ERROR,
					      prev_fp,
					      curr_fp);
			rc = -EPERM;
			break;
		}

		if (prev_fp->daccess & (FILE_WRITE_DATA_LE | FILE_APPEND_DATA_LE) &&
		    !(curr_fp->saccess & FILE_SHARE_WRITE_LE)) {
			smb_shared_mode_error(FILE_WRITE_ERROR,
					      prev_fp,
					      curr_fp);
			rc = -EPERM;
			break;
		}

		if (prev_fp->daccess & FILE_DELETE_LE &&
		    !(curr_fp->saccess & FILE_SHARE_DELETE_LE)) {
			smb_shared_mode_error(FILE_DELETE_ERROR,
					      prev_fp,
					      curr_fp);
			rc = -EPERM;
			break;
		}
	}
	read_unlock(&curr_fp->f_ci->m_lock);

	return rc;
}

bool is_asterisk(char *p)
{
	return p && p[0] == '*';
}

int ksmbd_override_fsids(struct ksmbd_work *work)
{
	struct ksmbd_session *sess = work->sess;
	struct ksmbd_share_config *share = work->tcon->share_conf;
	struct cred *cred;
	struct group_info *gi;
	unsigned int uid;
	unsigned int gid;

	uid = user_uid(sess->user);
	gid = user_gid(sess->user);
	if (share->force_uid != KSMBD_SHARE_INVALID_UID)
		uid = share->force_uid;
	if (share->force_gid != KSMBD_SHARE_INVALID_GID)
		gid = share->force_gid;

	cred = prepare_kernel_cred(&init_task);
	if (!cred)
		return -ENOMEM;

	cred->fsuid = make_kuid(&init_user_ns, uid);
	cred->fsgid = make_kgid(&init_user_ns, gid);

	gi = groups_alloc(0);
	if (!gi) {
		abort_creds(cred);
		return -ENOMEM;
	}
	set_groups(cred, gi);
	put_group_info(gi);

	if (!uid_eq(cred->fsuid, GLOBAL_ROOT_UID))
		cred->cap_effective = cap_drop_fs_set(cred->cap_effective);

	WARN_ON(work->saved_cred);
	work->saved_cred = override_creds(cred);
	if (!work->saved_cred) {
		abort_creds(cred);
		return -EINVAL;
	}
	return 0;
}

void ksmbd_revert_fsids(struct ksmbd_work *work)
{
	const struct cred *cred;

	WARN_ON(!work->saved_cred);

	cred = current_cred();
	revert_creds(work->saved_cred);
	put_cred(cred);
	work->saved_cred = NULL;
}

__le32 smb_map_generic_desired_access(__le32 daccess)
{
	if (daccess & FILE_GENERIC_READ_LE) {
		daccess |= cpu_to_le32(GENERIC_READ_FLAGS);
		daccess &= ~FILE_GENERIC_READ_LE;
	}

	if (daccess & FILE_GENERIC_WRITE_LE) {
		daccess |= cpu_to_le32(GENERIC_WRITE_FLAGS);
		daccess &= ~FILE_GENERIC_WRITE_LE;
	}

	if (daccess & FILE_GENERIC_EXECUTE_LE) {
		daccess |= cpu_to_le32(GENERIC_EXECUTE_FLAGS);
		daccess &= ~FILE_GENERIC_EXECUTE_LE;
	}

	if (daccess & FILE_GENERIC_ALL_LE) {
		daccess |= cpu_to_le32(GENERIC_ALL_FLAGS);
		daccess &= ~FILE_GENERIC_ALL_LE;
	}

	return daccess;
}
