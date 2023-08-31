// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *   Copyright (C) 2016 Namjae Jeon <linkinjeon@kernel.org>
 *   Copyright (C) 2018 Samsung Electronics Co., Ltd.
 */

#include "glob.h"
#include "asn1.h"
#include "nterr.h"
#include "ksmbd_work.h"
#include "smb_common.h"
#include "smb1pdu.h"
#include "mgmt/user_session.h"

/**
 * check_smb_hdr() - check for valid smb request header
 * @smb:        smb header to be checked
 *
 * check for valid smb signature and packet direction(request/response)
 * TODO: properly check client authetication and tree authentication
 *
 * Return:      0 on success, otherwise 1
 */
static int check_smb1_hdr(struct smb_hdr *smb)
{
	/* does it have the right SMB "signature" ? */
	if (*(__le32 *) smb->Protocol != SMB1_PROTO_NUMBER) {
		ksmbd_debug(SMB, "Bad protocol string signature header 0x%x\n",
				*(unsigned int *)smb->Protocol);
		return 1;
	}
	ksmbd_debug(SMB, "got SMB\n");

	/* if it's not a response then accept */
	/* TODO : check for oplock break */
	if (!(smb->Flags & SMBFLG_RESPONSE))
		return 0;

	ksmbd_debug(SMB, "Server sent request, not response\n");
	return 1;
}


static int smb1_req_struct_size(struct smb_hdr *hdr)
{
	int wc = hdr->WordCount;

	switch (hdr->Command) {
	case SMB_COM_CREATE_DIRECTORY:
	case SMB_COM_DELETE_DIRECTORY:
	case SMB_COM_QUERY_INFORMATION:
	case SMB_COM_TREE_DISCONNECT:
	case SMB_COM_NEGOTIATE:
	case SMB_COM_NT_CANCEL:
	case SMB_COM_CHECK_DIRECTORY:
	case SMB_COM_PROCESS_EXIT:
		if (wc != 0x0)
			return -EINVAL;
		break;
	case SMB_COM_FLUSH:
	case SMB_COM_DELETE:
	case SMB_COM_RENAME:
	case SMB_COM_ECHO:
	case SMB_COM_FIND_CLOSE2:
		if (wc != 0x1)
			return -EINVAL;
		break;
	case SMB_COM_LOGOFF_ANDX:
		if (wc != 0x2)
			return -EINVAL;
		break;
	case SMB_COM_CLOSE:
		if (wc != 0x3)
			return -EINVAL;
		break;
	case SMB_COM_TREE_CONNECT_ANDX:
	case SMB_COM_NT_RENAME:
		if (wc != 0x4)
			return -EINVAL;
		break;
	case SMB_COM_WRITE:
		if (wc != 0x5)
			return -EINVAL;
		break;
	case SMB_COM_SETATTR:
	case SMB_COM_LOCKING_ANDX:
		if (wc != 0x8)
			return -EINVAL;
		break;
	case SMB_COM_TRANSACTION:
		if (wc < 0xe)
			return -EINVAL;
		break;
	case SMB_COM_SESSION_SETUP_ANDX:
		if (wc != 0xc && wc != 0xd)
			return -EINVAL;
		break;
	case SMB_COM_OPEN_ANDX:
	case SMB_COM_TRANSACTION2:
		if (wc != 0xf)
			return -EINVAL;
		break;
	case SMB_COM_NT_CREATE_ANDX:
		if (wc != 0x18)
			return -EINVAL;
		break;
	case SMB_COM_READ_ANDX:
		if (wc != 0xa && wc != 0xc)
			return -EINVAL;
		break;
	case SMB_COM_WRITE_ANDX:
		if (wc != 0xc && wc != 0xe)
			return -EINVAL;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return wc;
}

static int smb1_get_byte_count(struct smb_hdr *hdr)
{
	int bc;

	bc = le16_to_cpu(*(__le16 *)((char *)hdr +
		sizeof(struct smb_hdr) + hdr->WordCount * 2));

	switch (hdr->Command) {
	case SMB_COM_CLOSE:
	case SMB_COM_FLUSH:
	case SMB_COM_READ_ANDX:
	case SMB_COM_TREE_DISCONNECT:
	case SMB_COM_LOGOFF_ANDX:
	case SMB_COM_NT_CANCEL:
	case SMB_COM_PROCESS_EXIT:
	case SMB_COM_FIND_CLOSE2:
		if (bc != 0x0)
			return -EINVAL;
		break;
	case SMB_COM_LOCKING_ANDX:
	case SMB_COM_TRANSACTION:
	case SMB_COM_TRANSACTION2:
	case SMB_COM_ECHO:
	case SMB_COM_SESSION_SETUP_ANDX:
		if (bc < 0x0)
			return -EINVAL;
		break;
	case SMB_COM_WRITE_ANDX:
		if (bc < 0x1)
			return -EINVAL;
		break;
	case SMB_COM_CREATE_DIRECTORY:
	case SMB_COM_DELETE_DIRECTORY:
	case SMB_COM_DELETE:
	case SMB_COM_RENAME:
	case SMB_COM_QUERY_INFORMATION:
	case SMB_COM_SETATTR:
	case SMB_COM_OPEN_ANDX:
	case SMB_COM_NEGOTIATE:
	case SMB_COM_CHECK_DIRECTORY:
		if (bc < 0x2)
			return -EINVAL;
		break;
	case SMB_COM_TREE_CONNECT_ANDX:
	case SMB_COM_WRITE:
		if (bc < 0x3)
			return -EINVAL;
		break;
	case SMB_COM_NT_RENAME:
		if (bc < 0x4)
			return -EINVAL;
		break;
	case SMB_COM_NT_CREATE_ANDX:
		if (hdr->Flags2 & SMBFLG2_UNICODE) {
			if (bc < 3)
				return -EINVAL;
		} else if (bc < 2)
			return -EINVAL;
		break;
	}

	return bc;
}

static unsigned int smb1_calc_size(struct smb_hdr *hdr)
{
	int len = sizeof(struct smb_hdr) - 4 + 2;
	int bc, struct_size = hdr->WordCount * 2;

	len += struct_size;
	bc = smb1_get_byte_count(hdr);
	if (bc < 0)
		return bc;
	ksmbd_debug(SMB, "SMB2 byte count %d, struct size : %d\n", bc,
		struct_size);
	len += bc;

	ksmbd_debug(SMB, "SMB1 len %d\n", len);
	return len;
}

static int smb1_get_data_len(struct smb_hdr *hdr)
{
	int data_len = 0;

	/* data offset check */
	switch (hdr->Command) {
	case SMB_COM_WRITE_ANDX:
	{
		struct smb_com_write_req *req = (struct smb_com_write_req *)hdr;

		data_len = le16_to_cpu(req->DataLengthLow);
		data_len |= (le16_to_cpu(req->DataLengthHigh) << 16);
		data_len += le16_to_cpu(req->DataOffset);
		break;
	}
	case SMB_COM_TRANSACTION:
	{
		struct smb_com_trans_req *req = (struct smb_com_trans_req *)hdr;

		data_len = le16_to_cpu(req->DataOffset) +
			le16_to_cpu(req->DataCount);
		break;
	}
	case SMB_COM_TRANSACTION2:
	{
		struct smb_com_trans2_req *req =
				(struct smb_com_trans2_req *)hdr;

		data_len = le16_to_cpu(req->DataOffset) +
			le16_to_cpu(req->DataCount);
		break;
	}
	}

	return data_len;
}

int ksmbd_smb1_check_message(struct ksmbd_work *work)
{
	struct smb_hdr *hdr = (struct smb_hdr *)work->request_buf;
	char *buf = work->request_buf;
	int command = hdr->Command;
	__u32 clc_len;  /* calculated length */
	__u32 len = get_rfc1002_len(buf);
	int wc, data_len;

	if (check_smb1_hdr(hdr))
		return 1;

	wc = smb1_req_struct_size(hdr);
	if (wc == -EOPNOTSUPP) {
		ksmbd_debug(SMB, "Not support cmd %x\n", command);
		return 1;
	} else if (hdr->WordCount != wc) {
		pr_err("Invalid word count, %d not %d. cmd %x\n",
		       hdr->WordCount, wc, command);
		return 1;
	}

	data_len = smb1_get_data_len(hdr);
	if (len < data_len) {
		pr_err("Invalid data area length %u not %u. cmd : %x\n",
		       len, data_len, command);
		return 1;
	}

	clc_len = smb1_calc_size(hdr);
	if (len != clc_len) {
		/*
		 * smbclient may return wrong byte count in smb header.
		 * But allow it to avoid write failure with smbclient.
		 */
		if (command == SMB_COM_WRITE_ANDX)
			return 0;

		if (len > clc_len) {
			ksmbd_debug(SMB,
				"cli req too long, len %d not %d. cmd:%x\n",
				len, clc_len, command);
			return 0;
		}

		pr_err("cli req too short, len %d not %d. cmd:%x\n",
		       len, clc_len, command);

		return 1;
	}

	return 0;
}

int smb_negotiate_request(struct ksmbd_work *work)
{
	return ksmbd_smb_negotiate_common(work, SMB_COM_NEGOTIATE);
}
