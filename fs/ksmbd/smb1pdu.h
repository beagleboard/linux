/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 *   Copyright (C) 2016 Namjae Jeon <linkinjeon@kernel.org>
 *   Copyright (C) 2018 Samsung Electronics Co., Ltd.
 */

#ifndef __SMB1PDU_H
#define __SMB1PDU_H

#define MAX_CIFS_HDR_SIZE 0x58

#define SMB1_CLIENT_GUID_SIZE		(16)
#define SMB1_MAX_MPX_COUNT		10
#define SMB1_MAX_VCS			1
#define SMB1_MAX_RAW_SIZE		65536
#define MAX_CIFS_LOOKUP_BUFFER_SIZE	(16*1024)

/*
 * Size of the ntlm client response
 */
#define CIFS_AUTH_RESP_SIZE		24
#define CIFS_SMB1_SIGNATURE_SIZE	8
#define CIFS_SMB1_SESSKEY_SIZE		16

#define SMB1_SERVER_CAPS					\
	(CAP_UNICODE | CAP_LARGE_FILES | CAP_EXTENDED_SECURITY |\
	 CAP_NT_SMBS | CAP_STATUS32 | CAP_LOCK_AND_READ |	\
	 CAP_NT_FIND | CAP_UNIX | CAP_LARGE_READ_X |		\
	 CAP_LARGE_WRITE_X | CAP_LEVEL_II_OPLOCKS)

#define SMB1_SERVER_SECU  (SECMODE_USER | SECMODE_PW_ENCRYPT)

/* Service Type of TreeConnect*/
#define SERVICE_DISK_SHARE	"A:"
#define SERVICE_IPC_SHARE	"IPC"
#define SERVICE_PRINTER_SHARE	"LPT1:"
#define SERVICE_COMM		"COMM"

#define NATIVE_FILE_SYSTEM	"NTFS"

#define SMB_NO_MORE_ANDX_COMMAND 0xFF
#define SMB1_PROTO_NUMBER cpu_to_le32(0x424d53ff)

/* Transact2 subcommand codes */
#define TRANS2_OPEN                   0x00
#define TRANS2_FIND_FIRST             0x01
#define TRANS2_FIND_NEXT              0x02
#define TRANS2_QUERY_FS_INFORMATION   0x03
#define TRANS2_SET_FS_INFORMATION     0x04
#define TRANS2_QUERY_PATH_INFORMATION 0x05
#define TRANS2_SET_PATH_INFORMATION   0x06
#define TRANS2_QUERY_FILE_INFORMATION 0x07
#define TRANS2_SET_FILE_INFORMATION   0x08
#define TRANS2_CREATE_DIRECTORY       0x0d
#define TRANS2_GET_DFS_REFERRAL       0x10
#define TRANS2_REPORT_DFS_INCOSISTENCY 0x11

/* SMB Transact (Named Pipe) subcommand codes */
#define TRANS_SET_NMPIPE_STATE      0x0001
#define TRANS_RAW_READ_NMPIPE       0x0011
#define TRANS_QUERY_NMPIPE_STATE    0x0021
#define TRANS_QUERY_NMPIPE_INFO     0x0022
#define TRANS_PEEK_NMPIPE           0x0023
#define TRANS_TRANSACT_NMPIPE       0x0026
#define TRANS_RAW_WRITE_NMPIPE      0x0031
#define TRANS_READ_NMPIPE           0x0036
#define TRANS_WRITE_NMPIPE          0x0037
#define TRANS_WAIT_NMPIPE           0x0053
#define TRANS_CALL_NMPIPE           0x0054

/* NT Transact subcommand codes */
#define NT_TRANSACT_CREATE            0x01
#define NT_TRANSACT_IOCTL             0x02
#define NT_TRANSACT_SET_SECURITY_DESC 0x03
#define NT_TRANSACT_NOTIFY_CHANGE     0x04
#define NT_TRANSACT_RENAME            0x05
#define NT_TRANSACT_QUERY_SECURITY_DESC 0x06
#define NT_TRANSACT_GET_USER_QUOTA    0x07
#define NT_TRANSACT_SET_USER_QUOTA    0x08

/*
 * SMB flag definitions
 */
#define SMBFLG_EXTD_LOCK 0x01   /* server supports lock-read write-unlock smb */
#define SMBFLG_RCV_POSTED 0x02  /* obsolete */
#define SMBFLG_RSVD 0x04
#define SMBFLG_CASELESS 0x08    /*
				 * all pathnames treated as caseless (off
				 * implies case sensitive file handling
				 * request)
				 */
#define SMBFLG_CANONICAL_PATH_FORMAT 0x10       /* obsolete */
#define SMBFLG_OLD_OPLOCK 0x20  /* obsolete */
#define SMBFLG_OLD_OPLOCK_NOTIFY 0x40   /* obsolete */
#define SMBFLG_RESPONSE 0x80    /* this PDU is a response from server */

/*
 * SMB flag2 definitions
 */
#define SMBFLG2_KNOWS_LONG_NAMES cpu_to_le16(1) /*
						 * can send long (non-8.3)
						 * path names in response
						 */
#define SMBFLG2_KNOWS_EAS cpu_to_le16(2)
#define SMBFLG2_SECURITY_SIGNATURE cpu_to_le16(4)
#define SMBFLG2_COMPRESSED (8)
#define SMBFLG2_SECURITY_SIGNATURE_REQUIRED (0x10)
#define SMBFLG2_IS_LONG_NAME cpu_to_le16(0x40)
#define SMBFLG2_REPARSE_PATH (0x400)
#define SMBFLG2_EXT_SEC cpu_to_le16(0x800)
#define SMBFLG2_DFS cpu_to_le16(0x1000)
#define SMBFLG2_PAGING_IO cpu_to_le16(0x2000)
#define SMBFLG2_ERR_STATUS cpu_to_le16(0x4000)
#define SMBFLG2_UNICODE cpu_to_le16(0x8000)

#define SMB_COM_CREATE_DIRECTORY      0x00 /* trivial response */
#define SMB_COM_DELETE_DIRECTORY      0x01 /* trivial response */
#define SMB_COM_CLOSE                 0x04 /* triv req/rsp, timestamp ignored */
#define SMB_COM_FLUSH                 0x05 /* triv req/rsp */
#define SMB_COM_DELETE                0x06 /* trivial response */
#define SMB_COM_RENAME                0x07 /* trivial response */
#define SMB_COM_QUERY_INFORMATION     0x08 /* aka getattr */
#define SMB_COM_SETATTR               0x09 /* trivial response */
#define SMB_COM_WRITE                 0x0b
#define SMB_COM_CHECK_DIRECTORY       0x10 /* trivial response */
#define SMB_COM_PROCESS_EXIT          0x11 /* trivial response */
#define SMB_COM_LOCKING_ANDX          0x24 /* trivial response */
#define SMB_COM_TRANSACTION	      0x25
#define SMB_COM_COPY                  0x29 /* trivial rsp, fail filename ignrd*/
#define SMB_COM_ECHO                  0x2B /* echo request */
#define SMB_COM_OPEN_ANDX             0x2D /* Legacy open for old servers */
#define SMB_COM_READ_ANDX             0x2E
#define SMB_COM_WRITE_ANDX            0x2F
#define SMB_COM_TRANSACTION2          0x32
#define SMB_COM_TRANSACTION2_SECONDARY 0x33
#define SMB_COM_FIND_CLOSE2           0x34 /* trivial response */
#define SMB_COM_TREE_DISCONNECT       0x71 /* trivial response */
#define SMB_COM_NEGOTIATE             0x72
#define SMB_COM_SESSION_SETUP_ANDX    0x73
#define SMB_COM_LOGOFF_ANDX           0x74 /* trivial response */
#define SMB_COM_TREE_CONNECT_ANDX     0x75
#define SMB_COM_QUERY_INFORMATION_DISK 0x80
#define SMB_COM_NT_TRANSACT           0xA0
#define SMB_COM_NT_TRANSACT_SECONDARY 0xA1
#define SMB_COM_NT_CREATE_ANDX        0xA2
#define SMB_COM_NT_CANCEL             0xA4 /* no response */
#define SMB_COM_NT_RENAME             0xA5 /* trivial response */

/* Negotiate response Capabilities */
#define CAP_RAW_MODE           0x00000001
#define CAP_MPX_MODE           0x00000002
#define CAP_UNICODE            0x00000004
#define CAP_LARGE_FILES        0x00000008
#define CAP_NT_SMBS            0x00000010       /* implies CAP_NT_FIND */
#define CAP_RPC_REMOTE_APIS    0x00000020
#define CAP_STATUS32           0x00000040
#define CAP_LEVEL_II_OPLOCKS   0x00000080
#define CAP_LOCK_AND_READ      0x00000100
#define CAP_NT_FIND            0x00000200
#define CAP_DFS                0x00001000
#define CAP_INFOLEVEL_PASSTHRU 0x00002000
#define CAP_LARGE_READ_X       0x00004000
#define CAP_LARGE_WRITE_X      0x00008000
#define CAP_LWIO               0x00010000 /* support fctl_srv_req_resume_key */
#define CAP_UNIX               0x00800000
#define CAP_COMPRESSED_DATA    0x02000000
#define CAP_DYNAMIC_REAUTH     0x20000000
#define CAP_PERSISTENT_HANDLES 0x40000000
#define CAP_EXTENDED_SECURITY  0x80000000

/* RFC 1002 session packet types */
#define RFC1002_SESSION_MESSAGE 0x00
#define RFC1002_SESSION_REQUEST  0x81
#define RFC1002_POSITIVE_SESSION_RESPONSE 0x82
#define RFC1002_NEGATIVE_SESSION_RESPONSE 0x83
#define RFC1002_RETARGET_SESSION_RESPONSE 0x84
#define RFC1002_SESSION_KEEP_ALIVE 0x85

/* Action bits */
#define GUEST_LOGIN 1

struct smb_negotiate_rsp {
	struct smb_hdr hdr;     /* wct = 17 */
	__le16 DialectIndex; /* 0xFFFF = no dialect acceptable */
	__u8 SecurityMode;
	__le16 MaxMpxCount;
	__le16 MaxNumberVcs;
	__le32 MaxBufferSize;
	__le32 MaxRawSize;
	__le32 SessionKey;
	__le32 Capabilities;    /* see below */
	__le32 SystemTimeLow;
	__le32 SystemTimeHigh;
	__le16 ServerTimeZone;
	__u8 EncryptionKeyLength;
	__le16 ByteCount;
	union {
		unsigned char EncryptionKey[8]; /* cap extended security off */
		/* followed by Domain name - if extended security is off */
		/* followed by 16 bytes of server GUID */
		/* then security blob if cap_extended_security negotiated */
		struct {
			unsigned char GUID[SMB1_CLIENT_GUID_SIZE];
			unsigned char SecurityBlob[1];
		} __packed extended_response;
	} __packed u;
} __packed;

struct smb_com_read_req {
	struct smb_hdr hdr;     /* wct = 12 */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__u16 Fid;
	__le32 OffsetLow;
	__le16 MaxCount;
	__le16 MinCount;                /* obsolete */
	__le32 MaxCountHigh;
	__le16 Remaining;
	__le32 OffsetHigh;
	__le16 ByteCount;
} __packed;

struct smb_com_read_rsp {
	struct smb_hdr hdr;     /* wct = 12 */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__le16 Remaining;
	__le16 DataCompactionMode;
	__le16 Reserved;
	__le16 DataLength;
	__le16 DataOffset;
	__le16 DataLengthHigh;
	__u64 Reserved2;
	__le16 ByteCount;
	/* read response data immediately follows */
} __packed;

struct smb_com_write_req {
	struct smb_hdr hdr;	/* wct = 14 */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__u16 Fid;
	__le32 OffsetLow;
	__u32 Reserved;
	__le16 WriteMode;
	__le16 Remaining;
	__le16 DataLengthHigh;
	__le16 DataLengthLow;
	__le16 DataOffset;
	__le32 OffsetHigh;
	__le16 ByteCount;
	__u8 Pad;		/*
				 * BB check for whether padded to DWORD
				 * boundary and optimum performance here
				 */
	char Data[0];
} __packed;

struct smb_com_write_req_32bit {
	struct smb_hdr hdr;	/* wct = 5 */
	__u16 Fid;
	__le16 Length;
	__le32 Offset;
	__u16 Estimate;
	__le16 ByteCount;	/* must be greater than 2 */
	__u8 BufferFormat;
	__u16 DataLength;
	char Data[0];
} __packed;

struct smb_com_write_rsp_32bit {
	struct smb_hdr hdr;	/* wct = 1 */
	__le16 Written;
	__le16 ByteCount;	/* must be 0 */
} __packed;

struct smb_com_write_rsp {
	struct smb_hdr hdr;	/* wct = 6 */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__le16 Count;
	__le16 Remaining;
	__le16 CountHigh;
	__u16  Reserved;
	__le16 ByteCount;
} __packed;

struct smb_com_rename_req {
	struct smb_hdr hdr;     /* wct = 1 */
	__le16 SearchAttributes;        /* target file attributes */
	__le16 ByteCount;
	__u8 BufferFormat;      /* 4 = ASCII or Unicode */
	unsigned char OldFileName[1];
	/* followed by __u8 BufferFormat2 */
	/* followed by NewFileName */
} __packed;

struct smb_com_rename_rsp {
	struct smb_hdr hdr;     /* wct = 0 */
	__le16 ByteCount;        /* bct = 0 */
} __packed;

/* SecurityMode bits */
#define SECMODE_USER          0x01      /* off indicates share level security */
#define SECMODE_PW_ENCRYPT    0x02
#define SECMODE_SIGN_ENABLED  0x04      /* SMB security signatures enabled */
#define SECMODE_SIGN_REQUIRED 0x08      /* SMB security signatures required */

struct smb_com_session_setup_req {	/* request format */
	struct smb_hdr hdr;	/* wct = 12 */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__le16 MaxBufferSize;
	__le16 MaxMpxCount;
	__le16 VcNumber;
	__u32 SessionKey;
	__le16 SecurityBlobLength;
	__u32 Reserved;
	__le32 Capabilities;	/* see below */
	__le16 ByteCount;
	unsigned char SecurityBlob[1];	/* followed by */
	/* STRING NativeOS */
	/* STRING NativeLanMan */
} __packed;	/* NTLM request format (with extended security) */

struct smb_com_session_setup_req_no_secext {	/* request format */
	struct smb_hdr hdr;	/* we will handle this :: wct = 13 */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__le16 MaxBufferSize;
	__le16 MaxMpxCount;
	__le16 VcNumber;
	__u32 SessionKey;
	__le16 CaseInsensitivePasswordLength;	/* ASCII password len */
	__le16 CaseSensitivePasswordLength;	/* Unicode password length*/
	__u32 Reserved;	/* see below */
	__le32 Capabilities;
	__le16 ByteCount;
	unsigned char CaseInsensitivePassword[0];	/* followed by: */
	/* unsigned char * CaseSensitivePassword; */
	/* STRING AccountName */
	/* STRING PrimaryDomain */
	/* STRING NativeOS */
	/* STRING NativeLanMan */
} __packed;	/* NTLM request format (without extended security */

struct smb_com_session_setup_resp {	/* default (NTLM) response format */
	struct smb_hdr hdr;	/* wct = 4 */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__le16 Action;	/* see below */
	__le16 SecurityBlobLength;
	__le16 ByteCount;
	unsigned char SecurityBlob[1];	/* followed by */
	/*      unsigned char  * NativeOS;      */
	/*      unsigned char  * NativeLanMan;  */
	/*      unsigned char  * PrimaryDomain; */
} __packed;	/* NTLM response (with or without extended sec) */

struct smb_com_session_setup_old_resp { /* default (NTLM) response format */
	struct smb_hdr hdr;	/* wct = 3 */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__le16 Action;	/* see below */
	__le16 ByteCount;
	unsigned char NativeOS[1];	/* followed by */
	/*      unsigned char * NativeLanMan; */
	/*      unsigned char * PrimaryDomain; */
} __packed;	/* pre-NTLM (LANMAN2.1) response */

union smb_com_session_setup_andx {
	struct smb_com_session_setup_req req;
	struct smb_com_session_setup_req_no_secext req_no_secext;
	struct smb_com_session_setup_resp resp;
	struct smb_com_session_setup_old_resp old_resp;
} __packed;

struct smb_com_tconx_req {
	__u8 WordCount;  /* wct = 4, it could be ANDX */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__le16 Flags;           /* see below */
	__le16 PasswordLength;
	__le16 ByteCount;
	unsigned char Password[1];      /* followed by */
	/* STRING Path    *//* \\server\share name */
	/* STRING Service */
} __packed;

struct smb_com_tconx_rsp {
	__u8 WordCount;     /* wct = 3 , not extended response */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__le16 OptionalSupport; /* see below */
	__le16 ByteCount;
	unsigned char Service[1];       /* always ASCII, not Unicode */
	/* STRING NativeFileSystem */
} __packed;

struct smb_com_tconx_rsp_ext {
	__u8 WordCount;	/* wct = 7, extended response */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__le16 OptionalSupport; /* see below */
	__le32 MaximalShareAccessRights;
	__le32 GuestMaximalShareAccessRights;
	__le16 ByteCount;
	unsigned char Service[1];       /* always ASCII, not Unicode */
	/* STRING NativeFileSystem */
} __packed;

struct andx_block {
	__u8 WordCount;
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
} __packed;

struct locking_andx_range64 {
	__le16 Pid;
	__le16 Pad;
	__le32 OffsetHigh;
	__le32 OffsetLow;
	__le32 LengthHigh;
	__le32 LengthLow;
} __packed;

struct locking_andx_range32 {
	__le16 Pid;
	__le32 Offset;
	__le32 Length;
} __packed;

#define LOCKING_ANDX_SHARED_LOCK     0x01
#define LOCKING_ANDX_OPLOCK_RELEASE  0x02
#define LOCKING_ANDX_CHANGE_LOCKTYPE 0x04
#define LOCKING_ANDX_CANCEL_LOCK     0x08
#define LOCKING_ANDX_LARGE_FILES     0x10       /* always on for us */

struct smb_com_lock_req {
	struct smb_hdr hdr;	/* wct = 8 */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__u16 Fid;
	__u8 LockType;
	__u8 OplockLevel;
	__le32 Timeout;
	__le16 NumberOfUnlocks;
	__le16 NumberOfLocks;
	__le16 ByteCount;
	char *Locks[1];
} __packed;

struct smb_com_lock_rsp {
	struct smb_hdr hdr;     /* wct = 2 */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__le16 ByteCount;
} __packed;

struct smb_com_query_information_disk_rsp {
	struct smb_hdr hdr;     /* wct = 5 */
	__le16 TotalUnits;
	__le16 BlocksPerUnit;
	__le16 BlockSize;
	__le16 FreeUnits;
	__le16 Pad;
	__le16 ByteCount;
} __packed;

/* tree connect Flags */
#define DISCONNECT_TID          0x0001
#define TCON_EXTENDED_SIGNATURES 0x0004
#define TCON_EXTENDED_SECINFO   0x0008

/* OptionalSupport bits */
#define SMB_SUPPORT_SEARCH_BITS 0x0001  /*
					 * "must have" directory search bits
					 * (exclusive searches supported)
					 */
#define SMB_SHARE_IS_IN_DFS     0x0002
#define SMB_CSC_MASK               0x000C
/* CSC flags defined as follows */
#define SMB_CSC_CACHE_MANUAL_REINT 0x0000
#define SMB_CSC_CACHE_AUTO_REINT   0x0004
#define SMB_CSC_CACHE_VDO          0x0008
#define SMB_CSC_NO_CACHING         0x000C
#define SMB_UNIQUE_FILE_NAME    0x0010
#define SMB_EXTENDED_SIGNATURES 0x0020

/* OpenFlags */
#define REQ_MORE_INFO      0x00000001  /* legacy (OPEN_AND_X) only */
#define REQ_OPLOCK         0x00000002
#define REQ_BATCHOPLOCK    0x00000004
#define REQ_OPENDIRONLY    0x00000008
#define REQ_EXTENDED_INFO  0x00000010

/* File type */
#define DISK_TYPE               0x0000
#define BYTE_PIPE_TYPE          0x0001
#define MESSAGE_PIPE_TYPE       0x0002
#define PRINTER_TYPE            0x0003
#define COMM_DEV_TYPE           0x0004
#define UNKNOWN_TYPE            0xFFFF

/* Device Type or File Status Flags */
#define NO_EAS                  0x0001
#define NO_SUBSTREAMS           0x0002
#define NO_REPARSETAG           0x0004
/* following flags can apply if pipe */
#define ICOUNT_MASK             0x00FF
#define PIPE_READ_MODE          0x0100
#define NAMED_PIPE_TYPE         0x0400
#define PIPE_END_POINT          0x4000
#define BLOCKING_NAMED_PIPE     0x8000

/* ShareAccess flags */
#define FILE_NO_SHARE     0x00000000
#define FILE_SHARE_READ   0x00000001
#define FILE_SHARE_WRITE  0x00000002
#define FILE_SHARE_DELETE 0x00000004
#define FILE_SHARE_ALL    0x00000007

/* CreateDisposition flags, similar to CreateAction as well */
#define FILE_SUPERSEDE    0x00000000
#define FILE_OPEN         0x00000001
#define FILE_CREATE       0x00000002
#define FILE_OPEN_IF      0x00000003
#define FILE_OVERWRITE    0x00000004
#define FILE_OVERWRITE_IF 0x00000005

/* ImpersonationLevel flags */
#define SECURITY_ANONYMOUS      0
#define SECURITY_IDENTIFICATION 1
#define SECURITY_IMPERSONATION  2
#define SECURITY_DELEGATION     3

/* SecurityFlags */
#define SECURITY_CONTEXT_TRACKING 0x01
#define SECURITY_EFFECTIVE_ONLY   0x02

struct smb_com_open_req {       /* also handles create */
	struct smb_hdr hdr;     /* wct = 24 */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__u8 Reserved;          /* Must Be Zero */
	__le16 NameLength;
	__le32 OpenFlags;
	__u32  RootDirectoryFid;
	__le32 DesiredAccess;
	__le64 AllocationSize;
	__le32 FileAttributes;
	__le32 ShareAccess;
	__le32 CreateDisposition;
	__le32 CreateOptions;
	__le32 ImpersonationLevel;
	__u8 SecurityFlags;
	__le16 ByteCount;
	char fileName[1];
} __packed;

/* open response for CreateAction shifted left */
#define CIFS_CREATE_ACTION 0x20000 /* file created */

/* Basic file attributes */
#define SMB_FILE_ATTRIBUTE_NORMAL	0x0000
#define SMB_FILE_ATTRIBUTE_READONLY	0x0001
#define SMB_FILE_ATTRIBUTE_HIDDEN	0x0002
#define SMB_FILE_ATTRIBUTE_SYSTEM	0x0004
#define SMB_FILE_ATTRIBUTE_VOLUME	0x0008
#define SMB_FILE_ATTRIBUTE_DIRECTORY	0x0010
#define SMB_FILE_ATTRIBUTE_ARCHIVE	0x0020
#define SMB_SEARCH_ATTRIBUTE_READONLY	0x0100
#define SMB_SEARCH_ATTRIBUTE_HIDDEN	0x0200
#define SMB_SEARCH_ATTRIBUTE_SYSTEM	0x0400
#define SMB_SEARCH_ATTRIBUTE_DIRECTORY	0x1000
#define SMB_SEARCH_ATTRIBUTE_ARCHIVE	0x2000

struct smb_com_open_rsp {
	struct smb_hdr hdr;     /* wct = 34 BB */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__u8 OplockLevel;
	__u16 Fid;
	__le32 CreateAction;
	__le64 CreationTime;
	__le64 LastAccessTime;
	__le64 LastWriteTime;
	__le64 ChangeTime;
	__le32 FileAttributes;
	__le64 AllocationSize;
	__le64 EndOfFile;
	__le16 FileType;
	__le16 DeviceState;
	__u8 DirectoryFlag;
	__le16 ByteCount;        /* bct = 0 */
} __packed;

struct smb_com_open_ext_rsp {
	struct smb_hdr hdr;     /* wct = 42 */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__u8 OplockLevel;
	__u16 Fid;
	__le32 CreateAction;
	__le64 CreationTime;
	__le64 LastAccessTime;
	__le64 LastWriteTime;
	__le64 ChangeTime;
	__le32 FileAttributes;
	__le64 AllocationSize;
	__le64 EndOfFile;
	__le16 FileType;
	__le16 DeviceState;
	__u8 DirectoryFlag;
	__u8 VolId[16];
	__u64 fid;
	__le32 MaxAccess;
	__le32 GuestAccess;
	__le16 ByteCount;        /* bct = 0 */
} __packed;

struct smb_com_close_req {
	struct smb_hdr hdr;     /* wct = 3 */
	__u16 FileID;
	__le32 LastWriteTime;    /* should be zero or -1 */
	__le16  ByteCount;        /* 0 */
} __packed;

struct smb_com_close_rsp {
	struct smb_hdr hdr;     /* wct = 0 */
	__le16 ByteCount;        /* bct = 0 */
} __packed;

struct smb_com_echo_req {
	struct  smb_hdr hdr;
	__le16  EchoCount;
	__le16  ByteCount;
	char    Data[1];
} __packed;

struct smb_com_echo_rsp {
	struct  smb_hdr hdr;
	__le16  SequenceNumber;
	__le16  ByteCount;
	char    Data[1];
} __packed;

struct smb_com_flush_req {
	struct smb_hdr hdr;     /* wct = 1 */
	__u16 FileID;
	__le16 ByteCount;        /* 0 */
} __packed;

struct smb_com_flush_rsp {
	struct smb_hdr hdr;     /* wct = 0 */
	__le16 ByteCount;        /* bct = 0 */
} __packed;

/* SMB_COM_TRANSACTION */
struct smb_com_trans_req {
	struct smb_hdr hdr;
	__le16 TotalParameterCount;
	__le16 TotalDataCount;
	__le16 MaxParameterCount;
	__le16 MaxDataCount;
	__u8 MaxSetupCount;
	__u8 Reserved;
	__le16 Flags;
	__le32 Timeout;
	__u16 Reserved2;
	__le16 ParameterCount;
	__le16 ParameterOffset;
	__le16 DataCount;
	__le16 DataOffset;
	__u8 SetupCount;
	__u8 Reserved3;
	__le16 SubCommand;
	__u8  Pad;
	__u8 Data[1];
} __packed;

struct smb_com_trans_pipe_req {
	struct smb_hdr hdr;
	__le16 TotalParameterCount;
	__le16 TotalDataCount;
	__le16 MaxParameterCount;
	__le16 MaxDataCount;
	__u8 MaxSetupCount;
	__u8 Reserved;
	__le16 Flags;
	__le32 Timeout;
	__u16 Reserved2;
	__le16 ParameterCount;
	__le16 ParameterOffset;
	__le16 DataCount;
	__le16 DataOffset;
	__u8 SetupCount;
	__u8 Reserved3;
	__u16 SubCommand;
	__u16 fid;
	__le16 ByteCount;
	__u8  Pad;
	__u8 Data[1];
} __packed;

struct smb_com_trans_rsp {
	struct smb_hdr hdr;     /* wct = 10+ */
	__le16 TotalParameterCount;
	__le16 TotalDataCount;
	__u16 Reserved;
	__le16 ParameterCount;
	__le16 ParameterOffset;
	__le16 ParameterDisplacement;
	__le16 DataCount;
	__le16 DataOffset;
	__le16 DataDisplacement;
	__u8 SetupCount;
	__u8 Reserved1;
	__le16 ByteCount;
	__u8 Pad;
} __packed;

/* SMB_COM_TRANSACTION subcommands */

#define TRANSACT_DCERPCCMD	0x26

/*****************************************************************************
 * TRANS2 command implementation functions
 *****************************************************************************/
#define NO_CHANGE_64          0xFFFFFFFFFFFFFFFFULL

/* QFSInfo Levels */
#define SMB_INFO_ALLOCATION         1
#define SMB_INFO_VOLUME             2
#define SMB_QUERY_FS_VOLUME_INFO    0x102
#define SMB_QUERY_FS_SIZE_INFO      0x103
#define SMB_QUERY_FS_DEVICE_INFO    0x104
#define SMB_QUERY_FS_ATTRIBUTE_INFO 0x105
#define SMB_QUERY_CIFS_UNIX_INFO    0x200
#define SMB_QUERY_POSIX_FS_INFO     0x201
#define SMB_QUERY_POSIX_WHO_AM_I    0x202
#define SMB_REQUEST_TRANSPORT_ENCRYPTION 0x203
#define SMB_QUERY_FS_PROXY          0x204 /*
					   * WAFS enabled. Returns structure
					   * FILE_SYSTEM__UNIX_INFO to tell
					   * whether new NTIOCTL available
					   * (0xACE) for WAN friendly SMB
					   * operations to be carried
					   */
#define SMB_QUERY_LABEL_INFO        0x3ea
#define SMB_QUERY_FS_QUOTA_INFO     0x3ee
#define SMB_QUERY_FS_FULL_SIZE_INFO 0x3ef
#define SMB_QUERY_OBJECTID_INFO     0x3f0

struct trans2_resp {
	/* struct smb_hdr hdr precedes. Note wct = 10 + setup count */
	__le16 TotalParameterCount;
	__le16 TotalDataCount;
	__u16 Reserved;
	__le16 ParameterCount;
	__le16 ParameterOffset;
	__le16 ParameterDisplacement;
	__le16 DataCount;
	__le16 DataOffset;
	__le16 DataDisplacement;
	__u8 SetupCount;
	__u8 Reserved1;
	/*
	 * SetupWords[SetupCount];
	 * __u16 ByteCount;
	 * __u16 Reserved2;
	 */
	/* data area follows */
} __packed;

struct smb_com_trans2_req {
	struct smb_hdr hdr;
	__le16 TotalParameterCount;
	__le16 TotalDataCount;
	__le16 MaxParameterCount;
	__le16 MaxDataCount;
	__u8 MaxSetupCount;
	__u8 Reserved;
	__le16 Flags;
	__le32 Timeout;
	__u16 Reserved2;
	__le16 ParameterCount;
	__le16 ParameterOffset;
	__le16 DataCount;
	__le16 DataOffset;
	__u8 SetupCount;
	__u8 Reserved3;
	__le16 SubCommand;      /* one setup word */
} __packed;

struct smb_com_trans2_qfsi_req {
	struct smb_hdr hdr;     /* wct = 14+ */
	__le16 TotalParameterCount;
	__le16 TotalDataCount;
	__le16 MaxParameterCount;
	__le16 MaxDataCount;
	__u8 MaxSetupCount;
	__u8 Reserved;
	__le16 Flags;
	__le32 Timeout;
	__u16 Reserved2;
	__le16 ParameterCount;
	__le16 ParameterOffset;
	__le16 DataCount;
	__le16 DataOffset;
	__u8 SetupCount;
	__u8 Reserved3;
	__le16 SubCommand;      /* one setup word */
	__le16 ByteCount;
	__u8 Pad;
	__le16 InformationLevel;
} __packed;

struct smb_com_trans2_qfsi_req_params {
	__le16 InformationLevel;
} __packed;

#define CIFS_SEARCH_CLOSE_ALWAYS	0x0001
#define CIFS_SEARCH_CLOSE_AT_END	0x0002
#define CIFS_SEARCH_RETURN_RESUME	0x0004
#define CIFS_SEARCH_CONTINUE_FROM_LAST	0x0008
#define CIFS_SEARCH_BACKUP_SEARCH	0x0010

struct smb_com_trans2_ffirst_req_params {
	__le16 SearchAttributes;
	__le16 SearchCount;
	__le16 SearchFlags;
	__le16 InformationLevel;
	__le32 SearchStorageType;
	char FileName[1];
} __packed;

struct smb_com_trans2_ffirst_rsp_parms {
	__u16 SearchHandle;
	__le16 SearchCount;
	__le16 EndofSearch;
	__le16 EAErrorOffset;
	__le16 LastNameOffset;
} __packed;

struct smb_com_trans2_fnext_req_params {
	__u16 SearchHandle;
	__le16 SearchCount;
	__le16 InformationLevel;
	__u32 ResumeKey;
	__le16 SearchFlags;
	char ResumeFileName[1];
} __packed;

struct smb_com_trans2_fnext_rsp_params {
	__le16 SearchCount;
	__le16 EndofSearch;
	__le16 EAErrorOffset;
	__le16 LastNameOffset;
} __packed;

struct smb_com_trans2_rsp {
	struct smb_hdr hdr;     /* wct = 10 + SetupCount */
	struct trans2_resp t2;
	__le16 ByteCount;
	__u8 Pad;       /* may be three bytes? *//* followed by data area */
	__u8 Buffer[0];
} __packed;

struct file_internal_info {
	__le64  UniqueId; /* inode number */
} __packed;      /* level 0x3ee */

/* DeviceType Flags */
#define FILE_DEVICE_CD_ROM              0x00000002
#define FILE_DEVICE_CD_ROM_FILE_SYSTEM  0x00000003
#define FILE_DEVICE_DFS                 0x00000006
#define FILE_DEVICE_DISK                0x00000007
#define FILE_DEVICE_DISK_FILE_SYSTEM    0x00000008
#define FILE_DEVICE_FILE_SYSTEM         0x00000009
#define FILE_DEVICE_NAMED_PIPE          0x00000011
#define FILE_DEVICE_NETWORK             0x00000012
#define FILE_DEVICE_NETWORK_FILE_SYSTEM 0x00000014
#define FILE_DEVICE_NULL                0x00000015
#define FILE_DEVICE_PARALLEL_PORT       0x00000016
#define FILE_DEVICE_PRINTER             0x00000018
#define FILE_DEVICE_SERIAL_PORT         0x0000001b
#define FILE_DEVICE_STREAMS             0x0000001e
#define FILE_DEVICE_TAPE                0x0000001f
#define FILE_DEVICE_TAPE_FILE_SYSTEM    0x00000020
#define FILE_DEVICE_VIRTUAL_DISK        0x00000024
#define FILE_DEVICE_NETWORK_REDIRECTOR  0x00000028

/* Filesystem Attributes. */
#define FILE_CASE_SENSITIVE_SEARCH      0x00000001
#define FILE_CASE_PRESERVED_NAMES       0x00000002
#define FILE_UNICODE_ON_DISK            0x00000004
/* According to cifs9f, this is 4, not 8 */
/* Acconding to testing, this actually sets the security attribute! */
#define FILE_PERSISTENT_ACLS            0x00000008
#define FILE_FILE_COMPRESSION           0x00000010
#define FILE_VOLUME_QUOTAS              0x00000020
#define FILE_SUPPORTS_SPARSE_FILES      0x00000040
#define FILE_SUPPORTS_REPARSE_POINTS    0x00000080
#define FILE_SUPPORTS_REMOTE_STORAGE    0x00000100
#define FS_LFN_APIS                     0x00004000
#define FILE_VOLUME_IS_COMPRESSED       0x00008000
#define FILE_SUPPORTS_OBJECT_IDS        0x00010000
#define FILE_SUPPORTS_ENCRYPTION        0x00020000
#define FILE_NAMED_STREAMS              0x00040000
#define FILE_READ_ONLY_VOLUME           0x00080000

/* PathInfo/FileInfo infolevels */
#define SMB_INFO_STANDARD                   1
#define SMB_SET_FILE_EA                     2
#define SMB_QUERY_FILE_EA_SIZE              2
#define SMB_INFO_QUERY_EAS_FROM_LIST        3
#define SMB_INFO_QUERY_ALL_EAS              4
#define SMB_INFO_IS_NAME_VALID              6
#define SMB_QUERY_FILE_BASIC_INFO       0x101
#define SMB_QUERY_FILE_STANDARD_INFO    0x102
#define SMB_QUERY_FILE_EA_INFO          0x103
#define SMB_QUERY_FILE_NAME_INFO        0x104
#define SMB_QUERY_FILE_ALLOCATION_INFO  0x105
#define SMB_QUERY_FILE_END_OF_FILEINFO  0x106
#define SMB_QUERY_FILE_ALL_INFO         0x107
#define SMB_QUERY_ALT_NAME_INFO         0x108
#define SMB_QUERY_FILE_STREAM_INFO      0x109
#define SMB_QUERY_FILE_COMPRESSION_INFO 0x10B
#define SMB_QUERY_FILE_UNIX_BASIC       0x200
#define SMB_QUERY_FILE_UNIX_LINK        0x201
#define SMB_QUERY_POSIX_ACL             0x204
#define SMB_QUERY_XATTR                 0x205  /* e.g. system EA name space */
#define SMB_QUERY_ATTR_FLAGS            0x206  /* append,immutable etc. */
#define SMB_QUERY_POSIX_PERMISSION      0x207
#define SMB_QUERY_POSIX_LOCK            0x208
/* #define SMB_POSIX_OPEN               0x209 */
/* #define SMB_POSIX_UNLINK             0x20a */
#define SMB_QUERY_FILE__UNIX_INFO2      0x20b
#define SMB_QUERY_FILE_INTERNAL_INFO    0x3ee
#define SMB_QUERY_FILE_ACCESS_INFO      0x3f0
#define SMB_QUERY_FILE_NAME_INFO2       0x3f1 /* 0x30 bytes */
#define SMB_QUERY_FILE_POSITION_INFO    0x3f6
#define SMB_QUERY_FILE_MODE_INFO        0x3f8
#define SMB_QUERY_FILE_ALGN_INFO        0x3f9


#define SMB_SET_FILE_BASIC_INFO         0x101
#define SMB_SET_FILE_DISPOSITION_INFO   0x102
#define SMB_SET_FILE_ALLOCATION_INFO    0x103
#define SMB_SET_FILE_END_OF_FILE_INFO   0x104
#define SMB_SET_FILE_UNIX_BASIC         0x200
#define SMB_SET_FILE_UNIX_LINK          0x201
#define SMB_SET_FILE_UNIX_HLINK         0x203
#define SMB_SET_POSIX_ACL               0x204
#define SMB_SET_XATTR                   0x205
#define SMB_SET_ATTR_FLAGS              0x206  /* append, immutable etc. */
#define SMB_SET_POSIX_LOCK              0x208
#define SMB_POSIX_OPEN                  0x209
#define SMB_POSIX_UNLINK                0x20a
#define SMB_SET_FILE_UNIX_INFO2         0x20b
#define SMB_SET_FILE_BASIC_INFO2        0x3ec
#define SMB_SET_FILE_RENAME_INFORMATION 0x3f2 /* BB check if qpathinfo too */
#define SMB_SET_FILE_DISPOSITION_INFORMATION   0x3f5   /* alias for 0x102 */
#define SMB_FILE_ALL_INFO2              0x3fa
#define SMB_SET_FILE_ALLOCATION_INFO2   0x3fb
#define SMB_SET_FILE_END_OF_FILE_INFO2  0x3fc
#define SMB_FILE_MOVE_CLUSTER_INFO      0x407
#define SMB_FILE_QUOTA_INFO             0x408
#define SMB_FILE_REPARSEPOINT_INFO      0x409
#define SMB_FILE_MAXIMUM_INFO           0x40d

/* Find File infolevels */
#define SMB_FIND_FILE_INFO_STANDARD       0x001
#define SMB_FIND_FILE_QUERY_EA_SIZE       0x002
#define SMB_FIND_FILE_QUERY_EAS_FROM_LIST 0x003
#define SMB_FIND_FILE_DIRECTORY_INFO      0x101
#define SMB_FIND_FILE_FULL_DIRECTORY_INFO 0x102
#define SMB_FIND_FILE_NAMES_INFO          0x103
#define SMB_FIND_FILE_BOTH_DIRECTORY_INFO 0x104
#define SMB_FIND_FILE_ID_FULL_DIR_INFO    0x105
#define SMB_FIND_FILE_ID_BOTH_DIR_INFO    0x106
#define SMB_FIND_FILE_UNIX                0x202

struct smb_com_trans2_qpi_req {
	struct smb_hdr hdr;     /* wct = 14+ */
	__le16 TotalParameterCount;
	__le16 TotalDataCount;
	__le16 MaxParameterCount;
	__le16 MaxDataCount;
	__u8 MaxSetupCount;
	__u8 Reserved;
	__le16 Flags;
	__le32 Timeout;
	__u16 Reserved2;
	__le16 ParameterCount;
	__le16 ParameterOffset;
	__le16 DataCount;
	__le16 DataOffset;
	__u8 SetupCount;
	__u8 Reserved3;
	__le16 SubCommand;      /* one setup word */
	__le16 ByteCount;
	__u8 Pad;
	__le16 InformationLevel;
	__u32 Reserved4;
	char FileName[1];
} __packed;

struct trans2_qpi_req_params {
	__le16 InformationLevel;
	__u32 Reserved4;
	char FileName[1];
} __packed;

/******************************************************************************/
/* QueryFileInfo/QueryPathinfo (also for SetPath/SetFile) data buffer formats */
/******************************************************************************/
struct file_basic_info {
	__le64 CreationTime;
	__le64 LastAccessTime;
	__le64 LastWriteTime;
	__le64 ChangeTime;
	__le32 Attributes;
	__u32 Pad;
} __packed;      /* size info, level 0x101 */

struct file_standard_info {
	__le64 AllocationSize;
	__le64 EndOfFile;
	__le32 NumberOfLinks;
	__u8 DeletePending;
	__u8 Directory;
	__le16 Reserved;
} __packed;

struct file_ea_info {
	__le32 EaSize;
} __packed;

struct alt_name_info {
	__le32 FileNameLength;
	char FileName[1];
} __packed;

struct file_name_info {
	__le32 FileNameLength;
	char FileName[1];
} __packed;

/* data block encoding of response to level 263 QPathInfo */
struct file_all_info {
	__le64 CreationTime;
	__le64 LastAccessTime;
	__le64 LastWriteTime;
	__le64 ChangeTime;
	__le32 Attributes;
	__u32 Pad1;
	__le64 AllocationSize;
	__le64 EndOfFile;       /* size ie offset to first free byte in file */
	__le32 NumberOfLinks;   /* hard links */
	__u8 DeletePending;
	__u8 Directory;
	__u16 Pad2;
	__le32 EASize;
	__le32 FileNameLength;
	char FileName[1];
} __packed; /* level 0x107 QPathInfo */

/* set path info/open file */
/* defines for enumerating possible values of the Unix type field below */
#define UNIX_FILE      0
#define UNIX_DIR       1
#define UNIX_SYMLINK   2
#define UNIX_CHARDEV   3
#define UNIX_BLOCKDEV  4
#define UNIX_FIFO      5
#define UNIX_SOCKET    6
#define UNIX_UNKNOWN   0xFFFFFFFF

struct file_unix_basic_info {
	__le64 EndOfFile;
	__le64 NumOfBytes;
	__le64 LastStatusChange; /*SNIA specs DCE time for the 3 time fields */
	__le64 LastAccessTime;
	__le64 LastModificationTime;
	__le64 Uid;
	__le64 Gid;
	__le32 Type;
	__le64 DevMajor;
	__le64 DevMinor;
	__le64 UniqueId;
	__le64 Permissions;
	__le64 Nlinks;
} __packed; /* level 0x200 QPathInfo */

struct smb_com_trans2_spi_req {
	struct smb_hdr hdr;     /* wct = 15 */
	__le16 TotalParameterCount;
	__le16 TotalDataCount;
	__le16 MaxParameterCount;
	__le16 MaxDataCount;
	__u8 MaxSetupCount;
	__u8 Reserved;
	__le16 Flags;
	__le32 Timeout;
	__u16 Reserved2;
	__le16 ParameterCount;
	__le16 ParameterOffset;
	__le16 DataCount;
	__le16 DataOffset;
	__u8 SetupCount;
	__u8 Reserved3;
	__le16 SubCommand;      /* one setup word */
	__le16 ByteCount;
	__u8 Pad;
	__u16 Pad1;
	__le16 InformationLevel;
	__u32 Reserved4;
	char FileName[1];
} __packed;

struct smb_com_trans2_spi_rsp {
	struct smb_hdr hdr;     /* wct = 10 + SetupCount */
	struct trans2_resp t2;
	__le16 ByteCount;
	__u16 Reserved2; /* parameter word is present for infolevels > 100 */
} __packed;

/* POSIX Open Flags */
#define SMB_O_RDONLY     0x1
#define SMB_O_WRONLY    0x2
#define SMB_O_RDWR      0x4
#define SMB_O_CREAT     0x10
#define SMB_O_EXCL      0x20
#define SMB_O_TRUNC     0x40
#define SMB_O_APPEND    0x80
#define SMB_O_SYNC      0x100
#define SMB_O_DIRECTORY 0x200
#define SMB_O_NOFOLLOW  0x400
#define SMB_O_DIRECT    0x800
#define SMB_ACCMODE	0x7

/* info level response for SMB_POSIX_PATH_OPEN */
#define SMB_NO_INFO_LEVEL_RESPONSE 0xFFFF

struct open_psx_req {
	__le32 OpenFlags; /* same as NT CreateX */
	__le32 PosixOpenFlags;
	__le64 Permissions;
	__le16 Level; /* reply level requested (see QPathInfo levels) */
} __packed; /* level 0x209 SetPathInfo data */

struct open_psx_rsp {
	__le16 OplockFlags;
	__u16 Fid;
	__le32 CreateAction;
	__le16 ReturnedLevel;
	__le16 Pad;
	/* struct following varies based on requested level */
} __packed; /* level 0x209 SetPathInfo data */

struct unlink_psx_rsp {
	__le16 EAErrorOffset;
} __packed; /* level 0x209 SetPathInfo data*/

/* Version numbers for CIFS UNIX major and minor. */
#define CIFS_UNIX_MAJOR_VERSION 1
#define CIFS_UNIX_MINOR_VERSION 0

struct filesystem_unix_info {
	__le16 MajorVersionNumber;
	__le16 MinorVersionNumber;
	__le64 Capability;
} __packed; /* Unix extension level 0x200*/

/* Linux/Unix extensions capability flags */
#define CIFS_UNIX_FCNTL_CAP             0x00000001 /* support for fcntl locks */
#define CIFS_UNIX_POSIX_ACL_CAP         0x00000002 /* support getfacl/setfacl */
#define CIFS_UNIX_XATTR_CAP             0x00000004 /* support new namespace   */
#define CIFS_UNIX_EXTATTR_CAP           0x00000008 /* support chattr/chflag   */
#define CIFS_UNIX_POSIX_PATHNAMES_CAP   0x00000010 /* Allow POSIX path chars  */
#define CIFS_UNIX_POSIX_PATH_OPS_CAP    0x00000020 /*
						    * Allow new POSIX path based
						    * calls including posix open
						    * and posix unlink
						    */
#define CIFS_UNIX_LARGE_READ_CAP        0x00000040 /*
						    * support reads >128K (up
						    * to 0xFFFF00
						    */
#define CIFS_UNIX_LARGE_WRITE_CAP       0x00000080
#define CIFS_UNIX_TRANSPORT_ENCRYPTION_CAP 0x00000100 /* can do SPNEGO crypt */
#define CIFS_UNIX_TRANSPORT_ENCRYPTION_MANDATORY_CAP  0x00000200 /* must do  */
#define CIFS_UNIX_PROXY_CAP             0x00000400 /*
						    * Proxy cap: 0xACE ioctl and
						    * QFS PROXY call
						    */
#ifdef CONFIG_CIFS_POSIX
/* presumably don't need the 0x20 POSIX_PATH_OPS_CAP since we never send
 * LockingX instead of posix locking call on unix sess (and we do not expect
 * LockingX to use different (ie Windows) semantics than posix locking on
 * the same session (if WINE needs to do this later, we can add this cap
 * back in later
 */

/* #define CIFS_UNIX_CAP_MASK              0x000000fb */
#define CIFS_UNIX_CAP_MASK              0x000003db
#else
#define CIFS_UNIX_CAP_MASK              0x00000013
#endif /* CONFIG_CIFS_POSIX */


#define CIFS_POSIX_EXTENSIONS           0x00000010 /* support for new QFSInfo */

/* Our server caps */

#define SMB_UNIX_CAPS	(CIFS_UNIX_FCNTL_CAP | CIFS_UNIX_POSIX_ACL_CAP | \
		CIFS_UNIX_XATTR_CAP | CIFS_UNIX_POSIX_PATHNAMES_CAP| \
		CIFS_UNIX_POSIX_PATH_OPS_CAP | CIFS_UNIX_LARGE_READ_CAP | \
		CIFS_UNIX_LARGE_WRITE_CAP)

#define SMB_SET_CIFS_UNIX_INFO    0x200
/* Level 0x200 request structure follows */
struct smb_com_trans2_setfsi_req {
	struct smb_hdr hdr;     /* wct = 15 */
	__le16 TotalParameterCount;
	__le16 TotalDataCount;
	__le16 MaxParameterCount;
	__le16 MaxDataCount;
	__u8 MaxSetupCount;
	__u8 Reserved;
	__le16 Flags;
	__le32 Timeout;
	__u16 Reserved2;
	__le16 ParameterCount;  /* 4 */
	__le16 ParameterOffset;
	__le16 DataCount;       /* 12 */
	__le16 DataOffset;
	__u8 SetupCount;        /* one */
	__u8 Reserved3;
	__le16 SubCommand;      /* TRANS2_SET_FS_INFORMATION */
	__le16 ByteCount;
	__u8 Pad;
	__u16 FileNum;          /* Parameters start. */
	__le16 InformationLevel;/* Parameters end. */
	__le16 ClientUnixMajor; /* Data start. */
	__le16 ClientUnixMinor;
	__le64 ClientUnixCap;   /* Data end */
} __packed;

/* response for setfsinfo levels 0x200 and 0x203 */
struct smb_com_trans2_setfsi_rsp {
	struct smb_hdr hdr;     /* wct = 10 */
	struct trans2_resp t2;
	__le16 ByteCount;
} __packed;

struct smb_com_trans2_setfsi_req_params {
	__u16 FileNum;
	__le16 InformationLevel;
	__le16 ClientUnixMajor; /* Data start. */
	__le16 ClientUnixMinor;
	__le64 ClientUnixCap;   /* Data end */
} __packed;

struct smb_trans2_qfi_req_params {
	__u16   Fid;
	__le16  InformationLevel;
} __packed;

/* FIND FIRST2 and FIND NEXT2 INFORMATION Level Codes*/

struct find_info_standard {
	__le16 CreationDate; /* SMB Date see above */
	__le16 CreationTime; /* SMB Time */
	__le16 LastAccessDate;
	__le16 LastAccessTime;
	__le16 LastWriteDate;
	__le16 LastWriteTime;
	__le32 DataSize; /* File Size (EOF) */
	__le32 AllocationSize;
	__le16 Attributes; /* verify not u32 */
	__le16 FileNameLength;
	char FileName[1];
} __packed;

struct find_info_query_ea_size {
	__le16 CreationDate; /* SMB Date see above */
	__le16 CreationTime; /* SMB Time */
	__le16 LastAccessDate;
	__le16 LastAccessTime;
	__le16 LastWriteDate;
	__le16 LastWriteTime;
	__le32 DataSize; /* File Size (EOF) */
	__le32 AllocationSize;
	__le16 Attributes; /* verify not u32 */
	__le32 EASize;
	__u8 FileNameLength;
	char FileName[1];
} __packed;

struct file_unix_info {
	__le32 NextEntryOffset;
	__u32 ResumeKey; /* as with FileIndex - no need to convert */
	struct file_unix_basic_info basic;
	char FileName[1];
} __packed; /* level 0x202 */

struct smb_com_trans2_sfi_req {
	struct smb_hdr hdr;     /* wct = 15 */
	__le16 TotalParameterCount;
	__le16 TotalDataCount;
	__le16 MaxParameterCount;
	__le16 MaxDataCount;
	__u8 MaxSetupCount;
	__u8 Reserved;
	__le16 Flags;
	__le32 Timeout;
	__u16 Reserved2;
	__le16 ParameterCount;
	__le16 ParameterOffset;
	__le16 DataCount;
	__le16 DataOffset;
	__u8 SetupCount;
	__u8 Reserved3;
	__le16 SubCommand;      /* one setup word */
	__le16 ByteCount;
	__u8 Pad;
	__u16 Pad1;
	__u16 Fid;
	__le16 InformationLevel;
	__u16 Reserved4;
} __packed;

struct smb_com_trans2_sfi_rsp {
	struct smb_hdr hdr;     /* wct = 10 + SetupCount */
	struct trans2_resp t2;
	__le16 ByteCount;
	__u16 Reserved2;        /*
				 * parameter word reserved -
				 * present for infolevels > 100
				 */
} __packed;

struct file_end_of_file_info {
	__le64 FileSize;                /* offset to end of file */
} __packed; /* size info, level 0x104 for set, 0x106 for query */

struct smb_com_create_directory_req {
	struct smb_hdr hdr;	/* wct = 0 */
	__le16 ByteCount;
	__u8 BufferFormat;	/* 4 = ASCII */
	unsigned char DirName[1];
} __packed;

struct smb_com_create_directory_rsp {
	struct smb_hdr hdr;	/* wct = 0 */
	__le16 ByteCount;	/* bct = 0 */
} __packed;

struct smb_com_check_directory_req {
	struct smb_hdr hdr;	/* wct = 0 */
	__le16 ByteCount;
	__u8 BufferFormat;	/* 4 = ASCII */
	unsigned char DirName[1];
} __packed;

struct smb_com_check_directory_rsp {
	struct smb_hdr hdr;	/* wct = 0 */
	__le16 ByteCount;	/* bct = 0 */
} __packed;

struct smb_com_process_exit_rsp {
	struct smb_hdr hdr;	/* wct = 0 */
	__le16 ByteCount;	/* bct = 0 */
} __packed;

struct smb_com_delete_directory_req {
	struct smb_hdr hdr;     /* wct = 0 */
	__le16 ByteCount;
	__u8 BufferFormat;      /* 4 = ASCII */
	unsigned char DirName[1];
} __packed;

struct smb_com_delete_directory_rsp {
	struct smb_hdr hdr;     /* wct = 0 */
	__le16 ByteCount;        /* bct = 0 */
} __packed;

struct smb_com_delete_file_req {
	struct smb_hdr hdr;     /* wct = 1 */
	__le16 SearchAttributes;
	__le16 ByteCount;
	__u8 BufferFormat;      /* 4 = ASCII */
	unsigned char fileName[1];
} __packed;

struct smb_com_delete_file_rsp {
	struct smb_hdr hdr;     /* wct = 0 */
	__le16 ByteCount;        /* bct = 0 */
} __packed;

#define CREATE_HARD_LINK         0x103

struct smb_com_nt_rename_req {  /* A5 - also used for create hardlink */
	struct smb_hdr hdr;     /* wct = 4 */
	__le16 SearchAttributes;        /* target file attributes */
	__le16 Flags;           /* spec says Information Level */
	__le32 ClusterCount;
	__le16 ByteCount;
	__u8 BufferFormat;      /* 4 = ASCII or Unicode */
	unsigned char OldFileName[1];
	/* followed by __u8 BufferFormat2 */
	/* followed by NewFileName */
} __packed;

struct smb_com_query_information_req {
	struct smb_hdr hdr;     /* wct = 0 */
	__le16 ByteCount;       /* 1 + namelen + 1 */
	__u8 BufferFormat;      /* 4 = ASCII */
	unsigned char FileName[1];
} __packed;

struct smb_com_query_information_rsp {
	struct smb_hdr hdr;     /* wct = 10 */
	__le16 attr;
	__le32  last_write_time;
	__le32 size;
	__u16  reserved[5];
	__le16 ByteCount;       /* bcc = 0 */
} __packed;

struct smb_com_findclose_req {
	struct smb_hdr hdr; /* wct = 1 */
	__u16 FileID;
	__le16 ByteCount;    /* 0 */
} __packed;

#define SMBOPEN_DISPOSITION_NONE        0
#define SMBOPEN_LOCK_GRANTED            0x8000

#define SMB_DA_ACCESS_READ              0
#define SMB_DA_ACCESS_WRITE             0x0001
#define SMB_DA_ACCESS_READ_WRITE        0x0002

/*
 * Flags on SMB open
 */
#define SMBOPEN_WRITE_THROUGH 0x4000
#define SMBOPEN_DENY_ALL      0x0010
#define SMBOPEN_DENY_WRITE    0x0020
#define SMBOPEN_DENY_READ     0x0030
#define SMBOPEN_DENY_NONE     0x0040
#define SMBOPEN_SHARING_MODE  (SMBOPEN_DENY_ALL |	\
				SMBOPEN_DENY_WRITE |	\
				SMBOPEN_DENY_READ |	\
				SMBOPEN_DENY_NONE)
#define SMBOPEN_READ          0x0000
#define SMBOPEN_WRITE         0x0001
#define SMBOPEN_READWRITE     0x0002
#define SMBOPEN_EXECUTE       0x0003

#define SMBOPEN_OCREATE       0x0010
#define SMBOPEN_OTRUNC        0x0002
#define SMBOPEN_OAPPEND       0x0001

/* format of legacy open request */
struct smb_com_openx_req {
	struct smb_hdr  hdr;    /* wct = 15 */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__le16 OpenFlags;
	__le16 Mode;
	__le16 Sattr; /* search attributes */
	__le16 FileAttributes;  /* dos attrs */
	__le32 CreateTime; /* os2 format */
	__le16 OpenFunction;
	__le32 EndOfFile;
	__le32 Timeout;
	__le32 Reserved;
	__le16  ByteCount;  /* file name follows */
	char   fileName[1];
} __packed;

struct smb_com_openx_rsp {
	struct smb_hdr  hdr;    /* wct = 15 */
	__u8 AndXCommand;
	__u8 AndXReserved;
	__le16 AndXOffset;
	__u16  Fid;
	__le16 FileAttributes;
	__le32 LastWriteTime; /* os2 format */
	__le32 EndOfFile;
	__le16 Access;
	__le16 FileType;
	__le16 IPCState;
	__le16 Action;
	__u32  FileId;
	__u16  Reserved;
	__le16 ByteCount;
} __packed;

struct filesystem_alloc_info {
	__le32 fsid;
	__le32 SectorsPerAllocationUnit;
	__le32 TotalAllocationUnits;
	__le32 FreeAllocationUnits;
	__le16  BytesPerSector;
} __packed;

struct file_allocation_info {
	__le64 AllocationSize; /* Note old Samba srvr rounds this up too much */
} __packed;      /* size used on disk: 0x103 for set, 0x105 for query */

struct file_info_standard {
	__le16 CreationDate; /* SMB Date see above */
	__le16 CreationTime; /* SMB Time */
	__le16 LastAccessDate;
	__le16 LastAccessTime;
	__le16 LastWriteDate;
	__le16 LastWriteTime;
	__le32 DataSize; /* File Size (EOF) */
	__le32 AllocationSize;
	__le16 Attributes; /* verify not u32 */
	__le32 EASize;
} __packed;  /* level 1 SetPath/FileInfo */

#define CIFS_MF_SYMLINK_LINK_MAXLEN (1024)

struct set_file_rename {
	__le32 overwrite;   /* 1 = overwrite dest */
	__u32 root_fid;   /* zero */
	__le32 target_name_len;
	char  target_name[0];  /* Must be unicode */
} __packed;

struct fea {
	unsigned char EA_flags;
	__u8 name_len;
	__le16 value_len;
	char name[1];
	/* optionally followed by value */
} __packed;

struct fealist {
	__le32 list_len;
	__u8 list[1];
} __packed;

/* POSIX ACL set/query path info structures */
#define CIFS_ACL_VERSION 1
struct cifs_posix_ace { /* access control entry (ACE) */
	__u8  cifs_e_tag;
	__u8  cifs_e_perm;
	__le64 cifs_uid; /* or gid */
} __packed;

struct cifs_posix_acl { /* access conrol list  (ACL) */
	__le16  version;
	__le16  access_entry_count;  /* access ACL - count of entries */
	__le16  default_entry_count; /* default ACL - count of entries */
	struct cifs_posix_ace ace_array[0];
	/*
	 * followed by
	 * struct cifs_posix_ace default_ace_arraay[]
	 */
} __packed;  /* level 0x204 */

struct smb_com_setattr_req {
	struct smb_hdr hdr; /* wct = 8 */
	__le16 attr;
	__le32 LastWriteTime;
	__le16 reserved[5]; /* must be zero */
	__le16 ByteCount;
	__u8   BufferFormat; /* 4 = ASCII */
	unsigned char fileName[1];
} __packed;

struct smb_com_setattr_rsp {
	struct smb_hdr hdr;     /* wct = 0 */
	__le16 ByteCount;        /* bct = 0 */
} __packed;

#ifdef CONFIG_SMB_INSECURE_SERVER
extern int init_smb1_server(struct ksmbd_conn *conn);
#endif

/* function prototypes */
extern int init_smb_rsp_hdr(struct ksmbd_work *work);
extern u16 get_smb_cmd_val(struct ksmbd_work *work);
extern void set_smb_rsp_status(struct ksmbd_work *work, __le32 err);
extern int smb_allocate_rsp_buf(struct ksmbd_work *work);
extern bool smb1_is_sign_req(struct ksmbd_work *work, unsigned int command);
extern int smb1_check_sign_req(struct ksmbd_work *work);
extern void smb1_set_sign_rsp(struct ksmbd_work *work);
extern int smb_check_user_session(struct ksmbd_work *work);
extern int smb_get_ksmbd_tcon(struct ksmbd_work *work);
extern int ksmbd_smb1_check_message(struct ksmbd_work *work);

/* smb1 command handlers */
extern int smb_rename(struct ksmbd_work *work);
extern int smb_negotiate_request(struct ksmbd_work *work);
#ifdef CONFIG_SMB_INSECURE_SERVER
extern int smb_handle_negotiate(struct ksmbd_work *work);
#endif
extern int smb_session_setup_andx(struct ksmbd_work *work);
extern int smb_tree_connect_andx(struct ksmbd_work *work);
extern int smb_trans2(struct ksmbd_work *work);
extern int smb_nt_create_andx(struct ksmbd_work *work);
extern int smb_trans(struct ksmbd_work *work);
extern int smb_locking_andx(struct ksmbd_work *work);
extern int smb_close(struct ksmbd_work *work);
extern int smb_read_andx(struct ksmbd_work *work);
extern int smb_tree_disconnect(struct ksmbd_work *work);
extern int smb_session_disconnect(struct ksmbd_work *work);
extern int smb_write_andx(struct ksmbd_work *work);
extern int smb_echo(struct ksmbd_work *work);
extern int smb_flush(struct ksmbd_work *work);
extern int smb_mkdir(struct ksmbd_work *work);
extern int smb_rmdir(struct ksmbd_work *work);
extern int smb_unlink(struct ksmbd_work *work);
extern int smb_nt_cancel(struct ksmbd_work *work);
extern int smb_nt_rename(struct ksmbd_work *work);
extern int smb_query_info(struct ksmbd_work *work);
extern int smb_closedir(struct ksmbd_work *work);
extern int smb_open_andx(struct ksmbd_work *work);
extern int smb_write(struct ksmbd_work *work);
extern int smb_setattr(struct ksmbd_work *work);
extern int smb_query_information_disk(struct ksmbd_work *work);
extern int smb_checkdir(struct ksmbd_work *work);
extern int smb_process_exit(struct ksmbd_work *work);
#endif /* __SMB1PDU_H */
