// SPDX-License-Identifier: LGPL-2.1+
/*
 *   Copyright (C) International Business Machines  Corp., 2007,2008
 *   Author(s): Steve French (sfrench@us.ibm.com)
 *   Copyright (C) 2020 Samsung Electronics Co., Ltd.
 *   Author(s): Namjae Jeon <linkinjeon@kernel.org>
 */

#include <linux/version.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/string.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0)
#include <linux/mnt_idmapping.h>
#endif

#include "smbacl.h"
#include "smb_common.h"
#include "server.h"
#include "misc.h"
#include "mgmt/share_config.h"

static const struct smb_sid domain = {1, 4, {0, 0, 0, 0, 0, 5},
	{cpu_to_le32(21), cpu_to_le32(1), cpu_to_le32(2), cpu_to_le32(3),
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} };

/* security id for everyone/world system group */
static const struct smb_sid creator_owner = {
	1, 1, {0, 0, 0, 0, 0, 3}, {0} };
/* security id for everyone/world system group */
static const struct smb_sid creator_group = {
	1, 1, {0, 0, 0, 0, 0, 3}, {cpu_to_le32(1)} };

/* security id for everyone/world system group */
static const struct smb_sid sid_everyone = {
	1, 1, {0, 0, 0, 0, 0, 1}, {0} };
/* security id for Authenticated Users system group */
static const struct smb_sid sid_authusers = {
	1, 1, {0, 0, 0, 0, 0, 5}, {cpu_to_le32(11)} };

/* S-1-22-1 Unmapped Unix users */
static const struct smb_sid sid_unix_users = {1, 1, {0, 0, 0, 0, 0, 22},
		{cpu_to_le32(1), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} };

/* S-1-22-2 Unmapped Unix groups */
static const struct smb_sid sid_unix_groups = { 1, 1, {0, 0, 0, 0, 0, 22},
		{cpu_to_le32(2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} };

/*
 * See http://technet.microsoft.com/en-us/library/hh509017(v=ws.10).aspx
 */

/* S-1-5-88 MS NFS and Apple style UID/GID/mode */

/* S-1-5-88-1 Unix uid */
static const struct smb_sid sid_unix_NFS_users = { 1, 2, {0, 0, 0, 0, 0, 5},
	{cpu_to_le32(88),
	 cpu_to_le32(1), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} };

/* S-1-5-88-2 Unix gid */
static const struct smb_sid sid_unix_NFS_groups = { 1, 2, {0, 0, 0, 0, 0, 5},
	{cpu_to_le32(88),
	 cpu_to_le32(2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} };

/* S-1-5-88-3 Unix mode */
static const struct smb_sid sid_unix_NFS_mode = { 1, 2, {0, 0, 0, 0, 0, 5},
	{cpu_to_le32(88),
	 cpu_to_le32(3), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} };

/*
 * if the two SIDs (roughly equivalent to a UUID for a user or group) are
 * the same returns zero, if they do not match returns non-zero.
 */
int compare_sids(const struct smb_sid *ctsid, const struct smb_sid *cwsid)
{
	int i;
	int num_subauth, num_sat, num_saw;

	if (!ctsid || !cwsid)
		return 1;

	/* compare the revision */
	if (ctsid->revision != cwsid->revision) {
		if (ctsid->revision > cwsid->revision)
			return 1;
		else
			return -1;
	}

	/* compare all of the six auth values */
	for (i = 0; i < NUM_AUTHS; ++i) {
		if (ctsid->authority[i] != cwsid->authority[i]) {
			if (ctsid->authority[i] > cwsid->authority[i])
				return 1;
			else
				return -1;
		}
	}

	/* compare all of the subauth values if any */
	num_sat = ctsid->num_subauth;
	num_saw = cwsid->num_subauth;
	num_subauth = num_sat < num_saw ? num_sat : num_saw;
	if (num_subauth) {
		for (i = 0; i < num_subauth; ++i) {
			if (ctsid->sub_auth[i] != cwsid->sub_auth[i]) {
				if (le32_to_cpu(ctsid->sub_auth[i]) >
				    le32_to_cpu(cwsid->sub_auth[i]))
					return 1;
				else
					return -1;
			}
		}
	}

	return 0; /* sids compare/match */
}

static void smb_copy_sid(struct smb_sid *dst, const struct smb_sid *src)
{
	int i;

	dst->revision = src->revision;
	dst->num_subauth = min_t(u8, src->num_subauth, SID_MAX_SUB_AUTHORITIES);
	for (i = 0; i < NUM_AUTHS; ++i)
		dst->authority[i] = src->authority[i];
	for (i = 0; i < dst->num_subauth; ++i)
		dst->sub_auth[i] = src->sub_auth[i];
}

/*
 * change posix mode to reflect permissions
 * pmode is the existing mode (we only want to overwrite part of this
 * bits to set can be: S_IRWXU, S_IRWXG or S_IRWXO ie 00700 or 00070 or 00007
 */
static umode_t access_flags_to_mode(struct smb_fattr *fattr, __le32 ace_flags,
				    int type)
{
	__u32 flags = le32_to_cpu(ace_flags);
	umode_t mode = 0;

	if (flags & GENERIC_ALL) {
		mode = 0777;
		ksmbd_debug(SMB, "all perms\n");
		return mode;
	}

	if ((flags & GENERIC_READ) || (flags & FILE_READ_RIGHTS))
		mode = 0444;
	if ((flags & GENERIC_WRITE) || (flags & FILE_WRITE_RIGHTS)) {
		mode |= 0222;
		if (S_ISDIR(fattr->cf_mode))
			mode |= 0111;
	}
	if ((flags & GENERIC_EXECUTE) || (flags & FILE_EXEC_RIGHTS))
		mode |= 0111;

	if (type == ACCESS_DENIED_ACE_TYPE || type == ACCESS_DENIED_OBJECT_ACE_TYPE)
		mode = ~mode;

	ksmbd_debug(SMB, "access flags 0x%x mode now %04o\n", flags, mode);

	return mode;
}

/*
 * Generate access flags to reflect permissions mode is the existing mode.
 * This function is called for every ACE in the DACL whose SID matches
 * with either owner or group or everyone.
 */
static void mode_to_access_flags(umode_t mode, umode_t bits_to_use,
				 __u32 *pace_flags)
{
	/* reset access mask */
	*pace_flags = 0x0;

	/* bits to use are either S_IRWXU or S_IRWXG or S_IRWXO */
	mode &= bits_to_use;

	/*
	 * check for R/W/X UGO since we do not know whose flags
	 * is this but we have cleared all the bits sans RWX for
	 * either user or group or other as per bits_to_use
	 */
	if (mode & 0444)
		*pace_flags |= SET_FILE_READ_RIGHTS;
	if (mode & 0222)
		*pace_flags |= FILE_WRITE_RIGHTS;
	if (mode & 0111)
		*pace_flags |= SET_FILE_EXEC_RIGHTS;

	ksmbd_debug(SMB, "mode: %o, access flags now 0x%x\n",
		    mode, *pace_flags);
}

static __u16 fill_ace_for_sid(struct smb_ace *pntace,
			      const struct smb_sid *psid, int type, int flags,
			      umode_t mode, umode_t bits)
{
	int i;
	__u16 size = 0;
	__u32 access_req = 0;

	pntace->type = type;
	pntace->flags = flags;
	mode_to_access_flags(mode, bits, &access_req);
	if (!access_req)
		access_req = SET_MINIMUM_RIGHTS;
	pntace->access_req = cpu_to_le32(access_req);

	pntace->sid.revision = psid->revision;
	pntace->sid.num_subauth = psid->num_subauth;
	for (i = 0; i < NUM_AUTHS; i++)
		pntace->sid.authority[i] = psid->authority[i];
	for (i = 0; i < psid->num_subauth; i++)
		pntace->sid.sub_auth[i] = psid->sub_auth[i];

	size = 1 + 1 + 2 + 4 + 1 + 1 + 6 + (psid->num_subauth * 4);
	pntace->size = cpu_to_le16(size);

	return size;
}

void id_to_sid(unsigned int cid, uint sidtype, struct smb_sid *ssid)
{
	switch (sidtype) {
	case SIDOWNER:
		smb_copy_sid(ssid, &server_conf.domain_sid);
		break;
	case SIDUNIX_USER:
		smb_copy_sid(ssid, &sid_unix_users);
		break;
	case SIDUNIX_GROUP:
		smb_copy_sid(ssid, &sid_unix_groups);
		break;
	case SIDCREATOR_OWNER:
		smb_copy_sid(ssid, &creator_owner);
		return;
	case SIDCREATOR_GROUP:
		smb_copy_sid(ssid, &creator_group);
		return;
	case SIDNFS_USER:
		smb_copy_sid(ssid, &sid_unix_NFS_users);
		break;
	case SIDNFS_GROUP:
		smb_copy_sid(ssid, &sid_unix_NFS_groups);
		break;
	case SIDNFS_MODE:
		smb_copy_sid(ssid, &sid_unix_NFS_mode);
		break;
	default:
		return;
	}

	/* RID */
	ssid->sub_auth[ssid->num_subauth] = cpu_to_le32(cid);
	ssid->num_subauth++;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static int sid_to_id(struct mnt_idmap *idmap,
#else
static int sid_to_id(struct user_namespace *user_ns,
#endif
		     struct smb_sid *psid, uint sidtype,
		     struct smb_fattr *fattr)
{
	int rc = -EINVAL;

	/*
	 * If we have too many subauthorities, then something is really wrong.
	 * Just return an error.
	 */
	if (unlikely(psid->num_subauth > SID_MAX_SUB_AUTHORITIES)) {
		pr_err("%s: %u subauthorities is too many!\n",
		       __func__, psid->num_subauth);
		return -EIO;
	}

	if (!compare_sids(psid, &sid_everyone))
		return -EIO;

	if (sidtype == SIDOWNER) {
		kuid_t uid;
		uid_t id;

		id = le32_to_cpu(psid->sub_auth[psid->num_subauth - 1]);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0) || \
    (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 52) && LINUX_VERSION_CODE < KERNEL_VERSION(5, 16, 0))
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
		uid = KUIDT_INIT(id);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		uid = from_vfsuid(idmap, &init_user_ns, VFSUIDT_INIT(uid));
#else
		uid = from_vfsuid(user_ns, &init_user_ns, VFSUIDT_INIT(uid));
#endif
#else
		uid = mapped_kuid_user(user_ns, &init_user_ns, KUIDT_INIT(id));
#endif
#else
		/*
		 * Translate raw sid into kuid in the server's user
		 * namespace.
		 */
		uid = make_kuid(&init_user_ns, id);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
		/* If this is an idmapped mount, apply the idmapping. */
		uid = kuid_from_mnt(user_ns, uid);
#endif
#endif
		if (uid_valid(uid)) {
			fattr->cf_uid = uid;
			rc = 0;
		}
	} else {
		kgid_t gid;
		gid_t id;

		id = le32_to_cpu(psid->sub_auth[psid->num_subauth - 1]);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0) || \
    (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 52) && LINUX_VERSION_CODE < KERNEL_VERSION(5, 16, 0))
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
		gid = KGIDT_INIT(id);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		gid = from_vfsgid(idmap, &init_user_ns, VFSGIDT_INIT(gid));
#else
		gid = from_vfsgid(user_ns, &init_user_ns, VFSGIDT_INIT(gid));
#endif
#else
		gid = mapped_kgid_user(user_ns, &init_user_ns, KGIDT_INIT(id));
#endif
#else
		/*
		 * Translate raw sid into kgid in the server's user
		 * namespace.
		 */
		gid = make_kgid(&init_user_ns, id);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
		/* If this is an idmapped mount, apply the idmapping. */
		gid = kgid_from_mnt(user_ns, gid);
#endif
#endif
		if (gid_valid(gid)) {
			fattr->cf_gid = gid;
			rc = 0;
		}
	}

	return rc;
}

void posix_state_to_acl(struct posix_acl_state *state,
			struct posix_acl_entry *pace)
{
	int i;

	pace->e_tag = ACL_USER_OBJ;
	pace->e_perm = state->owner.allow;
	for (i = 0; i < state->users->n; i++) {
		pace++;
		pace->e_tag = ACL_USER;
		pace->e_uid = state->users->aces[i].uid;
		pace->e_perm = state->users->aces[i].perms.allow;
	}

	pace++;
	pace->e_tag = ACL_GROUP_OBJ;
	pace->e_perm = state->group.allow;

	for (i = 0; i < state->groups->n; i++) {
		pace++;
		pace->e_tag = ACL_GROUP;
		pace->e_gid = state->groups->aces[i].gid;
		pace->e_perm = state->groups->aces[i].perms.allow;
	}

	if (state->users->n || state->groups->n) {
		pace++;
		pace->e_tag = ACL_MASK;
		pace->e_perm = state->mask.allow;
	}

	pace++;
	pace->e_tag = ACL_OTHER;
	pace->e_perm = state->other.allow;
}

int init_acl_state(struct posix_acl_state *state, int cnt)
{
	int alloc;

	memset(state, 0, sizeof(struct posix_acl_state));
	/*
	 * In the worst case, each individual acl could be for a distinct
	 * named user or group, but we don't know which, so we allocate
	 * enough space for either:
	 */
	alloc = sizeof(struct posix_ace_state_array)
		+ cnt * sizeof(struct posix_user_ace_state);
	state->users = kzalloc(alloc, GFP_KERNEL);
	if (!state->users)
		return -ENOMEM;
	state->groups = kzalloc(alloc, GFP_KERNEL);
	if (!state->groups) {
		kfree(state->users);
		return -ENOMEM;
	}
	return 0;
}

void free_acl_state(struct posix_acl_state *state)
{
	kfree(state->users);
	kfree(state->groups);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static void parse_dacl(struct mnt_idmap *idmap,
#else
static void parse_dacl(struct user_namespace *user_ns,
#endif
		       struct smb_acl *pdacl, char *end_of_acl,
		       struct smb_sid *pownersid, struct smb_sid *pgrpsid,
		       struct smb_fattr *fattr)
{
	int i, ret;
	int num_aces = 0;
	unsigned int acl_size;
	char *acl_base;
	struct smb_ace **ppace;
	struct posix_acl_entry *cf_pace, *cf_pdace;
	struct posix_acl_state acl_state, default_acl_state;
	umode_t mode = 0, acl_mode;
	bool owner_found = false, group_found = false, others_found = false;

	if (!pdacl)
		return;

	/* validate that we do not go past end of acl */
	if (end_of_acl < (char *)pdacl + sizeof(struct smb_acl) ||
	    end_of_acl < (char *)pdacl + le16_to_cpu(pdacl->size)) {
		pr_err("ACL too small to parse DACL\n");
		return;
	}

	ksmbd_debug(SMB, "DACL revision %d size %d num aces %d\n",
		    le16_to_cpu(pdacl->revision), le16_to_cpu(pdacl->size),
		    le32_to_cpu(pdacl->num_aces));

	acl_base = (char *)pdacl;
	acl_size = sizeof(struct smb_acl);

	num_aces = le32_to_cpu(pdacl->num_aces);
	if (num_aces <= 0)
		return;

	if (num_aces > ULONG_MAX / sizeof(struct smb_ace *))
		return;

	ppace = kmalloc_array(num_aces, sizeof(struct smb_ace *), GFP_KERNEL);
	if (!ppace)
		return;

	ret = init_acl_state(&acl_state, num_aces);
	if (ret)
		return;
	ret = init_acl_state(&default_acl_state, num_aces);
	if (ret) {
		free_acl_state(&acl_state);
		return;
	}

	/*
	 * reset rwx permissions for user/group/other.
	 * Also, if num_aces is 0 i.e. DACL has no ACEs,
	 * user/group/other have no permissions
	 */
	for (i = 0; i < num_aces; ++i) {
		if (end_of_acl - acl_base < acl_size)
			break;

		ppace[i] = (struct smb_ace *)(acl_base + acl_size);
		acl_base = (char *)ppace[i];
		acl_size = offsetof(struct smb_ace, sid) +
			offsetof(struct smb_sid, sub_auth);

		if (end_of_acl - acl_base < acl_size ||
		    ppace[i]->sid.num_subauth > SID_MAX_SUB_AUTHORITIES ||
		    (end_of_acl - acl_base <
		     acl_size + sizeof(__le32) * ppace[i]->sid.num_subauth) ||
		    (le16_to_cpu(ppace[i]->size) <
		     acl_size + sizeof(__le32) * ppace[i]->sid.num_subauth))
			break;

		acl_size = le16_to_cpu(ppace[i]->size);
		ppace[i]->access_req =
			smb_map_generic_desired_access(ppace[i]->access_req);

		if (!(compare_sids(&ppace[i]->sid, &sid_unix_NFS_mode))) {
			fattr->cf_mode =
				le32_to_cpu(ppace[i]->sid.sub_auth[2]);
			break;
		} else if (!compare_sids(&ppace[i]->sid, pownersid)) {
			acl_mode = access_flags_to_mode(fattr,
							ppace[i]->access_req,
							ppace[i]->type);
			acl_mode &= 0700;

			if (!owner_found) {
				mode &= ~(0700);
				mode |= acl_mode;
			}
			owner_found = true;
		} else if (!compare_sids(&ppace[i]->sid, pgrpsid) ||
			   ppace[i]->sid.sub_auth[ppace[i]->sid.num_subauth - 1] ==
			    DOMAIN_USER_RID_LE) {
			acl_mode = access_flags_to_mode(fattr,
							ppace[i]->access_req,
							ppace[i]->type);
			acl_mode &= 0070;
			if (!group_found) {
				mode &= ~(0070);
				mode |= acl_mode;
			}
			group_found = true;
		} else if (!compare_sids(&ppace[i]->sid, &sid_everyone)) {
			acl_mode = access_flags_to_mode(fattr,
							ppace[i]->access_req,
							ppace[i]->type);
			acl_mode &= 0007;
			if (!others_found) {
				mode &= ~(0007);
				mode |= acl_mode;
			}
			others_found = true;
		} else if (!compare_sids(&ppace[i]->sid, &creator_owner)) {
			continue;
		} else if (!compare_sids(&ppace[i]->sid, &creator_group)) {
			continue;
		} else if (!compare_sids(&ppace[i]->sid, &sid_authusers)) {
			continue;
		} else {
			struct smb_fattr temp_fattr;

			acl_mode = access_flags_to_mode(fattr, ppace[i]->access_req,
							ppace[i]->type);
			temp_fattr.cf_uid = INVALID_UID;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			ret = sid_to_id(idmap, &ppace[i]->sid, SIDOWNER, &temp_fattr);
#else
			ret = sid_to_id(user_ns, &ppace[i]->sid, SIDOWNER, &temp_fattr);
#endif
			if (ret || uid_eq(temp_fattr.cf_uid, INVALID_UID)) {
				pr_err("%s: Error %d mapping Owner SID to uid\n",
				       __func__, ret);
				continue;
			}

			acl_state.owner.allow = ((acl_mode & 0700) >> 6) | 0004;
			acl_state.users->aces[acl_state.users->n].uid =
				temp_fattr.cf_uid;
			acl_state.users->aces[acl_state.users->n++].perms.allow =
				((acl_mode & 0700) >> 6) | 0004;
			default_acl_state.owner.allow = ((acl_mode & 0700) >> 6) | 0004;
			default_acl_state.users->aces[default_acl_state.users->n].uid =
				temp_fattr.cf_uid;
			default_acl_state.users->aces[default_acl_state.users->n++].perms.allow =
				((acl_mode & 0700) >> 6) | 0004;
		}
	}
	kfree(ppace);

	if (owner_found) {
		/* The owner must be set to at least read-only. */
		acl_state.owner.allow = ((mode & 0700) >> 6) | 0004;
		acl_state.users->aces[acl_state.users->n].uid = fattr->cf_uid;
		acl_state.users->aces[acl_state.users->n++].perms.allow =
			((mode & 0700) >> 6) | 0004;
		default_acl_state.owner.allow = ((mode & 0700) >> 6) | 0004;
		default_acl_state.users->aces[default_acl_state.users->n].uid =
			fattr->cf_uid;
		default_acl_state.users->aces[default_acl_state.users->n++].perms.allow =
			((mode & 0700) >> 6) | 0004;
	}

	if (group_found) {
		acl_state.group.allow = (mode & 0070) >> 3;
		acl_state.groups->aces[acl_state.groups->n].gid =
			fattr->cf_gid;
		acl_state.groups->aces[acl_state.groups->n++].perms.allow =
			(mode & 0070) >> 3;
		default_acl_state.group.allow = (mode & 0070) >> 3;
		default_acl_state.groups->aces[default_acl_state.groups->n].gid =
			fattr->cf_gid;
		default_acl_state.groups->aces[default_acl_state.groups->n++].perms.allow =
			(mode & 0070) >> 3;
	}

	if (others_found) {
		fattr->cf_mode &= ~(0007);
		fattr->cf_mode |= mode & 0007;

		acl_state.other.allow = mode & 0007;
		default_acl_state.other.allow = mode & 0007;
	}

	if (acl_state.users->n || acl_state.groups->n) {
		acl_state.mask.allow = 0x07;

		if (IS_ENABLED(CONFIG_FS_POSIX_ACL)) {
			fattr->cf_acls =
				posix_acl_alloc(acl_state.users->n +
					acl_state.groups->n + 4, GFP_KERNEL);
			if (fattr->cf_acls) {
				cf_pace = fattr->cf_acls->a_entries;
				posix_state_to_acl(&acl_state, cf_pace);
			}
		}
	}

	if (default_acl_state.users->n || default_acl_state.groups->n) {
		default_acl_state.mask.allow = 0x07;

		if (IS_ENABLED(CONFIG_FS_POSIX_ACL)) {
			fattr->cf_dacls =
				posix_acl_alloc(default_acl_state.users->n +
				default_acl_state.groups->n + 4, GFP_KERNEL);
			if (fattr->cf_dacls) {
				cf_pdace = fattr->cf_dacls->a_entries;
				posix_state_to_acl(&default_acl_state, cf_pdace);
			}
		}
	}
	free_acl_state(&acl_state);
	free_acl_state(&default_acl_state);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static void set_posix_acl_entries_dacl(struct mnt_idmap *idmap,
#else
static void set_posix_acl_entries_dacl(struct user_namespace *user_ns,
#endif
				       struct smb_ace *pndace,
				       struct smb_fattr *fattr, u32 *num_aces,
				       u16 *size, u32 nt_aces_num)
{
	struct posix_acl_entry *pace;
	struct smb_sid *sid;
	struct smb_ace *ntace;
	int i, j;

	if (!fattr->cf_acls)
		goto posix_default_acl;

	pace = fattr->cf_acls->a_entries;
	for (i = 0; i < fattr->cf_acls->a_count; i++, pace++) {
		int flags = 0;

		sid = kmalloc(sizeof(struct smb_sid), GFP_KERNEL);
		if (!sid)
			break;

		if (pace->e_tag == ACL_USER) {
			uid_t uid;
			unsigned int sid_type = SIDOWNER;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			uid = posix_acl_uid_translate(idmap, pace);
#else
			uid = posix_acl_uid_translate(user_ns, pace);
#endif
			if (!uid)
				sid_type = SIDUNIX_USER;
			id_to_sid(uid, sid_type, sid);
		} else if (pace->e_tag == ACL_GROUP) {
			gid_t gid;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			gid = posix_acl_gid_translate(idmap, pace);
#else
			gid = posix_acl_gid_translate(user_ns, pace);
#endif
			id_to_sid(gid, SIDUNIX_GROUP, sid);
		} else if (pace->e_tag == ACL_OTHER && !nt_aces_num) {
			smb_copy_sid(sid, &sid_everyone);
		} else {
			kfree(sid);
			continue;
		}
		ntace = pndace;
		for (j = 0; j < nt_aces_num; j++) {
			if (ntace->sid.sub_auth[ntace->sid.num_subauth - 1] ==
					sid->sub_auth[sid->num_subauth - 1])
				goto pass_same_sid;
			ntace = (struct smb_ace *)((char *)ntace +
					le16_to_cpu(ntace->size));
		}

		if (S_ISDIR(fattr->cf_mode) && pace->e_tag == ACL_OTHER)
			flags = 0x03;

		ntace = (struct smb_ace *)((char *)pndace + *size);
		*size += fill_ace_for_sid(ntace, sid, ACCESS_ALLOWED, flags,
				pace->e_perm, 0777);
		(*num_aces)++;
		if (pace->e_tag == ACL_USER)
			ntace->access_req |=
				FILE_DELETE_LE | FILE_DELETE_CHILD_LE;

		if (S_ISDIR(fattr->cf_mode) &&
		    (pace->e_tag == ACL_USER || pace->e_tag == ACL_GROUP)) {
			ntace = (struct smb_ace *)((char *)pndace + *size);
			*size += fill_ace_for_sid(ntace, sid, ACCESS_ALLOWED,
					0x03, pace->e_perm, 0777);
			(*num_aces)++;
			if (pace->e_tag == ACL_USER)
				ntace->access_req |=
					FILE_DELETE_LE | FILE_DELETE_CHILD_LE;
		}

pass_same_sid:
		kfree(sid);
	}

	if (nt_aces_num)
		return;

posix_default_acl:
	if (!fattr->cf_dacls)
		return;

	pace = fattr->cf_dacls->a_entries;
	for (i = 0; i < fattr->cf_dacls->a_count; i++, pace++) {
		sid = kmalloc(sizeof(struct smb_sid), GFP_KERNEL);
		if (!sid)
			break;

		if (pace->e_tag == ACL_USER) {
			uid_t uid;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			uid = posix_acl_uid_translate(idmap, pace);
#else
			uid = posix_acl_uid_translate(user_ns, pace);
#endif
			id_to_sid(uid, SIDCREATOR_OWNER, sid);
		} else if (pace->e_tag == ACL_GROUP) {
			gid_t gid;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			gid = posix_acl_gid_translate(idmap, pace);
#else
			gid = posix_acl_gid_translate(user_ns, pace);
#endif
			id_to_sid(gid, SIDCREATOR_GROUP, sid);
		} else {
			kfree(sid);
			continue;
		}

		ntace = (struct smb_ace *)((char *)pndace + *size);
		*size += fill_ace_for_sid(ntace, sid, ACCESS_ALLOWED, 0x0b,
				pace->e_perm, 0777);
		(*num_aces)++;
		if (pace->e_tag == ACL_USER)
			ntace->access_req |=
				FILE_DELETE_LE | FILE_DELETE_CHILD_LE;
		kfree(sid);
	}
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static void set_ntacl_dacl(struct mnt_idmap *idmap,
#else
static void set_ntacl_dacl(struct user_namespace *user_ns,
#endif
			   struct smb_acl *pndacl,
			   struct smb_acl *nt_dacl,
			   unsigned int aces_size,
			   const struct smb_sid *pownersid,
			   const struct smb_sid *pgrpsid,
			   struct smb_fattr *fattr)
{
	struct smb_ace *ntace, *pndace;
	int nt_num_aces = le32_to_cpu(nt_dacl->num_aces), num_aces = 0;
	unsigned short size = 0;
	int i;

	pndace = (struct smb_ace *)((char *)pndacl + sizeof(struct smb_acl));
	if (nt_num_aces) {
		ntace = (struct smb_ace *)((char *)nt_dacl + sizeof(struct smb_acl));
		for (i = 0; i < nt_num_aces; i++) {
			unsigned short nt_ace_size;

			if (offsetof(struct smb_ace, access_req) > aces_size)
				break;

			nt_ace_size = le16_to_cpu(ntace->size);
			if (nt_ace_size > aces_size)
				break;

			memcpy((char *)pndace + size, ntace, nt_ace_size);
			size += nt_ace_size;
			aces_size -= nt_ace_size;
			ntace = (struct smb_ace *)((char *)ntace + nt_ace_size);
			num_aces++;
		}
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	set_posix_acl_entries_dacl(idmap, pndace, fattr,
#else
	set_posix_acl_entries_dacl(user_ns, pndace, fattr,
#endif
				   &num_aces, &size, nt_num_aces);
	pndacl->num_aces = cpu_to_le32(num_aces);
	pndacl->size = cpu_to_le16(le16_to_cpu(pndacl->size) + size);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static void set_mode_dacl(struct mnt_idmap *idmap,
#else
static void set_mode_dacl(struct user_namespace *user_ns,
#endif
			  struct smb_acl *pndacl, struct smb_fattr *fattr)
{
	struct smb_ace *pace, *pndace;
	u32 num_aces = 0;
	u16 size = 0, ace_size = 0;
	uid_t uid;
	const struct smb_sid *sid;

	pace = pndace = (struct smb_ace *)((char *)pndacl + sizeof(struct smb_acl));

	if (fattr->cf_acls) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		set_posix_acl_entries_dacl(idmap, pndace, fattr,
#else
		set_posix_acl_entries_dacl(user_ns, pndace, fattr,
#endif
					   &num_aces, &size, num_aces);
		goto out;
	}

	/* owner RID */
	uid = from_kuid(&init_user_ns, fattr->cf_uid);
	if (uid)
		sid = &server_conf.domain_sid;
	else
		sid = &sid_unix_users;
	ace_size = fill_ace_for_sid(pace, sid, ACCESS_ALLOWED, 0,
				    fattr->cf_mode, 0700);
	pace->sid.sub_auth[pace->sid.num_subauth++] = cpu_to_le32(uid);
	pace->size = cpu_to_le16(ace_size + 4);
	size += le16_to_cpu(pace->size);
	pace = (struct smb_ace *)((char *)pndace + size);

	/* Group RID */
	ace_size = fill_ace_for_sid(pace, &sid_unix_groups,
				    ACCESS_ALLOWED, 0, fattr->cf_mode, 0070);
	pace->sid.sub_auth[pace->sid.num_subauth++] =
		cpu_to_le32(from_kgid(&init_user_ns, fattr->cf_gid));
	pace->size = cpu_to_le16(ace_size + 4);
	size += le16_to_cpu(pace->size);
	pace = (struct smb_ace *)((char *)pndace + size);
	num_aces = 3;

	if (S_ISDIR(fattr->cf_mode)) {
		pace = (struct smb_ace *)((char *)pndace + size);

		/* creator owner */
		size += fill_ace_for_sid(pace, &creator_owner, ACCESS_ALLOWED,
					 0x0b, fattr->cf_mode, 0700);
		pace = (struct smb_ace *)((char *)pndace + size);

		/* creator group */
		size += fill_ace_for_sid(pace, &creator_group, ACCESS_ALLOWED,
					 0x0b, fattr->cf_mode, 0070);
		pace = (struct smb_ace *)((char *)pndace + size);
		num_aces = 5;
	}

	/* other */
	size += fill_ace_for_sid(pace, &sid_everyone, ACCESS_ALLOWED, 0,
				 fattr->cf_mode, 0007);

out:
	pndacl->num_aces = cpu_to_le32(num_aces);
	pndacl->size = cpu_to_le16(le16_to_cpu(pndacl->size) + size);
}

static int parse_sid(struct smb_sid *psid, char *end_of_acl)
{
	/*
	 * validate that we do not go past end of ACL - sid must be at least 8
	 * bytes long (assuming no sub-auths - e.g. the null SID
	 */
	if (end_of_acl < (char *)psid + 8) {
		pr_err("ACL too small to parse SID %p\n", psid);
		return -EINVAL;
	}

	return 0;
}

/* Convert CIFS ACL to POSIX form */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int parse_sec_desc(struct mnt_idmap *idmap, struct smb_ntsd *pntsd,
#else
int parse_sec_desc(struct user_namespace *user_ns, struct smb_ntsd *pntsd,
#endif
		   int acl_len, struct smb_fattr *fattr)
{
	int rc = 0;
	struct smb_sid *owner_sid_ptr, *group_sid_ptr;
	struct smb_acl *dacl_ptr; /* no need for SACL ptr */
	char *end_of_acl = ((char *)pntsd) + acl_len;
	__u32 dacloffset;
	int pntsd_type;

	if (!pntsd)
		return -EIO;

	if (acl_len < sizeof(struct smb_ntsd))
		return -EINVAL;

	owner_sid_ptr = (struct smb_sid *)((char *)pntsd +
			le32_to_cpu(pntsd->osidoffset));
	group_sid_ptr = (struct smb_sid *)((char *)pntsd +
			le32_to_cpu(pntsd->gsidoffset));
	dacloffset = le32_to_cpu(pntsd->dacloffset);
	dacl_ptr = (struct smb_acl *)((char *)pntsd + dacloffset);
	ksmbd_debug(SMB,
		    "revision %d type 0x%x ooffset 0x%x goffset 0x%x sacloffset 0x%x dacloffset 0x%x\n",
		    pntsd->revision, pntsd->type, le32_to_cpu(pntsd->osidoffset),
		    le32_to_cpu(pntsd->gsidoffset),
		    le32_to_cpu(pntsd->sacloffset), dacloffset);

	pntsd_type = le16_to_cpu(pntsd->type);
	if (!(pntsd_type & DACL_PRESENT)) {
		ksmbd_debug(SMB, "DACL_PRESENT in DACL type is not set\n");
		return rc;
	}

	pntsd->type = cpu_to_le16(DACL_PRESENT);

	if (pntsd->osidoffset) {
		rc = parse_sid(owner_sid_ptr, end_of_acl);
		if (rc) {
			pr_err("%s: Error %d parsing Owner SID\n", __func__, rc);
			return rc;
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		rc = sid_to_id(idmap, owner_sid_ptr, SIDOWNER, fattr);
#else
		rc = sid_to_id(user_ns, owner_sid_ptr, SIDOWNER, fattr);
#endif
		if (rc) {
			pr_err("%s: Error %d mapping Owner SID to uid\n",
			       __func__, rc);
			owner_sid_ptr = NULL;
		}
	}

	if (pntsd->gsidoffset) {
		rc = parse_sid(group_sid_ptr, end_of_acl);
		if (rc) {
			pr_err("%s: Error %d mapping Owner SID to gid\n",
			       __func__, rc);
			return rc;
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		rc = sid_to_id(idmap, group_sid_ptr, SIDUNIX_GROUP, fattr);
#else
		rc = sid_to_id(user_ns, group_sid_ptr, SIDUNIX_GROUP, fattr);
#endif
		if (rc) {
			pr_err("%s: Error %d mapping Group SID to gid\n",
			       __func__, rc);
			group_sid_ptr = NULL;
		}
	}

	if ((pntsd_type & (DACL_AUTO_INHERITED | DACL_AUTO_INHERIT_REQ)) ==
	    (DACL_AUTO_INHERITED | DACL_AUTO_INHERIT_REQ))
		pntsd->type |= cpu_to_le16(DACL_AUTO_INHERITED);
	if (pntsd_type & DACL_PROTECTED)
		pntsd->type |= cpu_to_le16(DACL_PROTECTED);

	if (dacloffset) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		parse_dacl(idmap, dacl_ptr, end_of_acl,
#else
		parse_dacl(user_ns, dacl_ptr, end_of_acl,
#endif
			   owner_sid_ptr, group_sid_ptr, fattr);
	}

	return 0;
}

/* Convert permission bits from mode to equivalent CIFS ACL */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int build_sec_desc(struct mnt_idmap *idmap,
#else
int build_sec_desc(struct user_namespace *user_ns,
#endif
		   struct smb_ntsd *pntsd, struct smb_ntsd *ppntsd,
		   int ppntsd_size, int addition_info, __u32 *secdesclen,
		   struct smb_fattr *fattr)
{
	int rc = 0;
	__u32 offset;
	struct smb_sid *owner_sid_ptr, *group_sid_ptr;
	struct smb_sid *nowner_sid_ptr, *ngroup_sid_ptr;
	struct smb_acl *dacl_ptr = NULL; /* no need for SACL ptr */
	uid_t uid;
	gid_t gid;
	unsigned int sid_type = SIDOWNER;

	nowner_sid_ptr = kmalloc(sizeof(struct smb_sid), GFP_KERNEL);
	if (!nowner_sid_ptr)
		return -ENOMEM;

	uid = from_kuid(&init_user_ns, fattr->cf_uid);
	if (!uid)
		sid_type = SIDUNIX_USER;
	id_to_sid(uid, sid_type, nowner_sid_ptr);

	ngroup_sid_ptr = kmalloc(sizeof(struct smb_sid), GFP_KERNEL);
	if (!ngroup_sid_ptr) {
		kfree(nowner_sid_ptr);
		return -ENOMEM;
	}

	gid = from_kgid(&init_user_ns, fattr->cf_gid);
	id_to_sid(gid, SIDUNIX_GROUP, ngroup_sid_ptr);

	offset = sizeof(struct smb_ntsd);
	pntsd->sacloffset = 0;
	pntsd->revision = cpu_to_le16(1);
	pntsd->type = cpu_to_le16(SELF_RELATIVE);
	if (ppntsd)
		pntsd->type |= ppntsd->type;

	if (addition_info & OWNER_SECINFO) {
		pntsd->osidoffset = cpu_to_le32(offset);
		owner_sid_ptr = (struct smb_sid *)((char *)pntsd + offset);
		smb_copy_sid(owner_sid_ptr, nowner_sid_ptr);
		offset += 1 + 1 + 6 + (nowner_sid_ptr->num_subauth * 4);
	}

	if (addition_info & GROUP_SECINFO) {
		pntsd->gsidoffset = cpu_to_le32(offset);
		group_sid_ptr = (struct smb_sid *)((char *)pntsd + offset);
		smb_copy_sid(group_sid_ptr, ngroup_sid_ptr);
		offset += 1 + 1 + 6 + (ngroup_sid_ptr->num_subauth * 4);
	}

	if (addition_info & DACL_SECINFO) {
		pntsd->type |= cpu_to_le16(DACL_PRESENT);
		dacl_ptr = (struct smb_acl *)((char *)pntsd + offset);
		dacl_ptr->revision = cpu_to_le16(2);
		dacl_ptr->size = cpu_to_le16(sizeof(struct smb_acl));
		dacl_ptr->num_aces = 0;

		if (!ppntsd) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			set_mode_dacl(idmap, dacl_ptr, fattr);
#else
			set_mode_dacl(user_ns, dacl_ptr, fattr);
#endif
		} else {
			struct smb_acl *ppdacl_ptr;
			unsigned int dacl_offset = le32_to_cpu(ppntsd->dacloffset);
			int ppdacl_size, ntacl_size = ppntsd_size - dacl_offset;

			if (!dacl_offset ||
			    (dacl_offset + sizeof(struct smb_acl) > ppntsd_size))
				goto out;

			ppdacl_ptr = (struct smb_acl *)((char *)ppntsd + dacl_offset);
			ppdacl_size = le16_to_cpu(ppdacl_ptr->size);
			if (ppdacl_size > ntacl_size ||
			    ppdacl_size < sizeof(struct smb_acl))
				goto out;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			set_ntacl_dacl(idmap, dacl_ptr, ppdacl_ptr,
#else
			set_ntacl_dacl(user_ns, dacl_ptr, ppdacl_ptr,
#endif
				       ntacl_size - sizeof(struct smb_acl),
				       nowner_sid_ptr, ngroup_sid_ptr,
				       fattr);
		}
		pntsd->dacloffset = cpu_to_le32(offset);
		offset += le16_to_cpu(dacl_ptr->size);
	}

out:
	kfree(nowner_sid_ptr);
	kfree(ngroup_sid_ptr);
	*secdesclen = offset;
	return rc;
}

static void smb_set_ace(struct smb_ace *ace, const struct smb_sid *sid, u8 type,
			u8 flags, __le32 access_req)
{
	ace->type = type;
	ace->flags = flags;
	ace->access_req = access_req;
	smb_copy_sid(&ace->sid, sid);
	ace->size = cpu_to_le16(1 + 1 + 2 + 4 + 1 + 1 + 6 + (sid->num_subauth * 4));
}

int smb_inherit_dacl(struct ksmbd_conn *conn,
		     const struct path *path,
		     unsigned int uid, unsigned int gid)
{
	const struct smb_sid *psid, *creator = NULL;
	struct smb_ace *parent_aces, *aces;
	struct smb_acl *parent_pdacl;
	struct smb_ntsd *parent_pntsd = NULL;
	struct smb_sid owner_sid, group_sid;
	struct dentry *parent = path->dentry->d_parent;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	struct mnt_idmap *idmap = mnt_idmap(path->mnt);
#else
	struct user_namespace *user_ns = mnt_user_ns(path->mnt);
#endif
	int inherited_flags = 0, flags = 0, i, ace_cnt = 0, nt_size = 0, pdacl_size;
	int rc = 0, num_aces, dacloffset, pntsd_type, pntsd_size, acl_len, aces_size;
	char *aces_base;
	bool is_dir = S_ISDIR(d_inode(path->dentry)->i_mode);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	pntsd_size = ksmbd_vfs_get_sd_xattr(conn, idmap,
#else
	pntsd_size = ksmbd_vfs_get_sd_xattr(conn, user_ns,
#endif
					    parent, &parent_pntsd);
	if (pntsd_size <= 0)
		return -ENOENT;
	dacloffset = le32_to_cpu(parent_pntsd->dacloffset);
	if (!dacloffset || (dacloffset + sizeof(struct smb_acl) > pntsd_size)) {
		rc = -EINVAL;
		goto free_parent_pntsd;
	}

	parent_pdacl = (struct smb_acl *)((char *)parent_pntsd + dacloffset);
	acl_len = pntsd_size - dacloffset;
	num_aces = le32_to_cpu(parent_pdacl->num_aces);
	pntsd_type = le16_to_cpu(parent_pntsd->type);
	pdacl_size = le16_to_cpu(parent_pdacl->size);

	if (pdacl_size > acl_len || pdacl_size < sizeof(struct smb_acl)) {
		rc = -EINVAL;
		goto free_parent_pntsd;
	}

	aces_base = kmalloc(sizeof(struct smb_ace) * num_aces * 2, GFP_KERNEL);
	if (!aces_base) {
		rc = -ENOMEM;
		goto free_parent_pntsd;
	}

	aces = (struct smb_ace *)aces_base;
	parent_aces = (struct smb_ace *)((char *)parent_pdacl +
			sizeof(struct smb_acl));
	aces_size = acl_len - sizeof(struct smb_acl);

	if (pntsd_type & DACL_AUTO_INHERITED)
		inherited_flags = INHERITED_ACE;

	for (i = 0; i < num_aces; i++) {
		int pace_size;

		if (offsetof(struct smb_ace, access_req) > aces_size)
			break;

		pace_size = le16_to_cpu(parent_aces->size);
		if (pace_size > aces_size)
			break;

		aces_size -= pace_size;

		flags = parent_aces->flags;
		if (!smb_inherit_flags(flags, is_dir))
			goto pass;
		if (is_dir) {
			flags &= ~(INHERIT_ONLY_ACE | INHERITED_ACE);
			if (!(flags & CONTAINER_INHERIT_ACE))
				flags |= INHERIT_ONLY_ACE;
			if (flags & NO_PROPAGATE_INHERIT_ACE)
				flags = 0;
		} else {
			flags = 0;
		}

		if (!compare_sids(&creator_owner, &parent_aces->sid)) {
			creator = &creator_owner;
			id_to_sid(uid, SIDOWNER, &owner_sid);
			psid = &owner_sid;
		} else if (!compare_sids(&creator_group, &parent_aces->sid)) {
			creator = &creator_group;
			id_to_sid(gid, SIDUNIX_GROUP, &group_sid);
			psid = &group_sid;
		} else {
			creator = NULL;
			psid = &parent_aces->sid;
		}

		if (is_dir && creator && flags & CONTAINER_INHERIT_ACE) {
			smb_set_ace(aces, psid, parent_aces->type, inherited_flags,
				    parent_aces->access_req);
			nt_size += le16_to_cpu(aces->size);
			ace_cnt++;
			aces = (struct smb_ace *)((char *)aces + le16_to_cpu(aces->size));
			flags |= INHERIT_ONLY_ACE;
			psid = creator;
		} else if (is_dir && !(parent_aces->flags & NO_PROPAGATE_INHERIT_ACE)) {
			psid = &parent_aces->sid;
		}

		smb_set_ace(aces, psid, parent_aces->type, flags | inherited_flags,
			    parent_aces->access_req);
		nt_size += le16_to_cpu(aces->size);
		aces = (struct smb_ace *)((char *)aces + le16_to_cpu(aces->size));
		ace_cnt++;
pass:
		parent_aces = (struct smb_ace *)((char *)parent_aces + pace_size);
	}

	if (nt_size > 0) {
		struct smb_ntsd *pntsd;
		struct smb_acl *pdacl;
		struct smb_sid *powner_sid = NULL, *pgroup_sid = NULL;
		int powner_sid_size = 0, pgroup_sid_size = 0, pntsd_size;

		if (parent_pntsd->osidoffset) {
			powner_sid = (struct smb_sid *)((char *)parent_pntsd +
					le32_to_cpu(parent_pntsd->osidoffset));
			powner_sid_size = 1 + 1 + 6 + (powner_sid->num_subauth * 4);
		}
		if (parent_pntsd->gsidoffset) {
			pgroup_sid = (struct smb_sid *)((char *)parent_pntsd +
					le32_to_cpu(parent_pntsd->gsidoffset));
			pgroup_sid_size = 1 + 1 + 6 + (pgroup_sid->num_subauth * 4);
		}

		pntsd = kzalloc(sizeof(struct smb_ntsd) + powner_sid_size +
				pgroup_sid_size + sizeof(struct smb_acl) +
				nt_size, GFP_KERNEL);
		if (!pntsd) {
			rc = -ENOMEM;
			goto free_aces_base;
		}

		pntsd->revision = cpu_to_le16(1);
		pntsd->type = cpu_to_le16(SELF_RELATIVE | DACL_PRESENT);
		if (le16_to_cpu(parent_pntsd->type) & DACL_AUTO_INHERITED)
			pntsd->type |= cpu_to_le16(DACL_AUTO_INHERITED);
		pntsd_size = sizeof(struct smb_ntsd);
		pntsd->osidoffset = parent_pntsd->osidoffset;
		pntsd->gsidoffset = parent_pntsd->gsidoffset;
		pntsd->dacloffset = parent_pntsd->dacloffset;

		if (pntsd->osidoffset) {
			struct smb_sid *owner_sid = (struct smb_sid *)((char *)pntsd +
					le32_to_cpu(pntsd->osidoffset));
			memcpy(owner_sid, powner_sid, powner_sid_size);
			pntsd_size += powner_sid_size;
		}

		if (pntsd->gsidoffset) {
			struct smb_sid *group_sid = (struct smb_sid *)((char *)pntsd +
					le32_to_cpu(pntsd->gsidoffset));
			memcpy(group_sid, pgroup_sid, pgroup_sid_size);
			pntsd_size += pgroup_sid_size;
		}

		if (pntsd->dacloffset) {
			struct smb_ace *pace;

			pdacl = (struct smb_acl *)((char *)pntsd + le32_to_cpu(pntsd->dacloffset));
			pdacl->revision = cpu_to_le16(2);
			pdacl->size = cpu_to_le16(sizeof(struct smb_acl) + nt_size);
			pdacl->num_aces = cpu_to_le32(ace_cnt);
			pace = (struct smb_ace *)((char *)pdacl + sizeof(struct smb_acl));
			memcpy(pace, aces_base, nt_size);
			pntsd_size += sizeof(struct smb_acl) + nt_size;
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		ksmbd_vfs_set_sd_xattr(conn, idmap,
#else
		ksmbd_vfs_set_sd_xattr(conn, user_ns,
#endif
				       path->dentry, pntsd, pntsd_size);
		kfree(pntsd);
	}

free_aces_base:
	kfree(aces_base);
free_parent_pntsd:
	kfree(parent_pntsd);
	return rc;
}

bool smb_inherit_flags(int flags, bool is_dir)
{
	if (!is_dir)
		return (flags & OBJECT_INHERIT_ACE) != 0;

	if (flags & OBJECT_INHERIT_ACE && !(flags & NO_PROPAGATE_INHERIT_ACE))
		return true;

	if (flags & CONTAINER_INHERIT_ACE)
		return true;
	return false;
}

int smb_check_perm_dacl(struct ksmbd_conn *conn, const struct path *path,
			__le32 *pdaccess, int uid)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	struct mnt_idmap *idmap = mnt_idmap(path->mnt);
#else
	struct user_namespace *user_ns = mnt_user_ns(path->mnt);
#endif
	struct smb_ntsd *pntsd = NULL;
	struct smb_acl *pdacl;
	struct posix_acl *posix_acls;
	int rc = 0, pntsd_size, acl_size, aces_size, pdacl_size, dacl_offset;
	struct smb_sid sid;
	int granted = le32_to_cpu(*pdaccess & ~FILE_MAXIMAL_ACCESS_LE);
	struct smb_ace *ace;
	int i, found = 0;
	unsigned int access_bits = 0;
	struct smb_ace *others_ace = NULL;
	struct posix_acl_entry *pa_entry;
	unsigned int sid_type = SIDOWNER;
	unsigned short ace_size;

	ksmbd_debug(SMB, "check permission using windows acl\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	pntsd_size = ksmbd_vfs_get_sd_xattr(conn, idmap,
#else
	pntsd_size = ksmbd_vfs_get_sd_xattr(conn, user_ns,
#endif
					    path->dentry, &pntsd);
	if (pntsd_size <= 0 || !pntsd)
		goto err_out;

	dacl_offset = le32_to_cpu(pntsd->dacloffset);
	if (!dacl_offset ||
	    (dacl_offset + sizeof(struct smb_acl) > pntsd_size))
		goto err_out;

	pdacl = (struct smb_acl *)((char *)pntsd + le32_to_cpu(pntsd->dacloffset));
	acl_size = pntsd_size - dacl_offset;
	pdacl_size = le16_to_cpu(pdacl->size);

	if (pdacl_size > acl_size || pdacl_size < sizeof(struct smb_acl))
		goto err_out;

	if (!pdacl->num_aces) {
		if (!(pdacl_size - sizeof(struct smb_acl)) &&
		    *pdaccess & ~(FILE_READ_CONTROL_LE | FILE_WRITE_DAC_LE)) {
			rc = -EACCES;
			goto err_out;
		}
		goto err_out;
	}

	if (*pdaccess & FILE_MAXIMAL_ACCESS_LE) {
		granted = READ_CONTROL | WRITE_DAC | FILE_READ_ATTRIBUTES |
			DELETE;

		ace = (struct smb_ace *)((char *)pdacl + sizeof(struct smb_acl));
		aces_size = acl_size - sizeof(struct smb_acl);
		for (i = 0; i < le32_to_cpu(pdacl->num_aces); i++) {
			if (offsetof(struct smb_ace, access_req) > aces_size)
				break;
			ace_size = le16_to_cpu(ace->size);
			if (ace_size > aces_size)
				break;
			aces_size -= ace_size;
			granted |= le32_to_cpu(ace->access_req);
			ace = (struct smb_ace *)((char *)ace + le16_to_cpu(ace->size));
		}

		if (!pdacl->num_aces)
			granted = GENERIC_ALL_FLAGS;
	}

	if (!uid)
		sid_type = SIDUNIX_USER;
	id_to_sid(uid, sid_type, &sid);

	ace = (struct smb_ace *)((char *)pdacl + sizeof(struct smb_acl));
	aces_size = acl_size - sizeof(struct smb_acl);
	for (i = 0; i < le32_to_cpu(pdacl->num_aces); i++) {
		if (offsetof(struct smb_ace, access_req) > aces_size)
			break;
		ace_size = le16_to_cpu(ace->size);
		if (ace_size > aces_size)
			break;
		aces_size -= ace_size;

		if (!compare_sids(&sid, &ace->sid) ||
		    !compare_sids(&sid_unix_NFS_mode, &ace->sid)) {
			found = 1;
			break;
		}
		if (!compare_sids(&sid_everyone, &ace->sid))
			others_ace = ace;

		ace = (struct smb_ace *)((char *)ace + le16_to_cpu(ace->size));
	}

	if (*pdaccess & FILE_MAXIMAL_ACCESS_LE && found) {
		granted = READ_CONTROL | WRITE_DAC | FILE_READ_ATTRIBUTES |
			DELETE;

		granted |= le32_to_cpu(ace->access_req);

		if (!pdacl->num_aces)
			granted = GENERIC_ALL_FLAGS;
	}

	if (IS_ENABLED(CONFIG_FS_POSIX_ACL)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
		posix_acls = get_inode_acl(d_inode(path->dentry), ACL_TYPE_ACCESS);
#else
		posix_acls = get_acl(d_inode(path->dentry), ACL_TYPE_ACCESS);
#endif
		if (posix_acls && !found) {
			unsigned int id = -1;

			pa_entry = posix_acls->a_entries;
			for (i = 0; i < posix_acls->a_count; i++, pa_entry++) {
				if (pa_entry->e_tag == ACL_USER)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
					id = posix_acl_uid_translate(idmap, pa_entry);
#else
					id = posix_acl_uid_translate(user_ns, pa_entry);
#endif
				else if (pa_entry->e_tag == ACL_GROUP)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
					id = posix_acl_gid_translate(idmap, pa_entry);
#else
					id = posix_acl_gid_translate(user_ns, pa_entry);
#endif
				else
					continue;

				if (id == uid) {
					mode_to_access_flags(pa_entry->e_perm,
							     0777,
							     &access_bits);
					if (!access_bits)
						access_bits =
							SET_MINIMUM_RIGHTS;
					posix_acl_release(posix_acls);
					goto check_access_bits;
				}
			}
		}
		if (posix_acls)
			posix_acl_release(posix_acls);
	}

	if (!found) {
		if (others_ace) {
			ace = others_ace;
		} else {
			ksmbd_debug(SMB, "Can't find corresponding sid\n");
			rc = -EACCES;
			goto err_out;
		}
	}

	switch (ace->type) {
	case ACCESS_ALLOWED_ACE_TYPE:
		access_bits = le32_to_cpu(ace->access_req);
		break;
	case ACCESS_DENIED_ACE_TYPE:
	case ACCESS_DENIED_CALLBACK_ACE_TYPE:
		access_bits = le32_to_cpu(~ace->access_req);
		break;
	}

check_access_bits:
	if (granted &
	    ~(access_bits | FILE_READ_ATTRIBUTES | READ_CONTROL | WRITE_DAC | DELETE)) {
		ksmbd_debug(SMB, "Access denied with winACL, granted : %x, access_req : %x\n",
			    granted, le32_to_cpu(ace->access_req));
		rc = -EACCES;
		goto err_out;
	}

	*pdaccess = cpu_to_le32(granted);
err_out:
	kfree(pntsd);
	return rc;
}

int set_info_sec(struct ksmbd_conn *conn, struct ksmbd_tree_connect *tcon,
		 const struct path *path, struct smb_ntsd *pntsd, int ntsd_len,
		 bool type_check)
{
	int rc;
	struct smb_fattr fattr = {{0}};
	struct inode *inode = d_inode(path->dentry);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	struct mnt_idmap *idmap = mnt_idmap(path->mnt);
#else
	struct user_namespace *user_ns = mnt_user_ns(path->mnt);
#endif
	struct iattr newattrs;

	fattr.cf_uid = INVALID_UID;
	fattr.cf_gid = INVALID_GID;
	fattr.cf_mode = inode->i_mode;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	rc = parse_sec_desc(idmap, pntsd, ntsd_len, &fattr);
#else
	rc = parse_sec_desc(user_ns, pntsd, ntsd_len, &fattr);
#endif
	if (rc)
		goto out;

	newattrs.ia_valid = ATTR_CTIME;
	if (!uid_eq(fattr.cf_uid, INVALID_UID)) {
		newattrs.ia_valid |= ATTR_UID;
		newattrs.ia_uid = fattr.cf_uid;
	}
	if (!gid_eq(fattr.cf_gid, INVALID_GID)) {
		inode->i_gid = fattr.cf_gid;
		newattrs.ia_valid |= ATTR_GID;
		newattrs.ia_gid = fattr.cf_gid;
	}
	newattrs.ia_valid |= ATTR_MODE;
	newattrs.ia_mode = (inode->i_mode & ~0777) | (fattr.cf_mode & 0777);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	ksmbd_vfs_remove_acl_xattrs(idmap, path->dentry);
#else
	ksmbd_vfs_remove_acl_xattrs(user_ns, path->dentry);
#endif
	/* Update posix acls */
	if (IS_ENABLED(CONFIG_FS_POSIX_ACL) && fattr.cf_dacls) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		rc = set_posix_acl(idmap, path->dentry,
#else
		rc = set_posix_acl(user_ns, path->dentry,
#endif
#else
		rc = set_posix_acl(user_ns, inode,
#endif
				   ACL_TYPE_ACCESS,
				   fattr.cf_acls);
#else
		rc = set_posix_acl(inode, ACL_TYPE_ACCESS, fattr.cf_acls);
#endif
		if (rc < 0)
			ksmbd_debug(SMB,
				    "Set posix acl(ACL_TYPE_ACCESS) failed, rc : %d\n",
				    rc);
		if (S_ISDIR(inode->i_mode) && fattr.cf_dacls) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			rc = set_posix_acl(idmap, path->dentry,
#else
			rc = set_posix_acl(user_ns, path->dentry,
#endif
#else
			rc = set_posix_acl(user_ns, inode,
#endif
					   ACL_TYPE_DEFAULT, fattr.cf_dacls);
#else
			rc = set_posix_acl(inode, ACL_TYPE_DEFAULT,
					   fattr.cf_dacls);
#endif
			if (rc)
				ksmbd_debug(SMB,
					    "Set posix acl(ACL_TYPE_DEFAULT) failed, rc : %d\n",
					    rc);
		}
	}

	inode_lock(inode);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	rc = notify_change(idmap, path->dentry, &newattrs, NULL);
#else
	rc = notify_change(user_ns, path->dentry, &newattrs, NULL);
#endif
#else
	rc = notify_change(path->dentry, &newattrs, NULL);
#endif
	inode_unlock(inode);
	if (rc)
		goto out;

	/* Check it only calling from SD BUFFER context */
	if (type_check && !(le16_to_cpu(pntsd->type) & DACL_PRESENT))
		goto out;

	if (test_share_config_flag(tcon->share_conf, KSMBD_SHARE_FLAG_ACL_XATTR)) {
		/* Update WinACL in xattr */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		ksmbd_vfs_remove_sd_xattrs(idmap, path->dentry);
		ksmbd_vfs_set_sd_xattr(conn, idmap,
#else
		ksmbd_vfs_remove_sd_xattrs(user_ns, path->dentry);
		ksmbd_vfs_set_sd_xattr(conn, user_ns,
#endif
				       path->dentry, pntsd, ntsd_len);
	}

out:
	posix_acl_release(fattr.cf_acls);
	posix_acl_release(fattr.cf_dacls);
	mark_inode_dirty(inode);
	return rc;
}

void ksmbd_init_domain(u32 *sub_auth)
{
	int i;

	memcpy(&server_conf.domain_sid, &domain, sizeof(struct smb_sid));
	for (i = 0; i < 3; ++i)
		server_conf.domain_sid.sub_auth[i + 1] = cpu_to_le32(sub_auth[i]);
}
