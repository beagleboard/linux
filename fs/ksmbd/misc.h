/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 *   Copyright (C) 2018 Samsung Electronics Co., Ltd.
 */

#ifndef __KSMBD_MISC_H__
#define __KSMBD_MISC_H__

struct ksmbd_share_config;
struct nls_table;
struct kstat;
struct ksmbd_file;

int match_pattern(const char *str, size_t len, const char *pattern);
int ksmbd_validate_filename(char *filename);
int parse_stream_name(char *filename, char **stream_name, int *s_type);
char *convert_to_nt_pathname(struct ksmbd_share_config *share,
			     const struct path *path);
int get_nlink(struct kstat *st);
void ksmbd_conv_path_to_unix(char *path);
void ksmbd_strip_last_slash(char *path);
void ksmbd_conv_path_to_windows(char *path);
char *ksmbd_casefold_sharename(struct unicode_map *um, const char *name);
char *ksmbd_extract_sharename(struct unicode_map *um, const char *treename);
char *convert_to_unix_name(struct ksmbd_share_config *share, const char *name);

#define KSMBD_DIR_INFO_ALIGNMENT	8
struct ksmbd_dir_info;
char *ksmbd_convert_dir_info_name(struct ksmbd_dir_info *d_info,
				  const struct nls_table *local_nls,
				  int *conv_len);

#define NTFS_TIME_OFFSET	((u64)(369 * 365 + 89) * 24 * 3600 * 10000000)
struct timespec64 ksmbd_NTtimeToUnix(__le64 ntutc);
u64 ksmbd_UnixTimeToNT(struct timespec64 t);
long long ksmbd_systime(void);
#endif /* __KSMBD_MISC_H__ */
