/*
 * Copyright (c) 2012 Broadcom Corporation
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#include <linux/debugfs.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/devcoredump.h>

#include <brcmu_wifi.h>
#include <brcmu_utils.h>
#include "core.h"
#include "bus.h"
#include "fweh.h"
#include "debug.h"

static struct dentry *root_folder;

static int
brcmf_debug_msgtrace_seqchk(u32 *prev, u32 cur)
{
	if ((cur == 0 && *prev == 0xFFFFFFFF) || ((cur - *prev) == 1)) {
		goto done;
	} else if (cur == *prev) {
		brcmf_dbg(FWCON, "duplicate trace\n");
		return -1;
	} else if (cur > *prev) {
		brcmf_dbg(FWCON, "lost %d packets\n", cur - *prev);
	} else {
		brcmf_dbg(FWCON, "seq out of order, host %d, dongle %d\n",
			  *prev, cur);
	}
done:
	*prev = cur;
	return 0;
}

static int
brcmf_debug_msg_parser(void *event_data)
{
	int err = 0;
	struct msgtrace_hdr *hdr;
	char *data, *s;
	static u32 seqnum_prev;

	hdr = (struct msgtrace_hdr *)event_data;
	data = (char *)event_data + MSGTRACE_HDRLEN;

	/* There are 2 bytes available at the end of data */
	data[ntohs(hdr->len)] = '\0';

	if (ntohl(hdr->discarded_bytes) || ntohl(hdr->discarded_printf)) {
		brcmf_dbg(FWCON, "Discarded_bytes %d discarded_printf %d\n",
			  ntohl(hdr->discarded_bytes),
				ntohl(hdr->discarded_printf));
	}

	err = brcmf_debug_msgtrace_seqchk(&seqnum_prev, ntohl(hdr->seqnum));
	if (err)
		return err;

	while (*data != '\0' && (s = strstr(data, "\n")) != NULL) {
		*s = '\0';
		brcmf_dbg(FWCON, "[FWLOG] %s\n", data);
		data = s + 1;
	}
	if (*data)
		brcmf_dbg(FWCON, "[FWLOG] %s", data);

	return err;
}

static int
brcmf_debug_trace_parser(struct brcmf_if *ifp,
			 const struct brcmf_event_msg *evtmsg,
			 void *event_data)
{
	int err = 0;
	struct msgtrace_hdr *hdr;

	hdr = (struct msgtrace_hdr *)event_data;
	if (hdr->version != MSGTRACE_VERSION) {
		brcmf_dbg(FWCON, "trace version mismatch host %d dngl %d\n",
			  MSGTRACE_VERSION, hdr->version);
		err = -EPROTO;
		return err;
	}

	if (hdr->trace_type == MSGTRACE_HDR_TYPE_MSG)
		err = brcmf_debug_msg_parser(event_data);

	return err;
}

int brcmf_debug_create_memdump(struct brcmf_bus *bus, const void *data,
			       size_t len)
{
	void *dump;
	size_t ramsize;
	int err;

	ramsize = brcmf_bus_get_ramsize(bus);
	if (!ramsize)
		return -ENOTSUPP;

	dump = vzalloc(len + ramsize);
	if (!dump)
		return -ENOMEM;

	if (data && len > 0)
		memcpy(dump, data, len);
	err = brcmf_bus_get_memdump(bus, dump + len, ramsize);
	if (err) {
		vfree(dump);
		return err;
	}

	dev_coredumpv(bus->dev, dump, len + ramsize, GFP_KERNEL);

	return 0;
}

void brcmf_debugfs_init(void)
{
	root_folder = debugfs_create_dir(KBUILD_MODNAME, NULL);
	if (IS_ERR(root_folder))
		root_folder = NULL;
}

void brcmf_debugfs_exit(void)
{
	if (!root_folder)
		return;

	debugfs_remove_recursive(root_folder);
	root_folder = NULL;
}

int brcmf_debug_attach(struct brcmf_pub *drvr)
{
	struct device *dev = drvr->bus_if->dev;

	if (!root_folder)
		return -ENODEV;

	drvr->dbgfs_dir = debugfs_create_dir(dev_name(dev), root_folder);
	return PTR_ERR_OR_ZERO(drvr->dbgfs_dir);
}

int brcmf_debug_fwlog_init(struct brcmf_pub *drvr)
{
	return brcmf_fweh_register(drvr, BRCMF_E_TRACE,
				brcmf_debug_trace_parser);
}

void brcmf_debug_detach(struct brcmf_pub *drvr)
{
	brcmf_fweh_unregister(drvr, BRCMF_E_PSM_WATCHDOG);

	if (!IS_ERR_OR_NULL(drvr->dbgfs_dir))
		debugfs_remove_recursive(drvr->dbgfs_dir);
}

struct dentry *brcmf_debugfs_get_devdir(struct brcmf_pub *drvr)
{
	return drvr->dbgfs_dir;
}

int brcmf_debugfs_add_entry(struct brcmf_pub *drvr, const char *fn,
			    int (*read_fn)(struct seq_file *seq, void *data))
{
	struct dentry *e;

	e = debugfs_create_devm_seqfile(drvr->bus_if->dev, fn,
					drvr->dbgfs_dir, read_fn);
	return PTR_ERR_OR_ZERO(e);
}
