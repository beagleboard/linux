// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2012 Broadcom Corporation
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
		brcmf_dbg(FWCON, "CONSOLE: %s\n", data);
		data = s + 1;
	}
	if (*data)
		brcmf_dbg(FWCON, "CONSOLE: %s", data);

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


int brcmf_debug_fwlog_init(struct brcmf_pub *drvr)
{
	return brcmf_fweh_register(drvr, BRCMF_E_TRACE,
				brcmf_debug_trace_parser);
}

struct dentry *brcmf_debugfs_get_devdir(struct brcmf_pub *drvr)
{
	return drvr->wiphy->debugfsdir;
}

void brcmf_debugfs_add_entry(struct brcmf_pub *drvr, const char *fn,
			    int (*read_fn)(struct seq_file *seq, void *data))
{
	WARN(!drvr->wiphy->debugfsdir, "wiphy not (yet) registered\n");
	debugfs_create_devm_seqfile(drvr->bus_if->dev, fn,
				    drvr->wiphy->debugfsdir, read_fn);
}
