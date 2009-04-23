/*
 * OMAP STI/XTI communications interface via netlink socket.
 *
 * Copyright (C) 2004, 2005, 2006 Nokia Corporation
 * Written by: Paul Mundt <paul.mundt@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/netlink.h>
#include <linux/socket.h>
#include <linux/skbuff.h>
#include <linux/mutex.h>
#include <net/sock.h>
#include <mach/sti.h>

static struct sock *sti_sock;
static DEFINE_MUTEX(sti_netlink_mutex);

enum {
	STI_READ,
	STI_WRITE,
};

#if defined(CONFIG_ARCH_OMAP1) || defined(CONFIG_ARCH_OMAP2)
static int sti_netlink_read(int pid, int seq, void *payload, int size)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	int ret, len = NLMSG_SPACE(size);
	unsigned char *tail;

	skb = alloc_skb(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	tail = skb->tail;
	nlh = NLMSG_PUT(skb, pid, seq, STI_READ,
			len - (sizeof(struct nlmsghdr)));
	nlh->nlmsg_flags = 0;
	memcpy(NLMSG_DATA(nlh), payload, size);
	nlh->nlmsg_len = skb->tail - tail;

	ret = netlink_unicast(sti_sock, skb, pid, MSG_DONTWAIT);
	if (ret > 0)
		ret = 0;

	return ret;

nlmsg_failure:
	if (skb)
		kfree_skb(skb);

	return -EINVAL;
}
#endif

/*
 * We abuse nlmsg_type and nlmsg_flags for our purposes.
 *
 * The ID is encoded into the upper 8 bits of the nlmsg_type, while the
 * channel number is encoded into the upper 8 bits of the nlmsg_flags.
 */
static int sti_netlink_receive_msg(struct sk_buff *skb, struct nlmsghdr *nlh)
{
	void *data;
	u8 chan, id;
	int size;
	int ret = 0, len = 0;

	data	= NLMSG_DATA(nlh);
	chan	= (nlh->nlmsg_flags >> 8) & 0xff;
	id	= (nlh->nlmsg_type  >> 8) & 0xff;
	size	= (int)(nlh->nlmsg_len - ((char *)data - (char *)nlh));

	switch (nlh->nlmsg_type & 0xff) {
	case STI_WRITE:
		sti_channel_write_trace(size, id, data, chan);
		break;
#if defined(CONFIG_ARCH_OMAP1) || defined(CONFIG_ARCH_OMAP2)
	case STI_READ:
		data = kmalloc(size, GFP_KERNEL);
		if (!data)
			return -ENOMEM;
		memset(data, 0, size);

		len = sti_read_packet(data, size);
		ret = sti_netlink_read(NETLINK_CB(skb).pid, nlh->nlmsg_seq,
				       data, len);
		kfree(data);
		break;
#endif
	default:
		return -ENOTTY;
	}

	return ret;
}

static int sti_netlink_receive_skb(struct sk_buff *skb)
{
	while (skb->len >= NLMSG_SPACE(0)) {
		struct nlmsghdr *nlh;
		u32 rlen;
		int ret;

		nlh = (struct nlmsghdr *)skb->data;
		if (nlh->nlmsg_len < sizeof(struct nlmsghdr) ||
		    skb->len < nlh->nlmsg_len)
			break;

		rlen = NLMSG_ALIGN(nlh->nlmsg_len);
		if (rlen > skb->len)
			rlen = skb->len;

		ret = sti_netlink_receive_msg(skb, nlh);
		if (ret)
			netlink_ack(skb, nlh, -ret);
		else if (nlh->nlmsg_flags & NLM_F_ACK)
			netlink_ack(skb, nlh, 0);

		skb_pull(skb, rlen);
	}

	return 0;
}

static void sti_netlink_receive(struct sk_buff *skb)
{
	if (!mutex_trylock(&sti_netlink_mutex))
		return;

	sti_netlink_receive_skb(skb);
	mutex_unlock(&sti_netlink_mutex);
}

static int __init sti_netlink_init(void)
{
	sti_sock = netlink_kernel_create(&init_net, NETLINK_USERSOCK, 0,
					 sti_netlink_receive, NULL,
					 THIS_MODULE);
	if (!sti_sock) {
		printk(KERN_ERR "STI: Failed to create netlink socket\n");
		return -ENODEV;
	}

	return 0;
}

module_init(sti_netlink_init);

MODULE_AUTHOR("Paul Mundt");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("STI netlink-driven communications interface");
