// SPDX-License-Identifier: GPL-2.0-only

#include <linux/sched.h>
#include <linux/virtio_light.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_net.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/sched/signal.h>
#include <linux/signal.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/scatterlist.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include "../light_vringh.h"

static struct light_vdev *vdev;
static volatile unsigned char __iomem *virtio_config;
static struct net_device *net_dev;

struct light_copy_desc {
	struct iovec *iov;
	u32 iovcnt;
	u8 vr_idx;
	u8 update_used;
	u32 out_len;
};

#define XMIT_Q			0
#define RCV_Q			1
#define MIN_MTU			ETH_MIN_MTU
#define DEFAULT_MTU		1500
#define MAX_MTU			ETH_MAX_MTU
#define MAX_NET_PKT_SIZE	(_ALIGN_UP(MAX_MTU + VLAN_ETH_HLEN, 64))
#define VNET_TIMEOUT		(120 * HZ)
#define DRV_NAME		"light-virtnet"

static bool light_use_dma;
static int napi_weight = 96;

static struct virtio_net_hdr net_hdr;

static struct iovec rcv_iov =  {
	.iov_base = NULL, .iov_len = 0 /* we will fill it in vnet_package_recv()*/
};

static u8 xmit_hdr[sizeof(struct virtio_net_hdr)];
static struct iovec xmit_iov[2] = {
	{ .iov_base = (void *)xmit_hdr, .iov_len = sizeof(xmit_hdr) },
	{ .iov_base = NULL, .iov_len = 0 }, /* we will fill it in vnet_xmit_skb() */
};

struct virtnet_priv {
	struct sk_buff *rx_skb;
};
static struct virtnet_priv *virt_net;

static int vnet_request_dma_chans(struct light_vdev *ldev)
{
	dma_cap_mask_t mask;
	struct dma_chan *chan;
	u16 i, chan_num;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	chan_num = ARRAY_SIZE(ldev->dma_ch);
	for (i = 0; i < chan_num; i++) {
		chan = dma_request_channel(mask, NULL, NULL);
		if (chan)
			ldev->dma_ch[i] = chan;
		else {
			pr_warn("Net backend: dma chan %d request failed\n", i);
			return -EINVAL;
		}
	}

	pr_info("Net backend: requested %d dma chans\n", chan_num);

	return 0;
}

static void vnet_free_dma_chans(struct light_vdev *ldev)
{
	u16 i, chan_num;

	chan_num = ARRAY_SIZE(ldev->dma_ch);
	for (i = 0; i < chan_num; i++) {
		if (ldev->dma_ch[i]) {
			dma_release_channel(ldev->dma_ch[i]);
			ldev->dma_ch[i] = NULL;
		}
	}
}

static u16 read_avail_idx(void)
{
	struct vringh *vrh;

	vrh = &vdev->vvr[XMIT_Q].vrh;
	return READ_ONCE(vrh->last_avail_idx);
}

static void vnet_disable_frontend_notify(int vq)
{
	struct light_vring *vring;

	vring = &vdev->vvr[vq].vring;
	if (!(vring->vr.used->flags & VRING_USED_F_NO_NOTIFY))
		vring->vr.used->flags |= VRING_USED_F_NO_NOTIFY;
}

static void vnet_enable_frontend_notify(int vq)
{
	struct light_vring *vring;

	vring = &vdev->vvr[vq].vring;
	if (vring->vr.used->flags & VRING_AVAIL_F_NO_INTERRUPT)
		vring->vr.used->flags &= ~VRING_AVAIL_F_NO_INTERRUPT;
}

static void vnet_napi_schedule(struct napi_struct *napi)
{
	if (napi_schedule_prep(napi)) {
		vnet_disable_frontend_notify(RCV_Q);
		__napi_schedule(napi);
	}
}

static irqreturn_t light_vnet_interrupt(int irq, void *opaque)
{
	struct light_vdev *virt_dev = (struct light_vdev *)opaque;
	struct light_vring *vring;
	struct vringh *vrh;

#ifdef CONFIG_SOC_INT_SRC7
	writel(0, vdev->backend_intr_reg);
#else
	/* clear interrupt*/
	writel(vdev->clear_intr, vdev->backend_intr_reg + 0x4);
#endif

	vring = &virt_dev->vvr[RCV_Q].vring;
	vrh = &virt_dev->vvr[RCV_Q].vrh;
	if (vrh->last_avail_idx == vring->vr.avail->idx)
		goto exit;

	vnet_napi_schedule(&vdev->napi);

exit:
	return IRQ_HANDLED;
}

static void light_vnet_notify(struct vringh *vrh)
{
#ifdef CONFIG_SOC_INT_SRC7
	writel(1, vdev->frontend_intr_reg);
#else
	/* Write the interrupt register to signal the other end */
	writel(1, vdev->frontend_intr_reg + 0x10);
#endif
}

static int light_vnet_vring_map(void)
{
	int i, ret, vr_size;
	u8 vq_num;
	u32 count = 0;
	u32 entry_num;
	u32 align;
	phys_addr_t phys_addr;

	init_waitqueue_head(&vdev->waitq);

	/* Wait front-end driver initialization ready, then we can go */
	while (!(readl(virtio_config + VIRTIO_LIGHT_STATUS) &
			VIRTIO_CONFIG_S_DRIVER_OK)) {
		msleep(200);
		if (count++ == 50) {
			count = 0;
			pr_debug("Net backend: waiting frontend driver ready!!!\n");
		}
	}

	/* We already know frontend is ready, now reset status */
	writel(0, virtio_config + VIRTIO_LIGHT_STATUS);

	entry_num = readl(virtio_config + VIRTIO_LIGHT_QUEUE_SIZE);
	if (entry_num == 0)
		return -EINVAL;

	align = readl(virtio_config + VIRTIO_LIGHT_QUEUE_ALIGN);
	if (align < 0)
		return -EINVAL;

	vq_num = readl(virtio_config + VIRTIO_LIGHT_QUEUE_NUM);
	if (!vq_num)
		pr_err("vq_num is invalid, forntend is wrong?\n");
	else
		/* vnet queue pair, at least 2 queues */
		vq_num = 2;

	for (i = 0; i < vq_num; i++) {
		struct light_vringh *vvr = &vdev->vvr[i];
		struct light_vring *vr = &vdev->vvr[i].vring;

		vr_size = PAGE_ALIGN(round_up(vring_size(entry_num, align), 4));

		/* init vring */
		phys_addr = (phys_addr_t)readl(virtio_config +
						VIRTIO_LIGHT_QUEUE_PFN + i * 4) << PAGE_SHIFT;
		vr->va = (void *) phys_to_virt(phys_addr);
		pr_debug("Net backend: vring(%d) virtual addr %lx, phys addr %llx\n",
				i, (unsigned long)vr->va, phys_addr);

		vr->len = vr_size;

		vring_init(&vr->vr, entry_num, vr->va, align);
		ret = vringh_init_kern(&vvr->vrh,
				       0, /* don't need features now,
					   * such as VIRTIO_RING_F_EVENT_IDX
					   */
				       entry_num, false,
				       vr->vr.desc, vr->vr.avail,
				       vr->vr.used);
		if (ret) {
			pr_err("%s %d err %d\n", __func__, __LINE__, ret);
			goto err;
		}

		vringh_kiov_init(&vvr->riov, NULL, 0);
		vringh_kiov_init(&vvr->wiov, NULL, 0);
		vvr->head = USHRT_MAX;
		vvr->vdev = vdev;
		vvr->vrh.notify = light_vnet_notify;
	}

	if (light_use_dma && vnet_request_dma_chans(vdev)) {
		light_use_dma = false;
		ret = -EINVAL;
		goto dma_err;
	}

	pr_info("Net backend: added device(%d) with %d vqs %d entries each, align %d.\n",
			vdev->virtio_id, vq_num, entry_num, align);

	return 0;

dma_err:
	vnet_free_dma_chans(vdev);
err:
	return ret;
}

static int vnet_desc_iov_process(struct light_vdev *vdev, struct light_copy_desc *copy, bool recv);

static u8 vnet_header_check(struct vring_desc *desc)
{
	if (desc->len != sizeof(struct virtio_net_hdr)) {
		pr_err("net header error: size\n");
		return -EIO;
	}

	if (!(desc->flags & VRING_DESC_F_NEXT)) {
		pr_err("net header error: next\n");
		return -EIO;
	}

	if (desc->flags & VRING_DESC_F_WRITE) {
		pr_err("net header error: write\n");
		return -EIO;
	}

	return 0;
}

static int vnet_read_header(struct virtio_net_hdr *hdr)
{
	struct iovec iovec;
	struct light_copy_desc copy;

	iovec.iov_len = sizeof(*hdr);
	iovec.iov_base = hdr;
	copy.iov = &iovec;
	copy.iovcnt = 1;
	copy.vr_idx = RCV_Q;
	copy.update_used = false;  /* do not update used index */

	return vnet_desc_iov_process(vdev, &copy, 0);
}

static int vnet_package_recv(struct light_copy_desc *copy, struct sk_buff *skb)
{
	copy->iov = &rcv_iov;
	copy->iovcnt = 1;
	copy->vr_idx = RCV_Q;
	copy->update_used = true;  /* update used index */
	copy->iov->iov_len = MAX_NET_PKT_SIZE;
	copy->iov->iov_base = skb;

	return vnet_desc_iov_process(vdev, copy, 1);
}

static int vnet_xmit_skb(struct light_copy_desc *copy, struct sk_buff *skb)
{
	copy->iov = xmit_iov;
	copy->iovcnt = 2;
	copy->vr_idx = XMIT_Q;
	copy->update_used = true;
	copy->iov[1].iov_len = skb->len;
	copy->iov[1].iov_base = skb->data;

	return vnet_desc_iov_process(vdev, copy, 0);
}

static void vnet_tx_timeout(struct net_device *netdev, unsigned int __always_unused txqueue)
{
	pr_info("Net backend: Transmit timeout at %ld, latency %ld\n",
			jiffies, dev_trans_start(netdev));

	netdev->stats.tx_errors++;
	netdev->stats.tx_dropped++;
	netif_wake_queue(netdev);
}

static int vnet_change_mtu(struct net_device *netdev, int new_mtu)
{
	pr_info("new mtu is %d\n", new_mtu);
	netdev->mtu = new_mtu;
	netdev_update_features(netdev);

	return 0;
}

static int vnet_close(struct net_device *netdev)
{
	netif_stop_queue(netdev);
	napi_disable(&vdev->napi);
	free_irq(vdev->irq, vdev);

	return 0;
}

static int vnet_open(struct net_device *netdev)
{
	struct virtnet_priv *virt_net;
	struct virtio_net_hdr *hdr;

	if (request_irq(vdev->irq, light_vnet_interrupt, IRQF_SHARED,
			"virtio_net", vdev) < 0) {
		pr_err("Net backend: register irq %d failed\n", vdev->irq);
		return -ENODEV;
	}

	napi_enable(&vdev->napi);
	netif_start_queue(netdev);
	virt_net = netdev_priv(netdev);
	if (!virt_net)
		return -EINVAL;

	hdr = (struct virtio_net_hdr *)xmit_hdr;
	hdr->flags |= VIRTIO_NET_HDR_F_DATA_VALID;

	/* We don't need guest notify us in xmit path */
	vnet_disable_frontend_notify(XMIT_Q);

	return 0;
}

/* xmit a package to frontend */
static netdev_tx_t vnet_tx_packet(struct sk_buff *skb, struct net_device *netdev)
{
	struct light_copy_desc copy;
	u16 avail_idx;
	int ret;
	static u16 requeue = 0;

	/* timestamp packet in software */
	skb_tx_timestamp(skb);

	avail_idx = read_avail_idx();
	if (avail_idx == READ_ONCE(vdev->vvr[XMIT_Q].vring.vr.avail->idx)) {
		/*
		 * Retry sending skb with max 300 times before drop
		 * if we have no resources to xmit
		 */
		if (requeue++ >= 300) {
			requeue = 0;
			goto drop;
		} else
			/* tell net core requeue the skb and xmit it again */
			return NETDEV_TX_BUSY;
	} else
		requeue = 0;

	ret = vnet_xmit_skb(&copy, skb);
	if (unlikely(ret)) {
		if (net_ratelimit())
			pr_warn("Unexpected vnet TX queue failure: %d\n", ret);
		goto drop;
	}

	net_dev->stats.tx_packets++;
	net_dev->stats.tx_bytes += skb->len;

	dev_consume_skb_any(skb);
	skb = NULL;
	netif_trans_update(net_dev);

	return NETDEV_TX_OK;

drop:
	netdev->stats.tx_errors++;
	netdev->stats.tx_dropped++;
	dev_kfree_skb_any(skb);
	skb = NULL;

	return NETDEV_TX_OK;
}

static const struct net_device_ops vnet_ops = {
	.ndo_open = vnet_open,
	.ndo_stop = vnet_close,
	.ndo_start_xmit = vnet_tx_packet,
	.ndo_change_mtu = vnet_change_mtu,
	.ndo_tx_timeout = vnet_tx_timeout,
};

static int vnet_receive(struct napi_struct *napi, int budget)
{
	int ret;
	int status;
	u16 avail_idx;
	u32 desc_idx;
	struct vring_desc *desc;
	struct light_vring *vring;
	struct vringh *vrh;
	struct light_copy_desc copy;
	u32 packages = 0;

	vring = &vdev->vvr[RCV_Q].vring;
	vrh = &vdev->vvr[RCV_Q].vrh;

loop:
	while (packages < budget && vrh->last_avail_idx != vring->vr.avail->idx) {
		packages++;

		avail_idx = vrh->last_avail_idx & (vring->vr.num - 1);
		desc_idx = vring->vr.avail->ring[avail_idx];
		desc = &vring->vr.desc[desc_idx];

		/*
		 * If don't specify GSO or CSUM features, we can ignore
		 * the header contents, just check header format.
		 */
		status = vnet_header_check(desc);
		ret = vnet_read_header(&net_hdr);
		if (ret < 0)
			pr_err("virtio_net hdr read err %d\n", ret);

		/* prepare skb */
		if (!virt_net->rx_skb) {
			virt_net->rx_skb = netdev_alloc_skb_ip_align(net_dev,
							net_dev->mtu + VLAN_ETH_HLEN);
			if (!virt_net->rx_skb) {
				net_dev->stats.rx_errors++;
				/* no memory, we have no choice but drop the package */
				net_dev->stats.rx_dropped++;
				pr_err("Net backend: package dropped as no memory\n");
				continue;
			}
		}
		skb_reserve(virt_net->rx_skb, NET_IP_ALIGN);

		ret = vnet_package_recv(&copy, virt_net->rx_skb);
		if (ret < 0 && status != 0) {
			if (net_ratelimit())
				pr_warn("Unexpected recv error %d %d\n", ret, status);
			kfree_skb(virt_net->rx_skb);
			virt_net->rx_skb = NULL;
			goto loop;
		}

		net_dev->stats.rx_packets++;
		net_dev->stats.rx_bytes += virt_net->rx_skb->len;
		virt_net->rx_skb->ip_summed = CHECKSUM_UNNECESSARY;
		virt_net->rx_skb->protocol = eth_type_trans(virt_net->rx_skb,
						virt_net->rx_skb->dev);

		pr_debug("backend rcv: skb proto 0x%04x len %i type %i, vring idx %d %d\n",
				ntohs(virt_net->rx_skb->protocol), virt_net->rx_skb->len,
				virt_net->rx_skb->pkt_type,
				vrh->last_avail_idx, vring->vr.avail->idx);

		napi_gro_receive(napi, virt_net->rx_skb);

		virt_net->rx_skb = NULL;
	}

	return packages;
}

bool vnet_pending(void)
{
	struct light_vring *vring = &vdev->vvr[RCV_Q].vring;
	struct vringh *vrh = &vdev->vvr[RCV_Q].vrh;

	return vrh->last_avail_idx != vring->vr.avail->idx;
}

static void vnet_napi_complete(struct napi_struct *napi, int processed)
{
	vnet_enable_frontend_notify(RCV_Q);
	if (likely(napi_complete_done(napi, processed))) {
		if (unlikely(vnet_pending()))
			vnet_napi_schedule(napi);
	} else
		vnet_disable_frontend_notify(RCV_Q);
}

static int vnet_poll(struct napi_struct *napi, int budget)
{
	u32 received;

	received = vnet_receive(napi, budget);

	/* all work done, exit the polling mode */
	if (received < budget)
		vnet_napi_complete(napi, received);

	return received;
}

int light_vnet_dev_init(void)
{
	int ret;
	int max_queue_pairs;

	/* Create netdev */
	max_queue_pairs = 1;
	net_dev = alloc_netdev_mqs(sizeof(struct virtnet_priv), "veth%d",
				   NET_NAME_UNKNOWN, ether_setup,
				   max_queue_pairs, max_queue_pairs);
	if (!net_dev) {
		ret = -ENOMEM;
		pr_err("Net backend: no memory for net dev\n");
		return ret;
	}
	net_dev->netdev_ops = &vnet_ops;

	net_dev->min_mtu = MIN_MTU;
	net_dev->mtu = DEFAULT_MTU;
	net_dev->max_mtu = MAX_MTU;

	net_dev->watchdog_timeo = VNET_TIMEOUT;

	/* random mac */
	eth_hw_addr_random(net_dev);

	ret = register_netdev(net_dev);
	if (ret) {
		pr_err("Net backend: registering device failed\n");
		goto err1;
	}

	virt_net = netdev_priv(net_dev);
	if (!virt_net)
		goto err;
	virt_net->rx_skb = NULL;

	netif_napi_add(net_dev, &vdev->napi, vnet_poll, napi_weight);

	pr_info("Net backend: registered device %s with %d RX and TX vq's",
			net_dev->name, max_queue_pairs);

	return 0;

err:
	unregister_netdev(net_dev);
err1:
	free_netdev(net_dev);
	return ret;
}

static void light_vnet_fill_features(void)
{
	u64 features = 0;

	/* Add features and its value */
	writel(1, virtio_config + VIRTIO_LIGHT_VERSION);

	writel(256, virtio_config + VIRTIO_LIGHT_QUEUE_SIZE_MAX);

	features |= 1ULL << VIRTIO_NET_F_MTU;
	features |= 1ULL << VIRTIO_NET_S_LINK_UP;

	writel(features, virtio_config + VIRTIO_LIGHT_DEVICE_FEATURES_LOW);

	writel(DEFAULT_MTU, virtio_config + VIRTIO_LIGHT_CONFIG +
		offsetof(struct virtio_net_config, mtu));

}

static void vnet_work_handler(struct work_struct *work)
{
	light_vnet_fill_features();

	if (light_vnet_vring_map()) {
		pr_err("Net backend: Init virtio_net backend failed!!!\n");
		return;
	}

	if (light_vnet_dev_init())
		pr_err("Net backend: Create netdev failed!!!\n");
}

static int light_vnet_sync_dma(dma_addr_t dma_dst, dma_addr_t dma_src, size_t len, int queue)
{
	dma_cookie_t cookie;
	struct dma_async_tx_descriptor *tx_desc;
	struct dma_chan *dma_chan = vdev->dma_ch[queue];

	tx_desc = dmaengine_prep_dma_memcpy(dma_chan, dma_dst, dma_src, len, DMA_CTRL_ACK);
	if (!tx_desc)
		goto err;
	else {
		cookie = dmaengine_submit(tx_desc);
		if (dma_submit_error(cookie))
			goto err;

		if (dma_sync_wait(dma_chan, cookie))
			goto err;
	}
	return 0;
err:
	return -1;
}

static int vnet_dma_or_memcpy(struct light_vdev *vdev, void *ubuf, size_t len,
				      unsigned long daddr, int vr_idx, bool read)
{
	if (light_use_dma) {
		dma_addr_t dma_src, dma_dst, dma_addr, daddr_dma;
		enum dma_data_direction dir;

		/* Why we need map frontend addr?
		 * 'daddr' is filled in virtqueue_add_split():
		 *     dma_addr_t addr = vring_map_one_sg(vq, sg, DMA_TO_DEVICE);
		 * actually, in vring_map_one_sg(), it will judge 'if (!vq->use_dma_api)',
		 * because currently vq->use_dma_api is set to false in frontend, so in
		 * backend we only can got the physical address, not the dma mapped address:
		 *     if (!vq->use_dma_api)
		 *          return (dma_addr_t)sg_phys(sg);//it is physical address
		 *
		 * so in backend, we need call dma_map_single() to do dma map as below code.
		 * please note, the map direction should be the same with the one set in
		 * virtqueue_add_split().
		 */
		daddr_dma = dma_map_single(&vdev->dev, (void *)daddr, len,
					   read ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		if (dma_mapping_error(&vdev->dev, daddr_dma))
			goto memcpy;

		if (read) {
			dir = DMA_FROM_DEVICE;
			dma_dst = dma_map_single(&vdev->dev, ubuf, len, dir);
			if (dma_mapping_error(&vdev->dev, dma_dst)) {
				dma_unmap_single(&vdev->dev, daddr_dma, len, dir);
				goto memcpy;
			}
			dma_src = daddr_dma;
			dma_addr = dma_dst;
		} else {
			dir = DMA_TO_DEVICE;
			dma_src = dma_map_single(&vdev->dev, ubuf, len, dir);
			if (dma_mapping_error(&vdev->dev, dma_src)) {
				dma_unmap_single(&vdev->dev, daddr_dma, len, dir);
				goto memcpy;
			}
			dma_dst = daddr_dma;
			dma_addr = dma_src;
		}

		if (light_vnet_sync_dma(dma_dst, dma_src, len, vr_idx)) {
			dma_unmap_single(&vdev->dev, dma_addr, len, dir);
			dma_unmap_single(&vdev->dev, daddr_dma, len,
					 read ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
			goto memcpy;
		}

		dma_unmap_single(&vdev->dev, dma_addr, len, dir);
		dma_unmap_single(&vdev->dev, daddr_dma, len,
				 read ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	} else {
memcpy:
		if (read)
			memcpy(ubuf, (void *)daddr, len);
		else
			memcpy((void *)daddr, ubuf, len);

		vdev->out_bytes += len;
	}

	return 0;
}

static inline u32 light_vringh_iov_consumed(struct vringh_kiov *iov)
{
	int i;
	u32 total = iov->consumed;

	for (i = 0; i < iov->i; i++)
		total += iov->iov[i].iov_len;

	return total;
}

static int vnet_vringh_copy(struct light_vdev *vdev, struct vringh_kiov *iov,
			   void *ubuf, size_t len, bool read, int vr_idx,
			   size_t *out_len)
{
	int ret = 0;
	size_t partlen, tot_len = 0;
	unsigned long daddr;

	while (len && iov->i < iov->used) {
		struct kvec *kiov = &iov->iov[iov->i];

		daddr = (unsigned long)phys_to_virt((unsigned long)kiov->iov_base);

		partlen = min(kiov->iov_len, len);
		ret = vnet_dma_or_memcpy(vdev, ubuf, partlen,
						 daddr, vr_idx, read);
		if (ret) {
			pr_err("%s %d err %d\n", __func__, __LINE__, ret);
			break;
		}
		len -= partlen;
		ubuf += partlen;
		tot_len += partlen;
		iov->consumed += partlen;
		kiov->iov_len -= partlen;
		kiov->iov_base += partlen;
		if (!kiov->iov_len) {
			/* Fix up old iov element then increment. */
			kiov->iov_len = iov->consumed;
			kiov->iov_base -= iov->consumed;

			iov->consumed = 0;
			iov->i++;
		}
	}
	*out_len = tot_len;
	return ret;
}

static int vnet_desc_iov_process(struct light_vdev *vdev, struct light_copy_desc *copy, bool recv)
{
	int ret;
	u32 iovcnt = copy->iovcnt;
	struct iovec *kiov = copy->iov;
	void *ubuf = NULL;
	struct light_vringh *vvr = &vdev->vvr[copy->vr_idx];
	struct vringh_kiov *riov = &vvr->riov;
	struct vringh_kiov *wiov = &vvr->wiov;
	struct vringh *vrh = &vvr->vrh;
	u16 *head = &vvr->head;
	size_t len = 0, out_len;
	u32 payload_len = 0;
	int i;
	bool kick = true;

	copy->out_len = 0;

	/* Fetch a new IOVEC if all previous elements have been processed */
	if (riov->i == riov->used && wiov->i == wiov->used) {
		ret = vringh_getdesc_kern(vrh, riov, wiov, head, GFP_ATOMIC);
		/* Check if there are available descriptors */
		if (ret <= 0)
			return -EFAULT; /* don't returen ret, as 0 doesn't mean OK here */
	} else
		/* We only use this kick indication in xmit path */
		kick = !netdev_xmit_more();

	/*
	 * riov recored the packages send(VRING_DESC_F_WRITE not set) by frontend,
	 * so we calculate length of packages payload only based on riov.
	 * The purpose is to firstly get payload_len for skb_put(), then in below
	 * while, we can copy the payload data to right place of skb.
	 */
	if (recv) {
		for (i = riov->i; i < riov->used && riov->used != 0; i++)
			payload_len += riov->iov[i].iov_len;
		kiov->iov_base = skb_put((struct sk_buff *)kiov->iov_base, payload_len);
	}

	while (iovcnt) {
		if (!len) {
			len = kiov->iov_len;
			ubuf = kiov->iov_base;
		}

		vnet_vringh_copy(vdev, riov, ubuf, len,
				      true, copy->vr_idx, &out_len);
		len -= out_len;
		ubuf += out_len;
		copy->out_len += out_len;

		vnet_vringh_copy(vdev, wiov, ubuf, len,
				false, copy->vr_idx, &out_len);
		len -= out_len;
		ubuf += out_len;
		copy->out_len += out_len;

		if (!len) {
			iovcnt--;
			kiov++;
		}

		if (riov->i == riov->used && wiov->i == wiov->used)
			break;
	}

	if (recv && payload_len != copy->out_len)
		pr_err("Recv: data len doesn't match(%d %d)!!!\n", payload_len, copy->out_len);

	/*
	 * Update the used ring if a descriptor was available and some data was
	 * copied in/out and the user asked for a used ring update.
	 */
	if (*head != USHRT_MAX && copy->out_len && copy->update_used) {
		u32 total = 0;

		/* Determine the total data consumed */
		total += light_vringh_iov_consumed(riov);
		total += light_vringh_iov_consumed(wiov);

		vringh_complete_kern(vrh, *head, total);
		*head = USHRT_MAX;

		/* Make sure kick is after vringh_need_notify_kern() */
		if (vringh_need_notify_kern(vrh) > 0 && kick)
			vringh_notify(vrh);
		vringh_kiov_cleanup(riov);
		vringh_kiov_cleanup(wiov);
	}

	return 0;
}

static int light_vnet_register_dev(struct device *dev)
{
	int err;
	u64 dma_mask = DMA_BIT_MASK(32);

	vdev->dev.parent = dev;
	vdev->dev.dma_mask = (u64 *)&dma_mask;
	vdev->dev.coherent_dma_mask = dma_mask;

	dev_set_name(&vdev->dev, "light_vnet_dev");

	err = device_register(&vdev->dev);
	if (err)
		goto free_dev;
	return 0;

free_dev:
	put_device(&vdev->dev);
	return err;
}

static int light_vnet_probe(struct platform_device *pdev)
{
	static struct device_node *node;
	struct device *dev;
	int rc = 0;
	int irq;
	char magic[4] = {'v', 'i', 'r', 't'};

	dev = &pdev->dev;
	node = dev->of_node;

	/* Alloc light_vdev */
	vdev = kzalloc(sizeof(struct light_vdev), GFP_KERNEL);
	if (!vdev)
		return -ENOMEM;

	rc = light_vnet_register_dev(dev);
	if (rc) {
		pr_err("Net backend: dma config err %d, use memcpy!!\n", rc);
		light_use_dma = false;
	}

	INIT_WORK(&vdev->work, vnet_work_handler);

	virtio_config = ioremap(0x1ff000, VIRTIO_LIGHT_CONFIG_LEN);

	/* Reset status, frontend will fill it */
	writel(0, virtio_config + VIRTIO_LIGHT_STATUS);

	vdev->virtio_id = VIRTIO_ID_NET;
	writel(vdev->virtio_id, virtio_config + VIRTIO_LIGHT_DEVICE_ID);

	writel(*(u32 *)((void *)magic), virtio_config +
			VIRTIO_LIGHT_MAGIC_VALUE);

#ifdef CONFIG_SOC_INT_SRC7
	vdev->frontend_intr_reg = ioremap(0xFFEF018094, 4);
	vdev->backend_intr_reg = ioremap(0xFFFF019094, 4);
#else
	/* MPW use mailbox as interrupt */
	vdev->frontend_intr_reg = ioremap(0xffefc50000, 0x100);
	vdev->backend_intr_reg  = ioremap(0xffffc3b000, 0x100);
	vdev->enable_intr = 1;
	vdev->clear_intr = 1;
	writel(vdev->enable_intr, vdev->frontend_intr_reg + 0xc);
	writel(vdev->clear_intr, vdev->frontend_intr_reg + 0x4);
#endif

	/* Common IRQ request */
	irq = of_irq_get(node, 0);
	if (irq <= 0) {
		pr_err("Cannot get IRQ resource for net backend\n");
		rc = -EINVAL;
		goto err;
	}
	vdev->irq = irq;

	schedule_work(&vdev->work);

	pr_info("light_net backend driver init successfully\n");

	return 0;

err:
	kfree(vdev);
	return rc;
}

static int light_vnet_remove(struct platform_device *pdev)
{
	unregister_netdev(net_dev);
	free_netdev(net_dev);
	cancel_work_sync(&vdev->work);
	free_irq(vdev->irq, vdev);
	vnet_free_dma_chans(vdev);
	kfree(vdev);

	return 0;
}

static const struct of_device_id light_vnet_of_table[] = {
	{ .compatible = "thead,virtnet-backend" },
	{ }
};

MODULE_DEVICE_TABLE(of, light_vnet_of_table);

static struct platform_driver light_vnet_driver = {
	.probe  = light_vnet_probe,
	.remove = light_vnet_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = light_vnet_of_table,
	},
};
module_platform_driver(light_vnet_driver);

MODULE_DESCRIPTION("T-head light virtnet backend driver");
MODULE_AUTHOR("Xianting Tian <xianting.tian@linux.alibaba.com>");
MODULE_LICENSE("GPL");
