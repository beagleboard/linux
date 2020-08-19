#include "avb_util.h"

void avb_log(int level, char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	switch (level) {
	case AVB_KERN_EMERG:
		vprintk(fmt, args);
		break;
	case AVB_KERN_ALERT:
		vprintk(fmt, args);
		break;
	case AVB_KERN_CRIT:
		vprintk(fmt, args);
		break;
	case AVB_KERN_ERR:
		vprintk(fmt, args);
		break;
	case AVB_KERN_WARN:
		vprintk(fmt, args);
		break;
	case AVB_KERN_NOT:
		vprintk(fmt, args);
		break;
#ifdef AVB_DEBUG
	case AVB_KERN_INFO:
		vprintk(fmt, args);
		break;
	case AVB_KERN_DEBUG:
		vprintk(fmt, args);
		break;
#else
	default:
		break;
#endif
	}

	va_end(args);
}

bool avb_socket_init(struct socketdata *sd, int rx_timeout)
{
	int err = 0;
	struct net_device *dev = NULL;
	struct timeval ts_opts;
	ts_opts.tv_sec = (rx_timeout / 1000);
	ts_opts.tv_usec = (rx_timeout % 1000);

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_socket_init");

	dev = dev_get_by_name_rcu(&init_net, "eth0");

	if ((err = sock_create_kern(&init_net, AF_PACKET, SOCK_RAW,
				    htons(sd->type), &sd->sock)) != 0) {
		avb_log(AVB_KERN_ERR,
			KERN_ERR "avb_socket_init Socket creation fails %d \n",
			err);
		return false;
	}

	memcpy(&sd->srcmac[0], dev->dev_addr, 6);
	sd->if_idx = dev->ifindex;

	rtnl_lock();
	dev_set_promiscuity(dev, 1);
	rtnl_unlock();

	if ((err = kernel_setsockopt(sd->sock, SOL_SOCKET, SO_RCVTIMEO_OLD,
				     (void *)&ts_opts, sizeof(ts_opts))) != 0) {
		avb_log(AVB_KERN_WARN,
			KERN_WARNING "avb_msrp_init set rx timeout fails %d\n",
			err);
		return false;
	}

	/* Index of the network device */
	sd->tx_sock_address.sll_family = AF_PACKET;
	sd->tx_sock_address.sll_protocol = htons(sd->type);
	sd->tx_sock_address.sll_ifindex = sd->if_idx;
	/* Address length*/
	sd->tx_sock_address.sll_halen = ETH_ALEN;
	/* Destination MAC */
	sd->tx_sock_address.sll_addr[0] = sd->destmac[0];
	sd->tx_sock_address.sll_addr[1] = sd->destmac[1];
	sd->tx_sock_address.sll_addr[2] = sd->destmac[2];
	sd->tx_sock_address.sll_addr[3] = sd->destmac[3];
	sd->tx_sock_address.sll_addr[4] = sd->destmac[4];
	sd->tx_sock_address.sll_addr[5] = sd->destmac[5];

	/* Set the message header */
	sd->tx_msg_hdr.msg_control = NULL;
	sd->tx_msg_hdr.msg_controllen = 0;
	sd->tx_msg_hdr.msg_flags = 0;
	sd->tx_msg_hdr.msg_name = &sd->tx_sock_address;
	sd->tx_msg_hdr.msg_namelen = sizeof(struct sockaddr_ll);
	sd->tx_msg_hdr.msg_iocb = NULL;

	/* Index of the network device */
	sd->rx_sock_address.sll_family = AF_PACKET;
	sd->rx_sock_address.sll_protocol = htons(sd->type);
	sd->rx_sock_address.sll_ifindex = sd->if_idx;
	/* Address length*/
	sd->rx_sock_address.sll_halen = ETH_ALEN;
	/* Destination MAC */
	sd->rx_sock_address.sll_addr[0] = sd->destmac[0];
	sd->rx_sock_address.sll_addr[1] = sd->destmac[1];
	sd->rx_sock_address.sll_addr[2] = sd->destmac[2];
	sd->rx_sock_address.sll_addr[3] = sd->destmac[3];
	sd->rx_sock_address.sll_addr[4] = sd->destmac[4];
	sd->rx_sock_address.sll_addr[5] = sd->destmac[5];

	/* Set the message header */
	sd->rx_msg_hdr.msg_control = NULL;
	sd->rx_msg_hdr.msg_controllen = 0;
	sd->rx_msg_hdr.msg_flags = 0;
	sd->rx_msg_hdr.msg_name = &sd->rx_sock_address;
	sd->rx_msg_hdr.msg_namelen = sizeof(struct sockaddr_ll);
	sd->rx_msg_hdr.msg_iocb = NULL;
	sd->rx_iov.iov_base = sd->rx_buf;
	sd->rx_iov.iov_len = AVB_MAX_ETH_FRAME_SIZE;
	iov_iter_init(&sd->rx_msg_hdr.msg_iter, READ, &sd->rx_iov, 1,
		      AVB_MAX_ETH_FRAME_SIZE);

	if ((err = kernel_bind(sd->sock,
			       (struct sockaddr *)&sd->rx_sock_address,
			       sizeof(sd->rx_sock_address))) != 0) {
		avb_log(AVB_KERN_WARN,
			KERN_WARNING "avb socket binding fails %d\n", err);
		return false;
	}

	return true;
}