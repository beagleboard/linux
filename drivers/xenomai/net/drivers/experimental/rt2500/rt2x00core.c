/* rt2x00core.c
 *
 * Copyright (C) 2004 - 2005 rt2x00-2.0.0-b3 SourceForge Project
 *			     <http://rt2x00.serialmonkey.com>
 *               2006        rtnet adaption by Daniel Gregorek
 *                           <dxg@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*
 * Module: rt2x00core
 * Abstract: rt2x00 core routines.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <asm/io.h>

#include <rtnet_port.h>

#include "rt2x00.h"

#ifdef DRV_NAME
#undef DRV_NAME
#define DRV_NAME			"rt_rt2x00core"
#endif /* DRV_NAME */

static int rt2x00_radio_on(struct _rt2x00_core * core);
static int rt2x00_radio_off(struct _rt2x00_core * core);

static int cards[MAX_UNITS] = { [0 ... (MAX_UNITS-1)] = 1 };
module_param_array(cards, int, NULL, 0444);
MODULE_PARM_DESC(cards, "array of cards to be supported (e.g. 1,0,1)");

/*
 * Writes the pending configuration to the device
 */
static void rt2x00_update_config(struct _rt2x00_core * core) {

    u16			   update_flags = 0x0000;

    if(!test_bit(DEVICE_ENABLED, &core->flags)
       && !test_bit(DEVICE_RADIO_ON, &core->flags))
	return;

    if(test_and_set_bit(DEVICE_CONFIG_UPDATE, &core->flags))
	return;

    update_flags = core->config.update_flags;
    core->config.update_flags = 0;

    if(likely(update_flags))
       core->handler->dev_update_config(core, update_flags);

    clear_bit(DEVICE_CONFIG_UPDATE,&core->flags);
}

/*
 * Radio control.
 */
static int rt2x00_radio_on(struct _rt2x00_core * core) {

    int status = 0x00000000;

    if(test_bit(DEVICE_RADIO_ON, &core->flags)) {
	WARNING("Radio already on.\n");
	return -ENOTCONN;
    }

    status = core->handler->dev_radio_on(core);
    if(status)
	return status;

    set_bit(DEVICE_RADIO_ON, &core->flags);

    return 0;

}

static int rt2x00_radio_off(struct _rt2x00_core * core) {

    if(!test_and_clear_bit(DEVICE_RADIO_ON, &core->flags)) {
	WARNING("Radio already off.\n");
	return -ENOTCONN;
    }

    core->handler->dev_radio_off(core);

    return 0;
}

/*
 * user space io handler
 */
static int rt2x00_ioctl(struct rtnet_device * rtnet_dev, unsigned int request, void * arg) {

    struct rtwlan_device * rtwlan_dev  = rtnetdev_priv(rtnet_dev);
    struct _rt2x00_core * core     = rtwlan_priv(rtwlan_dev);
    struct rtwlan_cmd * cmd;
    u8 rate, dsss_rate, ofdm_rate;
    u32 address, value;

    cmd = (struct rtwlan_cmd *)arg;

    switch(request) {

    case IOC_RTWLAN_IFINFO:
	cmd->args.info.bitrate = core->config.bitrate;
	cmd->args.info.channel = core->config.channel;
	cmd->args.info.retry   = core->config.short_retry;
	cmd->args.info.txpower = core->config.txpower;
	cmd->args.info.bbpsens = core->config.bbpsens;
	cmd->args.info.mode    = core->rtwlan_dev->mode;
	cmd->args.info.rx_packets = core->rtwlan_dev->stats.rx_packets;
	cmd->args.info.tx_packets = core->rtwlan_dev->stats.tx_packets;
	cmd->args.info.tx_retry   = core->rtwlan_dev->stats.tx_retry;
	cmd->args.info.autoresponder = core->config.config_flags & CONFIG_AUTORESP ? 1 : 0;
	cmd->args.info.dropbcast = core->config.config_flags & CONFIG_DROP_BCAST ? 1 : 0;
	cmd->args.info.dropmcast = core->config.config_flags & CONFIG_DROP_MCAST ? 1 : 0;
	DEBUG("rtwlan_dev->mode=%d\n", rtwlan_dev->mode);
	break;
    case IOC_RTWLAN_BITRATE:
	rate = cmd->args.set.bitrate;
	ofdm_rate = ieee80211_is_ofdm_rate(rate);
	dsss_rate = ieee80211_is_dsss_rate(rate);
	DEBUG("bitrate=%d\n", rate);
	if(!(dsss_rate ^ ofdm_rate))
	    NOTICE("Rate %d is not DSSS and not OFDM.\n", rate);
	core->config.bitrate = rate;
	core->config.update_flags |= UPDATE_BITRATE;
	break;
    case IOC_RTWLAN_CHANNEL:
	DEBUG("channel=%d\n", cmd->args.set.channel);
	core->config.channel = cmd->args.set.channel;
	core->config.update_flags |= UPDATE_CHANNEL;
	break;
    case IOC_RTWLAN_RETRY:
	core->config.short_retry = cmd->args.set.retry;
	core->config.update_flags |= UPDATE_RETRY;
	break;
    case IOC_RTWLAN_TXPOWER:
	core->config.txpower = cmd->args.set.txpower;
	core->config.update_flags |= UPDATE_TXPOWER;
	break;
    case IOC_RTWLAN_AUTORESP:
	if(cmd->args.set.autoresponder)
	    core->config.config_flags |= CONFIG_AUTORESP;
	else
	    core->config.config_flags &= ~CONFIG_AUTORESP;
	core->config.update_flags |= UPDATE_AUTORESP;
	break;
    case IOC_RTWLAN_DROPBCAST:
	if(cmd->args.set.dropbcast)
	    core->config.config_flags |= CONFIG_DROP_BCAST;
	else
	    core->config.config_flags &= ~CONFIG_DROP_BCAST;
	core->config.update_flags |= UPDATE_PACKET_FILTER;
	break;
    case IOC_RTWLAN_DROPMCAST:
	if(cmd->args.set.dropmcast)
	    core->config.config_flags |= CONFIG_DROP_MCAST;
	else
	    core->config.config_flags &= ~CONFIG_DROP_MCAST;
	core->config.update_flags |= UPDATE_PACKET_FILTER;
	break;
    case IOC_RTWLAN_TXMODE:
	core->rtwlan_dev->mode = cmd->args.set.mode;
	break;
    case IOC_RTWLAN_BBPSENS:
	value = cmd->args.set.bbpsens;
	if(value < 0)
	    value = 0;
	if(value > 127)
	    value = 127;
	core->config.bbpsens = value;
	core->config.update_flags |= UPDATE_BBPSENS;
	break;
    case IOC_RTWLAN_REGREAD:
    case IOC_RTWLAN_BBPREAD:
	address = cmd->args.reg.address;
	core->handler->dev_register_access(core, request, address, &value);
	cmd->args.reg.value = value;
	break;
    case IOC_RTWLAN_REGWRITE:
    case IOC_RTWLAN_BBPWRITE:
	address = cmd->args.reg.address;
	value = cmd->args.reg.value;
	core->handler->dev_register_access(core, request, address, &value) ;
	break;
    default:
	ERROR("Unknown request!\n");
	return -1;
    }

    if(request != IOC_RTWLAN_IFINFO)
	rt2x00_update_config(core);

    return 0;
}

/*
 * TX/RX related routines.
 */
static int rt2x00_start_xmit(struct rtskb *rtskb, struct rtnet_device *rtnet_dev) {

    struct rtwlan_device	* rtwlan_dev = rtnetdev_priv(rtnet_dev);
    struct _rt2x00_core		* core   = rtwlan_priv(rtwlan_dev);
    u16				xmit_flags = 0x0000;
    u8				rate = 0x00;

    if (unlikely(rtskb)) {

	rate = core->config.bitrate;
	if(ieee80211_is_ofdm_rate(rate))
	    xmit_flags |= XMIT_OFDM;

	/* Check if the packet should be acknowledged */
	if(core->rtwlan_dev->mode == RTWLAN_TXMODE_ACK)
	    xmit_flags |= XMIT_ACK;

	if(core->handler->dev_xmit_packet(core, rtskb, rate, xmit_flags))
	    ERROR("Packet dropped !");

	dev_kfree_rtskb(rtskb);
    }

    return 0;
}

/***
 *  rt2x00_open
 *  @rtdev
 */
static int rt2x00_open (struct rtnet_device *rtnet_dev) {

    struct rtwlan_device * rtwlan_dev = rtnetdev_priv(rtnet_dev);
    struct _rt2x00_core  * core   = rtwlan_priv(rtwlan_dev);
    int			  status  = 0x00000000;

    DEBUG("Start.\n");

    if(test_and_set_bit(DEVICE_ENABLED, &core->flags)){
	ERROR("device already enabled.\n");
	return -EBUSY;
    }

    /*
     * Start rtnet interface.
     */
    rt_stack_connect(rtnet_dev, &STACK_manager);

    status = rt2x00_radio_on(core);
    if(status){
	clear_bit(DEVICE_ENABLED, &core->flags);
	ERROR("Couldn't activate radio.\n");
	return status;
    }

    core->config.led_status = 1;
    core->config.update_flags |= UPDATE_LED_STATUS;
    rt2x00_update_config(core);

    rtnetif_start_queue(rtnet_dev);

    DEBUG("Exit success.\n");

    return 0;
}


/***
 *  rt2x00_close
 *  @rtdev
 */
static int rt2x00_close (struct rtnet_device *rtnet_dev) {

    struct rtwlan_device * rtwlan_dev = rtnetdev_priv(rtnet_dev);
    struct _rt2x00_core  * core   = rtwlan_priv(rtwlan_dev);

    DEBUG("Start.\n");

    if(!test_and_clear_bit(DEVICE_ENABLED, &core->flags)){
	ERROR("device already disabled.\n");
	return -EBUSY;
    }

    rt2x00_radio_off(core);

    rtnetif_stop_queue(rtnet_dev);
    rt_stack_disconnect(rtnet_dev);

    return 0;
}


/*
 * Initialization handlers.
 */
static void rt2x00_init_config(struct _rt2x00_core *core) {

    DEBUG("Start.\n");

    memset(&core->config.bssid, '\0', sizeof(core->config.bssid));

    core->config.channel = 1;
    core->config.bitrate = capabilities.bitrate[0];
    core->config.bbpsens = 50;
    core->config.config_flags = 0;
    core->config.config_flags |= CONFIG_DROP_BCAST | CONFIG_DROP_MCAST | CONFIG_AUTORESP;
    core->config.short_retry = 4;
    core->config.long_retry = 7;
    core->config.txpower = 100;
    core->config.plcp = 48;
    core->config.sifs = 10;
    core->config.slot_time = 20;
    core->rtwlan_dev->mode = RTWLAN_TXMODE_RAW;
    core->config.update_flags = UPDATE_ALL_CONFIG;
}

struct rtnet_device * rt2x00_core_probe(struct _rt2x00_dev_handler * handler,
					void * priv,
					u32 sizeof_dev) {

    struct rtnet_device	 * rtnet_dev  = NULL;
    struct _rt2x00_core	 * core       = NULL;
    struct rtwlan_device * rtwlan_dev = NULL;
    static int cards_found = -1;
    int err;

    DEBUG("Start.\n");

    cards_found++;
    if (cards[cards_found] == 0)
	goto exit;

    rtnet_dev = rtwlan_alloc_dev(sizeof_dev + sizeof(*core), RX_ENTRIES*2);
    if(!rtnet_dev)
	goto exit;

    rt_rtdev_connect(rtnet_dev, &RTDEV_manager);
    rtnet_dev->vers = RTDEV_VERS_2_0;

    rtwlan_dev = rtnetdev_priv(rtnet_dev);
    memset(rtwlan_dev, 0x00, sizeof(*rtwlan_dev));

    core = rtwlan_priv(rtwlan_dev);
    memset(core, 0x00, sizeof(*core));

    core->rtwlan_dev = rtwlan_dev;
    core->handler = handler;
    core->priv = (void*)core + sizeof(*core);
    core->rtnet_dev = rtnet_dev;

    /* Set configuration default values. */
    rt2x00_init_config(core);

    if(core->handler->dev_probe
       && core->handler->dev_probe(core, priv)){
	ERROR("device probe failed.\n");
	goto exit;
    }
    INFO("Device " MAC_FMT " detected.\n", MAC_ARG(rtnet_dev->dev_addr));

    rtwlan_dev->hard_start_xmit = rt2x00_start_xmit;

    rtnet_dev->open = &rt2x00_open;
    rtnet_dev->stop = &rt2x00_close;
    rtnet_dev->do_ioctl = &rt2x00_ioctl;
    rtnet_dev->hard_header = &rt_eth_header;

    if ((err = rt_register_rtnetdev(rtnet_dev)) != 0) {
	rtdev_free(rtnet_dev);
	ERROR("rtnet_device registration failed.\n");
	printk("err=%d\n", err);
	goto exit_dev_remove;
    }

    set_bit(DEVICE_AWAKE, &core->flags);

    return rtnet_dev;

  exit_dev_remove:
    if(core->handler->dev_remove)
	core->handler->dev_remove(core);

  exit:
    return NULL;
}
EXPORT_SYMBOL_GPL(rt2x00_core_probe);

void rt2x00_core_remove(struct rtnet_device * rtnet_dev) {

    rt_unregister_rtnetdev(rtnet_dev);
    rt_rtdev_disconnect(rtnet_dev);

    rtdev_free(rtnet_dev);
}
EXPORT_SYMBOL_GPL(rt2x00_core_remove);

/*
 * RT2x00 core module information.
 */
static char version[] = DRV_NAME " - " DRV_VERSION;

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION("RTnet rt2500 PCI WLAN driver (Core Module)");
MODULE_LICENSE("GPL");

static int __init rt2x00_core_init(void) {
    printk(KERN_INFO "Loading module: %s\n", version);
    return 0;
}

static void __exit rt2x00_core_exit(void) {
    printk(KERN_INFO "Unloading module: %s\n", version);
}

module_init(rt2x00_core_init);
module_exit(rt2x00_core_exit);
