/*
 * This file is part of TI BLE over SDIO
 *
 * Copyright (C) 2022 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */


#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/suspend.h>

#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio_func.h>
#include <linux/module.h>
#include <linux/devcoredump.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "btti_drv.h"
#include "btti_sdio.h"

#define VERSION "ti_1.0"

#define SDIO_DEVICE_ID_TI_CC33XX	0x4077


static void btti_sdio_if_remove(struct sdio_func *func);
static int __maybe_unused btti_sdio_unregister_dev(struct sdio_func *func);
static int btti_sdio_rx_packet(struct btti_private *private_dataate_data);
static int btti_sdio_tx_packet(struct btti_private *private_data,
		struct sk_buff *skb);
static void btti_sdio_irq_handler(struct sdio_func *func);
static int btti_sdio_power_up_fw(struct btti_private *private_data);
static int btti_handle_rx_vendor_event(struct btti_private *private_data,
		struct sk_buff *skb );
static void btti_acknldg_packet(struct btti_sdio_dev *sdiodev, u8 Ack);



static const struct of_device_id btti_sdio_of_match_table[] = {
	{ .compatible = "ti,cc33xxbt" },
	{ }
};

static int btti_sdio_if_probe_of(struct device *dev)
{

	if (!dev || !dev->of_node ||
	    !of_match_node(btti_sdio_of_match_table, dev->of_node)) {
			BT_ERR("[bt sdio] sdio device tree data "\
					" not available\n");
		return -1;
	}

	return 0;
}

static const struct btti_sdio_dev_reg_map btti_reg_map_cc33xx = {
	/* fun0 ,ELP Wakeup Reg address, func0 not used*/
	.sdio_wup_ble = 0x40,
	/*fun1, set 0x1, to enable interrupts, 0 to disable */
	.sdio_enable_int = 0x14,
	/* fun1,read and write to the card are at 0x0 address */
	.sdio_rt_data = 0x0,
	/*  fun1,write 1 to clear rx interrupt after reception*/
	.sdio_cl_int = 0x13,
	 /*fun1, bt mode status, bit 1 means ack mode,
	  *  currently the code just read it */
	.bt_mode_status = 0x20,
	/* fun1 read packet control, write 1 is set ack for RX,
	 *  write 0 for nack */
	.sdio_pc_rrt = 0x10,
};

static const struct btti_sdio_device btti_sdio_cc33xx_bt = {
	.reg_map		= &btti_reg_map_cc33xx,
};

static const struct sdio_device_id btti_sdio_ids[] = {
    { SDIO_DEVICE(SDIO_VENDOR_ID_TI, SDIO_DEVICE_ID_TI_CC33XX) ,.driver_data\
		    = (unsigned long)&btti_sdio_cc33xx_bt },
	{ }	/* Terminating entry */
};

MODULE_DEVICE_TABLE(sdio, btti_sdio_ids);

static int btti_sdio_enable_func_interrupt(struct btti_sdio_dev *sdiodev,
								u8 mask)
{
	int ret;

	sdio_writeb(sdiodev->func, mask, sdiodev->reg_map->sdio_enable_int,\
			&ret);
	if (ret) {
		BT_ERR("[bt sdio] Unable to enable the host interrupt!");
		ret = -EIO;
	}

	return ret;
}

static int btti_sdio_disable_func_interrupt(struct btti_sdio_dev *sdiodev,
								u8 mask)
{
	u8 sdio_enable_int_mask;
	int ret;

	sdio_enable_int_mask = sdio_readb(sdiodev->func,\
			sdiodev->reg_map->sdio_enable_int, &ret);
	if (ret)
		return -EIO;

	sdio_enable_int_mask &= ~mask;

	sdio_writeb(sdiodev->func, sdio_enable_int_mask,\
			sdiodev->reg_map->sdio_enable_int, &ret);
	if (ret < 0) {
		BT_ERR("[bt sdio] Unable to disable the host interrupt!");
		return -EIO;
	}

	return 0;
}



//Read Rx, called from btti_service_work_thread
static int btti_sdio_process_rx(struct btti_private *private_data)
{
	ulong flags;
	u8 intrpt_occur;
	struct btti_sdio_dev *sdiodev = private_data->btti_dev.sdiodev;

	spin_lock_irqsave(&private_data->irq_cnt_lock, flags);
	intrpt_occur = sdiodev->Intrpt_triggered;
	sdiodev->Intrpt_triggered = 0;
	spin_unlock_irqrestore(&private_data->irq_cnt_lock, flags);

	sdio_claim_host(sdiodev->func);
	if (intrpt_occur)
	{
		btti_sdio_rx_packet(private_data);//read Rx,
	}
	else{
		btti_acknldg_packet(sdiodev,1);//nack
	}

	sdio_release_host(sdiodev->func);

	return 0;
}

static int btti_sdio_clr_irq(struct btti_sdio_dev *sdiodev)
{
	int ret;

	BT_DBG("[bt sdio] clear int_status");

	sdio_writeb(sdiodev->func, 1 ,sdiodev->reg_map->sdio_cl_int, &ret);
	if (ret) {
		BT_ERR("[bt sdio] clear int status failed: %d", ret);
		return ret;
	}

	return 0;
}


static int btti_sdio_register_dev(struct btti_sdio_dev *sdiodev)
{
	struct sdio_func *func;
	u8 reg;
	int ret;

	if (!sdiodev || !sdiodev->func) {
		BT_ERR("[bt sdio] Error: sdiodev or function is NULL!");
		ret = -EINVAL;
		goto failed;
	}

	func = sdiodev->func;

	sdio_claim_host(func);

	func->enable_timeout = 0;

	ret = sdio_enable_func(func);
	if (ret) {
		//this is ok, since the ble is not up yet so SDIO_CCCR_IOEx
		//is not set on this sage
		BT_DBG("[bt sdio] sdio_enable_func: %d failed: ret=%d",\
				func->num, ret );
	}
	else{
		BT_INFO("[bt sdio] sdio_enable_func: %d success", func->num );
	}

	//set block size
	ret = sdio_set_block_size(sdiodev->func, SDIO_BLOCK_SIZE);
	if (ret) {
		BT_ERR("[bt sdio] cannot set SDIO block size");
		ret = -EIO;
		goto unreg_device;
	}

	//clear interrupt, write to clear
	ret = btti_sdio_clr_irq(sdiodev);

	//read status --not in use
	reg = sdio_readb(func, sdiodev->reg_map->bt_mode_status, &ret);
	if (ret < 0) {
		BT_ERR("[bt sdio] Failed to read bt_mode_status ");
	   ret = -EIO;
	   goto unreg_device;
	}
	sdiodev->bt_mode_status = ret;

	BT_DBG("[bt sdio] SDIO FUNC%d IO port: 0x%x BT MODE: 0x%x",\
			func->num, sdiodev->reg_map->sdio_rt_data,\
			sdiodev->bt_mode_status);

	sdio_set_drvdata(func, sdiodev);

	sdio_release_host(func);

	return 0;

unreg_device:
		sdio_release_host(func);
		btti_sdio_unregister_dev(func);
		return ret;
failed:
		return ret;

}

static int btti_sdio_unregister_dev(struct sdio_func *func)
{
	BT_INFO("[bt sdio] btti_sdio_unregister_dev ");

	if (func) {
		sdio_claim_host(func);
		BT_DBG("[bt sdio] sdio_release_irq");
		sdio_release_irq(func);
		sdio_disable_func(func);
		sdio_set_drvdata(func, NULL);
		sdio_release_host(func);
	}

	return 0;
}

static int btti_sdio_enable_int(struct btti_sdio_dev *sdiodev)
{
	int ret;

	if (!sdiodev || !sdiodev->func)
		return -EINVAL;

	sdio_claim_host(sdiodev->func);

	ret = btti_sdio_enable_func_interrupt(sdiodev, INTRPT_ENABLE_MASk);

	sdio_release_host(sdiodev->func);

	return ret;
}

static int btti_sdio_disable_int(struct btti_sdio_dev *sdiodev)
{
	int ret;

	if (!sdiodev || !sdiodev->func)
		return -EINVAL;

	sdio_claim_host(sdiodev->func);

	ret = btti_sdio_disable_func_interrupt(sdiodev, INTRPT_ENABLE_MASk);

	sdio_release_host(sdiodev->func);

	return ret;
}



static void btti_sdio_dump_regs(struct btti_private *private_data)
{
	struct btti_sdio_dev *sdiodev = private_data->btti_dev.sdiodev;
	int ret = 0;
	unsigned int reg, reg_start, reg_end;
	char buf[256], *ptr;
	u8 loop, func, data;
	int MAX_LOOP = 2;

	btti_sdio_power_up_fw(private_data);
	sdio_claim_host(sdiodev->func);

	for (loop = 0; loop < MAX_LOOP; loop++) {
		memset(buf, 0, sizeof(buf));
		ptr = buf;

		if (loop == 0) {
			/* Read the registers of SDIO function0 */
			func = loop;
			reg_start = 0;
			reg_end = 9;
		} else {
			func = 2;
			reg_start = 0;
			reg_end = 0x09;
		}

		ptr += sprintf(ptr, "[bt sdio] SDIO Func%d (%#x-%#x): ",
			       func, reg_start, reg_end);
		for (reg = reg_start; reg <= reg_end; reg++) {
			if (func == 0)
				data = sdio_f0_readb(sdiodev->func, reg, &ret);
			else
				data = sdio_readb(sdiodev->func, reg, &ret);

			if (!ret) {
				ptr += sprintf(ptr, "%02x ", data);
			} else {
				ptr += sprintf(ptr, "ERR");
				break;
			}
		}

		BT_INFO("%s", buf);
	}

	sdio_release_host(sdiodev->func);
}

/* This function dump sdio register and memory data */
static void btti_sdio_coredump(struct device *dev)
{
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct btti_sdio_dev *sdiodev;
	struct btti_private *private_data;

	sdiodev = sdio_get_drvdata(func);
	if(sdiodev)
	{
		private_data = sdiodev->private_data;
		if(!private_data){
			BT_ERR("[bt sdio] private_data is not allocated");
			return;
		}
	}else {
		BT_ERR("[bt sdio] no sdiodev");
		return;
	}

	/* dump sdio register first */
	btti_sdio_dump_regs(private_data);
}
static int btti_sdio_power_up_fw(struct btti_private *private_data)
{
	struct btti_sdio_dev *sdiodev = private_data->btti_dev.sdiodev;
	int ret = 0;

	BT_DBG("[bt sdio] power up FW");

	if (!sdiodev || !sdiodev->func) {
		BT_ERR("[bt sdio] sdiodev or function is NULL!");
		return -EINVAL;
	}

	sdio_claim_host(sdiodev->func);

	// BLE firmware remains awake , force Lx to remain awake;
	sdio_f0_writeb(sdiodev->func, HOST_POWER_UP_MASK,\
			sdiodev->reg_map->sdio_wup_ble, &ret);
	if (ret) {
		BT_ERR("[bt sdio] Unable to power up ble firmware!");
		ret = -EIO;
	}

	sdio_release_host(sdiodev->func);

	BT_INFO("[bt sdio] wake up firmware");

	return ret;
}

static int btti_sdio_power_dn_fw(struct btti_private *private_data)
{
	struct btti_sdio_dev *sdiodev = private_data->btti_dev.sdiodev;
	int ret = 0;

	BT_DBG("[bt sdio] power up FW");

	if (!sdiodev || !sdiodev->func) {
		BT_ERR("[bt sdio] sdiodev or function is NULL!");
		return -EINVAL;
	}

	sdio_claim_host(sdiodev->func);

	//  Lx Core can go to sleep (from host perspective)
	sdio_f0_writeb(sdiodev->func, ~HOST_POWER_UP_MASK,\
			sdiodev->reg_map->sdio_wup_ble, &ret);
	if (ret) {
		BT_ERR("[bt sdio] Unable to power down ble firmware!");
		ret = -EIO;
	}

	sdio_release_host(sdiodev->func);

	BT_INFO("[bt sdio] wake up firmware");

	return ret;
}
static void btti_sdio_irq_handler(struct sdio_func *func)
{
	struct btti_private *private_data;
	struct btti_sdio_dev *sdiodev;
	ulong flags;
	int ret;

	BT_INFO("[bt sdio] RX btti_sdio_irq_handler received");

	sdiodev = sdio_get_drvdata(func);
	if (!sdiodev || !sdiodev->private_data) {
		BT_ERR("[bt sdio] RX btti_hci_irq_handler(%d) "\
				"%p sdiodev or private_data is NULL,"\
				" sdiodev=%p",
		       func->num, func, sdiodev);
		return;
	}

	private_data = sdiodev->private_data;

	if (private_data->sdio_dev_removed)	{
		BT_ERR("[bt sdio] RX sdio_dev_removed");
		return;
	}

	sdio_claim_host(func);

	//clear interrupt
	ret = btti_sdio_clr_irq(sdiodev);

	sdio_release_host(func);

	if (ret){
		BT_ERR("[bt sdio] RX btti_sdio_irq_handler:failed to clear");
		return;
	}

	spin_lock_irqsave(&private_data->irq_cnt_lock, flags);
	sdiodev->Intrpt_triggered = 1;// signal that interrupt happened
	spin_unlock_irqrestore(&private_data->irq_cnt_lock, flags);


	btti_hci_irq_handler(private_data);
}


//rx
static int btti_sdio_rx_packet(struct btti_private *private_data)
{
	u32 packet_len = 0;
	int ret;
	u32 data_read_size;
	struct sk_buff *skb = NULL;
	u32 packet_type;

	struct hci_dev *hdev = private_data->btti_dev.hcidev;
	struct btti_sdio_dev *sdiodev = private_data->btti_dev.sdiodev;
	u8 *sdio_header = sdiodev->sdio_header;

	if (!sdiodev || !sdiodev->func) {
		BT_ERR("[bt sdio] RX sdiodev or function is NULL!");
		ret = -EINVAL;
		goto exit;
	}

	//Read the length of data to be transferred
	ret = sdio_readsb(sdiodev->func, sdio_header, \
			sdiodev->reg_map->sdio_rt_data, SDIO_HEADER_LEN);
	if (ret < 0) {
		BT_ERR("[bt sdio] RX read rx_len failed");
		ret = -EIO;
		goto exit;
	}

	packet_len = \
		sdio_header[0] | (sdio_header[1] << 8) | (sdio_header[2] << 16);
	packet_type = sdio_header[3];

	BT_INFO("[bt sdio] RX packet_len:%d packet_type:%d "\
			"packet header hex: %*ph", packet_len,packet_type,\
			SDIO_HEADER_LEN, sdio_header);

	if ((packet_len <= SDIO_HEADER_LEN)\
			|| (packet_len > HCI_MAX_FRAME_SIZE)) {
		BT_ERR("[bt sdio] RX packet length: %d not HCI"\
				" valid length", packet_len);
		ret = -EINVAL;
		goto exit;
	}


	if (packet_len > (TIDRV_SIZE_OF_CMD_BUFFER+SDIO_HEADER_LEN)) {
		BT_WARN("[bt sdio] RX packet length: %d not supported",\
				packet_len);
	}

	data_read_size = roundup((packet_len - SDIO_HEADER_LEN),\
			BTSDIO_RX_ALIGN);


	skb = bt_skb_alloc(data_read_size, GFP_KERNEL);
	if (!skb) {
		/* Out of memory. Prepare a read retry and just
		 * return with the expectation that the next time
		 * we're called we'll have more memory.
		 */
		BT_ERR("[bt sdio] No free skb");
		ret = -ENOMEM;
		goto exit;
	}

	skb_put(skb, packet_len-SDIO_HEADER_LEN);

	ret = sdio_readsb(sdiodev->func, skb->data,\
			sdiodev->reg_map->sdio_rt_data, data_read_size);
	if (ret < 0) {
		BT_ERR("[bt sdio] RX sdio_readsb failed: %d, size read:%d",\
				ret, data_read_size);
		ret = -EIO;
		goto exit;
	}
	BT_INFO("[bt sdio] RX packet , packet data(without header) hex: %*ph",\
			data_read_size, skb->data);

	switch (packet_type) {
	case HCI_ACLDATA_PKT:
	case HCI_SCODATA_PKT:
	case HCI_EVENT_PKT:
		if(hdev != NULL){
			btti_acknldg_packet(sdiodev, 0);//ack
			hdev->stat.byte_rx += data_read_size;
			hci_skb_pkt_type(skb) = packet_type;

			ret= hci_recv_frame(hdev, skb);
			if (ret < 0){
				BT_ERR("[bt sdio] RX hci_recv_frame"
						" failed :%d ", ret);
			}
			ret=0;
		}else
		{
			ret = -EPERM;
		}
		//no need to free the skb in here, it is freed at hci_recv_frame
		break;

	case HCI_VENDOR_PKT:
		BT_INFO("[bt sdio] vendor packet received");
		if (!(ret=btti_handle_rx_vendor_event(private_data, skb)))
		{
			//if hdev was just created read it
			hdev = private_data->btti_dev.hcidev;
			if(hdev != NULL)
			{
				btti_acknldg_packet(sdiodev, 0);//ack
				BT_INFO("[bt sdio] Hdev was created");
				hdev->stat.byte_rx += data_read_size;
				hci_skb_pkt_type(skb) = HCI_EVENT_PKT;
				ret= hci_recv_frame(hdev, skb);
				if (ret < 0){
					BT_ERR("[bt sdio] vendor RX "
							"hci_recv_frame "
							"failed :%d ", ret);
				}
				//no need to free the skb in here,
				//it is freed at hci_recv_frame
				ret=0;
			}else{
				ret=-EPERM;
			}
		}
		break;


	default:
		BT_ERR("[bt sdio] RX Unknown packet packet_type:%d",\
				packet_type);
		ret = -ENOENT;
		break;
	}

exit:
    //send nack
	if (ret) {
		if(hdev){
			hdev->stat.err_rx++;
		}
		kfree_skb(skb);
		skb = NULL;
		btti_acknldg_packet(sdiodev, 1);//nack
	}
	return ret;
}

static void btti_acknldg_packet(struct btti_sdio_dev *sdiodev, u8 Ack)
{
	int retACK = 0;

	//ack the packet, HCI will release the skb
	BT_DBG("[bt sdio] RX sdio_writesb: send ACK ");
	sdio_writeb(sdiodev->func, Ack,\
			sdiodev->reg_map->sdio_pc_rrt, &retACK);
	if (retACK && (Ack==0)) {
		BT_ERR("[bt sdio] RX sdio_writesb:"\
				" send ACK failed: %d", retACK);
	}
	if (retACK && (Ack==1)) {
		BT_ERR("[bt sdio] RX sdio_writesb:"\
				" send NACK failed: %d", retACK);
	}
}



static int btti_handle_rx_vendor_event(struct btti_private *private_data,
		struct sk_buff *skb )
{
	int ret = 0;
	struct btti_vendor_event* vendor_event;
	vendor_event = (struct btti_vendor_event *) skb->data;
	switch (vendor_event->event_opcode) {
		case BTTI_BLE_FIRMWARE_UP:
			BT_INFO("[bt sdio] vendor packet- ble is up");
			private_data->hci_adapter->ble_enable = 1;
			if (btti_hci_register_hdev(private_data)) {
				BT_ERR("[bt sdio] Register hdev failed!");
				ret = -ENODEV;
			}
			BT_INFO("[bt sdio] registered to HCI");

			break;
		default:
			BT_ERR("[bt sdio] unsupported rx vendor event code:"\
					" %d", vendor_event->event_code);
			ret = -EINVAL;
			break;
	}

	return ret;

}
//tx
//packet_len, the length includes the header.
static int btti_sdio_tx_packet(struct btti_private *private_data,
		struct sk_buff *skb)
{
	struct btti_sdio_dev *sdiodev = private_data->btti_dev.sdiodev;
	int ret = 0;
	int i = 0;
	void *tmpbuf = NULL;
	u32 data_send_size = 0;
	u32 packet_len;
	char* payload;
	bool alignment_required;


	BT_DBG("[bt sdio] TX btti_sdio_tx_packet");

	if (!sdiodev || !sdiodev->func) {
		BT_ERR("[bt sdio] sdiodev or function is NULL!");
		return -EINVAL;
	}

	packet_len = skb->len;
	payload = skb->data;
	data_send_size = roundup(packet_len, BTSDIO_TX_ALIGN);
	alignment_required = (data_send_size != packet_len);

	if(alignment_required)
	{
		tmpbuf = kzalloc(data_send_size, GFP_KERNEL);
		if (!tmpbuf){
			BT_ERR("[bt sdio] TX allocation failed");
			return -ENOMEM;
		}
		memcpy(tmpbuf, payload, data_send_size);
	}
	else
	{
		tmpbuf = payload;
	}

	pm_runtime_get_sync(&sdiodev->func->dev);

	sdio_claim_host(sdiodev->func);
	do {
		/* Transfer data to device */
		ret = sdio_writesb(sdiodev->func,\
				sdiodev->reg_map->sdio_rt_data, tmpbuf,\
				data_send_size);
		if (ret < 0) {
			i++;
			BT_ERR("[bt sdio] TX  i=%d writesb failed: %d", i, ret);
			BT_ERR("[bt sdio] TX data_send_size: %d hex: %*ph",\
					data_send_size, data_send_size,tmpbuf);
			ret = -EIO;
			if (i > MAX_SDIO_TX_RETRY)
				goto exit;
		}
		BT_INFO("[bt sdio] TX to SDIO sdiodev done : %*ph ",\
				data_send_size, tmpbuf);

	} while (ret);

exit:
	sdio_release_host(sdiodev->func);

	if(alignment_required){
		kfree(tmpbuf);
	}

	pm_runtime_mark_last_busy(&sdiodev->func->dev);
	pm_runtime_put_autosuspend(&sdiodev->func->dev);

	if(ret){
		private_data->btti_dev.hcidev->stat.err_tx++;
	}
	else{
		private_data->btti_dev.hcidev->stat.byte_tx\
			+= data_send_size;
	}
	kfree_skb(skb);
	return ret;
}


static int btti_sdio_if_probe(struct sdio_func *func,
					const struct sdio_device_id *id)
{
	int ret = 0;
	struct btti_private *private_data = NULL;
	struct btti_sdio_dev *sdiodev = NULL;

	BT_INFO("[bt sdio] PROBE vendor=0x%x, device=0x%x,"\
			" class=%d, fn=%d 0x%lx",
			id->vendor, id->device, id->class, func->num,\
			( unsigned long)func);

	/* We are only able to handle the wlan function */
	if (func->num != 0x01)
	{
		BT_DBG("[bt sdio] PROBE incorrect function number!");
		ret = -ENODEV;
		return ret;
	}

	/* Device tree node parsing */
	if(btti_sdio_if_probe_of(&func->dev)){
		ret = -ENODEV;
		return ret;
	}

	sdiodev = devm_kzalloc(&func->dev, sizeof(*sdiodev), GFP_KERNEL);
	if (!sdiodev){
		ret = -ENODEV;
		return ret;
	}

	sdiodev->func = func;
	sdiodev->private_data = NULL;

	if (id->driver_data) {
		struct btti_sdio_device *data = (void *) id->driver_data;
		sdiodev->reg_map = data->reg_map;
	}

	if (btti_sdio_register_dev(sdiodev) < 0) {
		BT_ERR("[bt sdio] PROBE Failed to register BT device!");
		ret = -ENODEV;
		goto remove_device;
	}
	//enable interrupts
	btti_sdio_enable_int(sdiodev);

	private_data = btti_hci_add_sdio_dev(sdiodev);
	if (!private_data) {
		BT_ERR("[bt sdio] PROBE Initializing sdiodev failed!");
		ret = -ENODEV;
		goto remove_device;
	}

	sdiodev->private_data = private_data;

	/* Initialize the interface specific function pointers */
	private_data->card_tx_packet_funcp = btti_sdio_tx_packet;
	private_data->card_power_up_firmware_funcp = btti_sdio_power_up_fw;
	private_data->card_power_dn_firmware_funcp = btti_sdio_power_dn_fw;
	private_data->card_process_rx_funcp = btti_sdio_process_rx;


	/* pm_runtime_enable would be done after the firmware is being
	 * downloaded because the core layer probably already enables
	 * runtime PM for this func such as the case host->caps &
	 * MMC_CAP_POWER_OFF_CARD.
	 */
	if (pm_runtime_enabled(&sdiodev->func->dev))
		pm_runtime_disable(&sdiodev->func->dev);

	/* As explaination in drivers/mmc/core/sdio_bus.c tells us:
	 * Unbound SDIO functions are always suspended.
	 * During probe, the function is set active and the usage count
	 * is incremented.  If the driver supports runtime PM,
	 * it should call pm_runtime_put_noidle() in its probe routine and
	 * pm_runtime_get_noresume() in its remove routine.
	 *
	 * So, put a pm_runtime_put_noidle here !
	 */
	pm_runtime_put_noidle(&sdiodev->func->dev);

	BT_DBG("[bt sdio] PROBE sdio_claim_irq");
	sdio_claim_host(func);
	ret = sdio_claim_irq(func, btti_sdio_irq_handler);
	sdio_release_host(func);

	if (ret) {
		BT_ERR("[bt sdio] PROBE sdio_claim_irq failed: ret=%d", ret);
		ret = -EIO;
		goto remove_device;
	}


	BT_INFO("[bt sdio] TI cc33xx BLE-over-SDIO driver is up and running!");

	return 0;

/*disable_sdio_dev_int:
	btti_sdio_disable_int(sdiodev);
	btti_sdio_unregister_dev(func);*/
remove_device:
	btti_sdio_if_remove(func);
	return ret;
}

static void btti_sdio_if_remove(struct sdio_func *func)
{
	struct btti_sdio_dev *sdiodev;
	BT_INFO("[bt sdio if] sdio remove");

	if (func != NULL) {
		sdiodev = sdio_get_drvdata(func);
		if (sdiodev != NULL) {
			BT_INFO("[bt sdio] disable interrupt");
			btti_sdio_disable_int(sdiodev);
			if(sdiodev->private_data != NULL){
				sdiodev->private_data->sdio_dev_removed = true;
				btti_hci_remove_sdio_dev(sdiodev->private_data);
			}
			else {
				BT_ERR("[bt sdio] private_data was "\
						"not allocated");
			}
		}
		btti_sdio_unregister_dev(func);
	}
	/* Be consistent the state in btti_sdio_if_probe */
	pm_runtime_get_noresume(&func->dev);

}

static int btti_sdio_if_suspend(struct device *dev)
{
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct btti_sdio_dev *sdiodev;
	struct btti_private *private_data;
	mmc_pm_flag_t pm_flags;
	struct hci_dev *hcidev;

	BT_INFO("[bt sdio] suspend");
	if (func) {
		pm_flags = sdio_get_host_pm_caps(func);
		BT_INFO("[bt sdio] %s: suspend: PM flags = 0x%x",\
				sdio_func_id(func),
		       pm_flags);
		if (!(pm_flags & MMC_PM_KEEP_POWER)) {
			BT_ERR("[bt sdio] %s: cannot remain"\
					" alive while suspended",
			       sdio_func_id(func));
			return -ENOSYS;
		}
		//sdio_set_host_pm_flags(func, MMC_PM_KEEP_POWER);

		sdiodev = sdio_get_drvdata(func);
		if (!sdiodev || !sdiodev->private_data) {
			BT_ERR("[bt sdio] sdiodev or private_data"\
					" structure is not valid");
			return 0;
		}
	} else {
		BT_ERR("[bt sdio] sdio_func is not specified");
		return 0;
	}

	private_data = sdiodev->private_data;
	private_data->hci_adapter->is_suspending = true;
	hcidev = private_data->btti_dev.hcidev;
	BT_DBG("[bt sdio] %s: SDIO suspend", hcidev->name);
	hci_suspend_dev(hcidev);
	private_data->hci_adapter->is_suspending = false;
	private_data->hci_adapter->is_suspended = true;

	//if(private_data->card_power_dn_firmware_funcp){
	//	private_data->card_power_dn_firmware_funcp(private_data);
	//}

	BT_DBG("[bt sdio] suspend end");
	return 0;
}

static int btti_sdio_if_resume(struct device *dev)
{
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct btti_sdio_dev *sdiodev;
	struct btti_private *private_data;
	mmc_pm_flag_t pm_flags;
	struct hci_dev *hcidev;

	BT_INFO("[bt sdio] resume");

	if (func) {
		pm_flags = sdio_get_host_pm_caps(func);
		BT_INFO("[bt sdio] %s: resume: PM flags = 0x%x",\
				sdio_func_id(func),
		       pm_flags);
		sdiodev = sdio_get_drvdata(func);
		if (!sdiodev || !sdiodev->private_data) {
			BT_ERR("[bt sdio] sdiodev or private_data"\
					" structure is not valid");
			return 0;
		}
	} else {
		BT_ERR("[bt sdio] sdio_func is not specified");
		return 0;
	}
	private_data = sdiodev->private_data;
	if(!private_data)
	{
		BT_ERR("[bt sdio] private_data is not allocated");
		return 0;
	}

	if (!private_data->hci_adapter->is_suspended) {
		BT_WARN("[bt sdio] device already resumed");
		return 0;
	}

	//if(private_data->card_power_up_firmware_funcp){
	//	private_data->card_power_up_firmware_funcp(private_data);
	//}
	hcidev = private_data->btti_dev.hcidev;
	private_data->hci_adapter->is_suspended = false;
	BT_INFO("[bt sdio] %s: SDIO resume", hcidev->name);
	hci_resume_dev(hcidev);

	return 0;
}

static const struct dev_pm_ops btti_sdio_pm_ops = {
	.suspend	= btti_sdio_if_suspend,
	.resume		= btti_sdio_if_resume,
};

static struct sdio_driver bt_ti_sdio = {
	.name		= "btti_cc33xx_bt_sdio",
	.id_table	= btti_sdio_ids,
	.probe		= btti_sdio_if_probe,
	.remove		= btti_sdio_if_remove,
	.drv = {
		.owner = THIS_MODULE,
		.coredump = btti_sdio_coredump,
		.pm = &btti_sdio_pm_ops,
	}
};

static int __init btti_sdio_init_module(void)
{
	BT_INFO("[bt sdio] BLE SDIO init module");
	if (sdio_register_driver(&bt_ti_sdio) != 0) {
		BT_ERR("[bt sdio] SDIO Driver Registration Failed");
		return -ENODEV;
	}
	return 0;
}

static void __exit btti_sdio_exit_module(void)
{
	BT_INFO("[bt sdio] BLE SDIO exit module");
	sdio_unregister_driver(&bt_ti_sdio);
}

module_init(btti_sdio_init_module);
module_exit(btti_sdio_exit_module);

MODULE_AUTHOR("Texas Instruments.");
MODULE_DESCRIPTION("Texas Instruments BT-over-SDIO driver ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL v2");
