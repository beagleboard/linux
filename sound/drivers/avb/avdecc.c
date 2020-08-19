#include "avdecc.h"

bool avb_avdecc_init(struct avdecc *avdecc)
{
	avb_log(AVB_KERN_INFO, KERN_INFO "avb_avdecc_init");

	avdecc->sd.type = ETH_P_TSN;
	avdecc->sd.destmac[0] = 0x91;
	avdecc->sd.destmac[1] = 0xe0;
	avdecc->sd.destmac[2] = 0xf0;
	avdecc->sd.destmac[3] = 0x01;
	avdecc->sd.destmac[4] = 0x00;
	avdecc->sd.destmac[5] = 0x00;

	avdecc->acmp_tx_state = AVB_ACMP_STATUS_NOT_CONNECTED;
	avdecc->acmp_rx_state = AVB_ACMP_STATUS_NOT_CONNECTED;

	return avb_socket_init(&avdecc->sd, 1000);
}

static void avb_acdecc_init_and_fill_eth_hdr(struct avdecc *avdecc,
					     u8 multicast)
{
	struct ethhdr *eh = (struct ethhdr *)&avdecc->sd.tx_buf[0];
	struct ethhdr *reh = (struct ethhdr *)&avdecc->sd.rx_buf[0];

	/* Initialize it */
	memset(avdecc->sd.tx_buf, 0, AVB_MAX_ETH_FRAME_SIZE);

	/* Fill in the Ethernet header */
	if (multicast == 0) {
		eh->h_dest[0] = reh->h_source[0];
		eh->h_dest[1] = reh->h_source[1];
		eh->h_dest[2] = reh->h_source[2];
		eh->h_dest[3] = reh->h_source[3];
		eh->h_dest[4] = reh->h_source[4];
		eh->h_dest[5] = reh->h_source[5];
	} else {
		eh->h_dest[0] = 0x91;
		eh->h_dest[1] = 0xe0;
		eh->h_dest[2] = 0xf0;
		eh->h_dest[3] = 0x01;
		eh->h_dest[4] = 0x00;
		eh->h_dest[5] = 0x00;
	}
	eh->h_source[0] = avdecc->sd.srcmac[0];
	eh->h_source[1] = avdecc->sd.srcmac[1];
	eh->h_source[2] = avdecc->sd.srcmac[2];
	eh->h_source[3] = avdecc->sd.srcmac[3];
	eh->h_source[4] = avdecc->sd.srcmac[4];
	eh->h_source[5] = avdecc->sd.srcmac[5];

	/* Fill in Ethertype field */
	eh->h_proto = htons(avdecc->sd.type);
}

static void avb_acdecc_fill_AVTP_ctrl_hdr(struct avdecc *avdecc, u8 sub_type,
					  u8 msg_type, u8 status, u16 data_len,
					  int stream_id)
{
	struct avt_pdu_control_hdr *hdr =
		(struct avt_pdu_control_hdr *)&avdecc->sd
			.tx_buf[sizeof(struct ethhdr)];

	hdr->h.f.sub_type = sub_type;
	AVB_AVTPDU_CTRL_HDR_SET_SV(hdr, 0);
	AVB_AVTPDU_CTRL_HDR_SET_VER(hdr, 0);
	AVB_AVTPDU_CTRL_HDR_SET_MSGTYPE(hdr, msg_type);
	AVB_AVTPDU_CTRL_HDR_SET_VALIDTIME(hdr, status);
	AVB_AVTPDU_CTRL_HDR_SET_DATALEN(hdr, data_len);
	if (stream_id == 0) {
		hdr->h.f.stream_id[0] =
			avdecc->sd.srcmac[0]; //avdecc->sd.srcmac[0];
		hdr->h.f.stream_id[1] =
			avdecc->sd.srcmac[1]; //avdecc->sd.srcmac[1];
		hdr->h.f.stream_id[2] =
			avdecc->sd.srcmac[2]; //avdecc->sd.srcmac[2];
		hdr->h.f.stream_id[3] = avdecc->sd.srcmac[3]; //0xff;
		hdr->h.f.stream_id[4] = avdecc->sd.srcmac[4]; //0xfe;
		hdr->h.f.stream_id[5] =
			avdecc->sd.srcmac[5]; //avdecc->sd.srcmac[3];
		hdr->h.f.stream_id[6] = 0; //avdecc->sd.srcmac[4];
		hdr->h.f.stream_id[7] = 1; //avdecc->sd.srcmac[5];
	} else if (stream_id > 0) {
		hdr->h.f.stream_id[0] = avdecc->sd.srcmac[0];
		hdr->h.f.stream_id[1] = avdecc->sd.srcmac[1];
		hdr->h.f.stream_id[2] = avdecc->sd.srcmac[2];
		hdr->h.f.stream_id[3] = 0xff;
		hdr->h.f.stream_id[4] = 0xfe;
		hdr->h.f.stream_id[5] = avdecc->sd.srcmac[3];
		hdr->h.f.stream_id[6] = avdecc->sd.srcmac[4];
		hdr->h.f.stream_id[7] = avdecc->sd.srcmac[5];
	} else {
		hdr->h.f.stream_id[0] = 0;
		hdr->h.f.stream_id[1] = 0;
		hdr->h.f.stream_id[2] = 0;
		hdr->h.f.stream_id[3] = 0;
		hdr->h.f.stream_id[4] = 0;
		hdr->h.f.stream_id[5] = 0;
		hdr->h.f.stream_id[6] = 0;
		hdr->h.f.stream_id[7] = 0;
	}
}

void avb_maap_announce(struct avdecc *avdecc)
{
	int tx_size = 0;
	int err = 0;

	struct maap_pdu *pdu =
		(struct maap_pdu *)&avdecc->sd.tx_buf[sizeof(struct ethhdr)];

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_maap_announce");

	avb_acdecc_init_and_fill_eth_hdr(avdecc, 1);
	avb_acdecc_fill_AVTP_ctrl_hdr(avdecc, AVB_AVTP_SUBTYPE_MAAP, 3, 1, 16,
				      -1);

	pdu->req_MAC[0] = 0x91;
	pdu->req_MAC[1] = 0xe0;
	pdu->req_MAC[2] = 0xf0;
	pdu->req_MAC[3] = 0x00;
	pdu->req_MAC[4] = 0x33;
	pdu->req_MAC[5] = 0x4b;

	pdu->req_count = 2;

	tx_size = sizeof(struct ethhdr) + sizeof(struct maap_pdu);

	avdecc->sd.tx_iov.iov_base = avdecc->sd.tx_buf;
	avdecc->sd.tx_iov.iov_len = tx_size;
	iov_iter_init(&avdecc->sd.tx_msg_hdr.msg_iter, WRITE,
		      &avdecc->sd.tx_iov, 1, tx_size);

	if ((err = sock_sendmsg(avdecc->sd.sock, &avdecc->sd.tx_msg_hdr)) <=
	    0) {
		avb_log(AVB_KERN_WARN,
			KERN_WARNING
			"avb_maap_announce Socket transmission fails %d \n",
			err);
		return;
	}
}

void avb_adp_discover(struct avdecc *avdecc)
{
	int tx_size = 0;
	int err = 0;

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_adp_discover");

	avb_acdecc_init_and_fill_eth_hdr(avdecc, 1);
	avb_acdecc_fill_AVTP_ctrl_hdr(avdecc, AVB_AVTP_SUBTYPE_ADP,
				      AVB_ADP_MSGTYPE_ENTITY_DISCOVER, 31,
				      AVB_ADP_CONTROL_DATA_LENGTH, 1);

	tx_size = sizeof(struct ethhdr) + sizeof(struct avt_pdu_control_hdr) +
		  AVB_ADP_CONTROL_DATA_LENGTH;

	avdecc->sd.tx_iov.iov_base = avdecc->sd.tx_buf;
	avdecc->sd.tx_iov.iov_len = tx_size;
	iov_iter_init(&avdecc->sd.tx_msg_hdr.msg_iter, WRITE,
		      &avdecc->sd.tx_iov, 1, tx_size);

	if ((err = sock_sendmsg(avdecc->sd.sock, &avdecc->sd.tx_msg_hdr)) <=
	    0) {
		avb_log(AVB_KERN_WARN,
			KERN_WARNING
			"avb_adp_discover Socket transmission fails %d \n",
			err);
		return;
	}
}

void avb_adp_advertise(struct avdecc *avdecc)
{
	int tx_size = 0;
	int err = 0;

	struct adpdu *adpdu =
		(struct adpdu *)&avdecc->sd
			.tx_buf[sizeof(struct ethhdr) +
				sizeof(struct avt_pdu_control_hdr)];

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_adp_advertise");

	avb_acdecc_init_and_fill_eth_hdr(avdecc, 1);
	avb_acdecc_fill_AVTP_ctrl_hdr(avdecc, AVB_AVTP_SUBTYPE_ADP,
				      AVB_ADP_MSGTYPE_ENTITY_AVAILABLE, 31,
				      AVB_ADP_CONTROL_DATA_LENGTH, 1);

	adpdu->entity_model_id[0] = avdecc->sd.srcmac[0];
	adpdu->entity_model_id[1] = avdecc->sd.srcmac[1];
	adpdu->entity_model_id[2] = avdecc->sd.srcmac[2];
	adpdu->entity_model_id[3] = avdecc->sd.srcmac[3];
	adpdu->entity_model_id[4] = avdecc->sd.srcmac[4];
	adpdu->entity_model_id[5] = avdecc->sd.srcmac[5];
	adpdu->entity_model_id[6] = 0x00;
	adpdu->entity_model_id[7] = 0x01;
	adpdu->entity_caps = htonl(0x00008508);
	adpdu->talker_stream_sources = htons(1);
	adpdu->talker_caps = htons(0x4001);
	adpdu->listener_stream_sinks = htons(1);
	adpdu->listener_caps = htons(0x4001);
	adpdu->control_caps = 0;
	adpdu->avai_idx = htonl(avdecc->adp_avai_idx++);
	adpdu->gptp_grand_master_id[0] =
		/* 0x8d; */ 0xa8; //avdecc->sd.srcmac[0];
	adpdu->gptp_grand_master_id[1] =
		/* 0x85; */ 0x60; //avdecc->sd.srcmac[1];
	adpdu->gptp_grand_master_id[2] =
		/* 0x90; */ 0xb6; //avdecc->sd.srcmac[2];
	adpdu->gptp_grand_master_id[3] = /* 0x2c; */ 0xFF;
	adpdu->gptp_grand_master_id[4] = /* 0xdf; */ 0xFE;
	adpdu->gptp_grand_master_id[5] =
		/* 0xe9; */ 0x14; //avdecc->sd.srcmac[3];
	adpdu->gptp_grand_master_id[6] =
		/* 0x00; */ 0xa6; //avdecc->sd.srcmac[4];
	adpdu->gptp_grand_master_id[7] =
		/* 0x00; */ 0x3f; //avdecc->sd.srcmac[5];
	adpdu->gptp_domain_number = 0;
	adpdu->iden_ctrl_idx = 0;
	adpdu->interface_idx = 0;

	tx_size = sizeof(struct ethhdr) + sizeof(struct avt_pdu_control_hdr) +
		  AVB_ADP_CONTROL_DATA_LENGTH;

	avdecc->sd.tx_iov.iov_base = avdecc->sd.tx_buf;
	avdecc->sd.tx_iov.iov_len = tx_size;
	iov_iter_init(&avdecc->sd.tx_msg_hdr.msg_iter, WRITE,
		      &avdecc->sd.tx_iov, 1, tx_size);

	if ((err = sock_sendmsg(avdecc->sd.sock, &avdecc->sd.tx_msg_hdr)) <=
	    0) {
		avb_log(AVB_KERN_WARN,
			KERN_WARNING
			"avb_adp_advertise Socket transmission fails %d \n",
			err);
		return;
	}
}

static int avb_avdecc_listen(struct avdecc *avdecc)
{
	int err = 0;
	mm_segment_t oldfs;
	struct kvec vec;

	memset(avdecc->sd.rx_buf, 0, AVB_MAX_ETH_FRAME_SIZE);
	avdecc->sd.rx_iov.iov_base = avdecc->sd.rx_buf;
	avdecc->sd.rx_iov.iov_len = AVB_MAX_ETH_FRAME_SIZE;
	iov_iter_init(&avdecc->sd.rx_msg_hdr.msg_iter, READ, &avdecc->sd.rx_iov,
		      1, AVB_MAX_ETH_FRAME_SIZE);

	vec.iov_base = avdecc->sd.rx_buf;
	vec.iov_len = AVB_MAX_ETH_FRAME_SIZE;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	err = kernel_recvmsg(avdecc->sd.sock, &avdecc->sd.rx_msg_hdr, &vec, 1,
			     AVB_MAX_ETH_FRAME_SIZE, MSG_DONTWAIT);
	set_fs(oldfs);

	if (err <= 0)
		if (err != -11)
			avb_log(AVB_KERN_WARN,
				KERN_WARNING
				"avb_adecc_listen Socket reception res %d \n",
				err);

	return err;
}

static void avb_avdecc_aecp_respond_to_AEM_cmd(struct avdecc *avdecc,
					       struct msrp *msrp)
{
	int i = 0;
	int err = 0;
	u16 tx_size = 0;
	int descp_size = 0;
	int max_cfg_idx = 0;
	int max_desc_idx = 0;
	struct aem_cmd *cmd =
		(struct aem_cmd *)&avdecc->sd
			.rx_buf[sizeof(struct ethhdr) +
				sizeof(struct avt_pdu_control_hdr)];

	struct read_descp_cmd *rd_cmd = (struct read_descp_cmd *)cmd;
	struct get_stream_info_cmd *gt_str_info_cmd =
		(struct get_stream_info_cmd *)cmd;
	struct read_descp_res *rd_res =
		(struct read_descp_res *)&avdecc->sd
			.tx_buf[sizeof(struct ethhdr) +
				sizeof(struct avt_pdu_control_hdr)];

	avb_acdecc_init_and_fill_eth_hdr(avdecc, 0);

	memcpy(&rd_res->hdr.ctrl_entity_id[0], &rd_cmd->hdr.ctrl_entity_id[0],
	       8);
	rd_res->hdr.seq_id = rd_cmd->hdr.seq_id;
	rd_res->hdr.cmd_type = cmd->cmd_type;

	switch (htons(cmd->cmd_type)) {
	case AVB_AEM_CMD_READ_DESCP: {
		rd_res->cfg_idx = rd_cmd->cfg_idx;
		rd_res->res = 0;

		switch (htons(rd_cmd->desc_type)) {
		case AVB_AEM_DESCP_ENTITY: {
			struct entity_descp *ent_descp =
				(struct entity_descp *)&avdecc->sd.tx_buf
					[sizeof(struct ethhdr) +
					 sizeof(struct avt_pdu_control_hdr) +
					 sizeof(struct read_descp_res)];

			avb_log(AVB_KERN_INFO, KERN_INFO
				"avb_aecp_readResponse for Entity Descriptor");

			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_SUCCESS,
				(sizeof(struct read_descp_res) +
				 sizeof(struct entity_descp)),
				1);
			ent_descp->desc_type = htons(AVB_AEM_DESCP_ENTITY);
			ent_descp->desc_idx = 0;
			ent_descp->entity_id[0] = avdecc->sd.srcmac[0];
			ent_descp->entity_id[1] = avdecc->sd.srcmac[1];
			ent_descp->entity_id[2] = avdecc->sd.srcmac[2];
			ent_descp->entity_id[3] = 0xff;
			ent_descp->entity_id[4] = 0xfe;
			ent_descp->entity_id[5] = avdecc->sd.srcmac[3];
			ent_descp->entity_id[6] = avdecc->sd.srcmac[4];
			ent_descp->entity_id[7] = avdecc->sd.srcmac[5];
			ent_descp->entity_model_id[0] = avdecc->sd.srcmac[0];
			ent_descp->entity_model_id[1] = avdecc->sd.srcmac[1];
			ent_descp->entity_model_id[2] = avdecc->sd.srcmac[2];
			ent_descp->entity_model_id[3] = avdecc->sd.srcmac[3];
			ent_descp->entity_model_id[4] = avdecc->sd.srcmac[4];
			ent_descp->entity_model_id[5] = avdecc->sd.srcmac[5];
			ent_descp->entity_model_id[6] = 0x00;
			ent_descp->entity_model_id[7] = 0x01;
			ent_descp->entity_caps = htonl(0x00008508);
			ent_descp->talker_stream_sources = htons(1);
			ent_descp->talker_caps = htons(0x4001);
			ent_descp->listener_stream_sinks = htons(1);
			ent_descp->listener_caps = htons(0x4001);
			ent_descp->control_caps = 0;
			ent_descp->avai_idx = htonl(avdecc->adp_avai_idx);
			strcpy(&ent_descp->entity_name[0], "ALSA_AVB_Driver");
			strcpy(&ent_descp->firmware_ver[0], "0.1");
			strcpy(&ent_descp->group_name[0], "0");
			strcpy(&ent_descp->serial_number[0], "0");
			ent_descp->vendor_name_string = htons(0x0000);
			ent_descp->model_name_string = htons(0x0001);
			ent_descp->cfg_count = htons(1);
			ent_descp->curr_cfg = htons(0);
			descp_size = sizeof(struct entity_descp);
			break;
		}
		case AVB_AEM_DESCP_CONFIGURATION: {
			struct config_descp *cfg_descp =
				(struct config_descp *)&avdecc->sd.tx_buf
					[sizeof(struct ethhdr) +
					 sizeof(struct avt_pdu_control_hdr) +
					 sizeof(struct read_descp_res)];

			avb_log(AVB_KERN_INFO, KERN_INFO
				"avb_aecp_readResponse for Configuration Descriptor");

			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_SUCCESS,
				(sizeof(struct read_descp_res) +
				 sizeof(struct config_descp)),
				1);
			cfg_descp->desc_type =
				htons(AVB_AEM_DESCP_CONFIGURATION);
			cfg_descp->desc_idx = 0;
			cfg_descp->localized_descp = htons(0xffff);
			cfg_descp->descp_count = htons(AVB_AEM_MAX_DESCP_COUNT);
			cfg_descp->descpOff = htons(74);

			cfg_descp->descps[0].desc_type =
				htons(AVB_AEM_DESCP_AUDIO_UNIT);
			cfg_descp->descps[0].desc_count = htons(1);
			cfg_descp->descps[1].desc_type =
				htons(AVB_AEM_DESCP_STREAM_IP);
			cfg_descp->descps[1].desc_count = htons(1);
			cfg_descp->descps[2].desc_type =
				htons(AVB_AEM_DESCP_STREAM_OP);
			cfg_descp->descps[2].desc_count = htons(1);
			cfg_descp->descps[3].desc_type =
				htons(AVB_AEM_DESCP_AVBINTERFACE);
			cfg_descp->descps[3].desc_count = htons(1);
			cfg_descp->descps[4].desc_type =
				htons(AVB_AEM_DESCP_CLOCKSOURCE);
			cfg_descp->descps[4].desc_count = htons(2);
			cfg_descp->descps[5].desc_type =
				htons(AVB_AEM_DESCP_LOCALE);
			cfg_descp->descps[5].desc_count = htons(1);
			cfg_descp->descps[6].desc_type =
				htons(AVB_AEM_DESCP_CLOCK_DOMAIN);
			cfg_descp->descps[6].desc_count = htons(1);
			descp_size = sizeof(struct config_descp);
			break;
		}
		case AVB_AEM_DESCP_AUDIO_UNIT: {
			struct audio_unit_descp *au_descp =
				(struct audio_unit_descp *)&avdecc->sd.tx_buf
					[sizeof(struct ethhdr) +
					 sizeof(struct avt_pdu_control_hdr) +
					 sizeof(struct read_descp_res)];

			avb_log(AVB_KERN_INFO, KERN_INFO
				"avb_aecp_readResponse for Audiounit Descriptor");

			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_SUCCESS,
				(sizeof(struct read_descp_res) +
				 sizeof(struct audio_unit_descp)),
				1);
			au_descp->desc_type = htons(AVB_AEM_DESCP_AUDIO_UNIT);
			au_descp->desc_idx = 0;
			au_descp->localized_descp = htons(0x0001);

			au_descp->num_stream_ip = htons(1);
			au_descp->num_stream_op = htons(1);
			au_descp->num_ext_ip = htons(8);
			au_descp->num_ext_op = htons(8);
			au_descp->current_sampling_rate = htonl(48000);
			au_descp->sampling_rates_offset = htons(144);
			au_descp->sampling_rates_count = htons(6);
			au_descp->sampling_rates[0] = htonl(44100);
			au_descp->sampling_rates[1] = htonl(48000);
			au_descp->sampling_rates[2] = htonl(88200);
			au_descp->sampling_rates[3] = htonl(96000);
			au_descp->sampling_rates[4] = htonl(176400);
			au_descp->sampling_rates[5] = htonl(192000);
			descp_size = sizeof(struct audio_unit_descp);
			break;
		}
		case AVB_AEM_DESCP_STREAM_PORT_IP:
		case AVB_AEM_DESCP_STREAM_PORT_OP: {
			struct stream_port_descp *st_port_descp =
				(struct stream_port_descp *)&avdecc->sd.tx_buf
					[sizeof(struct ethhdr) +
					 sizeof(struct avt_pdu_control_hdr) +
					 sizeof(struct read_descp_res)];

			if (htons(rd_cmd->desc_type) ==
			    AVB_AEM_DESCP_STREAM_PORT_IP)
				avb_log(AVB_KERN_INFO,
					KERN_INFO
					"avb_aecp_readResponse for Stream Input Port (%d) Descriptor",
					htons(rd_cmd->desc_idx));
			else
				avb_log(AVB_KERN_INFO,
					KERN_INFO
					"avb_aecp_readResponse for Stream Output Port (%d) Descriptor",
					htons(rd_cmd->desc_idx));

			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_SUCCESS,
				(sizeof(struct read_descp_res) +
				 sizeof(struct stream_port_descp)),
				1);
			st_port_descp->desc_type = rd_cmd->desc_type;
			st_port_descp->desc_idx = rd_cmd->desc_idx;

			st_port_descp->num_clusters = htons(8);
			st_port_descp->num_maps = htons(1);
			st_port_descp->port_flags =
				htons(((htons(rd_cmd->desc_type) ==
					AVB_AEM_DESCP_STREAM_PORT_IP) ?
					       (0) :
					       (1)));
			st_port_descp->base_cluster =
				htons(((htons(rd_cmd->desc_type) ==
					AVB_AEM_DESCP_STREAM_PORT_IP) ?
					       (0) :
					       (8)));
			st_port_descp->base_map =
				htons(((htons(rd_cmd->desc_type) ==
					AVB_AEM_DESCP_STREAM_PORT_IP) ?
					       (0) :
					       (1)));
			descp_size = sizeof(struct stream_port_descp);
			break;
		}
		case AVB_AEM_DESCP_EXT_PORT_IP:
		case AVB_AEM_DESCP_EXT_PORT_OP: {
			struct ext_port_descp *ext_port_descp =
				(struct ext_port_descp *)&avdecc->sd.tx_buf
					[sizeof(struct ethhdr) +
					 sizeof(struct avt_pdu_control_hdr) +
					 sizeof(struct read_descp_res)];

			if (htons(rd_cmd->desc_type) ==
			    AVB_AEM_DESCP_EXT_PORT_IP)
				avb_log(AVB_KERN_INFO,
					KERN_INFO
					"avb_aecp_readResponse for External Input Port (%d) Descriptor",
					htons(rd_cmd->desc_idx));
			else
				avb_log(AVB_KERN_INFO,
					KERN_INFO
					"avb_aecp_readResponse for External Output Port (%d) Descriptor",
					htons(rd_cmd->desc_idx));

			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_SUCCESS,
				(sizeof(struct read_descp_res) +
				 sizeof(struct ext_port_descp)),
				1);
			ext_port_descp->desc_type = rd_cmd->desc_type;
			ext_port_descp->desc_idx = rd_cmd->desc_idx;

			ext_port_descp->signal_type =
				htons(((htons(rd_cmd->desc_type) ==
					AVB_AEM_DESCP_EXT_PORT_IP) ?
					       (AVB_AEM_DESCP_INVALID) :
					       (AVB_AEM_DESCP_AUDIO_CLUSTER)));
			ext_port_descp->signal_idx =
				htons(((htons(rd_cmd->desc_type) ==
					AVB_AEM_DESCP_EXT_PORT_IP) ?
					       (0) :
					       (htons(rd_cmd->desc_idx))));
			descp_size = sizeof(struct ext_port_descp);
			max_desc_idx = 7;
			break;
		}
		case AVB_AEM_DESCP_JACK_IP:
		case AVB_AEM_DESCP_JACK_OP: {
			struct jack_descp *jack_descp =
				(struct jack_descp *)&avdecc->sd.tx_buf
					[sizeof(struct ethhdr) +
					 sizeof(struct avt_pdu_control_hdr) +
					 sizeof(struct read_descp_res)];

			if (htons(rd_cmd->desc_type) == AVB_AEM_DESCP_JACK_IP)
				avb_log(AVB_KERN_INFO,
					KERN_INFO
					"avb_aecp_readResponse for Jack Input Port (%d) Descriptor",
					htons(rd_cmd->desc_idx));
			else
				avb_log(AVB_KERN_INFO,
					KERN_INFO
					"avb_aecp_readResponse for Jack Output Port (%d) Descriptor",
					htons(rd_cmd->desc_idx));

			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_SUCCESS,
				(sizeof(struct read_descp_res) +
				 sizeof(struct jack_descp)),
				1);
			jack_descp->desc_type = rd_cmd->desc_type;
			jack_descp->desc_idx = rd_cmd->desc_idx;
			if (htons(rd_cmd->desc_type) == AVB_AEM_DESCP_JACK_IP)
				strcpy(&jack_descp->obj_name[0], "ip-");
			else
				strcpy(&jack_descp->obj_name[0], "op-");

			jack_descp->obj_name[3] = 48 + htons(rd_cmd->desc_idx);
			jack_descp->localized_descp = htons(0x0007);

			jack_descp->jack_type = htons(8); /* Balanced analog */
			descp_size = sizeof(struct jack_descp);
			max_desc_idx = 7;
			break;
		}
		case AVB_AEM_DESCP_AUDIO_CLUSTER: {
			struct audio_cluster_descp *au_cl_descp =
				(struct audio_cluster_descp *)&avdecc->sd.tx_buf
					[sizeof(struct ethhdr) +
					 sizeof(struct avt_pdu_control_hdr) +
					 sizeof(struct read_descp_res)];
			avb_log(AVB_KERN_INFO,
				KERN_INFO
				"avb_aecp_readResponse for Audio Cluster (%d) Descriptor",
				htons(rd_cmd->desc_idx));

			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_SUCCESS,
				(sizeof(struct read_descp_res) +
				 sizeof(struct audio_cluster_descp)),
				1);
			au_cl_descp->desc_type = rd_cmd->desc_type;
			au_cl_descp->desc_idx = rd_cmd->desc_idx;

			au_cl_descp->localized_descp = htons(0xffff);

			au_cl_descp->signal_type =
				htons(((htons(rd_cmd->desc_idx) < 8) ?
					       (AVB_AEM_DESCP_INVALID) :
					       (AVB_AEM_DESCP_EXT_PORT_IP)));
			au_cl_descp->signal_idx =
				htons(((htons(rd_cmd->desc_idx) < 8) ?
					       (0) :
					       (htons(rd_cmd->desc_idx) - 8)));
			au_cl_descp->num_channels = htons(1);
			au_cl_descp->format = 0x40;
			descp_size = sizeof(struct audio_cluster_descp);
			max_desc_idx = 15;
			break;
		}
		case AVB_AEM_DESCP_AUDIO_MAP: {
			struct audio_map_descp *au_map_descp =
				(struct audio_map_descp *)&avdecc->sd.tx_buf
					[sizeof(struct ethhdr) +
					 sizeof(struct avt_pdu_control_hdr) +
					 sizeof(struct read_descp_res)];
			avb_log(AVB_KERN_INFO,
				KERN_INFO
				"avb_aecp_readResponse for Audio Map (%d) Descriptor",
				htons(rd_cmd->desc_idx));

			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_SUCCESS,
				(sizeof(struct read_descp_res) +
				 sizeof(struct audio_map_descp)),
				1);
			au_map_descp->desc_type = rd_cmd->desc_type;
			au_map_descp->desc_idx = rd_cmd->desc_idx;

			au_map_descp->mapping_offset = htons(8);
			au_map_descp->num_mappings = htons(8);

			for (i = 0; i < 8; i++) {
				au_map_descp->map[i].stream_idx = htons(0);
				au_map_descp->map[i].stream_channel = htons(i);
				au_map_descp->map[i].cluster_offset = htons(i);
				au_map_descp->map[i].cluster_channel = htons(0);
			}
			descp_size = sizeof(struct audio_map_descp);
			max_desc_idx = 1;
			break;
		}
		case AVB_AEM_DESCP_CLOCKSOURCE: {
			struct clock_source_descp *clk_src_descp =
				(struct clock_source_descp *)&avdecc->sd.tx_buf
					[sizeof(struct ethhdr) +
					 sizeof(struct avt_pdu_control_hdr) +
					 sizeof(struct read_descp_res)];
			avb_log(AVB_KERN_INFO,
				KERN_INFO
				"avb_aecp_readResponse for Clock Source (%d) Descriptor",
				htons(rd_cmd->desc_idx));

			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_SUCCESS,
				(sizeof(struct read_descp_res) +
				 sizeof(struct clock_source_descp)),
				1);
			clk_src_descp->desc_type = rd_cmd->desc_type;
			clk_src_descp->desc_idx = rd_cmd->desc_idx;

			clk_src_descp->localized_descp =
				htons(0x0004 + htons(rd_cmd->desc_idx));

			clk_src_descp->clock_source_id[0] =
				avdecc->sd.srcmac[0];
			clk_src_descp->clock_source_id[1] =
				avdecc->sd.srcmac[1];
			clk_src_descp->clock_source_id[2] =
				avdecc->sd.srcmac[2];
			clk_src_descp->clock_source_id[3] =
				avdecc->sd.srcmac[3];
			clk_src_descp->clock_source_id[4] =
				avdecc->sd.srcmac[4];
			clk_src_descp->clock_source_id[5] =
				avdecc->sd.srcmac[5];
			clk_src_descp->clock_source_id[6] = 0;
			clk_src_descp->clock_source_id[7] =
				(u8)htons(rd_cmd->desc_idx);

			if (htons(rd_cmd->desc_idx) == 0) {
				clk_src_descp->clock_source_flags = htons(0x00);
				clk_src_descp->clock_source_type = htons(0x00);
				clk_src_descp->clock_source_loc_type =
					htons(AVB_AEM_DESCP_AUDIO_UNIT);
			} else if (htons(rd_cmd->desc_idx) == 1) {
				clk_src_descp->clock_source_flags = htons(0x00);
				clk_src_descp->clock_source_type = htons(0x01);
				clk_src_descp->clock_source_loc_type =
					htons(AVB_AEM_DESCP_JACK_IP);
			} else {
				clk_src_descp->clock_source_flags = htons(0x02);
				clk_src_descp->clock_source_type = htons(0x02);
				clk_src_descp->clock_source_loc_type =
					htons(AVB_AEM_DESCP_STREAM_IP);
			}

			descp_size = sizeof(struct clock_source_descp);
			max_desc_idx = 2;
			break;
		}
		case AVB_AEM_DESCP_CLOCK_DOMAIN: {
			struct clock_domain_descp *clk_do_descp =
				(struct clock_domain_descp *)&avdecc->sd.tx_buf
					[sizeof(struct ethhdr) +
					 sizeof(struct avt_pdu_control_hdr) +
					 sizeof(struct read_descp_res)];
			avb_log(AVB_KERN_INFO,
				KERN_INFO
				"avb_aecp_readResponse for Clock Domain (%d) Descriptor",
				htons(rd_cmd->desc_idx));

			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_SUCCESS,
				(sizeof(struct read_descp_res) +
				 sizeof(struct clock_domain_descp)),
				1);
			clk_do_descp->desc_type = rd_cmd->desc_type;
			clk_do_descp->desc_idx = rd_cmd->desc_idx;

			clk_do_descp->localized_descp = htons(0x0000);

			clk_do_descp->curr_clock_source = htons(0x00);
			clk_do_descp->clock_sources_offset = htons(0x4c);
			clk_do_descp->clock_sources_count = htons(3);
			clk_do_descp->clock_sources[0] = htons(0);
			clk_do_descp->clock_sources[1] = htons(1);
			clk_do_descp->clock_sources[2] = htons(2);

			descp_size = sizeof(struct clock_domain_descp);
			break;
		}
		case AVB_AEM_DESCP_AVBINTERFACE: {
			struct avb_if_descp *avb_if_descp =
				(struct avb_if_descp *)&avdecc->sd.tx_buf
					[sizeof(struct ethhdr) +
					 sizeof(struct avt_pdu_control_hdr) +
					 sizeof(struct read_descp_res)];

			avb_log(AVB_KERN_INFO, KERN_INFO
				"avb_aecp_readResponse for AVB Interface Descriptor");

			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_SUCCESS,
				(sizeof(struct read_descp_res) +
				 sizeof(struct avb_if_descp)),
				1);
			avb_if_descp->desc_type =
				htons(AVB_AEM_DESCP_AVBINTERFACE);
			avb_if_descp->desc_idx = 0;
			avb_if_descp->localized_descp = htons(0xffff);
			avb_if_descp->mac_addr[0] = avdecc->sd.srcmac[0];
			avb_if_descp->mac_addr[1] = avdecc->sd.srcmac[1];
			avb_if_descp->mac_addr[2] = avdecc->sd.srcmac[2];
			avb_if_descp->mac_addr[3] = avdecc->sd.srcmac[3];
			avb_if_descp->mac_addr[4] = avdecc->sd.srcmac[4];
			avb_if_descp->mac_addr[5] = avdecc->sd.srcmac[5];
			avb_if_descp->if_flags = htons(0x0007);
			avb_if_descp->clock_iden[0] = avdecc->sd.srcmac[0];
			avb_if_descp->clock_iden[1] = avdecc->sd.srcmac[1];
			avb_if_descp->clock_iden[2] = avdecc->sd.srcmac[2];
			avb_if_descp->clock_iden[3] = 0xff;
			avb_if_descp->clock_iden[4] = 0xfe;
			avb_if_descp->clock_iden[5] = avdecc->sd.srcmac[3];
			avb_if_descp->clock_iden[6] = avdecc->sd.srcmac[4];
			avb_if_descp->clock_iden[7] = avdecc->sd.srcmac[5];
			avb_if_descp->prio1 = 250;
			avb_if_descp->clock_class = 248;
			avb_if_descp->off_scaled_log_var = htons(0x4100);
			avb_if_descp->clock_accu = 254;
			avb_if_descp->prio2 = 250;
			avb_if_descp->domain_no = 0;
			avb_if_descp->log_sync_int = 15;
			avb_if_descp->log_anno_int = 15;
			avb_if_descp->log_pdelay_int = 13;
			avb_if_descp->port_no = htons(0x0001);
			descp_size = sizeof(struct avb_if_descp);
			break;
		}
		case AVB_AEM_DESCP_LOCALE: {
			struct locale_descp *locale_descp =
				(struct locale_descp *)&avdecc->sd.tx_buf
					[sizeof(struct ethhdr) +
					 sizeof(struct avt_pdu_control_hdr) +
					 sizeof(struct read_descp_res)];

			avb_log(AVB_KERN_INFO, KERN_INFO
				"avb_aecp_readResponse for Locale Descriptor");

			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_SUCCESS,
				(sizeof(struct read_descp_res) +
				 sizeof(struct locale_descp)),
				1);

			locale_descp->desc_type = htons(AVB_AEM_DESCP_LOCALE);
			locale_descp->desc_idx = 0;
			strcpy(&locale_descp->locale_id[0], "en-US");
			locale_descp->num_strings = htons(1);
			locale_descp->base_strings_idx = htons(0);
			descp_size = sizeof(struct locale_descp);
			break;
		}
		case AVB_AEM_DESCP_STRINGS: {
			struct strings_descp *strings_descp =
				(struct strings_descp *)&avdecc->sd.tx_buf
					[sizeof(struct ethhdr) +
					 sizeof(struct avt_pdu_control_hdr) +
					 sizeof(struct read_descp_res)];

			avb_log(AVB_KERN_INFO, KERN_INFO
				"avb_aecp_readResponse for Strings Descriptor");

			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_SUCCESS,
				(sizeof(struct read_descp_res) +
				 sizeof(struct strings_descp)),
				1);

			strings_descp->desc_type = htons(AVB_AEM_DESCP_STRINGS);
			strings_descp->desc_idx = 0;
			strcpy(&strings_descp->strings[0][0], "FH-Kiel");
			strcpy(&strings_descp->strings[1][0], "BBB");
			strcpy(&strings_descp->strings[2][0], "Stream IP");
			strcpy(&strings_descp->strings[3][0], "Stream OP");
			strcpy(&strings_descp->strings[4][0], "Domain Clock");
			strcpy(&strings_descp->strings[5][0], "External Clock");
			strcpy(&strings_descp->strings[6][0], "Stream Clock");
			descp_size = sizeof(struct strings_descp);
			break;
		}
		case AVB_AEM_DESCP_STREAM_IP: {
			struct stream_descp *strm_op_descp =
				(struct stream_descp *)&avdecc->sd.tx_buf
					[sizeof(struct ethhdr) +
					 sizeof(struct avt_pdu_control_hdr) +
					 sizeof(struct read_descp_res)];

			avb_log(AVB_KERN_INFO, KERN_INFO
				"avb_aecp_readResponse for Stream Out Descriptor");

			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_SUCCESS,
				(sizeof(struct read_descp_res) +
				 sizeof(struct stream_descp)),
				1);
			strm_op_descp->desc_type =
				htons(AVB_AEM_DESCP_STREAM_IP);
			strm_op_descp->desc_idx = 0;
			strm_op_descp->localized_descp = htons(0x0002);
			strm_op_descp->clock_domain_idx = 0;
			strm_op_descp->stream_flags = htons(0x0003);
			strm_op_descp->avb_if_idx = 0;
			strm_op_descp->buf_size = htonl(583333);
			strm_op_descp->fmts_count =
				htons(AVB_AEM_MAX_SUPP_FORMATS);
			strm_op_descp->fmts_off = htons(132);

#if 0
					strm_op_descp->supp_fmts[0].fmt.avtp.sub_type  = AVB_AEM_STREAM_FORMAT_AVTP;
					strm_op_descp->supp_fmts[0].fmt.avtp.b1.nsr   = 5;
					strm_op_descp->supp_fmts[0].fmt.avtp.format   = 4;
					strm_op_descp->supp_fmts[0].fmt.avtp.bit_depth = 16;
					strm_op_descp->supp_fmts[0].fmt.avtp.cpf      = 2;
					strm_op_descp->supp_fmts[0].fmt.avtp.b5.cpf   = 0;
					strm_op_descp->supp_fmts[0].fmt.avtp.b5.spf   = strm_op_descp->supp_fmts[0].fmt.avtp.b5.spf | 0x01;
					strm_op_descp->supp_fmts[0].fmt.avtp.b6.spf   = 0;
					strm_op_descp->supp_fmts[0].fmt.avtp.b6.res2  = 0;
					strm_op_descp->supp_fmts[0].fmt.avtp.res2     = 0;
#else
			strm_op_descp->supp_fmts[0].fmt.iec.sub_type = 0;
			strm_op_descp->supp_fmts[0].fmt.iec.b1.sf = 0x80;
			strm_op_descp->supp_fmts[0].fmt.iec.b1.fmt =
				strm_op_descp->supp_fmts[0].fmt.iec.b1.fmt |
				0x20;
			strm_op_descp->supp_fmts[0].fmt.iec.b2.fdf_sfc = 0x01;
			strm_op_descp->supp_fmts[0].fmt.iec.dbs = 0x08;
			strm_op_descp->supp_fmts[0].fmt.iec.b4.b = 0x60;
			strm_op_descp->supp_fmts[0].fmt.iec.label_mbla_cnt =
				0x08;
#endif
			memcpy(&strm_op_descp->supp_fmts[1],
			       &strm_op_descp->supp_fmts[0],
			       sizeof(struct stream_format));
			memcpy(&strm_op_descp->supp_fmts[2],
			       &strm_op_descp->supp_fmts[0],
			       sizeof(struct stream_format));
			memcpy(&strm_op_descp->supp_fmts[3],
			       &strm_op_descp->supp_fmts[0],
			       sizeof(struct stream_format));
			memcpy(&strm_op_descp->supp_fmts[4],
			       &strm_op_descp->supp_fmts[0],
			       sizeof(struct stream_format));
			memcpy(&strm_op_descp->supp_fmts[5],
			       &strm_op_descp->supp_fmts[0],
			       sizeof(struct stream_format));

			strm_op_descp->supp_fmts[1].fmt.iec.b2.fdf_sfc = 0x02;
			strm_op_descp->supp_fmts[2].fmt.iec.b2.fdf_sfc = 0x03;
			strm_op_descp->supp_fmts[3].fmt.iec.b2.fdf_sfc = 0x04;
			strm_op_descp->supp_fmts[4].fmt.iec.b2.fdf_sfc = 0x05;
			strm_op_descp->supp_fmts[5].fmt.iec.b2.fdf_sfc = 0x06;

			memcpy(&strm_op_descp->curr_fmt,
			       &strm_op_descp->supp_fmts[1],
			       sizeof(struct stream_format));

			descp_size = sizeof(struct stream_descp);
			break;
		}
		case AVB_AEM_DESCP_STREAM_OP: {
			struct stream_descp *strm_op_descp =
				(struct stream_descp *)&avdecc->sd.tx_buf
					[sizeof(struct ethhdr) +
					 sizeof(struct avt_pdu_control_hdr) +
					 sizeof(struct read_descp_res)];

			avb_log(AVB_KERN_INFO, KERN_INFO
				"avb_aecp_readResponse for Stream Out Descriptor");

			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_SUCCESS,
				(sizeof(struct read_descp_res) +
				 sizeof(struct stream_descp)),
				1);
			strm_op_descp->desc_type =
				htons(AVB_AEM_DESCP_STREAM_OP);
			strm_op_descp->desc_idx = 0;
			strm_op_descp->localized_descp = htons(0x0003);
			strm_op_descp->clock_domain_idx = 0;
			strm_op_descp->stream_flags = htons(0x0002);
			strm_op_descp->avb_if_idx = 0;
			strm_op_descp->buf_size = htonl(583333);
			strm_op_descp->fmts_count =
				htons(AVB_AEM_MAX_SUPP_FORMATS);
			strm_op_descp->fmts_off = htons(132);

#if 0
					strm_op_descp->supp_fmts[0].fmt.avtp.sub_type  = AVB_AEM_STREAM_FORMAT_AVTP;
					strm_op_descp->supp_fmts[0].fmt.avtp.b1.nsr   = 5;
					strm_op_descp->supp_fmts[0].fmt.avtp.format   = 4;
					strm_op_descp->supp_fmts[0].fmt.avtp.bit_depth = 16;
					strm_op_descp->supp_fmts[0].fmt.avtp.cpf      = 2;
					strm_op_descp->supp_fmts[0].fmt.avtp.b5.cpf   = 0;
					strm_op_descp->supp_fmts[0].fmt.avtp.b5.spf   = strm_op_descp->supp_fmts[0].fmt.avtp.b5.spf | 0x01;
					strm_op_descp->supp_fmts[0].fmt.avtp.b6.spf   = 0;
					strm_op_descp->supp_fmts[0].fmt.avtp.b6.res2  = 0;
					strm_op_descp->supp_fmts[0].fmt.avtp.res2     = 0;
#else
			strm_op_descp->supp_fmts[0].fmt.iec.sub_type = 0;
			strm_op_descp->supp_fmts[0].fmt.iec.b1.sf = 0x80;
			strm_op_descp->supp_fmts[0].fmt.iec.b1.fmt =
				strm_op_descp->supp_fmts[0].fmt.iec.b1.fmt |
				0x20;
			strm_op_descp->supp_fmts[0].fmt.iec.b2.fdf_sfc = 0x01;
			strm_op_descp->supp_fmts[0].fmt.iec.dbs = 0x08;
			strm_op_descp->supp_fmts[0].fmt.iec.b4.b = 0x60;
			strm_op_descp->supp_fmts[0].fmt.iec.label_mbla_cnt =
				0x08;
#endif
			memcpy(&strm_op_descp->supp_fmts[1],
			       &strm_op_descp->supp_fmts[0],
			       sizeof(struct stream_format));
			memcpy(&strm_op_descp->supp_fmts[2],
			       &strm_op_descp->supp_fmts[0],
			       sizeof(struct stream_format));
			memcpy(&strm_op_descp->supp_fmts[3],
			       &strm_op_descp->supp_fmts[0],
			       sizeof(struct stream_format));
			memcpy(&strm_op_descp->supp_fmts[4],
			       &strm_op_descp->supp_fmts[0],
			       sizeof(struct stream_format));
			memcpy(&strm_op_descp->supp_fmts[5],
			       &strm_op_descp->supp_fmts[0],
			       sizeof(struct stream_format));

			strm_op_descp->supp_fmts[1].fmt.iec.b2.fdf_sfc = 0x02;
			strm_op_descp->supp_fmts[2].fmt.iec.b2.fdf_sfc = 0x03;
			strm_op_descp->supp_fmts[3].fmt.iec.b2.fdf_sfc = 0x04;
			strm_op_descp->supp_fmts[4].fmt.iec.b2.fdf_sfc = 0x05;
			strm_op_descp->supp_fmts[5].fmt.iec.b2.fdf_sfc = 0x06;

			memcpy(&strm_op_descp->curr_fmt,
			       &strm_op_descp->supp_fmts[1],
			       sizeof(struct stream_format));

			descp_size = sizeof(struct stream_descp);
			break;
		}
		default: {
			struct read_descp_cmd *rd_res_cmd =
				(struct read_descp_cmd *)rd_res;
			rd_res_cmd->desc_type = rd_cmd->desc_type;
			rd_res_cmd->desc_idx = rd_cmd->desc_idx;
			descp_size = 0;
			tx_size = sizeof(struct ethhdr) +
				  sizeof(struct avt_pdu_control_hdr) +
				  sizeof(struct read_descp_cmd);
			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_NOT_IMPLEMENTED,
				(sizeof(struct read_descp_cmd)), 1);
			avb_log(AVB_KERN_INFO,
				KERN_INFO
				"avb_avdecc_aecp_respond_to_AEM_cmd unknown desc_type: %d",
				htons(rd_cmd->desc_type));
			break;
		}
		}

		if (descp_size > 0) {
			if ((htons(rd_cmd->cfg_idx) > max_cfg_idx) ||
			    (htons(rd_cmd->desc_idx) > max_desc_idx)) {
				struct read_descp_cmd *rd_res_cmd =
					(struct read_descp_cmd *)rd_res;
				rd_res_cmd->desc_type = rd_cmd->desc_type;
				rd_res_cmd->desc_idx = rd_cmd->desc_idx;
				tx_size = sizeof(struct ethhdr) +
					  sizeof(struct avt_pdu_control_hdr) +
					  sizeof(struct read_descp_cmd);
				avb_acdecc_fill_AVTP_ctrl_hdr(
					avdecc, AVB_AVTP_SUBTYPE_AECP,
					AVB_AECP_MSGTYPE_AEM_RESPONSE,
					AVB_AEM_RES_NO_SUCH_DESCRIPTOR,
					(sizeof(struct read_descp_cmd)), 1);
			} else {
				tx_size = sizeof(struct ethhdr) +
					  sizeof(struct avt_pdu_control_hdr) +
					  sizeof(struct read_descp_res) +
					  descp_size;
			}
		}

		break;
	}
	case AVB_AEM_CMD_ENTITY_ACQUIRE: {
		struct acquire_ent_cmd *acq_ent_cmd =
			(struct acquire_ent_cmd *)&avdecc->sd
				.tx_buf[sizeof(struct ethhdr) +
					sizeof(struct avt_pdu_control_hdr)];

		avb_log(AVB_KERN_INFO,
			KERN_INFO "avb_aecp_readResponse for Entiry Acquire");
		tx_size = sizeof(struct ethhdr) +
			  sizeof(struct avt_pdu_control_hdr) +
			  sizeof(struct acquire_ent_cmd);
		avb_acdecc_fill_AVTP_ctrl_hdr(avdecc, AVB_AVTP_SUBTYPE_AECP,
					      AVB_AECP_MSGTYPE_AEM_RESPONSE,
					      AVB_AEM_RES_SUCCESS,
					      (sizeof(struct acquire_ent_cmd)),
					      1);

		memcpy(&acq_ent_cmd->owner_id[0],
		       &acq_ent_cmd->hdr.ctrl_entity_id[0], 8);
		acq_ent_cmd->flags = 0;
		acq_ent_cmd->desc_type = htons(AVB_AEM_DESCP_ENTITY);
		acq_ent_cmd->desc_idx = 0;

		msrp->started = true;
		avb_msrp_talkerdeclarations(msrp, true, msrp->tx_state);
		break;
	}
	case AVB_AEM_CMD_ENTITY_AVAILABLE: {
		avb_log(AVB_KERN_INFO,
			KERN_INFO "avb_aecp_readResponse for Entity Available");
		tx_size = sizeof(struct ethhdr) +
			  sizeof(struct avt_pdu_control_hdr) +
			  sizeof(struct aem_cmd);
		avb_acdecc_fill_AVTP_ctrl_hdr(avdecc, AVB_AVTP_SUBTYPE_AECP,
					      AVB_AECP_MSGTYPE_AEM_RESPONSE,
					      AVB_AEM_RES_SUCCESS,
					      (sizeof(struct aem_cmd)), 1);
		break;
	}
	case AVB_AEM_CMD_SET_STREAM_FORMAT: {
		struct set_stream_format_cmd *reqStFmt =
			(struct set_stream_format_cmd *)&avdecc->sd
				.rx_buf[sizeof(struct ethhdr) +
					sizeof(struct avt_pdu_control_hdr)];
		struct set_stream_format_cmd *stFmt =
			(struct set_stream_format_cmd *)&avdecc->sd
				.tx_buf[sizeof(struct ethhdr) +
					sizeof(struct avt_pdu_control_hdr)];

		avb_log(AVB_KERN_INFO, KERN_INFO
			"avb_aecp_readResponse for Set Stream Format");
		tx_size = sizeof(struct ethhdr) +
			  sizeof(struct avt_pdu_control_hdr) +
			  sizeof(struct set_stream_format_cmd);
		avb_acdecc_fill_AVTP_ctrl_hdr(
			avdecc, AVB_AVTP_SUBTYPE_AECP,
			AVB_AECP_MSGTYPE_AEM_RESPONSE, AVB_AEM_RES_SUCCESS,
			(sizeof(struct set_stream_format_cmd)), 1);
		memcpy(stFmt, reqStFmt, sizeof(struct set_stream_format_cmd));
		break;
	}
	case AVB_AEM_CMD_GET_STREAM_INFO: {
		struct aem_stream_info *get_stream_info_req =
			(struct aem_stream_info *)&avdecc->sd
				.rx_buf[sizeof(struct ethhdr) +
					sizeof(struct avt_pdu_control_hdr)];
		struct aem_stream_info *aem_stream_info =
			(struct aem_stream_info *)&avdecc->sd
				.tx_buf[sizeof(struct ethhdr) +
					sizeof(struct avt_pdu_control_hdr)];

		avb_log(AVB_KERN_INFO, KERN_INFO
			"avb_aecp_readResponse for Get Stream Info command");

		avb_acdecc_fill_AVTP_ctrl_hdr(avdecc, AVB_AVTP_SUBTYPE_AECP,
					      AVB_AECP_MSGTYPE_AEM_RESPONSE,
					      AVB_AEM_RES_SUCCESS,
					      sizeof(struct aem_stream_info),
					      1);
		aem_stream_info->desc_type = get_stream_info_req->desc_type;
		aem_stream_info->desc_idx = 0;
		aem_stream_info->flags = htonl(0x0);
		aem_stream_info->curr_fmt.fmt.avtp.sub_type =
			AVB_AEM_STREAM_FORMAT_AVTP;
		aem_stream_info->curr_fmt.fmt.avtp.b1.nsr = 5;
		aem_stream_info->curr_fmt.fmt.avtp.format = 4;
		aem_stream_info->curr_fmt.fmt.avtp.bit_depth = 16;
		aem_stream_info->curr_fmt.fmt.avtp.cpf = 2;
		aem_stream_info->curr_fmt.fmt.avtp.b5.cpf = 0;
		aem_stream_info->curr_fmt.fmt.avtp.b5.spf =
			aem_stream_info->curr_fmt.fmt.avtp.b5.spf | 0x01;
		aem_stream_info->curr_fmt.fmt.avtp.b6.spf = 0;
		aem_stream_info->curr_fmt.fmt.avtp.b6.res2 = 0;
		aem_stream_info->curr_fmt.fmt.avtp.res2 = 0;
		aem_stream_info->stream_id[0] =
			((htons(get_stream_info_req->desc_type) ==
			  AVB_AEM_DESCP_STREAM_OP) ?
				 (avdecc->sd.srcmac[0]) :
				 (0));
		aem_stream_info->stream_id[1] =
			((htons(get_stream_info_req->desc_type) ==
			  AVB_AEM_DESCP_STREAM_OP) ?
				 (avdecc->sd.srcmac[1]) :
				 (0));
		aem_stream_info->stream_id[2] =
			((htons(get_stream_info_req->desc_type) ==
			  AVB_AEM_DESCP_STREAM_OP) ?
				 (avdecc->sd.srcmac[2]) :
				 (0));
		aem_stream_info->stream_id[3] =
			((htons(get_stream_info_req->desc_type) ==
			  AVB_AEM_DESCP_STREAM_OP) ?
				 (avdecc->sd.srcmac[3]) :
				 (0));
		aem_stream_info->stream_id[4] =
			((htons(get_stream_info_req->desc_type) ==
			  AVB_AEM_DESCP_STREAM_OP) ?
				 (avdecc->sd.srcmac[4]) :
				 (0));
		aem_stream_info->stream_id[5] =
			((htons(get_stream_info_req->desc_type) ==
			  AVB_AEM_DESCP_STREAM_OP) ?
				 (avdecc->sd.srcmac[5]) :
				 (0));
		aem_stream_info->stream_id[6] =
			((htons(get_stream_info_req->desc_type) ==
			  AVB_AEM_DESCP_STREAM_OP) ?
				 (0) :
				 (0));
		aem_stream_info->stream_id[7] =
			((htons(get_stream_info_req->desc_type) ==
			  AVB_AEM_DESCP_STREAM_OP) ?
				 (1) :
				 (0));
		aem_stream_info->msrp_accu_lat = htonl(0);
		aem_stream_info->stream_dest_MAC[0] = 0x91;
		aem_stream_info->stream_dest_MAC[1] = 0xe0;
		aem_stream_info->stream_dest_MAC[2] = 0xf0;
		aem_stream_info->stream_dest_MAC[3] = 0x00;
		aem_stream_info->stream_dest_MAC[4] = 0x3f;
		aem_stream_info->stream_dest_MAC[5] = 0xf1;
		aem_stream_info->msrp_failure_code = 0;
		aem_stream_info->stream_vlan_id = htons(2);

		if (gt_str_info_cmd->desc_idx > 0) {
			tx_size = sizeof(struct ethhdr) +
				  sizeof(struct avt_pdu_control_hdr) +
				  sizeof(struct get_stream_info_cmd);
			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_NO_SUCH_DESCRIPTOR,
				(sizeof(struct get_stream_info_cmd)), 1);
		} else {
			tx_size = sizeof(struct ethhdr) +
				  sizeof(struct avt_pdu_control_hdr) +
				  sizeof(struct aem_stream_info);
		}
		break;
	}
	case AVB_AEM_CMD_GET_COUNTERS: {
		struct counters_descp *counters_req =
			(struct counters_descp *)&avdecc->sd
				.rx_buf[sizeof(struct ethhdr) +
					sizeof(struct avt_pdu_control_hdr)];
		struct counters_descp *counters_descp =
			(struct counters_descp *)&avdecc->sd
				.tx_buf[sizeof(struct ethhdr) +
					sizeof(struct avt_pdu_control_hdr)];

		avb_log(AVB_KERN_INFO, KERN_INFO
			"avb_aecp_readResponse for Get Counters command");

		avb_acdecc_fill_AVTP_ctrl_hdr(avdecc, AVB_AVTP_SUBTYPE_AECP,
					      AVB_AECP_MSGTYPE_AEM_RESPONSE,
					      AVB_AEM_RES_SUCCESS,
					      sizeof(struct aem_stream_info),
					      1);

		counters_descp->desc_type = counters_req->desc_type;
		counters_descp->desc_idx = 0;

		if (counters_req->desc_type == AVB_AEM_DESCP_STREAM_IP) {
			counters_descp->counters_valid = htonl(0x00001800);

			counters_descp->counters[11] = 0;
			counters_descp->counters[12] = 0;
		} else {
			counters_descp->counters_valid = htonl(0x0);
		}

		if (gt_str_info_cmd->desc_idx > 0) {
			tx_size = sizeof(struct ethhdr) +
				  sizeof(struct avt_pdu_control_hdr) +
				  sizeof(struct get_counters_cmd);
			avb_acdecc_fill_AVTP_ctrl_hdr(
				avdecc, AVB_AVTP_SUBTYPE_AECP,
				AVB_AECP_MSGTYPE_AEM_RESPONSE,
				AVB_AEM_RES_NO_SUCH_DESCRIPTOR,
				(sizeof(struct get_counters_cmd)), 1);
		} else {
			tx_size = sizeof(struct ethhdr) +
				  sizeof(struct avt_pdu_control_hdr) +
				  sizeof(struct counters_descp);
		}
		break;
	}
	case AVB_AEM_CMD_REGISTER_UNSOLICITED_NOTIFICATION: {
		avb_log(AVB_KERN_INFO, KERN_INFO
			"avb_aecp_readResponse for Register Unsolicited Responses");
		tx_size = sizeof(struct ethhdr) +
			  sizeof(struct avt_pdu_control_hdr) +
			  sizeof(struct aem_cmd);
		avb_acdecc_fill_AVTP_ctrl_hdr(avdecc, AVB_AVTP_SUBTYPE_AECP,
					      AVB_AECP_MSGTYPE_AEM_RESPONSE,
					      AVB_AEM_RES_SUCCESS,
					      (sizeof(struct aem_cmd)), 1);
		break;
	}
	default: {
		tx_size = sizeof(struct ethhdr) +
			  sizeof(struct avt_pdu_control_hdr) +
			  sizeof(struct read_descp_res);
		avb_acdecc_fill_AVTP_ctrl_hdr(avdecc, AVB_AVTP_SUBTYPE_AECP,
					      AVB_AECP_MSGTYPE_AEM_RESPONSE,
					      AVB_AEM_RES_NOT_IMPLEMENTED,
					      (sizeof(struct read_descp_res)),
					      1);
		avb_log(AVB_KERN_INFO,
			KERN_INFO
			"avb_avdecc_aecp_respond_to_AEM_cmd unknown cmd_type: %d",
			htons(cmd->cmd_type));
		break;
	}
	}

	if (tx_size > 0) {
		avdecc->sd.tx_iov.iov_base = avdecc->sd.tx_buf;
		avdecc->sd.tx_iov.iov_len = tx_size;
		iov_iter_init(&avdecc->sd.tx_msg_hdr.msg_iter, WRITE,
			      &avdecc->sd.tx_iov, 1, tx_size);

		if ((err = sock_sendmsg(avdecc->sd.sock,
					&avdecc->sd.tx_msg_hdr)) <= 0) {
			avb_log(AVB_KERN_WARN,
				KERN_WARNING
				"avb_avdecc_aecp_respond_to_AEM_cmd Socket transmission fails %d \n",
				err);
			return;
		}
	}
}

static void avb_avdecc_aecp_respond_to_cmd(struct avdecc *avdecc,
					   struct msrp *msrp)
{
	struct avt_pdu_control_hdr *hdr =
		(struct avt_pdu_control_hdr *)&avdecc->sd
			.rx_buf[sizeof(struct ethhdr)];

	switch (hdr->h.f.b1.msg_type) {
	case AVB_AECP_MSGTYPE_AEM_COMMAND:
		avb_avdecc_aecp_respond_to_AEM_cmd(avdecc, msrp);
		break;

	default:
		avb_log(AVB_KERN_INFO,
			KERN_INFO
			"avb_avdecc_aecp_respond_to_cmd unknown sub_type: %d",
			hdr->h.f.b1.msg_type);
		break;
	}
}

static void avb_avdecc_acmp_respond_to_cmd(struct avdecc *avdecc)
{
	int err = 0;
	int dest = 0;
	u16 tx_size = 0;
	struct avt_pdu_control_hdr *hdr =
		(struct avt_pdu_control_hdr *)&avdecc->sd
			.rx_buf[sizeof(struct ethhdr)];
	struct acm_pdu *rx_acmp_du =
		(struct acm_pdu *)&avdecc->sd
			.rx_buf[sizeof(struct ethhdr) +
				sizeof(struct avt_pdu_control_hdr)];
	struct acm_pdu *tx_acmp_du =
		(struct acm_pdu *)&avdecc->sd
			.tx_buf[sizeof(struct ethhdr) +
				sizeof(struct avt_pdu_control_hdr)];

	if ((rx_acmp_du->talker_entity_id[0] == avdecc->sd.srcmac[0]) &&
	    (rx_acmp_du->talker_entity_id[1] == avdecc->sd.srcmac[1]) &&
	    (rx_acmp_du->talker_entity_id[2] == avdecc->sd.srcmac[2]) &&
	    (rx_acmp_du->talker_entity_id[3] == 0xff) &&
	    (rx_acmp_du->talker_entity_id[4] == 0xfe) &&
	    (rx_acmp_du->talker_entity_id[5] == avdecc->sd.srcmac[3]) &&
	    (rx_acmp_du->talker_entity_id[6] == avdecc->sd.srcmac[4]) &&
	    (rx_acmp_du->talker_entity_id[7] == avdecc->sd.srcmac[5])) {
		dest++;
	} else if ((rx_acmp_du->listener_entity_id[0] ==
		    avdecc->sd.srcmac[0]) &&
		   (rx_acmp_du->listener_entity_id[1] ==
		    avdecc->sd.srcmac[1]) &&
		   (rx_acmp_du->listener_entity_id[2] ==
		    avdecc->sd.srcmac[2]) &&
		   (rx_acmp_du->listener_entity_id[3] == 0xff) &&
		   (rx_acmp_du->listener_entity_id[4] == 0xfe) &&
		   (rx_acmp_du->listener_entity_id[5] ==
		    avdecc->sd.srcmac[3]) &&
		   (rx_acmp_du->listener_entity_id[6] ==
		    avdecc->sd.srcmac[4]) &&
		   (rx_acmp_du->listener_entity_id[7] ==
		    avdecc->sd.srcmac[5])) {
		dest--;
	} else {
		dest = 0;
	}

	if (((hdr->h.f.b1.msg_type == AVB_ACMP_MSGTYPE_GET_TX_STATE_CMD) ||
	     (hdr->h.f.b1.msg_type == AVB_ACMP_MSGTYPE_CONNECT_TX_CMD) ||
	     (hdr->h.f.b1.msg_type == AVB_ACMP_MSGTYPE_DISCONNECT_TX_CMD) ||
	     (hdr->h.f.b1.msg_type == AVB_ACMP_MSGTYPE_GET_TX_CONN_CMD)) &&
	    (dest <= 0)) {
		avb_log(AVB_KERN_INFO,
			KERN_INFO
			"avb_avdecc_aecp_respond_to_cmd ACMP TX cmd: %d not for us",
			hdr->h.f.b1.msg_type);
		return;
	}

	if (((hdr->h.f.b1.msg_type == AVB_ACMP_MSGTYPE_GET_RX_STATE_CMD) ||
	     (hdr->h.f.b1.msg_type == AVB_ACMP_MSGTYPE_CONNECT_RX_CMD) ||
	     (hdr->h.f.b1.msg_type == AVB_ACMP_MSGTYPE_DISCONNECT_RX_CMD)) &&
	    (dest >= 0)) {
		avb_log(AVB_KERN_INFO,
			KERN_INFO
			"avb_avdecc_aecp_respond_to_cmd ACMP RX cmd: %d not for us",
			hdr->h.f.b1.msg_type);
		return;
	}

	switch (hdr->h.f.b1.msg_type) {
	case AVB_ACMP_MSGTYPE_GET_TX_STATE_CMD:
		avb_log(AVB_KERN_INFO, KERN_INFO
			"avb_avdecc_adp_respond_to_cmd Get TX state command");
		avb_acdecc_init_and_fill_eth_hdr(avdecc, 1);
		avb_acdecc_fill_AVTP_ctrl_hdr(
			avdecc, AVB_AVTP_SUBTYPE_ACMP,
			AVB_ACMP_MSGTYPE_GET_TX_STATE_RESP,
			avdecc->acmp_tx_state, sizeof(struct acm_pdu), 0);
		memcpy(tx_acmp_du, rx_acmp_du, sizeof(struct acm_pdu));
		tx_acmp_du->stream_dest_MAC[0] = 0x91;
		tx_acmp_du->stream_dest_MAC[1] = 0xe0;
		tx_acmp_du->stream_dest_MAC[2] = 0xf0;
		tx_acmp_du->stream_dest_MAC[3] = 0x00;
		tx_acmp_du->stream_dest_MAC[4] = 0x33;
		tx_acmp_du->stream_dest_MAC[5] = 0x4b;
		tx_acmp_du->flags = htons(0x0000);
		tx_size = sizeof(struct ethhdr) +
			  sizeof(struct avt_pdu_control_hdr) +
			  sizeof(struct acm_pdu);
		break;

	case AVB_ACMP_MSGTYPE_GET_RX_STATE_CMD:
		avb_log(AVB_KERN_INFO, KERN_INFO
			"avb_avdecc_adp_respond_to_cmd Get RX state command");
		avb_acdecc_init_and_fill_eth_hdr(avdecc, 1);
		avb_acdecc_fill_AVTP_ctrl_hdr(
			avdecc, AVB_AVTP_SUBTYPE_ACMP,
			AVB_ACMP_MSGTYPE_GET_RX_STATE_RESP,
			avdecc->acmp_rx_state, sizeof(struct acm_pdu), 0);
		memcpy(tx_acmp_du, rx_acmp_du, sizeof(struct acm_pdu));
		tx_acmp_du->stream_dest_MAC[0] = 0x91;
		tx_acmp_du->stream_dest_MAC[1] = 0xe0;
		tx_acmp_du->stream_dest_MAC[2] = 0xf0;
		tx_acmp_du->stream_dest_MAC[3] = 0x00;
		tx_acmp_du->stream_dest_MAC[4] = 0x33;
		tx_acmp_du->stream_dest_MAC[5] = 0x4c;
		tx_acmp_du->flags = htons(0x0000);
		tx_size = sizeof(struct ethhdr) +
			  sizeof(struct avt_pdu_control_hdr) +
			  sizeof(struct acm_pdu);
		break;

	case AVB_ACMP_MSGTYPE_CONNECT_TX_CMD:
		avb_log(AVB_KERN_INFO, KERN_INFO
			"avb_avdecc_adp_respond_to_cmd Connect TX command");
		avb_acdecc_init_and_fill_eth_hdr(avdecc, 1);
		avb_acdecc_fill_AVTP_ctrl_hdr(avdecc, AVB_AVTP_SUBTYPE_ACMP,
					      AVB_ACMP_MSGTYPE_CONNECT_TX_RESP,
					      AVB_ACMP_STATUS_SUCCESS,
					      sizeof(struct acm_pdu), 0);
		memcpy(tx_acmp_du, rx_acmp_du, sizeof(struct acm_pdu));
		tx_acmp_du->stream_dest_MAC[0] = 0x91;
		tx_acmp_du->stream_dest_MAC[1] = 0xe0;
		tx_acmp_du->stream_dest_MAC[2] = 0xf0;
		tx_acmp_du->stream_dest_MAC[3] = 0x00;
		tx_acmp_du->stream_dest_MAC[4] = 0x33;
		tx_acmp_du->stream_dest_MAC[5] = 0x4b;
		tx_acmp_du->flags = htons(0x0000);
		tx_size = sizeof(struct ethhdr) +
			  sizeof(struct avt_pdu_control_hdr) +
			  sizeof(struct acm_pdu);
		break;

	case AVB_ACMP_MSGTYPE_DISCONNECT_TX_CMD:
		avb_log(AVB_KERN_INFO, KERN_INFO
			"avb_avdecc_adp_respond_to_cmd Disconnect TX command");
		avb_acdecc_init_and_fill_eth_hdr(avdecc, 1);
		avb_acdecc_fill_AVTP_ctrl_hdr(
			avdecc, AVB_AVTP_SUBTYPE_ACMP,
			AVB_ACMP_MSGTYPE_DISCONNECT_TX_RESP,
			AVB_ACMP_STATUS_SUCCESS, sizeof(struct acm_pdu), 0);
		memcpy(tx_acmp_du, rx_acmp_du, sizeof(struct acm_pdu));
		tx_acmp_du->flags = htons(0x0000);
		tx_size = sizeof(struct ethhdr) +
			  sizeof(struct avt_pdu_control_hdr) +
			  sizeof(struct acm_pdu);
		break;

	case AVB_ACMP_MSGTYPE_CONNECT_RX_CMD:
		avb_log(AVB_KERN_INFO, KERN_INFO
			"avb_avdecc_adp_respond_to_cmd Connect RX command");
		avb_acdecc_init_and_fill_eth_hdr(avdecc, 1);
		avb_acdecc_fill_AVTP_ctrl_hdr(avdecc, AVB_AVTP_SUBTYPE_ACMP,
					      AVB_ACMP_MSGTYPE_CONNECT_RX_RESP,
					      AVB_ACMP_STATUS_SUCCESS,
					      sizeof(struct acm_pdu), 0);
		memcpy(tx_acmp_du, rx_acmp_du, sizeof(struct acm_pdu));
		tx_acmp_du->stream_dest_MAC[0] = 0x91;
		tx_acmp_du->stream_dest_MAC[1] = 0xe0;
		tx_acmp_du->stream_dest_MAC[2] = 0xf0;
		tx_acmp_du->stream_dest_MAC[3] = 0x00;
		tx_acmp_du->stream_dest_MAC[4] = 0x33;
		tx_acmp_du->stream_dest_MAC[5] = 0x4c;
		tx_acmp_du->flags = htons(0x0000);
		tx_size = sizeof(struct ethhdr) +
			  sizeof(struct avt_pdu_control_hdr) +
			  sizeof(struct acm_pdu);
		break;

	case AVB_ACMP_MSGTYPE_DISCONNECT_RX_CMD:
		avb_log(AVB_KERN_INFO, KERN_INFO
			"avb_avdecc_adp_respond_to_cmd Disconnect RX command");
		avb_acdecc_init_and_fill_eth_hdr(avdecc, 1);
		avb_acdecc_fill_AVTP_ctrl_hdr(
			avdecc, AVB_AVTP_SUBTYPE_ACMP,
			AVB_ACMP_MSGTYPE_DISCONNECT_RX_RESP,
			AVB_ACMP_STATUS_SUCCESS, sizeof(struct acm_pdu), 0);
		memcpy(tx_acmp_du, rx_acmp_du, sizeof(struct acm_pdu));
		tx_acmp_du->flags = htons(0x0000);
		tx_size = sizeof(struct ethhdr) +
			  sizeof(struct avt_pdu_control_hdr) +
			  sizeof(struct acm_pdu);
		break;

	default:
		avb_log(AVB_KERN_INFO,
			KERN_INFO
			"avb_avdecc_acmp_respond_to_cmd unknown sub_type: %d",
			hdr->h.f.b1.msg_type);
		break;
	}

	if (tx_size > 0) {
		avdecc->sd.tx_iov.iov_base = avdecc->sd.tx_buf;
		avdecc->sd.tx_iov.iov_len = tx_size;
		iov_iter_init(&avdecc->sd.tx_msg_hdr.msg_iter, WRITE,
			      &avdecc->sd.tx_iov, 1, tx_size);

		if ((err = sock_sendmsg(avdecc->sd.sock,
					&avdecc->sd.tx_msg_hdr)) <= 0) {
			avb_log(AVB_KERN_WARN,
				KERN_WARNING
				"avb_avdecc_acmp_respondToAEMCmd Socket transmission fails %d \n",
				err);
			return;
		}
	}
}

static void avb_avdecc_adp_respond_to_cmd(struct avdecc *avdecc)
{
	struct avt_pdu_control_hdr *hdr =
		(struct avt_pdu_control_hdr *)&avdecc->sd
			.rx_buf[sizeof(struct ethhdr)];

	switch (hdr->h.f.b1.msg_type) {
	case AVB_ADP_MSGTYPE_ENTITY_AVAILABLE:
		avb_log(AVB_KERN_INFO, KERN_INFO
			"avb_avdecc_adp_respond_to_cmd Entity Available");
		break;

	case AVB_ADP_MSGTYPE_ENTITY_DEPARTING:
		avb_log(AVB_KERN_INFO, KERN_INFO
			"avb_avdecc_adp_respond_to_cmd Entity Departing");
		break;

	case AVB_ADP_MSGTYPE_ENTITY_DISCOVER:
		avb_log(AVB_KERN_INFO, KERN_INFO
			"avb_avdecc_adp_respond_to_cmd Entity Discover");
		break;

	default:
		avb_log(AVB_KERN_INFO,
			KERN_INFO
			"avb_avdecc_adp_respond_to_cmd unknown sub_type: %d",
			hdr->h.f.b1.msg_type);
		break;
	}
}

void avb_avdecc_listen_and_respond(struct avdecc *avdecc, struct msrp *msrp)
{
	struct avt_pdu_control_hdr *hdr =
		(struct avt_pdu_control_hdr *)&avdecc->sd
			.rx_buf[sizeof(struct ethhdr)];

	if (avb_avdecc_listen(avdecc) > 0) {
		switch (hdr->h.f.sub_type) {
		case AVB_AVTP_SUBTYPE_ADP:
			avb_avdecc_adp_respond_to_cmd(avdecc);
			break;

		case AVB_AVTP_SUBTYPE_AECP:
			avb_avdecc_aecp_respond_to_cmd(avdecc, msrp);
			break;

		case AVB_AVTP_SUBTYPE_ACMP:
			avb_avdecc_acmp_respond_to_cmd(avdecc);
			break;

		default:
			avb_log(AVB_KERN_INFO,
				KERN_INFO
				"avb_avdecc_listen_and_respond unknown sub_type: %d",
				hdr->h.f.sub_type);
			break;
		}
	}
}