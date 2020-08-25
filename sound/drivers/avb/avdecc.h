#ifndef AVDECC_H
#define AVDECC_H

#include "avb_util.h"
#include "msrp.h"

#pragma pack(push, 1)

struct avt_pdu_control_hdr {
	union tch {
		struct tcf {
			u8 sub_type;
			union tcb1 {
				u8 sv; /* 1 bit stream valid indication */
				u8 version; /* 3 bits version */
				u8 msg_type; /* 4 bit ControlData/MessageType */
			} b1;
			union tcb2 {
				u8 valid_time; /* 5 bit Status/ValidTime */
				u8 data_len; /* First 3 bits of control data length */
			} b2;
			u8 data_len; /* Last 8 bits of control data length */
			u8 stream_id[8]; /* Stream or entity id */
		} f;
		u8 bytes[AVTP_PDU_COMMON_CONTROL_HEADER_LENGTH];
	} h;
};

struct maap_pdu {
	union mtch {
		struct mtcf {
			u8 sub_type;
			union mtcb1 {
				u8 sv; /* 1 bit stream valid indication */
				u8 version; /* 3 bits version */
				u8 msg_type; /* 4 bit ControlData/MessageType */
			} b1;
			union mtcb2 {
				u8 maap_version; /* 5 bit Status/ValidTime */
				u8 data_len; /* First 3 bits of control data length */
			} b2;
			u8 data_len; /* Last 8 bits of control data length */
			u8 stream_id[8]; /* Stream or entity id */
		} f;
		u8 bytes[AVTP_PDU_COMMON_CONTROL_HEADER_LENGTH];
	} h;
	u8 req_MAC[6];
	u16 req_count;
	u8 conflict_MAC[6];
	u16 conflict_count;
};

struct avtp_stream_format {
	u8 sub_type;
	union sfb1 {
		u8 res1; /* 4 bit reserved */
		u8 nsr; /* 4 bit Nominal sample rate */
	} b1;
	u8 format;
	u8 bit_depth;
	u8 cpf; /* First 8 bits of channels per frame */
	union sfb5 {
		u8 cpf; /* Last 2 bits of channels per frame */
		u8 spf; /* First 6 bits of samples per frame */
	} b5;
	union sfb6 {
		u8 spf; /* Last 4 bits of samples per frame */
		u8 res2; /* First 4 bits of reserved */
	} b6;
	u8 res2; /* Last 8 bits of reserved */
};

struct iec_stream_format {
	u8 sub_type;
	union isfb1 {
		u8 sf; /* 1 bit stream format */
		u8 fmt; /* 6 bit fomrat */
		u8 r; /* 1 bit reserved */
	} b1;
	union isfb2 {
		u8 fdf_evt; /* 5 bits */
		u8 fdf_sfc; /* 3 bits */
	} b2;
	u8 dbs;
	union isfb4 {
		u8 b; /* 1 bit */
		u8 nb; /* 1 bit */
		u8 res; /* 6 bits */
	} b4;
	u8 label_iec_60958_cnt;
	u8 label_mbla_cnt;
	union isfb7 {
		u8 label_midi_cnt; /* 4 bits */
		u8 label_smptecnt; /* 4 bits */
	} b7;
};

struct stream_format {
	union fmt {
		struct avtp_stream_format avtp;
		struct iec_stream_format iec;
	} fmt;
};

struct acm_pdu {
	u8 ctrl_entity_id[8];
	u8 talker_entity_id[8];
	u8 listener_entity_id[8];
	u16 talker_unique_id;
	u16 listener_unique_id;
	u8 stream_dest_MAC[6];
	u16 connection_count;
	u16 sequence_id;
	u16 flags;
	u16 stream_vlan_id;
	u16 res;
};

struct aem_cmd {
	u8 ctrl_entity_id[8];
	u16 seq_id;
	u16 cmd_type;
};

struct acquire_ent_cmd {
	struct aem_cmd hdr;
	u32 flags;
	u8 owner_id[8];
	u16 desc_type;
	u16 desc_idx;
};

struct read_descp_cmd {
	struct aem_cmd hdr;
	u16 cfg_idx;
	u16 res;
	u16 desc_type;
	u16 desc_idx;
};

struct set_stream_format_cmd {
	struct aem_cmd hdr;
	u16 desc_type;
	u16 desc_idx;
	struct stream_format fmt;
};

struct read_descp_res {
	struct aem_cmd hdr;
	u16 cfg_idx;
	u16 res;
};

struct entity_descp {
	u16 desc_type;
	u16 desc_idx;
	u8 entity_id[8];
	u8 entity_model_id[8];
	u32 entity_caps;
	u16 talker_stream_sources;
	u16 talker_caps;
	u16 listener_stream_sinks;
	u16 listener_caps;
	u32 control_caps;
	u32 avai_idx;
	u8 association_id[8];
	u8 entity_name[64];
	u16 vendor_name_string;
	u16 model_name_string;
	u8 firmware_ver[64];
	u8 group_name[64];
	u8 serial_number[64];
	u16 cfg_count;
	u16 curr_cfg;
};

struct config_descp_count {
	u16 desc_type;
	u16 desc_count;
};

struct config_descp {
	u16 desc_type;
	u16 desc_idx;
	u8 obj_name[64];
	u16 localized_descp;
	u16 descp_count;
	u16 descpOff;
	struct config_descp_count descps[AVB_AEM_MAX_DESCP_COUNT];
};

struct audio_unit_descp {
	u16 desc_type;
	u16 desc_idx;
	u8 obj_name[64];
	u16 localized_descp;
	u16 clock_domain_idx;
	u16 num_stream_ip;
	u16 base_stream_ip;
	u16 num_stream_op;
	u16 base_stream_op;
	u16 num_ext_ip;
	u16 base_ext_ip;
	u16 num_ext_op;
	u16 base_ext_op;
	u16 num_int_ip;
	u16 base_int_ip;
	u16 num_int_op;
	u16 base_int_op;
	u16 num_controls;
	u16 base_control;
	u16 num_signal_selector;
	u16 base_signal_selector;
	u16 num_mixers;
	u16 base_mixer;
	u16 num_matrices;
	u16 base_matrix;
	u16 num_splitters;
	u16 base_splitter;
	u16 num_combiners;
	u16 base_combiner;
	u16 numde_multiplexer;
	u16 basede_multiplexer;
	u16 num_multiplexer;
	u16 base_multiplexer;
	u16 num_transcoders;
	u16 base_transcoder;
	u16 num_control_blocks;
	u16 base_control_block;
	u32 current_sampling_rate;
	u16 sampling_rates_offset;
	u16 sampling_rates_count;
	u32 sampling_rates[6];
};

struct stream_port_descp {
	u16 desc_type;
	u16 desc_idx;
	u16 clock_domain_idx;
	u16 port_flags;
	u16 num_controls;
	u16 base_control;
	u16 num_clusters;
	u16 base_cluster;
	u16 num_maps;
	u16 base_map;
};

struct ext_port_descp {
	u16 desc_type;
	u16 desc_idx;
	u16 clock_domain_idx;
	u16 port_flags;
	u16 num_controls;
	u16 base_control;
	u16 signal_type;
	u16 signal_idx;
	u16 signal_op;
	u32 block_latency;
	u16 jack_idx;
};

struct jack_descp {
	u16 desc_type;
	u16 desc_idx;
	u8 obj_name[64];
	u16 localized_descp;
	u16 jack_flags;
	u16 jack_type;
	u16 num_controls;
	u16 base_control;
};

struct audio_cluster_descp {
	u16 desc_type;
	u16 desc_idx;
	u8 obj_name[64];
	u16 localized_descp;
	u16 signal_type;
	u16 signal_idx;
	u16 signal_op;
	u32 path_latency;
	u32 block_latency;
	u16 num_channels;
	u8 format;
};

struct aud_map_fmt {
	u16 stream_idx;
	u16 stream_channel;
	u16 cluster_offset;
	u16 cluster_channel;
};

struct audio_map_descp {
	u16 desc_type;
	u16 desc_idx;
	u16 mapping_offset;
	u16 num_mappings;
	struct aud_map_fmt map[8];
};

struct clock_source_descp {
	u16 desc_type;
	u16 desc_idx;
	u8 obj_name[64];
	u16 localized_descp;
	u16 clock_source_flags;
	u16 clock_source_type;
	u8 clock_source_id[8];
	u16 clock_source_loc_type;
	u16 clock_source_loc_idx;
};

struct clock_domain_descp {
	u16 desc_type;
	u16 desc_idx;
	u8 obj_name[64];
	u16 localized_descp;
	u16 curr_clock_source;
	u16 clock_sources_offset;
	u16 clock_sources_count;
	u16 clock_sources[3];
};

struct avb_if_descp {
	u16 desc_type;
	u16 desc_idx;
	u8 if_name[64];
	u16 localized_descp;
	u8 mac_addr[6];
	u16 if_flags;
	u8 clock_iden[8];
	u8 prio1;
	u8 clock_class;
	u16 off_scaled_log_var;
	u8 clock_accu;
	u8 prio2;
	u8 domain_no;
	u8 log_sync_int;
	u8 log_anno_int;
	u8 log_pdelay_int;
	u16 port_no;
};

struct locale_descp {
	u16 desc_type;
	u16 desc_idx;
	u8 locale_id[64];
	u16 num_strings;
	u16 base_strings_idx;
};

struct strings_descp {
	u16 desc_type;
	u16 desc_idx;
	u8 strings[7][64];
};

struct stream_descp {
	u16 desc_type;
	u16 desc_idx;
	u8 obj_name[64];
	u16 localized_descp;
	u16 clock_domain_idx;
	u16 stream_flags;
	struct stream_format curr_fmt;
	u16 fmts_off;
	u16 fmts_count;
	u8 bkp_talker1_entity_id[8];
	u16 bkp_talker1_unique_id;
	u8 bkp_talker2_entity_id[8];
	u16 bkp_talker2_unique_id;
	u8 bkp_talker3_entity_id[8];
	u16 bkp_talker3_unique_id;
	u8 bkp_ip_talker_entity_id[8];
	u16 bkp_ip_talker_unique_id;
	u16 avb_if_idx;
	u32 buf_size;
	struct stream_format supp_fmts[AVB_AEM_MAX_SUPP_FORMATS];
};

struct get_counters_cmd {
	struct aem_cmd hdr;
	u16 desc_type;
	u16 desc_idx;
};

struct counters_descp {
	struct aem_cmd hdr;
	u16 desc_type;
	u16 desc_idx;
	u32 counters_valid;
	u32 counters[32];
};

struct get_stream_info_cmd {
	struct aem_cmd hdr;
	u16 desc_type;
	u16 desc_idx;
};

struct aem_stream_info {
	struct aem_cmd hdr;
	u16 desc_type;
	u16 desc_idx;
	u32 flags;
	struct stream_format curr_fmt;
	u8 stream_id[8];
	u32 msrp_accu_lat;
	u8 stream_dest_MAC[6];
	u8 msrp_failure_code;
	u8 res1;
	u8 msrp_failure_bridge_id[8];
	u16 stream_vlan_id;
	u16 res2;
};

struct adpdu {
	u8 entity_model_id[8];
	u32 entity_caps;
	u16 talker_stream_sources;
	u16 talker_caps;
	u16 listener_stream_sinks;
	u16 listener_caps;
	u32 control_caps;
	u32 avai_idx;
	u8 gptp_grand_master_id[8];
	u8 gptp_domain_number;
	u8 res1[3];
	u16 iden_ctrl_idx;
	u16 interface_idx;
	u8 association_id[8];
	u32 res2;
};

#pragma pack(pop)

struct avdecc {
	bool initialized;
	u8 acmp_tx_state;
	u8 acmp_rx_state;
	u32 adp_avai_idx;
	u64 last_ADP_adv_jiffy;
	struct socketdata sd;
};

bool avb_avdecc_init(struct avdecc *avdecc);
void avb_adp_discover(struct avdecc *avdecc);
void avb_adp_advertise(struct avdecc *avdecc);
void avb_maap_announce(struct avdecc *avdecc);
void avb_avdecc_listen_and_respond(struct avdecc *avdecc, struct msrp *msrp);

#endif