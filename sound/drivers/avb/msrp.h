#ifndef MSRP_H
#define MSRP_H

#include "avb_util.h"

#pragma pack(push, 1)

struct domain_msrp_first_value {
	u8 sr_class_id;
	u8 sr_class_prio;
	u16 sr_class_VID;
};

struct listener_msrp_first_value {
	u8 stream_id[8];
};

struct talker_msrp_first_value {
	u8 stream_id[8];
	u8 data_frame_params[8];
	u16 max_frame_size;
	u16 max_interval_frames;
	u8 priority_and_rank;
	u32 accumalated_latency;
};

struct bridge_msrp_firstvalue {
	u8 stream_id[8];
	u8 data_frame_params[8];
	u16 max_frame_size;
	u16 max_interval_frames;
	u8 priority_and_rank;
	u32 accumalated_latency;
	u8 bridge_id[8];
	u8 failure_reason;
};

struct vector_header {
	u16 number_of_values;
};

struct domain_vector_attribute {
	struct vector_header hdr;
	struct domain_msrp_first_value val;
	u8 vector[2];
};

struct listner_vector_attribute {
	struct vector_header hdr;
	struct listener_msrp_first_value val;
	u8 vector[2];
};

struct talker_vector_attribute {
	struct vector_header hdr;
	struct talker_msrp_first_value val;
	u8 vector[1];
};

struct domain_mrp_msg {
	u8 attribute_type;
	u8 attribute_len;
	u16 attribute_list_len;
	struct domain_vector_attribute attibute_list;
	u16 end_marker;
};

struct listner_mrp_msg {
	u8 attribute_type;
	u8 attribute_len;
	u16 attribute_list_len;
	struct listner_vector_attribute attibute_list;
	u16 end_marker;
};

struct talker_mrp_msg {
	u8 attribute_type;
	u8 attribute_len;
	u16 attribute_list_len;
	struct talker_vector_attribute attibute_list;
	u16 end_marker;
};

struct domain_msrp_du {
	u8 protocol_version;
	struct domain_mrp_msg msg;
	u16 end_marker;
};

struct listner_msrp_du {
	u8 protocol_version;
	struct listner_mrp_msg msg;
	u16 end_marker;
};

struct talker_msrp_du {
	u8 protocol_version;
	struct talker_mrp_msg msg;
	u16 end_marker;
};

#pragma pack(pop)

struct msrp {
	bool initialized;
	bool started;
	int rx_state;
	int tx_state;
	struct socketdata sd;
	u8 stream_id[8];
};

bool avb_msrp_init(struct msrp *msrp);
void avb_msrp_domaindeclarations(struct msrp *msrp);
void avb_msrp_talkerdeclarations(struct msrp *msrp, bool join, int state);
void avb_msrp_listenerdeclarations(struct msrp *msrp, bool join, int state);
int avb_msrp_listen(struct msrp *msrp);

#endif