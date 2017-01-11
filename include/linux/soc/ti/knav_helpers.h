/*
 * Copyright (C) 2015 Texas Instruments Incorporated
 * Authors:	Murali Karicheri
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SOC_TI_KEYSTONE_NAVIGATOR_HELPERS_H__
#define __SOC_TI_KEYSTONE_NAVIGATOR_HELPERS_H__

/* Helper functions - Get/Set */
static inline void knav_dma_get_pkt_info(dma_addr_t *buff, u32 *buff_len,
					 dma_addr_t *ndesc,
					 struct knav_dma_desc *desc)
{
	*buff_len = le32_to_cpu(desc->buff_len);
	*buff = le32_to_cpu(desc->buff);
	*ndesc = le32_to_cpu(desc->next_desc);
}

static inline u32 get_sw_data(int index, struct knav_dma_desc *desc)
{
	/* No Endian conversion needed as this data is untouched by hw */
	return desc->sw_data[index];
}

/* use these macros to get sw data */
#define KNAV_DMA_GET_SW_DATA0(desc) get_sw_data(0, desc)
#define KNAV_DMA_GET_SW_DATA1(desc) get_sw_data(1, desc)
#define KNAV_DMA_GET_SW_DATA2(desc) get_sw_data(2, desc)
#define KNAV_DMA_GET_SW_DATA3(desc) get_sw_data(3, desc)

static inline void knav_dma_get_desc_info(u32 *desc_info, u32 *pkt_info,
					  struct knav_dma_desc *desc)
{
	*desc_info = le32_to_cpu(desc->desc_info);
	*pkt_info = le32_to_cpu(desc->packet_info);
}

static inline void knav_get_org_pkt_info(dma_addr_t *buff, u32 *buff_len,
					 struct knav_dma_desc *desc)
{
	*buff = le32_to_cpu(desc->orig_buff);
	*buff_len = le32_to_cpu(desc->orig_len);
}

static inline void knav_dma_get_words(dma_addr_t *words, int num_words,
				      __le32 *desc)
{
	int i;

	for (i = 0; i < num_words; i++)
		words[i] = le32_to_cpu(desc[i]);
}

static inline void knav_dma_set_pkt_info(dma_addr_t buff, u32 buff_len,
					 u32 ndesc, struct knav_dma_desc *desc)
{
	desc->buff_len = cpu_to_le32(buff_len);
	desc->buff = cpu_to_le32(buff);
	desc->next_desc = cpu_to_le32(ndesc);
}

static inline void knav_dma_set_desc_info(u32 desc_info, u32 pkt_info,
					  struct knav_dma_desc *desc)
{
	desc->desc_info = cpu_to_le32(desc_info);
	desc->packet_info = cpu_to_le32(pkt_info);
}

static inline void set_sw_data(int index, u32 data, struct knav_dma_desc *desc)
{
	/* No Endian conversion needed as this data is untouched by hw */
	desc->sw_data[index] = data;
}

/* use these macros to set sw data */
#define KNAV_DMA_SET_SW_DATA0(data, desc) set_sw_data(0, data, desc)
#define KNAV_DMA_SET_SW_DATA1(data, desc) set_sw_data(1, data, desc)
#define KNAV_DMA_SET_SW_DATA2(data, desc) set_sw_data(2, data, desc)
#define KNAV_DMA_SET_SW_DATA3(data, desc) set_sw_data(3, data, desc)

static inline void knav_dma_set_org_pkt_info(dma_addr_t buff, u32 buff_len,
					     struct knav_dma_desc *desc)
{
	desc->orig_buff = cpu_to_le32(buff);
	desc->orig_len = cpu_to_le32(buff_len);
}

static inline void knav_dma_set_words(u32 *words, int num_words, __le32 *desc)
{
	int i;

	for (i = 0; i < num_words; i++)
		desc[i] = cpu_to_le32(words[i]);
}

#define knav_queue_get_id(q)	knav_queue_device_control(q, \
				KNAV_QUEUE_GET_ID, (unsigned long)NULL)

#define knav_queue_enable_notify(q) knav_queue_device_control(q,	\
					KNAV_QUEUE_ENABLE_NOTIFY,	\
					(unsigned long)NULL)

#define knav_queue_disable_notify(q) knav_queue_device_control(q,	\
					KNAV_QUEUE_DISABLE_NOTIFY,	\
					(unsigned long)NULL)

#define knav_queue_get_count(q)	knav_queue_device_control(q, \
				KNAV_QUEUE_GET_COUNT, (unsigned long)NULL)

#endif /* __SOC_TI_KEYSTONE_NAVIGATOR_HELPERS_H__ */
