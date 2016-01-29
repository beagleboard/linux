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
static inline void knav_dma_get_pkt_info(u32 *buff, u32 *buff_len, u32 *ndesc,
					 struct knav_dma_desc *desc)
{
	*buff_len = desc->buff_len;
	*buff = desc->buff;
	*ndesc = desc->next_desc;
}

static inline void knav_dma_get_desc_info(u32 *desc_info, u32 *pkt_info,
					  struct knav_dma_desc *desc)
{
	*desc_info = desc->desc_info;
	*pkt_info = desc->packet_info;
}

static inline void knav_dma_get_pad_info(u32 *pad0, u32 *pad1,
					 struct knav_dma_desc *desc)
{
	*pad0 = desc->pad[0];
	*pad1 = desc->pad[1];
}

static inline void knav_get_org_pkt_info(u32 *buff, u32 *buff_len,
					 struct knav_dma_desc *desc)
{
	*buff = desc->orig_buff;
	*buff_len = desc->orig_len;
}

static inline void knav_dma_get_words(u32 *words, int num_words, u32 *desc)
{
	int i;

	for (i = 0; i < num_words; i++)
		words[i] = desc[i];
}

static inline void knav_dma_set_pkt_info(u32 buff, u32 buff_len, u32 ndesc,
					 struct knav_dma_desc *desc)
{
	desc->buff_len = buff_len;
	desc->buff = buff;
	desc->next_desc = ndesc;
}

static inline void knav_dma_set_desc_info(u32 desc_info, u32 pkt_info,
					  struct knav_dma_desc *desc)
{
	desc->desc_info = desc_info;
	desc->packet_info = pkt_info;
}

static inline void knav_dma_set_pad_info(u32 pad0, u32 pad1,
					 struct knav_dma_desc *desc)
{
	desc->pad[0] = pad0;
	desc->pad[1] = pad1;
}

static inline void knav_dma_set_org_pkt_info(u32 buff, u32 buff_len,
					     struct knav_dma_desc *desc)
{
	desc->orig_buff = buff;
	desc->orig_len = buff_len;
}

static inline void knav_dma_set_words(u32 *words, int num_words, u32 *desc)
{
	int i;

	for (i = 0; i < num_words; i++)
		desc[i] = words[i];
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
