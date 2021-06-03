/**
 * @note Copyright (C) 2016 Philippe Gerum <rpm@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _RTDM_UAPI_SPI_H
#define _RTDM_UAPI_SPI_H

#include <linux/types.h>

struct rtdm_spi_config {
	__u32 speed_hz;
	__u16 mode;
	__u8 bits_per_word;
};

struct rtdm_spi_iobufs {
	__u32 io_len;
	__u32 i_offset;
	__u32 o_offset;
	__u32 map_len;
};

#define SPI_RTIOC_SET_CONFIG		_IOW(RTDM_CLASS_SPI, 0, struct rtdm_spi_config)
#define SPI_RTIOC_GET_CONFIG		_IOR(RTDM_CLASS_SPI, 1, struct rtdm_spi_config)
#define SPI_RTIOC_SET_IOBUFS		_IOR(RTDM_CLASS_SPI, 2, struct rtdm_spi_iobufs)
#define SPI_RTIOC_TRANSFER		_IO(RTDM_CLASS_SPI, 3)
#define SPI_RTIOC_TRANSFER_N		_IOR(RTDM_CLASS_SPI, 4, int)

#endif /* !_RTDM_UAPI_SPI_H */
