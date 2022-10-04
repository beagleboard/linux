/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * IMPORTANT: The following constants must match the ones used and defined in
 * external/qemu/include/hw/misc/goldfish_pipe.h
 */

#ifndef GOLDFISH_PIPE_QEMU_H
#define GOLDFISH_PIPE_QEMU_H

/* List of bitflags returned in status of CMD_POLL command */
enum PipePollFlags {
	PIPE_POLL_IN	= 1 << 0,
	PIPE_POLL_OUT	= 1 << 1,
	PIPE_POLL_HUP	= 1 << 2
};

/* Possible status values used to signal errors */
enum PipeErrors {
	PIPE_ERROR_INVAL	= -1,
	PIPE_ERROR_AGAIN	= -2,
	PIPE_ERROR_NOMEM	= -3,
	PIPE_ERROR_IO		= -4
};

/* Bit-flags used to signal events from the emulator */
enum PipeWakeFlags {
	/* emulator closed pipe */
	PIPE_WAKE_CLOSED		= 1 << 0,

	/* pipe can now be read from */
	PIPE_WAKE_READ			= 1 << 1,

	/* pipe can now be written to */
	PIPE_WAKE_WRITE			= 1 << 2,

	/* unlock this pipe's DMA buffer */
	PIPE_WAKE_UNLOCK_DMA		= 1 << 3,

	/* unlock DMA buffer of the pipe shared to this pipe */
	PIPE_WAKE_UNLOCK_DMA_SHARED	= 1 << 4,
};

/* Possible pipe closing reasons */
enum PipeCloseReason {
	/* guest sent a close command */
	PIPE_CLOSE_GRACEFUL		= 0,

	/* guest rebooted, we're closing the pipes */
	PIPE_CLOSE_REBOOT		= 1,

	/* close old pipes on snapshot load */
	PIPE_CLOSE_LOAD_SNAPSHOT	= 2,

	/* some unrecoverable error on the pipe */
	PIPE_CLOSE_ERROR		= 3,
};

/* Bit flags for the 'flags' field */
enum PipeFlagsBits {
	BIT_CLOSED_ON_HOST = 0,  /* pipe closed by host */
	BIT_WAKE_ON_WRITE  = 1,  /* want to be woken on writes */
	BIT_WAKE_ON_READ   = 2,  /* want to be woken on reads */
};

enum PipeV1Regs {
	/* write: value = command */
	PIPE_V1_REG_COMMAND		= 0x00,
	/* read */
	PIPE_V1_REG_STATUS		= 0x04,
	/* read/write: channel id */
	PIPE_V1_REG_CHANNEL		= 0x08,
	/* read/write: channel id */
	PIPE_V1_REG_CHANNEL_HIGH	= 0x30,
	/* read/write: buffer size */
	PIPE_V1_REG_SIZE		= 0x0C,
	/* write: physical address */
	PIPE_V1_REG_ADDRESS		= 0x10,
	/* write: physical address */
	PIPE_V1_REG_ADDRESS_HIGH	= 0x34,
	/* read: wake flags */
	PIPE_V1_REG_WAKES		= 0x14,
	/* read/write: batch data address */
	PIPE_V1_REG_PARAMS_ADDR_LOW	= 0x18,
	/* read/write: batch data address */
	PIPE_V1_REG_PARAMS_ADDR_HIGH	= 0x1C,
	/* write: batch access */
	PIPE_V1_REG_ACCESS_PARAMS	= 0x20,
	/* read: device version */
	PIPE_V1_REG_VERSION		= 0x24,
};

enum PipeV2Regs {
	PIPE_V2_REG_CMD = 0,

	PIPE_V2_REG_SIGNAL_BUFFER_HIGH = 4,
	PIPE_V2_REG_SIGNAL_BUFFER = 8,
	PIPE_V2_REG_SIGNAL_BUFFER_COUNT = 12,

	PIPE_V2_REG_OPEN_BUFFER_HIGH = 20,
	PIPE_V2_REG_OPEN_BUFFER = 24,

	PIPE_V2_REG_VERSION = 36,

	PIPE_V2_REG_GET_SIGNALLED = 48,
};

enum PipeCmdCode {
	/* to be used by the pipe device itself */
	PIPE_CMD_OPEN		= 1,

	PIPE_CMD_CLOSE,
	PIPE_CMD_POLL,
	PIPE_CMD_WRITE,
	PIPE_CMD_WAKE_ON_WRITE,
	PIPE_CMD_READ,
	PIPE_CMD_WAKE_ON_READ,

	/*
	 * TODO(zyy): implement a deferred read/write execution to allow
	 * parallel processing of pipe operations on the host.
	 */
	PIPE_CMD_WAKE_ON_DONE_IO,
	PIPE_CMD_DMA_HOST_MAP,
	PIPE_CMD_DMA_HOST_UNMAP,
};

#endif /* GOLDFISH_PIPE_QEMU_H */
