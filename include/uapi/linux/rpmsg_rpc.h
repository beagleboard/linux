/* SPDX-License-Identifier: ((GPL-2.0-only WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * Remote Processor Procedure Call Driver
 *
 * Copyright (C) 2012-2020 Texas Instruments Incorporated - http://www.ti.com/
 */

#ifndef _UAPI_LINUX_RPMSG_RPC_H_
#define _UAPI_LINUX_RPMSG_RPC_H_

#include <linux/ioctl.h>

/**
 * struct rppc_buf_fds - rppc buffer registration/deregistration
 * @num: number of file descriptors
 * @fds: pointer to the array holding the file descriptors
 */
struct rppc_buf_fds {
	uint32_t num;
	int32_t *fds;
};

/*
 * ioctl definitions
 */
#define RPPC_IOC_MAGIC		'r'
#define RPPC_IOC_CREATE		_IOW(RPPC_IOC_MAGIC, 1, char *)
#define RPPC_IOC_BUFREGISTER    _IOW(RPPC_IOC_MAGIC, 2, struct rppc_buf_fds)
#define RPPC_IOC_BUFUNREGISTER  _IOW(RPPC_IOC_MAGIC, 3, struct rppc_buf_fds)
#define RPPC_IOC_MAXNR		(4)

#define RPPC_MAX_PARAMETERS	(10)
#define RPPC_MAX_TRANSLATIONS	(1024)
#define RPPC_MAX_INST_NAMELEN	(48)

/**
 * enum rppc_param_type - RPC function parameter type
 * @RPPC_PARAM_TYPE_UNKNOWN: unrecognized parameter
 * @RPPC_PARAM_TYPE_ATOMIC: an atomic data type, 1 byte to architecture limit
 *			    sized bytes
 * @RPPC_PARAM_TYPE_PTR: a pointer to shared memory. The fd field in the
 *			 structures rppc_param and rppc_param_translation must
 *			 contain the file descriptor of the associated dma_buf
 * @RPPC_PARAM_TYPE_STRUCT: (unsupported) a structure type. Will be architecture
 *			    width aligned in memory
 *
 * These enum values are used to identify the parameter type for every
 * parameter argument of the remote function.
 */
enum rppc_param_type {
	RPPC_PARAM_TYPE_UNKNOWN = 0,
	RPPC_PARAM_TYPE_ATOMIC,
	RPPC_PARAM_TYPE_PTR,
	RPPC_PARAM_TYPE_STRUCT,
};

/**
 * struct rppc_param_translation - pointer translation helper structure
 * @index: index of the parameter where the translation needs to be done in.
 *	   used for computing the primary offset and mapping into kernel
 *	   the page from the buffer referred to in the corresponding parameter
 * @offset: offset from the primary base pointer to the pointer to translate.
 *	    This is the secondary offset, and used either for mentioning the
 *	    offset from an structure array element base, or within a single
 *	    structure which itself is at an offset in an allocated buffer
 * @base: the base user virtual address of the pointer to translate (used to
 *	  calculate translated pointer offset)
 * @fd: dma_buf file descriptor of the allocated buffer pointer within which
 *	the translated pointer is present
 */
struct rppc_param_translation {
	uint32_t index;
	ptrdiff_t offset;
	size_t base;
	int32_t fd;
};

/**
 * struct rppc_param - descriptor structure for each parameter
 * @type: type of the parameter, as dictated by enum rppc_param_type
 * @size: size of the data (for atomic types) or size of the containing
 *	  structure in which translations are performed
 * @data: either the parameter value itself (for atomic type) or
 *	  the actual user space pointer address to the data (for pointer type)
 * @base: the base user space pointer address of the original allocated buffer,
 *	  providing a reference if data has the pointer that is at an offset
 *	  from the original pointer
 * @fd: file descriptor of the exported allocation (will be used to
 *	import the associated dma_buf within the driver).
 */
struct rppc_param {
	uint32_t type;
	size_t size;
	size_t data;
	size_t base;
	int32_t fd;
};

/**
 * struct rppc_function - descriptor structure for the remote function
 * @fxn_id: index of the function to invoke on the opened rppc device
 * @num_params: number of parameters filled in the params field
 * @params: array of parameter descriptor structures
 * @num_translations: number of in-place translations to be performed within
 *		      the arguments.
 * @translations: an open array of the translation descriptor structures, whose
 *		  length is given in @num_translations. Used for translating
 *		  the pointers within the function data.
 *
 * This is the primary descriptor structure passed down from the userspace,
 * describing the function, its parameter arguments and the needed translations.
 */
struct rppc_function {
	uint32_t fxn_id;
	uint32_t num_params;
	struct rppc_param params[RPPC_MAX_PARAMETERS];
	uint32_t num_translations;
	struct rppc_param_translation translations[0];
};

/**
 * struct rppc_function_return - function return status descriptor structure
 * @fxn_id: index of the function invoked on the opened rppc device
 * @status: return value of the executed function
 */
struct rppc_function_return {
	uint32_t fxn_id;
	uint32_t status;
};

/**
 * struct rppc_create_instance - rppc channel connector helper
 * @name: Name of the rppc server device to establish a connection with
 */
struct rppc_create_instance {
	char name[RPPC_MAX_INST_NAMELEN];
};

/*
 * helper macros for manipulating the function index in the marshalled packet
 */
#define RPPC_DESC_EXEC_SYNC	(0x0100)
#define RPPC_DESC_TYPE_MASK	(0x0F00)

/*
 * helper macros for manipulating the function index in the marshalled packet.
 * The remote functions are offset by one relative to the client
 * XXX: Remove the relative offset
 */
#define RPPC_SET_FXN_IDX(idx)	(((idx) + 1) | 0x80000000)
#define RPPC_FXN_MASK(idx)	(((idx) - 1) & 0x7FFFFFFF)

/**
 * struct rppc_packet - the actual marshalled packet
 * @desc: type of function execution, currently only synchronous function
 *	  invocations are supported
 * @msg_id: an incremental message index identifier
 * @flags: a combination of job id and pool id of the worker threads
 *	   of the server
 * @fxn_id: id of the function to execute
 * @result: result of the remotely executed function
 * @data_size: size of the payload packet
 * @data: variable payload, containing the marshalled function data.
 *
 * This is actually a condensed structure of the Remote Command Messaging
 * (RCM) structure. The initial fields of the structure are used by the
 * remote-side server to schedule the execution of the function. The actual
 * variable payload data starts from the .data field. This marshalled packet
 * is the payload for a rpmsg message.
 *
 * XXX: remove or mask unneeded fields, some fields can be stripped down
 */
struct rppc_packet {
	uint16_t desc;
	uint16_t msg_id;
	uint32_t flags;
	uint32_t fxn_id;
	int32_t  result;
	uint32_t data_size;
	uint8_t  data[0];
} __packed;

#endif /* _UAPI_LINUX_RPMSG_RPC_H_ */
