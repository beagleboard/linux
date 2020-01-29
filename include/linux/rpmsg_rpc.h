/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause) */
/*
 * Remote Processor Procedure Call Driver
 *
 * Copyright (C) 2012-2020 Texas Instruments Incorporated - http://www.ti.com/
 */

#ifndef _LINUX_RPMSG_RPC_H_
#define _LINUX_RPMSG_RPC_H_

#include <uapi/linux/rpmsg_rpc.h>

#define RPPC_MAX_NUM_FUNCS		(1024)
#define RPPC_MAX_CHANNEL_NAMELEN	(64)
#define RPPC_MAX_FUNC_NAMELEN		(64)
#define RPPC_MAX_NUM_PARAMS		(10)

/**
 * enum rppc_param_direction - direction of the function parameter
 * @RPPC_PARAMDIR_IN: input argument
 * @RPPC_PARAMDIR_OUT: output argument
 * @RPPC_PARAMDIR_BI: an in and out argument
 * @RPPC_PARAMDIR_MAX: limit value for the direction type
 *
 * The parameter direction is described as relative to the function.
 */
enum rppc_param_direction {
	RPPC_PARAMDIR_IN = 0,
	RPPC_PARAMDIR_OUT,
	RPPC_PARAMDIR_BI,
	RPPC_PARAMDIR_MAX
};

/**
 * enum rppc_param_datatype - parameter data type and descriptor flags
 * @RPPC_PARAM_VOID: parameter is of type 'void'
 * @RPPC_PARAM_S08: parameter is of type 's8'
 * @RPPC_PARAM_U08: parameter is of type 'u8'
 * @RPPC_PARAM_S16: parameter is of type 's16'
 * @RPPC_PARAM_U16: parameter is of type 'u16'
 * @RPPC_PARAM_S32: parameter is of type 's32'
 * @RPPC_PARAM_U32: parameter is of type 'u32'
 * @RPPC_PARAM_S64: parameter is of type 's64'
 * @RPPC_PARAM_U64: parameter is of type 'u64'
 * @RPPC_PARAM_ATOMIC_MAX: limit value for scalar data types
 * @RPPC_PARAM_MASK: mask field for retrieving the scalar data type
 * @RPPC_PARAM_PTR: flag to indicate the data type is a pointer
 * @RPPC_PARAM_MAX: max limit value used as a marker
 *
 * This enum is used to describe the data type for the parameters.
 * A pointer of a data type is reflected by using an additional bit
 * mask field.
 */
enum rppc_param_datatype {
	RPPC_PARAM_VOID = 0,
	RPPC_PARAM_S08,
	RPPC_PARAM_U08,
	RPPC_PARAM_S16,
	RPPC_PARAM_U16,
	RPPC_PARAM_S32,
	RPPC_PARAM_U32,
	RPPC_PARAM_S64,
	RPPC_PARAM_U64,
	RPPC_PARAM_ATOMIC_MAX,

	RPPC_PARAM_MASK = 0x7F,
	RPPC_PARAM_PTR = 0x80,

	RPPC_PARAM_MAX
};

/*
 * helper macros to deal with parameter types
 */
#define RPPC_PTR_TYPE(type)	((type) | RPPC_PARAM_PTR)
#define RPPC_IS_PTR(type)	((type) & RPPC_PARAM_PTR)
#define RPPC_IS_ATOMIC(type)	(((type) > RPPC_PARAM_VOID) && \
				 ((type) < RPPC_PARAM_ATOMIC_MAX))

#endif /* _LINUX_RPMSG_RPC_H_ */
