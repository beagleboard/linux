/*
 * Remote Processor Procedure Call Driver
 *
 * Copyright (C) 2012-2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name Texas Instruments nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _RPMSG_RPC_INTERNAL_H_
#define _RPMSG_RPC_INTERNAL_H_

#include <linux/cdev.h>
#include <linux/idr.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/skbuff.h>
#include <linux/rpmsg.h>

typedef u32 virt_addr_t;
typedef u32 dev_addr_t;

/**
 * struct rppc_device - The per-device (server) data
 * @cdev: character device
 * @dev: device
 * @rpdev: rpmsg channel device associated with the remote server
 * @instances: list of currently opened/connected instances
 * @lock: mutex for protection of device variables
 * @comp: completion signal used for unblocking users during a
 *	  remote processor recovery
 * @sig_attr: array of device attributes to use with the publishing of
 *	      function information in sysfs for all the functions
 *	      associated with this remote server device.
 * @signatures: function signatures for the functions published by this
 *		remote server device
 * @minor: minor number for the character device
 * @num_funcs: number of functions published by this remote server device
 * @cur_func: counter used while querying information for each function
 *	      associated with this remote server device
 * @desc: description of the exposed service
 *
 * A rppc_device indicates the base remote server device that supports the
 * execution of a bunch of remote functions. Each such remote server device
 * has an associated character device that is used by the userland apps to
 * connect to it, and request the execution of any of these remote functions.
 */
struct rppc_device {
	struct cdev cdev;
	struct device *dev;
	struct rpmsg_channel *rpdev;
	struct list_head instances;
	struct mutex lock; /* device state variables lock */
	struct completion comp;
	struct device_attribute *sig_attr;
	struct rppc_func_signature *signatures;
	unsigned int minor;
	u32 num_funcs;
	u32 cur_func;
	char desc[RPMSG_NAME_SIZE];
};

/**
 * struct rppc_instance - The per-instance data structure (per user)
 * @list: list node
 * @rppcdev: the rppc device (remote server instance) handle
 * @dev: local device reference pointer of the rppc device
 * @queue: queue of buffers waiting to be read by the user
 * @lock: mutex for protecting instance variables
 * @readq: wait queue of blocked user threads waiting to read data
 * @reply_arrived: signal for unblocking the user thread
 * @ept: rpmsg endpoint associated with the rppc device
 * @in_transition: flag for storing a pending connection request
 * @dst: destination end-point of the remote server instance
 * @state: state of the opened instance, see enum rppc_state
 * @dma_idr: idr structure storing the imported buffers
 * @msg_id: last/current active message id tagged on a message sent
 *	    to the remote processor
 * @fxn_list: list of functions published by the remote server instance
 *
 * This structure is created whenever the user opens the driver. The
 * various elements of the structure are used to store its state and
 * information about the remote server it is connected to.
 */
struct rppc_instance {
	struct list_head list;
	struct rppc_device *rppcdev;
	struct device *dev;
	struct sk_buff_head queue;
	struct mutex lock; /* instance state variables lock */
	wait_queue_head_t readq;
	struct completion reply_arrived;
	struct rpmsg_endpoint *ept;
	int in_transition;
	u32 dst;
	int state;
	struct idr dma_idr;
	u16 msg_id;
	struct list_head fxn_list;
};

/**
 * struct rppc_function_list - outstanding function descriptor
 * @list: list node
 * @function: current remote function descriptor
 * @msg_id: message id for the function invocation
 *
 * This structure is used for storing the information about outstanding
 * functions that the remote side is executing. This provides the host
 * side a means to track every outstanding function, and a means to process
 * the responses received from the remote processor.
 */
struct rppc_function_list {
	struct list_head list;
	struct rppc_function *function;
	u16 msg_id;
};

/**
 * struct rppc_dma_buf - a rppc dma_buf descriptor for buffers imported by rppc
 * @fd: file descriptor of a buffer used to import the dma_buf
 * @id: idr index value for this descriptor
 * @buf: imported dma_buf handle for the buffer
 * @attach: attachment structure returned by exporter upon attaching to
 *	    the buffer by the rppc driver
 * @sgt: the scatter-gather structure associated with @buf
 * @pa: the physical address associated with the imported buffer
 * @autoreg: mode of how the descriptor is created
 *
 * This structure is used for storing the information relevant to the imported
 * buffer. The rpmsg rpc driver acts as a proxy on behalf of the remote core
 * and attaches itself to the driver while the remote processor/accelerators are
 * operating on the buffer.
 */
struct rppc_dma_buf {
	int fd;
	int id;
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	phys_addr_t pa;
	int autoreg;
};

/**
 * enum rppc_msg_type - message types exchanged between host and remote server
 * @RPPC_MSGTYPE_DEVINFO_REQ: request remote server for channel information
 * @RPPC_MSGTYPE_DEVINFO_RESP: response message from remote server for a
 *			       request of type RPPC_MSGTYPE_DEVINFO_REQ
 * @RPPC_MSGTYPE_FUNCTION_QUERY: request remote server for information about a
 *				 specific function
 * @RPPC_MSGTYPE_FUNCTION_INFO: response message from remote server for a prior
 *				request of type RPPC_MSGTYPE_FUNCTION_QUERY
 * @RPPC_MSGTYPE_CREATE_REQ: request the remote server manager to create a new
 *			     remote server instance. No secondary data is
 *			     needed
 * @RPPC_MSGTYPE_CREATE_RESP: response message from remote server manager for a
 *			      request of type RPPC_MSGTYPE_CREATE_REQ. The
 *			      message contains the new endpoint address in the
 *			      rppc_instance_handle
 * @RPPC_MSGTYPE_DELETE_REQ: request the remote server manager to delete a
 *			     remote server instance
 * @RPPC_MSGTYPE_DELETE_RESP: response message from remote server manager to a
 *			      request of type RPPC_MSGTYPE_DELETE_REQ. The
 *			      message contains the old endpoint address in the
 *			      rppc_instance_handle
 * @RPPC_MSGTYPE_FUNCTION_CALL: request remote server to execute a specific
 *				function
 * @RPPC_MSGTYPE_FUNCTION_RET: response message carrying the return status of a
 *			       specific function execution
 * @RPPC_MSGTYPE_ERROR: an error response message sent by either the remote
 *			server manager or remote server instance while
 *			processing any request messages
 * @RPPC_MSGTYPE_MAX: limit value to define the maximum message type value
 *
 * Every message exchanged between the host-side and the remote-side is
 * identified through a message type defined in this enum. The message type
 * is specified through the msg_type field of the struct rppc_msg_header,
 * which is the common header for rppc messages.
 */
enum rppc_msg_type {
	RPPC_MSGTYPE_DEVINFO_REQ	= 0,
	RPPC_MSGTYPE_DEVINFO_RESP	= 1,
	RPPC_MSGTYPE_FUNCTION_QUERY	= 2,
	RPPC_MSGTYPE_FUNCTION_INFO	= 3,
	RPPC_MSGTYPE_CREATE_REQ		= 6,
	RPPC_MSGTYPE_CREATE_RESP	= 8,
	RPPC_MSGTYPE_DELETE_REQ		= 4,
	RPPC_MSGTYPE_DELETE_RESP	= 7,
	RPPC_MSGTYPE_FUNCTION_CALL	= 5,
	RPPC_MSGTYPE_FUNCTION_RET	= 9,
	RPPC_MSGTYPE_ERROR = 10,
	RPPC_MSGTYPE_MAX
};

/**
 * enum rppc_infotype - function information query type
 * @RPPC_INFOTYPE_FUNC_SIGNATURE: function signature
 * @RPPC_INFOTYPE_NUM_CALLS: the number of times a function has been invoked
 * @RPPC_INFOTYPE_MAX: limit value to define the maximum info type
 *
 * This enum is used for identifying the type of information queried
 * from the remote processor. Only RPPC_INFOTYPE_FUNC_SIGNATURE is
 * currently used.
 */
enum rppc_infotype {
	RPPC_INFOTYPE_FUNC_SIGNATURE = 1,
	RPPC_INFOTYPE_NUM_CALLS,
	RPPC_INFOTYPE_MAX
};

/**
 * struct rppc_instance_handle - rppc instance information
 * @endpoint_address: end-point address of the remote server instance
 * @status: status of the request
 *
 * This structure indicates the format of the message payload exchanged
 * between the host and the remote sides for messages pertaining to
 * creation and deletion of the remote server instances. This payload
 * is associated with messages of type RPPC_MSGTYPE_CREATE_RESP and
 * RPPC_MSGTYPE_DELETE_RESP.
 */
struct rppc_instance_handle {
	u32 endpoint_address;
	u32 status;
} __packed;

/**
 * struct rppc_param_signature - parameter descriptor
 * @direction: input or output classifier, see enum rppc_param_direction
 * @type: parameter data type, see enum rppc_param_type
 * @count: used to do some basic sanity checking on array bounds
 */
struct rppc_param_signature {
	u32 direction;
	u32 type;
	u32 count;
};

/**
 * struct rppc_func_signature - remote function signature descriptor
 * @name: name of the function
 * @num_param: number of parameters to the function
 * @params: parameter descriptors for each of the parameters
 *
 * This structure contains the indicates the format of the message payload
 * exchanged between the host and the remote sides for messages pertaining
 * to creation and deletion of the remote server instances. This payload
 * is associated with messages of type RPPC_MSGTYPE_CREATE_RESP and
 * RPPC_MSGTYPE_FUNCTION_INFO.
 */
struct rppc_func_signature {
	char name[RPPC_MAX_CHANNEL_NAMELEN];
	u32 num_param;
	struct rppc_param_signature params[RPPC_MAX_NUM_PARAMS + 1];
};

/**
 * struct rppc_query_function - function info packet structure
 * @info_type: type of the function information requested, see
 *	       enum rppc_infotype
 * @fxn_id: function identifier on this specific remote server instance
 * @num_calls: number of types function is invoked, filled in during a response
 *	       (only valid for rppc_infotype RPPC_INFOTYPE_NUM_CALLS)
 * @signature: the signature of the function including its return type,
 *	       parameters and their description
 *	       (only valid for rppc_infotype RPPC_INFOTYPE_FUNC_SIGNATURE)
 *
 * This structure indicates the format of the message payload exchanged
 * between the host and the remote sides for messages pertaining to
 * information about each function supported by the remote server instance.
 * This payload is associated with messages of type RPPC_MSGTYPE_FUNCTION_QUERY
 * and RPPC_MSGTYPE_FUNCTION_INFO.
 */
struct rppc_query_function {
	u32 info_type;
	u32 fxn_id;
	union {
		u32 num_calls;
		struct rppc_func_signature signature;
	} info;
};

/**
 * enum rppc_translate_direction - pointer translation direction
 * @RPPC_UVA_TO_RPA: user virtual address to remote device address translation
 * @RPPC_RPA_TO_UVA: remote device address to user virtual address translation
 *
 * An enum used for identifying the rppc function message direction, whether
 * it is going to the remote side, or is a response from the remote side. This
 * is used in translating the pointers from the host-side to the remote-side
 * and vice versa depending on the packet direction.
 */
enum rppc_translate_direction {
	RPPC_UVA_TO_RPA,
	RPPC_RPA_TO_UVA,
};

/**
 * enum rppc_state - rppc instance state
 * @RPPC_STATE_DISCONNECTED: uninitialized state
 * @RPPC_STATE_CONNECTED: initialized state
 * @RPPC_STATE_STALE: invalid or stale state
 * @RPPC_STATE_MAX: limit value for the different state values
 *
 * This enum value is used to define the status values of a
 * rppc_instance object.
 */
enum rppc_state {
	RPPC_STATE_DISCONNECTED,
	RPPC_STATE_CONNECTED,
	RPPC_STATE_STALE,
	RPPC_STATE_MAX
};

/**
 * struct rppc_device_info - rppc remote server device info
 * @num_funcs: number of functions supported by a remote server instance
 *
 * This structure indicates the format of the message payload responded by
 * the remote side upon a request for message type RPPC_MSGTYPE_DEVINFO_REQ.
 * This payload is associated with messages of type RPPC_MSGTYPE_DEVINFO_RESP.
 */
struct rppc_device_info {
	u32 num_funcs;
};

/**
 * struct rppc_error - rppc error information
 * @endpoint_address: end-point address of the remote server instance
 * @status: status of the request
 *
 * This structure indicates the format of the message payload exchanged
 * between the host and the remote sides for error messages. This payload
 * is associated with messages of type RPPC_MSGTYPE_ERROR
 * XXX: check if this is needed still, not used anywhere at present
 */
struct rppc_error {
	u32 endpoint_address;
	u32 status;
} __packed;

/**
 * struct rppc_param_data - marshalled parameter data structure
 * @size: size of the parameter data type
 * @data: actual parameter data
 *
 * Each function parameter is marshalled in this form between the host
 * and remote sides. The @data field would contain the actual value of
 * of the parameter if it is a scalar argument type, or the remote-side
 * device address (virtual address) of the pointer if the argument is
 * of pointer type.
 */
struct rppc_param_data {
	size_t size;
	size_t data;
} __packed;

/**
 * struct rppc_msg_header - generic rpmsg rpc message header
 * @msg_type: message type, see enum rppc_msg_type
 * @msg_len: length of the message payload in bytes
 * @msg_data: the actual message payload (depends on message type)
 *
 * All RPPC messages will start with this common header (which will begin
 * right after the standard rpmsg header ends).
 */
struct rppc_msg_header {
	u32 msg_type;
	u32 msg_len;
	u8  msg_data[0];
} __packed;

#define RPPC_PAYLOAD(ptr, type)	\
		((struct type *)&(ptr)[sizeof(struct rppc_msg_header)])

/* from rpmsg_rpc.c */
dev_addr_t rppc_local_to_remote_da(struct rppc_instance *rpc, phys_addr_t pa);

/* from rpmsg_rpc_dmabuf.c */
struct rppc_dma_buf *rppc_alloc_dmabuf(struct rppc_instance *rpc,
				       int fd, bool autoreg);
struct rppc_dma_buf *rppc_find_dmabuf(struct rppc_instance *rpc, int fd);
int rppc_free_dmabuf(int id, void *p, void *data);
dev_addr_t rppc_buffer_lookup(struct rppc_instance *rpc, virt_addr_t uva,
			      virt_addr_t buva, int fd);
int rppc_xlate_buffers(struct rppc_instance *rpc, struct rppc_function *func,
		       int direction);

/* from rpmsg_rpc_sysfs.c */
int rppc_create_sysfs(struct rppc_device *rppcdev);
int rppc_remove_sysfs(struct rppc_device *rppcdev);

#endif
