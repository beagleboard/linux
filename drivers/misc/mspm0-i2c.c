// SPDX-License-Identifier: GPL-2.0
/*
 * MSPM0 I2C Driver
 *
 * Copyright (c) 2023 Ayush Singh <ayush@beagleboard.org>
 * Copyright (c) 2023 BeagleBoard.org Foundation
 */

#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/firmware.h>
#include <linux/byteorder/generic.h>
#include <linux/crc32.h>
#include <linux/gpio.h>

#define MSPM0_BSL_PASSWORD                                                  \
	{ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, \
	  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, \
	  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }
#define MSPM0_BSL_DATA_LEN(pkt_len) \
	(pkt_len - sizeof(struct bsl_no_data_req_pkt) + sizeof(u8))

/**
 * struct mspm0_i2c: MSPM0 I2C Driver
 *
 * @client: i2c client
 *
 * @fwl: underlying firmware upload device
 * @fwl_size: firmware size
 * @fwl_crc: firmware crc32
 * @bootloader_backdoor_gpio: cc1352p7 boot gpio
 * @rst_gpio: cc1352p7 reset gpio
 * @bsl_max_buffer_size: Max buffer size supported
 */
struct mspm0_i2c {
	struct i2c_client *client;

	struct fw_upload *fwl;
	u32 fwl_size;
	u32 fwl_crc;
	struct gpio_desc *bsl_backdoor_gpio;
	struct gpio_desc *rst_gpio;
	u16 bsl_max_buffer_size;
};

/**
 * enum mspm0_bsl_cmd: MSPM0 Bootloader Commands
 *
 * @COMMAND_CONNECTION: Establish connection between host and target
 * @COMMAND_MASS_ERASE: Erases the main application memory
 * @COMMAND_GET_DEVICE_INFO: Get the version information and buffer size 
 *                           available for data transaction
 * @COMMAND_PROGRAM_DATA: Write data starting from specific address
 * @COMMAND_UNLOCK_BOOTLOADER: Unlock the bootloader
 * @COMMAND_STANDALONE_VERIFICATION: Verify CRC for data in given memory range
 * @COMMAND_START_APPLICATION: Start the application
 */
enum mspm0_bsl_cmd {
	COMMAND_CONNECTION = 0x12,
	COMMAND_MASS_ERASE = 0x15,
	COMMAND_GET_DEVICE_INFO = 0x19,
	COMMAND_PROGRAM_DATA = 0x20,
	COMMAND_UNLOCK_BOOTLOADER = 0x21,
	COMMAND_STANDALONE_VERIFICATION = 0x26,
	COMMAND_START_APPLICATION = 0x40,
};

/**
 * enum mspm0_bsl_ack: BSL Acknowledgment
 *
 * @BSL_ACK: Packet received successfully
 */
enum mspm0_bsl_ack {
	BSL_ACK = 0x00,
	BSL_ERROR_HEADER_INCORRECT = 0x51,
	BSL_ERROR_CHECKSUM_INCORRECT = 0x52,
	BSL_ERROR_PACKET_SIZE_ZERO = 0x53,
	BSL_ERROR_PACKET_SIZE_TOO_BIG = 0x54,
	BSL_ERROR_UNKNOWN_ERROR = 0x55
};

/**
 * enum mspm0_bsl_header: BSL packet headers
 */
enum mspm0_bsl_header { REQUEST = 0x80, RESPONSE = 0x08 };

/**
 * enum mspm0_bsl_resp: BSL response byte
 */
enum mspm0_bsl_resp { CORE_MESSAGE = 0x3b, DETAILED_MESSAGE = 0x3a };

/**
 * enum mspm0_bsl_core_resp_msg: BSL core message response msg byte
 *
 * @OPERATION_SUCCESSFUL: Operation Successful
 * @BSL_LOCKED_ERROR: The BSL is not yet unlocked with Bootloader unlock
 *                    password command or After BSL unlock, a timeout would
 *                    have occurred in the command reception phase
 * @BSL_PASSWORD_ERROR: Incorrect password has been sent to unlock bootloader
 * @MULTIPLE_BSL_PASSWORD_ERROR: Incorrect password has been sent to unlock
 *                               bootloader 3 times
 * @UNKNOWN_COMMAND_ERROR: The command given to the BSL was not recognized as
 *                         valid command
 * @INVALID_MEMORY_RANGE: The given memory range is invalid
 * @INVALID_COMMAND: The command given to the BSL is a known command but it is
 *                   invalid at that time instant and cannot be processed
 * @FACTORY_RESET_DISABLED: Factory reset is disabled in the BCR configuration
 * @FACTORY_RESET_PASSWORD_ERROR: Incorrect or no password has been sent with
 *                                factory reset command, when the BCR 
 *                                configuration has factory reset 'Enabled with
 *                                password'
 * @READ_OUT_ERROR: Memory read out be disabled in BCR configuration
 * @INVALID_ALIGNMENT: Start address or data length for the flash programming 
 *                     is not 8-byte aligned
 * @INVALID_LENGTH: Data size sent for standalone verification is less than 1KB
 */
enum mspm0_bsl_core_resp_msg {
	OPERATION_SUCCESSFUL = 0x00,
	BSL_LOCKED_ERROR = 0x01,
	BSL_PASSWORD_ERROR = 0x02,
	MULTIPLE_BSL_PASSWORD_ERROR = 0x03,
	UNKNOWN_COMMAND_ERROR = 0x04,
	INVALID_MEMORY_RANGE = 0x05,
	INVALID_COMMAND = 0x06,
	FACTORY_RESET_DISABLED = 0x07,
	FACTORY_RESET_PASSWORD_ERROR = 0x08,
	READ_OUT_ERROR = 0x09,
	INVALID_ALIGNMENT = 0x0a,
	INVALID_LENGTH = 0x0b
};

/**
 * struct bsl_pkt_head
 *
 * @header: packet header
 * @len: length of data bytes (including command byte)
 * @cmd: BSL command
 */
struct bsl_pkt_head {
	u8 header;
	__le16 len;
	u8 cmd;
} __packed;

typedef __le32 bsl_pkt_crc32;

struct bsl_no_data_req_pkt {
	struct bsl_pkt_head head;
	bsl_pkt_crc32 tail;
} __packed;

struct bsl_device_info_resp_pkt {
	struct bsl_pkt_head head;
	__le16 cmd_interpreter_version;
	__le16 build_id;
	__le32 application_version;
	__le16 active_plugin_version;
	__le16 bsl_max_buffer_size;
	__le32 bsl_buffer_start_address;
	__le32 bcr_configuration_id;
	__le32 bsl_configuration_id;
	bsl_pkt_crc32 tail;
} __packed;

struct bsl_core_resp {
	struct bsl_pkt_head head;
	u8 msg;
	bsl_pkt_crc32 tail;
} __packed;

struct bsl_unlock_bsl_req_pkt {
	struct bsl_pkt_head head;
	u8 password[32];
	bsl_pkt_crc32 tail;
} __packed;

struct bsl_standalone_verification_req_pkt {
	struct bsl_pkt_head head;
	__le32 address;
	__le32 size;
	bsl_pkt_crc32 tail;
} __packed;

struct bsl_standalone_verification_resp_pkt {
	struct bsl_pkt_head head;
	__le32 crc;
	bsl_pkt_crc32 tail;
} __packed;

struct bsl_program_data_req_head_pkt {
	struct bsl_pkt_head head;
	__le32 address;
} __packed;

static enum fw_upload_err i2c_to_fw_err(int err)
{
	switch (err) {
	case -ETIMEDOUT:
		return FW_UPLOAD_ERR_TIMEOUT;
	case -EBUSY:
		return FW_UPLOAD_ERR_BUSY;
	default:
		return FW_UPLOAD_ERR_HW_ERROR;
	}
}

static enum fw_upload_err bsl_tx(struct i2c_client *client, const u8 data[],
				 size_t len)
{
	int ret = i2c_master_send(client, data, len);
	if (ret < 0)
		return i2c_to_fw_err(ret);

	if (ret != len)
		return FW_UPLOAD_ERR_RW_ERROR;

	return FW_UPLOAD_ERR_NONE;
}

static enum fw_upload_err bsl_rx(struct i2c_client *client, u8 data[],
				 size_t len)
{
	int ret = i2c_master_recv(client, data, len);
	if (ret < 0)
		return i2c_to_fw_err(ret);

	if (ret != len)
		return FW_UPLOAD_ERR_RW_ERROR;

	return FW_UPLOAD_ERR_NONE;
}

static enum fw_upload_err bsl_get_ack(struct i2c_client *client)
{
	enum fw_upload_err ret;
	u8 ack_byte;

	ret = bsl_rx(client, &ack_byte, sizeof(ack_byte));
	if (ret != FW_UPLOAD_ERR_NONE)
		return ret;

	switch (ack_byte) {
	case BSL_ACK:
		return FW_UPLOAD_ERR_NONE;
	case BSL_ERROR_PACKET_SIZE_ZERO:
	case BSL_ERROR_PACKET_SIZE_TOO_BIG:
		return FW_UPLOAD_ERR_INVALID_SIZE;
	default:
		return FW_UPLOAD_ERR_RW_ERROR;
	}
}

static struct bsl_no_data_req_pkt bsl_no_data_req_pkt_create(u8 cmd)
{
	const struct bsl_no_data_req_pkt pkt = {
		.head = {
			.header = REQUEST,
			.len = cpu_to_le16(sizeof(cmd)),
			.cmd = cmd,
		},
		.tail = crc32(0xffffffff, &cmd, sizeof(cmd))
	};

	return pkt;
}

static bool bsl_resp_pkt_validate(const struct bsl_pkt_head *pkt,
				  size_t pkt_len)
{
	u16 len;
	u32 calculated_crc, sent_crc;
	const bsl_pkt_crc32 *tail;
	const struct bsl_core_resp *msg;

	if (pkt->header != RESPONSE)
		return false;

	len = le16_to_cpu(pkt->len);
	if (len != MSPM0_BSL_DATA_LEN(pkt_len))
		return false;

	calculated_crc = crc32(0xffffffff, &pkt->cmd, len);
	tail = (const bsl_pkt_crc32 *)((const u8 *)&pkt->cmd + len);
	sent_crc = le32_to_cpu(*tail);

	if (calculated_crc != sent_crc)
		return false;

	if (pkt->cmd == CORE_MESSAGE) {
		msg = (const struct bsl_core_resp *)pkt;
		return msg->msg == OPERATION_SUCCESSFUL;
	}

	return true;
}

static enum fw_upload_err bsl_connect(struct mspm0_i2c *mspm0)
{
	enum fw_upload_err ret;
	const struct bsl_no_data_req_pkt pkt =
		bsl_no_data_req_pkt_create(COMMAND_CONNECTION);

	dev_info(&mspm0->client->dev, "Establishing Connection");

	ret = bsl_tx(mspm0->client, (const u8 *)&pkt, sizeof(pkt));
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(&mspm0->client->dev, ret,
				     "Failed to send connection command");

	return bsl_get_ack(mspm0->client);
}

static enum fw_upload_err bsl_get_device_info(struct mspm0_i2c *mspm0)
{
	enum fw_upload_err ret;
	struct bsl_device_info_resp_pkt response;
	struct device *dev = &mspm0->client->dev;
	const struct bsl_no_data_req_pkt pkt =
		bsl_no_data_req_pkt_create(COMMAND_GET_DEVICE_INFO);

	dev_info(dev, "Getting Device Info");

	ret = bsl_tx(mspm0->client, (const u8 *)&pkt, sizeof(pkt));
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(dev, ret,
				     "Failed to send get_device_info command");

	ret = bsl_get_ack(mspm0->client);
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(dev, ret, "Failed to get device info");

	ret = bsl_rx(mspm0->client, (u8 *)&response, sizeof(response));
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(dev, ret,
				     "Failed to get device info response");

	if (!bsl_resp_pkt_validate(&response.head, sizeof(response)))
		return dev_err_probe(dev, FW_UPLOAD_ERR_RW_ERROR,
				     "Invalid response");

	mspm0->bsl_max_buffer_size = le16_to_cpu(response.bsl_max_buffer_size);

	dev_info(dev, "MSPM0 BSL Device Info");
	dev_info(dev, "Command Interpreter Version: %u",
		 le16_to_cpu(response.cmd_interpreter_version));
	dev_info(dev, "Build ID: %u", le16_to_cpu(response.build_id));
	dev_info(dev, "Application Version: %u",
		 le32_to_cpu(response.application_version));
	dev_info(dev, "Active Plug-in interface version: %u",
		 le16_to_cpu(response.active_plugin_version));
	dev_info(dev, "BSL Max buffer size: %u", mspm0->bsl_max_buffer_size);
	dev_info(dev, "BSL Buffer Start address: %u",
		 le32_to_cpu(response.bsl_buffer_start_address));
	dev_info(dev, "BCR Configuration ID: %u",
		 le32_to_cpu(response.bcr_configuration_id));
	dev_info(dev, "BSL Configuration ID: %u",
		 le32_to_cpu(response.bsl_configuration_id));

	return FW_UPLOAD_ERR_NONE;
}

static enum fw_upload_err bsl_unlock_bsl(struct mspm0_i2c *mspm0)
{
	enum fw_upload_err ret;
	u32 crc;
	struct bsl_core_resp resp;
	const struct device *dev = &mspm0->client->dev;
	struct bsl_unlock_bsl_req_pkt
		pkt = { .head = {
				.header = 0x80,
				.len = cpu_to_le16(MSPM0_BSL_DATA_LEN(sizeof(pkt))),
				.cmd = COMMAND_UNLOCK_BOOTLOADER,
			},
		    .password = MSPM0_BSL_PASSWORD,
		};

	dev_info(dev, "Unlocking Bootloader");

	crc = crc32(0xffffffff, (const u8 *)&pkt.head.cmd,
		    MSPM0_BSL_DATA_LEN(sizeof(pkt)));
	pkt.tail = cpu_to_le32(crc);

	ret = bsl_tx(mspm0->client, (const u8 *)&pkt, sizeof(pkt));
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(
			dev, ret, "Failed to send unlock_bootloader request");

	ret = bsl_get_ack(mspm0->client);
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(dev, ret, "Failed to unlock bootloader");

	ret = bsl_rx(mspm0->client, (u8 *)&resp, sizeof(resp));
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(
			dev, ret, "Invalid Response for unlock bootloader 1");

	if (!bsl_resp_pkt_validate(&resp.head, sizeof(resp)))
		return dev_err_probe(
			dev, FW_UPLOAD_ERR_RW_ERROR,
			"Invalid Response for unlock bootloader 2");

	return FW_UPLOAD_ERR_NONE;
}

static enum fw_upload_err bsl_mass_erase(struct mspm0_i2c *mspm0)
{
	enum fw_upload_err ret;
	struct bsl_core_resp resp;
	const struct device *dev = &mspm0->client->dev;
	const struct bsl_no_data_req_pkt pkt =
		bsl_no_data_req_pkt_create(COMMAND_MASS_ERASE);

	dev_info(dev, "Perform Mass Erase");

	ret = bsl_tx(mspm0->client, (const u8 *)&pkt, sizeof(pkt));
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(dev, ret,
				     "Failed to send mass_erase command");

	ret = bsl_get_ack(mspm0->client);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to perform mass_erase");

	ret = bsl_rx(mspm0->client, (u8 *)&resp, sizeof(resp));
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(dev, ret,
				     "Invalid Response for mass_erase");

	if (!bsl_resp_pkt_validate(&resp.head, sizeof(resp)))
		return dev_err_probe(dev, FW_UPLOAD_ERR_RW_ERROR,
				     "Invalid Response for mass_erase");

	return 0;
}

/**
 * bsl_program_data_len(): calculate the length of data to send in 
 *                         COMMAND_PROGRAM_DATA
 *
 * @mspm0: i2c client
 * @len: length of buffer
 *
 * @returns max nuber of data bytes that can be sent
 *
 * Number of bytes should be 8 byte aligned
 */
static u16 bsl_program_data_len(struct mspm0_i2c *mspm0, size_t len)
{
	/* Absolute max len of data */
	u16 max_data_len = mspm0->bsl_max_buffer_size -
			   sizeof(struct bsl_no_data_req_pkt) - sizeof(__le32);
	/* BSL wants flashing adress and data to be 8 byte aligned */
	u16 aligned_data_len = max_data_len - (max_data_len % 8);

	return MIN(aligned_data_len, len);
}

static enum fw_upload_err bsl_program_data(struct mspm0_i2c *mspm0,
					   const u8 data[], size_t len,
					   u32 *written, u32 start_address)
{
	u32 crc;
	bsl_pkt_crc32 tail;
	enum fw_upload_err ret;
	struct bsl_core_resp resp;
	const struct device *dev = &mspm0->client->dev;
	u16 data_len = bsl_program_data_len(mspm0, len);
	const struct bsl_program_data_req_head_pkt head = {
		.head = {
		.header = REQUEST,
		.len = cpu_to_le16(data_len + sizeof(head.head.cmd) +
				   sizeof(head.address)),
		.cmd = COMMAND_PROGRAM_DATA,
		},
		.address = cpu_to_le32(start_address)
	};

	dev_info(dev, "Program %u bytes", data_len);

	crc = crc32(0xffffffff, &head.head.cmd,
		    sizeof(head.head.cmd) + sizeof(head.address));
	crc = crc32(crc, data, data_len);
	tail = cpu_to_le32(crc);

	ret = bsl_tx(mspm0->client, (const u8 *)&head, sizeof(head));
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(dev, ret,
				     "Failed to send program_data_fast head");

	ret = bsl_tx(mspm0->client, data, data_len);
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(dev, ret,
				     "Failed to send program_data data");

	ret = bsl_tx(mspm0->client, (const u8 *)&tail, sizeof(tail));
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(dev, ret,
				     "Failed to send program_data tail");

	ret = bsl_get_ack(mspm0->client);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to program data");

	ret = bsl_rx(mspm0->client, (u8 *)&resp, sizeof(resp));
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(dev, ret, "Invalid program_data response");

	if (!bsl_resp_pkt_validate(&resp.head, sizeof(resp)))
		return dev_err_probe(dev, FW_UPLOAD_ERR_RW_ERROR,
				     "Invalid program_data response 2");

	*written = data_len;

	return FW_UPLOAD_ERR_NONE;
}

static enum fw_upload_err bsl_start_application(struct mspm0_i2c *mspm0)
{
	enum fw_upload_err ret;
	const struct bsl_no_data_req_pkt pkt =
		bsl_no_data_req_pkt_create(COMMAND_START_APPLICATION);

	dev_info(&mspm0->client->dev, "Start Application");

	ret = bsl_tx(mspm0->client, (const u8 *)&pkt, sizeof(pkt));
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(&mspm0->client->dev, ret,
				     "Failed to send start_application");

	return bsl_get_ack(mspm0->client);
}

static enum fw_upload_err bsl_standalone_verification(struct mspm0_i2c *mspm0,
						      u32 *crc32_out)
{
	enum fw_upload_err ret;
	struct bsl_standalone_verification_resp_pkt response;
	const struct device *dev = &mspm0->client->dev;
	u16 data_len = MSPM0_BSL_DATA_LEN(
		sizeof(struct bsl_standalone_verification_req_pkt));
	struct bsl_standalone_verification_req_pkt request = {
		.head = { .header = REQUEST,
			  .len = cpu_to_le16(data_len),
			  .cmd = COMMAND_STANDALONE_VERIFICATION },
		.address = 0,
		.size = cpu_to_le32(mspm0->fwl_size),
	};
	u32 crc = crc32(0xffffffff, (const u8 *)&request.head.cmd, data_len);

	dev_info(dev, "Performing Verification");

	request.tail = cpu_to_le32(crc);

	ret = bsl_tx(mspm0->client, (const u8 *)&request, sizeof(request));
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(dev, ret,
				     "Failed to send standalone_verification");

	ret = bsl_get_ack(mspm0->client);
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(dev, ret,
				     "Failed standalone_verification");

	ret = bsl_rx(mspm0->client, (u8 *)&response, sizeof(response));
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(
			dev, ret, "Invalid standalone_verification response");

	if (!bsl_resp_pkt_validate(&response.head, sizeof(response)))
		return dev_err_probe(
			dev, FW_UPLOAD_ERR_RW_ERROR,
			"Invalid standalone_verification response");

	*crc32_out = le32_to_cpu(response.crc);

	return FW_UPLOAD_ERR_NONE;
}

static enum fw_upload_err mspm0_prepare(struct fw_upload *fw_upload,
					const u8 *data, u32 size)
{
	u32 crc;
	enum fw_upload_err ret;
	struct mspm0_i2c *mspm0 = fw_upload->dd_handle;

	dev_info(&mspm0->client->dev, "mspm0l1105 flashing initiated");

	mspm0->fwl_crc = crc32(0xffffffff, data, size);
	mspm0->fwl_size = size;

	dev_info(&mspm0->client->dev, "Booting into BSL");

	gpiod_direction_output(mspm0->bsl_backdoor_gpio, 0);
	gpiod_direction_output(mspm0->rst_gpio, 1);
	msleep(MSEC_PER_SEC);

	gpiod_set_value(mspm0->bsl_backdoor_gpio, 1);
	gpiod_direction_output(mspm0->rst_gpio, 0);
	msleep(MSEC_PER_SEC);

	gpiod_direction_output(mspm0->rst_gpio, 1);
	msleep(MSEC_PER_SEC);

	gpiod_direction_input(mspm0->rst_gpio);
	gpiod_direction_input(mspm0->bsl_backdoor_gpio);

	ret = bsl_connect(mspm0);
	if (ret != FW_UPLOAD_ERR_NONE)
		return ret;

	ret = bsl_get_device_info(mspm0);
	if (ret != FW_UPLOAD_ERR_NONE)
		return ret;

	ret = bsl_unlock_bsl(mspm0);
	if (ret != FW_UPLOAD_ERR_NONE)
		return ret;

	ret = bsl_standalone_verification(mspm0, &crc);
	if (ret != FW_UPLOAD_ERR_NONE)
		return ret;

	if (crc == mspm0->fwl_crc) {
		bsl_start_application(mspm0);
		return dev_warn_probe(&mspm0->client->dev,
				      FW_UPLOAD_ERR_FW_INVALID,
				      "Skipping reflashing same image");
	}

	return bsl_mass_erase(mspm0);
}

static enum fw_upload_err mspm0_write(struct fw_upload *fw_upload,
				      const u8 *data, u32 offset, u32 size,
				      u32 *written)
{
	enum fw_upload_err ret;
	struct mspm0_i2c *mspm0 = fw_upload->dd_handle;

	ret = bsl_program_data(mspm0, data + offset, size, written, offset);
	if (ret != FW_UPLOAD_ERR_NONE)
		return dev_err_probe(&mspm0->client->dev,
				     FW_UPLOAD_ERR_HW_ERROR,
				     "Failed to flash firmware");

	return FW_UPLOAD_ERR_NONE;
}

static enum fw_upload_err mspm0_poll_complete(struct fw_upload *fw_upload)
{
	u32 crc;
	enum fw_upload_err ret;
	struct mspm0_i2c *mspm0 = fw_upload->dd_handle;

	ret = bsl_standalone_verification(mspm0, &crc);
	if (ret != FW_UPLOAD_ERR_NONE)
		return ret;

	if (crc != mspm0->fwl_crc)
		return dev_err_probe(&mspm0->client->dev,
				     FW_UPLOAD_ERR_HW_ERROR,
				     "CRC32 check failed");

	ret = bsl_start_application(mspm0);
	if (ret != FW_UPLOAD_ERR_NONE)
		return ret;

	dev_info(&mspm0->client->dev, "Flashing successful");

	return FW_UPLOAD_ERR_NONE;
}

static void mspm0_cancel(struct fw_upload *fw_upload)
{
}

static void mspm0_cleanup(struct fw_upload *fw_upload)
{
}

static const struct fw_upload_ops mspm0_i2c_ops = {
	.prepare = mspm0_prepare,
	.write = mspm0_write,
	.poll_complete = mspm0_poll_complete,
	.cancel = mspm0_cancel,
	.cleanup = mspm0_cleanup,
};

static int mspm0_i2c_probe(struct i2c_client *client)
{
	struct mspm0_i2c *mspm0;
	struct gpio_desc *desc;
	struct fw_upload *fwl;
	int ret;

	mspm0 = devm_kmalloc(&client->dev, sizeof(*mspm0), GFP_KERNEL);
	if (!mspm0)
		return -ENOMEM;
	mspm0->client = client;

	desc = devm_gpiod_get(&client->dev, "bootloader-backdoor", GPIOD_IN);
	if (IS_ERR(desc))
		return PTR_ERR(desc);
	mspm0->bsl_backdoor_gpio = desc;

	desc = devm_gpiod_get(&client->dev, "reset", GPIOD_IN);
	if (IS_ERR(desc)) {
		ret = PTR_ERR(desc);
		goto free_boot;
	}
	mspm0->rst_gpio = desc;

	fwl = firmware_upload_register(THIS_MODULE, &client->dev, "mspm0l1105",
				       &mspm0_i2c_ops, mspm0);
	if (IS_ERR(fwl)) {
		ret = PTR_ERR(fwl);
		goto free_reset;
	}
	mspm0->fwl = fwl;

	i2c_set_clientdata(client, mspm0);

	return 0;

free_reset:
	devm_gpiod_put(&client->dev, mspm0->rst_gpio);
	mspm0->rst_gpio = NULL;
free_boot:
	devm_gpiod_put(&client->dev, mspm0->bsl_backdoor_gpio);
	mspm0->bsl_backdoor_gpio = NULL;
	return ret;
}

static void mspm0_i2c_remove(struct i2c_client *client)
{
	struct mspm0_i2c *mspm0 = i2c_get_clientdata(client);

	firmware_upload_unregister(mspm0->fwl);
}

static const struct of_device_id mspm0_i2c_of_match[] = {
	{
		.compatible = "ti,mspm0l1105",
	},
	{},
};
MODULE_DEVICE_TABLE(of, mspm0_i2c_of_match);

static struct i2c_driver mspm0_i2c_driver = {
	.probe = mspm0_i2c_probe,
	.remove = mspm0_i2c_remove,
	.driver = {
		.name = "mspm0_i2c", 
		.of_match_table = mspm0_i2c_of_match, 
	},
};
module_i2c_driver(mspm0_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ayush Singh <ayush@beagleboard.org>");
MODULE_DESCRIPTION("MSPM0 I2C driver");
