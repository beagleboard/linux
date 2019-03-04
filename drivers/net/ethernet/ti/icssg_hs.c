// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments ICSSG Firmware Handshake protocol helpers
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 */

#include "icssg_prueth.h"

static void icss_hs_get(struct prueth *prueth, int slice, struct icss_hs *hs)
{
	void __iomem *va;

	va = prueth->shram.va + slice * ICSS_HS_OFFSET_SLICE1;
	memcpy_fromio(hs, va, sizeof(*hs));
}

/* check if firmware is dead. returns error code in @err_code */
bool icss_hs_is_fw_dead(struct prueth *prueth, int slice, u16 *err_code)
{
	struct icss_hs *hs = &prueth->hs[slice];
	u32 status;

	icss_hs_get(prueth, slice, hs);
	status = le32_to_cpu(hs->fw_status);
	/* lower 16 bits contain error code */
	*err_code = status & 0xffff;
	return ((status & 0xffff0000) == ICSS_HS_FW_DEAD);
}

/* check if firmware is ready */
bool icss_hs_is_fw_ready(struct prueth *prueth, int slice)
{
	struct icss_hs *hs = &prueth->hs[slice];
	u32 status;

	icss_hs_get(prueth, slice, hs);
	status = le32_to_cpu(hs->fw_status);
	return (status == ICSS_HS_FW_READY);
}

/* send a command to firmware */
int icss_hs_send_cmd(struct prueth *prueth, int slice, u32 cmd,
		     u32 *idata, u32 ilen)
{
	void __iomem *va, *vax;
	struct icss_hs *hs = &prueth->hs[slice];
	int i;

	va = prueth->shram.va + slice * ICSS_HS_OFFSET_SLICE1;

	icss_hs_get(prueth, slice, hs);
	if (ilen > le16_to_cpu(hs->ilen_max))
		return -EINVAL;

	hs->ilen = cpu_to_le32(ilen);
	memcpy_toio(va + offsetof(struct icss_hs, ilen),
		    &hs->ilen, sizeof(hs->ilen));

	/* copy over input data */
	vax = va + hs->ioffset * 4;
	for (i = 0; i < ilen; i++)
		writel_relaxed(cpu_to_le32(idata[i]), vax + i * 4);

	cmd &= 0x1fffffff;
	hs->cmd = cpu_to_le32(cmd);
	memcpy_toio(va + offsetof(struct icss_hs, cmd),
		    &hs->cmd, sizeof(hs->cmd));

	return 0;
}

/* check if command done */
bool icss_hs_is_cmd_done(struct prueth *prueth, int slice)
{
	struct icss_hs *hs = &prueth->hs[slice];
	u32 cmd;
	int trys;

	for (trys = 1; trys < 3; trys++) {
		icss_hs_get(prueth, slice, hs);
		cmd = le32_to_cpu(hs->cmd);
		if (cmd & ICSS_HS_CMD_DONE)
			break;

		/* If firmware didn't see the command yet, wait and retry */
		if (!(cmd & ICSS_HS_CMD_BUSY)) {
			dev_err(prueth->dev,
				"slice %d fw didn't see cmd 0x%x, try: %d\n",
				slice, cmd, trys);
		}

		udelay(5);
	}

	return !!(cmd & ICSS_HS_CMD_DONE);
}

/* send command and check if command done */
int icss_hs_send_cmd_wait_done(struct prueth *prueth, int slice,
			       u32 cmd, u32 *idata, u32 ilen)
{
	int ret;

	ret = icss_hs_send_cmd(prueth, slice, cmd, idata, ilen);
	if (ret)
		return ret;

	udelay(5);	/* FW can take about 1uS */
	if (icss_hs_is_cmd_done(prueth, slice))
		return 0;

	return -EIO;
}

/* read back result (output data)
 * @odata: output data parameters. list of 32 bit words.
 * @olen: maximum number of 32-bit output words to read.
 * returns number of result words read. negative on error.
 */
int icss_hs_get_result(struct prueth *prueth, int slice, u32 *odata, u32 olen)
{
	void __iomem *va;
	struct icss_hs *hs = &prueth->hs[slice];
	int i;
	u32 cmd, hsolen;

	va = prueth->shram.va + slice * ICSS_HS_OFFSET_SLICE1;

	icss_hs_get(prueth, slice, hs);

	cmd = le32_to_cpu(hs->cmd);
	if (!(cmd & ICSS_HS_CMD_DONE))
		return -EBUSY;

	hsolen = le32_to_cpu(hs->olen);
	olen = min(hsolen, olen);

	va += hs->ooffset * 4;
	for (i = 0; i < olen; i++) {
		u32 val;

		val = readl_relaxed(va + i * 4);
		odata[i] = le32_to_cpu(val);
	}

	return olen;
}
