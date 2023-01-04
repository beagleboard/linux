// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 Alibaba Inc.
 */
#include <linux/clk.h>
#include <linux/compiler_types.h>
#include <linux/device.h>
#include <linux/pm_runtime.h>
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mfd/syscon.h>

#define CON		0x00
#define LCPAR		0x04
#define ADDR		0x40
#define WDATA		0x44
#define WDATA_MASK	0x48
#define WP0		0x50
#define WP1		0x54
#define WP2		0x58
#define WP3		0x5c
#define STA		0x70
#define RDATA0		0x80
#define SHADOW_RDATA0	0xc0
#define SHADOW_RDATA1   0xc4
#define SHADOW_RDATA2   0xc8
#define SHADOW_RDATA3   0xcc
#define SHADOW_RDATA4   0xd0
#define SHADOW_RDATA5   0xd4
#define SHADOW_RDATA6   0xd8
#define SHADOW_RDATA7   0xdc

#define TEE_SYS_EFUSE_LC_PRELD_OFF	0x64
#define TEE_SYS_EFUSE_DBG_KEY1_OFF	0x70
#define ENABLE_DFT_FUNC_MASK		GENMASK(3, 0)
#define ENABLE_DFT_FUNC			0x5
#define DISABLE_DFT_FUNC		0xa

/* bit definition for CON */
#define EFUSE_CON_POWER_MSK		BIT(14)

/* bit definition for STA */
#define EFUSE_STA_IDLE_MSK		BIT(0)

#define EFUSE_STA_RD_STATUS_POS		4
#define EFUSE_STA_RD_STATUS_MSK		(0x7UL << EFUSE_STA_RD_STATUS_POS)

#define	EFUSE_STA_WR_STATUS_POS		8
#define EFUSE_STA_WR_STATUS_MSK		(0x7UL << EFUSE_STA_WR_STATUS_POS)

#define EFUSE_STA_CMD_ILLEGAL_POS	11
#define EFUSE_STA_CMD_ILLEGAL_MSK	(0x1UL << EFUSE_STA_CMD_ILLEGAL_POS)

#define EFUSE_STA_KTRANS_ALARM_POS	14
#define EFUSE_STA_KTRANS_ALARM_MSK	(0x1UL << EFUSE_STA_KTRANS_ALARM_POS)

/* Max try time for idle check */
#define MAX_TRY_TIME_IDLE		10000
#define DEVICE_BUSY			1

#define EFUSE_CON_CMD_POS		8
#define EFUSE_CON_CMD_MSK		GENMASK(12, 8)
#define EFUSE_CON_CMD_IDLE		(0x0 << EFUSE_CON_CMD_POS)
#define EFUSE_CON_CMD_READ		(0x1 << EFUSE_CON_CMD_POS)
#define EFUSE_CON_CMD_WRITE		(0x2 << EFUSE_CON_CMD_POS)
#define EFUSE_CON_CMD_BLKREAD		(0x3 << EFUSE_CON_CMD_POS)
#define EFUSE_CON_CMD_WP_LOCK		(0x8 << EFUSE_CON_CMD_POS)
#define EFUSE_CON_CMD_CP_LOCK		(0x9 << EFUSE_CON_CMD_POS)
#define EFUSE_CON_CMD_RP_LOCK		(0xA << EFUSE_CON_CMD_POS)
#define EFUSE_CON_CMD_UP_LC		(0x10 << EFUSE_CON_CMD_POS)

#define EFUSE_CON_START			BIT(0)
#define EFUSE_CON_CLEAR			BIT(1)

#define EFUSE_CON_KEY_TRANS_MSK		BIT(13)
#define EFUSE_CON_LC_READ_MSK		BIT(2)
#define EFUSE_CON_START_MSK		BIT(0)

#define EFUSE_CON_MSK			(EFUSE_CON_LC_READ_MSK | \
					 EFUSE_CON_CMD_MSK |	 \
					 EFUSE_CON_KEY_TRANS_MSK | \
					 EFUSE_CON_START_MSK)

#define IS_CVKEY1(addr)         ((addr >= 0x38) && (addr < 0x3C))
#define IS_CVKEY2(addr)         ((addr >= 0x3C) && (addr < 0x78))
#define IS_USRKEY2(addr)        ((addr >= 0x78) && (addr < 0xd8))

/* Block width (bytes) definitions */
#define LIGHT_EFUSE_LIT_BLOCK_WIDTH		16
#define LIGHT_EFUSE_BIG_BLOCK_WIDTH             32
#define LIGHT_EFUSE_LIT_BLOCK_NUM		52
#define LIGHT_EFUSE_BIG_BLOCK_NUM		6

#define RMA_LIFE_CYCLE_PARA	0x1A946F9B
#define RIP_LIFE_CYCLE_PARA	0xEE45E8A7

struct light_efuse_priv {
	struct device *dev;
	void __iomem *base;
	struct regmap *teesys_regs;
	struct clk *clk;
	u32 sysfs_rd_offset;
	u32 sysfs_rd_len;
};

static u32 perm_spi_magic[] = {
        0x9804E1BC,
        0x4B8B59F5,
        0x36D33417,
        0x7491B7D5,
};
static u32 update_lc_magic[] = {
        0x768A7E2F,
        0xE4D53282,
        0x8BD97337,
        0x677B9E85,
};
static u32 read_magic[] = {
        0x32224E05,
        0xC3F981D0,
        0xF4D7FB08,
        0xA4C8C6DE,
};
static u32 write_magic[] = {
        0xB4BC4A0A,
        0x2A8B7E6F,
        0x974B25A1,
        0x67DB5F5F,
};
static u32 block_read_magic[] = {
        0x39CF83C1,
        0xD0DDD6B2,
        0xBD50693B,
        0x5F61B752,
};
static u32 wp_lock_magic[] = {
        0x0D11ECA6,
        0x06EDF631,
        0xB58CA544,
        0x1EBDE503,
};
static u32 cp_lock_magic[] = {
        0xC21E9BB8,
        0x0FC428F1,
        0xD8E95026,
        0x1C34AC41,
};
static u32 rp_lock_magic[] = {
        0xAEB3089A,
        0x8DE56E9A,
        0x453416C2,
        0x969F6937,
};
static u32 key_tran_magic[] = {
        0x1AF5952C,
        0x111B5E55,
        0xFAE8A83D,
        0xEDFE9E7F,
};

static u32 *cmd_perm_magic_num[] = {
	perm_spi_magic,
	update_lc_magic,
	read_magic,
	write_magic,
	block_read_magic,
	wp_lock_magic,
	cp_lock_magic,
	rp_lock_magic,
	key_tran_magic
};

enum permission_type {
	CMD_SPI = 0,
	CMD_UPDATE_LC,
	CMD_READ,
	CMD_WRITE,
	CMD_BLOCK_READ,
	CMD_WP_LOCK,
	CMD_CP_LOCK,
	CMD_RP_LOCK,
	CMD_KEY_TRAN,
	CMD_KEY_MAX,
};

enum con_cmd_type {
	CON_CMD_IDLE = 0,
	CON_CMD_READ,
	CON_CMD_WRITE,
	CON_CMD_BLOCK_RD,
	CON_CMD_WP_LOCK,
	CON_CMD_CP_LOCK,
	CON_CMD_RP_LOCK,
	CON_CMD_UP_LC,
	CON_CMD_MAX,
};

static inline bool efuse_poweron_status_get(void __iomem *base)
{
	return readl(base + CON) & EFUSE_CON_POWER_MSK ? false : true;
}

static inline int efuse_idle_check(void __iomem *base)
{
	int try_cnt = MAX_TRY_TIME_IDLE;

	while (try_cnt--) {
		if (!(readl(base + STA) & EFUSE_STA_IDLE_MSK))
			return 0;
	}

	if (try_cnt <= 0)
		return -DEVICE_BUSY;

	return 0;
}

static inline int efuse_status_check(void __iomem *base)
{
	u32 data = readl(base + STA);
	int errcode;

	errcode = data & (EFUSE_STA_RD_STATUS_MSK | EFUSE_STA_WR_STATUS_MSK |
			  EFUSE_STA_CMD_ILLEGAL_MSK | EFUSE_STA_KTRANS_ALARM_MSK);

	pr_debug("[%s,%d]efuse status before clear: 0x%x\n", __func__, __LINE__, errcode);

	/* If error happens, write clear should be added */
	if (errcode) {
		pr_err("error efuse operation STA status: 0x%x\n", errcode);
		writel(data, base + STA);
	}

	return -errcode;
}

static inline int efuse_poweron(void __iomem *base)
{
	u32 data;
	int ret;

	if (efuse_poweron_status_get(base))
		return 0;

	data = readl(base + CON);
	data &= ~EFUSE_CON_POWER_MSK;
	writel(data, base + CON);

	ret = efuse_idle_check(base);

	ret |= efuse_status_check(base);

	pr_debug("pd status: 0x%lx\n", readl(base + CON) & EFUSE_CON_POWER_MSK);

	return ret;
}

static inline u32 efuse_data_read(void __iomem *base)
{
	return readl(base + RDATA0);
}

static inline void efuse_data_clear(void __iomem *base)
{
	u32 data = readl(base + CON);
	data |= EFUSE_CON_CLEAR;
	writel(data, base + CON);
}

static
inline void efuse_permission_magic_config(void __iomem *base, u32 *magic_num[],
					  enum permission_type cmd)
{
	writel(magic_num[cmd][3], base + WP0);
	writel(magic_num[cmd][2], base + WP1);
	writel(magic_num[cmd][1], base + WP2);
	writel(magic_num[cmd][0], base + WP3);
}

static inline void efuse_addr_config(void __iomem *base, u32 addr)
{
	writel(addr, base + ADDR);
	pr_debug("[%s, %d]efuse addr reg: 0x%x\n", __func__, __LINE__, readl(base + ADDR));
}

static inline void efuse_data_mask_config(void __iomem *base, u32 mask)
{
	writel(mask, base + WDATA_MASK);
}

static inline void efuse_data_config(void __iomem *base, u32 data)
{
	writel(data, base + WDATA);
	pr_debug("[%s, %d]efuse data reg: 0x%x\n", __func__, __LINE__, readl(base + WDATA));
}

static inline void efuse_life_cycle_para_config(void __iomem *base, u32 data)
{
	writel(data, base + LCPAR);
}

static inline u32 efuse_life_cycle_para_get(void __iomem *base)
{
	return readl(base + LCPAR);
}

static inline int efuse_cmd_start(void __iomem *base, enum con_cmd_type cmd_type)
{
	u32 data, cmd;

	switch (cmd_type) {
		case CON_CMD_IDLE:
			cmd = EFUSE_CON_CMD_IDLE;
			break;
		case CON_CMD_READ:
			cmd = EFUSE_CON_CMD_READ;
			break;
		case CON_CMD_WRITE:
			cmd = EFUSE_CON_CMD_WRITE;
			break;
		case CON_CMD_BLOCK_RD:
			cmd = EFUSE_CON_CMD_BLKREAD;
			break;
		case CON_CMD_WP_LOCK:
			cmd = EFUSE_CON_CMD_WP_LOCK;
			break;
		case CON_CMD_CP_LOCK:
			cmd = EFUSE_CON_CMD_CP_LOCK;
			break;
		case CON_CMD_RP_LOCK:
			cmd = EFUSE_CON_CMD_RP_LOCK;
			break;
		case CON_CMD_UP_LC:
			cmd = EFUSE_CON_CMD_UP_LC;
			break;
		default:
			return -EINVAL;

	}

	/* Mask LC_Read, Key_transfer, command and start bits */
	data = readl(base + CON);
	data &= ~EFUSE_CON_MSK;
	data |= cmd | EFUSE_CON_START;
	writel(data, base + CON);

	return 0;
}

static DEFINE_MUTEX(light_efuse_mutex);

static int light_efuse_read_start(void __iomem *base, u32 addr, enum con_cmd_type cmd_type)
{
	int ret = 0;
	enum permission_type permission;

	if (cmd_type == CON_CMD_READ)
		permission = CMD_READ;
	else if (cmd_type == CON_CMD_BLOCK_RD)
		permission = CMD_BLOCK_READ;
	else {
		pr_err("invaid efuse read command type\n");
		return -EINVAL;
	}

	ret = efuse_idle_check(base);
	if (ret) {
		pr_err("the device is busy\n");
		return ret;
	}

	efuse_permission_magic_config(base, cmd_perm_magic_num, permission);

	efuse_addr_config(base, addr);

	ret = efuse_cmd_start(base, cmd_type);
	if (ret)
		return ret;

	/* Wait controller completed */
	ret = efuse_idle_check(base);

	/* Check status, if there has error, reort and clear status */
	ret |= efuse_status_check(base);
	if (ret) {
		pr_err("error occurs while start reading at efuse byte addr: %d\n", addr * 4);
		return ret;
	}

	if (cmd_type == CON_CMD_BLOCK_RD) {
		pr_debug("=======================================\n");
		pr_debug("shadow: 0x%x\n", readl(base + SHADOW_RDATA0));
		pr_debug("shadow: 0x%x\n", readl(base + SHADOW_RDATA1));
		pr_debug("shadow: 0x%x\n", readl(base + SHADOW_RDATA2));
		pr_debug("shadow: 0x%x\n", readl(base + SHADOW_RDATA3));
		pr_debug("shadow: 0x%x\n", readl(base + SHADOW_RDATA4));
		pr_debug("shadow: 0x%x\n", readl(base + SHADOW_RDATA5));
		pr_debug("shadow: 0x%x\n", readl(base + SHADOW_RDATA6));
		pr_debug("shadow: 0x%x\n", readl(base + SHADOW_RDATA7));
	}

	return ret;
}

static int light_efuse_read_word(void __iomem *base, u32 addr, u32 *val)
{
	int ret = 0;

	ret = efuse_idle_check(base);
	if (ret)
		return ret;

	ret = light_efuse_read_start(base, addr, CON_CMD_READ);
	if (ret) {
		pr_err("failed to start efuse read\n");
		goto exit;
	}

	*val = efuse_data_read(base);

	pr_debug("[%s][%d]data = 0x%x\n", __func__, __LINE__,*val);

exit:
	efuse_data_clear(base);

	return ret;
}

static int light_efuse_write_word(void __iomem *base, u32 addr, u32 data, u32 mask)
{
	int ret = 0;

	ret = efuse_idle_check(base);
	if (ret)
		return ret;

	/*
	 * Check permission:
	 * Check it every time to avoid wp0~3 are changed somewhere
	 */
	efuse_permission_magic_config(base, cmd_perm_magic_num, CMD_WRITE);

	/* Config address */
	efuse_addr_config(base, addr);

	/* Config data */
	efuse_data_config(base, data);

	/* Config data mask , if we're in keyrange mask should be set to 0 */
	if (IS_CVKEY1(addr) || IS_CVKEY2(addr) || IS_USRKEY2(addr))
		efuse_data_mask_config(base, 0);
	else
		efuse_data_mask_config(base, mask);

	/* Set write command */
	ret = efuse_cmd_start(base, CON_CMD_WRITE);
	if (ret)
		goto exit;

	/* Wait controller completed */
	ret = efuse_idle_check(base);

exit:
	/* Check status, if there has error, reort and clear status */
	ret |= efuse_status_check(base);
	if (ret)
		pr_err("error occurs while start writing at efuse byte addr: %d\n", addr * 4);

	efuse_data_clear(base);

	return ret;
}

static int light_efuse_read(void *context, unsigned int addr, void *data, size_t bytes)
{
	struct light_efuse_priv *priv = context;
	u32 byte_offset, read_count, read_addr;
	u8 *pdst, *psrc;
	u32 value;
	int ret = 0;

	mutex_lock(&light_efuse_mutex);

	dev_dbg(priv->dev, "[%s]efuse addr: 0x%x, bytes: %d\n", __func__, addr, (int)bytes);

	ret = pm_runtime_get_sync(priv->dev);
	if (ret < 0) {
		dev_err(priv->dev, "failed to get the efuse device(%d)\n", ret);
		pm_runtime_put_noidle(priv->dev);
		goto read_end;
	}

	if (efuse_poweron(priv->base)) {
		dev_err(priv->dev, "failed to power on efuse\n");
		ret = -EBUSY;
		goto read_end;
	}

	pdst	=	data;
	byte_offset =	addr & 0x3;
	read_addr =	addr / 4;	/* Efuse unit is 4 bytes */

	/* byte_offset != 0, means not 4 bytes aligned, read first word first */
	if (byte_offset) {
		ret = light_efuse_read_word(priv->base, read_addr, &value);
		if (ret) {
			dev_err(priv->dev, "failed to read efuse data\n");
			goto read_end;
		}

		read_count =	4 - byte_offset;
		psrc	   =	(u8 *)&value + byte_offset;
		if (bytes < read_count)
			read_count = bytes;
		memcpy(pdst, psrc, read_count);

		read_addr++;
		pdst += read_count;
		bytes -= read_count;
	}

	while (bytes >= 4) {
		ret = light_efuse_read_word(priv->base, read_addr, &value);
		if (ret) {
			dev_err(priv->dev, "failed to read efuse data\n");
			goto read_end;
		}
		memcpy(pdst, &value, 4);
		bytes	-= 4;
		read_addr++; /* the hardware will span over one word length automatically */
		pdst	+= 4;
	}

	if (bytes > 0) {
		ret = light_efuse_read_word(priv->base, read_addr, &value);
		if (ret) {
			dev_err(priv->dev, "failed to read data from efuse\n");
			goto read_end;
		}
		memcpy(pdst, &value, bytes);
	}

	pm_runtime_put_sync(priv->dev);

read_end:
	mutex_unlock(&light_efuse_mutex);

	return ret;
}

static int light_efuse_write(void *context, unsigned int addr, void *data, size_t bytes)
{
	struct light_efuse_priv *priv = context;
	int ret = 0;
	u32 byte_offset, write_addr, value = 0;
	u32 write_count, mask;
	u8 *psrc, *pdst;
	size_t __maybe_unused orign_bytes = bytes;

	mutex_lock(&light_efuse_mutex);

	dev_dbg(priv->dev, "[%s]efuse addr: 0x%x, bytes: %d\n", __func__, addr, (int)bytes);

	ret = pm_runtime_get_sync(priv->dev);
	if (ret < 0) {
		dev_err(priv->dev, "failed to get the efuse device(%d)\n", ret);
		pm_runtime_put_noidle(priv->dev);
		goto write_end;
	}

	if (efuse_poweron(priv->base)) {
		dev_err(priv->dev, "failed to power on efuse\n");
		ret = -EBUSY;
		goto write_end;
	}

	byte_offset =	addr & 0x3;
	psrc =		(u8 *)data;
	write_addr =	addr / 4;

	pr_debug("[%s][%d]: write addr = 0x%x\n", __func__, __LINE__, write_addr);

	pr_debug("Write data: ");
	if (byte_offset) {
		write_count = 4 - byte_offset;
		if (bytes < write_count)
			write_count = bytes;
		pdst = (u8 *)&value + byte_offset;
		memcpy(pdst, psrc, write_count);
		pr_debug("0x%x  ", *psrc);

		mask = ~value;
		ret = light_efuse_write_word(priv->base, write_addr, value, mask);
		if (ret) {
			dev_err(priv->dev, "failed to write data to efuse\n");
			goto write_end;
		}

		psrc += write_count;
		write_addr++;
		bytes	-= write_count;
	}

	while (bytes >= 4) {
		value = 0;
		pdst = (u8 *)&value;
		write_count = 4;
		memcpy(pdst, psrc, write_count);
		pr_debug("0x%x  ", *psrc);

		mask = ~value;
		ret = light_efuse_write_word(priv->base, write_addr, value, mask);
		if (ret) {
			dev_err(priv->dev, "failed to write data to efuse\n");
			goto write_end;
		}

		psrc	+= write_count;
		bytes	-= write_count;
		write_addr++;
	}

	if (bytes > 0) {
		value	= 0;
		pdst	= (u8 *)&value;
		memcpy(pdst, psrc, bytes);
		pr_debug("0x%x  ", *psrc);

		mask	= ~value;
		ret = light_efuse_write_word(priv->base, write_addr, value, mask);
		if (ret) {
			dev_err(priv->dev, "failed to write data to efuse\n");
			goto write_end;
		}
	}

	pr_debug("\n");

	pm_runtime_put_sync(priv->dev);

write_end:
	mutex_unlock(&light_efuse_mutex);

	return ret < 0 ? ret : orign_bytes;
}

static ssize_t efuse_nvmem_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct light_efuse_priv *priv = dev_get_drvdata(dev);
	char *start = (char *)buf;
	char type;
	unsigned long addr, len;
	unsigned char *data;
	int i, ret;

	/*
	 * echo types:
	 * echo w offset len 0x01 0x02 0x03 ... > efuse_nvmem
	 * echo r offset len > efuse_nvmem
	 */
	while (*start == ' ') /* skip space */
		start++;

	if (*start != 'w' && *start != 'r')
		return -EINVAL;

	type = *start;

	start++;
	while (*start == ' ')
		start++;
	addr = simple_strtoul(start, &start, 0);

	while (*start == ' ')
		start++;
	len = simple_strtoul(start, &start, 0);

	priv->sysfs_rd_offset	= addr;
	priv->sysfs_rd_len	= len;

	if (type == 'r')
		goto exit;

	data = kzalloc(sizeof(*data) * len, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	pr_debug("echo data:\n");
	for (i = 0; i < len; i++) {
		while (*start == ' ')
			start++;
		data[i] = simple_strtoul(start, &start, 0);
		pr_debug("0x%x ", data[i]);
	}

	ret = light_efuse_write(priv, addr, data, len);
	if (ret < 0) {
		kfree(data);
		return ret;
	}

	kfree(data);
exit:
	return count;
}

static ssize_t efuse_nvmem_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct light_efuse_priv *priv = dev_get_drvdata(dev);
	u32 addr = priv->sysfs_rd_offset;
	u32 len = priv->sysfs_rd_len;
	int ret, i;
	unsigned char *data;
	size_t bufpos = 0, count;

	data = kzalloc(sizeof(*data) * len, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = light_efuse_read(priv, addr, data, len);
	if (ret < 0)
		goto out;

	count = (len + 2) * 10;
	for (i = 0; i < len; i++) {
		snprintf(buf + bufpos, count - bufpos, "0x%.*x", 2, data[i]);
		bufpos += 4;
		if (i == len - 1 || (i !=0 && i % 16 == 0))
			buf[bufpos++] = '\n';
		else
			buf[bufpos++] = ' ';
	}

out:
	kfree(data);

	return bufpos;
}

static ssize_t rma_lc_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct light_efuse_priv *priv = dev_get_drvdata(dev);
	u32 value = RMA_LIFE_CYCLE_PARA;
	int ret;

	ret = pm_runtime_get_sync(priv->dev);
	if (ret < 0) {
		dev_err(priv->dev, "failed to get the efuse device(%d)\n", ret);
		pm_runtime_put_noidle(priv->dev);
		return ret;
	}

	regmap_update_bits(priv->teesys_regs,
			TEE_SYS_EFUSE_DBG_KEY1_OFF,
			ENABLE_DFT_FUNC_MASK,
			ENABLE_DFT_FUNC);

	efuse_permission_magic_config(priv->base, cmd_perm_magic_num, CMD_UPDATE_LC);

	efuse_life_cycle_para_config(priv->base, value);

	/* Set command */
	ret = efuse_cmd_start(priv->base, CON_CMD_UP_LC);
	if (ret)
		goto exit;

	/* Wait controller completed */
	ret = efuse_idle_check(priv->base);

	pr_debug("set life cycle value: 0x%x\n", value);

exit:
	regmap_update_bits(priv->teesys_regs,
			TEE_SYS_EFUSE_DBG_KEY1_OFF,
			ENABLE_DFT_FUNC_MASK,
			DISABLE_DFT_FUNC);
	/* Check status, if there has error, reort and clear status */
	ret |= efuse_status_check(priv->base);
	if (ret)
		pr_err("error occurs while starting write\n");

	efuse_data_clear(priv->base);

	pm_runtime_put_sync(priv->dev);

	return ret < 0 ? ret : count;
}

static ssize_t rip_lc_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct light_efuse_priv *priv = dev_get_drvdata(dev);
	u32 value = RIP_LIFE_CYCLE_PARA;
	int ret;

	ret = pm_runtime_get_sync(priv->dev);
	if (ret < 0) {
		dev_err(priv->dev, "failed to get the efuse device(%d)\n", ret);
		pm_runtime_put_noidle(priv->dev);
		return ret;
	}

	efuse_permission_magic_config(priv->base, cmd_perm_magic_num, CMD_UPDATE_LC);

	efuse_life_cycle_para_config(priv->base, value);

	/* Set command */
	ret = efuse_cmd_start(priv->base, CON_CMD_UP_LC);
	if (ret)
		goto exit;

	/* Wait controller completed */
	ret = efuse_idle_check(priv->base);

exit:
	/* Check status, if there has error, reort and clear status */
	ret |= efuse_status_check(priv->base);
	if (ret)
		pr_err("error occurs while starting write\n");

	efuse_data_clear(priv->base);

	pm_runtime_put_sync(priv->dev);

	return ret < 0 ? ret : count;
}

static ssize_t lc_preld_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct light_efuse_priv *priv = dev_get_drvdata(dev);
	int ret;
	u32 data;

	ret = regmap_read(priv->teesys_regs, TEE_SYS_EFUSE_LC_PRELD_OFF, &data);
	if (ret) {
		dev_err(dev, "failed to read data from LC_PRELD area\n");
		return ret;
	}

	return sprintf(buf, "0x%08x\n", data);
}

static ssize_t update_lc_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct light_efuse_priv *priv = dev_get_drvdata(dev);
	int ret;
	u32 value, data;
	const char *p, *life_cycle = buf;
	int len;

	p = memchr(buf, '\n', count);
	len = p ? p - buf : count;

	dev_dbg(dev, "life_cycle: %s, buf: %s, len: %d\n", life_cycle, buf, len);

	if (!strncmp(life_cycle, "LC_RMA", len)) {
		/* If target life cycle is RMA, open permission in teesystem regs */
		ret = regmap_read(priv->teesys_regs,
				  TEE_SYS_EFUSE_DBG_KEY1_OFF,
				  &data); /* Register from tee system */
		if (ret) {
			dev_err(dev, "failed to read data from DBG_KEY1 area\n");
			return ret;
		}

		data &= ~0xf;
		data |= 0x5;
		ret = regmap_write(priv->teesys_regs,
				   TEE_SYS_EFUSE_DBG_KEY1_OFF,
				   data);
		if (ret) {
			dev_err(dev, "failed to write data to DBG_KEY1 area\n");
			return ret;
		}

		value = 0x1A946F9B;
	} else if (!strncmp(life_cycle, "LC_OEM", len))
		value = 0x64EA9B8E;
	else if (!strncmp(life_cycle, "LC_PRO", len))
		value = 0xB0E047A8;
	else if (!strncmp(life_cycle, "LC_DEV", len))
		value = 0x59DD3BDF;
	else if (!strncmp(life_cycle, "LC_RIP", len))
		value = 0xEE45E8A7;
	else if (!strncmp(life_cycle, "LC_KILL_KEY1", len))
		value = 0x7D8E9CA1;
	else if (!strncmp(life_cycle, "LC_KILL_KEY0", len))
		value = 0xC29F604B;
	else {
		dev_err(dev, "invalid life cycle type!\n");
		return -EINVAL;
	}

	/*
	 * Check permission:
	 * Check it every time to avoid wp0~3 are changed somewhere
	 */
	efuse_permission_magic_config(priv->base, cmd_perm_magic_num, CMD_UPDATE_LC);

	/* Config life cycle */
	efuse_life_cycle_para_config(priv->base, value);

	/* set command */
	ret = efuse_cmd_start(priv->base, CON_CMD_UP_LC);
	if (ret)
		goto exit;

	/* Wait controller completed */
	ret = efuse_idle_check(priv->base);

exit:
	/* Check status, if there has error, reort and clear status */
	ret |= efuse_status_check(priv->base);
	if (ret)
		dev_err(dev, "error occurs while starting write\n");

	efuse_data_clear(priv->base);

	if (strncmp(life_cycle, "LC_RMA", len))
		goto out;

	dev_info(dev, "set LC_RMA life cycle\n");
	/* If target life cycle is RMA, close permission in teesystem regs */
	ret = regmap_read(priv->teesys_regs,
			  TEE_SYS_EFUSE_DBG_KEY1_OFF,
			  &data); /* Register from tee system */
	if (ret) {
		dev_err(dev, "failed to read data from DBG_KEY1 area\n");
		return ret;
	}

	data &= ~0xf;
	data |= 0xa;
	ret = regmap_write(priv->teesys_regs,
			   TEE_SYS_EFUSE_DBG_KEY1_OFF,
			   data);
	if (ret) {
		dev_err(dev, "failed to write data to DBG_KEY1 area\n");
		return ret;
	}

out:
	return ret < 0 ? ret : count;
}

static DEVICE_ATTR_WO(rma_lc);
static DEVICE_ATTR_WO(rip_lc);
static DEVICE_ATTR_RW(efuse_nvmem);
static DEVICE_ATTR_RO(lc_preld);
static DEVICE_ATTR_WO(update_lc);

static struct attribute *light_efuse_sysfs_entries[] = {
	&dev_attr_efuse_nvmem.attr,
	&dev_attr_rip_lc.attr,
	&dev_attr_rma_lc.attr,
	&dev_attr_lc_preld.attr,
	&dev_attr_update_lc.attr,
	NULL
};

static const struct attribute_group dev_attr_efuse_sysfs_group = {
	.attrs = light_efuse_sysfs_entries,
};

static const struct of_device_id light_efuse_of_match[] = {
	{.compatible = "thead,light-fm-efuse"},
	{ /* sentinel */},
};
MODULE_DEVICE_TABLE(of, light_efuse_of_match);

static int __maybe_unused light_efuse_runtime_suspend(struct device *dev)
{
	struct light_efuse_priv *priv = dev_get_drvdata(dev);
	u32 data;
	int ret;

	dev_dbg(dev, "[%s,%d]efuse runtime power down\n", __func__, __LINE__);

	if (!efuse_poweron_status_get(priv->base))
		return 0;

	data = readl(priv->base + CON);
	data |= EFUSE_CON_POWER_MSK;
	writel(data, priv->base + CON);

	ret = efuse_idle_check(priv->base);

	ret |= efuse_status_check(priv->base);

	dev_dbg(dev, "[%s,%d] ret = %d, pd status: 0x%lx\n", __func__, __LINE__, ret,
			readl(priv->base + CON) & EFUSE_CON_POWER_MSK);

	return ret;
}

static int __maybe_unused light_efuse_runtime_resume(struct device *dev)
{
	struct light_efuse_priv *priv = dev_get_drvdata(dev);

	dev_dbg(dev, "[%s,%d]efuse runtime power on\n", __func__, __LINE__);

	return efuse_poweron(priv->base);
}

static int light_efuse_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvmem_device *nvmem;
	struct nvmem_config config = {};
	struct light_efuse_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	/* optional clock, default open */
	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk))
		priv->clk = NULL;

	priv->teesys_regs = syscon_regmap_lookup_by_phandle(dev->of_node, "thead,teesys");
	if (IS_ERR(priv->teesys_regs)) {
		dev_err(dev, "unable to find teesys registers\n");
		return PTR_ERR(priv->teesys_regs);
	}

	priv->dev = dev;

	dev_set_drvdata(dev, priv);

	ret = sysfs_create_group(&dev->kobj, &dev_attr_efuse_sysfs_group);
	if (ret) {
		dev_err(dev, "failed to create efuse debug sysfs\n");
		return ret;
	}

	config.name = "light-efuse";
	config.read_only = false;
	config.stride = 1;
	config.word_size = 1; /* the least read and write unit on the upper level */
	config.reg_read = light_efuse_read;
	config.reg_write = light_efuse_write;
	config.size = LIGHT_EFUSE_LIT_BLOCK_NUM * LIGHT_EFUSE_LIT_BLOCK_WIDTH +
		LIGHT_EFUSE_BIG_BLOCK_NUM * LIGHT_EFUSE_BIG_BLOCK_WIDTH;
	config.priv = priv;
	config.dev = dev;

	nvmem = devm_nvmem_register(dev, &config);
	if (IS_ERR(nvmem))
		return PTR_ERR_OR_ZERO(nvmem);

	pm_runtime_enable(dev);

	dev_info(dev, "succeed to register light efuse driver\n");

	return 0;
}

static const struct dev_pm_ops efuse_runtime_pm_ops = {
	SET_RUNTIME_PM_OPS(light_efuse_runtime_suspend, light_efuse_runtime_resume, NULL)
};

static struct platform_driver light_efuse_driver = {
	.probe = light_efuse_probe,
	.driver = {
		.name = "light_efuse",
		.of_match_table = light_efuse_of_match,
		.pm = &efuse_runtime_pm_ops,
	},
};
module_platform_driver(light_efuse_driver);

MODULE_AUTHOR("wei.liu <lw312886@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead light nvmem driver");
MODULE_LICENSE("GPL v2");
