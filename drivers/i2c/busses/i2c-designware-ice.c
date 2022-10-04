#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/slab.h>

#define DW_IC_MAX_I2C_DEV     ( 6 )

#define I2C_CLK     (24*1000*1000)

#define I2C_SPEED   (400*1000)

#define I2C_BUF_DEP ( 128 )

#define I2C_FIFO_DATA_CNT ( 4 )

#define DW_IC_CON                   (0x00)
#define DW_IC_TAR                   (0x04)
#define DW_IC_DATA_CMD              (0x10)
#define DW_IC_SS_SCL_HCNT           (0x14)
#define DW_IC_SS_SCL_LCNT           (0x18)
#define DW_IC_FS_SCL_HCNT           (0x1c)
#define DW_IC_FS_SCL_LCNT           (0x20)
#define DW_IC_HS_SCL_HCNT           (0x24)
#define DW_IC_HS_SCL_LCNT           (0x28)
#define DW_IC_INTR_STAT             (0x2c)
#define DW_IC_INTR_MASK             (0x30)
#define DW_IC_RAW_INTSTATUS         (0x34)
#define DW_IC_RX_TL                 (0x38)
#define DW_IC_TX_TL                 (0x3c)
#define DW_IC_CLR_INTR              (0x40)
#define DW_IC_CLR_RX_UNDER          (0x44)
#define DW_IC_CLR_RX_OVER           (0x48)
#define DW_IC_CLR_TX_OVER           (0x4c)
#define DW_IC_CLR_RD_REQ            (0x50)
#define DW_IC_CLR_TX_ABRT           (0x54)
#define DW_IC_CLR_RX_DONE           (0x58)
#define DW_IC_CLR_ACTIVITY          (0x5c)
#define DW_IC_CLR_STOP_DET          (0x60)
#define DW_IC_CLR_START_DET         (0x64)
#define DW_IC_ENABLE                (0x6c)
#define DW_IC_STATUS                (0x70)
#define DW_IC_TXFLR                 (0x74)
#define DW_IC_RXFLR                 (0x78)
#define DW_IC_TX_ABRT_SOURCE        (0x80)
#define DW_IC_DMA_CR                (0x88)
#define DW_IC_DMA_TDLR              (0x8c)
#define DW_IC_DMA_RDLR              (0x90)
#define DW_IC_ENABLE_STATUS         (0x9c)
#define DW_IC_START                 (0xa0)

#define DW_IC_BIT_INT_STOP            BIT(9)
#define DW_IC_BIT_INT_FREE            BIT(8)
#define DW_IC_BIT_INT_TX_ABRT         BIT(6)

#define I2C_ENABLES     (1)
#define I2C_DISABLES    (0)
#define I2C_START       (1)
#define I2C_STOP        (0)

/* IC_CON */
#define DW_IC_BIT_MASTER_MODE         BIT(0)
#define DW_IC_BIT_SPDMD_FAST          BIT(2)
#define DW_IC_BIT_10_ADDR             BIT(4)
#define DW_IC_BIT_RESTART_EN          BIT(5)

#define DW_IC_BITSHF_SPDMODE          (0x01)
#define DW_IC_BITMASK_SPDMODE         (~0x06)

/* IC_STATUS */
#define DW_IC_BITMASK_ACTIVITY        (0x01)
#define DW_IC_BIT_ACTIVITY            (0x01)
#define DW_IC_BIT_TFE                 BIT(2)
#define DW_IC_BIT_RFNE                BIT(3)

/* IC_DATA_CMD */
#define DW_IC_BIT_CDM_READ            BIT(8)

#define I2C_GROUP_CMD_CNT        (256)

#define DW_IC_I2C_SPD_STANDARD    (100 * 1000)
#define DW_IC_I2C_SPD_FAST        (400 * 1000)
#define DW_IC_I2C_SPD_HIGH        (3400 * 1000)

typedef enum {
	DW_IC_I2C_SPDMODE_REV = 0,
	DW_IC_I2C_SPDMODE_STD,
	DW_IC_I2C_SPDMODE_FAST,
	DW_IC_I2C_SPDMODE_HIGH,
} DW_IC_I2C_SPDMODE_E;

typedef enum
{
	STATUS_IDLE      = 0,
	STATUS_HANDLING  = 1,
	STATUS_DONE      = 2,
	STATUS_ERROR     = 3,
	STATUS_BUTT   = 4,
}DW_IC_I2C_GROUP_STATUS_E;

typedef struct
{
	int devid;
	int reg_addr;
	int nofifo;
	int cmd_cnt;
	int cmd_pos;
	unsigned short cmdBuf[I2C_GROUP_CMD_CNT];
}DW_IC_I2C_GROUP_CMD_S;

#define DW_IC_I2C_CMD_BUF_DEP	(I2C_BUF_DEP * 2)
struct dw_i2c_dev
{
	void __iomem    *base;
	int             irq;

	struct mutex    lock;
	wait_queue_head_t   wait;
	int            complete;
	int             abort;
	struct i2c_adapter  adapter;

	unsigned short cmdBuf[DW_IC_I2C_CMD_BUF_DEP]; /* cmds for RD and WR */
};

void dw_misc_reg_write(unsigned int value,void *base,unsigned int offset)
{
	writel((value), base + offset);
}

unsigned int dw_misc_getreg(void *base,int offset)
{
	return readl(base + offset);
}

static unsigned int dw_ic_spd_mode;
static long i2c_base_addr[DW_IC_MAX_I2C_DEV];

DW_IC_I2C_GROUP_STATUS_E dw_group_status[DW_IC_MAX_I2C_DEV];
struct mutex    lock[DW_IC_MAX_I2C_DEV];
DW_IC_I2C_GROUP_CMD_S dw_group_cmd[DW_IC_MAX_I2C_DEV];


int dw_i2c_enable(void *base,unsigned int enabel)
{
	dw_misc_reg_write( enabel,base,DW_IC_ENABLE);
	return 0;
}

int dw_i2c_interrupt_init(void *base)
{
	unsigned int intr = 0;

	dw_misc_getreg(base,DW_IC_CLR_INTR);
	intr |= (DW_IC_BIT_INT_FREE | DW_IC_BIT_INT_TX_ABRT);
	dw_misc_reg_write( intr,base,DW_IC_INTR_MASK);
	return 0;
}

int dw_i2c_clk_config(void *base, unsigned int bus_clk, unsigned int clk)
{
	unsigned short cnt_hi,cnt_lo;
	unsigned int local_spd_mode,tmp;

	if (clk > DW_IC_I2C_SPD_FAST) {
		local_spd_mode = DW_IC_I2C_SPDMODE_HIGH;
	} else if (clk > DW_IC_I2C_SPD_STANDARD) {
		local_spd_mode = DW_IC_I2C_SPDMODE_FAST;
	} else {
		local_spd_mode = DW_IC_I2C_SPDMODE_STD;
	}

	if (dw_ic_spd_mode != local_spd_mode) {
		tmp = dw_misc_getreg(base, DW_IC_CON);
		tmp &= DW_IC_BITMASK_SPDMODE;
		tmp |= (local_spd_mode << DW_IC_BITSHF_SPDMODE);
		dw_misc_reg_write(tmp, base, DW_IC_CON);

		dw_ic_spd_mode = local_spd_mode;
	}

	cnt_hi = (bus_clk / clk) >> 1;
	cnt_lo = (bus_clk / clk) - cnt_hi;

	if (local_spd_mode == DW_IC_I2C_SPDMODE_HIGH) {
		dw_misc_reg_write( cnt_hi - 8,base,DW_IC_HS_SCL_HCNT);
		dw_misc_reg_write( cnt_lo - 1,base,DW_IC_HS_SCL_LCNT);
	} else if (local_spd_mode == DW_IC_I2C_SPDMODE_FAST) {
		dw_misc_reg_write( cnt_hi - 8,base,DW_IC_FS_SCL_HCNT);
		dw_misc_reg_write( cnt_lo - 1,base,DW_IC_FS_SCL_LCNT);
	} else {
		dw_misc_reg_write( cnt_hi - 8,base,DW_IC_SS_SCL_HCNT);
		dw_misc_reg_write( cnt_lo - 1,base,DW_IC_SS_SCL_LCNT);
	}

	return 0;
}

int dw_i2c_config_set(void *base)
{
	unsigned int cfg;

	cfg = DW_IC_BIT_MASTER_MODE | DW_IC_BIT_SPDMD_FAST | (1 << 6) \
		  | DW_IC_BIT_RESTART_EN;
	dw_misc_reg_write(cfg, base, DW_IC_CON);

	return 0;
}


int dw_i2c_SetSlaveAddr(void *base,unsigned int addr)
{
	dw_misc_reg_write( addr,base,DW_IC_TAR);
	return 0;
}

int dw_i2c_WaitTransmitOver(void *base,unsigned int fifoFlag)
{
	int i;
	unsigned int tmp;

	for (i = 0; i < 0xffff; i++) {
		tmp = dw_misc_getreg(base,DW_IC_STATUS);
		if ( DW_IC_BIT_ACTIVITY != (tmp & DW_IC_BITMASK_ACTIVITY)
				&& (tmp & fifoFlag )) {
			return 0;
		}
	}

	return -1;
}

int  dw_i2c_SetStart(void *base,unsigned int u32Enable)
{
	dw_misc_reg_write(u32Enable, base, DW_IC_START);

	return 0;
}


unsigned int dw_i2c_GetIntStatus(void *base)
{
	return dw_misc_getreg(base,DW_IC_INTR_STAT);
}

unsigned int dw_i2c_GetRecFifoCnt(void *base)
{
	return dw_misc_getreg(base,DW_IC_RXFLR);
}

int dw_i2c_SetGroupSendStatus(unsigned int i2c_ctl_index,DW_IC_I2C_GROUP_STATUS_E status)
{
	if(i2c_ctl_index > DW_IC_MAX_I2C_DEV - 1) {
		return -1;
	}

	dw_group_status[i2c_ctl_index] = status;

	return 0;
}

DW_IC_I2C_GROUP_STATUS_E dw_i2c_GetGroupSendStatus(unsigned int i2c_ctl_index)
{
	if(i2c_ctl_index > DW_IC_MAX_I2C_DEV - 1) {
		return -1;
	}

	return dw_group_status[i2c_ctl_index];
}

int dw_i2c_WriteFifo(unsigned int i2c_ctl_index,unsigned int hardgroup)
{
	int total_cnt;
	unsigned short cmd;
	unsigned int reg_addr,start = 0;
	void *lpRegBase;

	if (i2c_base_addr[i2c_ctl_index] != 0) {
		lpRegBase = (void *)i2c_base_addr[i2c_ctl_index];
	} else {
		return -1;
	}

	if (dw_group_cmd[i2c_ctl_index].nofifo == 1) {
		reg_addr = dw_group_cmd[i2c_ctl_index].reg_addr;
		if((reg_addr & 0xff00) != 0) {
			dw_misc_reg_write( ((reg_addr & 0xff00) >> 8) | (1 << 9) ,lpRegBase,DW_IC_DATA_CMD);
			dw_misc_reg_write( reg_addr & 0xff ,lpRegBase,DW_IC_DATA_CMD);
		} else {
			dw_misc_reg_write( (reg_addr & 0xff) | (1 << 9) ,lpRegBase,DW_IC_DATA_CMD);
		}

		dw_group_cmd[i2c_ctl_index].nofifo = 0;
	}

	while(1) {
		cmd = dw_group_cmd[i2c_ctl_index].cmdBuf[dw_group_cmd[i2c_ctl_index].cmd_pos];
		if((cmd & (1 << 9)) != 0) {
			if(hardgroup == 0) {
				if(start != 0)
					break;
				else
					start = 1;
			}
			total_cnt = dw_misc_getreg(lpRegBase,DW_IC_TXFLR);

			if(I2C_BUF_DEP - total_cnt < 4) {
				break;
			}

			dw_group_cmd[i2c_ctl_index].reg_addr = cmd;
			dw_misc_reg_write( cmd ,lpRegBase,DW_IC_DATA_CMD);
			dw_group_cmd[i2c_ctl_index].cmd_pos++;
			dw_group_cmd[i2c_ctl_index].cmd_cnt--;
			cmd = dw_group_cmd[i2c_ctl_index].cmdBuf[dw_group_cmd[i2c_ctl_index].cmd_pos];

			if((cmd & (1 << 9)) != 0) {
				dw_group_cmd[i2c_ctl_index].reg_addr <<= 8;
				dw_group_cmd[i2c_ctl_index].reg_addr += cmd & 0xff;
				dw_misc_reg_write( cmd & 0xff ,lpRegBase,DW_IC_DATA_CMD);
				dw_group_cmd[i2c_ctl_index].cmd_pos++;
				dw_group_cmd[i2c_ctl_index].cmd_cnt--;
			}
		} else {
			dw_misc_reg_write( cmd ,lpRegBase,DW_IC_DATA_CMD);
			dw_group_cmd[i2c_ctl_index].cmd_pos++;
			dw_group_cmd[i2c_ctl_index].cmd_cnt--;
			dw_group_cmd[i2c_ctl_index].reg_addr++;
		}

		if (dw_group_cmd[i2c_ctl_index].cmd_cnt <= 0) {
			break;
		}

		total_cnt = dw_misc_getreg(lpRegBase,DW_IC_TXFLR);
		if (total_cnt >= I2C_BUF_DEP) {
			cmd = dw_group_cmd[i2c_ctl_index].cmdBuf[dw_group_cmd[i2c_ctl_index].cmd_pos];
			if ((cmd & (1 << 9)) == 0) {
				dw_group_cmd[i2c_ctl_index].nofifo = 1;
			}
			break;
		}

	}
	return 0;
}

void dw_i2c_handle_irq(void *dev_id)
{
	unsigned int status;
	DW_IC_I2C_GROUP_STATUS_E groupStatus;
	struct dw_i2c_dev *dev = dev_id;
	groupStatus = dw_i2c_GetGroupSendStatus(dev->adapter.nr);
	status = dw_i2c_GetIntStatus(dev->base);

	if(groupStatus == STATUS_HANDLING) {
		if(status & DW_IC_BIT_INT_TX_ABRT) {
			dw_i2c_SetGroupSendStatus(dev->adapter.nr,STATUS_ERROR);
			dw_i2c_SetStart(dev->base,I2C_STOP);
			dw_i2c_enable(dev->base,I2C_DISABLES);
			return;
		}

		if(dw_group_cmd[dev->adapter.nr].cmd_cnt > 0) {
			dw_i2c_enable(dev->base,I2C_DISABLES);
			dw_i2c_interrupt_init(dev->base);
			dw_i2c_SetSlaveAddr(dev->base,dw_group_cmd[dev->adapter.nr].devid >> 1);
			dw_i2c_enable(dev->base,I2C_ENABLES);
			dw_i2c_WriteFifo( dev->adapter.nr ,1);
			dw_i2c_SetStart(dev->base,I2C_START);
		} else {
			dw_i2c_SetGroupSendStatus(dev->adapter.nr,STATUS_DONE);
			dw_i2c_SetStart(dev->base,I2C_STOP);
			dw_i2c_enable(dev->base,I2C_DISABLES);
		}

	} else {
		if(status & DW_IC_BIT_INT_TX_ABRT) {
			dev->abort = true;
		}
		dev->complete = true;
		dw_i2c_interrupt_init(dev->base);
		wake_up(&dev->wait);
	}
}

int dw_i2c_SendMsgs(struct dw_i2c_dev *dev, struct i2c_msg *msgs, int num )
{
	unsigned int timeout,i;
	int ret = num;
	unsigned short *cmd_dat_ptr = dev->cmdBuf;

	if(num != 1 || msgs->len > I2C_BUF_DEP)
		return -1;

	for(i = 0; i < msgs->len; i++)
		cmd_dat_ptr[i] = msgs->buf[i];
	dev->complete = false;
	dev->abort    = false;
	dw_i2c_enable(dev->base,I2C_DISABLES);
	dw_i2c_interrupt_init(dev->base);
	dw_i2c_SetSlaveAddr(dev->base,msgs->addr);
	dw_i2c_enable(dev->base,I2C_ENABLES);

	for(i = 0; i < msgs->len; i++ )
		dw_misc_reg_write( cmd_dat_ptr[i],dev->base,DW_IC_DATA_CMD);

	dw_i2c_SetStart(dev->base,I2C_START);

	timeout = wait_event_timeout(dev->wait, dev->complete == true, HZ * 3);

	if(0 == timeout || dw_i2c_WaitTransmitOver(dev->base,DW_IC_BIT_TFE))
		ret = 0;
	else if(dev->abort == true)
		ret = 0;
	else if(dev->complete != true)
		ret = 0;

	dw_i2c_SetStart(dev->base,I2C_STOP);
	dw_i2c_enable(dev->base,I2C_DISABLES);
	return ret;
}

int dw_i2c_ReceiveMsgs(struct dw_i2c_dev *dev, struct i2c_msg *msgs, int num)
{
	unsigned int max_cnt,timeout,i,j;
	int ret = num;
	unsigned short *cmd_dat_ptr;
	struct i2c_msg *read_msg,*write_msg;
	write_msg = msgs;
	read_msg = msgs + 1;
	cmd_dat_ptr = dev->cmdBuf;
	max_cnt = read_msg->len + write_msg->len;

	if (max_cnt > DW_IC_I2C_CMD_BUF_DEP) {
		printk("i2c read cnt %d,and write cnt %d,bigger than fifo dep %d\n",
				read_msg->len, write_msg->len, DW_IC_I2C_CMD_BUF_DEP);
		return -1;
	}

	for (i = 0; i < write_msg->len; i++) {
		cmd_dat_ptr[i] = write_msg->buf[i];
		cmd_dat_ptr[i] &= ~(DW_IC_BIT_CDM_READ);
	}

	for( j = 0; j < read_msg->len; j++, i++ ) {
		cmd_dat_ptr[i] = 0;
		cmd_dat_ptr[i] |= DW_IC_BIT_CDM_READ;
	}

	dev->complete = false;
	dev->abort = false;

	dw_i2c_enable(dev->base,I2C_DISABLES);
	dw_i2c_interrupt_init(dev->base);
	dw_i2c_SetSlaveAddr(dev->base,msgs->addr);
	dw_i2c_enable(dev->base,I2C_ENABLES);

	for(j = 0; j < i; j++ )
		dw_misc_reg_write( cmd_dat_ptr[j],dev->base,DW_IC_DATA_CMD);

	dw_i2c_SetStart(dev->base,I2C_START);

	timeout = wait_event_timeout(dev->wait, dev->complete == true, HZ * 3);

	if(0 == timeout || dw_i2c_WaitTransmitOver(dev->base,DW_IC_BIT_RFNE))
		ret = 0;
	else if(dev->abort == true)
		ret = 0;
	else if((dev->complete != true))
		ret = 0;

	max_cnt = dw_i2c_GetRecFifoCnt(dev->base);

	for(i = 0; i < max_cnt; i++)
		read_msg->buf[i] = dw_misc_getreg(dev->base,DW_IC_DATA_CMD);

	dw_i2c_SetStart(dev->base,I2C_STOP);
	dw_i2c_enable(dev->base,I2C_DISABLES);
	return ret;
}

int dw_i2c_initCmdBuf(unsigned int i2c_ctl_index)
{
	dw_group_cmd[i2c_ctl_index].devid = 0;
	dw_group_cmd[i2c_ctl_index].reg_addr = 0;
	dw_group_cmd[i2c_ctl_index].cmd_cnt = 0;
	dw_group_cmd[i2c_ctl_index].cmd_pos = 0;
	dw_group_cmd[i2c_ctl_index].nofifo = 0;
	return 0;
}

int dw_i2c_initLock(unsigned int i2c_ctl_index)
{
	mutex_init(&lock[i2c_ctl_index]);
	return 0;
}

int dw_i2c_get_lock(unsigned int i2c_ctl_index)
{
	if(dw_group_status[i2c_ctl_index] != STATUS_IDLE)
	{
		return -1;
	}
	mutex_lock(&lock[i2c_ctl_index]);
	return 0;
}

int dw_i2c_FreeLock(unsigned int i2c_ctl_index)
{
	mutex_unlock(&lock[i2c_ctl_index]);
	return 0;
}

int dw_i2c_uninit(void *base, unsigned int i2c_ctl_index)
{
	i2c_base_addr[i2c_ctl_index] = 0;

	return 0;
}

int dw_i2c_init(void *base, unsigned int bus_clk, unsigned int clk, unsigned int i2c_ctl_index)
{
	dw_ic_spd_mode = DW_IC_I2C_SPDMODE_REV;
	dw_i2c_enable(base, I2C_DISABLES);
	dw_i2c_interrupt_init(base);
	dw_i2c_config_set(base);
	dw_i2c_clk_config(base, bus_clk, clk);

	i2c_base_addr[i2c_ctl_index] = (long)base;

	return 0;
}

static unsigned int dw_i2c_id_index;

static irqreturn_t dw_i2c_irq(int irq, void *dev_id)
{
	dw_i2c_handle_irq(dev_id);

	return IRQ_HANDLED;
}

int dw_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	int ret;
	struct dw_i2c_dev *dev = i2c_get_adapdata(adap);
	dw_i2c_get_lock(adap->nr);

	if (num > 1 && !(msgs->flags & I2C_M_RD)) {
		ret = dw_i2c_ReceiveMsgs(dev, msgs,  num);
	} else {
		ret = dw_i2c_SendMsgs(dev, msgs,  num);
	}

	dw_i2c_FreeLock(adap->nr);
	return ret;
}

static unsigned int dw_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING;
}

static struct i2c_algorithm dw_i2c_algorithm = {
	.master_xfer    = dw_i2c_xfer,
	.functionality  = dw_i2c_func,
};

static int dw_i2c_probe(struct platform_device *pdev)
{
	struct resource *mem,*ioarea;
	int ret,irq;
	struct dw_i2c_dev *dev;
	struct i2c_adapter *adap;
	struct clk *bus_clk;
	unsigned int freq, bus_hz;

	pdev->id = dw_i2c_id_index;
	dw_i2c_id_index++;

	/* map the registers */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem == NULL)
	{
		dev_err(&pdev->dev, "no mem resource\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
	{
		dev_err(&pdev->dev, "cannot find IRQ\n");
		return irq; /* -ENXIO */
	}

	ioarea = request_mem_region(mem->start, resource_size(mem),pdev->name);
	if (ioarea == NULL)
	{
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -EBUSY;
	}

	dev = kzalloc(sizeof(struct dw_i2c_dev), GFP_KERNEL);
	if (!dev)
	{
		ret = -ENOMEM;
		goto err_release_region;
	}

	dev->irq = irq;
	platform_set_drvdata(pdev, dev);

	dev->base = ioremap(mem->start, resource_size(mem));
	if (dev->base == NULL)
	{
		dev_err(&pdev->dev, "failure mapping io resources\n");
		ret = -EBUSY;
		goto err_free_mem;
	}

	init_waitqueue_head(&dev->wait);

	ret = request_irq(dev->irq, dw_i2c_irq, 0,dev_name(&pdev->dev), dev);
	if (ret != 0)
	{
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", dev->irq);
		goto err_iounmap;
	}

	bus_clk = devm_clk_get(&(pdev->dev), NULL);
	if (IS_ERR(bus_clk)) {
		bus_hz = I2C_CLK;
	} else {
		bus_hz = clk_get_rate(bus_clk);
	}
	printk("i2c bus %dHz\n", bus_hz);

	if (device_property_read_u32(&(pdev->dev), "clock-frequency", &freq)) {
		freq = I2C_SPEED;
	} else {
		printk("get freq=%d\n", freq);
	}

	dw_i2c_init(dev->base, bus_hz, freq, pdev->id);
	dw_i2c_initLock(pdev->id);
	dw_i2c_initCmdBuf(pdev->id);
	dw_i2c_SetGroupSendStatus(pdev->id, STATUS_IDLE );
	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->owner   = THIS_MODULE;
	adap->algo    = &dw_i2c_algorithm;
	adap->class   = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	adap->dev.of_node = pdev->dev.of_node;
	strlcpy(adap->name, "Synopsys DesignWare I2C adapter", sizeof(adap->name));
	adap->dev.parent = &pdev->dev;
	adap->nr = pdev->id;
	ret = i2c_add_numbered_adapter(adap);
	if (ret < 0)
	{
		dev_err(&pdev->dev, "failed to add bus to i2c core\n");
		goto err_free_irq;
	}

	return 0;

err_free_irq:
	free_irq(dev->irq, dev);
	dw_i2c_uninit(dev->base, pdev->id);

err_iounmap:
	iounmap(dev->base);
err_free_mem:
	platform_set_drvdata(pdev, NULL);
	kfree(dev);

err_release_region:
	release_mem_region(mem->start, resource_size(mem));

	return ret;

}

static int   dw_i2c_remove(struct platform_device *pdev)
{
	struct dw_i2c_dev *dev = platform_get_drvdata(pdev);
	struct resource *mem;
	dw_i2c_uninit(dev->base, pdev->id);
	platform_set_drvdata(pdev, NULL);
	i2c_del_adapter(&dev->adapter);

	free_irq(dev->irq, dev);
	iounmap(dev->base);
	kfree(dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));

	return 0;
}

static const struct of_device_id dh_i2c_of_match[] = {
	{ .compatible = "snps,designware-i2c-ice"},
};
MODULE_DEVICE_TABLE(of, dh_i2c_of_match);


static struct platform_driver dw_i2c_driver = {
	.probe  = dw_i2c_probe,
	.remove = dw_i2c_remove,
	.driver = {
		.name   = "dw_i2c",
		.of_match_table  = dh_i2c_of_match,
	},
};

static int __init dw_i2c_bus_ice_init(void)
{
	int s32Ret;

	s32Ret = platform_driver_register(&dw_i2c_driver);
	if(s32Ret < 0)
	{
		return s32Ret;
	}

	return 0;
}

static void __exit dw_i2c_bus_ice_exit(void)
{
	platform_driver_unregister(&dw_i2c_driver);
}

module_init(dw_i2c_bus_ice_init);
module_exit(dw_i2c_bus_ice_exit);
