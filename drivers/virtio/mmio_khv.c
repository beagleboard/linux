// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/mmio_khv.h>
#include <uapi/linux/virtio_mmio.h>
#include <asm/sbi.h>

static struct mmio_khv_device *mk_dev;
static struct mmio_khv_device __mk_dev;

static bool init_flag = false;
void khv_shutdown(void)
{
	u64 stat = START | (0x5a << 16);

	if (!init_flag)
		return;

	writel(stat, mk_dev->base + KHV_STAT_OFF);

	/* notify host */
	writel(mk_dev->notify_host_reg_val, mk_dev->notify_host_reg);

	sbi_ecall(SBI_EXT_HSM, SBI_EXT_HSM_HART_STOP, 0, 0, 0, 0, 0, 0);
}

static void init_mk_dev(void)
{
	if (init_flag == true)
		return;

	init_flag = true;

	mk_dev = &__mk_dev;

	mk_dev->base = ioremap(0x1fd000, 0x100);
	memset(mk_dev->base, 0, 0x100);

#ifdef CONFIG_SOC_INT_SRC7
	mk_dev->notify_host_reg = ioremap(0xFFFF019094, 4);
#else
	mk_dev->notify_host_reg = ioremap(0xffffc3b010, 4);
#endif
	mk_dev->notify_host_reg_val = 1;
}

u64 readx(u32 offset, u32 bw)
{
	u64 val;
	u16 stat = START | (bw << 8);

	init_mk_dev();

	/* wait host ready */
	while (readb(mk_dev->base + KHV_STAT_OFF) != INVAL)
		cpu_relax();

	writew(stat, mk_dev->base + KHV_STAT_OFF);
	writel(offset, mk_dev->base + KHV_ADDR_OFF);

	/* notify host */
	writel(mk_dev->notify_host_reg_val, mk_dev->notify_host_reg);

	/* wait host ready */
	while (readb(mk_dev->base + KHV_STAT_OFF) != READY)
		cpu_relax();

	switch (bw) {
	case WIDTH_8:
		val = readb(mk_dev->base + KHV_VAL_OFF);
		break;
	case WIDTH_16:
		val = readw(mk_dev->base + KHV_VAL_OFF);
		break;
	case WIDTH_32:
		val = readl(mk_dev->base + KHV_VAL_OFF);
		break;
	case WIDTH_64:
		val = readq(mk_dev->base + KHV_VAL_OFF);
		break;
	default:
		BUG();
	}

	writeb(ACK, mk_dev->base + KHV_STAT_OFF);

	return val;
}

void writex(u64 val, u32 offset, u32 bw)
{
	u16 stat = START | IS_WRITE | (bw << 8);

	init_mk_dev();

	/* wait host ready */
	while (readb(mk_dev->base + KHV_STAT_OFF) != INVAL)
		cpu_relax();

	writew(stat, mk_dev->base + KHV_STAT_OFF);
	writel(offset, mk_dev->base + KHV_ADDR_OFF);

	switch (bw) {
	case WIDTH_8:
		writeb(val, mk_dev->base + KHV_VAL_OFF);
		break;
	case WIDTH_16:
		writew(val, mk_dev->base + KHV_VAL_OFF);
		break;
	case WIDTH_32:
		writel(val, mk_dev->base + KHV_VAL_OFF);
		break;
	case WIDTH_64:
		writeq(val, mk_dev->base + KHV_VAL_OFF);
		break;
	default:
		BUG();
	}

	/* notify host */
	writel(mk_dev->notify_host_reg_val, mk_dev->notify_host_reg);

	if ((offset & 0xff) != VIRTIO_MMIO_QUEUE_NOTIFY) {
		/* wait host ready */
		while (readb(mk_dev->base + KHV_STAT_OFF) != READY)
			cpu_relax();

		writeb(ACK, mk_dev->base + KHV_STAT_OFF);
	}

	return;
}
