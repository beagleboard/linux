/*
 *  linux/arch/arm/mach-omap/gpio.c
 *
 * Support functions for OMAP GPIO
 *
 * Copyright (C) 2003 Nokia Corporation
 * Written by Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/sysdev.h>
#include <linux/err.h>

#include <asm/hardware.h>
#include <asm/hardware/clock.h>
#include <asm/irq.h>
#include <asm/arch/irqs.h>
#include <asm/arch/gpio.h>
#include <asm/mach/irq.h>

#include <asm/io.h>

/*
 * OMAP1510 GPIO registers
 */
#define OMAP1510_GPIO_BASE		0xfffce000
#define OMAP1510_GPIO_DATA_INPUT	0x00
#define OMAP1510_GPIO_DATA_OUTPUT	0x04
#define OMAP1510_GPIO_DIR_CONTROL	0x08
#define OMAP1510_GPIO_INT_CONTROL	0x0c
#define OMAP1510_GPIO_INT_MASK		0x10
#define OMAP1510_GPIO_INT_STATUS	0x14
#define OMAP1510_GPIO_PIN_CONTROL	0x18

#define OMAP1510_IH_GPIO_BASE		64

/*
 * OMAP1610 specific GPIO registers
 */
#define OMAP1610_GPIO1_BASE		0xfffbe400
#define OMAP1610_GPIO2_BASE		0xfffbec00
#define OMAP1610_GPIO3_BASE		0xfffbb400
#define OMAP1610_GPIO4_BASE		0xfffbbc00
#define OMAP1610_GPIO_REVISION		0x0000
#define OMAP1610_GPIO_SYSCONFIG		0x0010
#define OMAP1610_GPIO_SYSSTATUS		0x0014
#define OMAP1610_GPIO_IRQSTATUS1	0x0018
#define OMAP1610_GPIO_IRQENABLE1	0x001c
#define OMAP1610_GPIO_WAKEUPENABLE	0x0028
#define OMAP1610_GPIO_DATAIN		0x002c
#define OMAP1610_GPIO_DATAOUT		0x0030
#define OMAP1610_GPIO_DIRECTION		0x0034
#define OMAP1610_GPIO_EDGE_CTRL1	0x0038
#define OMAP1610_GPIO_EDGE_CTRL2	0x003c
#define OMAP1610_GPIO_CLEAR_IRQENABLE1	0x009c
#define OMAP1610_GPIO_CLEAR_WAKEUPENA	0x00a8
#define OMAP1610_GPIO_CLEAR_DATAOUT	0x00b0
#define OMAP1610_GPIO_SET_IRQENABLE1	0x00dc
#define OMAP1610_GPIO_SET_WAKEUPENA	0x00e8
#define OMAP1610_GPIO_SET_DATAOUT	0x00f0

/*
 * OMAP730 specific GPIO registers
 */
#define OMAP730_GPIO1_BASE		0xfffbc000
#define OMAP730_GPIO2_BASE		0xfffbc800
#define OMAP730_GPIO3_BASE		0xfffbd000
#define OMAP730_GPIO4_BASE		0xfffbd800
#define OMAP730_GPIO5_BASE		0xfffbe000
#define OMAP730_GPIO6_BASE		0xfffbe800
#define OMAP730_GPIO_DATA_INPUT		0x00
#define OMAP730_GPIO_DATA_OUTPUT	0x04
#define OMAP730_GPIO_DIR_CONTROL	0x08
#define OMAP730_GPIO_INT_CONTROL	0x0c
#define OMAP730_GPIO_INT_MASK		0x10
#define OMAP730_GPIO_INT_STATUS		0x14

#define OMAP_MPUIO_MASK		(~OMAP_MAX_GPIO_LINES & 0xff)

struct gpio_bank {
	u32 base;
	u16 irq;
	u16 virtual_irq_start;
	u8 method;
	u32 reserved_map;
	u32 suspend_wakeup;
	u32 saved_wakeup;
	spinlock_t lock;
};

#define METHOD_MPUIO		0
#define METHOD_GPIO_1510	1
#define METHOD_GPIO_1610	2
#define METHOD_GPIO_730		3

#if defined(CONFIG_ARCH_OMAP16XX)
static struct gpio_bank gpio_bank_1610[5] = {
	{ OMAP_MPUIO_BASE,     INT_MPUIO,	    IH_MPUIO_BASE,     METHOD_MPUIO},
	{ OMAP1610_GPIO1_BASE, INT_GPIO_BANK1,	    IH_GPIO_BASE,      METHOD_GPIO_1610 },
	{ OMAP1610_GPIO2_BASE, INT_1610_GPIO_BANK2, IH_GPIO_BASE + 16, METHOD_GPIO_1610 },
	{ OMAP1610_GPIO3_BASE, INT_1610_GPIO_BANK3, IH_GPIO_BASE + 32, METHOD_GPIO_1610 },
	{ OMAP1610_GPIO4_BASE, INT_1610_GPIO_BANK4, IH_GPIO_BASE + 48, METHOD_GPIO_1610 },
};
#endif

#ifdef CONFIG_ARCH_OMAP1510
static struct gpio_bank gpio_bank_1510[2] = {
	{ OMAP_MPUIO_BASE,    INT_MPUIO,      IH_MPUIO_BASE, METHOD_MPUIO },
	{ OMAP1510_GPIO_BASE, INT_GPIO_BANK1, IH_GPIO_BASE,  METHOD_GPIO_1510 }
};
#endif

#ifdef CONFIG_ARCH_OMAP730
static struct gpio_bank gpio_bank_730[7] = {
	{ OMAP_MPUIO_BASE,     INT_730_MPUIO,	    IH_MPUIO_BASE,	METHOD_MPUIO },
	{ OMAP730_GPIO1_BASE,  INT_730_GPIO_BANK1,  IH_GPIO_BASE,	METHOD_GPIO_730 },
	{ OMAP730_GPIO2_BASE,  INT_730_GPIO_BANK2,  IH_GPIO_BASE + 32,	METHOD_GPIO_730 },
	{ OMAP730_GPIO3_BASE,  INT_730_GPIO_BANK3,  IH_GPIO_BASE + 64,	METHOD_GPIO_730 },
	{ OMAP730_GPIO4_BASE,  INT_730_GPIO_BANK4,  IH_GPIO_BASE + 96,	METHOD_GPIO_730 },
	{ OMAP730_GPIO5_BASE,  INT_730_GPIO_BANK5,  IH_GPIO_BASE + 128, METHOD_GPIO_730 },
	{ OMAP730_GPIO6_BASE,  INT_730_GPIO_BANK6,  IH_GPIO_BASE + 160, METHOD_GPIO_730 },
};
#endif

static struct gpio_bank *gpio_bank;
static int gpio_bank_count;

static inline struct gpio_bank *get_gpio_bank(int gpio)
{
#ifdef CONFIG_ARCH_OMAP1510
	if (cpu_is_omap1510()) {
		if (OMAP_GPIO_IS_MPUIO(gpio))
			return &gpio_bank[0];
		return &gpio_bank[1];
	}
#endif
#if defined(CONFIG_ARCH_OMAP16XX)
	if (cpu_is_omap16xx()) {
		if (OMAP_GPIO_IS_MPUIO(gpio))
			return &gpio_bank[0];
		return &gpio_bank[1 + (gpio >> 4)];
	}
#endif
#ifdef CONFIG_ARCH_OMAP730
	if (cpu_is_omap730()) {
		if (OMAP_GPIO_IS_MPUIO(gpio))
			return &gpio_bank[0];
		return &gpio_bank[1 + (gpio >> 5)];
	}
#endif
}

static inline int get_gpio_index(int gpio)
{
	if (cpu_is_omap730())
		return gpio & 0x1f;
	else
		return gpio & 0x0f;
}

static inline int gpio_valid(int gpio)
{
	if (gpio < 0)
		return -1;
	if (OMAP_GPIO_IS_MPUIO(gpio)) {
		if ((gpio & OMAP_MPUIO_MASK) > 16)
			return -1;
		return 0;
	}
#ifdef CONFIG_ARCH_OMAP1510
	if (cpu_is_omap1510() && gpio < 16)
		return 0;
#endif
#if defined(CONFIG_ARCH_OMAP16XX)
	if ((cpu_is_omap16xx()) && gpio < 64)
		return 0;
#endif
#ifdef CONFIG_ARCH_OMAP730
	if (cpu_is_omap730() && gpio < 192)
		return 0;
#endif
	return -1;
}

static int check_gpio(int gpio)
{
	if (unlikely(gpio_valid(gpio)) < 0) {
		printk(KERN_ERR "omap-gpio: invalid GPIO %d\n", gpio);
		dump_stack();
		return -1;
	}
	return 0;
}

static void _set_gpio_direction(struct gpio_bank *bank, int gpio, int is_input)
{
	u32 reg = bank->base;
	u32 l;

	switch (bank->method) {
	case METHOD_MPUIO:
		reg += OMAP_MPUIO_IO_CNTL;
		break;
	case METHOD_GPIO_1510:
		reg += OMAP1510_GPIO_DIR_CONTROL;
		break;
	case METHOD_GPIO_1610:
		reg += OMAP1610_GPIO_DIRECTION;
		break;
	case METHOD_GPIO_730:
		reg += OMAP730_GPIO_DIR_CONTROL;
		break;
	}
	l = __raw_readl((void __iomem *)reg);
	if (is_input)
		l |= 1 << gpio;
	else
		l &= ~(1 << gpio);
	__raw_writel(l, (void __iomem *)reg);
}

void omap_set_gpio_direction(int gpio, int is_input)
{
	struct gpio_bank *bank;

	if (check_gpio(gpio) < 0)
		return;
	bank = get_gpio_bank(gpio);
	spin_lock(&bank->lock);
	_set_gpio_direction(bank, get_gpio_index(gpio), is_input);
	spin_unlock(&bank->lock);
}

static void _set_gpio_dataout(struct gpio_bank *bank, int gpio, int enable)
{
	u32 reg = bank->base;
	u32 l = 0;

	switch (bank->method) {
	case METHOD_MPUIO:
		reg += OMAP_MPUIO_OUTPUT;
		l = __raw_readl((void __iomem *)reg);
		if (enable)
			l |= 1 << gpio;
		else
			l &= ~(1 << gpio);
		break;
	case METHOD_GPIO_1510:
		reg += OMAP1510_GPIO_DATA_OUTPUT;
		l = __raw_readl((void __iomem *)reg);
		if (enable)
			l |= 1 << gpio;
		else
			l &= ~(1 << gpio);
		break;
	case METHOD_GPIO_1610:
		if (enable)
			reg += OMAP1610_GPIO_SET_DATAOUT;
		else
			reg += OMAP1610_GPIO_CLEAR_DATAOUT;
		l = 1 << gpio;
		break;
	case METHOD_GPIO_730:
		reg += OMAP730_GPIO_DATA_OUTPUT;
		l = __raw_readl((void __iomem *)reg);
		if (enable)
			l |= 1 << gpio;
		else
			l &= ~(1 << gpio);
		break;
	default:
		BUG();
		return;
	}
	__raw_writel(l, (void __iomem *)reg);
}

void omap_set_gpio_dataout(int gpio, int enable)
{
	struct gpio_bank *bank;

	if (check_gpio(gpio) < 0)
		return;
	bank = get_gpio_bank(gpio);
	spin_lock(&bank->lock);
	_set_gpio_dataout(bank, get_gpio_index(gpio), enable);
	spin_unlock(&bank->lock);
}

int omap_get_gpio_datain(int gpio)
{
	struct gpio_bank *bank;
	u32 reg;

	if (check_gpio(gpio) < 0)
		return -1;
	bank = get_gpio_bank(gpio);
	reg = bank->base;
	switch (bank->method) {
	case METHOD_MPUIO:
		reg += OMAP_MPUIO_INPUT_LATCH;
		break;
	case METHOD_GPIO_1510:
		reg += OMAP1510_GPIO_DATA_INPUT;
		break;
	case METHOD_GPIO_1610:
		reg += OMAP1610_GPIO_DATAIN;
		break;
	case METHOD_GPIO_730:
		reg += OMAP730_GPIO_DATA_INPUT;
		break;
	default:
		BUG();
		return -1;
	}
	return (__raw_readl((void __iomem *)reg)
			& (1 << get_gpio_index(gpio))) != 0;
}

static int _set_gpio_edge_ctrl(struct gpio_bank *bank, int gpio, int edge)
{
	u32 reg = bank->base;
	u32 l;

	switch (bank->method) {
	case METHOD_MPUIO:
		reg += OMAP_MPUIO_GPIO_INT_EDGE;
		l = __raw_readl((void __iomem *)reg);
		if (edge == IRQT_RISING)
			l |= 1 << gpio;
		else if (edge == IRQT_FALLING)
			l &= ~(1 << gpio);
		else
			goto bad;
		break;
	case METHOD_GPIO_1510:
		reg += OMAP1510_GPIO_INT_CONTROL;
		l = __raw_readl((void __iomem *)reg);
		if (edge == IRQT_RISING)
			l |= 1 << gpio;
		else if (edge == IRQT_FALLING)
			l &= ~(1 << gpio);
		else
			goto bad;
		break;
	case METHOD_GPIO_1610:
		/* NOTE: knows __IRQT_{FAL,RIS}EDGE match OMAP hardware */
		edge &= 0x03;
		if (gpio & 0x08)
			reg += OMAP1610_GPIO_EDGE_CTRL2;
		else
			reg += OMAP1610_GPIO_EDGE_CTRL1;
		gpio &= 0x07;
		l = __raw_readl((void __iomem *)reg);
		l &= ~(3 << (gpio << 1));
		l |= edge << (gpio << 1);
		break;
	case METHOD_GPIO_730:
		reg += OMAP730_GPIO_INT_CONTROL;
		l = __raw_readl((void __iomem *)reg);
		if (edge == IRQT_RISING)
			l |= 1 << gpio;
		else if (edge == IRQT_FALLING)
			l &= ~(1 << gpio);
		else
			goto bad;
		break;
	default:
		BUG();
		goto bad;
	}
	__raw_writel(l, (void __iomem *)reg);
	return 0;
bad:
	return -EINVAL;
}

static int gpio_irq_type(unsigned irq, unsigned type)
{
	struct gpio_bank *bank;
	unsigned gpio;
	int retval;

	if (irq > IH_MPUIO_BASE)
		gpio = OMAP_MPUIO(irq - IH_MPUIO_BASE);
	else
		gpio = irq - IH_GPIO_BASE;

	if (check_gpio(gpio) < 0)
		return -EINVAL;

	if (type & (__IRQT_LOWLVL|__IRQT_HIGHLVL|IRQT_PROBE))
		return -EINVAL;

	bank = get_gpio_bank(gpio);
	spin_lock(&bank->lock);
	retval = _set_gpio_edge_ctrl(bank, get_gpio_index(gpio), type);
	spin_unlock(&bank->lock);
	return retval;
}


static int _get_gpio_edge_ctrl(struct gpio_bank *bank, int gpio)
{
	u32 reg = bank->base, l;

	switch (bank->method) {
	case METHOD_MPUIO:
		l = __raw_readl(reg + OMAP_MPUIO_GPIO_INT_EDGE);
		return (l & (1 << gpio)) ?
			IRQT_RISING : IRQT_FALLING;
	case METHOD_GPIO_1510:
		l = __raw_readl(reg + OMAP1510_GPIO_INT_CONTROL);
		return (l & (1 << gpio)) ?
			IRQT_RISING : IRQT_FALLING;
	case METHOD_GPIO_1610:
		if (gpio & 0x08)
			reg += OMAP1610_GPIO_EDGE_CTRL2;
		else
			reg += OMAP1610_GPIO_EDGE_CTRL1;
		/* NOTE: knows __IRQT_{FAL,RIS}EDGE match OMAP hardware */
		return (__raw_readl(reg) >> ((gpio & 0x07) << 1)) & 0x03;
	case METHOD_GPIO_730:
		l = __raw_readl(reg + OMAP730_GPIO_INT_CONTROL);
		return (l & (1 << gpio)) ?
			IRQT_RISING : IRQT_FALLING;
	default:
		BUG();
		return -1;
	}
}

static void _clear_gpio_irqbank(struct gpio_bank *bank, int gpio_mask)
{
	u32 reg = bank->base;

	switch (bank->method) {
	case METHOD_MPUIO:
		/* MPUIO irqstatus is reset by reading the status register,
		 * so do nothing here */
		return;
	case METHOD_GPIO_1510:
		reg += OMAP1510_GPIO_INT_STATUS;
		break;
	case METHOD_GPIO_1610:
		reg += OMAP1610_GPIO_IRQSTATUS1;
		break;
	case METHOD_GPIO_730:
		reg += OMAP730_GPIO_INT_STATUS;
		break;
	default:
		BUG();
		return;
	}
	__raw_writel(gpio_mask, (void __iomem *)reg);
}

static inline void _clear_gpio_irqstatus(struct gpio_bank *bank, int gpio)
{
	_clear_gpio_irqbank(bank, 1 << get_gpio_index(gpio));
}

static void _enable_gpio_irqbank(struct gpio_bank *bank, int gpio_mask, int enable)
{
	u32 reg = bank->base;
	u32 l;

	switch (bank->method) {
	case METHOD_MPUIO:
		reg += OMAP_MPUIO_GPIO_MASKIT;
		l = __raw_readl((void __iomem *)reg);
		if (enable)
			l &= ~(gpio_mask);
		else
			l |= gpio_mask;
		break;
	case METHOD_GPIO_1510:
		reg += OMAP1510_GPIO_INT_MASK;
		l = __raw_readl((void __iomem *)reg);
		if (enable)
			l &= ~(gpio_mask);
		else
			l |= gpio_mask;
		break;
	case METHOD_GPIO_1610:
		if (enable)
			reg += OMAP1610_GPIO_SET_IRQENABLE1;
		else
			reg += OMAP1610_GPIO_CLEAR_IRQENABLE1;
		l = gpio_mask;
		break;
	case METHOD_GPIO_730:
		reg += OMAP730_GPIO_INT_MASK;
		l = __raw_readl((void __iomem *)reg);
		if (enable)
			l &= ~(gpio_mask);
		else
			l |= gpio_mask;
		break;
	default:
		BUG();
		return;
	}
	__raw_writel(l, (void __iomem *)reg);
}

static inline void _set_gpio_irqenable(struct gpio_bank *bank, int gpio, int enable)
{
	_enable_gpio_irqbank(bank, 1 << get_gpio_index(gpio), enable);
}

/*
 * Note that ENAWAKEUP needs to be enabled in GPIO_SYSCONFIG register.
 * 1510 does not seem to have a wake-up register. If JTAG is connected
 * to the target, system will wake up always on GPIO events. While
 * system is running all registered GPIO interrupts need to have wake-up
 * enabled. When system is suspended, only selected GPIO interrupts need
 * to have wake-up enabled.
 */
static int _set_gpio_wakeup(struct gpio_bank *bank, int gpio, int enable)
{
	switch (bank->method) {
	case METHOD_GPIO_1610:
		spin_lock(&bank->lock);
		if (enable)
			bank->suspend_wakeup |= (1 << gpio);
		else
			bank->suspend_wakeup &= ~(1 << gpio);
		spin_unlock(&bank->lock);
		return 0;
	default:
		printk(KERN_ERR "Can't enable GPIO wakeup for method %i\n",
		       bank->method);
		return -EINVAL;
	}
}

/* Use disable_irq_wake() and enable_irq_wake() functions from drivers */
static int gpio_wake_enable(unsigned int irq, unsigned int enable)
{
	unsigned int gpio = irq - IH_GPIO_BASE;
	struct gpio_bank *bank;
	int retval;

	if (check_gpio(gpio) < 0)
		return -ENODEV;
	bank = get_gpio_bank(gpio);
	spin_lock(&bank->lock);
	retval = _set_gpio_wakeup(bank, get_gpio_index(gpio), enable);
	spin_unlock(&bank->lock);

	return retval;
}

int omap_request_gpio(int gpio)
{
	struct gpio_bank *bank;

	if (check_gpio(gpio) < 0)
		return -EINVAL;

	bank = get_gpio_bank(gpio);
	spin_lock(&bank->lock);
	if (unlikely(bank->reserved_map & (1 << get_gpio_index(gpio)))) {
		printk(KERN_ERR "omap-gpio: GPIO %d is already reserved!\n", gpio);
		dump_stack();
		spin_unlock(&bank->lock);
		return -1;
	}
	bank->reserved_map |= (1 << get_gpio_index(gpio));
#ifdef CONFIG_ARCH_OMAP1510
	if (bank->method == METHOD_GPIO_1510) {
		u32 reg;

		/* Claim the pin for the ARM */
		reg = bank->base + OMAP1510_GPIO_PIN_CONTROL;
		__raw_writel(__raw_readl(reg) | (1 << get_gpio_index(gpio)), reg);
	}
#endif
#ifdef CONFIG_ARCH_OMAP16XX
	if (bank->method == METHOD_GPIO_1610) {
		/* Enable wake-up during idle for dynamic tick */
		u32 reg = bank->base + OMAP1610_GPIO_SET_WAKEUPENA;
		__raw_writel(1 << get_gpio_index(gpio), (void __iomem *)reg);
	}
#endif
	spin_unlock(&bank->lock);

	return 0;
}

void omap_free_gpio(int gpio)
{
	struct gpio_bank *bank;

	if (check_gpio(gpio) < 0)
		return;
	bank = get_gpio_bank(gpio);
	spin_lock(&bank->lock);
	if (unlikely(!(bank->reserved_map & (1 << get_gpio_index(gpio))))) {
		printk(KERN_ERR "omap-gpio: GPIO %d wasn't reserved!\n", gpio);
		dump_stack();
		spin_unlock(&bank->lock);
		return;
	}
#ifdef CONFIG_ARCH_OMAP16XX
	if (bank->method == METHOD_GPIO_1610) {
		/* Disable wake-up during idle for dynamic tick */
		u32 reg = bank->base + OMAP1610_GPIO_CLEAR_WAKEUPENA;
		__raw_writel(1 << get_gpio_index(gpio), (void __iomem *)reg);
	}
#endif
	bank->reserved_map &= ~(1 << get_gpio_index(gpio));
	_set_gpio_direction(bank, get_gpio_index(gpio), 1);
	_set_gpio_irqenable(bank, gpio, 0);
	_clear_gpio_irqstatus(bank, gpio);
	spin_unlock(&bank->lock);
}

/*
 * We need to unmask the GPIO bank interrupt as soon as possible to
 * avoid missing GPIO interrupts for other lines in the bank.
 * Then we need to mask-read-clear-unmask the triggered GPIO lines
 * in the bank to avoid missing nested interrupts for a GPIO line.
 * If we wait to unmask individual GPIO lines in the bank after the
 * line's interrupt handler has been run, we may miss some nested
 * interrupts.
 */
static void gpio_irq_handler(unsigned int irq, struct irqdesc *desc,
			     struct pt_regs *regs)
{
	u32 isr_reg = 0;
	u32 isr;
	unsigned int gpio_irq;
	struct gpio_bank *bank;

	desc->chip->ack(irq);

	bank = (struct gpio_bank *) desc->data;
	if (bank->method == METHOD_MPUIO)
		isr_reg = bank->base + OMAP_MPUIO_GPIO_INT;
#ifdef CONFIG_ARCH_OMAP1510
	if (bank->method == METHOD_GPIO_1510)
		isr_reg = bank->base + OMAP1510_GPIO_INT_STATUS;
#endif
#if defined(CONFIG_ARCH_OMAP16XX)
	if (bank->method == METHOD_GPIO_1610)
		isr_reg = bank->base + OMAP1610_GPIO_IRQSTATUS1;
#endif
#ifdef CONFIG_ARCH_OMAP730
	if (bank->method == METHOD_GPIO_730)
		isr_reg = bank->base + OMAP730_GPIO_INT_STATUS;
#endif

	isr = __raw_readl((void __iomem *)isr_reg);
	_enable_gpio_irqbank(bank, isr, 0);
	_clear_gpio_irqbank(bank, isr);
	_enable_gpio_irqbank(bank, isr, 1);
	desc->chip->unmask(irq);

	if (unlikely(!isr))
		return;

	gpio_irq = bank->virtual_irq_start;
	for (; isr != 0; isr >>= 1, gpio_irq++) {
		struct irqdesc *d;
		if (!(isr & 1))
			continue;
		d = irq_desc + gpio_irq;
		d->handle(gpio_irq, d, regs);
	}
}

static void gpio_ack_irq(unsigned int irq)
{
	unsigned int gpio = irq - IH_GPIO_BASE;
	struct gpio_bank *bank = get_gpio_bank(gpio);

	_clear_gpio_irqstatus(bank, gpio);
}

static void gpio_mask_irq(unsigned int irq)
{
	unsigned int gpio = irq - IH_GPIO_BASE;
	struct gpio_bank *bank = get_gpio_bank(gpio);

	_set_gpio_irqenable(bank, gpio, 0);
}

static void gpio_unmask_irq(unsigned int irq)
{
	unsigned int gpio = irq - IH_GPIO_BASE;
	struct gpio_bank *bank = get_gpio_bank(gpio);

	if (_get_gpio_edge_ctrl(bank, get_gpio_index(gpio)) == IRQT_NOEDGE) {
		printk(KERN_ERR "OMAP GPIO %d: trying to enable GPIO IRQ while no edge is set\n",
		       gpio);
		_set_gpio_edge_ctrl(bank, get_gpio_index(gpio), IRQT_RISING);
	}
	_set_gpio_irqenable(bank, gpio, 1);
}

static void mpuio_ack_irq(unsigned int irq)
{
	/* The ISR is reset automatically, so do nothing here. */
}

static void mpuio_mask_irq(unsigned int irq)
{
	unsigned int gpio = OMAP_MPUIO(irq - IH_MPUIO_BASE);
	struct gpio_bank *bank = get_gpio_bank(gpio);

	_set_gpio_irqenable(bank, gpio, 0);
}

static void mpuio_unmask_irq(unsigned int irq)
{
	unsigned int gpio = OMAP_MPUIO(irq - IH_MPUIO_BASE);
	struct gpio_bank *bank = get_gpio_bank(gpio);

	_set_gpio_irqenable(bank, gpio, 1);
}

static struct irqchip gpio_irq_chip = {
	.ack	= gpio_ack_irq,
	.mask	= gpio_mask_irq,
	.unmask = gpio_unmask_irq,
	.type   = gpio_irq_type,
	.wake	= gpio_wake_enable,
};

static struct irqchip mpuio_irq_chip = {
	.ack	= mpuio_ack_irq,
	.mask	= mpuio_mask_irq,
	.unmask = mpuio_unmask_irq
};

static int initialized = 0;
static struct clk * gpio_ck = 0;

static int __init _omap_gpio_init(void)
{
	int i;
	struct gpio_bank *bank;

	initialized = 1;

	if (cpu_is_omap1510()) {
		gpio_ck = clk_get(0, "arm_gpio_ck");
		if (IS_ERR(gpio_ck))
			printk("Could not get arm_gpio_ck\n");
		else
			clk_use(gpio_ck);
	}

#ifdef CONFIG_ARCH_OMAP1510
	if (cpu_is_omap1510()) {
		printk(KERN_INFO "OMAP1510 GPIO hardware\n");
		gpio_bank_count = 2;
		gpio_bank = gpio_bank_1510;
	}
#endif
#if defined(CONFIG_ARCH_OMAP16XX)
	if (cpu_is_omap16xx()) {
		int rev;

		gpio_bank_count = 5;
		gpio_bank = gpio_bank_1610;
		rev = omap_readw(gpio_bank[1].base + OMAP1610_GPIO_REVISION);
		printk(KERN_INFO "OMAP GPIO hardware version %d.%d\n",
		       (rev >> 4) & 0x0f, rev & 0x0f);
	}
#endif
#ifdef CONFIG_ARCH_OMAP730
	if (cpu_is_omap730()) {
		printk(KERN_INFO "OMAP730 GPIO hardware\n");
		gpio_bank_count = 7;
		gpio_bank = gpio_bank_730;
	}
#endif
	for (i = 0; i < gpio_bank_count; i++) {
		int j, gpio_count = 16;

		bank = &gpio_bank[i];
		bank->reserved_map = 0;
		bank->base = IO_ADDRESS(bank->base);
		spin_lock_init(&bank->lock);
		if (bank->method == METHOD_MPUIO) {
			omap_writew(0xFFFF, OMAP_MPUIO_BASE + OMAP_MPUIO_GPIO_MASKIT);
		}
#ifdef CONFIG_ARCH_OMAP1510
		if (bank->method == METHOD_GPIO_1510) {
			__raw_writew(0xffff, bank->base + OMAP1510_GPIO_INT_MASK);
			__raw_writew(0x0000, bank->base + OMAP1510_GPIO_INT_STATUS);
		}
#endif
#if defined(CONFIG_ARCH_OMAP16XX)
		if (bank->method == METHOD_GPIO_1610) {
			__raw_writew(0x0000, bank->base + OMAP1610_GPIO_IRQENABLE1);
			__raw_writew(0xffff, bank->base + OMAP1610_GPIO_IRQSTATUS1);
			__raw_writew(0x0014, bank->base + OMAP1610_GPIO_SYSCONFIG);
		}
#endif
#ifdef CONFIG_ARCH_OMAP730
		if (bank->method == METHOD_GPIO_730) {
			__raw_writel(0xffffffff, bank->base + OMAP730_GPIO_INT_MASK);
			__raw_writel(0x00000000, bank->base + OMAP730_GPIO_INT_STATUS);

			gpio_count = 32; /* 730 has 32-bit GPIOs */
		}
#endif
		for (j = bank->virtual_irq_start;
		     j < bank->virtual_irq_start + gpio_count; j++) {
			if (bank->method == METHOD_MPUIO)
				set_irq_chip(j, &mpuio_irq_chip);
			else
				set_irq_chip(j, &gpio_irq_chip);
			set_irq_handler(j, do_simple_IRQ);
			set_irq_flags(j, IRQF_VALID);
		}
		set_irq_chained_handler(bank->irq, gpio_irq_handler);
		set_irq_data(bank->irq, bank);
	}

	/* Enable system clock for GPIO module.
	 * The CAM_CLK_CTRL *is* really the right place. */
	if (cpu_is_omap1610() || cpu_is_omap1710())
		omap_writel(omap_readl(ULPD_CAM_CLK_CTRL) | 0x04, ULPD_CAM_CLK_CTRL);

	return 0;
}

#ifdef CONFIG_ARCH_OMAP16XX
static int omap_gpio_suspend(struct sys_device *dev, pm_message_t mesg)
{
	int i;

	if (cpu_is_omap1510())
		return 0;

	for (i = 0; i < gpio_bank_count; i++) {
		struct gpio_bank *bank = &gpio_bank[i];
		u32 wake_status = bank->base + OMAP1610_GPIO_WAKEUPENABLE;
		u32 wake_clear = bank->base + OMAP1610_GPIO_CLEAR_WAKEUPENA;
		u32 wake_set = bank->base + OMAP1610_GPIO_SET_WAKEUPENA;

		if (bank->method != METHOD_GPIO_1610)
			continue;

		spin_lock(&bank->lock);
		bank->saved_wakeup = __raw_readl(wake_status);
		__raw_writel(0xffffffff, wake_clear);
		__raw_writel(bank->suspend_wakeup, wake_set);
		spin_unlock(&bank->lock);
	}

	return 0;
}

static int omap_gpio_resume(struct sys_device *dev)
{
	int i;

	if (cpu_is_omap1510())
		return 0;

	for (i = 0; i < gpio_bank_count; i++) {
		struct gpio_bank *bank = &gpio_bank[i];
		u32 wake_clear = bank->base + OMAP1610_GPIO_CLEAR_WAKEUPENA;
		u32 wake_set = bank->base + OMAP1610_GPIO_SET_WAKEUPENA;

		if (bank->method != METHOD_GPIO_1610)
			continue;

		spin_lock(&bank->lock);
		__raw_writel(0xffffffff, wake_clear);
		__raw_writel(bank->saved_wakeup, wake_set);
		spin_unlock(&bank->lock);
	}

	return 0;
}

static struct sysdev_class omap_gpio_sysclass = {
	set_kset_name("gpio"),
	.suspend	= omap_gpio_suspend,
	.resume		= omap_gpio_resume,
};

static struct sys_device omap_gpio_device = {
	.id		= 0,
	.cls		= &omap_gpio_sysclass,
};
#endif

/*
 * This may get called early from board specific init
 */
int omap_gpio_init(void)
{
	if (!initialized)
		return _omap_gpio_init();
	else
		return 0;
}

static int __init omap_gpio_sysinit(void)
{
	int ret = 0;

	if (!initialized)
		ret = _omap_gpio_init();

#ifdef CONFIG_ARCH_OMAP16XX
	if (cpu_is_omap16xx()) {
		if (ret == 0) {
			ret = sysdev_class_register(&omap_gpio_sysclass);
			if (ret == 0)
				ret = sysdev_register(&omap_gpio_device);
		}
	}
#endif

	return ret;
}

EXPORT_SYMBOL(omap_request_gpio);
EXPORT_SYMBOL(omap_free_gpio);
EXPORT_SYMBOL(omap_set_gpio_direction);
EXPORT_SYMBOL(omap_set_gpio_dataout);
EXPORT_SYMBOL(omap_get_gpio_datain);

arch_initcall(omap_gpio_sysinit);
