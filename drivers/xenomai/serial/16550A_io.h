/*
 * Copyright (C) 2007 Jan Kiszka <jan.kiszka@web.de>.
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/* Manages the I/O access method of the driver. */

typedef enum { MODE_PIO, MODE_MMIO } io_mode_t;

#if defined(CONFIG_XENO_DRIVERS_16550A_PIO) || \
    defined(CONFIG_XENO_DRIVERS_16550A_ANY)
static unsigned long io[MAX_DEVICES];
module_param_array(io, ulong, NULL, 0400);
MODULE_PARM_DESC(io, "I/O port addresses of the serial devices");
#endif /* CONFIG_XENO_DRIVERS_16550A_PIO || CONFIG_XENO_DRIVERS_16550A_ANY */

#if defined(CONFIG_XENO_DRIVERS_16550A_MMIO) || \
    defined(CONFIG_XENO_DRIVERS_16550A_ANY)
static unsigned long mem[MAX_DEVICES];
static void *mapped_io[MAX_DEVICES];
module_param_array(mem, ulong, NULL, 0400);
MODULE_PARM_DESC(mem, "I/O memory addresses of the serial devices");
#endif /* CONFIG_XENO_DRIVERS_16550A_MMIO || CONFIG_XENO_DRIVERS_16550A_ANY */

#ifdef CONFIG_XENO_DRIVERS_16550A_PIO

#define RT_16550_IO_INLINE inline

extern void *mapped_io[]; /* dummy */

static inline unsigned long rt_16550_addr_param(int dev_id)
{
	return io[dev_id];
}

static inline int rt_16550_addr_param_valid(int dev_id)
{
	return 1;
}

static inline unsigned long rt_16550_base_addr(int dev_id)
{
	return io[dev_id];
}

static inline io_mode_t rt_16550_io_mode(int dev_id)
{
	return MODE_PIO;
}

static inline io_mode_t
rt_16550_io_mode_from_ctx(struct rt_16550_context *ctx)
{
	return MODE_PIO;
}

static inline void
rt_16550_init_io_ctx(int dev_id, struct rt_16550_context *ctx)
{
	ctx->base_addr = io[dev_id];
}

#elif defined(CONFIG_XENO_DRIVERS_16550A_MMIO)

#define RT_16550_IO_INLINE inline

extern unsigned long io[]; /* dummy */

static inline unsigned long rt_16550_addr_param(int dev_id)
{
	return mem[dev_id];
}

static inline int rt_16550_addr_param_valid(int dev_id)
{
	return 1;
}

static inline unsigned long rt_16550_base_addr(int dev_id)
{
	return (unsigned long)mapped_io[dev_id];
}

static inline io_mode_t rt_16550_io_mode(int dev_id)
{
	return MODE_MMIO;
}

static inline io_mode_t
rt_16550_io_mode_from_ctx(struct rt_16550_context *ctx)
{
	return MODE_MMIO;
}

static inline void
rt_16550_init_io_ctx(int dev_id, struct rt_16550_context *ctx)
{
	ctx->base_addr = (unsigned long)mapped_io[dev_id];
}

#elif defined(CONFIG_XENO_DRIVERS_16550A_ANY)

#define RT_16550_IO_INLINE /* uninline */

static inline unsigned long rt_16550_addr_param(int dev_id)
{
	return (io[dev_id]) ? io[dev_id] : mem[dev_id];
}

static inline int rt_16550_addr_param_valid(int dev_id)
{
	return !(io[dev_id] && mem[dev_id]);
}

static inline unsigned long rt_16550_base_addr(int dev_id)
{
	return (io[dev_id]) ? io[dev_id] : (unsigned long)mapped_io[dev_id];
}

static inline io_mode_t rt_16550_io_mode(int dev_id)
{
	return (io[dev_id]) ? MODE_PIO : MODE_MMIO;
}

static inline io_mode_t
rt_16550_io_mode_from_ctx(struct rt_16550_context *ctx)
{
	return ctx->io_mode;
}

static inline void
rt_16550_init_io_ctx(int dev_id, struct rt_16550_context *ctx)
{
	if (io[dev_id]) {
		ctx->base_addr = io[dev_id];
		ctx->io_mode   = MODE_PIO;
	} else {
		ctx->base_addr = (unsigned long)mapped_io[dev_id];
		ctx->io_mode   = MODE_MMIO;
	}
}

#else
# error Unsupported I/O access method
#endif

static RT_16550_IO_INLINE u8
rt_16550_reg_in(io_mode_t io_mode, unsigned long base, int off)
{
	switch (io_mode) {
	case MODE_PIO:
		return inb(base + off);
	default: /* MODE_MMIO */
		return readb((void *)base + off);
	}
}

static RT_16550_IO_INLINE void
rt_16550_reg_out(io_mode_t io_mode, unsigned long base, int off, u8 val)
{
	switch (io_mode) {
	case MODE_PIO:
		outb(val, base + off);
		break;
	case MODE_MMIO:
		writeb(val, (void *)base + off);
		break;
	}
}

static int rt_16550_init_io(int dev_id, char* name)
{
	switch (rt_16550_io_mode(dev_id)) {
	case MODE_PIO:
		if (!request_region(rt_16550_addr_param(dev_id), 8, name))
			return -EBUSY;
		break;
	case MODE_MMIO:
		mapped_io[dev_id] = ioremap(rt_16550_addr_param(dev_id), 8);
		if (!mapped_io[dev_id])
			return -EBUSY;
		break;
	}
	return 0;
}

static void rt_16550_release_io(int dev_id)
{
	switch (rt_16550_io_mode(dev_id)) {
	case MODE_PIO:
		release_region(io[dev_id], 8);
		break;
	case MODE_MMIO:
		iounmap(mapped_io[dev_id]);
		break;
	}
}
