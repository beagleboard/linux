#include <linux/fb.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/serial_core.h>
#include <linux/ioport.h>
#include <linux/wait.h>

/* DC8000 hw reg*/
#define FRAMEBUFFER_CONFIG                                          0x1518
#define FRAMEBUFFER_ADDRESS                                         0x1400
#define FRAMEBUFFER_STRIDE                                          0x1408
#define HDISPLAY                                                    0x1430
#define HSYNC                                                       0x1438
#define VDISPLAY                                                    0x1440
#define VSYNC                                                       0x1448
#define PANEL_CONFIG                                                0x1418
#define DPI_CONFIG                                                  0x14B8
#define CURSOR_ADDRESS                                              0x146C
#define CURSOR_CONFIG                                               0x1468
#define CURSOR_LOCATION                                             0x1470
#define CURSOR_BACKGROUND                                           0x1474
#define CURSOR_FOREGROUND                                           0x1478
#define FRAMEBUFFER_UPLANAR_ADDRESS                                 0x1530
#define FRAMEBUFFER_VPLANAR_ADDRESS                                 0x1538
#define FRAMEBUFFER_USTRIDE                                         0x1800
#define FRAMEBUFFER_VSTRIDE                                         0x1808
#define INDEXCOLOR_TABLEINDEX                                       0x1818
#define INDEXCOLOR_TABLEDATA                                        0x1820
#define FRAMEBUFFER_SIZE                                            0x1810
#define FRAMEBUFFER_SCALEFACTORX                                    0x1828
#define FRAMEBUFFER_SCALEFACTORY                                    0x1830
#define FRAMEBUFFER_SCALEFCONFIG                                    0x1520
#define HORIFILTER_KERNELINDEX                                      0x1838
#define HORIFILTER_KERNEL                                           0x1A00
#define VERTIFILTER_KERNELINDEX                                     0x1A08
#define VERTIFILTER_KERNEL                                          0x1A10
#define FRAMEBUFFER_INITIALOFFSET                                   0x1A20
#define FRAMEBUFFER_COLORKEY                                        0x1508
#define FRAMEBUFFER_COLORHIGHKEY                                    0x1510
#define FRAMEBUFFER_BGCOLOR                                         0x1528
#define FRAMEBUFFER_CLEARVALUE                                      0x1A18
#define DISPLAY_INTRENABLE                                          0x1480
#define INT_STATE                                                   0x147C
#define PANEL_DEST_ADDRESS                                          0x14F0
#define MEM_DEST_ADDRESS                                            0x14E8
#define DEST_CONFIG                                                 0x14F8
#define DEST_STRIDE                                                 0x1500
#define DBI_CONFIG                                                  0x1488
#define AQHICLOCKCONTROL                                            0x0000
#define OVERLAY_CONFIG                                              0x1540
#define OVERLAY_STRIDE                                              0x1600
#define OVERLAY_USTRIDE                                             0x18C0
#define OVERLAY_VSTRIDE                                             0x1900
#define OVERLAY_TL                                                  0x1640
#define OVERLAY_BR                                                  0x1680
#define OVERLAY_ALPHA_BLEND_CONFIG                                  0x1580
#define OVERLAY_SRC_GLOBAL_COLOR                                    0x16C0
#define OVERLAY_DST_GLOBAL_COLOR                                    0x1700
#define OVERLAY_CLEAR_VALUE                                         0x1940
#define OVERLAY_SIZE                                                0x17C0
#define OVERLAY_COLOR_KEY                                           0x1740
#define OVERLAY_COLOR_KEY_HIGH                                      0x1780
#define OVERLAY_ADDRESS                                             0x15C0
#define OVERLAY_UPLANAR_ADDRESS                                     0x1840
#define OVERLAY_VPLANAR_ADDRESS                                     0x1880
#define OVERLAY_SCALE_CONFIG                                        0x1C00
#define OVERLAY_SCALE_FACTOR_X                                      0x1A40
#define OVERLAY_SCALE_FACTOR_Y                                      0x1A80
#define OVERLAY_HORI_FILTER_KERNEL_INDEX                            0x1AC0
#define OVERLAY_HORI_FILTER_KERNEL                                  0x1B00
#define OVERLAY_VERTI_FILTER_KERNEL_INDEX                           0x1B40
#define OVERLAY_VERTI_FILTER_KERNEL                                 0x1B80
#define OVERLAY_INITIAL_OFFSET                                      0x1BC0
#define GAMMA_EX_INDEX                                              0x1CF0
#define GAMMA_EX_DATA                                               0x1CF8
#define GAMMA_EX_ONE_DATA                                           0x1D80
#define GAMMA_INDEX                                                 0x1458
#define GAMMA_DATA                                                  0x1460
#define DISPLAY_DITHER_TABLE_LOW                                    0x1420
#define DISPLAY_DITHER_TABLE_HIGH                                   0x1428
#define DISPLAY_DITHER_CONFIG                                       0x1410

/* ultrafb.h */
#define ULTRAFBIO_CURSOR 0x46A0
#define ULTRAFBIO_BLENDING_MODE 0x46A4
#define ULTRAFBIO_GLOBAL_MODE_VALUE 0x46A5
#define ULTRAFBIO_BUFFER_SIZE 0x46A7
#define ULTRAFBIO_OVERLAY_RECT 0x46A8
#define ULTRAFBIO_SCALE_FILTER_TAP 0x46A9
#define ULTRAFBIO_SYNC_TABLE 0x46AA
#define ULTRAFBIO_GAMMA 0x46BD
#define ULTRAFBIO_DITHER 0x46BE
#define ULTRAFBIO_ROTATION 0x46BF
#define ULTRAFBIO_TILEMODE 0x46C1
#define ULTRAFBIO_COLORKEY 0x46C2

#define CURSOR_SIZE 32
#define GAMMA_INDEX_MAX 256

typedef struct _dc_frame_info {
	unsigned int width;
	unsigned int height;
	unsigned int stride;
}
dc_frame_info;

typedef struct _dc_overlay_rect {
	unsigned int tlx;
	unsigned int tly;
	unsigned int brx;
	unsigned int bry;
}
dc_overlay_rect;

typedef enum _dc_alpha_blending_mode {
	DC_BLEND_MODE_CLEAR = 0x0,
	DC_BLEND_MODE_SRC,
	DC_BLEND_MODE_DST,
	DC_BLEND_MODE_SRC_OVER,
	DC_BLEND_MODE_DST_OVER,
	DC_BLEND_MODE_SRC_IN,
	DC_BLEND_MODE_DST_IN,
	DC_BLEND_MODE_SRC_OUT,
}
dc_alpha_blending_mode;

typedef enum _dc_rot_angle {
	DC_ROT_ANGLE_ROT0 = 0x0,
	DC_ROT_ANGLE_FLIP_X,
	DC_ROT_ANGLE_FLIP_Y,
	DC_ROT_ANGLE_FLIP_XY,
	DC_ROT_ANGLE_ROT90,
	DC_ROT_ANGLE_ROT180,
	DC_ROT_ANGLE_ROT270,
}
dc_rot_angle;

typedef enum _dc_tile_mode {
	DC_TILE_MODE_LINEAR = 0x0,
	DC_TILE_MODE_TILED4X4,
	DC_TILE_MODE_SUPER_TILED_XMAJOR,
	DC_TILE_MODE_SUPER_TILED_YMAJOR,
	DC_TILE_MODE_TILE_MODE8X8,
	DC_TILE_MODE_TILE_MODE8X4,
	DC_TILE_MODE_SUPER_TILED_XMAJOR8X4,
	DC_TILE_MODE_SUPER_TILED_YMAJOR4X8,
	DC_TILE_MODE_TILE_Y,
}
dc_tile_mode;

typedef struct _dc_global_alpha {
	unsigned int global_alpha_mode;
	unsigned int global_alpha_value;
}
dc_global_alpha;

typedef struct _dc_filter_tap {
	unsigned int vertical_filter_tap;
	unsigned int horizontal_filter_tap;
}
dc_filter_tap;

typedef struct _dc_sync_table {
	unsigned int horkernel[128];
	unsigned int verkernel[128];
}
dc_sync_table;

typedef struct _dc_gamma_table {
	bool gamma_enable;
	unsigned int gamma[GAMMA_INDEX_MAX][3];
}
dc_gamma_table;

typedef struct _dc_color_key {
	unsigned char enable;
	unsigned int colorkey_low;
	unsigned int colorkey_high;
	/* background color only available for video, not available for overlay*/
	unsigned int bg_color;
}
dc_color_key;

#define DEBUG 0

// #define DEFAULT_DC_ROT_ANGLE	DC_ROT_ANGLE_ROT0;
#define DEFAULT_DC_ROT_ANGLE	DC_ROT_ANGLE_ROT180;	// for ICE EVB C910 board HW

#define DEFAULT_WIDTH           720
#define DEFAULT_HEIGHT          1280
#define DEFAULT_FB_SIZE         (DEFAULT_WIDTH * DEFAULT_HEIGHT * 4 * 3)
#define DEFAULT_OVERLAY_SIZE    (DEFAULT_WIDTH * DEFAULT_HEIGHT * 4 * 3)

#define PIX_FMT_NONE   0
#define PIX_FMT_RGB444 1
#define PIX_FMT_RGB555 3
#define PIX_FMT_RGB565 4
#define PIX_FMT_RGB888 6

#define CURSOR_FMT_DISALBE   0
#define CURSOR_FMT_MASKED    1
#define CURSOR_FMT_A8R8G8B8    2
#define CURSOR_SIZE 32

/*module param*/
static uint irq = 0;
module_param(irq, uint, 0644);
static ulong reg_base = 0;
module_param(reg_base, ulong, 0644);
static uint reg_size = 64 << 10;
module_param(reg_size, uint, 0644);


static void __iomem *reg_virtual;
static dma_addr_t framebuffer_phys = 0;
static dma_addr_t overlay_phys = 0;
static wait_queue_head_t vsync_wait;
static struct fb_info *overlay_info = NULL;
static u32 dst_global_alpha_mode = 0;
static u32 dst_global_alpha_value = 0;
static void *cursor_data = NULL;

#if DEBUG
static char __iomem *dest_virtual;
static dma_addr_t dest_phys = 0;
#endif

/* ToDo */
typedef struct _dc_last_overlay_scale_info {
	unsigned int lastverfiltertap;
	unsigned int lasthorfiltertap;
	unsigned int lastheight;
	unsigned int lastwidth;
	unsigned int lastrectx;
	unsigned int lastrecty;
}
dc_last_overlay_scale_info;

struct ultrafb_info {
	struct device  *dev;
	struct clk     *clk;
	struct fb_info *info;
	void __iomem *reg_virtual;

	dma_addr_t fb_start_dma;
	dc_tile_mode tile_mode;

	int format;
	u32 pseudo_palette[16];

	/* cursor info. */
	u32 cursor_format;
	int cursor_x;
	int cursor_y;
	int cursor_hot_x;
	int cursor_hot_y;
	dma_addr_t cursor_start_dma;
	void *cursor_virtual;

	/* frame buffer info. */
	dma_addr_t fb_current_dma;
	dc_frame_info framebuffersize;

	/* overlay info. */
	dma_addr_t overlay_current_dma;
	dc_overlay_rect overlayrect;
	dc_frame_info overlaysize;

	/* alpha blending info. */
	dc_global_alpha blending;
	dc_alpha_blending_mode mode;

	/* scale info. */
	dc_filter_tap filtertap;
	dc_sync_table synctable;

	/* gamma info. */
	dc_gamma_table gamma_table;

	/* dither info. */
	bool dither_enable;

	/* rotation info. */
	dc_rot_angle rotangle;

	/* colorkey info. */
	dc_color_key colorkey;

	u32 vblank_count;
};

struct ultrafb_mach_info {
	struct fb_videomode *modes;
	int num_modes;
	unsigned int pix_fmt;
};

struct fb_videomode video_modes[] = {
	[0] = {
		.refresh = 60,
		.xres    = 1920,
		.yres    = 1080,
		.hsync_len = 32,
		.left_margin = 120,
		.right_margin = 128,
		.vsync_len = 14,
		.upper_margin = 21,
		.lower_margin = 10,
	},
	[1] = {
		.refresh = 60,
		.xres    = 640,
		.yres    = 480,
		.hsync_len = 96,
		.left_margin = 48,
		.right_margin = 16,
		.vsync_len = 2,
		.upper_margin = 33,
		.lower_margin = 10,
	},
	[2] = {
		.refresh = 30,
		.xres    = 1280,
		.yres    = 720,
		.hsync_len = 43,
		.left_margin = 1917,
		.right_margin = 60,
		.vsync_len = 3,
		.upper_margin = 17,
		.lower_margin = 10,
	},
	[3] = {
		.refresh = 60,
		.xres    = 720,
		.yres    = 1280,
		.hsync_len = 10,
		.left_margin = 45,
		.right_margin = 40,
		.vsync_len = 4,
		.upper_margin = 10,
		.lower_margin = 11,
	},
	[4] = {
		.refresh = 50,
		.xres    = 1280,
		.yres    = 720,
		.hsync_len = 43,
		.left_margin = 597,
		.right_margin = 60,
		.vsync_len = 3,
		.upper_margin = 17,
		.lower_margin = 10,
	},
	[5] = {
		.refresh = 60,
		.xres    = 800,
		.yres    = 480,
		.hsync_len = 3,
		.left_margin = 43,
		.right_margin = 82,
		.vsync_len = 3,
		.upper_margin = 20,
		.lower_margin = 22,
	},
};

struct ultrafb_mach_info default_mach_info = {
	// .modes     = &video_modes[5],	/* for RGB 7" LCD */
	.modes     = &video_modes[3],		/* for MIPI LCD */
	.num_modes = ARRAY_SIZE(video_modes),
	.pix_fmt   = PIX_FMT_RGB888,
};

struct pix_fmt_info {
	int pix_fmt;
	int8_t bits_per_pixel;
	int8_t transp_offset;
	int8_t transp_length;
	int8_t red_offset;
	int8_t red_length;
	int8_t green_offset;
	int8_t green_length;
	int8_t blue_offset;
	int8_t blue_length;
};

static struct pix_fmt_info pix_fmt_xlate[] = {
	{PIX_FMT_RGB444, 16,  12, 4,  8,  4,  4, 4,  0, 4},
	{PIX_FMT_RGB444, 16,   0, 0,  8,  4,  4, 4,  0, 4},
	{PIX_FMT_RGB565, 16,   0, 0,  11, 5,  5, 6,  0, 5},
	{PIX_FMT_RGB555, 16,  15, 1,  10, 5,  5, 5,  0, 5},
	{PIX_FMT_RGB555, 16,   0, 0,  10, 5,  5, 5,  0, 5},
	{PIX_FMT_RGB888, 32,  24, 8,  16, 8,  8, 8,  0, 8},
	{PIX_FMT_RGB888, 32,   0, 0,  16, 8,  8, 8,  0, 8},
};

/* Now, dma_alloc_writecombine is not supported in riscv systm yet,
 * using kmalloc instead.
 */
static inline void *thead_dma_alloc(struct device *dev, size_t size,
				    dma_addr_t *dma_addr, gfp_t gfp)
{
#ifdef __riscv
	void *vaddr = kmalloc(size, GFP_KERNEL);
	if (vaddr == NULL) {
		printk("thead_dma_alloc(size=%zu) failed\n", size);
		return NULL;
	}

	*dma_addr = virt_to_phys(vaddr);
	return vaddr;
#else
	return dma_alloc_writecombine(dev, size, dma_addr, gfp);
#endif
}

static inline void teahd_dma_free(struct device *dev, size_t size,
				  void *cpu_addr, dma_addr_t dma_addr)
{
#ifdef __riscv
	kfree(cpu_addr);
#else
	dma_free_writecombine(dev, size, cpu_addr, dma_addr);
#endif
}

void write_register(uint32_t data, uint32_t addr)
{
	writel(data, reg_virtual + addr);
#if DEBUG
	printk("Write [0x%08x] 0x%08x\n", addr, data);
#endif
}

uint32_t read_register(uint32_t addr)
{
	uint32_t data = 0;
	data = readl(reg_virtual + addr);
#if DEBUG
	if (addr != INT_STATE)
	{
		printk("Read [0x%08x] 0x%08x\n", addr, data);
	}
#endif

	return data;
}

static int determine_best_pix_fmt(struct fb_var_screeninfo *var)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(pix_fmt_xlate); i++) {
		struct pix_fmt_info *info = &pix_fmt_xlate[i];

		if ((var->bits_per_pixel == info->bits_per_pixel) &&
			(var->transp.offset == info->transp_offset) &&
			(var->transp.length == info->transp_length) &&
			(var->red.offset == info->red_offset) &&
			(var->red.length == info->red_length) &&
			(var->green.offset == info->green_offset) &&
			(var->green.length == info->green_length) &&
			(var->blue.offset == info->blue_offset) &&
			(var->blue.length == info->blue_length)) {
			/* found format. */
			return info->pix_fmt;
		}
	}

	return 0;
}

static void set_pix_fmt(struct fb_var_screeninfo *var, int pix_fmt)
{
	size_t i;
	struct pix_fmt_info *info = NULL;

	for (i = 0; i < ARRAY_SIZE(pix_fmt_xlate); i++) {
		if (pix_fmt_xlate[i].pix_fmt == pix_fmt) {
			info = &pix_fmt_xlate[i];
			break;
		}
	}

	var->bits_per_pixel = info->bits_per_pixel;
	var->transp.offset = info->transp_offset;
	var->transp.length = info->transp_length;
	var->red.offset = info->red_offset;
	var->red.length = info->red_length;
	var->green.offset = info->green_offset;
	var->green.length = info->green_length;
	var->blue.offset = info->blue_offset;
	var->blue.length = info->blue_length;
}

static int get_bpp_by_format(int pix_fmt)
{
	size_t i;
	struct pix_fmt_info *info = NULL;
	int bpp = 0;

	for (i = 0; i < ARRAY_SIZE(pix_fmt_xlate); i++) {
		if (pix_fmt_xlate[i].pix_fmt == pix_fmt) {
			info = &pix_fmt_xlate[i];
			break;
		}
	}

	bpp = info ->bits_per_pixel / 8;
	return bpp;
}

static void set_mode(struct fb_var_screeninfo *var, struct fb_videomode *mode, int pix_fmt)
{
	set_pix_fmt(var, pix_fmt);
	fb_videomode_to_var(var, mode);
	var->grayscale = 0;
}

static void set_graphics_start(struct fb_info *info, int xoffset, int yoffset)
{
	struct ultrafb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;
	int pixel_offset;

	pixel_offset = (yoffset * var->xres_virtual) + xoffset;

	fbi->fb_current_dma = info->fix.smem_start + (pixel_offset * (var->bits_per_pixel >> 3));
}

static void set_overlay_graphics_start(struct fb_info *info, int xoffset, int yoffset)
{
	struct ultrafb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;
	int pixel_offset;

	pixel_offset = (yoffset * var->xres_virtual) + xoffset;

	fbi->overlay_current_dma = info->fix.smem_start + (pixel_offset * (var->bits_per_pixel >> 3));
}

static int ultrafb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct ultrafb_info *fbi = info->par;
	struct device *dev = fbi->dev;
	int pix_fmt;

	pix_fmt = determine_best_pix_fmt(var);
	if (!pix_fmt) {
		dev_err(dev, "unsupport pixel format\n");
		return -EINVAL;
	}
	fbi->format = pix_fmt;

	/*
	 * Basic geometry sanity checks.
	 */
	if (var->xres_virtual > var->xres) {
		dev_err(dev, "unsupport stride\n");
		return -EINVAL;
	}

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	/*
	 * Check size of framebuffer.
	 */
	if (var->xres_virtual * var->yres_virtual * (var->bits_per_pixel >> 3)
		> info->fix.smem_len) {
		dev_err(dev, "invalid framebuffer size requirement\n");
		return -EINVAL;
	}

	set_graphics_start(info, var->xoffset, var->yoffset);
	return 0;
}

static int ultrafb_set_par(struct fb_info *info)
{
	/* config dc register */
	uint32_t h_end, h_total, hsync_start, hsync_end;
	uint32_t v_end, v_total, vsync_start, vsync_end;
	struct fb_var_screeninfo *var = &info->var;
	struct ultrafb_info *fbi = info->par;
	uint32_t data;
	int j = 0;

	info->fix.line_length = var->xres_virtual * var->bits_per_pixel / 8;
	info->fix.ypanstep = var->yres;

	/* Get the timing from var info */
	h_end = var->xres;
	h_total = var->xres + var->left_margin + var->right_margin + var->hsync_len;
	hsync_start = var->xres + var->right_margin;
	hsync_end = hsync_start + var->hsync_len;

	v_end = var->yres;
	v_total = var->yres + var->upper_margin + var->lower_margin + var->vsync_len;
	vsync_start = var->yres + var->lower_margin;
	vsync_end = vsync_start + var->vsync_len;

	data = (h_total << 16) | h_end;
	write_register(data, HDISPLAY);

	data = (v_total << 16) | v_end;
	write_register(data, VDISPLAY);

	data = (0 << 31) | (1 << 30) | (hsync_end << 15) | (hsync_start);
	write_register(data, HSYNC);

	data = (0 << 31) | (1 << 30) | (vsync_end << 15) | (vsync_start);
	write_register(data, VSYNC);

	write_register(0x00000080, DBI_CONFIG);

	write_register(0x00000005, DPI_CONFIG);

	data = (1 << 0) | (0 << 1) | (1 << 4) | (0 << 5) | (1 << 8) | (0 << 9);
	write_register(data, PANEL_CONFIG);

	data = 0x0;
	write_register(data, FRAMEBUFFER_UPLANAR_ADDRESS);

	data = 0x0;
	write_register(data, FRAMEBUFFER_VPLANAR_ADDRESS);

	data = 0x0;
	write_register(data, FRAMEBUFFER_USTRIDE);

	data = 0x0;
	write_register(data, FRAMEBUFFER_VSTRIDE);

	write_register(0, INDEXCOLOR_TABLEINDEX);

	data = (fbi->framebuffersize.height << 15) | (fbi->framebuffersize.width);
	write_register(data, FRAMEBUFFER_SIZE);

	if ((var->xres != fbi->framebuffersize.width) || (var->yres != fbi->framebuffersize.height))
	{
		data = ((fbi->framebuffersize.width - 1) << 16) / (var->xres - 1);
		write_register(data , FRAMEBUFFER_SCALEFACTORX);

		data = ((fbi->framebuffersize.height - 1) << 16) / (var->yres - 1);
		if (data > (3 << 16))
		{
			data = (3 << 16);
		}
		write_register(data , FRAMEBUFFER_SCALEFACTORY);

		write_register(0, HORIFILTER_KERNELINDEX);

		for (j = 0; j < 128; j++)
		{
			data = fbi->synctable.horkernel[j];
			write_register(data , HORIFILTER_KERNEL);
		}

		write_register(0, VERTIFILTER_KERNELINDEX);

		for (j = 0; j < 128; j++)
		{
			data = fbi->synctable.verkernel[j];
			write_register(data , VERTIFILTER_KERNEL);
		}

		data = (0x8000 << 0) | (0x8000 << 16);
		write_register(data , FRAMEBUFFER_INITIALOFFSET);

		data = (fbi->filtertap.vertical_filter_tap << 0) | (fbi->filtertap.horizontal_filter_tap << 4);
		write_register(data , FRAMEBUFFER_SCALEFCONFIG);
	}

	data = fbi->colorkey.colorkey_low;
	write_register(data, FRAMEBUFFER_COLORKEY);

	data = fbi->colorkey.colorkey_high;
	write_register(data, FRAMEBUFFER_COLORHIGHKEY);

	data = fbi->colorkey.bg_color;
	write_register(data, FRAMEBUFFER_BGCOLOR);

	data = 0x0;
	write_register(data, FRAMEBUFFER_CLEARVALUE);

	data = 1 << 0;
	write_register(data, DISPLAY_INTRENABLE);

#if DEBUG
	/* Set dest related function */
	write_register(dest_phys, PANEL_DEST_ADDRESS);

	write_register(dest_phys, MEM_DEST_ADDRESS);

	data = (1 << 16) | (1 << 17);
	write_register(data, DEST_CONFIG);

	write_register(fbi->framebuffersize.stride, DEST_STRIDE);
#endif
	write_register(fbi->framebuffersize.stride, FRAMEBUFFER_STRIDE);
	return 0;
}

static int ultrafb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	uint32_t data;
	struct ultrafb_info *fbi = info->par;
	int pixel_byte = 0;

	pixel_byte = get_bpp_by_format(fbi->format);
	set_graphics_start(info, var->xoffset, var->yoffset);
	write_register(fbi->fb_current_dma, FRAMEBUFFER_ADDRESS);

	if (((fbi->rotangle == DC_ROT_ANGLE_ROT90) || (fbi->rotangle == DC_ROT_ANGLE_ROT270)) && (fbi->tile_mode == DC_TILE_MODE_LINEAR))
	{
		return -EINVAL;
	}
	else
	{
		if ((var->xres != fbi->framebuffersize.width) || (var->yres != fbi->framebuffersize.height))
		{
			data = (1 << 0) | (fbi->gamma_table.gamma_enable << 2) |(1 << 4) | (0 << 8) | (fbi->colorkey.enable << 9) |
				(fbi->rotangle << 11) | (fbi->tile_mode << 17) | (1 << 22) | (fbi->format << 26);
		}
		else
		{
			data = (1 << 0) | (fbi->gamma_table.gamma_enable << 2) |(1 << 4) | (0 << 8) | (fbi->colorkey.enable << 9) |
				(fbi->rotangle << 11) | (fbi->tile_mode << 17) | (0 << 22) | (fbi->format << 26);
		}
		write_register(data, FRAMEBUFFER_CONFIG);
	}

	var->yoffset = var->yoffset + var->yres;
	if (var->yoffset > info->var.yres_virtual - var->yres)
	{
		var->yoffset = 0;
	}

	fbi->gamma_table.gamma_enable = 0;
	fbi->colorkey.enable = 0;
	fbi->rotangle = DEFAULT_DC_ROT_ANGLE;
	fbi->framebuffersize.width = DEFAULT_WIDTH;
	fbi->framebuffersize.height = DEFAULT_HEIGHT;
	fbi->framebuffersize.stride = DEFAULT_WIDTH * pixel_byte;
	return 0;
}

static int ultrafb_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	uint32_t x, y, format;
	uint32_t data;
	int set = cursor->set;
	struct ultrafb_info *fbi = info->par;

	if (cursor->image.width > CURSOR_SIZE || cursor->image.height > CURSOR_SIZE)
		return -EINVAL;

	write_register(fbi->cursor_start_dma, CURSOR_ADDRESS);

	if (set & FB_CUR_SETPOS)
	{
		y = cursor->image.dy;
		x = cursor->image.dx;

		data = (y << 16) | x;
		write_register(data, CURSOR_LOCATION);
	}

	/*
	 * FB_CUR_SETIMAGE - the cursor image has changed
	 * FB_CUR_SETCMAP  - the cursor colors has changed
	 * FB_CUR_SETSHAPE - the cursor bitmask has changed
	 */
	if (cursor->set & (FB_CUR_SETSHAPE | FB_CUR_SETCMAP | FB_CUR_SETIMAGE)) {
		uint32_t image_size;
		uint32_t bg, fg;

		if (info->state != FBINFO_STATE_RUNNING)
			return 0;

		bg = cursor->image.bg_color;
		fg = cursor->image.fg_color;

		write_register(bg, CURSOR_BACKGROUND);
		write_register(fg, CURSOR_FOREGROUND);

		/* Use 32-bit operations on the data to improve performance */
		image_size = cursor->image.width * cursor->image.height;
		memcpy(fbi->cursor_virtual, (uint32_t *)cursor->image.data, image_size * 4);
	}

	if (cursor->enable)
	{
		/* Default ARGB8888 format */
		format = CURSOR_FMT_A8R8G8B8;
		data = format | (1 << 2) | (0 << 4) | (0 << 8) | (0 << 16);
		write_register(data, CURSOR_CONFIG);
	}
	else
	{
		data = (0 << 0) | (1 << 2) | (0 << 4) | (0 << 8) | (0 << 16);
		write_register(data, CURSOR_CONFIG);
	}

	return 0;
}

static int ultrafb_gamma(struct ultrafb_info *fbi)
{
	int i = 0;
	u32 data;

	for (i = 0; i < GAMMA_INDEX_MAX; i++)
	{
		data = i;
		write_register(data, GAMMA_INDEX);

		data = (fbi->gamma_table.gamma[i][2] << 0) | (fbi->gamma_table.gamma[i][1] << 10) |
				(fbi->gamma_table.gamma[i][0] << 20);
		write_register(data, GAMMA_DATA);
	}
	return 0;
}

static int ultrafb_dither(struct ultrafb_info *fbi)
{
	u32 data;

	if (fbi->dither_enable)
	{
		write_register(0x7B48F3C0, DISPLAY_DITHER_TABLE_LOW);
		write_register(0x596AD1E2, DISPLAY_DITHER_TABLE_HIGH);

		data = fbi->dither_enable << 31;
		write_register(data, DISPLAY_DITHER_CONFIG);
	}
	else
	{
		write_register(0, DISPLAY_DITHER_TABLE_LOW);
		write_register(0, DISPLAY_DITHER_TABLE_HIGH);
		write_register(0, DISPLAY_DITHER_CONFIG);
	}
	return 0;
}

static int ultrafb_blank(int blank, struct fb_info *info)
{
	u32 data, _new;

	data = read_register(FRAMEBUFFER_CONFIG);
	if (blank)
		_new = data & ~(1 << 0);
	else
		_new = data | (1 << 0);
	if (_new == data)
		return 0;

	write_register(_new, FRAMEBUFFER_CONFIG);

	return 0;
}

static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	return ((chan & 0xffff) >> (16 - bf->length)) << bf->offset;
}

static int ultrafb_setcolreg(unsigned int regno, unsigned int red, unsigned int green,
		 unsigned int blue, unsigned int trans, struct fb_info *info)
{
	struct ultrafb_info *fbi = info->par;
	u32 val;

	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
					7471 * blue) >> 16;

	if (info->fix.visual == FB_VISUAL_TRUECOLOR && regno < 16) {
		val =  chan_to_field(red,   &info->var.red);
		val |= chan_to_field(green, &info->var.green);
		val |= chan_to_field(blue , &info->var.blue);
		fbi->pseudo_palette[regno] = val;
	}

	return 0;
}

static int ultrafb_mmap(struct fb_info *info, struct vm_area_struct * vma)
{
	unsigned long mmio_pgoff;
	unsigned long start;
	u32 len;

	start = info->fix.smem_start;
	len = info->fix.smem_len;
	mmio_pgoff = PAGE_ALIGN((start & ~PAGE_MASK) + len) >> PAGE_SHIFT;
	if (vma->vm_pgoff >= mmio_pgoff) {
		if (info->var.accel_flags) {
			mutex_unlock(&info->mm_lock);
			return -EINVAL;
		}

		vma->vm_pgoff -= mmio_pgoff;
		start = info->fix.mmio_start;
		len = info->fix.mmio_len;
	}

	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	return vm_iomap_memory(vma, start, len);
}

static int ultrafb_ioctl(struct fb_info *info, unsigned int cmd,
		unsigned long argp)
{
	int ret, j;
	u32 vblank_count;
	dc_frame_info size;
	dc_filter_tap filtertap;
	dc_sync_table table;
	struct fb_cursor cursor;
	static dc_gamma_table gammatable;
	bool ditherenable;
	dc_rot_angle rotangle;
	dc_tile_mode tilemode;
	dc_color_key colorkey;

	struct ultrafb_info *fbi = info->par;

	switch (cmd) {
	case ULTRAFBIO_CURSOR:
		if (copy_from_user(&cursor, (void *)argp, sizeof(cursor)))
			return -EFAULT;

		cursor_data = kmalloc(CURSOR_SIZE * CURSOR_SIZE * 4, GFP_KERNEL);
		if (copy_from_user(cursor_data, (void *)cursor.image.data, CURSOR_SIZE * CURSOR_SIZE * 4))
			return -EFAULT;
		cursor.image.data = (char *)cursor_data;

		ret = ultrafb_cursor(info, &cursor);

		/* free buffer */
		kfree(cursor_data);
		return ret;
	case FBIO_WAITFORVSYNC:
		vblank_count = fbi->vblank_count;
		ret = wait_event_interruptible_timeout(vsync_wait, vblank_count != fbi->vblank_count, HZ /10);

		if(ret == 0)
		   return -ETIMEDOUT;
		return 0;
	case ULTRAFBIO_BUFFER_SIZE:
		if (copy_from_user(&size, (void *)argp, sizeof(size)))
			return -EFAULT;

		fbi->framebuffersize.width = size.width;
		fbi->framebuffersize.height = size.height;
		fbi->framebuffersize.stride = size.stride;
		return 0;
	case ULTRAFBIO_SCALE_FILTER_TAP:
		if (copy_from_user(&filtertap, (void *)argp, sizeof(filtertap)))
			return -EFAULT;

		fbi->filtertap.horizontal_filter_tap = filtertap.horizontal_filter_tap;
		fbi->filtertap.vertical_filter_tap = filtertap.vertical_filter_tap;
		return 0;
	case ULTRAFBIO_SYNC_TABLE:
		if (copy_from_user(&table, (void *)argp, sizeof(table)))
			return -EFAULT;

		for (j = 0; j < 128; j++)
		{
			fbi->synctable.horkernel[j] = table.horkernel[j];
			fbi->synctable.verkernel[j] = table.verkernel[j];
		}
		return 0;
	case ULTRAFBIO_GAMMA:
		if (copy_from_user(&gammatable, (void *)argp, sizeof(gammatable)))
			return -EFAULT;

		fbi->gamma_table.gamma_enable = gammatable.gamma_enable;
		for (j = 0; j < GAMMA_INDEX_MAX; j++)
		{
			fbi->gamma_table.gamma[j][0] = gammatable.gamma[j][0];
			fbi->gamma_table.gamma[j][1] = gammatable.gamma[j][1];
			fbi->gamma_table.gamma[j][2] = gammatable.gamma[j][2];
		}

		ret = ultrafb_gamma(fbi);
		return ret;
	case ULTRAFBIO_DITHER:
		if (copy_from_user(&ditherenable, (void *)argp, sizeof(ditherenable)))
			return -EFAULT;

		fbi->dither_enable = ditherenable;

		ret = ultrafb_dither(fbi);
		return ret;
	case ULTRAFBIO_ROTATION:
		if (copy_from_user(&rotangle, (void *)argp, sizeof(rotangle)))
			return -EFAULT;

		fbi->rotangle = rotangle;
		return 0;
	case ULTRAFBIO_TILEMODE:
		if (copy_from_user(&tilemode, (void *)argp, sizeof(tilemode)))
			return -EFAULT;

		fbi->tile_mode = tilemode;
		return 0;
	case ULTRAFBIO_COLORKEY:
		if (copy_from_user(&colorkey, (void *)argp, sizeof(colorkey)))
			return -EFAULT;

		if (colorkey.enable)
			fbi->colorkey.enable = 0x2;
		else
			fbi->colorkey.enable = 0x0;

		fbi->colorkey.colorkey_low = colorkey.colorkey_low;
		fbi->colorkey.colorkey_high = colorkey.colorkey_high;
		fbi->colorkey.bg_color = colorkey.bg_color;
		return 0;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ultrafb_release(struct fb_info *info, int user)
{
	uint32_t data;

	data = 0x0;
	write_register(data, FRAMEBUFFER_CONFIG);
	return 0;
}

static struct fb_ops ultrafb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = ultrafb_check_var,
	.fb_set_par = ultrafb_set_par,
	.fb_pan_display = ultrafb_pan_display,
	.fb_blank = ultrafb_blank,
	.fb_setcolreg = ultrafb_setcolreg,
	.fb_mmap = ultrafb_mmap,
	.fb_ioctl = ultrafb_ioctl,
	.fb_release = ultrafb_release,
};

static irqreturn_t ultrafb_handle_irq(int irq, void *dev_id)
{
	struct ultrafb_info *fbi = dev_id;
	uint32_t data;

	data = read_register(INT_STATE);
	fbi->vblank_count ++;
	if (!(data & 0x1))
		return IRQ_NONE;

	/* TODO: set fb cursor here. */

	return IRQ_HANDLED;
}

static void set_overlay_var(struct fb_var_screeninfo *var, struct fb_videomode *mode, int pix_fmt)
{
	set_pix_fmt(var, pix_fmt);

	var->xres = mode ->xres;
	var->yres = mode ->yres;
	var->xres_virtual = mode ->xres;
	var->yres_virtual = mode ->yres;
	var->xoffset = 0;
	var->yoffset = 0;
	var->height = -1;
	var->width = -1;
	var->vmode = FB_VMODE_NONINTERLACED;
	var->activate = FB_ACTIVATE_FORCE;
}

static int overlay_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct ultrafb_info *fbi = info->par;
	struct device *dev = fbi->dev;
	int pix_fmt;

	pix_fmt = determine_best_pix_fmt(var);
	if (!pix_fmt) {
		dev_err(dev, "unsupport pixel format\n");
		return -EINVAL;
	}
	fbi->format = pix_fmt;

	/*
	 * Basic geometry sanity checks.
	 */
	if (var->xres_virtual > var->xres) {
		dev_err(dev, "unsupport stride\n");
		return -EINVAL;
	}

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	/*
	 * Check size of framebuffer.
	 */
	if (var->xres_virtual * var->yres_virtual * (var->bits_per_pixel >> 3)
		> info->fix.smem_len) {
		dev_err(dev, "invalid framebuffer size requirement\n");
		return -EINVAL;
	}

	set_overlay_graphics_start(info, var->xoffset, var->yoffset);
	return 0;
}

static int overlay_set_par(struct fb_info *info)
{
	/* config overlay register */
	struct ultrafb_info *fbi = info->par;
	uint32_t data;
	uint32_t rectx, recty;
	int j = 0;

	rectx = fbi->overlayrect.brx - fbi->overlayrect.tlx;
	recty = fbi->overlayrect.bry - fbi->overlayrect.tly;

	data = fbi->overlaysize.stride;
	write_register(data, OVERLAY_STRIDE);

	data = 0;
	write_register(data, OVERLAY_USTRIDE);

	data = 0;
	write_register(data, OVERLAY_VSTRIDE);

	data = (fbi->overlayrect.tlx << 0) | (fbi->overlayrect.tly << 15);
	write_register(data, OVERLAY_TL);

	data = (fbi->overlayrect.brx << 0) | (fbi->overlayrect.bry << 15);
	write_register(data, OVERLAY_BR);

	switch(fbi->mode)
	{
	case DC_BLEND_MODE_CLEAR:
		data = (0 << 0) | (fbi->blending.global_alpha_mode << 3) | (0 << 5) | (0 << 8) |
				  (0 << 9) | (dst_global_alpha_mode << 10) | (0 << 12) | (0 << 15);
		break;
	case DC_BLEND_MODE_SRC:
		data = (0 << 0) | (fbi->blending.global_alpha_mode << 3) | (1 << 5) | (0 << 8) |
				  (0 << 9) | (dst_global_alpha_mode << 10) | (0 << 12) | (0 << 15);
		break;
	case DC_BLEND_MODE_DST:
		data = (0 << 0) | (fbi->blending.global_alpha_mode << 3) | (0 << 5) | (0 << 8) |
				  (0 << 9) | (dst_global_alpha_mode << 10) | (1 << 12) | (0 << 15);
		break;
	case DC_BLEND_MODE_SRC_OVER:
		data = (0 << 0) | (fbi->blending.global_alpha_mode << 3) | (1 << 5) | (0 << 8) |
				  (0 << 9) | (dst_global_alpha_mode << 10) | (3 << 12) | (0 << 15);
		break;
	case DC_BLEND_MODE_DST_OVER:
		data = (0 << 0) | (fbi->blending.global_alpha_mode << 3) | (3 << 5) | (0 << 8) |
				  (0 << 9) | (dst_global_alpha_mode << 10) | (1 << 12) | (0 << 15);
		break;
	case DC_BLEND_MODE_SRC_IN:
		data = (0 << 0) | (fbi->blending.global_alpha_mode << 3) | (2 << 5) | (0 << 8) |
				  (0 << 9) | (dst_global_alpha_mode << 10) | (0 << 12) | (0 << 15);
		break;
	case DC_BLEND_MODE_DST_IN:
		data = (0 << 0) | (fbi->blending.global_alpha_mode << 3) | (0 << 5) | (0 << 8) |
				  (0 << 9) | (dst_global_alpha_mode << 10) | (2 << 12) | (0 << 15);
		break;
	case DC_BLEND_MODE_SRC_OUT:
		data = (0 << 0) | (fbi->blending.global_alpha_mode << 3) | (3 << 5) | (0 << 8) |
				  (0 << 9) | (dst_global_alpha_mode << 10) | (0 << 12) | (0 << 15);
		break;
	}
	write_register(data, OVERLAY_ALPHA_BLEND_CONFIG);

	data = fbi->blending.global_alpha_value;
	write_register(data , OVERLAY_SRC_GLOBAL_COLOR);

	data = dst_global_alpha_value;
	write_register(data , OVERLAY_DST_GLOBAL_COLOR);

	write_register(0, OVERLAY_CLEAR_VALUE);

	data = (fbi->overlaysize.height << 15) | (fbi->overlaysize.width);
	write_register(data, OVERLAY_SIZE);

	data = fbi->colorkey.colorkey_low;
	write_register(data, OVERLAY_COLOR_KEY);

	data = fbi->colorkey.colorkey_high;
	write_register(data, OVERLAY_COLOR_KEY_HIGH);

	write_register(0, OVERLAY_UPLANAR_ADDRESS);

	write_register(0, OVERLAY_VPLANAR_ADDRESS);

	if ((rectx != fbi->overlaysize.width) || (recty != fbi->overlaysize.height))
	{
		data = ((fbi->overlaysize.width - 1) << 16) / (rectx - 1);
		write_register(data , OVERLAY_SCALE_FACTOR_X);

		data = ((fbi->overlaysize.height - 1) << 16) / (recty - 1);
		if (data > (3 << 16))
		{
			data = (3 << 16);
		}
		write_register(data , OVERLAY_SCALE_FACTOR_Y);

		write_register(0, OVERLAY_HORI_FILTER_KERNEL_INDEX);

		for (j = 0; j < 128; j++)
		{
			data = fbi->synctable.horkernel[j];
			write_register(data , OVERLAY_HORI_FILTER_KERNEL);
		}

		write_register(0, OVERLAY_VERTI_FILTER_KERNEL_INDEX);

		for (j = 0; j < 128; j++)
		{
			data = fbi->synctable.verkernel[j];
			write_register(data , OVERLAY_VERTI_FILTER_KERNEL);
		}

		data = (0x8000 << 0) | (0x8000 << 16);
		write_register(data , OVERLAY_INITIAL_OFFSET);

		data = (fbi->filtertap.vertical_filter_tap << 0) | (fbi->filtertap.horizontal_filter_tap << 4) | (1 << 8);
		write_register(data , OVERLAY_SCALE_CONFIG);
	}
	else
	{
		write_register(0, OVERLAY_SCALE_CONFIG);
	}

	return 0;
}

static int overlay_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	uint32_t data;
	struct ultrafb_info *fbi = info->par;

	set_overlay_graphics_start(info, var->xoffset, var->yoffset);
	write_register(fbi->overlay_current_dma, OVERLAY_ADDRESS);

	if (((fbi->rotangle == DC_ROT_ANGLE_ROT90) || (fbi->rotangle == DC_ROT_ANGLE_ROT270)) && (fbi->tile_mode != DC_TILE_MODE_LINEAR))
	{
		return -EINVAL;
	}
	else
	{
		data = (fbi->colorkey.enable << 0) | (6 << 16) | (1 << 24) | (fbi->rotangle << 2) | (0 << 13) |
			(0 << 0) | (0 << 25) | (fbi->tile_mode << 8);
		write_register(data, OVERLAY_CONFIG);
	}

	var->yoffset = var->yoffset + var->yres;
	if (var->yoffset > info->var.yres_virtual - var->yres)
	{
		var->yoffset = 0;
	}

	fbi->colorkey.enable = 0;
	fbi->rotangle = DEFAULT_DC_ROT_ANGLE;
	fbi->mode = DC_BLEND_MODE_SRC;
	fbi->overlayrect.tlx = 0;
	fbi->overlayrect.tly = 0;
	fbi->overlayrect.brx = 320;
	fbi->overlayrect.bry = 240;
	fbi->overlaysize.width = 320;
	fbi->overlaysize.height = 240;
	return 0;
}

static int overlay_ioctl(struct fb_info *info, unsigned int cmd, unsigned long argp)
{
	int ret, j;
	dc_global_alpha alpha;
	dc_frame_info size;
	dc_overlay_rect coordinate;
	struct ultrafb_info *fbi = info->par;
	dc_alpha_blending_mode mode;
	dc_filter_tap filtertap;
	dc_sync_table table;
	dc_rot_angle rotangle;
	dc_tile_mode tilemode;
	dc_color_key colorkey;

	switch (cmd) {
	case ULTRAFBIO_BUFFER_SIZE:
		if (copy_from_user(&size, (void *)argp, sizeof(size)))
			return -EFAULT;

		fbi->overlaysize.width = size.width;
		fbi->overlaysize.height = size.height;
		fbi->overlaysize.stride = size.stride;
		return 0;
	case ULTRAFBIO_OVERLAY_RECT:
		if (copy_from_user(&coordinate, (void *)argp, sizeof(coordinate)))
			return -EFAULT;

		fbi->overlayrect.tlx = coordinate.tlx;
		fbi->overlayrect.tly = coordinate.tly;
		fbi->overlayrect.brx = coordinate.brx;
		fbi->overlayrect.bry = coordinate.bry;
		return 0;
	case ULTRAFBIO_BLENDING_MODE:
		if (copy_from_user(&mode, (void *)argp, sizeof(mode)))
			return -EFAULT;

		fbi->mode = mode;
		return 0;
	case ULTRAFBIO_GLOBAL_MODE_VALUE:
		if (copy_from_user(&alpha, (void *)argp, sizeof(alpha)))
			return -EFAULT;

		fbi->blending.global_alpha_mode = alpha.global_alpha_mode;
		fbi->blending.global_alpha_value = alpha.global_alpha_value;
		return 0;
	case ULTRAFBIO_SCALE_FILTER_TAP:
		if (copy_from_user(&filtertap, (void *)argp, sizeof(filtertap)))
			return -EFAULT;

		fbi->filtertap.horizontal_filter_tap = filtertap.horizontal_filter_tap;
		fbi->filtertap.vertical_filter_tap = filtertap.vertical_filter_tap;
		return 0;
	case ULTRAFBIO_SYNC_TABLE:
		if (copy_from_user(&table, (void *)argp, sizeof(table)))
			return -EFAULT;

		for (j = 0; j < 128; j++)
		{
			fbi->synctable.horkernel[j] = table.horkernel[j];
			fbi->synctable.verkernel[j] = table.verkernel[j];
		}
		return 0;
	case ULTRAFBIO_ROTATION:
		if (copy_from_user(&rotangle, (void *)argp, sizeof(rotangle)))
			return -EFAULT;

		fbi->rotangle = rotangle;
		return 0;
	case ULTRAFBIO_TILEMODE:
		if (copy_from_user(&tilemode, (void *)argp, sizeof(tilemode)))
			return -EFAULT;

		fbi->tile_mode = tilemode;
		return 0;
	case ULTRAFBIO_COLORKEY:
		if (copy_from_user(&colorkey, (void *)argp, sizeof(colorkey)))
			return -EFAULT;

		if (colorkey.enable)
			fbi->colorkey.enable = 0x2;
		else
			fbi->colorkey.enable = 0x0;

		fbi->colorkey.colorkey_low = colorkey.colorkey_low;
		fbi->colorkey.colorkey_high = colorkey.colorkey_high;
		return 0;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int overlay_release(struct fb_info *info, int user)
{
	uint32_t data;

	data = 0x0;
	write_register(data, OVERLAY_CONFIG);
	return 0;
}

static struct fb_ops overlay_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = overlay_check_var,
	.fb_set_par = overlay_set_par,
	.fb_pan_display = overlay_pan_display,
	.fb_ioctl = overlay_ioctl,
	.fb_release = overlay_release,
};

static int dcultra_overlay_init(struct platform_device *pdev)
{
	struct fb_info *info;
	struct ultrafb_info *fbi;
	struct ultrafb_mach_info *mi;
	int ret = 0;

	mi = &default_mach_info;

	info = framebuffer_alloc(sizeof(struct ultrafb_info), &pdev->dev);
	if (info == NULL) {
		dev_err(&pdev->dev, "alloc overlay fb_info failed\n");
		return -ENOMEM;
	}

	fbi = info->par;
	fbi->info = info;
	fbi->dev = info->dev = &pdev->dev;
	fbi->format = mi->pix_fmt;
	fbi->mode = DC_BLEND_MODE_SRC;
	fbi->blending.global_alpha_mode = 0;
	fbi->blending.global_alpha_value = 0;
	fbi->filtertap.horizontal_filter_tap = 5;
	fbi->filtertap.vertical_filter_tap = 3;
	fbi->overlayrect.brx = 320;
	fbi->overlayrect.bry = 240;
	fbi->overlayrect.tlx = 0;
	fbi->overlayrect.tly = 0;
	fbi->overlaysize.height = 160;
	fbi->overlaysize.width = 120;
	fbi->rotangle = 0;
	fbi->tile_mode = 0;
	fbi->colorkey.enable = 0;

	/* set other parameter*/
	info->flags = FBINFO_DEFAULT | FBINFO_PARTIAL_PAN_OK |FBINFO_HWACCEL_XPAN | FBINFO_HWACCEL_YPAN;
	info->node = -1;
	info->pseudo_palette = NULL;
	info->fbops = &overlay_ops;

	/* set var parameter*/
	set_overlay_var(&info->var, mi->modes, mi->pix_fmt);

	/* set multibuffer. */
	info->var.xres_virtual = info->var.xres;
	info->var.yres_virtual = info->fix.smem_len /
		(info->var.xres_virtual * (info->var.bits_per_pixel >> 3)) / info->var.yres * info->var.yres;
	if (info->var.yres_virtual > 3 * info->var.yres)
		info->var.yres_virtual = 3 * info->var.yres;

	/* set fix parameter*/
	strlcpy(info->fix.id, "dcultra overlay", 16);
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux = 0;
	info->fix.xpanstep = 0;
	info->fix.ypanstep = info->var.yres;
	info->fix.ywrapstep = 0;
	info->fix.smem_len = PAGE_ALIGN(DEFAULT_OVERLAY_SIZE);
	info->screen_base = thead_dma_alloc(NULL, info->fix.smem_len, &overlay_phys, GFP_KERNEL);
	info->fix.smem_start = (unsigned long)overlay_phys;

	overlay_info = info;

	overlay_check_var(&info->var, info);
	overlay_set_par(info);

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register overlay fb_info\n");
		ret = -ENOMEM;
		goto on_error;
	}

	return 0;

on_error:
	teahd_dma_free(NULL, info->fix.smem_len, info->screen_base, overlay_phys);
	framebuffer_release(overlay_info);

	return ret;
}

static int ultrafb_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	struct ultrafb_info *fbi;
	struct ultrafb_mach_info *mi;
	int ret = 0;
	int pixel_byte = 0;
	_Static_assert(PAGE_ALIGN(DEFAULT_FB_SIZE) + 32 * 32 * 4 < 0x2000000, "error");

	printk("%s probe.\n", __func__);

	mi = &default_mach_info;
	if (!reg_base) {
		struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			dev_err(&pdev->dev, "no IO memory defined\n");
			return -ENOENT;
		}

		reg_base = res->start;
		reg_size = resource_size(res);
	}

	if (irq < 0) {
		dev_err(&pdev->dev, "no IRQ defined\n");
		return -ENOENT;
	}

	info = framebuffer_alloc(sizeof(struct ultrafb_info), &pdev->dev);
	if (info == NULL) {
		dev_err(&pdev->dev, "alloc framebuffer failed\n");
		return -ENOMEM;
	}

	fbi = info->par;
	fbi->info = info;
	fbi->dev = info->dev = &pdev->dev;
	fbi->format = mi->pix_fmt;
	fbi->vblank_count = 0;
	fbi->rotangle = 0;
	fbi->tile_mode = 0;
	fbi->colorkey.enable = 0;
	pixel_byte = get_bpp_by_format(fbi->format);
	fbi->framebuffersize.width = DEFAULT_WIDTH;
	fbi->framebuffersize.height = DEFAULT_HEIGHT;
	fbi->framebuffersize.stride = DEFAULT_WIDTH * pixel_byte;
	/*

	 * Initialise static fb parameters.
	 */
	info->flags = FBINFO_DEFAULT | FBINFO_PARTIAL_PAN_OK |
			  FBINFO_HWACCEL_XPAN | FBINFO_HWACCEL_YPAN;
	info->node = -1;
	strlcpy(info->fix.id, "Vivante dcultra", 16);
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux = 0;
	info->fix.xpanstep = 0;
	info->fix.ypanstep = 0;
	info->fix.ywrapstep = 0;
	info->fix.mmio_start = reg_base;
	info->fix.mmio_len = reg_size;
	info->fix.accel = FB_ACCEL_NONE;
	info->fbops = &ultrafb_ops;
	info->pseudo_palette = fbi->pseudo_palette;
	info->fix.visual = FB_VISUAL_TRUECOLOR;

	info->fix.smem_len = PAGE_ALIGN(DEFAULT_FB_SIZE);

	ret = request_irq(irq, ultrafb_handle_irq,
				   IRQF_SHARED, "dc_ultra", fbi);

	init_waitqueue_head(&vsync_wait);

	request_mem_region(reg_base, reg_size, "dc ultra region");

	info->screen_base = thead_dma_alloc(NULL, info->fix.smem_len, &framebuffer_phys, GFP_KERNEL);
	info->fix.smem_start = (unsigned long)framebuffer_phys;

	fbi->reg_virtual = (void __iomem *)ioremap(reg_base, reg_size);
	reg_virtual = fbi->reg_virtual;

#if DEBUG
	dest_virtual = thead_dma_alloc(NULL, info->fix.smem_len, &dest_phys, GFP_KERNEL);
#endif

	fbi->cursor_start_dma = PAGE_ALIGN(info->fix.smem_start + info->fix.smem_len);
	fbi->cursor_virtual = thead_dma_alloc(NULL, CURSOR_SIZE * CURSOR_SIZE * 4, &fbi->cursor_start_dma, GFP_KERNEL);

	set_mode(&info->var, mi->modes, mi->pix_fmt);

	/* set multibuffer. */
	info->var.xres_virtual = info->var.xres;
	info->var.yres_virtual = info->fix.smem_len /
		(info->var.xres_virtual * (info->var.bits_per_pixel >> 3)) / info->var.yres * info->var.yres;
	if (info->var.yres_virtual > 3 * info->var.yres)
		info->var.yres_virtual = 3 * info->var.yres;
	info->var.activate = FB_ACTIVATE_FORCE;

	ultrafb_check_var(&info->var, info);

	write_register(0x0, FRAMEBUFFER_CONFIG);
	write_register(0x00071900, AQHICLOCKCONTROL);
	write_register(0x00070900, AQHICLOCKCONTROL);

	ultrafb_set_par(info);

	ret = fb_alloc_cmap(&info->cmap, 16, 0);
	if (ret) {
		dev_err(&pdev->dev, "Fail to alloc cmap: %d\n", ret);
		goto on_error;
	}

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register dc-fb: %d\n", ret);
		ret = -ENXIO;
		goto on_error;
	}

	platform_set_drvdata(pdev, fbi);

	ret = dcultra_overlay_init(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to init overlay\n");
		return ret;
	}

	return 0;

on_error:
	if (&info->cmap)
		fb_dealloc_cmap(&info->cmap);

	teahd_dma_free(NULL, CURSOR_SIZE * CURSOR_SIZE * 4, fbi->cursor_virtual, fbi->cursor_start_dma);
#if DEBUG
	teahd_dma_free(NULL, info->fix.smem_len, dest_virtual, dest_phys);
#endif
	iounmap(fbi->reg_virtual);
	teahd_dma_free(NULL, info->fix.smem_len, info->screen_base, framebuffer_phys);
	framebuffer_release(info);

	dev_err(&pdev->dev, "frame buffer device init failed with %d\n", ret);
	return ret;
}

static int ultrafb_remove(struct platform_device *pdev)
{
	struct ultrafb_info *fbi = platform_get_drvdata(pdev);
	struct fb_info *info = fbi->info;
	uint32_t data;

	/* Disable DE */
	data = read_register(PANEL_CONFIG);
	data = data & ~0x1;
	write_register(data, PANEL_CONFIG);

	/* Reset framebuffer */
	write_register(0, FRAMEBUFFER_CONFIG);

	unregister_framebuffer(overlay_info);
	unregister_framebuffer(info);

	if (&info->cmap)
		fb_dealloc_cmap(&info->cmap);

	teahd_dma_free(NULL, info->fix.smem_len, info->screen_base, overlay_phys);

	teahd_dma_free(NULL, CURSOR_SIZE * CURSOR_SIZE * 4, fbi->cursor_virtual, fbi->cursor_start_dma);
#if DEBUG
	teahd_dma_free(NULL, info->fix.smem_len, dest_virtual, dest_phys);
#endif
	iounmap(fbi->reg_virtual);
	teahd_dma_free(NULL, info->fix.smem_len, info->screen_base, framebuffer_phys);

	release_mem_region(info->fix.mmio_start, info->fix.mmio_len);

	framebuffer_release(overlay_info);
	framebuffer_release(info);

	/* release irq */
	free_irq(irq, fbi);
	return 0;
}

static const struct of_device_id ultrafb_of_dev_id[] = {
	{ .compatible = "verisilicon,dc8000-fb", },
	{ }
};

static struct platform_driver ultrafb_driver = {
	.driver   = {
		.name = "viv-dc",
		.of_match_table = ultrafb_of_dev_id,
	},
	.probe = ultrafb_probe,
	.remove = ultrafb_remove,
};
module_platform_driver(ultrafb_driver);

MODULE_DESCRIPTION("Vivante dcultra Driver");
MODULE_LICENSE("GPL");
