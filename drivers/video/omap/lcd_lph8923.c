#include <linux/module.h>
#include <linux/delay.h>

#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>
#include <asm/arch/board.h>
#include <asm/arch/omapfb.h>
#include <asm/arch/lcd_lph8923.h>

#include <linux/spi/spi.h>
#include <asm/arch/mcspi.h>

#include "../../cbus/tahvo.h"

/* #define OMAPFB_DBG 1 */

#include "debug.h"

#define LPH8923_MODULE_NAME		"lcd_lph8923"

#define LPH8923_VER_BUGGY		1
#define LPH8923_VER_NON_BUGGY		3

struct {
	int		enabled;
	int		version;

	u8		display_id[3];
	unsigned int	saved_bklight_level;
	unsigned long	hw_guard_end;		/* next value of jiffies
						   when we can issue the
						   next sleep in/out command */
	unsigned long	hw_guard_wait;		/* max guard time in jiffies */
	struct omapfb_device *fbdev;
	struct spi_device *spi;
} lph8923;

#define LPH8923_CMD_READ_DISP_ID	0x04
#define LPH8923_CMD_READ_RED		0x06
#define LPH8923_CMD_READ_GREEN		0x07
#define LPH8923_CMD_READ_BLUE		0x08
#define LPH8923_CMD_READ_DISP_STATUS	0x09
#define LPH8923_CMD_SLEEP_IN		0x10
#define LPH8923_CMD_SLEEP_OUT		0x11
#define LPH8923_CMD_DISP_OFF		0x28
#define LPH8923_CMD_DISP_ON		0x29

static struct lcd_panel lph8923_panel;

#define LPH8923_SPEED_HZ	12000000

static int lph8923_spi_probe(struct spi_device *spi)
{
	DBGENTER(1);

	spi->dev.power.power_state = PMSG_ON;
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 9;
	spi_setup(spi);

	DBGPRINT(1, "spi %p\n", spi);
	lph8923.spi = spi;

	omapfb_register_panel(&lph8923_panel);

	return 0;
}

static int lph8923_spi_remove(struct spi_device *spi)
{
	DBGENTER(1);

	lph8923.spi = NULL;

	return 0;
}

static struct spi_driver lph8923_spi_driver = {
	.driver = {
		.name	= LPH8923_MODULE_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= lph8923_spi_probe,
	.remove	= __devexit_p(lph8923_spi_remove),
};

static int lph8923_drv_init(void)
{
	spi_register_driver(&lph8923_spi_driver);

	return 0;
}
module_init(lph8923_drv_init);

static void lph8923_drv_cleanup(void)
{
	spi_unregister_driver(&lph8923_spi_driver);
}
module_exit(lph8923_drv_cleanup);

static void set_spi_data_width(int width)
{
	if (lph8923.spi->bits_per_word != width) {
		lph8923.spi->bits_per_word = width;
		spi_setup(lph8923.spi);
	}
}

static void lph8923_read(int cmd, u8 *buf, int len)
{
	struct spi_message	m;
	struct spi_transfer	t;
	u16			w;

	BUG_ON(lph8923.spi == NULL);

	spi_message_init(&m);
	m.spi = lph8923.spi;

	if (len > 1) {
		cmd <<= 1;
		set_spi_data_width(10);
	} else
		set_spi_data_width(9);

	w		= cmd;
	t.cs_change	= len ? 1 : 0;
	t.tx_buf	= &w;
	t.rx_buf	= NULL;
	t.len		= 2;

	spi_message_add_tail(&t, &m);

	spi_sync(m.spi, &m);

	if (!len)
		return;

	spi_message_init(&m);
	m.spi = lph8923.spi;

	t.cs_change	= 0;
	t.tx_buf	= NULL;
	t.rx_buf	= buf;
	t.len		= len;

	set_spi_data_width(8);

	spi_message_add_tail(&t, &m);

	spi_sync(m.spi, &m);
}

static void lph8923_write(int cmd, const u8 *buf, int len)
{
	struct spi_message	m;
	struct spi_transfer	t;
	u16			w;
	int			i;

	BUG_ON(lph8923.spi == NULL);

	spi_message_init(&m);
	m.spi = lph8923.spi;

	if (len > 1) {
		cmd <<= 1;
		set_spi_data_width(10);
	} else {
		set_spi_data_width(9);
	}

	t.cs_change	= 0;
	w		= cmd;
	t.tx_buf	= &w;
	t.rx_buf	= NULL;
	t.len		= 2;

	spi_message_add_tail(&t, &m);
	spi_sync(m.spi, &m);

	if (!len)
		return;

	set_spi_data_width(9);

	t.tx_buf = &w;
	for (i = 0; i < len; i++) {
		u16 w;

		spi_message_init(&m);
		m.spi = lph8923.spi;
		spi_message_add_tail(&t, &m);
		w = buf[i] | (1 << 8);
		spi_sync(m.spi, &m);
	}
}

static inline void lph8923_cmd(int cmd)
{
	lph8923_write(cmd, NULL, 0);
}

struct cmd_data {
	u8 cmd;
	u8 len;
	const u8 *data;
} __attribute__ ((packed));;

static const struct cmd_data init_cmds_buggy_lph8923[] = {
	{ 0xb0, 1, "\x08" },
	{ 0xb1, 2, "\x0b\x1c" },
	{ 0xb2, 4, "\x00\x00\x00\x00" },
	{ 0xb3, 4, "\x00\x00\x00\x00" },
	{ 0xb4, 1, "\x87" },
	{ 0xb5, 4, "\x37\x07\x37\x07" },
	{ 0xb6, 2, "\x64\x24" },
	{ 0xb7, 1, "\x90" },
	{ 0xb8, 3, "\x10\x11\x20" },
	{ 0xb9, 2, "\x31\x02" },
	{ 0xba, 3, "\x04\xa3\x9d" },
	{ 0xbb, 4, "\x15\xb2\x8c\x00" },
	{ 0xc2, 3, "\x02\x00\x00" },
};

static const struct cmd_data init_cmds_non_buggy_lph8923[] = {
	{ 0xc2, 3, "\x02\x00\x00" },
};

static inline void lph8923_set_16bit_mode(void)
{
	lph8923_write(0x3a, "\x50", 1);
}

static void lph8923_send_init_string(void)
{
	int c;
	const struct cmd_data *cd;

	switch (lph8923.version) {
	case LPH8923_VER_BUGGY:
		c = sizeof(init_cmds_buggy_lph8923)/sizeof(init_cmds_buggy_lph8923[0]);
		cd = init_cmds_buggy_lph8923;
		break;
	case LPH8923_VER_NON_BUGGY:
	default:
		c = sizeof(init_cmds_non_buggy_lph8923)/sizeof(init_cmds_non_buggy_lph8923[0]);
		cd = init_cmds_non_buggy_lph8923;
		break;
	}
	while (c--) {
		lph8923_write(cd->cmd, cd->data, cd->len);
		cd++;
	}
	lph8923_set_16bit_mode();
}

static void hw_guard_start(int guard_msec)
{
	lph8923.hw_guard_wait = msecs_to_jiffies(guard_msec);
	lph8923.hw_guard_end = jiffies + lph8923.hw_guard_wait;
}

static void hw_guard_wait(void)
{
	unsigned long wait = lph8923.hw_guard_end - jiffies;

	if ((long)wait > 0 && wait <= lph8923.hw_guard_wait) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(wait);
	}
}

static void lph8923_set_sleep_mode(int on)
{
	int cmd, sleep_time = 5;

	if (on)
		cmd = LPH8923_CMD_SLEEP_IN;
	else
		cmd = LPH8923_CMD_SLEEP_OUT;
	hw_guard_wait();
	lph8923_cmd(cmd);
	hw_guard_start(120);
	/* When we enable the panel, it seems we _have_ to sleep
	 * 120 ms before sending the init string */
	if (!on)
		sleep_time = 120;
	msleep(sleep_time);
}

static void lph8923_set_display_state(int enabled)
{
	int cmd = enabled ? LPH8923_CMD_DISP_ON : LPH8923_CMD_DISP_OFF;

	lph8923_cmd(cmd);
}

static void lph8923_detect(void)
{
	lph8923_read(LPH8923_CMD_READ_DISP_ID, lph8923.display_id, 3);
	printk(KERN_INFO "Moscow display id: %02x%02x%02x\n",
		lph8923.display_id[0], lph8923.display_id[1],
		lph8923.display_id[2]);

	if (lph8923.display_id[0] == 0x45) {
		lph8923.version = LPH8923_VER_NON_BUGGY;
		printk(KERN_INFO "Non-buggy Moscow detected\n");
		return;
	} else {
		lph8923.version = LPH8923_VER_BUGGY;
		printk(KERN_INFO "Buggy Moscow detected\n");
	}
}

static int lph8923_enabled(void)
{
	u32 disp_status;
	int enabled;

	lph8923_read(LPH8923_CMD_READ_DISP_STATUS, (u8 *)&disp_status, 4);
	disp_status = __be32_to_cpu(disp_status);
	enabled = (disp_status & (1 << 17)) && (disp_status & (1 << 10));
	DBGPRINT(1, ": panel %senabled by bootloader (status 0x%04x)\n",
		enabled ? "" : "not ", disp_status);
	return enabled;
}

static int lph8923_panel_init(struct omapfb_device *fbdev)
{
	lph8923.fbdev = fbdev;

	lph8923.enabled = 1;
	lph8923_detect();
	if (lph8923.version == LPH8923_VER_NON_BUGGY)
		lph8923.enabled = lph8923_enabled();
	else
		/* We can't be sure, but assume the bootloader
		* enabled it already */
		lph8923.enabled = 1;

	return 0;
}

static void lph8923_panel_cleanup(void)
{
}

static int lph8923_panel_set_bklight_level(unsigned int level)
{
	if (level > 0xf)
		return -EINVAL;
	if (!lph8923.enabled) {
		lph8923.saved_bklight_level = level;
		return 0;
	}
	level = level * tahvo_get_max_backlight_level() / 0x0f;
	tahvo_set_backlight_level(level);

	return 0;
}

static unsigned int lph8923_panel_get_bklight_level(void)
{
	return tahvo_get_backlight_level() * 0x0f /
		tahvo_get_max_backlight_level();
}

static unsigned int lph8923_panel_get_bklight_max(void)
{
	return 0x0f;
}

static int lph8923_panel_enable(void)
{
	if (lph8923.enabled)
		return 0;

	lph8923_set_sleep_mode(0);
	lph8923.enabled = 1;
	lph8923_send_init_string();
	lph8923_set_display_state(1);
	lph8923_panel_set_bklight_level(lph8923.saved_bklight_level);

	return 0;
}

static void lph8923_panel_disable(void)
{
	if (!lph8923.enabled)
		return;
	lph8923.saved_bklight_level = lph8923_panel_get_bklight_level();
	lph8923_panel_set_bklight_level(0);
	lph8923_set_display_state(0);
	lph8923_set_sleep_mode(1);
	lph8923.enabled = 0;
}

static unsigned long lph8923_panel_get_caps(void)
{
	return OMAPFB_CAPS_SET_BACKLIGHT;
}

static u16 read_first_pixel(void)
{
	u8 b;
	u16 pixel;

	lph8923_read(LPH8923_CMD_READ_RED, &b, 1);
	pixel = (b >> 1) << 11;
	lph8923_read(LPH8923_CMD_READ_GREEN, &b, 1);
	pixel |= b << 5;
	lph8923_read(LPH8923_CMD_READ_BLUE, &b, 1);
	pixel |= (b >> 1);

	return pixel;
}

static int lph8923_panel_test(int test_num)
{
	static const u16 test_values[4] = {
		0x0000, 0xffff, 0xaaaa, 0x5555,
	};
	int i;

	if (test_num != LCD_LPH8923_TEST_RGB_LINES)
		return LCD_LPH8923_TEST_INVALID;

	for (i = 0; i < ARRAY_SIZE(test_values); i++) {
		int delay;
		unsigned long tmo;

		omapfb_write_first_pixel(lph8923.fbdev, test_values[i]);
		tmo = jiffies + msecs_to_jiffies(100);
		delay = msecs_to_jiffies(25);
		while (1) {
			u16 pixel;

			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout(delay);
			pixel = read_first_pixel();
			if (pixel == test_values[i])
				break;
			if (time_after(jiffies, tmo)) {
				printk(KERN_ERR "Moscow RGB I/F test failed: "
				       "expecting %04x, got %04x\n",
				       test_values[i], pixel);
				return LCD_LPH8923_TEST_FAILED;
			}
			delay = msecs_to_jiffies(10);
		}
	}

	return 0;
}

static struct lcd_panel lph8923_panel = {
	.name		= "lph8923",
	.config		= OMAP_LCDC_PANEL_TFT,

	.bpp		= 16,
	.data_lines	= 16,
	.x_res		= 800,
	.y_res		= 480,
	.pixel_clock	= 21940,
	.hsw		= 50,
	.hfp		= 20,
	.hbp		= 15,
	.vsw		= 2,
	.vfp		= 1,
	.vbp		= 3,

	.init		= lph8923_panel_init,
	.cleanup	= lph8923_panel_cleanup,
	.enable		= lph8923_panel_enable,
	.disable	= lph8923_panel_disable,
	.get_caps	= lph8923_panel_get_caps,
	.set_bklight_level= lph8923_panel_set_bklight_level,
	.get_bklight_level= lph8923_panel_get_bklight_level,
	.get_bklight_max= lph8923_panel_get_bklight_max,
	.run_test	= lph8923_panel_test,
};
