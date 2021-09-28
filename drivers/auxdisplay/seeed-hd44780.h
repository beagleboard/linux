#ifndef _SEEED_HD44780_H_
#define _SEEED_HD44780_H_

#define BUF_SIZE		64
#define ESC_SEQ_BUF_SIZE	4

struct hd44780_geometry {
	int cols;
	int rows;
	int start_addrs[];
};

struct hd44780 {
	struct cdev cdev;
	struct device *device;
	struct i2c_client *i2c_client;
	struct hd44780_geometry *geometry;

	/* Current cursor positon on the display */
	struct {
		int row;
		int col;
	} pos;

	char buf[BUF_SIZE];
	struct {
		char buf[ESC_SEQ_BUF_SIZE];
		int length;
	} esc_seq_buf;
	bool is_in_esc_seq;

	bool backlight;
	bool cursor_blink;
	bool cursor_display;

	bool dirty;

	struct mutex lock;
	struct list_head list;
};

void hd44780_write(struct hd44780 *, const char *, size_t);
void hd44780_init_lcd(struct hd44780 *);
void hd44780_print(struct hd44780 *, const char *);
void hd44780_flush(struct hd44780 *);
void hd44780_set_geometry(struct hd44780 *, struct hd44780_geometry *);
void hd44780_set_backlight(struct hd44780 *, bool);
void hd44780_set_cursor_blink(struct hd44780 *, bool);
void hd44780_set_cursor_display(struct hd44780 *, bool);

extern struct hd44780_geometry *hd44780_geometries[];
#endif
