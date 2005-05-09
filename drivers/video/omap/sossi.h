#ifndef DRIVERS_VIDEO_OMAP_SOSSI_H
#define DRIVERS_VIDEO_OMAP_SOSSI_H

#define SOSSI_FLAG_HS_INVERTED          0x01
#define SOSSI_FLAG_VS_INVERTED          0x02

extern int sossi_init(void);
extern void sossi_set_xfer_params(int tw0, int tw1, int bus_pick_count, int bus_pick_width);
extern void sossi_start_transfer(void);
extern void sossi_stop_transfer(void);
extern void sossi_send_cmd(const void *data, unsigned int len);
extern void sossi_send_data(const void *data, unsigned int len);
extern void sossi_send_data_const32(u32 data, unsigned int count);
extern void sossi_prepare_dma_transfer(unsigned int count);
extern void sossi_read_data(void *data, unsigned int len);
extern void sossi_set_tearing(int mode, int hs_counter, int detect_limit,
			      int vs_counter, int vs_detect_limit, int flags);

#endif
