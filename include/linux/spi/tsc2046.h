#ifndef _LINUX_SPI_TSC2046_H
#define _LINUX_SPI_TSC2046_H

#include <linux/types.h>
#include <linux/timer.h>

struct tsc2046_platform_data {
	s16	dav_gpio;
	s16	gpio_debounce;
	u16	ts_x_plate_ohm;
	u32	ts_max_pressure;  /* Samples with bigger pressure value will
				     be ignored, since the corresponding X, Y
				     values are unreliable */
	u32	ts_touch_pressure;/* Pressure limit until we report a
				     touch event. After that we switch
				     to ts_max_pressure. */
	unsigned ts_ignore_last : 1;

};

struct tsc2046_ts;

struct tsc2046 {
	struct spi_device	*spi;
	int                     gpio;

	struct tsc2046_ts	*ts;
};

/* The TSC2046 operates at a maximum speed of 2MHz */
#define TSC2046_HZ	2000000

#define TSC2046_DECL_MOD(module)					\
extern int  tsc2046_##module##_init(struct tsc2046 *tsc,		\
			   struct tsc2046_platform_data *pdata);	\
extern void tsc2046_##module##_exit(struct tsc2046 *tsc);		\
extern int  tsc2046_##module##_suspend(struct tsc2046 *tsc);		\
extern void tsc2046_##module##_resume(struct tsc2046 *tsc);

#define TSC2046_DECL_EMPTY_MOD(module)					\
static inline int tsc2046_##module##_init(struct tsc2046 *tsc,		\
			   struct tsc2046_platform_data *pdata)		\
{									\
	return 0;							\
}									\
static inline void tsc2046_##module##_exit(struct tsc2046 *tsc) {}	\
static inline int  tsc2046_##module##_suspend(struct tsc2046 *tsc)	\
{									\
	return 0;							\
}									\
static inline void tsc2046_##module##_resume(struct tsc2046 *tsc) {}

#if defined(CONFIG_TOUCHSCREEN_TSC2046) || \
    defined(CONFIG_TOUCHSCREEN_TSC2046_MODULE)
TSC2046_DECL_MOD(ts)
#else
TSC2046_DECL_EMPTY_MOD(ts)
#endif

#endif
