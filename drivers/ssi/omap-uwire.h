#ifndef __ARCH_OMAP_UWIRE_H
#define __ARCH_OMAP_UWIRE_H

#define UWIRE_READ_FALLING_EDGE		0x0000
#define UWIRE_READ_RISING_EDGE		0x0001
#define UWIRE_WRITE_FALLING_EDGE	0x0000
#define UWIRE_WRITE_RISING_EDGE		0x0002
#define UWIRE_CS_ACTIVE_LOW		0x0000
#define UWIRE_CS_ACTIVE_HIGH		0x0004
#define UWIRE_FREQ_DIV_2		0x0000
#define UWIRE_FREQ_DIV_4		0x0008
#define UWIRE_FREQ_DIV_8		0x0010
#define UWIRE_CHK_READY			0x0020
#define UWIRE_CLK_INVERTED		0x0040

/*
 * uWire for OMAP declarations
 */
extern void omap_uwire_configure_mode(int cs, unsigned long flags);

/* NOTE: Make sure you don't call this from an interrupt handler! */
extern int omap_uwire_data_transfer(int cs, u16 tx_data, int tx_size,
				    int rx_size, u16 *rx_buf,
				    int leave_cs_active);

#endif
