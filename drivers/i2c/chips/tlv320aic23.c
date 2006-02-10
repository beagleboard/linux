/*
 *   Texas Instrumens TLV320AIC23 audio codec's i2c interface.
 *
 *   Copyright (c) by Kai Svahn <kai.svahn@nokia.com>
 *   Copyright (c) by Jussi Laako <jussi.laako@nokia.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/arch/aic23.h>
#include <asm/arch/mcbsp.h>

#define TLV320AIC23_VERSION	"1.8"
#define TLV320AIC23_DATE	"10-Feb-2006"
#define MAX_VOL			100
#define MIN_VOL			0
#define MAX_GAIN		100
#define MIN_GAIN		0
#define OUTPUT_VOLUME_MIN       LHV_MIN
#define OUTPUT_VOLUME_MAX       LHV_MAX
#define OUTPUT_VOLUME_RANGE     (OUTPUT_VOLUME_MAX - OUTPUT_VOLUME_MIN)
#define INPUT_VOLUME_MIN 	LIV_MIN
#define INPUT_VOLUME_MAX 	LIV_MAX
#define INPUT_VOLUME_RANGE 	(INPUT_VOLUME_MAX - INPUT_VOLUME_MIN)

/* I2C Addresses to scan */
static unsigned short normal_i2c[] = { TLV320AIC23ID1, TLV320AIC23ID2, \
				       I2C_CLIENT_END };
/*static unsigned short normal_i2c_range[] = { I2C_CLIENT_END };*/

/* This makes all addr_data:s */
I2C_CLIENT_INSMOD;

static struct i2c_driver tlv320aic23_driver;
static struct i2c_client *new_client;
static int selftest;

static struct tlv320aic23_info {
	u16 volume_reg_left;
	u16 volume_reg_right;
	u16 input_gain_reg_left;
	u16 input_gain_reg_right;
	u16 power;			/* For POWER_DOWN_CONTROL_ADDR */
	u16 mask;			/* For ANALOG_AUDIO_CONTROL_ADDR */
	int mic_loopback;
	int mic_enable;
	int sta;
	int power_down;
	int initialization;
} tlv320aic23_info_l;

static int _tlv320aic23_write_value(struct i2c_client *client, u8 reg, u16 value)
{
	u8 val, wreg;

	/* TLV320AIC23 has 7 bit address and 9 bits of data
	 * so we need to switch one data bit into reg and rest
	 * of data into val
	 */

	wreg = (reg << 1);
	val = (0x01 & (value >> 8));
	wreg = (wreg | val);
	val = (0x00ff & value);

	return i2c_smbus_write_byte_data(client, wreg, val);
}

int tlv320aic23_write_value(u8 reg, u16 value)
{
	static struct i2c_client *client;
	client = new_client;
	_tlv320aic23_write_value(client, reg, value);

	return 0;
}

static int tlv320aic23_detect_client(struct i2c_adapter *adapter, int address,
				     int kind)
{
	int err = 0;
	const char *client_name = "TLV320AIC23 Audio Codec";

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA |
				     I2C_FUNC_SMBUS_WRITE_BYTE)) {
		printk(KERN_WARNING "%s functinality check failed\n", 
		       client_name);
		return err;
	}

	if (!(new_client = kmalloc(sizeof(struct i2c_client),
				   GFP_KERNEL))) {
		err = -ENOMEM;
		printk(KERN_WARNING "Couldn't allocate memory for %s\n", 
		       client_name);
		return err;
	}

	memset(new_client, 0x00, sizeof(struct i2c_client));
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &tlv320aic23_driver;
	new_client->flags = 0;
	strlcpy(new_client->name, client_name, I2C_NAME_SIZE);

	if ((err = i2c_attach_client(new_client))) {
		printk(KERN_WARNING "Couldn't attach %s\n", client_name);
		kfree(new_client);
		return err;
	}
	return 0;
}

static int tlv320aic23_detach_client(struct i2c_client *client)
{
	int err;

	if ((err = i2c_detach_client(client))) {
		printk("tlv320aic23.o: Client deregistration failed, \
		       client not detached.\n");
		return err;
	}
	kfree(client);
	return 0;
}

static int tlv320aic23_attach_adapter(struct i2c_adapter *adapter)
{
	int res;

	res = i2c_probe(adapter, &addr_data, &tlv320aic23_detect_client);
	return res;
}

static struct i2c_driver tlv320aic23_driver = {
	.driver {
		.name	= "OMAP+TLV320AIC23 codec",
		/*.flags	= I2C_DF_NOTIFY,*/
	},
	.id		= I2C_DRIVERID_MISC, /* Experimental ID */
	.attach_adapter	= tlv320aic23_attach_adapter,
	.detach_client	= tlv320aic23_detach_client,
};

/*
 * Configures the McBSP3 which is used to send clock to the AIC23 codec.
 * The input clock rate from DSP is 12MHz.
 * The DSP clock must be on before this is called. 
 */
static int omap_mcbsp3_tlv320aic23_clock_init(void)
{
	u16 w;
	
	/* enable 12MHz clock to mcbsp 1 & 3 */
	__raw_writew(__raw_readw(DSP_IDLECT2) | (1<<1), DSP_IDLECT2);
	__raw_writew(__raw_readw(DSP_RSTCT2) | 1 | 1<<1, DSP_RSTCT2);

	/* disable sample rate generator */
	OMAP_MCBSP_WRITE(OMAP1610_MCBSP3_BASE, SPCR1, 0x0000); 
	OMAP_MCBSP_WRITE(OMAP1610_MCBSP3_BASE, SPCR2, 0x0000);
	
	/* pin control register */
	OMAP_MCBSP_WRITE(OMAP1610_MCBSP3_BASE, PCR0,(CLKXM | CLKXP | CLKRP));
	
	/* configure srg to send 12MHz pulse from dsp peripheral clock */
	OMAP_MCBSP_WRITE(OMAP1610_MCBSP3_BASE, SRGR1, 0x0000); 
	OMAP_MCBSP_WRITE(OMAP1610_MCBSP3_BASE, SRGR2, CLKSM); 
	
	/* enable sample rate generator */
	w = OMAP_MCBSP_READ(OMAP1610_MCBSP3_BASE, SPCR2);
	OMAP_MCBSP_WRITE(OMAP1610_MCBSP3_BASE, SPCR2, (w | FREE | GRST)); 
	printk("Clock enabled to MCBSP1 & 3 \n");
	
	return 0;
}

static void update_volume_left(int volume)
{
	u16 val = 0;
	val = ((volume * OUTPUT_VOLUME_RANGE) / 100) + OUTPUT_VOLUME_MIN;
	tlv320aic23_write_value(LEFT_CHANNEL_VOLUME_ADDR, val);
	tlv320aic23_info_l.volume_reg_left = volume;
}

static void update_volume_right(int volume)
{
	u16 val = 0;
	val = ((volume * OUTPUT_VOLUME_RANGE) / 100) + OUTPUT_VOLUME_MIN;
	tlv320aic23_write_value(RIGHT_CHANNEL_VOLUME_ADDR, val);
	tlv320aic23_info_l.volume_reg_right = volume;
}

static void set_mic(int mic_en)
{
	u16 dg_ctrl;

	if (mic_en) {
		tlv320aic23_info_l.power = OSC_OFF | LINE_OFF;
		dg_ctrl = ADCHP_ON;
		tlv320aic23_info_l.mask &= ~MICM_MUTED;
		tlv320aic23_info_l.mask |= MICB_20DB; /* STE_ENABLED */
	} else {
		tlv320aic23_info_l.power =
			OSC_OFF | ADC_OFF | MIC_OFF | LINE_OFF;
		dg_ctrl = 0x00;
		tlv320aic23_info_l.mask = 
			DAC_SELECTED | INSEL_MIC | MICM_MUTED;
	}
	tlv320aic23_write_value(POWER_DOWN_CONTROL_ADDR,
				tlv320aic23_info_l.power);
	tlv320aic23_write_value(DIGITAL_AUDIO_CONTROL_ADDR, dg_ctrl);
	tlv320aic23_write_value(ANALOG_AUDIO_CONTROL_ADDR,
				tlv320aic23_info_l.mask);
	tlv320aic23_info_l.mic_enable = mic_en;

	printk(KERN_INFO "tlv320aic23 mic state: %i\n", mic_en);
}

static void tlv320aic23_init_power(void)
{
	tlv320aic23_write_value(RESET_CONTROL_ADDR, 0x00);
	
	if (tlv320aic23_info_l.initialization == 0) {
		tlv320aic23_write_value(LEFT_CHANNEL_VOLUME_ADDR, LHV_MIN);
		tlv320aic23_write_value(RIGHT_CHANNEL_VOLUME_ADDR, LHV_MIN);
	}
	else {
		update_volume_left(tlv320aic23_info_l.volume_reg_left);
		update_volume_right(tlv320aic23_info_l.volume_reg_right);
	}
	
	tlv320aic23_info_l.mask = DAC_SELECTED | INSEL_MIC | MICM_MUTED;
	tlv320aic23_write_value(ANALOG_AUDIO_CONTROL_ADDR,
			        tlv320aic23_info_l.mask);
	tlv320aic23_write_value(DIGITAL_AUDIO_CONTROL_ADDR, 0x00);
	tlv320aic23_write_value(DIGITAL_AUDIO_FORMAT_ADDR, LRP_ON | FOR_DSP);
	tlv320aic23_write_value(SAMPLE_RATE_CONTROL_ADDR, USB_CLK_ON);
	tlv320aic23_write_value(DIGITAL_INTERFACE_ACT_ADDR, ACT_ON);
	tlv320aic23_info_l.power = OSC_OFF | ADC_OFF | MIC_OFF | LINE_OFF;
	tlv320aic23_write_value(POWER_DOWN_CONTROL_ADDR,
			        tlv320aic23_info_l.power);

	/* enable mic input */
	if (tlv320aic23_info_l.mic_enable)
		set_mic(tlv320aic23_info_l.mic_enable);

	printk(KERN_INFO "tlv320aic23_init_power() done\n");
}

void tlv320aic23_power_down(void)
{
	printk("tlv320aic23 powering down\n");
	tlv320aic23_write_value(POWER_DOWN_CONTROL_ADDR, 0xff);
	tlv320aic23_info_l.power_down = 1;
}

void tlv320aic23_power_up(void)
{
	printk("tlv320aic23 powering up\n");
	tlv320aic23_init_power();
	tlv320aic23_info_l.power_down = 0;
}

/*----------------------------------------------------------------------*/
/*			sysfs initializations				*/
/*----------------------------------------------------------------------*/

static ssize_t store_volume_left(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	signed volume;

	sscanf(buf, "%i", &volume);

	if (volume < MIN_VOL) {
		tlv320aic23_power_down();
		return count;
	} else if (volume > MIN_VOL && tlv320aic23_info_l.power_down) {
		tlv320aic23_info_l.volume_reg_left = volume;
		tlv320aic23_power_up();
		return count;
	}
	if (volume > MAX_VOL)
		volume = MAX_VOL;

	update_volume_left(volume);
	return count;
}

static ssize_t show_volume_left(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", tlv320aic23_info_l.volume_reg_left);
}

static DEVICE_ATTR(volume_left, S_IRUGO | S_IWUGO,
		   show_volume_left, store_volume_left);

static ssize_t store_volume_right(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	signed volume;

	sscanf(buf, "%i", &volume);
	if (volume < MIN_VOL) {
		tlv320aic23_power_down();
		return count;
	} else if (volume > MIN_VOL && tlv320aic23_info_l.power_down) {
		tlv320aic23_info_l.volume_reg_right = volume;
		tlv320aic23_power_up();
		return count;
	}
	if (volume > MAX_VOL)
		volume = MAX_VOL;

	update_volume_right(volume);
	return count;
}

static ssize_t show_volume_right(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", tlv320aic23_info_l.volume_reg_right);
}

static DEVICE_ATTR(volume_right, S_IRUGO | S_IWUGO,
		   show_volume_right, store_volume_right);

static ssize_t store_gain_left(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	u16 val = 0;
	unsigned gain;

	sscanf(buf, "%u", &gain);
	if (gain > MAX_VOL)
		gain = MAX_VOL;

	val = ((gain * INPUT_VOLUME_RANGE) / 100) + INPUT_VOLUME_MIN;
	tlv320aic23_write_value(LEFT_LINE_VOLUME_ADDR, val);
	tlv320aic23_info_l.input_gain_reg_left = gain;

	return count;
}

static ssize_t show_gain_left(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", tlv320aic23_info_l.input_gain_reg_left);
}

static DEVICE_ATTR(gain_left, S_IRUGO | S_IWUSR, show_gain_left,
		   store_gain_left);

static ssize_t store_gain_right(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	u16 val = 0;
	unsigned gain;

	sscanf(buf, "%u", &gain);
	if (gain > MAX_VOL)
		gain = MAX_VOL;

	val = ((gain * INPUT_VOLUME_RANGE) / 100) + INPUT_VOLUME_MIN;
	tlv320aic23_write_value(RIGHT_LINE_VOLUME_ADDR, val);
	tlv320aic23_info_l.input_gain_reg_right = gain;

	return count;
}

static ssize_t show_gain_right(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", tlv320aic23_info_l.input_gain_reg_right);
}

static DEVICE_ATTR(gain_right, S_IRUGO | S_IWUSR, show_gain_right,
		   store_gain_right);

static ssize_t store_mic_loopback(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int mic;

	sscanf(buf, "%i", &mic);
	if (mic > 0) {
		tlv320aic23_write_value(POWER_DOWN_CONTROL_ADDR, \
					OSC_OFF | ADC_OFF | LINE_OFF);
		tlv320aic23_info_l.mask = STE_ENABLED | DAC_SELECTED \
					  | INSEL_MIC | MICB_20DB;
		tlv320aic23_write_value(ANALOG_AUDIO_CONTROL_ADDR, 
					tlv320aic23_info_l.mask);
		mic = 1;
	}
	else {
		tlv320aic23_write_value(POWER_DOWN_CONTROL_ADDR, \
					OSC_OFF | ADC_OFF | MIC_OFF | LINE_OFF);
		mic = 0;
	}
	tlv320aic23_info_l.mic_loopback = mic;

	return count;
}

static ssize_t show_mic_loopback(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", tlv320aic23_info_l.mic_loopback);
}

static DEVICE_ATTR(mic_loopback, S_IRUGO | S_IWUSR,
		   show_mic_loopback, store_mic_loopback);

static ssize_t store_st_attenuation(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned sta;
	u16 tmp;

	sscanf(buf, "%u", &sta);
	if (sta > 3)
		sta = 3;

	tmp = tlv320aic23_info_l.mask;
	tmp &= 0x3f;

	tlv320aic23_info_l.mask =  tmp | STA_REG(sta);
	tlv320aic23_write_value(ANALOG_AUDIO_CONTROL_ADDR,
				tlv320aic23_info_l.mask);
	tlv320aic23_info_l.sta = sta;

	return count;
}

static ssize_t show_st_attenuation(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", tlv320aic23_info_l.sta);
}

static DEVICE_ATTR(st_attenuation, S_IRUGO | S_IWUSR,
		   show_st_attenuation, store_st_attenuation);

static ssize_t store_mic_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int mic;

	sscanf(buf, "%i", &mic);
	set_mic(mic);

	return count;
}

static ssize_t show_mic_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", tlv320aic23_info_l.mic_enable);
}

static DEVICE_ATTR(mic_enable, S_IRUGO | S_IWUSR,
	show_mic_enable, store_mic_enable);

static ssize_t show_audio_selftest(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", selftest);
}

static DEVICE_ATTR(audio_selftest, S_IRUGO | S_IWUSR,
		show_audio_selftest, NULL);

static int audio_i2c_probe(struct platform_device *dev)
{
	int r;

	if ((r = device_create_file(&dev->dev, &dev_attr_volume_left)) != 0)
		return r;
	else if ((r = device_create_file(&dev->dev,
		&dev_attr_volume_right)) != 0)
		goto err_volume_left;
	else if ((r = device_create_file(&dev->dev,
		&dev_attr_gain_right)) != 0)
		goto err_volume_right;
	else if ((r = device_create_file(&dev->dev,
		&dev_attr_gain_left)) != 0)
		goto err_gain_right;
	else if ((r = device_create_file(&dev->dev,
		&dev_attr_mic_loopback)) != 0)
		goto err_gain_left;
	else if ((r = device_create_file(&dev->dev,
		&dev_attr_mic_enable)) != 0)
		goto err_mic_loopback;
	else if ((r = device_create_file(&dev->dev,
		&dev_attr_st_attenuation)) != 0)
		goto err_mic_enable;
	else if ((r = device_create_file(&dev->dev,
		&dev_attr_audio_selftest)) != 0)
		goto err_st_attenuation;
	else
		return r;

err_st_attenuation:
	device_remove_file(&dev->dev, &dev_attr_st_attenuation);
err_mic_enable:
	device_remove_file(&dev->dev, &dev_attr_mic_enable);
err_mic_loopback:
	device_remove_file(&dev->dev, &dev_attr_mic_loopback);
err_gain_left:
	device_remove_file(&dev->dev, &dev_attr_gain_left);
err_gain_right:
	device_remove_file(&dev->dev, &dev_attr_gain_right);
err_volume_right:
	device_remove_file(&dev->dev, &dev_attr_volume_right);
err_volume_left:
	device_remove_file(&dev->dev, &dev_attr_volume_left);

	return r;
}

static int audio_i2c_remove(struct platform_device *dev)
{
	device_remove_file(&dev->dev, &dev_attr_st_attenuation);
	device_remove_file(&dev->dev, &dev_attr_mic_enable);
	device_remove_file(&dev->dev, &dev_attr_mic_loopback);
	device_remove_file(&dev->dev, &dev_attr_gain_left);
	device_remove_file(&dev->dev, &dev_attr_gain_right);
	device_remove_file(&dev->dev, &dev_attr_volume_right);
	device_remove_file(&dev->dev, &dev_attr_volume_left);

	return 0;
}

/*----------------------------------------------------------------*/
/*			PM functions				  */
/*----------------------------------------------------------------*/

static void audio_i2c_shutdown(struct platform_device *dev)
{
	/* Let's mute the codec before powering off to prevent
	* glitch in the sound
	*/
	tlv320aic23_write_value(LEFT_CHANNEL_VOLUME_ADDR, LHV_MIN);
	tlv320aic23_write_value(RIGHT_CHANNEL_VOLUME_ADDR, LHV_MIN);
	tlv320aic23_power_down();
}

static int audio_i2c_suspend(struct platform_device *dev, pm_message_t state)
{
	/* Let's mute the codec before powering off to prevent
	 * glitch in the sound
	 */
	tlv320aic23_write_value(LEFT_CHANNEL_VOLUME_ADDR, LHV_MIN);
	tlv320aic23_write_value(RIGHT_CHANNEL_VOLUME_ADDR, LHV_MIN);
	tlv320aic23_power_down();

	return 0;
}

static int audio_i2c_resume(struct platform_device *dev)
{
	tlv320aic23_power_up();

	return 0;
}

static struct platform_driver audio_i2c_driver = {
	.driver {
		.owner	= THIS_MODULE,
		.name	= "audio-i2c",
	},
	.shutdown	= audio_i2c_shutdown,
	.probe		= audio_i2c_probe,
	.remove		= audio_i2c_remove,
	.suspend	= audio_i2c_suspend,
	.resume		= audio_i2c_resume,
};

static struct platform_device audio_i2c_device = {
	.name		= "audio-i2c",
	.id		= -1,
};

/*----------------------------------------------------------------*/

static int __init tlv320aic23_init(void)
{
	selftest =  0;
	tlv320aic23_info_l.initialization = 0;

	if (i2c_add_driver(&tlv320aic23_driver)) {
		printk("tlv320aic23 i2c: Driver registration failed, \
		      module not inserted.\n");
		selftest= -ENODEV;
		return selftest;
	}

	if (platform_driver_register(&audio_i2c_driver)) {
		printk(KERN_WARNING "Failed to register audio i2c driver\n");
		selftest = -ENODEV;
		return selftest;
	}

	if (platform_device_register(&audio_i2c_device)) {
		printk(KERN_WARNING "Failed to register audio i2c device\n");
		platform_driver_unregister(&audio_i2c_driver);
		selftest = -ENODEV;
		return selftest;
	}
	omap_mcbsp3_tlv320aic23_clock_init();
	tlv320aic23_power_up();
	tlv320aic23_info_l.initialization = 1;
	printk("TLV320AIC23 I2C version %s (%s)\n", 
	       TLV320AIC23_VERSION, TLV320AIC23_DATE);

	return selftest;
}

static void __exit tlv320aic23_exit(void)
{
	int res;

	tlv320aic23_power_down();
	if ((res = i2c_del_driver(&tlv320aic23_driver))) 
		printk("tlv320aic23 i2c: Driver remove failed, module not removed.\n");

	platform_device_unregister(&audio_i2c_device);
	platform_driver_unregister(&audio_i2c_driver);
}

MODULE_AUTHOR("Kai Svahn <kai.svahn@nokia.com>");
MODULE_DESCRIPTION("I2C interface for TLV320AIC23 codec.");
MODULE_LICENSE("GPL");

module_init(tlv320aic23_init)
module_exit(tlv320aic23_exit)

EXPORT_SYMBOL(tlv320aic23_write_value);
EXPORT_SYMBOL(tlv320aic23_power_up);
EXPORT_SYMBOL(tlv320aic23_power_down);
