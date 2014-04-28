#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <asm/io.h>
#include <linux/gpio.h>


//SSI
#define SSI_CLK 110
#define SSI_DATA 112
#define SSI_DONE 3
#define SSI_PROG 5
#define SSI_INIT 2
#define MODE0	0
#define MODE1 1
#define SSI_DELAY 1

//GPIO
#define GPIO3_BASE 0x481AE000
#define GPIO3_SETDATAOUT *(gpio_regs+1)
#define GPIO3_CLEARDATAOUT *(gpio_regs)

//I2C
#define I2C_IO_EXP_CONFIG_REG	0x03
#define I2C_IO_EXP_IN_REG	0x00
#define I2C_IO_EXP_OUT_REG	0x01


volatile unsigned * gpio_regs;


static inline void __delay_cycles(unsigned long cycles)
{
	while (cycles != 0) {
		cycles--;
	}
}

static inline void ssiSetClk(void)
{
	//gpio_set_value(SSI_CLK, 1);
	GPIO3_SETDATAOUT = (1 << 14);
}

static inline void ssiClearClk(void)
{
	//gpio_set_value(SSI_CLK, 0);
	GPIO3_CLEARDATAOUT = (1 << 14);
}

static inline void ssiSetData(void)
{
	//gpio_set_value(SSI_DATA, 1);
	GPIO3_SETDATAOUT = (1 << 16);
}

static inline void ssiClearData(void)
{
	//gpio_set_value(SSI_DATA, 0);
	GPIO3_CLEARDATAOUT = (1 << 16);
}

static inline void serialConfigWriteByte(unsigned char val)
{
	unsigned char bitCount = 0;
	unsigned char valBuf = val;

	for (bitCount = 0; bitCount < 8; bitCount++) {
		ssiClearClk();

		if ((valBuf & 0x80) != 0) {
			ssiSetData();
		} else {
			ssiClearData();
		}

		//__delay_cycles(SSI_DELAY);
		ssiSetClk();
		valBuf = (valBuf << 1);
		//__delay_cycles(SSI_DELAY);
	}
}

static inline void i2c_set_pin(struct i2c_client * io_cli, unsigned char pin, unsigned char val)
{
	unsigned char i2c_buffer[2];

	i2c_buffer[0] = I2C_IO_EXP_OUT_REG;
	i2c_master_send(io_cli, i2c_buffer, 1);
	i2c_master_recv(io_cli, &i2c_buffer[1], 1);

	if (val == 1) {
		i2c_buffer[1] |= (1 << pin);
	} else {
		i2c_buffer[1] &= ~(1 << pin);
	}

	i2c_master_send(io_cli, i2c_buffer, 2);
}

static inline unsigned char i2c_get_pin(struct i2c_client * io_cli, unsigned char pin)
{
	unsigned char i2c_buffer;

	i2c_buffer = I2C_IO_EXP_IN_REG;
	i2c_master_send(io_cli, &i2c_buffer, 1);
	i2c_master_recv(io_cli, &i2c_buffer, 1);
	//printk("reading value %x \n", i2c_buffer);

	return ((i2c_buffer >> pin) & 0x01);
}

int loadBitFile(struct i2c_client * io_cli, const unsigned char * bitBuffer_user, const unsigned int length)
{
	int iCfg;
	unsigned long int i;
	unsigned long int timer = 0;
	unsigned char * bitBuffer;
	unsigned char i2c_buffer[4];

	//request_mem_region(GPIO3_BASE + 0x190, 8, gDrvrName);
	gpio_regs = ioremap_nocache(GPIO3_BASE + 0x190, 2 * sizeof(int));

	bitBuffer = kmalloc(length, GFP_KERNEL);

	if (bitBuffer == NULL || copy_from_user(bitBuffer, bitBuffer_user, length)) {
		printk("Failed allocate buffer for configuration file \n");

		return -ENOTTY;
	}

	iCfg = gpio_request(SSI_CLK, "ssi_clk");

	if (iCfg < 0) {
		printk("Failed to take control over ssi_clk pin \n");

		return -ENOTTY;
	}

	iCfg = gpio_request(SSI_DATA, "ssi_data");

	if (iCfg < 0) {
		printk("Failed to take control over ssi_data pin \n");

		return -ENOTTY;
	}

	i2c_buffer[0] = I2C_IO_EXP_CONFIG_REG;
	i2c_buffer[1] = 0xFF;
	i2c_buffer[1] &= ~((1 << SSI_PROG) | (1 << MODE1) | (1 << MODE0));
	i2c_master_send(io_cli, i2c_buffer, 2); // set SSI_PROG, MODE0, MODE1 as output others as inputs
	i2c_set_pin(io_cli, MODE0, 1);
	i2c_set_pin(io_cli, MODE1, 1);
	i2c_set_pin(io_cli, SSI_PROG, 0);

	gpio_direction_output(SSI_CLK, 0);
	gpio_direction_output(SSI_DATA, 0);

	gpio_set_value(SSI_CLK, 0);
	i2c_set_pin(io_cli, SSI_PROG, 1);
	__delay_cycles(10 * SSI_DELAY);
	i2c_set_pin(io_cli, SSI_PROG, 0);
	__delay_cycles(5 * SSI_DELAY);

	while (i2c_get_pin(io_cli, SSI_INIT) > 0 && timer < 200)
		timer++; // waiting for init pin to go down

	if (timer >= 200) {
		printk("FPGA did not answer to prog request, init pin not going low \n");
		i2c_set_pin(io_cli, SSI_PROG, 1);
		gpio_free(SSI_CLK);
		gpio_free(SSI_DATA);

		return -ENOTTY;
	}

	timer = 0;
	__delay_cycles(5 * SSI_DELAY);
	i2c_set_pin(io_cli, SSI_PROG, 1);

	while (i2c_get_pin(io_cli, SSI_INIT) == 0 && timer < 256) { // need to find a better way ...
		timer++; // waiting for init pin to go up
	}

	if (timer >= 256) {
		printk("FPGA did not answer to prog request, init pin not going high \n");
		gpio_free(SSI_CLK);
		gpio_free(SSI_DATA);

		return -ENOTTY;
	}

	timer = 0;
	printk("Starting configuration of %d bits \n", length * 8);

	for (i = 0; i < length; i++) {
		serialConfigWriteByte(bitBuffer[i]);
		schedule();
	}

	printk("Waiting for done pin to go high \n");

	while (timer < 50) {
		ssiClearClk();
		__delay_cycles(SSI_DELAY);
		ssiSetClk();
		__delay_cycles(SSI_DELAY);
		timer++;
	}

	gpio_set_value(SSI_CLK, 0);
	gpio_set_value(SSI_DATA, 1);

	if (i2c_get_pin(io_cli, SSI_DONE) == 0) {
		printk("FPGA prog failed, done pin not going high \n");
		gpio_free(SSI_CLK);
		gpio_free(SSI_DATA);

		return -ENOTTY;
	}

	i2c_buffer[0] = I2C_IO_EXP_CONFIG_REG;
	i2c_buffer[1] = 0xDC;
	i2c_master_send(io_cli, i2c_buffer, 2); // set all unused config pins as input (keeping mode pins and PROG as output)
	gpio_direction_input(SSI_CLK);
	gpio_direction_input(SSI_DATA);
	gpio_free(SSI_CLK);
	gpio_free(SSI_DATA);
	iounmap(gpio_regs);
	//release_mem_region(GPIO3_BASE + 0x190, 8);
	kfree(bitBuffer);

	return length;
}
