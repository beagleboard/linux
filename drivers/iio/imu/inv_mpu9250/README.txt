Kernel driver inv-mpu-iio
Author: Invensense <http://invensense.com>

Description
-----------
This document describes how to install the Invensense device driver into a
Linux kernel. At the moment, this driver supports the ITG3500/MPU6050/MPU9150/MPU3050. The slave
address of these four chips are 0x68. However, the actual slave address depends on the board
configuration. The driver does not assume anything about it.

Files included in this package:
Kconfig
Makefile
inv_mpu_core.c
inv_mpu_misc.c
inv_mpu_trigger.c
inv_mpu3050_iio.c
inv_mpu_iio.h
inv_mpu_ring.c
inv_slave_bma250.c
dmpDefaultMPU6050.c
dmpkey.h
dmpmap.h
mpu.h
Including the driver in the Linux kernel
----------------------------------------
mpu.h should be added to "kernel/include/linux".
Other files listed should be added to the drivers/staging/iio/imu/mpu directory (or another
directory of your choosing). When building the kernel, the driver will not
appear in menuconfig without modifications similar to those below:

modify "drivers/staging/iio/imu/Kconfig" like
source "drivers/staging/iio/imu/mpu/Kconfig"

modify "drivers/staging/iio/imu/Makefile"
obj-y += mpu/

Board and Platform Data
-----------------------
The board file needs to be modified to register the device on an I2C bus. An
i2c_board_info instance must be defined as seen below. The hardcoded value of
140 corresponds to the GPIO input pin wired to the device's interrupt pin.
This pin will most likely be different for your platform.
platform data is for orientation matrix,  and secondary bus situations.
For MPU9150, it regarded as a MPU9150 and AKM8975 in the secondary.
So the secondary i2c address must be filled.
-----------------------------------------------------------------
The board file is arch/arm/mach-omap2/board-omap4panda.c or
modify the board file in your system as below:
--------------------------------------------------------
For AKM8963 in the secondary i2c bus of MPU6050,
static struct mpu_platform_data gyro_platform_data = {
	.int_config  = 0x00,
	.level_shifter = 0,
	.orientation = {  -1,  0,  0,
			   0,  1,  0,
			   0,  0, -1 },
	.sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS,
	.sec_slave_id   = COMPASS_ID_AK8963,
	.secondary_i2c_addr = 0x0E
};
-----------------------------------------------------------
For MPU9150, the secondary i2c bus address must be filled as below.
static struct mpu_platform_data gyro_platform_data = {
	.int_config  = 0x00,
	.level_shifter = 0,
	.orientation = {  -1,  0,  0,
			   0,  1,  0,
			   0,  0, -1 },
	.sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS,
	.sec_slave_id   = COMPASS_ID_AK8975,
	.secondary_i2c_addr = 0x0E
};
-----------------------------------------------------------
for BMA250 in the secondary, please use the platform data as:
static struct mpu_platform_data gyro_platform_data = {
	.int_config  = 0x00,
	.level_shifter = 0,
	.orientation = {  -1,  0,  0,
			   0,  1,  0,
			   0,  0, -1 },
	.sec_slave_type = SECONDARY_SLAVE_TYPE_ACCEL,
	.sec_slave_id   = ACCEL_ID_BMA250,
	.secondary_i2c_addr = 0x18,
};
---------------------------------------------------------------
the i2c init data is:
----------------------------------------------------------------
For MPU3050,
static struct i2c_board_info __initdata single_chip_board_info[] = {
	{
		I2C_BOARD_INFO("mpu3050", 0x68),
		.irq = (IH_GPIO_BASE + MPUIRQ_GPIO),
		.platform_data = &gyro_platform_data,
	},
};
----------------------------------------------------------------
for ITG3500:
static struct i2c_board_info __initdata single_chip_board_info[] = {
	{
		I2C_BOARD_INFO("itg3500", 0x68),
		.irq = (IH_GPIO_BASE + MPUIRQ_GPIO),
                .platform_data = &gyro_platform_data,
	},
};
for MPU6050
static struct i2c_board_info __initdata single_chip_board_info[] = {
	{
		I2C_BOARD_INFO("mpu6050", 0x68),
		.irq = (IH_GPIO_BASE + MPUIRQ_GPIO),
		.platform_data = &gyro_platform_data,
	},
};
for MPU9150
arch/arm/mach-omap2/board-omap4panda.c
static struct i2c_board_info __initdata single_chip_board_info[] = {
	{
		I2C_BOARD_INFO("mpu9150", 0x68),
		.irq = (IH_GPIO_BASE + MPUIRQ_GPIO),
		.platform_data = &gyro_platform_data,
	},
};

In the _i2c_init function, the device is registered in the following manner:

arch/arm/mach-omap2/board-omap4panda.c
    in static int __init omap4_panda_i2c_init(void)
omap_register_i2c_bus(4, 400, single_chip_board_info, ARRAY_SIZE(single_chip_board_info));

IIO subsystem
----------------------------------------------
successful installation will create two directories under /sys/bus/iio/devices
iio:device0
trigger0
Under /dev/ diretory, a file "iio:device0" will also be created(or iio:deviceX, if
you have more than one iio devices).
Communicating with the driver in userspace
------------------------------------------
Upon installation, the driver generates several files in sysfs. If your
platform is configured as detailed above, navigate to the following path to
find these files:
/sys/bus/iio/devices/iio:device0

The list below provides a brief description for each file.
--------------------------------------
For ITG3500:
temperature (Read-only)
Read temperature data directly from the temperature register.

sampling_frequency (Read/write)
Configure the ADC sampling rate and FIFO output rate.

sampling_frequency_available(read-only)
show commonly used frequency

clock_source (Read-only)
Check which clock-source is used by the chip.

power_state (Read/write)
turn on/off the power supply

self_test (read-only)
read this entry trigger self test. The return value is D.
D is the success/fail.
For different chip, the result is different for success/fail.
1 means success 0 means fail. The LSB of D is for gyro; the bit
next to LSB of D is for accel. The bit 2 of D is for compass result.

key (read-only)
show the key value of this driver. Used by MPL.

gyro_matrix (read-only)
show the orient matrix obtained from board file.

-------------------------------------------------------------
For MPU6050:
MPU6050 has all the sysfs files that ITG3500 has. It has additional files list below:

gyro_enable (read/write)
enable/disable gyro functionality. affect raw_gyro. turn off this will shut down gyro and save power.

accl_enable (read/write)
enable/disable accelerometer functionality. affect raw_accl. turn off this will shut down accel and save power.

firmware_loaded (read/write)
Flag indicate the whether firmware is loaded or not in the DMP engine. 0 means no firmware loaded.
1 means firmware is already loaded . This flag can only be written as 0. 1 is updated
internally.

dmp_on(read/write)
This entry controls whether to run DMP or not. To enable DMP , firmware_loaded must be 1. write 1 to enable
DMP and write 0 to disable dmp. 

dmp_in_on(read/write)
This entry controls whether dmp interrupt is on/off. firmware_loaded must be 1. sometimes, it is desirable
that interrupt is off while DMP is running.

dmp_firmware (write only binary file)
This is the entry that firmware code is loaded into. If the action is succeful, firmware_loaded will
be updated as 1. In order to load new firmware, firmware_loaded flag should be set 0.

lpa_mode(read-write)
Low power  accelerometer mode
lpa_freq(read-write)
low power acceleromter frequency.

accel_matrix
orient matrix for accel

flick_lower,
flick_upper,
flick_counter,
flick_message_on,
flick_int_on,
flick_axis,
Flick related entry

pedometer_time
pedometer_steps,
Pedometer related entry

event_flick
event_tap
event_orientation
event_display_orientation
event related entry

tap_on
control tap function of DMP

tap_time
tap_min_count
tap_threshold
tap related entries. control various parameters of tap function.

orientation_on
turn on/off orientation function of DMP.

display_orientation_on
turn on/off display orientation function of DMP.

quaternion_on
turn on/off quaterniion data output. must use DMP.
-------------------------------------------------------------------
for MPU9150 and secondary compass
MPU9150 has every entry MPU6050 has. It has additional entries:

compass_enable (read/write)
enable this will enable compass function. 

compass_matrix (read-only)
compass orient matrix
---------------------
for MPU3050 and secondary accelerometer(only BMA250 is supported right now)
It has every entry ITG3500 has and has two addiontal entries.
accl_matrix
accl_enable
----------------------------------------------------------------------------------
low power accelerometer mode
Lower power accelerometer mode is a special mode. It works only for accelerometer.
It has two entries, lpa_mode and lpa_freq. Only MPU6050 and MPU9150 has this mode.
To run low power accel mode, set lpa_mode to 1, set lpa_freq to 0~3, which corresponds
to 1.25Hz, 5Hz, 20Hz, 40Hz. "gyro_enable" and "compass_enable" must be zero. "dmp_on"
must be zero.
-----------------------------------------------------------------------------------
dmp event.
dmp event is event out by the DMP unit inside MPU. Only MPU6050 and MPU9150 supports this.
There are four sysfs entreis, event_flick, event_tap and event_orientation and
event_display_orientation. These four events must
be polled before read. The proper method to poll sysfs is:
1. open file.
2. dummy read.
3. poll.
4. once the poll passed, use fopen and fread to read the sysfs entry.
5. interpret the data.
------------------------------------------------------------------------------
If streaming to a userspace application, the recommended way to access gyro/accel/compass
data is via /dev/iio:device0. Follow these steps to get constant readings from
the driver:

1. Write a 1 to power_state to turn on the chip. This is the default setting
   after installing the driver.
2. Write the desired output rate to fifo_rate.
3. write 1 to enable to turn on the event.
4. Read /dev/iio:device0 to get a string of gyro/accel/compass data.
5. Parse this string to obtain each gyro/accel/compass element.
6. If dmp firmware code is loaded, using "dmp_on" to enable/disable dmp .
7. If compass is enabled, output will have compass data.
===========================================================================
                    Recommended sysfs entry setup senquence
1. without DMP firmware
1.1 set "power_state" to 1,
1.2 change scale and fifo rate value to your need.
1.3 change gyro_enable and accle_enable and compass_enable to your needs. For example,
if you want gyro only, set accl_enable to 0 or set accl_enable to zero and compass_enable to zero.
If you want accel only, set gyro_enable to 0 or set gyro_enable to zero and compass_enable to zero.
If you want compass only, disable gyro and accel.
1.4 set "enable" to 1. you will get output you want.

2. With DMP firmware
2.1 set "power_state" to 1,
2.2 write "0" to firmware_loaded if it is not zero already.
2.3 load firmware into "dmp_firmware" as a whole. Don't split the DMP firmware image.
2.4 make sure firmware_loaded is 1 after loading.
2.5 make other configurations similar to the situation as without DMP firmware.
2.6 set dmp_on to 1.
2.7 set "enable" to 1.
=======================================================
The enable function is using enable entry under "/sys/bus/iio/devices/iio:device0/buffer"
==========================================================
test applications:
Test application is under ARTHROPOD/trunk/software/simple_apps/mpu_iio
------------------------------------------
To run with MPU9150/MPU6050:
using the following command:
for orientation/tap/flick/display orientation event:
mpu_iio  -c 10 -l 3 -p 
for normal data print
mpu_iio  -c 10 -l 3
----------------------------------------
To run with MPU3050/ITG3500:
mpu_iio -c 10 -l 3 -r
-----------------------------------------
Please use mpu_iio.c and iio_utils.h as the sample code for your development.
