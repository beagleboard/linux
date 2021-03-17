
ccflags-y += -I$(srctree)/drivers/xenomai/analogy

obj-$(CONFIG_XENO_DRIVERS_ANALOGY) += xeno_analogy.o testing/ intel/ national_instruments/ sensoray/

xeno_analogy-y := \
	buffer.o \
	command.o \
	device.o \
	driver.o \
	driver_facilities.o \
	instruction.o \
	rtdm_helpers.o \
	subdevice.o \
	transfer.o \
	rtdm_interface.o
