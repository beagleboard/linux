#
#  Makefile for Gasket framework and dependent drivers.
#

obj-m += gasket.o
obj-m += apex.o

gasket-objs	:= gasket_core.o gasket_ioctl.o gasket_interrupt.o gasket_page_table.o gasket_sysfs.o
apex-objs	:= apex_driver.o

KVERSION := $(shell uname -r)

all:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) modules

clean:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
