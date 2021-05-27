
ccflags-y += -I$(srctree)/drivers/xenomai/can -I$(srctree)/drivers/xenomai/can/mscan

obj-$(CONFIG_XENO_DRIVERS_CAN_MSCAN) += xeno_can_mscan.o

xeno_can_mscan-y := rtcan_mscan.o rtcan_mscan_proc.o rtcan_mscan_mpc5xxx.o
