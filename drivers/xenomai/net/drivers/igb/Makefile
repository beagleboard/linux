ccflags-y += -Idrivers/xenomai/net/stack/include

obj-$(CONFIG_XENO_DRIVERS_NET_DRV_IGB) += rt_igb.o

rt_igb-y :=  					\
	e1000_82575.o				\
	e1000_i210.o				\
	e1000_mac.o				\
	e1000_mbx.o				\
	e1000_nvm.o				\
	e1000_phy.o				\
	igb_hwmon.o				\
	igb_main.o
