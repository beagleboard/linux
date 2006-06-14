# Makefile for OMAP1/2 camera driver

obj-$(CONFIG_VIDEO_OMAP_CAMERA) += omapcamera.o
obj-$(CONFIG_VIDEO_CAMERA_SENSOR_OV9640) += sensor_ov9640.o

objs-y$(CONFIG_ARCH_OMAP16XX) += omap16xxcam.o camera_core.o
objs-y$(CONFIG_MACH_OMAP_H3) += h3_sensor_power.o
objs-y$(CONFIG_MACH_OMAP_H4) += h4_sensor_power.o

omapcamera-objs := $(objs-yy)

EXTRA_CFLAGS = -I$(src)/..
