# Makefile for OMAP1/2 camera driver

obj-$(CONFIG_VIDEO_OMAP_CAMERA) += omapcamera.o

objs-y$(CONFIG_ARCH_OMAP16XX) += omap16xxcam.o camera_core.o

omapcamera-objs := $(objs-yy)

EXTRA_CFLAGS = -I$(src)/..
