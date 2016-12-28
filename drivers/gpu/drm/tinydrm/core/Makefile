ccflags-y += -I$(src)/../include

tinydrm-y := tinydrm-core.o tinydrm-pipe.o tinydrm-fb.o tinydrm-helpers.o
tinydrm-$(CONFIG_REGMAP) += tinydrm-regmap.o
tinydrm-$(CONFIG_DEBUG_FS) += tinydrm-debugfs.o

obj-$(CONFIG_DRM_TINYDRM) += tinydrm.o
