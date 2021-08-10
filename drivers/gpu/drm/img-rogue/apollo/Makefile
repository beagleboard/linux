img_basedir := drivers/gpu/drm/img-rogue

obj-$(CONFIG_DRM_POWERVR_APOLLO) += apollo.o

ccflags-y += \
 -include config_kernel.h \
 -I$(img_basedir)

include $(img_basedir)/apollo/apollo.mk
