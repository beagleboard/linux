img_basedir := drivers/gpu/drm/img-rogue
include $(img_basedir)/config_kernel.mk

obj-$(CONFIG_DRM_POWERVR_ROGUE) += pvrsrvkm.o

ccflags-y += \
 -include config_kernel.h \
 -Iinclude/drm \
 -I$(img_basedir) \
 -I$(img_basedir)/apollo \
 -I$(img_basedir)/km \
 -I$(img_basedir)/system \
 -D__linux__

include $(img_basedir)/pvrsrvkm.mk

obj-$(CONFIG_DRM_POWERVR_PDP) += pdp/

obj-$(CONFIG_DRM_POWERVR_APOLLO) += apollo/
