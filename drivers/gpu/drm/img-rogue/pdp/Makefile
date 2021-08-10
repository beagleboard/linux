img_basedir := drivers/gpu/drm/img-rogue

obj-$(CONFIG_DRM_POWERVR_PDP) += drm_pdp.o

ccflags-y += \
 -include config_kernel.h \
 -I$(img_basedir) \
 -I$(img_basedir)/apollo

include $(img_basedir)/pdp/drm_pdp.mk
