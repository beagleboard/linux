# SPDX-License-Identifier: GPL-2.0

tidss-y := tidss_drv.o \
	tidss_kms.o \
	tidss_crtc.o \
	tidss_plane.o \
	tidss_encoder.o \
	tidss_irq.o \
	tidss_scale_coefs.o

tidss-$(CONFIG_DRM_TIDSS_DSS6) += tidss_dispc6.o
tidss-$(CONFIG_DRM_TIDSS_DSS7) += tidss_dispc7.o

obj-$(CONFIG_DRM_TIDSS) += tidss.o
