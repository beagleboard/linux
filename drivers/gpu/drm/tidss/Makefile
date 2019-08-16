# SPDX-License-Identifier: GPL-2.0

tidss-y := tidss_crtc.o \
	tidss_drv.o \
	tidss_encoder.o \
	tidss_irq.o \
	tidss_kms.o \
	tidss_plane.o \
	tidss_scale_coefs.o \
	tidss_v_connector.o \
	tidss_v_encoder.o \
	tidss_v_plane.o \
	tidss_v_crtc.o

tidss-$(CONFIG_DRM_TIDSS_DSS6) += tidss_dispc6.o
tidss-$(CONFIG_DRM_TIDSS_DSS7) += tidss_dispc7.o

tidss-$(CONFIG_DRM_TIDSS_WB) += tidss_wb.o tidss_wb_m2m.o

obj-$(CONFIG_DRM_TIDSS) += tidss.o
