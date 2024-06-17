# SPDX-License-Identifier: GPL-2.0

# Optional Video feature configuration control

# (1)
# This config allows enabling or disabling of HEVC/H265 video
# decoding functionality with IMG VXD Video decoder. If you
# do not want HEVC decode capability, select N.
# If unsure, select Y
HAS_HEVC ?=y

# (2)
# This config enables error concealment with gray pattern.
# Disable if you do not want error concealment capability.
# If unsure, say Y
ERROR_CONCEALMENT ?=y

# (3)
# This config, if enabled, configures H264 video decoder to
# output frames in the decode order with no buffering and
# picture reordering inside codec.
# If unsure, say N
REDUCED_DPB_NO_PIC_REORDERING ?=n

# (4)
# This config, if enabled, enables all the debug traces in
# decoder driver. Enable it only for debug purpose
# Keep it always disabled for release codebase
DEBUG_DECODER_DRIVER ?=n

# (5)
# This config allows enabling or disabling of MJPEG video
# decoding functionality with IMG VXD Video decoder. If you
# do not want MJPEG decode capability, select N.
# If unsure, select Y
HAS_JPEG ?=y

# (6)
# This config allows simulation of Error recovery.
# This config is only for testing, never enable it for release build.
ERROR_RECOVERY_SIMULATION ?=n

# (7)
# This config enables allocation of capture buffers from
# dma contiguous memory.
# If unsure, say Y
CAPTURE_CONTIG_ALLOC ?=y

vxd-dec-y += common/img_mem_man.o \
	common/img_mem_unified.o \
	common/imgmmu.o \
	common/pool_api.o \
	common/idgen_api.o \
	common/talmmu_api.o \
	common/pool.o \
	common/hash.o \
	common/ra.o \
	common/addr_alloc.o \
	common/work_queue.o \
	common/lst.o \
	common/dq.o \
	common/resource.o \
	common/rman_api.o \

vxd-dec-y += decoder/vxd_core.o \
	decoder/vxd_pvdec.o \
	decoder/dec_resources.o \
	decoder/pixel_api.o \
	decoder/vdecdd_utils_buf.o \
	decoder/vdecdd_utils.o \
	decoder/vdec_mmu_wrapper.o \
	decoder/hw_control.o \
	decoder/vxd_int.o \
	decoder/translation_api.o \
	decoder/decoder.o \
	decoder/core.o \
	decoder/swsr.o \
	decoder/h264_secure_parser.o \
	decoder/bspp.o \
	decoder/vxd_dec.o \
	decoder/vxd_v4l2.o \


ifeq ($(HAS_HEVC),y)
ccflags-y   += -DHAS_HEVC
vxd-dec-y += decoder/hevc_secure_parser.o
endif

ifeq ($(HAS_JPEG),y)
ccflags-y   += -DHAS_JPEG
vxd-dec-y += decoder/jpeg_secure_parser.o
endif

ifeq ($(DEBUG_DECODER_DRIVER), y)
ccflags-y   += -DDEBUG_DECODER_DRIVER
ccflags-y   += -DDEBUG
endif

ifeq ($(ERROR_CONCEALMENT),y)
ccflags-y   += -DERROR_CONCEALMENT
endif

ifeq ($(REDUCED_DPB_NO_PIC_REORDERING),y)
ccflags-y   += -DREDUCED_DPB_NO_PIC_REORDERING
endif

ifeq ($(ERROR_RECOVERY_SIMULATION),y)
ccflags-y   += -DERROR_RECOVERY_SIMULATION
endif

ifeq ($(CAPTURE_CONTIG_ALLOC),y)
ccflags-y += -DCAPTURE_CONTIG_ALLOC
endif

obj-$(CONFIG_VIDEO_IMG_VXD_DEC) += vxd-dec.o

# (1)
# This config, if enabled, enables all the debug traces in
# encoder driver. Enable it only for debug purpose
# Keep it always disabled for release codebase
DEBUG_ENCODER_DRIVER ?=n

# (3)
# This config enables encoder performance profiling
# keep it always disabled. Enable it only for profiling in development
# environments.
ENABLE_PROFILING ?=n

vxe-enc-y += common/img_mem_man.o \
	common/img_mem_unified.o \
	common/talmmu_api.o \
	common/addr_alloc.o \
	common/lst.o \
	common/hash.o \
	common/ra.o \
	common/pool.o \
	common/rman_api.o \
	common/dq.o \
	common/idgen_api.o \
	common/imgmmu.o \
	common/work_queue.o \

vxe-enc-y += encoder/vxe_v4l2.o \
	encoder/vxe_enc.o \
	encoder/topaz_device.o \
	encoder/topazmmu.o \
	encoder/topaz_api.o \
	encoder/topaz_api_utils.o \
	encoder/header_gen.o \
	encoder/mtx_fwif.o \

obj-$(CONFIG_VIDEO_IMG_VXE_ENC) += vxe-enc.o

ifeq ($(DEBUG_ENCODER_DRIVER), y)
ccflags-y   += -DDEBUG_ENCODER_DRIVER
ccflags-y   += -DDEBUG
endif

ifeq ($(ENABLE_PROFILING),y)
ccflags-y	+= -DENABLE_PROFILING
endif

ccflags-y += -I$(srctree)/drivers/media/platform/imagination/vxe-vxd/common/
