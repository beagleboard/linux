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

#VXD
vxd-dec-y += vxd_core.o

#PVDEC
vxd-dec-y += vxd_pvdec.o

#MEM_MGR
vxd-dec-y += ../common/img_mem_man.o ../common/img_mem_unified.o
vxd-dec-y += ../common/imgmmu.o

#Utilities
vxd-dec-y += ../common/lst.o ../common/dq.o
vxd-dec-y += ../common/resource.o
vxd-dec-y += dec_resources.o
vxd-dec-y += ../common/rman_api.o
vxd-dec-y += pixel_api.o
vxd-dec-y += vdecdd_utils_buf.o
vxd-dec-y += vdecdd_utils.o

#MMU
vxd-dec-y += ../common/talmmu_api.o
vxd-dec-y += ../common/pool.o
vxd-dec-y += ../common/hash.o
vxd-dec-y += ../common/ra.o
vxd-dec-y += ../common/addr_alloc.o
vxd-dec-y += ../common/work_queue.o
vxd-dec-y += vdec_mmu_wrapper.o

#DECODER
vxd-dec-y += ../common/pool_api.o ../common/idgen_api.o
vxd-dec-y += hw_control.o
vxd-dec-y += vxd_int.o
vxd-dec-y += translation_api.o
vxd-dec-y += decoder.o
vxd-dec-y += core.o

#BSPP
vxd-dec-y += swsr.o
vxd-dec-y += h264_secure_parser.o
vxd-dec-y += bspp.o

#UM INTERFACE & SYSDEV
vxd-dec-y += vxd_dec.o

vxd-dec-y += vxd_v4l2.o

ifeq ($(DEBUG_DECODER_DRIVER), y)
ccflags-y   += -DDEBUG_DECODER_DRIVER
ccflags-y   += -DDEBUG
endif

ifeq ($(HAS_HEVC),y)
ccflags-y   += -DHAS_HEVC
vxd-dec-y += hevc_secure_parser.o
endif

ifeq ($(HAS_JPEG),y)
ccflags-y   += -DHAS_JPEG
vxd-dec-y += jpeg_secure_parser.o
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

ccflags-y += -I$(srctree)/drivers/media/platform/vxe-vxd/common
