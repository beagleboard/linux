# SPDX-License-Identifier: GPL-2.0

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

#UM INTERFACE & SYSDEV
vxe-enc-y += vxe_v4l2.o
vxe-enc-y += vxe_enc.o

#TOPAZ Device
vxe-enc-y += topaz_device.o

#MEM_MGR
vxe-enc-y += ../common/img_mem_man.o ../common/img_mem_unified.o
vxe-enc-y += ../common/talmmu_api.o
vxe-enc-y += ../common/addr_alloc.o
vxe-enc-y += ../common/lst.o
vxe-enc-y += ../common/hash.o
vxe-enc-y += ../common/ra.o
vxe-enc-y += ../common/pool.o
vxe-enc-y += ../common/rman_api.o
vxe-enc-y += ../common/dq.o
vxe-enc-y += ../common/idgen_api.o
vxe-enc-y += topazmmu.o
vxe-enc-y += ../common/imgmmu.o
vxe-enc-y += ../common/work_queue.o

#Topaz framework and firmware prep
vxe-enc-y += topaz_api.o
vxe-enc-y += topaz_api_utils.o
vxe-enc-y += header_gen.o

#FWIF
vxe-enc-y += mtx_fwif.o

obj-$(CONFIG_VIDEO_IMG_VXE_ENC) += vxe-enc.o

ifeq ($(DEBUG_ENCODER_DRIVER), y)
ccflags-y   += -DDEBUG_ENCODER_DRIVER
ccflags-y   += -DDEBUG
endif

ifeq ($(ENABLE_PROFILING),y)
ccflags-y	+= -DENABLE_PROFILING
endif

ccflags-y += -I$(srctree)/drivers/media/platform/vxe-vxd/common/
