#
# Makefile for the OMAP DSP driver.
#

# The target object and module list name.

obj-y := dsp_common.o

obj-$(CONFIG_OMAP_DSP) += dsp.o

# Declare multi-part drivers

dsp-objs	:= dsp_core.o ipbuf.o mblog.o task.o \
		   dsp_ctl_core.o dsp_ctl.o taskwatch.o error.o dsp_mem.o \
		   uaccess_dsp.o
