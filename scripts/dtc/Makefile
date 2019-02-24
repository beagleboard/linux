# SPDX-License-Identifier: GPL-2.0
# scripts/dtc makefile

hostprogs-y	:= dtc fdtoverlay
always		:= $(hostprogs-y)

dtc-objs	:= dtc.o flattree.o fstree.o data.o livetree.o treesource.o \
		   srcpos.o checks.o util.o
dtc-objs	+= dtc-lexer.lex.o dtc-parser.tab.o

fdtoverlay-objs := fdtoverlay.o util.o
fdtoverlay-objs += $(subst $(srctree)/$(src)/,,$(patsubst %.c,%.o,$(wildcard $(srctree)/$(src)/libfdt/*.c)))

# Source files need to get at the userspace version of libfdt_env.h to compile
HOST_EXTRACFLAGS := -I$(src)/libfdt

# Generated files need one more search path to include headers in source tree
HOSTCFLAGS_dtc-lexer.lex.o := -I$(src)
HOSTCFLAGS_dtc-parser.tab.o := -I$(src)
HOSTCFLAGS_DTC := -I$(src) -I$(src)/libfdt

HOSTCFLAGS_checks.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_data.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_dtc.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_flattree.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_fstree.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_livetree.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_srcpos.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_treesource.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_util.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_fdtoverlay.o := $(HOSTCFLAGS_DTC)

HOSTCFLAGS_dtc-lexer.lex.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_dtc-parser.tab.o := $(HOSTCFLAGS_DTC)

HOSTCFLAGS += $(HOSTCFLAGS_DTC)

# dependencies on generated files need to be listed explicitly
$(obj)/dtc-lexer.lex.o: $(obj)/dtc-parser.tab.h
