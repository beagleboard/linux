# SPDX-License-Identifier: GPL-2.0

ifeq ($(CONFIG_OF_OVERLAY),y)
DTC_FLAGS += -@
endif

subdir-y += sifive
subdir-y += kendryte
subdir-y += thead

obj-$(CONFIG_BUILTIN_DTB) := $(addsuffix /, $(subdir-y))
