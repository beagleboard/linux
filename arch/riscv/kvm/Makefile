# SPDX-License-Identifier: GPL-2.0
# Makefile for RISC-V KVM support
#

common-objs-y = $(addprefix ../../../virt/kvm/, kvm_main.o coalesced_mmio.o)
common-objs-y += $(addprefix ../../../virt/kvm/, eventfd.o)

ccflags-y := -Ivirt/kvm -Iarch/riscv/kvm

kvm-objs := $(common-objs-y)

kvm-objs += main.o vm.o mmu.o
kvm-objs += vcpu.o vcpu_exit.o

obj-$(CONFIG_KVM)	+= kvm.o
