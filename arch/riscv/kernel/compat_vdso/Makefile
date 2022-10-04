# SPDX-License-Identifier: GPL-2.0-only

# Absolute relocation type $(ARCH_REL_TYPE_ABS) needs to be defined before
# the inclusion of generic Makefile.
ARCH_REL_TYPE_ABS := R_RISCV_32|R_RISCV_64|R_RISCV_JUMP_SLOT
include $(srctree)/lib/vdso/Makefile
# Symbols present in the compat_vdso
compat_vdso-syms  = rt_sigreturn
compat_vdso-syms += getcpu
compat_vdso-syms += flush_icache

# Files to link into the compat_vdso
obj-compat_vdso = $(patsubst %, %.o, $(compat_vdso-syms)) note.o

ccflags-y := -fno-stack-protector

# Build rules
targets := $(obj-compat_vdso) compat_vdso.so compat_vdso.so.dbg compat_vdso.lds
obj-compat_vdso := $(addprefix $(obj)/, $(obj-compat_vdso))

obj-y += compat_vdso.o
CPPFLAGS_compat_vdso.lds += -P -C -U$(ARCH)

# Disable profiling and instrumentation for VDSO code
GCOV_PROFILE := n
KCOV_INSTRUMENT := n

# Force dependency
$(obj)/compat_vdso.o: $(obj)/compat_vdso.so

# link rule for the .so file, .lds has to be first
$(obj)/compat_vdso.so.dbg: $(obj)/compat_vdso.lds $(obj-compat_vdso) FORCE
	$(call if_changed,compat_vdsold)
LDFLAGS_compat_vdso.so.dbg = -shared -S -soname=linux-compat_vdso.so.1 \
	--build-id=sha1 --hash-style=both --eh-frame-hdr

# strip rule for the .so file
$(obj)/%.so: OBJCOPYFLAGS := -S
$(obj)/%.so: $(obj)/%.so.dbg FORCE
	$(call if_changed,objcopy)

# Generate VDSO offsets using helper script
gen-compat_vdsosym := $(srctree)/$(src)/gen_compat_vdso_offsets.sh
quiet_cmd_compat_vdsosym = VDSOSYM $@
	cmd_compat_vdsosym = $(NM) $< | $(gen-compat_vdsosym) | LC_ALL=C sort > $@

include/generated/compat_vdso-offsets.h: $(obj)/compat_vdso.so.dbg FORCE
	$(call if_changed,compat_vdsosym)

# actual build commands
# The DSO images are built using a special linker script
# Make sure only to export the intended __compat_vdso_xxx symbol offsets.
quiet_cmd_compat_vdsold = VDSOLD  $@
      cmd_compat_vdsold = $(LD) $(ld_flags) -T $(filter-out FORCE,$^) -o $@.tmp && \
                   $(OBJCOPY) $(patsubst %, -G __compat_vdso_%, $(compat_vdso-syms)) $@.tmp $@ && \
                   rm $@.tmp

# install commands for the unstripped file
quiet_cmd_compat_vdso_install = INSTALL $@
      cmd_compat_vdso_install = cp $(obj)/$@.dbg $(MODLIB)/compat_vdso/$@

compat_vdso.so: $(obj)/compat_vdso.so.dbg
	@mkdir -p $(MODLIB)/compat_vdso
	$(call cmd,compat_vdso_install)

compat_vdso_install: compat_vdso.so
