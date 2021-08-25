
obj-$(CONFIG_XENOMAI) += xenomai.o

xenomai-y :=	core.o		\
		device.o	\
		drvlib.o	\
		fd.o		\
		wrappers.o

ccflags-y += -I$(srctree)/$(src)/.. -I$(srctree)/kernel
