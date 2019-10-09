obj-$(CONFIG_XENOMAI) += xenomai.o rtdm/ posix/

xenomai-y :=	apc.o		\
		arith.o 	\
		bufd.o		\
		clock.o		\
		heap.o		\
		init.o		\
		intr.o		\
		lock.o		\
		registry.o	\
		sched-idle.o	\
		sched-rt.o	\
		sched.o		\
		select.o	\
		synch.o		\
		thread.o	\
		timer.o		\
		tree.o

xenomai-$(CONFIG_XENO_OPT_SCHED_QUOTA) += sched-quota.o
xenomai-$(CONFIG_XENO_OPT_SCHED_WEAK) += sched-weak.o
xenomai-$(CONFIG_XENO_OPT_SCHED_SPORADIC) += sched-sporadic.o
xenomai-$(CONFIG_XENO_OPT_SCHED_TP) += sched-tp.o
xenomai-$(CONFIG_XENO_OPT_DEBUG) += debug.o
xenomai-$(CONFIG_XENO_OPT_PIPE) += pipe.o
xenomai-$(CONFIG_XENO_OPT_MAP) += map.o
xenomai-$(CONFIG_PROC_FS) += vfile.o procfs.o
