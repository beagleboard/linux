# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_RPMSG)		+= rpmsg_core.o
obj-$(CONFIG_RPMSG_CHAR)	+= rpmsg_char.o
obj-$(CONFIG_RPMSG_PRU)		+= rpmsg_pru.o
obj-$(CONFIG_RPMSG_QCOM_GLINK_RPM) += qcom_glink_rpm.o
obj-$(CONFIG_RPMSG_QCOM_GLINK_NATIVE) += qcom_glink_native.o
obj-$(CONFIG_RPMSG_QCOM_GLINK_SMEM) += qcom_glink_smem.o
obj-$(CONFIG_RPMSG_QCOM_SMD)	+= qcom_smd.o
obj-$(CONFIG_RPMSG_VIRTIO)	+= virtio_rpmsg_bus.o

obj-$(CONFIG_RPMSG_RPC)		+= rpmsg-rpc.o
rpmsg-rpc-y			:= rpmsg_rpc.o rpmsg_rpc_sysfs.o rpmsg_rpc_dmabuf.o
