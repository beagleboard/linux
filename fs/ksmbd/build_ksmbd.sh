# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2019 Samsung Electronics Co., Ltd.
#

#!/bin/sh

KERNEL_SRC=''
COMP_FLAGS=''

function is_module
{
	local ok=$(cat "$KERNEL_SRC"/.config | grep "CONFIG_SMB_SERVER=m")

	if [ "z$ok" == "z" ]; then
		echo "1"
		return 1
	fi

	echo "0"
	return 0
}

function patch_fs_config
{
	local ok=$(pwd |  grep -c "fs/smbd")
	if [ "z$ok" != "z1" ]; then
		echo "ERROR: please ``cd`` to fs/smbd"
		exit 1
	fi

	KERNEL_SRC=$(pwd | sed -e 's/fs\/smbd//')
	if [ ! -f "$KERNEL_SRC"/fs/Kconfig ]; then
		echo "ERROR: please ``cd`` to fs/smbd"
		exit 1
	fi

	ok=$(cat "$KERNEL_SRC"/fs/Makefile | grep smbd)
	if [ "z$ok" == "z" ]; then
		echo 'obj-$(CONFIG_SMB_SERVER)	+= smbd/' \
			>> "$KERNEL_SRC"/fs/Makefile
	fi

	ok=$(cat "$KERNEL_SRC"/fs/Kconfig | grep smbd)
	if [ "z$ok" == "z" ]; then
		ok=$(cat "$KERNEL_SRC"/fs/Kconfig \
			| sed -e 's/fs\/cifs\/Kconfig/fs\/cifs\/Kconfig\"\nsource \"fs\/smbd\/Kconfig/' \
			> "$KERNEL_SRC"/fs/Kconfig.new)
		if [ $? != 0 ]; then
			exit 1
		fi
		mv "$KERNEL_SRC"/fs/Kconfig.new "$KERNEL_SRC"/fs/Kconfig
	fi

	ok=$(cat "$KERNEL_SRC"/.config | grep "CONFIG_NETWORK_FILESYSTEMS=y")
	if [ "z$ok" == "z" ]; then
		ok=$(echo "CONFIG_NETWORK_FILESYSTEMS=y" \
			>> "$KERNEL_SRC"/.config)
		if [ $? != 0 ]; then
			exit 1
		fi
	fi

	ok=$(is_module)
	if [ "z$ok" == "z1" ]; then
		ok=$(echo "CONFIG_SMB_SERVER=m" >> "$KERNEL_SRC"/.config)
		if [ $? != 0 ]; then
			exit 1
		fi
		ok=$(echo "CONFIG_SMB_INSECURE_SERVER=y" \
			>> "$KERNEL_SRC"/.config)
		if [ $? != 0 ]; then
			exit 1
		fi
	fi
}

function ksmbd_module_make
{
	echo "Running smbd make"

	local c="make "$COMP_FLAGS" -C "$KERNEL_SRC" M="$KERNEL_SRC"/fs/smbd"

	rm smbd.ko

	cd "$KERNEL_SRC"
	echo $c
	$c
	cd "$KERNEL_SRC"/fs/smbd

	if [ $? != 0 ]; then
		exit 1
	fi
}

function ksmbd_module_install
{
	echo "Running smbd install"

	local ok=$(lsmod | grep -c smbd)
	if [ "z$ok" == "z1" ]; then
		sudo rmmod smbd
		if [ $? -ne 0 ]; then
			echo "ERROR: unable to rmmod smbd"
			exit 1
		fi
	fi

	ok=$(is_module)
	if [ "z$ok" == "z1" ]; then
		echo "It doesn't look like SMB_SERVER is as a kernel module"
		exit 1
	fi

	if [ ! -f "$KERNEL_SRC"/fs/smbd/smbd.ko ]; then
		echo "ERROR: smbd.ko was not found"
		exit 1
	fi

	cd "$KERNEL_SRC"
	if [ -f "/lib/modules/$(uname -r)/kernel/fs/smbd/smbd.ko*" ]; then
		sudo rm /lib/modules/$(uname -r)/kernel/fs/smbd/smbd.ko*
		sudo cp "$KERNEL_SRC"/fs/smbd/smbd.ko \
			/lib/modules/$(uname -r)/kernel/fs/smbd/smbd.ko

		local VER=$(make kernelrelease)
		sudo depmod -A $VER
	else
		sudo make -C "$KERNEL_SRC" M="$KERNEL_SRC"/fs/smbd/ \
			modules_install
		local VER=$(make kernelrelease)
		sudo depmod -A $VER
	fi
	cd "$KERNEL_SRC"/fs/smbd
}

function ksmbd_module_clean
{
	echo "Running smbd clean"

	cd "$KERNEL_SRC"
	make -C "$KERNEL_SRC" M="$KERNEL_SRC"/fs/smbd/ clean
	cd "$KERNEL_SRC"/fs/smbd
}

function main
{
	patch_fs_config

	COMP_FLAGS="$FLAGS"

	case $1 in
		clean)
			ksmbd_module_clean
			exit 0
			;;
		install)
			ksmbd_module_make
			ksmbd_module_install
			exit 0
			;;
		make)
			ksmbd_module_make
			exit 0
			;;
		help)
			echo "Usage: build_ksmbd.sh [clean | make | install]"
			exit 0
			;;
		*)
			ksmbd_module_make
			exit 0
			;;
	esac
}

main $1
