#!/bin/bash

#git clone -b 5.10 https://github.com/beagleboard/linux --depth=10
#cd ./linux

export CC=/usr/bin/arm-linux-gnueabihf-

make ARCH=arm CROSS_COMPILE=${CC} clean
make ARCH=arm CROSS_COMPILE=${CC} bb.org_defconfig

echo "[make ARCH=arm -j4 CROSS_COMPILE=\"${binary}\" zImage]"
make ARCH=arm -j4 CROSS_COMPILE="ccache ${CC}" zImage
if [ ! -f arch/arm/boot/zImage ] ; then
	echo "failed: [arch/arm/boot/zImage]"
	exit 1
fi

echo "[make ARCH=arm -j4 CROSS_COMPILE=\"${binary}\" modules]"
make ARCH=arm -j4 CROSS_COMPILE="ccache ${CC}" modules
if [ ! -f drivers/spi/spidev.ko ] ; then
	echo "failed: [drivers/spi/spidev.ko]"
	exit 1
fi

echo "[make ARCH=arm CROSS_COMPILE=\"${binary}\" dtbs]"
make ARCH=arm CROSS_COMPILE="ccache ${CC}" dtbs
if [ ! -f arch/arm/boot/dts/am335x-boneblack.dtb ] ; then
	echo "failed: [arch/arm/boot/dts/am335x-boneblack.dtb]"
	exit 1
else
	if [ -f arch/arm/boot/dts/am335x-pocketbeagle.dts ] ; then
		if [ ! -f arch/arm/boot/dts/am335x-pocketbeagle.dtb ] ; then
			echo "failed: [arch/arm/boot/dts/am335x-pocketbeagle.dtb]"
			exit 1
		fi
	fi
fi

make ARCH=arm CROSS_COMPILE=${CC} clean
