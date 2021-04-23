#!/bin/bash

#gcc:

#
#https://developer.arm.com/-/media/Files/downloads/gnu-a/8.2-2018.08/gcc-arm-8.2-2018.08-x86_64-arm-linux-gnueabihf.tar.xz
#https://developer.arm.com/-/media/Files/downloads/gnu-a/8.2-2018.11/gcc-arm-8.2-2018.11-x86_64-arm-linux-gnueabihf.tar.xz
#https://developer.arm.com/-/media/Files/downloads/gnu-a/8.2-2019.01/gcc-arm-8.2-2019.01-x86_64-arm-linux-gnueabihf.tar.xz
#https://developer.arm.com/-/media/Files/downloads/gnu-a/8.3-2019.03/binrel/gcc-arm-8.3-2019.03-x86_64-arm-linux-gnueabihf.tar.x
#

gcc_html_path="https://developer.arm.com/-/media/Files/downloads/gnu-a/8.3-2019.03/binrel/"
gcc_filename_prefix="gcc-arm-8.3-2019.03-x86_64-arm-linux-gnueabihf"
gcc_banner="arm-linux-gnueabihf-gcc (GNU Toolchain for the A-profile Architecture 8.3-2019.03 (arm-rel-8.36)) 8.3.0"
gcc_copyright="2018"
datestamp="2019.03-gcc-arm-linux-gnueabihf"

if [ ! -d ${gcc_filename_prefix}/ ] ; then
	rm -rf ./gcc-* || true
	#wget -c ${site}/${version}/${filename}
	wget -c http://192.168.3.125/jenkins/gcc-arm-8.3-2019.03-x86_64-arm-linux-gnueabihf.tar.x
	tar xf ${gcc_filename_prefix}.tar.xz
fi

export CC=`pwd`/${gcc_filename_prefix}/bin/arm-linux-gnueabihf-

make ARCH=arm clean
make ARCH=arm bb.org_defconfig

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

make ARCH=arm clean
