#!/bin/bash

#gcc:

site="https://releases.linaro.org"

#
#https://releases.linaro.org/components/toolchain/binaries/6.3-2017.05/arm-linux-gnueabihf/gcc-linaro-6.3.1-2017.05-x86_64_arm-linux-gnueabihf.tar.xz
#https://releases.linaro.org/components/toolchain/binaries/6.4-2017.08/arm-linux-gnueabihf/gcc-linaro-6.4.1-2017.08-x86_64_arm-linux-gnueabihf.tar.xz
#https://releases.linaro.org/components/toolchain/binaries/6.4-2017.11/arm-linux-gnueabihf/gcc-linaro-6.4.1-2017.11-x86_64_arm-linux-gnueabihf.tar.xz
#https://releases.linaro.org/components/toolchain/binaries/6.4-2018.05/arm-linux-gnueabihf/gcc-linaro-6.4.1-2018.05-x86_64_arm-linux-gnueabihf.tar.xz
#https://releases.linaro.org/components/toolchain/binaries/6.5-2018.12/arm-linux-gnueabihf/gcc-linaro-6.5.0-2018.12-x86_64_arm-linux-gnueabihf.tar.xz
#

gcc_version="6.5"
gcc_minor=".0"
release="18.12"
target="arm-linux-gnueabihf"

version="components/toolchain/binaries/${gcc_version}-20${release}/${target}"
filename="gcc-linaro-${gcc_version}${gcc_minor}-20${release}-x86_64_${target}.tar.xz"
directory="gcc-linaro-${gcc_version}${gcc_minor}-20${release}-x86_64_${target}"

datestamp="${gcc_version}-20${release}-${target}"

binary="bin/${target}-"
#

if [ ! -d ${directory}/ ] ; then
	rm -rf ./gcc-* || true
	#wget -c ${site}/${version}/${filename}
	wget -c http://gfnd.rcn-ee.org/farm/jenkins-dl/${filename}
	tar xf ${filename}
fi

export CC=`pwd`/${directory}/bin/arm-linux-gnueabihf-

make ARCH=arm clean
make ARCH=arm bb.org_defconfig

echo "[make ARCH=arm -j2 CROSS_COMPILE=\"${binary}\" zImage]"
make ARCH=arm -j2 CROSS_COMPILE="ccache ${CC}" zImage
if [ ! -f arch/arm/boot/zImage ] ; then
	echo "failed: [arch/arm/boot/zImage]"
	exit 1
fi

echo "[make ARCH=arm -j2 CROSS_COMPILE=\"${binary}\" modules]"
make ARCH=arm -j2 CROSS_COMPILE="ccache ${CC}" modules
if [ ! -f drivers/spi/spidev.ko ] ; then
	echo "failed: [drivers/spi/spidev.ko]"
	exit 1
fi

echo "[make ARCH=arm -j2 CROSS_COMPILE=\"${binary}\" dtbs]"
make ARCH=arm -j2 CROSS_COMPILE="ccache ${CC}" dtbs
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
