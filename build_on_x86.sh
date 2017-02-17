#!/bin/bash

wget https://releases.linaro.org/components/toolchain/binaries/5.4-2017.01/arm-linux-gnueabihf/gcc-linaro-5.4.1-2017.01-x86_64_arm-linux-gnueabihf.tar.xz
tar xf gcc-linaro-5.4.1-2017.01-x86_64_arm-linux-gnueabihf.tar.xz
export CC=`pwd`/gcc-linaro-5.4.1-2017.01-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-

#wget https://releases.linaro.org/components/toolchain/binaries/6.2-2016.11/arm-linux-gnueabihf/gcc-linaro-6.2.1-2016.11-x86_64_arm-linux-gnueabihf.tar.xz
#tar xf gcc-linaro-6.2.1-2016.11-x86_64_arm-linux-gnueabihf.tar.xz
#export CC=`pwd`/gcc-linaro-6.2.1-2016.11-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-

make ARCH=arm bb.org_defconfig
make ARCH=arm -j4 CROSS_COMPILE="ccache ${CC}" zImage
make ARCH=arm -j4 CROSS_COMPILE="ccache ${CC}" dtbs
make ARCH=arm -j4 CROSS_COMPILE="ccache ${CC}" modules
