#!/bin/bash

#git clone -b 5.10 https://github.com/beagleboard/linux --depth=10
#cd ./linux

export CC=/usr/bin/arm-linux-gnueabihf-

make ARCH=arm CROSS_COMPILE=${CC} clean
make ARCH=arm CROSS_COMPILE=${CC} bb.org_defconfig

echo "make -j4 ARCH=arm KBUILD_DEBARCH=armhf CROSS_COMPILE=${CC} bindeb-pkg"
make -j4 ARCH=arm KBUILD_DEBARCH=armhf KDEB_PKGVERSION=1xross CROSS_COMPILE=${CC} bindeb-pkg
