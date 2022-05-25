#!/bin/bash

#git clone -b 5.10-arm64 https://github.com/beagleboard/linux --depth=10
#cd ./linux

export CC=/usr/bin/aarch64-linux-gnu-

make ARCH=arm64 CROSS_COMPILE=${CC} clean
make ARCH=arm64 CROSS_COMPILE=${CC} bb.org_defconfig

echo "make -j4 ARCH=arm64 KBUILD_DEBARCH=arm64 CROSS_COMPILE=${CC} bindeb-pkg"
make -j4 ARCH=arm64 KBUILD_DEBARCH=arm64 KDEB_PKGVERSION=1xross CROSS_COMPILE=${CC} bindeb-pkg
