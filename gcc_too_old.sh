#!/bin/bash

arm-linux-gnueabihf-gcc -v

echo "disabling: CONFIG_THUMB2_KERNEL"
sed -i -e 's:CONFIG_THUMB2_KERNEL=y:# CONFIG_THUMB2_KERNEL is not set:g' .config
sed -i -e 's:CONFIG_THUMB2_AVOID_R_ARM_THM_JUMP11=y::g' .config
sed -i -e 's:CONFIG_ARM_ASM_UNIFIED=y::g' .config
