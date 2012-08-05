#!/bin/bash
MAKE='make -C kernel ARCH=arm'
rm -rf modules
rm -rf initramfs/lib/modules
mkdir -p initramfs/lib/modules
$MAKE distclean
$MAKE u8500_rev00_janice_open_defconfig
$MAKE zImage
$MAKE modules
$MAKE INSTALL_MOD_PATH="../initramfs" modules_install
#ko=`find modules -type f -name *.ko`;\
#        for i in $ko; do mv $i initramfs/lib/modules/; done;\
#rm -rf modules
$MAKE zImage
cp kernel/arch/arm/boot/zImage ./kernel.bin
md5sum -t kernel.bin >> kernel.bin
mv kernel.bin kernel.bin.md5
