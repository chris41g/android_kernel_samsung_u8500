#!/bin/bash
export STERICCSON_WLAN_BUILT_IN=y

MAKE='make -C kernel ARCH=arm'
$MAKE distclean
$MAKE u8500_rev00_janice_open_defconfig
$MAKE modules
$MAKE INSTALL_MOD_PATH="../modules" modules_install
ko=`find modules -type f -name *.ko`;\
        for i in $ko; do mv $i initramfs/lib/modules/; done;\
rm -rf modules
$MAKE zImage
cp kernel/arch/arm/boot/zImage ./
