#!/bin/sh

make ARCH=arm64 rockchip_defconfig android-11.config  rk356x.config  rk3566_xf8_android_test_rochchip_defconfig 
 


make ARCH=arm64 BOOT_IMG=../rockdev/Image-rk3566_r/boot.img  rk3566-rk817-tablet-xf8-test.img -j24
