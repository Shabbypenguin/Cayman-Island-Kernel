#!/bin/bash

# This thread stuff detects the number of cores 
# and actually sets the -j flag for you :)
THREADS=$(expr 1 + $(grep processor /proc/cpuinfo | wc -l))

# This cleans out crud and makes new config
make clean -j$THREADS
make mrproper -j$THREADS
rm -rf ./package/ramdisk_tmp/lib/modules/*.ko
rm -f ./package/zImage
rm -f ./package/ramdisk.gz
make cayman_mpcs_us-perf_defconfig ARCH=arm CROSS_COMPILE=arm-eabi- -j$THREADS

# Edit this to change the kernel name
KBUILD_BUILD_VERSION="Cayman.Island.Kernel.0.1"
export KBUILD_BUILD_VERSION

# Finally making the kernel
make ARCH=arm CROSS_COMPILE=arm-eabi- -j$THREADS


# These move files to easier locations
#find -name '*.ko' -exec cp -av {} ./package/ramdisk_tmp/lib/modules/ \;
cp ./arch/arm/boot/zImage ./package/

# This part packs the img up :)
./package/mkbootfs ./package/ramdisk_tmp | gzip > ./package/ramdisk.gz
./package/mkbootimg --kernel ./package/zImage --ramdisk ./package/ramdisk.gz --cmdline "vmalloc=400M, console=ttyDCC0,115200,n8 androidboot.hardware=qcomloglevel=1" -o ./package/Boot.img --base 0x40200000
