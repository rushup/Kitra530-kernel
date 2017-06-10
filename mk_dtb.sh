#!/bin/sh

make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- dtbs

cp arch/arm/boot/dts/s5p4418-kitra530.dtb arch/arm/boot/dts/s5p4418-artik530-raptor-rev03.dtb
