#!/bin/bash

config=$(cat .config)

if [[ $config == *"CONFIG_KITRA530=y"* ]]; then
	echo "Building for KItra530..."
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- dtbs

	cp arch/arm/boot/dts/s5p4418-kitra530.dtb arch/arm/boot/dts/s5p4418-artik530-raptor-rev03.dtb
fi

if [[ $config == *"CONFIG_KITRA710C=y"* ]]; then
	echo "Building for KItra710C..."
	make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- dtbs

	cp arch/arm64/boot/dts/nexell/s5p6818-kitra710C.dtb arch/arm64/boot/dts/nexell/s5p6818-artik710-raptor-rev03.dtb
fi


