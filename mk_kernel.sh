#!/bin/bash

config=$(cat .config)

if [[ $config == *"CONFIG_KITRA530=y"* ]]; then
	echo "Building for Kitra530..."
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage -j4
fi

if [[ $config == *"CONFIG_KITRA710C=y"* ]]; then
	echo "Building for KItra710C..."
	make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image -j4
fi

