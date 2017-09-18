#!/bin/bash

config=$(cat .config)

if [[ $config == *"CONFIG_KITRA530=y"* ]]; then
	echo "Building for KItra530..."
	mkdir usr/modules
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules -j4
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules_install INSTALL_MOD_PATH=usr/modules INSTALL_MOD_STRIP=1
	make_ext4fs -b 4096 -L modules \
	    -l 32M usr/modules.img \
	    usr/modules/lib/modules/
	rm -rf usr/modules
fi

if [[ $config == *"CONFIG_KITRA710C=y"* ]]; then
	echo "Building for KItra710C..."
	mkdir usr/modules
	make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules -j4
	make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules_install INSTALL_MOD_PATH=usr/modules INSTALL_MOD_STRIP=1
	make_ext4fs -b 4096 -L modules \
		-l 32M usr/modules.img \
		usr/modules/lib/modules/
	rm -rf usr/modules
fi

