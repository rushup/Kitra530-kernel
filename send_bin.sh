#!/bin/bash

config=$(cat .config)

if [[ $config == *"CONFIG_KITRA530=y"* ]]; then
	echo "Sending to Kitra530..."
	scp arch/arm/boot/zImage root@$1:/root/
	scp arch/arm/boot/dts/s5p4418*.dtb root@$1:/root/
	scp usr/modules.img root@$1:/root/
fi

if [[ $config == *"CONFIG_KITRA710C=y"* ]]; then
	echo "Sending to KItra710C..."
	scp arch/arm64/boot/Image root@$1:/root
	scp arch/arm64/boot/dts/nexell/*.dtb root@$1:/root
	scp usr/modules.img root@$1:/root
fi

