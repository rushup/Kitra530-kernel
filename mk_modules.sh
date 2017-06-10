#/bin/sh

mkdir usr/modules
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules -j4
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules_install INSTALL_MOD_PATH=usr/modules INSTALL_MOD_STRIP=1
make_ext4fs -b 4096 -L modules \
    -l 32M usr/modules.img \
    usr/modules/lib/modules/
rm -rf usr/modules
