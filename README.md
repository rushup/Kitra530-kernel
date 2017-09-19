# Linux Kernel for ARTIK
## Contents
1. [Introduction](#1-introduction)
2. [Build guide](#2-build-guide)
3. [Update guide](#3-update-guide)

## 1. Introduction
Fork from 'linux-artik' repository adding Kitra530 and Kitra710C support.

## 2. Build guide
### 2.1 Install cross compiler

#### For Kitra530

```
sudo apt-get install gcc-arm-linux-gnueabihf
```
If you can't install the above toolchain, you can use linaro toolchain.
```
wget http://releases.linaro.org/components/toolchain/binaries/latest-5/arm-linux-gnueabihf/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf.tar.xz
tar xf gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf.tar.xz
export PATH=~/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf/bin:$PATH
```

#### For Kitra710C

You'll need an arm64 cross compiler
```
sudo apt-get install gcc-aarch64-linux-gnu
```
If you can't install the above toolchain, you can use linaro toolchain.
```
wget https://releases.linaro.org/components/toolchain/binaries/5.4-2017.05/aarch64-linux-gnu/gcc-linaro-5.4.1-2017.05-x86_64_aarch64-linux-gnu.tar.xz
tar xf gcc-linaro-5.4.1-2017.05-x86_64_aarch64-linux-gnu.tar.xz
export PATH=~/gcc-linaro-5.4.1-2017.05-x86_64_aarch64-linux-gnu/bin:$PATH
```

You can the path permernently through adding it into ~/.bashrc

### 2.2 Install android-fs-tools
To generate modules.img which contains kernel modules, you can use the make_ext4fs.
```
sudo apt-get install android-tools-fsutils
```

### 2.2 Build the kernel

```
make ARCH=arm kitra530_defconfig
or
make ARCH=arm64 kitra710C_defconfig
```
If you want to change kernel configurations,
```
make ARCH=arm menuconfig #for KItra530
or
make ARCH=arm64 menuconfig #for Kitra710C
```
Run:

```
./mk_kernel.sh
./mk_dtb.sh
./mk_modules.sh
```

## 3. Update Guide
Copy compiled binaries into your board.

```
scp arch/arm/boot/zImage root@{YOUR_BOARD_IP}:/root
scp arch/arm/boot/dts/s5p4418*.dtb root@{YOUR_BOARD_IP}:/root
scp usr/modules.img root@{YOUR_BOARD_IP}:/root
```

#### On KItra530
```
mount -o remount,rw /boot
cp /root/zImage /boot
cp /root/s5p4418*.dtb /boot
dd if=/root/modules.img of=/dev/mmcblk0p5
sync
reboot
```

#### On KItra710C
```
mount -o remount,rw /boot
cp /root/Image /boot
cp /root/*.dtb /boot
dd if=/root/modules.img of=/dev/mmcblk0p5
sync
reboot
```

