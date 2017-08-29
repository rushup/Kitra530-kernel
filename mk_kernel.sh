#!/bin/sh

make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage -j4
